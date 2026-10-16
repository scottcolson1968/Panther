// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2014-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */
/*#define _POSIX_SOURCE
#include <asm/signal.h>
#include <linux/semaphore.h>*/
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/ktime.h>		/* ktime_get_ns() for block timestamps */
#include <linux/trace.h>		/* tracing_snapshot() for stall capture */
#include <linux/ratelimit.h>	/* DEFINE_RATELIMIT_STATE for RX-dry log throttle */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/math64.h>
#include "../dummy/xadccountstofloat.h"
#include "../dummy/xsynch.h"
/*
 * The IIO DMAengine buffer combines the generic IIO DMA buffer infrastructure
 * with the DMAengine framework. The generic IIO DMA buffer infrastructure is
 * used to manage the buffer memory and implement the IIO buffer operations
 * while the DMAengine framework is used to perform the DMA transfers. Combined
 * this results in a device independent fully functional DMA buffer
 * implementation that can be used by device drivers for peripherals which are
 * connected to a DMA controller which has a DMAengine driver implementation.
 */

bool doSync=false;
bool setTriggerBlock=false;  // set next block
volatile int bWaitForADCTrigger=0;
bool bResetOLCount=false;

unsigned int g_blockCount=0; // since last sync
unsigned int g_sampling_freq = 262144; // ADC sampling rate, updated by sync_buf in ad7768.c
/*
 * Acquisition-in-progress flag. Set by sync_buf in ad7768.c on the FIRST sync IRQ
 * after the IIO RX buffer is enabled (which is when an acquisition truly starts).
 * Subsequent sync IRQs during the same run are spurious — sync_buf checks this flag
 * and skips the g_blockCount=0 reset so an accidental sync pulse mid-run doesn't
 * restart block counting. Cleared in iio_dmaengine_buffer_disable when the input
 * stream's buffer is disabled, so the next acquisition's first sync IRQ is again
 * treated as legitimate.
 */
bool g_acquiring = false;
EXPORT_SYMBOL_GPL(g_acquiring);
static u32 *g_dac_dma_vaddr    = NULL; // vaddr of current DAC DMA buffer
static u32  g_dac_dma_nwords   = 0;    // words in that buffer
static u32  g_dac_dma_bus_addr = 0;    // bus address of that buffer
static u32  g_dac_current_word = 0;    // sample captured at abort time
u32 currentTacho=0;
/* channel-indexed helpers from adccountstofloat.c (8-ch consolidated core) */
extern int Get_OLValid(int chan);
extern unsigned int Get_gainChanged(int chan);
extern unsigned int Get_iOverloadCount(int chan);
extern u32 axi_dmac_current_src_addr(struct dma_chan *c);
extern u32 readCurrentTacho(void);
extern XSynch *g_pSynch;
struct dmaengine_buffer {
	struct iio_dma_buffer_queue queue;

	struct dma_chan *chan;
	struct list_head active;

	size_t align;
	size_t max_size;
};

//static void sig_stop(int signum) { bWaitForADCTrigger=0 }

static struct dmaengine_buffer *iio_buffer_to_dmaengine_buffer(
		struct iio_buffer *buffer)
{
	return container_of(buffer, struct dmaengine_buffer, queue.buffer);
}
bool bStopOLReset=false;
bool bSynchError=0;
unsigned lastOverloadCount[]={0,0,0,0,0,0,0,0,0}; 
unsigned OverloadCount[]={0,0,0,0,0,0,0,0,0};
unsigned GainChangedCount[]={0,0,0,0,0,0,0,0,0}; 
unsigned lastGainChangedCount[]={0,0,0,0,0,0,0,0,0}; 
//int g_chanValid[8];
int g_numChans=0;
/*void *iOverloadCount_iomap[8];
void *OLValid_iomap[8];
void *reset_iomap[8];*/
volatile int loopcounts=0;
int g_cola_enabled = 0;
EXPORT_SYMBOL_GPL(g_cola_enabled);
static volatile bool g_cola_at_barrier = false;
static volatile bool g_dac_at_barrier  = false;
static volatile bool g_dual_sync       = false;
static u64 g_cola_arrive_ns = 0;
static u64 g_dac_arrive_ns  = 0;
static volatile struct dma_chan *g_sync_peer_chan = NULL;
static atomic_t g_sync_issued = ATOMIC_INIT(0);
static volatile bool g_cola_synced = false; /* set by cola at line 415, persists until next cola sentinel or disable */
/* Sentinel rendezvous waits -- sleeping, wall-clock. RV: wait for the peer's
 * sentinel (same ~1 s budget as the old udelay spin). POS1: wait for the peer
 * to issue both channels once both have arrived. */
#define RV_WAIT_NS	(1000ULL * 1000 * 1000)
#define POS1_WAIT_NS	(100ULL * 1000 * 1000)

/*
 * Upper bound on how long the DAC submit path will wait for the ADC RX path to
 * consume the trigger sentinel. Exposed as a module parameter because the
 * useful value depends on how the run is triggered: a self-triggered run
 * clears the flag in milliseconds, whereas a run armed ahead of an external
 * sync edge (multi-board) may legitimately wait for the master's trigger.
 * Settable at runtime via
 *   /sys/module/industrialio_buffer_dmaengine/parameters/adc_trigger_timeout_ms
 * or on the kernel command line.
 */
static unsigned int adc_trigger_timeout_ms = 3000;
module_param(adc_trigger_timeout_ms, uint, 0644);
MODULE_PARM_DESC(adc_trigger_timeout_ms,
	"ms the DAC waits for the ADC trigger sentinel before giving up (0 = wait forever, unsafe)");

/*
 * Which event the ADC sentinel stamp (0xffffffff written into an RX block) is
 * referenced to. The client uses the position of that stamp to measure the
 * DAC-to-ADC sample offset, and adapts to whatever it measures -- so if the
 * reference event is wrong the client silently adopts a wrong offset with
 * nothing reporting a fault. Hence this must match how the DAC is started.
 *
 *   0 = LEGACY (default). Stamp is armed in the RX path when the DAC DMA is
 *       issued (the doSync / bWaitForADCTrigger rendezvous). Correct while DAC
 *       playback begins immediately after the DMA is issued, i.e. whenever the
 *       FPGA DacStartGate is absent or bypassed.
 *
 *   1 = GATE. Stamp is armed by sync_buf() in ad7768.c on the hardware sync
 *       edge. Required when the DacStartGate is engaged (axi_gpio_dacgate
 *       bypass=0), because then DAC playback starts on the sync_n edge rather
 *       than at DMA-issue time, and those can be arbitrarily far apart -- the
 *       whole point of the gate is that boards arm and then wait for a master
 *       trigger. Referencing the stamp to the same sync edge that releases the
 *       DAC makes the measured offset the true pipeline latency, and makes it
 *       identical across boards.
 *
 * MUST be switched in step with the FPGA bypass bit. gate mode with bypass=1,
 * or legacy mode with bypass=0, both give a wrong offset.
 */
/*
 * Word index at which the DAC sentinel is written a SECOND time, so it
 * survives DacStartGate discarding its prefill.
 *
 * Must be comfortably ABOVE the gate's PREFILL_DEPTH, not merely equal to it:
 * the number of samples the gate swallows is NOT the fixed 16 it prefills, it
 * varies run to run (measured 15..17), so an echo at exactly 16 is itself
 * discarded about one run in six. Measured latch rate over 6 runs each:
 * index 16 -> 5/6, index 24 -> 6/6, index 32 -> 6/6. 32 gives 2x margin.
 *
 * That same variance is the capture-start jitter: however many samples the
 * gate eats is how far the DAC start shifts, so the echo onset moves by a few
 * samples run to run. Fixing the gate would remove both. 0 disables this.
 */
static int dac_sentinel_echo = 32;
module_param(dac_sentinel_echo, int, 0644);
MODULE_PARM_DESC(dac_sentinel_echo,
	"word index for the repeated DAC sentinel (>= DacStartGate PREFILL_DEPTH; 0 = off)");

int dac_gate_mode = 0;EXPORT_SYMBOL_GPL(dac_gate_mode);
module_param(dac_gate_mode, int, 0644);

/*
 * Verbose buffer tracing. OFF by default -- this fires on every IIO buffer
 * enable/disable, and a busy client already emits hundreds of lines a second.
 * Turn on only while debugging:
 *   echo 1 > /sys/module/industrialio_buffer_dmaengine/parameters/dacgate_debug
 *
 * Why it exists: the gate only logs when the *ad5791* buffer is enabled, so a
 * client that opens no DAC buffer at all is indistinguishable from one that
 * never got as far as trying. Logging every buffer with its device name and
 * direction makes "which devices did the client actually open?" a one-line
 * answer instead of an elimination exercise across two boards.
 */
static int dacgate_debug = 0;
module_param(dacgate_debug, int, 0644);
MODULE_PARM_DESC(dacgate_debug,
	"1 = log every IIO buffer enable/disable with device name and direction");
MODULE_PARM_DESC(dac_gate_mode,
	"0=legacy: ADC sentinel stamp armed at DAC DMA issue; 1=gate: armed by the hardware sync edge (use when DacStartGate bypass=0)");

#include <linux/io.h>
#define DRIVESCALE_BASE_ADDR       0x40000000UL
#define DRIVESCALE_SYNC_RESET_OFFSET 0x68
#define DRIVESCALE_ALIGN_OFFSET_OFFSET 0x70
#define DRIVESCALE_COLA_ENABLE_OFFSET  0x80
#define DRIVESCALE_GROUP_DELAY_OFFSET  0x88
static void __iomem *g_drivescale_base = NULL;

/*
 * DacStartGate control, via axi_gpio_dacgate in the PL.
 *   ch1 GPIO_DATA  (+0x00) bit0 = arm, bit1 = bypass
 *   ch2 GPIO2_DATA (+0x08) = {sync_count[15:0], started, prefilled, armed}
 *
 * bypass is deliberately NOT written here. It resets to 1 (transparent) from
 * the IP's C_DOUT_DEFAULT and is meant to be cleared by hand during bring-up,
 * so the driver can never engage the gate as a side effect. We read it, mask it
 * back in on every write, and only ever touch the arm bit.
 */
#define DACGATE_BASE_ADDR   0x400E0000UL
#define DACGATE_CH1_DATA    0x00
#define DACGATE_CH2_DATA    0x08
#define DACGATE_ARM_BIT     BIT(0)
#define DACGATE_BYPASS_BIT  BIT(1)
static void __iomem *g_dacgate_base = NULL;

/*
 * Set or clear the gate's arm bit, preserving bypass. No-op unless
 * dac_gate_mode is on, so legacy behaviour never touches the PL.
 *
 * Arming is safe to do early: DacStartGate counts real fifo_generator_1 writes
 * (dma_wr_valid), not the free-running sample strobe, so if the DAC DMA is not
 * streaming yet the FSM simply waits in PREFILL until data actually arrives.
 */
void dacgate_set_arm(bool on)
{
	u32 v;

	if (!dac_gate_mode)
		return;
	if (!g_dacgate_base) {
		g_dacgate_base = ioremap(DACGATE_BASE_ADDR, 0x1000);
		if (!g_dacgate_base) {
			printk(KERN_ERR "dacgate: ioremap failed\n");
			return;
		}
	}
	v = ioread32(g_dacgate_base + DACGATE_CH1_DATA);
	v = on ? (v | DACGATE_ARM_BIT) : (v & ~DACGATE_ARM_BIT);
	iowrite32(v, g_dacgate_base + DACGATE_CH1_DATA);
}

EXPORT_SYMBOL_GPL(dacgate_set_arm);

/* Status readback for logging; 0 if the gate is not mapped. */
u32 dacgate_status(void)
{
	if (!dac_gate_mode || !g_dacgate_base)
		return 0;
	return ioread32(g_dacgate_base + DACGATE_CH2_DATA);
}

EXPORT_SYMBOL_GPL(dacgate_status);

/*
 * True only for the main DAC's output buffer.
 *
 * Must NOT match cola: cola stalls in lockstep for free because
 * fifo_generator_3/rd_en is driven by fifo_generator_1/valid, which the gate
 * already holds off. Arming on cola would also arm the gate for runs that never
 * touch the main DAC at all.
 *
 * Matching on the IIO device name is deliberate -- at buffer-enable time there
 * is no data yet, so the 0xffffffff / 0xeeeeeeee sentinel used everywhere else
 * to tell DAC from cola is not available.
 */
static bool dacgate_is_dac_buffer(struct iio_buffer *buffer,
				  struct iio_dev *indio_dev)
{
	return buffer->direction == IIO_BUFFER_DIRECTION_OUT &&
	       indio_dev && indio_dev->name &&
	       !strcmp(indio_dev->name, "ad5791");
}

u64 start_timestamp=0;
u64 delta_timestamp = 0;

/*
 * Stall detection threshold.
 * Normal inter-block delta at 262144 sps / 4096 samples per block = 15,625,000 ns.
 * Threshold is set to 2x normal (two missed buffers) to avoid false positives
 * from normal jitter while reliably catching any genuine stall.
 *   2 x 15,625,000 = 31,250,000 ns
 */
#define STALL_THRESHOLD_NS	31250000ULL

/*
 * Dynamic stall threshold. Default = STALL_THRESHOLD_NS. Recomputed at the
 * first block_done of each acquisition from the current g_sampling_freq
 * (updated by sync_buf in ad7768.c) and the IIO buffer length.
 */
static u64 g_stall_threshold_ns = STALL_THRESHOLD_NS;

/*
 * Stall counter — tracks how many stall events have fired since boot.
 * Used to rate-limit snapshot captures (one per stall event, not per block).
 */
static unsigned int g_stallCount = 0;

/*
 * Flag set in block_done ISR to request a snapshot.
 * Consumed in submit_block which runs in process context where
 * tracing_snapshot() is safe to call.
 */
static bool g_stallSnapshotPending   = false;
static bool g_txStallSnapshotPending = false;
/* Shared rate limit for "RX DMA buffer ran dry in {done,submit}" — at most one
 * message per second across both call sites, so a sustained stall doesn't flood
 * dmesg. Use ratelimit_set_flags() at runtime if you want different behavior. */
static DEFINE_RATELIMIT_STATE(rx_dry_rs, HZ, 1);
/* Same idea for the TX side: at most one "TX DMA buffer ran dry in submit"
 * message per second so a sustained TX stall doesn't flood dmesg. */
static DEFINE_RATELIMIT_STATE(tx_dry_rs, HZ, 1);
static void iio_dmaengine_buffer_block_done(void *data,
		const struct dmaengine_result *result)
{
	struct iio_dma_buffer_block *block = data;
	struct iio_dma_buffer_queue *queue = block->queue;
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(&queue->buffer);
	unsigned long flags;
	bool pipeline_empty = false;

	/*
	 * Capture timestamp immediately on DMA completion — before any other
	 * processing — to accurately reflect when the transfer finished.
	 *
	 * ktime_get_ns() returns nanoseconds since boot (monotonic clock).
	 * The 64-bit value is packed into Extra section 2 alongside g_blockCount:
	 *
	 *   Extra 2 layout (8 channels = 8 x u32 words):
	 *     word 0: g_blockCount
	 *     word 1: timestamp low  32 bits  (ts_lo)
	 *     word 2: timestamp high 32 bits  (ts_hi)
	 *     words 3-7: reserved zero
	 *
	 * Host reconstruction:  timestamp_ns = ((u64)ts_hi << 32) | ts_lo
	 * Inter-block delta:    delta_ns = timestamp_ns - prev_timestamp_ns
	 *
	 * No buffer size change required on the host side — fits within the
	 * existing Extra 2 allocation of numChans * 4 bytes.
	 */
	u64 block_timestamp_ns = ktime_get_ns();
	
	spin_lock_irqsave(&block->queue->list_lock, flags);
	list_del(&block->head);
	spin_unlock_irqrestore(&block->queue->list_lock, flags);
	//block->block.bytes_used -= result->residue;
	
	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		int block_size = block->block.size;
		int extraBytes = block->block.size%1024; // frame size is multiple of 1024, remainder is the overload count
		//int chans = block->block.size/1024;
		u32* dataPt = block->vaddr;
		int numChans=g_numChans+1,i;  // figure out disable drive loopback later
		
		unsigned currentOverloadCount[8];
		unsigned currentGainChangeCount[8];
		pipeline_empty = list_empty(&dmaengine_buffer->active) ||
				 list_is_singular(&dmaengine_buffer->active);
	//	block->block.bytes_used = block->block.size; // update for extra overload data point
	//	printk(KERN_INFO "receiving block input used %d size %d vaddr 0x%x",block->block.bytes_used,block->block.size,dataPt);
		dataPt = block->vaddr+block->block.bytes_used;
//		printk(KERN_INFO "testing1\n");
#if 0
		if (bStopOLReset) { cannot be done in an isr
			bStopOLReset=false;
			for (i=0; i<8; i++) 
				XAdccountstofloat_Set_reset(&Adccountstofloat[i],0);
		}
		if (bResetOLCount) {
			for (i=0; i<8; i++) {
				XAdccountstofloat_Set_reset(&Adccountstofloat[i],1);
				lastOverloadCount[i]=0;
			}
			bResetOLCount=false;
			bStopOLReset=true;
		
		}
		
		for (i=0; i<numChans; i++) {
		if (XAdccountstofloat_Get_OLValid(&Adccountstofloat[i] && extraBytes>0) {
			currentOverloadCount[i]=XAdccountstofloat_Get_iOverloadCount(&Adccountstofloat[i]);
			/*bGainChange[i] = currentOverloadCount[i] & 0x80000000;
			currentOverloadCount[i] &= 0x7fffffff;*/
		//	XAdccountstofloat_Set_iOverloadCount_i(&Adccountstofloat[i],0);
			OverloadCount[g_numChans] = currentOverloadCount[i];//(currentOverloadCount[i]-lastOverloadCount[i];
			g_numChans++;
			if (currentOverloadCount[i] & 0xffff0000)
				printk(KERN_INFO "got current gain change\n");
			if (OverloadCount[g_numChans] & 0xffff0000)
				printk(KERN_INFO "got gain change\n");
	//		writel(reset_iomap[i],1);
		}
#endif
		if (extraBytes>0) 	{	// at least 1 extra byte per channel (overload count)
			extraBytes = extraBytes-(numChans*4);
			for (i=0; i<numChans-1; i++) 			
				*dataPt++=OverloadCount[i];	
			*dataPt++=0; // no overload for drive loopback			
		}
		if (extraBytes>0) 	{
			extraBytes = extraBytes-(numChans*4);
			*dataPt++=g_blockCount;			/* word 0: block count      */
			if (g_blockCount==0) {
				delta_timestamp = 0;
				start_timestamp = block_timestamp_ns;
			}
			else delta_timestamp = block_timestamp_ns - start_timestamp;

			/*
			 * Stall detection — fires at the FIRST block_done after a
			 * stall ends, which is exactly when the ring buffer contains
			 * the most useful pre-stall history.
			 *
			 * trace_printk() injects a timestamped marker directly into
			 * the ftrace ring buffer — perfectly correlated with the
			 * existing function trace history. Zero overhead when tracing
			 * is off (compiled to a no-op by the compiler).
			 *
			 * tracing_snapshot() freezes the ring buffer at this moment,
			 * preserving everything leading up to and including the gap.
			 * Protected by g_stallCount to capture once per event rather
			 * than once per block for the duration of the stall.
			 */
			/* Recompute threshold from current sampling_freq and IIO buffer length.
			 * Only on the first block of each acquisition (g_blockCount==0). */
			if (!g_blockCount && g_sampling_freq > 0 && queue->buffer.length > 0)
				g_stall_threshold_ns = div64_u64(
					2ULL * queue->buffer.length * 1000000000ULL,
					g_sampling_freq);

			if (g_blockCount > 1 &&
			    (delta_timestamp > g_stall_threshold_ns || pipeline_empty)) {
				g_stallCount++;
				trace_printk("DMAC STALL #%u: delta_ns=%llu threshold_ns=%llu "
				             "exhausted=%d block=%u samples_per_chan=%u\n",
				             g_stallCount,
				             delta_timestamp,
				             g_stall_threshold_ns,
				             (int)pipeline_empty,
				             g_blockCount,
				             block->block.bytes_used / (9 * 4));
				/* tracing_snapshot() is not safe in IRQ context —
				 * set flag for submit_block to handle in process context */
				g_stallSnapshotPending = true;
			}
			*dataPt++=(u32)(delta_timestamp & 0xFFFFFFFF); /* word 1: ts_lo */
			*dataPt++=(u32)(delta_timestamp >> 32);        /* word 2: ts_hi */
			for (i=3; i<numChans; i++) {		/* words 3-7: reserved zero */
				*dataPt++=0;		
			}
			start_timestamp = block_timestamp_ns; // for next buffer
		}

		if (extraBytes>0) 	{
		//	extraBytes = extraBytes-(numChans*4);
			*dataPt++=currentTacho;		
			for (i=1; i<numChans; i++) {
				*dataPt++ = 0;	
			}
		}
		
		g_blockCount++;
	//	u32 temp2;

		u32* temp = block->vaddr;
		if (setTriggerBlock) { // input block
		//	printk(KERN_INFO "working on trigger, reset block ID to 0");
			*temp = 0xffffffff; //set synch value on input data
//			printk(KERN_INFO "sync val 0x%x used %d address 0x%x vaddress 0x%x chansize %d\n",*temp,block->block.bytes_used,block->phys_addr,block->vaddr,numChans);
			setTriggerBlock=false;
			doSync=0;
		//	g_blockCount=0;	
		}

		if (doSync && bWaitForADCTrigger) {
			bWaitForADCTrigger=0;
		//	raise(SIGTERM);
			printk(KERN_INFO "ADC working on sync loops=%d waitForADC:%d",loopcounts,bWaitForADCTrigger);	


			doSync=false;
		//	bResetOLCount=true;
			/*
			 * Arm the ADC sentinel stamp for the next RX block -- but only in
			 * legacy mode. In gate mode the DAC does not start here (it starts
			 * on the hardware sync edge), so stamping here would reference the
			 * client's offset measurement to the wrong event. sync_buf() in
			 * ad7768.c arms it instead. See dac_gate_mode above.
			 *
			 * bWaitForADCTrigger is still cleared in both modes so the TX
			 * submit path is released either way.
			 */
			if (!dac_gate_mode)
				setTriggerBlock=true;  // set next block
		//	udelay(1);
		/*	for (i=0; i<8; i++) 
				XAdccountstofloat_Set_reset(&Adccountstofloat[i],1); this hangs the system (cannot be done in isr)*/
			if (!((g_blockCount)%1000))
				printk(KERN_INFO "ADC ALIVE loops:%d waitForADC:%d address0x%x",g_blockCount,bWaitForADCTrigger,&bWaitForADCTrigger);
		}
		if (bSynchError==1) {
			*temp = 0xeeeeeeee; //set synch error on input data
			bSynchError=0;
		}
	}
	block->block.bytes_used = block->block.size; // update for extra overload data point

	/*
	 * If the DMA pipeline ran dry on an input buffer (active list has ≤1
	 * entry left), re-prime it immediately by resubmitting the block back
	 * to hardware rather than delivering it to the client. The client
	 * would simply enqueue_block() it again, but the round-trip through
	 * userspace takes long enough to starve the FIFO. Calling
	 * dmaengine_prep/submit directly is safe here because block_done
	 * runs in softirq (DMA completion) context where DMA engine ops are
	 * permitted.
	 */
	if (pipeline_empty && queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		int extraBytes = block->block.size % 1024;
		size_t bytes = block->block.size - extraBytes;
		struct dma_async_tx_descriptor *rdesc;
		dma_cookie_t rcookie;
		if (__ratelimit(&rx_dry_rs))
			printk(KERN_INFO "RX DMA buffer ran dry in done");
		bytes = min_t(size_t, bytes, dmaengine_buffer->max_size);
		bytes = round_down(bytes, dmaengine_buffer->align);
		/* same scan alignment as submit (rounddown: scan size not pow2) */
		if (extraBytes > 0 && queue->buffer.bytes_per_datum > 0)
			bytes = rounddown(bytes, queue->buffer.bytes_per_datum);
		if (bytes > 0) {
			rdesc = dmaengine_prep_slave_single(dmaengine_buffer->chan,
				block->phys_addr, bytes, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT);
			if (rdesc) {
				block->block.bytes_used = bytes;
				rdesc->callback_result = iio_dmaengine_buffer_block_done;
				rdesc->callback_param = block;
				spin_lock_irqsave(&dmaengine_buffer->queue.list_lock, flags);
				list_add_tail(&block->head, &dmaengine_buffer->active);
				spin_unlock_irqrestore(&dmaengine_buffer->queue.list_lock, flags);
				rcookie = dmaengine_submit(rdesc);
				if (!dma_submit_error(rcookie)) {
					dma_async_issue_pending(dmaengine_buffer->chan);
					return; /* block re-owned by hardware */
				}
				/* submit failed — remove from active and fall through */
				spin_lock_irqsave(&dmaengine_buffer->queue.list_lock, flags);
				list_del(&block->head);
				spin_unlock_irqrestore(&dmaengine_buffer->queue.list_lock, flags);
			}
		}
		/* Fallback: deliver to client if resubmit failed */
	}

	/* TX (DAC/cola) pipeline-empty detection — flag for snapshot in submit context */
	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_OUT) {
		bool tx_empty = list_empty(&dmaengine_buffer->active) ||
				list_is_singular(&dmaengine_buffer->active);
		if (tx_empty)
			g_txStallSnapshotPending = true;
	}
	iio_dma_buffer_block_done(block);
	
}

static bool doOnce=true;
int printnum=0;
int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct dmaengine_buffer *dmaengine_buffer;
	enum dma_transfer_direction direction;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int numChans=0,i;
	unsigned j=0,msecs;
	bool bPrintMsg=false;
	allow_signal(SIGKILL);
	
	dmaengine_buffer = iio_buffer_to_dmaengine_buffer(&block->queue->buffer);

	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		/*
		 * Consume any pending stall snapshot request from block_done.
		 * tracing_snapshot() is safe here — submit_block runs in process
		 * context via the IIO DMA workqueue, not in IRQ context.
		 * Called before any other work so the ring buffer reflects the
		 * state as close to the stall moment as possible.
		 */
		if (g_stallSnapshotPending) {
			if (__ratelimit(&rx_dry_rs))
				printk(KERN_INFO "RX DMA buffer ran dry in submit");
			g_stallSnapshotPending = false;
			tracing_snapshot();
		}

		int extraBytes = block->block.size%1024; // frame size is multiple of 1024, remainder is the overload count
		unsigned currentOverloadCount[8]; 
		unsigned currentGainChangedCount[8];
		direction = DMA_DEV_TO_MEM;
//		block->block.bytes_used = block->block.size;
		block->block.bytes_used = block->block.size-extraBytes;
		/* Align the data region to a whole scan: if (size - extraBytes) is
		 * not a multiple of the scan size, the metadata words written at
		 * bytes_used in block_done overwrite the tail of the last data scan
		 * (lands mid-scan -> fs/blocksize comb on the affected channels).
		 * No-op for scan-aligned clients (e.g. the 1025-scan geometry).
		 * NB rounddown(), not round_down() - scan size is not a power of 2. */
		if (extraBytes > 0 && queue->buffer.bytes_per_datum > 0)
			block->block.bytes_used = rounddown(block->block.bytes_used,
						queue->buffer.bytes_per_datum);
		{
			static u32 last_logged_size;
			if (block->block.size != last_logged_size) {
				last_logged_size = block->block.size;
				printk(KERN_INFO "RX client buffer: %u bytes = %u scans of %u B, %u metadata bytes, data region %u bytes\n",
					block->block.size,
					queue->buffer.bytes_per_datum ?
						block->block.size / queue->buffer.bytes_per_datum : 0,
					queue->buffer.bytes_per_datum,
					extraBytes, (u32)block->block.bytes_used);
			}
		}
/*		if (bStopOLReset) { 
			bStopOLReset=false;
			for (i=0; i<8; i++) 
				XAdccountstofloat_Set_reset(&Adccountstofloat[i],0);
		}
		if (bResetOLCount) {
			for (i=0; i<8; i++) {
				XAdccountstofloat_Set_reset_i(&Adccountstofloat[i],1);
				lastOverloadCount[i]=0;
			}
			bResetOLCount=false;
	//		bStopOLReset=true;
		
		}	*/
		//g_numChans=0;
		currentTacho=readCurrentTacho();
#if 1	
		for (i=0; i<8; i++) {	
			if (Get_OLValid(i) && extraBytes) {
				currentGainChangedCount[i] = Get_gainChanged(i);
				GainChangedCount[numChans] = currentGainChangedCount[i]-lastGainChangedCount[i];
				lastGainChangedCount[i]=currentGainChangedCount[i];
				GainChangedCount[numChans] = GainChangedCount[numChans] << 16;
				currentOverloadCount[i]=Get_iOverloadCount(i);
				OverloadCount[numChans] = currentOverloadCount[i]-lastOverloadCount[i];
				OverloadCount[numChans] = OverloadCount[numChans] | GainChangedCount[numChans];
				lastOverloadCount[i]=currentOverloadCount[i];
			/*	if (j=GainChangedCount[g_numChans])
					printk(KERN_INFO "got chan %d current gain change 0x%x\n",i,j);
				if (j=OverloadCount[g_numChans])
					printk(KERN_INFO "got chan %d overload/gain change 0x%x\n",i,j);*/
				numChans++;
	//			writel(reset_iomap[i],1);
	//			XAdccountstofloat_Set_reset_i(&Adccountstofloat[i],1);  reset won't return to 0 in HLS for some reason
			}
		}
		g_numChans=numChans;
		
		if(!(printnum%100000)) {
//			u32 samples_per_chan = block->block.bytes_used / (9 * 4);
//			u64 expected_delta_ns = div_u64((u64)samples_per_chan * 1000000000ULL, 262144);
//			printk(KERN_INFO "samples_per_chan=%u expected_delta_ns=%llu actual_delta_ns=%llu bytes_used=%d\n",
//			samples_per_chan, expected_delta_ns, delta_timestamp,block->block.bytes_used);
			//printk(KERN_INFO "current delta_time between buffers 0x%llx", delta_timestamp);
		}
		printnum++;
#endif
	} else {
		direction = DMA_MEM_TO_DEV;
		/* Consume any pending TX pipeline-empty snapshot request */
		if (g_txStallSnapshotPending) {
			/*
			 * Context, not just the fact. A bare "ran dry" says nothing
			 * about how starved the DAC actually is -- whether the client
			 * is marginally behind or has stopped feeding entirely. The
			 * gap since the previous TX submit distinguishes those.
			 */
			static u64 last_tx_submit_ns;
			u64 now_ns = ktime_get_ns();
			if (__ratelimit(&tx_dry_rs))
				printk(KERN_INFO "TX DMA buffer ran dry in submit (gap since last TX submit %llu ms, dac_gate_mode=%d)",
				       last_tx_submit_ns ? div_u64(now_ns - last_tx_submit_ns, 1000000) : 0,
				       dac_gate_mode);
			last_tx_submit_ns = now_ns;
			g_txStallSnapshotPending = false;
			tracing_snapshot();
		}
		u32* data = block->vaddr;

		bool is_cola = (*data == 0xeeeeeeee);
		bool is_dac  = (*data == 0xffffffff);

		/*
		 * The gate throws away its prefill, so a sentinel in word 0 never
		 * reaches DriveScale and s_dac_ready never latches -- which keeps
		 * the DAC, the cola and the loopback outputs at zero forever.
		 * Repeat it past the prefill, where it does arrive. Word 0 is left
		 * intact for the rendezvous below. Costs one sample, and that
		 * sample is the first one actually played (0..15 are discarded).
		 *
		 * COLA ONLY. DriveScale detects and zeroes the sentinel inside its
		 * `if (bColaEnabled)` branch, so with cola off 0xffffffff is never
		 * converted and would flow on as a float NaN. The ready-gate it
		 * feeds lives in that same branch, so there is nothing to gain.
		 */
		if (is_dac && dac_gate_mode && g_cola_enabled && dac_sentinel_echo > 0 &&
		    block->block.bytes_used > (dac_sentinel_echo + 1) * sizeof(u32))
			data[dac_sentinel_echo] = 0xffffffff;

		if ((is_cola && g_cola_enabled) || is_dac) {
			bool dual = false;	/* this thread's rendezvous result */
			u64 rv_deadline;
			/* Fix 2: compute bytes_used and prepare descriptor BEFORE spinning */
			block->block.bytes_used = min_t(size_t, block->block.bytes_used,
							dmaengine_buffer->max_size);
			block->block.bytes_used = round_down(block->block.bytes_used,
							     dmaengine_buffer->align);
			if (block->block.bytes_used == 0) {
				iio_dma_buffer_block_done(block);
				return 0;
			}
			desc = dmaengine_prep_slave_single(dmaengine_buffer->chan,
				block->phys_addr, block->block.bytes_used,
				direction, DMA_PREP_INTERRUPT);
			if (!desc) {
				printk(KERN_INFO "sync ENOMEM dmaengine");
				return -ENOMEM;
			}
			desc->callback_result = iio_dmaengine_buffer_block_done;
			desc->callback_param = block;

			if (!is_cola) {
				g_dac_dma_vaddr    = (u32 *)block->vaddr;
				g_dac_dma_nwords   = block->block.bytes_used / sizeof(u32);
				g_dac_dma_bus_addr = (u32)block->phys_addr;
			}

			/*
			 * Cola/DAC sentinel rendezvous -- rewritten 2026-09-10 after three
			 * races were caught on hardware (kernel #614 journal):
			 *  1. Each side cleared its own barrier flag the moment it passed,
			 *     so a peer spinning on that flag could miss it: cola logged
			 *     "dac_wait done: g_dac=1" yet took the single-channel path.
			 *  2. Both threads wrote the shared g_dual_sync, so one side could
			 *     take the dual path while the other took the single path ->
			 *     "pos1 timeout: peer never arrived".
			 *  3. That pos1 timeout never issued its OWN channel, and both
			 *     waits were 1 s udelay() spins on the client's SCHED_FIFO TX
			 *     threads -- the pattern that once starved the cola thread.
			 * Now: an arrival flag stays up until the rendezvous completes or
			 * its owner gives up; the dual decision is this thread's local
			 * `dual` (g_dual_sync is kept for logging only); waits sleep
			 * against a wall-clock deadline; a pos1 timeout issues its own
			 * channel.
			 */
			if (is_cola) {
				g_cola_at_barrier = true;
				g_cola_synced = false;
				g_cola_arrive_ns = ktime_get_ns();
				printk(KERN_INFO "cola sentinel: g_dac_at_barrier=%d", g_dac_at_barrier);
				if (g_dac_at_barrier) {
					doSync = true;
					g_cola_synced = true;
					dual = true;
				} else {
					rv_deadline = ktime_get_ns() + RV_WAIT_NS;
					while (!g_dac_at_barrier && ktime_get_ns() < rv_deadline)
						usleep_range(10, 20);
					dual = g_dac_at_barrier;
					printk(KERN_INFO "cola dac_wait done: g_dac=%d", dual);
				}
			} else {
				g_dac_at_barrier = true;
				g_cola_synced = false;
				g_dac_arrive_ns = ktime_get_ns();
				printk(KERN_INFO "DAC sentinel: g_cola_at_barrier=%d g_cola_enabled=%d", g_cola_at_barrier, g_cola_enabled);
				doSync = true;
				if (!g_cola_enabled) {
					dual = false;
				} else if (g_cola_at_barrier) {
					dual = true;
				} else {
					rv_deadline = ktime_get_ns() + RV_WAIT_NS;
					while (!g_cola_at_barrier && !g_cola_synced &&
						   ktime_get_ns() < rv_deadline)
						usleep_range(10, 20);
					dual = g_cola_at_barrier || g_cola_synced;
					printk(KERN_INFO "DAC cola_wait done: g_cola=%d", dual);
				}
			}
			if (!dual) {
				/* Going alone: drop our flag so a late peer cannot pair
				 * with a run this side has already started by itself. */
				if (is_cola) g_cola_at_barrier = false;
				else         g_dac_at_barrier  = false;
			}
			g_dual_sync = dual;    /* logging only -- control uses `dual` */

			/* Reset HLS sync state on any sentinel so s_dac_ready/s_cola_ready
			 * are always clean at the start of a new sync attempt. */
			if (dual || is_dac) {
				if (!g_drivescale_base)
					g_drivescale_base = ioremap(DRIVESCALE_BASE_ADDR, 0x10000);
				if (g_drivescale_base) {
					iowrite32(1, g_drivescale_base + DRIVESCALE_SYNC_RESET_OFFSET);
					udelay(200);
					iowrite32(0, g_drivescale_base + DRIVESCALE_SYNC_RESET_OFFSET);
				}
			}

			/*
			 * Legacy mode only. In gate mode this wait is skipped entirely --
			 * see below for why it is not merely redundant but actively
			 * self-defeating there.
			 */
			if (is_dac && !dac_gate_mode) { /* DAC-to-ADC sync only; Cola uses g_sync_issued atomic instead */
				/*
				 * GATE MODE: SKIPPED. This wait sits BEFORE dmaengine_submit()
				 * and dma_async_issue_pending(), so it holds the DAC DMA back
				 * until the ADC confirms it consumed the trigger sentinel.
				 *
				 * With DacStartGate engaged that breaks the very ordering the
				 * gate needs:
				 *   - the DMA is not issued until the ADC is already running,
				 *     so PREFILL cannot complete before the trigger; and
				 *   - in the DAC-before-trigger sequence multi-board requires,
				 *     the ADC is not acquiring yet, so this would burn the full
				 *     adc_trigger_timeout_ms (3 s) on every single run.
				 * Measured on hardware: DAC data reached the kernel 160 ms
				 * AFTER the sync edge, and prefill completed later still.
				 *
				 * It is also unnecessary in gate mode by construction: the
				 * hardware sync edge supplies the start reference, so the DAC
				 * has no reason to ask the ADC when to begin. Skipping it lets
				 * the DMA issue immediately, data flow, and PREFILL complete
				 * while the board waits for the trigger -- which is exactly
				 * the state the gate must be in when the edge arrives.
				 *
				 * Legacy behaviour below is unchanged.
				 *
				 * Wait for the ADC RX path to consume the trigger sentinel and
				 * clear bWaitForADCTrigger.
				 *
				 * This loop used to be unbounded — the escape was commented
				 * out — so if the ADC sync IRQ never fired the kernel spun
				 * here forever inside submit_block, pinning the CPU in the IIO
				 * submit context and blocking the ext4 journal and udev into a
				 * systemd-level wedge after ~60 s.
				 *
				 * usleep_range rather than udelay so the task actually blocks:
				 * the client's TX threads run SCHED_FIFO and equal-priority
				 * FIFO tasks do not preempt each other, so a tight udelay spin
				 * here starves the peer TX thread and prevents the very
				 * rendezvous being waited on.
				 *
				 * The bound is a wall-clock deadline, not an iteration count,
				 * so it stays correct regardless of how long usleep_range
				 * actually sleeps.
				 */
				u64 adc_deadline_ns = adc_trigger_timeout_ms
					? ktime_get_ns() + (u64)adc_trigger_timeout_ms * 1000000ULL
					: 0;

				printk(KERN_INFO "DAC: bWaitForADCTrigger=1 g_dual_sync=%d", g_dual_sync);
				bWaitForADCTrigger = 1;
				loopcounts = 1;
				while (bWaitForADCTrigger == 1) {
					if (adc_deadline_ns && ktime_get_ns() > adc_deadline_ns) {
						bWaitForADCTrigger = 0;
						printk(KERN_ERR
						       "DAC: ADC trigger timeout after %u ms (loops=%d) — releasing DMA, sync degraded for this buffer\n",
						       adc_trigger_timeout_ms, loopcounts - 1);
						break;
					}
					usleep_range(10, 20);
					loopcounts++;
				}
			}


			/* Fix 3: coordinated dma_async_issue_pending */
			spin_lock_irq(&dmaengine_buffer->queue.list_lock);
			list_add_tail(&block->head, &dmaengine_buffer->active);
			spin_unlock_irq(&dmaengine_buffer->queue.list_lock);

			cookie = dmaengine_submit(desc);
			if (dma_submit_error(cookie))
				return dma_submit_error(cookie);

			if (dual) {
				int pos = atomic_inc_return(&g_sync_issued);
				if (pos == 1) {
					/* First: publish our channel, sleep until the peer issues both */
					u64 pos1_deadline = ktime_get_ns() + POS1_WAIT_NS;

					g_sync_peer_chan = dmaengine_buffer->chan;
					smp_wmb();
					while (atomic_read(&g_sync_issued) != 0 &&
						   ktime_get_ns() < pos1_deadline)
						usleep_range(10, 20);
					/*
					 * Still 1 = the peer never took pos 2. Reclaim the slot
					 * atomically (the peer may be taking it right now) and
					 * issue OUR channel: the old code only reset here, which
					 * left this sentinel block submitted but never issued.
					 * If the cmpxchg fails the peer holds pos 2 and issues both.
					 */
					if (atomic_cmpxchg(&g_sync_issued, 1, 0) == 1) {
						printk(KERN_ERR "pos1 timeout: peer never arrived - issuing own channel\n");
						g_sync_peer_chan = NULL;
						dma_async_issue_pending(dmaengine_buffer->chan);
						if (is_cola) g_cola_at_barrier = false;
						else         g_dac_at_barrier  = false;
					}
				} else {
					/* Second: issue both channels back-to-back */
					struct dma_chan *peer;
					smp_rmb();
					peer = (struct dma_chan *)g_sync_peer_chan;
					if (peer)
						dma_async_issue_pending(peer);
					dma_async_issue_pending(dmaengine_buffer->chan);
					g_dual_sync = false;
					g_sync_peer_chan = NULL;
					g_dac_at_barrier = false;
					g_cola_at_barrier = false;
					g_cola_synced = false;
					atomic_set(&g_sync_issued, 0);
					udelay(2000);
					if (g_drivescale_base)
						{ s64 delta_ns = (s64)g_dac_arrive_ns - (s64)g_cola_arrive_ns; printk(KERN_INFO "Dual sync alignOffset=%d delta=%lldms\n", (int)ioread32(g_drivescale_base + DRIVESCALE_ALIGN_OFFSET_OFFSET), div_s64(delta_ns, 1000000)); }
				}
			} else {
				dma_async_issue_pending(dmaengine_buffer->chan);
				udelay(2000);
				if (g_drivescale_base)
					printk(KERN_INFO "%s HLS alignOffset=%d\n", is_cola ? "Cola (no DAC)": "DAC (no Cola)", (int)ioread32(g_drivescale_base + DRIVESCALE_ALIGN_OFFSET_OFFSET));
			}

			/*
			 * NOTE: the gate is NOT armed here any more. Arming used to happen
			 * on this sentinel, which turned out to be wrong twice over:
			 *
			 *  1. Arm lifetime was then bound to a single submit, giving only a
			 *     ~1 s window before the DAC buffer closed and disarmed it. A
			 *     board waiting on a master trigger would silently disarm.
			 *  2. The client fires its sync BEFORE submitting the first DAC
			 *     buffer (measured: 4.3 s before), so arming on the sentinel
			 *     always armed AFTER the sync edge had already gone by.
			 *
			 * Arming now happens in iio_dmaengine_buffer_enable() instead, so
			 * the gate is armed for the whole streaming session.
			 */
			return 0;
		}
	}
	if (bPrintMsg==true) printk(KERN_INFO "DAC sync continue loops=%d msecs=%d",loopcounts,msecs);
	block->block.bytes_used = min_t(size_t, block->block.bytes_used,
					dmaengine_buffer->max_size);
	block->block.bytes_used = round_down(block->block.bytes_used,
					     dmaengine_buffer->align);

	if (direction == DMA_MEM_TO_DEV && block->block.bytes_used > 0) {
		g_dac_dma_vaddr    = (u32 *)block->vaddr;
		g_dac_dma_nwords   = block->block.bytes_used / sizeof(u32);
		g_dac_dma_bus_addr = (u32)block->phys_addr;
	}

	if (block->block.bytes_used == 0) {
		iio_dma_buffer_block_done(block);
		return 0;
	}

	if (block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC) {
		desc = dmaengine_prep_dma_cyclic(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used,
			block->block.bytes_used, direction, 0);
		if (!desc) {
			printk(KERN_INFO "ENOMEM block->block.flags");
			return -ENOMEM;
		}
	} else {
		desc = dmaengine_prep_slave_single(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used, direction,
			DMA_PREP_INTERRUPT);
		if (!desc) {
			printk(KERN_INFO "ENOMEM dmaengine");
			return -ENOMEM;
		}

		desc->callback_result = iio_dmaengine_buffer_block_done;
		desc->callback_param = block;
	}

	spin_lock_irq(&dmaengine_buffer->queue.list_lock);
	list_add_tail(&block->head, &dmaengine_buffer->active);
	spin_unlock_irq(&dmaengine_buffer->queue.list_lock);

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		printk(KERN_INFO "dmaengine_submit");
		return dma_submit_error(cookie);
	}
	dma_async_issue_pending(dmaengine_buffer->chan);
	if (bPrintMsg) printk(KERN_INFO "DAC sync complete");
	return 0;
}

EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_submit_block);
/* Read a word from the DAC DMA output buffer, counted back from the end.
 * Skips trailing IEEE 0.0f (0x00000000) words used as block terminators.
 * offset=0 -> last sample, offset=1 -> second-to-last sample, etc.
 * Must be called before iio_disable_all_buffers() to avoid use-after-free.
 */
u32 iio_dma_read_output_word_from_end(u32 offset)
{
	u32 nwords = g_dac_dma_nwords;

	if (!g_dac_dma_vaddr || nwords == 0)
		return 0;

	/* skip trailing zero words (IEEE 0.0f padding/terminator) */
	while (nwords > 0 && g_dac_dma_vaddr[nwords - 1] == 0)
		nwords--;

	if (nwords <= offset)
		return 0;
	return g_dac_dma_vaddr[nwords - 1 - offset];
}
EXPORT_SYMBOL_GPL(iio_dma_read_output_word_from_end);

/*
 * Stop-time DAC FIR flush.
 *
 * Every DAC run used to start with ~30 samples of the PREVIOUS run. The DAC
 * FIR (F262144_x1, 25 taps, Has_ARESETn=false -- no reset port) keeps its
 * delay line, and the DAC holds its last code when a run stops, so the next
 * run replays that history before its own data reaches the output. Only data
 * can clear it. A client-side zero tail is not practical: the client can have
 * up to 64 blocks queued, and a tail only survives the close if it is >= 3
 * blocks, because the abort discards the 2 in flight.
 *
 * So the driver plays DAC_FLUSH_SAMPLES_MAX zeros itself, right after the abort
 * has stopped the client's stream and BEFORE the gate is disarmed -- the gate
 * must still be passing DMA data (IDLE holds dma_enable low). 0x00000000 is
 * 0.0f volts. The DAC is left at 0 and the FIR clean, so the next run starts
 * from silence.
 *
 * The DAC DMA is paced at the ADC rate (axi_ad5766 dma_valid follows
 * `trigger`), and the interpolation chain runs from that rate, so the flush
 * is counted in INPUT-rate samples. 128 = ~2x the worst front-filter
 * history: taps/L is 36-41 input samples for every F<rate>_x<L> (25 for
 * F262144_x1), plus ~10 for the tail stages. Cost: 0.5 ms at 262144,
 * 125 ms at 1024. The timeout scales with g_sampling_freq the same way.
 *
 * Only runs when the DAC actually played (legacy mode, or gate released);
 * see iio_dmaengine_buffer_disable(). Bounded wait, never a busy-wait.
 * Disable with dac_flush_on_stop=0.
 */
#define DAC_FLUSH_SAMPLES_MAX	256	/* = DriveScale loopDelayBuf depth */

/*
 * Zeros to play on stop: enough to flush the interpolation FIR (the original
 * 128) AND to empty the whole ch9 loopback delay line, so the next capture
 * never replays this run on ch9.
 *
 * It has to be the WHOLE line, not just the delay currently programmed: the
 * next run's delay is not known here, and a run started at a higher rate
 * reads further back than a low-rate stop would have cleared. Measured with
 * the delay-sized version: stop at 1024 (delay 53, flush 128) then start at
 * 262144 SINC5 (delay 183) replayed 16 samples of the old run.
 */
static unsigned int dac_flush_samples(void)
{
	return DAC_FLUSH_SAMPLES_MAX;
}

static unsigned int dac_flush_on_stop = 1;
module_param(dac_flush_on_stop, uint, 0644);
MODULE_PARM_DESC(dac_flush_on_stop,
	"1 = play a short block of zeros when the DAC buffer is disabled, so the FIR and DAC start the next run clean");

static u32 *g_dac_flush_vaddr;
static dma_addr_t g_dac_flush_phys;
/*
 * The DAC buffer whose stop owes a flush. A pointer, not a flag: the DAC and
 * cola buffers are often disabled at the same moment (two writers closing
 * together), and abort runs for every OUT buffer. With a global bool, cola's
 * abort could take the flush and play the zeros on the cola channel (which
 * then timed out and left cola shifted), and cola's disable could clear it
 * before the DAC's abort ran (the DAC skipped its flush silently, leaving a
 * stale ch9 delay line). Found by the regression suite, 2026-10-15.
 */
static struct dmaengine_buffer *g_dac_flush_owner;

static void iio_dmaengine_dac_flush_done(void *arg)
{
	complete(arg);
}

static void iio_dmaengine_dac_flush(struct dmaengine_buffer *dmaengine_buffer)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct dma_chan *chan = dmaengine_buffer->chan;
	unsigned int nsamp = dac_flush_samples();
	size_t bytes = nsamp * sizeof(u32);
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	u64 t0 = ktime_get_ns();
	/* 2x the nominal play time at the current ADC rate, plus 20 ms */
	unsigned int rate = g_sampling_freq >= 1024 ? g_sampling_freq : 1024;
	unsigned int tmo_ms = nsamp * 2000 / rate + 20;

	if (!g_dac_flush_vaddr) {
		g_dac_flush_vaddr = dma_alloc_coherent(chan->device->dev,
		                       DAC_FLUSH_SAMPLES_MAX * sizeof(u32),
							   &g_dac_flush_phys, GFP_KERNEL);
		if (!g_dac_flush_vaddr) {
			printk(KERN_WARNING "dac flush: buffer alloc failed - DAC holds its last value\n");
			return;
		}
	}
	memset(g_dac_flush_vaddr, 0, bytes);

	desc = dmaengine_prep_slave_single(chan, g_dac_flush_phys, bytes,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		printk(KERN_WARNING "dac flush: prep failed - DAC holds its last value\n");
		return;
	}
	desc->callback = iio_dmaengine_dac_flush_done;
	desc->callback_param = &done;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		printk(KERN_WARNING "dac flush: submit failed - DAC holds its last value\n");
		return;
	}
	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&done,
					 msecs_to_jiffies(tmo_ms))) {
		/* terminate_sync also guarantees the callback can no longer
		 * touch the on-stack completion */
		dmaengine_terminate_sync(chan);
		printk(KERN_WARNING "dac flush: timed out after %u ms @%u Sa/s - DAC may hold its last value\n",
			   tmo_ms, rate);
		return;
	}
	if (dacgate_debug)
		printk(KERN_INFO "dac flush: %u zero samples @%u Sa/s played in %llu us\n",
			   nsamp, rate, div_u64(ktime_get_ns() - t0, 1000));
}

/* Return the waveform sample that was in flight when the DMA was aborted. */
u32 iio_dma_read_current_output_word(void)
{
	return g_dac_current_word;
}
EXPORT_SYMBOL_GPL(iio_dma_read_current_output_word);

void iio_dmaengine_buffer_abort(struct iio_dma_buffer_queue *queue)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(&queue->buffer);

	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_OUT) {
		/* Capture current DMA position before stopping.
		 * CURRENT_SRC_ADDR holds the bus address being read,
		 * so we can find the exact waveform sample in flight.
		 */
		/*
		 * Snapshot first: this runs for EVERY output buffer, cola included,
		 * while these globals describe the DAC block. Closing the DAC and
		 * cola buffers together had one thread clearing g_dac_dma_vaddr
		 * between the guard and the dereference below -- an oops at
		 * NULL + (nwords-1)*4. Work from local copies so the window is gone.
		 */
		u32 *vaddr_snapshot = READ_ONCE(g_dac_dma_vaddr);
		u32  base_snapshot  = READ_ONCE(g_dac_dma_bus_addr);
		u32  nwords_snapshot = READ_ONCE(g_dac_dma_nwords);

		if (vaddr_snapshot && base_snapshot && nwords_snapshot) {
			u32 cur  = axi_dmac_current_src_addr(dmaengine_buffer->chan);
			u32 off  = cur - base_snapshot;
			u32 idx  = off / sizeof(u32);
			if (idx < nwords_snapshot)
				g_dac_current_word = vaddr_snapshot[idx];
			else
				g_dac_current_word = vaddr_snapshot[nwords_snapshot - 1];
		}
		WRITE_ONCE(g_dac_dma_vaddr, NULL);
	}

	dmaengine_terminate_sync(dmaengine_buffer->chan);
	iio_dma_buffer_block_list_abort(queue, &dmaengine_buffer->active);

	/* Main-DAC buffer disable only: see iio_dmaengine_dac_flush() */
	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_OUT &&
	    cmpxchg(&g_dac_flush_owner, dmaengine_buffer, NULL) == dmaengine_buffer)
		iio_dmaengine_dac_flush(dmaengine_buffer);
	else if (queue->buffer.direction == IIO_BUFFER_DIRECTION_OUT &&
		 READ_ONCE(g_dac_flush_owner))
		printk(KERN_INFO "dac flush: pending during another OUT buffer's abort - left for the DAC\n");
}
EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_abort);

static int iio_dmaengine_buffer_enable(struct iio_buffer *buffer,
				       struct iio_dev *indio_dev)
{
	/* Fresh-run reset for the input stream. Clears any stale g_acquiring
	 * left from boot-time stray sync IRQs (which fire before synch.c has
	 * probed and set g_pSynch, leaving kernel/FPGA bRunningAcq inconsistent).
	 * The first sync_buf after enable is now always treated as the start. */
	int ret;

	if (dacgate_debug)
		printk(KERN_INFO "iio buffer ENABLE : %-10s (%s)\n",
		       (indio_dev && indio_dev->name) ? indio_dev->name : "?",
		       buffer->direction == IIO_BUFFER_DIRECTION_OUT ? "OUT" : "IN");

	if (buffer->direction == IIO_BUFFER_DIRECTION_IN) {
		g_acquiring = false;
		/*
		 * Gate mode: drop a stale start-marker flag. sync_buf() sets
		 * setTriggerBlock on every "acquisition start", and setup pulses
		 * before the first run (filter_type, resync, a stray slave edge) are
		 * taken as one while no RX buffer is open. Only a buffer DISABLE
		 * cleared it, so on the first run after boot the first RX block
		 * could be stamped before the real trigger. The trigger's own
		 * sync_buf() re-arms it. Legacy mode arms it from block_done after
		 * the RX buffer is running, so it is left alone there.
		 */
		if (dac_gate_mode)
			setTriggerBlock = false;
		if (g_pSynch) {
			XSynch_Set_bRunningAcq(g_pSynch, 0);
			/*
			 * Re-arm the Synch start latch for THIS run (2026-09-14).
			 *
			 * The ADC DMA only starts writing once Synch's bStarted is set
			 * (ap_return ANDs packed_fifo_wr_sync). bStarted latches in setup
			 * mode (bRunningAcq=0) on bufStart=1 plus a spiSync edge, and is
			 * cleared only by bufStart=0. synch_probe sets bufStart=1 at boot,
			 * so the first sync edge after boot -- a setup pulse, long before
			 * any run -- latched it. On the FIRST run after boot the client's
			 * "sync enable 1" was therefore a no-op, and the ADC DMA started
			 * at RX enable instead of at the start_pulse. The block count and
			 * sentinel then referenced a block that began BEFORE the edge.
			 * Measured on the two-box rig: the slave ADC ~56 samples off the
			 * master on the first run only. Later runs were right because the
			 * client's en=0 at stop / en=1 at start re-armed the latch.
			 *
			 * bRunningAcq=0 was written just above, so Synch is in setup mode
			 * and honours bufStart. Pulsing it 1->0->1 clears bStarted and
			 * re-arms, so the latch catches the NEXT edge -- the trigger --
			 * exactly as later runs always have. Nothing sets bRunningAcq=1
			 * before that edge (only sync_buf, on the edge), so this cannot
			 * leave the latch waiting forever. Left alone if software set
			 * bufStart=0 deliberately.
			 *
			 * Consequence: this board's RX buffer must be enabled BEFORE the
			 * master's start_pulse -- already required for alignment.
			 */
			if (XSynch_Get_bufStart(g_pSynch)) {
				XSynch_Set_bufStart(g_pSynch, 0);
				udelay(1);    /* >30 adc_clk cycles at 0 */
				XSynch_Set_bufStart(g_pSynch, 1);
				printk(KERN_INFO "synch: start latch re-armed for this run\n");
			}
		}
	}

	ret = iio_dma_buffer_enable(buffer, indio_dev);

	if (dacgate_debug && ret)
		printk(KERN_INFO "iio buffer ENABLE : %s FAILED ret=%d\n",
		       (indio_dev && indio_dev->name) ? indio_dev->name : "?", ret);

	/*
	 * Arm the FPGA DacStartGate for the whole streaming session.
	 *
	 * This is the right hook rather than the DAC sentinel, for two measured
	 * reasons: the sentinel gave only a ~1 s arm window before the buffer
	 * closed again, and the client fires its sync ~4.3 s BEFORE submitting the
	 * first DAC buffer, so a sentinel-armed gate always missed the edge.
	 * Armed here, the gate is ready before any sync can arrive and stays armed
	 * until the DAC buffer is disabled -- which is exactly what a board waiting
	 * on a master trigger needs.
	 *
	 * Arming this early is safe: DacStartGate counts real fifo_generator_1
	 * writes (dma_wr_valid), so with no data flowing yet it simply waits in
	 * PREFILL until the DMA actually starts streaming.
	 *
	 * No-op unless dac_gate_mode=1.
	 */
	if (ret == 0 && dacgate_is_dac_buffer(buffer, indio_dev)) {
		dacgate_set_arm(true);
		if (dac_gate_mode) {
			u32 s = dacgate_status();
			printk(KERN_INFO "dacgate: armed on buffer enable (armed=%u prefilled=%u started=%u sync_count=%u)\n",
			       s & 1, (s >> 1) & 1, (s >> 2) & 1, (s >> 3) & 0xffff);
		}
	}
	return ret;
}

static int iio_dmaengine_buffer_disable(struct iio_buffer *buffer,
					struct iio_dev *indio_dev)
{
	bool is_dac = dacgate_is_dac_buffer(buffer, indio_dev);
	int ret;

	if (dacgate_debug)
		printk(KERN_INFO "iio buffer DISABLE: %-10s (%s)\n",
		       (indio_dev && indio_dev->name) ? indio_dev->name : "?",
		       buffer->direction == IIO_BUFFER_DIRECTION_OUT ? "OUT" : "IN");

	doSync = false;
	bWaitForADCTrigger = 0;
	setTriggerBlock = false;
	g_dac_at_barrier = false;
	g_cola_at_barrier = false;
	g_cola_synced = false;
	g_dual_sync = false;
	atomic_set(&g_sync_issued, 0);
	/*
	 * Disarm the FPGA DacStartGate so the next run re-arms from IDLE. IDLE is
	 * the state that holds dma_enable low and rd_en high, which is what drains
	 * fifo_generator_1 -- staying armed across runs would carry stale samples
	 * into the next PREFILL and shift the release by however many were left.
	 *
	 * Symmetric with the arm in iio_dmaengine_buffer_enable: ONLY the main
	 * DAC's buffer disarms. Disarming on any buffer (as this used to) meant an
	 * ADC-side stop would silently disarm a board that was sitting armed and
	 * waiting for a master trigger -- precisely the multi-board case.
	 *
	 * No-op unless dac_gate_mode=1.
	 */
	/*
	 * ...but the disarm itself now happens at the END of this function, after
	 * iio_dma_buffer_disable(): the stop-time FIR flush runs inside the abort
	 * and needs the gate still passing DMA data. Flush only if the DAC
	 * actually played this run -- legacy mode (no gate), or gate released
	 * (status bit 2 = started). An armed gate that never released has
	 * dma_enable low, so a flush could never complete.
	 */
	if (is_dac && dac_flush_on_stop &&
	    (!dac_gate_mode || (dacgate_status() & 0x4)))
		WRITE_ONCE(g_dac_flush_owner, iio_buffer_to_dmaengine_buffer(buffer));
	/* End-of-run signal: clear g_acquiring when the input (RX/ADC) buffer is
	 * disabled. Next sync IRQ after the next buffer enable starts a new run.
	 * Mirror the off state to the FPGA Synch IP's bRunningAcq register. */
	if (buffer->direction == IIO_BUFFER_DIRECTION_IN) {
		g_acquiring = false;
		if (g_pSynch)
			XSynch_Set_bRunningAcq(g_pSynch, 0);
	}
	ret = iio_dma_buffer_disable(buffer, indio_dev);
	/* only ever clear our own claim; cola must not cancel the DAC's */
	if (is_dac)
		cmpxchg(&g_dac_flush_owner, iio_buffer_to_dmaengine_buffer(buffer), NULL);
	/* Disarm last, so the flush ran with the gate open. IDLE then drains
	 * fifo_generator_1 -- zeros, if we flushed -- out to the DAC. */
	if (is_dac)
		dacgate_set_arm(false);
	return ret;
}

static void iio_dmaengine_buffer_release(struct iio_buffer *buf)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(buf);

	iio_dma_buffer_release(&dmaengine_buffer->queue);
	kfree(dmaengine_buffer);
}

static const struct iio_buffer_access_funcs iio_dmaengine_buffer_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.enable = iio_dmaengine_buffer_enable,
	.disable = iio_dmaengine_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.space_available = iio_dma_buffer_space_available,
	.release = iio_dmaengine_buffer_release,

	.alloc_blocks = iio_dma_buffer_alloc_blocks,
	.free_blocks = iio_dma_buffer_free_blocks,
	.query_block = iio_dma_buffer_query_block,
	.enqueue_block = iio_dma_buffer_enqueue_block,
	.dequeue_block = iio_dma_buffer_dequeue_block,
	.mmap = iio_dma_buffer_mmap,

	.modes = INDIO_BUFFER_HARDWARE,
	.flags = INDIO_BUFFER_FLAG_FIXED_WATERMARK,
};

static const struct iio_dma_buffer_ops iio_dmaengine_default_ops = {
	.submit = iio_dmaengine_buffer_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static ssize_t iio_dmaengine_buffer_get_length_align(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_buffer *buffer = to_iio_dev_attr(attr)->buffer;
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(buffer);

	return sysfs_emit(buf, "%zu\n", dmaengine_buffer->align);
}

static IIO_DEVICE_ATTR(length_align_bytes, 0444,
		       iio_dmaengine_buffer_get_length_align, NULL, 0);

static const struct attribute *iio_dmaengine_buffer_attrs[] = {
	&iio_dev_attr_length_align_bytes.dev_attr.attr,
	NULL,
};

/**
 * iio_dmaengine_buffer_alloc() - Allocate new buffer which uses DMAengine
 * @dev: Parent device for the buffer
 * @channel: DMA channel name, typically "rx".
 *
 * This allocates a new IIO buffer which internally uses the DMAengine framework
 * to perform its transfers. The parent device will be used to request the DMA
 * channel.
 *
 * Once done using the buffer iio_dmaengine_buffer_free() should be used to
 * release it.
 */
static struct iio_buffer *iio_dmaengine_buffer_alloc(struct device *dev,
	const char *channel, const struct iio_dma_buffer_ops *ops,
	void *driver_data)
{
	struct dmaengine_buffer *dmaengine_buffer;
	unsigned int width, src_width, dest_width;
	struct dma_slave_caps caps;
	struct dma_chan *chan;
	int ret;

	dmaengine_buffer = kzalloc(sizeof(*dmaengine_buffer), GFP_KERNEL);
	if (!dmaengine_buffer) {
		printk(KERN_INFO "ERR_PTR(-ENOMEM)");
		return ERR_PTR(-ENOMEM);
	}
	chan = dma_request_chan(dev, channel);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		goto err_free;
	}

	ret = dma_get_slave_caps(chan, &caps);
	if (ret < 0)
		goto err_free;

	/* Needs to be aligned to the maximum of the minimums */
	if (caps.src_addr_widths)
		src_width = __ffs(caps.src_addr_widths);
	else
		src_width = 1;
	if (caps.dst_addr_widths)
		dest_width = __ffs(caps.dst_addr_widths);
	else
		dest_width = 1;
	width = max(src_width, dest_width);

	if (!width) { /* FIXME */
		pr_warn("%s:%d width %d (DMA width >= 256-bits ?)\n",
			__func__,__LINE__, width);
		width = 32;
	}

	INIT_LIST_HEAD(&dmaengine_buffer->active);
	dmaengine_buffer->chan = chan;
	dmaengine_buffer->align = width;
	dmaengine_buffer->max_size = dma_get_max_seg_size(chan->device->dev);

	if (!ops)
		ops = &iio_dmaengine_default_ops;

	iio_dma_buffer_init(&dmaengine_buffer->queue, chan->device->dev, ops,
		driver_data);

	dmaengine_buffer->queue.buffer.attrs = iio_dmaengine_buffer_attrs;
	dmaengine_buffer->queue.buffer.access = &iio_dmaengine_buffer_ops;

	return &dmaengine_buffer->queue.buffer;

err_free:
	printk(KERN_INFO "err_free dma_engine alloc");
	kfree(dmaengine_buffer);
	return ERR_PTR(ret);
}

/**
 * iio_dmaengine_buffer_free() - Free dmaengine buffer
 * @buffer: Buffer to free
 *
 * Frees a buffer previously allocated with iio_dmaengine_buffer_alloc().
 */
static void iio_dmaengine_buffer_free(struct iio_buffer *buffer)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(buffer);

	iio_dma_buffer_exit(&dmaengine_buffer->queue);
	dma_release_channel(dmaengine_buffer->chan);

	iio_buffer_put(buffer);
}

static void __devm_iio_dmaengine_buffer_free(void *buffer)
{
	iio_dmaengine_buffer_free(buffer);
}

/**
 * devm_iio_dmaengine_buffer_alloc() - Resource-managed iio_dmaengine_buffer_alloc()
 * @dev: Parent device for the buffer
 * @channel: DMA channel name, typically "rx".
 *
 * This allocates a new IIO buffer which internally uses the DMAengine framework
 * to perform its transfers. The parent device will be used to request the DMA
 * channel.
 *
 * The buffer will be automatically de-allocated once the device gets destroyed.
 */
struct iio_buffer *devm_iio_dmaengine_buffer_alloc(struct device *dev,
	const char *channel, const struct iio_dma_buffer_ops *ops,
	void *driver_data)
{
	struct iio_buffer *buffer;
	int ret;

	buffer = iio_dmaengine_buffer_alloc(dev, channel, ops, driver_data);
	if (IS_ERR(buffer)) {
		devres_free(buffer);
		return buffer;
	}

	ret = devm_add_action_or_reset(dev, __devm_iio_dmaengine_buffer_free,
				       buffer);
	if (ret)
		return ERR_PTR(ret);

	return buffer;
}
EXPORT_SYMBOL_GPL(devm_iio_dmaengine_buffer_alloc);

/**
 * devm_iio_dmaengine_buffer_setup() - Setup a DMA buffer for an IIO device
 * @dev: Parent device for the buffer
 * @indio_dev: IIO device to which to attach this buffer.
 * @channel: DMA channel name, typically "rx".
 *
 * This allocates a new IIO buffer with devm_iio_dmaengine_buffer_alloc()
 * and attaches it to an IIO device with iio_device_attach_buffer().
 * It also appends the INDIO_BUFFER_HARDWARE mode to the supported modes of the
 * IIO device.
 */
int devm_iio_dmaengine_buffer_setup(struct device *dev,
				    struct iio_dev *indio_dev,
				    const char *channel,
				    enum iio_buffer_direction dir)
{
	struct iio_buffer *buffer;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 channel, NULL, NULL);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;

	buffer->direction = dir;

	return iio_device_attach_buffer(indio_dev, buffer);
}
EXPORT_SYMBOL_GPL(devm_iio_dmaengine_buffer_setup);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA buffer for the IIO framework");
MODULE_LICENSE("GPL");
