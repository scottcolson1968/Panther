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
#include <linux/spinlock.h>
#include <linux/err.h>

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include "../dummy/xadccountstofloat.h" 
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
//sem_t *dac_sem;
/*struct iio_dma_buffer_queue *g_queue;
struct iio_dma_buffer_block *g_block;
struct iio_dma_buffer_queue *g_cola_queue;
struct iio_dma_buffer_block *g_cola_block;*/
unsigned int g_blockCount=0; // since last sync
extern XAdccountstofloat Adccountstofloat[];

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
static void iio_dmaengine_buffer_block_done(void *data,
		const struct dmaengine_result *result)
{
	struct iio_dma_buffer_block *block = data;
	struct iio_dma_buffer_queue *queue = block->queue;
	unsigned long flags;
	//printk(KERN_INFO "block done");
	spin_lock_irqsave(&block->queue->list_lock, flags);
	list_del(&block->head);
	spin_unlock_irqrestore(&block->queue->list_lock, flags);
	//block->block.bytes_used -= result->residue;
	
	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		int extraBytes = block->block.size%1024; // frame size is multiple of 1024, remainder is the overload count
		//int chans = block->block.size/1024;
		u32* dataPt = block->vaddr;
		int numChans=g_numChans+1,i;  // figure out disable drive loopback later
		
		unsigned currentOverloadCount[8];  
		unsigned currentGainChangeCount[8]; 
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
		for (i=0; i<numChans; i++) {
			if (extraBytes>0) 		
				*dataPt++=OverloadCount[i];				
		}

		for (i=0; i<numChans; i++) {
		/*	if (g_blockCount%1000==1 && i==0)
				printk(KERN_INFO "Block ID:%d extra:%d",g_blockCount,extraBytes);*/
			
			if ((extraBytes-(numChans*4))/numChans>0) {			// 2 extras ints per channel	
				*dataPt++=g_blockCount;			
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

		if (doSync) {
			bWaitForADCTrigger=0;
		//	raise(SIGTERM);
			printk(KERN_INFO "ADC working on sync loops=%d waitForADC:%d",loopcounts,bWaitForADCTrigger);	

#if 0
			if( g_queue && g_block) {
				printk(KERN_INFO "submitting dac block\n");
				bWaitForADCTrigger=false;
			
				iio_dmaengine_buffer_submit_block(g_queue, g_block);
			
			//	printk(KERN_INFO "HW send block 0x%x size:%d\n",g_block->phys_addr,g_block->block.bytes_used);
			}
			if(g_cola_queue && g_cola_block) {
				printk(KERN_INFO "submitting cola block\n");
		//		iio_dmaengine_buffer_submit_block(g_cola_queue, g_cola_block);	
			}
			
			g_cola_queue = NULL;
			g_cola_block = NULL;
			g_queue = NULL;
			g_block = NULL;
#endif
			doSync=false;
		//	bResetOLCount=true;
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
	iio_dma_buffer_block_done(block);
	
}
#if 0

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct dmaengine_buffer *dmaengine_buffer;
	enum dma_transfer_direction direction;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	dmaengine_buffer = iio_buffer_to_dmaengine_buffer(&block->queue->buffer);
	
	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		int extraBytes = block->block.size%1024; // frame size is multiple of 1024, remainder is the overload count
		
		direction = DMA_DEV_TO_MEM;
//		block->block.bytes_used = block->block.size;
		block->block.bytes_used = block->block.size-extraBytes;
		printk(KERN_INFO "submitting block input used %d size %d",block->block.bytes_used,block->block.size);
		
	} else {
		direction = DMA_MEM_TO_DEV;
//		printk(KERN_INFO "submitting block output");
		u32* data = block->vaddr;
		int retVal=0;
		
		if (*data == 0xffffffff) {
			//ad7768_sync_data(); 
		//	printk(KERN_INFO "name:%s",queue->dev->init_name);
			g_queue=queue;
			g_block=block;	
			
			doSync=true; 
			*data=0;
			
			printk(KERN_INFO "Do HW submit block 0x%x size:%d val:0x%x\n",block->phys_addr,block->block.bytes_used,*data ); // foir noiw
			bWaitForADCTrigger=1; // for now
			
//			ad7768_sync_data();
			return 0;  // hold off sending this output data til synch
		}	
		else {
			printk(KERN_INFO "dac submit");
			while (bWaitForADCTrigger==1)		{
				msleep_interruptable(1);	
				if(!(DacCounter++%1000))
					printk(KERN_INFO "DAC ALIVE");
			//	yield();
			}
		}
	}

	block->block.bytes_used = min_t(size_t, block->block.bytes_used,
					dmaengine_buffer->max_size);
	block->block.bytes_used = round_down(block->block.bytes_used,
					     dmaengine_buffer->align);

	if (block->block.bytes_used == 0) {
		printk(KERN_INFO "submit bytes used == 0\n");
		iio_dma_buffer_block_done(block);
		return 0;
	}

	if (block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC) {
		desc = dmaengine_prep_dma_cyclic(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used,
			block->block.bytes_used, direction, 0);
		if (!desc)
			return -ENOMEM;
	} else {
		desc = dmaengine_prep_slave_single(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used, direction,
			DMA_PREP_INTERRUPT);
		if (!desc)
			return -ENOMEM;

		desc->callback_result = iio_dmaengine_buffer_block_done;
		desc->callback_param = block;
	} 

	spin_lock_irq(&dmaengine_buffer->queue.list_lock);
	list_add_tail(&block->head, &dmaengine_buffer->active);
	spin_unlock_irq(&dmaengine_buffer->queue.list_lock);

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return dma_submit_error(cookie);

	dma_async_issue_pending(dmaengine_buffer->chan);
//	printk(KERN_INFO "block submitted");
	return 0;
}
#else
static bool doOnce=true;

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct dmaengine_buffer *dmaengine_buffer;
	enum dma_transfer_direction direction;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int numChans=9,i;
	unsigned j=0,msecs;
	bool bPrintMsg=false;
	allow_signal(SIGKILL);
	dmaengine_buffer = iio_buffer_to_dmaengine_buffer(&block->queue->buffer);

	if (queue->buffer.direction == IIO_BUFFER_DIRECTION_IN) {
		int extraBytes = block->block.size%1024; // frame size is multiple of 1024, remainder is the overload count
		unsigned currentOverloadCount[8]; 
		unsigned currentGainChangedCount[8];
		direction = DMA_DEV_TO_MEM;
//		block->block.bytes_used = block->block.size;
		block->block.bytes_used = block->block.size-extraBytes;
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
		g_numChans=0;
#if 1	
		for (i=0; i<8; i++) {	
			if (XAdccountstofloat_Get_OLValid(&Adccountstofloat[i]) && extraBytes) {	
				currentGainChangedCount[i] = XAdccountstofloat_Get_gainChanged(&Adccountstofloat[i]);
				GainChangedCount[g_numChans] = currentGainChangedCount[i]-lastGainChangedCount[i];
				lastGainChangedCount[i]=currentGainChangedCount[i];
				GainChangedCount[g_numChans] = GainChangedCount[g_numChans] << 16;
				currentOverloadCount[i]=XAdccountstofloat_Get_iOverloadCount(&Adccountstofloat[i]);
				OverloadCount[g_numChans] = currentOverloadCount[i]-lastOverloadCount[i];
				OverloadCount[g_numChans] = OverloadCount[g_numChans] | GainChangedCount[g_numChans];
				lastOverloadCount[i]=currentOverloadCount[i];
			/*	if (j=GainChangedCount[g_numChans])
					printk(KERN_INFO "got chan %d current gain change 0x%x\n",i,j);
				if (j=OverloadCount[g_numChans])
					printk(KERN_INFO "got chan %d overload/gain change 0x%x\n",i,j);*/
				g_numChans++;
	//			writel(reset_iomap[i],1);
	//			XAdccountstofloat_Set_reset_i(&Adccountstofloat[i],1);  reset won't return to 0 in HLS for some reason
			}
		}
#endif
	} else {
		direction = DMA_MEM_TO_DEV;
//		printk(KERN_INFO "submitting block output");
		u32* data = block->vaddr;
		int retVal=0;
		
		if (*data == 0xffffffff) {
			//ad7768_sync_data(); 
		//	printk(KERN_INFO "name:%s",queue->dev->init_name);
	/*		g_queue=queue;
			g_block=block;	*/
		//	sigaction(SIGTERM, sig_stop,NULL);
			
			doSync=true; 
			*data=0;
			
			bWaitForADCTrigger=1; // for now
			loopcounts=1;
			printk(KERN_INFO "Do HW submit block 0x%x size:%d val:0x%x waitForADC:%d address0x%x\n",block->phys_addr,block->block.bytes_used,*data ,bWaitForADCTrigger,&bWaitForADCTrigger); 
			while (bWaitForADCTrigger==1)		{
		//		sem_wait(dac_sem);
			//	loopcounts++;
				
		//		msecs=msleep_interruptible(1000);	
				
			//	msleep_interruptible(1);	
				udelay(10);
				if(!(loopcounts++%1000000))   {
					bWaitForADCTrigger=0; // give up
					bSynchError=1;
					printk(KERN_INFO "DAC synch error loops:%d waitForADC:%d address0x%x",loopcounts-1,bWaitForADCTrigger,&bWaitForADCTrigger);
				}
			//	bPrintMsg=true;
			//	yield();
			}
//			ad7768_sync_data();
		//	return 0;  // hold off sending this output data til synch
		}	
	/*	else {
		//	printk(KERN_INFO "dac submit");
			while (bWaitForADCTrigger)		{
				udelay(1);	
				yield();
			}
		}*/
	}
	if (bPrintMsg==true) printk(KERN_INFO "DAC sync continue loops=%d msecs=%d",loopcounts,msecs);
	block->block.bytes_used = min_t(size_t, block->block.bytes_used,
					dmaengine_buffer->max_size);
	block->block.bytes_used = round_down(block->block.bytes_used,
					     dmaengine_buffer->align);

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
#endif
EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_submit_block);

void iio_dmaengine_buffer_abort(struct iio_dma_buffer_queue *queue)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(&queue->buffer);

	dmaengine_terminate_sync(dmaengine_buffer->chan);
	iio_dma_buffer_block_list_abort(queue, &dmaengine_buffer->active);
}
EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_abort);

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
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
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
