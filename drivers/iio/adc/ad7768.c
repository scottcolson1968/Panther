// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7768 Analog to digital converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spi/spi.h> 
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h> 
#include <linux/of_irq.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_adc.h"
#include "../dummy/xsynch.h"

/* AD7768 registers definition */
#define AD7768_CH_MODE				0x01
#define AD7768_POWER_MODE			0x04
#define AD7768_DATA_CONTROL			0x06
#define AD7768_INTERFACE_CFG			0x07

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK		BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)	(((x) & 0x1) << 3)
#define AD7768_CH_MODE_GET_FILTER_TYPE(x)	(((x) >> 3) & 0x1)
#define AD7768_CH_MODE_DEC_RATE_MSK		GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)		(((x) & 0x7) << 0)

/* AD7768_POWER_MODE */
#define AD7768_POWER_MODE_POWER_MODE_MSK	GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)		(((x) & 0x3) << 4)
#define AD7768_POWER_MODE_GET_POWER_MODE(x)	(((x) >> 4) & 0x3)
#define AD7768_POWER_MODE_MCLK_DIV_MSK		GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)	(((x) & 0x3) << 0)

#define AD7768_POWER_MODE_SLEEP_TYPE_MSK		BIT(7)
#define AD7768_POWER_MODE_SLEEP_TYPE_MODE(x)	(((x) & 0x1) << 7)
#define AD7768_POWER_MODE_GET_SLEEP_TYPE(x)	(((x) >> 7) & 0x1)

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK	GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1		0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2		0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK	BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC		BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR	0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(((x) & 0x3) << 0)

#define AD7768_INTERFACE_CFG_CRC_SELECT_MSK	GENMASK(3, 2)
/* only 4 samples CRC calculation support exists */
#define AD7768_INTERFACE_CFG_CRC_SELECT		0x01

#define AD7768_MAX_SAMP_FREQ	262144
#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT	0x01


#define AD7768_CONFIGS_PER_MODE			0x06
#define AD7768_NUM_CONFIGS			0x04
#define AD7768_MAX_RATE				(AD7768_CONFIGS_PER_MODE - 1)
#define AD7768_MIN_RATE				0

extern bool doSync;
struct gpio_descs *incoupling[]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
extern int w1_timeout;
extern void Set_iCurrentRange(int chan, int val);
extern int Get_iCurrentRange(int chan);
extern void Set_pointOffset(int chan, int val); 
extern int Get_pointOffset(int chan);
extern void SetOffset(int chan, int val);
extern int GetOffset(int chan);
extern void SetStep(int chan, int val);
extern int GetStep(int chan);
extern int GetStatus(int chan);
static int gain[]={0,0,0,0,0,0,0,0};

struct gpio_desc *gpio_reset=NULL;

enum ad7768_power_modes {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE=2,
	AD7768_FAST_MODE
};

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	struct clk *mclk;
	unsigned int datalines;
	unsigned int sampling_freq;
	enum ad7768_power_modes power_mode;
	const struct axiadc_chip_info *chip_info;		
	__be16 d16;
};

static int ad7768_read_label(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, char *label)
{
	return sysfs_emit(label, "channel%d\n", chan->channel);
}

enum ad7768_device_ids {
	ID_AD7768,
	ID_AD7768_4
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
};

unsigned int ad7768_mclk_divs[] = {
	[AD7768_LOW_POWER_MODE] = 32,
	[AD7768_MEDIAN_MODE] = 8,
	[AD7768_FAST_MODE] = 4
};

int ad7768_sampling_rates[AD7768_NUM_CONFIGS][AD7768_CONFIGS_PER_MODE] = {
	[AD7768_LOW_POWER_MODE] = { 1024, 2048, 4096, 8192, 16384, 32768 },
	[AD7768_MEDIAN_MODE] = { 4096, 8192, 16384, 32768, 65536, 131072 },
	[AD7768_FAST_MODE] = { 8192, 16384, 32768, 65536, 131072, 262144 }
};

static const unsigned int ad7768_sampl_freq_avail[9] = {
	1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144
};


static const unsigned int ad7768_available_datalines[] = {
	1, 2, 8
};

static const unsigned int ad7768_4_available_datalines[] = {
	1, 4
};
int var=0;
int iOneIntOnly=2;
extern unsigned int g_blockCount;
extern unsigned int g_sampling_freq;
extern bool g_acquiring;
extern XSynch *g_pSynch;
/* Spurious-sync record, defined and exposed by synch.c (SYNC device). */
extern unsigned int g_spurious_run;
extern int g_spurious_first_block;
extern unsigned int g_spurious_total;
extern const char *g_spurious_origin;
/*
 * Both defined in drivers/iio/buffer/industrialio-buffer-dmaengine.c. See the
 * dac_gate_mode comment there for why the ADC sentinel stamp has to follow how
 * the DAC is started.
 */
extern bool setTriggerBlock;
extern int  dac_gate_mode;
extern u32  dacgate_status(void);   /* DacStartGate status, 0 if gate mode off */

/*
 * Set by ad7768_sync() immediately before it drives SPI_SYNC (defined further
 * down; declared here because sync_buf() reads it).
 */
extern u64 g_last_local_sync_ns;

/*
 * Did this sync edge come from OUR own SPI_SYNC write, or from another board on
 * the shared sync line?
 *
 * Multi-box only makes sense with exactly one master driving the line. Deciding
 * that from the logs previously meant cross-correlating two boards' uptime
 * clocks by hand; this answers it in the message itself. The window is generous
 * (20 ms) because the SPI transaction plus the ADC's SYNC_OUT response takes a
 * few ms, and a genuinely external edge is never that close to one of ours.
 */
#define AD7768_SELF_SYNC_WINDOW_NS  20000000ULL
static const char *sync_origin(void)
{
	u64 now = ktime_get_ns();

	if (!g_last_local_sync_ns)
		return "EXTERNAL (we have never issued one)";
	if (now - g_last_local_sync_ns < AD7768_SELF_SYNC_WINDOW_NS)
		return "SELF";
	return "EXTERNAL";
}

#define GPIO 55
static irqreturn_t sync_buf(int irq, void *dev_id,struct pt_regs *regs) {
	struct ad7768_state *st = dev_id;
	var++;
	/*
	 * Only the FIRST sync IRQ after the input buffer is enabled is treated
	 * as the start of a run. Subsequent sync IRQs during the same run are
	 * spurious — log and ignore. g_acquiring is cleared by
	 * iio_dmaengine_buffer_disable in dmaengine.c when the RX buffer is
	 * disabled, so the next acquisition starts fresh.
	 */
	if (!g_acquiring) {
		g_acquiring = true;
		g_blockCount = 0;
		WRITE_ONCE(g_spurious_run, 0);
		WRITE_ONCE(g_spurious_first_block, -1);
		/* Mirror to FPGA Synch IP bRunningAcq register so userspace +
		 * HLS can observe the acquisition-active state. */
		if (g_pSynch)
			XSynch_Set_bRunningAcq(g_pSynch, 1);
		/*
		 * Gate mode: THIS edge is what releases the FPGA DacStartGate, so it
		 * is also the correct reference for the ADC sentinel stamp. Arm the
		 * stamp here; the next RX block in dmaengine.c writes 0xffffffff into
		 * word 0 and the client measures the DAC-to-ADC offset from it. The
		 * same edge also resets util_ad7768_adc_pack, so the ADC scan counter
		 * restarts here too -- both streams share one reference.
		 *
		 * Legacy mode leaves this alone; dmaengine.c arms the stamp at DAC
		 * DMA-issue time instead.
		 */
		if (dac_gate_mode)
			setTriggerBlock = true;
		printk(KERN_INFO "sync interrupt! %d (acquisition start%s) [%s]\n",
		       var, dac_gate_mode ? ", gate mode: sentinel armed" : "",
		       sync_origin());
	} else {
		const char *origin = sync_origin();

		/* Record it for the client (SYNC spurious_* attributes, synch.c). */
		if (!g_spurious_run)
			WRITE_ONCE(g_spurious_first_block, (int)g_blockCount);
		WRITE_ONCE(g_spurious_run, g_spurious_run + 1);
		WRITE_ONCE(g_spurious_total, g_spurious_total + 1);
		WRITE_ONCE(g_spurious_origin, origin);
		/*
		 * Rate-limited: a noisy SYNC_IN/ line once fired this ~900 times
		 * a second and filled the SD card through rsyslog (v30.19). The
		 * count N is the running IRQ total, so the gaps stay visible and
		 * /proc/interrupts (sync_buf) still has the exact figure.
		 */
		printk_ratelimited(KERN_INFO "sync interrupt! %d (spurious — ignored) [%s]\n",
				   var, origin);
	}
	if (st)
		g_sampling_freq = st->sampling_freq;
	return IRQ_HANDLED;
}

static bool ad7768_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad7768_state *ad7768_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if (ad7768_has_axi_adc(&indio_dev->dev)) {
		/* AXI ADC*/
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

char in_buf[80];
/*
 * One TEDS read at a time: each read flips the channel's coupling GPIOs and
 * drives the single shared 1-Wire master, and w1_timeout is global. Two
 * readers (iio_info -u local: while a client is connected) used to interleave.
 */
static DEFINE_MUTEX(teds_lock);
static ssize_t getTedsId_locked(const struct iio_chan_spec *chan, char *TedsData);

static ssize_t getTedsId(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *TedsData) {
	ssize_t ret;

	/* bound check BEFORE indexing: in_voltage8 has this attribute too */
	if (chan->channel < 0 || chan->channel >= ARRAY_SIZE(incoupling) ||
	    !incoupling[chan->channel]) {
		sprintf(TedsData,"no coupling for teds chan %d",chan->channel);
		return 0; // for testing
	}
	mutex_lock(&teds_lock);
	ret = getTedsId_locked(chan, TedsData);
	mutex_unlock(&teds_lock);
	return ret;
}

static ssize_t getTedsId_locked(const struct iio_chan_spec *chan, char *TedsData) {

	struct file *fp;
	char filename[80],IDNum[80];
	unsigned long long offset=0;
	loff_t pos;
	size_t size=4;
	size_t count=64;
	int ret,i,tries=0,tedsok=0;

	unsigned long currentgain, tedsbit=6; // teds and single ended enabled
	w1_timeout=0;
//	 printk (KERN_INFO "read in coupling chan %d\n",chan->channel);
       
        gpiod_get_array_value_cansleep(incoupling[chan->channel]->ndescs, incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain); // save current coupling
        currentgain = currentgain&0x1f;
        gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&tedsbit); // turn on teds bit

//        msleep(1000);
                //        ret=getTedsId(sTedsId);
                //      printk(KERN_INFO "Using %s Teds device\n",sTedsId);
        
	sprintf(filename, "/sys/bus/w1/devices/w1_bus_master1/w1_master_search");
        fp = filp_open(filename,O_RDWR,0);
        /* filp_open returns ERR_PTR, never NULL: kernel_write on it Oopsed */
        if (IS_ERR(fp))
                printk(KERN_INFO "filp_open error %s (%ld)\n",filename,PTR_ERR(fp));
        else {
                pos=0;
		i=1;
		IDNum[0]='1';
		kernel_write(fp,IDNum,i,&pos);
                printk(KERN_INFO "kernel write %d",i);
                filp_close(fp,NULL);
        }

	sprintf(filename, "/sys/bus/w1/devices/w1_bus_master1/w1_master_slaves");
	msleep(100);
	while (tries++<2) {
	msleep(100);
	fp = filp_open(filename,O_RDONLY,0);
	msleep(100);
	for (ret=0;ret<80;ret++)
		IDNum[ret]='\0';
	if (IS_ERR(fp)) {
                printk(KERN_INFO "filp id error %s (%ld)\n",filename,PTR_ERR(fp));
		/* previously fell through to kernel_read/filp_close on the ERR_PTR */
		msleep(100);
		continue;
        }

	{
                pos=0;
		ret = kernel_read(fp, (void *)IDNum, 16, &pos);
		if(ret<15) {
			printk(KERN_INFO "invalid teds data %d %s\n",ret,IDNum);
		//	return 0; //invalid
			IDNum[0]='\0';
		}
		else {
			IDNum[15]='\0';
                	printk(KERN_INFO "Found TEDS sensor %s\n",IDNum);
			ret=0;
			tries=10; // exit loop
			tedsok=1;
		}
        }
        filp_close(fp,NULL);
	msleep(100);
	}

//	w1_timeout=10;
	if (!tedsok) {
		gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain);
		sprintf(TedsData, "\0");
		printk(KERN_INFO "no teds sensor found on chan %d", chan->channel);
		w1_timeout=3600000;
		return 0;
	}
	sprintf(filename, "/sys/bus/w1/devices/w1_bus_master1/%s/rw",IDNum);
        fp = filp_open(filename,O_RDONLY,0);
        if (IS_ERR(fp)) {
		printk(KERN_INFO "filp teds error %s\n",filename);
		gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain); // reset gain value
		w1_timeout=3600000;
		return 0;
	}
	
//	ret=vfs_read(fp,IDNUM,64,&pos);
        for (ret=0;ret<65;ret++)
                TedsData[ret]='\0';
        if (fp==NULL) {
                printk(KERN_INFO "filp_open error %s\n",filename);
                ret=0;
        }
        else {
                pos=0;
		count=64;
                ret = kernel_read(fp, (void *)TedsData, count, &pos);
	/*	for (i=0;i<64;i++)
	                printk(KERN_INFO "Teds[%d]:%x",i,(u8)TedsData[i]);
          */      ret=64;
        }
        filp_close(fp,NULL);
        gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain); // reset gain value
	msleep(100);
 	 sprintf(filename, "/sys/bus/w1/devices/w1_bus_master1/w1_master_remove");
         fp = filp_open(filename,O_WRONLY,0);
        if (IS_ERR(fp)) {
                printk(KERN_INFO "filp delete error %s\n",filename);
                w1_timeout=3600000;
                return ret; // the TEDS data itself was read fine
        }
	pos=0;
//	w1_timeout=1;
	 kernel_write(fp, (void *)IDNum,15,&pos);
	filp_close(fp,NULL); // was never closed: one leaked file per TEDS read
	printk(KERN_INFO "remove IDNum %s from %s gain %d",IDNum,filename,currentgain);
	w1_timeout=3600000;
//	printk(KERN_INFO "user mode ret:%d %s\n",ret,argv);
	return ret;
}

static int ad7768_spi_reg_read(struct ad7768_state *st, int addr,
			       unsigned int *val)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d16,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->d16,
			.len = 2,
		},
	};
	int ret;

	st->d16 = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	*val = be16_to_cpu(st->d16);

	return ret;
}

static int ad7768_spi_reg_write(struct ad7768_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->d16 = cpu_to_be16(((addr & 0x7F) << 8) | val);

	return spi_write(st->spi, &st->d16, sizeof(st->d16));
}

static int ad7768_spi_write_mask(struct ad7768_state *st,
				 unsigned int addr,
				 unsigned long int mask,
				 unsigned int val)
{
	unsigned int regval;
	int ret;

	ret = ad7768_spi_reg_read(st, addr, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return ad7768_spi_reg_write(st, addr, regval);
}

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = ad7768_get_data(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval) {
		ret = ad7768_spi_reg_read(st, reg, readval);
		if (ret < 0)
			goto exit;
		ret = 0;
	} else {
		ret = ad7768_spi_reg_write(st, reg, writeval);
	}
exit:
	mutex_unlock(&st->lock);

	return ret;
}

/*
 * Timestamp of the last SPI_SYNC this board issued itself. sync_buf() compares
 * against it to tell a SELF-generated edge from one arriving on the shared
 * sync line from another board. Without this, telling master from slave in a
 * multi-box rig means cross-correlating two boards' uptime clocks by hand.
 */
u64 g_last_local_sync_ns = 0;
EXPORT_SYMBOL_GPL(g_last_local_sync_ns);

/*
 * `why` identifies the CALLER. Three paths reach here and only one is a
 * deliberate trigger:
 *    start_pulse attr        - the intended trigger
 *    power_mode/sampling_freq- side effect of a sampling-frequency write
 *    filter_type             - side effect of a filter change
 * The last two fire real hardware sync edges during setup. With DacStartGate
 * engaged those can release the gate early if they land after prefill
 * completes, so knowing which is which matters.
 */
static int ad7768_sync(struct ad7768_state *st, int type, const char *why)
{
	int ret;
	//return 0;
	if (type==0) {
		printk(KERN_INFO "start trigger pulse removed, using spi_sync\n");
		type=3;
	}
	else if (type==3) {//SPI_SYNC
		printk(KERN_INFO "issue sync pulse (%s)", why);
		g_last_local_sync_ns = ktime_get_ns();
		ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
		if (ret < 0)
			return ret;
		
		return ad7768_spi_write_mask(st,  AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC);
		printk(KERN_INFO "issue 2nd sync pulse");
		ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
		if (ret < 0)
			return ret;
		
		return ad7768_spi_write_mask(st,  AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC);
	}
	else if (type==2) {//SPI_RESET
		printk(KERN_INFO "issue reset pulse");
		ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_RESET_MSK,
				    AD7768_DATA_CONTROL_SPI_RESET_1);
		if (ret < 0)
			return ret;
	
		return ad7768_spi_write_mask(st,  AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_RESET_MSK,
				    AD7768_DATA_CONTROL_SPI_RESET_2);
	}
//	return 0;

	else if (type==1) {
		printk(KERN_INFO "hard reset pulse",type);
		if (gpio_reset) {
			gpiod_set_value_cansleep(gpio_reset, 1);
			fsleep(2);
			gpiod_set_value_cansleep(gpio_reset, 0);
		//	fsleep(1660);
			printk(KERN_INFO "adc hard reset complete");
			st->sampling_freq=32768;
		}
	}
	 
	printk(KERN_INFO "sync pulse %d\n",2);
	return 0;
}

static int ad7768_set_clk_divs(struct ad7768_state *st,
			       unsigned int mclk_div,
			       unsigned int freq)
{
	unsigned int mclk, dclk_div, dec, div;
	unsigned int result = 0;
	int ret = 0;

	mclk = clk_get_rate(st->mclk);

	for (dclk_div = 0; dclk_div < 4 ; dclk_div++) {
		for (dec = 0; dec < ARRAY_SIZE(ad7768_dec_rate); dec++) {
			div = mclk_div *
			      (1 <<  (3 - dclk_div)) *
			      ad7768_dec_rate[dec];

			result = DIV_ROUND_CLOSEST_ULL(mclk, div);
			if (freq == result)
				break;
		}
	}
	if (freq != result)
		return -EINVAL;

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
			AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
			AD7768_INTERFACE_CFG_DCLK_DIV_MODE(3 - dclk_div));
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st, AD7768_CH_MODE,
				     AD7768_CH_MODE_DEC_RATE_MSK,
				     AD7768_CH_MODE_DEC_RATE_MODE(dec));
}

/*
 * Sampling rate most recently APPLIED by ad7768_set_sampling_freq() -- 0 until
 * the first successful apply after boot.
 *
 * Deliberately NOT st->sampling_freq: probe assigns that field directly without
 * programming the clocks, so comparing a client write against it could skip the
 * very first real configuration after boot. This is only ever set once the full
 * reprogram has succeeded, and is invalidated by a direct power_mode write.
 */
static unsigned int s_applied_freq;

static int ad7768_set_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int regval;
	int ret;

	/* Check if this mode supports the current sampling rate */
	if (st->sampling_freq > ad7768_sampling_rates[mode][AD7768_MAX_RATE] ||
	    st->sampling_freq < ad7768_sampling_rates[mode][AD7768_MIN_RATE])
		return -EINVAL;

	regval = AD7768_POWER_MODE_POWER_MODE(mode);
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_POWER_MODE_MSK,
				    regval);
	if (ret < 0)
		return ret;
	/* The values for the powermode correspond for mclk div */
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_MCLK_DIV_MSK,
				    AD7768_POWER_MODE_MCLK_DIV_MODE(mode));
	if (ret < 0)
		return ret;

	ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[mode],
				st->sampling_freq);
	if (ret < 0)
		return ret;
	regval=0xa;
	ad7768_spi_reg_write(st,0x5,regval); // set 2.5V reference
/*	ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
				    AD7768_CH_MODE_FILTER_TYPE_MSK,
				    AD7768_CH_MODE_FILTER_TYPE_MODE(0));printk (KERN_INFO "set power mode to %d",mode);*/
	/*
	 * Log the power-mode/rate change itself. The original printk above is
	 * inside a comment block, which is why these writes were invisible and
	 * the sync pulse they trigger appeared in dmesg with no explanation.
	 */
	printk(KERN_INFO "set power mode %d (sampling_freq=%u) - will issue sync\n",
	       mode, st->sampling_freq);
	ret = ad7768_sync(st,3,"power_mode/sampling_freq");
	if (ret < 0)
		return ret;
	ret=0;
	st->power_mode = mode;
	/*
	 * A direct power_mode attribute write (chan != NULL -- set_sampling_freq
	 * calls this with NULL) changes the clocks behind set_sampling_freq's back,
	 * so its "unchanged" cache can no longer be trusted. Force the next
	 * sampling_frequency write to re-apply in full.
	 */
	if (chan)
		s_applied_freq = 0;

	return ret;
}

static int ad7768_get_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int regval;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &regval);
	if (ret < 0)
		return ret;

	st->power_mode = AD7768_POWER_MODE_GET_POWER_MODE(regval);

	return st->power_mode;
}

static const char * const ad7768_power_mode_iio_enum[] = {
	[AD7768_LOW_POWER_MODE] = "LOW_POWER_MODE",
	[AD7768_MEDIAN_MODE] = "MEDIAN_MODE",
	[AD7768_FAST_MODE] = "FAST_MODE"
};

static const struct iio_enum ad7768_power_mode_enum = {
	.items = ad7768_power_mode_iio_enum,
	.num_items = ARRAY_SIZE(ad7768_power_mode_iio_enum),
	.set = ad7768_set_power_mode,
	.get = ad7768_get_power_mode,
};
static const char * const ad7768_filter_type_enum[] = {
	"WIDEBAND",
	"SINC5"
};

extern void Set_loopDelay(int val);


int g_loop_delay_trim;            /* client fine-tuning, written from ad5791.c */
EXPORT_SYMBOL_GPL(g_loop_delay_trim);

/*
 * ch9 loopback delay by interpolator path (samp_index 0..8 =
 * 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144).
 *
 * Measured 2026-09-15 on ZedBoard #1: an 8 V DAC step through the gate, then
 * in ONE capture compare where it lands on the analog loopback (capture word
 * 0) against the digital echo (word 8 = DriveScale output, before the
 * interpolation FIRs). Repeatable to +/-1 sample. Confirmed on both ADC
 * filters: the whole difference between them is the ADC filter latency, the
 * same term pointOffset already uses (34 WIDEBAND / 4 SINC5).
 *
 *     delay = interp_delay[samp_index] + filter term + client trim
 */
/*
 * Per-rate ch9 loopback delay, measured on the PRODUCTION analog route:
 * DAC coupling 46 (40 kHz reconstruction filter), ADC inputs DC (coupling 20).
 * THE ROUTE MATTERS: the first version of this table was measured at coupling
 * 14 (1.2 kHz) and was 156 samples out at 262144. The route delta is a fixed
 * ~595 us, so in samples it scales with the rate (156 at 262144 down to 1 at
 * 1024) -- it cannot be corrected with a constant. Re-measure with
 * /root/aligncheck2.sh <rate> <filter> <reps> 46 if the route changes.
 */
static const int ad7768_loop_delay[9] = { 18, 18, 19, 19, 20, 22, 23, 11,  23 };
static int s_loop_samp_index = -1;
static int s_loop_adc_filter = -1;      /* 0 = WIDEBAND, 1 = SINC5 */

/*
 * Per-rate cola delay, written into DriveScale group_delay (0x88).
 * Measured on the PRODUCTION route: DAC coupling 46 (40 kHz), ADC DC.
 * It lives here, not in the HLS, so retuning needs no bitstream -- the
 * HLS colaDelay table was removed for exactly that reason.
 * 65536 and 131072 differ from the rest because IN65536OUT262144 is the
 * only path that also feeds the cola interpolation stream.
 * TIED TO dac_sentinel_echo (currently 32). The cola read index subtracts
 * s_align_ofs = s_dac_invoc - s_cola_invoc, and the kernel delivers the DAC
 * sentinel at word dac_sentinel_echo, so moving that index moves cola by the
 * same number of samples -- measured exactly +16 when it went 16 -> 32, with
 * near-zero variance at 8192. These values are for 32; change one and the
 * other must follow.
 *
 * Re-measure with /root/colaalign.sh and do NOT inject sentinels while
 * doing so: the cola read index subtracts s_align_ofs, so priming the
 * handshake mid-stream shifts the very thing being measured.
 */
/*
 * 2026-10-28, for the cola FIFO drain + prefill bitstream: until then cola
 * started late by however many stale words the previous run left in the cola
 * FIFO, so the old values were medians over a moving target. With the fix the
 * offset repeats exactly, except that DriveScale latches alignOffset once per
 * boot (17 or 18), which moves every rate by one sample for the whole boot.
 * Centred on the common boot (18): shifted +2 at 262144/131072/32768 and +1
 * at 65536 and below (old: -307 -307 -306 -306 -305 -305 -323 -316 -306).
 * Measured with /root/colaalign_route4k.sh (-b 4096; -b 1024 underruns).
 */
static const int ad7768_cola_group_delay[9] = {
	-306, -306, -305, -305, -304, -303, -322, -314, -304
};

int g_group_delay_trim;	/* client trim, written from ad5791.c */
EXPORT_SYMBOL_GPL(g_group_delay_trim);

extern void Set_group_delay(int val);

static int ad7768_cola_route_delay(void);

/*
 * How far below its table value the cola group_delay can go before the read
 * index wraps the 256-entry cola ring (cola then lands ~254 samples late).
 * 65536: wraps at -325 on an alignOffset-18 boot and -324 on a 17 boot, so
 * with the table at -322 the floor of table - 1 = -323 is safe on both.
 * Older measurement (2026-10-28, previous table -323): wrapped at table - 2; 32768 >= 2, 131072 >= 8, 262144 >= 12 were tested
 * clean. 65536 is the only path where the route correction (-2 on the 10 kHz
 * route) or a client trim can reach the edge, so only it is limited here.
 */
static const int ad7768_cola_gd_headroom[9] = {
	1000, 1000, 1000, 1000, 1000, 1000, 1, 1000, 1000
};

static void ad7768_apply_cola_group_delay(void)
{
	int gd, route, floor;

	if (s_loop_samp_index < 0 || s_loop_samp_index > 8)
		return;
	route = ad7768_cola_route_delay();
	gd = ad7768_cola_group_delay[s_loop_samp_index] + route + g_group_delay_trim;
	floor = ad7768_cola_group_delay[s_loop_samp_index] -
		ad7768_cola_gd_headroom[s_loop_samp_index];
	if (gd < floor) {
		printk(KERN_WARNING "cola group_delay %d would wrap the cola ring at path %d, clamped to %d\n",
			   gd, s_loop_samp_index, floor);
		gd = floor;
	}
	Set_group_delay(gd);
	printk(KERN_INFO "cola group_delay %d (path %d, route %+d, trim %d)\n",
		   gd, s_loop_samp_index, route, g_group_delay_trim);
}

void ad7768_cola_group_delay_retrim(void)
{
	ad7768_apply_cola_group_delay();
}
EXPORT_SYMBOL_GPL(ad7768_cola_group_delay_retrim);

static void ad7768_apply_loop_delay(void);

/*
 * Extra ch9 delay for DAC analog routes other than the one the loop-delay
 * table was calibrated on (coupling 46, the 40 kHz reconstruction filter).
 * Values are the measured miss at 262144; the route difference is a fixed
 * TIME, so scale with the rate instead of using a constant.
 * Bank comes from the coupling write: (coupling >> 4) & 0x3.
 *   bank 0 = coupling 14 = 1.2 kHz, 1 = 30 = 10 kHz,
 *   bank 2 = 46 = 40 kHz (calibration route), 3 = 62 = CAL/short.
 */
static const int ad7768_route_delay_262144[4] = { 156, 11, 0, -6 };
static int s_dac_route_bank = 2;	/* 40 kHz until the client says otherwise */

static int ad7768_route_delay(void)
{
	int base, num;

	if (s_dac_route_bank < 0 || s_dac_route_bank > 3 ||
	    s_loop_samp_index < 0 || s_loop_samp_index > 8)
		return 0;
	base = ad7768_route_delay_262144[s_dac_route_bank];
	if (!base)
		return 0;
	/* base is quoted at samp_index 8; halve per step down, rounded */
	num = base << s_loop_samp_index;
	return num >= 0 ? (num + 128) / 256 : -(((-num) + 128) / 256);
}

/*
 * Cola group_delay correction for DAC routes other than the one the cola
 * table was calibrated on (coupling 46). It is NOT the negative of the
 * loop_delay route term: the route setting moves the cola output too, by a
 * different amount on the 10 kHz route (measured: DAC +9, cola +15 there).
 * Values are group_delay units at 262144 (+ = cola later), measured
 * 2026-10-28 with /root/colaalign_route.sh, 11 runs per route, cola - DAC
 * relative to the 40 kHz route: 1.2 kHz -156, 10 kHz +5.8, CAL +6.6.
 * A fixed time, so scaled with the rate like ad7768_route_delay();
 * confirmed at 32768 (-18.75 / +1.25 / +1 measured).
 * Bank 0 deliberately uses the 10 kHz value (-6), not its measured -156:
 * the 1.2 kHz filter is reported broken, so that number is not trusted.
 * Pending a measurement on a production unit.
 */
static const int ad7768_cola_route_delay_262144[4] = { -6, -6, 0, -7 };

static int ad7768_cola_route_delay(void)
{
	int base, num;

	if (s_dac_route_bank < 0 || s_dac_route_bank > 3 ||
	    s_loop_samp_index < 0 || s_loop_samp_index > 8)
		return 0;
	base = ad7768_cola_route_delay_262144[s_dac_route_bank];
	if (!base)
		return 0;
	num = base << s_loop_samp_index;
	return num >= 0 ? (num + 128) / 256 : -(((-num) + 128) / 256);
}

void ad7768_set_dac_route(int bank)
{
	if (bank < 0 || bank > 3 || bank == s_dac_route_bank)
		return;
	s_dac_route_bank = bank;
	ad7768_apply_loop_delay();
}
EXPORT_SYMBOL_GPL(ad7768_set_dac_route);

static void ad7768_apply_loop_delay(void)
{
	int d;

	if (s_loop_samp_index < 0 || s_loop_samp_index > 8 || s_loop_adc_filter < 0)
		return;
	d = ad7768_loop_delay[s_loop_samp_index]
		+ (s_loop_adc_filter ? 4 : 34)
		+ ad7768_route_delay()
		+ g_loop_delay_trim;
	Set_loopDelay(d);
	printk(KERN_INFO "loop_delay %d (path %d, %s, route bank %d %+d, trim %d)\n", d,
		   s_loop_samp_index, s_loop_adc_filter ? "SINC5" : "WIDEBAND",
		   s_dac_route_bank, ad7768_route_delay(), g_loop_delay_trim);
	ad7768_apply_cola_group_delay();
}

/* Called from ad5791.c when the client writes the trim. */
void ad7768_loop_delay_retrim(void)
{
	ad7768_apply_loop_delay();
}
EXPORT_SYMBOL_GPL(ad7768_loop_delay_retrim);

static int ad7768_set_filter_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int filter)
{
//	return 0;
	struct ad7768_state *st = ad7768_get_data(dev);
	int ret;
	if (filter)  // SINC5
		Set_pointOffset(chan->channel,4); // there's an internal buffer of 4 points in SINC5 filter
	else Set_pointOffset(chan->channel,34); // there's an internal buffer of 34 points in WIDEBAND filter
	s_loop_adc_filter = filter ? 1 : 0;
	ad7768_apply_loop_delay();
	/*
	 * Unchanged filter: skip the SPI write AND the sync pulse, for the same
	 * reason as an unchanged sampling_frequency -- the pulse is a real sync_n
	 * edge and would release an armed, not-yet-prefilled DacStartGate.
	 * Set_pointOffset above still runs, preserving the existing behaviour of a
	 * filter_type write resetting the point offset. If the read fails, fall
	 * through to the normal write + pulse rather than guess.
	 */
	{
		unsigned int cur;

		if (ad7768_spi_reg_read(st, AD7768_CH_MODE, &cur) >= 0 &&
		    AD7768_CH_MODE_GET_FILTER_TYPE(cur) == filter) {
			printk(KERN_INFO "filter_type %u unchanged - no sync pulse\n",
			       filter);
			return 0;
		}
	}
	ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
				    AD7768_CH_MODE_FILTER_TYPE_MSK,
				    AD7768_CH_MODE_FILTER_TYPE_MODE(filter));
	if (ret < 0)
		return ret;
//	printk(KERN_INFO "set filter to %d",filter);
	return ad7768_sync(st,3,"filter_type");
}

static int ad7768_get_filter_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int filter;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_CH_MODE, &filter);
	if (ret < 0)
		return ret;

	return AD7768_CH_MODE_GET_FILTER_TYPE(filter);
} 

static const struct iio_enum ad7768_filter_type_iio_enum = {
	.items = ad7768_filter_type_enum,
	.num_items = ARRAY_SIZE(ad7768_filter_type_enum),
	.set = ad7768_set_filter_type,
	.get = ad7768_get_filter_type
};

static const char * const ad7768_sleep_type_enum[] = {
	"AWAKE",
	"SLEEP"
};

static int ad7768_set_sleep_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int type) 
{
//	return 0;
	struct ad7768_state *st = ad7768_get_data(dev);
	int ret;

	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_SLEEP_TYPE_MSK,
				    AD7768_POWER_MODE_SLEEP_TYPE_MODE(type));
//	if (ret < 0)
		return ret;
	
}

static int ad7768_get_sleep_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int type;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &type);
	if (ret < 0)
		return ret;

	return AD7768_POWER_MODE_GET_SLEEP_TYPE(type);
}

static const struct iio_enum ad7768_sleep_type_iio_enum = {
	.items = ad7768_sleep_type_enum,
	.num_items = ARRAY_SIZE(ad7768_sleep_type_enum),
	.set = ad7768_set_sleep_type,
	.get = ad7768_get_sleep_type
};

static const char * const ad7768_start_pulse_enum[] = {
	"TRIGGER",
	"RESET",
	"SPI_RESET",
	"SPI_SYNC"
};

static int ad7768_set_start_pulse(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int type) 
{
//	return 0;
	struct ad7768_state *st = ad7768_get_data(dev);
	printk(KERN_INFO "start pulse %d",type);
	return ad7768_sync(st,type,"start_pulse attr");
	
}

static const struct iio_enum ad7768_start_pulse_iio_enum = {
	.items = ad7768_start_pulse_enum,
	.num_items = ARRAY_SIZE(ad7768_start_pulse_enum),
	.set = ad7768_set_start_pulse
};
extern void SetSampIndex(int val);
static int ad7768_set_sampling_freq(struct iio_dev *dev,
				    unsigned int freq)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	int power_mode = -1;
	unsigned int i, j;
	int ret = 0;
	int samp_index=0;
	if (!freq)
		return -EINVAL;
	for (i=0;i<9;i++) {
		if (freq==ad7768_sampl_freq_avail[i])
			samp_index = i;	/* latch AFTER the clock reconfig (see below) */
	}

	mutex_lock(&st->lock);

	for (i = 0; i < AD7768_NUM_CONFIGS; i++) {
		for (j = 0; j < AD7768_CONFIGS_PER_MODE; j++) {
			if (freq == ad7768_sampling_rates[i][j]) {
				power_mode = i;
				break;
			}
		}
	}

	if (power_mode == -1) {
		ret = -EINVAL;
		goto freq_err;
	}

	if (freq == s_applied_freq) {
		/*
		 * Unchanged rate: nothing to reprogram, so issue NO sync pulse.
		 *
		 * Every sampling_frequency write used to reprogram the clocks and
		 * fire ad7768_sync() even when the value had not changed. That pulse
		 * is a real edge on sync_n. With the DacStartGate armed (DAC buffer
		 * open) and REQUIRE_PREFILL=0, the gate releases on the first edge
		 * after arm -- so a redundant rate write released it BEFORE the DAC
		 * data arrived, and the run started unaligned (prefilled=0).
		 * Captured twice with the new client, 2026-09-10: a repeated 262144
		 * write 250 ms before the first WRITEBUF, and a repeated 131072 write
		 * issued just after the DAC buffer had been recreated.
		 *
		 * SetSampIndex still runs, so streamToData's held path select is
		 * guaranteed correct even when this is the first write after boot.
		 *
		 * This removes redundant pulses at their SOURCE. It cannot stop a
		 * slave from seeing the master's genuine rate-change pulse on the
		 * shared sync line -- that still needs the client to finish its rate
		 * writes before creating the DAC buffers.
		 */
		printk(KERN_INFO "sampling_freq %u unchanged - no reprogram, no sync pulse\n",
		       freq);
		SetSampIndex(samp_index);
		s_loop_samp_index = samp_index;
		ad7768_apply_loop_delay();
		goto freq_err;          /* ret == 0: unlock and return success */
	}

	ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[power_mode], freq);
	if (ret < 0)
		goto freq_err;

	st->sampling_freq = freq;

	ret = ad7768_set_power_mode(dev, NULL, power_mode);
	if (ret < 0)
		goto freq_err;

	/* Latch the streamToData interpolation index AFTER the ADC clock has been
	 * reconfigured.  Doing it before (the old order) latched the interpolator
	 * phase against the previous clock on a live rate change, leaving the
	 * DAC->ADC loopback ~12 samples misaligned until fs was rewritten. */
	SetSampIndex(samp_index);
	s_loop_samp_index = samp_index;
	ad7768_apply_loop_delay();
	s_applied_freq = freq;   /* only after the full reprogram succeeded */

freq_err:
	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t ad7768_attr_sampl_freq_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7768_state *st;
	int i, len = 0, size;

	st = ad7768_get_data(indio_dev);
	size = ARRAY_SIZE(ad7768_sampl_freq_avail);

	if (st->datalines == 1)
		size--;

	for (i = 0; i < size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			ad7768_sampl_freq_avail[i]);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(ad7768_attr_sampl_freq_avail);
extern int drive_vref_mv;
extern int drive_shift_val;
static int ad7768_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = ad7768_get_data(indio_dev);
	int ret,i;
/*	unsigned long currentgain,tedsbit=0x2;
	char sTedsId[180];
	int temp[64];*/
	switch (info) {	
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		if(chan->channel<8) //adc
			*val=GetOffset(chan->channel);
		else *val=0; //dac
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:	
		if(chan->channel<8) {//adc
			*val = gain[chan->channel];
			*val=Get_iCurrentRange(chan->channel);
			printk(KERN_INFO "channel %d gain sw:0x%x hw:0x%x\n",chan->channel,gain[chan->channel],*val);
			gain[chan->channel]=*val;
		}
		else *val=0; //dac
		return IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PHASE:	
		if(chan->channel<8) {//adc
		//	*val = gain[chan->channel];
			*val=Get_pointOffset(chan->channel);
			printk(KERN_INFO "channel %d phase hw:0x%x\n",chan->channel,*val);
		//	gain[chan->channel]=*val;
		}
		else *val=0; //dac
		return IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_RAW:
#if 0
		if (incoupling[chan->channel]) {
                    /*    printk (KERN_INFO "read in coupling chan %d\n",chan->channel);
			w1_timeout=0;
                        gpiod_get_array_value_cansleep(incoupling[chan->channel]->ndescs, incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain); // save current gain
			currentgain = currentgain&0x1f;
			gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&tedsbit); // turn on teds bit
			
			msleep(2000);*/
			ret=getTedsId(sTedsId,chan->channel);
		//	printk(KERN_INFO "Using %s Teds device\n",sTedsId);			
			if (ret>0) {
				for (i=0;i<64;i++) {
					temp[i]=sTedsId[i];
				printk(KERN_INFO "raw:%x",(u8)temp[i]);
				}
			}
			else {
				for(i=0;i<64;i++) temp[i]=0;
			}
			printk(KERN_INFO "count:%d",ret);
			*val2=ret;
		//	gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&currentgain); // reset gain value
		}
		else *val2=0;
#endif
		*val=1234;
	//	printk(KERN_INFO "read raw teds\n");
	//	for (ret=0;ret<64;ret++) val[ret]=ret;//sTedsId[ret];
	//	*val = testMsg;
		return IIO_VAL_INT;
		break;
	default:
		return -EINVAL;
	}
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	int current_val;
//	printk (KERN_INFO "write raw chan 0x%x value 0x%x 0x%x\n",chan->channel,val,val2);
	
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_set_sampling_freq(indio_dev, val);
	case IIO_CHAN_INFO_OFFSET:
		SetOffset(chan->channel,val);	
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		gain[chan->channel]=val;
//		printk(KERN_INFO "set chan %d gain to %d\n",chan->channel,val);
		Set_iCurrentRange(chan->channel,val);
		break;
	case IIO_CHAN_INFO_PHASE:	
//		printk(KERN_INFO "set chan %d phase to %d\n",chan->channel,val);
		Set_pointOffset(chan->channel,val);
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t getCoupling(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	if (chan->channel<8 && incoupling[chan->channel]) {
//		printk (KERN_INFO "read in coupling chan %d\n",chan->channel);
		gpiod_get_array_value_cansleep(incoupling[chan->channel]->ndescs, incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&val);
		val = val&0x1f;
//		printk (KERN_INFO "read in coupling chan %d 0x%x\n",chan->channel,val);
	//	return IIO_VAL_INT;
	}
	else val=0;
	len = sprintf(buf, "%d\0", val)+1;
//	printk(KERN_INFO "get coupling %d from %s len %d",val,buf,len);
	return len;
}

static ssize_t setCoupling(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long current_val,val;
	kstrtol(buf,10,&val);
	if (chan->channel==8) return len;
//	printk(KERN_INFO "set coupling %d from %s",val,buf);
	if (val & 0x1 && val & 0x2) // cannot have both ICP and TEDS on at the same time
			val &= 0xfc; // turn both off until i hear otherwise
	if (incoupling[chan->channel])  {
//		printk (KERN_INFO "write in coupling chan %d\n",chan->channel);
		gpiod_get_array_value_cansleep(incoupling[chan->channel]->ndescs, incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&current_val);
		current_val = current_val&0x1f;
//		printk (KERN_INFO "read in coupling chan 0x%x value 0x%x 0x%x ndesc %d\n",chan->channel,val,current_val,incoupling[chan->channel]->ndescs);	
		if (current_val & 0x1 && val & 0x2)	{ //ICP on and TEDS requested on, turn off ICP
			current_val = current_val & 0xfe;
			gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&current_val);
		}
		else if (current_val & 0x2 && val & 0x1) { //TEDS on and ICP requested on, turn off TEDS
			current_val = current_val & 0xfd;
			gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&current_val);
		}	
	//	printk (KERN_INFO "2nd write in coupling chan 0x%x value 0x%x 0x%x\n",chan->channel,val,val2);	
		gpiod_set_array_value_cansleep(incoupling[chan->channel]->ndescs,incoupling[chan->channel]->desc, incoupling[chan->channel]->info, (unsigned long *)&val);
//		printk (KERN_INFO "write in coupling chan 0x%x value 0x%x\n",chan->channel,val);
	} else printk (KERN_INFO "no coupling to write chan 0x%x value 0x%x\n",chan->channel,val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}
/*
static ssize_t Get_iCurrentOffset(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	val=GetOffset(chan->channel);

	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get offset %d from %s len %d",val,buf,len);
	return len;
}

static ssize_t Set_iCurrentOffset(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
//	printk(KERN_INFO "set offset %d from %s",val,buf);
	SetOffset(chan->channel, val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}
*/
static ssize_t Get_iCurrentStep(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	if (chan->channel==8) 
		val=0;
	else 
		val=GetStep(chan->channel);
	
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get step %d from %s len %d",val,buf,len);
	return len;
}

static ssize_t Get_iCurrentStatus(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	if (chan->channel==8) 
		val=0;
	else 
		val=GetStatus(chan->channel);
	
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get channel %d status %d from %s len %d",chan->channel,val,buf,len);
	return len;
}

static ssize_t Set_iCurrentStep(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
	//printk(KERN_INFO "set step %d from %s",val,buf);
	if (chan->channel<8)
		SetStep(chan->channel, val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}

/*
 * resync -- write-only: issue ONE sync pulse on demand, even when nothing has
 * changed (added 2026-09-14).
 *
 * A filter_type or sampling_frequency change on a SLAVE only takes effect at
 * the next sync edge, and a slave cannot produce one: its local SPI_SYNC
 * drives the AD7768's SYNC_OUT, which is not routed anywhere in slave mode.
 * The edge has to come from the master. When the client configures the master
 * first, the master's own setup pulse goes out BEFORE the slave's register
 * write lands, so the slave's ADC sits halted until the next edge -- the
 * start_pulse. Its DAC cannot prefill meanwhile (the prefill is paced by the
 * ADC sample strobe), so the slave's DacStartGate releases with an empty FIFO
 * and its DAC starts late. Reproduced on the two-box rig on the first run
 * after boot and on any run after a real ADC filter_type change
 * (SINC5 <-> WIDEBAND). Since kernel #612 rewriting an unchanged value no
 * longer pulses, so the client needs an explicit way to fire one.
 *
 * Client sequence: configure every board -> write resync=1 on the MASTER ->
 * open the DAC buffers -> submit data -> start_pulse.
 *
 * Refused with -EBUSY while this board's DAC gate is armed: an edge then would
 * release the gate before the prefill -- the misaligned start this exists to
 * prevent. The master cannot see a slave's gate, so the client must call it
 * before ANY board opens its DAC buffer.
 */
static ssize_t ad7768_read_resync(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan, char *buf)
{
	return sysfs_emit(buf, "0\n");
}

static ssize_t ad7768_write_resync(struct iio_dev *indio_dev, uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad7768_state *st = ad7768_get_data(indio_dev);
	bool doit;
	int ret;

	ret = kstrtobool(buf, &doit);
	if (ret)
		return ret;
	if (!doit)
		return len;

	if (dacgate_status() & 0x1) {
		printk(KERN_WARNING "resync refused: DAC gate armed (DAC buffer open) - the edge would release it before the prefill\n");
		return -EBUSY;
	}

	mutex_lock(&st->lock);
	ret = ad7768_sync(st, 3, "resync attr");
	mutex_unlock(&st->lock);

	return ret < 0 ? ret : len;
}

static struct iio_chan_spec_ext_info ad7768_ext_info[] = {
	IIO_ENUM("power_mode",
		 IIO_SHARED_BY_ALL,
		 &ad7768_power_mode_enum),
	IIO_ENUM_AVAILABLE("power_mode",
			   IIO_SHARED_BY_ALL,
			   &ad7768_power_mode_enum),
	IIO_ENUM("sleep_mode",
		 IIO_SHARED_BY_ALL,
		 &ad7768_sleep_type_iio_enum),
	IIO_ENUM_AVAILABLE("sleep_mode",
			   IIO_SHARED_BY_ALL,
			   &ad7768_sleep_type_iio_enum),
	IIO_ENUM("filter_type",
		 IIO_SHARED_BY_ALL,
		 &ad7768_filter_type_iio_enum),	
	IIO_ENUM_AVAILABLE("filter_type",
			   IIO_SHARED_BY_ALL,
			   &ad7768_filter_type_iio_enum),
	IIO_ENUM("start_pulse",
		 IIO_SHARED_BY_ALL,
		 &ad7768_start_pulse_iio_enum),	
	IIO_ENUM_AVAILABLE("start_pulse",
			   IIO_SHARED_BY_ALL,
			   &ad7768_start_pulse_iio_enum),
	{
		.name = "resync",
		.read = ad7768_read_resync,
		.write = ad7768_write_resync,
		.shared = IIO_SHARED_BY_ALL,
	},
	{
		.name="coupling",
		.read=getCoupling,
		.write=setCoupling,
		.shared=IIO_SEPARATE,
	},	
	{
		.name="step",
		.read=Get_iCurrentStep,
		.write=Set_iCurrentStep,
		.shared=IIO_SEPARATE,
	},
	{
		.name="AD7768_status",
		.read=Get_iCurrentStatus,
		.write=NULL,
		.shared=IIO_SEPARATE,
	},
	{
		.name="Teds_info",
		.read=getTedsId,
		.write=NULL,
		.shared=IIO_SEPARATE,
	},	
	{ },

};

static struct attribute *ad7768_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad7768_group = {
	.attrs = ad7768_attributes,
};

static const struct iio_info ad7768_info = {
	.attrs = &ad7768_group,
	.read_raw = &ad7768_read_raw,
	.write_raw = &ad7768_write_raw,
	.debugfs_reg_access = &ad7768_reg_access,
};

#define AD7768_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
							  BIT(IIO_CHAN_INFO_PHASE) | \
							  BIT(IIO_CHAN_INFO_OFFSET) | \	
							  BIT(IIO_CHAN_INFO_RAW), \								  					  
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.address = index,					\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.ext_info = ad7768_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
	}
	
#define DRIVE_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.address = index,					\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.ext_info = ad7768_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,					\
			.storagebits = 32,				\
		},							\
	}

static const struct axiadc_chip_info ad7768_conv_chip_info = {
	.id = ID_AD7768,
	.name = "ad7768_axi_adc",
	.num_channels = 9,
	.channel[0] = AD7768_CHAN(0),
	.channel[1] = AD7768_CHAN(1),
	.channel[2] = AD7768_CHAN(2),
	.channel[3] = AD7768_CHAN(3),
	.channel[4] = AD7768_CHAN(4),
	.channel[5] = AD7768_CHAN(5),
	.channel[6] = AD7768_CHAN(6),
	.channel[7] = AD7768_CHAN(7),
	.channel[8] = DRIVE_CHAN(8),	
};

static const struct axiadc_chip_info ad7768_4_conv_chip_info = {
	.id = ID_AD7768_4,
	.name = "ad7768_4_axi_adc",
	.num_channels = 4,
	.channel[0] = AD7768_CHAN(0),
	.channel[1] = AD7768_CHAN(1),
	.channel[2] = AD7768_CHAN(2),
	.channel[3] = AD7768_CHAN(3),
};

static const unsigned long ad7768_available_scan_masks[]  = { 0xFF, 0x00 };

static void ad7768_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static void ad7768_clk_disable(void *data)
{
	struct clk *clk = data; 

	clk_disable_unprepare(clk);
}

static int ad7768_datalines_from_dt(struct ad7768_state *st)
{
	const unsigned int *available_datalines;
	unsigned int i, len;
	int ret;

	st->datalines = 1;
	ret = device_property_read_u32(&st->spi->dev, "adi,data-lines",
				       &st->datalines);
	if (ret < 0)
		return (ret != -EINVAL) ? ret : 0;

	switch (st->chip_info->id) {
	case ID_AD7768:
		available_datalines = ad7768_available_datalines;
		len = ARRAY_SIZE(ad7768_available_datalines);
		break;
	case ID_AD7768_4:
		available_datalines = ad7768_4_available_datalines;
		len = ARRAY_SIZE(ad7768_4_available_datalines);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		if (available_datalines[i] == st->datalines)
			return 0;
	}

	return -EINVAL;
}

static int ad7768_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad7768_state *st = conv->phy;

	axiadc_write(axiadc_st, ADI_REG_CNTRL_3, ADI_CRC_EN);
	axiadc_write(axiadc_st, ADI_REG_CNTRL, ADI_NUM_LANES(st->datalines));

	return 0;
}

static int ad7768_register_axi_adc(struct ad7768_state *st)
{
	struct axiadc_converter	*conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->read_label = &ad7768_read_label;
	conv->chip_info = st->chip_info;
	conv->adc_output_mode = AD7768_OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->reg_access = &ad7768_reg_access;
	conv->write_raw = &ad7768_write_raw;
	conv->read_raw = &ad7768_read_raw;
	conv->post_setup = &ad7768_post_setup;
	conv->attrs = &ad7768_group;
	conv->phy = st;
	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int ad7768_register(struct ad7768_state *st, struct iio_dev *indio_dev)
{
	int ret;

	indio_dev->name = "ad7768";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad7768_info;
	indio_dev->available_scan_masks = ad7768_available_scan_masks;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent, indio_dev,
					      "rx", IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad7768_probe(struct spi_device *spi)
{
	
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	int ret,i;
	char cplStr[20];
	unsigned long val;
	int errno, irq_line;
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info = device_get_match_data(&spi->dev);
	if (!st->chip_info) {
		st->chip_info = (void *)spi_get_device_id(spi)->driver_data;
		if (!st->chip_info)
			return PTR_ERR(st->chip_info);
	}

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);
	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7768_reg_disable, st->vref);
	if (ret)
		return ret;

	st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);
	ret = clk_prepare_enable(st->mclk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad7768_clk_disable, st->mclk);
	if (ret)
		return ret;

	st->spi = spi;
	
	/* get out of reset state */ 
	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(2);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(1660);
	}
	irq_line=spi->irq;//of_irq_get(spi, 0);
	ret = ad7768_datalines_from_dt(st);
	if (ret < 0)
		return ret;

	ret = ad7768_set_sampling_freq(indio_dev, AD7768_MAX_SAMP_FREQ);
	if (ret < 0)
		return ret;

	/*
	 * Seed the ch9 loop delay from the chip's current filter (2026-09-15).
	 * Without this s_loop_adc_filter stays -1 and apply_loop_delay() bails
	 * out, so after a fresh boot ch9 carried NO delay until the client
	 * happened to write filter_type. Read rather than assume the reset
	 * default -- measured on hardware the part comes up in SINC5, not
	 * WIDEBAND. The fallback only matters if the SPI read itself fails.
	 */
	{
		unsigned int mode;

		if (ad7768_spi_reg_read(st, AD7768_CH_MODE, &mode) >= 0)
			s_loop_adc_filter = AD7768_CH_MODE_GET_FILTER_TYPE(mode) ? 1 : 0;
		else
			s_loop_adc_filter = 0;
		ad7768_apply_loop_delay();
	}
	
	ret =  ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
				     AD7768_INTERFACE_CFG_CRC_SELECT_MSK,
				     AD7768_INTERFACE_CFG_CRC_SELECT);
	if (ret < 0)
		return ret;
#if 0
	for(i=0;i<8;i++) {
		sprintf(cplStr, "incoupling%d",i);
		incoupling[i] = gpiod_get_array(&spi->dev, cplStr, GPIOD_OUT_LOW);
		if (IS_ERR(incoupling[i])) {
			printk(KERN_INFO "unable to get adc chan %d gpio pointer err:%d\n",i,PTR_ERR(incoupling[i]));
			incoupling[i] = NULL;
		}
		else {
			val=0x1C;
			printk(KERN_INFO "found %d ndesc %d\n",i,incoupling[i]->ndescs);
			gpiod_set_array_value(incoupling[i]->ndescs,incoupling[i]->desc, incoupling[i]->info, (unsigned long *)&val);
		}
	}
#endif
	mutex_init(&st->lock);
#if 1
/*	if (errno = gpio_direction_input(GPIO) != 0) {
		printk(KERN_INFO "Can't set gpio direction, errno = %d",errno);
		gpio_free(GPIO);
	}*/
//	irq_line = 88;//gpio_to_irq(GPIO);
	printk(KERN_INFO "irq_line = %d",irq_line);
	free_irq(irq_line, st);
	errno = request_irq(irq_line, (irq_handler_t)sync_buf, IRQF_TRIGGER_RISING, "sync_buf", st);
	if (errno < 0)
		printk(KERN_INFO "Problem requesting IRQ errno = %d", errno);
	printk(KERN_INFO "IRQ installed at %d",irq_line);
#endif
	/*  If there is a reference to a dma channel, the device is not using
	 *  the axi adc
	 */
	if (device_property_present(&spi->dev, "dmas"))
		ret = ad7768_register(st, indio_dev);
	else
		ret = ad7768_register_axi_adc(st);

	return ret;
}

static const struct spi_device_id ad7768_id[] = {
	{"ad7768", (kernel_ulong_t)&ad7768_conv_chip_info},
	{"ad7768-4", (kernel_ulong_t)&ad7768_4_conv_chip_info},
	{},
};
MODULE_DEVICE_TABLE(spi, ad7768_id);

static const struct of_device_id ad7768_of_match[]  = {
	{ .compatible = "adi,ad7768", .data = &ad7768_conv_chip_info },
	{ .compatible = "adi,ad7768-4", .data = &ad7768_4_conv_chip_info },
	{},
};
MODULE_DEVICE_TABLE(of, ad7768_of_match);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name	= "ad7768",
		.of_match_table = ad7768_of_match,
	},
	.probe		= ad7768_probe,
	.id_table	= ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC");
MODULE_LICENSE("GPL v2");
