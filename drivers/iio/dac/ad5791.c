// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5760, AD5780, AD5781, AD5790, AD5791 Voltage Output Digital to Analog
 * Converter
 *
 * Copyright 2011 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/slab.h> 
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>

#include <linux/math64.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/dac/ad5791.h> // see about getting rid of this

#define AD5791_DAC_MASK			GENMASK(19, 0)

#define AD5791_CMD_READ			BIT(23)
#define AD5791_CMD_WRITE		0
#define AD5791_ADDR(addr)		((addr) << 20)

/* Registers */
#define AD5791_ADDR_NOOP		0
#define AD5791_ADDR_DAC0		1
#define AD5791_ADDR_CTRL		2
#define AD5791_ADDR_CLRCODE		3
#define AD5791_ADDR_SW_CTRL		4

/* Control Register */
#define AD5791_CTRL_RBUF		BIT(1)
#define AD5791_CTRL_OPGND		BIT(2)
#define AD5791_CTRL_DACTRI		BIT(3)
#define AD5791_CTRL_BIN2SC		BIT(4)
#define AD5791_CTRL_SDODIS		BIT(5)
#define AD5761_CTRL_LINCOMP(x)		((x) << 6)

#define AD5791_LINCOMP_0_10		0
#define AD5791_LINCOMP_10_12		1
#define AD5791_LINCOMP_12_16		2
#define AD5791_LINCOMP_16_19		3
#define AD5791_LINCOMP_19_20		12

#define AD5780_LINCOMP_0_10		0
#define AD5780_LINCOMP_10_20		12

/* Software Control Register */
#define AD5791_SWCTRL_LDAC		BIT(0)
#define AD5791_SWCTRL_CLR		BIT(1)
#define AD5791_SWCTRL_RESET		BIT(2)

#define AD5791_DAC_PWRDN_6K		0
#define AD5791_DAC_PWRDN_3STATE		1
int iFilterSelect=3;
struct gpio_descs *outcoupling=NULL;
struct iio_dev *g_indio_dev;
//struct gpio_descs *synchgpio=NULL;
extern void SetDriveOffset(int);
extern int GetDriveOffset(void);
extern void SetDriveStep(int);
extern int GetDriveStep(void);
extern void SetDriveScale(int);
extern int GetDriveScale(void);
extern void SetClipValue(int);
extern int GetClipValue(void);

extern void GetDriveCounts(int counts[]);
extern void iio_disable_all_buffers(struct iio_dev *indio_dev);
extern int g_cola_enabled;
#define DRIVESCALE_BASE_ADDR_AD5791  0x40000000UL
#define DRIVESCALE_COLA_ENABLE_OFFSET  0x80
#define DRIVESCALE_GROUP_DELAY_OFFSET  0x88
static void __iomem *g_ds_base = NULL;
extern u32 iio_dma_read_output_word_from_end(u32 offset);
/**
 * struct ad5791_chip_info - chip specific information
 * @get_lin_comp:	function pointer to the device specific function
 */

struct ad5791_chip_info {
	int (*get_lin_comp)	(unsigned int span);
};

/**
 * struct ad5791_state - driver instance specific data
 * @spi:			spi_device
 * @reg_vdd:		positive supply regulator
 * @reg_vss:		negative supply regulator
 * @chip_info:		chip model specific constants
 * @vref_mv:		actual reference voltage used
 * @vref_neg_mv:	voltage of the negative supply
 * @pwr_down_mode	current power down mode
 */

struct ad5791_state {
	struct spi_device		*spi;
	struct regulator		*reg_vdd;
	struct regulator		*reg_vss;
	const struct ad5791_chip_info	*chip_info;
	unsigned short			vref_mv;
	unsigned int			vref_neg_mv;
	unsigned			ctrl;
	unsigned			pwr_down_mode;
	bool				pwr_down;
	unsigned int num_bits;
	unsigned			cached_val;
	u32				rampdown_start_val; /* DAC code captured in postdisable */
	struct spi_transfer spi_transfer;
	struct spi_message spi_msg;
	union {
		__be32 d32;
		u8 d8[4];
	} data[3] ____cacheline_aligned;
};

/**
 * ad5791_supported_device_ids:
 */

enum ad5791_supported_device_ids {
	ID_AD5760,
	ID_AD5780,
	ID_AD5781,
	ID_AD5791,
};

static int ad5791_spi_write(struct ad5791_state *st, u8 addr, u32 val)
{
	u8 write_command[3];
	st->data[0].d32 = cpu_to_be32(AD5791_CMD_WRITE |
			      AD5791_ADDR(addr) |
			      (val & AD5791_DAC_MASK));
			      
//	write_command[0] =(st->data[0].d32 >> 16) & 0xFF;
//	write_command[1] =(st->data[0].d32 >> 8) & 0xFF;
//	write_command[2] =(st->data[0].d32 >> 0) & 0xFF;
//	printk(KERN_INFO "SPI write 0x%x or 0x%x\n",st->data[0].d32,st->data[0].d8[1]);
//	return spi_write(st->spi, write_command, 3);
	return spi_write(st->spi, &st->data[0].d8[1], 3);
}

/* Like ad5791_spi_read but uses spi_sync_locked — call only while
 * spi_bus_lock is held (e.g. from postdisable).
 */
static int ad5791_spi_read_locked(struct ad5791_state *st, u8 addr, u32 *val)
{
	struct spi_transfer xfers[] = {
		{
			.tx_buf        = &st->data[0].d8[1],
			.bits_per_word = 8,
			.len           = 3,
			.cs_change     = 1,
		}, {
			.tx_buf        = &st->data[1].d8[1],
			.rx_buf        = &st->data[2].d8[1],
			.bits_per_word = 8,
			.len           = 3,
		},
	};
	struct spi_message msg;
	int ret;

	st->data[0].d32 = cpu_to_be32(AD5791_CMD_READ | AD5791_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(AD5791_ADDR(AD5791_ADDR_NOOP));

	spi_message_init_with_transfers(&msg, xfers, ARRAY_SIZE(xfers));
	ret = spi_sync_locked(st->spi, &msg);

	*val = be32_to_cpu(st->data[2].d32);
	return ret;
}

static int ad5791_spi_read(struct ad5791_state *st, u8 addr, u32 *val)
{
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.bits_per_word = 8,
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d8[1],
			.rx_buf = &st->data[2].d8[1],
			.bits_per_word = 8,
			.len = 3,
		},
	};

	st->data[0].d32 = cpu_to_be32(AD5791_CMD_READ |
			      AD5791_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(AD5791_ADDR(AD5791_ADDR_NOOP));

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));

	*val = be32_to_cpu(st->data[2].d32);

	return ret;
}

static const char * const ad5791_powerdown_modes[] = {
	"6kohm_to_gnd",
	"three_state",
};

static int ad5791_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	return st->pwr_down_mode;
}

static int ad5791_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	st->pwr_down_mode = mode;

	return 0;
}

static const struct iio_enum ad5791_powerdown_mode_enum = {
	.items = ad5791_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5791_powerdown_modes),
	.get = ad5791_get_powerdown_mode,
	.set = ad5791_set_powerdown_mode,
};

static ssize_t ad5791_read_dac_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->pwr_down);
}

static ssize_t ad5791_write_dac_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	 size_t len)
{
	bool pwr_down;
	int ret;
	struct ad5791_state *st = iio_priv(indio_dev);

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;
//	pwr_down=false; // for now
	if (!pwr_down) {
		st->ctrl &= ~(AD5791_CTRL_OPGND | AD5791_CTRL_DACTRI);
	} else {
		if (st->pwr_down_mode == AD5791_DAC_PWRDN_6K)
			st->ctrl |= AD5791_CTRL_OPGND;
		else if (st->pwr_down_mode == AD5791_DAC_PWRDN_3STATE)
			st->ctrl |= AD5791_CTRL_DACTRI;
	}
	st->pwr_down = pwr_down;

	ret = ad5791_spi_write(st, AD5791_ADDR_CTRL, st->ctrl);

	return ret ? ret : len;
}

static int ad5791_get_lin_comp(unsigned int span)
{
	if (span <= 10000)
		return AD5791_LINCOMP_0_10;
	else if (span <= 12000)
		return AD5791_LINCOMP_10_12;
	else if (span <= 16000)
		return AD5791_LINCOMP_12_16;
	else if (span <= 19000)
		return AD5791_LINCOMP_16_19;
	else
		return AD5791_LINCOMP_19_20;
}

static int ad5780_get_lin_comp(unsigned int span)
{
	if (span <= 10000)
		return AD5780_LINCOMP_0_10;
	else
		return AD5780_LINCOMP_10_20;
}
static const struct ad5791_chip_info ad5791_chip_info_tbl[] = {
	[ID_AD5760] = {
		.get_lin_comp = ad5780_get_lin_comp,
	},
	[ID_AD5780] = {
		.get_lin_comp = ad5780_get_lin_comp,
	},
	[ID_AD5781] = {
		.get_lin_comp = ad5791_get_lin_comp,
	},
	[ID_AD5791] = {
		.get_lin_comp = ad5791_get_lin_comp,
	},
};

int drive_vref_mv=0;
int drive_shift_val=0;

static int ad5791_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5791_state *st = iio_priv(indio_dev);
	u64 val64;
	int ret;
		
	switch (m) {
	case IIO_CHAN_INFO_RAW:
	/*	*val = st->cached_val;
		return IIO_VAL_INT;*/
		ret = ad5791_spi_read(st, chan->address, val);
		printk (KERN_INFO "read raw %d \n",*val);
		if (ret)
			return ret;
		*val &= AD5791_DAC_MASK;
		*val >>= chan->scan_type.shift;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val=GetDriveScale();
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val=GetDriveOffset();
		return IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PEAK:
		*val=GetClipValue();
		return IIO_VAL_INT;
		break;
/*	case IIO_CHAN_INFO_HYSTERESIS:	
		if (outcoupling) {
			gpiod_get_array_value_cansleep(outcoupling->ndescs, outcoupling->desc, outcoupling->info, (unsigned long *)val);
			printk (KERN_INFO "read out coupling 0x%x\n",*val);			
		}
		else {
			*val=0;
			printk (KERN_INFO "no out coupling\n",*val);	
		}
		return IIO_VAL_INT;*/
	default:
		return -EINVAL;
	}

};
#if 0
static ssize_t Get_iCurrentOffset(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	val=GetDriveOffset();

	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get offset %d from %s len %d",val,buf,len);
	return len;
}

static ssize_t Set_iCurrentOffset(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
//	printk(KERN_INFO "set offset %d from %s",val,buf);
	SetDriveOffset(val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}
#endif
static ssize_t Get_iCurrentStep(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	val=GetDriveStep();
	
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get step %d from %s len %d",val,buf,len);
	return len;
}
/*
static ssize_t Set_RampdownRate(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
	printk(KERN_INFO "set rate %d from %s",val,buf);
	SetRampRate(val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}

static ssize_t Get_RampdownRate(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	val=GetRampRate();
	
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get rate %d from %s len %d",val,buf,len);
	return len;
}



static ssize_t iSet_Rampdown(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
	printk(KERN_INFO "set rampdown %d from %s",val,buf);
//	SetRampDown(val);
	if (val) iio_disable_all_buffers(indio_dev);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}

static ssize_t iGet_Rampdown(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
//	val=GetRampDown();
	
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get rampdown %d from %s len %d",val,buf,len);
	return len; 
}
*/
/* Compute (a * b / c) for IEEE 754 single-precision floats
 * packed in u32, returning the result as s32.
 * No FPU instructions are used.
 */
static s32 fp32_muldiv_to_s32(u32 a, u32 b, u32 c)
{
	int sa, sb, sc, ea, eb, ec, result_exp, shift;
	u64 ma, mb, mab;
	u32 mc;
	u64 result_u64;

	/* Zero / denormal / inf / nan -> 0 */
	if (!(a & 0x7FFFFFFF) || !(b & 0x7FFFFFFF) || !(c & 0x7FFFFFFF))
		return 0;
	ea = (a >> 23) & 0xFF;
	eb = (b >> 23) & 0xFF;
	ec = (c >> 23) & 0xFF;
	if (!ea || !eb || !ec || ea == 0xFF || eb == 0xFF || ec == 0xFF)
		return 0;

	sa = (int)(a >> 31);
	sb = (int)(b >> 31);
	sc = (int)(c >> 31);

	/* Mantissas with implicit leading 1, range [2^23, 2^24) */
	ma = ((u64)(a & 0x7FFFFF)) | 0x800000ULL;
	mb = ((u64)(b & 0x7FFFFF)) | 0x800000ULL;
	mc = (u32)((c & 0x7FFFFF) | 0x800000U);

	/* ma*mb in [2^46, 2^48); dividing by mc in [2^23, 2^24) */
	/* gives mab in [2^22, 2^25)                               */
	mab = ma * mb;
	do_div(mab, mc);   /* do_div: u64/u32, avoids __aeabi_uldivmod */

	/* Float value = mab * 2^(ea+eb-ec-127-23) */
	result_exp = (int)(ea + eb - ec) - 127 - 23;

	if (result_exp >= 0) {
		if (result_exp >= 32)
			return (sa ^ sb ^ sc) ? S32_MIN : S32_MAX;
		result_u64 = mab << result_exp;
	} else {
		shift = -result_exp;
		if (shift >= 64)
			return 0;
		result_u64 = mab >> shift;
	}

	if (result_u64 > (u64)S32_MAX)
		result_u64 = (u64)S32_MAX;

	return (sa ^ sb ^ sc) ? -(s32)result_u64 : (s32)result_u64;
}

void SetRampDown(void) {
	struct ad5791_state *st = iio_priv(g_indio_dev);
	int val=0;
	s32 dac_from_dma;
	u32 last_dma        = iio_dma_read_output_word_from_end(0);
	u32 second_last_dma = iio_dma_read_output_word_from_end(1);
	dac_from_dma = fp32_muldiv_to_s32(last_dma,
					   (u32)GetDriveScale(),
					   (u32)GetDriveStep());
	iio_disable_all_buffers(g_indio_dev);
	/* DAC value was captured in postdisable before offload was
	 * disabled, so it is guaranteed to be a clean, in-range read.
	 */
	val = (int)st->rampdown_start_val;
	val &= AD5791_DAC_MASK;
//	val >>= chan->scan_type.shift;
	//printk (KERN_INFO "Ramp Down from Val %d\n",val);
	if (val & 0x80000) {  // negative value: increment in 20-bit space toward 0
		printk(KERN_INFO "ramp up to 0 from negative value 0x%x last_dma: 0x%x second_last_dma: 0x%x dac_from_dma: 0x%x", val, last_dma, second_last_dma, (u32)dac_from_dma);
		while (val != 0) {
			val = (val + 1) & AD5791_DAC_MASK;
			ad5791_spi_write(st, AD5791_ADDR_DAC0, val);
		}
	}
	else {
		printk(KERN_INFO "ramp down to 0 from positive value 0x%x last_dma: 0x%x second_last_dma: 0x%x dac_from_dma: 0x%x", val, last_dma, second_last_dma, (u32)dac_from_dma);
		while (val != 0) {
			val--;
			ad5791_spi_write(st, AD5791_ADDR_DAC0, val);
		}
	}
	printk(KERN_INFO "ramp down done");
}

static ssize_t Set_iCurrentStep(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
	//printk(KERN_INFO "set step %d from %s",val,buf);
	SetDriveStep(val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
}


static ssize_t getColaEnabled(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf) {
	printk(KERN_INFO "colaEnabled read: %d\n", g_cola_enabled);
	return sprintf(buf, "%d\n", g_cola_enabled);
}

static ssize_t setColaEnabled(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, const char *buf, size_t len) {
	long val;
	kstrtol(buf, 10, &val);
	g_cola_enabled = val ? 1 : 0;
	if (!g_ds_base)
		g_ds_base = ioremap(DRIVESCALE_BASE_ADDR_AD5791, 0x10000);
	if (g_ds_base) {
		iowrite32(g_cola_enabled, g_ds_base + DRIVESCALE_COLA_ENABLE_OFFSET);
		printk(KERN_INFO "colaEnabled set to %d\n", g_cola_enabled);
	} else {
		printk(KERN_INFO "colaEnabled write failed: ioremap returned NULL\n");
	}
	return len;
}

/*
 * ch9 loopback delay trim (2026-09-15).
 *
 * ad7768.c sets the delay itself from a per-rate table measured on hardware
 * (19,20,21,24,30,41,62,89,178 by interpolator path) plus the ADC filter term
 * (34 WIDEBAND / 4 SINC5). This attribute is a SIGNED trim added on top, for
 * fine tuning from the client. It is re-applied on every rate or filter
 * change, so it survives a table value changing underneath it.
 */
extern int g_loop_delay_trim;
extern void ad7768_loop_delay_retrim(void);

static ssize_t getLoopDelay(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan, char *buf) {
	return sprintf(buf, "%d\n", g_loop_delay_trim);
}

static ssize_t setLoopDelay(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan, const char *buf, size_t len) {
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	if (val < -255 || val > 255)
		return -ERANGE;
	g_loop_delay_trim = (int)val;
	ad7768_loop_delay_retrim();
	printk(KERN_INFO "loop_delay trim set to %d\n", g_loop_delay_trim);
	return len;
}

extern int g_group_delay_trim;
extern void ad7768_set_dac_route(int bank);
extern void ad7768_cola_group_delay_retrim(void);

/* This is the TRIM now (see setGroupDelay), so it must default to 0.
 * It used to be the absolute register value and defaulted to -89; leaving
 * that here made the attribute READ -89 while the trim actually applied
 * was 0, so any client doing a read-modify-write would have shifted cola
 * by 89 samples. */
static int g_group_delay;

static ssize_t getGroupDelay(struct iio_dev *indio_dev, uintptr_t private,
			      const struct iio_chan_spec *chan, char *buf) {
	return sprintf(buf, "%d\n", g_group_delay);
}

static ssize_t setGroupDelay(struct iio_dev *indio_dev, uintptr_t private,
			      const struct iio_chan_spec *chan, const char *buf, size_t len) {
	long val;
	kstrtol(buf, 10, &val);
	/*
	 * TRIM, not an absolute value (2026-09-15). ad7768.c owns the per-rate
	 * base and re-applies base + trim on every rate change -- the same
	 * arrangement as loop_delay. Writing this alone no longer pins the
	 * register.
	 */
	if (val < -255 || val > 255)
		return -ERANGE;
	g_group_delay = (int)val;
	g_group_delay_trim = (int)val;
	ad7768_cola_group_delay_retrim();
	printk(KERN_INFO "cola group_delay trim set to %d\n", g_group_delay_trim);
	return len;
}
static ssize_t getCoupling(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	if (outcoupling) {
		gpiod_get_array_value_cansleep(outcoupling->ndescs, outcoupling->desc, outcoupling->info, (unsigned long *)&val);
		val = val&0x3f;
		printk (KERN_INFO "read out coupling 0x%x\n",val);			
	}
	else {
	//	val=0;
		printk (KERN_INFO "no out coupling\n",val);	
		
	}
	len = sprintf(buf, "%d\0", val)+1;
//	printk(KERN_INFO "get coupling %d from %s len %d",val,buf,len);
	return len;
}

extern void Set_iCurrentFilter(int val);

static ssize_t setCoupling(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long current_val,val;
	int drive_counts[4];
	
	struct ad5791_state *st = iio_priv(indio_dev);
	kstrtol(buf,10,&val);
//	printk(KERN_INFO "set coupling %d from %s",val,buf);
	if (outcoupling) {
		gpiod_set_array_value_cansleep(outcoupling->ndescs,outcoupling->desc, outcoupling->info, (unsigned long *)&val);
		printk (KERN_INFO "write out coupling 0x%x\n",val);
		
		iFilterSelect=(val>>4)&0x3;
	}
	else {
		iFilterSelect=3;
		iFilterSelect=(val>>4)&0x3;
		printk(KERN_INFO "no coupling to write\n");
	}
	Set_iCurrentFilter(iFilterSelect);
	/* the same bank picks the ch9 route delay -- see ad7768.c */
	ad7768_set_dac_route(iFilterSelect);
//	drive_offset = (float)GetDriveOffset();
	GetDriveCounts(drive_counts);
	
//	drive_counts = (int)(drive_offset/drive_step)*-1;
//	rawval &= GENMASK(/*chan->scan_type.realbits - 1*/19, 0);
//	rawval <<= chan->scan_type.shift;
//	printk(KERN_INFO "Write raw mod 0x%x\n",val);
//	ad5791_spi_write(st, chan->address, drive_counts[iFilterSelect]);
	printk(KERN_INFO "select filter %d step %d",iFilterSelect,GetDriveStep());
	
	return len;
}

/*
 * DacStartGate explicit arm control, exposed to the client over libiio as
 * out_voltage0_dacgate_arm. The gate itself lives in the PL; these helpers are
 * exported by industrialio-buffer-dmaengine.c, which owns the ioremap.
 *
 * The gate arms itself on DAC buffer enable, so nothing normally needs to write
 * this. It stays writable purely as a diagnostic: writing 0/1 drives the arm bit
 * directly, which is useful for bench-testing the gate without a client.
 *
 * READING it is the useful part. started=1 with prefilled=1 means the run was
 * held and released on the sync edge -- deterministic and cross-board aligned.
 * started=1 with prefilled=0 means the data arrived after the trigger, so the
 * DAC played immediately and that run is knowingly NOT aligned.
 */
extern void dacgate_set_arm(bool on);
extern u32  dacgate_status(void);

static ssize_t setDacgateArm(struct iio_dev *indio_dev, uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dacgate_set_arm(val != 0);
	printk(KERN_INFO "dacgate: %s by direct write (status 0x%08x)\n",
	       val ? "ARMED" : "disarmed", dacgate_status());
	return len;
}

/*
 * Status readback, so the client can confirm the gate is armed AND that the
 * prefill actually completed before it fires the trigger. With the RTL no
 * longer requiring prefill, prefilled=0 at release means sample 0 is timed by
 * DMA response rather than by the sync edge -- i.e. that run is NOT
 * cross-board aligned. This is the tripwire for that.
 */
static ssize_t getDacgateArm(struct iio_dev *indio_dev, uintptr_t private,
			     const struct iio_chan_spec *chan, char *buf)
{
	u32 s = dacgate_status();

	return sprintf(buf, "armed=%u prefilled=%u started=%u sync_count=%u\n",
		       s & 1, (s >> 1) & 1, (s >> 2) & 1, (s >> 3) & 0xffff);
}

static const struct iio_chan_spec_ext_info ad5791_ext_info[] = {
	{
		.name = "cola_enabled",
		.read = getColaEnabled,
		.write = setColaEnabled,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "loop_delay",
		.read = getLoopDelay,
		.write = setLoopDelay,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "group_delay",
		.read = getGroupDelay,
		.write = setGroupDelay,
		.shared = IIO_SEPARATE,
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
/*{
		.name="rampdown_rate",
		.read=Get_RampdownRate,
		.write=Set_RampdownRate,
		.shared=IIO_SEPARATE,
	},
	{
		.name="rampdown_now",
		.read=iGet_Rampdown,
		.write=iSet_Rampdown,
		.shared=IIO_SEPARATE,
	},*/
	{
		.name = "powerdown",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ad5791_read_dac_powerdown,
		.write = ad5791_write_dac_powerdown,
	},
	{
		.name = "dacgate_arm",
		.read = getDacgateArm,
		.write = setDacgateArm,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SHARED_BY_TYPE,
		 &ad5791_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode",IIO_SHARED_BY_TYPE, &ad5791_powerdown_mode_enum),	
	{ },
};

#define AD5791_CHAN(bits, _shift) {			\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.address = AD5791_ADDR_DAC0,			\
	.channel = 0,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 	\		
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
		BIT(IIO_CHAN_INFO_PEAK) |		\
		BIT(IIO_CHAN_INFO_OFFSET),		\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = (bits),			\
		.storagebits = 32,			\
		.shift = (_shift),			\
		/*.endianness = IIO_BE, maybe */	\
	},						\
	.ext_info = ad5791_ext_info,			\
}

static const struct iio_chan_spec ad5791_channels[] = {
	[ID_AD5760] = AD5791_CHAN(16, 4),
	[ID_AD5780] = AD5791_CHAN(18, 2),
	[ID_AD5781] = AD5791_CHAN(18, 2),
	[ID_AD5791] = AD5791_CHAN(20, 0)
};

static int ad5791_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad5791_state *st = iio_priv(indio_dev);
//	printk (KERN_INFO "write raw mask %d val 0x%x looking for 0x%x\n",mask, val,IIO_CHAN_INFO_HYSTERESIS);
	
	switch (mask) {
	case IIO_CHAN_INFO_RAW:		
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		st->cached_val = val;
		printk(KERN_INFO "Write raw 0x%x\n",val);
		val &= GENMASK(/*chan->scan_type.realbits - 1*/19, 0);
		val <<= chan->scan_type.shift;
//		printk(KERN_INFO "Write raw mod 0x%x\n",val);
		return ad5791_spi_write(st, chan->address, val);
		break;
	case IIO_CHAN_INFO_OFFSET:
		SetDriveOffset(val);	
		break;
	case IIO_CHAN_INFO_PEAK:
		SetClipValue(val);	
		break;
	case IIO_CHAN_INFO_SCALE:
		SetDriveScale(val);	
		break;
/*	case IIO_CHAN_INFO_HYSTERESIS:
		if (outcoupling) {
			gpiod_set_array_value_cansleep(outcoupling->ndescs,outcoupling->desc, outcoupling->info, (unsigned long *)&val);
			printk (KERN_INFO "write out coupling 0x%x\n",val);
		}
		else printk(KERN_INFO "no coupling to write\n");
		break;*/
	default:
		return -EINVAL;
	}
	return 0;
}

static int ad5791_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad5791_state *st = iio_device_get_drvdata(indio_dev);
	int ret;
	
//	printk(KERN_INFO "HW post enable\n");
	
#if 1
	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = 0;
	st->spi_transfer.tx_buf = indio_dev->buffer;
	st->spi_transfer.len = 1;
	st->spi_transfer.bits_per_word = st->num_bits;
	st->spi_transfer.bits_per_word=24;
	st->spi_transfer.cs_change_delay.value=0;
	st->spi_transfer.cs_change_delay.unit=SPI_DELAY_UNIT_USECS;
//	st->spi_transfer.delay_usecs=0;
	st->spi_transfer.speed_hz=20000000;
//	printk(KERN_INFO "Buffer 0x%x\n",st->spi_transfer.tx_buf);
	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);
//	printk(KERN_INFO "spi init ok\n");
	spi_bus_lock(st->spi->master);
//	st->bus_locked = true;
//	printk(KERN_INFO "spi lock ok\n");
	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;
#endif
//	printk(KERN_INFO "HW post enable ok\n");
	spi_engine_offload_enable(st->spi, true);

	return 0;
}

static int ad5791_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad5791_state *st = iio_device_get_drvdata(indio_dev);

	/* Read the current 20-bit DAC code from the streamToData HLS IP
	 * AXI-Lite register. The IP writes DacCounts (clamped, 20-bit
	 * 2's-complement) to this register on every sample, so it always
	 * reflects the live DAC output value.
	 */
	{
		void __iomem *reg = ioremap(0x400D0000 + 0x10, 4);
		if (reg) {
			u32 raw = readl(reg) & 0xfffff;
			iounmap(reg);
			/* Sign-extend from 20 bits to 32 bits */
			st->rampdown_start_val = (u32)((s32)(raw << 12) >> 12);
		} else {
			st->rampdown_start_val = 0;
		}
	}

	spi_engine_offload_enable(st->spi, false);
//	indio_dev->modes = INDIO_DIRECT_MODE;
	printk(KERN_INFO "HW post disable\n");
//	st->bus_locked = false;unsigned int num_bits;
	return spi_bus_unlock(st->spi->master);
}
#if 0
extern bool doSync;
extern bool bWaitForADCTrigger;
struct iio_dma_buffer_queue *g_queue=NULL;
struct iio_dma_buffer_block *g_block=NULL;

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
			   struct iio_dma_buffer_block *block)
{
//	block->block.bytes_used = block->block.size;
	u32* data = block->vaddr;
	int retVal=0;
	
	if (*data == 0xffffffff) {
		//ad7768_sync_data();
		g_queue=queue;
		g_block=block;		
//		doSync=true;
		*data=0;
//		printk(KERN_INFO "Do HW submit block 0x%x size:%d val:0x%x\n",block->phys_addr,block->block.bytes_used,*data );
//		bWaitForADCTrigger=true;
		
//		ad7768_sync_data();
		retVal = 0;	
	//	mdelay(1);
	}	
	else {
		printk(KERN_INFO "dac submit");
		while (bWaitForADCTrigger)		{
			udelay(1);	
			yield();
		}
		retVal = iio_dmaengine_buffer_submit_block(queue, block);
	}
	return retVal;
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};
#endif
static const struct iio_buffer_setup_ops ad5791_buffer_ops = {
	.postenable = &ad5791_buffer_postenable,
	.postdisable = &ad5791_buffer_postdisable,
};
static const unsigned long ad5791_available_scan_masks[]  = { 0xFF, 0x00 };
static int ad5791_hardware_buffer_alloc(struct ad5791_state *st,struct iio_dev *indio_dev)
{
	int ret;

	indio_dev->name = "ad5791";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	
	indio_dev->available_scan_masks = ad5791_available_scan_masks;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent, indio_dev,
					      "tx", IIO_BUFFER_DIRECTION_OUT);
	if (ret)
		return ret;
	iio_device_set_drvdata(indio_dev,st);
	if (!g_ds_base)
		g_ds_base = ioremap(DRIVESCALE_BASE_ADDR_AD5791, 0x10000);
	if (g_ds_base)
		iowrite32((u32)g_group_delay, g_ds_base + DRIVESCALE_GROUP_DELAY_OFFSET);
	ret = devm_iio_device_register(&st->spi->dev, indio_dev);
	return ret;
} 

static const struct iio_info ad5791_info = {
	.read_raw = &ad5791_read_raw,
	.write_raw = &ad5791_write_raw,
};

static int ad5791_probe(struct spi_device *spi)
{
	struct ad5791_platform_data *pdata = spi->dev.platform_data;
//	struct iio_dev *indio_dev;
	struct ad5791_state *st;
	char cplStr[20];
	int ret, pos_voltage_uv = 0, neg_voltage_uv = 0,i=0;
	u32 core, val;
	struct device_node *np = spi->dev.of_node;
	printk(KERN_INFO "ad5791 probe\n");
	g_indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!g_indio_dev)
		return -ENOMEM;
	st = iio_priv(g_indio_dev);
	st->reg_vdd = devm_regulator_get(&spi->dev, "vdd");
	if (!IS_ERR(st->reg_vdd)) {
		ret = regulator_enable(st->reg_vdd);
		if (ret)
			return ret;

		ret = regulator_get_voltage(st->reg_vdd);
		if (ret < 0)
			goto error_disable_reg_pos;

		pos_voltage_uv = ret;
	}

	st->reg_vss = devm_regulator_get(&spi->dev, "vss");
	if (!IS_ERR(st->reg_vss)) {
		ret = regulator_enable(st->reg_vss);
		if (ret)
			goto error_disable_reg_pos;

		ret = regulator_get_voltage(st->reg_vss);
		if (ret < 0)
			goto error_disable_reg_neg;

		neg_voltage_uv = ret;
	}

	st->pwr_down = false;
	st->spi = spi;
/*	ret = of_property_read_u32(np, "core",&core);
	void __iomem *coreregs = ioremap(core,0x80);
	printk(KERN_INFO "dma mapped %d core:0x%x corereg:0x%x\n",core,coreregs);
	val=3;
	writel(val, coreregs + 0x40);
	val=0x186; //256kHz
	writel(val, coreregs + 0x4c);*/
unsigned int num_bits;
	if (!IS_ERR(st->reg_vss) && !IS_ERR(st->reg_vdd)) {
		st->vref_mv = (pos_voltage_uv + neg_voltage_uv) / 1000;
		st->vref_neg_mv = neg_voltage_uv / 1000;
	} else if (pdata) {
		st->vref_mv = pdata->vref_pos_mv + pdata->vref_neg_mv;
		st->vref_neg_mv = pdata->vref_neg_mv;
	} else {
		dev_warn(&spi->dev, "reference voltage unspecified\n");
	}
	drive_vref_mv=st->vref_mv;
	
	ret = ad5791_spi_write(st, AD5791_ADDR_SW_CTRL, AD5791_SWCTRL_RESET);
	if (ret)
		goto error_disable_reg_neg;

	st->chip_info =	&ad5791_chip_info_tbl[spi_get_device_id(spi)
					      ->driver_data];


	st->ctrl = AD5761_CTRL_LINCOMP(st->chip_info->get_lin_comp(st->vref_mv))
		  | ((pdata && pdata->use_rbuf_gain2) ? 0 : AD5791_CTRL_RBUF) |
		  AD5791_CTRL_BIN2SC;
//	st->ctrl=0; //for now
	ret = ad5791_spi_write(st, AD5791_ADDR_CTRL, st->ctrl |
		AD5791_CTRL_OPGND | AD5791_CTRL_DACTRI);
	if (ret)
		goto error_disable_reg_neg;
	ret = ad5791_spi_write(st, AD5791_ADDR_DAC0, st->ctrl);
	spi_set_drvdata(spi, g_indio_dev);

	g_indio_dev->dev.parent = &spi->dev;
	g_indio_dev->info = &ad5791_info;
	g_indio_dev->modes = INDIO_DIRECT_MODE| INDIO_BUFFER_HARDWARE;
	g_indio_dev->setup_ops = &ad5791_buffer_ops;
	g_indio_dev->channels
		= &ad5791_channels[spi_get_device_id(spi)->driver_data];
//	printk(KERN_INFO "dac device found %d\n",spi_get_device_id(spi)->driver_data);	
	g_indio_dev->num_channels = 1;
	g_indio_dev->name = spi_get_device_id(st->spi)->name;
//	g_indio_dev->direction = DMA_MEM_TO_DEV;
	st->num_bits = g_indio_dev->channels->scan_type.realbits;
	drive_shift_val = (1 << st->num_bits) - 1;
	ret = ad5791_hardware_buffer_alloc(st,g_indio_dev);
		st->ctrl=0; //for now
	ret = ad5791_spi_write(st, AD5791_ADDR_CTRL, st->ctrl);
	if (ret) printk(KERN_INFO "error setting dac");
	ret = ad5791_spi_write(st, AD5791_ADDR_DAC0, st->ctrl);
	if (ret) printk(KERN_INFO "error setting dac chan");
//	printk(KERN_INFO "ad5791 probe done\n");
#if 0
	sprintf(cplStr, "synchgpio");
	synchgpio = gpiod_get_array(&spi->dev, cplStr, GPIOD_OUT_HIGH);
	if (IS_ERR(synchgpio)) {
		printk(KERN_INFO "unable to get sync gpio pointer err:%d\n",PTR_ERR(synchgpio));
		synchgpio = NULL;
	}
	printk(KERN_INFO "sync gpio pointer found");

#endif
	printk(KERN_INFO "ad5791 probe done\n");
	return ret;

error_disable_reg_neg:
	if (!IS_ERR(st->reg_vss))
		regulator_disable(st->reg_vss);
error_disable_reg_pos:
	if (!IS_ERR(st->reg_vdd))
		regulator_disable(st->reg_vdd);
	return ret;
}

static void ad5791_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad5791_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg_vdd))
		regulator_disable(st->reg_vdd);

	if (!IS_ERR(st->reg_vss))
		regulator_disable(st->reg_vss);

}

static const struct spi_device_id ad5791_id[] = {
	{"ad5760", ID_AD5760},
	{"ad5780", ID_AD5780},
	{"ad5781", ID_AD5781},
	{"ad5790", ID_AD5791},
	{"ad5791", ID_AD5791},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5791_id);

static struct spi_driver ad5791_driver = {
	.driver = {
		   .name = "ad5791",
		   },
	.probe = ad5791_probe,
	.remove = ad5791_remove,
	.id_table = ad5791_id,
};
module_spi_driver(ad5791_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5760/AD5780/AD5781/AD5790/AD5791 DAC");
MODULE_LICENSE("GPL v2");
