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
#include <linux/gpio/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/dac/ad5791.h>

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
//struct gpio_descs *synchgpio=NULL;
extern void SetDriveOffset(int);
extern int GetDriveOffset(void);
extern void SetDriveStep(int);
extern int GetDriveStep(void);
extern void SetDriveScale(int);
extern int GetDriveScale(void);
extern void GetDriveCounts(int counts[]);
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
	st->data[0].d32 = /*cpu_to_be32(*/AD5791_CMD_WRITE |
			      AD5791_ADDR(addr) |
			      (val & AD5791_DAC_MASK);
			      
	write_command[0] =(st->data[0].d32 >> 16) & 0xFF;
	write_command[1] =(st->data[0].d32 >> 8) & 0xFF;
	write_command[2] =(st->data[0].d32 >> 0) & 0xFF;
//	printk(KERN_INFO "SPI write 0x%x or 0x%x\n",st->data[0].d32,st->data[0].d8[1]);
	return spi_write(st->spi, write_command, 3);
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
	printk (KERN_INFO "read raw mask %d \n",m);
	
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = st->cached_val;
		return IIO_VAL_INT;
		ret = ad5791_spi_read(st, chan->address, val);
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

static ssize_t Set_iCurrentStep(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long val;
	kstrtol(buf,10,&val);
	//printk(KERN_INFO "set step %d from %s",val,buf);
	SetDriveStep(val);
	return len;//=sprintf(buf,"%d",val);
//	return sprintf(buf,"%d",val);;
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

static const struct iio_chan_spec_ext_info ad5791_ext_info[] = {
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
		.name = "powerdown",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ad5791_read_dac_powerdown,
		.write = ad5791_write_dac_powerdown,
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
//		printk(KERN_INFO "Write raw 0x%x\n",val);
		val &= GENMASK(/*chan->scan_type.realbits - 1*/19, 0);
		val <<= chan->scan_type.shift;
//		printk(KERN_INFO "Write raw mod 0x%x\n",val);
		return ad5791_spi_write(st, chan->address, val);
		break;
	case IIO_CHAN_INFO_OFFSET:
		SetDriveOffset(val);	
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

	spi_engine_offload_enable(st->spi, false);
//	printk(KERN_INFO "HW post disable\n");
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
	struct iio_dev *indio_dev;
	struct ad5791_state *st;
	char cplStr[20];
	int ret, pos_voltage_uv = 0, neg_voltage_uv = 0,i=0;
	u32 core, val;
	struct device_node *np = spi->dev.of_node;
	printk(KERN_INFO "ad5791 probe\n");
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
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
	//ret = ad5791_spi_write(st, AD5791_ADDR_DAC0, st->ctrl);
	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ad5791_info;
	indio_dev->modes = INDIO_DIRECT_MODE| INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad5791_buffer_ops;
	indio_dev->channels
		= &ad5791_channels[spi_get_device_id(spi)->driver_data];
//	printk(KERN_INFO "dac device found %d\n",spi_get_device_id(spi)->driver_data);	
	indio_dev->num_channels = 1;
	indio_dev->name = spi_get_device_id(st->spi)->name;
//	indio_dev->direction = DMA_MEM_TO_DEV;
	st->num_bits = indio_dev->channels->scan_type.realbits;
	drive_shift_val = (1 << st->num_bits) - 1;
	ret = ad5791_hardware_buffer_alloc(st,indio_dev);
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
