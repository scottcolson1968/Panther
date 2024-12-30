// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AD5446 SPI DAC driver
 *
 * Copyright 2010 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/workqueue.h> 
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/spi/spi.h> 
#include <linux/spi/spi-engine.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/mod_devicetable.h>

#define MODE_PWRDWN_1k		0x1
#define MODE_PWRDWN_100k	0x2
#define MODE_PWRDWN_TRISTATE	0x3

/**
 * struct ad5446_state - driver instance specific data
 * @spi:		spi_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref_mv:		actual reference voltage used
 */

struct ad5446_state {
	struct spi_device *spi;
	struct device		*dev;
	const struct ad5446_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned short			vref_mv;
	unsigned			cached_val;
	unsigned			pwr_down_mode;
	unsigned			pwr_down;
	struct spi_transfer spi_transfer;
	struct spi_message spi_msg;
	unsigned int num_bits;
	bool is_dma_mapped;
	bool bus_locked;
	struct mutex lock;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16 data[2] ____cacheline_aligned;
};

/**
 * struct ad5446_chip_info - chip specific information
 * @channel:		channel spec for the DAC
 * @int_vref_mv:	AD5620/40/60: the internal reference voltage
 * @write:		chip specific helper function to write to the register
 */

struct ad5446_chip_info {
	struct iio_chan_spec	channel;
	u16			int_vref_mv;
	int			(*write)(struct ad5446_state *st, unsigned val);
};

static const char * const ad5446_powerdown_modes[] = {
	"1kohm_to_gnd", "100kohm_to_gnd", "three_state"
};

static int ad5446_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5446_state *st = iio_priv(indio_dev);

	st->pwr_down_mode = mode + 1;

	return 0;
}

static int ad5446_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad5446_state *st = iio_priv(indio_dev);

	return st->pwr_down_mode - 1;
}

static const struct iio_enum ad5446_powerdown_mode_enum = {
	.items = ad5446_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5446_powerdown_modes),
	.get = ad5446_get_powerdown_mode,
	.set = ad5446_set_powerdown_mode,
};

static ssize_t ad5446_read_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct ad5446_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->pwr_down);
}

static ssize_t ad5446_write_dac_powerdown(struct iio_dev *indio_dev,
					    uintptr_t private,
					    const struct iio_chan_spec *chan,
					    const char *buf, size_t len)
{
	struct ad5446_state *st = iio_priv(indio_dev);
	unsigned int shift;
	unsigned int val;
	bool powerdown;
	int ret;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	st->pwr_down = powerdown;

	if (st->pwr_down) {
		shift = chan->scan_type.realbits + chan->scan_type.shift;
		val = st->pwr_down_mode << shift;
	} else {
		val = st->cached_val;
	}

	ret = st->chip_info->write(st, val);
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info ad5446_ext_info_powerdown[] = {
	{
		.name = "powerdown",
		.read = ad5446_read_dac_powerdown,
		.write = ad5446_write_dac_powerdown,
		.shared = IIO_SEPARATE,		
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad5446_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode",IIO_SHARED_BY_TYPE, &ad5446_powerdown_mode_enum),
	{ },
};

#define _AD5446_CHANNEL(bits, storage, _shift, ext) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = 0, \
	.scan_index = 0, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = (bits), \
		.storagebits = (storage), \
		.shift = (_shift), \
		.endianness = IIO_BE, \
		}, \
	.ext_info = (ext), \
}

#define AD5446_CHANNEL(bits, storage, shift) \
	_AD5446_CHANNEL(bits, storage, shift, NULL)

#define AD5446_CHANNEL_POWERDOWN(bits, storage, shift) \
	_AD5446_CHANNEL(bits, storage, shift, ad5446_ext_info_powerdown)

static int ad5446_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5446_state *st = iio_priv(indio_dev);
	printk (KERN_INFO "cola read raw mask %d \n",m);
	
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = st->cached_val;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int ad5446_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad5446_state *st = iio_priv(indio_dev);
	int ret = 0;
	printk (KERN_INFO "cola write raw mask %d val 0x%x looking for 0x%x\n",mask, val,IIO_CHAN_INFO_HYSTERESIS);
	
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;
		st->cached_val = val;
		val <<= chan->scan_type.shift;
		mutex_lock(&indio_dev->mlock);
		
		if (!st->pwr_down)
			ret = st->chip_info->write(st, val);
		mutex_unlock(&indio_dev->mlock);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info ad5446_info = {
	.read_raw = ad5446_read_raw,
	.write_raw = ad5446_write_raw,
};
#if 0
static irqreturn_t ad5446_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	const struct iio_chan_spec *chan;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct ad5446_state *st = iio_priv(indio_dev);
	u8 sample[4];
	unsigned int i;
	unsigned val;
	int ret;
	return IRQ_HANDLED;
	printk(KERN_INFO "HW trigger\n");
	ret = iio_buffer_remove_sample(buffer, sample);
	if (ret < 0) { 
	/*	if(ret != -ENODATA) //This is an underrun, fills system log when not loading data to dac
			dev_err(&indio_dev->dev,
				"iio_buffer_remove_sample failed: %d", ret);*/
		goto out;
	}
	
	mutex_lock(&indio_dev->mlock);
	int j=0;
	//for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		val = (sample[1] << 8) + sample[0];
		st->cached_val = val;
	//	st->data[j] = cpu_to_be16(val);
		ret = st->chip_info->write(st, val);
	//}
	
	mutex_unlock(&indio_dev->mlock);
	
out:
	iio_trigger_notify_done(indio_dev->trig);
	
	return IRQ_HANDLED;
}
#endif
static int ad5446_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad5446_state *st = iio_priv(indio_dev);
	int ret;
	
//	printk(KERN_INFO "HW post enable\n");
	
#if 1
	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = 0;
	st->spi_transfer.tx_buf = indio_dev->buffer;
	st->spi_transfer.len = 1;
	st->spi_transfer.bits_per_word = st->num_bits;
	st->spi_transfer.bits_per_word=16;
//	st->spi_transfer.delay_usecs=2;
//	printk(KERN_INFO "Buffer 0x%x\n",st->spi_transfer.tx_buf);
	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);
//	printk(KERN_INFO "spi init ok\n");
	spi_bus_lock(st->spi->master);
	st->bus_locked = true;
//printk(KERN_INFO "spi lock ok\n");
	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;
#endif
//	printk(KERN_INFO "HW post enable ok\n");
	spi_engine_offload_enable(st->spi, true);

	return 0;
}

static int ad5446_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad5446_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi, false);
//	printk(KERN_INFO "HW post disable\n");
	st->bus_locked = false;
	return spi_bus_unlock(st->spi->master);
}
#if 0
extern bool bWaitForADCTrigger;
extern bool setTriggerBlock;
extern bool doSync;
bool bThrowOutOne=true;
struct iio_dma_buffer_queue *g_cola_queue=NULL;
struct iio_dma_buffer_block *g_cola_block=NULL;

// done in dmaengine now
static int hw_submit_block(struct iio_dma_buffer_queue *queue,
			   struct iio_dma_buffer_block *block)
{
//	block->block.bytes_used = block->block.size;
//	printk(KERN_INFO "HW submit block 0x%x size:%d\n",block->phys_addr,block->block.bytes_used );
/*	if (bWaitForADCTrigger && bThrowOutOne)		{
		bThrowOutOne=false;
		return 0;//udelay(1);	
	}*/
	int retVal;
//	printk(KERN_INFO "cola queue 0x%x",queue);
	
	if (doSync)	 {
		g_cola_queue=queue;
		g_cola_block=block;
		retVal=0;
	}
	else {
		printk(KERN_INFO "cola submit");
		while (bWaitForADCTrigger)		
			udelay(1);	
		retVal = iio_dmaengine_buffer_submit_block(queue, block);
	}
	return retVal;
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};
#endif
static const struct iio_buffer_setup_ops ad5446_buffer_ops = {
	.postenable = &ad5446_buffer_postenable,
	.postdisable = &ad5446_buffer_postdisable,
};
static const unsigned long ad5446_available_scan_masks[]  = { 0xFF, 0x00 };
static int ad5446_hardware_buffer_alloc(struct ad5446_state *st,struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	int ret;
	indio_dev->setup_ops = &ad5446_buffer_ops; 	

	indio_dev->name = "cola";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	
	indio_dev->available_scan_masks = ad5446_available_scan_masks;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent, indio_dev,
					      "tx", IIO_BUFFER_DIRECTION_OUT);
	if (ret)
		return ret;
	iio_device_set_drvdata(indio_dev,st);
	ret = devm_iio_device_register(&st->spi->dev, indio_dev);
	return ret;
} 


static int ad5446_probe(struct spi_device *spi, struct device *dev, const char *name,
			const struct ad5446_chip_info *chip_info)
{
	struct ad5446_state *st;
	struct iio_dev *indio_dev;
	struct regulator *reg;
	int ret, voltage_uv = 0;
	u32 core, val;
	struct device_node *np = spi->dev.of_node;
//	platform_device_register(&iio_gpio_trigger);
	printk(KERN_INFO "ad5446 probe %s\n",name);
	reg = devm_regulator_get(dev, "vcc");
	if (!IS_ERR(reg)) {
		ret = regulator_enable(reg);
		if (ret)
			return ret;

		ret = regulator_get_voltage(reg);
		if (ret < 0)
			goto error_disable_reg;

		voltage_uv = ret;
	}

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_disable_reg;
	}
	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = chip_info;
	spi_set_drvdata(spi, indio_dev);
//	dev_set_drvdata(dev, indio_dev);
	st->reg = reg;
	st->dev = dev;
	st->is_dma_mapped=spi_engine_offload_supported(spi);
/*	ret = of_property_read_u32(np, "core",&core);
	void __iomem *coreregs = ioremap(core,0x80);
	printk(KERN_INFO "%s dma mapped core:0x%x corereg:0x%x\n",name,core,coreregs);
	val=3;
	writel(val, coreregs + 0x40);
	val=0x1; //25kHz
	writel(val, coreregs + 0x4c);
	/* Establish that the iio_dev is a child of the device 
	indio_dev->dev.parent = dev;*/
	indio_dev->name = name;
	indio_dev->info = &ad5446_info;
	
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = &st->chip_info->channel;
	indio_dev->num_channels = 1;
	mutex_init(&st->lock);
//	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	st->pwr_down_mode = MODE_PWRDWN_1k;
	st->num_bits = indio_dev->channels->scan_type.realbits;
	if (st->chip_info->int_vref_mv)
		st->vref_mv = st->chip_info->int_vref_mv;
	else if (voltage_uv)
		st->vref_mv = voltage_uv / 1000;
	else
		dev_warn(dev, "reference voltage unspecified\n");
#if 0
	ret = iio_triggered_buffer_setup(indio_dev, NULL,
		&ad5446_trigger_handler, NULL);
#endif
	ret = ad5446_hardware_buffer_alloc(st,indio_dev);
/*	if (ret)
		goto error_disable_reg;
//	spi_set_drvdata(spi, indio_dev);
	ret = devm_iio_device_register(dev, indio_dev);
//	ret = iio_device_register(indio_dev);
	
	
	if (ret)
		goto error_disable_reg;*/
	printk(KERN_INFO "ad5446 probe done\n");
	return ret;

error_disable_reg:
	printk (KERN_WARNING "There was an error, disable regulator\n");
	
	if (!IS_ERR(reg))
		regulator_disable(reg);
	return ret;
}

static void ad5446_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5446_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

//	return 0;
}

#if IS_ENABLED(CONFIG_SPI_MASTER)

static int ad5446_write(struct ad5446_state *st, unsigned val)
{
	struct spi_device *spi = to_spi_device(st->dev);
	__be16 data ____cacheline_aligned;
	data = cpu_to_be16(val);

	return spi_write(spi, &data, sizeof(data));
}


/**
 * ad5446_supported_spi_device_ids:
 * The AD5620/40/60 parts are available in different fixed internal reference
 * voltage options. The actual part numbers may look differently
 * (and a bit cryptic), however this style is used to make clear which
 * parts are supported here.
 */
enum ad5446_supported_spi_device_ids {
	ID_AD5300,
	ID_AD5310,
	ID_AD5320,
	ID_AD5444,
	ID_AD5446,
	ID_AD5450,
	ID_AD5451,
	ID_AD5541A,
	ID_AD5512A,
	ID_AD5553,
	ID_AD5600,
	ID_AD5601,
	ID_AD5611,
	ID_AD5621,
	ID_AD5641,
	ID_AD5620_2500,
	ID_AD5620_1250,
	ID_AD5640_2500,
	ID_AD5640_1250,
	ID_AD5660_2500,
	ID_AD5660_1250,
	ID_AD5662,
};

static const struct ad5446_chip_info ad5446_spi_chip_info[] = {
	[ID_AD5300] = {
		.channel = AD5446_CHANNEL_POWERDOWN(8, 16, 4),
		.write = ad5446_write,
	},
	[ID_AD5310] = {
		.channel = AD5446_CHANNEL_POWERDOWN(10, 16, 2),
		.write = ad5446_write,
	},
	[ID_AD5320] = {
		.channel = AD5446_CHANNEL_POWERDOWN(12, 16, 0),
		.write = ad5446_write,
	},
	[ID_AD5444] = {
		.channel = AD5446_CHANNEL(12, 16, 2),
		.write = ad5446_write,
	},
	[ID_AD5446] = {
		.channel = AD5446_CHANNEL(14, 16, 0),
		.write = ad5446_write,
	},
	[ID_AD5450] = {
		.channel = AD5446_CHANNEL(8, 16, 6),
		.write = ad5446_write,
	},
	[ID_AD5451] = {
		.channel = AD5446_CHANNEL(10, 16, 4),
		.write = ad5446_write,
	},
	[ID_AD5541A] = {
		.channel = AD5446_CHANNEL(16, 32, 0),
		.write = ad5446_write,
	},
	[ID_AD5512A] = {
		.channel = AD5446_CHANNEL(12, 16, 4),
		.write = ad5446_write,
	},
	[ID_AD5553] = {
		.channel = AD5446_CHANNEL(14, 16, 0),
		.write = ad5446_write,
	},
	[ID_AD5600] = {
		.channel = AD5446_CHANNEL(16, 16, 0),
		.write = ad5446_write,
	},
	[ID_AD5601] = {
		.channel = AD5446_CHANNEL_POWERDOWN(8, 16, 6),
		.write = ad5446_write,
	},
	[ID_AD5611] = {
		.channel = AD5446_CHANNEL_POWERDOWN(10, 16, 4),
		.write = ad5446_write,
	},
	[ID_AD5621] = {
		.channel = AD5446_CHANNEL_POWERDOWN(12, 16, 2),
		.write = ad5446_write,
	},
	[ID_AD5641] = {
		.channel = AD5446_CHANNEL_POWERDOWN(14, 16, 0),
		.write = ad5446_write,
	},
	[ID_AD5620_2500] = {
		.channel = AD5446_CHANNEL_POWERDOWN(12, 16, 2),
		.int_vref_mv = 2500,
		.write = ad5446_write,
	},
	[ID_AD5620_1250] = {
		.channel = AD5446_CHANNEL_POWERDOWN(12, 16, 2),
		.int_vref_mv = 1250,
		.write = ad5446_write,
	},
	[ID_AD5640_2500] = {
		.channel = AD5446_CHANNEL_POWERDOWN(14, 16, 0),
		.int_vref_mv = 2500,
		.write = ad5446_write,
	},
	[ID_AD5640_1250] = {
		.channel = AD5446_CHANNEL_POWERDOWN(14, 16, 0),
		.int_vref_mv = 1250,
		.write = ad5446_write,
	},
	[ID_AD5660_2500] = {
		.channel = AD5446_CHANNEL_POWERDOWN(16, 16, 0),
		.int_vref_mv = 2500,
		.write = ad5446_write,
	},
	[ID_AD5660_1250] = {
		.channel = AD5446_CHANNEL_POWERDOWN(16, 16, 0),
		.int_vref_mv = 1250,
		.write = ad5446_write,
	},
	[ID_AD5662] = {
		.channel = AD5446_CHANNEL_POWERDOWN(16, 16, 0),
		.write = ad5446_write,
	},
};

static const struct spi_device_id ad5446_spi_ids[] = {
	{"ad5300", ID_AD5300},
	{"ad5310", ID_AD5310},
	{"ad5320", ID_AD5320},
	{"ad5444", ID_AD5444},
	{"ad5446", ID_AD5446},
	{"ad5450", ID_AD5450},
	{"ad5451", ID_AD5451},
	{"ad5452", ID_AD5444}, /* ad5452 is compatible to the ad5444 */
	{"ad5453", ID_AD5446}, /* ad5453 is compatible to the ad5446 */
	{"ad5512a", ID_AD5512A},
	{"ad5541a", ID_AD5541A},
	{"ad5542a", ID_AD5541A}, /* ad5541a and ad5542a are compatible */
	{"ad5542", ID_AD5541A}, /* ad5541a and ad5542a are compatible */
	{"ad5543", ID_AD5541A}, /* ad5541a and ad5543 are compatible */
	{"ad5553", ID_AD5553},
	{"ad5600", ID_AD5600},
	{"ad5601", ID_AD5601},
	{"ad5611", ID_AD5611},
	{"ad5621", ID_AD5621},
	{"ad5641", ID_AD5641},
	{"ad5620-2500", ID_AD5620_2500}, /* AD5620/40/60: */
	{"ad5620-1250", ID_AD5620_1250}, /* part numbers may look differently */
	{"ad5640-2500", ID_AD5640_2500},
	{"ad5640-1250", ID_AD5640_1250},
	{"ad5660-2500", ID_AD5660_2500},
	{"ad5660-1250", ID_AD5660_1250},
	{"ad5662", ID_AD5662},
	{"dac081s101", ID_AD5300}, /* compatible Texas Instruments chips */
	{"dac101s101", ID_AD5310},
	{"dac121s101", ID_AD5320},
	{"dac7512", ID_AD5320},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5446_spi_ids);

#ifdef CONFIG_OF
static const struct of_device_id ad5446_of_ids[] = {
	{ .compatible = "ti,dac7512" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5446_of_ids);
#endif

static int ad5446_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return ad5446_probe(spi, &spi->dev, spi->dev.of_node ?
		spi->dev.of_node->name : id->name,
		&ad5446_spi_chip_info[id->driver_data]);
}

static void ad5446_spi_remove(struct spi_device *spi)
{
	ad5446_remove(&spi->dev);
}

static struct spi_driver ad5446_spi_driver = {
	.driver = {
		.name	= "ad5446",
		.of_match_table = ad5446_of_ids,
	},
	.probe		= ad5446_spi_probe,
	.remove		= ad5446_spi_remove,
	.id_table	= ad5446_spi_ids,
};

static int __init ad5446_spi_register_driver(void)
{
	return spi_register_driver(&ad5446_spi_driver);
}

static void ad5446_spi_unregister_driver(void)
{
	spi_unregister_driver(&ad5446_spi_driver);
}

#else

static inline int ad5446_spi_register_driver(void) { return 0; }
static inline void ad5446_spi_unregister_driver(void) { }

#endif

#if IS_ENABLED(CONFIG_I2C)

static int ad5622_write(struct ad5446_state *st, unsigned val)
{
	struct i2c_client *client = to_i2c_client(st->dev);
	__be16 data = cpu_to_be16(val);

	return i2c_master_send(client, (char *)&data, sizeof(data));
}

/**
 * ad5446_supported_i2c_device_ids:
 * The AD5620/40/60 parts are available in different fixed internal reference
 * voltage options. The actual part numbers may look differently
 * (and a bit cryptic), however this style is used to make clear which
 * parts are supported here.
 */
enum ad5446_supported_i2c_device_ids {
	ID_AD5602,
	ID_AD5612,
	ID_AD5622,
};

static const struct ad5446_chip_info ad5446_i2c_chip_info[] = {
	[ID_AD5602] = {
		.channel = AD5446_CHANNEL_POWERDOWN(8, 16, 4),
		.write = ad5622_write,
	},
	[ID_AD5612] = {
		.channel = AD5446_CHANNEL_POWERDOWN(10, 16, 2),
		.write = ad5622_write,
	},
	[ID_AD5622] = {
		.channel = AD5446_CHANNEL_POWERDOWN(12, 16, 0),
		.write = ad5622_write,
	},
};

static int ad5446_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
/*	return ad5446_probe((struct spi_device *)NULL,&i2c->dev, id->name,
		&ad5446_i2c_chip_info[id->driver_data]);*/
		return 0;
}

static void ad5446_i2c_remove(struct i2c_client *i2c)
{
	ad5446_remove(&i2c->dev);
}

static const struct i2c_device_id ad5446_i2c_ids[] = {
	{"ad5301", ID_AD5602},
	{"ad5311", ID_AD5612},
	{"ad5321", ID_AD5622},
	{"ad5602", ID_AD5602},
	{"ad5612", ID_AD5612},
	{"ad5622", ID_AD5622},
	{}
};
MODULE_DEVICE_TABLE(i2c, ad5446_i2c_ids);

static struct i2c_driver ad5446_i2c_driver = {
	.driver = {
		   .name = "ad5446",
	},
	.probe = ad5446_i2c_probe,
	.remove = ad5446_i2c_remove,
	.id_table = ad5446_i2c_ids,
};

static int __init ad5446_i2c_register_driver(void)
{
	return i2c_add_driver(&ad5446_i2c_driver);
}

static void __exit ad5446_i2c_unregister_driver(void)
{
	i2c_del_driver(&ad5446_i2c_driver);
}

#else

static inline int ad5446_i2c_register_driver(void) { return 0; }
static inline void ad5446_i2c_unregister_driver(void) { }

#endif

static int __init ad5446_init(void)
{
	int ret;

	ret = ad5446_spi_register_driver();
	if (ret)
		return ret;

	ret = ad5446_i2c_register_driver();
	if (ret) {
		ad5446_spi_unregister_driver();
		return ret;
	}

	return 0;
}
module_init(ad5446_init);

static void __exit ad5446_exit(void)
{
	ad5446_i2c_unregister_driver();
	ad5446_spi_unregister_driver();
}
module_exit(ad5446_exit);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5444/AD5446 DAC");
MODULE_LICENSE("GPL v2");
