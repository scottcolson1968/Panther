// SPDX-License-Identifier: GPL-2.0
/*
 * Tacho Control IIO platform driver
 * AXI4-Lite IP at 0x400c0000, compatible "sd,tacho-control".
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "xtacho_control.h"

#define NEW_CMD  0x40000000
#define WR_REG1  0x0000000
#define WR_REG2  0x1000000
#define WR_REG3  0x2000000
#define WR_REG4  0x3000000
#define SETUP    0x4000000
#define START    0x5000000
#define RD_ADC   0x6000000
#define RD_RXBUF 0x7000000
#define SET_DAC  0x8000000
#define SET_SPI  0x9000000
#define SET_GPIO 0xA000000

struct iio_tacho_control_state {
	XTacho_control hw;
	bool bStarted;
	int SPIval;
	int SPIexpander;
	int DACval;
	int REG1;
	int REG2;
	int REG3;
	int Setup;
	struct mutex lock;
};

static XTacho_control *g_Tacho_control;

static inline XTacho_control *tc_hw(struct iio_dev *dev)
{
	return &((struct iio_tacho_control_state *)iio_priv(dev))->hw;
}

/* ---------- start/stop enum ---------- */
static const char * const start_counter_enum[] = { "STOP", "START" };

static int set_start_counter(struct iio_dev *dev,
			     const struct iio_chan_spec *chan, unsigned int type)
{
	struct iio_tacho_control_state *st = iio_priv(dev);

	st->bStarted = !!type;
	XTacho_control_Set_in_data_i(tc_hw(dev), NEW_CMD + START + type);
	return 0;
}

static int get_start_counter(struct iio_dev *dev,
			     const struct iio_chan_spec *chan)
{
	return ((struct iio_tacho_control_state *)iio_priv(dev))->bStarted;
}

static const struct iio_enum start_counter_iio_enum = {
	.items     = start_counter_enum,
	.num_items = ARRAY_SIZE(start_counter_enum),
	.get       = get_start_counter,
	.set       = set_start_counter,
};

/* ---------- ext_info handlers ---------- */
static int readREG1(struct iio_dev *indio_dev, uintptr_t private,
		    const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->REG1);
}
static ssize_t writeREG1(struct iio_dev *indio_dev, uintptr_t private,
			 const struct iio_chan_spec *chan,
			 const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->REG1 = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + WR_REG1 + val);
	return len;
}

static int readREG2(struct iio_dev *indio_dev, uintptr_t private,
		    const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->REG2);
}
static ssize_t writeREG2(struct iio_dev *indio_dev, uintptr_t private,
			 const struct iio_chan_spec *chan,
			 const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->REG2 = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + WR_REG2 + val);
	return len;
}

static int readREG3(struct iio_dev *indio_dev, uintptr_t private,
		    const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->REG3);
}
static ssize_t writeREG3(struct iio_dev *indio_dev, uintptr_t private,
			 const struct iio_chan_spec *chan,
			 const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->REG3 = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + WR_REG3 + val);
	return len;
}

static int getSETUP(struct iio_dev *indio_dev, uintptr_t private,
		    const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->Setup);
}
static ssize_t setSETUP(struct iio_dev *indio_dev, uintptr_t private,
			const struct iio_chan_spec *chan,
			const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->Setup = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + SETUP + val);
	return len;
}

static int getSPIinit(struct iio_dev *indio_dev, uintptr_t private,
		      const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->SPIval);
}
static ssize_t setSPIinit(struct iio_dev *indio_dev, uintptr_t private,
			  const struct iio_chan_spec *chan,
			  const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->SPIval = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + SET_SPI + val);
	return len;
}

static int getExpander(struct iio_dev *indio_dev, uintptr_t private,
		       const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->SPIexpander);
}
static ssize_t setExpander(struct iio_dev *indio_dev, uintptr_t private,
			   const struct iio_chan_spec *chan,
			   const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->SPIexpander = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + SET_GPIO + val);
	return len;
}

static int getDAC(struct iio_dev *indio_dev, uintptr_t private,
		  const struct iio_chan_spec *chan, char *buf)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->DACval);
}
static ssize_t setDAC(struct iio_dev *indio_dev, uintptr_t private,
		      const struct iio_chan_spec *chan,
		      const char *buf, size_t len)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	long val;
	kstrtol(buf, 0, &val);
	st->DACval = val;
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + SET_DAC + val);
	return len;
}

static int getADC(struct iio_dev *indio_dev, uintptr_t private,
		  const struct iio_chan_spec *chan, char *buf)
{
	XTacho_control_Set_in_data_i(tc_hw(indio_dev), NEW_CMD + RD_ADC);
	return sprintf(buf, "%d\n",
		       XTacho_control_Get_in_data_o(tc_hw(indio_dev)));
}

static const struct iio_chan_spec_ext_info tacho_control_ext_info[] = {
	{ .name = "Write_control_bits_REG1",  .read = readREG1,   .write = writeREG1,  .shared = IIO_SHARED_BY_ALL },
	{ .name = "Write_trigger_level_REG2", .read = readREG2,   .write = writeREG2,  .shared = IIO_SHARED_BY_ALL },
	{ .name = "Write_avg_period_REG3",    .read = readREG3,   .write = writeREG3,  .shared = IIO_SHARED_BY_ALL },
	{ .name = "Setup",                    .read = getSETUP,   .write = setSETUP,   .shared = IIO_SHARED_BY_ALL },
	{ .name = "SPI_init_code",            .read = getSPIinit, .write = setSPIinit, .shared = IIO_SHARED_BY_ALL },
	{ .name = "Set_Expander_port_code",   .read = getExpander,.write = setExpander,.shared = IIO_SHARED_BY_ALL },
	{ .name = "Write_DAC_Value",          .read = getDAC,     .write = setDAC,     .shared = IIO_SHARED_BY_ALL },
	{ .name = "Read_ADC",                 .read = getADC,                          .shared = IIO_SHARED_BY_ALL },
	IIO_ENUM("start_counter", IIO_SHARED_BY_ALL, &start_counter_iio_enum),
	IIO_ENUM_AVAILABLE("start_counter", IIO_SHARED_BY_ALL, &start_counter_iio_enum),
	{ }
};

static const struct iio_chan_spec iio_tacho_control_channels[] = {
	{
		.type = IIO_ROT,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW),
		.scan_index  = -1,
		.ext_info    = tacho_control_ext_info,
		.indexed     = 1,
		.channel     = 0,
		.extend_name = "tacho_control",
	},
};

static int iio_tacho_control_read_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2, long mask)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	if (mask == IIO_CHAN_INFO_RAW) {
		*val = XTacho_control_Get_in_data_o(&st->hw);
		ret = IIO_VAL_INT;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int iio_tacho_control_write_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int val, int val2, long mask)
{
	struct iio_tacho_control_state *st = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	XTacho_control_Set_in_data_i(&st->hw, val);
	return 0;
}

static const struct iio_info iio_tacho_control_info = {
	.read_raw  = iio_tacho_control_read_raw,
	.write_raw = iio_tacho_control_write_raw,
};

static int tacho_control_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct iio_tacho_control_state *st;
	void __iomem *base;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);
	st->hw.Control_BaseAddress = base;
	st->hw.IsReady = XIL_COMPONENT_IS_READY;
	g_Tacho_control = &st->hw;

	/* Power-on defaults */
	st->REG1 = 0x1bb;
	st->REG2 = 0xa6;
	st->REG3 = 1;

	indio_dev->name     = "tacho_control";
	indio_dev->channels = iio_tacho_control_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_tacho_control_channels);
	indio_dev->info  = &iio_tacho_control_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&pdev->dev, "Tacho control ready at %p\n", base);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int tacho_control_remove(struct platform_device *pdev)
{
	g_Tacho_control = NULL;
	return 0;
}

static const struct of_device_id tacho_control_of_match[] = {
	{ .compatible = "sd,tacho-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, tacho_control_of_match);

static struct platform_driver tacho_control_driver = {
	.probe  = tacho_control_probe,
	.remove = tacho_control_remove,
	.driver = {
		.name           = "tacho-control",
		.of_match_table = tacho_control_of_match,
	},
};
module_platform_driver(tacho_control_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD Tacho Control IIO platform driver");
MODULE_LICENSE("GPL v2");
