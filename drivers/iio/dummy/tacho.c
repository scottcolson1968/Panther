// SPDX-License-Identifier: GPL-2.0
/*
 * Tacho IIO platform driver
 * AXI4-Lite IP at 0x400b0000, compatible "sd,tacho".
 * Read-only tachometer — exposes out_data via IIO_ROT RAW channel.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "xtacho.h"

struct iio_tacho_state {
	XTacho hw;
	struct mutex lock;
};

static XTacho *g_Tacho;

static const struct iio_chan_spec iio_tacho_channels[] = {
	{
		.type = IIO_ROT,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW),
		.scan_index  = -1,
		.indexed     = 1,
		.channel     = 0,
		.extend_name = "tacho",
	},
};

static int iio_tacho_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct iio_tacho_state *st = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&st->lock);
	*val = XTacho_Get_out_data(&st->hw);
	mutex_unlock(&st->lock);
	return IIO_VAL_INT;
}

static const struct iio_info iio_tacho_info = {
	.read_raw = iio_tacho_read_raw,
};

u32 readCurrentTacho(void)
{
	if (g_Tacho && g_Tacho->IsReady == XIL_COMPONENT_IS_READY)
		return XTacho_Get_out_data(g_Tacho);
	return 0;
}

static int tacho_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct iio_tacho_state *st;
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
	g_Tacho = &st->hw;

	indio_dev->name     = "tacho";
	indio_dev->channels = iio_tacho_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_tacho_channels);
	indio_dev->info  = &iio_tacho_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&pdev->dev, "Tacho ready at %p\n", base);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int tacho_remove(struct platform_device *pdev)
{
	g_Tacho = NULL;
	return 0;
}

static const struct of_device_id tacho_of_match[] = {
	{ .compatible = "sd,tacho" },
	{ }
};
MODULE_DEVICE_TABLE(of, tacho_of_match);

static struct platform_driver tacho_driver = {
	.probe  = tacho_probe,
	.remove = tacho_remove,
	.driver = {
		.name           = "tacho",
		.of_match_table = tacho_of_match,
	},
};
module_platform_driver(tacho_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD Tacho IIO platform driver");
MODULE_LICENSE("GPL v2");
