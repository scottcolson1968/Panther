// SPDX-License-Identifier: GPL-2.0
/*
 * Synch IIO platform driver
 * AXI4-Lite IP at 0x400a0000, compatible "sd,synch".
 * Exposes sync enable register via IIO and GPIO sync lines via ext_info.
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
#include <linux/gpio/consumer.h>
#include <linux/reboot.h>
#include "xsynch.h"

struct iio_synch_state {
	XSynch hw;
	struct mutex lock;
};

extern struct gpio_descs *synchgpio;
XSynch *g_pSynch;
EXPORT_SYMBOL(g_pSynch);

/*
 * Spurious-sync record (v30.20), filled in by sync_buf() in ad7768.c and read
 * by the client through the SYNC attributes below. bSpuriousSynch (the Synch
 * IP latch) only says that at least one sync edge arrived during the run;
 * these say how many, from which block on the data is suspect, and whether
 * this unit or another box on the sync line produced the edge. A spurious
 * edge also resets the AD7768 filter, so data around it is not trustworthy.
 *
 *   spurious_count        edges during the current run (cleared at its start)
 *   spurious_first_block  run block number at the first of them, -1 = none
 *   spurious_total        edges during any run since boot
 *   spurious_origin       SELF / EXTERNAL of the most recent, "none" if never
 */
unsigned int g_spurious_run;
int g_spurious_first_block = -1;
unsigned int g_spurious_total;
const char *g_spurious_origin = "none";
EXPORT_SYMBOL(g_spurious_run);
EXPORT_SYMBOL(g_spurious_first_block);
EXPORT_SYMBOL(g_spurious_total);
EXPORT_SYMBOL(g_spurious_origin);

static ssize_t rebootNow(struct iio_dev *indio_dev, uintptr_t private,
			 const struct iio_chan_spec *chan,
			 const char *buf, size_t len)
{
	char *argv[] = { "/bin/bash", "/Panther/config.sh", NULL };
	char *envp[] = { "HOME=/", "PATH=/sbin:/bin:/usr/bin", NULL };
	int ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);

	if (ret)
		pr_err("synch: failed to run config script: %d\n", ret);
	return len;
}


static int get_bRunningAcq(struct iio_dev *indio_dev, uintptr_t private,
			    const struct iio_chan_spec *chan, char *buf)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%u\n", XSynch_Get_bRunningAcq(&st->hw));
}

static ssize_t set_bRunningAcq(struct iio_dev *indio_dev, uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	XSynch_Set_bRunningAcq(&st->hw, val);
	return len;
}

static int get_bSpuriousSynch(struct iio_dev *indio_dev, uintptr_t private,
			      const struct iio_chan_spec *chan, char *buf)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	return sprintf(buf, "%u\n", XSynch_Get_bSpuriousSynch_o(&st->hw));
}

static ssize_t set_bSpuriousSynch(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	u32 val;
	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	XSynch_Set_bSpuriousSynch_i(&st->hw, val);
	return len;
}

static int get_spurious_count(struct iio_dev *indio_dev, uintptr_t private,
			      const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(g_spurious_run));
}

static int get_spurious_first_block(struct iio_dev *indio_dev, uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(g_spurious_first_block));
}

static int get_spurious_total(struct iio_dev *indio_dev, uintptr_t private,
			      const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(g_spurious_total));
}

static int get_spurious_origin(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%s\n", READ_ONCE(g_spurious_origin));
}

static struct iio_chan_spec_ext_info sync_ext_info[] = {
	{
		.name   = "reboot",
		.write  = rebootNow,
		.shared = IIO_SEPARATE,
	},
	{
		.name   = "bRunningAcq",
		.read   = get_bRunningAcq,
		.write  = set_bRunningAcq,
		.shared = IIO_SHARED_BY_ALL,
	},
	{
		.name   = "bSpuriousSynch",
		.read   = get_bSpuriousSynch,
		.write  = set_bSpuriousSynch,
		.shared = IIO_SHARED_BY_ALL,
	},
	{ .name = "spurious_count",       .read = get_spurious_count,       .shared = IIO_SHARED_BY_ALL },
	{ .name = "spurious_first_block", .read = get_spurious_first_block, .shared = IIO_SHARED_BY_ALL },
	{ .name = "spurious_total",       .read = get_spurious_total,       .shared = IIO_SHARED_BY_ALL },
	{ .name = "spurious_origin",      .read = get_spurious_origin,      .shared = IIO_SHARED_BY_ALL },
	{ }
};

static const struct iio_chan_spec iio_synch_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) |
					   BIT(IIO_CHAN_INFO_ENABLE),
		.scan_index = -1,
		.ext_info   = sync_ext_info,
		.indexed    = 1,
		.channel    = 0,
		.extend_name = "sync",
	},
};

static int iio_synch_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (synchgpio) {
			gpiod_get_array_value_cansleep(synchgpio->ndescs,
				synchgpio->desc, synchgpio->info,
				(unsigned long *)val);
			*val &= 0xff;
			printk(KERN_INFO "read out synchgpio 0x%x\n", *val);
		} else {
			*val = 0;
			printk(KERN_INFO "no synchgpio to read\n");
		}
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_ENABLE:
		*val = XSynch_Get_bufStart(&st->hw);
		ret = IIO_VAL_INT;
		break;
	default:
		break;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int iio_synch_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct iio_synch_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (synchgpio) {
			val |= 0x10;
			gpiod_set_array_value_cansleep(synchgpio->ndescs,
				synchgpio->desc, synchgpio->info,
				(unsigned long *)&val);
			printk(KERN_INFO "write out synchgpio 0x%x\n", val);
		} else {
			printk(KERN_INFO "no synchgpio to write\n");
		}
		break;
	case IIO_CHAN_INFO_ENABLE:
		XSynch_Set_bufStart(&st->hw, val);
		printk(KERN_INFO "write sync enable %d\n", val);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct iio_info iio_synch_info = {
	.read_raw  = iio_synch_read_raw,
	.write_raw = iio_synch_write_raw,
};

static int synch_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct iio_synch_state *st;
	void __iomem *base;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);
	st->hw.Control_BaseAddress = (u64)(uintptr_t)base;
	st->hw.IsReady = XIL_COMPONENT_IS_READY;
	g_pSynch = &st->hw;

	/* v29.79 Synch HLS protocol: bufStart=1 must be set so the IP arms and
	 * latches bStarted=true on the first spiSync falling edge. Without
	 * this, ap_return stays 0 and the ADC DMA never receives sync, causing
	 * the dmaengine DAC-to-ADC rendezvous to busy-wait forever. Setting
	 * bRunningAcq=0 explicitly ensures we start in setup mode. */
	XSynch_Set_bRunningAcq(&st->hw, 0);
	XSynch_Set_bufStart(&st->hw, 1);

	indio_dev->name     = "SYNC";
	indio_dev->channels = iio_synch_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_synch_channels);
	indio_dev->info  = &iio_synch_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&pdev->dev, "Synch ready at %p\n", base);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id synch_of_match[] = {
	{ .compatible = "sd,synch" },
	{ }
};
MODULE_DEVICE_TABLE(of, synch_of_match);

static struct platform_driver synch_driver = {
	.probe  = synch_probe,
	.driver = {
		.name           = "synch",
		.of_match_table = synch_of_match,
	},
};
module_platform_driver(synch_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD Synch IIO platform driver");
MODULE_LICENSE("GPL v2");
