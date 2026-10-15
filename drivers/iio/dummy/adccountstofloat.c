// SPDX-License-Identifier: GPL-2.0
/*
 * ADC Counts-to-Float IIO platform driver
 *
 * One instance of the consolidated 8-channel Vitis HLS IP
 * (ADCCountsToFloat8) at 0x40010000, compatible "sd,adccal-constants".
 * A single probe registers all eight per-channel IIO devices
 * (gainrange0..gainrange7), so the userspace sysfs interface is unchanged
 * from the old 8-instance design.
 *
 * Register map (see ADCCountsToFloat8_regmap.md in the HLS source dir):
 *   0x010            reset (kept for compatibility, unused by HW)
 *   0x018            pointOffset (shared by all channels)
 *   0x020 + 4*ch     iCurrentRange[ch]
 *   0x040 + 4*ch     iOverloadCount[ch]  (RO)
 *   0x060 + 4*ch     OLValid[ch]         (RO)
 *   0x080 + 4*ch     gainChanged[ch]     (RO)
 *   0x0a0 + 4*ch     status[ch]          (RO)
 *   0x100 + 32*ch+4r cal_step[ch][range]   (IEEE-754 float bits)
 *   0x200 + 32*ch+4r cal_offset[ch][range] (IEEE-754 float bits)
 *
 * The exported channel-indexed helpers (Set_iCurrentRange, Set_pointOffset,
 * SetOffset, SetStep, GetStatus, ...) keep their old signatures for use by
 * other kernel modules (ad7768.c).
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADCCAL_NUM_CHAN  8
#define ADCCAL_NUM_RANGE 8

#define ADC8_REG_RESET             0x010
#define ADC8_REG_POINTOFFSET       0x018
#define ADC8_REG_ICURRENTRANGE(ch) (0x020 + 4 * (ch))
#define ADC8_REG_IOVERLOADCOUNT(ch) (0x040 + 4 * (ch))
#define ADC8_REG_OLVALID(ch)       (0x060 + 4 * (ch))
#define ADC8_REG_GAINCHANGED(ch)   (0x080 + 4 * (ch))
#define ADC8_REG_STATUS(ch)        (0x0a0 + 4 * (ch))
#define ADC8_REG_CAL_STEP(ch, r)   (0x100 + 32 * (ch) + 4 * (r))
#define ADC8_REG_CAL_OFFSET(ch, r) (0x200 + 32 * (ch) + 4 * (r))

static void __iomem *adc8_base;
static bool adc8_ready;

struct iio_adccounts_state {
	int chan_idx;
	struct mutex lock;
};

/* -------------------------------------------------------------------------
 * ext_info: cal_offset and cal_step — 8 u32 words (IEEE-754 float bits),
 * written/read as space-separated unsigned decimal values.
 * ------------------------------------------------------------------------- */
static int cal_offset_read(struct iio_dev *dev, uintptr_t priv,
			   const struct iio_chan_spec *chan, char *buf)
{
	struct iio_adccounts_state *st = iio_priv(dev);
	int len = 0, i;

	for (i = 0; i < ADCCAL_NUM_RANGE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u%c",
			ioread32(adc8_base + ADC8_REG_CAL_OFFSET(st->chan_idx, i)),
			i < ADCCAL_NUM_RANGE - 1 ? ' ' : '\n');
	return len;
}

static ssize_t cal_offset_write(struct iio_dev *dev, uintptr_t priv,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct iio_adccounts_state *st = iio_priv(dev);
	unsigned int v[ADCCAL_NUM_RANGE];
	int i, n;

	n = sscanf(buf, "%u %u %u %u %u %u %u %u",
		   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
	if (n != ADCCAL_NUM_RANGE)
		return -EINVAL;
	for (i = 0; i < ADCCAL_NUM_RANGE; i++)
		iowrite32(v[i], adc8_base + ADC8_REG_CAL_OFFSET(st->chan_idx, i));
	return len;
}

static int cal_step_read(struct iio_dev *dev, uintptr_t priv,
			 const struct iio_chan_spec *chan, char *buf)
{
	struct iio_adccounts_state *st = iio_priv(dev);
	int len = 0, i;

	for (i = 0; i < ADCCAL_NUM_RANGE; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u%c",
			ioread32(adc8_base + ADC8_REG_CAL_STEP(st->chan_idx, i)),
			i < ADCCAL_NUM_RANGE - 1 ? ' ' : '\n');
	return len;
}

static ssize_t cal_step_write(struct iio_dev *dev, uintptr_t priv,
			      const struct iio_chan_spec *chan,
			      const char *buf, size_t len)
{
	struct iio_adccounts_state *st = iio_priv(dev);
	unsigned int v[ADCCAL_NUM_RANGE];
	int i, n;

	n = sscanf(buf, "%u %u %u %u %u %u %u %u",
		   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
	if (n != ADCCAL_NUM_RANGE)
		return -EINVAL;
	for (i = 0; i < ADCCAL_NUM_RANGE; i++)
		iowrite32(v[i], adc8_base + ADC8_REG_CAL_STEP(st->chan_idx, i));
	return len;
}

static const struct iio_chan_spec_ext_info adccounts_ext_info[] = {
	{ .name = "cal_offset", .shared = IIO_SHARED_BY_ALL,
	  .read = cal_offset_read, .write = cal_offset_write },
	{ .name = "cal_step",   .shared = IIO_SHARED_BY_ALL,
	  .read = cal_step_read,   .write = cal_step_write },
	{ }
};

static const struct iio_chan_spec iio_adccounts_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW),
		.scan_index  = -1,
		.output      = 1,
		.indexed     = 1,
		.channel     = 0,
		.extend_name = "gain range",
		.ext_info    = adccounts_ext_info,
	},
};

static int iio_adccounts_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val, int *val2, long mask)
{
	struct iio_adccounts_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	if (mask == IIO_CHAN_INFO_RAW) {
		*val = ioread32(adc8_base + ADC8_REG_ICURRENTRANGE(st->chan_idx));
		ret = IIO_VAL_INT;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int iio_adccounts_write_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan,
				   int val, int val2, long mask)
{
	struct iio_adccounts_state *st = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	iowrite32(val, adc8_base + ADC8_REG_ICURRENTRANGE(st->chan_idx));
	return 0;
}

static const struct iio_info iio_adccounts_info = {
	.read_raw  = iio_adccounts_read_raw,
	.write_raw = iio_adccounts_write_raw,
};

/* -------------------------------------------------------------------------
 * Exported helpers called by other kernel modules (indexed by ADC channel).
 * Signatures unchanged from the 8-instance design.
 * ------------------------------------------------------------------------- */
void Set_pointOffset(int chan, int val)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return;
	if (adc8_ready) /* pointOffset is shared by all channels in ADCCountsToFloat8 */
		iowrite32(val, adc8_base + ADC8_REG_POINTOFFSET);
}

int Get_pointOffset(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return -ENODEV;
	if (!adc8_ready) return -ENODEV;
	return ioread32(adc8_base + ADC8_REG_POINTOFFSET);
}

int GetStatus(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return -ENODEV;
	if (!adc8_ready) return -ENODEV;
	return ioread32(adc8_base + ADC8_REG_STATUS(chan));
}

/* used by industrialio-buffer-dmaengine.c overload/gain-change collection */
int Get_OLValid(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN || !adc8_ready) return 0;
	return ioread32(adc8_base + ADC8_REG_OLVALID(chan));
}

unsigned int Get_gainChanged(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN || !adc8_ready) return 0;
	return ioread32(adc8_base + ADC8_REG_GAINCHANGED(chan));
}

unsigned int Get_iOverloadCount(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN || !adc8_ready) return 0;
	return ioread32(adc8_base + ADC8_REG_IOVERLOADCOUNT(chan));
}

void Set_iCurrentRange(int chan, int val)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return;
	if (adc8_ready)
		iowrite32(val, adc8_base + ADC8_REG_ICURRENTRANGE(chan));
}

int Get_iCurrentRange(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return -ENODEV;
	if (!adc8_ready) return -ENODEV;
	return ioread32(adc8_base + ADC8_REG_ICURRENTRANGE(chan));
}

void SetOffset(int chan, int val)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return;
	if (adc8_ready)
		iowrite32(val, adc8_base +
			  ADC8_REG_CAL_OFFSET(chan, Get_iCurrentRange(chan)));
}

int GetOffset(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return -ENODEV;
	if (!adc8_ready) return -ENODEV;
	return ioread32(adc8_base +
			ADC8_REG_CAL_OFFSET(chan, Get_iCurrentRange(chan)));
}

void SetStep(int chan, int val)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return;
	if (adc8_ready)
		iowrite32(val, adc8_base +
			  ADC8_REG_CAL_STEP(chan, Get_iCurrentRange(chan)));
}

int GetStep(int chan)
{
	if (chan < 0 || chan >= ADCCAL_NUM_CHAN) return -ENODEV;
	if (!adc8_ready) return -ENODEV;
	return ioread32(adc8_base +
			ADC8_REG_CAL_STEP(chan, Get_iCurrentRange(chan)));
}

/* -------------------------------------------------------------------------
 * Platform driver: one DT node, eight IIO devices (gainrange0..7)
 * ------------------------------------------------------------------------- */
static int adccal_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct iio_adccounts_state *st;
	void __iomem *base;
	int ch, ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	adc8_base = base;
	iowrite32(0, base + ADC8_REG_RESET);
	for (ch = 0; ch < ADCCAL_NUM_CHAN; ch++)
		iowrite32(0, base + ADC8_REG_ICURRENTRANGE(ch));
	adc8_ready = true;

	for (ch = 0; ch < ADCCAL_NUM_CHAN; ch++) {
		indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
		if (!indio_dev)
			return -ENOMEM;

		st = iio_priv(indio_dev);
		mutex_init(&st->lock);
		st->chan_idx = ch;

		indio_dev->name     = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						     "gainrange%d", ch);
		indio_dev->channels = iio_adccounts_channels;
		indio_dev->num_channels = ARRAY_SIZE(iio_adccounts_channels);
		indio_dev->info  = &iio_adccounts_info;
		indio_dev->modes = INDIO_DIRECT_MODE;

		ret = devm_iio_device_register(&pdev->dev, indio_dev);
		if (ret)
			return ret;
	}

	dev_info(&pdev->dev, "ADCcal 8-channel core ready at %p\n", base);
	return 0;
}

static int adccal_remove(struct platform_device *pdev)
{
	adc8_ready = false;
	return 0;
}

static const struct of_device_id adccal_of_match[] = {
	{ .compatible = "sd,adccal-constants" },
	{ }
};
MODULE_DEVICE_TABLE(of, adccal_of_match);

static struct platform_driver adccal_driver = {
	.probe  = adccal_probe,
	.remove = adccal_remove,
	.driver = {
		.name           = "adccal-constants",
		.of_match_table = adccal_of_match,
	},
};
module_platform_driver(adccal_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD ADC Cal Constants IIO platform driver (8-ch consolidated core)");
MODULE_LICENSE("GPL v2");
