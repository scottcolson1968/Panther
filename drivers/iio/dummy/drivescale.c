// SPDX-License-Identifier: GPL-2.0
/*
 * DriveScale IIO platform driver
 *
 * Wraps the Vitis HLS DriveScale IP (AXI4-Lite slave at 0x40000000) as an
 * IIO device.  Probed automatically via the device tree compatible string
 * "sd,drivescale".  Base address is ioremap'd once at probe time via
 * devm_platform_ioremap_resource.
 *
 * IIO sysfs interface (INDIO_DIRECT_MODE):
 *   in_voltage0_drive scaler_raw    - ScaleValue register (read/write)
 *   in_voltage0_drive scaler_en     - ZeroCross register  (read/write)
 *
 * Author: Scott Colson <colsons@sd-star.com>
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
#include "xdrivescale.h"

struct iio_drivescale_state {
	XDrivescale hw;
	struct mutex lock;
};

/* Module-level pointer used by exported helper functions (SetClipValue etc.)
 * called from other kernel modules in this subsystem. */
static XDrivescale *g_DriveScale;

/* -------------------------------------------------------------------------
 * IIO channel descriptors
 * ------------------------------------------------------------------------- */

#include "xdrivescale_hw.h"

/* New in DriveScale 1.5: software supplies 1/calibratedStep so the HLS core
 * multiplies instead of instantiating float dividers. */
#ifndef XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEPRECIP_BASE
#define XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEPRECIP_BASE 0x90
#define XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEPRECIP_HIGH 0x9f
#endif

#include <linux/math64.h>

/*
 * f32_recip_bits - reciprocal of an IEEE-754 single, integer math only
 * (arm32 kernel is soft-float).  Round-to-nearest, ≤1 ulp from a hardware
 * fdiv.  Zero/subnormal/inf/nan return 0, which the HLS core treats as
 * "no calibration" (it then keeps its built-in default scale).
 */
static u32 f32_recip_bits(u32 x)
{
	u32 sign = x & 0x80000000u;
	u32 exp  = (x >> 23) & 0xffu;
	u64 m    = (1u << 23) | (x & 0x7fffffu);
	u64 q, r;
	int e_r;

	if (exp == 0 || exp == 0xffu)
		return 0;
	q = div64_u64_rem(1ULL << 47, m, &r);
	if (2 * r >= m)			/* round to nearest */
		q++;
	e_r = 253 - (int)exp;
	if (q == (1ULL << 24)) {	/* power-of-two input: renormalize */
		q >>= 1;
		e_r++;
	}
	if (e_r <= 0 || e_r >= 0xff)	/* over/underflow: not reachable for cal steps */
		return sign;
	return sign | ((u32)e_r << 23) | ((u32)q & 0x7fffffu);
}

/* -------------------------------------------------------------------------
 * ext_info for calibration registers.
 * Float parameters (scale_value, calibrated_offset/step, clip_value) use
 * IEEE-754 bits as unsigned decimal so the kernel needs no floating-point.
 * ------------------------------------------------------------------------- */
#define DS_RD1(off)      ioread32(st->hw.Control_BaseAddress + (off))
#define DS_WR1(off, v)   iowrite32((v), st->hw.Control_BaseAddress + (off))

static int ds_read_u32(struct iio_dev *dev, uintptr_t offset,
		       const struct iio_chan_spec *chan, char *buf)
{
	struct iio_drivescale_state *st = iio_priv(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", DS_RD1(offset));
}

static ssize_t ds_write_u32(struct iio_dev *dev, uintptr_t offset,
			    const struct iio_chan_spec *chan,
			    const char *buf, size_t len)
{
	struct iio_drivescale_state *st = iio_priv(dev);
	unsigned int v;
	if (kstrtouint(buf, 0, &v))
		return -EINVAL;
	DS_WR1(offset, v);
	return len;
}

static int ds_read_u32x4(struct iio_dev *dev, uintptr_t base,
			 const struct iio_chan_spec *chan, char *buf)
{
	struct iio_drivescale_state *st = iio_priv(dev);
	int i, len = 0;
	for (i = 0; i < 4; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u%c",
			DS_RD1(base + i * 4), i < 3 ? ' ' : '\n');
	return len;
}

static ssize_t ds_write_u32x4(struct iio_dev *dev, uintptr_t base,
			      const struct iio_chan_spec *chan,
			      const char *buf, size_t len)
{
	struct iio_drivescale_state *st = iio_priv(dev);
	unsigned int v[4];
	int i, n;
	n = sscanf(buf, "%u %u %u %u", &v[0], &v[1], &v[2], &v[3]);
	if (n != 4)
		return -EINVAL;
	for (i = 0; i < 4; i++)
		DS_WR1(base + i * 4, v[i]);
	/* DriveScale 1.5: keep the reciprocal bank in step with calibratedStep */
	if (base == XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE)
		for (i = 0; i < 4; i++)
			DS_WR1(XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEPRECIP_BASE + i * 4,
			       f32_recip_bits(v[i]));
	return len;
}

static const struct iio_chan_spec_ext_info drivescale_ext_info[] = {
	{ .name = "scale_value",
	  .read  = ds_read_u32,  .write = ds_write_u32,
	  .private = XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA,
	  .shared = IIO_SHARED_BY_ALL },
	{ .name = "current_filter",
	  .read  = ds_read_u32,  .write = ds_write_u32,
	  .private = XDRIVESCALE_CONTROL_ADDR_ICURRENTFILTER_DATA,
	  .shared = IIO_SHARED_BY_ALL },
	{ .name = "clip_value",
	  .read  = ds_read_u32,  .write = ds_write_u32,
	  .private = XDRIVESCALE_CONTROL_ADDR_FCLIPVALUE_DATA,
	  .shared = IIO_SHARED_BY_ALL },
	{ .name = "calibrated_offset",
	  .read  = ds_read_u32x4, .write = ds_write_u32x4,
	  .private = XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE,
	  .shared = IIO_SHARED_BY_ALL },
	{ .name = "calibrated_step",
	  .read  = ds_read_u32x4, .write = ds_write_u32x4,
	  .private = XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE,
	  .shared = IIO_SHARED_BY_ALL },
	{ .name = "offset_counts",
	  .read  = ds_read_u32x4, .write = ds_write_u32x4,
	  .private = XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE,
	  .shared = IIO_SHARED_BY_ALL },
	{ }
};

static const struct iio_chan_spec iio_drivescale_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) |
					   BIT(IIO_CHAN_INFO_ENABLE),
		.scan_index = -1,
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.extend_name = "drive scaler",
		.ext_info    = drivescale_ext_info,
	},
};

/* -------------------------------------------------------------------------
 * IIO read/write callbacks
 * ------------------------------------------------------------------------- */
static int iio_drivescale_read_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan,
				   int *val, int *val2, long mask)
{
	struct iio_drivescale_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = XDrivescale_Get_ZeroCross(&st->hw);
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_RAW:
		*val = XDrivescale_Get_ScaleValue(&st->hw);
		ret = IIO_VAL_INT;
		break;
	default:
		break;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int iio_drivescale_write_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int val, int val2, long mask)
{
	struct iio_drivescale_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		XDrivescale_Set_ZeroCross(&st->hw, val);
		break;
	case IIO_CHAN_INFO_RAW:
		XDrivescale_Set_ScaleValue(&st->hw, val);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct iio_info iio_drivescale_info = {
	.read_raw  = iio_drivescale_read_raw,
	.write_raw = iio_drivescale_write_raw,
};

/* -------------------------------------------------------------------------
 * Helper functions called by other kernel modules in this subsystem.
 * All guard against probe not yet having run.
 * ------------------------------------------------------------------------- */
/*
 * Last filter-path select requested, or -1. Kept so a request that arrives
 * BEFORE this driver probes is not lost: ad7768_probe applies the boot rate
 * (and so calls SetSampIndex) ~0.2 s before drivescale probes, and the guard
 * below used to drop that write silently. streamToData then sat on its
 * reset-default path until the first sampling_frequency write -- measured
 * 2026-09-10 as the stop-time DAC flush consuming only ONE zero after boot.
 * drivescale_probe() applies it.
 */
static int s_pending_samp_index = -1;

/*
 * ch9 loopback delay (DriveScale register 0xa0, added 2026-09-15).
 *
 * The loopback is the DAC sample BEFORE the interpolation FIRs, so it leads
 * the analog output by a rate-dependent amount. DriveScale now delays it by
 * this many samples so ch9 lines up with the ADC data. ad7768.c computes the
 * value from the rate and the ADC filter; the client can trim it.
 */
#define DRIVESCALE_GROUP_DELAY_OFFSET	0x88
#define DRIVESCALE_LOOP_DELAY_OFFSET 0xa0

extern void ad7768_loop_delay_retrim(void);

/* group_delay (0x88) is signed and wraps mod the 256-entry cola ring,
 * so no clamping here -- ad7768.c owns the value. */
void Set_group_delay(int val)
{
	if (g_DriveScale)
		iowrite32((u32)val,
			  (void __iomem *)(uintptr_t)g_DriveScale->Control_BaseAddress
			  + DRIVESCALE_GROUP_DELAY_OFFSET);
}
EXPORT_SYMBOL_GPL(Set_group_delay);

void Set_loopDelay(int val)
{
	if (val < 0)
		val = 0;
	if (val > 255)
		val = 255;
	if (g_DriveScale)
		iowrite32((u32)val,
			  (void __iomem *)(uintptr_t)g_DriveScale->Control_BaseAddress
			  + DRIVESCALE_LOOP_DELAY_OFFSET);
}
EXPORT_SYMBOL_GPL(Set_loopDelay);

void SetSampIndex(int val)
{
	s_pending_samp_index = val;
	/* streamToData holds the filter-select in its s_axilite 'interp' register
	 * (base 0x400D0000, offset 0x10). It must be HELD (not the pulsed DriveScale
	 * 'interp' output) or streamToData delivers only 1 DAC sample per ADC sample
	 * and the interpolation never reaches the DAC. Set it on every rate change.
	 * Guard behind g_DriveScale: streamToData's AXI-Lite is on the adc_clk domain
	 * and does not ACK until that domain is up. Writing it at early ADC probe (before
	 * DriveScale has probed) stalls the AXI bus and HANGS boot. g_DriveScale being set
	 * means DriveScale probed, so the adc_clk interconnect is alive and the write is safe. */
	if (g_DriveScale) {
		static void __iomem *st_base;
		if (!st_base)
			st_base = ioremap(0x400D0000, 0x10000);
		if (st_base)
			iowrite32(val, st_base + 0x10);
		XDrivescale_Set_samp_index(g_DriveScale, val);
	}
}

void Set_iCurrentFilter(int val)
{
	if (val < 0 || val > 3) {
		pr_warn("drivescale: filter %d out of range 0-3\n", val);
		return;
	}
	if (g_DriveScale) {
		XDrivescale_Set_iCurrentFilter(g_DriveScale, val);
		printk(KERN_INFO "filter change %d\n", val);
	} else {
		printk(KERN_INFO "filter change not ready\n");
	}
}

int Get_iCurrentFilter(void)
{
	int val;
	if (!g_DriveScale)
		return -ENODEV;
	val = XDrivescale_Get_iCurrentFilter(g_DriveScale);
	printk(KERN_INFO "Current filter %d\n", val);
	return val;
}

void SetClipValue(int val)
{
	if (g_DriveScale) {
		XDrivescale_Set_fClipValue(g_DriveScale, val);
		printk(KERN_INFO "direct write clip value 0x%x\n", val);
	}
}

int GetClipValue(void)
{
	if (!g_DriveScale)
		return -ENODEV;
	return XDrivescale_Get_fClipValue(g_DriveScale);
}

void SetDriveOffset(int val)
{
	if (g_DriveScale)
		XDrivescale_Write_calibratedOffset_Words(g_DriveScale,
			Get_iCurrentFilter(), &val, 1);
}

int GetDriveOffset(void)
{
	int val;

	if (!g_DriveScale)
		return -ENODEV;
	XDrivescale_Read_calibratedOffset_Words(g_DriveScale,
		Get_iCurrentFilter(), &val, 1);
	return val;
}

void GetDriveCounts(int counts[])
{
	int i;
	if (g_DriveScale) {
		XDrivescale_Read_OffsetCounts_Words(g_DriveScale, 0, counts, 4);
		for (i = 0; i < 4; i++)
			printk(KERN_INFO "read offset counts[%d] %d\n", i, counts[i]);
	}
}

void SetDriveStep(int val)
{
	if (g_DriveScale) {
		int filt = Get_iCurrentFilter();

		XDrivescale_Write_calibratedStep_Words(g_DriveScale, filt, &val, 1);
		/* DriveScale 1.5: matching reciprocal for the same filter slot */
		iowrite32(f32_recip_bits((u32)val),
			  g_DriveScale->Control_BaseAddress +
			  XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEPRECIP_BASE + 4 * filt);
	}
}

int GetDriveStep(void)
{
	int val;

	if (!g_DriveScale)
		return -ENODEV;
	XDrivescale_Read_calibratedStep_Words(g_DriveScale,
		Get_iCurrentFilter(), &val, 1);
	printk(KERN_INFO "direct read drive step 0x%x\n", val);
	return val;
}

void SetDriveScale(int val)
{
	if (g_DriveScale)
		XDrivescale_Set_ScaleValue(g_DriveScale, val);
}

int GetDriveScale(void)
{
	int val;
	if (!g_DriveScale)
		return -ENODEV;
	val = XDrivescale_Get_ScaleValue(g_DriveScale);
	printk(KERN_INFO "direct read drive scale 0x%x\n", val);
	return val;
}

/* -------------------------------------------------------------------------
 * Platform driver probe / remove
 * ------------------------------------------------------------------------- */
static int drivescale_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct iio_drivescale_state *st;
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

	/* Initialise hardware to safe defaults */
	XDrivescale_Set_ScaleValue(&st->hw, 0);
	XDrivescale_Set_ZeroCross(&st->hw, 0);

	/* Publish pointer for helper functions */
	g_DriveScale = &st->hw;

	/*
	 * Apply a filter-path select that arrived before we probed (see
	 * s_pending_samp_index). Safe here for the same reason SetSampIndex's
	 * guard is: the ScaleValue/ZeroCross writes just above already went
	 * through the adc_clk AXI-Lite domain, so it is up and will ACK.
	 */
	if (s_pending_samp_index >= 0) {
		SetSampIndex(s_pending_samp_index);
		dev_info(&pdev->dev, "applied deferred samp_index %d\n",
			 s_pending_samp_index);
	}

	/*
	 * ad7768 probes before us, so its boot-time Set_loopDelay() hit a NULL
	 * g_DriveScale and was dropped. Now that the register exists, ask it to
	 * program the delay again.
	 */
	ad7768_loop_delay_retrim();

	indio_dev->name     = "drivescaler";
	indio_dev->channels = iio_drivescale_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_drivescale_channels);
	indio_dev->info  = &iio_drivescale_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&pdev->dev, "DriveScale ready at %p\n", base);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int drivescale_remove(struct platform_device *pdev)
{
	g_DriveScale = NULL;
	return 0;
}

static const struct of_device_id drivescale_of_match[] = {
	{ .compatible = "sd,drivescale" },
	{ }
};
MODULE_DEVICE_TABLE(of, drivescale_of_match);

static struct platform_driver drivescale_driver = {
	.probe  = drivescale_probe,
	.remove = drivescale_remove,
	.driver = {
		.name           = "drivescale",
		.of_match_table = drivescale_of_match,
	},
};
module_platform_driver(drivescale_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD DriveScale IIO platform driver");
MODULE_LICENSE("GPL v2");
