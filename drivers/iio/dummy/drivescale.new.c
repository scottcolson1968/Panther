/**
 * Copyright (c) 2011 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * A reference industrial I/O driver to illustrate the functionality available.
 *
 * There are numerous real drivers to illustrate the finer points.
 * The purpose of this driver is to provide a driver with far more comments
 * and explanatory notes than any 'real' driver would have.
 * Anyone starting out writing an IIO driver should first make sure they
 * understand all of this driver except those bits specifically marked
 * as being present to allow us to 'fake' the presence of hardware.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h> 
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/delay.h>
//#include <linux/iio/buffer.h>
#include <linux/iio/sw_device.h>
//#include "iio_simple_dummy.h"
#include "xdrivescale.h" 

struct iio_drivescale_state {
	int dac_val;
	struct mutex lock;
};

static const struct config_item_type iio_drivescale_type = {
	.ct_owner = THIS_MODULE,
};
XDrivescale DriveScale;
/**
 * struct iio_dummy_accel_calibscale - realworld to register mapping
 * @val: first value in read_raw - here integer part.
 * @val2: second value in read_raw etc - here micro part.
 * @regval: register value - magic device specific numbers.
 */
struct iio_dummy_accel_calibscale {
	int val;
	int val2;
	int regval; /* what would be written to hardware */
};

static const struct iio_dummy_accel_calibscale dummy_scales[] = {
	{ 0, 100000, 0x8 }, /* 0.000100 */
	{ 0, 133, 0x7 }, /* 0.000133 */
	{ 733, 13, 0x9 }, /* 733.000013 */
};

#if 0 /*CONFIG_IIO_SIMPLE_DUMMY_EVENTS

/*
 * simple event - triggered when value rises above
 * a threshold
 */
static const struct iio_event_spec iio_dummy_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
};

/*
 * simple step detect event - triggered when a step is detected
 */
static const struct iio_event_spec step_detect_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_NONE,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

/*
 * simple transition event - triggered when the reported running confidence
 * value rises above a threshold value
 */
static const struct iio_event_spec iio_running_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
};

/*
 * simple transition event - triggered when the reported walking confidence
 * value falls under a threshold value
 */
static const struct iio_event_spec iio_walking_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_FALLING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
};
#endif

/*
 * iio_drivescale_channels - Description of available channels
 *
 * This array of structures tells the IIO core about what the device
 * actually provides for a given channel.
 */
static const struct iio_chan_spec iio_drivescale_channels[] = {
	
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_ENABLE),
		.scan_index = -1, /* No buffer support */
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.extend_name = "drive scaler",
	},

};

/**
 * iio_drivescale_read_raw() - data read function.
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be read
 * @val:	first element of returned value (typically INT)
 * @val2:	second element of returned value (typically MICRO)
 * @mask:	what we actually want to read as per the info_mask_*
 *		in iio_chan_spec.
 */
static int iio_drivescale_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct iio_drivescale_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;
	float fHWCurrent;
	u32 /*iHysteresis, */temp;
	mutex_lock(&st->lock);
	
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = XDrivescale_Get_ZeroCross(&DriveScale);
		ret = IIO_VAL_INT;
		break;	
	case IIO_CHAN_INFO_RAW: /* magic value - channel value read */		
			*val = XDrivescale_Get_ScaleValue(&DriveScale);//200;//iHWCurrent;
			printk (KERN_INFO "read drive scale 0x%x\n",*val);
			ret = IIO_VAL_INT;
			break;

	default:
			*val = 77;//st->accel_val;
			ret = IIO_VAL_INT;
			break;
			
	}
		
/*	
	case IIO_CHAN_INFO_SCALE:		
			*val = 1;
		//	*val2 = 1000000;
			ret = IIO_VAL_INT;
			break;
			
		case IIO_ROT:
			*val = 0;
			*val2 = 1000;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			break;		
	
	
	}*/
	mutex_unlock(&st->lock);
	return ret;
}

/**
 * iio_drivescale_write_raw() - data write function.
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be written
 * @val:	first element of value to set (typically INT)
 * @val2:	second element of value to set (typically MICRO)
 * @mask:	what we actually want to write as per the info_mask_*
 *		in iio_chan_spec.
 *
 * Note that all raw writes are assumed IIO_VAL_INT and info mask elements
 * are assumed to be IIO_INT_PLUS_MICRO unless the callback write_raw_get_fmt
 * in struct iio_info is provided by the driver.
 */
static int iio_drivescale_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	int i;
	int ret = 0;
//	struct iio_dummy_state *st = iio_priv(indio_dev);

//	printk(KERN_INFO "write raw\n");
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:	
		XDrivescale_Set_ZeroCross(&DriveScale, val);
		break;
		
	case IIO_CHAN_INFO_RAW:	
		XDrivescale_Set_ScaleValue(&DriveScale, val);
	//	printk (KERN_INFO "write drive scale 0x%x\n",val);
		break;
	
	default:
		return -EINVAL;	

	}
	return 0;
}

/*
 * Device type specific information.
 */
static const struct iio_info iio_drivescale_info = {
	.read_raw = &iio_drivescale_read_raw,
	.write_raw = &iio_drivescale_write_raw,

};

// XTacho init
static int hls_drivescale_init(void) {
	int error = XDrivescale_Initialize(&DriveScale, "drivescale");
	printk(KERN_ALERT "DriveScale_BaseAddress=%x IsReady=%d return=%d\n",DriveScale.Control_BaseAddress,DriveScale.IsReady,error);

	/*XTacho_EnableAutoRestart(&Tacho);
	XTacho_Start(&Tacho);*/
	return error; 
}

/**
 * iio_drivescale_init_device() - device instance specific init
 * @indio_dev: the iio device structure
 *
 * Most drivers have one of these to set up default values,
 * reset the device to known state etc.
 */
static int iio_drivescale_init_device(struct iio_dev *indio_dev)
{
//	struct iio_dummy_state *st = iio_priv(indio_dev);

	
	int Status = hls_drivescale_init();
	if(Status != XST_SUCCESS) {
		printk(KERN_ALERT "DriveScale HLS Peripheral failed./n");
		return -1;
	}
	printk(KERN_ALERT "DriveScale Ready!");
	XDrivescale_Set_ScaleValue(&DriveScale, 0);
	XDrivescale_Set_ZeroCross(&DriveScale, 0);
	printk(KERN_ALERT "Drive Scale Initialized: Level=0 ");

	return 0;
}

/**
 * iio_drivescale_probe() - device instance probe
 * @index: an id number for this instance.
 *
 * Arguments are bus type specific.
 * I2C: iio_dummy_probe(struct i2c_client *client,
 *                      const struct i2c_device_id *id)
 * SPI: iio_dummy_probe(struct spi_device *spi)
 */
extern struct gpio_desc *green;
static struct iio_sw_device *iio_drivescale_probe(const char *name)
{
	int ret,i=0;
	struct iio_dev *indio_dev;
	struct iio_drivescale_state *st;
	struct iio_sw_device *swd;
	struct device *parent = NULL;

	/*
	 * With hardware: Set the parent device.
	 * parent = &spi->dev;
	 * parent = &client->dev;
	 */
	printk(KERN_INFO "Probing Drivescale...\n");
	swd = kzalloc(sizeof(*swd), GFP_KERNEL);
	if (!swd) {
		ret = -ENOMEM;
		goto error_kzalloc;
	}
	/*
	 * Allocate an IIO device.
	 *
	 * This structure contains all generic state
	 * information about the device instance.
	 * It also has a region (accessed by iio_priv()
	 * for chip specific state information.
	 */
	indio_dev = iio_device_alloc(parent, sizeof(*st));
	if (!indio_dev) {
		ret = -ENOMEM;
		goto error_ret;
	}
	printk(KERN_INFO "Drivescale probed\n");
	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	iio_drivescale_init_device(indio_dev);
	/*
	 * With hardware: Set the parent device.
	 * indio_dev->dev.parent = &spi->dev;
	 * indio_dev->dev.parent = &client->dev;
	 */

	 /*
	 * Make the iio_dev struct available to remove function.
	 * Bus equivalents
	 * i2c_set_clientdata(client, indio_dev);
	 * spi_set_drvdata(spi, indio_dev);
	 */
	swd->device = indio_dev;
	
	/*
	 * Set the device name.
	 *
	 * This is typically a part number and obtained from the module
	 * id table.
	 * e.g. for i2c and spi:
	 *    indio_dev->name = id->name;
	 *    indio_dev->name = spi_get_device_id(spi)->name;
	 */
	indio_dev->name = kstrdup(name, GFP_KERNEL);

	/* Provide description of available channels */
	indio_dev->channels = iio_drivescale_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_drivescale_channels);

	/*
	 * Provide device type specific interface functions and
	 * constant data.
	 */
	indio_dev->info = &iio_drivescale_info;

	/* Specify that device provides sysfs type interfaces */
	indio_dev->modes = INDIO_DIRECT_MODE;

/*	ret = iio_simple_dummy_events_register(indio_dev);
	if (ret < 0)
		goto error_free_device;

/*	ret = iio_simple_dummy_configure_buffer(indio_dev);
	if (ret < 0)
		goto error_unregister_events;*/

	ret = iio_device_register(indio_dev);
/*	if (ret < 0)
		goto error_unconfigure_buffer;*/

	iio_swd_group_init_type_name(swd, name, &iio_drivescale_type);
#if 0
	green = gpiod_get(&swd->device, "greenled", GPIOD_OUT_LOW);

	if (IS_ERR(green))
		printk(KERN_INFO "unable to get gpio pointer err:%d\n",PTR_ERR(green));
//	else {
	while (i < 1) {
/*		ssleep(1);
		gpiod_set_value(green, 1);
		ssleep(1);*/
		gpiod_set_value(green,0);
		i++;
		printk(KERN_INFO "led blink\n");
	}
//	}
#endif
	return swd;
/*error_unconfigure_buffer:
	iio_simple_dummy_unconfigure_buffer(indio_dev);
error_unregister_events:
	iio_simple_dummy_events_unregister(indio_dev);*/
error_free_device:
	iio_device_free(indio_dev);
error_ret:
	kfree(swd);
error_kzalloc:
	return ERR_PTR(ret);
}

/**
 * iio_dummy_remove() - device instance removal function
 * @swd: pointer to software IIO device abstraction
 *
 * Parameters follow those of iio_dummy_probe for buses.
 */
static int iio_drivescale_remove(struct iio_sw_device *swd)
{
	/*
	 * Get a pointer to the device instance iio_dev structure
	 * from the bus subsystem. E.g.
	 * struct iio_dev *indio_dev = i2c_get_clientdata(client);
	 * struct iio_dev *indio_dev = spi_get_drvdata(spi);
	 */
	struct iio_dev *indio_dev = swd->device;

	/* Unregister the device */
	iio_device_unregister(indio_dev);

	/* Device specific code to power down etc */

	/* Buffered capture related cleanup 
	iio_simple_dummy_unconfigure_buffer(indio_dev);*/

//	iio_simple_dummy_events_unregister(indio_dev);

	/* Free all structures */
	kfree(indio_dev->name);
	iio_device_free(indio_dev);

	return 0;
}
/**
 * module_iio_sw_device_driver() -  device driver registration
 *
 * Varies depending on bus type of the device. As there is no device
 * here, call probe directly. For information on device registration
 * i2c:
 * Documentation/i2c/writing-clients
 * spi:
 * Documentation/spi/spi-summary
 */
static const struct iio_sw_device_ops iio_drivescale_device_ops = {
	.probe = iio_drivescale_probe,
	.remove = iio_drivescale_remove,
};

static struct iio_sw_device_type iio_drivescale_device = {
	.name = "drivescale",
	.owner = THIS_MODULE,
	.ops = &iio_drivescale_device_ops,
};

module_iio_sw_device_driver(iio_drivescale_device);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD IIO drivescaler driver");
MODULE_LICENSE("GPL v2");
