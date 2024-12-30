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
#include <linux/iio/sw_device.h>

#include "xsynch.h" 
extern struct gpio_descs *synchgpio;
struct iio_synch_state {
	int dac_val;
	struct mutex lock;
};

static const struct config_item_type iio_synch_type = {
	.ct_owner = THIS_MODULE,
};
XSynch Synch;

static ssize_t getSync(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf) {
	unsigned long val,len;
	if (synchgpio) {
		gpiod_get_array_value_cansleep(synchgpio->ndescs, synchgpio->desc, synchgpio->info, (unsigned long *)&val);
		val = val&0x7f;
		printk (KERN_INFO "read out synchgpio 0x%x\n",val);			
	}
	else {
		val=0;
		printk (KERN_INFO "no out synchgpio\n",val);	
	}
	len = sprintf(buf, "%d\0", val)+1;
	printk(KERN_INFO "get synchgpio %d from %s len %d",val,buf,len);
	return len;
}

static ssize_t setSync(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len) {
	unsigned long current_val,val;
	kstrtol(buf,10,&val);
	printk(KERN_INFO "set synchgpio %d from %s",val,buf);
	if (synchgpio) {
		gpiod_set_array_value_cansleep(synchgpio->ndescs,synchgpio->desc, synchgpio->info, (unsigned long *)&val);
		printk (KERN_INFO "write out synchgpio 0x%x\n",val);
	}
	else printk(KERN_INFO "no synchgpio to write\n");
	return len;
}


static struct iio_chan_spec_ext_info sync_ext_info[] = {
	{
		.name="synch_val",
		.read=getSync,
		.write=setSync,
		.shared=IIO_SEPARATE,
	},	
	{},
};

/*
 * iio_Synch_channels - Description of available channels
 *
 * This array of structures tells the IIO core about what the device
 * actually provides for a given channel.
 */
static const struct iio_chan_spec iio_synch_channels[] = {
	
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_ENABLE),
		.scan_index = -1, /* No buffer support */
	//	.ext_info = sync_ext_info,				
	//	.output = 1,
		.indexed = 1,
		.channel = 0,
		.extend_name = "sync",
	},

};

/**
 * iio_Synch_read_raw() - data read function.
 * @indio_dev:	the struct iio_dev associated with this device instance
 * @chan:	the channel whose data is to be read
 * @val:	first element of returned value (typically INT)
 * @val2:	second element of returned value (typically MICRO)
 * @mask:	what we actually want to read as per the info_mask_*
 *		in iio_chan_spec.
 */
static int iio_synch_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask)
{
	struct iio_synch_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;
	float fHWCurrent;
	u32 /*iHysteresis, */temp;
	printk(KERN_INFO "read raw synch");
	mutex_lock(&st->lock);
	
	switch (mask) {
	
	case IIO_CHAN_INFO_RAW: /* magic value - channel value read */			
	//	*val = XSynch_Get_sync(&Synch);//200;//iHWCurrent;
	//	printk (KERN_INFO "read synch val 0x%x\n",*val);
		if (synchgpio) {
			gpiod_get_array_value_cansleep(synchgpio->ndescs, synchgpio->desc, synchgpio->info, (unsigned long *)val);
			*val = *val&0xff;
			printk (KERN_INFO "read out synchgpio 0x%x\n",*val);			
		}
		else {
			*val=0;
			printk (KERN_INFO "no synchgpio\n",*val);	
		}
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_ENABLE: 
		*val = XSynch_Get_sync(&Synch);
		printk(KERN_INFO "read sync enable %d\n",*val);
		ret = IIO_VAL_INT;
		break;
	default:
			printk (KERN_INFO "IIO_CHAN_RAW read sync not requested");
			break;
			
	}		

	mutex_unlock(&st->lock);
	return ret;
}

/**
 * iio_Synch_write_raw() - data write function. 
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
static int iio_synch_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	int i;
	int ret = 0;
//	struct iio_dummy_state *st = iio_priv(indio_dev);

	switch (mask) {
		
	case IIO_CHAN_INFO_RAW:	
	//	XSynch_Set_sync(&Synch, val);
		if (synchgpio) {
			val |= 0x10;  // keep start pin pullup resistor set high
			gpiod_set_array_value_cansleep(synchgpio->ndescs,synchgpio->desc, synchgpio->info, (unsigned long *)&val);
			printk (KERN_INFO "write out synchgpio 0x%x\n",val);
		}
		else printk(KERN_INFO "no synchgpio to write\n");
		
		break;
	case IIO_CHAN_INFO_ENABLE:
		XSynch_Set_sync(&Synch,val);
		printk(KERN_INFO "write sync enable %d\n",val);
		break;
	default:
		printk (KERN_INFO "IIO_CHAN_RAW write sync not requested");
		return -EINVAL;	
	}
	return 0;
}

/*
 * Device type specific information.
 */
static const struct iio_info iio_synch_info = {
	.read_raw = &iio_synch_read_raw,
	.write_raw = &iio_synch_write_raw,

};

// XSynch init
static int hls_synch_init(void) {
	int error = XSynch_Initialize(&Synch, "Synch");
	printk(KERN_ALERT "Synch_BaseAddress=%x IsReady=%d return=%d\n",Synch.Control_BaseAddress,Synch.IsReady,error);

	/*XTacho_EnableAutoRestart(&Tacho);
	XTacho_Start(&Tacho);*/
	return error; 
}

/**
 * iio_Synch_init_device() - device instance specific init
 * @indio_dev: the iio device structure
 *
 * Most drivers have one of these to set up default values,
 * reset the device to known state etc.
 */
static int iio_synch_init_device(struct iio_dev *indio_dev)
{
//	struct iio_dummy_state *st = iio_priv(indio_dev);

	
	int Status = hls_synch_init();
	if(Status != XST_SUCCESS) {
		printk(KERN_ALERT "Synch HLS Peripheral failed./n");
		return -1;
	}
	printk(KERN_ALERT "Synch Ready!");
/*	XSynch_Set_ScaleValue(&Synch, 0);
	XSynch_Set_ZeroCross(&Synch, 0);*/
	printk(KERN_ALERT "Synch Initialized: Level=0 ");

	return 0;
}

/**
 * iio_Synch_probe() - device instance probe
 * @index: an id number for this instance.
 *
 * Arguments are bus type specific.
 * I2C: iio_dummy_probe(struct i2c_client *client,
 *                      const struct i2c_device_id *id)
 * SPI: iio_dummy_probe(struct spi_device *spi)
 */

static struct iio_sw_device *iio_synch_probe(const char *name)
{
	int ret,i=0;
	struct iio_dev *indio_dev;
	struct iio_synch_state *st; 
	struct iio_sw_device *swd;
	char cplStr[20];
	struct device *parent = NULL;
	printk(KERN_INFO "Probing Synch...\n");
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
	printk(KERN_INFO "Synch probed\n");
	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	iio_synch_init_device(indio_dev);
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
	indio_dev->channels = iio_synch_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_synch_channels);

	/*
	 * Provide device type specific interface functions and
	 * constant data.*/
	 
	indio_dev->info = &iio_synch_info;

	/* Specify that device provides sysfs type interfaces */
	indio_dev->modes = INDIO_DIRECT_MODE;

/*	ret = iio_simple_dummy_events_register(indio_dev);
	if (ret < 0)
		goto error_free_device;

/*	ret = iio_simple_dummy_configure_buffer(indio_dev);
	if (ret < 0)
		goto error_unregister_events;
	sprintf(cplStr, "synch");
	synchgpio = gpiod_get_array(swd->device, cplStr, GPIOD_OUT_LOW);
	if (IS_ERR(synchgpio)) {
		printk(KERN_INFO "unable to get sync gpio pointer err:%d\n",PTR_ERR(synchgpio));
		synchgpio = NULL;
	}*/
	ret = iio_device_register(indio_dev);
/*	if (ret < 0)
		goto error_unconfigure_buffer;*/

	iio_swd_group_init_type_name(swd, name, &iio_synch_type);
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
static int iio_synch_remove(struct iio_sw_device *swd)
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
static const struct iio_sw_device_ops iio_synch_device_ops = {
	.probe = iio_synch_probe,
	.remove = iio_synch_remove,
};

static struct iio_sw_device_type iio_synch_device = {
	.name = "synch",
	.owner = THIS_MODULE,
	.ops = &iio_synch_device_ops,
};

module_iio_sw_device_driver(iio_synch_device);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD IIO Synch driver");
MODULE_LICENSE("GPL v2");
