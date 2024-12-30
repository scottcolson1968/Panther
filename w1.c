// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2004 Evgeniy Polyakov <zbr@ioremap.net>
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/hwmon.h>
#include <linux/of.h>

#include <linux/atomic.h>

#include "w1_internal.h"
#include "w1_netlink.h"

#define W1_FAMILY_DEFAULT	0

int w1_timeout = 3600; // every hour
module_param_named(timeout, w1_timeout, int, 0);
MODULE_PARM_DESC(timeout, "time in seconds between automatic slave searches");

static int w1_timeout_us = 50000;
module_param_named(timeout_us, w1_timeout_us, int, 0);
MODULE_PARM_DESC(timeout_us,
		 "time in microseconds between automatic slave searches");

/* A search stops when w1_max_slave_count devices have been found in that
 * search.  The next search will start over and detect the same set of devices
 * on a static 1-wire bus.  Memory is not allocated based on this number, just
 * on the number of devices known to the kernel.  Having a high number does not
 * consume additional resources.  As a special case, if there is only one
 * device on the network and w1_max_slave_count is set to 1, the device id can
 * be read directly skipping the normal slower search process.
 */
int w1_max_slave_count = 1;
module_param_named(max_slave_count, w1_max_slave_count, int, 0);
MODULE_PARM_DESC(max_slave_count,
	"maximum number of slaves detected in a search");

int w1_max_slave_ttl = 1;
module_param_named(slave_ttl, w1_max_slave_ttl, int, 0);
MODULE_PARM_DESC(slave_ttl,
	"Number of searches not seeing a slave before it will be removed");

DEFINE_MUTEX(w1_mlock);
LIST_HEAD(w1_masters);

static int w1_master_match(struct device *dev, struct device_driver *drv)
{
	return 1;
}

static int w1_master_probe(struct device *dev)
{
	return -ENODEV;
}

static void w1_master_release(struct device *dev)
{
	struct w1_master *md = dev_to_w1_master(dev);

	dev_dbg(dev, "%s: Releasing %s.\n", __func__, md->name);
	memset(md, 0, sizeof(struct w1_master) + sizeof(struct w1_bus_master));
	kfree(md);
}

static void w1_slave_release(struct device *dev)
{
	struct w1_slave *sl = dev_to_w1_slave(dev);

	dev_dbg(dev, "%s: Releasing %s [%p]\n", __func__, sl->name, sl);

	w1_family_put(sl->family);
	sl->master->slave_count--;
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(dev);

	return sprintf(buf, "%s\n", sl->name);
}
static DEVICE_ATTR_RO(name);

static ssize_t id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(dev);
	ssize_t count = sizeof(sl->reg_num);

	memcpy(buf, (u8 *)&sl->reg_num, count);
	return count;
}
static DEVICE_ATTR_RO(id);

static struct attribute *w1_slave_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_id.attr,
	NULL,
};
ATTRIBUTE_GROUPS(w1_slave);

/* Default family */

static ssize_t rw_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *bin_attr, char *buf, loff_t off,
			size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);

	mutex_lock(&sl->master->mutex);
	if (w1_reset_select_slave(sl)) {
		count = 0;
		goto out_up;
	}

	w1_write_block(sl->master, buf, count);
out_up:
	mutex_unlock(&sl->master->mutex);
	return count;
}

static ssize_t rw_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr, char *buf, loff_t off,
		       size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 wrbuf[3],tempbuf[8],status;
	int i,iManufacturerID,iModel,iSerialNumber,iVersionNumber,iCalDays,iPolarity,iDirection;
	unsigned uSensitivityVal,uRefFreqVal,uCutOffVal,uPositionID;
//	float fSensitivityVal;
	char cVersionLetter;
	int currentCount=0;
	mutex_lock(&sl->master->mutex);
/*	 wrbuf[0]=0xcc;	//skip rom
	 w1_write_block(sl->master,wrbuf,1);*/
//	w1_reset_select_slave(sl);
/*	wrbuf[0]=0xcc; //skip rom
	w1_reset_bus(sl->master);
	w1_write_8(sl->master,wrbuf[0]);
	wrbuf[0]=0x66; // read status register
	w1_write_8(sl->master,wrbuf[0]);
	wrbuf[1]=0x00;
	w1_write_8(sl->master,wrbuf[1]);
	status=w1_read_8(sl->master);
/*	w1_write_block(sl->master,wrbuf,2);
        w1_read_block(sl->master, buf, 1);*/
	w1_reset_bus(sl->master);
/*	if (status & 0x3) printk(KERN_INFO "applcation register not locked 0x%x\n",status);
	else {
		printk(KERN_INFO "application register locked 0x%x\n",status);*/
	wrbuf[0]=0xcc; //skip rom
        w1_write_8(sl->master,wrbuf[0]);

	wrbuf[0]=0xc3; //read app register
        wrbuf[1]=0x0;
	wrbuf[2]=0x0;
	for (i=0;i<count;i++) buf[i]=0xff;
        w1_write_block(sl->master,wrbuf,2);
	w1_read_block(sl->master, buf, 8); //read application register (basic TEDS)
/*	for (i=1; i<8; i++)
		buf[i] = tempbuf[i-1];	// data is offest by 1
	buf[0] = tempbuf[7];*/
	if (buf[0]==22 || buf[0]==23) { //Basic TEDS is stored in app register so include it
	//	printk(KERN_INFO "Basic TEDS stored in app register %d %d %d %d %d %d %d %d\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]); 
		buf+=8; //increment data buffer
		currentCount=8; //decrement count
//		w1_reset_bus(sl->master);

	}
	else if (buf[0]==0xff) printk(KERN_INFO "Basic TEDS stored in eeprom\n");
	else  {
		printk(KERN_INFO "Special handling triax sensor or non Endevco/PCB with data in App register");
		printk(KERN_INFO "Data stored in app register %d %d %d %d %d %d %d %d\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	}
	w1_reset_bus(sl->master);
	wrbuf[0]=0xcc; //skip rom
        w1_write_8(sl->master,wrbuf[0]);
//	w1_reset_bus(sl->master);
	wrbuf[0]=0xf0; //read eeprom
	wrbuf[1]=0x0;
	wrbuf[2]=0x0;
	w1_write_block(sl->master,wrbuf,3);
//	buf+=8;
	w1_read_block(sl->master, buf, count-currentCount);
	if (buf[8]==0xff) {//try again
			 w1_reset_bus(sl->master);
	        wrbuf[0]=0xcc; //skip rom
        	w1_write_8(sl->master,wrbuf[0]);
//      w1_reset_bus(sl->master);
	        wrbuf[0]=0xf0; //read eeprom
        	wrbuf[1]=0x0;
	        wrbuf[2]=0x0;
        	w1_write_block(sl->master,wrbuf,3);
//      buf+=8;
	        w1_read_block(sl->master, buf, count-currentCount);
	}
	mutex_unlock(&sl->master->mutex);
	if (((buf[8]>>2)&0x3F) == 0)	{ // Legacy TEDS template 0 for accelerometers
		iManufacturerID = buf[0]|((buf[1]&0xF)<<8);	// 12 bits
/*		if (!ConvertIDToManufacturer(chan, iManufacturerID[chan]))
			return false;		*/
		iModel = (buf[1]>>4)|(buf[2]<<4)|((buf[3]&0xF)<<12);	// 16 bits
		cVersionLetter = (char)(((buf[3]>>4)|((buf[4]&1)<<4))+64);	// +64 converts Chr5 to Ascii
		iVersionNumber = (buf[4]>>1)&0x3F;				// 6 bits
		iSerialNumber = (buf[4]>>7)|(buf[5]<<1)|(buf[6]<<9)|(buf[7]<<17); // 24 bits
		iCalDays = ((buf[11]&3)<<14)|(buf[10]<<6)|(buf[9]>>2);		
		uSensitivityVal = ((buf[13]&3)<<14)|(buf[12]<<6)|(buf[11]>>2);
//		fSensitivity = 100E-6*pow(1+2*0.0001,(int)uSensitivityVal);
		uRefFreqVal = ((buf[14]&3)<<6)|(buf[13]>>2);
//		fRefFreq[chan] = 10.17501895022*pow(1+2*.015,(int)uRefFreqVal);
		uCutOffVal = ((buf[15]&0x3F)<<6)|(buf[14]>>2);
//		fCutoffFreq[chan] = 0.01*pow(1+2*.001,(int)uCutOffVal);
		if (buf[15]&0x40)
			iPolarity = 1;
		else iPolarity = 0;
		iDirection = ((buf[16]&1)<<1)|(buf[15]>>7);
		uPositionID = ((buf[17]&3)<<7)|(buf[16]>>1);
	//	sprintf(sCalInitials[chan], "N/A");
		printk(KERN_INFO "Found Legacy TEDS 0:\niManufacturerID:%d\niModel:%d%c%d\niSerial:%d\nuSensitivity:%d\n",iManufacturerID,iModel,cVersionLetter,iVersionNumber,iSerialNumber,uSensitivityVal);
		
	}
	else if (((buf[8-currentCount]>>2)&0x3F) == 24)	{ // Legacy TEDS template 24 for accelerometers (not tested)
		iManufacturerID = buf[0]|((buf[1]&0xF)<<8);	// 12 bits
/*		if (!ConvertIDToManufacturer(chan, iManufacturerID))
			return false;		
*/		iModel = (buf[1]>>4)|(buf[2]<<4)|((buf[3]&0xF)<<12);	// 16 bits
		cVersionLetter = (char)(((buf[3]>>4)|((buf[4]&1)<<4))+64);	// +64 converts Chr5 to Ascii
		iVersionNumber = (buf[4]>>1)&0x3F;				// 6 bits
		iSerialNumber = (buf[4]>>7)|(buf[5]<<1)|(buf[6]<<9)|(buf[7]<<17); // 24 bits
		iCalDays = ((buf[11]&3)<<14)|(buf[10]<<6)|(buf[9]>>2);
		
		uSensitivityVal = ((buf[13]&3)<<14)|(buf[12]<<6)|(buf[11]>>2);
//		fSensitivity = 5E-7*pow(1+2*0.00015,(int)uSensitivityVal);
		uRefFreqVal = ((buf[14]&3)<<6)|(buf[13]>>2);
//		fRefFreq = 7.991035024961*pow(1+2*0.014735074616,(int)uRefFreqVal);
		uCutOffVal = ((buf[15]&0x7F)<<5)|(buf[14]>>3);
//		fCutoffFreq = 0.01*pow(1+2*.001,(int)uCutOffVal);
		if (buf[14]&0x4)
			iPolarity = 1;
		else iPolarity = -1;
		iDirection = buf[22]&3;
		uPositionID = ((buf[23]&0x3F)<<6)|((buf[22]>>2)&0x3F);
		 printk(KERN_INFO "Found Legacy TEDS 24:\niManufacturerID:%d\niModel:%d%c%d\niSerial:%d\nuSensitivity:%d\n",iManufacturerID,iModel,cVersionLetter,iVersionNumber,iSerialNumber,uSensitivityVal);
	}
	else if (((buf[8-currentCount]>>2)&0x3F) == 25)	{  // IEEE 1451.4 TEDS template 25
//		printk(KERN_INFO "TEDS template 25\n");
		// According to the spec, Basic TEDS should be stored in the application register, not in memory
		iManufacturerID = buf[0-currentCount]|((buf[1-currentCount]&0x3F)<<8);			// 14 bits
	/*	if (!ConvertIDToManufacturer(chan, iManufacturerID[chan]))
			return false;	*/
		iModel = (buf[1-currentCount]>>6)|(buf[2-currentCount]<<2)|((buf[3-currentCount]&0x1F)<<10);	// 15 bits 
		cVersionLetter = (char)(((buf[3-currentCount]>>5)|((buf[4-currentCount]&3)<<3))+64);	// +64 converts Chr5 to Ascii
		iVersionNumber = (buf[4-currentCount]>>2);						// 6 bits
		iSerialNumber = buf[5-currentCount]|(buf[6-currentCount]<<8)|(buf[7-currentCount]<<16); // 24 bits		
		if (buf[9-currentCount]&0x8)		// case 0 here indicates accelerometer
			return false;	// case 1 here indicates force transducer which we don't support
		if (buf[9-currentCount]&0x4)	// case 0 here indicates no extended functionality
			return false;	// case 1 here indicates extended funcitonality (programmable sensitivity) may need to handle this
		uSensitivityVal = ((buf[11-currentCount]&0xF)<<12)|(buf[10-currentCount]<<4)|(buf[9-currentCount]>>4);
//		fSensitivity = 5E-7*pow(1+2*0.00015,(int)uSensitivityVal);
		uCutOffVal = ((buf[12-currentCount]&0xF)<<4)|(buf[11-currentCount]>>4);
//		fCutoffFreq = 0.005*pow(1+2*.03,(int)uCutOffVal);
		iDirection = (buf[12-currentCount]>>4)&3;
		if (buf[13-currentCount]&0x10)
			iPolarity = 1;
		else iPolarity = -1;
		if (buf[13-currentCount]&0x20)		// case 0 here indicates no transfer function specified
			return false;	// case 1 here indicates transfer function follows 
		uRefFreqVal = ((buf[14-currentCount]&0x3F)<<2)|(buf[13-currentCount]>>6);
//		fRefFreq = 10.17501895022*pow(1+2*.015,(int)uRefFreqVal);
		iCalDays = ((buf[17-currentCount]&0x7)<<13)|(buf[16-currentCount]<<5)|(buf[15-currentCount]>>3);
		printk(KERN_INFO "Found TEDS template 25:\niManufacturerID:%d\niModel:%d%c%d\niSerial:%d\nuSensitivity:%d\n",iManufacturerID,iModel,cVersionLetter,iVersionNumber,iSerialNumber,uSensitivityVal);


	/*	sCalInitials[chan][0] = (char)((data[17]>>3)+64);
		sCalInitials[chan][1] = (char)((data[18]&0x1F)+64);
		sCalInitials[chan][2] = (char)(((data[18]>>5)|((data[19]&3)<<3))+64);
		sCalInitials[chan][3] = '\0';
		uPositionID[chan] = (data[20]>>6)|(data[21]<<2)|((data[22]&3)<<10);*/
	}
	else printk(KERN_INFO "No TEDS found buf[8]=0x%x\n",buf[8-currentCount]);
	return count;
}

static BIN_ATTR_RW(rw, PAGE_SIZE);

static struct bin_attribute *w1_slave_bin_attrs[] = {
	&bin_attr_rw,
	NULL,
};

static const struct attribute_group w1_slave_default_group = {
	.bin_attrs = w1_slave_bin_attrs,
};

static const struct attribute_group *w1_slave_default_groups[] = {
	&w1_slave_default_group,
	NULL,
};

static struct w1_family_ops w1_default_fops = {
	.groups		= w1_slave_default_groups,
};

static struct w1_family w1_default_family = {
	.fops = &w1_default_fops,
};

static int w1_uevent(struct device *dev, struct kobj_uevent_env *env);

static struct bus_type w1_bus_type = {
	.name = "w1",
	.match = w1_master_match,
	.uevent = w1_uevent,
};

struct device_driver w1_master_driver = {
	.name = "w1_master_driver",
	.bus = &w1_bus_type,
	.probe = w1_master_probe,
};

struct device w1_master_device = {
	.parent = NULL,
	.bus = &w1_bus_type,
	.init_name = "w1 bus master",
	.driver = &w1_master_driver,
	.release = &w1_master_release
};

static struct device_driver w1_slave_driver = {
	.name = "w1_slave_driver",
	.bus = &w1_bus_type,
};

#if 0
struct device w1_slave_device = {
	.parent = NULL,
	.bus = &w1_bus_type,
	.init_name = "w1 bus slave",
	.driver = &w1_slave_driver,
	.release = &w1_slave_release
};
#endif  /*  0  */

static ssize_t w1_master_attribute_show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%s\n", md->name);
	mutex_unlock(&md->mutex);

	return count;
}

static ssize_t w1_master_attribute_store_search(struct device * dev,
						struct device_attribute *attr,
						const char * buf, size_t count)
{
	long tmp;
	struct w1_master *md = dev_to_w1_master(dev);
	int ret;

	ret = kstrtol(buf, 0, &tmp);
	if (ret)
		return ret;

	mutex_lock(&md->mutex);
	md->search_count = tmp;
	mutex_unlock(&md->mutex);
	/* Only wake if it is going to be searching. */
	if (tmp)
		wake_up_process(md->thread);
	printk(KERN_INFO "search w1 %d",count);
	w1_timeout=0;
	return count;
}

static ssize_t w1_master_attribute_show_search(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%d\n", md->search_count);
	mutex_unlock(&md->mutex);

	return count;
}

static ssize_t w1_master_attribute_store_pullup(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	long tmp;
	struct w1_master *md = dev_to_w1_master(dev);
	int ret;

	ret = kstrtol(buf, 0, &tmp);
	if (ret)
		return ret;

	mutex_lock(&md->mutex);
	md->enable_pullup = tmp;
	mutex_unlock(&md->mutex);

	return count;
}

static ssize_t w1_master_attribute_show_pullup(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%d\n", md->enable_pullup);
	mutex_unlock(&md->mutex);

	return count;
}

static ssize_t w1_master_attribute_show_pointer(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "0x%p\n", md->bus_master);
	mutex_unlock(&md->mutex);
	return count;
}

static ssize_t w1_master_attribute_show_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count;
	count = sprintf(buf, "%d\n", w1_timeout);
	return count;
}

static ssize_t w1_master_attribute_show_timeout_us(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t count;
	count = sprintf(buf, "%d\n", w1_timeout_us);
	return count;
}

static ssize_t w1_master_attribute_store_max_slave_count(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct w1_master *md = dev_to_w1_master(dev);

	if (kstrtoint(buf, 0, &tmp) || tmp < 1)
		return -EINVAL;

	mutex_lock(&md->mutex);
	md->max_slave_count = tmp;
	/* allow each time the max_slave_count is updated */
	clear_bit(W1_WARN_MAX_COUNT, &md->flags);
	mutex_unlock(&md->mutex);

	return count;
}

static ssize_t w1_master_attribute_show_max_slave_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%d\n", md->max_slave_count);
	mutex_unlock(&md->mutex);
	return count;
}

static ssize_t w1_master_attribute_show_attempts(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%lu\n", md->attempts);
	mutex_unlock(&md->mutex);
	return count;
}

static ssize_t w1_master_attribute_show_slave_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	ssize_t count;

	mutex_lock(&md->mutex);
	count = sprintf(buf, "%d\n", md->slave_count);
	mutex_unlock(&md->mutex);
	return count;
}

static ssize_t w1_master_attribute_show_slaves(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct w1_master *md = dev_to_w1_master(dev);
	int c = PAGE_SIZE;
	struct list_head *ent, *n;
	struct w1_slave *sl = NULL;

	mutex_lock(&md->list_mutex);

	list_for_each_safe(ent, n, &md->slist) {
		sl = list_entry(ent, struct w1_slave, w1_slave_entry);

		c -= snprintf(buf + PAGE_SIZE - c, c, "%s\n", sl->name);
	}
	if (!sl)
		c -= snprintf(buf + PAGE_SIZE - c, c, "not found.\n");

	mutex_unlock(&md->list_mutex);

	return PAGE_SIZE - c;
}

static ssize_t w1_master_attribute_show_add(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int c = PAGE_SIZE;
	c -= snprintf(buf+PAGE_SIZE - c, c,
		"write device id xx-xxxxxxxxxxxx to add slave\n");
	return PAGE_SIZE - c;
}

static int w1_atoreg_num(struct device *dev, const char *buf, size_t count,
	struct w1_reg_num *rn)
{
	unsigned int family;
	unsigned long long id;
	int i;
	u64 rn64_le;

	/* The CRC value isn't read from the user because the sysfs directory
	 * doesn't include it and most messages from the bus search don't
	 * print it either.  It would be unreasonable for the user to then
	 * provide it.
	 */
	const char *error_msg = "bad slave string format, expecting "
		"ff-dddddddddddd\n";

	if (buf[2] != '-') {
		dev_err(dev, "%s", error_msg);
		return -EINVAL;
	}
	i = sscanf(buf, "%02x-%012llx", &family, &id);
	if (i != 2) {
		dev_err(dev, "%s", error_msg);
		return -EINVAL;
	}
	rn->family = family;
	rn->id = id;

	rn64_le = cpu_to_le64(*(u64 *)rn);
	rn->crc = w1_calc_crc8((u8 *)&rn64_le, 7);

#if 0
	dev_info(dev, "With CRC device is %02x.%012llx.%02x.\n",
		  rn->family, (unsigned long long)rn->id, rn->crc);
#endif

	return 0;
}

/* Searches the slaves in the w1_master and returns a pointer or NULL.
 * Note: must not hold list_mutex
 */
struct w1_slave *w1_slave_search_device(struct w1_master *dev,
	struct w1_reg_num *rn)
{
	struct w1_slave *sl;
	mutex_lock(&dev->list_mutex);
	list_for_each_entry(sl, &dev->slist, w1_slave_entry) {
		if (sl->reg_num.family == rn->family &&
				sl->reg_num.id == rn->id &&
				sl->reg_num.crc == rn->crc) {
			mutex_unlock(&dev->list_mutex);
			return sl;
		}
	}
	mutex_unlock(&dev->list_mutex);
	return NULL;
}

static ssize_t w1_master_attribute_store_add(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct w1_master *md = dev_to_w1_master(dev);
	struct w1_reg_num rn;
	struct w1_slave *sl;
	ssize_t result = count;

	if (w1_atoreg_num(dev, buf, count, &rn))
		return -EINVAL;

	mutex_lock(&md->mutex);
	sl = w1_slave_search_device(md, &rn);
	/* It would be nice to do a targeted search one the one-wire bus
	 * for the new device to see if it is out there or not.  But the
	 * current search doesn't support that.
	 */
	if (sl) {
		dev_info(dev, "Device %s already exists\n", sl->name);
		result = -EINVAL;
	} else {
		w1_attach_slave_device(md, &rn);
	}
	mutex_unlock(&md->mutex);

	return result;
}

static ssize_t w1_master_attribute_show_remove(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int c = PAGE_SIZE;
	c -= snprintf(buf+PAGE_SIZE - c, c,
		"write device id xx-xxxxxxxxxxxx to remove slave\n");
	return PAGE_SIZE - c;
}

static ssize_t w1_master_attribute_store_remove(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct w1_master *md = dev_to_w1_master(dev);
	struct w1_reg_num rn;
	struct w1_slave *sl;
	ssize_t result = count;

	if (w1_atoreg_num(dev, buf, count, &rn))
		return -EINVAL;

	mutex_lock(&md->mutex);
	sl = w1_slave_search_device(md, &rn);
	if (sl) {
		result = w1_slave_detach(sl);
		/* refcnt 0 means it was detached in the call */
		if (result == 0)
			result = count;
	} else {
		dev_info(dev, "Device %02x-%012llx doesn't exists\n", rn.family,
			(unsigned long long)rn.id);
		result = -EINVAL;
	}
	mutex_unlock(&md->mutex);

	return result;
}

#define W1_MASTER_ATTR_RO(_name, _mode)				\
	struct device_attribute w1_master_attribute_##_name =	\
		__ATTR(w1_master_##_name, _mode,		\
		       w1_master_attribute_show_##_name, NULL)

#define W1_MASTER_ATTR_RW(_name, _mode)				\
	struct device_attribute w1_master_attribute_##_name =	\
		__ATTR(w1_master_##_name, _mode,		\
		       w1_master_attribute_show_##_name,	\
		       w1_master_attribute_store_##_name)

static W1_MASTER_ATTR_RO(name, S_IRUGO);
static W1_MASTER_ATTR_RO(slaves, S_IRUGO);
static W1_MASTER_ATTR_RO(slave_count, S_IRUGO);
static W1_MASTER_ATTR_RW(max_slave_count, S_IRUGO | S_IWUSR | S_IWGRP);
static W1_MASTER_ATTR_RO(attempts, S_IRUGO);
static W1_MASTER_ATTR_RO(timeout, S_IRUGO);
static W1_MASTER_ATTR_RO(timeout_us, S_IRUGO);
static W1_MASTER_ATTR_RO(pointer, S_IRUGO);
static W1_MASTER_ATTR_RW(search, S_IRUGO | S_IWUSR | S_IWGRP);
static W1_MASTER_ATTR_RW(pullup, S_IRUGO | S_IWUSR | S_IWGRP);
static W1_MASTER_ATTR_RW(add, S_IRUGO | S_IWUSR | S_IWGRP);
static W1_MASTER_ATTR_RW(remove, S_IRUGO | S_IWUSR | S_IWGRP);

static struct attribute *w1_master_default_attrs[] = {
	&w1_master_attribute_name.attr,
	&w1_master_attribute_slaves.attr,
	&w1_master_attribute_slave_count.attr,
	&w1_master_attribute_max_slave_count.attr,
	&w1_master_attribute_attempts.attr,
	&w1_master_attribute_timeout.attr,
	&w1_master_attribute_timeout_us.attr,
	&w1_master_attribute_pointer.attr,
	&w1_master_attribute_search.attr,
	&w1_master_attribute_pullup.attr,
	&w1_master_attribute_add.attr,
	&w1_master_attribute_remove.attr,
	NULL
};

static const struct attribute_group w1_master_defattr_group = {
	.attrs = w1_master_default_attrs,
};

int w1_create_master_attributes(struct w1_master *master)
{
	return sysfs_create_group(&master->dev.kobj, &w1_master_defattr_group);
}

void w1_destroy_master_attributes(struct w1_master *master)
{
	sysfs_remove_group(&master->dev.kobj, &w1_master_defattr_group);
}

static int w1_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct w1_master *md = NULL;
	struct w1_slave *sl = NULL;
	char *event_owner, *name;
	int err = 0;

	if (dev->driver == &w1_master_driver) {
		md = container_of(dev, struct w1_master, dev);
		event_owner = "master";
		name = md->name;
	} else if (dev->driver == &w1_slave_driver) {
		sl = container_of(dev, struct w1_slave, dev);
		event_owner = "slave";
		name = sl->name;
	} else {
		dev_dbg(dev, "Unknown event.\n");
		return -EINVAL;
	}

	dev_dbg(dev, "Hotplug event for %s %s, bus_id=%s.\n",
			event_owner, name, dev_name(dev));

	if (dev->driver != &w1_slave_driver || !sl)
		goto end;

	err = add_uevent_var(env, "W1_FID=%02X", sl->reg_num.family);
	if (err)
		goto end;

	err = add_uevent_var(env, "W1_SLAVE_ID=%024LX",
			     (unsigned long long)sl->reg_num.id);
end:
	return err;
}

static int w1_family_notify(unsigned long action, struct w1_slave *sl)
{
	struct w1_family_ops *fops;
	int err;

	fops = sl->family->fops;

	if (!fops)
		return 0;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		/* if the family driver needs to initialize something... */
		if (fops->add_slave) {
			err = fops->add_slave(sl);
			if (err < 0) {
				dev_err(&sl->dev,
					"add_slave() call failed. err=%d\n",
					err);
				return err;
			}
		}
		if (fops->groups) {
			err = sysfs_create_groups(&sl->dev.kobj, fops->groups);
			if (err) {
				dev_err(&sl->dev,
					"sysfs group creation failed. err=%d\n",
					err);
				return err;
			}
		}
		if (IS_REACHABLE(CONFIG_HWMON) && fops->chip_info) {
			struct device *hwmon
				= hwmon_device_register_with_info(&sl->dev,
						"w1_slave_temp", sl,
						fops->chip_info,
						NULL);
			if (IS_ERR(hwmon)) {
				dev_warn(&sl->dev,
					 "could not create hwmon device\n");
			} else {
				sl->hwmon = hwmon;
			}
		}
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		if (IS_REACHABLE(CONFIG_HWMON) && fops->chip_info &&
			    sl->hwmon)
			hwmon_device_unregister(sl->hwmon);
		if (fops->remove_slave)
			sl->family->fops->remove_slave(sl);
		if (fops->groups)
			sysfs_remove_groups(&sl->dev.kobj, fops->groups);
		break;
	}
	return 0;
}

static int __w1_attach_slave_device(struct w1_slave *sl)
{
	int err;

	sl->dev.parent = &sl->master->dev;
	sl->dev.driver = &w1_slave_driver;
	sl->dev.bus = &w1_bus_type;
	sl->dev.release = &w1_slave_release;
	sl->dev.groups = w1_slave_groups;
	sl->dev.of_node = of_find_matching_node(sl->master->dev.of_node,
						sl->family->of_match_table);

	dev_set_name(&sl->dev, "%02x-%012llx",
		 (unsigned int) sl->reg_num.family,
		 (unsigned long long) sl->reg_num.id);
	snprintf(&sl->name[0], sizeof(sl->name),
		 "%02x-%012llx",
		 (unsigned int) sl->reg_num.family,
		 (unsigned long long) sl->reg_num.id);

	dev_dbg(&sl->dev, "%s: registering %s as %p.\n", __func__,
		dev_name(&sl->dev), sl);

	/* suppress for w1_family_notify before sending KOBJ_ADD */
	dev_set_uevent_suppress(&sl->dev, true);

	err = device_register(&sl->dev);
	if (err < 0) {
		dev_err(&sl->dev,
			"Device registration [%s] failed. err=%d\n",
			dev_name(&sl->dev), err);
		put_device(&sl->dev);
		return err;
	}
	w1_family_notify(BUS_NOTIFY_ADD_DEVICE, sl);

	dev_set_uevent_suppress(&sl->dev, false);
	kobject_uevent(&sl->dev.kobj, KOBJ_ADD);

	mutex_lock(&sl->master->list_mutex);
	list_add_tail(&sl->w1_slave_entry, &sl->master->slist);
	mutex_unlock(&sl->master->list_mutex);

	return 0;
}

int w1_attach_slave_device(struct w1_master *dev, struct w1_reg_num *rn)
{
	struct w1_slave *sl;
	struct w1_family *f;
	int err;
	struct w1_netlink_msg msg;

	sl = kzalloc(sizeof(struct w1_slave), GFP_KERNEL);
	if (!sl) {
		dev_err(&dev->dev,
			 "%s: failed to allocate new slave device.\n",
			 __func__);
		return -ENOMEM;
	}


	sl->owner = THIS_MODULE;
	sl->master = dev;
	set_bit(W1_SLAVE_ACTIVE, &sl->flags);

	memset(&msg, 0, sizeof(msg));
	memcpy(&sl->reg_num, rn, sizeof(sl->reg_num));
	atomic_set(&sl->refcnt, 1);
	atomic_inc(&sl->master->refcnt);
	dev->slave_count++;
	dev_info(&dev->dev, "Attaching one wire slave %02x.%012llx crc %02x\n",
		  rn->family, (unsigned long long)rn->id, rn->crc);

	/* slave modules need to be loaded in a context with unlocked mutex */
	mutex_unlock(&dev->mutex);
	request_module("w1-family-0x%02X", rn->family);
	mutex_lock(&dev->mutex);

	spin_lock(&w1_flock);
	f = w1_family_registered(rn->family);
	if (!f) {
		f= &w1_default_family;
		dev_info(&dev->dev, "Family %x for %02x.%012llx.%02x is not registered.\n",
			  rn->family, rn->family,
			  (unsigned long long)rn->id, rn->crc);
	}
	__w1_family_get(f);
	spin_unlock(&w1_flock);

	sl->family = f;

	err = __w1_attach_slave_device(sl);
	if (err < 0) {
		dev_err(&dev->dev, "%s: Attaching %s failed.\n", __func__,
			 sl->name);
		dev->slave_count--;
		w1_family_put(sl->family);
		atomic_dec(&sl->master->refcnt);
		kfree(sl);
		return err;
	}

	sl->ttl = dev->slave_ttl;

	memcpy(msg.id.id, rn, sizeof(msg.id));
	msg.type = W1_SLAVE_ADD;
	w1_netlink_send(dev, &msg);

	return 0;
}

int w1_unref_slave(struct w1_slave *sl)
{
	struct w1_master *dev = sl->master;
	int refcnt;
	mutex_lock(&dev->list_mutex);
	refcnt = atomic_sub_return(1, &sl->refcnt);
	if (refcnt == 0) {
		struct w1_netlink_msg msg;

		dev_dbg(&sl->dev, "%s: detaching %s [%p].\n", __func__,
			sl->name, sl);

		list_del(&sl->w1_slave_entry);

		memset(&msg, 0, sizeof(msg));
		memcpy(msg.id.id, &sl->reg_num, sizeof(msg.id));
		msg.type = W1_SLAVE_REMOVE;
		w1_netlink_send(sl->master, &msg);

		w1_family_notify(BUS_NOTIFY_DEL_DEVICE, sl);
		device_unregister(&sl->dev);
		#ifdef DEBUG
		memset(sl, 0, sizeof(*sl));
		#endif
		kfree(sl);
	}
	atomic_dec(&dev->refcnt);
	mutex_unlock(&dev->list_mutex);
	return refcnt;
}

int w1_slave_detach(struct w1_slave *sl)
{
	/* Only detach a slave once as it decreases the refcnt each time. */
	int destroy_now;
	mutex_lock(&sl->master->list_mutex);
	destroy_now = !test_bit(W1_SLAVE_DETACH, &sl->flags);
	set_bit(W1_SLAVE_DETACH, &sl->flags);
	mutex_unlock(&sl->master->list_mutex);

	if (destroy_now)
		destroy_now = !w1_unref_slave(sl);
	return destroy_now ? 0 : -EBUSY;
}

struct w1_master *w1_search_master_id(u32 id)
{
	struct w1_master *dev;
	int found = 0;

	mutex_lock(&w1_mlock);
	list_for_each_entry(dev, &w1_masters, w1_master_entry) {
		if (dev->id == id) {
			found = 1;
			atomic_inc(&dev->refcnt);
			break;
		}
	}
	mutex_unlock(&w1_mlock);

	return (found)?dev:NULL;
}

struct w1_slave *w1_search_slave(struct w1_reg_num *id)
{
	struct w1_master *dev;
	struct w1_slave *sl = NULL;
	int found = 0;

	mutex_lock(&w1_mlock);
	list_for_each_entry(dev, &w1_masters, w1_master_entry) {
		mutex_lock(&dev->list_mutex);
		list_for_each_entry(sl, &dev->slist, w1_slave_entry) {
			if (sl->reg_num.family == id->family &&
					sl->reg_num.id == id->id &&
					sl->reg_num.crc == id->crc) {
				found = 1;
				atomic_inc(&dev->refcnt);
				atomic_inc(&sl->refcnt);
				break;
			}
		}
		mutex_unlock(&dev->list_mutex);

		if (found)
			break;
	}
	mutex_unlock(&w1_mlock);

	return (found)?sl:NULL;
}

void w1_reconnect_slaves(struct w1_family *f, int attach)
{
	struct w1_slave *sl, *sln;
	struct w1_master *dev;

	mutex_lock(&w1_mlock);
	list_for_each_entry(dev, &w1_masters, w1_master_entry) {
		dev_dbg(&dev->dev, "Reconnecting slaves in device %s "
			"for family %02x.\n", dev->name, f->fid);
		mutex_lock(&dev->mutex);
		mutex_lock(&dev->list_mutex);
		list_for_each_entry_safe(sl, sln, &dev->slist, w1_slave_entry) {
			/* If it is a new family, slaves with the default
			 * family driver and are that family will be
			 * connected.  If the family is going away, devices
			 * matching that family are reconneced.
			 */
			if ((attach && sl->family->fid == W1_FAMILY_DEFAULT
				&& sl->reg_num.family == f->fid) ||
				(!attach && sl->family->fid == f->fid)) {
				struct w1_reg_num rn;

				mutex_unlock(&dev->list_mutex);
				memcpy(&rn, &sl->reg_num, sizeof(rn));
				/* If it was already in use let the automatic
				 * scan pick it up again later.
				 */
				if (!w1_slave_detach(sl))
					w1_attach_slave_device(dev, &rn);
				mutex_lock(&dev->list_mutex);
			}
		}
		dev_dbg(&dev->dev, "Reconnecting slaves in device %s "
			"has been finished.\n", dev->name);
		mutex_unlock(&dev->list_mutex);
		mutex_unlock(&dev->mutex);
	}
	mutex_unlock(&w1_mlock);
}

void w1_slave_found(struct w1_master *dev, u64 rn)
{
	struct w1_slave *sl;
	struct w1_reg_num *tmp;
	u64 rn_le = cpu_to_le64(rn);

	atomic_inc(&dev->refcnt);

	tmp = (struct w1_reg_num *) &rn;

	sl = w1_slave_search_device(dev, tmp);
	if (sl) {
		set_bit(W1_SLAVE_ACTIVE, &sl->flags);
	} else {
		if (rn && tmp->crc == w1_calc_crc8((u8 *)&rn_le, 7))
			w1_attach_slave_device(dev, tmp);
	}

	atomic_dec(&dev->refcnt);
}

/**
 * w1_search() - Performs a ROM Search & registers any devices found.
 * @dev: The master device to search
 * @search_type: W1_SEARCH to search all devices, or W1_ALARM_SEARCH
 * to return only devices in the alarmed state
 * @cb: Function to call when a device is found
 *
 * The 1-wire search is a simple binary tree search.
 * For each bit of the address, we read two bits and write one bit.
 * The bit written will put to sleep all devies that don't match that bit.
 * When the two reads differ, the direction choice is obvious.
 * When both bits are 0, we must choose a path to take.
 * When we can scan all 64 bits without having to choose a path, we are done.
 *
 * See "Application note 187 1-wire search algorithm" at www.maxim-ic.com
 *
 */
void w1_search(struct w1_master *dev, u8 search_type, w1_slave_found_callback cb)
{
	u64 last_rn, rn, tmp64;
	int i, slave_count = 0;
	int last_zero, last_device;
	int search_bit, desc_bit;
	u8  triplet_ret = 0;

	search_bit = 0;
	rn = dev->search_id;
	last_rn = 0;
	last_device = 0;
	last_zero = -1;

	desc_bit = 64;

	while ( !last_device && (slave_count++ < dev->max_slave_count) ) {
		last_rn = rn;
		rn = 0;

		/*
		 * Reset bus and all 1-wire device state machines
		 * so they can respond to our requests. 
		 *
		 * Return 0 - device(s) present, 1 - no devices present.
		 */
		mutex_lock(&dev->bus_mutex);
		if (w1_reset_bus(dev)) {
			mutex_unlock(&dev->bus_mutex);
//			printk(KERN_INFO "No devices present on the wire.\n");
			break;
		}
	//	printk(KERN_INFO "W1 slave found on wire\n");
		/* Do fast search on single slave bus */
		if (dev->max_slave_count == 1) {
			int rv;
			w1_write_8(dev, W1_READ_ROM);
			rv = w1_read_block(dev, (u8 *)&rn, 8);
			mutex_unlock(&dev->bus_mutex);
//			printk(KERN_INFO "read %d bytes0x%x\n",rv,rn);
/*			for (i=0;i<8;i++) 
				printk(KERN_INFO "%x",rn[i]);
*/			
			if (rv == 8 && rn){
//				printk(KERN_INFO "\nteds found\n");
				cb(dev, rn);
			}
			else
//			printk(KERN_INFO "No teds found\n");
			break;
		}

		/* Start the search */
		w1_write_8(dev, search_type);
		for (i = 0; i < 64; ++i) {
			/* Determine the direction/search bit */
			if (i == desc_bit)
				search_bit = 1;	  /* took the 0 path last time, so take the 1 path */
			else if (i > desc_bit)
				search_bit = 0;	  /* take the 0 path on the next branch */
			else
				search_bit = ((last_rn >> i) & 0x1);

			/* Read two bits and write one bit */
			triplet_ret = w1_triplet(dev, search_bit);

			/* quit if no device responded */
			if ( (triplet_ret & 0x03) == 0x03 )
				break;

			/* If both directions were valid, and we took the 0 path... */
			if (triplet_ret == 0)
				last_zero = i;

			/* extract the direction taken & update the device number */
			tmp64 = (triplet_ret >> 2);
			rn |= (tmp64 << i);

			if (test_bit(W1_ABORT_SEARCH, &dev->flags)) {
				mutex_unlock(&dev->bus_mutex);
				dev_dbg(&dev->dev, "Abort w1_search\n");
				return;
			}
		}
		mutex_unlock(&dev->bus_mutex);

		if ( (triplet_ret & 0x03) != 0x03 ) {
			if ((desc_bit == last_zero) || (last_zero < 0)) {
				last_device = 1;
				dev->search_id = 0;
			} else {
				dev->search_id = rn;
			}
			desc_bit = last_zero;
			cb(dev, rn);
		}

		if (!last_device && slave_count == dev->max_slave_count &&
			!test_bit(W1_WARN_MAX_COUNT, &dev->flags)) {
			/* Only max_slave_count will be scanned in a search,
			 * but it will start where it left off next search
			 * until all ids are identified and then it will start
			 * over.  A continued search will report the previous
			 * last id as the first id (provided it is still on the
			 * bus).
			 */
			dev_info(&dev->dev, "%s: max_slave_count %d reached, "
				"will continue next search.\n", __func__,
				dev->max_slave_count);
			set_bit(W1_WARN_MAX_COUNT, &dev->flags);
		}
	}
}

void w1_search_process_cb(struct w1_master *dev, u8 search_type,
	w1_slave_found_callback cb)
{
	struct w1_slave *sl, *sln;

	mutex_lock(&dev->list_mutex);
	list_for_each_entry(sl, &dev->slist, w1_slave_entry)
		clear_bit(W1_SLAVE_ACTIVE, &sl->flags);
	mutex_unlock(&dev->list_mutex);

	w1_search_devices(dev, search_type, cb);

	mutex_lock(&dev->list_mutex);
	list_for_each_entry_safe(sl, sln, &dev->slist, w1_slave_entry) {
		if (!test_bit(W1_SLAVE_ACTIVE, &sl->flags) && !--sl->ttl) {
			mutex_unlock(&dev->list_mutex);
			w1_slave_detach(sl);
			mutex_lock(&dev->list_mutex);
		}
		else if (test_bit(W1_SLAVE_ACTIVE, &sl->flags))
			sl->ttl = dev->slave_ttl;
	}
	mutex_unlock(&dev->list_mutex);

	if (dev->search_count > 0)
		dev->search_count--;
}

static void w1_search_process(struct w1_master *dev, u8 search_type)
{
	w1_search_process_cb(dev, search_type, w1_slave_found);
}

/**
 * w1_process_callbacks() - execute each dev->async_list callback entry
 * @dev: w1_master device
 *
 * The w1 master list_mutex must be held.
 *
 * Return: 1 if there were commands to executed 0 otherwise
 */
int w1_process_callbacks(struct w1_master *dev)
{
	int ret = 0;
	struct w1_async_cmd *async_cmd, *async_n;

	/* The list can be added to in another thread, loop until it is empty */
	while (!list_empty(&dev->async_list)) {
		list_for_each_entry_safe(async_cmd, async_n, &dev->async_list,
			async_entry) {
			/* drop the lock, if it is a search it can take a long
			 * time */
			mutex_unlock(&dev->list_mutex);
			async_cmd->cb(dev, async_cmd);
			ret = 1;
			mutex_lock(&dev->list_mutex);
		}
	}
	return ret;
}

int w1_process(void *data) 
{
	struct w1_master *dev = (struct w1_master *) data;
	/* As long as w1_timeout is only set by a module parameter the sleep
	 * time can be calculated in jiffies once.
	 */
	unsigned long jtime =
	  usecs_to_jiffies(w1_timeout * 1000000 + w1_timeout_us);
	/* remainder if it woke up early */
	unsigned long jremain = 0;

	for (;;) {
		 jtime = usecs_to_jiffies(w1_timeout * 1000000 + w1_timeout_us);

		if (!jremain && dev->search_count) {
			mutex_lock(&dev->mutex);
			w1_search_process(dev, W1_SEARCH);
			mutex_unlock(&dev->mutex);
		}

		mutex_lock(&dev->list_mutex);
		/* Note, w1_process_callback drops the lock while processing,
		 * but locks it again before returning.
		 */
		if (!w1_process_callbacks(dev) && jremain) {
			/* a wake up is either to stop the thread, process
			 * callbacks, or search, it isn't process callbacks, so
			 * schedule a search.
			 */
			jremain = 1;
		}

		__set_current_state(TASK_INTERRUPTIBLE);

		/* hold list_mutex until after interruptible to prevent loosing
		 * the wakeup signal when async_cmd is added.
		 */
		mutex_unlock(&dev->list_mutex);

		if (kthread_should_stop())
			break;

		/* Only sleep when the search is active. */
		if (dev->search_count) {
			if (!jremain)
				jremain = jtime;
			jremain = schedule_timeout(jremain);
		}
		else
			schedule();
	}

	atomic_dec(&dev->refcnt);

	return 0;
}

static int __init w1_init(void)
{
	int retval;

	pr_info("Driver for 1-wire Dallas network protocol.\n");

	w1_init_netlink();

	retval = bus_register(&w1_bus_type);
	if (retval) {
		pr_err("Failed to register bus. err=%d.\n", retval);
		goto err_out_exit_init;
	}

	retval = driver_register(&w1_master_driver);
	if (retval) {
		pr_err("Failed to register master driver. err=%d.\n",
			retval);
		goto err_out_bus_unregister;
	}

	retval = driver_register(&w1_slave_driver);
	if (retval) {
		pr_err("Failed to register slave driver. err=%d.\n",
			retval);
		goto err_out_master_unregister;
	}

	return 0;

#if 0
/* For undoing the slave register if there was a step after it. */
err_out_slave_unregister:
	driver_unregister(&w1_slave_driver);
#endif

err_out_master_unregister:
	driver_unregister(&w1_master_driver);

err_out_bus_unregister:
	bus_unregister(&w1_bus_type);

err_out_exit_init:
	return retval;
}

static void __exit w1_fini(void)
{
	struct w1_master *dev;

	/* Set netlink removal messages and some cleanup */
	list_for_each_entry(dev, &w1_masters, w1_master_entry)
		__w1_remove_master_device(dev);

	w1_fini_netlink();

	driver_unregister(&w1_slave_driver);
	driver_unregister(&w1_master_driver);
	bus_unregister(&w1_bus_type);
}

module_init(w1_init);
module_exit(w1_fini);

MODULE_AUTHOR("Evgeniy Polyakov <zbr@ioremap.net>");
MODULE_DESCRIPTION("Driver for 1-wire Dallas network protocol.");
MODULE_LICENSE("GPL");
