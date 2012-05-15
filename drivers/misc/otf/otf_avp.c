/* drivers/misc/otf/otf_avp.c
 *
 * Original source by Benee (c) 2012
 *
 * Modified for the OTF by vadonka 2012
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

/* Static containers */
static unsigned int MIN_AVPFREQ = 200000;
static unsigned int MAX_AVPFREQ = 280000;
static unsigned int DEF_AVPFREQ = 240000;

/* Boot time value */
unsigned int avpfreq = 240000;

static ssize_t avpfreq_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", avpfreq);
}

/** SYSFS */
extern unsigned int nitro;
static ssize_t avpfreq_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int dataavp;

	if (nitro != 1)
	{
		if (sscanf(buf, "%d\n", &dataavp) == 1)
		{
			if (dataavp != avpfreq)
			{
				avpfreq = min(max(dataavp, MIN_AVPFREQ), MAX_AVPFREQ);
				pr_info("AVPCONTROL threshold changed to %d\n", avpfreq);
			}
		}
		else
		{
			pr_info("AVPCONTROL invalid input\n");
		}
		return size;
	}
}

static ssize_t avpcontrol_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_AVPFREQ);
}

static ssize_t avpcontrol_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_AVPFREQ);
}

static ssize_t avpcontrol_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_AVPFREQ);
}

static DEVICE_ATTR(avpfreq, S_IRUGO | S_IWUGO, avpfreq_read, avpfreq_write);
static DEVICE_ATTR(avpfreqmin, S_IRUGO , avpcontrol_min, NULL);
static DEVICE_ATTR(avpfreqmax, S_IRUGO , avpcontrol_max, NULL);
static DEVICE_ATTR(avpfreqdef, S_IRUGO , avpcontrol_def, NULL);

static struct attribute *avpcontrol_attributes[] = {
	&dev_attr_avpfreq.attr,
	&dev_attr_avpfreqmin.attr,
	&dev_attr_avpfreqmax.attr,
	&dev_attr_avpfreqdef.attr,
	NULL
};

static struct attribute_group avpcontrol_group = {
	.attrs  = avpcontrol_attributes,
};

static struct miscdevice avpcontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "avpcontrol",
};

static int __init avpcontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, avpcontrol_device.name);
	ret = misc_register(&avpcontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, avpcontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&avpcontrol_device.this_device->kobj, &avpcontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", avpcontrol_device.name);
	}
	return 0;
}

device_initcall(avpcontrol_init);
