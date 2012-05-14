/* drivers/misc/otf/otf_vde.c
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
static unsigned int MIN_VDEFREQ = 600000;
static unsigned int MAX_VDEFREQ = 700000;
static unsigned int DEF_VDEFREQ = 650000;

/* Boot time value */
unsigned int vdefreq = 650000;

static ssize_t vdefreq_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", vdefreq);
}

extern unsigned int nitro;
static ssize_t vdefreq_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int datavde;

	if (sscanf(buf, "%d\n", &datavde) == 1)
	{
		if (datavde != vdefreq)
		{
			if (nitro == 1)
			{
				vdefreq = MAX_VDEFREQ;
				pr_info("NITRO Enabled! VDECONTROL threshold changed to %d\n", vdefreq);
			}
			else
			{
				vdefreq = min(max(datavde, MIN_VDEFREQ), MAX_VDEFREQ);
				pr_info("VDECONTROL threshold changed to %d\n", vdefreq);
			}
		}
	}
	else
	{
		pr_info("VDECONTROL invalid input\n");
	}
	return size;
}

static ssize_t vdecontrol_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_VDEFREQ);
}

static ssize_t vdecontrol_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_VDEFREQ);
}

static ssize_t vdecontrol_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_VDEFREQ);
}

static DEVICE_ATTR(vdefreq, S_IRUGO | S_IWUGO, vdefreq_read, vdefreq_write);
static DEVICE_ATTR(vdefreqmin, S_IRUGO , vdecontrol_min, NULL);
static DEVICE_ATTR(vdefreqmax, S_IRUGO , vdecontrol_max, NULL);
static DEVICE_ATTR(vdefreqdef, S_IRUGO , vdecontrol_def, NULL);

static struct attribute *vdecontrol_attributes[] = {
	&dev_attr_vdefreq.attr,
	&dev_attr_vdefreqmin.attr,
	&dev_attr_vdefreqmax.attr,
	&dev_attr_vdefreqdef.attr,
	NULL
};

static struct attribute_group vdecontrol_group = {
	.attrs  = vdecontrol_attributes,
};

static struct miscdevice vdecontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vdecontrol",
};

static int __init vdecontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, vdecontrol_device.name);
	ret = misc_register(&vdecontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, vdecontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&vdecontrol_device.this_device->kobj, &vdecontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", vdecontrol_device.name);
	}
	return 0;
}

device_initcall(vdecontrol_init);
