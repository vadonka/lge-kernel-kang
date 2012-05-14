/* drivers/misc/otf/otf_maxscroffmax.c
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
static unsigned int MIN_SCROFFMAXFREQ = 324000;
static unsigned int MAX_SCROFFMAXFREQ = 816000;
static unsigned int DEF_SCROFFMAXFREQ = 324000;

/* Boot time value */
unsigned int scroffmaxfreq = 324000;

static ssize_t scroffmaxfreq_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", scroffmaxfreq);
}

static ssize_t scroffmaxfreq_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int datascoff;

	if (sscanf(buf, "%d\n", &datascoff) == 1)
	{
		if (datascoff != scroffmaxfreq)
		{
			scroffmaxfreq = min(max(datascoff, MIN_SCROFFMAXFREQ), MAX_SCROFFMAXFREQ);
			pr_info("SCROFFMAXCONTROL threshold changed to %d\n", scroffmaxfreq);
		}
	}
	else
	{
		pr_info("SCROFFMAXCONTROL invalid input\n");
	}
	return size;
}

static ssize_t scroffmaxcontrol_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_SCROFFMAXFREQ);
}

static ssize_t scroffmaxcontrol_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_SCROFFMAXFREQ);
}

static ssize_t scroffmaxcontrol_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_SCROFFMAXFREQ);
}

static DEVICE_ATTR(scroffmaxfreq, S_IRUGO | S_IWUGO, scroffmaxfreq_read, scroffmaxfreq_write);
static DEVICE_ATTR(scroffmaxfreqmin, S_IRUGO , scroffmaxcontrol_min, NULL);
static DEVICE_ATTR(scroffmaxfreqmax, S_IRUGO , scroffmaxcontrol_max, NULL);
static DEVICE_ATTR(scroffmaxfreqdef, S_IRUGO , scroffmaxcontrol_def, NULL);

static struct attribute *scroffmaxcontrol_attributes[] = {
	&dev_attr_scroffmaxfreq.attr,
	&dev_attr_scroffmaxfreqmin.attr,
	&dev_attr_scroffmaxfreqmax.attr,
	&dev_attr_scroffmaxfreqdef.attr,
	NULL
};

static struct attribute_group scroffmaxcontrol_group = {
	.attrs  = scroffmaxcontrol_attributes,
};

static struct miscdevice scroffmaxcontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "scroffmaxcontrol",
};

static int __init scroffmaxcontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, scroffmaxcontrol_device.name);
	ret = misc_register(&scroffmaxcontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, scroffmaxcontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&scroffmaxcontrol_device.this_device->kobj, &scroffmaxcontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", scroffmaxcontrol_device.name);
	}
	return 0;
}

device_initcall(scroffmaxcontrol_init);
