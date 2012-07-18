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

/* Static containers
 * avpfreq values are set in the init/otfinit.c
 */
extern unsigned int MIN_AVPFREQ;
extern unsigned int MAX_AVPFREQ;

/* Boot time value taken from the kernel cmdline */
extern unsigned int avpfreq;

static ssize_t avpinfo_boot(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", avpfreq);
}

static ssize_t avpinfo_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_AVPFREQ);
}

static ssize_t avpinfo_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_AVPFREQ);
}

static DEVICE_ATTR(avpfreq, S_IRUGO , avpinfo_boot, NULL);
static DEVICE_ATTR(avpfreqmin, S_IRUGO , avpinfo_min, NULL);
static DEVICE_ATTR(avpfreqmax, S_IRUGO , avpinfo_max, NULL);

static struct attribute *avpinfo_attributes[] = {
	&dev_attr_avpfreq.attr,
	&dev_attr_avpfreqmin.attr,
	&dev_attr_avpfreqmax.attr,
	NULL
};

static struct attribute_group avpinfo_group = {
	.attrs  = avpinfo_attributes,
};

static struct miscdevice avpinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "avpinfo",
};

static int __init avpinfo_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, avpinfo_device.name);
	ret = misc_register(&avpinfo_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, avpinfo_device.name);
		return 1;
	}

	if (sysfs_create_group(&avpinfo_device.this_device->kobj, &avpinfo_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", avpinfo_device.name);
	}
	return 0;
}

device_initcall(avpinfo_init);
