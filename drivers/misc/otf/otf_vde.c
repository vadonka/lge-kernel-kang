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

/* Static containers
 * vdefreq values are set in the init/otfinit.c
 */
extern unsigned int MIN_VDEFREQ;
extern unsigned int MAX_VDEFREQ;

/* Boot time value taken from the kernel cmdline */
extern unsigned int vdefreq;

static ssize_t vdeinfo_boot(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", vdefreq);
}

static ssize_t vdeinfo_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_VDEFREQ);
}

static ssize_t vdeinfo_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_VDEFREQ);
}

static DEVICE_ATTR(vdefreq, S_IRUGO , vdeinfo_boot, NULL);
static DEVICE_ATTR(vdefreqmin, S_IRUGO , vdeinfo_min, NULL);
static DEVICE_ATTR(vdefreqmax, S_IRUGO , vdeinfo_max, NULL);

static struct attribute *vdeinfo_attributes[] = {
	&dev_attr_vdefreq.attr,
	&dev_attr_vdefreqmin.attr,
	&dev_attr_vdefreqmax.attr,
	NULL
};

static struct attribute_group vdeinfo_group = {
	.attrs  = vdeinfo_attributes,
};

static struct miscdevice vdeinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vdeinfo",
};

static int __init vdeinfo_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, vdeinfo_device.name);
	ret = misc_register(&vdeinfo_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, vdeinfo_device.name);
		return 1;
	}

	if (sysfs_create_group(&vdeinfo_device.this_device->kobj, &vdeinfo_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", vdeinfo_device.name);
	}
	return 0;
}

device_initcall(vdeinfo_init);
