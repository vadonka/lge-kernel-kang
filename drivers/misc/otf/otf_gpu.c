/* drivers/misc/otf/otf_gpu.c
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
 * gpufreq values are set in the init/otfinit.c
 */
extern unsigned int MIN_GPUFREQ;
extern unsigned int MAX_GPUFREQ;

/* Boot time value taken from the kernel cmdline */
extern unsigned int gpufreq;

static ssize_t gpuinfo_boot(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", gpufreq);
}

static ssize_t gpuinfo_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_GPUFREQ);
}

static ssize_t gpuinfo_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_GPUFREQ);
}

static DEVICE_ATTR(gpufreq, S_IRUGO , gpuinfo_boot, NULL);
static DEVICE_ATTR(gpufreqmin, S_IRUGO , gpuinfo_min, NULL);
static DEVICE_ATTR(gpufreqmax, S_IRUGO , gpuinfo_max, NULL);

static struct attribute *gpuinfo_attributes[] = {
	&dev_attr_gpufreq.attr,
	&dev_attr_gpufreqmin.attr,
	&dev_attr_gpufreqmax.attr,
	NULL
};

static struct attribute_group gpuinfo_group = {
	.attrs  = gpuinfo_attributes,
};

static struct miscdevice gpuinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpuinfo",
};

static int __init gpuinfo_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, gpuinfo_device.name);
	ret = misc_register(&gpuinfo_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, gpuinfo_device.name);
		return 1;
	}

	if (sysfs_create_group(&gpuinfo_device.this_device->kobj, &gpuinfo_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", gpuinfo_device.name);
	}
	return 0;
}

device_initcall(gpuinfo_init);
