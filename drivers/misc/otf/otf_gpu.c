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

/* Static containers */
static unsigned int MIN_GPUFREQ = 300000;
static unsigned int MAX_GPUFREQ = 400000;
static unsigned int DEF_GPUFREQ = 350000;

/* Boot time value */
unsigned int gpufreq = 350000;

static ssize_t gpufreq_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", gpufreq);
}

extern unsigned int nitro;
static ssize_t gpufreq_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int datagpu;

	if (sscanf(buf, "%d\n", &datagpu) == 1)
	{
		if (datagpu != gpufreq)
		{
			if (nitro == 1)
			{
				gpufreq = MAX_GPUFREQ;
				pr_info("NITRO Enabled! GPUCONTROL threshold changed to %d\n", gpufreq);
			}
			else
			{
				gpufreq = min(max(datagpu, MIN_GPUFREQ), MAX_GPUFREQ);
				pr_info("GPUCONTROL threshold changed to %d\n", gpufreq);
			}
		}
	}
	else
	{
		pr_info("GPUCONTROL invalid input\n");
	}
	return size;
}

static ssize_t gpucontrol_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_GPUFREQ);
}

static ssize_t gpucontrol_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_GPUFREQ);
}

static ssize_t gpucontrol_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_GPUFREQ);
}

static DEVICE_ATTR(gpufreq, S_IRUGO | S_IWUGO, gpufreq_read, gpufreq_write);
static DEVICE_ATTR(gpufreqmin, S_IRUGO , gpucontrol_min, NULL);
static DEVICE_ATTR(gpufreqmax, S_IRUGO , gpucontrol_max, NULL);
static DEVICE_ATTR(gpufreqdef, S_IRUGO , gpucontrol_def, NULL);

static struct attribute *gpucontrol_attributes[] = {
	&dev_attr_gpufreq.attr,
	&dev_attr_gpufreqmin.attr,
	&dev_attr_gpufreqmax.attr,
	&dev_attr_gpufreqdef.attr,
	NULL
};

static struct attribute_group gpucontrol_group = {
	.attrs  = gpucontrol_attributes,
};

static struct miscdevice gpucontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpucontrol",
};

static int __init gpucontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, gpucontrol_device.name);
	ret = misc_register(&gpucontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, gpucontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&gpucontrol_device.this_device->kobj, &gpucontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", gpucontrol_device.name);
	}
	return 0;
}

device_initcall(gpucontrol_init);
