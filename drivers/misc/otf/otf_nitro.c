/* drivers/misc/otf/otf_nitro.c
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
#include "otf.h"

/* Static containers */
static unsigned int MIN_NITRO = 0;
static unsigned int MAX_NITRO = 1;
static unsigned int DEF_NITRO = 0;
static unsigned int onminkhznitro = 655000;
static unsigned int ondelaynitro = 200;
static unsigned int offmaxkhznitro = 610000;
static unsigned int offdelaynitro = 200;
static unsigned int avpfreqnitro = 280000;
static unsigned int gpufreqnitro = 366000;
static unsigned int vdefreqnitro = 700000;

/* Boot time value */
unsigned int nitro = 0;
/* Boot time value end */

extern unsigned int onminkhz;
extern unsigned int ondelay;
extern unsigned int offmaxkhz;
extern unsigned int offdelay;
extern unsigned int avpfreq;
extern unsigned int gpufreq;
extern unsigned int vdefreq;

static unsigned int oldonminkhz;
static unsigned int oldondelay;
static unsigned int oldoffmaxkhz;
static unsigned int oldoffdelay;
static unsigned int oldavpfreq;
static unsigned int oldgpufreq;
static unsigned int oldvdefreq;

static ssize_t nitro_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", nitro);
}

static ssize_t nitro_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int datanitro;

	if (sscanf(buf, "%d\n", &datanitro) == 1)
	{
		if (datanitro != nitro)
		{
			nitro = min(max(datanitro, MIN_NITRO), MAX_NITRO);
			if (nitro == 1)
			{
				pr_info("NITRO enabled\n", nitro);

				oldonminkhz = onminkhz;
				onminkhz = onminkhznitro;
				NVRM_CPU1_ON_MIN_KHZ = onminkhz;
				pr_info("NITRO Enabled! CPU1_ON_MIN_KHZ threshold changed to %d\n", onminkhz);

				oldondelay = ondelay;
				ondelay = ondelaynitro;
				NVRM_CPU1_ON_PENDING_MS = ondelay;
				pr_info("NITRO Enabled! CPU1_ON_PENDING_MS threshold changed to %d\n", ondelay);

				oldoffmaxkhz = offmaxkhz;
				offmaxkhz = offmaxkhznitro;
				NVRM_CPU1_OFF_MAX_KHZ = offmaxkhz;
				pr_info("NITRO Enabled! CPU1_OFF_MAX_KHZ threshold changed to %d\n", offmaxkhz);

				oldoffdelay = offdelay;
				offdelay = offdelaynitro;
				NVRM_CPU1_OFF_PENDING_MS = offdelay;
				pr_info("NITRO Enabled! CPU1_OFF_PENDING_MS threshold changed to %d\n", offdelay);

				oldavpfreq = avpfreq;
				avpfreq = avpfreqnitro;
				pr_info("NITRO Enabled! AVPCONTROL threshold changed to %d\n", avpfreq);

				oldgpufreq = gpufreq;
				gpufreq = gpufreqnitro;
				pr_info("NITRO Enabled! GPUCONTROL threshold changed to %d\n", gpufreq);

				oldvdefreq = vdefreq;
				vdefreq = vdefreqnitro;
				pr_info("NITRO Enabled! VDECONTROL threshold changed to %d\n", vdefreq);
			}
			else
			{
				pr_info("NITRO disabled\n", nitro);

				onminkhz = oldonminkhz;
				NVRM_CPU1_ON_MIN_KHZ = onminkhz;
				pr_info("CPU1_ON_MIN_KHZ threshold restored to %d\n", onminkhz);

				ondelay = oldondelay;
				NVRM_CPU1_ON_PENDING_MS = ondelay;
				pr_info("CPU1_ON_PENDING_MS threshold restored to %d\n", ondelay);

				offmaxkhz = oldoffmaxkhz;
				NVRM_CPU1_OFF_MAX_KHZ = offmaxkhz;
				pr_info("CPU1_OFF_MAX_KHZ threshold restored to %d\n", offmaxkhz);

				offdelay = oldoffdelay;
				NVRM_CPU1_OFF_PENDING_MS = offdelay;
				pr_info("CPU1_OFF_PENDING_MS threshold restored to %d\n", offdelay);

				avpfreq = oldavpfreq;
				pr_info("AVPCONTROL threshold restored to %d\n", avpfreq);

				gpufreq = oldgpufreq;
				pr_info("GPUCONTROL threshold restored to %d\n", gpufreq);

				vdefreq = oldvdefreq;
				pr_info("VDECONTROL threshold restored to %d\n", vdefreq);
			}
		}
	}
	else
	{
		pr_info("NITRO invalid input\n");
	}
	return size;
}

static ssize_t nitrocontrol_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_NITRO);
}

static ssize_t nitrocontrol_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_NITRO);
}

static ssize_t nitrocontrol_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_NITRO);
}

static DEVICE_ATTR(nitro, S_IRUGO | S_IWUGO, nitro_read, nitro_write);
static DEVICE_ATTR(nitromin, S_IRUGO , nitrocontrol_min, NULL);
static DEVICE_ATTR(nitromax, S_IRUGO , nitrocontrol_max, NULL);
static DEVICE_ATTR(nitrodef, S_IRUGO , nitrocontrol_def, NULL);

static struct attribute *nitrocontrol_attributes[] = {
	&dev_attr_nitro.attr,
	&dev_attr_nitromin.attr,
	&dev_attr_nitromax.attr,
	&dev_attr_nitrodef.attr,
	NULL
};

static struct attribute_group nitrocontrol_group = {
	.attrs  = nitrocontrol_attributes,
};

static struct miscdevice nitrocontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "nitrocontrol",
};

static int __init nitrocontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, nitrocontrol_device.name);
	ret = misc_register(&nitrocontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, nitrocontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&nitrocontrol_device.this_device->kobj, &nitrocontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", nitrocontrol_device.name);
	}
	return 0;
}

device_initcall(nitrocontrol_init);
