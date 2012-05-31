/* drivers/misc/otf/otf_cpu1.c
 *
 * Original source by Benee (c) 2012
 *
 * Modified for the OTF by vadonka (c) 2012
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

/** Define the LG variables
 * You need to undef or comment out the original LG definitions
 * in 'arch/arm/mach-tegra/nvrm/core/ap20/ap20rm_power_dfs.h'
 */

/** Static containers */
/*
 * Defines CPU frequency threshold for slave CPU1 power management:
 * - CPU1 is turned Off when cpu clock is below ONMINKHZ for
 *   ONDELAY time in a row
 * - CPU1 is turned On when cpu clock is above OFFMAXKHZ for
 *   OFFDELAY time in a row
 */
static unsigned int MIN_ONMINKHZ  = 216000;	/* Minimum ON_MIN_KHZ value     */
static unsigned int MAX_ONMINKHZ  = 1100000;	/* Maximum ON_MIN_KHZ value     */
static unsigned int DEF_ONMINKHZ  = 816000;	/* Default ON_MIN_KHZ value     */

static unsigned int MIN_ONDELAY   = 200;	/* Minimum ON_PENDING_MS value  */
static unsigned int MAX_ONDELAY   = 3000;	/* Maximum ON_PENDING_MS value  */
static unsigned int DEF_ONDELAY   = 2000;	/* Default ON_PENDING_MS value  */

static unsigned int MIN_OFFMAXKHZ = 216000;	/* Minimum OFF_MAX_KHZ value    */
static unsigned int MAX_OFFMAXKHZ = 1100000;	/* Maximum OFF_MAX_KHZ value    */
static unsigned int DEF_OFFMAXKHZ = 860000;	/* Default OFF_MAX_KHZ value    */

static unsigned int MIN_OFFDELAY  = 200;	/* Mininum OFF_PENDING_MS value */
static unsigned int MAX_OFFDELAY  = 3000;	/* Maximum OFF_PENDING_MS value */
static unsigned int DEF_OFFDELAY  = 1000;	/* Default OFF_PENDING_MS value */

/** Static containers end */

/* Boot time values */
unsigned int onminkhz  = 816000;		/* ON_MIN_KHZ boot time value     */
unsigned int ondelay   = 2000;			/* ON_PENDING_MS boot time value  */

unsigned int offmaxkhz = 860000;		/* OFF_MAX_KHZ boot time value    */
unsigned int offdelay  = 1000;			/* OFF_PENDING_MS boot time value */

unsigned int NVRM_CPU1_ON_MIN_KHZ     = 816000;	/* ON_MIN_KHZ                     */
unsigned int NVRM_CPU1_ON_PENDING_MS  = 2000;	/* ON_PENDING_MS                  */
unsigned int NVRM_CPU1_OFF_MAX_KHZ    = 860000;	/* OFF_MAX_KHZ                    */
unsigned int NVRM_CPU1_OFF_PENDING_MS = 1000;	/* OFF_PENDING_MS                 */
/* Boot time values end */

/** SYSFS */
extern unsigned int nitro;
/* ON_MIN_KHZ */
static ssize_t onminkhz_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", onminkhz);
}

static ssize_t onminkhz_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int dataonminkhz;

	if (nitro != 1)
	{
		if (sscanf(buf, "%d\n", &dataonminkhz) == 1)
		{
			if (dataonminkhz != onminkhz)
			{
				onminkhz = min(max(dataonminkhz, MIN_ONMINKHZ), MAX_ONMINKHZ);
				/* LG variable get the new value */
				NVRM_CPU1_ON_MIN_KHZ = onminkhz;
				pr_info("CPU1_ON_MIN_KHZ threshold changed to %d\n", onminkhz);
			}
		}
		else
		{
			pr_info("CPU1_ON_MIN_KHZ invalid input\n");
		}
		return size;
	}
}

static ssize_t onminkhz_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_ONMINKHZ);
}

static ssize_t onminkhz_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_ONMINKHZ);
}

static ssize_t onminkhz_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_ONMINKHZ);
}

/* OFF_MAX_KHZ */
static ssize_t offmaxkhz_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", offmaxkhz);
}

static ssize_t offmaxkhz_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int dataoffmaxkhz;

	if (nitro == 0)
	{
		if (sscanf(buf, "%d\n", &dataoffmaxkhz) == 1)
		{
			if (dataoffmaxkhz != offmaxkhz)
			{
				offmaxkhz = min(max(dataoffmaxkhz, MIN_OFFMAXKHZ), MAX_OFFMAXKHZ);
				/* LG variable get the new value */
				NVRM_CPU1_OFF_MAX_KHZ = offmaxkhz;
				pr_info("CPU1_OFF_MAX_KHZ threshold changed to %d\n", offmaxkhz);
			}
		}
		else
		{
			pr_info("CPU1_OFF_MAX_KHZ invalid input\n");
		}
		return size;
	}
}

static ssize_t offmaxkhz_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_OFFMAXKHZ);
}

static ssize_t offmaxkhz_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_OFFMAXKHZ);
}

static ssize_t offmaxkhz_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_OFFMAXKHZ);
}

/* ON_PENDING_MS */
static ssize_t ondelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", ondelay);
}

static ssize_t ondelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int dataondelay;

	if (nitro == 0)
	{
		if (sscanf(buf, "%d\n", &dataondelay) == 1)
		{
			if (dataondelay != ondelay)
			{
				ondelay = min(max(dataondelay, MIN_ONDELAY), MAX_ONDELAY);
				/* LG variable get the new value */
				NVRM_CPU1_ON_PENDING_MS = ondelay;
				pr_info("CPU1_ON_PENDING_MS threshold changed to %d\n", ondelay);
			}
		}
		else
		{
			pr_info("CPU1_ON_PENDING_MS invalid input\n");
		}
		return size;
	}
}

static ssize_t ondelay_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_ONDELAY);
}

static ssize_t ondelay_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_ONDELAY);
}

static ssize_t ondelay_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_ONDELAY);
}

/* OFF_PENDING_MS */
static ssize_t offdelay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", offdelay);
}

static ssize_t offdelay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	int dataoffdelay;

	if (nitro == 0)
	{
		if (sscanf(buf, "%d\n", &dataoffdelay) == 1)
		{
			if (dataoffdelay != offdelay)
			{
				offdelay = min(max(dataoffdelay, MIN_OFFDELAY), MAX_OFFDELAY);
				/* LG variable get the new value */
				NVRM_CPU1_OFF_PENDING_MS = offdelay;
				pr_info("CPU1_OFF_PENDING_MS threshold changed to %d\n", offdelay);
			}
		}
		else
		{
			pr_info("CPU1_OFF_PENDING_MS invalid input\n");
		}
		return size;
	}
}

static ssize_t offdelay_min(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MIN_OFFDELAY);
}

static ssize_t offdelay_max(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", MAX_OFFDELAY);
}

static ssize_t offdelay_def(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", DEF_OFFDELAY);
}

/** SYSFS attributes */
/* ON_MIN_KHZ */
static DEVICE_ATTR(onminkhz, S_IRUGO | S_IWUGO, onminkhz_read, onminkhz_write);
static DEVICE_ATTR(onminkhzmin, S_IRUGO , onminkhz_min, NULL);
static DEVICE_ATTR(onminkhzmax, S_IRUGO , onminkhz_max, NULL);
static DEVICE_ATTR(onminkhzdef, S_IRUGO , onminkhz_def, NULL);

/* ON_PENDING_MS */
static DEVICE_ATTR(ondelay, S_IRUGO | S_IWUGO, ondelay_read, ondelay_write);
static DEVICE_ATTR(ondelaymin, S_IRUGO , ondelay_min, NULL);
static DEVICE_ATTR(ondelaymax, S_IRUGO , ondelay_max, NULL);
static DEVICE_ATTR(ondelaydef, S_IRUGO , ondelay_def, NULL);

/* OFF_MAX_KHZ */
static DEVICE_ATTR(offmaxkhz, S_IRUGO | S_IWUGO, offmaxkhz_read, offmaxkhz_write);
static DEVICE_ATTR(offmaxkhzmin, S_IRUGO , offmaxkhz_min, NULL);
static DEVICE_ATTR(offmaxkhzmax, S_IRUGO , offmaxkhz_max, NULL);
static DEVICE_ATTR(offmaxkhzdef, S_IRUGO , offmaxkhz_def, NULL);

/* OFF_PENDING_MS */
static DEVICE_ATTR(offdelay, S_IRUGO | S_IWUGO, offdelay_read, offdelay_write);
static DEVICE_ATTR(offdelaymin, S_IRUGO , offdelay_min, NULL);
static DEVICE_ATTR(offdelaymax, S_IRUGO , offdelay_max, NULL);
static DEVICE_ATTR(offdelaydef, S_IRUGO , offdelay_def, NULL);

static struct attribute *cpu1control_attributes[] = {
	&dev_attr_onminkhz.attr,
	&dev_attr_onminkhzmin.attr,
	&dev_attr_onminkhzmax.attr,
	&dev_attr_onminkhzdef.attr,
	&dev_attr_ondelay.attr,
	&dev_attr_ondelaymin.attr,
	&dev_attr_ondelaymax.attr,
	&dev_attr_ondelaydef.attr,
	&dev_attr_offmaxkhz.attr,
	&dev_attr_offmaxkhzmin.attr,
	&dev_attr_offmaxkhzmax.attr,
	&dev_attr_offmaxkhzdef.attr,
	&dev_attr_offdelay.attr,
	&dev_attr_offdelaymin.attr,
	&dev_attr_offdelaymax.attr,
	&dev_attr_offdelaydef.attr,
	NULL
};

static struct attribute_group cpu1control_group = {
	.attrs  = cpu1control_attributes,
};

static struct miscdevice cpu1control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cpu1control",
};

static int __init cpu1control_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, cpu1control_device.name);
	ret = misc_register(&cpu1control_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, cpu1control_device.name);
		return 1;
	}

	if (sysfs_create_group(&cpu1control_device.this_device->kobj, &cpu1control_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", cpu1control_device.name);
	}
	return 0;
}

device_initcall(cpu1control_init);
