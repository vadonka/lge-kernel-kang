/* drivers/misc/otf/otf_example.c
 *
 * Original source by Benee (c) 2012
 *
 * Modified for the OTF by vadonka 2012
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *

/* Some example */
#if 0
void example_block(void)
{
...code...
}
EXPORT_SYMBOL(example_block);
---
extern void example_block(void);
example_block();
#endif

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

int MIN_EXAMPLEVALUE = 1000;
int MAX_EXAMPLEVALUE = 2000;
int DEF_EXAMPLEVALUE = 1500;

static int example_value = 1500;
int examplecontrol_value(void) {
	return example_value;
}
EXPORT_SYMBOL(examplecontrol_value);
/*
 * Calling with this:
 ********************
 * extern int examplecontrol_value(void);
 *
 * Using:
 ********
 * examplecontrol_value()
 *
 */

static ssize_t examplecontrol_value_read(struct device * dev, struct device_attribute * attr, char * buf) {
	return sprintf(buf, "%d\n", example_value);
}

static ssize_t examplecontrol_value_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size) {
	int data;

	if (sscanf(buf, "%d\n", &data) == 1) {
		if (data != example_value) {
			example_value = min(max(data, MIN_EXAMPLEVALUE), MAX_EXAMPLEVALUE);
			pr_info("EXAMPLECONTROL threshold changed to %d\n", example_value);
		}
	} else {
		pr_info("EXAMPLECONTROL invalid input\n"); 
	}
	return size;
}

static ssize_t examplecontrol_min(struct device * dev, struct device_attribute * attr, char * buf) {
	return sprintf(buf, "%u\n", MIN_EXAMPLEVALUE);
}

static ssize_t examplecontrol_max(struct device * dev, struct device_attribute * attr, char * buf) {
	return sprintf(buf, "%u\n", MAX_EXAMPLEVALUE);
}

static ssize_t examplecontrol_def(struct device * dev, struct device_attribute * attr, char * buf) {
	return sprintf(buf, "%u\n", DEF_EXAMPLEVALUE);
}

static DEVICE_ATTR(examplevalue, S_IRUGO | S_IWUGO, examplecontrol_value_read, examplecontrol_value_write);
static DEVICE_ATTR(min, S_IRUGO , examplecontrol_min, NULL);
static DEVICE_ATTR(max, S_IRUGO , examplecontrol_max, NULL);
static DEVICE_ATTR(def, S_IRUGO , examplecontrol_def, NULL);

static struct attribute *examplecontrol_attributes[] = {
	&dev_attr_examplevalue.attr,
	&dev_attr_min.attr,
	&dev_attr_max.attr,
	&dev_attr_def.attr,
	NULL
};

static struct attribute_group examplecontrol_group = {
	.attrs  = examplecontrol_attributes,
};

static struct miscdevice examplecontrol_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "examplecontrol",
};

static int __init examplecontrol_init(void) {
	int ret;
	pr_info("%s misc_register(%s)\n", __FUNCTION__, examplecontrol_device.name);
	ret = misc_register(&examplecontrol_device);

	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, examplecontrol_device.name);
		return 1;
	}

	if (sysfs_create_group(&examplecontrol_device.this_device->kobj, &examplecontrol_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n", examplecontrol_device.name);
	}
	return 0;
}

device_initcall(examplecontrol_init);
