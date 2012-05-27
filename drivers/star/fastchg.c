/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/fastchg.h>
#include <linux/slab.h>

int force_charge_mode;

/* sysfs interface */
static ssize_t force_charge_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
return sprintf(buf, "%d\n", force_charge_mode);
}

static ssize_t force_charge_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
sscanf(buf, "%du", &force_charge_mode);
return count;
}

static struct kobj_attribute force_charge_mode_attribute =
__ATTR(force_charge_mode, 0666, force_charge_mode_show, force_charge_mode_store);

static struct attribute *attrs[] = {
&force_charge_mode_attribute.attr,
NULL,
};

static struct attribute_group attr_group = {
.attrs = attrs,
};

static struct kobject *force_charge_mode_kobj;

static int __init force_charge_mode_init(void)
{
	int retval;

	force_charge_mode = 0;

        force_charge_mode_kobj = kobject_create_and_add("fast_charge", kernel_kobj);
        if (!force_charge_mode_kobj) {
                return -ENOMEM;
        }
        retval = sysfs_create_group(force_charge_mode_kobj, &attr_group);
        if (retval)
                kobject_put(force_charge_mode_kobj);
        return retval;
}
/* end sysfs interface */

static void __exit force_charge_mode_exit(void)
{
	kobject_put(force_charge_mode_kobj);
}

module_init(force_charge_mode_init);
module_exit(force_charge_mode_exit);
