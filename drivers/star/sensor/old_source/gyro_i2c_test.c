/*
 * Copyright (C) 2009 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include "kxtf9.h"

//******************//
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
//#include <linux/tegra_devices.h>
#include <nvodm_services.h>
#include <nvodm_gyroscope.h>
//*****************//

#define NAME			"kxtf9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define WHO_AM_I		0x00
//******************//
#define DRIVER_NAME "nvodm_gyroscope"
#define READ_BUFFER_LENGTH 20

static struct input_dev *gyro_input_dev;
static struct i2c_client *gyro_client;

struct gyroscope_data {
	NvU32 x;
	NvU32 y;
	NvU32 z;
};

struct gyro_device_data
{
	NvOdmGyroscopeDeviceHandle	hOdmGyro;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
	NvOdmGyroscopeIntType	IntType;
	struct timeval		tv;
	struct gyroscope_data prev_data;
	struct gyroscope_data min_data;
	struct gyroscope_data max_data;
};

struct gyro_device_data *gyro_dev=NULL;

static enum {
	COMMAND_LOG = 0,
	COMMAND_FREQUENCY,
	COMMAND_FORCEMOTION,
	COMMAND_FORCETAP,
	COMMAND_TIMETAP,
	COMMAND_OPENCLOSE,
}gyro_enum;

//******************//


/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
struct {
	unsigned int cutoff;
	u8 mask;
} 

gyroscope_table[] = {
	{
	3,	ODR800F}, {
	5,	ODR400F}, {
	10,	ODR200F}, {
	20,	ODR100F}, {
	40,	ODR50F}, {
	80,	ODR25F}, {
	0,	ODR12_5F},
};

struct gyro_data {
	struct i2c_client *client;
	struct input_dev *gyro_input_dev;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;

	int hw_initialized;
	atomic_t enabled;
	//u8 resume[RESUME_ENTRIES];
	int res_interval;
	int irq;
};


/** Function to close the ODM device. This function will help in switching
 * between power modes 
 */
void close_odm_gyro(void)
{
	NvOdmGyroscopeClose(gyro_dev->hOdmGyro);
	gyro_dev->hOdmGyro = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous 
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_gyro(void)
{
	NvS32 err = -1;
	err = NvOdmGyroscopeOpen(&(gyro_dev->hOdmGyro));
	if (!err) {
		err = -ENODEV;
		pr_err("open_def_odm_Gyro: NvOdmGyroOpen failed\n");
		
		return err;
	}
		
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}	
    
	
	return err;	

	return err;
}


static NvS32 __init tegra_gyro_probe(struct platform_device *pdev)
{
	struct gyro_device_data *gyroscope = NULL;
	struct input_dev *input_dev = NULL;
	NvS32 err;
	
	unsigned char buf, val;
	
	gyroscope = kzalloc(sizeof(*gyroscope), GFP_KERNEL);
	if (gyroscope == NULL) {
		err = -ENOMEM;
		pr_err("tegra_gyroscope_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	gyro_dev = gyroscope;

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_gyroscope_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	gyro_dev->input_dev = input_dev;

	err = open_def_odm_gyro();
		if (!err) {
			pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
			goto allocate_dev_fail;
		}

#if 1 // diyu
do{


	mdelay(200);
	NvGyroscopeI2CGetRegs(gyro_dev->hOdmGyro, 0x00, &buf, 1);
	printk("-DY- ============> WHO_AM_I = 0x%x \n", buf);

}while(buf != 0x68);

#endif
	

	platform_set_drvdata(pdev, gyroscope);
	
	return 0;

/*err6:
	kxtf9_input_cleanup(tf9);*/
err5:	
	printk("TEST1--end8	%s \n",__FUNCTION__);
err4:
err3:
err2:

err1:

allocate_dev_fail:
			printk("TEST1--end2  %s \n",__FUNCTION__);
		close_odm_gyro();
		input_free_device(input_dev);
		kfree(gyroscope);
		gyroscope = 0;
		err = -ENOMEM;
err0:
		printk("TEST1--end  %s \n",__FUNCTION__);
	return err;
}

static NvS32 tegra_gyro_remove(struct platform_device *pdev)
{
	input_unregister_device(gyro_input_dev);
	return 0;
}

static NvS32 tegra_gyro_resume(struct platform_device *pdev)
{
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	return 0;
}

static NvS32 tegra_gyro_suspend(struct platform_device *pdev)
{
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	return 0;
}

static struct platform_driver tegra_gyro_driver = {
	.probe	= tegra_gyro_probe,
	.remove	= tegra_gyro_remove,
	.resume = tegra_gyro_resume,
	.suspend = tegra_gyro_suspend,
	.driver	= {
		.name = "tegra_gyroscope",
	},
};

static NvS32 __devinit tegra_gyro_init(void)
{
	printk("TEST1  %s \n",__FUNCTION__);

	return platform_driver_register(&tegra_gyro_driver);
}

static void __exit tegra_gyro_exit(void)
{
	printk("TEST1  %s \n",__FUNCTION__);

	platform_driver_unregister(&tegra_gyro_driver);
}

module_init(tegra_gyro_init);
module_exit(tegra_gyro_exit);

MODULE_DESCRIPTION("Tegra ODM gyro driver");
