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
#include <nvodm_accelerometer.h>
//*****************//

#define DEBUG			0
#define DEBUG_ACCEL_DATA 0

#define NAME			"kxtf9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define XOUT_H			0x07
#define YOUT_L			0x08
#define YOUT_H			0x09
#define ZOUT_L			0x0A
#define ZOUT_H			0x0B
#define INT_SRC_REG1		0x15
#define INT_STATUS_REG		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x00
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			32
#define FLAT			32
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

//******************//
#define DRIVER_NAME "nvodm_accelerometer"
#define READ_BUFFER_LENGTH 20

//typedef unsigned char	u8;

static struct input_dev *g_input_dev;
static struct i2c_client *g_client;

struct accelerometer_data {
	NvU32 x;
	NvU32 y;
	NvU32 z;
};

struct tegra_acc_device_data
{
	NvOdmAcrDeviceHandle	hOdmAcr;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
	NvBool			bThreadAlive;
	NvBool			show_log;
	NvOdmAccelIntType	IntType;
	NvOdmAccelAxisType	IntMotionAxis;
	NvOdmAccelAxisType	IntTapAxis;
	struct timeval		tv;
	struct accelerometer_data prev_data;
	struct accelerometer_data min_data;
	struct accelerometer_data max_data;
};

struct tegra_acc_device_data *accel_dev=NULL;


static const char* parameter[] = {
	"log=",
	"frequency=",
	"forcemotion=",
	"forcetap=",
	"timetap=",
	"openclose=",
};

static enum {
	COMMAND_LOG = 0,
	COMMAND_FREQUENCY,
	COMMAND_FORCEMOTION,
	COMMAND_FORCETAP,
	COMMAND_TIMETAP,
	COMMAND_OPENCLOSE,
}accel_enum;

//******************//


/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
struct {
	unsigned int cutoff;
	u8 mask;
} 

kxtf9_odr_table[] = {
	{
	3,	ODR800F}, {
	5,	ODR400F}, {
	10,	ODR200F}, {
	20,	ODR100F}, {
	40,	ODR50F}, {
	80,	ODR25F}, {
	0,	ODR12_5F},
};

struct kxtf9_data {
	struct i2c_client *client;
	struct input_dev *accel_input_dev;
	struct kxtf9_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;

	int hw_initialized;
	atomic_t enabled;
	u8 resume[RESUME_ENTRIES];
	int res_interval;
	int irq;
#if 1 // 2010-06-14, Need change to last kionix driver version, currently temp. modification.
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
#endif

};

static struct kxtf9_data *kxtf9_misc_data;

/** Function to close the ODM device. This function will help in switching
 * between power modes 
 */
void close_odm_accl(void)
{
	NvOdmAccelClose(accel_dev->hOdmAcr);
	accel_dev->hOdmAcr = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous 
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_accl(void)
{
	NvS32 err = -1;
	err = NvOdmAccelOpen(&(accel_dev->hOdmAcr));
	if (!err) {
		err = -ENODEV;
		pr_err("open_def_odm_accl: NvOdmAccelOpen failed\n");
		
		return err;
	}
	#if 1
	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
		NvOdmAccelInt_MotionThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);
	#endif
	
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}	
    
	//sk.hwang@lge.com
	#if defined(CONFIG_MACH_STAR)
	NvOdmAccelSetSampleRate(accel_dev->hOdmAcr, 800);
	#endif
	return err;	
#if 0
		err = NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
		 NvOdmAccelInt_MotionThreshold, 0, 900);
	if (!err) {
		printk("TEST1=7  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
		return err;
	}

	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
		NvOdmAccelInt_MotionThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);

	if (!err) {
		printk("TEST1=8  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}
	
	/*
	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);

	if (!err) {
		printk("TEST1=9  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}*/

	err = NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, 0, 120);

	if (!err) {
		printk("TEST1=10  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
		return err;
	}

	err = NvOdmAccelSetIntTimeThreshold(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, 0, 2);

	if (!err) {
		printk("TEST1=11  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntTimeThreshold\n");
		return err;
	}
#endif
	return err;
}

static int kxtf9_i2c_read(struct kxtf9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = tf9->client->addr,
		 .flags = (tf9->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};
	err = i2c_transfer(tf9->client->adapter, msgs, 2);

	

	if (err != 2)
		dev_err(&tf9->client->dev, "read transfer error\n");
	else
		err = 0;

	return err;
}

static int kxtf9_i2c_write(struct kxtf9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;
	int i;
	u8 buf[len + 1];

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	buf[0] = addr;
	for (i = 0; i < len; i++)
		buf[i + 1] = data[i];

	err = i2c_transfer(tf9->client->adapter, msgs, 1);

	if (err != 1)
		dev_err(&tf9->client->dev, "write transfer error\n");
	else
		err = 0;

	return err;
}

static int kxtf9_verify(struct kxtf9_data *tf9)
{
	int err;
	u8 buf;

	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, WHO_AM_I, &buf);
	
	
	#if DEBUG
		printk("WHO_AM_I = 0x%02x\n", buf);  // 	dev_info(&tf9->client->dev, "WHO_AM_I = 0x%02x\n", buf);
	#endif
	
	if (err < 0)
		printk("read err int source\n");  // 		dev_err(&tf9->client->dev, "read err int source\n");
	if (buf != 1)
		err = -1;
	return err;
}

static int kxtf9_hw_init(struct kxtf9_data *tf9)
{
	int err = -1;
	u8 buf[7];

	//buf[0] = PC1_OFF;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_OFF);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, DATA_CTRL, tf9->resume[RES_DATA_CTRL]);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG3, tf9->resume[RES_CTRL_REG3]);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, TILT_TIMER, tf9->resume[RES_TILT_TIMER]);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, WUF_TIMER, tf9->resume[RES_WUF_TIMER]);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, WUF_THRESH, tf9->resume[RES_WUF_THRESH]);

	if (err < 0)
		return err;
	buf[0] = tf9->resume[RES_TDT_TIMER];
	buf[1] = tf9->resume[RES_TDT_H_THRESH];
	buf[2] = tf9->resume[RES_TDT_L_THRESH];
	buf[3] = tf9->resume[RES_TAP_TIMER];
	buf[4] = tf9->resume[RES_TOTAL_TIMER];
	buf[5] = tf9->resume[RES_LAT_TIMER];
	buf[6] = tf9->resume[RES_WIN_TIMER];
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, TDT_TIMER, buf);
	
	if (err < 0)
		return err;
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, INT_CTRL1, tf9->resume[RES_INT_CTRL1]);
	
	if (err < 0)
		return err;
	buf[0] = (tf9->resume[RES_CTRL_REG1] | PC1_ON);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, buf);
	
	if (err < 0)
		return err;
	tf9->resume[RES_CTRL_REG1] = buf[0];
	tf9->hw_initialized = 1;

	return 0;
}

static void kxtf9_device_power_off(struct kxtf9_data *tf9)
{
	int err;
	u8 buf = PC1_OFF;

	//NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_OFF);
	printk("This code is %d,in %s\n",__LINE__, __FUNCTION__);
	if (err < 0)
		printk("soft power off failed\n");  //		dev_err(&tf9->client->dev, "soft power off failed\n");
	disable_irq(tf9->irq);
	if (tf9->pdata->power_off)
		tf9->pdata->power_off();
	tf9->hw_initialized = 0;
}

static int kxtf9_device_power_on(struct kxtf9_data *tf9)
{
	int err;


	if (tf9->pdata->power_on) {
		err = tf9->pdata->power_on();
		if (err < 0)
			return err;
	}
	enable_irq(tf9->irq);
	if (!tf9->hw_initialized) {
		mdelay(100);
		err = kxtf9_hw_init(tf9);
		if (err < 0) {
			kxtf9_device_power_off(tf9);
			return err;
		}
	}
	//NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, PC1_ON);
	printk("This code is %d,in %s\n",__LINE__, __FUNCTION__);

	return 0;
}

static irqreturn_t kxtf9_isr(int irq, void *dev)
{
	struct kxtf9_data *tf9 = dev;

	disable_irq_nosync(irq);
	schedule_work(&tf9->irq_work);

	return IRQ_HANDLED;
}

static u8 kxtf9_resolve_dir(struct kxtf9_data *tf9, u8 dir)
{
	switch (dir) {
	case 0x20:	/* -X */
		if (tf9->pdata->negate_x)
			dir = 0x10;
		if (tf9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (tf9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x10:	/* +X */
		if (tf9->pdata->negate_x)
			dir = 0x20;
		if (tf9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (tf9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x08:	/* -Y */
		if (tf9->pdata->negate_y)
			dir = 0x04;
		if (tf9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (tf9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x04:	/* +Y */
		if (tf9->pdata->negate_y)
			dir = 0x08;
		if (tf9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (tf9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x02:	/* -Z */
		if (tf9->pdata->negate_z)
			dir = 0x01;
		if (tf9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (tf9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	case 0x01:	/* +Z */
		if (tf9->pdata->negate_z)
			dir = 0x02;
		if (tf9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (tf9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	default:
		return -EINVAL;
	}

	return dir;
}

static void kxtf9_irq_work_func(struct work_struct *work)
{
/*
 *	int_status output:
 *	[INT_SRC_REG2][INT_SRC_REG1][TILT_POS_PRE][TILT_POS_CUR]
 *	INT_SRC_REG1, TILT_POS_PRE, and TILT_POS_CUR directions are translated
 *	based on platform data variables.
 */

	int err;
	int int_status = 0;
	u8 status;
	u8 buf[2];

	struct kxtf9_data *tf9
			= container_of(work, struct kxtf9_data, irq_work);

	//err = kxtf9_i2c_read(tf9, INT_STATUS_REG, &status, 1);
	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_STATUS_REG, &status);
	if (err < 0)
		printk("read err int source\n"); //		dev_err(&tf9->client->dev, "read err int source\n");
	int_status = status << 24;
	if ((status & TPS) > 0) {
		//err = kxtf9_i2c_read(tf9, TILT_POS_CUR, buf, 2);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, TILT_POS_CUR, &buf);
		
		if (err < 0)
			printk("read err tilt dir\n");  //			dev_err(&tf9->client->dev, "read err tilt dir\n");
		int_status |= kxtf9_resolve_dir(tf9, buf[0]);
		int_status |= kxtf9_resolve_dir(tf9, buf[1]) << 8;
		/*** DEBUG OUTPUT - REMOVE ***/
		#if DEBUG
			printk("IRQ TILT [%x]\n",
						kxtf9_resolve_dir(tf9, buf[0]));
		#endif
		
		/*dev_info(&tf9->client->dev, "IRQ TILT [%x]\n",
						kxtf9_resolve_dir(tf9, buf[0]));*/
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	if (((status & TDTS0) | (status & TDTS1)) > 0) {
		//err = kxtf9_i2c_read(tf9, INT_SRC_REG1, buf, 1);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_SRC_REG1, &buf);
		if (err < 0)
			printk("read err tap dir\n");  // 			dev_err(&tf9->client->dev, "read err tap dir\n");
		int_status |= (kxtf9_resolve_dir(tf9, buf[0])) << 16;
		/*** DEBUG OUTPUT - REMOVE ***/
		#if DEBUG
			printk("IRQ TAP%d [%x]\n",
				((status & TDTS1) ? (2) : (1)), kxtf9_resolve_dir(tf9, buf[0]));
		#endif
		/*dev_info(&tf9->client->dev, "IRQ TAP%d [%x]\n",
		((status & TDTS1) ? (2) : (1)), kxtf9_resolve_dir(tf9, buf[0]));*/
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	#if DEBUG	
	/*** DEBUG OUTPUT - REMOVE ***/
	if ((status & 0x02) > 0) {
		if (((status & TDTS0) | (status & TDTS1)) > 0)
			printk("IRQ WUF + TAP\n"); 			//dev_info(&tf9->client->dev, "IRQ WUF + TAP\n");
		else
			printk("IRQ WUF\n"); 			//dev_info(&tf9->client->dev, "IRQ WUF\n");
	}
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	#endif
	
	if (int_status & 0x2FFF) {
		input_report_abs(tf9->input_dev, ABS_MISC, int_status);
		input_sync(tf9->input_dev);
	}
	//err = kxtf9_i2c_read(tf9, INT_REL, buf, 1);
	err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_REL, &buf);
	if (err < 0)
		printk("error clearing interrupt status: %d\n", err);
		/*dev_err(&tf9->client->dev,
						"error clearing interrupt status: %d\n", err);*/

	enable_irq(tf9->irq);
}

int kxtf9_update_g_range(struct kxtf9_data *tf9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf;

	switch (new_g_range) {
	case KXTF9_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case KXTF9_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case KXTF9_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		printk("invalid g range request\n");
		//dev_err(&tf9->client->dev, "invalid g range request\n");
		return -EINVAL;
	}
	if (shift != tf9->pdata->shift_adj) {
		if (tf9->pdata->shift_adj > shift)
			tf9->resume[RES_WUF_THRESH] >>=
						(tf9->pdata->shift_adj - shift);
		if (tf9->pdata->shift_adj < shift)
			tf9->resume[RES_WUF_THRESH] <<=
						(shift - tf9->pdata->shift_adj);

		if (atomic_read(&tf9->enabled)) {
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr,  CTRL_REG1 , PC1_OFF);

			if (err < 0)
				return err;
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr,  WUF_THRESH , tf9->resume[RES_WUF_THRESH]);

			if (err < 0)
				return err;
			NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr,  CTRL_REG1 , (tf9->resume[RES_CTRL_REG1] & 0xE7) | new_g_range);
			if (err < 0)
				return err;
		}
	}
	tf9->resume[RES_CTRL_REG1] = buf;
	tf9->pdata->shift_adj = shift;

	return 0;
}

int kxtf9_update_odr(struct kxtf9_data *tf9, int poll_interval)
{
	int err = -1;
	int i;
	u8 config;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next slower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(kxtf9_odr_table); i++) {
		config = kxtf9_odr_table[i].mask;
		if (poll_interval < kxtf9_odr_table[i].cutoff)
			break;
	}

	if (atomic_read(&tf9->enabled)) {
		//err = kxtf9_i2c_write(tf9, DATA_CTRL, &config, 1);
		err = NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr,  DATA_CTRL , config);
		if (err < 0)
			return err;
		/*
		 *  Latch on input_dev - indicates that kxtf9_input_init passed
		 *  and this workqueue is available
		 */
		if (tf9->input_dev) {
			cancel_delayed_work_sync(&tf9->input_work);
			schedule_delayed_work(&tf9->input_work,
				      msecs_to_jiffies(poll_interval));
		}
	}
	tf9->resume[RES_DATA_CTRL] = config;

	return 0;
}

// return value
// int *ret_xyz : count value, range : 0~4095
#if 1 //  2010-06-14
static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *xyz_cnt)
{
#ifndef KXTF9_I2C_XOUT_L
#define KXTF9_I2C_XOUT_L 0x06
#endif

		int err;
		u8 acc_data[6]; // xyz data bytes from hardware
		char buf, Res = 0, G_range = 0;
		int range = 0, sensitivity;
		int x_sign, y_sign, z_sign;

		int hw_cnt[3] ; // calculated count value
		int hw_mg[3]; // calculated mg value
//		int xyz_cnt[3];
		int xyz_mg[3]; // acceleration data by mg (last return value)
	
//		err = kxtf9_i2c_read(tf9, 0x1B, &buf, 1);
//		if(err < 0 ) {
//			dev_err(&tf9->client->dev, "can't get acceleration data, err=%d\n", err);
//			return err;
//		}
		NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &buf, 1);
		G_range = (buf & 0x18) >> 3;
		switch(G_range)
		{
		case 0:
			range = 2;
			break;
		case 1:
			range = 4;
			break;
		case 2:
			range = 8;
			break;
		default:
			break;
		}
	
//		err = kxtf9_i2c_read(tf9, KXTF9_XOUT_L, acc_data, 6);
//		if(err < 0){
//			dev_err(&tf9->client->dev, "can't get acceleration data, err=%d\n", err);
//			return err;
//		}
		NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, KXTF9_I2C_XOUT_L, &acc_data[0], 6);

		Res = buf & 0x40;
		switch(Res)
		{
		case 0x00: // 8-bit : low resolution state
			hw_cnt[0] = hw_mg[0] = ((int)acc_data[1]);
			x_sign = hw_mg[0] >> 7;	//1 = negative; 0 = positive
			if (x_sign == 1){
				hw_mg[0] = ((~(hw_mg[0]) + 0x01) & 0x0FF);
				hw_mg[0] = -(hw_mg[0]);
			}
			hw_cnt[1] = hw_mg[1] = ((int)acc_data[3]);
			y_sign = hw_mg[1] >> 7;	//1 = negative; 0 = positive
			if (y_sign == 1){
				hw_mg[1] = ((~(hw_mg[1]) + 0x01) & 0x0FF);	//2's complement
				hw_mg[1] = -(hw_mg[1]);
			}
			hw_cnt[2] = hw_mg[2] = ((int)acc_data[5]);
			z_sign = hw_mg[2] >> 7;	//1 = negative; 0 = positive
			if (z_sign == 1){
				hw_mg[2] = ((~(hw_mg[2]) + 0x01) & 0x0FF);	//2's complement
				hw_mg[2] = -(hw_mg[2]); 					
			}
			sensitivity = (256)/(2*range);
			/* calculate milli-G's */
			hw_mg[0] = 1000 * (hw_mg[0]) / sensitivity; 
			hw_mg[1] = 1000 * (hw_mg[1]) / sensitivity; 
			hw_mg[2] = 1000 * (hw_mg[2]) / sensitivity;

			if(hw_cnt[0] < 128) hw_cnt[0] += 128;
			else hw_cnt[0] -= 128;
			if(hw_cnt[1] < 128) hw_cnt[1] += 128;
			else hw_cnt[1] -= 128;
			if(hw_cnt[2] < 128) hw_cnt[2] += 128;
			else hw_cnt[2] -= 128;

			break;
		case 0x40: // 12-bit : high-resolution state
			hw_mg[0] = ((int)acc_data[0]) >> 4;
			hw_cnt[0] = hw_mg[0] = hw_mg[0] + (((int)acc_data[1]) << 4);
			x_sign = hw_mg[0] >> 11;	//1 = negative; 0 = positive
			if (x_sign == 1){
				hw_mg[0] = ((~(hw_mg[0]) + 0x01) & 0x0FFF); //2's complement
				hw_mg[0] = -(hw_mg[0]);
			}
			hw_mg[1] = ((int)acc_data[2]) >> 4;
			hw_cnt[1] = hw_mg[1]	= hw_mg[1]  + (((int)acc_data[3]) << 4);
			y_sign = hw_mg[1]  >> 11; 	//1 = negative; 0 = positive
			if (y_sign == 1){
				hw_mg[1]	= ((~(hw_mg[1] ) + 0x01) & 0x0FFF);	//2's complement
				hw_mg[1]	= -(hw_mg[1] );
			}
			hw_mg[2] = ((int)acc_data[4]) >> 4;
			hw_cnt[2] = hw_mg[2] = hw_mg[2] + (((int)acc_data[5]) << 4);
			z_sign = hw_mg[2] >> 11;	//1 = negative; 0 = positive
			if (z_sign == 1){
				hw_mg[2] = ((~(hw_mg[2]) + 0x01) & 0x0FFF); //2's complement
				hw_mg[2] = -(hw_mg[2]);
			}
			sensitivity = (4096)/(2*range);
			/* calculate milli-G's */
			hw_mg[0] = 1000 * (hw_mg[0]) / sensitivity; 
			hw_mg[1] = 1000 * (hw_mg[1]) / sensitivity; 
			hw_mg[2] = 1000 * (hw_mg[2]) / sensitivity;

			if(hw_cnt[0] < 2048) hw_cnt[0] += 2048;
			else hw_cnt[0] -= 2048;

			if(hw_cnt[1] < 2048) hw_cnt[1] += 2048;
			else hw_cnt[1] -= 2048;

			if(hw_cnt[2] < 2048) hw_cnt[2] += 2048;
			else hw_cnt[2] -= 2048;

			break;
		}
	
		xyz_mg[0] = ((tf9->negate_x) ? (-hw_mg[tf9->axis_map_x]) : (hw_mg[tf9->axis_map_x]));
		xyz_mg[1] = ((tf9->negate_y) ? (-hw_mg[tf9->axis_map_y]) : (hw_mg[tf9->axis_map_y]));
		xyz_mg[2] = ((tf9->negate_z) ? (-hw_mg[tf9->axis_map_z]) : (hw_mg[tf9->axis_map_z]));

		xyz_cnt[0] = ((tf9->negate_x) ? (-hw_cnt[tf9->axis_map_x]) : (hw_cnt[tf9->axis_map_x]));
		xyz_cnt[1] = ((tf9->negate_y) ? (-hw_cnt[tf9->axis_map_y]) : (hw_cnt[tf9->axis_map_y]));
		xyz_cnt[2] = ((tf9->negate_z) ? (-hw_cnt[tf9->axis_map_z]) : (hw_cnt[tf9->axis_map_z]));

	
	//	dev_info(&tf9->client->dev, "x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);
#if 1 /*DEBUG_ACCEL_DATA*/
	printk("[%s:%d] ## 1 ##  XYZ =  xyz_cnt[%d] %d (%d mg),  xyz_cnt[%d] %d(%d mg), xyz_cn[%d]t  %d(%d  mg) \n",
			__FUNCTION__,__LINE__,0,  xyz_cnt[0],xyz_mg[0],1,  xyz_cnt[1], xyz_mg[1],2, xyz_cnt[2], xyz_mg[2]);
#endif
		return err;

	return 0;
}

#else // 2010-06-14
static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *ret_xyz)
{
#ifndef KXTF9_I2C_XOUT_L
#define KXTF9_I2C_XOUT_L 0x06
#endif

	int err;

	int status, x_sign, y_sign, z_sign, sensitivity;
	char x_char;
	char ret[3] = {0, 0, 0};
	char xyz[6] = {0, 0, 0, 0, 0, 0};
	int x = 0, gx;
	int y = 0, gy;
	int z = 0, gz;
	char Res = 0;
	char G_range = 0;
	int range = 0;

	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &Res, 1);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &G_range, 1);
	G_range = G_range & 0x18;
	G_range = G_range >> 3;
	switch(G_range){
		case 0:
			range = 2;
			break;
		case 1:
			range = 4;
			break;
		case 2:
			range = 8;
			break;
		default:
			break;
	}
	Res = Res & 0x40;
	switch(Res)
	{
		case 0x00:	//low-resolution state
			NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, KXTF9_I2C_XOUT_L, &xyz[0], 6);
			ret_xyz[0] = x = ((int)xyz[1]);
			x_sign = x >> 7;	//1 = negative; 0 = positive
			if (x_sign == 1){
				x = ((~(x) + 0x01) & 0x0FF);
				x = -(x);
			}
			ret_xyz[1] = y = ((int)xyz[3]);
			y_sign = y >> 7;	//1 = negative; 0 = positive
			if (y_sign == 1){
				y = ((~(y) + 0x01) & 0x0FF);	//2's complement
				y = -(y);
			}
			ret_xyz[2] = z = ((int)xyz[5]);
			z_sign = z >> 7;	//1 = negative; 0 = positive
			if (z_sign == 1){
				z = ((~(z) + 0x01) & 0x0FF);	//2's complement
				z = -(z);						
			}
			sensitivity = (256)/(2*range);
			/* calculate milli-G's */
			gx = 1000 * (x) / sensitivity; 
			gy = 1000 * (y) / sensitivity; 
			gz = 1000 * (z) / sensitivity;
			break;
		case 0x40:	//high-resolution state
			NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, KXTF9_I2C_XOUT_L, &xyz[0], 6);
			x = ((int)xyz[0]) >> 4;
			ret_xyz[0] = x = x + (((int)xyz[1]) << 4);
			if(ret_xyz[0] < 2048) ret_xyz[0] += 2048;
			else ret_xyz[0] -= 2048;
			x_sign = x >> 11; 	//1 = negative; 0 = positive
			if (x_sign == 1){
				x = ((~(x) + 0x01) & 0x0FFF);	//2's complement
				x = -(x);
			}
			y = ((int)xyz[2]) >> 4;
			ret_xyz[1] = y = y + (((int)xyz[3]) << 4);
			if(ret_xyz[1] < 2048) ret_xyz[1] += 2048;
			else ret_xyz[1] -= 2048;
			y_sign = y >> 11; 	//1 = negative; 0 = positive
			if (y_sign == 1){
				y = ((~(y) + 0x01) & 0x0FFF);	//2's complement
				y = -(y);
			}
			z = ((int)xyz[4]) >> 4;
			ret_xyz[2] = z = z + (((int)xyz[5]) << 4);
			if(ret_xyz[2] < 2048) ret_xyz[2] += 2048;
			else ret_xyz[2] -= 2048;
			z_sign = z >> 11; 	//1 = negative; 0 = positive
			if (z_sign == 1){
				z = ((~(z) + 0x01) & 0x0FFF);	//2's complement
				z = -(z);
			}
			sensitivity = (4096)/(2*range);
			/* calculate milli-G's */
			gx = 1000 * (x) / sensitivity; 
			gy = 1000 * (y) / sensitivity; 
			gz = 1000 * (z) / sensitivity;
			break;
	}
#if DEBUG_ACCEL_DATA
	printk("[%s:%d] XYZ = (%d, %d mg), (%d, %d mg), (%d, %d mg) \n",
			__FUNCTION__,__LINE__, ret_xyz[0], gx, ret_xyz[1], gy, ret_xyz[2], gz);
#endif
	return 0;
}
#endif // end, 2010-06-14

static void kxtf9_report_values(struct kxtf9_data *tf9, int *xyz)
{
	input_report_abs(tf9->input_dev, ABS_X, xyz[0]);
	input_report_abs(tf9->input_dev, ABS_Y, xyz[1]);
	input_report_abs(tf9->input_dev, ABS_Z, xyz[2]);
	input_sync(tf9->input_dev);
}

static int kxtf9_enable(struct kxtf9_data *tf9)
{
	int err;
	int int_status = 0;
	u8 buf;

	if (!atomic_cmpxchg(&tf9->enabled, 0, 1)) {
		err = kxtf9_device_power_on(tf9);
		//err = kxtf9_i2c_read(tf9, INT_REL, &buf, 1);
		err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, INT_REL , &buf);
		if (err < 0) {
			printk("[%s:%d] error clearing interrupt: %d\n", __FUNCTION__,  __LINE__, err);
			/*dev_err(&tf9->client->dev,
					"error clearing interrupt: %d\n", err);*/
			atomic_set(&tf9->enabled, 0);
			return err;
		}
		if ((tf9->resume[RES_CTRL_REG1] & TPE) > 0) {
			//err = kxtf9_i2c_read(tf9, TILT_POS_CUR, &buf, 1);
			err = NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, TILT_POS_CUR , &buf );
			if (err < 0) {
				printk("[%s:%d] read err current tilt\n", __FUNCTION__, __LINE__);
				/*dev_err(&tf9->client->dev,
					"read err current tilt\n");*/
			int_status |= kxtf9_resolve_dir(tf9, buf);
			input_report_abs(tf9->input_dev, ABS_MISC, int_status);
			input_sync(tf9->input_dev);
			}
		}
		schedule_delayed_work(&tf9->input_work,
			msecs_to_jiffies(tf9->res_interval));
	}

	return 0;
}

static int kxtf9_disable(struct kxtf9_data *tf9)
{
	if (atomic_cmpxchg(&tf9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&tf9->input_work);
		kxtf9_device_power_off(tf9);
	}

	return 0;
}

static int kxtf9_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = kxtf9_misc_data;

	return 0;
}

static int kxtf9_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	/*struct tegra_acc_device_data *accel =
		(struct tegra_acc_device_data*)accel_dev;*/
	u8 ctrl[2] = { CTRL_REG1, PC1_OFF };
	int err;
	int tmp;
	int xyz[3] = { 0 };
	NvS32 x=0, y=0, z=0;
	struct kxtf9_data *tf9 = file->private_data;

	switch (cmd) {
	case KXTF9_IOCTL_GET_DELAY:
		tmp = tf9->res_interval;
		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			return -EFAULT;
		break;
	case KXTF9_IOCTL_SET_DELAY:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;
		if (tmp < 0)
			return -EINVAL;
		tf9->res_interval = max(tmp, tf9->pdata->min_interval);
		err = kxtf9_update_odr(tf9, tf9->res_interval);
		if (err < 0)
			return err;
		ctrl[0] = CTRL_REG3;
//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1] & 0x18;
//please check it		tf9->resume_state[RES_CURRENT_ODR] = ctrl[1];
		ctrl[1] = (ctrl[1] >> 1) | (ctrl[1] >> 3);
//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it		if (err < 0)
//please check it			return err;
//please check it		tf9->resume_state[RES_CTRL_REG3] = ctrl[1];
		break;
	case KXTF9_IOCTL_SET_ENABLE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;
		if (tmp < 0 || tmp > 1)
			return -EINVAL;

		if (tmp)
			kxtf9_enable(tf9);
		else
			kxtf9_disable(tf9);
		break;
	case KXTF9_IOCTL_GET_ENABLE:
		tmp = atomic_read(&tf9->enabled);
		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			return -EINVAL;
		break;
	case KXTF9_IOCTL_SET_TILT_ENABLE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;
		if (tmp < 0 || tmp > 1)
			return -EINVAL;
//please check it		if (tmp)
//please check it			tf9->resume_state[RES_CTRL_REG1] |= TPE;
//please check it
//please check it		else
//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~TPE);
//please check it		ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
//please check it		err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it		if (err < 0)
//please check it			return err;
		break;
	case KXTF9_IOCTL_SET_WAKE_ENABLE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;
		if (tmp < 0 || tmp > 1)
			return -EINVAL;
//please check it		if (tmp) {
//please check it			tf9->resume_state[RES_CTRL_REG1] |= (WUFE | B2SE);
//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it			if (err < 0)
//please check it				return err;
//please check it		} else {
//please check it			tf9->resume_state[RES_CTRL_REG1] &= (~WUFE & ~B2SE);
//please check it			ctrl[1] = tf9->resume_state[RES_CTRL_REG1];
//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it			if (err < 0)
//please check it				return err;
//please check it		}
		break;
	case KXTF9_IOCTL_SELF_TEST:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;
		if (tmp < 0 || tmp > 1)
			return -EINVAL;
		ctrl[0] = 0x3A;
		if (tmp) {
			ctrl[1] = 0xCA;
//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it			if (err < 0)
//please check it				return err;
		} else {
			ctrl[1] = 0x00;
//please check it			err = kxtf9_i2c_write(tf9, ctrl, 1);
//please check it			if (err < 0)
//please check it				return err;
		}
		break;
	case KXTF9_IOCTL_READ_ACCEL_XYZ:
		err=kxtf9_get_acceleration_data(tf9, xyz);
		/*NvOdmAccelGetAcceleration(
			accel_dev->hOdmAcr, &x, &y, &z);
		xyz[0] = x;	xyz[1] = y; xyz[2] = z;*/
#if 1 /*DEBUG_ACCEL_DATA*/
				printk("[%s:%d] ## 2 ##  XYZ = (%d, %d mg), (%d, %d mg), (%d, %d mg) \n",
						__FUNCTION__,__LINE__, xyz[0], xyz[0], xyz[1], xyz[1], xyz[2], xyz[2]);
#endif
		
		if (err < 0)
				return err;

		if (copy_to_user(argp, xyz, sizeof(int)*3))
			return -EINVAL;

		return err;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations kxtf9_misc_fops = {
	.owner = THIS_MODULE,
	.open = kxtf9_misc_open,
	.ioctl = kxtf9_misc_ioctl,
};

static struct miscdevice kxtf9_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtf9",
	.fops = &kxtf9_misc_fops,
};

static void kxtf9_input_work_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of((struct delayed_work *)work,
						struct kxtf9_data, input_work);
	int xyz[3] = { 0 };

	mutex_lock(&tf9->lock);

	if (kxtf9_get_acceleration_data(tf9, xyz) == 0)
		kxtf9_report_values(tf9, xyz);
	
	schedule_delayed_work(&tf9->input_work,
			msecs_to_jiffies(tf9->res_interval));
	mutex_unlock(&tf9->lock);
}

int kxtf9_input_open(struct input_dev *input)
{
	struct kxtf9_data *tf9 = input_get_drvdata(input);

	return kxtf9_enable(tf9);
}

void kxtf9_input_close(struct input_dev *dev)
{
	struct kxtf9_data *tf9 = input_get_drvdata(dev);

	kxtf9_disable(tf9);
}

static int kxtf9_input_init(struct kxtf9_data *tf9)
{
	int err;

	INIT_DELAYED_WORK(&tf9->input_work, kxtf9_input_work_func);
	tf9->input_dev = input_allocate_device();
	if (!tf9->input_dev) {
		err = -ENOMEM;
		dev_err(&tf9->client->dev, "input device allocate failed\n");
		goto err0;
	}
	tf9->input_dev->open = kxtf9_input_open;
	tf9->input_dev->close = kxtf9_input_close;

	input_set_drvdata(tf9->input_dev, tf9);

	set_bit(EV_ABS, tf9->input_dev->evbit);
	set_bit(ABS_MISC, tf9->input_dev->absbit);

	input_set_abs_params(tf9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	tf9->input_dev->name = "kxtf9_accel";

	err = input_register_device(tf9->input_dev);
	if (err) {
		dev_err(&tf9->client->dev,
			"unable to register input polled device %s: %d\n",
			tf9->input_dev->name, err);
		goto err1;
	}

	return 0;
err1:
	input_free_device(tf9->input_dev);
err0:
	return err;
}

static void kxtf9_input_cleanup(struct kxtf9_data *tf9)
{
	input_unregister_device(tf9->input_dev);
}

/* sysfs */
static ssize_t kxtf9_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", tf9->res_interval);
}

static ssize_t kxtf9_delay_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);

	tf9->res_interval = max(val, tf9->pdata->min_interval);
	kxtf9_update_odr(tf9, tf9->res_interval);

	return count;
}

static ssize_t kxtf9_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", atomic_read(&tf9->enabled));
}

static ssize_t kxtf9_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		kxtf9_enable(tf9);
	else
		kxtf9_disable(tf9);
	return count;
}

static ssize_t kxtf9_tilt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	u8 tilt;

	if (tf9->resume[RES_CTRL_REG1] & TPE) {
		//kxtf9_i2c_read(tf9, TILT_POS_CUR, &tilt, 1);
		NvOdmAccelerometerGetParameter(accel_dev->hOdmAcr, TILT_POS_CUR , &tilt );
		return sprintf(buf, "%d\n", kxtf9_resolve_dir(tf9, tilt));
	} else {
		return sprintf(buf, "%d\n", 0);
	}
}

static ssize_t kxtf9_tilt_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= TPE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~TPE);
	//kxtf9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, tf9->resume[RES_CTRL_REG1]);
	return count;
}

static ssize_t kxtf9_wake_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	u8 val = tf9->resume[RES_CTRL_REG1] & WUFE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t kxtf9_wake_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= WUFE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~WUFE);
	//kxtf9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, tf9->resume[RES_CTRL_REG1]);
	return count;
}

static ssize_t kxtf9_tap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	u8 val = tf9->resume[RES_CTRL_REG1] & TDTE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t kxtf9_tap_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= TDTE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~TDTE);
	//kxtf9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, CTRL_REG1, tf9->resume[RES_CTRL_REG1]);
	return count;
}

static ssize_t kxtf9_selftest_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	u8 ctrl = 0x00;
	if (val)
		ctrl = 0xCA;
	//kxtf9_i2c_write(tf9, 0x3A, &ctrl, 1);
	NvOdmAccelerometerSetParameter(accel_dev->hOdmAcr, 0x3A, ctrl);
	return count;
}

#if 0 //  2010-06-14
// e.g.
// #cat sys/bus/i2c/devices/0-000f/pcbmount
// EVB ->   axis_map = [0, 1, 2], negate = [0, 0, 0]
// Rev. A ->    axis_map = [1, 0, 2], negate = [0, 0, 0]
static ssize_t kxtf9_pcbmount_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "axis_map = [%d, %d, %d], negate = [%d, %d, %d] \n",
		tf9->axis_map_x, tf9->axis_map_y, tf9->axis_map_z,
		tf9->negate_x, tf9->negate_y, tf9->negate_z );
}

#endif
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kxtf9_delay_show, kxtf9_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, kxtf9_enable_show,
						kxtf9_enable_store);
static DEVICE_ATTR(tilt, S_IRUGO|S_IWUSR, kxtf9_tilt_show, kxtf9_tilt_store);
static DEVICE_ATTR(wake, S_IRUGO|S_IWUSR, kxtf9_wake_show, kxtf9_wake_store);
static DEVICE_ATTR(tap, S_IRUGO|S_IWUSR, kxtf9_tap_show, kxtf9_tap_store);
static DEVICE_ATTR(selftest, S_IWUSR, NULL, kxtf9_selftest_store);
#if 0 // 2010-06-14
static DEVICE_ATTR(pcbmount, S_IRUGO, kxtf9_pcbmount_show, NULL);
#endif

static struct attribute *kxtf9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_tilt.attr,
	&dev_attr_wake.attr,
	&dev_attr_tap.attr,
	&dev_attr_selftest.attr,
#if 0 // 2010-06-14
	&dev_attr_pcbmount.attr,
#endif
	NULL
};

static struct attribute_group kxtf9_attribute_group = {
	.attrs = kxtf9_attributes
};
/* /sysfs */


static NvS32 __init tegra_acc_probe(struct platform_device *pdev)
{
	struct tegra_acc_device_data *accelerometer = NULL;
	struct input_dev *input_dev = NULL;
	struct kxtf9_platform_data *accpdata =  kzalloc(sizeof(*accpdata), GFP_KERNEL);
	NvS32 err;

	struct kxtf9_data *tf9 = kzalloc(sizeof(*tf9), GFP_KERNEL);
	if (tf9 == NULL) {
		/*dev_err(&client->dev,
			"failed to allocate memory for module data\n");*/
		printk("failed to allocate memory for module data\n");	
		err = -ENOMEM;
		goto err0;
	}
#if 1 // 2010-06-14
/* Rev. A. EVB :*/
	  tf9->axis_map_x = 0;
	  tf9->axis_map_y = 1;
	  tf9->axis_map_z = 2;

	  tf9->negate_x = 0;
	  tf9->negate_y = 0;
	  tf9->negate_z = 0;
// Rev. A.
/*
	tf9->axis_map_x = 1;
	tf9->axis_map_y = 0;
	tf9->axis_map_z = 2;
	
	tf9->negate_x = 0;
	tf9->negate_y = 1;
	tf9->negate_z = 0;
	*/
#endif
//**************************//	
	accelerometer = kzalloc(sizeof(*accelerometer), GFP_KERNEL);
	if (accelerometer == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	accel_dev = accelerometer;

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	accel_dev->input_dev = input_dev;

	err = open_def_odm_accl();
		if (!err) {
			pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
			goto allocate_dev_fail;
		}

#if 0 // yongjae
{
	NvBool err;
	unsigned char buf, val;
	int status, sensitivity;
	int x = 0;
	int y = 0;
	int z = 0;
	int x_sign, y_sign, z_sign;
	char xyz[6] = {0, 0, 0, 0, 0, 0};
	char Res = 0;
	char G_range = 0;
	int range = 0;
	int gx, gy, gz;

	mdelay(200);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x0F, &buf, 1);
	printk("-YJ- ============> WHO_AM_I = 0x%x \n", buf);

	val = 0x40;
	NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1B, &val, 1);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1D, &buf, 1);
	val = buf | (1<<7);
	NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1D, &val, 1);
	mdelay(200);

	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x0F, &buf, 1);
	printk("-YJ- ============> WHO_AM_I = 0x%x \n", buf);

	val = 0xc0;
	NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1B, &val, 1);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &buf, 1);
	printk("-YJ- ============> CTRL_REG1 = 0x%x \n", buf);

	//determine if in the low resolution or high resolution state
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &Res, 1);
	printk("-YJ- ============> Res = 0x%x \n", Res);
	NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x1B, &G_range, 1);
	printk("-YJ- ============> G_range = 0x%x \n", G_range);
	G_range = G_range & 0x18;
	G_range = G_range >> 3;
	switch(G_range){
	case 0:
		range = 2;
		break;
	case 1:
		range = 4;
		break;
	case 2:
		range = 8;
		break;
	default:
		break;
	}
	Res = Res & 0x40;
	printk("-YJ- ============> range = 0x%x \n", range);

	while(1)
	{
		NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x06, &xyz[0], 6); // XOUT_L
		x = ((int)xyz[0]) >> 4;
		x = x + (((int)xyz[1]) << 4);
		x_sign = x >> 11;	//1 = negative; 0 = positive
		if (x_sign == 1){
			x = ((~(x) + 0x01) & 0x0FFF);	//2's complement
			x = -(x);
		}
		y = ((int)xyz[2]) >> 4;
		y = y + (((int)xyz[3]) << 4);
		y_sign = y >> 11;	//1 = negative; 0 = positive
		if (y_sign == 1){
			y = ((~(y) + 0x01) & 0x0FFF);	//2's complement
			y = -(y);
		}
		z = ((int)xyz[4]) >> 4;
		z = z + (((int)xyz[5]) << 4);
		z_sign = z >> 11;	//1 = negative; 0 = positive
		if (z_sign == 1){
			z = ((~(z) + 0x01) & 0x0FFF);	//2's complement
			z = -(z);
		}
		sensitivity = (4096)/(2*range);
		/* calculate milli-G's */
		gx = 1000 * (x) / sensitivity; 
		gy = 1000 * (y) / sensitivity; 
		gz = 1000 * (z) / sensitivity;
		printk("[YJ] X/Y/Z = %d mg, %d mg, %d mg\n", gx, gy, gz);
		mdelay(200);
	}
}

#endif

#if  0	/* ###  Gyro + Accel + Compass  ### */ 
	accelerometer->input_dev = input_dev;
	set_bit(EV_SYN, accelerometer->input_dev->evbit);
	set_bit(EV_KEY, accelerometer->input_dev->evbit);
	set_bit(EV_ABS, accelerometer->input_dev->evbit);

	input_set_abs_params(accelerometer->input_dev, ABS_X, 
	accelerometer->min_data.x, 
	accelerometer->max_data.x, 0, 0);
	input_set_abs_params(accelerometer->input_dev, ABS_Y,
	accelerometer->min_data.y, 
	accelerometer->max_data.y, 0, 0);
	input_set_abs_params(accelerometer->input_dev, ABS_Z,
	accelerometer->min_data.z, 
	accelerometer->max_data.z, 0, 0);
	
	platform_set_drvdata(pdev, accelerometer);
	
	input_dev->name = "accelerometer_tegra";
	err = input_register_device(input_dev);
	if (err) {
	pr_err("tegra_acc_probe: Unable to register %s\
			input device\n", input_dev->name);
		goto input_register_device_failed;
		}
#endif 
//**************************//	
	mutex_init(&tf9->lock);
	mutex_lock(&tf9->lock);
	//tf9->client = client;
	//i2c_set_clientdata(client, tf9);
	//INIT_WORK(&tf9->irq_work, kxtf9_irq_work_func);
	
	tf9->pdata = kmalloc(sizeof(*tf9->pdata), GFP_KERNEL);
	if (tf9->pdata == NULL)
		goto err1;

	#if 1
	
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_delay);
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_enable);
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_tilt);
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_wake);
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_tap);
	err = device_create_file(&accel_dev->input_dev->dev, &dev_attr_selftest);
	#else
	err = sysfs_create_group(&accel_dev->input_dev->dev, &kxtf9_attribute_group);
	#endif
	if (err)
		goto err1;
	printk("TEST2-98  %s \n",__FUNCTION__);
/*
	memcpy(tf9->pdata, accpdata, sizeof(*tf9->pdata));
	if (tf9->pdata->init) {
		err = tf9->pdata->init();
		if (err < 0)
			goto err2;
	}*/
	printk("TEST2-97  %s \n",__FUNCTION__);

	//tf9->irq = gpio_to_irq(tf9->pdata->gpio);
	printk("TEST1--22  %s \n",__FUNCTION__);

	memset(tf9->resume, 0, ARRAY_SIZE(tf9->resume));
	tf9->resume[RES_DATA_CTRL] = tf9->pdata->data_odr_init;
	tf9->resume[RES_CTRL_REG1] = tf9->pdata->ctrl_reg1_init;
	tf9->resume[RES_INT_CTRL1] = tf9->pdata->int_ctrl_init;
	tf9->resume[RES_TILT_TIMER] = tf9->pdata->tilt_timer_init;
	tf9->resume[RES_CTRL_REG3] = tf9->pdata->engine_odr_init;
	tf9->resume[RES_WUF_TIMER] = tf9->pdata->wuf_timer_init;
	tf9->resume[RES_WUF_THRESH] = tf9->pdata->wuf_thresh_init;
	tf9->resume[RES_TDT_TIMER] = tf9->pdata->tdt_timer_init;
	tf9->resume[RES_TDT_H_THRESH] = tf9->pdata->tdt_h_thresh_init;
	tf9->resume[RES_TDT_L_THRESH] = tf9->pdata->tdt_l_thresh_init;
	tf9->resume[RES_TAP_TIMER] = tf9->pdata->tdt_tap_timer_init;
	tf9->resume[RES_TOTAL_TIMER] = tf9->pdata->tdt_total_timer_init;
	tf9->resume[RES_LAT_TIMER] = tf9->pdata->tdt_latency_timer_init;
	tf9->resume[RES_WIN_TIMER]    = tf9->pdata->tdt_window_timer_init;
	tf9->res_interval = tf9->pdata->poll_interval;
/*
	err = kxtf9_device_power_on(tf9);
	if (err < 0)
		goto err3;
	atomic_set(&tf9->enabled, 1);

	err = kxtf9_verify(tf9);
	if (err < 0) {
		printk("unresolved i2c client\n");
//		dev_err(&client->dev, "unresolved i2c client\n");
		goto err4;
	}
	printk("TEST1--33  %s \n",__FUNCTION__);

	err = kxtf9_update_g_range(tf9, tf9->pdata->g_range);
	if (err < 0)
		goto err4;
	printk("TEST1--44  %s \n",__FUNCTION__);

	err = kxtf9_update_odr(tf9, tf9->res_interval);
	if (err < 0)
		goto err4;
	printk("TEST1--55  %s \n",__FUNCTION__);

	err = kxtf9_input_init(tf9);
	if (err < 0)
		goto err4;*/
	kxtf9_misc_data = tf9;
	err = misc_register(&kxtf9_misc_device);
	if (err < 0) {
		printk("tf9_device register failed\n");
		//dev_err(&client->dev, "tf9_device register failed\n");
		goto err5;
	}
		printk("TEST1--66  %s \n",__FUNCTION__);
	//kxtf9_device_power_off(tf9);
	//atomic_set(&tf9->enabled, 0);
#if 0
	err = request_irq(tf9->irq, kxtf9_isr,
			IRQF_TRIGGER_RISING | IRQF_DISABLED, "kxtf9-irq", tf9);
	if (err < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, err);
		goto err6;
	}

	disable_irq_nosync(tf9->irq);
#endif

	mutex_unlock(&tf9->lock);

	return 0;

/*err6:
	kxtf9_input_cleanup(tf9);*/
err5:	
	printk("TEST1--end8	%s \n",__FUNCTION__);

	misc_deregister(&kxtf9_misc_device);
err4:
			printk("TEST1--end7  %s \n",__FUNCTION__);
	kxtf9_device_power_off(tf9);
err3:
			printk("TEST1--end6  %s \n",__FUNCTION__);
	if (tf9->pdata->exit)
		tf9->pdata->exit();
err2:
			printk("TEST1--end5  %s \n",__FUNCTION__);
	kfree(tf9->pdata);
	//sysfs_remove_group(&g_client->dev.kobj, &kxtf9_attribute_group);
err1:
			printk("TEST1--end4  %s \n",__FUNCTION__);
	mutex_unlock(&tf9->lock);
	kfree(tf9);
input_register_device_failed:
			printk("TEST1--end3  %s \n",__FUNCTION__);
	accelerometer->bThreadAlive = 0;	
allocate_dev_fail:
			printk("TEST1--end2  %s \n",__FUNCTION__);
		close_odm_accl();
		input_free_device(input_dev);
		kfree(accelerometer);
		accelerometer = 0;
		err = -ENOMEM;
err0:
		printk("TEST1--end  %s \n",__FUNCTION__);
	return err;
}

static NvS32 tegra_acc_remove(struct platform_device *pdev)
{
	//remove_sysfs_entry();
	input_unregister_device(g_input_dev);
	return 0;
}

static NvS32 tegra_acc_resume(struct platform_device *pdev)
{
	struct kxtf9_data *tf9 = (void *)kxtf9_misc_data;
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	kxtf9_device_power_on(tf9);

	//remove_sysfs_entry();
	//input_unregister_device(g_input_dev);
	return 0;
}

static NvS32 tegra_acc_suspend(struct platform_device *pdev)
{
	struct kxtf9_data *tf9 = (void *)kxtf9_misc_data;
	printk("=============>  [%s,%d]\n",__FUNCTION__, __LINE__);
	kxtf9_device_power_off(tf9);

	//remove_sysfs_entry();
	//input_unregister_device(g_input_dev);
	return 0;
}

static struct platform_driver tegra_acc_driver = {
	.probe	= tegra_acc_probe,
	.remove	= tegra_acc_remove,
	.resume = tegra_acc_resume,
	.suspend = tegra_acc_suspend,
	.driver	= {
		.name = "tegra_accelerometer",
	},
};

static NvS32 __devinit tegra_acc_init(void)
{
	printk("TEST1  %s \n",__FUNCTION__);

	return platform_driver_register(&tegra_acc_driver);
}

static void __exit tegra_acc_exit(void)
{
	printk("TEST1  %s \n",__FUNCTION__);

	platform_driver_unregister(&tegra_acc_driver);
}

module_init(tegra_acc_init);
module_exit(tegra_acc_exit);

MODULE_DESCRIPTION("Tegra ODM Accelerometer driver");
#if 0
static int __devexit kxtf9_remove(struct i2c_client *client)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	free_irq(tf9->irq, tf9);
	gpio_free(tf9->pdata->gpio);
	misc_deregister(&kxtf9_misc_device);
	kxtf9_input_cleanup(tf9);
	kxtf9_device_power_off(tf9);
	if (tf9->pdata->exit)
		tf9->pdata->exit();
	kfree(tf9->pdata);
	sysfs_remove_group(&client->dev.kobj, &kxtf9_attribute_group);
	kfree(tf9);

	return 0;
}

#ifdef CONFIG_PM
static int kxtf9_resume(struct i2c_client *client)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	return kxtf9_enable(tf9);
}

static int kxtf9_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	return kxtf9_disable(tf9);
}
#endif

static const struct i2c_device_id kxtf9_id[] = {
	{NAME, 0},
	{},
};

#endif
