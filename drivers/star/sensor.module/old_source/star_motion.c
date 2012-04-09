/*
 *  star_motion.c
 *  star motion sensor driver (Accelerometer, Gyroscope Sensor)
 *  Copyright (C) 2010 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/string.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>

#include <nvodm_gyroscope_accel.h>
#include <nvodm_gyro_accel_kxtf9.h>
#include "mach/lprintk.h"

#include "gyro_accel.h"
#include "mpu3050.h"
#include "kxtf9.h"
#include "star_sensor.h"

#define  DEBUG 0
#define  MISC_DYNAMIC_MINOR		 	255
#define  MAX_MOTION_DATA_LENGTH	 	10
#define  MIN_MOTION_POLLING_TIME    10
//#define  MIN_MOTION_POLLING_TIME    70//  motion daemon update every 70mS
//#define MAX_LGE_ACCEL_SAMPLERATE    7  /* for JSR256 API */

#ifdef CONFIG_HAS_EARLYSUSPEND // wkkim : temporary early suspend apply
#include <linux/earlysuspend.h>
static struct early_suspend    early_suspend;
#endif

extern atomic_t accel_flag;


static atomic_t tilt_flag;
static atomic_t tap_flag;
static atomic_t shake_flag;
static atomic_t snap_flag;
static atomic_t flip_flag;
static atomic_t gyro_flag;
//static atomic_t accel_delay;
static atomic_t tilt_delay;
static atomic_t gyro_delay;
static atomic_t	jsr256_flag;
static atomic_t suspend_flag;
static atomic_t composite[12];

static atomic_t	tilt_roll, tilt_pitch, tilt_yaw;
static atomic_t	accel_x, accel_y, accel_z;
static atomic_t	gyro_x, gyro_y, gyro_z;

struct star_motion_device {
	NvOdmGyroAccelDeviceHandle	hOdmGyroAccel;
	struct input_dev  		    *input_dev;
	NvU32						freq;
	NvBool						bThreadAlive;
	NvBool						show_log;
	NvOdmGyroAccelIntType		IntType;
	NvOdmGyroAccelAxisType		IntMotionAxis;
	NvOdmGyroAccelAxisType		IntTapAxis;

	int  						irq;
	int  						use_irq;
};

static struct star_motion_device   *star_motion_dev = NULL;
/* jay.sim */
//void magnetic_input_report(int *); /* wkkim magnetic repot */

#define write_lock(lock)		_write_lock(lock)
#define read_lock(lock)			_read_lock(lock)
rwlock_t getbuflock;
//static unsigned char accelrwbuf[200] = {0,};    /* MPU3050 i2c MAX data length */
static unsigned char rwbuf[200] = {0,};     	 /* MPU3050 i2c MAX data length */

static int  flip_test_count = 0;
static int  gyro_sleep_mode = 0;

/** Function to close the ODM device. This function will help in switching
 * between power modes
 */
void close_odm_gyro_accel(void)
{
	NvOdmGyroAccelClose(star_motion_dev->hOdmGyroAccel);
	star_motion_dev->hOdmGyroAccel = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous
 * settings will be lost. This function will help in switching
 * between power modes
 */
int open_def_odm_gyro_accel(void)
{
	NvS32 err = -1;
	err = NvOdmGyroAccelOpen(&(star_motion_dev->hOdmGyroAccel));
	//lprintk(" ##  open_def_odm_gyro_accel  ##\n");
	if (!err) {
		err = -ENODEV;
#if DEBUG
		lprintk("open_def_odm_gyro_accel: NvOdmGyroAccelOpen failed\n");
#endif
		return err;
	}

	return err;
}

/*---------------------------------------------------------------------------
  motion_send_event function
  ---------------------------------------------------------------------------*/
//motion_send_accel_detection
void motion_send_accel_detection(int accelx,int accely,int accelz)
{
	//lprintk("[Gyro_accel][%s:%d] %d %d %d\n",__FUNCTION__, __LINE__,accelx , accely, accelz );

	if (atomic_read(&accel_flag)) {
		input_report_abs(star_motion_dev->input_dev,ABS_X, accelx);
		input_report_abs(star_motion_dev->input_dev,ABS_Y, accely);
		input_report_abs(star_motion_dev->input_dev,ABS_Z, accelz);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tilt_detection(int yaw,int pitch,int roll)
{
	/*DY2*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&tilt_flag)) {
		/*DY*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_RX, yaw);
		input_report_rel(star_motion_dev->input_dev,REL_RY, pitch);
		input_report_rel(star_motion_dev->input_dev,REL_RZ, roll);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_composite_detection(int *value)
{
	int buf[12] = {0,};

	memcpy(buf, value, sizeof(int) * 12);

	//	lprintk("composite %d %d %d ... %d %d %d\n", buf[0],buf[1],buf[2],buf[9],buf[10],buf[11]);
	if (atomic_read(&tilt_flag))
	{
		input_report_abs(star_motion_dev->input_dev, ABS_GAS, buf[0]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1X, buf[1]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT1Y, buf[2]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT2X, buf[3]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT2Y, buf[4]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT3X, buf[5]);
		input_report_abs(star_motion_dev->input_dev, ABS_HAT3Y, buf[6]);
		input_report_abs(star_motion_dev->input_dev, ABS_TILT_X, buf[7]);
		input_report_abs(star_motion_dev->input_dev, ABS_TILT_Y, buf[8]);
		input_report_abs(star_motion_dev->input_dev, ABS_TOOL_WIDTH, buf[9]);
		input_report_abs(star_motion_dev->input_dev, ABS_VOLUME, buf[10]);
		input_report_abs(star_motion_dev->input_dev, ABS_MISC, buf[11]);
		//input_report_rel(star_motion_dev->input_dev,REL_MISC,buf[12]);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_gyro_detection(int gyro_x,int gyro_y,int gyro_z)
{
	/*DY2*///lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&gyro_flag)) {
		/*DY*///lprintk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
		input_report_rel(star_motion_dev->input_dev,REL_Z, gyro_x);
		input_report_rel(star_motion_dev->input_dev,REL_MISC, gyro_y);
		input_report_rel(star_motion_dev->input_dev,REL_MAX, gyro_z);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_tap_detection(int type,int direction)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if(atomic_read(&tap_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		/*Test*///type= 111; direction = 222;
		input_report_rel(star_motion_dev->input_dev, REL_X, type);
		input_report_rel(star_motion_dev->input_dev, REL_Y, direction);

		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_flip_detection(int value)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&flip_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_WHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_shake_detection(int value)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&shake_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev, REL_HWHEEL, value);
		input_sync(star_motion_dev->input_dev);
	}
}

void motion_send_snap_detection(int direction)
{
#if DEBUG
	/*DY2*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	if (atomic_read(&snap_flag)) {
#if DEBUG
		/*DY*/lprintk(D_SENSOR,"[Gyro_accel][%s:%d]\n", __FUNCTION__, __LINE__);
#endif
		input_report_rel(star_motion_dev->input_dev,REL_DIAL,direction);
		input_sync(star_motion_dev->input_dev);
	}
}


/*---------------------------------------------------------------------------
  ioctl command for heaven motion daemon process
  ---------------------------------------------------------------------------*/
static int star_motion_open(struct inode *inode, struct file *file)
{
	int status = -1;
	status = nonseekable_open(inode,file);

	return status;
}
static int star_motion_release(struct inode *inode, struct file *file)
{
	//lprintk("motion close\n");
	return 0;
}
static int count = 0 ;
static int star_motion_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char data[MAX_MOTION_DATA_LENGTH]={0,};
	unsigned char tempbuf[200] = {0,};     /* MPU3050 i2c MAX data length */
	unsigned char value;

	int buf[13] = {0,};
	int flag = 0;
	int delay = 0;
	int onoff_flag = 0;
	int ret = 0;
	int i = 0;

	switch (cmd) {
		case MOTION_IOCTL_ENABLE_DISABLE:
			/*
				0: disable sensor
				1: orientation (tilt)
				2: accelerometer
				3: tap
				4: shake
			*/

			//lprintk(".............star_motion_ioctl................\n");
			flag = STAR_SENSOR_NONE;

			if (atomic_read(&accel_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_ACCELEROMETER;
			}

			if (atomic_read(&tilt_flag)) {
				//lprintk(".............if(atomic_read(&tilt_flag)){................\n");
				flag |= STAR_TILT;
			}

			if (atomic_read(&gyro_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_GYRO;
			}

			if (atomic_read(&tap_flag)) {
				//lprintk(".............if(atomic_read(&tap_flag)){................\n");
				flag |= STAR_TAP;
			}

			if (atomic_read(&flip_flag)) {
				//lprintk(".............if(atomic_read(&flip_flag)){................\n");
				flag |= STAR_FLIP;
			}

			if (atomic_read(&shake_flag)) {
				//lprintk(".............if(atomic_read(&shake_flag)){................\n");
				flag |= STAR_SHAKE;
			}

			if (atomic_read(&snap_flag)) {
				//lprintk(".............if(atomic_read(&snap_flag)){................\n");
				flag |= STAR_SNAP;
			}

			if (atomic_read(&compass_flag)) {
				//lprintk(".............if(atomic_read(&gyro_flag)){................\n");
				flag |= STAR_COMPASS;
			}

			if (copy_to_user(argp,&flag, sizeof(flag))) {
				//lprintk(".............MOTION_IOCTL_SNAP................\n");
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_ACCEL_RAW:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = accel_x,  accel_y,  accel_z;	 */

			atomic_set(&accel_x, buf[0]);
			atomic_set(&accel_y, buf[1]);
			atomic_set(&accel_z, buf[2]);
#if 0 /*ACCEL_REPORT*/
			motion_send_accel_detection(buf[0],buf[1],buf[2]);
#endif
			//lprintk(".............MOTION_IOCTL_TILT................\n");
			break;
		case MOTION_IOCTL_TILT:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = roll,  pitch,  yaw;	 */

			atomic_set(&tilt_yaw, buf[0]);
			atomic_set(&tilt_pitch, buf[1]);
			atomic_set(&tilt_roll, buf[2]);
			//lprintk(".............MOTION_IOCTL_TILT................\n");
			break;
		case MOTION_IOCTL_COMPOSITE:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}

			for (i = 0; i < 12; i++) {
				atomic_set(&composite[i], buf[i]);
			}

			//motion_send_composite_detection(buf);
			break;
		case MOTION_IOCTL_GYRO_RAW:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 	buf[0], [1], [2] = gyro_x,  gyro_y,  gyro_z; */

			atomic_set(&gyro_x, buf[0]);
			atomic_set(&gyro_y, buf[1]);
			atomic_set(&gyro_z, buf[2]);

			//motion_send_tilt_detection(buf[0],buf[1],buf[2]);
			//lprintk(".............MOTION_IOCTL_GYRO_RAW................\n");
			break;
#if 0
		case MOTION_IOCTL_MAGNETIC_RAW: /* wkkim add to recieve compass raw value */
			if (copy_from_user(buf, argp, sizeof(int) * 3)) {
				return -EFAULT;
			}
			//magnetic_input_report(buf);
			atomic_set(&mag_x, buf[0]);
			atomic_set(&mag_y, buf[1]);
			atomic_set(&mag_z, buf[2]);
			break;
#endif
		case MOTION_IOCTL_TAP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = type;
			   buf[1] = direction;
			   */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_TAP................\n");
#endif
			motion_send_tap_detection(buf[0], buf[1]);
			break;
		case MOTION_IOCTL_FLIP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_FLIP................\n");
#endif
			motion_send_flip_detection(buf[0]);
			break;
		case MOTION_IOCTL_SHAKE:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/* 			buf[0] = event;   		  */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_SHAKE................\n");
#endif
			motion_send_shake_detection(buf[0]);
			break;
		case MOTION_IOCTL_SNAP:
			if (copy_from_user(&buf, argp, sizeof(buf))) {
				return -EFAULT;
			}
			/*
			   buf[0] = direction;
			   */
#if DEBUG
			lprintk(D_SENSOR,".............MOTION_IOCTL_SNAP................\n");
#endif
			motion_send_snap_detection(buf[0]);
			break;
		case MOTION_IOCTL_SENSOR_DELAY:
			delay = atomic_read(&tilt_delay);

			//lprintk("MOTION_IOCTL_SENSOR_DELAY[%d]",delay);

			if (copy_to_user(argp, &delay, sizeof(delay))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_SENSOR_SUSPEND_RESUME:
			onoff_flag = atomic_read(&suspend_flag);

			if (copy_to_user(argp, &onoff_flag, sizeof(onoff_flag))) {
				return -EFAULT;
			}
			break;
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE:
			//lprintk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_MODE................\n");
			motion_sensor_power_off();

			//twl4030_i2c_write_u8(0x13, 0x00,0x1b );
			//msleep(100);
			break;
		case MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP:
			//lprintk(".............MOTION_IOCTL_ACCEL_COMPASS_SLEEP_WAKE_UP................\n");
			motion_sensor_power_on();
			break;
		default:
			break;
	}

	return 0;

}

static struct file_operations  star_motion_fops = {
	.owner    = THIS_MODULE,
	.open     = star_motion_open,
	.release  = star_motion_release,
	.ioctl    = star_motion_ioctl,
};

static struct miscdevice  star_motion_misc_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= STAR_MOTION_IOCTL_NAME,
	.fops	= &star_motion_fops,
};


static NvS32 star_motion_resume(struct platform_device *pdev);
static NvS32 star_motion_suspend(struct platform_device *pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gyro_early_suspend(struct early_suspend *es)
{
#if DEBUG
	lprintk(D_SENSOR, "--->> Gyro early suspend\n");
#endif
	//gyro_sleep_mode = 1;
	//mdelay(200);
	star_motion_suspend(NULL);
	return;

}

static void gyro_late_resume(struct early_suspend *es)
{
#if DEBUG
	lprintk(D_SENSOR, "--->> Gyro early resume\n");
#endif
	star_motion_resume(NULL);
	//gyro_sleep_mode = 0;
	return;
}
#endif


/* TODO */
/*---------------------------------------------------------------------------
  platform device
  ---------------------------------------------------------------------------*/
static int __init star_motion_probe(struct platform_device *pdev)
{
	int err = 0;
	unsigned char value = 0;
	struct device *dev = &pdev->dev;
	struct star_motion_device *gyroscope_accel = NULL;
	struct input_dev *input_dev = NULL;

#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif

	gyroscope_accel = kzalloc(sizeof(*gyroscope_accel), GFP_KERNEL);
	star_motion_dev = gyroscope_accel;

#if DEBUG
	lprintk(KERN_INFO"%s: probe start\n", __func__);
#endif
	/*---------------------------------------------------------------------------
	  register misc device
	  ---------------------------------------------------------------------------*/
	err = misc_register(&star_motion_misc_device);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto exit_misc_device_register_failed;
	}

	/* Accel : jay.sim */
#if 0
	err = misc_register(&star_accel_misc_device);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_misc_device register failed\n");
#endif
		goto exit_misc_device_register_failed;
	}
#endif
	/**/

	/* Common : jay.sim */
	/*---------------------------------------------------------------------------
	  register input device
	  ---------------------------------------------------------------------------*/
	star_motion_dev->input_dev = input_allocate_device();
	if (star_motion_dev->input_dev == NULL) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed\n");
#endif
		goto err_input_allocate1;
	}

	star_motion_dev->input_dev->name = STAR_MOTION_INPUT_NAME;

#if 1
	set_bit(EV_SYN,star_motion_dev->input_dev->evbit);
	set_bit(EV_REL,star_motion_dev->input_dev->evbit);


	set_bit(REL_Z,star_motion_dev->input_dev->relbit);  	// gyro_x
	set_bit(REL_MISC,star_motion_dev->input_dev->relbit);  // gyro_y
	set_bit(REL_MAX,star_motion_dev->input_dev->relbit);  	// gyro_z
#endif

	platform_set_drvdata(pdev, gyroscope_accel);
	set_bit(EV_ABS,star_motion_dev->input_dev->evbit);
	input_set_abs_params(star_motion_dev->input_dev, ABS_X, -2000, 2000, 0, 0); /* x-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Y, -2000, 2000, 0, 0); /* y-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Z, -2000, 2000, 0, 0); /* z-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_GAS, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT1Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT2Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_HAT3Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_X, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TILT_Y, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_TOOL_WIDTH, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_VOLUME, 0, 0, 0, 0);
	input_set_abs_params(star_motion_dev->input_dev, ABS_MISC, 0, 0, 0, 0);

#if 0

	input_set_abs_params(star_motion_dev->input_dev, ABS_X, -2000, 2000, 0, 0); /* x-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Y, -2000, 2000, 0, 0); /* y-axis acceleration */
	input_set_abs_params(star_motion_dev->input_dev, ABS_Z, -2000, 2000, 0, 0); /* z-axis acceleration */

	//set_bit(ABS_X,star_motion_dev->input_dev->relbit);  // TAP - Type
	//set_bit(ABS_Y,star_motion_dev->input_dev->relbit);  // TAP - Direction
	//set_bit(ABS_Z,star_motion_dev->input_dev->relbit);  // TILT - Roll

	input_set_abs_params(compass_dev->input_dev, ABS_THROTTE, 0, 360, 0, 0);/* pitch */
	input_set_abs_params(compass_dev->input_dev, ABS_GAS, 	-180, 180, 0, 0); /* roll */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT1X, 	-90, 90, 0, 0);	/* status of magnetic sensor */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT1Y, 	0, 5, 0, 0); 	/* x-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT2X, 	-2000, 2000, 0, 0); 	/* y-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT2Y, 	-2000, 2000, 0, 0); 	/* z-axis acceleration */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT3X, 	-2000, 2000, 0, 0); 	/* x-axis of raw magnetic vector */
	input_set_abs_params(compass_dev->input_dev, ABS_HAT3Y, 	-3000, 3000, 0, 0); 	/* y-axis of raw magnetic vector */
#endif
	set_bit(REL_X, star_motion_dev->input_dev->relbit);  // TAP - Type
	set_bit(REL_Y, star_motion_dev->input_dev->relbit);  // TAP - Direction
	set_bit(REL_RX, star_motion_dev->input_dev->relbit);  // TILT - Yaw
	set_bit(REL_RY, star_motion_dev->input_dev->relbit);  // TILT - Pitch
	set_bit(REL_RZ, star_motion_dev->input_dev->relbit);  // TILT - Roll
	//set_bit(REL_RX,star_motion_dev->input_dev->relbit);  // TILT - Roll
	//set_bit(REL_RY,star_motion_dev->input_dev->relbit);  // TILT - PITCH
	//set_bit(REL_RZ,star_motion_dev->input_dev->relbit);  // TILT - Yaw
	set_bit(REL_HWHEEL, star_motion_dev->input_dev->relbit); // SHAKE
	set_bit(REL_DIAL, star_motion_dev->input_dev->relbit);   // SNAP - Direction
	set_bit(REL_WHEEL, star_motion_dev->input_dev->relbit);  // FLIP

	err = input_register_device(star_motion_dev->input_dev);
	if (err) {
#if DEBUG
		lprintk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed \n");
#endif
		goto err_input_allocate1;
	}
	/**/

	/*---------------------------------------------------------------------------
	  init. sysfs
	  ---------------------------------------------------------------------------*/
	if ((err = sysfs_create_group(&dev->kobj, &star_motion_group))) {
#if DEBUG
		lprintk("[motion_sensor] sysfs_create_group FAIL \n");
#endif
		goto err_sysfs_create;
	}

	/*---------------------------------------------------------------------------
	  init. timer
	  ---------------------------------------------------------------------------*/
#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
#endif

	return 0;
allocate_dev_fail:
	printk("##  sensor: allocated_device_failed\n");
	close_odm_gyro_accel();
err_input_allocate1:
	printk("##  sensor: input_device_failed\n");
	input_unregister_device(star_motion_dev->input_dev);
exit_misc_device_register_failed:
err_sysfs_create:
	printk("##  sensor: heaven motion misc_device_register_failed\n");
err_motion_accel_wq:
	printk("##  sensor: accel timer_failed\n");
	destroy_workqueue(star_motion_dev->accel_wq);
err_motion_tilt_wq:
	printk("##  sensor: tilt timer_failed\n");
	destroy_workqueue(star_motion_dev->tilt_wq);
err_motion_gyro_wq:
	printk("##  sensor: gyro timer_failed\n");
	destroy_workqueue(star_motion_dev->gyro_wq);
err_motion_compass_wq:
	printk("##  sensor: compass timer_failed\n");
//	destroy_workqueue(star_motion_dev->compass_wq);
	return err;

}
static int star_motion_remove(struct platform_device *pdev)
{
	//struct device *dev = &pdev->dev;
	input_unregister_device(star_motion_dev->input_dev);
#if 0  /*Star Feature*/
	i2c_del_driver(&gyro_i2c_driver);
	i2c_del_driver(&accel_i2c_driver);
#endif
	if (star_motion_dev->tilt_wq)
		destroy_workqueue(star_motion_dev->tilt_wq);

	return 0;
}

#if 0 /*Dont need the function about suspend and resume */
#define star_motion_suspend  NULL
#define star_motion_resume   NULL
#else
void mpu3050_sleep_mode(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PWR_MGM ,&value , 1 );
#if DEBUG
	lprintk("--->> %s:%d PWR Reg : 0x%x\n",__func__,__LINE__,value);
#endif
	/*
	   Bit 6   SLEEP
	   Bit 5   STBY_XG
	   Bit 4   STBY_YG
	   Bit 3   STBY_ZG
	   */

	if (!(value & 0x40)) {
		value |= 0x40;

		buf[0] = MPU3050_GYRO_I2C_PWR_MGM;
		buf[1] = value;

		//mpu3050_write_reg(mpu3050_i2c_client,buf);
		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, buf[0], &buf[1], 1);
#if DEBUG
		lprintk("%s:%d PWR Reg : 0x%x\n", __func__, __LINE__, value);
#endif
	}
}

void mpu3050_sleep_wake_up(void)
{
	unsigned char value = 0;
	unsigned char buf[5] = {0,};

	//mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PWR_MGM,&value);
#if DEBUG
	lprintk("--->> %s:%d PWR Reg : 0x%x, jiffies : %u\n", __func__, __LINE__, value, jiffies);
#endif
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PWR_MGM, &value, 1);

	if (value & 0x40) {
		value &= ~0x40;

		buf[0] = MPU3050_GYRO_I2C_PWR_MGM;
		buf[1] = value;

		//mpu3050_write_reg(mpu3050_i2c_client,buf);
		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, buf[0], &buf[1], 1);
	}
}

static NvS32 star_motion_resume(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 0);
	return 0;
}

static NvS32 star_motion_suspend(struct platform_device *pdev)
{
	atomic_set(&suspend_flag, 1);
	return 0;
}

#endif
static struct platform_driver star_motion_driver = {
	.probe    	= star_motion_probe,
	.remove   	= star_motion_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  	= star_motion_suspend,
	.resume   	= star_motion_resume,
#endif
	.driver =  {
		.name = "tegra_gyro_accel",
		.owner = THIS_MODULE,
	},
};
static int __init star_motion_init(void)
{
	int err;

#if DEBUG
	lprintk("[MPU3050] ## [%s:%d]\n", __FUNCTION__, __LINE__);
#endif
	rwlock_init(&getbuflock);
	memset(&accelrwbuf[0], 0, sizeof(unsigned char) * 200);
	memset(&rwbuf[0], 0, sizeof(unsigned char) * 200);

	err = platform_driver_register(&star_motion_driver);

	return 0;
}
static void __exit star_motion_exit(void)
{
#if DEBUG
	lprintk(KERN_INFO "[MPU3050] lge star_motion_exit was unloaded!\nHave a nice day!\n");
#endif
	platform_driver_unregister(&star_motion_driver);
	return;
}

module_init(star_motion_init);
module_exit(star_motion_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Gyro-accel Driver for Star");
MODULE_LICENSE("GPL");

//jongik2.kim 20100910 i2c_fix [start]
NvBool star_get_i2c_busy()
{
	return i2c_busy_flag;
}
EXPORT_SYMBOL(star_get_i2c_busy);

void star_set_i2c_busy()
{
	i2c_busy_flag = 1;
	return 0;
}
EXPORT_SYMBOL(star_set_i2c_busy);

void star_unset_i2c_busy()
{
	i2c_busy_flag = 0;
	return 0;
}
EXPORT_SYMBOL(star_unset_i2c_busy);
//jongik2.kim 20100910 i2c_fix [end]

