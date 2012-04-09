/*
 *  gyro_accel.c
 *  star motion sensor driver  (Accelerometer, Gyroscope Sensor)  
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

#include "gyro_accel.h"
#include "mpu3050.h"


#define  MISC_DYNAMIC_MINOR		 255

#define  MAX_MOTION_DATA_LENGTH	 10

#define  MIN_MOTION_POLLING_TIME    10

#if defined(CONFIG_MACH_LGE_STAR_REV_C)
#define  STAR_GYRO_INT_GPIO         41
#endif

#define MAX_LGE_ACCEL_SAMPLERATE    7  /* for JSR256 API */

struct tegra_gyro_accel_device_data
{
//	struct task_struct	*task;
	struct input_dev	*input_dev;
	
};
struct tegra_acc_device_data *gyro_accel_dev=NULL;

struct i2c_client    *kxtf9_i2c_client;

static struct regulator 	 		*star_motion_reg;
static struct regulator				*star_gyro_vio_reg;

static atomic_t 	   accel_flag;
static atomic_t 	   tilt_flag;    
static atomic_t 	   tap_flag;
static atomic_t 	   shake_flag;
static atomic_t 	   snap_flag;
static atomic_t 	   flip_flag;
static atomic_t 	   gyro_flag;
static atomic_t 	   accel_delay;
static atomic_t 	   tilt_delay;
static atomic_t	   	   jsr256_flag;
static atomic_t        poweronoff_flag;

static atomic_t	       tilt_roll, 	tilt_pitch,	tilt_yaw;
static atomic_t	       accel_x, 	accel_y, 	accel_z;
static atomic_t	       gyro_x, 		gyro_y,		gyro_z ;



struct star_motion_device {
	NvOdmGyroAccelDeviceHandle	hOdmGyroAccel;
	struct input_dev  		       *input_dev;         
	struct input_dev  		       *input_dev1;	 /* motion daemon process */	       
	NvU32			freq;
	NvBool			bThreadAlive;
	NvBool			show_log;
	NvOdmGyroAccelIntType	IntType;
	NvOdmGyroAccelAxisType	IntMotionAxis;
	NvOdmGyroAccelAxisType	IntTapAxis;
	struct timeval		tv;
	struct hrtimer       		 timer[3];            /* [0] acceleroemter raw data, [1] tilt , [2] Shake   */

	struct work_struct     		 tilt_work;

	
	struct workqueue_struct	 *timer_wq;
	
	int  irq;
	int  use_irq;
};

static struct star_motion_device   *star_motion_dev=NULL;

#define write_lock(lock)		_write_lock(lock)
#define read_lock(lock)			_read_lock(lock)
rwlock_t getbuflock;	
static unsigned char   	accelrwbuf[200]={0,};    /* MPU3050 i2c MAX data length */
static unsigned char   	rwbuf[200]={0,};     	 /* MPU3050 i2c MAX data length */



static int  flip_test_count = 0;

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
	//printk(" ##  open_def_odm_gyro_accel  ##\n");
	if (!err) {
		err = -ENODEV;
		printk("open_def_odm_gyro_accel: NvOdmGyroAccelOpen failed\n");
		
		return err;
	}
	
#if 0	
	
	err = NvOdmAccelSetIntEnable(star_motion_dev->hOdmGyroAccel,
		NvOdmAccelInt_MotionThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);
	
	
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}	
    
	
	NvOdmAccelSetSampleRate(star_motion_dev->hOdmGyroAccel, 800);
	if (!err) {
		printk("TEST1=8  %s \n",__FUNCTION__);
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}

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

/*---------------------------------------------------------------------------
	kxtf9_reg_i2c_client
   ---------------------------------------------------------------------------*/		
void  kxtf9_reg_i2c_client(struct i2c_client *client)
{
	printk("kxtf9_reg_i2c_client..........................\n");
	kxtf9_i2c_client =  client;
}
/*---------------------------------------------------------------------------
	kxtf9_read_reg_in_burst
   ---------------------------------------------------------------------------*/		
int kxtf9_read_reg_in_burst(struct i2c_client *client, unsigned char reg,unsigned char *buf,int length)
{
	int err;
	unsigned char reg_val = reg;
	
	struct i2c_msg msg[2] = { 
		{ client->addr, 0, 1,&reg_val },
		{ client->addr, I2C_M_RD, length, buf}
	};

	if ((err = i2c_transfer(client->adapter, msg, 2)) < 0) 
	{
		dev_err(&client->dev, "i2c read error\n");
		return -EIO;
	}
	
	return 0;
}

/*---------------------------------------------------------------------------
	kxtf9_write_reg_in_burst
   ---------------------------------------------------------------------------*/		
int kxtf9_write_reg_in_burst(struct i2c_client *client, unsigned char *value,int length)
{
	unsigned char buf[length];   
       int err;		
	struct i2c_msg msg ={
		.addr = client->addr,
		.flags = 0,
		.len   = sizeof(buf),
		.buf   = buf};

	memcpy(buf,value,length);
	
	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) 
	{
		dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}
	
	return 0;
	
}
/*---------------------------------------------------------------------------
	kxtf9_write_reg
   ---------------------------------------------------------------------------*/		
int kxtf9_write_reg(struct i2c_client *client, unsigned char *buffer)
{	
	unsigned char buf[2];   
       int err;
	struct i2c_msg msg ={
		.addr = client->addr,
		.flags = 0,
		.len   = 2,
		.buf   = buf};

	buf[0] = buffer[0];
	buf[1] = buffer[1];
	
	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) 
	{
	       dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}
	
	return 0;
	
}

/*---------------------------------------------------------------------------
	kxtf9_read_reg
   ---------------------------------------------------------------------------*/		
int kxtf9_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
	int err;
	unsigned char buf = reg;

	struct i2c_msg msg[2] = { 
		{ client->addr, 0, 1, &buf },
		{ client->addr, I2C_M_RD, 1, value}
	};

	if ((err = i2c_transfer(client->adapter, msg, 2)) < 0) 
	{
		dev_err(&client->dev, "i2c read error\n");
		return -EIO;
	}
	
	return 0;
}



/*---------------------------------------------------------------------------
	kxtf9_i2c_read
   ---------------------------------------------------------------------------*/		
int kxtf9_i2c_read(unsigned char reg,unsigned char *buf,int length)
{
	int status = 0;

	//status = kxtf9_read_reg_in_burst(kxtf9_i2c_client,reg,buf,length);
	//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//printk("reg: %d(%x) / buf: %d(%x)/ length : %d(%x)\n",reg, reg, buf, buf, length, length);
	//NvAccelerometerI2CGetRegsPassThrough(reg, buf, length); // XOUT_L
	//NvAccelerometerI2CGetRegs(accel_dev->hOdmAcr, 0x06, &xyz[0], 6); // XOUT_L

	return status;
}

/*---------------------------------------------------------------------------
	kxtf9_i2c_write
   ---------------------------------------------------------------------------*/		
int  kxtf9_i2c_write(unsigned char *buffer,int length)
{
	int status = 0;

	//status = kxtf9_write_reg_in_burst(kxtf9_i2c_client,buffer,length);
	//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);
	//printk("buffer[0]: %d(%x) / buffer[\1]: %d(%x)/ length : %d(%x)\n",buffer[0],buffer[0],buffer[1],buffer[1], length, length);
	//NvAccelerometerI2CSetRegsPassThrough(buffer[0] ,&buffer[1] , length-1 );
	//NvAccelerometerI2CSetRegs(accel_dev->hOdmAcr, 0x1B, &val, 1);


	return status;
}

/*---------------------------------------------------------------------------
	 motion_sensor_power_on/off
   ---------------------------------------------------------------------------*/		  
void motion_sensor_power_on(void)
{
#if 0
	regulator_enable(star_motion_reg);
	msleep(1);  
#if defined(CONFIG_MACH_LGE_STAR_REV_C)	
	regulator_enable(star_gyro_vio_reg); 
	msleep(1);
#endif
#endif
	atomic_set(&poweronoff_flag, 1);

}
void motion_sensor_power_off(void)
{
#if 0
#if defined(CONFIG_MACH_LGE_STAR_REV_C)	
	regulator_disable(star_gyro_vio_reg); 
#endif
	regulator_disable(star_motion_reg);
#endif       
	atomic_set(&poweronoff_flag, 0);
}

/*---------------------------------------------------------------------------
	 motion_gyro_sensor_sleep_mode/awake
   ---------------------------------------------------------------------------*/		  
void motion_gyro_sensor_sleep_mode(void)
{
       /* mpu3050 - standby mode*/
//	mpu3050_sleep_mode();    //for temp
	msleep(10);    
	   
}

void motion_gyro_sensor_sleep_wake_up(void)
{
	/* mpu3050 - power on mode*/
//	mpu3050_sleep_wake_up(); //for temp 
	msleep(10); 
}

/*---------------------------------------------------------------------------
	 motion_send_event function
   ---------------------------------------------------------------------------*/	
void motion_send_tilt_detection(int roll,int pitch,int yaw)
{
	printk("[Gyro_accel_compass][%s:%d]\n",__FUNCTION__, __LINE__);

	if (atomic_read(&tilt_flag))
	{
		input_report_rel(star_motion_dev->input_dev,REL_RX,roll);
		input_report_rel(star_motion_dev->input_dev,REL_RY,pitch);
		input_report_rel(star_motion_dev->input_dev,REL_RZ,yaw); // sglee76@lge.com 2010-04-28, tilt yaw   
				
		input_sync(star_motion_dev->input_dev);
	}
}
void motion_send_tap_detection(int type,int direction)
{
	printk("[Gyro_accel_compass][%s:%d]\n",__FUNCTION__, __LINE__);

	if(atomic_read(&tap_flag))
	{
		//input_report_rel(star_motion_dev->input_dev,REL_X,type); 
		//input_report_rel(star_motion_dev->input_dev,REL_Y,direction); 
		
		input_sync(star_motion_dev->input_dev);
	}
}
void motion_send_shake_detection(int value)
{
	printk("[Gyro_accel_compass][%s:%d]\n",__FUNCTION__, __LINE__);
	if(atomic_read(&shake_flag))
	{
		input_report_rel(star_motion_dev->input_dev,REL_HWHEEL,value); 
		input_sync(star_motion_dev->input_dev);
	}
}
void motion_send_flip_detection(int value)
{
	printk("[Gyro_accel_compass][%s:%d]\n",__FUNCTION__, __LINE__);

	if(atomic_read(&flip_flag))
	{
		input_report_rel(star_motion_dev->input_dev,REL_WHEEL,value); 
		input_sync(star_motion_dev->input_dev);
	}
}
void motion_send_snap_detection(int direction)
{
	//printk("[Gyro_accel][%s:%d]\n",__FUNCTION__, __LINE__);

	if(atomic_read(&snap_flag))
	{
		input_report_rel(star_motion_dev->input_dev,REL_DIAL,direction); 
		
		input_sync(star_motion_dev->input_dev);
	}
}

/*---------------------------------------------------------------------------
	 work function
   ---------------------------------------------------------------------------*/		  
static void motion_tilt_work_func(struct work_struct *work)                         
{
	int current_roll = 0, current_pitch = 0, current_yaw;

	current_roll  = atomic_read(&tilt_roll);
	current_pitch = atomic_read(&tilt_pitch);
	current_yaw   = atomic_read(&tilt_yaw); // sglee76@lge.com 2010-04-28, tilt yaw

	current_roll  = current_roll*(-1);
	current_pitch = current_pitch;//*(-1);
	current_yaw   = current_yaw*(-1);

    //printk("[motion_tilt_work_func] current_roll=[%d], current_pitch=[%d], current_yaw=[%d] \n",current_roll,current_pitch,current_yaw);
		
	motion_send_tilt_detection(current_roll,current_pitch,current_yaw);
	
}



/*---------------------------------------------------------------------------
	 motion polling timer
   ---------------------------------------------------------------------------*/		  
static enum hrtimer_restart motion_tilt_timer_func(struct hrtimer *timer)
{	
	unsigned long  polling_time;   

	if(atomic_read(&tilt_flag))
	{	
		queue_work(star_motion_dev->timer_wq, &star_motion_dev->tilt_work);

		polling_time = atomic_read(&tilt_delay);
		//printk("[motion_tilt_timer_func] tilt_delay = [%d] \n", polling_time);
		hrtimer_start(&star_motion_dev->timer[1], ktime_set(0,polling_time*1000000), HRTIMER_MODE_REL); 
	}

    	return HRTIMER_NORESTART;
}

/*---------------------------------------------------------------------------
	 sensor enable/disable (Sensor HAL)
   ---------------------------------------------------------------------------*/		  
static ssize_t motion_accel_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&accel_flag);
	//printk("[motion_accel_onoff_store] accel_flag [%d]\n",val);
	return val;
}
	
static ssize_t motion_accel_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_accel_onoff_store] accel_flag [%d]\n",val);

	if(val){
		atomic_set(&accel_flag, 1);
	}else{
		atomic_set(&accel_flag, 0);
	}
	
	return count;
}
  
static ssize_t motion_gyro_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&gyro_flag);
	//printk("[motion_gyro_onoff_show] gyro_flag [%d]\n",val);
	return val;
}
	
static ssize_t motion_gyro_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_gyro_onoff_store] gyro_flag [%d]\n",val);

	if(val){
		atomic_set(&gyro_flag, 1);
	}else{
		atomic_set(&gyro_flag, 0);
	}
	
	return count;
}


static ssize_t motion_tilt_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tilt_flag);
	return val;
}

static ssize_t motion_tilt_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;	
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_tilt_onoff_store]  flag [%d]\n",val);
		   
	if(val)	{
		atomic_set(&tilt_flag, 1);
		hrtimer_start(&star_motion_dev->timer[1], ktime_set(0,200*1000000), HRTIMER_MODE_REL); 
		
	}	else	{
		atomic_set(&tilt_flag, 0);
		hrtimer_cancel(&star_motion_dev->timer[1]);	   
	}

	return count;
}

static ssize_t motion_tap_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tilt_flag);
	return val;
}

static ssize_t motion_tap_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_tap_onoff_store] tap.... flag [%d]\n",val);

	if(val){
		atomic_set(&tap_flag, 1);
	}else{
		atomic_set(&tap_flag, 0);
	}
	
	return count;
}

static ssize_t motion_flip_onoff_show(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&flip_flag);
	return val;
}

static ssize_t motion_flip_onoff_store(struct device *dev,  struct device_attribute *attr,  char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_flip_onoff_store]  flag [%d]\n",val);

	if(val){
		atomic_set(&flip_flag, 1);
	}else{
		atomic_set(&flip_flag, 0);	
	}

	return count;
}

static ssize_t motion_shake_onoff_show(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&shake_flag);
	return val;
}

static ssize_t motion_shake_onoff_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_shake_onoff_store]  flag [%d]\n",val);

	if(val){
		atomic_set(&shake_flag, 1);
	}else{
		atomic_set(&shake_flag, 0);
	}

	return count;
}

static ssize_t motion_snap_onoff_show(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&snap_flag);
	return val;
}

static ssize_t motion_snap_onoff_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = simple_strtoul(buf, NULL, 10);
	//printk("[motion_set_snap_onoff_store]  flag [%d]\n",val);

	if(val){
		atomic_set(&snap_flag, 1);	
	}else{
		atomic_set(&snap_flag, 0);
	}

	return count;
}

static ssize_t motion_tilt_delay_show(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32    val;
	val = atomic_read(&tilt_flag);
	return val;
}

static ssize_t motion_tilt_delay_store(struct device *dev,  struct device_attribute *attr,const char *buf, size_t count)
{
	u32     val;
	unsigned long   current_delay = 100;
	val = simple_strtoul(buf, NULL, 10);
	current_delay = (short)val;

	//printk("[motion_set_tilt_delay_store]  val [%d] current_delay[%ld]\n",val,current_delay);

	if(atomic_read(&tilt_flag))
	{
		//hrtimer_cancel(&star_motion_dev->timer[1]);  
		if(current_delay < MIN_MOTION_POLLING_TIME)		{
			
			current_delay = MIN_MOTION_POLLING_TIME;
		}

		//printk("[motion_set_tilt_delay_store]  val [%d] current_delay[%ld]\n",val,current_delay);
		atomic_set(&tilt_delay, current_delay);

	}

	return count;

}
static DEVICE_ATTR(accel_onoff,0666,motion_accel_onoff_show,motion_accel_onoff_store);
static DEVICE_ATTR(gyro_onoff,0666,motion_gyro_onoff_show,motion_gyro_onoff_store);

static DEVICE_ATTR(tilt_onoff,0666,motion_tilt_onoff_show,motion_tilt_onoff_store);
static DEVICE_ATTR(tap_onoff,0666,motion_tap_onoff_show,motion_tap_onoff_store);
static DEVICE_ATTR(shake_onoff,0666,motion_shake_onoff_show,motion_shake_onoff_store);
static DEVICE_ATTR(snap_onoff,0666,motion_snap_onoff_show,motion_snap_onoff_store);
static DEVICE_ATTR(flip_onoff,0666,motion_flip_onoff_show,motion_flip_onoff_store);
static DEVICE_ATTR(tilt_delay,0666,motion_tilt_delay_show,motion_tilt_delay_store);


static struct attribute *star_motion_attributes[] = {
	&dev_attr_accel_onoff.attr,
	&dev_attr_gyro_onoff.attr,
	&dev_attr_tilt_onoff.attr,
	&dev_attr_tap_onoff.attr,
	&dev_attr_shake_onoff.attr,
	&dev_attr_snap_onoff.attr,
	&dev_attr_flip_onoff.attr,	
	&dev_attr_tilt_delay.attr,
	NULL
};

static const struct attribute_group star_motion_group = {
	.attrs = star_motion_attributes,
};

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
	//printk("motion close\n");
	return 0;
}
static int count = 0 ; 
static int star_motion_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char   	data[MAX_MOTION_DATA_LENGTH]={0,};	
	int                        buf[5] = {0,};
	int     		     	flag = 0;
	int                        delay = 0;
	
	unsigned char   	tempbuf[200]={0,};     /* MPU3050 i2c MAX data length */
       int 				ret = 0;
   unsigned char value;

	switch (cmd) 
	{
	case MOTION_IOCTL_ENABLE_DISABLE:
		 /*
			0 : disable sensor
			1:  orientation (tilt)
			2:  accelerometer
			3: tap
			4: shake
		*/
		 //printk(".............star_motion_ioctl................\n"); 
		flag = STAR_SENSOR_NONE;

		if(atomic_read(&accel_flag)){
		 //printk(".............if(atomic_read(&snap_flag)){................\n"); 			
			flag |= STAR_ACCELEROMETER;
		}

		if(atomic_read(&tilt_flag)){
		 //printk(".............if(atomic_read(&tilt_flag)){................\n"); 
			flag |= STAR_TILT;
		}
		
		if(atomic_read(&shake_flag)){
		 //printk(".............if(atomic_read(&shake_flag)){................\n"); 
			flag |= STAR_SHAKE;
		}

		if(atomic_read(&tap_flag)){
		 //printk(".............if(atomic_read(&tap_flag)){................\n"); 			
			flag |= STAR_TAP;
		}

		if(atomic_read(&flip_flag)){
		 //printk(".............if(atomic_read(&flip_flag)){................\n"); 			
			flag |= STAR_FLIP;
		}

		if(atomic_read(&snap_flag)){
		 //printk(".............if(atomic_read(&snap_flag)){................\n"); 			
			flag |= STAR_SNAP;
		}
		
		if(atomic_read(&gyro_flag)){
		 //printk(".............if(atomic_read(&snap_flag)){................\n"); 			
			flag |= STAR_GYRO;
		}
		
		 if (copy_to_user(argp,&flag, sizeof(flag)))
		 {
		 		 //printk(".............MOTION_IOCTL_SNAP................\n"); 
			return -EFAULT;
		 } 
		 break;
	case MOTION_IOCTL_ACCEL_RAW:
		  if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 /* 	buf[0], [1], [2] = accel_x,  accel_y,  accel_z;	 */
		 
		   atomic_set(&accel_x,buf[0]);  
		   atomic_set(&accel_y,buf[1]);
		   atomic_set(&accel_z,buf[2]);
		   
		   //motion_send_tilt_detection(buf[0],buf[1],buf[2]);
   		 //printk(".............MOTION_IOCTL_TILT................\n"); 
		   break;  		 		
	case MOTION_IOCTL_GYRO_RAW:
		  if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 /* 	buf[0], [1], [2] = gyro_x,  gyro_y,  gyro_z; */
		 
		   atomic_set(&gyro_x,buf[0]);  
		   atomic_set(&gyro_y,buf[1]);
		   atomic_set(&gyro_z,buf[2]);
		   
		   //motion_send_tilt_detection(buf[0],buf[1],buf[2]);
   		 //printk(".............MOTION_IOCTL_TILT................\n"); 
		   break;  		 		   			   
	case MOTION_IOCTL_SENSOR_DELAY:
		  delay = atomic_read(&tilt_delay);

		  //printk("MOTION_IOCTL_SENSOR_DELAY[%d]",delay);
		  
		 if (copy_to_user(argp, &delay, sizeof(delay)))
		 {
		 	 return -EFAULT;
		 }
		 break;
		 
	case MOTION_IOCTL_TILT:
		  if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 /* 	buf[0], [1], [2] = roll,  pitch,  yaw;	 */
		 
		   atomic_set(&tilt_roll,buf[0]);  
		   atomic_set(&tilt_pitch,buf[1]);
		   atomic_set(&tilt_yaw,buf[2]);
		   //motion_send_tilt_detection(buf[0],buf[1],buf[2]);
   		 //printk(".............MOTION_IOCTL_TILT................\n"); 
		   break;  		 		   
       case MOTION_IOCTL_SHAKE:   
	   	  if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 /* 			buf[0] = event;   		  */
		 //printk(".............MOTION_IOCTL_SHAKE................\n"); 		  
		   motion_send_shake_detection(buf[0]);
		 break;
	case MOTION_IOCTL_FLIP:
		 if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 //printk(".............MOTION_IOCTL_FLIP................\n"); 		

		   motion_send_flip_detection(buf[0]);
		 break;
	case MOTION_IOCTL_TAP:         
		 if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		   }		  
		 /* 
			buf[0] = type;   
			buf[1] = direction;			 
		  */
  		 //printk(".............MOTION_IOCTL_TAP................\n"); 
		   motion_send_tap_detection(buf[0],buf[1]);
		 break;
	case MOTION_IOCTL_SNAP:    
		 if (copy_from_user(&buf, argp, sizeof(buf)))
		  {
			return -EFAULT;
		  }
		 /* 
			buf[0] = direction;
		  */
		 //printk(".............MOTION_IOCTL_SNAP................\n"); 
		 motion_send_snap_detection(buf[0]);
		 break;	 
	case MOTION_IOCTL_MPU3050_SLEEP_MODE:
		 //printk(".............MOTION_IOCTL_MPU3050_SLEEP_MODE................\n");
		 //motion_gyro_sensor_sleep_mode();
		 motion_sensor_power_off();
		 
		 //twl4030_i2c_write_u8(0x13, 0x00,0x1b );
		 //msleep(100);	
		 break;
	case MOTION_IOCTL_MPU3050_SLEEP_WAKE_UP:
		 //printk(".............MOTION_IOCTL_MPU3050_SLEEP_WAKE_UP................\n");
		 //motion_gyro_sensor_sleep_wake_up();
		 
		 motion_sensor_power_on();
		 break;	 
	case MOTION_IOCTL_MPU3050_I2C_READ:
		//printk(".............MOTION_IOCTL_MPU3050_I2C_READ................\n");
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
		{
			//printk("FAIL!!!!!!copy_from_user.................MOTION_IOCTL_MPU3050_I2C_READ");
			return -EFAULT;
		}

		write_lock(&getbuflock);
		memcpy(&accelrwbuf[0], rwbuf, sizeof(rwbuf));
		write_unlock(&getbuflock); 	

		if (rwbuf[1] < 1)
		{
			printk("EINVAL ERROR......I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
			
		        return -EINVAL;
		}

		if(rwbuf[0] == GYRO_I2C_SLAVE_ADDR)
		{
			//printk("############ (_0_)############ rwbuf[2]: %d(%x) / rwbuf[3]: %d(%x)/ rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[3],rwbuf[3], rwbuf[1], rwbuf[1]);
			NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1]);
			if (ret < 0){
				printk("MOTION_IOCTL_I2C_READ : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
				return -EINVAL;
			}

			if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			{
		       printk("EINVAL ERROR.### GYRO ### I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
		 	 return -EFAULT;
			}
		}
		else if(accelrwbuf[0] == 0x0F)
		{
			//printk("#### (_0_) #### accelrwbuf[2]: %d(%x) / accelrwbuf[3]: %d(%x)/ accelrwbuf[1] : %d(%x)\n",accelrwbuf[2],accelrwbuf[2], accelrwbuf[3],accelrwbuf[3], accelrwbuf[1], accelrwbuf[1]);
			//printk("######################## accel get(read) ##########################\n");
			if ((!accelrwbuf)){
				printk("### EEROR #### accelrwbuf is NULL pointer \n");
				return -1;
			} else{
				NvAccelerometerI2CGetRegsPassThrough (accelrwbuf[2] ,&accelrwbuf[3] , accelrwbuf[1]);
			}
			
			if (ret < 0){
				printk("MOTION_IOCTL_I2C_READ : ACCEL_I2C_SLAVE_ADDR Address ERROR[%d]\n",accelrwbuf[0]);
				return -EINVAL;
			}

			if (copy_to_user(argp, &accelrwbuf, sizeof(accelrwbuf)))
			{
		       printk("EINVAL ERROR  ### ACCEL ## I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
		 	 return -EFAULT;
			}
		}
		#if 0 /**/
		else if(accelrwbuf[0] == 0x0e)
		{
			//printk("#### (_0_) #### accelrwbuf[2]: %d(%x) / accelrwbuf[3]: %d(%x)/ accelrwbuf[1] : %d(%x)\n",accelrwbuf[2],accelrwbuf[2], accelrwbuf[3],accelrwbuf[3], accelrwbuf[1], accelrwbuf[1]);
			//printk("######################## accel get(read) ##########################\n");
			if ((!accelrwbuf)){
				printk("### EEROR #### accelrwbuf is NULL pointer \n");
				return -1;
			} else{
				NvGyroAccelI2CGetRegs (accelrwbuf[2] ,&accelrwbuf[3] , accelrwbuf[1]);
			}
			
			if (ret < 0){
				printk("MOTION_IOCTL_I2C_READ : ACCEL_I2C_SLAVE_ADDR Address ERROR[%d]\n",accelrwbuf[0]);
				return -EINVAL;
			}

			if (copy_to_user(argp, &accelrwbuf, sizeof(accelrwbuf)))
			{
		       printk("EINVAL ERROR  ### ACCEL ## I2C SLAVE MOTION_IOCTL_I2C_READ : rwbuf[1] < 1...\n");
		 	 return -EFAULT;
			}
		}
		#endif
		else
		{
			printk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
			return -EINVAL;
		}
		
		
			
		break;
	case MOTION_IOCTL_MPU3050_I2C_WRITE:
		//printk(".............MOTION_IOCTL_MPU3050_I2C_WRITE................\n");
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
		{
		       printk("EINVAL ERROR.....copy_from_user.I2C SLAVE MOTION_IOCTL_I2C_WRITE \n");
			return -EFAULT;
		}
		/*	
   		rwbuf[0] = slave_addr;  // slave addr - GYRO(0x68-MPU) 
     		rwbuf[1] = 2;                   // number of bytes to write +1
     		rwbuf[2] = reg;               // register address
     		rwbuf[3] = value;          // register value		
		*/
		if (rwbuf[1] < 2)
		{
		       printk("MOTION_IOCTL_WRITE ..length ERROR!!![%d].....\n",rwbuf[1]);
			return -EINVAL;
		}

		if(rwbuf[0] == GYRO_I2C_SLAVE_ADDR)
		{
			//ret = mpu3050_i2c_write(&rwbuf[2],rwbuf[1]);
			NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, rwbuf[2] ,&rwbuf[3] , rwbuf[1]-1);
			if (ret < 0){
				 printk("MOTION_IOCTL_WRITE  : GYRO_I2C_SLAVE_ADDR Address ERROR[%d]\n",rwbuf[0]);
				return -EINVAL;
			}
		}
		else if(rwbuf[0] == 0x0F)
		{
			//ret = kxtf9_i2c_write(&rwbuf[2],rwbuf[1]);
			//printk("(_6_)rwbuf[2]: %d(%x) /  rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[1], rwbuf[1]);
//			printk("######################## accel set(write) ##########################\n");
			NvAccelerometerI2CSetRegsPassThrough(rwbuf[2] ,&rwbuf[3] , rwbuf[1]-1);
			//printk("(_7_) rwbuf[3]: %d(%x) \n", rwbuf[3],rwbuf[3]);
			if (ret < 0){
				  printk("[KXTF9] MOTION_IOCTL_WRITE  : ACCEL_I2C_SLAVE_ADDR ERROR[%d]\n",rwbuf[0]);
				return -EINVAL;
			}
		}
		#if 0
		else if(rwbuf[0] == 0x0e)
		{
			//ret = kxtf9_i2c_write(&rwbuf[2],rwbuf[1]);
			//printk("(_6_)rwbuf[2]: %d(%x) /  rwbuf[1] : %d(%x)\n",rwbuf[2],rwbuf[2], rwbuf[1], rwbuf[1]);
//			printk("######################## accel set(write) ##########################\n");
			NvGyroAccelI2CSetRegs(rwbuf[2] ,&rwbuf[3] , rwbuf[1]-1);
			//printk("(_7_) rwbuf[3]: %d(%x) \n", rwbuf[3],rwbuf[3]);
			if (ret < 0){
				  printk("[KXTF9] MOTION_IOCTL_WRITE  : ACCEL_I2C_SLAVE_ADDR ERROR[%d]\n",rwbuf[0]);
				return -EINVAL;
			}
		}
		#endif
		else
		{
			printk("......I2C SLAVE ADDRESS ERROR!!!...[0x%x]...\n",buf[0]);
			return -EINVAL;
		}
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
	.minor = MISC_DYNAMIC_MINOR,
	.name = STAR_MOTION_IOCTL_NAME,
	.fops   = &star_motion_fops,
};

#if 0  /*Star Feature*/

/*---------------------------------------------------------------------------
	 accel.  driver
---------------------------------------------------------------------------*/		  
static int __init accel_i2c_probe(struct i2c_client *client)
{	
       printk("----------accel_i2c_probe\n");	
	kxtf9_reg_i2c_client(client);

	return 0;
}

static int  accel_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#define accel_i2c_suspend	NULL
#define accel_i2c_resume		NULL

static const struct i2c_device_id accel_i2c_id[] = {
	{STAR_I2C_ACCEL_NAME, 0 },	
	{ /* end of list */ },
};

static struct i2c_driver accel_i2c_driver = {
       .probe     =  accel_i2c_probe,
	.remove  =  accel_i2c_remove,
	.id_table = accel_i2c_id,
#ifdef CONFIG_PM
	.suspend = accel_i2c_suspend,
	.resume	 = accel_i2c_resume,
#endif
	.driver = {
       .name = STAR_I2C_ACCEL_NAME,
	},
};


/*---------------------------------------------------------------------------
	 gyro driver
---------------------------------------------------------------------------*/		  
static int __init gyro_i2c_probe(struct i2c_client *client)
{	
	printk("----------gyro_i2c_probe\n");	
	mpu3050_reg_i2c_client(client);

	return 0;
		
}
static int  gyro_i2c_remove(struct i2c_client *client)
{
	return 0;
}


#define gyro_i2c_suspend		NULL
#define gyro_i2c_resume		NULL

static const struct i2c_device_id gyro_i2c_id[] = {
	{STAR_I2C_GYRO_NAME, 0 },	
	{ /* end of list */ },
};

static struct i2c_driver gyro_i2c_driver = {
       .probe = gyro_i2c_probe,
	.remove =  gyro_i2c_remove,
	.id_table = gyro_i2c_id,
#ifdef CONFIG_PM
	.suspend = gyro_i2c_suspend,
	.resume	 = gyro_i2c_resume,
#endif
	.driver = {
       .name = STAR_I2C_GYRO_NAME,
	},
};
#endif

void mpu3050_i2c_through_pass(int benable)
{
	unsigned char value;
	unsigned char buf[3]={0,};
	int status = 0;
		
	value = 0;
	//printk("[MPU3050].......mpu3050_i2c_through_pass................\n"); 
    //status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_USER_CTRL,&value);    
    printk("[MPU3050] [%s:%d]\n",__FUNCTION__, __LINE__);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_USER_CTRL ,&value , 1 );
	if(status <0)
	{
	     printk("[MPU3050] MPU3050_GYRO_I2C_USER_CTRL. i2c ERROR: 0x%x................................\n",value);
      	     return;
	}

	printk("[MPU3050]................................\n");
      	     	
      	if(benable ==MPU3050_BYPASS_MODE_ON)
      	{
	      	printk("[MPU3050] bypass on.....................................\n");
      	     if(value & 0x20)
      	        value&= ~(0x20);	
      	}
      	else  // bypass off
      	{
      		printk("[MPU3050] bypass off.....................................\n");
      	      if(!(value & 0x20))
      	           value|= 0x20;      
      	}
      	
      	if(!(value & 0x08))
      	       value|=0x08;	
      	       
      	buf[0] = MPU3050_GYRO_I2C_USER_CTRL;
      	buf[1] = value;
      	//status = mpu3050_write_reg(mpu3050_i2c_client,buf);	
		NvGyroAccelI2CSetRegs(star_motion_dev->hOdmGyroAccel, buf[0] ,&buf[1] , 1 );
      	
}


void mpu3050_initialize(void)
{
	unsigned char buf[3]={0,};
	unsigned char value = 0;
	int status = 0;

#if 0
	//  Read WHO AM I  
	value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_WHO_AM_I,&value);
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I ,&value , 1 );
	printk("[MPU3050] MPU3050_GYRO_I2C_WHO_AM_I : %x\n",value);
	
	// Read Product ID
	value = 0;
	//status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PRODUCT_ID,&value); 
	NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID ,&value , 1 );
	printk("[MPU3050] MPU3050_GYRO_I2C_PRODUCT_ID : %x\n",value);
	  /*---------------------------------------------------------------------------------------*/	  
#endif
	  
}


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

	 
	 printk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
	 
	 gyroscope_accel = kzalloc(sizeof(*gyroscope_accel), GFP_KERNEL);
	 star_motion_dev = gyroscope_accel;
	 
     printk(KERN_INFO"%s: probe start\n", __func__);
    
   /*---------------------------------------------------------------------------
       register i2c driver
     ---------------------------------------------------------------------------*/	 	
     #if 0  /*Star Feature*/
      err = i2c_add_driver(&gyro_i2c_driver);
     if(err < 0){
	    printk("************* LGE: gyro_i2c_test_client fail\n");
	    goto err_i2c_add_driver;
     }

     err = i2c_add_driver(&accel_i2c_driver);
     if(err < 0){
	    printk("************* LGE: accel_i2c_test_client fail\n");
	    goto err_i2c_add_driver;
     }	 
  	#endif
    

     /*---------------------------------------------------------------------------
       register misc device
     ---------------------------------------------------------------------------*/	 
       err = misc_register(&star_motion_misc_device);
       if (err) {
		printk(KERN_ERR"star_motion_misc_device register failed\n");
		goto exit_misc_device_register_failed;
	}

     /*---------------------------------------------------------------------------
       register input device
      ---------------------------------------------------------------------------*/
      star_motion_dev->input_dev = input_allocate_device();
      if(star_motion_dev->input_dev == NULL)
      {
           printk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed\n");
           goto err_input_allocate1;		
      }  

      star_motion_dev->input_dev->name = STAR_MOTION_INPUT_NAME;
  
       set_bit(EV_SYN,star_motion_dev->input_dev->evbit);	
	   set_bit(EV_REL,star_motion_dev->input_dev->evbit);

	   set_bit(REL_X,star_motion_dev->input_dev->relbit);  // TAP - Type 
	   set_bit(REL_Y,star_motion_dev->input_dev->relbit);  // TAP - Direction   
	   set_bit(REL_RX,star_motion_dev->input_dev->relbit);  // TILT - Roll
	   set_bit(REL_RY,star_motion_dev->input_dev->relbit);  // TILT - PITCH   
	   set_bit(REL_RZ,star_motion_dev->input_dev->relbit);  // TILT - Yaw
	   set_bit(REL_HWHEEL,star_motion_dev->input_dev->relbit); // SHAKE
	   set_bit(REL_DIAL,star_motion_dev->input_dev->relbit);   // SNAP - Direction 
	   set_bit(REL_WHEEL,star_motion_dev->input_dev->relbit);  // FLIP
  
       err = input_register_device(star_motion_dev->input_dev);
       if(err){
             printk(KERN_ERR"star_motion_sesnor_probe: input_allocate_device (1) failed \n");
             goto err_input_allocate1;		
       }

	/*---------------------------------------------------------------------------
		init. sysfs
	---------------------------------------------------------------------------*/			 
       if ((err = sysfs_create_group(&dev->kobj, &star_motion_group)))
       {
	    printk("[motion_sensor] sysfs_create_group FAIL \n");
	    goto err_sysfs_create;
      }

	/*---------------------------------------------------------------------------
		INIT_WORK 
	---------------------------------------------------------------------------*/		  			
	#if 1
	INIT_WORK(&star_motion_dev->tilt_work, motion_tilt_work_func);

    /*---------------------------------------------------------------------------
		init. workqueue 
	---------------------------------------------------------------------------*/		  			
	star_motion_dev->timer_wq = create_singlethread_workqueue("motion_timer_wq");
	if (!star_motion_dev->timer_wq) {
		printk("[motion_sensor] couldn't create timer queue\n");
		goto err_motion_timer_wq;
	}

   /*---------------------------------------------------------------------------
		init. timer
    ---------------------------------------------------------------------------*/
     // TILT POLLING TIMER 	 
     hrtimer_init(&star_motion_dev->timer[1], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
     star_motion_dev->timer[1].function = motion_tilt_timer_func;
	#endif 
     /*---------------------------------------------------------------------------
       power 
     ---------------------------------------------------------------------------*/	
     #if 0
#if defined(CONFIG_MACH_LGE_STAR_REV_C)	
	 star_gyro_vio_reg = regulator_get(&pdev->dev, "vaux2");
	 if (star_gyro_vio_reg == NULL) {
		    printk(KERN_ERR": Failed to get motion power resources !! \n");
		    return -ENODEV;
	 }
#endif

	 star_motion_reg = regulator_get(&pdev->dev, "vmmc2");
	  if (star_motion_reg == NULL) {
		    printk(KERN_ERR": Failed to get motion power resources !! \n");
		    return -ENODEV;
	  } 
#endif
	  printk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);

	  err = open_def_odm_gyro_accel();
		   if (!err) {
			   printk("open_def_odm_gyro_accel\n");
			   goto allocate_dev_fail;
		  }
	  printk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);

	  
	mdelay(50);
	 //  Read WHO AM I	
	 value = 0;
	 //status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_WHO_AM_I,&value);
	 NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_WHO_AM_I ,&value , 1 );
	 printk("[MPU3050] MPU3050_GYRO_I2C_WHO_AM_I : %x\n",value);
	 
	 // Read Product ID
	 value = 0;
	 //status = mpu3050_read_reg(mpu3050_i2c_client,MPU3050_GYRO_I2C_PRODUCT_ID,&value); 
	 NvGyroAccelI2CGetRegs(star_motion_dev->hOdmGyroAccel, MPU3050_GYRO_I2C_PRODUCT_ID ,&value , 1 );
	 printk("[MPU3050] MPU3050_GYRO_I2C_PRODUCT_ID : %x\n",value);

	 #if 0
	 err = open_def_odm_accl();
		   if (!err) {
			   printk("open_def_odm_gyro_accel\n");
			   goto allocate_dev_fail;
		  }
	  printk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);	
	  #endif
	 // mpu3050_initialize();

	  //motion_sensor_power_on();
	  //twl4030_i2c_write_u8(0x13, 0x00,0x1b );
	  //msleep(100);	
	  
	 return 0;	
#if 0  /*Star Feature*/
err_i2c_add_driver:
    i2c_del_driver(&gyro_i2c_driver);
	i2c_del_driver(&accel_i2c_driver);
#endif
allocate_dev_fail:
       printk("##  sensor: allocated_device_failed\n");        
	close_odm_gyro_accel();
err_input_allocate1:   
       printk("##  sensor: input_device_failed\n");        
	input_unregister_device(star_motion_dev->input_dev);	
exit_misc_device_register_failed:
err_sysfs_create:
       printk("##  sensor: heaven motion misc_device_register_failed\n");        
err_motion_timer_wq:
       printk("##  sensor: timer_failed\n");        
	destroy_workqueue(star_motion_dev->timer_wq);
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
	if (star_motion_dev->timer_wq)
		destroy_workqueue(star_motion_dev->timer_wq);
	   
    return 0;
}

#define star_motion_suspend  NULL
#define star_motion_resume   NULL

static struct platform_driver star_motion_driver = {
	.probe    	= star_motion_probe,
	.remove   	= star_motion_remove,
//	.suspend  	= star_motion_suspend,
//	.resume   	= star_motion_resume,
	.driver =  {
		       .name = "tegra_gyro_accel",
		       .owner = THIS_MODULE,
	  },
};
static int __init star_motion_init(void)
{	
     int err;
	 printk("[MPU3050] ## [%s:%d]\n",__FUNCTION__, __LINE__);
	 rwlock_init(&getbuflock);
	 memset(&accelrwbuf[0], 0, sizeof(unsigned char)*200);  
	 memset(&rwbuf[0], 0, sizeof(unsigned char)*200);  


     err = platform_driver_register(&star_motion_driver);
	
     return 0;
}
static void __exit star_motion_exit(void)
{
      printk(KERN_INFO "[MPU3050] lge star_motion_exit was unloaded!\nHave a nice day!\n");
	  
      return;
}

module_init(star_motion_init);
module_exit(star_motion_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Gyro-accel Driver for Star");
MODULE_LICENSE("GPL");

