/* drivers/i2c/chips/ami304.c - AMI304 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
//#include <linux/tegra_devices.h>

#include <nvodm_services.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include "star_compass.h"
#include <linux/kobject.h>
#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
//#include <nvodm_compass.h>

#define DEBUG_AMI304 0
#define DEBUG_IOCTL 0
#define DEBUG_HAL_IOCTL 0
#define DEBUG_DAEMON_IOCTL 0
#define DEBUG_DATA 0

#define STAR_COMPASS_DEBUG 0 

#define AMI_ORIENTATION_SENSOR		0
#define AMI_MAGNETIC_FIELD_SENSOR	1
#define AMI_ACCELEROMETER_SENSOR		2

//static struct i2c_client *ami304_i2c_client = NULL;

/* Addresses to scan */
//static unsigned short normal_i2c[] = { AMI304_I2C_ADDRESS, I2C_CLIENT_END };
/* Insmod parameters */
//I2C_CLIENT_INSMOD_1(ami304);
//static int ami304_i2c_attach_adapter(struct i2c_adapter *adapter);
//static int ami304_i2c_detect(struct i2c_adapter *adapter, int address, int kind);
//static int ami304_i2c_detach_client(struct i2c_client *client);

static struct input_dev *compass_input_dev;

struct _ami302_data {
	rwlock_t lock;
	int mode;
	int rate;
	volatile int updated;
} ami304_data;

struct _ami304mid_data {
	rwlock_t datalock;
	rwlock_t ctrllock;	
	int controldata[10];	
	int yaw;
	int roll;
	int pitch;
	int nmx;
	int nmy;
	int nmz;
	int nax;
	int nay;
	int naz;
	int mag_status;
} ami304mid_data;

//struct ami304mid_data *g_compass;
/*
struct compass_data {
	NvU32 x;
	NvU32 y;
	NvU32 z;
};
*/
typedef struct star_compass_device_data
{
	//NvOdmCompassDeviceHandle	hOdmComp;
	NvOdmServicesI2cHandle h_gen2_i2c;
	NvOdmServicesGpioHandle h_compass_gpio;
	NvOdmGpioPinHandle  h_compass_gpio_pin;
	NvOdmServicesGpioIntrHandle h_compass_intr;
	NvOdmServicesPmuHandle  h_compass_pmu;
	NvU32 i2c_address;
	NvU32 vdd_id;
	NvU32 intr_pin;
	NvU32 intr_port;

	struct task_struct	*task;
	struct input_dev	*input_dev;
	NvU32			freq;
//	NvBool			bThreadAlive;
//	NvBool			show_log;
	//NvOdmCompassIntType	IntType;
	//NvOdmCompassAxisType	IntMotionAxis;
	//NvOdmCompassAxisType	IntTapAxis;
//	struct timeval		tv;
//	struct compass_data prev_data;
//	struct compass_data min_data;
//	struct compass_data max_data;
}star_compass_device;

static star_compass_device *g_compass;

/** Function to close the ODM device. This function will help in switching
 * between power modes 
 */

static bool star_compass_i2c_write_data( star_compass_device *compass, unsigned char reg, unsigned char* data, unsigned char len )
{
	NvOdmI2cStatus i2c_status;
    NvOdmI2cTransactionInfo info;
    unsigned char* transfer_data;

    transfer_data = (unsigned char*)NvOdmOsAlloc(len+1);
    transfer_data[0] = reg;
    NvOdmOsMemcpy( &transfer_data[1], data, (size_t)len);

    info.Address = compass->i2c_address;
    info.Buf = transfer_data;
    info.Flags = NVODM_I2C_IS_WRITE;
    info.NumBytes = len+1;

    do{
        i2c_status = NvOdmI2cTransaction( compass->h_gen2_i2c, &info, 1, 400, 1000 );
    }while(i2c_status == NvOdmI2cStatus_Timeout);

    if( i2c_status != NvOdmI2cStatus_Success )
    {
        printk("[star compass driver] %s : i2c transaction error(Number = %d)!\n",__func__,i2c_status);
        goto err;
    }
    NvOdmOsFree(transfer_data);
    return true;
err:
    NvOdmOsFree(transfer_data);
    return false;
}

static bool star_compass_i2c_read_data( star_compass_device *compass, unsigned char reg, unsigned char* data, unsigned char len )
{
	NvOdmI2cStatus i2c_status;
    NvOdmI2cTransactionInfo info[2];
    unsigned char* transfer_data;

    transfer_data = (unsigned char*)NvOdmOsAlloc(len);

    #if STAR_COMPASS_DEBUG
    //printk("address = %#x\n", g_compass->i2c_address);
    #endif

    info[0].Address = compass->i2c_address;
    info[0].Buf = &reg;
    info[0].Flags = NVODM_I2C_IS_WRITE;
    info[0].NumBytes = 1;

    info[1].Address = ( compass->i2c_address | 0x01 );
    info[1].Buf = transfer_data;
    info[1].Flags = 0;
    info[1].NumBytes = len;

    do{
        i2c_status = NvOdmI2cTransaction( compass->h_gen2_i2c, info, 2, 400, 1000 );
    }while( i2c_status == NvOdmI2cStatus_Timeout );

    if( i2c_status != NvOdmI2cStatus_Success )
    {
        printk("[star driver] %s : i2c transaction error(Number= %d)!\n",__func__, i2c_status);
        goto err;
    }
    NvOdmOsMemcpy( data, transfer_data, len );
    NvOdmOsFree(transfer_data);
    return true;
err:
    NvOdmOsFree(transfer_data);
    return false;
} 

void close_odm_compass(void)
{
//	NvOdmCompassClose(g_compass->hOdmComp);
	//g_compass->hOdmComp = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous 
 * settings will be lost. This function will help in switching
 * between power modes
 */
 /*
int open_def_odm_compass(void)
{
	NvS32 err = -1;
	NvU32 I2cInstance = 0;
	const NvOdmPeripheralConnectivity *pcon;
	NvBool found_gpio = NV_FALSE, found_i2c =NV_FALSE;

	int loop;
	#if STAR_COMPASS_DEBUG
		printk("[skhwang][%s] \n", __func__ );
	#endif
//	err = NvOdmCompassOpen(&(g_compass->hOdmComp));
//	if (!err) {
//		err = -ENODEV;
//		pr_err("open_def_odm_compass: NvOdmCompassOpen failed\n");
//		return err;
//	}
*/	
/*
	err = NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
		 NvOdmAccelInt_MotionThreshold, 0, 900);
	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
		return err;
	}

	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
		NvOdmAccelInt_MotionThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);

	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}
	
	
	err = NvOdmAccelSetIntEnable(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, NvOdmAccelAxis_All, 0, NV_TRUE);

	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntEnable failed\n");
		return err;
	}

	err = NvOdmAccelSetIntForceThreshold(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, 0, 120);

	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntForceThreshold\n");
		return err;
	}

	err = NvOdmAccelSetIntTimeThreshold(accel_dev->hOdmAcr,
		NvOdmAccelInt_TapThreshold, 0, 2);

	if (!err) {
		pr_err("open_def_odm_accl: NvOdmAccelSetIntTimeThreshold\n");
		return err;
	}
	return err;
}
*/

/*
struct ami304_i2c_data {
	struct i2c_client client;
};

static struct i2c_driver ami304_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE, 
		.name	= "ami304 driver",
	},
	.attach_adapter	= ami304_i2c_attach_adapter,
	.detach_client	= ami304_i2c_detach_client,	
	.id		= I2C_DRIVERID_AMI304,
};
*/

static int AMI304_Reset_Init()
{
	u8 databuf[10];
	NvU8 regaddr;
	u8 ctrl1, ctrl2, ctrl3;
	
	
	#if STAR_COMPASS_DEBUG 
		printk("[%s:%d] SRST Setting \n", __FUNCTION__, __LINE__);
	#endif	

	databuf[1] = 0x80;
	#if STAR_COMPASS_DEBUG
	printk("[%s:%d] AMI304_REG_CTRL3 setting = 0x%x \n", __FUNCTION__, __LINE__, databuf[1] );	
	#endif
	//NvCompassI2CSetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);
	star_compass_i2c_write_data( g_compass, AMI304_REG_CTRL3, &databuf[1], 1 );
	
	#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif

	mdelay(20);
	regaddr = AMI304_REG_CTRL3;
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);
	star_compass_i2c_read_data( g_compass, AMI304_REG_CTRL3, &ctrl3, 1); 
	#if STAR_COMPASS_DEBUG
	printk("[%s:%d] AMI304_REG_CTRL3(read) = 0x%x \n", __FUNCTION__, __LINE__, ctrl2);
	#endif
	

	return 0;
}


static atomic_t dev_open_count;
static atomic_t hal_open_count;
static atomic_t daemon_open_count;

static int AMI304_Init(int mode)
{
	u8 databuf[10];
//	u8 regaddr;
	NvU8 regaddr;
	u8 ctrl1, ctrl2, ctrl3;
	
	regaddr = AMI304_REG_CTRL1;
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_CTRL1, &ctrl1, 1);
	star_compass_i2c_read_data(g_compass, AMI304_REG_CTRL1, &ctrl1, 1);
	

	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);

	#if DEBUG_AMI304
		printk("[%s:%d] AMI304_REG_CTRL1 = 0x%x \n", __FUNCTION__, __LINE__, ctrl1);
	#endif

	regaddr = AMI304_REG_CTRL2;
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_CTRL2, &ctrl2, 1);
	star_compass_i2c_read_data(g_compass, AMI304_REG_CTRL2, &ctrl2, 1);
	#if DEBUG_AMI304
		printk("[%s:%d] AMI304_REG_CTRL2 = 0x%x \n", __FUNCTION__, __LINE__, ctrl2);
	#endif

	regaddr = AMI304_REG_CTRL3;
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);
	star_compass_i2c_read_data(g_compass, AMI304_REG_CTRL3, &ctrl3, 1);
	#if DEBUG_AMI304
		printk("[%s:%d] AMI304_REG_CTRL3 = 0x%x \n", __FUNCTION__, __LINE__, ctrl3);
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif	
	databuf[0] = AMI304_REG_CTRL1;
	if( mode==AMI304_FORCE_MODE )
	{
		#if DEBUG_AMI304
			printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
		#endif	
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_FORCE;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_FORCE_MODE;
		write_unlock(&ami304_data.lock);			
	}
	else	
	{
		#if DEBUG_AMI304
			printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
		#endif
		databuf[1] = ctrl1 | AMI304_CTRL1_PC1 | AMI304_CTRL1_FS1_NORMAL | AMI304_CTRL1_ODR1;
		write_lock(&ami304_data.lock);
		ami304_data.mode = AMI304_NORMAL_MODE;
		write_unlock(&ami304_data.lock);			
	}
	#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif
	
	//NvCompassI2CSetRegs(g_compass->hOdmComp, AMI304_REG_CTRL1, &databuf[1], 1);
	star_compass_i2c_write_data( g_compass, AMI304_REG_CTRL1, &databuf[1], 1); 
	#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif
	
	databuf[0] = AMI304_REG_CTRL2;
	databuf[1] = ctrl2 | AMI304_CTRL2_DREN;
	//NvCompassI2CSetRegs(g_compass->hOdmComp, AMI304_REG_CTRL2, &databuf[1], 1);
	star_compass_i2c_write_data( g_compass, AMI304_REG_CTRL2, &databuf[1], 1); 
	#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif
	
	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = ctrl3 | AMI304_CTRL3_B0_LO_CLR;
	//NvCompassI2CSetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);
	star_compass_i2c_write_data( g_compass, AMI304_REG_CTRL3, &databuf[1], 1); 
	#if DEBUG_AMI304
		printk("## [%s:%d] ##\n", __FUNCTION__, __LINE__);
	#endif
	
	return 0;
}

static int AMI304_SetMode(int newmode)
{
	int mode = 0;
	
	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);		
	
	if (mode == newmode) 
		return 0;	
			
	return AMI304_Init(newmode);
}

static int AMI304_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=30))
		return -1;
	/*
	if (!ami304_i2c_client)
	{
		*buf = 0;
		return -2;
	}*/

	sprintf(buf, "AMI304 Chip");
	return 0;
}

static int AMI304_ReadSensorData(char *buf, int bufsize)
{
	char cmd;
	int mode = 0;	
	unsigned char ctrl3;
	unsigned char databuf[10];

	if ((!buf)||(bufsize<=80))
		return -1;
	/*
	if (!ami304_i2c_client)
	{
		*buf = 0;
		return -2;
	}*/
	
	read_lock(&ami304_data.lock);	
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);		

	databuf[0] = AMI304_REG_CTRL3;
	databuf[1] = AMI304_CTRL3_FORCE_BIT;
	//NvCompassI2CSetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &databuf[1], 1);
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_CTRL3, &ctrl3, 1);
	star_compass_i2c_write_data(g_compass, AMI304_REG_CTRL3, &databuf[1], 1 );
	star_compass_i2c_read_data(g_compass, AMI304_REG_CTRL3, &ctrl3, 1 );
	#if DEBUG_AMI304
		printk("[%s:%d] AMI304_REG_CTRL3 = 0x%x \n", __FUNCTION__, __LINE__, ctrl3);
	#endif

	// We can read all measured data in once
	//cmd = AMI304_REG_DATAXH;
	//NvCompassI2CGetRegs(g_compass->hOdmComp, AMI304_REG_DATAXH, &databuf[0], 6);
	star_compass_i2c_read_data(g_compass, AMI304_REG_DATAXH, &databuf[0], 6);


	sprintf(buf, "%02x %02x %02x %02x %02x %02x", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
	#if DEBUG_DATA
		//printk( "============================> COMPASS-DATA %02x %02x %02x %02x %02x %02x\n", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
	#endif 
	#if DEBUG_DATA
	{
		int mx=0;
		int my=0;
		int mz=0;
		int tmp_x=databuf[1];
		int tmp_y=databuf[3];
		int tmp_z=databuf[5];
		
		mx = tmp_x << 8 | databuf[0];
		my = tmp_y << 8 | databuf[2];
		mz = tmp_z << 8 | databuf[4];
		if (mx>32768)  mx = mx-65536;
		if (my>32768)  my = my-65536;
		if (mz>32768)  mz = mz-65536;
		mx += 2048;
		my += 2048;
		mz += 2048;
		printk("%s Magnetic : mx=%d my=%d mz=%d\n",__FUNCTION__, mx, my, mz);
	}
	#endif 
	
	return 0;
}

static int AMI304_ReadPostureData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d", ami304mid_data.yaw, ami304mid_data.pitch, ami304mid_data.roll, ami304mid_data.mag_status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadCaliData(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.datalock);
	sprintf(buf, "%d %d %d %d %d %d %d", ami304mid_data.nmx, ami304mid_data.nmy, ami304mid_data.nmz,ami304mid_data.nax,ami304mid_data.nay,ami304mid_data.naz,ami304mid_data.mag_status);
	read_unlock(&ami304mid_data.datalock);
	return 0;
}

static int AMI304_ReadMiddleControl(char *buf, int bufsize)
{
	if ((!buf)||(bufsize<=80))
		return -1;

	read_lock(&ami304mid_data.ctrllock);
	sprintf(buf, "%d %d %d %d %d %d %d %d %d %d", 
		ami304mid_data.controldata[0], ami304mid_data.controldata[1], ami304mid_data.controldata[2],ami304mid_data.controldata[3],ami304mid_data.controldata[4],
		ami304mid_data.controldata[5], ami304mid_data.controldata[6], ami304mid_data.controldata[7], ami304mid_data.controldata[8], ami304mid_data.controldata[9]);
	read_unlock(&ami304mid_data.ctrllock);
	return 0;
}

static int AMI304_Report_Value(int en_dis)
{
//	struct ami304_i2c_data *data = i2c_get_clientdata(ami304_i2c_client);
	
	if( !en_dis )
		return 0;
	#if DEBUG_DATA
		printk("\n## Kernel Accel ## %s, %d ## %d ,%d ,%d ,%d 	\n",__FUNCTION__, __LINE__, ami304mid_data.nax ,ami304mid_data.nay, 
		ami304mid_data.naz, ami304mid_data.mag_status);
		printk("\n## Kernel Orient ## %s, %d ## %d ,%d ,%d ,%d 	\n",__FUNCTION__, __LINE__, ami304mid_data.yaw ,ami304mid_data.pitch, 
		ami304mid_data.roll, ami304mid_data.mag_status);
		printk("## Kernel Magn   ## %s, %d ## %d ,%d ,%d ,%d \n",__FUNCTION__, __LINE__, ami304mid_data.nmx, ami304mid_data.nmy , 
		ami304mid_data.nmz, ami304mid_data.mag_status );
	#endif
	
	input_report_abs(g_compass->input_dev, ABS_RX, ami304mid_data.yaw);	/* yaw */
	input_report_abs(g_compass->input_dev, ABS_RY, ami304mid_data.pitch);/* pitch */
	input_report_abs(g_compass->input_dev, ABS_RZ, ami304mid_data.roll);/* roll */
	input_report_abs(g_compass->input_dev, ABS_RUDDER, ami304mid_data.mag_status);/* status of orientation sensor */

  //This module is released when compass
	input_report_abs(g_compass->input_dev, ABS_X, ami304mid_data.nax);// x-axis raw acceleration 
	input_report_abs(g_compass->input_dev, ABS_Y, ami304mid_data.nay);// y-axis raw acceleration 
	input_report_abs(g_compass->input_dev, ABS_Z, ami304mid_data.naz);// z-axis raw acceleration

	input_report_abs(g_compass->input_dev, ABS_HAT0X, ami304mid_data.nmx); /* x-axis of raw magnetic vector */
	input_report_abs(g_compass->input_dev, ABS_HAT0Y, ami304mid_data.nmy); /* y-axis of raw magnetic vector */
	input_report_abs(g_compass->input_dev, ABS_BRAKE, ami304mid_data.nmz); /* z-axis of raw magnetic vector */
	input_report_abs(g_compass->input_dev, ABS_WHEEL, ami304mid_data.mag_status);/* status of magnetic sensor */

	input_sync(g_compass->input_dev);
	   
	return 0;
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);		
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_posturedata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_calidata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t show_midcontrol_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	char strbuf[AMI304_BUFSIZE];
	AMI304_ReadMiddleControl(strbuf, AMI304_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);			
}

static ssize_t store_midcontrol_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	write_lock(&ami304mid_data.ctrllock);
	memcpy(&ami304mid_data.controldata[0], buf, sizeof(int)*10);	
 	write_unlock(&ami304mid_data.ctrllock);		
	return count;			
}

static ssize_t show_mode_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int mode=0;
	read_lock(&ami304_data.lock);
	mode = ami304_data.mode;
	read_unlock(&ami304_data.lock);		
	return sprintf(buf, "%d\n", mode);			
}

static ssize_t store_mode_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d", &mode);	
 	AMI304_SetMode(mode);
	return count;			
}

static DEVICE_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DEVICE_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DEVICE_ATTR(calidata, S_IRUGO, show_calidata_value, NULL);
static DEVICE_ATTR(midcontrol, S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );

static struct attribute *ami304_attributes[] = {
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_posturedata.attr,
	&dev_attr_calidata.attr,
	&dev_attr_midcontrol.attr,
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group ami304_attribute_group = {
	.attrs = ami304_attributes
};

static int ami304_open(struct inode *inode, struct file *file)
{	
	int ret = -1;
	#if DEBUG_AMI304
		printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
	#endif
	if( atomic_cmpxchg(&dev_open_count, 0, 1)==0 ) {
		printk(KERN_INFO "Open device node:ami304\n");
		ret = nonseekable_open(inode, file);
	}	
	return ret;
}

static int ami304_release(struct inode *inode, struct file *file)
{
	atomic_set(&dev_open_count, 0);
	#if DEBUG_AMI304
		printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
	#endif
	
	return 0;
}

static int ami304_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	char strbuf[AMI304_BUFSIZE];
	int controlbuf[10];
	void __user *data;
	int retval=0;
	int mode=0;
	#if DEBUG_AMI304
		printk(KERN_ERR  "ami304 - %s \n",__FUNCTION__);
	#endif
	
    //check the authority is root or not
    if(!capable(CAP_SYS_ADMIN)) {
        retval = -EPERM;
        goto err_out;
	}
		
	switch (cmd) {
		case AMI304_IOCTL_INIT:
			read_lock(&ami304_data.lock);
			mode = ami304_data.mode;
			read_unlock(&ami304_data.lock);
			AMI304_Init(mode);			
			break;
		
		case AMI304_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadChipInfo(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;

		case AMI304_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;				
						
		case AMI304_IOCTL_READ_POSTUREDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;			
	 
	        case AMI304_IOCTL_READ_CALIDATA:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
	        	break;
	        
	        case AMI304_IOCTL_READ_CONTROL:
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}						        
	        	break;

		case AMI304_IOCTL_SET_CONTROL:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);		
			break;
			
		case AMI304_IOCTL_SET_MODE:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}		
			AMI304_SetMode(mode);				
			break;
					        				
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static int ami304daemon_open(struct inode *inode, struct file *file)
{
	//return nonseekable_open(inode, file);
	int ret = -1;
	if( atomic_cmpxchg(&daemon_open_count, 0, 1)==0 ) {
		printk(KERN_INFO "Open device node:ami304daemon\n");
		ret = 0;
	}
	return ret;	
}

static int ami304daemon_release(struct inode *inode, struct file *file)
{
	atomic_set(&daemon_open_count, 0);
	#if DEBUG_AMI304
		printk(KERN_INFO "[%s:%d]Compass sensor \n",__FUNCTION__, __LINE__);	
	#endif
	return 0;
}

static int ami304daemon_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	int valuebuf[4];
	int calidata[7];
	int controlbuf[10];
	char strbuf[AMI304_BUFSIZE];
	void __user *data;
	int retval=0;
	int mode;
	int en_dis_Report;

    //check the authority is root or not
    if(!capable(CAP_SYS_ADMIN)) {
        retval = -EPERM;
        goto err_out;
    }
		
	switch (cmd) {
			
		case AMI304MID_IOCTL_GET_SENSORDATA:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_GET_SENSORDATA\n");	
			#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}
			break;
				
		case AMI304MID_IOCTL_SET_POSTURE:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_POSTURE\n");	
			#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&valuebuf, data, sizeof(valuebuf))) {
				retval = -EFAULT;
				goto err_out;
			}				
			write_lock(&ami304mid_data.datalock);
			ami304mid_data.yaw   = valuebuf[0];
			ami304mid_data.pitch = valuebuf[1];
			ami304mid_data.roll  = valuebuf[2];
			ami304mid_data.mag_status = valuebuf[3];
			write_unlock(&ami304mid_data.datalock);	
			break;		
			
		case AMI304MID_IOCTL_SET_CALIDATA:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_CALIDATA\n");	
			#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&calidata, data, sizeof(calidata))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.datalock);			
			ami304mid_data.nmx = calidata[0];
			ami304mid_data.nmy = calidata[1];
			ami304mid_data.nmz = calidata[2];
			ami304mid_data.nax = calidata[3];
			ami304mid_data.nay = calidata[4];
			ami304mid_data.naz = calidata[5];
			ami304mid_data.mag_status = calidata[6];
			write_unlock(&ami304mid_data.datalock);	
//			AMI304_Report_Value(en_dis_Report);
			break;								

		case AMI304MID_IOCTL_GET_CONTROL:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_GET_CONTROL\n");	
			#endif
			read_lock(&ami304mid_data.ctrllock);
			memcpy(controlbuf, &ami304mid_data.controldata[0], sizeof(controlbuf));
			read_unlock(&ami304mid_data.ctrllock);			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_to_user(data, controlbuf, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}					
			break;		
			
		case AMI304MID_IOCTL_SET_CONTROL:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_CONTROL\n");	
			#endif
			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(controlbuf, data, sizeof(controlbuf))) {
				retval = -EFAULT;
				goto err_out;
			}	
			write_lock(&ami304mid_data.ctrllock);
			memcpy(&ami304mid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&ami304mid_data.ctrllock);
			break;	
	
		case AMI304MID_IOCTL_SET_MODE:
			#if DEBUG_DAEMON_IOCTL
				printk("Release ami304daemon_ioctl, :AMI304MID_IOCTL_SET_MODE\n");	
			#endif
			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&mode, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}		
			AMI304_SetMode(mode);				
			break;
								
		//Add for input_device sync			
		case AMI304MID_IOCTL_SET_REPORT:
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			if (copy_from_user(&en_dis_Report, data, sizeof(mode))) {
				retval = -EFAULT;
				goto err_out;
			}				
			read_lock(&ami304mid_data.datalock);			
			AMI304_Report_Value(en_dis_Report);
			read_unlock(&ami304mid_data.datalock);						
			break;
						
		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static int ami304hal_open(struct inode *inode, struct file *file)
{
	//return nonseekable_open(inode, file);	
	atomic_inc_and_test(&hal_open_count);
	#if DEBUG_AMI304
		printk(KERN_INFO "Open device node:ami304hal %d times.\n", atomic_read(&hal_open_count));	
	#endif
	
	return 0;
}

static int ami304hal_release(struct inode *inode, struct file *file)
{
	atomic_dec_and_test(&hal_open_count);
	#if DEBUG_AMI304
		printk(KERN_INFO "Release ami304hal, remainder is %d times.\n", atomic_read(&hal_open_count));	
	#endif
	return 0;
}

static int ami304hal_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	char strbuf[AMI304_BUFSIZE];
	void __user *data;
	int retval=0;
	#if DEBUG_AMI304
		printk("Release ami304hal_ioctl\n");	
	#endif
	
	switch (cmd) {
		
		case AMI304HAL_IOCTL_GET_SENSORDATA:
			#if DEBUG_HAL_IOCTL			
				printk("Release ami304hal_ioctl, AMI304HAL_IOCTL_GET_SENSORDATA:\n");	
			#endif
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadSensorData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}		
			break;
									
		case AMI304HAL_IOCTL_GET_POSTURE:
			#if DEBUG_HAL_IOCTL			
				printk("Release ami304hal_ioctl, :AMI304HAL_IOCTL_GET_POSTURE\n");	
			#endif
			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadPostureData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
			break;			
	 
		case AMI304HAL_IOCTL_GET_CALIDATA:
			#if DEBUG_HAL_IOCTL			
				printk("Release ami304hal_ioctl, :AMI304HAL_IOCTL_GET_CALIDATA\n");	
			#endif
			
			data = (void __user *) arg;
			if (data == NULL)
				break;	
			AMI304_ReadCaliData(strbuf, AMI304_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				retval = -EFAULT;
				goto err_out;
			}				
	        	break;

		default:
			#if DEBUG_HAL_IOCTL			
				printk("Release ami304hal_ioctl, :\n");	
			#endif
			printk("[error] ami304hal_ioctl %s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
	}
	
err_out:
	return retval;	
}

static struct file_operations ami304_fops = {
	.owner = THIS_MODULE,
	.open = ami304_open,
	.release = ami304_release,
	.ioctl = ami304_ioctl,
};

static struct miscdevice ami304_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304",
	.fops = &ami304_fops,
};


static struct file_operations ami304daemon_fops = {
	.owner = THIS_MODULE,
	.open = ami304daemon_open,
	.release = ami304daemon_release,
	.ioctl = ami304daemon_ioctl,
};

static struct miscdevice ami304daemon_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304daemon",
	.fops = &ami304daemon_fops,
};

static struct file_operations ami304hal_fops = {
	.owner = THIS_MODULE,
	.open = ami304hal_open,
	.release = ami304hal_release,
	.ioctl = ami304hal_ioctl,
};

static struct miscdevice ami304hal_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ami304hal",
	.fops = &ami304hal_fops,
};
/*
static int ami304_i2c_attach_adapter(struct i2c_adapter *adapter)
{
	int res;
	printk(KERN_INFO "\n\nEnter ami304_i2c_attach_adapter!!\n");	
	res = i2c_probe(adapter, &addr_data, ami304_i2c_detect);
	printk(KERN_INFO "      res of ami304_i2c_attach_adapter= %d\n", res); 
	return res;
}
*/

// 20100702 taewan.kim@lge.com Power control bug fix [START]
static void star_compass_set_power_rail(NvU32 vdd_id, NvBool is_enable )
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();

    if(h_pmu)
    {
        NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
        if( is_enable )
        {
            #if STAR_COMPASS_DEBUG
            printk("[skhwang] PMU enable\n");
            #endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
            #if STAR_COMPASS_DEBUG
            printk("[skhwang] PMU do not enable\n");
            #endif
            NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
        }

        if(settletime)
            NvOdmOsWaitUS(settletime);
    }
    #if STAR_COMPASS_DEBUG
        printk("[skhwang] voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
    #endif
    NvOdmServicesPmuClose(h_pmu);
}
// 20100702 taewan.kim@lge.com Power control bug fix [END]

/**
 * All the device spefic initializations happen here. 
 */
static NvS32 __init ami304_probe(struct platform_device *pdev)
{

//	struct i2c_client *new_client = ami304_i2c_client;
//	struct star_g_compassice_data *compass = NULL;
//	struct input_dev *input_dev = NULL;
	NvS32 err;

	//sk.hwang start
	NvU32 I2cInstance;
	const NvOdmPeripheralConnectivity *pcon;
	struct device *dev = &pdev->dev;
	NvBool found_gpio=NV_FALSE, found_i2c=NV_FALSE;
	int loop;
	//sk.hwang end

	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
	g_compass = kzalloc(sizeof(*g_compass), GFP_KERNEL);
	if (g_compass == NULL) {
		err = -ENOMEM;
		pr_err("ami304_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
//	g_compass = compass;
	

	g_compass->input_dev = input_allocate_device();
	if (!g_compass->input_dev) {
		err = -ENOMEM;
		pr_err("tegra_com_ami304_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
//	g_compass->input_dev 	= input_dev;
	

/*
	err = open_def_odm_compass();
	if (!err) {
		pr_err("open_def_odm_comp: Failed \n");
		goto exit_alloc_data_failed;
	}
*/
    // 20100702 taewan.kim@lge.com Power control bug fix [START]
	/*g_compass->h_compass_pmu = NvOdmServicesPmuOpen();
	if(!g_compass->h_compass_pmu)
	{
		err = -ENOMEM;
		printk("[skhwang][compass] : fail to Open PMU\n");
		goto fail_to_open_pmu;
	}*/
    // 20100702 taewan.kim@lge.com Power control bug fix [END]
	
	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('c','o','m','p','a','s','s','-'));
	for( loop= 0; loop< pcon->NumAddress; loop++ )
	{
		switch(pcon->AddressList[loop].Interface)	
		{
			case NvOdmIoModule_I2c:
                g_compass->i2c_address = (pcon->AddressList[loop].Address<<1);
                I2cInstance = pcon->AddressList[loop].Instance;
                found_i2c = NV_TRUE;
				#if STAR_COMPASS_DEBUG
				printk("[skhwang][%s] I2c Addr = %#x, I2c Ch = %d\n", __func__, g_compass->i2c_address, I2cInstance );
				#endif
                break;
            case NvOdmIoModule_Gpio:
                g_compass->intr_port = pcon->AddressList[loop].Instance;
                g_compass->intr_pin = pcon->AddressList[loop].Address;
                found_gpio = NV_TRUE;
				#if STAR_COMPASS_DEBUG
				//check the pin number... AP20GPIO Macro bug...
				printk("[skhwang][%s] Gpio Port = %c Gpio Pin = %d\n", __func__, g_compass->intr_port+'a', g_compass->intr_pin);
				#endif
                break;
            case NvOdmIoModule_Vdd:
                g_compass->vdd_id = pcon->AddressList[loop].Address;
                #if STAR_COMPASS_DEBUG
                    printk("[skhwang] AMI304 POWER %d\n", g_compass->vdd_id );
                #endif
                // 20100702 taewan.kim@lge.com Power control bug fix
                star_compass_set_power_rail( g_compass->vdd_id, NV_TRUE);
				NvOdmOsWaitUS(30);
                break;
            default:
                break;
		}
	}
	
	if( !found_i2c ||  !found_gpio)
	{
		printk("[skhwang][%s] Not found i2c or gpio\n");
		err = -EINVAL;
		goto fail_to_open_pmu;
	}
	g_compass->h_compass_gpio = NvOdmGpioOpen();
	if( !g_compass->h_compass_gpio )
	{
		printk("[skhwang][%s] Fail to open gpio\n");
		err = -ENOSYS;
		goto err_open_gpio;
	}
	
	g_compass->h_compass_gpio_pin = NvOdmGpioAcquirePinHandle( g_compass->h_compass_gpio, g_compass->intr_port, g_compass->intr_pin );
	if( !g_compass->h_compass_gpio_pin )
	{
		printk("[skhwang][%s] Fail to acquire pin handle\n");
		err = -ENOSYS;
		goto err_acquire_pin_handle;
	}

	g_compass->h_gen2_i2c = NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2 );
	if( !g_compass->h_gen2_i2c )
	{
		printk("[skhwang][%s] Fail to Open I2C\n");
		err = -ENOSYS;
		goto err_open_i2c;
	}
	

	mdelay(50); //to waiting time(50ms) from PowerOFF to Stand-by
	//On soft reset
	AMI304_Reset_Init();
	
	AMI304_Init(AMI304_FORCE_MODE); // default is Force State	
	#if STAR_COMPASS_DEBUG
	printk("[%s:%d] Compass Sensor: AMI304 registered driver! \n", __FUNCTION__, __LINE__);
	#endif

	err = misc_register(&ami304_device);
	if (err) {
		printk(KERN_ERR
		       "ami304_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	#if 1
	platform_set_drvdata(pdev, g_compass);
	set_bit(EV_ABS, g_compass->input_dev->evbit);
	/* yaw */
	input_set_abs_params(g_compass->input_dev, ABS_RX, 0, 360, 0, 0);
	/* pitch */
	input_set_abs_params(g_compass->input_dev, ABS_RY, -180, 180, 0, 0);
	/* roll */
	input_set_abs_params(g_compass->input_dev, ABS_RZ, -90, 90, 0, 0);
	/* status of magnetic sensor */	
	input_set_abs_params(g_compass->input_dev, ABS_RUDDER, 0, 5, 0, 0);
	
	/* x-axis acceleration */
	input_set_abs_params(g_compass->input_dev, ABS_X, -2000, 2000, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(g_compass->input_dev, ABS_Y, -2000, 2000, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(g_compass->input_dev, ABS_Z, -2000, 2000, 0, 0);
	
	/* x-axis of raw magnetic vector */
	input_set_abs_params(g_compass->input_dev, ABS_HAT0X, -3000, 3000, 0, 0);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(g_compass->input_dev, ABS_HAT0Y, -3000, 3000, 0, 0);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(g_compass->input_dev, ABS_BRAKE, -3000, 3000, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(g_compass->input_dev, ABS_WHEEL, 0, 5, 0, 0);	

	#if STAR_COMPASS_DEBUG
	printk("[%s:%d] Compass Sensor \n", __FUNCTION__, __LINE__);
	#endif
	g_compass->input_dev->name = "Acompass";
	err = input_register_device(g_compass->input_dev);
	if (err) {
		pr_err("tegra_compass_probe: Unable to register %s\
				input device\n", g_compass->input_dev->name);
		goto exit_input_register_device_failed;
	}
	
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_chipinfo);
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_sensordata);
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_posturedata);
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_calidata);
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_midcontrol);
	err = device_create_file(&g_compass->input_dev->dev, &dev_attr_mode);

	#else
	platform_set_drvdata(pdev, g_compass);
	
	input_dev->name = "accelerometer_tegra";
	err = input_register_device(g_compass->input_dev);
	if (err) {
		pr_err("tegra_compass_probe: Unable to register %s\
				input device\n", g_compass->input_dev->name);
		goto input_register_device_failed;
	}

	err = add_sysfs_entry();
//	err = sysfs_create_group(&g_compass->input_dev, &ami304_attribute_group);
	if (err)
		printk("goto err1;\n");
	
 	#endif
	err = misc_register(&ami304daemon_device);
	if (err) {
		printk(KERN_ERR
		       "ami304daemon_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	
	err = misc_register(&ami304hal_device);
	if (err) {
		printk(KERN_ERR
		       "ami304hal_device register failed\n");
		goto exit_misc_device_register_failed;
	}	
	return 0;
exit_input_register_device_failed:
	input_free_device(g_compass->input_dev);
allocate_dev_fail:
exit_misc_device_register_failed:
	NvOdmOsFree(g_compass);
err_open_i2c:
	NvOdmOsFree(g_compass->h_gen2_i2c);
err_acquire_pin_handle:
	NvOdmOsFree(g_compass->h_compass_gpio_pin);
err_open_gpio:
	NvOdmOsFree(g_compass->h_compass_gpio);
fail_to_open_pmu:
	//NvOdmOsFree(g_compass->h_compass_pmu);
/*
exit_alloc_data_failed:
	close_odm_compass();
	input_free_device(g_compass->input_dev);
	kfree(g_compass);
	g_compass = 0;
	err = -ENOMEM;
exit:
*/
	return err;
}



static NvS32 ami304_remove(struct platform_device *pdev)
{
	int err;
	//struct ami304_i2c_data *data = i2c_get_clientdata(client);
	input_unregister_device(g_compass->input_dev);

	close_odm_compass();

	kfree(g_compass);	
	misc_deregister(&ami304hal_device);
	misc_deregister(&ami304daemon_device);
	misc_deregister(&ami304_device);	
	return 0;
}

static struct platform_driver star_compass_driver = {
	.probe	= ami304_probe,
	.remove = ami304_remove,
	.driver = {
		.name = "star_compass",
	},
};

static NvS32 __devinit  ami304_init(void)
{
	#if STAR_COMPASS_DEBUG
	printk(KERN_INFO "AMI304 MI sensor driver: init\n");
	#endif
	
	rwlock_init(&ami304mid_data.ctrllock);
	rwlock_init(&ami304mid_data.datalock);
	rwlock_init(&ami304_data.lock);
	memset(&ami304mid_data.controldata[0], 0, sizeof(int)*10);	
	ami304mid_data.controldata[0] = 200000; //Loop Delay (sleep time) , 200ms
	ami304mid_data.controldata[1] = 0; // Run	
	ami304mid_data.controldata[2] = 0; // Disable Start-AccCali
	ami304mid_data.controldata[3] = 1; // Enable Start-Cali
	ami304mid_data.controldata[4] = 250; // MW-Timout ( calibration )
	ami304mid_data.controldata[5] = 10; // MW-IIRStrength_M
	ami304mid_data.controldata[6] = 10; // MW-IIRStrength_G	
	atomic_set(&dev_open_count, 0);	
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);
	return platform_driver_register(&star_compass_driver);
}

static void __exit ami304_exit(void)
{
	atomic_set(&dev_open_count, 0);
	atomic_set(&hal_open_count, 0);
	atomic_set(&daemon_open_count, 0);	
	platform_driver_unregister(&star_compass_driver);
}

module_init(ami304_init);
module_exit(ami304_exit);

//MODULE_AUTHOR("Kyle K.Y. Chen");
MODULE_DESCRIPTION("AMI304 MI sensor input_dev driver v1.0.5.10");
MODULE_LICENSE("GPL");
