#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
//#include <linux/i2c.h>

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"
#include "star_accel.h"

#define STAR_ACCEL_DEBUG 0
#define STAR_ACCEL_NOT_IOCTL 0 //1 : scheduled by kernel , 0: scheduled by ami304d 


//static struct input_dev *g_input_dev;


struct accelerometer_data {
    NvU32 x;
    NvU32 y;
    NvU32 z;
};


typedef struct star_accel_device_data
{
	NvOdmServicesI2cHandle h_gen2_i2c;
	NvOdmServicesGpioHandle h_accel_gpio;
	NvOdmGpioPinHandle	h_accel_gpio_pin;
	NvOdmServicesGpioIntrHandle h_accel_intr;
	NvOdmServicesPmuHandle	h_accel_pmu;
	NvU32	i2c_address;
	NvU32	vdd_id;
	NvU32	intr_pin;
	NvU32	intr_port;
	struct  input_dev *input_dev;
	struct delayed_work delayed_work_accel;
	
    struct accelerometer_data prev_data;
    struct accelerometer_data min_data;
    struct accelerometer_data max_data;
}star_accel_device;

static star_accel_device *g_accel;

//struct tegra_acc_device_data *accel_dev;


static bool star_accel_i2c_write_data( star_accel_device *accel, unsigned char reg, unsigned char* data, unsigned char len )
{
	NvOdmI2cStatus i2c_status;
	NvOdmI2cTransactionInfo info;
	unsigned char* transfer_data;

	transfer_data = (unsigned char*)NvOdmOsAlloc(len+1);
    transfer_data[0] = reg; 
    NvOdmOsMemcpy( &transfer_data[1], data, (size_t)len);

    info.Address = accel->i2c_address;
    info.Buf = transfer_data;
    info.Flags = NVODM_I2C_IS_WRITE;
    info.NumBytes = len+1;
	
	do{  
        i2c_status = NvOdmI2cTransaction( accel->h_gen2_i2c, &info, 1, 400, 1000 );
    }while(i2c_status == NvOdmI2cStatus_Timeout);

    if( i2c_status != NvOdmI2cStatus_Success )
    {    
        printk("[star accel driver] %s : i2c transaction error(Number = %d)!\n",__func__,i2c_status);
        goto err; 
    }    
    NvOdmOsFree(transfer_data); 
    return true;
err:
    NvOdmOsFree(transfer_data);
    return false;
}

static bool star_accel_i2c_read_data( star_accel_device *accel, unsigned char reg, unsigned char* data, unsigned char len )
{
	NvOdmI2cStatus i2c_status;
    NvOdmI2cTransactionInfo info[2];
    unsigned char* transfer_data;

	transfer_data = (unsigned char*)NvOdmOsAlloc(len);

	#if STAR_ACCEL_DEBUG
	//printk("address = %#x\n", g_accel->i2c_address);
	#endif

    info[0].Address = g_accel->i2c_address;
    info[0].Buf = &reg;
    info[0].Flags = NVODM_I2C_IS_WRITE;
    info[0].NumBytes = 1;

    info[1].Address = ( g_accel->i2c_address | 0x01 );
    info[1].Buf = transfer_data;
    info[1].Flags = 0;
    info[1].NumBytes = len;

    do{
        i2c_status = NvOdmI2cTransaction( g_accel->h_gen2_i2c, info, 2, 400, 1000 );
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

static void star_accel_set_sample_rate( star_accel_device *accel, unsigned int samplerate )
{
	unsigned int sampledata[7][2] =
	{
		{ 12,  0x00 },
		{ 25,  0x01 },
		{ 50,  0x02 },
		{ 100, 0x03 },
		{ 200, 0x04 },
		{ 400, 0x05 },
		{ 800, 0x06 },
	};
	unsigned char i;
	unsigned char rate;
	for( i= 0; i< 7; i++ )
	{
		if(sampledata[i][0] == samplerate )
		{
			rate = (unsigned char)sampledata[i][1];
			break;
		}
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] i = %d, rate = %d, rate of index = %d\n", __func__, i, sampledata[i][0], rate); 
	#endif
	star_accel_i2c_write_data( accel, KXTF9_I2C_DATA_CTRL_REG, &rate, 1 );
}

static bool star_accel_get_data( star_accel_device *accel, int *x, int *y, int *z )
{
	NvU32 Res, G_range = 0;
//	NvU32 data_L = 0, data_H = 0;
	NvU8  acc_data[2];
	int   hw_d[3];
	NvS8 x_sign, y_sign, z_sign;
	NvS32 sensitivity, range;
	star_accel_i2c_read_data( g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &G_range, 1 );
	Res = G_range;

	G_range = G_range & 0x18;
	G_range = G_range >> 3;
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

	Res = Res & 0x40;
	if( Res == 0x40 )
	{
		/*
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_XOUT_H, &data_H, 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_XOUT_L, &data_L, 1 );	
		*x = ((NvS32)data_L) >> 4;
		*x = *x + (((NvS32)data_H) << 4);
		x_sign = *x >> 11;
		if(x_sign == 1)
		{
			*x = ((~(*x) + 0x01) & 0x0FF);	
			*x = -(*x);
		}

		star_accel_i2c_read_data( g_accel, KXTF9_I2C_YOUT_H, &data_H, 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_YOUT_L, &data_L, 1 );	
		*y = ((NvS32)data_L) >> 4;
        *y = *y + (((NvS32)data_H) << 4 );              
		y_sign = *y >> 11 ;  
		if( y_sign == 1 )    
		{                        
			*y = ((~(*y) + 0x01) & 0x0FF);  // 2's complement
			*y = -(*y);
		}   

		star_accel_i2c_read_data( g_accel, KXTF9_I2C_ZOUT_H, &data_H, 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_ZOUT_L, &data_L, 1 );	
		 *z = ((NvS32)data_L) >> 4 ;
         *z = *z + (((NvS32)data_H) << 4 ) ;
         z_sign = *z >> 11 ;  
        if( z_sign == 1 )
        {
   		     *z = ((~(*z) + 0x01) & 0x0FF);  // 2's complement
             *z = -(*z);
        }
		*/
		
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_XOUT_H, &acc_data[1], 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_XOUT_L, &acc_data[0], 1 );	
		hw_d[0] = (int) (((acc_data[1]) << 8) | acc_data[0] );
		hw_d[0] = (hw_d[0] & 0x8000) ? ( hw_d[0] | 0xFFFF0000 ) : (hw_d[0]);	
		hw_d[0] >>= 4;
		*x = hw_d[0];

		star_accel_i2c_read_data( g_accel, KXTF9_I2C_YOUT_H, &acc_data[1], 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_YOUT_L, &acc_data[0], 1 );	
		hw_d[1] = (int) (((acc_data[1]) << 8) | acc_data[0] );
		hw_d[1] = (hw_d[1] & 0x8000) ? ( hw_d[1] | 0xFFFF0000 ) : (hw_d[1]);	
		hw_d[1] >>= 4;
		*y = hw_d[1];
		
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_ZOUT_H, &acc_data[1], 1 );	
		star_accel_i2c_read_data( g_accel, KXTF9_I2C_ZOUT_L, &acc_data[0], 1 );	
		hw_d[2] = (int) (((acc_data[1]) << 8) | acc_data[0] );
		hw_d[2] = (hw_d[2] & 0x8000) ? ( hw_d[2] | 0xFFFF0000 ) : (hw_d[2]);	
		hw_d[2] >>= 4;
		*z = hw_d[2];
		#if STAR_ACCEL_DEBUG
		//printk("[row data]x = %#x, y = %#x, z = %#x \n", *x, *y, *z );
		printk("[row data]x = %5d, y = %5d, z = %5d \n", *x, *y, *z );
		#endif
		
	
		sensitivity = (4096)/(2*range);
		*x = 1000 * (*x) / sensitivity; 
		*y = 1000 * (*y) / sensitivity; 
		*z = 1000 * (*z) / sensitivity; 
	}
	#if STAR_ACCEL_DEBUG
//	printk("[skhwang][%s] : G_range = %#x\n", __func__, G_range );
	#endif


	return true;
}
#if STAR_ACCEL_NOT_IOCTL 
static void star_accel_delayed_wq_func(struct work_struct *work)
{
	int x,y,z;
	#if STAR_ACCEL_DEBUG
	printk("[skhwang][%s] ...\n", __func__);
	#endif
	star_accel_get_data(g_accel, &x, &y, &z);
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]accelerometer data x = %d, y = %d, z = %d \n",x, y, z); 
	#endif

	input_report_abs(g_accel->input_dev, ABS_X, x );
	input_report_abs(g_accel->input_dev, ABS_Y, y );
	input_report_abs(g_accel->input_dev, ABS_Z, z );
	input_sync(g_accel->input_dev);

	schedule_delayed_work(&g_accel->delayed_work_accel, 50);
}
#endif

static void star_accel_whoami( void )
{
	#define WHO_AM_I 0x0f
	
	unsigned char value;

	star_accel_i2c_read_data( g_accel, WHO_AM_I, &value, 1 );

	#if STAR_ACCEL_DEBUG
	printk("[skhwang][%s]: WHO AM I = %#x \n", __func__, value );
	#endif
}

// 20100702 taewan.kim@lge.com Power control bug fix [START]
static void star_accel_set_power_rail(NvU32 vdd_id, NvBool is_enable )
{
	NvOdmServicesPmuVddRailCapabilities vddrailcap;
	NvU32 settletime;

    NvOdmServicesPmuHandle h_pmu = NvOdmServicesPmuOpen();

	if(h_pmu)
	{
		NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
		if( is_enable )
		{
			#if STAR_ACCEL_DEBUG
			printk("[skhwang] PMU enable\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime);
		}
		else
		{
			#if STAR_ACCEL_DEBUG
			printk("[skhwang] PMU do not enable\n");
			#endif
			NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
		}

		if(settletime)
			NvOdmOsWaitUS(settletime);
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang] voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts);
	#endif
	NvOdmServicesPmuClose(h_pmu);
}
// 20100702 taewan.kim@lge.com Power control bug fix [END]

static int star_accel_misc_open( struct inode *inode, struct file *file )
{
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] function...\n");
	#endif
	return nonseekable_open(inode, file);
}

static int star_accel_misc_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	void __user *argp = (void __user *)arg;
	NvS32 x=0, y=0, z=0;
	int xyz[3] = { 0 };
	
	switch(cmd)
	{
		case KXTF9_IOCTL_READ_ACCEL_XYZ:
			star_accel_get_data( g_accel, &x, &y, &z );
			xyz[0] = x; xyz[1] = y; xyz[2] = z;
			if( copy_to_user(argp, xyz, sizeof(int)*3))
				return -EINVAL;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static const struct file_operations star_accel_misc_fop =
{
	.owner = THIS_MODULE,
	.open = star_accel_misc_open,
	.ioctl = star_accel_misc_ioctl,
};

static struct miscdevice star_accel_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtf9",
	.fops = &star_accel_misc_fop,
};

static int __init star_accel_probe( struct platform_device *pdev )
{
	NvU32 I2cInstance = 0;
	const NvOdmPeripheralConnectivity *pcon;
	struct device* dev = &pdev->dev;
	int err;
	NvBool found_gpio=NV_FALSE, found_i2c=NV_FALSE;
	unsigned char ctr_reg[4];
	unsigned char ctr_val[4];
	unsigned char reg_val;

	int loop;
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]%s\n", __func__);
	#endif

	g_accel = kzalloc(sizeof(*g_accel), GFP_KERNEL);
	if( g_accel == NULL )
	{
		err = -ENOMEM;
		printk("[skhwang][%s], Failed to alloc the memory\n", __func__);
		goto failtomemorydev;
	}

    // 20100702 taewan.kim@lge.com Power control bug fix [START]
	/*g_accel->h_accel_pmu = NvOdmServicesPmuOpen();
	if( !g_accel->h_accel_pmu )
	{err=-ENOSYS; goto failtomemorydev;}*/
    // 20100702 taewan.kim@lge.com Power control bug fix [START]

	pcon = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(NV_ODM_GUID('a','c','c','e','l','e','r','o'));
	for(loop = 0; loop< pcon->NumAddress; loop++)
	{
		switch(pcon->AddressList[loop].Interface)
		{
			case NvOdmIoModule_I2c:
				g_accel->i2c_address = (pcon->AddressList[loop].Address<<1);
				I2cInstance = pcon->AddressList[loop].Instance;
				found_i2c = NV_TRUE;
				break;
			case NvOdmIoModule_Gpio:
				g_accel->intr_port = pcon->AddressList[loop].Instance;
				g_accel->intr_pin = pcon->AddressList[loop].Address;
				found_gpio = NV_TRUE;
				break;
			case NvOdmIoModule_Vdd:
				g_accel->vdd_id = pcon->AddressList[loop].Address;
				#if STAR_ACCEL_DEBUG
					printk("[skhwang] KXTF9 POWER %d\n", g_accel->vdd_id );
				#endif
                // 20100702 taewan.kim@lge.com Power control bug fix
				star_accel_set_power_rail(g_accel->vdd_id, NV_TRUE);
				NvOdmOsWaitUS(30);
				break;
			default:
				break;
		}
	}
	#if STAR_ACCEL_DEBUG
		printk("[skhwang][%s] : I2c Address = %#x, Int Port = %c, Int Pin = %d\n", __func__,
			g_accel->i2c_address, (g_accel->intr_port+'a'), g_accel->intr_pin);
	#endif
	if( found_i2c != NV_TRUE || found_gpio != NV_TRUE )
	{printk("[skhwang][%s] : I2c or Gpio not found...\n",__func__); err = -ENOMEM; goto failtomemorydev;}

	g_accel->h_accel_gpio = NvOdmGpioOpen();
	if(!g_accel->h_accel_gpio )
	{
		printk("[skhwang][%s] : Failed to open gpio\n",__func__);
		err = - ENOSYS;
		goto err_open_gpio;
	}

	g_accel->h_accel_gpio_pin = NvOdmGpioAcquirePinHandle( g_accel->h_accel_gpio, g_accel->intr_port, g_accel->intr_pin );
	if(!g_accel->h_accel_gpio_pin)
	{
		printk("[skhwang][%s] : Failed to acquire the pin handle\n",__func__);
		err = -ENOSYS;
		goto err_acquire_pin;
	}
	
	#if STAR_ACCEL_DEBUG
		printk("[skhwang]Hu~~~ accel initialization end!!\n");
	#endif

	//#if STAR_ACCEL_INTR
	//int mode 
	//#endif

	g_accel->h_gen2_i2c = NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2 );
	if( !g_accel->h_gen2_i2c )
	{
		printk("[skhwang][%s] : failed to open I2c\n", __func__ );
		err = -ENOSYS;
		goto err_open_i2c;
	}	


	star_accel_whoami();
	//start to initialize KXTF9
	ctr_reg[0] = KIONIX_ACCEL_I2C_TILT_TIMER;
	ctr_val[0] =0x01;
	ctr_reg[1] =KIONIX_ACCEL_I2C_CTRL_REG2;
	ctr_val[1] =0x3f;
	ctr_reg[2] =KIONIX_ACCEL_I2C_CTRL_REG3;
	ctr_val[2] =0x4d;
	ctr_reg[3] =KXTF9_I2C_INT_CTRL_REG1;
	ctr_val[3] =0x10;

	for(loop= 0; loop< 4; loop++ )
		star_accel_i2c_write_data(g_accel, ctr_reg[loop], &ctr_val[loop], 1);
	
	star_accel_i2c_read_data(g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val, 1 );
	reg_val |= CTRL_REG1_TPS;
	reg_val |= CTRL_REG1_RES;
	star_accel_i2c_write_data(g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val, 1 );

	star_accel_i2c_read_data(g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val, 1 );
	reg_val |= CTRL_REG1_PC1;
	star_accel_i2c_write_data(g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val, 1 );

	#if STAR_ACCEL_DEBUG
		star_accel_i2c_read_data(g_accel, KIONIX_ACCEL_I2C_CTRL_REG1, &reg_val, 1 );
		printk("[skhwang] KIONIX_CTRL_REG1 = %#x\n", reg_val );
		star_accel_i2c_read_data(g_accel, 0x21, &reg_val, 1 );
		printk("[skhwang] KIONIX_DATA_CTRL_REG = %#x\n", reg_val );
	#endif

	star_accel_set_sample_rate(g_accel, 800);

	#if STAR_ACCEL_NOT_IOCTL //if implement ioctl, following code must be deleted.
	INIT_DELAYED_WORK(&g_accel->delayed_work_accel, star_accel_delayed_wq_func );
	schedule_delayed_work(&g_accel->delayed_work_accel, 100);
	#endif

	g_accel->input_dev = input_allocate_device();
	if(!g_accel->input_dev)
	{
		printk("[skhwang][%s] fail to allocate a input device\n", __func__ );
		err = -ENOMEM;
		goto err_input_alloc_dev;
	}

	#if STAR_ACCEL_NOT_IOCTL //if implement ioctl, following code must be deleted.
	set_bit(EV_SYN, g_accel->input_dev->evbit);
	set_bit(EV_KEY, g_accel->input_dev->evbit);
	set_bit(EV_ABS, g_accel->input_dev->evbit);
	
	input_set_abs_params( g_accel->input_dev, ABS_X, g_accel->min_data.x,
						g_accel->max_data.x, 0, 0 );
	input_set_abs_params( g_accel->input_dev, ABS_Y, g_accel->min_data.y,
						g_accel->max_data.y, 0, 0 );
	input_set_abs_params( g_accel->input_dev, ABS_Z, g_accel->min_data.z,
						g_accel->max_data.z, 0, 0 );
	#endif
	platform_set_drvdata(pdev, g_accel); //?????

	g_accel->input_dev->name = "Nvodm_accelerometer";
	err = input_register_device(g_accel->input_dev);
	if(err)
	{
		printk("[%s] error to register input device\n",__func__);	
		goto input_register_device_failed;
	}

	//for ioctl
	if( misc_register(&star_accel_misc_device))
	{
		printk("[star accel, KXTF9] failed to register misc device\n");	
		err = -ENOSYS;
		goto err_misc_register;
	}
	return 0;

err_misc_register:
input_register_device_failed:
	input_free_device(g_accel->input_dev);
err_input_alloc_dev:
err_open_i2c:
	NvOdmOsFree(g_accel->h_gen2_i2c);
err_acquire_pin:
	NvOdmOsFree(g_accel->h_accel_gpio_pin);
err_open_gpio:
	NvOdmOsFree(g_accel->h_accel_gpio);
failtomemorydev:
	g_accel = 0;
	return err;

}

static int star_accel_remove( struct platform_device *pdev )
{
	return 0;
}

int star_accel_suspend(struct platform_device *dev, pm_message_t state)
{
	star_accel_set_power_rail(g_accel->vdd_id, NV_FALSE);

	return 0;
}

int star_accel_resume(struct platform_device *dev)
{
	star_accel_set_power_rail(g_accel->vdd_id, NV_TRUE);

	return 0;
}

static struct platform_driver star_accel_driver = {
	.probe = star_accel_probe,
	.remove = star_accel_remove,
	.suspend = star_accel_suspend,
	.resume = star_accel_resume,
	.driver = {
		.name = "star_accel_driver_name",
	},
};

static int __init star_accel_init(void)
{
	return platform_driver_register(&star_accel_driver);
}

static void __exit star_accel_exit(void)
{
	platform_driver_unregister(&star_accel_driver);
}

module_init(star_accel_init);
module_exit(star_accel_exit);

MODULE_AUTHOR("sk.hwang@lge.com");
MODULE_DESCRIPTION("driver of star accelerometer sensor");
MODULE_LICENSE("GPL");
