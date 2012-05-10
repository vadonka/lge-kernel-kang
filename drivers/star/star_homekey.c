/*
 * star(LGE_SU660) home key
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Hyeongwon Oh <hyeongwon.oh@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/**
	@brief		 star(LGE_SU660) home key
 
	@author		 hyeongwon.oh@lge.com
	@date		 2010-11-29
 
	@version	 V1.00		 2010.11.29		 Hyeongwon Oh	 Create
*/

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <linux/input.h>
#include <linux/wakelock.h>  

#include "mach/nvrm_linux.h"
#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#include <mach/iomap.h>
#include <linux/io.h>

#define PMC_WAKE_STATUS 0x14
#define WAKEUP_HOMEKEY_MASK    (1 << 14)     // Wake Event 14 - HOMEKEY

typedef struct HomeKeyDeviceRec
{
    NvOdmServicesGpioHandle gpioHandle;
    NvOdmGpioPinHandle  pinHandle;
    NvOdmServicesGpioIntrHandle intr;
    struct input_dev    *inputDev;
    struct wake_lock wlock;
    struct delayed_work  work;
    NvU32    kill;
} HomeKeyDevice;

static HomeKeyDevice s_homekey;

static int homekey_wakeup_ISR = 0;

static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);

static void homekey_interrupt_handler(void* arg);

static void homekey_handle(struct work_struct *wq)
{
    NvU32   pinValue;
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear home key wakeup pad bit.
    // Because homekey interrupt might be called before homekey_resume() is called.
    // In this case, clear bit not to call home key press at homekey_resume() function.
    if( homekey_wakeup_ISR == 0 )
    {
        if( reg & WAKEUP_HOMEKEY_MASK){
            printk("[HOME_KEY] wakeup pad clear\n");
            //clear wakeuppad status
            writel(WAKEUP_HOMEKEY_MASK, pmc_base + PMC_WAKE_STATUS);
        }
        printk("[HOME_KEY] wakeup by homekey HOME_KEY press\n");
        input_report_key(s_homekey.inputDev, KEY_HOME, 1);
        input_sync(s_homekey.inputDev);
        
        NvOdmGpioGetState(s_homekey.gpioHandle, s_homekey.pinHandle, &pinValue);
        if(pinValue){
            printk("[HOME_KEY] wakeup by homekey HOME_KEY release\n");
            input_report_key(s_homekey.inputDev, KEY_HOME, 0);
            input_sync(s_homekey.inputDev);
        }
        homekey_wakeup_ISR = 1;
        NvOdmGpioInterruptDone(s_homekey.intr);
        return;
    }
	
    if(!s_homekey.gpioHandle || !s_homekey.pinHandle || !s_homekey.inputDev)
        printk("HOME_KEY handler error\n");
    
    NvOdmGpioGetState(s_homekey.gpioHandle, s_homekey.pinHandle, &pinValue);
    
    if(pinValue){
        printk("HOME_KEY release\n");
        input_report_key(s_homekey.inputDev, KEY_HOME, 0);
    }else{
        printk("HOME_KEY press\n");
        input_report_key(s_homekey.inputDev, KEY_HOME, 1);
    }
    input_sync(s_homekey.inputDev);
    NvOdmGpioInterruptDone(s_homekey.intr);
}

static void homekey_interrupt_handler(void* arg)
{
    HomeKeyDevice *homeKeyDevice = (HomeKeyDevice *)arg;
    
    printk("[star Homekey] homekey_interrupt_handler\n");

    schedule_delayed_work(&homeKeyDevice->work, msecs_to_jiffies(20));
    wake_lock_timeout(&s_homekey.wlock, msecs_to_jiffies(50));
}

static int __init homekey_probe(struct platform_device *pdev)
{
    int ret;
    NvU32 pin, port;
    const NvOdmPeripheralConnectivity *con = NULL;

    memset(&s_homekey, 0x00, sizeof(s_homekey));

    //get query
    con = NvOdmPeripheralGetGuid(NV_ODM_GUID('h','o','m','e','-','k','e','y'));
    if(!con){
        printk(KERN_ERR "[star homekey] ODM GUID Error \n");
        goto err_probe_fail;
    }

    if ( con->AddressList[0].Interface == NvOdmIoModule_Gpio){
        port = con->AddressList[0].Instance;
        pin = con->AddressList[0].Address;
    }else{
        printk(KERN_ERR "[star homekey] cannot find ODM GUID \n");
        goto err_probe_fail;
    }

    INIT_DELAYED_WORK(&s_homekey.work, homekey_handle);
    wake_lock_init(&s_homekey.wlock, WAKE_LOCK_SUSPEND, "homekey_delay");    

    //GPIO configuration
    s_homekey.gpioHandle = NvOdmGpioOpen();
    if (!s_homekey.gpioHandle)
    {
        printk(KERN_ERR "[star homekey] NvOdmGpioOpen Error \n");
        goto err_open_gpio_fail;
    }
    s_homekey.pinHandle = NvOdmGpioAcquirePinHandle(s_homekey.gpioHandle, 
                                                    port, pin);
    if (!s_homekey.pinHandle)
    {
        printk(KERN_ERR "[star homekey] NvOdmGpioAcquirePinHandle Error\n");
        goto err_gpio_pin_acquire_fail;
    }
    //NvOdmGpioSetState(s_homekey.gpioHandle, s_homekey.pinHandle, 0);
    NvOdmGpioConfig(s_homekey.gpioHandle, s_homekey.pinHandle, 
                    NvOdmGpioPinMode_InputData);

    //GPIO interrupt registration
    if (NvOdmGpioInterruptRegister(s_homekey.gpioHandle, &s_homekey.intr,
            s_homekey.pinHandle, NvOdmGpioPinMode_InputInterruptAny, 
            homekey_interrupt_handler, (void*)&s_homekey, 0) == NV_FALSE)
    {
        printk(KERN_ERR "[star Homekey] interrupt registeration fail!\n");
        goto err_interrupt_register_fail;
    }

    // input device
    s_homekey.inputDev = input_allocate_device();
    if (!s_homekey.inputDev) {
        printk(KERN_ERR "[star Homekey] input_allocate_device Error!\n");
        goto err_input_device_allocation_fail;
    }
    s_homekey.inputDev->name = "homekey";
    //s_homekey.inputDev->id.bustype = BUS_HOST;
    s_homekey.inputDev->evbit[0] = BIT(EV_KEY);
    set_bit(KEY_HOME, s_homekey.inputDev->keybit);

    ret = input_register_device(s_homekey.inputDev);
    if (ret) {
        printk(KERN_ERR "[star homekey] input_register_device Error\n");
        goto err_input_device_register_fail;
    }

	printk("[star Homekey] homekey driver prove is successful!");
    return 0;
err_input_device_register_fail: 
    input_free_device(s_homekey.inputDev);    
err_input_device_allocation_fail: 
    NvOdmGpioInterruptUnregister(s_homekey.gpioHandle, s_homekey.pinHandle,
        s_homekey.intr);    
err_interrupt_register_fail:  
    NvOdmGpioReleasePinHandle(s_homekey.gpioHandle, s_homekey.pinHandle);
err_gpio_pin_acquire_fail:
    NvOdmGpioClose(s_homekey.gpioHandle);  

err_open_gpio_fail:
  
err_probe_fail:

    return -ENOSYS;
}

static int homekey_remove(struct platform_device *pdev)
{
    input_unregister_device(s_homekey.inputDev);
    input_free_device(s_homekey.inputDev);  
    NvOdmGpioInterruptUnregister(s_homekey.gpioHandle, s_homekey.pinHandle,
        s_homekey.intr);
    
    NvOdmGpioReleasePinHandle(s_homekey.gpioHandle, s_homekey.pinHandle);
    NvOdmGpioClose(s_homekey.gpioHandle);

    wake_lock_destroy(&s_homekey.wlock);
	
    return 0;
}

int homekey_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned long reg;
    
    reg = readl(pmc_base + PMC_WAKE_STATUS);

    // Clear home key wakeup pad bit.
    if (reg & WAKEUP_HOMEKEY_MASK)
    {
        writel(WAKEUP_HOMEKEY_MASK, pmc_base + PMC_WAKE_STATUS);
    }

    homekey_wakeup_ISR = 0;

    return 0;
}

int homekey_resume(struct platform_device *dev)
{
    NvU32   pinValue;
    unsigned long reg;

    reg = readl(pmc_base + PMC_WAKE_STATUS);

    if (homekey_wakeup_ISR == 0)
    {
        if (reg & WAKEUP_HOMEKEY_MASK) {
            //clear wakeuppad status
            writel(WAKEUP_HOMEKEY_MASK, pmc_base + PMC_WAKE_STATUS);
            printk("[HOME_KEY] resume func HOMEKEY press\n");
            input_report_key(s_homekey.inputDev, KEY_HOME, 1);
            input_sync(s_homekey.inputDev);

            NvOdmGpioGetState(s_homekey.gpioHandle, s_homekey.pinHandle, &pinValue);
            if(pinValue){
                printk("[HOME_KEY] resume func HOMEKEY release\n");
                input_report_key(s_homekey.inputDev, KEY_HOME, 0);
                input_sync(s_homekey.inputDev);
            }
        }
        homekey_wakeup_ISR = 1;
    }

    return 0;
}


static struct platform_driver homekey_driver = {
    .probe      = homekey_probe,
    .remove     = homekey_remove,
    .suspend    = homekey_suspend,
    .resume     = homekey_resume,
    .driver = {
        .name   = "star_homekey",
        .owner  = THIS_MODULE,
    },
};

static int __init homekey_init(void)
{
    return platform_driver_register(&homekey_driver);
}

static void __exit homekey_exit(void)
{
    platform_driver_unregister(&homekey_driver);
}

module_init(homekey_init);
module_exit(homekey_exit);

MODULE_AUTHOR("hyeongwon.oh@lge.com");
MODULE_DESCRIPTION("star home key");
MODULE_LICENSE("GPL");

