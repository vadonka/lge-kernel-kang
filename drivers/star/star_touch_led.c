/*
 * star(lgp990) touch LED
 *
 * Copyright (C) 2009 LGE, Inc.
 *
 * Author: Changsu Ha <cs77.ha@lge.com>
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
	@brief		 star(lgp990) touch LED
 
	@author		 cs77.ha@lge.com
	@date		 2010-06-11
 
	@version	 V1.00		 2010.06.11		 Changsu Ha	 Create
*/
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>

#include "mach/nvrm_linux.h"
#include "nvcommon.h"
#include "nvos.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"


#define TOUCH_LED_TIMER


#ifdef TOUCH_LED_TIMER
#define BOOT_DELAY_SEC      30      /* second */
#define TOUCH_DELAY_SEC     10//5       /* second */
#endif

typedef struct TouchLEDRec{
    NvOdmServicesPmuHandle hPmu;
    NvOdmPeripheralConnectivity const *conn;
    struct work_struct  work;

#ifdef TOUCH_LED_TIMER
    struct hrtimer  timer;
    long            delay;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend	early_suspend;
#endif
    NvU8            setVal;
    NvU8            maxVal;

	NvU8			is_working;
    NvU8            keep_led_on;
} TouchLED;

static TouchLED s_touchLED;

static NvBool touchLED_Control(NvU8 value)
{
	NvU32 settle_us;

	/* set the rail volatage to the recommended */
	if(value)
	{
// 20100820 joseph.jung@lge.com LGE Touch LED Control [START]
#ifdef TOUCH_LED_TIMER
		hrtimer_cancel(&s_touchLED.timer);
		hrtimer_start(&s_touchLED.timer, ktime_set(s_touchLED.delay, 0), HRTIMER_MODE_REL);
#endif
// 20100820 joseph.jung@lge.com LGE Touch LED Control [END]
        NvOdmServicesPmuSetVoltage(s_touchLED.hPmu, s_touchLED.conn->AddressList[0].Address, s_touchLED.setVal, &settle_us);
	}
	else
	{
// 20100820 joseph.jung@lge.com LGE Touch LED Control [START]
#ifdef TOUCH_LED_TIMER
		hrtimer_cancel(&s_touchLED.timer);
#endif
// 20100820 joseph.jung@lge.com LGE Touch LED Control [END]
		if ( s_touchLED.keep_led_on != 1 )
		NvOdmServicesPmuSetVoltage(s_touchLED.hPmu, s_touchLED.conn->AddressList[0].Address, NVODM_VOLTAGE_OFF, &settle_us);
	}
	
	return NV_TRUE;
}

static void touchLED_timeout(struct work_struct *wq)
{
	touchLED_Control(NV_FALSE);
}

static enum hrtimer_restart touchLED_timer_func(struct hrtimer *timer)
{
    schedule_work(&s_touchLED.work);

    return HRTIMER_NORESTART;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touchLED_early_suspend(struct early_suspend *es)
{
	printk("[LED] touchLED_early_suspend\n");
	s_touchLED.is_working = 0;
	touchLED_Control(NV_FALSE);
	return;
}

static void touchLED_late_resume(struct early_suspend *es)
{
	printk("[LED] touchLED_late_resume\n");
	s_touchLED.is_working = 1;
	touchLED_Control(NV_TRUE);
	return;
}
#else
static int touchLED_suspend(struct platform_device *pdev, pm_message_t state)
{
	s_touchLED.is_working = 0;
	touchLED_Control(NV_FALSE);
	return 0;
}

static int touchLED_resume(struct platform_device *pdev)
{
	s_touchLED.is_working = 1;
	touchLED_Control(NV_TRUE);
	return 0;
}
#endif

//20101104, cs77.ha@lge.com, WLED set [START]
static ssize_t star_wled_show(struct device *dev, 
            struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "wled : %duA\n", (int)s_touchLED.setVal*100);
}

static ssize_t star_wled_store(struct device *dev, 
            struct device_attribute *attr, char *buf, size_t count)
{
    NvU8 val = 0;

    val = (NvU8)simple_strtoul(buf, NULL, 10);

    // 0~100 (0.0mA~10.0mA)
    if(val > s_touchLED.maxVal)
        s_touchLED.setVal = s_touchLED.maxVal;
    else
        s_touchLED.setVal = val;
    
#ifdef TOUCH_LED_TIMER
    hrtimer_cancel(&s_touchLED.timer);
#endif
    NvOdmServicesPmuSetVoltage(s_touchLED.hPmu, s_touchLED.conn->AddressList[0].Address, s_touchLED.setVal, NULL);

    return sprintf(buf, "wled : %duA\n", (int)s_touchLED.setVal*100);
}

static DEVICE_ATTR(wled, 0666, star_wled_show, star_wled_store);

static struct attribute *star_wled_attributes[] = {
    &dev_attr_wled.attr,
    NULL
};

static const struct attribute_group star_wled_group = {
    .attrs = star_wled_attributes,
};
//20101104, cs77.ha@lge.com, WLED set [END]

void keep_touch_led_on()
{
	s_touchLED.keep_led_on = 1;
	touchLED_Control(NV_TRUE);
}

EXPORT_SYMBOL(keep_touch_led_on);

static int __init touchLED_probe(struct platform_device *pdev)
{
    s_touchLED.conn = NvOdmPeripheralGetGuid( NV_ODM_GUID('t','o','u','c','h','L','E','D') );

    /* enable the power rail */
    s_touchLED.hPmu = NvOdmServicesPmuOpen();
    if( s_touchLED.conn->AddressList[0].Interface == NvOdmIoModule_Vdd )
    {
        NvOdmServicesPmuVddRailCapabilities cap;
        NvU32 settle_us;

        /* address is the vdd rail id */
        NvOdmServicesPmuGetCapabilities( s_touchLED.hPmu,
            s_touchLED.conn->AddressList[0].Address, &cap );

        s_touchLED.setVal = cap.requestMilliVolts;
        s_touchLED.maxVal = 100;    /*10.0mA*/

        /* set the rail volatage to the recommended */
        NvOdmServicesPmuSetVoltage( s_touchLED.hPmu,
            s_touchLED.conn->AddressList[0].Address, cap.requestMilliVolts,
            &settle_us );

        /* wait for rail to settle */
        NvOdmOsWaitUS( settle_us );
    }
    INIT_WORK(&s_touchLED.work, touchLED_timeout);

#ifdef TOUCH_LED_TIMER 
    hrtimer_init(&s_touchLED.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    s_touchLED.timer.function = touchLED_timer_func;
    s_touchLED.delay = TOUCH_DELAY_SEC;
    hrtimer_start(&s_touchLED.timer, ktime_set(BOOT_DELAY_SEC, 0), HRTIMER_MODE_REL);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    s_touchLED.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
    s_touchLED.early_suspend.suspend = touchLED_early_suspend;
    s_touchLED.early_suspend.resume = touchLED_late_resume;
    register_early_suspend(&s_touchLED.early_suspend);
#endif

    s_touchLED.keep_led_on = 0;
    //20101104, cs77.ha@lge.com, WLED set [START]
    if (sysfs_create_group(&pdev->dev.kobj, &star_wled_group)) {
        printk(KERN_ERR "[star touch led] sysfs_create_group ERROR\n");
    }
    //20101104, cs77.ha@lge.com, WLED set [END]

	s_touchLED.is_working = 1;

    return 0;
}

static int touchLED_remove(struct platform_device *pdev)
{
    //20101104, cs77.ha@lge.com, WLED set [START]
    sysfs_remove_group(&pdev->dev.kobj, &star_wled_group);
    //20101104, cs77.ha@lge.com, WLED set [END]

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&s_touchLED.early_suspend);
#endif

    touchLED_Control(NV_FALSE);
    NvOdmServicesPmuClose( s_touchLED.hPmu );
	
	s_touchLED.is_working = 0;
    return 0;
}

static void touchLED_shutdown(struct  platform_device *pdev)
{
    //20101104, cs77.ha@lge.com, WLED set [START]
    sysfs_remove_group(&pdev->dev.kobj, &star_wled_group);
    //20101104, cs77.ha@lge.com, WLED set [END]

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&s_touchLED.early_suspend);
#endif

    touchLED_Control(NV_TRUE);	// remove off-command to keep LED on before PMIC power-off
    NvOdmServicesPmuClose( s_touchLED.hPmu );

	s_touchLED.is_working = 0;
}


static struct platform_driver touchLED_driver = {
    .probe      = touchLED_probe,
    .remove     = touchLED_remove,
    .shutdown	= touchLED_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = touchLED_suspend,
    .resume     = touchLED_resume,
#endif
    .driver = {
        .name   = "star_touch_led",
        .owner  = THIS_MODULE,
    },
};

// 20100820 joseph.jung@lge.com LGE Touch LED Control [START]
void touchLED_enable(NvBool status)
{
	if(s_touchLED.is_working)
	    touchLED_Control(status);
}
// 20100820 joseph.jung@lge.com LGE Touch LED Control [END]


EXPORT_SYMBOL_GPL(touchLED_enable);


static int __init touchLED_init(void)
{
    return platform_driver_register(&touchLED_driver);
}

static void __exit touchLED_exit(void)
{
    platform_driver_unregister(&touchLED_driver);
}

module_init(touchLED_init);
module_exit(touchLED_exit);

MODULE_AUTHOR("cs77.ha@lge.com");
MODULE_DESCRIPTION("star touch led");
MODULE_LICENSE("GPL");

