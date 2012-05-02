/*
 * Copyright (c) 2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/time.h>
#include <linux/rtc.h>
#include "max8907.h"
#include "max8907_rtc.h"
#include "max8907_i2c.h"
#include "max8907_reg.h"

/**
* The Maxim 8907C does not have an RTC that simply counts
* seconds from some time t0 (as defined by the OS API).
* Instead, this RTC contains several BCD (Binary Coded Decimal)
* registers, including: seconds, minutes, hours, days, day of
* week, date, etc...  These registers account for leap year and
* the various days of the month as well.
*
* Since the OS interpretation of seconds to a particular
* date/time from some OS-defined t0 is unknown at this level of
* the implementation, it is not possible to translate the given
* seconds into these registers (at least, not without a
* dependency on some OS-specific information).
*
*/

#define MAX8907_SECONDS_PER_DAY    (60*60*24)
#define MAX8907_SECONDS_PER_HOUR   (60*60)
#define MAX8907_SECONDS_PER_MINUTE (60)

#define LINUX_RTC_BASE_YEAR 1900

/* Macro for conversion of BCD number to decimal format */
#define BCD_TO_DECIMAL(BCD) \
            ((((BCD) & 0xF0) >> 4) * 10 + ((BCD) & 0xF))
/* Macro for conversion of decimal number to BCD format */
#define DECIMAL_TO_BCD(DEC) \
            ((((DEC) / 10) << 4) | ((DEC) % 10))

static NvBool bRtcNotInitialized = NV_TRUE;

NvBool
Max8907RtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    NvU32 data = 0;
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 Hours, Minutes, Seconds;
    NvU32 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU32 DD, MM, YY1, YY2, YYYY;
#if NV_DEBUG
    struct rtc_time tm;
#endif

    *Count = 0;
    // Read seconds, minute, hour and weekday data from RTC registers
    if (Max8907RtcI2cReadTime(hDevice, MAX8907_RTC_SEC, &data))
    {
        NVODMPMU_PRINTF(("\n Read time data-sec=0x%x ", data));
        // Extract seconds, minute and hour data from RTC registers read
        BcdHours   = (data >>  8) & 0xFF;
        BcdMinutes = (data >> 16) & 0xFF;
        BcdSeconds = (data >> 24) & 0xFF;

        // Convert BCD time into decimal values
        Hours   = BCD_TO_DECIMAL(BcdHours);
        Minutes = BCD_TO_DECIMAL(BcdMinutes);
        Seconds = BCD_TO_DECIMAL(BcdSeconds);

        // Read day, month, yy1 and yy2 data from RTC registers
        if (Max8907RtcI2cReadTime(hDevice, MAX8907_RTC_DATE, &data))
        {
            NVODMPMU_PRINTF(("\n Read time data-year=0x%x ", data));
            // Extract day, month, yy1 and yy2 data from RTC registers read
            BcdYY2   = (data & 0xFF);
            BcdYY1   = (data >>  8) & 0xFF;
            BcdMM = (data >> 16) & 0xFF;
            BcdDD = (data >> 24) & 0xFF;
            // convert bcd day/month/year data to decimal values
            YY2 = BCD_TO_DECIMAL(BcdYY2);
            YY1 = BCD_TO_DECIMAL(BcdYY1);
            YYYY = (YY2 * 100 + YY1) & 0xFFFF;
            MM = BCD_TO_DECIMAL(BcdMM);
            DD = BCD_TO_DECIMAL(BcdDD);
            // get seconds since reference time value given
            // year, month, day, hour, minutes and seconds
            // NOTE: Using linux specific API mktime for conversion
            *Count = mktime(YYYY, (MM), DD, Hours, Minutes, Seconds);
            NVODMPMU_PRINTF(("\n Rtc read count=0x%x ", *Count));
            NVODMPMU_PRINTF(("\n mktime: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
                "Sec=%d, *Count=0x%x ", YYYY, (MM), DD, Hours, Minutes,
                Seconds, *Count));
#if NV_DEBUG
            // Call to verify that reverse conversion of seconds matches date
            rtc_time_to_tm(*Count, &tm);
            // Check if Local_rtc_time_to_tm can return values sent to mktime
            NVODMPMU_PRINTF(("\n rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d "
                "Min=%d Sec=%d, *Count=0x%x ", (tm.tm_year +
                LINUX_RTC_BASE_YEAR), tm.tm_mon, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, *Count));
#endif
        }
        else
        {
            NVODMPMU_PRINTF(("\n Max8907RtcCountRead() error. "));
            return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907RtcCountRead() error. "));
        return NV_FALSE;
    }
    NVODMPMU_PRINTF(("\n *Count=0x%x ", *Count));
    return NV_TRUE;
}

NvBool
Max8907RtcAlarmCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
    NvU32 data = 0;
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 Hours, Minutes, Seconds;
    NvU32 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU32 DD, MM, YY1, YY2, YYYY;
#if NV_DEBUG
    struct rtc_time tm;
#endif
    NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcAlarmCountRead()  "));
    *Count = 0;
    // Read seconds, minute, hour and weekday data from RTC registers
    if (Max8907RtcI2cReadTime(hDevice, MAX8907_ALARM0_SEC, &data))
    {
        NVODMPMU_PRINTF(("\n [Alarm] Read time data-sec=0x%x ", data));
        // Extract seconds, minute and hour data from RTC registers read
        BcdHours   = (data >>  8) & 0xFF;
        BcdMinutes = (data >> 16) & 0xFF;
        BcdSeconds = (data >> 24) & 0xFF;

        // Convert BCD time into decimal values
        Hours   = BCD_TO_DECIMAL(BcdHours);
        Minutes = BCD_TO_DECIMAL(BcdMinutes);
        Seconds = BCD_TO_DECIMAL(BcdSeconds);

        // Read day, month, yy1 and yy2 data from RTC registers
        if (Max8907RtcI2cReadTime(hDevice, MAX8907_ALARM0_DATE, &data))
        {
            NVODMPMU_PRINTF(("\n [Alarm] Read time data-year=0x%x ", data));
            // Extract day, month, yy1 and yy2 data from RTC registers read
            BcdYY2   = (data & 0xFF);
            BcdYY1   = (data >>  8) & 0xFF;
            BcdMM = (data >> 16) & 0xFF;
            BcdDD = (data >> 24) & 0xFF;
            // convert bcd day/month/year data to decimal values
            YY2 = BCD_TO_DECIMAL(BcdYY2);
            YY1 = BCD_TO_DECIMAL(BcdYY1);
            YYYY = (YY2 * 100 + YY1) & 0xFFFF;
            MM = BCD_TO_DECIMAL(BcdMM);
            DD = BCD_TO_DECIMAL(BcdDD);
            // get seconds since reference time value given
            // year, month, day, hour, minutes and seconds
            // NOTE: Using linux specific API mktime for conversion
            *Count = mktime(YYYY, (MM), DD, Hours, Minutes, Seconds);
            NVODMPMU_PRINTF(("\n [Alarm] Rtc read count=0x%x ", *Count));
            NVODMPMU_PRINTF(("\n [Alarm] mktime: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
                "Sec=%d, *Count=0x%x ", YYYY, (MM), DD, Hours, Minutes,
                Seconds, *Count));
#if NV_DEBUG
            // Call to verify that reverse conversion of seconds matches date
            rtc_time_to_tm(*Count, &tm);
            // Check if Local_rtc_time_to_tm can return values sent to mktime
            NVODMPMU_PRINTF(("\n [Alarm] rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d "
                "Min=%d Sec=%d, *Count=0x%x ", (tm.tm_year +
                LINUX_RTC_BASE_YEAR), tm.tm_mon, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec, *Count));
#endif
        }
        else
        {
            NVODMPMU_PRINTF(("\n Max8907RtcCountRead() error. "));
            return NV_FALSE;
        }
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907RtcCountRead() error. "));
        return NV_FALSE;
    }
    NVODMPMU_PRINTF(("\n *Count=0x%x ", *Count));
    return NV_TRUE;
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]
	
}

NvBool
Max8907RtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 data = 0;
    NvU8 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU16 YYYY;
    struct rtc_time tm;
#if NV_DEBUG
    NvU32 data1;
#endif

    NVODMPMU_PRINTF(("\n Rtc write count=0x%x ", Count));
    // convert seconds since reference time into date
    // NOTE: using linux specific convert function rtc_time_to_tm
    rtc_time_to_tm(Count, &tm);
    NVODMPMU_PRINTF(("\n rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
        "Sec=%d, *Count=0x%x ", (tm.tm_year + LINUX_RTC_BASE_YEAR),
        (tm.tm_mon + 1), tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec, Count));

    // Convert time to bcd format
    BcdHours   = DECIMAL_TO_BCD(tm.tm_hour);
    BcdMinutes = DECIMAL_TO_BCD(tm.tm_min);
    BcdSeconds = DECIMAL_TO_BCD(tm.tm_sec);

    data = (BcdSeconds << 24) | (BcdMinutes << 16) | (BcdHours << 8);
    // write time - seconds, minutes and hours in a day to RTC registers
    if (Max8907RtcI2cWriteTime(hDevice, MAX8907_RTC_SEC, data))
    {
        // set the day, month, year
        // Assuming we get the days since 1 Jan 1970

        // convert date to bcd format
        BcdDD = DECIMAL_TO_BCD((NvU8)tm.tm_mday);
        BcdMM = DECIMAL_TO_BCD((NvU8)tm.tm_mon+1);
        YYYY = (NvU16)tm.tm_year + LINUX_RTC_BASE_YEAR;
        BcdYY1 = DECIMAL_TO_BCD((NvU8)(YYYY % 100));
        BcdYY2 = DECIMAL_TO_BCD((NvU8)(YYYY / 100));
        data = (NvU32)((BcdDD << 24) | (BcdMM << 16) | (BcdYY1 << 8) | BcdYY2);
        // write date - day, month, and year to RTC registers
        if (!(Max8907RtcI2cWriteTime(hDevice, MAX8907_RTC_DATE, data)))
        {
            NVODMPMU_PRINTF(("\n Max8907RtcCountWrite() error. "));
            return NV_FALSE;
        }
#if NV_DEBUG
        // verify that read back values from RTC matches written values
        if (!(Max8907RtcI2cReadTime(hDevice, MAX8907_RTC_DATE, &data1)))
        {
            NVODMPMU_PRINTF(("\n Max8907RtcCountRead() error. "));
            return NV_FALSE;
        }
        if (data1 == data)
        {
            NVODMPMU_PRINTF(("\n Write read Success. "));
            return NV_TRUE;
        }
        else
        {
            // return error when read data does not match written data
            NVODMPMU_PRINTF(("\n Error: write data=0x%x, rd data=0x%x. ", data, data1));
            return NV_FALSE;
        }
#endif
    }
    else
    {
        NVODMPMU_PRINTF(("\n Max8907RtcCountWrite() error. "));
        return NV_FALSE;
    }

    return NV_TRUE;
}

NvBool
Max8907RtcAlarmCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
    NvU32 BcdHours, BcdMinutes, BcdSeconds;
    NvU32 data = 0;
    NvU8 BcdDD, BcdMM, BcdYY1, BcdYY2;
    NvU16 YYYY;
    struct rtc_time tm;
#if NV_DEBUG
    NvU32 data1;
#endif

    NVODMPMU_PRINTF(("\n [Alarm] Rtc write count=0x%x ", Count));
    // convert seconds since reference time into date
    // NOTE: using linux specific convert function rtc_time_to_tm
    rtc_time_to_tm(Count, &tm);
    NVODMPMU_PRINTF(("\n [Alarm] rtc_time_to_tm: YYYY=%d MM=%d DD=%d Hr=%d Min=%d "
        "Sec=%d, *Count=0x%x ", (tm.tm_year + LINUX_RTC_BASE_YEAR),
        (tm.tm_mon + 1), tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec, Count));

    // Convert time to bcd format
    BcdHours   = DECIMAL_TO_BCD(tm.tm_hour);
    BcdMinutes = DECIMAL_TO_BCD(tm.tm_min);
    BcdSeconds = DECIMAL_TO_BCD(tm.tm_sec);

    data = (BcdSeconds << 24) | (BcdMinutes << 16) | (BcdHours << 8);
    // write time - seconds, minutes and hours in a day to RTC registers
    if (Max8907RtcI2cWriteTime(hDevice, MAX8907_ALARM0_SEC, data))
    {
        // set the day, month, year
        // Assuming we get the days since 1 Jan 1970

        // convert date to bcd format
        BcdDD = DECIMAL_TO_BCD((NvU8)tm.tm_mday);
        BcdMM = DECIMAL_TO_BCD((NvU8)tm.tm_mon+1);
        YYYY = (NvU16)tm.tm_year + LINUX_RTC_BASE_YEAR;
        BcdYY1 = DECIMAL_TO_BCD((NvU8)(YYYY % 100));
        BcdYY2 = DECIMAL_TO_BCD((NvU8)(YYYY / 100));
        data = (NvU32)((BcdDD << 24) | (BcdMM << 16) | (BcdYY1 << 8) | BcdYY2);
        // write date - day, month, and year to RTC registers
        if (!(Max8907RtcI2cWriteTime(hDevice, MAX8907_ALARM0_DATE, data)))
        {
            NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcCountWrite() error. "));
            return NV_FALSE;
        }
#if NV_DEBUG
        // verify that read back values from RTC matches written values
        if (!(Max8907RtcI2cReadTime(hDevice, MAX8907_ALARM0_DATE, &data1)))
        {
            NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcCountRead() error. "));
            return NV_FALSE;
        }
        if (data1 == data)
        {
            NVODMPMU_PRINTF(("\n [Alarm] Write read Success. "));
            Max8907RtcAlarmIntEnable(hDevice, NV_TRUE);
            return NV_TRUE;
        }
        else
        {
            // return error when read data does not match written data
            NVODMPMU_PRINTF(("\n [Alarm] Error: write data=0x%x, rd data=0x%x. ", data, data1));
            return NV_FALSE;
        }
#endif
        Max8907RtcAlarmIntEnable(hDevice, NV_TRUE);
    }
    else
    {
        NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcCountWrite() error. "));
        return NV_FALSE;
    }
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]

    return NV_TRUE;
}

NvBool
Max8907RtcIsAlarmIntEnabled(NvOdmPmuDeviceHandle hDevice)
{
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
    NvU8 data = 0;
    NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcIsAlarmIntEnabled()  "));
    if (Max8907RtcI2cRead8(hDevice, MAX8907_ALARM0_CNTL, &data))
    {
        NVODMPMU_PRINTF(("\n [Alarm] Max8907I2cRead8() success : data = 0x%x ", data));
        if( data == MAX8907_ALARM0_CNTL_ENABLE )
        {
            NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcIsAlarmInt Enabled "));
            return NV_TRUE;
        }
        else
        {
            NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcIsAlarmInt disabled "));
            return NV_FALSE;
        }
    }

    NVODMPMU_PRINTF(("\n [Alarm] Max8907I2cRead32() error. "));
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]

    return NV_FALSE;
}

NvBool
Max8907RtcAlarmIntEnable(
    NvOdmPmuDeviceHandle hDevice,
    NvBool Enable)
{
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [START]
    NvU8 data = 0;

    NVODMPMU_PRINTF(("\n [Alarm] Max8907RtcAlarmIntEnable()  "));
    if( Enable )
        data=MAX8907_ALARM0_CNTL_ENABLE;		
	
    if (Max8907RtcI2cWrite8(hDevice, MAX8907_ALARM0_CNTL, data))
    {
        NVODMPMU_PRINTF(("\n [Alarm] Max8907I2cWrite32() success : data = 0x%x ", data));
        return NV_TRUE;
    }

    NVODMPMU_PRINTF(("\n [Alarm] Error !! : Max8907I2cWrite32()  "));
    //20100928, byoungwoo.yoon@lge.com, RTC alarm enable [END]

    return NV_FALSE;
}

NvBool
Max8907IsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    return (!bRtcNotInitialized);
}

