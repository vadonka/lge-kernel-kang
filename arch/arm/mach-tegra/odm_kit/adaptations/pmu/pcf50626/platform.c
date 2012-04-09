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

#include "pcf50626_batterycharger.h"
#include "pcf50626_adc.h"
#include "pcf50626_i2c.h"
#include "pcf50626_reg.h"
#include "ds2482_bridge.h"
#include "ds2482_i2c.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query.h"

// implementations of platform related functions.

#define OWI2CGUID   NV_ODM_GUID('o','w','i','2','c','_','d','s')
#define BATTERYGUID NV_ODM_GUID('b','a','t','t','_','m','o','t')

static NvBool ds2482Presented = NV_FALSE;

/* Initliase all registers that related to battery charger */
NvBool 
Pcf50626BatteryChargerSetup(NvOdmPmuDeviceHandle hDevice)
{
    
    NvOdmIoModule I2cModule = NvOdmIoModule_I2c;
    NvU32  I2cInstance = 0;
    NvU32  I2cAddress  = 0;
    const NvOdmPeripheralConnectivity *pConnectivity = 
                           NvOdmPeripheralGetGuid(OWI2CGUID);    
    NV_ASSERT(hDevice);

    if (pConnectivity != NULL) // OwI2c device is in database
    {
        NvU32 i = 0;        
        for (i = 0; i < pConnectivity->NumAddress; i ++)
        {
            if (pConnectivity->AddressList[i].Interface == NvOdmIoModule_I2c_Pmu)
            {
                I2cModule   = NvOdmIoModule_I2c_Pmu;
                I2cInstance = pConnectivity->AddressList[i].Instance;
                I2cAddress  = pConnectivity->AddressList[i].Address;
                break;
            }
        }
        ds2482Presented = NV_TRUE;
    }
    else
    {
        ds2482Presented = NV_FALSE;
    }

    if (ds2482Presented == NV_TRUE)
    {
        // init GPIO8
        if (!Pcf50626I2cWrite8(hDevice, PCF50626_GPIO8C1_ADDR, 0x0))
            return NV_FALSE;

        NvOdmOsWaitUS(50000);
        
        if (!Ds2482Setup(hDevice))
            return NV_FALSE;

        return NV_TRUE;
    }

    //TODO: add battery charger setup implementation here, base on HW.
    // by default, simply return TRUE if nothing needed.
    return NV_TRUE;
}


/* check CBC main batt presence */
NvBool
Pcf50626BatteryChargerCBCMainBatt(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    const NvOdmPeripheralConnectivity *pConnectivity = 
                           NvOdmPeripheralGetGuid(BATTERYGUID);
    
    NV_ASSERT(hDevice);

    if (pConnectivity != NULL) // battery is in database
    {
        ///TODO: retrieve data from the database;
        //Nothing needed on Concorde for now
    }

    if (ds2482Presented == NV_TRUE)
    {
        if (!Ds2482BatteryPresented(hDevice, status))
           return NV_FALSE;
    }
    else
    {
        //TODO: add battery detection impelentation here. by default always return TRUE.  
        *status = NV_TRUE;
    }
    
    return NV_TRUE;
}

/* Calculate the battery temperature */
NvU32 Pcf50626BatteryTemperature(NvU32 VBatSense, NvU32 VBatTemp)
{
    //TODO: implement the temperature algorithm based on the reading of Vbat sense and VBat temp.
    // Pending approval.
    return 0;
}


NvBool
Ds2482BatteryPresented(NvOdmPmuDeviceHandle hDevice, NvBool *BattPresence)
{
    ///TODO: detect battery via ds2482 based on the battery spec.
    ///Pending approval
    return NV_TRUE;
}



