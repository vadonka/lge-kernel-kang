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

#ifndef NVODM_PMU_TPS6586X_H_HH
#define NVODM_PMU_TPS6586X_H_HH

#include "nvodm_pmu.h"
#include "nvodm_services.h"
#include "nvodm_pmu_tps6586x_supply_info_table.h"
#include "tps6586x_reg.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#if (NV_DEBUG)
#define NVODMPMU_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMPMU_PRINTF(x)
#endif

typedef struct NvOdmPmuDeviceTPSRec
{
    /* The handle to the I2C */
    NvOdmServicesI2cHandle hOdmI2C;

    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;

    /* Gpio Handles (for external supplies) */
    NvOdmServicesGpioHandle hGpio;
    NvOdmGpioPinHandle hPin[TPS6586x_EXTERNAL_SUPPLY_AP_GPIO_NUM];

    /* the PMU I2C device Address */
    NvU32 DeviceAddr;

    /* Device's private data */
    void *priv;

    /* The ref cnt table of the power supplies */
    NvU32 supplyRefCntTable[TPS6586xPmuSupply_Num];

} NvOdmPmuDeviceTPS;

void Tps6586xGetCapabilities( NvU32 vddRail, NvOdmPmuVddRailCapabilities* pCapabilities);
NvBool Tps6586xGetVoltage( NvOdmPmuDeviceHandle hDevice, NvU32 vddRail, NvU32* pMilliVolts);
NvBool Tps6586xSetVoltage( NvOdmPmuDeviceHandle hDevice, NvU32 vddRail, NvU32 MilliVolts, NvU32* pSettleMicroSeconds);
NvBool Tps6586xSetup(NvOdmPmuDeviceHandle hDevice);
void Tps6586xRelease(NvOdmPmuDeviceHandle hDevice);
NvBool Tps6586xGetAcLineStatus( NvOdmPmuDeviceHandle hDevice, NvOdmPmuAcLineStatus *pStatus);
NvBool Tps6586xGetBatteryStatus( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvU8 *pStatus);
NvBool Tps6586xGetBatteryData( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvOdmPmuBatteryData *pData); 
void Tps6586xGetBatteryFullLifeTime( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvU32 *pLifeTime);
void Tps6586xGetBatteryChemistry( NvOdmPmuDeviceHandle hDevice, NvOdmPmuBatteryInstance batteryInst, NvOdmPmuBatteryChemistry *pChemistry);
NvBool Tps6586xSetChargingCurrent( NvOdmPmuDeviceHandle hDevice, NvOdmPmuChargingPath chargingPath, NvU32 chargingCurrentLimitMa, NvOdmUsbChargerType ChargerType); 
void Tps6586xInterruptHandler( NvOdmPmuDeviceHandle  hDevice);
NvBool Tps6586xReadRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 *Count);
NvBool Tps6586xWriteRtc( NvOdmPmuDeviceHandle  hDevice, NvU32 Count);
NvBool Tps6586xReadAlarm( NvOdmPmuDeviceHandle  hDevice, NvU32 *Count);
NvBool Tps6586xWriteAlarm( NvOdmPmuDeviceHandle  hDevice, NvU32 Count);
NvBool Tps6586xIsRtcInitialized( NvOdmPmuDeviceHandle  hDevice);

#if defined(__cplusplus)
}
#endif

#endif /* NVODM_PMU_TPS6586X_H_HH */
