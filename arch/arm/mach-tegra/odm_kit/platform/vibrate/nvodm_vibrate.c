/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
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

#include "nvodm_vibrate.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_pmu.h"

#define VIBRATE_DEVICE_GUID NV_ODM_GUID('v','i','b','r','a','t','o','r')

/**
 * @brief Used to enable/disable debug print messages.
 */
#define NV_ODM_DEBUG 0

#if NV_ODM_DEBUG
    #define NV_ODM_TRACE NvOdmOsDebugPrintf
#else
    #define NV_ODM_TRACE (void)
#endif

typedef struct NvOdmVibDeviceRec
{
    /* The handle to the Pmu device */
    NvOdmServicesPmuHandle hOdmServicePmuDevice;

    /*Pmu Vdd Rail capabilities*/
    NvOdmServicesPmuVddRailCapabilities RailCaps;

    /* Pmu Rail ID*/
    NvU32  VddId;

} NvOdmVibDevice;


/**
 *  @brief Allocates a handle to the device. Configures the PWM
 *   control to the Vibro motor with default values. To change
 *   the amplitude and frequency use NvOdmVibrateSetParameter API.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return  NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibOpen(NvOdmVibDeviceHandle *hOdmVibrate)
{
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvU32 Index = 0;

    NV_ASSERT(hOdmVibrate);

    /* Allocate the handle */
    (*hOdmVibrate) = (NvOdmVibDeviceHandle)NvOdmOsAlloc(sizeof(NvOdmVibDevice));
    if (*hOdmVibrate == NULL)
    {
        NV_ODM_TRACE(("Error Allocating NvOdmPmuDevice. \n"));
        return NV_FALSE;
    }
    NvOsMemset((*hOdmVibrate), 0, sizeof(NvOdmVibDevice));

    /* Get the PMU handle */
    (*hOdmVibrate)->hOdmServicePmuDevice = NvOdmServicesPmuOpen();
    if (!(*hOdmVibrate)->hOdmServicePmuDevice)
    {
        NV_ODM_TRACE(("Error Opening Pmu device. \n"));
        NvOdmOsFree(*hOdmVibrate);
        *hOdmVibrate = NULL;
        return NV_FALSE;
    }

        // Get the peripheral connectivity information
    pConnectivity = NvOdmPeripheralGetGuid(VIBRATE_DEVICE_GUID);
    if (pConnectivity == NULL)
        return NV_FALSE;

        // Search for the Vdd rail and set the proper volage to the rail.
    for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
    {
        if (pConnectivity->AddressList[Index].Interface == NvOdmIoModule_Vdd)
        {
            (*hOdmVibrate)->VddId = pConnectivity->AddressList[Index].Address;
            NvOdmServicesPmuGetCapabilities((*hOdmVibrate)->hOdmServicePmuDevice, (*hOdmVibrate)->VddId, &((*hOdmVibrate)->RailCaps));
            break;
        }
    }

    return NV_TRUE;
}

/**
 *  @brief Closes the ODM device and destroys all allocated resources.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return None.
 */
void NvOdmVibClose(NvOdmVibDeviceHandle hOdmVibrate)
{
    if (hOdmVibrate != NULL)
    {
        NvOdmServicesPmuClose(hOdmVibrate->hOdmServicePmuDevice);
        hOdmVibrate->hOdmServicePmuDevice = NULL;

        hOdmVibrate->VddId  = 0;

        NvOsMemset(&hOdmVibrate->RailCaps, 0, sizeof(NvOdmServicesPmuVddRailCapabilities));

        NvOdmOsFree(hOdmVibrate);
        hOdmVibrate = NULL;
    }
}


/**
 *  @brief Gets capabilities of the Vibrate device.
 *  @param hOdmVibrate    [IN] Opaque handle to the device.
 *  @param RequestedCaps  [IN] Specifies the capability to get.
 *  @param pCapsValue    [OUT] A pointer to the returned value.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibGetCaps(
    NvOdmVibDeviceHandle hOdmVibrate,
    NvOdmVibCaps RequestedCaps,
    NvU32 *pCapsValue)
{
    NV_ASSERT(hOdmVibrate);
    NV_ASSERT(pCapsValue);

    if (!hOdmVibrate || !pCapsValue)
    {
        return NV_FALSE;
    }

    return NV_TRUE;
}

/**
 *  @brief The frequency to the Vibro motor can be set
 *    using this function. A frequency less than zero will be
 *    clamped to zero and a frequency value beyond the max supported value
 *    will be clamped to the max supported value.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @param Freq         [IN] Frequency in Hz
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibSetFrequency(NvOdmVibDeviceHandle hOdmVibrate, NvS32 Freq)
{
    //AP20 Vibrator does'nt support setting Frequency
    return NV_TRUE;
}

/**
 *  @brief The dutycycle of the PWM driving the Vibro motor can be set
 *    using this function. A dutycycle less than zero will be
 *    clamped to zero and value beyond the max supported value
 *    will be clamped to the max supported value.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @param DCycle       [IN] Duty Cycle value in percentage (0%-100%)
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibSetDutyCycle(NvOdmVibDeviceHandle hOdmVibrate, NvS32 DCycle)
{
    //AP20 Vibrator does'nt support setting DutyCycle
    return NV_TRUE;
}

/**
 *  @brief Starts the Vibro with the frequency and duty-cycle set using the
 *    Set API.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibStart(NvOdmVibDeviceHandle hOdmVibrate)
{
    NvU32 SettlingTime = 0;

    NV_ASSERT(hOdmVibrate);

    if (!hOdmVibrate)
    {
        return NV_FALSE;
    }

    if (hOdmVibrate->hOdmServicePmuDevice != NULL)
    {
        // Search for the Vdd rail and power Off the module
        if (hOdmVibrate->VddId)
        {
            NvOdmServicesPmuSetVoltage(hOdmVibrate->hOdmServicePmuDevice,
                   hOdmVibrate->VddId, hOdmVibrate->RailCaps.requestMilliVolts, &SettlingTime);

            if (SettlingTime)
                NvOdmOsWaitUS(SettlingTime);
        }
    }

    return NV_TRUE;
}

/**
 *  @brief Stops the Vibro motor
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibStop(NvOdmVibDeviceHandle hOdmVibrate)
{
    NvU32 SettlingTime;

    NV_ASSERT(hOdmVibrate);

    if (!hOdmVibrate)
    {
        return NV_FALSE;
    }

    if (hOdmVibrate->hOdmServicePmuDevice != NULL)
    {
        // Search for the Vdd rail and power Off the module
        if (hOdmVibrate->VddId)
        {
            NvOdmServicesPmuSetVoltage(hOdmVibrate->hOdmServicePmuDevice,
                        hOdmVibrate->VddId, NVODM_VOLTAGE_OFF, &SettlingTime);

            if (SettlingTime)
                NvOdmOsWaitUS(SettlingTime);
        }
    }

    return NV_TRUE;
}
