/*
 * arch/arm/mach-tegra/odm_kit/query/harmony/nvodm_query_nand.c
 *
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

#include "nvodm_query_nand.h"
#include "nvcommon.h"

// fill params for all required nand flashes here.
// this list will end when vendor id and chipd id will be zero.
// hence, all supported chips should be listed before that.
NvOdmNandFlashParams g_Params[] =
{
    /*
    {
        VendorId, DeviceId, NandType, IsCopyBackCommandSupported, IsCacheWriteSupported, CapacityInMB, ZonesPerDevice,
        BlocksPerZone, OperationSuccessStatus, InterleaveCapability, EccAlgorithm,
        ErrorsCorrectable, SkippedSpareBytes,
        TRP, TRH (TREH), TWP, TWH, TCS, TWHR, TWB, TREA, TADL,
        TCLS, TCLH, TCH, TALS, TALH, TRC, TWC, TCR(TCLR), TAR, TRR, NandDeviceType, ReadIdFourthByte
    }
    Note :
    TADL values for flashes K9F1G08Q0M, K9F1G08U0M, TH58NVG4D4CTG00,
    TH58NVG3D4BTG00, TH58NVG2S3BFT00 is not available from their data sheets.
    Hence TADL is computed as
        tADL = (tALH + tALS + tWP).
    */
    // filling odm parameter structure for Samsung K9K8G08U0M
    {
        0xEC, 0xD3, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 1024, 4,
        2048, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        15, 10, 15, 10, 20, 60, 100, 26, 70,
        12, 5, 5, 12, 5, 25, 25, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x95
     },
    // filling odm parameter structure for Samsung K9GAG08U0D
    {
        0xEC, 0xD5, NvOdmNandFlashType_Mlc, NV_TRUE, NV_FALSE, 2048, 2,
        2048, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Eight, NvOdmNandSkipSpareBytes_4,
        15, 10, 15, 10, 20, 60, 100, 20, 100,
        15, 5, 5, 15, 5, 30, 30, 10, 10, 20, NvOdmNandDeviceType_Type2, 0x29
    },
    // filling odm parameter structure for Samsung K9W8G08U1M
    {
        0xEC, 0xDC, NvOdmNandFlashType_Slc, NV_TRUE, NV_TRUE, 1024, 2,
        4096, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        15, 10, 15, 10, 15, 60, 100, 18, 100,
        10, 5, 5, 10, 5, 30, 30, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x15
    },
    // filling odm parameter structure for Samsung K9F1G08Q0M
    {
        0xEC, 0xA1, NvOdmNandFlashType_Slc, NV_TRUE, NV_TRUE, 128, 1,
        1024, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        60, 20, 60, 20, 0, 60, 100, 60, 70,
        0, 10, 10, 0, 10, 80, 80, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x15
    },
    // filling odm parameter structure for Samsung K9F1G08U0M
    {
        0xEC, 0xF1, NvOdmNandFlashType_Slc, NV_TRUE, NV_TRUE, 128, 1,
        1024, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        25, 15, 25, 15, 0, 60, 100, 30, 35,
        0, 10, 10, 0, 10, 50, 45, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x15
     },
    // filling odm parameter structure for Samsung K9L8G08U0M
    {
        0xEC, 0xD3, NvOdmNandFlashType_Mlc, NV_FALSE, NV_FALSE, 1024, 4,
        1024, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        15, 10, 15, 10, 20, 60, 100, 20, 35,
        15, 5, 5, 15, 5, 30, 30, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x25
     },
    // filling odm parameter structure for Samsung K9G4G08U0M
    {
        0xEC, 0xDC, NvOdmNandFlashType_Mlc, NV_FALSE, NV_FALSE, 512, 2,
        1024, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        15, 10, 15, 15, 15, 60, 100, 18, 50,
        10, 5, 5, 10, 5, 30, 45, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x25
     },
    // filling odm parameter structure for Samsung K5E2G1GACM
    {
        0xEC, 0xAA, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 256, 2,
        1024, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        21, 15, 21, 15, 31, 60, 100, 30, 100,
        21, 5, 5, 21, 5, 42, 42, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x15
     },
/*
    // filling odm parameter structure for Toshiba TH58NVG4D4CTG00
    {
        0x98, 0xD5, NvOdmNandFlashType_Mlc, NV_FALSE, NV_FALSE, 2048, 1,
        8192, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        41, 15, 15, 10, 0, 30, 20, 200, 41, 21,
        0, 6, 6, 0, 6, NvOdmNandDeviceType_Type1, 0x25
    },
    // filling odm parameter structure for Toshiba TH58NVG3D4BTG00
    {
        0x98, 0xD3, NvOdmNandFlashType_Mlc, NV_FALSE, NV_TRUE, 1024, 1,
        4096, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        41, 15, 15, 10, 0, 30, 20, 200, 41, 35,
        0, 10, 10, 0, 10, NvOdmNandDeviceType_Type1, 0x25
     },
    // filling odm parameter structure for Toshiba TH58NVG2S3BFT00
    {
        0x98, 0xDC, NvOdmNandFlashType_Slc, NV_FALSE, NV_FALSE, 512, 1,
        1024, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        41, 15, 15, 10, 0, 30, 20, 200, 41, 35,
        0, 10, 10, 0, 10, NvOdmNandDeviceType_Type1, 0x25
     },
*/
    // filling odm parameter structure for Samsung K9LBG08U0M
    {
        0xEC, 0xD7, NvOdmNandFlashType_Mlc, NV_TRUE, NV_FALSE, 4096, 4, 2048,
        0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Six, NvOdmNandSkipSpareBytes_4,
        12, 10, 12, 10, 20, 60, 100, 20, 100,
        12, 5, 5, 12, 5, 25, 25, 10, 10, 20, NvOdmNandDeviceType_Type1, 0xB6
    },
    //Hynix H8BES0UQ0MCP 
     {
        0xAD, 0xBC,  NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 512, 2, 2048, 
        0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,  
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4, 
        25, 10, 25, 15, 35, 60, 100, 30, 100,
        25, 10, 10, 25, 10, 45, 45, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x55
     },
     //Hynix H8BCS0SJ0MCP 
    {
        0xAD, 0xBA, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 256, 1, 2048, 
        0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,  
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4, 
        25, 15, 25, 15, 35, 60, 100, 30, 100,
        25, 10, 10, 25, 10, 45, 45, 10, 10, 25, NvOdmNandDeviceType_Type1, 0x55
     },

     //Hynix H8BCS0RJ0MCP 
    {
        0xAD, 0xAA, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 256, 1, 2048, 
        0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon, 
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4, 
        25, 10, 25, 15, 35, 60, 100, 30, 100,
        25, 10, 10, 25, 10, 45, 45, 10, 10, 25, NvOdmNandDeviceType_Type1, 0x15
     },
     /*Numonyx MCP - NAND02GR3B2D*/ 
    {
        0x20, 0xAA, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 256, 1, 
        2048, 0x40, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon, 
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        25, 15, 25, 15, 35, 60, 100, 30, 100,
        25, 10, 10, 25, 10, 45, 45, 10, 10, 25, NvOdmNandDeviceType_Type1, 0x15
     },
     // Hynix HY27UF084G2B (readid 4th byte 0x95)
     {
        0xAD, 0xDC, NvOdmNandFlashType_Slc, NV_TRUE, NV_FALSE, 512, 2,
        2048, 0x60, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon, 
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4, 
        12, 10, 12, 10, 20, 80, 100, 20, 70,
        12, 5, 5, 12, 5, 25, 25, 10, 10, 20, NvOdmNandDeviceType_Type1, 0x95
     },
    /* "This is the end of device list please do not modify this. To add support for more flash parts,
        add device category for those parts before this element"*/
    {
        0, 0, NvOdmNandFlashType_UnKnown, NV_FALSE, NV_FALSE, 0, 0,
        0, 0, SINGLE_PLANE, NvOdmNandECCAlgorithm_ReedSolomon,
        NvOdmNandNumberOfCorrectableSymbolErrors_Four, NvOdmNandSkipSpareBytes_4,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NvOdmNandDeviceType_Type1, 0
    }
};

NvOdmNandFlashParams *NvOdmNandGetFlashInfo (NvU32 ReadID)
{
    NvU8 TempValue;
    NvU8 VendorId = 0;
    NvU8 DeviceId = 0;
    NvU8 ReadIdFourthByte = 0;
    NvOdmNandFlashType NandType;
    NvU8 i = 0;
    // To extract Vendor Id
    VendorId = (NvU8) (ReadID & 0xFF);
    // To extract Device Id
    DeviceId = (NvU8) ((ReadID >> DEVICE_SHIFT) & 0xFF);
    // To extract Fourth ID byte of Read ID - for checking if the flash is 42nm.
    ReadIdFourthByte = (NvU8) ((ReadID >> FOURTH_ID_SHIFT) & 0xFF);
    // To extract device Type Mask
    TempValue = (NvU8) ((ReadID >> FLASH_TYPE_SHIFT) & 0xC);
    if (TempValue)
    {
        NandType = NvOdmNandFlashType_Mlc;
    }
    else
    {
        NandType = NvOdmNandFlashType_Slc;
    }
    // following ORing is done to check if we reached the end of the list.
    while ((g_Params[i].VendorId) | (g_Params[i].DeviceId))
    {
        if ((g_Params[i].VendorId == VendorId) &&
            (g_Params[i].DeviceId == DeviceId) &&
            (g_Params[i].ReadIdFourthByte == ReadIdFourthByte) &&
            (g_Params[i].NandType == NandType))
        {
            return &g_Params[i];
        }
        else
            i++;
    }
    // This condition will be reached if "g_Params" is not having Parameters of the flash used.
    // Hence add the parameters required in the table.
    return NULL;
}

