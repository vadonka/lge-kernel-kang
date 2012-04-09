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
#include "max8907b.h"
#include "max8907b_i2c.h"
#include "max8907b_reg.h"

#define MAX8907B_I2C_SPEED_KHZ   400
#define MAX8907B_I2C_RETRY_CNT     2

// Maximum i2c transaction count
#define MAX_TRANSACTION_COUNT 5

NvBool Max8907bI2cWrite8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 Data)
{
    NvU32 i;
    NvU8 WriteBuffer[2];
    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;

    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = Addr & 0xFF;   // PMU offset
        WriteBuffer[1] = Data & 0xFF;   // written data

        TransactionInfo.Address = hPmu->DeviceAddr;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
            MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bI2cRead8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 *Data)
{
    NvU32 i;
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        NvU32 TransactionCount = 0;
        // Write the PMU offset
        ReadBuffer = Addr & 0xFF;

        TransactionInfo[TransactionCount].Address = hPmu->DeviceAddr;
        TransactionInfo[TransactionCount].Buf = &ReadBuffer;
        TransactionInfo[TransactionCount].Flags =
            NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        TransactionInfo[TransactionCount].Address = (hPmu->DeviceAddr | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer;
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        // Read data from PMU at the specified offset
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0],
            TransactionCount, MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
        {
            *Data = ReadBuffer;
            return NV_TRUE;
        }
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bI2cWrite32(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 Data)
{
    NvU32 i;
    NvU8 WriteBuffer[5];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo;

    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = (NvU8)(Addr & 0xFF);
        WriteBuffer[1] = (NvU8)((Data >> 24) & 0xFF);
        WriteBuffer[2] = (NvU8)((Data >> 16) & 0xFF);
        WriteBuffer[3] = (NvU8)((Data >>  8) & 0xFF);
        WriteBuffer[4] = (NvU8)(Data & 0xFF);

        TransactionInfo.Address = hPmu->DeviceAddr;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 5;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
            MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite32 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite32 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bI2cRead32(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 *Data)
{
    NvU32 i;
    NvU8 ReadBuffer[5];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];

    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        NvU32 TransactionCount = 0;
        ReadBuffer[0] = Addr & 0xFF;

        TransactionInfo[TransactionCount].Address = hPmu->DeviceAddr;
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags =
            NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        TransactionInfo[TransactionCount].Address = (hPmu->DeviceAddr | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 4;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0],
            TransactionCount, MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
        {
            *Data = (ReadBuffer[0] << 24) | (ReadBuffer[1] << 16) |
                    (ReadBuffer[2] << 8) | ReadBuffer[3];

            return NV_TRUE;
        }
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead32 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead32 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bRtcI2cWriteTime(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 Data)
{
    NvU32 i;
    NvU8 WriteBuffer[5];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo;

    NVODMPMU_PRINTF(("\n RTC I2C write: Addr=0x%x, Data=0x%x ", Addr, Data));
    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = (NvU8)(Addr & 0xFF);
        WriteBuffer[1] = (NvU8)((Data >> 24) & 0xFF);
        WriteBuffer[2] = (NvU8)((Data >> 16) & 0xFF);
        WriteBuffer[3] = (NvU8)((Data >>  8) & 0xFF);
        WriteBuffer[4] = (NvU8)(Data & 0xFF);

        TransactionInfo.Address = MAX8907B_RTC_SLAVE_ADDR;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 5;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
            MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("Max8907bRtcI2cWrite32 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("Max8907bRtcI2cWrite32 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bRtcI2cReadTime(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU32 *Data)
{
    NvU32 i;
    NvU8 ReadBuffer[4];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[MAX_TRANSACTION_COUNT];

    NVODMPMU_PRINTF(("\n RTC I2C read: Addr=0x%x ", Addr));
    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        NvU32 TransactionCount = 0;
        ReadBuffer[0] = Addr;

        TransactionInfo[TransactionCount].Address = MAX8907B_RTC_SLAVE_ADDR;
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags =
            NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        // Seconds / day
        if (TransactionCount >= MAX_TRANSACTION_COUNT)
            return NV_FALSE;
        TransactionInfo[TransactionCount].Address =
            (MAX8907B_RTC_SLAVE_ADDR | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        // Minutes / month
        if (TransactionCount >= MAX_TRANSACTION_COUNT)
            return NV_FALSE;
        TransactionInfo[TransactionCount].Address =
            (MAX8907B_RTC_SLAVE_ADDR | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[1];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        // Hours / YY1
        if (TransactionCount >= MAX_TRANSACTION_COUNT)
            return NV_FALSE;
        TransactionInfo[TransactionCount].Address =
            (MAX8907B_RTC_SLAVE_ADDR | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[2];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        // Weekday / YY2
        if (TransactionCount >= MAX_TRANSACTION_COUNT)
            return NV_FALSE;
        TransactionInfo[TransactionCount].Address =
            (MAX8907B_RTC_SLAVE_ADDR | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[3];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0],
            TransactionCount, MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
        if (status == NvOdmI2cStatus_Success)
        {
            *Data = (ReadBuffer[0] << 24) | (ReadBuffer[1] << 16) |
                    (ReadBuffer[2] << 8) | ReadBuffer[3];

            return NV_TRUE;
        }
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("Max8907bRtcI2cRead32 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("Max8907bRtcI2cRead32 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}


NvBool Max8907bRtcI2cWrite8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 Data)
{
    NvU32 i;
    NvU8 WriteBuffer[2];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo;

    NVODMPMU_PRINTF(("\n RTC I2C write: Addr=0x%x, Data=0x%x ", Addr, Data));
    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = Addr;
        WriteBuffer[1] = Data;

        TransactionInfo.Address = MAX8907B_RTC_SLAVE_ADDR;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
            MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);

        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("Max8907bRtcI2cWrite8 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("Max8907bRtcI2cWrite8 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}

NvBool Max8907bRtcI2cRead8(
   NvOdmPmuDeviceHandle hDevice,
   NvU8 Addr,
   NvU8 *Data)
{
    NvU32 i;
    NvU8 ReadBuffer[4];
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;
    Max8907bPrivData *hPmu = (Max8907bPrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[MAX_TRANSACTION_COUNT];

    NVODMPMU_PRINTF(("\n RTC I2C read: Addr=0x%x ", Addr));

    for (i = 0; i < MAX8907B_I2C_RETRY_CNT; i++)
    {
        NvU32 TransactionCount = 0;
        ReadBuffer[0] = Addr;
        TransactionInfo[TransactionCount].Address = MAX8907B_RTC_SLAVE_ADDR;
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags =
            NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        if (TransactionCount >= MAX_TRANSACTION_COUNT)
            return NV_FALSE;
        TransactionInfo[TransactionCount].Address =
            (MAX8907B_RTC_SLAVE_ADDR | 0x1);
        TransactionInfo[TransactionCount].Buf = &ReadBuffer[0];
        TransactionInfo[TransactionCount].Flags = 0;
        TransactionInfo[TransactionCount++].NumBytes = 1;

        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0],
            TransactionCount, MAX8907B_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
        if (status == NvOdmI2cStatus_Success)
        {
            *Data = ReadBuffer[0];
            return NV_TRUE;
        }
    }

    // Transaction Error
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("Max8907bRtcI2cRead8 Failed: Timeout\n"));
            break;
        case NvOdmI2cStatus_SlaveNotFound:
        default:
            NVODMPMU_PRINTF(("Max8907bRtcI2cRead8 Failed: SlaveNotFound\n"));
            break;
    }
    return NV_FALSE;
}
