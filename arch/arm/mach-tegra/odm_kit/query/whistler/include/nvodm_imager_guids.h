/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/include/nvodm_imager_guids.h
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

#ifndef NVODM_IMAGER_GUIDS_H
#define NVODM_IMAGER_GUIDS_H

#include "nvodm_query_discovery.h"

// E912-A02 and Concorde2 Sensors
#define OV5630_GUID NV_ODM_GUID('s','_','O','V','5','6','3','0')

// E911 Sensors
#define MI5130_GUID NV_ODM_GUID('s','_','M','I','5','1','3','0')
#define SEMCOVGA_GUID NV_ODM_GUID('S','E','M','C','O','V','G','A')

// E911 Focusers
// focuser for MI5130
#define DW9710_GUID NV_ODM_GUID('f','_','D','W','9','7','1','0')

/// focuser for OV5630
#define AD5820_GUID NV_ODM_GUID('f','_','A','D','5','8','2','0')

// E911 Flash
#define LTC3216_GUID NV_ODM_GUID('l','_','L','T','3','2','1','6')

// io addresses common to all imagers (clock)
#define COMMONIMAGER_GUID NV_ODM_GUID('s', '_', 'c', 'o', 'm', 'm', 'o', 'n')

// Pin Use Codes:
// VI/CSI Parallel and Serial Pins and GPIO Pins

// More than one device may be retrieved thru the query
#define NVODM_CAMERA_DEVICE_IS_DEFAULT   (1)

// The imager devices can connect to the parallel bus or the serial bus
// Parallel connections use pins VD0 thru VD9.
// Serial connections use the mipi pins (ex: CSI_D1AN/CSI_D1AP)
#define NVODM_CAMERA_DATA_PIN_SHIFT      (1)
#define NVODM_CAMERA_DATA_PIN_MASK       0x0F
#define NVODM_CAMERA_PARALLEL_VD0_TO_VD9 (1 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_PARALLEL_VD0_TO_VD7 (2 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1A      (4 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D2A      (5 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1A_D2A  (6 << NVODM_CAMERA_DATA_PIN_SHIFT)
#define NVODM_CAMERA_SERIAL_CSI_D1B      (7 << NVODM_CAMERA_DATA_PIN_SHIFT)

// Switching the encoding from the VideoInput module address to use with
// each GPIO module address.
// NVODM_IMAGER_GPIO will tell the nvodm imager how to use each gpio
// A gpio can be used for powerdown (lo, hi) or !powerdown (hi, lo)
// used for reset (hi, lo, hi) or for !reset (lo, hi, lo)
// Or, for mclk or pwm (unimplemented yet)
// We have moved the flash to its own, so it is not needed here
#define NVODM_IMAGER_GPIO_PIN_SHIFT    (24)
#define NVODM_IMAGER_UNUSED            (0x0)
#define NVODM_IMAGER_RESET             (0x1 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_RESET_AL          (0x2 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_POWERDOWN         (0x3 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_POWERDOWN_AL      (0x4 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// only on VGP0
#define NVODM_IMAGER_MCLK              (0x8 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// only on VGP6
#define NVODM_IMAGER_PWM               (0x9 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// If flash code wants the gpio's labelled
// use for any purpose, or not at all
#define NVODM_IMAGER_FLASH0           (0x5 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_FLASH1           (0x6 << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_FLASH2           (0x7 << NVODM_IMAGER_GPIO_PIN_SHIFT)
// Shutter control
#define NVOSM_IMAGER_SHUTTER          (0xA << NVODM_IMAGER_GPIO_PIN_SHIFT)
// 

#define NVODM_IMAGER_MASK              (0xF << NVODM_IMAGER_GPIO_PIN_SHIFT)
#define NVODM_IMAGER_CLEAR(_s)         ((_s) & ~(NVODM_IMAGER_MASK))
#define NVODM_IMAGER_IS_SET(_s)        (((_s) & (NVODM_IMAGER_MASK)) != 0)
#define NVODM_IMAGER_FIELD(_s)         ((_s) >> NVODM_IMAGER_GPIO_PIN_SHIFT)

// The imager devices can connect to the vi gpio (VGP) pins
// for various reasons: flash, powerdown, reset, pwm, mclk.
// Only certain pins can be used for certain activities.
// These flags should be OR'd together to form the proper 'address'
// in the NvOdmIoAddress for VideoInput.
// VGP1 & VGP2 are used for i2c
// _AL means 'active low', otherwise active high is assumed

#define NVODM_CAMERA_GPIO_PIN_SHIFT        (8)
#define NVODM_CAMERA_GPIO_PIN_MASK         (0x7)
#define NVODM_CAMERA_GPIO_PIN_WIDTH         3
#define NVODM_CAMERA_GPIO_PIN_COUNT         7

#define NVODM_CAMERA_UNUSED                (0x0)
#define NVODM_CAMERA_RESET(_s)         (0x1 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
#define NVODM_CAMERA_RESET_AL(_s)      (0x2 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
#define NVODM_CAMERA_POWERDOWN(_s)     (0x3 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
#define NVODM_CAMERA_POWERDOWN_AL(_s)  (0x4 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
#define NVODM_CAMERA_FLASH_LOW(_s)     (0x5 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
#define NVODM_CAMERA_FLASH_HIGH(_s)    (0x6 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
// only on VGP0
#define NVODM_CAMERA_MCLK(_s)          (0x7 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))
// only on VGP6
#define NVODM_CAMERA_PWM(_s)           (0x7 << (_s+NVODM_CAMERA_GPIO_PIN_SHIFT))

#define NVODM_VGP0_SHIFT            0
#define NVODM_VD10_SHIFT            (1*NVODM_CAMERA_GPIO_PIN_WIDTH)
#define NVODM_VD11_SHIFT            (2*NVODM_CAMERA_GPIO_PIN_WIDTH)
#define NVODM_VGP3_SHIFT            (3*NVODM_CAMERA_GPIO_PIN_WIDTH)
#define NVODM_VGP4_SHIFT            (4*NVODM_CAMERA_GPIO_PIN_WIDTH)
#define NVODM_VGP5_SHIFT            (5*NVODM_CAMERA_GPIO_PIN_WIDTH)
#define NVODM_VGP6_SHIFT            (6*NVODM_CAMERA_GPIO_PIN_WIDTH)

// VGP0
#define NVODM_CAMERA_VGP0_RESET        NVODM_CAMERA_RESET(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VGP0_SHIFT)
#define NVODM_CAMERA_VGP0_MCLK         NVODM_CAMERA_MCLK(NVODM_VGP0_SHIFT)
// VD10
#define NVODM_CAMERA_VD10_RESET        NVODM_CAMERA_RESET(NVODM_VD10_SHIFT)
#define NVODM_CAMERA_VD10_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VD10_SHIFT)
#define NVODM_CAMERA_VD10_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VD10_SHIFT)
#define NVODM_CAMERA_VD10_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VD10_SHIFT)
#define NVODM_CAMERA_VD10_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VD10_SHIFT)
#define NVODM_CAMERA_VD10_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VD10_SHIFT)
// VD11
#define NVODM_CAMERA_VD11_RESET        NVODM_CAMERA_RESET(NVODM_VD11_SHIFT)
#define NVODM_CAMERA_VD11_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VD11_SHIFT)
#define NVODM_CAMERA_VD11_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VD11_SHIFT)
#define NVODM_CAMERA_VD11_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VD11_SHIFT)
#define NVODM_CAMERA_VD11_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VD11_SHIFT)
#define NVODM_CAMERA_VD11_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VD11_SHIFT)
// VGP3
#define NVODM_CAMERA_VGP3_RESET        NVODM_CAMERA_RESET(NVODM_VGP3_SHIFT)
#define NVODM_CAMERA_VGP3_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VGP3_SHIFT)
#define NVODM_CAMERA_VGP3_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VGP3_SHIFT)
#define NVODM_CAMERA_VGP3_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VGP3_SHIFT)
#define NVODM_CAMERA_VGP3_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VGP3_SHIFT)
#define NVODM_CAMERA_VGP3_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VGP3_SHIFT)
// VGP4
#define NVODM_CAMERA_VGP4_RESET        NVODM_CAMERA_RESET(NVODM_VGP4_SHIFT)
#define NVODM_CAMERA_VGP4_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VGP4_SHIFT)
#define NVODM_CAMERA_VGP4_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VGP4_SHIFT)
#define NVODM_CAMERA_VGP4_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VGP4_SHIFT)
#define NVODM_CAMERA_VGP4_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VGP4_SHIFT)
#define NVODM_CAMERA_VGP4_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VGP4_SHIFT)
// VGP5
#define NVODM_CAMERA_VGP5_RESET        NVODM_CAMERA_RESET(NVODM_VGP5_SHIFT)
#define NVODM_CAMERA_VGP5_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VGP5_SHIFT)
#define NVODM_CAMERA_VGP5_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VGP5_SHIFT)
#define NVODM_CAMERA_VGP5_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VGP5_SHIFT)
#define NVODM_CAMERA_VGP5_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VGP5_SHIFT)
#define NVODM_CAMERA_VGP5_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VGP5_SHIFT)
// VGP6
#define NVODM_CAMERA_VGP6_RESET        NVODM_CAMERA_RESET(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_RESET_AL     NVODM_CAMERA_RESET_AL(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_POWERDOWN    NVODM_CAMERA_POWERDOWN(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_POWERDOWN_AL NVODM_CAMERA_POWERDOWN_AL(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_FLASH_LOW    NVODM_CAMERA_FLASH_LOW(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_FLASH_HIGH   NVODM_CAMERA_FLASH_HIGH(NVODM_VGP6_SHIFT)
#define NVODM_CAMERA_VGP6_PWM          NVODM_CAMERA_PWM(NVODM_VGP6_SHIFT)

#endif
