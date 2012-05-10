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

#ifndef INCLUDED_MAX8907_INTERRUPT_HEADER
#define INCLUDED_MAX8907_INTERRUPT_HEADER

#include "max8907.h"

#if defined(__cplusplus)
extern "C"
{
#endif

NvBool 
Max8907SetupInterrupt(
    NvOdmPmuDeviceHandle hDevice);

void 
Max8907InterruptHandler_int(
    NvOdmPmuDeviceHandle hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_MAX8907_INTERRUPT_HEADER

