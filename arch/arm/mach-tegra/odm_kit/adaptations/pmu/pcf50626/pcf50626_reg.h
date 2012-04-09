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

#ifndef PCF50626_REG_HEADER
#define PCF50626_REG_HEADER

#if defined(__cplusplus)
extern "C"
{
#endif


// The following are the OTP reset values
#define PCF50626_ID_DEFAULT         0x31 // TBD, may change
#define PCF50626_BVMC_RESET         0x18

#define PCF50626_D1REGC1_RESET      0x6C
#define PCF50626_D1REGC2_RESET      0x01
#define PCF50626_D1REGC3_RESET      0x00

#define PCF50626_D2REGC1_RESET      0x6C
#define PCF50626_D2REGC2_RESET      0xE1
#define PCF50626_D2REGC3_RESET      0x00

#define PCF50626_D3REGC1_RESET      0x18
#define PCF50626_D3REGC2_RESET      0xE1
#define PCF50626_D3REGC3_RESET      0x80

#define PCF50626_D7REGC2_RESET      0x01
#define PCF50626_D7REGC3_RESET      0x00

#define PCF50626_D8REGC2_RESET      0xE1
#define PCF50626_D8REGC3_RESET      0x40

#define PCF50626_RF1REGC1_RESET     0x58
#define PCF50626_RF1REGC2_RESET     0x01
#define PCF50626_RF1REGC3_RESET     0x40

#define PCF50626_RF2REGC1_RESET     0x58
#define PCF50626_RF2REGC2_RESET     0x01
#define PCF50626_RF2REGC3_RESET     0x40

#define PCF50626_IOREGC1_RESET      0x30
#define PCF50626_IOREGC2_RESET      0xE1
#define PCF50626_IOREGC3_RESET      0x40

#define PCF50626_USBREGC1_RESET     0x58
#define PCF50626_USBREGC2_RESET     0xE1
#define PCF50626_USBREGC3_RESET     0x40

#define PCF50626_LCREGC1_RESET      0x18
#define PCF50626_LCREGC2_RESET      0xE1
#define PCF50626_LCREGC3_RESET      0x00

#define PCF50626_DCD1C1_RESET       0x18
#define PCF50626_DCD1C2_RESET       0xE1
#define PCF50626_DCD1C3_RESET       0x00
#define PCF50626_DCD1C4_RESET       0x0F

#define PCF50626_DCD2C1_RESET       0x30
#define PCF50626_DCD2C2_RESET       0xE1
#define PCF50626_DCD2C3_RESET       0x80
#define PCF50626_DCD2C4_RESET       0x0F

#define PCF50626_DCUDC2_RESET       0xE1
#define PCF50626_DCUDC3_RESET       0xC0

#define PCF50626_ALMCAL_RESET       0x11
#define PCF50626_ALMCRV1_RESET      0x01
#define PCF50626_ALMCRV2_RESET      0x28
#define PCF50626_ALMCRV3_RESET      0x50
#define PCF50626_ALMCRV4_RESET      0x78
#define PCF50626_LED1C_RESET        0x4D
#define PCF50626_LEDCC_RESET        0x01

#define PCF50626_ADCC1_RESET        0x02
#define PCF50626_ADCC2_RESET        0x00
#define PCF50626_ADCC3_RESET        0x00
#define PCF50626_ADCC4_RESET        0x00

#define PCF50626_DCULEDC1_RESET     0x08
#define PCF50626_DCULEDC2_RESET     0x01
#define PCF50626_DCULEDC3_RESET     0x00
#define PCF50626_DCULEDDIMMAN_RESET 0x3F

#define PCF50626_CBCC1_RESET        0x2B
#define PCF50626_CBCC2_RESET        0x5A
#define PCF50626_CBCC3_RESET        0x12
#define PCF50626_CBCC4_RESET        0x12
#define PCF50626_CBCC5_RESET        0x0B
#define PCF50626_CBCC6_RESET        0x02

#define PCF50626_BBCC1_RESET        0x00


//Table.  168
#define PCF50626_ID_ADDR            0x00
#define PCF50626_INT1_ADDR          0x01
#define PCF50626_INT2_ADDR          0x02
#define PCF50626_INT3_ADDR          0x03
#define PCF50626_INT4_ADDR          0x04
#define PCF50626_INT5_ADDR          0x05
#define PCF50626_INT6_ADDR          0x06
#define PCF50626_INT7_ADDR          0x09

#define PCF50626_INT1M_ADDR         0x0A
#define PCF50626_INT2M_ADDR         0x0B
#define PCF50626_INT3M_ADDR         0x0C
#define PCF50626_INT4M_ADDR         0x0D
#define PCF50626_INT5M_ADDR         0x0E
#define PCF50626_INT6M_ADDR         0x0F
#define PCF50626_INT7M_ADDR         0x12

#define PCF50626_ERROR_ADDR         0x13

#define PCF50626_OOCC1_ADDR         0x14
#define PCF50626_OOCC2_ADDR         0x15
#define PCF50626_OOCPH_ADDR         0x16
#define PCF50626_OOCS_ADDR          0x17

#define PCF50626_BVMC_ADDR          0x18

#define PCF50626_RECC1_ADDR         0x19
#define PCF50626_RECC2_ADDR         0x1A
#define PCF50626_RECS_ADDR          0x1B

#define PCF50626_RTC1_ADDR          0x1C
#define PCF50626_RTC2_ADDR          0x1D
#define PCF50626_RTC3_ADDR          0x1E
#define PCF50626_RTC4_ADDR          0x1F

#define PCF50626_RTC1A_ADDR         0x20
#define PCF50626_RTC2A_ADDR         0x21
#define PCF50626_RTC3A_ADDR         0x22
#define PCF50626_RTC4A_ADDR         0x23

#define PCF50626_CBCC1_ADDR         0x24
#define PCF50626_CBCC2_ADDR         0x25
#define PCF50626_CBCC3_ADDR         0x26
#define PCF50626_CBCC4_ADDR         0x27
#define PCF50626_CBCC5_ADDR         0x28
#define PCF50626_CBCC6_ADDR         0x29

#define PCF50626_CBCS1_ADDR         0x2A
#define PCF50626_CBCS2_ADDR         0x2B
#define PCF50626_BBCC1_ADDR         0x2C
#define PCF50626_PWM1S_ADDR         0x2D
#define PCF50626_PWM1D_ADDR         0x2E
#define PCF50626_PWM2S_ADDR         0x2F
#define PCF50626_PWM2D_ADDR         0x30

#define PCF50626_LED1C_ADDR         0x31
#define PCF50626_LED2C_ADDR         0x32
#define PCF50626_LEDCC_ADDR         0x33

#define PCF50626_ADCC2_ADDR         0x34
#define PCF50626_ADCC3_ADDR         0x35
#define PCF50626_ADCC4_ADDR         0x36
#define PCF50626_ADCC1_ADDR         0x37
#define PCF50626_ADCS1_ADDR         0x38
#define PCF50626_ADCS2_ADDR         0x39
#define PCF50626_ADCS3_ADDR         0x3A

#define PCF50626_TSIC2_ADDR         0x3B
#define PCF50626_TSIC1_ADDR         0x3C
#define PCF50626_TSIDAT1_ADDR       0x3D
#define PCF50626_TSIDAT2_ADDR       0x3E
#define PCF50626_TSIDAT3_ADDR       0x3F

#define PCF50626_GPIO1C1_ADDR       0x40
#define PCF50626_E1REGC2_ADDR       0x41
#define PCF50626_E1REGC3_ADDR       0x42
#define PCF50626_GPIO2C1_ADDR       0x43
#define PCF50626_E2REGC2_ADDR       0x44
#define PCF50626_E2REGC3_ADDR       0x45
#define PCF50626_GPIO3C1_ADDR       0x46
#define PCF50626_E3REGC2_ADDR       0x47
#define PCF50626_E3REGC3_ADDR       0x48
#define PCF50626_GPIO4C1_ADDR       0x49
#define PCF50626_E4REGC2_ADDR       0x4A
#define PCF50626_E4REGC3_ADDR       0x4B
#define PCF50626_GPIO5C1_ADDR       0x4C
#define PCF50626_E5REGC2_ADDR       0x4D
#define PCF50626_E5REGC3_ADDR       0x4E
#define PCF50626_GPIO6C1_ADDR       0x4F
#define PCF50626_E6REGC2_ADDR       0x50
#define PCF50626_E6REGC3_ADDR       0x51

#define PCF50626_GPO1C1_ADDR        0x52
#define PCF50626_EO1REGC2_ADDR      0x53
#define PCF50626_EO1REGC3_ADDR      0x54
#define PCF50626_GPO2C1_ADDR        0x55
#define PCF50626_EO2REGC2_ADDR      0x56
#define PCF50626_EO2REGC3_ADDR      0x57
#define PCF50626_GPO3C1_ADDR        0x58
#define PCF50626_EO3REGC2_ADDR      0x59
#define PCF50626_EO3REGC3_ADDR      0x5A
#define PCF50626_GPO4C1_ADDR        0x5B
#define PCF50626_EO4REGC2_ADDR      0x5C
#define PCF50626_EO4REGC3_ADDR      0x5D

#define PCF50626_D1REGC1_ADDR       0x5E
#define PCF50626_D1REGC2_ADDR       0x5F
#define PCF50626_D1REGC3_ADDR       0x60

#define PCF50626_D2REGC1_ADDR       0x61
#define PCF50626_D2REGC2_ADDR       0x62
#define PCF50626_D2REGC3_ADDR       0x63

#define PCF50626_D3REGC1_ADDR       0x64
#define PCF50626_D3REGC2_ADDR       0x65
#define PCF50626_D3REGC3_ADDR       0x66

#define PCF50626_D4REGC1_ADDR       0x67
#define PCF50626_D4REGC2_ADDR       0x68
#define PCF50626_D4REGC3_ADDR       0x69

#define PCF50626_D5REGC1_ADDR       0x6A
#define PCF50626_D5REGC2_ADDR       0x6B
#define PCF50626_D5REGC3_ADDR       0x6C

#define PCF50626_D6REGC1_ADDR       0x6D
#define PCF50626_D6REGC2_ADDR       0x6E
#define PCF50626_D6REGC3_ADDR       0x6F

#define PCF50626_D7REGC1_ADDR       0x70
#define PCF50626_D7REGC2_ADDR       0x71
#define PCF50626_D7REGC3_ADDR       0x72

#define PCF50626_D8REGC1_ADDR       0x73
#define PCF50626_D8REGC2_ADDR       0x74
#define PCF50626_D8REGC3_ADDR       0x75

#define PCF50626_RF1REGC1_ADDR      0x76
#define PCF50626_RF1REGC2_ADDR      0x77
#define PCF50626_RF1REGC3_ADDR      0x78

#define PCF50626_RF2REGC1_ADDR      0x79
#define PCF50626_RF2REGC2_ADDR      0x7A
#define PCF50626_RF2REGC3_ADDR      0x7B

#define PCF50626_RF3REGC1_ADDR      0x7C
#define PCF50626_RF3REGC2_ADDR      0x7D
#define PCF50626_RF3REGC3_ADDR      0x7E

#define PCF50626_RF4REGC1_ADDR      0x7F
#define PCF50626_RF4REGC2_ADDR      0x80
#define PCF50626_RF4REGC3_ADDR      0x81

#define PCF50626_IOREGC1_ADDR       0x82
#define PCF50626_IOREGC2_ADDR       0x83
#define PCF50626_IOREGC3_ADDR       0x84

#define PCF50626_USBREGC1_ADDR      0x85
#define PCF50626_USBREGC2_ADDR      0x86
#define PCF50626_USBREGC3_ADDR      0x87

#define PCF50626_USIMREGC1_ADDR     0x88
#define PCF50626_USIMREGC2_ADDR     0x89
#define PCF50626_USIMREGC3_ADDR     0x8A

#define PCF50626_LCREGC1_ADDR       0x8B
#define PCF50626_LCREGC2_ADDR       0x8C
#define PCF50626_LCREGC3_ADDR       0x8D

#define PCF50626_HCREGC1_ADDR       0x8E
#define PCF50626_HCREGC2_ADDR       0x8F
#define PCF50626_HCREGC3_ADDR       0x90

#define PCF50626_DCD1C1_ADDR        0x91
#define PCF50626_DCD1C2_ADDR        0x92
#define PCF50626_DCD1C3_ADDR        0x93
#define PCF50626_DCD1C4_ADDR        0x94

#define PCF50626_DCD1DVM1_ADDR      0x95
#define PCF50626_DCD1DVM2_ADDR      0x96
#define PCF50626_DCD1DVM3_ADDR      0x97
#define PCF50626_DCD1DVMTIM_ADDR    0x98

#define PCF50626_DCD2C1_ADDR        0x99
#define PCF50626_DCD2C2_ADDR        0x9A
#define PCF50626_DCD2C3_ADDR        0x9B
#define PCF50626_DCD2C4_ADDR        0x9C

#define PCF50626_DCD2DVM1_ADDR      0x9D
#define PCF50626_DCD2DVM2_ADDR      0x9E
#define PCF50626_DCD2DVM3_ADDR      0x9F
#define PCF50626_DCD2DVMTIM_ADDR    0xA0

#define PCF50626_DCUDC1_ADDR        0xA1
#define PCF50626_DCUDC2_ADDR        0xA2
#define PCF50626_DCUDC3_ADDR        0xA3
#define PCF50626_DCUDC4_ADDR        0xA4
#define PCF50626_DCUDDVMTIM_ADDR    0xA5

#define PCF50626_DCULEDC1_ADDR      0xA6
#define PCF50626_DCULEDC2_ADDR      0xA7
#define PCF50626_DCULEDC3_ADDR      0xA8
#define PCF50626_DCULED_DIMMAN_ADDR 0xA9

#define PCF50626_ALMCAL_ADDR        0xAA
#define PCF50626_ALMCALMEA_ADDR     0xAB
#define PCF50626_ALMCRV1_ADDR       0xAC
#define PCF50626_ALMCRV2_ADDR       0xAD
#define PCF50626_ALMCRV3_ADDR       0xAE
#define PCF50626_ALMCRV4_ADDR       0xAF

#define PCF50626_GPIOS_ADDR         0xB0
#define PCF50626_DREGS1_ADDR        0xB1
#define PCF50626_DREGS2_ADDR        0xB2
#define PCF50626_RFREGS_ADDR        0xB3
#define PCF50626_GREGS_ADDR         0xB4

#define PCF50626_GPIO7C1_ADDR       0xB5
#define PCF50626_GPIO8C1_ADDR       0xB6

#define PCF50626_USIMDETC_ADDR      0xB7

#define PCF50626_TSINOI_ADDR        0xFE
#define PCF50626_TSIDAT4_ADDR       0xFF


/* field defines for register bit ops */
#define PCF50626_C2_OPMOD_SHIFT                     0x05
#define PCF50626_C2_OPMOD_ON                        0xE1
#define PCF50626_C2_OPMOD_OFF                       0x01
#define PCF50626_C1_OUTPUT_MASK                     0x7F

#define PCF50626_OOCS_ONKEY_SHIFT                   0x00
#define PCF50626_OOCS_ONKEY_MASK                    0x01
#define PCF50626_OOCS_REC1_SHIFT                    0x01
#define PCF50626_OOCS_REC1_MASK                     0x01
#define PCF50626_OOCS_BATOK_SHIFT                   0x02
#define PCF50626_OOCS_BATOK_MASK                    0x01
#define PCF50626_OOCS_MCHGOK_SHIFT                  0x04
#define PCF50626_OOCS_MCHGOK_MASK                   0x01
#define PCF50626_OOCS_UCHGOK_SHIFT                  0x05
#define PCF50626_OOCS_UCHGOK_MASK                   0x01
#define PCF50626_OOCS_TEMPOK_SHIFT                  0x06
#define PCF50626_OOCS_TEMPOK_MASK                   0x01

#define PCF50626_CBCS1_BATTFUL_SHIFT                0x00
#define PCF50626_CBCS1_BATTFUL_MASK                 0x01
#define PCF50626_CBCS1_TLIMIT_SHIFT                 0x01
#define PCF50626_CBCS1_TLIMIT_MASK                  0x01
#define PCF50626_CBCS1_WDEXP_SHIFT                  0x02
#define PCF50626_CBCS1_WDEXP_MASK                   0x01
#define PCF50626_CBCS1_ILIMIT_SHIFT                 0x03
#define PCF50626_CBCS1_ILIMIT_MASK                  0x01
#define PCF50626_CBCS1_VLIMIT_SHIFT                 0x04
#define PCF50626_CBCS1_VLIMIT_MASK                  0x01
#define PCF50626_CBCS1_RESSTAT_SHIFT                0x07
#define PCF50626_CBCS1_RESSTAT_MASK                 0x01
#define PCF50626_CBCS2_NOBAT_SHIFT                  0x00
#define PCF50626_CBCS2_NOBAT_MASK                   0x01
#define PCF50626_CBCS2_USBSUSPSTAT_SHIFT            0x02
#define PCF50626_CBCS2_USBSUSPSTAT_MASK             0x01
#define PCF50626_CBCS2_CHGOVP_SHIFT                 0x03
#define PCF50626_CBCS2_CHGOVP_MASK                  0x01

#define PCF50626_ADCC1_STARTCMD_SHIFT               0x00
#define PCF50626_ADCC1_STARTCMD_MASK                0x01
#define PCF50626_ADCC1_STARTCMD_START               0x01
#define PCF50626_ADCC1_STARTCMD_STOP                0x00

#define PCF50626_ADCS3_ADCRDY_SHIFT                 0x07
#define PCF50626_ADCS3_ADCRDY_MASK                  0x01
#define PCF50626_ADCS3_ADCRDY_READY                 0x01

#define PCF50626_DCULED_DIMMAN_LEDMAN_MASK          0x3F
#define PCF50626_DCULED_DIMMAN_LEDMAN_SHIFT         0x00

#define PCF50626_DCULED_DIMMAN_ALMSEL_MASK          0x01
#define PCF50626_DCULED_DIMMAN_ALMSEL_SHIFT         0x06
#define PCF50626_DCULED_DIMMAN_ALMSEL_ALM           0x01
#define PCF50626_DCULED_DIMMAN_ALMSEL_MAN           0x00

#define PCF50626_INT1_LOWBATT                       0x01
#define PCF50626_INT1_HIGHTEMP                      0x80
#define PCF50626_INT2_VMAX                          0x40
#define PCF50626_INT3_MCHGINS                       0x40
#define PCF50626_INT3_MCHGRM                        0x80
#define PCF50626_INT4_CHGRES                        0x01
#define PCF50626_INT4_BATFUL                        0x08

#define PCF50626_CBCC1_CHGENA_SHIFT                 0x00
#define PCF50626_CBCC1_CHGENA_MASK                  0x01
#define PCF50626_CBCC2_SUSPENA_SHIFT                 0x02
#define PCF50626_CBCC2_SUSPENA_MASK                  0x01

//rail specs
#define PCF50626_DCDXOUT_VOLTAGE_OFFSET_MV                  600
#define PCF50626_DCDXOUT_VOLTAGE_MIN_MV                     625
#define PCF50626_DCDXOUT_VOLTAGE_STEP_MV                    25
#define PCF50626_DCDXOUT_VOLTAGE_MAX_MV                     2700
#define PCF50626_DCDXOUT_TURNON_TIME_MICROSEC               365
#define PCF50626_DCDXOUT_SWITCH_TIME_MICROSEC               20

#define PCF50626_DCUDOUT_MODE1_VOLTAGE_OFFSET_MV            2675
#define PCF50626_DCUDOUT_MODE1_VOLTAGE_MIN_MV               3100
#define PCF50626_DCUDOUT_MODE1_VOLTAGE_STEP_MV              25
#define PCF50626_DCUDOUT_MODE1_VOLTAGE_MAX_MV               4975
#define PCF50626_DCUDOUT_TURNON_TIME_MICROSEC               65
#define PCF50626_DCUDOUT_SWITCH_TIME_MICROSEC               0

#define PCF50626_DXREGOUT_VOLTAGE_OFFSET_MV                 600
#define PCF50626_DXREGOUT_VOLTAGE_MIN_MV                    1200
#define PCF50626_DXREGOUT_VOLTAGE_STEP_MV                   100
#define PCF50626_DXREGOUT_VOLTAGE_MAX_MV                    3300
#define PCF50626_DXREGOUT_TURNON_TIME_MICROSEC              85
#define PCF50626_DXREGOUT_SWITCH_TIME_MICROSEC              0

#define PCF50626_RFXREGOUT_VOLTAGE_OFFSET_MV                600
#define PCF50626_RFXREGOUT_VOLTAGE_MIN_MV                   1200
#define PCF50626_RFXREGOUT_VOLTAGE_STEP_MV                  100
#define PCF50626_RFXREGOUT_VOLTAGE_MAX_MV                   3000
#define PCF50626_RFXREGOUT_TURNON_TIME_MICROSEC             285
#define PCF50626_RFXREGOUT_SWITCH_TIME_MICROSEC             0

#define PCF50626_HCREGOUT_VOLTAGE_OFFSET_MV                 600
#define PCF50626_HCREGOUT_VOLTAGE_MIN_MV                    1800
#define PCF50626_HCREGOUT_VOLTAGE_STEP_MV                   100
#define PCF50626_HCREGOUT_VOLTAGE_MAX_MV                    3000
#define PCF50626_HCREGOUT_TURNON_TIME_MICROSEC              85
#define PCF50626_HCREGOUT_SWITCH_TIME_MICROSEC              0

#define PCF50626_IOREGOUT_VOLTAGE_OFFSET_MV                 600
#define PCF50626_IOREGOUT_VOLTAGE_MIN_MV                    1200
#define PCF50626_IOREGOUT_VOLTAGE_STEP_MV                   100
#define PCF50626_IOREGOUT_VOLTAGE_MAX_MV                    3300
#define PCF50626_IOREGOUT_TURNON_TIME_MICROSEC              85
#define PCF50626_IOREGOUT_SWITCH_TIME_MICROSEC              0

#define PCF50626_USBREGOUT_VOLTAGE_OFFSET_MV                600
#define PCF50626_USBREGOUT_VOLTAGE_MIN_MV                   1200
#define PCF50626_USBREGOUT_VOLTAGE_STEP_MV                  100
#define PCF50626_USBREGOUT_VOLTAGE_MAX_MV                   3300
#define PCF50626_USBREGOUT_TURNON_TIME_MICROSEC             85
#define PCF50626_USBREGOUT_SWITCH_TIME_MICROSEC             0

#define PCF50626_USIMREGOUT_VOLTAGE_OFFSET_MV               600
#define PCF50626_USIMREGOUT_VOLTAGE_MIN_MV                  1800
#define PCF50626_USIMREGOUT_VOLTAGE_STEP_MV                 100
#define PCF50626_USIMREGOUT_VOLTAGE_MAX_MV                  3000
#define PCF50626_USIMREGOUT_TURNON_TIME_MICROSEC            105
#define PCF50626_USIMREGOUT_SWITCH_TIME_MICROSEC            0

#define PCF50626_LCREGOUT_VOLTAGE_OFFSET_MV                 600
#define PCF50626_LCREGOUT_VOLTAGE_MIN_MV                    600
#define PCF50626_LCREGOUT_VOLTAGE_STEP_MV                   50
#define PCF50626_LCREGOUT_VOLTAGE_MAX_MV                    2900
#define PCF50626_LCREGOUT_VOLTAGE_RESCHANGE_MV              1400
#define PCF50626_LCREGOUT_TURNON_TIME_MICROSEC              125
#define PCF50626_LCREGOUT_SWITCH_TIME_MICROSEC              0

#if defined(__cplusplus)
}
#endif


#endif //PCF50626_VOLTAGE_INFO_TABLE_HEADER

