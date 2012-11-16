/*
 * intel_soc_mrfld.h
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifdef CONFIG_INTEL_ATOM_MRFLD_POWER

#define   PM_SUPPORT		0x21

#define ISP_POS			7
#define ISP_SUB_CLASS		0x80

#define PMU1_MAX_DEVS			8
#define PMU2_MAX_DEVS			32

#define MRFLD_S3_HINT			0x64

#define GFX_LSS_INDEX			1
#define PMU_SDIO0_LSS_01		1
#define PMU_EMMC0_LSS_02		2
#define PMU_EMMC1_LSS_03		3
#define PMU_SDIO1_LSS_04		4
#define PMU_HSI_LSS_05			5
#define PMU_SECURITY_LSS_06		6
#define PMU_RESERVED_LSS_07		7
#define PMU_USB_MPH_LSS_08		8
#define PMU_USB3_LSS_09			9
#define PMU_AUDIO_LSS_10		10
#define PMU_AUDIO_DMA0_11		11
#define PMU_SSP2_LSS_11			11
#define PMU_AUDIO_DMA1_12		12
#define PMU_SSP0_LSS_12			12
#define PMU_SSP1_LSS_13			13
#define PMU_RESERVED_LSS_14		14
#define PMU_RESERVED_LSS_15		15
#define PMU_RESERVED_LSS_16		16
#define PMU_SSP3_LSS_17			17
#define PMU_SSP5_LSS_18			18
#define PMU_SSP6_LSS_19			19
#define PMU_I2C1_LSS_20			20
#define PMU_I2C2_LSS_21			21
#define PMU_I2C3_LSS_22			22
#define PMU_I2C4_LSS_23			23
#define PMU_I2C5_LSS_24			24
#define PMU_GP_DMA_LSS_25		25
#define PMU_I2C6_LSS_26			26
#define PMU_I2C7_LSS_27			27
#define PMU_USB_OTG_LSS_28		28
#define PMU_RESERVED_LSS_29		29
#define PMU_RESERVED_LSS_30		30
#define PMU_UART0_LSS_31		31
#define PMU_UART1_LSS_31		31
#define PMU_UART2_LSS_31		31

#define EMMC0_LSS			PMU_EMMC0_LSS_02

/*dont ignore anything as of now*/

#define IGNORE_SSS0			0
#define IGNORE_SSS1			0
#define IGNORE_SSS2			0
#define IGNORE_SSS3			0

#define IGNORE_S3_WKC0			0
#define IGNORE_S3_WKC1			0

#endif
