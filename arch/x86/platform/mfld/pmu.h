/*
 * pmu.h
 * Copyright (c) 2010, Intel Corporation.
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
#ifndef _MID_PMU_H_
#define _MID_PMU_H_

#define PCI_ID_ANY	(~0)

/* PMU register memory map */
#define PMU1_PMU_BASE_ADDR			0x940
#define PMU2_PMU_BASE_ADDR			0xFF11D000

#ifdef CONFIG_DRM_INTEL_MID
#define GFX_ENABLE
#endif

#define TELEPHONY_ENABLE_S0IX
#define WLAN_ENABLE_S0IX
#define I2C_ENABLE_S0IX
#define DWSPI_ENABLE_S0IX

/* PMU1/PMU2 PM Status Reg */
#define PMU_PM_STS_REG				0x00
/* PMU1/PMU2 PM Command Reg */
#define PMU_PM_CMD_REG				0x04
/* PMU1/PMU2 PM Interrupt Control and Status Reg */
#define PMU_PM_ICS_REG				0x08
/* PMU1/PMU2 PM subsystem configuration register */
#define PMU_PM_SSC_REG0				0x20
#define PMU_PM_SSC_REG1				0x24
#define PMU_PM_SSC_REG2				0x28
#define PMU_PM_SSC_REG3				0x2C
/* PMU1/PMU2 PM Subsystem status Reg */
#define PMU_PM_SSS_REG0				0x30
#define PMU_PM_SSS_REG1				0x34
#define PMU_PM_SSS_REG2				0x38
#define PMU_PM_SSS_REG3				0x3C
/* PMU2 PM Wake Control */
#define PMU_PM_WKC_REG0				0x10
#define PMU_PM_WKC_REG1				0x14
/* PMU2 PM Wake Status Reg */
#define PMU_PM_WKS_REG0				0x18
#define PMU_PM_WKS_REG1				0x1C
/* PMU2 PM Wake Status Reg */
#define PMU_PM_WSSC_REG0			0x40
#define PMU_PM_WSSC_REG1			0x44
#define PMU_PM_WSSC_REG2			0x48
#define PMU_PM_WSSC_REG3			0x4C
/* PMU2 PM Subsystem status Reg */
#define PMU_PM_C3C4_REG				0x50
/* PMU2 PM Subsystem status Reg */
#define PMU_PM_C5C6_REG				0x54
/* PMU2 PM Subsystem status Reg */
#define PMU_PM_MSIC_REG				0x58

#define ACK_C6_DMI_MSG				0xC2
#define MODE_ID_MAGIC_NUM			1
#define PMU1_END_ADDRESS			0x34
#define PMU2_END_ADDRESS			0x3B8

#define PMU2_SS_CFG_MASK			0x00FFFFFF
#define PMU2_WAKE_CFG_MASK			0x00000FFF
#define WAKE_CTRL_DEFAULT			0x1C2

/* PM table definitions */
#define PM_TABLE_BASE				0x1D000
#define PM_TABLE_PMU1_OFFSET			0x0
#define PM_TABLE_PMU2_OFFSET			0x0
#define PM_TABLE_PMU2_LOG_TO_PHY_MAP		0x0
#define PM_TABLE_MAX				0xF0
#define PM_TABLE_THRESH_MASK			0xFFFF

/* Inactivity register definitions */
#define PMU_INA_INT_STS_REG			0x220
#define PMU_INA_THRESH_BASE			0x240
#define PMU_INA_CTR_BASE			0x340
#define PMU_INA_STS_REG				0x210
#define PMU_INA_INVALID_SRC			0xFF
#define PMU_INA_CTR_MAX				32
#define SINGLE_BIT_MASK				0x1

/* External timers register definitions */
#define PMU2_EXT_TIMERS_BASE			0xff11B800
#define PMU2_EXT_TIMERS_BASE_MAX		0xB0
#define PMU2_EXT_TIMERS_MAX			8
#define PMU2_EXT_TIMERS_0_OFFSET		0x14
#define PMU2_EXT_TIMERS_LOAD_COUNT_REG		0x0
#define PMU2_EXT_TIMERS_CUR_VAL_REG		0x4
#define PMU2_EXT_TIMERS_CTRL_REG		0x8
#define PMU2_EXT_TIMERS_EOI_REG			0xC
#define PMU2_EXT_TIMERS_INT_STATUS_REG		0x10
#define PMU2_EXT_TIMERS_GLB_INT_STATUS_REG	0xa0
#define PMU2_EXT_TIMERS_GLB_EOI_REG		0xa4
#define PMU2_EXT_TIMERS_RAW_INT_STATUS_REG	0xa8
#define PMU2_EXT_TIMERS_COMP_VER		0xac
#define TIMER_COUNT_ALL_ONES			0xFFFFFFFF
#define TIMER_IS_MASKED				0x4
#define TIMER_IS_NOT_MASKED                     0x3
#define TIMER_FREE_RUNNING_MODE                 0x5
#define TIMER_USER_DEFINED_MODE                 0x2
#define TIMER_EN_VAL                            0x1

/* Definitions used for Enumeration */
#define   SFI_PCI_CFG_START			0x80000000
#define   SFI_PCI_CFG_SIZE			256
#define   PMU1_DEV_MASK				0x3FFF
#define   PMU2_DEV_MASK				0x3FFFFFFF
#define   PM_SUPPORT				0x21
#define   LOG_ID_MASK				0x7F
#define   SUB_CLASS_MASK			0xFF00

/* Definitions for Message Bus Interface */
#define MSG_CMD_REG				0xD0
#define MSG_DATA_REG				0xD4

/* Definition for C6 Offload MSR Address */
#define MSR_C6OFFLOAD_CTL_REG			0x120

#define MSR_C6OFFLOAD_SET_LOW			1
#define MSR_C6OFFLOAD_SET_HIGH			0

#define MSR_C6OFFLOAD_CLEAR_LOW			0
#define MSR_C6OFFLOAD_CLEAR_HIGH		0

#define C6OFFLOAD_BIT_MASK			0x2
#define C6OFFLOAD_BIT				0x2

/* Need to be changed after PMU driver is added as PCI device */
#define MID_PMU_MRST_DRV_DEV_ID			0x0810
#define MID_PMU_MFLD_DRV_DEV_ID			0x0828

#define PMU_DRV_NAME				"intel_pmu_driver"

#define MID_PCI_INDEX_HASH_BITS		7 /*size 128*/
#define MID_PCI_INDEX_HASH_SIZE		(1<<MID_PCI_INDEX_HASH_BITS)
#define MID_PCI_INDEX_HASH_MASK		(MID_PCI_INDEX_HASH_SIZE-1)

/* some random number for initvalue */
#define	MID_PCI_INDEX_HASH_INITVALUE	0x27041975

#define S5_VALUE	0x309D2601
#define S0I1_VALUE	0X30992601
#define LPMP3_VALUE	0X40492601
#define S0I3_VALUE	0X309B2601

#define WAKE_ENABLE_0		0xffffffff
#define WAKE_ENABLE_1		0xffffffff
#define INVALID_WAKE_SRC	0xFFFF

#define LOG_SS_MASK		0x80
#define MFLD_ISP_POS		7
#define ISP_SUB_CLASS		0x80

#define D0I0_MASK		0
#define D0I1_MASK		1
#define D0I2_MASK		2
#define D0I3_MASK		3

#define BITS_PER_LSS		2
#define MAX_LSS_POSSIBLE	64
#define SS_IDX_MASK           0x3
#define SS_POS_MASK           0xF

#define GFX_LSS_INDEX		1
#define PMU_SDIO0_LSS_00	0
#define PMU_EMMC0_LSS_01	1
#define PMU_AONT_LSS_02		2
#define PMU_HSI_LSS_03		3
#define PMU_SECURITY_LSS_04	4
#define PMU_EMMC1_LSS_05	5
#define PMU_USB_OTG_LSS_06	6
#define PMU_USB_HSIC_LSS_07	7
#define PMU_AUDIO_ENGINE_LSS_08	8
#define PMU_AUDIO_DMA_LSS_09	9
#define PMU_SRAM_LSS_10		10
#define PMU_SRAM_LSS_11		11
#define PMU_SRAM_LSS_12		12
#define PMU_SRAM_LSS_13		13
#define PMU_SDIO2_LSS_14	14
#define PMU_PTI_DAFCA_LSS_15	15
#define PMU_SC_DMA_LSS_16	16
#define PMU_SPIO_LSS_17		17
#define PMU_SPI1_LSS_18		18
#define PMU_SPI2_LSS_19		19
#define PMU_I2C0_LSS_20		20
#define PMU_I2C1_LSS_21		21
#define PMU_MAIN_FABRIC_LSS_22	22
#define PMU_SEC_FABRIC_LSS_23	23
#define PMU_SC_FABRIC_LSS_24	24
#define PMU_AUDIO_RAM_LSS_25	25
#define PMU_SCU_ROM_LSS_26	26
#define PMU_I2C2_LSS_27		27
#define PMU_SSC_LSS_28		28
#define PMU_SECURITY_LSS_29	29
#define PMU_SDIO1_LSS_30	30
#define PMU_SCU_RAM0_LSS_31	31
#define PMU_SCU_RAM1_LSS_32	32
#define PMU_I2C3_LSS_33		33
#define PMU_I2C4_LSS_34		34
#define PMU_I2C5_LSS_35		35
#define PMU_SPI3_LSS_36		36
#define PMU_GPIO1_LSS_37	37
#define PMU_PWR_BUTTON_LSS_38	38
#define PMU_GPIO0_LSS_39	39
#define PMU_KEYBRD_LSS_40	40
#define PMU_UART2_LSS_41	41
#define PMU_ADC_LSS_42	        42
#define PMU_CHARGER_LSS_43	43
#define PMU_SEC_TAPC_LSS_44	44
#define PMU_RTC_LSS_45		45
#define PMU_GPI_LSS_46		46
#define PMU_HDMI_VREG_LSS_47	47
#define PMU_RESERVED_LSS_48	48
#define PMU_AUDIO_SLIM1_LSS_49	49
#define PMU_RESET_LSS_50	50
#define PMU_AUDIO_SSP0_LSS_51	51
#define PMU_AUDIO_SSP1_LSS_52	52
#define PMU_IOSF_OCP_BRG_LSS_53	53
#define PMU_GP_DMA_LSS_54	54
#define PMU_SVID_LSS_55		55
#define PMU_SOC_FUSE_LSS_56	56
#define PMU_RSVD3_LSS_57	57
#define PMU_RSVD4_LSS_58	58
#define PMU_RSVD5_LSS_59	59
#define PMU_RSVD6_LSS_60	60
#define PMU_RSVD7_LSS_61	61
#define PMU_RSVD8_LSS_62	62
#define PMU_RSVD9_LSS_63	63

#define PMU_BASE_ADDR(pmu_num) ((pmu_num == 0) ? \
				(u32) base_addr.pmu1_base :\
				(u32) base_addr.pmu2_base);

#define SSMSK(mask, lss) ((mask) << ((lss) * 2))
#define SSWKC(lss) (1 << (lss))
#define S0IX_TARGET_SSS0_MASK ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I3_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I3_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_ENGINE_LSS_08) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_DMA_LSS_09) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define S0IX_TARGET_SSS1_MASK ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define S0IX_TARGET_SSS2_MASK ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I3_MASK, PMU_UART2_LSS_41-32))

#define S0IX_TARGET_SSS3_MASK ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48))

#define S0IX_TARGET_SSS0 ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I2_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I1_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_ENGINE_LSS_08) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_DMA_LSS_09) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define S0IX_TARGET_SSS1 ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define S0IX_TARGET_SSS2 ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I1_MASK, PMU_UART2_LSS_41-32))

#define S0IX_TARGET_SSS3 ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48))

#define LPMP3_TARGET_SSS0_MASK ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I3_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I3_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_ENGINE_LSS_08) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define LPMP3_TARGET_SSS1_MASK ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define LPMP3_TARGET_SSS2_MASK ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I3_MASK, PMU_UART2_LSS_41-32))

#define LPMP3_TARGET_SSS3_MASK ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48))

#define LPMP3_TARGET_SSS0 ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I2_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I1_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I0_MASK, PMU_AUDIO_ENGINE_LSS_08) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define LPMP3_TARGET_SSS1 ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define LPMP3_TARGET_SSS2 ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I1_MASK, PMU_UART2_LSS_41-32))

#define LPMP3_TARGET_SSS3 ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48))

#define IGNORE_SSS0 ( \
	SSMSK(D0I3_MASK, PMU_USB_HSIC_LSS_07) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_10) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_11) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_12) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_13) | \
	SSMSK(D0I3_MASK, PMU_PTI_DAFCA_LSS_15))

#define IGNORE_SSS1 ( \
	SSMSK(D0I3_MASK, PMU_SC_DMA_LSS_16-16) | \
	SSMSK(D0I3_MASK, PMU_SPIO_LSS_17-16) | \
	SSMSK(D0I3_MASK, PMU_MAIN_FABRIC_LSS_22-16) | \
	SSMSK(D0I3_MASK, PMU_SEC_FABRIC_LSS_23-16) | \
	SSMSK(D0I3_MASK, PMU_SC_FABRIC_LSS_24-16) | \
	SSMSK(D0I3_MASK, PMU_SCU_ROM_LSS_26-16) | \
	SSMSK(D0I3_MASK, PMU_SSC_LSS_28-16) | \
	SSMSK(D0I3_MASK, PMU_SECURITY_LSS_29-16) | \
	SSMSK(D0I3_MASK, PMU_SCU_RAM0_LSS_31-16))

#define IGNORE_SSS2 ( \
	SSMSK(D0I3_MASK, PMU_SCU_RAM1_LSS_32-32) | \
	SSMSK(D0I3_MASK, PMU_GPIO1_LSS_37-32) | \
	SSMSK(D0I3_MASK, PMU_PWR_BUTTON_LSS_38-32) | \
	SSMSK(D0I3_MASK, PMU_GPIO0_LSS_39-32) | \
	SSMSK(D0I3_MASK, PMU_ADC_LSS_42-32) | \
	SSMSK(D0I3_MASK, PMU_CHARGER_LSS_43-32) | \
	SSMSK(D0I3_MASK, PMU_SEC_TAPC_LSS_44-32) | \
	SSMSK(D0I3_MASK, PMU_RTC_LSS_45-32) | \
	SSMSK(D0I3_MASK, PMU_GPI_LSS_46-32) | \
	SSMSK(D0I3_MASK, PMU_HDMI_VREG_LSS_47-32))

#define IGNORE_SSS3 ( \
	SSMSK(D0I3_MASK, PMU_IOSF_OCP_BRG_LSS_53-48) | \
	SSMSK(D0I3_MASK, PMU_SVID_LSS_55-48) | \
	SSMSK(D0I3_MASK, PMU_SOC_FUSE_LSS_56-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD3_LSS_57-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD4_LSS_58-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD5_LSS_59-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD6_LSS_60-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD7_LSS_61-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD8_LSS_62-48) | \
	SSMSK(D0I3_MASK, PMU_RSVD9_LSS_63-48))

#define IGNORE_S3_WKC0 SSWKC(PMU_AONT_LSS_02)
#define IGNORE_S3_WKC1 SSWKC(PMU_ADC_LSS_42-32)

#define S0I3_SSS0 ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_AONT_LSS_02) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I2_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I1_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I1_MASK, PMU_USB_HSIC_LSS_07) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_ENGINE_LSS_08) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_DMA_LSS_09) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_12) | \
	SSMSK(D0I3_MASK, PMU_SRAM_LSS_13) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define S0I3_SSS1 ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_SPI2_LSS_19-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_RAM_LSS_25-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define S0I3_SSS2 ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I3_MASK, PMU_GPIO1_LSS_37-32) | \
	SSMSK(D0I3_MASK, PMU_PWR_BUTTON_LSS_38-32) | \
	SSMSK(D0I3_MASK, PMU_KEYBRD_LSS_40-32) | \
	SSMSK(D0I1_MASK, PMU_UART2_LSS_41-32))

#define S0I3_SSS3 ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SLIM1_LSS_49-48) | \
	SSMSK(D0I3_MASK, PMU_RESET_LSS_50-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48) | \
	SSMSK(D0I3_MASK, PMU_GP_DMA_LSS_54-48))

#define S0I1_SSS0 S0I3_SSS0
#define S0I1_SSS1 S0I3_SSS1
#define S0I1_SSS2 S0I3_SSS2
#define S0I1_SSS3 S0I3_SSS3

#define LPMP3_SSS0 ( \
	SSMSK(D0I3_MASK, PMU_SDIO0_LSS_00) | \
	SSMSK(D0I3_MASK, PMU_EMMC0_LSS_01) | \
	SSMSK(D0I3_MASK, PMU_AONT_LSS_02) | \
	SSMSK(D0I3_MASK, PMU_HSI_LSS_03) | \
	SSMSK(D0I2_MASK, PMU_SECURITY_LSS_04) | \
	SSMSK(D0I3_MASK, PMU_EMMC1_LSS_05) | \
	SSMSK(D0I1_MASK, PMU_USB_OTG_LSS_06) | \
	SSMSK(D0I1_MASK, PMU_USB_HSIC_LSS_07) | \
	SSMSK(D0I3_MASK, PMU_SDIO2_LSS_14))

#define LPMP3_SSS1 ( \
	SSMSK(D0I3_MASK, PMU_SPI1_LSS_18-16) | \
	SSMSK(D0I3_MASK, PMU_SPI2_LSS_19-16) | \
	SSMSK(D0I3_MASK, PMU_I2C0_LSS_20-16) | \
	SSMSK(D0I3_MASK, PMU_I2C1_LSS_21-16) | \
	SSMSK(D0I3_MASK, PMU_I2C2_LSS_27-16) | \
	SSMSK(D0I3_MASK, PMU_SDIO1_LSS_30-16))

#define LPMP3_SSS2 ( \
	SSMSK(D0I3_MASK, PMU_I2C3_LSS_33-32) | \
	SSMSK(D0I3_MASK, PMU_I2C4_LSS_34-32) | \
	SSMSK(D0I3_MASK, PMU_I2C5_LSS_35-32) | \
	SSMSK(D0I3_MASK, PMU_SPI3_LSS_36-32) | \
	SSMSK(D0I3_MASK, PMU_GPIO1_LSS_37-32) | \
	SSMSK(D0I3_MASK, PMU_PWR_BUTTON_LSS_38-32) | \
	SSMSK(D0I3_MASK, PMU_KEYBRD_LSS_40-32) | \
	SSMSK(D0I1_MASK, PMU_UART2_LSS_41-32))

#define LPMP3_SSS3 ( \
	SSMSK(D0I3_MASK, PMU_AUDIO_SLIM1_LSS_49-48) | \
	SSMSK(D0I3_MASK, PMU_RESET_LSS_50-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP0_LSS_51-48) | \
	SSMSK(D0I3_MASK, PMU_AUDIO_SSP1_LSS_52-48) | \
	SSMSK(D0I3_MASK, PMU_GP_DMA_LSS_54-48))


/* North Complex Power management */
#define OSPM_PUNIT_PORT         0x04
#define OSPM_OSPMBA             0x78
#define OSPM_PM_SSC             0x20
#define OSPM_PM_SSS             0x30

#define OSPM_APMBA              0x7a
#define APM_CMD                 0x0
#define APM_STS                 0x04

#define PCI_CMD_REG		0xD0
#define PCI_DATA_REG		0xD4
#define MSG_READ_CMD		0x10
#define MSG_WRITE_CMD		0x11

enum cm_trigger {
	NO_TRIG,		/* No trigger is required */
	LPM_EVENT_TRIG,		/* trigger on a LPM event */
	EXT_GPIO_INPUT_TRIG,	/* trigger on a GPIO input */
	C_STATE_TRANS_TRIG,	/* trigger on a C state transition */
	DMI_MSG_TRIG,		/* trigger on a C state transition */
	INVALID_TRIG
};

enum cm_mode {
	CM_NOP,			/* ignore the config mode value */
	CM_IMMEDIATE,		/*    */
	CM_DELAY,
	CM_TRIGGER,
	CM_INVALID
};

enum sys_state {
	SYS_STATE_S0I0,
	SYS_STATE_S0I1,
	SYS_STATE_S0I2,
	SYS_STATE_S0I3,
	SYS_STATE_S3,
	SYS_STATE_S5,
	SYS_STATE_MAX
};

enum pme_id {
	PME_ID_LPE = 1,		/* pme id for LPE */
	PME_ID_USB = 2		/* pme id for USB */
};

enum pm_cmd {
	SET_CFG_CMD = 1
};

enum int_status {
	INVALID_INT = 0,
	CMD_COMPLETE_INT = 1,
	CMD_ERROR_INT = 2,
	WAKE_RECEIVED_INT = 3,
	SUBSYS_POW_ERR_INT = 4,
	S0ix_MISS_INT = 5,
	NO_ACKC6_INT = 6,
	INVALID_SRC_INT
};

enum pmu_regs {
	PM_STATUS = 0,
	PM_CMD = 1,
	PM_INT_STATUS = 2,
	PM_WAKE_CFG = 3,
	PM_WAKE_STATUS = 4,
	PM_SUB_SYS_CFG = 5,
	PM_SUB_SYS_STATUS = 6,
	PM_WAKE_SUB_SYS_CFG = 7,
	PM_C3C4_CTR = 8,
	PM_C5C6_CTR = 9,
	PM_MSIC = 10,
	PM_INVALID
};

enum pmu_number {
	PMU_NUM_1,
	PMU_NUM_2,
	PMU_MAX_DEVS
};

enum pmu_ss_state {
	SS_STATE_D0I0 = 0,
	SS_STATE_D0I1 = 1,
	SS_STATE_D0I2 = 2,
	SS_STATE_D0I3 = 3
};

struct pci_dev_info {
	u8 ss_pos;
	u8 ss_idx;
	u8 pmu_num;

	u32 log_id;
	u32 cap;
	struct pci_dev *drv[PMU_MAX_LSS_SHARE];
	pci_power_t power_state[PMU_MAX_LSS_SHARE];
};

struct pmu_wake_ss_states {
	unsigned long wake_enable[2];
	unsigned long pmu1_wake_states;
	unsigned long pmu2_wake_states[4];
};

struct pmu_ss_states {
	unsigned long pmu1_states;
	unsigned long pmu2_states[4];
};

struct pmu_suspend_config {
	struct pmu_ss_states ss_state;
	struct pmu_wake_ss_states wake_state;
};

struct pci_dev_index {
	struct pci_dev	*pdev;
	u8		index;
};

/* PMU register interface */
struct mrst_pmu_reg {
	u32 pm_sts;             /* 0x00 */
	u32 pm_cmd;             /* 0x04 */
	u32 pm_ics;             /* 0x08 */
	u32 _resv1;
	u32 pm_wkc[2];          /* 0x10 */
	u32 pm_wks[2];          /* 0x18 */
	u32 pm_ssc[4];          /* 0x20 */
	u32 pm_sss[4];          /* 0x30 */
	u32 pm_wssc[4];         /* 0x40 */
	u32 pm_c3c4;            /* 0x50 */
	u32 pm_c5c6;            /* 0x54 */
	u32 pm_msic;            /* 0x58 */
};

/* PMU registers for pmu ( PMU2 /  PMU1) PMU2 --> LNG PMU1 --> LNC */
union pmu_pm_ss_cfg {
	struct {
		u32 ss_0:2;	/* cDmi  /  L1 power domain */
		u32 ss_1:2;	/* SD HC0 / Display */
		u32 ss_2:2;	/* SD HC1 / GFX */
		u32 ss_3:2;	/* Nand / Ved */
		u32 ss_4:2;	/* ISP Imaging / Vec */
		u32 ss_5:2;	/* Security / Mipi */
		u32 ss_6:2;	/* Display / LVDS */
		u32 ss_7:2;	/* USB Host */
		u32 ss_8:2;	/* USB OTG */
		u32 ss_9:2;	/* Audio */
		u32 ss_10:2;	/* Gpio */
		u32 ss_11:2;	/* Shared SRAM */
		u32 ss_12:2;	/*  rsvd */
		u32 ss_13:2;	/*  rsvd */
		u32 ss_14:2;	/*  rsvd */
		u32 ss_15:2;	/*  rsvd */
	} pmu_pm_ss_cfg_parts;
	u32 pmu_pm_ss_cfg_value;
};

union pmu_pm_ss_status {
	struct {
		u32 ss_0:2;	/* cDmi  /  L1 power domain  */
		u32 ss_1:2;	/* SD HC0 / Display */
		u32 ss_2:2;	/* SD HC1 / GFX */
		u32 ss_3:2;	/* Nand / Ved */
		u32 ss_4:2;	/* ISP Imaging / Vec */
		u32 ss_5:2;	/* Security / Mipi */
		u32 ss_6:2;	/* Display / LVDS */
		u32 ss_7:2;	/* USB Host */
		u32 ss_8:2;	/* USB OTG */
		u32 ss_9:2;	/* Audio */
		u32 ss_10:2;	/* Gpio */
		u32 ss_11:2;	/* Shared SRAM */
		u32 ss_12:2;	/*  rsvd */
		u32 ss_13:2;	/*  rsvd */
		u32 ss_14:2;	/*  rsvd */
		u32 ss_15:2;	/*  rsvd */
	} pmu_pm_ss_status_parts;
	u32 pmu_pm_ss_status_value;
};

union pmu_pm_status {
	struct {
		u32 pmu_rev:8;
		u32 pmu_busy:1;
		u32 mode_id:4;
		u32 Reserved:19;
	} pmu_status_parts;
	u32 pmu_status_value;
};

union pmu_pm_cmd {
	struct {
		u32 cmd:8;
		u32 ioc:1;
		u32 pmu_cmd_param:23;
	} pmu_pm_cmd_parts;
	u32 pmu_pm_cmd_value;
};

struct cfg_mode_params {
	u32 cfg_mode;
	u32 cfg_trigger;
	u32 cfg_trig_val;
	u32 cfg_delay;
	u32 cfg_cmbi;
};

/* pmu 1 pm set config parameters */
struct  pmu1_pm_set_cfg_cmd_parts {
	u32 cmd:8;
	u32 ioc:1;
	u32 cfg_mode:4;
	u32 cfg_delay:4;
	u32 cfg_trigger:4;
	u32 mode_id:4;
	u32 rsvd:7;
};

/* pmu 2 pm set config parameters */
struct cfg_delay_param_t {
	u32 cmd:8;
	u32 ioc:1;
	u32 cfg_mode:4;
	u32 mode_id:3;
	u32 sys_state:3;
	u32 cfg_delay:8;
	u32 rsvd:5;
};


struct cfg_trig_param_t {
	u32 cmd:8;
	u32 ioc:1;
	u32 cfg_mode:4;
	u32 mode_id:3;
	u32 sys_state:3;
	u32 cfg_trig_type:3;
	u32 cfg_trig_val:8;
	u32 cmbi:1;
	u32 rsvd1:1;
};

union pmu_pm_set_cfg_cmd_t {
	struct pmu1_pm_set_cfg_cmd_parts pmu1_params;
	union {
		struct cfg_delay_param_t d_param;
		struct cfg_trig_param_t t_param;
	} pmu2_params;
	u32 pmu_pm_set_cfg_cmd_value;
};

union pmu_pm_ics {
	struct {
		u32 int_status:8;
		u32 int_enable:1;
		u32 int_pend:1;
		u32 reserved:22;
	} pmu_pm_ics_parts;
	u32 pmu_pm_ics_value;
};

struct pmu_ospm_reg {
	union pmu_pm_status *pm_status;
	union pmu_pm_cmd *pm_cmd;
	union pmu_pm_ics *pm_ics;
	u32 pm_wkc;
	u32 rsvd1;
	u32 pm_wks;
	u32 rsvd2;
	union pmu_pm_ss_cfg *pm_ss_cfg;
	u32 rsvd3[3];
	union pmu_pm_ss_status *pm_ss_status;
	u32 rsvd4[3];
	u32 pm_wssc;
	u32 rsvd5[3];
	u32 pm_c3c4_ctr;
	u32 pm_c5c6_ctr;
	u32 pm_msic;
};

struct intel_mid_base_addr {
	u32 *pmu1_base;
	void __iomem *pmu2_base;
	u32 *pm_table_base;
	u32 __iomem *offload_reg;
};

struct mid_pmu_stats {
	u64 err_count[3];
	u64 count;
	u64 time;
	u64 last_entry;
	u64 last_try;
	u64 first_entry;
};

struct mid_pmu_dev {
	bool suspend_started;
	bool shutdown_started;

	u32 apm_base;
	u32 ospm_base;
	u32 pmu1_max_devs;
	u32 pmu2_max_devs;
	u32 ss_per_reg;
	u32 d0ix_stat[MAX_LSS_POSSIBLE][SS_STATE_D0I3+1];
	u32 num_wakes[MAX_DEVICES][SYS_STATE_MAX];

	u64 pmu_init_time;

	int cmd_error_int;
	int camera_off;
	int display_off;
	int s0ix_possible;
	int interactive_cmd_sent;
	int s0ix_entered;

	enum sys_state  pmu_current_state;

	struct pci_dev_info pci_devs[MAX_DEVICES];
	struct pci_dev_index
		pci_dev_hash[MID_PCI_INDEX_HASH_SIZE];
	struct intel_mid_base_addr base_addr;
	struct mrst_pmu_reg	__iomem *pmu_reg;
	struct semaphore scu_ready_sem;
	struct completion set_mode_complete;
	struct mid_pmu_stats pmu_stats[SYS_STATE_MAX];

#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock pmu_wake_lock;
#endif

	struct pmu_suspend_config *ss_config;
	struct pci_dev *pmu_dev;

	spinlock_t nc_ready_lock;
};

/* API used to change the platform mode */
extern int pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state);
extern pci_power_t pmu_pci_choose_state(struct pci_dev *pdev);
extern bool pmu_pci_power_manageable(struct pci_dev *pdev);
extern bool pmu_pci_can_wakeup(struct pci_dev *pdev);
extern int pmu_pci_sleep_wake(struct pci_dev *pdev, bool enable);
#endif
