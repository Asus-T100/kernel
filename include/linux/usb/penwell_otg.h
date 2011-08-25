/*
 * Intel Penwell USB OTG transceiver driver
 * Copyright (C) 2009 - 2010, Intel Corporation.
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

#ifndef __PENWELL_OTG_H__
#define __PENWELL_OTG_H__

#include <linux/usb/intel_mid_otg.h>

#define CI_USBCMD		0x30
#	define USBCMD_RST		BIT(1)
#	define USBCMD_RS		BIT(0)
#define CI_USBSTS		0x34
#	define USBSTS_SLI		BIT(8)
#	define USBSTS_URI		BIT(6)
#	define USBSTS_PCI		BIT(2)
#define CI_ULPIVP		0x60
#	define ULPI_WU			BIT(31)
#	define ULPI_RUN			BIT(30)
#	define ULPI_RW			BIT(29)
#	define ULPI_SS			BIT(27)
#	define ULPI_PORT		(BIT(26) | BIT(25) | BIT(24))
#	define ULPI_ADDR		(0xff << 16)
#	define ULPI_DATRD		(0xff << 8)
#	define ULPI_DATWR		(0xff << 0)
#define CI_PORTSC1		0x74
#	define PORTSC_PP		BIT(12)
#	define PORTSC_LS		(BIT(11) | BIT(10))
#	define PORTSC_SUSP		BIT(7)
#	define PORTSC_CCS		BIT(0)
#define CI_HOSTPC1		0xb4
#	define HOSTPC1_PHCD		BIT(22)
#define CI_OTGSC		0xf4
#	define OTGSC_DPIE		BIT(30)
#	define OTGSC_1MSE		BIT(29)
#	define OTGSC_BSEIE		BIT(28)
#	define OTGSC_BSVIE		BIT(27)
#	define OTGSC_ASVIE		BIT(26)
#	define OTGSC_AVVIE		BIT(25)
#	define OTGSC_IDIE		BIT(24)
#	define OTGSC_DPIS		BIT(22)
#	define OTGSC_1MSS		BIT(21)
#	define OTGSC_BSEIS		BIT(20)
#	define OTGSC_BSVIS		BIT(19)
#	define OTGSC_ASVIS		BIT(18)
#	define OTGSC_AVVIS		BIT(17)
#	define OTGSC_IDIS		BIT(16)
#	define OTGSC_DPS		BIT(14)
#	define OTGSC_1MST		BIT(13)
#	define OTGSC_BSE		BIT(12)
#	define OTGSC_BSV		BIT(11)
#	define OTGSC_ASV		BIT(10)
#	define OTGSC_AVV		BIT(9)
#	define OTGSC_ID			BIT(8)
#	define OTGSC_HABA		BIT(7)
#	define OTGSC_HADP		BIT(6)
#	define OTGSC_IDPU		BIT(5)
#	define OTGSC_DP			BIT(4)
#	define OTGSC_OT			BIT(3)
#	define OTGSC_HAAR		BIT(2)
#	define OTGSC_VC			BIT(1)
#	define OTGSC_VD			BIT(0)
#define CI_USBMODE		0xf8
#	define USBMODE_CM		(BIT(1) | BIT(0))
#	define USBMODE_IDLE		0
#	define USBMODE_DEVICE		0x2
#	define USBMODE_HOST		0x3
#define USBCFG_ADDR			0xff10801c
#define USBCFG_LEN			4
#	define USBCFG_VBUSVAL		BIT(14)
#	define USBCFG_AVALID		BIT(13)
#	define USBCFG_BVALID		BIT(12)
#	define USBCFG_SESEND		BIT(11)

#define OTGSC_INTEN_MASK \
	(OTGSC_DPIE | OTGSC_BSEIE | OTGSC_BSVIE \
	| OTGSC_ASVIE | OTGSC_AVVIE | OTGSC_IDIE)

#define OTGSC_INTSTS_MASK \
	(OTGSC_DPIS | OTGSC_BSEIS | OTGSC_BSVIS \
	| OTGSC_ASVIS | OTGSC_AVVIS | OTGSC_IDIS)

#define INTR_DUMMY_MASK (USBSTS_SLI | USBSTS_URI | USBSTS_PCI)

#define HOST_REQUEST_FLAG		BIT(0)

/* MSIC register for vbus power control */
#define MSIC_ID			0x00
#	define ID0_VENDID0		(BIT(7) | BIT(6))
#define MSIC_ID1		0x01
#	define ID1_VENDID1		(BIT(7) | BIT(6))
#define MSIC_VUSB330CNT		0xd4
#define MSIC_VOTGCNT		0xdf
#	define VOTGEN			BIT(7)
#	define VOTGRAMP			BIT(4)
#define MSIC_SPWRSRINT1		0x193
#	define SUSBCHPDET		BIT(6)
#	define SUSBDCDET		BIT(2)
#	define MSIC_SPWRSRINT1_MASK	(BIT(6) | BIT(2))
#	define SPWRSRINT1_CHRG_PORT	BIT(6)
#	define SPWRSRINT1_HOST_PORT	0
#	define SPWRSRINT1_DEDT_CHRG	(BIT(6) | BIT(2))
#define MSIC_IS4SET		0x2c8	/* Intel Specific */
#	define IS4_CHGDSERXDPINV	BIT(5)
#define MSIC_OTGCTRLSET		0x340
#define MSIC_OTGCTRLCLR		0x341
#	define DMPULLDOWNCLR		BIT(2)
#	define DPPULLDOWNCLR		BIT(1)
#define MSIC_PWRCTRLSET		0x342
#	define DPWKPUENSET		BIT(4)
#	define SWCNTRLSET		BIT(0)
#define MSIC_PWRCTRLCLR		0x343
#	define DPVSRCENCLR		BIT(6)
#	define SWCNTRLCLR		BIT(0)
#define MSIC_FUNCTRLSET		0x344
#	define OPMODESET0		BIT(3)
#define MSIC_FUNCTRLCLR		0x345
#	define OPMODECLR1		BIT(4)
#define MSIC_VS3SET		0x346	/* Vendor Specific */
#	define SWUSBDETSET		BIT(4)
#	define DATACONENSET		BIT(3)
#define MSIC_VS3CLR		0x347
#	define SWUSBDETCLR		BIT(4)
#	define DATACONENCLR		BIT(3)
#define MSIC_ULPIACCESSMODE	0x348
#	define SPIMODE			BIT(0)

/* MSIC TI implementation for ADP/ACA */
#define ULPI_TI_VS2		0x83
#	define TI_ID_FLOAT_STS		BIT(4)
#	define TI_ID_RARBRC_STS(d)	(((d)>>2)&3)
#	define TI_ID_RARBRC_STS_MASK	(BIT(3) | BIT(2))
#	define TI_ID_RARBRC_NONE	0
#	define TI_ID_RARBRC_A		1
#	define TI_ID_RARBRC_B		2
#	define TI_ID_RARBRC_C		3
#	define TI_ADP_INT_STS		BIT(1)
#define ULPI_TI_VS4		0x88
#	define TI_ACA_DET_EN		BIT(6)
#define ULPI_TI_VS5		0x8b
#	define TI_ADP_INT_EN		BIT(7)
#	define TI_ID_FLOAT_EN		BIT(5)
#	define TI_ID_RES_EN		BIT(4)
#define ULPI_TI_VS6		0x8e
#	define TI_HS_TXPREN		BIT(4)
#	define TI_ADP_MODE(d)		(((d)>>2)&3)
#	define TI_ADP_MODE_MASK		(BIT(3) | BIT(2))
#	define TI_ADP_MODE_DISABLE	0
#	define TI_ADP_MODE_SENSE	1
#	define TI_ADP_MODE_PRB_A	2
#	define TI_ADP_MODE_PRB_B	3
#	define TI_VBUS_IADP_SRC		BIT(1)
#	define TI_VBUS_IADP_SINK	BIT(0)
#define ULPI_TI_VS7		0x91
#	define TI_T_ADP_HIGH		(0xff)
#define ULPI_TI_VS8		0x94
#	define TI_T_ADP_LOW		(0xff)
#define ULPI_TI_VS9		0x97
#	define TI_T_ADP_RISE		(0xff)

#define TI_PRB_DELTA			0x08

/* MSIC FreeScale Implementation for ADP */
#define ULPI_FS_ADPCL		0x28
#	define ADPCL_PRBDSCHG		(BIT(5) | BIT(6))
#	define ADPCL_PRBDSCHG_4		0
#	define ADPCL_PRBDSCHG_8		1
#	define ADPCL_PRBDSCHG_16	2
#	define ADPCL_PRBDSCHG_32	3
#	define ADPCL_PRBPRD		(BIT(3) | BIT(4))
#	define ADPCL_PRBPRD_A_HALF	0
#	define ADPCL_PRBPRD_B_HALF	1
#	define ADPCL_PRBPRD_A		2
#	define ADPCL_PRBPRD_B		3
#	define ADPCL_SNSEN		BIT(2)
#	define ADPCL_PRBEN		BIT(1)
#	define ADPCL_ADPEN		BIT(0)
#define ULPI_FS_ADPCH		0x29
#	define ADPCH_PRBDELTA		(0x1f << 0)
#define ULPI_FS_ADPIE		0x2a
#	define ADPIE_ADPRAMPIE		BIT(2)
#	define ADPIE_SNSMISSIE		BIT(1)
#	define ADPIE_PRBTRGIE		BIT(0)
#define ULPI_FS_ADPIS		0x2b
#	define ADPIS_ADPRAMPS		BIT(5)
#	define ADPIS_SNSMISSS		BIT(4)
#	define ADPIS_PRBTRGS		BIT(3)
#	define ADPIS_ADPRAMPI		BIT(2)
#	define ADPIS_SNSMISSI		BIT(1)
#	define ADPIS_PRBTRGI		BIT(0)
#define ULPI_FS_ADPRL		0x2c
#	define ADPRL_ADPRAMP		(0xff << 0)
#define ULPI_FS_ADPRH		0x2d
#	define ADPRH_ADPRAMP		(0x7 << 0)

#define FS_ADPI_MASK	(ADPIS_ADPRAMPI | ADPIS_SNSMISSI | ADPIS_PRBTRGI)

enum penwell_otg_timer_type {
	TA_WAIT_VRISE_TMR,
	TA_WAIT_BCON_TMR,
	TA_AIDL_BDIS_TMR,
	TA_BIDL_ADIS_TMR,
	TA_WAIT_VFALL_TMR,
	TB_ASE0_BRST_TMR,
	TB_SE0_SRP_TMR,
	TB_SRP_FAIL_TMR, /* wait for response of SRP */
	TB_BUS_SUSPEND_TMR
};

#define TA_WAIT_VRISE		100
#define TA_WAIT_BCON		30000
#define TA_AIDL_BDIS		1500
#define TA_BIDL_ADIS		300
#define TA_WAIT_VFALL		950
#define TB_ASE0_BRST		300
#define TB_SE0_SRP		1800
#define TB_SSEND_SRP		1800
#	define SRP_MON_INVAL	200
#define TB_SRP_FAIL		5500
#define TB_BUS_SUSPEND		500
#define THOS_REQ_POL		1500

/* MSIC vendor information */
enum msic_vendor {
	MSIC_VD_FS,
	MSIC_VD_TI,
	MSIC_VD_UNKNOWN
};

struct adp_status {
	struct completion	adp_comp;
	u8			t_adp_rise;
};

struct penwell_otg {
	struct intel_mid_otg_xceiv	iotg;
	struct device			*dev;

	unsigned			region;
	unsigned			cfg_region;

	struct work_struct		work;
	struct work_struct		hnp_poll_work;
	struct workqueue_struct		*qwork;

	struct timer_list		hsm_timer;
	struct timer_list		hnp_poll_timer;

	enum msic_vendor		msic;

	struct notifier_block		iotg_notifier;

	struct adp_status		adp;
};

static inline
struct penwell_otg *iotg_to_penwell(struct intel_mid_otg_xceiv *iotg)
{
	return container_of(iotg, struct penwell_otg, iotg);
}

#endif /* __PENWELL_OTG_H__ */
