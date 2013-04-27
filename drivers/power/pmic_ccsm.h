/*
 * pmic_ccsm.h - Intel MID PMIC CCSM Driver header file
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jenny TC <jenny.tc@intel.com>
 */

#ifndef __PMIC_CCSM_H__
#define __PMIC_CCSM_H__

#include <asm/pmic_pdata.h>
/*********************************************************************
 *		Generic defines
 *********************************************************************/

#define D7 (1 << 7)
#define D6 (1 << 6)
#define D5 (1 << 5)
#define D4 (1 << 4)
#define D3 (1 << 3)
#define D2 (1 << 2)
#define D1 (1 << 1)
#define D0 (1 << 0)

#define PMIC_BZONE_LOW 0
#define PMIC_BZONE_HIGH 5

#define IRQLVL1_ADDR			0x01
#define IRQLVL1_MASK_ADDR		0x0c
#define IRQLVL1_CHRGR_MASK		D5

#define THRMZN0H_ADDR			0xCE
#define THRMZN0L_ADDR			0xCF
#define THRMZN1H_ADDR			0xD0
#define THRMZN1L_ADDR			0xD1
#define THRMZN2H_ADDR			0xD2
#define THRMZN2L_ADDR			0xD3
#define THRMZN3H_ADDR			0xD4
#define THRMZN3L_ADDR			0xD5
#define THRMZN4H_ADDR			0xD6
#define THRMZN4L_ADDR			0xD7

#define CHGRIRQ0_ADDR			0x07
#define CHGIRQ0_BZIRQ_MASK		D7
#define CHGIRQ0_BAT_CRIT_MASK		D6
#define CHGIRQ0_BAT1_ALRT_MASK		D5
#define CHGIRQ0_BAT0_ALRT_MASK		D4

#define MCHGRIRQ0_ADDR			0x12
#define MCHGIRQ0_RSVD_MASK		D7
#define MCHGIRQ0_MBAT_CRIT_MASK		D6
#define MCHGIRQ0_MBAT1_ALRT_MASK	D5
#define MCHGIRQ0_MBAT0_ALRT_MASK	D4

#define SCHGRIRQ0_ADDR			0x4E
#define SCHGIRQ0_RSVD_MASK		D7
#define SCHGIRQ0_SBAT_CRIT_MASK		D6
#define SCHGIRQ0_SBAT1_ALRT_MASK	D5
#define SCHGIRQ0_SBAT0_ALRT_MASK	D4

#define LOWBATTDET0_ADDR		0x2C
#define LOWBATTDET1_ADDR		0x2D
#define BATTDETCTRL_ADDR		0x2E
#define VBUSDETCTRL_ADDR		0x50
#define VDCINDETCTRL_ADDR		0x51

#define CHRGRIRQ1_ADDR			0x08
#define CHRGRIRQ1_SUSBIDDET_MASK	D3
#define CHRGRIRQ1_SBATTDET_MASK		D2
#define CHRGRIRQ1_SDCDET_MASK		D1
#define CHRGRIRQ1_SVBUSDET_MASK		D0
#define MCHGRIRQ1_ADDR			0x13
#define MCHRGRIRQ1_SUSBIDDET_MASK	D3
#define MCHRGRIRQ1_SBATTDET_MAS		D2
#define MCHRGRIRQ1_SDCDET_MASK		D1
#define MCHRGRIRQ1_SVBUSDET_MASK	D0
#define SCHGRIRQ1_ADDR			0x4F
#define SCHRGRIRQ1_SUSBIDDET_MASK	D3
#define SCHRGRIRQ1_SBATTDET_MASK	D2
#define SCHRGRIRQ1_SDCDET_MASK		D1
#define SCHRGRIRQ1_SVBUSDET_MASK	D0

#define PMIC_CHRGR_INT0_MASK		0xB1
#define PMIC_CHRGR_CCSM_INT0_MASK		0xB0
#define PMIC_CHRGR_EXT_CHRGR_INT_MASK	0x01

#define CHGRCTRL0_ADDR			0x4B
#define CHGRCTRL0_RSVD_MASK		(D7|D6|D5)
#define CHGRCTRL0_TTLCK_MASK		D4
#define CHGRCTRL0_SWCONTROL_MASK	D3
#define CHGRCTRL0_EXTCHRDIS_MASK	D2
#define	CHRCTRL0_EMRGCHREN_MASK		D1
#define	CHRCTRL0_CHGRRESET_MASK		D0

#define EXTCHRDIS_ENABLE		(0x01 << 2)
#define EXTCHRDIS_DISABLE		(~EXTCHRDIS_ENABLE & 0xFF)
#define SWCONTROL_ENABLE		(0x01 << 3)
#define EMRGCHREN_ENABLE		(0x01 << 1)

#define CHGRCTRL1_ADDR			0x4C
#define CHGRCTRL1_RSVD_MASK			(D7|D6)
#define CHGRCTRL1_FTEMP_EVENT_MASK		D5
#define CHGRCTRL1_FUSB_INLMT_1500		D4
#define CHGRCTRL1_FUSB_INLMT_900		D3
#define CHGRCTRL1_FUSB_INLMT_500		D2
#define CHGRCTRL1_FUSB_INLMT_150		D1
#define CHGRCTRL1_FUSB_INLMT_100		D0

#define CHGRSTATUS_ADDR				0x4D
#define CHGRSTATUS_RSVD_MASK			(D7|D6|D5|D4|D3|D2)
#define CHGRSTATUS_CHGDETB_LATCH_MASK		D1
#define CHGDETB_MASK				D0

#define THRMBATZONE_ADDR			0xB5
#define THRMBATZONE_MASK			(D0|D1|D2)

#define USBIDCTRL_ADDR		0x19
#define USBIDEN_MASK		0x01
#define ACADETEN_MASK		(0x01 << 1)

#define USBIDSTAT_ADDR		0x1A
#define WAKESRC_ADDR		0x24

#define CHRTTADDR_ADDR		0x56
#define CHRTTDATA_ADDR		0x57

#define TT_I2CDADDR_ADDR		0x00
#define TT_CHGRINIT0OS_ADDR		0x01
#define TT_CHGRINIT1OS_ADDR		0x02
#define TT_CHGRINIT2OS_ADDR		0x03
#define TT_CHGRINIT3OS_ADDR		0x04
#define TT_CHGRINIT4OS_ADDR		0x05
#define TT_CHGRINIT5OS_ADDR		0x06
#define TT_CHGRINIT6OS_ADDR		0x07
#define TT_CHGRINIT7OS_ADDR		0x08
#define TT_USBINPUTICCOS_ADDR		0x09
#define TT_USBINPUTICCMASK_ADDR		0x0A
#define TT_CHRCVOS_ADDR			0X0B
#define TT_CHRCVMASK_ADDR		0X0C
#define TT_CHRCCOS_ADDR			0X0D
#define TT_CHRCCMASK_ADDR		0X0E
#define TT_LOWCHROS_ADDR		0X0F
#define TT_LOWCHRMASK_ADDR		0X10
#define TT_WDOGRSTOS_ADDR		0X11
#define TT_WDOGRSTMASK_ADDR		0X12
#define TT_CHGRENOS_ADDR		0X13
#define TT_CHGRENMASK_ADDR		0X14

#define TT_CUSTOMFIELDEN_ADDR		0X15
#define TT_HOT_LC_EN			D1
#define TT_COLD_LC_EN			D0
#define TT_HOT_COLD_LC_MASK		(TT_HOT_LC_EN | TT_COLD_LC_EN)
#define TT_HOT_COLD_LC_EN		(TT_HOT_LC_EN | TT_COLD_LC_EN)
#define TT_HOT_COLD_LC_DIS		0

#define TT_CHGRINIT0VAL_ADDR		0X20
#define TT_CHGRINIT1VAL_ADDR		0X21
#define TT_CHGRINIT2VAL_ADDR		0X22
#define TT_CHGRINIT3VAL_ADDR		0X23
#define TT_CHGRINIT4VAL_ADDR		0X24
#define TT_CHGRINIT5VAL_ADDR		0X25
#define TT_CHGRINIT6VAL_ADDR		0X26
#define TT_CHGRINIT7VAL_ADDR		0X27
#define TT_USBINPUTICC100VAL_ADDR	0X28
#define TT_USBINPUTICC150VAL_ADDR	0X29
#define TT_USBINPUTICC500VAL_ADDR	0X2A
#define TT_USBINPUTICC900VAL_ADDR	0X2B
#define TT_USBINPUTICC1500VAL_ADDR	0X2C
#define TT_CHRCVEMRGLOWVAL_ADDR		0X2D
#define TT_CHRCVCOLDVAL_ADDR		0X2E
#define TT_CHRCVCOOLVAL_ADDR		0X2F
#define TT_CHRCVWARMVAL_ADDR		0X30
#define TT_CHRCVHOTVAL_ADDR		0X31
#define TT_CHRCVEMRGHIVAL_ADDR		0X32
#define TT_CHRCCEMRGLOWVAL_ADDR		0X33
#define TT_CHRCCCOLDVAL_ADDR		0X34
#define TT_CHRCCCOOLVAL_ADDR		0X35
#define TT_CHRCCWARMVAL_ADDR		0X36
#define TT_CHRCCHOTVAL_ADDR		0X37
#define TT_CHRCCEMRGHIVAL_ADDR		0X38
#define TT_LOWCHRENVAL_ADDR		0X39
#define TT_LOWCHRDISVAL_ADDR		0X3A
#define TT_WDOGRSTVAL_ADDR		0X3B
#define TT_CHGRENVAL_ADDR		0X3C
#define TT_CHGRDISVAL_ADDR		0X3D

/*Interrupt registers*/
#define BATT_CHR_BATTDET_MASK	D2
/*Status registers*/
#define BATT_PRESENT		1
#define BATT_NOT_PRESENT	0

#define BATT_STRING_MAX		8
#define BATTID_STR_LEN		8

#define CHARGER_PRESENT		1
#define CHARGER_NOT_PRESENT	0

/*FIXME: Modify default values */
#define BATT_DEAD_CUTOFF_VOLT		3400	/* 3400 mV */
#define BATT_CRIT_CUTOFF_VOLT		3700	/* 3700 mV */

#define MSIC_BATT_TEMP_MAX		60	/* 60 degrees */
#define MSIC_BATT_TEMP_MIN		0

#define BATT_TEMP_WARM			45	/* 45 degrees */
#define MIN_BATT_PROF			4

struct interrupt_info {
	/* Interrupt register mask*/
	u8 int_reg_mask;
	/* interrupt status register mask */
	u8 stat_reg_mask;
	/* log message if interrupt is set */
	char *log_msg_int_reg_true;
	/* log message if stat is true or false */
	char *log_msg_stat_true;
	char *log_msg_stat_false;
	/* handle if interrupt bit is set */
	void (*int_handle) (void);
	/* interrupt status handler */
	void (*stat_handle) (bool);
};

struct pmic_chrgr_drv_context {
	bool invalid_batt;
	bool is_batt_present;
	bool current_sense_enabled;
	unsigned int irq;		/* GPE_ID or IRQ# */
	void __iomem *pmic_intr_iomap;
	struct device *dev;
	int health;
	struct ps_batt_chg_prof *sfi_bcprof;
	struct ps_pse_mod_prof *actual_bcprof;
	struct ps_pse_mod_prof *runtime_bcprof;
	struct pmic_platform_data *pdata;
	struct usb_phy *otg;
	struct list_head evt_queue;
	struct work_struct evt_work;
	struct mutex evt_queue_lock;
};

struct pmic_event {
	struct list_head node;
	u8 chgrirq0_int;
	u8 chgrirq1_int;
	u8 chgrirq0_stat;
	u8 chgrirq1_stat;
};

struct temp_lookup {
	int adc_val;
	int temp;
	int temp_err;
};

#endif
