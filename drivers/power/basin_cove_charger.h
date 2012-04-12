
/*
 * basin_cove_charger.h - Intel MID Basin Cove PMIC Charger Driver header file
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

#ifndef __BASIN_COVE_CHARGER_H_
#define __BASIN_COVE_CHARGER_H_


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


#define CHGRIRQ0_ADDR			0x07
#define CHGIRQ0_BZIRQ_MASK		D7
#define CHGIRQ0_BAT_CRIT_MASK		D6
#define CHGIRQ0_BAT1_ALRT_MASK		D5
#define CHGIRQ0_BAT0_ALRT_MASK		D4
#define CHGIRQ0_NACK_MASK		D3
#define CHGIRQ0_I2CRDP_CMP_MASK		D2
#define CHGIRQ0_I2CWR_CMP_MASK		D1
#define CHGIRQ0_CHG_INTB_MASK		D0

#define MCHGRIRQ0_ADDR			0x12
#define MCHGIRQ0_RSVD_MASK		D7
#define MCHGIRQ0_MBAT_CRIT_MASK		D6
#define MCHGIRQ0_MBAT1_ALRT_MASK	D5
#define MCHGIRQ0_MBAT0_ALRT_MASK	D4
#define MCHGIRQ0_MNACK_MASK		D3
#define MCHGIRQ0_MI2CRDP_CMP_MASK	D2
#define MCHGIRQ0_MI2CWR_CMP_MASK	D1
#define MCHGIRQ0_MCHG_INTB_MASK		D0

#define SCHGRIRQ0_ADDR			0x13
#define SCHGIRQ0_RSVD_MASK		D7
#define SCHGIRQ0_SBAT_CRIT_MASK		D6
#define SCHGIRQ0_SBAT1_ALRT_MASK	D5
#define SCHGIRQ0_SBAT0_ALRT_MASK	D4
#define SCHGIRQ0_SNACK_MASK		D3
#define SCHGIRQ0_SI2CRDP_CMP_MASK	D2
#define SCHGIRQ0_SI2CWR_CMP_MASK	D1
#define SCHGIRQ0_SCHG_INTB	D0

#define CHGRCTRL0_ADDR			0x4B
#define CHGRCTRL0_RSVD_MASK		(D7|D6|D5)
#define CHGRCTRL0_TTLCK_MASK		D4
#define CHGRCTRL0_SWCONTROL_MASK	D3
#define CHGRCTRL0_EXTCHRDIS_MASK	D2
#define	CHRCTRL0_EMRGCHREN_MASK		D1
#define	CHRCTRL0_CHGRRESET_MASK		D0

#define EXTCHRDIS_ENABLE		1
#define SWCONTROL_ENABLE		1

#define CHGRCTRL1_ADDR			0x4C
#define CHGRCTRL1_RSVD_MASK			(D7|D6)
#define CHGRCTRL1_FTEMP_EVENT_MASK		D5
#define CHGRCTRL1_FUSB_INLMT_1500_MASK		D4
#define CHGRCTRL1_FUSB_INLMT_900_MASK		D3
#define CHGRCTRL1_FUSB_INLMT_500_MASK		D2
#define CHGRCTRL1_FUSB_INLMT_150_MASK		D1
#define CHGRCTRL1_FUSB_INLMT_100_MASK		D0

#define CHGRSTATUS_ADDR				0x4D
#define CHGRSTATUS_RSVD_MASK			(D7|D6|D5|D4|D3|D2)
#define CHGRSTATUS_CHGDETB_LATCH_MASK		D1
#define CHGDETB_MASK				D0

#define I2COVRCTRL_ADDR		0x58
#define I2COVRDADDR_ADDR	0x59
#define I2COVROFFSET_ADDR	0x5A
#define I2COVRWRDATA_ADDR	0x5B
#define I2COVRRDDATA_ADDR	0x5C

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
#define TT_CHGRINIT0VAL_ADDR		0X16
#define TT_CHGRINIT1VAL_ADDR		0X17
#define TT_CHGRINIT2VAL_ADDR		0X18
#define TT_CHGRINIT3VAL_ADDR		0X19
#define TT_CHGRINIT4VAL_ADDR		0X1A
#define TT_CHGRINIT5VAL_ADDR		0X1B
#define TT_CHGRINIT6VAL_ADDR		0X1C
#define TT_CHGRINIT7VAL_ADDR		0X1D
#define TT_USBINPUTICC100VAL_ADDR	0X1E
#define TT_USBINPUTICC150VAL_ADDR	0X1F
#define TT_USBINPUTICC500VAL_ADDR	0X20
#define TT_USBINPUTICC900VAL_ADDR	0X21
#define TT_USBINPUTICC1500VAL_ADDR	0X22
#define TT_CHRCVEMRGLOWVAL_ADDR		0X23
#define TT_CHRCVCOLDVAL_ADDR		0X24
#define TT_CHRCVCOOLVAL_ADDR		0X25
#define TT_CHRCVWARMVAL_ADDR		0X26
#define TT_CHRCVHOTVAL_ADDR		0X27
#define TT_CHRCVEMRGHIVAL_ADDR		0X28
#define TT_CHRCCEMRGLOWVAL_ADDR		0X29
#define TT_CHRCCCOLDVAL_ADDR		0X2A
#define TT_CHRCCOOLVAL_ADDR		0X2B
#define TT_CHRCCWARMVAL_ADDR		0X2C
#define TT_CHRCCHOTVAL_ADDR		0X2D
#define TT_CHRCCEMRGHIVAL_ADDR		0X2E
#define TT_LOWCHRENVAL_ADDR		0X2F
#define TT_LOWCHRDISVAL_ADDR		0X30
#define TT_WDOGRSTVAL_ADDR		0X31
#define TT_CHGRENVAL_ADDR		0X32
#define TT_CHGRDISVAL_ADDR		0X33

/*Interrupt registers*/
#define BATT_CHR_BATTDET_MASK	D2
/*Status registers*/
#define SCHGRIRQ1_ADDR		0x4F
#define BATT_PRESENT		1
#define BATT_NOT_PRESENT	0

#define CHARGER_PRESENT		1
#define CHARGER_NOT_PRESENT	0
/*FIXME: Modify default values */
#define BATT_DEAD_CUTOFF_VOLT		3400	/* 3400 mV */
#define BATT_CRIT_CUTOFF_VOLT		3700	/* 3700 mV */

#define MSIC_BATT_TEMP_MAX		60	/* 60 degrees */
#define MSIC_BATT_TEMP_MIN		0

/* Bit definitions */


struct bc_batt_props_cxt {
	unsigned int status;
	unsigned int health;
	bool present;
};
struct bc_charger_props_cxt {
	unsigned int charging_mode;
	unsigned int charger_present;
	unsigned int charger_health;
	unsigned int vbus_vol;
	char charger_model[BATT_STRING_MAX];
	char charger_vender[BATT_STRING_MAX];
};


/*
 * basinc cove charger driver info
 */
struct bc_chrgr_drv_context {

	struct platform_device *pdev;
	bool invalid_batt;
	bool is_batt_present;
	struct batt_charging_profile *chrg_profile;
	struct plat_battery_config *batt_config;

	/* lock to protect the charger properties
	 * locking is applied wherever read or write
	 * operation is being performed to the
	 * charger property structure.
	 */

	struct mutex bc_chrgr_lock;

	struct bc_charger_props_cxt chrgr_props_cxt;
	struct power_supply bc_chrgr_ps;

	unsigned int irq;		/* GPE_ID or IRQ# */

	/* bc battery data */
	/* lock to protect battery  properties
	* locking is applied wherever read or write
	* operation is being performed to the battery
	* property structure.
	*/
	struct mutex batt_lock;
	struct bc_batt_props_cxt batt_props_cxt;

	/* lock to avoid concurrent  access to HW Registers.
	 * As some charger control and parameter registers
	 * can be read or write at same time, bc_ipc_rw_lock lock
	 * is used to synchronize those Charger IPC read or write calls.
	 */
	struct mutex bc_ipc_rw_lock;
	/* Worker to handle otg callback events */
	struct delayed_work chrg_callback_dwrk;
	bool current_sense_enabled;

};

extern bool bc_is_current_sense_enabled(void);
extern bool bc_check_battery_present(void);
extern int bc_check_battery_health(void);
extern int bc_check_battery_status(void);
extern int bc_get_battery_pack_temp(int *val);

#endif
