
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



static void bcove_extchrgr_read_complete(bool);
static void bcove_extchrgr_write_complete(bool);
static void bcove_bat_zone_changed(void);
static void bcove_battery_overheat_handler(bool);
static void bcove_handle_ext_chrgr_irq(bool);


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

#define BCOVE_BZONE_LOW 0
#define BCOVE_BZONE_HIGH 5

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

#define SCHGRIRQ0_ADDR			0x4E
#define SCHGIRQ0_RSVD_MASK		D7
#define SCHGIRQ0_SBAT_CRIT_MASK		D6
#define SCHGIRQ0_SBAT1_ALRT_MASK	D5
#define SCHGIRQ0_SBAT0_ALRT_MASK	D4
#define SCHGIRQ0_SNACK_MASK		D3
#define SCHGIRQ0_SI2CRD_CMP_MASK	D2
#define SCHGIRQ0_SI2CWR_CMP_MASK	D1
#define SCHGIRQ0_SCHG_INTB	D0

#define LOWBATTDET0_ADDR		0x2C
#define LOWBATTDET1_ADDR		0x2D
#define BATTDETCTRL_ADDR		0x2E
#define VBUSDETCTRL_ADDR		0x50
#define VDCINDETCTRL_ADDR		0x51
#define CHRGRIRQ1_ADDR			0x08
#define MCHGRIRQ1_ADDR			0x13

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
#define CHGRCTRL1_FUSB_INLMT_1500_MASK		D4
#define CHGRCTRL1_FUSB_INLMT_900_MASK		D3
#define CHGRCTRL1_FUSB_INLMT_500_MASK		D2
#define CHGRCTRL1_FUSB_INLMT_150_MASK		D1
#define CHGRCTRL1_FUSB_INLMT_100_MASK		D0

#define CHGRSTATUS_ADDR				0x4D
#define CHGRSTATUS_RSVD_MASK			(D7|D6|D5|D4|D3|D2)
#define CHGRSTATUS_CHGDETB_LATCH_MASK		D1
#define CHGDETB_MASK				D0

#define THRMBATZONE_ADDR			0xB5
#define THRMBATZONE_MASK			(D0|D1|D2)

#define I2COVRCTRL_ADDR		0x58
#define I2COVRDADDR_ADDR	0x59
#define I2COVROFFSET_ADDR	0x5A
#define I2COVRWRDATA_ADDR	0x5B
#define I2COVRRDDATA_ADDR	0x5C

#define I2COVRCTRL_I2C_RD D1
#define I2COVRCTRL_I2C_WR D0

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
#define TT_CHRCCCOOLVAL_ADDR		0X2B
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

/* BQ24260 registers */
#define BQ24260_STAT_CTRL0_ADDR		0x00

#define BQ24260_FAULT_MASK		0x07
#define BQ24260_STAT_MASK		(0x03 << 4)
#define BQ24260_BOOST_MASK		(0x01 << 6)
#define BQ24260_TMR_RST_MASK		(0x01 << 7)

#define BQ24260_VOVP			0x01
#define BQ24260_LOW_SUPPLY		0x02
#define BQ24260_THERMAL_SHUTDOWN	0x03
#define BQ24260_BATT_TEMP_FAULT		0x04
#define BQ24260_TIMER_FAULT		0x05
#define BQ24260_BATT_OVP		0x06
#define BQ24260_NO_BATTERY		0x07
#define BQ24260_STAT_READY		0x00

#define BQ24260_STAT_CHRG_PRGRSS	(0x01 << 4)
#define BQ24260_STAT_CHRG_DONE		(0x02 << 4)
#define BQ24260_STAT_FAULT		(0x03 << 4)

#define BQ24260_CTRL_ADDR		0x01
#define BQ24260_CE_MASK			D1

#define BQ24260_CE_DISABLE		(0x01 << 1)
#define BQ24260_CE_ENABLE		0x00

#define BQ24260_BATT_VOL_CTRL_ADDR	0x02
#define BQ24260_VENDOR_REV_ADDR		0x03
#define BQ24260_TERM_FCC_ADDR		0x04
#define BQ24260_VINDPM_DPPM_STATUS_ADDR	0x05
#define BQ24260_ST_NTC_MON_ADDR		0x06

#define BATT_STRING_MAX		8
#define BATTID_STR_LEN		8

#define CHARGER_PRESENT		1
#define CHARGER_NOT_PRESENT	0

/*FIXME: Modify default values */
#define BATT_DEAD_CUTOFF_VOLT		3400	/* 3400 mV */
#define BATT_CRIT_CUTOFF_VOLT		3700	/* 3700 mV */

#define MSIC_BATT_TEMP_MAX		60	/* 60 degrees */
#define MSIC_BATT_TEMP_MIN		0

#define BQ24260_ICHRG_100mA		(0x01 << 3)
#define BQ24260_ICHRG_200mA		(0x01 << 4)
#define BQ24260_ICHRG_400mA		(0x01 << 5)
#define BQ24260_ICHRG_800mA		(0x01 << 6)
#define BQ24260_ICHRG_1600mA		(0x01 << 7)

#define BQ24260_VBREG_20mV		(0x01 << 2)
#define BQ24260_VBREG_40mV		(0x01 << 3)
#define BQ24260_VBREG_80mV		(0x01 << 4)
#define BQ24260_VBREG_160mV		(0x01 << 5)
#define BQ24260_VBREG_320mV		(0x01 << 6)
#define BQ24260_VBREG_640mV		(0x01 << 7)


enum {
	I2C_RD = 1,
	I2C_WR
};


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
	char charger_model[BATT_STRING_MAX];
	char charger_vender[BATT_STRING_MAX];
};

enum extern_charger {
		BQ24260,
};

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

struct ext_charger {
	int (*enable_charging)(u8 dev_addr);
	int (*disable_charging)(u8 dev_addr);
	int (*handle_irq) (u8 dev_addr);
	int  (*cc_to_reg)(int cc, u8 *reg_val);
	int  (*cv_to_reg)(int cv, u8 *reg_val);
	bool (*is_batt_charging)(u8 dev_addr);
};

/*
 * basinc cove charger driver info
 */
struct bc_chrgr_drv_context {

	atomic_t i2c_rw;
	bool invalid_batt;
	bool is_batt_present;
	bool current_sense_enabled;
	unsigned int irq;		/* GPE_ID or IRQ# */
	u8 ext_chrgr_addr;
	void __iomem *pmic_intr_iomap;
	void *ch_handle;
	wait_queue_head_t i2c_wait;

	struct device *dev;
	struct power_supply psy;
	struct charger_helper_charger ch_charger;
	struct ext_charger *ext_chrgr;
	struct notifier_block otg_nb;
	struct otg_transceiver *transceiver;
};
bool bc_is_current_sense_enabled(void);
bool bc_check_battery_present(void);
int bc_check_battery_health(void);
int bc_check_battery_status(void);
int bc_get_battery_pack_temp(int *val);


static struct interrupt_info chgrirq0_info[] = {
	{	CHGIRQ0_BZIRQ_MASK,
		/*igone interrupt status */
		0,
		"Battery temperature zone changed",
		NULL,
		NULL,
		bcove_bat_zone_changed,
		NULL,
	},
	{	CHGIRQ0_BAT_CRIT_MASK,
		SCHGIRQ0_SBAT_CRIT_MASK,
		NULL,
		"Battery Over heat exception",
		"Battery Over heat exception Recovered",
		NULL,
		bcove_battery_overheat_handler
	},
	{	CHGIRQ0_BAT0_ALRT_MASK,
		SCHGIRQ0_SBAT0_ALRT_MASK,
		NULL,
		"Battery0 temperature inside boundary",
		"Battery0 temperature outside boundary",
		NULL,
		bcove_battery_overheat_handler
	},
	{	CHGIRQ0_BAT1_ALRT_MASK,
		SCHGIRQ0_SBAT1_ALRT_MASK,
		NULL,
		"Battery1 temperature inside boundary",
		"Battery1 temperature outside boundary",
		NULL,
		NULL
	},
	{	CHGIRQ0_NACK_MASK,
		SCHGIRQ0_SNACK_MASK,
		NULL,
		"Previous I2C transaction completed successfully (ACK)",
		"Previous I2C transaction ended not completed (NACK)",
		NULL,
		NULL
	},
	{	CHGIRQ0_I2CRDP_CMP_MASK,
		SCHGIRQ0_SI2CRD_CMP_MASK,
		NULL,
		NULL,
		"I2C read in progress",
		NULL,
		bcove_extchrgr_read_complete,
	},
	{	CHGIRQ0_I2CWR_CMP_MASK,
		SCHGIRQ0_SI2CWR_CMP_MASK,
		NULL,
		NULL,
		"I2C write in progress",
		NULL,
		bcove_extchrgr_write_complete,
	},
	{	CHGIRQ0_CHG_INTB_MASK,
		SCHGIRQ0_SCHG_INTB,
		NULL,
		"External Charger IRQ",
		NULL,
		NULL,
		bcove_handle_ext_chrgr_irq
	},
};

struct temp_lookup {
	int adc_val;
	int temp;
	int temp_err;
};

static struct temp_lookup adc_tbl[] = {

		{0x24, 125, 0}, {0x28, 120, 0},
		{0x2D, 115, 0}, {0x32, 110, 0},
		{0x38, 105, 0}, {0x40, 100, 0},
		{0x48, 95, 0}, {0x51, 90, 0},
		{0x5C, 85, 0}, {0x68, 80, 0},
		{0x77, 75, 0}, {0x87, 70, 0},
		{0x99, 65, 0}, {0xAE, 60, 0},
		{0xC7, 55, 0}, {0xE2, 50, 0},
		{0x101, 45, 0}, {0x123, 40, 0},
		{0x149, 35, 0}, {0x172, 30, 0},
		{0x19F, 25, 0}, {0x1CE, 20, 0},
		{0x200, 15, 0}, {0x233, 10, 0},
		{0x266, 5, 0}, {0x299, 0, 0},
		{0x2CA, -5, 0}, {0x2F9, -10, 0},
		{0x324, -15, 0}, {0x34B, -20, 0},
		{0x36D, -25, 0}, {0x38A, -30, 0},
		{0x3A4, -35, 0}, {0x3B8, -40, 0},
	};

#endif
