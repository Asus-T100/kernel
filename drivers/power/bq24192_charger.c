/*
 * bq24192_charger.c - Charger driver for TI BQ24192,BQ24191 and BQ24190
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
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 * Author: Raj Pandey <raj.pandey@intel.com>
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/power/bq24192_charger.h>
#include <linux/sfi.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>
#include <linux/rpmsg.h>

#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>

#define DRV_NAME "bq24192_charger"
#define DEV_NAME "bq24192"

/*
 * D0, D1, D2 can be used to set current limits
 * and D3, D4, D5, D6 can be used to voltage limits
 */
#define BQ24192_INPUT_SRC_CNTL_REG		0x0
#define INPUT_SRC_CNTL_EN_HIZ			(1 << 7)
/*
 * set input voltage lim to 4.68V. This will help in charger
 * instability issue when duty cycle reaches 100%.
 */
#define INPUT_SRC_VOLT_LMT_DEF                 (3 << 4)
#define INPUT_SRC_VOLT_LMT_444                 (7 << 3)
#define INPUT_SRC_VOLT_LMT_468                 (5 << 4)

#define INPUT_SRC_VINDPM_MASK                  (0xF << 3)
#define INPUT_SRC_LOW_VBAT_LIMIT               3600
#define INPUT_SRC_MID_VBAT_LIMIT               4000
#define INPUT_SRC_HIG_VBAT_LIMIT               4200

/* D0, D1, D2 represent the input current limit */
#define INPUT_SRC_CUR_LMT0		0x0	/* 100mA */
#define INPUT_SRC_CUR_LMT1		0x1	/* 150mA */
#define INPUT_SRC_CUR_LMT2		0x2	/* 500mA */
#define INPUT_SRC_CUR_LMT3		0x3	/* 900mA */
#define INPUT_SRC_CUR_LMT4		0x4	/* 1200mA */
#define INPUT_SRC_CUR_LMT5		0x5	/* 1500mA */
#define INPUT_SRC_CUR_LMT6		0x6	/* 2000mA */
#define INPUT_SRC_CUR_LMT7		0x7	/* 3000mA */

/*
 * D1, D2, D3 can be used to set min sys voltage limit
 * and D4, D5 can be used to control the charger
 */
#define BQ24192_POWER_ON_CFG_REG		0x1
#define POWER_ON_CFG_RESET			(1 << 7)
#define POWER_ON_CFG_I2C_WDTTMR_RESET		(1 << 6)
#define CHR_CFG_BIT_POS				4
#define CHR_CFG_BIT_LEN				2
#define POWER_ON_CFG_CHRG_CFG_DIS		(0 << 4)
#define POWER_ON_CFG_CHRG_CFG_EN		(1 << 4)
#define POWER_ON_CFG_CHRG_CFG_OTG		(3 << 4)
#define POWER_ON_CFG_BOOST_LIM			(1 << 0)

/*
 * Charge Current control register
 * with range from 500 - 4532mA
 */
#define BQ24192_CHRG_CUR_CNTL_REG		0x2
#define BQ24192_CHRG_CUR_OFFSET		500	/* 500 mA */
#define BQ24192_CHRG_CUR_LSB_TO_CUR	64	/* 64 mA */
#define BQ24192_GET_CHRG_CUR(reg) ((reg>>2)*BQ24192_CHRG_CUR_LSB_TO_CUR\
			+ BQ24192_CHRG_CUR_OFFSET) /* in mA */

/* Pre charge and termination current limit reg */
#define BQ24192_PRECHRG_TERM_CUR_CNTL_REG	0x3
#define BQ24192_TERM_CURR_LIMIT_128		0	/* 128mA */
#define BQ24192_PRE_CHRG_CURR_256		(1 << 4)  /* 256mA */

/* Charge voltage control reg */
#define BQ24192_CHRG_VOLT_CNTL_REG	0x4
#define BQ24192_CHRG_VOLT_OFFSET	3504	/* 3504 mV */
#define BQ24192_CHRG_VOLT_LSB_TO_VOLT	16	/* 16 mV */
/* Low voltage setting 0 - 2.8V and 1 - 3.0V */
#define CHRG_VOLT_CNTL_BATTLOWV		(1 << 1)
/* Battery Recharge threshold 0 - 100mV and 1 - 300mV */
#define CHRG_VOLT_CNTL_VRECHRG		(1 << 0)
#define BQ24192_GET_CHRG_VOLT(reg) ((reg>>2)*BQ24192_CHRG_VOLT_LSB_TO_VOLT\
			+ BQ24192_CHRG_VOLT_OFFSET) /* in mV */

/* Charge termination and Timer control reg */
#define BQ24192_CHRG_TIMER_EXP_CNTL_REG		0x5
#define CHRG_TIMER_EXP_CNTL_EN_TERM		(1 << 7)
#define CHRG_TIMER_EXP_CNTL_TERM_STAT		(1 << 6)
/* WDT Timer uses 2 bits */
#define WDT_TIMER_BIT_POS			4
#define WDT_TIMER_BIT_LEN			2
#define CHRG_TIMER_EXP_CNTL_WDTDISABLE		(0 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT40SEC		(1 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT80SEC		(2 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT160SEC		(3 << 4)
#define WDTIMER_RESET_MASK			0x40
#define CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_0	(1<<4)
#define CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_1	(1<<5)
/* Safety Timer Enable bit */
#define CHRG_TIMER_EXP_CNTL_EN_TIMER		(1 << 3)
/* Charge Timer uses 2bits(20 hrs) */
#define SFT_TIMER_BIT_POS			1
#define SFT_TIMER_BIT_LEN			2
#define CHRG_TIMER_EXP_CNTL_SFT_TIMER		(3 << 1)

#define BQ24192_CHRG_THRM_REGL_REG		0x6

#define BQ24192_MISC_OP_CNTL_REG		0x7
#define MISC_OP_CNTL_DPDM_EN			(1 << 7)
#define MISC_OP_CNTL_TMR2X_EN			(1 << 6)
#define MISC_OP_CNTL_BATFET_DIS			(1 << 5)
#define MISC_OP_CNTL_BATGOOD_EN			(1 << 4)
/* To mask INT's write 0 to the bit */
#define MISC_OP_CNTL_MINT_CHRG			(1 << 1)
#define MISC_OP_CNTL_MINT_BATT			(1 << 0)

#define BQ24192_SYSTEM_STAT_REG			0x8
/* D6, D7 show VBUS status */
#define SYSTEM_STAT_VBUS_UNKNOWN		0
#define SYSTEM_STAT_VBUS_HOST			(1 << 6)
#define SYSTEM_STAT_VBUS_ADP			(2 << 6)
#define SYSTEM_STAT_VBUS_OTG			(3 << 6)
/* D4, D5 show charger status */
#define SYSTEM_STAT_NOT_CHRG			(0 << 4)
#define SYSTEM_STAT_PRE_CHRG			(1 << 4)
#define SYSTEM_STAT_FAST_CHRG			(2 << 4)
#define SYSTEM_STAT_CHRG_DONE			(3 << 4)
#define SYSTEM_STAT_DPM				(1 << 3)
#define SYSTEM_STAT_PWR_GOOD			(1 << 2)
#define SYSTEM_STAT_THERM_REG			(1 << 1)
#define SYSTEM_STAT_VSYS_LOW			(1 << 0)
#define SYSTEM_STAT_CHRG_MASK			(3 << 4)

#define BQ24192_FAULT_STAT_REG			0x9
#define FAULT_STAT_WDT_TMR_EXP			(1 << 7)
#define FAULT_STAT_OTG_FLT			(1 << 6)
/* D4, D5 show charger fault status */
#define FAULT_STAT_CHRG_NORMAL			(0 << 4)
#define FAULT_STAT_CHRG_IN_FLT			(1 << 4)
#define FAULT_STAT_CHRG_THRM_FLT		(2 << 4)
#define FAULT_STAT_CHRG_TMR_FLT			(3 << 4)
#define FAULT_STAT_BATT_FLT			(1 << 3)

#define BQ24192_VENDER_REV_REG			0xA
/* D3, D4, D5 indicates the chip model number */
#define BQ24190_IC_VERSION			0x0
#define BQ24191_IC_VERSION			0x1
#define BQ24192_IC_VERSION			0x2
#define BQ24192I_IC_VERSION			0x3

#define BQ24192_MAX_MEM		12
#define NR_RETRY_CNT		3

#define CHARGER_PS_NAME				"bq24192_charger"

#define BQ24192_DEF_VBATT_MAX		4192	/* 4192mV */
#define BQ24192_DEF_SDP_ILIM_CUR	500	/* 500mA */
#define BQ24192_DEF_DCP_ILIM_CUR	1500	/* 1500mA */
#define BQ24192_DEF_CHRG_CUR		1000	/* 1000mA */

#define BQ24192_CHRG_CUR_LOW		100	/* 100mA */
#define BQ24192_CHRG_CUR_MEDIUM		500	/* 500mA */
#define BQ24192_CHRG_CUR_HIGH		900	/* 900mA */
#define BQ24192_CHRG_CUR_NOLIMIT	1500	/* 1500mA */

#define STATUS_UPDATE_INTERVAL		(HZ * 60) /* 60sec */

#define BQ24192_CHRG_OTG_GPIO		36
#define MAINTENANCE_CHRG_JIFFIES	(HZ * 60) /* 60sec */
#define INITIAL_THREAD_JIFFY		(HZ / 2)  /* 500msec */

#define CLT_BPTHERM_CURVE_MAX_SAMPLES	23
#define CLT_BPTHERM_CURVE_MAX_VALUES	4
/* default Charger parameters */
#define CLT_BATT_CHRVOLTAGE_SET_DEF	4200 /*in mV */
#define CLT_BATT_DEFAULT_MAX_CAPACITY	1500 /*in mAH */

/* ADC Channel Numbers */
#define CLT_BATT_NUM_GPADC_SENSORS	1
#define CLT_GPADC_BPTHERM_CHNUM	0x9
#define CLT_GPADC_BPTHERM_SAMPLE_COUNT	1

/*CLT battery temperature  attributes*/
#define CLT_BTP_ADC_MIN	107
#define CLT_BTP_ADC_MAX	977

#define SFI_BATTPROP_TBL_ID	"OEM0"
#define CLT_ADC_TIME_TO_LIVE	(HZ/8)	/* 125 ms */

#define CLT_VBATT_FULL_DET_MARGIN	25	/* 25mV */
#define CLT_FULL_CURRENT_AVG_LOW	0
#define CLT_FULL_CURRENT_AVG_HIGH	100

#define CLT_BATT_VMIN_THRESHOLD_DEF	3600	/* 3600mV */
#define CLT_BATT_TEMP_MAX_DEF	45	/* 45 degrees */
#define CLT_BATT_TEMP_MIN_DEF	0
#define CLT_BATT_CRIT_CUTOFF_VOLT_DEF	3700	/* 3700 mV */

#define BQ24192_INVALID_CURR -1
#define BQ24192_INVALID_VOLT -1

/* SMIP related definitions */
/* sram base address for smip accessing*/
#define SMIP_SRAM_OFFSET_ADDR	0x44d
#define SMIP_SRAM_BATT_PROP_OFFSET_ADDR	0x460
#define TEMP_MON_RANGES	4

/* Signature comparision of SRAM data for supportted Battery Char's */
#define SBCT_REV	0x16
#define RSYS_MOHMS	0xAA

/* Max no. of tries to clear the charger from Hi-Z mode */
#define MAX_TRY		3

/* Max no. of tries to reset the bq24192i WDT */
#define MAX_RESET_WDT_RETRY 8

/* Master Charge control register */
#define MSIC_CHRCRTL	0x188
#define MSIC_CHRGENBL	0x40

/* charger interrupt pin */
#define CHRG_INT_N	93

static struct power_supply *fg_psy;
static struct ctp_batt_sfi_prop *ctp_sfi_table;


struct bq24192_chrg_regs {
	u8 in_src;
	u8 pwr_cfg;
	u8 chr_cur;
	u8 chr_volt;
};

struct bq24192_chip {
	struct i2c_client *client;
	struct bq24192_platform_data *pdata;
	struct power_supply usb;
	struct power_supply_charger_cap cap;
	struct delayed_work chrg_evt_wrkr;
	struct delayed_work maint_chrg_wrkr;
	struct mutex event_lock;

	int present;
	int online;
	enum power_supply_type chrg_type;
	int chrg_cur_cntl; /* contains the current limit index */

	/* battery info */
	int batt_status;
	bool votg;
	enum bq24192_bat_chrg_mode batt_mode;

	/* Handle for gpadc requests */
	void *gpadc_handle;
	struct ctp_batt_safety_thresholds batt_thrshlds;
	/* cached parameters for event worker handler needed
	 * to support extreme charging*/
	int curr_volt;
	int curr_chrg;
	int input_curr;
	int cached_chrg_cur_cntl;
	int batt_health;
	bool is_pwr_good;
	struct power_supply_charger_cap cached_cap;
	/* Wake lock to prevent platform from going to S3 when charging */
	struct wake_lock wakelock;
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *bq24192_dbgfs_root;
static char bq24192_dbg_regs[BQ24192_MAX_MEM][4];
#endif

static struct i2c_client *bq24192_client;

static char *bq24192_power_supplied_to[] = {
			"max170xx_battery",
			"max17042_battery",
			"max17047_battery",
};

/*
 * temperature v/s ADC value table to interpolate and calculate temp
 */
static int const ctp_bptherm_curve_data[CLT_BPTHERM_CURVE_MAX_SAMPLES]
	[CLT_BPTHERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 977, 961},
	{-10, -15, 961, 941},
	{-5, -10, 941, 917},
	{0, -5, 917, 887},
	{5, 0, 887, 853},
	{10, 5, 853, 813},
	{15, 10, 813, 769},
	{20, 15, 769, 720},
	{25, 20, 720, 669},
	{30, 25, 669, 615},
	{35, 30, 615, 561},
	{40, 35, 561, 508},
	{45, 40, 508, 456},
	{50, 45, 456, 407},
	{55, 50, 407, 357},
	{60, 55, 357, 315},
	{65, 60, 315, 277},
	{70, 65, 277, 243},
	{75, 70, 243, 212},
	{80, 75, 212, 186},
	{85, 80, 186, 162},
	{90, 85, 162, 140},
	{100, 90, 140, 107},
};


static enum power_supply_property bq24192_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH
};

/* it will be used to store threshold voltage during sleep mode */
static short int vbatt_sh_min;

/************************************************************************
 * End of structure definition section
 ***********************************************************************/

/*
 * sfi table parsing specific interfaces
 */

/**
 * ctp_sfi_table_invalid_batt - default battery SFI table values  to be
 * used in case of invalid battery
 *
 * @sfi_table : sfi table pointer
 * Context: can sleep
 * Note: These interfaces will move to a common file and will be
 * independent of the platform
 */
static void ctp_sfi_table_invalid_batt(struct ctp_batt_sfi_prop *sfi_table)
{
	/*
	 * In case of invalid battery we manually set
	 * the SFI parameters and limit the battery from
	 * charging, so platform will be in discharging mode
	 */
	memcpy(ctp_sfi_table->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	ctp_sfi_table->voltage_max = CLT_BATT_CHRVOLTAGE_SET_DEF;
	ctp_sfi_table->capacity = CLT_BATT_DEFAULT_MAX_CAPACITY;
	ctp_sfi_table->battery_type = POWER_SUPPLY_TECHNOLOGY_LION;
	ctp_sfi_table->temp_mon_ranges = 0;
}

/**
 * ctp_sfi_table_populate - Simple Firmware Interface table Populate
 * @sfi_table: Simple Firmware Interface table structure
 *
 * SFI table has entries for the temperature limits
 * which is populated in a local structure
 */
static int ctp_sfi_table_populate(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct ctp_batt_sfi_prop *pentry;
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	int totentrs = 0, totlen = 0;

	sb = (struct sfi_table_simple *)table;
	if (!sb) {
		dev_warn(&chip->client->dev, "SFI: Unable to map BATT signature\n");
		return -ENODEV;
	}

	totentrs = SFI_GET_NUM_ENTRIES(sb, struct ctp_batt_sfi_prop);
	if (totentrs) {
		dev_info(&chip->client->dev, "Valid battery detected.\n");
		pentry = (struct ctp_batt_sfi_prop *)sb->pentry;
		totlen = totentrs * sizeof(*pentry);
		memcpy(ctp_sfi_table, pentry, totlen);

		if (ctp_sfi_table->temp_mon_ranges != CLT_SFI_TEMP_NR_RNG)
			dev_warn(&chip->client->dev,
				"SFI: temperature monitoring range"
				"doesn't match with its Array elements size\n");
		chip->pdata->sfi_tabl_present = true;
	} else {
		dev_warn(&chip->client->dev, "Invalid battery detected\n");
		chip->pdata->sfi_tabl_present = false;
		ctp_sfi_table_invalid_batt(ctp_sfi_table);
	}

	return 0;
}

/* Check for valid Temp ADC range */
static bool ctp_is_valid_temp_adc(int adc_val)
{
	bool ret = false;

	if (adc_val >= CLT_BTP_ADC_MIN && adc_val <= CLT_BTP_ADC_MAX)
		ret = true;

	return ret;
}

/* Temperature conversion Macros */
static int ctp_conv_adc_temp(int adc_val,
	int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

/* Check if the adc value is in the curve sample range */
static bool ctp_is_valid_temp_adc_range(int val, int min, int max)
{
	bool ret = false;
	if (val >= min && val <= max)
		ret = true;
	return ret;
}

/**
 * ctp_adc_to_temp - convert ADC code to temperature
 * @adc_val : ADC sensor reading
 * @tmp : finally read temperature
 *
 * Returns 0 on success or -ERANGE in error case
 */
static int ctp_adc_to_temp(uint16_t adc_val, int *tmp)
{
	int temp = 0;
	int i;

	if (!ctp_is_valid_temp_adc(adc_val)) {
		dev_warn(&bq24192_client->dev,
			"Temperature out of Range: %u\n", adc_val);
		/*
		 * If the value returned as an ERANGE the battery icon shows an
		 * exclaimation mark in the COS.In order to fix the issue, if
		 * the ADC returns a value which is not in range specified, we
		 * update the value within the bound.
		 */
		if (adc_val > CLT_BTP_ADC_MAX)
			adc_val = CLT_BTP_ADC_MAX;
		else if (adc_val < CLT_BTP_ADC_MIN)
			adc_val = CLT_BTP_ADC_MIN;
	}

	for (i = 0; i < CLT_BPTHERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (ctp_is_valid_temp_adc_range(
			adc_val, ctp_bptherm_curve_data[i][3],
			ctp_bptherm_curve_data[i][2])) {

			temp = ctp_conv_adc_temp(adc_val,
				  ctp_bptherm_curve_data[i][2],
				  ctp_bptherm_curve_data[i][2] -
				  ctp_bptherm_curve_data[i][3],
				  ctp_bptherm_curve_data[i][0] -
				  ctp_bptherm_curve_data[i][1]);

			temp += ctp_bptherm_curve_data[i][1];
			break;
		}
	}

	if (i >= CLT_BPTHERM_CURVE_MAX_SAMPLES) {
		dev_warn(&bq24192_client->dev, "Invalid temp adc range\n");
		return -EINVAL;
	}
	*tmp = temp;

	return 0;
}

/**
 * ctp_read_adc_temp - read ADC sensor to get the temperature
 * @tmp: op parameter where temperature get's read
 *
 * Returns 0 if success else -1 or -ERANGE
 */
static int ctp_read_adc_temp(int *tmp)
{
	int gpadc_sensor_val = 0;
	int ret;
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);

	if (!chip->gpadc_handle) {
		ret = -ENODEV;
		goto read_adc_exit;
	}

	ret = intel_mid_gpadc_sample(chip->gpadc_handle,
				CLT_GPADC_BPTHERM_SAMPLE_COUNT,
				&gpadc_sensor_val);
	if (ret) {
		dev_err(&bq24192_client->dev,
			"adc driver api returned error(%d)\n", ret);
		goto read_adc_exit;
	}

	ret = ctp_adc_to_temp(gpadc_sensor_val, tmp);
read_adc_exit:
	return ret;
}

/**
 * ctp_sfi_temp_range_lookup - lookup SFI table to find the temperature range index
 * @adc_temp : temperature in Degree Celcius
 *
 * Returns temperature range index
 */
static int ctp_sfi_temp_range_lookup(int adc_temp)
{
	int i, idx = -1;
	int max_range;
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	struct ctp_temp_mon_table *temp_mon_tabl = NULL;
	short int temp_low_lim;

	if (chip->pdata->sfi_tabl_present) {
		dev_info(&chip->client->dev,
			 "Read the temperature range from sfi table\n");
		if (ctp_sfi_table->temp_mon_ranges < CLT_SFI_TEMP_NR_RNG)
			max_range = ctp_sfi_table->temp_mon_ranges;
		else
			max_range = CLT_SFI_TEMP_NR_RNG;
		temp_mon_tabl = &ctp_sfi_table->temp_mon_range[0];
	} else {
		dev_info(&chip->client->dev,
			 "Read the temperature range from platform data\n");
		temp_mon_tabl = &chip->pdata->temp_mon_range[0];
		max_range = chip->pdata->temp_mon_ranges;
	}

	for (i = max_range-1; i >= 0; i--) {
		temp_low_lim = temp_mon_tabl[i].temp_low_lim;
		if (adc_temp <= temp_mon_tabl[i].temp_up_lim &&
			adc_temp > temp_low_lim) {
			idx = i;
			break;
		}
	}

	dev_info(&chip->client->dev, "%s:temp idx = %d\n", __func__, idx);
	return idx;
}
/* returns the max and min temp in which battery is suppose to operate */
static void ctp_get_batt_temp_thresholds(short int *temp_high,
		short int *temp_low)
{
	int i, max_range;
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	struct ctp_temp_mon_table *temp_mon_tabl = NULL;

	*temp_high = *temp_low = 0;
	if (chip->pdata->sfi_tabl_present) {
		if (ctp_sfi_table->temp_mon_ranges < CLT_SFI_TEMP_NR_RNG)
			max_range = ctp_sfi_table->temp_mon_ranges;
		else
			max_range = CLT_SFI_TEMP_NR_RNG;
		temp_mon_tabl = &ctp_sfi_table->temp_mon_range[0];
	} else {
		temp_mon_tabl = &chip->pdata->temp_mon_range[0];
		max_range = chip->pdata->temp_mon_ranges;
	}

	for (i = 0; i < max_range; i++) {
		if (*temp_high < temp_mon_tabl[i].temp_up_lim)
			*temp_high = temp_mon_tabl[i].temp_up_lim;
	}

	for (i = 0; i < max_range; i++) {
		if (*temp_low > temp_mon_tabl[i].temp_low_lim)
			*temp_low = temp_mon_tabl[i].temp_low_lim;
	}
}

/*-------------------------------------------------------------------------*/


/*
 * Genenric register read/write interfaces to access registers in charger ic
 */

static int bq24192_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Write error:%d\n", ret);

	return ret;
}

static int bq24192_read_reg(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Read error:%d\n", ret);

	return ret;
}

#ifdef DEBUG
/*
 * This function dumps the bq24192 registers
 */
static void bq24192_dump_registers(struct bq24192_chip *chip)
{
	int ret;

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	/* Input Src Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Input Src Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG00 %x\n", ret);

	/* Pwr On Cfg register */
	ret = bq24192_read_reg(chip->client, BQ24192_POWER_ON_CFG_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pwr On Cfg reg read fail\n");
	dev_info(&chip->client->dev, "REG01 %x\n", ret);

	/* Chrg Curr Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_CUR_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Curr Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG02 %x\n", ret);

	/* Pre-Chrg Term register */
	ret = bq24192_read_reg(chip->client,
					BQ24192_PRECHRG_TERM_CUR_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pre-Chrg Term reg read fail\n");
	dev_info(&chip->client->dev, "REG03 %x\n", ret);

	/* Chrg Volt Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_VOLT_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Volt Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG04 %x\n", ret);

	/* Chrg Term and Timer Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"Chrg Term and Timer Ctrl reg read fail\n");
	}
	dev_info(&chip->client->dev, "REG05 %x\n", ret);

	/* Thermal Regulation register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_THRM_REGL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
				"Thermal Regulation reg read fail\n");
	}
	dev_info(&chip->client->dev, "REG06 %x\n", ret);

	/* Misc Operations Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_MISC_OP_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Misc Op Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG07 %x\n", ret);

	/* System Status register */
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "System Status reg read fail\n");
	dev_info(&chip->client->dev, "REG08 %x\n", ret);

	/* Fault Status register */
	ret = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Fault Status reg read fail\n");
	dev_info(&chip->client->dev, "REG09 %x\n", ret);

	/* Vendor Revision register */
	ret = bq24192_read_reg(chip->client, BQ24192_VENDER_REV_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	dev_info(&chip->client->dev, "REG0A %x\n", ret);
}
#endif

int bq24192_query_battery_status(void)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);

	return chip->batt_status;
}
EXPORT_SYMBOL(bq24192_query_battery_status);

/*
 * If the bit_set is TRUE then val 1s will be SET in the reg else val 1s will
 * be CLEARED
 */
static int bq24192_reg_read_modify(struct i2c_client *client, u8 reg,
							u8 val, bool bit_set)
{
	int ret;

	ret = bq24192_read_reg(client, reg);

	if (bit_set)
		ret |= val;
	else
		ret &= (~val);

	ret = bq24192_write_reg(client, reg, ret);

	return ret;
}

static int bq24192_reg_multi_bitset(struct i2c_client *client, u8 reg,
						u8 val, u8 pos, u8 len)
{
	int ret;
	u8 data;

	ret = bq24192_read_reg(client, reg);
	if (ret < 0) {
		dev_warn(&client->dev, "I2C SMbus Read error:%d\n", ret);
		return ret;
	}

	data = (1 << len) - 1;
	ret = (ret & ~(data << pos)) | val;
	ret = bq24192_write_reg(client, reg, ret);

	return ret;
}

/*
 * This function verifies if the bq24192i charger chip is in Hi-Z
 * If yes, then clear the Hi-Z to resume the charger operations
 */
static int bq24192_clear_hiz(struct bq24192_chip *chip)
{
	int ret, count;

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	mutex_lock(&chip->event_lock);

	for (count = 0; count < MAX_TRY; count++) {
		/*
		 * Read the bq24192i REG00 register for charger Hi-Z mode.
		 * If it is in Hi-Z, then clear the Hi-Z to resume the charging
		 * operations.
		 */
		ret = bq24192_read_reg(chip->client,
				BQ24192_INPUT_SRC_CNTL_REG);
		if (ret < 0) {
			dev_warn(&chip->client->dev,
					"Input src cntl read failed\n");
			goto i2c_error;
		}

		if (ret & INPUT_SRC_CNTL_EN_HIZ) {
			dev_warn(&chip->client->dev,
						"Charger IC in Hi-Z mode\n");
#ifdef DEBUG
			bq24192_dump_registers(chip);
#endif
			/* Clear the Charger from Hi-Z mode */
			ret = (chip->input_curr & ~INPUT_SRC_CNTL_EN_HIZ);

			/* Write the values back */
			ret = bq24192_write_reg(chip->client,
					BQ24192_INPUT_SRC_CNTL_REG, ret);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"Input src cntl write failed\n");
				goto i2c_error;
			}
			msleep(150);
		} else {
			dev_info(&chip->client->dev,
						"Charger is not in Hi-Z\n");
			break;
		}
	}
	mutex_unlock(&chip->event_lock);
	return ret;
i2c_error:
	mutex_unlock(&chip->event_lock);
	dev_err(&chip->client->dev, "%s\n", __func__);
	return ret;
}
/*****************************************************************************/
/*
 * Extreme Charging Section: This section defines sysfs interfaces used
 * for setting up the required thermal zone.
 */

/* Sysfs Entry for enable or disable Charging from user space */
static ssize_t set_charge_current_limit(struct device *device,
			struct device_attribute *attr, const char *buf,
			size_t count);
static ssize_t get_charge_current_limit(struct device *device,
			struct device_attribute *attr, char *buf);
static DEVICE_ATTR(charge_current_limit, S_IRUGO | S_IWUSR,
					get_charge_current_limit,
					set_charge_current_limit);


/* map charge current control setting
 * to input current limit value in mA.
 */
static int chrg_lim_idx_to_chrg_cur(int lim)
{
	int cur_lim;

	switch (lim) {
	case USER_SET_CHRG_LMT1:
		cur_lim = BQ24192_CHRG_CUR_LOW;
		break;
	case USER_SET_CHRG_LMT2:
		cur_lim = BQ24192_CHRG_CUR_MEDIUM;
		break;
	case USER_SET_CHRG_LMT3:
		cur_lim = BQ24192_CHRG_CUR_HIGH;
		break;
	default:
		cur_lim = -EINVAL;
	}
	return cur_lim;
}

/**
 * set_charge_current_limit - sysfs set api for charge_enable attribute
 * Parameter as define by sysfs interface
 * Context: can sleep
 *
 */
static ssize_t set_charge_current_limit(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	long value;
	int chr_mode;
	dev_info(&chip->client->dev, "+%s\n", __func__);

	/* Check if the charger is present. If not just return */
	if (!chip->present && !chip->online) {
		dev_info(&chip->client->dev,
				"%s: No Charger\n", __func__);
		return count;
	}

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	/* Allow only 0 to 4 for writing */
	if (value < USER_SET_CHRG_DISABLE || value > USER_SET_CHRG_NOLMT) {
		dev_info(&chip->client->dev,
			"%s: Thermal index %lu out of range\n", __func__,
			value);
		return -EINVAL;
	}

	/*
	 * Check for the battery health and if the battery health is good
	 * throttle/continue the charging else don't throttle coz the charging
	 * will be stopped if the battery health is not good.
	 */
	if (chip->batt_health == POWER_SUPPLY_HEALTH_COLD ||
		chip->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		dev_info(&chip->client->dev, "Battery in extreme temp zone\n");
		return -EINVAL;
	}

	chr_mode = chip->batt_mode;

	switch (value) {
	case USER_SET_CHRG_DISABLE:
		dev_dbg(&chip->client->dev,
			"%s: User App Charge Disable\n", __func__);
		mutex_lock(&chip->event_lock);
		chip->chrg_cur_cntl = value;
		mutex_unlock(&chip->event_lock);

		/* check if battery is in charging mode */
		if (chr_mode != BATT_CHRG_NONE) {
			/* Disable Charger before setting up usr_chrg_enable */
			dev_dbg(&chip->client->dev,
				"%s: Send POWER_SUPPLY_CHARGER_EVENT_SUSPEND\n"\
				, __func__);
			mutex_lock(&chip->event_lock);
			chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_SUSPEND;
			mutex_unlock(&chip->event_lock);
			schedule_delayed_work(&chip->chrg_evt_wrkr, 0);
		}
		break;
	case USER_SET_CHRG_LMT1:
	case USER_SET_CHRG_LMT2:
	case USER_SET_CHRG_LMT3:
	case USER_SET_CHRG_NOLMT:
		dev_dbg(&chip->client->dev, \
			"%s: User App Charge Enable\n", __func__);
		mutex_lock(&chip->event_lock);
		chip->chrg_cur_cntl = value;
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_RESUME;
		mutex_unlock(&chip->event_lock);
		schedule_delayed_work(&chip->chrg_evt_wrkr, 0);
		break;
	default:
		dev_err(&chip->client->dev, "Invalid request\n");
	}

	dev_info(&chip->client->dev,
		"%s:chr_mode : %d, chip->chrg_cur_cntl: %d\n", \
		__func__, chip->batt_mode, chip->chrg_cur_cntl);
	return count;
}

/**
 * get_chrg_enable - sysfs get api for charge_enable attribute
 * Parameter as define by sysfs interface
 * Context: can sleep
 *
 */
static ssize_t get_charge_current_limit(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	unsigned long value;

	dev_info(&chip->client->dev, "+%s\n", __func__);

	/* Update the variable for the user app */
	mutex_lock(&chip->event_lock);
	value = chip->chrg_cur_cntl;
	mutex_unlock(&chip->event_lock);

	return sprintf(buf, "%lu\n", value);
}
/****************************************************************************/
/*
 * charger and battery specific interfaces exposed to external modules
 */

/* returns the battery pack temperature read from adc */
int ctp_get_battery_pack_temp(int *temp)
{
	struct bq24192_chip *chip = NULL;

	if (!bq24192_client)
		return -ENODEV;

	chip = i2c_get_clientdata(bq24192_client);

	/* check if charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	return ctp_read_adc_temp(temp);
}
EXPORT_SYMBOL(ctp_get_battery_pack_temp);

/* returns battery status */
int ctp_query_battery_status(void)
{
	struct bq24192_chip *chip = NULL;

	if (!bq24192_client)
		return -ENODEV;

	chip = i2c_get_clientdata(bq24192_client);

	return chip->batt_status;
}
EXPORT_SYMBOL(ctp_query_battery_status);

/**
 * ctp_get_charger_health - to get the charger health status
 *
 * Returns charger health status
 */
int ctp_get_charger_health(void)
{
	int ret_status, ret_fault;
	struct bq24192_chip *chip =
		i2c_get_clientdata(bq24192_client);

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	ret_fault = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (ret_fault < 0) {
		dev_warn(&chip->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (ret_fault & FAULT_STAT_OTG_FLT)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	/* Check if the WeakVIN condition occured */
	ret_status = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret_status < 0) {
		dev_warn(&chip->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (!(ret_status & SYSTEM_STAT_PWR_GOOD) ||
	((ret_fault & FAULT_STAT_CHRG_IN_FLT) == FAULT_STAT_CHRG_IN_FLT))
		return POWER_SUPPLY_HEALTH_DEAD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

/**
 * ctp_get_battery_health - to get the battery health status
 *
 * Returns battery health status
 */
int ctp_get_battery_health(void)
{
	int batt_temp, ret;
	struct bq24192_chip *chip =
		i2c_get_clientdata(bq24192_client);

	dev_dbg(&chip->client->dev, "+%s\n", __func__);

	/* If power supply is emulating as battery, return health as good */
	if (!chip->pdata->sfi_tabl_present)
		return POWER_SUPPLY_HEALTH_GOOD;

	/* Get the battery pack temperature */
	ret = ctp_get_battery_pack_temp(&batt_temp);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"battery pack temp read fail:%d", ret);
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	if ((batt_temp > chip->batt_thrshlds.temp_high) ||
			(batt_temp < chip->batt_thrshlds.temp_low)) {
		dev_err(&chip->client->dev, "Battery over heat\n");
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	/* Check if battery OVP condition occured */
	ret = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (ret & FAULT_STAT_BATT_FLT)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	dev_dbg(&chip->client->dev, "-%s\n", __func__);
	return POWER_SUPPLY_HEALTH_GOOD;
}
EXPORT_SYMBOL(ctp_get_battery_health);

/***********************************************************************/

/* convert the input current limit value
 * into equivalent register setting.
 * Note: ilim must be in mA.
 */
static u8 chrg_ilim_to_reg(int ilim)
{
	u8 reg;

	/* set voltage to 5V */
	reg = INPUT_SRC_VOLT_LMT_DEF;


	/* Set the input source current limit
	 * between 100 to 1500mA */
	if (ilim <= 100)
		reg |= INPUT_SRC_CUR_LMT0;
	else if (ilim <= 150)
		reg |= INPUT_SRC_CUR_LMT1;
	else if (ilim <= 500)
		reg |= INPUT_SRC_CUR_LMT2;
	else if (ilim <= 900)
		reg |= INPUT_SRC_CUR_LMT3;
	else if (ilim <= 1200)
		reg |= INPUT_SRC_CUR_LMT4;
	else
		reg |= INPUT_SRC_CUR_LMT5;

	return reg;
}

/* convert the charge current value
 * into equivalent register setting
 */
static u8 chrg_cur_to_reg(int cur)
{
	u8 reg;

	if (cur <= BQ24192_CHRG_CUR_OFFSET)
		reg = 0x0;
	else
		reg = ((cur - BQ24192_CHRG_CUR_OFFSET) /
				BQ24192_CHRG_CUR_LSB_TO_CUR) + 1;

	/* D0, D1 bits of Charge Current
	 * register are not used */
	reg = reg << 2;
	return reg;
}

/* convert the charge voltage value
 * into equivalent register setting
 */
static u8 chrg_volt_to_reg(int volt)
{
	u8 reg;

	if (volt <= BQ24192_CHRG_VOLT_OFFSET)
		reg = 0x0;
	else
		reg = (volt - BQ24192_CHRG_VOLT_OFFSET) /
				BQ24192_CHRG_VOLT_LSB_TO_VOLT;

	reg = (reg << 2) | CHRG_VOLT_CNTL_BATTLOWV;
	return reg;
}

/*
 * chip->event_lock need to be acquired before calling this function
 * to avoid the race condition
 */
static int program_timers(struct bq24192_chip *chip, bool wdt_enable,
				bool sfttmr_enable)
{
	int ret;

	/* Read the timer control register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "TIMER CTRL reg read failed\n");
		return ret;
	}

	/* Enable/Disable the WD timer */
	if (wdt_enable) {
		/* Programming for 80Secs */
		ret &= ~(CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_0);
		ret |=  (CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_1);
	} else {
		ret &= ~(CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_0 |
			 CHRG_TIMER_EXP_CNTL_WDTIMER_LIMIT_1);
	}

	/* Enable/Disable the safety timer */
	if (sfttmr_enable)
		ret |= CHRG_TIMER_EXP_CNTL_EN_TIMER;
	else
		ret &= ~CHRG_TIMER_EXP_CNTL_EN_TIMER;

	/*
	 * Enable the HW termination. When disabled the HW termination, battery
	 * was taking too long to go from charging to full state. HW based
	 * termination could cause the battery capacity to drop but it would
	 * result in good battery life.
	 */
	ret |= CHRG_TIMER_EXP_CNTL_EN_TERM;

	/* Program the TIMER CTRL register */
	ret = bq24192_write_reg(chip->client,
				BQ24192_CHRG_TIMER_EXP_CNTL_REG,
				ret);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER CTRL I2C write failed\n");

	return ret;
}

/* This function should be called with the mutex held */
static int reset_wdt_timer(struct bq24192_chip *chip)
{
	int ret = 0, i;

	/* reset WDT timer */
	for (i = 0; i < MAX_RESET_WDT_RETRY; i++) {
		ret = bq24192_reg_read_modify(chip->client,
						BQ24192_POWER_ON_CFG_REG,
						WDTIMER_RESET_MASK, true);
		if (ret < 0)
			dev_warn(&chip->client->dev, "I2C write failed:%s\n",
							__func__);
	}
	return ret;
}

static int enable_charging(struct bq24192_chip *chip,
				struct bq24192_chrg_regs *reg)
{
	int ret;

	/* set input voltage and current reg */
	ret = bq24192_write_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG,
								reg->in_src);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);
		goto i2c_write_failed;
	}

	/* set charge current reg */
	ret = bq24192_write_reg(chip->client, BQ24192_CHRG_CUR_CNTL_REG,
								reg->chr_cur);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);
		goto i2c_write_failed;
	}

	/* set charge voltage reg */
	ret = bq24192_write_reg(chip->client, BQ24192_CHRG_VOLT_CNTL_REG,
								reg->chr_volt);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);
		goto i2c_write_failed;
	}


	/* enable charger */
	ret = bq24192_reg_multi_bitset(chip->client, BQ24192_POWER_ON_CFG_REG,
						POWER_ON_CFG_CHRG_CFG_EN,
					CHR_CFG_BIT_POS, CHR_CFG_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* Program the pre-chrg & termination current limits */
	ret = bq24192_write_reg(chip->client,
				BQ24192_PRECHRG_TERM_CUR_CNTL_REG,
				(BQ24192_PRE_CHRG_CURR_256 |
				 BQ24192_TERM_CURR_LIMIT_128));
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

i2c_write_failed:

#ifdef DEBUG
	bq24192_dump_registers(chip);
#endif
	return ret;

}

static int stop_charging(struct bq24192_chip *chip)
{
	int ret;

	mutex_lock(&chip->event_lock);
	/* Disable the charger */
	ret = bq24192_reg_multi_bitset(chip->client, BQ24192_POWER_ON_CFG_REG,
						POWER_ON_CFG_CHRG_CFG_DIS,
					CHR_CFG_BIT_POS, CHR_CFG_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/*Enable  the WDT and Disable Safety timer */
	ret = program_timers(chip, true, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER enable failed\n");
	/*
	 * While disabling the charging, program the input source control
	 * register with minimum current ~100mA, so that in case Hi-Z clears
	 * and the charger resumes the charging immediately, the charger should
	 * not withdraw more than 100mA.
	 */
	ret = bq24192_write_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG,
							INPUT_SRC_CUR_LMT0);
	if (ret < 0) {
			dev_warn(&chip->client->dev,
						"Input src cntl write failed\n");
	}
	mutex_unlock(&chip->event_lock);
	return ret;
}

static int update_chrcurr_settings(struct bq24192_chip *chip, int chrg_lim)
{
	int ret;
	u8 in_src;

	if (chrg_lim == POWER_SUPPLY_CHARGE_CURRENT_LIMIT_ZERO) {
		ret = stop_charging(chip);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"charge disabling failed\n");
			goto uptd_chrg_set_exit;
		}
	}

	ret = chrg_lim_idx_to_chrg_cur(chrg_lim);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"invalid chrg limit index %d\n", chrg_lim);
		goto uptd_chrg_set_exit;
	}

	in_src = chrg_cur_to_reg(ret);
	/* set charge current reg with the limited index*/
	ret = bq24192_write_reg(chip->client,
			BQ24192_CHRG_CUR_CNTL_REG, in_src);
	if (ret < 0)
		dev_warn(&chip->client->dev,
				"I2C write failed:%s\n", __func__);

uptd_chrg_set_exit:
	return ret;
}

static void set_up_charging(struct bq24192_chip *chip,
		struct bq24192_chrg_regs *reg, int chr_curr, int chr_volt)
{
	int ret;

	reg->in_src = chrg_ilim_to_reg(chip->cap.mA);
	reg->chr_cur = chrg_cur_to_reg(chr_curr);
	reg->chr_volt = chrg_volt_to_reg(chr_volt);

	dev_info(&chip->client->dev, "%s: in_src=0x%x, chr_cur=0x%x, chr_volt=0x%x\n",
		__func__, reg->in_src, reg->chr_cur, reg->chr_volt);
	chip->input_curr = reg->in_src;
	/* Enable the WDT and Safety timer */
	ret = program_timers(chip, true, true);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "TIMER enable failed\n");
		goto i2c_error;
	}
	return;

i2c_error:
	dev_err(&chip->client->dev, "%s: I2C Error Value %d\n", __func__, ret);
}

/* check_batt_psy -check for whether power supply type is battery
 * @dev : Power Supply dev structure
 * @data : Power Supply Driver Data
 * Context: can sleep
 *
 * Return true if power supply type is battery
 *
 */
static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	/* check for whether power supply type is battery */
	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		fg_psy = psy;
		return 1;
	}
	return 0;
}

/**
 * get_fg_chip_psy - identify the Fuel Gauge Power Supply device
 * Context: can sleep
 *
 * Return Fuel Gauge power supply structure
 */
static struct power_supply *get_fg_chip_psy(void)
{
	if (fg_psy)
		return fg_psy;

	/* loop through power supply class */
	class_for_each_device(power_supply_class, NULL, NULL,
			check_batt_psy);
	return fg_psy;
}

/**
 * fg_chip_get_property - read a power supply property from Fuel Gauge driver
 * @psp : Power Supply property
 *
 * Return power supply property value
 *
 */
static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = get_fg_chip_psy();
	if (fg_psy) {
		ret = fg_psy->get_property(fg_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}

/*
 *This function will modify the VINDPM as per the battery voltage
 */
static int bq24192_modify_vindpm(u8 vindpm)
{
	int ret;
	u8 vindpm_prev;
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);

	dev_info(&chip->client->dev, "%s\n", __func__);

	/* Get the input src ctrl values programmed */
	ret = bq24192_read_reg(chip->client,
				BQ24192_INPUT_SRC_CNTL_REG);

	if (ret < 0) {
		dev_warn(&chip->client->dev, "INPUT CTRL reg read failed\n");
		return ret;
	}

	/* Assign the return value of REG00 to vindpm_prev */
	vindpm_prev = ret & INPUT_SRC_VINDPM_MASK;
	ret &= ~INPUT_SRC_VINDPM_MASK;

	/*
	 * If both the previous and current values are same do not program
	 * the register.
	*/
	if (vindpm_prev != vindpm) {
		vindpm |= ret;
		ret = bq24192_write_reg(chip->client,
					BQ24192_INPUT_SRC_CNTL_REG, vindpm);
		if (ret < 0) {
			dev_info(&chip->client->dev, "VINDPM failed\n");
			return ret;
		}
	}
	return ret;
}

/* check if charger automatically terminated charging
 * even when charging is enabled */
static bool bq24192_is_chrg_terminated(struct bq24192_chip *chip)
{
	bool is_chrg_term = false;
	int ret;

	dev_info(&chip->client->dev, "+%s\n", __func__);
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0) {
		dev_err(&chip->client->dev, "i2c read err:%d\n", ret);
		goto is_chrg_term_exit;
	}

	if ((ret & SYSTEM_STAT_CHRG_MASK) == SYSTEM_STAT_NOT_CHRG)
		is_chrg_term = true;
is_chrg_term_exit:
	return is_chrg_term;
}

/*
 * bq24192_do_charging - Programs the charger as per the charge current passed
 * curr -charging current value passed as per the platform current state
 */
static int bq24192_do_charging(int curr, int volt)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	struct bq24192_chrg_regs reg;
	int ret = 0, chr_curr;

	dev_info(&chip->client->dev, "+ %s\n", __func__);
	/*
	 * Check if user has enabled charging through sysfs
	 * If yes then program the charge current  as per the user
	 * configuration
	 */
	mutex_lock(&chip->event_lock);
	if (chip->chrg_cur_cntl == USER_SET_CHRG_LMT1)
		chr_curr = INPUT_CHRG_CURR_100;
	else if (chip->chrg_cur_cntl == USER_SET_CHRG_LMT2)
		chr_curr = INPUT_CHRG_CURR_500;
	else if (chip->chrg_cur_cntl == USER_SET_CHRG_LMT3)
		chr_curr = INPUT_CHRG_CURR_950;
	else if (chip->chrg_cur_cntl == USER_SET_CHRG_DISABLE) {
		dev_info(&chip->client->dev,
			"Charging is disabled via sysfs interface %s\n",
			 __func__);
		goto bq24192_do_charging_exit;
	} else /* USER_SET_CHRG_NOLMT */
		chr_curr = curr;

	/*
	 * Make sure we program the lesser of the current values
	 * to satisfy the thermal requirement for the platform
	 */
	if (chr_curr > curr)
		chr_curr = curr;
		dev_info(&chip->client->dev,
			"voltage = %d, current = %d, usr_chrg_enable = %d\n",
			volt, curr, chip->chrg_cur_cntl);

	if (chip->batt_mode != BATT_CHRG_FULL) {
		set_up_charging(chip, &reg, chr_curr, volt);

		ret = enable_charging(chip, &reg);
		if (ret < 0) {
			dev_err(&chip->client->dev, "enable charging failed\n");
		} else {
			dev_info(&chip->client->dev, "Charging enabled\n");
			/* cache the current charge voltage and current*/
			chip->curr_volt = volt;
			chip->curr_chrg = chr_curr;
		}
	} else {
		dev_info(&chip->client->dev, "Battery is full. Don't charge\n");
	}
bq24192_do_charging_exit:
	mutex_unlock(&chip->event_lock);
	return ret;
}

/**
 * bq24192_check_charge_full -  check battery is full or not
 * @vref: battery voltage
 *
 * Return true if full
 *
 */
static  bool bq24192_check_charge_full(struct bq24192_chip *chip, int vref)
{
	static int volt_prev;
	bool is_full = false;
	int volt_now;
	int cur_avg;
	int ret;

	/* Read voltage and current from FG driver */
	volt_now = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_OCV);
	if (volt_now == -ENODEV || volt_now == -EINVAL) {
		dev_warn(&chip->client->dev, "Can't read voltage from FG\n");
		return false;
	}

	/* convert to milli volts */
	volt_now /= 1000;

	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0) {
		dev_err(&chip->client->dev, "i2c read err:%d\n", ret);
		return is_full;
	}

	/*
	 * Check if the charger power is good. This will be used in deciding
	 * whether charging should be enabled or not.
	 */
	if (!(ret & SYSTEM_STAT_PWR_GOOD)) {
		chip->is_pwr_good = false;
		dev_info(&chip->client->dev, "Pwr is not good\n");
	} else {
		chip->is_pwr_good = true;
		dev_info(&chip->client->dev, "Pwr is good\n");
	}

	if ((ret & SYSTEM_STAT_CHRG_MASK) == SYSTEM_STAT_CHRG_DONE) {
			dev_info(&chip->client->dev,
					"FULL: HW based termination\n");
			is_full = true;
		return is_full;
	}

	/* Using Current-avg instead of Current-now to take care of
	 * instantaneous spike or dip */
	cur_avg = fg_chip_get_property(POWER_SUPPLY_PROP_CURRENT_AVG);
	if (cur_avg == -ENODEV || cur_avg == -EINVAL) {
		dev_warn(&chip->client->dev, "Can't read current-avg from FG\n");
		return false;
	}

	/* convert to milli amps */
	cur_avg /= 1000;

	/* voltage must be consistently above the vref threshold
	 * and current flow should be below a limit to confirm that
	 * battery is fully charged
	 */
	if ((volt_now >= (vref - CLT_VBATT_FULL_DET_MARGIN)) &&
	    (volt_prev >= (vref - CLT_VBATT_FULL_DET_MARGIN))  &&
		((cur_avg <= CLT_FULL_CURRENT_AVG_HIGH) &&
		(cur_avg > CLT_FULL_CURRENT_AVG_LOW))) {
			dev_info(&chip->client->dev,
					"FULL: SW based termination\n");
			is_full = true;
		} else
			is_full = false;

	volt_prev = volt_now;

	return is_full;
}

/*
 * bq24192_maintenance_worker
 * Maintenace worker thread monitors current voltage w.r.t temperature
 * and makes sure that we are within the current range. It also monitors user
 * based overriding control and gives higher priority to the same
 */
static void bq24192_maintenance_worker(struct work_struct *work)
{
	int ret, batt_temp, battery_status, idx = 0, vbatt = 0;
	struct bq24192_chip *chip = container_of(work,
				struct bq24192_chip, maint_chrg_wrkr.work);
	short int cv = 0, usr_cc = -1;
	struct ctp_temp_mon_table *temp_mon = NULL;
	bool is_chrg_term = false, is_chrg_full = false;
	static int chrg_cur_cntl = USER_SET_CHRG_NOLMT;
	bool sysfs_stat = false;
	u8 vindpm = INPUT_SRC_VOLT_LMT_DEF;
	int batt_health = POWER_SUPPLY_HEALTH_GOOD;

	dev_dbg(&chip->client->dev, "+ %s\n", __func__);

	mutex_lock(&chip->event_lock);
	ret = reset_wdt_timer(chip);
	if (ret < 0)
		dev_warn(&chip->client->dev, "WDT reset failed\n");
	mutex_unlock(&chip->event_lock);

	/*
	 * Jump to the label in case of battery emulator
	 * Do not do additional unneccessary work
	 */
	if (!chip->pdata->sfi_tabl_present)
		goto sched_maint_work;
	/*
	 * We update the battery charging status as per the type of
	 * charger connected. If it is host mode cable connected then
	 * battery status should be discharging
	 */
	if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_HOST) {
		dev_info(&chip->client->dev, "Charger type Host\n");
		goto sched_maint_work;
	}

	/* Check if we have the charger present */
	if (chip->present) {
		dev_info(&chip->client->dev,
			"Charger is present\n");
	} else {
		dev_info(&chip->client->dev,
				"Charger is not present. Schedule worker\n");
		goto sched_maint_work;
	}

	/* read the temperature via adc */
	ret = ctp_read_adc_temp(&batt_temp);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to acquire batt temp\n");
		goto sched_maint_work;
	}

	/* find the temperature range */
	idx = ctp_sfi_temp_range_lookup(batt_temp);
	if (idx == -1) {
		dev_warn(&chip->client->dev,
			"battery temperature is outside the designated zones\n");

		if (batt_temp < chip->batt_thrshlds.temp_low) {
			dev_info(&chip->client->dev,
				"batt temp:POWER_SUPPLY_HEALTH_COLD\n");
		} else {
			dev_info(&chip->client->dev,
				"batt temp:POWER_SUPPLY_HEALTH_OVERHEAT\n");
		}

		batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		/*
		 * Read the Power-ON Cfg register to ensure charging disabled
		 * If already disabled. No need to disable again
		 */
		ret = bq24192_read_reg(chip->client, BQ24192_POWER_ON_CFG_REG);
		if (ret < 0) {
			dev_warn(&chip->client->dev,
					"%s I2C read failed\n", __func__);
			goto sched_maint_work;
		}

		if (ret & POWER_ON_CFG_CHRG_CFG_EN) {
			/*
			 * The battery profile does not support -ve temperature
			 * charging. Stop the charging explicitely
			 */
			ret = stop_charging(chip);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"Stop charging failed\n");
			}
		} else {
			dev_warn(&chip->client->dev, "Already Stopped Charging\n");
		}
		goto sched_maint_work;
	} else {
		if (chip->batt_mode != BATT_CHRG_FULL) {
			mutex_lock(&chip->event_lock);
			ret = bq24192_reg_multi_bitset(chip->client,
						BQ24192_POWER_ON_CFG_REG,
						POWER_ON_CFG_CHRG_CFG_EN,
						CHR_CFG_BIT_POS,
						CHR_CFG_BIT_LEN);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
					"I2C write failed:%s\n", __func__);
			}
			mutex_unlock(&chip->event_lock);
		}
		batt_health = POWER_SUPPLY_HEALTH_GOOD;
	}

	dev_info(&chip->client->dev, "temperature zone idx = %d\n", idx);

	/* read the battery voltage */
	vbatt = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_OCV);
	if (vbatt == -ENODEV || vbatt == -EINVAL) {
		dev_err(&chip->client->dev, "Can't read voltage from FG\n");
		goto sched_maint_work;
	}

	/* convert voltage into millivolts */
	vbatt /= 1000;
	dev_info(&chip->client->dev, "vbatt = %d\n", vbatt);

	if (vbatt > INPUT_SRC_LOW_VBAT_LIMIT &&
		vbatt < INPUT_SRC_MID_VBAT_LIMIT)
		vindpm = INPUT_SRC_VOLT_LMT_444;
	else if (vbatt > INPUT_SRC_MID_VBAT_LIMIT &&
		vbatt < INPUT_SRC_HIG_VBAT_LIMIT)
		vindpm = INPUT_SRC_VOLT_LMT_468;

	mutex_lock(&chip->event_lock);
	ret = bq24192_modify_vindpm(vindpm);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s failed\n", __func__);
	mutex_unlock(&chip->event_lock);

	/* read the charge current based upon user setting */
	if (chip->chrg_cur_cntl != chrg_cur_cntl) {
		usr_cc = chrg_lim_idx_to_chrg_cur(chip->chrg_cur_cntl);
		chrg_cur_cntl = chip->chrg_cur_cntl;
		sysfs_stat = true;
		dev_info(&chip->client->dev,
			"change in user setting %d usr_cc = %d\n",
			chip->chrg_cur_cntl, usr_cc);
	}

	/*
	 * A temporary work around to do maintenance charging until we
	 * we get the entries in SFI table
	 */
	if (!chip->pdata->sfi_tabl_present) {
		dev_info(&chip->client->dev, "Using Platform data table\n");
		temp_mon = &chip->pdata->temp_mon_range[idx];
	} else {
		dev_info(&chip->client->dev, "Using SFI table data\n");
		temp_mon = &ctp_sfi_table->temp_mon_range[idx];
	}

	/* Read the charger status bit for charge complete */
	is_chrg_term = bq24192_is_chrg_terminated(chip);

	if (chip->batt_mode == BATT_CHRG_MAINT)
		cv = temp_mon->maint_chrg_vol_ul;
	else
		cv = temp_mon->full_chrg_vol;


	if (chip->batt_mode == BATT_CHRG_FULL)
		is_chrg_full = true;
	else
		/* check if the charge is full */
		is_chrg_full = bq24192_check_charge_full(chip, cv);

	dev_info(&chip->client->dev,
		"charge_full=%d charging mode = %d is_chrg_term = %d\n",
		is_chrg_full, chip->batt_mode, is_chrg_term);

	switch (chip->batt_mode) {
	case BATT_CHRG_NONE:
		goto sched_maint_work;
	case BATT_CHRG_NORMAL:
		if (is_chrg_full == true) {
			dev_info(&chip->client->dev, "Charge is Full or terminated\n");
			ret = stop_charging(chip);
			if (ret < 0) {
				dev_info(&chip->client->dev,
					"Stop charging failed:%s\n", __func__);
				goto sched_maint_work;
			}
			mutex_lock(&chip->event_lock);
			/*
			 * This flag should be updated only when the battery
			 * has reached full and charging has been terminated
			 */
			chip->batt_mode = BATT_CHRG_FULL;
			mutex_unlock(&chip->event_lock);
		} else if ((sysfs_stat == true)) {
			/* If there is change in temperature zone
			 * or user mode charge current settings */
			ret = bq24192_do_charging(
				temp_mon->full_chrg_cur,
				temp_mon->full_chrg_vol);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
					"do_charing failed:\n");
				goto sched_maint_work;
			}
		}
		break;
	case BATT_CHRG_FULL:
		if (vbatt <= temp_mon->maint_chrg_vol_ll) {
			dev_info(&chip->client->dev,
				"vbatt is lower than maint_chrg_vol_ll\n");
			mutex_lock(&chip->event_lock);
			chip->batt_mode = BATT_CHRG_MAINT;
			mutex_unlock(&chip->event_lock);
			ret = bq24192_do_charging(
				temp_mon->maint_chrg_cur,
				temp_mon->maint_chrg_vol_ul);
			if (ret < 0) {
				dev_warn(&chip->client->dev, "do_charing failed\n");
				goto sched_maint_work;
			}
		}
		break;
	case BATT_CHRG_MAINT:
		dev_info(&chip->client->dev,
			"Current batt_mode : BATT_CHRG_MAINT\n");
		if (is_chrg_full == true) {
			/* Need to stop charging */
			ret = stop_charging(chip);
			if (ret < 0) {
				dev_warn(&chip->client->dev, "do_charing failed\n");
				goto sched_maint_work;
			}
			mutex_lock(&chip->event_lock);
			chip->batt_mode = BATT_CHRG_FULL;
			mutex_unlock(&chip->event_lock);
		} else if ((vbatt <= temp_mon->maint_chrg_vol_ll) &&
			(vbatt > (temp_mon->maint_chrg_vol_ll - RANGE))) {
			dev_info(&chip->client->dev,
				"Discharging and withing maintenance mode range\n");

			/* if within the range */
			if ((sysfs_stat == true)) {
				dev_info(&chip->client->dev,
					"Change in Temp Zone or User Setting:\n");
				ret = bq24192_do_charging(
					temp_mon->maint_chrg_cur,
					temp_mon->maint_chrg_vol_ul);
				if (ret < 0) {
					dev_warn(&chip->client->dev, "do_charing failed\n");
					goto sched_maint_work;
				}
			}
		} else if (vbatt <= temp_mon->maint_chrg_vol_ll - RANGE) {
			dev_info(&chip->client->dev,
				"vbatt less then low voltage threshold\n");
			/* This can happen because of more current being
			 * drawn then  maintenance mode charging charges at
			 */
			ret = bq24192_do_charging(
					temp_mon->full_chrg_cur,
					temp_mon->full_chrg_vol);
			if (ret < 0) {
				dev_warn(&chip->client->dev, "do_charing failed\n");
				goto sched_maint_work;
			}
			mutex_lock(&chip->event_lock);
			chip->batt_mode = BATT_CHRG_NORMAL;
			mutex_unlock(&chip->event_lock);
		} else if (sysfs_stat == true) {
			/* override if non of the condition succeeds
			 * This can happen if none of the cases match but
			 * there is a in that case we must run the
			 * maintenance mode settings again.
			 */
			dev_info(&chip->client->dev,
				"Override chrg params with User conifig\n");

			/* fetch the current voltage being driven */
			ret = bq24192_read_reg(chip->client,
				BQ24192_CHRG_VOLT_CNTL_REG);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
					"Charger Voltage register read failed\n");
				goto sched_maint_work;
			}
			cv = BQ24192_GET_CHRG_VOLT(ret);
			usr_cc = (usr_cc > 0) ? usr_cc :
				temp_mon->full_chrg_cur;
			ret = bq24192_do_charging(usr_cc, cv);
			if (ret < 0) {
				dev_warn(&chip->client->dev, "do_charing failed\n");
				goto sched_maint_work;
			}
		}
		break;
	default:
		dev_warn(&chip->client->dev, "Invalid charing mode\n");
		goto sched_maint_work;
	}

sched_maint_work:
	if ((chip->batt_mode == BATT_CHRG_MAINT) ||
	    (chip->batt_mode == BATT_CHRG_FULL)) {
		battery_status = POWER_SUPPLY_STATUS_FULL;
		/* When the charge status is FULL the FG interrupts
		 * are disabled so sending power supply notification
		 * to UI
		 */
		dev_info(&chip->client->dev, "Charge status FUll\n");
		power_supply_changed(&chip->usb);
	} else {
		battery_status = POWER_SUPPLY_STATUS_CHARGING;
	}

	/*
	 * Update the UI per the current battery/charger status
	 */
	if (((is_chrg_term == true) && (is_chrg_full == false) &&
		(chip->batt_mode == BATT_CHRG_NORMAL)) || (idx == -1))
		battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;


	if ((chip->chrg_type == POWER_SUPPLY_TYPE_USB_HOST))
		battery_status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (!chip->is_pwr_good)
		battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING) ||
		(battery_status == POWER_SUPPLY_STATUS_DISCHARGING)) {
		chip->online = 0;
	} else {
		chip->online = 1;
	}

	dev_info(&chip->client->dev, "battery_status %dchip->batt_status %d\n",
			battery_status, chip->batt_status);

	if ((chip->batt_status != battery_status) ||
		(chip->batt_health != batt_health)) {
		mutex_lock(&chip->event_lock);
		chip->batt_status = battery_status;
		chip->batt_health = batt_health;
		mutex_unlock(&chip->event_lock);
		power_supply_changed(&chip->usb);
	}

	schedule_delayed_work(&chip->maint_chrg_wrkr, MAINTENANCE_CHRG_JIFFIES);
	dev_info(&chip->client->dev, "battery mode is  %d\n", chip->batt_mode);
	dev_dbg(&chip->client->dev, "- %s\n", __func__);
}

/* This function should be called with the mutex held */
static int bq24192_turn_otg_vbus(struct bq24192_chip *chip, bool votg_on)
{
	int ret = 0;

	if (votg_on) {
			/* Program the timers */
			ret = program_timers(chip, true, false);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
					"TIMER enable failed %s\n", __func__);
				goto i2c_write_fail;
			}
			/* Configure the charger in OTG mode */
			ret = bq24192_reg_read_modify(chip->client,
					BQ24192_POWER_ON_CFG_REG,
					POWER_ON_CFG_CHRG_CFG_OTG, true);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"read reg modify failed\n");
				goto i2c_write_fail;
			}

			/* Put the charger IC in reverse boost mode. Since
			 * SDP charger can supply max 500mA charging current
			 * Setting the boost current to 500mA
			 */
			ret = bq24192_reg_read_modify(chip->client,
					BQ24192_POWER_ON_CFG_REG,
					POWER_ON_CFG_BOOST_LIM, false);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"read reg modify failed\n");
				goto i2c_write_fail;
			}
			/* assert the chrg_otg gpio now */
			gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 1);
	} else {
			/* Clear the charger from the OTG mode */
			ret = bq24192_reg_read_modify(chip->client,
					BQ24192_POWER_ON_CFG_REG,
					POWER_ON_CFG_CHRG_CFG_OTG, false);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"read reg modify failed\n");
				goto i2c_write_fail;
			}

			/* Put the charger IC out of reverse boost mode 500mA */
			ret = bq24192_reg_read_modify(chip->client,
					BQ24192_POWER_ON_CFG_REG,
					POWER_ON_CFG_BOOST_LIM, false);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"read reg modify failed\n");
				goto i2c_write_fail;
			}

			/* de-assert the chrg_otg gpio now */
			gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 0);
			gpio_direction_input(BQ24192_CHRG_OTG_GPIO);
	}
i2c_write_fail:
	return ret;
}

static void bq24192_event_worker(struct work_struct *work)
{
	struct bq24192_chip *chip = container_of(work,
				struct bq24192_chip, chrg_evt_wrkr.work);
	int ret;
	int disconnected = 0;

	dev_info(&chip->client->dev, "%s\n", __func__);

	switch (chip->cap.chrg_evt) {
	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
		pm_runtime_get_sync(&chip->client->dev);
	case POWER_SUPPLY_CHARGER_EVENT_UPDATE:
	case POWER_SUPPLY_CHARGER_EVENT_RESUME:
		if ((chip->chrg_cur_cntl == USER_SET_CHRG_DISABLE) &&
			(chip->chrg_cur_cntl == chip->cached_chrg_cur_cntl)) {
			/* cache the charging parameters as this has come from
			 * USB OTG driver. Typically ends up here when we have
			 * disabled charging through sysfs and connect charger
			 */
			dev_info(&chip->client->dev, "cache the charging parameters");
			dev_info(&chip->client->dev, "notification from USB driver\n");
			mutex_lock(&chip->event_lock);
			chip->cached_cap = chip->cap;
			mutex_unlock(&chip->event_lock);
			break;
		} else if ((chip->cached_chrg_cur_cntl !=
				chip->chrg_cur_cntl) &&
			    (chip->chrg_cur_cntl !=
				USER_SET_CHRG_DISABLE)) {
			/* This is a event generated by exterme charging sysfs
			 * interface restore the cacehd parameter and exit
			 * the switch case */
			mutex_lock(&chip->event_lock);
			chip->cap = chip->cached_cap;
			mutex_unlock(&chip->event_lock);
			dev_info(&chip->client->dev,
				"event generated by sysfs interface\n");
			/* 1. Check the previous power state of USB hardware */
			if ((chip->cached_cap.chrg_evt ==
				POWER_SUPPLY_CHARGER_EVENT_SUSPEND) ||
			    (chip->cached_cap.chrg_evt ==
				POWER_SUPPLY_CHARGER_EVENT_DISCONNECT)) {
				/* In this case the charger is not
				 * attached or is suspended and hence we
				 * will not resume charging
				 */
				dev_dbg(&chip->client->dev,
				"Charger not attached, dnt resume charging\n");
				break;
			}
		}

		ret = bq24192_clear_hiz(chip);
		if (ret < 0) {
			dev_warn(&chip->client->dev, "%s Hi-Z clear failed\n",
								__func__);
			goto i2c_write_fail;
		}

		/* updating this because we have resumed charging */
		mutex_lock(&chip->event_lock);
		chip->cached_cap = chip->cap;
		mutex_unlock(&chip->event_lock);

		if (chip->cap.chrg_type != POWER_SUPPLY_TYPE_USB_HOST) {
			dev_info(&chip->client->dev, "Enable charging\n");
			mutex_lock(&chip->event_lock);
			chip->batt_mode = BATT_CHRG_NORMAL;
			mutex_unlock(&chip->event_lock);
			/* This is the condition where event has occured
			 * because of SYSFS change or USB driver */
			if ((chip->curr_volt == BQ24192_INVALID_VOLT) ||
				(chip->curr_chrg == BQ24192_INVALID_CURR))
				ret = bq24192_do_charging(BQ24192_DEF_CHRG_CUR,
					BQ24192_DEF_VBATT_MAX);
			else
				ret = bq24192_do_charging(chip->curr_chrg,
					chip->curr_volt);

			if (ret < 0) {
				dev_err(&chip->client->dev,
					"charge enabling failed\n");
				goto i2c_write_fail;
			}

			mutex_lock(&chip->event_lock);
			chip->present = 1;
			chip->online = 1;
			mutex_unlock(&chip->event_lock);
		}

		mutex_lock(&chip->event_lock);
		chip->chrg_type = chip->cap.chrg_type;
		if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_DCP) {
			chip->usb.type = POWER_SUPPLY_TYPE_USB_DCP;
			dev_info(&chip->client->dev,
				 "Charger type DCP\n");
		} else if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_CDP) {
			chip->usb.type = POWER_SUPPLY_TYPE_USB_CDP;
			dev_info(&chip->client->dev,
				"Charger type CDP\n");
		} else if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_ACA) {
			chip->usb.type = POWER_SUPPLY_TYPE_USB_ACA;
			dev_info(&chip->client->dev,
				"Charger type ACA\n");
		} else if (chip->chrg_type == POWER_SUPPLY_TYPE_USB) {
			chip->usb.type = POWER_SUPPLY_TYPE_USB;
			dev_info(&chip->client->dev,
				 "Charger type SDP\n");
		} else if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_INVAL) {
			chip->usb.type = POWER_SUPPLY_TYPE_USB_INVAL;
			dev_info(&chip->client->dev,
				 "Charger type INVAL SDP\n");
		} else if (chip->chrg_type == POWER_SUPPLY_TYPE_USB_HOST) {
			dev_info(&chip->client->dev,
				 "Charger type USB HOST\n");
			ret = bq24192_turn_otg_vbus(chip, true);
			if (ret < 0) {
				dev_err(&chip->client->dev,
				"turning OTG vbus ON failed\n");
				mutex_unlock(&chip->event_lock);
				goto i2c_write_fail;
			}

			/* otg vbus is turned ON */
			chip->votg = true;
		} else {
			dev_info(&chip->client->dev,
				 "Unknown Charger type\n");
		}

		/*
		 * We update the battery charging status as per the type of
		 * charger connected. If it is host mode cable connected then
		 * battery status should be discharging
		 */
		if (chip->chrg_type != POWER_SUPPLY_TYPE_USB_HOST) {
			chip->batt_status = POWER_SUPPLY_STATUS_CHARGING;
		}

		mutex_unlock(&chip->event_lock);

		/* Schedule the maintenance now */
		schedule_delayed_work(&chip->maint_chrg_wrkr,
							INITIAL_THREAD_JIFFY);
		/*
		 * Prevent system from entering s3 while charger is connected
		 * or if any OTG device (mouse/keyboard) is connected.
		 */
		if (!wake_lock_active(&chip->wakelock))
			wake_lock(&chip->wakelock);
		/*
		 * Assert the chrg_otg gpio now. This will ensure that the
		 * current pulled out from VBUS is ~500mA in case of SDP
		 */
		gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 1);
		break;
	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
		disconnected = 1;
		pm_runtime_put_sync(&chip->client->dev);
		/* Cancel the maintenance worker here */
		cancel_delayed_work_sync(&chip->maint_chrg_wrkr);
	case POWER_SUPPLY_CHARGER_EVENT_SUSPEND:
		dev_info(&chip->client->dev, "Disable charging\n");
		ret = stop_charging(chip);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"charge disabling failed\n");
			goto i2c_write_fail;
		}
		mutex_lock(&chip->event_lock);
		if (chip->cap.chrg_evt ==
			POWER_SUPPLY_CHARGER_EVENT_SUSPEND) {
			chip->present = 1;
		} else {
			chip->present = 0;
			chip->chrg_type = chip->cap.chrg_type;
			chip->usb.type = POWER_SUPPLY_TYPE_USB;
		}
		chip->online = 0;
		chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		if (chip->votg) {
				ret = bq24192_turn_otg_vbus(chip, false);
				if (ret < 0) {
					dev_err(&chip->client->dev,
						"turning OTG vbus OFF failed\n");

					mutex_unlock(&chip->event_lock);
					goto i2c_write_fail;
				}

				/* otg vbus is turned OFF */
				chip->votg = false;
		}

		/* release the wake lock when charger is unplugged */
		if (wake_lock_active(&chip->wakelock))
			wake_unlock(&chip->wakelock);
		chip->batt_mode = BATT_CHRG_NORMAL;

		/* Cache all the parameters */
		chip->curr_volt = BQ24192_INVALID_VOLT;
		chip->curr_chrg = BQ24192_INVALID_CURR;
		/* update the caps if it's a notification coming from USB
		 * driver, since in that case exterme charging parameter
		 * will remain the same and caps must change.
		 */
		if (disconnected) {
			dev_info(&chip->client->dev, "Cached chip->cap\n");
			chip->cached_cap = chip->cap;
		} else
			dev_info(&chip->client->dev, "dnt Cache chip->cap\n");

		chip->cached_chrg_cur_cntl = chip->chrg_cur_cntl;

		mutex_unlock(&chip->event_lock);
		/* de-assert the chrg_otg gpio now */
		gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 0);
		gpio_direction_input(BQ24192_CHRG_OTG_GPIO);
		break;
	default:
		dev_err(&chip->client->dev,
			"invalid charger event:%d\n", chip->cap.chrg_evt);
		goto i2c_write_fail;
	}

	power_supply_changed(&chip->usb);
i2c_write_fail:
	return ;
}

int bq24192_slave_mode_enable_charging(int volt, int cur, int ilim)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	struct bq24192_chrg_regs reg;
	int ret;

	reg.in_src = chrg_ilim_to_reg(ilim);
	reg.chr_cur = chrg_cur_to_reg(cur);
	reg.chr_volt = chrg_volt_to_reg(volt);

	mutex_lock(&chip->event_lock);
	ret = enable_charging(chip, &reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "charge enable failed\n");

	mutex_unlock(&chip->event_lock);
	return ret;
}
EXPORT_SYMBOL(bq24192_slave_mode_enable_charging);

int bq24192_slave_mode_disable_charging(void)
{
	struct bq24192_chip *chip = i2c_get_clientdata(bq24192_client);
	int ret;

	ret = stop_charging(chip);
	if (ret < 0)
		dev_err(&chip->client->dev, "charge disable failed\n");
	return ret;
}
EXPORT_SYMBOL(bq24192_slave_mode_disable_charging);

static void bq24192_charging_port_changed(struct power_supply *psy,
				struct power_supply_charger_cap *cap)
{
	int ret;
	struct bq24192_chip *chip = container_of(psy,
				struct bq24192_chip, usb);

	mutex_lock(&chip->event_lock);
	chip->cap.chrg_evt = cap->chrg_evt;
	chip->cap.chrg_type = cap->chrg_type;
	chip->cap.mA = cap->mA;
	mutex_unlock(&chip->event_lock);

	dev_info(&chip->client->dev, "[chrg] evt:%d type:%d cur:%d\n",
				cap->chrg_evt, cap->chrg_type, cap->mA);
	/*
	 * If we have a battery emulator connected, disable the charging
	 */
	if (!chip->pdata->sfi_tabl_present) {
		ret = stop_charging(chip);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"%s charge disabling failed\n", __func__);
		}

		/*
		 * If the evt type is connect, schedule the maintenance which
		 * maintains the battery state machine, WDT reset etc
		 * else if the evt type is disconnect, cancel the maintenance
		 */
		if (cap->chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT)
			schedule_delayed_work(&chip->maint_chrg_wrkr, 0);
		else
		if (cap->chrg_evt == POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
			chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
			power_supply_changed(&chip->usb);
			cancel_delayed_work_sync(&chip->maint_chrg_wrkr);
		}
	} else
		schedule_delayed_work(&chip->chrg_evt_wrkr, 0);

}

#ifdef CONFIG_DEBUG_FS
#define DBGFS_REG_BUF_LEN	3

static int bq24192_show(struct seq_file *seq, void *unused)
{
	u16 val;
	long addr;

	if (kstrtol((char *)seq->private, 16, &addr))
		return -EINVAL;

	val = bq24192_read_reg(bq24192_client, addr);
	seq_printf(seq, "%x\n", val);

	return 0;
}

static int bq24192_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq24192_show, inode->i_private);
}

static ssize_t bq24192_dbgfs_reg_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[DBGFS_REG_BUF_LEN];
	long addr;
	unsigned long value;
	int ret;
	struct seq_file *seq = file->private_data;

	if (!seq || kstrtol((char *)seq->private, 16, &addr))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, DBGFS_REG_BUF_LEN-1))
		return -EFAULT;

	buf[DBGFS_REG_BUF_LEN-1] = '\0';
	if (kstrtoul(buf, 16, &value))
		return -EINVAL;

	dev_info(&bq24192_client->dev,
			"[dbgfs write] Addr:0x%x Val:0x%x\n",
			(u32)addr, (u32)value);


	ret = bq24192_write_reg(bq24192_client, addr, value);
	if (ret < 0)
		dev_warn(&bq24192_client->dev, "I2C write failed\n");

	return count;
}

static const struct file_operations bq24192_dbgfs_fops = {
	.owner		= THIS_MODULE,
	.open		= bq24192_dbgfs_open,
	.read		= seq_read,
	.write		= bq24192_dbgfs_reg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int bq24192_create_debugfs(struct bq24192_chip *chip)
{
	int i;
	struct dentry *entry;

	bq24192_dbgfs_root = debugfs_create_dir(DEV_NAME, NULL);
	if (IS_ERR(bq24192_dbgfs_root)) {
		dev_warn(&chip->client->dev, "DEBUGFS DIR create failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < BQ24192_MAX_MEM; i++) {
		sprintf((char *)&bq24192_dbg_regs[i], "%x", i);
		entry = debugfs_create_file(
					(const char *)&bq24192_dbg_regs[i],
					S_IRUGO,
					bq24192_dbgfs_root,
					&bq24192_dbg_regs[i],
					&bq24192_dbgfs_fops);
		if (IS_ERR(entry)) {
			debugfs_remove_recursive(bq24192_dbgfs_root);
			bq24192_dbgfs_root = NULL;
			dev_warn(&chip->client->dev,
					"DEBUGFS entry Create failed\n");
			return -ENOMEM;
		}
	}

	return 0;
}
static inline void bq24192_remove_debugfs(struct bq24192_chip *chip)
{
	if (bq24192_dbgfs_root)
		debugfs_remove_recursive(bq24192_dbgfs_root);
}
#else
static inline int bq24192_create_debugfs(struct bq24192_chip *chip)
{
	return 0;
}
static inline void bq24192_remove_debugfs(struct bq24192_chip *chip)
{
}
#endif

static int bq24192_usb_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct bq24192_chip *chip = container_of(psy,
				struct bq24192_chip, usb);

	mutex_lock(&chip->event_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = chip->chrg_type;
		break;
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT:
		val->intval = chip->chrg_cur_cntl;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chip->present)
			val->intval = ctp_get_charger_health();
		else
			val->intval = 0;
		break;
	default:
		mutex_unlock(&chip->event_lock);
		return -EINVAL;
	}
	mutex_unlock(&chip->event_lock);

	return 0;
}

static int bq24192_usb_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct bq24192_chip *chip = container_of(psy,
				struct bq24192_chip, usb);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT:
		ret = update_chrcurr_settings(chip, val->intval);
		if (ret < 0)
			goto usb_set_prop_exit;

		mutex_lock(&chip->event_lock);
		chip->chrg_cur_cntl = val->intval;
		mutex_unlock(&chip->event_lock);
		break;
	default:
		ret = -EPERM;
	}
usb_set_prop_exit:
	return ret;
}

static int bq24192_usb_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * This function checks the safety temperature limit and return the
 * appropriate value to program the Charger Master Temp Ctrl register
 */
static u8 ctp_get_safechrglimit(short int temp_high, short int temp_low)
{
	u8 sftmp = 0;

	/* Get the higher temp values */
	if (temp_high >= 60)
		sftmp |= CHRTMPCTRL_TMPH_60;
	else if (temp_high >= 55)
		sftmp |= CHRTMPCTRL_TMPH_55;
	else if (temp_high >= 50)
		sftmp |= CHRTMPCTRL_TMPH_50;
	else
		sftmp |= CHRTMPCTRL_TMPH_45;

	/* Get the lower temp values */
	if (temp_low >= 15)
		sftmp |= CHRTMPCTRL_TMPL_15;
	else if (temp_low >= 10)
		sftmp |= CHRTMPCTRL_TMPL_10;
	else if (temp_low >= 5)
		sftmp |= CHRTMPCTRL_TMPL_05;
	else
		sftmp |= CHRTMPCTRL_TMPL_00;

	return sftmp;
}
/*
 * This function programs the Charger Master Temperature Control
 * register with the temp value passed as parameter
 */
static int ctp_set_safechrglimit(short int temp)
{
	int ret = 0;
	unsigned short addr = MSIC_CHRTMPCTRL;

	/*
	 * Program the Charger Master Temperature control register
	 * with the temperature given
	 */
	ret = intel_scu_ipc_iowrite8(addr, temp);
	if (ret) {
		dev_warn(&bq24192_client->dev,
				"IPC Failed with %d error\n", ret);
	}
	return ret;
}

/**
 * init_batt_thresholds - initialize battery thresholds
 * @chip: charger driver device context
 * Context: can sleep
 */
static void init_batt_thresholds(struct bq24192_chip *chip)
{
	int ret;
	u8 validate_smip_data[4] = {0};
	u8 safetmp = 0;

	/* Read the Signature verification data form SRAM */
	ret = intel_scu_ipc_read_mip((u8 *)validate_smip_data,
					4,
					SMIP_SRAM_OFFSET_ADDR, 1);
	if (ret)
		dev_warn(&chip->client->dev, "smip read failed\n");

	dev_dbg(&chip->client->dev, "SBCT_REV: 0x%x RSYS_OHMS 0x%x",
			validate_smip_data[0], validate_smip_data[3]);

	/* Check for Valid SMIP data */
	if ((validate_smip_data[0] == SBCT_REV) && (
			validate_smip_data[3] == RSYS_MOHMS)) {

		dev_info(&chip->client->dev,
			"%s: Valid SMIP data\n", __func__);
		/* Read the threshold values from SRAM */
		ret = intel_scu_ipc_read_mip((u8 *)&chip->batt_thrshlds,
						sizeof(chip->batt_thrshlds),
						SMIP_SRAM_OFFSET_ADDR, 1);
		if (ret)
			dev_warn(&chip->client->dev,
				"%s:smip read failed\n", __func__);
		vbatt_sh_min = chip->batt_thrshlds.vbatt_sh_min;
	} else {
		chip->batt_thrshlds.vbatt_sh_min = CLT_BATT_VMIN_THRESHOLD_DEF;
		chip->batt_thrshlds.vbatt_crit = CLT_BATT_CRIT_CUTOFF_VOLT_DEF;
		chip->batt_thrshlds.temp_high = CLT_BATT_TEMP_MAX_DEF;
		chip->batt_thrshlds.temp_low = CLT_BATT_TEMP_MIN_DEF;
		dev_dbg(&chip->client->dev,
			"%s: No signature match	for SRAM Read\n ", __func__);
	}

	ctp_get_batt_temp_thresholds(&chip->batt_thrshlds.temp_high,
		&chip->batt_thrshlds.temp_low);
	/*
	 * Get the temperature limits to be programmed
	 * for safe charging
	 */
	safetmp = ctp_get_safechrglimit(chip->batt_thrshlds.temp_high,
						chip->batt_thrshlds.temp_low);
	/* Program the Charger Master Temperature Control register */
	ret = ctp_set_safechrglimit(safetmp);
	if (ret)
		dev_warn(&chip->client->dev, "Failed to set the SAFETEMP\n");
}

/* IRQ handler for charger Interrupts configured to GPIO pin */
static irqreturn_t mask_gpio_irq(int irq, void *devid)
{
	struct bq24192_chip *chip = (struct bq24192_chip *)devid;
	/**TODO: This hanlder will be used for charger Interrupts */
	dev_dbg(&chip->client->dev,
		"IRQ Handled for charger interrupt: %d\n", irq);

	return IRQ_HANDLED;
}

bool ctp_is_volt_shutdown_enabled(void)
{
	/* FPO1 is reserved in case of CTP so we are returning true */
	return true;
}
EXPORT_SYMBOL(ctp_is_volt_shutdown_enabled);

int ctp_get_vsys_min(void)
{
	if (vbatt_sh_min)
		return vbatt_sh_min * 1000;

	return CLT_BATT_VMIN_THRESHOLD_DEF * 1000;
}
EXPORT_SYMBOL(ctp_get_vsys_min);

static int bq24192_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bq24192_chip *chip;
	int ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform Data is NULL");
		return -EFAULT;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"SMBus doesn't support BYTE transactions\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct bq24192_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, chip);
	bq24192_client = client;
	ret = bq24192_read_reg(client, BQ24192_VENDER_REV_REG);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read err:%d\n", ret);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return -EIO;
	}

	/* D3, D4, D5 indicates the chip model number */
	ret = (ret >> 3) & 0x07;
	if ((ret != BQ24192I_IC_VERSION) &&
		(ret != BQ24192_IC_VERSION) &&
		(ret != BQ24191_IC_VERSION) &&
		(ret != BQ24190_IC_VERSION)) {
		dev_err(&client->dev, "device version mismatch: %x\n", ret);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return -EIO;
	}

	/* Enable the WDT and Disable Safety timer */
	ret = program_timers(chip, true, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER enable failed\n");

	ret = device_create_file(&chip->client->dev,
		&dev_attr_charge_current_limit);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to create sysfs:charge_current_limit\n");
	}

	ret = gpio_request(BQ24192_CHRG_OTG_GPIO, "CHRG_OTG");
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to request gpio %d with error %d\n",
			BQ24192_CHRG_OTG_GPIO, ret);
	}
	dev_info(&chip->client->dev, "request gpio %d for CHRG_OTG pin\n",
			BQ24192_CHRG_OTG_GPIO);

	/* Register a handler for Charger Interrupt gpio pin */
	ret = request_irq(gpio_to_irq(CHRG_INT_N),
			mask_gpio_irq, 0, "bq24192", chip);
	if (ret) {
		dev_warn(&bq24192_client->dev,
		"failed to register charger irq for pin %d\n", CHRG_INT_N);
	} else {
		/* FIXME This will be reverted once MIP XML
		changes are available */
		free_irq(gpio_to_irq(CHRG_INT_N), chip);
		dev_dbg(&bq24192_client->dev,
			"registered charger irq for pin %d\n", CHRG_INT_N);
	}
	/*
	 * FIXME Clear the CHRDIS bit in MSIC_CHRCRTL register to enable
	 * the charging. There is a bug in HW because of that charging gets
	 * disabled when the platform boot with the SDP charger connected.
	 */
	ret = intel_scu_ipc_iowrite8(MSIC_CHRCRTL, MSIC_CHRGENBL);
	if (ret) {
		dev_warn(&bq24192_client->dev,
				"IPC Failed with %d error\n", ret);
	}

	INIT_DELAYED_WORK(&chip->chrg_evt_wrkr, bq24192_event_worker);
	INIT_DELAYED_WORK(&chip->maint_chrg_wrkr, bq24192_maintenance_worker);
	mutex_init(&chip->event_lock);

	/* Initialize the wakelock */
	wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND,
						"ctp_charger_wakelock");
	chip->chrg_cur_cntl = POWER_SUPPLY_CHARGE_CURRENT_LIMIT_NONE;
	chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	chip->batt_mode = BATT_CHRG_NONE;
	chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chip->curr_volt = BQ24192_INVALID_VOLT;
	chip->curr_chrg = BQ24192_INVALID_CURR;
	chip->cached_chrg_cur_cntl = POWER_SUPPLY_CHARGE_CURRENT_LIMIT_NONE;

	/* register bq24192 usb with power supply subsystem */
	if (!chip->pdata->slave_mode) {
		chip->usb.name = CHARGER_PS_NAME;
		chip->usb.type = POWER_SUPPLY_TYPE_USB;
		chip->usb.supplied_to = bq24192_power_supplied_to;
		chip->usb.num_supplicants =
				ARRAY_SIZE(bq24192_power_supplied_to);
		chip->usb.properties = bq24192_usb_props;
		chip->usb.num_properties = ARRAY_SIZE(bq24192_usb_props);
		chip->usb.get_property = bq24192_usb_get_property;
		chip->usb.set_property = bq24192_usb_set_property;
		chip->usb.property_is_writeable =
					  bq24192_usb_property_is_writeable;
		chip->usb.charging_port_changed = bq24192_charging_port_changed;
		ret = power_supply_register(&client->dev, &chip->usb);
		if (ret) {
			dev_err(&client->dev, "failed:power supply register\n");
			i2c_set_clientdata(client, NULL);
			kfree(chip);
			return ret;
		}
	}

	ctp_sfi_table = kzalloc(sizeof(struct ctp_batt_sfi_prop), GFP_KERNEL);
	if (!ctp_sfi_table) {
		dev_err(&client->dev, "%s(): memory allocation failed\n",
			__func__);
		kfree(chip);
		return -ENOMEM;
	}

	/* check for valid SFI table entry for OEM0 table */
	if (sfi_table_parse(SFI_BATTPROP_TBL_ID, NULL, NULL,
		ctp_sfi_table_populate)) {
		chip->pdata->sfi_tabl_present = false;
		ctp_sfi_table_invalid_batt(ctp_sfi_table);
	}

	/* Allocate ADC Channels */
	chip->gpadc_handle =
		intel_mid_gpadc_alloc(CLT_BATT_NUM_GPADC_SENSORS,
				  CLT_GPADC_BPTHERM_CHNUM | CH_NEED_VCALIB |
				  CH_NEED_VREF);
	if (chip->gpadc_handle == NULL) {
		dev_err(&client->dev,
		 "ADC allocation failed: Check if ADC driver came up\n");
		return -1;
	}

	init_batt_thresholds(chip);

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

	/* create debugfs for maxim registers */
	ret = bq24192_create_debugfs(chip);
	if (ret < 0) {
		dev_err(&client->dev, "debugfs create failed\n");
		power_supply_unregister(&chip->usb);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		intel_mid_gpadc_free(chip->gpadc_handle);
		kfree(ctp_sfi_table);
		return ret;
	}

	/*
	 * Query the OTG driver to check if it has already sent the charger
	 * event. If yes, then we should start the charging. This would ensure
	 * charger driver doesn't miss any usb event.
	 */
	ret = penwell_otg_query_power_supply_cap(&chip->cap);
	if (ret < 0) {
		dev_err(&chip->client->dev,
					"OTG Query failed. OTGD not loaded\n");
	} else {
		dev_info(&chip->client->dev, "Schedule the event worker\n");
		schedule_delayed_work(&chip->chrg_evt_wrkr, 0);
	}

	return 0;
}

static int bq24192_remove(struct i2c_client *client)
{
	struct bq24192_chip *chip = i2c_get_clientdata(client);

	bq24192_remove_debugfs(chip);
	if (!chip->pdata->slave_mode)
		power_supply_unregister(&chip->usb);
	i2c_set_clientdata(client, NULL);
	intel_mid_gpadc_free(chip->gpadc_handle);
	wake_lock_destroy(&chip->wakelock);
	kfree(ctp_sfi_table);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int bq24192_suspend(struct device *dev)
{
	struct bq24192_chip *chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "bq24192 suspend\n");
	return 0;
}

static int bq24192_resume(struct device *dev)
{
	struct bq24192_chip *chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "bq24192 resume\n");
	return 0;
}
#else
#define bq24192_suspend NULL
#define bq24192_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bq24192_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24192_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24192_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bq24192_runtime_suspend	NULL
#define bq24192_runtime_resume		NULL
#define bq24192_runtime_idle		NULL
#endif

static const struct i2c_device_id bq24192_id[] = {
	{ DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bq24192_id);

static const struct dev_pm_ops bq24192_pm_ops = {
	.suspend		= bq24192_suspend,
	.resume			= bq24192_resume,
	.runtime_suspend	= bq24192_runtime_suspend,
	.runtime_resume		= bq24192_runtime_resume,
	.runtime_idle		= bq24192_runtime_idle,
};

static struct i2c_driver bq24192_i2c_driver = {
	.driver	= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &bq24192_pm_ops,
	},
	.probe		= bq24192_probe,
	.remove		= bq24192_remove,
	.id_table	= bq24192_id,
};

static int bq24192_init(void)
{
	return i2c_add_driver(&bq24192_i2c_driver);
}

static void bq24192_exit(void)
{
	i2c_del_driver(&bq24192_i2c_driver);
}

static int bq24192_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed bq24192 rpmsg device\n");

	ret = bq24192_init();

out:
	return ret;
}

static void bq24192_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	bq24192_exit();
	dev_info(&rpdev->dev, "Removed bq24192 rpmsg device\n");
}

static void bq24192_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id bq24192_rpmsg_id_table[] = {
	{ .name	= "rpmsg_bq24192" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, bq24192_rpmsg_id_table);

static struct rpmsg_driver bq24192_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= bq24192_rpmsg_id_table,
	.probe		= bq24192_rpmsg_probe,
	.callback	= bq24192_rpmsg_cb,
	.remove		= bq24192_rpmsg_remove,
};

static int __init bq24192_rpmsg_init(void)
{
	return register_rpmsg_driver(&bq24192_rpmsg);
}

module_init(bq24192_rpmsg_init);

static void __exit bq24192_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&bq24192_rpmsg);
}

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_AUTHOR("Raj Pandey <raj.pandey@intel.com>");
MODULE_DESCRIPTION("BQ24192 Charger Driver");
MODULE_LICENSE("GPL");
