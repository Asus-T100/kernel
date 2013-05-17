/*
 * Intel Baytrail ULPMC battery driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/notifier.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/extcon.h>
#include <linux/power/byt_ulpmc_battery.h>

/* Fuel Gauge Registers */
#define ULPMC_FG_REG_CNTL		0x00
#define ULPMC_FG_REG_TEMP		0x06
#define ULPMC_FG_REG_VOLT		0x08
#define ULPMC_FG_REG_FLAGS		0x0A
#define FG_FLAG_DSC			BIT(0)
#define FG_FLAG_SOCF			BIT(1) /* SOC threshold final */
#define FG_FLAG_SOC1			BIT(2) /* SOC threshold 1 */
#define FG_FLAG_BDET			BIT(3) /* Battery Detect */
#define FG_FLAG_CHG			BIT(8)
#define FG_FLAG_FC			BIT(9)
#define ULPMC_FG_REG_NAC		0x0C /* Nominal available capacity */
#define ULPMC_FG_REG_FAC		0x0E /* Full available capacity */
#define ULPMC_FG_REG_RMC		0x10 /* Remaining capacity */
#define ULPMC_FG_REG_FCC		0x12 /* Full charge capacity */
#define ULPMC_FG_REG_AI			0x14
#define ULPMC_FG_REG_ICUR		0x22
#define ULPMC_FG_REG_SOH		0x28 /* State Of Health */
#define ULPMC_FG_REG_CYCT		0x2A /* Cycle count total */
#define ULPMC_FG_REG_SOC		0x2C /* State Of Charge */
#define ULPMC_FG_REG_SOH_V4		0x1C /* State Of Health */
#define ULPMC_FG_REG_CYCT_V4		0x1E /* Cycle count total */
#define ULPMC_FG_REG_SOC_V4		0x20 /* State Of Charge */

/* capacity interrupt trigger delta in perc (8-bit) */
#define ULPMC_SOC_INT_TRIG_REG		0x4C

/* Charger Registers */
#define ULPMC_BC_REG_STAT			0x2E
/* D6, D7 show VBUS status */
#define BC_STAT_VBUS_UNKNOWN			(0 << 6)
#define BC_STAT_VBUS_HOST			(1 << 6)
#define BC_STAT_VBUS_ADP			(2 << 6)
#define BC_STAT_VBUS_OTG			(3 << 6)
#define BC_STAT_VBUS_MASK			(3 << 6)
/* D4, D5 show charger status */
#define BC_STAT_NOT_CHRG			(0 << 4)
#define BC_STAT_PRE_CHRG			(1 << 4)
#define BC_STAT_FAST_CHRG			(2 << 4)
#define BC_STAT_CHRG_DONE			(3 << 4)
#define BC_STAT_CHRG_MASK			(3 << 4)
#define BC_STAT_DPM				(1 << 3)
#define BC_STAT_PWR_GOOD			(1 << 2)
#define BC_STAT_THERM_REG			(1 << 1)
#define BC_STAT_VSYS_LOW			(1 << 0)
/*
 * Charge Current control register
 * with range from 500 - 2048mA
 */
#define ULPMC_BC_CHRG_CUR_CNTL_REG	0x2F
#define BC_CHRG_CUR_OFFSET		500	/* 500 mA */
#define BC_CHRG_CUR_LSB_TO_CUR		64	/* 64 mA */

#define ULPMC_BC_FAULT_REG			0x4B
#define FAULT_WDT_TMR_EXP			(1 << 7)
#define FAULT_VBUS_OVP				(1 << 6)
/* D4, D5 show charger fault status */
#define FAULT_CHRG_NORMAL			(0 << 4)
#define FAULT_CHRG_IN_FLT			(1 << 4)
#define FAULT_CHRG_THRM_FLT			(2 << 4)
#define FAULT_CHRG_TMR_FLT			(3 << 4)
#define FAULT_CHRG_MASK				(3 << 4)
#define FAULT_BATT_FLT				(1 << 3)
/* D0, D1, D2 show NTC fault info */
#define FAULT_NTC_NORMAL			(0 << 0)
#define FAULT_NTC_TS1_COLD			(1 << 0)
#define FAULT_NTC_TS1_HOT			(2 << 0)
#define FAULT_NTC_TS2_COLD			(3 << 0)
#define FAULT_NTC_TS2_HOT			(4 << 0)
#define FAULT_NTC_BOTH_COLD			(5 << 0)
#define FAULT_NTC_BOTH_HOT			(6 << 0)
#define FAULT_NTC_ONE_COLD_ONE_HOT		(7 << 0)
#define FAULT_NTC_MASK				(7 << 0)

/* ULPMC Battery Manager Registers */
#define ULPMC_BM_REG_LOWBAT_BTP		0x30
#define LOWBATT_THR_SETTING		15	/* 15 perc */
#define CRITBATT_THR_SETTING		5	/* 5 perc */
#define ULPMC_BM_REG_TEMP		0x34

#define ULPMC_BM_REG_RESVBATT_THR	0x52 /* Reserve Battery Threhsold */
#define ULPMC_BM_REG_CNTL		0x53 /* UMPLC command register */
#define CNTL_CHRG_EN			(1 << 0)
#define ULPMC_BM_REG_DIS_VSYS		0x55
#define BM_DISABLE_VSYS			0x1
#define BM_RESET_VSYS			0x2

#define ULPMC_BM_REG_BATT_PRESENT	0x4E
#define BATT_PRESENT_DET_FAIL		(0 << 0)
#define BATT_PRESENT_DET_1P		(1 << 0)
#define BATT_PRESENT_DET_2P		(2 << 0)
#define BATT_PRESENT_DET_MASK		(3 << 0)

#define ULPMC_BM_REG_INTSTAT		0x4F
#define INTSTAT_INSERT_AC		0x1
#define INTSTAT_REMOVE_AC		0x2
#define INTSTAT_INSERT_USB		0x3
#define INTSTAT_REMOVE_USB		0x4
#define INTSTAT_START_CHARGE		0x5
#define INTSTAT_STOP_CHARGE		0x6
#define INTSTAT_INSERT_BATTERY		0x7
#define INTSTAT_REMOVE_BATTERY		0x8
#define INTSTAT_LOW_BATTERY		0x9
#define INTSTAT_BTP_HIGH		0xA
#define INTSTAT_BTP_LOW			0xB
#define INTSTAT_THRM_BAT0		0xC
#define INTSTAT_THRM_BAT1		0xD
#define INTSTAT_THRM_SKIN0		0xE
#define INTSTAT_STOP_CHARGING		0xF
#define INTSTAT_PERC_HIGH		0x10
#define INTSTAT_PERC_LOW		0x11
#define INTSTAT_FAULT_TRIGGER		0x13
#define INTSTAT_BATT_WARN		0x14
#define INTSTAT_BATT_LOW		0x15
#define INTSTAT_BATT_CRIT		0x16

#define ULPMC_INTR_QSIZE		32

#define ULPMC_FG_SIGN_INDICATOR		0x8000

#define CHARGE_CURRENT_CNTL_LIM0	0x0
#define CHARGE_CURRENT_CNTL_LIM1	0x1
#define CHARGE_CURRENT_CNTL_LIM2	0x2
#define CHARGE_CURRENT_CNTL_LIM3	0x3
#define CHARGE_CURRENT_CNTL_LIM_MAX	0x4

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT	3

struct ulpmc_chip_info {
	struct i2c_client	*client;
	struct ulpmc_platform_data *pdata;

	struct power_supply	bat;
	struct power_supply	chrg;

	struct extcon_dev	*edev;
	struct notifier_block	nb;

	struct mutex lock;
	bool is_fwupdate_on;
	int cur_throttle_state;
	bool block_sdp;
};

static struct ulpmc_chip_info *chip_ptr;

static enum power_supply_property ulpmc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property ulpmc_charger_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static int ulpmc_write_reg16(struct i2c_client *client, u8 reg, u16 value)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_word_data(client, reg, value);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Write error:%d\n", ret);

	return ret;
}

static int ulpmc_read_reg16(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_word_data(client, reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Read error:%d\n", ret);

	return ret;
}

static int ulpmc_write_reg8(struct i2c_client *client, u8 reg, u8 value)
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

static int ulpmc_read_reg8(struct i2c_client *client, u8 reg)
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

static short adjust_sign_value(int value)
{
	short result, temp = (short)value;
	if (temp & ULPMC_FG_SIGN_INDICATOR) {
		result = ~temp;
		result++;
		result *= -1;
	} else {
		result = temp;
	}

	return result;
}

static int bc_reg_to_cur(u8 reg)
{
	/*
	 * D0, D1 bits of charge current
	 * register are not used.
	 */
	reg = reg >> 2;
	reg = (reg * BC_CHRG_CUR_LSB_TO_CUR) + BC_CHRG_CUR_OFFSET;

	return reg;
}

static u8 cur_to_cc_reg(int cur)
{
	u8 reg = 0;

	if (cur > BC_CHRG_CUR_OFFSET)
		reg = ((cur - BC_CHRG_CUR_OFFSET) /
				BC_CHRG_CUR_LSB_TO_CUR) + 1;

	/*
	 * D0, D1 bits of charge current
	 * register are not used.
	 */
	return reg << 2;
}

static int ulpmc_throttle_chrg_cur(struct ulpmc_chip_info *chip, int lim)
{
	int ret, cur;
	u8 chrg_en;

	switch (lim) {
	case CHARGE_CURRENT_CNTL_LIM0:
		/* re-program default setting */
		cur = chip->pdata->cc_lim0;
		break;
	case CHARGE_CURRENT_CNTL_LIM1:
		/* limit the charge current */
		cur = chip->pdata->cc_lim1;
		break;
	case CHARGE_CURRENT_CNTL_LIM2:
		/* limit the charge current */
		cur = chip->pdata->cc_lim2;
		break;
	case CHARGE_CURRENT_CNTL_LIM3:
		/* limit the charge current */
		cur = chip->pdata->cc_lim3;
		break;
	case CHARGE_CURRENT_CNTL_LIM_MAX:
		/* disable charging */
		cur = 0;
		break;
	default:
		dev_err(&chip->client->dev, "unknown limit:%d\n", lim);
		return -EINVAL;
	}

	ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_CNTL);
	if (ret < 0)
		goto cc_throttle_fail;
	else
		chrg_en = ret;

	if (cur) {
		ret = ulpmc_write_reg8(chip->client,
			ULPMC_BC_CHRG_CUR_CNTL_REG, cur_to_cc_reg(cur));
		if (ret < 0)
			goto cc_throttle_fail;

		if (!(chrg_en & CNTL_CHRG_EN)) {
			ret = ulpmc_write_reg8(chip->client, ULPMC_BM_REG_CNTL,
						(chrg_en | CNTL_CHRG_EN));
		}
	} else {
		ret = ulpmc_write_reg8(chip->client, ULPMC_BM_REG_CNTL,
						(chrg_en & ~CNTL_CHRG_EN));
	}

cc_throttle_fail:
	if (ret < 0)
		dev_err(&chip->client->dev, "i2c write error:%d\n", ret);
	return ret;
}

static int ulpmc_charger_health(struct ulpmc_chip_info *chip)
{
	int stat, fault, health;

	stat = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (stat < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", stat);
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto report_chrg_health;
	}

	fault = ulpmc_read_reg8(chip->client, ULPMC_BC_FAULT_REG);
	if (fault < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", fault);
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto report_chrg_health;
	}

	if (!(stat & BC_STAT_VBUS_MASK)) {
		/* no charger present */
		health = POWER_SUPPLY_HEALTH_UNKNOWN;
		goto report_chrg_health;
	}

	if (fault & FAULT_VBUS_OVP) {
		dev_info(&chip->client->dev, "charger over voltage fault\n");
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto report_chrg_health;
	}

	if ((fault & FAULT_CHRG_MASK) == FAULT_CHRG_THRM_FLT) {
		dev_info(&chip->client->dev, "charger over temp fault\n");
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto report_chrg_health;
	}

	if (((fault & FAULT_CHRG_MASK) == FAULT_CHRG_TMR_FLT) ||
			(fault & FAULT_WDT_TMR_EXP)) {
		dev_info(&chip->client->dev, "charger timer fault\n");
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto report_chrg_health;
	}

	if (!(stat & BC_STAT_PWR_GOOD)) {
		dev_info(&chip->client->dev, "charger low vbus fault\n");
		health =  POWER_SUPPLY_HEALTH_DEAD;
		goto report_chrg_health;
	}

	health = POWER_SUPPLY_HEALTH_GOOD;

report_chrg_health:
	return health;
}

static int ulpmc_battery_health(struct ulpmc_chip_info *chip)
{
	int stat, fault, health;

	stat = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (stat < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", stat);
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto report_batt_health;
	}

	fault = ulpmc_read_reg8(chip->client, ULPMC_BC_FAULT_REG);
	if (fault < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", fault);
		health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto report_batt_health;
	}

	if (fault & FAULT_BATT_FLT) {
		dev_info(&chip->client->dev, "battery over voltage fault\n");
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto report_batt_health;
	}

	if (fault & FAULT_NTC_MASK) {
		dev_info(&chip->client->dev, "battery over temp fault\n");
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto report_batt_health;
	}

	/*
	 * TODO: check battery temp  high
	 * and low thresholds and set health.
	 * this will be done on PR1.1
	 */

	if (stat & BC_STAT_VSYS_LOW) {
		dev_info(&chip->client->dev, "battery low fault\n");
		health = POWER_SUPPLY_HEALTH_DEAD;
		goto report_batt_health;
	}

	health = POWER_SUPPLY_HEALTH_GOOD;

report_batt_health:
	return health;
}

static int ulpmc_battery_status(struct ulpmc_chip_info *chip)
{
	int stat, batt_health, chrg_health, ret;
	bool fault;

	ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (ret < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", ret);
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		goto batt_stat_report;
	} else {
		stat = ret;
	}

	if ((stat & BC_STAT_VBUS_MASK) != BC_STAT_VBUS_ADP) {
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		goto batt_stat_report;
	}

	/* check for charger or battery fault */
	batt_health = ulpmc_battery_health(chip);
	chrg_health = ulpmc_charger_health(chip);

	if ((batt_health != POWER_SUPPLY_HEALTH_GOOD) &&
		(batt_health != POWER_SUPPLY_HEALTH_DEAD))
		fault = true;
	else if (chrg_health != POWER_SUPPLY_HEALTH_GOOD)
		fault = true;
	else
		fault = false;

	switch (stat & BC_STAT_CHRG_MASK) {
	case BC_STAT_NOT_CHRG:
		if (fault)
			ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			ret = POWER_SUPPLY_STATUS_FULL;
		break;
	case BC_STAT_PRE_CHRG:
	case BC_STAT_FAST_CHRG:
		ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case BC_STAT_CHRG_DONE:
		ret = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		dev_err(&chip->client->dev, "unkown battery status\n");
	}

batt_stat_report:
	return ret;
}

static int ulpmc_get_capacity(struct ulpmc_chip_info *chip)
{
	int ret;

	if (chip->pdata->version == BYTULPMCFGV3)
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_SOC);
	else
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_SOC_V4);

	return ret;
}

static int ulpmc_get_battery_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct ulpmc_chip_info *chip = container_of(psy,
				struct ulpmc_chip_info, bat);
	mutex_lock(&chip->lock);

	if (chip->is_fwupdate_on) {
		dev_warn(&chip->client->dev, "fwupdate in progress\n");
		mutex_unlock(&chip->lock);
		return -EAGAIN;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->block_sdp)
			val->intval = ulpmc_battery_status(chip);
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = ulpmc_battery_health(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_VOLT);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->pdata->volt_sh_min * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_ICUR);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ((int)adjust_sign_value(ret)) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_AI);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ((int)adjust_sign_value(ret)) * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_FLAGS);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = (ret & FG_FLAG_BDET) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = ulpmc_get_capacity(chip);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_TEMP);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret * 10;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_BATT_PRESENT);
		if (ret < 0)
			goto i2c_read_err;
		if ((ret & BATT_PRESENT_DET_MASK) == BATT_PRESENT_DET_2P)
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		else
			val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_RMC);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_FCC);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_FAC);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (chip->pdata->version == BYTULPMCFGV3)
			ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_CYCT);
		else
			ret = ulpmc_read_reg16(chip->client,
							ULPMC_FG_REG_CYCT_V4);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->pdata->battid;
		break;
	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->lock);
	return 0;

i2c_read_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ulpmc_get_charger_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0, stat;
	bool chg_present = false;
	struct ulpmc_chip_info *chip = container_of(psy,
				struct ulpmc_chip_info, chrg);

	mutex_lock(&chip->lock);

	if (chip->is_fwupdate_on) {
		dev_warn(&chip->client->dev, "fwupdate in progress\n");
		mutex_unlock(&chip->lock);
		return -EAGAIN;
	}

	stat = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (stat < 0) {
		ret = stat;
		goto i2c_read_err;
	}

	if (((stat & BC_STAT_VBUS_MASK)
		== BC_STAT_VBUS_ADP) && !chip->block_sdp)
		chg_present = true;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (chg_present)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg_present && (stat & BC_STAT_PWR_GOOD))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chg_present)
			val->intval = ulpmc_charger_health(chip);
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		/* return error if charger is not present */
		if (!chg_present) {
			ret = -EINVAL;
			goto i2c_read_err;
		}
		ret = ulpmc_read_reg8(chip->client, ULPMC_BC_CHRG_CUR_CNTL_REG);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = bc_reg_to_cur(ret) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = chip->cur_throttle_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = CHARGE_CURRENT_CNTL_LIM_MAX;
		break;
	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->lock);
	return 0;

i2c_read_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ulpmc_set_charger_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct ulpmc_chip_info *chip = container_of(psy,
				struct ulpmc_chip_info, chrg);
	int ret;

	mutex_lock(&chip->lock);

	if (chip->is_fwupdate_on) {
		dev_warn(&chip->client->dev, "fwupdate in progress\n");
		ret = -EAGAIN;
		goto set_prop_err;
	}

	ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (ret < 0) {
		dev_err(&chip->client->dev, "i2c read error:%d\n", ret);
		goto set_prop_err;
	}

	/* return error if charger is not present */
	if (!((ret & BC_STAT_VBUS_MASK) == BC_STAT_VBUS_ADP)) {
		ret = -EINVAL;
		goto set_prop_err;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if ((val->intval < 0) ||
			(val->intval > CHARGE_CURRENT_CNTL_LIM_MAX)) {
			ret = -ERANGE;
			goto set_prop_err;
		} else {
			ret = ulpmc_throttle_chrg_cur(chip, val->intval);
			if (ret < 0)
				goto set_prop_err;
			chip->cur_throttle_state = val->intval;
		}
		break;
	default:
		ret = -EINVAL;
	}

	/* return 0 on success */
	ret = 0;

set_prop_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void dump_registers(struct ulpmc_chip_info *chip)
{
	int ret;

	ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (ret < 0)
		dev_err(&chip->client->dev, "i2c read error:%d\n", ret);
	else
		dev_info(&chip->client->dev, "stat reg:%x\n", ret);

	ret = ulpmc_read_reg8(chip->client, ULPMC_BC_FAULT_REG);
	if (ret < 0)
		dev_err(&chip->client->dev, "i2c read error:%d\n", ret);
	else
		dev_info(&chip->client->dev, "fault reg:%x\n", ret);

	ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_FLAGS);
	if (ret < 0)
		dev_err(&chip->client->dev, "i2c read error:%d\n", ret);
	else
		dev_info(&chip->client->dev, "fg flags reg:%x\n", ret);
}

static void log_interrupt_event(struct ulpmc_chip_info *chip, int intstat)
{
	bool conn_evt = false;

	switch (intstat) {
	case INTSTAT_INSERT_AC:
		dev_info(&chip->client->dev, "AC charger inserted\n");
		conn_evt = true;
		break;
	case INTSTAT_REMOVE_AC:
		dev_info(&chip->client->dev, "AC charger removeed\n");
		break;
	case INTSTAT_INSERT_USB:
		dev_info(&chip->client->dev, "USB charger inserted\n");
		conn_evt = true;
		break;
	case INTSTAT_REMOVE_USB:
		dev_info(&chip->client->dev, "USB charger removed\n");
		break;
	case INTSTAT_START_CHARGE:
		dev_info(&chip->client->dev, "Charging start event\n");
		break;
	case INTSTAT_STOP_CHARGE:
		dev_info(&chip->client->dev, "Charging stop event\n");
		break;
	case INTSTAT_INSERT_BATTERY:
		dev_info(&chip->client->dev, "Battery inserted\n");
		break;
	case INTSTAT_REMOVE_BATTERY:
		dev_info(&chip->client->dev, "Battery removed\n");
		break;
	case INTSTAT_LOW_BATTERY:
		dev_info(&chip->client->dev, "BATTLOW event!!\n");
		break;
	case INTSTAT_BTP_HIGH:
		dev_info(&chip->client->dev, "Battery Trip point high\n");
		break;
	case INTSTAT_BTP_LOW:
		dev_info(&chip->client->dev, "Battery Trip point low\n");
		break;
	case INTSTAT_THRM_BAT0:
		dev_info(&chip->client->dev, "Battery0 thermal event\n");
		break;
	case INTSTAT_THRM_BAT1:
		dev_info(&chip->client->dev, "Battery1 thermal event\n");
		break;
	case INTSTAT_THRM_SKIN0:
		dev_info(&chip->client->dev, "Skin0 thermal event\n");
		break;
	case INTSTAT_STOP_CHARGING:
		dev_info(&chip->client->dev, "Stop Charging event\n");
		break;
	case INTSTAT_PERC_HIGH:
		dev_info(&chip->client->dev, "Battery perc change high event\n");
		break;
	case INTSTAT_PERC_LOW:
		dev_info(&chip->client->dev, "Battery perc change low event\n");
		break;
	case INTSTAT_FAULT_TRIGGER:
		dev_info(&chip->client->dev, "fault event recieved\n");
		break;
	case INTSTAT_BATT_WARN:
		dev_info(&chip->client->dev,
			"Warn Battery level threshold reached\n");
		break;
	case INTSTAT_BATT_LOW:
		dev_info(&chip->client->dev,
			"Low Battery level threshold reached\n");
		break;
	case INTSTAT_BATT_CRIT:
		dev_info(&chip->client->dev,
			"Critical Battery level threshold reached\n");
		break;
	default:
		dev_info(&chip->client->dev, "spurious event!!!\n");
	}

	if (conn_evt) {
		mutex_lock(&chip->lock);
		ulpmc_throttle_chrg_cur(chip, chip->cur_throttle_state);
		mutex_unlock(&chip->lock);
	}
}

static irqreturn_t ulpmc_thread_handler(int id, void *dev)
{
	struct ulpmc_chip_info *chip = dev;
	int ret, i;

	pm_runtime_get_sync(&chip->client->dev);

	for (i = 0; i < ULPMC_INTR_QSIZE; i++) {
		ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_INTSTAT);
		if (ret < 0) {
			dev_err(&chip->client->dev, "ulpmc stat reg read error\n");
			pm_runtime_put_sync(&chip->client->dev);
			return IRQ_NONE;
		}
		if (!ret)
			break;
		log_interrupt_event(chip, ret);
	}

	dump_registers(chip);
	power_supply_changed(&chip->bat);

	pm_runtime_put_sync(&chip->client->dev);
	return IRQ_HANDLED;
}

static void ulpmc_init_irq(struct ulpmc_chip_info *chip)
{
	int ret = 0;
	int gpio_num;

	/* get kernel GPIO number */
	gpio_num = acpi_get_gpio("\\_SB.GPO2", chip->pdata->gpio);
	/* get irq number */
	chip->client->irq = gpio_to_irq(gpio_num);
	/* register interrupt */
	ret = request_threaded_irq(chip->client->irq, NULL,
					ulpmc_thread_handler,
					IRQF_TRIGGER_FALLING,
					"ulpmc-battery", chip);
	if (ret) {
		dev_warn(&chip->client->dev,
			"cannot get IRQ:%d\n", chip->client->irq);
		chip->client->irq = -1;
	} else {
		dev_info(&chip->client->dev, "IRQ No:%d\n", chip->client->irq);
	}
}

int byt_ulpmc_cutoff_vsys(struct ulpmc_chip_info *chip)
{
	int ret;

	ret = ulpmc_write_reg8(chip->client,
			ULPMC_BM_REG_DIS_VSYS, BM_DISABLE_VSYS);
	return ret;
}

int byt_ulpmc_reset_vsys(struct ulpmc_chip_info *chip)
{
	int ret;

	ret = ulpmc_write_reg8(chip->client,
			ULPMC_BM_REG_DIS_VSYS, BM_RESET_VSYS);
	return ret;
}

int byt_ulpmc_suspend_sdp_charging(void)
{
	int ret = 0;

	if (chip_ptr) {
		ret = byt_ulpmc_cutoff_vsys(chip_ptr);
		mutex_lock(&chip_ptr->lock);
		chip_ptr->block_sdp = true;
		mutex_unlock(&chip_ptr->lock);
	}

	return ret;
}
EXPORT_SYMBOL(byt_ulpmc_suspend_sdp_charging);

int byt_ulpmc_reset_charger(void)
{
	int ret = 0;

	if (chip_ptr) {
		ret = byt_ulpmc_reset_vsys(chip_ptr);
		/*
		 * reset or clear the chip data
		 * which was set by functions like
		 * byt_ulpmc_suspend_sdp_charging().
		 */
		mutex_lock(&chip_ptr->lock);
		chip_ptr->block_sdp = false;
		mutex_unlock(&chip_ptr->lock);
	}

	return ret;
}
EXPORT_SYMBOL(byt_ulpmc_reset_charger);

static void handle_extcon_events(struct ulpmc_chip_info *chip)
{
	int chrg_type, idx, ret;
	u32 event;

	/* current extcon cable status */
	event = chip->edev->state;
	dev_info(&chip->client->dev, "extcon cur state:%d\n", event);

	if (!event) {
		chrg_type = POWER_SUPPLY_TYPE_USB;
		goto extcon_evt_ret;
	}
	/* get the edev index from extcon event */
	for (idx = 0; idx < chip->edev->max_supported; idx++) {
		if ((event >> idx) & 0x1)
			break;
	}
	/* get extcon cable type */
	ret = extcon_find_cable_type(chip->edev, idx);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"extcon_find_cable_type failed\n");
		chrg_type = POWER_SUPPLY_TYPE_USB;
		goto extcon_evt_ret;
	}

	switch (ret) {
	case EXTCON_SDP:
		chrg_type = POWER_SUPPLY_TYPE_USB;
		break;
	case EXTCON_CDP:
		chrg_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case EXTCON_DCP:
		chrg_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case EXTCON_ACA:
		chrg_type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	default:
		chrg_type = POWER_SUPPLY_TYPE_USB;
		dev_warn(&chip->client->dev, "unknown extcon cable\n");
	}

extcon_evt_ret:
	chip->chrg.type = chrg_type;
}

static int ulpmc_extcon_callback(struct notifier_block *nb,
					unsigned long old_state, void *data)
{
	struct ulpmc_chip_info *chip = container_of(nb,
					struct ulpmc_chip_info, nb);

	handle_extcon_events(chip);
	power_supply_changed(&chip->bat);
	return NOTIFY_OK;
}

static void set_s0ix_soc_thresholds(struct ulpmc_chip_info *chip)
{
	int ret;

	/* set 1% soc trigger threshold */
	ret = ulpmc_write_reg8(chip->client, ULPMC_SOC_INT_TRIG_REG, 1);
	if (ret < 0)
		goto s0ix_thr_err;

	/* set 0% as lowbatt threshold */
	ret = ulpmc_write_reg16(chip->client, ULPMC_BM_REG_LOWBAT_BTP, 0);
	if (ret < 0)
		goto s0ix_thr_err;

	return ;

s0ix_thr_err:
	dev_err(&chip->client->dev, "i2c write error:%d\n", ret);
}

static void set_s3_soc_thresholds(struct ulpmc_chip_info *chip)
{
	int ret;
	u16 low_thr;

	ret = ulpmc_get_capacity(chip);
	if (ret < 0)
		goto s3_thr_err;

	if (ret > LOWBATT_THR_SETTING)
		low_thr = LOWBATT_THR_SETTING;
	else if (ret > CRITBATT_THR_SETTING)
		low_thr = CRITBATT_THR_SETTING;
	else
		low_thr = 0;

	/* set lowbatt threshold */
	ret = ulpmc_write_reg16(chip->client, ULPMC_BM_REG_LOWBAT_BTP, low_thr);
	if (ret < 0)
		goto s3_thr_err;

	/* disable 1% soc change interrupts */
	ret = ulpmc_write_reg8(chip->client, ULPMC_SOC_INT_TRIG_REG, 0);
	if (ret < 0)
		goto s3_thr_err;

	return ;

s3_thr_err:
	dev_err(&chip->client->dev, "i2c write error:%d\n", ret);
}

void ulpmc_fwupdate_enter(void)
{
	if (chip_ptr) {
		pm_runtime_get_sync(&chip_ptr->client->dev);
		dev_info(&chip_ptr->client->dev, ":%s\n", __func__);
		mutex_lock(&chip_ptr->lock);
		chip_ptr->is_fwupdate_on = true;
		mutex_unlock(&chip_ptr->lock);
	}
}
EXPORT_SYMBOL(ulpmc_fwupdate_enter);

void ulpmc_fwupdate_exit(void)
{
	if (chip_ptr) {
		dev_info(&chip_ptr->client->dev, ":%s\n", __func__);
		mutex_lock(&chip_ptr->lock);
		chip_ptr->is_fwupdate_on = false;
		mutex_unlock(&chip_ptr->lock);
		/* init soc interrupt thresholds */
		set_s0ix_soc_thresholds(chip_ptr);
		pm_runtime_put_sync(&chip_ptr->client->dev);
	}
}
EXPORT_SYMBOL(ulpmc_fwupdate_exit);

struct i2c_client *ulpmc_get_i2c_client(void)
{
	dev_info(&chip_ptr->client->dev, ":%s\n", __func__);

	if (chip_ptr)
		return chip_ptr->client;
	else
		return NULL;
}
EXPORT_SYMBOL(ulpmc_get_i2c_client);

static void check_battery_presence(struct ulpmc_chip_info *chip)
{
	int ret;

	ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_BATT_PRESENT);
	if (ret < 0) {
		dev_info(&chip->client->dev, "i2c read error:%d\n", ret);
		return ;
	}

	if ((ret & BATT_PRESENT_DET_MASK) == BATT_PRESENT_DET_FAIL)
		dev_warn(&chip->client->dev,
				"battery invalid or not present\n");
	else if ((ret & BATT_PRESENT_DET_MASK) == BATT_PRESENT_DET_1P)
		dev_warn(&chip->client->dev, "single battery pack detected\n");
	else if ((ret & BATT_PRESENT_DET_MASK) == BATT_PRESENT_DET_2P)
		dev_warn(&chip->client->dev, "two battery packs detected\n");
	else
		dev_warn(&chip->client->dev, "batt id detection failed\n");
}

static void ulpmc_clear_pending_intr(struct ulpmc_chip_info *chip)
{
	int i, ret;

	for (i = 0; i < ULPMC_INTR_QSIZE; i++) {
		ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_INTSTAT);
		if (ret < 0)
			dev_err(&chip->client->dev,
				"ulpmc stat reg read error:%d\n", ret);
		if (!ret)
			break;
	}
}

static int ulpmc_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct ulpmc_chip_info *chip;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"SM bus doesn't support BYTE transactions\n");
		return -EIO;
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
				"SM bus doesn't support WORD transactions\n");
		return -EIO;
	}

	/* check if the device is accessible */
	ret = ulpmc_read_reg16(client, ULPMC_FG_REG_CNTL);
	if (ret < 0) {
		dev_err(&client->dev,
			"I2C read error:%s error:%d\n", __func__, ret);
		return -EIO;
	} else {
		dev_err(&client->dev, "FG control reg:%x\n", ret);
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, chip);

	mutex_init(&chip->lock);
	chip_ptr = chip;

	chip->bat.name = "byt-battery";
	chip->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->bat.properties = ulpmc_battery_props;
	chip->bat.num_properties = ARRAY_SIZE(ulpmc_battery_props);
	chip->bat.get_property = ulpmc_get_battery_property;
	ret = power_supply_register(&client->dev, &chip->bat);
	if (ret) {
		dev_err(&client->dev, "failed to register battery: %d\n", ret);
		goto probe_failed_1;
	}

	chip->chrg.name = "byt-charger";
	chip->chrg.type = POWER_SUPPLY_TYPE_USB;
	chip->chrg.properties = ulpmc_charger_properties;
	chip->chrg.num_properties = ARRAY_SIZE(ulpmc_charger_properties);
	chip->chrg.get_property = ulpmc_get_charger_property;
	chip->chrg.set_property = ulpmc_set_charger_property;
	ret = power_supply_register(&client->dev, &chip->chrg);
	if (ret) {
		dev_err(&client->dev, "failed to register charger: %d\n", ret);
		goto probe_failed_2;
	}

	/* get extcon device */
	chip->edev = extcon_get_extcon_dev(chip->pdata->extcon_devname);
	if (!chip->edev) {
		dev_err(&client->dev, "failed to get extcon device\n");
	} else {
		chip->nb.notifier_call = &ulpmc_extcon_callback;
		ret = extcon_register_notifier(chip->edev, &chip->nb);
		if (ret)
			dev_err(&client->dev,
				"failed to register extcon notifier:%d\n", ret);
	}

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

	/* check battery presence */
	check_battery_presence(chip);
	/* clear pending interrupts */
	ulpmc_clear_pending_intr(chip);
	/* init soc interrupt thresholds */
	set_s0ix_soc_thresholds(chip);
	/* get irq and register */
	ulpmc_init_irq(chip);
	/* check for charger presence */
	if (chip->edev)
		handle_extcon_events(chip);
	return 0;

probe_failed_2:
	power_supply_unregister(&chip->bat);
probe_failed_1:
	kfree(chip);
	return ret;
}

static int ulpmc_battery_remove(struct i2c_client *client)
{
	struct ulpmc_chip_info *chip = i2c_get_clientdata(client);

	if (chip->edev)
		extcon_unregister_notifier(chip->edev, &chip->nb);
	power_supply_unregister(&chip->chrg);
	power_supply_unregister(&chip->bat);
	pm_runtime_get_noresume(&chip->client->dev);
	mutex_destroy(&chip->lock);
	kfree(chip);

	return 0;
}

static int ulpmc_suspend(struct device *dev)
{
	struct ulpmc_chip_info *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		disable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
	}
	set_s3_soc_thresholds(chip);
	dev_dbg(&chip->client->dev, "ulpmc battery suspend\n");

	return 0;
}

static int ulpmc_resume(struct device *dev)
{
	struct ulpmc_chip_info *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0) {
		enable_irq(chip->client->irq);
		disable_irq_wake(chip->client->irq);
	}

	set_s0ix_soc_thresholds(chip);
	dev_dbg(&chip->client->dev, "ulpmc battery resume\n");

	return 0;
}

static int ulpmc_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int ulpmc_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int ulpmc_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops ulpmc_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(ulpmc_suspend,
				ulpmc_resume)
		SET_RUNTIME_PM_OPS(ulpmc_runtime_suspend,
				ulpmc_runtime_resume,
				ulpmc_runtime_idle)
};

static const struct i2c_device_id ulpmc_id[] = {
	{ "ulpmc", 0 },
};
MODULE_DEVICE_TABLE(i2c, ulpmc_id);

static struct i2c_driver ulpmc_battery_driver = {
	.driver = {
		.name = "ulpmc-battery",
		.owner	= THIS_MODULE,
		.pm	= &ulpmc_pm_ops,
	},
	.probe = ulpmc_battery_probe,
	.remove = ulpmc_battery_remove,
	.id_table = ulpmc_id,
};

/*
 * Module stuff
 */

static int __init ulpmc_battery_init(void)
{
	int ret = i2c_add_driver(&ulpmc_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register ULPMC i2c driver\n");

	return ret;
}
late_initcall(ulpmc_battery_init);

static void __exit ulpmc_battery_exit(void)
{
	i2c_del_driver(&ulpmc_battery_driver);
}
module_exit(ulpmc_battery_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("BYT ULPMC battery driver");
MODULE_LICENSE("GPL");
