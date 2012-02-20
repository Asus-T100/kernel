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
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb/penwell_otg.h>

#define DRV_NAME "bq24192_charger"
#define DEV_NAME "bq24192"

/*
 * D0, D1, D2 can be used to set current limits
 * and D3, D4, D5, D6 can be used to voltage limits
 */
#define BQ24192_INPUT_SRC_CNTL_REG		0x0
#define INPUT_SRC_CNTL_EN_HIZ			(1 << 7)
/* set input voltage lim to 5V */
#define INPUT_SRC_VOLT_LMT			(6 << 3)
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
#define POWER_ON_CFG_CHRG_CFG_OTG		(2 << 4)
#define POWER_ON_CFG_BOOST_LIM			(1 << 0)

/*
 * Charge Current control register
 * with range from 500 - 4532mA
 */
#define BQ24192_CHRG_CUR_CNTL_REG		0x2
#define BQ24192_CHRG_CUR_OFFSET		500	/* 500 mA */
#define BQ24192_CHRG_CUR_LSB_TO_CUR	64	/* 64 mA */

/* Pre charge and termination current limit reg */
#define BQ24192_PRECHRG_TERM_CUR_CNTL_REG	0x3

/* Charge voltage control reg */
#define BQ24192_CHRG_VOLT_CNTL_REG	0x4
#define BQ24192_CHRG_VOLT_OFFSET	3504	/* 3504 mV */
#define BQ24192_CHRG_VOLT_LSB_TO_VOLT	16	/* 16 mV */
/* Low voltage setting 0 - 2.8V and 1 - 3.0V */
#define CHRG_VOLT_CNTL_BATTLOWV		(1 << 1)
/* Battery Recharge threshold 0 - 100mV and 1 - 300mV */
#define CHRG_VOLT_CNTL_VRECHRG		(1 << 0)

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
#define SYSTEM_STAT_VBUS_UNKNOWN		(0 << 6)
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

#define BQ24192_DEF_VBATT_MAX		4200	/* 4200mV */
#define BQ24192_DEF_SDP_ILIM_CUR	500	/* 500mA */
#define BQ24192_DEF_DCP_ILIM_CUR	1500	/* 1500mA */
#define BQ24192_DEF_CHRG_CUR		1500	/* 1500mA */

#define BQ24192_CHRG_CUR_LOW		100	/* 100mA */
#define BQ24192_CHRG_CUR_MEDIUM		500	/* 500mA */
#define BQ24192_CHRG_CUR_HIGH		900	/* 900mA */
#define BQ24192_CHRG_CUR_NOLIMIT	1500	/* 1500mA */

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
	struct mutex event_lock;

	int present;
	int online;
	enum power_supply_type chrg_type;
	int chrg_cur_cntl;

	/* battery info */
	int batt_status;
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *bq24192_dbgfs_root;
static char bq24192_dbg_regs[BQ24192_MAX_MEM][4];
#endif

static void *otg_handle;
static struct i2c_client *bq24192_client;

static char *bq24192_power_supplied_to[] = {
			"max17042_battery",
};

static enum power_supply_property bq24192_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_CHARGE_CURRENT_LIMIT,
};

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

/* map charge current control setting
 * to input current limit value in mA.
 */
static int chrg_cntl_to_cur_lim(int lim)
{
	int cur_lim;

	switch (lim) {
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT_LOW:
		cur_lim = BQ24192_CHRG_CUR_LOW;
		break;
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT_MEDIUM:
		cur_lim = BQ24192_CHRG_CUR_MEDIUM;
		break;
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT_HIGH:
		cur_lim = BQ24192_CHRG_CUR_HIGH;
		break;
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT_NONE:
		cur_lim = BQ24192_CHRG_CUR_NOLIMIT;
		break;
	default:
		cur_lim = -EINVAL;
	}

	return cur_lim;
}

/* convert the input current limit value
 * into equivalent register setting.
 * Note: ilim must be in mA.
 */
static u8 chrg_ilim_to_reg(int ilim)
{
	u8 reg;

	/* set voltage to 5V */
	reg = INPUT_SRC_VOLT_LMT;

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
				BQ24192_CHRG_CUR_LSB_TO_CUR);

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

static int program_wdt_timer(struct bq24192_chip *chip, u8 val)
{
	int ret;

	/* program WDT timer value */
	ret = bq24192_reg_multi_bitset(chip->client,
					BQ24192_CHRG_TIMER_EXP_CNTL_REG,
					val,
					WDT_TIMER_BIT_POS, WDT_TIMER_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	return ret;
}

static int reset_wdt_timer(struct bq24192_chip *chip)
{
	int ret;

	/* reset WDT timer */
	ret = bq24192_reg_read_modify(chip->client, BQ24192_POWER_ON_CFG_REG,
						BQ24192_POWER_ON_CFG_REG, true);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

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

	/* disable WDT timer */
	ret = program_wdt_timer(chip, CHRG_TIMER_EXP_CNTL_WDTDISABLE);
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

i2c_write_failed:
	return ret;

}

static int stop_charging(struct bq24192_chip *chip)
{
	int ret;

	/* Disable the charger */
	ret = bq24192_reg_multi_bitset(chip->client, BQ24192_POWER_ON_CFG_REG,
						POWER_ON_CFG_CHRG_CFG_DIS,
					CHR_CFG_BIT_POS, CHR_CFG_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	return ret;
}

static int update_ilim_settings(struct bq24192_chip *chip, int chrg_lim)
{
	int ret;
	u8 in_src;

	if (chrg_lim == POWER_SUPPLY_CHARGE_CURRENT_LIMIT_ZERO) {
		ret = stop_charging(chip);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"charge disabling failed\n");
			return ret;
		}
	}

	ret = chrg_cntl_to_cur_lim(chrg_lim);
	if (ret < 0)
		return ret;

	in_src = chrg_ilim_to_reg(ret);
	/* set input voltage and current reg */
	ret = bq24192_write_reg(chip->client,
			BQ24192_INPUT_SRC_CNTL_REG, in_src);
	if (ret < 0)
		dev_warn(&chip->client->dev,
				"I2C write failed:%s\n", __func__);
	return ret;
}

static void set_up_charging(struct bq24192_chip *chip,
				struct bq24192_chrg_regs *reg)
{
	reg->in_src = chrg_ilim_to_reg(chip->cap.mA);
	reg->chr_cur = chrg_cur_to_reg(BQ24192_DEF_CHRG_CUR);
	reg->chr_volt = chrg_volt_to_reg(BQ24192_DEF_VBATT_MAX);
}

static void bq24192_event_worker(struct work_struct *work)
{
	struct bq24192_chip *chip = container_of(work,
				struct bq24192_chip, chrg_evt_wrkr.work);
	struct bq24192_chrg_regs reg;
	int ret;

	dev_info(&chip->client->dev, "%s\n", __func__);

	mutex_lock(&chip->event_lock);
	switch (chip->cap.chrg_evt) {
	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
		pm_runtime_get_sync(&chip->client->dev);
	case POWER_SUPPLY_CHARGER_EVENT_UPDATE:
	case POWER_SUPPLY_CHARGER_EVENT_RESUME:
		dev_info(&chip->client->dev, "Enable charging\n");
		set_up_charging(chip, &reg);
		ret = enable_charging(chip, &reg);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"charge enabling failed\n");
			goto i2c_write_fail;
		}
		chip->present = 1;
		chip->online = 1;
		chip->chrg_type = chip->cap.chrg_type;
		chip->batt_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
		pm_runtime_put_sync(&chip->client->dev);
	case POWER_SUPPLY_CHARGER_EVENT_SUSPEND:
		dev_info(&chip->client->dev, "Disable charging\n");
		ret = stop_charging(chip);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"charge disabling failed\n");
			goto i2c_write_fail;
		}
		if (chip->cap.chrg_evt ==
			POWER_SUPPLY_CHARGER_EVENT_SUSPEND) {
			chip->present = 1;
		} else {
			chip->present = 0;
			chip->chrg_type = POWER_SUPPLY_TYPE_USB;
		}
		chip->online = 0;
		chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		dev_err(&chip->client->dev,
			"invalid charger event:%d\n", chip->cap.chrg_evt);
		goto i2c_write_fail;
	}

	power_supply_changed(&chip->usb);
i2c_write_fail:
	mutex_unlock(&chip->event_lock);
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

	ret = enable_charging(chip, &reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "charge enable failed\n");

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
	struct bq24192_chip *chip = container_of(psy,
				struct bq24192_chip, usb);

	mutex_lock(&chip->event_lock);
	chip->cap.chrg_evt = cap->chrg_evt;
	chip->cap.chrg_type = cap->chrg_type;
	chip->cap.mA = cap->mA;
	mutex_unlock(&chip->event_lock);

	dev_info(&chip->client->dev, "[chrg] evt:%d type:%d cur:%d\n",
				cap->chrg_evt, cap->chrg_type, cap->mA);
	schedule_delayed_work(&chip->chrg_evt_wrkr, 0);
}

static int bq24192_charger_callback(void *arg, int event,
					struct otg_bc_cap *cap)
{
	struct bq24192_chip *chip = (struct bq24192_chip *)arg;

	mutex_lock(&chip->event_lock);
	if (event == USBCHRG_EVENT_CONNECT)
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
	else if (event == USBCHRG_EVENT_UPDATE)
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_UPDATE;
	else if (event == USBCHRG_EVENT_RESUME)
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_RESUME;
	else if (event == USBCHRG_EVENT_SUSPEND)
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_SUSPEND;
	else
		chip->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
	chip->cap.mA = cap->mA;
	chip->cap.chrg_type = cap->chrg_type;
	mutex_unlock(&chip->event_lock);

	dev_info(&chip->client->dev, "[chrg] evt:%d type:%d cur:%d\n",
				event, cap->chrg_type, cap->mA);

	schedule_delayed_work(&chip->chrg_evt_wrkr, 0);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
#define DBGFS_REG_BUF_LEN	4

static int bq24192_show(struct seq_file *seq, void *unused)
{
	u16 val;
	long addr;

	if (strict_strtol((char *)seq->private, 16, &addr))
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
	char buf[DBGFS_REG_BUF_LEN] = {'\0', };
	long addr, value;
	int ret;
	struct seq_file *seq = file->private_data;

	if (!seq || strict_strtol((char *)seq->private, 16, &addr))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, DBGFS_REG_BUF_LEN))
		return -EFAULT;

	if (strict_strtoul(buf, 16, &value))
		return -EINVAL;

	dev_info(&bq24192_client->dev,
			"[dbgfs write] Addr:0x%x Val:0x%x\n", addr, value);

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

	mutex_lock(&chip->event_lock);
	switch (psp) {
	case POWER_SUPPLY_CHARGE_CURRENT_LIMIT:
		ret = update_ilim_settings(chip, val->intval);
		if (ret < 0) {
			mutex_unlock(&chip->event_lock);
			return ret;
		}
		chip->chrg_cur_cntl = val->intval;
		break;
	default:
		mutex_unlock(&chip->event_lock);
		return -EPERM;
	}
	mutex_unlock(&chip->event_lock);
	return 0;
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

static void init_charger_regs(struct bq24192_chip *chip)
{
	int ret;

	/* disable WDT timer */
	ret = program_wdt_timer(chip, CHRG_TIMER_EXP_CNTL_WDTDISABLE);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* disable the charger */
	ret = bq24192_reg_multi_bitset(chip->client, BQ24192_POWER_ON_CFG_REG,
						POWER_ON_CFG_CHRG_CFG_DIS,
					CHR_CFG_BIT_POS, CHR_CFG_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* disable Charge Termination */
	ret = bq24192_reg_read_modify(chip->client,
			BQ24192_CHRG_TIMER_EXP_CNTL_REG,
				CHRG_TIMER_EXP_CNTL_EN_TERM, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* set safty charge time to maximum */
	ret = bq24192_reg_multi_bitset(chip->client,
					BQ24192_CHRG_TIMER_EXP_CNTL_REG,
					CHRG_TIMER_EXP_CNTL_SFT_TIMER,
					SFT_TIMER_BIT_POS, SFT_TIMER_BIT_LEN);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* disable charger interrupts */
	ret = bq24192_reg_read_modify(chip->client,
					BQ24192_MISC_OP_CNTL_REG,
					MISC_OP_CNTL_MINT_CHRG, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);

	/* disable battery interrupts */
	ret = bq24192_reg_read_modify(chip->client,
					BQ24192_MISC_OP_CNTL_REG,
					MISC_OP_CNTL_MINT_BATT, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "I2C write failed:%s\n", __func__);
}

static int __devinit bq24192_probe(struct i2c_client *client,
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

	INIT_DELAYED_WORK(&chip->chrg_evt_wrkr, bq24192_event_worker);
	mutex_init(&chip->event_lock);

	chip->chrg_cur_cntl = POWER_SUPPLY_CHARGE_CURRENT_LIMIT_NONE;
	chip->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;

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

		/* Register with OTG */
		otg_handle = penwell_otg_register_bc_callback(
				bq24192_charger_callback, (void *)chip);
		if (!otg_handle) {
			dev_err(&client->dev, "battery: OTG Registration failed\n");
			power_supply_unregister(&chip->usb);
			i2c_set_clientdata(client, NULL);
			kfree(chip);
			return -EBUSY;
		}
	}

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
		return ret;
	}

	return 0;
}

static int __devexit bq24192_remove(struct i2c_client *client)
{
	struct bq24192_chip *chip = i2c_get_clientdata(client);

	bq24192_remove_debugfs(chip);
	if (!chip->pdata->slave_mode) {
		penwell_otg_unregister_bc_callback(otg_handle);
		power_supply_unregister(&chip->usb);
	}
	i2c_set_clientdata(client, NULL);
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
	.remove		= __devexit_p(bq24192_remove),
	.id_table	= bq24192_id,
};

static int __init bq24192_init(void)
{
	return i2c_add_driver(&bq24192_i2c_driver);
}
module_init(bq24192_init);

static void __exit bq24192_exit(void)
{
	i2c_del_driver(&bq24192_i2c_driver);
}
module_exit(bq24192_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("BQ24192 Charger Driver");
MODULE_LICENSE("GPL");
