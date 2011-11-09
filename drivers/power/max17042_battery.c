/*
 * max17042_battery.c - Fuel gauge driver for Maxim 17042 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver is based on max17040_battery.c
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/notifier.h>

/* Status register bits */
#define STATUS_POR_BIT		(1 << 1)
#define STATUS_BST_BIT		(1 << 3)
#define STATUS_VMN_BIT		(1 << 8)
#define STATUS_TMN_BIT		(1 << 9)
#define STATUS_SMN_BIT		(1 << 10)
#define STATUS_BI_BIT		(1 << 11)
#define STATUS_VMX_BIT		(1 << 12)
#define STATUS_TMX_BIT		(1 << 13)
#define STATUS_SMX_BIT		(1 << 14)
#define STATUS_BR_BIT		(1 << 15)

#define MAX17042_IC_VERSION	0x0092

#define MAX_VOLT_THRLD		0xD2	/* 4200mv */
#define MIN_VOLT_THRLD		0xBC	/* 3760mv */

#define MAX_TEMP_THRLD		0x3C	/* 60 Degrees */
#define MIN_TEMP_THRLD		0x0	/* 0 Degrees */

/* Interrupt mask bits */
#define CONFIG_ALRT_BIT_ENBL	(1 << 2)
#define STATUS_INTR_VOLT_BIT	(1 << 12)
#define STATUS_INTR_TEMP_BIT	(1 << 13)
#define STATUS_INTR_SOC_BIT	(1 << 14)

#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK		0x0080
#define FG_MODEL_UNLOCK1	0X0059
#define FG_MODEL_UNLOCK2	0X00C4
#define FG_MODEL_LOCK1		0X0000
#define FG_MODEL_LOCK2		0X0000

#define dQ_ACC_DIV	0x4
#define dP_ACC_100	0x1900
#define dP_ACC_200	0x3200

#define	NTC_47K_TGAIN	0xE4E4
#define	NTC_47K_TOFF	0x2F1D

#define BATT_CHRG_FULL_DES		1550000
#define MAX17042_VOLT_CONV_FCTR		625
#define MAX17042_CURR_CONV_FCTR		156
#define MAX17042_CHRG_CONV_FCTR		500

#define MAX17042_TEMP_SIGN_MASK		0x8000

#define MAX17042_MAX_MEM	(0xFF + 1)

#define MAX17042_MODEL_MUL_FACTOR(a)	((a * 10) / 7)
#define MAX17042_MODEL_DIV_FACTOR(a)	((a * 7) / 10)
#define CONSTANT_TEMP_IN_POWER_SUPPLY	350
#define POWER_SUPPLY_VOLT_MIN_THRESHOLD	3500000

#define CYCLES_ROLLOVER_CUTOFF		0x0100
#define MAX17042_DEF_RO_LRNCFG		0x0076

#define MAX17042_SIGN_INDICATOR		0x8000

#define BYTE_VALUE			1
#define WORD_VALUE			0

enum max17042_register {
	MAX17042_STATUS		= 0x00,
	MAX17042_VALRT_Th	= 0x01,
	MAX17042_TALRT_Th	= 0x02,
	MAX17042_SALRT_Th	= 0x03,
	MAX17042_AtRate		= 0x04,
	MAX17042_RepCap		= 0x05,
	MAX17042_RepSOC		= 0x06,
	MAX17042_Age		= 0x07,
	MAX17042_TEMP		= 0x08,
	MAX17042_VCELL		= 0x09,
	MAX17042_Current	= 0x0A,
	MAX17042_AvgCurrent	= 0x0B,
	MAX17042_Qresidual	= 0x0C,
	MAX17042_SOC		= 0x0D,
	MAX17042_AvSOC		= 0x0E,
	MAX17042_RemCap		= 0x0F,
	MAX17042_FullCAP	= 0x10,
	MAX17042_TTE		= 0x11,
	MAX17042_V_empty	= 0x12,

	MAX17042_RSLOW		= 0x14,

	MAX17042_AvgTA		= 0x16,
	MAX17042_Cycles		= 0x17,
	MAX17042_DesignCap	= 0x18,
	MAX17042_AvgVCELL	= 0x19,
	MAX17042_MinMaxTemp	= 0x1A,
	MAX17042_MinMaxVolt	= 0x1B,
	MAX17042_MinMaxCurr	= 0x1C,
	MAX17042_CONFIG		= 0x1D,
	MAX17042_ICHGTerm	= 0x1E,
	MAX17042_AvCap		= 0x1F,
	MAX17042_ManName	= 0x20,
	MAX17042_DevName	= 0x21,
	MAX17042_DevChem	= 0x22,
	MAX17042_FullCAPNom	= 0x23,

	MAX17042_TempNom	= 0x24,
	MAX17042_TempCold	= 0x25,
	MAX17042_TempHot	= 0x26,
	MAX17042_AIN		= 0x27,
	MAX17042_LearnCFG	= 0x28,
	MAX17042_SHFTCFG	= 0x29,
	MAX17042_RelaxCFG	= 0x2A,
	MAX17042_MiscCFG	= 0x2B,
	MAX17042_TGAIN		= 0x2C,
	MAx17042_TOFF		= 0x2D,
	MAX17042_CGAIN		= 0x2E,
	MAX17042_COFF		= 0x2F,

	MAX17042_SOCempty	= 0x33,
	MAX17042_T_empty	= 0x34,
	MAX17042_FullCAP0	= 0x35,

	MAX17042_LAvg_empty	= 0x36,
	MAX17042_FCTC		= 0x37,
	MAX17042_RCOMP0		= 0x38,
	MAX17042_TempCo		= 0x39,
	MAX17042_ETC		= 0x3A,
	MAX17042_K_empty0	= 0x3B,
	MAX17042_TaskPeriod	= 0x3C,
	MAX17042_FSTAT		= 0x3D,

	MAX17042_SHDNTIMER	= 0x3F,

	MAX17042_dQacc		= 0x45,
	MAX17042_dPacc		= 0x46,
	MAX17042_VFSOC0         = 0x48,
	MAX17042_VFRemCap	= 0x4A,

	MAX17042_QH		= 0x4D,
	MAX17042_QL		= 0x4E,

	MAX17042_VFSOC0Enable	= 0x60,
	MAX17042_MLOCKReg1	= 0x62,
	MAX17042_MLOCKReg2	= 0x63,
	MAX17042_MODELChrTbl	= 0x80,
	MAX17042_OCV		= 0xEE,
	MAX17042_OCVInternal	= 0xFB,
	MAX17042_VFSOC		= 0xFF,

};

#define DRV_NAME "max17042_battery"
#define DEV_NAME "max17042"

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT	3

/* No of cell characterization words to be written to max17042 */
#define CELL_CHAR_TBL_SAMPLES	48

static uint16_t cell_char_tbl[] = {
	/* Data to be written from 0x80h */
	0xA250, 0xB720, 0xB800, 0xB880, 0xB920, 0xBA00, 0xBA60, 0xBBF0,
	0xBCF0, 0xBE50, 0xC060, 0xC2D0, 0xC520, 0xC750, 0xCA00, 0xD090,
	/* Data to be written from 0x90h */
	0x0120, 0x1C80, 0x0470, 0x0440, 0x0100, 0x5500, 0x0960, 0x2410,
	0x2250, 0x15F0, 0x0BD0, 0x0D00, 0x0B00, 0x0BB0, 0x08A0, 0x08A0,
	/* Data to be written from 0xA0h */
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
};

struct max17042_config_data {
	/*
	 * if config_init is 0, which means new
	 * configuration has been loaded in that case
	 * we need to perform complete init of chip
	 */
	u16	size;
	u8	table_type;
	u8	config_init;

	u16	rcomp0;
	u16	tempCo;
	u16	kempty0;
	u16	full_cap;
	u16	cycles;
	u16	full_capnom;
	u16	soc_empty;
	u16	ichgt_term;
	u16	design_cap;
	u16	etc;
	u16	rsense;
	u16	cfg;
	u16	learn_cfg;
	u16	filter_cfg;
	u16	relax_cfg;
	u16	cell_char_tbl[CELL_CHAR_TBL_SAMPLES];
} __packed;

struct max17042_chip {
	struct i2c_client *client;
	struct power_supply battery;
	struct max17042_platform_data *pdata;
	struct mutex batt_lock;
	struct mutex init_lock;

	int present;
	int status;
	int health;
	int technology;
	int charge_full_des;
	struct delayed_work init_worker;
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *max17042_dbgfs_root;
static char max17042_dbg_regs[MAX17042_MAX_MEM][4];
#endif

static int max17042_reboot_callback(struct notifier_block *nfb,
					unsigned long event, void *data);

static struct notifier_block max17042_reboot_notifier_block = {
	.notifier_call = max17042_reboot_callback,
	.priority = 0,
};

static void set_soc_intr_thresholds(struct max17042_chip *chip, u16 off);
static void save_runtime_params(struct max17042_chip *chip);
static u16 fg_vfSoc;
static struct max17042_config_data *fg_conf_data;
static struct i2c_client *max17042_client;

static enum power_supply_property max17042_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int max17042_write_reg(struct i2c_client *client, u8 reg, u16 value)
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

static int max17042_read_reg(struct i2c_client *client, u8 reg)
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

static int max17042_write_verify_reg(struct i2c_client *client,
						u8 reg, u16 value)
{
	int ret;

	/* Write the value to register */
	ret = max17042_write_reg(client, reg, value);

	/* Read the value from register */
	ret = max17042_read_reg(client, reg);

	/* compare the both the values */
	if (value != ret)
		dev_err(&client->dev,
			"write verify failed on Register:0x%x\n", reg);

	return ret;
}

static int max17042_reg_read_modify(struct i2c_client *client, u8 reg,
							u16 val, int bit_set)
{
	u16 data;
	int ret;

	data = max17042_read_reg(client, reg);

	if (bit_set)
		data |= val;
	else
		data &= (~val);

	ret = max17042_write_reg(client, reg, data);
	return ret;
}

static irqreturn_t max17042_intr_handler(int id, void *dev)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t max17042_thread_handler(int id, void *dev)
{
	struct max17042_chip *chip = dev;
	u16 val;

	pm_runtime_get_sync(&chip->client->dev);

	mutex_lock(&chip->batt_lock);
	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	if (val & STATUS_INTR_VOLT_BIT)
		dev_info(&chip->client->dev, "Volatge threshold INTR\n");

	if (val & STATUS_INTR_TEMP_BIT)
		dev_info(&chip->client->dev, "Temperature threshold INTR\n");

	if (val & STATUS_INTR_SOC_BIT) {
		dev_info(&chip->client->dev, "SOC threshold INTR\n");
		set_soc_intr_thresholds(chip, 1);
	}
	mutex_unlock(&chip->batt_lock);

	power_supply_changed(&chip->battery);
	pm_runtime_put_sync(&chip->client->dev);
	return IRQ_HANDLED;
}

static short adjust_sign_value(int value, int is_byte)
{
	short result, temp = (short)value;
	if (temp & MAX17042_SIGN_INDICATOR) {

		if (is_byte) {
			result = (~temp) >> 8;
			result &= 0xff;
		} else {
			result = ~temp;
		}

		result++;
		result *= -1;
	} else {
		if (is_byte)
			result = temp >> 8;
		else
			result = temp;
	}

	return result;
}

static int read_batt_pack_temp(struct max17042_chip *chip, int *temp)
{
	int ret;
	u16 val;

	/* Read battery pack temperature */
	if (chip->pdata->battery_pack_temp) {
		ret = chip->pdata->battery_pack_temp(temp);
		if (ret < 0)
			goto temp_read_err;
		val = (0xff & (char)*temp) << 8;
		ret = max17042_write_reg(chip->client, MAX17042_TEMP, val);
		if (ret < 0)
			goto temp_read_err;
	} else {
		ret = max17042_read_reg(chip->client, MAX17042_TEMP);
		if (ret < 0)
			goto temp_read_err;

		/* MAX17042_TEMP register gives the signed
		 * value and we are ignoring the lower byte
		 * which represents the decimal point */

		*temp = adjust_sign_value(ret, BYTE_VALUE);
	}

	return 0;

temp_read_err:
	dev_err(&chip->client->dev, "BP Temp read error:%d", ret);
	return ret;
}

static int max17042_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17042_chip *chip = container_of(psy,
				struct max17042_chip, battery);
	short int cur;
	int volt_ocv, ret, batt_temp;

	mutex_lock(&chip->batt_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = chip->technology;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->charge_full_des;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = max17042_read_reg(chip->client, MAX17042_RepCap);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = ret * MAX17042_CHRG_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = max17042_read_reg(chip->client, MAX17042_FullCAP);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = ret * MAX17042_CHRG_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = max17042_read_reg(chip->client, MAX17042_Current);
		if (ret < 0)
			goto ps_prop_read_err;

		cur = adjust_sign_value(ret, WORD_VALUE);

		if (fg_conf_data->rsense)
			val->intval = (cur * MAX17042_CURR_CONV_FCTR)
						/ fg_conf_data->rsense;
		else
			val->intval = cur * MAX17042_CURR_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = max17042_read_reg(chip->client, MAX17042_AvgCurrent);
		if (ret < 0)
			goto ps_prop_read_err;

		cur = adjust_sign_value(ret, WORD_VALUE);

		if (fg_conf_data->rsense)
			val->intval = (cur * MAX17042_CURR_CONV_FCTR)
						/ fg_conf_data->rsense;
		else
			val->intval = cur * MAX17042_CURR_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!chip->pdata->enable_current_sense) {
			val->intval = CONSTANT_TEMP_IN_POWER_SUPPLY;
			break;
		}
		ret = read_batt_pack_temp(chip, &batt_temp);
		if (ret < 0)
			goto ps_prop_read_err;
		/*
		 * Temperature is measured in units of degrees celcius, the
		 * power_supply class measures temperature in tenths of degrees
		 * celsius.
		 */
		val->intval = batt_temp * 10;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max17042_read_reg(chip->client, MAX17042_OCVInternal);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = (ret >> 3) * MAX17042_VOLT_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max17042_read_reg(chip->client, MAX17042_AvgVCELL);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = (ret >> 3) * MAX17042_VOLT_CONV_FCTR;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = max17042_read_reg(chip->client, MAX17042_V_empty);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = (ret >> 7) * 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* FIXME: WA to avoid modem crash, should be removed once
		 * everybody starts using the PR2 POR batteries */
		ret = max17042_read_reg(chip->client, MAX17042_OCVInternal);
		if (ret < 0)
			goto ps_prop_read_err;
		volt_ocv = (ret >> 3) * MAX17042_VOLT_CONV_FCTR;
		/* Check if the OCV is less than
		 * IA_APPS_RUN(3.6V) Threshold */
		if ((chip->pdata->enable_current_sense && volt_ocv <= 3600000)
				|| (chip->health == POWER_SUPPLY_HEALTH_DEAD)) {
			val->intval = 0;
			break;
		}
		if (!chip->pdata->enable_current_sense &&
				volt_ocv <= POWER_SUPPLY_VOLT_MIN_THRESHOLD) {
			val->intval = 0;
			break;
		}
		/* If current sensing is not enabled then read the
		 * voltage based fuel gauge register for SOC */
		if (chip->pdata->enable_current_sense)
			ret = max17042_read_reg(chip->client, MAX17042_RepSOC);
		else
			ret = max17042_read_reg(chip->client, MAX17042_VFSOC);
		if (ret < 0)
			goto ps_prop_read_err;
		val->intval = ret >> 8;
		/* Check if MSB of lower byte is set
		 * then round off the SOC to higher digit
		 */
		if (ret & 0x80)
			val->intval += 1;

		if (val->intval > 100)
			val->intval = 100;
		break;
	default:
		mutex_unlock(&chip->batt_lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->batt_lock);
	return 0;

ps_prop_read_err:
	mutex_unlock(&chip->batt_lock);
	return ret;
}

static void init_fg_config_data(void)
{
	fg_conf_data->cfg = 0x2210;
	fg_conf_data->learn_cfg = 0x0076;
	fg_conf_data->filter_cfg = 0x87A4;
	fg_conf_data->relax_cfg = 0x506B;
	memcpy(&fg_conf_data->cell_char_tbl, cell_char_tbl,
					sizeof(cell_char_tbl));
	fg_conf_data->rcomp0 = 0x0056;
	fg_conf_data->tempCo = 0x1621;
	fg_conf_data->etc = 0x2D51;
	fg_conf_data->kempty0 = 0x0350;
	fg_conf_data->ichgt_term = 0x0140;
	fg_conf_data->full_cap = 3100;
	fg_conf_data->design_cap = 3100;
	fg_conf_data->full_capnom = 3100;
	fg_conf_data->rsense = 1;
}

static void dump_fg_conf_data(struct max17042_chip *chip)
{
	int i;

	dev_info(&chip->client->dev, "size:%x\n", fg_conf_data->size);
	dev_info(&chip->client->dev, "table_type:%x\n",
					fg_conf_data->table_type);
	dev_info(&chip->client->dev, "config_init:%x\n",
					fg_conf_data->config_init);
	dev_info(&chip->client->dev, "rcomp0:%x\n", fg_conf_data->rcomp0);
	dev_info(&chip->client->dev, "tempCo:%x\n", fg_conf_data->tempCo);
	dev_info(&chip->client->dev, "kempty0:%x\n", fg_conf_data->kempty0);
	dev_info(&chip->client->dev, "full_cap:%x\n", fg_conf_data->full_cap);
	dev_info(&chip->client->dev, "cycles:%x\n", fg_conf_data->cycles);
	dev_info(&chip->client->dev, "full_capnom:%x\n",
						fg_conf_data->full_capnom);
	dev_info(&chip->client->dev, "soc_empty:%x\n",
						fg_conf_data->soc_empty);
	dev_info(&chip->client->dev, "ichgt_term:%x\n",
						fg_conf_data->ichgt_term);
	dev_info(&chip->client->dev, "design_cap:%x\n",
						fg_conf_data->design_cap);
	dev_info(&chip->client->dev, "etc:%x\n", fg_conf_data->etc);
	dev_info(&chip->client->dev, "rsense:%x\n", fg_conf_data->rsense);
	dev_info(&chip->client->dev, "cfg:%x\n", fg_conf_data->cfg);
	dev_info(&chip->client->dev, "learn_cfg:%x\n",
						fg_conf_data->learn_cfg);
	dev_info(&chip->client->dev, "filter_cfg:%x\n",
						fg_conf_data->filter_cfg);
	dev_info(&chip->client->dev, "relax_cfg:%x\n", fg_conf_data->relax_cfg);

	for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
		dev_info(&chip->client->dev, "%x, ",
				fg_conf_data->cell_char_tbl[i]);
	dev_info(&chip->client->dev, "\n");
}

static void enable_soft_POR(struct max17042_chip *chip)
{
	u16 val = 0x0000;

	max17042_write_reg(chip->client, MAX17042_MLOCKReg1, val);
	max17042_write_reg(chip->client, MAX17042_MLOCKReg2, val);
	max17042_write_reg(chip->client, MAX17042_STATUS, val);

	val = max17042_read_reg(chip->client, MAX17042_MLOCKReg1);
	if (val)
		dev_err(&chip->client->dev, "MLOCKReg1 read failed\n");

	val = max17042_read_reg(chip->client, MAX17042_MLOCKReg2);
	if (val)
		dev_err(&chip->client->dev, "MLOCKReg2 read failed\n");

	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	if (val)
		dev_err(&chip->client->dev, "STATUS read failed\n");

	/* send POR command */
	max17042_write_reg(chip->client, MAX17042_VFSOC0Enable, 0x000F);
	mdelay(2);

	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	if (val & STATUS_POR_BIT)
		dev_info(&chip->client->dev, "SoftPOR done!\n");
	else
		dev_err(&chip->client->dev, "SoftPOR failed\n");
}

static int write_characterization_data(struct max17042_chip *chip)
{
	uint16_t cell_data[CELL_CHAR_TBL_SAMPLES];
	uint16_t temp_data[CELL_CHAR_TBL_SAMPLES];
	int i;
	u8 addr;

	memset(cell_data, 0x0, sizeof(cell_data));
	/* Unlock model access */
	max17042_write_reg(chip->client, MAX17042_MLOCKReg1, FG_MODEL_UNLOCK1);
	max17042_write_reg(chip->client, MAX17042_MLOCKReg2, FG_MODEL_UNLOCK2);
	addr = MAX17042_MODELChrTbl;

	/* write the 48 words */
	for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
		max17042_write_reg(chip->client, addr + i,
				fg_conf_data->cell_char_tbl[i]);

	/* read the 48 words */
	for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
		cell_data[i] = max17042_read_reg(chip->client, addr + i);

	/* compare the data */
	if (memcmp(cell_data, fg_conf_data->cell_char_tbl, sizeof(cell_data))) {
		dev_err(&chip->client->dev, "%s write failed\n", __func__);
		for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
			dev_err(&chip->client->dev, "0x%x,0x%x\n", cell_data[i],
						fg_conf_data->cell_char_tbl[i]);
		/* Lock Model access regs */
		max17042_write_reg(chip->client, MAX17042_MLOCKReg1,
								FG_MODEL_LOCK1);
		max17042_write_reg(chip->client, MAX17042_MLOCKReg2,
								FG_MODEL_LOCK2);
		return -EIO;
	}

	memset(temp_data, 0x0, sizeof(temp_data));
	/* Lock Model access regs */
	max17042_write_reg(chip->client, MAX17042_MLOCKReg1, FG_MODEL_LOCK1);
	max17042_write_reg(chip->client, MAX17042_MLOCKReg2, FG_MODEL_LOCK2);

	/* read the 48 words */
	for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
		cell_data[i] = max17042_read_reg(chip->client, addr + i);

	/* compare the data */
	if (memcmp(cell_data, temp_data, sizeof(temp_data))) {
		dev_err(&chip->client->dev, "%s verify failed\n", __func__);
		for (i = 0; i < CELL_CHAR_TBL_SAMPLES; i++)
			dev_err(&chip->client->dev, "0x%x, ", cell_data[i]);
		dev_err(&chip->client->dev, "\n");
		return -EIO;
	}

	return 0;
}

static void configure_learncfg(struct max17042_chip *chip)
{

	u16 cycles;

	cycles = max17042_read_reg(chip->client, MAX17042_Cycles);
	if (cycles > CYCLES_ROLLOVER_CUTOFF)
		max17042_write_reg(chip->client, MAX17042_LearnCFG,
						MAX17042_DEF_RO_LRNCFG);
	else
		max17042_write_reg(chip->client, MAX17042_LearnCFG,
						fg_conf_data->learn_cfg);

}

static void write_config_regs(struct max17042_chip *chip)
{
	max17042_write_reg(chip->client, MAX17042_CONFIG, fg_conf_data->cfg);
	configure_learncfg(chip);

	max17042_write_reg(chip->client, MAX17042_SHFTCFG,
						fg_conf_data->filter_cfg);
	max17042_write_reg(chip->client, MAX17042_RelaxCFG,
						fg_conf_data->relax_cfg);
}

static void write_custom_regs(struct max17042_chip *chip)
{
	max17042_write_verify_reg(chip->client, MAX17042_RCOMP0,
						fg_conf_data->rcomp0);
	max17042_write_verify_reg(chip->client, MAX17042_TempCo,
						fg_conf_data->tempCo);
	max17042_write_reg(chip->client, MAX17042_ETC, fg_conf_data->etc);
	max17042_write_verify_reg(chip->client, MAX17042_K_empty0,
						fg_conf_data->kempty0);
	max17042_write_verify_reg(chip->client, MAX17042_ICHGTerm,
						fg_conf_data->ichgt_term);
	max17042_write_verify_reg(chip->client, MAX17042_SOCempty,
						fg_conf_data->soc_empty);
}

static void update_capacity_regs(struct max17042_chip *chip)
{
	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
			MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_cap)
			* fg_conf_data->rsense);
	max17042_write_reg(chip->client, MAX17042_DesignCap,
			fg_conf_data->design_cap * fg_conf_data->rsense);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAPNom,
			MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_capnom)
			* fg_conf_data->rsense);
}

static void reset_vfsoc0_reg(struct max17042_chip *chip)
{
	fg_vfSoc = max17042_read_reg(chip->client, MAX17042_VFSOC);
	max17042_write_reg(chip->client, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	max17042_write_verify_reg(chip->client, MAX17042_VFSOC0, fg_vfSoc);
	max17042_write_reg(chip->client, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
}

static void load_new_capacity_params(struct max17042_chip *chip, bool is_por)
{
	u16 full_cap0, rem_cap, rep_cap, dq_acc;

	if (is_por) {
		full_cap0 = max17042_read_reg(chip->client, MAX17042_FullCAP0);

		/* fg_vfSoc needs to shifted by 8 bits to get the
		 * perc in 1% accuracy, to get the right rem_cap multiply
		 * full_cap0, fg_vfSoc and devide by 100
		 */
		rem_cap = ((fg_vfSoc >> 8) * (u32)full_cap0) / 100;
		max17042_write_verify_reg(chip->client,
					MAX17042_RemCap, rem_cap);

		rep_cap = rem_cap;
		max17042_write_verify_reg(chip->client,
					MAX17042_RepCap, rep_cap);
	}

	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	dq_acc = MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_cap) / dQ_ACC_DIV;
	max17042_write_verify_reg(chip->client, MAX17042_dQacc, dq_acc);
	max17042_write_verify_reg(chip->client, MAX17042_dPacc, dP_ACC_200);

	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
			MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_cap)
			* fg_conf_data->rsense);
	max17042_write_reg(chip->client, MAX17042_DesignCap,
			fg_conf_data->design_cap * fg_conf_data->rsense);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAPNom,
			MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_capnom)
			* fg_conf_data->rsense);
}

static void save_runtime_params(struct max17042_chip *chip)
{
	int size, retval;

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	if (!chip->pdata->save_config_data || !chip->pdata->is_init_done)
		return ;

	fg_conf_data->rcomp0 = max17042_read_reg(chip->client,
							MAX17042_RCOMP0);
	fg_conf_data->tempCo = max17042_read_reg(chip->client,
							MAX17042_TempCo);
	fg_conf_data->kempty0 = max17042_read_reg(chip->client,
							MAX17042_K_empty0);
	fg_conf_data->full_capnom = max17042_read_reg(chip->client,
							MAX17042_FullCAPNom);
	fg_conf_data->full_cap = max17042_read_reg(chip->client,
							MAX17042_FullCAP);
	if (fg_conf_data->rsense) {
		fg_conf_data->full_capnom = MAX17042_MODEL_DIV_FACTOR(
			fg_conf_data->full_capnom) / fg_conf_data->rsense;
		fg_conf_data->full_cap /= fg_conf_data->rsense;
	}
	fg_conf_data->cycles = max17042_read_reg(chip->client,
							MAX17042_Cycles);

	/* Dump data before saving */
	dump_fg_conf_data(chip);

	size = sizeof(*fg_conf_data) - sizeof(fg_conf_data->cell_char_tbl);
	retval = chip->pdata->save_config_data(DRV_NAME, fg_conf_data, size);
	if (retval < 0) {
		dev_err(&chip->client->dev, "%s failed\n", __func__);
		return ;
	}

}

static void restore_runtime_params(struct max17042_chip *chip)
{
	u16 full_cap0, rem_cap, soc, dq_acc;
	int size, retval;

	if (!chip->pdata->restore_config_data)
		return ;

	size = sizeof(*fg_conf_data) - sizeof(fg_conf_data->cell_char_tbl);
	retval = chip->pdata->restore_config_data(DRV_NAME, fg_conf_data, size);
	if (retval < 0) {
		dev_err(&chip->client->dev, "%s failed\n", __func__);
		return ;
	}

	/* Dump data after restoring */
	dump_fg_conf_data(chip);

	max17042_write_verify_reg(chip->client, MAX17042_RCOMP0,
						fg_conf_data->rcomp0);
	max17042_write_verify_reg(chip->client, MAX17042_TempCo,
						fg_conf_data->tempCo);
	max17042_write_verify_reg(chip->client, MAX17042_K_empty0,
						fg_conf_data->kempty0);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAPNom,
			MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_capnom)
			* fg_conf_data->rsense);
	max17042_write_verify_reg(chip->client, MAX17042_Cycles,
						fg_conf_data->cycles);

	/* delay must be atleast 350mS to allow SOC
	 * to be calculated from the restored configuration
	 */
	msleep(350);

	/* restore full capacity value */
	full_cap0 = max17042_read_reg(chip->client, MAX17042_FullCAP0);
	soc = max17042_read_reg(chip->client, MAX17042_SOC);

	rem_cap = (soc * (u32)full_cap0) / 25600;
	max17042_write_verify_reg(chip->client, MAX17042_RemCap, rem_cap);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
			fg_conf_data->full_cap * fg_conf_data->rsense);

	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	dq_acc = MAX17042_MODEL_MUL_FACTOR(fg_conf_data->full_cap) / dQ_ACC_DIV;
	max17042_write_verify_reg(chip->client, MAX17042_dQacc, dq_acc);
	max17042_write_verify_reg(chip->client, MAX17042_dPacc, dP_ACC_200);

	/* delay must be atleast 350mS to write Cycles
	 * value from the restored configuration
	 */
	msleep(350);

	max17042_write_verify_reg(chip->client, MAX17042_Cycles,
						fg_conf_data->cycles);
}

static int max17042_reboot_callback(struct notifier_block *nfb,
					unsigned long event, void *data)
{
	struct max17042_chip *chip = i2c_get_clientdata(max17042_client);

	save_runtime_params(chip);
	return NOTIFY_OK;
}

static int init_max17042_chip(struct max17042_chip *chip)
{
	int ret = 0, val;
	bool is_por;

	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	dev_info(&chip->client->dev, "Status reg: %x\n", val);

	if (val & STATUS_POR_BIT)
		is_por = true;
	else
		is_por = false;

	/* Initialize configuration */
	write_config_regs(chip);

	/* write cell characterization data */
	ret = write_characterization_data(chip);
	if (ret < 0)
		return ret;

	/* write custom parameters */
	write_custom_regs(chip);

	/* update capacity params */
	update_capacity_regs(chip);

	/* delay must be atleast 350mS to allow VFSOC
	 * to be calculated from the new configuration
	 */
	msleep(350);

	/* reset vfsoc0 reg */
	reset_vfsoc0_reg(chip);

	/* advance to coulomb counter mode */
	max17042_write_verify_reg(chip->client,
			MAX17042_Cycles, fg_conf_data->cycles);

	/* adjust Temperature gain and offset */
	max17042_write_reg(chip->client,
			MAX17042_TGAIN, NTC_47K_TGAIN);
	max17042_write_reg(chip->client,
			MAx17042_TOFF, NTC_47K_TOFF);

	/* load new capacity params */
	load_new_capacity_params(chip, is_por);

	if (is_por) {
		/* Init complete, Clear the POR bit */
		val = max17042_read_reg(chip->client, MAX17042_STATUS);
		max17042_write_reg(chip->client, MAX17042_STATUS,
						val & (~STATUS_POR_BIT));
	}

	/* reset FullCap to non inflated value */
	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
			fg_conf_data->full_cap * fg_conf_data->rsense);

	return ret;
}

static void reset_max17042(struct max17042_chip *chip)
{
	int val;

	/* do soft power reset */
	enable_soft_POR(chip);

	/* After Power up, the MAX17042 requires 500mS in order
	 * to perform signal debouncing and initial SOC reporting
	 */
	msleep(500);

	max17042_write_reg(chip->client, MAX17042_CONFIG, 0x2210);

	/* adjust Temperature gain and offset */
	max17042_write_reg(chip->client, MAX17042_TGAIN, NTC_47K_TGAIN);
	max17042_write_reg(chip->client, MAx17042_TOFF, NTC_47K_TOFF);

	/* Init complete, Clear the POR bit */
	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	max17042_write_reg(chip->client, MAX17042_STATUS,
					val & (~STATUS_POR_BIT));

}

static void max17042_restore_conf_data(struct max17042_chip *chip)
{
	int retval = 0, val, size;

	/* return if lock already acquired */
	if (!mutex_trylock(&chip->init_lock))
		return;

	if (!chip->pdata->is_init_done && chip->pdata->restore_config_data) {
		retval = chip->pdata->restore_config_data(DRV_NAME,
					fg_conf_data, sizeof(*fg_conf_data));

		if (retval == -ENXIO) {		/* no device found */
			dev_err(&chip->client->dev, "device not found\n");
			chip->pdata->is_init_done = 1;
			chip->pdata->save_config_data = NULL;
		} else if (retval < 0) {	/* device not ready */
			dev_warn(&chip->client->dev, "device not ready\n");
		} else {			/* device ready */
			/* Dump data after restoring */
			dump_fg_conf_data(chip);

			val = max17042_read_reg(chip->client, MAX17042_STATUS);
			dev_info(&chip->client->dev, "Status reg: %x\n", val);
			if (!fg_conf_data->config_init ||
						(val & STATUS_POR_BIT)) {
				dev_info(&chip->client->dev,
					"config data needs to be loaded\n");
				retval = init_max17042_chip(chip);
				if (retval < 0) {
					dev_err(&chip->client->dev,
						"maxim chip init failed\n");
					reset_max17042(chip);
					chip->pdata->save_config_data = NULL;
				}
			}
			chip->pdata->is_init_done = 1;

			/* mark the dirty byte in non-volatile memory */
			if (!fg_conf_data->config_init && retval >= 0) {
				fg_conf_data->config_init = 0x1;
				size = sizeof(*fg_conf_data) -
					sizeof(fg_conf_data->cell_char_tbl);
				retval = chip->pdata->save_config_data(
					DRV_NAME, fg_conf_data, size);
				if (retval < 0)
					dev_err(&chip->client->dev,
						"%s failed\n", __func__);
			}
		}
	}

	/* Check if current sensing is enabled */
	if (chip->pdata->is_init_done && (retval == 0)) {
		if (chip->pdata->current_sense_enabled)
			chip->pdata->enable_current_sense =
				chip->pdata->current_sense_enabled();
		else
			chip->pdata->enable_current_sense = 1;
	}

	if (chip->pdata->enable_current_sense) {
		dev_info(&chip->client->dev, "current sensing enabled\n");
		/* enable coulomb counter based fuel gauging */
		configure_learncfg(chip);

		/* enable Alerts for SOCRep */
		max17042_write_reg(chip->client, MAX17042_MiscCFG, 0x0000);

		chip->technology = chip->pdata->technology;
	} else {
		dev_info(&chip->client->dev, "current sensing NOT enabled\n");
		/* Enable voltage based Fuel Gauging */
		max17042_write_reg(chip->client, MAX17042_LearnCFG, 0x0007);
		/* configure interrupts for SOCvf */
		max17042_write_reg(chip->client, MAX17042_MiscCFG, 0x0003);

		chip->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	mutex_unlock(&chip->init_lock);
}

static void max17042_init_worker(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(work,
				struct max17042_chip, init_worker.work);

	dev_info(&chip->client->dev, "%s\n", __func__);
	max17042_restore_conf_data(chip);
}

static void set_soc_intr_thresholds(struct max17042_chip *chip, u16 off)
{
	u16 soc, soc_tr;

	/* program interrupt thesholds such that we should
	 * get interrupt for every 'off' perc change in the soc
	 */
	soc = max17042_read_reg(chip->client, MAX17042_RepSOC) >> 8;
	soc_tr = (soc + off) << 8;
	soc_tr |= (soc - off);
	max17042_write_reg(chip->client, MAX17042_SALRT_Th, soc_tr);
}

static void set_intr_thresholds(struct max17042_chip *chip)
{
	u16 volt_tr, temp_tr;

	/* Max voltage threshold set to 4200mV */
	volt_tr = MAX_VOLT_THRLD << 8;
	/* Min voltage threshold set to 3760mV */
	volt_tr |= MIN_VOLT_THRLD;
	max17042_write_reg(chip->client, MAX17042_VALRT_Th, volt_tr);

	/* Max temperature threshold set 60 Degrees */
	temp_tr = MAX_TEMP_THRLD << 8;
	/* Min temperature threshold set 0 Degrees */
	temp_tr |= MIN_TEMP_THRLD;
	max17042_write_reg(chip->client, MAX17042_TALRT_Th, temp_tr);

	/* set soc threshold */
	set_soc_intr_thresholds(chip, 1);
}

static void max17042_external_power_changed(struct power_supply *psy)
{
	struct max17042_chip *chip = container_of(psy,
				struct max17042_chip, battery);

	pm_runtime_get_sync(&chip->client->dev);

	mutex_lock(&chip->batt_lock);
	/* get the battery status */
	if (chip->pdata->battery_status)
		chip->status = chip->pdata->battery_status();

	/* get the battery health */
	if (chip->pdata->battery_health)
		chip->health = chip->pdata->battery_health();
	mutex_unlock(&chip->batt_lock);

	/* Init maxim chip if it is not already initialized */
	if (!chip->pdata->is_init_done)
		schedule_delayed_work(&chip->init_worker, 0);

	power_supply_changed(&chip->battery);
	pm_runtime_put_sync(&chip->client->dev);
}

static void init_battery_props(struct max17042_chip *chip)
{
	u16 val;

	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	/* check battery present bit */
	if (val & STATUS_BST_BIT)
		chip->present = 0;
	else
		chip->present = 1;

	chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	chip->technology = chip->pdata->technology;
	chip->charge_full_des = BATT_CHRG_FULL_DES;
}

#ifdef CONFIG_DEBUG_FS
/**
 * max17042_show - debugfs: show the state of an endpoint.
 * @seq: The seq_file to write data to.
 * @unused: not used
 *
 * This debugfs entry shows the content of the register
 * given in the data parameter.
*/
static int max17042_show(struct seq_file *seq, void *unused)
{
	u16 val;
	long addr;

	if (strict_strtol((char *)seq->private, 16, &addr))
		return -EINVAL;

	val = max17042_read_reg(max17042_client, addr);
	seq_printf(seq, "%x\n", val);

	return 0;
}

static int max17042_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, max17042_show, inode->i_private);
}

static const struct file_operations max17042_dbgfs_fops = {
	.owner		= THIS_MODULE,
	.open		= max17042_dbgfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void max17042_create_debugfs(struct max17042_chip *chip)
{
	int i;
	struct dentry *entry;

	max17042_dbgfs_root = debugfs_create_dir(DEV_NAME, NULL);
	if (IS_ERR(max17042_dbgfs_root)) {
		dev_warn(&chip->client->dev, "DEBUGFS DIR create failed\n");
		return ;
	}

	for (i = 0; i < MAX17042_MAX_MEM; i++) {
		sprintf((char *)&max17042_dbg_regs[i], "%x", i);
		entry = debugfs_create_file(
					(const char *)&max17042_dbg_regs[i],
					S_IRUGO,
					max17042_dbgfs_root,
					&max17042_dbg_regs[i],
					&max17042_dbgfs_fops);
		if (IS_ERR(entry)) {
			debugfs_remove_recursive(max17042_dbgfs_root);
			max17042_dbgfs_root = NULL;
			dev_warn(&chip->client->dev,
					"DEBUGFS entry Create failed\n");
			return ;
		}
	}
}
static inline void max17042_remove_debugfs(struct max17042_chip *chip)
{
	if (max17042_dbgfs_root)
		debugfs_remove_recursive(max17042_dbgfs_root);
}
#else
static inline void max17042_create_debugfs(struct max17042_chip *chip)
{
}
static inline void max17042_remove_debugfs(struct max17042_chip *chip)
{
}
#endif

static int __devinit max17042_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17042_chip *chip;
	int ret;
	if (!client->dev.platform_data) {
		dev_err(&client->dev, "Platform Data is NULL");
		return -EFAULT;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
				"SM bus doesn't support DWORD transactions\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	fg_conf_data = kzalloc(sizeof(*fg_conf_data), GFP_KERNEL);
	if (!fg_conf_data) {
		dev_err(&client->dev, "mem alloc failed\n");
		kfree(chip);
		return -ENOMEM;
	}
	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
	max17042_client = client;

	ret = max17042_read_reg(chip->client, MAX17042_DevName);
	if (ret != MAX17042_IC_VERSION) {
		dev_err(&client->dev, "device version mismatch: %x\n", ret);
		kfree(chip);
		kfree(fg_conf_data);
		return -EIO;
	}

	/* init battery properties */
	init_battery_props(chip);
	INIT_DELAYED_WORK(&chip->init_worker, max17042_init_worker);
	mutex_init(&chip->batt_lock);
	mutex_init(&chip->init_lock);

	/* Initialize the chip with battery config data */
	max17042_restore_conf_data(chip);

	chip->battery.name		= "max17042_battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17042_get_property;
	chip->battery.external_power_changed = max17042_external_power_changed;
	chip->battery.properties	= max17042_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17042_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		kfree(fg_conf_data);
		return ret;
	}

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&chip->client->dev);
	pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);


	/* register interrupt */
	ret = request_threaded_irq(client->irq, max17042_intr_handler,
						max17042_thread_handler,
						0, DRV_NAME, chip);
	if (ret)
		dev_warn(&client->dev, "%s(): cannot get IRQ\n", __func__);
	else
		dev_info(&client->dev, "IRQ No:%d\n", client->irq);

	/* Enable Interrupts */
	max17042_reg_read_modify(client, MAX17042_CONFIG,
						CONFIG_ALRT_BIT_ENBL, 1);
	/* set the Interrupt threshold registers */
	set_intr_thresholds(chip);

	/* Create debugfs for maxim registers */
	max17042_create_debugfs(chip);

	/* Register reboot notifier callback */
	register_reboot_notifier(&max17042_reboot_notifier_block);

	return 0;
}

static int __devexit max17042_remove(struct i2c_client *client)
{
	struct max17042_chip *chip = i2c_get_clientdata(client);

	unregister_reboot_notifier(&max17042_reboot_notifier_block);
	max17042_remove_debugfs(chip);
	free_irq(client->irq, chip);
	power_supply_unregister(&chip->battery);
	pm_runtime_get_noresume(&chip->client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	kfree(fg_conf_data);
	return 0;
}

#ifdef CONFIG_PM
static int max17042_suspend(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	/* max17042 IC automatically goes into shutdown mode
	 * if the SCL and SDA were held low for more than
	 * timeout of SHDNTIMER register value
	 */
	dev_dbg(&chip->client->dev, "max17042 suspend\n");

	return 0;
}

static int max17042_resume(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	/* max17042 IC automatically wakes up if any edge
	 * on SDCl or SDA if we set I2CSH of CONFG reg
	 */
	dev_dbg(&chip->client->dev, "max17042 resume\n");

	return 0;
}
#else
#define max17042_suspend NULL
#define max17042_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int max17042_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int max17042_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int max17042_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define max17042_runtime_suspend	NULL
#define max17042_runtime_resume		NULL
#define max17042_runtime_idle		NULL
#endif

static const struct i2c_device_id max17042_id[] = {
	{ DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max17042_id);

static const struct dev_pm_ops max17042_pm_ops = {
	.suspend		= max17042_suspend,
	.resume			= max17042_resume,
	.runtime_suspend	= max17042_runtime_suspend,
	.runtime_resume		= max17042_runtime_resume,
	.runtime_idle		= max17042_runtime_idle,
};

static struct i2c_driver max17042_i2c_driver = {
	.driver	= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &max17042_pm_ops,
	},
	.probe		= max17042_probe,
	.remove		= __devexit_p(max17042_remove),
	.id_table	= max17042_id,
};

static int __init max17042_init(void)
{
	return i2c_add_driver(&max17042_i2c_driver);
}
module_init(max17042_init);

static void __exit max17042_exit(void)
{
	i2c_del_driver(&max17042_i2c_driver);
}
module_exit(max17042_exit);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("MAX17042 Fuel Gauge");
MODULE_LICENSE("GPL");
