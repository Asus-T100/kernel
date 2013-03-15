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
#define ULPMC_FG_REG_SOH		0x28 /* State Of Health */
#define ULPMC_FG_REG_CYCT		0x2A /* Cycle count total */
#define ULPMC_FG_REG_SOC		0x2C /* State Of Charge */
#define ULPMC_FG_REG_SOH_V4		0x1C /* State Of Health */
#define ULPMC_FG_REG_CYCT_V4		0x1E /* Cycle count total */
#define ULPMC_FG_REG_SOC_V4		0x20 /* State Of Charge */

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
#define BC_STAT_CHRG_MASK			(3 << 4)
/*
 * Charge Current control register
 * with range from 500 - 2048mA
 */
#define ULPMC_BC_CHRG_CUR_CNTL_REG	0x2F
#define BC_CHRG_CUR_OFFSET		500	/* 500 mA */
#define BC_CHRG_CUR_LSB_TO_CUR		64	/* 64 mA */
#define BC_GET_CHRG_CUR(reg)		((reg>>2)*BC_CHRG_CUR_LSB_TO_CUR\
					+ BC_CHRG_CUR_OFFSET) /* in mA */

/* ULPMC Battery Manager Registers */
#define ULPMC_BM_REG_LOWBATT_THR	0x50 /* Low Battery Threshold */
#define ULPMC_BM_REG_CRITBATT_THR	0x51 /* Critical Battery Threshold */
#define ULPMC_BM_REG_RESVBATT_THR	0x52 /* Reserve Battery Threhsold */
#define ULPMC_BM_REG_CNTL		0x53 /* UMPLC command register */
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

#define STATUS_MON_JIFFIES		(HZ * 60)	/*60 sec */
#define ULPMC_FG_SIGN_INDICATOR		0x8000

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT	3

struct ulpmc_chip_info {
	struct i2c_client	*client;
	struct ulpmc_platform_data *pdata;

	struct power_supply	bat;
	struct power_supply	chrg;

	struct extcon_dev	*edev;
	struct notifier_block	nb;

	struct delayed_work work;
	struct mutex lock;
};

static enum power_supply_property ulpmc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
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
	POWER_SUPPLY_PROP_HEALTH,
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

static void ulpmc_battery_monitor(struct work_struct *work)
{
	struct ulpmc_chip_info *chip =
		container_of(work, struct ulpmc_chip_info, work.work);

	power_supply_changed(&chip->bat);
	schedule_delayed_work(&chip->work, STATUS_MON_JIFFIES);
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

static int ulpmc_battery_status(struct ulpmc_chip_info *chip, int flags)
{
	int status;

	if (flags & FG_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & FG_FLAG_DSC)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	return status;
}
static int ulpmc_get_battery_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct ulpmc_chip_info *chip = container_of(psy,
				struct ulpmc_chip_info, bat);
	mutex_lock(&chip->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_FLAGS);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ulpmc_battery_status(chip, ret);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/*
		 * RVP doesn't support health detection yet.
		 * This feature will be supported on PR1.
		 */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
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
		if (chip->pdata->version == BYTULPMCFGV3)
			ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_SOC);
		else
			ret = ulpmc_read_reg16(chip->client,
							ULPMC_FG_REG_SOC_V4);
		if (ret < 0)
			goto i2c_read_err;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ulpmc_read_reg16(chip->client, ULPMC_FG_REG_TEMP);
		if (ret < 0)
			goto i2c_read_err;
		dev_dbg(&chip->client->dev, "BQ FG Temp:%d\n", ret - 2731);
		/*
		 * RVP doesn't support temperature readings.
		 * This feature will be supported on PR1.
		 */
		val->intval = 350;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
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

static int get_charger_health(struct ulpmc_chip_info *chip)
{
	int ret;

	ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
	if (ret < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (ret & BC_STAT_VBUS_MASK) {
		if (ret & BC_STAT_PWR_GOOD)
			return POWER_SUPPLY_HEALTH_GOOD;
		else
			return POWER_SUPPLY_HEALTH_DEAD;
	}
	return POWER_SUPPLY_HEALTH_UNKNOWN;
}

static int ulpmc_get_charger_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct ulpmc_chip_info *chip = container_of(psy,
				struct ulpmc_chip_info, chrg);

	mutex_lock(&chip->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
		if (ret < 0)
			goto i2c_read_err;
		if ((ret & BC_STAT_VBUS_MASK) == BC_STAT_VBUS_ADP)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = ulpmc_read_reg8(chip->client, ULPMC_BC_REG_STAT);
		if (ret < 0)
			goto i2c_read_err;
		if (ret & BC_STAT_PWR_GOOD)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_charger_health(chip);
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

static void log_interrupt_event(struct ulpmc_chip_info *chip, int intstat)
{
	if (intstat & INTSTAT_INSERT_AC)
		dev_info(&chip->client->dev, "AC charger inserted\n");
	else if (intstat & INTSTAT_REMOVE_AC)
		dev_info(&chip->client->dev, "AC charger removeed\n");
	else if (intstat & INTSTAT_INSERT_USB)
		dev_info(&chip->client->dev, "USB charger inserted\n");
	else if (intstat & INTSTAT_REMOVE_USB)
		dev_info(&chip->client->dev, "USB charger removed\n");
	else if (intstat & INTSTAT_START_CHARGE)
		dev_info(&chip->client->dev, "Charging start event\n");
	else if (intstat & INTSTAT_STOP_CHARGE)
		dev_info(&chip->client->dev, "Charging stop event\n");
	else if (intstat & INTSTAT_INSERT_BATTERY)
		dev_info(&chip->client->dev, "Battery inserted\n");
	else if (intstat & INTSTAT_REMOVE_BATTERY)
		dev_info(&chip->client->dev, "Battery removed\n");
	else if (intstat & INTSTAT_LOW_BATTERY)
		dev_info(&chip->client->dev, "Low Battery warning!\n");
	else if (intstat & INTSTAT_BTP_HIGH)
		dev_info(&chip->client->dev, "Battery Trip point high\n");
	else if (intstat & INTSTAT_BTP_LOW)
		dev_info(&chip->client->dev, "Battery Trip point low\n");
	else if (intstat & INTSTAT_THRM_BAT0)
		dev_info(&chip->client->dev, "Battery0 thermal event\n");
	else if (intstat & INTSTAT_THRM_BAT1)
		dev_info(&chip->client->dev, "Battery1 thermal event\n");
	else if (intstat & INTSTAT_THRM_SKIN0)
		dev_info(&chip->client->dev, "Skin0 thermal event\n");
	else
		dev_info(&chip->client->dev, "spurious event!!!\n");
}

static irqreturn_t ulpmc_intr_handler(int id, void *dev)
{
	struct ulpmc_chip_info *chip = dev;

	dev_info(&chip->client->dev, "ULPMC Interrupt!!\n");
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ulpmc_thread_handler(int id, void *dev)
{
	struct ulpmc_chip_info *chip = dev;
	int ret;

	ret = ulpmc_read_reg8(chip->client, ULPMC_BM_REG_INTSTAT);
	if (ret < 0) {
		dev_err(&chip->client->dev, "ulpmc stat reg read error\n");
		return IRQ_NONE;
	}
	log_interrupt_event(chip, ret);
	power_supply_changed(&chip->bat);
	return IRQ_HANDLED;
}

static void ulpmc_init_irq(struct ulpmc_chip_info *chip)
{
	int ret = 0;

	chip->client->irq = gpio_to_irq(chip->pdata->gpio);
	/* register interrupt */
	ret = request_threaded_irq(chip->client->irq,
					ulpmc_intr_handler,
					ulpmc_thread_handler,
					IRQF_TRIGGER_LOW | IRQF_SHARED,
					"ulpmc-battery", chip);
	if (ret) {
		dev_warn(&chip->client->dev,
			"cannot get IRQ:%d\n", chip->client->irq);
		chip->client->irq = -1;
	} else {
		dev_info(&chip->client->dev, "IRQ No:%d\n", chip->client->irq);
	}
}

static int ulpmc_extcon_callback(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct ulpmc_chip_info *chip = container_of(nb,
					struct ulpmc_chip_info, nb);

	dev_info(&chip->client->dev, "In extcon callback\n");
	power_supply_changed(&chip->bat);
	return NOTIFY_OK;
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


	INIT_DELAYED_WORK(&chip->work, ulpmc_battery_monitor);
	mutex_init(&chip->lock);

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
	chip->chrg.type = POWER_SUPPLY_TYPE_MAINS;
	chip->chrg.properties = ulpmc_charger_properties;
	chip->chrg.num_properties = ARRAY_SIZE(ulpmc_charger_properties);
	chip->chrg.get_property = ulpmc_get_charger_property;
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

	/* get irq and register */
	ulpmc_init_irq(chip);
	/* schedule status monitoring worker */
	schedule_delayed_work(&chip->work, STATUS_MON_JIFFIES);
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

	cancel_delayed_work_sync(&chip->work);
	if (chip->edev)
		extcon_unregister_notifier(chip->edev, &chip->nb);
	power_supply_unregister(&chip->chrg);
	power_supply_unregister(&chip->bat);
	mutex_destroy(&chip->lock);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id ulpmc_id[] = {
	{ "ulpmc", 0 },
};
MODULE_DEVICE_TABLE(i2c, ulpmc_id);

static struct i2c_driver ulpmc_battery_driver = {
	.driver = {
		.name = "ulpmc-battery",
		.owner	= THIS_MODULE,
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
