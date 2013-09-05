/*
 * Summit Microelectronics SMB347 Battery Charger Driver
 *
 * Copyright (C) 2011, Intel Corporation
 *
 * Authors: Bruce E. Robertson <bruce.e.robertson@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/power/smb347-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/extcon.h>

/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS_FUNCS			0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB		BIT(2)
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_DISABLED_OTG_I2C		0x00
#define CFG_OTHER_RID_DISABLED_OTG_PIN		0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C		0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW		BIT(5)
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK	0x03
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_FAULT_IRQ_OTG_UV			BIT(5)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT		BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_STATUS_IRQ_INOK			BIT(2)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_OTG_ENABLED			BIT(4)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_D				0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT		BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ		BIT(3)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
/*
 * In SMB349 the DCIN UV status bits are reversed in the
 * IRQSTAT_E register, the DCIN_UV IRQ & Status correspond
 * to bits 1 & 0 respectively.
 */
#define SMB349_IRQSTAT_E_DCIN_UV_STAT		BIT(0)
#define SMB349_IRQSTAT_E_DCIN_OV_STAT		BIT(3)
#define IRQSTAT_F				0x3a
#define IRQSTAT_F_OTG_UV_IRQ			BIT(5)
#define IRQSTAT_F_OTG_UV_STAT			BIT(4)
#define IRQSTAT_F_PWR_OK_IRQ			BIT(1)
#define IRQSTAT_F_PWR_OK_STAT			BIT(0)

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_HOLDOFF_STAT			BIT(3)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHG_TERM				BIT(5)
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_D					0x3E
#define STAT_E					0x3F
#define SMB349_SUSPENDED			(1<<7)

#define STATUS_UPDATE_INTERVAL			(HZ * 60)	/*60 sec */

struct smb347_otg_event {
	struct list_head	node;
	bool			param;
};

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @running: the driver is up and running
 * @dentry: for debugfs
 * @otg: pointer to OTG transceiver if any
 * @otg_nb: notifier for OTG notifications
 * @otg_work: work struct for OTG notifications
 * @otg_queue: queue holding received OTG notifications
 * @otg_queue_lock: protects concurrent access to @otg_queue
 * @otg_last_param: last parameter value from OTG notification
 * @otg_enabled: OTG VBUS is enabled
 * @otg_battery_uv: OTG battery undervoltage condition is on
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	bool			mains_online;
	bool			usb_online;
	bool			charging_enabled;
	bool			running;
	bool			is_smb349;
	struct dentry		*dentry;
	struct usb_phy		*otg;
	struct notifier_block	otg_nb;
	struct work_struct	otg_work;
	struct list_head	otg_queue;
	spinlock_t		otg_queue_lock;
	bool			otg_enabled;
	bool			otg_battery_uv;
	bool			is_disabled;
	const struct smb347_charger_platform_data	*pdata;
	struct extcon_dev	*edev;
	struct notifier_block	ext_nb;
};

static struct smb347_charger *smb347_dev;
static char *smb347_power_supplied_to[] = {
			"max170xx_battery",
			"max17042_battery",
			"max17047_battery",
			"max17050_battery",
};

static int smb347_read(struct smb347_charger *smb, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to read reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to write reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

static bool smb347_is_suspended(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_read(smb, STAT_E);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "i2c failed %d", ret);
		return false;
	}
	return (ret & SMB349_SUSPENDED) == SMB349_SUSPENDED;
}

/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

static int smb34x_get_health(struct smb347_charger *smb)
{
	bool usb = 0;
	int stat_e = 0;
	int chrg_health;

	if (!smb->is_smb349) {
		chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		goto end;
	}

	stat_e = smb347_read(smb, IRQSTAT_E);
	if (stat_e < 0) {
		dev_warn(&smb->client->dev, "i2c failed %d", stat_e);
		chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		goto end;
	}

	usb = !(stat_e & SMB349_IRQSTAT_E_DCIN_UV_STAT) &&
			!smb->is_disabled;

	if (usb) {
		if (stat_e & SMB349_IRQSTAT_E_DCIN_UV_STAT)
			chrg_health = POWER_SUPPLY_HEALTH_DEAD;
		else if (stat_e & SMB349_IRQSTAT_E_DCIN_OV_STAT)
			chrg_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			chrg_health = POWER_SUPPLY_HEALTH_GOOD;
	} else {
			chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
end:
	return chrg_health;
}

/**
 * smb347_update_status - updates the charging status
 * @smb: pointer to smb347 charger instance
 *
 * Function checks status of the charging and updates internal state
 * accordingly. Returns %0 if there is no change in status, %1 if the
 * status has changed and negative errno in case of failure.
 */
static int smb347_update_status(struct smb347_charger *smb)
{
	bool usb = false;
	bool dc = false;
	int ret;

	ret = smb347_read(smb, IRQSTAT_E);
	if (ret < 0)
		return ret;

	if (smb->is_smb349) {
		if (smb->pdata->use_mains)
			dc = !(ret & SMB349_IRQSTAT_E_DCIN_UV_STAT);
		else if (smb->pdata->use_usb) {
			usb = !(ret & SMB349_IRQSTAT_E_DCIN_UV_STAT) &&
				!smb->is_disabled;
		}

		mutex_lock(&smb->lock);
		ret = smb->mains_online != dc || smb->usb_online != usb;
		smb->mains_online = dc;
		smb->usb_online = usb;
		mutex_unlock(&smb->lock);
		return ret;
	}

	/*
	 * Dc and usb are set depending on whether they are enabled in
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if (smb->pdata->use_mains)
		dc = !(ret & IRQSTAT_E_DCIN_UV_STAT);
	if (smb->pdata->use_usb)
		usb = !(ret & IRQSTAT_E_USBIN_UV_STAT);

	mutex_lock(&smb->lock);
	ret = smb->mains_online != dc || smb->usb_online != usb;
	smb->mains_online = dc;
	smb->usb_online = usb;
	mutex_unlock(&smb->lock);

	return ret;
}

/*
 * smb347_is_online - returns whether input power source is connected
 * @smb: pointer to smb347 charger instance
 *
 * Returns %true if input power source is connected. Note that this is
 * dependent on what platform has configured for usable power sources. For
 * example if USB is disabled, this will return %false even if the USB
 * cable is connected.
 */
static bool smb347_is_online(struct smb347_charger *smb)
{
	bool ret;

	mutex_lock(&smb->lock);
	ret = (smb->usb_online || smb->mains_online) && !smb->otg_enabled;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_charging_status - returns status of charging
 * @smb: pointer to smb347 charger instance
 *
 * Function returns charging status. %0 means no charging is in progress,
 * %1 means pre-charging, %2 fast-charging and %3 taper-charging.
 */
static int smb347_charging_status(struct smb347_charger *smb)
{
	int ret;

	if (!smb347_is_online(smb))
		return 0;

	ret = smb347_read(smb, STAT_C);
	if (ret < 0)
		return 0;

	return (ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
}

static int smb347_charging_set(struct smb347_charger *smb, bool enable)
{
	int ret = 0;

	if (smb->pdata->enable_control != SMB347_CHG_ENABLE_SW) {
		dev_info(&smb->client->dev,
			"charging enable/disable in SW disabled\n");
		return 0;
	}

	mutex_lock(&smb->lock);
	if (smb->charging_enabled != enable) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb->charging_enabled = enable;

		if (enable)
			ret |= CMD_A_CHG_ENABLED;
		else
			ret &= ~CMD_A_CHG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
	}
out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_charging_enable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, true);
}

static inline int smb347_charging_disable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, false);
}

static int smb347_enable_suspend(struct smb347_charger *smb)
{
	int ret;

	if (!smb)
		return -EINVAL;

	mutex_lock(&smb->lock);
	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		goto en_sus_err;

	ret = (ret & ~CMD_A_CHG_ENABLED) | CMD_A_SUSPEND_ENABLED;

	ret = smb347_write(smb, CMD_A, ret);
	if (ret < 0)
		goto en_sus_err;

	mutex_unlock(&smb->lock);

	return 0;

en_sus_err:
	dev_info(&smb->client->dev, "i2c error %d", ret);
	mutex_unlock(&smb->lock);
	return ret;
}

static int smb347_disable_suspend(struct smb347_charger *smb)
{
	int ret;

	if (!smb)
		return -EINVAL;

	mutex_lock(&smb->lock);
	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		goto dis_sus_err;

	ret = (ret & ~CMD_A_SUSPEND_ENABLED) | CMD_A_CHG_ENABLED;

	ret = smb347_write(smb, CMD_A, ret);
	if (ret < 0)
		goto dis_sus_err;

	mutex_unlock(&smb->lock);

	return 0;

dis_sus_err:
	dev_info(&smb->client->dev, "i2c error %d", ret);
	mutex_unlock(&smb->lock);
	return ret;
}

/*
 * smb347_enable_charger:
 *	Exposed to EXTCON for enabling charging
 *	Update the status variable, for device active state
 */
int smb347_enable_charger()
{
	struct smb347_charger *smb = smb347_dev;
	int ret;

	ret = smb347_disable_suspend(smb);
	if (ret < 0)
		return ret;

	mutex_lock(&smb->lock);
	smb->is_disabled = false;
	mutex_unlock(&smb->lock);

	return 0;
}
EXPORT_SYMBOL(smb347_enable_charger);

/*
 * smb347_disable_charger:
 *	Exposed to EXTCON for disabling charging via SDP
 *	Update the status variable, for device active state
 */
int smb347_disable_charger()
{
	struct smb347_charger *smb = smb347_dev;
	int ret;

	ret = smb347_enable_suspend(smb);
	if (ret < 0)
		return ret;

	mutex_lock(&smb->lock);
	smb->is_disabled = true;
	mutex_unlock(&smb->lock);

	return 0;
}
EXPORT_SYMBOL(smb347_disable_charger);

static int smb347_update_online(struct smb347_charger *smb)
{
	int ret;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (smb347_is_online(smb)) {
		ret = smb347_charging_enable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to enable charging\n");
	} else {
		ret = smb347_charging_disable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to disable charging\n");
	}

	return ret;
}

static int smb347_otg_set(struct smb347_charger *smb, bool enable)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	mutex_lock(&smb->lock);

	if (pdata->otg_control == SMB347_OTG_CONTROL_SW) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		if (enable)
			ret |= CMD_A_OTG_ENABLED;
		else
			ret &= ~CMD_A_OTG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto out;
	} else {
		/*
		 * Switch to pin control or auto-OTG depending on how
		 * platform has configured.
		 */
		smb347_set_writable(smb, true);

		ret = smb347_read(smb, CFG_OTHER);
		if (ret < 0) {
			smb347_set_writable(smb, false);
			goto out;
		}

		ret &= ~CFG_OTHER_RID_MASK;

		switch (pdata->otg_control) {
		case SMB347_OTG_CONTROL_SW_PIN:
			if (enable) {
				ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
				ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			} else {
				ret &= ~CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			}
			break;

		case SMB347_OTG_CONTROL_SW_AUTO:
			if (enable)
				ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
			break;

		default:
			dev_err(&smb->client->dev,
				"impossible OTG control configuration: %d\n",
				pdata->otg_control);
			break;
		}

		ret = smb347_write(smb, CFG_OTHER, ret);
		smb347_set_writable(smb, false);
		if (ret < 0)
			goto out;
	}

	smb->otg_enabled = enable;

out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_otg_enable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, true);
}

static inline int smb347_otg_disable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, false);
}

static void smb347_otg_drive_vbus(struct smb347_charger *smb, bool enable)
{
	if (enable == smb->otg_enabled)
		return;

	if (enable) {
		if (smb->otg_battery_uv) {
			dev_dbg(&smb->client->dev,
				"battery low voltage, won't enable OTG VBUS\n");
			return;
		}

		/*
		 * Normal charging must be disabled first before we try to
		 * enable OTG VBUS.
		 */
		smb347_charging_disable(smb);
		smb347_otg_enable(smb);

		if (smb->pdata->use_mains)
			power_supply_changed(&smb->mains);
		if (smb->pdata->use_usb)
			power_supply_changed(&smb->usb);

		dev_dbg(&smb->client->dev, "OTG VBUS on\n");
	} else {
		smb347_otg_disable(smb);
		/*
		 * Only re-enable charging if we have some power supply
		 * connected.
		 */
		if (smb347_is_online(smb)) {
			smb347_charging_enable(smb);
			smb->otg_battery_uv = false;
			if (smb->pdata->use_mains)
				power_supply_changed(&smb->mains);
			if (smb->pdata->use_usb)
				power_supply_changed(&smb->usb);
		}

		dev_dbg(&smb->client->dev, "OTG VBUS off\n");
	}
}

static void smb347_otg_work(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, otg_work);
	struct smb347_otg_event *evt, *tmp;
	unsigned long flags;

	/* Process the whole event list in one go. */
	spin_lock_irqsave(&smb->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&smb->otg_queue_lock, flags);

		/* For now we only support set vbus events */
		smb347_otg_drive_vbus(smb, evt->param);
		kfree(evt);

		spin_lock_irqsave(&smb->otg_queue_lock, flags);
	}
	spin_unlock_irqrestore(&smb->otg_queue_lock, flags);
}

static int smb347_otg_notifier(struct notifier_block *nb, unsigned long event,
			       void *param)
{
	struct smb347_charger *smb =
		container_of(nb, struct smb347_charger, otg_nb);
	struct smb347_otg_event *evt;

	dev_dbg(&smb->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS || !smb->running)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&smb->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->param = *(bool *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&smb->otg_queue_lock);
	list_add_tail(&evt->node, &smb->otg_queue);
	spin_unlock(&smb->otg_queue_lock);

	queue_work(system_nrt_wq, &smb->otg_work);
	return NOTIFY_OK;
}

static int smb347_hw_init(struct smb347_charger *smb)
{
	int ret, loopCount, i;
	int regOffset = 0;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	loopCount = MAXSMB347_CONFIG_DATA_SIZE / 2;

	/*
	 * Program the platform specific configuration values to the device
	 */
	for (i = 0; i < loopCount; i++) {
		smb347_write(smb, smb->pdata->char_config_regs[regOffset],
			smb->pdata->char_config_regs[regOffset+1]);
		regOffset += 2;
		/* check if we reached the end of valid row */
		if ((i < loopCount - 1) &&
			(smb->pdata->char_config_regs[regOffset] == 0x0))
			break;
	}

	switch (smb->pdata->otg_control) {
	case SMB347_OTG_CONTROL_DISABLED:
		break;

	case SMB347_OTG_CONTROL_SW:
	case SMB347_OTG_CONTROL_SW_PIN:
	case SMB347_OTG_CONTROL_SW_AUTO:
		smb->otg = usb_get_transceiver();
		if (smb->otg) {
			INIT_WORK(&smb->otg_work, smb347_otg_work);
			INIT_LIST_HEAD(&smb->otg_queue);
			spin_lock_init(&smb->otg_queue_lock);

			smb->otg_nb.notifier_call = smb347_otg_notifier;
			ret = usb_register_notifier(smb->otg, &smb->otg_nb);
			if (ret < 0) {
				usb_put_transceiver(smb->otg);
				smb->otg = NULL;
				goto fail;
			}

			dev_info(&smb->client->dev,
				"registered to OTG notifications\n");
		}
		break;

	case SMB347_OTG_CONTROL_PIN:
	case SMB347_OTG_CONTROL_AUTO:
		/*
		 * no need to program again as the OTG settings
		 * as they are already configured as part of platform
		 * specific register configuration.
		 */
		break;
	}

	ret = smb347_update_status(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_update_online(smb);

	smb347_set_writable(smb, false);
	return ret;

fail:
	if (smb->otg) {
		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		usb_put_transceiver(smb->otg);
		smb->otg = NULL;
	}
	smb347_set_writable(smb, false);
	return ret;
}

static irqreturn_t smb347_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;

	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_c = smb347_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_d = smb347_read(smb, IRQSTAT_D);
	if (irqstat_d < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_D failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	irqstat_f = smb347_read(smb, IRQSTAT_F);
	if (irqstat_f < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_F failed\n");
		return IRQ_NONE;
	}

	/*
	 * If we get charger error we report the error back to user.
	 * If the error is recovered charging will resume again.
	 */
	if (stat_c & STAT_C_CHARGER_ERROR) {
		dev_err(&smb->client->dev,
			"charging stopped due to charger error\n");
		if (smb->pdata->use_usb && smb->is_smb349) {
			smb->usb_online = 0;
			power_supply_changed(&smb->usb);
		}
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);

		ret = IRQ_HANDLED;
	}

	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 */
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ | IRQSTAT_C_TAPER_IRQ)) {
		if ((irqstat_c & IRQSTAT_C_TERMINATION_STAT) &&
						smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		dev_info(&smb->client->dev,
			"[Charge Terminated] Going to HW Maintenance mode\n");
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got a complete charger timeout int that means the charge
	 * full is not detected with in charge timeout value.
	 */
	if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_IRQ) {
		dev_info(&smb->client->dev,
			"[Charge Timeout]:Total Charge Timeout INT recieved\n");
		if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_STAT)
			dev_info(&smb->client->dev,
				"[Charge Timeout]:charging stopped\n");
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (irqstat_e & (IRQSTAT_E_USBIN_UV_IRQ | IRQSTAT_E_DCIN_UV_IRQ)) {
		if (smb347_update_status(smb) > 0) {
			smb347_update_online(smb);
			/*
			 * In SMB349 chip the charging is not starting
			 * immediately after charger connect. We have to
			 * wait untill POWER OK signal/pin becomes good.
			 * So only send the uevent notification for smb347.
			 * For smb349 uevent notification will be sent upon
			 * Power Ok interrupt.
			 */
			if (smb->pdata->use_mains && !smb->is_smb349)
				power_supply_changed(&smb->mains);
			if (smb->pdata->use_usb && !smb->is_smb349)
				power_supply_changed(&smb->usb);
		}

		if (smb->mains_online || smb->usb_online)
			dev_info(&smb->client->dev, "Charger connected\n");
		else
			dev_info(&smb->client->dev, "Charger disconnected\n");

		ret = IRQ_HANDLED;
	}

	/*
	 * If the battery voltage falls below OTG UVLO the VBUS is
	 * automatically turned off but we must not enable it again unless
	 * UVLO is cleared. It will be cleared when external power supply
	 * is connected and the battery voltage goes over the UVLO
	 * threshold.
	 */
	if (irqstat_f & IRQSTAT_F_OTG_UV_IRQ) {
		smb->otg_battery_uv = !!(irqstat_f & IRQSTAT_F_OTG_UV_STAT);
		dev_info(&smb->client->dev, "Vbatt is below OTG UVLO\n");
		ret = IRQ_HANDLED;
	}

	if (irqstat_f & IRQSTAT_F_PWR_OK_IRQ) {
		dev_info(&smb->client->dev, "PowerOK INTR recieved\n");

		if (smb347_update_status(smb) > 0)
			smb347_update_online(smb);

		if (smb->pdata->use_mains)
			power_supply_changed(&smb->mains);
		if (smb->pdata->use_usb)
			power_supply_changed(&smb->usb);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int smb347_irq_set(struct smb347_charger *smb, bool enable)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Enable/disable interrupts for:
	 *	- under voltage
	 *	- termination current reached
	 *	- charger error
	 */
	if (enable) {
		int val = CFG_FAULT_IRQ_DCIN_UV;

		if (smb->otg)
			val |= CFG_FAULT_IRQ_OTG_UV;

		ret = smb347_write(smb, CFG_FAULT_IRQ, val);
		if (ret < 0)
			goto fail;

		val = CFG_STATUS_IRQ_CHARGE_TIMEOUT |
			CFG_STATUS_IRQ_TERMINATION_OR_TAPER |
			CFG_STATUS_IRQ_INOK;
		ret = smb347_write(smb, CFG_STATUS_IRQ, val);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	} else {
		ret = smb347_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	}

fail:
	smb347_set_writable(smb, false);
	return ret;
}

static inline int smb347_irq_enable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, true);
}

static inline int smb347_irq_disable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, false);
}

static int smb347_irq_init(struct smb347_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING, smb->client->name,
				   smb);
	if (ret < 0) {
		dev_info(&smb->client->dev, "request irq failed");
		goto fail_gpio;
	}

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto fail_irq;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb347_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb347_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb347_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

	smb347_set_writable(smb, false);
	smb->client->irq = irq;
	return 0;

fail_readonly:
	smb347_set_writable(smb, false);
fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb347_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, mains);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->mains_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb->usb_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb->usb_online;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb34x_get_health(smb);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

int smb347_get_charging_status(void)
{
	int ret, status;

	if (!smb347_dev)
		return -EINVAL;

	if (!smb347_is_online(smb347_dev))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return ret;

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return status;
}
EXPORT_SYMBOL_GPL(smb347_get_charging_status);

static enum power_supply_property smb347_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TYPE,
};

static int smb347_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb347_charger *smb =
			container_of(psy, struct smb347_charger, battery);
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	ret = smb347_update_status(smb);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = smb347_get_charging_status();
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!smb347_is_online(smb))
			return -ENODATA;

		/*
		 * We handle trickle and pre-charging the same, and taper
		 * and none the same.
		 */
		switch (smb347_charging_status(smb)) {
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->battery_info.voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->battery_info.voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb347_debugfs_show(struct seq_file *s, void *data)
{
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	ret = smb347_read(smb, CMD_A);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_A, ret);
	ret = smb347_read(smb, CMD_B);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_B, ret);
	ret = smb347_read(smb, CMD_C);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_C, ret);
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

static void smb_handle_extcon_events(struct smb347_charger *smb)
{
	int chrg_type, idx, ret;
	u32 event;

	/* current extcon cable status */
	event = smb->edev->state;
	dev_info(&smb->client->dev, "extcon cur state:%d\n", event);

	if (!event) {
		chrg_type = POWER_SUPPLY_TYPE_USB;
		goto extcon_evt_ret;
	}
	/* get the edev index from extcon event */
	for (idx = 0; idx < smb->edev->max_supported; idx++) {
		if ((event >> idx) & 0x1)
			break;
	}
	/* get extcon cable type */
	ret = extcon_find_cable_type(smb->edev, idx);
	if (ret < 0) {
		dev_warn(&smb->client->dev,
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
		dev_warn(&smb->client->dev, "unknown extcon cable\n");
	}

extcon_evt_ret:
	if (smb->pdata->use_usb)
		smb->usb.type = chrg_type;
}

static int smb_extcon_cb(struct notifier_block *nb,
				unsigned long old_state, void *data)
{
	struct smb347_charger *smb = container_of(nb,
					struct smb347_charger, ext_nb);

	smb_handle_extcon_events(smb);
	if (smb->pdata->use_mains)
		power_supply_changed(&smb->mains);
	if (smb->pdata->use_usb)
		power_supply_changed(&smb->usb);
	return NOTIFY_OK;
}

static int smb_extcon_dev_reg_callback(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct smb347_charger *chip = container_of(nb,
					struct smb347_charger, ext_nb);
	int ret;


	/* TODO: Add extcon name in platform data */
	chip->edev = extcon_get_extcon_dev("fsa9285");
	if (!chip->edev) {
		dev_err(&chip->client->dev, "failed to get extcon device\n");
		return NOTIFY_OK;
	} else {
		extcon_dev_unregister_notify(&chip->ext_nb);
		chip->ext_nb.notifier_call = &smb_extcon_cb;
		ret = extcon_register_notifier(chip->edev, &chip->ext_nb);
		if (ret)
			dev_err(&chip->client->dev,
				"failed to register extcon notifier:%d\n", ret);
	}

	return NOTIFY_OK;
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

	smb->is_smb349 = strcmp(id->name, "smb349") ? 0 : 1;
	dev_info(&smb->client->dev, "%s chip in use.\n",
			smb->is_smb349 ? "smb349" : "smb347");

	ret = smb347_hw_init(smb);
	if (ret < 0)
		return ret;

	if (smb->pdata->use_mains) {
		smb->mains.name = "smb347-mains";
		smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
		smb->mains.get_property = smb347_mains_get_property;
		smb->mains.properties = smb347_mains_properties;
		smb->mains.num_properties = ARRAY_SIZE(smb347_mains_properties);
		smb->mains.supplied_to = smb347_power_supplied_to;
		smb->mains.num_supplicants =
				ARRAY_SIZE(smb347_power_supplied_to);
		ret = power_supply_register(dev, &smb->mains);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->use_usb) {
		smb->usb.name = "smb347-usb";
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
		smb->usb.get_property = smb347_usb_get_property;
		smb->usb.properties = smb347_usb_properties;
		smb->usb.num_properties = ARRAY_SIZE(smb347_usb_properties);
		smb->usb.supplied_to = smb347_power_supplied_to;
		smb->usb.num_supplicants = ARRAY_SIZE(smb347_power_supplied_to);
		ret = power_supply_register(dev, &smb->usb);
		if (ret < 0) {
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	if (smb->pdata->show_battery) {
		smb->battery.name = "smb347-battery";
		smb->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		smb->battery.get_property = smb347_battery_get_property;
		smb->battery.properties = smb347_battery_properties;
		smb->battery.num_properties =
				ARRAY_SIZE(smb347_battery_properties);
		ret = power_supply_register(dev, &smb->battery);
		if (ret < 0) {
			if (smb->pdata->use_usb)
				power_supply_unregister(&smb->usb);
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb347_irq_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}

	smb->running = true;
	smb->dentry = debugfs_create_file("smb347-regs", S_IRUSR, NULL, smb,
					  &smb347_debugfs_fops);


	/* TODO: populate the name in plf. data */
	smb->edev = extcon_get_extcon_dev("fsa9285");

	if (!smb->edev) {
		dev_err(dev, "failed extcon");
		smb->ext_nb.notifier_call = &smb_extcon_dev_reg_callback;
		extcon_dev_register_notify(&smb->ext_nb);
	} else {
		smb->ext_nb.notifier_call = &smb_extcon_cb;
		ret = extcon_register_notifier(smb->edev, &smb->ext_nb);
		if (ret)
			dev_err(dev, "failed to register extcon ntf");
	}

	if (smb->edev)
		smb_handle_extcon_events(smb);

	smb347_dev = smb;

	return 0;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

	if (client->irq) {
		smb347_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}

	if (smb->edev)
		extcon_unregister_notifier(smb->edev, &smb->ext_nb);

	if (smb->otg) {
		struct smb347_otg_event *evt, *tmp;

		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		smb347_otg_disable(smb);
		usb_put_transceiver(smb->otg);

		/* Clear all the queued events. */
		flush_work_sync(&smb->otg_work);
		list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
			list_del(&evt->node);
			kfree(evt);
		}
	}
	if (smb->pdata->show_battery)
		power_supply_unregister(&smb->battery);
	if (smb->pdata->use_usb)
		power_supply_unregister(&smb->usb);
	if (smb->pdata->use_mains)
		power_supply_unregister(&smb->mains);
	return 0;
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0},
	{ "smb349", 1},
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_driver = {
	.driver = {
		.name	= "smb347",
		.owner	= THIS_MODULE,
	},
	.probe		= smb347_probe,
	.remove		= __devexit_p(smb347_remove),
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	printk(KERN_DEBUG "Inside smb347_init\n");
	return i2c_add_driver(&smb347_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
