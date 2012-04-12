/*
 * basin_cove_charger.c - Intel MID Basin Cove PMIC Charger Driver
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/sfi.h>
#include <linux/wakelock.h>
#include <linux/async.h>
#include <linux/reboot.h>
#include <linux/power/intel_mid_powersupply.h>
/*TODO remove this below header file once tangier_otg.h is available*/
#include <linux/usb/penwell_otg.h>
#include <asm/intel_basincove_gpadc.h>
#include "basin_cove_charger.h"

#define CHARGER_PS_NAME "bcove_charger"
#define DRIVER_NAME "bcove_charger"

#define DEVICE_NAME "bc_charger"

static struct device *bc_chrgr_dev;
static void *otg_handle;

/*
 * Basin Cove Charger power supply  properties
 */
static enum power_supply_property bc_chrgr_ps_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/*FIXME: Remove this IPC call stubs */
static int intel_scu_ipc_update_register(u16 addr, u8 bits, u8 mask)
{
	return 0;
}

/*FIXME: Remove this IPC call stub */
static int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	return 0;
}

/*TODO : Implement the function for adc value to temp conversion*/
static int adc_to_temp(uint16_t adc_val, int *tp)
{

	return 0;

}
static int bc_enable_charger(void)
{
	return intel_scu_ipc_update_register(CHGRCTRL0_ADDR, EXTCHRDIS_ENABLE,
					     CHGRCTRL0_EXTCHRDIS_MASK);

}

static int bc_disbale_charger(void)
{
	return intel_scu_ipc_update_register(CHGRCTRL0_ADDR, ~EXTCHRDIS_ENABLE,
					     CHGRCTRL0_EXTCHRDIS_MASK);

}

static int bc_set_chrgr_current_limit(int cur_inlmt)
{
	int inlmt_mask;

	if (cur_inlmt <= 100)
		inlmt_mask = CHGRCTRL1_FUSB_INLMT_100_MASK;
	else if (cur_inlmt <= 150)
		inlmt_mask = CHGRCTRL1_FUSB_INLMT_150_MASK;
	else if (cur_inlmt <= 500)
		inlmt_mask = CHGRCTRL1_FUSB_INLMT_500_MASK;
	else if (cur_inlmt <= 900)
		inlmt_mask = CHGRCTRL1_FUSB_INLMT_900_MASK;
	else if (cur_inlmt <= 1500)
		inlmt_mask = CHGRCTRL1_FUSB_INLMT_1500_MASK;
	else
		return -EINVAL;
	return intel_scu_ipc_update_register(CHGRCTRL1_ADDR, 0xFF, inlmt_mask);
}

/**
 * mrfl_read_adc_regs - read ADC value of specified sensors
 * @channel: channel of the sensor to be sampled
 * @sensor_val: pointer to the charger property to hold sampled value
 * @chrgr_drv_cxt :  battery info pointer
 *
 * Returns 0 if success
 */

static int mrfl_read_adc_regs(int channel, int *sensor_val,
	struct bc_chrgr_drv_context *chrgr_drv_cxt)
{
	int ret, adc_val;
	struct gpadc_result *adc_res;
	adc_res = kzalloc(sizeof(struct gpadc_result), GFP_KERNEL);
	if (!adc_res)
		return -ENOMEM;
	ret = intel_basincove_gpadc_sample(channel, adc_res);
	if (ret) {
		dev_err(&chrgr_drv_cxt->pdev->dev,
			"gpadc_sample failed:%d\n", ret);
		goto exit;
	}

	adc_val = GPADC_RSL(channel, adc_res);
	switch (channel) {
	case GPADC_BATTEMP0:
		ret = adc_to_temp(adc_val, sensor_val);
		break;
	default:
		dev_err(&chrgr_drv_cxt->pdev->dev,
			"invalid sensor%d", channel);
		ret = -EINVAL;
	}
exit:
	kfree(adc_res);
	return ret;
}

/**
 * bc_chrgr_get_property - charger power supply get property
 * @psy: charger power supply context
 * @psp: charger property
 * @val: charger property value
 * Context: can sleep
 *
 * Basin Cove power supply property needs to be provided to power_supply
 * subsystem for it to provide the information to users.
 */
static int bc_chrgr_ps_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct bc_chrgr_drv_context *chrgr_drv_cxt =
	    container_of(psy, struct bc_chrgr_drv_context, bc_chrgr_ps);
	int retval = 0;
	mutex_lock(&chrgr_drv_cxt->bc_chrgr_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chrgr_drv_cxt->chrgr_props_cxt.charger_present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (chrgr_drv_cxt->chrgr_props_cxt.charging_mode !=
		    POWER_SUPPLY_STATUS_NOT_CHARGING)
			val->intval =
			    chrgr_drv_cxt->chrgr_props_cxt.charger_present;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chrgr_drv_cxt->chrgr_props_cxt.charger_health;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/*from hardcoded value since no adc channel for vbus_volt*/
		val->intval = chrgr_drv_cxt->chrgr_props_cxt.vbus_vol * 1000;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chrgr_drv_cxt->chrgr_props_cxt.charger_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = chrgr_drv_cxt->chrgr_props_cxt.charger_vender;
		break;
	default:
		mutex_unlock(&chrgr_drv_cxt->bc_chrgr_lock);
		return -EINVAL;
	}

	mutex_unlock(&chrgr_drv_cxt->bc_chrgr_lock);
	return retval;
}



/**
 * init_batt_props_context - initialize battery properties
 * @chrgr_drv_cxt: basin cove charger driver structure
 * init_batt_props function initializes the
 * battery properties.
 */
static void init_batt_props_context(struct bc_chrgr_drv_context *chrgr_drv_cxt)
{
	unsigned char data;
	int retval;

	chrgr_drv_cxt->batt_props_cxt.status = POWER_SUPPLY_STATUS_DISCHARGING;
	chrgr_drv_cxt->batt_props_cxt.health = POWER_SUPPLY_HEALTH_GOOD;
	chrgr_drv_cxt->batt_props_cxt.present = BATT_NOT_PRESENT;
	/*TODO: remove the default assignment: Assumed as battery present*/
	chrgr_drv_cxt->current_sense_enabled = 1;

	/*read specific to determine the status*/
	retval = intel_scu_ipc_ioread8(SCHGRIRQ1_ADDR, &data);
	if (retval)
		dev_crit(bc_chrgr_dev, "PMIC IPC read access to SCHGRIRQ1_ADDR failed\n");
	/* determine battery Presence */
	else if (data & BATT_CHR_BATTDET_MASK)
		chrgr_drv_cxt->batt_props_cxt.present = BATT_PRESENT;
}

static void init_charger_ps_context(struct bc_chrgr_drv_context *chrgr_drv_cxt)
{
	chrgr_drv_cxt->bc_chrgr_ps.name = CHARGER_PS_NAME;
	chrgr_drv_cxt->bc_chrgr_ps.type = POWER_SUPPLY_TYPE_USB;
	chrgr_drv_cxt->bc_chrgr_ps.properties = bc_chrgr_ps_props;
	chrgr_drv_cxt->bc_chrgr_ps.num_properties =
	    ARRAY_SIZE(bc_chrgr_ps_props);
	chrgr_drv_cxt->bc_chrgr_ps.get_property = bc_chrgr_ps_get_property;

	chrgr_drv_cxt->chrgr_props_cxt.charger_present = CHARGER_NOT_PRESENT;
	chrgr_drv_cxt->chrgr_props_cxt.charger_health =
	    POWER_SUPPLY_HEALTH_UNKNOWN;
	memcpy(chrgr_drv_cxt->chrgr_props_cxt.charger_model, "Unknown",
	       sizeof("Unknown"));
	memcpy(chrgr_drv_cxt->chrgr_props_cxt.charger_vender, "Unknown",
	       sizeof("Unknown"));
	/*Vbus_volt hardcoded.Not reading from ADC since no channel available*/
	chrgr_drv_cxt->chrgr_props_cxt.vbus_vol = 4200;
}


static void populate_invalid_chrg_profile
	(struct batt_charging_profile *chrg_profile)
{

	/*
	 * In case of invalid battery we manually set
	 * the battery parameters and limit the battery from
	 * charging, so platform will be in discharging mode
	 */

	memcpy(chrg_profile->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	chrg_profile->temp_mon_ranges = 0;

}

static void populate_default_battery_config(
		struct plat_battery_config *batt_config)
{

	/*TODO: Get battery configuration from platfrom layer.Using default
	   hardcoded values till this is done */
	batt_config->vbatt_sh_min = BATT_DEAD_CUTOFF_VOLT;
	batt_config->vbatt_crit = BATT_CRIT_CUTOFF_VOLT;
	batt_config->temp_high = MSIC_BATT_TEMP_MAX;
	batt_config->temp_low = MSIC_BATT_TEMP_MIN;
	batt_config->itc = 50;

	return 0;
}


/*TO BE FIXED : event handler stub*/
static int pmic_bc_event_handler(void *arg, int event, struct otg_bc_cap *cap)
{
	return 0;
}
/*TO BE REMOVED : chargin cap query - OTG function stub*/
static int tangier_otg_query_charging_cap(struct otg_bc_cap *cap)
{
	return 0;
}
/*TO BE REMOVED : OTG callback register/unregister function stub*/
static void *tangier_otg_register_bc_callback(
int (*cb)(void *, int, struct otg_bc_cap *), void *arg)
{
	struct penwell_otg *temp_ptr = kzalloc(sizeof(struct penwell_otg),
					GFP_KERNEL);
	if (!temp_ptr)
		dev_err(bc_chrgr_dev, "%s(): memory allocation failed\n",
		__func__);
	return temp_ptr;
}

static int tangier_otg_unregister_bc_callback(void *handle)
{
	kfree(handle);
	return 0;
}

static void bc_chrg_callback_worker(struct work_struct *work)
{
	int retval;
	struct otg_bc_cap cap;
	struct bc_chrgr_drv_context *chrgr_drv_cxt =
		container_of(work, struct bc_chrgr_drv_context,
		chrg_callback_dwrk.work);
	retval = tangier_otg_query_charging_cap(&cap);
	if (retval)
		dev_err(bc_chrgr_dev, "%s(): Failed to get otg capabalities\n",
		__func__);
	else {
		retval = pmic_bc_event_handler(chrgr_drv_cxt,
						cap.current_event, &cap);
		if (retval)
			dev_err(bc_chrgr_dev, "%s(): Event handler failed\n",
			__func__);
	}
}

/* bc_charger_callback - callback for USB OTG*/
static int bc_charger_callback(void *arg, int event,
					struct otg_bc_cap *cap)
{
	struct bc_chrgr_drv_context *chrgr_drv_cxt =
		(struct bc_chrgr_drv_context *)arg;
	schedule_delayed_work(&chrgr_drv_cxt->chrg_callback_dwrk, 0);
	return 0;
}

/* Exported Functions to use with Fuel Gauge driver */

bool bc_is_current_sense_enabled(void)
{
	struct platform_device *pdev = container_of(bc_chrgr_dev,
					struct platform_device, dev);
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);
	bool val;
	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	mutex_lock(&chrgr_drv_cxt->batt_lock);
	val = chrgr_drv_cxt->current_sense_enabled;
	mutex_unlock(&chrgr_drv_cxt->batt_lock);

	return val;
}
EXPORT_SYMBOL(bc_is_current_sense_enabled);

bool bc_check_battery_present(void)
{
	struct platform_device *pdev = container_of(bc_chrgr_dev,
					struct platform_device, dev);
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);
	bool val;
	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	mutex_lock(&chrgr_drv_cxt->batt_lock);
	val = chrgr_drv_cxt->batt_props_cxt.present;
	mutex_unlock(&chrgr_drv_cxt->batt_lock);

	return val;
}
EXPORT_SYMBOL(bc_check_battery_present);

int bc_check_battery_health(void)
{
	struct platform_device *pdev = container_of(bc_chrgr_dev,
					struct platform_device, dev);
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);
	unsigned int val;
	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	mutex_lock(&chrgr_drv_cxt->batt_lock);
	val = chrgr_drv_cxt->batt_props_cxt.health;
	mutex_unlock(&chrgr_drv_cxt->batt_lock);

	return val;
}
EXPORT_SYMBOL(bc_check_battery_health);

int bc_check_battery_status(void)
{
	struct platform_device *pdev = container_of(bc_chrgr_dev,
					struct platform_device, dev);
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);
	unsigned int val;
	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	mutex_lock(&chrgr_drv_cxt->batt_lock);
	val = chrgr_drv_cxt->batt_props_cxt.status;
	mutex_unlock(&chrgr_drv_cxt->batt_lock);

	return val;
}
EXPORT_SYMBOL(bc_check_battery_status);


int bc_get_battery_pack_temp(int *temp)
{
	struct platform_device *pdev = container_of(bc_chrgr_dev,
					struct platform_device, dev);
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);

	if (!chrgr_drv_cxt->current_sense_enabled)
		return -ENODEV;
	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	return mrfl_read_adc_regs(GPADC_BATTEMP0, temp, chrgr_drv_cxt);
}
EXPORT_SYMBOL(bc_get_battery_pack_temp);


/**
 * bc_charger_probe - basin cove charger probe function
 * @pdev: basin cove platform device structure
 * Context: can sleep
 *
 * basin cove charger driver initializes its internal data
 * structure and other  infrastructure components for it
 * to work as expected.
 */
static int bc_chrgr_probe(struct platform_device *pdev)
{
	struct bc_chrgr_drv_context *chrgr_drv_cxt = NULL;
	int retval = 0;

	chrgr_drv_cxt =
	    kzalloc(sizeof(struct bc_chrgr_drv_context), GFP_KERNEL);
	if (!chrgr_drv_cxt) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		return -ENOMEM;
	}
	chrgr_drv_cxt->chrg_profile =
	    kzalloc(sizeof(struct batt_charging_profile), GFP_KERNEL);
	if (!chrgr_drv_cxt->chrg_profile) {

		dev_err(&pdev->dev, "%s(): memory allocation failed: Unable to"
			"allocate memory for battery profile\n", __func__);
		retval = -ENOMEM;
		goto chrg_profile_alloc_failed;
	}

	chrgr_drv_cxt->batt_config =
	    kzalloc(sizeof(struct plat_battery_config), GFP_KERNEL);
	if (!chrgr_drv_cxt->batt_config) {

		dev_err(&pdev->dev, "%s(): memory allocation failed: Unable to"
			"allocate memory for battery configuration\n",
			__func__);
		retval = -ENOMEM;
		goto batt_config_alloc_failed;
	}

	chrgr_drv_cxt->pdev = pdev;
	chrgr_drv_cxt->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, chrgr_drv_cxt);
	bc_chrgr_dev = &pdev->dev;

	/* Initialize work for otg callback worker*/
	INIT_DELAYED_WORK(&chrgr_drv_cxt->chrg_callback_dwrk,
				bc_chrg_callback_worker);

	if (get_batt_charging_profile(chrgr_drv_cxt->chrg_profile)) {
		dev_err(&pdev->dev, "%s() :Failed to get battery properties\n",
			__func__);
		populate_invalid_chrg_profile(chrgr_drv_cxt->chrg_profile);
		chrgr_drv_cxt->invalid_batt = true;
	}

	if (get_batt_config(chrgr_drv_cxt->batt_config)) {
		dev_err(&pdev->dev, "%s() :Failed to get battery settings\n",
			__func__);
		populate_default_battery_config(chrgr_drv_cxt->batt_config);
	}

	init_charger_ps_context(chrgr_drv_cxt);
	init_batt_props_context(chrgr_drv_cxt);

	/*TODO: Interrupt tree mapping */
	/*TODO: Register IRQ handler */

	/* initialize mutexes */
	mutex_init(&chrgr_drv_cxt->bc_chrgr_lock);
	mutex_init(&chrgr_drv_cxt->batt_lock);

	retval = power_supply_register(&pdev->dev, &chrgr_drv_cxt->bc_chrgr_ps);
	if (retval) {
		dev_err(&pdev->dev,
			"%s(): failed to register basin cove charger"
			"device with power supply subsystem\n", __func__);
		goto ps_reg_failed;
	}

	/*register OTG callback handle-	callback register function not
	implemented yet so a stub with temporary return type as
	integer is provided above*/
	otg_handle = tangier_otg_register_bc_callback(bc_charger_callback,
					(void *)chrgr_drv_cxt);

	if (!otg_handle) {
		dev_err(&pdev->dev, "battery: OTG Registration failed\n");
		retval = PTR_ERR(otg_handle);
		goto otg_reg_failed;
	}
	/* Enable SWCONTROL bit to handle USB events from SW */
	retval = intel_scu_ipc_update_register(CHGRCTRL0_ADDR, SWCONTROL_ENABLE,
					       CHGRCTRL0_SWCONTROL_MASK);
	if (retval) {

		dev_err(&pdev->dev, "%s(): failed to set SWCONTROL bit"
			"Continuing in HW charging mode\n", __func__);
	}
	return retval;
otg_reg_failed:
	power_supply_unregister(&chrgr_drv_cxt->bc_chrgr_ps);
ps_reg_failed:
	kfree(chrgr_drv_cxt->batt_config);
batt_config_alloc_failed:
	kfree(chrgr_drv_cxt->chrg_profile);
chrg_profile_alloc_failed:
	kfree(chrgr_drv_cxt);

	return retval;

}

static void bc_chrgr_do_exit_ops(struct bc_chrgr_drv_context *chrgr_drv_cxt)
{
	/*TODO:
	 * If charger is connected send IPC message to SCU to continue charging
	 */

}

/**
 * bc_charger_remove - basin cove charger finalize
 * @pdev: basin cove charger platform  device structure
 * Context: can sleep
 *
 * Basin cove charger finalizes its internal data structure and other
 * infrastructure components that it initialized in
 * bc_chrgr_probe.
 */
static int bc_chrgr_remove(struct platform_device *pdev)
{
	struct bc_chrgr_drv_context *chrgr_drv_cxt = platform_get_drvdata(pdev);

	if (chrgr_drv_cxt) {
		bc_chrgr_do_exit_ops(chrgr_drv_cxt);
		tangier_otg_unregister_bc_callback(otg_handle);
		power_supply_unregister(&chrgr_drv_cxt->bc_chrgr_ps);

		kfree(chrgr_drv_cxt->chrg_profile);
		kfree(chrgr_drv_cxt->batt_config);
		mutex_destroy(&chrgr_drv_cxt->bc_chrgr_lock);
		kfree(chrgr_drv_cxt);
	}

	return 0;
}

#ifdef CONFIG_PM
static int bc_chrgr_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bc_chrgr_suspend    NULL
#define bc_chgr_resume     NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bc_chrgr_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bc_chrgr_runtime_suspend	NULL
#define bc_chrgr_runtime_resume		NULL
#define bc_chrgr_runtime_idle		NULL
#endif
/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct platform_device_id bc_chrgr_id_table[] = {
	{DEVICE_NAME, 1},
};

static const struct dev_pm_ops bc_chrgr_pm_ops = {
	.suspend = bc_chrgr_suspend,
	.resume = bc_chrgr_resume,
	.runtime_suspend = bc_chrgr_runtime_suspend,
	.runtime_resume = bc_chrgr_runtime_resume,
	.runtime_idle = bc_chrgr_runtime_idle,
};

static struct platform_driver bc_chrgr_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &bc_chrgr_pm_ops,
		   },
	.probe = bc_chrgr_probe,
	.remove = __devexit_p(bc_chrgr_remove),
	.id_table = bc_chrgr_id_table,
};

static int __init bc_chrgr_init(void)
{
	int ret;
	struct platform_device *pdev;

	/*FIXME: move platfrom device registartion to platfrom layer
	   once bc_charger device is populated in platform device list */

	pdev = platform_device_alloc(DEVICE_NAME, 0);
	if (!pdev)
		return PTR_ERR(pdev);
	ret = platform_device_add(pdev);
	if (ret) {
		kfree(pdev);
		return ret;
	}

	ret = platform_driver_register(&bc_chrgr_driver);

	return ret;
}

static void __exit bc_chrgr_exit(void)
{

	platform_driver_unregister(&bc_chrgr_driver);
}
/* Defer init call so that dependant drivers will be loaded. Using  async
 * for parallel driver initialization */
late_initcall_async(bc_chrgr_init);
module_exit(bc_chrgr_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("Basin Cove Charger  Driver");
MODULE_LICENSE("GPL");
