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
/*TODO remove this below header file once tangier_otg.h is available*/
#include <linux/usb/penwell_otg.h>

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
		/* TODO: ADC sampling of voltage */
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
	return 0;
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
	/*TODO remove this assignment. No default val. Read from ADC */
	chrgr_drv_cxt->chrgr_props_cxt.vbus_vol = 4200;
}

static int get_charging_profile(struct charging_profile *chrg_profile)
{
	/*
	 * TODO: Get battery profile from platfrom layer. Using hardcoded sample
	 * value till this is done.
	 */

	memcpy(chrg_profile->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	chrg_profile->voltage_max = 4200;
	chrg_profile->capacity = 1500;
	chrg_profile->battery_type = 2;	/* POWER_SUPPLY_TECHNOLOGY_LION */
	chrg_profile->temp_mon_ranges = 4;

	chrg_profile->temp_mon_range[0].temp_low_lim = 45;
	chrg_profile->temp_mon_range[0].temp_up_lim = 60;
	chrg_profile->temp_mon_range[0].rbatt = 120;
	chrg_profile->temp_mon_range[0].full_chrg_cur = 950;
	chrg_profile->temp_mon_range[0].full_chrg_vol = 4100;
	chrg_profile->temp_mon_range[0].maint_chrg_cur = 950;
	chrg_profile->temp_mon_range[0].maint_chrg_vol_ll = 4000;
	chrg_profile->temp_mon_range[0].maint_chrg_vol_ul = 4050;

	chrg_profile->temp_mon_range[1].temp_low_lim = 10;
	chrg_profile->temp_mon_range[1].temp_up_lim = 45;
	chrg_profile->temp_mon_range[1].rbatt = 120;
	chrg_profile->temp_mon_range[1].full_chrg_cur = 950;
	chrg_profile->temp_mon_range[1].full_chrg_vol = 4200;
	chrg_profile->temp_mon_range[1].maint_chrg_cur = 950;
	chrg_profile->temp_mon_range[1].maint_chrg_vol_ll = 4100;
	chrg_profile->temp_mon_range[1].maint_chrg_vol_ul = 4150;

	chrg_profile->temp_mon_range[2].temp_low_lim = 0;
	chrg_profile->temp_mon_range[2].temp_up_lim = 10;
	chrg_profile->temp_mon_range[2].rbatt = 120;
	chrg_profile->temp_mon_range[2].full_chrg_cur = 950;
	chrg_profile->temp_mon_range[2].full_chrg_vol = 4100;
	chrg_profile->temp_mon_range[2].maint_chrg_cur = 950;
	chrg_profile->temp_mon_range[2].maint_chrg_vol_ll = 4000;
	chrg_profile->temp_mon_range[2].maint_chrg_vol_ul = 4050;

	chrg_profile->temp_mon_range[3].temp_low_lim = -10;
	chrg_profile->temp_mon_range[3].temp_up_lim = 0;
	chrg_profile->temp_mon_range[3].rbatt = 120;
	chrg_profile->temp_mon_range[3].full_chrg_cur = 950;
	chrg_profile->temp_mon_range[3].full_chrg_vol = 3900;
	chrg_profile->temp_mon_range[3].maint_chrg_cur = 950;
	chrg_profile->temp_mon_range[3].maint_chrg_vol_ll = 3950;
	chrg_profile->temp_mon_range[3].maint_chrg_vol_ul = 3950;

	return 0;

}

static void populate_invalid_chrg_profile(struct charging_profile *chrg_profile)
{

	/*
	 * In case of invalid battery we manually set
	 * the battery parameters and limit the battery from
	 * charging, so platform will be in discharging mode
	 */

	memcpy(chrg_profile->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	chrg_profile->temp_mon_ranges = 0;

}

static int get_batt_config(struct battery_config *batt_config)
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

static void populate_default_battery_config(struct battery_config *batt_config)
{
	batt_config->vbatt_sh_min = BATT_DEAD_CUTOFF_VOLT;
	batt_config->vbatt_crit = BATT_CRIT_CUTOFF_VOLT;
	batt_config->temp_high = MSIC_BATT_TEMP_MAX;
	batt_config->temp_low = MSIC_BATT_TEMP_MIN;
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
	    kzalloc(sizeof(struct charging_profile), GFP_KERNEL);
	if (!chrgr_drv_cxt->chrg_profile) {

		dev_err(&pdev->dev, "%s(): memory allocation failed: Unable to"
			"allocate memory for battery profile\n", __func__);
		retval = -ENOMEM;
		goto chrg_profile_alloc_failed;
	}

	chrgr_drv_cxt->batt_config =
	    kzalloc(sizeof(struct battery_config), GFP_KERNEL);
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

	if (get_charging_profile(chrgr_drv_cxt->chrg_profile)) {
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

	/*TODO: Interrupt tree mapping */
	/*TODO: Register IRQ handler */

	/* initialize mutexes */
	mutex_init(&chrgr_drv_cxt->bc_chrgr_lock);

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
