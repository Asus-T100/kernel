/*
 * charger_helper.c - Charger helper to abstract common charger operations
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
#include <linux/param.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/async.h>
#include <linux/reboot.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/power/charger_helper.h>

/*FIXME: Replace the header file with generic file*/
#include <linux/power/intel_mid_powersupply.h>

#define __flush_health_list(health_list) \
	do {\
		struct list_head *pos, *node;\
		struct health_node *tmp;\
		list_for_each_safe(pos, node, health_list) { \
			tmp = list_entry(pos, struct health_node, hlth_lst); \
			list_del(pos);\
			kfree(tmp);\
		} \
	} while (0)

#define is_exception(health) \
	(health != POWER_SUPPLY_HEALTH_UNKNOWN && \
	health != POWER_SUPPLY_HEALTH_GOOD)

struct health_node {
	struct list_head hlth_lst;
	int health_status;
};

struct charger_context {
	struct list_head chrgr_hlth_lst;
	struct list_head batt_hlth_lst;
	struct power_supply_charger_cap cap;
	uint8_t charger_type;
	uint8_t usb_charger_type;
	uint8_t charger_mode;
	bool charger_present;
	bool charger_enabled;
	char charger_model[BATT_STRING_MAX];
	char charger_vendor[BATT_STRING_MAX];
};

struct ch_context {

	struct charger_helper_charger *charger;

	struct delayed_work usb_chrgr_work;
	struct delayed_work excep_work;
	struct delayed_work chrg_work;

	struct charger_context chrgr_cxt;
	struct batt_charging_profile batt_chrg_profile;

	bool exep_work_running;
	bool chrgr_work_running;

	struct mutex chrgr_cxt_lock;
	struct mutex excep_lock;

	int usr_chrg_mA;

};

static int stop_charging(struct ch_context *chc);

static bool is_vsys_supported(struct ch_context *chc)
{

	bool retval;

	mutex_lock(&chc->chrgr_cxt_lock);
	retval =
	    (chc->chrgr_cxt.charger_type ==
	     CHARGER_USB) ? (chc->charger->flags & USB_VSYS_SUPPORTED) :
	    (chc->charger->flags & AC_VSYS_SUPPORTED);
	mutex_unlock(&chc->chrgr_cxt_lock);

	return retval;
}

static int get_health(struct ch_context *chc, int health_src)
{
	struct list_head *health_list;
	struct health_node *tmp;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;

	health_list =
	    (health_src ==
	     CH_SOURCE_TYPE_BATTERY) ? &chc->chrgr_cxt.
	    batt_hlth_lst : &chc->chrgr_cxt.chrgr_hlth_lst;

	/*return last reported health status */
	mutex_lock(&chc->excep_lock);
	if (!list_empty(health_list)) {
		tmp =
		    list_first_entry(health_list, struct health_node, hlth_lst);
		health = tmp->health_status;
	}
	mutex_unlock(&chc->excep_lock);

	return health;
}

static bool get_charger_online_property(struct ch_context *chc)
{
	bool charger_present;

	mutex_lock(&chc->chrgr_cxt_lock);
	charger_present = chc->chrgr_cxt.charger_present;
	mutex_unlock(&chc->chrgr_cxt_lock);

	/*return false if charger health is UNKNOWN */

	if (get_health(chc, CH_SOURCE_TYPE_CHARGER) ==
	    POWER_SUPPLY_HEALTH_UNKNOWN)
		return false;

	/* return true if vsys supported or battery is charging */

	return is_vsys_supported(chc) || chc->charger->is_battery_charging();
}

static int get_battery_status(struct ch_context *chc)
{
	bool charger_present, charger_enabled;
	int charger_mode;

	mutex_lock(&chc->chrgr_cxt_lock);
	charger_present = chc->chrgr_cxt.charger_present;
	charger_enabled = chc->chrgr_cxt.charger_enabled;
	charger_mode = chc->chrgr_cxt.charger_mode;
	mutex_unlock(&chc->chrgr_cxt_lock);

	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/* if charger is enabled but battery is not charging
	 * report NOT CHARGING.
	 */
	if (charger_enabled && !chc->charger->is_battery_charging())
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	/* Report full in Maintenance mode. */
	if (charger_mode == CHARGER_MODE_MAINTENANCE)
		return POWER_SUPPLY_STATUS_FULL;

	if (!charger_enabled)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	/*report CHARGING in all other cases. */
	return POWER_SUPPLY_STATUS_CHARGING;

}

static int __get_charger_type(struct ch_context *chc)
{
	int usb_charger_type;

	mutex_lock(&chc->chrgr_cxt_lock);
	usb_charger_type = chc->chrgr_cxt.usb_charger_type;
	mutex_unlock(&chc->chrgr_cxt_lock);

	if (chc->chrgr_cxt.charger_type == CHARGER_USB)
		return usb_charger_type;
	else
		return POWER_SUPPLY_TYPE_MAINS;
}

int charger_helper_get_property(void *handle, enum ch_source_type type,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct ch_context *chc = (struct ch_context *)handle;
	int retval = 0, charger_present, usb_charger_type;

	mutex_lock(&chc->chrgr_cxt_lock);

	charger_present = chc->chrgr_cxt.charger_present;
	usb_charger_type = chc->chrgr_cxt.usb_charger_type;

	mutex_unlock(&chc->chrgr_cxt_lock);

	if (type == CH_SOURCE_TYPE_CHARGER) {
		switch (psp) {

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = charger_present;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = (int)get_charger_online_property(chc);
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = get_health(chc, CH_SOURCE_TYPE_CHARGER);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			if (chc->charger->get_charger_voltage)
				val->intval =
				    chc->charger->get_charger_voltage() * 1000;
			else
				retval = -EINVAL;
			break;
		case POWER_SUPPLY_PROP_MODEL_NAME:
			val->strval = chc->chrgr_cxt.charger_model;
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			val->strval = chc->chrgr_cxt.charger_vendor;
			break;
		default:
			retval = -EINVAL;
		}
	} else if (type == CH_SOURCE_TYPE_BATTERY) {
		switch (psp) {

		case POWER_SUPPLY_PROP_STATUS:
			val->intval = get_battery_status(chc);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = get_health(chc, CH_SOURCE_TYPE_BATTERY);
			break;
		default:
			retval = -EINVAL;
		}
	}
	return retval;

}
EXPORT_SYMBOL(charger_helper_get_property);

static bool is_exception_pending(struct ch_context *chc)
{

	int batt_health = get_health(chc, CH_SOURCE_TYPE_BATTERY);
	int charger_health = get_health(chc, CH_SOURCE_TYPE_CHARGER);

	return is_exception(batt_health) || is_exception(charger_health);

}

static void __recover_exception(int reported_exception,
				enum ch_source_type exception_src,
				struct ch_context *chc)
{

	struct list_head *pos, *node, *health_list;
	struct health_node *tmp;

	health_list =
	    (exception_src ==
	     CH_SOURCE_TYPE_BATTERY) ? &chc->chrgr_cxt.
	    batt_hlth_lst : &chc->chrgr_cxt.chrgr_hlth_lst;

	mutex_lock(&chc->excep_lock);
	/*remove reported health from health list */
	list_for_each_safe(pos, node, health_list) {
		tmp = list_entry(pos, struct health_node, hlth_lst);
		if (tmp->health_status == reported_exception) {
			list_del(pos);
			kfree(tmp);
		}
	}
	mutex_unlock(&chc->excep_lock);
}

static void __report_health(int health, enum ch_source_type health_src,
			    struct ch_context *chc)
{

	struct list_head *health_list;
	struct health_node *tmp;

	/* if health is not an exception,no more exception is
	 *  pending. So clear health list
	 */

	health_list =
	    (health_src ==
	     CH_SOURCE_TYPE_BATTERY) ? &chc->chrgr_cxt.
	    batt_hlth_lst : &chc->chrgr_cxt.chrgr_hlth_lst;

	mutex_lock(&chc->excep_lock);

	if (!is_exception(health))
		__flush_health_list(health_list);

	/*Add reported health to health list */
	tmp = kzalloc(sizeof(struct health_node), GFP_KERNEL);
	if (!tmp) {
		pr_err("%s:%d Error in allocating memory\n",
		       __FILE__, __LINE__);
		mutex_unlock(&chc->excep_lock);
		return;
	}
	tmp->health_status = health;
	list_add(&tmp->hlth_lst, health_list);

	mutex_unlock(&chc->excep_lock);
}

void charger_helper_report_exception(void *handle,
				     enum ch_source_type exception_src,
				     int exception, bool is_recover)
{

	struct ch_context *chc = (struct ch_context *)handle;

	if (is_recover)
		__recover_exception(exception, exception_src, chc);
	else
		__report_health(exception, exception_src, chc);

	/*schedule exception thread if POLL_EXCP_RECVR flag is set */
	if (chc->charger->flags & POLL_EXCP_RECVR) {
		if (is_exception_pending(chc))
			schedule_delayed_work(&chc->excep_work, 0);
		else
			cancel_delayed_work_sync(&chc->excep_work);
	}

	chc->charger->charger_callback(__get_charger_type(chc));
}
EXPORT_SYMBOL(charger_helper_report_exception);

static bool poll_exception_recover(int type, struct list_head *health_list,
				   struct ch_context *chc)
{

	struct health_node *tmp;
	struct list_head *pos, *node;
	bool is_recvd = false;

	mutex_lock(&chc->excep_lock);
	/* check exception is recovered or not. If recovered
	 *  remove from health list
	 */
	list_for_each_safe(pos, node, health_list) {
		tmp = list_entry(pos, struct health_node, hlth_lst);
		if (!is_exception(tmp->health_status))
			continue;
		if (chc->
		    charger->is_exception_recovered(type, tmp->health_status)) {
			list_del(pos);
			kfree(tmp);
			is_recvd = true;
		}
	}
	mutex_unlock(&chc->excep_lock);
	return is_recvd;

}

static void recover_exception_worker(struct work_struct *work)
{
	struct ch_context *chc = container_of(work, struct ch_context,
					      excep_work.work);
	bool is_recvd = false;
	pr_debug("%s:%s\n", __FILE__, __func__);

	/*recover exception if exception is pending */
	if (is_exception_pending(chc)) {
		is_recvd =
		    poll_exception_recover(CH_SOURCE_TYPE_BATTERY,
					   &chc->chrgr_cxt.batt_hlth_lst, chc);
		is_recvd |=
		    poll_exception_recover(CH_SOURCE_TYPE_CHARGER,
					   &chc->chrgr_cxt.chrgr_hlth_lst, chc);
	}

	/*Notify charger driver if any of the  exception is recovered */
	if (is_recvd)
		chc->charger->charger_callback(__get_charger_type(chc));

	/* schedule  exception monitoring only if exception is pending */
	if (is_exception_pending(chc))
		schedule_delayed_work(&chc->excep_work,
				      chc->charger->excp_recv_delay * HZ);
	else
		pr_debug("%s:%s:No pending exceptions.Exception recover"
			 "thread stopped\n", __FILE__, __func__);

}

static int get_tempzone(struct batt_charging_profile batt_chrg_profile,
			int temp)
{

	int i = 0;
	int temp_range_cnt = batt_chrg_profile.temp_mon_ranges;

	if (temp < batt_chrg_profile.temp_low_lim
	    || temp >
	    batt_chrg_profile.temp_mon_range[temp_range_cnt - 1].temp_up_lim)
		return -EINVAL;
	for (i = 0; i < temp_range_cnt; ++i)
		if (temp <= batt_chrg_profile.temp_mon_range[i].temp_up_lim)
			break;
	return i;
}

static int get_chrg_mA(int tempmon_mA, int chrgr_mA, int usr_chrg_mA,
		       int chrgr_flags)
{
	int chrg_mA;
	chrg_mA = min(tempmon_mA, chrgr_mA);
	if (chrgr_flags & USR_CHRG_CTRL)
		chrg_mA = min(chrg_mA, usr_chrg_mA);
	return chrg_mA;
}

static int do_charging(struct ch_context *chc)
{

	int charger_mode, charger_type, charger_cur;
	int temp, tzone, chrg_mA, ret = 0, tempmon_mA, chrg_mV;

	mutex_lock(&chc->chrgr_cxt_lock);

	charger_mode = chc->chrgr_cxt.charger_mode;
	charger_type = chc->chrgr_cxt.charger_type;
	charger_cur = chc->chrgr_cxt.cap.mA;

	mutex_unlock(&chc->chrgr_cxt_lock);

	if (chc->charger->flags & POLL_TEMP_MON) {
		/* Set charge current and charger voltage
		 * for current temperature range
		 */

		/*FIXME: update charger parameters only
		 * if tempzone is changed
		 */
		temp = chc->charger->get_battery_temperature();
		tzone = get_tempzone(chc->batt_chrg_profile, temp);
		if (tzone == -EINVAL) {
			stop_charging(chc);
			return tzone;
		}
		tempmon_mA =
		    chc->batt_chrg_profile.temp_mon_range[tzone].full_chrg_cur;
		chrg_mA =
		    get_chrg_mA(tempmon_mA, charger_cur, chc->usr_chrg_mA,
				chc->charger->flags);

		if (charger_mode == CHARGER_MODE_MAINTENANCE)
			chrg_mV =
			    chc->batt_chrg_profile.temp_mon_range[tzone].
			    maint_chrg_vol_ul;
		else
			chrg_mV =
			    chc->batt_chrg_profile.temp_mon_range[tzone].
			    full_chrg_vol;

		chc->charger->set_ccmA_cvmV(chrg_mA, chrg_mV);
		pr_debug
		    ("%s:%s POLL_TEMP_MON: Charge params updated: Tempzone=%d"
		     "current=%dmA voltage=%dmV\n", __FILE__, __func__, tzone,
		     chrg_mA, chrg_mV);

	}

	mutex_lock(&chc->chrgr_cxt_lock);
	/*enable charger if not enabled already */
	if (!chc->chrgr_cxt.charger_enabled) {
		ret =
		    chc->charger->enable_charging(chc->chrgr_cxt.charger_type);
		if (!ret) {
			chc->chrgr_cxt.charger_enabled = true;
			pr_debug("%s:%s Charger enabled\n", __FILE__, __func__);
		} else
			pr_err("%s:%s Error in enabling charger\n", __FILE__,
			       __func__);

	}

	mutex_unlock(&chc->chrgr_cxt_lock);

	return ret;

}

static int stop_charging(struct ch_context *chc)
{
	int charger_type;
	int ret;

	mutex_lock(&chc->chrgr_cxt_lock);
	charger_type = chc->chrgr_cxt.charger_type;
	mutex_unlock(&chc->chrgr_cxt_lock);

	mutex_lock(&chc->chrgr_cxt_lock);
	ret = chc->charger->disable_charging(charger_type);
	if (!ret) {
		chc->chrgr_cxt.charger_enabled = false;
		pr_debug("%s:%s Charger disabled\n", __FILE__, __func__);
	} else
		pr_err("%s:%s Error in disabling charger\n", __FILE__,
		       __func__);
	mutex_unlock(&chc->chrgr_cxt_lock);

	return ret;

}

static void update_charger_info(enum charger_type chrgr_type,
				struct ch_context *chc,
				struct power_supply_charger_cap cap, int event)
{

	switch (event) {
	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
	case POWER_SUPPLY_CHARGER_EVENT_SUSPEND:
		pr_debug("%s:%s Charger DISCONN/SUSPEND event(%d)\n", __FILE__,
			 __func__, event);

		__report_health(POWER_SUPPLY_HEALTH_UNKNOWN,
				CH_SOURCE_TYPE_CHARGER, chc);

		mutex_lock(&chc->chrgr_cxt_lock);
		memcpy(chc->chrgr_cxt.charger_model, "Unknown",
		       sizeof("Unknown"));
		memcpy(chc->chrgr_cxt.charger_vendor, "Unknown",
		       sizeof("Unknown"));

		chc->chrgr_cxt.charger_present =
		    (event == POWER_SUPPLY_CHARGER_EVENT_SUSPEND);
		if (chrgr_type == CHARGER_USB)
			chc->chrgr_cxt.usb_charger_type = POWER_SUPPLY_TYPE_USB;

		mutex_unlock(&chc->chrgr_cxt_lock);
		break;
	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
	case POWER_SUPPLY_CHARGER_EVENT_UPDATE:
	case POWER_SUPPLY_CHARGER_EVENT_RESUME:
		pr_debug("%s:%s Charger CONNECT/UPDATE/RESUME event(%d)\n",
			 __FILE__, __func__, event);
		mutex_lock(&chc->chrgr_cxt_lock);
		if (chrgr_type == CHARGER_USB)
			chc->chrgr_cxt.usb_charger_type =
				chc->chrgr_cxt.cap.chrg_type;
		 else if (chrgr_type == CHARGER_AC)
			chc->chrgr_cxt.charger_type = POWER_SUPPLY_TYPE_MAINS;

		chc->chrgr_cxt.charger_present = true;
		mutex_unlock(&chc->chrgr_cxt_lock);

		__report_health(POWER_SUPPLY_HEALTH_GOOD,
				CH_SOURCE_TYPE_CHARGER, chc);

		/*FIXME: get charger model and vendor" */
		mutex_lock(&chc->chrgr_cxt_lock);
		memcpy(chc->chrgr_cxt.charger_model, "msic", sizeof("msic"));
		memcpy(chc->chrgr_cxt.charger_vendor, "Intel", sizeof("Intel"));
		mutex_unlock(&chc->chrgr_cxt_lock);
		break;
	default:
		pr_warning("%s:%s Invalid charger event(%d)\n", __FILE__,
			   __func__, event);
	}

	return;

}

static int charger_event_handler(enum charger_type chrgr_type,
				 struct ch_context *chc, int event,
				 struct power_supply_charger_cap cap)
{

	pr_debug("%s:%s\n", __FILE__, __func__);

	/* Update charger info */
	update_charger_info(chrgr_type, chc, cap, event);

	/* check for valid battery condition */
	if (chc->charger->invalid_battery) {
		chc->charger->charger_callback(__get_charger_type(chc));
		return 0;
	}

	switch (event) {

	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
		/*FIXME:PM policy to be decide by charger driver */
		pm_runtime_get_sync(chc->charger->dev);
	case POWER_SUPPLY_CHARGER_EVENT_RESUME:
	case POWER_SUPPLY_CHARGER_EVENT_UPDATE:
		/* if POLL_TEMP_MON is not set we use h/w temp mon.
		* So we do onetime charging register configuration
		*/
		chc->charger->set_ilimmA(cap.mA);
		if (!(chc->charger->flags & POLL_TEMP_MON))
			do_charging(chc);
		/*start charger thread if POLL_TEMP_MON or POLL_CHRG_FULL_DET
		 * or POLL_WDT_RESET set
		 */
		if ((chc->charger->flags & POLL_TEMP_MON)
		    || (chc->charger->flags & POLL_CHRG_FULL_DET)
		    || (chc->charger->flags & POLL_WDT_RESET))
			schedule_delayed_work(&chc->chrg_work, 0);

		mutex_lock(&chc->chrgr_cxt_lock);
		chc->chrgr_cxt.charger_mode = CHARGER_MODE_NORMAL;
		mutex_unlock(&chc->chrgr_cxt_lock);

		break;

	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
		/*FIXME:PM policy to be decide by charger driver */
		pm_runtime_put_sync(chc->charger->dev);
	case POWER_SUPPLY_CHARGER_EVENT_SUSPEND:
		cancel_delayed_work_sync(&chc->chrg_work);
		stop_charging(chc);
		/*disable charger*/
		mutex_lock(&chc->chrgr_cxt_lock);
		chc->charger->disable_charger(chc->chrgr_cxt.charger_type);
		mutex_unlock(&chc->chrgr_cxt_lock);
		break;
	default:
		pr_warning("%s:%s Invalid Charger Event(%d)\n",
			   __FILE__, __func__, event);
	}

	chc->charger->charger_callback(__get_charger_type(chc));

	return 0;
}

void charger_helper_report_battery_full(void *handle)
{

	struct ch_context *chc = (struct ch_context *)handle;

	pr_debug("%s:%s Battery full\n", __FILE__, __func__);

	mutex_lock(&chc->chrgr_cxt_lock);
	chc->chrgr_cxt.charger_mode = CHARGER_MODE_MAINTENANCE;
	mutex_unlock(&chc->chrgr_cxt_lock);

	stop_charging(chc);

	if (chc->charger->flags & POLL_MAINT_CHARGING)
		schedule_delayed_work(&chc->chrg_work, 0);

}
EXPORT_SYMBOL(charger_helper_report_battery_full);

static void charger_worker(struct work_struct *work)
{

	int tzone, temp, charger_mode;
	struct ch_context *chc = container_of(work, struct ch_context,
					      chrg_work.work);

	mutex_lock(&chc->chrgr_cxt_lock);
	charger_mode = chc->chrgr_cxt.charger_mode;
	mutex_unlock(&chc->chrgr_cxt_lock);

	if (charger_mode != CHARGER_MODE_MAINTENANCE) {
		if (chc->charger->flags & POLL_TEMP_MON)
			do_charging(chc);

		/*FIXME: Is this algorithm  generic? */
		if (chc->charger->flags & POLL_CHRG_FULL_DET) {
			if ((chc->charger->get_battery_avg_current() <=
			     chc->charger->term_cur)
			    && (chc->charger->get_battery_voltage() >=
				chc->charger->term_volt)) {
				charger_helper_report_battery_full((void *)chc);
			}

		}

	} else if (chc->charger->flags & POLL_MAINT_CHARGING) {

		/*FIXME: Is this algorithm  generic? */
		temp = chc->charger->get_battery_temperature();
		tzone = get_tempzone(chc->batt_chrg_profile, temp);
		if (tzone == -EINVAL)
			stop_charging(chc);
		else if (chc->charger->get_battery_voltage() <=
		    chc->batt_chrg_profile.
		    temp_mon_range[tzone].maint_chrg_vol_ll)
			do_charging(chc);
		else if (chc->charger->get_battery_voltage() >=
			 chc->batt_chrg_profile.temp_mon_range[tzone].
			 maint_chrg_vol_ul)
			stop_charging(chc);
	}

	if (chc->charger->flags & POLL_WDT_RESET) {
		chc->charger->reset_wdt();
		pr_debug("%s:%s Watch dog timer reset\n", __FILE__, __func__);
	}

	schedule_delayed_work(&chc->chrg_work,
			      chc->charger->chrg_mon_delay * HZ);
}

static void usb_charger_callback_worker(struct work_struct *work)
{
	struct ch_context *chc = container_of(work, struct ch_context,
					      usb_chrgr_work.work);
	charger_event_handler(CHARGER_USB, chc, chc->chrgr_cxt.cap.chrg_evt,
			      chc->chrgr_cxt.cap);
}

void charger_helper_charger_port_changed(void *handle,
					 struct power_supply_charger_cap *cap)
{
	struct ch_context *chc = (struct ch_context *)handle;

	memcpy(&chc->chrgr_cxt.cap, cap,
	       sizeof(struct power_supply_charger_cap));

	schedule_delayed_work(&chc->usb_chrgr_work, 0);
}


void *charger_helper_register_charger(struct charger_helper_charger *charger)
{

	struct ch_context *chc;

	if (!charger->dev)
		return NULL;

	if (!(charger->flags & USB_CHRGR_SUPPORTED)
	    && !(charger->flags & AC_ADAPT_SUPPORTED))
		return NULL;

	if (!charger->enable_charging || !charger->disable_charging
	    || !charger->set_ilimmA || !charger->charger_callback)
		return NULL;

	if ((charger->flags & POLL_WDT_RESET) && !charger->reset_wdt)
		return NULL;
	if ((charger->flags & POLL_EXCP_RECVR)
	    && !charger->is_exception_recovered)
		return NULL;
	chc = kzalloc(sizeof(struct ch_context), GFP_KERNEL);
	if (!chc) {
		pr_err("%s:%d Error in allocating memory\n",
		       __FILE__, __LINE__);
		return NULL;
	}

	chc->charger = charger;

	if (get_batt_charging_profile(&chc->batt_chrg_profile)) {
		pr_err("%s:%d Unable to find battery profile"
			       "Invalid battery detected\n",
				__FILE__, __LINE__);
		charger->invalid_battery = true;
	}

	if (charger->flags & POLL_TEMP_MON || charger->flags & POLL_WDT_RESET
	    || charger->flags & POLL_CHRG_FULL_DET
	    || charger->flags & POLL_MAINT_CHARGING) {

		INIT_DELAYED_WORK(&chc->chrg_work, charger_worker);
	}
	if (charger->flags & POLL_EXCP_RECVR)
		INIT_DELAYED_WORK(&chc->excep_work, recover_exception_worker);

	if (charger->flags & USB_CHRGR_SUPPORTED)
		INIT_DELAYED_WORK(&chc->usb_chrgr_work,
				  usb_charger_callback_worker);

	INIT_LIST_HEAD(&chc->chrgr_cxt.chrgr_hlth_lst);
	INIT_LIST_HEAD(&chc->chrgr_cxt.batt_hlth_lst);

	memcpy(chc->chrgr_cxt.charger_model, "Unknown", sizeof("Unknown"));
	memcpy(chc->chrgr_cxt.charger_vendor, "Unknown", sizeof("Unknown"));

	/*FIXME: move to init call? */
	mutex_init(&chc->chrgr_cxt_lock);
	mutex_init(&chc->excep_lock);

	__report_health(POWER_SUPPLY_HEALTH_UNKNOWN, CH_SOURCE_TYPE_CHARGER,
			chc);

	if (charger->invalid_battery)
		__report_health(POWER_SUPPLY_HEALTH_UNKNOWN,
				CH_SOURCE_TYPE_BATTERY, chc);
	else
		__report_health(POWER_SUPPLY_HEALTH_GOOD,
				CH_SOURCE_TYPE_BATTERY, chc);

	return (void *)chc;
}
EXPORT_SYMBOL(charger_helper_register_charger);

void charger_helper_unregister_charger(void *handle)
{
	struct ch_context *chc = (struct ch_context *)handle;
	if (chc) {
		flush_scheduled_work();
		__flush_health_list(&chc->chrgr_cxt.chrgr_hlth_lst);
		__flush_health_list(&chc->chrgr_cxt.batt_hlth_lst);
		kfree(handle);
	}
}
EXPORT_SYMBOL(charger_helper_unregister_charger);
