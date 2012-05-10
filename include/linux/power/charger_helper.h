/*
 * charger_helper.h - Charger helper header file
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
/*FIXME:
*	Replace with generic macros
*/

#ifndef USE_CUSTOM_OTG_MACRO
#define USE_CUSTOM_OTG_MACRO
#endif

#ifdef USE_CUSTOM_OTG_MACRO
enum usb_charger_type {
	CHRG_UNKNOWN,
	CHRG_SDP,		/* Standard Downstream Port */
	CHRG_CDP,		/* Charging Downstream Port */
	CHRG_DCP,		/* Dedicated Charging Port */
	CHRG_ACA		/* Accessory Charger Adapter */
};

struct otg_bc_cap {
	enum usb_charger_type chrg_type;
	unsigned int mA;
#define CHRG_CURR_UNKNOWN       0
#define CHRG_CURR_DISCONN       0
#define CHRG_CURR_SDP_SUSP      CONFIG_USB_GADGET_SUSPEND_VBUS_DRAW
#define CHRG_CURR_SDP_LOW       100
#define CHRG_CURR_SDP_HIGH      500
#define CHRG_CURR_CDP           1500
#define CHRG_CURR_DCP   1500
#define CHRG_CURR_ACA   1500
	unsigned int current_event;
};

struct penwell_otg {
	int dummy;
};


#endif

/*FIXME: Remove this mapping with generic macro*/
#define USB_CHRGR_DCP CHRG_DCP
#define USB_CHRGR_CDP CHRG_CDP
#define USB_CHRGR_SDP CHRG_SDP
#define USB_CHRGR_ACA CHRG_ACA

/* charger_flags */
#define	POLL_TEMP_MON			(0x01 << 0)
#define POLL_WDT_RESET			(0x01 << 1)
#define POLL_CHRG_FULL_DET		(0x01 << 2)
#define POLL_MAINT_CHARGING		(0x01 << 3)
#define POLL_EXCP_RECVR			(0x01 << 4)

#define AC_ADAPT_SUPPORTED		(0x01 << 5)
#define USB_CHRGR_SUPPORTED		(0x01 << 6)
#define USB_VSYS_SUPPORTED		(0x01 << 7)
#define AC_VSYS_SUPPORTED		(0x01 << 8)
#define USR_CHRG_CTRL			(0x01 << 9)


enum ch_source_type {
	CH_SOURCE_TYPE_BATTERY,
	CH_SOURCE_TYPE_CHARGER
};

enum charger_type {
	CHARGER_USB,
	CHARGER_AC,
};

enum charger_mode {
	CHARGER_MODE_NORMAL,
	CHARGER_MODE_MAINTENANCE,
};

/*FIXME: make this macros generic across
power_supply and otg layers */

enum event_type {
	EVENT_CONNECT = 0,
	EVENT_UPDATE,
	EVENT_RESUME,
	EVENT_SUSPEND,
	EVENT_DISCONN,
};

enum exception_status {
	EXCEPTION_REPORTED,
	EXCEPTION_RECOVERD,
};

struct charger_helper_charger {
	struct device *dev;
	int flags;
	char *name;
	int excp_recv_delay;

	bool invalid_battery;

	/* This would be used if SW_TEMP_MON,SW_WDT_RESET,
	*  SW_CHRG_TERM or MAINT_MODE_POLL is set
	*/
	int chrg_mon_delay;

	/* FIXME:This should be from thermal mon table? */
	int term_cur;
	int term_volt;

	/* callback function for enable/disable charging */
	int (*enable_charging) (enum charger_type chrg_type);
	int (*disable_charging) (enum charger_type chrg_type);

	/* callback function for enable/disable charger. */
	int (*disable_charger) (enum charger_type chrg_type);

	int (*set_ccmA_cvmV) (int chrcc_mA, int chrcv_mV);
	int (*set_ilimmA)(int ilim_mA);
	int (*reset_wdt) (void);
	bool(*is_battery_charging) (void);
	bool(*is_charger_online) (void);
	bool(*is_exception_recovered) (enum ch_source_type, int health);

	int (*get_battery_temperature) (void);
	int (*get_battery_avg_current) (void);
	int (*get_battery_voltage) (void);
	unsigned int (*get_charger_voltage) (void);

	void (*charger_callback) (int);
};

int charger_helper_get_property(void *handle, enum ch_source_type type,
		     enum power_supply_property psp,
		     union power_supply_propval *val);
void charger_helper_report_exception(void *handle,
	enum  ch_source_type exception_souce, int exception, bool is_recover);
void charger_helper_report_battery_full(void *handle);
void *charger_helper_register_charger(struct charger_helper_charger *charger);
void charger_helper_unregister_charger(void *handle);
void charger_helper_charger_port_changed(void *handle,
			struct power_supply_charger_cap *cap);
