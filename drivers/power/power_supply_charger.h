
#ifndef __POWER_SUPPLY_CHARGER_H__

#define __POWER_SUPPLY_CHARGER_H__
#include <linux/power/battery_id.h>
#include <linux/power_supply.h>

struct batt_props {
	struct list_head node;
	const char *name;
	unsigned long voltage_now;
	long current_now;
	int temperature;
	long status;
};

struct charger_props {
	struct list_head node;
	const char *name;
	bool present;
	bool status;
	bool online;
	unsigned long cable;
};

struct charging_algo {
	struct list_head node;
	unsigned int chrg_prof_type;
	char *name;
	int (*get_next_cc_cv)(struct batt_props, struct ps_batt_chg_prof,
				unsigned long *cc, unsigned long *cv);
};

extern int power_supply_register_charging_algo(struct charging_algo *);
extern int power_supply_unregister_charging_algo(struct charging_algo *);

static inline int set_ps_int_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      int prop_val)
{

	union power_supply_propval val;

	val.intval = prop_val;
	return psy->set_property(psy, psp, &val);
}

static inline int get_ps_int_property(struct power_supply *psy,
				      enum power_supply_property psp)
{
	union power_supply_propval val;

	psy->get_property(psy, psp, &val);
	return val.intval;
}

#define enable_charging(psy) \
		({if ((CABLE_TYPE(psy) != POWER_SUPPLY_CHARGER_TYPE_NONE) &&\
			!IS_CHARGING_ENABLED(psy)) \
		set_ps_int_property(psy, POWER_SUPPLY_PROP_ENABLE_CHARGING,\
					true);\
		enable_charger(psy); })
#define disable_charging(psy) \
		({if ((CABLE_TYPE(psy) != POWER_SUPPLY_CHARGER_TYPE_NONE) &&\
				IS_CHARGING_ENABLED(psy)) \
		set_ps_int_property(psy,\
				POWER_SUPPLY_PROP_ENABLE_CHARGING, false); })

#define enable_charger(psy) \
		set_ps_int_property(psy, POWER_SUPPLY_PROP_ENABLE_CHARGER, true)
#define disable_charger(psy) \
		set_ps_int_property(psy,\
				POWER_SUPPLY_PROP_ENABLE_CHARGER, false)

#define set_cc(psy, cc) \
		set_ps_int_property(psy, POWER_SUPPLY_PROP_CHARGE_CURRENT, cc)

#define set_cv(psy, cv) \
		set_ps_int_property(psy, POWER_SUPPLY_PROP_CHARGE_VOLTAGE, cv)

#define set_inlmt(psy, inlmt) \
		set_ps_int_property(psy, POWER_SUPPLY_PROP_INLMT, inlmt)
#define SET_MAX_CC(psy, max_cc) \
		set_ps_int_property(psy,\
				POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT, max_cc)
#define switch_cable(psy, new_cable) \
		set_ps_int_property(psy,\
				POWER_SUPPLY_PROP_CABLE_TYPE, new_cable)

#define CV(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_CHARGE_VOLTAGE)
#define CC(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_CHARGE_CURRENT)
#define INLMT(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_INLMT)
#define MAX_CC(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT)
#define MAX_CV(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE)
#define VOLTAGE_NOW(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW)
#define CURRENT_NOW(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW)
#define STATUS(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_STATUS)
#define TEMPERATURE(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_TEMP)
#define BATTERY_TYPE(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_TECHNOLOGY)
#define PRIORITY(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_PRIORITY)
#define CABLE_TYPE(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_CABLE_TYPE)

#define IS_CHARGING_ENABLED(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_ENABLE_CHARGING)
#define IS_CHARGER_ENABLED(psy) \
		get_ps_int_property(psy, POWER_SUPPLY_PROP_ENABLE_CHARGER)
#define IS_BATTERY(psy) (psy->type == POWER_SUPPLY_TYPE_BATTERY)
#define IS_CHARGER(psy) (psy->type == POWER_SUPPLY_TYPE_USB ||\
				psy->type == POWER_SUPPLY_TYPE_USB_CDP || \
			psy->type == POWER_SUPPLY_TYPE_USB_DCP ||\
			psy->type == POWER_SUPPLY_TYPE_USB_ACA)
#define IS_ONLINE(psy) \
		(get_ps_int_property(psy, POWER_SUPPLY_PROP_ONLINE) == 1)
#define IS_PRESENT(psy) \
		(get_ps_int_property(psy, POWER_SUPPLY_PROP_PRESENT) == 1)
#define IS_SUPPORTED_CABLE(psy, cable_type) \
		(psy->supported_cables & cable_type)
#define IS_CABLE_ACTIVE(status) \
	((status != EXTCON_CHRGR_CABLE_DISCONNECTED) ||\
			(status != EXTCON_CHRGR_CABLE_SUSPENDED))

#define IS_CHARGER_PROP_CHANGED(prop, cache_prop)\
	((cache_prop.online != prop.online) || \
	(cache_prop.present != prop.present))

#define IS_BAT_PROP_CHANGED(bat_prop, bat_cache)\
	((bat_cache.voltage_now != bat_prop.voltage_now) || \
	(bat_cache.current_now != bat_prop.current_now) || \
	(bat_cache.temperature != bat_prop.temperature))

#define THROTTLE_ACTION(psy, state)\
		(((psy->throttle_states)+state)->throttle_action)

#define MAX_THROTTLE_STATE(psy)\
		(get_ps_int_property(psy,\
			POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX))

#define CURRENT_THROTTLE_STATE(psy)\
		(get_ps_int_property(psy,\
			POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT))

#define CURRENT_THROTTLE_ACTION(psy)\
		THROTTLE_ACTION(psy, CURRENT_THROTTLE_STATE(psy))

#define THROTTLE_CC_VALUE(psy, state)\
		(((psy->throttle_states)+state)->throttle_val)

#define IS_CHARGING_CAN_BE_ENABLED(psy) \
	((CURRENT_THROTTLE_ACTION(psy) != PSY_THROTTLE_DISABLE_CHARGER)  &&\
	(CURRENT_THROTTLE_ACTION(psy) != PSY_THROTTLE_DISABLE_CHARGING))
#define IS_CHARGER_CAN_BE_ENABLED(psy) \
	(CURRENT_THROTTLE_ACTION(psy) != PSY_THROTTLE_DISABLE_CHARGER)

#endif
