#define DEBUG
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/extcon.h>
#include <linux/power/battery_id.h>
#include "power_supply.h"
#include "power_supply_charger.h"

#define MAX_CHARGER_COUNT 5

static LIST_HEAD(algo_list);

struct power_supply_charger {
	bool is_cable_evt_reg;
	/*cache battery and charger properties */
	struct list_head chrgr_cache_lst;
	struct list_head batt_cache_lst;
	struct list_head evt_queue;
	spinlock_t evt_lock;
};

struct charger_cable {
	struct work_struct work;
	struct notifier_block nb;
	struct extcon_chrgr_cbl_props cable_props;
	enum extcon_cable_name extcon_cable_type;
	enum power_supply_charger_cable_type psy_cable_type;
	struct extcon_specific_cable_nb extcon_dev;
	struct extcon_dev *edev;
};

static struct power_supply_charger psy_chrgr;

static struct charger_cable cable_list[] = {
	{
	 .psy_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP,
	 .extcon_cable_type = EXTCON_SDP,
	 },
	{
	 .psy_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP,
	 .extcon_cable_type = EXTCON_CDP,
	 },
	{
	 .psy_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP,
	 .extcon_cable_type = EXTCON_DCP,
	 },
	{
	 .psy_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_ACA,
	 .extcon_cable_type = EXTCON_ACA,
	 },
	{
	 .psy_cable_type = POWER_SUPPLY_CHARGER_TYPE_AC,
	 .extcon_cable_type = EXTCON_AC,
	 },
};

static int charger_cable_notifier(struct notifier_block *nb,
				  unsigned long event, void *ptr);
static void charger_cable_event_worker(struct work_struct *work);
struct charging_algo *power_supply_get_charging_algo
		(struct power_supply *, struct ps_batt_chg_prof *);

static void init_charger_cables(struct charger_cable *cable_lst, int count)
{
	struct charger_cable *cable;
	struct extcon_chrgr_cbl_props cable_props;
	const char *cable_name;

	while (--count) {
		cable = cable_lst++;
		/* initialize cable instance */
		INIT_WORK(&cable->work, charger_cable_event_worker);
		cable->nb.notifier_call = charger_cable_notifier;
		cable->cable_props.cable_stat = EXTCON_CHRGR_CABLE_DISCONNECTED;
		cable->cable_props.mA = 0;
		cable_name = extcon_cable_name[cable->extcon_cable_type];

		if (extcon_register_interest(&cable->extcon_dev,
				NULL, cable_name, &cable->nb))
				continue;

		cable->edev = cable->extcon_dev.edev;

		if (!cable->edev)
			continue;

		if (cable->edev->get_cable_properties(cable_name,
						      (void *)&cable_props)) {
			continue;

		} else if (cable_props.cable_stat !=
			   cable->cable_props.cable_stat) {
			cable->cable_props.cable_stat = cable_props.cable_stat;
			cable->cable_props.mA = cable_props.mA;
		}
	}
}

static inline void get_cur_chrgr_prop(struct power_supply *psy,
				      struct charger_props *chrgr_prop)
{
	chrgr_prop->name = psy->name;
	chrgr_prop->online = IS_ONLINE(psy);
	chrgr_prop->present = IS_PRESENT(psy);
	chrgr_prop->cable = CABLE_TYPE(psy);

}

static inline int get_chrgr_prop_cache(struct power_supply *psy,
				       struct charger_props *chrgr_cache)
{

	struct charger_props *chrgr_prop;
	int ret = -ENODEV;

	list_for_each_entry(chrgr_prop, &psy_chrgr.chrgr_cache_lst, node) {
		if (!strcmp(chrgr_prop->name, psy->name)) {
			memcpy(chrgr_cache, chrgr_prop, sizeof(*chrgr_cache));
			ret = 0;
			break;
		}
	}

	return ret;
}

static inline void cache_chrgr_prop(struct charger_props *chrgr_prop_new)
{

	struct charger_props *chrgr_cache;

	list_for_each_entry(chrgr_cache, &psy_chrgr.chrgr_cache_lst, node) {
		if (!strcmp(chrgr_cache->name, chrgr_prop_new->name))
			goto update_props;
	}

	chrgr_cache = kzalloc(sizeof(*chrgr_cache), GFP_KERNEL);
	if (IS_ERR(chrgr_cache)) {
		pr_err("%s:%dError in allocating memory\n", __FILE__, __LINE__);
		return;
	}

	INIT_LIST_HEAD(&chrgr_cache->node);
	list_add_tail(&chrgr_cache->node, &psy_chrgr.chrgr_cache_lst);

	chrgr_cache->name = chrgr_prop_new->name;

update_props:
	chrgr_cache->status = chrgr_prop_new->status;
	chrgr_cache->present = chrgr_prop_new->present;
	chrgr_cache->cable = chrgr_prop_new->cable;
}


static inline bool is_chrgr_prop_changed(struct power_supply *psy)
{

	struct charger_props chrgr_prop_cache, chrgr_prop;

	get_cur_chrgr_prop(psy, &chrgr_prop);
	/* Get cached battery property. If no cached property available
	 *  then cache the new property and return true
	 */
	if (get_chrgr_prop_cache(psy, &chrgr_prop_cache)) {
		cache_chrgr_prop(&chrgr_prop);
		return true;
	}

	if (!IS_CHARGER_PROP_CHANGED(chrgr_prop, chrgr_prop_cache))
		return false;

	cache_chrgr_prop(&chrgr_prop);
	return true;
}

static inline void cache_bat_prop(struct batt_props *bat_prop_new)
{

	struct batt_props *bat_cache;

	/* Find entry in cache list. If an entry is located update
	 * the existing entry else create new entry in the list */
	list_for_each_entry(bat_cache, &psy_chrgr.batt_cache_lst, node) {
		if (!strcmp(bat_cache->name, bat_prop_new->name))
			goto update_props;
	}

	bat_cache = kzalloc(sizeof(*bat_cache), GFP_KERNEL);
	if (IS_ERR(bat_cache)) {
		pr_err("%s:%dError in allocating memory\n", __FILE__, __LINE__);
		return;
	}
	INIT_LIST_HEAD(&bat_cache->node);
	list_add_tail(&bat_cache->node, &psy_chrgr.batt_cache_lst);

	bat_cache->name = bat_prop_new->name;

update_props:
	bat_cache->voltage_now = bat_prop_new->voltage_now;
	bat_cache->current_now = bat_prop_new->current_now;
	bat_cache->temperature = bat_prop_new->temperature;
	bat_cache->status = bat_prop_new->status;
}

static inline int get_bat_prop_cache(struct power_supply *psy,
				     struct batt_props *bat_cache)
{

	struct batt_props *bat_prop;
	int ret = -ENODEV;

	list_for_each_entry(bat_prop, &psy_chrgr.batt_cache_lst, node) {
		if (!strcmp(bat_prop->name, psy->name)) {
			memcpy(bat_cache, bat_prop, sizeof(*bat_cache));
			ret = 0;
			break;
		}
	}

	return ret;
}

static inline void get_cur_bat_prop(struct power_supply *psy,
				    struct batt_props *bat_prop)
{
	bat_prop->name = psy->name;
	bat_prop->voltage_now = VOLTAGE_NOW(psy) / 1000;
	bat_prop->current_now = CURRENT_NOW(psy) / 1000;
	bat_prop->temperature = TEMPERATURE(psy) / 10;
	bat_prop->status = STATUS(psy);

}

static inline bool is_batt_prop_changed(struct power_supply *psy)
{

	struct batt_props bat_prop_cache, bat_prop;

	/* Get cached battery property. If no cached property available
	 *  then cache the new property and return true
	 */
	get_cur_bat_prop(psy, &bat_prop);
	if (get_bat_prop_cache(psy, &bat_prop_cache)) {
		cache_bat_prop(&bat_prop);
		return true;
	}

	if (!IS_BAT_PROP_CHANGED(bat_prop, bat_prop_cache))
		return false;

	cache_bat_prop(&bat_prop);
	return true;
}

static inline bool is_trigger_charging_algo(struct power_supply *psy)
{

	/* trigger charging alorithm if battery or
	 * charger properties are changed
	 */

	if ((IS_CHARGER(psy)) && is_chrgr_prop_changed(psy))
		return true;

	if ((IS_BATTERY(psy)) && is_batt_prop_changed(psy))
		return true;

	return false;
}

static int get_supplied_by_list(struct power_supply *psy,
				struct power_supply *psy_lst[])
{
	struct class_dev_iter iter;
	struct device *dev;
	struct power_supply *pst;
	int cnt = 0, i, j;

	if (!IS_BATTERY(psy))
		return 0;

	/* Identify chargers which are supplying power to the battery */
	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (!IS_CHARGER(pst))
			continue;
		for (i = 0; i < pst->num_supplicants; i++) {
			if ((!strcmp(pst->supplied_to[i], psy->name)) &&
			    IS_ONLINE(pst))
				psy_lst[cnt++] = pst;
		}
	}
	class_dev_iter_exit(&iter);

	if (cnt <= 1)
		return cnt;

	/*sort based on priority. 0 has the highest priority  */
	for (i = 0; i < cnt; ++i)
		for (j = 0; j < cnt; ++j)
			if (PRIORITY(psy_lst[j]) > PRIORITY(psy_lst[i]))
				swap(psy_lst[j], psy_lst[i]);

	return cnt;
}

static int trigger_algo(struct power_supply *psy)
{
	unsigned long cc = 0, cv = 0, cc_min;
	struct power_supply *chrgr_lst[MAX_CHARGER_COUNT];
	struct batt_props bat_prop;
	struct charging_algo *algo;
	struct ps_batt_chg_prof chrg_profile;
	int cnt;

	if (psy->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;

	if (get_batt_prop(&chrg_profile)) {
		pr_err("Error in getting charge profile:%s:%d\n", __FILE__,
		       __LINE__);
		return -EINVAL;
	}

	get_bat_prop_cache(psy, &bat_prop);

	algo = power_supply_get_charging_algo(psy, &chrg_profile);
	if (!algo)
		return -EINVAL;

	algo->get_next_cc_cv(bat_prop, chrg_profile, &cc, &cv);

	if (!cc || !cv)
		return -ENODATA;

	/* CC needs to be updated for all chargers which are supplying
	 *  power to this battery to ensure that the sum of CCs of all
	 * chargers are never more than the CC selected by the algo.
	 * The CC is set based on the charger priority.
	 */
	cnt = get_supplied_by_list(psy, chrgr_lst);

	while (cnt--) {
		cc_min = min_t(unsigned long, MAX_CC(chrgr_lst[cnt]), cc);
		cc_min = min_t(unsigned long, INLMT(chrgr_lst[cnt]), cc_min);
		if (cc_min < 0)
			cc_min = 0;
		cc -= cc_min;
		set_cc(chrgr_lst[cnt], cc_min);
		set_cv(chrgr_lst[cnt], cv);
	}
	return 0;
}

static inline void enable_supplied_by_charging
		(struct power_supply *psy, bool is_enable)
{
	struct power_supply *chrgr_lst[MAX_CHARGER_COUNT];
	int cnt;

	if (psy->type != POWER_SUPPLY_TYPE_BATTERY)
		return;
	/* Get list of chargers supplying power to this battery and
	 * disable charging for all chargers
	 */
	cnt = get_supplied_by_list(psy, chrgr_lst);
	while (--cnt) {
		if (is_enable && IS_CHARGING_CAN_BE_ENABLED(psy))
			enable_charging(chrgr_lst[cnt]);
		else
			disable_charging(chrgr_lst[cnt]);
	}
}

void power_supply_trigger_charging_handler(struct power_supply *psy)
{
	int i;
	struct power_supply *psb = NULL;

	if (!psy_chrgr.is_cable_evt_reg)
		return;

	if (is_trigger_charging_algo(psy)) {

		if (IS_BATTERY(psy)) {
			if (trigger_algo(psy))
				enable_supplied_by_charging(psy, false);
			else
				enable_supplied_by_charging(psy, true);

		} else if (IS_CHARGER(psy)) {
			for (i = 0; i < psy->num_supplicants; i++) {
				psb =
				    power_supply_get_by_name(psy->
							     supplied_to[i]);

				if (psb && IS_BATTERY(psb) && IS_PRESENT(psb)) {
					if (trigger_algo(psb)) {
						disable_charging(psy);
						break;
					} else if (IS_CHARGING_CAN_BE_ENABLED
								(psy)) {
						enable_charging(psy);
					}
				}
			}
		}

	}
}
EXPORT_SYMBOL(power_supply_trigger_charging_handler);


static int select_chrgr_cable(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct charger_cable *cable, *max_mA_cable = NULL;
	struct charger_cable *cable_lst = (struct charger_cable *)data;
	unsigned int max_mA = 0;
	int i;

	if (!IS_CHARGER(psy))
		return 0;

	/* get cable with maximum capability */
	for (i = 0; i < ARRAY_SIZE(cable_list); ++i) {
		cable = cable_lst + i;
		if ((!IS_CABLE_ACTIVE(cable->cable_props.cable_stat)) ||
		    (!IS_SUPPORTED_CABLE(psy, cable->psy_cable_type)))
			continue;

		if (cable->cable_props.mA > max_mA) {
			max_mA_cable = cable;
			max_mA = cable->cable_props.mA;
		}
	}

	/* no cable connected. disable charging */
	if (!max_mA_cable) {

		if ((IS_CHARGER_ENABLED(psy) || IS_CHARGING_ENABLED(psy))) {
			disable_charging(psy);
			disable_charger(psy);
		}
		set_cc(psy, 0);
		set_cv(psy, 0);
		set_inlmt(psy, 0);
		switch_cable(psy, POWER_SUPPLY_CHARGER_TYPE_NONE);
		return 0;
	}

	/* cable type changed.New cable connected or existing cable
	 * capabilities changed.switch cable and enable charger and charging
	 */

	if (CABLE_TYPE(psy) != max_mA_cable->psy_cable_type) {
		switch_cable(psy, max_mA_cable->psy_cable_type);
		set_inlmt(psy, max_mA_cable->cable_props.mA);
	} else if (INLMT(psy) != max_mA_cable->cable_props.mA) {
		set_inlmt(psy, max_mA_cable->cable_props.mA);
	}

	power_supply_trigger_charging_handler(psy);
	/* Cable status is same as previous. No action to be taken */
	return 0;

}

static void configure_chrgr_source(struct charger_cable *cable_lst)
{

	class_for_each_device(power_supply_class, NULL,
			      cable_lst, select_chrgr_cable);

}

static void charger_cable_event_worker(struct work_struct *work)
{
	struct charger_cable *cable =
	    container_of(work, struct charger_cable, work);
	struct extcon_chrgr_cbl_props cable_props;

	if (cable->edev->
	    get_cable_properties(extcon_cable_name[cable->extcon_cable_type],
				 (void *)&cable_props)) {
		pr_err("Erron in getting cable(%s) properties from extcon device(%s):%s:%d",
				extcon_cable_name[cable->extcon_cable_type],
				cable->edev->name, __FILE__, __LINE__);
		return;
	} else {
		if (cable_props.cable_stat != cable->cable_props.cable_stat) {
			cable->cable_props.cable_stat = cable_props.cable_stat;
			cable->cable_props.mA = cable_props.mA;
			configure_chrgr_source(cable_list);
		}
	}

}

static int charger_cable_notifier(struct notifier_block *nb,
				  unsigned long stat, void *ptr)
{

	struct charger_cable *cable =
	    container_of(nb, struct charger_cable, nb);

	schedule_work(&cable->work);

	return NOTIFY_DONE | NOTIFY_STOP_MASK;
}

int psy_charger_throttle_charger(struct power_supply *psy,
					unsigned long state)
{

	if (state < 0 || state > MAX_THROTTLE_STATE(psy))
		return -EINVAL;

	switch THROTTLE_ACTION(psy, state)
	{

		case PSY_THROTTLE_DISABLE_CHARGER:
			disable_charger(psy);
			break;
		case PSY_THROTTLE_DISABLE_CHARGING:
			disable_charging(psy);
			break;
		case PSY_THROTTLE_CC_LIMIT:
			SET_MAX_CC(psy, THROTTLE_CC_VALUE(psy, state));
			power_supply_trigger_charging_handler(psy);
			break;
		case PSY_THROTTLE_INPUT_LIMIT:
			set_inlmt(psy, THROTTLE_CC_VALUE(psy, state));
			power_supply_trigger_charging_handler(psy);
			break;
		default:
			pr_err("Invalid throttle action for %s\n", psy->name);
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(psy_charger_throttle_charger);

int power_supply_register_charger(struct power_supply *psy)
{
	int ret = 0;

	if (!psy_chrgr.is_cable_evt_reg) {
		init_charger_cables(cable_list, ARRAY_SIZE(cable_list));
		psy_chrgr.is_cable_evt_reg = true;
		INIT_LIST_HEAD(&psy_chrgr.chrgr_cache_lst);
		INIT_LIST_HEAD(&psy_chrgr.batt_cache_lst);
	}
	return ret;
}
EXPORT_SYMBOL(power_supply_register_charger);

int power_supply_unregister_charger(struct power_supply *psy)
{
 /*TBD*/
	return 0;
}
EXPORT_SYMBOL(power_supply_unregister_charger);

int power_supply_register_charging_algo(struct charging_algo *algo)
{

	struct charging_algo *algo_new;

	algo_new = kzalloc(sizeof(*algo_new), GFP_KERNEL);
	algo_new->get_next_cc_cv = algo->get_next_cc_cv;
	algo_new->name = algo->name;
	algo_new->chrg_prof_type = algo->chrg_prof_type;

	list_add_tail(&algo_new->node, &algo_list);
	return 0;
}
EXPORT_SYMBOL(power_supply_register_charging_algo);

int power_supply_unregister_charging_algo(struct charging_algo *algo)
{
	struct charging_algo *algo_l, *tmp;

	list_for_each_entry_safe(algo_l, tmp, &algo_list, node) {
		if (!strcmp(algo_l->name, algo->name)) {
			list_del(&algo_l->node);
			kfree(algo_l);
		}
	}
	return 0;

}
EXPORT_SYMBOL(power_supply_unregister_charging_algo);

static struct charging_algo *get_charging_algo_byname(char *algo_name)
{
	struct charging_algo *algo;

	list_for_each_entry(algo, &algo_list, node) {
		if (!strcmp(algo->name, algo_name))
			return algo;
	}

	return NULL;
}

static struct charging_algo *get_charging_algo_by_type
		(enum batt_chrg_prof_type chrg_prof_type)
{
	struct charging_algo *algo;

	list_for_each_entry(algo, &algo_list, node) {
		if (algo->chrg_prof_type == chrg_prof_type)
			return algo;
	}

	return NULL;
}

struct charging_algo *power_supply_get_charging_algo
	(struct power_supply *psy, struct ps_batt_chg_prof *batt_prof)
{

	return get_charging_algo_by_type(batt_prof->chrg_prof_type);

}
EXPORT_SYMBOL_GPL(power_supply_get_charging_algo);
