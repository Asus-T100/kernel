#ifndef __BATTERY_ID_H__

#define __BATTERY_ID_H__

enum {
	POWER_SUPPLY_BATTERY_REMOVED = 0,
	POWER_SUPPLY_BATTERY_INSERTED,
};

enum batt_chrg_prof_type {
	PSE_MOD_CHRG_PROF = 0,
};

/* charging profile structure definition */
struct ps_batt_chg_prof {
	enum batt_chrg_prof_type chrg_prof_type;
	void *batt_prof;
};

/*For notification during battery change event*/
extern struct atomic_notifier_head    batt_id_notifier;

extern void battery_prop_changed(int battery_conn_stat,
				struct ps_batt_chg_prof *batt_prop);
extern int get_batt_prop(struct ps_batt_chg_prof *batt_prop);
extern int batt_id_reg_notifier(struct notifier_block *nb);
extern void batt_id_unreg_notifier(struct notifier_block *nb);
#endif
