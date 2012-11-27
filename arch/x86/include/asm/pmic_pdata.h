#ifndef __PMIC_PDATA_H__
#define __PMIC_PDATA_H__

/*
 * pmic cove charger driver info
 */
struct pmic_platform_data {
	void (*cc_to_reg)(int, u8*);
	void (*cv_to_reg)(int, u8*);
};

extern int pmic_get_status(void);
extern int pmic_enable_charger(bool);
extern int pmic_set_cc(int);
extern int pmic_set_cv(int);
extern int pmic_set_ilimmA(int);

extern int pmic_get_health(void);
extern int pmic_get_battery_pack_temp(int *);

#endif
