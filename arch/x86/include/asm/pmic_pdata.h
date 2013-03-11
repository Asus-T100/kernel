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
extern int pmic_enable_charging(bool);
extern int pmic_set_cc(int);
extern int pmic_set_cv(int);
extern int pmic_set_ilimmA(int);

extern void dump_pmic_regs(void);
#ifdef CONFIG_PMIC_CCSM
extern int pmic_get_health(void);
extern int pmic_get_battery_pack_temp(int *);
#else
static int pmic_get_health(void)
{
	return 0;
}
static int pmic_get_battery_pack_temp(int *temp)
{
	return 0;
}
#endif

#endif
