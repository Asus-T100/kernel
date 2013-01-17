/*
 * intel_mid_powersupply.h - Intel MID Power Supply header file
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

#ifndef __INTEL_MID_POWERSUPPLY_H_
#define __INTEL_MID_POWERSUPPLY_H_


/* Battery data Offset range in SMIP */
#define BATT_SMIP_BASE_OFFSET		0x314

#define BATT_STRING_MAX		8
#define BATTID_STR_LEN		8
#define BATT_PROF_MAX_TEMP_NR_RNG 6
/* Temperature Monitoring Table */

struct temp_mon_table {
	short int temp_up_lim;
	short int full_chrg_vol;
	short int full_chrg_cur;
	short int maint_chrg_vol_ll;
	short int maint_chrg_vol_ul;
	short int maint_chrg_cur;
} __packed;

/* Charging Profile */
/* FIXME: make the batt_chargig profile generic */
struct batt_charging_profile {
	char batt_id[BATTID_STR_LEN];
	u8 resvd;
	u8 battery_type;
	u16 capacity;
	u16 voltage_max;
	u16 chrg_term_mA;
	u16 low_batt_mV;
	u8 disch_tmp_ul;
	u8 disch_tmp_ll;
	u16 temp_mon_ranges;
	struct temp_mon_table temp_mon_range[BATT_PROF_MAX_TEMP_NR_RNG];
	short int temp_low_lim;
} __packed;

/* Battery Configuration info */
struct plat_battery_config {
	u8 smip_rev;
	u8 fpo;			/* fixed implementation options */
	u8 fpo1;		/* fixed implementation options1 */
	u8 rsys;		/* System Resistance for Fuel gauging */

	/* Minimum voltage necessary to
	 * be able to safely shut down */
	short int vbatt_sh_min;

	/* Voltage at which the battery driver
	 * should report the LEVEL as CRITICAL */
	short int vbatt_crit;

	short int itc;		/* Charge termination current */
	short int temp_high;	/* Safe Temp Upper Limit */
	short int temp_low;	/* Safe Temp lower Limit */
	u8 brd_id;		/* Unique Board ID */
} __packed;


extern struct batt_charging_profile *batt_chrg_profile;

extern struct plat_battery_config  *plat_batt_config;

static inline int get_batt_charging_profile
	(struct batt_charging_profile *chrg_profile)
{

	if (batt_chrg_profile) {
		memcpy(chrg_profile, batt_chrg_profile,
				sizeof(struct batt_charging_profile));
		return 0;
	}
	return -EINVAL;
}
static inline int get_batt_config(struct plat_battery_config *batt_config)
{
	if (plat_batt_config) {
		memcpy(batt_config, plat_batt_config,
				sizeof(struct plat_battery_config));
		return 0;
	}
	return -EINVAL;
}

#endif
