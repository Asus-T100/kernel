
/*
 * intel_mdf_battery.h - Intel Medfield MSIC Internal charger and Battery Driver
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
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#ifndef __INTEL_MDF_BATTERY_H_
#define __INTEL_MDF_BATTERY_H_

#define BATTID_STR_LEN		8
#define SFI_TEMP_NR_RNG		4

/* Battery Thresholds info which need to get from SMIP area */
struct batt_safety_thresholds {
	u8 smip_rev;
	u8 fpo;		/* fixed implementation options */
	u8 fpo1;	/* fixed implementation options1 */
	u8 rsys;	/* System Resistance for Fuel gauging */

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

/* Parameters defining the range */
struct temp_mon_table {
	short int temp_up_lim;
	short int temp_low_lim;
	short int rbatt;
	short int full_chrg_vol;
	short int full_chrg_cur;
	short int maint_chrg_vol_ll;
	short int maint_chrg_vol_ul;
	short int maint_chrg_cur;
} __packed;

/* SFI table entries structure.*/
struct msic_batt_sfi_prop {
	char batt_id[BATTID_STR_LEN];
	unsigned short int voltage_max;
	unsigned int capacity;
	u8 battery_type;
	u8 temp_mon_ranges;
	struct temp_mon_table temp_mon_range[SFI_TEMP_NR_RNG];
} __packed;

struct intel_msic_avp_pdata {
	bool	is_batt_valid;
	struct	batt_safety_thresholds batt_thrshlds;
	struct	msic_batt_sfi_prop sfi_table;
};

extern int intel_msic_is_current_sense_enabled(void);
extern int intel_msic_check_battery_present(void);
extern int intel_msic_check_battery_health(void);
extern int intel_msic_check_battery_status(void);
extern int intel_msic_get_battery_pack_temp(int *val);

extern void  mfld_umip_read_termination_current(u32 *term_curr);

extern bool intel_msic_is_capacity_shutdown_en(void);
extern bool intel_msic_is_volt_shutdown_en(void);
extern bool intel_msic_is_lowbatt_shutdown_en(void);
extern int intel_msic_get_vsys_min(void);

#endif /* __INTEL_MDF_BATTERY_H_ */
