/*
 * bq24192_charger.h - Charger driver for TI BQ24190/191/192/192I
 *
 * Copyright (C) 2012 Intel Corporation
 * Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __BQ24192_CHARGER_H_
#define __BQ24192_CHARGER_H_

#define SFI_TEMP_NR_RNG	4
#define BATTID_STR_LEN	8
#define RANGE	25
enum bq24192_bat_chrg_mode {
	BATT_CHRG_FULL = 0,
	BATT_CHRG_NORMAL = 1,
	BATT_CHRG_MAINT
};

struct bq24192_platform_data {
	bool slave_mode;
	unsigned int vHigh;
	unsigned int vLow;
	int fc_dcp_curr;
	int fc_sdp_curr;
	int maint_chrg_curr;
	bool sfi_tabl_present;
};

/*********************************************************************
SFI table entries Structures
*********************************************************************/

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

struct clt_batt_sfi_prop {
	char batt_id[BATTID_STR_LEN];
	unsigned short int voltage_max;
	unsigned int capacity;
	u8 battery_type;
	u8 temp_mon_ranges;
	struct temp_mon_table temp_mon_range[SFI_TEMP_NR_RNG];
};
int bq24192_slave_mode_enable_charging(int volt, int cur, int ilim);
int bq24192_slave_mode_disable_charging(void);
extern int bq24192_query_battery_status(void);

#endif /* __BQ24192_CHARGER_H_ */
