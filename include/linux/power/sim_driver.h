
/*
 * sim_driver.h - Intel MRFL Platform Charging Simulation Driver header file
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ajay Thomas D <ajay.thomas.david.rajamanickam@intel.com>
 */
#ifndef __BASIN_COVE_SIM_H_
#define __BASIN_COVE_SIM_H_

struct sim_drv_prop {
		int event_val;
		uint16_t chgrirq0_val;
		uint16_t schgrirq0_val;
		uint8_t ext_chgr_stat;
		int volt_now;
		int curr_now;
		int batt_temp;
		int vbus_volt_val;
		struct power_supply_charger_cap ps_change_cap;
		void (*chgr_irq_callback)(uint16_t chgrirq0_val,
			uint16_t schgrirq0_val, uint8_t ext_chrgr_stat);
		void *bc_arg;
		struct mutex prop_lock;
		struct mutex access_lock;
	};

enum usb_charger_event {
	CHRG_DISCONN,
	CHRG_SUSPEND,
	CHRG_CONNECT_SDP_LOW,
	CHRG_CONNECT_SDP_HIGH,
	CHRG_CONNECT_DCP,
	CHRG_CONNECT_CDP
};

void batt_sim_register_callback(
	void (*charger_irq_callback)(
		uint16_t chgrirq0, uint16_t schgrirq0, uint8_t ext_chrgr_stat));
int batt_sim_get_batt_temp(int *tmp);
int batt_sim_get_batt_current(int *cur);
int batt_sim_get_batt_volt(int *volt);
int batt_sim_get_vbus_volt(int *vbus_volt);

#endif
