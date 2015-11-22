/*
 * intel_crystal_cove_pmic.h - Support for Basin Cove pmic VR
 * Copyright (c) 2013, Intel Corporation.
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
 *
 */
#ifndef __INTEL_CRYSTAL_COVE_PMIC_H_
#define __INTEL_CRYSTAL_COVE_PMIC_H_

#include <linux/notifier.h>

/* Slave Address for all regulators */
#define V2P85SCNT_ADDR	0x065
#define V2P85SXCNT_ADDR	0x066
#define V3P3SXCNT_ADDR	0x069
#define V1P8SCNT_ADDR	0x05c
#define V1P8SXCNT_ADDR	0x05d
#define VSYS_SCNT_ADDR	0x06c
#define V1P0ACNT_ADDR	0x055
#define V1P8ACNT_ADDR	0x05a

#define CRYSTAL_COVE_REGULATOR_ID_START 1000

struct regulator_init_data;

enum intel_regulator_id {
	V2P85S = CRYSTAL_COVE_REGULATOR_ID_START,
	V2P85SX,
	V3P3SX,
	V1P8S,
	V1P8SX,
	V1P0A,
	V1P8A,
	VSYS_S,
};

struct regulator_info {
	struct regulator *regulator;
	struct device *dev;
};

/* Voltage tables for Regulators */
static const unsigned int V2P85S_VSEL_TABLE[] = {
	2565000, 2700000, 2850000, 2900000, 2950000, 3000000, 3135000, 3300000,
};

static const unsigned int V2P85SX_VSEL_TABLE[] = {
	2900000,
};

static const unsigned int V3P3SX_VSEL_TABLE[] = {
	3332000,
};

static const unsigned int  V1P8S_VSEL_TABLE[] = {
	1817000,
};

static const unsigned int  V1P8SX_VSEL_TABLE[] = {
	1817000,
};

static const unsigned int  VSYS_S_VSEL_TABLE[] = {
	4200000,
};

static const unsigned V1P0A_VSEL_TABLE[] = {
	900000, 950000, 1000000, 1020000, 1030000, 1050000, 1100000,
};

static const unsigned V1P8A_VSEL_TABLE[] = {
	1620000, 1710000, 1800000, 1836000, 1854000, 1890000, 1980000,
};

struct pmic_regulator_gpio_en {
	int gpio;
	int init_gpio_state;
};

/**
 * intel_pmic_info - platform data for intel pmic
 * @pmic_reg: pmic register that is to be used for this VR
 */
struct intel_pmic_info {
	struct regulator_init_data *init_data;
	struct regulator_dev *intel_pmic_rdev;
	struct pmic_regulator_gpio_en *en_pin;
	const u16 *table;
	u16 pmic_reg;
	u8 table_len;
};

#ifdef CONFIG_REGULATOR_CRYSTAL_COVE
extern void vrf_notifier_register(struct notifier_block *n);
extern void vrf_notifier_unregister(struct notifier_block *n);
extern void vrf_notifier_call_chain(unsigned int val);
#else
static inline void vrf_notifier_register(struct notifier_block *n) {}
static inline void vrf_notifier_unregister(struct notifier_block *n) {}
static inline void vrf_notifier_call_chain(unsigned int val) {}
#endif /* CONFIG_CRYSTAL_COVE */

#endif /* __INTEL_CRYSTAL_COVE_PMIC_H_ */
