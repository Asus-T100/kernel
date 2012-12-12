/*
 * platform_mrfl_bq24261.c: Platform data for bq24261 charger driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/power_supply.h>
#include <asm/pmic_pdata.h>
#include <linux/power/bq24261_charger.h>
#include <asm/intel-mid.h>

#include "platform_ipc.h"
#include "platform_bq24261.h"

static struct power_supply_throttle bq24261_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24261_CHRG_CUR_NOLIMIT,

	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24261_CHRG_CUR_HIGH,

	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24261_CHRG_CUR_MEDIUM,

	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGER,
	},

};

char *bq24261_supplied_to[] = {
				"max170xx_battery",
				"max17047_battery",
};


void __init *bq24261_platform_data(void *info)
{
	static struct bq24261_plat_data bq24261_pdata;


	bq24261_pdata.irq_map = PMIC_SRAM_INTR_MAP;
	bq24261_pdata.irq_mask = PMIC_EXT_INTR_MASK;
	bq24261_pdata.supplied_to = bq24261_supplied_to;
	bq24261_pdata.num_supplicants = ARRAY_SIZE(bq24261_supplied_to);
	bq24261_pdata.throttle_states = bq24261_throttle_states;
	bq24261_pdata.num_throttle_states = ARRAY_SIZE(bq24261_throttle_states);
	bq24261_pdata.enable_charging = NULL;
	bq24261_pdata.enable_charger = pmic_enable_charger;
	bq24261_pdata.set_inlmt = pmic_set_ilimmA;
	bq24261_pdata.set_cc = pmic_set_cc;
	bq24261_pdata.set_cv = pmic_set_cv;
	bq24261_pdata.set_iterm = NULL;
	bq24261_pdata.dump_master_regs = dump_pmic_regs;

	return &bq24261_pdata;
}
