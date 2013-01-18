/*
 * platform_msic_battery.c: MSIC battery platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
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
#include <linux/power/intel_mdf_battery.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_msic_battery.h"

#define SFI_SIG_OEM0        "OEM0"
#define CHARGER_PS_NAME "msic_charger"

/* Battery data Offset range in SMIP */
#define BATT_SMIP_BASE_OFFSET		0x314
#define BATT_SMIP_END_OFFSET		0x3F8

/* Battery data Offset range in UMIP */
#define BATT_UMIP_BASE_OFFSET		0x800
#define BATT_UMIP_END_OFFSET		0xBFF
/* UMIP parameter Offsets from UMIP base */
#define UMIP_REV_MAJ_MIN_NUMBER		0x800
#define UMIP_SIZE_IN_BYTES		0x802

#define UMIP_FG_TBL_SIZE		158
#define UMIP_REF_FG_TBL			0x806	/* 2 bytes */

#define UMIP_NO_OF_CFG_TBLS		0X8A4
#define UMIP_BATT_FG_TABLE_SIZE		0x9E
#define UMIP_NO_OF_CFG_TBLS_SIZE	0x01
#define UMIP_BATT_FG_TABLE_OFFSET	0x8A5
#define UMIP_BATT_FG_TERMINATION_CURRENT 0x2E

#define UMIP_BATT_FG_CFG_TBL1 UMIP_BATT_FG_TABLE_OFFSET

#define UMIP_BATT_FG_CFG_TBL2 \
	(UMIP_BATT_FG_CFG_TBL1 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL3 \
	(UMIP_BATT_FG_CFG_TBL2 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL4 \
	(UMIP_BATT_FG_CFG_TBL3 + UMIP_BATT_FG_TABLE_SIZE)
#define UMIP_BATT_FG_CFG_TBL5 \
	(UMIP_BATT_FG_CFG_TBL4 + UMIP_BATT_FG_TABLE_SIZE)

/* UMIP BATT or FG Table definition */
#define BATT_FG_TBL_REV			0	/* 2 bytes */
#define BATT_FG_TBL_NAME		2	/* 4 bytes */
#define BATT_FG_TBL_BATTID		6	/* 8 bytes */
#define BATT_FG_TBL_SIZE		14	/* 2 bytes */
#define BATT_FG_TBL_CHKSUM		16	/* 2 bytes */
#define BATT_FG_TBL_TYPE		18	/* 1 bytes */
#define BATT_FG_TBL_BODY		14	/* 144 bytes */

#define UMIP_READ	0
#define UMIP_WRITE	1
#define SMIP_READ	2
#define MSIC_IPC_READ	0
#define MSIC_IPC_WRITE	1
#define MAX_IPC_ERROR_COUNT 20

/* default settings for invalid battery */
#define CHR_CHRVOLTAGE_SET_DEF		4200
#define MSIC_BATT_VMIN_THRESHOLD	3600	/* 3600mV */
#define BATT_CRIT_CUTOFF_VOLT		3700	/* 3700 mV */
#define DEFAULT_MAX_CAPACITY		1500

/*current multiplication factor*/
#define TERMINATION_CUR_CONV_FACTOR  156
#define FULL_CURRENT_AVG_LOW	0
#define FULL_CURRENT_AVG_HIGH	50

static struct resource msic_battery_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_battery_res[] __initdata = {
	{
		.name                   = "msic_battery",
		.num_resources          = ARRAY_SIZE(msic_battery_resources),
		.resources              = msic_battery_resources,
	},

};


static struct intel_msic_avp_pdata pdata;

/**
* mfld_umip_read_termination_current - reads the termination current data from umip using IPC.
* @term_curr : termination current read from umip.
*/
void  mfld_umip_read_termination_current(u32 *term_curr)
{
	int mip_offset, ret;
	/* Read 2bytes of termination current data from the umip */
	mip_offset = UMIP_REF_FG_TBL + UMIP_BATT_FG_TERMINATION_CURRENT;
	ret = intel_scu_ipc_read_mip((u8 *)term_curr, 2, mip_offset, 0);
	if (ret) {
		pr_warn("Reading umip for termination_current failed. setting to default");
		*term_curr = FULL_CURRENT_AVG_HIGH;
	} else {
		 /* multiply the current with maxim current conversion factor*/
		 *term_curr *= TERMINATION_CUR_CONV_FACTOR;
		 /* convert in to mili amps */
		 *term_curr /= 1000;
	}
	pr_info("termination_current read from umip: %dmA\n", *term_curr);
}
EXPORT_SYMBOL(mfld_umip_read_termination_current);

/**
 * get_batt_fg_curve_index - get the fg curve ID from umip
 * Returns FG curve ID
 */
static int get_batt_fg_curve_index(struct msic_batt_sfi_prop *sfi_table)
{
	int mip_offset, i, ret;
	u8 batt_id[BATTID_STR_LEN + 1];
	u8 num_tbls = 0;

	/* get the no.of tables from mip */
	ret = intel_scu_ipc_read_mip((u8 *)&num_tbls, 1,
					UMIP_NO_OF_CFG_TBLS, 0);
	if (ret) {
		pr_warn("%s: umip read failed\n", __func__);
		goto get_idx_failed;
	}

	/* compare the batt ID provided by SFI table and FG table in mip */
	mip_offset = UMIP_BATT_FG_TABLE_OFFSET + BATT_FG_TBL_BATTID;
	for (i = 0; i < num_tbls; i++) {
		ret = intel_scu_ipc_read_mip(batt_id, BATTID_STR_LEN,
							mip_offset, 0);
		if (ret) {
			pr_warn("%s: umip read failed\n", __func__);
			goto get_idx_failed;
		}
		pr_info("[umip] tbl:%d, batt_id:%s\n", i, batt_id);

		if (!strncmp(batt_id, sfi_table->batt_id, BATTID_STR_LEN))
			break;

		mip_offset += UMIP_FG_TBL_SIZE;
		memset(batt_id, 0x0, BATTID_STR_LEN);
	}

	if (i < num_tbls)
		ret = i;
	else
		ret = -ENXIO;

get_idx_failed:
	return ret;
}

/**
 * intel_msic_store_referenced_table - store data to
 * referenced table from preconfigured battery data
 */
static int intel_msic_store_refrenced_table(struct msic_batt_sfi_prop
								*sfi_table)
{
	int mip_offset, ret, batt_index;
	void *data;
	u8 batt_id[BATTID_STR_LEN] = {0};

	pr_info("[sfi->batt_id]:%s\n", sfi_table->batt_id);

	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BATTID;
	ret = intel_scu_ipc_read_mip(batt_id, BATTID_STR_LEN,
						mip_offset, 0);
	if (ret) {
		pr_warn("%s: umip read failed\n", __func__);
		goto store_err;
	}

	/*if correct table is already in place then don't do anything*/
	if (!strncmp(batt_id, sfi_table->batt_id, BATTID_STR_LEN)) {
		pr_info("%s: match found in ref tbl already\n", __func__);
		return 0;
	}

	batt_index = get_batt_fg_curve_index(sfi_table);
	if (batt_index < 0) {
		pr_err("can't find fg battery index\n");
		return batt_index;
	}

	data = kmalloc(UMIP_FG_TBL_SIZE, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto store_err;
	}

	/* read the fg data from batt_index */
	mip_offset = UMIP_BATT_FG_TABLE_OFFSET + UMIP_FG_TBL_SIZE * batt_index;
	ret = intel_scu_ipc_read_mip((u8 *)data, UMIP_FG_TBL_SIZE,
							mip_offset, 0);
	if (ret) {
		pr_warn("%s: umip read failed\n", __func__);
		kfree(data);
		goto store_err;
	}
	/* write the data to ref table */
	mip_offset = UMIP_REF_FG_TBL;
	ret = intel_scu_ipc_write_umip((u8 *)data, UMIP_FG_TBL_SIZE,
					mip_offset);
	if (ret) {
		pr_warn("%s: umip read failed\n", __func__);
		kfree(data);
		goto store_err;
	}

	kfree(data);

store_err:
	return ret;
}

/**
 * intel_msic_restore_config_data - restore config data
 * @name : Power Supply name
 * @data : config data output pointer
 * @len : length of config data
 *
 */
int intel_msic_restore_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;
	struct	msic_batt_sfi_prop *sfi_table = &pdata.sfi_table;

	/* check if msic charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	ret = intel_msic_store_refrenced_table(sfi_table);
	if (ret < 0) {
		pr_err("%s failed to read fg data\n", __func__);
		return ret;
	}

	/* Read the fuel gauge config data from umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_read_mip((u8 *)data, len, mip_offset, 0);
	if (ret)
		pr_warn("%s: umip read failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(intel_msic_restore_config_data);

/**
 * intel_msic_save_config_data - save config data
 * @name : Power Supply name
 * @data : config data input pointer
 * @len : length of config data
 *
 */
int intel_msic_save_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;

	/* check if msic charger is ready */
	if (!power_supply_get_by_name("msic_charger"))
		return -EAGAIN;

	/* write the fuel gauge config data to umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_write_umip((u8 *)data, len, mip_offset);
	if (ret)
		pr_warn("%s: umip write failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(intel_msic_save_config_data);

/* init_batt_thresholds - initialize battery thresholds */
static void get_batt_thresholds(struct batt_safety_thresholds *batt_thrshlds)
{
	int ret;

	/* Read the threshold data from SMIP */
	ret = intel_scu_ipc_read_mip((u8 *) batt_thrshlds,
			  sizeof(struct batt_safety_thresholds),
			  BATT_SMIP_BASE_OFFSET, 1);
	if (ret)
		pr_err("%s: smip read failed\n", __func__);
}

/**
 * sfi_table_invalid_batt - default battery SFI table values  to be
 * used in case of invalid battery
 *
 * @sfi_table : sfi table pointer
 * Context: can sleep
 */
static void sfi_table_invalid_batt(struct msic_batt_sfi_prop *sfi_table)
{
	/*
	 * In case of invalid battery we manually set
	 * the SFI parameters and limit the battery from
	 * charging, so platform will be in discharging mode
	 */
	memcpy(sfi_table->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	sfi_table->voltage_max = CHR_CHRVOLTAGE_SET_DEF;
	sfi_table->capacity = DEFAULT_MAX_CAPACITY;
	sfi_table->battery_type = POWER_SUPPLY_TECHNOLOGY_LION;
	sfi_table->temp_mon_ranges = 0;
}

/**
 * sfi_table_populate - Simple Firmware Interface table Populate
 * @sfi_table: Simple Firmware Interface table structure
 *
 * SFI table has entries for the temperature limits
 * which is populated in a local structure
 */
static int __init sfi_table_populate(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct msic_batt_sfi_prop *pentry;
	int totentrs = 0, totlen = 0;
	int ret = 0;

	sb = (struct sfi_table_simple *)table;
	if (!sb) {
		pr_err("SFI: Unable to map BATT signature\n");
		return -ENODEV;
	}

	totentrs = SFI_GET_NUM_ENTRIES(sb, struct msic_batt_sfi_prop);
	if (totentrs) {
		pentry = (struct msic_batt_sfi_prop *)sb->pentry;
		totlen = totentrs * sizeof(*pentry);
		memcpy(&pdata.sfi_table, pentry, totlen);
		if (pdata.sfi_table.temp_mon_ranges != SFI_TEMP_NR_RNG)
			pr_debug("SFI: batt temp mon array size mismatch:%d\n",
					pdata.sfi_table.temp_mon_ranges);
	} else {
		pr_warn("Invalid battery detected\n");
		ret = -EINVAL;
	}

	return ret;
}
void __init *msic_battery_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;

	handle_ipc_irq_res(entry->irq, ipc_msic_battery_res);

	/* get battery thresholds */
	get_batt_thresholds(&pdata.batt_thrshlds);

	/* check for valid SFI table entry for OEM0 table */
	if (sfi_table_parse(SFI_SIG_OEM0, NULL, NULL, sfi_table_populate)) {
		sfi_table_invalid_batt(&pdata.sfi_table);
		pdata.is_batt_valid = false;
	} else {
		pdata.is_batt_valid = true;
	}

	return &pdata;
}
