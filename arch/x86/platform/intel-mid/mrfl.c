/*
 * mrfl.c: Intel Merrifield platform specific setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/sfi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <asm/setup.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>
#include <linux/power/intel_mid_powersupply.h>
#include <asm/intel_scu_ipc.h>

#define APIC_DIVISOR 16

unsigned long __init intel_mid_calibrate_tsc(void)
{
	/* [REVERT ME] fast timer calibration method to be defined */
	if (intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_VP) {
		lapic_timer_frequency = 50000;
		return 1000000;
	}

	if ((intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_SLE) ||
		(intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_NONE)) {

		unsigned long fast_calibrate;
		u32 lo, hi, ratio, fsb, bus_freq;

		/* *********************** */
		/* Compute TSC:Ratio * FSB */
		/* *********************** */

		/* Compute Ratio */
		rdmsr(MSR_PLATFORM_INFO, lo, hi);
		pr_debug("IA32 PLATFORM_INFO is 0x%x : %x\n", hi, lo);

		ratio = (lo >> 8) & 0xFF;
		pr_debug("ratio is %d\n", ratio);
		if (!ratio) {
			pr_err("Read a zero ratio, force tsc ratio to 4 ...\n");
			ratio = 4;
		}

		/* Compute FSB */
		rdmsr(MSR_FSB_FREQ, lo, hi);
		pr_debug("Actual FSB frequency detected by SOC 0x%x : %x\n",
			hi, lo);

		bus_freq = (lo >> 4) & 0x7;
		pr_debug("bus_freq = 0x%x\n", bus_freq);

		if (bus_freq == 0)
			fsb = TNG_IDI_FREQ_100SKU;
		else if (bus_freq == 1)
			fsb = TNG_IDI_FREQ_133SKU;
		else if (bus_freq == 2)
			fsb = TNG_IDI_FREQ_200SKU;
		else if (bus_freq == 3)
			fsb = TNG_IDI_FREQ_167SKU;
		else if (bus_freq == 4)
			fsb = TNG_IDI_FREQ_83SKU;
		else if (bus_freq == 5)
			fsb = TNG_IDI_FREQ_400SKU;
		else if (bus_freq == 6)
			fsb = TNG_IDI_FREQ_267SKU;
		else if (bus_freq == 7)
			fsb = TNG_IDI_FREQ_333SKU;
		else {
			BUG();
			pr_err("Invalid bus_freq! Setting to minimal value!\n");
			fsb = TNG_IDI_FREQ_100SKU;
		}

		/* TSC = FSB Freq * Resolved HFM Ratio */
		fast_calibrate = ratio * fsb;
		pr_debug("calculate tangier tsc %lu KHz\n", fast_calibrate);

		/* ************************************ */
		/* Calculate Local APIC Timer Frequency */
		/* ************************************ */
		lapic_timer_frequency = (fsb * 1000) / HZ;

		pr_debug("Setting lapic_timer_frequency = %d\n",
			lapic_timer_frequency);

		/* mark tsc clocksource as reliable */
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_TSC_RELIABLE);

		if (fast_calibrate)
			return fast_calibrate;
	}
	return 0;
}

/* Allow user to enable simulator quirks settings for kernel */
static int __init set_simulation_platform(char *str)
{
	int platform;
	if (get_option(&str, &platform)) {
		__intel_mrfl_sim_platform = platform;
		pr_info("simulator mode %d enabled.\n",
			__intel_mrfl_sim_platform);
		return 0;
	}

	return -EINVAL;
}
early_param("mrfld_simulation", set_simulation_platform);

struct plat_battery_config  *plat_batt_config;
EXPORT_SYMBOL(plat_batt_config);

static int __init get_plat_batt_config(void)
{
	int ret;
	plat_batt_config = kzalloc(sizeof(struct plat_battery_config),
				GFP_KERNEL);
	if (!plat_batt_config) {
		pr_err("%s : Error in allocating"
				"plat_battery_config\n", __func__);
		return -ENOMEM;
	}
	ret = intel_scu_ipc_read_mip((u8 *) plat_batt_config,
		sizeof(struct plat_battery_config), BATT_SMIP_BASE_OFFSET, 1);
	if (ret) {
		pr_err("%s(): Error in reading platform battery"
			" configuration\n", __func__);
		kfree(plat_batt_config);
		plat_batt_config = NULL;
	}
	return ret;
}
/* FIXME: SMIP access is failing. So disabling the SMIP read */
/*rootfs_initcall(get_plat_batt_config); */

struct batt_charging_profile *batt_chrg_profile;
EXPORT_SYMBOL(batt_chrg_profile);

static int __init sfi_parse_batt(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct batt_charging_profile *pentry;
	int totentrs = 0, totlen = 0;

	sb = (struct sfi_table_simple *)table;
	totentrs = SFI_GET_NUM_ENTRIES(sb, struct batt_charging_profile);
	if (totentrs) {
		batt_chrg_profile = kzalloc(
			sizeof(*batt_chrg_profile), GFP_KERNEL);
		if (!batt_chrg_profile) {
			pr_info("%s(): Error in kzalloc\n", __func__);
			return -ENOMEM;
		}
		pentry = (struct batt_charging_profile *)sb->pentry;
		totlen = totentrs * sizeof(*pentry);
		if (totlen <= sizeof(*batt_chrg_profile))
			memcpy(batt_chrg_profile, pentry, totlen);
		else {
			pr_err("%s: Error in copying batt charge profile\n",
							__func__);
			kfree(batt_chrg_profile);
			return -ENOMEM;
		}
	}
	return 0;

}

static int __init mrfl_platform_init(void)
{
	sfi_table_parse(SFI_SIG_OEM0, NULL, NULL, sfi_parse_batt);
	return 0;
}
arch_initcall(mrfl_platform_init);
