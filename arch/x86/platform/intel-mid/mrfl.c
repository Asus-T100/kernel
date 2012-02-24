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

unsigned long __init intel_mid_calibrate_tsc(void)
{
	/* [REVERT ME] fast timer calibration method to be defined */
	if (intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_VP) {
		lapic_timer_frequency = 50000;
		return 1000000;
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
