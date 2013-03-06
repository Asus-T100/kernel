/*
 * vlv2.c: Intel ValleyView2 platform specific setup code
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Bin Gao <bin.gao@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/i8259.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>
#include <asm/pci_x86.h>

extern struct legacy_pic default_legacy_pic;

/* Restore x86_init to default */
static void __init valleyview2_arch_setup(void)
{
	x86_init.resources.reserve_resources = reserve_standard_io_resources;

	x86_init.irqs.pre_vector_init		= init_ISA_irqs;

	x86_init.timers.setup_percpu_clockev	= setup_boot_APIC_clock;
	x86_init.timers.timer_init		= hpet_time_init;
	x86_init.timers.wallclock_init		= x86_init_noop;

	x86_init.pci.init			= x86_default_pci_init;
	x86_init.pci.fixup_irqs			= x86_default_pci_fixup_irqs;

	x86_cpuinit.setup_percpu_clockev	= setup_secondary_APIC_clock;

	x86_platform.calibrate_tsc		= native_calibrate_tsc;

	legacy_pic				= &default_legacy_pic;
}

static struct intel_mid_ops __initdata valleyview2_ops = {
	.arch_setup = valleyview2_arch_setup,
};

void __init *get_valleyview2_ops(void)
{
	return &valleyview2_ops;
}
