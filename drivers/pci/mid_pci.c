/*
 * mid_pci.c - register MRST PCI plaform ops
 *
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/mrst.h>
#include "pci.h"

static bool mid_pci_power_manageable(struct pci_dev *dev)
{
	return true;
}

static pci_power_t mid_pci_choose_state(struct pci_dev *pdev)
{
	return PCI_D3hot;
}

static bool mid_pci_can_wakeup(struct pci_dev *dev)
{
	return true;
}

static int mid_pci_sleep_wake(struct pci_dev *dev, bool enable)
{
	return 0;
}

static struct pci_platform_pm_ops mid_pci_platform_pm = {
	.is_manageable = mid_pci_power_manageable,
#ifdef CONFIG_X86_MRST
	.set_state = pmu_pci_set_power_state,
#endif	
	.choose_state = mid_pci_choose_state,
	.can_wakeup = mid_pci_can_wakeup,
	.sleep_wake = mid_pci_sleep_wake,
#warning pri#2 #28 need pci_platform_pm_ops.run_wake()?
#if 0
	.run_wake = TBD;
#endif
};

/**
 * mid_pci_init - It registers callback function for all the PCI devices
 * for platform specific device power on/shutdown acticities.
 */
static int __init mid_pci_init(void)
{
	int ret = 0;

	pr_info("mid_pci_init is called\n");

	/* Set Moorestown PM up on Moorestown, nothing else */
	if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x26)
		pci_set_platform_pm(&mid_pci_platform_pm);

	return ret;
}
arch_initcall(mid_pci_init);
