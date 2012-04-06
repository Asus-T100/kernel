/*
 * mfld.c: Intel Medfield platform setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/intel_mid_pm.h>
#include <linux/usb/penwell_otg.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/reboot.h>


#define MSIC_POWER_SRC_STAT 0x192
#define MSIC_POWER_BATT (1 << 0)
#define MSIC_POWER_USB  (1 << 1)

static bool check_charger_conn(void)
{
	int ret;
	struct otg_bc_cap cap;
	u8 data;

	ret = intel_scu_ipc_ioread8(MSIC_POWER_SRC_STAT, &data);
	if (ret)
		return false;

	if (!((data & MSIC_POWER_BATT) && (data & MSIC_POWER_USB)))
		return false;

	ret = penwell_otg_query_charging_cap(&cap);
	if (ret)
		return false;

	if (cap.chrg_type == CHRG_UNKNOWN)
		return false;

	return true;
}

void intel_mid_power_off(void)
{
#ifdef CONFIG_INTEL_MID_OSIP
	bool charger_conn = check_charger_conn();
	if (!get_force_shutdown_occured() && charger_conn) {
		/*
		 * Do a cold reset to let bootloader bring up the
		 * platform into acting dead mode to continue
		 * battery charging.
		 * If some special condition occured and wants to
		 * make a force shutdown, even if the charger is
		 * connected, dont boot(up to COS).
		 */
		intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 0);
	} else
#endif
		mfld_power_off();
}

unsigned long __init intel_mid_calibrate_tsc(void)
{
	unsigned long flags, fast_calibrate;
	u32 lo, hi, ratio, fsb;

	rdmsr(MSR_IA32_PERF_STATUS, lo, hi);
	pr_debug("IA32 perf status is 0x%x, 0x%0x\n", lo, hi);
	ratio = (hi >> 8) & 0x1f;
	pr_debug("ratio is %d\n", ratio);
	if (!ratio) {
		pr_err("read a zero ratio, should be incorrect!\n");
		pr_err("force tsc ratio to 16 ...\n");
		ratio = 16;
	}
	rdmsr(MSR_FSB_FREQ, lo, hi);
	if ((lo & 0x7) == 0x7)
		fsb = PENWELL_FSB_FREQ_83SKU;
	else
		fsb = PENWELL_FSB_FREQ_100SKU;
	fast_calibrate = ratio * fsb;
	pr_debug("read penwell tsc %lu khz\n", fast_calibrate);
	lapic_timer_frequency = fsb * 1000 / HZ;
	/* mark tsc clocksource as reliable */
	set_cpu_cap(&boot_cpu_data, X86_FEATURE_TSC_RELIABLE);

	if (fast_calibrate)
		return fast_calibrate;

	return 0;
}
