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

/* a 3 bits bit-map, from 0 to 7, default 0 */
unsigned char hsu_dma_enable;
EXPORT_SYMBOL_GPL(hsu_dma_enable);

int __init setup_hsu_dma_enable_flag(char *p)
{
	if (!p)
		return -EINVAL;

	hsu_dma_enable = (unsigned char)memparse(p, &p);
	if (hsu_dma_enable & (~0x7))
		return -EINVAL;

	return 0;
}
early_param("hsu_dma", setup_hsu_dma_enable_flag);

#define HSU0_CTS (13)
#define HSU0_RTS (96 + 29)
#define HSU1_RX (64)
#define HSU1_TX (65)
#define HSU1_CTS (68)
#define HSU1_RTS (66)
#define HSU1_ALT_RX (96 + 30)
#define HSU1_ALT_TX (96 + 31)
#define HSU2_RX (67)

/* on = 1: the port1 is muxed (named as port 3) for debug output
 * on = 0: the port1 is for modem fw download.
 */
void mfld_hsu_port1_switch(int on)
{
	static int first = 1;

	if (unlikely(first)) {
		gpio_request(HSU1_RX, "hsu");
		gpio_request(HSU1_TX, "hsu");
		gpio_request(HSU1_CTS, "hsu");
		gpio_request(HSU1_RTS, "hsu");
		gpio_request(HSU1_ALT_RX, "hsu");
		gpio_request(HSU1_ALT_TX, "hsu");
		first = 0;
	}
	if (on) {
		lnw_gpio_set_alt(HSU1_RX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_TX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_CTS, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_RTS, LNW_GPIO);
		gpio_direction_input(HSU1_RX);
		gpio_direction_input(HSU1_TX);
		gpio_direction_input(HSU1_CTS);
		gpio_direction_input(HSU1_RTS);
		gpio_direction_input(HSU1_ALT_RX);
		gpio_direction_output(HSU1_ALT_TX, 0);
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_ALT_TX, LNW_ALT_1);
	} else {
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_ALT_TX, LNW_GPIO);
		gpio_direction_input(HSU1_ALT_RX);
		gpio_direction_input(HSU1_ALT_TX);
		gpio_direction_input(HSU1_RX);
		gpio_direction_output(HSU1_TX, 0);
		gpio_direction_input(HSU1_CTS);
		gpio_direction_output(HSU1_RTS, 0);
		lnw_gpio_set_alt(HSU1_RX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_TX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_CTS, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_RTS, LNW_ALT_2);
	}
}
EXPORT_SYMBOL_GPL(mfld_hsu_port1_switch);

void mfld_hsu_enable_wakeup(int index, struct device *dev, irq_handler_t wakeup)
{
	int ret;

	switch (index) {
	case 0:
		lnw_gpio_set_alt(HSU0_CTS, LNW_GPIO);
		ret = request_irq(gpio_to_irq(HSU0_CTS), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu0_cts_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu0: failed to register wakeup irq\n");

		/* turn off flow control */
		gpio_set_value(HSU0_RTS, 1);
		lnw_gpio_set_alt(HSU0_RTS, LNW_GPIO);
		udelay(100);
		break;
	case 1:
		lnw_gpio_set_alt(HSU1_RX, LNW_GPIO);
		udelay(100);
		ret = request_irq(gpio_to_irq(HSU1_RX), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu1_rx_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu1: failed to register wakeup irq\n");
		break;
	case 2:
		break;
	case 3:
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_GPIO);
		udelay(100);
		ret = request_irq(gpio_to_irq(HSU1_ALT_RX), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu1_rx_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu1: failed to register wakeup irq\n");
		break;
	default:
		dev_err(dev, "hsu: unknow hsu port\n");
	}
}
EXPORT_SYMBOL_GPL(mfld_hsu_enable_wakeup);

void mfld_hsu_disable_wakeup(int index, struct device *dev)
{
	switch (index) {
	case 0:
		free_irq(gpio_to_irq(HSU0_CTS), dev);
		lnw_gpio_set_alt(HSU0_CTS, LNW_ALT_1);
		lnw_gpio_set_alt(HSU0_RTS, LNW_ALT_1);
		break;
	case 1:
		free_irq(gpio_to_irq(HSU1_RX), dev);
		lnw_gpio_set_alt(HSU1_RX, LNW_ALT_1);
		break;
	case 2:
		break;
	case 3:
		free_irq(gpio_to_irq(HSU1_ALT_RX), dev);
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_ALT_1);
		break;
	default:
		dev_err(dev, "hsu: unknow hsu port\n");
	}
}
