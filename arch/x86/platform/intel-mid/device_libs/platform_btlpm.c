/*
 * platform_btlpm: btlpm platform data initialization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/pm_runtime.h>
#include <asm/bcm_bt_lpm.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>

#define UART_PORT_NO 0 /* Bluetooth is using UART port number 0 */

#define LPM_ON

static void bcm_uart_wake(struct device *tty)
{
	pr_debug("%s: runtime get\n", __func__);
	/* Tell PM runtime to power on the tty device and block s0i3 */
	pm_runtime_get(tty);
}

static void bcm_uart_sleep(struct device *tty)
{
	pr_debug("%s: runtime put\n", __func__);
	/* Tell PM runtime to release tty device and allow s0i3 */
	pm_runtime_put(tty);
}

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = -EINVAL,
	.gpio_host_wake = -EINVAL,
	.int_host_wake = -EINVAL,
	.gpio_enable = -EINVAL,
	.port = UART_PORT_NO,
	.uart_enable = bcm_uart_wake,
	.uart_disable = bcm_uart_sleep,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

static int __init bluetooth_init(void)
{

	int error_reg;

	/* Get the GPIO numbers from SFI or ACPI table */

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		bcm_bt_lpm_pdata.gpio_enable = get_gpio_by_name("BT-reset");
		if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_enable)) {
			pr_err("%s: gpio %s not found\n", __func__, "BT-reset");
			return -ENODEV;
		}
	} else {
#ifndef CONFIG_ACPI
		bcm_bt_lpm_pdata.gpio_enable = 53;
		if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_enable)) {
			pr_err("%s: gpio %d not found\n", __func__,
						bcm_bt_lpm_pdata.gpio_enable);
			return -ENODEV;
		}
#endif
	}


#ifdef LPM_ON
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		bcm_bt_lpm_pdata.gpio_host_wake =
				get_gpio_by_name("bt_uart_enable");
		bcm_bt_lpm_pdata.int_host_wake =
				gpio_to_irq(bcm_bt_lpm_pdata.gpio_host_wake);

		bcm_bt_lpm_pdata.gpio_wake = get_gpio_by_name("bt_wakeup");

		if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_wake)) {
			pr_err("%s: gpio %s not found\n", __func__,
								"bt_wakeup");
			return -ENODEV;
		}
	} else {
		bcm_bt_lpm_pdata.gpio_host_wake = 17;
		pr_err("baytrail, hardcoding GPIO Host Wake to %d\n",
					bcm_bt_lpm_pdata.gpio_host_wake);
		bcm_bt_lpm_pdata.int_host_wake = 70;
#ifndef CONFIG_ACPI
		bcm_bt_lpm_pdata.gpio_wake = 52;
		if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_wake)) {
			pr_err("%s: gpio %d not found\n", __func__,
						bcm_bt_lpm_pdata.gpio_wake);
			return -ENODEV;
		}
#endif
	}

	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_host_wake)) {
		pr_err("%s: gpio %s not found\n", __func__, "bt_host_wake");
		return -ENODEV;
	}

	pr_debug("%s: gpio_wake %d, gpio_host_wake %d\n", __func__,
		bcm_bt_lpm_pdata.gpio_wake, bcm_bt_lpm_pdata.gpio_host_wake);
#endif

	error_reg = platform_device_register(&bcm_bt_lpm_device);
	if (error_reg < 0) {
		pr_err("%s: platform_device_register for %s failed\n",
					__func__, bcm_bt_lpm_device.name);
		return -ENODEV;
	}
	return 0;
}

device_initcall(bluetooth_init);
