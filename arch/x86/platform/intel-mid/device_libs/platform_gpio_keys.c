/*
 * platform_gpio_keys.c: gpio_keys platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/pnp.h>
#include <asm/intel_vlv2.h>
#include <asm/io_apic.h>
#include <asm/acpi.h>
#include <asm/hw_irq.h>
#include <asm/intel-mid.h>
#include "platform_gpio_keys.h"

/*
 * we will search these buttons in SFI GPIO table (by name)
 * and register them dynamically. Please add all possible
 * buttons here, we will shrink them if no GPIO found.
 */
static struct gpio_keys_button gpio_button[] = {
	{KEY_POWER,		-1, 1, "power_btn",	EV_KEY, 0, 3000},
	{KEY_PROG1,		-1, 1, "prog_btn1",	EV_KEY, 0, 20},
	{KEY_PROG2,		-1, 1, "prog_btn2",	EV_KEY, 0, 20},
	{SW_LID,		-1, 1, "lid_switch",	EV_SW,  0, 20},
	{KEY_VOLUMEUP,		-1, 1, "vol_up",	EV_KEY, 0, 20},
	{KEY_VOLUMEDOWN,	-1, 1, "vol_down",	EV_KEY, 0, 20},
	{KEY_CAMERA,		-1, 1, "camera_full",	EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,	-1, 1, "camera_half",	EV_KEY, 0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw1",	EV_SW,  0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw2",	EV_SW,  0, 20},
	{KEY_CAMERA,		-1, 1, "cam_capture",	EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,	-1, 1, "cam_focus",	EV_KEY, 0, 20},
	{KEY_MENU,              -1, 1, "fp_menu_key",   EV_KEY, 0, 20},
	{KEY_HOME,              -1, 1, "fp_home_key",   EV_KEY, 0, 20},
	{KEY_SEARCH,            -1, 1, "fp_search_key", EV_KEY, 0, 20},
	{KEY_BACK,              -1, 1, "fp_back_key",   EV_KEY, 0, 20},
	{KEY_VOLUMEUP,          -1, 1, "volume_up",     EV_KEY, 0, 20},
	{KEY_VOLUMEDOWN,        -1, 1, "volume_down",   EV_KEY, 0, 20},
	{KEY_MUTE,              -1, 1, "mute_enable",   EV_KEY, 0, 20},
	{KEY_CAMERA,            -1, 1, "camera0_sb1",   EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,      -1, 1, "camera0_sb2",   EV_KEY, 0, 20},
};

static struct gpio_keys_platform_data gpio_keys = {
	.buttons	= gpio_button,
	.rep		= 1,
	.nbuttons	= -1, /* will fill it after search */
};

static struct platform_device pb_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
	.dev		= {
		.platform_data	= &gpio_keys,
	},
};

/*
 * Shrink the non-existent buttons, register the gpio button
 * device if there is some
 */
static int __init pb_keys_init(void)
{
	struct gpio_keys_button *gb = gpio_button;
	int i, num, good = 0;

	num = sizeof(gpio_button) / sizeof(struct gpio_keys_button);
	for (i = 0; i < num; i++) {
		gb[i].gpio = get_gpio_by_name(gb[i].desc);
		pr_info("info[%2d]: name = %s, gpio = %d\n",
			 i, gb[i].desc, gb[i].gpio);
		if (gb[i].gpio == -1)
			continue;

		if (i != good)
			gb[good] = gb[i];
		good++;
	}

	if (good) {
		gpio_keys.nbuttons = good;
		return platform_device_register(&pb_device);
	}
	return 0;
}
late_initcall(pb_keys_init);

#ifdef	CONFIG_ACPI
static struct gpio_keys_button lesskey_button[] = {
	{KEY_POWER,		-1, 1, "power_btn",	EV_KEY, },
	{KEY_HOME,		-1, 1, "home_btn",	EV_KEY, },
	{KEY_VOLUMEUP,		-1, 1, "volume_up",	EV_KEY, },
	{KEY_VOLUMEDOWN,	-1, 1, "volume_down",	EV_KEY, },
	{KEY_RO,		-1, 1, "rotationlock",	EV_KEY, },
};

static struct gpio_keys_platform_data lesskey_keys = {
	.buttons	= lesskey_button,
	.rep		= 1,
	.nbuttons	= 0,
};

static struct platform_device lesskey_device = {
	.name		= "gpio-lesskey",
	.id		= -1,
	.dev		= {
		.platform_data	= &lesskey_keys,
	},
};

static int
lesskey_pnp_probe(struct pnp_dev *pdev, const struct pnp_device_id *id)
{
	int i, num, good = 0;
	struct gpio_keys_button *gb = lesskey_button;
	struct acpi_gpio_info info;

	num = sizeof(lesskey_button) / sizeof(lesskey_button[0]);
	for (i = 0; i < num; i++) {
		gb[i].gpio = acpi_get_gpio_by_index(&pdev->dev, i, &info);
		pr_info("lesskey [%2d]: name = %s, gpio = %d\n",
			 i, gb[i].desc, gb[i].gpio);
		if (gb[i].gpio < 0)
			continue;
		if (i != good)
			gb[good] = gb[i];
		good++;
	}

	if (good) {
		lesskey_keys.nbuttons = good;
		return platform_device_register(&lesskey_device);
	}

	return 0;
}

static const struct pnp_device_id lesskey_pnp_match[] = {
	{ "INTCFD9", 0},
	{ }
};
MODULE_DEVICE_TABLE(pnp, lesskey_pnp_match);

static struct pnp_driver lesskey_pnp_driver = {
	.name		= "lesskey",
	.id_table	= lesskey_pnp_match,
	.probe          = lesskey_pnp_probe,
};

static int __init lesskey_init(void)
{
	return pnp_register_driver(&lesskey_pnp_driver);
}

late_initcall(lesskey_init);
#endif
