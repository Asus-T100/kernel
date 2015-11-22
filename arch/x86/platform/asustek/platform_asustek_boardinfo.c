/* Copyright (c) 2013, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static struct platform_device asustek_boardinfo_device = {
	.name = "asustek_boardinfo",
	.id = -1,
};

void __init asustek_add_boardinfo_devices(void)
{
	printk("asustek_add_boardinfo_devices+\n");
	platform_device_register(&asustek_boardinfo_device);
	printk("asustek_add_boardinfo_devices+\n");
}

arch_initcall(asustek_add_boardinfo_devices);
//#endif
