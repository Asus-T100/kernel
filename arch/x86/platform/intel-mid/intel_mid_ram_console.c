/*
 * Intel mid ram console support
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/persistent_ram.h>
#include <linux/bootmem.h>

#include "../../../../drivers/staging/android/ram_console.h"

#define SZ_2M                           0x00200000
#define SZ_16M                          0x01000000

/* Board files use the following if they are ok with 16M start defaults */
#define INTEL_MID_PERSISTENT_RAM_START_DEFAULT	SZ_16M
#define INTEL_MID_PERSISTENT_RAM_SIZE_DEFAULT	SZ_2M

#define INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT	SZ_2M

#ifdef CONFIG_X86_32
#define INTEL_PERSISTENT_RAM_MAX_MEM  (max_low_pfn << PAGE_SHIFT)
#else
#define INTEL_PERSISTENT_RAM_MAX_MEM  (1 << 28)
#endif

static struct ram_console_platform_data ram_console_pdata = {
	.bootinfo = "Intel Ram Console",
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.dev		= {
		.platform_data = &ram_console_pdata,
	},
};

struct persistent_ram_descriptor ram_console_desc = {
	.name = "ram_console",
	.size = INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT,
};

struct persistent_ram ram_console_ram = {
	.start = INTEL_MID_PERSISTENT_RAM_START_DEFAULT,
	.size = INTEL_MID_PERSISTENT_RAM_SIZE_DEFAULT,
	.num_descs = 1,
	.descs = &ram_console_desc,
};

static __initdata bool intel_mid_ramconsole_inited;

/**
 * intel_mid_ram_console_register() - device_initcall to register ramconsole device
 */
static int __init intel_mid_ram_console_register(void)
{
	int ret;

	if (!intel_mid_ramconsole_inited)
		return -ENODEV;

	ret = platform_device_register(&ram_console_device);
	if (ret) {
		pr_err("%s: unable to register ram console device:", __func__);
		pr_err("start=0x%08x", (u32)ram_console_ram.start);
		pr_err("size=0x%08x", (u32)ram_console_ram.size);
		pr_err("ret=%d\n", ret);
	}

	return ret;
}
device_initcall(intel_mid_ram_console_register);

void __init ram_console_reserve_memory(void)
{
	int ret;
	phys_addr_t mem;
	size_t size;

	size = INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);

	mem = memblock_find_in_range(0, INTEL_PERSISTENT_RAM_MAX_MEM,
				     size, PAGE_SIZE);
	if (!mem) {
		pr_err("Cannot find memblock range for ram_console\n");
		return;
	}

	ram_console_ram.start = mem;
	ram_console_ram.size = size;

	ret = persistent_ram_early_init(&ram_console_ram);
	if (!ret)
		intel_mid_ramconsole_inited = true;
}

