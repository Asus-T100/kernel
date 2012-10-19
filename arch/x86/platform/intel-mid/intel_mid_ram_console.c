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
#include <linux/platform_data/ram_console.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/kmemleak.h>


#define SZ_2M                           0x00200000
#define SZ_16M                          0x01000000
#define SZ_20M                          0x01400000

/* Board files use the following if they are ok with 16M size defaults */
#define INTEL_MID_RAM_CONSOLE_START_DEFAULT	SZ_16M
#define INTEL_FTRACE_BUFFER_START_DEFAULT      SZ_20M

#define INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT	SZ_2M
#define INTEL_FTRACE_BUFFER_SIZE_DEFAULT       SZ_2M


#ifdef CONFIG_X86_32
#define INTEL_RAM_CONSOLE_MAX_MEM  (max_low_pfn << PAGE_SHIFT)
#else
#define INTEL_RAM_CONSOLE_MAX_MEM  (1 << 28)
#endif

static struct resource ram_console_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start  = INTEL_MID_RAM_CONSOLE_START_DEFAULT,
		.end    = INTEL_MID_RAM_CONSOLE_START_DEFAULT +
			  INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT - 1,
	},

	{
		.flags  = IORESOURCE_MEM,
		.start  = INTEL_FTRACE_BUFFER_START_DEFAULT,
		.end    = INTEL_FTRACE_BUFFER_START_DEFAULT +
			INTEL_FTRACE_BUFFER_SIZE_DEFAULT - 1,
	}

};

static struct ram_console_platform_data ram_console_pdata;

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
	.dev		= {
	.platform_data	= &ram_console_pdata,
	},
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
		pr_err("start=0x%08x", (u32)ram_console_resources[0].start);
		pr_err("end=0x%08x", (u32)ram_console_resources[0].end);
		pr_err("ret=%d\n", ret);
	}

	return ret;
}
device_initcall(intel_mid_ram_console_register);

void __init ram_consle_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;

	size = INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);

	mem = memblock_find_in_range(0, INTEL_RAM_CONSOLE_MAX_MEM,
							size, PAGE_SIZE);
	if (mem == MEMBLOCK_ERROR)
		panic("Cannot allocate\n");
	ram_console_resources[0].start = mem;
	ram_console_resources[0].end = mem + size - 1;
	memblock_x86_reserve_range(mem, mem + size, "ram_console");
	kmemleak_ignore(phys_to_virt(mem));

	size = INTEL_FTRACE_BUFFER_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);
	mem = memblock_find_in_range(0, INTEL_RAM_CONSOLE_MAX_MEM,
							size, PAGE_SIZE);
	if (mem == MEMBLOCK_ERROR)
		panic("Cannot allocate\n");
	ram_console_resources[1].start = mem;
	ram_console_resources[1].end = mem + size - 1;
	memblock_x86_reserve_range(mem, mem + size, "ftrace reserved memory");
	kmemleak_ignore(phys_to_virt(mem));

	intel_mid_ramconsole_inited = true;
}

