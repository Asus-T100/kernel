/*
 * platform_hsu.c: hsu platform data initilization file
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
#include <linux/interrupt.h>
#include <asm/intel_mid_hsu.h>
#include "platform_hsu.h"

#ifdef CONFIG_X86_MRFLD
static struct mfld_hsu_info tng_hsu_info[] = {
	[0] = {
		.id = 0,
		.name = DEVICE_NAME_0,
		.cts_gpio = 124,
		.cts_alt = 1,
		.rts_gpio = 125,
		.rts_alt = 1,
		.wake_gpio = 124,
	},
	[1] = {
		.id = 1,
		.name = DEVICE_NAME_1,
		.cts_gpio = 128,
		.cts_alt = 1,
		.rts_gpio = 129,
		.rts_alt = 1,
		.wake_gpio = 128,
	},
	[2] = {
		.id = 2,
		.name = DEVICE_NAME_2,
		.cts_gpio = 132,
		.cts_alt = 1,
		.rts_gpio = 133,
		.rts_alt = 1,
		.wake_gpio = 134,
	},
};

static int __init tng_hsu_init(void)
{
	platform_hsu_info = tng_hsu_info;
	return 0;
}
arch_initcall(tng_hsu_init);
#endif

/* TODO: put non-ctp platform info here too or in its own device file */
#ifdef CONFIG_BOARD_CTP
static struct mfld_hsu_info ctp_hsu_info[] = {
	[0] = {
		.id = 0,
		.name = DEVICE_NAME_0,
		.wake_gpio = 42,
		.cts_gpio = 96+28,
		.cts_alt = 1,
		.rts_gpio = 96+29,
		.rts_alt = 1,
	},
	[1] = {
		.id = 1,
		.name = DEVICE_NAME_1,
		.wake_gpio = 64,
		.rx_gpio = 64,
		.rx_alt = 1,
		.tx_gpio = 65,
		.tx_alt = 1,
		.cts_gpio = 68,
		.cts_alt = 1,
		.rts_gpio = 66,
		.rts_alt = 2,
	},
	[2] = {
		.id = 2,
		.name = DEVICE_NAME_2,
		.wake_gpio = 67,
		.rx_gpio = 67,
		.rx_alt = 1,
	},
	[3] = {
		.id = 1,
		.name = DEVICE_NAME_3,
		.wake_gpio = 96+30,
		.rx_gpio = 96+30,
		.rx_alt = 1,
		.tx_gpio = 96+31,
		.tx_alt = 1,
		.cts_gpio = 96+33,
		.cts_alt = 1,
		.rts_gpio = 96+32,
		.rts_alt = 2,
	},

};

static int __init ctp_hsu_init(void)
{
	platform_hsu_info = ctp_hsu_info;
	return 0;
}
arch_initcall(ctp_hsu_init);
#endif
