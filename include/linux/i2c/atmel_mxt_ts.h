/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

struct mxt_platform_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u32 info_crc;
	u32 config_crc;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
	int gpio_reset;
	const char *cfg_name;
	bool regulator_en;
	int hardware_id;
	struct mxt_platform_info info;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
