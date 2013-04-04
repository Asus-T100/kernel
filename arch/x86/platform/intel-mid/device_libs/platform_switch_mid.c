/*
 * platform_switch_mid.c: switch_mid platform data initilization file
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
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_switch_mid.h"

static struct platform_device switch_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
};

static char *wm_codecs[] = {"wm5102"};

static int __init sfi_parse_check_wm(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_device_table_entry *pentry;
	int num, i, j;

	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb,
			struct sfi_device_table_entry);
	pentry = (struct sfi_device_table_entry *) sb->pentry;
	for (i = 0; i < num; i++, pentry++) {
		for (j = 0; j < ARRAY_SIZE(wm_codecs); j++) {
			if (!strncmp(pentry->name, wm_codecs[j], SFI_NAME_LEN))
				return 1;
		}
	}
	return 0;
}

static int __init switch_mid_init(void)
{
	int err;

	err = sfi_table_parse(SFI_SIG_DEVS, NULL, NULL, sfi_parse_check_wm);
	if (err) {
		pr_info("This board uses wm codec, not loading switch-mid\n");
		return 0;
	}

	err = platform_device_register(&switch_device);
	if (err < 0)
		pr_err("Fail to register switch-mid platform device.\n");
	return 0;
}

device_initcall(switch_mid_init);
