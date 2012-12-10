/*
 * android_bootmode.c: Generic function to read the android platfrom boot mode
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Albin B (albin.bala.krishnan@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <asm/android_bootmode.h>

#define BOOTMODE_MAX_LEN	7
static int boot_mode;

/* This function returns the mode in which the platform has booted */
int read_boot_mode(void)
{
	return boot_mode;
}
EXPORT_SYMBOL_GPL(read_boot_mode);

/**
 * get_bootmode - androidboot.mode command line parameter parsing
 *
 * androidboot.mode command line parameter.
 * Use: read the androidboot.mode=charger/main/any other platfrom supported
 * os boot mode.
 * Based on bootmode input parameter, set into boot_mode variable. It will
 * be used for other functionalites based on the boot mode specific settings
 * in future.
 */
static int __init get_bootmode(char *bootmode)
{
	if (!bootmode)
		return -EINVAL;

	if (!strncmp(bootmode, "charger", BOOTMODE_MAX_LEN))
		boot_mode = BOOTMODE_CHARGER;
	else if (!strncmp(bootmode, "main", BOOTMODE_MAX_LEN))
		boot_mode = BOOTMODE_MAIN;
	else if (!strncmp(bootmode, "fota", BOOTMODE_MAX_LEN))
		boot_mode = BOOTMODE_FOTA;
	else
		boot_mode = BOOTMODE_UNKNOWN;

	return 0;
}
early_param("androidboot.mode", get_bootmode);
