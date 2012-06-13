/*
 * platform_msic_audio.c: MSIC audio platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Dharageswari R <dharageswari.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/i2c.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_mrfld_audio.h"

void *mrfld_audio_platform_data(void *info)
{
	int bus, ret;
	struct i2c_board_info i2c_info;
	struct platform_device *pdev;

	pr_debug("in mrfld_audio_platform_data\n");

	pdev = platform_device_alloc("sst-platform", -1);
	if (!pdev) {
		pr_err("failed to allocate audio platform device\n");
		return NULL;
	}
	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}
	/*FIXME: remove when using the POR codec for merrifield*/
	memset(&i2c_info, 0, sizeof(i2c_info));
	bus = 1;
	strncpy(i2c_info.type, "cs42l73", 16);
	i2c_info.irq = 0xff;
	i2c_info.addr = 0x4a;
	i2c_register_board_info(bus, &i2c_info, 1);
	return NULL;
}
