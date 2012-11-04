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
#include <asm/platform_sst_audio.h>
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

	pr_debug("in mrfld_audio_platform_data\n");

	ret = add_sst_platform_device();
	if (ret < 0)
		return NULL;

	/*FIXME: remove when using the POR codec for merrifield*/
	memset(&i2c_info, 0, sizeof(i2c_info));
	bus = 1;
	strncpy(i2c_info.type, "lm49453", 16);
	i2c_info.irq = 0xff;
	i2c_info.addr = 0x1a;
	i2c_register_board_info(bus, &i2c_info, 1);
	return NULL;
}
