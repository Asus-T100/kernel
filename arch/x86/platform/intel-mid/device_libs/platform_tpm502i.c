/*
 * platform_mxt_ts.c: mxt_ts platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include "platform_tpm502i.h"

static int tpm502i_init_platform_hw(struct i2c_client *client)
{
	int rc;
	rc = i2c_smbus_write_byte_data(client, 0, 0);
	if (rc < 0) {
		/* retry */
		msleep(60);
		rc = i2c_smbus_write_byte_data(client, 0, 0);
		if (rc < 0)
			client->addr = 0x57;
	}
	return 0;
}

void *tpm502i_platform_data(void *info)
{
	static struct mxt_platform_data mxt_pdata = {
	.init_platform_hw = tpm502i_init_platform_hw,
	};
	return &mxt_pdata;
}
