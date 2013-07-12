
/*
 * platform_byt_ffrd8_charger.c: platform data initilization file
 *                               for baytrail ffrd8 charger
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power/smb347-charger.h>
#include <linux/power/max17042_battery.h>
#include "platform_smb347.h"
#include "platform_max17042.h"

static struct smb347_charger_platform_data *smb_pdata;
static struct max17042_platform_data *max_pdata;

static struct i2c_board_info __initdata smb_i2c_device = {
	I2C_BOARD_INFO("smb349", 0x35),
};

static struct i2c_board_info __initdata max_i2c_device = {
	I2C_BOARD_INFO("max17050", 0x36),
};

/* Provide WA to register charger & FG in BYT-FFRD8 */
static int __init byt_ffrd8_charger_i2c_init(void)
{
	int ret = 0;

	smb_pdata = smb347_platform_data(NULL);

	if (smb_pdata == NULL) {
		pr_err("%s: unable to get the charger platform data\n",
				__func__);
		return -ENODEV;
	}

	smb_i2c_device.platform_data = smb_pdata;
	max_pdata = max17042_platform_data(&max_i2c_device);

	if (max_pdata == NULL) {
		pr_err("%s: unable to get the fg platform data\n", __func__);
		return -ENODEV;
	}
	max_i2c_device.platform_data = max_pdata;

	ret = i2c_register_board_info(1, &smb_i2c_device, 1);
	if (ret < 0) {
		pr_err("%s: unable to register charger(%d)\n", __func__, ret);
		return ret;
	}

	ret = i2c_register_board_info(1, &max_i2c_device, 1);

	if (ret < 0) {
		pr_err("%s: unable to register FG(%d)\n", __func__, ret);
		return ret;
	}
	return ret;
}
module_init(byt_ffrd8_charger_i2c_init);

