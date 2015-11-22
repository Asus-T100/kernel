/*
 * platform_smb347.c: smb347 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power/smb347-asus-charger.h>
#include <asm/intel-mid.h>
#include "platform_tf303cl_smb347.h"

static struct i2c_board_info __initdata smb347_i2c_device = {
	I2C_BOARD_INFO("smb345", 0x6A),
};

static struct smb347_charger_platform_data smb347_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.max_charge_current		= 3360000,
	.max_charge_voltage		= 4200000,
	.otg_uvlo_voltage		= 3300000,
	.chip_temp_threshold		= 120,
	.soft_cold_temp_limit		= 5,
	.soft_hot_temp_limit		= 50,
	.hard_cold_temp_limit		= 5,
	.hard_hot_temp_limit		= 55,
	.suspend_on_hard_temp_limit	= true,
	.soft_temp_limit_compensation	= SMB347_SOFT_TEMP_COMPENSATE_CURRENT
					| SMB347_SOFT_TEMP_COMPENSATE_VOLTAGE,
	.charge_current_compensation	= 900000,
	.use_mains			= true,
	.irq_gpio			= -1,
	.inok_gpio			= -1,
};

static int __init smb347_i2c_init(void) {
        int ret = 0;
        printk("===================  smb347_i2c_init    =====================\n");

	/* acquired gpio for battery INOK pin */
	smb347_pdata.inok_gpio = 207;

	smb347_i2c_device.platform_data = &smb347_pdata;

	ret = i2c_register_board_info(1, &smb347_i2c_device, 1);
	if (ret < 0) {
		pr_err("%s: unable to register smb347(%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

module_init(smb347_i2c_init);
