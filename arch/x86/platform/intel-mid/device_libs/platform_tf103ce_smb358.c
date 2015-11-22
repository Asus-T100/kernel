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
#if 0
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.irq_gpio			= SMB347_IRQ_GPIO,
#endif
	.irq_gpio			= -1,
	.inok_gpio			= -1,
#if defined(CONFIG_TF103CE)
        .gp_sdio_2_clk                  = 132,
#endif
};


static struct i2c_client *i2c_client;
static struct i2c_adapter *i2c_adap;

static struct i2c_board_info smb358_i2c_board_info = {
	.type          = "smb345",
	.flags         = 0x00,
	.addr          = 0x6a,
	.platform_data = &smb347_pdata,
	.archdata      = NULL,
	.irq           = -1,
};

#define	SMB358_I2C_ADAPTER	(1)

static int __init smb358_platform_init(void) {

       i2c_adap = i2c_get_adapter(SMB358_I2C_ADAPTER);
       if (!i2c_adap)  {
		printk("[%s] Cannot get i2c adapter %d\n", __func__, SMB358_I2C_ADAPTER);
		goto err;
       }


       i2c_client = i2c_new_device(i2c_adap, &smb358_i2c_board_info);
       if (!i2c_client) {
		printk("[%s] Unable to add I2C device for 0x%x\n", __func__,
		smb358_i2c_board_info.addr);
		goto err;
       }
err:    
        printk("=====================  %s \n",__func__);
        return 0;
}
module_init(smb358_platform_init);
