/*
 * platform_bq24192.c: bq24192 platform data initilization file
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
#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/intel-mid.h>
#include <asm/spid.h>
#include <linux/usb/otg.h>

static struct i2c_board_info __initdata ug3105_i2c_device = {
	I2C_BOARD_INFO("ug31xx-gauge", 0x70),
	.flags         = 0x00,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};

static struct i2c_client *i2c_client;
static struct i2c_adapter *i2c_adap;

static struct i2c_board_info ug31xx_i2c_board_info = {
	.type          = "ug31xx-gauge",
	.flags         = 0x00,
	.addr          = 0x70,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};


#define	UG31XX_I2C_ADAPTER	(1)
#define	UG31XX_REGISTER_I2C

static int __init ug3105_platform_init(void)
{
        printk("===================== TOM __init ug3105_platform_init \n");

#ifdef	UG31XX_REGISTER_I2C
	i2c_adap = i2c_get_adapter(UG31XX_I2C_ADAPTER);
	if (!i2c_adap) {
		printk("[%s] Cannot get i2c adapter %d\n", __func__, UG31XX_I2C_ADAPTER);
		goto err;
	}


	i2c_client = i2c_new_device(i2c_adap, &ug31xx_i2c_board_info);
	if (!i2c_client) {
		printk("[%s] Unable to add I2C device for 0x%x\n", __func__, ug31xx_i2c_board_info.addr);
		goto err;
	}
#endif	///< end of UG31XX_REGISTER_I2C

err:        
	//return i2c_register_board_info(1, &ug3105_i2c_device, 1);
        return 0;
}
module_init(ug3105_platform_init);
