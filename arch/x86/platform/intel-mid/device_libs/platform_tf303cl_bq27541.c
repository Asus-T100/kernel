/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang Tom_shen@asus.com
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/bq27520_battery.h>
#include <linux/power/smb347-asus-charger.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include "platform_bq27520.h"

static struct i2c_board_info __initdata hpa_i2c_device = {
	I2C_BOARD_INFO("bq27541-battery", 0x55),
};

static struct bq27520_platform_data bq27520_pdata = {
	.bat_id_gpio = -1,
	.low_bat = -1,
	.adc_alert = -1,
};

static int __init bq27541_i2c_init(void) {
        int ret = 0;
        printk("===================  bq27541_i2c_init    =====================\n");

	/* acquired gpio for battery low pin */
	bq27520_pdata.low_bat = get_gpio_by_name("P_BAT_LBO");

	/* acquired gpio for battery adc alert pin */
	bq27520_pdata.adc_alert = get_gpio_by_name("ADC_Alert");

	hpa_i2c_device.platform_data = &bq27520_pdata;

	ret = i2c_register_board_info(1, &hpa_i2c_device, 1);
	if (ret < 0) {
		pr_err("%s: unable to register bq27541(%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

module_init(bq27541_i2c_init);

