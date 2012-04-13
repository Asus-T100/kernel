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
#include "platform_mxt_ts.h"


/* Atmel mxt toucscreen platform setup*/
static int atmel_mxt_init_platform_hw(struct i2c_client *client)
{
	int rc;
	int reset_gpio, int_gpio;

	reset_gpio = TOUCH_RESET_GPIO;
	int_gpio = TOUCH_IRQ_GPIO;

	/* init interrupt gpio */
	rc = gpio_request(int_gpio, "mxt_ts_intr");
	if (rc < 0)
		return rc;

	rc = gpio_direction_input(int_gpio);
	if (rc < 0)
		goto err_int;

	/* init reset gpio */
	rc = gpio_request(reset_gpio, "mxt_ts_rst");
	if (rc < 0)
		goto err_int;

	rc = gpio_direction_output(reset_gpio, 1);
	if (rc < 0)
		goto err_reset;

	/* reset the chip */
	gpio_set_value(reset_gpio, 1);
	msleep(20);
	gpio_set_value(reset_gpio, 0);
	msleep(20);
	gpio_set_value(reset_gpio, 1);
	msleep(100);
	/*
	 * HACK: depending on which touchpanel is used the mxt1386 controller
	 * may be at i2c address 0x4d instead of the default 0x4c stated in SFI
	 * table. Probe for chip by sending a i2c message and wait for ACK.
	 * (writing 0x00 will prepare chip for reading chip family id)
	 */

	rc = i2c_smbus_write_byte_data(client, 0, 0);
	if (rc < 0) {
		/* retry */
		msleep(60);
		rc = i2c_smbus_write_byte_data(client, 0, 0);
		if (rc < 0) {
			if (client->addr == 0x4c)
				client->addr = 0x4d;
			else if (client->addr == 0x4d)
				client->addr = 0x4c;
		}
	}

	return 0;

err_reset:
	gpio_free(reset_gpio);
err_int:
	pr_err("mxt touchscreen: configuring reset or int gpio failed\n");
	gpio_free(int_gpio);

	return rc;
}

void *mxt_ts_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	static struct mxt_platform_data mxt_pdata = {
		.irqflags	= IRQF_TRIGGER_FALLING,
		.init_platform_hw = atmel_mxt_init_platform_hw,
	};

	i2c_info->irq = TOUCH_IRQ_GPIO + INTEL_MID_IRQ_OFFSET;
	return &mxt_pdata;
}
