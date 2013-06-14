/*
 * platform_btwilink.c: btwilink platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/lnw_gpio.h>
#include <linux/ti_wilink_st.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include "platform_btwilink.h"

/* Shared transport callbacks */
static int st_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int st_resume(struct platform_device *pdev)
{
	return 0;
}

static int st_chip_enable(struct kim_data_s *s)
{
	return 0;
}

static int st_chip_disable(struct kim_data_s *s)
{
	return 0;
}

static int st_chip_awake(struct kim_data_s *s)
{
	struct st_data_s *st = s->core_data;
	unsigned long flags;

	/* Tell PM runtime to power on the tty device and block S3 */
	spin_lock_irqsave(&st->pm_lock, flags);
	pm_runtime_get(st->tty_dev);
	wake_lock(&st->wake_lock);
	spin_unlock_irqrestore(&st->pm_lock, flags);

	return 0;
}

static int st_chip_asleep(struct kim_data_s *s)
{
	struct st_data_s *st = s->core_data;
	unsigned long flags;

	/* Tell PM runtime to release tty device and allow S3.
	 * PM runtime for the underlying tty device is modified from different
	 * sources (HSU driver, ST core, ST_ll, st kim). To release the device,
	 * The safe way would be to use pm_runtime_put_noidle() to avoid
	 * inconsistent runtime_usage ref counter ( < 0 ) and to schedule the
	 * idle() that will be handled by the hsu driver.
	 */
	spin_lock_irqsave(&st->pm_lock, flags);
	pm_runtime_put_noidle(st->tty_dev);
	if (!pm_runtime_suspended(st->tty_dev))
		__pm_runtime_idle(st->tty_dev, RPM_ASYNC);

	wake_unlock(&st->wake_lock);
	spin_unlock_irqrestore(&st->pm_lock, flags);

	return 0;
}

static struct ti_st_plat_data kim_pdata = {
	.nshutdown_gpio	= -1,/* BT, FM, GPS gpios */
	.flow_cntrl	= 1,		/* flow control flag */
	.suspend	= st_suspend,
	.resume		= st_resume,
	.chip_enable	= st_chip_enable,
	.chip_disable	= st_chip_disable,
	.chip_asleep	= st_chip_asleep,
	.chip_awake	= st_chip_awake,
};

static struct platform_device linux_kim_device = {
	.name           = "kim", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &kim_pdata,
};

/* BT WILINK related */
static int bt_enable(struct kim_data_s *s)
{
	return 0;
}
static int bt_disable(struct kim_data_s *s)
{
	return 0;
}

static struct ti_st_plat_data bt_pdata = {
	.chip_enable    = bt_enable,
	.chip_disable   = bt_disable,
};

static struct platform_device linux_bt_device = {
	.name           = "btwilink", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &bt_pdata,
};
static int __init bluetooth_init(void)
{
	unsigned int UART_index;
	long unsigned int UART_baud_rate;
	int error_reg;

	/* KIM INIT */
	/* Get the GPIO number from the SFI table
	   if FM gpio is not provided then BT-reset line is
	   also used to enable FM
	*/
	kim_pdata.nshutdown_gpio = get_gpio_by_name("BT-reset");
	if (kim_pdata.nshutdown_gpio == -1)
		return -ENODEV;

	/* Get Share Transport uart settings */
	/* TODO : add SFI table parsing and one SFI entry for this settings */
	UART_index = UART_PORT_INDEX;
	UART_baud_rate = UART_BAUD_RATE;
	/* Share Transport uart settings */
	sprintf((char *)kim_pdata.dev_name, "/dev/ttyMFD%u", UART_index);
	kim_pdata.baud_rate = UART_baud_rate;

	pr_info("%s: Setting platform_data with UART device name:%s and "
			"UART baud rate:%lu.\n",
			__func__, kim_pdata.dev_name, kim_pdata.baud_rate);

	error_reg = platform_device_register(&linux_kim_device);
	if (error_reg < 0) {
		pr_err("platform_device_register for kim failed\n");
		goto exit_on_error;
	}

	/* BT WILINK INIT */
	error_reg = platform_device_register(&linux_bt_device);
	if (error_reg < 0)
		pr_err("platform_device_register for btwilink failed\n");
exit_on_error:
	return error_reg;

}
device_initcall(bluetooth_init);
