/*
 * blackbay_pr2.c: Intel Medfield platform BlackBay PR2 specific setup code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * Note:
 *
 */
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/pci.h>
#include <linux/sfi.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <asm/mrst.h>
#include <linux/input/lis3dh.h>
#include <linux/ms5607.h>
#include <linux/a1026.h>
#include <linux/i2c-gpio.h>
#include <linux/lnw_gpio.h>

static struct lis3dh_acc_platform_data lis3dh_pdata = {
	.poll_interval = 200,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 0,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.gpio_int1 = 60,
	.gpio_int2 = 61,
};

/* AUDIENCE es305 PLATFORM DATA */
#define AUDIENCE_WAKEUP_GPIO               "audience-wakeup"
#define AUDIENCE_RESET_GPIO                 "audience-reset"
/* FIXME: Although a1026_platform_data has defined gpio number, but don't know
 * why add reset/wakeup function in the platform_data structure.
 * Currently set the gpio a fixed number, after changed driver code,
 * will remove below global define */
#define A1026_WAKEUP_GPIO	159
#define A1026_RESET_GPIO	111

static int audience_request_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		client->dev.platform_data;
	int ret;

	pr_debug("Audience: request ressource audience\n");
	if (!pdata)
		return -1;
	ret = gpio_request(pdata->gpio_a1026_wakeup, AUDIENCE_WAKEUP_GPIO);
	if (ret) {
		dev_err(&client->dev, "Request AUDIENCE WAKEUP GPIO %d fails %d\n",
			pdata->gpio_a1026_wakeup, ret);
		return -1;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_wakeup, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_wake;
	}

	ret = gpio_request(pdata->gpio_a1026_reset, AUDIENCE_RESET_GPIO);
	if (ret) {
		dev_err(&client->dev,
				"Request for Audience reset GPIO %d fails %d\n",
					pdata->gpio_a1026_reset, ret);
		goto err_wake;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_reset, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_reset;
	}
	if (mfld_board_id() == MFLD_BID_PR3 ||
			mfld_board_id() == MFLD_BID_PR3_PNP)
		sprintf(pdata->firmware_name, "%s", "vpimg_es305b.bin");
	else
		sprintf(pdata->firmware_name, "%s", "vpimg.bin");

	return 0;
err_reset:
	gpio_free(pdata->gpio_a1026_reset);
err_wake:
	gpio_free(pdata->gpio_a1026_wakeup);
	return -1;
}

static void audience_free_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		&client->dev.platform_data;

	gpio_free(pdata->gpio_a1026_wakeup);
	gpio_free(pdata->gpio_a1026_reset);
}

static void audience_wake_up(bool state)
{
	gpio_set_value(A1026_WAKEUP_GPIO, state);
	pr_debug("Audience: WAKE UP %d\n", state);
}

static void audience_reset(bool state)
{
	gpio_set_value(A1026_RESET_GPIO, state);
	pr_debug("Audience: RESET %d\n", state);
}

static struct a1026_platform_data mfld_audience_platform_data = {
	.gpio_a1026_wakeup	= A1026_WAKEUP_GPIO,
	.gpio_a1026_reset	= A1026_RESET_GPIO,
	.request_resources	= audience_request_resources,
	.wakeup			= audience_wake_up,
	.reset			= audience_reset,
};

static struct ms5607_platform_data baro_pdata = {
	.poll_interval = 100,
	.min_interval  = 0,
};

#define LTR502_GPIO 63
#define MPU_GPIO_PIN 56

static struct i2c_board_info pr2_i2c_bus5_devs[] = {
	{
		.type		= "accel",
		.irq		= 0xff,
		.addr		= 0x18,
		.platform_data	= &lis3dh_pdata,
	},
	{
		.type		= "compass",
		.irq		= 0xff,
		.addr		= 0x1E,
	},
	{
		.type		= "gyro",
		.irq		= MPU_GPIO_PIN,
		.addr		= 0x68,
	},
	{
		.type		= "baro",
		.irq		= 0xff,
		.addr		= 0x77,
		.platform_data	= &baro_pdata,
	},
	{
		.type		= "als",
		.irq		= LTR502_GPIO,
		.addr		= 0x1d,
	},
	{
		.type		= "audience_es305",
		.irq		= 0xff,
		.addr		= 0x3e,
		.platform_data	= &mfld_audience_platform_data,
	},
};

static void register_board_i2c_devs()
{
	i2c_register_board_info(5, pr2_i2c_bus5_devs,
				ARRAY_SIZE(pr2_i2c_bus5_devs));
}
static int __init platform_bkbpr2_subsys_init(void)
{
	switch (mfld_board_id()) {
	case MFLD_BID_PR2_PROTO:
	case MFLD_BID_PR2_PNP:
	case MFLD_BID_PR2_VOLUME:
	case MFLD_BID_PR3:
	case MFLD_BID_PR3_PNP:
		register_board_i2c_devs();
		return 0;
	default:
		return 1;
	}
}
device_initcall(platform_bkbpr2_subsys_init);

static int hdmi_i2c_workaround(void)
{
	int ret;
	struct platform_device *pdev;
	struct i2c_gpio_platform_data *pdata;

	/*
	 * Hard code a gpio controller platform device to take over
	 * the two gpio pins used to be controlled by i2c bus 3.
	 * This is to support HDMI EDID extension block read, which
	 * is not supported by the current i2c controller, so we use
	 * GPIO pin the simulate an i2c bus.
	 */
	pdev = platform_device_alloc("i2c-gpio", 8);
	if (!pdev) {
		pr_err("i2c-gpio: failed to alloc platform device\n");
		ret = -ENOMEM;
		goto out;
	}

	pdata = kzalloc(sizeof(struct i2c_gpio_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("i2c-gpio: failed to alloc platform data\n");
		kfree(pdev);
		ret = -ENOMEM;
		goto out;
	}
	pdata->scl_pin = 35 + 96;
	pdata->sda_pin = 36 + 96;
	pdata->sda_is_open_drain = 0;
	pdata->scl_is_open_drain = 0;
	pdev->dev.platform_data = pdata;

	platform_device_add(pdev);

	lnw_gpio_set_alt(pdata->sda_pin, LNW_GPIO);
	lnw_gpio_set_alt(pdata->scl_pin, LNW_GPIO);

out:
	return ret;
}
rootfs_initcall(hdmi_i2c_workaround);
