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
#include <linux/atmel_mxt224.h>
#include <linux/a1026.h>

static u8 mxt_valid_interrupt(void)
{
	return 1;
}

static void mxt_init_platform_hw(void)
{
	/* maXTouch wants 40mSec minimum after reset to get organized */
	/*
	gpio_set_value(mxt_reset_gpio, 1);
	msleep(40);
	*/
}

static void mxt_exit_platform_hw(void)
{
	/*
	printk(KERN_INFO "In %s.", __func__);
	gpio_set_value(mxt_reset_gpio, 0);
	gpio_set_value(mxt_intr_gpio, 0);
	*/
}

static struct mxt_platform_data mxt_pdata = {
	.numtouch       = 2,
	.init_platform_hw = &mxt_init_platform_hw,
	.exit_platform_hw = &mxt_exit_platform_hw,
	.max_x          = 1023,
	.max_y          = 975,
	.orientation    = MXT_MSGB_T9_ORIENT_HORZ_FLIP,
	.valid_interrupt = &mxt_valid_interrupt,
	.reset          = 129,
	.irq            = 62,
};

static struct i2c_board_info pr2_i2c_bus0_devs[] = {
	{
		.type       = "atmel_mxt224",
		.addr       = 0x4A,
		.platform_data = &mxt_pdata,
	},
};

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

static struct gpio_keys_button gpio_button[] = {
	{KEY_VOLUMEUP,		32, 1, "vol_up",	EV_KEY, 0, 20},
	{KEY_VOLUMEDOWN,	31, 1, "vol_down",	EV_KEY, 0, 20},
	{KEY_CAMERA,		43, 1, "cam_capture",	EV_KEY, 0, 20, 0, 0, 0, 1},
	{KEY_CAMERA_FOCUS,	36, 1, "cam_focus",	EV_KEY, 0, 20},
};

static struct gpio_keys_platform_data mrst_gpio_keys = {
	.buttons        = gpio_button,
	.rep            = 1,
	.nbuttons       = 4,
};

static struct platform_device pb_device = {
	.name           = "gpio-keys",
	.id             = -1,
	.dev            = {
		.platform_data  = &mrst_gpio_keys,
	},
};

static void bkbpr2_gpio_keys_init()
{
	platform_device_register(&pb_device);
}

static void register_board_i2c_devs()
{
	i2c_register_board_info(0, pr2_i2c_bus0_devs,
				ARRAY_SIZE(pr2_i2c_bus0_devs));
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
		//bkbpr2_gpio_keys_init();
		return 0;
	default:
		return 1;
	}
}
device_initcall(platform_bkbpr2_subsys_init);

