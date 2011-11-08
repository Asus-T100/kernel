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
#include <linux/atmel_mxt224.h>

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
static struct i2c_board_info pr2_i2c_bus5_devs[] = {
	{
		.type		= "accel",
		.irq		= 0xff,
		.addr		= 0x18,
		.platform_data	= &lis3dh_pdata,
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
		register_board_i2c_devs();
		//bkbpr2_gpio_keys_init();
		return 0;
	default:
		return 1;
	}
}
device_initcall(platform_bkbpr2_subsys_init);

