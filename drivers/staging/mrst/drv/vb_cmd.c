/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define I2C_ADAPTER 0x02
#define I2C_ADDRESS 0x54

enum vb_panel_type {
	PANEL_IGZO = 0,
	PANEL_CGS
};

/*
#define NO_POWER_OFF
#define NO_DRIVER_IC_INIT
#define NO_DCDC_POWER_OFF
#define NO_DISPLAY_RESET
#define ENABLE_PANEL_READ
*/
#define ENABLE_DEEP_SLEEP

static u8 ls04x_mcap[] = { 0xb0, 0x00 };
static u8 ls04x_mcap_cgs[] = { 0xb0, 0x04 };
static u8 ls04x_device_code_read[] = { 0xbf };
static u8 ls04x_frame_memory_access_igzo[] = {
	0xb3, 0x00, 0xc0, 0x00,
	0x00, 0x00, 0x00};
static u8 ls04x_frame_memory_access_cgs[] = {
	0xb3, 0x00, 0x00, 0x22,
	0x00, 0x00};
static u8 ls04x_interface_id[] = {
	0xb4, 0x0c, 0x12};
static u8 ls04x_dsi_control1_igzo[] = {
	0xb6, 0x39, 0xb3};
static u8 ls04x_dsi_control1_cgs[] = {
	0xb6, 0x31, 0xb5};
static u8 ls04x_dsi_control2[] = { 0xb7, 0x00 };
static u8 ls04x_panel_pin_ctrl[] = {
	0xcb, 0x67, 0x26, 0xc0,
	0x19, 0x0e, 0x00, 0x00,
	0x00, 0x00, 0xc0, 0x00};
static u8 ls04x_panel_interface_ctrl_igzo[] = { 0xcc, 0x01 };
static u8 ls04x_panel_interface_ctrl_cgs[] = { 0xcc, 0x06 };
static u8 ls04x_display_setting1_igzo[] = {
	0xc1, 0x0c, 0x62, 0x40,
	0x52, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x02,
	0xc7, 0x06, 0x02, 0x08,
	0x09, 0x08, 0x09, 0x00,
	0x00, 0x00, 0x00, 0x62,
	0x30, 0x40, 0xa5, 0x0f,
	0x04, 0x00, 0x20, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00};
static u8 ls04x_display_setting1_cgs[] = {
	0xc1, 0x04, 0x64, 0x10,
	0x41, 0x00, 0x00, 0x8e,
	0x29, 0xef, 0xbd, 0xf7,
	0xde, 0x7b, 0xef, 0xbd,
	0xf7, 0xde, 0x7b, 0xc5,
	0x1c, 0x02, 0x86, 0x08,
	0x22, 0x22, 0x00, 0x20};
static u8 ls04x_display_setting2_igzo[] = {
	0xc2, 0x30, 0xf5, 0x00,
	0x0c, 0x0e, 0x00, 0x00};
static u8 ls04x_display_setting2_cgs[] = {
	0xc2, 0x20, 0xf5, 0x00,
	0x14, 0x08, 0x00};
static u8 ls04x_tpc_sync_ctrl[] = {
	0xc3, 0x00, 0x00, 0x00};
static u8 ls04x_source_timing_setting_igzo[] = {
	0xc4, 0x70, 0x00, 0x00,
	0x00, 0x43, 0x2d, 0x00,
	0x00, 0x00, 0x00, 0x03,
	0x2d, 0x00};
static u8 ls04x_source_timing_setting_cgs[] = {
	0xc4, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x09,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x09};
static u8 ls04x_gip_timing_setting[] = {
	0xc6, 0xb2, 0x13, 0xa2,
	0xb2, 0x13, 0xa2};
static u8 ls04x_ltps_timing_setting_cgs[] = {
	0xc6, 0xb2, 0x00, 0xb1,
	0x05, 0xa6, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0b, 0x21,
	0x10, 0xb2, 0x00, 0xb1,
	0x05, 0xa6, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0b, 0x21,
	0x10};
static u8 ls04x_gamma_a_setting_igzo[] = {
	0xc7, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_b_setting_igzo[] = {
	0xc8, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_c_setting_igzo[] = {
	0xc9, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_a_setting_cgs[] = {
	0xc7, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_gamma_b_setting_cgs[] = {
	0xc8, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_gamma_c_setting_cgs[] = {
	0xc9, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_backlight_setting1_pwr_save[] = {
	0xb8, 0x18, 0x80, 0x18,
	0x18, 0xcf, 0x1f, 0x00,
	0x0c, 0x10, 0x5c, 0x10,
	0xac, 0x10, 0x0c, 0x10,
	0xda, 0x6d, 0xff, 0xff,
	0x10, 0x67, 0x89, 0xaf,
	0xd6, 0xff};
static u8 ls04x_backlight_setting2_pwr_save[] = {
	0xb9, 0x0f, 0x18, 0x04,
	0x40, 0x9f, 0x1f, 0x80};
static u8 ls04x_backlight_setting4_pwr_save[] = {
	0xba, 0x0f, 0x18, 0x04,
	0x40, 0x9f, 0x1f, 0xd7};
static u8 ls04x_backlight_setting6_igzo[] = {
	0xce, 0x00, 0x0f, 0x08,
	0x01, 0x00, 0x00, 0x00};
static u8 ls04x_power_setting_igzo[] = {
	0xd0, 0x00, 0x10, 0x19,
	0x18, 0x99, 0x99, 0x18,
	0x00, 0x89, 0x01, 0xbb,
	0x0c, 0x8f, 0x0e, 0x21,
	0x20};
static u8 ls04x_power_setting_cgs[] = {
	0xd0, 0x10, 0x4c, 0x18,
	0xcc, 0xda, 0x5a, 0x01,
	0x8a, 0x01, 0xbb, 0x58,
	0x4a};
static u8 ls04x_power_setting_int_pwr1_igzo[] = { 0xd2, 0x9c };
static u8 ls04x_power_setting_int_pwr1_cgs[] = { 0xd2, 0xb8 };
static u8 ls04x_power_setting_int_pwr2_igzo[] = {
	0xd3, 0x1b, 0xb3, 0xbb,
	0xbb, 0x33, 0x33, 0x33,
	0x33, 0x55, 0x01, 0x00,
	0xa0, 0xa8, 0xa0, 0x07,
	0xc7, 0xb7, 0x33, 0xa2,
	0x73, 0xc7, 0x00, 0x00,
	0x00};
static u8 ls04x_power_setting_int_pwr2_cgs[] = {
	0xd3, 0x1a, 0xb3, 0xbb,
	0xff, 0x77, 0x33, 0x33,
	0x33, 0x00, 0x01, 0x00,
	0xa0, 0x38, 0xa0, 0x00,
	0xdb, 0xb7, 0x33, 0xa2,
	0x72, 0xdb};
static u8 ls04x_vcom_setting[] = {
	0xd5, 0x06, 0x00, 0x00,
	0x01, 0x2b, 0x01, 0x2b};
static u8 ls04x_sleep_out_nvm_load_setting[] = { 0xd6, 0x01 };
static u8 ls04x_sequencer_timing_power_on_igzo[] = {
	0xd7, 0x44, 0x01, 0xff,
	0xff, 0x3f, 0xfc, 0x51,
	0x9d, 0x71, 0xf0, 0x0f,
	0x00, 0xe0, 0xff, 0x01,
	0xf0, 0x03, 0x00, 0x1e,
	0x00, 0x08, 0x94, 0x40,
	0xf0, 0x73, 0x7f, 0x78,
	0x08, 0xc0, 0x07, 0x0a,
	0x55, 0x15, 0x28, 0x54,
	0x55, 0xa0, 0xf0, 0xff,
	0x00, 0x40, 0x55, 0x05,
	0x00, 0x20, 0x20, 0x01};
static u8 ls04x_sequencer_timing_power_on_cgs[] = {
	0xd7, 0x20, 0x80, 0xfc,
	0xff, 0x7f, 0x22, 0xa2,
	0xa2, 0x80, 0x0a, 0xf0,
	0x60, 0x7e, 0x00, 0x3c,
	0x18, 0x40, 0x05, 0x7e,
	0x00, 0x00, 0x00};
static u8 ls04x_low_power_function[] = {
	0xd9, 0x09, 0x68, 0x4f,
	0x07, 0x00, 0x10, 0x00,
	0xc0, 0x00, 0x76, 0x33,
	0x33, 0x00, 0xf0, 0x33,
	0x33};
static u8 ls04x_panel_sync_out1[] = {
	0xec, 0x01, 0x40};
static u8 ls04x_panel_sync_out2[] = {
	0xed, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 ls04x_panel_sync_out3[] = { 0xee, 0x00 };
static u8 ls04x_panel_sync_out4[] = {
	0xef, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 ls04x_set_column_addr[] = {
	0x2a, 0x00, 0x00, 0x02,
	0xcf};
static u8 ls04x_set_page_addr[] = {
	0x2b, 0x00, 0x00, 0x04,
	0xff};
static u8 ls04x_ledpwm_on[] = { 0x53, 0x2c };
static u8 ls04x_cabc_on[] = { 0x55, 0x01 };
static u8 ls04x_deep_standby[] = { 0xb1, 0x01 };

static u8 ir2e69_power_on_seq[][2] = {
	{0x03, 0x01},
	{0,      20},
	{0x27, 0xe8},
	{0x03, 0x83},
	{0,      20},
	{0x28, 0x40},
	{0x2b, 0x01},
	{0x05, 0x0d},
	{0x06, 0x01},
	{0x25, 0x20},
	{0x0a, 0xc8},
	{0x0b, 0xc8},
	{0xdc, 0x3b},
	{0xee, 0x00},
	{0xf1, 0x00},
	{0xda, 0x30},
	{0xd8, 0x00} };
static u8 ir2e69_bias_on_seq[][2] = {
	{0x28, 0x40},
	{0,       1},
	{0x2b, 0x01},
	{0,       1} };
static u8 ir2e69_bias_off_seq[][2] = {
	{0x2b, 0x00},
	{0,       1},
	{0x28, 0x00},
	{0,       1} };
static u8 ir2e69_standby_seq[][2] = {
	{0x03, 0x00} };
static u8 ir2e69_normal_seq[][2] = {
	{0x03, 0x83} };
static u8 ls04x_reset_low[] = {0xd8, 0x00};
static u8 ls04x_reset_high[] = {0xd8, 0x10};

static struct i2c_board_info dcdc_board_info = {
	I2C_BOARD_INFO("vb_cmd_ir2e69", I2C_ADDRESS)
};

static struct i2c_device_id ir2e69_idtable[] = {
	{ "vb_cmd_ir2e69", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ir2e69_idtable);

static struct i2c_client *i2c_client;

static int mipi_reset_gpio = -1;

static struct attribute *ir2e69_attributes[] = {
	NULL
};

static struct attribute_group ir2e69_attribute_group = {
	.name = "vb_cmd_ir2e69",
	.attrs = ir2e69_attributes
};

static void ir2e69_reset();
static void vb_cmd_power_on_reset();
static void vb_cmd_panel_reset();

static int ir2e69_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int res;

	res = sysfs_create_group(&client->dev.kobj, &ir2e69_attribute_group);
	if (res) {
		dev_err(&client->dev, "creating sysfs entry failed\n");
		goto sysfs_error;
	}
	dev_info(&client->dev, "chip found\n");

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);

sysfs_error:
	return res;
}

static int __devexit ir2e69_remove(struct i2c_client *client)
{
	pm_runtime_get_sync(&client->dev);
	sysfs_remove_group(&client->dev.kobj, &ir2e69_attribute_group);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	return 0;
}

#ifdef CONFIG_PM
static int ir2e69_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return 0;
}

static int ir2e69_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return 0;
}

static const struct dev_pm_ops ir2e69_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ir2e69_suspend, ir2e69_resume)
};

#define IR2E69_PM_OPS (&ir2e69_pm_ops)

#else	/* CONFIG_PM */

#define ir2e69_suspend NULL
#define ir2e69_resume NULL
#define IR2E69_PM_OPS NULL

#endif	/* CONFIG_PM */

static struct i2c_driver ir2e69_driver = {
	.driver = {
		.name	= "vb_cmd_ir2e69",
		.owner	= THIS_MODULE,
		.pm	= &ir2e69_pm_ops
	},

	.id_table	= ir2e69_idtable,
	.probe		= ir2e69_probe,
	.remove		= __devexit_p(ir2e69_remove),
};

static void ir2e69_set_gpio(int value)
{
	PSB_DEBUG_ENTRY(": %d\n", value);
	if (mipi_reset_gpio == -1) {
		DRM_ERROR("mipi_reset_gpio is not correctly set\n");
		return;
	}

	gpio_set_value(mipi_reset_gpio, value);
}

static void ir2e69_register(void)
{
	struct i2c_adapter *adapter;
	int ret;

	adapter = i2c_get_adapter(I2C_ADAPTER);
	i2c_client = i2c_new_device(adapter, &dcdc_board_info);

	mipi_reset_gpio = get_gpio_by_name("mipi-reset");
	if (mipi_reset_gpio < 0) {
		DRM_ERROR("can't get mipi-reset gpio pin\n");
		mipi_reset_gpio = -1;
		return;
	}
	ret = gpio_request(mipi_reset_gpio, "mipi_display");
	if (ret < 0) {
		DRM_ERROR("failed to request gpio %d\n", mipi_reset_gpio);
		mipi_reset_gpio = -1;
		return;
	}
	gpio_direction_output(mipi_reset_gpio, 0);
}

static void ir2e69_unregister(void)
{
	if (i2c_client != NULL)
		i2c_unregister_device(i2c_client);

	if (mipi_reset_gpio != -1)
		gpio_free(mipi_reset_gpio);
}

static int ir2e69_send_sequence(u8 data[][2], int count)
{
	int r = 0;
	int i;

	for (i = 0; i < count && r >= 0; i++) {
		if (data[i][0] != 0) {
			r = i2c_master_send(i2c_client,
					data[i], sizeof(data[i]));
			if (r < 0)
				dev_err(&i2c_client->dev, "%d: error %d\n",
					i, r);
		} else
			usleep_range(data[i][1], data[i][1] * 3 / 2);
	}
	return r;
}

static void ir2e69_reset()
{
	int i;
	int r;

	PSB_DEBUG_ENTRY("\n");

	ir2e69_set_gpio(0);
	mdelay(20);
	ir2e69_set_gpio(1);
	mdelay(20);
	for (i = 0; i < ARRAY_SIZE(ir2e69_power_on_seq); i++) {
		if (ir2e69_power_on_seq[i][0] != 0) {
			r = i2c_master_send(i2c_client,
					    ir2e69_power_on_seq[i],
					    sizeof(ir2e69_power_on_seq[i]));
			if (r < 0)
				dev_err(&i2c_client->dev, "%d: error %d\n",
					i, r);
		} else
			mdelay(ir2e69_power_on_seq[i][1]);
	}
}

static void ir2e69_power_off()
{
	PSB_DEBUG_ENTRY("\n");
	ir2e69_send_sequence(ir2e69_bias_off_seq,
			ARRAY_SIZE(ir2e69_bias_off_seq));
	if (ir2e69_send_sequence(ir2e69_standby_seq,
				 ARRAY_SIZE(ir2e69_standby_seq)) < 0)
		mdelay(200);
	else
		mdelay(20);
#ifndef NO_DCDC_POWER_OFF
	ir2e69_set_gpio(0);
#endif
}

static void ir2e69_power_on()
{
	PSB_DEBUG_ENTRY("\n");
	ir2e69_send_sequence(ir2e69_normal_seq,
			ARRAY_SIZE(ir2e69_normal_seq));
	ir2e69_send_sequence(ir2e69_bias_on_seq,
			ARRAY_SIZE(ir2e69_bias_on_seq));
}

static void ls04x_reset()
{
	PSB_DEBUG_ENTRY("\n");
#ifndef NO_DISPLAY_RESET
	i2c_master_send(i2c_client, ls04x_reset_low, sizeof(ls04x_reset_low));
	mdelay(10);
#endif
	i2c_master_send(i2c_client, ls04x_reset_high, sizeof(ls04x_reset_high));
	mdelay(20);
}

static int ls04x_igzo_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;

	DRM_INFO("IGZO panel detected!\n");

#ifndef NO_DRIVER_IC_INIT
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender)
		return -EINVAL;
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_send_gen_short_lp(sender,
					ls04x_mcap[0],
					ls04x_mcap[1], 2,
					MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_frame_memory_access_igzo,
				   sizeof(ls04x_frame_memory_access_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_interface_id,
				   sizeof(ls04x_interface_id),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_dsi_control1_igzo,
				   sizeof(ls04x_dsi_control1_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_dsi_control2[0],
				    ls04x_dsi_control2[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_panel_pin_ctrl,
				   sizeof(ls04x_panel_pin_ctrl),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_panel_interface_ctrl_igzo[0],
				    ls04x_panel_interface_ctrl_igzo[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_display_setting1_igzo,
				   sizeof(ls04x_display_setting1_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_display_setting2_igzo,
				   sizeof(ls04x_display_setting2_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_tpc_sync_ctrl,
				   sizeof(ls04x_tpc_sync_ctrl),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_source_timing_setting_igzo,
				   sizeof(ls04x_source_timing_setting_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gip_timing_setting,
				   sizeof(ls04x_gip_timing_setting),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_backlight_setting1_pwr_save,
				   sizeof(ls04x_backlight_setting1_pwr_save),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_backlight_setting2_pwr_save,
				   sizeof(ls04x_backlight_setting2_pwr_save),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_backlight_setting4_pwr_save,
				   sizeof(ls04x_backlight_setting4_pwr_save),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_backlight_setting6_igzo,
				   sizeof(ls04x_backlight_setting6_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_a_setting_igzo,
				   sizeof(ls04x_gamma_a_setting_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_b_setting_igzo,
				   sizeof(ls04x_gamma_b_setting_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_c_setting_igzo,
				   sizeof(ls04x_gamma_c_setting_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_power_setting_igzo,
				   sizeof(ls04x_power_setting_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_power_setting_int_pwr1_igzo[0],
				    ls04x_power_setting_int_pwr1_igzo[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_power_setting_int_pwr2_igzo,
				   sizeof(ls04x_power_setting_int_pwr2_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	/* comment vcom setting for igzo and use default setting
	* it is used to avoiding 1-hz panel flicker issue.
	*/
	/*
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_vcom_setting,
				   sizeof(ls04x_vcom_setting),
				   MDFLD_DSI_SEND_PACKAGE);
	*/
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_sleep_out_nvm_load_setting[0],
				    ls04x_sleep_out_nvm_load_setting[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_sequencer_timing_power_on_igzo,
				   sizeof(ls04x_sequencer_timing_power_on_igzo),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_low_power_function,
				   sizeof(ls04x_low_power_function),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_panel_sync_out1,
				   sizeof(ls04x_panel_sync_out1),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_panel_sync_out2,
				   sizeof(ls04x_panel_sync_out2),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_panel_sync_out3[0],
				    ls04x_panel_sync_out3[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_panel_sync_out4,
				   sizeof(ls04x_panel_sync_out4),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_display, 0x2c, 1,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_cabc, 0x1, 1,
				    MDFLD_DSI_SEND_PACKAGE);

	/* Send column and page addr before write mem start,
	* as the default setting causes partial screen messy.
	*/
	mdfld_dsi_send_mcs_long_lp(sender,
				   ls04x_set_column_addr,
				   sizeof(ls04x_set_column_addr),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_long_lp(sender,
				   ls04x_set_page_addr,
				   sizeof(ls04x_set_page_addr),
				   MDFLD_DSI_SEND_PACKAGE);
#endif
	return 0;
}

static int ls04x_cgs_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;
	u8 data[16];

	DRM_INFO("CGS panel detected!\n");

#ifndef NO_DRIVER_IC_INIT
	memset(data, 0, sizeof(data));
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender)
		return -EINVAL;
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_send_gen_short_lp(sender,
					ls04x_mcap_cgs[0],
					ls04x_mcap_cgs[1], 2,
					MDFLD_DSI_SEND_PACKAGE);

	r = mdfld_dsi_read_gen_lp(sender,
				  ls04x_device_code_read[0], 0, 1,
				  data, 5);
	PSB_DEBUG_GENERAL("device code: %d %02x %02x %02x %02x %02x\n",
		r, data[0], data[1], data[2], data[3], data[4]);

	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_frame_memory_access_cgs,
				   sizeof(ls04x_frame_memory_access_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_interface_id,
				   sizeof(ls04x_interface_id),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_dsi_control1_cgs,
				   sizeof(ls04x_dsi_control1_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_panel_interface_ctrl_cgs[0],
				    ls04x_panel_interface_ctrl_cgs[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_display_setting1_cgs,
				   sizeof(ls04x_display_setting1_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_display_setting2_cgs,
				   sizeof(ls04x_display_setting2_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_tpc_sync_ctrl,
				   sizeof(ls04x_tpc_sync_ctrl),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_source_timing_setting_cgs,
				   sizeof(ls04x_source_timing_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_ltps_timing_setting_cgs,
				   sizeof(ls04x_ltps_timing_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_a_setting_cgs,
				   sizeof(ls04x_gamma_a_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_b_setting_cgs,
				   sizeof(ls04x_gamma_b_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_gamma_c_setting_cgs,
				   sizeof(ls04x_gamma_c_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_power_setting_cgs,
				   sizeof(ls04x_power_setting_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_power_setting_int_pwr1_cgs[0],
				    ls04x_power_setting_int_pwr1_cgs[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_power_setting_int_pwr2_cgs,
				   sizeof(ls04x_power_setting_int_pwr2_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_vcom_setting,
				   sizeof(ls04x_vcom_setting),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_short_lp(sender,
				    ls04x_sleep_out_nvm_load_setting[0],
				    ls04x_sleep_out_nvm_load_setting[1], 2,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender,
				   ls04x_sequencer_timing_power_on_cgs,
				   sizeof(ls04x_sequencer_timing_power_on_cgs),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_display, 0x2c, 1,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_cabc, 0x1, 1,
				    MDFLD_DSI_SEND_PACKAGE);

	/* Send column and page addr before write mem start,
	* as the default setting causes partial screen messy.
	*/
	mdfld_dsi_send_mcs_long_lp(sender,
				   ls04x_set_column_addr,
				   sizeof(ls04x_set_column_addr),
				   MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_long_lp(sender,
				   ls04x_set_page_addr,
				   sizeof(ls04x_set_page_addr),
				   MDFLD_DSI_SEND_PACKAGE);
#endif
	return 0;
}

static int ls04x_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;
	u8 data[16];

	PSB_DEBUG_ENTRY("\n");

	memset(data, 0, sizeof(data));
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender)
		return -EINVAL;
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_send_gen_short_lp(sender,
					ls04x_mcap[0],
					ls04x_mcap[1], 2,
					MDFLD_DSI_SEND_PACKAGE);

#ifdef ENABLE_PANEL_READ
	r = mdfld_dsi_read_gen_lp(sender,
				  ls04x_device_code_read[0], 0, 1, data, 5);
	PSB_DEBUG_GENERAL("device code read: %d %02x %02x %02x %02x %02x\n",
			  r, data[0], data[1], data[2], data[3], data[4]);
#endif

	if ((data[2] == 0x14) && (data[3] == 0x13))
		r = ls04x_igzo_drv_ic_init(dsi_config);
	else if ((data[2] == 0x34) && (data[3] == 0x15))
		r = ls04x_cgs_drv_ic_init(dsi_config);
	else
		DRM_INFO("unknown device code: %02x %02x\n", data[2], data[3]);

	return r;
}

static void ls04x_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x18;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x28;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->dbi_bw_ctrl = 820;
	hw_ctx->dphy_param = 0x160d3610;
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
	hw_ctx->mipi = TE_TRIGGER_GPIO_PIN | PASS_FROM_SPHY_TO_AFE;
	hw_ctx->mipi |= dsi_config->lane_config;
	hw_ctx->vgacntr = 0x80000000;
}

static struct drm_display_mode *ls04x_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->htotal = 920;
	mode->hdisplay = 720;
	mode->hsync_start = 816;
	mode->hsync_end = 824;
	mode->vtotal = 1300;
	mode->vdisplay = 1280;
	mode->vsync_start = 1294;
	mode->vsync_end = 1296;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void ls04x_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");
	if (pipe == 0) {
		pi->width_mm = PANEL_4DOT3_WIDTH;
		pi->height_mm = PANEL_4DOT3_HEIGHT;
	}
}

static int ls04x_cgs_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);

	if (drm_psb_enable_cabc) {
		u8 brightness[3] = {0, };

		brightness[0] = write_display_brightness;
		brightness[1] = 0;
		brightness[2] = (level * 255 / 100) & 0xff;
		mdfld_dsi_send_mcs_long_lp(sender,
				brightness,
				sizeof(brightness),
				MDFLD_DSI_SEND_PACKAGE);
	} else {
		u8 brightness[2];

		PSB_DEBUG_ENTRY("%d\n", level);
		brightness[0] = 0x0a;
		brightness[1] = level * 255 / 100;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
		brightness[0] = 0x0b;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
	}

	return 0;
}

static int ls04x_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);

	if (drm_psb_enable_cabc) {
		u8 brightness[3] = {0, };

		brightness[0] = write_display_brightness;
		brightness[1] = (level * 4095 / 100) >> 8;
		brightness[2] = (level * 4095 / 100) & 0xff;
		mdfld_dsi_send_mcs_long_lp(sender,
				brightness,
				sizeof(brightness),
				MDFLD_DSI_SEND_PACKAGE);
	} else {
		u8 brightness[2];

		PSB_DEBUG_ENTRY("%d\n", level);
		brightness[0] = 0x0a;
		brightness[1] = level * 255 / 100;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
		brightness[0] = 0x0b;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
	}

	return 0;
}

static void vb_cmd_panel_reset()
{
	PSB_DEBUG_ENTRY("\n");

	mdelay(10);
	mdelay(3);
}

static int vb_cmd_reset()
{
	PSB_DEBUG_ENTRY("\n");
#ifndef NO_POWER_OFF
	mdelay(10);
	ir2e69_reset();
	ls04x_reset();
	mdelay(3);
#endif
	return 0;
}

static int vb_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;
	u8 power = 0;

	ir2e69_power_on();
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	mdfld_dsi_send_mcs_short_lp(sender, set_tear_on, 0x00, 1,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, set_display_on, 0, 0,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, exit_sleep_mode, 0, 0,
				    MDFLD_DSI_SEND_PACKAGE);
	mdelay(100);

	if (drm_psb_enable_cabc) {
		mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_display, 0x2c, 1,
				MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, write_ctrl_cabc, 0x1, 1,
				MDFLD_DSI_SEND_PACKAGE);
	}
#ifdef ENABLE_PANEL_READ
	r = mdfld_dsi_get_power_mode(dsi_config, &power,
					MDFLD_DSI_LP_TRANSMISSION);
	PSB_DEBUG_GENERAL(": %d %x\n", r, power);
#endif
	return 0;
}

static int vb_cmd_power_off(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	u8 power = 0;


	PSB_DEBUG_ENTRY("\n");
	r = mdfld_dsi_send_mcs_short_lp(sender, set_display_off, 0, 0,
					MDFLD_DSI_SEND_PACKAGE);
	mdelay(16);
	r = mdfld_dsi_send_mcs_short_lp(sender, enter_sleep_mode, 0, 0,
					MDFLD_DSI_SEND_PACKAGE);
	mdelay(100);
#ifdef ENABLE_DEEP_SLEEP
	r = mdfld_dsi_send_gen_short_lp(sender,
					ls04x_deep_standby[0],
					ls04x_deep_standby[1],
					2, MDFLD_DSI_SEND_PACKAGE);
	mdelay(16);
#endif

	ir2e69_power_off();
	mdelay(15);

#ifndef NO_POWER_OFF
	/*
	r = mdfld_dsi_get_power_mode(dsi_config, &power,
					MDFLD_DSI_LP_TRANSMISSION);
	*/

	i2c_master_send(i2c_client, ls04x_reset_low, sizeof(ls04x_reset_low));
	mdelay(10);
#endif

	return 0;
}

static int vb_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		* FIXME: WA to detect the panel connection status, and need to
		* implement detection feature with get_power_mode DSI command.
		*/
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					       OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

#if 1
		struct drm_device *dev = dsi_config->dev;
		struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
		u32 dpll_val, device_ready_val;
		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		dsi_config->dsi_hw_context.panel_on = false;
		status = MDFLD_DSI_PANEL_CONNECTED;

		/* ------------ */
		struct mdfld_dsi_pkg_sender *sender
					= mdfld_dsi_get_pkg_sender(dsi_config);
		u8 data[5]; memset(data, 0, sizeof(data));
#ifdef ENABLE_PANEL_READ
		u8 r = mdfld_dsi_read_gen_lp(sender,
					     ls04x_device_code_read[0], 0, 1,
						data, 5);
		PSB_DEBUG_GENERAL("device code: %d %02x %02x %02x %02x %02x\n",
			r, data[0], data[1], data[2], data[3], data[4]);
#endif

		u8 power = 0;

		mdfld_dsi_send_mcs_short_lp(sender, exit_sleep_mode, 0, 0,
					    MDFLD_DSI_SEND_PACKAGE);
		mdelay(20);

#ifdef ENABLE_PANEL_READ
		r = mdfld_dsi_get_power_mode(dsi_config, &power,
					MDFLD_DSI_LP_TRANSMISSION);
		PSB_DEBUG_GENERAL(": %d %d %x\n", r, status, power);
#endif
		/* ------------ */

#else
		status = MDFLD_DSI_PANEL_DISCONNECTED;
		u8 power = 0;

		if (mdfld_dsi_get_power_mode(dsi_config, &power,
					     MDFLD_DSI_LP_TRANSMISSION) > 0) {
			if (power & 0x04)
				dsi_config->dsi_hw_context.panel_on = true;
			else
				dsi_config->dsi_hw_context.panel_on = false;
			status = MDFLD_DSI_PANEL_CONNECTED;
		}
		PSB_DEBUG_GENERAL(": %d %x\n", status, power);
		PSB_DEBUG_GENERAL("display connection state: %d, power: %x\n",
				status,
				power);
#endif

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;

}

static int vb_cmd_reboot(struct notifier_block *this, unsigned long code,
			 void *unused)
{
	ir2e69_power_off();
	return NOTIFY_DONE;
}

struct notifier_block vb_cmd_reboot_notifier_block = {
	.notifier_call = vb_cmd_reboot
};

void vb_igzo_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = ls04x_cmd_get_config_mode;
	p_funcs->get_panel_info = ls04x_cmd_get_panel_info;
	p_funcs->reset = vb_cmd_reset;
	p_funcs->drv_ic_init = ls04x_igzo_drv_ic_init;
	p_funcs->dsi_controller_init = ls04x_dsi_controller_init;
	p_funcs->detect = vb_cmd_detect;
	p_funcs->power_on = vb_cmd_power_on;
	p_funcs->power_off = vb_cmd_power_off;
	p_funcs->set_brightness = ls04x_cmd_set_brightness;
	ir2e69_register();
	register_reboot_notifier(&vb_cmd_reboot_notifier_block);
}

void vb_cgs_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = ls04x_cmd_get_config_mode;
	p_funcs->get_panel_info = ls04x_cmd_get_panel_info;
	p_funcs->reset = vb_cmd_reset;
	p_funcs->drv_ic_init = ls04x_cgs_drv_ic_init;
	p_funcs->dsi_controller_init = ls04x_dsi_controller_init;
	p_funcs->detect = vb_cmd_detect;
	p_funcs->power_on = vb_cmd_power_on;
	p_funcs->power_off = vb_cmd_power_off;
	p_funcs->set_brightness = ls04x_cgs_cmd_set_brightness;
	ir2e69_register();
	register_reboot_notifier(&vb_cmd_reboot_notifier_block);
}

void vb_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = ls04x_cmd_get_config_mode;
	p_funcs->get_panel_info = ls04x_cmd_get_panel_info;
	p_funcs->reset = vb_cmd_reset;
	p_funcs->drv_ic_init = ls04x_drv_ic_init;
	p_funcs->dsi_controller_init = ls04x_dsi_controller_init;
	p_funcs->detect = vb_cmd_detect;
	p_funcs->power_on = vb_cmd_power_on;
	p_funcs->power_off = vb_cmd_power_off;
	p_funcs->set_brightness = ls04x_cmd_set_brightness;
	ir2e69_register();
	register_reboot_notifier(&vb_cmd_reboot_notifier_block);
}

static int vb_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	enum vb_panel_type panel_type;

	DRM_INFO("%s\n", __func__);
	panel_type = platform_get_device_id(pdev)->driver_data;
	if (panel_type == PANEL_IGZO) {
		DRM_INFO("%s: IGZO panel detected\n", __func__);
		intel_mid_panel_register(vb_igzo_cmd_init);
	} else if (panel_type == PANEL_CGS) {
		DRM_INFO("%s: CGS panel detected\n", __func__);
		intel_mid_panel_register(vb_cgs_cmd_init);
	} else {
		DRM_ERROR("bad vb panel type %d\n", panel_type);
		return -EINVAL;
	}

	return 0;
}

static struct platform_device_id vb_panel_tbl[] = {
	{ "SHARP IGZO VKB", PANEL_IGZO },
	{ "SHARP CGS VKB",  PANEL_CGS },
	{ }
};

static struct platform_driver vb_lcd_driver = {
	.probe	= vb_lcd_probe,
	.driver	= {
		.name	= "vb_lcd",
		.owner	= THIS_MODULE,
	},
	.id_table = vb_panel_tbl
};

static int __init vb_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	return platform_driver_register(&vb_lcd_driver);
}

module_init(vb_lcd_init);
