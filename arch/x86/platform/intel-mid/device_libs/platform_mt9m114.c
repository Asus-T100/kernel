/*
 * platform_mt9m114.c: mt9m114 platform data initilization file
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
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"
#include "platform_mt9e013.h"


#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;


static struct regulator *vprog1_reg;


/*
 * MFLD PR2 secondary camera sensor - MT9M114 platform data
 */
static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
#ifdef CONFIG_BOARD_CTP
		gpio_set_value(camera_reset, 0);
		msleep(60);
#endif
		gpio_set_value(camera_reset, 1);
	} else
		gpio_set_value(camera_reset, 0);

	return 0;
}

static int mt9m114_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

#ifndef CONFIG_BOARD_CTP
static int mt9e013_reset_value;
#endif

static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_BOARD_CTP
	int reg_err;
#endif
#ifndef CONFIG_BOARD_CTP
	int ret;

	/* The camera powering is different on RedHookBay and VictoriaBay
	 * On RHB, vprog1 is at 2.8V and supplies both cameras
	 * On VB, vprog1 supplies the 2nd camera and must not rise over 1.2V
	 * Check if the RHB SW has accidentally been flashed to VB
	 * If yes, don't turn on the regulator. The VB secondary camera will
	 * be permanently damaged by the too high voltage
	 */
	if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO) ||
	    INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)) {
		printk(KERN_ALERT \
		"Aborted vprog1 enable to protect VictoriaBay 2nd camera HW\n");
		return -ENODEV;
	}

	/* Note here, there maybe a workaround to avoid I2C SDA issue */
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
#ifndef CONFIG_BOARD_REDRIDGE
		if (ret < 0)
			return ret;
#endif
		camera_power_down = ret;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}
#endif
	if (flag) {
#ifndef CONFIG_BOARD_CTP
		if (!mt9e013_reset_value) {
			if (mt9e013_reset)
				mt9e013_reset(sd);
			mt9e013_reset_value = 1;
		}
#ifdef CONFIG_BOARD_REDRIDGE
		gpio_direction_output(camera_reset, 0);
#endif
		gpio_set_value(camera_reset, 0);
#endif
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
#else
			intel_scu_ipc_msic_vprog1(1);
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
#else
		gpio_set_value(camera_power_down, 1);
#endif
#endif
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
#else
			intel_scu_ipc_msic_vprog1(0);
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);
#else
		gpio_set_value(camera_power_down, 0);
#endif

		mt9e013_reset_value = 0;
#endif
	}

	return 0;
}

static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		-1, 0, flag);
}

static int mt9m114_platform_init(struct i2c_client *client)
{
	int ret;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return ret;
}

static int mt9m114_platform_deinit(void)
{
	regulator_put(vprog1_reg);

	return 0;
}

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	= mt9m114_gpio_ctrl,
	.flisclk_ctrl	= mt9m114_flisclk_ctrl,
	.power_ctrl	= mt9m114_power_ctrl,
	.csi_cfg	= mt9m114_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = mt9m114_platform_init,
	.platform_deinit = mt9m114_platform_deinit,
#endif
};

void *mt9m114_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &mt9m114_sensor_platform_data;
}

