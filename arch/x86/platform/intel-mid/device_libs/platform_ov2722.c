/*
 * platform_ov2722.c: ov2722 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_ov2722.h"

static int camera_vprog1_on;
static int gp_camera1_power_down;
static int gp_camera1_reset;

/*
 * OV2722 platform data
 */

static int ov2722_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (gp_camera1_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera1_power_down = ret;
	}

	if (gp_camera1_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera1_reset = ret;
	}

	if (flag) {
		gpio_set_value(gp_camera1_power_down, 1);
		gpio_set_value(gp_camera1_reset, 0);
		msleep(50);
		gpio_set_value(gp_camera1_reset, 1);
	} else {
		gpio_set_value(gp_camera1_reset, 0);
		gpio_set_value(gp_camera1_power_down, 0);
	}

	return 0;
}

static int ov2722_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

/*
 * The power_down gpio pin is to control OV2722's
 * internal power state.
 */
static int ov2722_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (flag) {
		if (!camera_vprog1_on) {
			ret = intel_scu_ipc_msic_vprog1(1);
			if (!ret)
				camera_vprog1_on = 1;
			msleep(100);
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			ret = intel_scu_ipc_msic_vprog1(0);
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}

static int ov2722_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static struct camera_sensor_platform_data ov2722_sensor_platform_data = {
	.gpio_ctrl	= ov2722_gpio_ctrl,
	.flisclk_ctrl	= ov2722_flisclk_ctrl,
	.power_ctrl	= ov2722_power_ctrl,
	.csi_cfg	= ov2722_csi_configure,
};

void *ov2722_platform_data(void *info)
{
	gp_camera1_power_down = -1;
	gp_camera1_reset = -1;
	return &ov2722_sensor_platform_data;
}
