/*
 * platform_imx175.c: imx175 platform data initilization file
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
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_imx175.h"


static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
/*
 * MRFLD VV primary camera sensor - IMX175 platform data
 */

static int imx175_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		/* imx175 core silicon initializing time - t1+t2+t3
		 * 400us(t1) - Time to VDDL is supplied after REGEN high
		 * 600us(t2) - imx175 core Waking up time
		 * 459us(t3, 8825clocks) -Initializing time of silicon
		 */
		usleep_range(1500, 1600);

	} else {
		gpio_set_value(camera_reset, 0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		udelay(1);
	}

	return 0;
}

static int imx175_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int imx175_power_ctrl(struct v4l2_subdev *sd, int flag)
{

	if (flag) {
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
	}

	return 0;
}

static int imx175_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static struct camera_sensor_platform_data imx175_sensor_platform_data = {
	.gpio_ctrl      = imx175_gpio_ctrl,
	.flisclk_ctrl   = imx175_flisclk_ctrl,
	.power_ctrl     = imx175_power_ctrl,
	.csi_cfg        = imx175_csi_configure,
};

void *imx175_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &imx175_sensor_platform_data;
}
