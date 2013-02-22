/*
 * platform_imx135.c: imx135 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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
#include "platform_imx135.h"


static int camera_reset;
static int camera_vprog1_on;
/*
 * MRFLD VV primary camera sensor - IMX135 platform data
 */

static int imx135_gpio_ctrl(struct v4l2_subdev *sd, int flag)
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
		/* min 250us -Initializing time of silicon */
		usleep_range(250, 300);

	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int imx135_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int imx135_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	if (flag) {
		if (!camera_vprog1_on) {
			ret = intel_scu_ipc_msic_vprog1(1);
			if (!ret) {
				/* VDIG/VANA/VIF rise to XCLR release */
				usleep_range(500, 500);
				camera_vprog1_on = 1;
			}
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
}

static int imx135_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static struct camera_sensor_platform_data imx135_sensor_platform_data = {
	.gpio_ctrl      = imx135_gpio_ctrl,
	.flisclk_ctrl   = imx135_flisclk_ctrl,
	.power_ctrl     = imx135_power_ctrl,
	.csi_cfg        = imx135_csi_configure,
};

void *imx135_platform_data(void *info)
{
	camera_reset = -1;

	return &imx135_sensor_platform_data;
}

