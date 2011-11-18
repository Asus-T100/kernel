/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef ATOMISP_PLATFORM_H_
#define ATOMISP_PLATFORM_H_

#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include "atomisp.h"

enum intel_v4l2_subdev_type {
	RAW_CAMERA = 1,
	SOC_CAMERA = 2,
	CAMERA_MOTOR = 3,
	LED_FLASH = 4,
	XENON_FLASH = 5,
	FILE_INPUT = 6,
	TEST_PATTERN = 7,
};

struct intel_v4l2_subdev_id {
	char name[17];
	enum intel_v4l2_subdev_type type;
	enum atomisp_camera_port    port;
};

struct intel_v4l2_subdev_i2c_board_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
};

struct intel_v4l2_subdev_table {
	struct intel_v4l2_subdev_i2c_board_info v4l2_subdev;
	enum intel_v4l2_subdev_type type;
	enum atomisp_camera_port port;
};

struct atomisp_platform_data {
	struct intel_v4l2_subdev_table *subdevs;
};

struct camera_sensor_platform_data {
	int (*gpio_ctrl)(struct v4l2_subdev *subdev, int flag);
	int (*flisclk_ctrl)(struct v4l2_subdev *subdev, int flag);
	int (*power_ctrl)(struct v4l2_subdev *subdev, int flag);
	int (*csi_cfg)(struct v4l2_subdev *subdev, int flag);
};

struct camera_mipi_info {
	enum atomisp_camera_port        port;
	unsigned int                    num_lanes;
	enum atomisp_input_format       input_format;
	enum atomisp_bayer_order        raw_bayer_order;
	struct atomisp_sensor_mode_data data;
};

#endif /* ATOMISP_PLATFORM_H_ */
