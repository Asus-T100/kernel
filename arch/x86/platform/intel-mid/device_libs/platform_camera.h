/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

/* MFLD iCDK camera sensor GPIOs */

#define GP_CAMERA_0_POWER_DOWN          "cam0_vcm_2p8"
#define GP_CAMERA_1_POWER_DOWN          "camera_1_power"
#define GP_CAMERA_0_RESET               "camera_0_reset"
#define GP_CAMERA_1_RESET               "camera_1_reset"

extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
extern void intel_ignore_i2c_device_register(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev
				) __attribute__((weak));
#endif
