/*
 * platform_lm3554.c: lm3554 platform data initilization file
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
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_lm3554.h"

void *lm3554_platform_data(void *info)
{
	static struct camera_flash_platform_data lm3554_platform_data;
	void *ret = &lm3554_platform_data;

	lm3554_platform_data.gpio_reset  = get_gpio_by_name("GP_FLASH_RESET");
	lm3554_platform_data.gpio_strobe = get_gpio_by_name("GP_FLASH_STROBE");
	lm3554_platform_data.gpio_torch  = get_gpio_by_name("GP_FLASH_TORCH");

	if (lm3554_platform_data.gpio_reset == -1) {
		pr_err("%s: Unable to find GP_FLASH_RESET\n", __func__);
		ret = NULL;
	}
	if (lm3554_platform_data.gpio_strobe == -1) {
		pr_err("%s: Unable to find GP_FLASH_STROBE\n", __func__);
		ret = NULL;
	}
	if (lm3554_platform_data.gpio_torch == -1) {
		pr_err("%s: Unable to find GP_FLASH_TORCH\n", __func__);
		ret = NULL;
	}
	return ret;
}
