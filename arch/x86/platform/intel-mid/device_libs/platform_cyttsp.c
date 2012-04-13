/*
 * platform_cyttsp.c: cyttsp platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/cyttsp.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_cyttsp.h"

static int cyttsp_init(int on)
{
	int ret;

	if (on) {
		ret = gpio_request(CYTTSP_GPIO_PIN, "cyttsp_irq");
		if (ret < 0) {
			pr_err("%s: gpio request failed\n", __func__);
			return ret;
		}

		ret = gpio_direction_input(CYTTSP_GPIO_PIN);
		if (ret < 0) {
			pr_err("%s: gpio direction config failed\n", __func__);
			gpio_free(CYTTSP_GPIO_PIN);
			return ret;
		}
	} else {
		gpio_free(CYTTSP_GPIO_PIN);
	}
	return 0;
}

void *cyttsp_platform_data(void *info)
{
	static struct cyttsp_platform_data cyttsp_pdata = {
		.init = cyttsp_init,
		.mt_sync = input_mt_sync,
		.maxx = 479,
		.maxy = 853,
		.flags = 0,
		.gen = CY_GEN3,
		.use_st = 0,
		.use_mt = 1,
		.use_trk_id = 0,
		.use_hndshk = 1,
		.use_timer = 0,
		.use_sleep = 1,
		.use_gestures = 0,
		.act_intrvl = CY_ACT_INTRVL_DFLT,
		.tch_tmout = CY_TCH_TMOUT_DFLT,
		.lp_intrvl = CY_LP_INTRVL_DFLT / 2,
		.name = CY_I2C_NAME,
		.irq_gpio = CYTTSP_GPIO_PIN,
	};

	return &cyttsp_pdata;
}
