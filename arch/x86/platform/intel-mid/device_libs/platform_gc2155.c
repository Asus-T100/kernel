/*
 * platform_gc2155.c: gc2155 platform data initilization file
 *
 * (C) Copyright 2014 ASUS Corporation
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
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi_gpio.h>
#include <linux/lnw_gpio.h>

#include "camera_info.h"
#include "platform_camera.h"
#include "platform_gc2155.h"

static int camera_reset;
static int gpio_cam_pwdn;
static int camera_vprog_on = 0;

static int pwdn;
static int reset;
#define GC2155_SENSOR_NAME "gc2155"
//const char name[MAX_NAME] = {"gc2155"};
static int clock;

/* workround - pin defined for byt */

void setgpio()
{
#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
        pwdn = CAMERA_0_PWDN;
        reset = CAMERA_0_RESET;
        clock = OSC_CAM0_CLK;
#else
	pwdn = get_pwdn(GC2155_SENSOR_NAME);
	if (pwdn < 0) {
		pr_err("%s(): failed to get pwdn, use default\n", __func__);
		pwdn = CAMERA_2_PWDN;
	}
	reset = get_reset(GC2155_SENSOR_NAME);
	if (reset < 0) {
		pr_err("%s(): failed to get pwdn, use default\n", __func__);
		reset = CAMERA_2_RESET;
	}
	clock = get_clock(GC2155_SENSOR_NAME);
	if (clock < 0) {
		pr_err("%s(): failed to get pwdn, use default\n", __func__);
		clock = OSC_CAM1_CLK;
	}
#endif
	printk("%s(): GPIO is set! pwdn:%d, reset:%d, clock:%d\n", __func__, pwdn, reset, clock);

}

static int gc2155_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;
	pr_info("%s - E, flag: %d\n", __func__, flag);
	if (gpio_cam_pwdn < 0) {
		pin = pwdn;
		ret = gpio_request(pin, "camera_1_powerdown");
		if (ret < 0) {
			pr_err("%s: failed to request gpio(pin %d)\n", __func__, pin);
			return ret;
		}
		gpio_cam_pwdn = pin;

		ret = gpio_direction_output(pin, 1);

		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n", __func__, pin);
			gpio_free(pin);
			return ret;
		}
	}

	if (camera_reset < 0) {
		pin = reset;
		if (camera_reset < 0) {
			ret = gpio_request(pin, "camera_1_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n", __func__, pin);
				return ret;
			}
		}
		camera_reset = pin;
		ret = gpio_direction_output(pin, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n", __func__, pin);
			gpio_free(pin);
			return ret;
		}
	}

	if (flag) {
		/* gc2155 Power down:
		 * FE375CG front: gpio55 high to low
		 * FE375CXG rear: gpio9 high to low
		 */
		pr_info("%s(): pwdn(0)\n", __func__);
		gpio_set_value(gpio_cam_pwdn, 0);
		pr_info("%s(): reset(1)\n", __func__);
		gpio_set_value(camera_reset, 1);
	} else {
		/* gc2155 Power down:
		 * FE375CG front: gpio55 low to high
		 * FE375CXG rear: gpio9 low to high
		 */
		pr_info("%s(): pwdn(1)\n", __func__);
		gpio_set_value(gpio_cam_pwdn, 1);
		gpio_free(gpio_cam_pwdn);
		gpio_cam_pwdn = -1;

		pr_info("%s(): reset(0)\n", __func__);
		gpio_set_value(camera_reset, 0);
		gpio_free(camera_reset);
		camera_reset = -1;
	}

	return 0;
}

static int gc2155_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	static const unsigned int clock_khz = 19200;
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(clock, flag ? clock_khz : 0);
#endif
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		ret = pmc_pc_set_freq(clock, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(clock, CLK_ON);
	}
	ret = pmc_pc_configure(clock, CLK_OFF);
#endif
	return ret;
}

static int _gc2155_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	/* TODO:
	 *   Control 2V8 enable pin for FE375CG/FE375CXG.
	 */

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		if (intgc2155_gpio_ctrlel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
			ret = intel_scu_ipc_msic_vprog1(1);
#endif
#ifdef CONFIG_CRYSTAL_COVE
		/* 1V8 on */
		pr_info("%s(): 1V8 on\n", __func__);
		ret = camera_set_pmic_power(CAMERA_1P8V, true);
		if (ret) {
			pr_err("%s(): fail to turn on 1.8V\n", __func__);
			return ret;
		}

		usleep_range(1000, 1500);

		/* 2V8 on */
		pr_info("%s(): 2V8 on\n", __func__);
		ret = camera_set_pmic_power(CAMERA_2P8V, true);
		if (ret){
			pr_err("%s(): fail to turn on 2.8V\n", __func__);
			return ret;
		}

		usleep_range(1000, 5000);

		/* Enable MCLK: 19.2MHz */
		pr_info("%s(): mclk on\n", __func__);
		ret = gc2155_flisclk_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): flisclk_ctrl on failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);

		/* Power-down & Reset */
		ret = gc2155_gpio_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): gpio_ctrl on failed\n", __func__);
			return ret;
		}
		usleep_range(5, 10); /* Delay for I2C cmds: 100 mclk cycles */
#endif
	} else {
		/* Power-down & Reset */
		ret = gc2155_gpio_ctrl(sd, 0);
		if (ret) {
			pr_err("%s(): gpio_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);

		/* Disable MCLK */
		pr_info("%s(): mclk off\n", __func__);
		ret = gc2155_flisclk_ctrl(sd, 0);
		if (ret) {
			pr_err("%s(): flisclk_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(1000, 5000);
		/* 2V8 off */
		pr_info("%s(): 2V8 off\n", __func__);
		ret = camera_set_pmic_power(CAMERA_2P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 2.8V\n", __func__);
			return ret;
		}

		usleep_range(10, 12);

		/* 1V8 off */
		pr_info("%s(): 1V8 off\n", __func__);
		ret = camera_set_pmic_power(CAMERA_1P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 1.8V\n", __func__);
			return ret;
		}
	}
	pr_info("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int gc2155_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	if (camera_vprog_on != flag) {
		if((ret = _gc2155_power_ctrl(sd, flag)) == 0)
			camera_vprog_on = flag;
	}
	return ret;
}

static int gc2155_csi_configure(struct v4l2_subdev *sd, int flag)
{
#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
        return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 1,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
#else
	int port = decide_cam_id(GC2155_SENSOR_NAME);

	if (port == REAR_CAM) {
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 1,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
	}
	if (port == FRONT_CAM) {
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
	}
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
#endif
}

#ifdef CONFIG_BOARD_CTP
static int gc2155_platform_init(struct i2c_client *client)
{
	int ret;

	pr_info("%s(): project_id_for_cam = %d\n", __func__, project_id_for_cam);

	if (gpio_cam_pwdn < 0) {
		switch (project_id_for_cam) {
			case PROJECT_ID_ME581CL:
			case PROJECT_ID_FE375CXG:
				ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
						GPIOF_DIR_OUT, 1);
				break;

			case PROJECT_ID_FE375CG:
			default:
				ret = camera_sensor_gpio(-1, GP_SUB_CAM_PWDN,
						GPIOF_DIR_OUT, 1);
		}

		if (ret < 0) {
			pr_info("%s(): camera_sensor_gpio fail\n", __func__);
			return ret;
		}
		gpio_cam_pwdn = ret;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(gpio_cam_pwdn, LNW_GPIO);

		pr_info("%s(): pwdn(1)\n", __func__);
		gpio_set_value(gpio_cam_pwdn, 1);
		gpio_free(gpio_cam_pwdn);
		gpio_cam_pwdn = -1;
	}
	return 0;
}

static int gc2155_platform_deinit(void)
{
	pr_info("%s()\n", __func__);
	return 0;
}
#endif

static struct camera_sensor_platform_data gc2155_sensor_platform_data = {
	.gpio_ctrl       = gc2155_gpio_ctrl,
	.flisclk_ctrl    = gc2155_flisclk_ctrl,
	.power_ctrl      = gc2155_power_ctrl,
	.csi_cfg         = gc2155_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init   = gc2155_platform_init,
	.platform_deinit = gc2155_platform_deinit,
#endif
};

void *gc2155_platform_data(void *info)
{
	pr_info("%s()\n", __func__);
	setgpio();
	gpio_cam_pwdn = -1;
	camera_reset = -1;

	return &gc2155_sensor_platform_data;
}

