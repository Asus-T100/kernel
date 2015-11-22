/*
 * platform_gc0310.c: gc0310 platform data initilization file
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

#include "platform_camera.h"
#include "platform_gc0310.h"
#include "camera_info.h"

static int camera_reset;
static int gpio_cam_pwdn;
static int gpio_cam_2p8_en;

static int camera_vprog_on;

/* workround - pin defined for byt */

#define CAMERA_2P8_EN 153

static int gc0310_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	//ASUS_BSP+++
	int ret;
	int pin;
	/*
	* FIXME: WA using hardcoded GPIO value here.
	* The GPIO value would be provided by ACPI table, which is
	* not implemented currently.
	*/
	pin = CAMERA_2_PWDN;
	if (gpio_cam_pwdn < 0) {
		ret = gpio_request(pin, "camera_front_reset");
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n", __func__, pin);
			return ret;
		}
	}
	gpio_cam_pwdn = pin;
	ret = gpio_direction_output(pin, 0);
	if (ret) {
		pr_err("%s(): failed to set gpio(pin %d) direction\n", __func__, pin);
		gpio_free(pin);
		return ret;
	}

	if (flag == 1) {
		gpio_set_value(gpio_cam_pwdn, 1);
		usleep_range(10, 20);
		gpio_set_value(gpio_cam_pwdn, 0);
	} else if (flag == 2) {
		gpio_set_value(gpio_cam_pwdn, 1);
	} else if (flag == 0) {
		/* gc0310 Power down:
		* FE380CG front: gpio55 low to high
		* FE380CXG rear: gpio9 low to high
		*/
		gpio_set_value(gpio_cam_pwdn, 0);
		gpio_free(gpio_cam_pwdn);
		gpio_cam_pwdn = -1;
	}

	return 0;
}

static int gc0310_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	static const unsigned int clock_khz = 19200;

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);

#endif
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM1_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM1_CLK, CLK_ON);
	}
	ret = pmc_pc_configure(OSC_CAM1_CLK, CLK_OFF);
#endif
	return ret;
}

static int _gc0310_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
			ret = intel_scu_ipc_msic_vprog1(1);
#endif
#ifdef CONFIG_CRYSTAL_COVE
		/* 1V8 on */
		printk("%s(): turn on 1.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_1P8V, true);
		if (ret) {
			pr_err("%s(): fail to turn on 1.8V\n", __func__);
			return ret;
		}
		usleep_range(1000, 1500);
		/* 2V8 on */
		printk("%s(): turn on 2.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_2P8V, true);
		if (ret) {
			pr_err("%s(): fail to turn on 2.8V\n", __func__);
			return ret;
		}
		usleep_range(1000, 1500);
		/* Enable MCLK: 19.2MHz */
		printk("%s(): set EXTCLK 19.2MHz\n", __func__ );
		ret = gc0310_flisclk_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): flisclk_ctrl on failed\n", __func__);
			return ret;
		}
		usleep_range(5000, 6000);
		/* Power-down & Reset */
		ret = gc0310_gpio_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): gpio_ctrl on failed\n", __func__);
			return ret;
		}
		usleep_range(5, 10); /* Delay for I2C cmds: 100 mclk cycles */
#endif
	} else {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
			ret = intel_scu_ipc_msic_vprog1(0);
#endif
#ifdef CONFIG_CRYSTAL_COVE
		/* Power-down & Reset */
		ret = gc0310_gpio_ctrl(sd, 2);
		if (ret) {
			pr_err("%s(): gpio_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);
		/* Disable MCLK */
		pr_info("%s(): mclk off\n", __func__);
		ret = gc0310_flisclk_ctrl(sd, 0);
		if (ret) {
			pr_err("%s(): flisclk_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(1000, 5000);
		/* 2V8 off */
		printk("%s(): turn off 2.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_2P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 2.8V\n", __func__);
			return ret;
		}

		msleep(2);
		/* 1V8 off */
		printk("%s turn off 1.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_1P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 1.8V\n", __func__);
			return ret;
		}

		msleep(6);

		ret = gc0310_gpio_ctrl(sd, 0);
		if (ret) {
			pr_err("%s():  gpio_ctrl off- Lowstage failed\n", __func__);
			return ret;
		}
#endif
	}

	pr_info("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int gc0310_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	if (camera_vprog_on != flag) {
		if((ret = _gc0310_power_ctrl(sd, flag)) == 0)
		camera_vprog_on = flag;
	}
	return ret;
}

static int gc0310_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* The secondary MIPI port in moorefield doesn't support 2-lane.
	* So, we use TERTIARY instead of SECONDARY. */
	pr_info("%s(): port: SECONDARY; MIPI lane num: 1\n", __func__);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_grbg, flag);


}
#ifdef CONFIG_BOARD_CTP
static int gc0310_platform_init(struct i2c_client *client)
{
	int ret;

	pr_info("%s()\n", __func__);
	camera_vprog_on = 0;

	if (gpio_cam_pwdn < 0) {
		gpio_cam_pwdn = 55/*ret*/;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(gpio_cam_pwdn, LNW_GPIO);

		pr_info("%s(): pwdn(0)\n", __func__);
		gpio_set_value(gpio_cam_pwdn, 0);
		gpio_free(gpio_cam_pwdn);
		gpio_cam_pwdn = -1;
	}


	return 0;
}

static int gc0310_platform_deinit(void)
{
	pr_info("%s()\n", __func__);

	return 0;
}
#endif
static struct camera_sensor_platform_data gc0310_sensor_platform_data = {
	.gpio_ctrl       = gc0310_gpio_ctrl,
	.flisclk_ctrl    = gc0310_flisclk_ctrl,
	.power_ctrl      = gc0310_power_ctrl,
	.csi_cfg         = gc0310_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init   = gc0310_platform_init,
	.platform_deinit = gc0310_platform_deinit,
#endif
};

void *gc0310_platform_data(void *info)
{

	pr_info("%s()\n", __func__);

	gpio_cam_pwdn = -1;
	camera_reset = -1;
	gpio_cam_2p8_en = -1;

	return &gc0310_sensor_platform_data;
}

