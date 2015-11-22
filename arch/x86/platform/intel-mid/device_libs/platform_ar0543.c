/*
 * platform_ar0543.c: ar0543 platform data initilization file
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
#include <linux/mfd/intel_mid_pmic.h>

#include "platform_camera.h"
#include "platform_ar0543.h"
#include "camera_info.h"

/* workround - pin defined for byt */
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2
#endif

#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vcm_power_down;
static int camera_vprog1_on;
static int camera_2p8_gpio;

static struct regulator *vprog1_reg;

/*
 * MFLD PR2 secondary camera sensor - ar0543 platform data //test
 */
static int ar0543_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;
	/*
	* FIXME: WA using hardcoded GPIO value here.
	* The GPIO value would be provided by ACPI table, which is
	* not implemented currently.
	*/
	pin = CAMERA_1_PWDN;
	if (camera_reset < 0) {
		ret = gpio_request(pin, "camera_1_powerdown");
		if (ret) {
			pr_err("%s(): failed to request gpio(pin %d)\n", __func__, pin);
			return ret;
		}
	}
	camera_reset = pin;
	ret = gpio_direction_output(pin, 1);
	if (ret) {
		pr_err("%s(): failed to set gpio(pin %d) direction\n", __func__, pin);
		gpio_free(pin);
		return ret;
	}

	//for vcm
	pin = CAMERA_1_VCM_PD;
	if (camera_vcm_power_down < 0) {
		ret = gpio_request(pin, "camera_vcm_pd to non-acive");
		if (ret) {
			pr_err("%s(): failed to request gpio(pin %d)\n", __func__, pin);
			return ret;
		}
	}
	camera_vcm_power_down = pin;
	ret = gpio_direction_output(pin, 1);
	if (ret) {
		pr_err("%s(): failed to set gpio(pin %d) direction\n", __func__, pin);
		gpio_free(pin);
		return ret;
	}

	if (flag) {
#ifdef CONFIG_BOARD_CTP
		gpio_set_value(camera_reset, 0);
		msleep(60);
#endif
		gpio_set_value(camera_reset, 1);
		gpio_set_value(camera_vcm_power_down, 1);
	} else {
		gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_vcm_power_down, 0);
		gpio_free(camera_reset);
		gpio_free(camera_vcm_power_down);
		camera_reset = -1;
		camera_vcm_power_down = -1;
	}
	return 0;
}

static int ar0543_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#endif
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM0_CLK, CLK_ON);
	}
	ret = pmc_pc_configure(OSC_CAM0_CLK, CLK_OFF);
#endif
	return ret;
}

#ifndef CONFIG_BOARD_CTP
static int mt9e013_reset_value;
#endif

static int _ar0543_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0, pin = 0;

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
			ret = intel_scu_ipc_msic_vprog1(1);
#endif
#ifdef CONFIG_CRYSTAL_COVE
		/*
		* This should call VRF APIs.
		*
		* VRF not implemented for BTY, so call this
		* as WAs
		*/
		//turn on 1.8V
		printk("%s(): turn on 1.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_1P8V, true);
		if (ret){
			pr_err("%s(): fail to turn on 1.8V\n", __func__);
			return ret;
		}

		//set EXTCLK 19.2MHz
		printk("%s(): set EXTCLK 19.2MHz\n", __func__ );
		ret = ar0543_flisclk_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): fail to set EXTCLK 19.2MHz\n", __func__);
			return ret;
		}
		msleep(10);
		//set RESET_BAR & VCM
		printk("%s(): set RESET_BAR & VCM\n", __func__ );
		ret = ar0543_gpio_ctrl(sd, 1);
		if (ret) {
			pr_err("%s(): fail to set RESET_BAR & VCM\n", __func__);
			return ret;
		}
		msleep(5);

		//turn on 2.8V
		printk("%s(): turn on 2.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_2P8V, true);
		if (ret){
			pr_err("%s(): fail to turn on 2.8V\n", __func__);
			return ret;
		}

		//set CAM_2P8_GPIO high
		printk("%s(): set high CAM_2P8_GPIO\n", __func__ );
		pin = CAMERA_2P8_EN;
		if (camera_2p8_gpio < 0) {
			ret = gpio_request(pin, "camera_2p8_gpio");
			if (ret) {
				pr_err("%s(): failed to request gpio(pin %d)\n", __func__, pin);
				return ret;
			}
		 }
		camera_2p8_gpio = pin;
		ret = gpio_direction_output(pin, 1);
		if (ret) {
			pr_err("%s(): failed to set gpio(pin %d) direction\n", __func__, pin);
			gpio_free(pin);
			return ret;
		}
		gpio_set_value(camera_2p8_gpio, 1);
#endif
	} else {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
			ret = intel_scu_ipc_msic_vprog1(0);
#endif
#ifdef CONFIG_CRYSTAL_COVE

		//release RESET_BAR & VCM
		printk("%s(): release RESET_BAR & VCM\n", __func__ );
		ret = ar0543_gpio_ctrl(sd, 0);
		if (ret) {
			pr_err("%s(): fail to release RESET_BAR & VCM\n", __func__);
			return ret;
		}

		//set CAM_2P8_GPIO low
		printk("%s(): set low CAM_2P8_GPIO\n", __func__ );
		pin = CAMERA_2P8_EN;
		if (camera_2p8_gpio < 0) {
			ret = gpio_request(pin, "camera_2p8_gpio");
			if (ret) {
				pr_err("%s(): failed to request gpio(pin %d)\n", __func__, pin);
				return ret;
			}
		}
		camera_2p8_gpio = pin;
		ret = gpio_direction_output(pin, 1);
		if (ret) {
			pr_err("%s(): failed to set gpio(pin %d) direction\n", __func__, pin);
			gpio_free(pin);
			return ret;
		}
		gpio_set_value(camera_2p8_gpio, 0);
		gpio_free(camera_2p8_gpio);
		camera_2p8_gpio = -1;

		//turn off 2.8V
		printk("%s(): turn off 2.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_2P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 2.8V\n", __func__);
			return ret;
		}

		//turn off 1.8V
		printk("%s(): turn off 1.8V\n", __func__ );
		ret = camera_set_pmic_power(CAMERA_1P8V, false);
		if (ret){
			pr_err("%s(): fail to turn off 1.8V\n", __func__);
			return ret;
		}

		//release EXTCLK 19.2MHz
		printk("%s(): release EXTCLK 19.2MHz\n", __func__ );
		ret = ar0543_flisclk_ctrl(sd, 2);
		if (ret) {
			pr_err("%s(): fail to release EXTCLK 19.2MHz\n", __func__);
			return ret;
		}
#endif
	}
	return ret;
}

static int ar0543_power_ctrl(struct v4l2_subdev *sd, int flag)
{

	if (!flag) {
		camera_vprog1_on = _ar0543_power_ctrl(sd, flag);
	} else if (camera_vprog1_on != flag) {
		camera_vprog1_on = _ar0543_power_ctrl(sd, flag);
	}

	return camera_vprog1_on;
}

static int ar0543_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static int ar0543_platform_init(struct i2c_client *client)
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

static int ar0543_platform_deinit(void)
{
	pr_info("%s()\n", __func__);
	regulator_put(vprog1_reg);
	return 0;
}

static struct camera_sensor_platform_data ar0543_sensor_platform_data = {
	.gpio_ctrl       = ar0543_gpio_ctrl,
	.flisclk_ctrl    = ar0543_flisclk_ctrl,
	.power_ctrl      = ar0543_power_ctrl,
	.csi_cfg         = ar0543_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init   = ar0543_platform_init,
	.platform_deinit = ar0543_platform_deinit,
#endif
};

void *ar0543_platform_data(void *info)
{
	pr_info("%s()\n", __func__);

	camera_reset = -1;
	camera_power_down = -1;
	camera_vcm_power_down = -1;
	camera_2p8_gpio = -1;

	return &ar0543_sensor_platform_data;
}

