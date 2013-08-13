/*
 * platform_mt9m114.c: mt9m114 platform data initilization file
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
#include <linux/vlv2_plat_clock.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"
#include "platform_mt9e013.h"
#include <linux/acpi_gpio.h>

/* workround - pin defined for byt */
#define CAMERA_1_RESET 127
#define CAMERA_1_PWDN 124
#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
#endif
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2
#endif

#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;


static struct regulator *vprog1_reg;


/*
 * MFLD PR2 secondary camera sensor - MT9M114 platform data
 */
static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;
    int acpi_int;

        printk("[ASUS] Platform_mt9m114 mt9m114_gpio_ctrl [in], intel_mid_identify_cpu = %d\n", intel_mid_identify_cpu());
    
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
printk("[ASUS] if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)\n");
                if (camera_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			camera_reset = ret;
		}
	} else {
		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently.
		 */
printk("[ASUS] if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2)\n");		 
                acpi_int = acpi_get_gpio("\\_SB.GPO1", 10);
printk("[ASUS] acpi_int = %d\n", acpi_int);
                pin = CAMERA_1_RESET;
		if (camera_reset < 0) {
			ret = gpio_request(pin, "camera_1_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		camera_reset = pin;
		ret = gpio_direction_output(pin, 1);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
	if (flag) {
#ifdef CONFIG_BOARD_CTP
printk("[ASUS] CONFIG_BOARD_CTP, gpio_set_value(camera_reset, 0);\n");
		gpio_set_value(camera_reset, 0);
		msleep(60);
#endif
printk("[ASUS] CONFIG_BOARD_CTP, gpio_set_value(camera_reset, 1);\n");
		gpio_set_value(camera_reset, 1);
	} else
		gpio_set_value(camera_reset, 0);

	return 0;
}

static int mt9m114_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
					     flag ? clock_khz : 0);
#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		ret = vlv2_plat_set_clock_freq(OSC_CAM1_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	ret = vlv2_plat_configure_clock(OSC_CAM1_CLK, flag);
#endif
	return ret;
}

#ifndef CONFIG_BOARD_CTP
static int mt9e013_reset_value;
#endif

static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

    printk("[ASUS] Platform_mt9m114 mt9m114_power_ctrl [in], flag = %d\n", flag);

	if (flag) {
		if (!camera_vprog1_on) {
			if (intel_mid_identify_cpu() !=
			    INTEL_MID_CPU_CHIP_VALLEYVIEW2)
				ret = intel_scu_ipc_msic_vprog1(1);
#ifdef CONFIG_CRYSTAL_COVE
			/*
			 * This should call VRF APIs.
			 *
			 * VRF not implemented for BTY, so call this
			 * as WAs
			 */
			ret = camera_set_pmic_power(CAMERA_1P8V, true);
			if (ret)
				return ret;
			ret = camera_set_pmic_power(CAMERA_2P8V, true);
#endif
			if (!ret)
				camera_vprog1_on = 1;
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			if (intel_mid_identify_cpu() !=
			    INTEL_MID_CPU_CHIP_VALLEYVIEW2)
				ret = intel_scu_ipc_msic_vprog1(0);
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			if (ret)
				return ret;
			ret = camera_set_pmic_power(CAMERA_1P8V, false);
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}

static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static int mt9m114_platform_init(struct i2c_client *client)
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

static int mt9m114_platform_deinit(void)
{
	regulator_put(vprog1_reg);

	return 0;
}

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	= mt9m114_gpio_ctrl,
	.flisclk_ctrl	= mt9m114_flisclk_ctrl,
	.power_ctrl	= mt9m114_power_ctrl,
	.csi_cfg	= mt9m114_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = mt9m114_platform_init,
	.platform_deinit = mt9m114_platform_deinit,
#endif
};

void *mt9m114_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &mt9m114_sensor_platform_data;
}

