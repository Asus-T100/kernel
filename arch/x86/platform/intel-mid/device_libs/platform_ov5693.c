/*
 * platform_ov5693.c: ov5693 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
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
#include "platform_ov5693.h"
#include <linux/acpi_gpio.h>

/* workround - pin defined for byt */
#define CAMERA_1_RESET 127
#define CAMERA_0_PWDN 126		// <ASUS-Ian20131120>
#define CAMERA_0_VCM_PD 122		// <ASUS-Ian20131120>
#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x0
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
static int camera_front_reset;		// <ASUS-Ian20131120>
static int camera_power_down;
static int camera_vcm_power_down;
static int camera_led_mask;			// <ASUS-Ian20131120>
static int camera_vprog1_on;

static struct regulator *vprog1_reg;
int ov5693_gpio_request(int pin, int* gpio, const char *label);	// <ASUS-Ian20131120>

// <ASUS-Ian20131223+>
/*
 * For ov5693 probe
 */
static int ov5693_probe_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
    int pin;
          
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
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
		 	
		// Back camera reset pin
		pin = CAMERA_0_PWDN;
		ret = ov5693_gpio_request(pin, &camera_reset, "camera_0_powerdown");
		if (ret) {
			return ret;
		}
		
		//for vcm
		pin = CAMERA_0_VCM_PD;
 		ret = ov5693_gpio_request(pin, &camera_vcm_power_down, "camera_vcm_pd");
		if (ret) {
			return ret;
		}
	}
	if (flag) {
#ifdef CONFIG_BOARD_CTP
	    gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_vcm_power_down, 0);
	    msleep(60);
#endif
	    gpio_set_value(camera_reset, 1);
		gpio_set_value(camera_vcm_power_down, 1);
	} else {
	    gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_vcm_power_down, 0);
    }
	
	return 0;
}
// <ASUS-Ian20131223->

/*
 * camera sensor - ov5693 platform data
 */
// <ASUS-Ian20131120+>
static int ov5693_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
          int pin;
          
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
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
		 	
		// Back camera reset pin
		pin = CAMERA_0_PWDN;
		ret = ov5693_gpio_request(pin, &camera_reset, "camera_0_powerdown");
		if (ret) {
			return ret;
		}
		
		//for vcm
		pin = CAMERA_0_VCM_PD;
 		ret = ov5693_gpio_request(pin, &camera_vcm_power_down, "camera_vcm_pd");
		if (ret) {
			return ret;
		}
		
		// Front camera reset pin
		pin = CAMERA_1_RESET;
		ret = ov5693_gpio_request(pin, &camera_front_reset, "camera_1_reset");
		if (ret) {
			return ret;
		}
		
		// CAM_LED_MASK
		pin = acpi_get_gpio("\\_SB.GPO2", 8);
		ret = ov5693_gpio_request(pin, &camera_led_mask, "cam_led_mask");
		if (ret) {
			return ret;
		}
/*	 
		pin = CAMERA_1_PWDN;
		if (camera_reset < 0) {
			ret = gpio_request(pin, "camera_1_powerdown");			
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

		//for vcm
		pin = CAMERA_1_VCM_PD;
		if (camera_vcm_power_down < 0) {
			ret = gpio_request(pin, "camera_vcm_pd to non-acive");			
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		camera_vcm_power_down = pin;
		ret = gpio_direction_output(pin, 1);			
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
*/	
	}
	if (flag) {
#ifdef CONFIG_BOARD_CTP
	    gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_vcm_power_down, 0);
	    msleep(60);
#endif
	    gpio_set_value(camera_reset, 1);
	    gpio_set_value(camera_vcm_power_down, 1);
		
		// Enable LED indicator
		gpio_set_value(camera_front_reset, 1);
		gpio_set_value(camera_led_mask, 1);
	} else {
	    gpio_set_value(camera_reset, 0);
	    gpio_set_value(camera_vcm_power_down, 0);
		
		// Disable LED indicator
		gpio_set_value(camera_front_reset, 0);
		gpio_set_value(camera_led_mask, 0);
    }
	
	return 0;
}
// <ASUS-Ian20131120+>

// <ASUS-Ian20131120+>
int ov5693_gpio_request(int pin, int* gpio, const char *label){
	int ret = 0;
printk("<ASUS-Ian> pin = %d, gpio = %d\n", pin, *gpio);        
        if (*gpio < 0) {            
            ret = gpio_request(pin, label);        
printk("<ASUS-Ian> gpio_request = %d\n", ret);             
            if (ret) {
                pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
                return ret;
            }
            *gpio = pin;
printk("<ASUS-Ian> gpio = %d\n", *gpio);             
            ret = gpio_direction_output(*gpio, 0);
printk("<ASUS-Ian> gpio_direction_output = %d\n", ret);              
            if (ret) {
                pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, *gpio);
                gpio_free(*gpio);
                return ret;
            }
            gpio_free(*gpio);
        }
        return 0;
}
// <ASUS-Ian20131120->

static int ov5693_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
					     flag ? clock_khz : 0);
#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	ret = vlv2_plat_configure_clock(OSC_CAM0_CLK, flag);
#endif
	return ret;
}

/*
 * The power_down gpio pin is to control OV5693's
 * internal power state.
 */
static int ov5693_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
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

static int ov5693_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static struct camera_sensor_platform_data ov5693_sensor_platform_data = {
	.probe_gpio_ctrl = ov5693_probe_gpio_ctrl, // <ASUS-Ian20131223>
	.gpio_ctrl = ov5693_gpio_ctrl,
	.flisclk_ctrl	= ov5693_flisclk_ctrl,
	.power_ctrl	= ov5693_power_ctrl,
	.csi_cfg	= ov5693_csi_configure,
};

void *ov5693_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_vcm_power_down = -1;

	camera_front_reset = -1; // <ASUS-Ian20131120>
	camera_led_mask = -1;	 // <ASUS-Ian20131120>
		 
	return &ov5693_sensor_platform_data;
}
