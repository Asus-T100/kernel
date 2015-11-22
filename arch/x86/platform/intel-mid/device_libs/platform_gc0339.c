/*
 * platform_gc0339.c: GC0339 with iCatch 7002A ISP platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
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
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/regulator/consumer.h>
#include <linux/lnw_gpio.h>

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_gc0339.h"
#include "camera_info.h"

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
#endif

static int camera_power_down;
static int camera_reset;
static int camera_vprog1_on;

static int camera_I2C_3_SCL;
static int camera_I2C_3_SDA;

#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE) //ASUS_BSP ++
static int gc0339_i2c_gpio_set_alt(int flag)
{
	int ret;

	if (flag){
	    lnw_gpio_set_alt(SIO_I2C3_SCL, LNW_GPIO);
	    lnw_gpio_set_alt(SIO_I2C3_SDA, LNW_GPIO);

	    if (camera_I2C_3_SCL < 0) {
	        ret = camera_sensor_gpio(SIO_I2C3_SCL, GP_I2C_3_SCL,
	                GPIOF_DIR_OUT, 1);
	        if (ret < 0){
	            printk("%s not available.\n", GP_I2C_3_SCL);
	            return ret;
	        }
	        camera_I2C_3_SCL = SIO_I2C3_SCL;
	    }

	    if (camera_I2C_3_SDA < 0) {
	        ret = camera_sensor_gpio(SIO_I2C3_SDA, GP_I2C_3_SDA,
	                GPIOF_DIR_OUT, 1);
	        if (ret < 0){
	            printk("%s not available.\n", GP_I2C_3_SDA);
	            return ret;
	        }
	        camera_I2C_3_SDA = SIO_I2C3_SDA;
	    }

		if (camera_I2C_3_SCL >= 0){
			gpio_set_value(camera_I2C_3_SCL, 1);
			printk("<<< I2C_3 SCL = 1\n");
			msleep(1);
		}
		
		if (camera_I2C_3_SDA >= 0){
			gpio_set_value(camera_I2C_3_SDA, 1);
			printk("<<< I2C_3 SDA = 1\n");
			msleep(1);
		}

		lnw_gpio_set_alt(SIO_I2C3_SCL, LNW_ALT_1);
		lnw_gpio_set_alt(SIO_I2C3_SDA, LNW_ALT_1);
		
		msleep(2);
	}else{
		if (camera_I2C_3_SCL >= 0){
			gpio_free(camera_I2C_3_SCL);
			camera_I2C_3_SCL = -1;
			mdelay(1);
		}
		
		if (camera_I2C_3_SDA >= 0){
			gpio_free(camera_I2C_3_SDA);
			camera_I2C_3_SDA = -1;
			mdelay(1);
		}
	}

	return ret;
}
#endif//ASUS_BSP --

#if defined(CONFIG_ME176C) //ASUS_BSP ++
static int gc0339_gpio_init()
{
	int pin;
	int ret = 0;

	pin = CAMERA_2_RESET;
        printk("%s\n", __func__ );
	if (camera_reset < 0) {
		ret = gpio_request(pin, NULL);
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
		camera_reset = pin;
		ret = gpio_direction_output(pin, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
        printk("%s CAMERA_1_RESET: %d \n", __func__, camera_reset);

	pin = CAMERA_2_PWDN;
	if (camera_power_down < 0) {
		ret = gpio_request(pin, NULL);
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
		camera_power_down = pin;
		ret = gpio_direction_output(pin, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
        printk("%s CAMERA_2_PWDN: %d \n", __func__, CAMERA_2_PWDN);

	return ret;
}
#else //ASUS_BSP for other project
static int gc0339_gpio_init()
{
	int pin;
	int ret = 0;

	pin = CAMERA_1_RESET;
        printk("%s\n", __func__ );
	if (camera_reset < 0) {
		ret = gpio_request(pin, NULL);
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
		camera_reset = pin;
		ret = gpio_direction_output(pin, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
        printk("%s CAMERA_1_RESET: %d \n", __func__, camera_reset);

	pin = CAMERA_1_PWDN;
	if (camera_power_down < 0) {
		ret = gpio_request(pin, NULL);
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
		camera_power_down = pin;
		ret = gpio_direction_output(pin, 0);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
        printk("%s CAMERA_1_PWDN: %d \n", __func__, CAMERA_1_PWDN);

	return ret;
}
#endif//ASUS_BSP --

static int gc0339_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE) //ASUS_BSP ++

#ifdef CONFIG_INTEL_SOC_PMC
	int ret = 0;
	if (flag) {
                printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_ON\n",__func__,flag);
		ret = pmc_pc_set_freq(OSC_CAM1_CLK, (IS_CHT) ?
			CLK_19P2MHz_XTAL : CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM1_CLK, CLK_ON);
	}
        printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_OFF\n",__func__,flag);
	return pmc_pc_configure(OSC_CAM1_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
        static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz
        printk("%s: CONFIG_INTEL_SCU_IPC_UTIL\n",__func__);
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
				     flag ? clock_khz : 0);
#else
	pr_err("gc0339 clock is not set.\n");
	return 0;
#endif

#else

#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		int ret;
                printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_ON\n",__func__,flag);
		ret = pmc_pc_set_freq(OSC_CAM1_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM1_CLK, CLK_ON);
	}
        printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_OFF\n",__func__,flag);
	return pmc_pc_configure(OSC_CAM1_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
        static const unsigned int clock_khz = 19200;
        printk("%s: CONFIG_INTEL_SCU_IPC_UTIL\n",__func__);
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
			flag ? clock_khz : 0);
#else
	pr_err("gc0339 clock is not set.\n");
	return 0;
#endif

#endif //ASUS_BSP --
}

#if defined(CONFIG_ME176C)
int gc0339_set_gpio(int flag)
{
	int ret = 0;

    if (camera_power_down < 0) {
			ret = camera_sensor_gpio(CAMERA_2_PWDN, NULL, GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        camera_power_down = ret;
    }

	gpio_set_value(camera_power_down, flag);
    printk("<< camera_power_down:%d flag:%d\n", camera_power_down, flag);

	return ret;
}
#else
int gc0339_set_gpio(int flag)
{
	int ret = 0;

    if (camera_power_down < 0) {
			ret = camera_sensor_gpio(CAMERA_1_PWDN, NULL, GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        camera_power_down = ret;
    }

	gpio_set_value(camera_power_down, flag);
    printk("<< camera_power_down:%d flag:%d\n", camera_power_down, flag);

	return ret;
}
#endif
void gc0339_free_gpio()
{
	printk("%s: camera_power_down(%d)\n",__func__,camera_power_down);

	if (camera_power_down >= 0){
		gpio_free(camera_power_down);
		camera_power_down = -1;
		mdelay(1);
	}
}

static int gc0339_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	//gc0339_gpio_init();

	if (flag){
		//hm2056_set_gpio(0,0);
		//pull high reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 1);
			printk("%s camera_reset = 1\n", __func__);
			msleep(10);
		}
	}else{
		//pull high power down
		if (camera_power_down >= 0){
			gpio_set_value(camera_power_down, 0);
			printk("%s camera_power_down = 1\n", __func__);
			msleep(10);
		}

		//pull low reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("%s camera_reset = 0\n", __func__);
			msleep(10);
		}
		//hm2056_set_gpio(0,0);
		if (camera_reset >= 0){
			gpio_free(camera_reset);
			camera_reset = -1;
			mdelay(1);
		}

		if (camera_power_down >= 0){
			gpio_free(camera_power_down);
			camera_power_down = -1;
			mdelay(1);
		}
	}
	//hm2056_free_gpio();
	//gc0339_i2c_gpio_set_alt(flag);

	return ret;
}

static int gc0339_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

        gc0339_gpio_init();
	if (flag){
                //pull low power down first
                if (camera_power_down >= 0){
                        gpio_set_value(camera_power_down, 0);
                        printk("%s camera_power_down = 0\n", __func__);
                        msleep(5);
                }
                if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("%s camera_reset = 0\n", __func__);
			msleep(10);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
#ifdef CONFIG_CRYSTAL_COVE
                        printk("%s turn on 1.8V\n", __func__ );
			ret = camera_set_pmic_power(CAMERA_1P8V, true);
			if (ret)
				return ret;
                        printk("%s turn on 2.8V\n", __func__ );
			ret = camera_set_pmic_power(CAMERA_2P8V, true);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("gc0339 power is not set.\n");
#endif
			printk("<<< %s 1.8V and 2.8V = 1\n",__FUNCTION__);
			msleep(1);
		}

		//turn on MCLK
                printk("%s set EXTCLK 19.2MHz\n", __func__ );
		gc0339_flisclk_ctrl(sd, 1);
                printk("%s set GC0339 GPIO CONTROL\n", __func__ );
		gc0339_gpio_ctrl(sd, flag);

		msleep(10); 
	}else{
		//turn off MCLK
                printk("%s release EXTCLK 19.2MHz\n", __func__ );
		gc0339_flisclk_ctrl(sd, 0);
                printk("%s release GC0339 GPIO CONTROL\n", __func__ );
		gc0339_gpio_ctrl(sd, flag);
		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
#ifdef CONFIG_CRYSTAL_COVE
                        printk("%s turn off 2.8V\n", __func__ );
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			if (ret)
				return ret;
                        printk("%s turn off 1.8V\n", __func__ );
			ret = camera_set_pmic_power(CAMERA_1P8V, false);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("gc0339 power is not set.\n");
#endif
			printk("<<< %s 1.8V and 2.8V = 0\n",__FUNCTION__);
			msleep(10);
		}
	}

	return 0;

}

static int gc0339_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
#if defined(CONFIG_ME176C)
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
#else
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
                ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
#endif
}

static int gc0339_platform_init(struct i2c_client *client)
{
	return 0;
}

static int gc0339_platform_deinit(void)
{
	return 0;
}

static struct camera_sensor_platform_data gc0339_sensor_platform_data = {
	.gpio_ctrl	 = gc0339_gpio_ctrl,
	.flisclk_ctrl	 = gc0339_flisclk_ctrl,
	.power_ctrl	 = gc0339_power_ctrl,
	.csi_cfg	 = gc0339_csi_configure,
	//.platform_init   = gc0339_platform_init,
	//.platform_deinit = gc0339_platform_deinit,
};

void *gc0339_platform_data(void *info)
{
    camera_I2C_3_SCL = -1;
    camera_I2C_3_SDA = -1;

	camera_power_down = -1;
	camera_reset = -1;
	camera_vprog1_on = 0;
	return &gc0339_sensor_platform_data;
}
