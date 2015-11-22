/*
 * platform_hm2056.c: hm2056 platform data initilization file
 *
 * (C) Copyright 2013 Intel
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
#include <linux/lnw_gpio.h>
#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_hm2056.h"
#include "camera_info.h"

static int IsInFront;

static int camera_I2C_3_SCL;
static int camera_I2C_3_SDA;

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x0
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz  0x1
#endif

static int *p_camera_reset;
static int *p_camera_power_down;
static int *p_camera_vprog1_on;

static int camera_reset_b;
static int camera_power_down_b;
static int camera_vprog1_on_b;

static int camera_reset_f;
static int camera_power_down_f;
static int camera_vprog1_on_f;

#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
static int hm2056_i2c_gpio_set_alt(int flag)
{
	int ret = 0;

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
#endif

#if defined(CONFIG_ME176C)
static int hm2056_gpio_init()
{
    int ret = 0;
    if (*p_camera_reset < 0) {
	if(IsInFront==1){
	    printk("<<< hm2056_gpio_init CAMERA_2_RESET\n");
            ret = camera_sensor_gpio(CAMERA_2_RESET, NULL, GPIOF_DIR_OUT, 0);
	}else{
	    printk("<<< hm2056_gpio_init CAMERA_0_RESET\n");
            ret = camera_sensor_gpio(CAMERA_1_RESET, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_reset not available.\n");
            return ret;
        }
        *p_camera_reset = ret;
    }
    printk("%s camera_reset:%d IsInFront:%d\n", __func__, *p_camera_reset, IsInFront);

    if (*p_camera_power_down < 0) {
	if(IsInFront==1){
	    ret = camera_sensor_gpio(CAMERA_2_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}else{
	    ret = camera_sensor_gpio(CAMERA_1_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        *p_camera_power_down = ret;
    }
    printk("%s camera_power_down:%d IsInFront:%d\n", __func__, *p_camera_power_down, IsInFront);

    return ret;
}
#else
static int hm2056_gpio_init()
{
    int ret = 0;
    if (*p_camera_reset < 0) {
	if(IsInFront==1){
	    printk("<<< hm2056_gpio_init CAMERA_1_RESET\n");
            ret = camera_sensor_gpio(CAMERA_1_RESET, NULL, GPIOF_DIR_OUT, 0);
	}else{
	    printk("<<< hm2056_gpio_init CAMERA_0_RESET\n");
            ret = camera_sensor_gpio(CAMERA_0_RESET, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_reset not available.\n");
            return ret;
        }
        *p_camera_reset = ret;
    }
    printk("%s camera_reset:%d IsInFront:%d\n", __func__, *p_camera_reset, IsInFront);

    if (*p_camera_power_down < 0) {
	if(IsInFront==1){
	    ret = camera_sensor_gpio(CAMERA_1_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}else{
	    ret = camera_sensor_gpio(CAMERA_0_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        *p_camera_power_down = ret;
    }
    printk("%s camera_power_down:%d IsInFront:%d\n", __func__, *p_camera_power_down, IsInFront);

    return ret;
}
#endif

#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
int hm2056_set_gpio(int RearOrFront, int flag)
{
	int ret = 0;

    if (*p_camera_power_down < 0) {
	if(RearOrFront==1){//Front
	    ret = camera_sensor_gpio(CAMERA_1_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}else if(RearOrFront==0){//Rear
            ret = camera_sensor_gpio(CAMERA_0_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        *p_camera_power_down = ret;
    }

    gpio_set_value(*p_camera_power_down, flag);
    printk("<< camera_power_down:%d flag:%d\n", *p_camera_power_down, flag);

    return ret;
}
#else
int hm2056_set_gpio(int RearOrFront, int flag)
{
	int ret = 0;

    if (*p_camera_power_down < 0) {
	if(RearOrFront==1){//Front
	    ret = camera_sensor_gpio(CAMERA_2_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}else if(RearOrFront==0){//Rear
            ret = camera_sensor_gpio(CAMERA_1_PWDN, NULL, GPIOF_DIR_OUT, 0);
	}
        if (ret < 0){
            printk("camera_power_down not available.\n");
            return ret;
        }
        *p_camera_power_down = ret;
    }

    gpio_set_value(*p_camera_power_down, flag);
    printk("<< camera_power_down:%d flag:%d\n", *p_camera_power_down, flag);

    return ret;
}
#endif

void hm2056_free_gpio()
{
	printk("%s: camera_power_down(%d)\n",__func__,*p_camera_power_down);

	if (*p_camera_power_down >= 0){
		gpio_free(*p_camera_power_down);
		*p_camera_power_down = -1;
		mdelay(1);
	}
}

static int hm2056_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
    hm2056_gpio_init();

    if (flag){
	//gc0339_set_gpio(0);
        if (*p_camera_power_down >= 0){
            gpio_set_value(*p_camera_power_down, 0);
            printk("%s camera_power_down = 0\n", __func__);
            msleep(1);
        }
        if (*p_camera_reset >= 0){	
            gpio_set_value(*p_camera_reset, 0);
            msleep(20);
            gpio_set_value(*p_camera_reset, 1);
            printk("%s camera_reset = 1\n", __func__);
            msleep(10);
        }

    }
    else{
		if (*p_camera_power_down >= 0){
//<ASUS-Oscar140319+>  leakage protection for hw request
	        //gpio_set_value(*p_camera_power_down, 1);
	        //printk("<<< camera_power_down = 1\n");
	                gpio_set_value(*p_camera_power_down, 0);
	                printk("%s camera_power_down = 0\n", __func__);
                        mdelay(10);
//<ASUS-Oscar140319->  leakage protection for hw request
		}
		if (*p_camera_reset >= 0){
			gpio_set_value(*p_camera_reset, 0);
	                printk("%s camera_reset = 0\n", __func__);
                        mdelay(10);
		}

		if (*p_camera_reset >= 0){
			gpio_free(*p_camera_reset);
			*p_camera_reset = -1;
			mdelay(1);
		}

		if (*p_camera_power_down >= 0){
			gpio_free(*p_camera_power_down);
			*p_camera_power_down = -1;
			mdelay(1);
		}
		//gc0339_set_gpio(0);
    }
    //gc0339_free_gpio();
    mdelay(1);
    //hm2056_i2c_gpio_set_alt(flag);

    return 0;
}

#if defined(CONFIG_ME176C)
static int hm2056f_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
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
	pr_err("hm2056f clock is not set.\n");
	return 0;
#endif
}

static int hm2056b_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		int ret;
                printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_ON\n",__func__,flag);
		ret = pmc_pc_set_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM0_CLK, CLK_ON);
	}
        printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_OFF\n",__func__,flag);
	return pmc_pc_configure(OSC_CAM0_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
        static const unsigned int clock_khz = 19200;
        printk("%s: CONFIG_INTEL_SCU_IPC_UTIL\n",__func__);
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
			flag ? clock_khz : 0);
#else
	pr_err("hm2056b clock is not set.\n");
	return 0;
#endif
}
#else //ASUS_BSP++ for other project
static int hm2056_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		int ret;
                printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_ON\n",__func__,flag);
		ret = pmc_pc_set_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM0_CLK, CLK_ON);
	}
        printk("%s: flag: %d  CONFIG_INTEL_SOC_PMC: CLK_OFF\n",__func__,flag);
	return pmc_pc_configure(OSC_CAM0_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
        static const unsigned int clock_khz = 19200;
        printk("%s: CONFIG_INTEL_SCU_IPC_UTIL\n",__func__);
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
			flag ? clock_khz : 0);
#else
	pr_err("hm2056 clock is not set.\n");
	return 0;
#endif
}
#endif
static int hm2056_power_ctrl(struct v4l2_subdev *sd, int flag)
{
    int ret = 0;

    if (flag){
        //turn on power 1.8V and 2.8V
        if (!*p_camera_vprog1_on) {
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
				pr_err("hm2056 power is not set.\n");
			#endif
				if (!ret)
					*p_camera_vprog1_on = 1;			

            printk("<<< %s 1.8V and 2.8V = 1\n",__FUNCTION__);
            msleep(1);
        }

        return ret;
    }else{
        //turn off power 1.8V and 2.8V
        if (*p_camera_vprog1_on) {
			#ifdef CONFIG_CRYSTAL_COVE
                                printk("%s turn off 2.8V\n", __func__ );
				ret = camera_set_pmic_power(CAMERA_2P8V, false);
				if (ret)
					return ret;
                                printk("%s turn off 1.8V\n", __func__ );
				ret = camera_set_pmic_power(CAMERA_1P8V, false);
			#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
				ret = intel_suc_ipc_msic_vprog1(0);
			#else
				pr_err("hm2056 power is not set.\n");
			#endif
				if (!ret)
					*p_camera_vprog1_on = 0;

            printk("<<< %s 1.8V and 2.8V = 0\n",__FUNCTION__);
            msleep(1);
        }
        return ret;
    }
}

static void TurnOffAnotherSensor(struct v4l2_subdev *sd, int Front, int *reset, int *power_down, int *vprog1_on)
{
	IsInFront = Front;
	p_camera_reset = reset;
	p_camera_power_down = power_down;
	p_camera_vprog1_on = vprog1_on;

        printk("%s \n", __func__);
	//hm2056_flisclk_ctrl(sd,false);
	hm2056_gpio_init();

	if (*p_camera_power_down >= 0){
        gpio_set_value(*p_camera_power_down, 1);
        printk("%s camera_power_down = 1\n", __func__);
	}
	if (*p_camera_reset >= 0){
        mdelay(10);
		gpio_set_value(*p_camera_reset, 0);
        printk("%s camera_reset = 0\n", __func__);
	}

	if (*p_camera_reset >= 0){
		gpio_free(*p_camera_reset);
		*p_camera_reset = -1;
		mdelay(1);
	}
	
	if (*p_camera_power_down >= 0){
		gpio_free(*p_camera_power_down);
		*p_camera_power_down = -1;
		mdelay(1);
	}

}

static int hm2056b_power_ctrl(struct v4l2_subdev *sd, int flag)
{
        printk("%s \n", __func__);
	/*if(flag){
		TurnOffAnotherSensor(sd, 1, &camera_reset_f, &camera_power_down_f, &camera_vprog1_on_f);
	}*/

	IsInFront = 0;
	p_camera_reset = &camera_reset_b;
	p_camera_power_down = &camera_power_down_b;
	p_camera_vprog1_on = &camera_vprog1_on_b;

	return hm2056_power_ctrl(sd, flag);
}

static int hm2056f_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	if(flag){
		TurnOffAnotherSensor(sd, 0, &camera_reset_b, &camera_power_down_b, &camera_vprog1_on_b);
	}

	IsInFront = 1;
	p_camera_reset = &camera_reset_f;
	p_camera_power_down = &camera_power_down_f;
	p_camera_vprog1_on = &camera_vprog1_on_f;

	return hm2056_power_ctrl(sd, flag);
}

static int hm2056_csi_configure(struct v4l2_subdev *sd, int flag)
{
    static const int LANES = 1;
    enum atomisp_bayer_order bayer_order = atomisp_bayer_order_bggr;
    if (!strncmp(sd->name, "hm2056b", strlen("hm2056b"))) {
	bayer_order = atomisp_bayer_order_gbrg;
    }
#if defined(CONFIG_ME176C)
	if(IsInFront==1){
                printk("%s: Front camera\n",__func__);
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
				ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_grbg, flag);
	}else{
                printk("%s: Back camera\n",__func__);
                return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
                                ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_grbg, flag);
	}
#else
        if(IsInFront==1){
                printk("%s: Front camera\n",__func__);
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
				ATOMISP_INPUT_FORMAT_RAW_8, bayer_order, flag);
	}else{
                printk("%s: Back camera\n",__func__);
                return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
                                ATOMISP_INPUT_FORMAT_RAW_8, bayer_order, flag);
	}
#endif
}

static int hm2056b_platform_init(struct i2c_client *client)
{
    return 0;
}

static int hm2056f_platform_init(struct i2c_client *client)
{
    return 0;
}

static int hm2056_platform_deinit(void)
{
	return 0;
}

static struct camera_sensor_platform_data hm2056b_sensor_platform_data = {
    .gpio_ctrl	 = hm2056_gpio_ctrl,
#if defined(CONFIG_ME176C)
    .flisclk_ctrl	 = hm2056b_flisclk_ctrl,
#else
    .flisclk_ctrl	 = hm2056_flisclk_ctrl,
#endif
    .power_ctrl	 = hm2056b_power_ctrl,
    .csi_cfg	 = hm2056_csi_configure,
    //.platform_init   = hm2056b_platform_init,
    //.platform_deinit = hm2056_platform_deinit,
};

static struct camera_sensor_platform_data hm2056f_sensor_platform_data = {
    .gpio_ctrl	 = hm2056_gpio_ctrl,
#if defined(CONFIG_ME176C)
    .flisclk_ctrl	 = hm2056f_flisclk_ctrl,
#else
    .flisclk_ctrl	 = hm2056_flisclk_ctrl,
#endif
    .power_ctrl	 = hm2056f_power_ctrl,
    .csi_cfg	 = hm2056_csi_configure,
    //.platform_init   = hm2056f_platform_init,
    //.platform_deinit = hm2056_platform_deinit,
};

void *hm2056b_platform_data(void *info)
{
    camera_I2C_3_SCL = -1;
    camera_I2C_3_SDA = -1;

	camera_reset_b = -1;
	camera_power_down_b = -1;
	camera_vprog1_on_b = 0;
	camera_reset_f = -1;
	camera_power_down_f = -1;
	camera_vprog1_on_f = 0;

	p_camera_reset = &camera_reset_b;
	p_camera_power_down = &camera_power_down_b;
	p_camera_vprog1_on = &camera_vprog1_on_b;

    return &hm2056b_sensor_platform_data;
}
void *hm2056f_platform_data(void *info)
{
    camera_I2C_3_SCL = -1;
    camera_I2C_3_SDA = -1;

	camera_reset_b = -1;
	camera_power_down_b = -1;
	camera_vprog1_on_b = 0;
	camera_reset_f = -1;
	camera_power_down_f = -1;
	camera_vprog1_on_f = 0;

	p_camera_reset = &camera_reset_f;
	p_camera_power_down = &camera_power_down_f;
	p_camera_vprog1_on = &camera_vprog1_on_f;

    return &hm2056f_sensor_platform_data;
}

