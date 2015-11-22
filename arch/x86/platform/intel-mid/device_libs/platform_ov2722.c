/*
 * platform_ov2722.c: ov2722 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/atomisp_platform.h>
#include <linux/regulator/consumer.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/mfd/intel_mid_pmic.h>

#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif

#include "platform_camera.h"
#include "platform_ov2722.h"

/* workround - pin defined for byt */
#define CAMERA_1_RESET 127
#define CAMERA_1_RESET_CHT 148
#define CAMERA_1_RESET_CRV2 120
#define CAMERA_1_PWDN 124
#define CAMERA_1_PWDN_CHT 152
#define CAMERA_1P8_EN_CHT	153
#ifdef CONFIG_INTEL_SOC_PMC
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
/* workaround - use xtal for cht */
#define CLK_19P2MHz_XTAL 0x0
#define CLK_ON	0x1
#define CLK_OFF	0x2
#endif
/* PWDN - LOW active */
#define IS_PWDN_LOW_ACTIVE \
	(spid.hardware_id == BYT_TABLET_BLK_8PR0 ||\
	spid.hardware_id == BYT_TABLET_BLK_8PR1 ||\
	spid.hardware_id == BYT_TABLET_BLK_CRV2 ||\
	spid.hardware_id == CHT_TABLET_FRD_PR0 ||\
	spid.hardware_id == CHT_TABLET_FRD_PR1 ||\
	spid.hardware_id == CHT_TABLET_FRD_PR2 ||\
	spid.hardware_id == CHT_TABLET_RVP1 ||\
	spid.hardware_id == CHT_TABLET_RVP2 ||\
	spid.hardware_id == CHT_TABLET_RVP3)

#ifdef CONFIG_CRYSTAL_COVE
#define ALDO1_SEL_REG	0x28
#define ALDO1_CTRL3_REG	0x13
#define ALDO1_2P8V	0x16
#define ALDO1_CTRL3_SHIFT 0x05

#define ELDO2_SEL_REG	0x1a
#define ELDO2_CTRL2_REG 0x12
#define ELDO2_1P8V	0x16
#define ELDO2_CTRL2_SHIFT 0x01

#define LDO9_REG	0x49
#define LDO9_2P8V_ON	0x2f
#define LDO9_2P8V_OFF	0x2e

#define LDO10_REG	0x4a
#define LDO10_1P8V_ON	0x59
#define LDO10_1P8V_OFF	0x58

static struct regulator *v1p8_reg;
static struct regulator *v2p8_reg;

/* PMIC HID */
#define PMIC_HID_ROHM	"INT33FD:00"
#define PMIC_HID_XPOWER	"INT33F4:00"
#define PMIC_HID_TI	"INT33F5:00"

enum pmic_ids {
	PMIC_ROHM = 0,
	PMIC_XPOWER,
	PMIC_TI,
	PMIC_MAX
};

static enum pmic_ids pmic_id;
#endif

static int camera_vprog1_on;
static int camera_1p8_en;
static int gp_camera1_power_down;
static int gp_camera1_reset;

/*
 * OV2722 platform data
 */

#ifdef CONFIG_CRYSTAL_COVE
static int match_name(struct device *dev, void *data)
{
	const char *name = data;
	struct i2c_client *client = i2c_verify_client(dev);
	return client ? !strncmp(client->name, name, strlen(name)) : 0;
}

static struct i2c_client *i2c_find_client_by_name(char *name)
{
	struct device *dev = bus_find_device(&i2c_bus_type, NULL,
						name, match_name);
	return dev ? to_i2c_client(dev) : NULL;
}

static enum pmic_ids camera_pmic_probe()
{
	/* search by client name */
	struct i2c_client *client;
	if (spid.hardware_id != BYT_TABLET_BLK_CRV2 ||
		i2c_find_client_by_name(PMIC_HID_ROHM))
		return PMIC_ROHM;

	client = i2c_find_client_by_name(PMIC_HID_XPOWER);
	if (client)
		return PMIC_XPOWER;

	client = i2c_find_client_by_name(PMIC_HID_TI);
	if (client)
		return PMIC_TI;

	return PMIC_MAX;
}

static int camera_pmic_set(bool flag)
{
	int val;
	int ret = 0;
	if (pmic_id == PMIC_MAX) {
		pmic_id = camera_pmic_probe();
		if (pmic_id == PMIC_MAX)
			return -EINVAL;
	}

	if (flag) {
		switch (pmic_id) {
		case PMIC_ROHM:
			ret = regulator_enable(v2p8_reg);
			if (ret)
				return ret;

			ret = regulator_enable(v1p8_reg);
			if (ret)
				regulator_disable(v2p8_reg);
			break;
		case PMIC_XPOWER:
			/* ALDO1 */
			ret = intel_mid_pmic_writeb(ALDO1_SEL_REG, ALDO1_2P8V);
			if (ret)
				return ret;

			/* PMIC Output CTRL 3 for ALDO1 */
			val = intel_mid_pmic_readb(ALDO1_CTRL3_REG);
			val |= (1 << ALDO1_CTRL3_SHIFT);
			ret = intel_mid_pmic_writeb(ALDO1_CTRL3_REG, val);
			if (ret)
				return ret;

			/* ELDO2 */
			ret = intel_mid_pmic_writeb(ELDO2_SEL_REG, ELDO2_1P8V);
			if (ret)
				return ret;

			/* PMIC Output CTRL 2 for ELDO2 */
			val = intel_mid_pmic_readb(ELDO2_CTRL2_REG);
			val |= (1 << ELDO2_CTRL2_SHIFT);
			ret = intel_mid_pmic_writeb(ELDO2_CTRL2_REG, val);
			break;
		case PMIC_TI:
			/* LDO9 */
			ret = intel_mid_pmic_writeb(LDO9_REG, LDO9_2P8V_ON);
			if (ret)
				return ret;

			/* LDO10 */
			ret = intel_mid_pmic_writeb(LDO10_REG, LDO10_1P8V_ON);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}

	} else {
		switch (pmic_id) {
		case PMIC_ROHM:
			ret = regulator_disable(v2p8_reg);
			ret += regulator_disable(v1p8_reg);
			break;
		case PMIC_XPOWER:
			val = intel_mid_pmic_readb(ALDO1_CTRL3_REG);
			val &= ~(1 << ALDO1_CTRL3_SHIFT);
			ret = intel_mid_pmic_writeb(ALDO1_CTRL3_REG, val);
			if (ret)
				return ret;

			val = intel_mid_pmic_readb(ELDO2_CTRL2_REG);
			val &= ~(1 << ELDO2_CTRL2_SHIFT);
			ret = intel_mid_pmic_writeb(ELDO2_CTRL2_REG, val);
			break;
		case PMIC_TI:
			/* LDO9 */
			ret = intel_mid_pmic_writeb(LDO9_REG, LDO9_2P8V_OFF);
			if (ret)
				return ret;

			/* LDO10 */
			ret = intel_mid_pmic_writeb(LDO10_REG, LDO10_1P8V_OFF);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}
	}
	return ret;
}
#endif

static int ov2722_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;

	if (!IS_BYT && !IS_CHT) {
		if (gp_camera1_power_down < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			gp_camera1_power_down = ret;
		}

		if (gp_camera1_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			gp_camera1_reset = ret;
		}
	} else {
		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently.
		 */
		if (spid.hardware_id == BYT_TABLET_BLK_CRV2)
			pin = CAMERA_1_RESET_CRV2;
		else if (IS_CHT)
			pin = CAMERA_1_RESET_CHT;
		else
			pin = CAMERA_1_RESET;

		if (gp_camera1_reset < 0) {
			ret = gpio_request(pin, "camera_1_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		gp_camera1_reset = pin;
		ret = gpio_direction_output(pin, 1);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}

		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently.
		 */
		if (IS_CHT)
			pin = CAMERA_1_PWDN_CHT;
		else
			pin = CAMERA_1_PWDN;
		if (gp_camera1_power_down < 0) {
			ret = gpio_request(pin, "camera_1_power");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		gp_camera1_power_down = pin;

		if (IS_PWDN_LOW_ACTIVE)
			ret = gpio_direction_output(pin, 0);
		else
			ret = gpio_direction_output(pin, 1);

		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
	if (flag) {
		if (IS_PWDN_LOW_ACTIVE)
			gpio_set_value(gp_camera1_power_down, 0);
		else
			gpio_set_value(gp_camera1_power_down, 1);

		gpio_set_value(gp_camera1_reset, 0);
		msleep(20);
		gpio_set_value(gp_camera1_reset, 1);
	} else {
		gpio_set_value(gp_camera1_reset, 0);
		if (IS_PWDN_LOW_ACTIVE)
			gpio_set_value(gp_camera1_power_down, 1);
		else
			gpio_set_value(gp_camera1_power_down, 0);
	}

	return 0;
}

static int ov2722_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_INTEL_SOC_PMC
	int ret = 0;
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM1_CLK, (IS_CHT) ?
			CLK_19P2MHz_XTAL : CLK_19P2MHz);
		if (ret)
			return ret;
		return pmc_pc_configure(OSC_CAM1_CLK, CLK_ON);
	}
	return pmc_pc_configure(OSC_CAM1_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
				     flag ? clock_khz : 0);
#else
	pr_err("ov2722 clock is not set.\n");
	return 0;
#endif
}

/*
 * The power_down gpio pin is to control OV2722's
 * internal power state.
 */
static int ov2722_power_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_CRYSTAL_COVE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	int ret = 0;
	int pin = CAMERA_1P8_EN_CHT;

	/*
	 * FIXME: VRF has no implementation for CHT now,
	 * remove pmic power control when VRF is ready.
	 */
#ifdef CONFIG_CRYSTAL_COVE
	if (IS_CHT) {
		if (camera_1p8_en < 0) {
			ret = gpio_request(pin, "camera_v1p8_en");
			if (ret) {
				pr_err("Request camera_v1p8_en failed.\n");
				return ret;
			}
			camera_1p8_en = pin;
		}
		if (flag) {
			if (!camera_vprog1_on) {
				ret = camera_set_pmic_power(CAMERA_2P8V, true);
				if (ret) {
					dev_err(&client->dev,
						"Failed to enable pmic power v2p8\n");
					return ret;
				}

				ret = camera_set_pmic_power(CAMERA_1P8V, true);
				if (ret) {
					camera_set_pmic_power(CAMERA_2P8V, false);
					dev_err(&client->dev,
						"Failed to enable pmic power v1p8\n");
				}
				ret = gpio_direction_output(pin, 1);
				if (ret) {
					dev_err(&client->dev,
						"%s: failed to set gpio(pin %d) direction\n",
						__func__, pin);
					gpio_free(pin);
					return ret;
				}
				camera_vprog1_on = 1;
			}
		} else {
			if (camera_vprog1_on) {
				ret = camera_set_pmic_power(CAMERA_2P8V, false);
				if (ret)
					dev_warn(&client->dev,
						 "Failed to disable pmic power v2p8\n");
				ret = camera_set_pmic_power(CAMERA_1P8V, false);
				if (ret)
					dev_warn(&client->dev,
						 "Failed to disable pmic power v1p8\n");
				gpio_set_value(pin, 0);
				gpio_free(pin);
				camera_1p8_en = -1;
				camera_vprog1_on = 0;
			}
		}
		return ret;
	}
#endif
	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_pmic_set(flag);
			if (ret) {
				dev_err(&client->dev,
						"Failed to enable regulator\n");
				return ret;
			}
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("ov2722 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 1;
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_pmic_set(flag);
			if (ret) {
				dev_err(&client->dev,
						"Failed to enable regulator\n");
				return ret;
			}
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("ov2722 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}

static int ov2722_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

#ifdef CONFIG_CRYSTAL_COVE
static int ov2722_platform_init(struct i2c_client *client)
{
	if (IS_CHT)
		return 0;

	pmic_id = camera_pmic_probe();
	if (pmic_id != PMIC_ROHM)
		return 0;

	v1p8_reg = regulator_get(&client->dev, "v1p8sx");
	if (IS_ERR(v1p8_reg)) {
		dev_err(&client->dev, "v1p8s regulator_get failed\n");
		return PTR_ERR(v1p8_reg);
	}

	v2p8_reg = regulator_get(&client->dev, "v2p85sx");
	if (IS_ERR(v2p8_reg)) {
		regulator_put(v1p8_reg);
		dev_err(&client->dev, "v2p85sx regulator_get failed\n");
		return PTR_ERR(v2p8_reg);
	}

	return 0;
}

static int ov2722_platform_deinit(void)
{
	if (IS_CHT)
		return 0;
	if (pmic_id != PMIC_ROHM)
		return 0;

	regulator_put(v1p8_reg);
	regulator_put(v2p8_reg);

	return 0;
}
#endif

static struct camera_sensor_platform_data ov2722_sensor_platform_data = {
	.gpio_ctrl	= ov2722_gpio_ctrl,
	.flisclk_ctrl	= ov2722_flisclk_ctrl,
	.power_ctrl	= ov2722_power_ctrl,
	.csi_cfg	= ov2722_csi_configure,
#ifdef CONFIG_CRYSTAL_COVE
	.platform_init = ov2722_platform_init,
	.platform_deinit = ov2722_platform_deinit,
#endif
};

void *ov2722_platform_data(void *info)
{
	gp_camera1_power_down = -1;
	gp_camera1_reset = -1;
	camera_1p8_en = -1;
#ifdef CONFIG_CRYSTAL_COVE
	pmic_id = PMIC_MAX;
#endif
	return &ov2722_sensor_platform_data;
}
