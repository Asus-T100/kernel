/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include <linux/io.h>

#include "lm3554.h"

#define to_lm3554_priv(p_sd) \
	container_of(p_sd, struct lm3554_priv, sd)

#define INIT_FIELD(_reg_address, _lsb, _msb) { \
	.reg_address = _reg_address, \
	.lsb = _lsb, \
	.msb = _msb \
}

struct lm3554_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl) (struct v4l2_subdev *sd, __u32 val);
	int (*g_ctrl) (struct v4l2_subdev *sd, __s32 *val);
};

struct lm3554_priv {
	struct v4l2_subdev sd;
	struct mutex i2c_mutex;
	enum atomisp_flash_mode mode;
	int timeout;
	struct timer_list flash_off_delay;
	u32 intensity;
	struct camera_flash_platform_data *platform_data;
};

struct lm3554_reg_field {
	u32 reg_address;
	u32 lsb;
	u32 msb;
};

/* This defines the register field properties. The LSBs and MSBs come from
 * the lm3554 datasheet. */
static const struct lm3554_reg_field
	torch_mode         = INIT_FIELD(LM3554_TORCH_BRIGHTNESS_REG, 0, 2),
	torch_current      = INIT_FIELD(LM3554_TORCH_BRIGHTNESS_REG, 3, 5),
	indicator_current  = INIT_FIELD(LM3554_TORCH_BRIGHTNESS_REG, 6, 7),
	flash_mode         = INIT_FIELD(LM3554_FLASH_BRIGHTNESS_REG, 0, 2),
	flash_current      = INIT_FIELD(LM3554_FLASH_BRIGHTNESS_REG, 3, 6),
	strobe_sensitivity = INIT_FIELD(LM3554_FLASH_BRIGHTNESS_REG, 7, 7),
	flash_timeout      = INIT_FIELD(LM3554_FLASH_DURATION_REG,   0, 4),
	current_limit      = INIT_FIELD(LM3554_FLASH_DURATION_REG,   5, 6),
	flags              = INIT_FIELD(LM3554_FLAGS_REG,            0, 7),
	envm_tx2           = INIT_FIELD(LM3554_CONFIG_REG_1,         5, 5),
	tx2_polarity       = INIT_FIELD(LM3554_CONFIG_REG_1,         6, 6),
	tx2_shutdown       = INIT_FIELD(LM3554_CONFIG_REG_2,         0, 0);

static int set_reg_field(struct v4l2_subdev *sd,
			 const struct lm3554_reg_field *field,
			 u8 value)
{
	int ret;
	u32 tmp,
	    val = value,
	    bits = (field->msb - field->lsb) + 1,
	    mask = ((1<<bits)-1) << field->lsb;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);

	mutex_lock(&p_lm3554_priv->i2c_mutex);
	tmp = i2c_smbus_read_byte_data(client, field->reg_address);
	tmp &= ~mask;
	val = (val << field->lsb) & mask;
	ret = i2c_smbus_write_byte_data(client, field->reg_address, val | tmp);
	mutex_unlock(&p_lm3554_priv->i2c_mutex);

	if (ret < 0)
		dev_err(&client->dev, "%s: flash i2c fail", __func__);

	return ret;
}

static void get_reg_field(struct v4l2_subdev *sd,
			  const struct lm3554_reg_field *field,
			  u8 *value)
{
	u32 tmp,
	    bits = (field->msb - field->lsb) + 1,
	    mask = ((1<<bits)-1) << field->lsb;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);

	mutex_lock(&p_lm3554_priv->i2c_mutex);
	tmp = i2c_smbus_read_byte_data(client, field->reg_address);
	mutex_unlock(&p_lm3554_priv->i2c_mutex);

	*value = (tmp & mask) >> field->lsb;
}

static int set_gpio_output(int gpio, const char *name, int val)
{
	int ret = gpio_request(gpio, name);
	if (ret < 0)
		return ret;
	ret = gpio_direction_output(gpio, val);
	gpio_free(gpio);
	return ret;
}

static int lm3554_hw_reset(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	struct camera_flash_platform_data *pdata = p_lm3554_priv->platform_data;
	int ret;

	ret = gpio_request(pdata->gpio_reset, "flash reset");
	if (ret < 0)
		return ret;
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(50);
	gpio_set_value(pdata->gpio_reset, 1);
	msleep(50);
	gpio_free(pdata->gpio_reset);
	return ret;
}

static int lm3554_s_flash_timeout(struct v4l2_subdev *sd, u32 val)
{
	struct lm3554_priv *p_lm3554 = to_lm3554_priv(sd);

	val = clamp(val, LM3554_MIN_TIMEOUT, LM3554_MAX_TIMEOUT);
	p_lm3554->timeout = val;

	val = val / LM3554_TIMEOUT_STEPSIZE - 1;
	return set_reg_field(sd, &flash_timeout, (u8)val);
}

static int lm3554_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	u8 value;

	get_reg_field(sd, &flash_timeout, &value);
	*val = (u32)(value + 1) * LM3554_TIMEOUT_STEPSIZE;

	return 0;
}

static int lm3554_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);

	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_FLASH_STEP);

	p_lm3554_priv->intensity = intensity;

	return set_reg_field(sd, &flash_current, (u8)intensity);
}

static int lm3554_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{
	u8 value;

	get_reg_field(sd, &flash_current, &value);
	*val = LM3554_VALUE_TO_PERCENT((u32)value, LM3554_FLASH_STEP);

	return 0;
}

static int lm3554_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_TORCH_STEP);

	return set_reg_field(sd, &torch_current, (u8)intensity);
}

static int lm3554_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{
	u8 value;

	get_reg_field(sd, &torch_current, &value);
	*val = LM3554_VALUE_TO_PERCENT((u32)value, LM3554_TORCH_STEP);

	return 0;
}

static int lm3554_s_indicator_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_INDICATOR_STEP);

	return set_reg_field(sd, &indicator_current, (u8)intensity);
}

static int lm3554_g_indicator_intensity(struct v4l2_subdev *sd, s32 *val)
{
	u8 value;

	get_reg_field(sd, &indicator_current, &value);
	*val = LM3554_VALUE_TO_PERCENT((u32)value, LM3554_INDICATOR_STEP);

	return 0;
}

static int lm3554_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	int ret, timer_pending;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	struct camera_flash_platform_data *pdata = p_lm3554_priv->platform_data;

	/*
	 * An abnormal high flash current is observed when strobe off the
	 * flash. Workaround here is firstly set flash current to lower level,
	 * wait a short moment, and then strobe off the flash.
	 */

	timer_pending = del_timer_sync(&p_lm3554_priv->flash_off_delay);

	/* Flash off */
	if (!val) {
		/* set current to 70mA and wait a while */
		ret = set_reg_field(sd, &flash_current, 0);
		if (ret < 0)
			goto err;
		mod_timer(&p_lm3554_priv->flash_off_delay,
			  jiffies + msecs_to_jiffies(LM3554_TIMER_DELAY));
		return 0;
	}

	/* Flash on */

	/*
	 * If timer is killed before run, flash is not strobe off,
	 * so must strobe off here
	 */
	if (timer_pending != 0) {
		ret = set_gpio_output(pdata->gpio_strobe, "flash", 0);
		if (ret < 0)
			goto err;
	}

	/* Restore flash current settings */
	ret = set_reg_field(sd, &flash_current,
			    (u8)p_lm3554_priv->intensity);
	if (ret < 0)
		goto err;

	/* Strobe on Flash */
	ret = set_gpio_output(pdata->gpio_strobe, "flash", val);
	if (ret < 0)
		goto err;

	return 0;
err:
	dev_err(&client->dev, "failed to generate flash strobe (%d)\n",
		ret);
	return ret;
}

static int lm3554_s_flash_mode(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	enum atomisp_flash_mode new_mode = (enum atomisp_flash_mode)val;
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);

	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		if (p_lm3554_priv->mode == ATOMISP_FLASH_MODE_FLASH) {
			ret = set_reg_field(sd, &flash_mode,
					    LM3554_MODE_SHUTDOWN);
		} else {
			ret = set_reg_field(sd, &torch_mode,
					    LM3554_MODE_SHUTDOWN);
		}
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		ret = set_reg_field(sd, &flash_mode, LM3554_MODE_FLASH);
		break;
	case ATOMISP_FLASH_MODE_INDICATOR:
		ret = set_reg_field(sd, &flash_mode, LM3554_MODE_INDICATOR);
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		ret = set_reg_field(sd, &torch_mode, LM3554_MODE_TORCH);
		break;
	default:
		ret = -EINVAL;
	}
	if (ret == 0)
		p_lm3554_priv->mode = new_mode;
	return ret;
}

static int lm3554_g_flash_mode(struct v4l2_subdev *sd, s32 * val)
{
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	*val = p_lm3554_priv->mode;
	return 0;
}

static int lm3554_g_flash_status(struct v4l2_subdev *sd, s32 *val)
{
	u8 value;

	get_reg_field(sd, &flags, &value);

	/*
	 * do not take TX1/TX2 signal as an error.
	 * because MSIC will not turn off flash, but turn to
	 * torch mode according to gsm modem signal by hardware.
	 */
	if (value & LM3554_FLAG_TIMEOUT)
		*val = ATOMISP_FLASH_STATUS_TIMEOUT;
	else if (value & LM3554_FLAG_THERMAL_SHUTDOWN ||
			value & LM3554_FLAG_LED_FAULT ||
			value & LM3554_FLAG_LED_THERMAL_FAULT ||
			value & LM3554_FLAG_INPUT_VOLTAGE_LOW) {
		*val = ATOMISP_FLASH_STATUS_HW_ERROR;
	} else
		*val = ATOMISP_FLASH_STATUS_OK;

	if (*val == ATOMISP_FLASH_STATUS_HW_ERROR) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		dev_dbg(&client->dev, "LM3554 flag status: %d\n", value);
	}
	return 0;
}

static const struct lm3554_ctrl_id lm3554_ctrls[] = {
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TIMEOUT,
				"Flash Timeout",
				0,
				LM3554_MAX_TIMEOUT,
				1,
				LM3554_DEFAULT_TIMEOUT,
				0,
				lm3554_s_flash_timeout,
				lm3554_g_flash_timeout),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INTENSITY,
				"Flash Intensity",
				LM3554_MIN_PERCENT,
				LM3554_MAX_PERCENT,
				1,
				LM3554_FLASH_DEFAULT_BRIGHTNESS,
				0,
				lm3554_s_flash_intensity,
				lm3554_g_flash_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TORCH_INTENSITY,
				"Torch Intensity",
				LM3554_MIN_PERCENT,
				LM3554_MAX_PERCENT,
				1,
				LM3554_TORCH_DEFAULT_BRIGHTNESS,
				0,
				lm3554_s_torch_intensity,
				lm3554_g_torch_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INDICATOR_INTENSITY,
				"Indicator Intensity",
				LM3554_MIN_PERCENT,
				LM3554_MAX_PERCENT,
				1,
				LM3554_INDICATOR_DEFAULT_BRIGHTNESS,
				0,
				lm3554_s_indicator_intensity,
				lm3554_g_indicator_intensity),
	s_ctrl_id_entry_boolean(V4L2_CID_FLASH_STROBE,
				"Flash Strobe",
				0,
				0,
				lm3554_s_flash_strobe,
				NULL),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_MODE,
				"Flash Mode",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_MODE_OFF,
				0,
				lm3554_s_flash_mode,
				lm3554_g_flash_mode),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS,
				"Flash Status",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_STATUS_OK,
				0,
				NULL,
				lm3554_g_flash_status),
};

static const struct lm3554_ctrl_id *find_ctrl_id(unsigned int id)
{
	int i;
	int num;

	num = ARRAY_SIZE(lm3554_ctrls);
	for (i = 0; i < num; i++) {
		if (lm3554_ctrls[i].qc.id == id)
			return &lm3554_ctrls[i];
	}

	return NULL;
}

static int lm3554_g_chip_ident(struct v4l2_subdev *sd,
			  struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	chip->ident = LEDFLASH_LM3554_ID;
	chip->revision = 0;
	chip->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
	chip->match.addr = client->addr;
	strlcpy(chip->match.name, client->name, strlen(client->name));

	return 0;
}

static int lm3554_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct i2c_client *client;
	int num;

	client = v4l2_get_subdevdata(sd);

	if (!qc)
		return -EINVAL;

	num = ARRAY_SIZE(lm3554_ctrls);
	if (qc->id >= num)
		return -EINVAL;

	*qc = lm3554_ctrls[qc->id].qc;

	return 0;
}

static int lm3554_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct lm3554_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (!s_ctrl)
		return -EINVAL;

	return s_ctrl->s_ctrl(sd, ctrl->value);
}

static int lm3554_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct lm3554_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (s_ctrl == NULL)
		return -EINVAL;

	return s_ctrl->g_ctrl(sd, &ctrl->value);
}

static int lm3554_s_power(struct v4l2_subdev *sd, int power)
{
	sd = sd;
	power = power;
	return 0;
}

static const struct v4l2_subdev_core_ops lm3554_core_ops = {
	.g_chip_ident = lm3554_g_chip_ident,
	.queryctrl = lm3554_queryctrl,
	.g_ctrl = lm3554_g_ctrl,
	.s_ctrl = lm3554_s_ctrl,
	.s_power = lm3554_s_power,
};

static const struct v4l2_subdev_ops lm3554_ops = {
	.core = &lm3554_core_ops,
};

static const struct media_entity_operations lm3554_entity_ops = {
/*	.set_power = v4l2_subdev_set_power,	*/
};

static const struct i2c_device_id lm3554_id[] = {
	{LEDFLASH_LM3554_NAME, 0},
	{},
};

static int lm3554_detect(struct i2c_client *client)
{
	s32 status;
	struct i2c_adapter *adapter = client->adapter;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	struct camera_flash_platform_data *pdata = p_lm3554_priv->platform_data;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "lm3554_detect i2c error\n");
		return -ENODEV;
	}

	lm3554_hw_reset(client);

	ret = set_gpio_output(pdata->gpio_strobe, "flash", 0);
	if (ret < 0)
		goto fail;

	ret = set_gpio_output(pdata->gpio_torch, "torch", 0);
	if (ret < 0)
		goto fail;

	/* Set to TX2 mode, then ENVM/TX2 pin is a power
	 * amplifier sync input:
	 * ENVM/TX pin asserted, flash forced into torch;
	 * ENVM/TX pin desserted, flash set back;
	 */
	ret = set_reg_field(sd, &envm_tx2, 1);
	if (ret < 0)
		goto fail;

	ret = set_reg_field(sd, &tx2_polarity, 0);
	if (ret < 0)
		goto fail;

	/* set peak current limit to be 1000mA */
	ret = set_reg_field(sd, &current_limit, 0);
	if (ret < 0)
		goto fail;

	/* clear the flags register */
	ret = lm3554_g_flash_status(sd, &status);
	if (ret < 0)
		goto fail;

	dev_dbg(&client->dev, "Successfully detected lm3554 LED flash\n");
	return 0;

fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}

static void lm3554_flash_off_delay(long unsigned int arg)
{
	struct i2c_client *client = (struct i2c_client *)arg;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	struct camera_flash_platform_data *pdata = p_lm3554_priv->platform_data;

	if (set_gpio_output(pdata->gpio_strobe, "flash", 0) < 0)
		dev_err(&client->dev, "failed to flash strobe off\n");
}

static int __devinit lm3554_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	int err;
	struct lm3554_priv *p_lm3554_priv;

	p_lm3554_priv = kzalloc(sizeof(*p_lm3554_priv), GFP_KERNEL);
	if (!p_lm3554_priv) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	p_lm3554_priv->platform_data = client->dev.platform_data;
	if (!p_lm3554_priv->platform_data) {
		dev_err(&client->dev, "no platform data\n");
		kfree(p_lm3554_priv);
		return -ENODEV;
	}

	v4l2_i2c_subdev_init(&(p_lm3554_priv->sd), client, &lm3554_ops);
	p_lm3554_priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	p_lm3554_priv->sd.entity.ops = &lm3554_entity_ops;
	p_lm3554_priv->timeout = LM3554_DEFAULT_TIMEOUT;
	p_lm3554_priv->mode = ATOMISP_FLASH_MODE_OFF;
	p_lm3554_priv->intensity = 0;

	err = media_entity_init(&p_lm3554_priv->sd.entity, 0, NULL, 0);
	if (err) {
		dev_err(&client->dev, "error initialize a media entity.\n");
		goto fail1;
	}

	mutex_init(&p_lm3554_priv->i2c_mutex);

	setup_timer(&p_lm3554_priv->flash_off_delay, lm3554_flash_off_delay,
		    (unsigned long)client);

	err = lm3554_detect(client);
	if (err) {
		dev_err(&client->dev, "device not found\n");
		goto fail2;
	}

	return 0;
fail2:
	media_entity_cleanup(&p_lm3554_priv->sd.entity);
fail1:
	v4l2_device_unregister_subdev(&p_lm3554_priv->sd);
	kfree(p_lm3554_priv);

	return err;
}

static int lm3554_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554_priv *p_lm3554_priv = to_lm3554_priv(sd);
	struct camera_flash_platform_data *pdata = p_lm3554_priv->platform_data;
	int ret;

	media_entity_cleanup(&p_lm3554_priv->sd.entity);
	v4l2_device_unregister_subdev(sd);

	del_timer_sync(&p_lm3554_priv->flash_off_delay);

	ret = set_gpio_output(pdata->gpio_strobe, "flash", 0);
	if (ret < 0)
		goto fail;

	kfree(p_lm3554_priv);

	return 0;
fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}

MODULE_DEVICE_TABLE(i2c, lm3554_id);

static struct i2c_driver lm3554_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LEDFLASH_LM3554_NAME,
	},
	.probe = lm3554_probe,
	.remove = lm3554_remove,
	.id_table = lm3554_id,
};

static __init int init_lm3554(void)
{
	return i2c_add_driver(&lm3554_driver);
}

static __exit void exit_lm3554(void)
{
	i2c_del_driver(&lm3554_driver);
}

module_init(init_lm3554);
module_exit(exit_lm3554);
MODULE_AUTHOR("Jing Tao <jing.tao@intel.com>");
MODULE_LICENSE("GPL");
