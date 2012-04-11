/*
 * LED flash driver for LM3554
 *
 * Copyright (c) 2010-2012 Intel Corporation. All Rights Reserved.
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

#include <linux/atomisp.h>
#include <linux/atomisp_platform.h>

#include "lm3554.h"

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

struct lm3554 {
	struct v4l2_subdev sd;
	struct mutex i2c_mutex;
	enum atomisp_flash_mode mode;

	int timeout;
	u8 torch_current;
	u8 indicator_current;
	u8 flash_current;

	struct timer_list flash_off_delay;
	struct camera_flash_platform_data *pdata;
};

#define to_lm3554(p_sd)	container_of(p_sd, struct lm3554, sd)

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
	struct lm3554 *flash = to_lm3554(sd);

	mutex_lock(&flash->i2c_mutex);
	tmp = i2c_smbus_read_byte_data(client, field->reg_address);
	tmp &= ~mask;
	val = (val << field->lsb) & mask;
	ret = i2c_smbus_write_byte_data(client, field->reg_address, val | tmp);
	mutex_unlock(&flash->i2c_mutex);

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
	struct lm3554 *flash = to_lm3554(sd);

	mutex_lock(&flash->i2c_mutex);
	tmp = i2c_smbus_read_byte_data(client, field->reg_address);
	mutex_unlock(&flash->i2c_mutex);

	*value = (tmp & mask) >> field->lsb;
}

static int lm3554_hw_reset(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554 *flash = to_lm3554(sd);
	struct camera_flash_platform_data *pdata = flash->pdata;
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
	struct lm3554 *flash = to_lm3554(sd);

	val = clamp(val, LM3554_MIN_TIMEOUT, LM3554_MAX_TIMEOUT);
	val = val / LM3554_TIMEOUT_STEPSIZE - 1;

	flash->timeout = val;

	return set_reg_field(sd, &flash_timeout, (u8)val);
}

static int lm3554_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	struct lm3554 *flash = to_lm3554(sd);

	*val = (u32)(flash->timeout + 1) * LM3554_TIMEOUT_STEPSIZE;

	return 0;
}

static int lm3554_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct lm3554 *flash = to_lm3554(sd);

	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_FLASH_STEP);

	flash->flash_current = intensity;

	return set_reg_field(sd, &flash_current, (u8)intensity);
}

static int lm3554_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct lm3554 *flash = to_lm3554(sd);

	*val = LM3554_VALUE_TO_PERCENT((u32)flash->flash_current,
			LM3554_FLASH_STEP);

	return 0;
}

static int lm3554_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct lm3554 *flash = to_lm3554(sd);

	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_TORCH_STEP);

	flash->torch_current = intensity;

	return set_reg_field(sd, &torch_current, (u8)intensity);
}

static int lm3554_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct lm3554 *flash = to_lm3554(sd);

	*val = LM3554_VALUE_TO_PERCENT((u32)flash->torch_current,
			LM3554_TORCH_STEP);

	return 0;
}

static int lm3554_s_indicator_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	struct lm3554 *flash = to_lm3554(sd);

	intensity = LM3554_CLAMP_PERCENTAGE(intensity);
	intensity = LM3554_PERCENT_TO_VALUE(intensity, LM3554_INDICATOR_STEP);

	flash->indicator_current = intensity;

	return set_reg_field(sd, &indicator_current, (u8)intensity);
}

static int lm3554_g_indicator_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct lm3554 *flash = to_lm3554(sd);

	*val = LM3554_VALUE_TO_PERCENT((u32)flash->indicator_current,
			LM3554_INDICATOR_STEP);

	return 0;
}

static int lm3554_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	int ret, timer_pending;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lm3554 *flash = to_lm3554(sd);
	struct camera_flash_platform_data *pdata = flash->pdata;

	/*
	 * An abnormal high flash current is observed when strobe off the
	 * flash. Workaround here is firstly set flash current to lower level,
	 * wait a short moment, and then strobe off the flash.
	 */

	timer_pending = del_timer_sync(&flash->flash_off_delay);

	/* Flash off */
	if (!val) {
		/* set current to 70mA and wait a while */
		ret = set_reg_field(sd, &flash_current, 0);
		if (ret < 0)
			goto err;
		mod_timer(&flash->flash_off_delay,
			  jiffies + msecs_to_jiffies(LM3554_TIMER_DELAY));
		return 0;
	}

	/* Flash on */

	/*
	 * If timer is killed before run, flash is not strobe off,
	 * so must strobe off here
	 */
	if (timer_pending != 0)
		gpio_set_value(pdata->gpio_strobe, 0);

	/* Restore flash current settings */
	ret = set_reg_field(sd, &flash_current, flash->flash_current);
	if (ret < 0)
		goto err;

	/* Strobe on Flash */
	gpio_set_value(pdata->gpio_strobe, val);

	return 0;
err:
	dev_err(&client->dev, "failed to generate flash strobe (%d)\n",
		ret);
	return ret;
}

static int lm3554_s_flash_mode(struct v4l2_subdev *sd, u32 new_mode)
{
	int ret;
	struct lm3554 *flash = to_lm3554(sd);
	unsigned int mode;

	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		mode = LM3554_MODE_SHUTDOWN;
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		mode = LM3554_MODE_FLASH;
		break;
	case ATOMISP_FLASH_MODE_INDICATOR:
		mode = LM3554_MODE_INDICATOR;
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		mode = LM3554_MODE_TORCH;
		break;
	default:
		return -EINVAL;
	}
	ret = set_reg_field(sd, &flash_mode, mode);
	if (ret == 0)
		flash->mode = new_mode;
	return ret;
}

static int lm3554_g_flash_mode(struct v4l2_subdev *sd, s32 * val)
{
	struct lm3554 *flash = to_lm3554(sd);
	*val = flash->mode;
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

static int lm3554_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int num;

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
	return 0;
}

static const struct v4l2_subdev_core_ops lm3554_core_ops = {
	.queryctrl = lm3554_queryctrl,
	.g_ctrl = lm3554_g_ctrl,
	.s_ctrl = lm3554_s_ctrl,
	.s_power = lm3554_s_power,
};

static const struct v4l2_subdev_ops lm3554_ops = {
	.core = &lm3554_core_ops,
};

static int lm3554_detect(struct v4l2_subdev *sd)
{
	s32 status;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "lm3554_detect i2c error\n");
		return -ENODEV;
	}

	lm3554_hw_reset(client);

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
	return ret;
}

static int lm3554_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return lm3554_s_power(sd, 1);
}

static int lm3554_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return lm3554_s_power(sd, 0);
}

static const struct v4l2_subdev_internal_ops lm3554_internal_ops = {
	.registered = lm3554_detect,
	.open = lm3554_open,
	.close = lm3554_close,
};

static void lm3554_flash_off_delay(long unsigned int arg)
{
	struct v4l2_subdev *sd = i2c_get_clientdata((struct i2c_client *)arg);
	struct lm3554 *flash = to_lm3554(sd);
	struct camera_flash_platform_data *pdata = flash->pdata;

	gpio_set_value(pdata->gpio_strobe, 0);
}

static int __devinit lm3554_gpio_init(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554 *flash = to_lm3554(sd);
	struct camera_flash_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_request(pdata->gpio_strobe, "flash");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(pdata->gpio_strobe, 0);
	if (ret < 0)
		goto err_gpio_flash;

	ret = gpio_request(pdata->gpio_torch, "torch");
	if (ret < 0)
		goto err_gpio_flash;

	ret = gpio_direction_output(pdata->gpio_torch, 0);
	if (ret < 0)
		goto err_gpio_torch;

	return 0;

err_gpio_torch:
	gpio_free(pdata->gpio_torch);
err_gpio_flash:
	gpio_free(pdata->gpio_strobe);
	return ret;
}

static int __devexit lm3554_gpio_uninit(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554 *flash = to_lm3554(sd);
	struct camera_flash_platform_data *pdata = flash->pdata;
	int ret;

	ret = gpio_direction_output(pdata->gpio_torch, 0);
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(pdata->gpio_strobe, 0);
	if (ret < 0)
		return ret;

	gpio_free(pdata->gpio_torch);

	gpio_free(pdata->gpio_strobe);

	return 0;
}

static int __devinit lm3554_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	int err;
	struct lm3554 *flash;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	flash->pdata = client->dev.platform_data;

	v4l2_i2c_subdev_init(&flash->sd, client, &lm3554_ops);
	flash->sd.internal_ops = &lm3554_internal_ops;
	flash->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->mode = ATOMISP_FLASH_MODE_OFF;

	err = media_entity_init(&flash->sd.entity, 0, NULL, 0);
	if (err) {
		dev_err(&client->dev, "error initialize a media entity.\n");
		goto fail1;
	}

	flash->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	mutex_init(&flash->i2c_mutex);

	setup_timer(&flash->flash_off_delay, lm3554_flash_off_delay,
		    (unsigned long)client);

	err = lm3554_gpio_init(client);
	if (err) {
		dev_err(&client->dev, "gpio request/direction_output fail");
		goto fail2;
	}

	return 0;
fail2:
	media_entity_cleanup(&flash->sd.entity);
fail1:
	v4l2_device_unregister_subdev(&flash->sd);
	kfree(flash);

	return err;
}

static int __devexit lm3554_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lm3554 *flash = to_lm3554(sd);
	int ret;

	media_entity_cleanup(&flash->sd.entity);
	v4l2_device_unregister_subdev(sd);

	del_timer_sync(&flash->flash_off_delay);

	ret = lm3554_gpio_uninit(client);
	if (ret < 0)
		goto fail;

	kfree(flash);

	return 0;
fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}

static const struct i2c_device_id lm3554_id[] = {
	{LEDFLASH_LM3554_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lm3554_id);

static struct i2c_driver lm3554_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LEDFLASH_LM3554_NAME,
	},
	.probe = lm3554_probe,
	.remove = __devexit_p(lm3554_remove),
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
MODULE_DESCRIPTION("LED flash driver for LM3554");
MODULE_LICENSE("GPL");
