/*
 * Support for s5k8aay CMOS camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <linux/atomisp_platform.h>

#define S5K8AAY_TOK_16BIT	2
#define S5K8AAY_TOK_TERM	0xf0	/* terminating token for reg list */
#define S5K8AAY_TOK_DELAY	0xfe	/* delay token for reg list */

#define S5K8AAY_FORMAT		V4L2_MBUS_FMT_UYVY8_1X16

struct s5k8aay_reg {
	u8 tok;
	u16 reg;
	u16 val;
};

#include "s5k8aay_settings.h"

#define S5K8AAY_REG_CHIP_ID			0x00000040
#define S5K8AAY_REG_CHIP_ID_VAL			0x08AA
#define S5K8AAY_REG_ROM_REVISION		0x00000042 /* 0x00A0 / 0x00B0 */
#define S5K8AAY_REG_TC_IPRM_ERRORINFO		0x70000166
#define S5K8AAY_REG_TC_GP_ERRORPREVCONFIG	0x700001AE
#define S5K8AAY_REG_TC_GP_ERRORCAPCONFIG	0x700001B4
#define S5K8AAY_REG_TC_IPRM_INITHWERR		0x70000144
#define S5K8AAY_REG_TC_PZOOM_ERRORZOOM		0x700003A2
#define S5K8AAY_REG_TNP_SVNVERSION		0x700027C0
#define S5K8AAY_REG_TC_GP_ENABLEPREVIEW		0x7000019e
#define S5K8AAY_REG_TC_GP_ENABLEPREVIEWCHANGED	0x700001a0

#define S5K8AAY_R16_AHB_MSB_ADDR_PTR		0xfcfc

#define S5K8AAY_RES_WIDTH		1280
#define S5K8AAY_RES_HEIGHT		720

#define to_s5k8aay_sensor(_sd) container_of(_sd, struct s5k8aay_device, sd)

struct s5k8aay_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock;
};

static int
s5k8aay_simple_read16(struct i2c_client *client, int reg, u16 *val)
{
	unsigned char buffer[] = {
		reg >> 8, reg & 0xff
	};
	struct i2c_msg msg[] = { {
		.addr = client->addr,
		.len = ARRAY_SIZE(buffer),
		.flags = 0,
		.buf = buffer,
	}, {
		.addr = client->addr,
		.len = ARRAY_SIZE(buffer),
		.flags = I2C_M_RD,
		.buf = buffer,
	} };
	int err;

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (err < 0) {
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = buffer[1] + (buffer[0] << 8);
	return 0;
}

static int
s5k8aay_simple_write16(struct i2c_client *client, int reg, int val)
{
	unsigned char buffer[] = {
		reg >> 8, reg & 0xff,
		val >> 8, val & 0xff
	};
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = ARRAY_SIZE(buffer),
		.flags = 0,
		.buf = buffer,
	};
	int err;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, err);
	return 0;
}

static int s5k8aay_write_array(struct i2c_client *client,
			       const struct s5k8aay_reg *reglist)
{
	const struct s5k8aay_reg *next = reglist;
	int ret;

	for (; next->tok != S5K8AAY_TOK_TERM; next++) {
		if (next->tok == S5K8AAY_TOK_DELAY) {
			usleep_range(next->val * 1000, next->val * 1000);
			continue;
		}
		ret = s5k8aay_simple_write16(client, next->reg, next->val);
		if (ret) {
			dev_err(&client->dev, "register write failed\n");
			return ret;
		}
	}

	return 0;
}

static int s5k8aay_write(struct i2c_client *client, u32 addr, u16 val)
{
	int ret;

	ret = s5k8aay_simple_write16(client, S5K8AAY_R16_AHB_MSB_ADDR_PTR,
				     addr >> 16);
	if (ret < 0)
		return ret;

	return s5k8aay_simple_write16(client, addr & 0xffff, val);
}

static int s5k8aay_read(struct i2c_client *client, u32 addr, u16 *val)
{
	int ret;

	ret = s5k8aay_simple_write16(client, S5K8AAY_R16_AHB_MSB_ADDR_PTR,
				     addr >> 16);
	if (ret < 0)
		return ret;

	return s5k8aay_simple_read16(client, addr & 0xffff, val);
}

static int s5k8aay_check_error(struct s5k8aay_device *dev,
			       struct i2c_client *client)
{
	static struct {
		char *error;
		int address;
	} error_codes[] = {
		{ "ErrorInfo",		S5K8AAY_REG_TC_IPRM_ERRORINFO },
		{ "ErrorPrevConfig",	S5K8AAY_REG_TC_GP_ERRORPREVCONFIG },
		{ "ErrorCapConfig",	S5K8AAY_REG_TC_GP_ERRORCAPCONFIG },
		{ "InitHwErr",		S5K8AAY_REG_TC_IPRM_INITHWERR },
		{ "ErrorZoom",		S5K8AAY_REG_TC_PZOOM_ERRORZOOM },
		{ "TnP_SvnVersion",	S5K8AAY_REG_TNP_SVNVERSION },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(error_codes); i++) {
		u16 v;
		int ret = s5k8aay_read(client, error_codes[i].address, &v);
		if (ret)
			return ret;
		dev_info(&client->dev, "%s: %i\n", error_codes[i].error, v);
	}

	return 0;
}

static int s5k8aay_reset(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/* Reset */
	ret = s5k8aay_write(client, 0xd0000010, 0x0001);
	if (ret < 0)
		return ret;

	/* Clear host interrupt */
	ret = s5k8aay_write(client, 0xd0001030, 0x0000);
	if (ret < 0)
		return ret;

	/* ARM go */
	ret = s5k8aay_write(client, 0xd0000014, 0x0001);
	if (ret < 0)
		return ret;

	/* Allow startup code to run */
	usleep_range(1000, 1000);
}

static int s5k8aay_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = s5k8aay_write(client, S5K8AAY_REG_TC_GP_ENABLEPREVIEW, 0x0000);
	if (ret < 0)
		return ret;

	return s5k8aay_write(client, S5K8AAY_REG_TC_GP_ENABLEPREVIEWCHANGED,
			     0x0001);
}

static int s5k8aay_set_streaming(struct v4l2_subdev *sd)
{
	static struct s5k8aay_reg const *s5k8aay_regs[] = {
		s5k8aay_regs_1,
		s5k8aay_regs_2,
		s5k8aay_regs_3,
		s5k8aay_regs_4,
		s5k8aay_regs_5,
		s5k8aay_regs_6,
		s5k8aay_regs_7,
		s5k8aay_regs_8,
		s5k8aay_regs_9,
		s5k8aay_regs_10,
		s5k8aay_regs_11,
		s5k8aay_regs_12,
		s5k8aay_regs_13,
		s5k8aay_regs_14,
		s5k8aay_regs_15,
		s5k8aay_regs_16,
		s5k8aay_regs_17,
		s5k8aay_regs_18,
		s5k8aay_regs_19,
		s5k8aay_regs_20,
		s5k8aay_regs_21,
	};
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	int i;

	for (i = 0; i < ARRAY_SIZE(s5k8aay_regs); i++) {
		int ret = s5k8aay_write_array(client, s5k8aay_regs[i]);
		if (ret)
			return ret;
	}
	s5k8aay_check_error(dev, client);

	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	usleep_range(15, 15);

	/* Release reset */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		goto fail_gpio;

	/* 100 us is needed between power up and first i2c transaction. */
	usleep_range(100, 100);

	return 0;

fail_gpio:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_clk:
	dev->platform_data->power_ctrl(sd, 0);
fail_power:
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk off failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio off failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "power off failed\n");

	return ret;
}

static int s5k8aay_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0) {
		return power_down(sd);
	} else {
		int ret = power_up(sd);
		if (ret)
			return ret;

		ret = s5k8aay_reset(sd);
		if (ret)
			return ret;
	}
}

static int s5k8aay_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = S5K8AAY_RES_WIDTH;
	fmt->height = S5K8AAY_RES_HEIGHT;
	fmt->code = S5K8AAY_FORMAT;
	return 0;
}

static int s5k8aay_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = S5K8AAY_RES_WIDTH;
	fmt->height = S5K8AAY_RES_HEIGHT;
	fmt->code = S5K8AAY_FORMAT;
	return 0;
}

static int s5k8aay_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = S5K8AAY_RES_WIDTH;
	fmt->height = S5K8AAY_RES_HEIGHT;
	fmt->code = S5K8AAY_FORMAT;
	return 0;
}

static int s5k8aay_detect(struct s5k8aay_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 ret;
	u16 id = -1, revision = -1;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	ret = s5k8aay_read(client, S5K8AAY_REG_CHIP_ID, &id);
	if (ret)
		return ret;

	ret = s5k8aay_read(client, S5K8AAY_REG_ROM_REVISION, &revision);
	if (ret)
		return ret;

	dev_info(&client->dev, "chip id 0x%4.4x, ROM revision 0x%4.4x\n",
		 id, revision);

	if (id != S5K8AAY_REG_CHIP_ID_VAL) {
		dev_err(&client->dev, "failed to detect sensor\n");
		return -ENODEV;
	}
	return 0;
}

static int
s5k8aay_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev->platform_data = platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			dev_err(&client->dev, "s5k8aay platform init err\n");
			return ret;
		}
	}

	ret = s5k8aay_s_power(sd, 1);
	if (ret) {
		dev_err(&client->dev, "s5k8aay power-up err %i\n", ret);
		return ret;
	}

	/* config & detect sensor */
	ret = s5k8aay_detect(dev, client);
	if (ret) {
		dev_err(&client->dev, "s5k8aay_detect err s_config.\n");
		goto fail_detect;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = s5k8aay_set_suspend(sd);
	if (ret) {
		dev_err(&client->dev, "s5k8aay suspend err");
		return ret;
	}

	ret = s5k8aay_s_power(sd, 0);
	if (ret) {
		dev_err(&client->dev, "s5k8aay power down err");
		return ret;
	}

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	s5k8aay_s_power(sd, 0);
	dev_err(&client->dev, "sensor detection failed\n");
	return ret;
}

static int s5k8aay_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		return s5k8aay_set_streaming(sd);
	else
		return s5k8aay_set_suspend(sd);
}

static int
s5k8aay_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = S5K8AAY_RES_WIDTH;
	fsize->discrete.height = S5K8AAY_RES_HEIGHT;
	return 0;
}

static int s5k8aay_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_UYVY8_1X16;

	return 0;
}

static int s5k8aay_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	fse->min_width = S5K8AAY_RES_WIDTH;
	fse->min_height = S5K8AAY_RES_HEIGHT;
	fse->max_width = S5K8AAY_RES_WIDTH;
	fse->max_height = S5K8AAY_RES_HEIGHT;

	return 0;
}

static struct v4l2_mbus_framefmt *
__s5k8aay_get_pad_format(struct s5k8aay_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
s5k8aay_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct s5k8aay_device *snr = to_s5k8aay_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__s5k8aay_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
s5k8aay_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct s5k8aay_device *snr = to_s5k8aay_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__s5k8aay_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static const struct v4l2_subdev_video_ops s5k8aay_video_ops = {
	.try_mbus_fmt = s5k8aay_try_mbus_fmt,
	.s_mbus_fmt = s5k8aay_set_mbus_fmt,
	.g_mbus_fmt = s5k8aay_get_mbus_fmt,
	.s_stream = s5k8aay_s_stream,
	.enum_framesizes = s5k8aay_enum_framesizes,
};

static const struct v4l2_subdev_core_ops s5k8aay_core_ops = {
	.s_power = s5k8aay_s_power,
};

static const struct v4l2_subdev_pad_ops s5k8aay_pad_ops = {
	.enum_mbus_code = s5k8aay_enum_mbus_code,
	.enum_frame_size = s5k8aay_enum_frame_size,
	.get_fmt = s5k8aay_get_pad_format,
	.set_fmt = s5k8aay_set_pad_format,
};

static const struct v4l2_subdev_ops s5k8aay_ops = {
	.core = &s5k8aay_core_ops,
	.video = &s5k8aay_video_ops,
	.pad = &s5k8aay_pad_ops,
};

static int s5k8aay_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct s5k8aay_device *dev;
	int ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	v4l2_i2c_subdev_init(&dev->sd, client, &s5k8aay_ops);

	ret = s5k8aay_s_config(&dev->sd, client->irq,
			       client->dev.platform_data);
	if (ret) {
		kfree(dev);
		return ret;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_UYVY8_1X16;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		kfree(dev);
		return ret;
	}

	return 0;
}

static int s5k8aay_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	media_entity_cleanup(&dev->sd.entity);
	mutex_destroy(&dev->input_lock);
	kfree(dev);
	return 0;
}

static const struct i2c_device_id s5k8aay_id[] = {
	{ "s5k8aay", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, s5k8aay_id);

static struct i2c_driver s5k8aay_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "s5k8aay"
	},
	.probe = s5k8aay_probe,
	.remove = s5k8aay_remove,
	.id_table = s5k8aay_id,
};

static int init_s5k8aay(void)
{
	return i2c_add_driver(&s5k8aay_driver);
}

static void exit_s5k8aay(void)
{
	i2c_del_driver(&s5k8aay_driver);
}

module_init(init_s5k8aay);
module_exit(exit_s5k8aay);

MODULE_AUTHOR("Tuukka Toivonen <tuukka.toivonen@intel.com>");
MODULE_LICENSE("GPL");
