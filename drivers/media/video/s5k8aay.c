/*
 * Support for s5k8aay Camera Sensor.
 * Based on the mt9m114.c driver.
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
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "s5k8aay.h"
#include "s5k8aay_modes.h"

#define to_s5k8aay_sensor(sd) container_of(sd, struct s5k8aay_device, sd)

static int
s5k8aay_read_reg(struct i2c_client *client, u16 data_length, u32 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u16) (reg >> 8);
	data[1] = (u16) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MISENSOR_8BIT)
			*val = data[0];
		else if (data_length == MISENSOR_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
			    (data[1] << 16) + (data[0] << 24);

		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
s5k8aay_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[6] = {0};
	u16 *wreg;
	int retry = 0;

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		dev_err(&client->dev, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + data_length;
	msg.buf = data;

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == MISENSOR_8BIT) {
		data[2] = (u8)(val);
	} else if (data_length == MISENSOR_16BIT) {
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu((u16)val);
	} else {
		/* MISENSOR_32BIT */
		u32 *wdata = (u32 *)&data[2];
		*wdata = be32_to_cpu(val);
	}

	num_msg = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * HACK: Need some delay here for Rev 2 sensors otherwise some
	 * registers do not seem to load correctly.
	 */
	mdelay(1);

	if (num_msg >= 0)
		return 0;

	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

/**
 * misensor_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16/32-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
misensor_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
		     u32 mask, u32 set)
{
	int err;
	u32 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	switch (data_length) {
	case MISENSOR_8BIT:
		if (mask & ~0xff)
			return -EINVAL;
		break;
	case MISENSOR_16BIT:
		if (mask & ~0xffff)
			return -EINVAL;
		break;
	case MISENSOR_32BIT:
		break;
	default:
		/* Wrong @data_length */
		return -EINVAL;
	}

	err = s5k8aay_read_reg(client, data_length, reg, &val);
	if (err) {
		dev_err(&client->dev, "misensor_rmw_reg error exit, read failed\n");
		return -EINVAL;
	}

	val &= ~mask;

	/*
	 * Perform the OR function if the @set exists.
	 * Shift @set value to target bit location. @set should set only
	 * bits included in @mask.
	 *
	 * REVISIT: This function expects @set to be non-shifted. Its shift
	 * value is then defined to be equal to mask's LSB position.
	 * How about to inform values in their right offset position and avoid
	 * this unneeded shift operation?
	 */
	set <<= ffs(mask) - 1;
	val |= set & mask;

	err = s5k8aay_write_reg(client, data_length, reg, val);
	if (err) {
		dev_err(&client->dev, "misensor_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


static int __s5k8aay_flush_reg_array(struct i2c_client *client,
				     struct s5k8aay_write_ctrl *ctrl)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

	if (ctrl->index == 0)
		return 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + ctrl->index;
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	msg.buf = (u8 *)&ctrl->buffer;

	ret = i2c_transfer(client->adapter, &msg, num_msg);
	if (ret != num_msg) {
		if (++retry <= I2C_RETRY_COUNT) {
			dev_dbg(&client->dev, "retrying... %d\n", retry);
			msleep(20);
			goto again;
		}
		dev_err(&client->dev, "%s: i2c transfer error\n", __func__);
		return -EIO;
	}

	ctrl->index = 0;

	/*
	 * REVISIT: Previously we had a delay after writing data to sensor.
	 * But it was removed as our tests have shown it is not necessary
	 * anymore.
	 */

	return 0;
}

static int __s5k8aay_buf_reg_array(struct i2c_client *client,
				   struct s5k8aay_write_ctrl *ctrl,
				   const struct misensor_reg *next)
{
	u16 *data16;
	u32 *data32;
	int err;

	/* Insufficient buffer? Let's flush and get more free space. */
	if (ctrl->index + next->length >= S5K8AAY_MAX_WRITE_BUF_SIZE) {
		err = __s5k8aay_flush_reg_array(client, ctrl);
		if (err)
			return err;
	}

	switch (next->length) {
	case MISENSOR_8BIT:
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case MISENSOR_16BIT:
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	case MISENSOR_32BIT:
		data32 = (u32 *)&ctrl->buffer.data[ctrl->index];
		*data32 = cpu_to_be32(next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg;

	ctrl->index += next->length;

	return 0;
}

static int
__s5k8aay_write_reg_is_consecutive(struct i2c_client *client,
				   struct s5k8aay_write_ctrl *ctrl,
				   const struct misensor_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

/*
 * s5k8aay_write_reg_array - Initializes a list of s5k8aay registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __s5k8aay_flush_reg_array, __s5k8aay_buf_reg_array() and
 * __s5k8aay_write_reg_is_consecutive() are internal functions to
 * s5k8aay_write_reg_array() and should be not used anywhere else.
 *
 */
static int s5k8aay_write_reg_array(struct i2c_client *client,
				const struct misensor_reg *reglist)
{
	const struct misensor_reg *next = reglist;
	struct s5k8aay_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->length != MISENSOR_TOK_TERM; next++) {
		switch (next->length & MISENSOR_TOK_MASK) {
		case MISENSOR_TOK_DELAY:
			err = __s5k8aay_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__s5k8aay_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __s5k8aay_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __s5k8aay_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	err = __s5k8aay_flush_reg_array(client, &ctrl);
	if (err)
		return err;

	return 0;
}

static int s5k8aay_write(struct i2c_client *client, u32 addr, u16 val)
{
	int ret;

	ret = s5k8aay_write_reg(client, MISENSOR_16BIT,
				AHB_MSB_ADDR_PTR, addr >> 16);
	if (ret < 0) {
		dev_err(&client->dev, "write page select failed\n");
		return ret;
	}
	ret = s5k8aay_write_reg(client, MISENSOR_16BIT, addr & 0xffff, val);
	if (ret < 0) {
		dev_err(&client->dev, "register write failed\n");
		return ret;
	}
	return 0;
}

static int s5k8aay_read(struct i2c_client *client, u32 addr, u16 *val)
{
	int ret;
	u32 v;

	ret = s5k8aay_write_reg(client, MISENSOR_16BIT,
				AHB_MSB_ADDR_PTR, addr >> 16);
	if (ret < 0) {
		dev_err(&client->dev, "read page select failed\n");
		return ret;
	}
	ret = s5k8aay_read_reg(client, MISENSOR_16BIT, addr & 0xffff, &v);
	if (ret < 0) {
		dev_err(&client->dev, "register read failed\n");
		return ret;
	}
	*val = v;
	return 0;
}

static int s5k8aay_check_error(struct s5k8aay_device *dev,
			       struct i2c_client *client)
{
	static struct {
		char *error;
		int address;
	} error_codes[] = {
		{ "REG_TC_IPRM_ErrorInfo",	0x70000166 },
		{ "REG_TC_GP_ErrorPrevConfig",	0x700001AE },
		{ "REG_TC_GP_ErrorCapConfig",	0x700001B4 },
		{ "REG_TC_IPRM_InitHwErr",	0x70000144 },
		{ "REG_TC_PZOOM_ErrorZoom",	0x700003A2 },
		{ "TnP_SvnVersion",		0x700027C0 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(error_codes); i++) {
		u16 v;
		int ret = s5k8aay_read(client, error_codes[i].address, &v);
		if (ret) {
			dev_err(&client->dev, "i2c error %i", ret);
			return ret;
		}
		dev_info(&client->dev, "%s: %i\n", error_codes[i].error, v);
	}

	return 0;
}

static int s5k8aay_reset(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
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
	usleep_range(1000, 4 * 1000);
	s5k8aay_check_error(dev, client);
}

static int s5k8aay_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/*  #REG_TC_GP_EnablePreview */
	ret = s5k8aay_write(client, 0x7000019e, 0x0000);
	if (ret < 0)
		return ret;

	/* #REG_TC_GP_EnablePreviewChanged */
	ret = s5k8aay_write(client, 0x700001a0, 0x0001);
	if (ret < 0)
		return ret;
	return 0;
}

static int s5k8aay_set_streaming(struct v4l2_subdev *sd)
{
	static struct misensor_reg const *s5k8aay_regs[] = {
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
		int ret = s5k8aay_write_reg_array(client, s5k8aay_regs[i]);
		if (ret) {
			dev_err(&client->dev,
				"register list %i write failed with %i\n",
				i, ret);
			return ret;
		}
	}
	s5k8aay_check_error(dev, client);

	return 0;
}

static int s5k8aay_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = s5k8aay_reset(sd);
	if (ret) {
		dev_err(&client->dev, "sensor reset failed\n");
		return ret;
	}
	ret = s5k8aay_set_streaming(sd);
	return ret;
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

	usleep_range(15, 4 * 15);

	/* Release reset */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* 100 us is needed between power up and first i2c transaction. */
	usleep_range(100, 4 * 100);

	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	return ret;
}

static int s5k8aay_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0) {
		return power_down(sd);
	} else {
		if (power_up(sd))
			return -EINVAL;

		return s5k8aay_init_common(sd);
	}
}

static void s5k8aay_try_res(u32 *w, u32 *h)
{
	int i;

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < ARRAY_SIZE(s5k8aay_res); i++) {
		if (s5k8aay_res[i].width >= *w &&
		    s5k8aay_res[i].height >= *h)
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == ARRAY_SIZE(s5k8aay_res))
		i--;

	*w = s5k8aay_res[i].width;
	*h = s5k8aay_res[i].height;
}

static struct s5k8aay_res_struct *s5k8aay_to_res(u32 w, u32 h)
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(s5k8aay_res); index++) {
		if ((s5k8aay_res[index].width == w) &&
		    (s5k8aay_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= ARRAY_SIZE(s5k8aay_res))
		return NULL;

	return &s5k8aay_res[index];
}

static int s5k8aay_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	s5k8aay_try_res(&fmt->width, &fmt->height);
	return 0;
}

static int s5k8aay_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	switch (res) {
	case S5K8AAY_RES_QCIF:
		hsize = S5K8AAY_RES_QCIF_SIZE_H;
		vsize = S5K8AAY_RES_QCIF_SIZE_V;
		break;
	case S5K8AAY_RES_QVGA:
		hsize = S5K8AAY_RES_QVGA_SIZE_H;
		vsize = S5K8AAY_RES_QVGA_SIZE_V;
		break;
	case S5K8AAY_RES_VGA:
		hsize = S5K8AAY_RES_VGA_SIZE_H;
		vsize = S5K8AAY_RES_VGA_SIZE_V;
		break;
	case S5K8AAY_RES_480P:
		hsize = S5K8AAY_RES_480P_SIZE_H;
		vsize = S5K8AAY_RES_480P_SIZE_V;
		break;
	case S5K8AAY_RES_720P:
		hsize = S5K8AAY_RES_720P_SIZE_H;
		vsize = S5K8AAY_RES_720P_SIZE_V;
		break;
	case S5K8AAY_RES_960P:
		hsize = S5K8AAY_RES_960P_SIZE_H;
		vsize = S5K8AAY_RES_960P_SIZE_V;
		break;
	default:
		WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
		return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;

	return 0;
}

static int s5k8aay_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	int width, height;
	int ret;

	ret = s5k8aay_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int s5k8aay_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct s5k8aay_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;

	s5k8aay_try_res(&width, &height);
	res_index = s5k8aay_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
	case S5K8AAY_RES_720P:
		break;
	default:
		dev_err(&client->dev,
			"set resolution: %d failed!\n", res_index->res);
		return -EINVAL;
	}

	if (s5k8aay_set_suspend(sd))
		return -EINVAL;

	if (dev->res != res_index->res) {
		int index;

		/* Switch to different size */
		if (width <= 640) {
			dev->nctx = 0x00; /* Set for context A */
		} else {
			/*
			 * Context B is used for resolutions larger than 640x480
			 * Using YUV for Context B.
			 */
			dev->nctx = 0x01; /* set for context B */
		}

		/*
		 * Marked current sensor res as being "used"
		 *
		 * REVISIT: We don't need to use an "used" field on each mode
		 * list entry to know which mode is selected. If this
		 * information is really necessary, how about to use a single
		 * variable on sensor dev struct?
		 */
		for (index = 0; index < ARRAY_SIZE(s5k8aay_res); index++) {
			if ((width == s5k8aay_res[index].width) &&
			    (height == s5k8aay_res[index].height)) {
				s5k8aay_res[index].used = 1;
				continue;
			}
			s5k8aay_res[index].used = 0;
		}
	}

	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;

	return 0;
}

/* TODO: Update to SOC functions, remove exposure and gain */
static int s5k8aay_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (S5K8AAY_FOCAL_LENGTH_NUM << 16) | S5K8AAY_FOCAL_LENGTH_DEM;
	return 0;
}

static int s5k8aay_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for s5k8aay*/
	*val = (S5K8AAY_F_NUMBER_DEFAULT_NUM << 16) | S5K8AAY_F_NUMBER_DEM;
	return 0;
}

static int s5k8aay_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (S5K8AAY_F_NUMBER_DEFAULT_NUM << 24) |
		(S5K8AAY_F_NUMBER_DEM << 16) |
		(S5K8AAY_F_NUMBER_DEFAULT_NUM << 8) | S5K8AAY_F_NUMBER_DEM;
	return 0;
}

/* Horizontal flip the image. */
static int s5k8aay_g_hflip(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int ret;
	u32 data;
	ret = s5k8aay_read_reg(c, MISENSOR_16BIT,
			(u32)MISENSOR_READ_MODE, &data);
	if (ret)
		return ret;
	*val = !!(data & MISENSOR_HFLIP_MASK);

	return 0;
}

static int s5k8aay_g_vflip(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int ret;
	u32 data;

	ret = s5k8aay_read_reg(c, MISENSOR_16BIT,
			(u32)MISENSOR_READ_MODE, &data);
	if (ret)
		return ret;
	*val = !!(data & MISENSOR_VFLIP_MASK);

	return 0;
}

static int s5k8aay_s_freq(struct v4l2_subdev *sd, s32  val)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	int ret;

	if (val != S5K8AAY_FLICKER_MODE_50HZ &&
			val != S5K8AAY_FLICKER_MODE_60HZ)
		return -EINVAL;

	ret = -EINVAL;

	if (ret == 0)
		dev->lightfreq = val;

	return ret;
}

static int s5k8aay_g_2a_status(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	unsigned int status_exp, status_wb;
	*val = 0;

	ret = s5k8aay_read_reg(client, MISENSOR_16BIT,
			MISENSOR_AE_TRACK_STATUS, &status_exp);
	if (ret)
		return ret;

	ret = s5k8aay_read_reg(client, MISENSOR_16BIT,
			MISENSOR_AWB_STATUS, &status_wb);
	if (ret)
		return ret;

	if (status_exp & MISENSOR_AE_READY)
		*val = V4L2_2A_STATUS_AE_READY;

	if (status_wb & MISENSOR_AWB_STEADY)
		*val |= V4L2_2A_STATUS_AWB_READY;

	return 0;
}

/* Horizontal flip the image. */
static int s5k8aay_t_hflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	int err;
	/* set for direct mode */
	err = s5k8aay_write_reg(c, MISENSOR_16BIT, 0x098E, 0xC850);
	if (value) {
		/* enable H flip ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x01, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x01, 0x01);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x01, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x01, 0x01);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_HFLIP_MASK, MISENSOR_FLIP_EN);

		dev->bpat = S5K8AAY_BPAT_GRGRBGBG;
	} else {
		/* disable H flip ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x01, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x01, 0x00);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x01, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x01, 0x00);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_HFLIP_MASK, MISENSOR_FLIP_DIS);

		dev->bpat = S5K8AAY_BPAT_BGBGGRGR;
	}

	err += s5k8aay_write_reg(c, MISENSOR_8BIT, 0x8404, 0x06);
	udelay(10);

	return !!err;
}

/* Vertically flip the image */
static int s5k8aay_t_vflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int err;
	/* set for direct mode */
	err = s5k8aay_write_reg(c, MISENSOR_16BIT, 0x098E, 0xC850);
	if (value >= 1) {
		/* enable H flip - ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x02, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x02, 0x01);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x02, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x02, 0x01);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_VFLIP_MASK, MISENSOR_FLIP_EN);
	} else {
		/* disable H flip - ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x02, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x02, 0x00);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x02, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x02, 0x00);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_VFLIP_MASK, MISENSOR_FLIP_DIS);
	}

	err += s5k8aay_write_reg(c, MISENSOR_8BIT, 0x8404, 0x06);
	udelay(10);

	return !!err;
}

static struct s5k8aay_control s5k8aay_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image v-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = s5k8aay_g_vflip,
		.tweak = s5k8aay_t_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image h-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = s5k8aay_g_hflip,
		.tweak = s5k8aay_t_hflip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = S5K8AAY_FOCAL_LENGTH_DEFAULT,
			.maximum = S5K8AAY_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = S5K8AAY_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = s5k8aay_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = S5K8AAY_F_NUMBER_DEFAULT,
			.maximum = S5K8AAY_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = S5K8AAY_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = s5k8aay_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = S5K8AAY_F_NUMBER_RANGE,
			.maximum =  S5K8AAY_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = S5K8AAY_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = s5k8aay_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_POWER_LINE_FREQUENCY,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Light frequency filter",
			.minimum = 1,
			.maximum =  2, /* 1: 50Hz, 2:60Hz */
			.step = 1,
			.default_value = 1,
			.flags = 0,
		},
		.tweak = s5k8aay_s_freq,
	},
	{
		.qc = {
			.id = V4L2_CID_2A_STATUS,
			.type = V4L2_CTRL_TYPE_BITMASK,
			.name = "AE and AWB status",
			.minimum = 0,
			.maximum = V4L2_2A_STATUS_AE_READY |
				V4L2_2A_STATUS_AWB_READY,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = s5k8aay_g_2a_status,
	},

};
#define N_CONTROLS (ARRAY_SIZE(s5k8aay_controls))

static struct s5k8aay_control *s5k8aay_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (s5k8aay_controls[i].qc.id == id)
			return &s5k8aay_controls[i];
	}
	return NULL;
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

	ret = s5k8aay_read(client, S5K8AAY_CHIP_ID, &id);
	if (ret)
		return ret;
	ret = s5k8aay_read(client, S5K8AAY_ROM_REVISION, &revision);
	if (ret)
		return ret;

	dev_info(&client->dev, "chip id 0x%4.4x, ROM revision 0x%4.4x\n",
		 id, revision);

	if (id != S5K8AAY_CHIP_ID_VAL) {
		dev_err(&client->dev, "failed to detect sensor\n");
		return -ENODEV;
	}

	s5k8aay_check_error(dev, client);
	return 0;
}

static int
s5k8aay_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct s5k8aay_device *dev = to_s5k8aay_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

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
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int s5k8aay_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct s5k8aay_control *ctrl = s5k8aay_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int s5k8aay_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k8aay_control *octrl = s5k8aay_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int s5k8aay_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k8aay_control *octrl = s5k8aay_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int s5k8aay_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	if (enable)
		ret = s5k8aay_set_streaming(sd);
	else
		ret = s5k8aay_set_suspend(sd);

	return ret;
}

static int
s5k8aay_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= ARRAY_SIZE(s5k8aay_res))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = s5k8aay_res[index].width;
	fsize->discrete.height = s5k8aay_res[index].height;

	/* FIXME: Wrong way to know used mode */
	fsize->reserved[0] = s5k8aay_res[index].used;

	return 0;
}

static int s5k8aay_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	int i;

	if (fival->index >= ARRAY_SIZE(s5k8aay_res))
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < ARRAY_SIZE(s5k8aay_res); i++) {
		if (s5k8aay_res[i].width >= fival->width &&
		    s5k8aay_res[i].height >= fival->height)
			break;
	}
	if (i == ARRAY_SIZE(s5k8aay_res))
		i--;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = s5k8aay_res[i].fps;

	return 0;
}

static int
s5k8aay_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_S5K8AAY, 0);
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
	if (fse->index >= ARRAY_SIZE(s5k8aay_res))
		return -EINVAL;

	fse->min_width = s5k8aay_res[fse->index].width;
	fse->min_height = s5k8aay_res[fse->index].height;
	fse->max_width = s5k8aay_res[fse->index].width;
	fse->max_height = s5k8aay_res[fse->index].height;

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

static int s5k8aay_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct s5k8aay_device *snr = to_s5k8aay_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < ARRAY_SIZE(s5k8aay_res); index++) {
		if (s5k8aay_res[index].res == snr->res)
			break;
	}

	if (index >= ARRAY_SIZE(s5k8aay_res))
		return -EINVAL;

	*frames = s5k8aay_res[index].skip_frames;

	return 0;
}
static const struct v4l2_subdev_video_ops s5k8aay_video_ops = {
	.try_mbus_fmt = s5k8aay_try_mbus_fmt,
	.s_mbus_fmt = s5k8aay_set_mbus_fmt,
	.g_mbus_fmt = s5k8aay_get_mbus_fmt,
	.s_stream = s5k8aay_s_stream,
	.enum_framesizes = s5k8aay_enum_framesizes,
	.enum_frameintervals = s5k8aay_enum_frameintervals,
};

static struct v4l2_subdev_sensor_ops s5k8aay_sensor_ops = {
	.g_skip_frames	= s5k8aay_g_skip_frames,
};

static const struct v4l2_subdev_core_ops s5k8aay_core_ops = {
	.g_chip_ident = s5k8aay_g_chip_ident,
	.queryctrl = s5k8aay_queryctrl,
	.g_ctrl = s5k8aay_g_ctrl,
	.s_ctrl = s5k8aay_s_ctrl,
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
	.sensor = &s5k8aay_sensor_ops,
};

static int s5k8aay_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k8aay_device *dev =
				container_of(sd, struct s5k8aay_device, sd);

	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

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

	v4l2_i2c_subdev_init(&dev->sd, client, &s5k8aay_ops);

	ret = s5k8aay_s_config(&dev->sd, client->irq,
			       client->dev.platform_data);
	if (ret) {
		kfree(dev);
		return ret;
	}

	/*TODO add format code here*/
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

