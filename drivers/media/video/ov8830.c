/*
 * Support for OmniVision ov8830 1080p HD camera sensor.
 *
 * Copyright (c) 2011 Intel Corporation. All Rights Reserved.
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "ov8830.h"

#define OV8830_BIN_FACTOR_MAX	2

#define to_ov8830_sensor(sd) container_of(sd, struct ov8830_device, sd)

#define HOME_POS 255

/* divides a by b using half up rounding and div/0 prevention
 * (result is 0 if b == 0) */
#define divsave_rounded(a, b)	(((b) != 0) ? (((a)+((b)>>1))/(b)) : (-1))

typedef unsigned int sensor_register;
struct sensor_mode_data {
	sensor_register coarse_integration_time_min;
	sensor_register coarse_integration_time_max_margin;
	sensor_register fine_integration_time_min;
	sensor_register fine_integration_time_max_margin;
	sensor_register fine_integration_time_def;
	sensor_register frame_length_lines;
	sensor_register line_length_pck;
	sensor_register read_mode;
	int vt_pix_clk_freq_mhz;
};

/*
 * TODO: use debug parameter to actually define when debug messages should
 * be printed.
 */
static int debug;
static u16 real_model_id;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug messages");

static struct ov8830_resolution ov8830_res_preview[] = {
	{
		 .desc = "OV8830_PREVIEW_848x616",
		 .width = 848,
		 .height = 616,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 3608,
		 .lines_per_frame = 1773,
		 .regs = ov8830_PREVIEW_848x616_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		 .desc = "OV8830_WidePreview"	,
		 .width = 1280,
		 .height = 720,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 3608,
		 .lines_per_frame = 2586,
		 .regs = ov8830_PREVIEW_WIDE_PREVIEW_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		 .desc = "OV8830_PREVIEW1600x1200",
		 .width = 1632,
		 .height = 1224,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 3608,
		 .lines_per_frame = 1773,
		 .regs = ov8830_PREVIEW_1632x1224_30fps	,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(ov8830_res_preview))

static struct ov8830_resolution ov8830_res_still[] = {
	{
		 .desc = "STILL_VGA_15fps",
		 .width = 656,
		 .height = 496,
		 .fps = 15,
		 .used = 0,
		 .pixels_per_line = 4696,
		 .lines_per_frame = 2724,
		 .regs = ov8830_VGA_STILL_15fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
	},
	{
		 .desc = "STILL_1080P_15fps",
		 .width = 1936,
		 .height = 1104,
		 .fps = 15,
		 .used = 0,
		 .pixels_per_line = 4696,
		 .lines_per_frame = 2724,
		 .regs = ov8830_1080P_STILL_15fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
	},
	{
		 .desc = "STILL_1M_15fps",
		 .width = 1040,
		 .height = 784,
		 .fps = 15,
		 .used = 0,
		 .pixels_per_line = 4696,
		 .lines_per_frame = 2724,
		 .regs = ov8830_1M_STILL_15fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
	},
	{
		 .desc = "STILL_2M_15fps",
		 .width = 1640,
		 .height = 1232,
		 .fps = 15,
		 .used = 0,
		 .pixels_per_line = 4696,
		 .lines_per_frame = 2724,
		 .regs = ov8830_2M_STILL_15fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
	},
	{
		.desc = "STILL_3M_15fps",
		.width = 2064,
		.height = 1552,
		.fps = 15,
		.used = 0,
		.pixels_per_line = 4696,
		.lines_per_frame = 2724,
		.regs = ov8830_3M_STILL_15fps,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
	},
	{
		.desc = "STILL_5M_15fps",
		.width = 2576,
		.height = 1936,
		.fps = 15,
		.used = 0,
		.pixels_per_line = 4696,
		.lines_per_frame = 2724,
		.regs = ov8830_5M_STILL_15fps,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
	},
	{
		 .desc = "STILL_6M_15fps",
		 .width = 3280,
		 .height = 1852,
		 .fps = 15,
		 .used = 0,
		 .pixels_per_line = 4696,
		 .lines_per_frame = 2724,
		 .regs = ov8830_6M_STILL_15fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
	},
	{
		.desc = "STILL_8M_15fps",
		.width = 3280,
		.height = 2464,
		.fps = 15,
		.used = 0,
		.pixels_per_line = 4464,
		.lines_per_frame = 2867,
		.regs = ov8830_8M_STILL_15fps,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
	},
};

#define N_RES_STILL (ARRAY_SIZE(ov8830_res_still))

static struct ov8830_resolution ov8830_res_video[] = {
	{
		 .desc = "QCIF_strong_dvs_30fps",
		 .width = 216,
		 .height = 176,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 4128,
		 .lines_per_frame = 1550,
		 .regs = ov8830_QCIF_strong_dvs_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		 .desc = "QVGA_strong_dvs_30fps",
		 .width = 408,
		 .height = 308,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 4128,
		 .lines_per_frame = 1550,
		 .regs = ov8830_QVGA_strong_dvs_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		 .desc = "VGA_strong_dvs_30fps",
		 .width = 820,
		 .height = 616,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 4128,
		 .lines_per_frame = 1550,
		 .regs = ov8830_VGA_strong_dvs_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		.desc = "480p_strong_dvs_30fps",
		.width = 936,
		.height = 602,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 4128,
		.lines_per_frame = 1550,
		.regs = ov8830_480p_strong_dvs_30fps,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 0,
	},
	{
		 .desc = "720p_strong_dvs_30fps",
		 .width = 1568,
		 .height = 880,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 4128,
		 .lines_per_frame = 1550,
		 .regs = ov8830_720p_strong_dvs_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
	},
	{
		.desc = "MODE1920x1080_DVS_OFF",
		.width = 1936,
		.height = 1104,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 3200,
		.lines_per_frame = 2000,
		.regs = ov8830_1080p_30fps_dvs_off,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 0,
	},
	{
		 .desc = "MODE1920x1080",
		 .width = 2336,
		 .height = 1320,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 3200,
		 .lines_per_frame = 2000,
		 .regs = ov8830_1080p_strong_dvs_30fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 0,
	},
};

#define N_RES_VIDEO (ARRAY_SIZE(ov8830_res_video))

static struct ov8830_resolution *ov8830_res = ov8830_res_preview;
static int N_RES = N_RES_PREVIEW;

static int
ov8830_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[OV8830_SHORT_MAX];
	int err, i;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > OV8830_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == OV8830_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int ov8830_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == num_msg)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_err(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

static int
ov8830_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != OV8830_8BIT && data_length != OV8830_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == OV8830_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV8830_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu(val);
	}

	ret = ov8830_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/**
 * ov8830_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8830_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
			   u16 mask, u16 set)
{
	int err;
	u16 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	if (data_length == OV8830_8BIT && mask & ~0xff)
		return -EINVAL;

	err = ov8830_read_reg(client, data_length, reg, &val);
	if (err) {
		v4l2_err(client, "ov8830_rmw_reg error exit, read failed\n");
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

	err = ov8830_write_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "ov8830_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


/*
 * ov8830_write_reg_array - Initializes a list of MT9M114 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ov8830_flush_reg_array, __ov8830_buf_reg_array() and
 * __ov8830_write_reg_is_consecutive() are internal functions to
 * ov8830_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ov8830_flush_reg_array(struct i2c_client *client,
				     struct ov8830_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov8830_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ov8830_buf_reg_array(struct i2c_client *client,
				   struct ov8830_write_ctrl *ctrl,
				   const struct ov8830_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case OV8830_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case OV8830_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= OV8830_MAX_WRITE_BUF_SIZE)
		__ov8830_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ov8830_write_reg_is_consecutive(struct i2c_client *client,
				   struct ov8830_write_ctrl *ctrl,
				   const struct ov8830_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ov8830_write_reg_array(struct i2c_client *client,
				   const struct ov8830_reg *reglist)
{
	const struct ov8830_reg *next = reglist;
	struct ov8830_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != OV8830_TOK_TERM; next++) {
		switch (next->type & OV8830_TOK_MASK) {
		case OV8830_TOK_DELAY:
			err = __ov8830_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		case OV8830_RMW:
			err = __ov8830_flush_reg_array(client, &ctrl);
			err |= ov8830_rmw_reg(client,
					       next->type & ~OV8830_RMW,
					       next->reg.sreg, next->val,
					       next->val2);
			if (err) {
				v4l2_err(client, "%s: rwm error, "
						"aborted\n", __func__);
				return err;
			}
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ov8830_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ov8830_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ov8830_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ov8830_flush_reg_array(client, &ctrl);
}

static int drv201_write8(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct drv201_device *dev = to_drv201_device(sd);
	struct i2c_msg msg;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DRV201_I2C_ADDR;
	msg.len = 2;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int drv201_write16(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct drv201_device *dev = to_drv201_device(sd);
	struct i2c_msg msg;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DRV201_I2C_ADDR;
	msg.len = 3;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val >> 8;
	msg.buf[2] = val & 0xFF;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int drv201_read8(struct v4l2_subdev *sd, int reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct drv201_device *dev = to_drv201_device(sd);
	struct i2c_msg msg[2];
	int r;

	memset(msg, 0 , sizeof(msg));
	msg[0].addr = DRV201_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = dev->buffer;
	msg[0].buf[0] = reg;

	msg[1].addr = DRV201_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = dev->buffer;

	r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (r != ARRAY_SIZE(msg))
		return -EIO;

	return dev->buffer[0];
}

static int drv201_init(struct v4l2_subdev *sd)
{
	struct drv201_device *dev = to_drv201_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev->platform_data = camera_get_af_platform_data();
	if (!dev->platform_data) {
		v4l2_err(client, "failed to get platform data\n");
		return -ENXIO;
	}
	return 0;
}

static int drv201_power_up(struct v4l2_subdev *sd)
{
	/* Transition time required from shutdown to standby state */
	const int WAKEUP_DELAY_US = 100;
	const int DEFAULT_CONTROL_VAL = 0x02;

	struct drv201_device *dev = to_drv201_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int r;

	/* Enable power */
	r = dev->platform_data->power_ctrl(sd, 1);
	if (r)
		return r;

	udelay(1);		/* Wait for VBAT to stabilize */

	/* jiggle SCL pin to wake up device */
	drv201_write8(sd, DRV201_CONTROL, 1);

	usleep_range(WAKEUP_DELAY_US, WAKEUP_DELAY_US * 10);

	/* Reset device */
	r = drv201_write8(sd, DRV201_CONTROL, 1);
	if (r < 0)
		goto fail_powerdown;

	/* Detect device */
	r = drv201_read8(sd, DRV201_CONTROL);
	if (r < 0)
		goto fail_powerdown;
	if (r != DEFAULT_CONTROL_VAL) {
		r = -ENXIO;
		goto fail_powerdown;
	}

	dev->focus = DRV201_MAX_FOCUS_POS;
	dev->initialized = true;

	v4l2_info(client, "detected drv201\n");
	return 0;

fail_powerdown:
	dev->platform_data->power_ctrl(sd, 0);
	return r;
}

static int drv201_power_down(struct v4l2_subdev *sd)
{
	struct drv201_device *dev = to_drv201_device(sd);

	return dev->platform_data->power_ctrl(sd, 0);
}

static int drv201_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct drv201_device *dev = to_drv201_device(sd);
	int r;

	if (!dev->initialized)
		return -ENODEV;

	value = clamp(value, 0, DRV201_MAX_FOCUS_POS);
	r = drv201_write16(sd, DRV201_VCM_CURRENT,
			   DRV201_MAX_FOCUS_POS - value);
	if (r < 0)
		return r;

	getnstimeofday(&dev->focus_time);
	dev->focus = value;
	return 0;
}

static int drv201_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct drv201_device *dev = to_drv201_device(sd);
	*value = dev->focus;
	return 0;
}

static int drv201_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct drv201_device *dev = to_drv201_device(sd);
	return drv201_t_focus_abs(sd, dev->focus + value);
}

static int drv201_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	static const struct timespec move_time = {
		/* The time required for focus motor to move the lens */
		.tv_sec = 0,
		.tv_nsec = 60000000
	};
	struct drv201_device *dev = to_drv201_device(sd);
	struct timespec current_time, finish_time, delta_time;

	getnstimeofday(&current_time);
	finish_time = timespec_add(dev->focus_time, move_time);
	delta_time = timespec_sub(current_time, finish_time);
	if (delta_time.tv_sec >= 0 && delta_time.tv_nsec >= 0) {
		/* VCM motor is not moving */
		*value = ATOMISP_FOCUS_HP_COMPLETE |
			 ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
	} else {
		/* VCM motor is still moving */
		*value = ATOMISP_FOCUS_STATUS_MOVING |
			 ATOMISP_FOCUS_HP_IN_PROGRESS;
	}
	return 0;
}

/* Start group hold for the following register writes */
static int ov8830_grouphold_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const int group = 0;

	return ov8830_write_reg(client, OV8830_8BIT,
				OV8830_GROUP_ACCESS,
				group | OV8830_GROUP_ACCESS_HOLD_START);
}

/* End group hold and quick launch it */
static int ov8830_grouphold_launch(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const int group = 0;
	int ret;

	/* End group */
	ret = ov8830_write_reg(client, OV8830_8BIT,
			       OV8830_GROUP_ACCESS,
			       group | OV8830_GROUP_ACCESS_HOLD_END);
	if (ret)
		return ret;

	/* Delay launch group (during next vertical blanking) */
	return ov8830_write_reg(client, OV8830_8BIT,
				OV8830_GROUP_ACCESS,
				group | OV8830_GROUP_ACCESS_DELAY_LAUNCH);
}

/*
 * Read EEPROM data from the le24l042cs chip and store
 * it into a kmalloced buffer. On error return NULL.
 * The caller must kfree the buffer when no more needed.
 * @size: set to the size of the returned EEPROM data.
 */
static void *le24l042cs_read(struct i2c_client *client, int *size)
{
	static const unsigned int LE24L042CS_I2C_ADDR = 0xA0 >> 1;
	static const unsigned int LE24L042CS_EEPROM_SIZE = 512;
	static const unsigned int MAX_READ_SIZE = OV8830_MAX_WRITE_BUF_SIZE;
	struct i2c_msg msg[2];
	int addr;
	char *buffer;

	buffer = kmalloc(LE24L042CS_EEPROM_SIZE, GFP_KERNEL);
	if (!buffer)
		return NULL;

	memset(msg, 0, sizeof(msg));
	for (addr = 0; addr < LE24L042CS_EEPROM_SIZE; addr += MAX_READ_SIZE) {
		unsigned int i2c_addr = LE24L042CS_I2C_ADDR;
		unsigned char addr_buf;
		int r;

		i2c_addr |= (addr >> 8) & 1;
		addr_buf = addr & 0xFF;

		msg[0].addr = i2c_addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &addr_buf;

		msg[1].addr = i2c_addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = min(MAX_READ_SIZE, LE24L042CS_EEPROM_SIZE - addr);
		msg[1].buf = &buffer[addr];

		r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (r != ARRAY_SIZE(msg)) {
			kfree(buffer);
			dev_err(&client->dev, "read failed at 0x%03x\n", addr);
			return NULL;
		}
	}

	if (size)
		*size = LE24L042CS_EEPROM_SIZE;
	return buffer;
}

static int ov8830_g_priv_int_data(struct v4l2_subdev *sd,
				  struct v4l2_private_int_data *priv)
{
	int size, r = 0;
	void *b = le24l042cs_read(v4l2_get_subdevdata(sd), &size);

	if (!b)
		return -EIO;

	if (copy_to_user(priv->data, b, min(priv->size, size)))
		r = -EFAULT;

	priv->size = size;
	kfree(b);

	return r;
}

static int __ov8830_set_exposure(struct v4l2_subdev *sd, int exposure, int gain)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int exp_val, ret;

	/* OV sensor driver has a limitation that the exposure
	 * should not exceed beyond VTS-14
	 */
	 if (exposure > dev->lines_per_frame - 14)
		exposure = dev->lines_per_frame - 14;

	ret = ov8830_grouphold_start(sd);
	if (ret)
		goto out;

	/* set exposure time */
	exp_val = exposure << 4;
	ret = ov8830_write_reg(client, OV8830_8BIT,
			       OV8830_LONG_EXPO+2, exp_val & 0xFF);
	if (ret)
		goto out;

	ret = ov8830_write_reg(client, OV8830_8BIT,
			       OV8830_LONG_EXPO+1, (exp_val >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = ov8830_write_reg(client, OV8830_8BIT,
			       OV8830_LONG_EXPO, (exp_val >> 16) & 0x0F);
	if (ret)
		goto out;

	/* set global gain */
	ret = ov8830_write_reg(client, OV8830_8BIT,
			       OV8830_AGC_ADJ, gain);
	if (ret)
		goto out;

	ret = ov8830_grouphold_launch(sd);
	if (ret)
		goto out;

	dev->gain     = gain;
	dev->exposure = exposure;

out:
	return ret;
}

static int ov8830_set_exposure(struct v4l2_subdev *sd, int exposure, int gain)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ov8830_set_exposure(sd, exposure, gain);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov8830_s_exposure(struct v4l2_subdev *sd,
			      struct atomisp_exposure *exposure)
{
	int exp = exposure->integration_time[0];
	int gain = exposure->gain[0];

	return ov8830_set_exposure(sd, exp, gain);
}

static long ov8830_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov8830_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ov8830_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ov8830_init_registers(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = ov8830_write_reg_array(client, ov8830_SwReset);
	ret |= ov8830_write_reg_array(client, ov8830_PLL192MHz);
	ret |= ov8830_write_reg_array(client, ov8830_BasicSettings);
	ret |= ov8830_write_reg_array(client, ov8830_MIPI_Settings_684Mbps);
	ret |= ov8830_write_reg_array(client, ov8830_PREVIEW_848x616_30fps);

	return ret;
}

static int __ov8830_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;

	/* set inital registers */
	ret = ov8830_init_registers(sd);

	/* restore settings */
	ov8830_res = ov8830_res_preview;
	N_RES = N_RES_PREVIEW;

	return ret;
}


static int ov8830_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);
	ret = __ov8830_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static void ov8830_uninit(struct v4l2_subdev *sd)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	dev->exposure = 0;
	dev->gain     = 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret;

	/* Enable power */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* Release reset */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* Enable clock */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* Minumum delay is 8192 clock cycles before first i2c transaction,
	 * which is 1.37 ms at the lowest allowed clock rate 6 MHz */
	msleep(2);
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
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

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

	return ret;
}

static int __ov8830_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret, r;

	if (on == 0) {
		ov8830_uninit(sd);
		ret = power_down(sd);
		r = drv201_power_down(sd);
		if (ret == 0)
			ret = r;
		dev->power = 0;
	} else {
		ret = power_up(sd);
		if (ret)
			return ret;
		drv201_power_up(sd);

		dev->power = 1;
		/* init motor initial position */
		ret = __ov8830_init(sd, 0);
	}

	return ret;
}

static int ov8830_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	mutex_lock(&dev->input_lock);
	ret = __ov8830_s_power(sd, on);
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int ov8830_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV8830, 0);

	return 0;
}

/* Return value of the specified register, first try getting it from
 * the register list and if not found, get from the sensor via i2c.
 */
static int ov8830_get_register(struct v4l2_subdev *sd, int reg,
			       const struct ov8830_reg *reglist)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct ov8830_reg *next;
	u16 val;

	/* Try if the values is in the register list */
	for (next = reglist; next->type != OV8830_TOK_TERM; next++) {
		if (next->type != OV8830_8BIT) {
			v4l2_err(sd, "only 8-bit registers supported\n");
			return -ENXIO;
		}
		if (next->reg.sreg == reg)
			return next->val;
	}

	/* If not, read from sensor */
	if (ov8830_read_reg(client, OV8830_8BIT, reg, &val)) {
		v4l2_err(sd, "failed to read register 0x%04X\n", reg);
		return -EIO;
	}

	return val;
}

static int ov8830_get_intg_factor(struct v4l2_subdev *sd,
				  struct camera_mipi_info *info,
				  const struct ov8830_reg *reglist)
{
	const int ext_clk = 19200000; /* MHz */
	struct sensor_mode_data *m = (struct sensor_mode_data *)&info->data;
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int pll2_prediv;
	int pll2_multiplier;
	int pll2_divs;
	int pll2_seld5;
	int t1, t2, t3;
	int sclk;

	memset(&info->data, 0, sizeof(info->data));

	pll2_prediv     = ov8830_get_register(sd, OV8830_PLL_PLL10, reglist);
	pll2_multiplier = ov8830_get_register(sd, OV8830_PLL_PLL11, reglist);
	pll2_divs       = ov8830_get_register(sd, OV8830_PLL_PLL12, reglist);
	pll2_seld5      = ov8830_get_register(sd, OV8830_PLL_PLL13, reglist);

	if (pll2_prediv < 0 || pll2_multiplier < 0 ||
	    pll2_divs < 0 || pll2_seld5 < 0)
		return -EIO;

	pll2_prediv &= 0x07;
	pll2_multiplier &= 0x3F;
	pll2_divs = (pll2_divs & 0x0F) + 1;
	pll2_seld5 &= 0x03;

	if (pll2_prediv <= 0)
		return -EIO;

	t1 = ext_clk / pll2_prediv;
	t2 = t1 * pll2_multiplier;
	t3 = t2 / pll2_divs;
	sclk = t3;
	if (pll2_seld5 == 0)
		sclk = t3;
	else if (pll2_seld5 == 3)
		sclk = t3 * 2 / 5;
	else
		sclk = t3 / pll2_seld5;
	m->vt_pix_clk_freq_mhz = sclk;

	/* HTS and VTS */
	m->frame_length_lines = ov8830_res[dev->fmt_idx].lines_per_frame;
	m->line_length_pck = ov8830_res[dev->fmt_idx].pixels_per_line;

	m->coarse_integration_time_min = 0;
	m->coarse_integration_time_max_margin = OV8830_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time. */
	m->fine_integration_time_min = 0;
	m->fine_integration_time_max_margin = 0;

	/*
	 * read_mode inicate whether binning is used for calculating
	 * the correct exposure value from the user side. So adapt the
	 * read mode values accordingly.
	 */
	m->read_mode = ov8830_res[dev->fmt_idx].bin_factor_x ?
		     OV8830_READ_MODE_BINNING_ON : OV8830_READ_MODE_BINNING_OFF;
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov8830_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	*value = dev->exposure;
	return 0;
}

static int ov8830_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov8830_write_reg(client, OV8830_16BIT, 0x3070, value);
}

static int ov8830_v_flip(struct v4l2_subdev *sd, s32 value)
{
	return -ENXIO;
}

static int ov8830_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV8830_FOCAL_LENGTH_NUM << 16) | OV8830_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov8830_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ov8830*/
	*val = (OV8830_F_NUMBER_DEFAULT_NUM << 16) | OV8830_F_NUMBER_DEM;
	return 0;
}

static int ov8830_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV8830_F_NUMBER_DEFAULT_NUM << 24) |
		(OV8830_F_NUMBER_DEM << 16) |
		(OV8830_F_NUMBER_DEFAULT_NUM << 8) | OV8830_F_NUMBER_DEM;
	return 0;
}

static int ov8830_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int r = ov8830_get_register(sd, OV8830_TIMING_X_INC,
		ov8830_res[dev->fmt_idx].regs);

	if (r < 0)
		return r;

	*val = fls((r >> 4) + (r & 0xF)) - 2;

	return 0;
}

static int ov8830_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int r = ov8830_get_register(sd, OV8830_TIMING_Y_INC,
		ov8830_res[dev->fmt_idx].regs);

	if (r < 0)
		return r;

	*val = fls((r >> 4) + (r & 0xF)) - 2;

	return 0;
}

static struct ov8830_control ov8830_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov8830_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov8830_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov8830_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = DRV201_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = drv201_t_focus_abs,
		.query = drv201_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = -DRV201_MAX_FOCUS_POS,
			.maximum = DRV201_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = drv201_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = drv201_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV8830_FOCAL_LENGTH_DEFAULT,
			.maximum = OV8830_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV8830_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov8830_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV8830_F_NUMBER_DEFAULT,
			.maximum = OV8830_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV8830_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov8830_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV8830_F_NUMBER_RANGE,
			.maximum =  OV8830_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV8830_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov8830_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = OV8830_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_READ_ONLY,
		},
		.query = ov8830_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = OV8830_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_READ_ONLY,
		},
		.query = ov8830_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov8830_controls))

static struct ov8830_control *ov8830_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov8830_controls[i].qc.id == id)
			return &ov8830_controls[i];
	return NULL;
}

static int ov8830_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct ov8830_control *ctrl = ov8830_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* ov8830 control set/get */
static int ov8830_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct ov8830_control *s_ctrl;
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = ov8830_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov8830_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct ov8830_control *octrl = ov8830_find_control(ctrl->id);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
/* tune this value so that the DVS resolutions get selected properly,
 * but make sure 16:9 does not match 4:3.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
static int distance(struct ov8830_resolution const *res, const u32 w,
				const u32 h)
{
	unsigned int w_ratio = ((res->width<<13)/w);
	unsigned int h_ratio = ((res->height<<13)/h);
	int match   = abs(((w_ratio<<13)/h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)
		|| (match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/*
 * Returns the nearest higher resolution index.
 * @w: width
 * @h: height
 * matching is done based on enveloping resolution and
 * aspect ratio. If the aspect ratio cannot be matched
 * to any index, -1 is returned.
 */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ov8830_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov8830_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov8830_res[i].width)
			continue;
		if (h != ov8830_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int ov8830_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;

	if ((fmt->width > OV8830_RES_WIDTH_MAX) || (fmt->height > OV8830_RES_HEIGHT_MAX)) {
		fmt->width = OV8830_RES_WIDTH_MAX;
		fmt->height = OV8830_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller resolutions.
		 * If it fails, it means the requested resolution is higher than we
		 * can support. Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = ov8830_res[idx].width;
		fmt->height = ov8830_res[idx].height;
	}

	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;


	return 0;
}

static int ov8830_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	const struct ov8830_reg *ov8830_def_reg;
	struct camera_mipi_info *ov8830_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ov8830_info = v4l2_get_subdev_hostdata(sd);
	if (ov8830_info == NULL)
		return -EINVAL;

	ret = ov8830_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}

	mutex_lock(&dev->input_lock);
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);

	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	ov8830_def_reg = ov8830_res[dev->fmt_idx].regs;
	ret = ov8830_write_reg_array(client, ov8830_def_reg);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	dev->fps = ov8830_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ov8830_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ov8830_res[dev->fmt_idx].lines_per_frame;

	ret = ov8830_get_intg_factor(sd, ov8830_info, ov8830_PLL192MHz);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/* restore exposure, gain settings */
	if (dev->exposure) {
		ret = __ov8830_set_exposure(sd, dev->exposure, dev->gain);
		if (ret)
			v4l2_warn(sd, "failed to set exposure time\n");
	}

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ov8830_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = ov8830_res[dev->fmt_idx].width;
	fmt->height = ov8830_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov8830_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (ov8830_read_reg(client, OV8830_8BIT, 0x3001,
			     &high)) {
		v4l2_err(client, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	if (ov8830_read_reg(client, OV8830_8BIT, 0x3002,
			     &low)) {
		v4l2_err(client, "sensor_id_low = 0x%x\n", high);
		return -ENODEV;
	}
	*id = (((u8) high) << 8) | (u8) low;
	v4l2_info(client, "sensor_id = 0x%x\n", *id);
	real_model_id = *id;

	/* Reco settings changes this 0x2a88 from init registers*/
	if (*id != 0x2a88) {
		v4l2_err(client, "sensor ID error\n");
		return -ENODEV;
	}

	v4l2_info(client, "detect ov8830 success\n");

	/* REVISIT: HACK: Driver is currently forcing revision to 0 */
	*revision = 0;

	return 0;
}

/*
 * ov8830 stream on/off
 */
static int ov8830_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	mutex_lock(&dev->input_lock);

	ret = ov8830_write_reg(client, OV8830_8BIT, 0x0100, enable ? 1 : 0);
	if (ret != 0) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}

	dev->streaming = enable;

	/* restore settings */
	ov8830_res = ov8830_res_preview;
	N_RES = N_RES_PREVIEW;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/*
 * ov8830 enum frame size, frame intervals
 */
static int ov8830_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov8830_res[index].width;
	fsize->discrete.height = ov8830_res[index].height;
	fsize->reserved[0] = ov8830_res[index].used;

	return 0;
}

static int ov8830_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	/* since the isp will donwscale the resolution to the right size, find the nearest one that will allow the isp to do so
	 * important to ensure that the resolution requested is padded correctly by the requester, which is the atomisp driver in this case.
	 */
	index = nearest_resolution_index(fival->width, fival->height);

	if (-1 == index)
		index = N_RES - 1;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
/*	fival->width = ov8830_res[index].width;
	fival->height = ov8830_res[index].height; */
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov8830_res[index].fps;

	return 0;
}

static int ov8830_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return 0;
}

static int ov8830_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "ov8830 platform init err\n");
			return ret;
		}
	}

	ret = __ov8830_s_power(sd, 1);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "ov8830 power-up err.\n");
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ov8830_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ov8830_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* power off sensor */
	ret = __ov8830_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(client, "ov8830 power-down err.\n");
		return ret;
	}

	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__ov8830_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
ov8830_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int
ov8830_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov8830_res[index].width;
	fse->min_height = ov8830_res[index].height;
	fse->max_width = ov8830_res[index].width;
	fse->max_height = ov8830_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov8830_get_pad_format(struct ov8830_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		v4l2_err(client, "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

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
ov8830_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov8830_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
ov8830_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov8830_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
ov8830_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ov8830_res = ov8830_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ov8830_res = ov8830_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ov8830_res = ov8830_res_preview;
		N_RES = N_RES_PREVIEW;
	}

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int
ov8830_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;

	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->exposure > dev->lines_per_frame) {
		if (dev->exposure == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal exposure:0x%x\n",
				  __func__, dev->exposure);
			lines_per_frame = dev->exposure;
		} else
			lines_per_frame = dev->exposure + 1;
	} else
		lines_per_frame = dev->lines_per_frame;

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = OV8830_MCLK * 1000000;

	return 0;
}

static const struct v4l2_subdev_video_ops ov8830_video_ops = {
	.s_stream = ov8830_s_stream,
	.enum_framesizes = ov8830_enum_framesizes,
	.enum_frameintervals = ov8830_enum_frameintervals,
	.enum_mbus_fmt = ov8830_enum_mbus_fmt,
	.try_mbus_fmt = ov8830_try_mbus_fmt,
	.g_mbus_fmt = ov8830_g_mbus_fmt,
	.s_mbus_fmt = ov8830_s_mbus_fmt,
	.s_parm = ov8830_s_parm,
	.g_frame_interval = ov8830_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov8830_core_ops = {
	.g_chip_ident = ov8830_g_chip_ident,
	.queryctrl = ov8830_queryctrl,
	.g_ctrl = ov8830_g_ctrl,
	.s_ctrl = ov8830_s_ctrl,
	.s_power = ov8830_s_power,
	.ioctl = ov8830_ioctl,
	.init = ov8830_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops ov8830_pad_ops = {
	.enum_mbus_code = ov8830_enum_mbus_code,
	.enum_frame_size = ov8830_enum_frame_size,
	.get_fmt = ov8830_get_pad_format,
	.set_fmt = ov8830_set_pad_format,
};

static const struct v4l2_subdev_ops ov8830_ops = {
	.core = &ov8830_core_ops,
	.video = &ov8830_video_ops,
	.pad = &ov8830_pad_ops,
};

static const struct media_entity_operations ov8830_entity_ops = {
	.link_setup = NULL,
};

static int ov8830_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int ov8830_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ov8830_device *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ov8830_ops);

	ret = drv201_init(&dev->sd);
	if (ret < 0)
		goto out_free;

	if (client->dev.platform_data) {
		ret = ov8830_s_config(&dev->sd, client->irq,
				      client->dev.platform_data);
		if (ret)
			goto out_free;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &ov8830_entity_ops;
	dev->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ov8830_remove(client);
		return ret;
	}

	return 0;

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

static const struct i2c_device_id ov8830_id[] = {
	{OV8830_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov8830_id);

static struct i2c_driver ov8830_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV8830_NAME,
	},
	.probe = ov8830_probe,
	.remove = ov8830_remove,
	.id_table = ov8830_id,
};

static __init int ov8830_init_mod(void)
{
	return i2c_add_driver(&ov8830_driver);
}

static __exit void ov8830_exit_mod(void)
{
	i2c_del_driver(&ov8830_driver);
}

module_init(ov8830_init_mod);
module_exit(ov8830_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Omnivision OV8830 sensors");
MODULE_LICENSE("GPL");
