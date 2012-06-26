/*
 * Support for lc898211 actuator.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * Author: David Cohen <david.a.cohen@intel.com>
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

#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#include "lc898211.h"

#define to_lc898211_dev(sd) container_of(sd, struct lc898211_dev, sd)

static int
__lc898211_read_reg(struct i2c_client *client, u16 addr, u16 len, u8 reg,
		    u8 *val)
{
	int err;
	struct i2c_msg msg[2] = {
		{
			.addr = addr,
			.flags = 0,
			.len = sizeof(reg),
			.buf = &reg,
		}, {
			.addr = addr,
			.len = len,
			.flags = I2C_M_RD,
			.buf = val,
		}
	};

	if (!client->adapter) {
		v4l2_err(client, "%s: error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (len > LC898211_MAX_I2C_MSG_LEN) {
		v4l2_err(client, "%s: error, too big message length\n",
			 __func__);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != ARRAY_SIZE(msg)) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	return 0;

error:
	v4l2_err(client, "%s: read from offset 0x%x error %d", __func__, reg,
		 err);
	return err;
}

static int
lc898211_read_reg(struct i2c_client *client, u16 len, u8 reg, void *val)
{
	return __lc898211_read_reg(client, client->addr, len, reg, val);
}

static int
lc898211_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret == num_msg)
		return 0;

	return ret > 0 ? -EIO : ret;
}

static int
__lc898211_write_reg(struct i2c_client *client, u16 data_length, u8 reg,
		     u8 *val)
{
	u8 data[3];
	const u16 len = sizeof(u8) + data_length; /* 8-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s: error, no client->adapter\n", __func__);
		return -ENODEV;
	}
	if (data_length > 2) {
		v4l2_err(client, "%s: write error: invalid length type %d\n",
			 __func__, data_length);
		return -EINVAL;
	}

	data[0] = reg;
	data[1] = val[0];
	if (data_length == 2)
		data[2] = val[1];

	return lc898211_i2c_write(client, len, data);
}

static int
lc898211_write_reg8(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = __lc898211_write_reg(client, sizeof(u8), reg, &val);
	if (ret)
		v4l2_err(client, "%s: write error: wrote 0x%x to offset 0x%x "
				 "error %d\n", __func__, val, reg, ret);

	return ret;
}

static int
lc898211_write_reg16(struct i2c_client *client, u8 reg, u16 val)
{
	int ret;
	u8 data[2] = { (u8)(val >> 8), (u8)val };

	ret = __lc898211_write_reg(client, sizeof(u16), reg, data);
	if (ret)
		v4l2_err(client, "%s: write error: wrote 0x%x to offset 0x%x "
				 "error %d\n", __func__, val, reg, ret);

	return ret;
}

static int lc898211_wait_focus(struct v4l2_subdev *sd);
static int lc898211_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
static int __lc898211_q_focus_abs(struct v4l2_subdev *sd, s16 *pos);

static int lc898211_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	int ret;
	s16 step;
	s16 focus;

	/* Correct value */
	value = clamp_t(s16, value, dev->af_tun.focus_abs_max,
			dev->af_tun.focus_abs_min);

	ret = __lc898211_q_focus_abs(sd, &focus);
	if (ret < 0) {
		v4l2_err(client, "%s: error when requesting position\n",
			 __func__);
		return ret;
	}
	step = focus < value ? dev->step : -dev->step;
	ret = lc898211_write_reg16(client, LC898211_REG16_MS1Z12, step);
	if (ret) {
		v4l2_err(client, "%s: error when setting slew\n",
			 __func__);
		return ret;
	}

	ret = lc898211_write_reg8(client, LC898211_REG_STMVINT, dev->timing);
	if (ret < 0) {
		v4l2_err(client, "%s: error when setting timing\n",
			 __func__);
		return ret;
	}

	ret = lc898211_write_reg16(client, LC898211_REG16_STMVEND, value << 6);
	if (ret < 0) {
		v4l2_err(client, "%s: error when setting new position\n",
			 __func__);
		return ret;
	}

	ret = lc898211_write_reg8(client, LC898211_REG_STMVEN,
				  LC898211_REG_STMVEN_MOVE_BUSY);
	if (ret < 0) {
		v4l2_err(client, "%s: error when enabling moving\n",
			 __func__);
		return ret;
	}

	return 0;
}

static int lc898211_wait_focus(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int retry, ret;
	u8 val;

	retry = LC898211_VCM_SLEW_RETRY_MAX;
	while (--retry) {
		s16 pos;
		usleep_range(1000, 1500);
		ret = __lc898211_q_focus_abs(sd, &pos);
		if (ret < 0) {
			v4l2_err(client, "%s: error when checking current "
					 "position\n", __func__);
			return ret;
		}

		if (pos == LC898211_VCM_INVALID_MIN_POS ||
		    pos == LC898211_VCM_INVALID_MAX_POS) {
			v4l2_err(client, "%s: invalid position\n", __func__);
			return ret;
		}

		ret = lc898211_read_reg(client, LC898211_8BIT,
					LC898211_REG_STMVEN, &val);
		if (ret < 0) {
			v4l2_err(client, "%s: error when checking if new "
					 "position is ready\n", __func__);
			return ret;
		}
		if (!(val & LC898211_REG_STMVEN_MOVE_BUSY))
			break;
	}
	if (!retry) {
		v4l2_err(client, "%s: timeout when setting new position\n",
			 __func__);
		/* Clear move bit */
		lc898211_write_reg8(client, LC898211_REG_STMVEN, 0);
		return -ETIMEDOUT;
	}

	usleep_range(5000, 5500);

	retry = LC898211_VCM_STAB_RETRY_MAX;
	while (--retry) {
		usleep_range(1000, 1500);
		ret = lc898211_read_reg(client, LC898211_8BIT,
					LC898211_REG_MSSET, &val);
		if (ret < 0) {
			v4l2_err(client, "%s: error when checking if new "
					 "position is ready\n", __func__);
			return ret;
		}
		if (!(val & LC898211_REG_MSSET_BUSY))
			break;
	}
	if (!retry) {
		v4l2_err(client, "%s: timeout when waiting for lens "
			 "stabilization\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static int lc898211_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	s16 focus;
	int ret;

	ret = __lc898211_q_focus_abs(sd, &focus);
	if (ret < 0) {
		v4l2_info(client, "%s: invalid focus position, assuming "
				  "default\n", __func__);
		focus = (dev->af_tun.focus_abs_max +
					dev->af_tun.focus_abs_min) / 2;
	} else {
		/* Normalizing focus value */
		focus >>= 6;
	}

	return lc898211_t_focus_abs(sd, focus + value);
}

static int lc898211_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int busy;
	int ret;
	u8 val;

	ret = lc898211_read_reg(client, LC898211_8BIT,
			LC898211_REG_STMVEN, &val);
	if (ret < 0) {
		v4l2_err(client, "%s: error when checking if new "
				 "position is ready\n", __func__);
		return ret;
	}
	busy = val & LC898211_REG_STMVEN_MOVE_BUSY;
	if (busy)
		goto out;

	ret = lc898211_read_reg(client, LC898211_8BIT,
			LC898211_REG_MSSET, &val);
	if (ret < 0) {
		v4l2_err(client, "%s: error when checking if lens "
				 "stabilization is ready\n", __func__);
		return ret;
	}
	busy = val & LC898211_REG_MSSET_BUSY;

out:
	if (busy) {
		*value = ATOMISP_FOCUS_STATUS_MOVING |
			 ATOMISP_FOCUS_HP_IN_PROGRESS;
	} else {
		*value = ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE |
			 ATOMISP_FOCUS_HP_COMPLETE;
	}

	return 0;
}

static int __lc898211_q_focus_abs(struct v4l2_subdev *sd, s16 *pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = lc898211_read_reg(client, LC898211_16BIT, LC898211_REG16_RZ,
				pos);
	if (ret < 0) {
		v4l2_err(client, "%s: failed to read lens position\n",
			 __func__);
		return ret;
	}

	*pos = be16_to_cpu(*pos);
	if (*pos == LC898211_VCM_INVALID_MIN_POS ||
	    *pos == LC898211_VCM_INVALID_MAX_POS)
		return -EINVAL;
	*pos >>= 6;

	return 0;
}

static int lc898211_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	s16 focus;
	int ret;

	ret = __lc898211_q_focus_abs(sd, &focus);
	if (ret < 0)
		return ret;

	*value = focus;

	return 0;
}

static int lc898211_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);

	if (value & ~LC898211_VCM_SLEW_MASK || value > LC898211_VCM_SLEW_MAX)
		return -EINVAL;

	dev->step = value;

	return 0;
}

static int lc898211_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);

	if (value < 0 || value > LC898211_VCM_SLEW_TIME_MAX)
		return -EINVAL;

	dev->timing = value;

	return 0;
}

static struct lc898211_control lc898211_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = lc898211_t_focus_abs,
		.query = lc898211_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = lc898211_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = lc898211_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.step = 1,
			.default_value = LC898211_VCM_SLEW_DEFAULT,
			.flags = 0,
		},
		.tweak = lc898211_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = lc898211_t_vcm_timing,
	},
};
#define N_CONTROLS (ARRAY_SIZE(lc898211_controls))

static struct lc898211_control *lc898211_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (lc898211_controls[i].qc.id == id)
			return &lc898211_controls[i];
	return NULL;
}

static int lc898211_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	struct lc898211_control *ctrl = lc898211_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int lc898211_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	struct lc898211_control *s_ctrl;
	int ret;

	s_ctrl = lc898211_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int lc898211_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	struct lc898211_control *octrl = lc898211_find_control(ctrl->id);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/*
 * Normalize AF tuning values in 2 steps:
 * 1: Values come in big endian format
 * 2: Values are s10 written in the most significant 10 bits of s16.
 */
static void lc898211_get_af_tuning_value(unsigned char *buf, int offset,
					 s16 *value)
{
	*value = (s16)be16_to_cpu(*(s16 *)&buf[offset]);
	*value >>= 6;
}

static int lc898211_get_af_tuning(struct v4l2_subdev *sd)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);

	if (!dev->eeprom_buf || dev->eeprom_size < LC898211_EEP_AF_TUN_END)
		return -ENODEV;

	lc898211_get_af_tuning_value(dev->eeprom_buf, LC898211_EEP_INF1,
				     &dev->af_tun.focus_abs_min);
	lc898211_get_af_tuning_value(dev->eeprom_buf, LC898211_EEP_MAC1,
				     &dev->af_tun.focus_abs_max);
	lc898211_get_af_tuning_value(dev->eeprom_buf, LC898211_EEP_INF2,
				     &dev->af_tun.inf_pos);
	lc898211_get_af_tuning_value(dev->eeprom_buf, LC898211_EEP_MAC2,
				     &dev->af_tun.mac_pos);

	return 0;
}

/*
 * Read EEPROM data from the EEPROM chip and store
 * it into a kmalloced buffer. On error return NULL.
 * The caller must kfree the buffer when no more needed.
 * @size: set to the size of the returned EEPROM data.
 */
static void *lc898211_eeprom_read(struct i2c_client *client, int *size)
{
	struct i2c_msg msg[2];
	int addr;
	char *buffer;

	buffer = kmalloc(LC898211_EEP_SIZE, GFP_KERNEL);
	if (!buffer)
		return NULL;

	memset(msg, 0, sizeof(msg));
	for (addr = 0;
	     addr < LC898211_EEP_SIZE; addr += LC898211_MAX_I2C_MSG_LEN) {
		unsigned int i2c_addr = LC898211_EEP_ID_BASE;
		unsigned char addr_buf;
		int r;

		i2c_addr += addr >> 8;
		addr_buf = addr & 0xFF;

		msg[0].addr = i2c_addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &addr_buf;

		msg[1].addr = i2c_addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = min(LC898211_MAX_I2C_MSG_LEN,
				 LC898211_EEP_SIZE - addr);
		msg[1].buf = &buffer[addr];

		r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (r != ARRAY_SIZE(msg)) {
			kfree(buffer);
			dev_err(&client->dev, "read failed at 0x%03x\n", addr);
			return NULL;
		}
	}

	if (size)
		*size = LC898211_EEP_SIZE;
	return buffer;
}

static int lc898211_init_registers(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	struct lc898211_eeprom_data *edata;
	int i;

	/* Read actuator initialization registers list from EEPROM */

	if (!dev->eeprom_buf ||
	    dev->eeprom_size <
	    LC898211_EEP_NUM_DATA * sizeof(struct lc898211_eeprom_data) +
	    LC898211_EEP_START_ADDR)
		return -ENODEV;

	edata = (struct lc898211_eeprom_data *)
		(dev->eeprom_buf + LC898211_EEP_START_ADDR);

	for (i = 0; i < LC898211_EEP_NUM_DATA; i++, edata++) {
		int len;
		int ret;
		u8 dest_data[2];

		if (edata->addr == LC898211_EEP_ADDR_EOF)
			break;

		if (edata->addr == LC898211_EEP_ADDR_DELAY) {
			u16 delay = be16_to_cpu(*(u16 *)edata->data);
			usleep_range(delay * 1000, delay * 1500);
			continue;
		}

		len = edata->size;
		if (len > LC898211_EEP_DATA_SIZE_MAX) {
			v4l2_err(client, "%s: malformed initialization data "
				 "from EEPROM: data size too long.\n",
				 __func__);
			return -EINVAL;
		}

		switch (edata->type) {
		case LC898211_EEP_DATA_DIRECT:
			dest_data[0] = edata->data[0];
			dest_data[1] = edata->data[1];
			break;
		case LC898211_EEP_DATA_INDIRECT_EEP:
			if (len > sizeof(dest_data) ||
			    edata->data[0] + len > dev->eeprom_size) {
				v4l2_err(client, "%s: error reading indirect "
					 "data from EEPROM.\n", __func__);
				return -ENODEV;
			}
			memcpy(dest_data, &dev->eeprom_buf[edata->data[0]],
			       len);
			break;
		case LC898211_EEP_DATA_INDIRECT_HVCA:
			ret = lc898211_read_reg(client, len, edata->data[0],
						dest_data);
			if (ret < 0) {
				v4l2_err(client, "%s: error reading indirect "
						 "data from HVCA.\n", __func__);
				return ret;
			}
			break;
		case LC898211_EEP_DATA_MASK_AND:
			ret = lc898211_read_reg(client, len, edata->addr,
						dest_data);
			if (ret < 0) {
				v4l2_err(client, "%s: error reading indirect "
						 "data from HVCA.\n", __func__);
				return ret;
			}
			dest_data[0] &= edata->data[0];
			dest_data[1] &= edata->data[1];
			break;
		case LC898211_EEP_DATA_MASK_OR:
			ret = lc898211_read_reg(client, len, edata->addr,
						dest_data);
			if (ret < 0) {
				v4l2_err(client, "%s: error reading indirect "
						 "data from HVCA.\n", __func__);
				return ret;
			}
			dest_data[0] |= edata->data[0];
			dest_data[1] |= edata->data[1];
			break;
		default:
			v4l2_err(client, "%s: malformed initialization data "
				 "from EEPROM: unknown data type.\n", __func__);
			return -EINVAL;
		}

		ret = __lc898211_write_reg(client, edata->size, edata->addr,
					   dest_data);
		if (ret < 0) {
			v4l2_err(client, "%s: error writing initialization "
					 "data.\n", __func__);
			return ret;
		}
	}

	return 0;
}

static int lc898211_init(struct v4l2_subdev *sd, u32 val)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	/* set inital registers */
	ret = lc898211_t_vcm_slew(sd, LC898211_VCM_SLEW_DEFAULT);
	if (ret)
		goto out;
	ret = lc898211_init_registers(sd);
	if (ret)
		goto out;
	ret = lc898211_t_vcm_timing(sd, LC898211_VCM_SLEW_TIME_DEFAULT);
	if (ret)
		goto out;

	/* set VCM to home position */
	ret = lc898211_t_focus_abs(sd,
			(dev->af_tun.inf_pos + dev->af_tun.mac_pos) / 2);

out:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int lc898211_power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lc898211_dev *dev = to_lc898211_dev(sd);
	int ret;

       /* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	msleep(20);

	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int lc898211_power_down(struct v4l2_subdev *sd)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);
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

static int lc898211_g_priv_int_data(struct v4l2_subdev *sd,
				    struct v4l2_private_int_data *priv)
{
	struct lc898211_dev *dev = to_lc898211_dev(sd);

	if (copy_to_user(priv->data, dev->eeprom_buf,
			 min(priv->size, dev->eeprom_size)))
		return -EFAULT;
	priv->size = dev->eeprom_size;

	return 0;
}

static long lc898211_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_G_MOTOR_PRIV_INT_DATA:
		return lc898211_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_subdev_core_ops lc898211_core_ops = {
	.queryctrl = lc898211_queryctrl,
	.g_ctrl = lc898211_g_ctrl,
	.s_ctrl = lc898211_s_ctrl,
	.init = lc898211_init,
	.ioctl = lc898211_ioctl,
};

static const struct v4l2_subdev_ops lc898211_ops = {
	.core = &lc898211_core_ops,
};

static int lc898211_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898211_dev *dev = to_lc898211_dev(sd);

	kfree(dev->eeprom_buf);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int lc898211_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct lc898211_dev *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	v4l2_i2c_subdev_init(&dev->sd, client, &lc898211_ops);

	dev->platform_data = client->dev.platform_data;
	if (!dev->platform_data)
		v4l2_info(client, "%s: driver has no platform data\n",
			  __func__);

	lc898211_power_up(&dev->sd);

	dev->eeprom_buf = lc898211_eeprom_read(client, &dev->eeprom_size);
	if (!dev->eeprom_buf) {
		ret = -ENODEV;
		v4l2_err(client, "%s: failed to read EEPROM\n",
			 __func__);
		goto err;
	}

	ret = lc898211_get_af_tuning(&dev->sd);
	if (ret) {
		v4l2_err(client, "%s: failed to read AF tuning data\n",
			 __func__);
		goto err;
	}

	lc898211_power_down(&dev->sd);

	v4l2_info(client, "LC898211 actuator successfully initialized\n");

	return 0;

err:
	lc898211_remove(client);
	return ret;
}

static const struct i2c_device_id lc898211_id[] = {
	{LC898211_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lc898211_id);

static struct i2c_driver lc898211_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LC898211_NAME,
	},
	.probe = lc898211_probe,
	.remove = lc898211_remove,
	.id_table = lc898211_id,
};

static __init int init_lc898211(void)
{
	return i2c_add_driver(&lc898211_driver);
}

static __exit void exit_lc898211(void)
{
	i2c_del_driver(&lc898211_driver);
}

module_init(init_lc898211);
module_exit(exit_lc898211);

MODULE_DESCRIPTION("A low-level driver for LC898211 actuator");
MODULE_AUTHOR("David Cohen <david.a.cohen@intel.com>");
MODULE_LICENSE("GPL");

