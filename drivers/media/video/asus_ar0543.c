/*
 * Support for Aptina ar0543 1080p HD camera sensor.
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
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_rpmsg.h>
#include "asus_ar0543.h"

#define to_ar0543_sensor(sd) container_of(sd, struct ar0543_device, sd)

#define HOME_POS 255
#define PR3_2_FW 0x0400
#define PR3_3_FW 0x0500

/* divides a by b using half up rounding and div/0 prevention
 * (result is 0 if b == 0) */
#define divsave_rounded(a, b)	(((b) != 0) ? (((a)+((b)>>1))/(b)) : (-1))

//Add for ATD read camera status+++
int ATD_ar0543_status = 0;  //Add for ATD read camera status
static ssize_t ar0543_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	//printk("%s: get ar0543 status (%d) !!\n", __func__, ATD_ar0543_status);
   	//Check sensor connect status, just do it  in begining for ATD camera status

	return sprintf(buf,"%d\n", ATD_ar0543_status);
}

static DEVICE_ATTR(ar0543_status, S_IRUGO,ar0543_show_status,NULL);

static struct attribute *ar0543_attributes[] = {
	&dev_attr_ar0543_status.attr,
	NULL
};
//Add for ATD read camera status---

//Add for vcm +++
static int vcm_i2c_rd8(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	buf[0] = reg;
	buf[1] = 0;

	msg[0].addr = VCM_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];

	msg[1].addr = VCM_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	*val = 0;
	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -EIO;
	*val = buf[1];
	return 0;
}

static int vcm_i2c_wr8(struct i2c_client *client, u8 reg, u8 val)
{
        int err;
	struct i2c_msg msg;
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;
	msg.addr = 0xc;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &buf[0];

        err = i2c_transfer(client->adapter, &msg, 1);

	if (err != 1){
                printk("%s: rear camera vcm i2c fail, err code = %d\n", __func__, err);
		return -EIO;
        }
	return 0;
}
//Add for vcm ---

static int
ar0543_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	u16 data[AR0543_SHORT_MAX];
	int err, i;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = I2C_MSG_LENGTH,
			.buf = (unsigned char *)&reg,
		}, {
			.addr = client->addr,
			.len = len,
			.flags = I2C_M_RD,
			.buf = (u8 *)data,
		}
	};

	reg = cpu_to_be16(reg);

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > AR0543_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == AR0543_8BIT) {
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

static int ar0543_i2c_write(struct i2c_client *client, u16 len, u8 *data)
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
		dev_dbg(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

static int
ar0543_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != AR0543_8BIT && data_length != AR0543_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}
	/* high byte goes out first */
	*(u16 *)data = cpu_to_be16(reg);
	switch (data_length) {
	case AR0543_8BIT:
		data[2] = (u8)val;
		break;
	case AR0543_16BIT:
		*(u16 *)&data[2] = cpu_to_be16(val);
		break;
	default:
		dev_err(&client->dev,
			"write error: invalid length type %d\n", data_length);
	}

	ret = ar0543_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/**
 * ar0543_rmw_reg - Read/Modify/Write a value to a register in the sensor
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
static int ar0543_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
			   u16 mask, u16 set)
{
	int err;
	u16 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	if (data_length == AR0543_8BIT && mask & ~0xff)
		return -EINVAL;

	err = ar0543_read_reg(client, data_length, reg, &val);
	if (err) {
		v4l2_err(client, "ar0543_rmw_reg error exit, read failed\n");
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

	err = ar0543_write_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "ar0543_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


/*
 * ar0543_write_reg_array - Initializes a list of ar0543 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ar0543_flush_reg_array, __ar0543_buf_reg_array() and
 * __ar0543_write_reg_is_consecutive() are internal functions to
 * ar0543_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ar0543_flush_reg_array(struct i2c_client *client,
				     struct ar0543_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ar0543_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ar0543_buf_reg_array(struct i2c_client *client,
				   struct ar0543_write_ctrl *ctrl,
				   const struct ar0543_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case AR0543_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case AR0543_16BIT:
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
	if (ctrl->index + sizeof(u16) >= AR0543_MAX_WRITE_BUF_SIZE)
		return __ar0543_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ar0543_write_reg_is_consecutive(struct i2c_client *client,
				   struct ar0543_write_ctrl *ctrl,
				   const struct ar0543_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ar0543_write_reg_array(struct i2c_client *client,
				   const struct ar0543_reg *reglist)
{
	const struct ar0543_reg *next = reglist;
	struct ar0543_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != AR0543_TOK_TERM; next++) {
		switch (next->type & AR0543_TOK_MASK) {
		case AR0543_TOK_DELAY:
			err = __ar0543_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		case AR0543_RMW:
			err = __ar0543_flush_reg_array(client, &ctrl);
			err |= ar0543_rmw_reg(client,
					       next->type & ~AR0543_RMW,
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
			if (!__ar0543_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ar0543_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ar0543_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ar0543_flush_reg_array(client, &ctrl);
}

static int ar0543_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int ret = 0;
	u8 msb, lsb;

        value = clamp(value, 0, VCM_MAX_FOCUS_POS);
        msb = (value >> 8 & 0xff);
        lsb = (value & 0xff);

        // printk("%s: ========== FOCUS_POS:%x \n", __func__, value);
        vcm_i2c_wr8(client, 0x04, msb);
        vcm_i2c_wr8(client, 0x05, lsb);
	//ret = ar0543_write_reg(client, AR0543_16BIT, AR0543_VCM_CODE, value);
	//if (ret == 0) {
	//	dev->number_of_steps = value - dev->focus;
	//	dev->focus = value;
	//	getnstimeofday(&(dev->timestamp_t_focus_abs));
	//}
	return ret;
}

static int ar0543_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	return ar0543_t_focus_abs(sd, dev->focus + value);
}

#define DELAY_PER_STEP_NS	1000000
#define DELAY_MAX_PER_STEP_NS	(1000000*40)
static int ar0543_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	u32 status = 0;
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct timespec temptime;
	const struct timespec timedelay = {
		0,
		min((u32)abs(dev->number_of_steps)*DELAY_PER_STEP_NS,
			(u32)DELAY_MAX_PER_STEP_NS),
	};

	getnstimeofday(&temptime);

	temptime = timespec_sub(temptime, (dev->timestamp_t_focus_abs));

	if (timespec_compare(&temptime, &timedelay) <= 0) {
		status |= ATOMISP_FOCUS_STATUS_MOVING;
		status |= ATOMISP_FOCUS_HP_IN_PROGRESS;
	} else {
		status |= ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
		status |= ATOMISP_FOCUS_HP_COMPLETE;
	}
	*value = status;
	return 0;
}

static int ar0543_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	s32 val;

	ar0543_q_focus_status(sd, &val);

	if (val & ATOMISP_FOCUS_STATUS_MOVING)
		*value  = dev->focus - dev->number_of_steps;
	else
		*value  = dev->focus ;

	return 0;
}
static int ar0543_real_to_register_gain(u16 gain, u16 *real_gain)
{

                u16 reg_gain;
                u16 cg, asc1;
		u16 gain_value;
		u16 calculated_gain;
#if 0
                if (_gain > 255)
			gain = 255 * 256; /*Cap max gain to use only analog portion.*/
		else
			gain = _gain * 256;
#endif
                if (gain < 22) /*Use 2nd stage as soon as possible*/
                {
				calculated_gain = gain * 2;
                                cg = 0x0;
                                asc1 = 0x0;
                }
                else if (gain < 32)
                {
				calculated_gain = gain * 6 / 4;
                                cg = 0x0;
                                asc1 = 0x100;
                }
                else if (gain < 43)
                {
				calculated_gain = gain;
                                cg = 0x800;
                                asc1 = 0x0;
                }
                else if (gain < 48)
                {
				calculated_gain = gain * 768 / 0x400;
                                cg = 0x800;
                                asc1 = 0x100;
                }
                else if (gain < 64)
                {
				calculated_gain = gain * 682 / 0x400;
                                cg = 0x400;
                                asc1 = 0x0;
                }
                else if (gain < 85)
                {
				calculated_gain = gain * 2 / 4;
                                cg = 0xc00;
                                asc1 = 0x0;
                }
                else if (gain < 128)
                {
				calculated_gain = gain * 384 / 0x400;
                                cg = 0xc00;
                                asc1 = 0x100;
                } 
                else
                {
				calculated_gain = gain / 4;
                                cg = 0xc00;
                                asc1 = 0x200;
                }
		reg_gain = calculated_gain;
		//printk("%s gain %d calculated_gain %x\n", __func__, gain, reg_gain);
                reg_gain |= cg;
                reg_gain |= asc1;
		*real_gain = reg_gain;

		return 0;
}

static long ar0543_set_exposure(struct v4l2_subdev *sd, u16 coarse_itg,
				 u16 fine_itg, u16 a_gain, u16 d_gain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 frame_length;
	u16 real_gain;
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	mutex_lock(&dev->input_lock);

	ret = ar0543_read_reg(client, AR0543_16BIT,
			       AR0543_FRAME_LENGTH_LINES, &frame_length);
	if (ret)
		goto out;

	/* enable group hold */
	ret = ar0543_write_reg_array(client, ar0543_param_hold);
	if (ret)
		goto out;

	if (frame_length < coarse_itg)
	{
		frame_length = coarse_itg + 5;
		ret = ar0543_write_reg(client, AR0543_16BIT,
			AR0543_FRAME_LENGTH_LINES, frame_length);
		if (ret)
			goto out_disable;
	}

	/* set coarse integration time */
	ret = ar0543_write_reg(client, AR0543_16BIT,
			AR0543_COARSE_INTEGRATION_TIME, coarse_itg);
	if (ret)
		goto out_disable;

	/* set fine integration time */
	ret = ar0543_write_reg(client, AR0543_16BIT,
			AR0543_FINE_INTEGRATION_TIME, fine_itg);
	if (ret)
		goto out_disable;

	ret = ar0543_real_to_register_gain(a_gain, &real_gain);
	real_gain |= 0x1000;

	//printk("%s1 write gain %x fine %x coarse %x frame_length %x\n", __func__,
	//	real_gain, fine_itg, coarse_itg, frame_length);

	/* set global gain */
	ret = ar0543_write_reg(client, AR0543_16BIT,
			AR0543_GLOBAL_GAIN, real_gain);

	if (ret)
		goto out_disable;


	dev->gain       = real_gain;
	dev->coarse_itg = coarse_itg;
	dev->fine_itg   = fine_itg;

out_disable:
	/* disable group hold */
	ar0543_write_reg_array(client, ar0543_param_update);
out:
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long ar0543_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg, fine_itg, analog_gain, digital_gain;

	coarse_itg = exposure->integration_time[0];
	fine_itg = exposure->integration_time[1];
	analog_gain = exposure->gain[0];
	digital_gain = exposure->gain[1];

	/* we should not accept the invalid value below */
	if (fine_itg == 0 || analog_gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	//printk("%s analog %x digital %x\n", __func__, analog_gain, digital_gain);
	return ar0543_set_exposure(sd, coarse_itg, fine_itg, analog_gain, digital_gain);
}

static int ar0543_read_reg_array(struct i2c_client *client, u16 size, u16 addr,
				  void *data)
{
	u8 *buf = data;
	u16 index;
	int ret = 0;

	for (index = 0; index + AR0543_BYTE_MAX <= size;
	     index += AR0543_BYTE_MAX) {
		ret = ar0543_read_reg(client, AR0543_BYTE_MAX, addr + index,
				       (u16 *)&buf[index]);
		if (ret)
			return ret;
	}

	if (size - index > 0)
		ret = ar0543_read_reg(client, size - index, addr + index,
				       (u16 *)&buf[index]);

	return ret;
}

static int
__ar0543_otp_read(struct v4l2_subdev *sd, struct ar0543_af_data *buf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int retry = 100;
	u16 ready;

	/* RESET_REGISTER_REG_RD_EN */
	ret = ar0543_rmw_reg(client, AR0543_16BIT, AR0543_OTP_READ_EN, 0x20, 1);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}
	ar0543_read_reg(client, AR0543_16BIT, AR0543_OTP_READ_EN, &ready);

	/* disable streaming */
	ret = ar0543_write_reg(client, AR0543_16BIT,
				AR0543_OTP_READ_EN, 0x0610);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}
	ar0543_read_reg(client, AR0543_16BIT, AR0543_OTP_READ_EN, &ready);

	/* timing parameters for otp read */
	ret = ar0543_write_reg(client, AR0543_16BIT,
				AR0543_OTP_TIMING, 0xCD95);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}
	ar0543_read_reg(client, AR0543_16BIT, AR0543_OTP_TIMING, &ready);

	/* choose to only read record type 0x30 */
	ret = ar0543_rmw_reg(client, AR0543_16BIT,
				AR0543_OTP_RECORD_TYPE, 0xFF00, 0x30);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}
	ar0543_read_reg(client, AR0543_16BIT, AR0543_OTP_RECORD_TYPE, &ready);

	/* auto read start */
	ret = ar0543_write_reg(client, AR0543_16BIT,
				AR0543_OTP_AUTO_READ, 0x0010);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}
	ar0543_read_reg(client, AR0543_16BIT, AR0543_OTP_AUTO_READ, &ready);

	do {
		ret = ar0543_read_reg(client, AR0543_16BIT,
				       AR0543_OTP_AUTO_READ, &ready);
		if (ret) {
			v4l2_err(client, "%s: failed to read OTP memory "
					 "status\n", __func__);
			return ret;
		}
		if (ready & AR0543_OTP_READY_REG_DONE)
			break;
	} while (--retry);

	if (!retry) {
		v4l2_err(client, "%s: OTP memory read timeout.\n", __func__);
		return -ETIMEDOUT;
	}

	if (!(ready & AR0543_OTP_READY_REG_OK)) {
		v4l2_err(client, "%s: OTP memory was initialized with error\n",
			  __func__);
		return -EIO;
	}

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_OTP_AF_INF_POS, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s Inf AF position %x : %x\n", __func__,
		//	AR0543_OTP_AF_INF_POS, ready);
		buf->af_inf_pos = ready;
	}

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_OTP_AF_1M_POS, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s 1M AF position %x : %x\n", __func__,
		//	AR0543_OTP_AF_1M_POS, ready);
		buf->af_1m_pos = ready;
	}

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_OTP_AF_10CM_POS, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s 10cm AF position %x : %x\n", __func__,
		//	AR0543_OTP_AF_10CM_POS, ready);
		buf->af_10cm_pos = ready;
	}

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_OTP_AF_START_CURR, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s start current %x : %x\n", __func__,
		//	AR0543_OTP_AF_START_CURR, ready);
		buf->af_start_curr = ready;
	}

	ret = ar0543_read_reg(client, AR0543_8BIT,
				AR0543_OTP_AF_MODULE_ID, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s module ID %x : %x\n", __func__,
		//	AR0543_OTP_AF_MODULE_ID, ready);
		buf->module_id = ready;
	}

	ret = ar0543_read_reg(client, AR0543_8BIT,
				AR0543_OTP_AF_VENDOR_ID, &ready);
	if (ret) {
		v4l2_err(client, "%s: failed to read OTP data\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s vendor ID %x : %x\n", __func__,
		//	AR0543_OTP_AF_VENDOR_ID, ready);
		buf->vendor_id = ready;
	}
	return 0;
}

static void *ar0543_otp_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void *buf;
	int ret;

	buf = kmalloc(AR0543_OTP_DATA_SIZE, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	/*
	 * Try all banks in reverse order and return after first success.
	 * Last used bank has most up-to-date data.
	 */
	ret = __ar0543_otp_read(sd, buf);
	/* Driver has failed to find valid data */
	if (ret) {
		v4l2_err(client, "%s: sensor found no valid OTP data\n",
			  __func__);
		kfree(buf);
		return ERR_PTR(ret);
	}

	return buf;
}

static u8 *ar0543_fuseid_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 data[4];
	u16 *fuseid;
	int ret, i;

	/* RESET_REGISTER_REG_RD_EN */
	ret = ar0543_rmw_reg(client, AR0543_16BIT, AR0543_OTP_READ_EN, 0x20, 1);
	if (ret) {
		v4l2_err(client, "%s: failed to prepare OTP memory\n",
			 __func__);
		return ret;
	}

	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_FUSEID_1, &data[0]);
	if (ret) {
		v4l2_err(client, "%s: failed to read AR0543_FUSEID_1\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s fuse ID %x : %x\n", __func__,
		//	AR0543_FUSEID_1, data[0]);
	}

	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_FUSEID_2, &data[1]);
	if (ret) {
		v4l2_err(client, "%s: failed to read AR0543_FUSEID_2\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s fuse ID %x : %x\n", __func__,
		//	AR0543_FUSEID_2, data[1]);
	}
	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_FUSEID_3, &data[2]);
	if (ret) {
		v4l2_err(client, "%s: failed to read AR0543_FUSEID_3\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s fuse ID %x : %x\n", __func__,
		//	AR0543_FUSEID_3, data[2]);
	}
	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_FUSEID_4, &data[3]);
	if (ret) {
		v4l2_err(client, "%s: failed to read AR0543_FUSEID_4\n", __func__);
		return ret;
	} else {
		//printk(KERN_DEBUG "%s fuse ID %x : %x\n", __func__,
		//	AR0543_FUSEID_4, data[3]);
	}

	fuseid = kmalloc(AR0543_FUSEID_SIZE, GFP_KERNEL);

	if (!fuseid) {
		v4l2_err(client, "%s: no memory available when reading "
				 "FUSEID.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* FUSEID is in reverse order */
	for (i = 0; i < AR0543_FUSEID_NUM; i++)
		fuseid[i] = data[AR0543_FUSEID_NUM - i - 1];

	return fuseid;
}

static int ar0543_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	/* No OTP data available on sensor */
	if (!dev->otp_data || !dev->fuseid)
		return -EIO;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	/* Correct read_size value only if bigger than maximum */
	if (read_size > AR0543_OTP_DATA_SIZE)
		read_size = AR0543_OTP_DATA_SIZE;

	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}

	/* No room for FUSEID */
	if (priv->size <= AR0543_OTP_DATA_SIZE)
		goto out;

	read_size = priv->size - AR0543_OTP_DATA_SIZE;
	if (read_size > AR0543_FUSEID_SIZE)
		read_size = AR0543_FUSEID_SIZE;
	to += AR0543_OTP_DATA_SIZE;

	ret = copy_to_user(to, dev->fuseid, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy FUSEID to user\n",
			 __func__);
		return -EFAULT;
	}

out:
	/* Return correct size */
	priv->size = AR0543_OTP_DATA_SIZE + AR0543_FUSEID_SIZE;

	return 0;
}

static long ar0543_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ar0543_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ar0543_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ar0543_init_registers(struct v4l2_subdev *sd)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        vcm_i2c_wr8(client, 0x01, 0x01); // vcm init test
	ret  = ar0543_write_reg_array(client, ar0543_reset_register);

	return ret;
}

static int __ar0543_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	u8 fw_rev[16];

	/* set inital registers */
	ret = ar0543_init_registers(sd);

	/*set VCM to home position */
	//ret |= ar0543_t_focus_abs(sd, HOME_POS);

	/* restore settings */
	ar0543_res = ar0543_res_preview;
	N_RES = N_RES_PREVIEW;

	/* The only way to detect whether this VCM maintains focus after putting
	 * the sensor into standby mode is by looking at the SCU FW version.
	 * PR3.3 or higher runs on FW version 05.00 or higher.
	 * We cannot distinguish between PR3.2 and PR3.25, so we need to be
	 * conservative. PR3.25 owners can change the comparison to compare
	 * to PR3_2_FW instead of PR3_3_FW for testing purposes.
	 */
	dev->keeps_focus_pos = false;
	ret |= rpmsg_send_generic_command(IPCMSG_FW_REVISION, 0, NULL, 0,
				       (u32 *)fw_rev, 4);
	if (ret == 0) {
		u16 fw_version = (fw_rev[15] << 8) | fw_rev[14];
		dev->keeps_focus_pos = fw_version >= PR3_3_FW;
	}
	if (!dev->keeps_focus_pos) {
		v4l2_warn(sd, "VCM does not maintain focus position in standby"
			      "mode, using software workaround\n");
	}

	return ret;
}

static int ar0543_init(struct v4l2_subdev *sd, u32 val)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int ret;
	mutex_lock(&dev->input_lock);
	ret = __ar0543_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static void ar0543_uninit(struct v4l2_subdev *sd)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	dev->coarse_itg = 0;
	dev->fine_itg   = 0;
	dev->gain       = 0;
	dev->focus      = AR0543_INVALID_CONFIG;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_device *dev = to_ar0543_sensor(sd);
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

static int power_down(struct v4l2_subdev *sd)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
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

static int __ar0543_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int ret;

	if (on == 0) {
		ar0543_uninit(sd);
		ret = power_down(sd);
		dev->power = 0;
	} else {
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			/* init motor initial position */
			//return __ar0543_init(sd, 0);
		}
		ar0543_init_registers(sd);
	}

	return ret;
}

static int ar0543_s_power(struct v4l2_subdev *sd, int on)
{
    printk("%s: s\n",__FUNCTION__);
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ar0543_s_power(sd, on);
	mutex_unlock(&dev->input_lock);
    printk("%s: e\n",__FUNCTION__);
	return ret;
}

static int ar0543_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_AR0543, 0);

	return 0;
}

static int
ar0543_get_intg_factor(struct i2c_client *client,
			struct camera_mipi_info *info,
			const struct ar0543_reg *reglist)
{
	unsigned int	vt_pix_clk_div;
	unsigned int	vt_sys_clk_div;
	unsigned int	pre_pll_clk_div;
	unsigned int	pll_multiplier;
	unsigned int	op_pix_clk_div;
	unsigned int	op_sys_clk_div;

    /* TODO: this should not be a constant but should be set by a call to
     * MSIC's driver to get the ext_clk that MSIC supllies to the sensor.
     */
	const int ext_clk_freq_mhz = 19200000;
	struct atomisp_sensor_mode_data buf;
	const struct ar0543_reg *next = reglist;
	int vt_pix_clk_freq_mhz;
	u16 data[AR0543_SHORT_MAX];

	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int read_mode;
	u16 value;
	int ret;

	//printk("%s\n",__FUNCTION__);

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, AR0543_SHORT_MAX * sizeof(u16));
	if (ar0543_read_reg(client, 12, AR0543_VT_PIX_CLK_DIV, data))
		return -EINVAL;
	vt_pix_clk_div = data[0];
	vt_sys_clk_div = data[1];
	pre_pll_clk_div = data[2];
	pll_multiplier = data[3];
	op_pix_clk_div = data[4];
	op_sys_clk_div = data[5];

	memset(data, 0, AR0543_SHORT_MAX * sizeof(u16));
	if (ar0543_read_reg(client, 4, AR0543_FRAME_LENGTH_LINES, data))
		return -EINVAL;
	frame_length_lines = data[0];
	line_length_pck = data[1];

	memset(data, 0, AR0543_SHORT_MAX * sizeof(u16));
	if (ar0543_read_reg(client, 8, AR0543_COARSE_INTG_TIME_MIN, data))
		return -EINVAL;
	coarse_integration_time_min = data[0];
	coarse_integration_time_max_margin = data[1];
	fine_integration_time_min = data[2];
	fine_integration_time_max_margin = data[3];

	memset(data, 0, AR0543_SHORT_MAX * sizeof(u16));
	if (ar0543_read_reg(client, 2, AR0543_READ_MODE, data))
		return -EINVAL;
	read_mode = data[0];

	vt_pix_clk_freq_mhz = divsave_rounded(ext_clk_freq_mhz*pll_multiplier,
				pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div);

	for (; next->type != AR0543_TOK_TERM; next++) {
		if (next->type == AR0543_16BIT) {
			if (next->reg.sreg == AR0543_FINE_INTEGRATION_TIME) {
				buf.fine_integration_time_def = next->val;
				break;
			}
		}
	}

	/* something's wrong here, this mode does not have fine_igt set! */
	if (next->type == AR0543_TOK_TERM)
		return -EINVAL;

	buf.coarse_integration_time_min = coarse_integration_time_min;
	buf.coarse_integration_time_max_margin =
					coarse_integration_time_max_margin;
	buf.fine_integration_time_min = fine_integration_time_min;
	buf.fine_integration_time_max_margin = fine_integration_time_max_margin;
	buf.vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf.line_length_pck = line_length_pck;
	buf.frame_length_lines = frame_length_lines;
	buf.read_mode = read_mode;

	/* 1: normal 3:inc 2, 7:inc 4 addresses in X direction*/
	buf.binning_factor_x =
		(((read_mode & AR0543_READ_MODE_X_ODD_INC) >> 6) + 1) / 2;

	/*
	 * 1:normal 3:inc 2, 7:inc 4, 15:inc 8, 31:inc 16, 63:inc 32 addresses
	 * in Y direction
	 */
	buf.binning_factor_y =
			((read_mode & AR0543_READ_MODE_Y_ODD_INC) + 1) / 2;

	/* Get the cropping and output resolution to ISP for this mode. */
	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_HORIZONTAL_START_H, &value);
	if (ret)
		return ret;
	buf.crop_horizontal_start = value;

	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_VERTICAL_START_H,
				&value);
	if (ret)
		return ret;
	buf.crop_vertical_start = value;

	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_HORIZONTAL_END_H,
				&value);
	if (ret)
		return ret;
	buf.crop_horizontal_end = value;

	ret = ar0543_read_reg(client, AR0543_16BIT, AR0543_VERTICAL_END_H,
				&value);
	if (ret)
		return ret;
	buf.crop_vertical_end = value;

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_HORIZONTAL_OUTPUT_SIZE_H, &value);
	if (ret)
		return ret;
	buf.output_width = value;

	ret = ar0543_read_reg(client, AR0543_16BIT,
				AR0543_VERTICAL_OUTPUT_SIZE_H, &value);
	if (ret)
		return ret;
	buf.output_height = value;

	memcpy(&info->data, &buf, sizeof(buf));

	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ar0543_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = ar0543_read_reg(client, AR0543_16BIT,
			       AR0543_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

static int ar0543_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ar0543_write_reg(client, AR0543_16BIT, 0x3070, value);
}

static int ar0543_v_flip(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (value > 1)
		return -EINVAL;

	ret = ar0543_write_reg_array(client, ar0543_param_hold);
	if (ret)
		return ret;
	ret = ar0543_rmw_reg(client, AR0543_16BIT & ~AR0543_RMW,
			       0x3040, 0x8000, value);
	if (ret)
		return ret;
	return ar0543_write_reg_array(client, ar0543_param_update);
}


static int ar0543_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (value > AR0543_VCM_SLEW_STEP_MAX)
		return -EINVAL;

	return ar0543_rmw_reg(client, AR0543_16BIT, AR0543_VCM_SLEW_STEP,
				AR0543_VCM_SLEW_STEP_MASK, value);
}

static int ar0543_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Max 16 bits */
	if (value > AR0543_VCM_SLEW_TIME_MAX)
		return -EINVAL;

	return ar0543_write_reg(client, AR0543_16BIT, AR0543_VCM_SLEW_TIME,
				 value);
}

static int ar0543_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (AR0543_FOCAL_LENGTH_NUM << 16) | AR0543_FOCAL_LENGTH_DEM;
	return 0;
}

static int ar0543_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ar0543*/
	*val = (AR0543_F_NUMBER_DEFAULT_NUM << 16) | AR0543_F_NUMBER_DEM;
	return 0;
}

static int ar0543_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (AR0543_F_NUMBER_DEFAULT_NUM << 24) |
		(AR0543_F_NUMBER_DEM << 16) |
		(AR0543_F_NUMBER_DEFAULT_NUM << 8) | AR0543_F_NUMBER_DEM;
	return 0;
}

static int ar0543_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	*val = ar0543_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int ar0543_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	*val = ar0543_res[dev->fmt_idx].bin_factor_y;

	return 0;
}

static struct ar0543_control ar0543_controls[] = {
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
		.query = ar0543_q_exposure,
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
		.tweak = ar0543_test_pattern,
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
		.tweak = ar0543_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = VCM_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_t_focus_abs,
		//.query = ar0543_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = AR0543_MAX_FOCUS_NEG,
			.maximum = AR0543_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_t_focus_rel,
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
		.query = ar0543_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = AR0543_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = AR0543_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = AR0543_FOCAL_LENGTH_DEFAULT,
			.maximum = AR0543_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = AR0543_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ar0543_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = AR0543_F_NUMBER_DEFAULT,
			.maximum = AR0543_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = AR0543_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ar0543_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = AR0543_F_NUMBER_RANGE,
			.maximum =  AR0543_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = AR0543_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ar0543_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = AR0543_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ar0543_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = AR0543_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ar0543_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ar0543_controls))

static struct ar0543_control *ar0543_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ar0543_controls[i].qc.id == id)
			return &ar0543_controls[i];
	return NULL;
}

static int ar0543_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct ar0543_control *ctrl = ar0543_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* ar0543 control set/get */
static int ar0543_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct ar0543_control *s_ctrl;
	int ret;
	if (!ctrl)
		return -EINVAL;

	s_ctrl = ar0543_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ar0543_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct ar0543_control *octrl = ar0543_find_control(ctrl->id);
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
static int distance(struct ar0543_resolution const *res, const u32 w,
		    const u32 h, const s32 m)
{
	u32 w_ratio = ((res->width<<13)/w);
	u32 h_ratio = ((res->height<<13)/h);
	s32 match   = abs(((w_ratio<<13)/h_ratio) - ((s32)8192));

	if ((w_ratio < (s32)8192) || (h_ratio < (s32)8192)  || (match > m))
		return -1;

	return w_ratio + h_ratio;
}


/*
 * Tune this value so that the DVS resolutions get selected properly,
 * but make sure 16:9 does not match 4:3
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 140

/*
 * Returns the nearest higher resolution index.
 * @w: width
 * @h: height
 * matching is done based on enveloping resolution and
 * aspect ratio. If the aspect ratio cannot be matched
 * to any index, the search is done again using envelopel
 * matching only. if no match can be found again, -1 is
 * returned.
 */
static int nearest_resolution_index(int w, int h)
{
	int i, j;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ar0543_resolution *tmp_res = NULL;
	s32 m = LARGEST_ALLOWED_RATIO_MISMATCH;

	for (j = 0; j < 2; ++j) {
		for (i = 0; i < N_RES; i++) {
			tmp_res = &ar0543_res[i];
			dist = distance(tmp_res, w, h, m);
			if (dist == -1)
				continue;
			if (dist < min_dist) {
				min_dist = dist;
				idx = i;
			}
		}
		if (idx != -1)
			break;
		m = LONG_MAX;
	}
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ar0543_res[i].width)
			continue;
		if (h != ar0543_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int __ar0543_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;

	if (fmt->width > AR0543_RES_WIDTH_MAX ||
	    fmt->height > AR0543_RES_HEIGHT_MAX) {
		fmt->width = AR0543_RES_WIDTH_MAX;
		fmt->height = AR0543_RES_HEIGHT_MAX;
	}

	/*
	 * We still need to find a matching resolution from the table
	 * even when width and height is larger than MAX
	 */
	{
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 * resolutions. If it fails, it means the requested
		 * resolution is higher than we can support.
		 * Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = ar0543_res[idx].width;
		fmt->height = ar0543_res[idx].height;
	}

	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ar0543_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ar0543_try_mbus_fmt(sd, fmt);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ar0543_get_mbus_format_code(struct v4l2_subdev *sd)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	if (!dev->power)
		return -EIO;

	return V4L2_MBUS_FMT_SGRBG10_1X10;
}

static int ar0543_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	printk("%s: s \n",__FUNCTION__);
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	const struct ar0543_reg *ar0543_def_reg;
	struct camera_mipi_info *ar0543_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ar0543_info = v4l2_get_subdev_hostdata(sd);
	if (ar0543_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	printk("%s: before __ar0543_try_mbus_fmt(%dx%d)\n",__FUNCTION__, fmt->width, fmt->height);
	ret = __ar0543_try_mbus_fmt(sd, fmt);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}
	printk("%s: after __ar0543_try_mbus_fmt\n",__FUNCTION__);
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);
	printk("%s: after __get_resolution_index width:%d  height:%d \n",__FUNCTION__, fmt->width, fmt->height);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	ar0543_def_reg = ar0543_res[dev->fmt_idx].regs;

	printk("%s: before ar0543_write_reg_array %s\n",__FUNCTION__, ar0543_res[dev->fmt_idx].desc);
	ret = ar0543_write_reg_array(client, ar0543_def_reg);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}
    printk("%s: before ar0543_get_mbus_format_code\n",__FUNCTION__);
	fmt->code = ar0543_get_mbus_format_code(sd);
	if (fmt->code < 0) {
		mutex_unlock(&dev->input_lock);
		return fmt->code;
	}
	dev->fps = ar0543_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ar0543_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ar0543_res[dev->fmt_idx].lines_per_frame;
	dev->coarse_itg = 0;
	dev->fine_itg = 0;
	dev->gain = 0;

	ret = ar0543_get_intg_factor(client, ar0543_info, ar0543_def_reg);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}
        printk("%s: e\n",__FUNCTION__);
	return 0;
}

static int ar0543_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = ar0543_res[dev->fmt_idx].width;
	fmt->height = ar0543_res[dev->fmt_idx].height;
	fmt->code = ar0543_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return fmt->code < 0 ? fmt->code : 0;
}

static int ar0543_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 reg;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip model and revision IDs */
	if (ar0543_read_reg(client, AR0543_16BIT, AR0543_SC_CMMN_CHIP_ID,
				id)) {
		v4l2_err(client, "Reading sensor_id error.\n");
		return -ENODEV;
	}

	if (*id != AR0543_ID && *id != AR0543_ID2) {
        ATD_ar0543_status = 0;//jimmy add
		v4l2_err(client,
			"sensor ID error, sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	else {
		printk("Main sensor ID = 0x%x\n", *id);
		ATD_ar0543_status = 1;//jimmy add
	}
	if (ar0543_read_reg(client, AR0543_8BIT, AR0543_SC_CMMN_REV_ID,
				&reg)) {
		v4l2_err(client, "Reading sensor_rev_id error.\n");
		return -ENODEV;
	}
	*revision = (u8)reg;

	return 0;
}

/*
 * ar0543 stream on/off
 */
static int ar0543_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (enable) {
		if (!dev->keeps_focus_pos) {
			struct ar0543_reg ar0543_stream_enable[] = {
				ar0543_streaming[0],
				{AR0543_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
				INIT_VCM_CONTROL,
				{AR0543_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
				{AR0543_TOK_DELAY, {0}, 60},
				{AR0543_TOK_TERM, {0}, 0}
			};

			ar0543_stream_enable[1].val = dev->focus + 1;
			ar0543_stream_enable[3].val = dev->focus;

			ret = ar0543_write_reg_array(client, ar0543_stream_enable);
		} else {
			ret = ar0543_write_reg_array(client, ar0543_streaming);
		}

		if (ret != 0) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "write_reg_array err\n");
			return ret;
		}
		dev->streaming = 1;
	} else {

		ret = ar0543_write_reg_array(client, ar0543_soft_standby);
		if (ret != 0) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "write_reg_array err\n");
			return ret;
		}
		dev->streaming = 0;
	}

	/* restore settings */
//Do not restore settings here, as we do not set parameters when switching resolution in video recording
//	ar0543_res = ar0543_res_preview;
//	N_RES = N_RES_PREVIEW;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/*
 * ar0543 enum frame size, frame intervals
 */
static int ar0543_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ar0543_res[index].width;
	fsize->discrete.height = ar0543_res[index].height;
	fsize->reserved[0] = ar0543_res[index].used;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);

	/*
	 * Since the isp will donwscale the resolution to the right size,
	 * find the nearest one that will allow the isp to do so important
	 * to ensure that the resolution requested is padded correctly by
	 * the requester, which is the atomisp driver in this case.
	 */
	index = nearest_resolution_index(fival->width, fival->height);

	if (-1 == index) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
/*	fival->width = ar0543_res[index].width;
	fival->height = ar0543_res[index].height; */
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ar0543_res[index].fps;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	mutex_lock(&dev->input_lock);
	*code = ar0543_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return *code < 0 ? *code : 0;
}

static int ar0543_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	void *otp_data;
	void *fuseid;

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "ar0543 platform init err\n");
			return ret;
		}
	}

	/*
	 * The initial state of physical power is unknown
	 * so first power down it to make it to a known
	 * state, and then start the power up sequence
	 */
	power_down(sd);
	msleep(20);

	ret = __ar0543_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "ar0543 power-up err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ar0543_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ar0543_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* Read sensor's OTP data */
	otp_data = ar0543_otp_read(sd);
	if (!IS_ERR(otp_data))
		dev->otp_data = otp_data;

	fuseid = ar0543_fuseid_read(sd);
	if (!IS_ERR(fuseid))
		dev->fuseid = fuseid;

	/* power off sensor */
	ret = __ar0543_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(client, "ar0543 power-down err.\n");
		return ret;
	}

	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__ar0543_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
ar0543_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	if (code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = ar0543_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return code->code < 0 ? code->code : 0;
}

static int
ar0543_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fse->min_width = ar0543_res[index].width;
	fse->min_height = ar0543_res[index].height;
	fse->max_width = ar0543_res[index].width;
	fse->max_height = ar0543_res[index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0543_get_pad_format(struct ar0543_device *sensor,
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
ar0543_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ar0543_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
ar0543_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ar0543_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
ar0543_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	mutex_lock(&dev->input_lock);

	if (dev->streaming) {
		mutex_unlock(&dev->input_lock);
		return -EBUSY;
	}

	dev->run_mode = param->parm.capture.capturemode;

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ar0543_res = ar0543_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ar0543_res = ar0543_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ar0543_res = ar0543_res_preview;
		N_RES = N_RES_PREVIEW;
	}

	/* Reset sensor mode */
	dev->fmt_idx = 0;
	dev->fps = ar0543_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ar0543_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ar0543_res[dev->fmt_idx].lines_per_frame;
	dev->coarse_itg = 0;
	dev->fine_itg = 0;
	dev->gain = 0;
	printk("%s run_mode=%x desc=%s\n", __func__, dev->run_mode, ar0543_res[dev->fmt_idx].desc);

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
ar0543_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;

	mutex_lock(&dev->input_lock);

	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		mutex_unlock(&dev->input_lock);
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->coarse_itg > dev->lines_per_frame) {
		if (dev->coarse_itg == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal coarse_itg:0x%x\n",
				  __func__, dev->coarse_itg);
			lines_per_frame = dev->coarse_itg;
		} else
			lines_per_frame = dev->coarse_itg + 1;
	} else
		lines_per_frame = dev->lines_per_frame;

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = AR0543_MCLK * 1000000;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*frames = ar0543_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}
static const struct v4l2_subdev_video_ops ar0543_video_ops = {
	.s_stream = ar0543_s_stream,
	.enum_framesizes = ar0543_enum_framesizes,
	.enum_frameintervals = ar0543_enum_frameintervals,
	.enum_mbus_fmt = ar0543_enum_mbus_fmt,
	.try_mbus_fmt = ar0543_try_mbus_fmt,
	.g_mbus_fmt = ar0543_g_mbus_fmt,
	.s_mbus_fmt = ar0543_s_mbus_fmt,
	.s_parm = ar0543_s_parm,
	.g_frame_interval = ar0543_g_frame_interval,
};

static struct v4l2_subdev_sensor_ops ar0543_sensor_ops = {
	.g_skip_frames	= ar0543_g_skip_frames,
};

static const struct v4l2_subdev_core_ops ar0543_core_ops = {
	.g_chip_ident = ar0543_g_chip_ident,
	.queryctrl = ar0543_queryctrl,
	.g_ctrl = ar0543_g_ctrl,
	.s_ctrl = ar0543_s_ctrl,
	.s_power = ar0543_s_power,
	.ioctl = ar0543_ioctl,
	.init = ar0543_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops ar0543_pad_ops = {
	.enum_mbus_code = ar0543_enum_mbus_code,
	.enum_frame_size = ar0543_enum_frame_size,
	.get_fmt = ar0543_get_pad_format,
	.set_fmt = ar0543_set_pad_format,
};

static const struct v4l2_subdev_ops ar0543_ops = {
	.core = &ar0543_core_ops,
	.video = &ar0543_video_ops,
	.pad = &ar0543_pad_ops,
	.sensor = &ar0543_sensor_ops,
};

static const struct media_entity_operations ar0543_entity_ops = {
	.link_setup = NULL,
};

static int ar0543_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0543_device *dev = to_ar0543_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev->otp_data);
	kfree(dev->fuseid);
	kfree(dev);

	return 0;
}

static int ar0543_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ar0543_device *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}
        

        //Add for ATD read camera status+++
	dev->sensor_i2c_attribute.attrs = ar0543_attributes;

	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ar0543_ops);

	if (client->dev.platform_data) {
		ret = ar0543_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &ar0543_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ar0543_remove(client);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id ar0543_id[] = {
	{AR0543_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ar0543_id);

static struct i2c_driver ar0543_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AR0543_NAME,
	},
	.probe = ar0543_probe,
	.remove = ar0543_remove,
	.id_table = ar0543_id,
};

static __init int init_ar0543(void)
{
	return i2c_add_driver(&ar0543_driver);
}

static __exit void exit_ar0543(void)
{
	i2c_del_driver(&ar0543_driver);
}

module_init(init_ar0543);
module_exit(exit_ar0543);

MODULE_DESCRIPTION("A low-level driver for Aptina AR0543 sensors");
MODULE_LICENSE("GPL");
