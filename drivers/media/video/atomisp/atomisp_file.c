/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
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

#include <css/sh_css.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include "atomisp_internal.h"
#include "atomisp_common.h"
#include "atomisp_subdev.h"
#include "atomisp_cmd.h"
#include "atomisp_file.h"

static int file_input_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;
	struct atomisp_video_pipe *out_pipe = &isp->isp_subdev.video_in;
	unsigned char *buf =
		   videobuf_to_vmalloc(out_pipe->outq.bufs[0]);
	int width = isp->input_format->out.width;
	int height = isp->input_format->out.height;
	unsigned short *data = (unsigned short *)buf;
	int i, j;

	/* extend RAW8 pixel type to unsigned short */
	if (out_pipe->out_fmt->depth == 8) {
		data = vmalloc(width*height*2);
		if (!data) {
			v4l2_err(&atomisp_dev,
				"Failed to allocate memory for file input\n");
			return -ENOMEM;
		}

		for (i = 0; i < height; i++)
			for (j = 0; j < width; j++)
				*data++ = (unsigned short)*buf++;
		data -= width*height;
	}

	while (!sh_css_isp_has_started())
		/* we are facing ISP timeout, if removed this delay */
		mdelay(1);

	sh_css_send_input_frame(data, width, height);

	if (out_pipe->out_fmt->depth == 8)
		vfree(data);
	return 0;
}

static int file_input_g_parm(struct v4l2_subdev *sd,
		struct v4l2_streamparm *param)
{
	/*to fake*/
	return 0;
}

static int file_input_s_parm(struct v4l2_subdev *sd,
		struct v4l2_streamparm *param)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	/*to fake*/
	return 0;
}

static int file_input_try_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;
	struct atomisp_video_pipe *out_pipe = &isp->isp_subdev.video_in;

	if (fmt == NULL)
		return -EINVAL;

	if ((fmt->width > out_pipe->out_fmt->width) ||
	    (fmt->height > out_pipe->out_fmt->height))
		return -EINVAL;

	fmt->width = out_pipe->out_fmt->width;
	fmt->height = out_pipe->out_fmt->height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int file_input_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;
	struct atomisp_video_pipe *out_pipe = &isp->isp_subdev.video_in;

	if (fmt == NULL)
		return -EINVAL;

	fmt->width = out_pipe->out_fmt->width;
	fmt->height = out_pipe->out_fmt->height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int file_input_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	/*to fake*/
	struct atomisp_file_device *file_dev = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = file_dev->isp;
	struct atomisp_video_pipe *out_pipe = &isp->isp_subdev.video_in;
	int ret;

	if (fmt == NULL)
		return -EINVAL;

	ret = file_input_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "file input try fmt failed\n");
		return ret;
	}

	fmt->width = out_pipe->out_fmt->width;
	fmt->height = out_pipe->out_fmt->height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	sh_css_input_set_mode(SH_CSS_INPUT_MODE_FIFO);
	return 0;
}

static int file_input_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *chip)
{
	struct atomisp_file_device *dev;
	dev = container_of(sd, struct atomisp_file_device, sd);

	if (!chip)
		return -EINVAL;

	return 0;
}

static int file_input_log_status(struct v4l2_subdev *sd)
{
	/*to fake*/
	return 0;
}

static int file_input_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/*to fake*/
	return -EINVAL;
}

static int file_input_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/*to fake*/
	return -EINVAL;
}

static int file_input_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/*to fake*/
	return 0;
}

static int file_input_s_power(struct v4l2_subdev *sd, int on)
{
	/* to fake */
	return 0;
}

static int file_input_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	/*to fake*/
	return 0;
}

static int file_input_enum_frame_ival(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	/*to fake*/
	return 0;
}

static const struct v4l2_subdev_video_ops file_input_video_ops = {
	.s_stream = file_input_s_stream,
	.g_parm = file_input_g_parm,
	.s_parm = file_input_s_parm,
	.enum_framesizes = file_input_enum_framesizes,
	.enum_frameintervals = file_input_enum_frameintervals,
	.enum_mbus_fmt = file_input_enum_mbus_fmt,
	.try_mbus_fmt = file_input_try_mbus_fmt,
	.g_mbus_fmt = file_input_g_mbus_fmt,
	.s_mbus_fmt = file_input_s_mbus_fmt,
};

static const struct v4l2_subdev_core_ops file_input_core_ops = {
	.g_chip_ident = file_input_g_chip_ident,
	.log_status = file_input_log_status,
	.queryctrl = file_input_queryctrl,
	.g_ctrl = file_input_g_ctrl,
	.s_ctrl = file_input_s_ctrl,
	.s_power = file_input_s_power,
};

static const struct v4l2_subdev_pad_ops file_input_pad_ops = {
	.enum_mbus_code = file_input_enum_mbus_code,
	.enum_frame_size = file_input_enum_frame_size,
	.enum_frame_interval = file_input_enum_frame_ival,
};

static const struct v4l2_subdev_ops file_input_ops = {
	.core = &file_input_core_ops,
	.video = &file_input_video_ops,
	.pad = &file_input_pad_ops,
};

static const struct media_entity_operations file_input_entity_ops = {
/*	.set_power = v4l2_subdev_set_power,	*/
};

void
atomisp_file_input_unregister_entities(struct atomisp_file_device *file_dev)
{
	media_entity_cleanup(&file_dev->sd.entity);
	v4l2_device_unregister_subdev(&file_dev->sd);
}

int atomisp_file_input_register_entities(struct atomisp_file_device *file_dev,
			struct v4l2_device *vdev)
{
	/* Register the subdev and video nodes. */
	return  v4l2_device_register_subdev(vdev, &file_dev->sd);
}

void atomisp_file_input_cleanup(struct atomisp_device *isp)
{
	return;
}

int atomisp_file_input_init(struct atomisp_device *isp)
{
	struct atomisp_file_device *file_dev = &isp->file_dev;
	struct v4l2_subdev *sd = &file_dev->sd;
	struct media_pad *pads = file_dev->pads;
	struct media_entity *me = &sd->entity;
	struct camera_mipi_info *file_input_info = NULL;
	int ret;

	file_dev->isp = isp;
	v4l2_subdev_init(sd, &file_input_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(sd->name, "file_input_subdev");
	v4l2_set_subdevdata(sd, file_dev);

	file_input_info = kzalloc(sizeof(*file_input_info), GFP_KERNEL);
	if (!file_input_info) {
		v4l2_err(&atomisp_dev,
			    "Failed to allocate memory for file input\n");
		return -ENOMEM;
	}

	file_input_info->port = ATOMISP_CAMERA_PORT_PRIMARY;
	file_input_info->input_format = SH_CSS_INPUT_FORMAT_RAW_10;
	file_input_info->raw_bayer_order = sh_css_bayer_order_bggr;
	v4l2_set_subdev_hostdata(sd, (void *)file_input_info);

	pads[0].flags = MEDIA_PAD_FL_SINK;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;

	ret = media_entity_init(me, 1, pads, 0);
	if (ret < 0)
		goto fail;
	return 0;
fail:
	kfree(file_input_info);
	atomisp_file_input_cleanup(isp);
	return ret;
}
