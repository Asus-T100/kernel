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
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include "atomisp_internal.h"

static const unsigned int isp_subdev_input_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};

static const unsigned int isp_subdev_vf_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_NV21,
};

static const unsigned int isp_subdev_ss_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_NV21,
};

static const unsigned int isp_subdev_mo_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565, nv11, yuv422, nv16, yv16, yuy2 */
	/* rgb565, rgb888 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_YUV422P,
	V4L2_PIX_FMT_YUV444,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_NV16,
	V4L2_PIX_FMT_NV61,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_RGB32
};

/*
 * V4L2 subdev operations
 */

/*
 * isp_subdev_get_ctrl - V4L2 control get handler
 * @sd: ISP V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int isp_subdev_get_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * isp_subdev_set_ctrl - V4L2 control set handler
 * @sd: ISP CCDC V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int isp_subdev_set_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * isp_subdev_ioctl - CCDC module private ioctl's
 * @sd: ISP V4L2 subdevice
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Return 0 on success or a negative error code otherwise.
 */
static long isp_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int ret = 0;

	return ret;
}

/*
 * isp_subdev_set_power - Power on/off the CCDC module
 * @sd: ISP V4L2 subdevice
 * @on: power on/off
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int isp_subdev_set_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int isp_subdev_subscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{

	/* TBD
	return v4l2_event_subscribe(fh, sub);
	*/
	return 0;
}

static int isp_subdev_unsubscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

/*
* isp_subdev_set_stream - Enable/Disable streaming on the isp sub module
* @sd: ISP V4L2 subdevice
* @enable: Enable/disable stream
*/
static int isp_subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	/* handle continuous, single shot or stopped state */
	return ret;
}

static struct v4l2_mbus_framefmt *
__isp_subdev_get_format(struct atomisp_sub_device *isp_subdev,
	struct v4l2_subdev_fh *fh, unsigned int pad,
	enum v4l2_subdev_format_whence which)
{

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		if (pad == ATOMISP_SUBDEV_PAD_SINK ||
		    pad == ATOMISP_SUBDEV_PAD_SOURCE_VF ||
		    pad == ATOMISP_SUBDEV_PAD_SOURCE_SS ||
		    pad == ATOMISP_SUBDEV_PAD_SOURCE_MO)
			return &isp_subdev->formats[pad];
		else
			return NULL;
	default:
		return NULL;
	}
}

/*
 * isp_subdev_try_format - Try video format on a pad
 * @isp_subdev: ISP v4l2 sub device
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 */
static void
isp_subdev_try_format(struct atomisp_sub_device *isp_subdev,
	struct v4l2_subdev_fh *fh,
	unsigned int pad, struct v4l2_mbus_framefmt *fmt,
	enum v4l2_subdev_format_whence which)
{
	struct v4l2_mbus_framefmt *format;
	unsigned int width = fmt->width;
	unsigned int height = fmt->height;
	enum v4l2_mbus_pixelcode pixelcode;
	unsigned int i;

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		if (isp_subdev->input == ATOMISP_SUBDEV_INPUT_MEMORY) {
			fmt->width = clamp_t(u32, fmt->width,
				ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
			fmt->height = clamp_t(u32, fmt->height,
				ATOM_ISP_MIN_HEIGHT, ATOM_ISP_MAX_HEIGHT);
		}

		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		for (i = 0; i < ARRAY_SIZE(isp_subdev_input_fmts); i++) {
			if (fmt->code == isp_subdev_input_fmts[i])
				break;
		}

		/* If not found, use SGRBG10 as default */
		if (i >= ARRAY_SIZE(isp_subdev_input_fmts))
			fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

		/* Clamp the input size. */
		fmt->width = clamp_t(u32, width, 32, 4096);
		fmt->height = clamp_t(u32, height, 32, 4096);

		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
	case ATOMISP_SUBDEV_PAD_SOURCE_SS:
		pixelcode = fmt->code;
		format = __isp_subdev_get_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		switch (pixelcode) {
		/* yuv420, nv12, yv12, nv21, rgb565 */
		case V4L2_MBUS_FMT_YUYV8_1X16:
		case V4L2_MBUS_FMT_UYVY8_1X16:
		case V4L2_MBUS_FMT_RGB565_2X8_LE:
		case V4L2_MBUS_FMT_YUYV8_1_5X8:
			fmt->code = pixelcode;
			break;
		default:
			fmt->code = V4L2_MBUS_FMT_YUYV8_1X16;
			break;
		}

		/* The data formatter truncates the number of horizontal output
		* pixels to a multiple of 16. To avoid clipping data, allow
		* callers to request an output size bigger than the input size
		* up to the nearest multiple of 16.
		*/
		fmt->width = clamp_t(u32, width, 256, 1920);
		fmt->height = clamp_t(u32, height, 0, 1080);
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_MO:
		pixelcode = fmt->code;
		format = __isp_subdev_get_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));
		switch (pixelcode) {
		/* yuv420, nv12, yv12, nv21, rgb565, nv11, yuv422 */
		/* nv16, yv16, yuy2 */
		/* rgb565, rgb888 */
		/* TODO: check these formats */
		case V4L2_MBUS_FMT_YUYV8_1X16:
		case V4L2_MBUS_FMT_UYVY8_1X16:
		case V4L2_MBUS_FMT_RGB565_2X8_LE:
		case V4L2_MBUS_FMT_YUYV8_1_5X8:
			fmt->code = pixelcode;
			break;
		default:
			fmt->code = V4L2_MBUS_FMT_YUYV8_1X16;
			break;
		}

		/* The number of lines that can be clocked out from the video
		* port output must be at least one line less than the number
		* of input lines.
		*/
		fmt->width = clamp_t(u32, width, 256, 4608);
		fmt->height = clamp_t(u32, height, 0, fmt->height - 1);
		break;
	}

	/* Data is written to memory unpacked, each 10-bit pixel is stored on
	* 2 bytes.
	*/
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
}

/*
 * isp_subdev_enum_mbus_code - Handle pixel format enumeration
 * @sd: pointer to v4l2 subdev structure
 * @fh : V4L2 subdev file handle
 * @code: pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int isp_subdev_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		if (code->index >= ARRAY_SIZE(isp_subdev_input_fmts))
			return -EINVAL;

		code->code = isp_subdev_input_fmts[code->index];
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_vf_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_vf_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_SS:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_ss_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_vf_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_MO:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_mo_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_mo_output_fmts[code->index];
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int isp_subdev_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	struct atomisp_sub_device *isp_subdev = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	isp_subdev_try_format(isp_subdev, fh, fse->pad,
		&format, V4L2_SUBDEV_FORMAT_TRY);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	isp_subdev_try_format(isp_subdev, fh, fse->pad,
		&format, V4L2_SUBDEV_FORMAT_TRY);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * isp_subdev_get_format - Retrieve the video format on a pad
 * @sd : ISP V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int isp_subdev_get_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	struct atomisp_sub_device *isp_subdev = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __isp_subdev_get_format(isp_subdev, fh, fmt->pad, fmt->which);
	if (format == NULL) {
		v4l2_err(&atomisp_dev, "no format available\n");
		return -EINVAL;
	}

	fmt->format = *format;
	return 0;
}

/*
 * isp_subdev_set_format - Set the video format on a pad
 * @sd : ISP subdev V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int isp_subdev_set_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	struct atomisp_sub_device *isp_subdev = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __isp_subdev_get_format(isp_subdev, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	isp_subdev_try_format(isp_subdev, fh, fmt->pad,
		&fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == ATOMISP_SUBDEV_PAD_SINK) {
		format = __isp_subdev_get_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SOURCE_VF, fmt->which);
		*format = fmt->format;
		isp_subdev_try_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SOURCE_VF, format, fmt->which);

		format = __isp_subdev_get_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SOURCE_MO, fmt->which);
		*format = fmt->format;
		isp_subdev_try_format(isp_subdev, fh,
			ATOMISP_SUBDEV_PAD_SOURCE_MO, format, fmt->which);
	}

	return 0;
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops isp_subdev_v4l2_core_ops = {
	 .g_ctrl = isp_subdev_get_ctrl,
	 .s_ctrl = isp_subdev_set_ctrl,
	 .ioctl = isp_subdev_ioctl,
	 .s_power = isp_subdev_set_power,
	 .subscribe_event = isp_subdev_subscribe_event,
	 .unsubscribe_event = isp_subdev_unsubscribe_event,
};

/* V4L2 subdev video operations */
static const struct v4l2_subdev_video_ops isp_subdev_v4l2_video_ops = {
	 .s_stream = isp_subdev_set_stream,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops isp_subdev_v4l2_pad_ops = {
	 .enum_mbus_code = isp_subdev_enum_mbus_code,
	 .enum_frame_size = isp_subdev_enum_frame_size,
	 .get_fmt = isp_subdev_get_format,
	 .set_fmt = isp_subdev_set_format,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops isp_subdev_v4l2_ops = {
	 .core = &isp_subdev_v4l2_core_ops,
	 .video = &isp_subdev_v4l2_video_ops,
	 .pad = &isp_subdev_v4l2_pad_ops,
};

static void isp_subdev_init_params(struct atomisp_sub_device *isp_subdev)
{
	/* parameters initialization */
}

/*
* isp_subdev_link_setup - Setup isp subdev connections
* @entity: ispsubdev media entity
* @local: Pad at the local end of the link
* @remote: Pad at the remote end of the link
* @flags: Link flags
*
* return -EINVAL or zero on success
*/
static int isp_subdev_link_setup(struct media_entity *entity,
	const struct media_pad *local,
	const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;

	switch (local->index | media_entity_type(remote->entity)) {
	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		/* Read from the sensor CSI2-4p or CSI2-1p. */
		if (!(flags & MEDIA_LNK_FL_ENABLED)) {
			isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
			break;
		}

		if (isp_sd->input != ATOMISP_SUBDEV_INPUT_NONE)
			return -EBUSY;

		if (remote->entity == &isp->csi2_4p.subdev.entity)
			isp_sd->input = ATOMISP_SUBDEV_INPUT_CSI2_4P;
		else if (remote->entity == &isp->csi2_1p.subdev.entity)
			isp_sd->input = ATOMISP_SUBDEV_INPUT_CSI2_1P;
		else
			isp_sd->input = ATOMISP_SUBDEV_INPUT_CSI2_4P;

		break;

	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_DEVNODE:
		/* read from memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (isp_sd->input == ATOMISP_SUBDEV_INPUT_CSI2_4P
			|| isp_sd->input == ATOMISP_SUBDEV_INPUT_CSI2_1P)
				return -EBUSY;
			isp_sd->input = ATOMISP_SUBDEV_INPUT_MEMORY;
		} else {
			if (isp_sd->input == ATOMISP_SUBDEV_INPUT_MEMORY)
				isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
		}
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_VF | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_SS | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_MO | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations isp_subdev_media_ops = {
	 .link_setup = isp_subdev_link_setup,
/*	 .set_power = v4l2_subdev_set_power,	*/
};

/*
 * isp_subdev_init_entities - Initialize V4L2 subdev and media entity
 * @isp_subdev: ISP CCDC module
 *
 * Return 0 on success and a negative error code on failure.
 */
static int isp_subdev_init_entities(struct atomisp_sub_device *isp_subdev)
{
	struct v4l2_subdev *sd = &isp_subdev->subdev;
	struct media_pad *pads = isp_subdev->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	isp_subdev->input = ATOMISP_SUBDEV_INPUT_NONE;

	v4l2_subdev_init(sd, &isp_subdev_v4l2_ops);
	strlcpy(sd->name, "ATOM ISP SUBDEV", sizeof(sd->name));
	v4l2_set_subdevdata(sd, isp_subdev);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->nevents = 16; /* TBD */

	pads[ATOMISP_SUBDEV_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_VF].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_SS].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_MO].flags = MEDIA_PAD_FL_SOURCE;

	isp_subdev->formats[ATOMISP_SUBDEV_PAD_SINK].code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->formats[ATOMISP_SUBDEV_PAD_SOURCE_VF].code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->formats[ATOMISP_SUBDEV_PAD_SOURCE_SS].code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->formats[ATOMISP_SUBDEV_PAD_SOURCE_MO].code =
		V4L2_MBUS_FMT_SBGGR10_1X10;

	me->ops = &isp_subdev_media_ops;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(me, ATOMISP_SUBDEV_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	isp_subdev->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	isp_subdev->video_in.isp = isp_subdev->isp;
	spin_lock_init(&isp_subdev->video_in.irq_lock);

	isp_subdev->video_out_vf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_vf.isp = isp_subdev->isp;
	isp_subdev->video_out_vf.pipe_type = ATOMISP_PIPE_VIEWFINDER;
	spin_lock_init(&isp_subdev->video_out_vf.irq_lock);

	isp_subdev->video_out_ss.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_ss.isp = isp_subdev->isp;
	isp_subdev->video_out_ss.pipe_type = ATOMISP_PIPE_SNAPSHOT;
	spin_lock_init(&isp_subdev->video_out_ss.irq_lock);

	isp_subdev->video_out_mo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_mo.isp = isp_subdev->isp;
	isp_subdev->video_out_mo.pipe_type = ATOMISP_PIPE_MASTEROUTPUT;
	spin_lock_init(&isp_subdev->video_out_mo.irq_lock);

	ret = atomisp_video_init(&isp_subdev->video_in, "MEMORY");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_mo, "MAINOUTPUT");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_ss, "SNAPSHOT");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_vf, "VIEWFINDER");
	if (ret < 0)
		return ret;

	/* Connect the isp subdev to the video node. */
	ret = media_entity_create_link(&isp_subdev->video_in.vdev.entity,
		0, &isp_subdev->subdev.entity, ATOMISP_SUBDEV_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_VF,
		&isp_subdev->video_out_vf.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_SS,
		&isp_subdev->video_out_ss.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_MO,
		&isp_subdev->video_out_mo.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *isp_subdev)
{
	media_entity_cleanup(&isp_subdev->subdev.entity);

	v4l2_device_unregister_subdev(&isp_subdev->subdev);
	atomisp_video_unregister(&isp_subdev->video_in);
	atomisp_video_unregister(&isp_subdev->video_out_vf);
	atomisp_video_unregister(&isp_subdev->video_out_ss);
	atomisp_video_unregister(&isp_subdev->video_out_mo);
}

int atomisp_subdev_register_entities(struct atomisp_sub_device *isp_subdev,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &isp_subdev->subdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_mo, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_ss, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_vf, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_in, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	atomisp_subdev_unregister_entities(isp_subdev);
	return ret;
}


/*
 * atomisp_subdev_init - ISP Subdevice  initialization.
 * @dev: Device pointer specific to the ATOM ISP.
 *
 * TODO: Get the initialisation values from platform data.
 *
 * Return 0 on success or a negative error code otherwise.
 */
int atomisp_subdev_init(struct atomisp_device *isp)
{
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev;
	int ret;

	spin_lock_init(&isp_subdev->lock);
	isp_subdev->isp = isp;
	isp_subdev_init_params(isp_subdev);
	ret = isp_subdev_init_entities(isp_subdev);
	if (ret < 0)
		atomisp_subdev_cleanup(isp);

	return ret;
}

void atomisp_subdev_cleanup(struct atomisp_device *isp)
{

}

