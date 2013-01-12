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
#include "atomisp_common.h"
#include "atomisp_internal.h"

static const unsigned int isp_subdev_input_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};

static const unsigned int isp_subdev_preview_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_NV21,
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

static const unsigned int isp_subdev_capture_output_fmts[] = {
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
	if (IS_MRFLD || sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int isp_subdev_unsubscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
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

	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_preview_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_preview_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_vf_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_vf_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_capture_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_capture_output_fmts[code->index];
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int isp_subdev_validate_rect(struct v4l2_subdev *sd, uint32_t pad,
				    uint32_t target)
{
	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return 0;
		}
		break;
	}

	return -EINVAL;
}

struct v4l2_rect *atomisp_subdev_get_rect(struct v4l2_subdev *sd,
					  struct v4l2_subdev_fh *fh,
					  uint32_t which, uint32_t pad,
					  uint32_t target)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);

	if (which == V4L2_SUBDEV_FORMAT_TRY) {
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return v4l2_subdev_get_try_crop(fh, pad);
		}
	}

	switch (target) {
	case V4L2_SEL_TGT_CROP:
		return &isp_sd->fmt[pad].crop;
	}

	BUG();

	return NULL;
}

struct v4l2_mbus_framefmt
*atomisp_subdev_get_mfmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			 uint32_t which, uint32_t pad)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &isp_sd->fmt[pad].fmt;
}

static void isp_subdev_propagate(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 uint32_t which, uint32_t pad, uint32_t target)
{
	struct v4l2_mbus_framefmt *f =
		atomisp_subdev_get_mfmt(sd, fh, which, pad);
	struct v4l2_rect *crop =
		atomisp_subdev_get_rect(sd, fh, which, pad, V4L2_SEL_TGT_CROP);

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			crop->width = f->width;
			crop->height = f->height;
			break;
		}
		break;
	}
}

static int isp_subdev_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_selection *sel)
{
	int rval = isp_subdev_validate_rect(sd, sel->pad, sel->target);
	if (rval)
		return rval;

	sel->r = *atomisp_subdev_get_rect(sd, fh, sel->which, sel->pad,
					   sel->target);

	return 0;
}

int atomisp_subdev_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh, uint32_t which,
				 uint32_t pad, uint32_t target,
				 struct v4l2_rect *r)
{
	struct v4l2_rect *__r = atomisp_subdev_get_rect(sd, fh, which, pad,
							target);
	*__r = *r;

	return 0;
}

static int isp_subdev_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_selection *sel)
{
	int rval = isp_subdev_validate_rect(sd, sel->pad, sel->target);
	if (rval)
		return rval;

	return atomisp_subdev_set_selection(sd, fh, sel->which, sel->pad,
					    sel->target, &sel->r);
}

static int atomisp_get_sensor_bin_factor(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;
	int hbin, vbin;
	int ret;

	memset(&ctrl, 0, sizeof(ctrl));

	ctrl.id = V4L2_CID_BIN_FACTOR_HORZ;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
			       g_ctrl, &ctrl);
	hbin = ctrl.value;
	ctrl.id = V4L2_CID_BIN_FACTOR_VERT;
	ret |= v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
				g_ctrl, &ctrl);
	vbin = ctrl.value;

	/*
	 * ISP needs to know binning factor from sensor.
	 * In case horizontal and vertical sensor's binning factors
	 * are different or sensor does not support binning factor CID,
	 * ISP will apply default 0 value.
	 */
	if (ret || hbin != vbin)
		hbin = 0;

	return hbin;
}

int atomisp_subdev_set_mfmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    uint32_t which, uint32_t pad,
			    struct v4l2_mbus_framefmt *ffmt)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	struct v4l2_mbus_framefmt *__ffmt =
		atomisp_subdev_get_mfmt(sd, fh, which, pad);

	*__ffmt = *ffmt;

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		isp_subdev_propagate(sd, fh, which, pad,
				     V4L2_SEL_TGT_CROP);

		if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			sh_css_input_set_resolution(ffmt->width, ffmt->height);
			sh_css_input_set_binning_factor(
				atomisp_get_sensor_bin_factor(isp));
		}

		break;
	}

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
	fmt->format = *atomisp_subdev_get_mfmt(sd, fh, fmt->which, fmt->pad);

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
	return atomisp_subdev_set_mfmt(sd, fh, fmt->which, fmt->pad,
				       &fmt->format);
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops isp_subdev_v4l2_core_ops = {
	 .ioctl = isp_subdev_ioctl,
	 .s_power = isp_subdev_set_power,
	 .subscribe_event = isp_subdev_subscribe_event,
	 .unsubscribe_event = isp_subdev_unsubscribe_event,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops isp_subdev_v4l2_pad_ops = {
	 .enum_mbus_code = isp_subdev_enum_mbus_code,
	 .get_fmt = isp_subdev_get_format,
	 .set_fmt = isp_subdev_set_format,
	 .get_selection = isp_subdev_get_selection,
	 .set_selection = isp_subdev_set_selection,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops isp_subdev_v4l2_ops = {
	 .core = &isp_subdev_v4l2_core_ops,
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
	unsigned int i;

	switch (local->index | media_entity_type(remote->entity)) {
	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		/* Read from the sensor CSI2-ports. */
		if (!(flags & MEDIA_LNK_FL_ENABLED)) {
			isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
			break;
		}

		if (isp_sd->input != ATOMISP_SUBDEV_INPUT_NONE)
			return -EBUSY;

		for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++) {
			if (remote->entity != &isp->csi2_port[i].subdev.entity)
				continue;

			isp_sd->input = ATOMISP_SUBDEV_INPUT_CSI2_PORT1 + i;
			return 0;
		}

		return -EINVAL;

	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_DEVNODE:
		/* read from memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (isp_sd->input >= ATOMISP_SUBDEV_INPUT_CSI2_PORT1 &&
				isp_sd->input < (ATOMISP_SUBDEV_INPUT_CSI2_PORT1
						+ ATOMISP_CAMERA_NR_PORTS))
				return -EBUSY;
			isp_sd->input = ATOMISP_SUBDEV_INPUT_MEMORY;
		} else {
			if (isp_sd->input == ATOMISP_SUBDEV_INPUT_MEMORY)
				isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
		}
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_VF | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE | MEDIA_ENT_T_DEVNODE:
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

static int s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = &s_ctrl,
};

static const struct v4l2_ctrl_config ctrl_fmt_auto = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_FMT_AUTO,
	.name = "Automatic format guessing",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.def = 1,
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

	pads[ATOMISP_SUBDEV_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_VF].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].flags = MEDIA_PAD_FL_SOURCE;

	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SINK].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_VF].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;

	me->ops = &isp_subdev_media_ops;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(me, ATOMISP_SUBDEV_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	isp_subdev->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	isp_subdev->video_in.isp = isp_subdev->isp;
	spin_lock_init(&isp_subdev->video_in.irq_lock);

	isp_subdev->video_out_preview.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_preview.isp = isp_subdev->isp;
	isp_subdev->video_out_preview.pipe_type = ATOMISP_PIPE_PREVIEW;
	spin_lock_init(&isp_subdev->video_out_preview.irq_lock);

	isp_subdev->video_out_vf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_vf.isp = isp_subdev->isp;
	isp_subdev->video_out_vf.pipe_type = ATOMISP_PIPE_VIEWFINDER;
	spin_lock_init(&isp_subdev->video_out_vf.irq_lock);

	isp_subdev->video_out_capture.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_capture.isp = isp_subdev->isp;
	isp_subdev->video_out_capture.pipe_type = ATOMISP_PIPE_CAPTURE;
	spin_lock_init(&isp_subdev->video_out_capture.irq_lock);

	ret = atomisp_video_init(&isp_subdev->video_in, "MEMORY");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_capture, "CAPTURE");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_vf, "VIEWFINDER");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_preview, "PREVIEW");
	if (ret < 0)
		return ret;

	/* Connect the isp subdev to the video node. */
	ret = media_entity_create_link(&isp_subdev->video_in.vdev.entity,
		0, &isp_subdev->subdev.entity, ATOMISP_SUBDEV_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW,
		&isp_subdev->video_out_preview.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_VF,
		&isp_subdev->video_out_vf.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
		&isp_subdev->video_out_capture.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_ctrl_handler_init(&isp_subdev->ctrl_handler, 1);
	if (ret)
		return ret;

	isp_subdev->fmt_auto = v4l2_ctrl_new_custom(&isp_subdev->ctrl_handler,
						    &ctrl_fmt_auto, NULL);

	/* Make controls visible on subdev as well. */
	isp_subdev->subdev.ctrl_handler = &isp_subdev->ctrl_handler;

	return isp_subdev->ctrl_handler.error;
}

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *isp_subdev)
{
	v4l2_ctrl_handler_free(&isp_subdev->ctrl_handler);

	media_entity_cleanup(&isp_subdev->subdev.entity);

	v4l2_device_unregister_subdev(&isp_subdev->subdev);
	atomisp_video_unregister(&isp_subdev->video_in);
	atomisp_video_unregister(&isp_subdev->video_out_preview);
	atomisp_video_unregister(&isp_subdev->video_out_vf);
	atomisp_video_unregister(&isp_subdev->video_out_capture);
}

int atomisp_subdev_register_entities(struct atomisp_sub_device *isp_subdev,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &isp_subdev->subdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_capture, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_vf, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_preview, vdev);
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

