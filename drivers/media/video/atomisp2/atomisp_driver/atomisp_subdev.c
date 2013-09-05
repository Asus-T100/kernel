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
#include <asm/intel-mid.h>

#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_compat.h"
#include "atomisp_internal.h"

const struct atomisp_in_fmt_conv atomisp_in_fmt_conv[] = {
	{ V4L2_MBUS_FMT_SBGGR8_1X8, 8, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_BGGR },
	{ V4L2_MBUS_FMT_SGBRG8_1X8, 8, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_GBRG },
	{ V4L2_MBUS_FMT_SGRBG8_1X8, 8, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_GRBG },
	{ V4L2_MBUS_FMT_SRGGB8_1X8, 8, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_RGGB },
	{ V4L2_MBUS_FMT_SBGGR10_1X10, 10, 10, CSS_FORMAT_RAW_10, CSS_BAYER_ORDER_BGGR },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, 10, 10, CSS_FORMAT_RAW_10, CSS_BAYER_ORDER_GBRG },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, 10, 10, CSS_FORMAT_RAW_10, CSS_BAYER_ORDER_GRBG },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, 10, 10, CSS_FORMAT_RAW_10, CSS_BAYER_ORDER_RGGB },
#ifndef CONFIG_VIDEO_ATOMISP_CSS20
	{ V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8, 10, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_BGGR },
	{ V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8, 10, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_GBRG },
	{ V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8, 10, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_GRBG },
	{ V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8, 10, 8, CSS_FORMAT_RAW_8, CSS_BAYER_ORDER_RGGB },
#endif /* CONFIG_VIDEO_ATOMISP_CSS20 */
	{ V4L2_MBUS_FMT_SBGGR12_1X12, 12, 12, CSS_FORMAT_RAW_12, CSS_BAYER_ORDER_BGGR },
	{ V4L2_MBUS_FMT_SGBRG12_1X12, 12, 12, CSS_FORMAT_RAW_12, CSS_BAYER_ORDER_GBRG },
	{ V4L2_MBUS_FMT_SGRBG12_1X12, 12, 12, CSS_FORMAT_RAW_12, CSS_BAYER_ORDER_GRBG },
	{ V4L2_MBUS_FMT_SRGGB12_1X12, 12, 12, CSS_FORMAT_RAW_12, CSS_BAYER_ORDER_RGGB },
	{ V4L2_MBUS_FMT_UYVY8_1X16, 8, 8, ATOMISP_INPUT_FORMAT_YUV422_8, 0 },
	{ V4L2_MBUS_FMT_YUYV8_1X16, 8, 8, ATOMISP_INPUT_FORMAT_YUV422_8, 0 },
	{}
};

static const struct {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_mbus_pixelcode compressed;
} compressed_codes[] = {
	{ V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8 },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8 },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8 },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8 },
};

enum v4l2_mbus_pixelcode atomisp_subdev_uncompressed_code(
	enum v4l2_mbus_pixelcode code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(compressed_codes); i++)
		if (code == compressed_codes[i].compressed)
			return compressed_codes[i].code;

	return code;
}

bool atomisp_subdev_is_compressed(enum v4l2_mbus_pixelcode code)
{
	const struct atomisp_in_fmt_conv *ic = atomisp_in_fmt_conv;

	while (ic->code) {
		if (code == ic->code)
			return ic->bpp != ic->depth;
		ic++;
	}

	return false;
}

const struct atomisp_in_fmt_conv *atomisp_find_in_fmt_conv(
	enum v4l2_mbus_pixelcode code)
{
	const struct atomisp_in_fmt_conv *ic = atomisp_in_fmt_conv;

	while (ic->code) {
		if (code == ic->code)
			return ic;
		ic++;
	}

	return NULL;
}

bool atomisp_subdev_format_conversion(struct atomisp_device *isp,
				      unsigned int source_pad)
{
	struct v4l2_mbus_framefmt *sink, *src;

	sink = atomisp_subdev_get_ffmt(&isp->asd.subdev, NULL,
				       V4L2_SUBDEV_FORMAT_ACTIVE,
				       ATOMISP_SUBDEV_PAD_SINK);
	src = atomisp_subdev_get_ffmt(&isp->asd.subdev, NULL,
				      V4L2_SUBDEV_FORMAT_ACTIVE, source_pad);

	return atomisp_is_mbuscode_raw(sink->code)
		&& !atomisp_is_mbuscode_raw(src->code);
}

uint16_t atomisp_subdev_source_pad(struct video_device *vdev)
{
	return vdev->entity.links[0].source->index;
}

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
	struct v4l2_fh *fh, struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC &&
	    sub->type != V4L2_EVENT_ATOMISP_3A_STATS_READY)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int isp_subdev_unsubscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh, struct v4l2_event_subscription *sub)
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
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(atomisp_in_fmt_conv) - 1)
		return -EINVAL;

	code->code = atomisp_in_fmt_conv[code->index].code;

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
	default:
		switch (target) {
		case V4L2_SEL_TGT_COMPOSE:
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
		case V4L2_SEL_TGT_COMPOSE:
			return v4l2_subdev_get_try_compose(fh, pad);
		}
	}

	switch (target) {
	case V4L2_SEL_TGT_CROP:
		return &isp_sd->fmt[pad].crop;
	case V4L2_SEL_TGT_COMPOSE:
		return &isp_sd->fmt[pad].compose;
	}

	return NULL;
}

struct v4l2_mbus_framefmt
*atomisp_subdev_get_ffmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			 uint32_t which, uint32_t pad)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &isp_sd->fmt[pad].fmt;
}

static void isp_get_fmt_rect(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			     uint32_t which, struct v4l2_mbus_framefmt **ffmt,
			     struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
			     struct v4l2_rect *comp[ATOMISP_SUBDEV_PADS_NUM])
{
	unsigned int i;

	for (i = 0; i < ATOMISP_SUBDEV_PADS_NUM; i++) {
		ffmt[i] = atomisp_subdev_get_ffmt(sd, fh, which, i);
		crop[i] = atomisp_subdev_get_rect(sd, fh, which, i,
						  V4L2_SEL_TGT_CROP);
		comp[i] = atomisp_subdev_get_rect(sd, fh, which, i,
						  V4L2_SEL_TGT_COMPOSE);
	}
}

static void isp_subdev_propagate(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 uint32_t which, uint32_t pad, uint32_t target,
				 uint32_t flags)
{
	struct v4l2_mbus_framefmt *ffmt[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
		*comp[ATOMISP_SUBDEV_PADS_NUM];

	if (flags & V4L2_SEL_FLAG_KEEP_CONFIG)
		return;

	isp_get_fmt_rect(sd, fh, which, ffmt, crop, comp);

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK: {
		struct v4l2_rect r = {0};

		/* Only crop target supported on sink pad. */
		r.width = ffmt[pad]->width;
		r.height = ffmt[pad]->height;

		atomisp_subdev_set_selection(sd, fh, which, pad, target, flags,
					     &r);
		break;
	}
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

static char *atomisp_pad_str[] = { "ATOMISP_SUBDEV_PAD_SINK",
				   "ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE",
				   "ATOMISP_SUBDEV_PAD_SOURCE_VF",
				   "ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW" };

int atomisp_subdev_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh, uint32_t which,
				 uint32_t pad, uint32_t target, uint32_t flags,
				 struct v4l2_rect *r)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	struct v4l2_mbus_framefmt *ffmt[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
		*comp[ATOMISP_SUBDEV_PADS_NUM];
	unsigned int i;
	unsigned int padding_w = pad_w;
	unsigned int padding_h = pad_h;

	isp_get_fmt_rect(sd, fh, which, ffmt, crop, comp);

	dev_dbg(isp->dev,
		"sel: pad %s tgt %s l %d t %d w %d h %d which %s f 0x%8.8x\n",
		atomisp_pad_str[pad], target == V4L2_SEL_TGT_CROP
		? "V4L2_SEL_TGT_CROP" : "V4L2_SEL_TGT_COMPOSE",
		r->left, r->top, r->width, r->height,
		which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY"
		: "V4L2_SUBDEV_FORMAT_ACTIVE", flags);

	r->width = rounddown(r->width, ATOM_ISP_STEP_WIDTH);
	r->height = rounddown(r->height, ATOM_ISP_STEP_HEIGHT);

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK: {
		/* Only crop target supported on sink pad. */
		unsigned int dvs_w, dvs_h;

		crop[pad]->width = ffmt[pad]->width;
		crop[pad]->height = ffmt[pad]->height;

		/* Workaround for BYT 1080p perfectshot since the maxinum resolution of
		 * front camera ov2722 is 1932x1092 and cannot use pad_w > 12*/
		if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2
			&& crop[pad]->height == 1092) {
			padding_w = 12;
			padding_h = 12;
		}

		if (atomisp_subdev_format_conversion(isp, isp_sd->capture_pad)
		    && crop[pad]->width && crop[pad]->height)
			crop[pad]->width -= padding_w, crop[pad]->height -= padding_h;

		/* if subdev type is SOC camera,we do not need to set DVS */
		if (isp->inputs[isp_sd->input_curr].type == SOC_CAMERA)
			isp_sd->params.video_dis_en = 0;

		if (isp_sd->params.video_dis_en &&
		    isp->asd.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
			/* This resolution contains 20 % of DVS slack
			 * (of the desired captured image before
			 * scaling, or 1 / 6 of what we get from the
			 * sensor) in both width and height. Remove
			 * it. */
			crop[pad]->width = roundup(crop[pad]->width * 5 / 6,
						   ATOM_ISP_STEP_WIDTH);
			crop[pad]->height = roundup(crop[pad]->height * 5 / 6,
						    ATOM_ISP_STEP_HEIGHT);
		}

		crop[pad]->width = min(crop[pad]->width, r->width);
		crop[pad]->height = min(crop[pad]->height, r->height);

		if (!(flags & V4L2_SEL_FLAG_KEEP_CONFIG)) {
			for (i = ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE;
			     i <= ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW; i++) {
				struct v4l2_rect tmp = *crop[pad];

				atomisp_subdev_set_selection(
					sd, fh, which, i, V4L2_SEL_TGT_COMPOSE,
					flags, &tmp);
			}
		}

		if (which == V4L2_SUBDEV_FORMAT_TRY)
			break;

		if (isp_sd->params.video_dis_en &&
		    isp->asd.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
			dvs_w = rounddown(crop[pad]->width / 5,
					  ATOM_ISP_STEP_WIDTH);
			dvs_h = rounddown(crop[pad]->height / 5,
					  ATOM_ISP_STEP_HEIGHT);
		} else {
			dvs_w = dvs_h = 0;
		}

		atomisp_css_video_set_dis_envelope(isp_sd, dvs_w, dvs_h);
		atomisp_css_input_set_effective_resolution(isp_sd,
					crop[pad]->width, crop[pad]->height);

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE: {
		/* Only compose target is supported on source pads. */

		if (isp->asd.vfpp->val == ATOMISP_VFPP_DISABLE_LOWLAT) {
			/* Scaling is disabled in this mode */
			r->width = crop[ATOMISP_SUBDEV_PAD_SINK]->width;
			r->height = crop[ATOMISP_SUBDEV_PAD_SINK]->height;
		}

		if (crop[ATOMISP_SUBDEV_PAD_SINK]->width == r->width
		    && crop[ATOMISP_SUBDEV_PAD_SINK]->height == r->height)
			isp_sd->params.yuv_ds_en = false;
		else
			isp_sd->params.yuv_ds_en = true;

		comp[pad]->width = r->width;
		comp[pad]->height = r->height;

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		comp[pad]->width = r->width;
		comp[pad]->height = r->height;
		break;
	default:
		return -EINVAL;
	}

	/* Set format dimensions on non-sink pads as well. */
	if (pad != ATOMISP_SUBDEV_PAD_SINK) {
		ffmt[pad]->width = comp[pad]->width;
		ffmt[pad]->height = comp[pad]->height;
	}

	*r = *atomisp_subdev_get_rect(sd, fh, which, pad, target);

	dev_dbg(isp->dev, "sel actual: l %d t %d w %d h %d\n",
		r->left, r->top, r->width, r->height);

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
					    sel->target, sel->flags, &sel->r);
}

static int atomisp_get_sensor_bin_factor(struct atomisp_sub_device *asd)
{
	struct v4l2_control ctrl = {0};
	struct atomisp_device *isp = asd->isp;
	int hbin, vbin;
	int ret;

	if (isp->inputs[asd->input_curr].type == FILE_INPUT ||
		isp->inputs[asd->input_curr].type == TEST_PATTERN)
		return 0;

	ctrl.id = V4L2_CID_BIN_FACTOR_HORZ;
	ret = v4l2_subdev_call(isp->inputs[asd->input_curr].camera, core,
			       g_ctrl, &ctrl);
	hbin = ctrl.value;
	ctrl.id = V4L2_CID_BIN_FACTOR_VERT;
	ret |= v4l2_subdev_call(isp->inputs[asd->input_curr].camera, core,
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

void atomisp_subdev_set_ffmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			     uint32_t which, uint32_t pad,
			     struct v4l2_mbus_framefmt *ffmt)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	struct v4l2_mbus_framefmt *__ffmt =
		atomisp_subdev_get_ffmt(sd, fh, which, pad);

	dev_dbg(isp->dev, "ffmt: pad %s w %d h %d code 0x%8.8x which %s\n",
		atomisp_pad_str[pad], ffmt->width, ffmt->height, ffmt->code,
		which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY"
		: "V4L2_SUBDEV_FORMAT_ACTIVE");

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK: {
		const struct atomisp_in_fmt_conv *fc =
			atomisp_find_in_fmt_conv(ffmt->code);

		if (!fc) {
			fc = atomisp_in_fmt_conv;
			ffmt->code = fc->code;
			dev_dbg(isp->dev, "using 0x%8.8x instead\n",
				ffmt->code);
		}

		*__ffmt = *ffmt;

		isp_subdev_propagate(sd, fh, which, pad,
				     V4L2_SEL_TGT_CROP, 0);

		if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			atomisp_css_input_set_resolution(isp_sd, ffmt);
			atomisp_css_input_set_binning_factor(isp_sd,
				atomisp_get_sensor_bin_factor(isp_sd));
			atomisp_css_input_set_bayer_order(isp_sd,
							  fc->bayer_order);
			atomisp_css_input_set_format(isp_sd, fc->in_sh_fmt);
		}

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		__ffmt->code = ffmt->code;
		break;
	}
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
	fmt->format = *atomisp_subdev_get_ffmt(sd, fh, fmt->which, fmt->pad);

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
	atomisp_subdev_set_ffmt(sd, fh, fmt->which, fmt->pad, &fmt->format);

	return 0;
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops isp_subdev_v4l2_core_ops = {
	 .ioctl = isp_subdev_ioctl, .s_power = isp_subdev_set_power,
	 .subscribe_event = isp_subdev_subscribe_event,
	 .unsubscribe_event = isp_subdev_unsubscribe_event,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops isp_subdev_v4l2_pad_ops = {
	 .enum_mbus_code = isp_subdev_enum_mbus_code,
	 .get_fmt = isp_subdev_get_format, .set_fmt = isp_subdev_set_format,
	 .get_selection = isp_subdev_get_selection,
	 .set_selection = isp_subdev_set_selection,
	 .link_validate = v4l2_subdev_link_validate_default,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops isp_subdev_v4l2_ops = {
	 .core = &isp_subdev_v4l2_core_ops, .pad = &isp_subdev_v4l2_pad_ops,
};

static void isp_subdev_init_params(struct atomisp_sub_device *asd)
{
	/* parameters initialization */
	INIT_LIST_HEAD(&asd->s3a_stats);
	INIT_LIST_HEAD(&asd->dis_stats);
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
	 .link_validate = v4l2_subdev_link_validate,
/*	 .set_power = v4l2_subdev_set_power,	*/
};

static int __atomisp_update_run_mode(struct atomisp_sub_device *asd)
{
	struct atomisp_device *isp = asd->isp;
	struct v4l2_ctrl *ctrl = isp->asd.run_mode;
	struct v4l2_ctrl *c;
	struct v4l2_streamparm p = {0};
	int modes[] = { CI_MODE_NONE,
			CI_MODE_VIDEO,
			CI_MODE_STILL_CAPTURE,
			CI_MODE_CONTINUOUS,
			CI_MODE_PREVIEW };
	s32 mode;

	if (ctrl->val != ATOMISP_RUN_MODE_VIDEO &&
	    isp->asd.continuous_mode->val)
		mode = ATOMISP_RUN_MODE_PREVIEW;
	else
		mode = ctrl->val;

	c = v4l2_ctrl_find(
		isp->inputs[asd->input_curr].camera->ctrl_handler,
		V4L2_CID_RUN_MODE);

	if (c)
		return v4l2_ctrl_s_ctrl(c, mode);

	/* Fall back to obsolete s_parm */
	p.parm.capture.capturemode = modes[mode];

	return v4l2_subdev_call(
		isp->inputs[asd->input_curr].camera, video, s_parm, &p);
}

int atomisp_update_run_mode(struct atomisp_sub_device *asd)
{
	int rval;

	mutex_lock(asd->ctrl_handler.lock);
	rval = __atomisp_update_run_mode(asd);
	mutex_unlock(asd->ctrl_handler.lock);

	return rval;
}

static int s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct atomisp_sub_device *asd = container_of(
		ctrl->handler, struct atomisp_sub_device, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_RUN_MODE:
		return __atomisp_update_run_mode(asd);
	}

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

static const char * const ctrl_run_mode_menu[] = {
	NULL,
	"Video",
	"Still capture",
	"Continuous capture",
	"Preview",
};

static const struct v4l2_ctrl_config ctrl_run_mode = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_RUN_MODE,
	.name = "Atomisp run mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 1,
	.def = 1,
	.max = 4,
	.qmenu = ctrl_run_mode_menu,
};

static const char * const ctrl_vfpp_mode_menu[] = {
	"Enable",			/* vfpp always enabled */
	"Disable to scaler mode",	/* CSS into video mode and disable */
	"Disable to low latency mode",	/* CSS into still mode and disable */
};

static const struct v4l2_ctrl_config ctrl_vfpp = {
	.id = V4L2_CID_VFPP,
	.name = "Atomisp vf postprocess",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.def = 0,
	.max = 2,
	.qmenu = ctrl_vfpp_mode_menu,
};

/*
 * Control for ISP continuous mode
 *
 * When enabled, capture processing is possible without
 * stopping the preview pipeline. When disabled, ISP needs
 * to be restarted between preview and capture.
 */
static const struct v4l2_ctrl_config ctrl_continuous_mode = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Continuous mode",
	.min = 0,
	.max = 1,
	.def = 0,
};

/*
 * Control for continuous mode raw buffer size
 *
 * The size of the RAW ringbuffer sets limit on how much
 * back in time application can go when requesting capture
 * frames to be rendered, and how many frames can be rendered
 * in a burst at full sensor rate.
 *
 * Note: this setting has a big impact on memory consumption of
 * the CSS subsystem.
 */
static const struct v4l2_ctrl_config ctrl_continuous_raw_buffer_size = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Continuous raw ringbuffer size",
	.min = 1,
	.max = 100, /* depends on CSS version, runtime checked */
	.step = 1,
	.def = 3,
};

/*
 * Control for enabling continuous viewfinder
 *
 * When enabled, and ISP is in continuous mode (see ctrl_continuous_mode ),
 * preview pipeline continues concurrently with capture
 * processing. When disabled, and continuous mode is used,
 * preview is paused while captures are processed, but
 * full pipeline restart is not needed.
 *
 * By setting this to disabled, capture processing is
 * essentially given priority over preview, and the effective
 * capture output rate may be higher than with continuous
 * viewfinder enabled.
 */
static const struct v4l2_ctrl_config ctrl_continuous_viewfinder = {
	.id = V4L2_CID_ATOMISP_CONTINUOUS_VIEWFINDER,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Continuous viewfinder",
	.min = 0,
	.max = 1,
	.def = 0,
};

static void atomisp_init_subdev_pipe(struct atomisp_sub_device *asd,
		struct atomisp_video_pipe *pipe, enum v4l2_buf_type buf_type)
{
	pipe->type = buf_type;
	pipe->asd = asd;
	pipe->isp = asd->isp;
	spin_lock_init(&pipe->irq_lock);
	INIT_LIST_HEAD(&pipe->activeq);
	INIT_LIST_HEAD(&pipe->activeq_out);
}

/*
 * isp_subdev_init_entities - Initialize V4L2 subdev and media entity
 * @asd: ISP CCDC module
 *
 * Return 0 on success and a negative error code on failure.
 */
static int isp_subdev_init_entities(struct atomisp_sub_device *asd)
{
	struct v4l2_subdev *sd = &asd->subdev;
	struct media_pad *pads = asd->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	asd->input = ATOMISP_SUBDEV_INPUT_NONE;

	v4l2_subdev_init(sd, &isp_subdev_v4l2_ops);
	strlcpy(sd->name, "ATOM ISP SUBDEV", sizeof(sd->name));
	v4l2_set_subdevdata(sd, asd);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[ATOMISP_SUBDEV_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_VF].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].flags = MEDIA_PAD_FL_SOURCE;

	asd->fmt[ATOMISP_SUBDEV_PAD_SINK].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	asd->fmt[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	asd->fmt[ATOMISP_SUBDEV_PAD_SOURCE_VF].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	asd->fmt[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;

	me->ops = &isp_subdev_media_ops;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(me, ATOMISP_SUBDEV_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	atomisp_init_subdev_pipe(asd, &asd->video_in,
			V4L2_BUF_TYPE_VIDEO_OUTPUT);

	atomisp_init_subdev_pipe(asd, &asd->video_out_preview,
			V4L2_BUF_TYPE_VIDEO_CAPTURE);

	atomisp_init_subdev_pipe(asd, &asd->video_out_vf,
			V4L2_BUF_TYPE_VIDEO_CAPTURE);

	atomisp_init_subdev_pipe(asd, &asd->video_out_capture,
			V4L2_BUF_TYPE_VIDEO_CAPTURE);

	ret = atomisp_video_init(&asd->video_in, "MEMORY");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&asd->video_out_capture, "CAPTURE");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&asd->video_out_vf, "VIEWFINDER");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&asd->video_out_preview, "PREVIEW");
	if (ret < 0)
		return ret;

	/* Connect the isp subdev to the video node. */
	ret = media_entity_create_link(&asd->video_in.vdev.entity,
		0, &asd->subdev.entity, ATOMISP_SUBDEV_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&asd->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW,
		&asd->video_out_preview.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&asd->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_VF,
		&asd->video_out_vf.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&asd->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
		&asd->video_out_capture.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_ctrl_handler_init(&asd->ctrl_handler, 1);
	if (ret)
		return ret;

	asd->fmt_auto = v4l2_ctrl_new_custom(&asd->ctrl_handler,
						    &ctrl_fmt_auto, NULL);
	asd->run_mode = v4l2_ctrl_new_custom(&asd->ctrl_handler,
						    &ctrl_run_mode, NULL);
	asd->vfpp = v4l2_ctrl_new_custom(&asd->ctrl_handler,
						&ctrl_vfpp, NULL);
	asd->continuous_mode = v4l2_ctrl_new_custom(&asd->ctrl_handler,
					     &ctrl_continuous_mode, NULL);
	asd->continuous_viewfinder = v4l2_ctrl_new_custom(&asd->ctrl_handler,
					     &ctrl_continuous_viewfinder,
					     NULL);
	asd->continuous_raw_buffer_size =
			v4l2_ctrl_new_custom(&asd->ctrl_handler,
					     &ctrl_continuous_raw_buffer_size,
					     NULL);

	/* Make controls visible on subdev as well. */
	asd->subdev.ctrl_handler = &asd->ctrl_handler;

	return asd->ctrl_handler.error;
}

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *asd)
{
	v4l2_ctrl_handler_free(&asd->ctrl_handler);

	media_entity_cleanup(&asd->subdev.entity);

	v4l2_device_unregister_subdev(&asd->subdev);
	atomisp_video_unregister(&asd->video_in);
	atomisp_video_unregister(&asd->video_out_preview);
	atomisp_video_unregister(&asd->video_out_vf);
	atomisp_video_unregister(&asd->video_out_capture);
}

int atomisp_subdev_register_entities(struct atomisp_sub_device *asd,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &asd->subdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&asd->video_out_capture, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&asd->video_out_vf, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&asd->video_out_preview, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&asd->video_in, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	atomisp_subdev_unregister_entities(asd);
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
	struct atomisp_sub_device *asd = &isp->asd;
	int ret;

	spin_lock_init(&asd->lock);
	asd->isp = isp;
	isp_subdev_init_params(asd);
	ret = isp_subdev_init_entities(asd);
	if (ret < 0)
		atomisp_subdev_cleanup(isp);

	return ret;
}

void atomisp_subdev_cleanup(struct atomisp_device *isp)
{

}

