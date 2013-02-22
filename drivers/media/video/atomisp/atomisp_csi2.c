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

#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include "atomisp_internal.h"

static const unsigned int csi2_input_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};

static const unsigned int csi2_output_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};


/* V4L2 subdev operations */

static struct v4l2_mbus_framefmt *
__csi2_get_format(struct atomisp_mipi_csi2_device *csi2,
		struct v4l2_subdev_fh *fh,
		unsigned int pad,
		enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &csi2->formats[pad];
}

static enum v4l2_mbus_pixelcode
isp_video_uncompressed_code(enum v4l2_mbus_pixelcode code)
{
	switch (code) {
	case V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8:
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	case V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8:
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	case V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8:
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	default:
		return code;
	}
}

static void
csi2_try_format(struct atomisp_mipi_csi2_device *csi2,
	struct v4l2_subdev_fh *fh,
	unsigned int pad,
	struct v4l2_mbus_framefmt *fmt,
	enum v4l2_subdev_format_whence which)
{
	enum v4l2_mbus_pixelcode pixelcode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	switch (pad) {
	case CSI2_PAD_SINK:
		/* Clamp the width and height to valid range (1-8191). */
		for (i = 0; i < ARRAY_SIZE(csi2_input_fmts); i++) {
			if (fmt->code == csi2_input_fmts[i])
				break;
		}

		/* If not found, use SGRBG10 as default */
		if (i >= ARRAY_SIZE(csi2_input_fmts))
			fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

		fmt->width = clamp_t(u32, fmt->width, 1, 4608);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);
		break;

	case CSI2_PAD_SOURCE:
		/* Source format same as sink format, except for DPCM
		 * compression.
		 */
		pixelcode = fmt->code;
		format = __csi2_get_format(csi2, fh, CSI2_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* allow dpcm decompression */
		if (isp_video_uncompressed_code(fmt->code) == pixelcode)
			fmt->code = pixelcode;

		break;

	default:
		break;
	}

	/* RGB, non-interlaced */
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
}

/*
 * csi2_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @fh     : V4L2 subdev file handle
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
*/
static int csi2_enum_mbus_code(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_mbus_code_enum *code)
{
	struct atomisp_mipi_csi2_device *csi2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (code->pad == CSI2_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(csi2_input_fmts))
			return -EINVAL;
		code->code = csi2_input_fmts[code->index];
	} else {
		format = __csi2_get_format(csi2, fh, CSI2_PAD_SINK,
			V4L2_SUBDEV_FORMAT_TRY);
		switch (code->index) {
		case 0:
			/* Passthrough sink pad code */
			code->code = format->code;
			break;
		case 1:
			/* Uncompressed code */
			code->code = isp_video_uncompressed_code(format->code);
			break;
		default:
			/* Fallthrough if above is false */
			return -EINVAL;
		}
	}

	return 0;
}

static int csi2_enum_frame_size(struct v4l2_subdev *sd,
			 struct v4l2_subdev_fh *fh,
			 struct v4l2_subdev_frame_size_enum *fse)
{
	struct atomisp_mipi_csi2_device *csi2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	csi2_try_format(csi2, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	csi2_try_format(csi2, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * csi2_get_format - Handle get format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @fh : V4L2 subdev file handle
 * @pad: pad num
 * @fmt: pointer to v4l2 format structure
 * return -EINVAL or zero on sucess
*/
static int csi2_get_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	struct atomisp_mipi_csi2_device *csi2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csi2_get_format(csi2, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

/*
 * csi2_set_format - Handle set format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @fh : V4L2 subdev file handle
 * @pad: pad num
 * @fmt: pointer to v4l2 format structure
 * return -EINVAL or zero on success
*/
static int csi2_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		    struct v4l2_subdev_format *fmt)
{
	struct atomisp_mipi_csi2_device *csi2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csi2_get_format(csi2, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	csi2_try_format(csi2, fh, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == CSI2_PAD_SINK) {
		format = __csi2_get_format(csi2, fh, CSI2_PAD_SOURCE,
			fmt->which);
		*format = fmt->format;
		csi2_try_format(csi2, fh, CSI2_PAD_SOURCE, format, fmt->which);
	}

	return 0;
}

/*
 * csi2_set_stream - Enable/Disable streaming on the CSI2 module
 * @sd: ISP CSI2 V4L2 subdevice
 * @enable: Enable/disable stream (1/0)
 *
 * Return 0 on success or a negative error code otherwise.
*/
static int csi2_set_stream(struct v4l2_subdev *sd, int enable)
{
	 return 0;
}

/* subdev core operations */
static const struct v4l2_subdev_core_ops csi2_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops csi2_video_ops = {
	.s_stream = csi2_set_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops csi2_pad_ops = {
	.enum_mbus_code = csi2_enum_mbus_code,
	.enum_frame_size = csi2_enum_frame_size,
	.get_fmt = csi2_get_format,
	.set_fmt = csi2_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops csi2_ops = {
	.core = &csi2_core_ops,
	.video = &csi2_video_ops,
	.pad = &csi2_pad_ops,
};


/*
 * csi2_link_setup - Setup CSI2 connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
*/
static int csi2_link_setup(struct media_entity *entity,
	    const struct media_pad *local,
	    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct atomisp_mipi_csi2_device *csi2 = v4l2_get_subdevdata(sd);
	u32 result = local->index | media_entity_type(remote->entity);

	switch (result) {
	case CSI2_PAD_SOURCE | MEDIA_ENT_T_DEVNODE:
		/* not supported yet */
		return -EINVAL;

	case CSI2_PAD_SOURCE | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (csi2->output & ~CSI2_OUTPUT_ISP_SUBDEV)
				return -EBUSY;
			csi2->output |= CSI2_OUTPUT_ISP_SUBDEV;
		} else {
			csi2->output &= ~CSI2_OUTPUT_ISP_SUBDEV;
		}
		break;

	default:
		/* Link from camera to CSI2 is fixed... */
		return -EINVAL;
	}
	return 0;
}

/* media operations */
static const struct media_entity_operations csi2_media_ops = {
	.link_setup = csi2_link_setup,
};

/*
* ispcsi2_init_entities - Initialize subdev and media entity.
* @csi2: Pointer to ispcsi2 structure.
* return -ENOMEM or zero on success
*/
static int mipi_csi2_init_entities(struct atomisp_mipi_csi2_device *csi2,
					int port)
{
	struct v4l2_subdev *sd = &csi2->subdev;
	struct media_pad *pads = csi2->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	v4l2_subdev_init(sd, &csi2_ops);
	if (port == ATOMISP_CAMERA_PORT_SECONDARY)
		strlcpy(sd->name, "ATOM ISP CSI2-1p", sizeof(sd->name));
	else if (port == ATOMISP_CAMERA_PORT_PRIMARY)
		strlcpy(sd->name, "ATOM ISP CSI2-4p", sizeof(sd->name));

	v4l2_set_subdevdata(sd, csi2);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[CSI2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	pads[CSI2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	me->ops = &csi2_media_ops;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(me, CSI2_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	csi2->formats[CSI2_PAD_SINK].code = V4L2_MBUS_FMT_SBGGR10_1X10;
	csi2->formats[CSI2_PAD_SOURCE].code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

void
atomisp_mipi_csi2_unregister_entities(struct atomisp_mipi_csi2_device *csi2)
{
	media_entity_cleanup(&csi2->subdev.entity);
	v4l2_device_unregister_subdev(&csi2->subdev);
}

int atomisp_mipi_csi2_register_entities(struct atomisp_mipi_csi2_device *csi2,
			struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &csi2->subdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	atomisp_mipi_csi2_unregister_entities(csi2);
	return ret;
}

/*
 * atomisp_mipi_csi2_cleanup - Routine for module driver cleanup
*/
void atomisp_mipi_csi2_cleanup(struct atomisp_device *isp)
{
}


int atomisp_mipi_csi2_init(struct atomisp_device *isp)
{
	struct atomisp_mipi_csi2_device *csi2_4p = &isp->csi2_4p;
	struct atomisp_mipi_csi2_device *csi2_1p = &isp->csi2_1p;
	int ret;

	csi2_4p->isp = isp;
	csi2_1p->isp = isp;

	ret = mipi_csi2_init_entities(csi2_4p, ATOMISP_CAMERA_PORT_PRIMARY);
	if (ret < 0)
		goto fail;

	ret = mipi_csi2_init_entities(csi2_1p, ATOMISP_CAMERA_PORT_SECONDARY);
	if (ret < 0)
		goto fail;
	return 0;

fail:
	atomisp_mipi_csi2_cleanup(isp);
	return ret;
}

