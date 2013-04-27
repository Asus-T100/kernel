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

#include <linux/delay.h>

#include <asm/intel-mid.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf-vmalloc.h>

#include "atomisp_acc.h"
#include "atomisp_compat.h"
#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp-regs.h"

#include "sh_css_hrt.h"
#include "sh_css_debug.h"
#include "gp_device.h"
#include "device_access.h"
#include "irq.h"

#include "hrt/hive_isp_css_mm_hrt.h"

/* for v4l2_capability */
static const char *DRIVER = "atomisp";	/* max size 15 */
static const char *CARD = "ATOM ISP";	/* max size 31 */
static const char *BUS_INFO = "PCI-3";	/* max size 31 */
static const u32 VERSION = DRIVER_VERSION;

/*
 * FIXME: ISP should not know beforehand all CIDs supported by sensor.
 * Instead, it needs to propagate to sensor unkonwn CIDs.
 */
static struct v4l2_queryctrl ci_v4l2_controls[] = {
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Automatic White Balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Red Balance",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Blue Balance",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_GAMMA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Gamma",
		.minimum = 0x00,
		.maximum = 0xff,
		.step = 1,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_HFLIP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image h-flip",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_VFLIP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image v-flip",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Light frequency filter",
		.minimum = 1,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Image Color Effect",
		.minimum = 0,
		.maximum = 9,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Bad Pixel Correction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "GDC/CAC",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_VIDEO_STABLIZATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Video Stablization",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_FIXED_PATTERN_NR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Fixed Pattern Noise Reduction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "False Color Correction",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_REQUEST_FLASH,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Request flash frames",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_ATOMISP_LOW_LIGHT,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Low light mode",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 1,
	},
	{
		.id = V4L2_CID_BIN_FACTOR_HORZ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Horizontal binning factor",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_BIN_FACTOR_VERT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Vertical binning factor",
		.minimum = 0,
		.maximum = 10,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_2A_STATUS,
		.type = V4L2_CTRL_TYPE_BITMASK,
		.name = "AE and AWB status",
		.minimum = 0,
		.maximum = V4L2_2A_STATUS_AE_READY | V4L2_2A_STATUS_AWB_READY,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "exposure",
		.minimum = -4,
		.maximum = 4,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_SCENE_MODE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "scene mode",
		.minimum = 0,
		.maximum = 13,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_ISO_SENSITIVITY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "iso",
		.minimum = -4,
		.maximum = 4,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "white balance",
		.minimum = 0,
		.maximum = 9,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_METERING,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "metering",
		.minimum = 0,
		.maximum = 2,
		.step = 1,
		.default_value = 1,
	},
};
static const u32 ctrls_num = ARRAY_SIZE(ci_v4l2_controls);

/*
 * supported V4L2 fmts and resolutions
 */
const struct atomisp_format_bridge atomisp_output_fmts[] = {
	{
		.pixelformat = V4L2_PIX_FMT_YUV420,
		.depth = 12,
		.mbus_code = 0x8001,
		.sh_fmt = IA_CSS_FRAME_FORMAT_YUV420,
		.description = "YUV420, planner"
	}, {
		.pixelformat = V4L2_PIX_FMT_YVU420,
		.depth = 12,
		.mbus_code = 0x8002,
		.sh_fmt = IA_CSS_FRAME_FORMAT_YV12,
		.description = "YVU420, planner"
	}, {
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.depth = 16,
		.mbus_code = 0x8003,
		.sh_fmt = IA_CSS_FRAME_FORMAT_YUV422,
		.description = "YUV422, planner"
	}, {
		.pixelformat = V4L2_PIX_FMT_YUV444,
		.depth = 24,
		.mbus_code = 0x8004,
		.sh_fmt = IA_CSS_FRAME_FORMAT_YUV444,
		.description = "YUV444"
	}, {
		.pixelformat = V4L2_PIX_FMT_NV12,
		.depth = 12,
		.mbus_code = 0x8005,
		.sh_fmt = IA_CSS_FRAME_FORMAT_NV12,
		.description = "NV12, interleaved"
	}, {
		.pixelformat = V4L2_PIX_FMT_NV21,
		.depth = 12,
		.mbus_code = 0x8006,
		.sh_fmt = IA_CSS_FRAME_FORMAT_NV21,
		.description = "NV21, interleaved"
	}, {
		.pixelformat = V4L2_PIX_FMT_NV16,
		.depth = 16,
		.mbus_code = 0x8007,
		.sh_fmt = IA_CSS_FRAME_FORMAT_NV16,
		.description = "NV16, interleaved"
	}, {
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.depth = 16,
		.mbus_code = 0x8008,
		.sh_fmt = IA_CSS_FRAME_FORMAT_YUYV,
		.description = "YUYV, interleaved"
	}, {
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_1X16,
		.sh_fmt = IA_CSS_FRAME_FORMAT_UYVY,
		.description = "UYVY, interleaved"
	}, { /* This one is for parallel sensors! DO NOT USE! */
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.sh_fmt = IA_CSS_FRAME_FORMAT_UYVY,
		.description = "UYVY, interleaved"
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR16,
		.depth = 16,
		.mbus_code = 0x8009,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 16"
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SBGGR8_1X8,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGBRG8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGBRG8_1X8,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGRBG8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGRBG8_1X8,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"
	}, {
		.pixelformat = V4L2_PIX_FMT_SRGGB8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SRGGB8_1X8,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGBRG10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGBRG10_1X10,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"
	}, {
		.pixelformat = V4L2_PIX_FMT_SRGGB10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SRGGB10_1X10,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR12_1X12,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGBRG12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGBRG12_1X12,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"
	}, {
		.pixelformat = V4L2_PIX_FMT_SGRBG12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG12_1X12,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"
	}, {
		.pixelformat = V4L2_PIX_FMT_SRGGB12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SRGGB12_1X12,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"
	}, {
		.pixelformat = V4L2_PIX_FMT_RGB32,
		.depth = 32,
		.mbus_code = 0x800a,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RGBA888,
		.description = "32 RGB 8-8-8-8"
	}, {
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_BGR565_2X8_LE,
		.sh_fmt = IA_CSS_FRAME_FORMAT_RGB565,
		.description = "16 RGB 5-6-5"
	},
};

const struct atomisp_format_bridge *atomisp_get_format_bridge(
	unsigned int pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(atomisp_output_fmts); i++) {
		if (atomisp_output_fmts[i].pixelformat == pixelformat)
			return &atomisp_output_fmts[i];
	}

	return NULL;
}

const struct atomisp_format_bridge *atomisp_get_format_bridge_from_mbus(
	enum v4l2_mbus_pixelcode mbus_code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(atomisp_output_fmts); i++) {
		if (mbus_code == atomisp_output_fmts[i].mbus_code)
			return &atomisp_output_fmts[i];
	}

	return NULL;
}

unsigned int atomisp_get_pipe_index(struct atomisp_device *isp,
					uint16_t source_pad)
{
	switch (source_pad) {
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO
		    || !isp->isp_subdev.enable_vfpp->val)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else
			return (unsigned int) IA_CSS_PIPE_ID_CAPTURE;
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else if (!atomisp_is_mbuscode_raw(
				 isp->isp_subdev.
				 fmt[isp->isp_subdev.capture_pad].fmt.code))
			return (unsigned int) IA_CSS_PIPE_ID_CAPTURE;
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			return (unsigned int) IA_CSS_PIPE_ID_VIDEO;
		else
			return (unsigned int) IA_CSS_PIPE_ID_PREVIEW;
	}

	return IA_CSS_PIPE_ID_NUM;
}
static int __get_css_frame_info(struct atomisp_device *isp,
				uint16_t source_pad,
				struct ia_css_frame_info *frame_info)
{
	struct ia_css_pipe_info info;
	int pipe_index = atomisp_get_pipe_index(isp, source_pad);

	ia_css_pipe_get_info(isp->css2_basis.pipes[pipe_index], &info);
	switch (source_pad) {
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
		*frame_info = info.output_info;
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		*frame_info = info.vf_output_info;
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO)
			*frame_info = info.vf_output_info;
		else
			*frame_info = info.output_info;
		break;
	default:
		frame_info = NULL;
		break;
	}
	return frame_info ? 0 : -EINVAL;
}

/*
 * v4l2 ioctls
 * return ISP capabilities
 *
 * FIXME: capabilities should be different for video0/video2/video3
 */
static int atomisp_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	int ret = 0;

	memset(cap, 0, sizeof(struct v4l2_capability));

	WARN_ON(sizeof(DRIVER) > sizeof(cap->driver) ||
		sizeof(CARD) > sizeof(cap->card) ||
		sizeof(BUS_INFO) > sizeof(cap->bus_info));

	strncpy(cap->driver, DRIVER, sizeof(cap->driver) - 1);
	strncpy(cap->card, CARD, sizeof(cap->card) - 1);
	strncpy(cap->bus_info, BUS_INFO, sizeof(cap->card) - 1);

	cap->version = VERSION;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
	    V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;

	return ret;
}

/*
 * return sensor chip identification
 */
static int atomisp_g_chip_ident(struct file *file, void *fh,
	struct v4l2_dbg_chip_ident *chip)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret = 0;

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       core, g_chip_ident, chip);

	if (ret)
		v4l2_err(&atomisp_dev,
			    "failed to g_chip_ident for sensor\n");
	return ret;
}

/*
 * enum input are used to check primary/secondary camera
 */
static int atomisp_enum_input(struct file *file, void *fh,
	struct v4l2_input *input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int index = input->index;

	if (index >= isp->input_cnt)
		return -EINVAL;

	if (!isp->inputs[index].camera)
		return -EINVAL;

	memset(input, 0, sizeof(struct v4l2_input));
	strncpy(input->name, isp->inputs[index].camera->name,
		sizeof(input->name) - 1);

	/*
	 * HACK: append actuator's name to sensor's
	 * As currently userspace can't talk directly to subdev nodes, this
	 * ioctl is the only way to enum inputs + possible external actuators
	 * for 3A tuning purpose.
	 */
	if (isp->inputs[index].motor &&
	    strlen(isp->inputs[index].motor->name) > 0) {
		const int cur_len = strlen(input->name);
		const int max_size = sizeof(input->name) - cur_len - 1;

		if (max_size > 0) {
			input->name[cur_len] = '+';
			strncpy(&input->name[cur_len + 1],
				isp->inputs[index].motor->name, max_size - 1);
		}
	}

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->index = index;
	input->reserved[0] = isp->inputs[index].type;
	input->reserved[1] = isp->inputs[index].port;

	return 0;
}

static unsigned int atomisp_streaming_count(struct atomisp_device *isp)
{
	return isp->isp_subdev.video_out_preview.capq.streaming
		+ isp->isp_subdev.video_out_capture.capq.streaming
		+ isp->isp_subdev.video_in.capq.streaming;
}

/*
 * get input are used to get current primary/secondary camera
 */
static int atomisp_g_input(struct file *file, void *fh, unsigned int *input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	mutex_lock(&isp->mutex);
	*input = isp->input_curr;
	mutex_unlock(&isp->mutex);

	return 0;
}
/*
 * set input are used to set current primary/secondary camera
 */
static int atomisp_s_input(struct file *file, void *fh, unsigned int input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_subdev *camera = NULL;
	int ret;

	mutex_lock(&isp->mutex);
	if (input >= ATOM_ISP_MAX_INPUTS || input > isp->input_cnt) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "input_cnt: %d\n",isp->input_cnt);
		ret = -EINVAL;
		goto error;
	}

	camera = isp->inputs[input].camera;
	if (!camera) {
		v4l2_err(&atomisp_dev,
			 "%s, no camera\n",__func__);
		ret = -EINVAL;
		goto error;
	}

	if (atomisp_streaming_count(isp)) {
		v4l2_err(&atomisp_dev,
			 "ISP is still streaming, stop first\n");
		ret = -EINVAL;
		goto error;
	}

	/* power off the current sensor, as it is not used this time */
	if (isp->input_curr != input) {
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, s_power, 0);
		if (ret)
			v4l2_warn(&atomisp_dev,
				    "Failed to power-off sensor\n");
	}

	/* powe on the new sensor */
	ret = v4l2_subdev_call(isp->inputs[input].camera,
			       core, s_power, 1);
	if (ret) {
		v4l2_err(&atomisp_dev, "Failed to power-on sensor\n");
		goto error;
	}

	if (!isp->sw_contex.file_input && isp->inputs[input].motor)
		ret = v4l2_subdev_call(isp->inputs[input].motor, core,
				       init, 1);

	isp->input_curr = input;
	mutex_unlock(&isp->mutex);

	return 0;

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

static int atomisp_enum_fmt_cap(struct file *file, void *fh,
	struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(atomisp_output_fmts))
		return -EINVAL;

	f->pixelformat = atomisp_output_fmts[f->index].pixelformat;
	memset(f->description, 0, sizeof(f->description));
	strncpy(f->description, atomisp_output_fmts[f->index].description,
		strlen(atomisp_output_fmts[f->index].description));

	return 0;
}

static int atomisp_g_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	int ret;

	mutex_lock(&isp->mutex);
	ret = atomisp_get_fmt(vdev, f);
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_g_fmt_file(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	mutex_lock(&isp->mutex);
	f->fmt.pix = pipe->pix;
	mutex_unlock(&isp->mutex);

	return 0;
}

/* This function looks up the closest available resolution. */
static int atomisp_try_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	mutex_lock(&isp->mutex);
	ret = atomisp_try_fmt(vdev, f, NULL);
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_s_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	mutex_lock(&isp->mutex);
	ret = atomisp_set_fmt(vdev, f);
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_s_fmt_file(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	mutex_lock(&isp->mutex);
	ret = atomisp_set_fmt_file(vdev, f);
	mutex_unlock(&isp->mutex);
	return ret;
}

/*
 * is_resolution_supported - Check whether resolution is supported
 * @width: check resolution width
 * @height: check resolution height
 *
 * Return 1 on supported or 0 otherwise.
*/
static int is_resolution_supported(u32 width, u32 height)
{
	if ((width > ATOM_ISP_MIN_WIDTH) && (width <= ATOM_ISP_MAX_WIDTH) &&
	    (height > ATOM_ISP_MIN_HEIGHT) && (height <= ATOM_ISP_MAX_HEIGHT)) {
		if (!(width % ATOM_ISP_STEP_WIDTH) &&
		    !(height % ATOM_ISP_STEP_HEIGHT))
			return 1;
	}

	return 0;
}

/*
 * This ioctl allows applications to enumerate all frame intervals that the
 * device supports for the given pixel format and frame size.
 *
 * framerate =  1 / frameintervals
 */
static int atomisp_enum_frameintervals(struct file *file, void *fh,
	struct v4l2_frmivalenum *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	if (arg->index != 0)
		return -EINVAL;

	if (!atomisp_get_format_bridge(arg->pixel_format))
		return -EINVAL;

	if (!is_resolution_supported(arg->width, arg->height))
		return -EINVAL;

	mutex_lock(&isp->mutex);
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
		video, enum_frameintervals, arg);

	if (ret) {
		/* set the FPS to default 15*/
		arg->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		arg->discrete.numerator = 1;
		arg->discrete.denominator = 15;
	}
	mutex_unlock(&isp->mutex);

	return 0;
}

/*
 * Free videobuffer buffer priv data
 */
void atomisp_videobuf_free_buf(struct videobuf_buffer *vb)
{
	struct videobuf_vmalloc_memory *vm_mem;

	if (vb == NULL)
		return;

	vm_mem = vb->priv;
	if (vm_mem && vm_mem->vaddr) {
		ia_css_frame_free(vm_mem->vaddr);
		vm_mem->vaddr = NULL;
	}
}

/*
 * this function is used to free video buffer queue
 */
static void atomisp_videobuf_free_queue(struct videobuf_queue *q)
{
	int i;

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		atomisp_videobuf_free_buf(q->bufs[i]);
		kfree(q->bufs[i]);
		q->bufs[i] = NULL;
	}
}

int atomisp_alloc_css_stat_bufs(struct atomisp_device *isp)
{
	struct atomisp_s3a_buf *s3a_buf = NULL;
	struct atomisp_dvs_buf *dvs_buf = NULL;
	/* 2 css pipes consuming 3a buffers */
	int count = ATOMISP_CSS_Q_DEPTH * 2;

	if (!list_empty(&isp->s3a_stats) && !list_empty(&isp->dvs_stats))
		return 0;

	while (count--) {
		if (isp->params.curr_grid_info.s3a_grid.enable) {
			v4l2_dbg(5, dbg_level, &atomisp_dev,
				 "allocating %d 3a buffers\n", count);
			s3a_buf = kzalloc(sizeof(struct atomisp_s3a_buf), GFP_KERNEL);
			if (!s3a_buf) {
				v4l2_err(&atomisp_dev, "s3a stat buf alloc failed\n");
				goto s3a_error;
			}

			s3a_buf->s3a_stat =
			    ia_css_isp_3a_statistics_allocate(&isp->params.curr_grid_info.s3a_grid);
			if (!s3a_buf->s3a_stat) {
				v4l2_err(&atomisp_dev,
					 "3a stat buf allocation failed\n");
				kfree(s3a_buf);
				goto s3a_error;
			}
			list_add_tail(&s3a_buf->list, &isp->s3a_stats);
		}

		if (isp->params.curr_grid_info.dvs_grid.enable) {
			v4l2_dbg(5, dbg_level, &atomisp_dev,
				 "allocating %d dvs buffers\n", count);
			dvs_buf = kzalloc(sizeof(struct atomisp_dvs_buf), GFP_KERNEL);
			if (!dvs_buf) {
				v4l2_err(&atomisp_dev, "dis stat buf alloc failed\n");
				goto dvs_error;
			}

			dvs_buf->dvs_stat =
			    ia_css_isp_dvs_statistics_allocate(&isp->params.curr_grid_info.dvs_grid);
			if (!dvs_buf->dvs_stat) {
				v4l2_err(&atomisp_dev,
					 "dvs stat buf allocation failed\n");
				kfree(dvs_buf);
				goto dvs_error;
			}
			list_add_tail(&dvs_buf->list, &isp->dvs_stats);
		}

	}

	return 0;
dvs_error:
	v4l2_err(&atomisp_dev,
		    "failed to allocate statistics buffers\n");

	while (!list_empty(&isp->dvs_stats)) {
		dvs_buf = list_entry(isp->dvs_stats.next,
				     struct atomisp_dvs_buf, list);
		ia_css_isp_dvs_statistics_free(dvs_buf->dvs_stat);
		list_del(&dvs_buf->list);
		kfree(dvs_buf);
	}
s3a_error:
	while (!list_empty(&isp->s3a_stats)) {
		s3a_buf = list_entry(isp->s3a_stats.next,
					   struct atomisp_s3a_buf, list);
		ia_css_isp_3a_statistics_free(s3a_buf->s3a_stat);
		list_del(&s3a_buf->list);
		kfree(s3a_buf);
	}
	return -ENOMEM;
}

/*
 * Initiate Memory Mapping or User Pointer I/O
 */
int __atomisp_reqbufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct ia_css_frame_info frame_info;
	struct ia_css_frame *frame;
	struct videobuf_vmalloc_memory *vm_mem;
	uint16_t source_pad = atomisp_subdev_source_pad(vdev);
	int ret = 0, i = 0;

	if (req->count == 0) {
		mutex_lock(&pipe->capq.vb_lock);
		if (!list_empty(&pipe->capq.stream)) {
			videobuf_queue_cancel(&pipe->capq);
		}
		atomisp_videobuf_free_queue(&pipe->capq);
		mutex_unlock(&pipe->capq.vb_lock);
		return 0;
	}

	ret = videobuf_reqbufs(&pipe->capq, req);
	if (ret)
		return ret;

	atomisp_alloc_css_stat_bufs(isp);

	/*
	 * for user pointer type, buffers are not really allcated here,
	 * buffers are setup in QBUF operation through v4l2_buffer structure
	 */
	if (req->memory == V4L2_MEMORY_USERPTR)
		return 0;

	ret = __get_css_frame_info(isp, source_pad, &frame_info);
	if (ret)
		return -EINVAL;

	/*
	 * Allocate the real frame here for selected node using our
	 * memory management function
	 */
	for (i = 0; i < req->count; i++) {
		if (ia_css_frame_allocate_from_info(&frame, &frame_info))
			goto error;
		vm_mem = pipe->capq.bufs[i]->priv;
		vm_mem->vaddr = frame;
	}

	return ret;

error:
	while (i--) {
		vm_mem = pipe->capq.bufs[i]->priv;
		ia_css_frame_free(vm_mem->vaddr);
	}

	if (isp->vf_frame)
		ia_css_frame_free(isp->vf_frame);

	return -ENOMEM;
}

int atomisp_reqbufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s.\n", __func__);
	mutex_lock(&isp->mutex);
	ret = __atomisp_reqbufs(file, fh, req);
	mutex_unlock(&isp->mutex);
	v4l2_dbg(5, dbg_level, &atomisp_dev, "<%s.\n", __func__);

	return ret;
}

static int atomisp_reqbufs_file(struct file *file, void *fh,
		struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	if (req->count == 0) {
		mutex_lock(&pipe->outq.vb_lock);
		atomisp_videobuf_free_queue(&pipe->outq);
		mutex_unlock(&pipe->outq.vb_lock);
		return 0;
	}

	return videobuf_reqbufs(&pipe->outq, req);
}

/* application query the status of a buffer */
static int atomisp_querybuf(struct file *file, void *fh,
	struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return videobuf_querybuf(&pipe->capq, buf);
}

static int atomisp_querybuf_file(struct file *file, void *fh,
				struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return videobuf_querybuf(&pipe->outq, buf);
}

/*
 * Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) or
 * filled (output) buffer in the drivers incoming queue.
 */
static int atomisp_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	static const int NOFLUSH_FLAGS = V4L2_BUF_FLAG_NO_CACHE_INVALIDATE |
					 V4L2_BUF_FLAG_NO_CACHE_CLEAN;
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct videobuf_buffer *vb;
	struct videobuf_vmalloc_memory *vm_mem;
	struct ia_css_frame_info frame_info;
	struct ia_css_frame *handle = NULL;
	u32 length;
	u32 pgnr;
	int ret = 0;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s.\n", __func__);
	mutex_lock(&isp->mutex);
	if (isp->isp_fatal_error) {
		ret = -EIO;
		goto error;
	}

	if (isp->streaming == ATOMISP_DEVICE_STREAMING_STOPPING) {
		v4l2_err(&atomisp_dev, "ISP ERROR\n");
		ret = -EIO;
		goto error;
	}

	if (!buf || buf->index >= VIDEO_MAX_FRAME ||
		!pipe->capq.bufs[buf->index]) {
		v4l2_err(&atomisp_dev,
			    "Invalid index for qbuf.\n");
		ret = -EINVAL;
		goto error;
	}

	/*
	 * For userptr type frame, we convert user space address to physic
	 * address and reprograme out page table properly
	 */
	if (buf->memory == V4L2_MEMORY_USERPTR) {
		struct hrt_userbuffer_attr attributes;
		vb = pipe->capq.bufs[buf->index];
		vm_mem = vb->priv;
		if (!vm_mem) {
			ret = -EINVAL;
			goto error;
		}

		length = vb->bsize;
		pgnr = (length + (PAGE_SIZE - 1)) >> PAGE_SHIFT;

		if (vb->baddr == buf->m.userptr && vm_mem->vaddr)
			goto done;

		if (__get_css_frame_info(isp, atomisp_subdev_source_pad(vdev),
					 &frame_info)) {
			ret = -EIO;
			goto error;
		}

		attributes.pgnr = pgnr;
#ifdef CONFIG_ION
		attributes.type = buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_ION
					? HRT_USR_ION : HRT_USR_PTR;
#else
		attributes.type = HRT_USR_PTR;
#endif
		ret = ia_css_frame_map(&handle, &frame_info,(void *)buf->m.userptr,
				       0, &attributes);
		if (ret != IA_CSS_SUCCESS) {
			dev_err(isp->dev, "Failed to map user buffer\n");
			ret = -ENOMEM;
			goto error;
		}

		if (vm_mem->vaddr) {
			mutex_lock(&pipe->capq.vb_lock);
			ia_css_frame_free(vm_mem->vaddr);
			vm_mem->vaddr = NULL;
			vb->state = VIDEOBUF_NEEDS_INIT;
			mutex_unlock(&pipe->capq.vb_lock);
		}

		vm_mem->vaddr = handle;

		buf->flags &= ~V4L2_BUF_FLAG_MAPPED;
		buf->flags |= V4L2_BUF_FLAG_QUEUED;
		buf->flags &= ~V4L2_BUF_FLAG_DONE;

	} else if (buf->memory == V4L2_MEMORY_MMAP) {
		buf->flags |= V4L2_BUF_FLAG_MAPPED;
		buf->flags |= V4L2_BUF_FLAG_QUEUED;
		buf->flags &= ~V4L2_BUF_FLAG_DONE;
	}

done:
	if (!((buf->flags & NOFLUSH_FLAGS) == NOFLUSH_FLAGS))
		wbinvd();

	ret = videobuf_qbuf(&pipe->capq, buf);
	if (ret)
		goto error;

	/* TODO: do this better, not best way to queue to css */
	if (isp->streaming == ATOMISP_DEVICE_STREAMING_ENABLED) {
		atomisp_qbuffers_to_css(isp);

		if (!timer_pending(&isp->wdt) && atomisp_buffers_queued(isp))
			mod_timer(&isp->wdt, jiffies + isp->wdt_duration);
	}
	mutex_unlock(&isp->mutex);

	dev_dbg(isp->dev, "qbuf buffer %d (%s)\n", buf->index, vdev->name);

	return ret;

error:
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_qbuf_file(struct file *file, void *fh,
					struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	mutex_lock(&isp->mutex);
	if (isp->isp_fatal_error) {
		ret = -EIO;
		goto error;
	}

	if (!buf || buf->index >= VIDEO_MAX_FRAME ||
		!pipe->outq.bufs[buf->index]) {
		v4l2_err(&atomisp_dev,
			    "Invalid index for qbuf.\n");
		ret = -EINVAL;
		goto error;
	}

	if (buf->memory != V4L2_MEMORY_MMAP) {
		v4l2_err(&atomisp_dev, "Unsupported memory method\n");
		ret = -EINVAL;
		goto error;
	}

	if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev, "Unsupported buffer type\n");
		ret = -EINVAL;
		goto error;
	}
	mutex_unlock(&isp->mutex);

	return videobuf_qbuf(&pipe->outq, buf);

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

/*
 * Applications call the VIDIOC_DQBUF ioctl to dequeue a filled (capturing) or
 * displayed (output buffer)from the driver's outgoing queue
 */
static int atomisp_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret = 0;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s.\n", __func__);
	mutex_lock(&isp->mutex);

	if (isp->isp_fatal_error) {
		mutex_unlock(&isp->mutex);
		return -EIO;
	}

	if (isp->streaming == ATOMISP_DEVICE_STREAMING_STOPPING) {
		mutex_unlock(&isp->mutex);
		v4l2_err(&atomisp_dev, "ISP ERROR\n");
		return -EIO;
	}

	mutex_unlock(&isp->mutex);

	ret = videobuf_dqbuf(&pipe->capq, buf, file->f_flags & O_NONBLOCK);
	if (ret) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
				"<%s: %d\n", __func__, ret);
		return ret;
	}
	mutex_lock(&isp->mutex);
	buf->bytesused = pipe->pix.sizeimage;
	buf->reserved = isp->frame_status[buf->index];
	mutex_unlock(&isp->mutex);

	dev_dbg(isp->dev, "dqbuf buffer %d (%s)\n", buf->index, vdev->name);

	return 0;
}

enum ia_css_pipe_id atomisp_get_css_pipe_id(struct atomisp_device *isp)
{
	if (isp->isp_subdev.continuous_mode->val &&
	    isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO)
		return IA_CSS_PIPE_ID_PREVIEW;

	if (!isp->isp_subdev.enable_vfpp->val)
		return IA_CSS_PIPE_ID_CAPTURE;

	switch (isp->isp_subdev.run_mode->val) {
	case ATOMISP_RUN_MODE_PREVIEW:
		return IA_CSS_PIPE_ID_PREVIEW;
	case ATOMISP_RUN_MODE_VIDEO:
		return IA_CSS_PIPE_ID_VIDEO;
	case ATOMISP_RUN_MODE_STILL_CAPTURE:
		/* fall through */
	default:
		return IA_CSS_PIPE_ID_CAPTURE;
	}
}

static unsigned int atomisp_sensor_start_stream(struct atomisp_device *isp)
{
	if (!isp->isp_subdev.enable_vfpp->val)
		return 1;

	if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO ||
	    (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_STILL_CAPTURE &&
	     !atomisp_is_mbuscode_raw(
		     isp->isp_subdev.fmt[
			     isp->isp_subdev.capture_pad].fmt.code) &&
	     !isp->isp_subdev.continuous_mode->val))
		return 2;
	else
		return 1;
}

/*
 * This ioctl start the capture during streaming I/O.
 */
static int atomisp_streamon(struct file *file, void *fh,
	enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	unsigned int sensor_start_stream;
	int ret = 0;
	unsigned long irqflags;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_dbg(isp->dev, "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	if (isp->streaming == ATOMISP_DEVICE_STREAMING_STOPPING) {
		ret = -EBUSY;
		goto out;
	}

	if (pipe->capq.streaming)
		goto out;

	/*
	 * The number of streaming video nodes is based on which
	 * binary is going to be run.
	 */
	sensor_start_stream = atomisp_sensor_start_stream(isp);

	spin_lock_irqsave(&pipe->irq_lock, irqflags);
	if (list_empty(&(pipe->capq.stream))) {
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		dev_dbg(isp->dev, "no buffer in the queue\n");
		ret = -EINVAL;
		goto out;
	}
	spin_unlock_irqrestore(&pipe->irq_lock, irqflags);

	ret = videobuf_streamon(&pipe->capq);
	if (ret)
		goto out;

	if (atomisp_streaming_count(isp) > sensor_start_stream) {
		/* trigger still capture */
		if (isp->isp_subdev.continuous_mode->val &&
		    atomisp_subdev_source_pad(vdev)
		    == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE &&
		    isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
			if (isp->delayed_init != ATOMISP_DELAYED_INIT_DONE) {
				flush_work_sync(&isp->delayed_init_work);
				mutex_unlock(&isp->mutex);
				if (wait_for_completion_interruptible(
						&isp->init_done) != 0)
					return -ERESTARTSYS;
				mutex_lock(&isp->mutex);
			}
			ret = ia_css_stream_capture(
					isp->css2_basis.stream,
					isp->params.offline_parm.num_captures,
					isp->params.offline_parm.skip_frames,
					isp->params.offline_parm.offset);
			if (ret) {
				ret = -EINVAL;
				goto out;
			}
		}
		atomisp_qbuffers_to_css(isp);
		goto out;
	}

	if (isp->streaming == ATOMISP_DEVICE_STREAMING_ENABLED) {
		atomisp_qbuffers_to_css(isp);
		goto start_sensor;
	}

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_ISP2400 && isp->need_gfx_throttle) {
		/*
		 * As per h/w architect and ECO 697611 we need to throttle the
		 * GFX performance (freq) while camera is up to prevent peak
		 * current issues. this is done by setting the camera busy bit.
		 */
		msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, MFLD_OR1);
		msg_ret |= 0x100;
		intel_mid_msgbus_write32(PUNIT_PORT, MFLD_OR1, msg_ret);
	}
#endif

	ret = atomisp_acc_load_extensions(isp);
	if (ret < 0) {
		dev_err(isp->dev, "acc extension failed to load\n");
		goto out;
	}

	ret = ia_css_start(isp, false);
	if (ret != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "ia_css_start fails: %d\n", ret);
		ret = -EINVAL;
		goto out;
	}
	if (isp->isp_subdev.continuous_mode->val &&
	    isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO) {
		INIT_COMPLETION(isp->init_done);
		isp->delayed_init = ATOMISP_DELAYED_INIT_QUEUED;
		queue_work(isp->delayed_init_workq, &isp->delayed_init_work);
	}

	/* Make sure that update_isp_params is called at least once.*/
	isp->params.css_update_params_needed = true;
	isp->streaming = ATOMISP_DEVICE_STREAMING_ENABLED;
	atomic_set(&isp->sof_count, -1);
	atomic_set(&isp->sequence, -1);
	atomic_set(&isp->sequence_temp, -1);
	atomic_set(&isp->wdt_count, 0);
	if (isp->sw_contex.file_input)
		isp->wdt_duration = ATOMISP_ISP_FILE_TIMEOUT_DURATION;
	else
		isp->wdt_duration = ATOMISP_ISP_TIMEOUT_DURATION;
	if (atomisp_buffers_queued(isp))
		mod_timer(&isp->wdt, jiffies + isp->wdt_duration);
	isp->fr_status = ATOMISP_FRAME_STATUS_OK;
	isp->sw_contex.invalid_frame = false;
	isp->params.dvs_proj_data_valid = false;

	atomisp_qbuffers_to_css(isp);

	/* Only start sensor when the last streaming instance started */
	if (atomisp_streaming_count(isp) < sensor_start_stream)
		goto out;

start_sensor:
	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 1);
		isp->params.num_flash_frames = 0;
		isp->params.flash_state = ATOMISP_FLASH_IDLE;
		atomisp_setup_flash(isp);
	}

	if (!isp->sw_contex.file_input) {
		ia_css_irq_enable(IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF,
					true);

		atomisp_set_term_en_count(isp);

		if (IS_ISP2400 &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_AUTO) < 0)
			dev_dbg(isp->dev, "dfs failed!\n");
	} else {
		if (IS_MRFLD &&
			atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_MAX) < 0)
			dev_dbg(isp->dev, "dfs failed!\n");
	}

	/* stream on the sensor */
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       video, s_stream, 1);
	if (ret) {
		atomisp_reset(isp);
		ret = -EINVAL;
	}

out:
	mutex_unlock(&isp->mutex);
	return ret;
}

int __atomisp_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_video_pipe *capture_pipe = NULL;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *preview_pipe = NULL;
	struct videobuf_buffer *vb, *_vb;
	int ret;
	unsigned long flags;
	bool first_streamoff = false;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif

	BUG_ON(!mutex_is_locked(&isp->mutex));
	BUG_ON(!mutex_is_locked(&isp->streamoff_mutex));

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_dbg(isp->dev, "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	/*
	 * do only videobuf_streamoff for capture & vf pipes in
	 * case of continuous capture
	 */
	if (isp->isp_subdev.run_mode->val != ATOMISP_RUN_MODE_VIDEO &&
	    isp->isp_subdev.continuous_mode->val &&
	    atomisp_subdev_source_pad(vdev)
	    != ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW) {

		/* stop continuous still capture if needed */
		if (atomisp_subdev_source_pad(vdev)
		    == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE &&
		    isp->params.offline_parm.num_captures == -1)
			ia_css_stream_capture(isp->css2_basis.stream,
					0, 0, 0);
		/*
		 * Currently there is no way to flush buffers queued to css.
		 * When doing videobuf_streamoff, active buffers will be
		 * marked as VIDEOBUF_NEEDS_INIT. HAL will be able to use
		 * these buffers again, and these buffers might be queued to
		 * css more than once! Warn here, if HAL has not dequeued all
		 * buffers back before calling streamoff.
		 */
		if (pipe->buffers_in_css != 0)
			WARN(1, "%s: buffers of vdev %s still in CSS!\n",
			     __func__, pipe->vdev.name);

		return videobuf_streamoff(&pipe->capq);
	}

	if (!pipe->capq.streaming)
		return 0;

	spin_lock_irqsave(&isp->lock, flags);
	if (isp->streaming == ATOMISP_DEVICE_STREAMING_ENABLED) {
		isp->streaming = ATOMISP_DEVICE_STREAMING_STOPPING;
		first_streamoff = true;
	}
	spin_unlock_irqrestore(&isp->lock, flags);

	if (first_streamoff) {
		mutex_unlock(&isp->mutex);
		del_timer_sync(&isp->wdt);
		cancel_work_sync(&isp->wdt_work);

		/*
		 * must stop sending pixels into GP_FIFO before stop
		 * the pipeline.
		 */
		if (isp->sw_contex.file_input)
			v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
					video, s_stream, 0);

		mutex_lock(&isp->mutex);
		atomisp_acc_unload_extensions(isp);
	}

	spin_lock_irqsave(&isp->lock, flags);
	if (atomisp_streaming_count(isp) == 1)
		isp->streaming = ATOMISP_DEVICE_STREAMING_DISABLED;
	spin_unlock_irqrestore(&isp->lock, flags);

	if (!first_streamoff) {
		ret = videobuf_streamoff(&pipe->capq);
		if (ret)
			return ret;
		goto stopsensor;
	}

	atomisp_clear_css_buffer_counters(isp);

	if (!isp->sw_contex.file_input)
		ia_css_irq_enable(IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF,
					false);

	if (isp->delayed_init == ATOMISP_DELAYED_INIT_QUEUED) {
		cancel_work_sync(&isp->delayed_init_work);
		isp->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;
	}

	ret = ia_css_stop(isp, false);
	if (ret) {
		v4l2_err(&atomisp_dev, "stop css failed, ret=%d.\n", ret);
		return ret;
	}

	/* cancel work queue*/
	if (isp->isp_subdev.video_out_capture.users) {
		capture_pipe = &isp->isp_subdev.video_out_capture;
		wake_up_interruptible(&capture_pipe->capq.wait);
	}
	if (isp->isp_subdev.video_out_vf.users) {
		vf_pipe = &isp->isp_subdev.video_out_vf;
		wake_up_interruptible(&vf_pipe->capq.wait);
	}
	if (isp->isp_subdev.video_out_preview.users) {
		preview_pipe = &isp->isp_subdev.video_out_preview;
		wake_up_interruptible(&preview_pipe->capq.wait);
	}
	ret = videobuf_streamoff(&pipe->capq);
	if (ret)
		return ret;

	/* cleanup css here */
	/* no need for this, as ISP will be reset anyway */
	/*atomisp_flush_bufs_in_css(isp);*/

	spin_lock_irqsave(&pipe->irq_lock, flags);
	list_for_each_entry_safe(vb, _vb, &pipe->activeq, queue) {
		vb->state = VIDEOBUF_PREPARED;
		list_del(&vb->queue);
	}
	spin_unlock_irqrestore(&pipe->irq_lock, flags);

stopsensor:
	if (atomisp_streaming_count(isp) + 1
	    != atomisp_sensor_start_stream(isp))
		return 0;

	if (!isp->sw_contex.file_input)
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 0);

	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 0);
		isp->params.num_flash_frames = 0;
		isp->params.flash_state = ATOMISP_FLASH_IDLE;
	}

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_ISP2400 && isp->need_gfx_throttle) {
		/* Free camera_busy bit */
		msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, MFLD_OR1);
		msg_ret &= ~0x100;
		intel_mid_msgbus_write32(PUNIT_PORT, MFLD_OR1, msg_ret);
	}
#endif

	if (IS_ISP2400 && atomisp_freq_scaling(isp, ATOMISP_DFS_MODE_LOW))
		v4l2_warn(&atomisp_dev, "DFS failed.\n");
	/*
	 * ISP work around, need to reset isp
	 * Is it correct time to reset ISP when first node does streamoff?
	 */
	if (isp->sw_contex.power_state == ATOM_ISP_POWER_UP)
		atomisp_reset(isp);

	return ret;
}

static int atomisp_streamoff(struct file *file, void *fh,
			     enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int rval;

	mutex_lock(&isp->streamoff_mutex);
	mutex_lock(&isp->mutex);
	rval = __atomisp_streamoff(file, fh, type);
	mutex_unlock(&isp->mutex);
	mutex_unlock(&isp->streamoff_mutex);

	return rval;
}

/*
 * To get the current value of a control.
 * applications initialize the id field of a struct v4l2_control and
 * call this ioctl with a pointer to this structure
 */
static int atomisp_g_ctrl(struct file *file, void *fh,
	struct v4l2_control *control)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int i, ret = -EINVAL;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret)
		return ret;

	mutex_lock(&isp->mutex);

	switch (control->id) {
	case V4L2_CID_IRIS_ABSOLUTE:
	case V4L2_CID_EXPOSURE_ABSOLUTE:
	case V4L2_CID_FNUMBER_ABSOLUTE:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_2A_STATUS:
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_SCENE_MODE:
	case V4L2_CID_ISO_SENSITIVITY:
	case V4L2_CID_EXPOSURE_METERING:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_SHARPNESS:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, g_ctrl, control);
	case V4L2_CID_COLORFX:
		ret = atomisp_color_effect(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION:
		ret = atomisp_bad_pixel(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC:
		ret = atomisp_gdc_cac(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_VIDEO_STABLIZATION:
		ret = atomisp_video_stable(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_FIXED_PATTERN_NR:
		ret = atomisp_fixed_pattern(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION:
		ret = atomisp_false_color(isp, 0, &control->value);
		break;
	case V4L2_CID_ATOMISP_LOW_LIGHT:
		ret = atomisp_low_light(isp, 0, &control->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&isp->mutex);
	return ret;
}

/*
 * To change the value of a control.
 * applications initialize the id and value fields of a struct v4l2_control
 * and call this ioctl.
 */
static int atomisp_s_ctrl(struct file *file, void *fh,
			  struct v4l2_control *control)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int i, ret = -EINVAL;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret)
		return ret;

	mutex_lock(&isp->mutex);
	switch (control->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_SCENE_MODE:
	case V4L2_CID_ISO_SENSITIVITY:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_POWER_LINE_FREQUENCY:
	case V4L2_CID_EXPOSURE_METERING:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_SHARPNESS:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, s_ctrl, control);
	case V4L2_CID_COLORFX:
		ret = atomisp_color_effect(isp, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_BAD_PIXEL_DETECTION:
		ret = atomisp_bad_pixel(isp, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_POSTPROCESS_GDC_CAC:
		ret = atomisp_gdc_cac(isp, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_VIDEO_STABLIZATION:
		ret = atomisp_video_stable(isp, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_FIXED_PATTERN_NR:
		ret = atomisp_fixed_pattern(isp, 1, &control->value);
		break;
	case V4L2_CID_ATOMISP_FALSE_COLOR_CORRECTION:
		ret = atomisp_false_color(isp, 1, &control->value);
		break;
	case V4L2_CID_REQUEST_FLASH:
		ret = atomisp_flash_enable(isp, control->value);
		break;
	case V4L2_CID_ATOMISP_LOW_LIGHT:
		ret = atomisp_low_light(isp, 1, &control->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&isp->mutex);
	return ret;
}
/*
 * To query the attributes of a control.
 * applications set the id field of a struct v4l2_queryctrl and call the
 * this ioctl with a pointer to this structure. The driver fills
 * the rest of the structure.
 */
static int atomisp_queryctl(struct file *file, void *fh,
			    struct v4l2_queryctrl *qc)
{
	int i, ret = -EINVAL;

	if (qc->id & V4L2_CTRL_FLAG_NEXT_CTRL)
		return ret;

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == qc->id) {
			memcpy(qc, &ci_v4l2_controls[i],
			       sizeof(struct v4l2_queryctrl));
			qc->reserved[0] = 0;
			ret = 0;
			break;
		}
	}
	if (ret != 0)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	return ret;
}

static int atomisp_camera_g_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		switch (ctrl.id) {
		case V4L2_CID_EXPOSURE_ABSOLUTE:
		case V4L2_CID_IRIS_ABSOLUTE:
		case V4L2_CID_FNUMBER_ABSOLUTE:
		case V4L2_CID_BIN_FACTOR_HORZ:
		case V4L2_CID_BIN_FACTOR_VERT:
			/*
			 * Exposure related control will be handled by sensor
			 * driver
			 */
			ret = v4l2_subdev_call(isp->inputs
					       [isp->input_curr].camera,
					       core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_FOCUS_ABSOLUTE:
		case V4L2_CID_FOCUS_RELATIVE:
		case V4L2_CID_FOCUS_STATUS:
		case V4L2_CID_FOCUS_AUTO:
			if (isp->inputs[isp->input_curr].motor)
				ret = v4l2_subdev_call(
					isp->inputs[isp->input_curr].motor,
					core, g_ctrl, &ctrl);
			else
				ret = v4l2_subdev_call(
					isp->inputs[isp->input_curr].camera,
					core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_FLASH_STATUS:
		case V4L2_CID_FLASH_INTENSITY:
		case V4L2_CID_FLASH_TORCH_INTENSITY:
		case V4L2_CID_FLASH_INDICATOR_INTENSITY:
		case V4L2_CID_FLASH_TIMEOUT:
		case V4L2_CID_FLASH_STROBE:
		case V4L2_CID_FLASH_MODE:
			if (isp->flash)
				ret = v4l2_subdev_call(
					isp->flash, core, g_ctrl, &ctrl);
			break;
		case V4L2_CID_ZOOM_ABSOLUTE:
			mutex_lock(&isp->mutex);
			ret = atomisp_digital_zoom(isp, 0, &ctrl.value);
			mutex_unlock(&isp->mutex);
			break;
		case V4L2_CID_G_SKIP_FRAMES:
			ret = v4l2_subdev_call(
				isp->inputs[isp->input_curr].camera,
				sensor, g_skip_frames, (u32 *)&ctrl.value);
			break;
		default:
			ret = -EINVAL;
		}

		if (ret) {
			c->error_idx = i;
			break;
		}
		c->controls[i].value = ctrl.value;
	}
	return ret;
}

/* This ioctl allows the application to get multiple controls by class */
static int atomisp_g_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct v4l2_control ctrl;
	int i, ret = 0;

	/* input_lock is not need for the Camera releated IOCTLs
	 * The input_lock downgrade the FPS of 3A*/
	ret = atomisp_camera_g_ext_ctrls(file, fh, c);
	if (ret != -EINVAL)
		return ret;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		ret = atomisp_g_ctrl(file, fh, &ctrl);
		c->controls[i].value = ctrl.value;
		if (ret) {
			c->error_idx = i;
			break;
		}
	}
	return ret;
}

static int atomisp_camera_s_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_control ctrl;
	int i;
	int ret = 0;

	for (i = 0; i < c->count; i++) {
		struct v4l2_ctrl *ctr;

		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		switch (ctrl.id) {
		case V4L2_CID_EXPOSURE_ABSOLUTE:
		case V4L2_CID_IRIS_ABSOLUTE:
		case V4L2_CID_FNUMBER_ABSOLUTE:
		case V4L2_CID_VCM_TIMEING:
		case V4L2_CID_VCM_SLEW:
		case V4L2_CID_TEST_PATTERN:
			ret = v4l2_subdev_call(
				isp->inputs[isp->input_curr].camera,
				core, s_ctrl, &ctrl);
			break;
		case V4L2_CID_FOCUS_ABSOLUTE:
		case V4L2_CID_FOCUS_RELATIVE:
		case V4L2_CID_FOCUS_STATUS:
		case V4L2_CID_FOCUS_AUTO:
			if (isp->inputs[isp->input_curr].motor)
				ret = v4l2_subdev_call(
					isp->inputs[isp->input_curr].motor,
					core, s_ctrl, &ctrl);
			else
				ret = v4l2_subdev_call(
					isp->inputs[isp->input_curr].camera,
					core, s_ctrl, &ctrl);
			break;
		case V4L2_CID_FLASH_STATUS:
		case V4L2_CID_FLASH_INTENSITY:
		case V4L2_CID_FLASH_TORCH_INTENSITY:
		case V4L2_CID_FLASH_INDICATOR_INTENSITY:
		case V4L2_CID_FLASH_TIMEOUT:
		case V4L2_CID_FLASH_STROBE:
		case V4L2_CID_FLASH_MODE:
			mutex_lock(&isp->mutex);
			if (isp->flash) {
				ret = v4l2_subdev_call(isp->flash,
					core, s_ctrl, &ctrl);
				/* When flash mode is changed we need to reset
				 * flash state */
				if (ctrl.id == V4L2_CID_FLASH_MODE) {
					isp->params.flash_state = ATOMISP_FLASH_IDLE;
					isp->params.num_flash_frames = 0;
				}
			}
			mutex_unlock(&isp->mutex);
			break;
		case V4L2_CID_ZOOM_ABSOLUTE:
			mutex_lock(&isp->mutex);
			ret = atomisp_digital_zoom(isp, 1, &ctrl.value);
			mutex_unlock(&isp->mutex);
			break;
		default:
			ctr = v4l2_ctrl_find(&isp->isp_subdev.ctrl_handler,
					     ctrl.id);
			if (ctr)
				ret = v4l2_ctrl_s_ctrl(ctr, ctrl.value);
			else
				ret = -EINVAL;
		}

		if (ret) {
			c->error_idx = i;
			break;
		}
		c->controls[i].value = ctrl.value;
	}
	return ret;
}

/* This ioctl allows the application to set multiple controls by class */
static int atomisp_s_ext_ctrls(struct file *file, void *fh,
	struct v4l2_ext_controls *c)
{
	struct v4l2_control ctrl;
	int i, ret = 0;

	/* input_lock is not need for the Camera releated IOCTLs
	 * The input_lock downgrade the FPS of 3A*/
	ret = atomisp_camera_s_ext_ctrls(file, fh, c);
	if (ret != -EINVAL)
		return ret;

	for (i = 0; i < c->count; i++) {
		ctrl.id = c->controls[i].id;
		ctrl.value = c->controls[i].value;
		ret = atomisp_s_ctrl(file, fh, &ctrl);
		c->controls[i].value = ctrl.value;
		if (ret) {
			c->error_idx = i;
			break;
		}
	}
	return ret;
}

/*
 * vidioc_g/s_param are used to switch isp running mode
 */
static int atomisp_g_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	parm->parm.capture.capturemode = isp->isp_subdev.run_mode->val;
	mutex_unlock(&isp->mutex);

	return 0;
}

static int atomisp_s_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int mode;
	int rval;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);

	switch (parm->parm.capture.capturemode) {
	case CI_MODE_NONE: {
		struct v4l2_subdev_frame_interval fi = {0};

		fi.interval = parm->parm.capture.timeperframe;

		rval = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
					video, s_frame_interval, &fi);
		if (!rval)
			parm->parm.capture.timeperframe = fi.interval;
		goto out;
	}
	case CI_MODE_VIDEO:
		mode = ATOMISP_RUN_MODE_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		mode = ATOMISP_RUN_MODE_STILL_CAPTURE;
		break;
	case CI_MODE_CONTINUOUS:
		mode = ATOMISP_RUN_MODE_CONTINUOUS_CAPTURE;
		break;
	case CI_MODE_PREVIEW:
		mode = ATOMISP_RUN_MODE_PREVIEW;
		break;
	default:
		rval = -EINVAL;
		goto out;
	}

	rval = v4l2_ctrl_s_ctrl(isp->isp_subdev.run_mode, mode);

out:
	mutex_unlock(&isp->mutex);

	return rval == -ENOIOCTLCMD ? 0 : rval;
}

static int atomisp_s_parm_file(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type for output\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	isp->sw_contex.file_input = 1;
	mutex_unlock(&isp->mutex);

	return 0;
}

/* set default atomisp ioctl value */
static long atomisp_vidioc_default(struct file *file, void *fh,
	bool valid_prio, int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int err;

	mutex_lock(&isp->mutex);
	switch (cmd) {
	case ATOMISP_IOC_G_XNR:
		err = atomisp_xnr(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_XNR:
		err = atomisp_xnr(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_NR:
		err = atomisp_nr(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_NR:
		err = atomisp_nr(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_TNR:
		err = atomisp_tnr(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_TNR:
		err = atomisp_tnr(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_BLACK_LEVEL_COMP:
		err = atomisp_black_level(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_BLACK_LEVEL_COMP:
		err = atomisp_black_level(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_EE:
		err = atomisp_ee(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_EE:
		err = atomisp_ee(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_DIS_STAT:
		err = atomisp_get_dis_stat(isp, arg);
		break;

	case ATOMISP_IOC_S_DIS_COEFS:
		err = atomisp_set_dis_coefs(isp, arg);
		break;

	case ATOMISP_IOC_S_DIS_VECTOR:
		err = atomisp_set_dis_vector(isp, arg);
		break;

	case ATOMISP_IOC_G_ISP_PARM:
		err = atomisp_param(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_PARM:
		err = atomisp_param(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_3A_STAT:
		err = atomisp_3a_stat(isp, 0, arg);
		break;

	case ATOMISP_IOC_G_ISP_GAMMA:
		err = atomisp_gamma(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GAMMA:
		err = atomisp_gamma(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_GDC_TAB:
		err = atomisp_gdc_cac_table(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GDC_TAB:
		err = atomisp_gdc_cac_table(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_MACC:
		err = atomisp_macc_table(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_MACC:
		err = atomisp_macc_table(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_BAD_PIXEL_DETECTION:
		err = atomisp_bad_pixel_param(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_BAD_PIXEL_DETECTION:
		err = atomisp_bad_pixel_param(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_FALSE_COLOR_CORRECTION:
		err = atomisp_false_color_param(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_FALSE_COLOR_CORRECTION:
		err = atomisp_false_color_param(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_CTC:
		err = atomisp_ctc(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_CTC:
		err = atomisp_ctc(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_ISP_WHITE_BALANCE:
		err = atomisp_white_balance_param(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_WHITE_BALANCE:
		err = atomisp_white_balance_param(isp, 1, arg);
		break;

	case ATOMISP_IOC_G_3A_CONFIG:
		err = atomisp_3a_config_param(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_3A_CONFIG:
		err = atomisp_3a_config_param(isp, 1, arg);
		break;

	case ATOMISP_IOC_S_ISP_FPN_TABLE:
		err = atomisp_fixed_pattern_table(isp, arg);
		break;

	case ATOMISP_IOC_ISP_MAKERNOTE:
		err = atomisp_exif_makernote(isp, arg);
		break;

	case ATOMISP_IOC_G_SENSOR_MODE_DATA:
		err = atomisp_get_sensor_mode_data(isp, arg);
		break;

	case ATOMISP_IOC_G_MOTOR_PRIV_INT_DATA:
		mutex_unlock(&isp->mutex);
		if (isp->inputs[isp->input_curr].motor)
			return v4l2_subdev_call(
					isp->inputs[isp->input_curr].motor,
					core, ioctl, cmd, arg);
		else
			return v4l2_subdev_call(
					isp->inputs[isp->input_curr].camera,
					core, ioctl, cmd, arg);

	case ATOMISP_IOC_S_EXPOSURE:
	case ATOMISP_IOC_G_SENSOR_CALIBRATION_GROUP:
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		mutex_unlock(&isp->mutex);
		return v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
					core, ioctl, cmd, arg);

	case ATOMISP_IOC_ACC_LOAD:
		err = atomisp_acc_load(isp, arg);
		break;

	case ATOMISP_IOC_ACC_LOAD_TO_PIPE:
		err = atomisp_acc_load_to_pipe(isp, arg);
		break;

	case ATOMISP_IOC_ACC_UNLOAD:
		err = atomisp_acc_unload(isp, arg);
		break;

	case ATOMISP_IOC_ACC_START:
		err = atomisp_acc_start(isp, arg);
		break;

	case ATOMISP_IOC_ACC_WAIT:
		err = atomisp_acc_wait(isp, arg);
		break;

	case ATOMISP_IOC_ACC_MAP:
		err = atomisp_acc_map(isp, arg);
		break;

	case ATOMISP_IOC_ACC_UNMAP:
		err = atomisp_acc_unmap(isp, arg);
		break;

	case ATOMISP_IOC_ACC_S_MAPPED_ARG:
		err = atomisp_acc_s_mapped_arg(isp, arg);
		break;

	case ATOMISP_IOC_S_ISP_SHD_TAB:
		err = atomisp_set_shading_table(isp, arg);
		break;

	case ATOMISP_IOC_G_ISP_GAMMA_CORRECTION:
		err = atomisp_gamma_correction(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_GAMMA_CORRECTION:
		err = atomisp_gamma_correction(isp, 1, arg);
		break;

	case ATOMISP_IOC_S_PARAMETERS:
		err = atomisp_set_parameters(isp, arg);
		break;

	case ATOMISP_IOC_S_CONT_CAPTURE_CONFIG:
		err = atomisp_offline_capture_configure(isp, arg);
		break;

	default:
		mutex_unlock(&isp->mutex);
		return -EINVAL;
	}
	mutex_unlock(&isp->mutex);
	return err;
}

const struct v4l2_ioctl_ops atomisp_ioctl_ops = {
	.vidioc_querycap = atomisp_querycap,
	.vidioc_g_chip_ident = atomisp_g_chip_ident,
	.vidioc_enum_input = atomisp_enum_input,
	.vidioc_g_input = atomisp_g_input,
	.vidioc_s_input = atomisp_s_input,
	.vidioc_queryctrl = atomisp_queryctl,
	.vidioc_s_ctrl = atomisp_s_ctrl,
	.vidioc_g_ctrl = atomisp_g_ctrl,
	.vidioc_s_ext_ctrls = atomisp_s_ext_ctrls,
	.vidioc_g_ext_ctrls = atomisp_g_ext_ctrls,
	.vidioc_enum_fmt_vid_cap = atomisp_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap = atomisp_try_fmt_cap,
	.vidioc_g_fmt_vid_cap = atomisp_g_fmt_cap,
	.vidioc_s_fmt_vid_cap = atomisp_s_fmt_cap,
	.vidioc_s_fmt_type_private = atomisp_s_fmt_cap,
	.vidioc_reqbufs = atomisp_reqbufs,
	.vidioc_querybuf = atomisp_querybuf,
	.vidioc_qbuf = atomisp_qbuf,
	.vidioc_dqbuf = atomisp_dqbuf,
	.vidioc_streamon = atomisp_streamon,
	.vidioc_streamoff = atomisp_streamoff,
	.vidioc_default = atomisp_vidioc_default,
	.vidioc_enum_frameintervals = atomisp_enum_frameintervals,
	.vidioc_s_parm = atomisp_s_parm,
	.vidioc_g_parm = atomisp_g_parm,
};

const struct v4l2_ioctl_ops atomisp_file_ioctl_ops = {
	.vidioc_querycap = atomisp_querycap,
	.vidioc_g_fmt_vid_out = atomisp_g_fmt_file,
	.vidioc_s_fmt_vid_out = atomisp_s_fmt_file,
	.vidioc_s_parm = atomisp_s_parm_file,
	.vidioc_reqbufs = atomisp_reqbufs_file,
	.vidioc_querybuf = atomisp_querybuf_file,
	.vidioc_qbuf = atomisp_qbuf_file,
};
