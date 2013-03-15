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

#include "atomisp_ioctl.h"
#include "atomisp_cmd.h"
#include "atomisp_fops.h"
#include "css/sh_css.h"
#include <css/sh_css_debug.h>
#include <asm/intel-mid.h>

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
		.id = V4L2_CID_AUTO_FOCUS_START,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.name = "single focus",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_FOCUS_AUTO,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "continuous focus",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_3A_LOCK,
		.type = V4L2_CTRL_TYPE_BITMASK,
		.name = "pause focus",
		.minimum = 0,
		.maximum = 1 << 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_FOCUS_STOP,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.name = "release focus",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_FOCUS_STATUS,
		.type = V4L2_CTRL_TYPE_BITMASK,
		.name = "focus status",
		.minimum = 0,
		.maximum = 3,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "target image luminance",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_ABSOLUTE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "exposure",
		.minimum = 0x0,
		.maximum = 0xffff,
		.step = 0x01,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_AUTO_EXPOSURE_BIAS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "exposure bias",
		.minimum = 0x0,
		.maximum = 0xffff,
		.step = 0x01,
		.default_value = 0x00,
	},
	{
		.id = V4L2_CID_ISO_SENSITIVITY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "iso",
		.minimum = 0x0,
		.maximum = 0xffff,
		.step = 0x01,
		.default_value = 0x00,
	},
};
static const u32 ctrls_num = ARRAY_SIZE(ci_v4l2_controls);

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

	mutex_lock(&isp->input_lock);
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       core, g_chip_ident, chip);
	mutex_unlock(&isp->input_lock);

	if (ret)
		v4l2_err(&atomisp_dev,
			    "failed to g_chip_ident for sensor\n");
	return ret;
}

/*
 * crop capability is the max resolution both ISP and Sensor supported
 */
static int atomisp_cropcap(struct file *file, void *fh,
	struct v4l2_cropcap *cropcap)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	int ret;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev, "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	/*Only capture node supports cropcap*/
	if (!pipe->is_main)
		return 0;

	cropcap->bounds.left = 0;
	cropcap->bounds.top = 0;

	snr_mbus_fmt.code = V4L2_MBUS_FMT_FIXED;
	snr_mbus_fmt.height = ATOM_ISP_MAX_HEIGHT_TMP;
	snr_mbus_fmt.width = ATOM_ISP_MAX_WIDTH_TMP;

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       video, try_mbus_fmt, &snr_mbus_fmt);
	if (ret) {
		v4l2_err(&atomisp_dev,
			"failed to try_mbus_fmt for sensor"
			", try try_fmt\n");
	} else {
		cropcap->bounds.width = snr_mbus_fmt.width;
		cropcap->bounds.height = snr_mbus_fmt.height;
		isp->snr_max_width = snr_mbus_fmt.width;
		isp->snr_max_height = snr_mbus_fmt.height;
		isp->snr_pixelformat = snr_mbus_fmt.code;
	}

	memcpy(&cropcap->defrect, &cropcap->bounds, sizeof(struct v4l2_rect));
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}
/*
 *  FIXME:G_CROP and S_CROP are used for bayer downscaling case
 */
static int atomisp_g_crop(struct file *file, void *fh,
	struct v4l2_crop *crop)
{
	/*
	 * g_crop is used to enable bayer downscaling previously.
	 * current bayer ds is replaced by yuv ds.
	 * so remove the support of cropping.
	 */
	v4l2_err(&atomisp_dev,
		"crop is unavailable now\n");
	return -EINVAL;

}

static int atomisp_s_crop(struct file *file, void *fh,
	struct v4l2_crop *crop)
{
	/*
	 * s_crop is used to enable bayer downscaling previously.
	 * current bayer ds is replaced by yuv ds.
	 * so remove the support of cropping.
	 */
	v4l2_err(&atomisp_dev,
		"crop is unavailable now\n");
	return -EINVAL;
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
/*
 * get input are used to get current primary/secondary camera
 */
static int atomisp_g_input(struct file *file, void *fh, unsigned int *input)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	*input = isp->input_curr;

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

	if (input >= ATOM_ISP_MAX_INPUTS || input > isp->input_cnt)
		return -EINVAL;

	mutex_lock(&isp->input_lock);
	camera = isp->inputs[input].camera;
	if (!camera) {
		mutex_unlock(&isp->input_lock);
		return -EINVAL;
	}

	if ((isp->isp_subdev.video_out_vf.capq.streaming == 1) ||
	    (isp->isp_subdev.video_out_mo.capq.streaming == 1) ||
	    (isp->isp_subdev.video_in.capq.streaming == 1)) {
		v4l2_err(&atomisp_dev,
			 "ISP is still streaming, stop first\n");
		mutex_unlock(&isp->input_lock);
		return -EINVAL;
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
	if (!isp->sw_contex.file_input) {
		ret = v4l2_subdev_call(isp->inputs[input].camera,
				       core, s_power, 1);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Failed to power-on sensor\n");
			mutex_unlock(&isp->input_lock);
			return -EINVAL;
		}
		if (isp->inputs[input].motor)
			ret = v4l2_subdev_call(isp->inputs[input].motor, core,
					       init, 1);
	}

	isp->input_curr = input;

	mutex_unlock(&isp->input_lock);

	return 0;
}

static int atomisp_g_std(struct file *file, void *fh, v4l2_std_id * id)
{
	return -EINVAL;
}

static int atomisp_s_std(struct file *file, void *fh, v4l2_std_id * id)
{
	return -EINVAL;
}

static int atomisp_enum_fmt_cap(struct file *file, void *fh,
	struct v4l2_fmtdesc *f)
{
	u32 index = f->index;

	/* check buf index */
	if (index >= atomisp_output_fmts_num) {
		v4l2_err(&atomisp_dev,
			    "fmt index extends maxiumn"
			    " supported fmts number\n");
		return -EINVAL;
	}
	/* check buf type */
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	f->pixelformat = atomisp_output_fmts[index].pixelformat;
	memset(f->description, 0, sizeof(char)*32);
	strncpy(f->description, atomisp_output_fmts[index].description,
		strlen(atomisp_output_fmts[index].description));

	return 0;
}

static int atomisp_g_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);

	int ret;

	ret = atomisp_get_fmt(vdev, f);
	return ret;
}

static int atomisp_g_fmt_file(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
				"unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	memset(f, 0, sizeof(struct v4l2_format));
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	switch (isp->sw_contex.output_mode) {
	case OUTPUT_MODE_FILE:
		f->fmt.pix.width = pipe->out_fmt->width;
		f->fmt.pix.height = pipe->out_fmt->height;
		f->fmt.pix.pixelformat = pipe->out_fmt->pixelformat;
		f->fmt.pix.bytesperline = pipe->out_fmt->bytesperline;
		f->fmt.pix.sizeimage = pipe->out_fmt->imagesize;
		break;
	case OUTPUT_MODE_TEXT:
		f->fmt.pix.sizeimage = pipe->out_fmt->framesize;
		break;
	default:
		v4l2_err(&atomisp_dev, "Unspported output mode\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* This function looks up the closest available resolution. */
static int atomisp_try_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	int ret;

	ret = atomisp_try_fmt(vdev, f, NULL);
	return ret;
}

static int atomisp_s_fmt_cap(struct file *file, void *fh,
	struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret;

	mutex_lock(&isp->input_lock);
	ret = atomisp_set_fmt(vdev, f);
	mutex_unlock(&isp->input_lock);
	return ret;
}

static int atomisp_s_fmt_file(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct video_device *vdev = video_devdata(file);
	int ret;

	ret = atomisp_set_fmt_file(vdev, f);
	return ret;
}

/*
 * This ioctl allows applications to enumerate all frame sizes that the
 * device supports for the given pixel format.
 * discrete means the applicatons should increase the index until EINVAL is
 * returned.
 * stepwise means the applications only need to set pixel_format, then
 * driver will return maximum value and minimum value of frame size supported
 * and step size.
 */
static int atomisp_enum_framesizes(struct file *file, void *fh,
	struct v4l2_frmsizeenum *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct v4l2_mbus_framefmt snr_mbus_fmt;
	struct v4l2_streamparm sensor_parm;
	struct atomisp_input_subdev *input;
	unsigned int padding_w, padding_h;
	int max_width, max_height, min_width, min_height;
	int ret;
	bool same_type;
	u32 pixel_format;

	if (arg->index != 0)
		return -EINVAL;

	if (!atomisp_is_pixelformat_supported(arg->pixel_format))
		return -EINVAL;

	input = &isp->inputs[isp->input_curr];

	if (input->type != SOC_CAMERA && input->type != RAW_CAMERA)
		return -EINVAL;

	pixel_format = input->frame_size.pixel_format;
	/*
	 * only judge if the pixel format and previous pixel format are
	 * the same type
	 */
	same_type = is_pixelformat_raw(pixel_format) ==
					is_pixelformat_raw(arg->pixel_format);

	/*
	 * when frame size is requested previously, we can get the value
	 * rapidly from cache.
	 */
	if (input->frame_size.pixel_format != 0 &&
		same_type) {
		memcpy(arg, &input->frame_size, sizeof(input->frame_size));

		return 0;
	}

	/* get padding value via subdev type and requested isp pixelformat */
	if (input->type == SOC_CAMERA || (input->type == RAW_CAMERA &&
				is_pixelformat_raw(arg->pixel_format))) {
		padding_h = 0;
		padding_w = 0;
	} else {
		padding_h = pad_h;
		padding_w = pad_w;
	}

	/* setting run mode to the sensor */
	memset(&sensor_parm, 0, sizeof(sensor_parm));
	sensor_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sensor_parm.parm.capture.capturemode = CI_MODE_STILL_CAPTURE;
	v4l2_subdev_call(input->camera, video, s_parm, &sensor_parm);

	/* get the sensor max resolution supported */
	snr_mbus_fmt.height = ATOM_ISP_MAX_HEIGHT;
	snr_mbus_fmt.width = ATOM_ISP_MAX_WIDTH;

	ret = v4l2_subdev_call(input->camera, video, try_mbus_fmt,
							&snr_mbus_fmt);
	if (ret < 0)
		return ret;

	max_width = snr_mbus_fmt.width - padding_w;
	max_height = snr_mbus_fmt.height - padding_h;

	/* app vs isp */
	max_width = max_width - max_width % ATOM_ISP_STEP_WIDTH;
	max_height = max_height - max_height % ATOM_ISP_STEP_HEIGHT;

	max_width = clamp(max_width, ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
	max_height = clamp(max_height, ATOM_ISP_MIN_HEIGHT,
			   ATOM_ISP_MAX_HEIGHT);

	/* set the supported minimum resolution to sub-QCIF resolution */
	min_width = ATOM_RESOLUTION_SUBQCIF_WIDTH;
	min_height = ATOM_RESOLUTION_SUBQCIF_HEIGHT;

	/* app vs isp */
	min_width = min_width - min_width % ATOM_ISP_STEP_WIDTH;
	min_height = min_height - min_height % ATOM_ISP_STEP_HEIGHT;

	min_width = clamp(min_width, ATOM_ISP_MIN_WIDTH, ATOM_ISP_MAX_WIDTH);
	min_height = clamp(min_height, ATOM_ISP_MIN_HEIGHT,
			   ATOM_ISP_MAX_HEIGHT);

	arg->stepwise.max_width = max_width;
	arg->stepwise.max_height = max_height;
	arg->stepwise.min_width = min_width;
	arg->stepwise.min_height = min_height;
	arg->stepwise.step_width = ATOM_ISP_STEP_WIDTH;
	arg->stepwise.step_height = ATOM_ISP_STEP_HEIGHT;
	arg->type = V4L2_FRMSIZE_TYPE_STEPWISE;

	/*
	 * store frame size in particular struct of every subdev,
	 * when enumerate frame size next,we can get it rapidly.
	 */
	memcpy(&input->frame_size, arg, sizeof(*arg));

	return 0;
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

	if (!atomisp_is_pixelformat_supported(arg->pixel_format))
		return -EINVAL;

	if (!is_resolution_supported(arg->width, arg->height))
		return -EINVAL;

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
		video, enum_frameintervals, arg);

	if (ret) {
		/* set the FPS to default 15*/
		arg->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		arg->discrete.numerator = 1;
		arg->discrete.denominator = 15;
	}

	return 0;
}

/*
 * Initiate Memory Mapping or User Pointer I/O
 */
int atomisp_reqbufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct sh_css_frame_info out_info, vf_info;
	int ret = 0;

	if (req->count == 0) {
		if ((!isp->isp_subdev.video_out_vf.opened) &&
		(isp->vf_frame)) {
			sh_css_frame_free(isp->vf_frame);
			isp->vf_frame = NULL;
		}
		goto done;
	}

	if ((!pipe->is_main) && (!atomisp_is_viewfinder_support(isp)))
		return -EINVAL;

	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		if (isp->main_format->out_sh_fmt != SH_CSS_FRAME_FORMAT_RAW) {
			if (sh_css_capture_get_viewfinder_frame_info(&vf_info))
				goto error;
			if ((!isp->isp_subdev.video_out_vf.opened) &&
			    (sh_css_frame_allocate_from_info(&isp->vf_frame,
							     &vf_info)))
					goto error;
		}
		if (sh_css_capture_get_output_frame_info(&out_info))
			goto error;
		break;
	case CI_MODE_VIDEO:
		if (sh_css_video_get_viewfinder_frame_info(&vf_info))
			goto error;

		if ((!isp->isp_subdev.video_out_vf.opened) &&
		    (sh_css_frame_allocate_from_info(&isp->vf_frame, &vf_info)))
				goto error;

		if (sh_css_video_get_output_frame_info(&out_info))
			goto error;
		break;
	case CI_MODE_PREVIEW:
		if (sh_css_preview_get_output_frame_info(&out_info))
			goto error;
		break;
	default:
		return -EINVAL;
	}

	/* pipe->main_info.frame_info is void* type */
	memcpy(pipe->main_info.frame_info, &out_info,
	       sizeof(*pipe->main_info.frame_info));
	memcpy(pipe->vf_info.frame_info, &vf_info,
	       sizeof(*pipe->main_info.frame_info));

done:
	ret = vb2_reqbufs(&pipe->capq, req);
	if (ret)
		goto error;

	return 0;

error:
	if (isp->vf_frame) {
		sh_css_frame_free(isp->vf_frame);
		isp->vf_frame = NULL;
	}

	return -ENOMEM;
}

static int atomisp_reqbufs_file(struct file *file, void *fh,
		struct v4l2_requestbuffers *req)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	ret = vb2_reqbufs(&pipe->outq, req);
	if (ret)
		return ret;

	if (isp->sw_contex.output_mode == OUTPUT_MODE_TEXT)
		return 0;

	/*
	 *  TODO: Implement file input function
	 */

	return 0;
}

/* application query the status of a buffer */
static int atomisp_querybuf(struct file *file, void *fh,
	struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_querybuf(&pipe->capq, buf);
}

static int atomisp_querybuf_file(struct file *file, void *fh,
				struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_querybuf(&pipe->outq, buf);
}

/*
 * Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) or
 * filled (output) buffer in the drivers incoming queue.
 */
static int atomisp_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret = 0;

	if ((!pipe->is_main) &&
	    (!atomisp_is_viewfinder_support(isp)))
		return -EINVAL;

	if (!buf || buf->index >= VIDEO_MAX_FRAME ||
		!pipe->capq.bufs[buf->index]) {
		v4l2_err(&atomisp_dev,
			    "Invalid index for qbuf.\n");
		return -EINVAL;
	}

	/*
	 * For userptr type frame, we convert user space address to physic
	 * address and reprograme out page table properly
	 */
	if (buf->memory == V4L2_MEMORY_USERPTR) {
		/* We must stop to atomisp to remove the
		* race condition when updating the new userptr. */
		if (buf->flags & V4L2_BUF_FLAG_BUFFER_INVALID) {
			isp->sw_contex.updating_uptr = true;
			return 0;
		}
		/* Check whether need to start the atomisp_work */
		if (buf->flags & V4L2_BUF_FLAG_BUFFER_VALID) {
			isp->sw_contex.updating_uptr = false;
			wake_up_interruptible_sync(&pipe->vb_queued);
			return 0;
		}
		/*
		 * setting for frame buf type.
		 * current for ION type implementation
		 */
		if (pipe->is_main)
			pipe->main_info.type = buf->type;
		else
			pipe->vf_info.type = buf->type;
	}

	ret = vb2_prepare_buf(&pipe->capq, buf);
	if (ret)
		return ret;

	ret = vb2_qbuf(&pipe->capq, buf);
	return ret;
}

static int atomisp_qbuf_file(struct file *file, void *fh,
					struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	if (!buf || buf->index >= VIDEO_MAX_FRAME ||
		!pipe->outq.bufs[buf->index]) {
		v4l2_err(&atomisp_dev,
			    "Invalid index for qbuf.\n");
		return -EINVAL;
	}

	if (buf->memory != V4L2_MEMORY_MMAP) {
		v4l2_err(&atomisp_dev, "Unsupported memory method\n");
		return -EINVAL;
	}

	if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev, "Unsupported buffer type\n");
		return -EINVAL;
	}

	return vb2_qbuf(&pipe->outq, buf);
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

	if ((!pipe->is_main) &&
	    (!atomisp_is_viewfinder_support(isp)))
		return -EINVAL;

	if (isp->sw_contex.error) {
		v4l2_err(&atomisp_dev, "ISP ERROR\n");
		return -EINVAL;
	}

	ret = vb2_dqbuf(&pipe->capq, buf, file->f_flags & O_NONBLOCK);
	if (ret)
		return ret;
	buf->bytesused = pipe->format->out.sizeimage;
	buf->reserved = isp->frame_status[buf->index];

	return 0;
}

/*
 * This ioctl start the capture during streaming I/O.
 */
static int atomisp_streamon(struct file *file, void *fh,
	enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_streamon(&pipe->capq, type);
}

/*This ioctl stop the capture or output process during streaming I/O.*/
int atomisp_streamoff(struct file *file, void *fh,
	enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_streamoff(&pipe->capq, type);
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

	mutex_lock(&isp->input_lock);

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		mutex_unlock(&isp->input_lock);
		return ret;
	}

	switch (control->id) {
	case V4L2_CID_IRIS_ABSOLUTE:
	case V4L2_CID_EXPOSURE_ABSOLUTE:
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
	case V4L2_CID_ISO_SENSITIVITY:
	case V4L2_CID_FNUMBER_ABSOLUTE:
	case V4L2_CID_AUTO_FOCUS_STATUS:
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, g_ctrl, control);
		break;
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

	mutex_unlock(&isp->input_lock);
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

	mutex_lock(&isp->input_lock);

	for (i = 0; i < ctrls_num; i++) {
		if (ci_v4l2_controls[i].id == control->id) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		mutex_unlock(&isp->input_lock);
		return ret;
	}

	switch (control->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_POWER_LINE_FREQUENCY:
	case V4L2_CID_AUTO_FOCUS_START:
	case V4L2_CID_AUTO_FOCUS_STOP:
	case V4L2_CID_FOCUS_AUTO:
	case V4L2_CID_3A_LOCK:
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, s_ctrl, control);
		break;
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
	mutex_unlock(&isp->input_lock);
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
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int i, ret = -EINVAL;

	if (qc->id & V4L2_CTRL_FLAG_NEXT_CTRL)
		return ret;

	mutex_lock(&isp->input_lock);

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

	mutex_unlock(&isp->input_lock);
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
		case V4L2_CID_AUTO_EXPOSURE_BIAS:
		case V4L2_CID_ISO_SENSITIVITY:
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
			ret = atomisp_digital_zoom(isp, 0, &ctrl.value);
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
			if (isp->flash)
				ret = v4l2_subdev_call(isp->flash,
					core, s_ctrl, &ctrl);
			break;
		case V4L2_CID_ZOOM_ABSOLUTE:
			ret = atomisp_digital_zoom(isp, 1, &ctrl.value);
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

	mutex_lock(&isp->input_lock);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		mutex_unlock(&isp->input_lock);
		return -EINVAL;
	}

	parm->parm.capture.capturemode = isp->sw_contex.run_mode;

	mutex_unlock(&isp->input_lock);
	return 0;
}

static int atomisp_s_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret = 0;

	mutex_lock(&isp->input_lock);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		mutex_unlock(&isp->input_lock);
		return -EINVAL;
	}

	isp->sw_contex.run_mode = parm->parm.capture.capturemode;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				video, s_parm, parm);

	mutex_unlock(&isp->input_lock);
	/*
	 * why do we think the return value -ENOIOCTLCMD is ok?
	 * that's because some sensor drivers may don't need s_parm
	 * to set their run_mode and so have no the s_parm interface.
	 */
	if (ret && ret != -ENOIOCTLCMD) {
		v4l2_err(&atomisp_dev,
			    "failed to s_parm for sensor\n");
		return ret;
	}
	return 0;
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

	isp->sw_contex.output_mode = parm->parm.output.outputmode;
	if (isp->sw_contex.output_mode == OUTPUT_MODE_FILE)
		isp->sw_contex.file_input = 1;


	return 0;
}

/* set default atomisp ioctl value */
static long atomisp_vidioc_default(struct file *file, void *fh,
	bool valid_prio, int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

	switch (cmd) {
	case ATOMISP_IOC_G_XNR:
		return atomisp_xnr(isp, 0, arg);

	case ATOMISP_IOC_S_XNR:
		return atomisp_xnr(isp, 1, arg);

	case ATOMISP_IOC_G_NR:
		return atomisp_nr(isp, 0, arg);

	case ATOMISP_IOC_S_NR:
		return atomisp_nr(isp, 1, arg);

	case ATOMISP_IOC_G_TNR:
		return atomisp_tnr(isp, 0, arg);

	case ATOMISP_IOC_S_TNR:
		return atomisp_tnr(isp, 1, arg);

	case ATOMISP_IOC_G_HISTOGRAM:
		return atomisp_histogram(isp, 0, arg);

	case ATOMISP_IOC_S_HISTOGRAM:
		return atomisp_histogram(isp, 1, arg);

	case ATOMISP_IOC_G_BLACK_LEVEL_COMP:
		return atomisp_black_level(isp, 0, arg);

	case ATOMISP_IOC_S_BLACK_LEVEL_COMP:
		return atomisp_black_level(isp, 1, arg);

	case ATOMISP_IOC_G_EE:
		return atomisp_ee(isp, 0, arg);

	case ATOMISP_IOC_S_EE:
		return atomisp_ee(isp, 1, arg);

	case ATOMISP_IOC_G_DIS_STAT:
		return atomisp_get_dis_stat(isp,
				(struct atomisp_dis_statistics *)arg);

	case ATOMISP_IOC_S_DIS_COEFS:
		return atomisp_set_dis_coefs(isp,
				(struct atomisp_dis_coefficients *)arg);

	case ATOMISP_IOC_S_DIS_VECTOR:
		return atomisp_set_dis_vector(isp,
				(struct atomisp_dis_vector *) arg);

	case ATOMISP_IOC_G_ISP_PARM:
		return atomisp_param(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_PARM:
		return atomisp_param(isp, 1, arg);

	case ATOMISP_IOC_G_3A_STAT:
		return atomisp_3a_stat(isp, 0, arg);

	case ATOMISP_IOC_G_ISP_GAMMA:
		return atomisp_gamma(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_GAMMA:
		return atomisp_gamma(isp, 1, arg);

	case ATOMISP_IOC_G_ISP_GDC_TAB:
		return atomisp_gdc_cac_table(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_GDC_TAB:
		return atomisp_gdc_cac_table(isp, 1, arg);

	case ATOMISP_IOC_G_ISP_MACC:
		return atomisp_macc_table(isp, 0,
			(struct atomisp_macc_config *)arg);

	case ATOMISP_IOC_S_ISP_MACC:
		return atomisp_macc_table(isp, 1,
			(struct atomisp_macc_config *)arg);

	case ATOMISP_IOC_G_ISP_BAD_PIXEL_DETECTION:
		return atomisp_bad_pixel_param(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_BAD_PIXEL_DETECTION:
		return atomisp_bad_pixel_param(isp, 1, arg);

	case ATOMISP_IOC_G_ISP_FALSE_COLOR_CORRECTION:
		return atomisp_false_color_param(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_FALSE_COLOR_CORRECTION:
		return atomisp_false_color_param(isp, 1, arg);

	case ATOMISP_IOC_G_ISP_CTC:
		return atomisp_ctc(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_CTC:
		return atomisp_ctc(isp, 1, arg);

	case ATOMISP_IOC_G_ISP_WHITE_BALANCE:
		return atomisp_white_balance_param(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_WHITE_BALANCE:
		return atomisp_white_balance_param(isp, 1, arg);

	case ATOMISP_IOC_G_3A_CONFIG:
		return atomisp_3a_config_param(isp, 0, arg);

	case ATOMISP_IOC_S_3A_CONFIG:
		return atomisp_3a_config_param(isp, 1, arg);

	case ATOMISP_IOC_S_ISP_FPN_TABLE:
		return atomisp_fixed_pattern_table(isp, arg);

	case ATOMISP_IOC_G_ISP_OVERLAY:
		return atomisp_vf_overlay(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_OVERLAY:
		return atomisp_vf_overlay(isp, 1, arg);

	case ATOMISP_IOC_ISP_MAKERNOTE:
		return atomisp_exif_makernote(isp, arg);

	case ATOMISP_IOC_G_SENSOR_MODE_DATA:
		return atomisp_get_sensor_mode_data(isp, arg);

	case ATOMISP_IOC_G_MOTOR_PRIV_INT_DATA:
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
		return v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
					core, ioctl, cmd, arg);

	case ATOMISP_IOC_ACC_LOAD:
		return atomisp_acc_load(isp, arg);

	case ATOMISP_IOC_ACC_UNLOAD:
		return atomisp_acc_unload(isp, arg);

	case ATOMISP_IOC_ACC_S_ARG:
		return atomisp_acc_set_arg(isp, arg);

	case ATOMISP_IOC_ACC_START:
		return atomisp_acc_start(isp, arg);

	case ATOMISP_IOC_ACC_WAIT:
		return atomisp_acc_wait(isp, arg);

	case ATOMISP_IOC_ACC_ABORT:
		return atomisp_acc_abort(isp, arg);

	case ATOMISP_IOC_ACC_DESTAB:
		return atomisp_acc_destabilize(isp, arg);

	case ATOMISP_IOC_CAMERA_BRIDGE:
		return -EINVAL;

	case ATOMISP_IOC_S_ISP_SHD_TAB:
		return atomisp_set_shading_table(isp, arg);

	case ATOMISP_IOC_G_ISP_GAMMA_CORRECTION:
		return atomisp_gamma_correction(isp, 0, arg);

	case ATOMISP_IOC_S_ISP_GAMMA_CORRECTION:
		return atomisp_gamma_correction(isp, 1, arg);

	case ATOMISP_IOC_S_PARAMETERS:
		return atomisp_set_parameters(isp, arg);

	default:
		return -EINVAL;
	}
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
	.vidioc_cropcap = atomisp_cropcap,
	.vidioc_enum_framesizes = atomisp_enum_framesizes,
	.vidioc_enum_frameintervals = atomisp_enum_frameintervals,
	.vidioc_s_parm = atomisp_s_parm,
	.vidioc_g_parm = atomisp_g_parm,
	.vidioc_g_std = atomisp_g_std,
	.vidioc_s_std = atomisp_s_std,
	.vidioc_g_crop = atomisp_g_crop,
	.vidioc_s_crop = atomisp_s_crop,
};

const struct v4l2_ioctl_ops atomisp_file_ioctl_ops = {
	.vidioc_querycap = atomisp_querycap,
	.vidioc_g_fmt_vid_out = atomisp_g_fmt_file,
	.vidioc_s_fmt_vid_out = atomisp_s_fmt_file,
	.vidioc_s_parm = atomisp_s_parm_file,
	.vidioc_reqbufs = atomisp_reqbufs_file,
	.vidioc_querybuf = atomisp_querybuf_file,
	.vidioc_qbuf = atomisp_qbuf_file,
	/* .vidioc_streamon = atomisp_streamon_out, */
};
