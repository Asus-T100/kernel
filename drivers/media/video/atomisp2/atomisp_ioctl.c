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
#include "atomisp_acc.h"
#include "atomisp_fops.h"
#include <linux/delay.h>

#include "sh_css_hrt.h"
#include "sh_css.h"
#include <sh_css_debug.h>
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

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       core, g_chip_ident, chip);

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
	if (pipe->pipe_type != ATOMISP_PIPE_CAPTURE)
		return 0;

	mutex_lock(&isp->mutex);
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
	mutex_unlock(&isp->mutex);

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

	if ((isp->isp_subdev.video_out_preview.capq.streaming == 1) ||
	    (isp->isp_subdev.video_out_capture.capq.streaming == 1) ||
	    (isp->isp_subdev.video_in.capq.streaming == 1)) {
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
	if (!isp->sw_contex.file_input) {
		ret = v4l2_subdev_call(isp->inputs[input].camera,
				       core, s_power, 1);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Failed to power-on sensor\n");
			ret = -EINVAL;
			goto error;
		}
		if (isp->inputs[input].motor)
			ret = v4l2_subdev_call(isp->inputs[input].motor, core,
					       init, 1);
	}

	isp->input_curr = input;
	mutex_unlock(&isp->mutex);

	return 0;

error:
	mutex_unlock(&isp->mutex);

	return ret;
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
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);

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

	mutex_lock(&isp->mutex);
	f->pixelformat = atomisp_output_fmts[index].pixelformat;
	memset(f->description, 0, sizeof(char)*32);
	strncpy(f->description, atomisp_output_fmts[index].description,
		strlen(atomisp_output_fmts[index].description));
	mutex_unlock(&isp->mutex);

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
	int ret;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_err(&atomisp_dev,
				"unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	memset(f, 0, sizeof(struct v4l2_format));
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	mutex_lock(&isp->mutex);
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
	mutex_unlock(&isp->mutex);

	return ret;
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

	mutex_lock(&isp->mutex);
	input = &isp->inputs[isp->input_curr];

	if (input->type != SOC_CAMERA && input->type != RAW_CAMERA) {
		ret = -EINVAL;
		goto error;
	}

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
		mutex_unlock(&isp->mutex);

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
	sensor_parm.parm.capture.capturemode = CI_MODE_STILL_CAPTURE;
	v4l2_subdev_call(input->camera, video, s_parm, &sensor_parm);

	/* get the sensor max resolution supported */
	snr_mbus_fmt.height = ATOM_ISP_MAX_HEIGHT;
	snr_mbus_fmt.width = ATOM_ISP_MAX_WIDTH;

	ret = v4l2_subdev_call(input->camera, video, try_mbus_fmt,
							&snr_mbus_fmt);
	if (ret < 0)
		goto error;

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
	mutex_unlock(&isp->mutex);

	return 0;

error:
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

	if (!atomisp_is_pixelformat_supported(arg->pixel_format))
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
 * this function is used to free video buffer
 */
static void atomisp_videobuf_free(struct videobuf_queue *q)
{
	int i;
	struct videobuf_vmalloc_memory *vm_mem;

	mutex_lock(&q->vb_lock);

	if (!q) {
		mutex_unlock(&q->vb_lock);
		return;
	}

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		vm_mem = q->bufs[i]->priv;

		if (vm_mem && vm_mem->vaddr) {
			sh_css_frame_free(vm_mem->vaddr);
			vm_mem->vaddr = NULL;
		}
		kfree(q->bufs[i]);
		q->bufs[i] = NULL;
	}

	mutex_unlock(&q->vb_lock);
}

int atomisp_alloc_css_stat_bufs(struct atomisp_device *isp, int count)
{
	struct atomisp_s3a_buf *s3a_buf = NULL;
	struct atomisp_dis_buf *dis_buf = NULL;

	if (!count ||
	    (!list_empty(&isp->s3a_stats) &&
	     !list_empty(&isp->dis_stats))) {
		return 0;
	}

	v4l2_dbg(2, dbg_level, &atomisp_dev,
		    "allocating %d 3a & dis buffers\n", count);
	while (count--) {
		s3a_buf = kzalloc(sizeof(struct atomisp_s3a_buf), GFP_KERNEL);
		if (!s3a_buf) {
			v4l2_err(&atomisp_dev, "s3a stat buf alloc failed\n");
			goto error;
		}

		dis_buf = kzalloc(sizeof(struct atomisp_dis_buf), GFP_KERNEL);
		if (!dis_buf) {
			v4l2_err(&atomisp_dev, "dis stat buf alloc failed\n");
			goto error;
		}

		if (isp->params.curr_grid_info.s3a_grid.use_dmem) {
			if (sh_css_allocate_stat_buffers_from_info(
						&s3a_buf->s3a_data,
						&dis_buf->dis_data,
						&isp->params.curr_grid_info)) {
				v4l2_err(&atomisp_dev,
						"stat buf allocation failed\n");
				goto error;
			}
		} else {
			if (sh_css_allocate_stat_buffers_from_info(
						&s3a_buf->s3a_data,
						&dis_buf->dis_data,
						&isp->params.curr_grid_info)) {
				v4l2_err(&atomisp_dev,
						"stat buf allocation failed\n");
				goto error;
			}
		}
		list_add_tail(&s3a_buf->list, &isp->s3a_stats);
		list_add_tail(&dis_buf->list, &isp->dis_stats);
	}

	return 0;
error:
	v4l2_err(&atomisp_dev,
		    "failed to allocate statistics buffers\n");
	kfree(s3a_buf);
	kfree(dis_buf);
	return -ENOMEM;
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
	struct sh_css_frame_info out_info, vf_info, frame_info;
	struct sh_css_frame *frame;
	struct videobuf_vmalloc_memory *vm_mem;
	int ret = 0, i = 0;

	if (req->count == 0) {
		atomisp_videobuf_free(&pipe->capq);
		return 0;
	}

/*
	if (pipe->pipe_type != ATOMISP_PIPE_CAPTURE &&
	    !atomisp_is_viewfinder_support(isp))
		return -EINVAL;
*/

	ret = videobuf_reqbufs(&pipe->capq, req);
	if (ret)
		return ret;

	mutex_lock(&isp->mutex);
	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		if (isp->capture_format &&
		    isp->capture_format->out_sh_fmt !=
		    SH_CSS_FRAME_FORMAT_RAW) {
			if (sh_css_capture_get_viewfinder_frame_info(&vf_info))
				goto error;
		}
		if (sh_css_capture_get_output_frame_info(&out_info))
			goto error;
		break;
	case CI_MODE_VIDEO:
		if (sh_css_video_get_viewfinder_frame_info(&vf_info))
			goto error;

		if (sh_css_video_get_output_frame_info(&out_info))
			goto error;
		break;
	case CI_MODE_PREVIEW:
		if (sh_css_preview_get_output_frame_info(&vf_info))
			goto error;
		break;
	default:
		mutex_unlock(&isp->mutex);
		return -EINVAL;
	}

	atomisp_alloc_css_stat_bufs(isp, ATOMISP_CSS_Q_DEPTH);

	/*
	 * for user pointer type, buffers are not really allcated here,
	 * buffers are setup in QBUF operation through v4l2_buffer structure
	 */
	if (req->memory == V4L2_MEMORY_USERPTR) {
		mutex_unlock(&isp->mutex);
		return 0;
	}

	switch (pipe->pipe_type) {
	case ATOMISP_PIPE_CAPTURE:
		frame_info = out_info;
		break;
	case ATOMISP_PIPE_PREVIEW:
	case ATOMISP_PIPE_VIEWFINDER:
		frame_info = vf_info;
		break;
	/* node not supported */
	default:
		mutex_unlock(&isp->mutex);
		return -EINVAL;
	}
	/*
	 * Allocate the real frame here for selected node using our
	 * memory management function
	 */
	for (i = 0; i < req->count; i++) {
		if (sh_css_frame_allocate_from_info(&frame, &frame_info))
			goto error;
		vm_mem = pipe->capq.bufs[i]->priv;
		vm_mem->vaddr = frame;
	}

	mutex_unlock(&isp->mutex);
	return ret;

error:
	while (i--) {
		vm_mem = pipe->capq.bufs[i]->priv;
		sh_css_frame_free(vm_mem->vaddr);
	}

	if (isp->vf_frame)
		sh_css_frame_free(isp->vf_frame);

	mutex_unlock(&isp->mutex);
	return -ENOMEM;
}

static int atomisp_reqbufs_file(struct file *file, void *fh,
		struct v4l2_requestbuffers *req)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	if (req->count == 0) {
		atomisp_videobuf_free(&pipe->outq);
		return 0;
	}

	ret = videobuf_reqbufs(&pipe->outq, req);
	if (ret)
		return ret;

	mutex_lock(&isp->mutex);
	if (isp->sw_contex.output_mode == OUTPUT_MODE_TEXT) {
		mutex_unlock(&isp->mutex);
		return 0;
	}

	/*
	 *  TODO: Implement file input function
	 */
	mutex_unlock(&isp->mutex);

	return 0;
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
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	unsigned long userptr = buf->m.userptr;
	struct videobuf_buffer *vb;
	struct videobuf_vmalloc_memory *vm_mem;
	struct sh_css_frame_info out_info, vf_info, frame_info;
	struct sh_css_frame *handle = NULL;
	u32 length;
	u32 pgnr;
	int ret = 0;

	mutex_lock(&isp->mutex);
	if (isp->sw_contex.error && isp->sw_contex.isp_streaming) {
		v4l2_err(&atomisp_dev, "ISP ERROR\n");
		ret = -EIO;
		goto error;
	}

/*
	if ((!pipe->is_main) &&
	    (!atomisp_is_viewfinder_support(isp)))
		return -EINVAL;
*/
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
		vb = pipe->capq.bufs[buf->index];
		vm_mem = vb->priv;
		if (!vm_mem) {
			ret = -EINVAL;
			goto error;
		}

		length = vb->bsize;
		pgnr = (length + (PAGE_SIZE - 1)) >> PAGE_SHIFT;

		if ((vb->baddr == userptr) && (vm_mem->vaddr))
			goto done;

		switch (isp->sw_contex.run_mode) {
		case CI_MODE_STILL_CAPTURE:
			if (isp->capture_format &&
			    (isp->capture_format->out_sh_fmt !=
				SH_CSS_FRAME_FORMAT_RAW) &&
			sh_css_capture_get_viewfinder_frame_info(&vf_info))
				goto error;

			if (sh_css_capture_get_output_frame_info(&out_info))
				goto error;
			break;
		case CI_MODE_VIDEO:
			if (sh_css_video_get_viewfinder_frame_info(&vf_info))
				goto error;
			if (sh_css_video_get_output_frame_info(&out_info))
				goto error;
			break;
		case CI_MODE_PREVIEW:
			if (sh_css_preview_get_output_frame_info(&vf_info))
				goto error;
			break;
		}
#ifdef CONFIG_ION
		hrt_isp_css_mm_set_user_ptr(userptr, pgnr,
			buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_ION
				? HRT_USR_ION : HRT_USR_PTR);
#else
		hrt_isp_css_mm_set_user_ptr(userptr, pgnr, HRT_USR_PTR);
#endif

		switch (pipe->pipe_type) {
		case ATOMISP_PIPE_CAPTURE:
			frame_info = out_info;
			break;
		case ATOMISP_PIPE_VIEWFINDER:
		case ATOMISP_PIPE_PREVIEW:
			frame_info = vf_info;
			break;
		/*TODO: fileinput support missing
		case ATOMISP_PIPE_FILEINPUT:
			break;
		*/
		/* node not supported */
		default:
			ret = -EINVAL;
			goto error;
		}
		ret = sh_css_frame_allocate_from_info(&handle,
							&frame_info);

		hrt_isp_css_mm_set_user_ptr(0, 0, HRT_USR_PTR);

		if (ret != sh_css_success) {
			v4l2_err(&atomisp_dev, "Error to allocate frame\n");
			ret = -ENOMEM;
			goto error;
		}

		if (vm_mem->vaddr) {
			mutex_lock(&pipe->capq.vb_lock);
			sh_css_frame_free(vm_mem->vaddr);
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
	ret = videobuf_qbuf(&pipe->capq, buf);
	/* TODO: do this better, not best way to queue to css */
	if (isp->sw_contex.isp_streaming)
		atomisp_qbuffers_to_css(isp, pipe);
	mutex_unlock(&isp->mutex);
	return ret;

error:
	mutex_unlock(&isp->mutex);
	v4l2_err(&atomisp_dev, "<%s: get_output_frame_info error\n", __func__);
	return -EINVAL;
}

static int atomisp_qbuf_file(struct file *file, void *fh,
					struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;

	mutex_lock(&isp->mutex);
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

	mutex_lock(&isp->mutex);

/*
	if ((!pipe->is_main) &&
	    (!atomisp_is_viewfinder_support(isp)))
		return -EINVAL;
*/
	if (isp->sw_contex.error) {
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
	buf->bytesused = pipe->format->out.sizeimage;
	buf->reserved = isp->frame_status[buf->index];
	mutex_unlock(&isp->mutex);

	return 0;
}

int atomisp_get_css_pipe_id(struct atomisp_device *isp, enum sh_css_pipe_id *pipe)
{
	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		*pipe = SH_CSS_CAPTURE_PIPELINE;
		break;
	case CI_MODE_PREVIEW:
		*pipe = SH_CSS_PREVIEW_PIPELINE;
		break;
	case CI_MODE_VIDEO:
		*pipe = SH_CSS_VIDEO_PIPELINE;
		break;
	default:
	v4l2_err(&atomisp_dev,"atomisp_get_css_pipe_id\n");
		return -EINVAL;
	}
	return 0;
}

int atomisp_get_css_buf_type(struct atomisp_device *isp,
			 struct atomisp_video_pipe *pipe)
{
	int buf_type;
	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
	case CI_MODE_VIDEO:
		if (pipe->pipe_type == ATOMISP_PIPE_CAPTURE)
			buf_type = SH_CSS_BUFFER_TYPE_OUTPUT_FRAME;
		else
			buf_type = SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME;
		break;
	case CI_MODE_PREVIEW:
		buf_type = SH_CSS_BUFFER_TYPE_OUTPUT_FRAME;
		break;
	default:
		return -EINVAL;
	}
	return buf_type;
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
	enum sh_css_pipe_id css_pipe_id;
	int ret;
	unsigned long irqflags;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif
	v4l2_dbg(3, dbg_level, &atomisp_dev, ">%s, [%d]\n",
			__func__, pipe->pipe_type);
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	spin_lock_irqsave(&pipe->irq_lock, irqflags);
	if (list_empty(&(pipe->capq.stream))) {
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
		v4l2_err(&atomisp_dev,
			"no buffer in the queue\n");
		ret = -EINVAL;
		goto error;
	}
	spin_unlock_irqrestore(&pipe->irq_lock, irqflags);

	ret = videobuf_streamon(&pipe->capq);
	if (ret)
		goto error;


	if (isp->sw_contex.work_queued)
		goto done;

	if (isp->sw_contex.isp_streaming) {
		atomisp_qbuffers_to_css(isp, pipe);
		goto start_workq;
	}

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_MRFLD) {
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

	ret = atomisp_get_css_pipe_id(isp, &css_pipe_id);
	if (ret < 0)
		goto error;


	ret = sh_css_start(css_pipe_id);
	if (ret) {
		v4l2_err(&atomisp_dev,
				"sh_css_start fails: %d\n", ret);
		goto error;
	}

	/* Make sure that update_isp_params is called at least once.*/
	isp->params.css_update_params_needed = true;
	isp->sw_contex.isp_streaming = true;
	isp->isp_timeout = false;
	atomic_set(&isp->sequence, -1);
	atomic_set(&isp->wdt_count, 0);
	mod_timer(&isp->wdt, jiffies + ATOMISP_ISP_TIMEOUT_DURATION);
	isp->fr_status = ATOMISP_FRAME_STATUS_OK;
	isp->sw_contex.error = false;
	isp->sw_contex.invalid_frame = false;
	isp->irq_infos = 0;

	atomisp_qbuffers_to_css(isp, pipe);
	//sh_css_dump_sp_sw_debug_info();

	/* don't start workq yet, wait for another pipe*/
	/* for capture pipe + raw output,  ISP only support output main*/
	if (isp->sw_contex.run_mode == CI_MODE_VIDEO ||
		(isp->sw_contex.run_mode == CI_MODE_STILL_CAPTURE &&
			(isp->capture_format->out_sh_fmt !=
			 SH_CSS_FRAME_FORMAT_RAW)))
		goto done;

start_workq:
	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 1);
		isp->params.num_flash_frames = 0;
		isp->params.flash_state = ATOMISP_FLASH_IDLE;
		atomisp_setup_flash(isp);
	}

	isp->sw_contex.work_queued = true;

	if (!isp->sw_contex.file_input) {
#ifndef CONFIG_X86_MRFLD
		sh_css_enable_interrupt(SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF,
					true);
#endif /* CONFIG_X86_MRFLD */

		atomisp_set_term_en_count(isp);
		/*
		 * stream on the sensor, power on is called before
		 * work queue start
		 */
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 1);
		if (ret) {
			atomisp_reset(isp);
			ret = -EINVAL;
			goto error;
		}
		isp->sw_contex.sensor_streaming = true;
	}

done:
	/*
	 * setting error to false temporarily here because HAL starts to
	 * dequeue immediately after streamon and if we have not managed to
	 * get into workq before thati(error set to false there usually),
	 * this flag will be still true, causing dqbuf to fail
	 */
	isp->sw_contex.error = false;
	mutex_unlock(&isp->mutex);
	v4l2_dbg(3, dbg_level, &atomisp_dev, "<%s\n", __func__);
	return 0;

error:
	mutex_unlock(&isp->mutex);
	v4l2_dbg(3, dbg_level, &atomisp_dev, "<%s ret: %d\n", __func__, ret);

	return ret;
}

/*This ioctl stop the capture or output process during streaming I/O.*/
int atomisp_streamoff(struct file *file, void *fh,
		      enum v4l2_buf_type type)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct atomisp_video_pipe *capture_pipe = NULL;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *preview_pipe = NULL;
	struct videobuf_buffer *vb = NULL;
	int ret;
	unsigned long flags;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif

	v4l2_dbg(3, dbg_level, &atomisp_dev, ">%s, [%d]\n",
			__func__, pipe->pipe_type);
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupported v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	if(!isp->sw_contex.isp_streaming) {
		ret = videobuf_streamoff(&pipe->capq);
		if (ret)
			goto error;

		if (isp->sw_contex.sensor_streaming)
			goto stopsensor;
		else {
			mutex_unlock(&isp->mutex);
			return ret;
		}
	}

	isp->sw_contex.isp_streaming = false;
	isp->sw_contex.error = true;

	isp->s3a_bufs_in_css = 0;
	isp->frame_bufs_in_css = 0;
	isp->dis_bufs_in_css = 0;
	isp->vf_frame_bufs_in_css = 0;

#ifndef CONFIG_X86_MRFLD
	if (!isp->sw_contex.file_input)
		sh_css_enable_interrupt(SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF,
					false);
#endif /* CONFIG_X86_MRFLD */

	switch (isp->sw_contex.run_mode) {
	case CI_MODE_STILL_CAPTURE:
		v4l2_dbg(3, dbg_level, &atomisp_dev,
				"%s, stop capture\n",__func__);
		sh_css_capture_stop();
		break;
	case CI_MODE_PREVIEW:
		v4l2_dbg(3, dbg_level, &atomisp_dev,
				"%s, stop preview\n",__func__);
		sh_css_preview_stop();
		break;
	case CI_MODE_VIDEO:
		v4l2_dbg(3, dbg_level, &atomisp_dev,
				"%s, stop video\n",__func__);
		sh_css_video_stop();
		break;
	}

	del_timer_sync(&isp->wdt);
	cancel_work_sync(&isp->wdt_work);

	/* cancel work queue*/
	if (isp->isp_subdev.video_out_capture.opened) {
		capture_pipe = &isp->isp_subdev.video_out_capture;
		wake_up_interruptible(&capture_pipe->capq.wait);
	}
	if (isp->isp_subdev.video_out_vf.opened) {
		vf_pipe = &isp->isp_subdev.video_out_vf;
		wake_up_interruptible(&vf_pipe->capq.wait);
	}
	if (isp->isp_subdev.video_out_preview.opened) {
		preview_pipe = &isp->isp_subdev.video_out_preview;
		wake_up_interruptible(&preview_pipe->capq.wait);
	}
	isp->sw_contex.work_queued = false;

	ret = videobuf_streamoff(&pipe->capq);
	if (ret)
		goto error;

	/* cleanup css here */
	/* no need for this, as ISP will be reset anyway */
	/*atomisp_flush_bufs_in_css(isp);*/

	spin_lock_irqsave(&pipe->irq_lock, flags);
	while (!list_empty(&pipe->activeq)) {
		vb = list_first_entry(&pipe->activeq, struct videobuf_buffer, queue);
		if (!vb)
			break;
		vb->state = VIDEOBUF_PREPARED;
		list_del(&vb->queue);
	}
	spin_unlock_irqrestore(&pipe->irq_lock, flags);

	/* stream off sensor, power off is called in senor driver */
	/*
	 * so this now introduces dpendency to streamoff order between mo and
	 * vf pipes
	 */
	/*stream off sensor, power off is called in senor driver*/
	if ((pipe->pipe_type == ATOMISP_PIPE_PREVIEW ||
	     pipe->pipe_type == ATOMISP_PIPE_VIEWFINDER) &&
	    isp->isp_subdev.video_out_capture.capq.streaming == 1) {
		mutex_unlock(&isp->mutex);
		return 0;
	}

stopsensor:
	if (!isp->sw_contex.file_input) {
		v4l2_dbg(3, dbg_level, &atomisp_dev,
			 "%s, streamoff sensor\n",__func__);
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 0);
	}

	if (isp->flash) {
		ret += v4l2_subdev_call(isp->flash, core, s_power, 0);
		isp->params.num_flash_frames = 0;
		isp->params.flash_state = ATOMISP_FLASH_IDLE;
	}

	isp->sw_contex.sensor_streaming = false;

#ifdef PUNIT_CAMERA_BUSY
	if (!IS_MRFLD) {
		/* Free camera_busy bit */
		msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, MFLD_OR1);
		msg_ret &= ~0x100;
		intel_mid_msgbus_write32(PUNIT_PORT, MFLD_OR1, msg_ret);
	}
#endif

	/*
	 * ISP work around, need to reset isp
	 * Is it correct time to reset ISP when first node does streamoff?
	 */
	if (isp->sw_contex.power_state == ATOM_ISP_POWER_UP)
		atomisp_reset(isp);
	mutex_unlock(&isp->mutex);

	v4l2_dbg(3, dbg_level, &atomisp_dev, "<%s\n", __func__);
	return ret;

error:
	mutex_unlock(&isp->mutex);
	v4l2_dbg(3, dbg_level, &atomisp_dev, "<%s ret: %d\n", __func__, ret);

	return ret;
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
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_POWER_LINE_FREQUENCY:
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
	parm->parm.capture.capturemode = isp->sw_contex.run_mode;
	mutex_unlock(&isp->mutex);

	return 0;
}

static int atomisp_s_parm(struct file *file, void *fh,
	struct v4l2_streamparm *parm)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	int ret = 0;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&atomisp_dev,
			    "unsupport v4l2 buf type\n");
		return -EINVAL;
	}

	mutex_lock(&isp->mutex);
	isp->sw_contex.run_mode = parm->parm.capture.capturemode;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				video, s_parm, parm);
	mutex_unlock(&isp->mutex);

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

	mutex_lock(&isp->mutex);
	isp->sw_contex.output_mode = parm->parm.output.outputmode;
	if (isp->sw_contex.output_mode == OUTPUT_MODE_FILE)
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

	case ATOMISP_IOC_G_HISTOGRAM:
		err = atomisp_histogram(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_HISTOGRAM:
		err = atomisp_histogram(isp, 1, arg);
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

	case ATOMISP_IOC_G_ISP_OVERLAY:
		err = atomisp_vf_overlay(isp, 0, arg);
		break;

	case ATOMISP_IOC_S_ISP_OVERLAY:
		err = atomisp_vf_overlay(isp, 1, arg);
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

	case ATOMISP_IOC_CAMERA_BRIDGE:
		err = -EINVAL;
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
