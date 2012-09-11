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
#include <linux/pm_qos_params.h>
#include <linux/async.h>
#include "atomisp_ioctl.h"
#include "atomisp_cmd.h"
#include "atomisp_fops.h"
#include "atomisp_file.h"

#include "device_access.h"

/* cross componnet debug message flag */
int dbg_level = 0;
module_param(dbg_level, int, 0644);
MODULE_PARM_DESC(dbg_level, "debug message on/off (default:off)");

int mipicsi_flag;
module_param(mipicsi_flag, int, 0644);
MODULE_PARM_DESC(mipicsi_flag, "mipi csi compression predictor algorithm");

/*set to 16x16 since this is the amount of lines and pixels the sensor
exports extra. If these are kept at the 10x8 that they were on, in yuv
downscaling modes incorrect resolutions where requested to the sensor
driver with strange outcomes as a result. The proper way tot do this
would be to have a list of tables the specify the sensor res, mipi rec,
output res, and isp output res. however since we do not have this yet,
the chosen solution is the next best thing. */
int pad_w = 16;
module_param(pad_w, int, 0644);
MODULE_PARM_DESC(pad_w, "extra data for ISP processing");

int pad_h = 16;
module_param(pad_h, int, 0644);
MODULE_PARM_DESC(pad_h, "extra data for ISP processing");

struct v4l2_device atomisp_dev = {
	.name = "atomisp",
};

void __iomem *atomisp_io_base;

int atomisp_pci_vendor; /* pci vendor id */
int atomisp_pci_device; /* pci device id */

/*
 * supported V4L2 fmts and resolutions
 */
const struct atomisp_format_bridge atomisp_output_fmts[] = {
	{
		.pixelformat = V4L2_PIX_FMT_YUV420,
		.depth = 12,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_YUV420,
		.description = "YUV420, planner"},
	{
		.pixelformat = V4L2_PIX_FMT_YVU420,
		.depth = 12,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_YV12,
		.description = "YVU420, planner"},
	{
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_YUV422,
		.description = "YUV422, planner"},
	{
		.pixelformat = V4L2_PIX_FMT_YUV444,
		.depth = 24,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_YUV444,
		.description = "YUV444"},
	{
		.pixelformat = V4L2_PIX_FMT_NV12,
		.depth = 12,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_NV12,
		.description = "NV12, interleaved"},
	{
		.pixelformat = V4L2_PIX_FMT_NV21,
		.depth = 12,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_NV21,
		.description = "NV21, interleaved"},
	{
		.pixelformat = V4L2_PIX_FMT_NV16,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_NV16,
		.description = "NV16, interleaved"},
	{
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_YUYV,
		.description = "YUYV, interleaved"},
	{
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.sh_fmt = SH_CSS_FRAME_FORMAT_UYVY,
		.description = "UYVY, interleaved"},
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR16,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 16"},
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGRBG8_1X8,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"},
	{
		.pixelformat = V4L2_PIX_FMT_SGBRG8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGRBG8_1X8,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"},
	{
		.pixelformat = V4L2_PIX_FMT_SGRBG8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGRBG8_1X8,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"},
	{
		.pixelformat = V4L2_PIX_FMT_SRGGB8,
		.depth = 8,
		.mbus_code = V4L2_MBUS_FMT_SGRBG8_1X8,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 8"},
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"},
	{
		.pixelformat = V4L2_PIX_FMT_SGBRG10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"},
	{
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"},
	{
		.pixelformat = V4L2_PIX_FMT_SRGGB10,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 10"},
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR12_1X12,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"},
	{
		.pixelformat = V4L2_PIX_FMT_SGBRG12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR12_1X12,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"},
	{
		.pixelformat = V4L2_PIX_FMT_SGRBG12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR12_1X12,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"},
	{
		.pixelformat = V4L2_PIX_FMT_SRGGB12,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_SBGGR12_1X12,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RAW,
		.description = "Bayer 12"},
	{
		.pixelformat = V4L2_PIX_FMT_RGB32,
		.depth = 32,
		.mbus_code = V4L2_MBUS_FMT_FIXED,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RGBA888,
		.description = "32 RGB 8-8-8-8"},
	{
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.depth = 16,
		.mbus_code = V4L2_MBUS_FMT_BGR565_2X8_LE,
		.sh_fmt = SH_CSS_FRAME_FORMAT_RGB565,
		.description = "16 RGB 5-6-5"}
};

const u32 atomisp_output_fmts_num = ARRAY_SIZE(atomisp_output_fmts);

static struct atomisp_tvnorm tvnorms[] = {
	{
		.name = "atomisp format",
		.id = V4L2_STD_NTSC_M,
		.cxiformat = 0,
		.cxoformat = 0x181f0008,
	},
	{
		.name = "atomisp format NULL",
		.id = V4L2_STD_NTSC_M_JP,
		.cxiformat = 1,
		.cxoformat = 0x181f0008,
	}
};

static const struct video_device atomisp_video_dev = {
	.name = "atomisp",
	.minor = -1,
	.fops = &atomisp_fops,
	.release = video_device_release_empty,
	.ioctl_ops = &atomisp_ioctl_ops
};

int atomisp_video_init(struct atomisp_video_pipe *video, const char *name)
{
	int ret;
	const char *direction;

	switch (video->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		direction = "output";
		video->pad.flags = MEDIA_PAD_FL_SINK;
		video->vdev.fops = &atomisp_fops;
		video->vdev.ioctl_ops = &atomisp_ioctl_ops;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		direction = "input";
		video->pad.flags = MEDIA_PAD_FL_SOURCE;
		video->vdev.fops = &atomisp_file_fops;
		video->vdev.ioctl_ops = &atomisp_file_ioctl_ops;
		break;
	default:
		return -EINVAL;
	}

	ret = media_entity_init(&video->vdev.entity, 1, &video->pad, 0);
	if (ret < 0)
		return ret;

	/* Initialize the video device. */
	snprintf(video->vdev.name, sizeof(video->vdev.name),
		 "ATOMISP ISP %s %s", name, direction);
	video->vdev.release = video_device_release_empty;
	video_set_drvdata(&video->vdev, video->isp);
	video->opened = false;

	return 0;
}

int atomisp_video_register(struct atomisp_video_pipe *video,
	struct v4l2_device *vdev)
{
	int ret;

	video->vdev.v4l2_dev = vdev;

	ret = video_register_device(&video->vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		v4l2_err(&atomisp_dev,
			"%s: could not register video device (%d)\n",
			__func__, ret);

	return ret;
}

void atomisp_video_unregister(struct atomisp_video_pipe *video)
{
	if (video_is_registered(&video->vdev)) {
		media_entity_cleanup(&video->vdev.entity);
		video_unregister_device(&video->vdev);
	}
}

#ifdef CONFIG_PM
static int atomisp_runtime_suspend(struct device *dev)
{
	struct atomisp_device *isp = (struct atomisp_device *)
		dev_get_drvdata(dev);
	int ret;

	/* save IUnit context */
	atomisp_save_iunit_reg(isp);

	/*Turn off the ISP d-phy*/
	ret = atomisp_ospm_dphy_down(isp);
	if (!ret)
		pm_qos_update_request(&isp->pm_qos, PM_QOS_DEFAULT_VALUE);

	return ret;
}

static int atomisp_runtime_resume(struct device *dev)
{
	struct atomisp_device *isp = (struct atomisp_device *)
		dev_get_drvdata(dev);
	int ret;

	pm_qos_update_request(&isp->pm_qos, ATOMISP_MAX_ISR_LATENCY);
	if (isp->sw_contex.power_state == ATOM_ISP_POWER_DOWN) {
		/*Turn on ISP d-phy */
		ret = atomisp_ospm_dphy_up(isp);
		if (ret) {
			v4l2_err(&atomisp_dev,
				    "Failed to power up ISP!.\n");
			return -EINVAL;
		}
	}

	/*restore register values for iUnit and iUnitPHY registers*/
	if (isp->hw_contex.pcicmdsts)
		atomisp_restore_iunit_reg(isp);
	atomisp_save_iunit_reg(isp);

	return 0;
}

static int atomisp_suspend(struct device *dev)
{
	struct atomisp_device *isp = (struct atomisp_device *)
		dev_get_drvdata(dev);
	int ret;

	/*
	 * FIXME: Suspend is not supported by sensors. Abort if any video
	 * node was opened.
	 */
	if (isp->sw_contex.init == true)
		return -EBUSY;

	if (isp->sw_contex.isp_streaming) {
		v4l2_err(&atomisp_dev,
			    "atomisp cannot suspend at this time.\n");
		return -EINVAL;
	}

	/* save IUnit context */
	atomisp_save_iunit_reg(isp);

	/*Turn off the ISP d-phy */
	ret = atomisp_ospm_dphy_down(isp);
	if (ret)
		v4l2_err(&atomisp_dev,
			    "fail to power off ISP\n");
	else
		pm_qos_update_request(&isp->pm_qos, PM_QOS_DEFAULT_VALUE);

	return ret;
}

static int atomisp_resume(struct device *dev)
{
	struct atomisp_device *isp = (struct atomisp_device *)
		dev_get_drvdata(dev);
	int ret;

	pm_qos_update_request(&isp->pm_qos, ATOMISP_MAX_ISR_LATENCY);

	/*Turn on ISP d-phy */
	ret = atomisp_ospm_dphy_up(isp);
	if (ret) {
		v4l2_err(&atomisp_dev,
			    "Failed to power up ISP!.\n");
		return -EINVAL;
	}

	/*restore register values for iUnit and iUnitPHY registers*/
	if (isp->hw_contex.pcicmdsts)
		atomisp_restore_iunit_reg(isp);
	atomisp_save_iunit_reg(isp);

	return 0;
}
#endif

static int atomisp_subdev_probe(struct atomisp_device *isp)
{
	struct atomisp_platform_data *pdata = NULL;
	struct intel_v4l2_subdev_table *subdevs;
	struct v4l2_subdev *subdev = NULL;
	struct i2c_adapter *adapter = NULL;
	struct i2c_board_info *board_info;
	int raw_index = -1;

	/*
	 * fixing me!
	 * currently no function intel_get_v4l2_subdev_table()
	 * defined in board specific source code
	 */
#ifndef CONFIG_X86_MRFLD
	pdata = (struct atomisp_platform_data *)intel_get_v4l2_subdev_table();
#else
	pdata = NULL;
#endif
	if (pdata == NULL) {
		v4l2_err(&atomisp_dev, "no platform data available\n");
		return -ENODEV;
	}

	for (subdevs = pdata->subdevs; subdevs->type; ++subdevs) {
		board_info = &subdevs->v4l2_subdev.board_info;

		adapter = i2c_get_adapter(subdevs->v4l2_subdev.i2c_adapter_id);
		if (adapter == NULL) {
			v4l2_err(&atomisp_dev,
				    "Failed to find i2c adapter for subdev %s\n"
				    , board_info->type);
			break;
		}

		subdev = v4l2_i2c_new_subdev_board(&isp->v4l2_dev, adapter,
				board_info, NULL);

		if (subdev == NULL) {
			v4l2_warn(&atomisp_dev,
				    "Subdev %s detection fail\n",
				    board_info->type);
			continue;
		}

		v4l2_info(&atomisp_dev,
			    "Subdev %s successfully register\n",
			  board_info->type);

		switch (subdevs->type) {
		case RAW_CAMERA:
			raw_index = isp->input_cnt;
			v4l2_dbg(2, dbg_level, &atomisp_dev,
					"%s, raw_index: %d\n", __func__, raw_index);
		case SOC_CAMERA:
			v4l2_dbg(2, dbg_level, &atomisp_dev,
					"%s, SOC_INDEX: %d\n", __func__, isp->input_cnt);
			if (isp->input_cnt >= ATOM_ISP_MAX_INPUTS) {
				v4l2_warn(&atomisp_dev,
					"too many atomisp inputs, ignored\n");
				break;
			}

			isp->inputs[isp->input_cnt].type = subdevs->type;
			isp->inputs[isp->input_cnt].port = subdevs->port;
			isp->inputs[isp->input_cnt].camera = subdev;
			isp->inputs[isp->input_cnt].shading_table = NULL;
			isp->inputs[isp->input_cnt].morph_table = NULL;
			/*
			 * initialize the subdev frame size, then next we can
			 * judge whether frame_size store effective value via
			 * pixel_format.
			 */
			isp->inputs[isp->input_cnt].frame_size.pixel_format = 0;
			isp->input_cnt++;
			break;
		case CAMERA_MOTOR:
			isp->motor = subdev;
			break;
		case LED_FLASH:
		case XENON_FLASH:
			isp->flash = subdev;
			break;
		default:
			v4l2_dbg(1, dbg_level, &atomisp_dev,
				"unkonw subdev probed\n");
			break;
		}

	}

	/*
	 * HACK: Currently VCM belongs to primary sensor only, but correct
	 * approach must be to acquire from platform code which sensor
	 * owns it.
	 */
	if (isp->motor && raw_index >= 0)
		isp->inputs[raw_index].motor = isp->motor;

	/*Check camera for at least one subdev in it */
	if (!isp->inputs[0].camera) {
		v4l2_err(&atomisp_dev, "atomisp: "
		       "no camera attached or fail to detect\n");
		return -ENODEV;
	}
	return 0;
}

static void atomisp_unregister_entities(struct atomisp_device *isp)
{
	atomisp_subdev_unregister_entities(&isp->isp_subdev);
	atomisp_tpg_unregister_entities(&isp->tpg);
	atomisp_file_input_unregister_entities(&isp->file_dev);
	atomisp_mipi_csi2_unregister_entities(&isp->csi2_1p);
	atomisp_mipi_csi2_unregister_entities(&isp->csi2_4p);

	v4l2_device_unregister(&isp->v4l2_dev);
	media_device_unregister(&isp->media_dev);
}

static int atomisp_register_entities(struct atomisp_device *isp)
{
	int ret = 0;
	int i = 0;
	struct v4l2_subdev *subdev = NULL;
	struct media_entity *input = NULL;
	unsigned int flags;
	unsigned int pad;

	isp->media_dev.dev = isp->dev;

	strlcpy(isp->media_dev.model, "Intel Atom ISP",
		sizeof(isp->media_dev.model));

	ret = media_device_register(&isp->media_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev, "%s: Media device registration "
			 "failed (%d)\n", __func__, ret);
		return ret;
	}

	isp->v4l2_dev.mdev = &isp->media_dev;
	ret = v4l2_device_register(isp->dev, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"%s: V4L2 device registration failed (%d)\n",
			__func__, ret);
		goto v4l2_device_failed;
	}

	/*
	 * fixing me!
	 * not sub device exists on
	 * mrfld vp
	 */
	if (!IS_MRFLD) {
		ret = atomisp_subdev_probe(isp);
		if (ret < 0)
			goto lane4_and_subdev_probe_failed;
	}

	/* Register internal entities */
	ret =
	atomisp_mipi_csi2_register_entities(&isp->csi2_4p, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"atomisp_mipi_csi2_register_entities 4p\n");
		goto lane4_and_subdev_probe_failed;
	}

	ret =
	atomisp_mipi_csi2_register_entities(&isp->csi2_1p, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"atomisp_mipi_csi2_register_entities 1p\n");
		goto lane1_failed;
	}

	ret =
	atomisp_file_input_register_entities(&isp->file_dev, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"atomisp_file_input_register_entities\n");
		goto file_input_register_failed;
	}

	ret = atomisp_tpg_register_entities(&isp->tpg, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev, "atomisp_tpg_register_entities\n");
		goto tpg_register_failed;
	}

	ret =
	atomisp_subdev_register_entities(&isp->isp_subdev, &isp->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"atomisp_subdev_register_entities fail\n");
		goto subdev_register_failed;
	}

	for (i = 0; i < isp->input_cnt; i++) {
		subdev = isp->inputs[i].camera;
		switch (isp->inputs[i].port) {
		case ATOMISP_CAMERA_PORT_PRIMARY:
			input = &isp->csi2_4p.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = 0;
			break;
		case ATOMISP_CAMERA_PORT_SECONDARY:
			input = &isp->csi2_1p.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = 0;
			break;
		default:
			v4l2_dbg(1, dbg_level, &atomisp_dev,
				  "isp->inputs type not supported\n");
			break;
		}
		ret = media_entity_create_link(&subdev->entity, 0,
			input, pad, flags);
		if (ret < 0) {
			v4l2_err(&atomisp_dev,
				"snr to mipi csi link failed\n");
			goto link_failed;
		}
	}

	v4l2_dbg(1, dbg_level, &atomisp_dev,
		"FILE_INPUT enable, camera_cnt: %d\n", isp->input_cnt);
	isp->inputs[isp->input_cnt].type = FILE_INPUT;
	isp->inputs[isp->input_cnt].port = -1;
	isp->inputs[isp->input_cnt].shading_table = NULL;
	isp->inputs[isp->input_cnt].morph_table = NULL;
	isp->inputs[isp->input_cnt++].camera = &isp->file_dev.sd;

	if (isp->input_cnt < ATOM_ISP_MAX_INPUTS) {
		v4l2_dbg(1, dbg_level, &atomisp_dev,
			"TPG detected, camera_cnt: %d\n", isp->input_cnt);
		isp->inputs[isp->input_cnt].type = TEST_PATTERN;
		isp->inputs[isp->input_cnt].port = -1;
		isp->inputs[isp->input_cnt].shading_table = NULL;
		isp->inputs[isp->input_cnt].morph_table = NULL;
		isp->inputs[isp->input_cnt++].camera = &isp->tpg.sd;
	} else {
		v4l2_warn(&atomisp_dev,
			"too many atomisp inputs, TPG ignored.\n");
	}

	ret = v4l2_device_register_subdev_nodes(&isp->v4l2_dev);
	if (ret < 0)
		goto link_failed;

	return ret;

link_failed:
	atomisp_subdev_unregister_entities(&isp->isp_subdev);
subdev_register_failed:
	atomisp_tpg_unregister_entities(&isp->tpg);
tpg_register_failed:
	atomisp_file_input_unregister_entities(&isp->file_dev);
file_input_register_failed:
	atomisp_mipi_csi2_unregister_entities(&isp->csi2_1p);
lane1_failed:
	atomisp_mipi_csi2_unregister_entities(&isp->csi2_4p);
lane4_and_subdev_probe_failed:
	v4l2_device_unregister(&isp->v4l2_dev);
v4l2_device_failed:
	media_device_unregister(&isp->media_dev);
	return ret;
}

static int atomisp_initialize_modules(struct atomisp_device *isp)
{
	int ret;

	ret = atomisp_mipi_csi2_init(isp);
	if (ret < 0) {
		v4l2_err(&atomisp_dev, "mipi csi2 initialization failed\n");
		goto error_mipi_csi2;
	}

	ret = atomisp_file_input_init(isp);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
			"file input device initialization failed\n");
		goto error_file_input;
	}

	ret = atomisp_tpg_init(isp);
	if (ret < 0) {
		v4l2_err(&atomisp_dev, "tpg initialization failed\n");
		goto error_tpg;
	}

	ret = atomisp_subdev_init(isp);
	if (ret < 0) {
		v4l2_err(&atomisp_dev, "ISP subdev initialization failed\n");
		goto error_isp_subdev;
	}

	/* connet submoduels */
	ret = media_entity_create_link(
			&isp->csi2_4p.subdev.entity,
			CSI2_PAD_SOURCE,
			&isp->isp_subdev.subdev.entity,
			ATOMISP_SUBDEV_PAD_SINK,
			0);
	if (ret < 0)
		goto error_link;
	ret = media_entity_create_link(
			&isp->csi2_1p.subdev.entity,
			CSI2_PAD_SOURCE,
			&isp->isp_subdev.subdev.entity,
			ATOMISP_SUBDEV_PAD_SINK,
			0);
	if (ret < 0)
		goto error_link;
	return 0;

error_link:
error_isp_subdev:
	atomisp_subdev_cleanup(isp);
error_tpg:
	atomisp_tpg_cleanup(isp);
error_file_input:
	atomisp_file_input_cleanup(isp);
error_mipi_csi2:
	atomisp_mipi_csi2_cleanup(isp);
	return ret;
}

static const struct firmware *
load_firmware(struct device *dev)
{
	const struct firmware *fw;
	int rc;

	rc = request_firmware(&fw, FW_PATH, dev);
	if (rc) {
		if (rc == -ENOENT)
			v4l2_err(&atomisp_dev,
				    "Error ISP firmware %s not found.\n",
				    FW_PATH);
		else
			v4l2_err(&atomisp_dev,
				    "atomisp: Error %d while requesting"
				    " firmware %s\n", rc, FW_PATH);
		return NULL;
	}

	if (fw->data == NULL) {
		v4l2_err(&atomisp_dev,
			    "ISP firmware data is NULL.\n");
		return NULL;
	}

	return fw;
}

static struct pci_driver atomisp_pci_driver;
static int __devinit atomisp_pci_probe(struct pci_dev *dev,
					const struct pci_device_id *id)
{
	struct atomisp_device *isp = NULL;
	unsigned int start, len;
	void __iomem *base = NULL;
	int err = 0;

	if (!dev) {
		v4l2_err(&atomisp_dev, "atomisp: erorr device ptr\n");
		return -EINVAL;
	}

	atomisp_pci_vendor = id->vendor;
	atomisp_pci_device = id->device;

	err = pci_enable_device(dev);
	if (err) {
		v4l2_err(&atomisp_dev,
			    "Failed to enable CI ISP device\n");
		return err;
	}

	start = pci_resource_start(dev, 0);
	len = pci_resource_len(dev, 0);

	err = pci_request_region(dev, 0, atomisp_pci_driver.name);
	if (err) {
		v4l2_err(&atomisp_dev,
			    "Failed to request region 0x%1x-0x%Lx\n",
			    start, (unsigned long long)pci_resource_end(dev,
				0));
		goto request_region_fail;
	}

	v4l2_dbg(1, dbg_level, &atomisp_dev, "start: 0x%x\n",
			(unsigned int)start);
	v4l2_dbg(1, dbg_level, &atomisp_dev, "len: 0x%x\n",
			(unsigned int)len);

	base = ioremap_nocache(start, len);
	v4l2_dbg(1, dbg_level, &atomisp_dev, "base: 0x%x\n",
			(unsigned int)base);
	if (!base) {
		v4l2_err(&atomisp_dev,
			    "Failed to I/O memory remapping\n");
		err = -ENOMEM;
		goto ioremap_fail;
	}

	isp = kzalloc(sizeof(struct atomisp_device), GFP_KERNEL);
	if (!isp) {
		v4l2_err(&atomisp_dev, "Failed to alloc CI ISP structure\n");
		goto kzalloc_fail;
	}
	isp->sw_contex.probed = false;
	isp->sw_contex.init = false;
	isp->pdev = dev;
	isp->dev = &dev->dev;
	isp->sw_contex.power_state = ATOM_ISP_POWER_UP;
	isp->hw_contex.pci_root = pci_get_bus_and_slot(0, 0);

	/* Load isp firmware from user space */
	/*
	 * fixing me:
	 * MRFLD VP does not use firmware loading
	 * from file system
	 */
	if (!IS_MRFLD) {
		isp->firmware = load_firmware(&dev->dev);
		if (!isp->firmware) {
			v4l2_err(&atomisp_dev, "Load firmwares failed\n");
			goto load_fw_fail;
		}
	}

	err = atomisp_initialize_modules(isp);
	if (err < 0) {
		v4l2_err(&atomisp_dev, "atomisp_initialize_modules\n");
		goto init_mod_fail;
	}

	err = atomisp_register_entities(isp);
	if (err < 0) {
		v4l2_err(&atomisp_dev, "atomisp_register_entities failed\n");
		goto init_mod_fail;
	}

	INIT_LIST_HEAD(&isp->s3a_stats);
	INIT_LIST_HEAD(&isp->dis_stats);
	init_completion(&isp->wq_frame_complete);
	init_completion(&isp->dis_state_complete);
	spin_lock_init(&isp->irq_lock);

	isp->work_queue = create_singlethread_workqueue(isp->v4l2_dev.name);
	if (isp->work_queue == NULL) {
		v4l2_err(&atomisp_dev, "Failed to initialize work queue\n");
		goto work_queue_fail;
	}
	INIT_WORK(&isp->work, atomisp_work);

	isp->hw_contex.ispmmadr = start;

	pci_set_master(dev);
	atomisp_io_base = base;

	v4l2_dbg(1, dbg_level, &atomisp_dev, "atomisp_io_base: 0x%x\n",
			(unsigned int)atomisp_io_base);

	isp->tvnorm = tvnorms;
	mutex_init(&isp->input_lock);
	/* isp_lock is to protect race access of css functions */
	mutex_init(&isp->isp_lock);
	isp->sw_contex.updating_uptr = false;
	isp->isp3a_stat_ready = false;

	pci_set_drvdata(dev, isp);

	err = pci_enable_msi(dev);
	if (err) {
		v4l2_err(&atomisp_dev,
			    "Failed to enable msi\n");
		goto enable_msi_fail;
	}
	err = request_irq(dev->irq, atomisp_isr,
			  IRQF_SHARED, "isp_irq", isp);
	if (err) {
		v4l2_err(&atomisp_dev,
			    "Failed to request irq\n");
		goto request_irq_fail;
	}

	setup_timer(&isp->wdt, atomisp_wdt_wakeup_dog, (unsigned long)isp);

	atomisp_msi_irq_init(isp, dev);

	pm_qos_add_request(&isp->pm_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);
	/*
	 * fixing me!
	 * MRFLD VP does not implement
	 * PM Core
	 */
#ifdef CONFIG_PM
	if (!IS_MRFLD) {
		pm_runtime_put_noidle(&dev->dev);
		pm_runtime_allow(&dev->dev);
	}
#endif
	isp->sw_contex.probed = true;

	return 0;

request_irq_fail:
	pci_disable_msi(dev);
enable_msi_fail:
	pci_set_drvdata(dev, NULL);
	destroy_workqueue(isp->work_queue);
work_queue_fail:
	atomisp_unregister_entities(isp);
init_mod_fail:
	release_firmware(isp->firmware);
load_fw_fail:
	kfree(isp);
kzalloc_fail:
	iounmap(base);
ioremap_fail:
	pci_release_region(dev, 0);
request_region_fail:
	pci_disable_device(dev);
	return err;
}

static void __devexit atomisp_pci_remove(struct pci_dev *dev)
{
	struct atomisp_device *isp = (struct atomisp_device *)
		pci_get_drvdata(dev);

	pm_runtime_forbid(&dev->dev);
	pm_runtime_get_noresume(&dev->dev);
	pm_qos_remove_request(&isp->pm_qos);

	atomisp_msi_irq_uninit(isp, dev);
	free_irq(dev->irq, isp);
	pci_disable_msi(dev);
	pci_dev_put(isp->hw_contex.pci_root);

	atomisp_unregister_entities(isp);

	flush_workqueue(isp->work_queue);
	destroy_workqueue(isp->work_queue);

	iounmap(atomisp_io_base);
	pci_set_drvdata(dev, NULL);
	pci_release_region(dev, 0);
	pci_disable_device(dev);

	/* in case user forget to close */
	/*
	 * fixing me:
	 * MRFLD VP does not use firmware loading
	 * from file system
	 */
	if (!IS_MRFLD)
		release_firmware(isp->firmware);

	kfree(isp);
}

static DEFINE_PCI_DEVICE_TABLE(atomisp_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0148)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0149)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014A)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014B)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014C)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014D)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014E)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x014F)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08D0)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1178)},
	{0,}
};

MODULE_DEVICE_TABLE(pci, atomisp_pci_tbl);

#ifdef CONFIG_PM
static const struct dev_pm_ops atomisp_pm_ops = {
	.runtime_suspend = atomisp_runtime_suspend,
	.runtime_resume = atomisp_runtime_resume,
	.suspend = atomisp_suspend,
	.resume = atomisp_resume,
};

#define DEV_PM_OPS (&atomisp_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif

static struct pci_driver atomisp_pci_driver = {
	.driver = {
		.pm = DEV_PM_OPS,
	},
	.name = "atomisp",
	.id_table = atomisp_pci_tbl,
	.probe = atomisp_pci_probe,
	.remove = atomisp_pci_remove,
};

static int __init atomisp_init(void)
{
	return pci_register_driver(&atomisp_pci_driver);
}

static void __exit atomisp_exit(void)
{
	pci_unregister_driver(&atomisp_pci_driver);
}

module_init(atomisp_init);
module_exit(atomisp_exit);

MODULE_AUTHOR("Wen Wang <wen.w.wang@intel.com>");
MODULE_AUTHOR("Xiaolin Zhang <xiaolin.zhang@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ATOM Platform ISP Driver");
