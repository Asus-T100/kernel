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

static int mrfld_csi_lane_config(struct atomisp_device *isp)
{
	static const u8 mipi_lanes[MRFLD_PORT_CONFIG_NUM][MRFLD_PORT_NUM] = {
		{4, 1, 0},
		{3, 1, 0},
		{2, 1, 0},
		{1, 1, 0},
		{2, 1, 2},
		{3, 1, 1},
		{2, 1, 1},
		{1, 1, 1}
	};

	unsigned int i, j;
	u8 sensor_lanes[MRFLD_PORT_NUM] = {0};
	u32 data;

	for (i = 0; i < isp->input_cnt; i++) {
		struct camera_mipi_info *mipi_info;

		if (isp->inputs[i].type != RAW_CAMERA &&
			isp->inputs[i].type != SOC_CAMERA)
			continue;

		mipi_info = atomisp_to_sensor_mipi_info(isp->inputs[i].camera);
		if (!mipi_info)
			continue;

		switch (mipi_info->port) {
		case ATOMISP_CAMERA_PORT_PRIMARY:
			sensor_lanes[0] = mipi_info->num_lanes;
			break;
		case ATOMISP_CAMERA_PORT_SECONDARY:
			sensor_lanes[1] = mipi_info->num_lanes;
			break;
		case ATOMISP_CAMERA_PORT_THIRD:
			sensor_lanes[2] = mipi_info->num_lanes;
			break;
		default:
			v4l2_err(&atomisp_dev,
				"%s: invalid port: %d for the %dth sensor\n",
				__func__, mipi_info->port, i);
			break;
		}
	}

	for (i = 0; i < MRFLD_PORT_CONFIG_NUM; i++) {
		for (j = 0; j < MRFLD_PORT_NUM; j++)
			if (sensor_lanes[j]
				&& sensor_lanes[j] != mipi_lanes[i][j])
				break;

		if (j == MRFLD_PORT_NUM)
			break;	/* matched setting is found */
	}

	if (i == MRFLD_PORT_CONFIG_NUM) {
		v4l2_err(&atomisp_dev,
			"%s: could not find the CSI port setting for %d-%d-%d\n",
			__func__, sensor_lanes[0],
			sensor_lanes[1], sensor_lanes[2]);
		return -EINVAL;
	}

	pci_read_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, &data);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
				"%s: original CSI_CONTROL is 0x%x\n",
				__func__, data);
	data &= ~MRFLD_PORT_CONFIG_MASK;
	data |= (i << MRFLD_PORT_CONFIGCODE_SHIFT)
		| (mipi_lanes[i][2] ? 0 : (1 << MRFLD_PORT3_ENABLE_SHIFT))
		| (((1 << mipi_lanes[i][0]) - 1) << MRFLD_PORT1_LANES_SHIFT)
		| (((1 << mipi_lanes[i][1]) - 1) << MRFLD_PORT2_LANES_SHIFT)
		| (((1 << mipi_lanes[i][2]) - 1) << MRFLD_PORT3_LANES_SHIFT);

	pci_write_config_dword(isp->pdev, MRFLD_PCI_CSI_CONTROL, data);

	v4l2_dbg(3, dbg_level, &atomisp_dev,
		"%s: the portconfig is %d-%d-%d, CSI_CONTROL is 0x%x\n",
		__func__, mipi_lanes[i][0], mipi_lanes[i][1],
		mipi_lanes[i][2], data);

	return 0;
}


static int atomisp_subdev_probe(struct atomisp_device *isp)
{
	const struct atomisp_platform_data *pdata;
	struct intel_v4l2_subdev_table *subdevs;
	int raw_index = -1;

	pdata = intel_get_v4l2_subdev_table();
	if (pdata == NULL) {
		dev_err(isp->dev, "no platform data available\n");
		return 0;
	}

	for (subdevs = pdata->subdevs; subdevs->type; ++subdevs) {
		struct v4l2_subdev *subdev;
		struct i2c_board_info *board_info =
			&subdevs->v4l2_subdev.board_info;
		struct i2c_adapter *adapter =
			i2c_get_adapter(subdevs->v4l2_subdev.i2c_adapter_id);

		if (adapter == NULL) {
			dev_err(isp->dev,
				"Failed to find i2c adapter for subdev %s\n",
				board_info->type);
			break;
		}

		subdev = v4l2_i2c_new_subdev_board(&isp->v4l2_dev, adapter,
				board_info, NULL);

		if (subdev == NULL) {
			dev_warn(isp->dev, "Subdev %s detection fail\n",
				 board_info->type);
			continue;
		}

		dev_info(isp->dev, "Subdev %s successfully register\n",
			 board_info->type);

		switch (subdevs->type) {
		case RAW_CAMERA:
			raw_index = isp->input_cnt;
			dev_dbg(isp->dev, "raw_index: %d\n", raw_index);
		case SOC_CAMERA:
			dev_dbg(isp->dev, "SOC_INDEX: %d\n", isp->input_cnt);
			if (isp->input_cnt >= ATOM_ISP_MAX_INPUTS) {
				dev_warn(isp->dev,
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
			dev_dbg(isp->dev, "unknown subdev probed\n");
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
		dev_err(isp->dev, "no camera attached or fail to detect\n");
		return -ENODEV;
	}

	if (IS_MRFLD)
		return mrfld_csi_lane_config(isp);

	return 0;
}

static void atomisp_unregister_entities(struct atomisp_device *isp)
{
	unsigned int i;

	atomisp_subdev_unregister_entities(&isp->isp_subdev);
	atomisp_tpg_unregister_entities(&isp->tpg);
	atomisp_file_input_unregister_entities(&isp->file_dev);
	for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++)
		atomisp_mipi_csi2_unregister_entities(&isp->csi2_port[i]);

	v4l2_device_unregister(&isp->v4l2_dev);
	media_device_unregister(&isp->media_dev);
}

static int atomisp_register_entities(struct atomisp_device *isp)
{
	int ret = 0;
	unsigned int i;
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

	ret = atomisp_subdev_probe(isp);
	if (ret < 0)
		goto csi_and_subdev_probe_failed;

	/* Register internal entities */
	for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++) {
		ret = atomisp_mipi_csi2_register_entities(&isp->csi2_port[i],
								&isp->v4l2_dev);
		if (ret == 0)
			continue;

		/* error case */
		v4l2_err(&atomisp_dev,
			"failed to register the CSI port: %d\n", i);
		/* deregister all registered CSI ports */
		while (i--)
			atomisp_mipi_csi2_unregister_entities(
							&isp->csi2_port[i]);

		goto csi_and_subdev_probe_failed;
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
		if (isp->inputs[i].port >= ATOMISP_CAMERA_NR_PORTS) {
			v4l2_err(&atomisp_dev,
					"isp->inputs port %d not supported\n",
					isp->inputs[i].port);
			ret = -EINVAL;
			goto link_failed;
		}

		subdev = isp->inputs[i].camera;
		input = &isp->csi2_port[isp->inputs[i].port].subdev.entity;
		pad = CSI2_PAD_SINK;
		flags = 0;

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
	for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++)
		atomisp_mipi_csi2_unregister_entities(&isp->csi2_port[i]);
csi_and_subdev_probe_failed:
	v4l2_device_unregister(&isp->v4l2_dev);
v4l2_device_failed:
	media_device_unregister(&isp->media_dev);
	return ret;
}

static int atomisp_initialize_modules(struct atomisp_device *isp)
{
	int ret;
	unsigned int i;

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
	for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++) {
		ret = media_entity_create_link(
				&isp->csi2_port[i].subdev.entity,
				CSI2_PAD_SOURCE,
				&isp->isp_subdev.subdev.entity,
				ATOMISP_SUBDEV_PAD_SINK,
				0);
		if (ret < 0)
			goto error_link;
	}
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
	char *fw_path = IS_MRFLD ? MRFLD_FW_PATH : MFLD_FW_PATH;

	rc = request_firmware(&fw, fw_path, dev);
	if (rc) {
		if (rc == -ENOENT)
			v4l2_err(&atomisp_dev,
				    "Error ISP firmware %s not found.\n",
				    fw_path);
		else
			v4l2_err(&atomisp_dev,
				    "atomisp: Error %d while requesting"
				    " firmware %s\n", rc, fw_path);
		return NULL;
	}

	if (fw->data == NULL) {
		v4l2_err(&atomisp_dev,
			    "ISP firmware data is NULL.\n");
		return NULL;
	}

	return fw;
}

#define ATOM_ISP_PCI_BAR	0

static int __devinit atomisp_pci_probe(struct pci_dev *dev,
				       const struct pci_device_id *id)
{
	struct atomisp_device *isp;
	unsigned int start;
	void __iomem *base;
	int err;

	if (!dev) {
		dev_err(&dev->dev, "atomisp: error device ptr\n");
		return -EINVAL;
	}

	atomisp_pci_vendor = id->vendor;
	atomisp_pci_device = id->device;

	err = pcim_enable_device(dev);
	if (err) {
		dev_err(&dev->dev, "Failed to enable CI ISP device (%d)\n",
			err);
		return err;
	}

	start = pci_resource_start(dev, ATOM_ISP_PCI_BAR);
	v4l2_dbg(1, dbg_level, &atomisp_dev, "start: 0x%x\n", start);

	err = pcim_iomap_regions(dev, 1 << ATOM_ISP_PCI_BAR, pci_name(dev));
	if (err) {
		dev_err(&dev->dev, "Failed to I/O memory remapping (%d)\n",
			err);
		return err;
	}

	base = pcim_iomap_table(dev)[ATOM_ISP_PCI_BAR];
	v4l2_dbg(1, dbg_level, &atomisp_dev, "base: %p\n", base);

	atomisp_io_base = base;

	v4l2_dbg(1, dbg_level, &atomisp_dev, "atomisp_io_base: %p\n",
			atomisp_io_base);

	isp = devm_kzalloc(&dev->dev, sizeof(struct atomisp_device), GFP_KERNEL);
	if (!isp) {
		dev_err(&dev->dev, "Failed to alloc CI ISP structure\n");
		return -ENOMEM;
	}
	isp->pdev = dev;
	isp->dev = &dev->dev;
	isp->sw_contex.power_state = ATOM_ISP_POWER_UP;
	isp->hw_contex.pci_root = pci_get_bus_and_slot(0, 0);
	if (!isp->hw_contex.pci_root) {
		dev_err(&dev->dev, "Unable to find PCI host\n");
		return -ENODEV;
	}
	isp->hw_contex.ispmmadr = start;
	isp->tvnorm = tvnorms;

	mutex_init(&isp->mutex);

	/* Load isp firmware from user space */
	isp->firmware = load_firmware(&dev->dev);
	if (!isp->firmware) {
		dev_err(&dev->dev, "Load firmwares failed\n");
		goto load_fw_fail;
	}

	INIT_LIST_HEAD(&isp->acc.memory_maps);
	INIT_LIST_HEAD(&isp->s3a_stats);
	INIT_LIST_HEAD(&isp->dis_stats);
	init_completion(&isp->dis_state_complete);

	isp->wdt_work_queue = alloc_workqueue(isp->v4l2_dev.name, 0, 1);
	if (isp->wdt_work_queue == NULL) {
		dev_err(&dev->dev, "Failed to initialize work queue\n");
		goto work_queue_fail;
	}
	INIT_WORK(&isp->wdt_work, atomisp_wdt_work);

	pci_set_master(dev);
	pci_set_drvdata(dev, isp);

	err = pci_enable_msi(dev);
	if (err) {
		dev_err(&dev->dev, "Failed to enable msi (%d)\n", err);
		goto enable_msi_fail;
	}

	err = devm_request_threaded_irq(&dev->dev, dev->irq,
					atomisp_isr, atomisp_isr_thread,
					IRQF_SHARED, "isp_irq", isp);
	if (err) {
		dev_err(&dev->dev, "Failed to request irq (%d)\n", err);
		goto enable_msi_fail;
	}

	setup_timer(&isp->wdt, atomisp_wdt, (unsigned long)isp);

	atomisp_msi_irq_init(isp, dev);

	pm_qos_add_request(&isp->pm_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	if (IS_MRFLD) {
		/*
		 * for MRFLD, Software/firmware needs to write a 1 to bit 0 of
		 * the register at CSI_RECEIVER_SELECTION_REG to enable SH CSI
		 * backend write 0 will enable Arasan CSI backend, which has
		 * bugs(like sighting:4567697 and 4567699) and will be removed
		 * in B0
		 */
		device_store_uint32(MRFLD_CSI_RECEIVER_SELECTION_REG, 1);
	}

	err = atomisp_initialize_modules(isp);
	if (err < 0) {
		dev_err(&dev->dev, "atomisp_initialize_modules (%d)\n", err);
		goto enable_msi_fail;
	}

	err = atomisp_register_entities(isp);
	if (err < 0) {
		dev_err(&dev->dev, "atomisp_register_entities failed (%d)\n",
			err);
		goto enable_msi_fail;
	}

	pm_runtime_put_noidle(&dev->dev);
	pm_runtime_allow(&dev->dev);

	return 0;

enable_msi_fail:
	destroy_workqueue(isp->wdt_work_queue);
work_queue_fail:
	release_firmware(isp->firmware);
load_fw_fail:
	pci_dev_put(isp->hw_contex.pci_root);
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
	pci_dev_put(isp->hw_contex.pci_root);

	atomisp_unregister_entities(isp);

	destroy_workqueue(isp->wdt_work_queue);

	release_firmware(isp->firmware);
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
