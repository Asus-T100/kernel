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
#ifndef ATOMISP_SUBDEV_H_
#define ATOMISP_SUBDEV_H_

#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "sh_css.h"

enum atomisp_subdev_input_entity {
	ATOMISP_SUBDEV_INPUT_NONE,
	ATOMISP_SUBDEV_INPUT_MEMORY,
	ATOMISP_SUBDEV_INPUT_CSI2,
	ATOMISP_SUBDEV_INPUT_CSI2_4P,
	ATOMISP_SUBDEV_INPUT_CSI2_1P
};

#define ATOMISP_SUBDEV_PAD_SINK			0
#define ATOMISP_SUBDEV_PAD_SOURCE_MO		1 /* regular output */
#define ATOMISP_SUBDEV_PAD_SOURCE_SS		2 /* snapshot output */
#define ATOMISP_SUBDEV_PAD_SOURCE_VF		3 /* viewfinder output */
#define ATOMISP_SUBDEV_PADS_NUM			4

#define ATOMISP_PIPE_MASTEROUTPUT		0
#define ATOMISP_PIPE_SNAPSHOT			1
#define ATOMISP_PIPE_VIEWFINDER			2
#define ATOMISP_PIPE_FILEINPUT			3

struct atomisp_3a_dis_stat_buf {
	union sh_css_s3a_data s3a_data;
	struct sh_css_dis_data dis_data;
	struct list_head list;
};

struct atomisp_s3a_buf {
	union sh_css_s3a_data s3a_data;
	struct list_head list;
};

struct atomisp_dis_buf {
	struct sh_css_dis_data dis_data;
	struct list_head list;
};

struct atomisp_video_pipe {
	struct video_device vdev;
	enum v4l2_buf_type type;
	struct media_pad pad;
	struct videobuf_queue capq;
	struct videobuf_queue outq;
	struct list_head activeq;
	struct list_head activeq_out;
	struct mutex mutex;

	spinlock_t irq_lock;
	bool opened;
	unsigned int pipe_type;

	struct atomisp_device *isp;
	struct atomisp_fmt *out_fmt;
	struct atomisp_video_pipe_format *format;
};

struct atomisp_sub_device {
	struct v4l2_subdev subdev;
	struct media_pad pads[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_mbus_framefmt formats[ATOMISP_SUBDEV_PADS_NUM];

	enum atomisp_subdev_input_entity input;
	unsigned int output;
	struct atomisp_video_pipe video_in;
	struct atomisp_video_pipe video_out_mo; /* main output */
	struct atomisp_video_pipe video_out_ss; /* downscaled capture output */
	struct atomisp_video_pipe video_out_vf; /* view finder */
	/* struct isp_subdev_params params; */
	spinlock_t lock;
	struct atomisp_device *isp;
};

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *isp_subdev);
int atomisp_subdev_register_entities(struct atomisp_sub_device *isp_subdev,
	struct v4l2_device *vdev);
int atomisp_subdev_init(struct atomisp_device *isp);
void atomisp_subdev_cleanup(struct atomisp_device *isp);

#endif
