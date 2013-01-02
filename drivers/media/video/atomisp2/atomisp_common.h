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

#ifndef	__ATOMISP_COMMON_H__
#define	__ATOMISP_COMMON_H__

#include <linux/atomisp.h>

#include <linux/v4l2-mediabus.h>

#include <media/videobuf-core.h>

#include <sh_css.h>

extern int dbg_level;
extern int mipicsi_flag;
extern int pad_w;
extern int pad_h;

extern int atomisp_pci_vendor;
extern int atomisp_pci_device;

#define MFLD_MAX_ZOOM_FACTOR	64

#define MRFLD_MAX_ZOOM_FACTOR	1024

#define IS_MRFLD ((atomisp_pci_device & 0xfff8) == 0x1178)

struct atomisp_format_bridge {
	unsigned int pixelformat;
	unsigned int depth;
	enum v4l2_mbus_pixelcode mbus_code;
	enum sh_css_frame_format sh_fmt;
	unsigned char description[32];	/* the same as struct v4l2_fmtdesc */
};

struct atomisp_resolution {
	u32 width;
	u32 height;
};

struct atomisp_fmt {
	u32 pixelformat;
	u32 depth;
	u32 bytesperline;
	u32 framesize;
	u32 imagesize;
	u32 width;
	u32 height;
	u32 bayer_order;
};

struct atomisp_video_pipe_format {
	struct v4l2_pix_format out;
	struct v4l2_pix_format in;
	unsigned int out_sh_fmt;
};

struct atomisp_buffer {
	struct videobuf_buffer	vb;
};

/*
 * supported V4L2 fmts and resolutions
 */
extern const struct atomisp_format_bridge atomisp_output_fmts[];
extern const u32 atomisp_output_fmts_num;
extern struct v4l2_device atomisp_dev;
#endif
