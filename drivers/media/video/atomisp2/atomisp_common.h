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

#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#include <asm/irq.h>

#include <hmm/hmm.h>

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>		/* ioremap */
#include <linux/ioport.h>
#include <linux/kernel.h>	/* atomisp_dbg */
#include <linux/mm.h>		/* for GFP_ATOMIC */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/param.h>	/* access_ok */
#include <linux/pci.h>		/* for DMA */
#include <linux/pm_runtime.h>	/* for runtime pm */
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>		/* for kmalloc */
#include <linux/stddef.h>
#include <linux/stringify.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>	/* access_ok */
#include <linux/version.h>	/* atomisp_dbg */
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>

#include <media/videobuf-core.h>
#include <media/videobuf-vmalloc.h>

#include <sh_css.h>

#include "atomisp_internal.h"
#include <linux/atomisp.h>

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
