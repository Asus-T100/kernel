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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* atomisp_dbg */
#include <linux/version.h>	/* atomisp_dbg */
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/slab.h>		/* for kmalloc */
#include <linux/mm.h>		/* for GFP_ATOMIC */
#include <linux/pci.h>		/* for DMA */
#include <linux/pm_runtime.h>	/* for runtime pm */
#include <linux/stringify.h>
#include <linux/io.h>		/* ioremap */
#include <linux/uaccess.h>	/* access_ok */
#include <linux/param.h>	/* access_ok */
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/poll.h>
#include <linux/stddef.h>
#include <linux/firmware.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>
#include <media/videobuf-vmalloc.h>

#ifndef CONFIG_X86_MRFLD
#include <hmm/hmm.h>
#endif

#include <sh_css.h>

#include "atomisp_internal.h"

extern int dbg_level;
extern int mipicsi_flag;
extern int pad_w;
extern int pad_h;

#define PUNIT_PORT		0x04

#define IUNIT_PORT_MDFLD		0x08
/*
 * Fixing me!
 * Not that IUnit port change to 0x1C according to
 * IUnit HAS 0.95_R6, but currently VP4.0.0 still
 * use the old 0x15 value
 */
#define IUNIT_PORT_MRFLD		0x15

#define IUNITPHY_PORT		0x09

#define CSI_RCOMP		0x00
#define PCICMDSTS		0x01
#define CSI_AFE			0x02
#define CSI_CONTROL		0x03
#define ISPMMADR		0x04
#define INTR			0x0f
#define MSI_CAPID		0x24
#define MSI_ADDRESS		0x25
#define MSI_DATA		0x26
#define INTR_CTL		0x27
#define PMCS			0x35
#define CG_DIS			0x36
#define I_CONTROL		0x3f
#define OR1			0x72
#define APMBA			0x7a

#define PCI_MSI_CAPID		0x90
#define PCI_MSI_ADDR		0x94
#define PCI_MSI_DATA		0x98
#define PCI_INTERRUPT_CTRL	0x9C
#define PCI_PRI_D0		0xD0
#define PCI_PMCS		0xD4
#define PCI_CG_DIS		0xD8
#define PCI_I_CONTROL		0xFC
/*
 * Enables the combining of adjacent 32-byte read requests to the same
 * cache line. When cleared, each 32-byte read request is sent as a
 * separate request on the IB interface.
 */
#define PCI_I_CONTROL_ENABLE_READ_COMBINING	0x10000

/*
 * Enables the combining of adjacent 32-byte write requests to the same
 * cache line. When cleared, each 32-byte write request is sent as a
 * separate request on the IB interface.
 */
#define PCI_I_CONTROL_ENABLE_WRITE_COMBINING	0x20000

#define PCI_MEM_ACCESS		0x100002

extern int atomisp_pci_vendor;
extern int atomisp_pci_device;

#define IS_MRFLD ((atomisp_pci_device & 0xfff8) == 0x1178)

struct atomisp_tvnorm {
	char *name;
	v4l2_std_id id;
	u32 cxiformat;
	u32 cxoformat;
};

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
