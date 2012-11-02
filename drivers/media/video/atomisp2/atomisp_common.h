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

/* common register definitions */
#define PUNIT_PORT		0x04

#define PCICMDSTS		0x01
#define INTR			0x0f
#define MSI_CAPID		0x24
#define MSI_ADDRESS		0x25
#define MSI_DATA		0x26
#define INTR_CTL		0x27

#define PCI_MSI_CAPID		0x90
#define PCI_MSI_ADDR		0x94
#define PCI_MSI_DATA		0x98
#define PCI_INTERRUPT_CTRL	0x9C
#define PCI_I_CONTROL		0xfc

/* MFLD specific register definitions */
#define MFLD_IUNITPHY_PORT	0x09

#define MFLD_CSI_RCOMP		0x00
#define MFLD_CSI_AFE		0x02
#define MFLD_CSI_CONTROL	0x03
#define MFLD_CG_DIS		0x36
#define MFLD_OR1		0x72

#define MFLD_PCI_PMCS		0xd4
#define MFLD_PCI_CG_DIS		0xd8

/*
 * Enables the combining of adjacent 32-byte read requests to the same
 * cache line. When cleared, each 32-byte read request is sent as a
 * separate request on the IB interface.
 */
#define MFLD_PCI_I_CONTROL_ENABLE_READ_COMBINING	0x10000

/*
 * Enables the combining of adjacent 32-byte write requests to the same
 * cache line. When cleared, each 32-byte write request is sent as a
 * separate request on the IB interface.
 */
#define MFLD_PCI_I_CONTROL_ENABLE_WRITE_COMBINING	0x20000

#define MFLD_MAX_ZOOM_FACTOR	64

/* MRFLD specific register definitions */
#define MRFLD_CSI_AFE		0x39
#define MRFLD_CSI_CONTROL	0x3a
#define MRFLD_CSI_RCOMP		0x3d

#define MRFLD_PCI_PMCS		0x84
#define MRFLD_PCI_CSI_ACCESS_CTRL_VIOL	0xd4
#define MRFLD_PCI_CSI_AFE_HS_CONTROL	0xdc
#define MRFLD_PCI_CSI_AFE_RCOMP_CONTROL	0xe0
#define MRFLD_PCI_CSI_CONTROL		0xe8
#define MRFLD_PCI_CSI_AFE_TRIM_CONTROL	0xe4
#define MRFLD_PCI_CSI_DEADLINE_CONTROL	0xec
#define MRFLD_PCI_CSI_RCOMP_CONTROL	0xf4

/*
 * Enables the combining of adjacent 32-byte read requests to the same
 * cache line. When cleared, each 32-byte read request is sent as a
 * separate request on the IB interface.
 */
#define MRFLD_PCI_I_CONTROL_ENABLE_READ_COMBINING	0x1

/*
 * Enables the combining of adjacent 32-byte write requests to the same
 * cache line. When cleared, each 32-byte write request is sent as a
 * separate request on the IB interface.
 */
#define MRFLD_PCI_I_CONTROL_ENABLE_WRITE_COMBINING	0x2

/*
 * This register is IUINT MMIO register, it is used to select the CSI
 * receiver backend.
 * 1: SH CSI backend
 * 0: Arasan CSI backend
 */
#define MRFLD_CSI_RECEIVER_SELECTION_REG       0x8081c

#define MRFLD_MAX_ZOOM_FACTOR	1024

/* MRFLD ISP POWER related */
#define MRFLD_ISPSSPM0         0x39
#define MRFLD_ISPSSPM0_ISPSSC_OFFSET   0
#define MRFLD_ISPSSPM0_ISPSSS_OFFSET   24
#define MRFLD_ISPSSPM0_IUNIT_POWER_ON  0
#define MRFLD_ISPSSPM0_IUNIT_POWER_OFF 0x3

/* MRFLD CSI lane configuration related */
#define MRFLD_PORT_CONFIG_NUM  8
#define MRFLD_PORT_NUM         3
#define MRFLD_PORT3_ENABLE_SHIFT       2
#define MRFLD_PORT1_LANES_SHIFT        3
#define MRFLD_PORT2_LANES_SHIFT        7
#define MRFLD_PORT3_LANES_SHIFT        8
#define MRFLD_PORT_CONFIGCODE_SHIFT    16
#define MRFLD_PORT_CONFIG_MASK 0x000f03ff

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
