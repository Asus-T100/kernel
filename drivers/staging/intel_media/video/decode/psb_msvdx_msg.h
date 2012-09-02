/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
 * Copyright (c) Imagination Technologies Limited, UK
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors:
 *    Fei Jiang <fei.jiang@intel.com>
 *    Li Zeng <li.zeng@intel.com>
 *
 **************************************************************************/

#ifndef _PSB_MSVDX_MSG_H_
#define _PSB_MSVDX_MSG_H_

#include "psb_drv.h"
#include "img_types.h"

/* This type defines the framework specified message ids */
enum {
	/* ! Sent by the VA driver on the host to the mtx firmware.
	 */
	MTX_MSGID_PADDING = 0,
	MTX_MSGID_INIT = FWRK_MSGID_START_PSR_HOSTMTX_MSG,
	MTX_MSGID_DECODE_FE,
	MTX_MSGID_DEBLOCK,
	MTX_MSGID_INTRA_OOLD,
	MTX_MSGID_DECODE_BE,
	MTX_MSGID_HOST_BE_OPP,

	/*! Sent by the mtx firmware to itself.
	 */
	MTX_MSGID_RENDER_MC_INTERRUPT,

	/* used to ditinguish mrst and mfld */
	MTX_MSGID_DEBLOCK_MFLD = FWRK_MSGID_HOST_EMULATED,
	MTX_MSGID_INTRA_OOLD_MFLD,
	MTX_MSGID_DECODE_BE_MFLD,
	MTX_MSGID_HOST_BE_OPP_MFLD,

	/*! Sent by the DXVA firmware on the MTX to the host.
	 */
	MTX_MSGID_COMPLETED = FWRK_MSGID_START_PSR_MTXHOST_MSG,
	MTX_MSGID_COMPLETED_BATCH,
	MTX_MSGID_DEBLOCK_REQUIRED,
	MTX_MSGID_TEST_RESPONCE,
	MTX_MSGID_ACK,
	MTX_MSGID_FAILED,
	MTX_MSGID_CONTIGUITY_WARNING,
	MTX_MSGID_HW_PANIC,
};

#define MTX_GENMSG_SIZE_TYPE		uint8_t
#define MTX_GENMSG_SIZE_MASK		(0xFF)
#define MTX_GENMSG_SIZE_SHIFT		(0)
#define MTX_GENMSG_SIZE_OFFSET		(0x0000)

#define MTX_GENMSG_ID_TYPE		uint8_t
#define MTX_GENMSG_ID_MASK		(0xFF)
#define MTX_GENMSG_ID_SHIFT		(0)
#define MTX_GENMSG_ID_OFFSET		(0x0001)

#define MTX_GENMSG_HEADER_SIZE 2

#define MTX_GENMSG_FENCE_TYPE		uint16_t
#define MTX_GENMSG_FENCE_MASK		(0xFFFF)
#define MTX_GENMSG_FENCE_OFFSET		(0x0002)
#define MTX_GENMSG_FENCE_SHIFT		(0)

#define FW_INVALIDATE_MMU		(0x0010)

struct fw_init_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t reserved:16;
		} bits;
		uint32_t value;
	} header;
	uint32_t rendec_addr0;
	uint32_t rendec_addr1;
	union {
		struct {
			uint32_t rendec_size0:16;
			uint32_t rendec_size1:16;
		} bits;
		uint32_t value;
	} rendec_size;
};

struct fw_decode_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
	union {
		struct {
			uint32_t flags:16;
			uint32_t buffer_size:16;
		} bits;
		uint32_t value;
	} flag_size;
	uint32_t crtl_alloc_addr;
	union {
		struct {
			uint32_t context:8;
			uint32_t mmu_ptd:24;
		} bits;
		uint32_t value;
	} mmu_context;
	uint32_t operating_mode;
};

struct fw_deblock_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
	union {
		struct {
			uint32_t flags:16;
			uint32_t slice_field_type:2;
			uint32_t reserved:14;
		} bits;
		uint32_t value;
	} flag_type;
	uint32_t operating_mode;
	union {
		struct {
			uint32_t context:8;
			uint32_t mmu_ptd:24;
		} bits;
		uint32_t value;
	} mmu_context;
	union {
		struct {
			uint32_t frame_height_mb:16;
			uint32_t pic_width_mb:16;
		} bits;
		uint32_t value;
	} pic_size;
	uint32_t address_a0;
	uint32_t address_a1;
	uint32_t mb_param_address;
	uint32_t ext_stride_a;
	uint32_t address_b0;
	uint32_t address_b1;
	uint32_t alt_output_flags_b;
	/* additional msg outside of IMG msg */
	uint32_t address_c0;
	uint32_t address_c1;
};

struct fw_padding_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
		} bits;
		uint16_t value;
	} header;
};

struct fw_completed_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
	union {
		struct {
			uint32_t last_mb:16;
			uint32_t start_mb:16;
		} bits;
		uint32_t value;
	} mb;
	uint32_t flags;
	uint32_t vdebcr;
	uint32_t fe_begin_setup;
	uint32_t fe_begin_decode;
	uint32_t fe_end_decode;
	uint32_t be_begin_setup;
	uint32_t be_begin_decode;
	uint32_t be_end_decode;
};

struct fw_deblock_required_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
};

struct fw_panic_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
	uint32_t fe_status;
	uint32_t be_status;
	union {
		struct {
			uint32_t last_mb:16;
			uint32_t reserved2:16;
		} bits;
		uint32_t value;
	} mb;
};

struct fw_contiguity_msg {
	union {
		struct {
			uint32_t msg_size:8;
			uint32_t msg_type:8;
			uint32_t msg_fence:16;
		} bits;
		uint32_t value;
	} header;
	union {
		struct {
			uint32_t end_mb_num:16;
			uint32_t begin_mb_num:16;
		} bits;
		uint32_t value;
	} mb;
};

#endif
