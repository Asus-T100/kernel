/* Release Version: ci_master_byt_20130905_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#ifndef __IA_CSS_I_RMGR_VBUF_HOST_SHARED_H_INCLUDED__
#define __IA_CSS_I_RMGR_VBUF_HOST_SHARED_H_INCLUDED__

#include "ia_css_i_rmgr_vbuf_public.h"

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "sh_css_internal.h"

/* complete the types for the sp implementation of the dma resource pool */

struct ia_css_i_host_rmgr_vbuf_handle {
	hrt_vaddress vptr;
	uint8_t count;
	uint32_t size;
};

struct ia_css_i_host_rmgr_vbuf_pool {
	uint8_t copy_on_write;
	uint8_t recycle;
	uint32_t size;
	uint32_t index;
	struct ia_css_i_host_rmgr_vbuf_handle **handles;
};

extern struct ia_css_i_host_rmgr_vbuf_pool *vbuf_ref;
extern struct ia_css_i_host_rmgr_vbuf_pool *vbuf_write;
extern struct ia_css_i_host_rmgr_vbuf_pool *hmm_buffer_pool;

#endif /* __IA_CSS_I_RMGR_VBUF_HOST_SHARED_H_INCLUDED__ */
