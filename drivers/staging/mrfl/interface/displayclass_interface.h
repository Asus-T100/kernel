/*****************************************************************************
 *
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************/

#ifndef __DC_INTERFACE_H__
#define __DC_INTERFACE_H__

/*NOTE: this file is exported to user mode*/

#ifdef __KERNEL__

#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include "ttm/ttm_bo_driver.h"
#include <ttm/ttm_bo_api.h>

/*
 * TODO: remove these macros as we don't need swapchains anymore
 */
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE (0 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A    (1 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B    (1 << 1)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C    (1 << 2)

/*
 * TODO: move this definition back to psb_fb.h
 * This is NOT a part of IMG display class
 */
struct psb_framebuffer {
	struct drm_framebuffer base;
	struct address_space *addr_space;
	struct ttm_buffer_object *bo;
	struct fb_info *fbdev;
	uint32_t tt_pages;
	uint32_t stolen_base;
	void *vram_addr;
	/* struct ttm_bo_kmap_obj kmap; */
	void *hKernelMemInfo;
	uint32_t depth;
	uint32_t size;
	uint32_t offset;
};

#endif /*__KERNEL__*/

typedef enum intel_dc_plane_types {
	DC_UNKNOWN_PLANE = 0,
	DC_SPRITE_PLANE = 1,
	DC_PRIMARY_PLANE,
	DC_OVERLAY_PLANE,
} DC_MRFLD_PLANE_TYPE;

#define SPRITE_UPDATE_SURFACE			(0x00000001UL)
#define SPRITE_UPDATE_CONTROL			(0x00000002UL)
#define SPRITE_UPDATE_POSITION			(0x00000004UL)
#define SPRITE_UPDATE_SIZE			(0x00000008UL)
#define SPRITE_UPDATE_WAIT_VBLANK		(0X00000010UL)
#define SPRITE_UPDATE_ALL			(0x0000001fUL)

typedef struct intel_dc_overlay_ctx {
	uint32_t index;
	uint32_t pipe;
	uint32_t ovadd;
} DC_MRFLD_OVERLAY_CONTEXT;

typedef struct intel_dc_sprite_ctx {
	uint32_t update_mask;
	/*plane index 0-A, 1-B, 2-C,etc*/
	uint32_t index;
	uint32_t pipe;

	uint32_t cntr;
	uint32_t linoff;
	uint32_t stride;
	uint32_t pos;
	uint32_t size;
	uint32_t keyminval;
	uint32_t keymask;
	uint32_t surf;
	uint32_t keymaxval;
	uint32_t tileoff;
	uint32_t contalpa;
} DC_MRFLD_SPRITE_CONTEXT;

typedef struct intel_dc_primary_ctx {
	uint32_t update_mask;
	/*plane index 0-A, 1-B, 2-C,etc*/
	uint32_t index;
	uint32_t pipe;

	uint32_t cntr;
	uint32_t linoff;
	uint32_t stride;
	uint32_t pos;
	uint32_t size;
	uint32_t keyminval;
	uint32_t keymask;
	uint32_t surf;
	uint32_t keymaxval;
	uint32_t tileoff;
	uint32_t contalpa;
} DC_MRFLD_PRIMARY_CONTEXT;

typedef struct intel_dc_plane_ctx {
	enum intel_dc_plane_types type;
	union {
		struct intel_dc_overlay_ctx ov_ctx;
		struct intel_dc_sprite_ctx sp_ctx;
		struct intel_dc_primary_ctx prim_ctx;
	} ctx;
} DC_MRFLD_SURF_CUSTOM;

IMG_VOID DCUnAttachPipe(IMG_UINT32 uiPipe);

#endif				/* __DC_INTERFACE_H__ */
