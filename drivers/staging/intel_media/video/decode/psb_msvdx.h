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
 *    Li Zeng <li.zeng@intel.com>
 *    Fei Jiang <fei.jiang@intel.com>
 *
 **************************************************************************/

#ifndef _PSB_MSVDX_H_
#define _PSB_MSVDX_H_

#include "psb_drv.h"
#include "img_types.h"
#include "psb_msvdx_reg.h"

extern int drm_msvdx_pmpolicy;
extern int drm_msvdx_delay;
extern int hdmi_state;
extern int drm_psb_msvdx_tiling;

#define FIRMWAREID		0x014d42ab

/* psb_mmu.c */
uint32_t psb_get_default_pd_addr(struct psb_mmu_driver *driver);

/* psb_msvdxinit.c */
int psb_wait_for_register(struct drm_psb_private *dev_priv,
			  uint32_t offset, uint32_t value, uint32_t enable,
			  uint32_t poll_cnt, uint32_t timeout);
int psb_msvdx_init(struct drm_device *dev);
int psb_msvdx_uninit(struct drm_device *dev);
int psb_msvdx_core_reset(struct drm_psb_private *dev_priv);
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
/* TODO: psb_msvdx_reset is used for the case of fw loading by driver
 * Later we can test if it can be removed. */
int psb_msvdx_reset(struct drm_psb_private *dev_priv);
#endif
int psb_msvdx_post_boot_init(struct drm_device *dev);

/* psb_msvdx.c */
IMG_BOOL psb_msvdx_interrupt(IMG_VOID *pvData);
int psb_mtx_send(struct drm_psb_private *dev_priv, const void *pvMsg);
#if 0
void psb_msvdx_lockup(struct drm_psb_private *dev_priv,
		      int *msvdx_lockup, int *msvdx_idle);
#endif
int psb_check_msvdx_idle(struct drm_device *dev);
int psb_wait_msvdx_idle(struct drm_device *dev);
int psb_cmdbuf_video(struct drm_file *priv,
		     struct list_head *validate_list,
		     uint32_t fence_type,
		     struct drm_psb_cmdbuf_arg *arg,
		     struct ttm_buffer_object *cmd_buffer,
		     struct psb_ttm_fence_rep *fence_arg);
int psb_msvdx_save_context(struct drm_device *dev);
int psb_msvdx_restore_context(struct drm_device *dev);
void psb_msvdx_check_reset_fw(struct drm_device *dev);
void psb_powerdown_msvdx(struct work_struct *work);
void psb_msvdx_flush_cmd_queue(struct drm_device *dev);


/* psb_msvdx_fw.c */
int32_t psb_msvdx_alloc_fw_bo(struct drm_psb_private *dev_priv);
int psb_setup_fw(struct drm_device *dev);
int psb_setup_msvdx(struct drm_device *dev);

/*  Non-Optimal Invalidation is not default */
#define MSVDX_DEVICE_NODE_FLAGS_MMU_NONOPT_INV	2

#define FW_VA_RENDER_HOST_INT		0x00004000
#define MSVDX_DEVICE_NODE_FLAGS_MMU_HW_INVALIDATION	0x00000020

/* There is no work currently underway on the hardware */
#define MSVDX_FW_STATUS_HW_IDLE	0x00000001
#define MSVDX_DEVICE_NODE_FLAG_BRN23154_BLOCK_ON_FE	0x00000200
#define MSVDX_DEVICE_NODE_FLAGS_DEFAULT_D0 \
	(MSVDX_DEVICE_NODE_FLAGS_MMU_NONOPT_INV |			\
		MSVDX_DEVICE_NODE_FLAGS_MMU_HW_INVALIDATION |		\
		MSVDX_DEVICE_NODE_FLAG_BRN23154_BLOCK_ON_FE)

#define MSVDX_DEVICE_NODE_FLAGS_DEFAULT_D1 \
	(MSVDX_DEVICE_NODE_FLAGS_MMU_HW_INVALIDATION |			\
		MSVDX_DEVICE_NODE_FLAG_BRN23154_BLOCK_ON_FE)

#define POULSBO_D0	0x5
#define POULSBO_D1	0x6
#define PSB_REVID_OFFSET 0x8

#define MTX_CODE_BASE		(0x80900000)
#define MTX_DATA_BASE		(0x82880000)
#define PC_START_ADDRESS	(0x80900000)

#define MTX_CORE_CODE_MEM	(0x10)
#define MTX_CORE_DATA_MEM	(0x18)

#define RENDEC_A_SIZE	(4 * 1024 * 1024)
#define RENDEC_B_SIZE	(1024 * 1024)

#define MSVDX_RESET_NEEDS_REUPLOAD_FW		(0x2)
#define MSVDX_RESET_NEEDS_INIT_FW		(0x1)

/* HOST_BE_OPP parameters */
struct HOST_BE_OPP_PARAMS {
	uint32_t handle;	/* struct ttm_buffer_object * of REGIO */
	uint32_t buffer_stride;
	uint32_t buffer_size;
	uint32_t picture_width_mb;
	uint32_t size_mb;
};

typedef struct drm_psb_msvdx_frame_info {
	uint32_t handle;
	uint32_t surface_id;
	uint32_t fence;
	uint32_t buffer_stride;
	uint32_t buffer_size;
	uint32_t picture_width_mb;
	uint32_t fw_status;
	uint32_t size_mb;
	drm_psb_msvdx_decode_status_t decode_status;
} drm_psb_msvdx_frame_info_t;

#define MAX_DECODE_BUFFERS (24)
#define PSB_MAX_EC_INSTANCE (4)
#define PSB_MSVDX_INVALID_FENCE (0xffffffff)
#define PSB_MSVDX_INVALID_OFFSET (0xffffffff)
#define PSB_MSVDX_EC_ROLLBACK (9)

struct psb_msvdx_ec_ctx {
	struct ttm_object_file *tfile; /* protected by cmdbuf_mutex */
	uint32_t context_id;
	drm_psb_msvdx_frame_info_t frame_info[MAX_DECODE_BUFFERS];
	drm_psb_msvdx_frame_info_t *cur_frame_info;
	int frame_idx;

	/* 12 render msg + 1 deblock msg
	 * 12 * 20 + 1 * 48 = 288;
	*/
	unsigned char unfenced_cmd[300];
	uint32_t cmd_size;
	uint32_t deblock_cmd_offset;
	uint32_t fence;
	drm_psb_msvdx_decode_status_t decode_status;
};

/* MSVDX private structure */
struct msvdx_private {
	struct drm_device *dev;
	int msvdx_needs_reset;

	unsigned int pmstate;

	struct sysfs_dirent *sysfs_pmstate;

	uint32_t msvdx_current_sequence;
	uint32_t msvdx_last_sequence;

	struct drm_psb_private *dev_priv;
	/*
	 *MSVDX Rendec Memory
	 */
	struct ttm_buffer_object *ccb0;
	uint32_t base_addr0;
	struct ttm_buffer_object *ccb1;
	uint32_t base_addr1;

	struct ttm_buffer_object *fw;
	uint32_t is_load;
	uint32_t mtx_mem_size;

	/*
	 *MSVDX tile regions
	*/
	uint32_t tile_region_start0;
	uint32_t tile_region_end0;
	uint32_t tile_region_start1;
	uint32_t tile_region_end1;

	/*
	 *msvdx command queue
	 */
	spinlock_t msvdx_lock;
	struct mutex msvdx_mutex;
	struct list_head msvdx_queue;
	int msvdx_busy;
	int rendec_init;
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	int msvdx_fw_loaded;
	void *msvdx_fw;
	int msvdx_fw_size;
#endif
	uint32_t msvdx_hw_busy;

#ifdef PSB_MSVDX_SAVE_RESTORE_VEC
	uint32_t *vec_local_mem_data;
	uint32_t vec_local_mem_size;
	uint32_t vec_local_mem_saved;
#endif

#ifdef CONFIG_VIDEO_MRFLD
	uint32_t vec_ec_mem_data[4];
	uint32_t vec_ec_mem_saved;
#endif

	uint32_t psb_dash_access_ctrl;
	uint32_t decoding_err;
	uint32_t fw_loaded_by_punit;

	drm_psb_msvdx_frame_info_t frame_info[MAX_DECODE_BUFFERS];
	drm_psb_msvdx_decode_status_t decode_status;
	uint32_t host_be_opp_enabled;

	uint32_t ec_fence;
	uint32_t ref_pic_fence;
	/*work for error concealment*/
	struct work_struct ec_work;
	struct ttm_object_file *tfile; /* protected by cmdbuf_mutex */
	struct psb_msvdx_ec_ctx *msvdx_ec_ctx[PSB_MAX_EC_INSTANCE];
	struct psb_msvdx_ec_ctx *cur_msvdx_ec_ctx;
	uint32_t deblock_cmd_offset;
	int num_cmd;

	struct drm_video_displaying_frameinfo displaying_frame;

	/* pm suspend wq */
	struct delayed_work msvdx_suspend_wq;

	/* protected by msvdx_mutex */
	/* Current video context */
	struct psb_video_ctx *msvdx_ctx;
	/* previous vieo context */
	struct psb_video_ctx *last_msvdx_ctx;
	uint32_t pm_gating_count;

	struct page *mmu_recover_page;
};

struct psb_msvdx_cmd_queue {
	struct list_head head;
	void *cmd;
	unsigned long cmd_size;
	uint32_t sequence;
	uint32_t msvdx_tile;
	uint32_t host_be_opp_enabled;
	uint32_t deblock_cmd_offset;
	struct ttm_object_file *tfile;
};

#ifdef CONFIG_VIDEO_MRFLD
struct psb_msvdx_ec_ctx *psb_msvdx_find_ec_ctx(
			struct msvdx_private *msvdx_priv,
			struct ttm_object_file *tfile,
			void *cmd);
#endif

static inline void psb_msvdx_clearirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned long mtx_int = 0;

	PSB_DEBUG_IRQ("MSVDX: clear IRQ\n");

	/* Clear MTX interrupt */
	REGIO_WRITE_FIELD_LITE(mtx_int, MSVDX_INTERRUPT_STATUS, MTX_IRQ,
			       1);
	PSB_WMSVDX32(mtx_int, MSVDX_INTERRUPT_CLEAR_OFFSET);
}

/* following two functions also works for CLV and MFLD */
/* PSB_INT_ENABLE_R is set in psb_irq_(un)install_islands */
static inline void psb_msvdx_disableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*uint32_t ier = dev_priv->vdc_irq_mask & (~_PSB_IRQ_MSVDX_FLAG); */

	unsigned long enables = 0;

	PSB_DEBUG_IRQ("MSVDX: enable MSVDX MTX IRQ\n");
	REGIO_WRITE_FIELD_LITE(enables, MSVDX_INTERRUPT_STATUS, MTX_IRQ,
			       0);
	PSB_WMSVDX32(enables, MSVDX_HOST_INTERRUPT_ENABLE_OFFSET);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}

static inline void psb_msvdx_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/* uint32_t ier = dev_priv->vdc_irq_mask | _PSB_IRQ_MSVDX_FLAG; */
	unsigned long enables = 0;

	PSB_DEBUG_IRQ("MSVDX: enable MSVDX MTX IRQ\n");
	/* Only enable the master core IRQ*/
	REGIO_WRITE_FIELD_LITE(enables, MSVDX_INTERRUPT_STATUS, MTX_IRQ,
			       1);
	PSB_WMSVDX32(enables, MSVDX_HOST_INTERRUPT_ENABLE_OFFSET);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}

static inline void psb_msvdx_mtx_set_clocks(struct drm_device *dev, uint32_t clock_state)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	uint32_t old_clock_state = 0;
	/* PSB_DEBUG_MSVDX("SetClocks to %x.\n", clock_state); */
	old_clock_state = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE_OFFSET);
	if (old_clock_state == clock_state)
		return;

	if (clock_state == 0) {
		/* Turn off clocks procedure */
		if (old_clock_state) {
			/* Turn off all the clocks except core */
			PSB_WMSVDX32(
				MSVDX_MAN_CLK_ENABLE__CORE_MAN_CLK_ENABLE_MASK,
				MSVDX_MAN_CLK_ENABLE_OFFSET);

			/* Make sure all the clocks are off except core */
			psb_wait_for_register(dev_priv,
				MSVDX_MAN_CLK_ENABLE_OFFSET,
				MSVDX_MAN_CLK_ENABLE__CORE_MAN_CLK_ENABLE_MASK,
				0xffffffff, 2000000, 5);

			/* Turn off core clock */
			PSB_WMSVDX32(0, MSVDX_MAN_CLK_ENABLE_OFFSET);
		}
	} else {
		uint32_t clocks_en = clock_state;

		/*Make sure that core clock is not accidentally turned off */
		clocks_en |= MSVDX_MAN_CLK_ENABLE__CORE_MAN_CLK_ENABLE_MASK;

		/* If all clocks were disable do the bring up procedure */
		if (old_clock_state == 0) {
			/* turn on core clock */
			PSB_WMSVDX32(
				MSVDX_MAN_CLK_ENABLE__CORE_MAN_CLK_ENABLE_MASK,
				MSVDX_MAN_CLK_ENABLE_OFFSET);

			/* Make sure core clock is on */
			psb_wait_for_register(dev_priv,
				MSVDX_MAN_CLK_ENABLE_OFFSET,
				MSVDX_MAN_CLK_ENABLE__CORE_MAN_CLK_ENABLE_MASK,
				0xffffffff, 2000000, 5);

			/* turn on the other clocks as well */
			PSB_WMSVDX32(clocks_en, MSVDX_MAN_CLK_ENABLE_OFFSET);

			/* Make sure that all they are on */
			psb_wait_for_register(dev_priv,
					MSVDX_MAN_CLK_ENABLE_OFFSET,
					clocks_en, 0xffffffff, 2000000, 5);
		} else {
			PSB_WMSVDX32(clocks_en, MSVDX_MAN_CLK_ENABLE_OFFSET);

			/* Make sure that they are on */
			psb_wait_for_register(dev_priv,
					MSVDX_MAN_CLK_ENABLE_OFFSET,
					clocks_en, 0xffffffff, 2000000, 5);
		}
	}
}

#define MSVDX_NEW_PMSTATE(drm_dev, msvdx_priv, new_state)		\
do {									\
	msvdx_priv->pmstate = new_state;				\
	if (new_state == PSB_PMSTATE_POWERDOWN)				\
		msvdx_priv->pm_gating_count++;				\
	sysfs_notify_dirent(msvdx_priv->sysfs_pmstate);			\
	PSB_DEBUG_PM("MSVDX: %s, power gating count 0x%08x\n",		\
		(new_state == PSB_PMSTATE_POWERUP) ? "powerup"		\
		: ((new_state == PSB_PMSTATE_POWERDOWN) ? "powerdown"	\
			: "clockgated"), msvdx_priv->pm_gating_count);	\
} while (0)

#if 0
#define PSB_WATCHDOG_DELAY (DRM_HZ * 2)
extern void psb_schedule_watchdog(struct drm_psb_private *dev_priv);
extern void psb_watchdog_init(struct drm_psb_private *dev_priv);
extern void psb_watchdog_takedown(struct drm_psb_private *dev_priv);
#endif

extern int psb_submit_video_cmdbuf(struct drm_device *dev,
				   struct ttm_buffer_object *cmd_buffer,
				   unsigned long cmd_offset,
				   unsigned long cmd_size,
				   struct ttm_fence_object *fence);

#endif
