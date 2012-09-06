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
 **************************************************************************/

#ifndef _LNC_TOPAZ_H_
#define _LNC_TOPAZ_H_

#include "psb_drv.h"

#define LNC_TOPAZ_NO_IRQ 0
#define TOPAZ_MTX_REG_SIZE (34 * 4 + 183 * 4)
/*Must be euqal to  IMG_CODEC_NUM */
#define LNC_IMG_CODEC_NUM_MAX (11)

extern int drm_topaz_pmpolicy;

/* XXX: it's a copy of msvdx cmd queue. should have some change? */
struct lnc_topaz_cmd_queue {
	struct list_head head;
	void *cmd;
	unsigned long cmd_size;
	uint32_t sequence;
};

/* define structure */
/* firmware file's info head */
struct topaz_fwinfo {
	unsigned int ver:16;
	unsigned int codec:16;

	unsigned int text_size;
	unsigned int data_size;
	unsigned int data_location;
};

/* firmware data array define  */
struct topaz_codec_fw {
	uint32_t ver;
	uint32_t codec;

	uint32_t text_size;
	uint32_t data_size;
	uint32_t data_location;

	struct ttm_buffer_object *text;
	struct ttm_buffer_object *data;
};

struct topaz_private {
	unsigned int pmstate;
	struct sysfs_dirent *sysfs_pmstate;
	int frame_skip;

	void *topaz_mtx_reg_state;
	struct ttm_buffer_object *topaz_mtx_data_mem;
	uint32_t topaz_cur_codec;
	uint32_t cur_mtx_data_size;
	int topaz_needs_reset;

	/*
	 *topaz command queue
	 */
	spinlock_t topaz_lock;
	struct mutex topaz_mutex;
	struct list_head topaz_queue;
	int topaz_busy;		/* 0 means topaz is free */
	int topaz_fw_loaded;

	/* topaz ccb data */
	/* XXX: should the addr stored by 32 bits? more compatible way?? */
	uint32_t topaz_ccb_buffer_addr;
	uint32_t topaz_ccb_ctrl_addr;
	uint32_t topaz_ccb_size;
	uint32_t topaz_cmd_windex;
	uint16_t topaz_cmd_seq;

	uint32_t stored_initial_qp;
	uint32_t topaz_dash_access_ctrl;

	/* 4K->2K/2K for writeback/sync */
	struct ttm_buffer_object *topaz_bo;
	struct ttm_bo_kmap_obj topaz_bo_kmap;
	void *topaz_ccb_wb;
	uint32_t topaz_wb_offset;
	uint32_t *topaz_sync_addr;
	uint32_t topaz_sync_offset;
	uint32_t topaz_sync_cmd_seq;
	uint32_t topaz_mtx_saved;

	/* firmware */
	struct topaz_codec_fw topaz_fw[LNC_IMG_CODEC_NUM_MAX];

	uint32_t topaz_hw_busy;
};

/* external function declare */
/*lnc_topaz.c*/
extern int lnc_cmdbuf_video(struct drm_file *priv,
			    struct list_head *validate_list,
			    uint32_t fence_type,
			    struct drm_psb_cmdbuf_arg *arg,
			    struct ttm_buffer_object *cmd_buffer,
			    struct psb_ttm_fence_rep *fence_arg);

extern bool lnc_topaz_interrupt(void *pvData);

/* lnc_topazinit.c*/
extern int lnc_wait_topaz_idle(struct drm_device *dev);
extern int lnc_check_topaz_idle(struct drm_device *dev);
extern void lnc_unmap_topaz_reg(struct drm_device *dev);
extern void lnc_map_topaz_reg(struct drm_device *dev);

extern int lnc_topaz_restore_mtx_state(struct drm_device *dev);

extern int lnc_topaz_init(struct drm_device *dev);
extern int lnc_topaz_uninit(struct drm_device *dev);

extern void lnc_topaz_handle_timeout(struct ttm_fence_device *fdev);

extern void lnc_topaz_enableirq(struct drm_device *dev);
extern void lnc_topaz_disableirq(struct drm_device *dev);

extern int lnc_topaz_save_mtx_state(struct drm_device *dev);

#define TOPAZ_NEW_PMSTATE(drm_dev, topaz_priv, new_state)		\
do { \
	topaz_priv->pmstate = new_state;				\
	sysfs_notify_dirent(topaz_priv->sysfs_pmstate);			\
	PSB_DEBUG_PM("TOPAZ: %s\n",					\
		(new_state == PSB_PMSTATE_POWERUP) ? "powerup"		\
		: ((new_state == PSB_PMSTATE_POWERDOWN) ? "powerdown"	\
			: "clockgated"));				\
} while (0)

#endif				/* _LNC_TOPAZ_H_ */
