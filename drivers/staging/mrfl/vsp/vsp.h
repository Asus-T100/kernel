/**
 * file vsp.h
 * Author: Binglin Chen <binglin.chen@intel.com>
 *
 */

/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
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

#ifndef _VSP_H_
#define _VSP_H_

#include "psb_drv.h"
#include "vsp_fw.h"

/* reg define */
#define SP1_SP_DMEM_IP 0x70000

#define SP0_SP_REG_BASE 0x0
#define SP1_SP_REG_BASE 0x50000

#define SP_STAT_AND_CTRL_REG 0x0
#define SP_STAT_AND_CTRL_REG_RESET_FLAG 0
#define SP_STAT_AND_CTRL_REG_START_FLAG 1
#define SP_STAT_AND_CTRL_REG_RUN_FLAG 3
#define SP_STAT_AND_CTRL_REG_READY_FLAG 5
#define SP_STAT_AND_CTRL_REG_SLEEP_FLAG 6
#define SP_STAT_AND_CTRL_REG_ICACHE_INVALID_FLAG 0xc
#define SP_STAT_AND_CTRL_REG_ICACHE_PREFETCH_FLAG 0xd

#define SP_BASE_ADDR_REG (0x1 * 4)

#define SP_CFG_PMEM_MASTER 0x10

#define MMU_INVALID 0x1B0000
#define MMU_TABLE_ADDR 0x1B0004

#define VSP_IRQ_REG_BASE 0x190000

#define VSP_IRQ_CTRL_IRQ_EDGE 0x0
#define VSP_IRQ_CTRL_IRQ_MASK 0x4
#define VSP_IRQ_CTRL_IRQ_STATUS 0x8
#define VSP_IRQ_CTRL_IRQ_CLR 0xC
#define VSP_IRQ_CTRL_IRQ_ENB 0x10
#define VSP_SP1_IRQ_SHIFT 0x8
#define IRQ_CTRL_IRQ_LEVEL_PLUS 0x14

#define VSP_CONFIG_REG_SDRAM_BASE 0x1A0000
#define VSP_CONFIG_REG_START 0x8

/* help macro */
#define MM_WRITE32(base, offset, value)					\
	do {								\
		*((unsigned long *)((unsigned char *)(dev_priv->vsp_reg) \
				    + base + offset)) = value;		\
	} while (0)

#define MM_READ32(base, offset, pointer)				\
	do {								\
		*(pointer) =						\
			*((unsigned long *)((unsigned char *)		\
					    (dev_priv->vsp_reg)		\
						 + base + offset));	\
	} while (0)

#define SP1_DMEM_WRITE32(offset, value)		\
	MM_WRITE32(SP1_SP_DMEM_IP, offset, value)
#define SP1_DMEM_READ32(offset, pointer)	\
	MM_READ32(SP1_SP_DMEM_IP, offset, pointer)

#define SP_REG_WRITE32(offset, value, processor)			\
	do {								\
		if ((processor) == vsp_sp0)				\
			MM_WRITE32(SP0_SP_REG_BASE, offset, value);	\
		else							\
			MM_WRITE32(SP1_SP_REG_BASE, offset, value);	\
	} while (0)

#define SP_REG_READ32(offset, pointer, processor)		\
	do {							\
		if ((processor) == vsp_sp0)				\
			MM_READ32(SP0_SP_REG_BASE, offset, pointer);	\
		else							\
			MM_READ32(SP1_SP_REG_BASE, offset, pointer);	\
	} while (0)


#define SP0_REG_WRITE32(offset, value)		\
	MM_WRITE32(SP0_SP_REG_BASE, offset, value)
#define SP0_REG_READ32(offset, pointer)		\
	MM_READ32(SP0_SP_REG_BASE, offset, pointer)

#define SP1_REG_WRITE32(offset, value)		\
	MM_WRITE32(SP1_SP_REG_BASE, offset, value)
#define SP1_REG_READ32(offset, pointer)		\
	MM_READ32(SP1_SP_REG_BASE, offset, pointer)

#define CONFIG_REG_WRITE32(offset, value)			\
	MM_WRITE32(VSP_CONFIG_REG_SDRAM_BASE, ((offset) * 4), value)
#define CONFIG_REG_READ32(offset, pointer)			\
	MM_READ32(VSP_CONFIG_REG_SDRAM_BASE, ((offset) * 4), pointer)

#define PAGE_TABLE_SHIFT PAGE_SHIFT
#define INVALID_MMU MM_WRITE32(0, MMU_INVALID, 0x1)
#define SET_MMU_PTD(address)						\
	do {								\
		INVALID_MMU;						\
		MM_WRITE32(0, MMU_TABLE_ADDR, address);			\
	} while (0)

#define VSP_SET_FLAG(val, offset) \
	(val) = ((val) | (0x1 << (offset)))
#define VSP_CLEAR_FLAG(val, offset) \
	(val) = ((val) & (~(0x1 << (offset))))
#define VSP_READ_FLAG(val, offset) \
	(((val) & (0x1 << (offset))) >> (offset))
#define VSP_REVERT_FLAG(val, offset) \
	((val) = (val ^ (0x1 << (offset))))

#define IRQ_REG_WRITE32(offset, value)		\
	MM_WRITE32(VSP_IRQ_REG_BASE, offset, value)
#define IRQ_REG_READ32(offset, pointer)		\
	MM_READ32(VSP_IRQ_REG_BASE, offset, pointer)

#define VSP_NEW_PMSTATE(drm_dev, vsp_priv, new_state)			\
do {									\
	vsp_priv->pmstate = new_state;					\
	sysfs_notify_dirent(vsp_priv->sysfs_pmstate);			\
	PSB_DEBUG_PM("VSP: %s\n",					\
		(new_state == PSB_PMSTATE_POWERUP) ? "powerup"		\
		: ((new_state == PSB_PMSTATE_POWERDOWN) ? "powerdown"	\
		: "clockgated"));					\
} while (0)

struct vsp_private {
	uint32_t current_sequence;

	int fw_loaded;
	int needs_reset;

	spinlock_t lock;

	unsigned int cmd_queue_size;
	unsigned int ack_queue_size;

	struct ttm_buffer_object *firmware;
	unsigned int firmware_sz;

	struct ttm_buffer_object *cmd_queue_bo;
	unsigned int cmd_queue_sz;
	struct ttm_bo_kmap_obj cmd_kmap;
	struct vss_command_t *cmd_queue;

	struct ttm_buffer_object *ack_queue_bo;
	unsigned int ack_queue_sz;
	struct ttm_bo_kmap_obj ack_kmap;
	struct vss_response_t *ack_queue;

	struct vsp_config config;

	struct vsp_ctrl_reg *ctrl;


	unsigned int pmstate;
	struct sysfs_dirent *sysfs_pmstate;
	unsigned int vsp_busy;

	uint64_t vss_cc_acc;
};

extern int vsp_init(struct drm_device *dev);
extern int vsp_deinit(struct drm_device *dev);

extern int vsp_reset(struct drm_psb_private *dev_priv);

extern int vsp_init_fw(struct drm_device *dev);
extern int vsp_setup_fw(struct drm_psb_private *dev_priv);

extern void vsp_enableirq(struct drm_device *dev);
extern void vsp_disableirq(struct drm_device *dev);

extern bool vsp_interrupt(void *pvData);

extern int vsp_cmdbuf_vpp(struct drm_file *priv,
			  struct list_head *validate_list,
			  uint32_t fence_type,
			  struct drm_psb_cmdbuf_arg *arg,
			  struct ttm_buffer_object *cmd_buffer,
			  struct psb_ttm_fence_rep *fence_arg);

extern uint32_t vsp_fence_poll(struct drm_psb_private *dev_priv);

extern void vsp_new_context(struct drm_device *dev);
extern void vsp_rm_context(struct drm_device *dev);
extern uint32_t psb_get_default_pd_addr(struct psb_mmu_driver *driver);

extern int psb_vsp_save_context(struct drm_device *dev);
extern int psb_vsp_restore_context(struct drm_device *dev);
extern int psb_check_vsp_idle(struct drm_device *dev);

static inline
unsigned int vsp_is_idle(struct drm_psb_private *dev_priv,
			 unsigned int processor)
{
	unsigned int reg, start_bit, idle_bit;

	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &reg, processor);
	start_bit = VSP_READ_FLAG(reg, SP_STAT_AND_CTRL_REG_START_FLAG);
	idle_bit = VSP_READ_FLAG(reg, SP_STAT_AND_CTRL_REG_READY_FLAG);

	return !start_bit && idle_bit;
}

static inline
unsigned int vsp_is_sleeping(struct drm_psb_private *dev_priv,
			     unsigned int processor)
{
	unsigned int reg;

	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &reg, processor);
	return VSP_READ_FLAG(reg, SP_STAT_AND_CTRL_REG_SLEEP_FLAG);
}

static inline
void vsp_config_icache(struct drm_psb_private *dev_priv,
		       unsigned int processor)
{
	unsigned int reg;

	reg = 0;
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_INVALID_FLAG);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_PREFETCH_FLAG);
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);

	return;
}
#endif	/* _VSP_H_ */
