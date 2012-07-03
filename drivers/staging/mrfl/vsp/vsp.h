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

/* reg define */
#define SP1_SP_DMEM_IP 0x70000
#if 1				/* 0308 */
#define SP1_PPC_VP0_ADDR 0xE08
#define SP1_PPC_VP1_ADDR 0xE0C
#define SP1_VPP_CHAIN_VP0_ADDR 0xE10
#define SP1_VPP_CHAIN_VP1_ADDR 0xE14
#define SP1_VPP_CHAIN_SP1_ADDR 0xE18
/* vpp chain sp0 0xe14 */
#define SP1_MEMC_APP_ADDR 0xE1C
#define SP1_CONTEXT_SIZE_ADDR 0x570
#define SP1_CONTEXT_BASE_ADDR 0x574
#define SP1_VSS_NUM_QUEUE_ADDR 0x56C
#define SP1_VSS_CMD_QUEUE_ADDR 0x12C
#define SP1_VSS_CMD_BUFFER_ADDR 0x14C
#define SP1_VSS_ACK_QUEUE_ADDR 0x3CC
#define SP1_VSS_ACK_BUFFER_ADDR 0x3EC

/* start addr */
#define SP_MAIN_ENTRY 0x2B55
/*
#define SP_MAIN_ENTRY 0x2AD3
*/
#endif
#if 0				/* 0203 */
#define SP1_PPC_VP0_ADDR 0xE04
#define SP1_PPC_VP1_ADDR 0xE08
#define SP1_VPP_CHAIN_VP0_ADDR 0xE0C
#define SP1_VPP_CHAIN_VP1_ADDR 0xE10
#define SP1_VPP_CHAIN_SP1_ADDR 0xE14
/* vpp chain sp0 0xe14 */
#define SP1_MEMC_APP_ADDR 0xE18
#define SP1_CONTEXT_SIZE_ADDR 0x570
#define SP1_CONTEXT_BASE_ADDR 0x574
#define SP1_VSS_NUM_QUEUE_ADDR 0x56C
#define SP1_VSS_CMD_QUEUE_ADDR 0x12C
#define SP1_VSS_CMD_BUFFER_ADDR 0x14C
#define SP1_VSS_ACK_QUEUE_ADDR 0x3CC
#define SP1_VSS_ACK_BUFFER_ADDR 0x3EC

/* start addr */
#define SP_MAIN_ENTRY 0x2AAF
#endif
#if 0				/* 1222 */
#define SP1_PPC_VP0_ADDR 0xD38
#define SP1_PPC_VP1_ADDR 0xD3C
#define SP1_VPP_CHAIN_VP0_ADDR 0xD40
#define SP1_VPP_CHAIN_VP1_ADDR 0xD44
#define SP1_MEMC_APP_ADDR 0xD48
#define SP1_CONTEXT_SIZE_ADDR 0x4D0
#define SP1_CONTEXT_BASE_ADDR 0x4D4
#define SP1_VSS_NUM_QUEUE_ADDR 0x4CC
#define SP1_VSS_CMD_QUEUE_ADDR 0x10C
#define SP1_VSS_CMD_BUFFER_ADDR 0x12C
#define SP1_VSS_ACK_QUEUE_ADDR 0x32C
#define SP1_VSS_ACK_BUFFER_ADDR 0x34C

/* start addr */
#define SP_MAIN_ENTRY 0x1FF0
#endif
#if 0				/* private */
#define SP1_PPC_VP0_ADDR 0xD38
#define SP1_PPC_VP1_ADDR 0xD3C
#define SP1_VPP_CHAIN_VP0_ADDR 0xD40
#define SP1_VPP_CHAIN_VP1_ADDR 0xD44
#define SP1_MEMC_APP_ADDR 0xD48
/* ..... */
#define SP1_CONTEXT_SIZE_ADDR 0x490
#define SP1_CONTEXT_BASE_ADDR 0x494
#define SP1_VSS_NUM_QUEUE_ADDR 0x48C
#define SP1_VSS_CMD_QUEUE_ADDR 0xCC
#define SP1_VSS_CMD_BUFFER_ADDR 0xEC
#define SP1_VSS_ACK_QUEUE_ADDR 0x2EC
#define SP1_VSS_ACK_BUFFER_ADDR 0x30C

/* start addr */
#define SP_MAIN_ENTRY 0x62E

#endif


#define SP0_SP_REG_BASE 0x0

#define SP0_CFG_PMEM_MASTER 0x10
#define SP0_XMEM_MASTER 0x14


#define SP1_SP_REG_BASE 0x50000

#define SP_STAT_AND_CTRL_REG 0x0
#define SP_STAT_AND_CTRL_REG_RESET_FLAG 0
#define SP_STAT_AND_CTRL_REG_START_FLAG 1
#define SP_STAT_AND_CTRL_REG_RUN_FLAG 3

#define SP_LOOP_CACHE_INVALIDATE_REG (0xB * 4)
#define SP_LOOP_CACHE_INVALIDATE_REG_INVALIDATE_FLAG 0

#define SP1_CFG_PMEM_MASTER 0x10
#define SP1_XMEM_MASTER 0x14


#define VP0_VP_REG_BASE 0x80000

#define VP0_CFG_PMEM_MASTER 0x10
#define VP0_XD2MEM_MASTER 0x14
#define VP0_XD2MEMW_MASTER 0x18
#define VP0_XMEM_MASTER 0x1c


#define VP1_VP_REG_BASE 0xc0000

#define VP1_CFG_PMEM_MASTER 0x10
#define VP1_XD2MEM_MASTER 0x14
#define VP1_XD2MEMW_MASTER 0x18
#define VP1_XMEM_MASTER 0x1c


#define MEA_REG_BASE 0x100000

#define MEA_CFG_PMEM_MASTER 0x10
#define MEA_XMEM_MASTER 0x14
#define MEA_XD2MEM_MASTER 0x18

#define SP_BASE_ADDR_REG (0x1 * 4)

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

#define SP0_REG_WRITE32(offset, value)		\
	MM_WRITE32(SP0_SP_REG_BASE, offset, value)
#define SP0_REG_READ32(offset, pointer)		\
	MM_READ32(SP0_SP_REG_BASE, offset, pointer)

#define SP1_REG_WRITE32(offset, value)		\
	MM_WRITE32(SP1_SP_REG_BASE, offset, value)
#define SP1_REG_READ32(offset, pointer)		\
	MM_READ32(SP1_SP_REG_BASE, offset, pointer)

#define VP0_REG_WRITE32(offset, value)		\
	MM_WRITE32(VP0_VP_REG_BASE, offset, value)
#define VP0_REG_READ32(offset, pointer)		\
	MM_READ32(VP0_VP_REG_BASE, offset, pointer)

#define VP1_REG_WRITE32(offset, value)		\
	MM_WRITE32(VP1_VP_REG_BASE, offset, value)
#define VP1_REG_READ32(offset, pointer)		\
	MM_READ32(VP1_VP_REG_BASE, offset, pointer)

#define MEA_REG_WRITE32(offset, value)		\
	MM_WRITE32(MEA_REG_BASE, offset, value)
#define MEA_REG_READ32(offset, pointer)		\
	MM_READ32(MEA_REG_BASE, offset, pointer)

#define PAGE_TABLE_SHIFT PAGE_SHIFT
#define INVALID_MMU MM_WRITE32(0, MMU_INVALID, 0x1)
#define SET_MMU_PTD(address)						\
	do {								\
		INVALID_MMU;						\
		MM_WRITE32(0, MMU_TABLE_ADDR, address);			\
	} while (0)

#define VSP_SET_FLAG(val, offset) \
	(val) = (val | (0x1 << (offset)))
#define VSP_CLEAR_FLAG(val, offset) \
	(val) = (val & (~(0x1 << (offset))))

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

	unsigned int context_size;
	struct ttm_buffer_object *context_buf;

	unsigned int cmd_queue_size;
	unsigned int ack_queue_size;

	struct ttm_buffer_object *ppc_vp0_fw;
	unsigned int ppc_vp0_fw_sz;

	struct ttm_buffer_object *ppc_vp1_fw;
	unsigned int ppc_vp1_fw_sz;

	struct ttm_buffer_object *vpp_chain_vp0_fw;
	unsigned int vpp_chain_vp0_fw_sz;

	struct ttm_buffer_object *vpp_chain_vp1_fw;
	unsigned int vpp_chain_vp1_fw_sz;

	struct ttm_buffer_object *vpp_chain_sp1_fw;
	unsigned int vpp_chain_sp1_fw_sz;

	struct ttm_buffer_object *memc_app_fw;
	unsigned int memc_app_fw_sz;

	struct ttm_buffer_object *proc_fw;
	unsigned int proc_fw_sz;

	unsigned int pmstate;
	struct sysfs_dirent *sysfs_pmstate;
	unsigned int vsp_busy;
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

extern void vsp_reset_fw_status(struct drm_device *dev);
extern uint32_t psb_get_default_pd_addr(struct psb_mmu_driver *driver);

extern int psb_vsp_save_context(struct drm_device *dev);
extern int psb_vsp_restore_context(struct drm_device *dev);
extern int psb_check_vsp_idle(struct drm_device *dev);

#endif	/* _VSP_H_ */
