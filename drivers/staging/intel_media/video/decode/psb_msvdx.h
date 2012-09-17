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
 *
 **************************************************************************/

#ifndef _PSB_MSVDX_H_
#define _PSB_MSVDX_H_

#include "psb_drv.h"
#include "img_types.h"

extern int drm_msvdx_pmpolicy;
extern int drm_msvdx_delay;
extern int hdmi_state;
extern int drm_psb_msvdx_tiling;

#define FIRMWAREID		0x014d42ab

typedef enum {
	PSB_DMAC_BSWAP_NO_SWAP = 0x0,   /* !< No byte swapping will be performed. */
	PSB_DMAC_BSWAP_REVERSE = 0x1,   /* !< Byte order will be reversed. */

} DMAC_eBSwap;

typedef enum {
	PSB_DMAC_DIR_MEM_TO_PERIPH = 0x0, /* !< Data from memory to peripheral. */
	PSB_DMAC_DIR_PERIPH_TO_MEM = 0x1, /* !< Data from peripheral to memory. */

} DMAC_eDir;

typedef enum {
	PSB_DMAC_ACC_DEL_0              = 0x0,          /* !< Access delay zero clock cycles */
	PSB_DMAC_ACC_DEL_256    = 0x1,      /* !< Access delay 256 clock cycles */
	PSB_DMAC_ACC_DEL_512    = 0x2,      /* !< Access delay 512 clock cycles */
	PSB_DMAC_ACC_DEL_768    = 0x3,      /* !< Access delay 768 clock cycles */
	PSB_DMAC_ACC_DEL_1024   = 0x4,      /* !< Access delay 1024 clock cycles */
	PSB_DMAC_ACC_DEL_1280   = 0x5,      /* !< Access delay 1280 clock cycles */
	PSB_DMAC_ACC_DEL_1536   = 0x6,      /* !< Access delay 1536 clock cycles */
	PSB_DMAC_ACC_DEL_1792   = 0x7,      /* !< Access delay 1792 clock cycles */

} DMAC_eAccDel;

typedef enum {
	PSB_DMAC_INCR_OFF               = 0,            /* !< Static peripheral address. */
	PSB_DMAC_INCR_ON                = 1                     /* !< Incrementing peripheral address. */

} DMAC_eIncr;

typedef enum {
	PSB_DMAC_BURST_0                = 0x0,          /* !< burst size of 0 */
	PSB_DMAC_BURST_1        = 0x1,      /* !< burst size of 1 */
	PSB_DMAC_BURST_2        = 0x2,      /* !< burst size of 2 */
	PSB_DMAC_BURST_3        = 0x3,      /* !< burst size of 3 */
	PSB_DMAC_BURST_4        = 0x4,      /* !< burst size of 4 */
	PSB_DMAC_BURST_5        = 0x5,      /* !< burst size of 5 */
	PSB_DMAC_BURST_6        = 0x6,      /* !< burst size of 6 */
	PSB_DMAC_BURST_7        = 0x7,      /* !< burst size of 7 */
} DMAC_eBurst;

/* psb_mmu.c */
uint32_t psb_get_default_pd_addr(struct psb_mmu_driver *driver);

/* psb_reset.c */
void psb_msvdx_flush_cmd_queue(struct drm_device *dev);

/* psb_msvdxinit.c */
int psb_wait_for_register(struct drm_psb_private *dev_priv,
			  uint32_t offset,
			  uint32_t value,
			  uint32_t enable);
int psb_msvdx_init(struct drm_device *dev);
int psb_msvdx_uninit(struct drm_device *dev);
int psb_msvdx_reset(struct drm_psb_private *dev_priv);
void psb_write_mtx_core_reg(struct drm_psb_private *dev_priv,
			    const uint32_t core_reg, const uint32_t val);

/* psb_msvdx.c */
IMG_BOOL psb_msvdx_interrupt(IMG_VOID *pvData);
int psb_mtx_send(struct drm_psb_private *dev_priv, const void *pvMsg);

void psb_msvdx_lockup(struct drm_psb_private *dev_priv,
		      int *msvdx_lockup, int *msvdx_idle);
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
int psb_msvdx_check_reset_fw(struct drm_device *dev);
void psb_powerdown_msvdx(struct work_struct *work);

/* psb_msvdx_fw.c */
int32_t psb_msvdx_alloc_fw_bo(struct drm_psb_private *dev_priv);
int psb_setup_fw(struct drm_device *dev);
int psb_setup_msvdx(struct drm_device *dev);

/*  Non-Optimal Invalidation is not default */
#define MSVDX_DEVICE_NODE_FLAGS_MMU_NONOPT_INV	2
#define MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK	(0x00000100)

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

#define MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK		(0x00000100)
#define MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_SHIFT		(8)
#define MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_FE_SOFT_RESET_MASK	\
	(0x00010000)
#define MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_BE_SOFT_RESET_MASK	\
	(0x00100000)
#define MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_VEC_MEMIF_SOFT_RESET_MASK	\
	(0x01000000)
#define MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_VEC_RENDEC_DEC_SOFT_RESET_MASK \
	(0x10000000)

#define clk_enable_auto	\
(MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_PROCESS_AUTO_CLK_ENABLE_MASK	| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_ACCESS_AUTO_CLK_ENABLE_MASK	| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDMC_AUTO_CLK_ENABLE_MASK		| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ENTDEC_AUTO_CLK_ENABLE_MASK	| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ITRANS_AUTO_CLK_ENABLE_MASK	| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK		| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_MTX_MAN_CLK_ENABLE_MASK)

#define msvdx_sw_reset_all \
(MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK |	  \
MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_FE_SOFT_RESET_MASK |	  \
MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_BE_SOFT_RESET_MASK	|		\
MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_VEC_MEMIF_SOFT_RESET_MASK |	\
MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_VEC_RENDEC_DEC_SOFT_RESET_MASK)

#define MTX_INTERNAL_REG(R_SPECIFIER , U_SPECIFIER)	\
	(((R_SPECIFIER)<<4) | (U_SPECIFIER))
#define MTX_PC		MTX_INTERNAL_REG(0, 5)

#define RENDEC_A_SIZE	(4 * 1024 * 1024)
#define RENDEC_B_SIZE	(1024 * 1024)

#define MEMIO_READ_FIELD(vpMem, field)	  \
	((uint32_t)(((*((field##_TYPE*)(((uint32_t)vpMem) + field##_OFFSET))) \
			& field##_MASK) >> field##_SHIFT))

#define MEMIO_WRITE_FIELD(vpMem, field, value)				\
	((*((field##_TYPE*)(((uint32_t)vpMem) + field##_OFFSET))) =	\
		((*((field##_TYPE*)(((uint32_t)vpMem) + field##_OFFSET))) \
			& (field##_TYPE)~field##_MASK) |		\
	(field##_TYPE)(((uint32_t)(value) << field##_SHIFT) & field##_MASK))

#define MEMIO_WRITE_FIELD_LITE(vpMem, field, value)			\
	 (*((field##_TYPE*)(((uint32_t)vpMem) + field##_OFFSET))) =	\
	((*((field##_TYPE*)(((uint32_t)vpMem) + field##_OFFSET))) |	\
		(field##_TYPE)(((uint32_t)(value) << field##_SHIFT)));

#define REGIO_READ_FIELD(reg_val, reg, field)				\
	((reg_val & reg##_##field##_MASK) >> reg##_##field##_SHIFT)

#define REGIO_WRITE_FIELD(reg_val, reg, field, value)			\
	(reg_val) =							\
	((reg_val) & ~(reg##_##field##_MASK)) |				\
		(((value) << (reg##_##field##_SHIFT)) & (reg##_##field##_MASK));

#define REGIO_WRITE_FIELD_LITE(reg_val, reg, field, value)		\
	(reg_val) =							\
	((reg_val) | ((value) << (reg##_##field##_SHIFT)));

#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK	\
	(0x00000001)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_PROCESS_MAN_CLK_ENABLE_MASK \
	(0x00000002)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_ACCESS_MAN_CLK_ENABLE_MASK \
	(0x00000004)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDMC_MAN_CLK_ENABLE_MASK \
			(0x00000008)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ENTDEC_MAN_CLK_ENABLE_MASK \
	(0x00000010)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ITRANS_MAN_CLK_ENABLE_MASK \
	(0x00000020)
#define MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_MTX_MAN_CLK_ENABLE_MASK	\
	(0x00000040)

#define clk_enable_all	\
	(MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK	| \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_PROCESS_MAN_CLK_ENABLE_MASK | \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDEB_ACCESS_MAN_CLK_ENABLE_MASK | \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VDMC_MAN_CLK_ENABLE_MASK	 | \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ENTDEC_MAN_CLK_ENABLE_MASK | \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_VEC_ITRANS_MAN_CLK_ENABLE_MASK | \
MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_MTX_MAN_CLK_ENABLE_MASK)

#define clk_enable_minimal \
	MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK | \
	MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_MTX_MAN_CLK_ENABLE_MASK

/* MTX registers */
#define MSVDX_MTX_ENABLE		(0x0000)
#define MSVDX_MTX_KICKI			(0x0088)
#define MSVDX_MTX_KICK			(0x0080)
#define MSVDX_MTX_REGISTER_READ_WRITE_REQUEST	(0x00FC)
#define MSVDX_MTX_REGISTER_READ_WRITE_DATA	(0x00F8)
#define MSVDX_MTX_RAM_ACCESS_DATA_TRANSFER	(0x0104)
#define MSVDX_MTX_RAM_ACCESS_CONTROL	(0x0108)
#define MSVDX_MTX_RAM_ACCESS_STATUS	(0x010C)
#define MSVDX_MTX_SOFT_RESET		(0x0200)
#define MSVDX_MTX_SYSC_TIMERDIV		(0x0208)
#define MTX_CORE_CR_MTX_SYSC_CDMAS0_OFFSET              (0x0348)
#define MTX_CORE_CR_MTX_SYSC_CDMAA_OFFSET               (0x0344)
#define MTX_CORE_CR_MTX_SYSC_CDMAT_OFFSET               (0x0350)
#define MTX_CORE_CR_MTX_SYSC_CDMAC_OFFSET               (0x0340)

/* MSVDX registers */
#define MSVDX_CONTROL			(0x0600)
#define MSVDX_INTERRUPT_CLEAR		(0x060C)
#define MSVDX_INTERRUPT_STATUS		(0x0608)
#define MSVDX_HOST_INTERRUPT_ENABLE	(0x0610)
#define MSVDX_CORE_REV			(0x0640)
#define MSVDX_MMU_CONTROL0		(0x0680)
#define MSVDX_MMU_STATUS		(0x068C)
#define MSVDX_MMU_MEM_REQ		(0x06D0)
#define MSVDX_MMU_TILE_BASE0		(0x06D4)
#define MSVDX_MMU_TILE_BASE1		(0x06D8)
#define MSVDX_MTX_RAM_BANK		(0x06F0)
#define MSVDX_MTX_DEBUG			MSVDX_MTX_RAM_BANK
#define MSVDX_MAN_CLK_ENABLE		(0x0620)
#define MSVDX_CORE_CR_MSVDX_CONTROL_OFFSET              (0x0600)
#define MSVDX_CORE_CR_MMU_BANK_INDEX_OFFSET             (0x0688)
#define MSVDX_CORE_CR_MMU_DIR_LIST_BASE_OFFSET          (0x0694)
#define MSVDX_CORE_CR_MMU_CONTROL0_OFFSET               MSVDX_MMU_CONTROL0
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH		(0x66c)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH		(0x678)

/* RENDEC registers */
#define MSVDX_RENDEC_CONTROL0		(0x0868)
#define MSVDX_RENDEC_CONTROL1		(0x086C)
#define MSVDX_RENDEC_BUFFER_SIZE	(0x0870)
#define MSVDX_RENDEC_BASE_ADDR0		(0x0874)
#define MSVDX_RENDEC_BASE_ADDR1		(0x0878)
#define MSVDX_RENDEC_READ_DATA		(0x0898)
#define MSVDX_RENDEC_CONTEXT0		(0x0950)
#define MSVDX_RENDEC_CONTEXT1		(0x0954)
#define MSVDX_RENDEC_CONTEXT2		(0x0958)
#define MSVDX_RENDEC_CONTEXT3		(0x095C)
#define MSVDX_RENDEC_CONTEXT4		(0x0960)
#define MSVDX_RENDEC_CONTEXT5		(0x0964)

/* VEC registers */
#define MSVDX_VEC_SHIFTREG_CONTROL	(0x0818)

/* DMAC registers */
#define DMAC_DMAC_SETUP_OFFSET          (0x0500)
#define DMAC_DMAC_COUNT_OFFSET          (0x0504)
#define DMAC_DMAC_PERIPH_OFFSET         (0x0508)
#define DMAC_DMAC_IRQ_STAT_OFFSET       (0x050C)
#define DMAC_DMAC_PERIPHERAL_ADDR_OFFSET                (0x0514)

/* DMAC control */
#define PSB_DMAC_VALUE_COUNT(BSWAP, PW, DIR, PERIPH_INCR, COUNT)                                \
	((((BSWAP)           & DMAC_DMAC_COUNT_BSWAP_LSBMASK)        << DMAC_DMAC_COUNT_BSWAP_SHIFT) | \
	(((PW)                  & DMAC_DMAC_COUNT_PW_LSBMASK)           << DMAC_DMAC_COUNT_PW_SHIFT)    | \
	(((DIR)                 & DMAC_DMAC_COUNT_DIR_LSBMASK)          << DMAC_DMAC_COUNT_DIR_SHIFT)   | \
	(((PERIPH_INCR) & DMAC_DMAC_COUNT_PI_LSBMASK)           << DMAC_DMAC_COUNT_PI_SHIFT)    | \
	(((COUNT)               & DMAC_DMAC_COUNT_CNT_LSBMASK)          << DMAC_DMAC_COUNT_CNT_SHIFT))

#define PSB_DMAC_VALUE_PERIPH_PARAM(ACC_DEL, INCR, BURST)							\
	((((ACC_DEL)     & DMAC_DMAC_PERIPH_ACC_DEL_LSBMASK)     << DMAC_DMAC_PERIPH_ACC_DEL_SHIFT)      | \
	(((INCR)        & DMAC_DMAC_PERIPH_INCR_LSBMASK)        << DMAC_DMAC_PERIPH_INCR_SHIFT)         | \
	(((BURST)       & DMAC_DMAC_PERIPH_BURST_LSBMASK)       << DMAC_DMAC_PERIPH_BURST_SHIFT))


/* CMD */
#define MSVDX_CMDS_END_SLICE_PICTURE	(0x1404)

/*
 * This defines the MSVDX communication buffer
 */
#define MSVDX_COMMS_SIGNATURE_VALUE	(0xA5A5A5A5)	/*!< Signature value */
/*!< Host buffer size (in 32-bit words) */
#define NUM_WORDS_HOST_BUF		(100)
/*!< MTX buffer size (in 32-bit words) */
#define NUM_WORDS_MTX_BUF		(100)

/* There is no work currently underway on the hardware */
#define MSVDX_FW_STATUS_HW_IDLE	0x00000001

#define MSVDX_EXT_FW_ERROR_STATE (0x2884)
#define MSVDX_COMMS_AREA_ADDR (0x02fe0)

#define MSVDX_COMMS_CORE_WTD			(MSVDX_COMMS_AREA_ADDR - 0x08)
#define MSVDX_COMMS_ERROR_TRIG			(MSVDX_COMMS_AREA_ADDR - 0x08)
#define MSVDX_COMMS_FIRMWARE_ID			(MSVDX_COMMS_AREA_ADDR - 0x0C)
#define MSVDX_COMMS_OFFSET_FLAGS		(MSVDX_COMMS_AREA_ADDR + 0x18)
#define	MSVDX_COMMS_MSG_COUNTER			(MSVDX_COMMS_AREA_ADDR - 0x04)
#define MSVDX_COMMS_FW_STATUS			(MSVDX_COMMS_AREA_ADDR - 0x10)
#define	MSVDX_COMMS_SIGNATURE			(MSVDX_COMMS_AREA_ADDR + 0x00)
#define	MSVDX_COMMS_TO_HOST_BUF_SIZE		(MSVDX_COMMS_AREA_ADDR + 0x04)
#define MSVDX_COMMS_TO_HOST_RD_INDEX		(MSVDX_COMMS_AREA_ADDR + 0x08)
#define MSVDX_COMMS_TO_HOST_WRT_INDEX		(MSVDX_COMMS_AREA_ADDR + 0x0C)
#define MSVDX_COMMS_TO_MTX_BUF_SIZE		(MSVDX_COMMS_AREA_ADDR + 0x10)
#define MSVDX_COMMS_TO_MTX_RD_INDEX		(MSVDX_COMMS_AREA_ADDR + 0x14)
#define MSVDX_COMMS_TO_MTX_CB_RD_INDEX		(MSVDX_COMMS_AREA_ADDR + 0x18)
#define MSVDX_COMMS_TO_MTX_WRT_INDEX		(MSVDX_COMMS_AREA_ADDR + 0x1C)
#define MSVDX_COMMS_TO_HOST_BUF			(MSVDX_COMMS_AREA_ADDR + 0x20)
#define MSVDX_COMMS_TO_MTX_BUF	\
	(MSVDX_COMMS_TO_HOST_BUF + (NUM_WORDS_HOST_BUF << 2))

#define DSIABLE_FW_WDT			        0x0008
#define ABORT_ON_ERRORS_IMMEDIATE       0x0010
#define ABORT_FAULTED_SLICE_IMMEDIATE   0x0020
#define RETURN_VDEB_DATA_IN_COMPLETION  0x0800
#define DSIABLE_Auto_CLOCK_GATING	    0x1000
#define DSIABLE_IDLE_GPIO_SIG			0x2000
/*
#define MSVDX_COMMS_AREA_END	\
  (MSVDX_COMMS_TO_MTX_BUF + (NUM_WORDS_HOST_BUF << 2))
*/
#define MSVDX_COMMS_AREA_END 0x03000

#if (MSVDX_COMMS_AREA_END != 0x03000)
#error
#endif

#define MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK	(0x80000000)
#define MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_SHIFT	(31)

#define MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_RNW_MASK	(0x00010000)
#define MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_RNW_SHIFT	(16)

#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMID_MASK		(0x0FF00000)
#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMID_SHIFT		(20)

#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCM_ADDR_MASK	(0x000FFFFC)
#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCM_ADDR_SHIFT	(2)

#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMAI_MASK	(0x00000002)
#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMAI_SHIFT	(1)

#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMR_MASK	(0x00000001)
#define MSVDX_MTX_RAM_ACCESS_CONTROL_MTX_MCMR_SHIFT	(0)

#define MSVDX_MTX_SOFT_RESET_MTX_RESET_MASK		(0x00000001)
#define MSVDX_MTX_SOFT_RESET_MTX_RESET_SHIFT		(0)

#define MSVDX_MTX_ENABLE_MTX_ENABLE_MASK		(0x00000001)
#define MSVDX_MTX_ENABLE_MTX_ENABLE_SHIFT		(0)

#define MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK		(0x00000100)
#define MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_SHIFT		(8)

#define MSVDX_INTERRUPT_STATUS_CR_MMU_FAULT_IRQ_MASK	(0x00000F00)
#define MSVDX_INTERRUPT_STATUS_CR_MMU_FAULT_IRQ_SHIFT	(8)

#define MSVDX_INTERRUPT_STATUS_CR_MTX_IRQ_MASK		(0x00004000)
#define MSVDX_INTERRUPT_STATUS_CR_MTX_IRQ_SHIFT		(14)

#define MSVDX_MMU_CONTROL0_CR_MMU_PAUSE_MASK			(0x00000002)
#define MSVDX_MMU_CONTROL0_CR_MMU_PAUSE_SHIFT			(1)

#define MSVDX_MTX_RAM_BANK_CR_MTX_RAM_BANK_SIZE_MASK		(0x000F0000)
#define MSVDX_MTX_RAM_BANK_CR_MTX_RAM_BANK_SIZE_SHIFT		(16)

#define MSVDX_MTX_DEBUG_MTX_DBG_IS_SLAVE_MASK                (0x00000004)
#define MSVDX_MTX_DEBUG_MTX_DBG_IS_SLAVE_LSBMASK             (0x00000001)
#define MSVDX_MTX_DEBUG_MTX_DBG_IS_SLAVE_SHIFT               (2)

#define MSVDX_MTX_DEBUG_MTX_DBG_GPIO_IN_MASK         (0x00000003)
#define MSVDX_MTX_DEBUG_MTX_DBG_GPIO_IN_LSBMASK              (0x00000003)
#define MSVDX_MTX_DEBUG_MTX_DBG_GPIO_IN_SHIFT                (0)

#define MSVDX_RENDEC_BUFFER_SIZE_RENDEC_BUFFER_SIZE0_MASK	(0x0000FFFF)
#define MSVDX_RENDEC_BUFFER_SIZE_RENDEC_BUFFER_SIZE0_SHIFT	(0)

#define MSVDX_RENDEC_BUFFER_SIZE_RENDEC_BUFFER_SIZE1_MASK	(0xFFFF0000)
#define MSVDX_RENDEC_BUFFER_SIZE_RENDEC_BUFFER_SIZE1_SHIFT	(16)

#define MSVDX_RENDEC_CONTROL1_RENDEC_DECODE_START_SIZE_MASK	(0x000000FF)
#define MSVDX_RENDEC_CONTROL1_RENDEC_DECODE_START_SIZE_SHIFT	(0)

#define MSVDX_RENDEC_CONTROL1_RENDEC_BURST_SIZE_W_MASK		(0x000C0000)
#define MSVDX_RENDEC_CONTROL1_RENDEC_BURST_SIZE_W_SHIFT		(18)

#define MSVDX_RENDEC_CONTROL1_RENDEC_BURST_SIZE_R_MASK		(0x00030000)
#define MSVDX_RENDEC_CONTROL1_RENDEC_BURST_SIZE_R_SHIFT		(16)

#define MSVDX_RENDEC_CONTROL1_RENDEC_EXTERNAL_MEMORY_MASK	(0x01000000)
#define MSVDX_RENDEC_CONTROL1_RENDEC_EXTERNAL_MEMORY_SHIFT	(24)

#define MSVDX_RENDEC_CONTROL1_RENDEC_DEC_DISABLE_MASK		(0x08000000)
#define MSVDX_RENDEC_CONTROL1_RENDEC_DEC_DISABLE_SHIFT	(27)

#define MSVDX_RENDEC_CONTROL0_RENDEC_INITIALISE_MASK		(0x00000001)
#define MSVDX_RENDEC_CONTROL0_RENDEC_INITIALISE_SHIFT		(0)

#define VEC_SHIFTREG_CONTROL_SR_MASTER_SELECT_MASK         (0x00000300)
#define VEC_SHIFTREG_CONTROL_SR_MASTER_SELECT_SHIFT                (8)

#define MTX_CORE_CR_MTX_SYSC_CDMAC_BURSTSIZE_MASK               (0x07000000)
#define MTX_CORE_CR_MTX_SYSC_CDMAC_BURSTSIZE_SHIFT              (24)

#define MTX_CORE_CR_MTX_SYSC_CDMAC_RNW_MASK             (0x00020000)
#define MTX_CORE_CR_MTX_SYSC_CDMAC_RNW_SHIFT            (17)

#define MTX_CORE_CR_MTX_SYSC_CDMAC_ENABLE_MASK          (0x00010000)
#define MTX_CORE_CR_MTX_SYSC_CDMAC_ENABLE_SHIFT         (16)

#define MTX_CORE_CR_MTX_SYSC_CDMAC_LENGTH_MASK          (0x0000FFFF)
#define MTX_CORE_CR_MTX_SYSC_CDMAC_LENGTH_SHIFT         (0)

#define MSVDX_CORE_CR_MSVDX_CONTROL_DMAC_CH0_SELECT_MASK                (0x00001000)
#define MSVDX_CORE_CR_MSVDX_CONTROL_DMAC_CH0_SELECT_SHIFT               (12)

#define MSVDX_CORE_CR_MMU_CONTROL0_CR_MMU_INVALDC_MASK          (0x00000008)
#define MSVDX_CORE_CR_MMU_CONTROL0_CR_MMU_INVALDC_SHIFT         (3)

#define DMAC_DMAC_COUNT_BSWAP_LSBMASK           (0x00000001)
#define DMAC_DMAC_COUNT_BSWAP_SHIFT             (30)

#define DMAC_DMAC_COUNT_PW_LSBMASK              (0x00000003)
#define DMAC_DMAC_COUNT_PW_SHIFT                (27)

#define DMAC_DMAC_COUNT_DIR_LSBMASK             (0x00000001)
#define DMAC_DMAC_COUNT_DIR_SHIFT               (26)

#define DMAC_DMAC_COUNT_PI_LSBMASK              (0x00000003)
#define DMAC_DMAC_COUNT_PI_SHIFT                (24)

#define DMAC_DMAC_COUNT_CNT_LSBMASK             (0x0000FFFF)
#define DMAC_DMAC_COUNT_CNT_SHIFT               (0)

#define DMAC_DMAC_PERIPH_ACC_DEL_LSBMASK                (0x00000007)
#define DMAC_DMAC_PERIPH_ACC_DEL_SHIFT          (29)

#define DMAC_DMAC_PERIPH_INCR_LSBMASK           (0x00000001)
#define DMAC_DMAC_PERIPH_INCR_SHIFT             (27)

#define DMAC_DMAC_PERIPH_BURST_LSBMASK          (0x00000007)
#define DMAC_DMAC_PERIPH_BURST_SHIFT            (24)

#define DMAC_DMAC_PERIPHERAL_ADDR_ADDR_MASK             (0x007FFFFF)
#define DMAC_DMAC_PERIPHERAL_ADDR_ADDR_LSBMASK          (0x007FFFFF)
#define DMAC_DMAC_PERIPHERAL_ADDR_ADDR_SHIFT            (0)

#define DMAC_DMAC_COUNT_EN_MASK         (0x00010000)
#define DMAC_DMAC_COUNT_EN_SHIFT                (16)

#define DMAC_DMAC_IRQ_STAT_TRANSFER_FIN_MASK            (0x00020000)

/*watch dog for FE and BE*/
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_OFFSET		(0x0064)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CNT_CTRL */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CNT_CTRL_MASK		(0x00060000)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CNT_CTRL_LSBMASK		(0x00000003)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CNT_CTRL_SHIFT		(17)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_ENABLE */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ENABLE_MASK		(0x00010000)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ENABLE_LSBMASK		(0x00000001)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ENABLE_SHIFT		(16)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_ACTION1 */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION1_MASK		(0x00003000)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION1_LSBMASK		(0x00000003)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION1_SHIFT		(12)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_ACTION0 */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION0_MASK		(0x00000100)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION0_LSBMASK		(0x00000001)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_ACTION0_SHIFT		(8)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CLEAR_SELECT */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLEAR_SELECT_MASK		(0x00000030)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLEAR_SELECT_LSBMASK		(0x00000003)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLEAR_SELECT_SHIFT		(4)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CLKDIV_SELECT */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLKDIV_SELECT_MASK		(0x00000007)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLKDIV_SELECT_LSBMASK		(0x00000007)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL_FE_WDT_CLKDIV_SELECT_SHIFT		(0)

#define MSVDX_CORE_CR_FE_MSVDX_WDTIMER_OFFSET		(0x0068)

/* MSVDX_CORE, CR_FE_MSVDX_WDTIMER, FE_WDT_COUNTER */
#define MSVDX_CORE_CR_FE_MSVDX_WDTIMER_FE_WDT_COUNTER_MASK		(0x0000FFFF)
#define MSVDX_CORE_CR_FE_MSVDX_WDTIMER_FE_WDT_COUNTER_LSBMASK		(0x0000FFFF)
#define MSVDX_CORE_CR_FE_MSVDX_WDTIMER_FE_WDT_COUNTER_SHIFT		(0)

#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_OFFSET		(0x006C)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_COMPAREMATCH, FE_WDT_CM1 */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM1_MASK		(0xFFFF0000)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM1_LSBMASK		(0x0000FFFF)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM1_SHIFT		(16)

/* MSVDX_CORE, CR_FE_MSVDX_WDT_COMPAREMATCH, FE_WDT_CM0 */
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM0_MASK		(0x0000FFFF)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM0_LSBMASK		(0x0000FFFF)
#define MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH_FE_WDT_CM0_SHIFT		(0)

#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_OFFSET		(0x0070)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CNT_CTRL */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CNT_CTRL_MASK		(0x001E0000)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CNT_CTRL_LSBMASK		(0x0000000F)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CNT_CTRL_SHIFT		(17)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL, BE_WDT_ENABLE */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ENABLE_MASK		(0x00010000)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ENABLE_LSBMASK		(0x00000001)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ENABLE_SHIFT		(16)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL, BE_WDT_ACTION0 */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ACTION0_MASK		(0x00000100)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ACTION0_LSBMASK		(0x00000001)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_ACTION0_SHIFT		(8)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CLEAR_SELECT */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLEAR_SELECT_MASK		(0x000000F0)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLEAR_SELECT_LSBMASK		(0x0000000F)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLEAR_SELECT_SHIFT		(4)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CLKDIV_SELECT */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLKDIV_SELECT_MASK		(0x00000007)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLKDIV_SELECT_LSBMASK		(0x00000007)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL_BE_WDT_CLKDIV_SELECT_SHIFT		(0)

#define MSVDX_CORE_CR_BE_MSVDX_WDTIMER_OFFSET		(0x0074)

/* MSVDX_CORE, CR_BE_MSVDX_WDTIMER, BE_WDT_COUNTER */
#define MSVDX_CORE_CR_BE_MSVDX_WDTIMER_BE_WDT_COUNTER_MASK		(0x0000FFFF)
#define MSVDX_CORE_CR_BE_MSVDX_WDTIMER_BE_WDT_COUNTER_LSBMASK		(0x0000FFFF)
#define MSVDX_CORE_CR_BE_MSVDX_WDTIMER_BE_WDT_COUNTER_SHIFT		(0)

#define MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH_OFFSET		(0x0078)

/* MSVDX_CORE, CR_BE_MSVDX_WDT_COMPAREMATCH, BE_WDT_CM0 */
#define MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH_BE_WDT_CM0_MASK		(0x0000FFFF)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH_BE_WDT_CM0_LSBMASK		(0x0000FFFF)
#define MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH_BE_WDT_CM0_SHIFT		(0)
/*watch dog end*/

/* Start of parser specific Host->MTX messages. */
#define	FWRK_MSGID_START_PSR_HOSTMTX_MSG	(0x80)

/* Start of parser specific MTX->Host messages. */
#define	FWRK_MSGID_START_PSR_MTXHOST_MSG	(0xC0)

/* Host defined msg, just for host use, MTX not recgnize */
#define	FWRK_MSGID_HOST_EMULATED		(0x40)

#define FWRK_MSGID_PADDING			(0)

#define FWRK_GENMSG_SIZE_TYPE		uint8_t
#define FWRK_GENMSG_SIZE_MASK		(0xFF)
#define FWRK_GENMSG_SIZE_SHIFT		(0)
#define FWRK_GENMSG_SIZE_OFFSET		(0x0000)
#define FWRK_GENMSG_ID_TYPE		uint8_t
#define FWRK_GENMSG_ID_MASK		(0xFF)
#define FWRK_GENMSG_ID_SHIFT		(0)
#define FWRK_GENMSG_ID_OFFSET		(0x0001)
#define FWRK_PADMSG_SIZE		(2)

/* vec local MEM save/restore */
#define VEC_LOCAL_MEM_BYTE_SIZE (4 * 1024)
#define VEC_LOCAL_MEM_OFFSET 0x2000

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
	int msvdx_is_setup;
	int msvdx_fw_loaded;
	void *msvdx_fw;
	int msvdx_fw_size;

	uint32_t msvdx_hw_busy;

	uint32_t *vec_local_mem_data;
	uint32_t vec_local_mem_size;
	uint32_t vec_local_mem_saved;
	uint32_t psb_dash_access_ctrl;
	uint32_t fw_status;
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
};

#define REGISTER(__group__, __reg__) (__group__##_##__reg__##_OFFSET)

#define FW_VA_LAST_SLICE_OF_EXT_DMA                                         0x00001000

struct psb_msvdx_ec_ctx *psb_msvdx_find_ec_ctx(
			struct msvdx_private *msvdx_priv,
			struct ttm_object_file *tfile,
			void *cmd);

static inline void psb_msvdx_clearirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned long mtx_int = 0;

	PSB_DEBUG_IRQ("MSVDX: clear IRQ\n");

	/* Clear MTX interrupt */
	REGIO_WRITE_FIELD_LITE(mtx_int, MSVDX_INTERRUPT_STATUS, CR_MTX_IRQ,
			       1);
	PSB_WMSVDX32(mtx_int, MSVDX_INTERRUPT_CLEAR);
}

/* following two functions also works for CLV and MFLD */
/* PSB_INT_ENABLE_R is set in psb_irq_(un)install_islands */
static inline void psb_msvdx_disableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*uint32_t ier = dev_priv->vdc_irq_mask & (~_PSB_IRQ_MSVDX_FLAG); */

	unsigned long enables = 0;

	PSB_DEBUG_IRQ("MSVDX: enable MSVDX MTX IRQ\n");
	REGIO_WRITE_FIELD_LITE(enables, MSVDX_INTERRUPT_STATUS, CR_MTX_IRQ,
			       0);
	PSB_WMSVDX32(enables, MSVDX_HOST_INTERRUPT_ENABLE);

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
	REGIO_WRITE_FIELD_LITE(enables, MSVDX_INTERRUPT_STATUS, CR_MTX_IRQ,
			       1);
	PSB_WMSVDX32(enables, MSVDX_HOST_INTERRUPT_ENABLE);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}

static inline void psb_msvdx_mtx_set_clocks(struct drm_device *dev, uint32_t clock_state)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	uint32_t old_clock_state, reg_value = 0;
	/* PSB_DEBUG_MSVDX("SetClocks to %x.\n", clock_state); */
	old_clock_state = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE);
	if (old_clock_state == clock_state)
		return;

	if (clock_state == 0) {
		/* Turn off clocks procedure */
		if (old_clock_state) {
			/* Turn off all the clocks except core */
			PSB_WMSVDX32(MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK, MSVDX_MAN_CLK_ENABLE);

			/* Make sure all the clocks are off except core */
			reg_value = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE);
			BUG_ON(reg_value != MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK);

			/* Turn off core clock */
			PSB_WMSVDX32(0, MSVDX_MAN_CLK_ENABLE);
		}
	} else {
		uint32_t clocks_en = clock_state;

		/*Make sure that core clock is not accidentally turned off */
		clocks_en |= MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK;

		/* If all clocks were disable do the bring up procedure */
		if (old_clock_state == 0) {
			/* turn on core clock */
			PSB_WMSVDX32(MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK, MSVDX_MAN_CLK_ENABLE);

			/* Make sure it is on */
			reg_value = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE);
			BUG_ON(reg_value != MSVDX_CORE_CR_MSVDX_MAN_CLK_ENABLE_CR_CORE_MAN_CLK_ENABLE_MASK);

			/* turn on the other clocks as well */
			PSB_WMSVDX32(clocks_en, MSVDX_MAN_CLK_ENABLE);

			/* Make sure that they are on */
			reg_value = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE);
			BUG_ON(reg_value != clocks_en);
		} else {
			PSB_WMSVDX32(clocks_en, MSVDX_MAN_CLK_ENABLE);

			/* Make sure that they are on */
			reg_value = PSB_RMSVDX32(MSVDX_MAN_CLK_ENABLE);
			BUG_ON(reg_value != clocks_en);
		}
	}
}

#define MSVDX_NEW_PMSTATE(drm_dev, msvdx_priv, new_state)		\
do {									\
	msvdx_priv->pmstate = new_state;				\
	sysfs_notify_dirent(msvdx_priv->sysfs_pmstate);			\
	PSB_DEBUG_PM("MSVDX: %s\n",					\
		(new_state == PSB_PMSTATE_POWERUP) ? "powerup"		\
		: ((new_state == PSB_PMSTATE_POWERDOWN) ? "powerdown"	\
			: "clockgated"));				\
} while (0)

#endif
