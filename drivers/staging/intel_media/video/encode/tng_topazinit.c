/**
#endif
 * file tng_topazinit.c
 * TOPAZ initialization and mtx-firmware upload
 *
 */

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

/* NOTE: (READ BEFORE REFINE CODE)
 * 1. The FIRMWARE's SIZE is measured by byte, we have to pass the size
 * measured by word to DMAC.
 *
 *
 *
 */

/* include headers */

/* #define DRM_DEBUG_CODE 2 */

#include <linux/firmware.h>

#include <drm/drmP.h>
#include <drm/drm.h>

#include "psb_drv.h"
#include "tng_topaz.h"
#include "psb_powermgmt.h"
#include "tng_topaz_hw_reg.h"

/* WARNING: this define is very important */
#define RAM_SIZE	(1024 * 24)
#define MTX_PC		(0x05)

#define TOPAZHP_MMU_BASE	0x80400000

#define	MEMORY_ONLY	0
#define	MEM_AND_CACHE	1
#define CACHE_ONLY	2

#define FIRMWARE_NAME	"topazhp_fw.bin"

/* #define TOPAZHP_ENCODE_FPGA */
static int tng_init_error_dump_reg(struct drm_psb_private *dev_priv)
{
	uint32_t reg_val;
	DRM_ERROR("MULTICORE Registers:\n");
	MULTICORE_READ32(0x00, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_SRST %08x\n", reg_val);
	MULTICORE_READ32(0x04, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_INT_STAT %08x\n", reg_val);
	MULTICORE_READ32(0x08, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_MTX_INT_ENAB %08x\n", reg_val);
	MULTICORE_READ32(0x0C, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_HOST_INT_ENAB %08x\n", reg_val);
	MULTICORE_READ32(0x10, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_INT_CLEAR %08x\n", reg_val);
	MULTICORE_READ32(0x14, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_MAN_CLK_GATE %08x\n", reg_val);
	MULTICORE_READ32(0x18, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZ_MTX_C_RATIO %08x\n", reg_val);
	MULTICORE_READ32(0x1c, &reg_val);
	PSB_DEBUG_GENERAL("MMU_STATUS %08x\n", reg_val);
	MULTICORE_READ32(0x1c, &reg_val);
	PSB_DEBUG_GENERAL("MMU_STATUS %08x\n", reg_val);
	MULTICORE_READ32(0x20, &reg_val);
	PSB_DEBUG_GENERAL("MMU_MEM_REQ %08x\n", reg_val);
	MULTICORE_READ32(0x24, &reg_val);
	PSB_DEBUG_GENERAL("MMU_CONTROL0 %08x\n", reg_val);
	MULTICORE_READ32(0x28, &reg_val);
	PSB_DEBUG_GENERAL("MMU_CONTROL1 %08x\n", reg_val);
	MULTICORE_READ32(0x2c , &reg_val);
	PSB_DEBUG_GENERAL("MMU_CONTROL2 %08x\n", reg_val);
	MULTICORE_READ32(0x30, &reg_val);
	PSB_DEBUG_GENERAL("MMU_DIR_LIST_BASE %08x\n", reg_val);
	MULTICORE_READ32(0x38, &reg_val);
	PSB_DEBUG_GENERAL("MMU_TILE %08x\n", reg_val);
	MULTICORE_READ32(0x44, &reg_val);
	PSB_DEBUG_GENERAL("MTX_DEBUG_MSTR %08x\n", reg_val);
	MULTICORE_READ32(0x48, &reg_val);
	PSB_DEBUG_GENERAL("MTX_DEBUG_SLV %08x\n", reg_val);
	MULTICORE_READ32(0x50, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_CORE_SEL_0 %08x\n", reg_val);
	MULTICORE_READ32(0x54, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_CORE_SEL_1 %08x\n", reg_val);
	MULTICORE_READ32(0x58, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_HW_CFG %08x\n", reg_val);
	MULTICORE_READ32(0x60, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_CMD_FIFO_WRITE %08x\n", reg_val);
	MULTICORE_READ32(0x64, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_CMD_FIFO_WRITE_SPACE %08x\n", reg_val);
	MULTICORE_READ32(0x70, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZ_CMD_FIFO_READ %08x\n", reg_val);
	MULTICORE_READ32(0x74, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZ_CMD_FIFO_READ_AVAILABLE %08x\n", reg_val);
	MULTICORE_READ32(0x78, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZ_CMD_FIFO_FLUSH %08x\n", reg_val);
	MULTICORE_READ32(0x80, &reg_val);
	PSB_DEBUG_GENERAL("MMU_TILE_EXT %08x\n", reg_val);
	MULTICORE_READ32(0x100, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_1 %08x\n", reg_val);
	MULTICORE_READ32(0x104, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_2 %08x\n", reg_val);
	MULTICORE_READ32(0x108, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_3 %08x\n", reg_val);
	MULTICORE_READ32(0x110, &reg_val);
	PSB_DEBUG_GENERAL("CYCLE_COUNTER %08x\n", reg_val);
	MULTICORE_READ32(0x114, &reg_val);
	PSB_DEBUG_GENERAL("CYCLE_COUNTER_CTRL %08x\n", reg_val);
	MULTICORE_READ32(0x118, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_IDLE_PWR_MAN %08x\n", reg_val);
	MULTICORE_READ32(0x124, &reg_val);
	PSB_DEBUG_GENERAL("DIRECT_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x128, &reg_val);
	PSB_DEBUG_GENERAL("INTRA_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x12c, &reg_val);
	PSB_DEBUG_GENERAL("INTER_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x130, &reg_val);
	PSB_DEBUG_GENERAL("INTRA_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x134, &reg_val);
	PSB_DEBUG_GENERAL("QPCB_QPCR_OFFSET %08x\n", reg_val);
	MULTICORE_READ32(0x140, &reg_val);
	PSB_DEBUG_GENERAL("INTER_INTRA_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x144, &reg_val);
	PSB_DEBUG_GENERAL("SKIPPED_CODED_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x148, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_ALPHA_COEFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x14c, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_GAMMA_COEFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x150, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_CUTOFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x154, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_ALPHA_COEFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x158, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_GAMMA_COEFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x15c, &reg_val);
	PSB_DEBUG_GENERAL("POLYNOM_CUTOFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x300, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_4 %08x\n", reg_val);
	MULTICORE_READ32(0x304, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_5 %08x\n", reg_val);
	MULTICORE_READ32(0x308, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_6 %08x\n", reg_val);
	MULTICORE_READ32(0x30c, &reg_val);
	PSB_DEBUG_GENERAL("FIRMWARE_REG_7 %08x\n", reg_val);
	MULTICORE_READ32(0x3b0, &reg_val);
	PSB_DEBUG_GENERAL("MULTICORE_RSVD0 %08x\n", reg_val);

	DRM_ERROR("TopazHP Core Registers:\n");
	TOPAZCORE_READ32(0, 0x0, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_SRST %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x4, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_INTSTAT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x8, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_MTX_INTENAB %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0xc, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_HOST_INTENAB %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x10, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_INTCLEAR %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x14, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_INT_COMB_SEL %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x18, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_BUSY %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x24, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_AUTO_CLOCK_GATING %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x28, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_MAN_CLOCK_GATING %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x30, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_RTM %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x34, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_RTM_VALUE %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x38, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_MB_PERFORMANCE_RESULT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3c, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_MB_PERFORMANCE_MB_NUMBER %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x188, &reg_val);
	PSB_DEBUG_GENERAL("FIELD_PARITY %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3d0, &reg_val);
	PSB_DEBUG_GENERAL("WEIGHTED_PRED_CONTROL %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3d4, &reg_val);
	PSB_DEBUG_GENERAL("WEIGHTED_PRED_COEFFS %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3e0, &reg_val);
	PSB_DEBUG_GENERAL("WEIGHTED_PRED_INV_WEIGHT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3f0, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_RSVD0 %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3f4, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZHP_CRC_CLEAR %08x\n", reg_val);


	DRM_ERROR("MTX Registers:\n");
	MTX_READ32(0x00, &reg_val);
	PSB_DEBUG_GENERAL("MTX_ENABLE %08x\n", reg_val);
	MTX_READ32(0x08, &reg_val);
	PSB_DEBUG_GENERAL("MTX_STATUS %08x\n", reg_val);
	MTX_READ32(0x80, &reg_val);
	PSB_DEBUG_GENERAL("MTX_KICK %08x\n", reg_val);
	MTX_READ32(0x88, &reg_val);
	PSB_DEBUG_GENERAL("MTX_KICKI %08x\n", reg_val);
	MTX_READ32(0x90, &reg_val);
	PSB_DEBUG_GENERAL("MTX_FAULT0 %08x\n", reg_val);
	MTX_READ32(0xf8, &reg_val);
	PSB_DEBUG_GENERAL("MTX_REGISTER_READ_WRITE_DATA %08x\n", reg_val);
	MTX_READ32(0xfc, &reg_val);
	PSB_DEBUG_GENERAL("MTX_REGISTER_READ_WRITE_REQUEST %08x\n", reg_val);
	MTX_READ32(0x100, &reg_val);
	PSB_DEBUG_GENERAL("MTX_RAM_ACCESS_DATA_EXCHANGE %08x\n", reg_val);
	MTX_READ32(0x104, &reg_val);
	PSB_DEBUG_GENERAL("MTX_RAM_ACCESS_DATA_TRANSFER %08x\n", reg_val);
	MTX_READ32(0x108, &reg_val);
	PSB_DEBUG_GENERAL("MTX_RAM_ACCESS_CONTROL %08x\n", reg_val);
	MTX_READ32(0x10c, &reg_val);
	PSB_DEBUG_GENERAL("MTX_RAM_ACCESS_STATUS %08x\n", reg_val);
	MTX_READ32(0x200, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SOFT_RESET %08x\n", reg_val);
	MTX_READ32(0x340, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SYSC_CDMAC %08x\n", reg_val);
	MTX_READ32(0x344, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SYSC_CDMAA %08x\n", reg_val);
	MTX_READ32(0x348, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SYSC_CDMAS0 %08x\n", reg_val);
	MTX_READ32(0x34c, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SYSC_CDMAS1 %08x\n", reg_val);
	MTX_READ32(0x350, &reg_val);
	PSB_DEBUG_GENERAL("MTX_SYSC_CDMAT %08x\n", reg_val);

	DRM_ERROR("DMA Registers:\n");
	DMAC_READ32(0x00, &reg_val);
	PSB_DEBUG_GENERAL("DMA_Setup_n %08x\n", reg_val);
	DMAC_READ32(0x04, &reg_val);
	PSB_DEBUG_GENERAL("DMA_Count_n %08x\n", reg_val);
	DMAC_READ32(0x08, &reg_val);
	PSB_DEBUG_GENERAL(" DMA_Peripheral_param_n %08x\n", reg_val);
	DMAC_READ32(0x0C, &reg_val);
	PSB_DEBUG_GENERAL("DMA_IRQ_Stat_n %08x\n", reg_val);
	DMAC_READ32(0x10, &reg_val);
	PSB_DEBUG_GENERAL("DMA_2D_Mode_n %08x\n", reg_val);
	DMAC_READ32(0x14, &reg_val);
	PSB_DEBUG_GENERAL("DMA_Peripheral_addr_n %08x\n", reg_val);
	DMAC_READ32(0x18, &reg_val);
	PSB_DEBUG_GENERAL("DMA_Per_hold %08x\n", reg_val);
	return 0;
}

/* static function define */
static int32_t mtx_dmac_transfer(struct drm_psb_private *dev_priv,
			      uint32_t channel, uint32_t src_phy_addr,
			      uint32_t offset, uint32_t mtx_addr,
			      uint32_t byte_num, uint32_t is_write);

static int32_t get_mtx_control_from_dash(struct drm_psb_private *dev_priv);

static void release_mtx_control_from_dash(struct drm_psb_private *dev_priv);

static void tng_topaz_mmu_hwsetup(struct drm_psb_private *dev_priv);

static int mtx_dma_write(
	struct drm_device *dev,
	struct ttm_buffer_object *src_bo,
	uint32_t dst_ram_addr,
	uint32_t size);

static int tng_error_dump_reg(struct drm_psb_private *dev_priv)
{
	uint32_t reg_val;
	DRM_ERROR("MULTICORE Registers:\n");
	MULTICORE_READ32(0x00, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_SRST %08x\n", reg_val);
	MULTICORE_READ32(0x04, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_INT_STAT %08x\n", reg_val);
	MULTICORE_READ32(0x08, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_MTX_INT_ENAB %08x\n", reg_val);
	MULTICORE_READ32(0x0C, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_HOST_INT_ENAB %08x\n", reg_val);
	MULTICORE_READ32(0x10, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_INT_CLEAR %08x\n", reg_val);
	MULTICORE_READ32(0x14, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_MAN_CLK_GATE %08x\n", reg_val);
	MULTICORE_READ32(0x18, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZ_MTX_C_RATIO %08x\n", reg_val);
	MULTICORE_READ32(0x1c, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_STATUS %08x\n", reg_val);
	MULTICORE_READ32(0x1c, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_STATUS %08x\n", reg_val);
	MULTICORE_READ32(0x20, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_MEM_REQ %08x\n", reg_val);
	MULTICORE_READ32(0x24, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_CONTROL0 %08x\n", reg_val);
	MULTICORE_READ32(0x28, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_CONTROL1 %08x\n", reg_val);
	MULTICORE_READ32(0x2c , &reg_val);
	PSB_DEBUG_TOPAZ("MMU_CONTROL2 %08x\n", reg_val);
	MULTICORE_READ32(0x30, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_DIR_LIST_BASE %08x\n", reg_val);
	MULTICORE_READ32(0x38, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_TILE %08x\n", reg_val);
	MULTICORE_READ32(0x44, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_DEBUG_MSTR %08x\n", reg_val);
	MULTICORE_READ32(0x48, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_DEBUG_SLV %08x\n", reg_val);
	MULTICORE_READ32(0x50, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_CORE_SEL_0 %08x\n", reg_val);
	MULTICORE_READ32(0x54, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_CORE_SEL_1 %08x\n", reg_val);
	MULTICORE_READ32(0x58, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_HW_CFG %08x\n", reg_val);
	MULTICORE_READ32(0x60, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_CMD_FIFO_WRITE %08x\n", reg_val);
	MULTICORE_READ32(0x64, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_CMD_FIFO_WRITE_SPACE %08x\n", reg_val);
	MULTICORE_READ32(0x70, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZ_CMD_FIFO_READ %08x\n", reg_val);
	MULTICORE_READ32(0x74, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZ_CMD_FIFO_READ_AVAILABLE %08x\n", reg_val);
	MULTICORE_READ32(0x78, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZ_CMD_FIFO_FLUSH %08x\n", reg_val);
	MULTICORE_READ32(0x80, &reg_val);
	PSB_DEBUG_TOPAZ("MMU_TILE_EXT %08x\n", reg_val);
	MULTICORE_READ32(0x100, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_1 %08x\n", reg_val);
	MULTICORE_READ32(0x104, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_2 %08x\n", reg_val);
	MULTICORE_READ32(0x108, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_3 %08x\n", reg_val);
	MULTICORE_READ32(0x110, &reg_val);
	PSB_DEBUG_TOPAZ("CYCLE_COUNTER %08x\n", reg_val);
	MULTICORE_READ32(0x114, &reg_val);
	PSB_DEBUG_TOPAZ("CYCLE_COUNTER_CTRL %08x\n", reg_val);
	MULTICORE_READ32(0x118, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_IDLE_PWR_MAN %08x\n", reg_val);
	MULTICORE_READ32(0x124, &reg_val);
	PSB_DEBUG_TOPAZ("DIRECT_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x128, &reg_val);
	PSB_DEBUG_TOPAZ("INTRA_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x12c, &reg_val);
	PSB_DEBUG_TOPAZ("INTER_BIAS_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x130, &reg_val);
	PSB_DEBUG_TOPAZ("INTRA_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x134, &reg_val);
	PSB_DEBUG_TOPAZ("QPCB_QPCR_OFFSET %08x\n", reg_val);
	MULTICORE_READ32(0x140, &reg_val);
	PSB_DEBUG_TOPAZ("INTER_INTRA_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x144, &reg_val);
	PSB_DEBUG_TOPAZ("SKIPPED_CODED_SCALE_TABLE %08x\n", reg_val);
	MULTICORE_READ32(0x148, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_ALPHA_COEFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x14c, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_GAMMA_COEFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x150, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_CUTOFF_CORE0 %08x\n", reg_val);
	MULTICORE_READ32(0x154, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_ALPHA_COEFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x158, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_GAMMA_COEFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x15c, &reg_val);
	PSB_DEBUG_TOPAZ("POLYNOM_CUTOFF_CORE1 %08x\n", reg_val);
	MULTICORE_READ32(0x300, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_4 %08x\n", reg_val);
	MULTICORE_READ32(0x304, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_5 %08x\n", reg_val);
	MULTICORE_READ32(0x308, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_6 %08x\n", reg_val);
	MULTICORE_READ32(0x30c, &reg_val);
	PSB_DEBUG_TOPAZ("FIRMWARE_REG_7 %08x\n", reg_val);
	MULTICORE_READ32(0x3b0, &reg_val);
	PSB_DEBUG_TOPAZ("MULTICORE_RSVD0 %08x\n", reg_val);

	DRM_ERROR("TopazHP Core Registers:\n");
	TOPAZCORE_READ32(0, 0x0, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_SRST %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x4, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_INTSTAT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x8, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_MTX_INTENAB %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0xc, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_HOST_INTENAB %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x10, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_INTCLEAR %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x14, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_INT_COMB_SEL %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x18, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_BUSY %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x24, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_AUTO_CLOCK_GATING %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x28, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_MAN_CLOCK_GATING %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x30, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_RTM %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x34, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_RTM_VALUE %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x38, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_MB_PERFORMANCE_RESULT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3c, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_MB_PERFORMANCE_MB_NUMBER %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x188, &reg_val);
	PSB_DEBUG_TOPAZ("FIELD_PARITY %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3d0, &reg_val);
	PSB_DEBUG_TOPAZ("WEIGHTED_PRED_CONTROL %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3d4, &reg_val);
	PSB_DEBUG_TOPAZ("WEIGHTED_PRED_COEFFS %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3e0, &reg_val);
	PSB_DEBUG_TOPAZ("WEIGHTED_PRED_INV_WEIGHT %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3f0, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_RSVD0 %08x\n", reg_val);
	TOPAZCORE_READ32(0, 0x3f4, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZHP_CRC_CLEAR %08x\n", reg_val);


	DRM_ERROR("MTX Registers:\n");
	MTX_READ32(0x00, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_ENABLE %08x\n", reg_val);
	MTX_READ32(0x08, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_STATUS %08x\n", reg_val);
	MTX_READ32(0x80, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_KICK %08x\n", reg_val);
	MTX_READ32(0x88, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_KICKI %08x\n", reg_val);
	MTX_READ32(0x90, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_FAULT0 %08x\n", reg_val);
	MTX_READ32(0xf8, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_REGISTER_READ_WRITE_DATA %08x\n", reg_val);
	MTX_READ32(0xfc, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_REGISTER_READ_WRITE_REQUEST %08x\n", reg_val);
	MTX_READ32(0x100, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_RAM_ACCESS_DATA_EXCHANGE %08x\n", reg_val);
	MTX_READ32(0x104, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_RAM_ACCESS_DATA_TRANSFER %08x\n", reg_val);
	MTX_READ32(0x108, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_RAM_ACCESS_CONTROL %08x\n", reg_val);
	MTX_READ32(0x10c, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_RAM_ACCESS_STATUS %08x\n", reg_val);
	MTX_READ32(0x200, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SOFT_RESET %08x\n", reg_val);
	MTX_READ32(0x340, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SYSC_CDMAC %08x\n", reg_val);
	MTX_READ32(0x344, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SYSC_CDMAA %08x\n", reg_val);
	MTX_READ32(0x348, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SYSC_CDMAS0 %08x\n", reg_val);
	MTX_READ32(0x34c, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SYSC_CDMAS1 %08x\n", reg_val);
	MTX_READ32(0x350, &reg_val);
	PSB_DEBUG_TOPAZ("MTX_SYSC_CDMAT %08x\n", reg_val);

	DRM_ERROR("DMA Registers:\n");
	DMAC_READ32(0x00, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_Setup_n %08x\n", reg_val);
	DMAC_READ32(0x04, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_Count_n %08x\n", reg_val);
	DMAC_READ32(0x08, &reg_val);
	PSB_DEBUG_TOPAZ(" DMA_Peripheral_param_n %08x\n", reg_val);
	DMAC_READ32(0x0C, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_IRQ_Stat_n %08x\n", reg_val);
	DMAC_READ32(0x10, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_2D_Mode_n %08x\n", reg_val);
	DMAC_READ32(0x14, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_Peripheral_addr_n %08x\n", reg_val);
	DMAC_READ32(0x18, &reg_val);
	PSB_DEBUG_TOPAZ("DMA_Per_hold %08x\n", reg_val);
	return 0;
}

int equal_func(uint32_t reqval, uint32_t regval, uint32_t mask)
{
	return ((reqval & mask) == (regval & mask));
}

int notequal_func(uint32_t reqval, uint32_t regval, uint32_t mask)
{
	return ((reqval & mask) != (regval & mask));
}

int tng_topaz_wait_for_register(
	struct drm_psb_private *dev_priv,
	uint32_t check_func,
	uint32_t addr,
	uint32_t value,
	uint32_t mask)
{
#ifdef KPDUMP
	printk(KERN_ERR "POL :REG_TOPAZHP_MULTICORE:0x%x 0x%x 0x%x\n",
		addr, value, mask);
#endif
	uint32_t tmp;
	uint32_t count = 10000;
	int (*func)(uint32_t reqval, uint32_t regval, uint32_t mask);

	switch (check_func) {
	case CHECKFUNC_ISEQUAL:
		func = &equal_func;
		break;
	case CHECKFUNC_NOTEQUAL:
		func = &notequal_func;
		break;
	default:
		func = &equal_func;
		break;
	}

	/* # poll topaz register for certain times */
	while (count) {
		MM_READ32(addr, 0, &tmp);

		if (func(value, tmp, mask))
			return 0;

		/* FIXME: use cpu_relax instead */
		PSB_UDELAY(1000);/* derive from reference driver */
		--count;
	}

	/* # now waiting is timeout, return 1 indicat failed */
	/* XXX: testsuit means a timeout 10000 */

	if (check_func == CHECKFUNC_ISEQUAL) {
		DRM_ERROR("TOPAZ: Time out to poll addr(0x%x) " \
			"expected value(0x%08x), " \
			"actual 0x%08x = (0x%08x & 0x%08x)\n",
			addr, value & mask, tmp & mask, tmp, mask);
	} else if (check_func == CHECKFUNC_NOTEQUAL) {
		DRM_ERROR("ERROR time out to poll addr(0x%x) expected " \
			"0x%08x != (0x%08x & 0x%08x)\n",
			addr, value & mask, tmp, mask);
	}

	return -EBUSY;

}

static ssize_t psb_topaz_pmstate_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct drm_psb_private *dev_priv;
	struct tng_topaz_private *topaz_priv;
	unsigned int pmstate;
	unsigned long flags;
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	dev_priv = drm_dev->dev_private;
	topaz_priv = dev_priv->topaz_private;
	pmstate = topaz_priv->pmstate;

	pmstate = topaz_priv->pmstate;
	spin_lock_irqsave(&topaz_priv->topaz_lock, flags);
	ret = snprintf(buf, 64, "%s\n",
		       (pmstate == PSB_PMSTATE_POWERUP) ?
		       "powerup" : "powerdown");
	spin_unlock_irqrestore(&topaz_priv->topaz_lock, flags);

	return ret;
}

static DEVICE_ATTR(topaz_pmstate, 0444, psb_topaz_pmstate_show, NULL);

#ifdef TOPAZHP_ENCODE_FPGA
#define DEFAULT_TILE_STRIDE 0
static void tng_topaz_mmu_configure(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_TILE(0),
		F_ENCODE(1, TOPAZHP_TOP_CR_TILE_ENABLE) |
		F_ENCODE(DEFAULT_TILE_STRIDE, TOPAZHP_TOP_CR_TILE_STRIDE) |
			F_ENCODE((PSB_MEM_VRAM_START + 0x800000 +
			pci_resource_len(dev->pdev, 2) - 0x800000) >> 20,
		TOPAZHP_TOP_CR_TILE_MAX_ADDR) |
			F_ENCODE((PSB_MEM_VRAM_START + 0x800000) >> 20,
		TOPAZHP_TOP_CR_TILE_MIN_ADDR));
}
#endif

void tng_topaz_mmu_enable_tiling(
	struct drm_device *dev,
	struct drm_psb_private *dev_priv,
	uint32_t pipe_id)
{
	uint32_t reg_val;
	uint32_t min_addr = 0x80000000 + 0x800000;
	uint32_t max_addr = 0x80000000 + pci_resource_len(dev->pdev, 2);
	PSB_DEBUG_TOPAZ("TOPAZ: Enable tiled memory from %08x ~ %08x\n",
		min_addr, max_addr);
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_TILE_ENABLE) |
		F_ENCODE(0, TOPAZHP_TOP_CR_TILE_STRIDE) |
		F_ENCODE(((0x80000000 + 0x800000) +
			(pci_resource_len(dev->pdev, 2) - 0x800000)) >> 20,
			TOPAZHP_TOP_CR_TILE_MAX_ADDR) |
			F_ENCODE((0x80000000 + 0x800000) >> 20,
		TOPAZHP_TOP_CR_TILE_MIN_ADDR);

	TOPAZCORE_WRITE32(pipe_id, 0x0038, reg_val);
}

void tng_powerdown_topaz(struct work_struct *work)
{
	struct tng_topaz_private *topaz_priv =
		container_of(work, struct tng_topaz_private,
					topaz_suspend_work);
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)topaz_priv->dev->dev_private;
	struct drm_device *dev = (struct drm_device *)topaz_priv->dev;

	PSB_DEBUG_GENERAL("TOPAZ: Task start\n");

	tng_topaz_dequeue_send(dev);

	/* If topaz_busy is not 0, then this task should return */
	if (drm_topaz_pmpolicy != PSB_PMPOLICY_NOPM) {
		if (topaz_priv->topaz_busy == 0) {
			PSB_DEBUG_GENERAL("TOPAZ: topaz_busy = 0, " \
				"try to power down topaz\n");
			ospm_apm_power_down_topaz(topaz_priv->dev);
		} else {
			PSB_DEBUG_GENERAL("TOPAZ: topaz_busy = 1," \
				 " bypass power down and saving context\n");
		}
	} else {
		if (get_ctx_cnt(dev) > 1 &&
		    topaz_priv->topaz_busy == 0) {
			PSB_DEBUG_GENERAL("TOPAZ: more than 1 context," \
			" save current context status\n");
			tng_topaz_save_mtx_state(dev);
		}
	}
out:
	PSB_DEBUG_GENERAL("TOPAZ: Task finish\n");
	return;
}

/* this function finish the first part of initialization, the rest
 * should be done in pnw_topaz_setup_fw
 */
int tng_topaz_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	int ret = 0, n;
	int ui32RegValue;
	bool is_iomem;
	uint32_t reg_val;
	struct tng_topaz_private *topaz_priv;
	void *topaz_bo_virt;
	PSB_DEBUG_TOPAZ("TOPAZ: init topazsc data structures\n");
	topaz_priv = kmalloc(sizeof(struct tng_topaz_private), GFP_KERNEL);
	if (topaz_priv == NULL)
		return -1;
	dev_priv->topaz_private = topaz_priv;
	memset(topaz_priv, 0, sizeof(struct tng_topaz_private));

	/* get device --> drm_device --> drm_psb_private --> topaz_priv
	* for psb_topaz_pmstate_show: topaz_pmpolicy
	* if not pci_set_drvdata, can't get drm_device from device
	*/
	pci_set_drvdata(dev->pdev, dev);

	if (device_create_file(&dev->pdev->dev, &dev_attr_topaz_pmstate))
		DRM_ERROR("TOPAZ: could not create sysfs file\n");
	topaz_priv->sysfs_pmstate = sysfs_get_dirent(dev->pdev->dev.kobj.sd,
						     NULL,
						     "topaz_pmstate");

	topaz_priv = dev_priv->topaz_private;
	topaz_priv->dev = dev;
	INIT_DELAYED_WORK(&topaz_priv->topaz_suspend_work,
		&tng_powerdown_topaz);

	INIT_LIST_HEAD(&topaz_priv->topaz_queue);
	mutex_init(&topaz_priv->topaz_mutex);
	spin_lock_init(&topaz_priv->topaz_lock);

	topaz_priv->topaz_busy = 0;
	topaz_priv->topaz_fw_loaded = 0;
	topaz_priv->cur_codec = 0;
	topaz_priv->topaz_hw_busy = 1;

	topaz_priv->saved_queue = kzalloc(\
			sizeof(struct tng_topaz_cmd_queue), \
			GFP_KERNEL);
	if (!topaz_priv->saved_queue) {
		DRM_ERROR("TOPAZ: Failed to alloc memory for saved queue\n");
		return -1;
	}

	topaz_priv->saved_cmd = kzalloc(2048, GFP_KERNEL);
	if (!topaz_priv->saved_cmd) {
		DRM_ERROR("TOPAZ: Failed to alloc memory for saved cmd\n");
		return -1;
	}

	/* tng_topaz_mmu_configure(dev); */

	tng_topaz_mmu_flushcache(dev_priv);

	ui32RegValue = F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_IO_SOFT_RESET);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, ui32RegValue);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 0x0);

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG,
		&topaz_priv->topaz_num_pipes);

	topaz_priv->topaz_num_pipes =
		F_EXTRACT(topaz_priv->topaz_num_pipes,
			TOPAZHP_TOP_CR_NUM_CORES_SUPPORTED);

	PSB_DEBUG_TOPAZ("TOPAZ: Number of pipes: %d\n",
		topaz_priv->topaz_num_pipes);

	if (topaz_priv->topaz_num_pipes > TOPAZHP_PIPE_NUM) {
		DRM_ERROR("TOPAZ: Number of pipes: 0x%x\n",
			topaz_priv->topaz_num_pipes);
		topaz_priv->topaz_num_pipes = TOPAZHP_PIPE_NUM;
	}

	/* tng_topaz_mmu_flushcache(dev_priv); */

	tng_topaz_mmu_hwsetup(dev_priv);

	/* tng_topaz_mmu_enable_tiling(dev, dev_priv, 0); */

	/* # gain write back structure,we may only need 32+4=40DW */
	ret = ttm_buffer_object_create(bdev, 4096, ttm_bo_type_kernel,
		DRM_PSB_FLAG_MEM_MMU | TTM_PL_FLAG_NO_EVICT,
		0, 0, 0, NULL, &(topaz_priv->topaz_bo));
	if (ret) {
		DRM_ERROR("TOPAZ: failed to allocate topaz BO.\n");
		ttm_bo_unref(&topaz_priv->topaz_bo);
		return ret;
	}

	topaz_priv->frame_count = 0;

	ret = ttm_bo_kmap(topaz_priv->topaz_bo, 0,
			  topaz_priv->topaz_bo->num_pages,
			  &topaz_priv->topaz_bo_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: map topaz BO bo failed......\n");
		ttm_bo_unref(&topaz_priv->topaz_bo);
		return ret;
	}

	topaz_bo_virt = ttm_kmap_obj_virtual(&topaz_priv->topaz_bo_kmap,
					     &is_iomem);

	topaz_priv->topaz_sync_addr = (uint32_t *)topaz_bo_virt;
	*topaz_priv->topaz_sync_addr = ~0;

	topaz_priv->cur_context = NULL;

	/* tng_topaz_mmu_flushcache(dev_priv); */

	for (n = 0; n < topaz_priv->topaz_num_pipes; n++) {
		PSB_DEBUG_TOPAZ("TOPAZ: Reset topaz registers for pipe %d",
			n);
		reg_val = F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_JMCOMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PREFETCH_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_VLC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_DB_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_LTRITC_SOFT_RESET);

		TOPAZCORE_WRITE32(n, TOPAZHP_CR_TOPAZHP_SRST, reg_val);

		TOPAZCORE_WRITE32(n, TOPAZHP_CR_TOPAZHP_SRST, 0);
	}

	tng_topaz_mmu_flushcache(dev_priv);

	PSB_DEBUG_TOPAZ("TOPAZ: Create fiwmware text/data storage");
	/* create firmware storage */
	for (n = 0; n < IMG_CODEC_NUM; ++n) {
		/* FIXME: 12*4096 wast mem? */
		ret = ttm_buffer_object_create(bdev, 12 * 4096,
			ttm_bo_type_kernel,
			DRM_PSB_FLAG_MEM_MMU | TTM_PL_FLAG_NO_EVICT,
			0, 0, 0, NULL, &topaz_priv->topaz_fw[n].text);
		if (ret) {
			DRM_ERROR("Failed to allocate memory " \
				"for firmware text section.\n");
			goto out;
		}

		ret = ttm_buffer_object_create(bdev, 12 * 4096,
			ttm_bo_type_kernel,
			DRM_PSB_FLAG_MEM_MMU | TTM_PL_FLAG_NO_EVICT,
			0, 0, 0, NULL, &topaz_priv->topaz_fw[n].data);
		if (ret) {
			DRM_ERROR("Failed to allocate memory " \
				"for firmware data section.\n");
			goto out;
		}
	}

	tng_set_producer(dev, 0);

	tng_set_consumer(dev, 0);


/* #define TEST_FW */
#ifdef TEST_FW
	ret = tng_topaz_init_fw(dev);
	if (ret) {
		DRM_ERROR("FW TESTING: Failed to load topazhp_fw.bin, " \
			"ensure udevd is configured correctly!\n");
			return ret;
	}

	ret = tng_topaz_setup_fw(dev, 0, 1);
	if (ret) {
		DRM_ERROR("FW TESTING: Setup H264_NO_RC firmware fails!\n");
		return ret;
	}
#endif
	return ret;

out:

	kfree(topaz_priv);
	dev_priv->topaz_private = NULL;

	for (n = 0; n < IMG_CODEC_NUM; ++n) {
		if (topaz_priv->topaz_fw[n].text)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].text);
		if (topaz_priv->topaz_fw[n].data)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].data);
	}

	ttm_bo_kunmap(&topaz_priv->topaz_bo_kmap);

	if (topaz_priv->topaz_bo)
		ttm_bo_unref(&topaz_priv->topaz_bo);

	return ret;
}

int tng_topaz_reset(struct drm_psb_private *dev_priv)
{
	struct tng_topaz_private *topaz_priv;
	uint32_t i;
	uint32_t reg_val;

	topaz_priv = dev_priv->topaz_private;
	topaz_priv->topaz_busy = 0;

	topaz_priv->topaz_needs_reset = 0;

	for (i = 0; i < topaz_priv->topaz_num_pipes; i++) {
		/* # reset topaz */
		PSB_DEBUG_TOPAZ("TOPAZ: reset pipe %d", i);
		reg_val = F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_JMCOMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PREFETCH_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_VLC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_DB_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_LTRITC_SOFT_RESET) ;

		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_SRST, reg_val);
		TOPAZCORE_READ32(i, TOPAZHP_CR_TOPAZHP_SRST, &reg_val);

		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_SRST, 0);
		TOPAZCORE_READ32(i, TOPAZHP_CR_TOPAZHP_SRST, &reg_val);
	}

	tng_topaz_mmu_hwsetup(dev_priv);

	return 0;
}

/* FIXME: When D0i3 enabled,
flush mmu and reset may have issue because hw power down */
int tng_topaz_uninit(struct drm_device *dev)
{
	int n;
	struct drm_psb_private *dev_priv;
	struct tng_topaz_private *topaz_priv;
	dev_priv = dev->dev_private;

	if (NULL == dev_priv) {
		PSB_DEBUG_TOPAZ("TOPAZ: dev_priv is NULL!\n");
		return -1;
	}

	topaz_priv = dev_priv->topaz_private;

	if (NULL == topaz_priv) {
		PSB_DEBUG_TOPAZ("TOPAZ: topaz_priv is NULL!\n");
		return -1;
	}

	tng_topaz_mmu_flushcache(dev_priv);

	tng_topaz_reset(dev_priv);

	PSB_DEBUG_TOPAZ("TOPAZ: Release fiwmware text/data storage");

	for (n = 0; n < IMG_CODEC_NUM; ++n) {
		if (topaz_priv->topaz_fw[n].text != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].text);

		if (topaz_priv->topaz_fw[n].data != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].data);
	}

	if (topaz_priv->topaz_bo) {
		PSB_DEBUG_TOPAZ("TOPAZ: Release fiwmware topaz_bo storage");
		ttm_bo_kunmap(&topaz_priv->topaz_bo_kmap);
		ttm_bo_unref(&topaz_priv->topaz_bo);
	}

	if (topaz_priv) {
		pci_set_drvdata(dev->pdev, NULL);
		device_remove_file(&dev->pdev->dev,
				   &dev_attr_topaz_pmstate);
		sysfs_put(topaz_priv->sysfs_pmstate);
		topaz_priv->sysfs_pmstate = NULL;

		kfree(topaz_priv);
		dev_priv->topaz_private = NULL;
	}

	return 0;
}

/* read firmware bin file and load all data into driver */
int tng_topaz_init_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	const struct firmware *raw = NULL;
	unsigned char *ptr;
	int n;
	struct tng_fwinfo *cur_fw;
	int cur_size, total_size;
	struct tng_topaz_codec_fw *cur_codec;
	struct ttm_buffer_object **cur_drm_obj;
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
#ifdef VERIFYFW_INIT
	uint32_t i, *p_buf;
#endif
	uint32_t ret = 0;
	/* # get firmware */
	ret = request_firmware(&raw, FIRMWARE_NAME, &dev->pdev->dev);
	if (ret) {
		DRM_ERROR("TOPAZ: Request firmware failed: %d\n", ret);
		return ret;
	}

	if ((NULL == raw) || (raw->size < sizeof(struct tng_fwinfo))) {
		DRM_ERROR("TOPAZ: Firmware file size is not correct.\n");
		ret = -1;
		goto out;
	}

	total_size = raw->size;
	PSB_DEBUG_TOPAZ("TOPAZ: Opened firmware, size %d\n", raw->size);

	ptr = (unsigned char *) raw->data;
	if (!ptr) {
		DRM_ERROR("TOPAZ: Failed to load firmware.\n");
		ret = -1;
		goto out;
	}

	/* # load fw from file */
	PSB_DEBUG_TOPAZ("TOPAZ: Load firmware......\n");
	cur_fw = NULL;
	for (n = 0; n < IMG_CODEC_NUM; ++n) {
		if (total_size < sizeof(struct tng_fwinfo)) {
			DRM_ERROR("TOPAZ: WARNING: Rearch end of" \
				"firmware. Have loaded %d firmwares.", n);
			break;
		}

		total_size -=  sizeof(struct tng_fwinfo);
		cur_fw = (struct tng_fwinfo *) ptr;

		cur_codec = &topaz_priv->topaz_fw[cur_fw->codec];
		cur_codec->ver = cur_fw->ver;
		cur_codec->codec = cur_fw->codec;
		cur_codec->text_size = cur_fw->text_size;
		cur_codec->data_size = cur_fw->data_size;
		cur_codec->data_location = cur_fw->data_location;

		total_size -= cur_fw->text_size;
		total_size -= cur_fw->data_size;

		if (total_size < 0) {
			DRM_ERROR("TOPAZ: WARNING: wrong size number " \
				"of data or text. Have loaded " \
				"%d firmwares.", n);
			break;
		}

		PSB_DEBUG_TOPAZ("TOPAZ: load firemware %s.\n",
				  codec_to_string(cur_fw->codec));

		/* handle text section */
		ptr += sizeof(struct tng_fwinfo);
		cur_drm_obj = &cur_codec->text;
		cur_size = cur_fw->text_size;

		/* fill DRM object with firmware data */
		ret = ttm_bo_kmap(*cur_drm_obj, 0, (*cur_drm_obj)->num_pages,
				  &tmp_kmap);
		if (ret) {
			DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
			ttm_bo_unref(cur_drm_obj);
			*cur_drm_obj = NULL;
			goto out;
		}

		PSB_DEBUG_TOPAZ("TOPAZ: load codec %d, text size: %d," \
			"data size %d, data location 08%x\n",
			cur_codec->codec, cur_codec->text_size,
			cur_codec->data_size, cur_codec->data_location);

		memcpy(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), ptr,
		       cur_size);
#ifdef VERIFYFW_INIT
		PSB_DEBUG_TOPAZ("TOPAZ: ###Firmware text section(FW)###\n");
		p_buf = (unsigned int *)ttm_kmap_obj_virtual(
				&tmp_kmap, &is_iomem);
		PSB_DEBUG_TOPAZ("TOPAZ: First 10(FW):\n");
		for (i = 0; i < 10; i++)
			PSB_DEBUG_TOPAZ("%08x ,", p_buf[i]);
#endif
		ttm_bo_kunmap(&tmp_kmap);

		/* handle data section */
		ptr += cur_fw->text_size;
		cur_drm_obj = &cur_codec->data;
		cur_size = cur_fw->data_size;

		/* fill DRM object with firmware data */
		ret = ttm_bo_kmap(*cur_drm_obj, 0, (*cur_drm_obj)->num_pages,
				  &tmp_kmap);
		if (ret) {
			DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
			ttm_bo_unref(cur_drm_obj);
			*cur_drm_obj = NULL;
			goto out;
		}

		memcpy(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), ptr,
		       cur_size);
#ifdef VERIFYFW_INIT
		PSB_DEBUG_TOPAZ("TOPAZ: ###Firmware data section(FW)###\n");
		p_buf = (unsigned int *)ttm_kmap_obj_virtual(
				&tmp_kmap, &is_iomem);
		PSB_DEBUG_TOPAZ("TOPAZ: First 10(FW):\n");
		for (i = 0; i < 10; i++)
			PSB_DEBUG_TOPAZ("%08x ,", p_buf[i]);
#endif
		ttm_bo_kunmap(&tmp_kmap);

		/* update ptr */
		ptr += cur_fw->data_size;
	}

	release_firmware(raw);
	PSB_DEBUG_TOPAZ("Return from firmware init\n");

	return ret;

out:
	if (raw) {
		DRM_ERROR("Error occues, release firmware....\n");
		release_firmware(raw);
	}

	return ret;
}

static void tng_set_auto_clk_gating(
	struct drm_device *dev,
	enum drm_tng_topaz_codec codec,
	uint32_t gating)
{
	uint32_t reg_val;
	struct drm_psb_private *dev_priv = dev->dev_private;
	PSB_DEBUG_TOPAZ("TOPAZ: Setting automatic clock gating on ");
	PSB_DEBUG_TOPAZ("codec %d to %d\n", codec, gating);

	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_WRITES_CORE_ALL);
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CORE_SEL_0,
			  reg_val);

	/* enable auto clock is essential for this driver */
	reg_val = F_ENCODE(1, TOPAZHP_CR_TOPAZHP_VLC_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_DEB_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE0_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE1_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE0_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE1_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP4X4_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP8X8_AUTO_CLK_GATE)|
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP16X16_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_JMCOMP_AUTO_CLK_GATE)|
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_DM_AUTO_CLK_GATE) |
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_DMS_AUTO_CLK_GATE)|
		  F_ENCODE(1, TOPAZHP_CR_TOPAZHP_CABAC_AUTO_CLK_GATE);

	TOPAZCORE_WRITE32(0, TOPAZHP_CR_TOPAZHP_AUTO_CLOCK_GATING, reg_val);

	if (codec != IMG_CODEC_JPEG) {
		reg_val = 0;
		TOPAZCORE_READ32(0, TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING,
				 &reg_val);

		/* Disable LRITC clocks */
		reg_val = F_INSERT(reg_val, 1,
			TOPAZHP_CR_TOPAZHP_LRITC_MAN_CLK_GATE);

#if  (defined(ENABLE_PREFETCH_GATING))
		/* Disable PREFETCH clocks */
		reg_val = F_INSERT(reg_val , 1,
			TOPAZHP_CR_TOPAZHP_PREFETCH_MAN_CLK_GATE);
#endif
		TOPAZCORE_WRITE32(0,
			TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING, reg_val);
	}

	reg_val = F_ENCODE(0, TOPAZHP_TOP_CR_WRITES_CORE_ALL);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CORE_SEL_0, reg_val);
}

static void tng_get_bank_size(
	struct drm_device *dev,
	struct psb_video_ctx *video_ctx,
	enum drm_tng_topaz_codec codec)
{
	uint32_t last_bank_ram_size;
	struct drm_psb_private *dev_priv;
	struct tng_topaz_private *topaz_priv;

	dev_priv = dev->dev_private;
	topaz_priv = dev_priv->topaz_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MTX_DEBUG_MSTR,
			 &video_ctx->mtx_debug_val);

	last_bank_ram_size = 0x1 << (F_EXTRACT(video_ctx->mtx_debug_val,
			TOPAZHP_TOP_CR_MTX_MSTR_LAST_RAM_BANK_SIZE) + 2);

	video_ctx->mtx_bank_size = 0x1 << (F_EXTRACT(video_ctx->mtx_debug_val,
			TOPAZHP_TOP_CR_MTX_MSTR_RAM_BANK_SIZE) + 2);

	video_ctx->mtx_ram_size = last_bank_ram_size +
			(video_ctx->mtx_bank_size *
			(F_EXTRACT(video_ctx->mtx_debug_val,
			TOPAZHP_TOP_CR_MTX_MSTR_RAM_BANKS) - 1));

	PSB_DEBUG_TOPAZ("TOPAZ: Last bank ram size: %08x," \
			"mtx bank size: %08x, mtx ram size: %08x\n",
			last_bank_ram_size,
			video_ctx->mtx_bank_size, video_ctx->mtx_ram_size);

	video_ctx->fw_data_dma_offset =
		topaz_priv->topaz_fw[codec].data_location &
		~(MTX_DMA_BURSTSIZE_BYTES - 1);

	video_ctx->fw_data_dma_size = video_ctx->mtx_ram_size -
			(video_ctx->fw_data_dma_offset - MTX_DMA_MEMORY_BASE);

	video_ctx->fw_data_dma_size /= 4;

	PSB_DEBUG_TOPAZ("TOPAZ: Save/Restore MTX at RAM address %08x of" \
		"size: %08x bytes for codec %s on ctx %08x\n",
		video_ctx->fw_data_dma_offset, video_ctx->fw_data_dma_size,
		codec_to_string(codec), (unsigned int)(video_ctx->filp));
}

/* setup fw when start a new context */
int tng_topaz_init_board(
	struct drm_device *dev,
	struct psb_video_ctx *video_ctx,
	enum drm_tng_topaz_codec codec)
{
	struct drm_psb_private *dev_priv;
	struct tng_topaz_private *topaz_priv;
	uint32_t verify_pc;
	int32_t i;
	int32_t ret = 0;
	uint32_t reg_val = 0;

	dev_priv = dev->dev_private;
	topaz_priv = dev_priv->topaz_private;

	/*psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);*/

	PSB_DEBUG_GENERAL("TOPAZ: will setup firmware ....\n");

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG, &reg_val);
	PSB_DEBUG_GENERAL("TOPAZ: HW_CFG 0x%08x\n", reg_val);

	if (topaz_priv->topaz_num_pipes > TOPAZHP_PIPE_NUM) {
		DRM_ERROR("TOPAZ: Number of pipes: 0x%x\n",
			topaz_priv->topaz_num_pipes);
		topaz_priv->topaz_num_pipes = TOPAZHP_PIPE_NUM;
	}

	tng_topaz_mmu_flushcache(dev_priv);

	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_IO_SOFT_RESET);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, reg_val);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 0x0);

	/* tng_topaz_mmu_flushcache(dev_priv); */

	tng_topaz_mmu_hwsetup(dev_priv);

	for (i = 0; i < topaz_priv->topaz_num_pipes; i++) {
		PSB_DEBUG_GENERAL("TOPAZ: Reset topaz registers for pipe %d",
			i);
		reg_val = F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_JMCOMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PREFETCH_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_VLC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_DB_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_LTRITC_SOFT_RESET);

		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_SRST, reg_val);

		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_SRST, 0);
	}

	tng_topaz_mmu_flushcache(dev_priv);
	tng_set_producer(dev, 0);
	tng_set_consumer(dev, 0);
	return 0;
}

int tng_topaz_setup_fw(
	struct drm_device *dev,
	struct psb_video_ctx *video_ctx,
	enum drm_tng_topaz_codec codec)
{
	struct drm_psb_private *dev_priv;
	struct tng_topaz_private *topaz_priv;
	uint32_t verify_pc;
	int32_t i;
	int32_t ret = 0;
	uint32_t reg_val = 0;

	dev_priv = dev->dev_private;
	topaz_priv = dev_priv->topaz_private;

	/*psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);*/

	PSB_DEBUG_TOPAZ("TOPAZ: will setup firmware ....\n");

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG, &reg_val);
	PSB_DEBUG_TOPAZ("TOPAZ: HW_CFG 0x%08x\n", reg_val);

	if (topaz_priv->topaz_num_pipes > TOPAZHP_PIPE_NUM) {
		DRM_ERROR("TOPAZ: Number of pipes: 0x%x\n",
			topaz_priv->topaz_num_pipes);
		topaz_priv->topaz_num_pipes = TOPAZHP_PIPE_NUM;
	}

	/* Soft reset of MTX */
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		  F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, reg_val);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 0x0);

	tng_get_bank_size(dev, video_ctx, codec);

	tng_set_auto_clk_gating(dev, codec, 1);

	PSB_DEBUG_TOPAZ("TOPAZ: will upload firmware to %d pipes\n",
			  topaz_priv->topaz_num_pipes);

	ret = mtx_upload_fw(dev, codec, video_ctx, 0);
	if (ret) {
		DRM_ERROR("Failed to upload firmware for codec %s\n",
				codec_to_string(codec));
		/*tng_error_dump_reg(dev_priv);*/
		return ret;
	}

	/* D0.5, D0.6 and D0.7 */
	for (i = 5; i < 8; i++)  {
		ret = mtx_write_core_reg(dev_priv, 0x1 | (i << 4), 0);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			return ret;
		}
	}

	/* Restore 8 Registers of D1 Bank, D1Re0,
	D1Ar5, D1Ar3, D1Ar1, D1RtP, D1.5, D1.6 and D1.7 */
	for (i = 5; i < 8; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x2 | (i << 4), 0);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			return ret;
		}
	}

	PSB_DEBUG_TOPAZ("TOPAZ: setting up pc address 0x%08x\n",
		PC_START_ADDRESS);

	/* Set Starting PC address */
	ret = mtx_write_core_reg(dev_priv, MTX_PC, PC_START_ADDRESS);
	if (ret) {
		DRM_ERROR("Failed to write core reg");
		return ret;
	}

	ret = mtx_read_core_reg(dev_priv, MTX_PC, &verify_pc);
	if (ret) {
		DRM_ERROR("Failed to read core reg");
		return ret;
	}

	if (verify_pc != PC_START_ADDRESS) {
		DRM_ERROR("TOPAZ: Wrong starting PC address");
		return -1;
	} else {
		PSB_DEBUG_TOPAZ("TOPAZ: verify pc address = 0x%08x\n",
				  verify_pc);
	}

	/* flush the command FIFO - only has effect on master MTX */
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_CMD_FIFO_FLUSH);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_TOPAZ_CMD_FIFO_FLUSH, reg_val);

	MULTICORE_WRITE32(MTX_SCRATCHREG_IDLE, 0);

	/* turn on MTX */
	mtx_start(dev);
	mtx_kick(dev);

	/* While we aren't using comm serialization, we do still need to ensure
	 * that the firmware has completed it's setup before continuing
	 */
	PSB_DEBUG_TOPAZ("TOPAZ: Polling to ensure " \
		"firmware has completed its setup before continuing\n");
	ret = tng_topaz_wait_for_register(
		dev_priv, CHECKFUNC_ISEQUAL,
		TOPAZHP_TOP_CR_FIRMWARE_REG_1 + (MTX_SCRATCHREG_TOHOST << 2),
		0x00000ED5, 0x00000ED5);
	if (ret) {
		DRM_ERROR("Firmware failed to complete its setup" \
			"before continuing\n");
		return ret;
	}

	MULTICORE_WRITE32(
		TOPAZHP_TOP_CR_FIRMWARE_REG_1 + (MTX_SCRATCHREG_TOHOST << 2),
		0);

	PSB_DEBUG_TOPAZ("TOPAZ: Firmware uploaded successfully.\n");

	/* tng_topaz_enableirq(dev); */
#ifdef TOPAZHP_IRQ_ENABLED
	psb_irq_preinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
	psb_irq_postinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);

	tng_topaz_enableirq(dev);
#endif

	return ret;
}

int mtx_upload_fw(struct drm_device *dev,
		  enum drm_tng_topaz_codec codec,
		  struct psb_video_ctx *video_ctx,
		  uint32_t is_restore)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	const struct tng_topaz_codec_fw *cur_codec_fw;
	uint32_t text_size, data_size;
	uint32_t data_location;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int ret = 0;

	if (codec >= IMG_CODEC_NUM) {
		DRM_ERROR("TOPAZ: Invalid codec %d\n", codec);
		return -1;
	}

	if (!is_restore) {
		/* set target to current or all MTXs */
		mtx_set_target(dev_priv);

		/* MTX reset */
		MTX_WRITE32(MTX_CR_MTX_SOFT_RESET,
			    MASK_MTX_MTX_RESET);
	}

	/* upload the master and slave firmware by DMA */
	cur_codec_fw = &topaz_priv->topaz_fw[codec];

	PSB_DEBUG_TOPAZ("TOPAZ: Upload firmware, text_location = %08x," \
		"text_size = %d, data_location = %08x, data_size = %d\n",
		MTX_DMA_MEMORY_BASE, cur_codec_fw->text_size,
		cur_codec_fw->data_location, cur_codec_fw->data_size);

	/* upload text. text_size is byte size*/
	text_size = cur_codec_fw->text_size / 4;
	/* adjust transfer sizes of text and data sections to match burst size*/
	text_size = ((text_size * 4 + (MTX_DMA_BURSTSIZE_BYTES - 1)) &
		    ~(MTX_DMA_BURSTSIZE_BYTES - 1)) / 4;

	PSB_DEBUG_TOPAZ("TOPAZ: Text size round up to %d\n", text_size);
	/* setup the MTX to start recieving data:
	   use a register for the transfer which will point to the source
	   (MTX_CR_MTX_SYSC_CDMAT) */
	/*MTX burst size (4 * 2 * 32bits = 32bytes) should match DMA burst
	  size (2 * 128bits = 32bytes) */
	/* #.# fill the dst addr */

	/* Transfer the text section */

	PSB_DEBUG_TOPAZ("TOPAZ: Text offset %08x\n",
		(unsigned int)cur_codec_fw->text->offset);

	ret = mtx_dmac_transfer(dev_priv, 0, cur_codec_fw->text->offset, 0,
		MTX_DMA_MEMORY_BASE, text_size, 1);
	if (ret) {
		DRM_ERROR("Failed to transfer text by DMA\n");
		/* tng_error_dump_reg(dev_priv); */
		return ret;
	}

	/* wait for it to finish */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
			REG_OFFSET_TOPAZ_DMAC + IMG_SOC_DMAC_IRQ_STAT(0),
			F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
			F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	if (ret) {
		DRM_ERROR("Transfer text by DMA timeout\n");
		/* tng_error_dump_reg(dev_priv); */
		return ret;
	}

	/* clear the interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	PSB_DEBUG_TOPAZ("TOPAZ: Firmware text upload complete.\n");

	/* # upload data */
	data_size = cur_codec_fw->data_size / 4;
	data_size = ((data_size * 4 + (MTX_DMA_BURSTSIZE_BYTES - 1)) &
			~(MTX_DMA_BURSTSIZE_BYTES - 1)) / 4;

	data_location = cur_codec_fw->data_location;

	/* transfer the codec */
	/* Context restore has different DMA size and offset */
	if (is_restore) {
		ret = mtx_dmac_transfer(dev_priv, 0,
			video_ctx->data_saving_bo->offset, 0,
			video_ctx->fw_data_dma_offset,
			video_ctx->fw_data_dma_size, 1);
	} else {
		ret = mtx_dmac_transfer(dev_priv, 0,
			cur_codec_fw->data->offset,
			0, /*offset + 0 = source address*/
			data_location, data_size, 1);
	}

	if (ret) {
		DRM_ERROR("Failed to transfer data by DMA\n");
		/* tng_error_dump_reg(dev_priv); */
		return ret;
	}

	/* wait for it to finish */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
			REG_OFFSET_TOPAZ_DMAC + IMG_SOC_DMAC_IRQ_STAT(0),
			F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
			F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	if (ret) {
		DRM_ERROR("Transfer data by DMA timeout\n");
		/* tng_error_dump_reg(dev_priv); */
		return ret;
	}

	/* clear the interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	tng_topaz_mmu_flushcache(dev_priv);

	PSB_DEBUG_TOPAZ("TOPAZ: Firmware data upload complete.\n");

	return 0;
}

int mtx_dmac_transfer(struct drm_psb_private *dev_priv, uint32_t channel,
		     uint32_t src_phy_addr, uint32_t offset,
		     uint32_t mtx_addr, uint32_t byte_num,
		     uint32_t is_write)
{
	uint32_t dmac_count;
	uint32_t irq_stat;
	uint32_t count;

	PSB_DEBUG_TOPAZ("TOPAZ: using dma to transfer firmware\n");

	/* check that no transfer is currently in progress */
	DMAC_READ32(IMG_SOC_DMAC_COUNT(channel), &dmac_count);
	if (0 != (dmac_count & (MASK_IMG_SOC_EN | MASK_IMG_SOC_LIST_EN))) {
		DRM_ERROR("TOPAZ: there is tranfer in progress (0x%08x)\n",
			dmac_count);
		return -1;
	}

	/* clear status of any previous interrupts */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(channel), 0);

	/* and that no interrupts are outstanding */
	DMAC_READ32(IMG_SOC_DMAC_IRQ_STAT(channel), &irq_stat);
	if (0 != irq_stat) {
		DRM_ERROR("TOPAZ: there is hold up\n");
		return -1;
	}

	/* Transfer the data section */
	/* MTX Address */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA, mtx_addr);

	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(4, MTX_BURSTSIZE) |
		    F_ENCODE(is_write ? 0 : 1, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) |
		    F_ENCODE(byte_num, MTX_LENGTH));

	/* Write System DMAC registers */
	/* per hold - allow HW to sort itself out */
	DMAC_WRITE32(IMG_SOC_DMAC_PER_HOLD(channel), 16);

	/* clear previous interrupts */
	/* DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(channel), 0);*/

	/* Set start address */
	DMAC_WRITE32(IMG_SOC_DMAC_SETUP(channel), (src_phy_addr + offset));

	/* count reg */
	count = DMAC_VALUE_COUNT(DMAC_BSWAP_NO_SWAP, DMAC_PWIDTH_32_BIT,
			(is_write ? 0 : 1), DMAC_PWIDTH_32_BIT, byte_num);
	/* generate an interrupt at the end of transfer */
	count |= MASK_IMG_SOC_TRANSFER_IEN;

	/*count |= F_ENCODE(is_write, IMG_SOC_DIR);*/
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel), count);

	/* don't inc address, set burst size */
	DMAC_WRITE32(IMG_SOC_DMAC_PERIPH(channel),
		DMAC_VALUE_PERIPH_PARAM(DMAC_ACC_DEL_0, 0, DMAC_BURST_2));

	/* Target correct MTX DMAC port */
	DMAC_WRITE32(IMG_SOC_DMAC_PERIPHERAL_ADDR(channel),
		MTX_CR_MTX_SYSC_CDMAT + REG_START_TOPAZ_MTX_HOST);

	/* Finally, rewrite the count register
	with the enable bit set to kick off the transfer */
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel),
		count | MASK_IMG_SOC_EN);

	/* DMAC_READ32(IMG_SOC_DMAC_COUNT(channel), &tmp);*/

	PSB_DEBUG_TOPAZ("TOPAZ: dma transfer started.\n");

	return 0;
}

int32_t mtx_write_core_reg(
	struct drm_psb_private *dev_priv,
	uint32_t reg,
	const uint32_t val)
{
	int32_t ret = 0;
	ret = get_mtx_control_from_dash(dev_priv);
	if (ret) {
		DRM_ERROR("Failed to get control from dash");
		return ret;
	}

	/* put data into MTX_RW_DATA */
	MTX_WRITE32(MTX_CR_MTX_REGISTER_READ_WRITE_DATA, val);

	/* DREADY is set to 0 and request a write*/
	MTX_WRITE32(MTX_CR_MTX_REGISTER_READ_WRITE_REQUEST,
		reg & ~MASK_MTX_MTX_DREADY);

	/* wait for operation finished */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_MTX + MTX_CR_MTX_REGISTER_READ_WRITE_REQUEST,
		MASK_MTX_MTX_DREADY, MASK_MTX_MTX_DREADY);
	if (ret) {
		DRM_ERROR("Wait for register timeout");
		return ret;
	}

	release_mtx_control_from_dash(dev_priv);

	return ret;
}

int32_t mtx_read_core_reg(
	struct drm_psb_private *dev_priv,
	uint32_t reg,
	uint32_t *ret_val)
{
	int32_t ret = 0;

	ret = get_mtx_control_from_dash(dev_priv);
	if (ret) {
		DRM_ERROR("Failed to get control from dash");
		return ret;
	}

	/* Issue read request */
	MTX_WRITE32(MTX_CR_MTX_REGISTER_READ_WRITE_REQUEST,
		    MASK_MTX_MTX_RNW | reg & ~MASK_MTX_MTX_DREADY);

	/* Wait for done */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_MTX + MTX_CR_MTX_REGISTER_READ_WRITE_REQUEST,
		MASK_MTX_MTX_DREADY, MASK_MTX_MTX_DREADY);
	if (ret) {
		DRM_ERROR("Wait for register timeout");
		return ret;
	}

	/* Read */
	MTX_READ32(MTX_CR_MTX_REGISTER_READ_WRITE_DATA, ret_val);

	release_mtx_control_from_dash(dev_priv);

	return ret;
}

static int32_t get_mtx_control_from_dash(struct drm_psb_private *dev_priv)
{
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t count = 0;
	uint32_t reg_val = 0;

	/* Request the bus from the Dash...*/
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_MTX_MSTR_DBG_IS_SLAVE) |
		  F_ENCODE(0x2, TOPAZHP_TOP_CR_MTX_MSTR_DBG_GPIO_IN);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MTX_DEBUG_MSTR, reg_val);

	do {
		MULTICORE_READ32(TOPAZHP_TOP_CR_MTX_DEBUG_MSTR, &reg_val);
		count++;
	} while (((reg_val & 0x18) != 0) && count < 50000);

	if (count >= 50000) {
		DRM_ERROR("TOPAZ: timeout in getting control from dash");
		return -1;
	}

	/* Save the access control register...*/
	MTX_READ32(MTX_CR_MTX_RAM_ACCESS_CONTROL,
		&topaz_priv->topaz_dash_access_ctrl);

	return 0;
}

static void release_mtx_control_from_dash(struct drm_psb_private *dev_priv)
{
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t reg_val;

	/* Restore the access control register...*/
	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_CONTROL,
		topaz_priv->topaz_dash_access_ctrl);

	/* Release the bus...*/
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_MTX_MSTR_DBG_IS_SLAVE);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MTX_DEBUG_MSTR, reg_val);
}

void tng_topaz_mmu_hwsetup(struct drm_psb_private *dev_priv)
{
	uint32_t reg_val = 0;
	uint32_t pd_addr = 0;

	PSB_DEBUG_TOPAZ("TOPAZ: Setup MMU\n");

	/* pd_addr = TOPAZHP_MMU_BASE; */
	pd_addr = psb_get_default_pd_addr(dev_priv->mmu);
	/* bypass all request while MMU is being configured */
	reg_val = F_ENCODE(1, TOPAZHP_TOP_CR_MMU_BYPASS_TOPAZ);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL0, reg_val);

	/* set MMU hardware at the page table directory */
	PSB_DEBUG_TOPAZ("TOPAZ: write PD phyaddr=0x%08x " \
		"into MMU_DIR_LIST0/1\n", pd_addr);

	/*There's two of these (0) and (1).. only 0 is currently used*/
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_DIR_LIST_BASE(0), pd_addr);
	/* setup index register, all pointing to directory bank 0 */

	/* now enable MMU access for all requestors */
	reg_val = F_ENCODE(0, TOPAZHP_TOP_CR_MMU_BYPASS_TOPAZ);
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL0, reg_val);

	/* This register does not get reset between encoder runs,
	so we need to ensure we always set it up one way or another here */
	/* 36-bit actually means "not 32-bit" */
	reg_val = F_ENCODE(0, TOPAZHP_TOP_CR_MMU_ENABLE_36BIT_ADDRESSING);
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL2, reg_val);
}

void tng_topaz_mmu_flushcache(struct drm_psb_private *dev_priv)
{
	uint32_t mmu_control;

	if (dev_priv->topaz_disabled) {
		printk(KERN_ERR "topazhp disabled\n");
		return;
	}

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_CONTROL0, &mmu_control);

	/* Set Invalid flag (this causes a flush with MMU still
	operating afterwards even if not cleared,
	but may want to replace with MMU_FLUSH? */
	mmu_control |= F_ENCODE(1, TOPAZHP_TOP_CR_MMU_INVALDC);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL0, mmu_control);

	/* Clear Invalid flag */
	mmu_control |= F_ENCODE(0, TOPAZHP_TOP_CR_MMU_INVALDC);
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL0, mmu_control);
	/*
	psb_gl3_global_invalidation(dev_priv->dev);
	*/
}

int32_t mtx_dma_read(struct drm_device *dev, struct ttm_buffer_object *dst_bo,
		 uint32_t src_ram_addr, uint32_t size)
{
	uint32_t irq_state;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct ttm_buffer_object *target;
	int32_t ret = 0;

	/* clear status of any previous interrupts */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);
	DMAC_READ32(IMG_SOC_DMAC_IRQ_STAT(0), &irq_state);
	if (irq_state != 0) {
		DRM_ERROR("irq state is not 0");
		return -1;
	}

	target = dst_bo;
	/* transfer the data */
	ret = mtx_dmac_transfer(dev_priv, 0, (uint32_t)target->offset, 0,
			       src_ram_addr,
			       size, 0);
	if (ret) {
		/* tng_error_dump_reg(dev_priv); */
		DRM_ERROR("DMA transfer failed");
		return ret;
	}

	/* wait for it transfer */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_DMAC + IMG_SOC_DMAC_IRQ_STAT(0),
		F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
		F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	if (ret) {
		/* tng_error_dump_reg(dev_priv); */
		DRM_ERROR("Waiting register timeout");
		return ret;
	}

	/* clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	return ret;
}

static int mtx_dma_write(
	struct drm_device *dev,
	struct ttm_buffer_object *src_bo,
	uint32_t dst_ram_addr, uint32_t size)
{
	uint32_t irq_state;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct ttm_buffer_object *target;
	int32_t ret = 0;

	/* clear status of any previous interrupts */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);
	DMAC_READ32(IMG_SOC_DMAC_IRQ_STAT(0), &irq_state);
	if (irq_state != 0) {
		DRM_ERROR("irq state is not 0");
		return -1;
	}

	/* give the DMAC access to the host memory via BIF */
	/*FPGA_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1, 0);*/

	target = src_bo;
	/* transfert the data */
	ret = mtx_dmac_transfer(dev_priv, 0, target->offset, 0,
			       dst_ram_addr,
			       size, 1);


	if (ret) {
		/* tng_error_dump_reg(dev_priv); */
		DRM_ERROR("DMA transfer failed");
		return ret;
	}

	/* wait for it transfer */
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_DMAC + IMG_SOC_DMAC_IRQ_STAT(0),
		F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
		F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	if (ret) {
		/* tng_error_dump_reg(dev_priv); */
		DRM_ERROR("Waiting register timeout");
		return ret;
	}

	/* clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	return ret;
}
