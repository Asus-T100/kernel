/**************************************************************************
 * psb_msvdxinit.c
 * MSVDX initialization and mtx-firmware upload
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
 *    Binglin Chen <binglin.chen@intel.com>
 *    Fei Jiang <fei.jiang@intel.com>
 *
 **************************************************************************/

#include <drm/drmP.h>
#include <drm/drm.h>
#ifdef CONFIG_DRM_VXD_BYT
#include "vxd_drv.h"
#else
#include "psb_drv.h"
#endif

#include "psb_msvdx.h"
#include "psb_msvdx_msg.h"
#include "psb_msvdx_reg.h"
#ifdef CONFIG_VIDEO_MRFLD
#include "psb_msvdx_ec.h"
#endif
#include <linux/firmware.h>

uint8_t psb_rev_id;

/*
 * the original 1000 of udelay is derive from reference driver
 * From Liu, Haiyang, changed the originial udelay value from 1000 to 5
 * can save 3% C0 residence
 */
int psb_wait_for_register(struct drm_psb_private *dev_priv,
			  uint32_t offset, uint32_t value, uint32_t enable,
			  uint32_t poll_cnt, uint32_t timeout)
{
	uint32_t reg_value = 0;
	/* uint32_t poll_cnt = 2000000; */
	while (poll_cnt) {
		reg_value = PSB_RMSVDX32(offset);
		if (value == (reg_value & enable))
			return 0;

		/* Wait a bit */
		PSB_UDELAY(timeout);
		/* PSB_UDELAY(5); */
		poll_cnt--;
	}
	PSB_DEBUG_MSVDX("MSVDX: Timeout while waiting for register %08x:"
		  " expecting %08x (mask %08x), got %08x\n",
		  offset, value, enable, reg_value);

	return 1;
}

#if 0
int psb_poll_mtx_irq(struct drm_psb_private *dev_priv)
{
	int ret = 0;
	uint32_t mtx_int = 0;

	REGIO_WRITE_FIELD_LITE(mtx_int, MSVDX_INTERRUPT_STATUS, MTX_IRQ,
			       1);

	ret = psb_wait_for_register(dev_priv, MSVDX_INTERRUPT_STATUS_OFFSET,
				    /* Required value */
				    mtx_int,
				    /* Enabled bits */
				    mtx_int, 2000000, 5);

	if (ret) {
		DRM_ERROR("MSVDX: Error Mtx did not return"
			  " int within a resonable time\n");
		return ret;
	}

	PSB_DEBUG_IRQ("MSVDX: Got MTX Int\n");

	/* Got it so clear the bit */
	PSB_WMSVDX32(mtx_int, MSVDX_INTERRUPT_CLEAR_OFFSET);

	return ret;
}
#endif

static void msvdx_free_ccb(struct ttm_buffer_object **ccb)
{
	ttm_bo_unref(ccb);
	*ccb = NULL;
}

int psb_msvdx_core_reset(struct drm_psb_private *dev_priv)
{
	int ret = 0;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t core_rev;
	/* Enable Clocks */
	PSB_DEBUG_GENERAL("Enabling clocks\n");
	psb_msvdx_mtx_set_clocks(dev_priv->dev, clk_enable_all);

	/* Always pause the MMU as the core may be still active
	 * when resetting.  It is very bad to have memory
	 * activity at the same time as a reset - Very Very bad
	 */
	PSB_WMSVDX32(2, MSVDX_MMU_CONTROL0_OFFSET);

	/* BRN26106, BRN23944, BRN33671 */
	/* This is neccessary for all cores up to Tourmaline */
	if ((PSB_RMSVDX32(MSVDX_CORE_REV_OFFSET) < 0x00050502) &&
		(PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET)
			& MSVDX_INTERRUPT_STATUS__MMU_FAULT_IRQ_MASK) &&
		(PSB_RMSVDX32(MSVDX_MMU_STATUS_OFFSET) & 1)) {
		unsigned int *pptd;
		unsigned int loop;
		uint32_t ptd_addr;

		/* do work around */
		ptd_addr = page_to_pfn(msvdx_priv->mmu_recover_page)
					<< PAGE_SHIFT;
		pptd = kmap(msvdx_priv->mmu_recover_page);
		if (!pptd) {
			DRM_ERROR("failed to kmap mmu recover page.\n");
			return -1;
		}
		for (loop = 0; loop < 1024; loop++)
			pptd[loop] = ptd_addr | 0x00000003;
		PSB_WMSVDX32(ptd_addr, MSVDX_MMU_DIR_LIST_BASE_OFFSET +  0);
		PSB_WMSVDX32(ptd_addr, MSVDX_MMU_DIR_LIST_BASE_OFFSET +  4);
		PSB_WMSVDX32(ptd_addr, MSVDX_MMU_DIR_LIST_BASE_OFFSET +  8);
		PSB_WMSVDX32(ptd_addr, MSVDX_MMU_DIR_LIST_BASE_OFFSET + 12);

		PSB_WMSVDX32(6, MSVDX_MMU_CONTROL0_OFFSET);
		PSB_WMSVDX32(MSVDX_INTERRUPT_STATUS__MMU_FAULT_IRQ_MASK,
					MSVDX_INTERRUPT_STATUS_OFFSET);
		kunmap(msvdx_priv->mmu_recover_page);
	}

	/* make sure *ALL* outstanding reads have gone away */
	{
		int loop;
		uint32_t cmd;
		for (loop = 0; loop < 10; loop++)
			ret = psb_wait_for_register(dev_priv,
						MSVDX_MMU_MEM_REQ_OFFSET,
						0, 0xff, 100, 1);
		if (ret) {
			PSB_DEBUG_WARN("MSVDX_MMU_MEM_REQ is %d,\n"
				"indicate outstanding read request 0.\n",
				PSB_RMSVDX32(MSVDX_MMU_MEM_REQ_OFFSET));
			ret = -1;
			return ret;
		}
		/* disconnect RENDEC decoders from memory */
		cmd = PSB_RMSVDX32(MSVDX_RENDEC_CONTROL1_OFFSET);
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1, RENDEC_DEC_DISABLE, 1);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL1_OFFSET);

		/* Issue software reset for all but core */
		PSB_WMSVDX32((unsigned int)~MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL_OFFSET);
		PSB_RMSVDX32(MSVDX_CONTROL_OFFSET);
		/* bit format is set as little endian */
		PSB_WMSVDX32(0, MSVDX_CONTROL_OFFSET);
		/* make sure read requests are zero */
		ret = psb_wait_for_register(dev_priv, MSVDX_MMU_MEM_REQ_OFFSET,
						0, 0xff, 100, 100);
		if (!ret) {
			/* Issue software reset */
			PSB_WMSVDX32(MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL_OFFSET);

			ret = psb_wait_for_register(dev_priv, MSVDX_CONTROL_OFFSET, 0,
					MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK,
					2000000, 5);
			if (!ret) {
				/* Clear interrupt enabled flag */
				PSB_WMSVDX32(0, MSVDX_HOST_INTERRUPT_ENABLE_OFFSET);

				/* Clear any pending interrupt flags */
				PSB_WMSVDX32(0xFFFFFFFF, MSVDX_INTERRUPT_CLEAR_OFFSET);
			} else {
				PSB_DEBUG_WARN("MSVDX_CONTROL_OFFSET is %d,\n"
					"indicate software reset failed.\n",
					PSB_RMSVDX32(MSVDX_CONTROL_OFFSET));
			}
		} else {
			PSB_DEBUG_WARN("MSVDX_MMU_MEM_REQ is %d,\n"
				"indicate outstanding read request 1.\n",
				PSB_RMSVDX32(MSVDX_MMU_MEM_REQ_OFFSET));
		}
	}
	return ret;
}

/**
 * Reset chip and disable interrupts.
 * Return 0 success, 1 failure
 */
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
int psb_msvdx_reset(struct drm_psb_private *dev_priv)
{
	int ret = 0;

	/* Issue software reset */
	/* PSB_WMSVDX32(msvdx_sw_reset_all, MSVDX_CONTROL); */
	PSB_WMSVDX32(MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL_OFFSET);

	ret = psb_wait_for_register(dev_priv, MSVDX_CONTROL_OFFSET, 0,
			MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, 2000000, 5);
	if (!ret) {
		/* Clear interrupt enabled flag */
		PSB_WMSVDX32(0, MSVDX_HOST_INTERRUPT_ENABLE_OFFSET);

		/* Clear any pending interrupt flags */
		PSB_WMSVDX32(0xFFFFFFFF, MSVDX_INTERRUPT_CLEAR_OFFSET);
	} else {
		PSB_DEBUG_WARN("MSVDX_CONTROL_OFFSET is %d,\n"
			"indicate software reset failed.\n",
			PSB_RMSVDX32(MSVDX_CONTROL_OFFSET));
	}

	return ret;
}
#endif

static int msvdx_allocate_ccb(struct drm_device *dev,
			    struct ttm_buffer_object **ccb,
			    uint32_t *base_addr, unsigned long size)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	int ret;
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;

	PSB_DEBUG_INIT("MSVDX: allocate CCB\n");

	ret = ttm_buffer_object_create(bdev, size,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT, 0, 0, 0,
				       NULL, ccb);
	if (ret) {
		DRM_ERROR("MSVDX:failed to allocate CCB.\n");
		*ccb = NULL;
		return 1;
	}

	ret = ttm_bo_kmap(*ccb, 0, (*ccb)->num_pages, &tmp_kmap);
	if (ret) {
		DRM_ERROR("ttm_bo_kmap failed ret: %d\n", ret);
		ttm_bo_unref(ccb);
		*ccb = NULL;
		return 1;
	}

	memset(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), 0,
	       size);
	ttm_bo_kunmap(&tmp_kmap);

	*base_addr = (*ccb)->offset;
	return 0;
}

static ssize_t psb_msvdx_pmstate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct drm_psb_private *dev_priv = NULL;
	struct msvdx_private *msvdx_priv = NULL;
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	dev_priv = drm_dev->dev_private;
	msvdx_priv = dev_priv->msvdx_private;
#ifndef CONFIG_DRM_VXD_BYT
	ret = snprintf(buf, 64, "MSVDX Power state 0x%s, gating count 0x%08x\n",
		       ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND)
				? "ON" : "OFF", msvdx_priv->pm_gating_count);
#endif
	return ret;
}

static DEVICE_ATTR(msvdx_pmstate, 0444, psb_msvdx_pmstate_show, NULL);

static int32_t msvdx_alloc_ccb_for_rendec(struct drm_device *dev)
{
	struct msvdx_private *msvdx_priv = NULL;
	int32_t ret = 0;

	PSB_DEBUG_INIT("MSVDX: Setting up RENDEC,allocate CCB 0/1\n");

	if (dev == NULL)
		return 1;

	struct drm_psb_private *dev_priv = psb_priv(dev);
	if (dev_priv == NULL)
		return 1;

	msvdx_priv = dev_priv->msvdx_private;
	if (msvdx_priv == NULL)
		return 1;

	/* Allocate device virtual memory as required by rendec.... */
	if (!msvdx_priv->ccb0) {
		ret = msvdx_allocate_ccb(dev, &msvdx_priv->ccb0,
				       &msvdx_priv->base_addr0,
				       RENDEC_A_SIZE);
		if (ret) {
			DRM_ERROR("Allocate Rendec A fail.\n");
			goto err_exit;
		}
	}

	if (!msvdx_priv->ccb1) {
		ret = msvdx_allocate_ccb(dev, &msvdx_priv->ccb1,
				       &msvdx_priv->base_addr1,
				       RENDEC_B_SIZE);
		if (ret) {
			DRM_ERROR("Allocate Rendec B fail.\n");
			goto err_exit;
		}
	}

	PSB_DEBUG_INIT("MSVDX: RENDEC A: %08x RENDEC B: %08x\n",
			  msvdx_priv->base_addr0, msvdx_priv->base_addr1);

	return 0;

err_exit:
	DRM_ERROR("MSVDX: %s failed.\n", __func__);
	if (msvdx_priv->ccb0)
		msvdx_free_ccb(&msvdx_priv->ccb0);
	if (msvdx_priv->ccb1)
		msvdx_free_ccb(&msvdx_priv->ccb1);

	return 1;
}

static void msvdx_rendec_init_by_reg(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t cmd;

	PSB_WMSVDX32(msvdx_priv->base_addr0, MSVDX_RENDEC_BASE_ADDR0_OFFSET);
	PSB_WMSVDX32(msvdx_priv->base_addr1, MSVDX_RENDEC_BASE_ADDR1_OFFSET);

	cmd = 0;
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_BUFFER_SIZE,
			RENDEC_BUFFER_SIZE0, RENDEC_A_SIZE / 4096);
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_BUFFER_SIZE,
			RENDEC_BUFFER_SIZE1, RENDEC_B_SIZE / 4096);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_BUFFER_SIZE_OFFSET);

	cmd = 0;
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
			RENDEC_DECODE_START_SIZE, 0);
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
			RENDEC_BURST_SIZE_W, 1);
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
			RENDEC_BURST_SIZE_R, 1);
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
			RENDEC_EXTERNAL_MEMORY, 1);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL1_OFFSET);

	cmd = 0x00101010;
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT0_OFFSET);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT1_OFFSET);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT2_OFFSET);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT3_OFFSET);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT4_OFFSET);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT5_OFFSET);

	cmd = 0;
	REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL0, RENDEC_INITIALISE,
			1);
	PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL0_OFFSET);
}

static int32_t msvdx_rendec_init_by_msg(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/* at this stage, FW is uplaoded successfully,
	 * can send RENDEC init message */
	struct fw_init_msg init_msg;
	init_msg.header.bits.msg_size = sizeof(struct fw_init_msg);
	init_msg.header.bits.msg_type = MTX_MSGID_INIT;
	init_msg.rendec_addr0 = msvdx_priv->base_addr0;
	init_msg.rendec_addr1 = msvdx_priv->base_addr1;
	init_msg.rendec_size.bits.rendec_size0 = RENDEC_A_SIZE / (4 * 1024);
	init_msg.rendec_size.bits.rendec_size1 = RENDEC_B_SIZE / (4 * 1024);
	return psb_mtx_send(dev_priv, (void *)&init_msg);
}

#if 0
static void msvdx_sw_rest(struct drm_psb_private *dev_priv)
{
	uint32_t reg_val;

	/* Issue software reset for all but core*/
	PSB_WMSVDX32((uint32_t) ~MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL_OFFSET);

	reg_val = PSB_RMSVDX32(MSVDX_CONTROL_OFFSET);
	PSB_WMSVDX32(0, MSVDX_CONTROL_OFFSET);
	PSB_WMSVDX32(MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL_OFFSET);

	reg_val = 0;
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CNT_CTRL, 0x3);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_ENABLE, 0);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_ACTION0, 1);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CLEAR_SELECT, 1);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CLKDIV_SELECT, 7);
	PSB_WMSVDX32(820, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	PSB_WMSVDX32(reg_val, FE_MSVDX_WDT_CONTROL_OFFSET);

	reg_val = 0;
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CNT_CTRL, 0x7);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_ENABLE, 0);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_ACTION0, 1);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CLEAR_SELECT, 0xd);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CLKDIV_SELECT, 7);
	PSB_WMSVDX32(8200, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	PSB_WMSVDX32(reg_val, BE_MSVDX_WDT_CONTROL_OFFSET);
}
#endif

static void msvdx_tile_setup(struct drm_psb_private *dev_priv)
{
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	msvdx_priv->tile_region_start0 =
		dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].gpu_offset;

	msvdx_priv->tile_region_end0 = msvdx_priv->tile_region_start0 +
	(dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].size << PAGE_SHIFT);

	msvdx_priv->tile_region_start1 =
		dev_priv->bdev.man[TTM_PL_TT].gpu_offset;

	msvdx_priv->tile_region_end1 = msvdx_priv->tile_region_start1 +
		(dev_priv->bdev.man[TTM_PL_TT].size << PAGE_SHIFT);

#if 0
	if (drm_psb_msvdx_tiling && IS_MSVDX_MEM_TILE(dev)) {
		uint32_t tile_start =
			dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].gpu_offset;
		uint32_t tile_end = tile_start +
		(dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].size << PAGE_SHIFT);

		/* Enable memory tiling */
		cmd = ((tile_start >> 20) + (((tile_end >> 20) - 1) << 12) +
					((0x8 | 2) << 24)); /* 2k stride */

		PSB_DEBUG_GENERAL("MSVDX: MMU Tiling register0 %08x\n", cmd);
		PSB_DEBUG_GENERAL("	  Region 0x%08x-0x%08x\n",
					tile_start, tile_end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE0_OFFSET);

		tile_start =
			dev_priv->bdev.man[TTM_PL_TT].gpu_offset;
		tile_end = tile_start +
			(dev_priv->bdev.man[TTM_PL_TT].size << PAGE_SHIFT);

		cmd = ((tile_start >> 20) + (((tile_end >> 20) - 1) << 12) +
					((0x8 | 2) << 24)); /* 2k stride */

		PSB_DEBUG_GENERAL("MSVDX: MMU Tiling register1 %08x\n", cmd);
		PSB_DEBUG_GENERAL("	  Region 0x%08x-0x%08x\n",
					tile_start, tile_end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE1_OFFSET);
	}
#endif
	return;
}

#ifdef CONFIG_VIDEO_MRFLD_EC
static void msvdx_init_ec(struct msvdx_private *msvdx_priv)
{
	struct drm_psb_private *dev_priv = msvdx_priv->dev_priv;

	/* we should restore the state, if we power down/up
	 * during EC */
	PSB_WMSVDX32(0, 0x2000 + 0xcb0); /* EXT_FW_LAST_MBS */
	PSB_WMSVDX32(0, 0x2000 + 0xcb4); /* EXT_FW_LAST_MBS */
	PSB_WMSVDX32(0, 0x2000 + 0xcb8); /* EXT_FW_LAST_MBS */
	PSB_WMSVDX32(0, 0x2000 + 0xcbc); /* EXT_FW_LAST_MBS */

	msvdx_priv->msvdx_ec_ctx[0] =
		kzalloc(sizeof(struct psb_msvdx_ec_ctx) *
				PSB_MAX_EC_INSTANCE,
				GFP_KERNEL);
	if (msvdx_priv->msvdx_ec_ctx[0] == NULL) {
		DRM_ERROR("MSVDX:fail to allocate memory for ec ctx\n");
	} else {
		int i;
		for (i = 1; i < PSB_MAX_EC_INSTANCE; i++)
			msvdx_priv->msvdx_ec_ctx[i] =
				msvdx_priv->msvdx_ec_ctx[0] + i;
		for (i = 0; i < PSB_MAX_EC_INSTANCE; i++)
			msvdx_priv->msvdx_ec_ctx[i]->fence =
					PSB_MSVDX_INVALID_FENCE;
	}
	INIT_WORK(&(msvdx_priv->ec_work), psb_msvdx_do_concealment);
	return;
}
#endif

#if 0
void msvdx_init_test(struct drm_device *dev)
{
	/* Send test message */
	uint32_t msg_buf[FW_VA_DEBUG_TEST2_SIZE >> 2];

	MEMIO_WRITE_FIELD(msg_buf, FW_VA_DEBUG_TEST2_MSG_SIZE,
			  FW_VA_DEBUG_TEST2_SIZE);
	MEMIO_WRITE_FIELD(msg_buf, FW_VA_DEBUG_TEST2_ID,
			  VA_MSGID_TEST2);

	ret = psb_mtx_send(dev_priv, msg_buf);
	if (ret) {
		DRM_ERROR("psb: MSVDX sending fails.\n");
		goto out;
	}

	/* Wait for Mtx to ack this message */
	psb_poll_mtx_irq(dev_priv);
}
#endif

static int msvdx_startup_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	int ret;
	struct msvdx_private *msvdx_priv;

	printk(KERN_INFO "MSVDX: core id	%x\n",
				PSB_RMSVDX32(MSVDX_CORE_ID_OFFSET));
	printk(KERN_INFO "MSVDX: core rev	%x\n",
				PSB_RMSVDX32(MSVDX_CORE_REV_OFFSET));

	msvdx_priv = kmalloc(sizeof(struct msvdx_private), GFP_KERNEL);
	if (msvdx_priv == NULL) {
		DRM_ERROR("MSVDX: alloc msvdx_private failed.\n");
		return 1;
	}

	dev_priv->msvdx_private = msvdx_priv;
	memset(msvdx_priv, 0, sizeof(struct msvdx_private));
	msvdx_priv->dev_priv = dev_priv;
	msvdx_priv->dev = dev;
#ifdef CONFIG_DRM_VXD_BYT
	msvdx_priv->fw_loaded_by_punit = 0;
#else
#ifdef MERRIFIELD
	msvdx_priv->msvdx_needs_reset = 1;

	if (IS_MRFLD(dev))
		msvdx_priv->fw_loaded_by_punit = 0;
	else
#endif
		msvdx_priv->fw_loaded_by_punit =
			((dev)->pdev->revision >= 0xc) || \
			(((dev)->pci_device & 0xffff) == 0x08c7) || \
			(((dev)->pci_device & 0xffff) == 0x08c8);
#endif
	msvdx_tile_setup(dev_priv);
	msvdx_priv->pm_gating_count = 0;

	/* get device --> drm_device --> drm_psb_private --> msvdx_priv
	 * for psb_msvdx_pmstate_show: msvdx_pmpolicy
	 * if not pci_set_drvdata, can't get drm_device from device
	 */
	/* pci_set_drvdata(dev->pdev, dev); */
	if (device_create_file(&dev->pdev->dev,
			       &dev_attr_msvdx_pmstate))
		DRM_ERROR("MSVDX: could not create sysfs file\n");
	msvdx_priv->sysfs_pmstate = sysfs_get_dirent(
					    dev->pdev->dev.kobj.sd,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
					    NULL,
#endif
					    "msvdx_pmstate");
#ifdef CONFIG_VIDEO_MRFLD_EC
	msvdx_init_ec(msvdx_priv);
#endif
	INIT_DELAYED_WORK(&msvdx_priv->msvdx_suspend_wq,
				&psb_powerdown_msvdx);
	/* Initialize comand msvdx queueing */
	INIT_LIST_HEAD(&msvdx_priv->msvdx_queue);
	mutex_init(&msvdx_priv->msvdx_mutex);
	spin_lock_init(&msvdx_priv->msvdx_lock);
#ifndef CONFIG_DRM_VXD_BYT
	/*figure out the stepping */
	pci_read_config_byte(dev->pdev, PSB_REVID_OFFSET, &psb_rev_id);
#endif
	msvdx_priv->mmu_recover_page = alloc_page(GFP_DMA32);
	if (!msvdx_priv->mmu_recover_page)
		goto err_exit;

#ifdef PSB_MSVDX_SAVE_RESTORE_VEC
	msvdx_priv->vec_local_mem_size = VEC_LOCAL_MEM_BYTE_SIZE;
	if (!msvdx_priv->vec_local_mem_data) {
		msvdx_priv->vec_local_mem_data =
			kmalloc(msvdx_priv->vec_local_mem_size, GFP_KERNEL);
		if (!msvdx_priv->vec_local_mem_data) {
			DRM_ERROR("Allocate vec local mem data fail\n");
			goto err_exit;
		}
		memset(msvdx_priv->vec_local_mem_data, 0, msvdx_priv->vec_local_mem_size);
	}
#endif
	return 0;

err_exit:
	DRM_ERROR("MSVDX: init one time failed\n");
	kfree(dev_priv->msvdx_private);

	return 1;
}

void msvdx_post_powerup_core_reset(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	psb_msvdx_mtx_set_clocks(dev_priv->dev, clk_enable_all);

	/* msvdx sw reset should be done by gunit before loading fw */
#if 0
	/* Issue software reset for all but core*/
	PSB_WMSVDX32(~MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK,
				MSVDX_CONTROL_OFFSET);

	PSB_RMSVDX32(MSVDX_CONTROL_OFFSET);
	PSB_WMSVDX32(0, MSVDX_CONTROL_OFFSET);

	psb_wait_for_register(dev_priv, MSVDX_MMU_MEM_REQ_OFFSET, 0,
						0xff, 100, 100);
	PSB_WMSVDX32(MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK,
				MSVDX_CONTROL_OFFSET);
	psb_wait_for_register(dev_priv, MSVDX_CONTROL_OFFSET, 0,
			MSVDX_CONTROL__MSVDX_SOFT_RESET_MASK,
									100, 100);
#endif

	/* psb_msvdx_clearirq only clear CR_MTX_IRQ int,
	 * while DDK set 0xFFFFFFFF */
	psb_msvdx_clearirq(dev);
	psb_msvdx_enableirq(dev);
}

int msvdx_mtx_init(struct drm_device *dev, int error_reset)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t clk_divider = 200;
	int ret;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/* Reset MTX not done here, should be done before loading fw
	 * while fw is loaded by gunit now */
#if 0
	PSB_WMSVDX32(MTX_SOFT_RESET__MTXRESET, MTX_SOFT_RESET_OFFSET);
	PSB_WMSVDX32(0, MSVDX_COMMS_SIGNATURE);
#endif

	/* These should not be reprogrames after a error reset */
	if (!error_reset) {
		PSB_WMSVDX32(0, MSVDX_COMMS_MSG_COUNTER);
		PSB_WMSVDX32(0, MSVDX_EXT_FW_ERROR_STATE);
	}

	PSB_WMSVDX32(0, MSVDX_COMMS_ERROR_TRIG);
	PSB_WMSVDX32(0, MSVDX_COMMS_TO_HOST_RD_INDEX);
	PSB_WMSVDX32(0, MSVDX_COMMS_TO_HOST_WRT_INDEX);
	PSB_WMSVDX32(0, MSVDX_COMMS_TO_MTX_RD_INDEX);
	PSB_WMSVDX32(0, MSVDX_COMMS_TO_MTX_WRT_INDEX);
	PSB_WMSVDX32(0, MSVDX_COMMS_FIRMWARE_ID);
	/* IMG DDK set as: gui32DeviceNodeFlags & 0x4000, while it is not set in fw spec
	 * The bit neede to be set preboot is the performce data bit since this
	 * controls caused the firmware to rebalance the message queues. */
	PSB_WMSVDX32(0, MSVDX_COMMS_OFFSET_FLAGS);

	/* DDK: check device_node_flags with 0x400 to | (1<<16),
	 * while it is not set in fw spec */
	PSB_WMSVDX32(clk_divider - 1, MTX_SYSC_TIMERDIV_OFFSET);

	/* DDK: LLDMA upload fw, which is now done by gunit */

	/* DDK: redefine toHost and toMTX msg buffer, seems not needed */

	/* Wait for the signature value to be written back */
	ret = psb_wait_for_register(dev_priv, MSVDX_COMMS_SIGNATURE,
				    MSVDX_COMMS_SIGNATURE_VALUE,
				    0xffffffff,
				    1000, 1000);
	if (ret) {
		PSB_DEBUG_WARN("WARN: Gunit upload fw failure,\n"
				"MSVDX_COMMS_SIGNATURE reg is 0x%x,"
				"MSVDX_COMMS_FW_STATUS reg is 0x%x,"
				"MTX_ENABLE reg is 0x%x.\n",
				PSB_RMSVDX32(MSVDX_COMMS_SIGNATURE),
				PSB_RMSVDX32(MSVDX_COMMS_FW_STATUS),
				PSB_RMSVDX32(MTX_ENABLE_OFFSET));
		msvdx_priv->msvdx_needs_reset |=
				MSVDX_RESET_NEEDS_REUPLOAD_FW |
				MSVDX_RESET_NEEDS_INIT_FW;
	}
	return ret;
}

/* This value is hardcoded in FW */
#define WDT_CLOCK_DIVIDER 128
int psb_msvdx_post_boot_init(struct drm_device *dev)
{
	struct fw_init_msg init_msg;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t device_node_flags =
			DSIABLE_IDLE_GPIO_SIG | DSIABLE_Auto_CLOCK_GATING |
			RETURN_VDEB_DATA_IN_COMPLETION;

	/* DDK set fe_wdt_clks as 0x820 and be_wdt_clks as 0x8200 */
	uint32_t fe_wdt_clks = 0x334 * WDT_CLOCK_DIVIDER;
	uint32_t be_wdt_clks = 0x2008 * WDT_CLOCK_DIVIDER;

	PSB_WMSVDX32(FIRMWAREID, MSVDX_COMMS_FIRMWARE_ID);
	PSB_WMSVDX32(device_node_flags, MSVDX_COMMS_OFFSET_FLAGS);

	/* read register bank size */
	{
		uint32_t ram_bank_size;
		uint32_t bank_size, reg;
		reg = PSB_RMSVDX32(MSVDX_MTX_RAM_BANK_OFFSET);
		bank_size =
			REGIO_READ_FIELD(reg, MSVDX_MTX_RAM_BANK,
					 MTX_RAM_BANK_SIZE);
		ram_bank_size = (uint32_t)(1 << (bank_size + 2));
		PSB_DEBUG_GENERAL("MSVDX: RAM bank size = %d bytes\n",
				  ram_bank_size);
	}
	/* host end */

	/* DDK setup tiling region here */
	/* DDK set MMU_CONTROL2 register */

#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	/* set watchdog timer here */
	int reg_val = 0;
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CNT_CTRL, 0x3);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_ENABLE, 0);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_ACTION0, 1);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CLEAR_SELECT, 1);
	REGIO_WRITE_FIELD(reg_val, FE_MSVDX_WDT_CONTROL, FE_WDT_CLKDIV_SELECT, 7);
	PSB_WMSVDX32(fe_wdt_clks / WDT_CLOCK_DIVIDER, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	PSB_WMSVDX32(reg_val, FE_MSVDX_WDT_CONTROL_OFFSET);

	reg_val = 0;
	/* DDK set BE_WDT_CNT_CTRL as 0x5 and BE_WDT_CLEAR_SELECT as 0x1 */
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CNT_CTRL, 0x7);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_ENABLE, 0);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_ACTION0, 1);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CLEAR_SELECT, 0xd);
	REGIO_WRITE_FIELD(reg_val, BE_MSVDX_WDT_CONTROL, BE_WDT_CLKDIV_SELECT, 7);

	PSB_WMSVDX32(be_wdt_clks / WDT_CLOCK_DIVIDER, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	PSB_WMSVDX32(reg_val, BE_MSVDX_WDT_CONTROL_OFFSET);
#else
	/* for the other two, use the default value punit set */
	PSB_WMSVDX32(fe_wdt_clks / WDT_CLOCK_DIVIDER, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	PSB_WMSVDX32(be_wdt_clks / WDT_CLOCK_DIVIDER, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);
#endif
	return msvdx_rendec_init_by_msg(dev);
}

#ifdef PSB_MSVDX_FW_LOADED_BY_HOST

int psb_msvdx_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	uint32_t cmd;
	int ret;
	struct msvdx_private *msvdx_priv;

	if (!dev_priv->msvdx_private) {
		if (msvdx_startup_init(dev))
			return 1;
	}

	if (dev_priv->msvdx_private == NULL)
		return 1;

	msvdx_priv = dev_priv->msvdx_private;

	msvdx_priv->msvdx_busy = 0;
	msvdx_priv->msvdx_hw_busy = 1;

	/* Enable Clocks */
	PSB_DEBUG_INIT("Enabling clocks\n");
	psb_msvdx_mtx_set_clocks(dev_priv->dev, clk_enable_all);
#if 0
	msvdx_sw_rest(dev);
#endif
	if (!msvdx_priv->fw_loaded_by_punit) {
		/* Enable MMU by removing all bypass bits */
		PSB_WMSVDX32(0, MSVDX_MMU_CONTROL0_OFFSET);
	}

	ret = msvdx_alloc_ccb_for_rendec(dev);
	if (ret) {
		DRM_ERROR("msvdx_alloc_ccb_for_rendec failed.\n");
		return 1;
	}

	/* PSB_WMSVDX32(clk_enable_minimal, MSVDX_MAN_CLK_ENABLE); */
	PSB_DEBUG_INIT("MSVDX:defer firmware loading to the"
		       " place when receiving user space commands\n");

	if (!msvdx_priv->fw_loaded_by_punit) {
		msvdx_rendec_init_by_reg(dev);
		if (!msvdx_priv->fw) {
			ret = psb_msvdx_alloc_fw_bo(dev_priv);
			if (ret) {
				DRM_ERROR("psb_msvdx_alloc_fw_bo failed.\n");
				return 1;
			}
		}
		/* move fw loading to the place receiving first cmd buffer */
		msvdx_priv->msvdx_fw_loaded = 0; /* need to load firware */
		/* it should be set at punit post boot init phase */
		PSB_WMSVDX32(820, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
		PSB_WMSVDX32(8200, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);

		PSB_WMSVDX32(820, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
		PSB_WMSVDX32(8200, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	} else {
		msvdx_priv->rendec_init = 0;
		/* for the other two, use the default value punit set */
		PSB_WMSVDX32(0x334, FE_MSVDX_WDT_COMPAREMATCH_OFFSET);
		PSB_WMSVDX32(0x2008, BE_MSVDX_WDT_COMPAREMATCH_OFFSET);
	}

	psb_msvdx_clearirq(dev);
	psb_msvdx_enableirq(dev);

#ifndef CONFIG_DRM_VXD_BYT
	PSB_DEBUG_INIT("MSDVX:old clock gating disable = 0x%08x\n",
		       PSB_RVDC32(PSB_MSVDX_CLOCKGATING));
#endif
	if (!msvdx_priv->fw_loaded_by_punit) {
		cmd = 0;
		cmd = PSB_RMSVDX32(VEC_SHIFTREG_CONTROL_OFFSET);
		REGIO_WRITE_FIELD(cmd,
				  VEC_SHIFTREG_CONTROL,
				  SR_MASTER_SELECT,
				  1);  /* Host */
		PSB_WMSVDX32(cmd, VEC_SHIFTREG_CONTROL_OFFSET);
	}

	return 0;
}

#else

int psb_msvdx_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	int ret;
	struct msvdx_private *msvdx_priv;

	if (!dev_priv->msvdx_private) {
		if (msvdx_startup_init(dev))
			return 1;
	}

	if (dev_priv->msvdx_private == NULL)
		return 1;

	msvdx_priv = dev_priv->msvdx_private;

	msvdx_priv->msvdx_busy = 0;
	msvdx_priv->msvdx_hw_busy = 1;

	/* DDK: Configure MSVDX Memory Stalling iwth the min, max and ratio of access */

	msvdx_post_powerup_core_reset(dev);

	/* Enable Clocks */
	PSB_DEBUG_INIT("Enabling clocks\n");
	psb_msvdx_mtx_set_clocks(dev_priv->dev, clk_enable_all);

	msvdx_priv->rendec_init = 0;

	ret = msvdx_mtx_init(dev, msvdx_priv->decoding_err);
	if (ret) {
		PSB_DEBUG_WARN("WARN: msvdx_mtx_init failed.\n");
		return 1;
	}

	ret = msvdx_alloc_ccb_for_rendec(dev);
	if (ret) {
		PSB_DEBUG_WARN("WARN: msvdx_alloc_ccb_for_rendec failed.\n");
		return 1;
	}

#ifndef CONFIG_DRM_VXD_BYT
	PSB_DEBUG_INIT("MSDVX:old clock gating disable = 0x%08x\n",
		       PSB_RVDC32(PSB_MSVDX_CLOCKGATING));
#endif
	return 0;
}

#endif

int psb_msvdx_uninit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/* PSB_WMSVDX32 (clk_enable_minimal, MSVDX_MAN_CLK_ENABLE); */
	PSB_DEBUG_INIT("MSVDX:set the msvdx clock to 0\n");
	psb_msvdx_mtx_set_clocks(dev_priv->dev, 0);

	if (NULL == msvdx_priv) {
		DRM_ERROR("MSVDX: psb_msvdx_uninit: msvdx_priv is NULL!\n");
		return -1;
	}

	if (msvdx_priv->ccb0)
		msvdx_free_ccb(&msvdx_priv->ccb0);
	if (msvdx_priv->ccb1)
		msvdx_free_ccb(&msvdx_priv->ccb1);
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	if (msvdx_priv->msvdx_fw)
		kfree(msvdx_priv->msvdx_fw);
#endif
#ifdef PSB_MSVDX_SAVE_RESTORE_VEC
	if (msvdx_priv->vec_local_mem_data)
		kfree(msvdx_priv->vec_local_mem_data);
#endif
	kfree(msvdx_priv->msvdx_ec_ctx[0]);

	if (msvdx_priv->mmu_recover_page)
		__free_page(msvdx_priv->mmu_recover_page);

	if (msvdx_priv) {
		/* pci_set_drvdata(dev->pdev, NULL); */
		device_remove_file(&dev->pdev->dev, &dev_attr_msvdx_pmstate);
		sysfs_put(msvdx_priv->sysfs_pmstate);
		msvdx_priv->sysfs_pmstate = NULL;

		kfree(msvdx_priv);
		dev_priv->msvdx_private = NULL;
	}

	return 0;
}

/*
 * watchdog function can be enabled whenever required.
 */
#if 0
void psb_schedule_watchdog(struct drm_psb_private *dev_priv)
{
	struct timer_list *wt = &dev_priv->watchdog_timer;
	unsigned long irq_flags;

	spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
	if (dev_priv->timer_available && !timer_pending(wt)) {
		wt->expires = jiffies + PSB_WATCHDOG_DELAY;
		add_timer(wt);
	}
	spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);
}


static void psb_watchdog_func(unsigned long data)
{
	struct drm_psb_private *dev_priv = (struct drm_psb_private *) data;
	int msvdx_lockup;
	int msvdx_idle;
	unsigned long irq_flags;

	psb_msvdx_lockup(dev_priv, &msvdx_lockup, &msvdx_idle);

	if (msvdx_lockup) {
		spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
		dev_priv->timer_available = 0;
		spin_unlock_irqrestore(&dev_priv->watchdog_lock,
				       irq_flags);
		if (msvdx_lockup)
			schedule_work(&dev_priv->msvdx_watchdog_wq);
	}
	if (!msvdx_idle)
		psb_schedule_watchdog(dev_priv);
}

static void psb_msvdx_reset_wq(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, msvdx_watchdog_wq);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	unsigned long irq_flags;

	mutex_lock(&msvdx_priv->msvdx_mutex);
	if (msvdx_priv->fw_loaded_by_punit)
		msvdx_priv->msvdx_needs_reset |= MSVDX_RESET_NEEDS_REUPLOAD_FW |
			MSVDX_RESET_NEEDS_INIT_FW;
	else
		msvdx_priv->msvdx_needs_reset = 1;
	msvdx_priv->msvdx_current_sequence++;
	PSB_DEBUG_GENERAL
	("MSVDXFENCE: incremented msvdx_current_sequence to :%d\n",
	 msvdx_priv->msvdx_current_sequence);

	psb_fence_error(msvdx_priv->dev, PSB_ENGINE_DECODE,
			msvdx_priv->msvdx_current_sequence,
			_PSB_FENCE_TYPE_EXE, DRM_CMD_HANG);

	spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
	dev_priv->timer_available = 1;
	spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);

	psb_msvdx_flush_cmd_queue(msvdx_priv->dev);

	psb_schedule_watchdog(dev_priv);
	mutex_unlock(&msvdx_priv->msvdx_mutex);
}

void psb_watchdog_init(struct drm_psb_private *dev_priv)
{
	struct timer_list *wt = &dev_priv->watchdog_timer;
	unsigned long irq_flags;

	spin_lock_init(&dev_priv->watchdog_lock);
	spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
	init_timer(wt);
	INIT_WORK(&dev_priv->msvdx_watchdog_wq, &psb_msvdx_reset_wq);
	wt->data = (unsigned long) dev_priv;
	wt->function = &psb_watchdog_func;
	dev_priv->timer_available = 1;
	spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);
}

void psb_watchdog_takedown(struct drm_psb_private *dev_priv)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
	dev_priv->timer_available = 0;
	spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);
	(void) del_timer_sync(&dev_priv->watchdog_timer);
}
#endif
