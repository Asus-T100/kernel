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
 **************************************************************************/

#include <drm/drmP.h>
#include <drm/drm.h>
#include "psb_drv.h"
#include "psb_msvdx.h"
#include "psb_msvdx_ec.h"
#include <linux/firmware.h>

uint8_t psb_rev_id;

/*
 * the original 1000 of udelay is derive from reference driver
 * From Liu, Haiyang, changed the originial udelay value from 1000 to 5
 * can save 3% C0 residence
 */
int psb_wait_for_register(struct drm_psb_private *dev_priv,
			  uint32_t offset, uint32_t value, uint32_t enable)
{
	uint32_t reg_value;
	uint32_t poll_cnt = 2000000;
	while (poll_cnt) {
		reg_value = PSB_RMSVDX32(offset);
		if (value == (reg_value & enable))	/* All the bits are reset   */
			return 0;	/* So exit			*/

		/* Wait a bit */
		PSB_UDELAY(5);
		poll_cnt--;
	}
	DRM_ERROR("MSVDX: Timeout while waiting for register %08x:"
		  " expecting %08x (mask %08x), got %08x\n",
		  offset, value, enable, reg_value);

	return 1;
}

int psb_poll_mtx_irq(struct drm_psb_private *dev_priv)
{
	int ret = 0;
	uint32_t mtx_int = 0;

	REGIO_WRITE_FIELD_LITE(mtx_int, MSVDX_INTERRUPT_STATUS, CR_MTX_IRQ,
			       1);

	ret = psb_wait_for_register(dev_priv, MSVDX_INTERRUPT_STATUS,
				    /* Required value */
				    mtx_int,
				    /* Enabled bits */
				    mtx_int);

	if (ret) {
		DRM_ERROR("MSVDX: Error Mtx did not return"
			  " int within a resonable time\n");
		return ret;
	}

	PSB_DEBUG_IRQ("MSVDX: Got MTX Int\n");

	/* Got it so clear the bit */
	PSB_WMSVDX32(mtx_int, MSVDX_INTERRUPT_CLEAR);

	return ret;
}

void psb_write_mtx_core_reg(struct drm_psb_private *dev_priv,
			    const uint32_t core_reg, const uint32_t val)
{
	uint32_t reg = 0;

	/* Put data in MTX_RW_DATA */
	PSB_WMSVDX32(val, MSVDX_MTX_REGISTER_READ_WRITE_DATA);

	/* DREADY is set to 0 and request a write */
	reg = core_reg;
	REGIO_WRITE_FIELD_LITE(reg, MSVDX_MTX_REGISTER_READ_WRITE_REQUEST,
			       MTX_RNW, 0);
	REGIO_WRITE_FIELD_LITE(reg, MSVDX_MTX_REGISTER_READ_WRITE_REQUEST,
			       MTX_DREADY, 0);
	PSB_WMSVDX32(reg, MSVDX_MTX_REGISTER_READ_WRITE_REQUEST);

	psb_wait_for_register(dev_priv,
			      MSVDX_MTX_REGISTER_READ_WRITE_REQUEST,
			      MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK,
			      MSVDX_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK);
}

static void psb_free_ccb(struct ttm_buffer_object **ccb)
{
	ttm_bo_unref(ccb);
	*ccb = NULL;
}

static int psb_msvdx_reset_internal_unused(struct drm_psb_private *dev_priv)
{
	int ret = 0;
	if (IS_D0(dev_priv->dev)) {
		uint32_t core_rev;
		/* Enable Clocks */
		PSB_DEBUG_GENERAL("Enabling clocks\n");
		PSB_WMSVDX32(clk_enable_all, MSVDX_MAN_CLK_ENABLE);

		/* Always pause the MMU as the core may be still active
		 * when resetting.  It is very bad to have memory
		 * activity at the same time as a reset - Very Very bad
		 */
		PSB_WMSVDX32(2, MSVDX_MMU_CONTROL0);

		core_rev = PSB_RMSVDX32(MSVDX_CORE_REV);
		if (core_rev < 0x00050502) {
			/* if there's any page fault */
			int int_status = PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS);

			if (int_status & MSVDX_INTERRUPT_STATUS_CR_MMU_FAULT_IRQ_MASK) {
				/* was it a page table rather than a protection fault */
				int mmu_status = PSB_RMSVDX32(MSVDX_MMU_STATUS);

				if (mmu_status & 1) {
					struct page *p;
					unsigned int *pptd;
					unsigned int loop;
					uint32_t ptd_addr;

					/* do work around */
					p = alloc_page(GFP_DMA32);
					if (!p) {
						ret = -1;
						return ret;
					}
					ptd_addr = page_to_pfn(p) << PAGE_SHIFT;
					pptd = kmap(p);
					for (loop = 0; loop < 1024; loop++)
						pptd[loop] = ptd_addr | 0x00000003;
					PSB_WMSVDX32(ptd_addr, MSVDX_CORE_CR_MMU_DIR_LIST_BASE_OFFSET +  0);
					PSB_WMSVDX32(ptd_addr, MSVDX_CORE_CR_MMU_DIR_LIST_BASE_OFFSET +  4);
					PSB_WMSVDX32(ptd_addr, MSVDX_CORE_CR_MMU_DIR_LIST_BASE_OFFSET +  8);
					PSB_WMSVDX32(ptd_addr, MSVDX_CORE_CR_MMU_DIR_LIST_BASE_OFFSET + 12);

					PSB_WMSVDX32(6, MSVDX_MMU_CONTROL0);
					PSB_WMSVDX32(MSVDX_INTERRUPT_STATUS_CR_MMU_FAULT_IRQ_MASK, MSVDX_INTERRUPT_STATUS);
					__free_page(p);
				}
			}
		}

		/* make sure *ALL* outstanding reads have gone away */
		{
			int loop;
			uint32_t cmd;
			for (loop = 0; loop < 10; loop++)
				ret = psb_wait_for_register(dev_priv, MSVDX_MMU_MEM_REQ, 0, 0xff);
			if (ret) {
				ret = -1;
				return ret;
			}
			/* disconnect RENDEC decoders from memory */
			cmd = PSB_RMSVDX32(MSVDX_RENDEC_CONTROL1);
			REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1, RENDEC_DEC_DISABLE, 1);
			PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL1);

			/* Issue software reset for all but core */
			PSB_WMSVDX32((unsigned int)~MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL);
			PSB_RMSVDX32(MSVDX_CONTROL);
			PSB_WMSVDX32(0, MSVDX_CONTROL);
			/* make sure read requests are zero */
			ret = psb_wait_for_register(dev_priv, MSVDX_MMU_MEM_REQ, 0, 0xff);
			if (!ret) {
				/* Issue software reset */
				PSB_WMSVDX32(MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL);

				ret = psb_wait_for_register(dev_priv, MSVDX_CONTROL, 0,
						MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK);

				if (!ret) {
					/* Clear interrupt enabled flag */
					PSB_WMSVDX32(0, MSVDX_HOST_INTERRUPT_ENABLE);

					/* Clear any pending interrupt flags */
					PSB_WMSVDX32(0xFFFFFFFF, MSVDX_INTERRUPT_CLEAR);
				}
			}
		}
	} else {
		int loop;
		/* Enable Clocks */
		PSB_DEBUG_GENERAL("Enabling clocks\n");
		PSB_WMSVDX32(clk_enable_all, MSVDX_MAN_CLK_ENABLE);
		/* Always pause the MMU as the core may be still active when resetting.  It is very bad to have memory
		   activity at the same time as a reset - Very Very bad */
		PSB_WMSVDX32(2, MSVDX_MMU_CONTROL0);
		for (loop = 0; loop < 50; loop++)
			ret = psb_wait_for_register(dev_priv, MSVDX_MMU_MEM_REQ, 0,
					0xff);
		if (ret)
			return ret;
	}
	return ret;
}

/**
 * Reset chip and disable interrupts.
 * Return 0 success, 1 failure
 */
int psb_msvdx_reset(struct drm_psb_private *dev_priv)
{
	int ret = 0;

	/*
	ret = psb_msvdx_reset_internal_unused(dev_priv);
	if (ret)
		return ret;
	*/

	/* Issue software reset */
	/* PSB_WMSVDX32(msvdx_sw_reset_all, MSVDX_CONTROL); */
	PSB_WMSVDX32(MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK, MSVDX_CONTROL);

	ret = psb_wait_for_register(dev_priv, MSVDX_CONTROL, 0,
			MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK);

	if (!ret) {
		/* Clear interrupt enabled flag */
		PSB_WMSVDX32(0, MSVDX_HOST_INTERRUPT_ENABLE);

		/* Clear any pending interrupt flags */
		PSB_WMSVDX32(0xFFFFFFFF, MSVDX_INTERRUPT_CLEAR);
	}

	/* mutex_destroy(&msvdx_priv->msvdx_mutex); */
out:
	return ret;
}

static int psb_allocate_ccb(struct drm_device *dev,
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
		PSB_DEBUG_GENERAL("ttm_bo_kmap failed ret: %d\n", ret);
		ttm_bo_unref(ccb);
		*ccb = NULL;
		return 1;
	}
	/*
	  memset(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), 0,
	  RENDEC_A_SIZE);
	*/
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
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	ret = snprintf(buf, 64, "MSVDX Power state 0x%s\n",
		       ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND)
				? "ON" : "OFF");

	return ret;
}

static DEVICE_ATTR(msvdx_pmstate, 0444, psb_msvdx_pmstate_show, NULL);

static void psb_msvdx_conceal_mb(char *ec_start, char *ref_start,
				 uint32_t start_err_mb, uint32_t end_err_mb,
				 uint32_t pic_width_mb, uint32_t stride,
				 uint32_t mb_width, uint32_t mb_height)
{
	int i, offset_start, offset_end, size, full_line_start, full_line_end, extra_line_start, extra_line_end;
	char *src, *dst;

	(void)offset_end;
	(void)offset_start;

	full_line_start = (start_err_mb / pic_width_mb) * stride * mb_height;
	full_line_end = ((end_err_mb + 1) / pic_width_mb) * stride * mb_height;
	extra_line_start = (start_err_mb % pic_width_mb) * mb_width;
	extra_line_end = (end_err_mb + 1) % pic_width_mb * mb_width;
	if (extra_line_start != 0) {
		size = stride - extra_line_start;
		src = ref_start + full_line_start + extra_line_start;
		dst = ec_start + full_line_start + extra_line_start;
		for (i = 0; i < mb_height; i++) {
			memcpy(dst, src, size);
			/* memset(dst, 255, size); */
			src += stride;
			dst += stride;
		}
		full_line_start += stride * mb_height;
	}
	src = ref_start + full_line_start;
	dst = ec_start + full_line_start;
	size =  full_line_end - full_line_start;
	memcpy(dst, src, size);
	/* memset(dst, 255, size); */
	if (extra_line_end != 0) {
		size = extra_line_end;
		src = ref_start + full_line_end;
		dst = ec_start + full_line_end;
		for (i = 0; i < mb_height; i++) {
			memcpy(dst, src, size);
			/* memset(dst, 255, size); */
			src += stride;
			dst += stride;
		}
	}
}


static void psb_msvdx_error_concealment(struct work_struct *data)
{
#if 0
	uint32_t i;
	int ret;
	struct msvdx_private *msvdx_priv = container_of(data, struct msvdx_private, ec_work);
	drm_psb_msvdx_frame_info_t *ec_frame = NULL;
	drm_psb_msvdx_frame_info_t *ref_frame = NULL;
	static struct ttm_bo_kmap_obj ec_kmap, ref_kmap;
	struct ttm_buffer_object *ec_bo = NULL;
	struct ttm_buffer_object *ref_bo = NULL;
	struct ttm_object_file *tfile = msvdx_priv->tfile;
	drm_psb_msvdx_decode_status_t *decode_status = NULL;
	bool is_iomem;
	char *ec_start, *ref_start;

	if (msvdx_priv->ec_fence > 0)
		msvdx_priv->ref_pic_fence = msvdx_priv->ec_fence - 1;
	else {
		DRM_ERROR("Can't do error concealment for the first frame.\n");
		return;
	}

	/*get the frame_info struct for error concealment frame*/
	for (i = 0; i < MAX_DECODE_BUFFERS; i++) {
		if (msvdx_priv->frame_info[i].fence == msvdx_priv->ec_fence) {
			ec_frame = &msvdx_priv->frame_info[i];
			break;
		}
	}
	if (!ec_frame) {
		DRM_ERROR("MSVDX: didn't find frame_info which matched the ec fence\n");
		return;
	}
	decode_status = &ec_frame->decode_status;
	ec_bo = ttm_buffer_object_lookup(tfile, ec_frame->handle);
	if (unlikely(ec_bo == NULL)) {
		printk(KERN_ERR " : Could not find buffer object for setstatus.\n");
		return;
	}
	ret = ttm_bo_reserve(ec_bo, true, true, false, 0);
	if (ret) {
		DRM_ERROR("MSVDX ERROR CONCEALMENT: reserver failed.\n");
		return;
	}
	ret = ttm_bo_kmap(ec_bo,
			  0,
			  (ec_frame->buffer_size +
			   PAGE_SIZE - 1) >> PAGE_SHIFT,
			  &ec_kmap);
	if (ret) {
		printk("MSVDX ERROR CONCEALMENT: ec kmap failed, %d.\n", ret);
		return;
	}
	ec_start = (unsigned char *) ttm_kmap_obj_virtual(&ec_kmap,
			&is_iomem);

	/*get the frame_info struct for reference frame*/
	for (i = 0; i < MAX_DECODE_BUFFERS; i++) {
		if (msvdx_priv->frame_info[i].fence == msvdx_priv->ref_pic_fence) {
			ref_frame = &msvdx_priv->frame_info[i];
			break;
		}
	}
	if (!ref_frame) {
		DRM_ERROR("MSVDX: didn't find frame_info which matched the ref fence\n");
		return;
	}
	ref_bo = ttm_buffer_object_lookup(tfile, ref_frame->handle);
	if (unlikely(ref_bo == NULL)) {
		printk(KERN_ERR " : Could not find buffer object for setstatus.\n");
	}
	ret = ttm_bo_reserve(ref_bo, true, true, false, 0);
	if (ret) {
		DRM_ERROR("MSVDX ERROR CONCEALMENT: reserver failed.\n");
		return;
	}
	ret = ttm_bo_kmap(ref_bo,
			  0,
			  (ref_frame->buffer_size +
			   PAGE_SIZE - 1) >> PAGE_SHIFT,
			  &ref_kmap);
	if (ret) {
		printk("MSVDX ERROR CONCEALMENT: ref kmap failed, %d.\n", ret);
		return;
	}
	ref_start = (unsigned char *) ttm_kmap_obj_virtual(&ref_kmap,
			&is_iomem);

	/*copy missing mb from ref picture to ec picture*/
	for (i = 0; i < decode_status->num_error_slice; i++) {
		if ((decode_status->start_error_mb_list[i] >= ec_frame->size_mb) ||
		    (decode_status->end_error_mb_list[i] >= ec_frame->size_mb) ||
		    (decode_status->start_error_mb_list[i] > decode_status->end_error_mb_list[i]))
			continue;
		psb_msvdx_conceal_mb(ec_start, ref_start,
				     decode_status->start_error_mb_list[i],
				     decode_status->end_error_mb_list[i],
				     ec_frame->picture_width_mb, ec_frame->buffer_stride,
				     16, 16);
		psb_msvdx_conceal_mb(ec_start + ec_frame->buffer_size * 2 / 3,
				     ref_start + ec_frame->buffer_size * 2 / 3,
				     decode_status->start_error_mb_list[i],
				     decode_status->end_error_mb_list[i],
				     ec_frame->picture_width_mb, ec_frame->buffer_stride,
				     16, 8);
	}

	ttm_bo_kunmap(&ec_kmap);
	ttm_bo_kunmap(&ref_kmap);
	ttm_bo_unreserve(ec_bo);
	ttm_bo_unreserve(ref_bo);

	if (ec_bo)
		ttm_bo_unref(&ec_bo);
	if (ref_bo)
		ttm_bo_unref(&ref_bo);

#endif
}

int psb_msvdx_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/* uint32_t clk_gate_ctrl = clk_enable_all; */
	uint32_t cmd;
	int ret;
	struct msvdx_private *msvdx_priv;
	/*
		uint32_t reg_value;
		uint32_t reg_val;
	*/
	if (!dev_priv->msvdx_private) {
		msvdx_priv = kmalloc(sizeof(struct msvdx_private), GFP_KERNEL);
		if (msvdx_priv == NULL)
			goto err_exit;

		dev_priv->msvdx_private = msvdx_priv;
		memset(msvdx_priv, 0, sizeof(struct msvdx_private));
		msvdx_priv->dev_priv = dev_priv;
		msvdx_priv->dev = dev;

		msvdx_priv->tile_region_start0 =
			dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].gpu_offset;

		msvdx_priv->tile_region_end0 = msvdx_priv->tile_region_start0 +
		(dev_priv->bdev.man[DRM_PSB_MEM_MMU_TILING].size << PAGE_SHIFT);

		msvdx_priv->tile_region_start1 =
			dev_priv->bdev.man[TTM_PL_TT].gpu_offset;

		msvdx_priv->tile_region_end1 = msvdx_priv->tile_region_start1 +
		(dev_priv->bdev.man[TTM_PL_TT].size << PAGE_SHIFT);

		drm_psb_msvdx_tiling = 0;
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

		msvdx_priv->msvdx_ec_ctx[0] =
			kzalloc(sizeof(struct psb_msvdx_ec_ctx) *
					PSB_MAX_EC_INSTANCE,
					GFP_KERNEL);
		if (msvdx_priv->msvdx_ec_ctx[0] == NULL)
			DRM_ERROR("MSVDX:fail to allocate memory for ec ctx\n");
		else {
			int i;
			for (i = 1; i < PSB_MAX_EC_INSTANCE; i++)
				msvdx_priv->msvdx_ec_ctx[i] =
					msvdx_priv->msvdx_ec_ctx[0] + i;
			for (i = 0; i < PSB_MAX_EC_INSTANCE; i++)
				msvdx_priv->msvdx_ec_ctx[i]->fence =
						PSB_MSVDX_INVALID_FENCE;
		}
		INIT_WORK(&(msvdx_priv->ec_work), psb_msvdx_do_concealment);
		INIT_DELAYED_WORK(&msvdx_priv->msvdx_suspend_wq,
					&psb_powerdown_msvdx);
	}

	msvdx_priv = dev_priv->msvdx_private;
	if (!msvdx_priv->ccb0) { /* one for the first time */
		/* Initialize comand msvdx queueing */
		INIT_LIST_HEAD(&msvdx_priv->msvdx_queue);
		mutex_init(&msvdx_priv->msvdx_mutex);
		spin_lock_init(&msvdx_priv->msvdx_lock);
		/*figure out the stepping */
		pci_read_config_byte(dev->pdev, PSB_REVID_OFFSET, &psb_rev_id);
	}

	msvdx_priv->vec_local_mem_size = VEC_LOCAL_MEM_BYTE_SIZE;
	if (!msvdx_priv->vec_local_mem_data) {
		msvdx_priv->vec_local_mem_data =
			kmalloc(msvdx_priv->vec_local_mem_size, GFP_KERNEL);
		if (!msvdx_priv->vec_local_mem_data) {
			PSB_DEBUG_GENERAL("Allocate vec local mem data fail\n");
			goto err_exit;
		}
		memset(msvdx_priv->vec_local_mem_data, 0, msvdx_priv->vec_local_mem_size);
	}

	msvdx_priv->msvdx_busy = 0;
	msvdx_priv->msvdx_hw_busy = 1;

	/* Enable Clocks */
	PSB_DEBUG_GENERAL("Enabling clocks\n");
	PSB_WMSVDX32(clk_enable_all, MSVDX_MAN_CLK_ENABLE);


	/* Issue software reset for all but core*/
	/*
		PSB_WMSVDX32((uint32_t) ~MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK, REGISTER(MSVDX_CORE, CR_MSVDX_CONTROL));
		reg_value = PSB_RMSVDX32(REGISTER(MSVDX_CORE, CR_MSVDX_CONTROL));
		PSB_WMSVDX32(0, REGISTER(MSVDX_CORE, CR_MSVDX_CONTROL));
		PSB_WMSVDX32(MSVDX_CORE_CR_MSVDX_CONTROL_CR_MSVDX_SOFT_RESET_MASK, REGISTER(MSVDX_CORE, CR_MSVDX_CONTROL));

		reg_val = 0;
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CNT_CTRL, 0x3);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL, FE_WDT_ENABLE, 0);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL, FE_WDT_ACTION0, 1);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CLEAR_SELECT, 1);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_FE_MSVDX_WDT_CONTROL, FE_WDT_CLKDIV_SELECT, 7);
		PSB_WMSVDX32(820, REGISTER( MSVDX_CORE, CR_FE_MSVDX_WDT_COMPAREMATCH ));
		PSB_WMSVDX32(reg_val, REGISTER( MSVDX_CORE, CR_FE_MSVDX_WDT_CONTROL ));

		reg_val = 0;
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CNT_CTRL, 0x7);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL, BE_WDT_ENABLE, 0);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL, BE_WDT_ACTION0, 1);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CLEAR_SELECT, 0xd);
		REGIO_WRITE_FIELD(reg_val, MSVDX_CORE_CR_BE_MSVDX_WDT_CONTROL, BE_WDT_CLKDIV_SELECT, 7);
		PSB_WMSVDX32(8200, REGISTER(MSVDX_CORE, CR_BE_MSVDX_WDT_COMPAREMATCH));
		PSB_WMSVDX32(reg_val, REGISTER(MSVDX_CORE, CR_BE_MSVDX_WDT_CONTROL));
	*/
	if (!IS_D0(dev)) {
		/* Enable MMU by removing all bypass bits */
		PSB_WMSVDX32(0, MSVDX_MMU_CONTROL0);
	}
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
		PSB_DEBUG_GENERAL("       Region 0x%08x-0x%08x\n",
					tile_start, tile_end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE0);

		tile_start =
			dev_priv->bdev.man[TTM_PL_TT].gpu_offset;
		tile_end = tile_start +
		(dev_priv->bdev.man[TTM_PL_TT].size << PAGE_SHIFT);

		cmd = ((tile_start >> 20) + (((tile_end >> 20) - 1) << 12) +
					((0x8 | 2) << 24)); /* 2k stride */

		PSB_DEBUG_GENERAL("MSVDX: MMU Tiling register1 %08x\n", cmd);
		PSB_DEBUG_GENERAL("       Region 0x%08x-0x%08x\n",
					tile_start, tile_end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE1);
	}
#endif

	/* move firmware loading to the place receiving first command buffer */

	PSB_DEBUG_GENERAL("MSVDX: Setting up RENDEC,allocate CCB 0/1\n");
	/* Allocate device virtual memory as required by rendec.... */
	if (!msvdx_priv->ccb0) {
		ret = psb_allocate_ccb(dev, &msvdx_priv->ccb0,
				       &msvdx_priv->base_addr0,
				       RENDEC_A_SIZE);
		if (ret) {
			PSB_DEBUG_GENERAL("Allocate Rendec A fail\n");
			goto err_exit;
		}
	}

	if (!msvdx_priv->ccb1) {
		ret = psb_allocate_ccb(dev, &msvdx_priv->ccb1,
				       &msvdx_priv->base_addr1,
				       RENDEC_B_SIZE);
		if (ret)
			goto err_exit;
	}

	if (!msvdx_priv->fw) {
		uint32_t core_rev;
		uint32_t fw_bo_size;

		core_rev = PSB_RMSVDX32(MSVDX_CORE_REV);

		if ((core_rev & 0xffffff) < 0x020000)
			msvdx_priv->mtx_mem_size = 16 * 1024;
		else
			msvdx_priv->mtx_mem_size = 40 * 1024;

		fw_bo_size =  msvdx_priv->mtx_mem_size + 4096;

		PSB_DEBUG_INIT("MSVDX: MTX mem size is 0x%08xbytes  allocate firmware BO size 0x%08x\n", msvdx_priv->mtx_mem_size,
			       fw_bo_size);

		ret = ttm_buffer_object_create(&dev_priv->bdev, fw_bo_size, /* DMA may run over a page */
					       ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU | TTM_PL_FLAG_NO_EVICT,
					       0, 0, 0, NULL, &msvdx_priv->fw);

		if (ret) {
			PSB_DEBUG_GENERAL("Allocate firmware BO fail\n");
			goto err_exit;
		}
	}

	PSB_DEBUG_GENERAL("MSVDX: RENDEC A: %08x RENDEC B: %08x\n",
			  msvdx_priv->base_addr0, msvdx_priv->base_addr1);
	if (!IS_D0(dev)) {
		PSB_WMSVDX32(msvdx_priv->base_addr0, MSVDX_RENDEC_BASE_ADDR0);
		PSB_WMSVDX32(msvdx_priv->base_addr1, MSVDX_RENDEC_BASE_ADDR1);

		cmd = 0;
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_BUFFER_SIZE,
				RENDEC_BUFFER_SIZE0, RENDEC_A_SIZE / 4096);
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_BUFFER_SIZE,
				RENDEC_BUFFER_SIZE1, RENDEC_B_SIZE / 4096);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_BUFFER_SIZE);
	}
	if (!msvdx_priv->fw) {
		uint32_t core_rev;

		core_rev = PSB_RMSVDX32(MSVDX_CORE_REV);

		if ((core_rev & 0xffffff) < 0x020000)
			msvdx_priv->mtx_mem_size = 16 * 1024;
		else
			msvdx_priv->mtx_mem_size = 40 * 1024;

		PSB_DEBUG_INIT("MSVDX: MTX mem size is 0x%08xbytes  allocate firmware BO size 0x%08x\n", msvdx_priv->mtx_mem_size,
			       msvdx_priv->mtx_mem_size + 4096);

		ret = ttm_buffer_object_create(&dev_priv->bdev, msvdx_priv->mtx_mem_size + 4096, /* DMA may run over a page */
					       ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU | TTM_PL_FLAG_NO_EVICT,
					       0, 0, 0, NULL, &msvdx_priv->fw);

		if (ret) {
			PSB_DEBUG_GENERAL("Allocate firmware BO fail\n");
			goto err_exit;
		}
	}
	if (!IS_D0(dev)) {
		cmd = 0;
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
				RENDEC_DECODE_START_SIZE, 0);
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
				RENDEC_BURST_SIZE_W, 1);
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
				RENDEC_BURST_SIZE_R, 1);
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL1,
				RENDEC_EXTERNAL_MEMORY, 1);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL1);

		cmd = 0x00101010;
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT0);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT1);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT2);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT3);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT4);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTEXT5);

		cmd = 0;
		REGIO_WRITE_FIELD(cmd, MSVDX_RENDEC_CONTROL0, RENDEC_INITIALISE,
				1);
		PSB_WMSVDX32(cmd, MSVDX_RENDEC_CONTROL0);
	}

	/* PSB_WMSVDX32(clk_enable_minimal, MSVDX_MAN_CLK_ENABLE); */
	PSB_DEBUG_INIT("MSVDX:defer firmware loading to the"
		       " place when receiving user space commands\n");

	msvdx_priv->msvdx_fw_loaded = 0; /* need to load firware */

	if (!IS_D0(dev)) {
		/* it should be set at punit post boot init phase */
		PSB_WMSVDX32(820, MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH);
		PSB_WMSVDX32(8200, MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH);

		PSB_WMSVDX32(820, MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH);
		PSB_WMSVDX32(8200, MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH);
	} else {
		/* for the other two, use the default value punit set */
		PSB_WMSVDX32(0x334, MSVDX_CORE_CR_FE_MSVDX_WDT_COMPAREMATCH);
		PSB_WMSVDX32(0x2008, MSVDX_CORE_CR_BE_MSVDX_WDT_COMPAREMATCH);
	}

	psb_msvdx_clearirq(dev);
	psb_msvdx_enableirq(dev);

	PSB_DEBUG_INIT("MSDVX:old clock gating disable = 0x%08x\n",
		       PSB_RVDC32(PSB_MSVDX_CLOCKGATING));

	if (!IS_D0(dev)) {
		cmd = 0;
		cmd = PSB_RMSVDX32(MSVDX_VEC_SHIFTREG_CONTROL); /* VEC_SHIFTREG_CONTROL */
		REGIO_WRITE_FIELD(cmd,
				  VEC_SHIFTREG_CONTROL,
				  SR_MASTER_SELECT,
				  1);  /* Host */
		PSB_WMSVDX32(cmd, MSVDX_VEC_SHIFTREG_CONTROL);
	}


#if 0
	ret = psb_setup_fw(dev);
	if (ret)
		goto err_exit;
	/* Send Initialisation message to firmware */
	if (0) {
		uint32_t msg_init[FW_VA_INIT_SIZE >> 2];
		MEMIO_WRITE_FIELD(msg_init, FWRK_GENMSG_SIZE,
				  FW_VA_INIT_SIZE);
		MEMIO_WRITE_FIELD(msg_init, FWRK_GENMSG_ID, VA_MSGID_INIT);

		/* Need to set this for all but A0 */
		MEMIO_WRITE_FIELD(msg_init, FW_VA_INIT_GLOBAL_PTD,
				  psb_get_default_pd_addr(dev_priv->mmu));

		ret = psb_mtx_send(dev_priv, msg_init);
		if (ret)
			goto err_exit;

		psb_poll_mtx_irq(dev_priv);
	}
#endif

	return 0;

err_exit:
	DRM_ERROR("MSVDX: initialization failed\n");
	if (msvdx_priv && msvdx_priv->ccb0)
		psb_free_ccb(&msvdx_priv->ccb0);
	if (msvdx_priv && msvdx_priv->ccb1)
		psb_free_ccb(&msvdx_priv->ccb1);
	kfree(dev_priv->msvdx_private);

	return 1;
}

int psb_msvdx_uninit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/* Reset MSVDX chip */
	psb_msvdx_reset(dev_priv);

	/* PSB_WMSVDX32 (clk_enable_minimal, MSVDX_MAN_CLK_ENABLE); */
	PSB_DEBUG_INIT("MSVDX:set the msvdx clock to 0\n");
	PSB_WMSVDX32(0, MSVDX_MAN_CLK_ENABLE);

	if (NULL == msvdx_priv) {
		DRM_ERROR("MSVDX: psb_msvdx_uninit: msvdx_priv is NULL!\n");
		return -1;
	}

	if (msvdx_priv->ccb0)
		psb_free_ccb(&msvdx_priv->ccb0);
	if (msvdx_priv->ccb1)
		psb_free_ccb(&msvdx_priv->ccb1);
	if (msvdx_priv->msvdx_fw)
		kfree(msvdx_priv->msvdx_fw
		     );
	if (msvdx_priv->vec_local_mem_data)
		kfree(msvdx_priv->vec_local_mem_data);

	kfree(msvdx_priv->msvdx_ec_ctx[0]);

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
