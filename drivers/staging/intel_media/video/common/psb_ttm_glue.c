/**************************************************************************
 * Copyright (c) 2008, Intel Corporation.
 * All Rights Reserved.
 * Copyright (c) 2008, Tungsten Graphics Inc.  Cedar Park, TX., USA.
 * All Rights Reserved.
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
#include "psb_drv.h"
#include "psb_ttm_userobj_api.h"
#include <linux/io.h>
#include <asm/intel-mid.h>
#include "psb_msvdx.h"
#include "pnw_topaz.h"

/*IMG Headers*/
#include "private_data.h"

extern int PVRMMap(struct file *pFile, struct vm_area_struct *ps_vma);

static struct vm_operations_struct psb_ttm_vm_ops;

/**
 * NOTE: driver_private of drm_file is now a PVRSRV_FILE_PRIVATE_DATA struct
 * pPriv in PVRSRV_FILE_PRIVATE_DATA contains the original psb_fpriv;
 */
int psb_open(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv;
	struct drm_psb_private *dev_priv;
	struct psb_fpriv *psb_fp;
	PVRSRV_FILE_PRIVATE_DATA *pvr_file_priv;
	int ret;

	DRM_DEBUG("\n");

	ret = drm_open(inode, filp);
	if (unlikely(ret))
		return ret;

	psb_fp = kzalloc(sizeof(*psb_fp), GFP_KERNEL);

	if (unlikely(psb_fp == NULL))
		goto out_err0;

	file_priv = (struct drm_file *) filp->private_data;

	/* In case that the local file priv has created a master,
	 * which has been referenced, even if it's not authenticated
	 * (non-root user). */
	if ((file_priv->minor->master)
		&& (file_priv->master == file_priv->minor->master)
		&& (!file_priv->is_master))
		file_priv->is_master = 1;

	dev_priv = psb_priv(file_priv->minor->dev);

	DRM_DEBUG("is_master %d\n", file_priv->is_master ? 1 : 0);

	psb_fp->tfile = ttm_object_file_init(dev_priv->tdev,
					     PSB_FILE_OBJECT_HASH_ORDER);
	psb_fp->bcd_index = -1;
	if (unlikely(psb_fp->tfile == NULL))
		goto out_err1;

	pvr_file_priv = (PVRSRV_FILE_PRIVATE_DATA *)file_priv->driver_priv;
	if (!pvr_file_priv) {
		DRM_ERROR("drm file private is NULL\n");
		goto out_err1;
	}

	pvr_file_priv->pPriv = psb_fp;
	if (unlikely(dev_priv->bdev.dev_mapping == NULL))
		dev_priv->bdev.dev_mapping = dev_priv->dev->dev_mapping;

	return 0;

out_err1:
	kfree(psb_fp);
out_err0:
	(void) drm_release(inode, filp);
	return ret;
}

int psb_release(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv;
	struct psb_fpriv *psb_fp;
	struct drm_psb_private *dev_priv;
	struct msvdx_private *msvdx_priv;
	int ret, i;
	struct psb_msvdx_ec_ctx *ec_ctx;
	uint32_t ui32_reg_value = 0;
	file_priv = (struct drm_file *) filp->private_data;
	struct ttm_object_file *tfile = psb_fpriv(file_priv)->tfile;
	psb_fp = psb_fpriv(file_priv);
	dev_priv = psb_priv(file_priv->minor->dev);

#ifdef CONFIG_MDFD_VIDEO_DECODE

	msvdx_priv = (struct msvdx_private *)dev_priv->msvdx_private;

	/*cleanup for msvdx*/
	/*
	if (msvdx_priv->tfile == psb_fpriv(file_priv)->tfile) {
		msvdx_priv->fw_status = 0;
		msvdx_priv->host_be_opp_enabled = 0;
		msvdx_priv->deblock_enabled = 0;
		memset(&msvdx_priv->frame_info, 0, sizeof(struct drm_psb_msvdx_frame_info) * MAX_DECODE_BUFFERS);
	}
	*/

	if (msvdx_priv->msvdx_ec_ctx[0] != NULL) {
		for (i = 0; i < PSB_MAX_EC_INSTANCE; i++) {
			if (msvdx_priv->msvdx_ec_ctx[i]->tfile == tfile)
				break;
		}

		if (i < PSB_MAX_EC_INSTANCE) {
			ec_ctx = msvdx_priv->msvdx_ec_ctx[i];
			printk(KERN_DEBUG "remove ec ctx with tfile 0x%08x\n",
			       ec_ctx->tfile);
			ec_ctx->tfile = NULL;
			ec_ctx->fence = PSB_MSVDX_INVALID_FENCE;
		}
	}

	ttm_object_file_release(&psb_fp->tfile);
#endif
	kfree(psb_fp);

#ifdef CONFIG_MDFD_VIDEO_DECODE
	/* remove video context */
	psb_remove_videoctx(dev_priv, filp);

	if (IS_MRST(dev_priv->dev)) {
		/*
		schedule_delayed_work(&dev_priv->scheduler.topaz_suspend_wq, 10);
		*/
		/* FIXME: workaround for HSD3469585
		 *        re-enable DRAM Self Refresh Mode
		 *        by setting DUNIT.DPMC0
		 */
		ui32_reg_value = intel_mid_msgbus_read32_raw((0xD0 << 24) |
			(0x1 << 16) | (0x4 << 8) | 0xF0);
		intel_mid_msgbus_write32_raw((0xE0 << 24) | (0x1 << 16) |
			(0x4 << 8) | 0xF0, ui32_reg_value | (0x1 << 7));
	} else if (IS_MDFLD(dev_priv->dev)) {
		struct pnw_topaz_private *topaz_priv =
			(struct pnw_topaz_private *)dev_priv->topaz_private;
		schedule_delayed_work(&topaz_priv->topaz_suspend_wq,
						msecs_to_jiffies(10));
	}
	schedule_delayed_work(&msvdx_priv->msvdx_suspend_wq,
					msecs_to_jiffies(10));
#endif
	ret = drm_release(inode, filp);

	return ret;
}

#ifdef CONFIG_MDFD_VIDEO_DECODE
int psb_fence_signaled_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file_priv)
{

	return ttm_fence_signaled_ioctl(psb_fpriv(file_priv)->tfile, data);
}

int psb_fence_finish_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	return ttm_fence_finish_ioctl(psb_fpriv(file_priv)->tfile, data);
}

int psb_fence_unref_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	return ttm_fence_unref_ioctl(psb_fpriv(file_priv)->tfile, data);
}

int psb_pl_waitidle_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	return ttm_pl_waitidle_ioctl(psb_fpriv(file_priv)->tfile, data);
}

int psb_pl_setstatus_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	return ttm_pl_setstatus_ioctl(psb_fpriv(file_priv)->tfile,
				      &psb_priv(dev)->ttm_lock, data);

}

int psb_pl_synccpu_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	return ttm_pl_synccpu_ioctl(psb_fpriv(file_priv)->tfile, data);
}

int psb_pl_unref_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	return ttm_pl_unref_ioctl(psb_fpriv(file_priv)->tfile, data);

}

int psb_pl_reference_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	return  ttm_pl_reference_ioctl(psb_fpriv(file_priv)->tfile, data);

}

int psb_pl_create_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);

	return ttm_pl_create_ioctl(psb_fpriv(file_priv)->tfile,
				   &dev_priv->bdev, &dev_priv->ttm_lock, data);

}

int psb_pl_ub_create_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);

	return ttm_pl_ub_create_ioctl(psb_fpriv(file_priv)->tfile,
				      &dev_priv->bdev, &dev_priv->ttm_lock, data);

}
/**
 * psb_ttm_fault - Wrapper around the ttm fault method.
 *
 * @vma: The struct vm_area_struct as in the vm fault() method.
 * @vmf: The struct vm_fault as in the vm fault() method.
 *
 * Since ttm_fault() will reserve buffers while faulting,
 * we need to take the ttm read lock around it, as this driver
 * relies on the ttm_lock in write mode to exclude all threads from
 * reserving and thus validating buffers in aperture- and memory shortage
 * situations.
 */

static int psb_ttm_fault(struct vm_area_struct *vma,
			 struct vm_fault *vmf)
{
	struct ttm_buffer_object *bo = (struct ttm_buffer_object *)
				       vma->vm_private_data;
	struct drm_psb_private *dev_priv =
		container_of(bo->bdev, struct drm_psb_private, bdev);
	int ret;

	ret = ttm_read_lock(&dev_priv->ttm_lock, true);
	if (unlikely(ret != 0))
		return VM_FAULT_NOPAGE;

	ret = dev_priv->ttm_vm_ops->fault(vma, vmf);

	ttm_read_unlock(&dev_priv->ttm_lock);
	return ret;
}
#endif

/**
 * if vm_pgoff < DRM_PSB_FILE_PAGE_OFFSET call directly to
 * PVRMMap
 */
int psb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file_priv;
	struct drm_psb_private *dev_priv;
	int ret;

	if (vma->vm_pgoff < DRM_PSB_FILE_PAGE_OFFSET ||
	    vma->vm_pgoff > 2 * DRM_PSB_FILE_PAGE_OFFSET)
		return PVRMMap(filp, vma);

	file_priv = (struct drm_file *) filp->private_data;
	dev_priv = psb_priv(file_priv->minor->dev);

	ret = ttm_bo_mmap(filp, vma, &dev_priv->bdev);
	if (unlikely(ret != 0))
		return ret;

	if (unlikely(dev_priv->ttm_vm_ops == NULL)) {
		dev_priv->ttm_vm_ops = (struct vm_operations_struct *)vma->vm_ops;
		psb_ttm_vm_ops = *vma->vm_ops;
#ifdef CONFIG_MDFD_VIDEO_DECODE
		psb_ttm_vm_ops.fault = &psb_ttm_fault;
#endif
	}

	vma->vm_ops = &psb_ttm_vm_ops;

	return 0;
}

/*
ssize_t psb_ttm_write(struct file *filp, const char __user *buf,
		      size_t count, loff_t *f_pos)
{
	struct drm_file *file_priv = (struct drm_file *)filp->private_data;
	struct drm_psb_private *dev_priv = psb_priv(file_priv->minor->dev);

	return ttm_bo_io(&dev_priv->bdev, filp, buf, NULL, count, f_pos, 1);
}

ssize_t psb_ttm_read(struct file *filp, char __user *buf,
		     size_t count, loff_t *f_pos)
{
	struct drm_file *file_priv = (struct drm_file *)filp->private_data;
	struct drm_psb_private *dev_priv = psb_priv(file_priv->minor->dev);

	return ttm_bo_io(&dev_priv->bdev, filp, NULL, buf, count, f_pos, 1);
}
*/
int psb_verify_access(struct ttm_buffer_object *bo,
		      struct file *filp)
{
	struct drm_file *file_priv = (struct drm_file *)filp->private_data;

	if (capable(CAP_SYS_ADMIN))
		return 0;

	/* workaround drm authentification issue on Android for ttm_bo_mmap */
	/*
	if (unlikely(!file_priv->authenticated))
		return -EPERM;
	*/
#ifdef CONFIG_MDFD_VIDEO_DECODE
	return ttm_pl_verify_access(bo, psb_fpriv(file_priv)->tfile);
#else
	return 0;
#endif
}

#ifdef CONFIG_MDFD_VIDEO_DECODE
static int psb_ttm_mem_global_init(struct drm_global_reference *ref)
{
	return ttm_mem_global_init(ref->object);
}

static void psb_ttm_mem_global_release(struct drm_global_reference *ref)
{
	ttm_mem_global_release(ref->object);
}

int psb_ttm_global_init(struct drm_psb_private *dev_priv)
{
	struct drm_global_reference *global_ref;
	struct drm_global_reference *global;
	int ret;

	global_ref = &dev_priv->mem_global_ref;
	global_ref->global_type = DRM_GLOBAL_TTM_MEM;
	global_ref->size = sizeof(struct ttm_mem_global);
	global_ref->init = &psb_ttm_mem_global_init;
	global_ref->release = &psb_ttm_mem_global_release;

	ret = drm_global_item_ref(global_ref);
	if (unlikely(ret != 0)) {
		DRM_ERROR("Failed referencing a global TTM memory object.\n");
		return ret;
	}

	dev_priv->bo_global_ref.mem_glob = dev_priv->mem_global_ref.object;
	global = &dev_priv->bo_global_ref.ref;
	global->global_type = DRM_GLOBAL_TTM_BO;
	global->size = sizeof(struct ttm_bo_global);
	global->init = &ttm_bo_global_init;
	global->release = &ttm_bo_global_release;
	ret = drm_global_item_ref((struct drm_global_reference *)global);
	if (ret != 0) {
		DRM_ERROR("Failed setting up TTM BO subsystem.\n");
		drm_global_item_unref((struct drm_global_reference *)global_ref);
		return ret;
	}

	return 0;
}

void psb_ttm_global_release(struct drm_psb_private *dev_priv)
{
	drm_global_item_unref(&dev_priv->mem_global_ref);
}
#endif

int psb_getpageaddrs_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_psb_getpageaddrs_arg *arg = data;
	struct ttm_buffer_object *bo;
	struct ttm_tt *ttm;
	struct page **tt_pages;
	unsigned long i, num_pages;
	unsigned long *p = arg->page_addrs;
	int ret = 0;

#ifdef CONFIG_MDFD_VIDEO_DECODE
	bo = ttm_buffer_object_lookup(psb_fpriv(file_priv)->tfile,
				      arg->handle);
	if (unlikely(bo == NULL)) {
		printk(KERN_ERR
		       "Could not find buffer object for getpageaddrs.\n");
		return -EINVAL;
	}
	arg->gtt_offset = bo->offset;
	ttm = bo->ttm;
	num_pages = ttm->num_pages;
	tt_pages = ttm->pages;

	for (i = 0; i < num_pages; i++)
		p[i] = (unsigned long)page_to_phys(tt_pages[i]);

	ttm_bo_unref(&bo);
#endif
	return ret;
}
