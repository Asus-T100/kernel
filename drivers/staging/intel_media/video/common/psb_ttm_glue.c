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
#include "psb_video_drv.h"
#include "psb_ttm_userobj_api.h"
#include <linux/io.h>
#include <asm/intel-mid.h>
#include "psb_msvdx.h"
#include "pnw_topaz.h"

/*IMG Headers*/
#include "private_data.h"

static int ied_enabled;

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
int psb_ttm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct ttm_buffer_object *bo = (struct ttm_buffer_object *)
				       vma->vm_private_data;
	struct drm_psb_private *dev_priv =
		container_of(bo->bdev, struct drm_psb_private, bdev);
	int ret;

	ret = ttm_read_lock(&dev_priv->ttm_lock, true);
	if (unlikely(ret != 0))
		return ret;

	ret = dev_priv->ttm_vm_ops->fault(vma, vmf);

	ttm_read_unlock(&dev_priv->ttm_lock);
	return ret;
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

int psb_getpageaddrs_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_psb_getpageaddrs_arg *arg = data;
	struct ttm_buffer_object *bo;
	struct ttm_tt *ttm;
	struct page **tt_pages;
	unsigned long i, num_pages;
	unsigned long *p = arg->page_addrs;

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

	return 0;
}

void psb_remove_videoctx(struct drm_psb_private *dev_priv, struct file *filp)
{
	struct psb_video_ctx *pos, *n;
	struct psb_video_ctx *found_ctx = NULL;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	/* iterate to query all ctx to if there is DRM running*/
	ied_enabled = 0;

	mutex_lock(&dev_priv->video_ctx_mutex);
	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if (pos->filp == filp) {
			found_ctx = pos;
			list_del(&pos->head);
		} else {
			if (pos->ctx_type & VA_RT_FORMAT_PROTECTED)
				ied_enabled = 1;
		}
	}
	mutex_unlock(&dev_priv->video_ctx_mutex);

	if (found_ctx) {
		PSB_DEBUG_PM("Video:remove context profile %d,"
				  " entrypoint %d\n",
				  (found_ctx->ctx_type >> 8) & 0xff,
				  (found_ctx->ctx_type & 0xff));
		/* if current ctx points to it, set to NULL */
		if (VAEntrypointEncSlice ==
				(found_ctx->ctx_type & 0xff)
			|| VAEntrypointEncPicture ==
				(found_ctx->ctx_type & 0xff)) {
			if (dev_priv->topaz_ctx == found_ctx) {
				pnw_reset_fw_status(dev_priv->dev,
					PNW_TOPAZ_END_CTX);
				dev_priv->topaz_ctx = NULL;
			} else {
				PSB_DEBUG_PM("Remove a inactive "\
						"encoding context.\n");
			}
			if (dev_priv->last_topaz_ctx == found_ctx)
				dev_priv->last_topaz_ctx = NULL;
		} else {
			mutex_lock(&msvdx_priv->msvdx_mutex);
			if (msvdx_priv->msvdx_ctx == found_ctx)
				msvdx_priv->msvdx_ctx = NULL;
			if (msvdx_priv->last_msvdx_ctx == found_ctx)
				msvdx_priv->last_msvdx_ctx = NULL;
			mutex_unlock(&msvdx_priv->msvdx_mutex);
		}
		kfree(found_ctx);
	}
}

#ifdef PSB_MSVDX_TILE_SUPPORT
static struct psb_video_ctx *psb_find_videoctx(struct drm_psb_private *dev_priv,
						struct file *filp)
{
	struct psb_video_ctx *pos, *n;

	mutex_lock(&dev_priv->video_ctx_mutex);
	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if (pos->filp == filp) {
			mutex_unlock(&dev_priv->video_ctx_mutex);
			return pos;
		}
	}
	mutex_unlock(&dev_priv->video_ctx_mutex);
	return NULL;
}
#endif

static int psb_entrypoint_number(struct drm_psb_private *dev_priv,
		uint32_t entry_type)
{
	struct psb_video_ctx *pos, *n;
	int count = 0;

	entry_type &= 0xff;

	if (entry_type < VAEntrypointVLD ||
			entry_type > VAEntrypointEncPicture) {
		DRM_ERROR("Invalide entrypoint value %d.\n", entry_type);
		return -EINVAL;
	}

	mutex_lock(&dev_priv->video_ctx_mutex);
	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if (entry_type == (pos->ctx_type & 0xff))
			count++;
	}
	mutex_unlock(&dev_priv->video_ctx_mutex);

	PSB_DEBUG_GENERAL("There are %d active entrypoint %d.\n",
			count, entry_type);
	return count;
}

int psb_video_getparam(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct drm_lnc_video_getparam_arg *arg = data;
	int ret = 0;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)file_priv->minor->dev->dev_private;
	drm_psb_msvdx_frame_info_t *current_frame = NULL;
	uint32_t handle, i;
	uint32_t device_info = 0;
	uint32_t ctx_type = 0;
	struct psb_video_ctx *video_ctx = NULL;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t imr_info[2], ci_info[2];

	switch (arg->key) {
	case LNC_VIDEO_GETPARAM_IMR_INFO:
		imr_info[0] = dev_priv->imr_region_start;
		imr_info[1] = dev_priv->imr_region_size;
		ret = copy_to_user((void __user *)((unsigned long)arg->value),
				   &imr_info[0],
				   sizeof(imr_info));
		break;
	case LNC_VIDEO_DEVICE_INFO:
		device_info = 0xffff & dev_priv->video_device_fuse;
		device_info |= (0xffff & dev->pci_device) << 16;

		ret = copy_to_user((void __user *)((unsigned long)arg->value),
				   &device_info, sizeof(device_info));
		break;
	case IMG_VIDEO_NEW_CONTEXT:
		/* add video decode/encode context */
		ret = copy_from_user(&ctx_type, (void __user *)((unsigned long)arg->value),
				     sizeof(ctx_type));
		if (ret)
			break;

		video_ctx = kmalloc(sizeof(struct psb_video_ctx), GFP_KERNEL);
		if (video_ctx == NULL) {
			ret = -ENOMEM;
			break;
		}
		INIT_LIST_HEAD(&video_ctx->head);
		video_ctx->ctx_type = ctx_type;
		video_ctx->filp = file_priv->filp;
		mutex_lock(&dev_priv->video_ctx_mutex);
		list_add(&video_ctx->head, &dev_priv->video_ctx);
		mutex_unlock(&dev_priv->video_ctx_mutex);

		if (IS_MDFLD(dev_priv->dev) &&
				(VAEntrypointEncSlice ==
				 (ctx_type & 0xff)))
			pnw_reset_fw_status(dev_priv->dev,
				PNW_TOPAZ_START_CTX);

		PSB_DEBUG_INIT("Video:add ctx profile %d, entry %d.\n",
					((ctx_type >> 8) & 0xff),
					(ctx_type & 0xff));
		PSB_DEBUG_INIT("Video:add context protected 0x%x.\n",
					(ctx_type & VA_RT_FORMAT_PROTECTED));
		if (ctx_type & VA_RT_FORMAT_PROTECTED)
			ied_enabled = 1;
		break;
	case IMG_VIDEO_RM_CONTEXT:
		psb_remove_videoctx(dev_priv, file_priv->filp);
		break;
#ifdef PSB_MSVDX_TILE_SUPPORT
	case IMG_VIDEO_UPDATE_CONTEXT:
		ret = copy_from_user(&ctx_type,
				(void __user *)((unsigned long)arg->value),
				sizeof(ctx_type));
		if (ret)
			break;
		video_ctx = psb_find_videoctx(dev_priv, file_priv->filp);
		if (video_ctx) {
			PSB_DEBUG_GENERAL(
				"Video: update video ctx old value 0x%08x\n",
				video_ctx->ctx_type);
			video_ctx->ctx_type = ctx_type;
			PSB_DEBUG_GENERAL(
				"Video: update video ctx new value 0x%08x\n",
				video_ctx->ctx_type);
		} else
			PSB_DEBUG_GENERAL(
				"Video:fail to find context profile %d, entrypoint %d",
				(ctx_type >> 8), (ctx_type & 0xff));
		break;
#endif
	case IMG_VIDEO_DECODE_STATUS:
#ifdef CONFIG_VIDEO_MRFLD
		if (msvdx_priv->host_be_opp_enabled) {
			/*get the right frame_info struct for current surface*/
			ret = copy_from_user(&handle,
					     (void __user *)((unsigned long)arg->arg), 4);
			if (ret)
				break;

			for (i = 0; i < MAX_DECODE_BUFFERS; i++) {
				if (msvdx_priv->frame_info[i].handle == handle) {
					current_frame = &msvdx_priv->frame_info[i];
					break;
				}
			}
			if (!current_frame) {
				DRM_ERROR("MSVDX: didn't find frame_info which matched the surface_id. \n");
				ret = -EFAULT;
				break;
			}
			ret = copy_to_user((void __user *)((unsigned long)arg->value),
					   &current_frame->fw_status, sizeof(current_frame->fw_status));
		} else
#endif
		{
			ret = copy_to_user((void __user *)((unsigned long)arg->value),
					   &msvdx_priv->decoding_err, sizeof(msvdx_priv->decoding_err));
		}
		break;
#ifdef CONFIG_VIDEO_MRFLD
	case IMG_VIDEO_MB_ERROR:
		/*get the right frame_info struct for current surface*/
		ret = copy_from_user(&handle,
				     (void __user *)((unsigned long)arg->arg), 4);
		if (ret)
			break;

		for (i = 0; i < MAX_DECODE_BUFFERS; i++) {
			if (msvdx_priv->frame_info[i].handle == handle) {
				current_frame = &msvdx_priv->frame_info[i];
				break;
			}
		}
		if (!current_frame) {
			DRM_ERROR("MSVDX: didn't find frame_info which matched the surface_id. \n");
			ret = -EFAULT;
			break;
		}
		ret = copy_to_user((void __user *)((unsigned long)arg->value),
				   &(current_frame->decode_status), sizeof(drm_psb_msvdx_decode_status_t));
		break;
#endif
	case IMG_VIDEO_SET_DISPLAYING_FRAME:
		ret = copy_from_user(&msvdx_priv->displaying_frame,
				(void __user *)((unsigned long)arg->value),
				sizeof(msvdx_priv->displaying_frame));
		break;
	case IMG_VIDEO_GET_DISPLAYING_FRAME:
		ret = copy_to_user((void __user *)((unsigned long)arg->value),
				&msvdx_priv->displaying_frame,
				sizeof(msvdx_priv->displaying_frame));
		break;
	case IMG_VIDEO_GET_HDMI_STATE:
		ret = copy_to_user((void __user *)((unsigned long)arg->value),
				&hdmi_state,
				sizeof(hdmi_state));
		break;
	case IMG_VIDEO_SET_HDMI_STATE:
		if (!hdmi_state) {
			PSB_DEBUG_ENTRY(
				"wait 100ms for kernel hdmi pipe ready.\n");
			msleep(100);
		}
		if (dev_priv->bhdmiconnected)
			hdmi_state = (int)arg->value;
		else
			PSB_DEBUG_ENTRY(
				"skip hdmi_state setting, for unplugged.\n");

		PSB_DEBUG_ENTRY("%s, set hdmi_state = %d\n",
				 __func__, hdmi_state);
		break;
	case PNW_VIDEO_QUERY_ENTRY:
		ret = copy_from_user(&handle,
				(void __user *)((unsigned long)arg->arg),
				sizeof(handle));
		if (ret)
			break;
		/*Return the number of active entries*/
		i = psb_entrypoint_number(dev_priv, handle);
		if (i >= 0)
			ret = copy_to_user((void __user *)
					((unsigned long)arg->value),
					&i, sizeof(i));
		break;
	case IMG_VIDEO_IED_STATE:
		if (IS_MDFLD(dev)) {
			int enabled = dev_priv->ied_enabled ? 1 : 0;
			ret = copy_to_user((void __user *)
				((unsigned long)arg->value),
				&enabled, sizeof(enabled));
		} else {
			DRM_ERROR("IMG_VIDEO_IED_EANBLE error.\n");
			return -EFAULT;
		}
		break;

	default:
		ret = -EFAULT;
		break;
	}

out:
	if (ret) {
		DRM_ERROR("%s: failed to call sub-ioctl 0x%x",
			__func__, arg->key);
		return -EFAULT;
	}

	return 0;
}
