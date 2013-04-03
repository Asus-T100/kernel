/**************************************************************************
 * MSVDX I/O operations and IRQ handling
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
#include "psb_drm.h"
#include "psb_drv.h"
#include "psb_msvdx.h"
#include "psb_msvdx_msg.h"
#include "psb_msvdx_reg.h"
#ifdef CONFIG_VIDEO_MRFLD
#include "psb_msvdx_ec.h"
#endif

#include "psb_powermgmt.h"
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/history_record.h>

#ifdef CONFIG_MDFD_GL3
#include "mdfld_gl3.h"
#endif

#ifndef list_first_entry
#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)
#endif

static int psb_msvdx_send(struct drm_device *dev, void *cmd,
			  unsigned long cmd_size);
static void psb_msvdx_set_tile(struct drm_device *dev,
				unsigned long msvdx_tile);
static int psb_msvdx_dequeue_send(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_msvdx_cmd_queue *msvdx_cmd = NULL;
	int ret = 0;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	unsigned long irq_flags;

	spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	if (list_empty(&msvdx_priv->msvdx_queue)) {
		PSB_DEBUG_GENERAL("MSVDXQUE: msvdx list empty.\n");
		msvdx_priv->msvdx_busy = 0;
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		return -EINVAL;
	}

	msvdx_cmd = list_first_entry(&msvdx_priv->msvdx_queue,
				     struct psb_msvdx_cmd_queue, head);
	list_del(&msvdx_cmd->head);
	spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);

	PSB_DEBUG_GENERAL("MSVDXQUE: Queue has id %08x\n", msvdx_cmd->sequence);
	if (IS_MSVDX_MEM_TILE(dev) && drm_psb_msvdx_tiling)
		psb_msvdx_set_tile(dev, msvdx_cmd->msvdx_tile);

#ifdef CONFIG_VIDEO_MRFLD_EC
	/* Seperate update frame and backup cmds because if a batch of cmds
	 * doesn't have * host_be_opp message, no need to update frame info
	 * but still need to backup cmds.
	 * This case can happen if an batch of cmds is not the entire frame
	*/
	if (msvdx_cmd->host_be_opp_enabled) {
		psb_msvdx_update_frame_info(msvdx_priv, msvdx_cmd->tfile,
			msvdx_cmd->cmd + msvdx_cmd->deblock_cmd_offset);
	}
	psb_msvdx_backup_cmd(msvdx_priv, msvdx_cmd->tfile,
			msvdx_cmd->cmd,
			msvdx_cmd->cmd_size,
			msvdx_cmd->deblock_cmd_offset);
#endif
	ret = psb_msvdx_send(dev, msvdx_cmd->cmd, msvdx_cmd->cmd_size);
	if (ret) {
		DRM_ERROR("MSVDXQUE: psb_msvdx_send failed\n");
		ret = -EINVAL;
	}

	kfree(msvdx_cmd->cmd);
	kfree(msvdx_cmd);

	return ret;
}

void psb_msvdx_flush_cmd_queue(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_msvdx_cmd_queue *msvdx_cmd;
	struct list_head *list, *next;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	unsigned long irq_flags;
	spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	/*Flush the msvdx cmd queue and signal all fences in the queue */
	list_for_each_safe(list, next, &msvdx_priv->msvdx_queue) {
		msvdx_cmd =
			list_entry(list, struct psb_msvdx_cmd_queue, head);
		list_del(list);
		PSB_DEBUG_GENERAL("MSVDXQUE: flushing sequence:0x%08x\n",
				  msvdx_cmd->sequence);
		msvdx_priv->msvdx_current_sequence = msvdx_cmd->sequence;
		psb_fence_error(dev, PSB_ENGINE_DECODE,
				msvdx_cmd->sequence,
				_PSB_FENCE_TYPE_EXE, DRM_CMD_HANG);
		kfree(msvdx_cmd->cmd);
		kfree(msvdx_cmd);
	}
	msvdx_priv->msvdx_busy = 0;
	spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
}

static int psb_msvdx_map_command(struct drm_device *dev,
				 struct ttm_buffer_object *cmd_buffer,
				 unsigned long cmd_offset, unsigned long cmd_size,
				 void **msvdx_cmd, uint32_t sequence, int copy_cmd)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	int ret = 0;
	unsigned long cmd_page_offset = cmd_offset & ~PAGE_MASK;
	unsigned long cmd_size_remaining;
	struct ttm_bo_kmap_obj cmd_kmap;
	void *cmd, *cmd_copy, *cmd_start;
	bool is_iomem;


	/* command buffers may not exceed page boundary */
	if ((cmd_size > PAGE_SIZE) || (cmd_size + cmd_page_offset > PAGE_SIZE))
		return -EINVAL;

	ret = ttm_bo_kmap(cmd_buffer, cmd_offset >> PAGE_SHIFT, 1, &cmd_kmap);
	if (ret) {
		DRM_ERROR("MSVDXQUE:ret:%d\n", ret);
		return ret;
	}

	cmd_start = (void *)ttm_kmap_obj_virtual(&cmd_kmap, &is_iomem)
		    + cmd_page_offset;
	cmd = cmd_start;
	cmd_size_remaining = cmd_size;

	msvdx_priv->host_be_opp_enabled = 0;
	msvdx_priv->deblock_cmd_offset = PSB_MSVDX_INVALID_OFFSET;

	while (cmd_size_remaining > 0) {
		if (cmd_size_remaining < MTX_GENMSG_HEADER_SIZE) {
			ret = -EINVAL;
			goto out;
		}
		uint32_t cur_cmd_size = MEMIO_READ_FIELD(cmd, MTX_GENMSG_SIZE);
		uint32_t cur_cmd_id = MEMIO_READ_FIELD(cmd, MTX_GENMSG_ID);
		uint32_t mmu_ptd = 0, msvdx_mmu_invalid = 0;

		PSB_DEBUG_GENERAL("cmd start at %08x cur_cmd_size = %d"
				  " cur_cmd_id = %02x fence = %08x\n",
				  (uint32_t) cmd, cur_cmd_size, cur_cmd_id, sequence);
		if ((cur_cmd_size % sizeof(uint32_t))
		    || (cur_cmd_size > cmd_size_remaining)) {
			ret = -EINVAL;
			DRM_ERROR("MSVDX: ret:%d\n", ret);
			goto out;
		}

		switch (cur_cmd_id) {
		case MTX_MSGID_DECODE_FE: {
			if (sizeof(struct fw_decode_msg) > cmd_size_remaining) {
				/* Msg size is not correct */
				ret = -EINVAL;
				PSB_DEBUG_MSVDX("MSVDX: wrong msg size.\n");
				goto out;
			}
			struct fw_decode_msg *decode_msg =
					(struct fw_decode_msg *)cmd;
			decode_msg->header.bits.msg_fence = sequence;

			mmu_ptd = psb_get_default_pd_addr(dev_priv->mmu);
			msvdx_mmu_invalid = atomic_cmpxchg(&dev_priv->msvdx_mmu_invaldc,
							   1, 0);
			if (msvdx_mmu_invalid == 1) {
				decode_msg->flag_size.bits.flags |=
						FW_INVALIDATE_MMU;
#ifdef CONFIG_MDFD_GL3
				gl3_invalidate();
#endif
				PSB_DEBUG_GENERAL("MSVDX:Set MMU invalidate\n");
			}
			/*
			if (msvdx_mmu_invalid == 1)
				psb_mmu_pgtable_dump(dev);
			*/

			decode_msg->mmu_context.bits.mmu_ptd = mmu_ptd >> 8;
			PSB_DEBUG_MSVDX("MSVDX: MSGID_DECODE_FE:"
					  " - fence: %08x"
					  " - flags: %08x - buffer_size: %08x"
					  " - crtl_alloc_addr: %08x"
					  " - context: %08x - mmu_ptd: %08x"
					  " - operating_mode: %08x.\n",
					  decode_msg->header.bits.msg_fence,
					  decode_msg->flag_size.bits.flags,
					  decode_msg->flag_size.bits.buffer_size,
					  decode_msg->crtl_alloc_addr,
					  decode_msg->mmu_context.bits.context,
					  decode_msg->mmu_context.bits.mmu_ptd,
					  decode_msg->operating_mode);
			break;
		}

#ifdef CONFIG_VIDEO_MRFLD
		case MTX_MSGID_HOST_BE_OPP_MFLD:
			msvdx_priv->host_be_opp_enabled = 1;
			msvdx_priv->deblock_cmd_offset =
					cmd_size - cmd_size_remaining;
#endif
		case MTX_MSGID_INTRA_OOLD_MFLD:
		case MTX_MSGID_DEBLOCK_MFLD: {
			if (sizeof(struct fw_deblock_msg) > cmd_size_remaining) {
				/* Msg size is not correct */
				ret = -EINVAL;
				PSB_DEBUG_MSVDX("MSVDX: wrong msg size.\n");
				goto out;
			}
			struct fw_deblock_msg *deblock_msg =
					(struct fw_deblock_msg *)cmd;
			mmu_ptd = psb_get_default_pd_addr(dev_priv->mmu);
			msvdx_mmu_invalid = atomic_cmpxchg(&dev_priv->msvdx_mmu_invaldc,
							   1, 0);
			if (msvdx_mmu_invalid == 1) {
				deblock_msg->flag_type.bits.flags |=
							FW_INVALIDATE_MMU;
				PSB_DEBUG_GENERAL("MSVDX:Set MMU invalidate\n");
			}

			/* patch to right cmd type */
			deblock_msg->header.bits.msg_type =
					cur_cmd_id -
					MTX_MSGID_DEBLOCK_MFLD +
					MTX_MSGID_DEBLOCK;

			deblock_msg->header.bits.msg_fence = (uint16_t)(sequence & 0xffff);
			deblock_msg->mmu_context.bits.mmu_ptd = (mmu_ptd >> 8);
			PSB_DEBUG_MSVDX("MSVDX: MSGID_DEBLOCK:"
				" - fence: %08x"
				" - flags: %08x - slice_field_type: %08x"
				" - operating_mode: %08x"
				" - context: %08x - mmu_ptd: %08x"
				" - frame_height_mb: %08x - pic_width_mb: %08x"
				" - address_a0: %08x - address_a1: %08x"
				" - mb_param_address: %08x"
				" - ext_stride_a: %08x"
				" - address_b0: %08x - address_b1: %08x"
				" - alt_output_flags_b: %08x.\n",
				deblock_msg->header.bits.msg_fence,
				deblock_msg->flag_type.bits.flags,
				deblock_msg->flag_type.bits.slice_field_type,
				deblock_msg->operating_mode,
				deblock_msg->mmu_context.bits.context,
				deblock_msg->mmu_context.bits.mmu_ptd,
				deblock_msg->pic_size.bits.frame_height_mb,
				deblock_msg->pic_size.bits.pic_width_mb,
				deblock_msg->address_a0,
				deblock_msg->address_a1,
				deblock_msg->mb_param_address,
				deblock_msg->ext_stride_a,
				deblock_msg->address_b0,
				deblock_msg->address_b1,
				deblock_msg->alt_output_flags_b);
			cmd += (sizeof(struct fw_deblock_msg) - cur_cmd_size);
			cmd_size_remaining -= (sizeof(struct fw_deblock_msg) -
						cur_cmd_size);
			break;
		}

		default:
			/* Msg not supported */
			ret = -EINVAL;
			PSB_DEBUG_GENERAL("MSVDX: ret:%d\n", ret);
			goto out;
		}

		cmd += cur_cmd_size;
		cmd_size_remaining -= cur_cmd_size;
		if (((sequence++) & 0xf) == 0xf) {
			ret = -EINVAL;
			PSB_DEBUG_GENERAL("MSVDX: too many cmds, abort\n");
			goto out;
		}
	}

	msvdx_priv->num_cmd = ((--sequence) & 0xf);

	if (copy_cmd) {
		PSB_DEBUG_GENERAL("MSVDXQUE:copying command\n");

		cmd_copy = kzalloc(cmd_size, GFP_KERNEL);
		if (cmd_copy == NULL) {
			ret = -ENOMEM;
			DRM_ERROR("MSVDX: fail to callc,ret=:%d\n", ret);
			goto out;
		}
		memcpy(cmd_copy, cmd_start, cmd_size);
		*msvdx_cmd = cmd_copy;
	} else {
		PSB_DEBUG_GENERAL("MSVDXQUE:did NOT copy command\n");
		if (IS_MSVDX_MEM_TILE(dev) && drm_psb_msvdx_tiling) {
			unsigned long msvdx_tile =
				((msvdx_priv->msvdx_ctx->ctx_type >> 16) & 0xff);
			psb_msvdx_set_tile(dev, msvdx_tile);
		}
#ifdef CONFIG_VIDEO_MRFLD_EC
		if (msvdx_priv->host_be_opp_enabled) {
			psb_msvdx_update_frame_info(msvdx_priv,
				msvdx_priv->tfile,
				cmd_start + msvdx_priv->deblock_cmd_offset);
		}
		psb_msvdx_backup_cmd(msvdx_priv, msvdx_priv->tfile,
				cmd_start,
				cmd_size,
				msvdx_priv->deblock_cmd_offset);
#endif
		ret = psb_msvdx_send(dev, cmd_start, cmd_size);
		if (ret) {
			DRM_ERROR("MSVDXQUE: psb_msvdx_send failed\n");
			ret = -EINVAL;
		}
	}

out:
	ttm_bo_kunmap(&cmd_kmap);

	return ret;
}

int psb_submit_video_cmdbuf(struct drm_device *dev,
			    struct ttm_buffer_object *cmd_buffer,
			    unsigned long cmd_offset, unsigned long cmd_size,
			    struct ttm_fence_object *fence,
			    struct psb_video_ctx *msvdx_ctx)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t sequence =  (dev_priv->sequence[PSB_ENGINE_DECODE] << 4);
	unsigned long irq_flags;
	int ret = 0;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	int offset = 0;

	spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);

	msvdx_priv->msvdx_ctx = msvdx_ctx;
	msvdx_priv->last_msvdx_ctx = msvdx_priv->msvdx_ctx;

	PSB_DEBUG_PM("sequence is 0x%x, needs_reset is 0x%x.\n",
			sequence, msvdx_priv->msvdx_needs_reset);
	if (msvdx_priv->msvdx_needs_reset) {
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		PSB_DEBUG_GENERAL("MSVDX: will reset msvdx\n");
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
		if (!msvdx_priv->fw_loaded_by_punit) {
			if (psb_msvdx_reset(dev_priv)) {
				ret = -EBUSY;
				DRM_ERROR("MSVDX: Reset failed\n");
				return ret;
			}
		}
#endif
		msvdx_priv->msvdx_needs_reset = 0;
		msvdx_priv->msvdx_busy = 0;

		if (psb_msvdx_init(dev)) {
			ret = -EBUSY;
			PSB_DEBUG_WARN("WARN: psb_msvdx_init failed.\n");
			return ret;
		}

#ifdef PSB_MSVDX_SAVE_RESTORE_VEC
		/* restore vec local mem if needed */
		if (msvdx_priv->vec_local_mem_saved) {
			for (offset = 0; offset < VEC_LOCAL_MEM_BYTE_SIZE / 4; ++offset)
				PSB_WMSVDX32(msvdx_priv->vec_local_mem_data[offset],
					     VEC_LOCAL_MEM_OFFSET + offset * 4);
			msvdx_priv->vec_local_mem_saved = 0;
		}
#endif

#ifdef CONFIG_VIDEO_MRFLD_EC
		/* restore the state when power up during EC */
		if (msvdx_priv->vec_ec_mem_saved) {
			for (offset = 0; offset < 4; ++offset)
				PSB_WMSVDX32(msvdx_priv->vec_ec_mem_data[offset],
					     0x2cb0 + offset * 4);
			msvdx_priv->vec_ec_mem_saved = 0;
		}
#endif

		spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	}

	if (msvdx_priv->fw_loaded_by_punit && !msvdx_priv->rendec_init) {
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		PSB_DEBUG_GENERAL("MSVDX:setup msvdx.\n");
		ret = psb_msvdx_post_boot_init(dev);
		if (ret) {
			DRM_ERROR("MSVDX:fail to setup msvdx.\n");
			/* FIXME: find a proper return value */
			return -EFAULT;
		}
		msvdx_priv->rendec_init = 1;

		PSB_DEBUG_GENERAL("MSVDX: setup msvdx successfully\n");
		spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	}

#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	if (!msvdx_priv->fw_loaded_by_punit && !msvdx_priv->msvdx_fw_loaded) {
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		PSB_DEBUG_GENERAL("MSVDX:reload FW to MTX\n");
		ret = psb_setup_fw(dev);
		if (ret) {
			DRM_ERROR("MSVDX:fail to load FW\n");
			/* FIXME: find a proper return value */
			return -EFAULT;
		}
		msvdx_priv->msvdx_fw_loaded = 1;

		PSB_DEBUG_GENERAL("MSVDX: load firmware successfully\n");
		spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	}
#endif
	if (!msvdx_priv->msvdx_busy) {
		msvdx_priv->msvdx_busy = 1;
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		PSB_DEBUG_GENERAL("MSVDX: commit command to HW,seq=0x%08x\n",
				  sequence);
		ret = psb_msvdx_map_command(dev, cmd_buffer, cmd_offset,
					    cmd_size, NULL, sequence, 0);
		if (ret) {
			DRM_ERROR("MSVDXQUE: Failed to extract cmd\n");
			return ret;
		}
	} else {
		struct psb_msvdx_cmd_queue *msvdx_cmd;
		void *cmd = NULL;

		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		/* queue the command to be sent when the h/w is ready */
		PSB_DEBUG_GENERAL("MSVDXQUE: queueing sequence:%08x..\n",
				  sequence);
		msvdx_cmd = kzalloc(sizeof(struct psb_msvdx_cmd_queue),
				    GFP_KERNEL);
		if (msvdx_cmd == NULL) {
			DRM_ERROR("MSVDXQUE: Out of memory...\n");
			return -ENOMEM;
		}

		ret = psb_msvdx_map_command(dev, cmd_buffer, cmd_offset,
					    cmd_size, &cmd, sequence, 1);
		if (ret) {
			DRM_ERROR("MSVDXQUE: Failed to extract cmd\n");
			kfree(msvdx_cmd
			     );
			return ret;
		}
		msvdx_cmd->cmd = cmd;
		msvdx_cmd->cmd_size = cmd_size;
		msvdx_cmd->sequence = sequence;

		msvdx_cmd->msvdx_tile =
			((msvdx_priv->msvdx_ctx->ctx_type >> 16) & 0xff);
		msvdx_cmd->deblock_cmd_offset =
			msvdx_priv->deblock_cmd_offset;
		msvdx_cmd->host_be_opp_enabled =
			msvdx_priv->host_be_opp_enabled;
		msvdx_cmd->tfile =
			msvdx_priv->tfile;
		spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
		list_add_tail(&msvdx_cmd->head, &msvdx_priv->msvdx_queue);
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		if (!msvdx_priv->msvdx_busy) {
			msvdx_priv->msvdx_busy = 1;
			PSB_DEBUG_GENERAL("MSVDXQUE: Need immediate dequeue\n");
			psb_msvdx_dequeue_send(dev);
		}
	}

	return ret;
}

int psb_cmdbuf_video(struct drm_file *priv,
		     struct list_head *validate_list,
		     uint32_t fence_type,
		     struct drm_psb_cmdbuf_arg *arg,
		     struct ttm_buffer_object *cmd_buffer,
		     struct psb_ttm_fence_rep *fence_arg,
		     struct psb_video_ctx *msvdx_ctx)
{
	struct drm_device *dev = priv->minor->dev;
	struct ttm_fence_object *fence;
	int ret;

	/*
	 * Check this. Doesn't seem right. Have fencing done AFTER command
	 * submission and make sure drm_psb_idle idles the MSVDX completely.
	 */
	ret = psb_submit_video_cmdbuf(dev, cmd_buffer, arg->cmdbuf_offset,
					arg->cmdbuf_size, NULL, msvdx_ctx);
	if (ret)
		return ret;


	/* DRM_ERROR("Intel: Fix video fencing!!\n"); */
	psb_fence_or_sync(priv, PSB_ENGINE_DECODE, fence_type,
			  arg->fence_flags, validate_list, fence_arg,
			  &fence);

	ttm_fence_object_unref(&fence);
	spin_lock(&cmd_buffer->bdev->fence_lock);
	if (cmd_buffer->sync_obj != NULL)
		ttm_fence_sync_obj_unref(&cmd_buffer->sync_obj);
	spin_unlock(&cmd_buffer->bdev->fence_lock);

	return 0;
}


static int psb_msvdx_send(struct drm_device *dev, void *cmd,
			  unsigned long cmd_size)
{
	int ret = 0;
	struct drm_psb_private *dev_priv = dev->dev_private;

	while (cmd_size > 0) {
		uint32_t cur_cmd_size = MEMIO_READ_FIELD(cmd, MTX_GENMSG_SIZE);
		uint32_t cur_cmd_id = MEMIO_READ_FIELD(cmd, MTX_GENMSG_ID);
		if (cur_cmd_size > cmd_size) {
			ret = -EINVAL;
			DRM_ERROR("MSVDX:cmd_size %lu cur_cmd_size %lu\n",
				  cmd_size, (unsigned long)cur_cmd_size);
			goto out;
		}

		/* Send the message to h/w */
		ret = psb_mtx_send(dev_priv, cmd);
		if (ret) {
			PSB_DEBUG_GENERAL("MSVDX: ret:%d\n", ret);
			goto out;
		}
		cmd += cur_cmd_size;
		cmd_size -= cur_cmd_size;
		if (cur_cmd_id == MTX_MSGID_HOST_BE_OPP ||
			cur_cmd_id == MTX_MSGID_DEBLOCK ||
			cur_cmd_id == MTX_MSGID_INTRA_OOLD) {
			cmd += (sizeof(struct fw_deblock_msg) - cur_cmd_size);
			cmd_size -=
				(sizeof(struct fw_deblock_msg) - cur_cmd_size);
		}
	}

out:
	PSB_DEBUG_GENERAL("MSVDX: ret:%d\n", ret);
	return ret;
}

int psb_mtx_send(struct drm_psb_private *dev_priv, const void *msg)
{
	static struct fw_padding_msg pad_msg;
	const uint32_t *p_msg = (uint32_t *) msg;
	uint32_t msg_num, words_free, ridx, widx, buf_size, buf_offset;
	int ret = 0;

	PSB_DEBUG_GENERAL("MSVDX: psb_mtx_send\n");

	/* we need clocks enabled before we touch VEC local ram,
	 * but fw will take care of the clock after fw is loaded
	 */

	msg_num = (MEMIO_READ_FIELD(msg, MTX_GENMSG_SIZE) + 3) / 4;

#if 0
	{
		int i;
		printk(KERN_DEBUG "MSVDX: psb_mtx_send is %dDW\n",
		       msg_num);

		for (i = 0; i < msg_num; i++)
			printk(KERN_DEBUG "0x%08x ", p_msg[i]);
		printk(KERN_DEBUG "\n");
	}
#endif
	buf_size = PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_BUF_SIZE) & ((1 << 16) - 1);

	if (msg_num > buf_size) {
		ret = -EINVAL;
		DRM_ERROR("MSVDX: message exceed maximum,ret:%d\n", ret);
		goto out;
	}

	ridx = PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_RD_INDEX);
	widx = PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_WRT_INDEX);


	buf_size = PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_BUF_SIZE) & ((1 << 16) - 1);
	/*0x2000 is VEC Local Ram offset*/
	buf_offset =
		(PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_BUF_SIZE) >> 16) + 0x2000;

	/* message would wrap, need to send a pad message */
	if (widx + msg_num > buf_size) {
		/* Shouldn't happen for a PAD message itself */
		if (MEMIO_READ_FIELD(msg, MTX_GENMSG_ID)
		       == MTX_MSGID_PADDING)
			DRM_INFO("MSVDX WARNING: should not wrap pad msg, "
				"buf_size is %d, widx is %d, msg_num is %d.\n",
				buf_size, widx, msg_num);

		/* if the read pointer is at zero then we must wait for it to
		 * change otherwise the write pointer will equal the read
		 * pointer,which should only happen when the buffer is empty
		 *
		 * This will only happens if we try to overfill the queue,
		 * queue management should make
		 * sure this never happens in the first place.
		 */
		if (0 == ridx) {
			ret = -EINVAL;
			DRM_ERROR("MSVDX: RIndex=0, ret:%d\n", ret);
			goto out;
		}

		/* Send a pad message */
		pad_msg.header.bits.msg_size = (buf_size - widx) << 2;
		pad_msg.header.bits.msg_type = MTX_MSGID_PADDING;
		psb_mtx_send(dev_priv, (void *)&pad_msg);
		widx = PSB_RMSVDX32(MSVDX_COMMS_TO_MTX_WRT_INDEX);
	}

	if (widx >= ridx)
		words_free = buf_size - (widx - ridx) - 1;
	else
		words_free = ridx - widx - 1;

	if (msg_num > words_free) {
		ret = -EINVAL;
		DRM_ERROR("MSVDX: msg_num > words_free, ret:%d\n", ret);
		goto out;
	}
	while (msg_num > 0) {
		PSB_WMSVDX32(*p_msg++, buf_offset + (widx << 2));
		msg_num--;
		widx++;
		if (buf_size == widx)
			widx = 0;
	}

	PSB_WMSVDX32(widx, MSVDX_COMMS_TO_MTX_WRT_INDEX);

	/* Make sure clocks are enabled before we kick
	 * but fw will take care of the clock after fw is loaded
	 */

	/* signal an interrupt to let the mtx know there is a new message */
	PSB_WMSVDX32(1, MTX_KICK_INPUT_OFFSET);

	/* Read MSVDX Register several times in case Idle signal assert */
	PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET);
	PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET);
	PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET);
	PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET);

out:
	return ret;
}

/*
 * MSVDX MTX interrupt
 */
static void psb_msvdx_mtx_interrupt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	static uint32_t buf[128]; /* message buffer */
	uint32_t ridx, widx, buf_size, buf_offset;
	uint32_t num, ofs; /* message num and offset */
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	int i;

	PSB_DEBUG_GENERAL("MSVDX:Got a MSVDX MTX interrupt\n");

	/* we need clocks enabled before we touch VEC local ram,
	 * but fw will take care of the clock after fw is loaded
	 */

loop: /* just for coding style check */
	ridx = PSB_RMSVDX32(MSVDX_COMMS_TO_HOST_RD_INDEX);
	widx = PSB_RMSVDX32(MSVDX_COMMS_TO_HOST_WRT_INDEX);

	/* Get out of here if nothing */
	if (ridx == widx)
		goto done;

	buf_size = PSB_RMSVDX32(MSVDX_COMMS_TO_HOST_BUF_SIZE) & ((1 << 16) - 1);
	/*0x2000 is VEC Local Ram offset*/
	buf_offset =
		(PSB_RMSVDX32(MSVDX_COMMS_TO_HOST_BUF_SIZE) >> 16) + 0x2000;

	ofs = 0;
	buf[ofs] = PSB_RMSVDX32(buf_offset + (ridx << 2));

	/* round to nearest word */
	num = (MEMIO_READ_FIELD(buf, MTX_GENMSG_SIZE) + 3) / 4;

	/* ASSERT(num <= sizeof(buf) / sizeof(uint32_t)); */

	if (++ridx >= buf_size)
		ridx = 0;

	for (ofs++; ofs < num; ofs++) {
		buf[ofs] = PSB_RMSVDX32(buf_offset + (ridx << 2));

		if (++ridx >= buf_size)
			ridx = 0;
	}

	/* Update the Read index */
	PSB_WMSVDX32(ridx, MSVDX_COMMS_TO_HOST_RD_INDEX);

	if (msvdx_priv->msvdx_needs_reset)
		goto loop;

	switch (MEMIO_READ_FIELD(buf, MTX_GENMSG_ID)) {
	case MTX_MSGID_HW_PANIC: {
		/* For VXD385 firmware, fence value is not validate here */
		uint32_t diff = 0;
		uint32_t fence, last_mb;
		drm_psb_msvdx_frame_info_t *failed_frame = NULL;

		struct fw_panic_msg *panic_msg = (struct fw_panic_msg *)buf;

		PSB_DEBUG_WARN("MSVDX: MSGID_CMD_HW_PANIC:"
				  "Fault detected"
				  " - Fence: %08x"
				  " - fe_status mb: %08x"
				  " - be_status mb: %08x"
				  " - reserved2: %08x"
				  " - last mb: %08x"
				  " - resetting and ignoring error\n",
				  panic_msg->header.bits.msg_fence,
				  panic_msg->fe_status,
				  panic_msg->be_status,
				  panic_msg->mb.bits.reserved2,
				  panic_msg->mb.bits.last_mb);
		PSB_DEBUG_WARN("MSVDX: MSVDX_COMMS_ERROR_TRIG is 0x%x.\n",
					PSB_RMSVDX32(MSVDX_COMMS_ERROR_TRIG));
		fence = panic_msg->header.bits.msg_fence;
		last_mb = panic_msg->mb.bits.last_mb;

		if (msvdx_priv->fw_loaded_by_punit)
			msvdx_priv->msvdx_needs_reset |= MSVDX_RESET_NEEDS_REUPLOAD_FW |
				MSVDX_RESET_NEEDS_INIT_FW;
		else
			msvdx_priv->msvdx_needs_reset = 1;

		diff = msvdx_priv->msvdx_current_sequence
		       - dev_priv->sequence[PSB_ENGINE_DECODE];

		if (diff > 0x0FFFFFFF)
			msvdx_priv->msvdx_current_sequence++;

		PSB_DEBUG_WARN("MSVDX: Fence ID missing, "
				  "assuming %08x\n",
				  msvdx_priv->msvdx_current_sequence);

		psb_fence_error(dev, PSB_ENGINE_DECODE,
				msvdx_priv->msvdx_current_sequence,
				_PSB_FENCE_TYPE_EXE, DRM_CMD_FAILED);

		/* Flush the command queue */
		psb_msvdx_flush_cmd_queue(dev);
#ifdef CONFIG_VIDEO_MRFLD
		if (msvdx_priv->host_be_opp_enabled) {
			/*get the frame_info struct for error concealment frame*/
			for (i = 0; i < MAX_DECODE_BUFFERS; i++) {
				/*by default the fence is 0, so there is problem here???*/
				if (msvdx_priv->frame_info[i].fence == fence) {
					failed_frame = &msvdx_priv->frame_info[i];
					break;
				}
			}
			if (!failed_frame) {
				DRM_ERROR("MSVDX: didn't find frame_info which matched the fence %d in failed/panic message\n", fence);
				goto done;
			}

			failed_frame->fw_status = 1; /* set ERROR flag */
		}
#endif
		msvdx_priv->decoding_err = 1;

		goto done;
	}

	case MTX_MSGID_COMPLETED: {
		uint32_t fence, flags;
		struct fw_completed_msg *completed_msg =
					(struct fw_completed_msg *)buf;

		PSB_DEBUG_GENERAL("MSVDX: MSGID_CMD_COMPLETED:"
			" - Fence: %08x - flags: %08x - vdebcr: %08x"
			" - fe_begin_setup : %08x - fe_begin_decode: %08x"
			" - fe_end_decode : %08x - be_begin_setup: %08x"
			" - be_begin_decode : %08x - be_end_decode: %08x\n",
			completed_msg->header.bits.msg_fence,
			completed_msg->flags, completed_msg->vdebcr,
			completed_msg->fe_begin_setup,
			completed_msg->fe_begin_decode,
			completed_msg->fe_end_decode,
			completed_msg->be_begin_setup,
			completed_msg->be_begin_decode,
			completed_msg->be_end_decode);

		flags = completed_msg->flags;
		fence = completed_msg->header.bits.msg_fence;

		msvdx_priv->msvdx_current_sequence = fence;
				;
		msvdx_priv->ref_pic_fence = fence;

		psb_fence_handler(dev, PSB_ENGINE_DECODE);

		if (flags & FW_VA_RENDER_HOST_INT) {
			/*Now send the next command from the msvdx cmd queue */
			psb_msvdx_dequeue_send(dev);
			goto done;
		}

		break;
		msvdx_priv->decoding_err = 0;
	}

#ifdef CONFIG_VIDEO_MRFLD
	case MTX_MSGID_CONTIGUITY_WARNING: {
		drm_psb_msvdx_decode_status_t *fault_region = NULL;
		struct psb_msvdx_ec_ctx *msvdx_ec_ctx = NULL;
		uint32_t reg_idx;
		int found = 0;

		struct fw_contiguity_msg *contiguity_msg =
					(struct fw_contiguity_msg *)buf;

		PSB_DEBUG_GENERAL("MSVDX: MSGID_CONTIGUITY_WARNING:");
		PSB_DEBUG_GENERAL(
			"- Fence: %08x - end_mb: %08x - begin_mb: %08x\n",
			contiguity_msg->header.bits.msg_fence,
			contiguity_msg->mb.bits.end_mb_num,
			contiguity_msg->mb.bits.begin_mb_num);

		/*get erro info*/
		uint32_t fence = contiguity_msg->header.bits.msg_fence;
		uint32_t start = contiguity_msg->mb.bits.begin_mb_num;
		uint32_t end = contiguity_msg->mb.bits.end_mb_num;

		/*get the frame_info struct for error concealment frame*/
		for (i = 0; i < PSB_MAX_EC_INSTANCE; i++)
			if (msvdx_priv->msvdx_ec_ctx[i]->fence ==
							(fence & (~0xf))) {
				msvdx_ec_ctx = msvdx_priv->msvdx_ec_ctx[i];
				found++;
			}
		/* psb_msvdx_mtx_message_dump(dev); */
		if (!msvdx_ec_ctx || !(msvdx_ec_ctx->tfile) || found > 1) {
			PSB_DEBUG_MSVDX(
			"no matched ctx: fence 0x%x, found %d, ctx 0x%08x\n",
				fence, found, msvdx_ec_ctx);
			goto done;
		}


		fault_region = &msvdx_ec_ctx->decode_status;
		if (start > end)
			start = end;
		if (start < PSB_MSVDX_EC_ROLLBACK)
			start = 0;
		else
			start -= PSB_MSVDX_EC_ROLLBACK;

		if (fault_region->num_region) {
			reg_idx = fault_region->num_region - 1;
			if ((start <= fault_region->mb_regions[reg_idx].end) &&
			    (end > fault_region->mb_regions[reg_idx].end))
				fault_region->mb_regions[reg_idx].end = end;
			else {
				reg_idx = fault_region->num_region++;
				if (unlikely(reg_idx >=
					MAX_SLICES_PER_PICTURE)) {
					PSB_DEBUG_MSVDX(
						"too many fault regions\n");
					break;
				}
				fault_region->mb_regions[reg_idx].start = start;
				fault_region->mb_regions[reg_idx].end = end;
			}
		} else {
			fault_region->num_region++;
			fault_region->mb_regions[0].start = start;
			fault_region->mb_regions[0].end = end;
		}

		break;

	}

	case MTX_MSGID_DEBLOCK_REQUIRED: {
		struct fw_deblock_required_msg *deblock_required_msg =
					(struct fw_deblock_required_msg *)buf;
		uint32_t fence;

		fence = deblock_required_msg->header.bits.msg_fence;
		PSB_DEBUG_GENERAL(
		    "MSVDX: MTX_MSGID_DEBLOCK_REQUIRED Fence=%08x\n", fence);


		struct psb_msvdx_ec_ctx *msvdx_ec_ctx = NULL;
		int found = 0;
		PSB_DEBUG_MSVDX("Get deblock required msg for ec\n");
		for (i = 0; i < PSB_MAX_EC_INSTANCE; i++)
			if (msvdx_priv->msvdx_ec_ctx[i]->fence
						== (fence & (~0xf))) {
				msvdx_ec_ctx =
					msvdx_priv->msvdx_ec_ctx[i];
				found++;
			}
		/* if found > 1, fence wrapping happens */
		if (!msvdx_ec_ctx ||
		    !(msvdx_ec_ctx->tfile) || found > 1) {
			PSB_DEBUG_MSVDX(
		"no matched ctx: fence 0x%x, found %d, ctx 0x%08x\n",
				fence, found, msvdx_ec_ctx);
			PSB_WMSVDX32(0, MSVDX_CMDS_END_SLICE_PICTURE_OFFSET);
			PSB_WMSVDX32(1, MSVDX_CMDS_END_SLICE_PICTURE_OFFSET);
			goto done;
		}

		msvdx_ec_ctx->cur_frame_info->fw_status = 1;

		/* try to unblock rendec */
		/*
		PSB_WMSVDX32(0, MSVDX_CMDS_END_SLICE_PICTURE);
		PSB_WMSVDX32(1, MSVDX_CMDS_END_SLICE_PICTURE);
		*/
		/*do error concealment with hw*/
		msvdx_priv->cur_msvdx_ec_ctx = msvdx_ec_ctx;
		schedule_work(&msvdx_priv->ec_work);
		break;
	}
#endif

	default:
		DRM_ERROR("ERROR: msvdx Unknown message from MTX, ID:0x%08x\n", MEMIO_READ_FIELD(buf, MTX_GENMSG_ID));
		goto done;
	}

done:
	PSB_DEBUG_GENERAL("MSVDX Interrupt: finish process a message\n");
	if (ridx != widx) {
		PSB_DEBUG_GENERAL("MSVDX Interrupt: there are more message to be read\n");
		goto loop;
	}

	/* we get a frame/slice done, try to save some power*/
	if (msvdx_priv->fw_loaded_by_punit) {
		if ((drm_msvdx_pmpolicy == PSB_PMPOLICY_POWERDOWN) &&
			(msvdx_priv->msvdx_busy == 0)) {
			PSB_DEBUG_PM("MSVDX: schedule work queue to\n"
				"suspend msvdx, current sequence is 0x%x.\n",
				msvdx_priv->msvdx_current_sequence);
			schedule_delayed_work(&msvdx_priv->msvdx_suspend_wq, 0);
		}
	}
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	else {
		if (drm_msvdx_pmpolicy != PSB_PMPOLICY_NOPM)
			schedule_delayed_work(&msvdx_priv->msvdx_suspend_wq, 0);
	}
#endif
	DRM_MEMORYBARRIER();	/* TBD check this... */
}


/*
 * MSVDX interrupt.
 */
IMG_BOOL psb_msvdx_interrupt(IMG_VOID *pvData)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct msvdx_private *msvdx_priv;
	uint32_t msvdx_stat;
	struct saved_history_record *precord = NULL;

	if (pvData == IMG_NULL) {
		DRM_ERROR("ERROR: msvdx %s, Invalid params\n", __func__);
		return IMG_FALSE;
	}

	dev = (struct drm_device *)pvData;

	dev_priv = (struct drm_psb_private *) dev->dev_private;
	msvdx_priv = dev_priv->msvdx_private;

	msvdx_priv->msvdx_hw_busy = REG_READ(0x20D0) & (0x1 << 9);

	msvdx_stat = PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET);

	precord = get_new_history_record();
	if (precord) {
		precord->type = 4;
		precord->record_value.msvdx_stat = msvdx_stat;
	}

	/* driver only needs to handle mtx irq
	 * For MMU fault irq, there's always a HW PANIC generated
	 * if HW/FW is totally hang, the lockup function will handle
	 * the reseting
	 */
	if (msvdx_stat & MSVDX_INTERRUPT_STATUS__MMU_FAULT_IRQ_MASK) {
		/*Ideally we should we should never get to this */
		PSB_DEBUG_IRQ("MSVDX:MMU Fault:0x%x\n", msvdx_stat);

		/* Pause MMU */
		PSB_WMSVDX32(MSVDX_MMU_CONTROL0__MMU_PAUSE_MASK,
			     MSVDX_MMU_CONTROL0_OFFSET);
		DRM_WRITEMEMORYBARRIER();

		/* Clear this interupt bit only */
		PSB_WMSVDX32(MSVDX_INTERRUPT_STATUS__MMU_FAULT_IRQ_MASK,
			     MSVDX_INTERRUPT_CLEAR_OFFSET);
		PSB_RMSVDX32(MSVDX_INTERRUPT_CLEAR_OFFSET);
		DRM_READMEMORYBARRIER();

		msvdx_priv->msvdx_needs_reset = 1;
	} else if (msvdx_stat & MSVDX_INTERRUPT_STATUS__MTX_IRQ_MASK) {
		PSB_DEBUG_IRQ
			("MSVDX: msvdx_stat: 0x%x(MTX)\n", msvdx_stat);

		/* Clear all interupt bits */
		if (msvdx_priv->fw_loaded_by_punit)
			PSB_WMSVDX32(MSVDX_INTERRUPT_STATUS__MTX_IRQ_MASK,
				     MSVDX_INTERRUPT_CLEAR_OFFSET);
		else
			PSB_WMSVDX32(0xffff, MSVDX_INTERRUPT_CLEAR_OFFSET);

		PSB_RMSVDX32(MSVDX_INTERRUPT_CLEAR_OFFSET);
		DRM_READMEMORYBARRIER();

		psb_msvdx_mtx_interrupt(dev);
	}

	return IMG_TRUE;
}

#if 0
void psb_msvdx_lockup(struct drm_psb_private *dev_priv,
		      int *msvdx_lockup, int *msvdx_idle)
{
	int diff;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	*msvdx_lockup = 0;
	*msvdx_idle = 1;

	PSB_DEBUG_GENERAL("MSVDXTimer: current_sequence:%d "
			  "last_sequence:%d and last_submitted_sequence :%d\n",
			  msvdx_priv->msvdx_current_sequence,
			  msvdx_priv->msvdx_last_sequence,
			  dev_priv->sequence[PSB_ENGINE_DECODE]);

	diff = msvdx_priv->msvdx_current_sequence -
	       dev_priv->sequence[PSB_ENGINE_DECODE];

	if (diff > 0x0FFFFFFF) {
		if (msvdx_priv->msvdx_current_sequence ==
		    msvdx_priv->msvdx_last_sequence) {
			DRM_ERROR("MSVDXTimer:locked-up for sequence:%d\n",
				  msvdx_priv->msvdx_current_sequence);
			*msvdx_lockup = 1;
		} else {
			PSB_DEBUG_GENERAL("MSVDXTimer: "
					  "msvdx responded fine so far\n");
			msvdx_priv->msvdx_last_sequence =
				msvdx_priv->msvdx_current_sequence;
			*msvdx_idle = 0;
		}
	}
}
#endif

int psb_check_msvdx_idle(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t loop, ret;

	if (msvdx_priv->fw_loaded_by_punit && msvdx_priv->rendec_init == 0)
		return 0;
#ifdef PSB_MSVDX_FW_LOADED_BY_HOST
	if (!msvdx_priv->fw_loaded_by_punit && msvdx_priv->msvdx_fw_loaded == 0)
		return 0;
#endif
	if (msvdx_priv->msvdx_busy) {
		PSB_DEBUG_PM("MSVDX: msvdx_busy was set, return busy.\n");
		return -EBUSY;
	}

	if (msvdx_priv->fw_loaded_by_punit) {
		if (!(PSB_RMSVDX32(MSVDX_COMMS_FW_STATUS) &
					MSVDX_FW_STATUS_HW_IDLE)) {
			PSB_DEBUG_PM("MSVDX_COMMS_SIGNATURE reg is 0x%x,\n"
				"MSVDX_COMMS_FW_STATUS reg is 0x%x,\n"
				"indicate hw is busy.\n",
				PSB_RMSVDX32(MSVDX_COMMS_SIGNATURE),
				PSB_RMSVDX32(MSVDX_COMMS_FW_STATUS));
			return -EBUSY;
		}
	}

	/* on some cores below 50502, there is one instance that
	 * read requests may not go to zero is in the case of a page fault,
	 * check core revision by reg MSVDX_CORE_REV, 385 core is 0x20001
	 * check if mmu page fault happend by reg MSVDX_INTERRUPT_STATUS,
	 * check was it a page table rather than a protection fault
	 * by reg MSVDX_MMU_STATUS, for such case,
	 * need call psb_msvdx_core_reset as the work around */
	if ((PSB_RMSVDX32(MSVDX_CORE_REV_OFFSET) < 0x00050502) &&
		(PSB_RMSVDX32(MSVDX_INTERRUPT_STATUS_OFFSET)
			& MSVDX_INTERRUPT_STATUS__MMU_FAULT_IRQ_MASK) &&
		(PSB_RMSVDX32(MSVDX_MMU_STATUS_OFFSET) & 1)) {
		PSB_DEBUG_WARN("mmu page fault, recover by core_reset.\n");
		return 0;
	}

	/* check MSVDX_MMU_MEM_REQ to confirm there's no memory requests */
	for (loop = 0; loop < 10; loop++)
		ret = psb_wait_for_register(dev_priv,
					MSVDX_MMU_MEM_REQ_OFFSET,
					0, 0xff, 100, 1);
	if (ret) {
		PSB_DEBUG_WARN("MSVDX: MSVDX_MMU_MEM_REQ reg is 0x%x,\n"
				"indicate mem busy, prevent power off vxd,"
				"MSVDX_COMMS_FW_STATUS reg is 0x%x,"
				"MSVDX_COMMS_ERROR_TRIG reg is 0x%x,",
				PSB_RMSVDX32(MSVDX_MMU_MEM_REQ_OFFSET),
				PSB_RMSVDX32(MSVDX_COMMS_FW_STATUS),
				PSB_RMSVDX32(MSVDX_COMMS_ERROR_TRIG));
#ifdef CONFIG_MDFD_GL3
		PSB_DEBUG_WARN("WARN: gl3 state is %d, 0 is off, 1 is on,\n"
				"gl3 MDFLD_GCL_CR_CTL2 reg is 0x%x,"
				"gl3 MDFLD_GCL_ERR_ADDR reg is 0x%x,"
				"gl3 MDFLD_GCL_ERR_STATUS reg is 0x%x,"
				"gl3 MDFLD_GCL_CR_ECO reg is 0x%x,"
				"gl3 MDFLD_GL3_CONTROL reg is 0x%x,"
				"gl3 MDFLD_GL3_USE_WRT_INVAL reg is 0x%x,"
				"gl3 MDFLD_GL3_STATUS reg is 0x%x.\n",
				psb_get_power_state(OSPM_GL3_CACHE_ISLAND),
				MDFLD_GL3_READ(MDFLD_GCL_CR_CTL2),
				MDFLD_GL3_READ(MDFLD_GCL_ERR_ADDR),
				MDFLD_GL3_READ(MDFLD_GCL_ERR_STATUS),
				MDFLD_GL3_READ(MDFLD_GCL_CR_ECO),
				MDFLD_GL3_READ(MDFLD_GL3_CONTROL),
				MDFLD_GL3_READ(MDFLD_GL3_USE_WRT_INVAL),
				MDFLD_GL3_READ(MDFLD_GL3_STATUS));
#endif
		return -EBUSY;
	}
	/*
		if (msvdx_priv->msvdx_hw_busy) {
			PSB_DEBUG_PM("MSVDX: %s, HW is busy\n", __func__);
			return -EBUSY;
		}
	*/
	return 0;
}

int psb_msvdx_save_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	int offset;

	if (msvdx_priv->fw_loaded_by_punit)
		msvdx_priv->msvdx_needs_reset = MSVDX_RESET_NEEDS_INIT_FW;
	else
		msvdx_priv->msvdx_needs_reset = 1;

#ifdef PSB_MSVDX_SAVE_RESTORE_VEC
	for (offset = 0; offset < VEC_LOCAL_MEM_BYTE_SIZE / 4; ++offset)
		msvdx_priv->vec_local_mem_data[offset] =
			PSB_RMSVDX32(VEC_LOCAL_MEM_OFFSET + offset * 4);

	msvdx_priv->vec_local_mem_saved = 1;
#endif

#ifdef CONFIG_VIDEO_MRFLD_EC
	/* we should restore the state, if we power down/up during EC */
	for (offset = 0; offset < 4; ++offset)
		msvdx_priv->vec_ec_mem_data[offset] =
			PSB_RMSVDX32(0x2cb0 + offset * 4);
	msvdx_priv->vec_ec_mem_saved = 1;
#endif

	/* Reset MTX */
	PSB_WMSVDX32(MTX_SOFT_RESET__MTXRESET, MTX_SOFT_RESET_OFFSET);

	/* why need reset msvdx before power off it, need check IMG */
	if (psb_msvdx_core_reset(dev_priv))
		PSB_DEBUG_WARN("failed to call psb_msvdx_core_reset.\n");

	/* Initialize VEC Local RAM */
	for (offset = 0; offset < VEC_LOCAL_MEM_BYTE_SIZE / 4; ++offset)
		PSB_WMSVDX32(0, VEC_LOCAL_MEM_OFFSET + offset * 4);

	if (msvdx_priv->fw_loaded_by_punit) {
		PSB_WMSVDX32(0, MTX_ENABLE_OFFSET);
		psb_msvdx_mtx_set_clocks(dev_priv->dev, 0);
	}

	return 0;
}

int psb_msvdx_restore_context(struct drm_device *dev)
{
	return 0;
}

void psb_msvdx_check_reset_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	unsigned long irq_flags;

	spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);

	/* handling fw upload here if required */
	/* power off first, then hw_begin will power up/upload FW correctly */
	if (msvdx_priv->msvdx_needs_reset & MSVDX_RESET_NEEDS_REUPLOAD_FW) {
		msvdx_priv->msvdx_needs_reset &= ~MSVDX_RESET_NEEDS_REUPLOAD_FW;
		spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
		PSB_DEBUG_PM("MSVDX: force to power off msvdx due to decoding error.\n");
		ospm_apm_power_down_msvdx(dev, 1);
		spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	}
	spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);
}

static void psb_msvdx_set_tile(struct drm_device *dev, unsigned long msvdx_tile)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	uint32_t cmd, msvdx_stride;
	uint32_t start = msvdx_priv->tile_region_start0;
	uint32_t end = msvdx_priv->tile_region_end0;

	msvdx_stride = (msvdx_tile & 0xf);
	/* Enable memory tiling */
	cmd = ((start >> 20) + (((end >> 20) - 1) << 12) +
				((0x8 | (msvdx_stride - 1)) << 24));
	if (msvdx_stride) {
		PSB_DEBUG_GENERAL("MSVDX: MMU Tiling register0 %08x\n", cmd);
		PSB_DEBUG_GENERAL("       Region 0x%08x-0x%08x\n",
					start, end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE0_OFFSET);
	}

	start = msvdx_priv->tile_region_start1;
	end = msvdx_priv->tile_region_end1;

	msvdx_stride = (msvdx_tile >> 4);
	/* Enable memory tiling */
	PSB_WMSVDX32(0, MSVDX_MMU_TILE_BASE1_OFFSET);
	cmd = ((start >> 20) + (((end >> 20) - 1) << 12) +
				((0x8 | (msvdx_stride - 1)) << 24));
	if (msvdx_stride) {
		PSB_DEBUG_GENERAL("MSVDX: MMU Tiling register1 %08x\n", cmd);
		PSB_DEBUG_GENERAL("       Region 0x%08x-0x%08x\n",
					start, end);
		PSB_WMSVDX32(cmd, MSVDX_MMU_TILE_BASE1_OFFSET);
	}
}

void psb_powerdown_msvdx(struct work_struct *work)
{
	struct msvdx_private *msvdx_priv =
		container_of(work, struct msvdx_private, msvdx_suspend_wq.work);

	ospm_apm_power_down_msvdx(msvdx_priv->dev, 0);
}
