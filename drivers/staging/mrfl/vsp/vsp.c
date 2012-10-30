/**
 * file vsp.c
 * VSP IRQ handling and I/O ops
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

#include <drm/drmP.h>
#include <linux/math64.h>

#include "psb_drv.h"
#include "psb_drm.h"
#include "vsp.h"
#include "ttm/ttm_execbuf_util.h"
#include "vsp_fw.h"

#define ERR_ID_MASK 0xFFFF
#define ERR_INFO_SHIFT 16

static int vsp_submit_cmdbuf(struct drm_device *dev,
			     unsigned char *cmd_start,
			     unsigned long cmd_size);
static int vsp_send_command(struct drm_device *dev,
			    unsigned char *cmd_start,
			    unsigned long cmd_size);
static int vsp_assign_fence(struct drm_file *priv,
			    struct list_head *validate_list,
			    uint32_t fence_type,
			    struct drm_psb_cmdbuf_arg *arg,
			    unsigned char *cmd_start,
			    struct psb_ttm_fence_rep *fence_arg);
static void check_invalid_cmd_type(unsigned int info);
static void check_invalid_cmd_arg(unsigned int info);
static int vsp_fence_surfaces(struct drm_file *priv,
			      struct list_head *validate_list,
			      uint32_t fence_type,
			      struct drm_psb_cmdbuf_arg *arg,
			      struct psb_ttm_fence_rep *fence_arg,
			      struct ttm_buffer_object *pic_param_bo);
static void handle_error_response(unsigned int error);

bool vsp_interrupt(void *pvData)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct vsp_private *vsp_priv;
	unsigned int rd, wr;
	unsigned int idx;
	unsigned int msg_num;
	struct vss_response_t *msg;
	unsigned long status;
	bool ret;
	uint32_t sequence;

	VSP_DEBUG("got vsp interrupt\n");

	if (pvData == NULL) {
		DRM_ERROR("VSP: vsp %s, Invalid params\n", __func__);
		return false;
	}

	dev = (struct drm_device *)pvData;
	dev_priv = (struct drm_psb_private *) dev->dev_private;
	vsp_priv = dev_priv->vsp_private;

	/* read interrupt status */
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_STATUS, &status);
	VSP_DEBUG("irq status %lx\n", status);
	/* clear interrupt status */
	if (!(status & (1 << VSP_SP1_IRQ_SHIFT))) {
		DRM_ERROR("VSP: invalid irq\n");
		return false;
	} else {
		IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_CLR, (1 << VSP_SP1_IRQ_SHIFT));
	}

	sequence = vsp_priv->current_sequence;
	spin_lock(&vsp_priv->lock);

	rd = vsp_priv->ctrl->ack_rd;
	wr = vsp_priv->ctrl->ack_wr;
	msg_num = wr > rd ? wr - rd : wr == rd ? 0 :
		VSP_ACK_QUEUE_SIZE - (rd - wr);

	VSP_DEBUG("ack rd %d wr %d, msg_num %d, size %d\n", rd, wr,
		  msg_num, VSP_ACK_QUEUE_SIZE);

	ret = true;
	for (idx = 0; idx < msg_num; ++idx) {
		/* FIXME: could remove idx here, but pls consider race
		 * condition
		 */
		msg = vsp_priv->ack_queue + (idx + rd) % VSP_ACK_QUEUE_SIZE;

		switch (msg->type) {
		case VssErrorResponse:
			handle_error_response(msg->buffer);
			DRM_ERROR("VSP: there're %d response remaining\n",
				  msg_num - idx - 1);
			ret = false;
			break;

		case VssEndOfSequenceResponse:
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			VSP_DEBUG("end of the sequence received\n");
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssOutputSurfaceReadyResponse:
			VSP_DEBUG("sequence %x is done\n", msg->buffer);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			sequence = msg->buffer;

			/* FIXME: need to check current handler */

			break;

		case VssOutputSurfaceFreeResponse:
			VSP_DEBUG("sequence surface %x should be freed\n",
				  msg->buffer);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssOutputSurfaceCrcResponse:
			VSP_DEBUG("Crc of sequence %x is %x\n", msg->buffer,
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssInputSurfaceReadyResponse:
			VSP_DEBUG("input surface ready\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssCommandBufferReadyResponse:
			VSP_DEBUG("command buffer ready\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssIdleResponse:
			VSP_DEBUG("VSP is idle, can be powered off\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		default:
			DRM_ERROR("VSP: Unknown response type %x\n",
				  msg->type);
			DRM_ERROR("VSP: there're %d response remaining\n",
				  msg_num - idx - 1);
			ret = false;
			break;
		}
		vsp_priv->ctrl->ack_rd = (vsp_priv->ctrl->ack_rd + 1) %
			VSP_ACK_QUEUE_SIZE;
		VSP_DEBUG("idx %d\n", idx);
	}

	spin_unlock(&vsp_priv->lock);
	VSP_DEBUG("will leave interrupt\n");

	/* for compile with VP */
	if (sequence != vsp_priv->current_sequence) {
		vsp_priv->current_sequence = sequence;
		psb_fence_handler(dev, VSP_ENGINE_VPP);
	}

	return ret;
}

int vsp_cmdbuf_vpp(struct drm_file *priv,
		    struct list_head *validate_list,
		    uint32_t fence_type,
		    struct drm_psb_cmdbuf_arg *arg,
		    struct ttm_buffer_object *cmd_buffer,
		    struct psb_ttm_fence_rep *fence_arg)
{
	struct drm_device *dev = priv->minor->dev;
	int ret = 0;
	unsigned char *cmd_start;
	unsigned long cmd_page_offset = arg->cmdbuf_offset & ~PAGE_MASK;
	struct ttm_bo_kmap_obj cmd_kmap;
	bool is_iomem;

	/* check command buffer parameter */
	if ((arg->cmdbuf_offset > cmd_buffer->acc_size) ||
	    (arg->cmdbuf_size > cmd_buffer->acc_size) ||
	    (arg->cmdbuf_size + arg->cmdbuf_offset) > cmd_buffer->acc_size) {
		DRM_ERROR("VSP: the size of cmdbuf is invalid!");
		DRM_ERROR("VSP: offset=%x, size=%x,cmd_buffer size=%x\n",
			  arg->cmdbuf_offset, arg->cmdbuf_size,
			  cmd_buffer->acc_size);
		ret = -EFAULT;
		goto out;
	}

	VSP_DEBUG("map command first\n");
	ret = ttm_bo_kmap(cmd_buffer, arg->cmdbuf_offset >> PAGE_SHIFT, 2,
			  &cmd_kmap);
	if (ret) {
		DRM_ERROR("VSP: ttm_bo_kmap failed: %d\n", ret);
		return ret;
	}

	cmd_start = (unsigned char *) ttm_kmap_obj_virtual(&cmd_kmap,
			&is_iomem) + cmd_page_offset;

	VSP_DEBUG("will assign fence\n");
	ret = vsp_assign_fence(priv, validate_list, fence_type, arg,
			       cmd_start, fence_arg);
	if (ret)
		goto out;

	VSP_DEBUG("will submit command\n");
	ret = vsp_submit_cmdbuf(dev, cmd_start, arg->cmdbuf_size);
	if (ret)
		goto out;

out:
	ttm_bo_kunmap(&cmd_kmap);

	spin_lock(&cmd_buffer->bdev->fence_lock);
	if (cmd_buffer->sync_obj != NULL)
		ttm_fence_sync_obj_unref(&cmd_buffer->sync_obj);
	spin_unlock(&cmd_buffer->bdev->fence_lock);

	return ret;
}

int vsp_submit_cmdbuf(struct drm_device *dev,
		      unsigned char *cmd_start,
		      unsigned long cmd_size)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret;
	unsigned long irq_flags;

	/* FIXME: context checking? */

	if (vsp_priv->fw_loaded == 0) {
		ret = vsp_init_fw(dev);
		if (ret != 0) {
			DRM_ERROR("VSP: failed to load firmware\n");
			return -EFAULT;
		}
	}

	/* consider to invalidate/flush MMU */
	if (vsp_priv->needs_reset) {
		VSP_DEBUG("needs reset\n");

		if (vsp_reset(dev_priv)) {
			ret = -EBUSY;
			DRM_ERROR("VSP: failed to reset\n");
			return ret;
		}
	}

	/* submit command to HW */
	ret = vsp_send_command(dev, cmd_start, cmd_size);
	if (ret != 0)
		DRM_ERROR("VSP: failed to send command\n");

	return ret;
}

int vsp_send_command(struct drm_device *dev,
		     unsigned char *cmd_start,
		     unsigned long cmd_size)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int rd, wr;
	unsigned int remaining_space;
	unsigned int cmd_idx;
	struct vss_command_t *cur_cmd, *cur_cell_cmd;

	VSP_DEBUG("will send command here: cmd_start %p, cmd_size %ld\n",
		  cmd_start, cmd_size);

	cur_cmd = (struct vss_command_t *)cmd_start;
	while (cmd_size) {
		rd = vsp_priv->ctrl->cmd_rd;
		wr = vsp_priv->ctrl->cmd_wr;

		remaining_space = rd >= wr + 1 ? rd - wr - 1 :
			VSP_CMD_QUEUE_SIZE - (wr + 1 - rd) ;

		VSP_DEBUG("VSP: rd %d, wr %d, remaining_space %d, "
			  "cmd_size %ld sizeof(*cur_cmd) %d\n",
			  rd, wr, remaining_space, cmd_size, sizeof(*cur_cmd));

		if (remaining_space < 1) {
			DRM_ERROR("VSP: no enough space to submit"
				  " command, waiting\n");
			DRM_ERROR("VSP: rd %d, wr %d, remaining_space %d\n",
				  rd, wr, remaining_space);
			/* VP handle the data very slowly,
			* so we have to delay longer
			*/
#ifdef CONFIG_BOARD_MRFLD_VP
			udelay(1000);
#else
			udelay(10);
#endif
			continue;
		}

		VSP_DEBUG("VSP: cmd_queue_size %d\n", VSP_CMD_QUEUE_SIZE);
		for (cmd_idx = 0; cmd_idx < remaining_space;) {
			VSP_DEBUG("current cmd type %x\n", cur_cmd->type);
			if (cur_cmd->type == VspFencePictureParamCommand) {
				VSP_DEBUG("skip VspFencePictureParamCommand"
					  " during cmd submit\n");
				cur_cmd++;
				cmd_size -= sizeof(*cur_cmd);
				VSP_DEBUG("first cmd_size %ld\n", cmd_size);
				if (cmd_size == 0)
					goto out;
				else
					continue;
			} else if (cur_cmd->type == VspSetContextCommand) {
				VSP_DEBUG("set context and new vsp context\n");

				VSP_DEBUG("set context base %x, size %x\n",
					  cur_cmd->buffer, cur_cmd->size);
				vsp_priv->ctrl->context_buf_addr =
					cur_cmd->buffer;
				vsp_priv->ctrl->context_buf_sz = cur_cmd->size;

				vsp_new_context(dev_priv->dev);

				cur_cmd++;
				cmd_size -= sizeof(*cur_cmd);
				if (cmd_size == 0)
					goto out;
				else
					continue;
			}

			VSP_DEBUG("command buffer %x\n", cur_cmd->buffer);

			/* FIXME: could remove cmd_idx here */
			cur_cell_cmd = vsp_priv->cmd_queue +
				(wr + cmd_idx) % VSP_CMD_QUEUE_SIZE;
			++cmd_idx;
			memcpy(cur_cell_cmd, cur_cmd, sizeof(*cur_cmd));
			/* cur_cell_cmd->sequence_number = sequence; */

			/* update write index */
			vsp_priv->ctrl->cmd_wr =
				(vsp_priv->ctrl->cmd_wr + 1) %
				VSP_CMD_QUEUE_SIZE;

			cur_cmd++;
			cmd_size -= sizeof(*cur_cmd);
			VSP_DEBUG("cmd_size %ld\n", cmd_size);
			if (cmd_size == 0)
				goto out;
			else if (cmd_size < sizeof(*cur_cmd)) {
				DRM_ERROR("VSP: invalid command, "
					  "current cmd_size %ld\n",
					  cmd_size);
				goto out;
			}
		}
	}
out:
	return 0;
}

static int vsp_assign_fence(struct drm_file *priv,
			    struct list_head *validate_list,
			    uint32_t fence_type,
			    struct drm_psb_cmdbuf_arg *arg,
			    unsigned char *cmd_start,
			    struct psb_ttm_fence_rep *fence_arg)
{
	struct ttm_object_file *tfile = BCVideoGetPriv(priv)->tfile;
	struct vss_command_t *cur_cmd;
	unsigned int cmd_size = arg->cmdbuf_size;
	int ret = 0;
	struct ttm_buffer_object *pic_param_bo;
	int pic_param_num;
	struct ttm_validate_buffer *pos, *next;

	cur_cmd = (struct vss_command_t *)cmd_start;

	pic_param_num = 0;
	VSP_DEBUG("cmd size %d\n", cmd_size);
	while (cmd_size) {
		VSP_DEBUG("cmd type %d, buffer offset %x\n", cur_cmd->type,
			  cur_cmd->buffer);
		if (cur_cmd->type == VspFencePictureParamCommand) {
			pic_param_bo =
				ttm_buffer_object_lookup(tfile,
							 cur_cmd->buffer);
			if (pic_param_bo == NULL) {
				DRM_ERROR("VSP: failed to find %x bo\n",
					  cur_cmd->buffer);
				ret = -1;
				goto out;
			}
			pic_param_num++;
			VSP_DEBUG("find pic param buffer: id %x, offset %lx\n",
				  cur_cmd->buffer, pic_param_bo->offset);
			VSP_DEBUG("pic param placement %x bus.add %p\n",
				  pic_param_bo->mem.placement,
				  pic_param_bo->mem.bus.addr);
			if (pic_param_num > 1) {
				DRM_ERROR("VSP: there should be only 1"
					  " fence pic param cmd\n");
				ret = -1;
				goto out;
			}
		}

		cmd_size -= sizeof(*cur_cmd);
		cur_cmd++;
	}

	if (pic_param_num > 0) {
		ret = vsp_fence_surfaces(priv, validate_list, fence_type, arg,
					 fence_arg, pic_param_bo);
	} else {
		/* unreserve these buffer */
		list_for_each_entry_safe(pos, next, validate_list, head) {
			ttm_bo_unreserve(pos->bo);
		}

		VSP_DEBUG("no fence for this command\n");
		goto out;
	}

	VSP_DEBUG("finished fencing\n");
out:
	return ret;
}

int vsp_fence_surfaces(struct drm_file *priv,
		       struct list_head *validate_list,
		       uint32_t fence_type,
		       struct drm_psb_cmdbuf_arg *arg,
		       struct psb_ttm_fence_rep *fence_arg,
		       struct ttm_buffer_object *pic_param_bo)
{
	struct ttm_bo_kmap_obj pic_param_kmap;
	struct psb_ttm_fence_rep local_fence_arg;
	bool is_iomem;
	int ret = 0;
	struct VssProcPictureParameterBuffer *pic_param;
	int output_surf_num;
	int idx;
	int found;
	uint32_t surf_handler;
	struct ttm_buffer_object *surf_bo;
	struct ttm_fence_object *fence = NULL;
	struct list_head surf_list, tmp_list;
	struct ttm_validate_buffer *pos, *next, *cur_valid_buf;
	struct ttm_object_file *tfile = BCVideoGetPriv(priv)->tfile;

	INIT_LIST_HEAD(&surf_list);
	INIT_LIST_HEAD(&tmp_list);

	/* map pic param */
	ret = ttm_bo_kmap(pic_param_bo, 0, pic_param_bo->num_pages,
			  &pic_param_kmap);
	if (ret) {
		DRM_ERROR("VSP: ttm_bo_kmap failed: %d\n", ret);
		goto out;
	}

	pic_param = (struct VssProcPictureParameterBuffer *)
		ttm_kmap_obj_virtual(&pic_param_kmap, &is_iomem);

	output_surf_num = pic_param->num_output_pictures;
	VSP_DEBUG("output surf num %d\n", output_surf_num);

	/* create surface fence*/
	for (idx = 0; idx < output_surf_num - 1; ++idx) {
		found = 0;

		surf_handler = pic_param->output_picture[idx].surface_id;
		VSP_DEBUG("handling surface id %x\n", surf_handler);

		surf_bo = ttm_buffer_object_lookup(tfile, surf_handler);
		if (surf_bo == NULL) {
			DRM_ERROR("VSP: failed to find %x surface\n",
				  surf_handler);
			ret = -1;
			goto out;
		}
		VSP_DEBUG("find target surf_bo %lx\n", surf_bo->offset);

		/* remove from original validate list */
		list_for_each_entry_safe(pos, next,
					 validate_list, head) {
			if (surf_bo->offset ==  pos->bo->offset) {
				cur_valid_buf = pos;
				list_del_init(&pos->head);
				found = 1;
				break;
			}
		}

		BUG_ON(!list_empty(&surf_list));
		/* create fence */
		if (found == 1) {
			/* create right list */
			list_add_tail(&cur_valid_buf->head, &surf_list);
			psb_fence_or_sync(priv, VSP_ENGINE_VPP,
					  fence_type, arg->fence_flags,
					  &surf_list, &local_fence_arg,
					  &fence);
			list_del_init(&pos->head);
			/* reserve it */
			list_add_tail(&pos->head, &tmp_list);
		} else {
			DRM_ERROR("VSP: failed to find %d bo: %x\n",
				  idx, surf_handler);
			ret = -1;
			goto out;
		}

		/* assign sequence number
		 * FIXME: do we need fc lock for sequence read?
		 */
		if (fence) {
			VSP_DEBUG("fence sequence %x,pic_idx %d,surf %x\n",
				  fence->sequence, idx,
				  pic_param->output_picture[idx].surface_id);

			pic_param->output_picture[idx].surface_id =
				fence->sequence;
			ttm_fence_object_unref(&fence);
		}
	}

	/* just fence pic param if this is not end command */
	/* only send last output fence_arg back */
	psb_fence_or_sync(priv, VSP_ENGINE_VPP, fence_type,
			  arg->fence_flags, validate_list,
			  fence_arg, &fence);
	if (fence) {
		VSP_DEBUG("fence sequence %x at output pic %d\n",
			  fence->sequence, idx);
		pic_param->output_picture[idx].surface_id = fence->sequence;
		ttm_fence_object_unref(&fence);
	}

	/* add surface back into validate_list */
	list_for_each_entry_safe(pos, next, &tmp_list, head) {
		list_add_tail(&pos->head, validate_list);
	}
out:
	ttm_bo_kunmap(&pic_param_kmap);

	return ret;
}

uint32_t vsp_fence_poll(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv;
	unsigned int rd, wr;
	unsigned int idx;
	unsigned int msg_num;
	uint32_t sequence;
	unsigned long irq_flags;
	struct vss_response_t *msg;

	VSP_DEBUG("polling vsp msg\n");

	vsp_priv = dev_priv->vsp_private;
	sequence = vsp_priv->current_sequence;

	spin_lock_irqsave(&vsp_priv->lock, irq_flags);

	msg_num = 0;
	rd = vsp_priv->ctrl->ack_rd;
	wr = vsp_priv->ctrl->ack_wr;

	msg_num = wr > rd ? wr - rd : wr == rd ? 0 :
		VSP_ACK_QUEUE_SIZE - (rd - wr);

	VSP_DEBUG("ack rd %d wr %d, msg_num %d, size %d\n", rd, wr,
		  msg_num, VSP_ACK_QUEUE_SIZE);
	VSP_DEBUG("polling fence, and there're %d msgs\n", msg_num);

	for (idx = 0; idx < msg_num; ++idx) {
		msg = vsp_priv->ack_queue + (idx + rd) % VSP_ACK_QUEUE_SIZE;

		switch (msg->type) {
		case VssErrorResponse:
			/* FIXME: should handling error? */
			handle_error_response(msg->buffer);
			DRM_ERROR("VSP: vss error response, skip for polling");
			DRM_ERROR(" errono %d type %x\n", msg->buffer,
				  msg->type);
			break;

		case VssOutputSurfaceReadyResponse:
			VSP_DEBUG("sequence %x is done\n", msg->buffer);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);

			vsp_priv->vss_cc_acc += msg->vss_cc;
			VSP_PERF("VSP output ready, clock count is %ld\n",
				vsp_priv->vss_cc_acc);

			vsp_priv->current_sequence = msg->buffer;
			sequence = msg->buffer;
			break;

		case VssOutputSurfaceFreeResponse:
			VSP_DEBUG("sequence surface %x should be freed\n",
				  msg->buffer);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssOutputSurfaceCrcResponse:
			VSP_DEBUG("Crc of sequence %x is %x\n", msg->buffer,
				  msg->vss_cc);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssInputSurfaceReadyResponse:
			VSP_DEBUG("got input surface ready response\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssCommandBufferReadyResponse:
			VSP_DEBUG("got command buffer ready response\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssEndOfSequenceResponse:
			VSP_DEBUG("end of the sequence received\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		case VssIdleResponse:
			VSP_DEBUG("VSP is idle, can be powered off\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
			break;

		default:
			VSP_DEBUG("other response, skip for polling\n");
			VSP_DEBUG("should correctly handle this response\n");
			break;
		}

		vsp_priv->ctrl->ack_rd =
			(vsp_priv->ctrl->ack_rd + 1) % VSP_ACK_QUEUE_SIZE;
	}

	spin_unlock_irqrestore(&vsp_priv->lock, irq_flags);

	return sequence;
}

void vsp_new_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int ctx_uninit_ack;
	unsigned int ctx_init_req;
	const int ctx_id = 0;

	dev_priv = dev->dev_private;
	if (dev_priv == NULL) {
		DRM_ERROR("VSP: drm driver is not initialized correctly\n");
		return;
	}

	vsp_priv = dev_priv->vsp_private;
	if (vsp_priv == NULL) {
		DRM_ERROR("VSP: vsp driver is not initialized correctly\n");
		return;
	}

	ctx_uninit_ack = vsp_priv->ctrl->context_uninit_ack;
	ctx_init_req = vsp_priv->ctrl->context_init_req;

	if (VSP_READ_FLAG(ctx_uninit_ack, ctx_id) !=
	    VSP_READ_FLAG(ctx_init_req, ctx_id)) {
		DRM_ERROR("VSP: context is not validated\n");
		return;
	}

	VSP_REVERT_FLAG(ctx_init_req, ctx_id);
	vsp_priv->ctrl->context_init_req = ctx_init_req;

	vsp_priv->vss_cc_acc = 0;

	return;
}

void vsp_rm_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int ctx_uninit_req;
	unsigned int ctx_uninit_ack;
	const int ctx_id = 0;

	dev_priv = dev->dev_private;
	if (dev_priv == NULL) {
		DRM_ERROR("VSP: new vsp context, drm driver is not"
			  " initialized correctly\n");
		return;
	}

	vsp_priv = dev_priv->vsp_private;
	if (vsp_priv == NULL) {
		DRM_ERROR("VSP: new vsp context, vsp driver is not"
			  " initialized correctly\n");
		return;
	}

	if (vsp_priv->ctrl == NULL)
		return;

	ctx_uninit_req = vsp_priv->ctrl->context_uninit_req;
	ctx_uninit_ack = vsp_priv->ctrl->context_uninit_ack;

	if (VSP_READ_FLAG(ctx_uninit_req, ctx_id) ==
	    VSP_READ_FLAG(ctx_uninit_ack, ctx_id)) {
		DRM_ERROR("VSP: context invalid\n");
		return;
	}

	VSP_REVERT_FLAG(ctx_uninit_ack, ctx_id);
	vsp_priv->ctrl->context_uninit_ack = ctx_uninit_ack;

	vsp_priv->ctrl->entry_kind = vsp_exit;
	do {
		udelay(10);
	} while (!vsp_is_idle(dev_priv, vsp_priv->config.api_processor));

	vsp_priv->needs_reset = 1;

	/* FIXME: frequency should change */
	VSP_PERF("the total time spend on VSP is %ld ms\n",
		 div_u64(vsp_priv->vss_cc_acc, 200 * 1000));

	return;
}

int psb_vsp_save_context(struct drm_device *dev)
{
	/* save the VSP info */
	return 0;
}

int psb_vsp_restore_context(struct drm_device *dev)
{
	/* restore the VSP info */
	return 0;
}

int psb_check_vsp_idle(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	if (vsp_priv->fw_loaded == 0)
		return 0;

	if (vsp_priv->vsp_busy) {
		PSB_DEBUG_PM("VSP: %s return busy", __func__);
		return -EBUSY;
	}

	return 0;
}

inline int psb_try_power_down_vsp(struct drm_device *dev)
{
	ospm_apm_power_down_vsp(dev);
	return 0;
}

void check_invalid_cmd_type(unsigned int info)
{
	switch (info >> ERR_INFO_SHIFT) {
	case VssProcSharpenParameterCommand:
		DRM_ERROR("VSP: Sharpen parameter command is received ");
		DRM_ERROR("before pipeline command %x\n", info);
		break;

	case VssProcDenoiseParameterCommand:
		DRM_ERROR("VSP: Denoise parameter command is received ");
		DRM_ERROR("before pipeline command %x\n", info);
		break;

	case VssProcColorEnhancementParameterCommand:
		DRM_ERROR("VSP: color enhancer parameter command is received");
		DRM_ERROR("before pipeline command %x\n", info);
		break;

	case VssProcFrcParameterCommand:
		DRM_ERROR("VSP: Frc parameter command is received ");
		DRM_ERROR("before pipeline command %x\n", info);
		break;

	case VssProcPictureCommand:
		DRM_ERROR("VSP: Picture parameter command is received ");
		DRM_ERROR("before pipeline command %x\n", info);
		break;

	default:
		DRM_ERROR("VSP: Unknown command type %x\n",
			  info >> ERR_INFO_SHIFT);
		break;
	}

	return;
}

void check_invalid_cmd_arg(unsigned int info)
{
	switch (info >> ERR_INFO_SHIFT) {
	case VssProcDenoiseParameterCommand:
		DRM_ERROR("VSP: unsupport value for denoise parameter\n");
		break;
	default:
		DRM_ERROR("VSP: input frame resolution is different"
			  "from previous command\n");
		break;
	}

	return;
}

void handle_error_response(unsigned int error)
{
	switch (error & ERR_ID_MASK) {
	case VssInvalidCommandType:
		check_invalid_cmd_type(error);
		DRM_ERROR("VSP: Invalid command\n");
		break;
	case VssInvalidCommandArgument:
		check_invalid_cmd_arg(error);
		DRM_ERROR("VSP: Invalid command\n");
		break;
	case VssInvalidProcPictureCommand:
		DRM_ERROR("VSP: wrong num of input/output\n");
		break;

	default:
		DRM_ERROR("VSP: Unknown error, code %d\n", error);
		break;
	}

}
