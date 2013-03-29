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

#include "psb_powermgmt.h"

#define ERR_ID_MASK 0xFFFF
#define ERR_INFO_SHIFT 16
#define PARTITIONS_MAX 9

static int vsp_submit_cmdbuf(struct drm_device *dev,
			     unsigned char *cmd_start,
			     unsigned long cmd_size);
static int vsp_send_command(struct drm_device *dev,
			    unsigned char *cmd_start,
			    unsigned long cmd_size);
static int vsp_prehandle_command(struct drm_file *priv,
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
static int vsp_fence_vp8enc_surfaces(struct drm_file *priv,
				struct list_head *validate_list,
				uint32_t fence_type,
				struct drm_psb_cmdbuf_arg *arg,
				struct psb_ttm_fence_rep *fence_arg,
				struct ttm_buffer_object *pic_param_bo,
				struct ttm_buffer_object *coded_buf_bo);

static inline void psb_clflush(void *addr)
{
	__asm__ __volatile__("wbinvd ");
}

int vsp_handle_response(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	int ret = 0;
	unsigned int rd, wr;
	unsigned int idx;
	unsigned int msg_num;
	struct vss_response_t *msg;
	uint32_t sequence;

	rd = vsp_priv->ctrl->ack_rd;
	wr = vsp_priv->ctrl->ack_wr;
	msg_num = wr > rd ? wr - rd : wr == rd ? 0 :
		VSP_ACK_QUEUE_SIZE - (rd - wr);
	VSP_DEBUG("ack rd %d wr %d, msg_num %d, size %d\n",
		  rd, wr, msg_num, VSP_ACK_QUEUE_SIZE);

	sequence = vsp_priv->current_sequence;
	for (idx = 0; idx < msg_num; ++idx) {

		msg = vsp_priv->ack_queue + (idx + rd) % VSP_ACK_QUEUE_SIZE;
		VSP_DEBUG("ack[%d]->type = %x\n", idx, msg->type);

		switch (msg->type) {
		case VssErrorResponse:
		case VssErrorResponse_VP8:
			DRM_ERROR("error response:%.8x %.8x %.8x %.8x %.8x\n",
				  msg->context, msg->type, msg->buffer,
				  msg->size, msg->vss_cc);
			handle_error_response(msg->buffer);
			ret = false;
			break;

		case VssEndOfSequenceResponse:
		case VssEndOfSequenceResponse_VP8:
			VSP_DEBUG("end of the sequence received\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;

			break;

		case VssOutputSurfaceReadyResponse:
			VSP_DEBUG("sequence %x is done!!\n", msg->buffer);
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;
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
		case VssIdleResponse_VP8:
		{
			unsigned int cmd_rd, cmd_wr;

			VSP_DEBUG("VSP is idle\n");
			VSP_DEBUG("VSP clock cycles from pre response %x\n",
				  msg->vss_cc);
			vsp_priv->vss_cc_acc += msg->vss_cc;

			cmd_rd = vsp_priv->ctrl->cmd_rd;
			cmd_wr = vsp_priv->ctrl->cmd_wr;

			VSP_DEBUG("cmd_rd=%d, cmd_wr=%d\n", cmd_rd, cmd_wr);
			vsp_priv->vsp_state = VSP_STATE_IDLE;


			/* If there is still commands in the cmd buffer,
			 * set CONTINUE command and start API main directly not
			 * via the boot program. The boot program might damage
			 * the application state.
			 */
			if (cmd_rd == cmd_wr) {
				vsp_priv->vsp_state = VSP_STATE_IDLE;
#if 0
				if (!vsp_priv->handling_cmd) {
					VSP_DEBUG("Trying to shut down ...\n");
					schedule_delayed_work(
						&vsp_priv->vsp_suspend_wq, 0);
				}
#endif
			} else {
				VSP_DEBUG("cmd_queue has data,continue...\n");
				vsp_continue_function(dev_priv);
			}
			break;
		}
		case VssVp8encSetSequenceParametersResponse:
			VSP_DEBUG("receive vp8 sequence response\n");
			break;
		case VssVp8encEncodeFrameResponse:
		{
			struct VssVp8encPictureParameterBuffer *t =
				vsp_priv->vp8_encode_frame_cmd[vsp_priv->rd];
			struct VssVp8encEncodedFrame *encoded_frame =
				(struct VssVp8encEncodedFrame *)
					(vsp_priv->coded_buf);
			int i = 0;
			int j = 0;
			VSP_DEBUG("receive vp8 encoded frame buffer %x",
					msg->buffer);
			VSP_DEBUG("size %x cur command id is %d\n",
					msg->size, vsp_priv->rd);
			VSP_DEBUG("read vp8 pic param address %x at %d\n",
				vsp_priv->vp8_encode_frame_cmd[vsp_priv->rd],
				vsp_priv->rd);

			psb_clflush(vsp_priv->coded_buf);

			sequence = t->input_frame.surface_id;
			VSP_DEBUG("sequence %x\n", sequence);
			VSP_DEBUG("size %d\n", t->encoded_frame_size);

			VSP_DEBUG("status[0=success] %x\n",
					encoded_frame->status);
			VSP_DEBUG("frame flags[1=Key, 0=Non-key] %d\n",
					encoded_frame->frame_flags);
			VSP_DEBUG("segments = %d\n", encoded_frame->segments);
			VSP_DEBUG("frame size %d[bytes]\n",
					encoded_frame->frame_size);
			VSP_DEBUG("partitions num %d\n",
					encoded_frame->partitions);
			VSP_DEBUG("coded data start %p\n",
					encoded_frame->coded_data);

			if (encoded_frame->partitions > PARTITIONS_MAX) {
				VSP_DEBUG("partitions num error\n");
				encoded_frame->partitions = PARTITIONS_MAX;
			}

			for (; i < encoded_frame->partitions; i++) {
				VSP_DEBUG("%d partitions size %d start %x\n", i,
					encoded_frame->partition_size[i],
					encoded_frame->partition_start[i]);
			}

			/* dump coded buf for debug */
			int size = sizeof(struct VssVp8encEncodedFrame);
			char *tmp = (char *) (vsp_priv->coded_buf);
			for (j = 0; j < 32; j++) {
				VSP_DEBUG("coded buf is: %d, %x\n", j,
					*(tmp + size - 4 + j));
			}

			vsp_priv->rd++;
			if (vsp_priv->rd >= VSP_CMD_QUEUE_SIZE)
				vsp_priv->rd = 0;

			break;
		}
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
	}

	return sequence;
}

bool vsp_interrupt(void *pvData)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct vsp_private *vsp_priv;
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
	if (!(status & (1 << VSP_SP0_IRQ_SHIFT))) {
		DRM_ERROR("VSP: invalid irq\n");
		return false;
	} else {
		IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_CLR, (1 << VSP_SP0_IRQ_SHIFT));
	}

	/* handle the response message */
	spin_lock(&vsp_priv->lock);
	sequence = vsp_handle_response(dev_priv);
	spin_unlock(&vsp_priv->lock);

	/* handle fence info */
	if (sequence != vsp_priv->current_sequence) {
		vsp_priv->current_sequence = sequence;
		psb_fence_handler(dev, VSP_ENGINE_VPP);
	}

	VSP_DEBUG("will leave interrupt\n");
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

	/* handle the Context and Fence command */
	VSP_DEBUG("handle Context and Fence commands\n");
	ret = vsp_prehandle_command(priv, validate_list, fence_type, arg,
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

	vsp_priv->num_cmd = 1;

	if (vsp_priv->fw_loaded == 0) {
		ret = vsp_init_fw(dev);
		if (ret != 0) {
			DRM_ERROR("VSP: failed to load firmware\n");
			return -EFAULT;
		}
	}

	/* consider to invalidate/flush MMU */
	if (vsp_priv->vsp_state == VSP_STATE_DOWN) {
		VSP_DEBUG("needs reset\n");

		if (vsp_reset(dev_priv)) {
			ret = -EBUSY;
			DRM_ERROR("VSP: failed to reset\n");
			return ret;
		}
	}

	/* submit command to HW */
	ret = vsp_send_command(dev, cmd_start, cmd_size);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to send command\n");
		return ret;
	}

	/* If the VSP is ind idle, need to send "Continue" */
	if (vsp_priv->vsp_state == VSP_STATE_IDLE) {
		vsp_continue_function(dev_priv);
		VSP_DEBUG("The VSP is on idle, send continue!\n");
	}

	/* If the VSP is in Suspend, need to send "Resume" */
	if (vsp_priv->vsp_state == VSP_STATE_SUSPEND) {
		vsp_resume_function(dev_priv);
		VSP_DEBUG("The VSP is on suspend, send resume!\n");
	}

	vsp_priv->num_cmd = 0;

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
	unsigned int cmd_idx, num_cmd = 0;
	struct vss_command_t *cur_cmd, *cur_cell_cmd;

	VSP_DEBUG("will send command here: cmd_start %p, cmd_size %ld\n",
		  cmd_start, cmd_size);

	cur_cmd = (struct vss_command_t *)cmd_start;

	/* if the VSP in suspend, update the saved config info */
	if (vsp_priv->vsp_state == VSP_STATE_SUSPEND) {
		VSP_DEBUG("In suspend, need update saved cmd_wr!\n");
		vsp_priv->ctrl = (struct vsp_ctrl_reg *)
				 &(vsp_priv->saved_config_regs[2]);
	}

	while (cmd_size) {
		rd = vsp_priv->ctrl->cmd_rd;
		wr = vsp_priv->ctrl->cmd_wr;

		remaining_space = rd >= wr + 1 ? rd - wr - 1 :
			VSP_CMD_QUEUE_SIZE - (wr + 1 - rd) ;

		VSP_DEBUG("VSP: rd %d, wr %d, remaining_space %d, ",
			  rd, wr, remaining_space);
		VSP_DEBUG("cmd_size %ld sizeof(*cur_cmd) %d\n",
			  cmd_size, sizeof(*cur_cmd));

		if (remaining_space < vsp_priv->num_cmd) {
			DRM_ERROR("no enough space for cmd queue\n");
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

		for (cmd_idx = 0; cmd_idx < remaining_space;) {
			VSP_DEBUG("current cmd type %x\n", cur_cmd->type);
			if (cur_cmd->type == VspFencePictureParamCommand) {
				VSP_DEBUG("skip VspFencePictureParamCommand");
				cur_cmd++;
				cmd_size -= sizeof(*cur_cmd);
				VSP_DEBUG("first cmd_size %ld\n", cmd_size);
				if (cmd_size == 0)
					goto out;
				else
					continue;
			} else if (cur_cmd->type == VspSetContextCommand ||
				cur_cmd->type == Vss_Sys_STATE_BUF_COMMAND) {
				VSP_DEBUG("skip VspSetContextCommand");
				cur_cmd++;
				cmd_size -= sizeof(*cur_cmd);
				if (cmd_size == 0)
					goto out;
				else
					continue;
			}

			/* FIXME: could remove cmd_idx here */
			cur_cell_cmd = vsp_priv->cmd_queue +
				(wr + cmd_idx) % VSP_CMD_QUEUE_SIZE;
			++cmd_idx;
			memcpy(cur_cell_cmd, cur_cmd, sizeof(*cur_cmd));
			VSP_DEBUG("cmd: %.8x %.8x %.8x %.8x %.8x %.8x\n",
				cur_cell_cmd->context, cur_cell_cmd->type,
				cur_cell_cmd->buffer, cur_cell_cmd->size,
				cur_cell_cmd->buffer_id, cur_cell_cmd->irq);
			num_cmd++;
			cur_cmd++;
			cmd_size -= sizeof(*cur_cmd);
			if (cmd_size == 0)
				goto out;
			else if (cmd_size < sizeof(*cur_cmd)) {
				DRM_ERROR("invalid command size %ld\n",
					  cmd_size);
				goto out;
			}
		}
	}
out:
	/* update write index */
	VSP_DEBUG("%d cmd will send to VSP!\n", num_cmd);
	vsp_priv->ctrl->cmd_wr =
		(vsp_priv->ctrl->cmd_wr + num_cmd) % VSP_CMD_QUEUE_SIZE;

	return 0;
}

static int vsp_prehandle_command(struct drm_file *priv,
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
	struct drm_device *dev = priv->minor->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	struct ttm_buffer_object *pic_bo_vp8;
	struct ttm_buffer_object *coded_buf_bo;
	int vp8_pic_num = 0;

	cur_cmd = (struct vss_command_t *)cmd_start;

	pic_param_num = 0;
	VSP_DEBUG("cmd size %d\n", cmd_size);
	while (cmd_size) {
		VSP_DEBUG("cmd type %x, buffer offset %x\n", cur_cmd->type,
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
				DRM_ERROR("pic_param_num invalid(%d)!\n",
					  pic_param_num);
				ret = -1;
				goto out;
			}
		} else if (cur_cmd->type == VspSetContextCommand ||
				cur_cmd->type == Vss_Sys_STATE_BUF_COMMAND) {

			VSP_DEBUG("set context and new vsp context\n");
			VSP_DEBUG("set context base %x, size %x\n",
				  cur_cmd->buffer, cur_cmd->size);

			vsp_priv->setting->state_buffer_size = cur_cmd->size;
			vsp_priv->setting->state_buffer_addr = cur_cmd->buffer;
			vsp_priv->fw_type = cur_cmd->type;

			vsp_new_context(dev_priv->dev);
		} else
			/* calculate the numbers of cmd send to VSP */
			vsp_priv->num_cmd = vsp_priv->num_cmd + 1;

		if (cur_cmd->type == VssVp8encEncodeFrameCommand) {
			pic_bo_vp8 =
				ttm_buffer_object_lookup(tfile,
						cur_cmd->reserved7);

			if (pic_bo_vp8 == NULL) {
				DRM_ERROR("VSP: failed to find %x bo\n",
					cur_cmd->reserved7);
				ret = -1;
				goto out;
			}

			coded_buf_bo =
				ttm_buffer_object_lookup(tfile,
						cur_cmd->reserved6);
			if (coded_buf_bo == NULL) {
				DRM_ERROR("VSP: failed to find %x bo\n",
					cur_cmd->reserved6);
				ret = -1;
				goto out;
			}

			vp8_pic_num++;
			VSP_DEBUG("find pic param buffer: id %x, offset %lx\n",
				cur_cmd->reserved7, pic_bo_vp8->offset);
			VSP_DEBUG("pic param placement %x bus.add %p\n",
				pic_bo_vp8->mem.placement,
				pic_bo_vp8->mem.bus.addr);
			if (pic_param_num > 1) {
				DRM_ERROR("should be only 1 pic param cmd\n");
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
	} else if (vp8_pic_num > 0) {
		ret = vsp_fence_vp8enc_surfaces(priv, validate_list,
					fence_type, arg,
					fence_arg, pic_bo_vp8, coded_buf_bo);
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

static int vsp_fence_vp8enc_surfaces(struct drm_file *priv,
				struct list_head *validate_list,
				uint32_t fence_type,
				struct drm_psb_cmdbuf_arg *arg,
				struct psb_ttm_fence_rep *fence_arg,
				struct ttm_buffer_object *pic_param_bo,
				struct ttm_buffer_object *coded_buf_bo)
{
	struct psb_ttm_fence_rep local_fence_arg;
	bool is_iomem;
	int ret = 0;
	struct VssVp8encPictureParameterBuffer *pic_param;
	uint32_t surf_handler;
	struct ttm_buffer_object *surf_bo;
	struct ttm_fence_object *fence = NULL;
	struct list_head surf_list;
	struct ttm_validate_buffer *pos, *next, *cur_valid_buf;
	struct ttm_object_file *tfile = BCVideoGetPriv(priv)->tfile;
	struct drm_device *dev = priv->minor->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	INIT_LIST_HEAD(&surf_list);

	/* map pic param */
	ret = ttm_bo_kmap(pic_param_bo, 0, pic_param_bo->num_pages,
			  &vsp_priv->vp8_encode_frame__kmap[vsp_priv->wr]);
	if (ret) {
		DRM_ERROR("VSP: ttm_bo_kmap failed: %d\n", ret);
		goto out;
	}

	pic_param = (struct VssVp8encPictureParameterBuffer *)
		ttm_kmap_obj_virtual(
				&vsp_priv->vp8_encode_frame__kmap[vsp_priv->wr],
				&is_iomem);

	VSP_DEBUG("save vp8 pic param address %x at %d\n",
			pic_param, vsp_priv->wr);

	vsp_priv->vp8_encode_frame_cmd[vsp_priv->wr] = pic_param;
	vsp_priv->wr++;

	VSP_DEBUG("id %d bo addr %x  kernel addr %x surfaceid %x base %x\n",
			vsp_priv->wr-1,
			pic_param_bo,
			pic_param,
			pic_param->input_frame.surface_id,
			pic_param->input_frame.base);
	VSP_DEBUG("pic_param->encoded_frame_base = %p\n",
			pic_param->encoded_frame_base);

	/* map coded buffer */
	ret = ttm_bo_kmap(coded_buf_bo, 0, coded_buf_bo->num_pages,
			  &vsp_priv->coded_buf_kmap);
	if (ret) {
		DRM_ERROR("VSP: ttm_bo_kmap failed: %d\n", ret);
		goto out;
	}

	vsp_priv->coded_buf = (void *)
		ttm_kmap_obj_virtual(
				&vsp_priv->coded_buf_kmap,
				&is_iomem);

	if (vsp_priv->wr >= VSP_CMD_QUEUE_SIZE)
		vsp_priv->wr = 0;

	/* just fence pic param if this is not end command */
	/* only send last output fence_arg back */
	psb_fence_or_sync(priv, VSP_ENGINE_VPP, fence_type,
			  arg->fence_flags, validate_list,
			  fence_arg, &fence);
	if (fence) {
		VSP_DEBUG("vp8 fence sequence %x at output pic %x\n",
			  fence->sequence);
		pic_param->input_frame.surface_id = fence->sequence;
		ttm_fence_object_unref(&fence);
	} else {
		VSP_DEBUG("NO fence?????\n");
	}

out:
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

	/* handle the response message */
	spin_lock_irqsave(&vsp_priv->lock, irq_flags);
	sequence = vsp_handle_response(dev_priv);
	spin_unlock_irqrestore(&vsp_priv->lock, irq_flags);

	if (sequence != vsp_priv->current_sequence)
		vsp_priv->current_sequence = sequence;

	return sequence;
}

void vsp_new_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

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
	vsp_priv->vss_cc_acc = 0;

	return;
}

void vsp_rm_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	dev_priv = dev->dev_private;
	if (dev_priv == NULL) {
		DRM_ERROR("RM context, but dev_priv is NULL");
		return;
	}

	vsp_priv = dev_priv->vsp_private;
	if (vsp_priv == NULL) {
		DRM_ERROR("RM context, but vsp_priv is NULL");
		return;
	}

	if (vsp_priv->ctrl == NULL)
		return;

	vsp_priv->ctrl->entry_kind = vsp_exit;

	schedule_delayed_work(&vsp_priv->vsp_suspend_wq, 0);

	vsp_priv->vsp_state = VSP_STATE_DOWN;
	vsp_priv->fw_loaded = 0;
	vsp_priv->current_sequence = 0;
	vsp_priv->num_cmd = 0;
	VSP_DEBUG("VSP: OK. Power down the HW!\n");

	/* FIXME: frequency should change */
	VSP_PERF("the total time spend on VSP is %ld ms\n",
		 div_u64(vsp_priv->vss_cc_acc, 200 * 1000));

	return;
}

int psb_vsp_save_context(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int i;

	/* save the VSP config registers */
	for (i = 2; i < VSP_CONFIG_SIZE; i++)
		CONFIG_REG_READ32(i, &(vsp_priv->saved_config_regs[i]));

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

	if (vsp_priv->fw_loaded == 0 || vsp_priv->vsp_state == VSP_STATE_DOWN)
		return 0;

	/* make sure VSP system has really been idle
	 * vsp-api runs on sp0 or sp1, but we don't know which one when booting
	 * securely. So wait for both.
	 */
	if (!vsp_is_idle(dev_priv, vsp_sp0)) {
		PSB_DEBUG_PM("VSP: sp0 return busy!\n");
		return -EBUSY;
	}
	if (!vsp_is_idle(dev_priv, vsp_sp1)) {
		PSB_DEBUG_PM("VSP: sp1 return busy!\n");
		return -EBUSY;
	}

	/* set VSP entry status */
	vsp_priv->ctrl->entry_kind = vsp_entry_booted;

	return 0;
}

int psb_vsp_dump_info(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int reg, i, j, *cmd_p;

	/* config info */
	for (i = 2; i < VSP_CONFIG_SIZE; i++) {
		CONFIG_REG_READ32(i, &reg);
		VSP_DEBUG("partition1_config_reg_d%d=%x\n", i, reg);
	}

	/* firmware*/
	VSP_DEBUG("firmware addr:%x\n", vsp_priv->firmware->offset);

	/* ma_header_reg */
	MM_READ32(vsp_priv->boot_header.ma_header_reg, 0, &reg);
	VSP_DEBUG("ma_header_reg:%x\n", reg);

	/* The setting-struct */
	VSP_DEBUG("setting addr:%x\n", vsp_priv->setting_bo->offset);
	VSP_DEBUG("setting->reserved0:%x\n",
			vsp_priv->setting->reserved0);
	VSP_DEBUG("setting->command_queue_size:0x%x\n",
			vsp_priv->setting->command_queue_size);
	VSP_DEBUG("setting->command_queue_addr:%x\n",
			vsp_priv->setting->command_queue_addr);
	VSP_DEBUG("setting->response_queue_size:0x%x\n",
			vsp_priv->setting->response_queue_size);
	VSP_DEBUG("setting->response_queue_addr:%x\n",
			vsp_priv->setting->response_queue_addr);
	VSP_DEBUG("setting->state_buffer_size:0x%x\n",
			vsp_priv->setting->state_buffer_size);
	VSP_DEBUG("setting->state_buffer_addr:%x\n",
			vsp_priv->setting->state_buffer_addr);

	/* IRQ registers */
	for (i = 0; i < 6; i++) {
		MM_READ32(0x180000, i * 4, &reg);
		VSP_DEBUG("partition1_gp_ireg_IRQ%d:%x", i, reg);
	}
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_EDGE, &reg);
	VSP_DEBUG("partition1_irq_control_irq_edge:%x\n", reg);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &reg);
	VSP_DEBUG("partition1_irq_control_irq_mask:%x\n", reg);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_STATUS, &reg);
	VSP_DEBUG("partition1_irq_control_irq_status:%x\n", reg);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_CLR, &reg);
	VSP_DEBUG("partition1_irq_control_irq_clear:%x\n", reg);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &reg);
	VSP_DEBUG("partition1_irq_control_irq_enable:%x\n", reg);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_LEVEL_PULSE, &reg);
	VSP_DEBUG("partition1_irq_control_irq_pulse:%x\n", reg);

	/* MMU table address */
	MM_READ32(MMU_TABLE_ADDR, 0x0, &reg);
	VSP_DEBUG("mmu_page_table_address:%x\n", reg);

	/* SP0 info */
	VSP_DEBUG("sp0_processor:%d\n", vsp_sp0);
	SP_REG_READ32(0x0, &reg, vsp_sp0);
	VSP_DEBUG("sp0_stat_and_ctrl:%x\n", reg);
	SP_REG_READ32(0x4, &reg, vsp_sp0);
	VSP_DEBUG("sp0_base_address:%x\n", reg);
	SP_REG_READ32(0x24, &reg, vsp_sp0);
	VSP_DEBUG("sp0_debug_pc:%x\n", reg);
	SP_REG_READ32(0x28, &reg, vsp_sp0);
	VSP_DEBUG("sp0_cfg_pmem_iam_op0:%x\n", reg);
	SP_REG_READ32(0x10, &reg, vsp_sp0);
	VSP_DEBUG("sp0_cfg_pmem_master:%x\n", reg);

	/* SP1 info */
	VSP_DEBUG("sp1_processor:%d\n", vsp_sp1);
	SP_REG_READ32(0x0, &reg, vsp_sp1);
	VSP_DEBUG("sp1_stat_and_ctrl:%x\n", reg);
	SP_REG_READ32(0x4, &reg, vsp_sp1);
	VSP_DEBUG("sp1_base_address:%x\n", reg);
	SP_REG_READ32(0x24, &reg, vsp_sp1);
	VSP_DEBUG("sp1_debug_pc:%x\n", reg);
	SP_REG_READ32(0x28, &reg, vsp_sp1);
	VSP_DEBUG("sp1_cfg_pmem_iam_op0:%x\n", reg);
	SP_REG_READ32(0x10, &reg, vsp_sp1);
	VSP_DEBUG("sp1_cfg_pmem_master:%x\n", reg);

	/* WDT info */
	for (i = 0; i < 14; i++) {
		MM_READ32(0x170000, i * 4, &reg);
		VSP_DEBUG("partition1_wdt_reg%d:%x\n", i, reg);
	}

	/* command queue */
	VSP_DEBUG("command queue:\n");
	for (i = 0; i < VSP_CMD_QUEUE_SIZE; i++) {
		cmd_p = &(vsp_priv->cmd_queue[i]);
		VSP_DEBUG("cmd[%d]:%.8x %.8x %.8x %.8x %.8x %.8x %.8x %.8x", i,
			vsp_priv->cmd_queue[i].context,
			vsp_priv->cmd_queue[i].type,
			vsp_priv->cmd_queue[i].buffer,
			vsp_priv->cmd_queue[i].size,
			vsp_priv->cmd_queue[i].buffer_id,
			vsp_priv->cmd_queue[i].irq,
			vsp_priv->cmd_queue[i].reserved6,
			vsp_priv->cmd_queue[i].reserved7);
	}

	/* response queue */
	VSP_DEBUG("ack queue:\n");
	for (i = 0; i < VSP_ACK_QUEUE_SIZE; i++) {
		cmd_p = &(vsp_priv->ack_queue[i]);
		VSP_DEBUG("ack[%d]:%.8x %.8x %.8x %.8x %.8x %.8x %.8x %.8x", i,
			vsp_priv->ack_queue[i].context,
			vsp_priv->ack_queue[i].type,
			vsp_priv->ack_queue[i].buffer,
			vsp_priv->ack_queue[i].size,
			vsp_priv->ack_queue[i].vss_cc,
			vsp_priv->ack_queue[i].reserved5,
			vsp_priv->ack_queue[i].reserved6,
			vsp_priv->ack_queue[i].reserved7);
	}

}

void psb_powerdown_vsp(struct work_struct *work)
{
	struct vsp_private *vsp_priv =
		container_of(work, struct vsp_private, vsp_suspend_wq.work);
	int ret = 0;

	if (!vsp_priv)
		return;

	ret = ospm_apm_power_down_vsp(vsp_priv->dev);

	if (ret)
		VSP_DEBUG("The VSP could NOT be powered off!\n");
	else {
		VSP_DEBUG("The VSP has been powered off!\n");
		vsp_priv->vsp_state = VSP_STATE_SUSPEND;
	}

	return;
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
		DRM_ERROR("VSP: input frame resolution is different");
		DRM_ERROR("from previous command\n");
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
	case VssInvalidDdrAddress:
		DRM_ERROR("VSP: DDR address isn't in allowed 1GB range\n");
		break;
	default:
		DRM_ERROR("VSP: Unknown error, code %x\n", error);
		break;
	}

}


