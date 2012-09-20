/**
 * file pnw_topaz.c
 * TOPAZ I/O operations and IRQ handling
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

/* include headers */
/* #define DRM_DEBUG_CODE 2 */
#include <drm/drmP.h>

#include "psb_drv.h"
#include "psb_drm.h"
#include "pnw_topaz.h"
#include "psb_powermgmt.h"
#include "pnw_topaz_hw_reg.h"
#include "lnc_topaz.h"

#include <linux/io.h>
#include <linux/delay.h>

#define TOPAZ_MAX_COMMAND_IN_QUEUE 0x1000
/* static function define */
static int pnw_topaz_deliver_command(struct drm_device *dev,
				     struct ttm_buffer_object *cmd_buffer,
				     unsigned long cmd_offset,
				     unsigned long cmd_size,
				     void **topaz_cmd, uint32_t sequence,
				     int copy_cmd);
static int pnw_topaz_send(struct drm_device *dev, void *cmd,
			  unsigned long cmd_size, uint32_t sync_seq);
static int pnw_topaz_dequeue_send(struct drm_device *dev);
static int pnw_topaz_save_command(struct drm_device *dev, void *cmd,
				  unsigned long cmd_size, uint32_t sequence);

static void topaz_mtx_kick(struct drm_psb_private *dev_priv, uint32_t core_id,
			   uint32_t kick_count);

bool pnw_topaz_interrupt(void *pvData)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	uint32_t clr_flag;
	struct pnw_topaz_private *topaz_priv;
	uint32_t topaz_stat;
	uint32_t cur_seq, cmd_id;

	PSB_DEBUG_IRQ("Got an TopazSC interrupt\n");

	if (pvData == NULL) {
		DRM_ERROR("ERROR: TOPAZ %s, Invalid params\n", __func__);
		return false;
	}

	dev = (struct drm_device *)pvData;

	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
		DRM_ERROR("ERROR: interrupt arrived but HW is power off\n");
		return false;
	}

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	topaz_priv = dev_priv->topaz_private;

	/*TODO : check if topaz is busy */
	topaz_priv->topaz_hw_busy = REG_READ(0x20D0) & (0x1 << 11);

	TOPAZ_READ32(TOPAZ_CR_IMG_TOPAZ_INTSTAT, &topaz_stat, 0);
	clr_flag = pnw_topaz_queryirq(dev);

	pnw_topaz_clearirq(dev, clr_flag);

	TOPAZ_MTX_WB_READ32(topaz_priv->topaz_sync_addr,
			    0, MTX_WRITEBACK_CMDWORD, &cmd_id);
	cmd_id = (cmd_id & 0x7f);	/* CMD ID */
	if (cmd_id != MTX_CMDID_NULL)
		return true;

	TOPAZ_MTX_WB_READ32(topaz_priv->topaz_sync_addr,
			    0, MTX_WRITEBACK_VALUE, &cur_seq);

	PSB_DEBUG_IRQ("TOPAZ:Got SYNC IRQ,sync seq:0x%08x (MTX)"
		      " vs 0x%08x(fence)\n",
		      cur_seq, dev_priv->sequence[LNC_ENGINE_ENCODE]);

	psb_fence_handler(dev, LNC_ENGINE_ENCODE);

	topaz_priv->topaz_busy = 1;
	pnw_topaz_dequeue_send(dev);

	if (drm_topaz_pmpolicy != PSB_PMPOLICY_NOPM
	    && topaz_priv->topaz_busy == 0) {
		PSB_DEBUG_IRQ("TOPAZ:Schedule a work to power down Topaz\n");
		schedule_delayed_work(&dev_priv->scheduler.topaz_suspend_wq, 0);
	}

	return true;
}

static int pnw_submit_encode_cmdbuf(struct drm_device *dev,
				    struct ttm_buffer_object *cmd_buffer,
				    unsigned long cmd_offset,
				    unsigned long cmd_size,
				    struct ttm_fence_object *fence)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned long irq_flags;
	int ret = 0;
	void *cmd;
	uint32_t tmp;
	uint32_t sequence = dev_priv->sequence[LNC_ENGINE_ENCODE];
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: command submit\n");

	PSB_DEBUG_GENERAL("TOPAZ: topaz busy = %d\n", topaz_priv->topaz_busy);

	if (dev_priv->last_topaz_ctx != dev_priv->topaz_ctx) {
		/* todo: save current context into dev_priv->last_topaz_ctx
		 * and reload dev_priv->topaz_ctx context
		 */
		PSB_DEBUG_GENERAL("TOPAZ: context switch\n");
		dev_priv->last_topaz_ctx = dev_priv->topaz_ctx;
	}

	if (topaz_priv->topaz_fw_loaded == 0) {
		/* #.# load fw to driver */
		PSB_DEBUG_INIT("TOPAZ: load /lib/firmware/topaz_fwsc.bin\n");
		ret = pnw_topaz_init_fw(dev);
		if (ret != 0) {
			/* FIXME: find a proper return value */
			DRM_ERROR("TOPAX:load /lib/firmware/topaz_fwsc.bin"
				  " fails, ensure udevd is configured"
				  " correctly!\n");
			return -EFAULT;
		}
		topaz_priv->topaz_fw_loaded = 1;
	}

	tmp = atomic_cmpxchg(&dev_priv->topaz_mmu_invaldc, 1, 0);
	if (tmp == 1)
		pnw_topaz_mmu_flushcache(dev_priv);

	/* # schedule watchdog */
	/* psb_schedule_watchdog(dev_priv); */

	/* # spin lock irq save [msvdx_lock] */
	spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);

	/* # if topaz need to reset, reset it */
	if (topaz_priv->topaz_needs_reset) {
		/* #.# reset it */
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);
		PSB_DEBUG_GENERAL("TOPAZ: needs reset.\n");

		if (pnw_topaz_reset(dev_priv)) {
			ret = -EBUSY;
			DRM_ERROR("TOPAZ: reset failed.\n");
			return ret;
		}

		PSB_DEBUG_GENERAL("TOPAZ: reset ok.\n");

		/* #.# upload firmware */
		if (pnw_topaz_setup_fw(dev, topaz_priv->topaz_cur_codec)) {
			DRM_ERROR("TOPAZ: upload FW to HW failed\n");
			return -EBUSY;
		}

		spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);
	}

	if (!topaz_priv->topaz_busy) {
		/* # direct map topaz command if topaz is free */
		PSB_DEBUG_GENERAL("TOPAZ:direct send command,sequence %08x\n",
				  sequence);

		topaz_priv->topaz_busy = 1;
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

		ret = pnw_topaz_deliver_command(dev, cmd_buffer, cmd_offset,
						cmd_size, NULL, sequence, 0);

		if (ret) {
			DRM_ERROR("TOPAZ: failed to extract cmd...\n");
			return ret;
		}
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: queue command,sequence %08x\n",
				sequence);
		cmd = NULL;

		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

		ret = pnw_topaz_deliver_command(dev, cmd_buffer, cmd_offset,
						cmd_size, &cmd, sequence, 1);
		if (cmd == NULL || ret) {
			DRM_ERROR("TOPAZ: map command for save fialed\n");
			return ret;
		}

		ret = pnw_topaz_save_command(dev, cmd, cmd_size, sequence);
		if (ret)
			DRM_ERROR("TOPAZ: save command failed\n");
	}

	return ret;
}

static int pnw_topaz_save_command(struct drm_device *dev, void *cmd,
				  unsigned long cmd_size, uint32_t sequence)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct pnw_topaz_cmd_queue *topaz_cmd;
	unsigned long irq_flags;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: queue command,sequence: %08x..\n", sequence);

	topaz_cmd = kzalloc(sizeof(struct pnw_topaz_cmd_queue), GFP_KERNEL);
	if (topaz_cmd == NULL) {
		mutex_unlock(&topaz_priv->topaz_mutex);
		DRM_ERROR("TOPAZ: out of memory....\n");
		return -ENOMEM;
	}

	topaz_cmd->cmd = cmd;
	topaz_cmd->cmd_size = cmd_size;
	topaz_cmd->sequence = sequence;

	spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);
	list_add_tail(&topaz_cmd->head, &topaz_priv->topaz_queue);
	if (!topaz_priv->topaz_busy) {
		/* topaz_priv->topaz_busy = 1; */
		PSB_DEBUG_GENERAL("TOPAZ: need immediate dequeue...\n");
		pnw_topaz_dequeue_send(dev);
		PSB_DEBUG_GENERAL("TOPAZ: after dequeue command\n");
	}

	spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

	return 0;
}

int pnw_cmdbuf_video(struct drm_file *priv,
		     struct list_head *validate_list,
		     uint32_t fence_type,
		     struct drm_psb_cmdbuf_arg *arg,
		     struct ttm_buffer_object *cmd_buffer,
		     struct psb_ttm_fence_rep *fence_arg)
{
	struct drm_device *dev = priv->minor->dev;
	struct ttm_fence_object *fence = NULL;
	int ret;

	PSB_DEBUG_GENERAL("TOPAZ : enter %s cmdsize: %d\n", __func__,
			  arg->cmdbuf_size);

	ret = pnw_submit_encode_cmdbuf(dev, cmd_buffer, arg->cmdbuf_offset,
				       arg->cmdbuf_size, fence);
	if (ret)
		return ret;

	/* workaround for interrupt issue */
	psb_fence_or_sync(priv, LNC_ENGINE_ENCODE, fence_type, arg->fence_flags,
			  validate_list, fence_arg, &fence);

	if (fence)
		ttm_fence_object_unref(&fence);

	spin_lock(&cmd_buffer->bdev->fence_lock);
	if (cmd_buffer->sync_obj != NULL)
		ttm_fence_sync_obj_unref(&cmd_buffer->sync_obj);
	spin_unlock(&cmd_buffer->bdev->fence_lock);

	PSB_DEBUG_GENERAL("TOPAZ exit %s\n", __func__);
	return 0;
}

int pnw_wait_on_sync(struct drm_psb_private *dev_priv,
		     uint32_t sync_seq, uint32_t *sync_p)
{
	int count = 10000;
	if (sync_p == NULL) {
		DRM_ERROR("TOPAZ: pnw_wait_on_sync invalid memory address\n ");
		return -1;
	}

	while (count && (sync_seq != *sync_p)) {
		PSB_UDELAY(100);	/* experimental value */
		--count;
	}
	if ((count == 0) && (sync_seq != *sync_p)) {
		DRM_ERROR("TOPAZ: wait sycn timeout (0x%08x),actual 0x%08x\n",
			  sync_seq, *sync_p);
		return -EBUSY;
	}
	PSB_DEBUG_GENERAL("TOPAZ: SYNC done, seq=0x%08x\n", *sync_p);
	return 0;
}

int pnw_topaz_deliver_command(struct drm_device *dev,
			      struct ttm_buffer_object *cmd_buffer,
			      unsigned long cmd_offset, unsigned long cmd_size,
			      void **topaz_cmd, uint32_t sequence, int copy_cmd)
{
	unsigned long cmd_page_offset = cmd_offset & ~PAGE_MASK;
	struct ttm_bo_kmap_obj cmd_kmap;
	bool is_iomem;
	int ret;
	unsigned char *cmd_start, *tmp;

	ret = ttm_bo_kmap(cmd_buffer, cmd_offset >> PAGE_SHIFT, 2, &cmd_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: drm_bo_kmap failed: %d\n", ret);
		return ret;
	}
	cmd_start = (unsigned char *)ttm_kmap_obj_virtual(&cmd_kmap,
							  &is_iomem) +
	    cmd_page_offset;

	if (copy_cmd) {
		PSB_DEBUG_GENERAL("TOPAZ: queue commands\n");
		tmp = kzalloc(cmd_size, GFP_KERNEL);
		if (tmp == NULL) {
			ret = -ENOMEM;
			goto out;
		}
		memcpy(tmp, cmd_start, cmd_size);
		*topaz_cmd = tmp;
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: directly send the command\n");
		ret = pnw_topaz_send(dev, cmd_start, cmd_size, sequence);
		if (ret) {
			DRM_ERROR("TOPAZ: commit commands failed.\n");
			ret = -EINVAL;
		}
	}

 out:
	PSB_DEBUG_GENERAL("TOPAZ:cmd_size(%ld), sequence(%d)"
			  " copy_cmd(%d)\n", cmd_size, sequence, copy_cmd);

	ttm_bo_kunmap(&cmd_kmap);

	return ret;
}

int pnw_topaz_kick_null_cmd(struct drm_psb_private *dev_priv,
			    uint32_t core_id,
			    uint32_t wb_offset,
			    uint32_t sync_seq, uint8_t irq_enable)
{
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t cur_free_space;
	struct topaz_cmd_header cur_cmd_header;
	int ret;

	POLL_TOPAZ_FREE_FIFO_SPACE(4, 100, 10000, &cur_free_space);
	if (ret) {
		DRM_ERROR("TOPAZ: error -- ret(%d)\n", ret);
		return ret;
	}

	cur_cmd_header.core = core_id;
	cur_cmd_header.seq = sync_seq,
	    cur_cmd_header.enable_interrupt = ((irq_enable == 0) ? 0 : 1);
	cur_cmd_header.id = MTX_CMDID_NULL;

	topaz_priv->topaz_cmd_count %= MAX_TOPAZ_CMD_COUNT;
	PSB_DEBUG_GENERAL("TOPAZ: free FIFO space %d\n", cur_free_space);
	PSB_DEBUG_GENERAL("TOPAZ: write 4 words to FIFO:"
			  "0x%08x,0x%08x,0x%08x,0x%08x\n",
			  cur_cmd_header.val, 0, wb_offset, cur_cmd_header.seq);

	TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0,
				cur_cmd_header.val);
	TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0, 0);
	TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0, wb_offset);
	TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0, sync_seq);

	PSB_DEBUG_GENERAL("TOPAZ: Write back value for NULL CMD is %d\n",
			  sync_seq);

	topaz_mtx_kick(dev_priv, 0, 1);

	return 0;
}

static void pnw_topaz_save_bias_table(struct pnw_topaz_private *topaz_priv,
				      const void *cmd, int byte_size, int core)
{
	PSB_DEBUG_GENERAL("TOPAZ: Save BIAS table(size %d) for core %d\n",
			  byte_size, core);

	if (byte_size > PNW_TOPAZ_BIAS_TABLE_MAX_SIZE) {
		DRM_ERROR("Invalid BIAS table size %d!\n", byte_size);
		return;
	}

	if (core > (topaz_priv->topaz_num_cores - 1)) {
		DRM_ERROR("Invalid core id %d\n", core);
		return;
	}

	if (topaz_priv->topaz_bias_table[core] == NULL) {
		topaz_priv->topaz_bias_table[core] =
		    kmalloc(PNW_TOPAZ_BIAS_TABLE_MAX_SIZE, GFP_KERNEL);
		if (NULL == topaz_priv->topaz_bias_table[core]) {
			DRM_ERROR("Run out of memory!\n");
			return;
		}
	}

	memcpy(topaz_priv->topaz_bias_table[core], cmd, byte_size);
	return;
}

int
pnw_topaz_send(struct drm_device *dev, void *cmd,
	       unsigned long cmd_size, uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret = 0;
	unsigned char *command = (unsigned char *)cmd;
	struct topaz_cmd_header *cur_cmd_header;
	uint32_t cur_cmd_size = 4, cur_cmd_id, cur_free_space = 0;
	uint32_t codec;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t reg_off, reg_val, reg_cnt;
	uint32_t *p_command;

	PSB_DEBUG_GENERAL("TOPAZ: send the command in the buffer one by one\n");

	while (cmd_size > 0) {
		cur_cmd_header = (struct topaz_cmd_header *)command;
		cur_cmd_id = cur_cmd_header->id;
		PSB_DEBUG_GENERAL("TOPAZ: %s:\n", cmd_to_string(cur_cmd_id));

		switch (cur_cmd_id) {
		case MTX_CMDID_SW_NEW_CODEC:
			codec = *((uint32_t *) cmd + 1);
			topaz_priv->frame_h = (uint16_t)
			    ((*((uint32_t *) cmd + 2)) & 0xffff);
			topaz_priv->frame_w = (uint16_t)
			    (((*((uint32_t *) cmd + 2))
			      & 0xffff0000) >> 16);

			PSB_DEBUG_GENERAL("TOPAZ: setup new codec %s (%d),"
					  " width %d, height %d\n",
					  codec_to_string(codec), codec,
					  topaz_priv->frame_w,
					  topaz_priv->frame_h);
			if (pnw_topaz_setup_fw(dev, codec)) {
				DRM_ERROR("TOPAZ: upload FW to HW failed\n");
				return -EBUSY;
			}
			topaz_priv->topaz_cur_codec = codec;
			cur_cmd_size = 3;
			break;

		case MTX_CMDID_SW_ENTER_LOWPOWER:
			PSB_DEBUG_GENERAL("TOPAZ: enter lowpower....\n");
			PSB_DEBUG_GENERAL("XXX: implement it\n");
			cur_cmd_size = 1;
			break;

		case MTX_CMDID_SW_LEAVE_LOWPOWER:
			PSB_DEBUG_GENERAL("TOPAZ: leave lowpower...\n");
			PSB_DEBUG_GENERAL("XXX: implement it\n");
			cur_cmd_size = 1;
			break;

		case MTX_CMDID_SW_WRITEREG:
			p_command = (uint32_t *) (command);
			p_command++;
			cur_cmd_size = *p_command;

			if ((drm_topaz_pmpolicy != PSB_PMPOLICY_NOPM) &&
			    PNW_IS_H264_ENC(topaz_priv->topaz_cur_codec)) {
				pnw_topaz_save_bias_table(topaz_priv,
							  (const void *)command,
							  (cur_cmd_size * 2 +
							   2) * 4,
							  cur_cmd_header->core);
			}

			p_command++;
			PSB_DEBUG_GENERAL("TOPAZ: Start to write"
					  " %d Registers\n", cur_cmd_size);
			if (cur_cmd_size > (cmd_size / 4)) {
				DRM_ERROR("TOPAZ: Wrong number of write "
					  "operations.Exceed command buffer."
					  "(%d)\n", (int)(cmd_size / 4));
				goto out;
			}

			for (reg_cnt = 0; reg_cnt < cur_cmd_size; reg_cnt++) {
				reg_off = *p_command;
				p_command++;
				reg_val = *p_command;
				p_command++;
				if (reg_off > TOPAZSC_REG_OFF_MAX)
					DRM_ERROR("TOPAZ: Ignore write (0x%08x)"
						  " to register 0x%08x\n",
						  reg_val, reg_off);
				else
					MM_WRITE32(0, reg_off, reg_val);
			}
			/* Reg_off and reg_val are stored in a pair of words */
			cur_cmd_size *= 2;
			/* Header size, 2 words */
			cur_cmd_size += 2;
			break;
		case MTX_CMDID_PAD:
			/*Ignore this command, which is used to skip
			 * some commands in user space*/
			cur_cmd_size = 4;
			break;
			/* ordinary commmand */
		case MTX_CMDID_START_PIC:
		case MTX_CMDID_DO_HEADER:
		case MTX_CMDID_ENCODE_SLICE:
		case MTX_CMDID_END_PIC:
		case MTX_CMDID_SETQUANT:
		case MTX_CMDID_RESET_ENCODE:
		case MTX_CMDID_ISSUEBUFF:
		case MTX_CMDID_SETUP:
		case MTX_CMDID_NULL:
			cur_cmd_header->seq = topaz_priv->topaz_cmd_count++;
			cur_cmd_header->enable_interrupt = 0;
			cur_cmd_size = 4;
			if (cur_free_space < cur_cmd_size) {
				POLL_TOPAZ_FREE_FIFO_SPACE(4, 100, 10000,
							   &cur_free_space);
				if (ret) {
					DRM_ERROR("TOPAZ: error -- ret(%d)\n",
						  ret);
					goto out;
				}
			}

			PSB_DEBUG_GENERAL("TOPAZ: free FIFO space %d\n",
					  cur_free_space);
			PSB_DEBUG_GENERAL("TOPAZ: write 4 words to FIFO:"
					  "0x%08x,0x%08x,0x%08x,0x%08x\n",
					  cur_cmd_header->val,
					  *((uint32_t *) (command) + 1),
					  TOPAZ_MTX_WB_OFFSET(topaz_priv->
							      topaz_wb_offset,
							      cur_cmd_header->
							      core),
					  cur_cmd_header->seq);

			TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0,
						cur_cmd_header->val);
			TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0,
						*((uint32_t *) (command) + 1));
			TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0,
						TOPAZ_MTX_WB_OFFSET(topaz_priv->
							topaz_wb_offset,
							cur_cmd_header->core));
			TOPAZ_MULTICORE_WRITE32(TOPAZSC_CR_MULTICORE_CMD_FIFO_0,
						cur_cmd_header->seq);

			cur_free_space -= 4;
			topaz_priv->topaz_cmd_count %= MAX_TOPAZ_CMD_COUNT;
			topaz_mtx_kick(dev_priv, 0, 1);
#ifdef SYNC_FOR_EACH_COMMAND
			pnw_wait_on_sync(dev_priv, cur_cmd_header->seq,
					 topaz_priv->topaz_mtx_wb +
					 cur_cmd_header->core *
					 MTX_WRITEBACK_DATASIZE_ROUND + 1);
#endif
			break;
		default:
			DRM_ERROR("TOPAZ: unsupported command id: %x\n",
				  cur_cmd_id);
			goto out;
		}

		/*cur_cmd_size indicate the number of words of cur command*/
		command += cur_cmd_size * 4;
		cmd_size -= cur_cmd_size * 4;
	}
#if PNW_TOPAZ_NO_IRQ
	PSB_DEBUG_GENERAL("reset NULL writeback to 0xffffffff,"
			  "topaz_priv->topaz_sync_addr=0x%p\n",
			  topaz_priv->topaz_sync_addr);

	*((uint32_t *) topaz_priv->topaz_sync_addr + MTX_WRITEBACK_VALUE) = ~0;
	pnw_topaz_kick_null_cmd(dev_priv, 0,
				topaz_priv->topaz_sync_offset, sync_seq, 0);

	if (0 != pnw_wait_on_sync(dev_priv, sync_seq,
				  topaz_priv->topaz_sync_addr +
				  MTX_WRITEBACK_VALUE)) {
		DRM_ERROR("TOPAZSC: Polling the writeback of last command"
			  " failed!\n");
		topaz_read_core_reg(dev_priv, 0, 0x5, &reg_val);
		DRM_ERROR("TOPAZSC: PC pointer of core 0 is %x\n", reg_val);
		topaz_read_core_reg(dev_priv, 1, 0x5, &reg_val);
		DRM_ERROR("TOPAZSC: PC pointer of core 1 is %x\n", reg_val);
		TOPAZ_MULTICORE_READ32(TOPAZSC_CR_MULTICORE_CMD_FIFO_1,
				       &reg_val);
		reg_val &= MASK_TOPAZSC_CR_CMD_FIFO_SPACE;
		DRM_ERROR("TOPAZSC: Free words in command FIFO %d\n", reg_val);
		DRM_ERROR("TOPAZSC: Last writeback of core 0 %d\n",
			  *(topaz_priv->topaz_mtx_wb + 1));
		DRM_ERROR("TOPAZSC: Last writeback of core 1 %d\n",
			  *(topaz_priv->topaz_mtx_wb +
			    MTX_WRITEBACK_DATASIZE_ROUND + 1));
	}

	PSB_DEBUG_GENERAL("Kicked command with sequence 0x%08x,"
			  " and polling it, got 0x%08x\n",
			  sync_seq,
			  *(topaz_priv->topaz_sync_addr + MTX_WRITEBACK_VALUE));
	PSB_DEBUG_GENERAL("Can handle unfence here, but let fence"
			  " polling do it\n");
	topaz_priv->topaz_busy = 0;
#else
	PSB_DEBUG_GENERAL("Kick command with sequence %x\n", sync_seq);
	/* This may be reset in topaz_setup_fw */
	topaz_priv->topaz_busy = 1;
	pnw_topaz_kick_null_cmd(dev_priv, 0,
			topaz_priv->topaz_sync_offset, sync_seq, 1);
#endif
 out:
	return ret;
}

int pnw_topaz_dequeue_send(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct pnw_topaz_cmd_queue *topaz_cmd = NULL;
	int ret;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: dequeue command and send it to topaz\n");

	if (list_empty(&topaz_priv->topaz_queue)) {
		topaz_priv->topaz_busy = 0;
		return 0;
	}

	topaz_priv->topaz_busy = 1;
	topaz_cmd = list_first_entry(&topaz_priv->topaz_queue,
				     struct pnw_topaz_cmd_queue, head);

	PSB_DEBUG_GENERAL("TOPAZ: queue has id %08x\n", topaz_cmd->sequence);
	ret = pnw_topaz_send(dev, topaz_cmd->cmd, topaz_cmd->cmd_size,
			     topaz_cmd->sequence);
	if (ret) {
		DRM_ERROR("TOPAZ: pnw_topaz_send failed.\n");
		ret = -EINVAL;
	}

	list_del(&topaz_cmd->head);
	kfree(topaz_cmd->cmd);
	kfree(topaz_cmd);

	return ret;
}

void topaz_mtx_kick(struct drm_psb_private *dev_priv, uint32_t core_id,
		    uint32_t kick_count)
{
	PSB_DEBUG_GENERAL("TOPAZ: kick core(%d) mtx count(%d).\n",
			  core_id, kick_count);
	topaz_set_mtx_target(dev_priv, core_id, 0);
	MTX_WRITE32(MTX_CR_MTX_KICK, kick_count, core_id);
	return;
}

int pnw_check_topaz_idle(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t reg_val;

	if (dev_priv->topaz_ctx == NULL)
		return 0;

	if (topaz_priv->topaz_busy)
		return -EBUSY;

	if (topaz_priv->topaz_hw_busy) {
		PSB_DEBUG_PM("TOPAZ: %s, HW is busy\n", __func__);
		return -EBUSY;
	}
	TOPAZ_MULTICORE_READ32(TOPAZSC_CR_MULTICORE_CMD_FIFO_1, &reg_val);
	reg_val &= MASK_TOPAZSC_CR_CMD_FIFO_SPACE;
	if (reg_val != 32) {
		PSB_DEBUG_GENERAL("TOPAZ: HW is busy. Free words in command"
				  "FIFO is %d.\n", reg_val);
		return -EBUSY;
	}

	return 0;		/* we think it is idle */
}

int pnw_video_get_core_num(struct drm_device *dev, uint64_t user_pointer)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
	int ret;

	ret = copy_to_user((void __user *)((unsigned long)user_pointer),
			   &topaz_priv->topaz_num_cores,
			   sizeof(topaz_priv->topaz_num_cores));

	if (ret)
		return -EFAULT;

	return 0;

}

int pnw_video_frameskip(struct drm_device *dev, uint64_t user_pointer)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
	int ret;

	ret = copy_to_user((void __user *)((unsigned long)user_pointer),
			   &topaz_priv->frame_skip,
			   sizeof(topaz_priv->frame_skip));

	if (ret)
		return -EFAULT;

	return 0;
}

static void pnw_topaz_flush_cmd_queue(struct pnw_topaz_private *topaz_priv)
{
	struct pnw_topaz_cmd_queue *entry, *next;

	/* remind to reset topaz */
	topaz_priv->topaz_needs_reset = 1;

	if (list_empty(&topaz_priv->topaz_queue)) {
		topaz_priv->topaz_busy = 0;
		return;
	}

	/* flush all command in queue */
	list_for_each_entry_safe(entry, next, &topaz_priv->topaz_queue, head) {
		list_del(&entry->head);
		kfree(entry->cmd);
		kfree(entry);
	}

	return;
}

void pnw_topaz_handle_timeout(struct ttm_fence_device *fdev)
{
	struct drm_psb_private *dev_priv =
	    container_of(fdev, struct drm_psb_private, fdev);
	struct drm_device *dev =
	    container_of((void *)dev_priv, struct drm_device, dev_private);
	struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;

	pnw_topaz_flush_cmd_queue(topaz_priv);
}

void pnw_map_topaz_reg(struct drm_device *dev)
{
	unsigned long resource_start;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	resource_start = pci_resource_start(dev->pdev, PSB_MMIO_RESOURCE);

	if (IS_MDFLD(dev) && !dev_priv->topaz_disabled) {
		dev_priv->topaz_reg = ioremap(resource_start + PNW_TOPAZ_OFFSET,
					      PNW_TOPAZ_SIZE);
		if (!dev_priv->topaz_reg)
			DRM_ERROR("failed to map TOPAZ register address\n");
	}

	return;
}

void pnw_unmap_topaz_reg(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (IS_MDFLD(dev)) {
		if (dev_priv->topaz_reg) {
			iounmap(dev_priv->topaz_reg);
			dev_priv->topaz_reg = NULL;
		}
	}

	return;
}

void pnw_topaz_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/* uint32_t ier = dev_priv->vdc_irq_mask | _PNW_IRQ_TOPAZ_FLAG; */

	PSB_DEBUG_IRQ("TOPAZ: enable IRQ\n");

	/* Only enable the master core IRQ */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTENAB,
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_MAS_INTEN) |
		      /* F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MVEA) | */
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MMU_FAULT) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MTX) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MTX_HALT), 0);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}

void pnw_topaz_disableirq(struct drm_device *dev)
{

	struct drm_psb_private *dev_priv = dev->dev_private;
	/*uint32_t ier = dev_priv->vdc_irq_mask & (~_PNW_IRQ_TOPAZ_FLAG); */

	PSB_DEBUG_INIT("TOPAZ: disable IRQ\n");

	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTENAB, 0, 0);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}
