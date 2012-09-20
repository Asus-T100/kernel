/**
 * file lnc_topaz.c
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
#include "lnc_topaz.h"
#include "psb_powermgmt.h"
#include "lnc_topaz_hw_reg.h"

#include <linux/io.h>
#include <linux/delay.h>

#define TOPAZ_RM_MULTI_MTX_WRITE

/* static function define */
static int lnc_topaz_deliver_command(struct drm_device *dev,
				     struct ttm_buffer_object *cmd_buffer,
				     unsigned long cmd_offset,
				     unsigned long cmd_size,
				     void **topaz_cmd, uint32_t sequence,
				     int copy_cmd);
static int lnc_topaz_send(struct drm_device *dev, void *cmd,
			  unsigned long cmd_size, uint32_t sync_seq);
static int lnc_mtx_send(struct drm_psb_private *dev_priv, const void *cmd);
static int lnc_topaz_dequeue_send(struct drm_device *dev);
static int lnc_topaz_save_command(struct drm_device *dev, void *cmd,
				  unsigned long cmd_size, uint32_t sequence);

bool lnc_topaz_interrupt(void *pvData)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	uint32_t clr_flag;
	struct topaz_private *topaz_priv;
	uint32_t topaz_stat;
	uint32_t cur_seq;

	if (pvData == NULL) {
		DRM_ERROR("ERROR: TOPAZ %s, Invalid params\n", __func__);
		return false;
	}

	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
		DRM_ERROR("ERROR: interrupt arrived but HW is power off\n");
		return false;
	}

	dev = (struct drm_device *)pvData;
	dev_priv = (struct drm_psb_private *)dev->dev_private;
	topaz_priv = dev_priv->topaz_private;

	topaz_priv->topaz_hw_busy = REG_READ(0x20D0) & (0x1 << 11);

	TOPAZ_READ32(TOPAZ_CR_IMG_TOPAZ_INTSTAT, &topaz_stat);
	clr_flag = lnc_topaz_queryirq(dev);

	lnc_topaz_clearirq(dev, clr_flag);

	/* ignore non-SYNC interrupts */
	if ((CCB_CTRL_SEQ(dev_priv) & 0x8000) == 0)
		return true;

	cur_seq = *(uint32_t *) topaz_priv->topaz_sync_addr;

	PSB_DEBUG_IRQ("TOPAZ:Got SYNC IRQ,sync seq:0x%08x (MTX) vs 0x%08x\n",
		      cur_seq, dev_priv->sequence[LNC_ENGINE_ENCODE]);

	psb_fence_handler(dev, LNC_ENGINE_ENCODE);

	/* save frame skip flag for query */
	topaz_priv->frame_skip = CCB_CTRL_FRAMESKIP(dev_priv);

	topaz_priv->topaz_busy = 1;
	lnc_topaz_dequeue_send(dev);

	if (drm_topaz_pmpolicy != PSB_PMPOLICY_NOPM)
		schedule_delayed_work(&dev_priv->scheduler.topaz_suspend_wq, 0);

	return true;
}

static int lnc_submit_encode_cmdbuf(struct drm_device *dev,
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
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: command submit\n");

	PSB_DEBUG_GENERAL("TOPAZ: topaz busy = %d\n", topaz_priv->topaz_busy);

	if (dev_priv->last_topaz_ctx != dev_priv->topaz_ctx) {
		/* todo: save current context into topaz_priv->current_ctx
		 * and reload dev_priv->topaz_ctx context
		 */
		PSB_DEBUG_GENERAL("TOPAZ: context switch\n");
		dev_priv->last_topaz_ctx = dev_priv->topaz_ctx;
	}

	if (topaz_priv->topaz_fw_loaded == 0) {
		/* #.# load fw to driver */
		PSB_DEBUG_INIT("TOPAZ: load /lib/firmware/topaz_fw.bin\n");
		ret = topaz_init_fw(dev);
		if (ret != 0) {
			/* FIXME: find a proper return value */
			DRM_ERROR("TOPAX:load /lib/firmware/topaz_fw.bin fail,"
				  "ensure udevd is configured correctly!\n");

			return -EFAULT;
		}
		topaz_priv->topaz_fw_loaded = 1;
	}

	tmp = atomic_cmpxchg(&dev_priv->topaz_mmu_invaldc, 1, 0);
	if (tmp == 1)
		topaz_mmu_flushcache(dev_priv);

	/* # schedule watchdog */
	/* psb_schedule_watchdog(dev_priv); */

	/* # spin lock irq save [msvdx_lock] */
	spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);

	/* # if topaz need to reset, reset it */
	if (topaz_priv->topaz_needs_reset) {
		/* #.# reset it */
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);
		PSB_DEBUG_GENERAL("TOPAZ: needs reset.\n");

		if (lnc_topaz_reset(dev_priv)) {
			ret = -EBUSY;
			DRM_ERROR("TOPAZ: reset failed.\n");
			return ret;
		}

		PSB_DEBUG_GENERAL("TOPAZ: reset ok.\n");

		/* #.# upload firmware */
		if (topaz_setup_fw(dev, topaz_priv->topaz_cur_codec)) {
			DRM_ERROR("TOPAZ: upload FW to HW failed\n");
			return -EBUSY;
		}

		spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);
	}

	if (!topaz_priv->topaz_busy) {
		/* direct map topaz command if topaz is free */
		PSB_DEBUG_GENERAL("TOPAZ:direct send command,sequence %08x\n",
				sequence);

		topaz_priv->topaz_busy = 1;
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

		ret = lnc_topaz_deliver_command(dev, cmd_buffer, cmd_offset,
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

		ret = lnc_topaz_deliver_command(dev, cmd_buffer, cmd_offset,
						cmd_size, &cmd, sequence, 1);
		if (cmd == NULL || ret) {
			DRM_ERROR("TOPAZ: map command for save fialed\n");
			return ret;
		}

		ret = lnc_topaz_save_command(dev, cmd, cmd_size, sequence);
		if (ret)
			DRM_ERROR("TOPAZ: save command failed\n");
	}

	return ret;
}

static int lnc_topaz_save_command(struct drm_device *dev, void *cmd,
				  unsigned long cmd_size, uint32_t sequence)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct lnc_topaz_cmd_queue *topaz_cmd;
	unsigned long irq_flags;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: queue command,sequence: %08x..\n", sequence);

	topaz_cmd = kzalloc(sizeof(struct lnc_topaz_cmd_queue), GFP_KERNEL);
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
		lnc_topaz_dequeue_send(dev);
		PSB_DEBUG_GENERAL("TOPAZ: after dequeue command\n");
	}

	spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

	return 0;
}

int lnc_cmdbuf_video(struct drm_file *priv,
		     struct list_head *validate_list,
		     uint32_t fence_type,
		     struct drm_psb_cmdbuf_arg *arg,
		     struct ttm_buffer_object *cmd_buffer,
		     struct psb_ttm_fence_rep *fence_arg)
{
	struct drm_device *dev = priv->minor->dev;
	struct ttm_fence_object *fence = NULL;
	int ret;

	ret = lnc_submit_encode_cmdbuf(dev, cmd_buffer, arg->cmdbuf_offset,
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

	return 0;
}

static int lnc_topaz_sync(struct drm_device *dev, uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t sync_cmd[3];
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

#if 0
	struct ttm_fence_device *fdev = &dev_priv->fdev;
	struct ttm_fence_class_manager *fc =
	    &fdev->fence_class[LNC_ENGINE_ENCODE];
	unsigned long irq_flags;
#endif
#if LNC_TOPAZ_NO_IRQ
	uint32_t *sync_p = (uint32_t *) topaz_priv->topaz_sync_addr;
	int count = 10000;
	uint32_t cur_seq;
#endif

	/* insert a SYNC command here */
	topaz_priv->topaz_sync_cmd_seq = (1 << 15) |
	    topaz_priv->topaz_cmd_seq++;
	sync_cmd[0] = 1 | (MTX_CMDID_SYNC << 1) | (3 << 8) |
	    (topaz_priv->topaz_sync_cmd_seq << 16);
	sync_cmd[1] = topaz_priv->topaz_sync_offset;
	sync_cmd[2] = sync_seq;

	PSB_DEBUG_GENERAL("TOPAZ:MTX_CMDID_SYNC: size(3),cmd seq (0x%04x),"
			  "sync_seq (0x%08x)\n",
			  topaz_priv->topaz_sync_cmd_seq, sync_seq);

	if (drm_topaz_sbuswa)
		TOPAZ_WAIT_UNTIL_IDLE;

	lnc_mtx_send(dev_priv, sync_cmd);

#if LNC_TOPAZ_NO_IRQ		/* workaround for interrupt issue */
	/* # poll topaz register for certain times */
	while (count && *sync_p != sync_seq) {
		DRM_UDELAY(100);
		--count;
	}
	if ((count == 0) && (*sync_p != sync_seq)) {
		DRM_ERROR("TOPAZ: wait sycn timeout (0x%08x),actual 0x%08x\n",
			  sync_seq, *sync_p);
		return -EBUSY;
	}
	PSB_DEBUG_GENERAL("TOPAZ: SYNC done, seq=0x%08x\n", *sync_p);

	topaz_priv->topaz_busy = 0;

	/* XXX: check psb_fence_handler is suitable for topaz */
	cur_seq = *sync_p;
#if 0
	write_lock_irqsave(&fc->lock, irq_flags);
	ttm_fence_handler(fdev, LNC_ENGINE_ENCODE,
			  cur_seq, _PSB_FENCE_TYPE_EXE, 0);
	write_unlock_irqrestore(&fc->lock, irq_flags);
#endif
#endif
	return 0;
}

int
lnc_topaz_deliver_command(struct drm_device *dev,
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
		ret = lnc_topaz_send(dev, cmd_start, cmd_size, sequence);
		if (ret) {
			DRM_ERROR("TOPAZ: commit commands failed.\n");
			ret = -EINVAL;
		}
	}

 out:
	PSB_DEBUG_GENERAL("TOPAZ:cmd_size(%ld), sequence(%d) copy_cmd(%d)\n",
			  cmd_size, sequence, copy_cmd);

	ttm_bo_kunmap(&cmd_kmap);

	return ret;
}

int
lnc_topaz_send(struct drm_device *dev, void *cmd,
	       unsigned long cmd_size, uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret = 0;
	unsigned char *command = (unsigned char *)cmd;
	struct topaz_cmd_header *cur_cmd_header;
	uint32_t cur_cmd_size, cur_cmd_id;
	uint32_t codec;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: send the command in the buffer one by one\n");

	while (cmd_size > 0) {
		cur_cmd_header = (struct topaz_cmd_header *)command;
		cur_cmd_size = cur_cmd_header->size * 4;
		cur_cmd_id = cur_cmd_header->id;

		switch (cur_cmd_id) {
		case MTX_CMDID_SW_NEW_CODEC:
			codec = *((uint32_t *) cmd + 1);

			PSB_DEBUG_GENERAL("TOPAZ: setup new codec %s (%d)\n",
					  codec_to_string(codec), codec);
			if (topaz_setup_fw(dev, codec)) {
				DRM_ERROR("TOPAZ: upload FW to HW failed\n");
				return -EBUSY;
			}

			topaz_priv->topaz_cur_codec = codec;
			break;

		case MTX_CMDID_SW_ENTER_LOWPOWER:
			PSB_DEBUG_GENERAL("TOPAZ: enter lowpower....\n");
			PSB_DEBUG_GENERAL("XXX: implement it\n");
			break;

		case MTX_CMDID_SW_LEAVE_LOWPOWER:
			PSB_DEBUG_GENERAL("TOPAZ: leave lowpower...\n");
			PSB_DEBUG_GENERAL("XXX: implement it\n");
			break;

			/* ordinary commmand */
		case MTX_CMDID_START_PIC:
			/* XXX: specially handle START_PIC hw command */
			CCB_CTRL_SET_QP(dev_priv,
					*(command + cur_cmd_size - 4));
			/* strip the QP parameter (it's software arg) */
			cur_cmd_header->size--;
		default:
			cur_cmd_header->seq = 0x7fff &
			    topaz_priv->topaz_cmd_seq++;

			PSB_DEBUG_GENERAL("TOPAZ: %s: size(%d),"
					  " seq (0x%04x)\n",
					  cmd_to_string(cur_cmd_id),
					  cur_cmd_size, cur_cmd_header->seq);

			if (drm_topaz_sbuswa && cur_cmd_id !=
			    MTX_CMDID_START_PIC)
				TOPAZ_WAIT_UNTIL_IDLE;

			ret = lnc_mtx_send(dev_priv, command);
			if (ret) {
				DRM_ERROR("TOPAZ: error -- ret(%d)\n", ret);
				goto out;
			}
			break;
		}

		command += cur_cmd_size;
		cmd_size -= cur_cmd_size;
	}
	lnc_topaz_sync(dev, sync_seq);
 out:
	return ret;
}

static int lnc_mtx_send(struct drm_psb_private *dev_priv, const void *cmd)
{
	struct topaz_cmd_header *cur_cmd_header =
	    (struct topaz_cmd_header *)cmd;
	uint32_t cmd_size = cur_cmd_header->size;
	uint32_t read_index, write_index;
	const uint32_t *cmd_pointer = (uint32_t *) cmd;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	int ret = 0;

	/* <msvdx does> # enable all clock */

	write_index = topaz_priv->topaz_cmd_windex;
	if (write_index + cmd_size + 1 > topaz_priv->topaz_ccb_size) {
		int free_space = topaz_priv->topaz_ccb_size - write_index;

		PSB_DEBUG_GENERAL("TOPAZ: -------will wrap CCB write point.\n");
		if (free_space > 0) {
			struct topaz_cmd_header pad_cmd;

			pad_cmd.id = MTX_CMDID_NULL;
			pad_cmd.size = free_space;
			pad_cmd.seq = 0x7fff & topaz_priv->topaz_cmd_seq;

			PSB_DEBUG_GENERAL("TOPAZ: MTX_CMDID_NULL:"
					  " size(%d),seq (0x%04x)\n",
					  pad_cmd.size, pad_cmd.seq);

#ifndef TOPAZ_RM_MULTI_MTX_WRITE
			TOPAZ_BEGIN_CCB(dev_priv);
			TOPAZ_OUT_CCB(dev_priv, pad_cmd.val);
#else
			topaz_write_mtx_mem(dev_priv,
					    topaz_priv->topaz_ccb_buffer_addr
					    + topaz_priv->topaz_cmd_windex * 4,
					    pad_cmd.val);
			topaz_priv->topaz_cmd_windex++;
#endif
			TOPAZ_END_CCB(dev_priv, 1);

			POLL_WB_SEQ(dev_priv, pad_cmd.seq);
			++topaz_priv->topaz_cmd_seq;
		}
		POLL_WB_RINDEX(dev_priv, 0);
		if (ret == 0)
			topaz_priv->topaz_cmd_windex = 0;
		else {
			DRM_ERROR("TOPAZ: poll rindex timeout\n");
			return ret;	/* HW may hang, need reset */
		}
		PSB_DEBUG_GENERAL("TOPAZ: -------wrap CCB was done.\n");
	}

	read_index = CCB_CTRL_RINDEX(dev_priv);	/* temperily use CCB CTRL */
	write_index = topaz_priv->topaz_cmd_windex;

	PSB_DEBUG_GENERAL("TOPAZ: write index(%d), read index(%d,WB=%d)\n",
			  write_index, read_index,
			  WB_CCB_CTRL_RINDEX(dev_priv));

#ifndef TOPAZ_RM_MULTI_MTX_WRITE
	TOPAZ_BEGIN_CCB(dev_priv);
	while (cmd_size > 0) {
		TOPAZ_OUT_CCB(dev_priv, *cmd_pointer++);
		--cmd_size;
	}
#else
	while (cmd_size > 0) {
		topaz_write_mtx_mem(dev_priv,
				    topaz_priv->topaz_ccb_buffer_addr
				    + topaz_priv->topaz_cmd_windex * 4,
				    *cmd_pointer++);
		topaz_priv->topaz_cmd_windex++;
		--cmd_size;
	}
#endif
	TOPAZ_END_CCB(dev_priv, 1);

#if 0
	DRM_UDELAY(1000);
	lnc_topaz_clearirq(dev, lnc_topaz_queryirq(dev));
	LNC_TRACEL("TOPAZ: after clear, query again\n");
	lnc_topaz_queryirq(dev_priv);
#endif

	return ret;
}

int lnc_topaz_dequeue_send(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct lnc_topaz_cmd_queue *topaz_cmd = NULL;
	int ret;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: dequeue command and send it to topaz\n");

	if (list_empty(&topaz_priv->topaz_queue)) {
		topaz_priv->topaz_busy = 0;
		return 0;
	}

	topaz_cmd = list_first_entry(&topaz_priv->topaz_queue,
				     struct lnc_topaz_cmd_queue, head);

	PSB_DEBUG_GENERAL("TOPAZ: queue has id %08x\n", topaz_cmd->sequence);
	ret = lnc_topaz_send(dev, topaz_cmd->cmd, topaz_cmd->cmd_size,
			     topaz_cmd->sequence);
	if (ret) {
		DRM_ERROR("TOPAZ: lnc_topaz_send failed.\n");
		ret = -EINVAL;
	}

	list_del(&topaz_cmd->head);
	kfree(topaz_cmd->cmd);
	kfree(topaz_cmd);

	return ret;
}

void topaz_mtx_kick(struct drm_psb_private *dev_priv, uint32_t kick_count)
{
	PSB_DEBUG_GENERAL("TOPAZ: kick mtx count(%d).\n", kick_count);
	MTX_WRITE32(MTX_CR_MTX_KICK, kick_count);
}

int lnc_check_topaz_idle(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	if (topaz_priv->topaz_fw_loaded == 0)
		return 0;

	if (topaz_priv->topaz_busy)
		return -EBUSY;

	if (topaz_priv->topaz_hw_busy) {
		PSB_DEBUG_PM("TOPAZ: %s, HW is busy\n", __func__);
		return -EBUSY;
	}

	return 0;		/* we think it is idle */
}

int lnc_video_frameskip(struct drm_device *dev, uint64_t user_pointer)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	int ret;

	ret = copy_to_user((void __user *)((unsigned long)user_pointer),
			   &topaz_priv->frame_skip,
			   sizeof(topaz_priv->frame_skip));

	if (ret)
		return -EFAULT;

	return 0;
}

static void lnc_topaz_flush_cmd_queue(struct topaz_private *topaz_priv)
{
	struct lnc_topaz_cmd_queue *entry, *next;

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

void lnc_topaz_handle_timeout(struct ttm_fence_device *fdev)
{
	struct drm_psb_private *dev_priv =
	    container_of(fdev, struct drm_psb_private, fdev);
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	lnc_topaz_flush_cmd_queue(topaz_priv);
}

inline int psb_try_power_down_topaz(struct drm_device *dev)
{
	ospm_apm_power_down_topaz(dev);
	return 0;
}

void lnc_map_topaz_reg(struct drm_device *dev)
{
	unsigned long resource_start;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	resource_start = pci_resource_start(dev->pdev, PSB_MMIO_RESOURCE);

	if (IS_TOPAZ(dev)) {
		dev_priv->topaz_reg =
		    ioremap(resource_start + LNC_TOPAZ_OFFSET, LNC_TOPAZ_SIZE);
		if (!dev_priv->topaz_reg)
			DRM_ERROR("failed to map TOPAZ register address\n");
	}

	return;
}

void lnc_unmap_topaz_reg(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (IS_TOPAZ(dev)) {
		if (dev_priv->topaz_reg) {
			iounmap(dev_priv->topaz_reg);
			dev_priv->topaz_reg = NULL;
		}
	}

	return;
}

void lnc_topaz_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	/* uint32_t ier = dev_priv->vdc_irq_mask | _LNC_IRQ_TOPAZ_FLAG; */

	PSB_DEBUG_IRQ("TOPAZ: enable IRQ\n");

	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTENAB,
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_MAS_INTEN) |
		      /* F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MVEA) | */
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MMU_FAULT) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MTX) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTEN_MTX_HALT));

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}

void lnc_topaz_disableirq(struct drm_device *dev)
{

	struct drm_psb_private *dev_priv = dev->dev_private;
	/* uint32_t ier = dev_priv->vdc_irq_mask & (~_LNC_IRQ_TOPAZ_FLAG); */

	PSB_DEBUG_INIT("TOPAZ: disable IRQ\n");

	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTENAB, 0);

	/* write in sysirq.c */
	/* PSB_WVDC32(ier, PSB_INT_ENABLE_R); /\* essential *\/ */
}
