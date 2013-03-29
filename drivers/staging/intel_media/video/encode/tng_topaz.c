/**
 * file tng_topaz.c
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
#include <drm/drm.h>

#include "psb_drv.h"
#include "tng_topaz.h"
#include "psb_powermgmt.h"
#include "tng_topaz_hw_reg.h"
/*#include "private_data.h"*/

#include <linux/io.h>
#include <linux/delay.h>

#define TOPAZ_MAX_COMMAND_IN_QUEUE 0x1000
#define MASK_MTX_INT_ENAB 0x4000ff00

#define LOOP_COUNT 10000
/*static uint32_t setv_cnt = 0;*/

enum MTX_MESSAGE_ID {
	MTX_MESSAGE_ACK,   /* !< (no data)\n Null command does nothing\n */
	MTX_MESSAGE_CODED, /* !< (no data)\n Null command does nothing\n */
} ;

/* static function define */
static int tng_topaz_deliver_command(
	struct drm_device *dev,
	struct drm_file *file_priv,
	struct ttm_buffer_object *cmd_buffer,
	uint32_t cmd_offset,
	uint32_t cmd_size,
	void **topaz_cmd, uint32_t sequence,
	int copy_cmd);

static int tng_topaz_send(
	struct drm_device *dev,
	struct drm_file *file_priv,
	void *cmd,
	uint32_t cmd_size,
	uint32_t sync_seq);

static int tng_topaz_save_command(
	struct drm_device *dev,
	struct drm_file *file_priv,
	void *cmd,
	uint32_t cmd_size,
	uint32_t sequence);

void mtx_start(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	mtx_set_target(dev_priv);
	MTX_WRITE32(MTX_CR_MTX_ENABLE, MASK_MTX_MTX_ENABLE);
}

void mtx_stop(struct drm_psb_private *dev_priv)
{
	mtx_set_target(dev_priv);
	MTX_WRITE32(MTX_CR_MTX_ENABLE, MASK_MTX_MTX_TOFF);
}

void mtx_kick(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	mtx_set_target(dev_priv);
	PSB_DEBUG_GENERAL("TOPAZ: Kick MTX to start\n");
	MTX_WRITE32(MTX_CR_MTX_KICK, 1);
}

void tng_set_consumer(struct drm_device *dev, uint32_t consumer)
{
	unsigned int reg_val;
	struct drm_psb_private *dev_priv = dev->dev_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOMTX << 2), &reg_val);

	reg_val = F_INSERT(reg_val, consumer, WB_CONSUMER);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOMTX << 2), reg_val);
}

uint32_t tng_get_consumer(struct drm_device *dev)
{
	unsigned int reg_val;
	struct drm_psb_private *dev_priv = dev->dev_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOMTX << 2), &reg_val);

	return F_DECODE(reg_val, WB_CONSUMER);
}

void tng_set_producer(struct drm_device *dev, uint32_t producer)
{
	unsigned int reg_val;
	struct drm_psb_private *dev_priv = dev->dev_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOHOST << 2), &reg_val);

	reg_val = F_INSERT(reg_val, producer, WB_PRODUCER);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOHOST << 2), reg_val);
}

uint32_t tng_get_producer(struct drm_device *dev)
{
	unsigned int reg_val;
	struct drm_psb_private *dev_priv = dev->dev_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		(MTX_SCRATCHREG_TOHOST << 2), &reg_val);

	return F_DECODE(reg_val, WB_PRODUCER);
}

uint32_t tng_wait_for_ctrl(struct drm_device *dev, uint32_t control)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int32_t ret = 0;
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		TOPAZHP_TOP_CR_FIRMWARE_REG_1 + (MTX_SCRATCHREG_TOHOST << 2),
		control, 0x80000000);
	if (ret)
		DRM_ERROR("Wait for register timeout");

	return ret;
}

uint32_t tng_serialize_enter(struct drm_device *dev)
{
	uint32_t reg_val;
	int32_t ret;
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*
	* Poll for idle register to tell that both HW
	* and FW are idle (`FW_IDLE_STATUS_IDLE` state)
	*/
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		MTX_SCRATCHREG_IDLE,
		F_ENCODE(FW_IDLE_STATUS_IDLE, FW_IDLE_REG_STATUS),
		MASK_FW_IDLE_REG_STATUS);
	if (ret)
		DRM_ERROR("Wait for register timeout");

	MULTICORE_READ32(MTX_SCRATCHREG_IDLE, &reg_val);

	return F_EXTRACT(reg_val, FW_IDLE_REG_RECEIVED_COMMANDS);
}

void tng_serialize_exit(struct drm_device *dev, uint32_t enter_token)
{
	int32_t ret;
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*
	* Poll for idle register to tell that both HW
	* and FW are idle (`FW_IDLE_STATUS_IDLE` state)
	*/
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
		MTX_SCRATCHREG_IDLE,
		F_ENCODE(enter_token, FW_IDLE_REG_RECEIVED_COMMANDS),
		MASK_FW_IDLE_REG_RECEIVED_COMMANDS);
	if (ret)
		DRM_ERROR("Wait for register timeout");
}

static void tng_topaz_Int_clear(
	struct drm_psb_private *dev_priv,
	uint32_t intClearMask)
{
	unsigned long irq_flags;
	struct tng_topaz_private *topaz_priv;

	topaz_priv = dev_priv->topaz_private;
	spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);
	/* clear interrupt */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_INT_CLEAR,
		intClearMask);
	spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);
}

static struct psb_video_ctx *get_ctx_from_fp(
	struct drm_device *dev, struct file *filp)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_video_ctx *pos;

	list_for_each_entry(pos, &dev_priv->video_ctx, head)
		if (pos->filp == filp)
			return pos;

	return NULL;
}

uint32_t get_ctx_cnt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_video_ctx *pos;
	int count = 0;

	list_for_each_entry(pos, &dev_priv->video_ctx, head)
		count++;

	return count;
}

int32_t dispatch_wb_message_polling(
	struct drm_device *dev,
	struct drm_file *file_priv,
	int32_t sync_seq,
	unsigned char *command)
{
	struct psb_video_ctx *video_ctx;
	struct tng_topaz_private *topaz_priv;
	struct drm_psb_private *dev_priv;
	struct tng_topaz_cmd_header *cur_cmd_header;
	struct IMG_WRITEBACK_MSG *wb_msg;
	int32_t ret;

	dev_priv = (struct drm_psb_private *) dev->dev_private;
	if (!dev_priv)
		DRM_ERROR("Failed to get dev_priv\n");

	topaz_priv = dev_priv->topaz_private;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);

	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		return -1;
	}

	topaz_priv->consumer = tng_get_consumer(dev);
	topaz_priv->producer = tng_get_producer(dev);

	/* Read and compare consumer and producer */
	if (topaz_priv->producer == topaz_priv->consumer) {
		PSB_DEBUG_GENERAL("TOPAZ: producer = consumer = %d",
			topaz_priv->producer);
		PSB_DEBUG_GENERAL("polling producer until change\n");
		/* if the same -> poll on Producer change */
		ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
			TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				(MTX_SCRATCHREG_TOHOST << 2),
			topaz_priv->consumer, MASK_WB_PRODUCER);
		if (ret) {
			wb_msg = (struct IMG_WRITEBACK_MSG *)
				video_ctx->wb_addr[topaz_priv->consumer];
			DRM_ERROR("Polling timeout, ui32CmdWord = %08x, " \
				"ui32Data = %08x, ui32ExtraData = %08x, " \
				"ui32WritebackVal = %08x, " \
				"ui32CodedBufferConsumed = %08x\n",
				wb_msg->ui32CmdWord, wb_msg->ui32Data,
				wb_msg->ui32ExtraData, wb_msg->ui32WritebackVal,
				wb_msg->ui32CodedBufferConsumed);

			return ret;
		}

		topaz_priv->producer = tng_get_producer(dev);
	}

	/* Dispatch new messages */
	do {
		PSB_DEBUG_GENERAL("TOPAZ: Dispatch write back message, " \
			"producer = %d, consumer = %d\n",
			topaz_priv->producer, topaz_priv->consumer);

		ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
			TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				(MTX_SCRATCHREG_TOHOST << 2),
			topaz_priv->consumer, MASK_WB_PRODUCER);
		if (ret) {
			DRM_ERROR("Wait for register timeout");
			return ret;
		}

		topaz_priv->consumer++;
		if (topaz_priv->consumer == WB_FIFO_SIZE)
			topaz_priv->consumer = 0;

		tng_set_consumer(dev, topaz_priv->consumer);

	} while (topaz_priv->consumer != topaz_priv->producer);

	cur_cmd_header = (struct tng_topaz_cmd_header *)command;

	if (cur_cmd_header->id != MTX_CMDID_ENCODE_FRAME &&
	    cur_cmd_header->id != MTX_CMDID_ISSUEBUFF)
		return 0;

	PSB_DEBUG_GENERAL("TOPAZ: Handle context saving/fence " \
		"handler/dequeue send on ENCODE_FRAME command\n");

	if (get_ctx_cnt(dev) > 1) {
		PSB_DEBUG_GENERAL("TOPAZ: More than one context, " \
			"save current context\n");
		if (topaz_priv->cur_context->codec != IMG_CODEC_JPEG) {
			ret = tng_topaz_save_mtx_state(dev);
			if (ret) {
				DRM_ERROR("Failed to save mtx status");
				return ret;
			}
		}
	}

	*topaz_priv->topaz_sync_addr = sync_seq;

	psb_fence_handler(dev, LNC_ENGINE_ENCODE);

	topaz_priv->topaz_busy = 1;
	tng_topaz_dequeue_send(dev);

	return ret;
}

int32_t dispatch_wb_message_irq(struct drm_device *dev)
{
	struct tng_topaz_private *topaz_priv;
	struct drm_psb_private *dev_priv;
	/* uint32_t crMultiCoreIntStat;*/
	/* struct psb_video_ctx *video_ctx; */
	/* struct IMG_WRITEBACK_MSG *wb_msg; */
	int32_t ret;
	int32_t count = 0;

	dev_priv = (struct drm_psb_private *) dev->dev_private;
	if (!dev_priv)
		DRM_ERROR("Failed to get dev_priv\n");

	topaz_priv = dev_priv->topaz_private;

	topaz_priv->consumer = tng_get_consumer(dev);

	do {
		topaz_priv->producer = tng_get_producer(dev);
		count++;
	} while (topaz_priv->producer == topaz_priv->consumer &&
		 count < 300000);

	if (count == 300000) {
		DRM_ERROR("Waiting for IRQ timeout\n");
		return -1;
	}

	PSB_DEBUG_GENERAL("TOPAZ: Producer = %d, Consumer = %d\n",
		topaz_priv->producer, topaz_priv->consumer);

	do {
		ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
			TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				(MTX_SCRATCHREG_TOHOST << 2),
			topaz_priv->consumer, MASK_WB_PRODUCER);
		if (ret)
			return ret;

		topaz_priv->consumer++;
		if (topaz_priv->consumer == WB_FIFO_SIZE)
			topaz_priv->consumer = 0;

		/* Indicate buffer used by consmer is available */
		tng_set_consumer(dev, topaz_priv->consumer);
	} while (topaz_priv->consumer != topaz_priv->producer);

	return 0;
}

int32_t tng_wait_on_sync(
	struct drm_device *dev,
	int32_t sync_seq,
	unsigned long cmd_id)
{
	struct tng_topaz_private *topaz_priv;
	struct drm_psb_private *dev_priv;
	/* struct IMG_WRITEBACK_MSG *wb_msg; */
	/* struct tng_topaz_cmd_header *cur_cmd_header; */
	int32_t ret;
	int32_t count = 0;
	/* uint32_t crMultiCoreIntStat; */

	dev_priv = (struct drm_psb_private *) dev->dev_private;
	if (!dev_priv)
		DRM_ERROR("Failed to get dev_priv\n");

	topaz_priv = dev_priv->topaz_private;

	topaz_priv->consumer = tng_get_consumer(dev);

#ifdef TOPAZHP_IRQ_ENABLED
	do {
		topaz_priv->producer = tng_get_producer(dev);
		PSB_UDELAY(1000);
		count++;
	} while (topaz_priv->producer == topaz_priv->consumer &&
		 count < LOOP_COUNT);

	if (count == LOOP_COUNT) {
		DRM_ERROR("Waiting for IRQ timeout\n");
		return -1;
	}
#else
	topaz_priv->producer = tng_get_producer(dev);

	/* Read and compare consumer and producer */
	if (topaz_priv->producer == topaz_priv->consumer) {
		PSB_DEBUG_GENERAL("TOPAZ: producer = consumer = %d, " \
			"polling producer until change\n",
			topaz_priv->producer);
		/* if the same -> poll on Producer change */
		ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
			TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				(MTX_SCRATCHREG_TOHOST << 2),
			topaz_priv->consumer, MASK_WB_PRODUCER);
		if (ret) {

			DRM_ERROR("Polling timeout\n");

			return ret;
		}

		topaz_priv->producer = tng_get_producer(dev);
	}
#endif
	/* Dispatch new messages */
	do {
		PSB_DEBUG_GENERAL("TOPAZ: Dispatch write back message, " \
			"producer = %d, consumer = %d\n",
			topaz_priv->producer, topaz_priv->consumer);
		ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_NOTEQUAL,
			TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				(MTX_SCRATCHREG_TOHOST << 2),
			topaz_priv->consumer, MASK_WB_PRODUCER);
		if (ret)
			return ret;

		topaz_priv->consumer++;
		if (topaz_priv->consumer == WB_FIFO_SIZE)
			topaz_priv->consumer = 0;

		/* Indicate buffer used by consmer is available */
		tng_set_consumer(dev, topaz_priv->consumer);
	} while (topaz_priv->consumer != topaz_priv->producer);

	/* When IRQ enabled, the belowing code will be called in ISR */
#ifdef TOPAZHP_IRQ_ENABLED
	return 0;
#endif

	if (cmd_id != MTX_CMDID_ENCODE_FRAME &&
	    cmd_id != MTX_CMDID_ISSUEBUFF)
		return 0;

	PSB_DEBUG_GENERAL("TOPAZ: Handle context saving/fence " \
		"handler/dequeue send on ENCODE_FRAME command\n");

	if (get_ctx_cnt(dev) > 1) {
		PSB_DEBUG_GENERAL("TOPAZ: More than one context, " \
			"save current context\n");
		if (topaz_priv->cur_context->codec != IMG_CODEC_JPEG) {
			ret = tng_topaz_save_mtx_state(dev);
			if (ret) {
				DRM_ERROR("Failed to save mtx status");
				return ret;
			}
		}
	}

	*topaz_priv->topaz_sync_addr = sync_seq;

	psb_fence_handler(dev, LNC_ENGINE_ENCODE);

	topaz_priv->topaz_busy = 1;
	tng_topaz_dequeue_send(dev);

	return ret;
}

bool tng_topaz_interrupt(void *pvData)
{
	struct drm_device *dev;
	/* struct drm_minor *minor; */
	struct tng_topaz_private *topaz_priv;
	struct drm_psb_private *dev_priv;
	uint32_t crMultiCoreIntStat;
	struct psb_video_ctx *video_ctx;
	struct IMG_WRITEBACK_MSG *wb_msg;

	if (pvData == NULL) {
		DRM_ERROR("Invalid params\n");
		return false;
	}
	dev = (struct drm_device *)pvData;
	/*
	minor = (struct drm_minor *)container_of(dev, struct drm_minor, dev);
	file_priv = (struct drm_file *)container_of(minor,
			struct drm_file, minor);
	*/
	/*
	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
		DRM_ERROR("ERROR: interrupt arrived but HW is power off\n");
		return false;
	}
	*/
	dev_priv = (struct drm_psb_private *) dev->dev_private;
	if (!dev_priv)
		DRM_ERROR("Failed to get dev_priv\n");

	topaz_priv = dev_priv->topaz_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_INT_STAT,
			 &crMultiCoreIntStat);

	/* if interrupts enabled and fired */
	if ((crMultiCoreIntStat & MASK_TOPAZHP_TOP_CR_INT_STAT_MTX) ==
		MASK_TOPAZHP_TOP_CR_INT_STAT_MTX) {
		PSB_DEBUG_GENERAL("TOPAZ: Get MTX interrupt , clear IRQ\n");
		tng_topaz_Int_clear(dev_priv, MASK_TOPAZHP_TOP_CR_INTCLR_MTX);
	} else
		return 0;

	topaz_priv->consumer = tng_get_consumer(dev);
	topaz_priv->producer = tng_get_producer(dev);

	video_ctx = topaz_priv->irq_context;

	wb_msg = (struct IMG_WRITEBACK_MSG *)
		video_ctx->wb_addr[(topaz_priv->producer == 0) \
			? 31 \
			: topaz_priv->producer - 1];

	PSB_DEBUG_GENERAL("TOPAZ: Dispatch write back message, " \
		"producer = %d, consumer = %d\n",
		topaz_priv->producer, topaz_priv->consumer);

	if (video_ctx->codec != IMG_CODEC_JPEG) {
		while (topaz_priv->consumer != topaz_priv->producer) {
			topaz_priv->consumer++;
			if (topaz_priv->consumer == WB_FIFO_SIZE)
				topaz_priv->consumer = 0;
			tng_set_consumer(dev, topaz_priv->producer);
		};
	}

	PSB_DEBUG_GENERAL("TOPAZ: Context %08x(%s), command %s IRQ\n",
		(unsigned int)video_ctx, codec_to_string(video_ctx->codec),
		cmd_to_string(wb_msg->ui32CmdWord));

	if (video_ctx->codec == IMG_CODEC_JPEG) {
		if (wb_msg->ui32CmdWord != MTX_CMDID_NULL) {
			/* The LAST ISSUEBUF cmd means encoding complete */
			if (--topaz_priv->issuebuf_cmd_count) {
				PSB_DEBUG_GENERAL("TOPAZ: JPEG ISSUEBUF cmd " \
					  "count left %d, return\n", \
					  topaz_priv->issuebuf_cmd_count);
			return true;
			}
		} else {
			return true;
		}
	}

	*topaz_priv->topaz_sync_addr = wb_msg->ui32WritebackVal;

	PSB_DEBUG_GENERAL("TOPAZ: Set seq %08x, " \
		"dqueue cmd and schedule other work queue\n",
		wb_msg->ui32WritebackVal);
	psb_fence_handler(dev, LNC_ENGINE_ENCODE);

	/* Launch the task anyway */
	schedule_delayed_work(&topaz_priv->topaz_suspend_work, 0);

	return true;
}

static int tng_submit_encode_cmdbuf(struct drm_device *dev,
				    struct drm_file *file_priv,
				    struct ttm_buffer_object *cmd_buffer,
				    uint32_t cmd_offset, uint32_t cmd_size,
				    struct ttm_fence_object *fence)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned long irq_flags;
	int ret = 0;
	void *cmd;
	uint32_t sequence = dev_priv->sequence[LNC_ENGINE_ENCODE];
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: command submit, topaz busy = %d\n",
		topaz_priv->topaz_busy);

	if (topaz_priv->topaz_fw_loaded == 0) {
		/* #.# load fw to driver */
		PSB_DEBUG_INIT("TOPAZ: load /lib/firmware/topazhp_fw.bin\n");
		ret = tng_topaz_init_fw(dev);
		if (ret) {
			/* FIXME: find a proper return value */
			DRM_ERROR("TOPAX:load /lib/firmware/topaz_fwsc.bin" \
				  " fails, ensure udevd is configured" \
				  " correctly!\n");
			return -EFAULT;
		}
		topaz_priv->topaz_fw_loaded = 1;
	}

	tng_topaz_mmu_flushcache(dev_priv);

	/* # schedule watchdog */
	/* psb_schedule_watchdog(dev_priv); */

	/* # spin lock irq save [msvdx_lock] */
	spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);

	/* # if topaz need to reset, reset it */
	if (topaz_priv->topaz_needs_reset) {
		/* #.# reset it */
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);
		PSB_DEBUG_GENERAL("TOPAZ: needs reset.\n");

		tng_topaz_reset(dev_priv);

		PSB_DEBUG_GENERAL("TOPAZ: reset ok.\n");

		/* #.# upload firmware */
		ret = tng_topaz_setup_fw(dev, 0, topaz_priv->cur_codec);
		if (ret) {
			DRM_ERROR("TOPAZ: upload FW to HW failed\n");
			return ret;
		}

		spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags);
	}

	if (!topaz_priv->topaz_busy) {
		/* # direct map topaz command if topaz is free */
		PSB_DEBUG_GENERAL("TOPAZ:direct send command,sequence %08x\n",
				  sequence);

		topaz_priv->topaz_busy = 1;
		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

		ret = tng_topaz_deliver_command(dev, file_priv,
			cmd_buffer, cmd_offset, cmd_size, NULL, sequence, 0);

		if (ret) {
			DRM_ERROR("TOPAZ: failed to extract cmd...\n");
			return ret;
		}
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: queue command of sequence %08x\n",
				  sequence);
		cmd = NULL;

		spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags);

		ret = tng_topaz_deliver_command(dev, file_priv,
			cmd_buffer, cmd_offset, cmd_size, &cmd, sequence, 1);
		if (cmd == NULL || ret) {
			DRM_ERROR("TOPAZ: map command for save fialed\n");
			return ret;
		}

		ret = tng_topaz_save_command(dev, file_priv,
			cmd, cmd_size, sequence);
		if (ret)
			DRM_ERROR("TOPAZ: save command failed\n");
	}

	return ret;
}

static int tng_topaz_save_command(
	struct drm_device *dev,
	struct drm_file *file_priv,
	void *cmd,
	uint32_t cmd_size,
	uint32_t sequence)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_cmd_queue *topaz_cmd;
	unsigned long irq_flags;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;

	PSB_DEBUG_GENERAL("TOPAZ: queue command,sequence: %08x..\n",
			  sequence);

	topaz_cmd = kzalloc(sizeof(struct tng_topaz_cmd_queue),
			    GFP_KERNEL);
	if (!topaz_cmd) {
		DRM_ERROR("TOPAZ: out of memory....\n");
		return -ENOMEM;
	}

	topaz_cmd->file_priv = file_priv;
	topaz_cmd->cmd = cmd;
	topaz_cmd->cmd_size = cmd_size;
	topaz_cmd->sequence = sequence;

	/* spin_lock_irqsave(&topaz_priv->topaz_lock, irq_flags); */
	/* Avoid race condition with dequeue buffer in kernel task */
	mutex_lock(&topaz_priv->topaz_mutex);
	list_add_tail(&topaz_cmd->head, &topaz_priv->topaz_queue);
	mutex_unlock(&topaz_priv->topaz_mutex);

	if (!topaz_priv->topaz_busy) {
		/* topaz_priv->topaz_busy = 1; */
		PSB_DEBUG_GENERAL("TOPAZ: need immediate dequeue...\n");
		tng_topaz_dequeue_send(dev);
		PSB_DEBUG_GENERAL("TOPAZ: after dequeue command\n");
	}
	/* spin_unlock_irqrestore(&topaz_priv->topaz_lock, irq_flags); */

	return 0;
}

int tng_cmdbuf_video(struct drm_file *file_priv,
		     struct list_head *validate_list,
		     uint32_t fence_type,
		     struct drm_psb_cmdbuf_arg *arg,
		     struct ttm_buffer_object *cmd_buffer,
		     struct psb_ttm_fence_rep *fence_arg)
{
	struct drm_device *dev = file_priv->minor->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_fence_object *fence = NULL;
	int32_t ret = 0;

	PSB_DEBUG_GENERAL("TOPAZ : enter %s cmdsize: %d\n", __func__,
			  arg->cmdbuf_size);

	ret = tng_submit_encode_cmdbuf(dev, file_priv, cmd_buffer,
		arg->cmdbuf_offset, arg->cmdbuf_size, fence);
	if (ret)
		return ret;

	/* workaround for interrupt issue */
	psb_fence_or_sync(file_priv, LNC_ENGINE_ENCODE,
		fence_type, arg->fence_flags,
		validate_list, fence_arg, &fence);
	PSB_DEBUG_GENERAL("TOPAZ : current fence 0x%08x\n",
		dev_priv->sequence[LNC_ENGINE_ENCODE]);
	if (fence)
		ttm_fence_object_unref(&fence);

	spin_lock(&cmd_buffer->bdev->fence_lock);
	if (cmd_buffer->sync_obj != NULL)
		ttm_fence_sync_obj_unref(&cmd_buffer->sync_obj);
	spin_unlock(&cmd_buffer->bdev->fence_lock);

	PSB_DEBUG_GENERAL("TOPAZ exit %s\n", __func__);
	return ret;
}

#define SHIFT_MTX_CMDWORD_ID    (0)
#define MASK_MTX_CMDWORD_ID     (0xff << SHIFT_MTX_CMDWORD_ID)
#define SHIFT_MTX_CMDWORD_CORE  (8)
#define MASK_MTX_CMDWORD_CORE   (0xff << SHIFT_MTX_CMDWORD_CORE)
#define SHIFT_MTX_CMDWORD_COUNT (16)
#define MASK_MTX_CMDWORD_COUNT  (0xffff << SHIFT_MTX_CMDWORD_COUNT)

#define SHIFT_MTX_WBWORD_ID    (0)
#define MASK_MTX_WBWORD_ID     (0xff << SHIFT_MTX_WBWORD_ID)
#define SHIFT_MTX_WBWORD_CORE  (8)
#define MASK_MTX_WBWORD_CORE   (0xff << SHIFT_MTX_WBWORD_CORE)

static int tng_error_dump_reg(struct drm_psb_private *dev_priv)
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

int tng_topaz_deliver_command(struct drm_device *dev,
			      struct drm_file *file_priv,
			      struct ttm_buffer_object *cmd_buffer,
			      uint32_t cmd_offset, uint32_t cmd_size,
			      void **topaz_cmd, uint32_t sequence,
			      int copy_cmd)
{
	unsigned long cmd_page_offset = cmd_offset & ~PAGE_MASK;
	struct ttm_bo_kmap_obj cmd_kmap;
	bool is_iomem;
	int ret;
	unsigned char *cmd_start, *tmp;

	if (cmd_size > (cmd_buffer->num_pages << PAGE_SHIFT)) {
		DRM_ERROR("Command size %d is bigger than " \
			"command buffer size %d", cmd_size,
			(uint32_t)cmd_buffer->num_pages << PAGE_SHIFT);
		return -EINVAL;
	}

	ret = ttm_bo_kmap(cmd_buffer, cmd_offset >> PAGE_SHIFT, 2,
			  &cmd_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: drm_bo_kmap failed: %d\n", ret);
		return ret;
	}
	cmd_start = (unsigned char *) ttm_kmap_obj_virtual(&cmd_kmap,
		    &is_iomem) + cmd_page_offset;

	if (copy_cmd) {
		tmp = kzalloc(cmd_size, GFP_KERNEL);
		if (tmp == NULL) {
			ret = -ENOMEM;
			goto out;
		}
		memcpy(tmp, cmd_start, cmd_size);
		*topaz_cmd = tmp;
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: directly send the command\n");
		ret = tng_topaz_send(dev, file_priv,
			cmd_start, cmd_size, sequence);
		if (ret) {
			DRM_ERROR("TOPAZ: commit commands failed.\n");
			ret = -EINVAL;
		}
	}

out:
	ttm_bo_kunmap(&cmd_kmap);

	return ret;
}

static int32_t tng_topaz_wait_for_completion(struct drm_psb_private *dev_priv)
{
	int32_t ret = 0;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;

	mtx_set_target(dev_priv);

	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_MTX + MTX_CR_MTX_ENABLE,
		MASK_MTX_MTX_TOFF,
		MASK_MTX_MTX_TOFF | MASK_MTX_MTX_ENABLE);
	if (ret)
		DRM_ERROR("TOPAZ: Wait for MTX completion time out\n");

	topaz_priv->frame_count = 0;

	return ret;
}

static int32_t tng_restore_bias_table(
	struct drm_device *dev,
	struct psb_video_ctx *video_ctx)
{
	bool is_iomem;
	uint32_t i;
	uint32_t *p_command;
	uint32_t reg_id;
	uint32_t reg_off;
	uint32_t reg_val;
	uint32_t size;
	struct ttm_bo_kmap_obj tmp_kmap;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int32_t ret = 0;

	ret = ttm_bo_kmap(video_ctx->reg_saving_bo, 0,
			  video_ctx->reg_saving_bo->num_pages,
			  &tmp_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: Failed to map BIAS table BO\n");
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		return ret;
	}

	p_command = ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem);

	p_command += 32 * 1024 / 4;

	size = *p_command++;

	PSB_DEBUG_GENERAL("TOPAZ: Restore BIAS %d registers " \
		"of ctx %08x(%s)\n",
		size - 1, (unsigned int)video_ctx, \
		codec_to_string(video_ctx->codec));
	for (i = 0; i < size; i++) {
		reg_id = *p_command;
		p_command++;
		reg_off = *p_command;
		p_command++;
		reg_val = *p_command;
		p_command++;

		switch (reg_id) {
		case TOPAZ_MULTICORE_REG:
			MULTICORE_WRITE32(reg_off, reg_val);
			break;
		case TOPAZ_CORE_REG:
			TOPAZCORE_WRITE32(0, reg_off, reg_val);
			break;
		case TOPAZ_VLC_REG:
			VLC_WRITE32(0, reg_off, reg_val);
			break;
		default:
			DRM_ERROR("Unknown reg space id: %08x\n", reg_id);
			ttm_bo_kunmap(&tmp_kmap);
			return -1;
		}
	}

	ttm_bo_kunmap(&tmp_kmap);

	return ret;
}

static void tng_reset_ESB(struct drm_device *dev)
{
	uint32_t num_pipes;
	uint32_t index;
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG, &num_pipes);
	num_pipes = num_pipes & MASK_TOPAZHP_TOP_CR_NUM_CORES_SUPPORTED;

	/* initialise the ESBs */
	for (index = 0; index < num_pipes; index++) {
		uint32_t n = 0;
		uint32_t reg_offset = TOPAZHP_CR_PROC_ESB_ACCESS_WORD0;

		for (n = 0; n < 4; n++, reg_offset += 4)
			TOPAZCORE_WRITE32(index, reg_offset, 0xEd15Dead);

		for (n = 0; n < (10 * 1024); n += 4)
			TOPAZCORE_WRITE32(index,
			TOPAZHP_CR_PROC_ESB_ACCESS_CONTROL,
			F_ENCODE(n >> 4, TOPAZHP_CR_PROC_ESB_ADDR) |
			F_ENCODE(0, TOPAZHP_CR_PROC_ESB_READ_N_WRITE) |
			F_ENCODE(1, TOPAZHP_CR_PROC_ESB_OP_VALID));
	}
}

int32_t tng_topaz_restore_mtx_state(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	uint32_t *mtx_reg_state;
	int i, need_restore;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *pos, *n;
	const struct tng_topaz_codec_fw *cur_codec_fw;
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;
	int32_t ret = 0;
	uint32_t num_pipes;
	struct psb_video_ctx *video_ctx = topaz_priv->cur_context;

	/*if (!topaz_priv->topaz_mtx_saved)
		return -1;
	*/

	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if ((pos->ctx_type & 0xff) == VAEntrypointEncSlice ||
			(pos->ctx_type & 0xff) == VAEntrypointEncPicture)
			need_restore = 1;
	}

	if (0 == need_restore) {
		topaz_priv->topaz_mtx_saved = 0;
		PSB_DEBUG_GENERAL("topaz: no vec context found. needn't" \
			" to restore mtx registers.\n");
		return ret;
	}

	if (topaz_priv->topaz_fw_loaded == 0) {
		PSB_DEBUG_GENERAL("TOPAZ: needn't to restore context" \
			" without firmware uploaded\n");
		return ret;
	}

	if (video_ctx->data_saving_bo == NULL) {
		PSB_DEBUG_GENERAL("TOPAZ: try to restore context" \
			" without space allocated, return" \
			" directly without restore\n");
		ret = -1;
		return ret;
	}

	/*TopazSC will be reset, no need to restore context.*/
	if (topaz_priv->topaz_needs_reset)
		return ret;
	/*There is no need to restore context for JPEG encoding*/
	/*
	if (TNG_IS_JPEG_ENC(topaz_priv->cur_codec)) {
		if (tng_topaz_setup_fw(dev, 0, topaz_priv->cur_codec))
			DRM_ERROR("TOPAZ: Setup JPEG firmware fails!\n");
		topaz_priv->topaz_mtx_saved = 0;
		return 0;
	}
	*/
	PSB_DEBUG_GENERAL("TOPAZ: Restore context %08x(%s)\n",
		(unsigned int)video_ctx, codec_to_string(video_ctx->codec));

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG, &num_pipes);
	num_pipes = num_pipes & MASK_TOPAZHP_TOP_CR_NUM_CORES_SUPPORTED;

	mtx_set_target(dev_priv);

	tng_reset_ESB(dev);

	/* write the topaz reset bits */
	/*1) Disable MTX by writing one to the MTX_TOFF
	field of the MTX_ENABLE register*/
	MTX_WRITE32(MTX_CR_MTX_ENABLE,
		    MASK_MTX_MTX_TOFF);

	/* 2) Software reset MTX only by writing 0x1
	then 0x0 to the MULTICORE_SRST register */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 1);
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 0);

	/* 3) Software reset MTX, cores, and IO by writing 0x7
	then 0x0 to the MULTICORE_SRST register */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST,
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_IO_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET));
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_SRST, 0);

	MTX_WRITE32(MTX_CR_MTX_SOFT_RESET, MASK_MTX_MTX_RESET);

	PSB_UDELAY(6);

	MTX_WRITE32(MTX_CR_MTX_SOFT_RESET, 0);

	cur_codec_fw = &topaz_priv->topaz_fw[video_ctx->codec];

	PSB_DEBUG_GENERAL("TOPAZ: Restore status to %08x of " \
		"size %08x of context %08x(%s)\n",
		video_ctx->fw_data_dma_offset, video_ctx->fw_data_dma_size,
		(unsigned int)video_ctx, codec_to_string(video_ctx->codec));

	PSB_DEBUG_GENERAL("TOPAZ: Restore MTX core registers\n");

	ret = ttm_bo_kmap(video_ctx->reg_saving_bo, 0,
		video_ctx->reg_saving_bo->num_pages, &tmp_kmap);

	if (ret) {
		DRM_ERROR("TOPAZ: Failed to map reg saving BO\n");
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		return ret;
	}

	mtx_reg_state = (uint32_t *)ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem);

	/* restore register */
	/* Saves 8 Registers of D0 Bank  */
	/* DoRe0, D0Ar6, D0Ar4, D0Ar2, D0FrT, D0.5, D0.6 and D0.7 */
	for (i = 0; i < 8; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x1 | (i << 4),
			*mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 8 Registers of D1 Bank  */
	/* D1Re0, D1Ar5, D1Ar3, D1Ar1, D1RtP, D1.5, D1.6 and D1.7 */
	for (i = 0; i < 8; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x2 | (i << 4),
			*mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 4 Registers of A0 Bank  */
	/* A0StP, A0FrP, A0.2 and A0.3 */
	for (i = 0; i < 4; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x3 | (i << 4),
			*mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 4 Registers of A1 Bank  */
	/* A1GbP, A1LbP, A1.2 and A1.3 */
	for (i = 0; i < 4; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x4 | (i << 4),
			*mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves PC and PCX  */
	for (i = 0; i < 2; i++) {
		ret = mtx_write_core_reg(dev_priv, 0x5 | (i << 4),
			*mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to write core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 8 Control Registers */
	/* TXSTAT, TXMASK, TXSTATI, TXMASKI, TXPOLL, TXGPIOI, TXPOLLI,
	 * TXGPIOO */

	/* NOTE: TXSTAT, TXSTATI, TXPOLL,
	TXPOLLI contain non-restorable states */
	for (i = 0; i < 8; i++) {
		if (i == 1 || i == 3 || i == 7) {
			ret = mtx_write_core_reg(dev_priv, 0x7 | (i << 4),
				*mtx_reg_state);
			if (ret) {
				DRM_ERROR("Failed to write core reg");
				goto out;
			} else
				mtx_reg_state++;
		}
	}

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_DIR_LIST_BASE(0),
			  *mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_TILE(0),
			  *mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_TILE(1),
			  *mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL2, *mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL1, *mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MMU_CONTROL0, *mtx_reg_state);
	mtx_reg_state++;

	/* CARC registers */
	for (i = 0; i < num_pipes; i++) {
		TOPAZCORE_WRITE32(i, INTEL_JMCMP_CF_TOTAL,
			*mtx_reg_state);
		mtx_reg_state++;
	}

	for (i = 0; i < num_pipes; i++) {
		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_AUTO_CLOCK_GATING,
			*mtx_reg_state);
		mtx_reg_state++;
		TOPAZCORE_WRITE32(i, TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING,
			*mtx_reg_state);
		mtx_reg_state++;
	}

	tng_topaz_mmu_flushcache(dev_priv);

	ret = mtx_upload_fw(dev, video_ctx->codec, video_ctx, 1);
	if (ret) {
		DRM_ERROR("Failed to upload firmware for codec %s\n",
			codec_to_string(video_ctx->codec));
			/* tng_error_dump_reg(dev_priv); */
		return ret;
	}

	ttm_bo_kunmap(&tmp_kmap);

	/* Turn on MTX */
	mtx_start(dev);

	ret = tng_restore_bias_table(dev, video_ctx);
	if (ret) {
		DRM_ERROR("Failed to restore BIAS table");
		goto out;
	}

	/* topaz_priv->topaz_mtx_saved = 0; */
	PSB_DEBUG_GENERAL("TOPAZ: Restore MTX status successfully\n");

	video_ctx->status &= ~MASK_TOPAZ_CONTEXT_SAVED;

#ifdef TOPAZHP_IRQ_ENABLED
	psb_irq_preinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
	psb_irq_postinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);

	tng_topaz_enableirq(dev);
#endif

	return ret;

out:
	ttm_bo_kunmap(&tmp_kmap);
	return ret;
}

static int  tng_poll_hw_inactive(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int32_t ret = 0;
	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		MTX_SCRATCHREG_IDLE,
		F_ENCODE(FW_IDLE_STATUS_IDLE, FW_IDLE_REG_STATUS),
		MASK_FW_IDLE_REG_STATUS);
	if (ret)
		DRM_ERROR("Wait for register timeout");

	return ret;
}

static int mtx_wait_for_completion(struct drm_psb_private *dev_priv)
{
	int32_t ret;

	mtx_set_target(dev_priv);

	ret = tng_topaz_wait_for_register(dev_priv, CHECKFUNC_ISEQUAL,
		REG_OFFSET_TOPAZ_MTX + MTX_CR_MTX_ENABLE,
		MASK_MTX_MTX_TOFF,
		MASK_MTX_MTX_TOFF | MASK_MTX_MTX_ENABLE);
	if (ret)
		DRM_ERROR("TOPAZ: Wait for MTX completion time out\n");

	return ret;
}

int tng_topaz_save_mtx_state(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	uint32_t *mtx_reg_state;
	int i, need_save = 0;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *pos, *n;
	struct ttm_buffer_object *target;
	struct ttm_bo_kmap_obj tmp_kmap;
	struct psb_video_ctx *video_ctx;
	bool is_iomem;
	uint32_t num_pipes;
	int32_t ret = 0;

#ifdef TOPAZHP_IRQ_ENABLED
	video_ctx = topaz_priv->irq_context;
#else
	if (topaz_priv->cur_context) {
		video_ctx = topaz_priv->cur_context;
	} else {
		DRM_ERROR("Invalid context\n");
		return -1;
	}
#endif
	PSB_DEBUG_GENERAL("TOPAZ: Save context %08x(%s)\n",
		(unsigned int)video_ctx,
		codec_to_string(video_ctx->codec));
	/* topaz_priv->topaz_mtx_saved = 0; */
	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if ((pos->ctx_type & 0xff) == VAEntrypointEncSlice ||
			(pos->ctx_type & 0xff) == VAEntrypointEncPicture)
			need_save = 1;
	}

	if (0 == need_save) {
		PSB_DEBUG_GENERAL("TOPAZ: vec context not found. No need" \
			" to save mtx registers.\n");
		return ret;
	}

	/*TopazSC will be reset, no need to save context.*/
	if (topaz_priv->topaz_needs_reset)
		return ret;

	if (topaz_priv->topaz_fw_loaded == 0) {
		PSB_DEBUG_GENERAL("TOPAZ: No need to restore context since" \
			" firmware has not been uploaded.\n");
		return -1;
	}

	/*JPEG encoding needn't to save context*/
	if (TNG_IS_JPEG_ENC(topaz_priv->cur_codec)) {
		PSB_DEBUG_GENERAL("TOPAZ: Bypass context saving for JPEG\n");
		topaz_priv->topaz_mtx_saved = 1;
		return ret;
	}

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HW_CFG, &num_pipes);
	num_pipes = num_pipes & MASK_TOPAZHP_TOP_CR_NUM_CORES_SUPPORTED;

	mtx_set_target(dev_priv);

	ret = tng_poll_hw_inactive(dev);
	if (ret)
		return ret;

	/* Turn off MTX */
	mtx_stop(dev_priv);
	ret = mtx_wait_for_completion(dev_priv);
	if (ret) {
		DRM_ERROR("Mtx wait for completion error");
		return ret;
	}

	ret = ttm_bo_kmap(video_ctx->reg_saving_bo, 0,
			  video_ctx->reg_saving_bo->num_pages,
			  &tmp_kmap);

	if (ret) {
		DRM_ERROR("TOPAZ: Failed to map reg saving BO\n");
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		return ret;
	}

	mtx_reg_state = (uint32_t *)ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem);

	/* Save MTX register states  */
	/* Saves 8 Registers of D0 Bank  */
	/* DoRe0, D0Ar6, D0Ar4, D0Ar2, D0FrT, D0.5, D0.6 and D0.7 */
	for (i = 0; i < 8; i++) {
		ret = mtx_read_core_reg(dev_priv, 0x1 | (i << 4),
			mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 8 Registers of D1 Bank  */
	/* D1Re0, D1Ar5, D1Ar3, D1Ar1, D1RtP, D1.5, D1.6 and D1.7 */
	for (i = 0; i < 8; i++) {
		ret = mtx_read_core_reg(dev_priv, 0x2 | (i << 4),
			mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			goto out;
		} else
			mtx_reg_state++;
	}

	/* Saves 4 Registers of A0 Bank  */
	/* A0StP, A0FrP, A0.2 and A0.3 */
	for (i = 0; i < 4; i++) {
		ret = mtx_read_core_reg(dev_priv, 0x3 | (i << 4),
			mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves 4 Registers of A1 Bank  */
	/* A1GbP, A1LbP, A1.2 and A1.3 */
	for (i = 0; i < 4; i++) {
		ret = mtx_read_core_reg(dev_priv, 0x4 | (i << 4),
			mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			goto out;
		} else
			mtx_reg_state++;
	}
	/* Saves PC and PCX  */
	for (i = 0; i < 2; i++) {
		ret = mtx_read_core_reg(dev_priv, 0x5 | (i << 4),
			mtx_reg_state);
		if (ret) {
			DRM_ERROR("Failed to read core reg");
			return ret;
		} else
			mtx_reg_state++;
	}
	/* Saves 8 Control Registers */
	/* TXSTAT, TXMASK, TXSTATI, TXMASKI, TXPOLL, TXGPIOI, TXPOLLI,
	 * TXGPIOO */

	/* NOTE: TXSTAT, TXSTATI, TXPOLL,
	TXPOLLI contain non-restorable states */
	for (i = 0; i < 8; i++) {
		if (i == 1 || i == 3 || i == 7) {
			ret = mtx_read_core_reg(dev_priv, 0x7 | (i << 4),
				mtx_reg_state);
			if (ret) {
				DRM_ERROR("Failed to read core reg");
				goto out;
			} else
				mtx_reg_state++;
		}
	}

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_DIR_LIST_BASE(0), mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_TILE(0), mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_TILE(1), mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_CONTROL2, mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_CONTROL1, mtx_reg_state);
	mtx_reg_state++;

	MULTICORE_READ32(TOPAZHP_TOP_CR_MMU_CONTROL0, mtx_reg_state);
	mtx_reg_state++;

	/* CARC registers */
	for (i = 0; i < num_pipes; i++) {
		TOPAZCORE_READ32(i, INTEL_JMCMP_CF_TOTAL, mtx_reg_state);
		mtx_reg_state++;
	}

	for (i = 0; i < num_pipes; i++) {
		TOPAZCORE_READ32(i, TOPAZHP_CR_TOPAZHP_AUTO_CLOCK_GATING,
			mtx_reg_state);
		mtx_reg_state++;
		TOPAZCORE_READ32(i, TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING,
			mtx_reg_state);
		mtx_reg_state++;
	}

	ttm_bo_kunmap(&tmp_kmap);

	PSB_DEBUG_GENERAL("TOPAZ: Save core registers done\n");

	target = video_ctx->data_saving_bo;

	PSB_DEBUG_GENERAL("TOPAZ: Save status from %08x of size %08x, " \
		"to BO %08x offset %08x\n",
		 video_ctx->fw_data_dma_offset, video_ctx->fw_data_dma_size,
		(unsigned int)target, (unsigned int)target->offset);

	ret = mtx_dma_read(dev, target,
			   video_ctx->fw_data_dma_offset,
			   video_ctx->fw_data_dma_size);
	if (ret) {
		DRM_ERROR("TOPAZ: mtx_dma_read failed!\n");
		goto out;
	}

	tng_topaz_mmu_flushcache(dev_priv);

	video_ctx->status |= MASK_TOPAZ_CONTEXT_SAVED;

	/* topaz_priv->topaz_mtx_saved = 1; */
	PSB_DEBUG_GENERAL("TOPAZ: Save MTX status successfully\n");

	return ret;

out:
	ttm_bo_kunmap(&tmp_kmap);
	return ret;
}

struct file *tng_get_context_fp(
	struct drm_psb_private *dev_priv,
	struct drm_file *file_priv)
{
	struct file *current_context = NULL;
	struct psb_video_ctx *pos, *n;

	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		if (pos->filp == file_priv->filp)
			current_context = pos->filp;
	}

	return current_context;
}

static int tng_save_bias_table(
	struct drm_device *dev,
	struct drm_file *file_priv,
	const void *cmd)
{
	bool is_iomem;
	uint32_t *reg_saving_ptr;
	uint32_t size;
	struct ttm_bo_kmap_obj tmp_kmap;
	uint32_t *p_command;
	struct psb_video_ctx *video_ctx;
	int32_t ret = 0;

	p_command = (uint32_t *)cmd;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);
	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		ret = -1;
		goto out;
	}

	ret = ttm_bo_kmap(video_ctx->reg_saving_bo, 0,
		video_ctx->reg_saving_bo->num_pages,
		&tmp_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: Failed to map BIAS table BO\n");
		DRM_ERROR("reg_saving_bo (0x%x), num pages (0x%x)\n",
		video_ctx->reg_saving_bo,
		video_ctx->reg_saving_bo->num_pages);
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		return ret;
	}

	reg_saving_ptr = (uint32_t *)ttm_kmap_obj_virtual(
			&tmp_kmap, &is_iomem);

	reg_saving_ptr += 32 * 1024 / 4;

	p_command++;
	/* Register count */
	size = *reg_saving_ptr = *p_command;
	PSB_DEBUG_GENERAL("TOPAZ: Save BIAS table %d registers " \
		"for context %08x\n", size, (unsigned int)video_ctx);

	p_command++;
	reg_saving_ptr++;

	memcpy(reg_saving_ptr, p_command, size * 3 * 4);

	ttm_bo_kunmap(&tmp_kmap);

out:
	return ret;
}

/*
 * Check contxt status and assign ID for new context
 * Return -1 on error
 */
static int tng_check_context_status(
	struct drm_device *dev,
	struct drm_file *file_priv,
	uint32_t reg_handle,
	uint32_t data_handle,
	uint32_t codec,
	uint32_t *buf_idx)
{
	return 0;
}

/*
 * If current context issued MTX_CMDID_SHUTDOWN command, mark the ctx_status,
 * clean related reg/data BO, write back BO. Return -1 on the last context.
 */
static int32_t tng_release_context(
	struct drm_device *dev,
	struct drm_file *file_priv,
	uint32_t cur_codec)
{
	struct psb_video_ctx *video_ctx;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);
	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		return -1;
	}

	if (cur_codec != IMG_CODEC_JPEG) {
		PSB_DEBUG_GENERAL("TOPAZ: Unmap reg/data saving BO\n");
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		video_ctx->reg_saving_bo = NULL;

		ttm_bo_unref(&video_ctx->data_saving_bo);
		video_ctx->data_saving_bo = NULL;

		video_ctx->status |= MASK_TOPAZ_FIRMWARE_EXIT;
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: JPEG bypass unmap " \
			"reg/data saving BO\n");
	}

	return 0;
}

int tng_topaz_kick_null_cmd(struct drm_device *dev,
			    uint32_t sync_seq)
{
	uint32_t cur_free_space;
	uint32_t wb_val;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

#ifdef TOPAZHP_SERIALIZED
	uint32_t serializeToken;
	serializeToken = tng_serialize_enter(dev);
#endif

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE,
			 &cur_free_space);

	cur_free_space = F_DECODE(cur_free_space,
			TOPAZHP_TOP_CR_CMD_FIFO_SPACE);

	while (cur_free_space < 4) {
		POLL_TOPAZ_FREE_FIFO_SPACE(4, 100, 10000, &cur_free_space);
		if (ret) {
			DRM_ERROR("TOPAZ : error ret %d\n", ret);
			return ret;
		}

		MULTICORE_READ32(
			TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE,
			&cur_free_space);

		cur_free_space = F_DECODE(cur_free_space,
				TOPAZHP_TOP_CR_CMD_FIFO_SPACE);
	}

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  MTX_CMDID_NULL | MTX_CMDID_WB_INTERRUPT);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  0);

	/* Write back address is always 0 */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  0);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			 sync_seq);

	PSB_DEBUG_GENERAL("TOPAZ: Write to command FIFO: " \
		"%08x, %08x, %08x, %08x\n",
		MTX_CMDID_NULL, 0, 0, 0);

	/* Notify ISR which context trigger interrupt */
#ifdef TOPAZHP_IRQ_ENABLED
	topaz_priv->irq_context = topaz_priv->cur_context;
#endif
	mtx_kick(dev);

#ifdef TOPAZHP_SERIALIZED
	tng_serialize_exit(dev, serializeToken);
#endif

	return ret;
}

int mtx_write_FIFO(
	struct drm_device *dev,
	struct tng_topaz_cmd_header *cmd_header,
	uint32_t param,
	uint32_t param_addr,
	uint32_t sync_seq)
{
	uint32_t cur_free_space;
	uint32_t wb_val;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

#ifdef TOPAZHP_SERIALIZED
	uint32_t serializeToken;
	serializeToken = tng_serialize_enter(dev);
#endif
	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE,
			 &cur_free_space);

	cur_free_space = F_DECODE(cur_free_space,
			TOPAZHP_TOP_CR_CMD_FIFO_SPACE);

	while (cur_free_space < 4) {
		POLL_TOPAZ_FREE_FIFO_SPACE(4, 100, 10000, &cur_free_space);
		if (ret) {
			DRM_ERROR("TOPAZ : error ret %d\n", ret);
			return ret;
		}

		MULTICORE_READ32(
			TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE,
			&cur_free_space);

		cur_free_space = F_DECODE(cur_free_space,
				TOPAZHP_TOP_CR_CMD_FIFO_SPACE);
	}

	/* Trigger interrupt on MTX_CMDID_ENCODE_FRAME cmd */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
				  cmd_header->val);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  param);

	/* Write back address is always 0 */
	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  param_addr);

	/* prepare Writeback value */
	wb_val = (cmd_header->id & MTX_CMDID_PRIORITY) ?
		 cmd_header->high_cmd_count << 24 :
		 cmd_header->low_cmd_count << 16;

	topaz_priv->low_cmd_count = cmd_header->low_cmd_count;
	topaz_priv->core_id = cmd_header->core;

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			  wb_val);
	PSB_DEBUG_GENERAL("TOPAZ: Write to command FIFO: " \
		"%08x, %08x, %08x, %08x\n",
		cmd_header->val, param, param_addr, wb_val);

	/* Notify ISR which context trigger interrupt */
#ifdef TOPAZHP_IRQ_ENABLED
	topaz_priv->irq_context = topaz_priv->cur_context;
#endif
	mtx_kick(dev);

#ifdef TOPAZHP_SERIALIZED
	tng_serialize_exit(dev, serializeToken);
#endif

	return ret;
}

static int tng_context_switch(
	struct drm_device *dev,
	struct drm_file *file_priv,
	uint32_t codec,
	uint32_t is_first_frame)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *video_ctx;
	int32_t ret = 0;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);
	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		return -1;
	}

	if (codec == IMG_CODEC_JPEG) {
		PSB_DEBUG_GENERAL("TOPAZ: JPEG context(%08x), " \
			"continue doing other commands\n",
			(unsigned int)video_ctx);
		topaz_priv->cur_context = video_ctx;
		topaz_priv->cur_codec = codec;
		return ret;
	}

	/* Continue doing other commands */
	if (is_first_frame) {
		PSB_DEBUG_GENERAL("TOPAZ: First frame of ctx %08x(%s), " \
			"continue doing other commands\n",
			(unsigned int)video_ctx, codec_to_string(codec));
		topaz_priv->cur_context = video_ctx;
		topaz_priv->cur_codec = codec;
		return ret;
	}

	/* Continue doing other commands */
	if ((topaz_priv->cur_context == video_ctx) &&
		!(video_ctx->status & MASK_TOPAZ_CONTEXT_SAVED)) {
		PSB_DEBUG_GENERAL("TOPAZ: Current context equals " \
			"incoming context %08x(%s) status %08x, " \
			"continue doing other commands\n",
			(unsigned int)video_ctx, codec_to_string(codec),\
			video_ctx->status);
		topaz_priv->cur_context = video_ctx;
		topaz_priv->cur_codec = codec;
		return ret;
	} else {
		PSB_DEBUG_GENERAL("TOPAZ: Restore context %08x(%s)" \
			" status %08x\n", video_ctx, \
			codec_to_string(video_ctx->codec), \
			video_ctx->status);
		/* Context switch */
		topaz_priv->cur_context = video_ctx;
		topaz_priv->cur_codec = codec;
		ret = tng_topaz_restore_mtx_state(dev);
		if (ret) {
			DRM_ERROR("Failed to restore mtx status");
			return ret;
		}

		topaz_priv->cur_context = video_ctx;
		topaz_priv->cur_codec = codec;
	}

	return ret;
}

static int32_t tng_setup_WB_mem(
	struct drm_device *dev,
	struct drm_file *file_priv,
	const void *command)
{
	struct ttm_object_file *tfile = BCVideoGetPriv(file_priv)->tfile;
	uint32_t i;
	int ret;
	bool is_iomem;
	uint32_t wb_handle;
	struct psb_video_ctx *video_ctx;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);
	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		return -1;
	}

	wb_handle = *((uint32_t *)command + 3);
	PSB_DEBUG_GENERAL("TOPAZ: Map write back memory from handle %08x\n",
		wb_handle);


	video_ctx->wb_bo = ttm_buffer_object_lookup(tfile, wb_handle);

	PSB_DEBUG_GENERAL("TOPAZ: wb_bo 0x%08x, 0x%08x\n",
		video_ctx->wb_bo, wb_handle);
	PSB_DEBUG_GENERAL("TOPAZ: wb_bo num page, 0x%08x\n",
		video_ctx->wb_bo->num_pages);
	if (unlikely(video_ctx->wb_bo == NULL)) {
		DRM_ERROR("TOPAZ: error wb_bo\n");
		return -1;
	}

	ret = ttm_bo_reserve(video_ctx->wb_bo , true, true, false, 0);
	if (ret) {
		DRM_ERROR("TOPAZ: reserver failed.\n");
		return;
	}

	ret = ttm_bo_kmap(video_ctx->wb_bo, 0,
			  video_ctx->wb_bo->num_pages,
			  &video_ctx->wb_bo_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: Failed to map topaz WriteBack BO......\n");
		ttm_bo_unref(&video_ctx->wb_bo);
		return -1;
	}

	video_ctx->wb_addr[0] = (uint32_t)ttm_kmap_obj_virtual(
				&video_ctx->wb_bo_kmap, &is_iomem);

	memset((void *)video_ctx->wb_addr[0], ~0x0,
		sizeof(struct IMG_WRITEBACK_MSG));

	for (i = 1; i < WB_FIFO_SIZE; i++) {
		video_ctx->wb_addr[i] = video_ctx->wb_addr[i - 1] + 0x1000;
		memset((void *)video_ctx->wb_addr[i], ~0x0,
			sizeof(struct IMG_WRITEBACK_MSG));
	}

	return ret;
}

static int tng_setup_new_context(
	struct drm_device *dev,
	struct drm_file *file_priv,
	uint32_t *cmd,
	uint32_t codec)
{
	struct ttm_object_file *tfile = BCVideoGetPriv(file_priv)->tfile;
	struct psb_video_ctx *video_ctx;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

	video_ctx = get_ctx_from_fp(dev, file_priv->filp);
	if (video_ctx == NULL) {
		DRM_ERROR("Failed to get video contex from filp");
		ret = -1;
		goto out;
	}

	video_ctx->codec = codec;
	video_ctx->status = 0;

	PSB_DEBUG_GENERAL("TOPAZ: new context %08x(%s)\n",
		(unsigned int)video_ctx, codec_to_string(codec));

	if (video_ctx->codec != IMG_CODEC_JPEG) {
		video_ctx->reg_saving_bo = ttm_buffer_object_lookup(
						tfile, *(cmd + 2));
		if (unlikely(video_ctx->reg_saving_bo == NULL)) {
			DRM_ERROR("Failed to lookup (0x%x) handle\n",
				*(cmd + 2));
			DRM_ERROR("in new context");
			ret = -1;
			goto out;
		}

		ret = ttm_bo_reserve(video_ctx->reg_saving_bo , \
				     true, true, false, 0);
		if (ret) {
			DRM_ERROR("Reserver register saving BO failed.\n");
			return -1;
		}

		video_ctx->data_saving_bo = ttm_buffer_object_lookup(
						tfile, *(cmd + 3));
		if (unlikely(video_ctx->data_saving_bo == NULL)) {
			DRM_ERROR("Failed to lookup (0x%x) handle\n",
				*(cmd + 2));
			DRM_ERROR("in new context");
			ret = -1;
			goto out;
		}

		ret = ttm_bo_reserve(video_ctx->data_saving_bo , \
				     true, true, false, 0);
		if (ret) {
			DRM_ERROR("Reserver data saving BO failed.\n");
			return -1;
		}
	} else {
		video_ctx->reg_saving_bo = NULL;
		video_ctx->data_saving_bo = NULL;
		topaz_priv->issuebuf_cmd_count = *(cmd + 2);
		PSB_DEBUG_GENERAL("TOPAZ: JPEG ISSUEBUF cmd count is " \
				  "%d\n", topaz_priv->issuebuf_cmd_count);
	}

	/*
	topaz_priv->frame_h = (uint16_t)((*((uint32_t *) cmd + 1)) & 0xffff);
	topaz_priv->frame_w = (uint16_t)(((*((uint32_t *) cmd + 1))
				& 0xffff0000)  >> 16) ;
	*/

	/* Upload the new codec firmware */
	ret = tng_topaz_init_board(dev, video_ctx, codec);
	if (ret) {
		DRM_ERROR("TOPAZ: init board failed\n");
		/* tng_error_dump_reg(dev_priv, 0); */
		goto out;
	}


	/* Upload the new codec firmware */
	ret = tng_topaz_setup_fw(dev, video_ctx, codec);
	if (ret) {
		DRM_ERROR("TOPAZ: upload FW to HW failed\n");
		/* tng_error_dump_reg(dev_priv, 0); */
		goto out;
	}
out:
	return ret;
}

static int32_t tng_check_bias_register(uint32_t reg_id, uint32_t reg_off)
{
	switch (reg_id) {
	case TOPAZ_MULTICORE_REG:
		if (reg_off < REG_MIN_TOPAZ_MULTICORE ||
			reg_off > REG_MAX_TOPAZ_MULTICORE) {
			DRM_ERROR("Invalid MULTICORE register %08x\n",
				reg_off);
			return -1;
		}
		break;
	case TOPAZ_CORE_REG:
		if (reg_off < REG_MIN_TOPAZ_CORE ||
			reg_off > REG_MAX_TOPAZ_CORE) {
			DRM_ERROR("Invalid CORE register %08x\n", reg_off);
			return -1;
		}
		break;
	case TOPAZ_VLC_REG:
		if (reg_off < REG_MIN_TOPAZ_VLC ||
			reg_off > REG_MAX_TOPAZ_VLC) {
			DRM_ERROR("Invalid VLC register %08x\n", reg_off);
			return -1;
		}
		break;
	default:
		DRM_ERROR("Unknown reg space id: %08x\n", reg_id);
		return -1;
	}

	return 0;
}

static int tng_topaz_set_bias(
	struct drm_device *dev,
	struct drm_file *file_priv,
	const uint32_t *command,
	uint32_t codec,
	uint32_t *cmd_size)
{
	uint32_t reg_id, reg_off, reg_val, reg_cnt;
	uint32_t *p_command;
	struct drm_psb_private *dev_priv;
	int ret = 0;

	dev_priv = dev->dev_private;
	p_command = (uint32_t *)command;
	p_command++;
	*cmd_size = *p_command;
	p_command++;

	PSB_DEBUG_GENERAL("TOPAZ: Start to write %d Registers\n", *cmd_size);
	for (reg_cnt = 0; reg_cnt < *cmd_size; reg_cnt++) {
		/* Reg space ID */
		reg_id = *p_command;
		p_command++;
		/* Reg offset */
		reg_off = *p_command;
		p_command++;
		/* Reg value */
		reg_val = *p_command;
		p_command++;

		ret = tng_check_bias_register(reg_id, reg_off);
		if (ret) {
			DRM_ERROR("Failed in checking BIAS register");
			return ret;
		}

		switch (reg_id) {
		case TOPAZ_MULTICORE_REG:
			MULTICORE_WRITE32(reg_off, reg_val);
			break;
		case TOPAZ_CORE_REG:
			TOPAZCORE_WRITE32(0, reg_off, reg_val);
			break;
		case TOPAZ_VLC_REG:
			VLC_WRITE32(0, reg_off, reg_val);
			break;
		default:
			DRM_ERROR("Unknown reg space id: (%08x)\n", reg_id);
			return -1;
		}
	}

	p_command = (uint32_t *)command;

	/* For now, saving BIAS table no matter necessary or not */
	ret = tng_save_bias_table(dev, file_priv, p_command);
	if (ret) {
		DRM_ERROR("Failed to save BIAS table");
		return ret;
	}

	/* Update Globals */
	if (codec != IMG_CODEC_JPEG) {

		uint32_t ui32ToMtxReg = 0;
		uint32_t ui32ToHostReg = 0;

		MULTICORE_WRITE32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			(MTX_SCRATCHREG_TOMTX<<2), ui32ToMtxReg);

		MULTICORE_WRITE32(TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			(MTX_SCRATCHREG_TOHOST<<2), ui32ToHostReg);

		PSB_DEBUG_GENERAL("TOPAZ: Init producer(%08x) " \
			"and consumer(%08x)\n", ui32ToMtxReg, ui32ToHostReg);
	}

	return ret;
}
/* #define MULTI_STREAM_TEST */

int
tng_topaz_send(
	struct drm_device *dev,
	struct drm_file *file_priv,
	void *cmd,
	uint32_t cmd_size_in,
	uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned char *command = (unsigned char *) cmd;
	struct tng_topaz_cmd_header *cur_cmd_header;
	uint32_t cur_cmd_id;
	uint32_t codec = 0;
	int32_t cur_cmd_size = 4;
	int32_t cmd_size = (int32_t)cmd_size_in;
	unsigned long irq_flags;

	/* uint32_t m; */
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;
#ifdef MULTI_STREAM_TEST
	struct psb_video_ctx *video_ctx;
#endif

	PSB_DEBUG_GENERAL("TOPAZ : send the command in the buffer " \
		"one by one, cmdsize(%d), sequence(%08x)\n",
		cmd_size, sync_seq);

	/*
	printk(KERN_ERR "\t\t\tFrame number = %d\n", \
	       topaz_priv->frame_count);
	*/

	while (cmd_size > 0) {
		cur_cmd_header = (struct tng_topaz_cmd_header *) command;
		cur_cmd_id = cur_cmd_header->id;

		PSB_DEBUG_GENERAL("TOPAZ : cmd is(%s), " \
			"remaining cmd size is(%d)\n",
			cmd_to_string(cur_cmd_id & (~MTX_CMDID_PRIORITY)),
			cmd_size);

		switch (cur_cmd_id) {
		case MTX_CMDID_SW_NEW_CODEC:
			codec = (*((uint32_t *) cmd) & 0xFF00) >> 8;
			cur_cmd_size = (codec == IMG_CODEC_JPEG) ? 3 : 4;
			ret = tng_setup_new_context(dev, file_priv,
				(uint32_t *)command,
				codec);
			if (ret) {
				DRM_ERROR("Failed to setup new context");
				return ret;
			}

			break;
		case MTX_CMDID_SW_ENTER_LOWPOWER:
			PSB_DEBUG_GENERAL("TOPAZ : Enter lowpower....\n");
			PSB_DEBUG_GENERAL("XXX : implement it\n");
			cur_cmd_size = 1;
			break;

		case MTX_CMDID_SW_LEAVE_LOWPOWER:
			cur_cmd_size = 2;
			ret = tng_context_switch(dev, file_priv,
				*((uint32_t *)command + 1),
				((*((uint32_t *)command) & 0xFF00) >> 8));
			if (ret) {
				DRM_ERROR("Failed to switch context");
				return ret;
			}

			break;
		case MTX_CMDID_SW_WRITEREG:
			ret = tng_topaz_set_bias(dev, file_priv,
				(const uint32_t *)command, codec,
				&cur_cmd_size);
			if (ret) {
				DRM_ERROR("Failed to set BIAS table");
				return ret;
			}

			/*
			* cur_cmd_size if the register count here,
			* reg_id, reg_off and reg_val are stored in a 3 words
			* */
			cur_cmd_size *= 3;
			/* Header size, 2 words */
			cur_cmd_size += 2;
			break;
		case MTX_CMDID_PAD:
			/*Ignore this command, which is used to skip
			 * some commands in user space*/
			cur_cmd_size = 4;
			break;

		/* Ordinary commmand */
		case MTX_CMDID_SETVIDEO:
		case MTX_CMDID_SETUP_INTERFACE:
			ret = tng_setup_WB_mem(dev, file_priv,
				(const void *)command);
			if (ret) {
				DRM_ERROR("Failed to setup " \
					"write back memory region");
				return ret;
			}

		case MTX_CMDID_DO_HEADER:
		case MTX_CMDID_ENCODE_FRAME:
		case MTX_CMDID_GETVIDEO:
		case MTX_CMDID_PICMGMT:
		case (MTX_CMDID_PICMGMT | MTX_CMDID_PRIORITY):

		case MTX_CMDID_START_FRAME:
		case MTX_CMDID_ENCODE_SLICE:
		case MTX_CMDID_END_FRAME:
		case MTX_CMDID_RC_UPDATE:
		case (MTX_CMDID_RC_UPDATE | MTX_CMDID_PRIORITY):
		case MTX_CMDID_PROVIDE_SOURCE_BUFFER:
		case MTX_CMDID_PROVIDE_REF_BUFFER:
		case MTX_CMDID_PROVIDE_CODED_BUFFER:
		case (MTX_CMDID_PROVIDE_SOURCE_BUFFER | MTX_CMDID_PRIORITY):
		case (MTX_CMDID_PROVIDE_REF_BUFFER | MTX_CMDID_PRIORITY):
		case (MTX_CMDID_PROVIDE_CODED_BUFFER | MTX_CMDID_PRIORITY):

		case MTX_CMDID_SETQUANT:
		case MTX_CMDID_ISSUEBUFF:
		case MTX_CMDID_SETUP:
		case MTX_CMDID_SHUTDOWN:
			/*
			if (cur_cmd_header->id == MTX_CMDID_SHUTDOWN) {
				cur_cmd_size = 4;
				PSB_DEBUG_GENERAL("TOPAZ : Doesn't handle " \
					"SHUTDOWN command for now\n");
				break;
			}
			*/

			/* Write command to FIFO */
			ret = mtx_write_FIFO(dev, cur_cmd_header,
				*((uint32_t *)(command) + 1),
				*((uint32_t *)(command) + 2), sync_seq);
			if (ret) {
				DRM_ERROR("Failed to write command to FIFO");
				goto out;
			}

			tng_wait_on_sync(dev, sync_seq, cur_cmd_id);

			/*
			for (m = 0; m < 1000; m++) {
				PSB_UDELAY(100);
			}
			PSB_UDELAY(6);
			*/

			/* Calculate command size */
			switch (cur_cmd_id) {
			case MTX_CMDID_SETVIDEO:
			case MTX_CMDID_SETUP_INTERFACE:
			case MTX_CMDID_SHUTDOWN:
				cur_cmd_size = 4;
				break;
			case (MTX_CMDID_PROVIDE_SOURCE_BUFFER |
				MTX_CMDID_PRIORITY):
			case (MTX_CMDID_PROVIDE_REF_BUFFER |
				MTX_CMDID_PRIORITY):
			case (MTX_CMDID_PROVIDE_CODED_BUFFER |
				MTX_CMDID_PRIORITY):
			case (MTX_CMDID_PICMGMT | MTX_CMDID_PRIORITY):
				cur_cmd_size = 3;
				break;
			default:
				cur_cmd_size = 3;
				break;
			}


			break;
		default:
			DRM_ERROR("TOPAZ: unsupported command id: %x\n",
				  cur_cmd_id);
			return -1;
		}

		/*cur_cmd_size indicate the number of words of
		current command*/
		command += cur_cmd_size * 4;
		cmd_size -= cur_cmd_size * 4;

		PSB_DEBUG_GENERAL("TOPAZ : remaining cmd size is(%d)\n",
			cmd_size);

	}

	tng_topaz_kick_null_cmd(dev, sync_seq);

#ifdef MULTI_STREAM_TEST
	if (cur_cmd_id != MTX_CMDID_SHUTDOWN) {
		ret = tng_topaz_save_mtx_state(dev);
		if (ret) {
			DRM_ERROR("Failed to save mtx status");
			goto out;
		}

		video_ctx = get_ctx_from_fp(dev, file_priv->filp);
		if (video_ctx == NULL) {
			DRM_ERROR("Failed to get video contex from filp");
			ret = -1;
			goto out;
		}

		ret = tng_topaz_restore_mtx_state(dev);
		if (ret) {
			DRM_ERROR("Failed to restore mtx status");
			return ret;
		}
	}
#endif

	topaz_priv->frame_count++;

out:
	return ret;
}

int tng_topaz_remove_ctx(
	struct drm_psb_private *dev_priv,
	struct psb_video_ctx *video_ctx)
{
	struct tng_topaz_private *topaz_priv;
	/* struct psb_video_ctx *video_ctx; */
	struct psb_video_ctx *pos;
	int32_t ret;

	topaz_priv = dev_priv->topaz_private;
	topaz_priv->topaz_busy = 0;
	/* video_ctx = NULL; */

	/* Disable ISR */
	/*if (TOPAZHP_IRQ_ENABLED) {
		PSB_DEBUG_GENERAL("TOPAZ: Disalbe IRQ and " \
			"Wait for MTX completion\n");
		tng_topaz_disableirq(dev_priv);
	}*/

	/* Stop the MTX */
	/*
	mtx_stop(dev_priv);
	ret = mtx_wait_for_completion(dev_priv);
	if (ret) {
		DRM_ERROR("Mtx wait for completion error");
		return ret;
	}

	list_for_each_entry(pos, &dev_priv->video_ctx, head) {
		if (pos->filp == filp) {
			video_ctx = pos;
			break;
		}
	}
	*/

	if (video_ctx == NULL) {
		DRM_ERROR("Invalid video context\n");
		return -1;
	}

	PSB_DEBUG_GENERAL("TOPAZ: release context %08x(%s)\n",
		(unsigned int)video_ctx, codec_to_string(video_ctx->codec));

	/* tng_topaz_mmu_flushcache(dev_priv); */

	if (video_ctx->reg_saving_bo) {
		PSB_DEBUG_GENERAL("TOPAZ: unref reg saving bo\n");
		ttm_bo_unref(&video_ctx->reg_saving_bo);
		video_ctx->reg_saving_bo = NULL;
	}

	if (video_ctx->data_saving_bo) {
		PSB_DEBUG_GENERAL("TOPAZ: unref data saving bo\n");
		ttm_bo_unref(&video_ctx->data_saving_bo);
		video_ctx->data_saving_bo = NULL;
	}

	if (video_ctx->wb_bo) {
		PSB_DEBUG_GENERAL("TOPAZ: unref write back bo\n");
		ttm_bo_kunmap(&video_ctx->wb_bo_kmap);
		ttm_bo_unreserve(video_ctx->wb_bo);
		ttm_bo_unref(&video_ctx->wb_bo);
		video_ctx->wb_bo = NULL;
	}

	video_ctx->fw_data_dma_size = 0;
	video_ctx->fw_data_dma_offset = 0;
	video_ctx->status = 0;
	video_ctx->codec = 0;

	topaz_priv->frame_count = 0;

	return 0;
}

int tng_topaz_dequeue_send(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct tng_topaz_cmd_queue *topaz_cmd = NULL;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

	/* Avoid race condition with queue buffer when topaz_busy = 1 */
	mutex_lock(&topaz_priv->topaz_mutex);
	if (list_empty(&topaz_priv->topaz_queue)) {
		topaz_priv->topaz_busy = 0;
		PSB_DEBUG_GENERAL("TOPAZ: empty command queue, " \
			"set topaz_busy = 0, directly return\n");
		mutex_unlock(&topaz_priv->topaz_mutex);
		return ret;
	}
	mutex_unlock(&topaz_priv->topaz_mutex);

	topaz_priv->topaz_busy = 1;
	topaz_cmd = list_first_entry(&topaz_priv->topaz_queue,
			struct tng_topaz_cmd_queue, head);

	topaz_priv->saved_queue->file_priv = topaz_cmd->file_priv;
	topaz_priv->saved_queue->cmd_size = topaz_cmd->cmd_size;
	topaz_priv->saved_queue->sequence = topaz_cmd->sequence;
	topaz_priv->saved_queue->head = topaz_cmd->head;

	memcpy(topaz_priv->saved_cmd, topaz_cmd->cmd, topaz_cmd->cmd_size);

	list_del(&topaz_cmd->head);
	kfree(topaz_cmd->cmd);
	kfree(topaz_cmd);

	PSB_DEBUG_GENERAL("TOPAZ: dequeue command of sequence %08x " \
			"and send it to topaz\n", \
			topaz_priv->saved_queue->sequence);

	ret = tng_topaz_send(dev,
		topaz_priv->saved_queue->file_priv,
		topaz_priv->saved_cmd,
		topaz_priv->saved_queue->cmd_size,
		topaz_priv->saved_queue->sequence);
	if (ret) {
		DRM_ERROR("TOPAZ: tng_topaz_send failed.\n");
		ret = -EINVAL;
	}

	PSB_DEBUG_GENERAL("TOPAZ: dequeue command of sequence %08x " \
			"finished\n", topaz_priv->saved_queue->sequence);

	return ret;
}

int32_t tng_check_topaz_idle(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t reg_val;

	if (dev_priv->topaz_ctx == NULL) {
		PSB_DEBUG_GENERAL("TOPAZ: topaz context is null\n");
		return 0;
	}

	/*HW is stuck. Need to power off TopazSC to reset HW*/
	if (1 == topaz_priv->topaz_needs_reset) {
		PSB_DEBUG_GENERAL("TOPAZ: Topaz needs reset\n");
		return 0;
	}

	if (topaz_priv->topaz_busy) {
		PSB_DEBUG_GENERAL("TOPAZ: can't save, topaz_busy = %d\n", \
				   topaz_priv->topaz_busy);
		return -EBUSY;
	}

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE,
		&reg_val);
	reg_val &= MASK_TOPAZHP_TOP_CR_CMD_FIFO_SPACE;
	if (reg_val != 32) {
		PSB_DEBUG_GENERAL("TOPAZ: HW is busy. Free words in command" \
				"FIFO is %d.\n",
				reg_val);
		return -EBUSY;
	}

	return 0; /* we think it is idle */
}


int32_t tng_video_get_core_num(struct drm_device *dev, uint64_t user_pointer)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

	ret = copy_to_user((void __user *)((unsigned long)user_pointer),
			   &topaz_priv->topaz_num_pipes,
			   sizeof(topaz_priv->topaz_num_pipes));

	if (ret)
		return -EFAULT;

	return ret;

}

int32_t tng_video_frameskip(struct drm_device *dev, uint64_t user_pointer)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;
	int32_t ret = 0;

	ret = copy_to_user((void __user *)((unsigned long)user_pointer),
		&topaz_priv->frame_skip, sizeof(topaz_priv->frame_skip));

	if (ret)
		return -EFAULT;

	return ret;
}

static void tng_topaz_flush_cmd_queue(struct tng_topaz_private *topaz_priv)
{
	struct tng_topaz_cmd_queue *entry, *next;

	/* remind to reset topaz */
	topaz_priv->topaz_needs_reset = 1;
	topaz_priv->topaz_busy = 0;

	if (list_empty(&topaz_priv->topaz_queue))
		return;

	/* flush all command in queue */
	list_for_each_entry_safe(entry, next,
				 &topaz_priv->topaz_queue,
				 head) {
		list_del(&entry->head);
		kfree(entry->cmd);
		kfree(entry);
	}

	return;
}

void tng_topaz_handle_timeout(struct ttm_fence_device *fdev)
{
	struct drm_psb_private *dev_priv =
		container_of(fdev, struct drm_psb_private, fdev);
	/*struct drm_device *dev =
		container_of((void *)dev_priv,
		struct drm_device, dev_private);*/
	struct tng_topaz_private *topaz_priv = dev_priv->topaz_private;

	/* if (IS_MRST(dev))
		return  lnc_topaz_handle_timeout(fdev);
	*/
	DRM_ERROR("TOPAZ: current codec is %s\n",
			codec_to_string(topaz_priv->cur_codec));
	tng_topaz_flush_cmd_queue(topaz_priv);

	/* Power down TopazSC to reset HW*/
	/* schedule_delayed_work(&topaz_priv->topaz_suspend_wq, 0); */
}

void tng_topaz_enableirq(struct drm_device *dev)
{
#ifdef TOPAZHP_IRQ_ENABLED
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t crImgTopazIntenab;

	PSB_DEBUG_GENERAL("TOPAZ: Enable TOPAZHP IRQ\n");

	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB,
		&crImgTopazIntenab);

	crImgTopazIntenab |= (MASK_TOPAZHP_TOP_CR_MTX_INTEN_MTX |
		MASK_TOPAZHP_TOP_CR_HOST_TOPAZHP_MAS_INTEN);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB,
		crImgTopazIntenab);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_MTX_INT_ENAB,
		MASK_MTX_INT_ENAB);
#else
	return 0;
#endif
}

void tng_topaz_disableirq(struct drm_device *dev)
{
	uint32_t crImgTopazIntenab;
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_GENERAL("TOPAZ: Disable TOPAZHP IRQ\n");
	MULTICORE_READ32(TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB,
		&crImgTopazIntenab);

	crImgTopazIntenab &= ~(MASK_TOPAZHP_TOP_CR_MTX_INTEN_MTX);

	MULTICORE_WRITE32(TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB,
		crImgTopazIntenab);
}
