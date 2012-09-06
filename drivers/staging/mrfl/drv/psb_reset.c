/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
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
 * Authors: Thomas Hellstrom <thomas-at-tungstengraphics-dot-com>
 **************************************************************************/

#include <drm/drmP.h>
#include "psb_drv.h"
#include "psb_reg.h"
#include "psb_intel_reg.h"
#include "psb_msvdx.h"
#include "lnc_topaz.h"
#include "pnw_topaz.h"
#include <linux/spinlock.h>

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
	struct drm_psb_private *dev_priv = (struct drm_psb_private *)data;
	int msvdx_lockup;
	int msvdx_idle;
	unsigned long irq_flags;

	psb_msvdx_lockup(dev_priv, &msvdx_lockup, &msvdx_idle);

	if (msvdx_lockup) {
		spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
		dev_priv->timer_available = 0;
		spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);
		if (msvdx_lockup)
			schedule_work(&dev_priv->msvdx_watchdog_wq);
	}
	if (!msvdx_idle)
		psb_schedule_watchdog(dev_priv);
}

void psb_msvdx_flush_cmd_queue(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_msvdx_cmd_queue *msvdx_cmd;
	struct list_head *list, *next;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/*Flush the msvdx cmd queue and signal all fences in the queue */
	list_for_each_safe(list, next, &msvdx_priv->msvdx_queue) {
		msvdx_cmd = list_entry(list, struct psb_msvdx_cmd_queue, head);
		PSB_DEBUG_GENERAL("MSVDXQUE: flushing sequence:0x%08x\n",
				  msvdx_cmd->sequence);
		msvdx_priv->msvdx_current_sequence = msvdx_cmd->sequence;
		psb_fence_error(dev, PSB_ENGINE_VIDEO,
				msvdx_priv->msvdx_current_sequence,
				_PSB_FENCE_TYPE_EXE, DRM_CMD_HANG);
		list_del(list);
		kfree(msvdx_cmd->cmd);
		kfree(msvdx_cmd);
	}
}

static void psb_msvdx_reset_wq(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
	    container_of(work, struct drm_psb_private, msvdx_watchdog_wq);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	struct psb_scheduler *scheduler = &dev_priv->scheduler;
	unsigned long irq_flags;

	mutex_lock(&msvdx_priv->msvdx_mutex);
	msvdx_priv->msvdx_needs_reset = 1;
	msvdx_priv->msvdx_current_sequence++;
	PSB_DEBUG_GENERAL
	    ("MSVDXFENCE: incremented msvdx_current_sequence to :%d\n",
	     msvdx_priv->msvdx_current_sequence);

	psb_fence_error(scheduler->dev, PSB_ENGINE_VIDEO,
			msvdx_priv->msvdx_current_sequence,
			_PSB_FENCE_TYPE_EXE, DRM_CMD_HANG);

	spin_lock_irqsave(&dev_priv->watchdog_lock, irq_flags);
	dev_priv->timer_available = 1;
	spin_unlock_irqrestore(&dev_priv->watchdog_lock, irq_flags);

	spin_lock_irqsave(&msvdx_priv->msvdx_lock, irq_flags);
	psb_msvdx_flush_cmd_queue(scheduler->dev);
	spin_unlock_irqrestore(&msvdx_priv->msvdx_lock, irq_flags);

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
	wt->data = (unsigned long)dev_priv;
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
	(void)del_timer_sync(&dev_priv->watchdog_timer);
}

static void psb_lid_timer_func(unsigned long data)
{
	struct drm_psb_private *dev_priv = (struct drm_psb_private *)data;
	struct drm_device *dev = (struct drm_device *)dev_priv->dev;
	struct timer_list *lid_timer = &dev_priv->lid_timer;
	unsigned long irq_flags;
	u32 *lid_state = dev_priv->lid_state;
	u32 pp_status;

	if (*lid_state == dev_priv->lid_last_state)
		goto lid_timer_schedule;

	if ((*lid_state) & 0x01) {
		/*lid state is open */
		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) | POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while ((pp_status & PP_ON) == 0);

		/*FIXME: should be backlight level before */
		psb_intel_lvds_set_brightness(dev, 100);
	} else {
		psb_intel_lvds_set_brightness(dev, 0);

		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) & ~POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while ((pp_status & PP_ON) == 0);
	}
	/* printk(KERN_INFO"%s: lid: closed\n", __FUNCTION__); */

	dev_priv->lid_last_state = *lid_state;

 lid_timer_schedule:
	spin_lock_irqsave(&dev_priv->lid_lock, irq_flags);
	if (!timer_pending(lid_timer)) {
		lid_timer->expires = jiffies + PSB_LID_DELAY;
		add_timer(lid_timer);
	}
	spin_unlock_irqrestore(&dev_priv->lid_lock, irq_flags);
}

void psb_lid_timer_init(struct drm_psb_private *dev_priv)
{
	struct timer_list *lid_timer = &dev_priv->lid_timer;
	unsigned long irq_flags;

	spin_lock_init(&dev_priv->lid_lock);
	spin_lock_irqsave(&dev_priv->lid_lock, irq_flags);

	init_timer(lid_timer);

	lid_timer->data = (unsigned long)dev_priv;
	lid_timer->function = psb_lid_timer_func;
	lid_timer->expires = jiffies + PSB_LID_DELAY;

	add_timer(lid_timer);
	spin_unlock_irqrestore(&dev_priv->lid_lock, irq_flags);
}

void psb_lid_timer_takedown(struct drm_psb_private *dev_priv)
{
	del_timer_sync(&dev_priv->lid_timer);
}
