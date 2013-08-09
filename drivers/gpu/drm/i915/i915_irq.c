/* i915_irq.c -- IRQ support for the I915 -*- linux-c -*-
 */
/*
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/sysrq.h>
#include <linux/slab.h>
#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include "intel_ringbuffer.h"

/* Added for HDMI Audio */
#include "hdmi_audio_if.h"

/* For display hotplug interrupt */
static void
ironlake_enable_display_irq(drm_i915_private_t *dev_priv, u32 mask)
{
	if ((dev_priv->irq_mask & mask) != 0) {
		dev_priv->irq_mask &= ~mask;
		I915_WRITE(DEIMR, dev_priv->irq_mask);
		POSTING_READ(DEIMR);
	}
}

static inline void
ironlake_disable_display_irq(drm_i915_private_t *dev_priv, u32 mask)
{
	if ((dev_priv->irq_mask & mask) != mask) {
		dev_priv->irq_mask |= mask;
		I915_WRITE(DEIMR, dev_priv->irq_mask);
		POSTING_READ(DEIMR);
	}
}

void
i915_enable_pipestat(drm_i915_private_t *dev_priv, int pipe, u32 mask)
{
	if ((dev_priv->pipestat[pipe] & mask) != mask) {
		u32 reg = PIPESTAT(pipe);

		dev_priv->pipestat[pipe] |= mask;
		/* Enable the interrupt, clear any pending status */
		I915_WRITE(reg, dev_priv->pipestat[pipe] | (mask >> 16));
		POSTING_READ(reg);
	}
}

void
i915_disable_pipestat(drm_i915_private_t *dev_priv, int pipe, u32 mask)
{
	if ((dev_priv->pipestat[pipe] & mask) != 0) {
		u32 reg = PIPESTAT(pipe);

		dev_priv->pipestat[pipe] &= ~mask;
		I915_WRITE(reg, dev_priv->pipestat[pipe]);
		POSTING_READ(reg);
	}
}

/* Added for HDMI AUDIO */
void
i915_enable_lpe_pipestat(drm_i915_private_t *dev_priv, int pipe)
{
	u32 mask;
	mask = dev_priv->hdmi_audio_interrupt_mask;
	mask |= I915_HDMI_AUDIO_UNDERRUN | I915_HDMI_AUDIO_BUFFER_DONE;
	/* Enable the interrupt, clear any pending status */
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, mask);
	POSTING_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
}

void
i915_disable_lpe_pipestat(drm_i915_private_t *dev_priv, int pipe)
{
	u32 mask;
	mask = dev_priv->hdmi_audio_interrupt_mask;
	mask |= I915_HDMI_AUDIO_UNDERRUN | I915_HDMI_AUDIO_BUFFER_DONE;
	/* Disable the interrupt, clear any pending status */
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, mask);
	POSTING_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
}

/**
 * intel_enable_asle - enable ASLE interrupt for OpRegion
 */
void intel_enable_asle(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long irqflags;

	/* FIXME: opregion/asle for VLV */
	if (IS_VALLEYVIEW(dev))
		return;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);

	if (HAS_PCH_SPLIT(dev))
		ironlake_enable_display_irq(dev_priv, DE_GSE);
	else {
		i915_enable_pipestat(dev_priv, 1,
				     PIPE_LEGACY_BLC_EVENT_ENABLE);
		if (INTEL_INFO(dev)->gen >= 4)
			i915_enable_pipestat(dev_priv, 0,
					     PIPE_LEGACY_BLC_EVENT_ENABLE);
	}

	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

/**
 * i915_pipe_enabled - check if a pipe is enabled
 * @dev: DRM device
 * @pipe: pipe to check
 *
 * Reading certain registers when the pipe is disabled can hang the chip.
 * Use this routine to make sure the PLL is running and the pipe is active
 * before reading such registers if unsure.
 */
static int
i915_pipe_enabled(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	return I915_READ(PIPECONF(pipe)) & PIPECONF_ENABLE;
}

/* Called from drm generic code, passed a 'crtc', which
 * we use as a pipe index
 */
static u32 i915_get_vblank_counter(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long high_frame;
	unsigned long low_frame;
	u32 high1, high2, low;

	if (!i915_pipe_enabled(dev, pipe)) {
		DRM_DEBUG_DRIVER("trying to get vblank count for disabled "
				"pipe %c\n", pipe_name(pipe));
		return 0;
	}

	high_frame = PIPEFRAME(pipe);
	low_frame = PIPEFRAMEPIXEL(pipe);

	/*
	 * High & low register fields aren't synchronized, so make sure
	 * we get a low value that's stable across two reads of the high
	 * register.
	 */
	do {
		high1 = I915_READ(high_frame) & PIPE_FRAME_HIGH_MASK;
		low   = I915_READ(low_frame)  & PIPE_FRAME_LOW_MASK;
		high2 = I915_READ(high_frame) & PIPE_FRAME_HIGH_MASK;
	} while (high1 != high2);

	high1 >>= PIPE_FRAME_HIGH_SHIFT;
	low >>= PIPE_FRAME_LOW_SHIFT;
	return (high1 << 8) | low;
}

static u32 gm45_get_vblank_counter(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int reg = PIPE_FRMCOUNT_GM45(pipe);

	if (!i915_pipe_enabled(dev, pipe)) {
		DRM_DEBUG_DRIVER("trying to get vblank count for disabled "
				 "pipe %c\n", pipe_name(pipe));
		return 0;
	}

	return I915_READ(reg);
}

static int i915_get_crtc_scanoutpos(struct drm_device *dev, int pipe,
			     int *vpos, int *hpos)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 vbl = 0, position = 0;
	int vbl_start, vbl_end, htotal, vtotal;
	bool in_vbl = true;
	int ret = 0;

	if (!i915_pipe_enabled(dev, pipe)) {
		DRM_DEBUG_DRIVER("trying to get scanoutpos for disabled "
				 "pipe %c\n", pipe_name(pipe));
		return 0;
	}

	/* Get vtotal. */
	vtotal = 1 + ((I915_READ(VTOTAL(pipe)) >> 16) & 0x1fff);

	if (INTEL_INFO(dev)->gen >= 4) {
		/* No obvious pixelcount register. Only query vertical
		 * scanout position from Display scan line register.
		 */
		position = I915_READ(PIPEDSL(pipe));

		/* Decode into vertical scanout position. Don't have
		 * horizontal scanout position.
		 */
		*vpos = position & 0x1fff;
		*hpos = 0;
	} else {
		/* Have access to pixelcount since start of frame.
		 * We can split this into vertical and horizontal
		 * scanout position.
		 */
		position = (I915_READ(PIPEFRAMEPIXEL(pipe)) & PIPE_PIXEL_MASK) >> PIPE_PIXEL_SHIFT;

		htotal = 1 + ((I915_READ(HTOTAL(pipe)) >> 16) & 0x1fff);
		*vpos = position / htotal;
		*hpos = position - (*vpos * htotal);
	}

	/* Query vblank area. */
	vbl = I915_READ(VBLANK(pipe));

	/* Test position against vblank region. */
	vbl_start = vbl & 0x1fff;
	vbl_end = (vbl >> 16) & 0x1fff;

	if ((*vpos < vbl_start) || (*vpos > vbl_end))
		in_vbl = false;

	/* Inside "upper part" of vblank area? Apply corrective offset: */
	if (in_vbl && (*vpos >= vbl_start))
		*vpos = *vpos - vtotal;

	/* Readouts valid? */
	if (vbl > 0)
		ret |= DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_ACCURATE;

	/* In vblank? */
	if (in_vbl)
		ret |= DRM_SCANOUTPOS_INVBL;

	return ret;
}

static int i915_get_vblank_timestamp(struct drm_device *dev, int pipe,
			      int *max_error,
			      struct timeval *vblank_time,
			      unsigned flags)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	if (pipe < 0 || pipe >= dev_priv->num_pipe) {
		DRM_ERROR("Invalid crtc %d\n", pipe);
		return -EINVAL;
	}

	/* Get drm_crtc to timestamp: */
	crtc = intel_get_crtc_for_pipe(dev, pipe);
	if (crtc == NULL) {
		DRM_ERROR("Invalid crtc %d\n", pipe);
		return -EINVAL;
	}

	if (!crtc->enabled) {
		DRM_DEBUG_KMS("crtc %d is disabled\n", pipe);
		return -EBUSY;
	}

	/* Helper routine in DRM core does all the work: */
	return drm_calc_vbltimestamp_from_scanoutpos(dev, pipe, max_error,
						     vblank_time, flags,
						     crtc);
}

/*
 * Handle hotplug events outside the interrupt handler proper.
 */
static void i915_hotplug_work_func(struct work_struct *work)
{
	drm_i915_private_t *dev_priv = container_of(work, drm_i915_private_t,
						    hotplug_work);
	struct drm_device *dev = dev_priv->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder;
	char *envp[] = {"hdcp_hpd", NULL};

	/* Should not be here during suspend state */
	if (i915_is_device_suspended(dev)) {
		DRM_ERROR("FIXME: No hotplugs during suspend\n");
		return;
	}

	mutex_lock(&mode_config->mutex);
	DRM_DEBUG_KMS("running encoder hotplug functions\n");

	list_for_each_entry(encoder, &mode_config->encoder_list, base.head)
		if (encoder->hot_plug)
			encoder->hot_plug(encoder);

	mutex_unlock(&mode_config->mutex);

	/* Just fire off a uevent and let userspace tell us what to do */
	drm_helper_hpd_irq_event(dev);

	/* HDCPD needs a uevent, every time when there is a hotplug */
	kobject_uevent_env(&dev->primary->kdev.kobj, KOBJ_CHANGE, envp);
}
/* defined intel_pm.c */
extern spinlock_t mchdev_lock;

static void ironlake_handle_rps_change(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 busy_up, busy_down, max_avg, min_avg;
	u8 new_delay;
	unsigned long flags;

	spin_lock_irqsave(&mchdev_lock, flags);

	I915_WRITE16(MEMINTRSTS, I915_READ(MEMINTRSTS));

	new_delay = dev_priv->cur_delay;

	I915_WRITE16(MEMINTRSTS, MEMINT_EVAL_CHG);
	busy_up = I915_READ(RCPREVBSYTUPAVG);
	busy_down = I915_READ(RCPREVBSYTDNAVG);
	max_avg = I915_READ(RCBMAXAVG);
	min_avg = I915_READ(RCBMINAVG);

	/* Handle RCS change request from hw */
	if (busy_up > max_avg) {
		if (dev_priv->cur_delay != dev_priv->max_delay)
			new_delay = dev_priv->cur_delay - 1;
		if (new_delay < dev_priv->max_delay)
			new_delay = dev_priv->max_delay;
	} else if (busy_down < min_avg) {
		if (dev_priv->cur_delay != dev_priv->min_delay)
			new_delay = dev_priv->cur_delay + 1;
		if (new_delay > dev_priv->min_delay)
			new_delay = dev_priv->min_delay;
	}

	if (ironlake_set_drps(dev, new_delay))
		dev_priv->cur_delay = new_delay;

	spin_unlock_irqrestore(&mchdev_lock, flags);

	return;
}

static void notify_ring(struct drm_device *dev,
			struct intel_ring_buffer *ring)
{
	if (ring->obj == NULL)
		return;

	trace_i915_gem_request_complete(ring, ring->get_seqno(ring, false));

	wake_up_all(&ring->irq_queue);
}

/**
 * vlv_calc_delay_from_C0_counters - Increase/Decrease freq based on GPU
 * busy-ness calculated from C0 counters of render & media power wells
 * @dev_priv: DRM device private
 *
 * UP-Threshold will be calculated on every interrupt where as Down threshold
 * will be calculated once in 5 interrupts. If either render or media power
 * well is in C0 for 90% or above of EI period, then increace freq by one step
 * If both render and media are in C0 for less than 70% of 5 continuous EI
 * periods, then decrease the freq by one step
 */
static u32 vlv_calc_delay_from_C0_counters(struct drm_i915_private *dev_priv)
{
	u32 cz_ts = 0;
	u32 render_count = 0, media_count = 0;
	u32 elapsed_render = 0, elapsed_media = 0;
	u32 elapsed_time = 0;
	u32 residency_C0_up = 0, residency_C0_down = 0;
	u8 new_delay;

	dev_priv->rps.ei_interrupt_count++;

	/* read the CZ clock time stamp from P-unint &
	 * render/media C0 counters from MMIO reg
	 */
	intel_punit_read32(dev_priv, PUNIT_REG_CZ_TIMESTAMP, &cz_ts);

	render_count = I915_READ(VLV_RENDER_C0_COUNT_REG);
	media_count = I915_READ(VLV_MEDIA_C0_COUNT_REG);

	/* If this is the very first call, save the counters and
	 * apply the current delay itself
	 */
	if (0 == dev_priv->rps.cz_ts_up_EI) {

		dev_priv->rps.cz_ts_up_EI = dev_priv->rps.cz_ts_down_EI = cz_ts;
		dev_priv->rps.render_up_EI_C0 = dev_priv->rps.render_down_EI_C0
						= render_count;
		dev_priv->rps.media_up_EI_C0 = dev_priv->rps.media_down_EI_C0
						= media_count;

		return dev_priv->rps.cur_delay;
	}

	/* Calculate Elapsed time and render/media counters. Since all
	* variables used are of type unsigned, standard C compiler will
	* will take care of roll-off scenario with subtractions
	*/
	elapsed_time = cz_ts - dev_priv->rps.cz_ts_up_EI;
	dev_priv->rps.cz_ts_up_EI = cz_ts;

	elapsed_render = render_count - dev_priv->rps.render_up_EI_C0;
	dev_priv->rps.render_up_EI_C0 = render_count;

	elapsed_media = media_count - dev_priv->rps.media_up_EI_C0;
	dev_priv->rps.media_up_EI_C0 = media_count;

	/* Convert all the counters into common unit of milli sec */
	elapsed_time /= VLV_CZ_CLOCK_TO_MILLI_SEC;
	elapsed_render /= (dev_priv->rps.cz_freq / 1000);
	elapsed_media /= (dev_priv->rps.cz_freq / 1000);

	/* Calculate overall C0 residency percentage only
	 * if elapsed time is non zero
	 */
	if (elapsed_time) {
		residency_C0_up = ((max(elapsed_render, elapsed_media)
						* 100) / elapsed_time);
	}

	/* To down throttle, C0 residency should be less down threshold
	 * for continous EI intervals. So calculate down EI counters
	 * once in VLV_INT_COUNT_FOR_DOWN_EI
	 */
	if (VLV_INT_COUNT_FOR_DOWN_EI == dev_priv->rps.ei_interrupt_count) {

		dev_priv->rps.ei_interrupt_count = 0;

		elapsed_time = cz_ts - dev_priv->rps.cz_ts_down_EI;
		dev_priv->rps.cz_ts_down_EI = cz_ts;

		elapsed_render = render_count - dev_priv->rps.render_down_EI_C0;
		dev_priv->rps.render_down_EI_C0 = render_count;

		elapsed_media = media_count - dev_priv->rps.media_down_EI_C0;
		dev_priv->rps.media_down_EI_C0 = media_count;

		/* Convert all the counters into common unit of milli sec */
		elapsed_time /= 100000;
		elapsed_render /= (dev_priv->rps.cz_freq / 1000);
		elapsed_media /= (dev_priv->rps.cz_freq / 1000);

		/* Calculate overall C0 residency percentage only
		 * if elapsed time is non zero
		 */
		if (elapsed_time) {
			residency_C0_down =
				((max(elapsed_render, elapsed_media) * 100)
							/ elapsed_time);
		}

	}

	new_delay = dev_priv->rps.cur_delay;

	/* C0 residency is greater than UP threshold. Increase Frequency */
	if (residency_C0_up >= VLV_RP_UP_EI_THRESHOLD) {

		if (dev_priv->rps.cur_delay < dev_priv->rps.max_delay)
			new_delay = dev_priv->rps.cur_delay + 1;

		atomic_inc(&dev_priv->turbodebug.up_threshold);

	} else if (!dev_priv->rps.ei_interrupt_count &&
			(residency_C0_down < VLV_RP_DOWN_EI_THRESHOLD)) {
		/* This means, C0 residency is less than down threshold over
		 * a period of VLV_INT_COUNT_FOR_DOWN_EI. So, reduce the freq
		 */
		if (dev_priv->rps.cur_delay > dev_priv->rps.min_delay)
			new_delay = dev_priv->rps.cur_delay - 1;

		atomic_inc(&dev_priv->turbodebug.down_threshold);
	}

	return new_delay;
}

static void gen6_pm_rps_work(struct work_struct *work)
{
	drm_i915_private_t *dev_priv = container_of(work, drm_i915_private_t,
						    rps.work);
	u32 pm_iir, pm_imr;
	u8 new_delay;

	spin_lock_irq(&dev_priv->rps.lock);
	pm_iir = dev_priv->rps.pm_iir;
	dev_priv->rps.pm_iir = 0;
	pm_imr = I915_READ(GEN6_PMIMR);
	I915_WRITE(GEN6_PMIMR, 0);
	spin_unlock_irq(&dev_priv->rps.lock);

	if ((pm_iir & (GEN6_PM_DEFERRED_EVENTS | VLV_PM_DEFERRED_EVENTS)) == 0)
		return;

	mutex_lock(&dev_priv->dev->struct_mutex);

	/* Make sure we have current freq updated properly. Doing this
	 * here becuase, on VLV, P-Unit doesnt garauntee that last requested
	 * freq by driver is actually the current running frequency
	 */
	if (IS_VALLEYVIEW(dev_priv->dev))
		valleyview_update_cur_delay(dev_priv->dev);

	if (pm_iir & GEN6_PM_RP_UP_THRESHOLD) {
		if (dev_priv->rps.cur_delay >= dev_priv->rps.max_delay) {
			I915_WRITE(GEN6_PMINTRMSK,
					I915_READ(GEN6_PMINTRMSK) | 1 << 5);
			dev_priv->rps.rp_up_masked = 1;
			new_delay = dev_priv->rps.cur_delay;
		} else {
			new_delay = dev_priv->rps.cur_delay + 1;
		}
		if (dev_priv->rps.rp_down_masked) {
			I915_WRITE(GEN6_PMINTRMSK,
					I915_READ(GEN6_PMINTRMSK) & ~(1 << 4));
			dev_priv->rps.rp_down_masked = 0;
		}
		atomic_inc(&dev_priv->turbodebug.up_threshold);
	} else if (pm_iir & GEN6_PM_RP_UP_EI_EXPIRED)
		new_delay = vlv_calc_delay_from_C0_counters(dev_priv);
	else {
		if (dev_priv->rps.cur_delay <= dev_priv->rps.min_delay) {
			I915_WRITE(GEN6_PMINTRMSK,
					I915_READ(GEN6_PMINTRMSK) | 1 << 4);
			dev_priv->rps.rp_down_masked = 1;
			new_delay = dev_priv->rps.cur_delay;
		} else {
			new_delay = dev_priv->rps.cur_delay - 1;
		}
		if (dev_priv->rps.rp_up_masked) {
			I915_WRITE(GEN6_PMINTRMSK,
					I915_READ(GEN6_PMINTRMSK) & ~(1 << 5));
			dev_priv->rps.rp_up_masked = 0;
		}
		atomic_inc(&dev_priv->turbodebug.down_threshold);
	}

	if (IS_VALLEYVIEW(dev_priv->dev)) {
		valleyview_set_rps(dev_priv->dev, new_delay);

		if (new_delay > dev_priv->rps.rpe_delay) {
			/* Freq is more than Rpe. Trigger the timer to take
			 * care of the cases where RC6 is entered at this freq
			 * Cancel the previous one first. No need to cancel sync
			 * as struct_mutex will provide required synchronization
			 */
			cancel_delayed_work(&dev_priv->rps.rps_timer_work);

			queue_delayed_work(dev_priv->wq,
					&dev_priv->rps.rps_timer_work,
					msecs_to_jiffies(VLV_RPS_TIMER_VALUE));
		}
	}
	else
		gen6_set_rps(dev_priv->dev, new_delay);

	mutex_unlock(&dev_priv->dev->struct_mutex);
}


/**
 * ivybridge_parity_work - Workqueue called when a parity error interrupt
 * occurred.
 * @work: workqueue struct
 *
 * Doesn't actually do anything except notify userspace. As a consequence of
 * this event, userspace should try to remap the bad rows since statistically
 * it is likely the same row is more likely to go bad again.
 */
static void ivybridge_parity_work(struct work_struct *work)
{
	drm_i915_private_t *dev_priv = container_of(work, drm_i915_private_t,
						    parity_error_work);
	u32 error_status, row, bank, subbank;
	char *parity_event[5];
	uint32_t misccpctl;
	unsigned long flags;

	/* We must turn off DOP level clock gating to access the L3 registers.
	 * In order to prevent a get/put style interface, acquire struct mutex
	 * any time we access those registers.
	 */
	mutex_lock(&dev_priv->dev->struct_mutex);

	misccpctl = I915_READ(GEN7_MISCCPCTL);
	I915_WRITE(GEN7_MISCCPCTL, misccpctl & ~GEN7_DOP_CLOCK_GATE_ENABLE);
	POSTING_READ(GEN7_MISCCPCTL);

	error_status = I915_READ(GEN7_L3CDERRST1);
	row = GEN7_PARITY_ERROR_ROW(error_status);
	bank = GEN7_PARITY_ERROR_BANK(error_status);
	subbank = GEN7_PARITY_ERROR_SUBBANK(error_status);

	I915_WRITE(GEN7_L3CDERRST1, GEN7_PARITY_ERROR_VALID |
				    GEN7_L3CDERRST1_ENABLE);
	POSTING_READ(GEN7_L3CDERRST1);

	I915_WRITE(GEN7_MISCCPCTL, misccpctl);

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	dev_priv->gt_irq_mask &= ~GT_GEN7_L3_PARITY_ERROR_INTERRUPT;
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	mutex_unlock(&dev_priv->dev->struct_mutex);

	parity_event[0] = "L3_PARITY_ERROR=1";
	parity_event[1] = kasprintf(GFP_KERNEL, "ROW=%d", row);
	parity_event[2] = kasprintf(GFP_KERNEL, "BANK=%d", bank);
	parity_event[3] = kasprintf(GFP_KERNEL, "SUBBANK=%d", subbank);
	parity_event[4] = NULL;

	kobject_uevent_env(&dev_priv->dev->primary->kdev.kobj,
			   KOBJ_CHANGE, parity_event);

	DRM_DEBUG("Parity error: Row = %d, Bank = %d, Sub bank = %d.\n",
		  row, bank, subbank);

	kfree(parity_event[3]);
	kfree(parity_event[2]);
	kfree(parity_event[1]);
}

static void ivybridge_handle_parity_error(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long flags;

	if (!HAS_L3_GPU_CACHE(dev))
		return;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	dev_priv->gt_irq_mask |= GT_GEN7_L3_PARITY_ERROR_INTERRUPT;
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	queue_work(dev_priv->wq, &dev_priv->parity_error_work);
}

static void snb_gt_irq_handler(struct drm_device *dev,
			       struct drm_i915_private *dev_priv,
			       u32 gt_iir)
{
	struct intel_ring_buffer *ring;

	if (gt_iir & (GEN6_RENDER_USER_INTERRUPT |
		      GEN6_RENDER_PIPE_CONTROL_NOTIFY_INTERRUPT))
		notify_ring(dev, &dev_priv->ring[RCS]);
	if (gt_iir & GEN6_BSD_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[VCS]);
	if (gt_iir & GEN6_BLITTER_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[BCS]);

	if (gt_iir & GEN6_RENDER_TIMEOUT_COUNTER_EXPIRED) {
		DRM_DEBUG_TDR("Render timeout counter exceeded\n");

		/* Stop the counter to prevent further interrupts */
		ring = &dev_priv->ring[RCS];
		I915_WRITE(RING_CNTR(ring->mmio_base), RCS_WATCHDOG_DISABLE);

		i915_handle_error(dev, &dev_priv->hangcheck[RCS], 1);
	}

	if (gt_iir & GEN6_BSD_TIMEOUT_COUNTER_EXPIRED) {
		DRM_DEBUG_TDR("Video timeout counter exceeded\n");

		/* Stop the counter to prevent further interrupts */
		ring = &dev_priv->ring[VCS];
		I915_WRITE(RING_CNTR(ring->mmio_base), VCS_WATCHDOG_DISABLE);

		i915_handle_error(dev, &dev_priv->hangcheck[VCS], 1);
	}

	/* Command streamer error interrupts will be dealt with by TDR*/

	if (gt_iir & GT_GEN7_L3_PARITY_ERROR_INTERRUPT)
		ivybridge_handle_parity_error(dev);
}

static void gen6_queue_rps_work(struct drm_i915_private *dev_priv,
				u32 pm_iir)
{
	unsigned long flags;

	/*
	 * IIR bits should never already be set because IMR should
	 * prevent an interrupt from being shown in IIR. The warning
	 * displays a case where we've unsafely cleared
	 * dev_priv->rps.pm_iir. Although missing an interrupt of the same
	 * type is not a problem, it displays a problem in the logic.
	 *
	 * The mask bit in IMR is cleared by dev_priv->rps.work.
	 */

	spin_lock_irqsave(&dev_priv->rps.lock, flags);
	dev_priv->rps.pm_iir |= pm_iir;
	I915_WRITE(GEN6_PMIMR, dev_priv->rps.pm_iir);
	POSTING_READ(GEN6_PMIMR);
	spin_unlock_irqrestore(&dev_priv->rps.lock, flags);

	queue_work(dev_priv->wq, &dev_priv->rps.work);
}

static irqreturn_t valleyview_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 iir, gt_iir, pm_iir;
	irqreturn_t ret = IRQ_NONE;
	unsigned long irqflags;
	int pipe;
	u32 pipe_stats[I915_MAX_PIPES] = {0};
	bool blc_event;
	int lpe_stream;

	atomic_inc(&dev_priv->irq_received);

	while (true) {
		iir = I915_READ(VLV_IIR);
		gt_iir = I915_READ(GTIIR);
		pm_iir = I915_READ(GEN6_PMIIR);
		if (gt_iir == 0 && pm_iir == 0 && iir == 0)
			goto out;

		ret = IRQ_HANDLED;

		snb_gt_irq_handler(dev, dev_priv, gt_iir);

		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/*
			 * Clear the PIPE*STAT regs before the IIR
			 */
			if (pipe_stats[pipe] & 0x8000ffff) {
				if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
					DRM_ERROR("pipe %c underrun\n",
							 pipe_name(pipe));
				I915_WRITE(reg, pipe_stats[pipe]);
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		for_each_pipe(pipe) {
			if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS)
				drm_handle_vblank(dev, pipe);

			if (pipe_stats[pipe] & PLANE_FLIPDONE_INT_STATUS_VLV) {
				intel_prepare_page_flip(dev, pipe);
				intel_finish_page_flip(dev, pipe);
			}
			if (pipe_stats[pipe] & PIPE_DPST_EVENT_STATUS) {
#ifdef CONFIG_DEBUG_FS
				dev_priv->dpst.num_interrupt++;
#endif
				if (dev_priv->dpst_task != NULL)
					send_sig_info(dev_priv->dpst_signal,
						SEND_SIG_FORCED,
							dev_priv->dpst_task);
			}
		}

		if (iir & I915_LPE_PIPE_B_INTERRUPT) {
			lpe_stream = I915_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
			if (lpe_stream & I915_HDMI_AUDIO_UNDERRUN) {
				I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B,
						lpe_stream);
				mid_hdmi_audio_signal_event(dev,
					HAD_EVENT_AUDIO_BUFFER_UNDERRUN);
			}

			lpe_stream = I915_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
			if (lpe_stream & I915_HDMI_AUDIO_BUFFER_DONE) {
				I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B,
						lpe_stream);
				mid_hdmi_audio_signal_event(dev,
					HAD_EVENT_AUDIO_BUFFER_DONE);
			}
		}

		/* Consume port.  Then clear IIR or we'll miss events */
		if (iir & I915_DISPLAY_PORT_INTERRUPT) {
			u32 hotplug_status = I915_READ(PORT_HOTPLUG_STAT);

			DRM_DEBUG_DRIVER("hotplug event received, stat 0x%08x\n",
					 hotplug_status);
			if (hotplug_status & dev_priv->hotplug_supported_mask)
				queue_work(dev_priv->wq,
					   &dev_priv->hotplug_work);

			I915_WRITE(PORT_HOTPLUG_STAT, hotplug_status);
			I915_READ(PORT_HOTPLUG_STAT);
		}
#ifdef CONFIG_DRM_VXD_BYT
		if (iir & VED_BLOCK_INTERRUPT) {
			if (dev_priv->psb_msvdx_interrupt)
				dev_priv->psb_msvdx_interrupt(dev);
		}
#endif
		if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
			blc_event = true;

		if (pm_iir & (GEN6_PM_DEFERRED_EVENTS | VLV_PM_DEFERRED_EVENTS))
			gen6_queue_rps_work(dev_priv, pm_iir);

		I915_WRITE(GTIIR, gt_iir);
		I915_WRITE(GEN6_PMIIR, pm_iir);
		I915_WRITE(VLV_IIR, iir);
	}

out:
	return ret;
}

static void ibx_irq_handler(struct drm_device *dev, u32 pch_iir)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	if (pch_iir & SDE_AUDIO_POWER_MASK)
		DRM_DEBUG_DRIVER("PCH audio power change on port %d\n",
				 (pch_iir & SDE_AUDIO_POWER_MASK) >>
				 SDE_AUDIO_POWER_SHIFT);

	if (pch_iir & SDE_GMBUS)
		DRM_DEBUG_DRIVER("PCH GMBUS interrupt\n");

	if (pch_iir & SDE_AUDIO_HDCP_MASK)
		DRM_DEBUG_DRIVER("PCH HDCP audio interrupt\n");

	if (pch_iir & SDE_AUDIO_TRANS_MASK)
		DRM_DEBUG_DRIVER("PCH transcoder audio interrupt\n");

	if (pch_iir & SDE_POISON)
		DRM_ERROR("PCH poison interrupt\n");

	if (pch_iir & SDE_FDI_MASK)
		for_each_pipe(pipe)
			DRM_DEBUG_DRIVER("  pipe %c FDI IIR: 0x%08x\n",
					 pipe_name(pipe),
					 I915_READ(FDI_RX_IIR(pipe)));

	if (pch_iir & (SDE_TRANSB_CRC_DONE | SDE_TRANSA_CRC_DONE))
		DRM_DEBUG_DRIVER("PCH transcoder CRC done interrupt\n");

	if (pch_iir & (SDE_TRANSB_CRC_ERR | SDE_TRANSA_CRC_ERR))
		DRM_DEBUG_DRIVER("PCH transcoder CRC error interrupt\n");

	if (pch_iir & SDE_TRANSB_FIFO_UNDER)
		DRM_DEBUG_DRIVER("PCH transcoder B underrun interrupt\n");
	if (pch_iir & SDE_TRANSA_FIFO_UNDER)
		DRM_DEBUG_DRIVER("PCH transcoder A underrun interrupt\n");
}

static void cpt_irq_handler(struct drm_device *dev, u32 pch_iir)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	if (pch_iir & SDE_AUDIO_POWER_MASK_CPT)
		DRM_DEBUG_DRIVER("PCH audio power change on port %d\n",
				 (pch_iir & SDE_AUDIO_POWER_MASK_CPT) >>
				 SDE_AUDIO_POWER_SHIFT_CPT);

	if (pch_iir & SDE_AUX_MASK_CPT)
		DRM_DEBUG_DRIVER("AUX channel interrupt\n");

	if (pch_iir & SDE_GMBUS_CPT)
		DRM_DEBUG_DRIVER("PCH GMBUS interrupt\n");

	if (pch_iir & SDE_AUDIO_CP_REQ_CPT)
		DRM_DEBUG_DRIVER("Audio CP request interrupt\n");

	if (pch_iir & SDE_AUDIO_CP_CHG_CPT)
		DRM_DEBUG_DRIVER("Audio CP change interrupt\n");

	if (pch_iir & SDE_FDI_MASK_CPT)
		for_each_pipe(pipe)
			DRM_DEBUG_DRIVER("  pipe %c FDI IIR: 0x%08x\n",
					 pipe_name(pipe),
					 I915_READ(FDI_RX_IIR(pipe)));
}

static irqreturn_t ivybridge_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 de_iir, gt_iir, de_ier, pm_iir;
	irqreturn_t ret = IRQ_NONE;
	int i;

	atomic_inc(&dev_priv->irq_received);

	/* disable master interrupt before clearing iir  */
	de_ier = I915_READ(DEIER);
	I915_WRITE(DEIER, de_ier & ~DE_MASTER_IRQ_CONTROL);

	gt_iir = I915_READ(GTIIR);
	if (gt_iir) {
		snb_gt_irq_handler(dev, dev_priv, gt_iir);
		I915_WRITE(GTIIR, gt_iir);
		ret = IRQ_HANDLED;
	}

	de_iir = I915_READ(DEIIR);
	if (de_iir) {
		if (de_iir & DE_GSE_IVB)
			intel_opregion_gse_intr(dev);

		for (i = 0; i < 3; i++) {
			if (de_iir & (DE_PLANEA_FLIP_DONE_IVB << (5 * i))) {
				intel_prepare_page_flip(dev, i);
				intel_finish_page_flip_plane(dev, i);
			}
			if (de_iir & (DE_PIPEA_VBLANK_IVB << (5 * i)))
				drm_handle_vblank(dev, i);
		}

		/* check event from PCH */
		if (de_iir & DE_PCH_EVENT_IVB) {
			u32 pch_iir = I915_READ(SDEIIR);

			if (pch_iir & SDE_HOTPLUG_MASK_CPT)
				queue_work(dev_priv->wq, &dev_priv->hotplug_work);
			cpt_irq_handler(dev, pch_iir);

			/* clear PCH hotplug event before clear CPU irq */
			I915_WRITE(SDEIIR, pch_iir);
		}

		I915_WRITE(DEIIR, de_iir);
		ret = IRQ_HANDLED;
	}

	pm_iir = I915_READ(GEN6_PMIIR);
	if (pm_iir) {
		if (pm_iir & GEN6_PM_DEFERRED_EVENTS)
			gen6_queue_rps_work(dev_priv, pm_iir);
		I915_WRITE(GEN6_PMIIR, pm_iir);
		ret = IRQ_HANDLED;
	}

	I915_WRITE(DEIER, de_ier);
	POSTING_READ(DEIER);

	return ret;
}

static void ilk_gt_irq_handler(struct drm_device *dev,
			       struct drm_i915_private *dev_priv,
			       u32 gt_iir)
{
	if (gt_iir & (GT_USER_INTERRUPT | GT_PIPE_NOTIFY))
		notify_ring(dev, &dev_priv->ring[RCS]);
	if (gt_iir & GT_BSD_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[VCS]);
}

static irqreturn_t ironlake_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int ret = IRQ_NONE;
	u32 de_iir, gt_iir, de_ier, pch_iir, pm_iir;
	u32 hotplug_mask;

	atomic_inc(&dev_priv->irq_received);

	/* disable master interrupt before clearing iir  */
	de_ier = I915_READ(DEIER);
	I915_WRITE(DEIER, de_ier & ~DE_MASTER_IRQ_CONTROL);
	POSTING_READ(DEIER);

	de_iir = I915_READ(DEIIR);
	gt_iir = I915_READ(GTIIR);
	pch_iir = I915_READ(SDEIIR);
	pm_iir = I915_READ(GEN6_PMIIR);

	if (de_iir == 0 && gt_iir == 0 && pch_iir == 0 &&
	    (!IS_GEN6(dev) || pm_iir == 0))
		goto done;

	if (HAS_PCH_CPT(dev))
		hotplug_mask = SDE_HOTPLUG_MASK_CPT;
	else
		hotplug_mask = SDE_HOTPLUG_MASK;

	ret = IRQ_HANDLED;

	if (IS_GEN5(dev))
		ilk_gt_irq_handler(dev, dev_priv, gt_iir);
	else
		snb_gt_irq_handler(dev, dev_priv, gt_iir);

	if (de_iir & DE_GSE)
		intel_opregion_gse_intr(dev);

	if (de_iir & DE_PLANEA_FLIP_DONE) {
		intel_prepare_page_flip(dev, 0);
		intel_finish_page_flip_plane(dev, 0);
	}

	if (de_iir & DE_PLANEB_FLIP_DONE) {
		intel_prepare_page_flip(dev, 1);
		intel_finish_page_flip_plane(dev, 1);
	}

	if (de_iir & DE_PIPEA_VBLANK)
		drm_handle_vblank(dev, 0);

	if (de_iir & DE_PIPEB_VBLANK)
		drm_handle_vblank(dev, 1);

	/* check event from PCH */
	if (de_iir & DE_PCH_EVENT) {
		if (pch_iir & hotplug_mask)
			queue_work(dev_priv->wq, &dev_priv->hotplug_work);
		if (HAS_PCH_CPT(dev))
			cpt_irq_handler(dev, pch_iir);
		else
			ibx_irq_handler(dev, pch_iir);
	}

	if (IS_GEN5(dev) &&  de_iir & DE_PCU_EVENT)
		ironlake_handle_rps_change(dev);

	if (IS_GEN6(dev) && pm_iir & GEN6_PM_DEFERRED_EVENTS)
		gen6_queue_rps_work(dev_priv, pm_iir);

	/* should clear PCH hotplug event before clear CPU irq */
	I915_WRITE(SDEIIR, pch_iir);
	I915_WRITE(GTIIR, gt_iir);
	I915_WRITE(DEIIR, de_iir);
	I915_WRITE(GEN6_PMIIR, pm_iir);

done:
	I915_WRITE(DEIER, de_ier);
	POSTING_READ(DEIER);

	return ret;
}

/**
 * i915_error_work_func - do process context error handling work
 * @work: work struct
 *
 * Fire an error uevent so userspace can see that a hang or error
 * was detected.
 */
static void i915_error_work_func(struct work_struct *work)
{
	drm_i915_private_t *dev_priv = container_of(work, drm_i915_private_t,
						    error_work);
	struct drm_device *dev = dev_priv->dev;
	char *error_event[] = { "ERROR=1", NULL };
	char *gpu_reset[] = { "GPU RESET=1", NULL };
	char *reset_done_event[] = { "ERROR=0", NULL };
	uint32_t i;

	int pipe;
	struct drm_crtc *crtc;
	struct intel_crtc *intel_crtc;
	struct intel_unpin_work *unpin_work;

	char *reset_event[2];
	reset_event[1] = NULL;

	DRM_DEBUG_TDR("Start recovery work\n");

	/* Set this flag as it should force any waiting processes to release
	* struct->mutex if they are holding it */
	atomic_set(&dev_priv->mm.wedged, 1);

	mutex_lock(&dev->struct_mutex);

	kobject_uevent_env(&dev->primary->kdev.kobj, KOBJ_CHANGE, error_event);

	/* Skip individual ring reset requests if full_reset requested*/
	if (!atomic_read(&dev_priv->full_reset)) {
		/* Check each ring for a pending reset condition */
		for (i = 0; i < I915_NUM_RINGS; i++) {
			if (atomic_read(&dev_priv->hangcheck[i].reset)) {
				DRM_DEBUG_TDR("resetting ring %d\n", i);

				if (i915_handle_hung_ring(dev, i) != 0) {
					DRM_ERROR("ring %d reset failed", i);
					atomic_set(&dev_priv->full_reset, 1);
					break;
				}
			}
		}
	}

	/* Release struct->mutex for the full GPU reset. It will take
	* it itself when it needs it */
	mutex_unlock(&dev->struct_mutex);

	/* Full gpu reset can be requested from caller or may be triggered due
	 * to an indiviudal ring failing to reset */
	if (atomic_read(&dev_priv->full_reset)) {
		/* Full GPU reset requested */
		DRM_DEBUG_TDR("resetting chip\n");
		kobject_uevent_env(&dev->primary->kdev.kobj,
					KOBJ_CHANGE, gpu_reset);

		if (!i915_reset(dev)) {
			kobject_uevent_env(&dev->primary->kdev.kobj,
				 KOBJ_CHANGE, reset_done_event);
			atomic_set(&dev_priv->full_reset, 0);

			for (i = 0; i < I915_NUM_RINGS; i++) {
				/* Clear individual ring reset flags*/
				atomic_set(&dev_priv->hangcheck[i].reset, 0);
			}

			/* Release any pending page flip
			* This is particularly important if ring_stop was set.
			*/
			for_each_pipe(pipe) {
				crtc =
					dev_priv->pipe_to_crtc_mapping[pipe];
				intel_crtc = to_intel_crtc(crtc);
				unpin_work = intel_crtc->unpin_work;

				if (unpin_work
				&& unpin_work->pending_flip_obj) {
					intel_prepare_page_flip(dev, pipe);
					intel_finish_page_flip(dev, pipe);
					DRM_DEBUG_TDR("Clr pg flip\n");
				}
			}
		}
	}

	/* Clear wedged condition and wake up waiters*/
	atomic_set(&dev_priv->mm.wedged, 0);

	kobject_uevent_env(&dev->primary->kdev.kobj,
				KOBJ_CHANGE, reset_done_event);

	/* Wake anyone waiting on error handling completion*/
	wake_up_all(&dev_priv->error_queue);

	DRM_DEBUG_TDR("End recovery work\n\n");
}

static void i915_get_instdone(struct drm_device *dev,
			    uint32_t *instdone,
			    struct intel_ring_buffer *ring)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	memset(instdone, 0,
		I915_MAX_INSTDONE_REG * sizeof(u32));

	switch (INTEL_INFO(dev)->gen) {
	case 2:
	case 3:
		instdone[0] = I915_READ(INSTDONE);
		break;
	case 4:
	case 5:
	case 6:
		instdone[0] = I915_READ(INSTDONE_I965);
		instdone[1] = I915_READ(INSTDONE1);
		break;
	default:
		WARN_ONCE(1, "Unsupported platform, defaulting to gen 7\n");
	case 7:
		instdone[0] =
			I915_READ(RING_INSTDONE(ring->mmio_base));

		if (ring->id == RCS) {
			instdone[1] = I915_READ(GEN7_SC_INSTDONE);
			instdone[2] = I915_READ(GEN7_SAMPLER_INSTDONE);
			instdone[3] = I915_READ(GEN7_ROW_INSTDONE);
		}
		break;
	}
}

#ifdef CONFIG_DEBUG_FS
static struct drm_i915_error_object *
i915_error_object_create(struct drm_i915_private *dev_priv,
			 struct drm_i915_gem_object *src)
{
	struct drm_i915_error_object *dst;
	int page, page_count;
	u32 reloc_offset;

	if (src == NULL || src->pages == NULL)
		return NULL;

	page_count = src->base.size / PAGE_SIZE;

	dst = kmalloc(sizeof(*dst) + page_count * sizeof(u32 *), GFP_ATOMIC);
	if (dst == NULL)
		return NULL;

	reloc_offset = src->gtt_offset;
	for (page = 0; page < page_count; page++) {
		unsigned long flags;
		void *d;

		d = kmalloc(PAGE_SIZE, GFP_ATOMIC);
		if (d == NULL)
			goto unwind;

		local_irq_save(flags);
		if (reloc_offset < dev_priv->mm.gtt_mappable_end &&
		    src->has_global_gtt_mapping) {
			void __iomem *s;

			/* Simply ignore tiling or any overlapping fence.
			 * It's part of the error state, and this hopefully
			 * captures what the GPU read.
			 */

			s = io_mapping_map_atomic_wc(dev_priv->mm.gtt_mapping,
						     reloc_offset);
			memcpy_fromio(d, s, PAGE_SIZE);
			io_mapping_unmap_atomic(s);
		} else {
			void *s;

			drm_clflush_pages(&src->pages[page], 1);

			s = kmap_atomic(src->pages[page]);
			memcpy(d, s, PAGE_SIZE);
			kunmap_atomic(s);

			drm_clflush_pages(&src->pages[page], 1);
		}
		local_irq_restore(flags);

		dst->pages[page] = d;

		reloc_offset += PAGE_SIZE;
	}
	dst->page_count = page_count;
	dst->gtt_offset = src->gtt_offset;

	return dst;

unwind:
	while (page--)
		kfree(dst->pages[page]);
	kfree(dst);
	return NULL;
}

static void
i915_error_object_free(struct drm_i915_error_object *obj)
{
	int page;

	if (obj == NULL)
		return;

	for (page = 0; page < obj->page_count; page++)
		kfree(obj->pages[page]);

	kfree(obj);
}

void
i915_error_state_free(struct kref *error_ref)
{
	struct drm_i915_error_state *error = container_of(error_ref,
							  typeof(*error), ref);
	int i;

	for (i = 0; i < ARRAY_SIZE(error->ring); i++) {
		i915_error_object_free(error->ring[i].batchbuffer);
		i915_error_object_free(error->ring[i].ringbuffer);
		kfree(error->ring[i].requests);
	}

	kfree(error->active_bo);
	kfree(error->overlay);
	kfree(error);
}
static void capture_bo(struct drm_i915_error_buffer *err,
		       struct drm_i915_gem_object *obj)
{
	err->size = obj->base.size;
	err->name = obj->base.name;
	err->rseqno = obj->last_read_seqno;
	err->wseqno = obj->last_write_seqno;
	err->gtt_offset = obj->gtt_offset;
	err->read_domains = obj->base.read_domains;
	err->write_domain = obj->base.write_domain;
	err->fence_reg = obj->fence_reg;
	err->pinned = 0;
	if (obj->pin_count > 0)
		err->pinned = 1;
	if (obj->user_pin_count > 0)
		err->pinned = -1;
	err->tiling = obj->tiling_mode;
	err->dirty = obj->dirty;
	err->purgeable = obj->madv != I915_MADV_WILLNEED;
	err->ring = obj->ring ? obj->ring->id : -1;
	err->cache_level = obj->cache_level;
}

static u32 capture_active_bo(struct drm_i915_error_buffer *err,
			     int count, struct list_head *head)
{
	struct drm_i915_gem_object *obj;
	int i = 0;

	list_for_each_entry(obj, head, mm_list) {
		capture_bo(err++, obj);
		if (++i == count)
			break;
	}

	return i;
}

static u32 capture_pinned_bo(struct drm_i915_error_buffer *err,
			     int count, struct list_head *head)
{
	struct drm_i915_gem_object *obj;
	int i = 0;

	list_for_each_entry(obj, head, gtt_list) {
		if (obj->pin_count == 0)
			continue;

		capture_bo(err++, obj);
		if (++i == count)
			break;
	}

	return i;
}

static void i915_gem_record_fences(struct drm_device *dev,
				   struct drm_i915_error_state *error)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;

	/* Fences */
	switch (INTEL_INFO(dev)->gen) {
	case 7:
	case 6:
		for (i = 0; i < 16; i++)
			error->fence[i] = I915_READ64(FENCE_REG_SANDYBRIDGE_0 + (i * 8));
		break;
	case 5:
	case 4:
		for (i = 0; i < 16; i++)
			error->fence[i] = I915_READ64(FENCE_REG_965_0 + (i * 8));
		break;
	case 3:
		if (IS_I945G(dev) || IS_I945GM(dev) || IS_G33(dev))
			for (i = 0; i < 8; i++)
				error->fence[i+8] = I915_READ(FENCE_REG_945_8 + (i * 4));
	case 2:
		for (i = 0; i < 8; i++)
			error->fence[i] = I915_READ(FENCE_REG_830_0 + (i * 4));
		break;

	}
}

static struct drm_i915_error_object *
i915_error_first_batchbuffer(struct drm_i915_private *dev_priv,
			     struct intel_ring_buffer *ring)
{
	struct drm_i915_gem_object *obj;
	u32 seqno;

	if (!ring->get_seqno)
		return NULL;

	seqno = ring->get_seqno(ring, false);
	list_for_each_entry(obj, &dev_priv->mm.active_list, mm_list) {
		if (obj->ring != ring)
			continue;

		if (i915_seqno_passed(seqno, obj->last_read_seqno))
			continue;

		if ((obj->base.read_domains & I915_GEM_DOMAIN_COMMAND) == 0)
			continue;

		/* We need to copy these to an anonymous buffer as the simplest
		 * method to avoid being overwritten by userspace.
		 */
		return i915_error_object_create(dev_priv, obj);
	}

	return NULL;
}

static void i915_record_ring_state(struct drm_device *dev,
				   struct drm_i915_error_state *error,
				   struct intel_ring_buffer *ring)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen >= 6) {
		error->rc_psmi[ring->id] = I915_READ(ring->mmio_base + 0x50);
		error->fault_reg[ring->id] = I915_READ(RING_FAULT_REG(ring));
		error->semaphore_mboxes[ring->id][0]
			= I915_READ(RING_SYNC_0(ring->mmio_base));
		error->semaphore_mboxes[ring->id][1]
			= I915_READ(RING_SYNC_1(ring->mmio_base));
	}

	if (INTEL_INFO(dev)->gen >= 4) {
		error->faddr[ring->id] = I915_READ(RING_DMA_FADD(ring->mmio_base));
		error->ipeir[ring->id] = I915_READ(RING_IPEIR(ring->mmio_base));
		error->ipehr[ring->id] = I915_READ(RING_IPEHR(ring->mmio_base));
		error->instps[ring->id] = I915_READ(RING_INSTPS(ring->mmio_base));
		if (ring->id == RCS) {
			error->bbaddr = I915_READ64(BB_ADDR);
		}
	} else {
		error->faddr[ring->id] = I915_READ(DMA_FADD_I8XX);
		error->ipeir[ring->id] = I915_READ(IPEIR);
		error->ipehr[ring->id] = I915_READ(IPEHR);
	}
	i915_get_instdone(dev, error->instdone[ring->id],
		&dev_priv->ring[ring->id]);

	error->waiting[ring->id] = waitqueue_active(&ring->irq_queue);
	error->instpm[ring->id] = I915_READ(RING_INSTPM(ring->mmio_base));
	error->seqno[ring->id] = ring->get_seqno(ring, false);
	error->acthd[ring->id] = intel_ring_get_active_head(ring);
	error->head[ring->id] = I915_READ_HEAD(ring);
	error->tail[ring->id] = I915_READ_TAIL(ring);

	error->cpu_ring_head[ring->id] = ring->head;
	error->cpu_ring_tail[ring->id] = ring->tail;
}

static void i915_gem_record_rings(struct drm_device *dev,
				  struct drm_i915_error_state *error)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	struct drm_i915_gem_request *request;
	int i, count;

	for_each_ring(ring, dev_priv, i) {
		i915_record_ring_state(dev, error, ring);

		error->ring[i].batchbuffer =
			i915_error_first_batchbuffer(dev_priv, ring);

		error->ring[i].ringbuffer =
			i915_error_object_create(dev_priv, ring->obj);

		count = 0;
		list_for_each_entry(request, &ring->request_list, list)
			count++;

		error->ring[i].num_requests = count;
		error->ring[i].requests =
			kmalloc(count*sizeof(struct drm_i915_error_request),
				GFP_ATOMIC);
		if (error->ring[i].requests == NULL) {
			error->ring[i].num_requests = 0;
			continue;
		}

		count = 0;
		list_for_each_entry(request, &ring->request_list, list) {
			struct drm_i915_error_request *erq;

			erq = &error->ring[i].requests[count++];
			erq->seqno = request->seqno;
			erq->jiffies = request->emitted_jiffies;
			erq->tail = request->tail;
		}
	}
}

/**
 * i915_capture_error_state - capture an error record for later analysis
 * @dev: drm device
 *
 * Should be called when an error is detected (either a hang or an error
 * interrupt) to capture error state from the time of the error.  Fills
 * out a structure which becomes available in debugfs for user level tools
 * to pick up.
 */
static void i915_capture_error_state(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	struct drm_i915_error_state *error;
	unsigned long flags;
	int i, pipe;

	spin_lock_irqsave(&dev_priv->error_lock, flags);
	error = dev_priv->first_error;
	spin_unlock_irqrestore(&dev_priv->error_lock, flags);
	if (error)
		return;

	/* Account for pipe specific data like PIPE*STAT */
	error = kzalloc(sizeof(*error), GFP_ATOMIC);
	if (!error) {
		DRM_DEBUG_DRIVER("out of memory, not capturing error state\n");
		return;
	}

	DRM_INFO("capturing error event; look for more information in /debug/dri/%d/i915_error_state\n",
		 dev->primary->index);

	kref_init(&error->ref);
	error->eir = I915_READ(EIR);
	error->pgtbl_er = I915_READ(PGTBL_ER);
	error->ccid = I915_READ(CCID);

	if (HAS_PCH_SPLIT(dev))
		error->ier = I915_READ(DEIER) | I915_READ(GTIER);
	else if (IS_VALLEYVIEW(dev))
		error->ier = I915_READ(GTIER) | I915_READ(VLV_IER);
	else if (IS_GEN2(dev))
		error->ier = I915_READ16(IER);
	else
		error->ier = I915_READ(IER);

	for_each_pipe(pipe)
		error->pipestat[pipe] = I915_READ(PIPESTAT(pipe));

	if (INTEL_INFO(dev)->gen >= 6) {
		error->error = I915_READ(ERROR_GEN6);
		error->done_reg = I915_READ(DONE_REG);
	}

	for (i = 0; i < I915_NUM_RINGS; i++) {
		i915_get_instdone(dev, error->instdone[i],
			&dev_priv->ring[i]);
	}

	i915_gem_record_fences(dev, error);
	i915_gem_record_rings(dev, error);

	/* Record buffers on the active and pinned lists. */
	error->active_bo = NULL;
	error->pinned_bo = NULL;

	i = 0;
	list_for_each_entry(obj, &dev_priv->mm.active_list, mm_list)
		i++;
	error->active_bo_count = i;
	list_for_each_entry(obj, &dev_priv->mm.gtt_list, gtt_list)
		if (obj->pin_count)
			i++;
	error->pinned_bo_count = i - error->active_bo_count;

	error->active_bo = NULL;
	error->pinned_bo = NULL;
	if (i) {
		error->active_bo = kmalloc(sizeof(*error->active_bo)*i,
					   GFP_ATOMIC);
		if (error->active_bo)
			error->pinned_bo =
				error->active_bo + error->active_bo_count;
	}

	if (error->active_bo)
		error->active_bo_count =
			capture_active_bo(error->active_bo,
					  error->active_bo_count,
					  &dev_priv->mm.active_list);

	if (error->pinned_bo)
		error->pinned_bo_count =
			capture_pinned_bo(error->pinned_bo,
					  error->pinned_bo_count,
					  &dev_priv->mm.gtt_list);

	do_gettimeofday(&error->time);

	error->overlay = intel_overlay_capture_error_state(dev);
	error->display = intel_display_capture_error_state(dev);

	spin_lock_irqsave(&dev_priv->error_lock, flags);
	if (dev_priv->first_error == NULL) {
		dev_priv->first_error = error;
		error = NULL;
	}
	spin_unlock_irqrestore(&dev_priv->error_lock, flags);

	if (error)
		i915_error_state_free(&error->ref);
}

void i915_destroy_error_state(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_error_state *error;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->error_lock, flags);
	error = dev_priv->first_error;
	dev_priv->first_error = NULL;
	spin_unlock_irqrestore(&dev_priv->error_lock, flags);

	if (error)
		kref_put(&error->ref, i915_error_state_free);
}
#else
#define i915_capture_error_state(x)
#endif

static void i915_report_and_clear_eir(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t instdone[I915_MAX_INSTDONE_REG];
	u32 eir;
	int pipe, i;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->error_lock, flags);

	eir = I915_READ(EIR);
	if (!eir)
		goto i915_report_and_clear_eir_exit;

	pr_err("render error detected, EIR: 0x%08x\n", eir);

	i915_get_instdone(dev, instdone, &dev_priv->ring[RCS]);

	if (IS_G4X(dev)) {
		if (eir & (GM45_ERROR_MEM_PRIV | GM45_ERROR_CP_PRIV)) {
			u32 ipeir = I915_READ(IPEIR_I965);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR_I965));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR_I965));
			for (i = 0; i < ARRAY_SIZE(instdone); i++)
				pr_err("  INSTDONE_%d: 0x%08x\n",
					i, instdone[i]);
			pr_err("  INSTPS: 0x%08x\n", I915_READ(INSTPS));
			pr_err("  INSTDONE1: 0x%08x\n", I915_READ(INSTDONE1));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD_I965));
			I915_WRITE(IPEIR_I965, ipeir);
			POSTING_READ(IPEIR_I965);
		}
		if (eir & GM45_ERROR_PAGE_TABLE) {
			u32 pgtbl_err = I915_READ(PGTBL_ER);
			pr_err("page table error\n");
			pr_err("  PGTBL_ER: 0x%08x\n", pgtbl_err);
			I915_WRITE(PGTBL_ER, pgtbl_err);
			POSTING_READ(PGTBL_ER);
		}
	}

	if (!IS_GEN2(dev)) {
		if (eir & I915_ERROR_PAGE_TABLE) {
			u32 pgtbl_err = I915_READ(PGTBL_ER);
			pr_err("page table error\n");
			pr_err("  PGTBL_ER: 0x%08x\n", pgtbl_err);
			I915_WRITE(PGTBL_ER, pgtbl_err);
			POSTING_READ(PGTBL_ER);
		}
	}

	if (eir & I915_ERROR_MEMORY_REFRESH) {
		pr_err("memory refresh error:\n");
		for_each_pipe(pipe)
			pr_err("pipe %c stat: 0x%08x\n",
			       pipe_name(pipe), I915_READ(PIPESTAT(pipe)));
		/* pipestat has already been acked */
	}
	if (eir & I915_ERROR_INSTRUCTION) {
		pr_err("instruction error\n");
		pr_err("  INSTPM: 0x%08x\n", I915_READ(INSTPM));
		if (INTEL_INFO(dev)->gen < 4) {
			u32 ipeir = I915_READ(IPEIR);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR));
			pr_err("  INSTDONE: 0x%08x\n", I915_READ(INSTDONE));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD));
			I915_WRITE(IPEIR, ipeir);
			POSTING_READ(IPEIR);
		} else {
			u32 ipeir = I915_READ(IPEIR_I965);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR_I965));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR_I965));
			pr_err("  INSTDONE: 0x%08x\n",
			       I915_READ(INSTDONE_I965));
			pr_err("  INSTPS: 0x%08x\n", I915_READ(INSTPS));
			pr_err("  INSTDONE1: 0x%08x\n", I915_READ(INSTDONE1));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD_I965));
			I915_WRITE(IPEIR_I965, ipeir);
			POSTING_READ(IPEIR_I965);
		}
	}

	I915_WRITE(EIR, eir);
	POSTING_READ(EIR);
	eir = I915_READ(EIR);
	if (eir) {
		/*
		 * some errors might have become stuck,
		 * mask them.
		 */
		DRM_ERROR("EIR stuck: 0x%08x, masking\n", eir);
		I915_WRITE(EMR, I915_READ(EMR) | eir);
		I915_WRITE(IIR, I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);
	}

i915_report_and_clear_eir_exit:
	spin_unlock_irqrestore(&dev_priv->error_lock, flags);
}

/**
 * i915_handle_error - handle an error interrupt
 * @dev: drm device
 *
 * Do some basic checking of register state at error interrupt time and
 * dump it to the syslog.  Also call i915_capture_error_state() to make
 * sure we get a record and make it available in debugfs.  Fire a uevent
 * so userspace knows something bad happened (should trigger collection
 * of a ring dump etc.).
 */
void i915_handle_error(struct drm_device *dev, struct intel_hangcheck *hc,
			int watchdog)
{
	u32 i;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int full_reset = 0;
	unsigned long cur_time;
	unsigned long last_reset;

	/* This can be called due to an error interrupt or because
	* one of the rings has hung.*/
	i915_capture_error_state(dev);
	i915_report_and_clear_eir(dev);

	/* Currently we only support individual ring reset for GEN7 onwards,
	 * older chips will revert to a full reset.
	 * Error interrupts trigger a full reset (hc == NULL)*/
	if ((INTEL_INFO(dev)->gen >= 7) && hc) {
		if (!watchdog) {
			cur_time = get_seconds();
			last_reset = hc->last_reset;
			hc->last_reset = cur_time;
		}

		if (!watchdog && ((cur_time - last_reset) <
			i915_ring_reset_min_alive_period)) {
			/* This ring is hanging too frequently.
			* Opt for full-reset instead */
			DRM_DEBUG_TDR("Ring %d hanging too quickly...\r\n",
				hc->ringid);
			full_reset = 1;
		} else {
			if (atomic_read(&hc->reset)) {
				/* Reset already in progress for this ring */
				return;
			}

			atomic_set(&hc->reset, 1);
		}
	} else
		full_reset = 1;

	if (!hc || full_reset) {
		/* Full GPU reset */
		if (atomic_read(&dev_priv->full_reset))
			return;

		atomic_set(&dev_priv->full_reset, 1);
	}

	/* We must wake up the irq_queue. A thread may be in
	* the __wait_seqno function and may currently hold
	* dev->struct_mutex. The current driver design forces
	* it to wakeup so that it will see the reset flags and
	* abandon the wait, which in turn will release the
	* mutex. New wait requests will not be allowed to start
	* whilst the reset/full_reset flags are set.*/
	for (i = 0; i < I915_NUM_RINGS; i++)
		wake_up_all(&dev_priv->ring[i].irq_queue);

	/* If error_work is already in the work queue then it will not be added
	* again. It hasn't yet executed so it will see the reset flags when
	* it is scheduled. If it isn't in the queue or it is currently
	* executing then this call will add it to the queue again so that
	* even if it misses the reset flags during the current call it is
	* guaranteed to see them on the next call.
	*/
	DRM_DEBUG_TDR("Queue error work...\r\n");

	queue_work(dev_priv->wq, &dev_priv->error_work);
}

static void i915_pageflip_stall_check(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_i915_gem_object *obj;
	struct intel_unpin_work *work;
	unsigned long flags;
	bool stall_detected;

	/* Ignore early vblank irqs */
	if (intel_crtc == NULL)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	work = intel_crtc->unpin_work;

	if (work == NULL || work->pending || !work->enable_stall_check) {
		/* Either the pending flip IRQ arrived, or we're too early. Don't check */
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return;
	}

	/* Potential stall - if we see that the flip has happened, assume a missed interrupt */
	obj = work->pending_flip_obj;
	if (INTEL_INFO(dev)->gen >= 4) {
		int dspsurf = DSPSURF(intel_crtc->plane);
		stall_detected = I915_HI_DISPBASE(I915_READ(dspsurf)) ==
					obj->gtt_offset;
	} else {
		int dspaddr = DSPADDR(intel_crtc->plane);
		stall_detected = I915_READ(dspaddr) == (obj->gtt_offset +
							crtc->y * crtc->fb->pitches[0] +
							crtc->x * crtc->fb->bits_per_pixel/8);
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (stall_detected) {
		DRM_DEBUG_DRIVER("Pageflip stall detected\n");
		intel_prepare_page_flip(dev, intel_crtc->plane);
	}
}

/* Called from drm generic code, passed 'crtc' which
 * we use as a pipe index
 */
static int i915_enable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (INTEL_INFO(dev)->gen >= 4)
		i915_enable_pipestat(dev_priv, pipe,
				     PIPE_START_VBLANK_INTERRUPT_ENABLE);
	else
		i915_enable_pipestat(dev_priv, pipe,
				     PIPE_VBLANK_INTERRUPT_ENABLE);

	/* maintain vblank delivery even in deep C-states */
	if (dev_priv->info->gen == 3)
		I915_WRITE(INSTPM, _MASKED_BIT_DISABLE(INSTPM_AGPBUSY_DIS));
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int ironlake_enable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_enable_display_irq(dev_priv, (pipe == 0) ?
				    DE_PIPEA_VBLANK : DE_PIPEB_VBLANK);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int ivybridge_enable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_enable_display_irq(dev_priv,
				    DE_PIPEA_VBLANK_IVB << (5 * pipe));
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int valleyview_enable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;
	u32 imr;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(VLV_IMR);
	if (pipe == 0)
		imr &= ~I915_DISPLAY_PIPE_A_VBLANK_INTERRUPT;
	else
		imr &= ~I915_DISPLAY_PIPE_B_VBLANK_INTERRUPT;
	I915_WRITE(VLV_IMR, imr);
	i915_enable_pipestat(dev_priv, pipe,
			     PIPE_START_VBLANK_INTERRUPT_ENABLE);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

/* Added fo HDMI AUdio */
int i915_enable_hdmi_audio_int(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;
	u32 imr;
	int pipe = 1;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(VLV_IMR);
	/* Audio is on Stream B */
	imr &= ~I915_LPE_PIPE_B_INTERRUPT;
	I915_WRITE(VLV_IMR, imr);
	i915_enable_lpe_pipestat(dev_priv, pipe);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

/* Called from drm generic code, passed 'crtc' which
 * we use as a pipe index
 */
static void i915_disable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (dev_priv->info->gen == 3)
		I915_WRITE(INSTPM, _MASKED_BIT_ENABLE(INSTPM_AGPBUSY_DIS));

	i915_disable_pipestat(dev_priv, pipe,
			      PIPE_VBLANK_INTERRUPT_ENABLE |
			      PIPE_START_VBLANK_INTERRUPT_ENABLE);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static void ironlake_disable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_disable_display_irq(dev_priv, (pipe == 0) ?
				     DE_PIPEA_VBLANK : DE_PIPEB_VBLANK);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static void ivybridge_disable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_disable_display_irq(dev_priv,
				     DE_PIPEA_VBLANK_IVB << (pipe * 5));
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static void valleyview_disable_vblank(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;
	u32 imr;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_disable_pipestat(dev_priv, pipe,
			      PIPE_START_VBLANK_INTERRUPT_ENABLE);
	imr = I915_READ(VLV_IMR);
	if (pipe == 0)
		imr |= I915_DISPLAY_PIPE_A_VBLANK_INTERRUPT;
	else
		imr |= I915_DISPLAY_PIPE_B_VBLANK_INTERRUPT;
	I915_WRITE(VLV_IMR, imr);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

/* Added for HDMI Audio */
int i915_disable_hdmi_audio_int(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;
	u32 imr;
	int pipe = 1;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(VLV_IMR);
	imr |= I915_LPE_PIPE_B_INTERRUPT;
	I915_WRITE(VLV_IMR, imr);
	i915_disable_lpe_pipestat(dev_priv, pipe);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}
static u32
ring_last_seqno(struct intel_ring_buffer *ring)
{
	return list_entry(ring->request_list.prev,
			  struct drm_i915_gem_request, list)->seqno;
}

static bool i915_hangcheck_ring_idle(struct intel_ring_buffer *ring, bool *err)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t head;
	uint32_t tail;
	int pending_work = 1;

	if (list_empty(&ring->request_list) ||
	    i915_seqno_passed(ring->get_seqno(ring, false),
			      ring_last_seqno(ring))) {
		/* Issue a wake-up to catch stuck h/w. */
		if (waitqueue_active(&ring->irq_queue)) {
			*err = true;
		}
		pending_work = 0;
	}

	/* Make sure that the ring has caught up with any
	* commands that are inserted directly in the ring
	* Note: Head & tail will become equal *before* the
	* ring has finished executing the current command
	* so it is possible that we will consider the ring as idle
	* prematurely. This is ok as the hang will be detected
	* when new work is next added to the ring.*/
	head = I915_READ_HEAD(ring) & HEAD_ADDR;
	tail = I915_READ_TAIL(ring) & TAIL_ADDR;

	/* The ring is idle if the head pointer == tail and no more work*/
	return (((head == tail) && (pending_work == 0)) ? true : false);
}

static bool kick_ring(struct intel_ring_buffer *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp = I915_READ_CTL(ring);
	if (tmp & RING_WAIT) {
		DRM_ERROR("Kicking stuck wait on %s\n",
			  ring->name);
		I915_WRITE_CTL(ring, tmp);
		return true;
	}
	return false;
}

static bool i915_hangcheck_hung(struct intel_hangcheck *hc)
{
	struct drm_device *dev = hc->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t mbox_wait;
	uint32_t threshold;
	struct intel_ring_buffer *ring;

	DRM_DEBUG_TDR("Ring [%d] hc->count = %d\r\n", hc->ringid, hc->count);

	ring = &dev_priv->ring[hc->ringid];

	/* Is this ring waiting on a semaphore mbox?
	* If so, give it a bit longer as it may be waiting on another
	* ring which has actually hung. Give the other ring chance to
	* reset and clear the hang.
	*/
	mbox_wait = ((I915_READ(RING_CTL(ring->mmio_base)) >> 10) & 0x1);
	threshold = mbox_wait ? MBOX_HANGCHECK_THRESHOLD : HANGCHECK_THRESHOLD;

	if (hc->count++ > threshold) {
		bool hung = true;

		DRM_DEBUG_TDR("Hangcheck timer elapsed... ring %d hung\n",
			hc->ringid);

		/* Reset the counter*/
		hc->count = 0;

		if (!IS_GEN2(dev)) {
			/* If the ring is hanging on a WAIT_FOR_EVENT
			 * then simply poke the RB_WAIT bit
			 * and break the hang. This should work on
			 * all but the second generation chipsets.
			 */
			ring = &dev_priv->ring[hc->ringid];
			hung &= !kick_ring(ring);
		}

		if (hung)
			i915_handle_error(dev, hc, 0);

		return hung;
	}

	return false;
}

/**
 * This is called when the chip hasn't reported back with completed
 * batchbuffers in a long time. The first time this is called we simply record
 * ACTHD. If ACTHD hasn't changed by the time the hangcheck timer elapses
 * again, we assume the chip is wedged and try to fix it.
 */
void i915_hangcheck_elapsed(unsigned long data)
{
	struct intel_hangcheck *hc = (struct intel_hangcheck *)data;
	struct drm_device *dev;
	drm_i915_private_t *dev_priv;
	uint32_t acthd, instdone[I915_MAX_INSTDONE_REG];
	struct intel_ring_buffer *ring;
	bool err = false, idle;

	if (!i915_enable_hangcheck || !hc)
		return;

	dev = hc->dev;
	dev_priv = dev->dev_private;

	/* Check if the specified ring is idle and record active head */
	ring = &dev_priv->ring[hc->ringid];
	idle = i915_hangcheck_ring_idle(ring, &err);
	acthd = intel_ring_get_active_head(ring);

	DRM_DEBUG_TDR("[%d] ahd: 0x%08x lst: 0x0%08x tl: 0x%08x idle: %d\r\n",
		hc->ringid, acthd, hc->last_acthd,
		I915_READ(RING_TAIL(ring->mmio_base)), idle);

	/* If all work is done then ACTHD clearly hasn't advanced */
	if (idle) {
		if (err) {
			i915_hangcheck_hung(hc);
			goto repeat;
		}

		hc->count = 0;
		return;
	}

	i915_get_instdone(dev, instdone, ring);
	if ((hc->last_acthd == acthd) &&
	(memcmp(hc->prev_instdone, instdone, sizeof(instdone)) == 0)) {
		i915_hangcheck_hung(hc);
	} else {
		hc->count = 0;

		hc->last_acthd = acthd;
		memcpy(hc->prev_instdone, instdone, sizeof(instdone));
	}

repeat:
	/* Reset timer in case chip hangs without another request being added*/
	mod_timer(&hc->timer, jiffies + DRM_I915_HANGCHECK_JIFFIES);
}

/* drm_dma.h hooks
*/
static void ironlake_irq_preinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	atomic_set(&dev_priv->irq_received, 0);

	I915_WRITE(HWSTAM, 0xeffe);

	/* XXX hotplug from PCH */

	I915_WRITE(DEIMR, 0xffffffff);
	I915_WRITE(DEIER, 0x0);
	POSTING_READ(DEIER);

	/* and GT */
	I915_WRITE(GTIMR, 0xffffffff);
	I915_WRITE(GTIER, 0x0);
	POSTING_READ(GTIER);

	/* south display irq */
	I915_WRITE(SDEIMR, 0xffffffff);
	I915_WRITE(SDEIER, 0x0);
	POSTING_READ(SDEIER);
}

static void valleyview_irq_preinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	atomic_set(&dev_priv->irq_received, 0);

	/* VLV magic */
	I915_WRITE(VLV_IMR, 0);
	I915_WRITE(RING_IMR(RENDER_RING_BASE), 0);
	I915_WRITE(RING_IMR(GEN6_BSD_RING_BASE), 0);
	I915_WRITE(RING_IMR(BLT_RING_BASE), 0);

	/* and GT */
	I915_WRITE(GTIIR, I915_READ(GTIIR));
	I915_WRITE(GTIIR, I915_READ(GTIIR));
	I915_WRITE(GTIMR, 0xffffffff);
	I915_WRITE(GTIER, 0x0);
	POSTING_READ(GTIER);

	I915_WRITE(DPINVGTT, 0xff);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);
	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	POSTING_READ(VLV_IER);
}

/*
 * Enable digital hotplug on the PCH, and configure the DP short pulse
 * duration to 2ms (which is the minimum in the Display Port spec)
 *
 * This register is the same on all known PCH chips.
 */

static void ironlake_enable_pch_hotplug(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32	hotplug;

	hotplug = I915_READ(PCH_PORT_HOTPLUG);
	hotplug &= ~(PORTD_PULSE_DURATION_MASK|PORTC_PULSE_DURATION_MASK|PORTB_PULSE_DURATION_MASK);
	hotplug |= PORTD_HOTPLUG_ENABLE | PORTD_PULSE_DURATION_2ms;
	hotplug |= PORTC_HOTPLUG_ENABLE | PORTC_PULSE_DURATION_2ms;
	hotplug |= PORTB_HOTPLUG_ENABLE | PORTB_PULSE_DURATION_2ms;
	I915_WRITE(PCH_PORT_HOTPLUG, hotplug);
}

static int ironlake_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	/* enable kind of interrupts always enabled */
	u32 display_mask = DE_MASTER_IRQ_CONTROL | DE_GSE | DE_PCH_EVENT |
			   DE_PLANEA_FLIP_DONE | DE_PLANEB_FLIP_DONE;
	u32 render_irqs;
	u32 hotplug_mask;

	dev_priv->irq_mask = ~display_mask;

	/* should always can generate irq */
	I915_WRITE(DEIIR, I915_READ(DEIIR));
	I915_WRITE(DEIMR, dev_priv->irq_mask);
	I915_WRITE(DEIER, display_mask | DE_PIPEA_VBLANK | DE_PIPEB_VBLANK);
	POSTING_READ(DEIER);

	dev_priv->gt_irq_mask = ~0;

	I915_WRITE(GTIIR, I915_READ(GTIIR));
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);

	if (IS_GEN6(dev))
		render_irqs =
			GT_USER_INTERRUPT |
			GEN6_BSD_USER_INTERRUPT |
			GEN6_BLITTER_USER_INTERRUPT;
	else
		render_irqs =
			GT_USER_INTERRUPT |
			GT_PIPE_NOTIFY |
			GT_BSD_USER_INTERRUPT;
	I915_WRITE(GTIER, render_irqs);
	POSTING_READ(GTIER);

	if (HAS_PCH_CPT(dev)) {
		hotplug_mask = (SDE_CRT_HOTPLUG_CPT |
				SDE_PORTB_HOTPLUG_CPT |
				SDE_PORTC_HOTPLUG_CPT |
				SDE_PORTD_HOTPLUG_CPT);
	} else {
		hotplug_mask = (SDE_CRT_HOTPLUG |
				SDE_PORTB_HOTPLUG |
				SDE_PORTC_HOTPLUG |
				SDE_PORTD_HOTPLUG |
				SDE_AUX_MASK);
	}

	dev_priv->pch_irq_mask = ~hotplug_mask;

	I915_WRITE(SDEIIR, I915_READ(SDEIIR));
	I915_WRITE(SDEIMR, dev_priv->pch_irq_mask);
	I915_WRITE(SDEIER, hotplug_mask);
	POSTING_READ(SDEIER);

	ironlake_enable_pch_hotplug(dev);

	if (IS_IRONLAKE_M(dev)) {
		/* Clear & enable PCU event interrupts */
		I915_WRITE(DEIIR, DE_PCU_EVENT);
		I915_WRITE(DEIER, I915_READ(DEIER) | DE_PCU_EVENT);
		ironlake_enable_display_irq(dev_priv, DE_PCU_EVENT);
	}

	return 0;
}

static int ivybridge_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	/* enable kind of interrupts always enabled */
	u32 display_mask =
		DE_MASTER_IRQ_CONTROL | DE_GSE_IVB | DE_PCH_EVENT_IVB |
		DE_PLANEC_FLIP_DONE_IVB |
		DE_PLANEB_FLIP_DONE_IVB |
		DE_PLANEA_FLIP_DONE_IVB;
	u32 render_irqs;
	u32 hotplug_mask;

	dev_priv->irq_mask = ~display_mask;

	/* should always can generate irq */
	I915_WRITE(DEIIR, I915_READ(DEIIR));
	I915_WRITE(DEIMR, dev_priv->irq_mask);
	I915_WRITE(DEIER,
		   display_mask |
		   DE_PIPEC_VBLANK_IVB |
		   DE_PIPEB_VBLANK_IVB |
		   DE_PIPEA_VBLANK_IVB);
	POSTING_READ(DEIER);

	dev_priv->gt_irq_mask = ~GT_GEN7_L3_PARITY_ERROR_INTERRUPT;

	I915_WRITE(GTIIR, I915_READ(GTIIR));
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);

	render_irqs = GT_USER_INTERRUPT | GEN6_BSD_USER_INTERRUPT |
		GEN6_BLITTER_USER_INTERRUPT | GT_GEN7_L3_PARITY_ERROR_INTERRUPT;
	I915_WRITE(GTIER, render_irqs);
	POSTING_READ(GTIER);

	hotplug_mask = (SDE_CRT_HOTPLUG_CPT |
			SDE_PORTB_HOTPLUG_CPT |
			SDE_PORTC_HOTPLUG_CPT |
			SDE_PORTD_HOTPLUG_CPT);
	dev_priv->pch_irq_mask = ~hotplug_mask;

	I915_WRITE(SDEIIR, I915_READ(SDEIIR));
	I915_WRITE(SDEIMR, dev_priv->pch_irq_mask);
	I915_WRITE(SDEIER, hotplug_mask);
	POSTING_READ(SDEIER);

	ironlake_enable_pch_hotplug(dev);

	return 0;
}

static int valleyview_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 enable_mask;
	u32 lpe_status_clear;
	u32 hotplug_en = I915_READ(PORT_HOTPLUG_EN);
	u32 pipestat_enable = PLANE_FLIP_DONE_INT_EN_VLV;
	u32 render_irqs;

	enable_mask = I915_DISPLAY_PORT_INTERRUPT;
	enable_mask |= I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_A_VBLANK_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		/*I915_DISPLAY_PIPE_B_VBLANK_INTERRUPT;*/
		/* Added for HDMI Audio */
		I915_DISPLAY_PIPE_B_VBLANK_INTERRUPT |
		I915_LPE_PIPE_B_INTERRUPT |
		I915_LPE_PIPE_A_INTERRUPT;

#ifdef CONFIG_DRM_VXD_BYT
	enable_mask |= VED_BLOCK_INTERRUPT;
#endif
	/*
	 *Leave vblank interrupts masked initially.  enable/disable will
	 * toggle them based on usage.
	 */
	dev_priv->irq_mask = (~enable_mask) |
		I915_DISPLAY_PIPE_A_VBLANK_INTERRUPT |
		I915_DISPLAY_PIPE_B_VBLANK_INTERRUPT;

	/* Added for HDMI Audio.
	 *Leave LPE interrupts masked initially.  enable/disable will
	 * toggle them based on usage.
	 */
	dev_priv->irq_mask = (dev_priv->irq_mask) |
		I915_LPE_PIPE_A_INTERRUPT |
		I915_LPE_PIPE_B_INTERRUPT;

	dev_priv->pipestat[0] = 0;
	dev_priv->pipestat[1] = 0;

	I915_WRITE(VLV_IMR, dev_priv->irq_mask);
	I915_WRITE(VLV_IER, enable_mask);
	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(PIPESTAT(0), 0xffff);
	I915_WRITE(PIPESTAT(1), 0xffff);
	/* Added for HDMI Audio */
	lpe_status_clear = I915_HDMI_AUDIO_UNDERRUN |
			I915_HDMI_AUDIO_BUFFER_DONE;

	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_A, lpe_status_clear);
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, lpe_status_clear);
	POSTING_READ(VLV_IER);

	i915_enable_pipestat(dev_priv, 0, pipestat_enable);
	i915_enable_pipestat(dev_priv, 1, pipestat_enable);

	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IIR, 0xffffffff);

	I915_WRITE(GTIIR, I915_READ(GTIIR));
	dev_priv->gt_irq_mask = ~(GT_GEN7_L3_PARITY_ERROR_INTERRUPT |
				GT_GEN6_BSD_WATCHDOG_INTERRUPT |
				GT_GEN6_RENDER_WATCHDOG_INTERRUPT);
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);

	render_irqs = GT_USER_INTERRUPT | GEN6_BSD_USER_INTERRUPT |
		GEN6_BLITTER_USER_INTERRUPT |
		GEN6_RENDER_TIMEOUT_COUNTER_EXPIRED |
		GEN6_BSD_TIMEOUT_COUNTER_EXPIRED;
	I915_WRITE(GTIER, render_irqs);
	POSTING_READ(GTIER);

	/* ack & enable invalid PTE error interrupts */
#if 0 /* FIXME: add support to irq handler for checking these bits */
	I915_WRITE(DPINVGTT, DPINVGTT_STATUS_MASK);
	I915_WRITE(DPINVGTT, DPINVGTT_EN_MASK);
#endif

	I915_WRITE(VLV_MASTER_IER, MASTER_INTERRUPT_ENABLE);
	/* Note HDMI and DP share bits */
	if (dev_priv->hotplug_supported_mask & HDMIB_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMIB_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & HDMIC_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMIC_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & HDMID_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMID_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & SDVOC_HOTPLUG_INT_STATUS_I915)
		hotplug_en |= SDVOC_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & SDVOB_HOTPLUG_INT_STATUS_I915)
		hotplug_en |= SDVOB_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & CRT_HOTPLUG_INT_STATUS) {
		hotplug_en |= CRT_HOTPLUG_INT_EN;
		hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;
	}

	I915_WRITE(PORT_HOTPLUG_EN, hotplug_en);

	return 0;
}

static void valleyview_irq_uninstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	if (!dev_priv)
		return;

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);

	I915_WRITE(HWSTAM, 0xffffffff);
	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);
	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	POSTING_READ(VLV_IER);
}

static void ironlake_irq_uninstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (!dev_priv)
		return;

	I915_WRITE(HWSTAM, 0xffffffff);

	I915_WRITE(DEIMR, 0xffffffff);
	I915_WRITE(DEIER, 0x0);
	I915_WRITE(DEIIR, I915_READ(DEIIR));

	I915_WRITE(GTIMR, 0xffffffff);
	I915_WRITE(GTIER, 0x0);
	I915_WRITE(GTIIR, I915_READ(GTIIR));

	I915_WRITE(SDEIMR, 0xffffffff);
	I915_WRITE(SDEIER, 0x0);
	I915_WRITE(SDEIIR, I915_READ(SDEIIR));
}

static void i8xx_irq_preinstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	atomic_set(&dev_priv->irq_received, 0);

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE16(IMR, 0xffff);
	I915_WRITE16(IER, 0x0);
	POSTING_READ16(IER);
}

static int i8xx_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	dev_priv->pipestat[0] = 0;
	dev_priv->pipestat[1] = 0;

	I915_WRITE16(EMR,
		     ~(I915_ERROR_PAGE_TABLE | I915_ERROR_MEMORY_REFRESH));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		  I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
		  I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);
	I915_WRITE16(IMR, dev_priv->irq_mask);

	I915_WRITE16(IER,
		     I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		     I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		     I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT |
		     I915_USER_INTERRUPT);
	POSTING_READ16(IER);

	return 0;
}

static irqreturn_t i8xx_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u16 iir, new_iir;
	u32 pipe_stats[2] = {0};
	unsigned long irqflags;
	int irq_received;
	int pipe;
	u16 flip_mask =
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;

	atomic_inc(&dev_priv->irq_received);

	iir = I915_READ16(IIR);
	if (iir == 0)
		return IRQ_NONE;

	while (iir & ~flip_mask) {
		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/*
			 * Clear the PIPE*STAT regs before the IIR
			 */
			if (pipe_stats[pipe] & 0x8000ffff) {
				if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
					DRM_DEBUG_DRIVER("pipe %c underrun\n",
							 pipe_name(pipe));
				I915_WRITE(reg, pipe_stats[pipe]);
				irq_received = 1;
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		I915_WRITE16(IIR, iir & ~flip_mask);
		new_iir = I915_READ16(IIR); /* Flush posted writes */

		i915_update_dri1_breadcrumb(dev);

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);

		if (pipe_stats[0] & PIPE_VBLANK_INTERRUPT_STATUS &&
		    drm_handle_vblank(dev, 0)) {
			if (iir & I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT) {
				intel_prepare_page_flip(dev, 0);
				intel_finish_page_flip(dev, 0);
				flip_mask &= ~I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT;
			}
		}

		if (pipe_stats[1] & PIPE_VBLANK_INTERRUPT_STATUS &&
		    drm_handle_vblank(dev, 1)) {
			if (iir & I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT) {
				intel_prepare_page_flip(dev, 1);
				intel_finish_page_flip(dev, 1);
				flip_mask &= ~I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;
			}
		}

		iir = new_iir;
	}

	return IRQ_HANDLED;
}

static void i8xx_irq_uninstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	for_each_pipe(pipe) {
		/* Clear enable bits; then clear status bits */
		I915_WRITE(PIPESTAT(pipe), 0);
		I915_WRITE(PIPESTAT(pipe), I915_READ(PIPESTAT(pipe)));
	}
	I915_WRITE16(IMR, 0xffff);
	I915_WRITE16(IER, 0x0);
	I915_WRITE16(IIR, I915_READ16(IIR));
}

static void i915_irq_preinstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	atomic_set(&dev_priv->irq_received, 0);

	if (I915_HAS_HOTPLUG(dev)) {
		I915_WRITE(PORT_HOTPLUG_EN, 0);
		I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	}

	I915_WRITE16(HWSTAM, 0xeffe);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);
	POSTING_READ(IER);
}

static int i915_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 enable_mask;

	dev_priv->pipestat[0] = 0;
	dev_priv->pipestat[1] = 0;

	I915_WRITE(EMR, ~(I915_ERROR_PAGE_TABLE | I915_ERROR_MEMORY_REFRESH));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_ASLE_INTERRUPT |
		  I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		  I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
		  I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);

	enable_mask =
		I915_ASLE_INTERRUPT |
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT |
		I915_USER_INTERRUPT;

	if (I915_HAS_HOTPLUG(dev)) {
		/* Enable in IER... */
		enable_mask |= I915_DISPLAY_PORT_INTERRUPT;
		/* and unmask in IMR */
		dev_priv->irq_mask &= ~I915_DISPLAY_PORT_INTERRUPT;
	}

	I915_WRITE(IMR, dev_priv->irq_mask);
	I915_WRITE(IER, enable_mask);
	POSTING_READ(IER);

	if (I915_HAS_HOTPLUG(dev)) {
		u32 hotplug_en = I915_READ(PORT_HOTPLUG_EN);

		if (dev_priv->hotplug_supported_mask & HDMIB_HOTPLUG_INT_STATUS)
			hotplug_en |= HDMIB_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & HDMIC_HOTPLUG_INT_STATUS)
			hotplug_en |= HDMIC_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & HDMID_HOTPLUG_INT_STATUS)
			hotplug_en |= HDMID_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & SDVOC_HOTPLUG_INT_STATUS_I915)
			hotplug_en |= SDVOC_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & SDVOB_HOTPLUG_INT_STATUS_I915)
			hotplug_en |= SDVOB_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & CRT_HOTPLUG_INT_STATUS) {
			hotplug_en |= CRT_HOTPLUG_INT_EN;
			hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;
		}

		/* Ignore TV since it's buggy */

		I915_WRITE(PORT_HOTPLUG_EN, hotplug_en);
	}

	intel_opregion_enable_asle(dev);

	return 0;
}

static irqreturn_t i915_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 iir, new_iir, pipe_stats[I915_MAX_PIPES] = {0};
	unsigned long irqflags;
	u32 flip_mask =
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;
	u32 flip[2] = {
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT,
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT
	};
	int pipe, ret = IRQ_NONE;

	atomic_inc(&dev_priv->irq_received);

	iir = I915_READ(IIR);
	do {
		bool irq_received = (iir & ~flip_mask) != 0;
		bool blc_event = false;

		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/* Clear the PIPE*STAT regs before the IIR */
			if (pipe_stats[pipe] & 0x8000ffff) {
				if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
					DRM_DEBUG_DRIVER("pipe %c underrun\n",
							 pipe_name(pipe));
				I915_WRITE(reg, pipe_stats[pipe]);
				irq_received = true;
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		if (!irq_received)
			break;

		/* Consume port.  Then clear IIR or we'll miss events */
		if ((I915_HAS_HOTPLUG(dev)) &&
		    (iir & I915_DISPLAY_PORT_INTERRUPT)) {
			u32 hotplug_status = I915_READ(PORT_HOTPLUG_STAT);

			DRM_DEBUG_DRIVER("hotplug event received, stat 0x%08x\n",
				  hotplug_status);
			if (hotplug_status & dev_priv->hotplug_supported_mask)
				queue_work(dev_priv->wq,
					   &dev_priv->hotplug_work);

			I915_WRITE(PORT_HOTPLUG_STAT, hotplug_status);
			POSTING_READ(PORT_HOTPLUG_STAT);
		}

		I915_WRITE(IIR, iir & ~flip_mask);
		new_iir = I915_READ(IIR); /* Flush posted writes */

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);

		for_each_pipe(pipe) {
			int plane = pipe;
			if (IS_MOBILE(dev))
				plane = !plane;
			if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS &&
			    drm_handle_vblank(dev, pipe)) {
				if (iir & flip[plane]) {
					intel_prepare_page_flip(dev, plane);
					intel_finish_page_flip(dev, pipe);
					flip_mask &= ~flip[plane];
				}
			}

			if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
				blc_event = true;
		}

		if (blc_event || (iir & I915_ASLE_INTERRUPT))
			intel_opregion_asle_intr(dev);

		/* With MSI, interrupts are only generated when iir
		 * transitions from zero to nonzero.  If another bit got
		 * set while we were handling the existing iir bits, then
		 * we would never get another interrupt.
		 *
		 * This is fine on non-MSI as well, as if we hit this path
		 * we avoid exiting the interrupt handler only to generate
		 * another one.
		 *
		 * Note that for MSI this could cause a stray interrupt report
		 * if an interrupt landed in the time between writing IIR and
		 * the posting read.  This should be rare enough to never
		 * trigger the 99% of 100,000 interrupts test for disabling
		 * stray interrupts.
		 */
		ret = IRQ_HANDLED;
		iir = new_iir;
	} while (iir & ~flip_mask);

	i915_update_dri1_breadcrumb(dev);

	return ret;
}

static void i915_irq_uninstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	if (I915_HAS_HOTPLUG(dev)) {
		I915_WRITE(PORT_HOTPLUG_EN, 0);
		I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	}

	I915_WRITE16(HWSTAM, 0xffff);
	for_each_pipe(pipe) {
		/* Clear enable bits; then clear status bits */
		I915_WRITE(PIPESTAT(pipe), 0);
		I915_WRITE(PIPESTAT(pipe), I915_READ(PIPESTAT(pipe)));
	}
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);

	I915_WRITE(IIR, I915_READ(IIR));
}

static void i965_irq_preinstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	atomic_set(&dev_priv->irq_received, 0);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	I915_WRITE(HWSTAM, 0xeffe);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);
	POSTING_READ(IER);
}

static int i965_irq_postinstall(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 hotplug_en;
	u32 enable_mask;
	u32 error_mask;

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask = ~(I915_ASLE_INTERRUPT |
			       I915_DISPLAY_PORT_INTERRUPT |
			       I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
			       I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
			       I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
			       I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
			       I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);

	enable_mask = ~dev_priv->irq_mask;
	enable_mask |= I915_USER_INTERRUPT;

	if (IS_G4X(dev))
		enable_mask |= I915_BSD_USER_INTERRUPT;

	dev_priv->pipestat[0] = 0;
	dev_priv->pipestat[1] = 0;

	/*
	 * Enable some error detection, note the instruction error mask
	 * bit is reserved, so we leave it masked.
	 */
	if (IS_G4X(dev)) {
		error_mask = ~(GM45_ERROR_PAGE_TABLE |
			       GM45_ERROR_MEM_PRIV |
			       GM45_ERROR_CP_PRIV |
			       I915_ERROR_MEMORY_REFRESH);
	} else {
		error_mask = ~(I915_ERROR_PAGE_TABLE |
			       I915_ERROR_MEMORY_REFRESH);
	}
	I915_WRITE(EMR, error_mask);

	I915_WRITE(IMR, dev_priv->irq_mask);
	I915_WRITE(IER, enable_mask);
	POSTING_READ(IER);

	/* Note HDMI and DP share hotplug bits */
	hotplug_en = 0;
	if (dev_priv->hotplug_supported_mask & HDMIB_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMIB_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & HDMIC_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMIC_HOTPLUG_INT_EN;
	if (dev_priv->hotplug_supported_mask & HDMID_HOTPLUG_INT_STATUS)
		hotplug_en |= HDMID_HOTPLUG_INT_EN;
	if (IS_G4X(dev)) {
		if (dev_priv->hotplug_supported_mask & SDVOC_HOTPLUG_INT_STATUS_G4X)
			hotplug_en |= SDVOC_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & SDVOB_HOTPLUG_INT_STATUS_G4X)
			hotplug_en |= SDVOB_HOTPLUG_INT_EN;
	} else {
		if (dev_priv->hotplug_supported_mask & SDVOC_HOTPLUG_INT_STATUS_I965)
			hotplug_en |= SDVOC_HOTPLUG_INT_EN;
		if (dev_priv->hotplug_supported_mask & SDVOB_HOTPLUG_INT_STATUS_I965)
			hotplug_en |= SDVOB_HOTPLUG_INT_EN;
	}
	if (dev_priv->hotplug_supported_mask & CRT_HOTPLUG_INT_STATUS) {
		hotplug_en |= CRT_HOTPLUG_INT_EN;

		/* Programming the CRT detection parameters tends
		   to generate a spurious hotplug event about three
		   seconds later.  So just do it once.
		   */
		if (IS_G4X(dev))
			hotplug_en |= CRT_HOTPLUG_ACTIVATION_PERIOD_64;
		hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;
	}

	/* Ignore TV since it's buggy */

	I915_WRITE(PORT_HOTPLUG_EN, hotplug_en);

	intel_opregion_enable_asle(dev);

	return 0;
}

static irqreturn_t i965_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 iir, new_iir;
	u32 pipe_stats[I915_MAX_PIPES] = {0};
	unsigned long irqflags;
	int irq_received;
	int ret = IRQ_NONE, pipe;

	atomic_inc(&dev_priv->irq_received);

	iir = I915_READ(IIR);

	for (;;) {
		bool blc_event = false;

		irq_received = iir != 0;

		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/*
			 * Clear the PIPE*STAT regs before the IIR
			 */
			if (pipe_stats[pipe] & 0x8000ffff) {
				if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
					DRM_DEBUG_DRIVER("pipe %c underrun\n",
							 pipe_name(pipe));
				I915_WRITE(reg, pipe_stats[pipe]);
				irq_received = 1;
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		if (!irq_received)
			break;

		ret = IRQ_HANDLED;

		/* Consume port.  Then clear IIR or we'll miss events */
		if (iir & I915_DISPLAY_PORT_INTERRUPT) {
			u32 hotplug_status = I915_READ(PORT_HOTPLUG_STAT);

			DRM_DEBUG_DRIVER("hotplug event received, stat 0x%08x\n",
				  hotplug_status);
			if (hotplug_status & dev_priv->hotplug_supported_mask)
				queue_work(dev_priv->wq,
					   &dev_priv->hotplug_work);

			I915_WRITE(PORT_HOTPLUG_STAT, hotplug_status);
			I915_READ(PORT_HOTPLUG_STAT);
		}

		I915_WRITE(IIR, iir);
		new_iir = I915_READ(IIR); /* Flush posted writes */

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);
		if (iir & I915_BSD_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[VCS]);

		if (iir & I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT)
			intel_prepare_page_flip(dev, 0);

		if (iir & I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT)
			intel_prepare_page_flip(dev, 1);

		for_each_pipe(pipe) {
			if (pipe_stats[pipe] & PIPE_START_VBLANK_INTERRUPT_STATUS &&
			    drm_handle_vblank(dev, pipe)) {
				i915_pageflip_stall_check(dev, pipe);
				intel_finish_page_flip(dev, pipe);
			}

			if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
				blc_event = true;
		}


		if (blc_event || (iir & I915_ASLE_INTERRUPT))
			intel_opregion_asle_intr(dev);

		/* With MSI, interrupts are only generated when iir
		 * transitions from zero to nonzero.  If another bit got
		 * set while we were handling the existing iir bits, then
		 * we would never get another interrupt.
		 *
		 * This is fine on non-MSI as well, as if we hit this path
		 * we avoid exiting the interrupt handler only to generate
		 * another one.
		 *
		 * Note that for MSI this could cause a stray interrupt report
		 * if an interrupt landed in the time between writing IIR and
		 * the posting read.  This should be rare enough to never
		 * trigger the 99% of 100,000 interrupts test for disabling
		 * stray interrupts.
		 */
		iir = new_iir;
	}

	i915_update_dri1_breadcrumb(dev);

	return ret;
}

static void i965_irq_uninstall(struct drm_device * dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int pipe;

	if (!dev_priv)
		return;

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	I915_WRITE(HWSTAM, 0xffffffff);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe),
			   I915_READ(PIPESTAT(pipe)) & 0x8000ffff);
	I915_WRITE(IIR, I915_READ(IIR));
}

void intel_irq_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	INIT_WORK(&dev_priv->hotplug_work, i915_hotplug_work_func);
	INIT_WORK(&dev_priv->error_work, i915_error_work_func);
	init_waitqueue_head(&dev_priv->error_queue);
	INIT_WORK(&dev_priv->rps.work, gen6_pm_rps_work);
	INIT_WORK(&dev_priv->parity_error_work, ivybridge_parity_work);

	dev->driver->get_vblank_counter = i915_get_vblank_counter;
	dev->max_vblank_count = 0xffffff; /* only 24 bits of frame count */
	if (IS_G4X(dev) || INTEL_INFO(dev)->gen >= 5) {
		dev->max_vblank_count = 0xffffffff; /* full 32 bit counter */
		dev->driver->get_vblank_counter = gm45_get_vblank_counter;
	}

	if (drm_core_check_feature(dev, DRIVER_MODESET))
		dev->driver->get_vblank_timestamp = i915_get_vblank_timestamp;
	else
		dev->driver->get_vblank_timestamp = NULL;
	dev->driver->get_scanout_position = i915_get_crtc_scanoutpos;

	if (IS_VALLEYVIEW(dev)) {
		dev->driver->irq_handler = valleyview_irq_handler;
		dev->driver->irq_preinstall = valleyview_irq_preinstall;
		dev->driver->irq_postinstall = valleyview_irq_postinstall;
		dev->driver->irq_uninstall = valleyview_irq_uninstall;
		dev->driver->enable_vblank = valleyview_enable_vblank;
		dev->driver->disable_vblank = valleyview_disable_vblank;
	} else if (IS_IVYBRIDGE(dev)) {
		/* Share pre & uninstall handlers with ILK/SNB */
		dev->driver->irq_handler = ivybridge_irq_handler;
		dev->driver->irq_preinstall = ironlake_irq_preinstall;
		dev->driver->irq_postinstall = ivybridge_irq_postinstall;
		dev->driver->irq_uninstall = ironlake_irq_uninstall;
		dev->driver->enable_vblank = ivybridge_enable_vblank;
		dev->driver->disable_vblank = ivybridge_disable_vblank;
	} else if (IS_HASWELL(dev)) {
		/* Share interrupts handling with IVB */
		dev->driver->irq_handler = ivybridge_irq_handler;
		dev->driver->irq_preinstall = ironlake_irq_preinstall;
		dev->driver->irq_postinstall = ivybridge_irq_postinstall;
		dev->driver->irq_uninstall = ironlake_irq_uninstall;
		dev->driver->enable_vblank = ivybridge_enable_vblank;
		dev->driver->disable_vblank = ivybridge_disable_vblank;
	} else if (HAS_PCH_SPLIT(dev)) {
		dev->driver->irq_handler = ironlake_irq_handler;
		dev->driver->irq_preinstall = ironlake_irq_preinstall;
		dev->driver->irq_postinstall = ironlake_irq_postinstall;
		dev->driver->irq_uninstall = ironlake_irq_uninstall;
		dev->driver->enable_vblank = ironlake_enable_vblank;
		dev->driver->disable_vblank = ironlake_disable_vblank;
	} else {
		if (INTEL_INFO(dev)->gen == 2) {
			dev->driver->irq_preinstall = i8xx_irq_preinstall;
			dev->driver->irq_postinstall = i8xx_irq_postinstall;
			dev->driver->irq_handler = i8xx_irq_handler;
			dev->driver->irq_uninstall = i8xx_irq_uninstall;
		} else if (INTEL_INFO(dev)->gen == 3) {
			/* IIR "flip pending" means done if this bit is set */
			I915_WRITE(ECOSKPD, _MASKED_BIT_DISABLE(ECO_FLIP_DONE));

			dev->driver->irq_preinstall = i915_irq_preinstall;
			dev->driver->irq_postinstall = i915_irq_postinstall;
			dev->driver->irq_uninstall = i915_irq_uninstall;
			dev->driver->irq_handler = i915_irq_handler;
		} else {
			dev->driver->irq_preinstall = i965_irq_preinstall;
			dev->driver->irq_postinstall = i965_irq_postinstall;
			dev->driver->irq_uninstall = i965_irq_uninstall;
			dev->driver->irq_handler = i965_irq_handler;
		}
		dev->driver->enable_vblank = i915_enable_vblank;
		dev->driver->disable_vblank = i915_disable_vblank;
	}
}
