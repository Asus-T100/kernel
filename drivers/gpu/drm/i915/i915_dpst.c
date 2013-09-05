/*
 * Copyright © 2008 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 *
 */

#include <linux/sysrq.h>
#include <linux/slab.h>
#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "drm_crtc.h"
#include "i915_drv.h"
#include "intel_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"

int i915_dpst_context(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct dpst_initialize_context	*init_context	=	NULL;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 btgr_data = 0;
	u32 hcr_data = 0;
	u32 bdr_data = 0;
	u32 bpcr_data = 0;

	init_context = (struct dpst_initialize_context *) data;
	switch (init_context->dpst_ioctl_type) {
	case DPST_ENABLE:
		i915_dpst_enable_hist_interrupt(dev, true);
	break;

	case DPST_DISABLE:
		i915_dpst_enable_hist_interrupt(dev, false);
	break;

	case DPST_INIT_DATA:
	{
		int pipe = 0;
		u32 gb_val = 0;
		struct drm_crtc *crtc;
		struct drm_display_mode *mode = NULL;

		DRM_DEBUG_DRIVER("init_context - INIT_DATA case ..\n");
		dev_priv->is_dpst_enabled = true ;
		dev_priv->dpst_backlight_factor = DPST_MAX_FACTOR;

		crtc = intel_get_crtc_for_pipe(dev, pipe);
		if (crtc) {
			mode = intel_crtc_mode_get(dev, crtc);
			if (mode) {
				gb_val = (DEFAULT_GUARDBAND_VAL *
					mode->hdisplay * mode->vdisplay)/1000;
				if (gb_val !=
					init_context->init_data.threshold_gb) {
					init_context->init_data.threshold_gb =
						gb_val;
				}
				init_context->init_data.image_res =
						mode->hdisplay*mode->vdisplay;
			}
		}
		btgr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG);
		btgr_data |=  DPST_BTGR_HIST_ENABLE | DPST_BTGR_INT_STATUS
				| (init_context->init_data.gb_delay << 22)
				| init_context->init_data.threshold_gb;

		hcr_data  = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		hcr_data |=
			DPST_IEHCR_HIST_ENABLE | DPST_IEHCR_HIST_MODE_SELECT;

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);

		/* Wait for VBLANK */
		intel_wait_for_vblank(dev, 0);

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);

		dev_priv->dpst_task = current;
		dev_priv->dpst_signal = init_context->init_data.sig_num;
		dev_priv->dpst_backlight_factor = DPST_MAX_FACTOR;

		/* enable pipe interrupt */
		i915_enable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);
	}
	break;

	case DPST_GET_BIN_DATA:
	{
		int index;
		DRM_DEBUG_DRIVER("init_context - GET_BIN_DATA case ..\n");

		drm_i915_private_t *dev_priv =
				(drm_i915_private_t *)dev->dev_private;

		/* Read Image Enhancement Histogram Control Register */
		hcr_data  = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		hcr_data = hcr_data & ~(DPST_BIN_REG_INDEX_MASK
			| DPST_BIN_REG_FUNC_SELECT_MASK);
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);

		for (index = 0; index < DPST_BIN_COUNT; index++) {
			bdr_data =
			I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEBDR_REG);

			if (!(bdr_data & IEBDR_BUSY_BIT)) {
				init_context->hist_status.histogram_bins.
					status[index] =	bdr_data &
						DPST_SEGVALUE_MAX_22_BIT;
#ifdef CONFIG_DEBUG_FS
				dev_priv->dpst.bin_data[index] = bdr_data;
#endif
			} else {
				/* Engine is busy. Reset index to 0 to grab
				 * fresh histogram data */
				DRM_DEBUG_DRIVER("busy bit set .....\n");
				index = -1;
				hcr_data = 0;

				/* Read Image Enhancement Histogram
				 * Control Register */
				hcr_data =
					I915_READ(VLV_DISPLAY_BASE +
						DPST_VLV_IEHCR_REG);

				/* Reset the Bin Register index to 0, which
				 * ensures that we start reading from Bin 0 */
				hcr_data = hcr_data &
						~(DPST_BIN_REG_INDEX_MASK);
				I915_WRITE(VLV_DISPLAY_BASE +
						DPST_VLV_IEHCR_REG, hcr_data);
			}
		}
	}
	break;

	case DPST_APPLY_LUMA:
	{
		u32 diet_factor, i, temp = 0, temp2 = 0;
		DRM_DEBUG_DRIVER("init_context - APPLY_LUMA case ..\n");

		if (!dev_priv->is_dpst_enabled)
			return -EINVAL;

		/* Backlight settings */
		dev_priv->dpst_backlight_factor =
		init_context->ie_container.dpst_blc_factor;

		i915_dpst_set_brightness(dev, dev_priv->backlight_level);

		temp = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		temp2 = DPST_BIN_REG_FUNC_SELECT_MASK |
						DPST_BIN_REG_INDEX_MASK;
		hcr_data =
			((temp & ~(temp2)) | (DPST_BIN_REG_FUNC_SELECT_MASK));

		/* The DIET data format = 10 bits with upper limit mapped to
		 * 1.9 multiplier */
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);

		for (i = 0; i < DPST_DIET_ENTRY_COUNT; i++) {
			diet_factor = init_context->ie_container.
				dpst_ie_st.factor_present[i] * 0x200 / 10000;
			/* Write 10 bit value to Image Enhancement Bin Data
			 * register */
			I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEBDR_REG,
					diet_factor);
#ifdef CONFIG_DEBUG_FS
			dev_priv->dpst.luma_data[i] = diet_factor;
#endif
		}
		/* reset the mask */
		temp = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		temp2 = DPST_MOD_TBL_ENABLE_MASK | DPST_ALT_ENHANCE_MOD_MASK;
		hcr_data = (temp & ~(temp2)) | DPST_RESET_IE;

		/* The DIET data format = 10 bits with upper limit mapped to
		 * 1.9 multiplier */
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);

		temp = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
	}
	break;

	case DPST_RESET_HISTOGRAM_STATUS:
		DRM_DEBUG_DRIVER("init_context-DPST_RESET_HISTOGRAM..\n");
		i915_reset_histogram(dev);
	break;

	default:
		DRM_ERROR("init_context - default case ..\n");
		return -EINVAL;
	break;
	}
	return 0;
}

int i915_reset_histogram(struct drm_device *dev)
{
	u32 btgr_data = 0;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	btgr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG);

	if (btgr_data & DPST_BTGR_INT_STATUS) {
		DRM_DEBUG_DRIVER("clearing btgr\n");
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);
	}
	return 0;
}

int i915_dpst_enable_hist_interrupt(struct drm_device *dev, bool enable)
{
	u32 btgr_data = 0;
	u32 hcr_data = 0;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (enable) {
		dev_priv->is_dpst_enabled = true;
		dev_priv->dpst_backlight_factor = DPST_MAX_FACTOR ;
		/* Read the Histogram Threshold Guardband register */
		btgr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG);

		/* Set the Interrupt Enable bitand clear pending interrupts */
		btgr_data |= DPST_BTGR_HIST_ENABLE | DPST_BTGR_INT_STATUS ;

		/* Write to the Histogram Threshold Guardband Register to enable
		 * histogram interrupts */
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);
		/* Write to the Image Enhancement Histogram Control Register */
		hcr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		hcr_data |= DPST_IEHCR_HIST_ENABLE |
					DPST_IEHCR_HIST_MODE_SELECT;

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);
		/* Wait for VBLANK */
		intel_wait_for_vblank(dev, 0);

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);

		i915_enable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);
	} else {
		dev_priv->is_dpst_enabled = false;
		dev_priv->dpst_backlight_factor = DPST_MAX_FACTOR;
		/* Read the Histogram Threshold Guardband register */
		btgr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG);
		/* Set the Interrupt Enable and clear pending interrupts */
		btgr_data &=
			~(DPST_BTGR_HIST_ENABLE | DPST_BTGR_INT_STATUS);

		/* Write to the Histogram Threshold Guardband Register to
		 * disable histogram interrupts */
		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);

		/* Write to the Image Enhancement Histogram Control Register */
		hcr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
		hcr_data &=
		~(DPST_IEHCR_HIST_ENABLE | DPST_IEHCR_HIST_MODE_SELECT);

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG, hcr_data);
		/* Wait for VBLANK */
		intel_wait_for_vblank(dev, 0);

		I915_WRITE(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG, btgr_data);
		/* setting user blc level */
		intel_panel_actually_set_backlight(dev,
						dev_priv->backlight_level);
		i915_disable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);
	}
	return 0;
}

u32 i915_dpst_get_brightness(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	if (!dev_priv->is_dpst_enabled)
		return 0;

	return dev_priv->backlight_level;
}
void i915_dpst_set_brightness(struct drm_device *dev, u32 brightness_val)
{
	u32 backlight_level = brightness_val;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	backlight_level = ((brightness_val *
				dev_priv->dpst_backlight_factor)/100)/100;

	DRM_DEBUG_DRIVER("user_level 0x%x dpst_level = 0x%x\n",
				dev_priv->backlight_level, backlight_level);

	intel_panel_actually_set_backlight(dev, backlight_level);
	return 0;
}
