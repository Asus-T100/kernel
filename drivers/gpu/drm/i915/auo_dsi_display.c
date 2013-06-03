/*
 * Copyright © 2013 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"

#include "auo_dsi_display.h"

void  auo_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = 216;
		connector->display_info.height_mm = 135;
	}

	return;
}

bool auo_init(struct intel_dsi_device *dsi)
{
	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */
	DRM_DEBUG_KMS("\n");

	dsi->eotp_pkt = 1;
	dsi->operation_mode = DSI_VIDEO_MODE;
	dsi->video_mode_type = DSI_VIDEO_NBURST_SPULSE;
	dsi->pixel_format = VID_MODE_FORMAT_RGB888;

	return true;
}

void auo_create_resources(struct intel_dsi_device *dsi) { }

void auo_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	if (enable) {
		int i;

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

int auo_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool auo_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode)
{
	return true;
}

void auo_prepare(struct intel_dsi_device *dsi) { }

void auo_commit(struct intel_dsi_device *dsi) { }

void auo_mode_set(struct intel_dsi_device *dsi,
		  struct drm_display_mode *mode,
		  struct drm_display_mode *adjusted_mode) { }

enum drm_connector_status auo_detect(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->is_mipi = true;
	return connector_status_connected;
}

bool auo_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *auo_get_modes(struct intel_dsi_device *dsi)
{
	u32 hblank = 0x78;
	u32 vblank = 0x0C;
	u32 hsync_offset = 0x28;
	u32 hsync_width  = 0x28;
	u32 vsync_offset = 0x4;
	u32 vsync_width  = 0x4;
	struct drm_display_mode *mode = NULL;
	struct drm_i915_private *dev_priv = dsi->dev_priv;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("AUO Panel: No memory\n");
		return NULL;
	}

	/* Hardcode 1920x1200*/
	strcpy(mode->name, "1920x1200");
	mode->hdisplay = 0x780;
	mode->vdisplay = 0x4B0;
	mode->vrefresh = 60;
	mode->clock =  148350;

	/* Calculate */
	mode->hsync_start = mode->hdisplay + hsync_offset;
	mode->hsync_end = mode->hdisplay + hsync_offset
		+ hsync_width;
	mode->htotal = mode->hdisplay + hblank;
	mode->vsync_start = mode->vdisplay + vsync_offset;
	mode->vsync_end = mode->vdisplay + vsync_offset
		+ vsync_width;
	mode->vtotal = mode->vdisplay + vblank;

	/* Configure */
	mode->flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	mode->status = MODE_OK;

	return mode;
}

void auo_dump_regs(struct intel_dsi_device *dsi) { }

void auo_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops auo_dsi_display_ops = {
	.init = auo_init,
	.get_info = auo_vid_get_panel_info,
	.create_resources = auo_create_resources,
	.dpms = auo_dpms,
	.mode_valid = auo_mode_valid,
	.mode_fixup = auo_mode_fixup,
	.prepare = auo_prepare,
	.commit = auo_commit,
	.mode_set = auo_mode_set,
	.detect = auo_detect,
	.get_hw_state = auo_get_hw_state,
	.get_modes = auo_get_modes,
	.destroy = auo_destroy,
	.dump_regs = auo_dump_regs,
};
