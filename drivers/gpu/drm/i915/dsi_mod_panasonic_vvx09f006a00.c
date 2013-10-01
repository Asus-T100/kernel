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
#include <asm/intel-mid.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_panasonic_vvx09f006a00.h"

static void  vvx09f006a00_get_panel_info(int pipe,
					struct drm_connector *connector)
{
	if (!connector) {
		DRM_DEBUG_KMS("Panasonic: Invalid input to get_info\n");
		return;
	}

	if (pipe == 0) {
		connector->display_info.width_mm = 192;
		connector->display_info.height_mm = 120;
	}

	return;
}

static void vvx09f006a00_destroy(struct intel_dsi_device *dsi)
{
}

static void vvx09f006a00_dump_regs(struct intel_dsi_device *dsi)
{
}

static void vvx09f006a00_create_resources(struct intel_dsi_device *dsi)
{
}

static struct drm_display_mode *vvx09f006a00_get_modes(
	struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode = NULL;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("Panasonic panel: No memory\n");
		return NULL;
	}

	/* Hardcode 1920x1200 */
	mode->hdisplay = 1920;
	mode->hsync_start = mode->hdisplay + 110;
	mode->hsync_end = mode->hsync_start + 38;
	mode->htotal = mode->hsync_end + 90;

	mode->vdisplay = 1200;
	mode->vsync_start = mode->vdisplay + 15;
	mode->vsync_end = mode->vsync_start + 10;
	mode->vtotal = mode->vsync_end + 10;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	/* Configure */
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}


static bool vvx09f006a00_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static enum drm_connector_status vvx09f006a00_detect(
					struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->is_mipi = true;
	return connector_status_connected;
}

static bool vvx09f006a00_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

static int vvx09f006a00_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void vvx09f006a00_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

bool vvx09f006a00_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

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

	DRM_DEBUG_KMS("Init: Panasonic panel\n");

	if (!dsi) {
		DRM_DEBUG_KMS("Init: Invalid input to panasonic_init\n");
		return false;
	}

	dsi->eotp_pkt = 1;
	dsi->operation_mode = DSI_VIDEO_MODE;
	dsi->video_mode_type = DSI_VIDEO_NBURST_SPULSE;
	dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	dsi->port_bits = 0;
	dsi->turn_arnd_val = 0x14;
	dsi->rst_timer_val = 0xffff;
	dsi->hs_to_lp_count = 0x46;
	dsi->lp_byte_clk = 1;
	dsi->bw_timer = 0x820;
	dsi->clk_lp_to_hs_count = 0xa;
	dsi->clk_hs_to_lp_count = 0x14;
	dsi->video_frmt_cfg_bits = 0;
	dsi->dphy_reg = 0x3c1fc51f;

	dsi->backlight_off_delay = 20;
	dsi->send_shutdown = true;
	dsi->shutdown_pkt_delay = 20;
	dev_priv->mipi.panel_bpp = 24;

	return true;
}


/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops panasonic_vvx09f006a00_dsi_display_ops = {
	.init = vvx09f006a00_init,
	.get_info = vvx09f006a00_get_panel_info,
	.create_resources = vvx09f006a00_create_resources,
	.dpms = vvx09f006a00_dpms,
	.mode_valid = vvx09f006a00_mode_valid,
	.mode_fixup = vvx09f006a00_mode_fixup,
	.detect = vvx09f006a00_detect,
	.get_hw_state = vvx09f006a00_get_hw_state,
	.get_modes = vvx09f006a00_get_modes,
	.destroy = vvx09f006a00_destroy,
	.dump_regs = vvx09f006a00_dump_regs,
};
