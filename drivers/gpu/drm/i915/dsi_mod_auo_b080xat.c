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
#include "dsi_mod_auo_b080xat.h"


static void b080xat_get_panel_info(int pipe, struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = B080XAT_10x7_PANEL_WIDTH;
		connector->display_info.height_mm = B080XAT_10x7_PANEL_HEIGHT;
	}

	return;
}

bool b080xat_init(struct intel_dsi_device *dsi)
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
	dsi->video_mode_type = DSI_VIDEO_BURST | IP_TG_CONFIG;
	dsi->pixel_format = VID_MODE_FORMAT_RGB666_LOOSE;
	dsi->port_bits = BANDGAP_LNC_CIRCUIT | LP_OUTPUT_HOLD;
	dsi->turn_arnd_val = 0x3f;
	dsi->rst_timer_val = 0xff;
	dsi->hs_to_lp_count = 0x1b;
	dsi->lp_byte_clk = 4;
	dsi->bw_timer = 0;
	dsi->clk_lp_to_hs_count = 0x1b;
	dsi->clk_hs_to_lp_count = 0x0c;
	dsi->video_frmt_cfg_bits = IP_TG_CONFIG;
	dsi->dphy_reg = 0x1B104315;

	return true;
}

void b080xat_create_resources(struct intel_dsi_device *dsi) { }

void b080xat_dpms(struct intel_dsi_device *dsi, bool enable)
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

int b080xat_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool b080xat_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void b080xat_prepare(struct intel_dsi_device *dsi) { }

void b080xat_commit(struct intel_dsi_device *dsi)
{
}

void b080xat_mode_set(struct intel_dsi_device *dsi,
		  struct drm_display_mode *mode,
		  struct drm_display_mode *adjusted_mode)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* No other better place to do the panel reset */
	intel_gpio_nc_write32(dev_priv, 0x4160, 0x2000CC00);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000004);
	mdelay(10);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000005);
	mdelay(20);
}

enum drm_connector_status b080xat_detect(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->is_mipi = true;

	return connector_status_connected;
}

bool b080xat_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *b080xat_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 768;
	mode->hsync_start = 828;
	mode->hsync_end = 892;
	mode->htotal = 948;

	mode->vdisplay = 1024;
	mode->vsync_start = 1060;
	mode->vsync_end = 1110;
	mode->vtotal = 1140;

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void b080xat_dump_regs(struct intel_dsi_device *dsi) { }

void b080xat_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops auo_b080xat_dsi_display_ops = {
	.init = b080xat_init,
	.get_info = b080xat_get_panel_info,
	.create_resources = b080xat_create_resources,
	.dpms = b080xat_dpms,
	.mode_valid = b080xat_mode_valid,
	.mode_fixup = b080xat_mode_fixup,
	.prepare = b080xat_prepare,
	.commit = b080xat_commit,
	.mode_set = b080xat_mode_set,
	.detect = b080xat_detect,
	.get_hw_state = b080xat_get_hw_state,
	.get_modes = b080xat_get_modes,
	.destroy = b080xat_destroy,
	.dump_regs = b080xat_dump_regs,
};
