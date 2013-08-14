/*
 * Copyright c 2013 Intel Corporation
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
 * Author: Chris Tsai <chrisx.tsai@intel.com>
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
#include "dsi_mod_hv101hd1-1e2.h"
#include <linux/mfd/intel_mid_pmic.h>

static void  hv101hd1_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector) {
		DRM_DEBUG_KMS("Panasonic: Invalid input to get_info\n");
		return;
	}

	if (pipe == 0) {
		connector->display_info.width_mm = 224;
		connector->display_info.height_mm = 126;
	}

	return;
}

static void hv101hd1_msgbus_reset()
{
	u32 msg_bus_port;
	u32 msg_bus_reg;
	u32 val;

	DRM_DEBUG_KMS("\n");

	/* MIPI Escape control register */
	msg_bus_port = 0x14;
	msg_bus_reg = 0x6d;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);

	/* dsi0 control register */
	msg_bus_port = 0x14;
	msg_bus_reg = 0x6f;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);

	/* dsi0 control register */
	msg_bus_port = 0x14;
	msg_bus_reg = 0x6f;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);
}

static void hv101hd1_destroy(struct intel_dsi_device *dsi)
{
}

static void hv101hd1_commit(struct intel_dsi_device *dsi)
{
}

static void hv101hd1_prepare(struct intel_dsi_device *dsi)
{
}

static void hv101hd1_dump_regs(struct intel_dsi_device *dsi)
{
}

static void hv101hd1_mode_set(struct intel_dsi_device *dsi,
		  struct drm_display_mode *mode,
		  struct drm_display_mode *adjusted_mode)
{
}

static void hv101hd1_create_resources(struct intel_dsi_device *dsi)
{
}

static struct drm_display_mode *hv101hd1_get_modes(
	struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode = NULL;

	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("Panasonic panel: No memory\n");
		return NULL;
	}

	/* Hardcode 1368x768 */


	mode->hdisplay = 1368;
	mode->hsync_start = mode->hdisplay + 42;  //front porch
	mode->hsync_end = mode->hsync_start + 20;  //pulse width
	mode->htotal = mode->hsync_end + 26;  // back porch

	mode->vdisplay = 768;
	mode->vsync_start = mode->vdisplay + 13;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 8;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;


	/* Configure */
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}


static bool hv101hd1_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static enum drm_connector_status hv101hd1_detect(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->is_mipi = true;
	return connector_status_connected;
}

static bool hv101hd1_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

static int hv101hd1_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void hv101hd1_dpms(struct intel_dsi_device *dsi, bool enable)
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

bool hv101hd1_init(struct intel_dsi_device *dsi)
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

	DRM_DEBUG_KMS("Init: Panasonic panel\n");

	if (!dsi) {
		DRM_DEBUG_KMS("Init: Invalid input to panasonic_init\n");
		return false;
	}

	dsi->eotp_pkt = 1;
	dsi->operation_mode = DSI_VIDEO_MODE;
	dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	dsi->port_bits = 0;
	dsi->turn_arnd_val = 0x14;
	dsi->rst_timer_val = 0xff;
	dsi->hs_to_lp_count = 0x46;
	dsi->lp_byte_clk = 0x3;
	dsi->bw_timer = 0;
	dsi->clk_lp_to_hs_count = 0x1F;
	dsi->clk_hs_to_lp_count = 0x0D;
	dsi->video_frmt_cfg_bits = 0;
	dsi->dphy_reg = 0x360D360B;

	/* Program MIPI reset */
	hv101hd1_msgbus_reset();

	intel_mid_pmic_writeb(0x52,1);//PANEL_EN
	intel_mid_pmic_writeb(0x51,1);//BACKLIGHT_EN

	return true;
}


/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops hv101hd1_dsi_display_ops = {
	.init = hv101hd1_init,
	.get_info = hv101hd1_get_panel_info,
	.create_resources = hv101hd1_create_resources,
	.dpms = hv101hd1_dpms,
	.mode_valid = hv101hd1_mode_valid,
	.mode_fixup = hv101hd1_mode_fixup,
	.prepare = hv101hd1_prepare,
	.commit = hv101hd1_commit,
	.mode_set = hv101hd1_mode_set,
	.detect = hv101hd1_detect,
	.get_hw_state = hv101hd1_get_hw_state,
	.get_modes = hv101hd1_get_modes,
	.destroy = hv101hd1_destroy,
	.dump_regs = hv101hd1_dump_regs,
};