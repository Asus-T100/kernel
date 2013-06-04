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
 *	 : Shobhit Kumar <shobhit.kumar@intel.com>
 *	 : Yogesh Mohan Marimuthu <yogesh.mohan.marimuthu@intel.com> */

/**
 * This file should lay the groundwork for supporting DSI video and command mode
 * panels. In the short term, this could be hard-coded to support some panel. In
 * the long term, this should use a framework (such as the Common Display
 * Framework, CDF) to control the panel specific functionality. Thus this file
 * should be the MIPI DSI IP block driver, while panel specific stuff should be
 * elsewhere.
 *
 * It should probably be possible to initialize the DSI output without the panel
 * driver being probed. In this case, the output should be disconnected until
 * the panel driver is loaded. This has a certain resemblance of hotplug,
 * although the panel is fixed in the assembly.
 *
 * The the CLV driver, panel specific stuff is, at least in theory, in separate
 * files, and called through callbacks (struct panel_funcs *). This should be
 * replaced with CDF.
 *
 * We should provide a framework to send DSI commands, once the rest is in
 * place.
 *
 * Make the whole shebang pipe aware. There are two DSI pipes available. Lane
 * configuration between pipes needs to be sorted out.
 *
 * Panel/display self refresh for command mode.
 *
 * ACPI/BIOS/Firmware provided info:
 *
 * - connected pipe(s), number of DSI panels
 * - lane configuration between pipes
 * - resolution and timings for each panel
 * - connector->display_info.{width,height}_mm
 * - video mode vs. command mode for each panel
 *
 * Common Display Framework
 * ========================
 *
 * i915 gets probed first, panel drivers later. AFAICT we need to create DSI/DPI
 * encoders/connectors for all possible combinations the hw supports in advance,
 * and leave them disconnected until the panel driver is probed. This seems like
 * hotplug, even though the panels are fixed in reality.
 *
 * The panel drivers themselves need to get display properties from ACPI or
 * board files or whatever.
 *
 * One video source per encoder/connector, or many video sources per
 * encoder/connector? How many display entities per video source? Can the video
 * source limit display entities it supports?
 *
 * Figure Out
 * ==========
 *
 * How to configure lanes between pipes? Should i915 be dynamic and let panel
 * drivers allocate what they want based on BIOS info, or should k915 configure
 * stuff between panel drivers?
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include "linux/mfd/intel_mid_pmic.h"
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "intel_dsi_pll.h"

/* the panel drivers are here */
static const struct intel_dsi_device intel_dsi_devices[] = {
	{
		.panel_id = MIPI_DSI_CMI_PANEL_ID,
		.type = INTEL_DSI_COMMAND_MODE,
		.name = "cmi-dsi-cmd-mode-display",
		.dev_ops = &cmi_dsi_display_ops,
		.lane_count = 4, /* XXX: this really doesn't belong here */
	},
	{
		.panel_id = MIPI_DSI_CMI_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "cmi-dsi-vid-mode-display",
		.dev_ops = &cmi_dsi_display_ops,
		.lane_count = 4, /* XXX: this really doesn't belong here */
	},
	{
		.panel_id = MIPI_DSI_AUO_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-dsi-vid-mode-display",
		.dev_ops = &auo_dsi_display_ops,
		.lane_count = 4, /* XXX: this really doesn't belong here */
	},
};

static struct intel_dsi *intel_attached_dsi(struct drm_connector *connector)
{
	return container_of(intel_attached_encoder(connector),
			    struct intel_dsi, base);
}

static bool intel_dsi_connector_get_hw_state(struct intel_connector
		*connector)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(&connector->base);

	return intel_dsi->dev.dev_ops->get_hw_state(&intel_dsi->dev);
}

static bool intel_dsi_get_hw_state(struct intel_encoder *encoder,
				    enum pipe *pipe)
{
	DRM_DEBUG_KMS("\n");
	return true;
}

static void intel_dsi_pre_pll_enable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

static void intel_dsi_pre_enable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

void intel_dsi_enable(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 temp, status;

	DRM_DEBUG_KMS("\n");

	/* Device Ready */
	temp = I915_READ(MIPI_DEVICE_READY(pipe));
	temp &= ~ULPS_STATE_MASK;
	temp &= ~DEVICE_READY; /* XXX */
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp | ULPS_STATE_EXIT);
	msleep(20);
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp);

	temp = I915_READ(MIPI_PORT_CTRL(pipe));
	temp = temp | intel_dsi->dev.port_bits;
	I915_WRITE(MIPI_PORT_CTRL(pipe), temp | DPI_ENABLE);
	POSTING_READ(MIPI_PORT_CTRL(pipe));

	/* XXX: Placement of this? */
	temp = I915_READ(MIPI_DEVICE_READY(pipe));
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp | DEVICE_READY);


	/* XXX: Enable DPMS later */
	/*intel_dsi->dev.dev_ops->dpms(&intel_dsi->dev, true); */

	/* XXX: fix the bits with constants */
	I915_WRITE(MIPI_DPI_CONTROL(pipe), ((0x1 << 1) &
			~(0x1 << 0) & ~(0x1 << 6)));
	I915_WRITE(MIPI_DPI_CONTROL(pipe), 0x2);

	/* Wait till SPL Packet Sent status bit is not set */
	if (wait_for(I915_READ(MIPI_INTR_STAT(pipe)) &
			SPL_PKT_SENT_INTERRUPT, 50))
		DRM_DEBUG_KMS("SPL Packet Sent failed\n");
}

/* XXX: We need to call this from crtc disable sequence ??? */
static void intel_dsi_disable(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 temp;

	DRM_DEBUG_KMS("\n");

	/* XXX: Check if we have to do this
	I915_WRITE_BITS(0x61204, 0, 0x00000001);
	while (I915_READ(0x61200) & 0xB0000000) {};
	*/

	intel_dsi->dev.dev_ops->dpms(&intel_dsi->dev, false);

	/* lanes to ulps in MIPI_DEVICE_READY */
	temp = I915_READ(MIPI_DEVICE_READY(pipe));
	temp &= ~ULPS_STATE_MASK;
	temp &= ~DEVICE_READY;
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp | ULPS_STATE_ENTER);

	/* XXX: port ctrl is a mess */
	temp = I915_READ(MIPI_PORT_CTRL(pipe));
	I915_WRITE(MIPI_PORT_CTRL(pipe), temp & ~DPI_ENABLE);
	POSTING_READ(MIPI_PORT_CTRL(pipe));

	/* dpi: SHUTDOWN in MIPI_DPI_CONTROL through dpi_send_cmd */

	/* device ready state off */
}

static void intel_dsi_post_disable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

/* this is basically intel_connector_dpms and intel_encoder_dpms
 * combined, with the mipi dev specific call as well
 *
 */
static void intel_dsi_dpms(struct drm_connector *connector, int mode)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_crtc *crtc;

	DRM_DEBUG_KMS("mode %d\n", mode);

	/* mipi supports only 2 dpms states. */
	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	if (mode == connector->dpms)
		return;

	connector->dpms = mode;

	/* Only need to change hw state when actually enabled */
	crtc = intel_dsi->base.base.crtc;
	if (!crtc) {
		/*intel_dsi->base.connectors_active = false;*/
		return;
	}

	if (mode == DRM_MODE_DPMS_ON) {
		/*intel_dsi->base.connectors_active = true;*/

		/* calls crtc_enable/disable:
		 * - intel_enable_pll
		 * - encoder->pre_enable
		 *	=> enable dsi pll
		 * - intel_enable_pipe
		 * - intel_enable_plane
		 * - intel_crtc_load_lut
		 * - intel_update_fbc
		 * - intel_crtc_dpms_overlay
		 * - intel_crtc_update_cursor
		 * - encoder->enable
		 *
		 */
		/*intel_crtc_update_dpms(crtc);*/

		intel_dsi->dev.dev_ops->dpms(&intel_dsi->dev, true);
	} else {
		intel_dsi->dev.dev_ops->dpms(&intel_dsi->dev, false);

		/*intel_dsi->base.connectors_active = false;*/

		/* calls crtc_enable/disable:
		 * - encoder->disable

		 * - ...
		 * - intel_disable_plane
		 * - intel_disable_pipe
		 * - encoder->post_disable
		 * - intel_disable_pll
		 * -
		 */
		/*intel_crtc_update_dpms(crtc);*/
	}

	/*intel_modeset_check_state(connector->dev);*/
}

static int intel_dsi_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);

	DRM_DEBUG_KMS("\n");

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	/* XXX: Validate clock range */
	/* XXX: fixed mode */
	if (intel_dsi->panel_fixed_mode) {
		if (mode->hdisplay > intel_dsi->panel_fixed_mode->hdisplay)
			return MODE_PANEL;
		if (mode->vdisplay > intel_dsi->panel_fixed_mode->vdisplay)
			return MODE_PANEL;
	}

	return intel_dsi->dev.dev_ops->mode_valid(&intel_dsi->dev, mode);\
}

static bool intel_dsi_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);

	DRM_DEBUG_KMS("\n");

	/* If we have timings from the BIOS for the panel, put them in
	 * to the adjusted mode.  The CRTC will be set up for this mode,
	 * with the panel scaling set up to source from the H/VDisplay
	 * of the original mode.
	 */
	if (intel_dsi->panel_fixed_mode != NULL) {
#define C(x) (adjusted_mode->x = intel_dsi->panel_fixed_mode->x)
		C(hdisplay);
		C(hsync_start);
		C(hsync_end);
		C(htotal);
		C(vdisplay);
		C(vsync_start);
		C(vsync_end);
		C(vtotal);
		C(clock);
#undef C
	}

	if (intel_dsi->dev.dev_ops->mode_fixup)
		return intel_dsi->dev.dev_ops->mode_fixup(
			&intel_dsi->dev, mode, adjusted_mode);

	return true;
}

static void intel_dsi_mode_prepare(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

static void intel_dsi_commit(struct drm_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

/* return pixels in terms of txbyteclkhs */
static u32 txbyteclkhs(u32 pixels, int bpp, int lane_count)
{
	u32 pixel_bytes;
	pixel_bytes =  ((pixels * bpp) / 8) + (((pixels * bpp) % 8) && 1);
	return (pixel_bytes / lane_count) + ((pixel_bytes % lane_count) && 1);
}

static void set_dsi_timings(struct drm_encoder *encoder,
			     struct drm_display_mode *mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->bpp;
	unsigned int lane_count = intel_dsi->dev.lane_count;

	u16 hactive, hfp, hsync, hbp, vfp, vsync, vbp;

	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	/* horizontal values are in terms of high speed byte clock */
	hactive = txbyteclkhs(hactive, bpp, lane_count);
	hfp = txbyteclkhs(hfp, bpp, lane_count);
	hsync = txbyteclkhs(hsync, bpp, lane_count);
	hbp = txbyteclkhs(hbp, bpp, lane_count);

	I915_WRITE(MIPI_HACTIVE_AREA_COUNT(pipe), hactive);
	I915_WRITE(MIPI_HFP_COUNT(pipe), hfp);
	I915_WRITE(MIPI_HSYNC_PADDING_COUNT(pipe), hsync);
	I915_WRITE(MIPI_HBP_COUNT(pipe), hbp);

	/* vertical values are in terms of lines */
	I915_WRITE(MIPI_VFP_COUNT(pipe), vfp);
	I915_WRITE(MIPI_VSYNC_PADDING_COUNT(pipe), vsync);
	I915_WRITE(MIPI_VBP_COUNT(pipe), vbp);

	DRM_DEBUG_KMS("lane_count = %0d bpp = %0d\n", lane_count, bpp);
	DRM_DEBUG_KMS("MIPI_HACTIVE_AREA_COUNT %0x = %0x\n",
			MIPI_HACTIVE_AREA_COUNT(pipe),
			I915_READ(MIPI_HACTIVE_AREA_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_HFP_COUNT %0x = %0x\n",
			MIPI_HFP_COUNT(pipe), I915_READ(MIPI_HFP_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_HSYNC_PADDING_COUNT %0x = %0x\n",
			MIPI_HSYNC_PADDING_COUNT(pipe),
			I915_READ(MIPI_HSYNC_PADDING_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_HBP_COUNT %0x = %0x\n",
			MIPI_HBP_COUNT(pipe), I915_READ(MIPI_HBP_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_VFP_COUNT %0x = %0x\n",
			MIPI_VFP_COUNT(pipe), I915_READ(MIPI_VFP_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_VSYNC_PADDING_COUNT %0x = %0x\n",
			MIPI_VSYNC_PADDING_COUNT(pipe),
			I915_READ(MIPI_VSYNC_PADDING_COUNT(pipe)));
	DRM_DEBUG_KMS("MIPI_VBP_COUNT %0x = %0x\n",
			MIPI_VBP_COUNT(pipe), I915_READ(MIPI_VBP_COUNT(pipe)));
}


/* this is called for each encoder after i9xx_crtc_mode_set, from
 * intel_crtc_mode_set.
 */
static void intel_dsi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	int plane = intel_crtc->plane;
	unsigned int bpp = intel_crtc->bpp;
	u32 val;

	DRM_DEBUG_KMS("\n");

	/* Enable bandgap fix */
	intel_cck_write32_bits(dev_priv, 0x6D, 0x00010000, 0x00030000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x6E, 0x00010000, 0x00030000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x6F, 0x00010000, 0x00030000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x00, 0x00008000, 0x00008000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x00, 0x00000000, 0x00008000);
	msleep(20);

	/* Turn Display Trunk on */
	intel_cck_write32_bits(dev_priv, 0x6B, 0x00020000, 0x00030000);
	msleep(20);

	intel_cck_write32_bits(dev_priv, 0x6C, 0x00020000, 0x00030000);
	msleep(20);

	intel_cck_write32_bits(dev_priv, 0x6D, 0x00020000, 0x00030000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x6E, 0x00020000, 0x00030000);
	msleep(20);
	intel_cck_write32_bits(dev_priv, 0x6F, 0x00020000, 0x00030000);

	/* Need huge delay, otherwise clock is not stable */
	msleep(100);

	/* MIPI PORT Control register */
	I915_WRITE(0x61190, 0x80010000);

#ifdef CONFIG_CRYSTAL_COVE
	/* Crystal cove PMIC is not there in Baytrail-M
	 * XXX:
	 * We need to fix for Backlight on Baytrail-M when
	 * we start work on that
	 */

	/* enable panel backlight and pwm*/
	/* need to do this as per panel enabling sequence */
	intel_mid_pmic_writeb(0x4B, 0xFF);
	intel_mid_pmic_writeb(0x4E, 0xFF);
	intel_mid_pmic_writeb(0x51, 0x01);
	intel_mid_pmic_writeb(0x52, 0x01);
#endif

	dsi_config(encoder);

	I915_WRITE(MIPI_DPI_RESOLUTION(pipe),
		   adjusted_mode->vdisplay << VERTICAL_ADDRESS_SHIFT |
		   adjusted_mode->hdisplay << HORIZONTAL_ADDRESS_SHIFT);

	set_dsi_timings(encoder, adjusted_mode);

	val = intel_dsi->channel << VID_MODE_CHANNEL_NUMBER_SHIFT |
			intel_dsi->dev.lane_count  << DATA_LANES_PRG_REG_SHIFT |
			intel_dsi->dev.pixel_format;
	I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

	if ((intel_dsi->dev.operation_mode == DSI_VIDEO_MODE) && \
			(intel_dsi->dev.video_mode_type == DSI_VIDEO_BURST))
		I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
			txbyteclkhs(adjusted_mode->htotal + 1, bpp,
			intel_dsi->dev.lane_count));
	else
		I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
		   txbyteclkhs(adjusted_mode->vtotal * adjusted_mode->htotal,
			       bpp, intel_dsi->dev.lane_count));

	I915_WRITE(MIPI_LP_RX_TIMEOUT(pipe), 0xffff);
	I915_WRITE(MIPI_TURN_AROUND_TIMEOUT(pipe),
					intel_dsi->dev.turn_arnd_val);
	I915_WRITE(MIPI_DEVICE_RESET_TIMER(pipe),
					intel_dsi->dev.rst_timer_val);
	/* in terms of low power clock */
	I915_WRITE(MIPI_INIT_COUNT(pipe), 0x7d0);

	if (intel_dsi->dev.eotp_pkt)
		I915_WRITE(MIPI_EOT_DISABLE(pipe), 0);
	else
		I915_WRITE(MIPI_EOT_DISABLE(pipe), 1);

	I915_WRITE(MIPI_HIGH_LOW_SWITCH_COUNT(pipe), \
					intel_dsi->dev.hs_to_lp_count);
	I915_WRITE(MIPI_LP_BYTECLK(pipe), intel_dsi->dev.lp_byte_clk);
	I915_WRITE(MIPI_DBI_BW_CTRL(pipe), intel_dsi->dev.bw_timer);

	I915_WRITE(MIPI_CLK_LANE_SWITCH_TIME_CNT(pipe),
		((u32)intel_dsi->dev.clk_lp_to_hs_count
		<< LP_HS_SSW_CNT_SHIFT) |
		(intel_dsi->dev.clk_hs_to_lp_count << HS_LP_PWR_SW_CNT_SHIFT));

	if ((intel_dsi->dev.operation_mode == DSI_VIDEO_MODE) && \
			(intel_dsi->dev.video_mode_type ==
					DSI_VIDEO_NBURST_SPULSE))
		I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe),
				intel_dsi->dev.video_frmt_cfg_bits |
				VIDEO_MODE_NON_BURST_WITH_SYNC_PULSE);
	else if ((intel_dsi->dev.operation_mode == DSI_VIDEO_MODE) &&	\
			(intel_dsi->dev.video_mode_type ==
					DSI_VIDEO_NBURST_SEVENT))
		I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe),
				intel_dsi->dev.video_frmt_cfg_bits |
				VIDEO_MODE_NON_BURST_WITH_SYNC_EVENTS);
	else
		I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe),
				intel_dsi->dev.video_frmt_cfg_bits |
				VIDEO_MODE_BURST);
}

static enum drm_connector_status
intel_dsi_detect(struct drm_connector *connector, bool force)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	DRM_DEBUG_KMS("\n");

	/* Either eDP or MIPI will be there, so if eDP detect
	 * assume no MIPI
	 * TBD: Fix proper MIPI detection logic
	 */
	if (dev_priv->is_edp) {
		dev_priv->is_mipi = false;
		return connector_status_disconnected;
	} else
		return intel_dsi->dev.dev_ops->detect(&intel_dsi->dev);
}

static int intel_dsi_get_modes(struct drm_connector *connector)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct drm_display_mode *mode = NULL;

	/* Get the mode info from panel specific callback */
	intel_dsi->panel_fixed_mode =
		intel_dsi->dev.dev_ops->get_modes(&intel_dsi->dev);
	if (!intel_dsi->panel_fixed_mode) {
		DRM_ERROR("out of memory for fixed panel mode\n");
		return 0;
	}

	mode = drm_mode_duplicate(connector->dev, intel_dsi->panel_fixed_mode);
	if (!mode)
		return 0;

	mode->status = MODE_OK;

	/* Add this mode in probed mode list */
	drm_mode_probed_add(connector, mode);
	DRM_DEBUG_KMS("Mode read done\n");

	/* Fill the panel info here */
	intel_dsi->dev.dev_ops->get_info(0, connector);

	return 1;
}

static void intel_dsi_destroy(struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static int intel_dsi_set_property(struct drm_connector *connector,
		struct drm_property *property,
		uint64_t value)
{
	DRM_DEBUG_KMS("\n");
	return 0;
}

static const struct drm_encoder_helper_funcs intel_dsi_helper_funcs = {
	.mode_fixup = intel_dsi_mode_fixup,
	.mode_set = intel_dsi_mode_set,
	.disable = intel_encoder_noop,
	.prepare = intel_dsi_mode_prepare,
	.commit = intel_dsi_commit,
};

static const struct drm_encoder_funcs intel_dsi_funcs = {
	.destroy = intel_encoder_destroy,
};

static const struct drm_connector_helper_funcs
	intel_dsi_connector_helper_funcs = {
	.get_modes = intel_dsi_get_modes,
	.mode_valid = intel_dsi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_connector_funcs intel_dsi_connector_funcs = {
	.dpms = intel_dsi_dpms,
	.detect = intel_dsi_detect,
	.destroy = intel_dsi_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
};

static void dsi_config(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->bpp;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	/* escape clock divider, 20MHz, shared for A and C. device ready must be
	 * off when doing this! txclkesc? */
	tmp = I915_READ(MIPI_CTRL(0));
	tmp &= ~ESCAPE_CLOCK_DIVIDER_MASK;
	I915_WRITE(MIPI_CTRL(0), tmp | ESCAPE_CLOCK_DIVIDER_1);

	/* read request priority is per pipe */
	tmp = I915_READ(MIPI_CTRL(pipe));
	tmp &= ~READ_REQUEST_PRIORITY_MASK;
	I915_WRITE(MIPI_CTRL(pipe), tmp | READ_REQUEST_PRIORITY_HIGH);

	/* XXX: why here, why like this? handling in irq handler?! */
	I915_WRITE(MIPI_INTR_EN(pipe), 0xffffffff);

	/* why here, was elsewhere... also 2a, 0c, 60, 08 for values */
	I915_WRITE(MIPI_DPHY_PARAM(pipe), intel_dsi->dev.dphy_reg);
}

bool intel_dsi_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi;
	struct intel_encoder *intel_encoder;
	struct drm_encoder *encoder;
	struct intel_connector *intel_connector;
	struct drm_connector *connector;
	struct drm_display_mode *fixed_mode = NULL;
	const struct intel_dsi_device *dsi;
	unsigned int i;

	DRM_DEBUG_KMS("\n");

	intel_dsi = kzalloc(sizeof(struct intel_dsi), GFP_KERNEL);
	if (!intel_dsi)
		return false;

	intel_connector = kzalloc(sizeof(struct intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dsi);
		return false;
	}

	intel_encoder = &intel_dsi->base;
	encoder = &intel_encoder->base;

	connector = &intel_connector->base;

	/* XXX: encoder type */
	drm_encoder_init(dev, encoder, &intel_dsi_funcs,
			DRM_MODE_ENCODER_MIPI);

	/* Initialize panel id based on kernel param.
	 * If no kernel param use panel id from VBT
	 * If no  param and no VBT initialize with
	 * default ASUS panel ID for now */
	if (i915_mipi_panel_id <= 0) {
		/* check if panel id available from VBT */
		if (!dev_priv->mipi.panel_id) {
			/* default ASUS panel */
			dev_priv->mipi.panel_id = MIPI_DSI_AUO_PANEL_ID;
		}
	} else
		dev_priv->mipi.panel_id = i915_mipi_panel_id;

	for (i = 0; i < ARRAY_SIZE(intel_dsi_devices); i++) {
		dsi = &intel_dsi_devices[i];
		/* find the panel based on panel_id */
		if (dsi->panel_id == dev_priv->mipi.panel_id) {
			intel_dsi->dev = *dsi;
			memcpy(&intel_dsi->dev, dsi,
				sizeof(struct intel_dsi_device));

			if (dsi->dev_ops->init(&intel_dsi->dev))
				break;
		}
	}

	if (i == ARRAY_SIZE(intel_dsi_devices)) {
		DRM_ERROR("Unsupported MIPI Panel id:%d\n",
					dev_priv->mipi.panel_id);
		goto err;
	}

	intel_dsi->dsi_packet_format = dsi_24Bpp_packed;
	intel_dsi->channel = 0;

	if (i == ARRAY_SIZE(intel_dsi_devices))
		goto err;

	intel_encoder->type = INTEL_OUTPUT_DSI;
	intel_encoder->crtc_mask = (1 << 0); /* XXX */

	intel_encoder->cloneable = false;
	/* XXX: connector type */
	drm_connector_init(dev, connector, &intel_dsi_connector_funcs,
			DRM_MODE_CONNECTOR_MIPI);

	drm_connector_helper_add(connector, &intel_dsi_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	connector->display_info.subpixel_order = SubPixelHorizontalRGB; /*XXX*/
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_encoder_helper_add(encoder, &intel_dsi_helper_funcs);

	intel_connector_attach_encoder(intel_connector, intel_encoder);

	drm_sysfs_connector_add(connector);


	/* FIXME: try to get a fixed mode. */

	/* FIXME: if fail, try to get the current mode, if any */

	return true;

err:
	drm_encoder_cleanup(&intel_encoder->base);
	kfree(intel_dsi);
	kfree(intel_connector);

	return false;
}
