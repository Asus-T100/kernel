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
 * Author: Jani Nikula <jani.nikula@intel.com>  */

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
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"

/* the panel drivers are here */
static const struct intel_dsi_device intel_dsi_devices[] = {
	{
		.panel_id = MIPI_DSI_CMI_PANEL_ID,
		.type = INTEL_DSI_COMMAND_MODE,
		.name = "cmi-dsi-cmd-mode-display",
		.dev_ops = &cmi_dsi_display_ops,
		.lane_count = 3, /* XXX: this really doesn't belong here */
	},
	{
		.panel_id = MIPI_DSI_CMI_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "cmi-dsi-vid-mode-display",
		.dev_ops = &cmi_dsi_display_ops,
		.lane_count = 3, /* XXX: this really doesn't belong here */
	},
	{
		.panel_id = MIPI_DSI_AUO_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-dsi-vid-mode-display",
		.dev_ops = &auo_dsi_display_ops,
		.lane_count = 3, /* XXX: this really doesn't belong here */
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

static void intel_dsi_enable(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 temp;

	DRM_DEBUG_KMS("\n");

	/* device ready */

	/* bridge reset & configure etc... */


	temp = I915_READ(MIPI_DEVICE_READY(pipe));
	temp &= ~ULPS_STATE_MASK;
	temp &= ~DEVICE_READY; /* XXX */
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp | ULPS_STATE_EXIT);
	msleep(20);
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp);

	/* XXX: lane configuration etc. XXX: port ctrl is a mess */
	temp = I915_READ(MIPI_PORT_CTRL(pipe));
	I915_WRITE(MIPI_PORT_CTRL(pipe), temp | DPI_ENABLE);
	POSTING_READ(MIPI_PORT_CTRL(pipe));

	intel_dsi->dev.dev_ops->dpms(&intel_dsi->dev, true);

	/* XXX: Placement of this? */
	temp = I915_READ(MIPI_DEVICE_READY(pipe));
	I915_WRITE(MIPI_DEVICE_READY(pipe), temp | DEVICE_READY);
}

static void intel_dsi_disable(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 temp;

	DRM_DEBUG_KMS("\n");

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
static u16 txbyteclkhs(u16 pixels, int bpp, int lane_count)
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
	I915_WRITE(MIPI_VBP_COUNT(pipe), vbp); }


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
	unsigned int bpp = intel_crtc->bpp;
	u32 val;

	DRM_DEBUG_KMS("\n");

	I915_WRITE(MIPI_DPI_RESOLUTION(pipe),
		   adjusted_mode->vdisplay << VERTICAL_ADDRESS_SHIFT |
		   adjusted_mode->hdisplay << HORIZONTAL_ADDRESS_SHIFT);

	set_dsi_timings(encoder, adjusted_mode);

	val = intel_dsi->channel << VID_MODE_CHANNEL_NUMBER_SHIFT |
		intel_dsi->dev.lane_count << DATA_LANES_PRG_REG_SHIFT;

	switch (intel_crtc->bpp) {
	case 16:
		val |= VID_MODE_FORMAT_RGB565;
		break;
	case 18:
		val |= VID_MODE_FORMAT_RGB666;
		break;
	default:
		DRM_ERROR("%d bpp is not supported\n", intel_crtc->bpp);
	case 24:
		val |= VID_MODE_FORMAT_RGB888;
		break;
	}

	/* XXX: cmd mode vs. video mode */
	I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

	/* one frame IIUC. if counter expires, EOT and stop state */
	I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
		   txbyteclkhs(adjusted_mode->vtotal * adjusted_mode->htotal,
			       bpp, intel_dsi->dev.lane_count));
	I915_WRITE(MIPI_LP_RX_TIMEOUT(pipe), 0xffff);
	I915_WRITE(MIPI_TURN_AROUND_TIMEOUT(pipe), 0x14); /* XXX: unit??? */

	I915_WRITE(MIPI_DEVICE_RESET_TIMER(pipe), 0xffff); /* also 0xff */


	/* in terms of low power clock */
	I915_WRITE(MIPI_INIT_COUNT(pipe), 0x7d0); /* also 0xf0 */

	I915_WRITE(MIPI_EOT_DISABLE(pipe), 0);

	/* in terms of txbyteclkhs. actual high to low switch +
	 * MIPI_STOP_STATE_STALL * MIPI_LP_BYTECLK */
	I915_WRITE(MIPI_HIGH_LOW_SWITCH_COUNT(pipe), 0x46); /* also 0x25 */

	/* XXX: low power clock equivalence in terms of byte clock. the number
	 * of byte clocks occupied in one low power clock. based on txbyteclkhs
	 * and txclkesc. txclkesc time / txbyteclk time * (105 +
	 * MIPI_STOP_STATE_STALL) / 105. ???
	 */
	I915_WRITE(MIPI_LP_BYTECLK(pipe), 4);

	/* the bw essential for transmitting 16 long packets containing 252
	 * bytes meant for dcs write memory command is programmed in this
	 * register in terms of byte clocks. based on dsi transfer rate and the
	 * number of lanes configured the time taken to transmit 16 long packets
	 * in a dsi stream varies. */
	I915_WRITE(MIPI_DBI_BW_CTRL(pipe), 0x820);

	I915_WRITE(MIPI_CLK_LANE_SWITCH_TIME_CNT(pipe),
		   0xa << LP_HS_SSW_CNT_SHIFT |
		   0x14 << HS_LP_PWR_SW_CNT_SHIFT);

	/* XXX: video mode vs. command mode */
	I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe), VIDEO_MODE_BURST);

	/* XXX: MIPI_DEVICE_READY here? where? */

	/* set up pipe... is done in mode set func *before* this one is
	 * called */

	/* this should be done someplace else. it's odd all in all. is this dpi
	 * or video mode? seems dpi but (why) do we need it?! */

#if 0	/* rule of thumb, no intel_dsi_dsi.c functions called in this file? */
	dpi_send_cmd(intel_dsi, TURN_ON);
#endif
}

static enum drm_connector_status
intel_dsi_detect(struct drm_connector *connector, bool force)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	DRM_DEBUG_KMS("\n");
	return intel_dsi->dev.dev_ops->detect(&intel_dsi->dev);
}

static int intel_dsi_get_modes(struct drm_connector *connector)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_display_mode *mode;

	DRM_DEBUG_KMS("\n");

	if (!intel_dsi->panel_fixed_mode)
		return 0;

	mode = drm_mode_duplicate(connector->dev, intel_dsi->panel_fixed_mode);
	if (!mode)
		return 0;

	drm_mode_probed_add(connector, mode);
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

/* XXX: I don't know where all this should be set... */
static void dsi_config(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->bpp;
	unsigned int lane_count = intel_dsi->dev.lane_count;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	/* XXX:
	 *
	 * - order of all of these
	 * - persistence; *when* do we need to reset these
	 * - video vs. command mode needs
	 * - etc.
	 *
	 */
	/* */

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
	I915_WRITE(MIPI_DPHY_PARAM(pipe),
		   0x15 << EXIT_ZERO_COUNT_SHIFT |
		   0x0a << TRAIL_COUNT_SHIFT |
		   0x60 << CLK_ZERO_COUNT_SHIFT |
		   0x0f << PREPARE_COUNT_SHIFT);

#if 0	/* XXX: do we need to set/check these: */
	I915_WRITE(MIPI_TEARING_CTRL(pipe), 0);
	I915_WRITE(_MIPIA_AUTOPWG, 0);

	/*
	MIPI_INTR_STAT - irq handling, or just checking them?
	MIPI_INTR_EN off
	MIPI_DBI_FIFO_THROTTLE
	MIPI_MAX_RETURN_PKT_SIZE - command mode?

	MIPI_MAX_RETURN_PKT_SIZE - more checks? just in command mode? what?

	MIPI_HS_LS_DBI_ENABLE - command mode? needed to change mode in advance?


	"stop state stall: need to change MIPI_HIGH_LOW_SWITCH_COUNT and
	MIPI_LP_BYTECLK to compensate too."
	MIPI_STOP_STATE_STALL
	MIPI_INTR_STAT_REG_1
	MIPI_INTR_EN_REG_1

	MIPIA_DBI_TYPEC_CTRL - type C? only pipe a?
	*/
#endif

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

	intel_dsi = kzalloc(sizeof(*intel_dsi), GFP_KERNEL);
	if (!intel_dsi)
		return false;

	intel_connector = kzalloc(sizeof(*intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dsi);
		return false;
	}

	intel_encoder = &intel_dsi->base;
	encoder = &intel_encoder->base;

	connector = &intel_connector->base;

	/* XXX: encoder type */
	drm_encoder_init(dev, encoder, &intel_dsi_funcs,
			 DRM_MODE_ENCODER_LVDS);
#if 0
	intel_encoder->pre_enable = intel_dsi_pre_enable;
	intel_encoder->enable = intel_dsi_enable;
	intel_encoder->pre_pll_enable = intel_dsi_pre_pll_enable;
	intel_encoder->disable = intel_dsi_disable;
	intel_encoder->post_disable = intel_dsi_post_disable;
	intel_encoder->get_hw_state = intel_dsi_get_hw_state;
	intel_connector->get_hw_state = intel_connector_get_hw_state;
#endif
	for (i = 0; i < ARRAY_SIZE(intel_dsi_devices); i++) {
		dsi = &intel_dsi_devices[i];
		/* XXX: find the panel based on name?! lane setup, etc. */

		intel_dsi->dev = *dsi;

		if (dsi->dev_ops->init(&intel_dsi->dev))
			break;
	}

	if (i == ARRAY_SIZE(intel_dsi_devices))
		goto err;

	intel_encoder->type = INTEL_OUTPUT_DSI;
	intel_encoder->crtc_mask = (1 << 0); /* XXX */

	intel_encoder->cloneable = false;
	/* XXX: connector type */
	drm_connector_init(dev, connector, &intel_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_LVDS);

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

	/*
	 * FIXME: if CDF requires adding the panel driver module later, this may
	 * need to be completely hotplug based from drm perspective
	 */

	 /* XXX: fixed_mode */
	/*intel_panel_init(&intel_connector->panel, fixed_mode);*/

	return true;

err:
	drm_encoder_cleanup(&intel_encoder->base);
	kfree(intel_dsi);
	kfree(intel_connector);

	return false;
}
