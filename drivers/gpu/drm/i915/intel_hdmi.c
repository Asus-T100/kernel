/*
 * Copyright 2006 Dave Airlie <airlied@linux.ie>
 * Copyright © 2006-2009 Intel Corporation
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
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "drmP.h"
#include "drm.h"
#include "drm_crtc.h"
#include "drm_edid.h"
#include "intel_drv.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "../drm_edid_modes.h"
/* Added for HDMI Audio */
#include "hdmi_audio_if.h"

static void
assert_hdmi_port_disabled(struct intel_hdmi *intel_hdmi)
{
	struct drm_device *dev = intel_hdmi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t enabled_bits;

	enabled_bits = IS_HASWELL(dev) ? DDI_BUF_CTL_ENABLE : SDVO_ENABLE;

	WARN(I915_READ(intel_hdmi->sdvox_reg) & enabled_bits,
	     "HDMI port enabled, expecting disabled\n");
}

struct intel_hdmi *enc_to_intel_hdmi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct intel_hdmi, base.base);
}

static struct intel_hdmi *intel_attached_hdmi(struct drm_connector *connector)
{
	return container_of(intel_attached_encoder(connector),
			    struct intel_hdmi, base);
}

void intel_dip_infoframe_csum(struct dip_infoframe *frame)
{
	uint8_t *data = (uint8_t *)frame;
	uint8_t sum = 0;
	unsigned i;

	frame->checksum = 0;
	frame->ecc = 0;

	for (i = 0; i < frame->len + DIP_HEADER_SIZE; i++)
		sum += data[i];

	frame->checksum = 0x100 - sum;
}

static u32 g4x_infoframe_index(struct dip_infoframe *frame)
{
	switch (frame->type) {
	case DIP_TYPE_AVI:
		return VIDEO_DIP_SELECT_AVI;
	case DIP_TYPE_SPD:
		return VIDEO_DIP_SELECT_SPD;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", frame->type);
		return 0;
	}
}

static u32 g4x_infoframe_enable(struct dip_infoframe *frame)
{
	switch (frame->type) {
	case DIP_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI;
	case DIP_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", frame->type);
		return 0;
	}
}

static u32 hsw_infoframe_enable(struct dip_infoframe *frame)
{
	switch (frame->type) {
	case DIP_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI_HSW;
	case DIP_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD_HSW;
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", frame->type);
		return 0;
	}
}

static u32 hsw_infoframe_data_reg(struct dip_infoframe *frame, enum pipe pipe)
{
	switch (frame->type) {
	case DIP_TYPE_AVI:
		return HSW_TVIDEO_DIP_AVI_DATA(pipe);
	case DIP_TYPE_SPD:
		return HSW_TVIDEO_DIP_SPD_DATA(pipe);
	default:
		DRM_DEBUG_DRIVER("unknown info frame type %d\n", frame->type);
		return 0;
	}
}

static void g4x_write_infoframe(struct drm_encoder *encoder,
				struct dip_infoframe *frame)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(VIDEO_DIP_CTL);
	unsigned i, len = DIP_HEADER_SIZE + frame->len;

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(frame);

	val &= ~g4x_infoframe_enable(frame);

	I915_WRITE(VIDEO_DIP_CTL, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VIDEO_DIP_DATA, *data);
		data++;
	}
	mmiowb();

	val |= g4x_infoframe_enable(frame);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(VIDEO_DIP_CTL, val);
	POSTING_READ(VIDEO_DIP_CTL);
}

static void ibx_write_infoframe(struct drm_encoder *encoder,
				struct dip_infoframe *frame)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	unsigned i, len = DIP_HEADER_SIZE + frame->len;
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(frame);

	val &= ~g4x_infoframe_enable(frame);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	mmiowb();

	val |= g4x_infoframe_enable(frame);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void cpt_write_infoframe(struct drm_encoder *encoder,
				struct dip_infoframe *frame)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	unsigned i, len = DIP_HEADER_SIZE + frame->len;
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(frame);

	/* The DIP control register spec says that we need to update the AVI
	 * infoframe without clearing its enable bit */
	if (frame->type != DIP_TYPE_AVI)
		val &= ~g4x_infoframe_enable(frame);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	mmiowb();

	val |= g4x_infoframe_enable(frame);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void vlv_write_infoframe(struct drm_encoder *encoder,
				     struct dip_infoframe *frame)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	unsigned i, len = DIP_HEADER_SIZE + frame->len;
	u32 val = I915_READ(reg);

	WARN(!(val & VIDEO_DIP_ENABLE), "Writing DIP with CTL reg disabled\n");

	val &= ~(VIDEO_DIP_SELECT_MASK | 0xf); /* clear DIP data offset */
	val |= g4x_infoframe_index(frame);

	val &= ~g4x_infoframe_enable(frame);

	I915_WRITE(reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < VIDEO_DIP_DATA_SIZE; i += 4)
		I915_WRITE(VLV_TVIDEO_DIP_DATA(intel_crtc->pipe), 0);
	mmiowb();

	val |= g4x_infoframe_enable(frame);
	val &= ~VIDEO_DIP_FREQ_MASK;
	val |= VIDEO_DIP_FREQ_VSYNC;

	I915_WRITE(reg, val);
	POSTING_READ(reg);
}

static void hsw_write_infoframe(struct drm_encoder *encoder,
				struct dip_infoframe *frame)
{
	uint32_t *data = (uint32_t *)frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	u32 ctl_reg = HSW_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 data_reg = hsw_infoframe_data_reg(frame, intel_crtc->pipe);
	unsigned int i, len = DIP_HEADER_SIZE + frame->len;
	u32 val = I915_READ(ctl_reg);

	if (data_reg == 0)
		return;

	val &= ~hsw_infoframe_enable(frame);
	I915_WRITE(ctl_reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(data_reg + i, *data);
		data++;
	}
	mmiowb();

	val |= hsw_infoframe_enable(frame);
	I915_WRITE(ctl_reg, val);
	POSTING_READ(ctl_reg);
}

static void intel_set_infoframe(struct drm_encoder *encoder,
				struct dip_infoframe *frame)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);

	intel_dip_infoframe_csum(frame);
	intel_hdmi->write_infoframe(encoder, frame);
}

static void intel_hdmi_set_avi_infoframe(struct drm_encoder *encoder,
					 struct drm_display_mode *adjusted_mode)
{
	enum hdmi_picture_aspect PAR;
	struct dip_infoframe avi_if = {
		.type = DIP_TYPE_AVI,
		.ver = DIP_VERSION_AVI,
		.len = DIP_LEN_AVI,
		.body.avi.Y_A_B_S = 0,
		.body.avi.C_M_R = 8,
		.body.avi.ITC_EC_Q_SC = 0,
		.body.avi.VIC = 0,
		.body.avi.YQ_CN_PR = 0,
		.body.avi.top_bar_end = 0,
		.body.avi.bottom_bar_start = 0,
		.body.avi.left_bar_end = 0,
		.body.avi.right_bar_start = 0,
	};

	/* Bar information */
	avi_if.body.avi.Y_A_B_S |= DIP_AVI_BAR_BOTH;

	avi_if.body.avi.VIC = drm_match_cea_mode(adjusted_mode);

	/* Set full range quantization for non-CEA modes
		and 640x480 */
	if (avi_if.body.avi.VIC > 1)
		avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_RGB_QUANT_RANGE_LIMITED;
	else
		avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_RGB_QUANT_RANGE_FULL;

	if (avi_if.body.avi.VIC) {
		/* colorimetry */
		if ((adjusted_mode->vdisplay == 480) ||
			(adjusted_mode->vdisplay == 576) ||
			(adjusted_mode->vdisplay == 240) ||
			(adjusted_mode->vdisplay == 288)) {
			avi_if.body.avi.C_M_R |= DIP_AVI_COLOR_ITU601;
		} else if ((adjusted_mode->vdisplay == 720) ||
			(adjusted_mode->vdisplay == 1080)) {
			avi_if.body.avi.C_M_R |= DIP_AVI_COLOR_ITU709;
		}

		/* picture aspect ratio */
		PAR = drm_get_cea_aspect_ratio(avi_if.body.avi.VIC);
		switch (PAR) {
		case HDMI_PICTURE_ASPECT_4_3:
			avi_if.body.avi.C_M_R |= 0x10;
			break;
		case HDMI_PICTURE_ASPECT_16_9:
			avi_if.body.avi.C_M_R |= 0x20;
			break;
		default:
			break;
		}
	} else {
		if (!(adjusted_mode->vdisplay % 3) &&
			((adjusted_mode->vdisplay * 4 / 3) ==
			adjusted_mode->hdisplay))
			avi_if.body.avi.C_M_R |= 0x10;
		else if (!(adjusted_mode->vdisplay % 9) &&
			((adjusted_mode->vdisplay * 16 / 9) ==
			adjusted_mode->hdisplay))
			avi_if.body.avi.C_M_R |= 0x20;
	}

	if (adjusted_mode->flags & DRM_MODE_FLAG_DBLCLK)
		avi_if.body.avi.YQ_CN_PR |= DIP_AVI_PR_2;

	avi_if.body.avi.ITC_EC_Q_SC |= DIP_AVI_IT_CONTENT;

	intel_set_infoframe(encoder, &avi_if);
}

static void intel_hdmi_set_spd_infoframe(struct drm_encoder *encoder)
{
	struct dip_infoframe spd_if;

	memset(&spd_if, 0, sizeof(spd_if));
	spd_if.type = DIP_TYPE_SPD;
	spd_if.ver = DIP_VERSION_SPD;
	spd_if.len = DIP_LEN_SPD;
	strncpy(spd_if.body.spd.vn, "Intel", sizeof(spd_if.body.spd.vn) - 1);
	strncpy(spd_if.body.spd.pd, "Integrated gfx",
		 sizeof(spd_if.body.spd.pd) - 1);
	spd_if.body.spd.sdi = DIP_SPD_PC;

	intel_set_infoframe(encoder, &spd_if);
}

static void g4x_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = VIDEO_DIP_CTL;
	u32 val = I915_READ(reg);
	u32 port;

	assert_hdmi_port_disabled(intel_hdmi);

	/* If the registers were not initialized yet, they might be zeroes,
	 * which means we're selecting the AVI DIP and we're setting its
	 * frequency to once. This seems to really confuse the HW and make
	 * things stop working (the register spec says the AVI always needs to
	 * be sent every VSync). So here we avoid writing to the register more
	 * than we need and also explicitly select the AVI DIP and explicitly
	 * set its frequency to every VSync. Avoiding to write it twice seems to
	 * be enough to solve the problem, but being defensive shouldn't hurt us
	 * either. */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	switch (intel_hdmi->sdvox_reg) {
	case SDVOB:
		port = VIDEO_DIP_PORT_B;
		break;
	case SDVOC:
		port = VIDEO_DIP_PORT_C;
		break;
	default:
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~VIDEO_DIP_ENABLE_VENDOR;

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void ibx_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);
	u32 port;

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	switch (intel_hdmi->sdvox_reg) {
	case HDMIB:
		port = VIDEO_DIP_PORT_B;
		break;
	case HDMIC:
		port = VIDEO_DIP_PORT_C;
		break;
	case HDMID:
		port = VIDEO_DIP_PORT_D;
		break;
	default:
		return;
	}

	if (port != (val & VIDEO_DIP_PORT_MASK)) {
		if (val & VIDEO_DIP_ENABLE) {
			val &= ~VIDEO_DIP_ENABLE;
			I915_WRITE(reg, val);
			POSTING_READ(reg);
		}
		val &= ~VIDEO_DIP_PORT_MASK;
		val |= port;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void cpt_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~(VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI);
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	/* Set both together, unset both together: see the spec. */
	val |= VIDEO_DIP_ENABLE | VIDEO_DIP_ENABLE_AVI;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void vlv_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = VLV_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	/* See the big comment in g4x_set_infoframes() */
	val |= VIDEO_DIP_SELECT_AVI | VIDEO_DIP_FREQ_VSYNC;

	if (!intel_hdmi->has_hdmi_sink) {
		if (!(val & VIDEO_DIP_ENABLE))
			return;
		val &= ~VIDEO_DIP_ENABLE;
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	val |= VIDEO_DIP_ENABLE;
	val &= ~(VIDEO_DIP_ENABLE_VENDOR | VIDEO_DIP_ENABLE_GAMUT |
		 VIDEO_DIP_ENABLE_GCP);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

static void hsw_set_infoframes(struct drm_encoder *encoder,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_i915_private *dev_priv = encoder->dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 reg = HSW_TVIDEO_DIP_CTL(intel_crtc->pipe);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	if (!intel_hdmi->has_hdmi_sink) {
		I915_WRITE(reg, 0);
		POSTING_READ(reg);
		return;
	}

	val &= ~(VIDEO_DIP_ENABLE_VSC_HSW | VIDEO_DIP_ENABLE_GCP_HSW |
		 VIDEO_DIP_ENABLE_VS_HSW | VIDEO_DIP_ENABLE_GMP_HSW);

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, adjusted_mode);
	intel_hdmi_set_spd_infoframe(encoder);
}

int intel_hdmi_encoder_status(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 status = I915_READ(I915_LPE_AUDIO_HDMI_CONFIG_B);
	if (status & I915_LPE_AUDIO_HDMI_ENABLE)
		return true;
	else
		return false;
}

static void intel_hdmi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 sdvox;

	i915_rpm_get_callback(dev);
	sdvox = SDVO_ENCODING_HDMI;
	if (!HAS_PCH_SPLIT(dev))
		sdvox |= intel_hdmi->color_range;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
		sdvox |= SDVO_VSYNC_ACTIVE_HIGH;
	if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
		sdvox |= SDVO_HSYNC_ACTIVE_HIGH;

	if (intel_crtc->bpp > 24)
		sdvox |= COLOR_FORMAT_12bpc;
	else
		sdvox |= COLOR_FORMAT_8bpc;

	/* Required on CPT */
	if (intel_hdmi->has_hdmi_sink && HAS_PCH_CPT(dev))
		sdvox |= HDMI_MODE_SELECT;

	if (intel_hdmi->has_audio) {
		DRM_DEBUG_DRIVER("Enabling HDMI audio on pipe %c\n",
				 pipe_name(intel_crtc->pipe));
		sdvox |= SDVO_AUDIO_ENABLE;
		sdvox |= SDVO_NULL_PACKETS_DURING_VSYNC;
		intel_write_eld(encoder, adjusted_mode);
	}

	if (HAS_PCH_CPT(dev))
		sdvox |= PORT_TRANS_SEL_CPT(intel_crtc->pipe);
	else if (intel_crtc->pipe == PIPE_B)
		sdvox |= SDVO_PIPE_B_SELECT;

	if (intel_hdmi->pfit && (adjusted_mode->hdisplay < PFIT_SIZE_LIMIT)) {
		u32 val = 0;
		if (intel_hdmi->pfit == AUTOSCALE)
			val =  PFIT_ENABLE | (intel_crtc->pipe <<
				PFIT_PIPE_SHIFT) | PFIT_SCALING_AUTO;
		if (intel_hdmi->pfit == PILLARBOX)
			val =  PFIT_ENABLE | (intel_crtc->pipe <<
				PFIT_PIPE_SHIFT) | PFIT_SCALING_PILLAR;
		else if (intel_hdmi->pfit == LETTERBOX)
			val =  PFIT_ENABLE | (intel_crtc->pipe <<
				PFIT_PIPE_SHIFT) | PFIT_SCALING_LETTER;
		DRM_DEBUG_DRIVER("pfit val = %x", val);
		I915_WRITE(PFIT_CONTROL, val);
	}

	I915_WRITE(intel_hdmi->sdvox_reg, sdvox);
	POSTING_READ(intel_hdmi->sdvox_reg);

	intel_hdmi->set_infoframes(encoder, adjusted_mode);
	i915_rpm_put_callback(dev);
}

static void intel_hdmi_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	u32 temp;
	u32 enable_bits = SDVO_ENABLE;

	i915_rpm_get_callback(dev);
	if (intel_hdmi->has_audio)
		enable_bits |= SDVO_AUDIO_ENABLE;

	temp = I915_READ(intel_hdmi->sdvox_reg);

	/* HW workaround for IBX, we need to move the port to transcoder A
	 * before disabling it. */
	if (HAS_PCH_IBX(dev)) {
		struct drm_crtc *crtc = encoder->crtc;
		int pipe = crtc ? to_intel_crtc(crtc)->pipe : -1;

		if (mode != DRM_MODE_DPMS_ON) {
			/* If device is resuming, no need of dpms off.
			 * ALready the pipe is off and inactive
			 */
			if (dev_priv->is_resuming == true) {
				DRM_DEBUG("device is resuming.Returning\n");
				goto exit;
			}
			if (temp & SDVO_PIPE_B_SELECT) {
				temp &= ~SDVO_PIPE_B_SELECT;
				I915_WRITE(intel_hdmi->sdvox_reg, temp);
				POSTING_READ(intel_hdmi->sdvox_reg);

				/* Again we need to write this twice. */
				I915_WRITE(intel_hdmi->sdvox_reg, temp);
				POSTING_READ(intel_hdmi->sdvox_reg);

				/* Transcoder selection bits only update
				 * effectively on vblank. */
				if (crtc)
					intel_wait_for_vblank(dev, pipe);
				else
					msleep(50);
			}
		} else {
			/* Restore the transcoder select bit. */
			if (pipe == PIPE_B)
				enable_bits |= SDVO_PIPE_B_SELECT;
		}
	}

exit:
	/* HW workaround, need to toggle enable bit off and on for 12bpc, but
	 * we do this anyway which shows more stable in testing.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->sdvox_reg, temp & ~SDVO_ENABLE);
		POSTING_READ(intel_hdmi->sdvox_reg);
	}

	if (mode != DRM_MODE_DPMS_ON) {
		temp &= ~enable_bits;
	} else {
		temp |= enable_bits;
	}

	I915_WRITE(intel_hdmi->sdvox_reg, temp);
	POSTING_READ(intel_hdmi->sdvox_reg);

	if (mode == DRM_MODE_DPMS_ON) {
		if (wait_for(((I915_READ(DPLL(0)) & 0x0F) == 0), 40))
			DRM_ERROR("DPLL %x failed to lock\n",
					I915_READ(DPLL(0)));
	}

	/* HW workaround, need to write this twice for issue that may result
	 * in first write getting masked.
	 */
	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(intel_hdmi->sdvox_reg, temp);
		POSTING_READ(intel_hdmi->sdvox_reg);
	}
	i915_rpm_put_callback(dev);
}

static int intel_hdmi_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;
	if (mode->clock < 20000)
		return MODE_CLOCK_LOW;

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	return MODE_OK;
}

static bool intel_hdmi_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static bool g4x_hdmi_connected(struct intel_hdmi *intel_hdmi)
{
	struct drm_device *dev = intel_hdmi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t bit;

	switch (intel_hdmi->sdvox_reg) {
	case SDVOB:
		bit = HDMIB_HOTPLUG_LIVE_STATUS;
		break;
	case SDVOC:
		bit = HDMIC_HOTPLUG_LIVE_STATUS;
		break;
	default:
		bit = 0;
		break;
	}

	return I915_READ(PORT_HOTPLUG_STAT) & bit;
}

void intel_hdmi_reset(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;

	/* Clean previous detects and modes */
	dev_priv->is_hdmi = false;
	intel_hdmi->edid_mode_count = 0;
	intel_cleanup_modes(connector);
	kfree(intel_hdmi->edid);
	intel_hdmi->edid = NULL;
}

static enum drm_connector_status
intel_hdmi_detect(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct edid *edid = NULL;
	enum drm_connector_status status = connector_status_disconnected;

	i915_rpm_get_callback(dev);

	if (IS_G4X(connector->dev) && !g4x_hdmi_connected(intel_hdmi))
		goto out;

#if 0
        /* HOTPLUG Detect is not working in some of VLV A0
         * boards. For those boards enable this WA
         */
	if (IS_VALLEYVIEW(connector->dev))
		return connector_status_connected;
#endif

	/* If its force detection, dont read EDID again */
	if (force && dev_priv->is_hdmi) {
		status = connector->status;
		goto out;
	}

	/* Suppress spurious IRQ, if current status is same as live status*/
	if ((connector->status == connector_status_connected)
		&& g4x_hdmi_connected(intel_hdmi)) {
		status = connector->status;
		goto out;
	}

	dev_priv->is_hdmi = false;
	intel_hdmi->has_hdmi_sink = false;
	intel_hdmi->has_audio = false;

	/* Read live status, get EDID if connected */
	if (g4x_hdmi_connected(intel_hdmi)) {
		edid = drm_get_edid(connector,
			intel_gmbus_get_adapter(dev_priv,
					intel_hdmi->ddc_bus));

	}

	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL) {
			status = connector_status_connected;
			dev_priv->is_hdmi = true;
			if (intel_hdmi->force_audio != HDMI_AUDIO_OFF_DVI)
				intel_hdmi->has_hdmi_sink =
					drm_detect_hdmi_monitor(edid);
			intel_hdmi->has_audio =
				drm_detect_monitor_audio(edid);
		}
		/* Free previously saved EDID and save new one */
		kfree(intel_hdmi->edid);
		intel_hdmi->edid = edid;
		connector->display_info.raw_edid = (char *)edid;
	} else {
		/* Connector is disconneted, so remove old EDID */
		kfree(intel_hdmi->edid);
		intel_hdmi->edid = NULL;
		connector->display_info.raw_edid = NULL;
	}

	/* Disable CRTC on HDMI hot un-plug */
	if (status == connector_status_disconnected) {
		if (intel_hdmi->base.base.crtc) {
			struct drm_crtc *crtc = intel_hdmi->base.base.crtc;
			struct drm_device *dev = crtc->dev;
			connector->encoder = NULL;
			drm_helper_disable_unused_functions(dev);

			/* Enable Max Fifo on HDMI hot un-plg */
			if (is_maxfifo_needed(dev_priv))
				I915_WRITE(FW_BLC_SELF_VLV, FW_CSPWRDWNEN);
		}
	}

	if ((status == connector_status_connected)
			&& (status != i915_hdmi_state)) {
		/* Added for HDMI Audio */
	if (intel_hdmi->has_audio)
		i915_notify_had = 1;
		if (intel_hdmi->force_audio != HDMI_AUDIO_AUTO)
			intel_hdmi->has_audio =
			(intel_hdmi->force_audio == HDMI_AUDIO_ON);
	} else if (status != i915_hdmi_state)  {
		/* Added for HDMI Audio */
		mid_hdmi_audio_signal_event(dev_priv->dev,
			HAD_EVENT_HOT_UNPLUG);
	if (intel_hdmi->has_audio)
		i915_notify_had = 0;
	}

	/* Added for HDMI Audio */
	i915_hdmi_state = status;

out:
	i915_rpm_put_callback(dev);
	return status;
}

static int intel_hdmi_get_modes(struct drm_connector *connector)
{
	int count = 0;
	struct drm_display_mode *mode = NULL;
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct edid *edid = intel_hdmi->edid;
	/* Added for HDMI Audio */
	int ret;

	/* No need to read modes if no connection */
	if (connector->status != connector_status_connected)
		return 0;

	/* Need not to read modes again if previously read modes are
	available and display is connected */
	if (dev_priv->is_hdmi) {
		list_for_each_entry(mode, &connector->modes, head) {
			mode->status = MODE_OK;
			count++;
		}
		/* If modes are availlable, no need to read again */
		if (count)
			return count;
	}

	/* If EDID is available from detect, re-use that, else read now */
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		drm_edid_to_eld(connector, edid);
	} else {
		ret = intel_ddc_get_modes(connector,
		intel_gmbus_get_adapter(dev_priv,
			intel_hdmi->ddc_bus));
	}

	intel_hdmi->edid_mode_count = ret;
	hdmi_get_eld(connector->eld);
	return ret;
}

static bool
intel_hdmi_detect_audio(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct edid *edid;
	bool has_audio = false;

	edid = drm_get_edid(connector,
			    intel_gmbus_get_adapter(dev_priv,
						    intel_hdmi->ddc_bus));
	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL)
			has_audio = drm_detect_monitor_audio(edid);

		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	return has_audio;
}

static int
intel_hdmi_set_property(struct drm_connector *connector,
			struct drm_property *property,
			uint64_t val)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	int ret;

	ret = drm_connector_property_set_value(connector, property, val);
	if (ret)
		return ret;

	/*Check force audio */
	if (property == dev_priv->force_audio_property) {
		enum hdmi_force_audio i = val;
		bool has_audio;

		if (i == intel_hdmi->force_audio)
			return 0;

		intel_hdmi->force_audio = i;

		if (i == HDMI_AUDIO_AUTO)
			has_audio = intel_hdmi_detect_audio(connector);
		else
			has_audio = (i == HDMI_AUDIO_ON);

		if (i == HDMI_AUDIO_OFF_DVI)
			intel_hdmi->has_hdmi_sink = 0;

		intel_hdmi->has_audio = has_audio;
		goto done;
	}

	if (property == dev_priv->broadcast_rgb_property) {
		if (val == !!intel_hdmi->color_range)
			return 0;

		intel_hdmi->color_range = val ? SDVO_COLOR_RANGE_16_235 : 0;
		goto done;
	}

	if (property == dev_priv->force_pfit_property) {
		if (val == intel_hdmi->pfit)
			return 0;

		DRM_DEBUG_DRIVER("val = %d", (int)val);
		intel_hdmi->pfit = val;
		goto done;
	}

	return -EINVAL;

done:
	if (intel_hdmi->base.base.crtc) {
		struct drm_crtc *crtc = intel_hdmi->base.base.crtc;
		drm_crtc_helper_set_mode(crtc, &crtc->mode,
					 crtc->x, crtc->y,
					 crtc->fb);
	}

	return 0;
}

static void intel_hdmi_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static const struct drm_encoder_helper_funcs intel_hdmi_helper_funcs_hsw = {
	.dpms = intel_ddi_dpms,
	.mode_fixup = intel_hdmi_mode_fixup,
	.prepare = intel_encoder_prepare,
	.mode_set = intel_ddi_mode_set,
	.commit = intel_encoder_commit,
};

static const struct drm_encoder_helper_funcs intel_hdmi_helper_funcs = {
	.dpms = intel_hdmi_dpms,
	.mode_fixup = intel_hdmi_mode_fixup,
	.prepare = intel_encoder_prepare,
	.mode_set = intel_hdmi_mode_set,
	.commit = intel_encoder_commit,
	.inuse = intel_hdmi_encoder_status,
};

static const struct drm_connector_funcs intel_hdmi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = intel_hdmi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_hdmi_set_property,
	.destroy = intel_hdmi_destroy,
	.reset = intel_hdmi_reset,
};

static const struct drm_connector_helper_funcs intel_hdmi_connector_helper_funcs = {
	.get_modes = intel_hdmi_get_modes,
	.mode_valid = intel_hdmi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_encoder_funcs intel_hdmi_enc_funcs = {
	.destroy = intel_encoder_destroy,
};

static void
intel_hdmi_add_properties(struct intel_hdmi *intel_hdmi, struct drm_connector *connector)
{
	intel_attach_force_audio_property(connector);
	intel_attach_broadcast_rgb_property(connector);
	intel_attach_force_pfit_property(connector);
}


/* Added for HDMI Audio */
void i915_had_wq(struct work_struct *work)
{
	struct drm_i915_private *dev_priv = container_of(work,
		struct drm_i915_private, hdmi_audio_wq);

	DRM_DEBUG_DRIVER("Checking for HDMI connection at boot\n");
	if (i915_hdmi_state == connector_status_connected) {
		DRM_DEBUG_DRIVER("hdmi_do_audio_wq: HDMI plugged in\n");
		mid_hdmi_audio_signal_event(dev_priv->dev,
			HAD_EVENT_HOT_PLUG);
	}
}

void intel_hdmi_init(struct drm_device *dev, int sdvox_reg, enum port port)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_connector *connector;
	struct intel_encoder *intel_encoder;
	struct intel_connector *intel_connector;
	struct intel_hdmi *intel_hdmi;
	/* Added for HDMI Audio */
	struct hdmi_audio_priv *hdmi_priv;

	intel_hdmi = kzalloc(sizeof(struct intel_hdmi), GFP_KERNEL);
	if (!intel_hdmi)
		return;
	intel_hdmi->pfit = 0;

	intel_connector = kzalloc(sizeof(struct intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_hdmi);
		return;
	}

	intel_encoder = &intel_hdmi->base;
	drm_encoder_init(dev, &intel_encoder->base, &intel_hdmi_enc_funcs,
			 DRM_MODE_ENCODER_TMDS);

	connector = &intel_connector->base;
	drm_connector_init(dev, connector, &intel_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &intel_hdmi_connector_helper_funcs);

	intel_encoder->type = INTEL_OUTPUT_HDMI;

	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	intel_encoder->crtc_mask = (1 << 0) | (1 << 1) | (1 << 2);

	intel_encoder->cloneable = false;

	intel_hdmi->ddi_port = port;
	if (IS_VALLEYVIEW(dev))
		intel_encoder->port = sdvox_reg;

	switch (port) {
	case PORT_B:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPB;
		dev_priv->hotplug_supported_mask |= HDMIB_HOTPLUG_INT_STATUS;
		break;
	case PORT_C:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPC;
		dev_priv->hotplug_supported_mask |= HDMIC_HOTPLUG_INT_STATUS;
		break;
	case PORT_D:
		intel_hdmi->ddc_bus = GMBUS_PORT_DPD;
		dev_priv->hotplug_supported_mask |= HDMID_HOTPLUG_INT_STATUS;
		break;
	case PORT_A:
		/* Internal port only for eDP. */
	default:
		BUG();
	}

	intel_hdmi->sdvox_reg = sdvox_reg;

	if (IS_VALLEYVIEW(dev)) {
		intel_hdmi->write_infoframe = vlv_write_infoframe;
		intel_hdmi->set_infoframes = vlv_set_infoframes;
	} else if (IS_HASWELL(dev)) {
		intel_hdmi->write_infoframe = hsw_write_infoframe;
		intel_hdmi->set_infoframes = hsw_set_infoframes;
	} else if (HAS_PCH_IBX(dev)) {
		intel_hdmi->write_infoframe = ibx_write_infoframe;
		intel_hdmi->set_infoframes = ibx_set_infoframes;
	} else if (!HAS_PCH_SPLIT(dev)) {
		intel_hdmi->write_infoframe = g4x_write_infoframe;
		intel_hdmi->set_infoframes = g4x_set_infoframes;
	} else {
		intel_hdmi->write_infoframe = cpt_write_infoframe;
		intel_hdmi->set_infoframes = cpt_set_infoframes;
	}

	if (IS_HASWELL(dev))
		drm_encoder_helper_add(&intel_encoder->base, &intel_hdmi_helper_funcs_hsw);
	else
		drm_encoder_helper_add(&intel_encoder->base, &intel_hdmi_helper_funcs);

	intel_hdmi_add_properties(intel_hdmi, connector);

	intel_connector_attach_encoder(intel_connector, intel_encoder);
	drm_sysfs_connector_add(connector);

	/* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written
	 * 0xd.  Failure to do so will result in spurious interrupts being
	 * generated on the port when a cable is not attached.
	 */
	if (IS_G4X(dev) && !IS_GM45(dev)) {
		u32 temp = I915_READ(PEG_BAND_GAP_DATA);
		I915_WRITE(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
	}
	/* Added for HDMI Audio */
	/* HDMI private data */
	INIT_WORK(&dev_priv->hdmi_audio_wq, i915_had_wq);
	hdmi_priv = kzalloc(sizeof(struct hdmi_audio_priv), GFP_KERNEL);
	if (!hdmi_priv) {
		pr_err("failed to allocate memory");
	} else {
		hdmi_priv->dev = dev;
		hdmi_priv->hdmib_reg = SDVOB;
		hdmi_priv->monitor_type = MONITOR_TYPE_HDMI;
		hdmi_priv->is_hdcp_supported = true;
		i915_hdmi_audio_init(hdmi_priv);
	}
}
