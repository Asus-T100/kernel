/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including withouti limitation
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

#ifndef _INTEL_DSI_H
#define _INTEL_DSI_H

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"

#define dsi_18Bpp_loosely_packed	0
#define dsi_18Bpp_packed		1
#define dsi_16Bpp_packed		2
#define dsi_24Bpp_packed		3

#define ColorConversionInBridge		0
#define ColorConversionInHostController	1

#define DSI_HSS_PACKET_SIZE 4
#define DSI_HSE_PACKET_SIZE 4
#define DSI_HSA_LPACKET_EXTRA_SIZE 6
#define DSI_HBP_LPACKET_EXTRA_SIZE 6
#define DSI_HACTIVE_LPACKET_EXTRA_SIZE 6
#define DSI_HFP_LPACKET_EXTRA_SIZE 6

struct intel_dsi_device {
	unsigned short panel_id;
	const char *name;
	int type;
	unsigned int lane_count;
	const struct intel_dsi_dev_ops *dev_ops;
	void *dev_priv;
};

/* XXX: this is just copy-paste from dvo for now */
struct intel_dsi_dev_ops {
	bool (*init)(struct intel_dsi_device *dsi);

	void (*get_info)(int pipe, struct drm_connector *connector);

	void (*create_resources)(struct intel_dsi_device *dsi);

	void (*dpms)(struct intel_dsi_device *dsi, bool enable);

	int (*mode_valid)(struct intel_dsi_device *dsi,
			  struct drm_display_mode *mode);

	bool (*mode_fixup)(struct intel_dsi_device *dsi,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode);

	void (*prepare)(struct intel_dsi_device *dsi);

	void (*commit)(struct intel_dsi_device *dsi);

	void (*mode_set)(struct intel_dsi_device *dsi,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);

	enum drm_connector_status (*detect)(struct intel_dsi_device *dsi);

	bool (*get_hw_state)(struct intel_dsi_device *dev);

	struct drm_display_mode *(*get_modes)(struct intel_dsi_device *dsi);

	void (*destroy) (struct intel_dsi_device *dsi);

	void (*dump_regs)(struct intel_dsi_device *dsi);
};

struct intel_dsi {
	struct intel_encoder base;

	struct intel_dsi_device dev;

	struct drm_display_mode *panel_fixed_mode;

	/* XXX: are hs mode and channel properties of intel_dsi or
	 * intel_dsi_device?
	 */
	bool hs; /* if true, use HS mode, otherwise LP */
	int channel;
	uint32_t reg_base;
	enum pipe pipe;
	u8 dsi_packet_format;
};

struct panel_info {
	u32 width_mm;
	u32 height_mm;

	/*other infos*/
};

static inline struct intel_dsi *enc_to_intel_dsi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct intel_dsi, base.base);
}

/* the panel drivers */
extern struct intel_dsi_dev_ops cmi_dsi_display_ops;
extern struct intel_dsi_dev_ops auo_dsi_display_ops;

/* internal functions */
static void dsi_config(struct drm_encoder *encoder);

/* external functions */
void intel_dsi_enable(struct intel_encoder *encoder);

/* FIXME */
#define	MIPI_DSI_CMI_PANEL_ID	0x00
#define	MIPI_DSI_AUO_PANEL_ID	0x01

#endif /* _INTEL_DSI_H */
