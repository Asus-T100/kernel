/*
 * Copyright (C) 2013 Intel Corporation
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
 *	Chris Tsai <chrisx.tsai@intel.com>
 */
#ifndef __HV101HD1_VID_H__
#define __HV101HD1_VID_H__

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"

/* FIXME: To get the HV101HD1 panel width/height inches. */
#define HV101HD1_PANEL_WIDTH         88
#define HV101HD1_PANEL_HEIGHT        50

/* Function Prototypes */
static
void hv101hd1_msgbus_reset();

bool hv101hd1_init(struct intel_dsi_device *dsi);

static
void hv101hd1_destroy(struct intel_dsi_device *dsi);

static
void hv101hd1_commit(struct intel_dsi_device *dsi);

static
void hv101hd1_prepare(struct intel_dsi_device *dsi);

static
void hv101hd1_dump_regs(struct intel_dsi_device *dsi);

static
void hv101hd1_dpms(struct intel_dsi_device *dsi, bool enable);

static
bool hv101hd1_get_hw_state(struct intel_dsi_device *dev);

static
void hv101hd1_create_resources(struct intel_dsi_device *dsi);

static
struct drm_display_mode *hv101hd1_get_modes(struct intel_dsi_device *dsi);

static
enum drm_connector_status hv101hd1_detect(struct intel_dsi_device *dsi);

static
void hv101hd1_mode_set(struct intel_dsi_device *dsi,
	struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode);

static
bool hv101hd1_mode_fixup(struct intel_dsi_device *dsi,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode);

static
int hv101hd1_mode_valid(struct intel_dsi_device *dsi,
	struct drm_display_mode *mode);

#endif /* __HV101HD1_VID_H__ */