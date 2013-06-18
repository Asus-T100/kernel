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
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 */
#ifndef __PANASONIC_VID_H__
#define __PANASONIC_VID_H__

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"

/* FIXME: To get the Panasonic panel width/height inches. */
#define PANASONIC_PANEL_WIDTH         72
#define PANASONIC_PANEL_HEIGHT        128

/* Function Prototypes */
static
void panasonic_msgbus_reset();

bool panasonic_init(struct intel_dsi_device *dsi);

static
void panasonic_destroy(struct intel_dsi_device *dsi);

static
void panasonic_commit(struct intel_dsi_device *dsi);

static
void panasonic_prepare(struct intel_dsi_device *dsi);

static
void panasonic_dump_regs(struct intel_dsi_device *dsi);

static
void panasonic_dpms(struct intel_dsi_device *dsi, bool enable);

static
bool panasonic_get_hw_state(struct intel_dsi_device *dev);

static
void panasonic_create_resources(struct intel_dsi_device *dsi);

static
struct drm_display_mode *panasonic_get_modes(struct intel_dsi_device *dsi);

static
enum drm_connector_status panasonic_detect(struct intel_dsi_device *dsi);

static
void panasonic_mode_set(struct intel_dsi_device *dsi,
	struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode);

static
bool panasonic_mode_fixup(struct intel_dsi_device *dsi,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode);

static
int panasonic_mode_valid(struct intel_dsi_device *dsi,
	struct drm_display_mode *mode);

#endif /* __PANASONIC_VID_H__ */
