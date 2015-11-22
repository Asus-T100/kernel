/*
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (c) 2007, 2010 Intel Corporation
 *   Jesse Barnes <jesse.barnes@intel.com>
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
 */

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include "i915_drv.h"

/**
 * intel_connector_update_modes - update connector from edid
 * @connector: DRM connector device to use
 * @edid: previously read EDID information
 */
int intel_connector_update_modes(struct drm_connector *connector,
				struct edid *edid)
{
	int ret;

	drm_mode_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);
	drm_edid_to_eld(connector, edid);

	return ret;
}

/**
 * intel_ddc_get_modes - get modelist from monitor
 * @connector: DRM connector device to use
 * @adapter: i2c adapter
 *
 * Fetch the EDID information from @connector using the DDC bus.
 */
int intel_ddc_get_modes(struct drm_connector *connector,
			struct i2c_adapter *adapter)
{
	struct edid *edid;
	int ret;

	edid = drm_get_edid(connector, adapter);
	if (!edid)
		return 0;

	ret = intel_connector_update_modes(connector, edid);
	kfree(edid);

	return ret;
}

void intel_cleanup_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode = NULL;
	struct drm_display_mode *t = NULL;

	list_for_each_entry_safe(mode, t, &connector->probed_modes, head)
		drm_mode_remove(connector, mode);

	list_for_each_entry_safe(mode, t, &connector->modes, head)
		drm_mode_remove(connector, mode);
}

static const struct drm_prop_enum_list drrs_capability_names[] = {
	{ DRRS_NOT_SUPPORTED, "Off" },
	{ STATIC_DRRS_SUPPORT, "Static" },
	{ SEAMLESS_DRRS_SUPPORT, "Seamless-HW" },
	{ SEAMLESS_DRRS_SUPPORT_SW, "Seamless-SW" },
};

void
intel_attach_drrs_capability_property(struct drm_connector *connector,
					unsigned int init_val)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->drrs_capability_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, 0,
					   "drrs_capability",
					   drrs_capability_names,
					   ARRAY_SIZE(drrs_capability_names));
		if (prop == NULL)
			return;

		dev_priv->drrs_capability_property = prop;
	}
	drm_object_attach_property(&connector->base, prop, init_val);
}

static const struct drm_prop_enum_list force_audio_names[] = {
	{ HDMI_AUDIO_OFF_DVI, "force-dvi" },
	{ HDMI_AUDIO_OFF, "off" },
	{ HDMI_AUDIO_AUTO, "auto" },
	{ HDMI_AUDIO_ON, "on" },
};

void
intel_attach_force_audio_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->force_audio_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, 0,
					   "audio",
					   force_audio_names,
					   ARRAY_SIZE(force_audio_names));
		if (prop == NULL)
			return;

		dev_priv->force_audio_property = prop;
	}
	drm_object_attach_property(&connector->base, prop, 0);
}

static const struct drm_prop_enum_list broadcast_rgb_names[] = {
	{ INTEL_BROADCAST_RGB_AUTO, "Automatic" },
	{ INTEL_BROADCAST_RGB_FULL, "Full" },
	{ INTEL_BROADCAST_RGB_LIMITED, "Limited 16:235" },
};

void
intel_attach_broadcast_rgb_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->broadcast_rgb_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, DRM_MODE_PROP_ENUM,
					   "Broadcast RGB",
					   broadcast_rgb_names,
					   ARRAY_SIZE(broadcast_rgb_names));
		if (prop == NULL)
			return;

		dev_priv->broadcast_rgb_property = prop;
	}

	drm_object_attach_property(&connector->base, prop, 0);
}

static const struct drm_prop_enum_list pfit_names[] = {
	{ 0, "Pfit off" },
	{ 1, "Auto scale" },
	{ 2, "PillarBox" },
	{ 3, "LetterBox" },
};

void
intel_attach_force_pfit_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;
	struct drm_mode_object *obj = &connector->base;

	prop = dev_priv->force_pfit_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, 0,
						"pfit",
						pfit_names,
						ARRAY_SIZE(pfit_names));
		if (prop == NULL)
			return;

		dev_priv->force_pfit_property = prop;
	}

	drm_object_attach_property(obj, prop, 0);
}

void
intel_attach_scaling_src_size_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;
	struct drm_mode_object *obj = &connector->base;

	prop = dev_priv->scaling_src_size_property;
	if (prop == NULL) {
		prop = drm_property_create_range(dev, 0,
						"scaling_src_size",
						0,
						UINT_MAX);
		if (prop == NULL)
			return;

		dev_priv->scaling_src_size_property = prop;
	}

	drm_object_attach_property(obj, prop, 0);
}
