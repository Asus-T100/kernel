/*
 * Copyright (c)  2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
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
 * Thomas Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_output.h"

#include "displays/hdmi.h"
#include "displays/jdi_vid.h"
#include "displays/jdi_cmd.h"
#include "displays/cmi_vid.h"
#include "displays/cmi_cmd.h"
#include "psb_drv.h"
#include "android_hdmi.h"

static struct intel_mid_panel_list panel_list[] = {
	{JDI_VID, MDFLD_DSI_ENCODER_DPI, jdi_vid_init},
	{JDI_CMD, MDFLD_DSI_ENCODER_DBI, jdi_cmd_init},
	{CMI_VID, MDFLD_DSI_ENCODER_DPI, cmi_vid_init},
	{CMI_CMD, MDFLD_DSI_ENCODER_DBI, cmi_cmd_init}
};

enum panel_type get_panel_type(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;

	return dev_priv->panel_id;
}

/**
 * is_panel_vid_or_cmd(struct drm_device *dev)
 * Function return value: panel encoder type
 */
mdfld_dsi_encoder_t is_panel_vid_or_cmd(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	int i;

	for (i = 0; i < ARRAY_SIZE(panel_list); i++) {
		if (panel_list[i].p_type == dev_priv->panel_id)
			return panel_list[i].encoder_type;
	}

	BUG();
}

void init_panel(struct drm_device *dev, int mipi_pipe, enum panel_type p_type)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct panel_funcs *p_funcs = NULL;
	int i = 0, ret = 0;
	struct drm_connector *connector;

#ifdef CONFIG_SUPPORT_HDMI
	if (p_type == HDMI) {
		PSB_DEBUG_ENTRY("GFX: Initializing HDMI");
		android_hdmi_driver_init(dev, &dev_priv->mode_dev);
		if (!IS_MRFLD(dev))
			return;

		mutex_lock(&dev->mode_config.mutex);
		list_for_each_entry(connector,
				&dev->mode_config.connector_list, head) {
			if ((connector->connector_type !=
						DRM_MODE_CONNECTOR_MIPI) &&
					(connector->connector_type !=
					 DRM_MODE_CONNECTOR_LVDS))
				connector->polled = DRM_CONNECTOR_POLL_HPD;
		}
		mutex_unlock(&dev->mode_config.mutex);

		return;
	}
#endif

	dev_priv->cur_pipe = mipi_pipe;
	p_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);

	for (i = 0; i < ARRAY_SIZE(panel_list); i++) {
		if (panel_list[i].p_type == dev_priv->panel_id) {
			panel_list[i].panel_init(
					dev,
					p_funcs);
			ret = mdfld_dsi_output_init(dev,
					mipi_pipe,
					NULL,
					p_funcs);
			if (ret)
				kfree(p_funcs);
			break;
		}
	}
}

void mdfld_output_init(struct drm_device *dev)
{
	enum panel_type p_type1;

	/* MIPI panel 1 */
	p_type1 = get_panel_type(dev, 0);
	init_panel(dev, 0, p_type1);

#ifdef CONFIG_MDFD_DUAL_MIPI
	{
		/* MIPI panel 2 */
		enum panel_type p_type2;
		p_type2 = get_panel_type(dev, 2);
		init_panel(dev, 2, p_type2);
	}
#endif

#ifdef CONFIG_SUPPORT_HDMI
	/* HDMI panel */
	init_panel(dev, 0, HDMI);
#endif
}

