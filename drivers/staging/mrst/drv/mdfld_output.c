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
#include "android_hdmi.h"

#include "displays/tmd_6x10_vid.h"
#include "displays/h8c7_vid.h"
#include "displays/auo_sc1_vid.h"
#include "displays/auo_sc1_cmd.h"
#include "displays/gi_sony_vid.h"
#include "displays/gi_sony_cmd.h"
#include "displays/h8c7_cmd.h"
#include "displays/tc35876x_vid.h"
#include "displays/gi_renesas_cmd.h"
#include "displays/yb_cmi_vid.h"
#include "displays/hdmi.h"
#include "psb_drv.h"

static struct intel_mid_panel_list panel_list[] = {
	{TMD_6X10_VID,	MDFLD_DSI_ENCODER_DPI, tmd_6x10_vid_init},
	{H8C7_VID,	MDFLD_DSI_ENCODER_DPI, h8c7_vid_init},
	{H8C7_CMD,	MDFLD_DSI_ENCODER_DBI, h8c7_cmd_init},
	{AUO_SC1_VID,	MDFLD_DSI_ENCODER_DPI, auo_sc1_vid_init},
	{AUO_SC1_CMD,	MDFLD_DSI_ENCODER_DBI, auo_sc1_cmd_init},
	{GI_SONY_VID,	MDFLD_DSI_ENCODER_DPI, gi_sony_vid_init},
	{GI_SONY_CMD,	MDFLD_DSI_ENCODER_DBI, gi_sony_cmd_init},
	{GI_RENESAS_CMD, MDFLD_DSI_ENCODER_DBI, gi_renesas_cmd_init},
	{TC35876X_VID,	MDFLD_DSI_ENCODER_DPI, tc35876x_vid_init},
	{YB_CMI_VID,	MDFLD_DSI_ENCODER_DPI, yb_cmi_vid_init}
};

enum panel_type get_panel_type(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;

	return dev_priv->panel_id;
}

int is_panel_vid_or_cmd(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(panel_list); i++) {
		if (panel_list[i].p_type == dev_priv->panel_id)
			return panel_list[i].encoder_type;
	}
}

void init_panel(struct drm_device* dev, int mipi_pipe, enum panel_type p_type)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct panel_funcs *p_funcs = NULL;
	int i = 0, ret = 0;

#ifdef CONFIG_SUPPORT_HDMI
	if (p_type == HDMI) {
		PSB_DEBUG_ENTRY( "GFX: Initializing HDMI");
		android_hdmi_driver_init(dev, &dev_priv->mode_dev);
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
	enum panel_type p_type1, p_type2;

	/* MIPI panel 1 */
	p_type1 = get_panel_type(dev, 0);
	init_panel(dev, 0, p_type1);

#ifdef CONFIG_MDFD_DUAL_MIPI
	/* MIPI panel 2 */
	p_type2 = get_panel_type(dev, 2);
	init_panel(dev, 2, p_type2);
#endif

#ifdef CONFIG_SUPPORT_HDMI
	/* HDMI panel */
	init_panel(dev, 0, HDMI);
#endif
}

