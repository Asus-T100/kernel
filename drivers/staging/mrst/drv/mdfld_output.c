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
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_output.h"
#include "android_hdmi.h"

#include "displays/tmd_6x10_vid.h"
#include "displays/h8c7_vid.h"
#include "displays/auo_sc1_vid.h"
#include "displays/auo_sc1_cmd.h"
#include "displays/gi_sony_vid.h"
#include "displays/gi_sony_cmd.h"
#include "displays/h8c7_cmd.h"
#include "displays/hdmi.h"
#include "psb_drv.h"
#include "tc35876x_vid.h"

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

	int ret = 0;
	switch(dev_priv->panel_id) {
	case AUO_SC1_VID:
	case GI_SONY_VID:
	case TMD_6X10_VID:
	case H8C7_VID:
	case TC_35876X_VID:
		ret =  MDFLD_DSI_ENCODER_DPI;
		break;
	case AUO_SC1_CMD:
	case GI_SONY_CMD:
	case H8C7_CMD:
	default:
		ret =  MDFLD_DSI_ENCODER_DBI;
		break;
	}
	return ret;
}

void init_panel(struct drm_device* dev, int mipi_pipe, enum panel_type p_type)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct panel_funcs * p_cmd_funcs = NULL; 
	struct panel_funcs * p_vid_funcs = NULL;
	int ret = 0;

	dev_priv->cur_pipe = mipi_pipe;
	p_cmd_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);
	p_vid_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);
	
	switch (p_type) {
	case AUO_SC1_CMD:
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		auo_sc1_cmd_init(dev, p_cmd_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
				NULL);
		break;
	case AUO_SC1_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		auo_sc1_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe, NULL, NULL,
				p_vid_funcs);
		break;
	case GI_SONY_CMD:
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		gi_sony_cmd_init(dev, p_cmd_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
				NULL);
		break;
	case H8C7_CMD:
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		h8c7_cmd_init(dev, p_cmd_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
				NULL);
		break;
	case GI_SONY_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		gi_sony_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe, NULL, NULL,
				p_vid_funcs);
		break;
	case TMD_6X10_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		tmd_6x10_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe,
					NULL,
					NULL,
					p_vid_funcs);
		break;
	case H8C7_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		h8c7_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe,
					NULL,
					NULL,
					p_vid_funcs);
		break;
	case TC_35876X_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		tc35876x_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe,
					NULL,
					NULL,
					p_vid_funcs);
		break;
#ifdef CONFIG_MDFD_HDMI
	case HDMI:
		kfree(p_vid_funcs);
		kfree(p_cmd_funcs);
		p_vid_funcs = NULL;
		p_cmd_funcs = NULL;
		/*hdmi_init(dev);*/
		PSB_DEBUG_ENTRY( "GFX: Initializing HDMI");
		android_hdmi_driver_init(dev, &dev_priv->mode_dev);
		/*hdmi_output_init(dev);*/
		break;
#endif
	default:
		kfree(p_vid_funcs);
		kfree(p_cmd_funcs);
		p_vid_funcs = NULL;
		p_cmd_funcs = NULL;
		break;
	}

	if (ret) {
		if (p_cmd_funcs)
			kfree(p_cmd_funcs);
		if (p_vid_funcs)
			kfree(p_vid_funcs);
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

#ifdef CONFIG_MDFD_HDMI
	/* HDMI panel */
	init_panel(dev, 0, HDMI);
#endif
}

