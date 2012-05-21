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
#include "mdfld_output.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_output.h"

#include "displays/tpo_cmd.h"
#include "displays/tpo_vid.h"
#include "displays/tmd_cmd.h"
#include "displays/tmd_vid.h"
#include "displays/pyr_cmd.h"
#include "displays/pyr_vid.h"
#include "displays/tmd_6x10_vid.h"
#include "displays/hdmi.h"
#include "psb_drv.h"

enum panel_type get_panel_type(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	return dev_priv->panel_id;
}

int is_panel_vid_or_cmd(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	int ret = 0;
	switch (dev_priv->panel_id) {
	case TMD_VID:
	case TMD_6X10_VID:
	case TPO_VID:
	case PYR_VID:
		ret = MDFLD_DSI_ENCODER_DPI;
		break;
	case TMD_CMD:
	case TPO_CMD:
	case PYR_CMD:
	default:
		ret = MDFLD_DSI_ENCODER_DBI;
		break;
	}
	return ret;
}

void mdfld_output_init(struct drm_device *dev)
{
	enum panel_type p_type1, p_type2;

	/* MIPI panel 1 */
	p_type1 = get_panel_type(dev, 0);
	/* DIV5-MM-DISPLAY-NC-LCM_INIT-00 */
	PSB_DEBUG_ENTRY("[DISPLAY] %s: panel type is %d\n",
			__func__, p_type1);
	init_panel(dev, 0, p_type1);

	/* Set the panel type as PYR_CMD by default */
	p_type2 = PYR_CMD;
#ifdef CONFIG_MID_DUAL_MIPI
	/* MIPI panel 2 */
	p_type2 = get_panel_type(dev, 2);
	init_panel(dev, 2, p_type2);
#endif

#ifdef CONFIG_MID_HDMI
	/* HDMI panel */
	init_panel(dev, 0, HDMI);
#endif
}

void init_panel(struct drm_device *dev, int mipi_pipe, enum panel_type p_type)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct panel_funcs *p_cmd_funcs = NULL;
	struct panel_funcs *p_vid_funcs = NULL;
	int ret = 0;

	p_cmd_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);
	p_vid_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);

	switch (p_type) {
	case TPO_CMD:
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		tpo_cmd_init(dev, p_cmd_funcs);
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
					  NULL);
		break;
	case TPO_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		tpo_vid_init(dev, p_vid_funcs);
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, NULL,
					  p_vid_funcs);
		break;
	case TMD_6X10_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		tmd_6x10_vid_init(dev, p_vid_funcs);
		ret = mdfld_dsi_output_init(dev, mipi_pipe,
					    NULL, NULL, p_vid_funcs);
		break;
	case TMD_CMD:
		/* tmd_cmd_init(dev, p_cmd_funcs); */
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
					  NULL);
		break;
	case TMD_VID:
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		tmd_vid_init(dev, p_vid_funcs);
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, NULL,
					  p_vid_funcs);
		break;
	case PYR_CMD:
		kfree(p_vid_funcs);
		p_vid_funcs = NULL;
		pyr_cmd_init(dev, p_cmd_funcs);
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
					  NULL);
		break;
	case PYR_VID:
		/* pyr_vid_init(dev, p_vid_funcs); */
		kfree(p_cmd_funcs);
		p_cmd_funcs = NULL;
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, NULL,
					  p_vid_funcs);
		break;
	case TPO:	/*TPO panel supports both cmd & vid interfaces */
		tpo_cmd_init(dev, p_cmd_funcs);
		tpo_vid_init(dev, p_vid_funcs);
		ret =
		    mdfld_dsi_output_init(dev, mipi_pipe, NULL, p_cmd_funcs,
					  p_vid_funcs);
		break;
	case TMD:
	case PYR:
		kfree(p_vid_funcs);
		kfree(p_cmd_funcs);
		p_vid_funcs = NULL;
		p_cmd_funcs = NULL;
		break;
	case HDMI:
		kfree(p_vid_funcs);
		kfree(p_cmd_funcs);
		p_vid_funcs = NULL;
		p_cmd_funcs = NULL;
		/*hdmi_init(dev); */
		PSB_DEBUG_ENTRY("GFX: Initializing HDMI");
		mdfld_hdmi_init(dev, &dev_priv->mode_dev);
		/*hdmi_output_init(dev); */
		break;
	default:
		kfree(p_vid_funcs);
		kfree(p_cmd_funcs);
		p_vid_funcs = NULL;
		p_cmd_funcs = NULL;
		break;
	}

	if (ret) {
		/* if (p_cmd_funcs) */
		kfree(p_cmd_funcs);
		/* if (p_vid_funcs) */
		kfree(p_vid_funcs);
	}
}
