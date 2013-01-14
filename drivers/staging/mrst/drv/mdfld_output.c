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

struct intel_mid_panel_list panel_list[] = {
	{
		TMD_6X10_VID,
		MDFLD_DSI_ENCODER_DPI,
		"TMD BB PRx",
		tmd_6x10_vid_init
	},
	{
		H8C7_VID,
		MDFLD_DSI_ENCODER_DPI,
		"H8C7 VID RHB",
		h8c7_vid_init
	},
	{
		H8C7_CMD,
		MDFLD_DSI_ENCODER_DBI,
		"H8C7 CMD RHB",
		h8c7_cmd_init
	},
	{
		GI_SONY_VID,
		MDFLD_DSI_ENCODER_DPI,
		"Sony LEX PRx",
		gi_sony_vid_init
	},
	{
		GI_SONY_CMD,
		MDFLD_DSI_ENCODER_DBI,
		"Sony LEX PRx",
		gi_sony_cmd_init
	},
	{
		GI_RENESAS_CMD,
		MDFLD_DSI_ENCODER_DBI,
		"Renesas LEX DV1",
		gi_renesas_cmd_init
	},
	{
		TC35876X_VID,
		MDFLD_DSI_ENCODER_DPI,
		"TMD RR",
		tc35876x_vid_init
	},
	{
		YB_CMI_VID,
		MDFLD_DSI_ENCODER_DPI,
		"CMI YB VID",
		yb_cmi_vid_init
	}
};

int parse_panel_id_from_gct(char *panel_name, int mipi_mode)
{
	int idx = 0;
	for (idx = 0; idx < ARRAY_SIZE(panel_list); idx++) {
		if (!strncmp(panel_name, panel_list[idx].panel_name,
				strlen(panel_list[idx].panel_name)) &&
				mipi_mode == panel_list[idx].encoder_type)
			return panel_list[idx].p_type;
	}

	return -EINVAL;
}

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
	return -1;
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
/*
* use to overwrite fw setting
*/
static void Overwrite_fw_setting(struct mdfld_dsi_config *dsi_config)
{
	return;
}
/*
* In some case Fw has initialized different with driver requirement.
* so driver needs to check whether reusable or not.
* if can reuse, driver can overwrite some setting according driver.
* if can not reuse, driver need reset-panel and display controller.
*/
bool Check_fw_initilized_reusable(struct mdfld_dsi_config *dsi_config,
				struct panel_funcs *p_funcs)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int fw_type = 0xff;
	bool b_reuseable = true;
	u32 pipe_config = REG_READ(regs->pipeconf_reg);

	PSB_DEBUG_ENTRY("\n");

	if (!p_funcs || !dsi_config) {
		DRM_ERROR("invalid parameter!\n");
		return false;
	}

	if (!(pipe_config & PIPEACONF_ENABLE))
		return false;

	/*check fw initialized command/vide mode*/
	if (pipe_config & PIPEACONF_DSR)
		fw_type = MDFLD_DSI_ENCODER_DBI;
	else
		fw_type = MDFLD_DSI_ENCODER_DPI;

	/*check whether the same as driver requirment*/
	if (dsi_config->type == fw_type) {
		b_reuseable = true;
		/*overwrite or totall reuse*/
		Overwrite_fw_setting(dsi_config);
	} else {
		DRM_INFO("can not reuse fw panel setting,do reset\n") ;
		/*reset dispaly controller*/
		acquire_ospm_lock();
		ospm_power_island_down(OSPM_DISPLAY_ISLAND);
		ospm_power_island_up(OSPM_DISPLAY_ISLAND);
		release_ospm_lock();
		b_reuseable = false;
	}

	return b_reuseable;
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

