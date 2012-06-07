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



#ifndef MDFLD_OUTPUT_H
#define MDFLD_OUTPUT_H

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>

#include "psb_drv.h"

#define TPO_PANEL_WIDTH		84
#define TPO_PANEL_HEIGHT	46
#define TMD_PANEL_WIDTH		53 /* PR3 */
#define TMD_PANEL_HEIGHT	89 /* PR3 */
#define PYR_PANEL_WIDTH		53
#define PYR_PANEL_HEIGHT	95
#define PANEL_4DOT3_WIDTH	53
#define PANEL_4DOT3_HEIGHT	95
#define AUO_PANEL_WIDTH		54
#define AUO_PANEL_HEIGHT	96
#define PANEL_3DOT47_WIDTH	49
#define PANEL_3DOT47_HEIGHT	73

struct mdfld_dsi_config;

/*DSI panel connection status*/
enum {
	MDFLD_DSI_PANEL_CONNECTED = 0,
	MDFLD_DSI_PANEL_DISCONNECTED,
};

enum {
	MDFLD_DSI_PANEL_POWER_ON = 0,
	MDFLD_DSI_PANEL_POWER_OFF,
};

struct panel_info {
	u32 width_mm;
	u32 height_mm;

	/*other infos*/
};

/**
 *Panel specific callbacks.
 *
 *@encoder_funcs: a pointer to drm encoder functions
 *@encoder_helper_funcs: a pointer to helper functions of drm encoder which is
 *assigned to this panel.
 *@get_config_mode: return the fixed display mode of panel.
 *@update_fb: command mode panel only. update on-panel framebuffer.
 *@get_panel_info: return panel information. such as physical size.
 *@reset: panel hard reset.
 *@drv_ic_init: initialize panel driver IC and additional HW initialization.
 *@detect: return the panel physical connection status.
 *@dsi_controller_init: Initialize MIPI IP for this panel.
 *@get_panel_power_state: return panel power state.
 *@power_on: turn on panel. e.g. send a TURN_ON special packet.
 *@power_off: turn off panel. e.g. send a SHUT_DOWN special packet.
 *
 *When adding a new panel, the driver code should implement these callbacks
 *according to corresponding panel specs. DPI and DBI implementation will
 *call these callbacks to take the specific actions for the new panel.
 */
struct panel_funcs {
	const struct drm_encoder_funcs* encoder_funcs;
	const struct drm_encoder_helper_funcs* encoder_helper_funcs;
	struct drm_display_mode* (*get_config_mode)(struct drm_device*);
	void (*update_fb)(struct mdfld_dsi_dbi_output*, int);
	int (*get_panel_info)(struct drm_device *, int, struct panel_info *);
	int (*reset)(struct mdfld_dsi_config *dsi_config, int reset_from);
	int (*drv_ic_init)(struct mdfld_dsi_config *dsi_config, int pipe);
	int (*detect)(struct mdfld_dsi_config *dsi_config, int pipe);
	void (*dsi_controller_init)(struct mdfld_dsi_config *dsi_config,
				int pipe, int update);
	int (*get_panel_power_state)(struct mdfld_dsi_config *dsi_config,
				int pipe);
	int (*power_on)(struct mdfld_dsi_config *dsi_config);
	int (*power_off)(struct mdfld_dsi_config *dsi_config);
	int (*set_brightness)(struct mdfld_dsi_config *dsi_config, int level);
	void (*disp_control_init)(struct drm_device *);
	bool (*esd_detection)(struct mdfld_dsi_config *dsi_config);
	void (*get_reset_delay_time)(int *pdelay_between_dispaly_island_off_on,
			int *pdelay_after_reset_gpio_toggle);
};

void mdfld_output_init(struct drm_device* dev);

void init_panel(struct drm_device* dev, int mipi_pipe, enum panel_type p_type);
enum panel_type get_panel_type(struct drm_device *dev, int pipe);
int is_panel_vid_or_cmd(struct drm_device *dev);
#endif


