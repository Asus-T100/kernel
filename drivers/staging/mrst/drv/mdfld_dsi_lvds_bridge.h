/*
 * Copyright © 2011 Intel Corporation
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
 */

#ifndef __MDFLD_DSI_LVDS_BRIDGE_H__
#define __MDFLD_DSI_LVDS_BRIDGE_H__

void dsi_lvds_set_bridge_reset_state(int state);
void dsi_lvds_configure_lvds_bridge(struct drm_device *dev);
void dsi_lvds_toshiba_bridge_panel_off(void);
void dsi_lvds_toshiba_bridge_panel_on(void);
void dsi_lvds_init_lvds_bridge(struct drm_device *dev);
void dsi_lvds_deinit_lvds_bridge(struct drm_device *dev);
void dsi_lvds_bridge_get_display_params(struct drm_display_mode *mode);
void dsi_lvds_resume_lvds_bridge(struct drm_device *dev);
void dsi_lvds_suspend_lvds_bridge(struct drm_device *dev);

#endif /*__MDFLD_DSI_LVDS_BRIDGE_H__*/
