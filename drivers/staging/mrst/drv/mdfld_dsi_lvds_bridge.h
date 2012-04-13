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

#define GPIOPWMCTRL	0x38F
#define PWM0CLKDIV0	0x62 /* low byte */
#define PWM0CLKDIV1	0x61 /* high byte */

#define SYSTEMCLK	19200000UL /* 19.2 MHz */
#define PWM_FREQUENCY	9600 /* Hz */

/* Panel CABC registers */
#define PANEL_PWM_CONTROL	0x90
#define PANEL_FREQ_DIVIDER_HI	0x91
#define PANEL_FREQ_DIVIDER_LO	0x92
#define PANEL_DUTY_CONTROL	0x93
#define PANEL_MODIFY_RGB	0x94
#define PANEL_FRAMERATE_CONTROL	0x96
#define PANEL_PWM_MIN		0x97
#define PANEL_PWM_REF		0x98
#define PANEL_PWM_MAX		0x99
#define PANEL_ALLOW_DISTORT	0x9A
#define PANEL_BYPASS_PWMI	0x9B

/* Panel color management registers */
#define PANEL_CM_ENABLE		0x700
#define PANEL_CM_HUE		0x701
#define PANEL_CM_SATURATION	0x702
#define PANEL_CM_INTENSITY	0x703
#define PANEL_CM_BRIGHTNESS	0x704
#define PANEL_CM_CE_ENABLE	0x705
#define PANEL_CM_PEAK_EN	0x710
#define PANEL_CM_GAIN		0x711
#define PANEL_CM_HUETABLE_START	0x730
#define PANEL_CM_HUETABLE_END	0x747 /* inclusive */

extern struct i2c_client *cmi_lcd_i2c_client;

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
