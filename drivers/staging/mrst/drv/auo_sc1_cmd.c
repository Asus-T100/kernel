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
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li <yaodong.li@intel.com>
 * Gideon Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 * Ivan Chou <ivan.y.chou@intel.com>
 * Austin Hu <austin.hu@intel.com>
*/

#include "displays/auo_sc1_cmd.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define GPIO_MIPI_PANEL_RESET 128

static void
mdfld_auo_dsi_controller_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int lane_count = dsi_config->lane_count;

	PSB_DEBUG_ENTRY("%s: initializing dsi controller on pipe %d\n",
			__func__, pipe);

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x15;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->lp_byteclk = 0x0;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150c3408;
	hw_ctx->dbi_bw_ctrl = 0x820;
	if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
		hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | SEL_FLOPPED_HSTX
				| TE_TRIGGER_GPIO_PIN;
	else
		hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | TE_TRIGGER_GPIO_PIN;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0xa000 | lane_count);
}

static struct drm_display_mode *auo_cmd_get_config_mode(struct drm_device* dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	struct mrst_vbt *pVBT = &dev_priv->vbt_data;
	bool use_gct = false ;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	if (use_gct) {
		PSB_DEBUG_ENTRY("gct find MIPI panel.\n");

		mode->hdisplay = (ti->hactive_hi << 8) | ti->hactive_lo;
		mode->vdisplay = (ti->vactive_hi << 8) | ti->vactive_lo;
		mode->hsync_start = mode->hdisplay + \
				((ti->hsync_offset_hi << 8) | \
				ti->hsync_offset_lo);
		mode->hsync_end = mode->hsync_start + \
				((ti->hsync_pulse_width_hi << 8) | \
				ti->hsync_pulse_width_lo);
		mode->htotal = mode->hdisplay + ((ti->hblank_hi << 8) | \
								ti->hblank_lo);
		mode->vsync_start = \
			mode->vdisplay + ((ti->vsync_offset_hi << 8) | \
						ti->vsync_offset_lo);
		mode->vsync_end = \
			mode->vsync_start + ((ti->vsync_pulse_width_hi << 8) | \
						ti->vsync_pulse_width_lo);
		mode->vtotal = mode->vdisplay + \
				((ti->vblank_hi << 8) | ti->vblank_lo);
		mode->clock = ti->pixel_clock * 10;

	} else {
		mode->hdisplay = 540;
		mode->vdisplay = 960;
		/* HFP = 40, HSYNC = 10, HBP = 20 */
		mode->hsync_start = mode->hdisplay + 40;
		mode->hsync_end = mode->hsync_start + 10;
		mode->htotal = mode->hsync_end + 20;
		/* VFP = 4, VSYNC = 2, VBP = 4 */
		mode->vsync_start = mode->vdisplay + 4;
		mode->vsync_end = mode->vsync_start + 2;
		mode->vtotal = mode->vsync_end + 4;
		mode->vrefresh = 60;
		mode->clock = mode->vrefresh * mode->vtotal *
			mode->htotal / 1000;
	}

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static bool mdfld_auo_dsi_dbi_mode_fixup(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_display_mode *fixed_mode = auo_cmd_get_config_mode(dev);

	PSB_DEBUG_ENTRY("\n");

	if (fixed_mode) {
		adjusted_mode->hdisplay = fixed_mode->hdisplay;
		adjusted_mode->hsync_start = fixed_mode->hsync_start;
		adjusted_mode->hsync_end = fixed_mode->hsync_end;
		adjusted_mode->htotal = fixed_mode->htotal;
		adjusted_mode->vdisplay = fixed_mode->vdisplay;
		adjusted_mode->vsync_start = fixed_mode->vsync_start;
		adjusted_mode->vsync_end = fixed_mode->vsync_end;
		adjusted_mode->vtotal = fixed_mode->vtotal;
		adjusted_mode->clock = fixed_mode->clock;
		drm_mode_set_crtcinfo(adjusted_mode, CRTC_INTERLACE_HALVE_V);
		kfree(fixed_mode);
	}

	return true;
}

static int __mdfld_auo_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 cmd = 0;
	u8 param = 0;
	u8 param_set[4];
	int err = 0;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* Send DCS commands. */
	cmd = write_mem_start;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_PIPE,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = exit_sleep_mode;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	msleep(120);

	cmd = write_display_brightness;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			(u8)0xff,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_cabc;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			MOVING_IMAGE,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_display;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			BRIGHT_CNTL_BLOCK_ON | DISPLAY_DIMMING_ON,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_tear_on;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			&param,
			1,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_column_address;
	param_set[0] = 0 >> 8;
	param_set[1] = 0;
	param_set[2] = 539 >> 8;
	param_set[3] = (u8)539;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			param_set,
			4,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_page_addr;
	param_set[0] = 0 >> 8;
	param_set[1] = 0;
	param_set[2] = 959 >> 8;
	param_set[3] = (u8)959;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			param_set,
			4,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_mem_start;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_PIPE,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_display_on;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_display;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			BRIGHT_CNTL_BLOCK_ON | DISPLAY_DIMMING_ON |
			BACKLIGHT_ON,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	return err;
}

static int __mdfld_auo_dsi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 cmd = 0;
	int err = 0;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*enter sleep mode*/
	cmd = enter_sleep_mode;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	msleep(120);

	/*set display off*/
	cmd = set_display_off;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	return err;
}

/*
 * Update the DBI MIPI Panel Frame Buffer.
 */
static void auo_dsi_dbi_update_fb(struct mdfld_dsi_dbi_output *dbi_output,
		int pipe)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;
	u32 dpll_reg = MRST_DPLL_A;
	u32 dspcntr_reg = DSPACNTR;
	u32 pipeconf_reg = PIPEACONF;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dspsurf_reg = DSPASURF;

	if (!dev_priv->dsi_init_done)
		return;

	/* if mode setting on-going, back off */
	if ((dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
			(psb_crtc &&
			 (psb_crtc->mode_flags & MODE_SETTING_ON_GOING)) ||
			!(dbi_output->mode_flags & MODE_SETTING_ENCODER_DONE))
		return;

	if (pipe == 2) {
		dspcntr_reg = DSPCCNTR;
		pipeconf_reg = PIPECCONF;
		dsplinoff_reg = DSPCLINOFF;
		dspsurf_reg = DSPCSURF;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return;
	}

	/* check DBI FIFO status */
	if (!(REG_READ(dpll_reg) & DPLL_VCO_ENABLE) ||
	   !(REG_READ(dspcntr_reg) & DISPLAY_PLANE_ENABLE) ||
	   !(REG_READ(pipeconf_reg) & DISPLAY_PLANE_ENABLE)) {
		goto update_fb_out0;
	}

	/* refresh plane changes */
	REG_WRITE(dsplinoff_reg, REG_READ(dsplinoff_reg));
	REG_WRITE(dspsurf_reg, REG_READ(dspsurf_reg));
	REG_READ(dspsurf_reg);

	mdfld_dsi_send_dcs(sender,
			   write_mem_start,
			   NULL,
			   0,
			   CMD_DATA_SRC_PIPE,
			   MDFLD_DSI_SEND_PACKAGE);

	dbi_output->dsr_fb_update_done = true;

update_fb_out0:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static int auo_cmd_get_panel_info(struct drm_device *dev,
		int pipe, struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;

	pi->width_mm = AUO_PANEL_WIDTH;
	pi->height_mm = AUO_PANEL_HEIGHT;

	return 0;
}

static int mdfld_auo_dsi_cmd_detect(struct mdfld_dsi_config *dsi_config,
		int pipe)
{
	int status;

	PSB_DEBUG_ENTRY("Detecting panel %d connection status....\n", pipe);
	/* dsi_config->dsi_hw_context.pll_bypass_mode = 0; */

	if (pipe == 0) {
		/* reconfig lane configuration */
		/* [SC1] in bb2 is 3 */
		dsi_config->lane_count = 2;
		/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz. Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return 0;
}

static int mdfld_auo_dsi_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_value;
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_value = ((level * 0xff) / 100) & 0xff;

	param[0] = backlight_value;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
				 write_display_brightness,
				 param,
				 3,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);

	if (err)
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);

	return 0;
}

static int mdfld_auo_dsi_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	static bool b_gpio_required[PSB_NUM_PIPE] = {0};
	int ret = 0;

	if (reset_from == RESET_FROM_BOOT_UP) {
		b_gpio_required[dsi_config->pipe] = false;
		if (dsi_config->pipe) {
			PSB_DEBUG_ENTRY("PR2 GPIO reset for MIPIC is"
					"skipped!\n");
			goto fun_exit;
		}

		ret = gpio_request(GPIO_MIPI_PANEL_RESET, "gfx");
		if (ret) {
			DRM_ERROR("Failed to request gpio %d\n",
					GPIO_MIPI_PANEL_RESET);
			goto err;
		}

		b_gpio_required[dsi_config->pipe] = true;

		goto fun_exit;
	}

	if (b_gpio_required[dsi_config->pipe]) {
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);

		/* reset low level width 20ms */
		msleep(20);

		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 1);

		/* reset time 20ms */
		msleep(20);
	} else {
		PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	}

fun_exit:
	if (b_gpio_required[dsi_config->pipe])
		PSB_DEBUG_ENTRY("pr2 panel reset successfull.");
	return 0;

err:
	gpio_free(GPIO_MIPI_PANEL_RESET);
	PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	return 0;
}

void auo_sc1_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	p_funcs->get_config_mode = &auo_cmd_get_config_mode;
	p_funcs->update_fb = auo_dsi_dbi_update_fb;
	p_funcs->get_panel_info = auo_cmd_get_panel_info;
	p_funcs->reset = mdfld_auo_dsi_panel_reset;
	p_funcs->drv_ic_init = NULL;
	p_funcs->dsi_controller_init = mdfld_auo_dsi_controller_init;
	p_funcs->detect = mdfld_auo_dsi_cmd_detect;
	p_funcs->set_brightness = mdfld_auo_dsi_cmd_set_brightness;
	p_funcs->power_on = __mdfld_auo_dsi_power_on;
	p_funcs->power_off = __mdfld_auo_dsi_power_off;
}
