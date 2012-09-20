/*
 * Copyright © 2010 Intel Corporation
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
 * Authors:
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 * Gideon Eaton <eaton.
 * Scott Rowe <scott.m.rowe@intel.com>
 */

#include "displays/tmd_vid.h"
#include "mdfld_dsi_dpi.h"
#include "psb_intel_reg.h"
#include "mdfld_dsi_pkg_sender.h"

struct drm_display_mode *tmd_vid_get_config_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false;	/*Disable GCT for now */

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	mode->hdisplay = 480;
	mode->vdisplay = 800;
	mode->hsync_start = 496;
	mode->hsync_end = 504;
	mode->htotal = 512;
	mode->vsync_start = 804;
	mode->vsync_end = 806;
	mode->vtotal = 808;
	mode->clock = 33264;

	PSB_DEBUG_ENTRY("[DISPLAY]: hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("[DISPLAY]: vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("[DISPLAY]: HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("[DISPLAY]: HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("[DISPLAY]: htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("[DISPLAY]: VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("[DISPLAY]: VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("[DISPLAY]: vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("[DISPLAY]: clock is %d\n", mode->clock);
#else
	if (use_gct) {
		PSB_DEBUG_ENTRY("gct find MIPI panel.\n");

		mode->hdisplay = (ti->hactive_hi << 8) | ti->hactive_lo;
		mode->vdisplay = (ti->vactive_hi << 8) | ti->vactive_lo;
		mode->hsync_start = mode->hdisplay +
		    ((ti->hsync_offset_hi << 8) | ti->hsync_offset_lo);
		mode->hsync_end = mode->hsync_start +
		    ((ti->hsync_pulse_width_hi << 8) |
		     ti->hsync_pulse_width_lo);
		mode->htotal = mode->hdisplay + ((ti->hblank_hi << 8) |
						 ti->hblank_lo);
		mode->vsync_start =
		    mode->vdisplay + ((ti->vsync_offset_hi << 8) |
				      ti->vsync_offset_lo);
		mode->vsync_end =
		    mode->vsync_start + ((ti->vsync_pulse_width_hi << 8) |
					 ti->vsync_pulse_width_lo);
		mode->vtotal = mode->vdisplay +
		    ((ti->vblank_hi << 8) | ti->vblank_lo);
		mode->clock = ti->pixel_clock * 10;

		PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
		PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
		PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
		PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
		PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
		PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
		PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
		PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
		PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);
	} else {
		mode->hdisplay = 480;
		mode->vdisplay = 854;
		mode->hsync_start = 487;
		mode->hsync_end = 490;
		mode->htotal = 499;
		mode->vsync_start = 861;
		mode->vsync_end = 865;
		mode->vtotal = 873;
		/* 60(refresh rate) * (873 + 1) * (499+1) / 1000 / 2 */
		mode->clock = 13080;
	}
#endif
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static int tmd_vid_get_panel_info(struct drm_device *dev,
				  int pipe, struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;

	pi->width_mm = TMD_PANEL_WIDTH;
	pi->height_mm = TMD_PANEL_HEIGHT;

	return 0;
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_init_TMD_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
static u32 tmd_cmd_mcap_off[] = { 0x000000b2 };
static u32 tmd_cmd_enable_lane_switch[] = { 0x000101ef };
static u32 tmd_cmd_set_lane_num[] = { 0x006360ef };
static u32 tmd_cmd_pushing_clock0[] = { 0x00cc2fef };
static u32 tmd_cmd_pushing_clock1[] = { 0x00dd6eef };
static u32 tmd_cmd_set_mode[] = { 0x000000b3 };
static u32 tmd_cmd_set_sync_pulse_mode[] = { 0x000961ef };
static u32 tmd_cmd_set_column[] = { 0x0100002a, 0x000000df };
static u32 tmd_cmd_set_page[] = { 0x0300002b, 0x00000055 };
static u32 tmd_cmd_set_video_mode[] = { 0x00000153 };

/*no auto_bl,need add in furture*/
static u32 tmd_cmd_enable_backlight[] = { 0x00005ab4 };
static u32 tmd_cmd_set_backlight_dimming[] = { 0x00000ebd };

void mdfld_dsi_tmd_drv_ic_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
	    = mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;
	DRM_INFO("Enter mdfld init TMD MIPI display.\n");

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return;
	}

	/*wait for 3ms */
	wait_timeout = jiffies + (3 * HZ / 1000);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_mcap_off, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_enable_lane_switch, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_set_lane_num, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_pushing_clock0, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_pushing_clock1, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_set_mode, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_set_sync_pulse_mode, 1, 0);
	mdfld_dsi_send_mcs_long_lp(sender, tmd_cmd_set_column, 2, 0);
	mdfld_dsi_send_mcs_long_lp(sender, tmd_cmd_set_page, 2, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_set_video_mode, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_enable_backlight, 1, 0);
	mdfld_dsi_send_gen_long_lp(sender, tmd_cmd_set_backlight_dimming, 1, 0);

	/*wait for a frame. ~20ms */
	wait_timeout = jiffies + HZ / 50;
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
}

static void
mdfld_dsi_tmd_dsi_controller_init(struct mdfld_dsi_config *dsi_config,
				  int pipe, int update)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	int lane_count = dsi_config->lane_count;
	u32 val = 0;
	u32 mipi_control_reg = dsi_config->regs.mipi_control_reg;
	u32 intr_en_reg = dsi_config->regs.intr_en_reg;
	u32 hs_tx_timeout_reg = dsi_config->regs.hs_tx_timeout_reg;
	u32 lp_rx_timeout_reg = dsi_config->regs.lp_rx_timeout_reg;
	u32 turn_around_timeout_reg = dsi_config->regs.turn_around_timeout_reg;
	u32 device_reset_timer_reg = dsi_config->regs.device_reset_timer_reg;
	u32 high_low_switch_count_reg =
	    dsi_config->regs.high_low_switch_count_reg;
	u32 init_count_reg = dsi_config->regs.init_count_reg;
	u32 eot_disable_reg = dsi_config->regs.eot_disable_reg;
	u32 lp_byteclk_reg = dsi_config->regs.lp_byteclk_reg;
	u32 clk_lane_switch_time_cnt_reg =
	    dsi_config->regs.clk_lane_switch_time_cnt_reg;
	u32 video_mode_format_reg = dsi_config->regs.video_mode_format_reg;
	u32 dphy_param_reg = dsi_config->regs.dphy_param_reg;
	u32 dsi_func_prg_reg = dsi_config->regs.dsi_func_prg_reg;

	PSB_DEBUG_ENTRY("%s: initializing dsi controller on pipe %d\n",
			__func__, pipe);

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0x9f880;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x46;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150c3408;

	/*setup video mode format */
	val = 0;
	val = dsi_config->video_mode | DSI_DPI_COMPLETE_LAST_LINE;
	hw_ctx->video_mode_format = val;

	/*set up func_prg */
	val = 0;
	val |= lane_count;
	val |= dsi_config->channel_num << DSI_DPI_VIRT_CHANNEL_OFFSET;

	switch (dsi_config->bpp) {
	case 16:
		val |= DSI_DPI_COLOR_FORMAT_RGB565;
		break;
	case 18:
		val |= DSI_DPI_COLOR_FORMAT_RGB666;
		break;
	case 24:
		val |= DSI_DPI_COLOR_FORMAT_RGB888;
		break;
	default:
		DRM_ERROR("unsupported color format, bpp = %d\n",
			  dsi_config->bpp);
	}
	hw_ctx->dsi_func_prg = val;

	if (update) {
		REG_WRITE(dphy_param_reg, hw_ctx->dphy_param);
		REG_WRITE(mipi_control_reg, hw_ctx->mipi_control);
		REG_WRITE(intr_en_reg, hw_ctx->intr_en);
		REG_WRITE(hs_tx_timeout_reg, hw_ctx->hs_tx_timeout);
		REG_WRITE(lp_rx_timeout_reg, hw_ctx->lp_rx_timeout);
		REG_WRITE(turn_around_timeout_reg, hw_ctx->turn_around_timeout);
		REG_WRITE(device_reset_timer_reg, hw_ctx->device_reset_timer);
		REG_WRITE(high_low_switch_count_reg,
			  hw_ctx->high_low_switch_count);
		REG_WRITE(init_count_reg, hw_ctx->init_count);
		REG_WRITE(eot_disable_reg, hw_ctx->eot_disable);
		REG_WRITE(lp_byteclk_reg, hw_ctx->lp_byteclk);
		REG_WRITE(clk_lane_switch_time_cnt_reg,
			  hw_ctx->clk_lane_switch_time_cnt);
		REG_WRITE(video_mode_format_reg, hw_ctx->video_mode_format);
		REG_WRITE(dsi_func_prg_reg, hw_ctx->dsi_func_prg);
	}
}

static int mdfld_dsi_tmd_detect(struct mdfld_dsi_config *dsi_config, int pipe)
{
	int ret, status;
	u32 data = 0;
	u32 dspcntr_reg = dsi_config->regs.dspcntr_reg;
	u32 pipeconf_reg = dsi_config->regs.pipeconf_reg;
	u32 mipi_reg = dsi_config->regs.mipi_reg;
	u32 device_ready_reg = dsi_config->regs.device_ready_reg;
	u32 dpll_reg = dsi_config->regs.dpll_reg;
	u32 fp_reg = dsi_config->regs.fp_reg;
	int mipi_enabled = 0;
	int dsi_controller_enabled = 0;
	struct mdfld_dsi_pkg_sender *sender =
	    mdfld_dsi_get_pkg_sender(dsi_config);
	struct drm_device *dev = dsi_config->dev;
	u32 val;
	u32 tmp;
	int retry;
	int pipe0_enabled;
	int pipe2_enabled;

	dsi_config->dsi_hw_context.pll_bypass_mode = 0;

	if (REG_READ(mipi_reg) & BIT31)
		mipi_enabled = 1;

	if (REG_READ(device_ready_reg) & BIT0)
		dsi_controller_enabled = 1;

	/*reconfig if dsi controller is active */
	if (mipi_enabled || dsi_controller_enabled) {
		tmp = REG_READ(pipeconf_reg);
		val = REG_READ(dspcntr_reg) & ~BIT31;
		REG_WRITE(dspcntr_reg, val);
		val = tmp | 0x000c0000;
		REG_WRITE(pipeconf_reg, val);

		val = REG_READ(pipeconf_reg) & ~BIT31;
		REG_WRITE(pipeconf_reg, val);

		retry = 100000;
		while (--retry && (REG_READ(pipeconf_reg) * BIT30))
			udelay(5);

		if (!retry)
			DRM_ERROR("Failed to disable pipe\n");

		/*shutdown panel */
		mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					      MDFLD_DSI_DPI_SPK_SHUT_DOWN);

		/*disable DSI controller */
		REG_WRITE(device_ready_reg, 0x0);

		/*Disable mipi port */
		REG_WRITE(mipi_reg, REG_READ(mipi_reg) & ~BIT31);

		/*Disable DSI PLL */
		pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
		pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

		if (!pipe0_enabled && !pipe2_enabled) {
			REG_WRITE(dpll_reg, 0x0);
			/*power gate pll */
			REG_WRITE(dpll_reg, BIT30);
		}
	}

	/*enable DSI PLL */
	if (!(REG_READ(dpll_reg) & BIT31)) {
		REG_WRITE(dpll_reg, 0x0);
		REG_WRITE(fp_reg, 0xc1);
		REG_WRITE(dpll_reg, ((0x800000) & ~BIT30));
		udelay(2);
		val = REG_READ(dpll_reg);
		REG_WRITE(dpll_reg, (val | BIT31));

		/*wait for PLL lock on pipe */
		retry = 10000;
		while (--retry && !(REG_READ(PIPEACONF) & BIT29))
			udelay(3);
		if (!retry)
			DRM_ERROR("PLL failed to lock on pipe\n");
	}

	/*reconfig mipi port lane configuration */
	REG_WRITE(mipi_reg, REG_READ(mipi_reg) | MDFLD_DSI_DATA_LANE_2_2
		  | PASS_FROM_SPHY_TO_AFE);

	/*re-configure DSI controller */
	mdfld_dsi_tmd_dsi_controller_init(dsi_config, pipe, 1);

	/*enable DSI controller */
	REG_WRITE(device_ready_reg, 0x1);

	/*init drvIC */
	mdfld_dsi_tmd_drv_ic_init(dsi_config, pipe);

	ret = mdfld_dsi_get_power_mode(dsi_config,
				       &data, MDFLD_DSI_LP_TRANSMISSION);
	if (ret)
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	else
		status = MDFLD_DSI_PANEL_CONNECTED;

	/*Disable controller */
	REG_WRITE(device_ready_reg, 0x0);

	/*Disable DSI PLL */
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(dpll_reg, 0x0);
		/*power gate pll */
		REG_WRITE(dpll_reg, BIT30);
	}

	PSB_DEBUG_ENTRY("TMD on pipe %d is %s\n", pipe,
			status ? "disconnected" : "connected");

	return status;
}

static int
mdfld_dsi_tmd_get_power_state(struct mdfld_dsi_config *dsi_config, int pipe)
{
	PSB_DEBUG_ENTRY("Getting power state...");

	/*All panels have been turned off during panel detection */
	return MDFLD_DSI_PANEL_POWER_OFF;
}

static int mdfld_dsi_tmd_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
	    mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_tmd_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
	    mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_tmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
	    mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_value;

	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_value = ((level * 0xff) / 100) & 0xff;

	/*send backlight control packet */
	mdfld_dsi_send_mcs_short_lp(sender,
				    tmd_write_display_brightness,
				    backlight_value, 1, MDFLD_DSI_SEND_PACKAGE);

	return 0;
}

/*TMD DPI encoder helper funcs*/
static const
struct drm_encoder_helper_funcs mdfld_tmd_dpi_encoder_helper_funcs = {
	.save = mdfld_dsi_dpi_save,
	.restore = mdfld_dsi_dpi_restore,
	.dpms = mdfld_dsi_dpi_dpms,
	.mode_fixup = mdfld_dsi_dpi_mode_fixup,
	.prepare = mdfld_dsi_dpi_prepare,
	.mode_set = mdfld_dsi_dpi_mode_set,
	.commit = mdfld_dsi_dpi_commit,
};

/*TPO DPI encoder funcs*/
static const struct drm_encoder_funcs mdfld_tmd_dpi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

void tmd_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->encoder_funcs = &mdfld_tmd_dpi_encoder_funcs;
	p_funcs->encoder_helper_funcs = &mdfld_tmd_dpi_encoder_helper_funcs;
	p_funcs->get_config_mode = &tmd_vid_get_config_mode;
	p_funcs->update_fb = NULL;
	p_funcs->get_panel_info = tmd_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_tmd_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_tmd_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_tmd_detect;
	p_funcs->get_panel_power_state = mdfld_dsi_tmd_get_power_state;
	p_funcs->power_on = mdfld_dsi_tmd_power_on;
	p_funcs->power_off = mdfld_dsi_tmd_power_off;
	p_funcs->set_brightness = mdfld_dsi_tmd_set_brightness;
}
