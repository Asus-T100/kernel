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
 * Jackie Li <yaodong.li@intel.com>
 * Gideon Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 * Ivan Chou <ivan.y.chou@intel.com>
 * Austin Hu <austin.hu@intel.com>
 */

#include "displays/auo_sc1_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"

#define GPIO_MIPI_PANEL_RESET 128

/* ************************************************************************* *\
 * FUNCTION: mdfld_dsi_sc1_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
/* [SC1] Setting */
static u32 sc1_mcs_protect_on[] = {0x000003b0};
static u32 sc1_mcs_protect_off[] = {0x000004b0};
static u32 sc1_enter_sleep_mode[] = {0x00000010};
static u32 sc1_exit_sleep_mode[] = {0x00000011};
/* static u32 sc1_set_brightness_max[] = {0x0000ff51}; */
static u32 sc1_set_brightness[] = {0x00000051};
static u32 sc1_select_CABC_mode[] = {0x00000355};
static u32 sc1_enable_CABC_bl_off[] = {0x00002853};
static u32 sc1_enable_CABC_bl_on[] = {0x00002C53};
/* static u32 sc1_set_tearing_effect_on[] = {0x00000035}; */
static u32 sc1_set_display_on[] = {0x00000029};
static u32 sc1_set_display_off[] = {0x00000028};
static u32 sc1_enter_low_power_mode[] = {0x000001b1};

int mdfld_dsi_sc1_ic_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/*wait for 5ms*/
	wait_timeout = jiffies + (HZ / 200);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
	/* Now In Sleep Mode */
}

static void
mdfld_dsi_sc1_dsi_controller_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	int lane_count = dsi_config->lane_count;
	u32 mipi_control_reg = dsi_config->regs.mipi_control_reg;
	u32 intr_en_reg = dsi_config->regs.intr_en_reg;
	u32 hs_tx_timeout_reg = dsi_config->regs.hs_tx_timeout_reg;
	u32 lp_rx_timeout_reg = dsi_config->regs.lp_rx_timeout_reg;
	u32 turn_around_timeout_reg =
		dsi_config->regs.turn_around_timeout_reg;
	u32 device_reset_timer_reg =
		dsi_config->regs.device_reset_timer_reg;
	u32 high_low_switch_count_reg =
		dsi_config->regs.high_low_switch_count_reg;
	u32 init_count_reg = dsi_config->regs.init_count_reg;
	u32 eot_disable_reg = dsi_config->regs.eot_disable_reg;
	u32 lp_byteclk_reg = dsi_config->regs.lp_byteclk_reg;
	u32 clk_lane_switch_time_cnt_reg =
		dsi_config->regs.clk_lane_switch_time_cnt_reg;
	u32 video_mode_format_reg =
		dsi_config->regs.video_mode_format_reg;
	u32 dphy_param_reg = dsi_config->regs.dphy_param_reg;
	u32 dsi_func_prg_reg = dsi_config->regs.dsi_func_prg_reg;

	PSB_DEBUG_ENTRY("%s: initializing dsi controller on pipe %d\n",
			__func__, pipe);

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xff;
	hw_ctx->high_low_switch_count = 0x25;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150a600f;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | lane_count);
}

static int mdfld_dsi_sc1_detect(struct mdfld_dsi_config *dsi_config,
				int pipe)
{
	int status;
#ifndef CONFIG_DRM_DPMS
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
#endif

	printk(KERN_ALERT"%s\n", __func__);

	if (pipe == 0) {
		/*reconfig lane configuration*/
		/* [SC1] in bb2 is 3 */
		dsi_config->lane_count = 2;
		/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz.  Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		status = MDFLD_DSI_PANEL_CONNECTED;
#ifndef CONFIG_DRM_DPMS
		if (ctx->pll_bypass_mode) {
			uint32_t dpll = 0;
			struct drm_device *dev = dsi_config->dev;
			struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;

			REG_WRITE(regs->dpll_reg, dpll);
			if (ctx->cck_div)
				dpll = dpll | BIT11;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT12;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT13;
			REG_WRITE(regs->dpll_reg, dpll);
			dpll = dpll | BIT31;
			REG_WRITE(regs->dpll_reg, dpll);
		}
#endif
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_sc1_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*change power state*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_exit_sleep_mode, 4, 0);

	msleep(120);

	/*enable CABC with backlight off*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_select_CABC_mode, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_off, 4, 0);

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_display_on, 4, 0);

	msleep(21);

	/*enable BLON , CABC*/
	if (drm_psb_enable_cabc) {
		mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_on, 4, 0);
		printk(KERN_ALERT "enable SC1 cabc\n");
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_sc1_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*change power state here*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_display_off, 4, 0);

	/*disable BLCON, disable CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_off, 4, 0);
	printk(KERN_ALERT "disable SC1 cabc\n");

	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, sc1_enter_sleep_mode, 4, 0);

	msleep(120);

	/*put panel into deep standby mode*/
	mdfld_dsi_send_gen_long_hs(sender, sc1_enter_low_power_mode, 4, 0);

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_on, 4, 0);
	return 0;
}

static int mdfld_dsi_sc1_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*update duty value*/
	sc1_set_brightness[0] =  (0x00000051 | (duty_val << 8));
	/* [SC1] change backlight control- brightness */
	mdfld_dsi_send_gen_long_hs(sender, sc1_set_brightness, 4, 0);

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_on, 4, 0);

	return 0;
}

static int mdfld_dsi_sc1_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int ret = 0;
	static bool b_gpio_required[PSB_NUM_PIPE] = {0};
	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (reset_from == RESET_FROM_BOOT_UP) {
		b_gpio_required[dsi_config->pipe] = false;
		if (dsi_config->pipe) {
			PSB_DEBUG_ENTRY(
				"PR2 GPIO reset for MIPIC is skipped!\n");
			goto fun_exit;
		}
		ret = gpio_request(GPIO_MIPI_PANEL_RESET, "gfx");
		if (ret) {
			DRM_ERROR(
			"Failed to request gpio %d\n", GPIO_MIPI_PANEL_RESET);
			goto err;
		}
		b_gpio_required[dsi_config->pipe] = true;

		/* for get date from panel side is not easy,
		so here use display side setting to judge
		wheather panel have enabled or not by FW */
		if ((REG_READ(regs->dpll_reg) & BIT31) &&
			(REG_READ(regs->pipeconf_reg) & BIT30) &&
			(REG_READ(regs->mipi_reg) & BIT31)) {
			PSB_DEBUG_ENTRY(
				"FW has initialized the panel, skip reset during boot up\n.");
			psb_enable_vblank(dev, dsi_config->pipe);
			goto fun_exit;
		}
	}
	if (b_gpio_required[dsi_config->pipe]) {
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);

		/*reset low level width 11ms*/
		mdelay(10);
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 1);

		/*reset time 20ms*/
		mdelay(20);
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

struct drm_display_mode *sc1_vid_get_config_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false; /*Disable GCT for now*/
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
		/* [SC1] use john's value */
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
		mode->clock = 16500;
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static int sc1_vid_get_panel_info(struct drm_device *dev,
				int pipe,
				struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;
	pi->width_mm = TMD_PANEL_WIDTH;
	pi->height_mm = TMD_PANEL_HEIGHT;

	return 0;
}

void auo_sc1_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = &sc1_vid_get_config_mode;
	p_funcs->update_fb = NULL;
	p_funcs->get_panel_info = sc1_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_sc1_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_sc1_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_sc1_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_sc1_detect;
	p_funcs->power_on = mdfld_dsi_sc1_power_on;
	p_funcs->power_off = mdfld_dsi_sc1_power_off;
	p_funcs->set_brightness = mdfld_dsi_sc1_set_brightness;
}
