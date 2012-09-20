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
 * Austin Hu <austin.hu@intel.com>
*/

#include "displays/gi_sony_cmd.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define GPIO_MIPI_PANEL_RESET 128

enum delay_type {
	MSLEEP,
	JIFFIES_TIMEOUT,
	MDELAY
};


/* delay */
static void mdfld_ms_delay(enum delay_type d_type, int delay)
{
	unsigned long wait_timeout;

	switch (d_type) {
	case MSLEEP:
		msleep(delay);
		break;
	case JIFFIES_TIMEOUT:
		wait_timeout = jiffies + (HZ * delay / 1000);
		while (time_before_eq(jiffies, wait_timeout))
			cpu_relax();
		break;
	case MDELAY:
		mdelay(delay);
		break;
	}
}


/* ************************************************************************* *\
 * FUNCTION: mdfld_gi_l5f3_dbi_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */

static u32 gi_l5f3_set_column_add[] = {0x0100002a, 0x0000003f};
static u32 gi_l5f3_set_row_add[] = {0x0100002b, 0x000000df};
/* static u32 gi_l5f3_set_address_mode[] = {0x00004036}; */
static u32 gi_l5f3_set_address_mode[] = {0x0000D036};
static u32 gi_l5f3_set_pixel_format[] = {0x0000773a};
static u32 gi_l5f3_set_te_scanline[] = {0x00000044};
static u32 gi_l5f3_set_tear_on[] = {0x00000035};
static u32 gi_l5f3_passwd1_on[] = {0x005a5af0};
static u32 gi_l5f3_passwd2_on[] = {0x005a5af1};
static u32 gi_l5f3_dstb_on[] = {0x000001df};
static u32 gi_l5f3_set_disctl[] = {0x0f4c3bf2, 0x08082020, 0x00080800,
	0x4c000000, 0x20202020};
static u32 gi_l5f3_set_pwrctl[] = {0x000007f4, 0x00000000, 0x05440000,
	0x00054400};
static u32 gi_l5f3_set_vcmctl[] = {0x171500f5, 0x00020000, 0x15000000,
	0x00000017};
static u32 gi_l5f3_set_srcctl[] = {0x080001f6, 0x01000103, 0x00000000};
static u32 gi_l5f3_set_ifctl[] = {0x108048f7, 0x00000003};
static u32 gi_l5f3_set_panelctl[] = {0x000055f8};
static u32 gi_l5f3_set_gammasel[] = {0x000027f9};
static u32 gi_l5f3_set_pgammactl[] = {0x06040cfa, 0x24211f1e, 0x2e2f2d21,
	0x00000f2e, 0x00000000};
static u32 gi_l5f3_set_ngammactl[] = {0x0f040cfb, 0x2d2f2e2e, 0x1f212421,
	0x0000061e, 0x00000000};
static u32 gi_l5f3_set_miectl1[] = {0x108080c0};
/* static u32 gi_l5f3_set_bcmode[] = {0x000011c1}; */
static u32 gi_l5f3_set_bcmode[] = {0x000013c1};
static u32 gi_l5f3_set_wrmiectl2[] = {0x000008c2, 0x0000df01, 0x00003f01};
static u32 gi_l5f3_set_wrblctl[] = {0x201000c3};
static u32 gi_l5f3_passwd1_off[] = {0x00a5a5f0};
static u32 gi_l5f3_set_full_brightness[] = {0x0000ff51};
static u32 gi_l5f3_turn_on_backlight[] = {0x00002453};
static u32 gi_l5f3_disable_cabc[] = {0x00000055};
static u32 gi_l5f3_exit_sleep_mode[] = {0x00000011};
static u32 gi_l5f3_set_display_on[] = {0x00000029};

/* FIXME Optimize the delay time after PO.  */
static int mdfld_gi_l5f3_dbi_ic_init(struct mdfld_dsi_config *dsi_config,
		int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	/*wait for 5ms*/
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_column_add, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_row_add, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_pixel_format, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set TE scanline and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_te_scanline, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_tear_on, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set password on and wait for 10ms. */
	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_on, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_disctl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pwrctl, 16, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_vcmctl, 16, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_srcctl, 12, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ifctl, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_panelctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_gammasel, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pgammactl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ngammactl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_miectl1, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_bcmode, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrmiectl2, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrblctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_off, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_turn_on_backlight, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_disable_cabc, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_exit_sleep_mode, 4, 0);
	mdfld_ms_delay(MSLEEP, 150);

	dsi_config->drv_ic_inited = 1;

	return 0;
}

static void
mdfld_gi_sony_dsi_controller_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int lane_count = dsi_config->lane_count;

	PSB_DEBUG_ENTRY("%s: initializing dsi controller on pipe %d\n",
			__func__, pipe);

	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x28;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x0;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150a600f;
	hw_ctx->dbi_bw_ctrl = 0x820;

	if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
		hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE |
			SEL_FLOPPED_HSTX | TE_TRIGGER_GPIO_PIN;
	else
		hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | TE_TRIGGER_GPIO_PIN;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0xa000 | lane_count);
}

static struct drm_display_mode
*gi_sony_cmd_get_config_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false;

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
		mode->hdisplay = 320;
		mode->vdisplay = 480;
		/* HFP = 20, HSYNC = 8, HBP = 20 */
		mode->hsync_start = mode->hdisplay + 20;
		mode->hsync_end = mode->hsync_start + 8;
		mode->htotal = mode->hsync_end + 20;
		/* VFP = 32, VSYNC = 8, VBP = 32 */
		mode->vsync_start = mode->vdisplay + 32;
		mode->vsync_end = mode->vsync_start + 8;
		mode->vtotal = mode->vsync_end + 32;
		mode->vrefresh = 60;
		mode->clock = mode->vrefresh * mode->vtotal *
						mode->htotal / 1000;
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static int __mdfld_gi_sony_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_hw_registers *regs =
		&dsi_config->regs;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
				 exit_sleep_mode,
				 param,
				 3,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);
		goto power_err;
	}

	REG_WRITE(regs->dsplinoff_reg, dev_priv->init_screen_offset);
	REG_WRITE(regs->dspsurf_reg, dev_priv->init_screen_start);

	err = mdfld_dsi_send_dcs(sender,
				   write_mem_start,
				   NULL,
				   0,
				   CMD_DATA_SRC_PIPE,
				   MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
		goto power_err;
	}

	if (drm_psb_enable_cabc) {

		param[0] = 0x03;
		param[1] = 0x00;
		param[2] = 0x00;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_cabc,
			 param,
			 3,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - sent set_tear_on faild\n", __func__);
			goto power_err;
		}

		param[0] = 0x28;
		param[1] = 0x00;
		param[2] = 0x00;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_display,
			 param,
			 3,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - sent set_tear_on faild\n", __func__);
			goto power_err;
		}

		param[0] = 0x2c;
		param[1] = 0x00;
		param[2] = 0x00;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_display,
			 param,
			 3,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - sent set_tear_on faild\n", __func__);
			goto power_err;
		}

	}

	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 set_tear_on,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", set_tear_on);
		goto power_err;
	}

	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_display_on faild\n", __func__);
		goto power_err;
	}
	msleep(21);

power_err:
	return err;
}

static int __mdfld_gi_sony_dsi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* turn off display */
	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 set_display_off,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_display_off faild\n", __func__);
		goto power_err;
	}
	mdelay(70);

	/* disable BLCON, disable CABC */
	param[0] = 0x28;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 write_ctrl_display,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent write_ctrl_display faild\n", __func__);
		goto power_err;
	}

	/* Enter sleep mode */
	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
				 enter_sleep_mode,
				 param,
				 3,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", enter_sleep_mode);
		goto power_err;
	}
	msleep(120);

	/* DSTB, deep standby sequenc */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_passwd2_on, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_dstb_on, 4, 0);
	PSB_DEBUG_ENTRY("putting panel into deep sleep standby\n");

	msleep(50);

power_err:
	return err;
}

/*
 * Update the DBI MIPI Panel Frame Buffer.
 */
static void gi_sony_dsi_dbi_update_fb(struct mdfld_dsi_dbi_output *dbi_output,
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
	mdfld_dsi_cmds_kick_out(sender);

update_fb_out0:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static int gi_sony_cmd_get_panel_info(struct drm_device *dev,
		int pipe, struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;

	pi->width_mm = PANEL_3DOT47_WIDTH;
	pi->height_mm = PANEL_3DOT47_HEIGHT;

	return 0;
}

static int mdfld_gi_sony_dsi_cmd_detect(struct mdfld_dsi_config *dsi_config,
		int pipe)
{
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	uint32_t dpll = 0;
	int status;
	uint32_t dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("Detecting panel %d connection status....\n", pipe);
	/* dsi_config->dsi_hw_context.pll_bypass_mode = 0; */

	if (pipe == 0) {
		dsi_config->lane_count = 1;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz. Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
				(dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			status = MDFLD_DSI_PANEL_CONNECTED;
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			status = MDFLD_DSI_PANEL_DISCONNECTED;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
		dsi_config->dsi_hw_context.panel_on = 0;
	}

	return 0;
}

static int
mdfld_gi_sony_dsi_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_value;
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (dsi_config->drv_ic_inited == 0)
		return err;

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

static int mdfld_gi_sony_dsi_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	static bool b_gpio_required[PSB_NUM_PIPE] = {0};
	static bool first_reset = true;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (first_reset) {
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
		first_reset = false;
		b_gpio_required[dsi_config->pipe] = true;

	}

	if (b_gpio_required[dsi_config->pipe]) {
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);

		/* reset low level width 11ms */
		mdelay(11);

		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 1);

		/* reset time 20ms */
		mdelay(20);
	} else {
		PSB_DEBUG_ENTRY("pr2 panel reset fail.!\n");
	}

fun_exit:
	if (b_gpio_required[dsi_config->pipe])
		PSB_DEBUG_ENTRY("pr2 panel reset successfull.\n");
	return 0;

err:
	gpio_free(GPIO_MIPI_PANEL_RESET);
	PSB_DEBUG_ENTRY("pr2 panel reset fail.!\n");
	return 0;
}

int mdfld_gi_sony_power_on(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct panel_funcs *p_funcs = dbi_output->p_funcs;
	struct mdfld_dsi_hw_registers *regs = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct drm_device *dev = encoder->dev;
	int err = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("\n");


	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;


	/* set low power output hold */
	mdfld_ms_delay(MSLEEP, 5);
	REG_WRITE(regs->mipi_reg, ctx->mipi);
	mdfld_ms_delay(MSLEEP, 30);
	/* D-PHY parameter */
	REG_WRITE(regs->dphy_param_reg, ctx->dphy_param);

	/* Configure DSI controller */
	REG_WRITE(regs->mipi_control_reg, ctx->mipi_control);
	REG_WRITE(regs->intr_en_reg, ctx->intr_en);
	REG_WRITE(regs->hs_tx_timeout_reg, ctx->hs_tx_timeout);
	REG_WRITE(regs->lp_rx_timeout_reg, ctx->lp_rx_timeout);
	REG_WRITE(regs->turn_around_timeout_reg, ctx->turn_around_timeout);
	REG_WRITE(regs->device_reset_timer_reg, ctx->device_reset_timer);
	REG_WRITE(regs->high_low_switch_count_reg, ctx->high_low_switch_count);
	REG_WRITE(regs->init_count_reg, ctx->init_count);
	REG_WRITE(regs->eot_disable_reg, ctx->eot_disable);
	REG_WRITE(regs->lp_byteclk_reg, ctx->lp_byteclk);
	REG_WRITE(regs->clk_lane_switch_time_cnt_reg,
			ctx->clk_lane_switch_time_cnt);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);
	REG_WRITE(regs->dbi_bw_ctrl_reg, ctx->dbi_bw_ctrl);

	REG_WRITE(0x70184, 0);
	REG_WRITE(0x70188, 0x500);
	REG_WRITE(0x7018c, 0);
	REG_WRITE(0x70190, 0x01df013f);
	REG_WRITE(0x70180, 0x98000000);
	REG_WRITE(0x7019c, 0);
	REG_WRITE(0x6001c, 0x013f01df);

	mdfld_ms_delay(MSLEEP, 30);
	/* Enable DSI Controller */
	REG_WRITE(regs->device_ready_reg, BIT0);
	mdfld_ms_delay(MSLEEP, 30);

	/*panel drvIC init*/
	if (p_funcs->drv_ic_init)
		p_funcs->drv_ic_init(dsi_config, 0);

	REG_WRITE(0x70008, 0x84000000);
	mdfld_ms_delay(MSLEEP, 30);


	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_display_on faild\n", __func__);
		goto power_err;
	}
	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 4, 0);
	err = mdfld_dsi_send_dcs(sender,
				   write_mem_start,
				   NULL,
				   0,
				   CMD_DATA_SRC_PIPE,
				   MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
		goto power_err;
	}

	mdfld_ms_delay(MSLEEP, 50);

	/*mdfld_dbi_dsr_timer_start(dev_priv->dbi_dsr_info);*/
power_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

void gi_sony_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	p_funcs->get_config_mode = &gi_sony_cmd_get_config_mode;
	p_funcs->update_fb = gi_sony_dsi_dbi_update_fb;
	p_funcs->get_panel_info = gi_sony_cmd_get_panel_info;
	p_funcs->reset = mdfld_gi_sony_dsi_panel_reset;
	p_funcs->drv_ic_init = mdfld_gi_l5f3_dbi_ic_init;
	p_funcs->dsi_controller_init = mdfld_gi_sony_dsi_controller_init;
	p_funcs->detect = mdfld_gi_sony_dsi_cmd_detect;
	p_funcs->set_brightness = mdfld_gi_sony_dsi_cmd_set_brightness;
	p_funcs->power_on = __mdfld_gi_sony_dsi_power_on;
	p_funcs->power_off = __mdfld_gi_sony_dsi_power_off;
}
