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
 */

#include "displays/h8c7_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <linux/gpio.h>
#include <linux/sfi.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>

struct h8c7_regulator_factory {
	bool h8c7_mmc2_on;
	struct regulator *regulator;
};

static struct h8c7_regulator_factory h8c7_regulator_status;


/* ************************************************************************* *\
 * FUNCTION: mdfld_h8c7_dpi_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */


static u32 h8c7_exit_sleep_mode[] = {0x00000011};
static u32 h8c7_mcs_protect_off[] = {0x9283ffb9};
static u32 h8c7_set_tear_on[] = {0x00000035};
static u32 h8c7_set_brightness[] = {0x00000051};
static u32 h8c7_set_full_brightness[] = {0x0000ff51};
static u32 h8c7_turn_on_backlight[] = {0x00002453};
static u32 h8c7_turn_off_backlight[] = {0x00000053};
static u32 h8c7_disable_cabc[] = {0x00000055};
static u32 h8c7_ic_bias_current[] = {0x826005bf, 0x00000000};
static u32 h8c7_set_power[] = {0x44007cb1, 0x0d0d00a5, 0x3f3f1a12, 0x00007242};
static u32 h8c7_set_power_dstb[] = {0x000101b1, 0x00000000, 0x00000000,
	0x00000000};
static u32 h8c7_set_disp_reg[] = {0x05c80fb2, 0x0084040f,
		0x040f05ff, 0x00000020};
static u32 h8c7_set_command_cyc[] = {0x050000b4, 0x1605a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_set_mipi_ctrl[] = {0x008312ba};
static u32 h8c7_command_mode[] = {0x000008c2};
static u32 h8c7_set_blanking_opt_2[] = {0x004000c7};
static u32 h8c7_set_panel[] = {0x000008cc};
static u32 h8c7_set_eq_func_ltps[] = {0x00000cd4};
static u32 h8c7_set_ltps_ctrl_output[] = {0x080800d5, 0x66554400, 0xcccccc77,
	0x667700cc, 0xcccc4455, 0x0000cccc};
static u32 h8c7_set_video_cyc[] = {0x040000d8, 0x1604a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_gamma_r[] = {0x3c3e3ae0, 0x3332312f, 0x0c080446, 0x110f100d,
	0x3e3a1710, 0x32312f3c, 0x08044633, 0x0f100d0c, 0x00171011};
static u32 h8c7_gamma_g[] = {0x3d3e3be1, 0x33323131, 0x0b070346, 0x110e100d,
	0x3e3b1710, 0x3231313d, 0x07034633, 0x0e100d0b, 0x00171011};
static u32 h8c7_gamma_b[] = {0x070601e2, 0x1f322a2d, 0x0e0c0540, 0x13121411,
	0x0601180f, 0x322a2d07, 0x0c05401f, 0x1214110e, 0x00180f13};
static u32 h8c7_enter_set_cabc[] = {0x1e001fc9, 0x0000001e, 0x00003e01};

static u32 h8c7_mcs_clumn_addr[] = {0x0200002a, 0xcf};
static u32 h8c7_mcs_page_addr[] = {0x0400002b, 0xff};

static u32 h8c7_mcs_protect_on[] = {0x000000b9};
static u32 h8c7_set_address_mode[] = {0x00000036};

#define MIN_BRIGHTNESS_LEVEL 60
#define MAX_BRIGHTNESS_LEVEL 100
/* MIPI display panel reset GPIO */
static int mdfld_mipi_panel_gpio_reset = -EINVAL;

static int mdfld_mipi_panel_gpio_parse(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb = (struct sfi_table_simple *)table;
	struct sfi_gpio_table_entry *entry;
	int i, num;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_gpio_table_entry);
	entry = (struct sfi_gpio_table_entry *)sb->pentry;

	for (i = 0; i < num; i++, entry++) {
		if (!strncmp(entry->pin_name, "mipi-reset", SFI_NAME_LEN))
			mdfld_mipi_panel_gpio_reset = entry->pin_no;
	}

	return 0;
}

static int  mdfld_h8c7_drv_ic_init(struct mdfld_dsi_config *dsi_config,
				int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	/*wait for 5ms*/
	wait_timeout = jiffies + (HZ / 200);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_exit_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set password*/
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_off, 4, 0);

	/* set TE on*/
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_tear_on, 4, 0);

	/* set backlight on*/
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_turn_on_backlight, 4, 0);

	/* disalble CABC*/
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_disable_cabc, 4, 0);

	mdfld_dsi_send_gen_long_lp(sender, h8c7_ic_bias_current, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_power, 16, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_disp_reg, 16, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_command_cyc, 24, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_mipi_ctrl, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_command_mode, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_blanking_opt_2, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_panel, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_eq_func_ltps, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_ltps_ctrl_output, 24, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_video_cyc, 24, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_r, 36, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_g, 36, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_b, 36, 0);
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_enter_set_cabc, 10, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_on, 4, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_clumn_addr, 8, 0);
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_page_addr, 8, 0);

	return 0;
}

static void
mdfld_h8c7_dsi_controller_init(struct mdfld_dsi_config *dsi_config, int pipe)
{

	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
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
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	/*
	 * HW team suggested 1390
	 * for bandwidth setting
	 */
	hw_ctx->dbi_bw_ctrl = 1390;
	/*
	 * fixed bug#30415/30418
	 * LP/HS Exit timing did not meet the specification
	 */
	hw_ctx->dphy_param = 0x20124E1A;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0xa000 | lane_count);

	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | TE_TRIGGER_GPIO_PIN;
	hw_ctx->mipi |= dsi_config->lane_config;
}

struct drm_display_mode *h8c7_cmd_get_config_mode(struct drm_device *dev)
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
	} else {
		mode->htotal = 920;
		mode->hdisplay = 720;
		mode->hsync_start = 816;
		mode->hsync_end = 824;
		mode->vtotal = 1300;
		mode->vdisplay = 1280;
		/*
		 * confirmed new vsync_start and vsync_end
		 * from CMI
		 */
		mode->vsync_start = 1294;
		mode->vsync_end = 1296;

		mode->vrefresh = 60;
		mode->clock =  mode->vrefresh * mode->vtotal *
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

int mdfld_dsi_h8c7_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];

	int err = 0;
	int enable_err, enabled = 0;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!IS_ERR(h8c7_regulator_status.regulator)) {

		if (!h8c7_regulator_status.h8c7_mmc2_on) {
			PSB_DEBUG_ENTRY("Before power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
			PSB_DEBUG_ENTRY("Begin to power on\n");
			h8c7_regulator_status.h8c7_mmc2_on = true;
		} else {
			DRM_ERROR("power on several times without off\n");
		}

		enabled = regulator_is_enabled(h8c7_regulator_status.regulator);
		enable_err = regulator_enable(h8c7_regulator_status.regulator);
		if (enable_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}

		/* vemmc2 need 50ms delay due to stability
		** If already enabled, no need to wait for this delay.
		** This code isn't race proof but since in addition to
		** this panel driver only touch driver is enabling this
		** regulator and does it after this function has been
		** finished, this code works well enough for now.
		*/
		if (!enabled)
			msleep(50);
		PSB_DEBUG_ENTRY("After power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*clean on-panel FB*/
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


	/*sleep out*/
	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
				 exit_sleep_mode,
				 param,
				 1,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);
		goto power_err;
	}

	mdelay(120);

	/*turn on display*/
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

power_err:
	return err;
}

static int mdfld_dsi_h8c7_cmd_power_off(struct mdfld_dsi_config *dsi_config)
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

	/*turn off backlight*/
	err = mdfld_dsi_send_mcs_long_lp(sender, h8c7_turn_off_backlight, 4, 0);
	if (err) {
		DRM_ERROR("%s: failed to turn off backlight\n", __func__);
		goto power_err;
	}

	mdelay(1);

	/*turn off display */
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

	/*Enter sleep mode */
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

	mdelay(120);

	/*enter deep standby mode*/
	err = mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_off, 4, 0);
	if (err) {
		DRM_ERROR("Failed to turn off protection\n");
		goto power_err;
	}

	err = mdfld_dsi_send_gen_long_lp(sender, h8c7_set_power_dstb, 16, 0);
	if (err) {
		DRM_ERROR("Failed to enter DSTB\n");
		goto power_dstp_err;
	}

power_dstp_err:
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_on, 4, 0);
power_err:
	if (!IS_ERR(h8c7_regulator_status.regulator)) {
		if (h8c7_regulator_status.h8c7_mmc2_on) {
			h8c7_regulator_status.h8c7_mmc2_on = false;
			PSB_DEBUG_GENERAL("Begin to power off\n");
		} else
			DRM_ERROR("power off several times without on\n");
		regulator_disable(h8c7_regulator_status.regulator);
		PSB_DEBUG_GENERAL("After power off, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	return err;
}

static void h8c7_dsi_dbi_update_fb(struct mdfld_dsi_dbi_output *dbi_output,
		int pipe)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	struct drm_device *dev = dbi_output->dev;
	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;
	u32 dpll_reg = MRST_DPLL_A;
	u32 dspcntr_reg = DSPACNTR;
	u32 pipeconf_reg = PIPEACONF;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dspsurf_reg = DSPASURF;

	/* if mode setting on-going, back off */

	if ((dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
			(psb_crtc &&
			 (psb_crtc->mode_flags & MODE_SETTING_ON_GOING)) ||
			!(dbi_output->mode_flags & MODE_SETTING_ENCODER_DONE)) {
		return;
	}

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
/*	mdfld_dsi_gen_fifo_ready(dev, GEN_FIFO_STAT_REG, DBI_FIFO_EMPTY);*/
	dbi_output->dsr_fb_update_done = true;
	mdfld_dsi_cmds_kick_out(sender);

update_fb_out0:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

int h8c7_cmd_get_panel_info(struct drm_device *dev,
				int pipe,
				struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;

	pi->width_mm = PANEL_4DOT3_WIDTH;
	pi->height_mm = PANEL_4DOT3_HEIGHT;

	return 0;
}

int mdfld_dsi_h8c7_cmd_detect(struct mdfld_dsi_config *dsi_config,
				int pipe)
{
	int status;

	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	uint32_t dpll = 0;

	PSB_DEBUG_ENTRY("\n");

	mutex_lock(&dsi_config->context_lock);

	if (pipe == 0) {
		/*reconfig lane configuration*/
		dsi_config->lane_count = 3;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz.  Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		if (IS_CTP(dev))
			dsi_config->dsi_hw_context.pll_bypass_mode = 0;

		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			mutex_unlock(&dsi_config->context_lock);
			return -EAGAIN;
		}

		if ((REG_READ(regs->device_ready_reg) & DSI_DEVICE_READY) &&
				(REG_READ(regs->dpll_reg) & DPLL_VCO_ENABLE))
			dsi_config->dsi_hw_context.panel_on = 1;
		else {
			dsi_config->dsi_hw_context.panel_on = 0;
			DRM_INFO("%s: panel not detected!", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	/*FIXME:
	 * IFIW should also apply patch 53200 to avoid screen flash
	 * otherwise, driver should do power off and on
	 */
	dsi_config->dsi_hw_context.panel_on = 0;
	mutex_unlock(&dsi_config->context_lock);
	return status;
}

int mdfld_dsi_h8c7_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	unsigned long wait_timeout;

	PSB_DEBUG_ENTRY("%s\n", __func__);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;
	h8c7_set_brightness[0] = (0x00000051 | (duty_val << 8));

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
}

int mdfld_dsi_h8c7_cmd_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int ret = 0;
	static bool b_gpio_required[PSB_NUM_PIPE] = {0};
	unsigned gpio_mipi_panel_reset = 128;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	PSB_DEBUG_ENTRY("\n");

	if (IS_CTP(dev)) {
		sfi_table_parse(SFI_SIG_GPIO,
				NULL, NULL, mdfld_mipi_panel_gpio_parse);
		gpio_mipi_panel_reset = mdfld_mipi_panel_gpio_reset;
	}

	if (reset_from == RESET_FROM_BOOT_UP) {
		b_gpio_required[dsi_config->pipe] = false;
		if (dsi_config->pipe) {
			PSB_DEBUG_ENTRY(
				"GPIO reset for MIPIC is skipped!\n");
			goto fun_exit;
		}
		ret = gpio_request(gpio_mipi_panel_reset, "gfx");
		if (ret) {
			DRM_ERROR(
			"Failed to request gpio %d\n", gpio_mipi_panel_reset);
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
		gpio_direction_output(gpio_mipi_panel_reset, 0);
		gpio_set_value_cansleep(gpio_mipi_panel_reset, 0);

		/*reset low level width 11ms*/
		mdelay(10);

		gpio_direction_output(gpio_mipi_panel_reset, 1);
		gpio_set_value_cansleep(gpio_mipi_panel_reset, 1);

		/*reset time 5ms*/
		mdelay(5);
	} else {
		PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	}
fun_exit:
	if (b_gpio_required[dsi_config->pipe])
		PSB_DEBUG_ENTRY("bapr2 panel reset successfull.");

	return 0;
err:
	gpio_free(gpio_mipi_panel_reset);
	PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	return 0;
}

void h8c7_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ena_err;

	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = h8c7_cmd_get_config_mode;
	p_funcs->update_fb = h8c7_dsi_dbi_update_fb;
	p_funcs->get_panel_info = h8c7_cmd_get_panel_info;
	p_funcs->reset = mdfld_dsi_h8c7_cmd_panel_reset;
	p_funcs->drv_ic_init = mdfld_h8c7_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_cmd_detect;
	p_funcs->power_on = mdfld_dsi_h8c7_cmd_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_cmd_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_cmd_set_brightness;

	/* Please check the file pmic_avp.c for the correct regulator name */
	h8c7_regulator_status.regulator = regulator_get(NULL, "vemmc2");
	if (IS_ERR(h8c7_regulator_status.regulator)) {
		DRM_ERROR("H8C7 device failed to get mmc regulator\n");
		return ;
	}

	h8c7_regulator_status.h8c7_mmc2_on =
		regulator_is_enabled(h8c7_regulator_status.regulator);

	if (h8c7_regulator_status.h8c7_mmc2_on) {
		ena_err = regulator_enable(h8c7_regulator_status.regulator);
		if (ena_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}
	}
}
