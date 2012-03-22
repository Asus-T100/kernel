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
#include "psb_intel_drv.h"
#include <linux/gpio.h>
#include <linux/sfi.h>

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
static u32 h8c7_disable_cabc[] = {0x00000055};
static u32 h8c7_ic_bias_current[] = {0x826005bf, 0x00000000};
static u32 h8c7_set_power[] = {0x44007cb1, 0x0d0d0024, 0x3f3f1a12, 0x00007242};
static u32 h8c7_set_disp_reg[] = {0x05c80fb2, 0x0084080f, 0x040f05ff, 0x00000020};
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

static u32 h8c7_mcs_clumn_addr[] = {0x0200002a,0xcf};
static u32 h8c7_mcs_page_addr[] = {0x0400002b,0xff};

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

static void mdfld_h8c7_dci_ic_init(struct mdfld_dsi_config *dsi_config, int pipe)
{

	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return;
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

	/* set password and wait for 10ms. */
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_off, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_tear_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_full_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_turn_on_backlight, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_disable_cabc, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_ic_bias_current, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_power, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_disp_reg, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_command_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_mipi_ctrl, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_command_mode, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_blanking_opt_2, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_panel, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_eq_func_ltps, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_ltps_ctrl_output, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_video_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_r, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_g, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_b, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_clumn_addr, 8, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_page_addr, 8, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
}


static void
mdfld_h8c7_dci_controller_init(struct mdfld_dsi_config *dsi_config,
				int pipe, int update)
{

	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
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
	hw_ctx->dbi_bw_ctrl = 0x820;

	hw_ctx->dphy_param = 0x150c3408;
	hw_ctx->mipi = 0x810008;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0xa000 | lane_count);

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

		printk(KERN_ALERT"===hdisplay is %d\n", mode->hdisplay);
		printk(KERN_ALERT"===vdisplay is %d\n", mode->vdisplay);
		printk(KERN_ALERT"===HSS is %d\n", mode->hsync_start);
		printk(KERN_ALERT"===HSE is %d\n", mode->hsync_end);
		printk(KERN_ALERT"===htotal is %d\n", mode->htotal);
		printk(KERN_ALERT"===VSS is %d\n", mode->vsync_start);
		printk(KERN_ALERT"===VSE is %d\n", mode->vsync_end);
		printk(KERN_ALERT"===vtotal is %d\n", mode->vtotal);
		printk(KERN_ALERT"===clock is %d\n", mode->clock);

	} else {
		mode->htotal = 920;
		mode->hdisplay = 720;
		mode->hsync_start = 816;
		mode->hsync_end = 824;
		mode->vtotal = 1300;
		mode->vdisplay = 1280;
		mode->vsync_start = 1296;
		mode->vsync_end = 1298;

		mode->vrefresh = 40;
		mode->clock =  mode->vrefresh * mode->vtotal *
				mode->htotal / 1000;
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}
static bool mdfld_h8c7_dsi_dbi_mode_fixup(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_display_mode *fixed_mode = h8c7_cmd_get_config_mode(dev);

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

int mdfld_dsi_h8c7_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];

	int err = 0;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0); */

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

	msleep(120);


	param[0] = 0x00; //0x03;
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

	param[0] = 0x24; //0x28;
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

	param[0] = 0x24;//0x2c;
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

power_err:
	return err;
}

int mdfld_h8c7_dsi_dbi_power_on(struct drm_encoder *encoder)
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

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	/* HW-Reset */

	/* set low power output hold */
	REG_WRITE(regs->mipi_reg, ctx->mipi);
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

	/* Enable DSI Controller */
	REG_WRITE(regs->device_ready_reg, BIT0);

	/*
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */

	if (p_funcs && p_funcs->power_on) {
		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}
//--------------------------------- add below
/*
 * Power off sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int mdfld_h8c7_dsi_dbi_power_off(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct panel_funcs *p_funcs = dbi_output->p_funcs;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int err = 0;
	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs->set_brightness(dsi_config, 0))
		DRM_ERROR("Failed to set panel brightness\n");

	/*
	 * Different panel may have different ways to have
	 * panel turned off. Support it!
	 */
	if (p_funcs && p_funcs->power_off) {
		if (p_funcs->power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}

power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}
static int mdfld_h8c7_dsi_dbi_set_power(struct drm_encoder *encoder, bool on)
{
	int ret = 0;
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	u32 reg_offset = 0;
	int pipe = (dbi_output->channel_num == 0) ? 0 : 2;
	PSB_DEBUG_ENTRY("pipe %d : %s, panel on: %s\n", pipe, on ? "On" : "Off",
			dbi_output->dbi_panel_on ? "True" : "False");

	if (pipe == 2) {
		if (on)
			dev_priv->dual_mipi = true;
		else
			dev_priv->dual_mipi = false;

		reg_offset = MIPIC_REG_OFFSET;
	} else {
		if (!on)
			dev_priv->dual_mipi = false;
	}
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return -EAGAIN;
	}

	mutex_lock(&dsi_config->context_lock);

	if (on) {
		if (dbi_output->dbi_panel_on)
			goto out_err;

		ret = mdfld_h8c7_dsi_dbi_power_on(encoder);
		if (ret) {
			DRM_ERROR("power on error\n");
			goto out_err;
		}

		dbi_output->dbi_panel_on = true;

		if (pipe == 2)
			dev_priv->dbi_panel_on2 = true;
		else
			dev_priv->dbi_panel_on = true;

		mdfld_enable_te(dev, pipe);

		dsi_config->dsi_hw_context.panel_on = 1;
	} else {
		if (!dbi_output->dbi_panel_on && !dbi_output->first_boot)
			goto out_err;

		dbi_output->dbi_panel_on = false;
		dbi_output->first_boot = false;

		if (pipe == 2)
			dev_priv->dbi_panel_on2 = false;
		else
			dev_priv->dbi_panel_on = false;

		mdfld_disable_te(dev, pipe);

		ret = mdfld_h8c7_dsi_dbi_power_off(encoder);
		if (ret) {
			DRM_ERROR("power on error\n");
			goto out_err;
		}

		dsi_config->dsi_hw_context.panel_on = 0;
	}

out_err:
	mutex_unlock(&dsi_config->context_lock);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	if (ret)
		DRM_ERROR("failed\n");
	else
		PSB_DEBUG_ENTRY("successfully\n");

	return ret;
}
static void mdfld_h8c7_dsi_dbi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dsi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct mdfld_dsi_connector *dsi_connector = dsi_config->connector;
	int pipe = dsi_connector->pipe;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[8];
	int err = 0;
	/* values */
	u32 h_active_area = mode->hdisplay;
	u32 v_active_area = mode->vdisplay;

	PSB_DEBUG_ENTRY("type %s\n", (pipe == 2) ? "MIPI2" : "MIPI");
	PSB_DEBUG_ENTRY("h %d v %d\n", mode->hdisplay, mode->vdisplay);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return;
	}

	/* 20ms delay before sending exit_sleep_mode */
	msleep(20);

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
		goto out_err;
	}
	msleep(120);

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
		goto out_err;
	}

	/* TODO: this looks ugly, try to move it to CRTC mode setting */
	if (pipe == 2)
		dev_priv->pipeconf2 |= PIPEACONF_DSR;
	else
		dev_priv->pipeconf |= PIPEACONF_DSR;

	err = mdfld_dsi_dbi_update_area(dsi_output, 0, 0, h_active_area - 1,
			v_active_area - 1);
	if (err) {
		DRM_ERROR("update area failed\n");
		goto out_err;
	}

out_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	if (err)
		DRM_ERROR("mode set failed\n");
	else
		PSB_DEBUG_ENTRY("mode set done successfully\n");

	return;
}
static void mdfld_h8c7_dsi_dbi_prepare(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);

	PSB_DEBUG_ENTRY("\n");
	dbi_output->mode_flags |= MODE_SETTING_IN_ENCODER;
	dbi_output->mode_flags &= ~MODE_SETTING_ENCODER_DONE;

	/* mdfld_dsi_dbi_set_power(encoder, false); */
	gbdispstatus = false;
}
static void mdfld_h8c7_dsi_dbi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder =
		MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*DSI DPU was still on debugging, will remove this option later*/
#ifdef CONFIG_MDFLD_DSI_DPU
	struct psb_drm_dpu_rect rect;
#endif

	PSB_DEBUG_ENTRY("\n");
	/* mdfld_dsi_dbi_exit_dsr (dev, MDFLD_DSR_2D_3D, 0, 0); [SC1] */

	mdfld_h8c7_dsi_dbi_set_power(encoder, true);
	/* [SC1] */
	if (gbgfxsuspended)
		gbgfxsuspended = false;

	gbdispstatus = true;

	dbi_output->mode_flags &= ~MODE_SETTING_IN_ENCODER;

#ifdef CONFIG_MDFLD_DSI_DPU
	rect.x = rect.y = 0;
	rect.width = 864;
	rect.height = 480;
#endif

	if (dbi_output->channel_num == 1) {
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_2;
#ifdef CONFIG_MDFLD_DSI_DPU
		/* if dpu enabled report a fullscreen damage */
		mdfld_dbi_dpu_report_damage(dev, MDFLD_PLANEC, &rect);
#endif
	} else {
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_0;

#ifdef CONFIG_MDFLD_DSI_DPU
		mdfld_dbi_dpu_report_damage(dev, MDFLD_PLANEA, &rect);
		/* start dpu timer */
		if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
			mdfld_dbi_dpu_timer_start(dev_priv->dbi_dpu_info);
#endif
	}

	dbi_output->mode_flags |= MODE_SETTING_ENCODER_DONE;
}
static void mdfld_h8c7_dsi_dbi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	static bool bdispoff;
	PSB_DEBUG_ENTRY("%s:\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (dsi_config->drv_ic_inited == 0)
		return;

	if (mode == DRM_MODE_DPMS_ON) {
		/*
		 * FIXME: in case I am wrong!
		 * we don't need to exit dsr here to wake up plane/pipe/pll
		 * if everything goes right, hw_begin will resume them all
		 * during set_power.
		 */
		if (bdispoff)
			mdfld_dsi_dbi_exit_dsr(dev, MDFLD_DSR_2D_3D, 0, 0);

		mdfld_dsi_dbi_set_power(encoder, true);

		if (gbgfxsuspended)
			gbgfxsuspended = false;

		bdispoff = false;
		gbdispstatus = true;
	} else {
		/*
		 * I am not sure whether this is the perfect place to
		 * turn rpm on since we still have a lot of CRTC turnning
		 * on work to do.
		 */
		mdfld_dsi_dbi_set_power(encoder, false);
		bdispoff = true;
		gbdispstatus = false;
	}
}
void mdfld_h8c7_dsi_dbi_save(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	/*turn off*/
	mdfld_h8c7_dsi_dbi_set_power(encoder, false);
}

void mdfld_h8c7_dsi_dbi_restore(struct drm_encoder *encoder)
{
	if (!encoder)
		return;
	/*turn on*/
	mdfld_h8c7_dsi_dbi_set_power(encoder, true);
}
/*
 * Update the DBI MIPI Panel Frame Buffer.
 */
void mdfld_dsi_dbi_CB_ready (struct drm_device *dev, u32 mipi_command_address_reg, u32 gen_fifo_stat_reg)
{
	u32 DBI_CB_time_out_count = 0;

	/* Check MIPI Adatper command registers */
	for (DBI_CB_time_out_count = 0; DBI_CB_time_out_count < DBI_CB_TIME_OUT; DBI_CB_time_out_count++)
	{
		if (!(REG_READ(mipi_command_address_reg) & BIT0))
			break;
	}

	if (DBI_CB_time_out_count == DBI_CB_TIME_OUT)
		DRM_ERROR("Timeout waiting for DBI COMMAND status. \n");

	if (!gen_fifo_stat_reg)
		return;

	/* Check and make sure the MIPI DBI BUFFER is empty. */
	for (DBI_CB_time_out_count = 0; DBI_CB_time_out_count < DBI_CB_TIME_OUT; DBI_CB_time_out_count++)
	{
		if (REG_READ(gen_fifo_stat_reg) & DBI_FIFO_EMPTY)
			break;
	}

	if (DBI_CB_time_out_count == DBI_CB_TIME_OUT)
		DRM_ERROR("Timeout waiting for DBI FIFO empty. \n");
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
			!(dbi_output->mode_flags & MODE_SETTING_ENCODER_DONE))
	{
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

    mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_address_mode, 4, 0);
    mdelay(1);

	mdfld_dsi_send_dcs(sender,
			   write_mem_start,
			   NULL,
			   0,
			   CMD_DATA_SRC_PIPE,
			   MDFLD_DSI_SEND_PACKAGE);
//	mdfld_dsi_gen_fifo_ready(dev, GEN_FIFO_STAT_REG, DBI_FIFO_EMPTY);
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


	if (pipe == 0) {
		/*reconfig lane configuration*/
		dsi_config->lane_count = 3;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz.  Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		if (IS_CTP(dev))
			dsi_config->dsi_hw_context.pll_bypass_mode = 0;

		status = MDFLD_DSI_PANEL_CONNECTED;

#ifndef CONFIG_DRM_DPMS
	PSB_DEBUG_ENTRY("ifndef CONFIG_DRM_DPMS....\n");
		//if (ctx->pll_bypass_mode)
	     {
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

			REG_WRITE(MRST_FPA0, 0);
		}
#endif


	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;

}
int mdfld_dsi_h8c7_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	unsigned long wait_timeout;

	/*
	 * FIXME: need to check the CABA setting about brightness adjustment
	 * range.
	 */

	if (level < MIN_BRIGHTNESS_LEVEL || level > MAX_BRIGHTNESS_LEVEL) {
		printk(KERN_ALERT"Invalid brightness level: %d\n", level);
		return -EINVAL;
	}

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

	if (IS_CTP(dev)) {
		sfi_table_parse(SFI_SIG_GPIO, NULL, NULL, mdfld_mipi_panel_gpio_parse);
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
int mdfld_h8c7_cmd_power_on(struct drm_encoder *encoder)
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

	struct drm_display_mode *fixed_mode = h8c7_cmd_get_config_mode(dev);  //h8c7_cmd

	printk(KERN_ALERT"%s\n",__func__);
	PSB_DEBUG_ENTRY("\n");

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	/* set low power output hold */
	msleep(5);
	REG_WRITE(regs->mipi_reg, 0x810000); // ctx->mipi);
	msleep(30);
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
	REG_WRITE(0x70184, 0x1c2000);
	REG_WRITE(0x70188, fixed_mode->hdisplay*4);
	REG_WRITE(0x7018c, 0);
	REG_WRITE(0x70190, ((fixed_mode->vdisplay - 1)<<16) | (fixed_mode->hdisplay - 1));  //h8c7_cmd
	REG_WRITE(0x70180, 0x98000000);

	REG_WRITE(0x71400, 0x80000000);

	REG_WRITE(0x7019c, 0);
	REG_WRITE(0x6001c, ((fixed_mode->hdisplay - 1)<<16) | (fixed_mode->vdisplay - 1));

	msleep(30);
	/* Enable DSI Controller */
	REG_WRITE(regs->device_ready_reg, BIT0);
	msleep(30);

	/*panel drvIC init*/
	if (p_funcs->drv_ic_init)
		p_funcs->drv_ic_init(dsi_config, 0);

	REG_WRITE(0x70008, 0x84000000);
	msleep(30);

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

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_address_mode, 4, 0);

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

	msleep(50);

	// mdfld_dbi_dsr_timer_start(dev_priv->dbi_dsr_info);   //h8c7_cmd disable timer

power_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}


/* TPO DBI encoder helper funcs */
static const struct drm_encoder_helper_funcs h8c7_dsi_dbi_helper_funcs = {
	.save = mdfld_h8c7_dsi_dbi_save,
	.restore = mdfld_h8c7_dsi_dbi_restore,
	.dpms = mdfld_h8c7_dsi_dbi_dpms,
	.mode_fixup = mdfld_h8c7_dsi_dbi_mode_fixup,
	.prepare = mdfld_h8c7_dsi_dbi_prepare,
	.mode_set = mdfld_h8c7_dsi_dbi_mode_set,
	.commit = mdfld_h8c7_dsi_dbi_commit,
};


/*PR2 panel DPI encoder funcs*/
static const struct drm_encoder_funcs mdfld_h8c7_dpi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

void h8c7_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}
	PSB_DEBUG_ENTRY("\n");

	p_funcs->encoder_funcs = &mdfld_h8c7_dpi_encoder_funcs;
	p_funcs->encoder_helper_funcs = &h8c7_dsi_dbi_helper_funcs;
	p_funcs->get_config_mode = &h8c7_cmd_get_config_mode;
	p_funcs->update_fb = h8c7_dsi_dbi_update_fb;
	p_funcs->get_panel_info = h8c7_cmd_get_panel_info;
	p_funcs->reset = mdfld_dsi_h8c7_cmd_panel_reset;
	p_funcs->drv_ic_init = mdfld_h8c7_dci_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dci_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_cmd_detect;
	//p_funcs->get_panel_power_state = mdfld_h8c7_cmd_get_power_state;
	p_funcs->power_on = mdfld_dsi_h8c7_cmd_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_cmd_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_cmd_set_brightness;

}
