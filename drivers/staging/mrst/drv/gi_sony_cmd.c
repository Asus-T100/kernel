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
static u32 gi_l5f3_set_address_mode[] = {0x00000036};
static u32 gi_l5f3_set_pixel_format[] = {0x0000773a};
static u32 gi_l5f3_set_te_scanline[] = {0x00000044};
static u32 gi_l5f3_set_tear_on[] = {0x00000035};
static u32 gi_l5f3_passwd1_on[] = {0x005a5af0};
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
static u32 gi_l5f3_set_bcmode[] = {0x000012c1};
static u32 gi_l5f3_set_wrmiectl2[] = {0x000008c2, 0x0000df01, 0x00003f01};
static u32 gi_l5f3_set_wrblctl[] = {0x201000c3};
static u32 gi_l5f3_passwd1_off[] = {0x00a5a5f0};
static u32 gi_l5f3_set_full_brightness[] = {0x0000ff51};
static u32 gi_l5f3_turn_on_backlight[] = {0x00002453};
static u32 gi_l5f3_disable_cabc[] = {0x00000055};
static u32 gi_l5f3_exit_sleep_mode[] = {0x00000011};
static u32 gi_l5f3_set_brightness[] = {0x00000051};
static u32 gi_l5f3_set_display_on[] = {0x00000029};

/* FIXME Optimize the delay time after PO.  */
static void mdfld_gi_l5f3_dbi_ic_init(struct mdfld_dsi_config *dsi_config,
		int pipe)
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
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_column_add, 2, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_row_add, 2, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_pixel_format, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set TE scanline and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_te_scanline, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_tear_on, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set password on and wait for 10ms. */
	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_on, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_disctl, 5, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pwrctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_vcmctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_srcctl, 3, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ifctl, 2, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_panelctl, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_gammasel, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pgammactl, 5, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ngammactl, 5, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_miectl1, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_bcmode, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrmiectl2, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrblctl, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_off, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_full_brightness, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_turn_on_backlight, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_disable_cabc, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_exit_sleep_mode, 1, 0);
	mdfld_ms_delay(MSLEEP, 150);

	/* set display on */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_display_on, 1, 0);
	mdfld_ms_delay(MSLEEP, 5);

	dsi_config->drv_ic_inited = 1;
}

static void
mdfld_gi_sony_dsi_controller_init(struct mdfld_dsi_config *dsi_config,
				int pipe, int update)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
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
	hw_ctx->dphy_param = 0x120a2b07;
	hw_ctx->dbi_bw_ctrl = 0x820;
	hw_ctx->mipi = 0x810000;

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
		mode->clock = 16500;
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static bool mdfld_gi_sony_dsi_dbi_mode_fixup(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_display_mode *fixed_mode = gi_sony_cmd_get_config_mode(dev);

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

static int __mdfld_gi_sony_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
#if 0
	u32 sc1_set_brightness_max[] = {0x0000ff51};
	u32 sc1_select_CABC_mode[] = {0x00000355};
	u32 sc1_enable_CABC_bl_on[] = {0x00002C53};
	u32 sc1_enable_CABC_bl_off[] = {0x00002853};
	u32 sc1_set_display_on[] = {0x00000029};
	u32 sc1_set_display_off[] = {0x00000028};
	u32 sc1_mcs_protect_on[] = {0x000003b0};
	u32 sc1_mcs_protect_off[] = {0x000004b0};
	u32 sc1_exit_sleep_mode[] = {0x00000011};
	u32 sc1_set_te_on[] = {0x00000035};
#endif
	int err = 0;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 1, 0); */
#if 1
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

	msleep(120);
	/*param[0] = 0xff;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
						 write_display_brightness,
						 &param,
						 3,
						 CMD_DATA_SRC_SYSTEM_MEM,
						 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_tear_on faild\n", __func__);
		goto power_err;
	}*/

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

	/*err = mdfld_dsi_send_dcs(sender,
				   write_mem_start,
				   NULL,
				   0,
				   CMD_DATA_SRC_PIPE,
				   MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
		goto power_err;
	}*/
power_err:
	return err;
#else

	/* change power state */
	mdfld_dsi_send_mcs_long_hs(sender, sc1_exit_sleep_mode, 1, 0);

	msleep(120);

	/* enable CABC with backlight off */
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_brightness_max, 1, 0);
	mdfld_dsi_send_mcs_long_hs(sender, sc1_select_CABC_mode, 1, 0);
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_off, 1, 0);
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_te_on, 1, 0);

	err = mdfld_dsi_dbi_update_area(dbi_output, 0, 0, 539, 959);
	if (err)
		DRM_ERROR("update area failed\n");

	/* set display on */
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_display_on, 1, 0);

	msleep(21);

	/* enable BLON, CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_on, 1, 0);
	/* mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_on, 1, 0); */

	/* send TURN_ON packet */
	/*err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}*/
	return err;
#endif
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

power_err:
	return err;
}

static int mdfld_gi_sony_dsi_dbi_power_on(struct drm_encoder *encoder)
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
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config, RESET_FROM_OSPM_RESUME);

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

/*
 * Power off sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int mdfld_gi_sony_dsi_dbi_power_off(struct drm_encoder *encoder)
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

static int mdfld_gi_sony_dsi_dbi_set_power(struct drm_encoder *encoder, bool on)
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

		ret = mdfld_gi_sony_dsi_dbi_power_on(encoder);
		if (ret) {
			DRM_ERROR("power on error\n");
			goto out_err;
		}

		dbi_output->dbi_panel_on = true;

		if (pipe == 2)
			dev_priv->dbi_panel_on2 = true;
		else
			dev_priv->dbi_panel_on = true;

		psb_enable_vblank(dev, pipe);

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

		psb_disable_vblank(dev, pipe);

		ret = mdfld_gi_sony_dsi_dbi_power_off(encoder);
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

static void mdfld_gi_sony_dsi_dbi_mode_set(struct drm_encoder *encoder,
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

	/*send exit_sleep_mode DCS*/
	/*ret = mdfld_dsi_dbi_send_dcs(dsi_output, exit_sleep_mode, NULL, 0,
		CMD_DATA_SRC_SYSTEM_MEM);
	if(ret) {
		DRM_ERROR("sent exit_sleep_mode faild\n");
		goto out_err;
	}

	mdelay(120);*/
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

	/*
	if (dev_priv->platform_rev_id != MDFLD_PNW_A0) {
		send set_tear_on DCS*/
		/*param = 0;
		ret = mdfld_dsi_dbi_send_dcs(dsi_output, set_tear_on, &param, 1,
			CMD_DATA_SRC_SYSTEM_MEM);

		if(ret) {
			DRM_ERROR("%s - sent set_tear_on faild\n", __func__);
			goto out_err;
		}
	}
	*/

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

static void mdfld_gi_sony_dsi_dbi_prepare(struct drm_encoder *encoder)
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

static void mdfld_gi_sony_dsi_dbi_commit(struct drm_encoder *encoder)
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

	mdfld_gi_sony_dsi_dbi_set_power(encoder, true);
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

static void mdfld_gi_sony_dsi_dbi_dpms(struct drm_encoder *encoder, int mode)
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

void mdfld_gi_sony_dsi_dbi_save(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	/*turn off*/
	mdfld_gi_sony_dsi_dbi_set_power(encoder, false);
}

void mdfld_gi_sony_dsi_dbi_restore(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	/*turn on*/
	mdfld_gi_sony_dsi_dbi_set_power(encoder, true);
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

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 1, 0);
	mdfld_dsi_send_dcs(sender,
			   write_mem_start,
			   NULL,
			   0,
			   CMD_DATA_SRC_PIPE,
			   MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_gen_fifo_ready(dev, GEN_FIFO_STAT_REG,
			HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	REG_WRITE(HS_GEN_CTRL_REG, (1 << WORD_COUNTS_POS) | GEN_READ_0);

	dbi_output->dsr_fb_update_done = true;

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
#ifndef CONFIG_DRM_DPMS
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	uint32_t dpll = 0;
#endif
	int status;

	PSB_DEBUG_ENTRY("Detecting panel %d connection status....\n", pipe);
	/* dsi_config->dsi_hw_context.pll_bypass_mode = 0; */

	if (pipe == 0) {
		/* reconfig lane configuration */
		/* [SC1] in bb2 is 3 */
		dsi_config->lane_count = 1;
		/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
		/*
		 * FIXME: JLIU7 dsi_config->lane_config =
		 * MDFLD_DSI_DATA_LANE_4_0;
		 */
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz. Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		status = MDFLD_DSI_PANEL_CONNECTED;
#ifndef CONFIG_DRM_DPMS
	PSB_DEBUG_ENTRY("ifndef CONFIG_DRM_DPMS....\n");
		if (ctx->pll_bypass_mode) {
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

	/*mdfld_dsi_send_mcs_short_hs(sender,
				write_display_brightness,
				(u8)backlight_value,
				1,
				MDFLD_DSI_SEND_PACKAGE);


	if (level == 0)
		backlight_value = 0;
	else
		backlight_value = dev_priv->mipi_ctrl_display;

		mdfld_dsi_send_mcs_short_hs(sender,
					write_ctrl_display,
					(u8)backlight_value,
					1,
					MDFLD_DSI_SEND_PACKAGE);*/

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
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct drm_device *dev = dsi_config->dev;
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
#if 0
		/*
		 * for get date from panel side is not easy, so here use
		 * display side setting to judge wheather panel have enabled or
		 * not by FW
		 */
		if ((REG_READ(regs->dpll_reg) & BIT31) &&
				(REG_READ(regs->pipeconf_reg) & BIT30) &&
				(REG_READ(regs->mipi_reg) & BIT31)) {
			PSB_DEBUG_ENTRY("FW has initialized the panel, skip"
					"reset during boot up\n.");
			psb_enable_vblank(dev, dsi_config->pipe);
			goto fun_exit;
		}
#endif
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
	unsigned long wait_timeout;
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

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 1, 0);
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

	mdfld_dbi_dsr_timer_start(dev_priv->dbi_dsr_info);
power_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/* TPO DBI encoder helper funcs */
static const struct drm_encoder_helper_funcs gi_sony_dsi_dbi_helper_funcs = {
	.save = mdfld_gi_sony_dsi_dbi_save,
	.restore = mdfld_gi_sony_dsi_dbi_restore,
	.dpms = mdfld_gi_sony_dsi_dbi_dpms,
	.mode_fixup = mdfld_gi_sony_dsi_dbi_mode_fixup,
	.prepare = mdfld_gi_sony_dsi_dbi_prepare,
	.mode_set = mdfld_gi_sony_dsi_dbi_mode_set,
	.commit = mdfld_gi_sony_dsi_dbi_commit,
};

/* TPO DBI encoder funcs */
static const struct drm_encoder_funcs gi_sony_dsi_dbi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

void gi_sony_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	p_funcs->encoder_funcs = &gi_sony_dsi_dbi_encoder_funcs;
	p_funcs->encoder_helper_funcs = &gi_sony_dsi_dbi_helper_funcs;
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
