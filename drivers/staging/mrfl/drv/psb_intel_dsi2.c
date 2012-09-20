/*
 * Copyright © 2006-2007 Intel Corporation
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
 *	jim liu <jim.liu@intel.com>
 */

/**
 * write hysteresis values.
 */
static void mdfld_dbi_write_hysteresis(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_data_reg = HS_GEN_DATA_REG;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 *p_gen_data_val = 0;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;
	u32 i = 0;
	u8 hysteresis[68] = { write_hysteresis, 0x0f, 0x00, 0x42, 0x00,
		0x64, 0x00, 0x8c, 0x00,
		0xbf, 0x00, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0x0a, 0x00, 0x38, 0x00,
		0x50, 0x00, 0x82, 0x00,
		0xab, 0x00, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff,
	};

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	p_gen_data_val = (u32 *) hysteresis;

	for (i = 0; i < (68 / 4); i++) {
		REG_WRITE(hs_gen_data_reg, *(p_gen_data_val + i));
	}

	gen_ctrl_val = 65 << WORD_COUNTS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_LONG_WRITE;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * write display profile values.
 */
static void mdfld_dbi_write_display_profile(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_data_reg = HS_GEN_DATA_REG;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 *p_gen_data_val = 0;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;
	u32 i = 0;
	u8 profile[20] = { write_display_profile, 0x14, 0x28, 0x50,
		0x82, 0xc8, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
	};

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	p_gen_data_val = (u32 *) profile;

	for (i = 0; i < (20 / 4); i++) {
		REG_WRITE(hs_gen_data_reg, *(p_gen_data_val + i));
	}

	gen_ctrl_val = 17 << WORD_COUNTS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_LONG_WRITE;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * write KBBC profile values.
 */
static void mdfld_dbi_write_kbbc_profile(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_data_reg = HS_GEN_DATA_REG;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 *p_gen_data_val = 0;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;
	u32 i = 0;
	/* Klockwork is flagging an error on the profile initialization but
	 * I'm not sure how to resolve it's complaint
	 */
	u8 profile[20] = { write_kbbc_profile, 0xcc, 0xff, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
	};

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	p_gen_data_val = (u32 *) profile;

	for (i = 0; i < (20 / 4); i++) {
		REG_WRITE(hs_gen_data_reg, *(p_gen_data_val + i));
	}

	gen_ctrl_val = 17 << WORD_COUNTS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_LONG_WRITE;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * write gamma setting.
 */
static void mdfld_dbi_write_gamma_setting(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_data_reg = HS_GEN_DATA_REG;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 *p_gen_data_val = 0;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;
	u32 i = 0;
	u8 profile[12] = { write_gamma_setting, 0x11, 0x11, 0x81,
		0x88, 0x88, 0x88, 0x88,
		0x88, 0x88, 0x88, 0x88,
	};

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	p_gen_data_val = (u32 *) profile;

	for (i = 0; i < (12 / 4); i++) {
		REG_WRITE(hs_gen_data_reg, *(p_gen_data_val + i));
	}

	gen_ctrl_val = 9 << WORD_COUNTS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_LONG_WRITE;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * Check and see if the generic control or data buffer is empty and ready.
 */
void mdfld_dsi_gen_fifo_ready(struct drm_device *dev, u32 gen_fifo_stat_reg,
			      u32 fifo_stat)
{
	u32 GEN_BF_time_out_count = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	/* Check MIPI Adatper command registers */
	for (GEN_BF_time_out_count = 0; GEN_BF_time_out_count < GEN_FB_TIME_OUT;
	     GEN_BF_time_out_count++) {
		if ((REG_READ(gen_fifo_stat_reg) & fifo_stat) == fifo_stat)
			break;
		udelay(100);
	}

	if (GEN_BF_time_out_count == GEN_FB_TIME_OUT)
		DRM_ERROR
		    ("mdfld_dsi_gen_fifo_ready, Timeout. gen_fifo_stat_reg = 0x%x. \n",
		     gen_fifo_stat_reg);
}

/**
 * Manage the DSI MIPI keyboard and display brightness.
 */
void mdfld_dsi_brightness_init(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
	}

	/* Set default display backlight value to 85% (0xd8) */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	gen_ctrl_val = 0xd8;
	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_display_brightness << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);

	/* Set minimum brightness setting of CABC function to 20% (0x33) */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	gen_ctrl_val = 0x33;
	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_cabc_min_bright << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);

	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	mdfld_dbi_write_hysteresis(dev, pipe);

	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	mdfld_dbi_write_display_profile(dev, pipe);

	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	mdfld_dbi_write_kbbc_profile(dev, pipe);

	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	mdfld_dbi_write_gamma_setting(dev, pipe);

	/* Enable LABC */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	gen_ctrl_val =
	    BRIGHT_CNTL_BLOCK_ON | AMBIENT_LIGHT_SENSE_ON | DISPLAY_DIMMING_ON |
	    BACKLIGHT_ON | DISPLAY_BRIGHTNESS_AUTO | GAMMA_AUTO;
	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_ctrl_display << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);

	/* Enable CABC */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	gen_ctrl_val = UI_IMAGE;
	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_ctrl_cabc << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * Manage the mipi display brightness.
 */
void mdfld_dsi_brightness_control(struct drm_device *dev, int pipe, int level)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 hs_gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 gen_ctrl_val = 0;
	u32 dcsChannelNumber = dev_priv->channelNumber;

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		hs_gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
	}

	gen_ctrl_val = ((level * 0xff) / BRIGHTNESS_MAX_LEVEL) & 0xff;

	PSB_DEBUG_ENTRY("pipe = %d, gen_ctrl_val = %d.  \n", pipe,
			gen_ctrl_val);

	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_display_brightness << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	/* Set display backlight value */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);

	/* Enable LABC */
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);

	if (level == 0)
		gen_ctrl_val = 0;
	else
		gen_ctrl_val =
		    BRIGHT_CNTL_BLOCK_ON | AMBIENT_LIGHT_SENSE_ON |
		    DISPLAY_DIMMING_ON | BACKLIGHT_ON | DISPLAY_BRIGHTNESS_AUTO
		    | GAMMA_AUTO;

	gen_ctrl_val <<= MCS_PARAMETER_POS;
	gen_ctrl_val |= write_ctrl_display << MCS_COMMANDS_POS;
	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= MCS_SHORT_WRITE_1;
	REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
}

/**
 * Check and see if the DBI command buffer is empty and ready.
 */
void mdfld_dsi_dbi_CB_ready(struct drm_device *dev,
			    u32 mipi_command_address_reg, u32 gen_fifo_stat_reg)
{
	u32 DBI_CB_time_out_count = 0;

	/* Check MIPI Adatper command registers */
	for (DBI_CB_time_out_count = 0; DBI_CB_time_out_count < DBI_CB_TIME_OUT;
	     DBI_CB_time_out_count++) {
		if (!(REG_READ(mipi_command_address_reg) & BIT0))
			break;
	}

	if (DBI_CB_time_out_count == DBI_CB_TIME_OUT)
		DRM_ERROR("Timeout waiting for DBI COMMAND status. \n");

	if (!gen_fifo_stat_reg)
		return;

	/* Check and make sure the MIPI DBI BUFFER is empty. */
	for (DBI_CB_time_out_count = 0; DBI_CB_time_out_count < DBI_CB_TIME_OUT;
	     DBI_CB_time_out_count++) {
		if (REG_READ(gen_fifo_stat_reg) & DBI_FIFO_EMPTY)
			break;
	}

	if (DBI_CB_time_out_count == DBI_CB_TIME_OUT)
		DRM_ERROR("Timeout waiting for DBI FIFO empty. \n");
}

#if MDFLD_JLIU7_DSR
#if MDFLD_JLIU7_DPU
/* Currently we only support 64x64 cursors */
#define CURSOR_SIZE 64
extern void psb_dpu_combine_rect(struct psb_drm_dpu_rect *d_r_1,
				 struct psb_drm_dpu_rect *d_r_2,
				 struct psb_drm_dpu_rect *d_r_result);
#if MDFLD_JLIU7_DPU_2
/**
 * Get the damaged rect in DBI MIPI Frame Buffer.
 */
static int mdfld_dbi_damage_rect(struct drm_device *dev,
				 struct psb_drm_dpu_rect *damage_rect_0,
				 struct psb_drm_dpu_rect *damage_rect_2)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	u32 HactiveArea = dev_priv->HactiveArea;
	u32 VactiveArea = dev_priv->VactiveArea;
	u32 offset = dev_priv->offset_0;
	u32 bpp = dev_priv->bpp_0;
	int cursor_x0 = dev_priv->cursor_0_x0;
	int cursor_y0 = dev_priv->cursor_0_y0;
	int *cursor_x1 = &dev_priv->cursor_0_x1;
	int *cursor_y1 = &dev_priv->cursor_0_y1;
	int ret = -1;
	u32 stride_reg = DSPASTRIDE;
	u32 DSR_CURSOR = MDFLD_DSR_CURSOR_0;
	u32 stride_value, pipe_src_value, dsp_size_value, dsp_offset_value;
	struct psb_drm_dpu_rect damage_rect_2d_3d = dev_priv->damage_rect_2d_3d;
	struct psb_drm_dpu_rect damage_rect_cursor = { 0, 0, 0, 0 };
	struct psb_drm_dpu_rect damage_rect_tmp = { 0, 0, 0, 0 };
	struct psb_drm_dpu_rect damage_rect_tmp2 = { 0, 0, 0, 0 };
	struct psb_drm_dpu_rect *damage_rect = &damage_rect_tmp2;
	uint32_t pos = CURAPOS;
	uint32_t base = CURABASE;
	uint32_t control = CURACNTR;
	uint32_t temp = 0;
	uint32_t addr = dev_priv->cursor_addr_0;
	uint32_t cursor_cntr = dev_priv->cursor_cntr_0;

#if 0
	PSB_DEBUG_ENTRY("\n");
#endif

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, true))
		return ret;

	/* Calculate the cursor damage rect */
	if (dev_priv->dsr_fb_update & DSR_CURSOR) {
		if (cursor_x0 < *cursor_x1) {
			if (cursor_x0 < 0)
				damage_rect_cursor.x = 0;
			else
				damage_rect_cursor.x = cursor_x0;

			damage_rect_cursor.width =
			    *cursor_x1 - cursor_x0 + CURSOR_SIZE;
		} else {
			if (*cursor_x1 < 0)
				damage_rect_cursor.x = 0;
			else
				damage_rect_cursor.x = *cursor_x1;

			damage_rect_cursor.width =
			    cursor_x0 - *cursor_x1 + CURSOR_SIZE;
		}

		if (cursor_y0 < *cursor_y1) {
			if (cursor_y0 < 0)
				damage_rect_cursor.y = 0;
			else
				damage_rect_cursor.y = cursor_y0;

			damage_rect_cursor.height =
			    *cursor_y1 - cursor_y0 + CURSOR_SIZE;
		} else {
			if (*cursor_y1 < 0)
				damage_rect_cursor.y = 0;
			else
				damage_rect_cursor.y = *cursor_y1;

			damage_rect_cursor.height =
			    cursor_y0 - *cursor_y1 + CURSOR_SIZE;
		}

		ret = 0;
	} else {
		addr = 0;
		cursor_cntr = CURSOR_MODE_DISABLE;
	}

	if (!ret) {
		if ((HactiveArea - damage_rect_cursor.x) < CURSOR_SIZE)
			damage_rect_cursor.x = HactiveArea - CURSOR_SIZE;

		if ((VactiveArea - damage_rect_cursor.y) < CURSOR_SIZE)
			damage_rect_cursor.y = VactiveArea - CURSOR_SIZE;

		if ((damage_rect_cursor.x + damage_rect_cursor.width) >
		    HactiveArea)
			damage_rect_cursor.width =
			    HactiveArea - damage_rect_cursor.x;

		if ((damage_rect_cursor.y + damage_rect_cursor.height) >
		    VactiveArea)
			damage_rect_cursor.height =
			    VactiveArea - damage_rect_cursor.y;

		damage_rect_tmp = damage_rect_cursor;
	}

	/* Calculate the 2d/3d damage rect */
	if (dev_priv->dsr_fb_update & MDFLD_DSR_2D_3D) {

		if (!ret)
			psb_dpu_combine_rect(&damage_rect_cursor,
					     &damage_rect_2d_3d,
					     &damage_rect_tmp);
		else
			damage_rect_tmp = damage_rect_2d_3d;

		ret = 0;
	}

	if (!ret) {

		if (damage_rect_tmp.x < HactiveArea)
			damage_rect->x = damage_rect_tmp.x;
		else
			damage_rect->x = HactiveArea - 1;

		if (damage_rect_tmp.y < VactiveArea)
			damage_rect->y = damage_rect_tmp.y;
		else
			damage_rect->y = VactiveArea - 1;

		if ((damage_rect_tmp.x + damage_rect_tmp.width) > HactiveArea)
			damage_rect->width = HactiveArea - damage_rect_tmp.x;
		else
			damage_rect->width = damage_rect_tmp.width;

		if ((damage_rect_tmp.y + damage_rect_tmp.height) > VactiveArea)
			damage_rect->height = VactiveArea - damage_rect_tmp.y;
		else
			damage_rect->height = damage_rect_tmp.height;

		*cursor_x1 = cursor_x0;
		*cursor_y1 = cursor_y0;

		if (cursor_x0 < 0) {
			temp |= (CURSOR_POS_SIGN << CURSOR_X_SHIFT);
			cursor_x0 = -cursor_x0;
		} else {
			if (cursor_x0 > damage_rect->x)
				cursor_x0 = cursor_x0 - damage_rect->x;
			else
				cursor_x0 = 0;
		}

		if (cursor_y0 < 0) {
			temp |= (CURSOR_POS_SIGN << CURSOR_Y_SHIFT);
			cursor_y0 = -cursor_y0;
		} else {
			if (cursor_y0 > damage_rect->y)
				cursor_y0 = cursor_y0 - damage_rect->y;
			else
				cursor_y0 = 0;
		}

		temp |= ((cursor_x0 & CURSOR_POS_MASK) << CURSOR_X_SHIFT);
		temp |= ((cursor_y0 & CURSOR_POS_MASK) << CURSOR_Y_SHIFT);

		REG_WRITE(control, cursor_cntr);
		REG_WRITE(pos, temp);
		REG_WRITE(base, addr);

		stride_value = REG_READ(stride_reg);
		pipe_src_value =
		    (damage_rect->height -
		     1) | ((damage_rect->width - 1) << 16);
		dsp_size_value =
		    ((damage_rect->height - 1) << 16) | (damage_rect->width -
							 1);
		dsp_offset_value =
		    offset + damage_rect->x * bpp +
		    damage_rect->y * stride_value;

		REG_WRITE(PIPEASRC, pipe_src_value);
		REG_WRITE(DSPASIZE, dsp_size_value);
		REG_WRITE(DSPALINOFF, dsp_offset_value);
		REG_WRITE(DSPASURF, REG_READ(DSPASURF));

		REG_WRITE(PIPECSRC, pipe_src_value);
		REG_WRITE(DSPCSIZE, dsp_size_value);
		REG_WRITE(DSPCLINOFF, dsp_offset_value);
		REG_WRITE(DSPCSURF, REG_READ(DSPCSURF));

		if (damage_rect_0)
			*damage_rect_0 = *damage_rect;
		if (damage_rect_2)
			*damage_rect_2 = *damage_rect;

	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return ret;
}
#else				/* MDFLD_JLIU7_DPU_2 */
/**
 * Get the damaged rect in DBI MIPI Frame Buffer.
 */
static int mdfld_dbi_damage_rect(struct drm_device *dev, int pipe,
				 struct psb_drm_dpu_rect *damage_rect)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	u32 HactiveArea = dev_priv->HactiveArea;
	u32 VactiveArea = dev_priv->VactiveArea;
	u32 offset = dev_priv->offset_0;
	u32 bpp = dev_priv->bpp_0;
	int cursor_x0 = dev_priv->cursor_0_x0;
	int cursor_y0 = dev_priv->cursor_0_y0;
	int *cursor_x1 = &dev_priv->cursor_0_x1;
	int *cursor_y1 = &dev_priv->cursor_0_y1;
	int ret = -1;
	u32 dspsurf_reg = DSPASURF;
	u32 stride_reg = DSPASTRIDE;
	u32 pipe_src_reg = PIPEASRC;
	u32 dsp_size_reg = DSPASIZE;
	u32 dsp_offset_reg = DSPALINOFF;
	u32 DSR_CURSOR = MDFLD_DSR_CURSOR_0;
	u32 stride_value, pipe_src_value, dsp_size_value, dsp_offset_value;
	struct psb_drm_dpu_rect damage_rect_2d_3d = dev_priv->damage_rect_2d_3d;
	struct psb_drm_dpu_rect damage_rect_cursor = { 0, 0, 0, 0 };
	struct psb_drm_dpu_rect damage_rect_tmp = { 0, 0, 0, 0 };
	uint32_t pos = CURAPOS;
	uint32_t base = CURABASE;
	uint32_t control = CURACNTR;
	uint32_t temp = 0;
	uint32_t addr = dev_priv->cursor_addr_0;
	uint32_t cursor_cntr = dev_priv->cursor_cntr_0;
	bool *b_cursor_update = &dev_priv->b_cursor_update_0;

#if 0
	PSB_DEBUG_ENTRY("pipe = %d \n", pipe);
#endif

	switch (pipe) {
	case 0:
		break;
	case 2:
		HactiveArea = dev_priv->HactiveArea2;
		VactiveArea = dev_priv->VactiveArea2;
		offset = dev_priv->offset_2;
		bpp = dev_priv->bpp_2;
		cursor_x0 = dev_priv->cursor_2_x0;
		cursor_y0 = dev_priv->cursor_2_y0;
		cursor_x1 = &dev_priv->cursor_2_x1;
		cursor_y1 = &dev_priv->cursor_2_y1;
		dspsurf_reg = DSPCSURF;
		stride_reg = DSPCSTRIDE;
		pipe_src_reg = PIPECSRC;
		dsp_size_reg = DSPCSIZE;
		dsp_offset_reg = DSPCLINOFF;
		DSR_CURSOR = MDFLD_DSR_CURSOR_2;
		pos = CURCPOS;
		base = CURCBASE;
		control = CURCCNTR;
		addr = dev_priv->cursor_addr_2;
		cursor_cntr = dev_priv->cursor_cntr_2;
		b_cursor_update = &dev_priv->b_cursor_update_2;
		break;
	default:
		DRM_ERROR("%s, Illegal Pipe Number.\n", __func__);
		return ret;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, true))
		return ret;

	/* Calculate the cursor damage rect */
	if (dev_priv->dsr_fb_update & DSR_CURSOR) {
		if (cursor_x0 < *cursor_x1) {
			if (cursor_x0 < 0)
				damage_rect_cursor.x = 0;
			else
				damage_rect_cursor.x = cursor_x0;

			damage_rect_cursor.width =
			    *cursor_x1 - cursor_x0 + CURSOR_SIZE;
		} else {
			if (*cursor_x1 < 0)
				damage_rect_cursor.x = 0;
			else
				damage_rect_cursor.x = *cursor_x1;

			damage_rect_cursor.width =
			    cursor_x0 - *cursor_x1 + CURSOR_SIZE;
		}

		if (cursor_y0 < *cursor_y1) {
			if (cursor_y0 < 0)
				damage_rect_cursor.y = 0;
			else
				damage_rect_cursor.y = cursor_y0;

			damage_rect_cursor.height =
			    *cursor_y1 - cursor_y0 + CURSOR_SIZE;
		} else {
			if (*cursor_y1 < 0)
				damage_rect_cursor.y = 0;
			else
				damage_rect_cursor.y = *cursor_y1;

			damage_rect_cursor.height =
			    cursor_y0 - *cursor_y1 + CURSOR_SIZE;
		}

		*b_cursor_update = true;
		ret = 0;
	} else if (*b_cursor_update) {

		if (*cursor_x1 < 0)
			damage_rect_cursor.x = 0;
		else
			damage_rect_cursor.x = *cursor_x1;

		damage_rect_cursor.width = CURSOR_SIZE;

		if (*cursor_y1 < 0)
			damage_rect_cursor.y = 0;
		else
			damage_rect_cursor.y = *cursor_y1;

		damage_rect_cursor.height = CURSOR_SIZE;

		cursor_x0 = *cursor_x1;
		cursor_y0 = *cursor_y1;
		cursor_cntr = 0;
		ret = 0;
	}

	if (!ret) {
		if ((HactiveArea - damage_rect_cursor.x) < CURSOR_SIZE)
			damage_rect_cursor.x = HactiveArea - CURSOR_SIZE;

		if ((VactiveArea - damage_rect_cursor.y) < CURSOR_SIZE)
			damage_rect_cursor.y = VactiveArea - CURSOR_SIZE;

		if ((damage_rect_cursor.x + damage_rect_cursor.width) >
		    HactiveArea)
			damage_rect_cursor.width =
			    HactiveArea - damage_rect_cursor.x;

		if ((damage_rect_cursor.y + damage_rect_cursor.height) >
		    VactiveArea)
			damage_rect_cursor.height =
			    VactiveArea - damage_rect_cursor.y;

		damage_rect_tmp = damage_rect_cursor;
	}

	/* Calculate the 2d/3d damage rect */
	if (dev_priv->dsr_fb_update & MDFLD_DSR_2D_3D) {

		if (!ret)
			psb_dpu_combine_rect(&damage_rect_cursor,
					     &damage_rect_2d_3d,
					     &damage_rect_tmp);
		else {
			cursor_cntr = 0;
			damage_rect_tmp = damage_rect_2d_3d;
		}

		ret = 0;
	}

	if (!ret) {
		if (damage_rect_tmp.x < HactiveArea)
			damage_rect->x = damage_rect_tmp.x;
		else
			ret = -1;

		if (damage_rect_tmp.y < VactiveArea)
			damage_rect->y = damage_rect_tmp.y;
		else
			ret = -1;

		if (!ret) {
			if ((damage_rect_tmp.x + damage_rect_tmp.width) >
			    HactiveArea)
				damage_rect->width =
				    HactiveArea - damage_rect_tmp.x;
			else
				damage_rect->width = damage_rect_tmp.width;

			if ((damage_rect_tmp.y + damage_rect_tmp.height) >
			    VactiveArea)
				damage_rect->height =
				    VactiveArea - damage_rect_tmp.y;
			else
				damage_rect->height = damage_rect_tmp.height;
		}
	}

	if (!ret) {
		if ((dev_priv->dsr_fb_update & DSR_CURSOR) || *b_cursor_update) {
			*b_cursor_update = false;
			*cursor_x1 = cursor_x0;
			*cursor_y1 = cursor_y0;

			if (cursor_x0 < 0) {
				temp |= (CURSOR_POS_SIGN << CURSOR_X_SHIFT);
				cursor_x0 = -cursor_x0;
			} else {
				if (cursor_x0 > damage_rect->x)
					cursor_x0 = cursor_x0 - damage_rect->x;
				else
					cursor_x0 = 0;
			}

			if (cursor_y0 < 0) {
				temp |= (CURSOR_POS_SIGN << CURSOR_Y_SHIFT);
				cursor_y0 = -cursor_y0;
			} else {
				if (cursor_y0 > damage_rect->y)
					cursor_y0 = cursor_y0 - damage_rect->y;
				else
					cursor_y0 = 0;
			}

			temp |=
			    ((cursor_x0 & CURSOR_POS_MASK) << CURSOR_X_SHIFT);
			temp |=
			    ((cursor_y0 & CURSOR_POS_MASK) << CURSOR_Y_SHIFT);

			if (pipe == 0) {
				REG_WRITE(control, cursor_cntr);
				REG_WRITE(pos, temp);
				REG_WRITE(base, addr);
			}
		}

		stride_value = REG_READ(stride_reg);
		pipe_src_value =
		    (damage_rect->height -
		     1) | ((damage_rect->width - 1) << 16);
		dsp_size_value =
		    ((damage_rect->height - 1) << 16) | (damage_rect->width -
							 1);
		dsp_offset_value =
		    offset + damage_rect->x * bpp +
		    damage_rect->y * stride_value;

/* DRM_INFO("damage_rect, x_n_2 = %d, y_n_2 = %d, width_n_2 = %d, height_n_2 = %d. \n", damage_rect->x, damage_rect->y, damage_rect->width, damage_rect->height); */
/* DRM_INFO ("%s, p_src = 0x%x, d_size = 0x%x, off = 0x%x. \n", __FUNCTION__, pipe_src_value, dsp_size_value, dsp_offset_value); */
		REG_WRITE(pipe_src_reg, pipe_src_value);
		REG_WRITE(dsp_size_reg, dsp_size_value);
		REG_WRITE(dsp_offset_reg, dsp_offset_value);
		REG_WRITE(dspsurf_reg, REG_READ(dspsurf_reg));
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return ret;
}
#endif				/* MDFLD_JLIU7_DPU_2 */
#endif				/* MDFLD_JLIU7_DPU */

#if MDFLD_JLIU7_DPU_2
/**
 * Update the DBI MIPI Panel Frame Buffer.
 */
void mdfld_dbi_update_fb(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	u8 *p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer;
	u8 *p_DBI_commandBuffer2 = dev_priv->p_DBI_commandBuffer2;
	u32 DBI_CB_phys = dev_priv->DBI_CB_phys;
	u32 DBI_CB_phys2 = dev_priv->DBI_CB_phys2;
	u32 *pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer);
	bool *update_done = &dev_priv->dsr_fb_update_done;
#if MDFLD_JLIU7_DPU
	struct psb_drm_dpu_rect damage_rect = { 0, 0, 864, 480 };
	struct psb_drm_dpu_rect damage_rect2 = { 0, 0, 864, 480 };
#endif				/* MDFLD_JLIU7_DPU */
#if 0				/* MDFLD_PO_JLIU7 */
	static u32 count_te;
	static u32 count_update;
#endif				/* MDFLD_PO_JLIU7 */

#if 0
	PSB_DEBUG_ENTRY("\n");
#endif

	/* FIXME_JLIU7 MDFLD_PO */
	/* disable all the MIPI interrupts at the beginning. */
	/* enable all the MIPI interrupts at the end. */
	/* Make sure dbi command & data buffer are empty */
	if (*pDBI_CB_pointer != 0) {
		DRM_ERROR
		    ("mdfld_dbi_update_fb, dbi command buffer was interrupted before finished. \n");
		return;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, true))
		return;

#if 0				/* MDFLD_PO_JLIU7 */
	count_te++;
#endif				/* MDFLD_PO_JLIU7 */
	if ((dev_priv->dbi_panel_on) && (dev_priv->dbi_panel_on2)) {
		if ((!(REG_READ(MIPI_COMMAND_ADDRESS_REG) & BIT0))
		    && (REG_READ(GEN_FIFO_STAT_REG) & DBI_FIFO_EMPTY)
		    && (REG_READ(PIPEACONF) & PIPEACONF_ENABLE)
		    &&
		    (!(REG_READ(MIPI_COMMAND_ADDRESS_REG + MIPIC_REG_OFFSET) &
		       BIT0))
		    && (REG_READ(GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET) &
			DBI_FIFO_EMPTY)
		    && (REG_READ(PIPECCONF) & PIPEACONF_ENABLE)) {
			/* udelay(5000); */
			*pDBI_CB_pointer = 0;
#if MDFLD_JLIU7_DPU
			if (dev_priv->b_dpu_enable) {
				if (!
				    (mdfld_dbi_damage_rect
				     (dev, &damage_rect, &damage_rect2))) {
					/* set_column_address */
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      set_column_address;

					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      damage_rect.x >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = damage_rect.x;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.x + damage_rect.width - 1) >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.x + damage_rect.width - 1);

					*pDBI_CB_pointer = 8;
					/* set_page_addr */
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = set_page_addr;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      damage_rect.y >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = damage_rect.y;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.y + damage_rect.height - 1) >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.y + damage_rect.height - 1);

					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      set_column_address;

					*pDBI_CB_pointer = 0;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     set_column_address;

					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.x >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.x;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.x + damage_rect2.width - 1) >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.x + damage_rect2.width - 1);

					*pDBI_CB_pointer = 8;
					/* set_page_addr */
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) = set_page_addr;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.y >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.y;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.y + damage_rect2.height - 1) >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.y + damage_rect2.height - 1);

					*pDBI_CB_pointer = 16;
					/* write_mem_start */
					*(p_DBI_commandBuffer +
					  *pDBI_CB_pointer) = write_mem_start;
					*(p_DBI_commandBuffer2 +
					  *pDBI_CB_pointer) = write_mem_start;

					REG_WRITE(MIPI_COMMAND_LENGTH_REG,
						  0x010505);
					REG_WRITE(MIPI_COMMAND_LENGTH_REG +
						  MIPIC_REG_OFFSET, 0x010505);
					REG_WRITE(MIPI_COMMAND_ADDRESS_REG,
						  DBI_CB_phys | BIT0 | BIT1);
					REG_WRITE(MIPI_COMMAND_ADDRESS_REG +
						  MIPIC_REG_OFFSET,
						  DBI_CB_phys2 | BIT0 | BIT1);
					udelay(3000);
					/* udelay(5000); */
					*pDBI_CB_pointer = 0;
					*update_done = true;
				}
			} else {
#endif				/* MDFLD_JLIU7_DPU */
				/* write_mem_start */
				*(p_DBI_commandBuffer + *pDBI_CB_pointer) =
				    write_mem_start;
				*(p_DBI_commandBuffer2 + *pDBI_CB_pointer) =
				    write_mem_start;
				REG_WRITE(MIPI_COMMAND_LENGTH_REG, 0x01);
				REG_WRITE(MIPI_COMMAND_LENGTH_REG +
					  MIPIC_REG_OFFSET, 0x01);
				REG_WRITE(MIPI_COMMAND_ADDRESS_REG,
					  DBI_CB_phys | BIT0 | BIT1);
				REG_WRITE(MIPI_COMMAND_ADDRESS_REG +
					  MIPIC_REG_OFFSET,
					  DBI_CB_phys2 | BIT0 | BIT1);
				udelay(3000);
				/* udelay(5000); */
				*pDBI_CB_pointer = 0;
				*update_done = true;
#if MDFLD_JLIU7_DPU
			}
#endif				/* MDFLD_JLIU7_DPU */

#if 0				/* MDFLD_PO_JLIU7 */
			count_update++;
#endif				/* MDFLD_PO_JLIU7 */
		}
	} else if (dev_priv->dbi_panel_on) {
		if ((!(REG_READ(MIPI_COMMAND_ADDRESS_REG) & BIT0))
		    && (REG_READ(GEN_FIFO_STAT_REG) & DBI_FIFO_EMPTY)
		    && (REG_READ(PIPEACONF) & PIPEACONF_ENABLE)) {
			/* udelay(5000); */
			*pDBI_CB_pointer = 0;
#if MDFLD_JLIU7_DPU
			if (dev_priv->b_dpu_enable) {
				if (!
				    (mdfld_dbi_damage_rect
				     (dev, &damage_rect, 0))) {
					/* set_column_address */
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      set_column_address;

					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      damage_rect.x >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = damage_rect.x;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.x + damage_rect.width - 1) >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.x + damage_rect.width - 1);

					*pDBI_CB_pointer = 8;
					/* set_page_addr */
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = set_page_addr;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      damage_rect.y >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) = damage_rect.y;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.y + damage_rect.height - 1) >> 8;
					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      (damage_rect.y + damage_rect.height - 1);

					*(p_DBI_commandBuffer +
					  (*pDBI_CB_pointer)++) =
		      set_column_address;

					*pDBI_CB_pointer = 16;
					/* write_mem_start */
					*(p_DBI_commandBuffer +
					  *pDBI_CB_pointer) = write_mem_start;

					REG_WRITE(MIPI_COMMAND_LENGTH_REG,
						  0x010505);
					REG_WRITE(MIPI_COMMAND_ADDRESS_REG,
						  DBI_CB_phys | BIT0 | BIT1);
					udelay(3000);
					/* udelay(5000); */
					*pDBI_CB_pointer = 0;
					*update_done = true;
				}

			} else {
#endif				/* MDFLD_JLIU7_DPU */
				/* write_mem_start */
				*(p_DBI_commandBuffer + *pDBI_CB_pointer) =
				    write_mem_start;
				REG_WRITE(MIPI_COMMAND_LENGTH_REG, 0x01);
				REG_WRITE(MIPI_COMMAND_ADDRESS_REG,
					  DBI_CB_phys | BIT0 | BIT1);
				udelay(3000);
				/* udelay(5000); */
				*pDBI_CB_pointer = 0;
				*update_done = true;
#if MDFLD_JLIU7_DPU
			}
#endif				/* MDFLD_JLIU7_DPU */
		}
	} else if (dev_priv->dbi_panel_on2) {
		if ((!
		     (REG_READ(MIPI_COMMAND_ADDRESS_REG + MIPIC_REG_OFFSET) &
		      BIT0))
		    && (REG_READ(GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET) &
			DBI_FIFO_EMPTY)
		    && (REG_READ(PIPECCONF) & PIPEACONF_ENABLE)) {
			/* udelay(5000); */
			*pDBI_CB_pointer = 0;
#if MDFLD_JLIU7_DPU
			if (dev_priv->b_dpu_enable) {
				if (!
				    (mdfld_dbi_damage_rect
				     (dev, &damage_rect, 0))) {
					/* set_column_address */
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     set_column_address;

					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.x >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.x;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.x + damage_rect2.width - 1) >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.x + damage_rect2.width - 1);

					*pDBI_CB_pointer = 8;
					/* set_page_addr */
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) = set_page_addr;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.y >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     damage_rect2.y;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.y + damage_rect2.height - 1) >> 8;
					*(p_DBI_commandBuffer2 +
					  (*pDBI_CB_pointer)++) =
		     (damage_rect2.y + damage_rect2.height - 1);

					*pDBI_CB_pointer = 16;
					/* write_mem_start */
					*(p_DBI_commandBuffer2 +
					  *pDBI_CB_pointer) = write_mem_start;

					REG_WRITE(MIPI_COMMAND_LENGTH_REG +
						  MIPIC_REG_OFFSET, 0x010505);
					REG_WRITE(MIPI_COMMAND_ADDRESS_REG +
						  MIPIC_REG_OFFSET,
						  DBI_CB_phys | BIT0 | BIT1);
					udelay(3000);
					/* udelay(5000); */
					*pDBI_CB_pointer = 0;
					*update_done = true;
				}
			} else {
#endif				/* MDFLD_JLIU7_DPU */
				/* write_mem_start */
				*(p_DBI_commandBuffer2 + *pDBI_CB_pointer) =
				    write_mem_start;
				REG_WRITE(MIPI_COMMAND_LENGTH_REG +
					  MIPIC_REG_OFFSET, 0x01);
				REG_WRITE(MIPI_COMMAND_ADDRESS_REG +
					  MIPIC_REG_OFFSET,
					  DBI_CB_phys2 | BIT0 | BIT1);
				udelay(3000);
				/* udelay(5000); */
				*pDBI_CB_pointer = 0;
				*update_done = true;
			}

#if 0				/* MDFLD_PO_JLIU7 */
			count_update++;
#endif				/* MDFLD_PO_JLIU7 */
		}
	}
#if 0				/* MDFLD_PO_JLIU7 */
	if (((count_te % 200) == 0) || ((count_te % 201) == 0)) {
		DRM_INFO
		    ("count_te = 0x%x, count_update = 0x%x, pipe = 0x%x  \n",
		     count_te, count_update, pipe);
		DRM_INFO("dbi_panel_on = 0x%x, dbi_panel_on2 = 0x%x. \n",
			 dev_priv->dbi_panel_on, dev_priv->dbi_panel_on2);
	}
#endif				/* MDFLD_PO_JLIU7 */
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}
#else				/* MDFLD_JLIU7_DPU_2 */
/**
 * Update the DBI MIPI Panel Frame Buffer.
 */
void mdfld_dbi_update_fb(struct drm_device *dev, int pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	u8 *p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer;
	u32 DBI_CB_phys = dev_priv->DBI_CB_phys;
	u32 *pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer);
	u32 mipi_command_length_reg = MIPI_COMMAND_LENGTH_REG;
	u32 mipi_command_address_reg = MIPI_COMMAND_ADDRESS_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 pipeconf_reg = PIPEACONF;
	bool *update_done = &dev_priv->dsr_fb_update_done_0;
#if MDFLD_JLIU7_DPU
	struct psb_drm_dpu_rect damage_rect = { 0, 0, 864, 480 };
#endif				/* MDFLD_JLIU7_DPU */
#if 0				/* MDFLD_PO_JLIU7 */
	static u32 count_te;
	static u32 count_update;
#endif				/* MDFLD_PO_JLIU7 */

#if 0
	PSB_DEBUG_ENTRY("pipe = %d \n", pipe);
#endif

	switch (pipe) {
	case 0:
		break;
	case 2:
		p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer2;
		DBI_CB_phys = dev_priv->DBI_CB_phys2;
		pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer2);

		mipi_command_length_reg =
		    MIPI_COMMAND_LENGTH_REG + MIPIC_REG_OFFSET;
		mipi_command_address_reg =
		    MIPI_COMMAND_ADDRESS_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		pipeconf_reg = PIPECCONF;
		update_done = &dev_priv->dsr_fb_update_done_2;
		break;
	default:
		DRM_ERROR("mdfld_dbi_update_fb, Illegal Pipe Number. \n");
		return;
	}

	/* FIXME_JLIU7 MDFLD_PO */
	/* disable all the MIPI interrupts at the beginning. */
	/* enable all the MIPI interrupts at the end. */
	/* Make sure dbi command & data buffer are empty */
	if (*pDBI_CB_pointer != 0) {
		DRM_ERROR
		    ("mdfld_dbi_update_fb, dbi command buffer was interrupted before finished. pipe = %d\n",
		     pipe);
		return;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, true))
		return;

#if 0				/* MDFLD_PO_JLIU7 */
	count_te++;
#endif				/* MDFLD_PO_JLIU7 */
	if ((!(REG_READ(mipi_command_address_reg) & BIT0))
	    && (REG_READ(gen_fifo_stat_reg) & DBI_FIFO_EMPTY)
	    && (REG_READ(pipeconf_reg) & PIPEACONF_ENABLE)) {
		/* udelay(5000); */
		*pDBI_CB_pointer = 0;
#if MDFLD_JLIU7_DPU
		if (dev_priv->b_dpu_enable) {
			if (!(mdfld_dbi_damage_rect(dev, pipe, &damage_rect))) {
/*                              DRM_ERROR("mdfld_dbi_update_fb, damaged rect update pipe = %d \n", pipe); */
				/* set_column_address */
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    set_column_address;

				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    damage_rect.x >> 8;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    damage_rect.x;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    (damage_rect.x + damage_rect.width -
				     1) >> 8;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    (damage_rect.x + damage_rect.width - 1);

				*pDBI_CB_pointer = 8;
				/* set_page_addr */
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    set_page_addr;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    damage_rect.y >> 8;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    damage_rect.y;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    (damage_rect.y + damage_rect.height -
				     1) >> 8;
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    (damage_rect.y + damage_rect.height - 1);

				*pDBI_CB_pointer = 16;
				/* write_mem_start */
				*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
				    write_mem_start;

				REG_WRITE(mipi_command_length_reg, 0x010505);

				REG_WRITE(mipi_command_address_reg,
					  DBI_CB_phys | BIT0 | BIT1);

				udelay(3000);
				/* udelay(5000); */
				*pDBI_CB_pointer = 0;
				*update_done = true;
			}

		} else {
			/* write_mem_start */
			*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
			    write_mem_start;
			REG_WRITE(mipi_command_length_reg, 1);

			REG_WRITE(mipi_command_address_reg,
				  DBI_CB_phys | BIT0 | BIT1);

			udelay(3000);
			/* udelay(5000); */
			*pDBI_CB_pointer = 0;
			*update_done = true;
		}
#else				/* MDFLD_JLIU7_DPU */
		/* write_mem_start */
		*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = write_mem_start;

		REG_WRITE(mipi_command_length_reg, 1);
		REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0 | BIT1);

		udelay(3000);
		/* udelay(5000); */
		*pDBI_CB_pointer = 0;
		*update_done = true;
#if 0				/* MDFLD_PO_JLIU7 */
		count_update++;
#endif				/* MDFLD_PO_JLIU7 */
#endif				/* MDFLD_JLIU7_DPU */
	}
#if 0				/* MDFLD_PO_JLIU7 */
	if (((count_te % 200) == 0) || ((count_te % 201) == 0)) {
		DRM_INFO
		    ("count_te = 0x%x, count_update = 0x%x, pipe = 0x%x  \n",
		     count_te, count_update, pipe);
		DRM_INFO("dbi_panel_on = 0x%x, dbi_panel_on2 = 0x%x. \n",
			 dev_priv->dbi_panel_on, dev_priv->dbi_panel_on2);
	}
#endif				/* MDFLD_PO_JLIU7 */
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}
#endif				/* MDFLD_JLIU7_DPU_2 */
#endif				/*FIXME JLIU */

#if MDFLD_JLIU7_DSR
/**
 * Sets the power management mode of the pipe and plane.
 *
 * This code should probably grow support for turning the cursor off and back
 * on appropriately at the same time as we're turning the pipe off/on.
 */
void mdfld_dbi_dpms(struct drm_device *dev, int pipe, bool enabled)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	int dpll_reg = MRST_DPLL_A;
	int dspcntr_reg = DSPACNTR;
	int dspbase_reg = MRST_DSPABASE;
	int pipeconf_reg = PIPEACONF;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	u32 temp;
	int timeout = 0;
#if 0				/* MDFLD_PO_JLIU7 */
	static u32 count_te1;
	static u32 count_te2;
#endif				/* MDFLD_PO_JLIU7 */

#if 0
	PSB_DEBUG_ENTRY("pipe = %d \n", pipe);
#endif

	switch (pipe) {
	case 0:
		break;
	case 2:
		dpll_reg = MRST_DPLL_A;
		dspcntr_reg = DSPCCNTR;
		dspbase_reg = MDFLD_DSPCBASE;
		pipeconf_reg = PIPECCONF;
		pipeconf = dev_priv->pipeconf2;
		dspcntr = dev_priv->dspcntr2;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		break;
	case 1:
	case 3:
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return;
	}

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;

	if (enabled) {
#if 0				/* MDFLD_PO_JLIU7 */
		count_te1++;
		if ((count_te1 % 40) == 0) {
			DRM_ERROR("%s, enabled, b_dsr = 0x%x.\n",
				  __func__, dev_priv->b_dsr);
		}
#endif				/* MDFLD_PO_JLIU7 */
		/* Enable the DPLL */
		temp = REG_READ(dpll_reg);

		if ((temp & DPLL_VCO_ENABLE) == 0) {
			/* When ungating power of DPLL, needs to wait 0.5us before enable the VCO */
			if (temp & MDFLD_PWR_GATE_EN) {
				temp &= ~MDFLD_PWR_GATE_EN;
				REG_WRITE(dpll_reg, temp);
				/* FIXME_MDFLD PO - change 500 to 1 after PO */
				udelay(500);
			}

			REG_WRITE(dpll_reg, temp);
			REG_READ(dpll_reg);
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(500);

			REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);

			/* wait for DSI PLL to lock */
			while ((timeout < 20000)
			       && !(REG_READ(pipeconf_reg) &
				    PIPECONF_DSIPLL_LOCK)) {
				udelay(150);
				timeout++;
			}
		}

		/* Enable the pipe */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) == 0) {
			REG_WRITE(pipeconf_reg, pipeconf);

			/* Wait for for the pipe enable to take effect. */
			mdfldWaitForPipeEnable(dev, pipe);
		}

		/* Enable the plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
			REG_WRITE(dspcntr_reg, temp | DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		}
#if 0				/* MDFLD_JLIU7 DSR */
		/* Restore display pipe & dphy */
		/* Restore device states */

		/* Set DSI host into Utra Low Power State */
		temp = REG_READ(device_ready_reg);
		temp &= ~ULPS_MASK;
		temp |= ENTERING_ULPS;
		REG_WRITE(device_ready_reg, temp);

		temp = REG_READ(mipi_reg);
		REG_WRITE(mipi_reg, temp);

		/* Set DSI host to exit from Utra Low Power State */
		temp = REG_READ(device_ready_reg);
		temp &= ~ULPS_MASK;
		temp |= EXITING_ULPS;
		REG_WRITE(device_ready_reg, temp);

#endif				/* MDFLD_JLIU7 DSR */
	} else {
#if 0				/* MDFLD_PO_JLIU7 */
		count_te2++;
		if ((count_te2 % 40) == 0) {
			DRM_ERROR("%s, disabled, b_dsr = 0x%x.\n",
				  __func__, dev_priv->b_dsr);
		}
#endif				/* MDFLD_PO_JLIU7 */
		mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
					 HS_CTRL_FIFO_EMPTY |
					 HS_DATA_FIFO_EMPTY);

#if 0				/* MDFLD_JLIU7 DSR */
		/* Set DSI host into Utra Low Power State */
		temp = REG_READ(device_ready_reg);
		temp &= ~ULPS_MASK;
		temp |= ENTERING_ULPS;
		REG_WRITE(device_ready_reg, temp);

		temp = REG_READ(mipi_reg);
		temp &= ~PASS_FROM_SPHY_TO_AFE;
		REG_WRITE(mipi_reg, temp);

		/* Saving device states */
		/* Power gate display pipe & dphy */
#endif				/* MDFLD_JLIU7 DSR */
		/* Disable display plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
			REG_WRITE(dspcntr_reg, temp & ~DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			REG_READ(dspbase_reg);
		}

		/* FIXME_JLIU7 MDFLD_PO revisit */
		/* Wait for vblank for the disable to take effect */
/* MDFLD_PO_JLIU7               psb_intel_wait_for_vblank(dev); */

		/* Next, disable display pipes */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) != 0) {
			temp &= ~PIPEACONF_ENABLE;
			temp |= PIPECONF_PLANE_OFF | PIPECONF_CURSOR_OFF;
			REG_WRITE(pipeconf_reg, temp);
/*                      REG_WRITE(pipeconf_reg, 0); */
			REG_READ(pipeconf_reg);

			/* Wait for for the pipe disable to take effect. */
			mdfldWaitForPipeDisable(dev, pipe);
		}

		temp = REG_READ(dpll_reg);
		if (temp & DPLL_VCO_ENABLE) {
			if (!
			    ((REG_READ(PIPEACONF) | REG_READ(PIPECCONF)) &
			     PIPEACONF_ENABLE)) {
				temp &= ~(DPLL_VCO_ENABLE);
				REG_WRITE(dpll_reg, temp);
				REG_READ(dpll_reg);
				/* Wait for the clocks to turn off. */
				/* FIXME_MDFLD PO may need more delay */
				udelay(500);
			}
		}

	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/**
 * Enter DSR
 */
void mdfld_dbi_enter_dsr(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
#if 0				/* MDFLD_PO_JLIU7 */
	static u32 count_te1;
#endif				/* MDFLD_PO_JLIU7 */

	PSB_DEBUG_ENTRY(" \n");

#if 0				/* MDFLD_PO_JLIU7 */
	count_te1++;
#endif				/* MDFLD_PO_JLIU7 */

/*      mutex_lock(&dev_priv->dsr_mutex); */

	if (!dev_priv->b_dsr) {

		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, true))
			return;

		dev_priv->b_dsr = true;

		if (dev_priv->dbi_panel_on) {
			mdfld_dsi_dbi_CB_ready(dev, MIPI_COMMAND_ADDRESS_REG,
					       GEN_FIFO_STAT_REG);
			mdfld_dbi_dpms(dev, 0, false);
		}

		if (dev_priv->dbi_panel_on2) {
			mdfld_dsi_dbi_CB_ready(dev,
					       MIPI_COMMAND_ADDRESS_REG +
					       MIPIC_REG_OFFSET,
					       GEN_FIFO_STAT_REG +
					       MIPIC_REG_OFFSET);
			mdfld_dbi_dpms(dev, 2, false);
		}

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}
#if 0				/* MDFLD_PO_JLIU7 */
	if ((count_te1 % 200) == 0) {
		DRM_ERROR("%s, count_te1 = 0x%x, b_dsr = 0x%x.\n",
			  __func__, count_te1, dev_priv->b_dsr);
	}
#endif				/* MDFLD_PO_JLIU7 */

/*      mutex_unlock(&dev_priv->dsr_mutex); */
}

/**
 * Exit from DSR
 */
void mdfld_dbi_exit_dsr(struct drm_device *dev, u32 update_src)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct timer_list *dsr_timer = &dev_priv->dsr_timer;
	unsigned long irq_flags;
#if 0				/* MDFLD_PO_JLIU7 */
	static u32 count_te2;
#endif				/* MDFLD_PO_JLIU7 */

	PSB_DEBUG_ENTRY("update_src = 0x%x. \n", update_src);

#if 0				/* MDFLD_PO_JLIU7 */
	count_te2++;
#endif				/* MDFLD_PO_JLIU7 */

/*      mutex_lock(&dev_priv->dsr_mutex); */

	if (dev_priv->b_dsr) {
		if (dev_priv->dbi_panel_on)
			mdfld_dbi_dpms(dev, 0, true);

		if (dev_priv->dbi_panel_on2)
			mdfld_dbi_dpms(dev, 2, true);

		dev_priv->b_dsr = false;
	}
/*      mutex_unlock(&dev_priv->dsr_mutex); */

	dev_priv->dsr_fb_update |= update_src;
#if 0				/* MDFLD_PO_JLIU7 */
	if ((count_te2 % 200) == 0) {
		DRM_ERROR("%s, count_te2 = 0x%x, b_dsr = 0x%x.\n",
			  __func__, count_te2, dev_priv->b_dsr);
	}
#endif				/* MDFLD_PO_JLIU7 */

	spin_lock_irqsave(&dev_priv->dsr_lock, irq_flags);
	if (!timer_pending(dsr_timer)) {
		dsr_timer->expires = jiffies + MDFLD_DSR_DELAY;
		add_timer(dsr_timer);
	}
	spin_unlock_irqrestore(&dev_priv->dsr_lock, irq_flags);
}
#endif				/*FIXME JLIU */

static void mdfld_wait_for_HS_DATA_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(gen_fifo_stat_reg) &
				     HS_DATA_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: HS Data FIFO was never cleared!\n");
}

static void mdfld_wait_for_HS_CTRL_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(gen_fifo_stat_reg) &
				     HS_CTRL_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}
	if (timeout == 20000)
		DRM_INFO("MIPI: HS CMD FIFO was never cleared!\n");
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_init_TPO_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
void mdfld_init_TPO_MIPI(struct drm_device *dev, u32 pipe)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 dcsChannelNumber = dev_priv->channelNumber;
	u32 gen_data_reg = HS_GEN_DATA_REG;
	u32 gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 gen_ctrl_val = GEN_LONG_WRITE;

	DRM_INFO("Enter mrst init TPO MIPI display.\n");

	if (pipe == 2) {
		dcsChannelNumber = dev_priv->channelNumber2;
		gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;

	/* Flip page order */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00008036);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x02 << WORD_COUNTS_POS));

	/* 0xF0 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5af0);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* Write protection key */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5af1);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xFC */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5afc);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xB7 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x770000b7);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000044);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x05 << WORD_COUNTS_POS));

	/* 0xB6 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000a0ab6);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xF2 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x081010f2);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x4a070708);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000000c5);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xF8 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x024003f8);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x01030a04);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x0e020220);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000004);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x0d << WORD_COUNTS_POS));

	/* 0xE2 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x398fc3e2);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x0000916f);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x06 << WORD_COUNTS_POS));

	/* 0xB0 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000000b0);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x02 << WORD_COUNTS_POS));

	/* 0xF4 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x240242f4);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x78ee2002);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2a071050);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x507fee10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x10300710);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x14 << WORD_COUNTS_POS));

	/* 0xBA */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x19fe07ba);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x101c0a31);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000010);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xBB */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x28ff07bb);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x24280a31);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000034);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xFB */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535d05fb);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1b1a2130);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x221e180e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x131d2120);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535d0508);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1c1a2131);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x231f160d);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x111b2220);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535c2008);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1f1d2433);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2c251a10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2c34372d);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000023);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x31 << WORD_COUNTS_POS));

	/* 0xFA */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x525c0bfa);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1c1c232f);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2623190e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x18212625);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x545d0d0e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1e1d2333);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x26231a10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1a222725);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x545d280f);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x21202635);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x31292013);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x31393d33);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000029);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x31 << WORD_COUNTS_POS));

	/* Set DM */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000100f7);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));
}

/**
 * Sets the power state for the dpi panel.
 */
static void mdfld_dpi_set_power(struct drm_device *dev,
				struct psb_intel_output *output, bool on)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
#if 0				/*FIXME JLIU7 */
	u32 pp_status;
#endif				/*FIXME JLIU7 */
	bool *pdpi_panel_on = &(dev_priv->dpi_panel_on);
	u32 dpi_control_reg = DPI_CONTROL_REG;
	u32 pipeconf_reg = PIPEACONF;
	u32 pwm_clt_reg = BLC_PWM_CTL;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 intr_stat_reg = INTR_STAT_REG;
	int pipe = 0;
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	struct backlight_device bd;
#endif

	PSB_DEBUG_ENTRY("on = %d, output_type = 0x%x \n", on, output->type);

	if (output->type == INTEL_OUTPUT_MIPI2) {
		if (on) {
			dev_priv->dual_mipi = true;
		} else
			dev_priv->dual_mipi = false;

		pdpi_panel_on = &(dev_priv->dpi_panel_on2);
		dpi_control_reg = DPI_CONTROL_REG + MIPIC_REG_OFFSET;
		pipeconf_reg = PIPECCONF;
		pwm_clt_reg = BLC_PWM_CTL_C;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		intr_stat_reg = INTR_STAT_REG + MIPIC_REG_OFFSET;
		pipe = 2;
	} else if (output->type == INTEL_OUTPUT_MIPI) {
		if (!on) {
			dev_priv->dual_mipi = false;
		}
	}

	PSB_DEBUG_ENTRY("dpi_panel_on = %d. \n", *pdpi_panel_on);

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		if (!(*pdpi_panel_on)) {
			/* wait for DPI FIFO to clear */
			while ((REG_READ(gen_fifo_stat_reg) & DPI_FIFO_EMPTY)
			       != DPI_FIFO_EMPTY) {
				/* Do Nothing Here */
				/* This should make checkpatch work */
			}

			if ((REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
				REG_WRITE(intr_stat_reg, SPL_PKT_SENT);
			}

			REG_WRITE(dpi_control_reg, DPI_TURN_ON);
			/* make sure Turn On Packet Sent */
			while (!(REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
				/* Do Nothing Here */
			}

			if ((REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
				REG_WRITE(intr_stat_reg, SPL_PKT_SENT);
			}
#if DSI_TPO_864x480 || DSI_TPO_864x480_2
			mdfld_init_TPO_MIPI(dev, pipe);
#endif				/* DSI_TPO_864x480 */
			*pdpi_panel_on = true;
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
			bd.props.brightness = psb_get_brightness(&bd);
			psb_set_brightness(&bd);
#endif
		}
	} else {
		if ((*pdpi_panel_on) || dev_priv->first_boot) {
			mdfld_dsi_brightness_control(dev, pipe, 0);

			/* wait for DPI FIFO to clear */
			mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
						 DPI_FIFO_EMPTY);

			if ((REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
				REG_WRITE(intr_stat_reg, SPL_PKT_SENT);
			}

			REG_WRITE(dpi_control_reg, DPI_SHUT_DOWN);

			/* make sure Turn off Packet Sent */
			mdfld_dsi_gen_fifo_ready(dev, intr_stat_reg,
						 SPL_PKT_SENT);

			*pdpi_panel_on = false;
			dev_priv->first_boot = false;
		}
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/**
 * Sets the power state for the dbi panel.
 */
static void mdfld_dbi_set_power(struct drm_device *dev,
				struct psb_intel_output *output, bool on)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
#if 0				/*FIXME JLIU7 */
	u32 pp_status;
#endif				/*FIXME JLIU7 */
	bool *pdbi_panel_on = &(dev_priv->dbi_panel_on);
	u8 *p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer;
	u32 *pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer);
	u32 DBI_CB_phys = dev_priv->DBI_CB_phys;
	u32 mipi_command_length_reg = MIPI_COMMAND_LENGTH_REG;
	u32 mipi_command_address_reg = MIPI_COMMAND_ADDRESS_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	u32 pipeconf_reg = PIPEACONF;
	u32 disp_cntr_reg = DSPACNTR;
	int pipe = 0;
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	struct backlight_device bd;
#endif

	PSB_DEBUG_ENTRY("on = %d, output_type = 0x%x \n", on, output->type);

	if (output->type == INTEL_OUTPUT_MIPI2) {
		if (on) {
			dev_priv->dual_mipi = true;
		} else
			dev_priv->dual_mipi = false;

		pdbi_panel_on = &(dev_priv->dbi_panel_on2);
		p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer2;
		DBI_CB_phys = dev_priv->DBI_CB_phys2;
		pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer2);
		mipi_command_length_reg =
		    MIPI_COMMAND_LENGTH_REG + MIPIC_REG_OFFSET;
		mipi_command_address_reg =
		    MIPI_COMMAND_ADDRESS_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		pipeconf = dev_priv->pipeconf2;
		dspcntr = dev_priv->dspcntr2;
		pipeconf_reg = PIPECCONF;
		disp_cntr_reg = DSPCCNTR;
		pipe = 2;
	} else if (output->type == INTEL_OUTPUT_MIPI) {
		if (!on) {
			dev_priv->dual_mipi = false;
		}
	}

	PSB_DEBUG_ENTRY("dbi_panel_on = %d \n", *pdbi_panel_on);

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		if (!(*pdbi_panel_on)) {
			/* FIXME_JLIU7 MDFLD_PO */
			/* disable all the MIPI interrupts at the beginning. */
			/* enable all the MIPI interrupts at the end. */
			/* Make sure dbi command & data buffer are empty */
			if (*pDBI_CB_pointer != 0) {
				DRM_ERROR
				    ("dbi command buffer was interrupted before finished. \n");
			}

			mdfld_dsi_dbi_CB_ready(dev, mipi_command_address_reg,
					       gen_fifo_stat_reg);

			/* Wait for 20ms. */
			udelay(20000);

#if 1				/* MDFLD_PO_JLIU7 */
			/* exit sleep mode */
			*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
			    exit_sleep_mode;

			REG_WRITE(mipi_command_length_reg, 1);
			udelay(5000);
			REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0);

			/* The host processor must wait five milliseconds after sending exit_sleep_mode command before sending another
			   command. This delay allows the supply voltages and clock circuits to stabilize */
			udelay(5000);

			mdfld_dsi_dbi_CB_ready(dev, mipi_command_address_reg,
					       gen_fifo_stat_reg);

			*pDBI_CB_pointer = 0;

			/* Wait for 20ms. */
			udelay(20000);
#endif				/* MDFLD_PO_JLIU7 */

			/*      set display on */
			*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
			    set_display_on;

			/* FIXME_jliu7 mapVitualToPhysical(dev_priv->p_DBI_commandBuffer); */
			REG_WRITE(mipi_command_length_reg, 1);
			udelay(5000);
			REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0);

			*pDBI_CB_pointer = 0;

			*pdbi_panel_on = true;
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
			bd.props.brightness = psb_get_brightness(&bd);
			psb_set_brightness(&bd);
#endif
		}
/*FIXME JLIU7 */
/* Need to figure out how to control the MIPI panel power on sequence*/
	} else {
/*FIXME JLIU7 */
/* Need to figure out how to control the MIPI panel power down sequence*/
		/*
		 * Only save the current backlight value if we're going from
		 * on to off.
		 */
		if ((*pdbi_panel_on) || dev_priv->first_boot) {
#if 1				/* MDFLD_PO_JLIU7 */
			/* FIXME_JLIU7 MDFLD_PO */
			/* disable all the MIPI interrupts at the beginning. */
			/* enable all the MIPI interrupts at the end. */
			/* Make sure dbi command & data buffer are empty */
			if (*pDBI_CB_pointer != 0) {
				DRM_ERROR
				    ("dbi command buffer was interrupted before finished. \n");
			}

			*pdbi_panel_on = false;
			mdfld_dsi_brightness_control(dev, pipe, 0);

			mdfld_dsi_dbi_CB_ready(dev, mipi_command_address_reg,
					       gen_fifo_stat_reg);

			/* Wait for 20ms. */
			udelay(20000);

			*pDBI_CB_pointer = 0;
			/* enter sleep mode */
			*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) =
			    enter_sleep_mode;

			REG_WRITE(mipi_command_length_reg, 1);
			udelay(5000);
			REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0);

			*pDBI_CB_pointer = 0;
#endif				/* MDFLD_PO_JLIU7 */
			dev_priv->first_boot = false;
		}
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/**
 * Sets the power state for the mipi panel.
 */
static void mdfld_dsi_set_power(struct drm_device *dev,
				struct psb_intel_output *output, bool on)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	bool dpi = dev_priv->dpi;

	if (output->type == INTEL_OUTPUT_MIPI2)
		dpi = dev_priv->dpi2;

	if (dpi)
		mdfld_dpi_set_power(dev, output, on);
	else
		mdfld_dbi_set_power(dev, output, on);
}

static void mdfld_dsi_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("%s \n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (mode == DRM_MODE_DPMS_ON)
		mdfld_dsi_set_power(dev, output, true);
	else
		mdfld_dsi_set_power(dev, output, false);

	/* XXX: We never power down the DSI pairs. */
}

static enum drm_connector_status mdfld_dsi_detect(struct drm_connector
						  *connector)
{
	PSB_DEBUG_ENTRY("\n");

	return connector_status_connected;
}

static int mdfld_dsi_set_property(struct drm_connector *connector,
				  struct drm_property *property, uint64_t value)
{

	struct drm_encoder *pEncoder = connector->encoder;

	PSB_DEBUG_ENTRY("\n");

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		struct psb_intel_crtc *pPsbCrtc =
		    to_psb_intel_crtc(pEncoder->crtc);
		bool bTransitionFromToCentered;
		uint64_t curValue;

		if (!pPsbCrtc)
			goto set_prop_error;

		switch (value) {
		case DRM_MODE_SCALE_FULLSCREEN:
			break;
		case DRM_MODE_SCALE_NO_SCALE:
			break;
		case DRM_MODE_SCALE_ASPECT:
			break;
		default:
			goto set_prop_error;
		}

		if (drm_connector_property_get_value
		    (connector, property, &curValue))
			goto set_prop_error;

		if (curValue == value)
			goto set_prop_done;

		if (drm_connector_property_set_value
		    (connector, property, value))
			goto set_prop_error;

		bTransitionFromToCentered =
		    (curValue == DRM_MODE_SCALE_NO_SCALE)
		    || (value == DRM_MODE_SCALE_NO_SCALE);

		if (pPsbCrtc->saved_mode.hdisplay != 0 &&
		    pPsbCrtc->saved_mode.vdisplay != 0) {
			if (bTransitionFromToCentered) {
				if (!drm_crtc_helper_set_mode
				    (pEncoder->crtc, &pPsbCrtc->saved_mode,
				     pEncoder->crtc->x, pEncoder->crtc->y,
				     pEncoder->crtc->fb))
					goto set_prop_error;
			} else {
				struct drm_encoder_helper_funcs *pEncHFuncs =
				    pEncoder->helper_private;
				pEncHFuncs->mode_set(pEncoder,
						     &pPsbCrtc->saved_mode,
						     &pPsbCrtc->saved_adjusted_mode);
			}
		}
	} else if (!strcmp(property->name, "backlight") && pEncoder) {
		PSB_DEBUG_ENTRY("backlight \n");
		if (drm_connector_property_set_value
		    (connector, property, value))
			goto set_prop_error;
		else {
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
			struct backlight_device bd;
			bd.props.brightness = value;
			psb_set_brightness(&bd);
#endif
		}
	} else if (!strcmp(property->name, "DPMS") && pEncoder) {
		struct drm_encoder_helper_funcs *pEncHFuncs =
		    pEncoder->helper_private;
		/*struct drm_crtc_helper_funcs *pCrtcHFuncs = pEncoder->crtc->helper_private; */
		PSB_DEBUG_ENTRY("DPMS \n");
		pEncHFuncs->dpms(pEncoder, value);
		/*pCrtcHFuncs->dpms(pEncoder->crtc, value); */
	}

 set_prop_done:
	return 0;
 set_prop_error:
	return -1;
}

void mdfld_dsi_prepare(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
#if 0				/* FIXME JLIU7 */
	struct psb_intel_mode_device *mode_dev = output->mode_dev;
#endif				/* FIXME JLIU7 */

	PSB_DEBUG_ENTRY("\n");

#if 0				/* FIXME JLIU7 */
	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;
	mode_dev->saveBLC_PWM_CTL = REG_READ(BLC_PWM_CTL);
	mode_dev->backlight_duty_cycle = (mode_dev->saveBLC_PWM_CTL &
					  BACKLIGHT_DUTY_CYCLE_MASK);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
#endif				/* FIXME JLIU7 */

	mdfld_dsi_set_power(dev, output, false);
}

/* ************************************************************************* *\
FUNCTION: mdfld_GetHSA_Count

DESCRIPTION: Shows the horizontal sync value in terms of byte clock
			(txbyteclkhs)
	Minimum HSA period should be sufficient to transmit a hsync start short
		packet(4 bytes)
		i) For Non-burst Mode with sync pulse, Min value � 4 in decimal [plus
			an optional 6 bytes for a zero payload blanking packet]. But if
			the value is less than 10 but more than 4, then this count will
			be added to the HBP�s count for one lane.
		ii) For Non-Burst Sync Event & Burst Mode, there is no HSA, so you
			can program this to zero. If you program this register, these
			byte values will be added to HBP.
		iii) For Burst mode of operation, normally the values programmed in
			terms of byte clock are based on the principle - time for transfering
			HSA in Burst mode is the same as in non-bust mode.
\* ************************************************************************* */
static u32 mdfld_GetHSA_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 HSA_count;
	u32 HSA_countX8;
	u32 bpp = dev_priv->bpp;
	u32 HsyncWidth = dev_priv->HsyncWidth;
	u32 videoModeFormat = dev_priv->videoModeFormat;
	u32 DDR_Clock = dev_priv->DDR_Clock;
	u32 DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated;
	u32 laneCount = dev_priv->laneCount;

	if (dsi_num == 2) {
		bpp = dev_priv->bpp2;
		HsyncWidth = dev_priv->HsyncWidth2;
		videoModeFormat = dev_priv->videoModeFormat2;
		DDR_Clock = dev_priv->DDR_Clock2;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated2;
		laneCount = dev_priv->laneCount2;
	}

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HSA_countX8 = HsyncWidth * bpp;

#if MDFLD_GET_SYNC_BURST
	if (videoModeFormat == BURST_MODE) {
		HSA_countX8 *= DDR_Clock / DDR_Clock_Calculated;
	}
#endif				/* MDFLD_GET_SYNC_BURST */

	HSA_count = HSA_countX8 / 8;

	/* FIXME_JLIU7 the above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HSA according to equation:
	   (hsync_width) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* FIXME_JLIU the lower equation = the upper equation / (2 * lane number) */

	HSA_count /= (2 * laneCount);

	if (HSA_count < 4)	/* minimum value of 4 */
		HSA_count = 4;

	PSB_DEBUG_HV("mdfld_HSA_count is %d, for dsi_num %d. \n", HSA_count,
		     dsi_num);

	return HSA_count;
}

/* ************************************************************************* *\
FUNCTION: mdfld_GetHBP_Count

DESCRIPTION: Shows the horizontal back porch value in terms of txbyteclkhs.
	Minimum HBP period should be sufficient to transmit a �hsync end short
		packet(4 bytes) + Blanking packet overhead(6 bytes) + RGB packet header(4 bytes)�
	For Burst mode of operation, normally the values programmed in terms of
		byte clock are based on the principle - time for transfering HBP
		in Burst mode is the same as in non-bust mode.

	Min value � 14 in decimal [ accounted with zero payload for blanking packet] for one lane.
	Max value � any value greater than 14 based on DPI resolution
\* ************************************************************************* */
static u32 mdfld_GetHBP_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 HBP_count, HBP_countX8;
	u32 bpp = dev_priv->bpp;
	u32 HbackPorch = dev_priv->HbackPorch;
	u32 videoModeFormat = dev_priv->videoModeFormat;
	u32 DDR_Clock = dev_priv->DDR_Clock;
	u32 DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated;
	u32 laneCount = dev_priv->laneCount;

	if (dsi_num == 2) {
		bpp = dev_priv->bpp2;
		HbackPorch = dev_priv->HbackPorch2;
		videoModeFormat = dev_priv->videoModeFormat2;
		DDR_Clock = dev_priv->DDR_Clock2;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated2;
		laneCount = dev_priv->laneCount2;
	}

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HBP_countX8 = HbackPorch * bpp;

#if MDFLD_GET_SYNC_BURST
	if (videoModeFormat == BURST_MODE) {
		HBP_countX8 *= DDR_Clock / DDR_Clock_Calculated;
	}
#endif				/* MDFLD_GET_SYNC_BURST */

	HBP_count = HBP_countX8 / 8;

	/* FIXME_JLIU7 the above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HBP according to equation:
	   (hsync_backporch) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* FIXME_JLIU the lower equation = the upper equation / (2 * lane number) */

	HBP_count /= (2 * laneCount);

	if (HBP_count < 8)	/* minimum value of 8 */
		HBP_count = 8;

	PSB_DEBUG_HV("mdfld_HBP_count is %d, for dsi_num %d. \n", HBP_count,
		     dsi_num);

/* MDFLD_PO_JLIU7 */
	return 0x0e;
	return HBP_count;
}

/* ************************************************************************* *\
FUNCTION: mdfld_GetHFP_Count

DESCRIPTION: Shows the horizontal front porch value in terms of txbyteclkhs.
	Minimum HFP period should be sufficient to transmit �RGB Data packet
	footer(2 bytes) + Blanking packet overhead(6 bytes)� for non burst mode.

	For burst mode, Minimum HFP period should be sufficient to transmit
	Blanking packet overhead(6 bytes)�

	For Burst mode of operation, normally the values programmed in terms of
		byte clock are based on the principle - time for transfering HFP
		in Burst mode is the same as in non-bust mode.

	Min value � 8 in decimal  for non-burst mode [accounted with zero payload
		for blanking packet] for one lane.
	Min value � 6 in decimal for burst mode for one lane.

	Max value � any value greater than the minimum vaue based on DPI resolution
\* ************************************************************************* */
static u32 mdfld_GetHFP_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 HFP_count, HFP_countX8;
	u32 bpp = dev_priv->bpp;
	u32 HfrontPorch = dev_priv->HfrontPorch;
	u32 videoModeFormat = dev_priv->videoModeFormat;
	u32 DDR_Clock = dev_priv->DDR_Clock;
	u32 DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated;
	u32 laneCount = dev_priv->laneCount;

	if (dsi_num == 2) {
		bpp = dev_priv->bpp2;
		HfrontPorch = dev_priv->HfrontPorch2;
		videoModeFormat = dev_priv->videoModeFormat2;
		DDR_Clock = dev_priv->DDR_Clock2;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated2;
		laneCount = dev_priv->laneCount2;
	}

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HFP_countX8 = HfrontPorch * bpp;

#if MDFLD_GET_SYNC_BURST
	if (videoModeFormat == BURST_MODE) {
		HFP_countX8 *= DDR_Clock / DDR_Clock_Calculated;
	}
#endif				/* MDFLD_GET_SYNC_BURST */

	HFP_count = HFP_countX8 / 8;

	/* FIXME_JLIU7 the above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HFP according to equation:
	   (hsync_frontporch) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* FIXME_JLIU the lower equation = the upper equation / (2 * lane number) */

	HFP_count /= (2 * laneCount);

	if (HFP_count < 8)	/* minimum value of 8 */
		HFP_count = 8;

	PSB_DEBUG_HV("mdfld_HFP_count is %d, for dsi_num %d. \n", HFP_count,
		     dsi_num);

	return HFP_count;
}

/* ************************************************************************* *\
FUNCTION: mdfld_GetHAdr_Count

DESCRIPTION: Shows the horizontal active area value in terms of txbyteclkhs.
	In Non Burst Mode, Count equal to RGB word count value

	In Burst Mode, RGB pixel packets are time-compressed, leaving more time
		during a scan line for LP mode (saving power) or for multiplexing
		other transmissions onto the DSI link. Hence, the count equals the
		time in txbyteclkhs for sending  time compressed RGB pixels plus
		the time needed for moving to power save mode or the time needed
		for secondary channel to use the DSI link.

	But if the left out time for moving to low power mode is less than
		8 txbyteclkhs [2txbyteclkhs for RGB data packet footer and
		6txbyteclkhs for a blanking packet with zero payload],  then
		this count will be added to the HFP's count for one lane.

	Min value � 8 in decimal  for non-burst mode [accounted with zero payload
		for blanking packet] for one lane.
	Min value � 6 in decimal for burst mode for one lane.

	Max value � any value greater than the minimum vaue based on DPI resolution
\* ************************************************************************* */
static u32 mdfld_GetHAdr_Count(struct drm_device *dev,
			       DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 HAdr_count, HAdr_countX8;
	u32 bpp = dev_priv->bpp;
	u32 HactiveArea = dev_priv->HactiveArea;
	u32 videoModeFormat = dev_priv->videoModeFormat;
	u32 DDR_Clock = dev_priv->DDR_Clock;
	u32 DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated;
	u32 laneCount = dev_priv->laneCount;

	if (dsi_num == 2) {
		bpp = dev_priv->bpp2;
		HactiveArea = dev_priv->HactiveArea2;
		videoModeFormat = dev_priv->videoModeFormat2;
		DDR_Clock = dev_priv->DDR_Clock2;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated2;
		laneCount = dev_priv->laneCount2;
	}

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HAdr_countX8 = HactiveArea * bpp;

#if MDFLD_GET_SYNC_BURST
	if (videoModeFormat == BURST_MODE) {
		HAdr_countX8 *= DDR_Clock / DDR_Clock_Calculated;
	}
#endif				/* MDFLD_GET_SYNC_BURST */

	HAdr_count = HAdr_countX8 / 8;

	/* FIXME_JLIU7 the above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HAdr according to equation:
	   (horizontal active) * 24 bpp / (8 bits per lane * 2 lanes) */
	/* FIXME_JLIU the lower equation = the upper equation / (lane number) */

	HAdr_count /= laneCount;

	PSB_DEBUG_HV("mdfld_HAdr_count is %d, for dsi_num %d. \n", HAdr_count,
		     dsi_num);

	return HAdr_count;
}

/* ************************************************************************* *\
FUNCTION: mdfld_GetVSA_Count

DESCRIPTION: Shows the vertical sync value in terms of lines

\* ************************************************************************* */
static u32 mdfld_GetVSA_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 VSA_count;
	u32 VsyncWidth = dev_priv->VsyncWidth;

	if (dsi_num == 2) {
		VsyncWidth = dev_priv->VsyncWidth2;
	}

	/* Get the vsync pulse width */
	VSA_count = VsyncWidth;

	if (VSA_count < 2)	/* minimum value of 2 */
		VSA_count = 2;

	PSB_DEBUG_HV("mdfld_VSA_count is %d, for dsi_num %d. \n", VSA_count,
		     dsi_num);

	return VSA_count;
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_GetVBP_Count
 *
 * DESCRIPTION: Shows the vertical back porch value in lines.
 *
\* ************************************************************************* */
static u32 mdfld_GetVBP_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 VBP_count;
	u32 VbackPorch = dev_priv->VbackPorch;

	if (dsi_num == 2) {
		VbackPorch = dev_priv->VbackPorch2;
	}

	/* Get the Vertical Backporch width */
	VBP_count = VbackPorch;

	if (VBP_count < 2)	/* minimum value of 2 */
		VBP_count = 2;

	PSB_DEBUG_HV("mdfld_VBP_count is %d, for dsi_num %d. \n", VBP_count,
		     dsi_num);

	return VBP_count;
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_GetVFP_Count
 *
 * DESCRIPTION: Shows the vertical front porch value in terms of lines.
 *
\* ************************************************************************* */
static u32 mdfld_GetVFP_Count(struct drm_device *dev,
			      DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 VFP_count;
	u32 VfrontPorch = dev_priv->VfrontPorch;

	if (dsi_num == 2) {
		VfrontPorch = dev_priv->VfrontPorch2;
	}

	/* Get the Vertical Frontporch width */
	VFP_count = VfrontPorch;

	if (VFP_count < 2)	/* minimum value of 2 */
		VFP_count = 2;

	PSB_DEBUG_HV("mdfld_VFP_count is %d, for dsi_num %d. \n", VFP_count,
		     dsi_num);

	return VFP_count;
}

static void mdfld_dpi_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
#if 0				/* FIXME_JLIU7 add it later */
	uint64_t curValue = DRM_MODE_SCALE_FULLSCREEN;
#endif				/* FIXME_JLIU7 add it later */
	u32 SupportedFormat = 0;
	u32 resolution = 0;

	u32 bpp = dev_priv->bpp;
	u32 HactiveArea = dev_priv->HactiveArea;
	u32 VactiveArea = dev_priv->VactiveArea;
	u32 videoModeFormat = dev_priv->videoModeFormat;
	u32 channelNumber = dev_priv->channelNumber;
	u32 laneCount = dev_priv->laneCount;
	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	bool *pdpi_panel_on = &(dev_priv->dpi_panel_on);
	bool *pdbi_panel_on = &(dev_priv->dbi_panel_on);

	u32 mipi_reg = MIPI;
	u32 mipi_control_reg = MIPI_CONTROL_REG;
	u32 intr_en_reg = INTR_EN_REG;
	u32 turn_around_timeout_reg = TURN_AROUND_TIMEOUT_REG;
	u32 device_reset_reg = DEVICE_RESET_REG;
	u32 init_count_reg = INIT_COUNT_REG;
	u32 dsi_func_prg_reg = DSI_FUNC_PRG_REG;
	u32 dpi_resolution_reg = DPI_RESOLUTION_REG;
	u32 vert_sync_pad_count_reg = VERT_SYNC_PAD_COUNT_REG;
	u32 vert_back_porch_count_reg = VERT_BACK_PORCH_COUNT_REG;
	u32 vert_front_porch_count_reg = VERT_FRONT_PORCH_COUNT_REG;
	u32 horiz_sync_pad_count_reg = HORIZ_SYNC_PAD_COUNT_REG;
	u32 horiz_back_porch_count_reg = HORIZ_BACK_PORCH_COUNT_REG;
	u32 horiz_front_porch_count_reg = HORIZ_FRONT_PORCH_COUNT_REG;
	u32 horiz_active_area_count_reg = HORIZ_ACTIVE_AREA_COUNT_REG;
	u32 video_fmt_reg = VIDEO_FMT_REG;
	u32 hs_tx_timeout_reg = HS_TX_TIMEOUT_REG;
	u32 lp_rx_timeout_reg = LP_RX_TIMEOUT_REG;
	u32 high_low_switch_count_reg = HIGH_LOW_SWITCH_COUNT_REG;
	u32 eot_disable_reg = EOT_DISABLE_REG;
	u32 lp_byteclk_reg = LP_BYTECLK_REG;
	u32 device_ready_reg = DEVICE_READY_REG;
	u32 dpi_control_reg = DPI_CONTROL_REG;
	u32 dphy_param_reg = DPHY_PARAM_REG;
	u32 clk_lane_swt_reg = CLK_LANE_SWT_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 intr_stat_reg = INTR_STAT_REG;
	u32 pipeconf_reg = PIPEACONF;
	u32 disp_cntr_reg = DSPACNTR;

	/* defaut to be dpi on values on pipe A. */
	/* Enable MIPI Port */
	u32 mipi_val = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE | SEL_FLOPPED_HSTX;
	u32 mipi_control_val = 0x00000018;
	u32 dphy_param_val = 0x150c3408;	/* dual dbi - dpi *//* single = 0x0B14540C; */
	u32 clk_lane_swt_val = 0x000A0014;
	u32 intr_en_val = 0xffffffff;
	u32 turn_around_timeout_val = 0x0000001F;
	u32 device_reset_val = 0x000000ff;	/* old value = 0x00000015 may depends on the DSI RX device */
	u32 init_count_val = 0x000007d0;	/* 0x00000050; Minimum value = 0x000007d0 */
	u32 dsi_func_prg_val = 0;
	u32 horiz_sync_pad_count_val = mdfld_GetHSA_Count(dev, dev_priv, 1);
	u32 horiz_back_porch_count_val = mdfld_GetHBP_Count(dev, dev_priv, 1);
	u32 horiz_front_porch_count_val = mdfld_GetHFP_Count(dev, dev_priv, 1);
	u32 horiz_active_area_count_val = mdfld_GetHAdr_Count(dev, dev_priv, 1);
	u32 VSA_count = mdfld_GetVSA_Count(dev, dev_priv, 1);
	u32 VBP_count = mdfld_GetVBP_Count(dev, dev_priv, 1);
	u32 VFP_count = mdfld_GetVFP_Count(dev, dev_priv, 1);

#if 1				/* MDFLD_PO_JLIU7 QCIF_176x144_SLE || DSI_TPO_800x600 || DBI_TPO_480x864 || DSI_TPO_800x480 ||DSI_TPO_864x480 || DBI_TPO_864x480 */
	u32 hs_tx_timeout_val = 0x3fffff;
	u32 lp_rx_timeout_val = 0xffff;
	u32 high_low_switch_count_val = 0x46;
#else				/*JLIU7_PO hard coded for NSC PO */
	u32 hs_tx_timeout_val = GetHS_TX_timeoutCount(dev_priv);
	u32 lp_rx_timeout_val = GetLP_RX_timeoutCount(dev_priv);
	u32 high_low_switch_count_val = GetHighLowSwitchCount(dev_priv);
#endif				/*JLIU7_PO hard coded for NSC PO */

/* MDFLD_PO_JLIU7	u32 eot_disable_val = ENABLE_CLOCK_STOPPING; */
	u32 eot_disable_val = 0;
	u32 lp_byteclk_val = 0x00000004;	/* FIXME JLIU7 for NSC PO */
	u32 device_ready_val = 0x00000001;
	u32 dpi_control_val = 0x00000002;	/* Turn On */
	u32 timeout = 0;
	u32 pipe = 0;

	PSB_DEBUG_ENTRY("output_type = 0x%x \n", output->type);

	if (output->type == INTEL_OUTPUT_MIPI2) {
		/* MIPI_A has to be on before we can enable MIPI_C */
		if (!(dev_priv->dpi_panel_on || dev_priv->dbi_panel_on)) {
			DRM_ERROR
			    ("mdfld_dpi_mode_set need to enable MIPI0 before enabling MIPI1. \n");
			return;
		}

		bpp = dev_priv->bpp2;
	}

	switch (bpp) {
	case 16:
		SupportedFormat = RGB_565_FMT;
		break;
	case 18:
		SupportedFormat = RGB_666_FMT;
		break;
	case 24:
		SupportedFormat = RGB_888_FMT;
		break;
	default:
		DRM_INFO("mdfld_dpi_mode_set,  invalid bpp \n");
		break;
	}

	SupportedFormat <<= FMT_DPI_POS;
	channelNumber <<= DPI_CHANNEL_NUMBER_POS;
	dsi_func_prg_val = laneCount | SupportedFormat | channelNumber;

	if (output->type == INTEL_OUTPUT_MIPI2) {
		pipe = 2;
		HactiveArea = dev_priv->HactiveArea2;
		VactiveArea = dev_priv->VactiveArea2;
		videoModeFormat = dev_priv->videoModeFormat2;
		channelNumber = dev_priv->channelNumber2;
		laneCount = dev_priv->laneCount2;
		pipeconf = dev_priv->pipeconf2;
		dspcntr = dev_priv->dspcntr2;
		pdpi_panel_on = &(dev_priv->dpi_panel_on2);
		pdbi_panel_on = &(dev_priv->dbi_panel_on2);

		mipi_reg = MIPI_C;
		mipi_control_reg = MIPI_CONTROL_REG + MIPIC_REG_OFFSET;
		intr_en_reg = INTR_EN_REG + MIPIC_REG_OFFSET;
		device_reset_reg = DEVICE_RESET_REG + MIPIC_REG_OFFSET;
		turn_around_timeout_reg =
		    TURN_AROUND_TIMEOUT_REG + MIPIC_REG_OFFSET;
		init_count_reg = INIT_COUNT_REG + MIPIC_REG_OFFSET;
		dsi_func_prg_reg = DSI_FUNC_PRG_REG + MIPIC_REG_OFFSET;
		dpi_resolution_reg = DPI_RESOLUTION_REG + MIPIC_REG_OFFSET;
		vert_sync_pad_count_reg =
		    VERT_SYNC_PAD_COUNT_REG + MIPIC_REG_OFFSET;
		vert_back_porch_count_reg =
		    VERT_BACK_PORCH_COUNT_REG + MIPIC_REG_OFFSET;
		vert_front_porch_count_reg =
		    VERT_FRONT_PORCH_COUNT_REG + MIPIC_REG_OFFSET;
		horiz_sync_pad_count_reg =
		    HORIZ_SYNC_PAD_COUNT_REG + MIPIC_REG_OFFSET;
		horiz_back_porch_count_reg =
		    HORIZ_BACK_PORCH_COUNT_REG + MIPIC_REG_OFFSET;
		horiz_front_porch_count_reg =
		    HORIZ_FRONT_PORCH_COUNT_REG + MIPIC_REG_OFFSET;
		horiz_active_area_count_reg =
		    HORIZ_ACTIVE_AREA_COUNT_REG + MIPIC_REG_OFFSET;
		video_fmt_reg = VIDEO_FMT_REG + MIPIC_REG_OFFSET;
		hs_tx_timeout_reg = HS_TX_TIMEOUT_REG + MIPIC_REG_OFFSET;
		lp_rx_timeout_reg = LP_RX_TIMEOUT_REG + MIPIC_REG_OFFSET;
		high_low_switch_count_reg =
		    HIGH_LOW_SWITCH_COUNT_REG + MIPIC_REG_OFFSET;
		eot_disable_reg = EOT_DISABLE_REG + MIPIC_REG_OFFSET;
		lp_byteclk_reg = LP_BYTECLK_REG + MIPIC_REG_OFFSET;
		device_ready_reg = DEVICE_READY_REG + MIPIC_REG_OFFSET;
		dpi_control_reg = DPI_CONTROL_REG + MIPIC_REG_OFFSET;
		dphy_param_reg = DPHY_PARAM_REG + MIPIC_REG_OFFSET;
		clk_lane_swt_reg = CLK_LANE_SWT_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		intr_stat_reg = INTR_STAT_REG + MIPIC_REG_OFFSET;

		pipeconf_reg = PIPECCONF;
		disp_cntr_reg = DSPCCNTR;

		horiz_sync_pad_count_val = mdfld_GetHSA_Count(dev, dev_priv, 2);
		horiz_back_porch_count_val =
		    mdfld_GetHBP_Count(dev, dev_priv, 2);
		horiz_front_porch_count_val =
		    mdfld_GetHFP_Count(dev, dev_priv, 2);
		horiz_active_area_count_val =
		    mdfld_GetHAdr_Count(dev, dev_priv, 2);
		VSA_count = mdfld_GetVSA_Count(dev, dev_priv, 2);
		VBP_count = mdfld_GetVBP_Count(dev, dev_priv, 2);
		VFP_count = mdfld_GetVFP_Count(dev, dev_priv, 2);
	} else {
		mipi_val |= dev_priv->mipi_lane_config;
	}

	resolution = HactiveArea | (VactiveArea << RES_V_POS);

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;

#if 0				/* FIXME_JLIU7 add it later */
	drm_connector_property_get_value(&enc_to_psb_intel_output
					 (encoder)->base,
					 dev->mode_config.scaling_mode_property,
					 &curValue);

	if (curValue == DRM_MODE_SCALE_NO_SCALE)
		REG_WRITE(PFIT_CONTROL, 0);
	else if (curValue == DRM_MODE_SCALE_ASPECT) {
		if ((mode->vdisplay != adjusted_mode->crtc_vdisplay)
		    || (mode->hdisplay != adjusted_mode->crtc_hdisplay)) {
			if ((adjusted_mode->crtc_hdisplay * mode->vdisplay) ==
			    (mode->hdisplay * adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
			else if ((adjusted_mode->crtc_hdisplay *
				  mode->vdisplay) >
				 (mode->hdisplay *
				  adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL,
					  PFIT_ENABLE |
					  PFIT_SCALING_MODE_PILLARBOX);
			else
				REG_WRITE(PFIT_CONTROL,
					  PFIT_ENABLE |
					  PFIT_SCALING_MODE_LETTERBOX);
		} else
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
	} else			/*(curValue == DRM_MODE_SCALE_FULLSCREEN) */
		REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
#endif				/* FIXME_JLIU7 add it later */

	/* udelay(20000); */
	/* wait for PIPE A to disable */
#if 0				/* FIXME_JLIU7 */
	while (REG_READ(pipeconf_reg) & PIPECONF_ACTIVE) {
		/* Do Nothing Here */
		/* This should make checkpatch work */
	}
	/* wait for DPI FIFO to clear */
	while ((timeout < 20000)
	       && ((REG_READ(gen_fifo_stat_reg) & DPI_FIFO_EMPTY)
		   != DPI_FIFO_EMPTY)) {
		udelay(100);
	}
#endif

	/* clear intr status register */

	/* Clear Device Ready Bit */
	REG_WRITE(device_ready_reg, 0x00000000);

	REG_WRITE(mipi_reg, mipi_val);

	/* FIXME_MDFLD JLIU7 revisit MIPI_CONTROL_REG */
	/* JLIU7_FIXME set MIPI clock ratio to 1:1 for NSC init */
	REG_WRITE(mipi_control_reg, mipi_control_val);

	/* Get the value from HW SV */
	REG_WRITE(dphy_param_reg, dphy_param_val);
	REG_WRITE(clk_lane_swt_reg, clk_lane_swt_val);

	/* Enable all the error interrupt */
	REG_WRITE(intr_en_reg, intr_en_val);
	REG_WRITE(turn_around_timeout_reg, turn_around_timeout_val);
	REG_WRITE(device_reset_reg, device_reset_val);	/* old value = 0x00000015 may depends on the DSI RX device */
	REG_WRITE(init_count_reg, init_count_val);	/* Minimum value = 0x000007d0 */

	REG_WRITE(dsi_func_prg_reg, dsi_func_prg_val);

	REG_WRITE(vert_sync_pad_count_reg, VSA_count);
	REG_WRITE(vert_back_porch_count_reg, VBP_count);
	REG_WRITE(vert_front_porch_count_reg, VFP_count);

	REG_WRITE(horiz_sync_pad_count_reg, horiz_sync_pad_count_val);
	REG_WRITE(horiz_back_porch_count_reg, horiz_back_porch_count_val);
	REG_WRITE(horiz_front_porch_count_reg, horiz_front_porch_count_val);
	REG_WRITE(horiz_active_area_count_reg, horiz_active_area_count_val);

	REG_WRITE(video_fmt_reg, videoModeFormat | COMPLETE_LAST_PCKT);

	REG_WRITE(dpi_resolution_reg, resolution);

	REG_WRITE(hs_tx_timeout_reg, hs_tx_timeout_val);
	REG_WRITE(lp_rx_timeout_reg, lp_rx_timeout_val);
	REG_WRITE(high_low_switch_count_reg, high_low_switch_count_val);

	REG_WRITE(eot_disable_reg, eot_disable_val);
	REG_WRITE(lp_byteclk_reg, lp_byteclk_val);

	REG_WRITE(device_ready_reg, device_ready_val);

	/* wait for DPI FIFO to clear */
	while ((timeout < 20000)
	       && ((REG_READ(gen_fifo_stat_reg) & DPI_FIFO_EMPTY)
		   != DPI_FIFO_EMPTY)) {
		udelay(100);
	}

	if ((REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
		REG_WRITE(intr_stat_reg, SPL_PKT_SENT);
	}

	REG_WRITE(dpi_control_reg, dpi_control_val);	/* Turn On */
	udelay(20000);
	udelay(20000);

	/* make sure Turn On Packet Sent */
/*      udelay(20000); */
	while (!(REG_READ(intr_stat_reg) & SPL_PKT_SENT)) {
		/* Do Nothing Here */
	}

	REG_WRITE(pipeconf_reg, pipeconf);
	REG_READ(pipeconf_reg);

	/* Wait for for the pipe enable to take effect. */
	mdfldWaitForPipeEnable(dev, pipe);

	REG_WRITE(disp_cntr_reg, dspcntr);

	/* Wait for 20ms for the plane enable to take effect. */
	udelay(20000);

	*pdpi_panel_on = true;

#if DSI_TPO_864x480 || DSI_TPO_864x480_2
	mdfld_init_TPO_MIPI(dev, pipe);
#endif				/* DSI_TPO_864x480 */
	mdfld_dsi_brightness_init(dev, pipe);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mdfld_dbi_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
#if 0				/* FIXME_JLIU7 add it later */
	uint64_t curValue = DRM_MODE_SCALE_FULLSCREEN;
#endif				/* FIXME_JLIU7 add it later */
	u32 DcsPixelFormat = 0;
	u32 DBI_dataWidth = 0;

	u32 bpp = dev_priv->bpp;
	u32 HactiveArea = dev_priv->HactiveArea;
	u32 VactiveArea = dev_priv->VactiveArea;
	u32 channelNumber = dev_priv->channelNumber;
	u32 laneCount = dev_priv->laneCount;
	u32 *pipeconf = &dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	bool *pdbi_panel_on = &(dev_priv->dbi_panel_on);
	u8 *p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer;
	u32 DBI_CB_phys = dev_priv->DBI_CB_phys;
	u32 *pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer);
	u32 mipi_reg = MIPI;
	u32 mipi_control_reg = MIPI_CONTROL_REG;
	u32 intr_en_reg = INTR_EN_REG;
	u32 turn_around_timeout_reg = TURN_AROUND_TIMEOUT_REG;
	u32 device_reset_reg = DEVICE_RESET_REG;
	u32 init_count_reg = INIT_COUNT_REG;
	u32 dsi_func_prg_reg = DSI_FUNC_PRG_REG;
	u32 hs_tx_timeout_reg = HS_TX_TIMEOUT_REG;
	u32 lp_rx_timeout_reg = LP_RX_TIMEOUT_REG;
	u32 high_low_switch_count_reg = HIGH_LOW_SWITCH_COUNT_REG;
	u32 eot_disable_reg = EOT_DISABLE_REG;
	u32 lp_byteclk_reg = LP_BYTECLK_REG;
	u32 device_ready_reg = DEVICE_READY_REG;
	u32 dphy_param_reg = DPHY_PARAM_REG;
	u32 clk_lane_swt_reg = CLK_LANE_SWT_REG;
	u32 dbi_bw_ctrl_reg = DBI_BW_CTRL_REG;
	u32 hs_ls_dbi_enable_reg = HS_LS_DBI_ENABLE_REG;
	u32 mipi_command_length_reg = MIPI_COMMAND_LENGTH_REG;
	u32 mipi_command_address_reg = MIPI_COMMAND_ADDRESS_REG;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 pipeconf_reg = PIPEACONF;
	u32 disp_cntr_reg = DSPACNTR;

	/* defaut to be dbi values on pipe A. */
	u32 mipi_val = PASS_FROM_SPHY_TO_AFE | SEL_FLOPPED_HSTX;
	u32 dphy_param_val = 0x150c3408;	/* dual dbi - dpi *//* single 0x180a2b07; *//*SLE 0x0b14540c; *//* HW SIMU 0x0b061a10; */
	u32 mipi_control_val = 0x00000018;
	u32 hs_tx_timeout_val = 0x3fffff;
	u32 lp_rx_timeout_val = 0xffff;
	u32 turn_around_timeout_val = 0x0000001F;
	u32 device_reset_val = 0x000000ff;	/* old value = 0x00000015 may depends on the DSI RX device */
	u32 intr_en_val = 0xffffffff;
/* MDFLD_PO_JLIU7	u32 eot_disable_val = ENABLE_CLOCK_STOPPING; */
	u32 eot_disable_val = 0;
	u32 init_count_val = 0x000007d0;
	u32 dsi_func_prg_val = 0;
	u32 device_ready_val = 0x00000001;

	/* FIXME_JLIU7 need to update the following MIPI adaptor register values */
	u32 clk_lane_swt_val = 0x000A0014;
	u32 dbi_bw_ctrl_val = 0x00000400;	/* HW SIMU 0x00000820; */
	u32 hs_ls_dbi_enable_val = 0;
	u32 high_low_switch_count_val = 0x46;
	u32 lp_byteclk_val = 0x00000004;	/* FIXME JLIU7 for NSC PO */
	u32 pipe = 0;

	PSB_DEBUG_ENTRY("output_type = 0x%x \n", output->type);

	if (output->type == INTEL_OUTPUT_MIPI2) {
		/* MIPI_A has to be on before we can enable MIPI_C */
		if (!(dev_priv->dpi_panel_on || dev_priv->dbi_panel_on)) {
			DRM_ERROR
			    ("mdfld_dbi_mode_set need to enable MIPI0 before enabling MIPI1. \n");
			return;
		}

		bpp = dev_priv->bpp2;
	}

	switch (bpp) {
	case 16:
		DcsPixelFormat = DCS_PIXEL_FORMAT_16bbp;
		break;
	case 18:
		DcsPixelFormat = DCS_PIXEL_FORMAT_18bbp;
		break;
	case 24:
		DcsPixelFormat = DCS_PIXEL_FORMAT_24bbp;
		break;
	default:
		DRM_INFO("mdfld_dbi_mode_set,  invalid bpp \n");
		break;
	}

	if (output->type == INTEL_OUTPUT_MIPI2) {
		pipe = 2;
		HactiveArea = dev_priv->HactiveArea2;
		VactiveArea = dev_priv->VactiveArea2;
		channelNumber = dev_priv->channelNumber2;
		laneCount = dev_priv->laneCount2;
		pipeconf = &dev_priv->pipeconf2;
		dspcntr = dev_priv->dspcntr2;
		pdbi_panel_on = &(dev_priv->dbi_panel_on2);
		p_DBI_commandBuffer = dev_priv->p_DBI_commandBuffer2;
		DBI_CB_phys = dev_priv->DBI_CB_phys2;
		pDBI_CB_pointer = &(dev_priv->DBI_CB_pointer2);

		mipi_reg = MIPI_C;
		mipi_control_reg = MIPI_CONTROL_REG + MIPIC_REG_OFFSET;
		intr_en_reg = INTR_EN_REG + MIPIC_REG_OFFSET;
		device_reset_reg = DEVICE_RESET_REG + MIPIC_REG_OFFSET;
		turn_around_timeout_reg =
		    TURN_AROUND_TIMEOUT_REG + MIPIC_REG_OFFSET;
		init_count_reg = INIT_COUNT_REG + MIPIC_REG_OFFSET;
		dsi_func_prg_reg = DSI_FUNC_PRG_REG + MIPIC_REG_OFFSET;
		hs_tx_timeout_reg = HS_TX_TIMEOUT_REG + MIPIC_REG_OFFSET;
		lp_rx_timeout_reg = LP_RX_TIMEOUT_REG + MIPIC_REG_OFFSET;
		high_low_switch_count_reg =
		    HIGH_LOW_SWITCH_COUNT_REG + MIPIC_REG_OFFSET;
		eot_disable_reg = EOT_DISABLE_REG + MIPIC_REG_OFFSET;
		lp_byteclk_reg = LP_BYTECLK_REG + MIPIC_REG_OFFSET;
		device_ready_reg = DEVICE_READY_REG + MIPIC_REG_OFFSET;
		dphy_param_reg = DPHY_PARAM_REG + MIPIC_REG_OFFSET;
		clk_lane_swt_reg = CLK_LANE_SWT_REG + MIPIC_REG_OFFSET;
		dbi_bw_ctrl_reg = DBI_BW_CTRL_REG + MIPIC_REG_OFFSET;
		hs_ls_dbi_enable_reg = HS_LS_DBI_ENABLE_REG + MIPIC_REG_OFFSET;
		mipi_command_length_reg =
		    MIPI_COMMAND_LENGTH_REG + MIPIC_REG_OFFSET;
		mipi_command_address_reg =
		    MIPI_COMMAND_ADDRESS_REG + MIPIC_REG_OFFSET;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;

		pipeconf_reg = PIPECCONF;
		disp_cntr_reg = DSPCCNTR;

	} else {
		mipi_val |= dev_priv->mipi_lane_config;
	}

	*pipeconf |= PIPEACONF_DSR;
	channelNumber <<= DBI_CHANNEL_NUMBER_POS;
	DBI_dataWidth = DBI_DATA_WIDTH_OPT2 << DBI_DATA_WIDTH_POS;
	dsi_func_prg_val = laneCount | channelNumber | DBI_dataWidth;

	if (!ospm_power_using_hw_begin
	    (OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;

#if 0				/* FIXME_JLIU7 add it later */
	drm_connector_property_get_value(&enc_to_psb_intel_output
					 (encoder)->base,
					 dev->mode_config.scaling_mode_property,
					 &curValue);

	if (curValue == DRM_MODE_SCALE_NO_SCALE)
		REG_WRITE(PFIT_CONTROL, 0);
	else if (curValue == DRM_MODE_SCALE_ASPECT) {
		if ((mode->vdisplay != adjusted_mode->crtc_vdisplay)
		    || (mode->hdisplay != adjusted_mode->crtc_hdisplay)) {
			if ((adjusted_mode->crtc_hdisplay * mode->vdisplay) ==
			    (mode->hdisplay * adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
			else if ((adjusted_mode->crtc_hdisplay *
				  mode->vdisplay) >
				 (mode->hdisplay *
				  adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL,
					  PFIT_ENABLE |
					  PFIT_SCALING_MODE_PILLARBOX);
			else
				REG_WRITE(PFIT_CONTROL,
					  PFIT_ENABLE |
					  PFIT_SCALING_MODE_LETTERBOX);
		} else
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
	} else			/*(curValue == DRM_MODE_SCALE_FULLSCREEN) */
		REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
#endif				/* FIXME_JLIU7 add it later */

	/* clear intr status register */

	/* Clear Device Ready Bit */
	REG_WRITE(device_ready_reg, 0x00000000);

	REG_WRITE(mipi_reg, mipi_val);

	/* FIXME_MDFLD JLIU7 revisit MIPI_CONTROL_REG */
	/* JLIU7_FIXME set MIPI clock ratio to 1:1 for NSC init */
	REG_WRITE(mipi_control_reg, mipi_control_val);

	/* Get the value from HW SV */
	REG_WRITE(dphy_param_reg, dphy_param_val);
	REG_WRITE(clk_lane_swt_reg, clk_lane_swt_val);
	REG_WRITE(dbi_bw_ctrl_reg, dbi_bw_ctrl_val);
	REG_WRITE(hs_ls_dbi_enable_reg, hs_ls_dbi_enable_val);

	/* Enable all the error interrupt */
	REG_WRITE(intr_en_reg, intr_en_val);
	REG_WRITE(turn_around_timeout_reg, turn_around_timeout_val);
	REG_WRITE(device_reset_reg, device_reset_val);	/* old value = 0x00000015 may depends on the DSI RX device */
	REG_WRITE(init_count_reg, init_count_val);	/* Minimum value = 0x000007d0 */
	REG_WRITE(dsi_func_prg_reg, dsi_func_prg_val);
	REG_WRITE(hs_tx_timeout_reg, hs_tx_timeout_val);
	REG_WRITE(lp_rx_timeout_reg, lp_rx_timeout_val);
	REG_WRITE(high_low_switch_count_reg, high_low_switch_count_val);
	REG_WRITE(eot_disable_reg, eot_disable_val);
	REG_WRITE(lp_byteclk_reg, lp_byteclk_val);
	REG_WRITE(device_ready_reg, device_ready_val);

	/* Wait for 20ms for the pipe enable to take effect. */
	udelay(20000);

	REG_WRITE(disp_cntr_reg, dspcntr);

	/* Wait for 20ms for the plane enable to take effect. */
	udelay(20000);

	/* FIXME_JLIU7 MDFLD_PO */
	/* disable all the MIPI interrupts at the beginning. */
	/* enable all the MIPI interrupts at the end. */
	/* Make sure dbi command & data buffer are empty */
	if (*pDBI_CB_pointer != 0) {
		DRM_ERROR
		    ("dbi command buffer was interrupted before finished. \n");
	}

	mdfld_dsi_dbi_CB_ready(dev, mipi_command_address_reg,
			       gen_fifo_stat_reg);
	/* Wait for 20ms. */
	udelay(20000);

	/* exit sleep mode */
	*p_DBI_commandBuffer = exit_sleep_mode;

	/* FIXME_jliu7 mapVitualToPhysical(dev_priv->p_DBI_commandBuffer); */
	REG_WRITE(mipi_command_length_reg, 1);
	udelay(5000);
	REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0);

	/* The host processor must wait five milliseconds after sending exit_sleep_mode command before sending another
	   command. This delay allows the supply voltages and clock circuits to stabilize */
	udelay(5000);

	mdfld_dsi_dbi_CB_ready(dev, mipi_command_address_reg, 0);
	/* Wait for 20ms. */
	udelay(20000);
	mdfld_dsi_brightness_init(dev, pipe);
	mdfld_dsi_gen_fifo_ready(dev, gen_fifo_stat_reg,
				 HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);

	REG_WRITE(pipeconf_reg, *pipeconf);
	REG_READ(pipeconf_reg);

	*pDBI_CB_pointer = 0;
	/* set_column_address */
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = set_column_address;
	*((u16 *) (p_DBI_commandBuffer + *pDBI_CB_pointer)) = 0;
	*pDBI_CB_pointer += 2;
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = (HactiveArea - 1) >> 8;
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = HactiveArea - 1;

	*pDBI_CB_pointer = 8;
	/* set_page_addr */
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = set_page_addr;
	*((u16 *) (p_DBI_commandBuffer + *pDBI_CB_pointer)) = 0;
	*pDBI_CB_pointer += 2;
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = (VactiveArea - 1) >> 8;
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = VactiveArea - 1;

#if 0				/*FIXME JLIU7 */
	/* write_LUT if needed. */
#endif				/*FIXME JLIU7 */

	*pDBI_CB_pointer = 16;
	/* write_mem_start */
	*(p_DBI_commandBuffer + (*pDBI_CB_pointer)++) = write_mem_start;

	REG_WRITE(mipi_command_length_reg, 0x010505);
	udelay(5000);
	REG_WRITE(mipi_command_address_reg, DBI_CB_phys | BIT0 | BIT1);
	udelay(5000);

	/* FIXME_Enable pipe vblank interrupt */

	*pDBI_CB_pointer = 0;

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

#if MDFLD_JLIU7_DSR
	if (output->type == INTEL_OUTPUT_MIPI) {
		mdfld_dsr_timer_init(dev_priv);
	}
#endif				/* MDFLD_JLIU7_DSR */

}

static void mdfld_dsi_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	bool dpi = dev_priv->dpi;

	if (output->type == INTEL_OUTPUT_MIPI2)
		dpi = dev_priv->dpi2;

	if (dpi)
		mdfld_dpi_mode_set(encoder, mode, adjusted_mode);
	else
		mdfld_dbi_mode_set(encoder, mode, adjusted_mode);
}

void mdfld_dsi_commit(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
#if 0				/* FIXME_MDFLD JLIU7_PO */
	struct psb_intel_mode_device *mode_dev = output->mode_dev;
#endif				/* FIXME_MDFLD JLIU7_PO */

	PSB_DEBUG_ENTRY("Enter  mdfld_dsi_commit \n");

#if 0				/* FIXME_MDFLD JLIU7_PO */
	if (mode_dev->backlight_duty_cycle == 0)
		mode_dev->backlight_duty_cycle =
		    mrst_dsi_get_max_backlight(dev);
#endif				/* FIXME_MDFLD JLIU7_PO */

	mdfld_dsi_set_power(dev, output, true);
}

static const struct drm_encoder_helper_funcs mdfld_dsi_helper_funcs = {
	.dpms = mdfld_dsi_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = mdfld_dsi_prepare,
	.mode_set = mdfld_dsi_mode_set,
	.commit = mdfld_dsi_commit,
};

static const struct drm_connector_funcs mdfld_dsi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.save = mrst_dsi_save,
	.restore = mrst_dsi_restore,
	.detect = mdfld_dsi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = mdfld_dsi_set_property,
	.destroy = psb_intel_lvds_destroy,
};

/* ************************************************************************* *\
FUNCTION: mrstDSI_clockInit
																`
DESCRIPTION:

\* ************************************************************************* */
static u32 mdfld_sku_83_mipi_2xclk[4] = { 166667, 333333, 444444, 666667 };
static u32 mdfld_sku_100_mipi_2xclk[4] = { 200000, 400000, 533333, 800000 };
static u32 mdfld_sku_100L_mipi_2xclk[4] = { 100000, 200000, 266667, 400000 };

#define MDFLD_MIPI_2XCLK_COUNT			0x04

static bool mdfldDSI_clockInit(DRM_DRIVER_PRIVATE_T *dev_priv, int dsi_num)
{
	u32 Htotal = 0, Vtotal = 0, RRate = 0, mipi_2xclk = 0;
	u32 i = 0;
	u32 *p_mipi_2xclk = NULL;
	u32 pixelClock, HsyncWidth, HbackPorch, HfrontPorch, HactiveArea,
	    VsyncWidth, VbackPorch, VfrontPorch, VactiveArea, bpp;
	u32 laneCount, DDR_Clock_Calculated;

	if (dsi_num == 1) {
		laneCount = dev_priv->laneCount;
		pixelClock = dev_priv->pixelClock;
		HsyncWidth = dev_priv->HsyncWidth;
		HbackPorch = dev_priv->HbackPorch;
		HfrontPorch = dev_priv->HfrontPorch;
		HactiveArea = dev_priv->HactiveArea;
		VsyncWidth = dev_priv->VsyncWidth;
		VbackPorch = dev_priv->VbackPorch;
		VfrontPorch = dev_priv->VfrontPorch;
		VactiveArea = dev_priv->VactiveArea;
		bpp = dev_priv->bpp;
	} else {
		laneCount = dev_priv->laneCount2;
		pixelClock = dev_priv->pixelClock2;
		HsyncWidth = dev_priv->HsyncWidth2;
		HbackPorch = dev_priv->HbackPorch2;
		HfrontPorch = dev_priv->HfrontPorch2;
		HactiveArea = dev_priv->HactiveArea2;
		VsyncWidth = dev_priv->VsyncWidth2;
		VbackPorch = dev_priv->VbackPorch2;
		VfrontPorch = dev_priv->VfrontPorch2;
		VactiveArea = dev_priv->VactiveArea2;
		bpp = dev_priv->bpp2;
	}

	Htotal = HsyncWidth + HbackPorch + HfrontPorch + HactiveArea;
	Vtotal = VsyncWidth + VbackPorch + VfrontPorch + VactiveArea;

	RRate = ((pixelClock * 1000) / (Htotal * Vtotal)) + 1;

	/* ddr clock frequence = (pixel clock frequence *  bits per pixel)/2 */
	mipi_2xclk = (pixelClock * bpp) / laneCount;	/* KHz */

	if (dsi_num == 1) {
		dev_priv->RRate = RRate;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated = mipi_2xclk / 2;	/* KHz */
	} else {
		dev_priv->RRate2 = RRate;
		DDR_Clock_Calculated = dev_priv->DDR_Clock_Calculated2 = mipi_2xclk / 2;	/* KHz */
	}

	PSB_DEBUG_ENTRY("RRate = %d, mipi_2xclk = %d. \n", RRate, mipi_2xclk);

	switch (dev_priv->core_freq) {
	case 100:
		p_mipi_2xclk = mdfld_sku_100L_mipi_2xclk;
		break;
	case 166:
		p_mipi_2xclk = mdfld_sku_83_mipi_2xclk;
		break;
	case 200:
		p_mipi_2xclk = mdfld_sku_100_mipi_2xclk;
		break;
	default:
		PSB_DEBUG_ENTRY("Invalid core_freq = %d. \n",
				dev_priv->core_freq);
		return false;
	}

	for (; i < MDFLD_MIPI_2XCLK_COUNT; i++) {
		if (p_mipi_2xclk) {
			if ((DDR_Clock_Calculated * 2) < p_mipi_2xclk[i]) {
				break;
			}
		}
	}

	if (i == MIPI_2XCLK_COUNT) {
		PSB_DEBUG_ENTRY
		    ("the DDR clock is too big, DDR_Clock_Calculated is = %d\n",
		     dev_priv->DDR_Clock_Calculated);
		return false;
	}

	if (p_mipi_2xclk) {
		if (dsi_num == 1) {
			dev_priv->DDR_Clock = p_mipi_2xclk[i] / 2;
			dev_priv->ClockBits = i;
		} else {
			dev_priv->DDR_Clock2 = p_mipi_2xclk[i] / 2;
			dev_priv->ClockBits2 = i;
		}
	}
#if 0				/*JLIU7_PO */
#if 0				/* FIXME remove it after power on */
	mipiControlReg = REG_READ(MIPI_CONTROL_REG) & (~MIPI_2X_CLOCK_BITS);
	mipiControlReg |= i;
	REG_WRITE(MIPI_CONTROL_REG, mipiControlReg);
#else				/* FIXME remove it after power on */
	mipiControlReg |= i;
	REG_WRITE(MIPI_CONTROL_REG, mipiControlReg);
#endif				/* FIXME remove it after power on */
#endif				/*JLIU7_PO */

	PSB_DEBUG_ENTRY
	    ("mipi_2x_clock_divider = 0x%x, DDR_Clock_Calculated is = %d\n", i,
	     DDR_Clock_Calculated);

	return true;
}

/** Returns the panel fixed mode from configuration. */
/** FIXME JLIU7 need to revist it. */
struct drm_display_mode *mid_dsi_get_configuration_mode(struct drm_device *dev,
							int dsi_num)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
#if MDFLD_GCT_JLIU7
	u8 panel_index = dev_priv->gct_data.bpi;
	u8 panel_type = dev_priv->gct_data.pt;
#endif				/* MDFLD_GCT_JLIU7 */
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false;
	uint32_t Panel_RRate = 0;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

#if MDFLD_GCT_JLIU7
	if (dev_priv->vbt_data.Size != 0x00)	/*if non-zero, vbt is present */
		if ((1 << panel_index) & panel_type)	/* if non-zero, */
			use_gct = true;	/*then mipi panel. */
#endif				/* MDFLD_GCT_JLIU7 */

	if (use_gct) {
		PSB_DEBUG_ENTRY("gct find MIPI panel. \n");

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
		if (dsi_num == 1) {
#if DSI_TPO_864x480		/*FIXME jliu7 remove it later */
			mode->hdisplay = 864;
			mode->vdisplay = 480;
			mode->hsync_start = 873;
			mode->hsync_end = 876;
			mode->htotal = 887;
			mode->vsync_start = 487;
			mode->vsync_end = 490;
			mode->vtotal = 499;
			mode->clock = 33264;

			dev_priv->dpi = true;
			dev_priv->bpp = 24;
			dev_priv->videoModeFormat = BURST_MODE;
			dev_priv->laneCount = 2;
			dev_priv->channelNumber = 0;
#endif				/*FIXME jliu7 remove it later */
#if DBI_TPO_864x480		/* get from spec. */
			mode->hdisplay = 864;
			mode->vdisplay = 480;

			Panel_RRate = 60;
			dev_priv->dpi = false;
			dev_priv->bpp = 24;

			/* FIXME hard code values. */
			dev_priv->laneCount = 2;
			dev_priv->channelNumber = 0;
#endif				/*FIXME jliu7 remove it later */
#if DBI_TPO_480x864		/* get from spec. */
			mode->hdisplay = 480;
			mode->vdisplay = 864;

			Panel_RRate = 60;
			dev_priv->dpi = false;
			dev_priv->bpp = 24;

			/* FIXME hard code values. */
			dev_priv->laneCount = 2;
			dev_priv->channelNumber = 0;
#endif				/*FIXME jliu7 remove it later */
		} else {
#if DSI_TPO_864x480_2		/*FIXME jliu7 remove it later */
			mode->hdisplay = 864;
			mode->vdisplay = 480;
			mode->hsync_start = 873;
			mode->hsync_end = 876;
			mode->htotal = 887;
			mode->vsync_start = 487;
			mode->vsync_end = 490;
			mode->vtotal = 499;
			mode->clock = 33264;

			dev_priv->dpi2 = true;
			dev_priv->bpp2 = 24;
			dev_priv->videoModeFormat2 = BURST_MODE;
			dev_priv->laneCount2 = 2;
			dev_priv->channelNumber2 = 0;
#endif				/*FIXME jliu7 remove it later */
#if DBI_TPO_864x480_2		/* get from spec. */
			mode->hdisplay = 864;
			mode->vdisplay = 480;

			Panel_RRate = 60;
			dev_priv->dpi2 = false;
			dev_priv->bpp2 = 24;

			/* FIXME hard code values. */
			dev_priv->laneCount2 = 2;
			dev_priv->channelNumber2 = 0;
#endif				/*FIXME jliu7 remove it later */
#if DBI_TPO_480x864_2		/* get from spec. */
			mode->hdisplay = 480;
			mode->vdisplay = 864;

			Panel_RRate = 60;
			dev_priv->dpi2 = false;
			dev_priv->bpp2 = 24;

			/* FIXME hard code values. */
			dev_priv->laneCount2 = 2;
			dev_priv->channelNumber2 = 0;
#endif				/*FIXME jliu7 remove it later */
		}

	}

	if (((dsi_num == 1) && !dev_priv->dpi)
	    || ((dsi_num == 2) && !dev_priv->dpi2)) {

		mode->hsync_start = mode->hdisplay + 8;
		mode->hsync_end = mode->hsync_start + 4;
		mode->htotal = mode->hsync_end + 8;
		mode->vsync_start = mode->vdisplay + 2;
		mode->vsync_end = mode->vsync_start + 2;
		mode->vtotal = mode->vsync_end + 2;
		mode->clock =
		    (mode->htotal * mode->vtotal * Panel_RRate) / 1000;
	}

	if (dsi_num == 1) {
		dev_priv->pixelClock = mode->clock;	/*KHz */
		dev_priv->HsyncWidth = mode->hsync_end - mode->hsync_start;
		dev_priv->HbackPorch = mode->htotal - mode->hsync_end;
		dev_priv->HfrontPorch = mode->hsync_start - mode->hdisplay;
		dev_priv->HactiveArea = mode->hdisplay;
		dev_priv->VsyncWidth = mode->vsync_end - mode->vsync_start;
		dev_priv->VbackPorch = mode->vtotal - mode->vsync_end;
		dev_priv->VfrontPorch = mode->vsync_start - mode->vdisplay;
		dev_priv->VactiveArea = mode->vdisplay;

		PSB_DEBUG_ENTRY("pixelClock is %d\n", dev_priv->pixelClock);
		PSB_DEBUG_ENTRY("HsyncWidth is %d\n", dev_priv->HsyncWidth);
		PSB_DEBUG_ENTRY("HbackPorch is %d\n", dev_priv->HbackPorch);
		PSB_DEBUG_ENTRY("HfrontPorch is %d\n", dev_priv->HfrontPorch);
		PSB_DEBUG_ENTRY("HactiveArea is %d\n", dev_priv->HactiveArea);
		PSB_DEBUG_ENTRY("VsyncWidth is %d\n", dev_priv->VsyncWidth);
		PSB_DEBUG_ENTRY("VbackPorch is %d\n", dev_priv->VbackPorch);
		PSB_DEBUG_ENTRY("VfrontPorch is %d\n", dev_priv->VfrontPorch);
		PSB_DEBUG_ENTRY("VactiveArea is %d\n", dev_priv->VactiveArea);
	} else {
		dev_priv->pixelClock2 = mode->clock;	/*KHz */
		dev_priv->HsyncWidth2 = mode->hsync_end - mode->hsync_start;
		dev_priv->HbackPorch2 = mode->htotal - mode->hsync_end;
		dev_priv->HfrontPorch2 = mode->hsync_start - mode->hdisplay;
		dev_priv->HactiveArea2 = mode->hdisplay;
		dev_priv->VsyncWidth2 = mode->vsync_end - mode->vsync_start;
		dev_priv->VbackPorch2 = mode->vtotal - mode->vsync_end;
		dev_priv->VfrontPorch2 = mode->vsync_start - mode->vdisplay;
		dev_priv->VactiveArea2 = mode->vdisplay;

		PSB_DEBUG_ENTRY("pixelClock2 is %d\n", dev_priv->pixelClock2);
		PSB_DEBUG_ENTRY("HsyncWidth2 is %d\n", dev_priv->HsyncWidth2);
		PSB_DEBUG_ENTRY("HbackPorch2 is %d\n", dev_priv->HbackPorch2);
		PSB_DEBUG_ENTRY("HfrontPorch2 is %d\n", dev_priv->HfrontPorch2);
		PSB_DEBUG_ENTRY("HactiveArea2 is %d\n", dev_priv->HactiveArea2);
		PSB_DEBUG_ENTRY("VsyncWidth2 is %d\n", dev_priv->VsyncWidth2);
		PSB_DEBUG_ENTRY("VbackPorch2 is %d\n", dev_priv->VbackPorch2);
		PSB_DEBUG_ENTRY("VfrontPorch2 is %d\n", dev_priv->VfrontPorch2);
		PSB_DEBUG_ENTRY("VactiveArea2 is %d\n", dev_priv->VactiveArea2);
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

/**
 * mdfld_dsi_init - setup MIPI pipe A connectors on this device
 * @dev: drm device
 *
 * Create the connector, try to figure out what
 * modes we can display on the MIPI panel (if present).
 */
void mid_dsi_init(struct drm_device *dev,
		  struct psb_intel_mode_device *mode_dev, int dsi_num)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_display_mode *panel_fixed_mode;
	struct psb_gtt *pg = dev_priv->pg;

	PSB_DEBUG_ENTRY("Enter mid_dsi_init2\n");

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	drm_connector_init(dev, &psb_intel_output->base,
			   &mdfld_dsi_connector_funcs, DRM_MODE_CONNECTOR_MIPI);

	drm_encoder_init(dev, &psb_intel_output->enc, &psb_intel_lvds_enc_funcs,
			 DRM_MODE_ENCODER_MIPI);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type =
	    (dsi_num == 1) ? INTEL_OUTPUT_MIPI : INTEL_OUTPUT_MIPI2;

	drm_encoder_helper_add(encoder, &mdfld_dsi_helper_funcs);
	drm_connector_helper_add(connector, &mrst_dsi_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_connector_attach_property(connector,
				      dev->mode_config.scaling_mode_property,
				      DRM_MODE_SCALE_FULLSCREEN);
	drm_connector_attach_property(connector, dev_priv->backlight_property,
				      BRIGHTNESS_MAX_LEVEL);

	if (dsi_num == 1) {
		dsi_backlight = BRIGHTNESS_MAX_LEVEL;
		blc_pol = BLC_POLARITY_NORMAL;
		blc_freq = 0xc8;

		/*
		 * MIPI discovery:
		 * 1) check for DDB data
		 * 2) check for VBT data
		 * 4) make sure lid is open
		 *    if closed, act like it's not there for now
		 */

		/* FIXME change it to true if GET_DDB works */
		dev_priv->config_phase = false;
		dev_priv->mipi_lane_config = 0x0;
#if MDFLD_JLIU7_DSR
		dev_priv->dsr_fb_update = MDFLD_DSR_2D_3D;
		dev_priv->b_dsr_enable = false;
#endif				/* MDFLD_JLIU7_DSR */

	} else {
		dsi_backlight2 = BRIGHTNESS_MAX_LEVEL;
		blc_pol2 = BLC_POLARITY_NORMAL;
		blc_freq2 = 0xc8;

		/*
		 * MIPI discovery:
		 * 1) check for DDB data
		 * 2) check for VBT data
		 * 4) make sure lid is open
		 *    if closed, act like it's not there for now
		 */

		/* FIXME change it to true if GET_DDB works */
		dev_priv->config_phase2 = false;
		dev_priv->mipi_lane_config = 0x2;
	}

	/*
	 * If we didn't get DDB data, try geting panel timing
	 * from configuration data
	 */
	panel_fixed_mode = mid_dsi_get_configuration_mode(dev, dsi_num);

	if (dsi_num == 1) {
		/* GPIO control to reset MIP */
		gpio_request(128, "gfx");
		gpio_direction_output(128, 1);
		gpio_get_value(128);
		mode_dev->panel_fixed_mode = panel_fixed_mode;
	} else {
		/* GPIO control to reset MIP */
		gpio_request(34, "gfx");
		gpio_direction_output(34, 1);
		gpio_get_value(128);
		mode_dev->panel_fixed_mode2 = panel_fixed_mode;
	}

	if (panel_fixed_mode) {
		panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
	} else {
		/* If we still don't have a mode after all that, give up. */
		DRM_DEBUG("Found no modes on the lvds, ignoring the LVDS\n");
		goto failed_find;
	}

	if (!mdfldDSI_clockInit(dev_priv, dsi_num)) {
		DRM_DEBUG("Can't iniitialize MRST DSI clock.\n");
#if 0				/* FIXME JLIU7 */
		goto failed_find;
#endif				/* FIXME JLIU7 */
	}

	if (dsi_num == 1) {
		/* FIXME_JLIU7 MDFLD_PO do we need to allocate DMA-capable memory? */
		/*      dev_priv->p_DBI_commandBuffer = (u8 *)((u32)kzalloc(DBI_COMMAND_BUFFER_SIZE, GFP_KERNEL) & ALIGNMENT_32BYTE_MASK); */
		dev_priv->DBI_CB_phys = pg->gtt_phys_start - 0x1000;
		dev_priv->p_DBI_commandBuffer =
		    (u8 *) ioremap_nocache(dev_priv->DBI_CB_phys, 0x800);
		if (!dev_priv->p_DBI_commandBuffer)
			goto failed_find;

		/*dev_priv->DBI_CB_phys = (u32) virt_to_phys(dev_priv->p_DBI_commandBuffer); */
		PSB_DEBUG_DBI_BF
		    ("mid_dsi_init2 p_DBI_commandBuffer = 0x%x, DBI_CB_phys = 0x%x. \n",
		     (u32) dev_priv->p_DBI_commandBuffer,
		     dev_priv->DBI_CB_phys);
	} else {
		dev_priv->DBI_CB_phys2 = pg->gtt_phys_start - 0x800;
		dev_priv->p_DBI_commandBuffer2 =
		    (u8 *) ioremap_nocache(dev_priv->DBI_CB_phys2, 0x800);
		if (!dev_priv->p_DBI_commandBuffer2)
			goto failed_find;

		/*dev_priv->DBI_CB_phys2 = (u32) virt_to_phys(dev_priv->p_DBI_commandBuffer2); */
		PSB_DEBUG_DBI_BF
		    ("mid_dsi_init2 p_DBI_commandBuffer2 = 0x%x, DBI_CB_phys2 = 0x%x. \n",
		     (u32) dev_priv->p_DBI_commandBuffer2,
		     dev_priv->DBI_CB_phys2);
	}

	dev_priv->first_boot = true;

	drm_sysfs_connector_add(connector);
	return;

 failed_find:
	DRM_DEBUG("No MIIP modes found, disabling.\n");
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	kfree(connector);
}
