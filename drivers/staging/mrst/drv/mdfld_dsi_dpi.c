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
 * jim liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include "psb_drv.h"
#include "mdfld_csc.h"

/* Local functions */
static void mdfld_dsi_dpi_shut_down(struct mdfld_dsi_dpi_output *output, int pipe);
struct drm_encoder *gencoder;
extern struct drm_device *gpDrmDevice;
extern int drm_psb_enable_gamma;
extern int drm_psb_enable_color_conversion;

static void mdfld_wait_for_HS_DATA_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		gen_fifo_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
		(REG_READ(gen_fifo_stat_reg) & DSI_FIFO_GEN_HS_DATA_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: HS Data FIFO was never cleared!\n");
}

static void mdfld_wait_for_HS_CTRL_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		gen_fifo_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
		(REG_READ(gen_fifo_stat_reg) & DSI_FIFO_GEN_HS_CTRL_FULL)) {
		udelay(100);
		timeout++;
	}
	if (timeout == 20000)
		DRM_INFO("MIPI: HS CMD FIFO was never cleared!\n");
}

static void mdfld_wait_for_PIPEA_DISABLE(struct drm_device *dev, u32 pipe)
{
	u32 pipeconf_reg = PIPEACONF;
	int timeout = 0;

	if (pipe == 2)
		pipeconf_reg = PIPECCONF;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(pipeconf_reg) & 0x40000000)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: PIPE was not disabled!\n");
}

static void mdfld_wait_for_DPI_CTRL_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		gen_fifo_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && ((REG_READ(gen_fifo_stat_reg) & DPI_FIFO_EMPTY)
		!= DPI_FIFO_EMPTY)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: DPI FIFO was never cleared!\n");
}

static void mdfld_wait_for_SPL_PKG_SENT(struct drm_device *dev, u32 pipe)
{
	u32 intr_stat_reg = MIPIA_INTR_STAT_REG;
	int timeout = 0;

	if (pipe == 2)
		intr_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
		(!(REG_READ(intr_stat_reg) & DSI_INTR_STATE_SPL_PKG_SENT))) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: SPL_PKT was not sent successfully!\n");
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_dsi_tpo_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
void mdfld_dsi_tpo_ic_init(struct mdfld_dsi_config *dsi_config, u32 pipe)
{
	struct drm_device *dev = dsi_config->dev;
	u32 dcsChannelNumber = dsi_config->channel_num;
	u32 gen_data_reg = MIPIA_HS_GEN_DATA_REG;
	u32 gen_ctrl_reg = MIPIA_HS_GEN_CTRL_REG;
	u32 gen_ctrl_val = GEN_LONG_WRITE;

	DRM_INFO("Enter mrst init TPO MIPI display.\n");

	if (pipe == 2) {
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

static u16 mdfld_dsi_dpi_to_byte_clock_count(int pixel_clock_count, int num_lane, int bpp)
{
	return (u16)((pixel_clock_count * bpp) / (num_lane * 8)); 
}

/*
 * Calculate the dpi time basing on a given drm mode @mode
 * return 0 on success.
 * FIXME: I was using proposed mode value for calculation, may need to 
 * use crtc mode values later 
 */
int mdfld_dsi_dpi_timing_calculation(struct drm_display_mode *mode,
				struct mdfld_dsi_dpi_timing *dpi_timing,
				int num_lane, int bpp)
{
	int pclk_hsync, pclk_hfp, pclk_hbp, pclk_hactive;
	int pclk_vsync, pclk_vfp, pclk_vbp, pclk_vactive;
	
	if(!mode || !dpi_timing) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}
	
	PSB_DEBUG_ENTRY("pclk %d, hdisplay %d, hsync_start %d, hsync_end %d, htotal %d\n", 
			mode->clock, mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal);
	PSB_DEBUG_ENTRY("vdisplay %d, vsync_start %d, vsync_end %d, vtotal %d\n", 
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal);
	
	pclk_hactive = mode->hdisplay;
	pclk_hfp = mode->hsync_start - mode->hdisplay;
	pclk_hsync = mode->hsync_end - mode->hsync_start;
	pclk_hbp = mode->htotal - mode->hsync_end;
	
	pclk_vactive = mode->vdisplay;
	pclk_vfp = mode->vsync_start - mode->vdisplay;
	pclk_vsync = mode->vsync_end - mode->vsync_start;
	pclk_vbp = mode->vtotal - mode->vsync_end;

#ifdef MIPI_DEBUG_LOG
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hactive = %d\n", __func__, pclk_hactive);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hfp = %d\n", __func__, pclk_hfp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hsync = %d\n", __func__, pclk_hsync);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hbp = %d\n", __func__, pclk_hbp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vactive = %d\n", __func__, pclk_vactive);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vfp = %d\n", __func__, pclk_vfp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vsync = %d\n", __func__, pclk_vsync);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vbp = %d\n", __func__, pclk_vbp);
#endif
	/*
	 * byte clock counts were calculated by following formula
	 * bclock_count = pclk_count * bpp / num_lane / 8
	 */
	dpi_timing->hsync_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hsync, num_lane, bpp);
	dpi_timing->hbp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hbp, num_lane, bpp);
	dpi_timing->hfp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hfp, num_lane, bpp);
	dpi_timing->hactive_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hactive, num_lane, bpp);

	dpi_timing->vsync_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vsync, num_lane, bpp);
	dpi_timing->vbp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vbp, num_lane, bpp);
	dpi_timing->vfp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vfp, num_lane, bpp);

	PSB_DEBUG_ENTRY("DPI timings: %d, %d, %d, %d, %d, %d, %d\n", 
			dpi_timing->hsync_count, dpi_timing->hbp_count,
			dpi_timing->hfp_count, dpi_timing->hactive_count,
			dpi_timing->vsync_count, dpi_timing->vbp_count,
			dpi_timing->vfp_count);

	return 0; 
}

void mdfld_dsi_dpi_controller_init(struct mdfld_dsi_config * dsi_config, int pipe)
{
	struct drm_device * dev = dsi_config->dev;
	u32 reg_offset = pipe ? MIPIC_REG_OFFSET : 0;
	int lane_count = dsi_config->lane_count;
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct drm_display_mode * mode = dsi_config->mode;
	u32 val = 0;
	
	PSB_DEBUG_ENTRY("Init DPI interface on pipe %d...\n", pipe);

	/*un-ready device*/
	REG_FLD_MOD(DEVICE_READY_REG + reg_offset, 0, 0, 0);
	
	/*init dsi adapter before kicking off*/
	REG_WRITE((MIPIA_CONTROL_REG + reg_offset), 0x00000018);
	
	/*enable all interrupts*/
	REG_WRITE((MIPIA_INTR_EN_REG + reg_offset), 0xffffffff);
	
	/*set up func_prg*/
	val |= lane_count;
	val |= dsi_config->channel_num << DSI_DPI_VIRT_CHANNEL_OFFSET;
		
	switch(dsi_config->bpp) {
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
		DRM_ERROR("unsupported color format, bpp = %d\n", dsi_config->bpp);
	}
	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);
	
	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset), 
			(mode->vtotal * mode->htotal * dsi_config->bpp / (8 * lane_count)) & DSI_HS_TX_TIMEOUT_MASK);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset), 0xffff & DSI_LP_RX_TIMEOUT_MASK);
	
	/*max value: 20 clock cycles of txclkesc*/
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset), 0x14 & DSI_TURN_AROUND_TIMEOUT_MASK);
	
	/*min 21 txclkesc, max: ffffh*/
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset), 0xffff & DSI_RESET_TIMER_MASK);

	REG_WRITE((MIPIA_DPI_RESOLUTION_REG + reg_offset), mode->vdisplay << 16 | mode->hdisplay);
	
	/*set DPI timing registers*/
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing, dsi_config->lane_count, dsi_config->bpp);
	
	REG_WRITE((MIPIA_HSYNC_COUNT_REG + reg_offset), dpi_timing.hsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HBP_COUNT_REG + reg_offset), dpi_timing.hbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HFP_COUNT_REG + reg_offset), dpi_timing.hfp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HACTIVE_COUNT_REG + reg_offset), dpi_timing.hactive_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VSYNC_COUNT_REG + reg_offset), dpi_timing.vsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VBP_COUNT_REG + reg_offset), dpi_timing.vbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VFP_COUNT_REG + reg_offset), dpi_timing.vfp_count & DSI_DPI_TIMING_MASK);
	
	/*min: 7d0 max: 4e20*/
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0x000007d0);
	
	/*set up video mode*/
	val = 0;
	val = dsi_config->video_mode | DSI_DPI_COMPLETE_LAST_LINE;
	REG_WRITE((MIPIA_VIDEO_MODE_FORMAT_REG + reg_offset), val);
	
	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);
	
	/*TODO: figure out how to setup these registers*/
	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);
	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3408);
	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset), (0xa << 16) | 0x14);

	/*set device ready*/
	REG_FLD_MOD((DEVICE_READY_REG + reg_offset), 1, 0, 0);
}
void mdfld_dsi_dpi_set_color_mode(struct mdfld_dsi_config *dsi_config , bool on)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	u32  spk_pkg =  (on == true) ? MDFLD_DSI_DPI_SPK_COLOR_MODE_ON :
		MDFLD_DSI_DPI_SPK_COLOR_MODE_OFF;


	PSB_DEBUG_ENTRY("Turn  color mode %s  pkg value= %d...\n",
		(on ? "on" : "off"), spk_pkg);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return ;
	}

	/*send turn on/off color mode packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				spk_pkg);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return ;
	}
	PSB_DEBUG_ENTRY("Turn  color mode %s successful.\n",
			(on ? "on" : "off"));
	return;
}

void mdfld_dsi_dpi_turn_on(struct mdfld_dsi_dpi_output * output, int pipe)
{
	struct drm_device * dev = output->dev;
	/* struct drm_psb_private * dev_priv = dev->dev_private; */
	u32 reg_offset = 0;
	
	PSB_DEBUG_ENTRY("pipe %d panel state %d\n", pipe, output->panel_on);
	
	if(output->panel_on) 
		return;
		
	if(pipe) 
		reg_offset = MIPIC_REG_OFFSET;

	/* clear special packet sent bit */
	if(REG_READ(MIPIA_INTR_STAT_REG + reg_offset) & DSI_INTR_STATE_SPL_PKG_SENT) {
		REG_WRITE((MIPIA_INTR_STAT_REG + reg_offset), DSI_INTR_STATE_SPL_PKG_SENT);
	}
		
	/*send turn on package*/
	REG_WRITE((MIPIA_DPI_CONTROL_REG + reg_offset), DSI_DPI_CTRL_HS_TURN_ON);
	
	/*wait for SPL_PKG_SENT interrupt*/
	mdfld_wait_for_SPL_PKG_SENT(dev, pipe);
	
	if(REG_READ(MIPIA_INTR_STAT_REG + reg_offset) & DSI_INTR_STATE_SPL_PKG_SENT) {
		REG_WRITE((MIPIA_INTR_STAT_REG + reg_offset), DSI_INTR_STATE_SPL_PKG_SENT);
	}

	output->panel_on = 1;

	/* FIXME the following is disabled to WA the X slow start issue for TMD panel */
	/* if(pipe == 2) */
	/* 	dev_priv->dpi_panel_on2 = true; */
	/* else if (pipe == 0) */
	/* 	dev_priv->dpi_panel_on = true; */
}

static int __dpi_enter_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	if (ctx->device_ready & DSI_POWER_STATE_ULPS_MASK) {
		DRM_ERROR("Broken ULPS states\n");
		return -EINVAL;
	}

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= DSI_POWER_STATE_ULPS_ENTER;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	DRM_INFO("%s: entered ULPS state\n", __func__);
	return 0;
}

static int __dpi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	/*enter ULPS EXIT state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= DSI_POWER_STATE_ULPS_EXIT;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	/*wait for 1ms as spec suggests*/
	mdelay(1);

	/*clear ULPS state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);
	return 0;
}

static void mdfld_dsi_dpi_shut_down(struct mdfld_dsi_dpi_output *output, int pipe)
{
	struct drm_device *dev = output->dev;
	/* struct drm_psb_private * dev_priv = dev->dev_private; */
	u32 reg_offset = 0;

	PSB_DEBUG_ENTRY("pipe %d panel state %d\n", pipe, output->panel_on);

	/*if output is on, or mode setting didn't happen, ignore this*/
	if ((!output->panel_on) || output->first_boot) {
		output->first_boot = 0;
		return;
	}

	if (pipe)
		reg_offset = MIPIC_REG_OFFSET;

	/* Wait for dpi fifo to empty */
	mdfld_wait_for_DPI_CTRL_FIFO(dev, pipe);

	/* Clear the special packet interrupt bit if set */
	if (REG_READ(MIPIA_INTR_STAT_REG + reg_offset) & DSI_INTR_STATE_SPL_PKG_SENT)
		REG_WRITE((MIPIA_INTR_STAT_REG + reg_offset), DSI_INTR_STATE_SPL_PKG_SENT);

	if (REG_READ(MIPIA_DPI_CONTROL_REG + reg_offset) == DSI_DPI_CTRL_HS_SHUTDOWN) {
		PSB_DEBUG_ENTRY("try to send the same package again, abort!");
		goto shutdown_out;
	}

	REG_WRITE((MIPIA_DPI_CONTROL_REG + reg_offset), DSI_DPI_CTRL_HS_SHUTDOWN);

shutdown_out:
	output->panel_on = 0;
	output->first_boot = 0;

	/* FIXME the following is disabled to WA the X slow start issue for TMD panel */
	/* if(pipe == 2) */
	/*	dev_priv->dpi_panel_on2 = false; */
	/* else if (pipe == 0) */
	/*	dev_priv->dpi_panel_on = false;	 */
}

/**
 * Power on sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_on(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int retry, reset_count = 10;
	int i;
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;
reset_recovery:
	--reset_count;
	/*HW-Reset*/
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config, RESET_FROM_OSPM_RESUME);

	/*Enable DSI PLL*/
	if (!(REG_READ(regs->dpll_reg) & BIT31)) {
		if(ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if(ctx->cck_div)
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
		} else {
			REG_WRITE(regs->dpll_reg, 0x0);
			REG_WRITE(regs->fp_reg, 0x0);
			REG_WRITE(regs->fp_reg, ctx->fp);
			REG_WRITE(regs->dpll_reg, ((ctx->dpll) & ~BIT30));

			/* FIXME WA for CTP PO */
			#if 0
			if (IS_CTP(dev)) {
				REG_WRITE(regs->fp_reg, 0x179);
				REG_WRITE(regs->dpll_reg, 0x100000);
#ifdef CONFIG_SUPPORT_MIPI_H8C7_DISPLAY
				REG_WRITE(regs->fp_reg, 0x100C1);
				REG_WRITE(regs->dpll_reg, 0x800000);
#endif
			}
			#endif

			udelay(2);
			val = REG_READ(regs->dpll_reg);
			REG_WRITE(regs->dpll_reg, (val | BIT31));

			/*wait for PLL lock on pipe*/
			retry = 10000;
			while (--retry && !(REG_READ(PIPEACONF) & BIT29))
				udelay(3);
			if (!retry) {
				DRM_ERROR("PLL failed to lock on pipe\n");
				err = -EAGAIN;
				goto power_on_err;
			}
		}
	}

	/*D-PHY parameter*/
	REG_WRITE(regs->dphy_param_reg, ctx->dphy_param);

	/*Configure DSI controller*/
	REG_WRITE(regs->mipi_control_reg, ctx->mipi_control);
	REG_WRITE(regs->intr_en_reg, ctx->intr_en);
	REG_WRITE(regs->hs_tx_timeout_reg, ctx->hs_tx_timeout);
	REG_WRITE(regs->lp_rx_timeout_reg, ctx->lp_rx_timeout);
	REG_WRITE(regs->turn_around_timeout_reg,
		ctx->turn_around_timeout);
	REG_WRITE(regs->device_reset_timer_reg,
		ctx->device_reset_timer);
	REG_WRITE(regs->high_low_switch_count_reg,
		ctx->high_low_switch_count);
	REG_WRITE(regs->init_count_reg, ctx->init_count);
	REG_WRITE(regs->eot_disable_reg, ctx->eot_disable);
	REG_WRITE(regs->lp_byteclk_reg, ctx->lp_byteclk);
	REG_WRITE(regs->clk_lane_switch_time_cnt_reg,
		ctx->clk_lane_switch_time_cnt);
	REG_WRITE(regs->video_mode_format_reg, ctx->video_mode_format);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);

	/*DSI timing*/
	REG_WRITE(regs->dpi_resolution_reg, ctx->dpi_resolution);
	REG_WRITE(regs->hsync_count_reg, ctx->hsync_count);
	REG_WRITE(regs->hbp_count_reg, ctx->hbp_count);
	REG_WRITE(regs->hfp_count_reg, ctx->hfp_count);
	REG_WRITE(regs->hactive_count_reg, ctx->hactive_count);
	REG_WRITE(regs->vsync_count_reg, ctx->vsync_count);
	REG_WRITE(regs->vbp_count_reg, ctx->vbp_count);
	REG_WRITE(regs->vfp_count_reg, ctx->vfp_count);

	/*Setup pipe timing*/
	REG_WRITE(regs->htotal_reg, ctx->htotal);
	REG_WRITE(regs->hblank_reg, ctx->hblank);
	REG_WRITE(regs->hsync_reg, ctx->hsync);
	REG_WRITE(regs->vtotal_reg, ctx->vtotal);
	REG_WRITE(regs->vblank_reg, ctx->vblank);
	REG_WRITE(regs->vsync_reg, ctx->vsync);
	REG_WRITE(regs->pipesrc_reg, ctx->pipesrc);

	REG_WRITE(regs->dsppos_reg, ctx->dsppos);
	REG_WRITE(regs->dspstride_reg, ctx->dspstride);

	/*Setup plane*/
	REG_WRITE(regs->dspsize_reg, ctx->dspsize);
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	REG_WRITE(regs->vgacntr_reg, ctx->vgacntr);

	/*restore color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		REG_WRITE(regs->color_coef_reg + (i<<2), ctx->color_coef[i]);
	}

	/* restore palette (gamma) */
	for (i = 0; i < 256; i++)
		REG_WRITE(regs->palette_reg + (i<<2), ctx->palette[i]);

	/*exit ULPS state*/
	__dpi_exit_ulps_locked(dsi_config);

	/*Enable DSI Controller*/
	REG_WRITE(regs->device_ready_reg, ctx->device_ready | BIT0);

	/*set low power output hold*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT16));

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init) {
		if (p_funcs->drv_ic_init(dsi_config, 0)) {
			if (!reset_count) {
				err = -EAGAIN;
				goto power_on_err;
			}
			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_DOWN, OSPM_REG_TYPE);

			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_UP, OSPM_REG_TYPE);

			DRM_ERROR("Failed to init dsi controller, reset it!\n");
			goto reset_recovery;
		}
	}

	/**
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */
	if (p_funcs && p_funcs->power_on)
		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}

	/*Enable MIPI Port*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT31));

	/*Enable pipe*/
	val = ctx->pipeconf;
	val &= ~0x000c0000;
	val |= BIT31;

	/* disable gamma if needed */
	if (drm_psb_enable_color_conversion == 0)
		 val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE);


	REG_WRITE(regs->pipeconf_reg, val);
	REG_WRITE(regs->pipestat_reg, ctx->pipestat |
			PIPE_VBLANK_INTERRUPT_ENABLE);

	/*Wait for pipe enabling,when timing generator
	is wroking */
	if (REG_READ(regs->mipi_reg) & BIT31) {
		retry = 10000;
		while (--retry && !(REG_READ(regs->pipeconf_reg) & BIT30))
			udelay(3);

		if (!retry) {
			DRM_ERROR("Failed to enable pipe\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}
	/*enable plane*/
	val = ctx->dspcntr | BIT31;

	/* disable gamma if needed */
	if (drm_psb_enable_gamma == 0)
		 val &= ~(PIPEACONF_GAMMA);

	REG_WRITE(regs->dspcntr_reg, val);

	if (p_funcs->set_brightness(dsi_config, ctx->lastbrightnesslevel))
		DRM_ERROR("Failed to set panel brightness\n");

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power off sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_off(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	u32 tmp = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	int retry;
	int i;
	int pipe0_enabled;
	int pipe2_enabled;
	int err = 0;
	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs->set_brightness(dsi_config, 0))
		DRM_ERROR("Failed to set panel brightness\n");

	/*save the plane informaton, for it will updated*/
	ctx->dspsurf = dev_priv->init_screen_start;
	ctx->dsplinoff = dev_priv->init_screen_offset;
	ctx->pipestat = REG_READ(regs->pipestat_reg);
	ctx->dspcntr = REG_READ(regs->dspcntr_reg);
	ctx->dspstride= REG_READ(regs->dspstride_reg);

	tmp = REG_READ(regs->pipeconf_reg);

	/*save color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		ctx->color_coef[i] = REG_READ(regs->color_coef_reg + (i<<2));
	}

	/* save palette (gamma) */
	for (i = 0; i < 256; i++)
		ctx->palette[i] = REG_READ(regs->palette_reg + (i<<2));

	/*
	 * Couldn't disable the pipe until DRM_WAIT_ON signaled by last
	 * vblank event when playing video, otherwise the last vblank event
	 * will lost when pipe disabled before vblank interrupt coming sometimes.
	 */

	/*Disable panel*/
	val = ctx->dspcntr;
	val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE | PIPEACONF_GAMMA);
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));
	/*Disable overlay & cursor panel assigned to this pipe*/
	REG_WRITE(regs->pipeconf_reg, (tmp | (0x000c0000)));

	/*Disable pipe*/
	val = REG_READ(regs->pipeconf_reg);
	ctx->pipeconf = val;
	REG_WRITE(regs->pipeconf_reg, (val & ~BIT31));

	/*wait for pipe disabling,
	pipe synchronization plus , only avaiable when
	timer generator is working*/
	if (REG_READ(regs->mipi_reg) & BIT31) {
		retry = 100000;
		while (--retry && (REG_READ(regs->pipeconf_reg) & BIT30))
			udelay(5);

		if (!retry) {
			DRM_ERROR("Failed to disable pipe\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}
	/*Disable MIPI port*/
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT31));

	/**
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

	/*clear Low power output hold*/
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT16));

	/*Disable DSI controller*/
	REG_WRITE(regs->device_ready_reg, (ctx->device_ready & ~BIT0));

	/*enter ULPS*/
	__dpi_enter_ulps_locked(dsi_config);

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg , 0x0);
		/*power gate pll*/
		REG_WRITE(regs->dpll_reg, BIT30);
	}

power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Send TURN_ON package to dpi panel to turn it on
 */
static int mdfld_dsi_dpi_panel_turn_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
	/*According HW DSI spec, here need wait for 100ms*/
	msleep(100);

	return err;
}

/**
 * Send SHUT_DOWN package to dpi panel to turn if off
 */
static int mdfld_dsi_dpi_panel_shut_down(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	/*According HW DSI spec, here need wait for 100ms*/
	msleep(100);

	return err;
}

/**
 * Setup Display Controller to turn on/off a video mode panel.
 * Most of the video mode MIPI panel should follow the power on/off
 * sequence in this function.
 * NOTE: do NOT modify this function for new panel Enabling. Register
 * new panel function callbacks to make this function available for a
 * new video mode panel
 */
static int __mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_connector *dsi_connector;
	struct mdfld_dsi_dpi_output *dpi_output;
	struct mdfld_dsi_config *dsi_config;
	struct panel_funcs *p_funcs;
	int pipe;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	static int last_ospm_suspend = -1;

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("%s, last_ospm_suspend = %s\n", (on ? "on" : "off"),
			(last_ospm_suspend ? "true" : "false"));

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	p_funcs = dpi_output->p_funcs;
	pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_encoder);
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (dsi_connector->status != connector_status_connected)
		return 0;

	mutex_lock(&dsi_config->context_lock);

	if (last_ospm_suspend == -1)
		last_ospm_suspend = false;

	if (dpi_output->first_boot && dsi_config->dsi_hw_context.panel_on) {
		printk(KERN_ALERT "skip panle power setting for first boot!"
				" panel is already powered on\n");
		goto fun_exit;
	}

	/**
	 * if ospm has turned panel off, but dpms tries to turn panel on, skip
	 */
	if (dev_priv->dpms_on_off && on && last_ospm_suspend)
		goto fun_exit;

	switch (on) {
	case true:
		/* panel is already on */
		if (dsi_config->dsi_hw_context.panel_on)
			goto fun_exit;
		/* For DPMS case, just turn on/off panel */
		if (dev_priv->dpms_on_off) {
			if (mdfld_dsi_dpi_panel_turn_on(dsi_config)) {
				DRM_ERROR("Faild to turn on panel\n");
				goto set_power_err;
			}
		} else {
			if (__dpi_panel_power_on(dsi_config, p_funcs)) {
				DRM_ERROR("Faild to turn on panel\n");
				goto set_power_err;
			}
		}
		/**
		 * If power on, turn off color mode by default,
		 * let panel in full color mode
		 */
		mdfld_dsi_dpi_set_color_mode(dsi_config, false);

		dsi_config->dsi_hw_context.panel_on = 1;
		last_ospm_suspend = false;
		break;
	case false:
		if (dev_priv->dpms_on_off &&
				dsi_config->dsi_hw_context.panel_on) {
			if (mdfld_dsi_dpi_panel_shut_down(dsi_config))
				DRM_ERROR("Faild to shutdown panel\n");

			last_ospm_suspend = false;
		} else if (!dev_priv->dpms_on_off && !last_ospm_suspend) {
			if (__dpi_panel_power_off(dsi_config, p_funcs)) {
				DRM_ERROR("Faild to turn off panel\n");
				goto set_power_err;
			}
			/* ospm suspend called? */
			last_ospm_suspend = true;
		}
		dsi_config->dsi_hw_context.panel_on = 0;
		break;
	default:
		break;
	}

fun_exit:
	mutex_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("successfully\n");
	return 0;
set_power_err:
	mutex_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("unsuccessfully!!!!\n");
	return -EAGAIN;
}

void mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	u32 mipi_reg = MIPI;
	u32 pipeconf_reg = PIPEACONF;

	PSB_DEBUG_ENTRY("set power %s on pipe %d\n", on ? "On" : "Off", pipe);

	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);

	if (pipe)
		if (!(dev_priv->panel_desc & DISPLAY_B) ||
				!(dev_priv->panel_desc & DISPLAY_C))
			return;

	if (pipe) {
		mipi_reg = MIPI_C;
		pipeconf_reg = PIPECCONF;
	}

	/*start up display island if it was shutdown*/
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return;

	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_power(encoder, on);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

void mdfld_dsi_dpi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_config *dsi_config;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY(
			"%s\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));
	if (!gbdispstatus) {
		PSB_DEBUG_ENTRY(
		"panel in suspend status, skip turn on/off from DMPS");
		return ;
	}

	mutex_lock(&dev_priv->dpms_mutex);
	dev_priv->dpms_on_off = true;
	if (mode == DRM_MODE_DPMS_ON)
		mdfld_dsi_dpi_set_power(encoder, true);
	else
		mdfld_dsi_dpi_set_power(encoder, false);
	dev_priv->dpms_on_off = false;
	mutex_unlock(&dev_priv->dpms_mutex);
}

bool mdfld_dsi_dpi_mode_fixup(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_display_mode *fixed_mode = dsi_config->fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if(fixed_mode) {
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
	}
	
	return true;
}

void mdfld_dsi_dpi_prepare(struct drm_encoder *encoder)
{
	PSB_DEBUG_ENTRY("\n");
}

void mdfld_dsi_dpi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dpi_output *dpi_output;
	struct mdfld_dsi_hw_context *ctx;
	struct mdfld_dsi_config *dsi_config;
	struct drm_device *dev;
	u32 temp_val = 0;

	PSB_DEBUG_ENTRY("\n");

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dev = dsi_config->dev;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;
	temp_val = REG_READ(PIPEACONF);
	temp_val &= ~(BIT27 | BIT28);
	/* Setup pipe configuration for different panels*/
	REG_WRITE(PIPEACONF, temp_val | (ctx->pipeconf & (BIT27 | BIT28)));
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	/*Everything is ready, commit DSI hw context to HW*/
	__mdfld_dsi_dpi_set_power(encoder, true);
	dpi_output->first_boot = 0;
}

void dsi_debug_MIPI_reg(struct drm_device *dev)
{
	u32 temp_val = 0;

	PSB_DEBUG_ENTRY("[DISPLAY] Enter %s\n", __func__);
	temp_val = REG_READ(MIPI);
	PSB_DEBUG_ENTRY("[DISPLAY] MIPI = %x\n", temp_val);

	/* set the lane speed */
	temp_val = REG_READ(MIPI_CONTROL_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] MIPI_CONTROL_REG = %x\n", temp_val);

	/* Enable all the error interrupt */
	temp_val = REG_READ(INTR_EN_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] INTR_EN_REG = %x\n", temp_val);
	temp_val = REG_READ(TURN_AROUND_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] TURN_AROUND_TIMEOUT_REG = %x\n", temp_val);
	temp_val = REG_READ(DEVICE_RESET_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DEVICE_RESET_REG = %x\n", temp_val);
	temp_val = REG_READ(INIT_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] INIT_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(DSI_FUNC_PRG_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DSI_FUNC_PRG_REG = %x\n", temp_val);

	temp_val = REG_READ(DPI_RESOLUTION_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPI_RESOLUTION_REG = %x\n", temp_val);

	temp_val = REG_READ(VERT_SYNC_PAD_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_SYNC_PAD_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(VERT_BACK_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_BACK_PORCH_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(VERT_FRONT_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_FRONT_PORCH_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(HORIZ_SYNC_PAD_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_SYNC_PAD_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(HORIZ_BACK_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_BACK_PORCH_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(HORIZ_FRONT_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_FRONT_PORCH_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(HORIZ_ACTIVE_AREA_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_ACTIVE_AREA_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(VIDEO_FMT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VIDEO_FMT_REG = %x\n", temp_val);

	temp_val = REG_READ(HS_TX_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HS_TX_TIMEOUT_REG = %x\n", temp_val);
	temp_val = REG_READ(LP_RX_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] LP_RX_TIMEOUT_REG = %x\n", temp_val);

	temp_val = REG_READ(HIGH_LOW_SWITCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HIGH_LOW_SWITCH_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(EOT_DISABLE_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] EOT_DISABLE_REG = %x\n", temp_val);

	temp_val = REG_READ(LP_BYTECLK_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] LP_BYTECLK_REG = %x\n", temp_val);
	temp_val = REG_READ(MAX_RET_PAK_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] MAX_RET_PAK_REG = %x\n", temp_val);
	temp_val = REG_READ(DPI_CONTROL_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPI_CONTROL_REG = %x\n", temp_val);
	temp_val = REG_READ(DPHY_PARAM_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPHY_PARAM_REG = %x\n", temp_val);
/*	temp_val = REG_READ(PIPEACONF);
	PSB_DEBUG_ENTRY("[DISPLAY] PIPEACONF = %x\n", temp_val);
	temp_val = REG_READ(DSPACNTR);
	PSB_DEBUG_ENTRY("[DISPLAY] DSPACNTR = %x\n", temp_val);
*/
}

/**
 * Setup DPI timing for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static void __mdfld_dsi_dpi_set_timing(struct mdfld_dsi_config *config,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct mdfld_dsi_hw_context *ctx;

	if (!config) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	mode = adjusted_mode;
	ctx = &config->dsi_hw_context;

	mutex_lock(&config->context_lock);

	/*dpi resolution*/
	ctx->dpi_resolution = (mode->vdisplay << 16 | mode->hdisplay);

	/*Calculate DPI timing*/
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					config->lane_count,
					config->bpp);

	/*update HW context with new DPI timings*/
	ctx->hsync_count = dpi_timing.hsync_count;
	ctx->hbp_count = dpi_timing.hbp_count;
	ctx->hfp_count = dpi_timing.hfp_count;
	ctx->hactive_count = dpi_timing.hactive_count;
	ctx->vsync_count = dpi_timing.vsync_count;
	ctx->vbp_count = dpi_timing.vbp_count;
	ctx->vfp_count = dpi_timing.vfp_count;

	/*setup mipi port configuration*/
	if (config->pipe == 0)
		ctx->mipi = PASS_FROM_SPHY_TO_AFE | config->lane_config;

	mutex_unlock(&config->context_lock);
}


void mdfld_dsi_dpi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dpi_output *dpi_output =
			MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
	mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);

	u32 pipeconf_reg = PIPEACONF;
	u32 dspcntr_reg = DSPACNTR;
	u32 mipi_reg = MIPI;
	u32 reg_offset = 0;
	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	u32 mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE;
	int retry = 0;

	/* The bit defination changed from PNW_A0 -> B0 and forward,
	* Only for PNW_A0 that we need to set FLOPPED_HSTX
	* */
	if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
		mipi |= SEL_FLOPPED_HSTX;

	PSB_DEBUG_ENTRY("set mode %dx%d on pipe %d",
			mode->hdisplay, mode->vdisplay, pipe);

	if (pipe) {
		pipeconf_reg = PIPECCONF;
		dspcntr_reg = DSPCCNTR;
		mipi_reg = MIPI_C;
		reg_offset = MIPIC_REG_OFFSET;
	} else {
		mipi |= 2;
	}

	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_timing(dsi_config, mode, adjusted_mode);
}

void mdfld_dsi_dpi_save(struct drm_encoder *encoder)
{
	printk(KERN_ALERT"%s\n", __func__);

	if (!encoder)
		return;

	/*turn off*/
	__mdfld_dsi_dpi_set_power(encoder, false);
}

void mdfld_dsi_dpi_restore(struct drm_encoder *encoder)
{
	printk(KERN_ALERT"%s\n", __func__);

	if (!encoder)
		return;

	/*turn on*/
	__mdfld_dsi_dpi_set_power(encoder, true);
}

/**
 * Exit from DSR
 */
void mdfld_dsi_dpi_exit_idle(struct drm_device *dev,
				u32 update_src,
				void *p_surfaceAddr,
				bool check_hw_on_only)
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	unsigned long irqflags;

	/* PSB_DEBUG_ENTRY("\n"); */

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
		OSPM_UHB_ONLY_IF_ON)) {
		DRM_ERROR("display island is in off state\n");
		return;
	}

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
	if (dev_priv->b_is_in_idle) {
		/* update the surface base address. */
		if (p_surfaceAddr) {
			REG_WRITE(DSPASURF, *((u32 *)p_surfaceAddr));
#if defined(CONFIG_MDFD_DUAL_MIPI)
			REG_WRITE(DSPCSURF, *((u32 *)p_surfaceAddr));
#endif
		}

		mid_enable_pipe_event(dev_priv, 0);
		psb_enable_pipestat(dev_priv, 0, PIPE_VBLANK_INTERRUPT_ENABLE);
		dev_priv->b_is_in_idle = false;
		dev_priv->dsr_idle_count = 0;
	}
	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/*
 * Init DSI DPI encoder. 
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DPI encoder, NULL on error
 */ 
struct mdfld_dsi_encoder *mdfld_dsi_dpi_init(struct drm_device *dev,
				struct mdfld_dsi_connector *dsi_connector,
				struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_display_mode *fixed_mode = NULL;
	int pipe;
	int ret;

	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);
 
	if(!dsi_connector || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}
	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;

	/*panel hard-reset*/
	if (p_funcs->reset) {
		ret = p_funcs->reset(dsi_config, RESET_FROM_BOOT_UP);
		if (ret) {
			DRM_ERROR("Panel %d hard-reset failed\n", pipe);
			return NULL;
		}
	}

	/*detect panel connection stauts*/
	if (p_funcs->detect) {
		ret = p_funcs->detect(dsi_config, pipe);
		if (ret) {
			DRM_INFO("Detecting Panel %d, Not connected\n", pipe);
			dsi_connector->status = connector_status_disconnected;
		} else {
			PSB_DEBUG_ENTRY("Panel %d is connected\n", pipe);
			dsi_connector->status = connector_status_connected;
		}

		if (dsi_connector->status == connector_status_disconnected &&
			pipe == 0) {
			DRM_ERROR("Primary panel disconnected\n");
			return NULL;
		}
	} else {
		/*use the default config*/
		if (pipe == 0)
			dsi_connector->status = connector_status_connected;
		else
			dsi_connector->status = connector_status_disconnected;
	}

	/*init DSI controller*/
	if (p_funcs->dsi_controller_init)
		p_funcs->dsi_controller_init(dsi_config, pipe, 0);

	/**
	 * TODO: can we keep these code out of display driver as
	 * it will make display driver hard to be maintained
	 */
	if (dsi_connector->status == connector_status_connected) {
		if (pipe == 0)
			dev_priv->panel_desc |= DISPLAY_A;
		if (pipe == 2)
			dev_priv->panel_desc |= DISPLAY_C;
	}

	dpi_output = kzalloc(sizeof(struct mdfld_dsi_dpi_output), GFP_KERNEL);
	if (!dpi_output) {
		DRM_ERROR("No memory\n");
		return NULL;
	}

	dpi_output->dev = dev;
	dpi_output->p_funcs = p_funcs;
	dpi_output->first_boot = 1;

	/*get fixed mode*/
	fixed_mode = dsi_config->fixed_mode;

	/*detect power status of the connected panel*/
	if (p_funcs->get_panel_power_state) {
		ret = p_funcs->get_panel_power_state(dsi_config, pipe);
		if (ret == MDFLD_DSI_PANEL_POWER_OFF)
			dsi_config->dsi_hw_context.panel_on = 0;
		else
			dsi_config->dsi_hw_context.panel_on = 1;
	} else {
		/*use the default config*/
		if (pipe == 0)
			dsi_config->dsi_hw_context.panel_on = 1;
		else
			dsi_config->dsi_hw_context.panel_on = 0;
	}

	/*create drm encoder object*/
	connector = &dsi_connector->base.base;
	encoder = &dpi_output->base.base;
	drm_encoder_init(dev,
			encoder,
			p_funcs->encoder_funcs,
			DRM_MODE_ENCODER_MIPI);
	drm_encoder_helper_add(encoder,
				p_funcs->encoder_helper_funcs);
	
	/*attach to given connector*/
	drm_mode_connector_attach_encoder(connector, encoder);
	
	/*set possible crtcs and clones*/
	if(dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dpi_exit_idle;
#if defined(CONFIG_MDFLD_DSI_DPU) || defined(CONFIG_MDFLD_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif /*CONFIG_MDFLD_DSI_DSR*/


	PSB_DEBUG_ENTRY("successfully\n");

	return &dpi_output->base;
}
