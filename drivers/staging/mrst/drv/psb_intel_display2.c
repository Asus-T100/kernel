/*
 * Copyright Â© 2006-2007 Intel Corporation
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
 *	Eric Anholt <eric@anholt.net>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_csc.h"
//#include "mdfld_dsi_output.h"
#ifdef CONFIG_MDFLD_DSI_DPU
#include "mdfld_dsi_dbi_dpu.h"
#endif

#include "drmlfb.h"
#include <linux/pm_runtime.h>

#include "psb_intel_display.h"

#ifdef MIN
#undef MIN
#endif

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/* MDFLD_PLATFORM start */
void mdfldWaitForPipeDisable(struct drm_device *dev, int pipe)
{
	int count, temp;
	u32 pipeconf_reg = PIPEACONF;

	switch (pipe) {
	case 0:
		break;
	case 1:
		pipeconf_reg = PIPEBCONF;
		break;
	case 2:
		pipeconf_reg = PIPECCONF;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return;
	}

	/* Wait for for the pipe disable to take effect. */
	for (count = 0; count < COUNT_MAX; count++) {
		temp = REG_READ(pipeconf_reg);
		if (!(temp & PIPEACONF_PIPE_STATE))
			break;

		udelay(20);
	}

	PSB_DEBUG_ENTRY("cout = %d. \n", count);
}

void mdfldWaitForPipeEnable(struct drm_device *dev, int pipe)
{
	int count, temp;
	u32 pipeconf_reg = PIPEACONF;

	switch (pipe) {
	case 0:
		break;
	case 1:
		pipeconf_reg = PIPEBCONF;
		break;
	case 2:
		pipeconf_reg = PIPECCONF;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return;
	}

	/* Wait for for the pipe enable to take effect. */
	for (count = 0; count < COUNT_MAX; count++) {
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_PIPE_STATE))
			break;

		udelay(20);
	}

	PSB_DEBUG_ENTRY("cout = %d. \n", count);
}

/*
 * set display controller side palette, it will influence
 * brightness , saturation , contrast.
 * KAI1
*/

int mdfld_intel_crtc_set_gamma(struct drm_device *dev,
				struct gamma_setting *setting_data)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_config *dsi_config = NULL;
	int ret = 0;
	int pipe = 0;
	u32 val = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dev || !setting_data) {
		ret = -EINVAL;
		return ret;
	}

	if (!(setting_data->type &
		(GAMMA_SETTING|GAMMA_INITIA))) {
		ret = -EINVAL;
		return ret;
	}
	if (setting_data->type == GAMMA_SETTING &&
		setting_data->data_len != GAMMA_10_BIT_TABLE_COUNT) {
		ret = -EINVAL;
		return ret;
	}

	dev_priv = dev->dev_private;
	pipe = setting_data->pipe;

	if (pipe == 0)
		dsi_config = dev_priv->dsi_configs[0];
	else if (pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];
	else if (pipe == 1) {
		PSB_DEBUG_ENTRY("/KAI1 palette no implement for HDMI\n"
				"do it later\n");
		return -EINVAL;
	} else
		return -EINVAL;

	mutex_lock(&dev_priv->gamma_csc_lock);

	ctx = &dsi_config->dsi_hw_context;
	regs = &dsi_config->regs;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
		ret = -EAGAIN;
		goto _fun_exit;
	}

	/*enable gamma*/
	if (drm_psb_enable_gamma && setting_data->enable_state) {
		int i = 0, temp = 0;
		u32 integer_part = 0, fraction_part = 0, even_part = 0, odd_part = 0;
		u32 int_red_9_2 = 0, int_green_9_2 = 0, int_blue_9_2 = 0;
		u32 int_red_1_0 = 0, int_green_1_0 = 0, int_blue_1_0 = 0;
		u32 fra_red = 0, fra_green = 0, fra_blue = 0;
		int j = 0;
		/*here set r/g/b the same curve*/
		for (i = 0; i <= 1024; i = i + 8) {
			if (setting_data->type == GAMMA_INITIA) {
				switch (setting_data->initia_mode) {
				case GAMMA_05:
					/* gamma 0.5 */
					temp = 32 * int_sqrt(i *  10000);
					printk(KERN_ALERT "gamma 0.5\n");
					break;
				case GAMMA_20:
					/* gamma 2 */
					temp = (i * i * 100) / 1024;
					printk(KERN_ALERT "gamma 2\n");
					break;
				case GAMMA_05_20:
					/* 0 ~ 511 gamma0.5    512 ~1024 gamma 2 */
					if (i < 512)
						temp =  int_sqrt(i * 512 * 10000);
					else
						temp = ((i - 512) * (i - 512) * 100 / 512  + 512 * 100);
					printk(KERN_ALERT "gamma 0.5  + gamma2 \n");
					break;
				case GAMMA_20_05:
					/* 0 ~ 511 gamma2    512 ~1024 gamma 0.5 */
					if (i < 512)
						temp = i * i * 100 / 512;
					else
						temp = int_sqrt((i - 512) * 512 * 10000) + 512 * 100;
					printk(KERN_ALERT "gamma 2  + gamma0.5 \n");
					break;
				case GAMMA_10:
					/* gamma 1 */
					temp = i * 100;
					printk(KERN_ALERT "gamma 1\n");
					break;
				default:
					/* gamma 0.5 */
					temp = 32 * int_sqrt(i *  10000);
					printk(KERN_ALERT "gamma 0.5\n");
					break;
				}
			} else {
				temp = setting_data->gamma_tableX100[i / 8];
			}

			if (temp < 0)
				temp = 0;
			if (temp > 1024 * 100)
				temp = 1024 * 100;

			/* printk(KERN_ALERT "index = %d, temp= 0x%d\n",i, temp); */
			integer_part = temp / 100;
			fraction_part = (temp - integer_part * 100);
			/* printk(KERN_ALERT "index = %d, inte= 0x%x frai= 0x%x\n",i, integer_part, fraction_part); */
			/*get r/g/b each channel*/
			int_blue_9_2 = integer_part >> 2;
			int_green_9_2 = int_blue_9_2 << 8;
			int_red_9_2 = int_blue_9_2 << 16;
			/* printk(KERN_ALERT "blue9_2 = 0x%x green9_2= 0x%x \n",int_blue_9_2,int_green_9_2); */
			int_blue_1_0 = (integer_part & 0x3) << 6;
			int_green_1_0 = int_blue_1_0 << 8;
			int_red_1_0 = int_blue_1_0 << 16;
			fra_blue = fraction_part*64/100;
			fra_green = fra_blue << 8;
			fra_red = fra_blue << 16;
			/* printk(KERN_ALERT "fra_blue = 0x%x fra_green= 0x%x \n",fra_blue, fra_green); */
			/*get even and odd part*/
			odd_part = int_red_9_2 | int_green_9_2 | int_blue_9_2;
			even_part = int_red_1_0 | fra_red | int_green_1_0 | fra_green|
				int_blue_1_0 | fra_blue;
			/* printk(KERN_ALERT "even = 0x%x odd= 0x%x \n",even_part, odd_part); */
			if (i != 1024) {
				REG_WRITE(regs->palette_reg + j, even_part);
				/* printk(KERN_ALERT "offset = 0x%x read even= 0x%x \n",(regs->palette_reg +j), REG_READ(regs->palette_reg +j)); */
				REG_WRITE(regs->palette_reg + j + 4, odd_part);
				/* printk(KERN_ALERT "offset = 0x%x read odd = 0x%x \n",(regs->palette_reg + j+ 4), REG_READ(regs->palette_reg + j+ 4)); */
			} else {
				REG_WRITE(regs->gamma_red_max_reg, (integer_part << 6) | (fraction_part));
				REG_WRITE(regs->gamma_green_max_reg, (integer_part << 6) | (fraction_part));
				REG_WRITE(regs->gamma_blue_max_reg, (integer_part << 6) | (fraction_part));
				printk(KERN_ALERT "max read%x max green 0x%x max blue 0x%x \n",
						REG_READ(regs->gamma_red_max_reg),
						REG_READ(regs->gamma_green_max_reg),
						REG_READ(regs->gamma_blue_max_reg)
				      );
			}

			j = j + 8;
		}
		/*enable*/
		val = REG_READ(regs->pipeconf_reg);
		val |= (PIPEACONF_GAMMA);
		REG_WRITE(regs->pipeconf_reg, val);
		REG_WRITE(regs->dspcntr_reg, REG_READ(regs->dspcntr_reg)|DISPPLANE_GAMMA_ENABLE);
		REG_READ(regs->dspcntr_reg);
	} else {
		drm_psb_enable_gamma = 0;
		/*disable */
		val = REG_READ(regs->pipeconf_reg);
		val &= ~(PIPEACONF_GAMMA);
		REG_WRITE(regs->pipeconf_reg, val);
		REG_WRITE(regs->dspcntr_reg, REG_READ(regs->dspcntr_reg) & ~(DISPPLANE_GAMMA_ENABLE));
		REG_READ(regs->dspcntr_reg);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

_fun_exit:
	mutex_unlock(&dev_priv->gamma_csc_lock);
	return ret;
}
/*
 * set display controller side color conversion
 * KAI1
*/
int mdfld_intel_crtc_set_color_conversion(struct drm_device *dev,
					struct csc_setting *setting_data)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_config *dsi_config = NULL;
	int ret = 0;
	int i = 0;
	int pipe = 0;
	u32 val = 0;
	/*Rx, Ry, Gx, Gy, Bx, By, Wx, Wy*/
	/*sRGB color space*/
	uint32_t chrom_input[8] = {	6400, 3300,
		3000, 6000,
		1500, 600,
		3127, 3290 };
	/* PR3 color space*/
	uint32_t chrom_output[8] = { 6382, 3361,
		2979, 6193,
		1448, 478,
		3000, 3236 };

	PSB_DEBUG_ENTRY("\n");

	if (!dev) {
		ret = -EINVAL;
		return ret;
	}

	if (!(setting_data->type &
		(CSC_CHROME_SETTING | CSC_INITIA | CSC_SETTING))) {
		ret = -EINVAL;
		return ret;
	}
	if ((setting_data->type == CSC_SETTING &&
		setting_data->data_len != CSC_COUNT) ||
		(setting_data->type == CSC_CHROME_SETTING &&
		setting_data->data_len != CHROME_COUNT)) {
		ret = -EINVAL;
		return ret;
	}

	dev_priv = dev->dev_private;
	pipe = setting_data->pipe;

	if (pipe == 0)
		dsi_config = dev_priv->dsi_configs[0];
	else if (pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];
	else if (pipe == 1) {
		PSB_DEBUG_ENTRY("/KAI1 color conversion no implement for HDMI\n"
				"do it later\n");
		return -EINVAL;
	} else
		return -EINVAL;

	mutex_lock(&dev_priv->gamma_csc_lock);

	ctx = &dsi_config->dsi_hw_context;
	regs = &dsi_config->regs;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
		ret = -EAGAIN;
		goto _fun_exit;
	}

	if (drm_psb_enable_color_conversion && setting_data->enable_state) {
		if (setting_data->type == CSC_INITIA) {
			/*initialize*/
			csc(dev, &chrom_input[0], &chrom_output[0], pipe);
		} else if (setting_data->type == CSC_CHROME_SETTING) {
			/*use chrome to caculate csc*/
			memcpy(chrom_input, setting_data->data.chrome_data, 8*sizeof(int));
			memcpy(chrom_output, setting_data->data.chrome_data+8, 8*sizeof(int));
			csc(dev, &chrom_input[0], &chrom_output[0], pipe);
		} else if (setting_data->type == CSC_SETTING) {
			/*use user space csc*/
			csc_program_DC(dev, &setting_data->data.csc_data[0], pipe);
		}

		/*enable*/
		val = REG_READ(regs->pipeconf_reg);
		val |= (PIPEACONF_COLOR_MATRIX_ENABLE);
		REG_WRITE(regs->pipeconf_reg, val);
		val = REG_READ(regs->dspcntr_reg);
		REG_WRITE(regs->dspcntr_reg, val);
	} else {
		drm_psb_enable_color_conversion = 0;
		/*disable*/
		val = REG_READ(regs->pipeconf_reg);
		val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE);
		REG_WRITE(regs->pipeconf_reg, val);
		val = REG_READ(regs->dspcntr_reg);
		REG_WRITE(regs->dspcntr_reg, val);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

_fun_exit:
	mutex_unlock(&dev_priv->gamma_csc_lock);
	return ret;
}

static int mdfld_intel_crtc_cursor_set(struct drm_crtc *crtc,
				 struct drm_file *file_priv,
				 uint32_t handle,
				 uint32_t width, uint32_t height)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt * pg = dev_priv->pg;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	uint32_t control = CURACNTR;
	uint32_t base = CURABASE;
	uint32_t temp;
	size_t addr = 0;
	uint32_t page_offset;
	size_t size;
	void *bo;
	int ret;

	DRM_DEBUG("\n");

	switch (pipe) {
	case 0:
		break;
	case 1:
		control = CURBCNTR;
		base = CURBBASE;
		break;
	case 2:
		control = CURCCNTR;
		base = CURCBASE;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return -EINVAL;
	}

	/* Can't enalbe HW cursor on plane B/C. */
	if (pipe != 0)
		return 0;

	/* if we want to turn of the cursor ignore width and height */
	if (!handle) {
		DRM_DEBUG("cursor off\n");
		/* turn off the cursor */
		temp = 0;
		temp |= CURSOR_MODE_DISABLE;

		if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					      OSPM_UHB_ONLY_IF_ON)) {
			REG_WRITE(control, temp);
			REG_WRITE(base, 0);
			ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		}

		/* unpin the old bo */
		if (psb_intel_crtc->cursor_bo) {
			mode_dev->bo_unpin_for_scanout(dev,
						       psb_intel_crtc->
						       cursor_bo);
			psb_intel_crtc->cursor_bo = NULL;
		}
		return 0;
	}

	/* Currently we only support 64x64 cursors */
	if (width != 64 || height != 64) {
		DRM_ERROR("we currently only support 64x64 cursors\n");
		return -EINVAL;
	}

	bo = mode_dev->bo_from_handle(dev, file_priv, handle);
	if (!bo)
		return -ENOENT;

	ret = mode_dev->bo_pin_for_scanout(dev, bo);
	if (ret)
		return ret;
	size = mode_dev->bo_size(dev, bo);
	if (size < width * height * 4) {
		DRM_ERROR("buffer is to small\n");
		return -ENOMEM;
	}

        /*insert this bo into gtt*/
	ret = psb_gtt_map_meminfo(dev, (IMG_HANDLE)handle, 0, &page_offset);
        if(ret) {
                DRM_ERROR("Can not map meminfo to GTT. handle 0x%x\n", handle);
                return ret;
        }

	addr = page_offset << PAGE_SHIFT;

	if(IS_POULSBO(dev)) {
		addr += pg->stolen_base;
	}

	psb_intel_crtc->cursor_addr = addr;

	temp = 0;
	/* set the pipe for the cursor */
	temp |= (pipe << 28);
	temp |= CURSOR_MODE_64_ARGB_AX | MCURSOR_GAMMA_ENABLE;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE(control, temp);
		REG_WRITE(base, addr);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	/* unpin the old bo */
	if (psb_intel_crtc->cursor_bo && psb_intel_crtc->cursor_bo != bo) {
		mode_dev->bo_unpin_for_scanout(dev, psb_intel_crtc->cursor_bo);
		psb_intel_crtc->cursor_bo = bo;
	}

	return 0;
}

static int mdfld_intel_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct drm_device *dev = crtc->dev;
#ifndef CONFIG_MDFLD_DSI_DPU
	struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
#else
	struct psb_drm_dpu_rect rect;
#endif
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	uint32_t pos = CURAPOS;
	uint32_t base = CURABASE;
	uint32_t temp = 0;
	uint32_t addr;

	switch (pipe) {
	case 0:
#ifndef CONFIG_MDFLD_DSI_DPU
		if (!(dev_priv->dsr_fb_update & MDFLD_DSR_CURSOR_0))
			mdfld_dsi_dbi_exit_dsr (dev, MDFLD_DSR_CURSOR_0, 0, 0);
#else /*CONFIG_MDFLD_DSI_DPU*/
		rect.x = x;
		rect.y = y;

		mdfld_dbi_dpu_report_damage(dev, MDFLD_CURSORA, &rect);
		mdfld_dpu_exit_dsr(dev);
#endif
		break;
	case 1:
		pos = CURBPOS;
		base = CURBBASE;
		break;
	case 2:
#ifndef CONFIG_MDFLD_DSI_DPU
		if (!(dev_priv->dsr_fb_update & MDFLD_DSR_CURSOR_2))
			mdfld_dsi_dbi_exit_dsr (dev, MDFLD_DSR_CURSOR_2, 0, 0);
#else /*CONFIG_MDFLD_DSI_DPU*/
		mdfld_dbi_dpu_report_damage(dev, MDFLD_CURSORC, &rect);
		mdfld_dpu_exit_dsr(dev);
#endif
		pos = CURCPOS;
		base = CURCBASE;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return -EINVAL;
	}

	/* Can't enalbe HW cursor on plane B/C. */
	if (pipe != 0)
		return 0;

	if (x < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_X_SHIFT);
		x = -x;
	}
	if (y < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_Y_SHIFT);
		y = -y;
	}

	temp |= ((x & CURSOR_POS_MASK) << CURSOR_X_SHIFT);
	temp |= ((y & CURSOR_POS_MASK) << CURSOR_Y_SHIFT);

	addr = psb_intel_crtc->cursor_addr;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE(pos, temp);
		REG_WRITE(base, addr);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	return 0;
}

const struct drm_crtc_funcs mdfld_intel_crtc_funcs = {
#ifndef CONFIG_X86_MDFLD
	.save = psb_intel_crtc_save,
	.restore = psb_intel_crtc_restore,
#endif
	.cursor_set = mdfld_intel_crtc_cursor_set,
	.cursor_move = mdfld_intel_crtc_cursor_move,
	.gamma_set = psb_intel_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = psb_intel_crtc_destroy,
};

static struct drm_device globle_dev;

void mdfld__intel_plane_set_alpha(int enable)
{
	struct drm_device *dev = &globle_dev;
	int dspcntr_reg = DSPACNTR;
	u32 dspcntr;

	dspcntr = REG_READ(dspcntr_reg);

	if (enable) {
		dspcntr &= ~DISPPLANE_32BPP_NO_ALPHA;
		dspcntr |= DISPPLANE_32BPP;
	} else {
		dspcntr &= ~DISPPLANE_32BPP;
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
	}

	REG_WRITE(dspcntr_reg, dspcntr);
}

static void pfit_landscape(struct drm_device *dev,
				int hsrc_sz, int vsrc_sz,
				int hdst_sz, int vdst_sz)
{
	int hscale = 0, vscale = 0;

	REG_WRITE(PFIT_CONTROL, PFIT_ENABLE | PFIT_PIPE_SELECT_B |
					PFIT_SCALING_MODE_PROGRAM);

        hscale = PFIT_FRACTIONAL_VALUE * (hsrc_sz + 1) / (hdst_sz + 1);
        vscale = PFIT_FRACTIONAL_VALUE * (vsrc_sz + 1) / (vdst_sz + 1);

        PSB_DEBUG_ENTRY("hscale = 0x%X, vscale = 0X%X\n", hscale, vscale);

        REG_WRITE(PFIT_PGM_RATIOS,
                hscale << PFIT_HORIZ_SCALE_SHIFT |
                vscale << PFIT_VERT_SCALE_SHIFT);
}

static int mdfld_intel_set_scaling_property(struct drm_crtc *crtc, int x, int y, int pipe)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct drm_framebuffer *fb = crtc->fb;
	struct drm_display_mode *adjusted_mode = & psb_intel_crtc->saved_adjusted_mode;
	uint64_t scalingType = psb_intel_crtc->scaling_type;
	uint64_t scalingStep = psb_intel_crtc->scaling_step;
	static uint64_t lastScalingStep = -1;
	static int landscape = -1, last_landscape =-1;
	static int last_src_image_hor = 0, last_src_image_vert = 0;
	static int horScalingCount = 0;
	static int vertScalingCount = 0;
	int pipesrc_reg = PIPEASRC;
	int dspsize_reg = DSPASIZE;
	int dsppos_reg = DSPAPOS;
	int sprite_pos_x = 0, sprite_pos_y = 0;
	int sprite_width = 0, sprite_height = 0;
	int src_image_hor = 0, src_image_vert = 0;
	int hValue=-1, vValue=-1;

	switch (pipe) {
        case 0:
                break;
        case 1:
		pipesrc_reg = PIPEBSRC;
		dspsize_reg = DSPBSIZE;
		dsppos_reg = DSPBPOS;
                break;
        case 2:
		pipesrc_reg = PIPECSRC;
		dspsize_reg = DSPCSIZE;
		dsppos_reg = DSPCPOS;
                break;
        default:
                DRM_ERROR("Illegal Pipe Number. \n");
                return -EINVAL;
        }

	PSB_DEBUG_ENTRY("scalingType %llu\n", scalingType);
	/* pipesrc and dspsize control the size that is scaled from,
	 * which should always be the user's requested size.
	 */
	if (pipe == 1) {
		/*
		 * Frame buffer size may beyond active region in case of
		 * panning mode.
		 */
		sprite_width = MIN(fb->width, adjusted_mode->hdisplay);
		sprite_height = MIN(fb->height, adjusted_mode->vdisplay);

		PSB_DEBUG_ENTRY("fb->width:%d,fb->height:%d,adjusted_mode->hdisplay:%d,adjusted_mode->vdisplay:%d\n", fb->width,fb->height,adjusted_mode->hdisplay,adjusted_mode->vdisplay);

		if ((last_src_image_hor == 0) && (last_src_image_vert == 0)) {
			last_src_image_hor = adjusted_mode->hdisplay;
			last_src_image_vert = adjusted_mode->vdisplay;
		}

		hValue = (scalingStep&0xF0)>>4;
		vValue = (scalingStep&0xF00)>>8;

		PSB_DEBUG_ENTRY("last_src_image_hor %d, last_src_image_vert %d \n", last_src_image_hor, last_src_image_vert);

		if((last_src_image_hor != adjusted_mode->hdisplay)
			&&(last_src_image_vert != adjusted_mode->vdisplay)){
			psb_intel_crtc->scaling_step =0;
			scalingStep =0;
		}
		PSB_DEBUG_ENTRY("scalingType %llu, scalingStep ox%x, hValue %d, vValue %d \n", scalingType, scalingStep, hValue, vValue);

		switch (scalingType) {
		case DRM_MODE_SCALE_NONE:
		case DRM_MODE_SCALE_CENTER:
			/* This mode is used to support centering the screen by setting reg
			 * in DISPLAY controller */
			src_image_hor = adjusted_mode->hdisplay;
			src_image_vert = adjusted_mode->vdisplay;
			sprite_pos_x = (src_image_hor - sprite_width) / 2;
			sprite_pos_y = (src_image_vert - sprite_height) / 2;

			REG_WRITE(PFIT_CONTROL,
				REG_READ(PFIT_CONTROL) & ~PFIT_ENABLE);

			break;

		case DRM_MODE_SCALE_FULLSCREEN:
			src_image_hor = sprite_width;
			src_image_vert = sprite_height;
			sprite_pos_x = 0;
			sprite_pos_y = 0;

			if ((adjusted_mode->hdisplay > sprite_width) ||
					(adjusted_mode->vdisplay > sprite_height))
				REG_WRITE(PFIT_CONTROL,
						PFIT_ENABLE |
						PFIT_PIPE_SELECT_B |
						PFIT_SCALING_MODE_AUTO);
			break;

		case DRM_MODE_SCALE_ASPECT:
			sprite_pos_x = 0;
			sprite_pos_y = 0;
			sprite_height = fb->height;
			sprite_width = fb->width;
			src_image_hor = fb->width;
			src_image_vert = fb->height;

			/* Use panel fitting when the display does not match
			 * with the framebuffer size */
			if ((adjusted_mode->hdisplay != fb->width) ||
			    (adjusted_mode->vdisplay != fb->height)) {
				if (fb->width > fb->height) {
					pr_debug("[hdmi]: Landscape mode...\n");
					/* Landscape mode: program ratios is
					 * used because 480p does not work with
					 * auto */
                                        if (adjusted_mode->vdisplay == 480)
                                                pfit_landscape(dev,
                                                        sprite_width,
                                                        sprite_height,
                                                        adjusted_mode->hdisplay,
                                                        adjusted_mode->vdisplay);
                                        else
						REG_WRITE(PFIT_CONTROL,
							PFIT_ENABLE |
							PFIT_PIPE_SELECT_B |
							PFIT_SCALING_MODE_AUTO);
				} else {
					/* Portrait mode */
					pr_debug("[hdmi]: Portrait mode...\n");
					if (adjusted_mode->vdisplay == 768 &&
						adjusted_mode->hdisplay == 1024) {
							src_image_hor = adjusted_mode->hdisplay *
								fb->height /
								adjusted_mode->vdisplay;
							src_image_vert = fb->height;
							sprite_pos_x = (src_image_hor - fb->width) / 2;
							REG_WRITE(PFIT_CONTROL,
								PFIT_ENABLE |
								PFIT_PIPE_SELECT_B |
								PFIT_SCALING_MODE_AUTO);
					} else
						REG_WRITE(PFIT_CONTROL,
							PFIT_ENABLE |
							PFIT_PIPE_SELECT_B |
							PFIT_SCALING_MODE_PILLARBOX);
				}
			} else {
				/* Disable panel fitting */
				REG_WRITE(PFIT_CONTROL, 0);
			}
			break;

		default:
			/* Android will not change mode, however ,we have tools
			to change HDMI timing so there is some cases frame
			buffer no change ,but timing changed mode setting, in
			this case. mode information for source size is not
			right, so here use fb information for source/sprite
			size*/

			/* The defined sprite rectangle must always be
			completely contained within the displayable area of the
			screen image (frame buffer). */
			sprite_pos_x = 0;
			sprite_pos_y = 0;
			sprite_height = fb->height;
			sprite_width = fb->width;
			src_image_hor = fb->width;
			src_image_vert = fb->height;
			if ((adjusted_mode->hdisplay != fb->width) ||
					(adjusted_mode->vdisplay != fb->height))
				REG_WRITE(PFIT_CONTROL, PFIT_ENABLE |
						PFIT_PIPE_SELECT_B);

			break;
		}

		if (scalingStep != 0)
		{
			u32 sprite_pos, sprite_size, src_size;

			if(fb->width > fb->height)
				landscape = 0;
			if(fb->width < fb->height)
				landscape = 1;

			if ((last_landscape == -1)
				||(last_landscape == landscape))
			{
				horScalingCount = hValue;
				vertScalingCount = vValue;

				PSB_DEBUG_ENTRY("Before: Sprite position: (%d, %d)\n", sprite_pos_x,
					sprite_pos_y);
				PSB_DEBUG_ENTRY("Before: Sprite size: %d x %d\n", sprite_width,
					sprite_height);
				PSB_DEBUG_ENTRY("Before: Pipe source image size: %d x %d\n",
					src_image_hor, src_image_vert);
				PSB_DEBUG_ENTRY("hValue %d, vValue %d \n", hValue, vValue);
				if (!((fb->width < fb->height) &&(scalingType == DRM_MODE_SCALE_ASPECT)))
				{
					if (hValue !=0) {
						sprite_pos_x = sprite_pos_x + (adjusted_mode->hdisplay* hValue)/100;
						src_image_hor = src_image_hor + (adjusted_mode->hdisplay*hValue*2)/100;
					}
				}

				if (vValue!=0) {
					sprite_pos_y = sprite_pos_y + (adjusted_mode->vdisplay*vValue)/100;
					src_image_vert = src_image_vert + (adjusted_mode->vdisplay*vValue*2)/100;
				}

				if ((hValue == -1)||(vValue==-1))
					psb_intel_crtc->scaling_step = 0;

				last_landscape = landscape;
			}

			if (last_landscape != landscape) {
				if (!((fb->width < fb->height) &&(scalingType == DRM_MODE_SCALE_ASPECT))){
					sprite_pos_x = sprite_pos_x + (adjusted_mode->hdisplay* horScalingCount)/100;
					src_image_hor = src_image_hor + (adjusted_mode->hdisplay*horScalingCount*2)/100;
				}

				sprite_pos_y = sprite_pos_y + (adjusted_mode->vdisplay*vertScalingCount)/100;
				src_image_vert = src_image_vert + (adjusted_mode->vdisplay*vertScalingCount*2)/100;

				last_landscape = landscape;
			}
		}

		if(scalingStep == 0) {
			horScalingCount = 0;
			vertScalingCount = 0;
			last_landscape = -1;
			hValue =-1;
			vValue =-1;
		}

		last_src_image_hor = adjusted_mode->hdisplay;
		last_src_image_vert = adjusted_mode->vdisplay;

		PSB_DEBUG_ENTRY("After: Sprite position: (%d, %d)\n", sprite_pos_x,
				sprite_pos_y);
		PSB_DEBUG_ENTRY("After: Sprite size: %d x %d\n", sprite_width,
				sprite_height);
		PSB_DEBUG_ENTRY("After: Pipe source image size: %d x %d\n",
				src_image_hor, src_image_vert);
		PSB_DEBUG_ENTRY(" Adjust mode size: %d x %d\n",
				adjusted_mode->hdisplay, adjusted_mode->vdisplay);

		REG_WRITE(dsppos_reg, (sprite_pos_y << 16) | sprite_pos_x);
		REG_WRITE(dspsize_reg, ((sprite_height - 1) << 16) |
				(sprite_width - 1));
		REG_WRITE(pipesrc_reg, ((src_image_hor - 1) << 16) |
				(src_image_vert - 1));
	} else {
		REG_WRITE(dspsize_reg, ((adjusted_mode->vdisplay - 1) << 16) | (adjusted_mode->hdisplay - 1));
		REG_WRITE(pipesrc_reg, ((adjusted_mode->hdisplay - 1) << 16) | (adjusted_mode->vdisplay - 1));
		REG_WRITE(dsppos_reg, 0);
	}

	return 0;
}

int mdfld__intel_pipe_set_base(struct drm_crtc *crtc, int x, int y, struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_i915_master_private *master_priv; */
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_framebuffer *psbfb = to_psb_fb(crtc->fb);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	unsigned long Start, Size, Offset;
	int dsplinoff = DSPALINOFF;
	int dspsurf = DSPASURF;
	int dspstride = DSPASTRIDE;
	int dspcntr_reg = DSPACNTR;
	u32 dspcntr;
	int ret = 0;
	MRST_ERROR eError = MRST_ERROR_GENERIC;

	memcpy(&globle_dev, dev, sizeof(struct drm_device));

	PSB_DEBUG_ENTRY("pipe = 0x%x. \n", pipe);

	/* no fb bound */
	if (!crtc->fb) {
		PSB_DEBUG_ENTRY("No FB bound\n");
		return 0;
	}

	switch (pipe) {
	case 0:
		if (IS_MID(dev))
			dsplinoff = DSPALINOFF;
		break;
	case 1:
		dsplinoff = DSPBLINOFF;
		dspsurf = DSPBSURF;
		dspstride = DSPBSTRIDE;
		dspcntr_reg = DSPBCNTR;
		break;
	case 2:
		dsplinoff = DSPCLINOFF;
		dspsurf = DSPCSURF;
		dspstride = DSPCSTRIDE;
		dspcntr_reg = DSPCCNTR;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return -EINVAL;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return 0;

	mdfld_intel_set_scaling_property(crtc, x, y, pipe);

	Start = mode_dev->bo_offset(dev, psbfb);
	Size = mode_dev->bo_size(dev, psbfb);
	Offset = y * crtc->fb->pitch + x * (crtc->fb->bits_per_pixel / 8);

	/* Try to attach/de-attach Plane B to an existing swap chain,
	 * especially with another frame buffer inserted into GTT. */
	eError = MRSTLFBChangeSwapChainProperty(&Start, Size, pipe);
	if ((eError != MRST_OK) && (eError != MRST_ERROR_INIT_FAILURE)) {
		DRM_ERROR("Failed to attach/de-attach pipe %d to a"
				"swap chain.\n", pipe);
		ret = -EINVAL;
		goto psb_intel_pipe_set_base_exit;
	}

	REG_WRITE(dspstride, crtc->fb->pitch);
	dspcntr = REG_READ(dspcntr_reg);
	dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;

	switch (crtc->fb->bits_per_pixel) {
	case 8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case 16:
		if (crtc->fb->depth == 15)
			dspcntr |= DISPPLANE_15_16BPP;
		else
			dspcntr |= DISPPLANE_16BPP;
		break;
	case 24:
	case 32:
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
		break;
	default:
		DRM_ERROR("Unknown color depth\n");
		ret = -EINVAL;
		goto psb_intel_pipe_set_base_exit;
	}
	REG_WRITE(dspcntr_reg, dspcntr);

	PSB_DEBUG_ENTRY("Writing base %08lX %08lX %d %d\n", Start, Offset, x, y);

	if (IS_I965G(dev) || IS_MID(dev)) {
		REG_WRITE(dsplinoff, Offset);
		REG_READ(dsplinoff);
		REG_WRITE(dspsurf, Start);
		REG_READ(dspsurf);
	} else {
		REG_WRITE(dsplinoff, Start + Offset);
		REG_READ(dsplinoff);
	}

psb_intel_pipe_set_base_exit:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return ret;
}

/**
 * Disable the pipe, plane and pll.
 *
 */
void mdfld_disable_crtc (struct drm_device *dev, int pipe)
{
	int dpll_reg = MRST_DPLL_A;
	int dspcntr_reg = DSPACNTR;
	int dspbase_reg = MRST_DSPABASE;
	int pipeconf_reg = PIPEACONF;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 temp;

	PSB_DEBUG_ENTRY("pipe = %d\n", pipe);

#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/**
	 * NOTE: this path only works for TMD panel now. update it to
	 * support all MIPI panels later.
	 */
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
		(get_panel_type(dev, pipe) == TMD_6X10_VID) ||
		(get_panel_type(dev, pipe) == H8C7_VID) ||
		(get_panel_type(dev, pipe) == GI_SONY_VID) ||
		/* SC1 setting */
		(get_panel_type(dev, pipe) == AUO_SC1_VID)))
		return;
#endif

	switch (pipe) {
	case 0:
		break;
	case 1:
		dpll_reg = MDFLD_DPLL_B;
		dspcntr_reg = DSPBCNTR;
		dspbase_reg = DSPBSURF;
		pipeconf_reg = PIPEBCONF;
		break;
	case 2:
		dpll_reg = MRST_DPLL_A;
		dspcntr_reg = DSPCCNTR;
		dspbase_reg = MDFLD_DSPCBASE;
		pipeconf_reg = PIPECCONF;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return;
	}

	if (pipe != 1)
		mdfld_dsi_gen_fifo_ready (dev, gen_fifo_stat_reg, HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);

	/* Disable display plane */
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
		REG_WRITE(dspcntr_reg,
			  temp & ~DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		REG_READ(dspbase_reg);
	}

	/* Next, disable display pipes */
	temp = REG_READ(pipeconf_reg);
	if ((temp & PIPEACONF_ENABLE) != 0) {
		temp &= ~PIPEACONF_ENABLE;
		temp |= PIPECONF_PLANE_OFF | PIPECONF_CURSOR_OFF;
		REG_WRITE(pipeconf_reg, temp);
		REG_READ(pipeconf_reg);

		/* Wait for for the pipe disable to take effect. */
		mdfldWaitForPipeDisable(dev, pipe);
	}

	temp = REG_READ(dpll_reg);
	if (temp & DPLL_VCO_ENABLE) {
		if (((pipe != 1) && !((REG_READ(PIPEACONF) | REG_READ(PIPECCONF)) & PIPEACONF_ENABLE))
				|| (pipe == 1)){
			temp &= ~(DPLL_VCO_ENABLE);
			REG_WRITE(dpll_reg, temp);
			REG_READ(dpll_reg);
			/* Wait for the clocks to turn off. */
			/* FIXME_MDFLD PO may need more delay */
			udelay(500);

			if (!(temp & MDFLD_PWR_GATE_EN)) {
				/* gating power of DPLL */
				REG_WRITE(dpll_reg, temp | MDFLD_PWR_GATE_EN);
				/* FIXME_MDFLD PO - change 500 to 1 after PO */
				udelay(5000);
			}
		}
	}

}

/**
 * Sets the power management mode of the pipe and plane.
 *
 * This code should probably grow support for turning the cursor off and back
 * on appropriately at the same time as we're turning the pipe off/on.
 */
static void mdfld_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *dev = crtc->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	int dpll_reg = MRST_DPLL_A;
	int dspcntr_reg = DSPACNTR;
	int dspbase_reg = MRST_DSPABASE;
	int pipeconf_reg = PIPEACONF;
	u32 pipestat_reg = PIPEASTAT;
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	u32 mipi_enable_reg = MIPIA_DEVICE_READY_REG;
	u32 temp;
	u32 mipi_port_ctrl = 0, mipi_dev_ready = 0;
	bool enabled;
	int timeout = 0;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_registers *regs = NULL;

	PSB_DEBUG_ENTRY("mode = %d, pipe = %d\n", mode, pipe);

#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/**
	 * MIPI dpms
	 * NOTE: this path only works for TMD panel now. update it to
	 * support all MIPI panels later.
	 */
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
		(get_panel_type(dev, pipe) == TMD_6X10_VID) ||
		(get_panel_type(dev, pipe) == H8C7_VID) ||
		(get_panel_type(dev, pipe) == GI_SONY_VID) ||
		/* SC1 setting */
		(get_panel_type(dev, pipe) == AUO_SC1_VID))) {
			return;
	}
#endif

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	 /* Ignore if system is already in DSR and in suspended state. */
	if(gbgfxsuspended && gbdispstatus == false && mode == 3){
	    if(dev_priv->rpm_enabled && pipe == 1){
	//          dev_priv->is_mipi_on = false;
	            pm_request_idle(&gpDrmDevice->pdev->dev);
	    }

	    ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	    return;
	}else if(mode == 0) {
		//do not need to set gbdispstatus=true in crtc.
		//this will be set in encoder such as mdfld_dsi_dbi_dpms
	    //gbdispstatus = true;
	}

	switch (pipe) {
	case 0:
		break;
	case 1:
		dpll_reg = DPLL_B;
		dspcntr_reg = DSPBCNTR;
		dspbase_reg = MRST_DSPBBASE;
		pipeconf_reg = PIPEBCONF;
		pipeconf = dev_priv->pipeconf1;
		dspcntr = dev_priv->dspcntr1;
		if (IS_MDFLD(dev))
			dpll_reg = MDFLD_DPLL_B;
		break;
	case 2:
		dpll_reg = MRST_DPLL_A;
		dspcntr_reg = DSPCCNTR;
		dspbase_reg = MDFLD_DSPCBASE;
		pipeconf_reg = PIPECCONF;
		pipestat_reg = PIPECSTAT;
		pipeconf = dev_priv->pipeconf2;
		dspcntr = dev_priv->dspcntr2;
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		mipi_enable_reg = MIPIA_DEVICE_READY_REG + MIPIC_REG_OFFSET;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		return;
	}

	/* XXX: When our outputs are all unaware of DPMS modes other than off
	 * and on, we should map those modes to DRM_MODE_DPMS_OFF in the CRTC.
	 */
	switch (mode) {
	case DRM_MODE_DPMS_ON:
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
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

			/**
			 * wait for DSI PLL to lock
			 * NOTE: only need to poll status of pipe 0 and pipe 1,
			 * since both MIPI pipes share the same PLL.
			 */
			while ((pipe != 2) && (timeout < 20000) && !(REG_READ(pipeconf_reg) & PIPECONF_DSIPLL_LOCK)) {
				udelay(150);
				timeout ++;
			}
		}

		/* Enable the plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
			REG_WRITE(dspcntr_reg,
				temp | DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		}

		/* Enable the pipe */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) == 0) {
			REG_WRITE(pipeconf_reg, pipeconf);

			/* Wait for for the pipe enable to take effect. */
			mdfldWaitForPipeEnable(dev, pipe);
		}

		/*workaround for sighting 3741701 Random X blank display*/
		/*perform w/a in video mode only on pipe A or C*/
		if ((pipe == 0 || pipe == 2) &&
			(is_panel_vid_or_cmd(dev) == MDFLD_DSI_ENCODER_DPI)) {
			REG_WRITE(pipestat_reg, REG_READ(pipestat_reg));
			msleep(100);
			if(PIPE_VBLANK_STATUS & REG_READ(pipestat_reg)) {
				PSB_DEBUG_ENTRY("OK");
			} else {
				PSB_DEBUG_ENTRY("STUCK!!!!");
				/*shutdown controller*/
				temp = REG_READ(dspcntr_reg);
				REG_WRITE(dspcntr_reg, temp & ~DISPLAY_PLANE_ENABLE);
				REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
				/*mdfld_dsi_dpi_shut_down(dev, pipe);*/
				REG_WRITE(0xb048, 1);
				msleep(100);
				temp = REG_READ(pipeconf_reg);
				temp &= ~PIPEACONF_ENABLE;
				REG_WRITE(pipeconf_reg, temp);
				msleep(100); /*wait for pipe disable*/
			/*printk(KERN_ALERT "70008 is %x\n", REG_READ(0x70008));
			printk(KERN_ALERT "b074 is %x\n", REG_READ(0xb074));*/
				REG_WRITE(mipi_enable_reg, 0);
				msleep(100);
				PSB_DEBUG_ENTRY("70008 is %x\n", REG_READ(0x70008));
				PSB_DEBUG_ENTRY("b074 is %x\n", REG_READ(0xb074));
				REG_WRITE(0xb004, REG_READ(0xb004));
				/* try to bring the controller back up again*/
				REG_WRITE(mipi_enable_reg, 1);
				temp = REG_READ(dspcntr_reg);
				REG_WRITE(dspcntr_reg, temp | DISPLAY_PLANE_ENABLE);
				REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
				/*mdfld_dsi_dpi_turn_on(dev, pipe);*/
				REG_WRITE(0xb048, 2);
				msleep(100);
				temp = REG_READ(pipeconf_reg);
				temp |= PIPEACONF_ENABLE;
				REG_WRITE(pipeconf_reg, temp);
			}
		}

		psb_intel_crtc_load_lut(crtc);

		/* Give the overlay scaler a chance to enable
		   if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, true); TODO */

		break;
	case DRM_MODE_DPMS_OFF:
		/* Give the overlay scaler a chance to disable
		 * if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, FALSE); TODO */
		if (pipe != 1)
			mdfld_dsi_gen_fifo_ready (dev, gen_fifo_stat_reg, HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY);

		/* Disable the VGA plane that we never use */
		REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);

		/* Disable display plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
			REG_WRITE(dspcntr_reg,
				  temp & ~DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			REG_READ(dspbase_reg);
		}

		/* Next, disable display pipes */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) != 0) {
			temp &= ~PIPEACONF_ENABLE;
			temp |= PIPECONF_PLANE_OFF | PIPECONF_CURSOR_OFF;
			REG_WRITE(pipeconf_reg, temp);
			REG_READ(pipeconf_reg);

			/* Wait for for the pipe disable to take effect. */
			mdfldWaitForPipeDisable(dev, pipe);
		}

		temp = REG_READ(dpll_reg);
		if (temp & DPLL_VCO_ENABLE) {
			if (((pipe != 1) && !((REG_READ(PIPEACONF) | REG_READ(PIPECCONF)) & PIPEACONF_ENABLE))
					|| (pipe == 1)){
				/*
				 * FIXME: better to move it into the MIPI
				 * encoder DPMS off process.
				 */
				if (get_panel_type(dev, pipe) == AUO_SC1_CMD && pipe < sizeof(dev_priv->dsi_configs)/sizeof(*(dev_priv->dsi_configs))) {
					dsi_config =
						dev_priv->dsi_configs[pipe];
					regs = &dsi_config->regs;
					mipi_dev_ready =
						REG_READ(regs->device_ready_reg)
						& ~DSI_DEVICE_READY;
					mipi_port_ctrl =
						REG_READ(regs->mipi_reg)
						& ~MIPI_PORT_EN;
					REG_WRITE(regs->device_ready_reg,
							mipi_dev_ready);
					REG_WRITE(regs->mipi_reg,
							mipi_port_ctrl);
				}

				temp &= ~(DPLL_VCO_ENABLE);
				if (get_panel_type(dev, pipe) == AUO_SC1_CMD)
					temp |= MDFLD_PWR_GATE_EN;

				REG_WRITE(dpll_reg, temp);
				REG_READ(dpll_reg);
				/* Wait for the clocks to turn off. */
				/* FIXME_MDFLD PO may need more delay */
				udelay(500);
#if 0 /* FIXME_MDFLD Check if we need to power gate the PLL */
		if (!(temp & MDFLD_PWR_GATE_EN)) {
			/* gating power of DPLL */
			REG_WRITE(dpll_reg, temp | MDFLD_PWR_GATE_EN);
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(5000);
		}
#endif
			}
		}
		break;
	}

	enabled = crtc->enabled && mode != DRM_MODE_DPMS_OFF;

#if 0				/* JB: Add vblank support later */
	if (enabled)
		dev_priv->vblank_pipe |= (1 << pipe);
	else
		dev_priv->vblank_pipe &= ~(1 << pipe);
#endif

#if 0				/* JB: Add sarea support later */
	if (!dev->primary->master)
		return;

	master_priv = dev->primary->master->driver_priv;
	if (!master_priv->sarea_priv)
		return;

	switch (pipe) {
	case 0:
		master_priv->sarea_priv->planeA_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeA_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	case 1:
		master_priv->sarea_priv->planeB_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeB_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	default:
		DRM_ERROR("Can't update pipe %d in SAREA\n", pipe);
		break;
	}
#endif

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}


#define MDFLD_LIMT_DPLL_19	    0
#define MDFLD_LIMT_DPLL_25	    1
#define MDFLD_LIMT_DPLL_38	    2
#define MDFLD_LIMT_DPLL_83	    3
#define MDFLD_LIMT_DPLL_100	    4
#define MDFLD_LIMT_DSIPLL_19	    5
#define MDFLD_LIMT_DSIPLL_25	    6
#define MDFLD_LIMT_DSIPLL_38	    7
#define MDFLD_LIMT_DSIPLL_83	    8
#define MDFLD_LIMT_DSIPLL_100	    9

#define MDFLD_DOT_MIN		  19750
#define MDFLD_DOT_MAX		  120000
#define MDFLD_DPLL_M_MIN_19	    113
#define MDFLD_DPLL_M_MAX_19	    155
#define MDFLD_DPLL_P1_MIN_19	    2
#define MDFLD_DPLL_P1_MAX_19	    10
#define MDFLD_DPLL_M_MIN_25	    101
#define MDFLD_DPLL_M_MAX_25	    130
#define MDFLD_DPLL_P1_MIN_25	    2
#define MDFLD_DPLL_P1_MAX_25	    10
#define MDFLD_DPLL_M_MIN_38        113
#define MDFLD_DPLL_M_MAX_38        155
#define MDFLD_DPLL_P1_MIN_38       2
#define MDFLD_DPLL_P1_MAX_38       10
#define MDFLD_DPLL_M_MIN_83	    64
#define MDFLD_DPLL_M_MAX_83	    64
#define MDFLD_DPLL_P1_MIN_83	    2
#define MDFLD_DPLL_P1_MAX_83	    2
#define MDFLD_DPLL_M_MIN_100	    64
#define MDFLD_DPLL_M_MAX_100	    64
#define MDFLD_DPLL_P1_MIN_100	    2
#define MDFLD_DPLL_P1_MAX_100	    2
#define MDFLD_DSIPLL_M_MIN_19	    64
#define MDFLD_DSIPLL_M_MAX_19	    175
#define MDFLD_DSIPLL_P1_MIN_19	    3
#define MDFLD_DSIPLL_P1_MAX_19	    8
#define MDFLD_DSIPLL_M_MIN_25	    97
#define MDFLD_DSIPLL_M_MAX_25	    140
#define MDFLD_DSIPLL_P1_MIN_25	    3
#define MDFLD_DSIPLL_P1_MAX_25	    9
#define MDFLD_DSIPLL_M_MIN_38      66
#define MDFLD_DSIPLL_M_MAX_38      87
#define MDFLD_DSIPLL_P1_MIN_38     3
#define MDFLD_DSIPLL_P1_MAX_38     8
#define MDFLD_DSIPLL_M_MIN_83	    33
#define MDFLD_DSIPLL_M_MAX_83	    92
#define MDFLD_DSIPLL_P1_MIN_83	    2
#define MDFLD_DSIPLL_P1_MAX_83	    3
#define MDFLD_DSIPLL_M_MIN_100	    97
#define MDFLD_DSIPLL_M_MAX_100	    140
#define MDFLD_DSIPLL_P1_MIN_100	    3
#define MDFLD_DSIPLL_P1_MAX_100	    9
#define VCO_MAX                     3200000

static const struct mrst_limit_t mdfld_limits[] = {
	{			/* MDFLD_LIMT_DPLL_19 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_19, .max = MDFLD_DPLL_M_MAX_19},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_19, .max = MDFLD_DPLL_P1_MAX_19},
	 },
	{			/* MDFLD_LIMT_DPLL_25 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_25, .max = MDFLD_DPLL_M_MAX_25},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_25, .max = MDFLD_DPLL_P1_MAX_25},
	 },
	{			/* MDFLD_LIMT_DPLL_38 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_38, .max = MDFLD_DPLL_M_MAX_38},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_38, .max = MDFLD_DPLL_P1_MAX_38},
	 },
	{			/* MDFLD_LIMT_DPLL_83 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_83, .max = MDFLD_DPLL_M_MAX_83},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_83, .max = MDFLD_DPLL_P1_MAX_83},
	 },
	{			/* MDFLD_LIMT_DPLL_100 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_100, .max = MDFLD_DPLL_M_MAX_100},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_100, .max = MDFLD_DPLL_P1_MAX_100},
	 },
	{			/* MDFLD_LIMT_DSIPLL_19 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_19, .max = MDFLD_DSIPLL_M_MAX_19},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_19, .max = MDFLD_DSIPLL_P1_MAX_19},
	 },
	{			/* MDFLD_LIMT_DSIPLL_25 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_25, .max = MDFLD_DSIPLL_M_MAX_25},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_25, .max = MDFLD_DSIPLL_P1_MAX_25},
	 },
	{			/* MDFLD_LIMT_DSIPLL_38 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_38, .max = MDFLD_DSIPLL_M_MAX_38},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_38, .max = MDFLD_DSIPLL_P1_MAX_38},
	 },
	{			/* MDFLD_LIMT_DSIPLL_83 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_83, .max = MDFLD_DSIPLL_M_MAX_83},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_83, .max = MDFLD_DSIPLL_P1_MAX_83},
	 },
	{			/* MDFLD_LIMT_DSIPLL_100 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_100, .max = MDFLD_DSIPLL_M_MAX_100},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_100, .max = MDFLD_DSIPLL_P1_MAX_100},
	 },
};

#define MDFLD_M_MIN	    21
#define MDFLD_M_MAX	    180
static const u32 mdfld_m_converts[] = {
/* M configuration table from 9-bit LFSR table */
	224, 368, 440, 220, 366, 439, 219, 365, 182, 347, /* 21 - 30 */
	173, 342, 171, 85, 298, 149, 74, 37, 18, 265,   /* 31 - 40 */
	388, 194, 353, 432, 216, 108, 310, 155, 333, 166, /* 41 - 50 */
	83, 41, 276, 138, 325, 162, 337, 168, 340, 170, /* 51 - 60 */
	341, 426, 469, 234, 373, 442, 221, 110, 311, 411, /* 61 - 70 */
	461, 486, 243, 377, 188, 350, 175, 343, 427, 213, /* 71 - 80 */
	106, 53, 282, 397, 354, 227, 113, 56, 284, 142, /* 81 - 90 */
	71, 35, 273, 136, 324, 418, 465, 488, 500, 506, /* 91 - 100 */
	253, 126, 63, 287, 399, 455, 483, 241, 376, 444, /* 101 - 110 */
	478, 495, 503, 251, 381, 446, 479, 239, 375, 443, /* 111 - 120 */
	477, 238, 119, 315, 157, 78, 295, 147, 329, 420, /* 121 - 130 */
	210, 105, 308, 154, 77, 38, 275, 137, 68, 290, /* 131 - 140 */
	145, 328, 164, 82, 297, 404, 458, 485, 498, 249, /* 141 - 150 */
	380, 190, 351, 431, 471, 235, 117, 314, 413, 206, /* 151 - 160 */
	103, 51, 25, 12, 262, 387, 193, 96, 48, 280, /* 161 - 170 */
	396, 198, 99, 305, 152, 76, 294, 403, 457, 228, /* 171 - 180 */
};

static const struct mrst_limit_t *mdfld_limit(struct drm_crtc *crtc)
{
	const struct mrst_limit_t *limit = NULL;
	struct drm_device *dev = crtc->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_MIPI)
	    || psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_MIPI2)) {
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) || (dev_priv->ksel == KSEL_BYPASS_19))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_19];
		else if (dev_priv->ksel == KSEL_BYPASS_25)
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_25];
		else if (dev_priv->ksel == KSEL_CRYSTAL_38)
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_38];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) && (dev_priv->core_freq == 166))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_83];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			 (dev_priv->core_freq == 100 || dev_priv->core_freq == 200))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_100];
	} else if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_HDMI)) {
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) || (dev_priv->ksel == KSEL_BYPASS_19))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_19];
		else if (dev_priv->ksel == KSEL_BYPASS_25)
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_25];
		else if (dev_priv->ksel == KSEL_CRYSTAL_38)
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_38];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) && (dev_priv->core_freq == 166))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_83];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			 (dev_priv->core_freq == 100 || dev_priv->core_freq == 200))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_100];
	} else {
		limit = NULL;
		PSB_DEBUG_ENTRY("mdfld_limit Wrong display type. \n");
	}

	return limit;
}

/** Derive the pixel clock for the given refclk and divisors for 8xx chips. */
static void mdfld_clock(int refclk, struct mrst_clock_t *clock)
{
	clock->dot = (refclk * clock->m) / clock->p1;
}

/** Derive the vco clock for the given refclk and divisors for 8xx chips. */
static int mdfld_vco_clock(int refclk, struct mrst_clock_t *clock)
{
	return refclk * clock->m;
}

/**
 * Returns a set of divisors for the desired target clock with the given refclk,
 * or FALSE.  Divisor values are the actual divisors for
 */
static bool
mdfldFindBestPLL(struct drm_crtc *crtc, int target, int refclk,
		struct mrst_clock_t *best_clock)
{
	struct mrst_clock_t clock;
	const struct mrst_limit_t *limit = mdfld_limit(crtc);
	int err = target;
	int vco_clock;

	memset(best_clock, 0, sizeof(*best_clock));

	PSB_DEBUG_ENTRY("mdfldFindBestPLL target = %d,"
			 "m_min = %d, m_max = %d, p_min = %d, p_max = %d. \n", target, limit->m.min, limit->m.max, limit->p1.min, limit->p1.max);

	for (clock.m = limit->m.min; clock.m <= limit->m.max; clock.m++) {
		for (clock.p1 = limit->p1.min; clock.p1 <= limit->p1.max;
		     clock.p1++) {
			int this_err;

			mdfld_clock(refclk, &clock);
			vco_clock = mdfld_vco_clock(refclk, &clock);

			this_err = abs(clock.dot - target);
			if (this_err <= err && vco_clock < VCO_MAX) {
				*best_clock = clock;
				err = this_err;
			}
		}
	}
	PSB_DEBUG_ENTRY("mdfldFindBestPLL target = %d,"
			 "m = %d, p = %d. \n", target, best_clock->m, best_clock->p1);
	PSB_DEBUG_ENTRY("mdfldFindBestPLL err = %d.\n", err);

	return err != target;
}

static int mdfld_crtc_dsi_pll_calc(struct drm_crtc *crtc,
				struct mdfld_dsi_config *dsi_config,
				struct drm_device *dev,
				u32 *out_dpll,
				u32 *out_fp,
				struct drm_display_mode *adjusted_mode)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct mrst_clock_t clock;
	u32 dpll = 0, fp = 0;
	int refclk = 0;
	int clk_n = 0, clk_p2 = 0, clk_byte = 1, clk = 0, m_conv = 0, clk_tmp = 0;
	bool ok;

	if ((dev_priv->ksel == KSEL_CRYSTAL_19) || (dev_priv->ksel == KSEL_BYPASS_19))
	{
		refclk = 19200;
		clk_n = 1, clk_p2 = 8;
	} else if (dev_priv->ksel == KSEL_BYPASS_25) {
		refclk = 25000;
		clk_n = 1, clk_p2 = 8;
	} else if (dev_priv->ksel == KSEL_CRYSTAL_38) {
		refclk = 38400;
		clk_n = 1, clk_p2 = 8;
	} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) && (dev_priv->core_freq == 166)) {
		refclk = 83000;
		clk_n = 4, clk_p2 = 8;
	} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
		   (dev_priv->core_freq == 100 || dev_priv->core_freq == 200)) {
		refclk = 100000;
		clk_n = 4, clk_p2 = 8;
	}else{
		refclk = 19200;
		clk_n = 1, clk_p2 = 8;
	}

	dev_priv->bpp = 24;
	clk_byte = dev_priv->bpp / 8;

	if (dsi_config->lane_count)
		clk = adjusted_mode->clock / dsi_config->lane_count;
	else
		clk = adjusted_mode->clock;

	clk_tmp = clk * clk_n * clk_p2 * clk_byte;

	PSB_DEBUG_ENTRY("ref_clk: %d, clk = %d, clk_n = %d, clk_p2 = %d. \n", refclk, clk, clk_n, clk_p2);
	PSB_DEBUG_ENTRY("adjusted_mode->clock = %d, clk_tmp = %d. \n", adjusted_mode->clock, clk_tmp);

	ok = mdfldFindBestPLL(crtc, clk_tmp, refclk, &clock);
	dev_priv->tmds_clock_khz = clock.dot / (clk_n * clk_p2 * clk_byte);

	if (!ok) {
		DRM_ERROR
		    ("mdfldFindBestPLL fail in mdfld_crtc_mode_set. \n");
	} else {
		m_conv = mdfld_m_converts[(clock.m - MDFLD_M_MIN)];
		PSB_DEBUG_ENTRY("dot clock = %d,"
			 "m = %d, p1 = %d, m_conv = %d. \n", clock.dot, clock.m,
			 clock.p1, m_conv);
	}

	dpll = 0x00000000;
	fp = (clk_n / 2) << 16;
	fp |= m_conv;

	/* compute bitmask from p1 value */
	dpll |= (1 << (clock.p1 - 2)) << 17;

	*(out_dpll) = dpll;
	*(out_fp) = fp;

	PSB_DEBUG_ENTRY("dsi dpll = 0x%x  fp = 0x%x\n", dpll, fp);
	return 0;
}

static int mdfld_crtc_dsi_mode_set(struct drm_crtc *crtc,
				struct mdfld_dsi_config *dsi_config,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb)
{
	struct drm_device *dev;
	struct psb_intel_crtc *mdfld_dsi_crtc;
	struct psb_framebuffer *mdfld_fb;
	struct psb_intel_mode_device *mode_dev;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	int fb_bpp;
	int fb_pitch;
	int fb_depth;
	int hdelay;
	static int init_flag = 1;   /*bootstrap flag*/

	if (!crtc || !crtc->fb) {
		DRM_ERROR("Invalid CRTC\n");
		return -EINVAL;
	}

	if (!dsi_config) {
		DRM_ERROR("Invalid DSI config\n");
		return -EINVAL;
	}

	mdfld_dsi_crtc = to_psb_intel_crtc(crtc);
	mdfld_fb = to_psb_fb(crtc->fb);
	mode_dev = mdfld_dsi_crtc->mode_dev;
	mode = adjusted_mode;
	ctx = &dsi_config->dsi_hw_context;
	fb_bpp = crtc->fb->bits_per_pixel;
	fb_pitch = crtc->fb->pitch;
	fb_depth = crtc->fb->depth;
	dev = crtc->dev;
	dev_priv = (struct drm_psb_private *)dev->dev_private;

	mutex_lock(&dsi_config->context_lock);

	ctx->vgacntr = 0x80000000;

	/*setup pll*/
	mdfld_crtc_dsi_pll_calc(crtc, dsi_config, dev,
				 &ctx->dpll,
				 &ctx->fp,
				 adjusted_mode);

	/*set up pipe timings*/
	ctx->htotal = (mode->crtc_hdisplay - 1) |
		((mode->crtc_htotal - 1) << 16);
	ctx->hblank = (mode->crtc_hblank_start - 1) |
		((mode->crtc_hblank_end - 1) << 16);
	ctx->hsync = (mode->crtc_hsync_start - 1) |
		((mode->crtc_hsync_end - 1) << 16);
	ctx->vtotal = (mode->crtc_vdisplay - 1) |
		((mode->crtc_vtotal - 1) << 16);
	ctx->vblank = (mode->crtc_vblank_start - 1) |
		((mode->crtc_vblank_end - 1) << 16);
	ctx->vsync = (mode->crtc_vsync_start - 1) |
		((mode->crtc_vsync_end - 1) << 16);

	/*pipe source*/
	ctx->pipesrc = ((mode->crtc_hdisplay - 1) << 16) |
		(mode->crtc_vdisplay - 1);

	/*setup dsp plane*/
	ctx->dsppos = 0;
	/* PR2 panel has 200 pixel dummy clocks,
	* So the display timing should be 800x1024, and surface
	* is 608x1024(64 bits align), then the information between android
	* and Linux frame buffer is not consistent.
	*/
	if (get_panel_type(dev, 0) == TMD_6X10_VID)
		ctx->dspsize = ((mode->crtc_vdisplay - 1) << 16) |
			(mode->crtc_hdisplay - 200  - 1);
	else
		ctx->dspsize = ((mode->crtc_vdisplay - 1) << 16) |
			(mode->crtc_hdisplay - 1);

	ctx->dspstride = fb_pitch;
	ctx->dspsurf = mode_dev->bo_offset(dev, mdfld_fb);
	ctx->dsplinoff = y * fb_pitch + x * (fb_bpp / 8);

	if (init_flag == 1) {
		printk(KERN_DEBUG"%s: ctx->dspsurf = 0x%x, ctx->dsplinoff = 0x%x\n",
				__func__, ctx->dsplinoff, ctx->dspsurf);
		dev_priv->init_screen_start = ctx->dspsurf;
		dev_priv->init_screen_offset = ctx->dsplinoff;
		dev_priv->init_screen_size = ctx->dspsize;
		dev_priv->init_screen_stride = ctx->dspstride;
		init_flag = 0;
	}

	switch (fb_bpp) {
	case 8:
		ctx->dspcntr = DISPPLANE_8BPP;
		break;
	case 16:
		if (fb_depth == 15)
			ctx->dspcntr = DISPPLANE_15_16BPP;
		else
			ctx->dspcntr = DISPPLANE_16BPP;
		break;
	case 24:
	case 32:
		ctx->dspcntr = DISPPLANE_32BPP_NO_ALPHA;
		break;
	default:
		DRM_ERROR("Unknown color depth\n");
		mutex_unlock(&dsi_config->context_lock);
		return -EINVAL;
	}

	if (dsi_config->pipe == 2)
		ctx->dspcntr |= (0x2 << 24);

	/*
	 * Setup pipe configuration for different panels
	 * The formula recommended from hw team is as below:
	 * (htotal * 5ns * hdelay) >= 8000ns
	 * hdelay is the count of delayed HBLANK scan lines
	 * And the max hdelay is 4
	 * by programming of PIPE(A/C) CONF bit 28:27:
	 * 00 = 1 scan line, 01 = 2 scan line,
	 * 02 = 3 scan line, 03 = 4 scan line
	 */
	ctx->pipeconf &= ~(BIT27 | BIT28);

	hdelay = 8000/mode->crtc_htotal/5;
	if (8000%(mode->crtc_htotal*5) > 0)
		hdelay += 1;

	if (hdelay > 4) {
		DRM_ERROR("Do not support such panel setting yet\n");
		hdelay = 4; /* Use the max hdelay instead*/
	}

	ctx->pipeconf |= ((hdelay-1) << 27);

	mutex_unlock(&dsi_config->context_lock);
	return 0;
}

static int mdfld_crtc_mode_set(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	int pipe = psb_intel_crtc->pipe;
	int fp_reg = MRST_FPA0;
	int dpll_reg = MRST_DPLL_A;
	int dspcntr_reg = DSPACNTR;
	int pipeconf_reg = PIPEACONF;
	int htot_reg = HTOTAL_A;
	int hblank_reg = HBLANK_A;
	int hsync_reg = HSYNC_A;
	int vtot_reg = VTOTAL_A;
	int vblank_reg = VBLANK_A;
	int vsync_reg = VSYNC_A;
	int dspsize_reg = DSPASIZE;
	int dsppos_reg = DSPAPOS;
	int pipesrc_reg = PIPEASRC;
	u32 *pipeconf = &dev_priv->pipeconf;
	u32 *dspcntr = &dev_priv->dspcntr;
	int refclk = 0;
	int clk_n = 0, clk_p2 = 0, clk_byte = 1, clk = 0, m_conv = 0, clk_tmp = 0;
	struct mrst_clock_t clock;
	bool ok;
	u32 dpll = 0, fp = 0;
	/* One hot encoding for P1 = 8 */
	u32 p1_post = 0x40;
	bool is_crt = false, is_lvds = false, is_tv = false;
	bool is_mipi = false, is_mipi2 = false, is_hdmi = false;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct psb_intel_output *psb_intel_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	uint64_t scalingType = DRM_MODE_SCALE_CENTER;
	struct drm_encoder *encoder;
	struct drm_connector * connector;
	int timeout = 0;
	struct drm_encoder *mipi_encoder;

	struct mdfld_dsi_hw_context *ctx;

	PSB_DEBUG_ENTRY("pipe = 0x%x\n", pipe);

	if (pipe == 0)   //h8c7_cmd
		dsi_config = dev_priv->dsi_configs[0];
	else if (pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];
	ctx = &dsi_config->dsi_hw_context;


#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	/**
	 * MIPI panel mode setting
	 * NOTE: this path only works for TMD panel now. update it to
	 * support all MIPI panels later.
	 */
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
		(get_panel_type(dev, pipe) == TMD_6X10_VID) ||
		(get_panel_type(dev, pipe) == H8C7_VID) ||
		(get_panel_type(dev, pipe) == GI_SONY_VID) ||
		/* SC1 setting */
		(get_panel_type(dev, pipe) == AUO_SC1_VID))) {
		if (pipe == 0)
			dsi_config = dev_priv->dsi_configs[0];
		else if (pipe == 2)
			dsi_config = dev_priv->dsi_configs[1];
		else
			return -EINVAL;
		return mdfld_crtc_dsi_mode_set(crtc, dsi_config, mode,
				adjusted_mode, x, y, old_fb);
	 }
#endif
#endif

	switch (pipe) {
	case 0:
		break;
	case 1:
		fp_reg = FPB0;
		dpll_reg = DPLL_B;
		dspcntr_reg = DSPBCNTR;
		pipeconf_reg = PIPEBCONF;
		htot_reg = HTOTAL_B;
		hblank_reg = HBLANK_B;
		hsync_reg = HSYNC_B;
		vtot_reg = VTOTAL_B;
		vblank_reg = VBLANK_B;
		vsync_reg = VSYNC_B;
		dspsize_reg = DSPBSIZE;
		dsppos_reg = DSPBPOS;
		pipesrc_reg = PIPEBSRC;
		pipeconf = &dev_priv->pipeconf1;
		dspcntr = &dev_priv->dspcntr1;
		if (IS_MDFLD(dev)) {
			fp_reg = MDFLD_DPLL_DIV0;
			dpll_reg = MDFLD_DPLL_B;
		}
		break;
	case 2:
		dpll_reg = MRST_DPLL_A;
		dspcntr_reg = DSPCCNTR;
		pipeconf_reg = PIPECCONF;
		htot_reg = HTOTAL_C;
		hblank_reg = HBLANK_C;
		hsync_reg = HSYNC_C;
		vtot_reg = VTOTAL_C;
		vblank_reg = VBLANK_C;
		vsync_reg = VSYNC_C;
		dspsize_reg = DSPCSIZE;
		dsppos_reg = DSPCPOS;
		pipesrc_reg = PIPECSRC;
		pipeconf = &dev_priv->pipeconf2;
		dspcntr = &dev_priv->dspcntr2;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return 0;
	}

	PSB_DEBUG_ENTRY("adjusted_hdisplay = %d\n",
		 adjusted_mode->hdisplay);
	PSB_DEBUG_ENTRY("adjusted_vdisplay = %d\n",
		 adjusted_mode->vdisplay);
	PSB_DEBUG_ENTRY("adjusted_hsync_start = %d\n",
		 adjusted_mode->hsync_start);
	PSB_DEBUG_ENTRY("adjusted_hsync_end = %d\n",
		 adjusted_mode->hsync_end);
	PSB_DEBUG_ENTRY("adjusted_htotal = %d\n",
		 adjusted_mode->htotal);
	PSB_DEBUG_ENTRY("adjusted_vsync_start = %d\n",
		 adjusted_mode->vsync_start);
	PSB_DEBUG_ENTRY("adjusted_vsync_end = %d\n",
		 adjusted_mode->vsync_end);
	PSB_DEBUG_ENTRY("adjusted_vtotal = %d\n",
		 adjusted_mode->vtotal);
	PSB_DEBUG_ENTRY("adjusted_clock = %d\n",
		 adjusted_mode->clock);
	PSB_DEBUG_ENTRY("adjusted_refresh = %d\n",
		 adjusted_mode->vrefresh);
	PSB_DEBUG_ENTRY("hdisplay = %d\n",
		 mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay = %d\n",
		 mode->vdisplay);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return 0;

	memcpy(&psb_intel_crtc->saved_mode, mode, sizeof(struct drm_display_mode));
	memcpy(&psb_intel_crtc->saved_adjusted_mode, adjusted_mode, sizeof(struct drm_display_mode));

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		if(!connector)
			continue;

		encoder = connector->encoder;

		if(!encoder)
			continue;

		if (encoder->crtc != crtc)
			continue;

		psb_intel_output = to_psb_intel_output(connector);

		PSB_DEBUG_ENTRY("output->type = 0x%x \n", psb_intel_output->type);

		switch (psb_intel_output->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		case INTEL_OUTPUT_ANALOG:
			is_crt = true;
			break;
		case INTEL_OUTPUT_MIPI:
			is_mipi = true;
			mipi_encoder = encoder;
			break;
		case INTEL_OUTPUT_MIPI2:
			is_mipi2 = true;
			break;
		case INTEL_OUTPUT_HDMI:
			is_hdmi = true;
			break;
		}
	}

	/* Disable the VGA plane that we never use */
	REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);

	/* Disable the panel fitter if it was on our pipe */
	if (psb_intel_panel_fitter_pipe(dev) == pipe)
		REG_WRITE(PFIT_CONTROL, 0);

	if (psb_intel_output)
		drm_connector_property_get_value(&psb_intel_output->base,
			dev->mode_config.scaling_mode_property, &scalingType);

	if ((scalingType == DRM_MODE_SCALE_NO_SCALE)
		||(scalingType < DRM_MODE_SCALE_NO_SCALE))
	{
		psb_intel_crtc->scaling_type = scalingType;
		psb_intel_crtc->scaling_step = 0;
	}
	else
	{
		psb_intel_crtc->scaling_step = scalingType;
		if (drm_connector_property_set_value(&psb_intel_output->base,dev->mode_config.scaling_mode_property,psb_intel_crtc->scaling_type))
			return -EINVAL;
	}

	if (scalingType == DRM_MODE_SCALE_NO_SCALE) {
		/*Moorestown doesn't have register support for centering so we need to
		  mess with the h/vblank and h/vsync start and ends to get centering*/
		int offsetX = 0, offsetY = 0;

		offsetX = (adjusted_mode->crtc_hdisplay - mode->crtc_hdisplay) / 2;
		offsetY = (adjusted_mode->crtc_vdisplay - mode->crtc_vdisplay) / 2;

		REG_WRITE(htot_reg, (mode->crtc_hdisplay - 1) |
			((adjusted_mode->crtc_htotal - 1) << 16));
		REG_WRITE(vtot_reg, (mode->crtc_vdisplay - 1) |
			((adjusted_mode->crtc_vtotal - 1) << 16));
		REG_WRITE(hblank_reg, (adjusted_mode->crtc_hblank_start - offsetX - 1) |
			((adjusted_mode->crtc_hblank_end - offsetX - 1) << 16));
		REG_WRITE(hsync_reg, (adjusted_mode->crtc_hsync_start - offsetX - 1) |
			((adjusted_mode->crtc_hsync_end - offsetX - 1) << 16));
		REG_WRITE(vblank_reg, (adjusted_mode->crtc_vblank_start - offsetY - 1) |
			((adjusted_mode->crtc_vblank_end - offsetY - 1) << 16));
		REG_WRITE(vsync_reg, (adjusted_mode->crtc_vsync_start - offsetY - 1) |
			((adjusted_mode->crtc_vsync_end - offsetY - 1) << 16));
	} else {
		REG_WRITE(htot_reg, (adjusted_mode->crtc_hdisplay - 1) |
			((adjusted_mode->crtc_htotal - 1) << 16));
		REG_WRITE(vtot_reg, (adjusted_mode->crtc_vdisplay - 1) |
			((adjusted_mode->crtc_vtotal - 1) << 16));
		REG_WRITE(hblank_reg, (adjusted_mode->crtc_hblank_start - 1) |
			((adjusted_mode->crtc_hblank_end - 1) << 16));
		REG_WRITE(hsync_reg, (adjusted_mode->crtc_hsync_start - 1) |
			((adjusted_mode->crtc_hsync_end - 1) << 16));
		REG_WRITE(vblank_reg, (adjusted_mode->crtc_vblank_start - 1) |
			((adjusted_mode->crtc_vblank_end - 1) << 16));
		REG_WRITE(vsync_reg, (adjusted_mode->crtc_vsync_start - 1) |
			((adjusted_mode->crtc_vsync_end - 1) << 16));
	}

	/* Flush the plane changes */
	{
		struct drm_crtc_helper_funcs *crtc_funcs =
		    crtc->helper_private;
		crtc_funcs->mode_set_base(crtc, x, y, old_fb);
	}

	/* setup pipeconf */
	*pipeconf = PIPEACONF_ENABLE;

	/* Set up the display plane register */
 	*dspcntr = REG_READ(dspcntr_reg);
	*dspcntr |= pipe << DISPPLANE_SEL_PIPE_POS;
	*dspcntr |= DISPLAY_PLANE_ENABLE;

	if (is_mipi2)
	{
		goto mrst_crtc_mode_set_exit;
	}

	clk = adjusted_mode->clock;

	if (is_hdmi) {
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) || (dev_priv->ksel == KSEL_BYPASS_19))
		{
			refclk = 19200;

			if (is_mipi || is_mipi2)
			{
				clk_n = 1, clk_p2 = 8;
			} else if (is_hdmi) {
				clk_n = 1, clk_p2 = 10;
			}
		} else if (dev_priv->ksel == KSEL_BYPASS_25) {
			refclk = 25000;

			if (is_mipi || is_mipi2)
			{
				clk_n = 1, clk_p2 = 8;
			} else if (is_hdmi) {
				clk_n = 1, clk_p2 = 10;
			}
		} else if (dev_priv->ksel == KSEL_CRYSTAL_38) {
			refclk = 38400;

			if (is_mipi || is_mipi2) {
				clk_n = 1, clk_p2 = 8;
			} else if (is_hdmi) {
				clk_n = 2, clk_p2 = 10;
			}
		} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) && (dev_priv->core_freq == 166)) {
			refclk = 83000;

			if (is_mipi || is_mipi2)
			{
				clk_n = 4, clk_p2 = 8;
			} else if (is_hdmi) {
				clk_n = 4, clk_p2 = 10;
			}
		} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			   (dev_priv->core_freq == 100 || dev_priv->core_freq == 200)) {
			refclk = 100000;
			if (is_mipi || is_mipi2)
			{
				clk_n = 4, clk_p2 = 8;
			} else if (is_hdmi) {
				clk_n = 4, clk_p2 = 10;
			}
		}

		if (is_mipi)
			clk_byte = dev_priv->bpp / 8;
		else if (is_mipi2)
			clk_byte = dev_priv->bpp2 / 8;

		clk_tmp = clk * clk_n * clk_p2 * clk_byte;

		PSB_DEBUG_ENTRY("clk = %d, clk_n = %d, clk_p2 = %d. \n", clk, clk_n, clk_p2);
		PSB_DEBUG_ENTRY("adjusted_mode->clock = %d, clk_tmp = %d. \n", adjusted_mode->clock, clk_tmp);

		ok = mdfldFindBestPLL(crtc, clk_tmp, refclk, &clock);
		dev_priv->tmds_clock_khz = clock.dot / (clk_n * clk_p2 * clk_byte);

		if (!ok) {
			DRM_ERROR
			    ("mdfldFindBestPLL fail in mdfld_crtc_mode_set. \n");
		} else {
			m_conv = mdfld_m_converts[(clock.m - MDFLD_M_MIN)];

			PSB_DEBUG_ENTRY("dot clock = %d,"
				 "m = %d, p1 = %d, m_conv = %d. \n", clock.dot, clock.m,
				 clock.p1, m_conv);
		}

		dpll = REG_READ(dpll_reg);

		if (dpll & DPLL_VCO_ENABLE) {
			dpll &= ~DPLL_VCO_ENABLE;
			REG_WRITE(dpll_reg, dpll);
			REG_READ(dpll_reg);

			/* FIXME check the DPLL lock bit PIPEACONF[29] */
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(500);

			/* reset M1, N1 & P1 */
			REG_WRITE(fp_reg, 0);
			dpll &= ~MDFLD_P1_MASK;
			REG_WRITE(dpll_reg, dpll);
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(500);
		}

		/* When ungating power of DPLL, needs to wait 0.5us before enable the VCO */
		if (dpll & MDFLD_PWR_GATE_EN) {
			dpll &= ~MDFLD_PWR_GATE_EN;
			REG_WRITE(dpll_reg, dpll);
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(500);
		}

		dpll = 0;

#if 0 /* FIXME revisit later */
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) || (dev_priv->ksel == KSEL_BYPASS_19) || (dev_priv->ksel == KSEL_BYPASS_25)) {
			dpll &= ~MDFLD_INPUT_REF_SEL;
		} else if (dev_priv->ksel == KSEL_BYPASS_83_100) {
			dpll |= MDFLD_INPUT_REF_SEL;
		}
#endif /* FIXME revisit later */

		if (is_hdmi)
			dpll |= MDFLD_VCO_SEL;

		fp = (clk_n / 2) << 16;
		fp |= m_conv;

		/* compute bitmask from p1 value */
		dpll |= (1 << (clock.p1 - 2)) << 17;

	} else {
		if (pipe == 2)
			dsi_config = dev_priv->dsi_configs[1];
		else
			dsi_config = dev_priv->dsi_configs[0];

		/*caculate pll*/
		mdfld_crtc_dsi_pll_calc(crtc, dsi_config, dev,
				 &dpll,
				 &fp,
				 adjusted_mode);

	}

	/*set pll*/
	REG_WRITE(dpll_reg, 0);
	REG_WRITE(fp_reg, 0);
	REG_WRITE(fp_reg, fp);
	REG_WRITE(dpll_reg, dpll & (~BIT30));
	/* FIXME_MDFLD PO - change 500 to 1 after PO */
	udelay(2);
	dpll = REG_READ(dpll_reg);
	REG_WRITE(dpll_reg, dpll | BIT31);

	printk(KERN_ALERT "dpll = 0x%x, fb = 0x%x", REG_READ(dpll_reg), REG_READ(fp_reg));

	/* wait for DSI PLL to lock */
	while ((timeout < 20000) && !(REG_READ(pipeconf_reg) & PIPECONF_DSIPLL_LOCK)) {
		udelay(150);
		timeout ++;
	}

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	if (pipe == 0 && ctx != NULL) {
		ctx->dpll = dpll;
		ctx->fp = fp;
	}
#endif

	if (is_mipi) {
		if (get_panel_type(dev, pipe) == GI_SONY_CMD)
			mdfld_gi_sony_power_on(mipi_encoder);
		else if (get_panel_type(dev, pipe) == H8C7_CMD)
			mdfld_h8c7_cmd_power_on(mipi_encoder);

		goto mrst_crtc_mode_set_exit;
	}

	PSB_DEBUG_ENTRY("is_mipi = 0x%x\n", is_mipi);

	REG_WRITE(pipeconf_reg, *pipeconf);
	REG_READ(pipeconf_reg);

	REG_WRITE(dspcntr_reg, *dspcntr);
	psb_intel_wait_for_vblank(dev);

mrst_crtc_mode_set_exit:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

/* MDFLD_PLATFORM end */
