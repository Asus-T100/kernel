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

/* This enables setting backlights on with a delay at startup,
   should be removed after resolving issue with backlights going off
   after setting them on in initial mrst_dsi_set_power call */
#define AAVA_BACKLIGHT_HACK

#include <linux/version.h>
#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>

#include <asm/intel_scu_ipc.h>

#ifdef AAVA_BACKLIGHT_HACK
#include <linux/workqueue.h>
#endif				/* AAVA_BACKLIGHT_HACK */

#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"

/* Debug trace definitions */
#define DBG_LEVEL 0
#define AAVA_EV_0_5

#if (DBG_LEVEL > 0)
#define DBG_TRACE(format, args...) printk(KERN_ERR "%s: " format "\n", \
						__func__ , ## args)
#else
#define DBG_TRACE(format, args...)
#endif

#define DBG_ERR(format, args...) printk(KERN_ERR "%s: " format "\n", \
						__func__ , ## args)

#define BRIGHTNESS_MAX_LEVEL 100

#define DRM_MODE_ENCODER_MIPI   5

#define DBG_PRINTS 0

#define NEW_CRAP_SAMPLE_SETTINGS

#define VSIZE		480
#define HSIZE		864
#define HFP_DOTS	10
#define HBP_DOTS	10
#define HSYNC_DOTS	4
#define VFP_LINES	8
#define VBP_LINES	8
#define VSYNC_LINES	4

#define MIPI_LANES	2
#define MIPI_HACT	((HSIZE * 3) / MIPI_LANES)
#define MIPI_HFP	((HFP_DOTS * 3) / MIPI_LANES)
#define MIPI_HBP	((HBP_DOTS * 3) / MIPI_LANES)
#define MIPI_HSPAD	((HSYNC_DOTS * 3) / MIPI_LANES)
#define MIPI_VFP	VFP_LINES
#define MIPI_VSPAD	VSYNC_LINES
#define MIPI_VBP	VBP_LINES

#define DISP_HPIX		(HSIZE - 1)
#define DISP_VPIX		(VSIZE - 1)
#define DISP_HBLANK_START	DISP_HPIX
#define DISP_HBLANK_END		(DISP_HBLANK_START + HFP_DOTS + \
				 HSYNC_DOTS + HBP_DOTS - 1)
#define DISP_HSYNC_START	(DISP_HBLANK_START + HFP_DOTS - 1)
#define DISP_HSYNC_END		(DISP_HSYNC_START + HSYNC_DOTS - 1)
#define DISP_VBLANK_START	 DISP_VPIX
#define DISP_VBLANK_END		(DISP_VBLANK_START + VFP_LINES + \
				VSYNC_LINES + VBP_LINES - 1)
#define DISP_VSYNC_START	(DISP_VBLANK_START + VFP_LINES - 1)
#define DISP_VSYNC_END		(DISP_VSYNC_START + VSYNC_LINES - 1)

#define MAX_FIFO_WAIT_MS	100

#define MIPI_2XCLK_COUNT	0x04
#define BLC_POLARITY_NORMAL	0

static unsigned int dphy_reg = 0x0d0a7f06;
static unsigned int mipi_clock = 0x2;

#ifdef AAVA_BACKLIGHT_HACK
static void dsi_bl_work_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(bl_work, dsi_bl_work_handler);
#endif				/* AAVA_BACKLIGHT_HACK */

static int dsi_wait_hs_data_fifo(struct drm_device *dev)
{
	int fifo_wait_time = 0;

	while ((REG_READ(GEN_FIFO_STAT_REG) & HS_DATA_FIFO_FULL) ==
	       HS_DATA_FIFO_FULL) {
		if (fifo_wait_time == MAX_FIFO_WAIT_MS) {
			DBG_ERR("timeout");
			return -1;
		}
		udelay(1000);
		fifo_wait_time++;
	}
	return 0;
}

static int dsi_wait_hs_ctrl_fifo(struct drm_device *dev)
{
	int fifo_wait_time = 0;

	while ((REG_READ(GEN_FIFO_STAT_REG) & HS_CTRL_FIFO_FULL) ==
	       HS_CTRL_FIFO_FULL) {
		if (fifo_wait_time == MAX_FIFO_WAIT_MS) {
			DBG_ERR("timeout");
			return -1;
		}
		udelay(1000);
		fifo_wait_time++;
	}
	return 0;
}

static void dsi_set_backlight_state(int state)
{
#ifdef CONFIG_X86_MRST
	u8 addr[2], value[2];

	addr[0] = 0x2a;
	addr[1] = 0x28;

	if (state) {
		value[0] = 0xaa;
#ifdef AAVA_EV_0_5
		value[1] = 0x30;
#else
		value[1] = 0x60;
#endif
	} else {
		value[0] = 0x0;
		value[1] = 0x0;
	}

	intel_scu_ipc_iowrite8(addr[0], value[0]);
	intel_scu_ipc_iowrite8(addr[1], value[1]);
#endif
}

#ifdef AAVA_BACKLIGHT_HACK
static void dsi_bl_work_handler(struct work_struct *work)
{
	DBG_TRACE("");
	dsi_set_backlight_state(1);
}
#endif				/* AAVA_BACKLIGHT_HACK */

static void dsi_set_panel_reset_state(int state)
{
	if (state) {
		/*FIXME: if wrong */
		u8 addr, value;

#ifdef AAVA_EV_0_5
		addr = 0xe6;
		value = 0x01;
#else		 /*CDK*/
		    addr = 0xf4;

		if (intel_scu_ipc_ioread8(addr, &value)) {
			printk(KERN_ERR
			       "panel_reset_on: failed to read pmic reg 0xf4!\n");
			return;
		}

		value &= 0xbf;
#endif				/*AAVA_EV_0_5 */
#ifdef CONFIG_X86_MRST
		if (intel_scu_ipc_iowrite8(addr, value)) {
			printk(KERN_ERR
			       "panel_reset_on: failed to write pmic reg 0xf4!\n");
			return;
		}
#endif
		/* Minimum active time to trigger reset is 10us */
		udelay(10);
	} else {

		u8 addr, value;

#ifdef AAVA_EV_0_5
		addr = 0xe6;
		value = 0x09;
#else
		addr = 0xf4;

		if (intel_scu_ipc_ioread8(addr, &value)) {
			printk(KERN_ERR
			       "panel_reset_off: failed to read pmic reg 0xf4!\n");
			return;
		}

		value |= 0x40;
#endif
#ifdef CONFIG_X86_MRST
		if (intel_scu_ipc_iowrite8(addr, value)) {
			printk(KERN_WARNING
			       "panel_reset_off: failed to write pmic reg 0xe6!\n");
		}
#endif
		/* Maximum startup time from reset is 120ms */
		msleep(120);
	}
}

static void dsi_init_panel(struct drm_device *dev)
{
	DBG_TRACE("");

	/* Flip page order to have correct image orientation */
	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
	REG_WRITE(0xb068, 0x00008036);
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000229);

	/* Write protection key to allow DM bit setting */
	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
	REG_WRITE(0xb068, 0x005a5af1);
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000329);

	/* Set DM bit to enable video mode */
	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
	REG_WRITE(0xb068, 0x000100f7);
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000329);

	/* Write protection keys to allow TCON setting */
	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
	REG_WRITE(0xb068, 0x005a5af0);
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000329);

	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
	REG_WRITE(0xb068, 0x005a5afc);
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000329);

	/* Write TCON setting */
	if (dsi_wait_hs_data_fifo(dev) < 0)
		return;
#if 0				/* Suggested by TPO, doesn't work */
	REG_WRITE(0xb068, 0x110000b7);
	REG_WRITE(0xb068, 0x00000044);
#else
	REG_WRITE(0xb068, 0x770000b7);
	REG_WRITE(0xb068, 0x00000044);
#endif
	if (dsi_wait_hs_ctrl_fifo(dev) < 0)
		return;
	REG_WRITE(0xb070, 0x00000529);
}

static void dsi_set_ptarget_state(struct drm_device *dev, int state)
{
	u32 pp_sts_reg;

	DBG_TRACE("%d", state);

	if (state) {
		REG_WRITE(PP_CONTROL, (REG_READ(PP_CONTROL) | POWER_TARGET_ON));
		do {
			pp_sts_reg = REG_READ(PP_STATUS);
		} while ((pp_sts_reg & (PP_ON | PP_READY)) == PP_READY);
	} else {
		REG_WRITE(PP_CONTROL,
			  (REG_READ(PP_CONTROL) & ~POWER_TARGET_ON));
		do {
			pp_sts_reg = REG_READ(PP_STATUS);
		} while (pp_sts_reg & PP_ON);
	}
}

static void dsi_send_turn_on_packet(struct drm_device *dev)
{
	DBG_TRACE("");

	REG_WRITE(DPI_CONTROL_REG, DPI_TURN_ON);

	/*
	 * Short delay to wait that display turns on
	 *
	 * msleep < 20ms can sleep for up to 20ms;
	 * see Documentation/timers/timers-howto.txt
	*/
	msleep(20);
}

static void dsi_send_shutdown_packet(struct drm_device *dev)
{
	DBG_TRACE("");

	REG_WRITE(DPI_CONTROL_REG, DPI_SHUT_DOWN);
}

static void dsi_set_pipe_plane_enable_state(struct drm_device *dev, int state)
{
	u32 temp_reg;

	DBG_TRACE("%d", state);

	if (state) {
		/* Enable pipe */
		temp_reg = REG_READ(PIPEACONF);
		temp_reg |= (PIPEACONF_ENABLE);
		REG_WRITE(PIPEACONF, temp_reg);
		temp_reg = REG_READ(PIPEACONF);

		/* Wait for 20ms for the pipe enable to take effect. */
		msleep(20);

		/* Enable plane */
		temp_reg = REG_READ(DSPACNTR);
		temp_reg |= (DISPLAY_PLANE_ENABLE);
		REG_WRITE(DSPACNTR, temp_reg);
		temp_reg = REG_READ(DSPACNTR);

		/* Flush plane change by read/write/read of BASE reg */
		temp_reg = REG_READ(MRST_DSPABASE);
		REG_WRITE(MRST_DSPABASE, temp_reg);
		temp_reg = REG_READ(MRST_DSPABASE);

		/* Wait for 20ms for the plane enable to take effect. */
		msleep(20);
	} else {
		/* Disable plane */
		temp_reg = REG_READ(DSPACNTR);
		temp_reg &= ~(DISPLAY_PLANE_ENABLE);
		REG_WRITE(DSPACNTR, temp_reg);
		temp_reg = REG_READ(DSPACNTR);

		/* Flush plane change by read/write/read of BASE reg */
		temp_reg = REG_READ(MRST_DSPABASE);
		REG_WRITE(MRST_DSPABASE, temp_reg);
		temp_reg = REG_READ(MRST_DSPABASE);

		/* Wait for 20ms for the plane disable to take effect. */
		msleep(20);

		/* Disable pipe */
		temp_reg = REG_READ(PIPEACONF);
		temp_reg &= ~(PIPEACONF_ENABLE);
		REG_WRITE(PIPEACONF, temp_reg);
		temp_reg = REG_READ(PIPEACONF);

		/* Wait for 20ms for the pipe disable to take effect. */
		msleep(20);
	}
}

static void dsi_set_device_ready_state(struct drm_device *dev, int state)
{
	DBG_TRACE("%d", state);

	if (state)
		REG_WRITE(DEVICE_READY_REG, 0x00000001);
	else
		REG_WRITE(DEVICE_READY_REG, 0x00000000);
}

static void dsi_configure_mipi_block(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 color_format = (RGB_888_FMT << FMT_DPI_POS);
	u32 res = 0;

	DBG_TRACE("");

	/* MIPI clock ratio 1:1 */
	/* REG_WRITE(MIPI_CONTROL_REG, 0x00000018); */
	/* REG_WRITE(0xb080, 0x0b061a02); */

	/* MIPI clock ratio 2:1 */
	/* REG_WRITE(MIPI_CONTROL_REG, 0x00000019); */
	/* REG_WRITE(0xb080, 0x3f1f1c04); */

	/* MIPI clock ratio 3:1 */
	/* REG_WRITE(MIPI_CONTROL_REG, 0x0000001a); */
	/* REG_WRITE(0xb080, 0x091f7f08); */

	/* MIPI clock ratio 4:1 */
	REG_WRITE(MIPI_CONTROL_REG, (0x00000018 | mipi_clock));
	REG_WRITE(0xb080, dphy_reg);

	/* Enable all interrupts */
	REG_WRITE(INTR_EN_REG, 0xffffffff);

	REG_WRITE(TURN_AROUND_TIMEOUT_REG, 0x0000000A);
	REG_WRITE(DEVICE_RESET_REG, 0x000000ff);
	REG_WRITE(INIT_COUNT_REG, 0x00000fff);
	REG_WRITE(HS_TX_TIMEOUT_REG, 0x90000);
	REG_WRITE(LP_RX_TIMEOUT_REG, 0xffff);
	REG_WRITE(HIGH_LOW_SWITCH_COUNT_REG, 0x46);
	REG_WRITE(EOT_DISABLE_REG, 0x00000000);
	REG_WRITE(LP_BYTECLK_REG, 0x00000004);

	REG_WRITE(VIDEO_FMT_REG, dev_priv->videoModeFormat);

	REG_WRITE(DSI_FUNC_PRG_REG, (dev_priv->laneCount | color_format));

	res = dev_priv->HactiveArea | (dev_priv->VactiveArea << RES_V_POS);
	REG_WRITE(DPI_RESOLUTION_REG, res);

	REG_WRITE(VERT_SYNC_PAD_COUNT_REG, dev_priv->VsyncWidth);
	REG_WRITE(VERT_BACK_PORCH_COUNT_REG, dev_priv->VbackPorch);
	REG_WRITE(VERT_FRONT_PORCH_COUNT_REG, dev_priv->VfrontPorch);

	REG_WRITE(HORIZ_SYNC_PAD_COUNT_REG, dev_priv->HsyncWidth);
	REG_WRITE(HORIZ_BACK_PORCH_COUNT_REG, dev_priv->HbackPorch);
	REG_WRITE(HORIZ_FRONT_PORCH_COUNT_REG, dev_priv->HfrontPorch);
	REG_WRITE(HORIZ_ACTIVE_AREA_COUNT_REG, MIPI_HACT);

	/* Enable MIPI Port */
	REG_WRITE(MIPI, MIPI_PORT_EN);
}

static void dsi_configure_down(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DBG_TRACE("");

	if (!dev_priv->dpi_panel_on) {
		DBG_TRACE("already off");
		return;
	}

	/* Disable backlight */
	dsi_set_backlight_state(0);

	/* Disable pipe and plane */
	dsi_set_pipe_plane_enable_state(dev, 0);

	/* Disable PTARGET */
	dsi_set_ptarget_state(dev, 0);

	/* Send shutdown command, can only be sent if
	 * interface is configured
	 */
	if (dev_priv->dsi_device_ready)
		dsi_send_shutdown_packet(dev);

	/* Clear device ready state */
	dsi_set_device_ready_state(dev, 0);

	/* Set panel to reset */
	dsi_set_panel_reset_state(1);

	dev_priv->dpi_panel_on = false;
}

static void dsi_configure_up(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DBG_TRACE("");

	if (dev_priv->dpi_panel_on) {
		DBG_TRACE("already on");
		return;
	}

	/* Get panel from reset */
	dsi_set_panel_reset_state(0);

	/* Set device ready state */
	dsi_set_device_ready_state(dev, 1);

	/* Send turn on command */
	dsi_send_turn_on_packet(dev);

	/* Enable PTARGET */
	dsi_set_ptarget_state(dev, 1);

	/* Initialize panel */
	dsi_init_panel(dev);

	/* Enable plane and pipe */
	dsi_set_pipe_plane_enable_state(dev, 1);

	/* Enable backlight */
	dsi_set_backlight_state(1);

	dev_priv->dpi_panel_on = true;
}

static void dsi_init_drv_ic(struct drm_device *dev)
{
	DBG_TRACE("");
}

static void dsi_schedule_work(struct drm_device *dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DBG_TRACE("");

	schedule_work(&dev_priv->dsi_work);
}

static void dsi_work_handler(struct work_struct *work)
{
	struct drm_psb_private *dev_priv = container_of(work,
							struct drm_psb_private,
							dsi_work);

	DBG_TRACE("");

	dsi_configure_up(dev_priv->dev);
}

static void dsi_init_mipi_config(DRM_DRIVER_PRIVATE_T *dev_priv)
{
	DBG_TRACE("");

	/* Fixed values for TPO display */
	dev_priv->pixelClock = 33264;
	dev_priv->HsyncWidth = MIPI_HSPAD;
	dev_priv->HbackPorch = MIPI_HBP;
	dev_priv->HfrontPorch = MIPI_HFP;
	dev_priv->HactiveArea = HSIZE;
	dev_priv->VsyncWidth = MIPI_VSPAD;
	dev_priv->VbackPorch = MIPI_VBP;
	dev_priv->VfrontPorch = MIPI_VFP;
	dev_priv->VactiveArea = VSIZE;
	dev_priv->bpp = 24;

	/* video mode */
	dev_priv->dpi = true;

	/* Set this true since firmware or kboot has enabled display */
	dev_priv->dpi_panel_on = true;

	/* Set this false to ensure proper initial configuration */
	dev_priv->dsi_device_ready = false;

	/* 2 lanes */
	dev_priv->laneCount = MIPI_LANES;

	/* Burst mode */
	dev_priv->videoModeFormat = BURST_MODE;

	dev_priv->init_drvIC = dsi_init_drv_ic;
	dev_priv->dsi_prePowerState = dsi_configure_down;
	dev_priv->dsi_postPowerState = dsi_schedule_work;
}

static struct drm_display_mode *dsi_get_fixed_display_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	/* MiKo, fixed mode for TPO display
	   Note: Using defined values for easier match with ITP scripts
	   and adding 1 since psb_intel_display.c decreases by 1
	 */
	mode->hdisplay = (DISP_HPIX + 1);
	mode->vdisplay = (DISP_VPIX + 1);
	mode->hsync_start = (DISP_HSYNC_START + 1);
	mode->hsync_end = (DISP_HSYNC_END + 1);
	mode->htotal = (DISP_HBLANK_END + 1);
	mode->vsync_start = (DISP_VSYNC_START + 1);
	mode->vsync_end = (DISP_VSYNC_END + 1);
	mode->vtotal = (DISP_VBLANK_END + 1);
	mode->clock = 33264;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

/* Encoder funcs */
static void dsi_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	uint64_t scale_mode = DRM_MODE_SCALE_FULLSCREEN;

	DBG_TRACE("");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {
		DBG_ERR("OSPM_DISPLAY_ISLAND OSPM_UHB_FORCE_POWER_ON failed");
		return;
	}

	/* Sleep to ensure that the graphics engine is ready
	 * since its mode_set is called before ours
	 */
	msleep(100);

	/* Only one mode is supported,
	 * so configure only if not yet configured
	 */
	if (!dev_priv->dsi_device_ready) {
		drm_connector_property_get_value(&enc_to_psb_intel_output
						 (encoder)->base,
						 dev->
				mode_config.scaling_mode_property,
						 &scale_mode);
		if (scale_mode == DRM_MODE_SCALE_CENTER)
			REG_WRITE(PFIT_CONTROL, 0);
		else if (scale_mode == DRM_MODE_SCALE_FULLSCREEN)
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
		else {
			DBG_ERR("unsupported scaling");
			REG_WRITE(PFIT_CONTROL, 0);
		}
		dsi_configure_mipi_block(dev);
		dev_priv->dsi_device_ready = true;
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void dsi_encoder_prepare(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;

	DBG_TRACE("");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {
		DBG_ERR("OSPM_DISPLAY_ISLAND OSPM_UHB_FORCE_POWER_ON failed");
		return;
	}

	dsi_configure_down(dev);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void dsi_encoder_commit(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DBG_TRACE("");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {
		DBG_ERR("OSPM_DISPLAY_ISLAND OSPM_UHB_FORCE_POWER_ON failed");
		return;
	}

	if (!work_pending(&dev_priv->dsi_work))
		dsi_configure_up(dev);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DBG_TRACE("%s", ((mode == DRM_MODE_DPMS_ON) ? "ON" : "OFF"));

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {
		DBG_ERR("OSPM_DISPLAY_ISLAND OSPM_UHB_FORCE_POWER_ON failed");
		return;
	}

	if (mode == DRM_MODE_DPMS_ON) {
		if (!work_pending(&dev_priv->dsi_work))
			dsi_configure_up(dev);
	} else
		dsi_configure_down(dev);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/* Connector funcs */
static enum drm_connector_status dsi_connector_detect(struct drm_connector
						      *connector)
{
	DBG_TRACE("");
	return connector_status_connected;
}

static int dsi_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *psb_output = to_psb_intel_output(connector);
	struct psb_intel_mode_device *mode_dev = psb_output->mode_dev;
	struct drm_display_mode *mode;

	DBG_TRACE("");

	/* Didn't get an DDB, so
	 * Set wide sync ranges so we get all modes
	 * handed to valid_mode for checking
	 */
	connector->display_info.min_vfreq = 0;
	connector->display_info.max_vfreq = 200;
	connector->display_info.min_hfreq = 0;
	connector->display_info.max_hfreq = 200;

	if (mode_dev->panel_fixed_mode != NULL) {
		mode = drm_mode_duplicate(dev, mode_dev->panel_fixed_mode);
		drm_mode_probed_add(connector, mode);
		return 1;
	}
	return 0;
}

static void mrst_dsi_save(struct drm_connector *connector)
{
	DBG_TRACE("");
}

static void mrst_dsi_restore(struct drm_connector *connector)
{
	DBG_TRACE("");
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.dpms = dsi_encoder_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = dsi_encoder_prepare,
	.mode_set = dsi_encoder_mode_set,
	.commit = dsi_encoder_commit,
};

static const struct drm_connector_helper_funcs
	mrst_dsi_connector_helper_funcs = {
	.get_modes = dsi_connector_get_modes,
	.mode_valid = psb_intel_lvds_mode_valid,
	.best_encoder = psb_intel_best_encoder,
};

static const struct drm_connector_funcs connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.save = mrst_dsi_save,
	.restore = mrst_dsi_restore,
	.detect = dsi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = psb_intel_lvds_set_property,
	.destroy = psb_intel_lvds_destroy,
};

void aava_koski_dsi_init(struct drm_device *dev,
			 struct psb_intel_mode_device *mode_dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	/* panel_reset(); */

#ifdef AAVA_BACKLIGHT_HACK
	schedule_delayed_work(&bl_work, 2 * HZ);
#endif				/* AAVA_BACKLIGHT_HACK */

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	drm_connector_init(dev,
			   &psb_intel_output->base,
			   &connector_funcs, DRM_MODE_CONNECTOR_MIPI);

	drm_encoder_init(dev,
			 &psb_intel_output->enc,
			 &psb_intel_lvds_enc_funcs, DRM_MODE_ENCODER_MIPI);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type = INTEL_OUTPUT_MIPI;

	drm_encoder_helper_add(encoder, &encoder_helper_funcs);
	drm_connector_helper_add(connector, &mrst_dsi_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_connector_attach_property(connector,
				      dev->mode_config.scaling_mode_property,
				      DRM_MODE_SCALE_FULLSCREEN);
	drm_connector_attach_property(connector,
				      dev_priv->backlight_property,
				      BRIGHTNESS_MAX_LEVEL);

	mode_dev->panel_wants_dither = false;

	dsi_init_mipi_config(dev_priv);

	/* No config phase */
	dev_priv->config_phase = false;

	/* Get the fixed mode */
	mode_dev->panel_fixed_mode = dsi_get_fixed_display_mode();
	if (mode_dev->panel_fixed_mode)
		mode_dev->panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
	else {
		DBG_ERR("Fixed mode not available!\n");
		goto failed_find;
	}
	dev_priv->dsi_plane_pipe_control = true;
	drm_sysfs_connector_add(connector);

	/* Initialize work queue */
	INIT_WORK(&dev_priv->dsi_work, dsi_work_handler);

	return;

 failed_find:
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	kfree(connector);
}

/*EXPORT_SYMBOL_GPL(aava_koski_dsi_init); */
