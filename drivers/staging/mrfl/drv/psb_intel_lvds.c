/*
 * Copyright © 2006-2007 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *	Dave Airlie <airlied@linux.ie>
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
/* #include <drm/drm_crtc.h> */
/* #include <drm/drm_edid.h> */
#include <drm/drmP.h>

#include "psb_intel_bios.h"
#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"
#include <linux/pm_runtime.h>

/* MRST defines start */
uint8_t blc_freq;
uint8_t blc_minbrightness;
uint8_t blc_i2caddr;
uint8_t blc_brightnesscmd;
int lvds_backlight;		/* restore backlight to this value */

u32 CoreClock;
u32 PWMControlRegFreq;

/**
 * LVDS I2C backlight control macros
 */
#define BRIGHTNESS_MAX_LEVEL 100
#define BRIGHTNESS_MASK 0xFF
#define BLC_I2C_TYPE	0x01
#define BLC_PWM_TYPT	0x02

#define BLC_POLARITY_NORMAL 0
#define BLC_POLARITY_INVERSE 1

#define PSB_BLC_MAX_PWM_REG_FREQ       (0xFFFE)
#define PSB_BLC_MIN_PWM_REG_FREQ	(0x2)
#define PSB_BLC_PWM_PRECISION_FACTOR	(10)
#define PSB_BACKLIGHT_PWM_CTL_SHIFT	(16)
#define PSB_BACKLIGHT_PWM_POLARITY_BIT_CLEAR (0xFFFE)

struct psb_intel_lvds_priv {
	/**
	 * Saved LVDO output states
	 */
	uint32_t savePP_ON;
	uint32_t savePP_OFF;
	uint32_t saveLVDS;
	uint32_t savePP_CONTROL;
	uint32_t savePP_CYCLE;
	uint32_t savePFIT_CONTROL;
	uint32_t savePFIT_PGM_RATIOS;
	uint32_t saveBLC_PWM_CTL;
};

/* MRST defines end */

/**
 * Returns the maximum level of the backlight duty cycle field.
 */
static u32 psb_intel_lvds_get_max_backlight(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	u32 retVal;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		retVal = ((REG_READ(BLC_PWM_CTL) &
			   BACKLIGHT_MODULATION_FREQ_MASK) >>
			  BACKLIGHT_MODULATION_FREQ_SHIFT) * 2;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else
		retVal = ((dev_priv->saveBLC_PWM_CTL &
			   BACKLIGHT_MODULATION_FREQ_MASK) >>
			  BACKLIGHT_MODULATION_FREQ_SHIFT) * 2;

	return retVal;
}

/**
 * Set LVDS backlight level by I2C command
 */
static int psb_lvds_i2c_set_brightness(struct drm_device *dev,
				       unsigned int level)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	struct psb_intel_i2c_chan *lvds_i2c_bus = dev_priv->lvds_i2c_bus;
	u8 out_buf[2];
	unsigned int blc_i2c_brightness;

	struct i2c_msg msgs[] = {
		{
		 .addr = lvds_i2c_bus->slave_addr,
		 .flags = 0,
		 .len = 2,
		 .buf = out_buf,
		 }
	};

	blc_i2c_brightness = BRIGHTNESS_MASK & ((unsigned int)level *
						BRIGHTNESS_MASK /
						BRIGHTNESS_MAX_LEVEL);

	if (dev_priv->lvds_bl->pol == BLC_POLARITY_INVERSE)
		blc_i2c_brightness = BRIGHTNESS_MASK - blc_i2c_brightness;

	out_buf[0] = dev_priv->lvds_bl->brightnesscmd;
	out_buf[1] = (u8) blc_i2c_brightness;

	if (i2c_transfer(&lvds_i2c_bus->adapter, msgs, 1) == 1) {
		DRM_DEBUG("I2C set brightness.(command, value) (%d, %d)\n",
			  blc_brightnesscmd, blc_i2c_brightness);
		return 0;
	}

	DRM_ERROR("I2C transfer error\n");
	return -1;
}

static int psb_lvds_pwm_set_brightness(struct drm_device *dev, int level)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	u32 max_pwm_blc;
	u32 blc_pwm_duty_cycle;

	max_pwm_blc = psb_intel_lvds_get_max_backlight(dev);

	/*BLC_PWM_CTL Should be initiated while backlight device init */
	BUG_ON((max_pwm_blc & PSB_BLC_MAX_PWM_REG_FREQ) == 0);

	blc_pwm_duty_cycle = level * max_pwm_blc / BRIGHTNESS_MAX_LEVEL;

	if (dev_priv->lvds_bl->pol == BLC_POLARITY_INVERSE)
		blc_pwm_duty_cycle = max_pwm_blc - blc_pwm_duty_cycle;

	blc_pwm_duty_cycle &= PSB_BACKLIGHT_PWM_POLARITY_BIT_CLEAR;
	REG_WRITE(BLC_PWM_CTL,
		  (max_pwm_blc << PSB_BACKLIGHT_PWM_CTL_SHIFT) |
		  (blc_pwm_duty_cycle));

	return 0;
}

/**
 * Set LVDS backlight level either by I2C or PWM
 */
void psb_intel_lvds_set_brightness(struct drm_device *dev, int level)
{
	/*u32 blc_pwm_ctl; */
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	DRM_DEBUG("backlight level is %d\n", level);

	if (!dev_priv->lvds_bl) {
		DRM_ERROR("NO LVDS Backlight Info\n");
		return;
	}

	if (dev_priv->lvds_bl->type == BLC_I2C_TYPE)
		psb_lvds_i2c_set_brightness(dev, level);
	else
		psb_lvds_pwm_set_brightness(dev, level);
}

/**
 * Sets the backlight level.
 *
 * \param level backlight level, from 0 to psb_intel_lvds_get_max_backlight().
 */
static void psb_intel_lvds_set_backlight(struct drm_device *dev, int level)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	u32 blc_pwm_ctl;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_ONLY_IF_ON)) {
		blc_pwm_ctl =
		    REG_READ(BLC_PWM_CTL) & ~BACKLIGHT_DUTY_CYCLE_MASK;
		REG_WRITE(BLC_PWM_CTL,
			  (blc_pwm_ctl |
			   (level << BACKLIGHT_DUTY_CYCLE_SHIFT)));
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		blc_pwm_ctl = dev_priv->saveBLC_PWM_CTL &
		    ~BACKLIGHT_DUTY_CYCLE_MASK;
		dev_priv->saveBLC_PWM_CTL = (blc_pwm_ctl |
					     (level <<
					      BACKLIGHT_DUTY_CYCLE_SHIFT));
	}
}

/**
 * Sets the power state for the panel.
 */
static void psb_intel_lvds_set_power(struct drm_device *dev,
				     struct psb_intel_output *output, bool on)
{
	u32 pp_status;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) | POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while ((pp_status & PP_ON) == 0);

		psb_intel_lvds_set_backlight(dev,
					     output->
					     mode_dev->backlight_duty_cycle);
	} else {
		psb_intel_lvds_set_backlight(dev, 0);

		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) & ~POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while (pp_status & PP_ON);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void psb_intel_lvds_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	if (mode == DRM_MODE_DPMS_ON)
		psb_intel_lvds_set_power(dev, output, true);
	else
		psb_intel_lvds_set_power(dev, output, false);

	/* XXX: We never power down the LVDS pairs. */
}

static void psb_intel_lvds_save(struct drm_connector *connector)
{
}

static void psb_intel_lvds_restore(struct drm_connector *connector)
{
}

int psb_intel_lvds_mode_valid(struct drm_connector *connector,
			      struct drm_display_mode *mode)
{
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct drm_display_mode *fixed_mode =
	    psb_intel_output->mode_dev->panel_fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if (psb_intel_output->type == INTEL_OUTPUT_MIPI2)
		fixed_mode = psb_intel_output->mode_dev->panel_fixed_mode2;

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	if (fixed_mode) {
		if (mode->hdisplay > fixed_mode->hdisplay)
			return MODE_PANEL;
		if (mode->vdisplay > fixed_mode->vdisplay)
			return MODE_PANEL;
	}
	return MODE_OK;
}

bool psb_intel_lvds_mode_fixup(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct psb_intel_mode_device *mode_dev =
	    enc_to_psb_intel_output(encoder)->mode_dev;
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc =
	    to_psb_intel_crtc(encoder->crtc);
	struct drm_encoder *tmp_encoder;
	struct drm_display_mode *panel_fixed_mode = mode_dev->panel_fixed_mode;
	struct psb_intel_output *psb_intel_output =
	    enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("type = 0x%x, pipe = %d.\n", psb_intel_output->type,
			psb_intel_crtc->pipe);

	if (psb_intel_output->type == INTEL_OUTPUT_MIPI2)
		panel_fixed_mode = mode_dev->panel_fixed_mode2;

	/* Should never happen!! */
	if (IS_MID(dev) && psb_intel_crtc->pipe == 1) {
		printk(KERN_ERR "Can't support LVDS/MIPI on pipe B on MRST\n");
		return false;
	}
	/* Should never happen!! */
	list_for_each_entry(tmp_encoder, &dev->mode_config.encoder_list, head) {
		if (tmp_encoder != encoder
		    && tmp_encoder->crtc == encoder->crtc) {
			printk(KERN_ERR "Can't enable LVDS and another "
			       "encoder on the same pipe\n");
			return false;
		}
	}

	/*
	 * If we have timings from the BIOS for the panel, put them in
	 * to the adjusted mode.  The CRTC will be set up for this mode,
	 * with the panel scaling set up to source from the H/VDisplay
	 * of the original mode.
	 */
	if (panel_fixed_mode != NULL) {
		adjusted_mode->hdisplay = panel_fixed_mode->hdisplay;
		adjusted_mode->hsync_start = panel_fixed_mode->hsync_start;
		adjusted_mode->hsync_end = panel_fixed_mode->hsync_end;
		adjusted_mode->htotal = panel_fixed_mode->htotal;
		adjusted_mode->vdisplay = panel_fixed_mode->vdisplay;
		adjusted_mode->vsync_start = panel_fixed_mode->vsync_start;
		adjusted_mode->vsync_end = panel_fixed_mode->vsync_end;
		adjusted_mode->vtotal = panel_fixed_mode->vtotal;
		adjusted_mode->clock = panel_fixed_mode->clock;
		drm_mode_set_crtcinfo(adjusted_mode, CRTC_INTERLACE_HALVE_V);
	}

	/*
	 * XXX: It would be nice to support lower refresh rates on the
	 * panels to reduce power consumption, and perhaps match the
	 * user's requested refresh rate.
	 */

	return true;
}

static void psb_intel_lvds_prepare(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct psb_intel_mode_device *mode_dev = output->mode_dev;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	mode_dev->saveBLC_PWM_CTL = REG_READ(BLC_PWM_CTL);
	mode_dev->backlight_duty_cycle = (mode_dev->saveBLC_PWM_CTL &
					  BACKLIGHT_DUTY_CYCLE_MASK);

	psb_intel_lvds_set_power(dev, output, false);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void psb_intel_lvds_commit(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct psb_intel_mode_device *mode_dev = output->mode_dev;

	PSB_DEBUG_ENTRY("\n");

	if (mode_dev->backlight_duty_cycle == 0)
		mode_dev->backlight_duty_cycle =
		    psb_intel_lvds_get_max_backlight(dev);

	psb_intel_lvds_set_power(dev, output, true);
}

static void psb_intel_lvds_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct psb_intel_mode_device *mode_dev =
	    enc_to_psb_intel_output(encoder)->mode_dev;
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc =
	    to_psb_intel_crtc(encoder->crtc);
	u32 pfit_control;

	/*
	 * The LVDS pin pair will already have been turned on in the
	 * psb_intel_crtc_mode_set since it has a large impact on the DPLL
	 * settings.
	 */

	/*
	 * Enable automatic panel scaling so that non-native modes fill the
	 * screen.  Should be enabled before the pipe is enabled, according to
	 * register description and PRM.
	 */
	if (mode->hdisplay != adjusted_mode->hdisplay ||
	    mode->vdisplay != adjusted_mode->vdisplay)
		pfit_control = (PFIT_ENABLE | VERT_AUTO_SCALE |
				HORIZ_AUTO_SCALE | VERT_INTERP_BILINEAR |
				HORIZ_INTERP_BILINEAR);
	else
		pfit_control = 0;

	if (mode_dev->panel_wants_dither)
		pfit_control |= PANEL_8TO6_DITHER_ENABLE;

	REG_WRITE(PFIT_CONTROL, pfit_control);
}

/**
 * Detect the LVDS connection.
 *
 * This always returns CONNECTOR_STATUS_CONNECTED.
 * This connector should only have
 * been set up if the LVDS was actually connected anyway.
 */
static enum drm_connector_status psb_intel_lvds_detect(struct drm_connector
						       *connector)
{
	return connector_status_connected;
}

/**
 * Return the list of DDC modes if available, or the BIOS fixed mode otherwise.
 */
static int psb_intel_lvds_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct psb_intel_mode_device *mode_dev = psb_intel_output->mode_dev;
	int ret = 0;

	ret = psb_intel_ddc_get_modes(psb_intel_output);

	if (ret)
		return ret;

	/* Didn't get an EDID, so
	 * Set wide sync ranges so we get all modes
	 * handed to valid_mode for checking
	 */
	connector->display_info.min_vfreq = 0;
	connector->display_info.max_vfreq = 200;
	connector->display_info.min_hfreq = 0;
	connector->display_info.max_hfreq = 200;

	if (mode_dev->panel_fixed_mode != NULL) {
		struct drm_display_mode *mode =
		    drm_mode_duplicate(dev, mode_dev->panel_fixed_mode);
		drm_mode_probed_add(connector, mode);
		return 1;
	}

	return 0;
}

/**
 * psb_intel_lvds_destroy - unregister and free LVDS structures
 * @connector: connector to free
 *
 * Unregister the DDC bus for this connector then free the driver private
 * structure.
 */
void psb_intel_lvds_destroy(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);

	if (psb_intel_output->ddc_bus)
		psb_intel_i2c_destroy(psb_intel_output->ddc_bus);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

int psb_intel_lvds_set_property(struct drm_connector *connector,
				struct drm_property *property, uint64_t value)
{
	struct drm_encoder *pEncoder = connector->encoder;

	PSB_DEBUG_ENTRY("\n");

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		struct psb_intel_crtc *pPsbCrtc =
		    to_psb_intel_crtc(pEncoder->crtc);
		uint64_t curValue;

		PSB_DEBUG_ENTRY("scaling mode \n");

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

		if (drm_connector_property_get_value(connector,
						     property, &curValue))
			goto set_prop_error;

		if (curValue == value)
			goto set_prop_done;

		if (drm_connector_property_set_value(connector,
						     property, value))
			goto set_prop_error;

		if (pPsbCrtc->saved_mode.hdisplay != 0 &&
		    pPsbCrtc->saved_mode.vdisplay != 0) {
			if (!drm_crtc_helper_set_mode(pEncoder->crtc,
						      &pPsbCrtc->saved_mode,
						      pEncoder->crtc->x,
						      pEncoder->crtc->y,
						      pEncoder->crtc->fb))
				goto set_prop_error;
		}
	} else if (!strcmp(property->name, "backlight") && pEncoder) {
		PSB_DEBUG_ENTRY("backlight \n");

		if (drm_connector_property_set_value(connector,
						     property, value))
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

static const struct drm_encoder_helper_funcs psb_intel_lvds_helper_funcs = {
	.dpms = psb_intel_lvds_encoder_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = psb_intel_lvds_prepare,
	.mode_set = psb_intel_lvds_mode_set,
	.commit = psb_intel_lvds_commit,
};

static const struct drm_connector_helper_funcs
    psb_intel_lvds_connector_helper_funcs = {
	.get_modes = psb_intel_lvds_get_modes,
	.mode_valid = psb_intel_lvds_mode_valid,
	.best_encoder = psb_intel_best_encoder,
};

static const struct drm_connector_funcs psb_intel_lvds_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.save = psb_intel_lvds_save,
	.restore = psb_intel_lvds_restore,
	.detect = psb_intel_lvds_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = psb_intel_lvds_set_property,
	.destroy = psb_intel_lvds_destroy,
};

static void psb_intel_lvds_enc_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

const struct drm_encoder_funcs psb_intel_lvds_enc_funcs = {
	.destroy = psb_intel_lvds_enc_destroy,
};

/* MRST platform start */

/*
 * FIXME need to move to register define head file
 */
#define MRST_BACKLIGHT_MODULATION_FREQ_SHIFT		(16)
#define MRST_BACKLIGHT_MODULATION_FREQ_MASK		(0xffff << 16)

/* The max/min PWM frequency in BPCR[31:17] - */
/* The smallest number is 1 (not 0) that can fit in the
 * 15-bit field of the and then*/
/* shifts to the left by one bit to get the actual 16-bit
 * value that the 15-bits correspond to.*/
#define MRST_BLC_MAX_PWM_REG_FREQ	    0xFFFF

#define BRIGHTNESS_MAX_LEVEL 100
#define BLC_PWM_PRECISION_FACTOR 10	/* 10000000 */
#define BLC_PWM_FREQ_CALC_CONSTANT 32
#define MHz 1000000
#define BLC_POLARITY_NORMAL 0
#define BLC_POLARITY_INVERSE 1

/**
 * Sets the power state for the panel.
 */
static void mrst_lvds_set_power(struct drm_device *dev,
				struct psb_intel_output *output, bool on)
{
	u32 pp_status;
	/* u32 pp_status; */
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) | POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while ((pp_status & (PP_ON | PP_READY)) == PP_READY);
		dev_priv->is_lvds_on = true;
	} else {
		REG_WRITE(PP_CONTROL, REG_READ(PP_CONTROL) & ~POWER_TARGET_ON);
		do {
			pp_status = REG_READ(PP_STATUS);
		} while (pp_status & PP_ON);
		dev_priv->is_lvds_on = false;
		pm_request_idle(&dev->pdev->dev);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mrst_lvds_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("\n");

	if (mode == DRM_MODE_DPMS_ON)
		mrst_lvds_set_power(dev, output, true);
	else
		mrst_lvds_set_power(dev, output, false);

	/* XXX: We never power down the LVDS pairs. */
}

static void mrst_lvds_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct psb_intel_mode_device *mode_dev =
	    enc_to_psb_intel_output(encoder)->mode_dev;
	struct drm_device *dev = encoder->dev;
	u32 lvds_port;
	uint64_t curValue = DRM_MODE_SCALE_FULLSCREEN;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	/*
	 * The LVDS pin pair will already have been turned on in the
	 * psb_intel_crtc_mode_set since it has a large impact on the DPLL
	 * settings.
	 */

	lvds_port = (REG_READ(LVDS) &
		     (~LVDS_PIPEB_SELECT)) | LVDS_PORT_EN | LVDS_BORDER_EN;

	if (mode_dev->panel_wants_dither)
		lvds_port |= MRST_PANEL_8TO6_DITHER_ENABLE;

	REG_WRITE(LVDS, lvds_port);

	drm_connector_property_get_value(&enc_to_psb_intel_output
					 (encoder)->base,
					 dev->mode_config.scaling_mode_property,
					 &curValue);

	if (curValue == DRM_MODE_SCALE_NO_SCALE)
		REG_WRITE(PFIT_CONTROL, 0);
	else if (curValue == DRM_MODE_SCALE_ASPECT) {
		if ((mode->vdisplay != adjusted_mode->crtc_vdisplay) ||
		    (mode->hdisplay != adjusted_mode->crtc_hdisplay)) {
			if ((adjusted_mode->crtc_hdisplay * mode->vdisplay) ==
			    (mode->hdisplay * adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
			else if ((adjusted_mode->crtc_hdisplay *
				  mode->vdisplay) > (mode->hdisplay *
						     adjusted_mode->crtc_vdisplay))
				REG_WRITE(PFIT_CONTROL,
					  PFIT_ENABLE |
					  PFIT_SCALING_MODE_PILLARBOX);
			else
				REG_WRITE(PFIT_CONTROL, PFIT_ENABLE |
					  PFIT_SCALING_MODE_LETTERBOX);
		} else
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);
	} else			/*(curValue == DRM_MODE_SCALE_FULLSCREEN) */
		REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static const struct drm_encoder_helper_funcs mrst_lvds_helper_funcs = {
	.dpms = mrst_lvds_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = psb_intel_lvds_prepare,
	.mode_set = mrst_lvds_mode_set,
	.commit = psb_intel_lvds_commit,
};

static struct drm_display_mode lvds_configuration_modes[] = {
	/* hard coded fixed mode for TPO LTPS LPJ040K001A */
	{DRM_MODE("800x480", DRM_MODE_TYPE_DRIVER, 33264, 800, 836,
		  846, 1056, 0, 480, 489, 491, 525, 0, 0)},
	/* hard coded fixed mode for LVDS 800x480 */
	{DRM_MODE("800x480", DRM_MODE_TYPE_DRIVER, 30994, 800, 801,
		  802, 1024, 0, 480, 481, 482, 525, 0, 0)},
	/* hard coded fixed mode for Samsung 480wsvga LVDS 1024x600@75 */
	{DRM_MODE("1024x600", DRM_MODE_TYPE_DRIVER, 53990, 1024, 1072,
		  1104, 1184, 0, 600, 603, 604, 608, 0, 0)},
	/* hard coded fixed mode for Samsung 480wsvga LVDS 1024x600@75 */
	{DRM_MODE("1024x600", DRM_MODE_TYPE_DRIVER, 53990, 1024, 1104,
		  1136, 1184, 0, 600, 603, 604, 608, 0, 0)},
	/* hard coded fixed mode for Sharp wsvga LVDS 1024x600 */
	{DRM_MODE("1024x600", DRM_MODE_TYPE_DRIVER, 48885, 1024, 1124,
		  1204, 1312, 0, 600, 607, 610, 621, 0, 0)},
	/* hard coded fixed mode for LVDS 1024x768 */
	{DRM_MODE("1024x768", DRM_MODE_TYPE_DRIVER, 65000, 1024, 1048,
		  1184, 1344, 0, 768, 771, 777, 806, 0, 0)},
	/* hard coded fixed mode for LVDS 1366x768 */
	{DRM_MODE("1366x768", DRM_MODE_TYPE_DRIVER, 77500, 1366, 1430,
		  1558, 1664, 0, 768, 769, 770, 776, 0, 0)},
};

/** Returns the panel fixed mode from configuration. */
static struct drm_display_mode *mrst_lvds_get_configuration_mode(struct
								 drm_device
								 *dev)
{
	struct drm_display_mode *mode = NULL;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;

	if (dev_priv->vbt_data.Size != 0x00) {	/*if non-zero, then use vbt */
		mode = kzalloc(sizeof(*mode), GFP_KERNEL);
		if (!mode)
			return NULL;

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
		    mode->vdisplay + ((ti->vsync_offset_hi << 4) |
				      ti->vsync_offset_lo);
		mode->vsync_end =
		    mode->vsync_start + ((ti->vsync_pulse_width_hi << 4) |
					 ti->vsync_pulse_width_lo);
		mode->vtotal = mode->vdisplay +
		    ((ti->vblank_hi << 8) | ti->vblank_lo);
		mode->clock = ti->pixel_clock * 10;
	} else
		mode = drm_mode_duplicate(dev, &lvds_configuration_modes[2]);

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

/**
 * mrst_lvds_init - setup LVDS connectors on this device
 * @dev: drm device
 *
 * Create the connector, register the LVDS DDC bus, and try to figure out what
 * modes we can display on the LVDS panel (if present).
 */
void mrst_lvds_init(struct drm_device *dev,
		    struct psb_intel_mode_device *mode_dev)
{
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct edid *edid;
	int ret = 0;
	struct i2c_adapter *i2c_adap;
	struct drm_display_mode *scan;	/* *modes, *bios_mode; */

	PSB_DEBUG_ENTRY("\n");

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	dev_priv->is_lvds_on = true;
	drm_connector_init(dev, &psb_intel_output->base,
			   &psb_intel_lvds_connector_funcs,
			   DRM_MODE_CONNECTOR_LVDS);

	drm_encoder_init(dev, &psb_intel_output->enc, &psb_intel_lvds_enc_funcs,
			 DRM_MODE_ENCODER_LVDS);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type = INTEL_OUTPUT_LVDS;

	drm_encoder_helper_add(encoder, &mrst_lvds_helper_funcs);
	drm_connector_helper_add(connector,
				 &psb_intel_lvds_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_connector_attach_property(connector,
				      dev->mode_config.scaling_mode_property,
				      DRM_MODE_SCALE_FULLSCREEN);
	drm_connector_attach_property(connector,
				      dev_priv->backlight_property,
				      BRIGHTNESS_MAX_LEVEL);

	lvds_backlight = BRIGHTNESS_MAX_LEVEL;

	mode_dev->panel_wants_dither = false;
	if (dev_priv->vbt_data.Size != 0x00)
		mode_dev->panel_wants_dither =
		    (dev_priv->
		     gct_data.Panel_Port_Control &
		     MRST_PANEL_8TO6_DITHER_ENABLE);

	/*
	 * LVDS discovery:
	 * 1) check for EDID on DDC
	 * 2) check for VBT data
	 * 3) check to see if LVDS is already on
	 *    if none of the above, no panel
	 * 4) make sure lid is open
	 *    if closed, act like it's not there for now
	 */
	i2c_adap = i2c_get_adapter(2);
	if (i2c_adap == NULL)
		printk(KERN_ALERT "No ddc adapter available!\n");
	/* Set up the DDC bus. */
/*	psb_intel_output->ddc_bus = psb_intel_i2c_create(dev,
							 GPIOC,
							 "LVDSDDC_C");
	if (!psb_intel_output->ddc_bus) {
		dev_printk(KERN_ERR, &dev->pdev->dev,
			   "DDC bus registration " "failed.\n");
		goto failed_ddc;
	}*/

	/*
	 * Attempt to get the fixed panel mode from DDC.  Assume that the
	 * preferred mode is the right one.
	 */
	if (i2c_adap) {
		edid = drm_get_edid(connector, i2c_adap);
		if (edid) {
			drm_mode_connector_update_edid_property(connector,
								edid);
			ret = drm_add_edid_modes(connector, edid);
			kfree(edid);
		}

		list_for_each_entry(scan, &connector->probed_modes, head) {
			if (scan->type & DRM_MODE_TYPE_PREFERRED) {
				mode_dev->panel_fixed_mode =
				    drm_mode_duplicate(dev, scan);
				goto out;	/* FIXME: check for quirks */
			}
		}
	}

	/*
	 * If we didn't get EDID, try geting panel timing
	 * from configuration data
	 */
	mode_dev->panel_fixed_mode = mrst_lvds_get_configuration_mode(dev);

	if (mode_dev->panel_fixed_mode) {
		mode_dev->panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
		goto out;	/* FIXME: check for quirks */
	}

	/* If we still don't have a mode after all that, give up. */
	if (!mode_dev->panel_fixed_mode) {
		DRM_DEBUG("Found no modes on the lvds, ignoring the LVDS\n");
		goto failed_find;
	}

 out:
	drm_sysfs_connector_add(connector);
	return;

 failed_find:
	DRM_DEBUG("No LVDS modes found, disabling.\n");
	if (psb_intel_output->ddc_bus)
		psb_intel_i2c_destroy(psb_intel_output->ddc_bus);

/* failed_ddc: */

	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	kfree(connector);
}

/* MRST platform end */
