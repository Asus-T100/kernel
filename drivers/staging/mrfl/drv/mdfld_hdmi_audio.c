/*
 * Copyright (c) 2010, Intel Corporation.
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
 *	jim liu <jim.liu@intel.com>
 */

#include <drm/drmP.h>
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_intel_hdmi_reg.h"
#include "psb_intel_hdmi_edid.h"
#include "psb_intel_hdmi.h"
#include "mdfld_hdmi_audio_if.h"

/*
 * Audio register range 0x69000 to 0x69117
 */

#define IS_HDMI_AUDIO_REG(reg) ((reg >= 0x69000) && (reg < 0x69118))

/*
 *
 */
static struct mid_intel_hdmi_priv *hdmi_priv;

void mdfld_hdmi_audio_init(struct mid_intel_hdmi_priv *p_hdmi_priv)
{
	hdmi_priv = p_hdmi_priv;
}

/**
 * mdfld_hdmi_audio_write:
 * used to write into display controller HDMI audio registers.
 *
 */
static int mdfld_hdmi_audio_write(uint32_t reg, uint32_t val)
{
	struct drm_device *dev = hdmi_priv->dev;
	int ret = 0;

	if (IS_HDMI_AUDIO_REG(reg))
		REG_WRITE(reg, val);
	else
		ret = -EINVAL;

	return ret;
}

/**
 * mdfld_hdmi_audio_read:
 * used to get the register value read from display controller
 * HDMI audio registers.
 */
static int mdfld_hdmi_audio_read(uint32_t reg, uint32_t *val)
{
	struct drm_device *dev = hdmi_priv->dev;
	int ret = 0;

	if (IS_HDMI_AUDIO_REG(reg))
		*val = REG_READ(reg);
	else
		ret = -EINVAL;

	return ret;
}

/**
 * mdfld_hdmi_audio_rmw:
 * used to update the masked bits in display controller HDMI audio registers .
 *
 */
static int mdfld_hdmi_audio_rmw(uint32_t reg, uint32_t val, uint32_t mask)
{
	struct drm_device *dev = hdmi_priv->dev;
	int ret = 0;
	uint32_t val_tmp = 0;

	if (IS_HDMI_AUDIO_REG(reg)) {
		val_tmp = (val & mask) | (REG_READ(reg) & ~mask);
		REG_WRITE(reg, val_tmp);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/**
 * mdfld_hdmi_audio_get_caps:
 * used to return the HDMI audio capabilities.
 * e.g. resolution, frame rate.
 */
static int mdfld_hdmi_audio_get_caps(enum had_caps_list get_element,
				     void *capabilities)
{
	struct drm_device *dev = hdmi_priv->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	switch (get_element) {
	case HAD_GET_ELD:
		memcpy(capabilities, &(hdmi_priv->eeld), sizeof(hdmi_eeld_t));
		break;
	case HAD_GET_SAMPLING_FREQ:
		memcpy(capabilities, &(dev_priv->tmds_clock_khz),
		       sizeof(uint32_t));
		break;
	default:
		break;
	}

	return ret;
}

/**
 * mdfld_hdmi_audio_set_caps:
 * used to set the HDMI audio capabilities.
 * e.g. Audio INT.
 */
static int mdfld_hdmi_audio_set_caps(enum had_caps_list set_element,
				     void *capabilties)
{
	struct drm_device *dev = hdmi_priv->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	int ret = 0;
	u32 hdmib;
	u32 int_masks = 0;

	PSB_DEBUG_ENTRY("\n");

	switch (set_element) {
	case HAD_SET_ENABLE_AUDIO:
		hdmib = REG_READ(hdmi_priv->hdmib_reg);

		if ((hdmib & HDMIB_PORT_EN) && hdmi_priv->has_hdmi_sink)
			hdmib |= HDMI_AUDIO_ENABLE;

		REG_WRITE(hdmi_priv->hdmib_reg, hdmib);
		REG_READ(hdmi_priv->hdmib_reg);
		break;
	case HAD_SET_DISABLE_AUDIO:
		hdmib = REG_READ(hdmi_priv->hdmib_reg) & ~HDMI_AUDIO_ENABLE;
		REG_WRITE(hdmi_priv->hdmib_reg, hdmib);
		REG_READ(hdmi_priv->hdmib_reg);
		break;
	case HAD_SET_ENABLE_AUDIO_INT:
		if (*((u32 *) capabilties) & HDMI_AUDIO_UNDERRUN)
			int_masks |= PIPE_HDMI_AUDIO_UNDERRUN;

		if (*((u32 *) capabilties) & HDMI_AUDIO_BUFFER_DONE)
			int_masks |= PIPE_HDMI_AUDIO_BUFFER_DONE;

		if (dev_priv->hdmi_audio_interrupt_mask != int_masks) {
			dev_priv->hdmi_audio_interrupt_mask |= int_masks;
			mdfld_irq_enable_hdmi_audio(dev);
		}

		break;
	case HAD_SET_DISABLE_AUDIO_INT:
		if (*((u32 *) capabilties) & HDMI_AUDIO_UNDERRUN)
			int_masks |= PIPE_HDMI_AUDIO_UNDERRUN;

		if (*((u32 *) capabilties) & HDMI_AUDIO_BUFFER_DONE)
			int_masks |= PIPE_HDMI_AUDIO_BUFFER_DONE;

		if (dev_priv->hdmi_audio_interrupt_mask & int_masks) {
			dev_priv->hdmi_audio_interrupt_mask &= ~int_masks;

			if (dev_priv->hdmi_audio_interrupt_mask)
				mdfld_irq_enable_hdmi_audio(dev);
			else
				mdfld_irq_disable_hdmi_audio(dev);
		}

		break;
	default:
		break;
	}

	return ret;
}

static struct hdmi_audio_registers_ops mdfld_hdmi_audio_reg_ops = {
	.hdmi_audio_read_register = mdfld_hdmi_audio_read,
	.hdmi_audio_write_register = mdfld_hdmi_audio_write,
	.hdmi_audio_read_modify = mdfld_hdmi_audio_rmw,
};

static struct hdmi_audio_query_set_ops mdfld_hdmi_audio_get_set_ops = {
	.hdmi_audio_get_caps = mdfld_hdmi_audio_get_caps,
	.hdmi_audio_set_caps = mdfld_hdmi_audio_set_caps,
};

int intel_hdmi_audio_query_capabilities(had_event_call_back audio_callbacks,
				struct hdmi_audio_registers_ops
				*reg_ops, struct hdmi_audio_query_set_ops
				*query_ops)
{
	struct drm_device *dev = hdmi_priv->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	int ret = 0;

	reg_ops->hdmi_audio_read_register =
	    (mdfld_hdmi_audio_reg_ops.hdmi_audio_read_register);
	reg_ops->hdmi_audio_write_register =
	    (mdfld_hdmi_audio_reg_ops.hdmi_audio_write_register);
	reg_ops->hdmi_audio_read_modify =
	    (mdfld_hdmi_audio_reg_ops.hdmi_audio_read_modify);
	query_ops->hdmi_audio_get_caps =
	    mdfld_hdmi_audio_get_set_ops.hdmi_audio_get_caps;
	query_ops->hdmi_audio_set_caps =
	    mdfld_hdmi_audio_get_set_ops.hdmi_audio_set_caps;

	dev_priv->mdfld_had_event_callbacks = audio_callbacks;

	return ret;
}

int display_register(struct snd_intel_had_interface *driver, void *had_data)
{
	struct drm_device *dev = hdmi_priv->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	dev_priv->had_pvt_data = had_data;
	dev_priv->had_interface = driver;

	/* The Audio driver is loading now and we need to notify
	 * it if there is an HDMI device attached */
	DRM_INFO("display_register: Scheduling HDMI audio work queue\n");
	schedule_work(&dev_priv->hdmi_audio_wq);

	return 0;
}
