/*
 * Copyright © 2013 Intel Corporation
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
 * Author: Jani Nikula <jani.nikula@intel.com>  */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"

/**
 * XXX:
 *
 * Is intel_dsi good handle to pass here?
 *
 * Is the pipe static throughout the lifetime of a encoder/connector?!?!
 *
 * DPI vs. DBI in the old code... separate files?
 *
 * DSI error handling
 *
 * MEM WRITE
 *
 * Locking?
 *
 * XXX: use of these registers??? for mem write?
 *
 * MIPI_DATA_ADDRESS, updated data for the display panel
 * MIPI_DATA_LENGTH, remaining length of data that needs to be read
 *
 * MIPI_COMMAND_ADDRESS, address to read new commands from?! has "command data
 * mode" to use data for memory write from pipe A rendering.
 *
 * MIPI_COMMAND_LENGTH, lengths of up to four commands. wtf.
 *
 * MIPI_READ_DATA_RETURN, configuration reads only. max read should be no
 * greater than 32 bytes.
 *
 * MIPI_READ_DATA_VALID, bits 0-7 indicate validity of MIPI_READ_DATA_RETURN(n).
 *
 *
 * XXX: do we need sync vs. nosync variants?
 */

enum dsi_type {
	DSI_DCS,
	DSI_GENERIC,
};


static int dsi_vc_send_short(struct intel_dsi *intel_dsi, int channel,
			     u8 data_type, u16 data)
{
	struct drm_encoder *encoder = &intel_dsi->base.base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	enum pipe pipe = intel_crtc->pipe;
	u32 ctrl_reg;
	u32 ctrl;
	u32 mask = DBI_FIFO_EMPTY;

	/* XXX: set MIPI_HS_LS_DBI_ENABLE? wait for dbi fifo empty first */

	/* XXX: short write, do we need to wait for data FIFO? */
	if (intel_dsi->hs) {
		ctrl_reg = MIPI_HS_GEN_CTRL(pipe);
		mask |= HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY;
	} else {
		ctrl_reg = MIPI_LP_GEN_CTRL(pipe);
		mask |= LP_CTRL_FIFO_EMPTY | LP_DATA_FIFO_EMPTY;
	}

	/* Note: Could also wait for !full instead of empty. */
	if (wait_for_restrict((I915_READ(MIPI_GEN_FIFO_STAT(pipe)) & mask) == mask, 50))	//<asus-Bruce 20131223+>
		DRM_ERROR("Timeout waiting for FIFO empty\n");

	ctrl = data << SHORT_PACKET_PARAM_SHIFT |
		channel << VIRTUAL_CHANNEL_SHIFT |
		data_type << DATA_TYPE_SHIFT;

	I915_WRITE(ctrl_reg, ctrl);

	return 0;
}

static int dsi_vc_send_long(struct intel_dsi *intel_dsi, int channel,
			    u8 data_type, const u8 *data, u16 len) {
	struct drm_encoder *encoder = &intel_dsi->base.base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	enum pipe pipe = intel_crtc->pipe;
	u32 data_reg, ctrl_reg, ctrl;
	u16 i, j, n;
	u32 mask = DBI_FIFO_EMPTY;

	/* XXX: set MIPI_HS_LS_DBI_ENABLE? wait for dbi fifo empty first */

	/* XXX: pipe, hs */
	if (intel_dsi->hs) {
		data_reg = MIPI_HS_GEN_DATA(pipe);
		ctrl_reg = MIPI_HS_GEN_CTRL(pipe);
		mask |= HS_CTRL_FIFO_EMPTY | HS_DATA_FIFO_EMPTY;
	} else {
		data_reg = MIPI_LP_GEN_DATA(pipe);
		ctrl_reg = MIPI_LP_GEN_CTRL(pipe);
		mask |= LP_CTRL_FIFO_EMPTY | LP_DATA_FIFO_EMPTY;
	}

	/* Note: Could also wait for !full instead of empty. */
	if (wait_for_restrict((I915_READ(MIPI_GEN_FIFO_STAT(pipe)) & mask) == mask, 50))	//<asus-Bruce 20131223+>
		DRM_ERROR("Timeout waiting for FIFO empty\n");

	for (i = 0; i < len; i += n) {
		u32 val = 0;
		n = min(len - i, 4);

		for (j = 0; j < n; j++)
			val |= *data++ << 8 * j;

		I915_WRITE(data_reg, val);
	}

	ctrl = len << LONG_PACKET_WORD_COUNT_SHIFT;
	ctrl |= channel << VIRTUAL_CHANNEL_SHIFT;
	ctrl |= data_type << DATA_TYPE_SHIFT;

	I915_WRITE(ctrl_reg, ctrl);

	return 0;
}

static int dsi_vc_write_nosync_common(struct intel_dsi *intel_dsi,
				      int channel, const u8 *data, int len,
				      enum dsi_type type)
{
	int ret;

	if (len == 0) {
		BUG_ON(type == DSI_GENERIC);
		ret = dsi_vc_send_short(intel_dsi, channel,
					MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM,
					0);
	} else if (len == 1) {
		ret = dsi_vc_send_short(intel_dsi, channel,
					type == DSI_GENERIC ?
					MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM :
					MIPI_DSI_DCS_SHORT_WRITE, data[0]);
	} else if (len == 2) {
		ret = dsi_vc_send_short(intel_dsi, channel,
					type == DSI_GENERIC ?
					MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM :
					MIPI_DSI_DCS_SHORT_WRITE_PARAM,
					(data[1] << 8) | data[0]);
	} else {
		ret = dsi_vc_send_long(intel_dsi, channel,
				       type == DSI_GENERIC ?
				       MIPI_DSI_GENERIC_LONG_WRITE :
				       MIPI_DSI_DCS_LONG_WRITE, data, len);
	}

	return ret;
}

static int dsi_vc_dcs_write_nosync(struct intel_dsi *intel_dsi,
				   int channel, const u8 *data, int len)
{
	return dsi_vc_write_nosync_common(intel_dsi, channel, data, len,
					  DSI_DCS);
}

static int dsi_vc_generic_write_nosync(struct intel_dsi *intel_dsi,
				       int channel, const u8 *data, int len)
{
	return dsi_vc_write_nosync_common(intel_dsi, channel, data, len,
					  DSI_GENERIC);
}

static int dsi_vc_sync(struct intel_dsi *intel_dsi, int channel)
{
	/* XXX: wait for bta from device */
	return 0;
}

static int dsi_vc_write_common(struct intel_dsi *intel_dsi, int channel,
			       const u8 *data, int len, enum dsi_type type)
{
	int ret;

	ret = dsi_vc_write_nosync_common(intel_dsi, channel, data, len, type);
	if (ret)
		return ret;

	ret = dsi_vc_sync(intel_dsi, channel);
	if (ret)
		return ret;

	/* XXX: fifo check */

	return 0;
}

int dsi_vc_dcs_write(struct intel_dsi *intel_dsi, int channel,
		     const u8 *data, int len)
{
	return dsi_vc_write_common(intel_dsi, channel, data, len, DSI_DCS);
}
EXPORT_SYMBOL(dsi_vc_dcs_write);

int dsi_vc_generic_write(struct intel_dsi *intel_dsi, int channel,
			 const u8 *data, int len)
{
	return dsi_vc_write_common(intel_dsi, channel, data, len, DSI_GENERIC);
}
EXPORT_SYMBOL(dsi_vc_generic_write);


static int dsi_vc_dcs_send_read_request(struct intel_dsi *intel_dsi,
					int channel, u8 dcs_cmd)
{
	return dsi_vc_send_short(intel_dsi, channel, MIPI_DSI_DCS_READ,
				 dcs_cmd);
}

static int dsi_vc_generic_send_read_request(struct intel_dsi *intel_dsi,
					    int channel, u8 *reqdata,
					    int reqlen)
{
	u16 data = 0;
	u8 data_type = 0;

	switch (reqlen) {
	case 0:
		data_type = MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM;
		data = 0;
		break;
	case 1:
		data_type = MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM;
		data = reqdata[0];
		break;
	case 2:
		data_type = MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM;
		data = (reqdata[1] << 8) | reqdata[0];
		break;
	default:
		BUG();
	}

	return dsi_vc_send_short(intel_dsi, channel, data_type, data);
}

static int dsi_read_data_return(struct intel_dsi *intel_dsi,
				u8 *buf, int buflen)
{
	struct drm_encoder *encoder = &intel_dsi->base.base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	enum pipe pipe = intel_crtc->pipe;
	int i, j, len = 0;
	u32 data_valid, val;

	data_valid = I915_READ(MIPI_READ_DATA_VALID(pipe));

	/* XXX: byte order of data in return registers? */
	for (i = 0; i < 8 && len < buflen; i++) {
		if (!(data_valid & (1 << i)))
			break;

		val = I915_READ(MIPI_READ_DATA_RETURN(pipe, i));
		for (j = 0; j < 4 && len < buflen; j++, len++)
			buf[len] = val >> 8 * j;
	}

	I915_WRITE(MIPI_READ_DATA_VALID(pipe), data_valid);

	return len;
}

int dsi_vc_dcs_read(struct intel_dsi *intel_dsi, int channel, u8 dcs_cmd,
		    u8 *buf, int buflen)
{
	int ret;

	/* XXX: set MIPI_MAX_RETURN_PKT_SIZE? */

	ret = dsi_vc_dcs_send_read_request(intel_dsi, channel, dcs_cmd);
	if (ret)
		return ret;

	ret = dsi_vc_sync(intel_dsi, channel);
	if (ret)
		return ret;

	ret = dsi_read_data_return(intel_dsi, buf, buflen);
	if (ret < 0)
		return ret;

	if (ret != buflen)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(dsi_vc_dcs_read);

int dsi_vc_generic_read(struct intel_dsi *intel_dsi, int channel,
			u8 *reqdata, int reqlen, u8 *buf, int buflen)
{
	int ret;

	ret = dsi_vc_generic_send_read_request(intel_dsi, channel, reqdata,
					       reqlen);
	if (ret)
		return ret;

	ret = dsi_vc_sync(intel_dsi, channel);
	if (ret)
		return ret;

	ret = dsi_read_data_return(intel_dsi, buf, buflen);
	if (ret < 0)
		return ret;

	if (ret != buflen)
		return -EIO;

	return 0;
}


/* DPI */

/* XXX: what is "spk" or "spl" packet? XXX: how about MIPI_DPI_DATA? */
int dpi_send_cmd(struct intel_dsi *intel_dsi, u32 cmd)
{
	struct drm_encoder *encoder = &intel_dsi->base.base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	enum pipe pipe = intel_crtc->pipe;
	u32 mask;

	/* XXX: pipe, hs */
	if (intel_dsi->hs)
		cmd &= ~DPI_HS_MODE;
	else
		cmd |= DPI_HS_MODE;

	/* DPI virtual channel?! */

	mask = DPI_FIFO_EMPTY;
	if (wait_for((I915_READ(MIPI_GEN_FIFO_STAT(pipe)) & mask) == mask, 50))
		DRM_ERROR("fifo\n");

	/* clear bit */
	I915_WRITE(MIPI_INTR_STAT(pipe), SPL_PKT_SENT_INTERRUPT);

	/* XXX: old code skips write if control unchanged */
	I915_WRITE(MIPI_DPI_CONTROL(pipe), cmd);

	mask = SPL_PKT_SENT_INTERRUPT;
	if (wait_for((I915_READ(MIPI_INTR_STAT(pipe)) & mask) == 0, 50))
		DRM_ERROR("fail\n");

	return 0;
}
