/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 * Neither the name of Intel Corporation nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __ANDROID_HDMI_H
#define __ANDROID_HDMI_H

#include <linux/types.h>
#include <drm/drmP.h>

#define CEA_EXT     0x02
#define VTB_EXT     0x10
#define DI_EXT      0x40
#define LS_EXT      0x50
#define MI_EXT      0x60

/* Define the monitor type HDMI or DVI */
#define MONITOR_TYPE_HDMI 1
#define MONITOR_TYPE_DVI  2

struct android_hdmi_priv {
	/* common */
	struct drm_device *dev;

	/*medfield specific */
	u32 hdmib_reg;
	u32 save_HDMIB;

	/* Delayed Encoder Restore */
	struct drm_display_mode *current_mode;
	bool need_encoder_restore;
	struct delayed_work enc_work;
	struct delayed_work hdmi_delayed_wq;
	void *data;

	bool is_hdcp_supported;
	int monitor_type;
	void *context;
};

extern int psb_intel_panel_fitter_pipe(struct drm_device *dev);
extern void mid_hdmi_audio_init(struct android_hdmi_priv *p_hdmi_priv);

#ifdef CONFIG_SUPPORT_HDMI

/**
 * This function initializes the hdmi driver called during bootup
 * @dev		: handle to drm_device
 * @mode_dev	: device mode
 *
 * Returns nothing
 *
 * This function initializes the hdmi driver called during bootup
 * which includes initializing drm_connector, drm_encoder, hdmi audio
 * and msic and collects all information reqd in hdmi private.
 */
void android_hdmi_driver_init(struct drm_device *dev,
			      void *mode_dev);

/**
 * This function sets the hdmi driver during bootup
 * @dev		: handle to drm_device
 *
 * Returns nothing
 *
 * This function is called from psb driver to setup the
 * hdmi driver. Called only once during boot-up of the system
 */
void android_hdmi_driver_setup(struct drm_device *dev);

/**
 * DRM connector helper routine.
 * @connector	: drm_connector handle
 * @mode		: drm_display_mode handle
 *
 * Returns integer values which tell whether the hdmi mode
 * is valid or not
 * MODE_CLOCK_LOW - mode clock less than min pixel clock value
 * MODE_CLOCK_HIGH - mode clock greater than min pixel clock value
 * MODE_BAD - mode values are incorrect
 * MODE_OK - mode values are correct
 * MODE_NO_DBLESCAN - double scan mode not supported
 * MODE_NO_INTERLACE - interlace mode not supported
 * This is the DRM connector helper routine
 */
int android_hdmi_mode_valid(struct drm_connector *connector,
			    struct drm_display_mode *mode);

/**
 * DRM get modes helper routine
 * @connector	: handle to drm_connector
 *
 * Returns the number of modes added
 * This is a helper routines for DRM get modes.
 * This function gets the edid information from the external sink
 * device using i2c when connected and parses the edid information
 * obtained and adds the modes to connector list
 * If sink device is not connected, then static edid timings are
 * used and those modes are added to the connector list
 */
int android_hdmi_get_modes(struct drm_connector *connector);

/**
 * DRM encoder save helper routine
 * @encoder      : handle to drm_encoder
 *
 * Returns nothing
 * This helper routine is used by DRM during early suspend
 * operation to simply disable active plane.
 */
void android_hdmi_encoder_save(struct drm_encoder *encoder);

/**
 * DRM encoder restore helper routine
 * @encoder      : handle to drm_encoder
 *
 * Returns nothing
 * This helper routine is used by DRM during late resume
 * operation for restoring the pipe and enabling it. The
 * operation itself is completed in a delayed workqueue
 * item which ensures restore can be done once the system
 * is resumed.
 */
void android_hdmi_encoder_restore(struct drm_encoder *encoder);

/**
 * DRM encoder mode fixup helper routine
 * @encoder      : handle to drm_encoder
 * @mode         : proposed display mode
 * @adjusted_mode: actual mode to be displayed by HW
 *
 * Returns boolean to indicate success/failure
 * This routine can be used to make adjustments to actual
 * mode parameters as required by underlying HW.
 * This is currently not required.
 */
bool android_hdmi_mode_fixup(struct drm_encoder *encoder,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode);


/**
 * DRM connector save helper routine
 * @connector       : handle to drm_connector
 *
 * Returns nothing.
 * This routine is used to save connector state.
 */
void android_hdmi_connector_save(struct drm_connector *connector);

/**
 * DRM connector restore helper routine
 * @connector       : handle to drm_connector
 *
 * Returns nothing.
 * This routine is used to restore connector state.
 */
void android_hdmi_connector_restore(struct drm_connector *connector);

 /**
 * Description: programming display registers as per the scaling property.
 *
 * @crtc:		crtc
 *
 * Returns:	0 on success
 *		-1 on failure
 */
int android_hdmi_set_scaling_property(struct drm_crtc *crtc);

/**
 * Description: crtc mode set for hdmi pipe.
 *
 * @crtc:		crtc
 * @mode:		mode requested
 * @adjusted_mode:	adjusted mode
 * @x, y, old_fb:	old frame buffer values used for flushing old plane.
 *
 * Returns:	0 on success
 *		-EINVAL on NULL input arguments
 */
int android_hdmi_crtc_mode_set(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb);

/**
 * Description: encoder mode set for hdmi pipe.
 *
 * @encoder:		hdmi encoder
 * @mode:		mode requested
 * @adjusted_mode:	adjusted mode
 *
 * Returns:	none.
 */
void android_hdmi_enc_mode_set(struct drm_encoder *encoder,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode);

/**
 * Store the HDMI registers and enable the display
 * Input parameters:
 *	psDrmDev: Drm Device.
 * Returns: none
 */
void android_hdmi_restore_and_enable_display(struct drm_device *dev);

/**
 * Save the HDMI display registers
 * Input parameters:
 *	psDrmDev: Drm Device.
 * Returns: none
 */
void android_hdmi_save_display_registers(struct drm_device *dev);

/**
 * Prepare HDMI EDID-like data and copy it to the given buffer
 * Input parameters:
 * @dev: drm Device
 * @eld: pointer to otm_hdmi_eld_t data structure
 *
 * Returns:	0 on success
 *		-EINVAL on NULL input arguments
 */
int android_hdmi_get_eld(struct drm_device *dev, void *eld);


/**
 * disable HDMI display
 * Input parameters:
 *	psDrmDev: Drm Device.
 * Returns: none
 */
void android_disable_hdmi(struct drm_device *dev);

/**
 * Enable HDCP on HDMI display
 * @dev:	drm device
 *
 * Returns:	true on success else false
 */
bool android_enable_hdmi_hdcp(struct drm_device *dev);

/**
 * disable HDCP on HDMI display
 * @dev:	drm device
 *
 * Returns:	true on success else false
 */
bool android_disable_hdmi_hdcp(struct drm_device *dev);

/**
 * Query whether HDCP is enabled & encrypting on HDMI display
 * @dev:	drm device
 *
 * Returns:	true if encrypting else false
 */
bool android_check_hdmi_hdcp_enc_status(struct drm_device *dev);

/**
 * Query HDCP Phase 3 Link Status
 * @dev:	drm device
 *
 * Returns:	true if link is not compromised else false
 */
bool android_check_hdmi_hdcp_link_status(struct drm_device *dev);

/**
 * Query presence of a HDCP Sink Device
 * @dev:	drm device
 * @bksv:	ksv value of the sink device will be returned on success
 *
 * Returns:	true on successful detection else false
 */
bool android_query_hdmi_hdcp_sink(struct drm_device *dev, uint8_t *bksv);

/**
 * Description: hdmi helper function to detect whether hdmi/dvi
 *		is connected or not.
 *
 * @connector:	hdmi connector
 *
 * Returns:	connector_status_connected if hdmi/dvi is connected.
 *		connector_status_disconnected if hdmi/dvi is not connected.
 */
enum drm_connector_status android_hdmi_detect(struct drm_connector
					      *connector, bool force);


/**
 * Description: check whether hdmi/dvi is connected or not.
 *
 * @dev:	drm device
 *
 * Returns:	true if hdmi/dvi is connected.
 *		false if hdmi/dvi is not connected.
 */
bool android_hdmi_is_connected(struct drm_device *dev);

/**
 * Description: Turn on or off HDMI power rails.
 *
 * @on:	true or false
 *
 * Returns: none
 */
void android_hdmi_set_power_rails(bool on);


/**
 * Description: hdmi helper function to manage power to the display (dpms)
 *
 * @encoder:	hdmi encoder
 * @mode:	dpms on or off
 *
 * Returns:	none
 */
void android_hdmi_dpms(struct drm_encoder *encoder, int mode);

/**
 * Description: hdmi helper function to manage power to the connector (dpms)
 *
 * @connector: drm_connector
 * @mode:	  dpms on or off
 *
 * Returns:	none
 */
void android_hdmi_connector_dpms(struct drm_connector *connector, int mode);

#else /* CONFIG_SUPPORT_HDMI */

static inline void android_hdmi_driver_init(struct drm_device *dev,
						void *mode_dev) {}

static inline void android_hdmi_driver_setup(struct drm_device *dev) {}

static inline int android_hdmi_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode) { return 0; }

static inline int android_hdmi_get_modes(struct drm_connector *connector)
				{ return 0; }

static inline void android_hdmi_encoder_save(struct drm_encoder *encoder) {}

static inline void android_hdmi_encoder_restore(struct drm_encoder *encoder) {}

static inline bool android_hdmi_mode_fixup(struct drm_encoder *encoder,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode)
{ return false; }

static inline void android_hdmi_connector_save(struct drm_connector
					       *connector) {}

static inline void android_hdmi_connector_restore(struct drm_connector
						  *connector) {}

static inline int android_hdmi_set_scaling_property(struct drm_crtc *crtc)
{ return 0; }

static inline int android_hdmi_crtc_mode_set(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb) { return 0; }

static inline void android_hdmi_enc_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode) {}

static inline void android_hdmi_restore_and_enable_display(
				struct drm_device *dev) {}

static inline void android_hdmi_save_display_registers(
				struct drm_device *dev) {}

static inline void android_disable_hdmi(struct drm_device *dev) {}

static inline int android_hdmi_get_eld(struct drm_device *dev, void *eld)
{ return 0; }

static inline bool android_enable_hdmi_hdcp(struct drm_device *dev)
{ return false; }
static inline bool android_disable_hdmi_hdcp(struct drm_device *dev)
{ return false; }
static inline bool android_check_hdmi_hdcp_enc_status(struct drm_device *dev)
{ return false; }
static inline bool android_check_hdmi_hdcp_link_status(struct drm_device *dev)
{ return false; }
static inline bool android_query_hdmi_hdcp_sink(struct drm_device *dev,
						uint8_t *bksv)
{ return false; }

static inline enum drm_connector_status android_hdmi_detect(struct drm_connector
							 *connector, bool force)
{ return connector_status_disconnected; }

static inline bool android_hdmi_is_connected(struct drm_device *dev)
{ return false; }

static inline void android_hdmi_set_power_rails(bool on)
{}

static inline void android_hdmi_dpms(struct drm_encoder *encoder,
				int mode) {}

#endif /* CONFIG_SUPPORT_HDMI */

/*
 * Description: hdmi helper function to parse cmdline option
 *		from hdmicmd tool
 *
 * @cmdoption:	cmdline option
 *
 * Returns:	error codes 0(success),-1(cmd option),-2(invalid input)
 */
int otm_cmdline_parse_option(char *cmdoption);

/*
 * Description: hdmi helper function to parse vic option
 *		from hdmicmd tool
 *
 * @cmdoption:	cmdline option
 *
 * Returns:	error codes 0(success),-1(error)
 */
int otm_cmdline_set_vic_option(int vic);

/*
 * Description: hdmi helper function to print cmdline options
 *		from hdmicmd tool
 *
 * Returns:	none
 */
void otm_print_cmdline_option(void);

/*
 * Description: hdmi helper function to print edid information
 *		from report_edid tool
 *
 * Returns:	none
 */
void test_otm_hdmi_report_edid_full(void);

#endif /* __ANDROID_HDMI_H */
