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

#include "ips_hdmi.h"

#include <linux/types.h>

/* TODO: remove later */
#include "hdmi_hal_common.h"
#include "soc_hdmi_common.h"

static hdmi_device_t *hdmi_dev;

otm_hdmi_ret_t ips_hdmi_set_hdmi_dev(hdmi_device_t *dev)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	if (!dev)
		rc = OTM_HDMI_ERR_NULL_ARG;
	else
		hdmi_dev = dev;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_decide_I2C_HW(hdmi_context_t *ctx)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
#ifdef SOC_HW_I2C_ENABLE
	ctx->hw_i2c =
	    (g_hw_i2c == ~0) ? (ctx->dev.id != HDMI_PCI_REV_CE3100) : g_hw_i2c;
#else
	/* Use SW I2C */
	ctx->hw_i2c = 0;
#endif
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_5V_enable(hdmi_device_t *dev)
{
	return TOGGLE_BIT(dev, BIT03, HDMI_UNIT_CONTROL, GDL_TRUE);
}

otm_hdmi_ret_t ips_hdmi_general_5V_disable(hdmi_device_t *dev)
{
	return TOGGLE_BIT(dev, BIT03, HDMI_UNIT_CONTROL, GDL_FALSE);
}

otm_hdmi_ret_t ips_hdmi_set_program_clocks(hdmi_context_t *ctx,
						unsigned int dclk)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_audio_init(hdmi_context_t *ctx)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_unit_enable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_unit_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_hdcp_clock_enable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_hdcp_clock_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_audio_clock_enable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_audio_clock_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_pixel_clock_enable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_pixel_clock_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_tdms_clock_enable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_general_tdms_clock_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_i2c_disable(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_enable_infoframe(hdmi_device_t *dev,
		unsigned int type, otm_hdmi_packet_t *pkt, unsigned int freq)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_disable_infoframe(hdmi_device_t *dev,
					unsigned int type)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

otm_hdmi_ret_t ips_hdmi_disable_all_infoframes(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	return rc;
}

void ips_hdmi_save_display_registers(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

void ips_hdmi_save_data_island(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

void ips_hdmi_restore_and_enable_display(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

void ips_hdmi_restore_data_island(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

void ips_hdmi_destroy_saved_data(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

void ips_disable_hdmi(hdmi_device_t *dev)
{
	/* TODO: TO BE IMPLEMENTED */
}

bool ips_hdmi_power_rails_on(void)
{
	/* TODO: TO BE IMPLEMENTED */
	return true;
}
