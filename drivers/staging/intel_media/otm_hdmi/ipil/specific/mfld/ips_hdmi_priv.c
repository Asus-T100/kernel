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


#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include "otm_hdmi.h"
#include "hdmi_internal.h"
#include "ips_hdmi.h"
#include "mfld_hdmi_reg.h"
#include "ipil_internal.h"
#include "ipil_utils.h"

#define IPS_MIN_PIXEL_CLOCK 25174	/* 640x480@59.94Hz */
#define IPS_MAX_PIXEL_CLOCK 74250	/* 1920x1080@30Hz */
#define IPS_PREFERRED_HDISPLAY 1920
#define IPS_PREFERRED_VDISPLAY 1080
#define IPS_PREFERRED_REFRESH_RATE 30


/**
 * Description: get pixel clock range
 *
 * @pc_min:	minimum pixel clock
 * @pc_max:	maximum pixel clock
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_FAILED on NULL input arguments.
 */
otm_hdmi_ret_t ips_get_pixel_clock_range(unsigned int *pc_min,
						unsigned int *pc_max)
{
	if (!pc_min || !pc_max)
		return OTM_HDMI_ERR_FAILED;

	*pc_min = IPS_MIN_PIXEL_CLOCK;
	*pc_max = IPS_MAX_PIXEL_CLOCK;
	return OTM_HDMI_SUCCESS;
}

/**
 * Returns if the given values is preferred mode or not
 * @hdisplay	: width
 * @vdisplay	: height
 * @refresh	: refresh rate
 *
 * Returns true if preferred mode else false
 */
bool ips_hdmi_is_preferred_mode(int hdisplay, int vdisplay, int refresh)
{
	if (hdisplay == IPS_PREFERRED_HDISPLAY &&
		vdisplay == IPS_PREFERRED_VDISPLAY &&
		refresh == IPS_PREFERRED_REFRESH_RATE)
		return true;
	else
		return false;
}


