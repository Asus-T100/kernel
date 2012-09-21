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

#ifndef __MFLD_HDMI_REG_H
#define __MFLD_HDMI_REG_H

/* Video Data Island Packet Control */
#define IPS_HDMI_VID_DIP_CTL_ADDR		(0x61170)

#define IPS_HDMI_VID_EN_DIP			((1) << 31)
#define IPS_HDMI_VID_PORT_SELECT_MASK		((0x3) << 29)
#define IPS_HDMI_VID_PORT_B_SELECT		((1) << 29)

/* Video DIP Type Values Bits 24:21 */
#define IPS_HDMI_EN_DIP_TYPE_MASK		((0xF) << 21)
#define IPS_HDMI_EN_DIP_TYPE_AVI		((1) << 21)
#define IPS_HDMI_EN_DIP_TYPE_VS			((1) << 22)
#define IPS_HDMI_EN_DIP_TYPE_SPD		((1) << 24)

/* Video DIP Type Buffer Index Bits 20:19 */
#define IPS_HDMI_DIP_BUFF_INDX_MASK		((0x3) << 19)
#define IPS_HDMI_DIP_BUFF_INDX_AVI		((0x0) << 19)
#define IPS_HDMI_DIP_BUFF_INDX_VS		((0x1) << 19)
#define IPS_HDMI_DIP_BUFF_INDX_SPD		((0x3) << 19)

/* Video Dip Transmission Frequency 17:16 */
#define IPS_HDMI_DIP_TX_FREQ_SHIFT		(16)
#define IPS_HDMI_DIP_TRANSMISSION_FREQ_MASK	((0x3) << 16)

/* Video Dip Access Address 3:0 */
#define IPS_HDMI_DIP_ACCESS_ADDR_MASK		(0xF)

/* Video Dip Data Register */
#define IPS_HDMI_VIDEO_DIP_DATA_ADDR		(0x61178)

/* Audio Data Island Packet Control */
#define IPS_HDMI_AUD_DIP_CTL_ADDR		(0x69060)

#endif /* MFLD_HDMI_REG_H */
