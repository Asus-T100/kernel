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

#include <linux/delay.h>
#include "otm_hdmi.h"
#include "ipil_hdmi.h"
#include "ipil_utils.h"
#include "ipil_internal.h"
#include "ips_hdmi.h"

/* TMDS clocks in KHz*/
#define IPS_MIN_PIXEL_CLOCK 25174	/* 640x480@59.94Hz */
#define IPS_MAX_PIXEL_CLOCK 148500	/* 1920x1080@60Hz */

/* Preferred mode if none is indicated in EDID */
#define IPS_PREFERRED_HDISPLAY 1920
#define IPS_PREFERRED_VDISPLAY 1080
#define IPS_PREFERRED_REFRESH_RATE 60

struct data_rate_divider_selector_list_t {
	uint32_t target_data_rate;
	int m1;
	int m2;
	int n;
	int p1;
	int p2;
};
static struct data_rate_divider_selector_list_t
	data_rate_divider_selector_list[] = {
	{25200, 2, 105, 1, 2, 16},
	{27000, 3, 75, 1, 2, 16},
	{27027, 3, 75, 1, 2, 16},
	{28320, 2, 118, 1, 2, 16},
	{31500, 2, 123, 1, 3, 10},
	{40000, 2, 125, 1, 3, 8},
	{49500, 2, 116, 1, 3, 6},
	{65000, 3, 79, 1, 2, 7},
	{74250, 2, 145, 1, 3, 5},
	{74481, 3, 97, 1, 3, 5},
	{108000, 3, 75, 1, 2, 4},
	{135000, 2, 141, 1, 2, 4},
	{148352, 3, 103, 1, 2, 4},
	{148500, 2, 116, 1, 3, 2}
};

#define NUM_SELECTOR_LIST (sizeof( \
		data_rate_divider_selector_list) \
	/ sizeof(struct data_rate_divider_selector_list_t))

/* DPLL registers on IOSF */
#define PLLA_DWORD3_1   0x800C
#define PLLA_DWORD3_2   0x802C
#define PLLA_DWORD5_1   0x8014
#define PLLA_DWORD5_2   0x8034
#define PLLA_DWORD7_1   0x801C
#define PLLA_DWORD7_2   0x803C
#define PLLB_DWORD8     0x8040
#define PLLB_DWORD10_1  0x8048
#define PLLB_DWORD10_2  0x8068
#define CMN_DWORD3      0x810C
#define CMN_DWORD8      0x8100
#define REF_DWORD18     0x80C0
#define REF_DWORD22     0x80D0
#define DPLL_CML_CLK1   0x8238
#define DPLL_CML_CLK2   0x825C
#define DPLL_LRC_CLK    0x824C
#define DPLL_Tx_GRC     0x8244
#define PCS_DWORD12_1   0x0230
#define PCS_DWORD12_2   0x0430
#define TX_SWINGS_1     0x8294
#define TX_SWINGS_2     0x8290
#define TX_SWINGS_3     0x8288
#define TX_SWINGS_4     0x828C
#define TX_SWINGS_5     0x0690
#define TX_SWINGS_6     0x822C
#define TX_SWINGS_7     0x8224

#define DPLL_IOSF_EP 0x13

/**
 * Description: Write to DPLL register via IOSF
 *
 * @ep_id:	IOSF endpoint ID (0x13 for DPLL)
 * @reg:        address of register
 * @val:        value to write to register
 *
 * Returns:	none
 */
void gunit_iosf_write32(u32 ep_id, u32 reg, u32 val)
{
	u32 ret;
	int retry = 0;
	u32 sb_pkt = (1 << 16) | (ep_id << 8) | 0xf0;

	/* Write value to side band register */
	hdmi_write32(0x2108, reg);
	hdmi_write32(0x2104, val);
	hdmi_write32(0x2100, sb_pkt);

	/* Check if transaction is complete */
	ret = hdmi_read32(0x210C);
	while ((retry++ < 0x1000) && (ret != 0x2)) {
		usleep_range(500, 1000);
		ret = hdmi_read32(0x210C);
	}

	if (ret != 2)
		pr_err("%s: failed to program DPLL\n", __func__);
}

/**
 * Description: Read DPLL register via IOSF
 *
 * @ep_id:	IOSF endpoint ID (0x13 for DPLL)
 * @reg:        address of register
 *
 * Returns:	value of register
 */
u32 gunit_iosf_read32(u32 ep_id, u32 reg)
{
	u32 ret;
	int retry = 0;
	u32 sb_pkt = (0 << 16) | (ep_id << 8) | 0xf0;

	/* Read side band register */
	hdmi_write32(0x2108, reg);
	hdmi_write32(0x2100, sb_pkt);

	/* Check if transaction is complete */
	ret = hdmi_read32(0x210C);
	while ((retry < 0x1000) && (ret != 2)) {
		usleep_range(500, 1000);
		ret = hdmi_read32(0x210C);
	}

	if (ret != 2)
		pr_err("%s: Failed to read\n", __func__);
	else
		ret = hdmi_read32(0x2104);

	return ret;
}

/**
 * Description: Find the m, n and p for DPLL.
 *              Use the nominal pixel clock as TMDS clock.
 *
 * @dclk:	refresh rate dot clock in kHz of current mode
 * @real_clk:   nominal dot clock used as TMDS dot clock. Note it
 *              has a small difference from real HW clock.
 * @m1, m2:     DPLL m values
 * @n:          DPLL n value
 * @p1, p2:     DPLL p values
 *
 * Returns:	true on success
 *		false on NULL input arguments
 */
static bool __ips_hdmi_get_divider_selector(
			uint32_t dclk,
			uint32_t *real_dclk,
			int *m1, int *m2,
			int *n, int *p1, int *p2)
{
	int i;

	for (i = 0; i < NUM_SELECTOR_LIST; i++) {
		if (dclk <=
			data_rate_divider_selector_list[i].target_data_rate) {
			*real_dclk =
			data_rate_divider_selector_list[i].target_data_rate;
			*m1 = data_rate_divider_selector_list[i].m1;
			*m2 = data_rate_divider_selector_list[i].m2;
			*n = data_rate_divider_selector_list[i].n;
			*p1 = data_rate_divider_selector_list[i].p1;
			*p2 = data_rate_divider_selector_list[i].p2;
			return true;
		}
	}

	pr_err("Could not find supported mode\n");
	return false;
}

/**
 * Description: programs dpll clocks, enables dpll and waits
 *		till it locks with DSI PLL
 *
 * @m1, m2:     DPLL m values
 * @n:          DPLL n value
 * @p1, p2:     DPLL p values
 *
 * Returns:	none
*/
static void __ips_hdmi_set_program_dpll(int n, int p1, int p2, int m1, int m2)
{
	u32 ret, tmp;
	int retry = 0;
	u32 div = (0x11 << 24) | (p1 << 21) | (p2 << 16) | (n << 12) |
		  (0x1 << 11)  | (m1 << 8)  | (m2);

	pr_debug("enter %s\n", __func__);

	/* Common reset */
	hdmi_write32(IPS_DPLL_B, 0x70006800);

	/* Program DPLL registers via IOSF (TNG display HAS) */

	/* Process monitor to 19.2MHz */
	gunit_iosf_write32(DPLL_IOSF_EP, REF_DWORD22, 0x19080000);

	/* LRC clock to 19.2MHz */
	gunit_iosf_write32(DPLL_IOSF_EP, DPLL_LRC_CLK, 0x00000F10);

	/* Disable periodic GRC IREF update for DPLL */
	tmp = gunit_iosf_read32(DPLL_IOSF_EP, PLLB_DWORD8);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLB_DWORD8, tmp & 0x00FFFFFF);

	/* Enable Tx for periodic GRC update*/
	gunit_iosf_write32(DPLL_IOSF_EP, DPLL_Tx_GRC, 0x0100000F);

	/* GRC cal clock set to 19.2MHZ */
	gunit_iosf_write32(DPLL_IOSF_EP, REF_DWORD18, 0x30002400);

	/* Set lock time to 53us.
	 * Disable fast lock.
	 */
	gunit_iosf_write32(DPLL_IOSF_EP, CMN_DWORD8, 0x0);

	/* Set divisors*/
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD3_1, div);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD3_2, div);

	/* Set up LCPLL in digital mode */
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD5_1, 0x0DF44300);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD5_2, 0x0DF44300);

	/* LPF co-efficients for LCPLL in digital mode */
	gunit_iosf_write32(DPLL_IOSF_EP, PLLB_DWORD10_1, 0x005F0021);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLB_DWORD10_2, 0x005F0021);

	/* Disable unused TLine clocks on right side */
	gunit_iosf_write32(DPLL_IOSF_EP, CMN_DWORD3, 0x14540000);

	/* Enable DPLL */
	tmp = hdmi_read32(IPS_DPLL_B);
	hdmi_write32(IPS_DPLL_B, tmp | IPIL_DPLL_VCO_ENABLE);

	/* Enable DCLP to core */
	tmp = gunit_iosf_read32(DPLL_IOSF_EP, PLLA_DWORD7_1);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD7_1, tmp | (1 << 24));
	tmp = gunit_iosf_read32(DPLL_IOSF_EP, PLLA_DWORD7_2);
	gunit_iosf_write32(DPLL_IOSF_EP, PLLA_DWORD7_2, tmp | (1 << 24));

	/* Set HDMI lane CML clock */
	gunit_iosf_write32(DPLL_IOSF_EP, DPLL_CML_CLK1, 0x07760018);
	gunit_iosf_write32(DPLL_IOSF_EP, DPLL_CML_CLK2, 0x00400888);

	/* Swing settings */
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_1, 0x00000000);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_2, 0x2B407055);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_3, 0x55A0983A);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_4, 0x0C782040);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_5, 0x2B247878);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_6, 0x00030000);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_7, 0x00004000);
	gunit_iosf_write32(DPLL_IOSF_EP, TX_SWINGS_1, 0x80000000);

	/* Stagger Programming */
	gunit_iosf_write32(DPLL_IOSF_EP, PCS_DWORD12_1, 0x00401F00);
	gunit_iosf_write32(DPLL_IOSF_EP, PCS_DWORD12_2, 0x00451F00);

	/* Wait until DPLL is locked */
	ret = hdmi_read32(IPS_DPLL_B);
	ret &= 0x8000;
	while ((retry++ < 1000) && (ret != 0x8000)) {
		usleep_range(500, 1000);
		ret = hdmi_read32(IPS_DPLL_B);
		ret &= 0x8000;
	}

	if (ret != 0x8000) {
		pr_err("%s: DPLL failed to lock, exit...\n", __func__);
		return;
	}
}

/**
 * Description: programs dpll clocks, enables dpll and waits
 *		till it locks with DSI PLL
 *
 * @dev:	hdmi_device_t
 * @dclk:	refresh rate dot clock in kHz of current mode
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t	ips_hdmi_crtc_mode_set_program_dpll(hdmi_device_t *dev,
							unsigned long dclk)
{
	int n, p1, p2, m1, m2;
	uint32_t target_dclk;

	pr_debug("enter %s\n", __func__);

	if (__ips_hdmi_get_divider_selector(dclk,
			&target_dclk, &m1, &m2, &n, &p1, &p2)) {
		__ips_hdmi_set_program_dpll(n, p1, p2, m1, m2);
		dev->clock_khz = target_dclk;
		return OTM_HDMI_SUCCESS;
	} else
		return OTM_HDMI_ERR_INVAL;
}

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

