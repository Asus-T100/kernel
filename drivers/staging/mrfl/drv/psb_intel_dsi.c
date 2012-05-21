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
 *	jim liu <jim.liu@intel.com>
 */

#include <linux/backlight.h>
#include <linux/version.h>
#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <asm/intel_scu_ipc.h>
/* #include <asm/mrst.h> */
#include <linux/pm_runtime.h>

#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"

#ifndef CONFIG_X86_MRST
/* Added by Alex to work around kernel config issue*/
int mfld_board_type(void)
{
	return MFLD_BOARD_UNKNOWN;
}
#endif

#define DRM_MODE_ENCODER_MIPI  5
/* #define DRM_MODE_CONNECTOR_MIPI		       13 */

#define BRIGHTNESS_MAX_LEVEL 100
#define BLC_POLARITY_NORMAL 0

uint8_t blc_pol2;
uint8_t blc_freq2;
int dsi_backlight;		/* restore backlight to this value */
int dsi_backlight2;		/* restore backlight to this value */

/**
 * Returns the maximum level of the backlight duty cycle field.
 */
static u32 mrst_dsi_get_max_backlight(struct drm_device *dev)
{
	PSB_DEBUG_ENTRY("\n");

	return BRIGHTNESS_MAX_LEVEL;

}

/**
 * Sets the power state for the panel.
 */
static void mrst_dsi_set_power(struct drm_device *dev,
			       struct psb_intel_output *output, bool on)
{
	/* u32 pp_status; */
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	DRM_INFO("Enter mrst_dsi_set_power \n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		/* program MIPI DSI controller and Display Controller
		 * set the device ready bit + set 'turn on' bit b048
		 * wait for 100 ms ??
		 * set pipe enable bit */
		REG_WRITE(DPI_CONTROL_REG, DPI_TURN_ON);
		msleep(100);

		if (dev_priv->panel_make == TPO_864X480)
			dev_priv->init_drvIC(dev);	/* initialize the panel */
		/* Turn on backlight */
		REG_WRITE(BLC_PWM_CTL, 0x2faf1fc9);
		dev_priv->is_mipi_on = true;
	} else {
		/* set the shutdown bit b048h
		 * de-assert pipe enable
		 * clear device ready bit unless DBI is to be left on */
		REG_WRITE(BLC_PWM_CTL, 0x2faf0000);
		REG_WRITE(DPI_CONTROL_REG, 1);
		dev_priv->is_mipi_on = false;
		pm_request_idle(&dev->pdev->dev);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mrst_dsi_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);

	PSB_DEBUG_ENTRY("%s \n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (mode == DRM_MODE_DPMS_ON)
		mrst_dsi_set_power(dev, output, true);
	else
		mrst_dsi_set_power(dev, output, false);
}

static void mrst_dsi_connector_dpms(struct drm_connector *connector, int mode)
{
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (!dev_priv->rpm_enabled) {
		drm_helper_connector_dpms(connector, mode);
		return;
	}

	pm_runtime_forbid(&dev->pdev->dev);
	drm_helper_connector_dpms(connector, mode);
	pm_runtime_allow(&dev->pdev->dev);
}

static void mrst_dsi_save(struct drm_connector *connector)
{
	PSB_DEBUG_ENTRY("\n");

}

static void mrst_dsi_restore(struct drm_connector *connector)
{
	PSB_DEBUG_ENTRY("\n");

}

static void mrst_dsi_prepare(struct drm_encoder *encoder)
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

	mrst_dsi_set_power(dev, output, false);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mrst_dsi_commit(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct psb_intel_mode_device *mode_dev = output->mode_dev;

	PSB_DEBUG_ENTRY("\n");

	if (mode_dev->backlight_duty_cycle == 0)
		mode_dev->backlight_duty_cycle =
		    mrst_dsi_get_max_backlight(dev);

	mrst_dsi_set_power(dev, output, true);
}

#if 0				/* for future reference */
/* ************************************************************************* *\
FUNCTION: GetHS_TX_timeoutCount
DESCRIPTION: In burst mode, value  greater than one DPI line Time in byte clock
	(txbyteclkhs). To timeout this timer 1+ of the
	above said value is recommended.

	In non-burst mode, Value greater than one DPI frame time
	in byte clock(txbyteclkhs).

	To timeout this timer 1+ of the above said value is recommended.

\* ************************************************************************* */
static u32 GetHS_TX_timeoutCount(DRM_DRIVER_PRIVATE_T *dev_priv)
{

	u32 timeoutCount = 0, HTOT_count = 0, VTOT_count = 0, HTotalPixel = 0;

	/* Total pixels need to be transfer per line */
	HTotalPixel = (dev_priv->HsyncWidth +
		       dev_priv->HbackPorch +
		       dev_priv->HfrontPorch) *
	    dev_priv->laneCount + dev_priv->HactiveArea;

	/* byte count = (pixel count *  bits per pixel) / 8 */
	HTOT_count = (HTotalPixel * dev_priv->bpp) / 8;

	if (dev_priv->videoModeFormat == BURST_MODE) {
		timeoutCount = HTOT_count + 1;
#if 1				/*FIXME revisit it. */
		VTOT_count = dev_priv->VactiveArea +
		    dev_priv->VbackPorch +
		    dev_priv->VfrontPorch + dev_priv->VsyncWidth;

		/* timeoutCount = (HTOT_count * VTOT_count) + 1; */
		timeoutCount = (HTOT_count * VTOT_count) + 1;
#endif
	} else {
		VTOT_count = dev_priv->VactiveArea +
		    dev_priv->VbackPorch +
		    dev_priv->VfrontPorch + dev_priv->VsyncWidth;
		/* timeoutCount = (HTOT_count * VTOT_count) + 1; */
		timeoutCount = (HTOT_count * VTOT_count) + 1;
	}

	return timeoutCount & 0xFFFF;
}

/* ************************************************************************* *\
FUNCTION: GetLP_RX_timeoutCount

DESCRIPTION: The timeout value is protocol specific. Time out value is
		calculated from txclkesc(50ns).

	Minimum value =
		Time to send one Trigger message = 4 X txclkesc
					[Escape mode entry sequence)
		+ 8-bit trigger message (2x8xtxclkesc)
		+1 txclksesc [stop_state]
	= 21 X txclkesc [ 15h]

	Maximum Value =
		Time to send a long packet with maximum payload data
			= 4 X txclkesc [Escape mode entry sequence)
		+ 8-bit Low power data transmission Command (2x8xtxclkesc)
		+ packet header [ 4X8X2X txclkesc]
		+payload [ nX8X2Xtxclkesc]
		+CRC[2X8X2txclkesc]
		+1 txclksesc [stop_state]
	= 117 txclkesc +n[payload in terms of bytes]X16txclkesc.

\* ************************************************************************* */
static u32 GetLP_RX_timeoutCount(DRM_DRIVER_PRIVATE_T *dev_priv)
{

	u32 timeoutCount = 0;

	if (dev_priv->config_phase) {
		/* Assuming 256 byte DDB data. */
		timeoutCount = 117 + 256 * 16;
	} else {
		/* For DPI video only mode use the minimum value. */
		timeoutCount = 0x15;
#if 1				/*FIXME revisit it */
		/* Assuming 256 byte DDB data. */
		timeoutCount = 117 + 256 * 16;
#endif
	}

	return timeoutCount;
}
#endif				/* #if 0 - to avoid warnings */

/* ************************************************************************* *\
FUNCTION: GetHSA_Count

DESCRIPTION: Shows the horizontal sync value in terms of byte clock
			(txbyteclkhs)
	Minimum HSA period should be sufficient to transmit a hsync start short
		packet(4 bytes)
		i) For Non-burst Mode with sync pulse, Min value 4 in decimal
			[plus an optional 6 bytes for a zero payload blanking
			 packet]. But if the value is less than 10 but more
			than 4, then this count will be added to the HBP s
			count for one lane.
		ii) For Non-Burst Sync Event & Burst Mode, there is no HSA,
			so you can program this to zero. If you program this
			register, these byte values will be added to HBP.
		iii) For Burst mode of operation, normally the values
			programmed in terms of byte clock are based on the
			principle - time for transfering
			HSA in Burst mode is the same as in non-bust mode.
\* ************************************************************************* */
static u32 GetHSA_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 HSA_count;
	u32 HSA_countX8;

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HSA_countX8 = dev_priv->HsyncWidth * dev_priv->bpp;

	if (dev_priv->videoModeFormat == BURST_MODE) {
		HSA_countX8 *= dev_priv->DDR_Clock /
		    dev_priv->DDR_Clock_Calculated;
	}

	HSA_count = HSA_countX8 / 8;

	/* The above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HSA according to equation:
	   (hsync_width) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* the lower equation = the upper equation / (2 * lane number) */

	HSA_count /= (2 * dev_priv->laneCount);

	if (HSA_count < 4)	/* minimum value of 4 */
		HSA_count = 4;

	PSB_DEBUG_ENTRY("HSA_count is %d\n", HSA_count);

	return HSA_count;
}

/* ************************************************************************* *\
FUNCTION: GetHBP_Count

DESCRIPTION: Shows the horizontal back porch value in terms of txbyteclkhs.
	Minimum HBP period should be sufficient to transmit a �hsync end short
		packet(4 bytes) + Blanking packet overhead(6 bytes) +
		RGB packet header(4 bytes)�
	For Burst mode of operation, normally the values programmed in terms of
		byte clock are based on the principle - time for transfering HBP
		in Burst mode is the same as in non-bust mode.

	Min value � 14 in decimal
		[accounted with zero payload for blanking packet] for one lane.
	Max value � any value greater than 14 based on DPI resolution
\* ************************************************************************* */
static u32 GetHBP_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 HBP_count, HBP_countX8;
	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HBP_countX8 = dev_priv->HbackPorch * dev_priv->bpp;

	if (dev_priv->videoModeFormat == BURST_MODE) {
		HBP_countX8 *= dev_priv->DDR_Clock /
		    dev_priv->DDR_Clock_Calculated;
	}

	HBP_count = HBP_countX8 / 8;

	/* The above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HBP according to equation:
	   (hsync_backporch) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* The lower equation = the upper equation / (2 * lane number) */

	HBP_count /= (2 * dev_priv->laneCount);

	if (HBP_count < 8)	/* minimum value of 8 */
		HBP_count = 8;

	PSB_DEBUG_ENTRY("HBP_count is %d\n", HBP_count);

	return HBP_count;
}

/* ************************************************************************* *\
FUNCTION: GetHFP_Count

DESCRIPTION: Shows the horizontal front porch value in terms of txbyteclkhs.
Minimum HFP period should be sufficient to transmit �RGB Data packet
footer(2 bytes) + Blanking packet overhead(6 bytes)� for non burst mode.

For burst mode, Minimum HFP period should be sufficient to transmit
Blanking packet overhead(6 bytes)�

For Burst mode of operation, normally the values programmed in terms of
	byte clock are based on the principle - time for transfering HFP
	in Burst mode is the same as in non-bust mode.

Min value � 8 in decimal  for non-burst mode [accounted with zero payload
	for blanking packet] for one lane.
Min value � 6 in decimal for burst mode for one lane.

Max value � any value greater than the minimum vaue based on DPI resolution
\* ************************************************************************* */
static u32 GetHFP_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 HFP_count, HFP_countX8;

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HFP_countX8 = dev_priv->HfrontPorch * dev_priv->bpp;

	if (dev_priv->videoModeFormat == BURST_MODE) {
		HFP_countX8 *= dev_priv->DDR_Clock /
		    dev_priv->DDR_Clock_Calculated;
	}

	HFP_count = HFP_countX8 / 8;

	/* The above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HFP according to equation:
	   (hsync_frontporch) * 24 bpp / (2 * 8 bits per lane * 2 lanes) */
	/* The lower equation = the upper equation / (2 * lane number) */

	HFP_count /= (2 * dev_priv->laneCount);

	if (HFP_count < 8)	/* minimum value of 8 */
		HFP_count = 8;

	PSB_DEBUG_ENTRY("HFP_count is %d\n", HFP_count);

	return HFP_count;
}

/* ************************************************************************* *\
FUNCTION: GetHAdr_Count

DESCRIPTION: Shows the horizontal active area value in terms of txbyteclkhs.
	In Non Burst Mode, Count equal to RGB word count value

In Burst Mode, RGB pixel packets are time-compressed, leaving more time
	during a scan line for LP mode (saving power) or for multiplexing
	other transmissions onto the DSI link. Hence, the count equals the
	time in txbyteclkhs for sending  time compressed RGB pixels plus
	the time needed for moving to power save mode or the time needed
	for secondary channel to use the DSI link.

But if the left out time for moving to low power mode is less than
	8 txbyteclkhs [2txbyteclkhs for RGB data packet footer and
	6txbyteclkhs for a blanking packet with zero payload],	then
	this count will be added to the HFP's count for one lane.

Min value � 8 in decimal  for non-burst mode [accounted with zero payload
	for blanking packet] for one lane.
Min value � 6 in decimal for burst mode for one lane.

Max value � any value greater than the minimum vaue based on DPI resolution
\* ************************************************************************* */
static u32 GetHAdr_Count(struct drm_device *dev,
			 DRM_DRIVER_PRIVATE_T *dev_priv)
{
	u32 HAdr_count, HAdr_countX8;

	/* byte clock count = (pixel clock count *  bits per pixel) /8 */
	HAdr_countX8 = dev_priv->HactiveArea * dev_priv->bpp;

	if (dev_priv->videoModeFormat == BURST_MODE) {
		HAdr_countX8 *= dev_priv->DDR_Clock /
		    dev_priv->DDR_Clock_Calculated;
	}

	HAdr_count = HAdr_countX8 / 8;

	/* The above formulus is deduced from the MIPI spec. The following
	   equation comes from HW SV. need to double check it.  */
	/* compute HAdr according to equation:
	   (horizontal active) * 24 bpp / (8 bits per lane * 2 lanes) */
	/* The lower equation = the upper equation / (lane number) */

	HAdr_count /= dev_priv->laneCount;

	PSB_DEBUG_ENTRY("HAdr_count is %d\n", HAdr_count);

	return HAdr_count;
}

/* ************************************************************************* *\
FUNCTION: GetVSA_Count

DESCRIPTION: Shows the vertical sync value in terms of lines

\* ************************************************************************* */
static u32 GetVSA_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 VSA_count;

	/* Get the vsync pulse width */
	VSA_count = dev_priv->VsyncWidth;

	if (VSA_count < 2)	/* minimum value of 2 */
		VSA_count = 2;

	PSB_DEBUG_ENTRY("VSA_count is %d\n", VSA_count);

	return VSA_count;
}

/* ************************************************************************* *\
 * FUNCTION: GetVBP_Count
 *
 * DESCRIPTION: Shows the vertical back porch value in lines.
 *
\* ************************************************************************* */
static u32 GetVBP_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 VBP_count;

	/* Get the Vertical Backporch width */
	VBP_count = dev_priv->VbackPorch;

	if (VBP_count < 2)	/* minimum value of 2 */
		VBP_count = 2;

	PSB_DEBUG_ENTRY("VBP_count is %d\n", VBP_count);

	return VBP_count;
}

/* ************************************************************************* *\
 * FUNCTION: GetVFP_Count
 *
 * DESCRIPTION: Shows the vertical front porch value in terms of lines.
 *
\* ************************************************************************* */
static u32 GetVFP_Count(struct drm_device *dev, DRM_DRIVER_PRIVATE_T * dev_priv)
{
	u32 VFP_count;

	/* Get the Vertical Frontporch width */
	VFP_count = dev_priv->VfrontPorch;

	if (VFP_count < 2)	/* minimum value of 2 */
		VFP_count = 2;

	PSB_DEBUG_ENTRY("VFP_count is %d\n", VFP_count);

	return VFP_count;
}

#if 0				/* For future reference. */
/* ************************************************************************* *\
FUNCTION: GetHighLowSwitchCount

DESCRIPTION: High speed to low power or Low power to high speed switching time
	in terms byte clock (txbyteclkhs). This value is based on the
	byte clock (txbyteclkhs) and low power clock frequency (txclkesc)

Typical value - Number of byte clocks required to switch from low power mode
	to high speed mode after "txrequesths" is asserted.

The worst count value among the low to high or high to low switching time
	in terms of txbyteclkhs  has to be programmed in this register.

Usefull Formulae:
	DDR clock period = 2 times UI
	txbyteclkhs clock = 8 times UI
	Tlpx = 1 / txclkesc
	CALCULATION OF LOW POWER TO HIGH SPEED SWITCH COUNT VALUE
	(from Standard D-PHY spec)

	LP01 + LP00 + HS0 = 1Tlpx + 1Tlpx + 3Tlpx [Approx] +
	1DDR clock [2UI] + 1txbyteclkhs clock [8UI]

	CALCULATION OF	HIGH SPEED TO LOW POWER SWITCH COUNT VALUE
	(from Standard D-PHY spec)

	Ths-trail = 1txbyteclkhs clock [8UI] +
			5DDR clock [10UI] + 4 Tlpx [Approx]
\* ************************************************************************* */
static u32 GetHighLowSwitchCount(DRM_DRIVER_PRIVATE_T *dev_priv)
{
	u32 HighLowSwitchCount, HighToLowSwitchCount, LowToHighSwitchCount;

/* ************************************************************************* *\
CALCULATION OF	HIGH SPEED  TO LOW POWER SWITCH COUNT VALUE
(from Standard D-PHY spec)

Ths-trail = 1txbyteclkhs clock [8UI] + 5DDR clock [10UI] + 4 Tlpx [Approx]

Tlpx = 50 ns, Using max txclkesc (20MHz)

txbyteclkhs_period = 4000 / dev_priv->DDR_Clock; in ns
UI_period = 500 / dev_priv->DDR_Clock; in ns

HS_to_LP = Ths-trail = 18 * UI_period  + 4 * Tlpx
		= 9000 / dev_priv->DDR_Clock + 200;

HighToLowSwitchCount = HS_to_LP / txbyteclkhs_period
	= (9000 / dev_priv->DDR_Clock + 200) / (4000 / dev_priv->DDR_Clock)
		= (9000 + (200 *  dev_priv->DDR_Clock)) / 4000

\* ************************************************************************* */
	HighToLowSwitchCount = (9000 + (200 * dev_priv->DDR_Clock)) / 4000 + 1;

/* ************************************************************************* *\
CALCULATION OF LOW POWER TO HIGH SPEED SWITCH COUNT VALUE
(from Standard D-PHY spec)

LP01 + LP00 + HS0 = 1Tlpx + 1Tlpx + 3Tlpx [Approx] +
1DDR clock [2UI] + 1txbyteclkhs clock [8UI]

	LP_to_HS = 10 * UI_period + 5 * Tlpx =
			= 5000 / dev_priv->DDR_Clock + 250;

	LowToHighSwitchCount = LP_to_HS / txbyteclkhs_period
			= (5000 / dev_priv->DDR_Clock + 250) /
			  (4000 / dev_priv->DDR_Clock)

			= (5000 + (250 *  dev_priv->DDR_Clock)) / 4000

\* ************************************************************************* */
	LowToHighSwitchCount = (5000 + (250 * dev_priv->DDR_Clock)) / 4000 + 1;

	if (HighToLowSwitchCount > LowToHighSwitchCount)
		HighLowSwitchCount = HighToLowSwitchCount;
	else
		HighLowSwitchCount = LowToHighSwitchCount;

	/* FIXME jliu need to fine tune the above formulae and remove the
	 * following after power on */
	if (HighLowSwitchCount < 0x1f)
		HighLowSwitchCount = 0x1f;

	return HighLowSwitchCount;
}

/* ************************************************************************* *\
FUNCTION: mrst_gen_long_write
DESCRIPTION:
\* ************************************************************************* */
static void mrst_gen_long_write(struct drm_device *dev,
				u32 *data, u16 wc, u8 vc)
{
	u32 gen_data_reg = HS_GEN_DATA_REG;
	u32 gen_ctrl_reg = HS_GEN_CTRL_REG;
	u32 date_full_bit = HS_DATA_FIFO_FULL;
	u32 control_full_bit = HS_CTRL_FIFO_FULL;
	u16 wc_saved = wc;

	PSB_DEBUG_ENTRY("Enter mrst_gen_long_write \n");

	/* sanity check */
	if (vc > 4) {
		DRM_ERROR
		    (KERN_ERR "MIPI Virtual channel Can't greater than 4.\n");
		return;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	while (wc >= 4) {
		/* Check if MIPI IP generic data fifo is not full */
		while ((REG_READ(GEN_FIFO_STAT_REG) & date_full_bit)
		       == date_full_bit) {
			/* Do Nothing Here */
			/* This will make checkpatch work */
		}

		/* write to data buffer */
		REG_WRITE(gen_data_reg, *data);

		wc -= 4;
		data++;
	}

	switch (wc) {
	case 1:
		REG_WRITE8(gen_data_reg, *((u8 *) data));
		break;
	case 2:
		REG_WRITE16(gen_data_reg, *((u16 *) data));
		break;
	case 3:
		REG_WRITE16(gen_data_reg, *((u16 *) data));
		data = (u32 *) ((u8 *) data + 2);
		REG_WRITE8(gen_data_reg, *((u8 *) data));
		break;
	}

	/* Check if MIPI IP generic control fifo is not full */
	while ((REG_READ(GEN_FIFO_STAT_REG) & control_full_bit)
	       == control_full_bit) {
		/* Do Nothing Here */
		/* This will make Checkpatch work */
	}
	/* write to control buffer */
	REG_WRITE(gen_ctrl_reg, 0x29 | (wc_saved << 8) | (vc << 6));

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/* ************************************************************************* *\
FUNCTION: mrst_init_HIMAX_MIPI_bridge
DESCRIPTION:
\* ************************************************************************* */
static void mrst_init_HIMAX_MIPI_bridge(struct drm_device *dev)
{
	u32 gen_data[2];
	u16 wc = 0;
	u8 vc = 0;
	u32 gen_data_intel = 0x200105;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	/* exit sleep mode */
	wc = 0x5;
	gen_data[0] = gen_data_intel | (0x11 << 24);
	gen_data[1] = 0;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_pixel_format */
	gen_data[0] = gen_data_intel | (0x3A << 24);
	gen_data[1] = 0x77;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* Set resolution for (800X480) */
	wc = 0x8;
	gen_data[0] = gen_data_intel | (0x2A << 24);
	gen_data[1] = 0x1F030000;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[0] = gen_data_intel | (0x2B << 24);
	gen_data[1] = 0xDF010000;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* System control */
	wc = 0x6;
	gen_data[0] = gen_data_intel | (0xEE << 24);
	gen_data[1] = 0x10FA;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* INPUT TIMING FOR TEST PATTERN(800X480) */
	/* H-size */
	gen_data[1] = 0x2000;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0301;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* V-size */
	gen_data[1] = 0xE002;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0103;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* H-total */
	gen_data[1] = 0x2004;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0405;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* V-total */
	gen_data[1] = 0x0d06;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0207;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* H-blank */
	gen_data[1] = 0x0308;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0009;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* H-blank */
	gen_data[1] = 0x030A;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x000B;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* H-start */
	gen_data[1] = 0xD80C;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x000D;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* V-start */
	gen_data[1] = 0x230E;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x000F;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* RGB domain */
	gen_data[1] = 0x0027;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* INP_FORM Setting */
	/* set_1 */
	gen_data[1] = 0x1C10;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_2 */
	gen_data[1] = 0x0711;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_3 */
	gen_data[1] = 0x0012;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_4 */
	gen_data[1] = 0x0013;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_5 */
	gen_data[1] = 0x2314;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_6 */
	gen_data[1] = 0x0015;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_7 */
	gen_data[1] = 0x2316;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_8 */
	gen_data[1] = 0x0017;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_1 */
	gen_data[1] = 0x0330;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC Setting */
	/* FRC_set_2 */
	gen_data[1] = 0x237A;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC_set_3 */
	gen_data[1] = 0x4C7B;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC_set_4 */
	gen_data[1] = 0x037C;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC_set_5 */
	gen_data[1] = 0x3482;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC_set_7 */
	gen_data[1] = 0x1785;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* OUTPUT TIMING FOR TEST PATTERN (800X480) */
	/* out_htotal */
	gen_data[1] = 0x2090;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0491;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_hsync */
	gen_data[1] = 0x0392;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0093;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_hstart */
	gen_data[1] = 0xD894;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0095;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_hsize */
	gen_data[1] = 0x2096;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0397;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_vtotal */
	gen_data[1] = 0x0D98;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x0299;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_vsync */
	gen_data[1] = 0x039A;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x009B;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_vstart */
	gen_data[1] = 0x239C;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x009D;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* out_vsize */
	gen_data[1] = 0xE09E;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x019F;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* FRC_set_6 */
	gen_data[1] = 0x9084;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* Other setting */
	gen_data[1] = 0x0526;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* RBG domain */
	gen_data[1] = 0x1177;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* rgbw */
	/* set_1 */
	gen_data[1] = 0xD28F;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_2 */
	gen_data[1] = 0x02D0;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_3 */
	gen_data[1] = 0x08D1;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_4 */
	gen_data[1] = 0x05D2;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_5 */
	gen_data[1] = 0x24D4;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* set_6 */
	gen_data[1] = 0x00D5;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x02D7;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x00D8;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	gen_data[1] = 0x48F3;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0xD4F2;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x3D8E;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x60FD;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x00B5;
	mrst_gen_long_write(dev, gen_data, wc, vc);
	gen_data[1] = 0x48F4;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	/* inside patten */
	gen_data[1] = 0x0060;
	mrst_gen_long_write(dev, gen_data, wc, vc);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}
#endif

void mrst_wait_for_INTR_PKT_SENT(struct drm_device *dev)
{
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (!(REG_READ(INTR_STAT_REG) & SPL_PKT_SENT))) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO
		    ("MIPI: SPL_PKT_SENT_INTERRUPT was not set correctly!\n");
}

static void mrst_wait_for_PIPEA_DISABLE(struct drm_device *dev)
{
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(0x70008) & 0x40000000)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: PIPEA was not disabled!\n");
}

static void mrst_wait_for_DPI_CTRL_FIFO(struct drm_device *dev)
{
	int timeout = 0;
	udelay(500);

#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000)
	       && ((REG_READ(GEN_FIFO_STAT_REG) & DPI_FIFO_EMPTY)
		   != DPI_FIFO_EMPTY)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: DPI FIFO was never cleared!\n");
}

static void mrst_wait_for_LP_CTRL_FIFO(struct drm_device *dev)
{
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(GEN_FIFO_STAT_REG) &
				     LP_CTRL_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: LP CMD FIFO was never cleared!\n");
}

static void mrst_wait_for_HS_DATA_FIFO(struct drm_device *dev)
{
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(GEN_FIFO_STAT_REG) &
				     HS_DATA_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: HS Data FIFO was never cleared!\n");
}

static void mrst_wait_for_HS_CTRL_FIFO(struct drm_device *dev)
{
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(GEN_FIFO_STAT_REG) &
				     HS_CTRL_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}
	if (timeout == 20000)
		DRM_INFO("MIPI: HS CMD FIFO was never cleared!\n");
}

/* ************************************************************************* *\
FUNCTION: mrst_init_NSC_MIPI_bridge
DESCRIPTION:	This function is called only by mrst_dsi_mode_set and
		restore_display_registers.  since this function does not
		acquire the mutex, it is important that the calling function
		does!
\* ************************************************************************* */
void mrst_init_NSC_MIPI_bridge(struct drm_device *dev)
{

	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("\n");

	/* Program MIPI IP to 100MHz DSI, Non-Burst mode with sync event,
	   2 Data Lanes */

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* enable RGB24 */
	REG_WRITE(LP_GEN_CTRL_REG, 0x003205e3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* enable all error reporting */
	REG_WRITE(LP_GEN_CTRL_REG, 0x000040e3);
	mrst_wait_for_LP_CTRL_FIFO(dev);
	REG_WRITE(LP_GEN_CTRL_REG, 0x000041e3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* enable 2 data lane; video shaping & error reporting */
	REG_WRITE(LP_GEN_CTRL_REG, 0x00a842e3);	/* 0x006842e3 for 1 data lane */

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* HS timeout */
	REG_WRITE(LP_GEN_CTRL_REG, 0x009243e3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* setle = 6h; low power timeout = ((2^21)-1)*4TX_esc_clks. */
	REG_WRITE(LP_GEN_CTRL_REG, 0x00e645e3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* enable all virtual channels */
	REG_WRITE(LP_GEN_CTRL_REG, 0x000f46e3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* set output strength to low-drive */
	REG_WRITE(LP_GEN_CTRL_REG, 0x00007de3);

	mrst_wait_for_LP_CTRL_FIFO(dev);
	switch (dev_priv->core_freq) {
	case 100:
		/* set escape clock to divede by 16 */
		REG_WRITE(LP_GEN_CTRL_REG, 0x001044e3);
		break;
	case 166:
		/* set escape clock to divede by 8 */
		REG_WRITE(LP_GEN_CTRL_REG, 0x000044e3);
		break;
	case 200:
		/* set escape clock to divede by 32 */
		/*REG_WRITE(LP_GEN_CTRL_REG, 0x003044e3); */
		REG_WRITE(LP_GEN_CTRL_REG, 0x001044e3);

		/*mrst_wait_for_LP_CTRL_FIFO(dev); */
		/* setle = 6h; low power timeout = ((2^21)-1)*4TX_esc_clks. */
		/*REG_WRITE(LP_GEN_CTRL_REG, 0x00ec45e3); */
		break;
	}

	mrst_wait_for_LP_CTRL_FIFO(dev);
	/* CFG_VALID=1; RGB_CLK_EN=1. */
	REG_WRITE(LP_GEN_CTRL_REG, 0x00057fe3);

	/*ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND); */
}

static int mrst_check_mipi_error(struct drm_device *dev)
{
	u32 int_status_reg = 0;
	u32 relevant_error_bits = 0x0fff;	/* only care about error bits 0-11 */
	u32 reported_errors = 0;

	mrst_wait_for_LP_CTRL_FIFO(dev);
	REG_WRITE(LP_GEN_CTRL_REG, 0x010524);	/* 2-parameter gen short read */

	/* sleep 100 microseconds */
	udelay(100);

	int_status_reg = REG_READ(INTR_STAT_REG);
	printk(KERN_ALERT "MIPI Intr Status Reg: 0x%X\n", int_status_reg);

	reported_errors = int_status_reg & relevant_error_bits;
	if (reported_errors) {
		printk(KERN_ALERT "MIPI Init sequence reported errs: 0x%X\n",
		       reported_errors);
		/* Clear the error bits */
		REG_WRITE(INTR_STAT_REG, reported_errors);
		return reported_errors;
	}

	return 0;
}

/* ************************************************************************* *\
 * FUNCTION: mrst_init_TPO_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
void mrst_init_TPO_MIPI(struct drm_device *dev)
{
	/*DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private; */

	DRM_INFO("Enter mrst init TPO MIPI display.\n");

	/* Flip page order */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00008036);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000229);

	/* 0xF0 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x005a5af0);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000329);

	/* Write protection key */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x005a5af1);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000329);

	/* 0xFC */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x005a5afc);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000329);

	/* 0xB7 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x770000b7);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000044);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000529);

	/* 0xB6 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x000a0ab6);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000329);

	/* 0xF2 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x081010f2);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x4a070708);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x000000c5);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000929);

	/* 0xF8 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x024003f8);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x01030a04);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x0e020220);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000004);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000d29);

	/* 0xE2 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x398fc3e2);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x0000916f);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000629);

	/* 0xB0 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x000000b0);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000229);

	/* 0xF4 */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x240242f4);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x78ee2002);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x2a071050);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x507fee10);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x10300710);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00001429);

	/* 0xBA */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x19fe07ba);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x101c0a31);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000010);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000929);

	/* 0xBB */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x28ff07bb);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x24280a31);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000034);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000929);

	/* 0xFB */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x535d05fb);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1b1a2130);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x221e180e);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x131d2120);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x535d0508);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1c1a2131);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x231f160d);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x111b2220);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x535c2008);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1f1d2433);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x2c251a10);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x2c34372d);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000023);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00003129);

	/* 0xFA */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x525c0bfa);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1c1c232f);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x2623190e);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x18212625);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x545d0d0e);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1e1d2333);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x26231a10);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x1a222725);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x545d280f);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x21202635);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x31292013);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x31393d33);
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x00000029);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00003129);

	/* Set DM */
	mrst_wait_for_HS_DATA_FIFO(dev);
	REG_WRITE(0xb068, 0x000100f7);
	mrst_wait_for_HS_CTRL_FIFO(dev);
	REG_WRITE(0xb070, 0x00000329);
}

/* ************************************************************************* *\
 * FUNCTION: mrst_init_LGE_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *		 restore_display_registers.  since this function does not
 *		 acquire the mutex, it is important that the calling function
 *		 does!
\* ************************************************************************* */
void mrst_init_LGE_MIPI(struct drm_device *dev)
{
	/*DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private; */
	int i = 0;

	DRM_INFO("Enter mrst init LGE MIPI display.\n");

	mrst_wait_for_LP_CTRL_FIFO(dev);
	REG_WRITE(0xb06c, 0x00870123);

	/* LGE 480x1024 Panel Initialization sequence */
	for (i = 0; i < 10; i++) {
		/* Panel Characteristics Settings */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xb2200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x0ec820);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x7 << 8 | 0x0 << 6);

		/* Panel Driver Setting */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xb3200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x5 << 8 | 0x0 << 6);

		/* Display Mode Control */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xb4200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x5 << 8 | 0x0 << 6);

		/* Display Mode and Frame Memory write Mode Setting */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xb5200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x000f0f12);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x9 << 8 | 0x0 << 6);

		/* Display Control (GIP Specific) */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xb6200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x40021803);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x3010);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xa << 8 | 0x0 << 6);

		/* Power Setting */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xc0200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x1f01);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x6 << 8 | 0x0 << 6);

		/* Power Setting */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xc3200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x03040407);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x07);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x9 << 8 | 0x0 << 6);

		/*   */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xc4200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x15154412);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x6d04);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xa << 8 | 0x0 << 6);

		/*   */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xc5200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x64);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x5 << 8 | 0x0 << 6);

		/*   */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xc6200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x004024);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x7 << 8 | 0x0 << 6);

		/* red */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd0200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd1200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		/* green */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd2200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd3200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		/* blue */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd4200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0xd5200105);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x06774701);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x00200000);
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x02);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0xd << 8 | 0x0 << 6);

		if (!mrst_check_mipi_error(dev)) {
			i = 0;
			break;
		}
	}

	for (i = 0; i < 10; i++) {
		/* Sleep Out  */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x11200105);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x4 << 8 | 0x0 << 6);

		if (!mrst_check_mipi_error(dev)) {
			i = 0;
			break;
		}
	}

	udelay(10000);

	for (i = 0; i < 10; i++) {
		/* Display On */
		mrst_wait_for_HS_DATA_FIFO(dev);
		REG_WRITE(HS_GEN_DATA_REG, 0x29200105);
		mrst_wait_for_HS_CTRL_FIFO(dev);
		REG_WRITE(HS_GEN_CTRL_REG, 0x29 | 0x4 << 8 | 0x0 << 6);

		if (!mrst_check_mipi_error(dev)) {
			i = 0;
			break;
		}
	}

	/*ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND); */
}

/*enum mipi_panel_type {
	NSC_800X480 = 0,
	LGE_480X1024 = 1,
	TPO_864X480 = 2
};*/

static void mrst_dsi_mode_set(struct drm_encoder *encoder,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode)
{
	struct psb_intel_mode_device *mode_dev =
	    enc_to_psb_intel_output(encoder)->mode_dev;
	struct drm_device *dev = encoder->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	u32 dsiFuncPrgValue = 0;
	u32 SupportedFormat = 0;
	u32 resolution = 0;
	u32 mipi_control_val = 0;
	u32 intr_en_val = 0;
	u32 turnaround_timeout_val = 0;
	u32 device_reset_val = 0;
	u32 init_count_val = 0;
	u32 hs_tx_timeout_val = 0;
	u32 lp_rx_timeout_val = 0;
	u32 high_low_switch_count_val = 0;
	u32 eot_disable_val = 0;
	u32 lp_byteclk_val = 0;
	u32 device_ready_val = 0;
	/*u32 dpi_control_val = 0; */
	u32 vsa_count = 0;
	u32 vbp_count = 0;
	u32 vfp_count = 0;
	u32 hsa_count = 0;
	u32 hbp_count = 0;
	u32 hfp_count = 0;
	u32 haa_count = 0;
	u32 video_mode_format = 0;
	u32 max_ret_packet_size = 0;
	uint64_t curValue = DRM_MODE_SCALE_FULLSCREEN;
	u32 mipi_port;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	switch (dev_priv->bpp) {
	case 16:
		SupportedFormat = RGB_565_FMT;
		break;
	case 18:
		SupportedFormat = RGB_666_FMT;
		break;
	case 24:
		SupportedFormat = RGB_888_FMT;
		break;
	default:
		DRM_INFO("mrst_dsi_mode_set,  invalid bpp \n");
		break;
	}

	if (dev_priv->dpi) {
		drm_connector_property_get_value(&enc_to_psb_intel_output
						 (encoder)->base,
						 dev->
						 mode_config.scaling_mode_property,
						 &curValue);

		if (curValue == DRM_MODE_SCALE_NO_SCALE)
			REG_WRITE(PFIT_CONTROL, 0);
		else if (curValue == DRM_MODE_SCALE_ASPECT) {
			if ((mode->vdisplay != adjusted_mode->crtc_vdisplay) ||
			    (mode->hdisplay != adjusted_mode->crtc_hdisplay)) {
				if ((adjusted_mode->crtc_hdisplay *
				     mode->vdisplay) == (mode->hdisplay *
							 adjusted_mode->crtc_vdisplay))
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
		} else		/*(curValue == DRM_MODE_SCALE_FULLSCREEN) */
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE);

		switch (dev_priv->panel_make) {
		case NSC_800X480:
			intr_en_val = 0xffffffff;
			turnaround_timeout_val = 0x00000001;
			device_reset_val = 0x000000ff;
			init_count_val = 0x00000fff;
			resolution = dev_priv->HactiveArea |
			    (dev_priv->VactiveArea << RES_V_POS);
			SupportedFormat <<= FMT_DPI_POS;
			dsiFuncPrgValue = dev_priv->laneCount | SupportedFormat;
			vsa_count = GetVSA_Count(dev, dev_priv);
			vbp_count = GetVBP_Count(dev, dev_priv);
			vfp_count = GetVFP_Count(dev, dev_priv);
			hsa_count = GetHSA_Count(dev, dev_priv);
			hbp_count = GetHBP_Count(dev, dev_priv);
			hfp_count = GetHFP_Count(dev, dev_priv);
			haa_count = GetHAdr_Count(dev, dev_priv);
			video_mode_format = dev_priv->videoModeFormat;
			hs_tx_timeout_val = 0x00001000;
			lp_rx_timeout_val = 0x0000ffff;
			high_low_switch_count_val = 0x46;
			eot_disable_val = 0x00000000;
			lp_byteclk_val = 0x00000004;
			device_ready_val = 0x00000001;
			max_ret_packet_size = 0x40;
			break;
		case TPO_864X480:
			intr_en_val = 0xffffffff;
			turnaround_timeout_val = 0x0000000a;
			device_reset_val = 0x000000ff;
			init_count_val = 0x00000fff;
			resolution = 0x01e00360;
			dsiFuncPrgValue = 0x00000202;
			vsa_count = 0x00000004;
			vbp_count = 0x00000008;
			vfp_count = 0x00000008;
			hsa_count = 0x00000006;
			hbp_count = 0x0000000f;
			hfp_count = 0x0000000f;
			haa_count = 0x00000510;
			video_mode_format = 0x00000003;
			hs_tx_timeout_val = 0x00090000;
			lp_rx_timeout_val = 0x0000ffff;
			high_low_switch_count_val = 0x00000046;
			eot_disable_val = 0x00000000;
			lp_byteclk_val = 0x00000004;
			device_ready_val = 0x00000001;
			max_ret_packet_size = 0x40;
			break;
		case LGE_480X1024:
			intr_en_val = 0xffffffff;
			turnaround_timeout_val = 0x00000012;
			device_reset_val = 0x000000ff;
			init_count_val = 0x00000fff;
			resolution = 0x040001e0;
			dsiFuncPrgValue = 0x00000202;
			vsa_count = 0x00000005;
			vbp_count = 0x0000000f;
			vfp_count = 0x0000000f;
			hsa_count = 0x00000008;
			hbp_count = 0x00000018;
			hfp_count = 0x0000000f;
			haa_count = 0x00000320;
			video_mode_format = 0x00000003;
			hs_tx_timeout_val = 0x00ffffff;
			lp_rx_timeout_val = 0x0000ffff;
			high_low_switch_count_val = 0x00000016;
			eot_disable_val = 0x00000000;
			lp_byteclk_val = 0x00000004;
			device_ready_val = 0x00000001;
			max_ret_packet_size = 0x40;
			break;
		}

		/* set 100 mhz dsi clk based on gfx core freq */
		switch (dev_priv->core_freq) {
		case 100:
			mipi_control_val = 0x0019;	/* 50 mhz * 2 = 100 mhz */
			break;
		case 166:
			mipi_control_val = 0x0018;	/* 100 mhz * 1 = 100 mhz */
			break;
		case 200:
			mipi_control_val = 0x0018;	/* 100 mhz * 1 = 100 mhz */
			break;
		}

		/* wait for PIPE A to disable */
		mrst_wait_for_PIPEA_DISABLE(dev);

		/* wait for DPI FIFO to clear */
		mrst_wait_for_DPI_CTRL_FIFO(dev);

		/* Clear Device Ready Bit */
		REG_WRITE(DEVICE_READY_REG, 0x00000000);

		/* Enable MIPI Port */
		mipi_port = MIPI_PORT_EN | MIPI_BORDER_EN;

		/* Enable dithering if required */
		if (mode_dev->panel_wants_dither)
			mipi_port |= MRST_PANEL_8TO6_DITHER_ENABLE;

		REG_WRITE(MIPI, mipi_port);

		/* set the lane speed */
		REG_WRITE(MIPI_CONTROL_REG, mipi_control_val);

		/* Enable all the error interrupt */
		REG_WRITE(INTR_EN_REG, intr_en_val);
		REG_WRITE(TURN_AROUND_TIMEOUT_REG, turnaround_timeout_val);
		REG_WRITE(DEVICE_RESET_REG, device_reset_val);
		REG_WRITE(INIT_COUNT_REG, init_count_val);

		REG_WRITE(DSI_FUNC_PRG_REG, dsiFuncPrgValue);

		REG_WRITE(DPI_RESOLUTION_REG, resolution);
		/*REG_WRITE(DBI_RESOLUTION_REG, 0x00000000); */

		REG_WRITE(VERT_SYNC_PAD_COUNT_REG, vsa_count);
		REG_WRITE(VERT_BACK_PORCH_COUNT_REG, vbp_count);
		REG_WRITE(VERT_FRONT_PORCH_COUNT_REG, vfp_count);

		REG_WRITE(HORIZ_SYNC_PAD_COUNT_REG, hsa_count);
		REG_WRITE(HORIZ_BACK_PORCH_COUNT_REG, hbp_count);
		REG_WRITE(HORIZ_FRONT_PORCH_COUNT_REG, hfp_count);
		REG_WRITE(HORIZ_ACTIVE_AREA_COUNT_REG, haa_count);

		REG_WRITE(VIDEO_FMT_REG, video_mode_format);

		REG_WRITE(HS_TX_TIMEOUT_REG, hs_tx_timeout_val);
		REG_WRITE(LP_RX_TIMEOUT_REG, lp_rx_timeout_val);

		REG_WRITE(HIGH_LOW_SWITCH_COUNT_REG, high_low_switch_count_val);

		REG_WRITE(EOT_DISABLE_REG, eot_disable_val);

		REG_WRITE(LP_BYTECLK_REG, lp_byteclk_val);
		REG_WRITE(MAX_RET_PAK_REG, max_ret_packet_size);

		REG_WRITE(DEVICE_READY_REG, device_ready_val);
		REG_WRITE(DPI_CONTROL_REG, DPI_TURN_ON);
	}

	if ((REG_READ(INTR_STAT_REG) & SPL_PKT_SENT)) {
		REG_WRITE(INTR_STAT_REG, SPL_PKT_SENT);
	}
	mrst_wait_for_INTR_PKT_SENT(dev);

	if ((dev_priv->panel_make == NSC_800X480)
	    || (dev_priv->panel_make == LGE_480X1024))
		dev_priv->init_drvIC(dev);	/* initialize the mipi panel */

	/* set the dphy settings for 100 mhz */
	REG_WRITE(0xb080, 0x0b061c04);

	REG_WRITE(PIPEACONF, dev_priv->pipeconf);
	/* REG_READ(PIPEACONF); */

	/* Wait for 20ms for the pipe enable to take effect. */
	/*udelay(20000); */

	REG_WRITE(DSPACNTR, dev_priv->dspcntr);

	/* Wait for 20ms for the plane enable to take effect. */
	/*udelay(20000); */

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/**
 * Detect the MIPI connection.
 *
 * This always returns CONNECTOR_STATUS_CONNECTED.
 * This connector should only have
 * been set up if the MIPI was actually connected anyway.
 */
static enum drm_connector_status mrst_dsi_detect(struct drm_connector
						 *connector)
{
	PSB_DEBUG_ENTRY("\n");

	return connector_status_connected;
}

/**
 * Return the list of MIPI DDB modes if available.
 */
static int mrst_dsi_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct psb_intel_mode_device *mode_dev = psb_intel_output->mode_dev;
	struct drm_display_mode *panel_fixed_mode = mode_dev->panel_fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if (psb_intel_output->type == INTEL_OUTPUT_MIPI2)
		panel_fixed_mode = mode_dev->panel_fixed_mode2;

/* FIXME get the MIPI DDB modes */

	/* Didn't get an DDB, so
	 * Set wide sync ranges so we get all modes
	 * handed to valid_mode for checking
	 */
	connector->display_info.min_vfreq = 0;
	connector->display_info.max_vfreq = 200;
	connector->display_info.min_hfreq = 0;
	connector->display_info.max_hfreq = 200;

	if (panel_fixed_mode != NULL) {
		struct drm_display_mode *mode =
		    drm_mode_duplicate(dev, panel_fixed_mode);
		drm_mode_probed_add(connector, mode);
		return 1;
	}

	return 0;
}

static const struct drm_encoder_helper_funcs mrst_dsi_helper_funcs = {
	.dpms = mrst_dsi_dpms,
	.mode_fixup = psb_intel_lvds_mode_fixup,
	.prepare = mrst_dsi_prepare,
	.mode_set = mrst_dsi_mode_set,
	.commit = mrst_dsi_commit,
};

static const struct drm_connector_helper_funcs mrst_dsi_connector_helper_funcs = {
	.get_modes = mrst_dsi_get_modes,
	.mode_valid = psb_intel_lvds_mode_valid,
	.best_encoder = psb_intel_best_encoder,
};

static const struct drm_connector_funcs mrst_dsi_connector_funcs = {
	.dpms = mrst_dsi_connector_dpms,
	.save = mrst_dsi_save,
	.restore = mrst_dsi_restore,
	.detect = mrst_dsi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = psb_intel_lvds_set_property,
	.destroy = psb_intel_lvds_destroy,
};

static struct drm_display_mode dsi_configuration_modes[] = {
	/* LGE 480x1024 tentative timings */
	{DRM_MODE("480x1024", DRM_MODE_TYPE_DRIVER, 33264, 480, 499,
		  506, 517, 0, 1024, 1039, 1041, 1047, 0, 0)},
	/* copy from SV - hard coded fixed mode for DSI TPO 3.8" panel */
	{DRM_MODE("864x480", DRM_MODE_TYPE_DRIVER, 33264, 864, 873,
		  876, 887, 0, 480, 487, 490, 499, 0, 0)},
	/* hard coded fixed mode for DSI TPO TD043MTEA2 LCD panel */
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
static struct drm_display_mode *mrst_dsi_get_configuration_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode = NULL;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	u8 panel_index = dev_priv->gct_data.bpi;
	u8 panel_type = dev_priv->gct_data.pt;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false;

	PSB_DEBUG_ENTRY("\n");

	if (dev_priv->vbt_data.Size != 0x00)	/*if non-zero, vbt is present */
		if ((1 << panel_index) & panel_type)	/* if non-zero, */
			use_gct = true;	/*then mipi panel. */

	if (use_gct) {
		PSB_DEBUG_ENTRY("gct find MIPI panel. \n");

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

		PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
		PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
		PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
		PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
		PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
		PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
		PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
		PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
		PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);
	} else
		mode = drm_mode_duplicate(dev, &dsi_configuration_modes[1]);

	dev_priv->pixelClock = mode->clock;	/*KHz */
	dev_priv->HsyncWidth = mode->hsync_end - mode->hsync_start;
	dev_priv->HbackPorch = mode->htotal - mode->hsync_end;
	dev_priv->HfrontPorch = mode->hsync_start - mode->hdisplay;
	dev_priv->HactiveArea = mode->hdisplay;
	dev_priv->VsyncWidth = mode->vsync_end - mode->vsync_start;
	dev_priv->VbackPorch = mode->vtotal - mode->vsync_end;
	dev_priv->VfrontPorch = mode->vsync_start - mode->vdisplay;
	dev_priv->VactiveArea = mode->vdisplay;

	PSB_DEBUG_ENTRY("pixelClock is %d\n", dev_priv->pixelClock);
	PSB_DEBUG_ENTRY("HsyncWidth is %d\n", dev_priv->HsyncWidth);
	PSB_DEBUG_ENTRY("HbackPorch is %d\n", dev_priv->HbackPorch);
	PSB_DEBUG_ENTRY("HfrontPorch is %d\n", dev_priv->HfrontPorch);
	PSB_DEBUG_ENTRY("HactiveArea is %d\n", dev_priv->HactiveArea);
	PSB_DEBUG_ENTRY("VsyncWidth is %d\n", dev_priv->VsyncWidth);
	PSB_DEBUG_ENTRY("VbackPorch is %d\n", dev_priv->VbackPorch);
	PSB_DEBUG_ENTRY("VfrontPorch is %d\n", dev_priv->VfrontPorch);
	PSB_DEBUG_ENTRY("VactiveArea is %d\n", dev_priv->VactiveArea);

	/* DPI MIPI display */
	dev_priv->dpi = true;

	/* Should get the lane count from GCT or from the attached MIPI panel spec. */
	dev_priv->laneCount = 2;
	dev_priv->bpp = 24;

	dev_priv->videoModeFormat = NON_BURST_MODE_SYNC_EVENTS;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

/* ************************************************************************* *\
FUNCTION: mrstDSI_clockInit
DESCRIPTION:

\* ************************************************************************* */
static u32 sku_83_mipi_2xclk[4] = { 166667, 333333, 444444, 666667 };
static u32 sku_100_mipi_2xclk[4] = { 200000, 400000, 533333, 800000 };
static u32 sku_100L_mipi_2xclk[4] = { 100000, 200000, 266667, 400000 };

#define MIPI_2XCLK_COUNT			0x04

static bool mrstDSI_clockInit(DRM_DRIVER_PRIVATE_T *dev_priv)
{
	u32 Htotal = 0, Vtotal = 0, RRate = 0, mipi_2xclk = 0;
	u32 i = 0;
	u32 *p_mipi_2xclk = NULL;

	Htotal =
	    dev_priv->HsyncWidth + dev_priv->HbackPorch +
	    dev_priv->HfrontPorch + dev_priv->HactiveArea;
	Vtotal =
	    dev_priv->VsyncWidth + dev_priv->VbackPorch +
	    dev_priv->VfrontPorch + dev_priv->VactiveArea;

	RRate = ((dev_priv->pixelClock * 1000) / (Htotal * Vtotal)) + 1;

	dev_priv->RRate = RRate;

	/* ddr clock frequence = (pixel clock frequence *  bits per pixel)/2 */
	mipi_2xclk = (dev_priv->pixelClock * dev_priv->bpp) / dev_priv->laneCount;	/* KHz */
	dev_priv->DDR_Clock_Calculated = mipi_2xclk / 2;	/* KHz */

	PSB_DEBUG_ENTRY("mrstDSI_clockInit RRate = %d, mipi_2xclk = %d. \n",
			RRate, mipi_2xclk);

	switch (dev_priv->core_freq) {
	case 100:
		p_mipi_2xclk = sku_100L_mipi_2xclk;
		break;
	case 166:
		p_mipi_2xclk = sku_83_mipi_2xclk;
		break;
	case 200:
		p_mipi_2xclk = sku_100_mipi_2xclk;
		break;
	}

	for (; i < MIPI_2XCLK_COUNT; i++) {
		if ((dev_priv->DDR_Clock_Calculated * 2) < p_mipi_2xclk[i])
			break;
	}

	if (i == MIPI_2XCLK_COUNT) {
		PSB_DEBUG_ENTRY
		    ("mrstDSI_clockInit the DDR clock is too big, DDR_Clock_Calculated is = %d\n",
		     dev_priv->DDR_Clock_Calculated);
		return false;
	}

	dev_priv->DDR_Clock = p_mipi_2xclk[i] / 2;
	dev_priv->ClockBits = i;

	PSB_DEBUG_ENTRY
	    ("mrstDSI_clockInit, mipi_2x_clock_divider = 0x%x, DDR_Clock_Calculated is = %d\n",
	     i, dev_priv->DDR_Clock_Calculated);

	return true;
}

/**
 * mrst_dsi_init - setup MIPI connectors on this device
 * @dev: drm device
 *
 * Create the connector, try to figure out what
 * modes we can display on the MIPI panel (if present).
 */
void mrst_dsi_init(struct drm_device *dev,
		   struct psb_intel_mode_device *mode_dev)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;

#ifdef CONFIG_X86_MRST
	if (mrst_platform_id() == MRST_PLATFORM_AAVA_SC) {
		aava_koski_dsi_init(dev, mode_dev);
		return;
	}
#endif

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	dev_priv->is_mipi_on = true;
	drm_connector_init(dev, &psb_intel_output->base,
			   &mrst_dsi_connector_funcs, DRM_MODE_CONNECTOR_MIPI);

	drm_encoder_init(dev, &psb_intel_output->enc, &psb_intel_lvds_enc_funcs,
			 DRM_MODE_ENCODER_MIPI);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type = INTEL_OUTPUT_MIPI;

	drm_encoder_helper_add(encoder, &mrst_dsi_helper_funcs);
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

	dsi_backlight = BRIGHTNESS_MAX_LEVEL;
	blc_pol = BLC_POLARITY_NORMAL;
	blc_freq = 0xc8;

	mode_dev->panel_wants_dither = false;
	if (dev_priv->vbt_data.Size != 0x00) {
		mode_dev->panel_wants_dither =
		    (dev_priv->
		     gct_data.Panel_MIPI_Display_Descriptor & (BIT3 | BIT4));
		switch (dev_priv->gct_data.bpi) {	/* set panel make */
		case 1:
			dev_priv->panel_make = NSC_800X480;
			break;
		case 2:
			dev_priv->panel_make = TPO_864X480;
			break;
		case 3:
			dev_priv->panel_make = LGE_480X1024;
			break;
		default:
			DRM_INFO("MIPI: unknown panel type!  Setting NSC.\n");
			dev_priv->panel_make = NSC_800X480;	/* assume NSC */
		}
	} else {
		DRM_INFO("MIPI: No GCT! Setting NSC.\n");
		dev_priv->panel_make = NSC_800X480;
	}

	/* set panel initialize function */
	switch (dev_priv->panel_make) {
	case NSC_800X480:
		dev_priv->init_drvIC = mrst_init_NSC_MIPI_bridge;
		break;
	case TPO_864X480:
		dev_priv->init_drvIC = mrst_init_TPO_MIPI;
		break;
	case LGE_480X1024:
		dev_priv->init_drvIC = mrst_init_LGE_MIPI;
		break;
	}

	/*
	 * MIPI discovery:
	 * 1) check for DDB data
	 * 2) check for VBT data
	 * 4) make sure lid is open
	 *    if closed, act like it's not there for now
	 */

	/* FIXME change it to true if GET_DDB works */
	dev_priv->config_phase = false;

	/*
	 * If we didn't get DDB data, try geting panel timing
	 * from configuration data
	 */
	mode_dev->panel_fixed_mode = mrst_dsi_get_configuration_mode(dev);

	if (mode_dev->panel_fixed_mode) {
		mode_dev->panel_fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
	} else {
		/* If we still don't have a mode after all that, give up. */
		DRM_ERROR("Found no modes on the mipi, ignoring the MIPI.\n");
		goto failed_find;
	}

	if (!mrstDSI_clockInit(dev_priv)) {
		DRM_ERROR("Can't iniitialize MRST DSI clock.\n");
	}

	drm_sysfs_connector_add(connector);
	return;

 failed_find:
	DRM_DEBUG("No MIIP modes found, disabling.\n");
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	kfree(connector);
}
