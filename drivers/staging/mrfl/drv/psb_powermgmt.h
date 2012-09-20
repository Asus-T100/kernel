/**************************************************************************
 * Copyright (c) 2009, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Benjamin Defnet <benjamin.r.defnet@intel.com>
 *    Rajesh Poornachandran <rajesh.poornachandran@intel.com>
 *
 */
#ifndef _PSB_POWERMGMT_H_
#define _PSB_POWERMGMT_H_

#include <linux/pci.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <linux/intel_mid_pm.h>
#include "psb_drv.h"

#define OSPM_GRAPHICS_ISLAND	APM_GRAPHICS_ISLAND
#define OSPM_VIDEO_DEC_ISLAND	APM_VIDEO_DEC_ISLAND
#define OSPM_VIDEO_ENC_ISLAND	APM_VIDEO_ENC_ISLAND
#define OSPM_VIDEO_VPP_ISLAND   0x80
#define OSPM_DISPLAY_ISLAND	0x40
/*
	As per TNG Punit HAS.
*/
#define GFX_SS_PM0		0x30
#define GFX_SS_PM1		0x31
#define VED_SS_PM0		0x32
#define VED_SS_PM1		0x33
#define VEC_SS_PM0		0x34
#define VEC_SS_PM1		0x35
#define DSP_SS_PM		0x36
#define VSP_SS_PM0		0x37
#define VSP_SS_PM1		0x38
#define MIO_SS_PM		0x3b
#define HDMIO_SS_PM		0x3c
#define NC_PM_SSS		0x3f

/*
	gfx sub-systems
*/
#define DSPASSC			0x3
#define DSPBSSC			0xc
#define DSPCSSC			0x30
#define DSPASSS			0x3000000
#define DSPBSSS			0xc000000
#define DSPCSSS			0x30000000
#define MIOSSC			0x3
#define MIOSSS			0x3000000
#define HDMIOSSC		0x3
#define HDMIOSSS		0x3000000
#define GFX_SLC_SSC		0x3
#define GFX_SDKCK_SSC		0xc
#define GFX_RSCD_SSC		0x30
#define GFX_SLC_LDO_SSC		0xc0
#define GFX_SLC_SSS		0x3000000
#define GFX_SDKCK_SSS		0xc000000
#define GFX_RSCD_SSS		0x30000000
#define GFX_SLC_LDO_SSS		0xc0000000
/*
	video sub-system

	SSS--Subsystem Status (These bits read ONLY)
	SSC--Subsystem Control
	     00--i0,No clock gating or power gating
	     01--i1,Clock gated
	     10--i2,Soft reset
	     11--D3,Power gated with no HW state retention
 */
#define VSP_SSC			0x3
#define VED_SSC			0x3
#define VEC_SSC			0x3
#define VSP_SSS			0x3000000
#define VED_SSS			0x3000000
#define VEC_SSS			0x3000000


#define PUNIT_PORT		0x04
enum MRFLD_GFX_ISLANDS {
	MRFLD_GFX_DSPA = 0,
	MRFLD_GFX_DSPB,
	MRFLD_GFX_DSPC,
	MRFLD_GFX_MIO,
	MRFLD_GFX_HDMIO,
	MRFLD_GFX_SLC,
	MRFLD_GFX_SDKCK,
	MRFLD_GFX_RSCD,
	MRFLD_GFX_SLC_LDO,
	MRFLD_GFX_ALL_ISLANDS,
};

#define POWER_ISLAND_DOWN 0
#define POWER_ISLAND_UP 1

#define OSPM_ALL_ISLANDS	((OSPM_GRAPHICS_ISLAND) |\
				(OSPM_VIDEO_ENC_ISLAND) |\
				(OSPM_VIDEO_DEC_ISLAND) |\
				(OSPM_VIDEO_VPP_ISLAND) |\
				(OSPM_DISPLAY_ISLAND))

/* IPC message and command defines used to enable/disable mipi panel voltages */
#define IPC_MSG_PANEL_ON_OFF    0xE9
#define IPC_CMD_PANEL_ON        1
#define IPC_CMD_PANEL_OFF       0

/* Panel presence */
#define DISPLAY_A 0x1
#define DISPLAY_B 0x2
#define DISPLAY_C 0x4

extern int drm_psb_dsr;
extern bool gbgfxsuspended;
extern int lastFailedBrightness;
extern struct drm_device *gpDrmDevice;

enum UHBUsage {
	OSPM_UHB_ONLY_IF_ON = 0,
	OSPM_UHB_FORCE_POWER_ON,
};

struct mdfld_dsi_config;
void mdfld_save_display(struct drm_device *dev);
void mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on);
void mdfld_dsi_dbi_set_power(struct drm_encoder *encoder, bool on);
/* extern int psb_check_msvdx_idle(struct drm_device *dev); */
/* extern int lnc_check_topaz_idle(struct drm_device *dev); */
/* Use these functions to power down video HW for D0i3 purpose  */
void ospm_apm_power_down_msvdx(struct drm_device *dev);
void ospm_apm_power_down_topaz(struct drm_device *dev);
void ospm_apm_power_down_vsp(struct drm_device *dev);

void ospm_power_init(struct drm_device *dev);
void ospm_post_init(struct drm_device *dev);
void ospm_power_uninit(void);
void ospm_subsystem_no_gating(struct drm_device *dev, int subsystem);
void ospm_subsystem_power_gate(struct drm_device *dev, int subsystem);

/*
 * OSPM will call these functions
 */
int ospm_power_suspend(struct pci_dev *pdev, pm_message_t state);
int ospm_power_resume(struct pci_dev *pdev);

/*
 * These are the functions the driver should use to wrap all hw access
 * (i.e. register reads and writes)
 */
bool ospm_power_using_hw_begin(int hw_island, enum UHBUsage usage);
void ospm_power_using_hw_end(int hw_island);

/*
 * Use this function to do an instantaneous check for if the hw is on.
 * Only use this in cases where you know the g_state_change_mutex
 * is already held such as in irq install/uninstall and you need to
 * prevent a deadlock situation.  Otherwise use ospm_power_using_hw_begin().
 */
bool ospm_power_is_hw_on(int hw_islands);

/*
 * Power up/down different hw component rails/islands
 */
void mdfld_save_display(struct drm_device *dev);
void ospm_power_island_down(int hw_islands);
void ospm_power_island_up(int hw_islands);
void ospm_suspend_graphics(void);
int mrfld_set_power_state(int islands, int sub_islands, int state_type);
/*
 * GFX-Runtime PM callbacks
 */
int psb_runtime_suspend(struct device *dev);
int psb_runtime_resume(struct device *dev);
int psb_runtime_idle(struct device *dev);
int ospm_runtime_pm_allow(struct drm_device *dev);
void ospm_runtime_pm_forbid(struct drm_device *dev);

#endif /*_PSB_POWERMGMT_H_*/
