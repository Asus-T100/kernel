/*
 * Copyright © 2006-2007 Intel Corporation
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
 * Authors:
 *	jim liu <jim.liu@intel.com>
 */

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_mode.h>
#include "psb_intel_drv.h"
#include "psb_drv.h"
#include "psb_irq.h"
#include "psb_intel_reg.h"
#include "psb_intel_hdmi_reg.h"
#include "psb_intel_hdmi_edid.h"
#include "psb_intel_hdmi.h"
#include "mdfld_dsi_output.h"
#include "mdfld_hdmi_audio_if.h"
#include <linux/pm_runtime.h>
#include <linux/intel_mid_pm.h>
#include "mdfld_ti_tpd.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
#include <asm/intel_scu_pmic.h>
#endif

/* FIXME_MDFLD HDMI EDID supports */
char EDID_Samsung[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] =
{
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0x2D, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x14, 0x11, 0x01, 0x03, 0x80, 0x10, 0x09, 0x8C, 0x0A, 0xE2, 0xBD, 0xA1, 0x5B, 0x4A, 0x98, 0x24,
	0x15, 0x47, 0x4A, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28,
	0x55, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E, 0x20,
	0xB8, 0x28, 0x55, 0x40, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x31,
	0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x41, 0x4D, 0x53, 0x55, 0x4E, 0x47, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xB3,
	0x02, 0x03, 0x1A, 0x71, 0x46, 0x84, 0x13, 0x05, 0x14, 0x03, 0x12, 0x23, 0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0x66, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x80, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C,
	0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x9E, 0x01, 0x1D, 0x80, 0xD0,
	0x72, 0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x9E, 0x8C, 0x0A,
	0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x18,
	0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00, 0xA0, 0x5A, 0x00, 0x00,
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0
};

char EDID_Samsung_2493HM[EDID_LENGTH] =
{
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x4c, 0x2d, 0x68, 0x03, 0x34, 0x32, 0x49, 0x4b,
	0x0c, 0x12, 0x01, 0x03, 0x0e, 0x34, 0x20, 0xa0, 0x2a, 0xef, 0x91, 0xa3, 0x54, 0x4c, 0x9b, 0x26,
	0x0f, 0x50, 0x54, 0xbf, 0xef, 0x80, 0xa9, 0x40, 0x81, 0x80, 0x81, 0x40, 0x71, 0x4f, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x28, 0x3c, 0x80, 0xa0, 0x70, 0xb0, 0x23, 0x40, 0x30, 0x20,
	0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x38, 0x4b, 0x1e,
	0x51, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x53,
	0x79, 0x6e, 0x63, 0x4d, 0x61, 0x73, 0x74, 0x65, 0x72, 0x0a, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
	0x00, 0x48, 0x56, 0x52, 0x51, 0x33, 0x30, 0x30, 0x35, 0x35, 0x30, 0x0a, 0x20, 0x20, 0x00, 0x20
};

char EDID_Dell[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] =
{
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0xac, 0x5d, 0xa0, 0x55, 0x36, 0x4e, 0x32,
	0x23, 0x14, 0x01, 0x03, 0x80, 0x35, 0x1e, 0x78, 0xee, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26,
	0x0f, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f, 0x81, 0x80, 0xd1, 0xc0, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xff, 0x00, 0x58, 0x31, 0x37,
	0x35, 0x52, 0x30, 0x38, 0x51, 0x32, 0x4e, 0x36, 0x55, 0x0a, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x44,
	0x45, 0x4c, 0x4c, 0x20, 0x53, 0x54, 0x32, 0x34, 0x31, 0x30, 0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd,
	0x00, 0x32, 0x4c, 0x1e, 0x53, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x6d,
	0x02, 0x03, 0x1f, 0xf1, 0x4c, 0x90, 0x05, 0x04, 0x03, 0x02, 0x07, 0x16, 0x01, 0x14, 0x1f, 0x12,
	0x13, 0x23, 0x09, 0x07, 0x07, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x83, 0x01, 0x00, 0x00, 0x02,
	0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00,
	0x1e, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0x13, 0x2b, 0x21,
	0x00, 0x00, 0x9e, 0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96,
	0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b
};

char EDID_Benq[EDID_LENGTH] =
{
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x09, 0xD1, 0x22, 0x79, 0x45, 0x54, 0x00, 0x00,
	0x2A, 0x14, 0x01, 0x03, 0x0E, 0x35, 0x1E, 0x78, 0x2E, 0x60, 0x85, 0xA6, 0x56, 0x4A, 0x9C, 0x25,
	0x12, 0x50, 0x54, 0xA5, 0x6B, 0x80, 0x81, 0x80, 0x81, 0x00, 0x81, 0xC0, 0xA9, 0xC0, 0x81, 0x40,
	0xD1, 0xC0, 0x61, 0xC0, 0xB3, 0x00, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0x13, 0x2B, 0x21, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x50, 0x41, 0x41,
	0x30, 0x31, 0x39, 0x33, 0x38, 0x30, 0x32, 0x36, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x32,
	0x4C, 0x18, 0x53, 0x15, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x42, 0x65, 0x6E, 0x51, 0x20, 0x45, 0x57, 0x32, 0x34, 0x32, 0x30, 0x0A, 0x20, 0x00, 0x8E
};


char EDID_Toshiba_32RV525RZ[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] =
{
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0xac, 0x5d, 0xa0, 0x55, 0x36, 0x4e, 0x32,
	0x23, 0x14, 0x01, 0x03, 0x80, 0x35, 0x1e, 0x78, 0xee, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26,
	0x0f, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f, 0x81, 0x80, 0xd1, 0xc0, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xff, 0x00, 0x58, 0x31, 0x37,
	0x35, 0x52, 0x30, 0x38, 0x51, 0x32, 0x4e, 0x36, 0x55, 0x0a, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x44,
	0x45, 0x4c, 0x4c, 0x20, 0x53, 0x54, 0x32, 0x34, 0x31, 0x30, 0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd,
	0x00, 0x32, 0x4c, 0x1e, 0x53, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x6d,
	0x02, 0x03, 0x1f, 0xf1, 0x4c, 0x90, 0x05, 0x04, 0x03, 0x02, 0x07, 0x16, 0x01, 0x14, 0x1f, 0x12,
	0x13, 0x23, 0x09, 0x07, 0x07, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x83, 0x01, 0x00, 0x00, 0x02,
	0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00,
	0x1e, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0x13, 0x2b, 0x21,
	0x00, 0x00, 0x9e, 0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96,
	0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b
};

char EDID_Toshiba_Regza[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] =
{
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x52, 0x62, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x11, 0x01, 0x03, 0x80, 0x69, 0x3b, 0x78, 0x0a, 0x0d, 0xc9, 0xa0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4c, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10,
	0x10, 0x3e, 0x96, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x54,
	0x53, 0x42, 0x2d, 0x54, 0x56, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfd,
	0x00, 0x17, 0x3d, 0x0f, 0x44, 0x0f, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x03,
	0x02, 0x03, 0x20, 0x77, 0x4a, 0x90, 0x05, 0x04, 0x03, 0x07, 0x02, 0x06, 0x01, 0x16, 0x15, 0x23,
	0x09, 0x07, 0x07, 0x6c, 0x03, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x1e, 0xc0, 0x2b, 0x2b, 0x33, 0x33,
	0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00, 0xc4, 0x8e, 0x21, 0x00,
	0x00, 0x1e, 0x8c, 0x0a, 0xa0, 0x14, 0x51, 0xf0, 0x16, 0x00, 0x26, 0x7c, 0x43, 0x00, 0xc4, 0x8e,
	0x21, 0x00, 0x00, 0x98, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00,
	0x13, 0x8e, 0x21, 0x00, 0x00, 0x18, 0x8c, 0x0a, 0xa0, 0x14, 0x51, 0xf0, 0x16, 0x00, 0x26, 0x7c,
	0x43, 0x00, 0x13, 0x8e, 0x21, 0x00, 0x00, 0x98, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20,
	0x58, 0x2c, 0x25, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x9e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb3
};

static struct hdmi_edid_info mdfld_hdmi_edid[] = {
	{ HDMI_EDID_INFO("SAMSUNG", EDID_Samsung) },
	{ HDMI_EDID_INFO("SAMSUNG_2493HM", EDID_Samsung_2493HM) },
	{ HDMI_EDID_INFO("DELL", EDID_Dell) },
	{ HDMI_EDID_INFO("BENQ", EDID_Benq) },
	{ HDMI_EDID_INFO("TOSHIBA_32RV525RZ", EDID_Toshiba_32RV525RZ) },
	{ HDMI_EDID_INFO("TOSHIBA_REGZA", EDID_Toshiba_Regza) },
};

mdfld_hdmi_timing_t mdfld_hdmi_video_mode_table[] = {
	/* 640*480/60 */
	{640, 480, 60, 25200, 800, 640, 800, 656, 752, 525, 480, 525, 490, 492},
	/* 720*480/60 */
	{720, 480, 60, 27027, 858, 720, 858, 736, 798, 525, 480, 525, 489, 495},
	/* 720*480/60-16:9 */
	{720, 480, 60, 27027, 858, 720, 858, 736, 798, 525, 480, 525, 489, 495},
	/* 1280*720/60 */
	{1280, 720, 60, 74250, 1650, 1280, 1650, 1390, 1430, 750, 720, 750, 725, 730},
	{5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	/* 16 */
	{16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	/* 1280*720/50 */
	{1280, 720, 50, 74250, 1980, 1280, 1980, 1720, 1760, 750, 720, 750, 725, 730},
	{20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	/* 1080p/24 */
	{1920, 1080, 24, 74250, 2750, 1920, 2750, 2558, 2602, 1125, 1080, 1125, 1084, 1089},
	/* 1080p/25 */
	{1920, 1080, 25, 74250, 2640, 1920, 2640, 2448, 2492, 1125, 1080, 1125, 1084, 1089},
	/* 1080p/30 */
	{1920, 1080, 30, 74250, 2200, 1920, 2200, 2008, 2052, 1125, 1080, 1125, 1084, 1089},
	{35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

/*
*VIC for AVI InfoFrame Data Byte 4 and CEA Short Descriptors
*/
struct hdmi_video_format_timing mdfld_hdmi_video_format_timing[] = {
	/*NULL*/
	{0, 0, 0, 60, 0, 1, 1, 0},
	/*640*480p@60hz 4:3*/
	{1, 640, 480, 60, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*480p@60hz 4:3*/
	{2, 720, 480, 60, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*480p@60hz 16:9*/
	{3, 720, 480, 60, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1280*720p@60hz 16:9*/
	{4, 1280, 720, 60, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*1920*1080i@60hz 16:9*/
	{5, 1920, 1080, 60, 1, 1, 1, HDMI_AVI_PAR_16_9},
	/*720*480i@60hz 4:3*/
	{6, 720, 480, 60, 1, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*480i@60hz 16:9*/
	{7, 720, 480, 60, 1, 0, 0, HDMI_AVI_PAR_16_9},
	/*720*240p@60hz 4:3*/
	{8, 720, 240, 60, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*240p@60hz 16:9*/
	{9, 720, 240, 60, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*2880*480i@60hz 4:3*/
	{10, 2880, 480, 60, 1, 0, 0, HDMI_AVI_PAR_4_3},
	/*2880*480i@60hz 16:9*/
	{11, 2880, 480, 60, 1, 0, 0, HDMI_AVI_PAR_16_9},
	/*2880*240p@60hz 4:3*/
	{12, 2880, 240, 60, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*2880*240p@60hz 16:9*/
	{13, 2880, 240, 60, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1440*480p@60hz 4:3*/
	{14, 1440, 480, 60, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*1440*480p@60hz 16:9*/
	{15, 1440, 480, 60, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1920*1080p@60hz 16:9*/
	{16, 1920, 1080, 60, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*720*5760p@50hz 4:3*/
	{17, 720, 576, 50, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*5760p@50hz 16:9*/
	{18, 720, 576, 50, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1280*720p@50hz 16:9*/
	{19, 1280, 720, 50, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*1920*1080i@50hz 16:9*/
	{20, 1920, 1080, 50, 1, 1, 1, HDMI_AVI_PAR_16_9},
	/*720*576i@50hz 4:3*/
	{21, 720, 576, 50, 1, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*576i@50hz 16:9*/
	{22, 720, 576, 50, 1, 0, 0, HDMI_AVI_PAR_16_9},
	/*720*288p@50hz 4:3*/
	{23, 720, 288, 50, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*720*288p@50hz 16:9*/
	{24, 720, 288, 50, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*2880*576i@50hz 4:3*/
	{25, 2880, 576, 50, 1, 0, 0, HDMI_AVI_PAR_4_3},
	/*2880*576i@50hz 16:9*/
	{26, 2880, 576, 50, 1, 0, 0, HDMI_AVI_PAR_16_9},
	/*2880*288p@50hz 4:3*/
	{27, 2880, 288, 50, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*2880*288p@50hz 16:9*/
	{28, 2880, 288, 50, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1440*576p@50hz 4:3*/
	{29, 1440, 576, 50, 0, 0, 0, HDMI_AVI_PAR_4_3},
	/*1440*576p@50hz 16:9*/
	{30, 1440, 576, 50, 0, 0, 0, HDMI_AVI_PAR_16_9},
	/*1920*1080p@50hz 16:9 */
	{31, 1920, 1080, 50, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*1920*1080p@24hz 16:9 */
	{32, 1920, 1080, 24, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*1920*1080p@25hz 16:9 */
	{33, 1920, 1080, 25, 0, 1, 1, HDMI_AVI_PAR_16_9},
	/*1920*1080p@30hz 16:9 */
	{34, 1920, 1080, 30, 0, 1, 1, HDMI_AVI_PAR_16_9},
};

static int mdfld_hdmi_get_cached_edid_block(struct drm_connector *connector,
		uint32_t num_block, uint8_t *edid_block, uint32_t size);

static int mdfld_hdmi_timing_get_vic(struct drm_display_mode *mode)
{
	int i = 0;
	int vic = 0;
	u32 video_code_count = sizeof(mdfld_hdmi_video_format_timing) /
		sizeof(struct hdmi_video_format_timing);
	struct hdmi_video_format_timing vft_table_entry = {0};

	PSB_DEBUG_ENTRY("%s\n", __func__);

	/*find video code for this mode*/
	for (i = 0; i < video_code_count; i++) {
		vft_table_entry = mdfld_hdmi_video_format_timing[i];

		if (mode->hdisplay == vft_table_entry.hdisplay &&
				mode->vdisplay == vft_table_entry.vdisplay &&
				mode->vrefresh == vft_table_entry.refresh &&
				((mode->flags & DRM_MODE_FLAG_INTERLACE) >> 4)
				== vft_table_entry.bInterlace) {
			vic = vft_table_entry.video_code;
			PSB_DEBUG_ENTRY(
			"find hdmi mode video code= %d hpolar=%d vpolar= %d\n",
					vft_table_entry.video_code,
					vft_table_entry.hpolarity,
					vft_table_entry.vpolarity);
			break;
		}
	}

	if (i == video_code_count) {
		PSB_DEBUG_ENTRY(
		"Not supported HDMI mode: width= %d height = %d, vrefresh %d\n.",
			mode->hdisplay, mode->vdisplay, mode->vrefresh);
	}

	return vic;
}

static int mdfld_hdmi_get_aspect_ratio(int h_active, int v_active)
{
	unsigned char aspect_ratio = 0xFF;
	if (!v_active)
		return aspect_ratio;

	if ((h_active * 10) / v_active == 16 * 10 / 10)
		/* 16:10 aspect ratio for EDID 1.3 */
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_10;
	else if ((h_active * 10) / v_active == 4 * 10 / 3)
		/* 4:3 aspect ratio */
		aspect_ratio = EDID_STD_ASPECT_RATIO_4_3;
	else if ((h_active * 10) / v_active ==  5 * 10 / 4)
		/* 5:4 aspect ratio*/
		aspect_ratio = EDID_STD_ASPECT_RATIO_5_4;
	else if ((h_active * 10) / v_active == 16 * 10 / 9)
		/* 16:9 aspect ratio*/
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_9;
	else if (v_active == h_active)
		/*1:1 aspect ratio for EDID prior to EDID 1.3*/
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_10;

	PSB_DEBUG_ENTRY("%s fb_width= %d fb_height= %d aspect_ratio %d\n",
			 __func__, h_active, v_active, aspect_ratio);

	return aspect_ratio;
}

static uint8_t mdfld_hdmi_calc_hbuf_csum(uint8_t *data, uint8_t size)
{
	uint8_t csum = 0;
	int i;

	for (i = 0; i < size; i++)
		csum += data[i];

	return 0x100 - csum;
}

static int mdfld_hdmi_set_avi_infoframe(struct drm_device *dev,
	struct drm_connector *connector, struct drm_display_mode *mode)
{
	u32 buf_size = 0;
	u32 viddipctl_val = 0;
	int i = 0;
	int picture_aspect_ratio = 0xFF, active_aspect_ratio = 0xFF;
	char *edid_block = NULL;
	baseedid_1_4_t *base_edid_block = NULL;
	ce_edid_t *cea_edid_block = NULL;
	int vic = 0;

	if (!connector->edid_blob_ptr)
		return -1;

	if (!connector->edid_blob_ptr->data)
		return -1;

	edid_block = connector->edid_blob_ptr->data;
	/*initialization avi_infoframe*/
	avi_if_t avi_if = {0};
	/*
	union _avi_if avi_if = { {
			0x82, 0x02, 0x0d, 0x00, 0x00, 0x10, 0x18,
			0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00
	} };
	*/
	u32 *p_vsif = (u32 *)&avi_if;

	PSB_DEBUG_ENTRY("%s\n", __func__);

	if(unlikely(!mode))
	{
		PSB_DEBUG_ENTRY("display mode is NULL!\n");
		return 0;
	}

	/*get edid infomation*/
	if (connector && connector->edid_blob_ptr) {
		edid_block = connector->edid_blob_ptr->data;
		cea_edid_block = (ce_edid_t *)(edid_block + EDID_BLOCK_SIZE);
		base_edid_block = (baseedid_1_4_t *)edid_block;
	}

	/*set header information*/
	avi_if.avi_info.avi_if_header.type = HDMI_AVI_TYPE;
	avi_if.avi_info.avi_if_header.version = HDMI_AVI_VERSION2;
	avi_if.avi_info.avi_if_header.length =
			 HDMI_AVI_LENGTH - HDMI_AVI_RESERVED_LENGTH - 1;
	avi_if.avi_info.avi_if_header.ecc = 0;
	avi_if.avi_info.chksum = 0;

	/* Data Byte 1 */
	avi_if.avi_info.byte1_bits.scan_info = HDMI_AVI_SCAN_NODATA;
	avi_if.avi_info.byte1_bits.bar_info = HDMI_AVI_BAR_INVALID;
	avi_if.avi_info.byte1_bits.format = HDMI_AVI_AFI_VALID;
	avi_if.avi_info.byte1_bits.enc_mode = HDMI_AVI_RGB_MODE;
	if (cea_edid_block) {
		if (cea_edid_block->ucCapabilty & BIT7)
			avi_if.avi_info.byte1_bits.scan_info =
				 HDMI_AVI_SCAN_UNDERSCAN;
		/* if set YCbCr, should prepare YUV buffer
		if (cea_edid_block->ucCapabilty & BIT5)
			avi_if.avi_info.byte1_bits.enc_mode =
				 HDMI_AVI_YCRCB444_MODE;
		else if (cea_edid_block->ucCapabilty & BIT4)
			avi_if.avi_info.byte1_bits.enc_mode =
				 HDMI_AVI_YCRCB422_MODE;
		*/
	}
	avi_if.avi_info.byte1_bits.b1rsvd = 0;

	/* Data Byte 2
	* Set aspect ratio
	*/
	avi_if.avi_info.byte2_bits.afar = HDMI_AVI_AFAR_SAME;
	avi_if.avi_info.byte2_bits.par = HDMI_AVI_PAR_NODATA;

	if (mode->width_mm && mode->height_mm) {
		picture_aspect_ratio = mdfld_hdmi_get_aspect_ratio(
			mode->width_mm,	mode->height_mm);
		PSB_DEBUG_ENTRY("PAR caculate by width_mm.\n");
	} else if (base_edid_block &&
			 base_edid_block->ucMaxHIS &&
			 base_edid_block->ucMaxVIS) {
		picture_aspect_ratio = mdfld_hdmi_get_aspect_ratio(
			base_edid_block->ucMaxHIS,
			base_edid_block->ucMaxVIS);
		PSB_DEBUG_ENTRY("PAR caculate by ucMaxHIS.\n");
	} else {
		picture_aspect_ratio = mdfld_hdmi_get_aspect_ratio(
			mode->hdisplay, mode->vdisplay);
		PSB_DEBUG_ENTRY("PAR caculate by hdisplay.\n");
	}
	if (picture_aspect_ratio != 0xFF) {
		switch (picture_aspect_ratio) {
		case EDID_STD_ASPECT_RATIO_4_3:
			avi_if.avi_info.byte2_bits.par =
				HDMI_AVI_PAR_4_3;
			PSB_DEBUG_ENTRY("picture aspect ratio 4:3\n");
			break;
		case EDID_STD_ASPECT_RATIO_16_9:
			avi_if.avi_info.byte2_bits.par =
				HDMI_AVI_PAR_16_9;
			PSB_DEBUG_ENTRY("picture aspect ratio 16:9\n");
			break;
		default:
			avi_if.avi_info.byte2_bits.par =
				HDMI_AVI_PAR_NODATA;
			PSB_DEBUG_ENTRY("picture aspect ratio no data\n");
			break;
		}
	} else
		PSB_DEBUG_ENTRY("picture aspect ratio no data\n");

	avi_if.avi_info.byte2_bits.colorimetry = HDMI_AVI_COLOR_NODATA;
	if (avi_if.avi_info.byte1_bits.enc_mode != HDMI_AVI_RGB_MODE) {
		if (mode->hdisplay <= 720)
			avi_if.avi_info.byte2_bits.colorimetry =
				HDMI_AVI_COLOR_ITU601;
		else
			avi_if.avi_info.byte2_bits.colorimetry =
				HDMI_AVI_COLOR_ITU709;
	}

	/* Data Byte 3 */
	avi_if.avi_info.byte3_bits.scaling_info = HDMI_AVI_SCALING_NODATA;
	avi_if.avi_info.byte3_bits.rgbquant_range = HDMI_AVI_RGBQUANT_DEFAULT;
	avi_if.avi_info.byte3_bits.ext_colorimetry = HDMI_COLORIMETRY_RGB256;
	avi_if.avi_info.byte3_bits.it_content = HDMI_AVI_ITC_NODATA;

	/* Data Byte 4 */
	/*get hdmi mode video code*/
	vic =  mdfld_hdmi_timing_get_vic(mode);
	if (mode->hdisplay == 720 && vic &&
		avi_if.avi_info.byte2_bits.par == HDMI_AVI_PAR_16_9)
		vic += 1;

	avi_if.avi_info.byte4_bits.vic = vic;
	if (avi_if.avi_info.byte4_bits.vic) {
		/*sync the par here with the vic*/
		avi_if.avi_info.byte2_bits.par =
			mdfld_hdmi_video_format_timing[vic].par;
	}
	PSB_DEBUG_ENTRY("set HDMI avi info.vic= %d\n", vic);
	/* Data Byte 5 */
	avi_if.avi_info.byte5_bits.pr = HDMI_PR_ONE;

	/* Other Data Bytes */
	avi_if.avi_info.byte6 = 0x00;
	avi_if.avi_info.byte7 = 0x00;
	avi_if.avi_info.byte8 = 0x00;
	avi_if.avi_info.byte9 = 0x00;
	avi_if.avi_info.byte10 = 0x00;
	avi_if.avi_info.byte11 = 0x00;
	avi_if.avi_info.byte12 = 0x00;
	avi_if.avi_info.byte13 = 0x00;

	/* clear PB14-PB27 reserved data to zero*/
	for (i = 0; i < HDMI_AVI_RESERVED_LENGTH; i++)
		avi_if.avi_info.byte_reserved[i] = 0x00;

	avi_if.avi_info.chksum =
		mdfld_hdmi_calc_hbuf_csum(
			(uint8_t *)avi_if.avi_buf, HDMI_AVI_TOTAL_LENGTH);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return 0;

	/*clear DIP buffer firstly*/
	for (i = 0; i <= DIP_RAM_ADDR_INDEX_MAX; i++)
		REG_WRITE(VIDEO_DIP_DATA, 0x0);

	/* Wait for 2 VSyncs. */
	mdelay(20); /* msleep(20); */
	mdelay(20); /* msleep(20); */
	/* psb_intel_wait_for_vblank(dev); */
	/* Wait for 3 HSync. */
	/* Disable the DIP type . */
	viddipctl_val = REG_READ(VIDEO_DIP_CTL);
	viddipctl_val &= ~(DIP_TYPE_MASK | DIP_BUFF_INDX_MASK |
			DIP_RAM_ADDR_MASK | DIP_TX_FREQ_MASK);
	viddipctl_val |= DIP_BUFF_INDX_AVI | PORT_B_SELECT;
	REG_WRITE(VIDEO_DIP_CTL, viddipctl_val);

	/* Get the buffer size in DWORD. */
	buf_size = HDMI_AVI_TOTAL_LENGTH / 4;

	/* Write HDMI  InfoFrame. */
	for (i = 0; i < buf_size; i++)
		REG_WRITE(VIDEO_DIP_DATA, *(p_vsif++));

	/*Write default DIF data so that Index pointer come back*/
	for (i = buf_size; i <= DIP_RAM_ADDR_INDEX_MAX; i++)
		REG_WRITE(VIDEO_DIP_DATA, 0x00);

	/* Enable the DIP type and transmission frequency. */
	viddipctl_val |= DIP_TYPE_AVI | DIP_TX_FREQ_1VSNC;
	REG_WRITE(VIDEO_DIP_CTL, viddipctl_val);

	viddipctl_val |= EN_DIP;
	REG_WRITE(VIDEO_DIP_CTL, viddipctl_val);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}


static void mdfld_hdmi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	u32 hdmib, hdmi_phy_misc;
	int vic = 0;
	u32 hpolarity = 0, vpolarity = 0;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	void *had_pvt_data = dev_priv->had_pvt_data;
	enum had_event_type event_type = HAD_EVENT_MODE_CHANGING;
#endif

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	hdmi_phy_misc = REG_READ(HDMIPHYMISCCTL) & ~HDMI_PHY_POWER_DOWN;
	REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc);

	hdmib = REG_READ(hdmi_priv->hdmib_reg) | HDMIB_PIPE_B_SELECT;

	if (dev_priv->bDVIport) {
		hdmib &= ~(HDMIB_NULL_PACKET | HDMI_AUDIO_ENABLE) ;
		REG_WRITE(VIDEO_DIP_CTL, 0x0);
		REG_WRITE(AUDIO_DIP_CTL, 0x0);
	} else {
		hdmib |= (HDMIB_NULL_PACKET | HDMI_AUDIO_ENABLE);
		mdfld_hdmi_set_avi_infoframe(dev, &output->base, adjusted_mode);
	}

	vic = mdfld_hdmi_timing_get_vic(adjusted_mode);

	/*Get mode hpolarity vpolarity*/
	if (adjusted_mode->flags & (DRM_MODE_FLAG_PHSYNC |
			DRM_MODE_FLAG_PVSYNC)) {
		hpolarity = 1;
		vpolarity = 1;
		PSB_DEBUG_ENTRY("polarity get from flag P\n");
	} else if (adjusted_mode->flags & (DRM_MODE_FLAG_NHSYNC |
			DRM_MODE_FLAG_NVSYNC)) {
		hpolarity = 0;
		vpolarity = 0;
		PSB_DEBUG_ENTRY("polarity get from flag N\n");
	} else if (vic) {
		hpolarity = mdfld_hdmi_video_format_timing[vic].hpolarity;
		vpolarity = mdfld_hdmi_video_format_timing[vic].vpolarity;
		PSB_DEBUG_ENTRY("polarity get from table\n");
	} else {
		if (adjusted_mode->vdisplay <= 576) {
			hpolarity = 0;
			vpolarity = 0;
		} else {
			hpolarity = 1;
			vpolarity = 1;
		}
	}

	/* Update hpolarity */
	if (hpolarity == 1)
		hdmib |= BIT3;
	else
		hdmib &= ~BIT3;

	/* Update vpolarity */
	if (vpolarity == 1)
		hdmib |= BIT4;
	else
		hdmib &= ~BIT4;

	/*port will be enabled in dpms function
	here make sure it is off before mode set completed */
	hdmib &= ~HDMIB_PORT_EN;

	REG_WRITE(hdmi_priv->hdmib_reg, hdmib);
	REG_READ(hdmi_priv->hdmib_reg);

	/*save current set mode*/
	if (hdmi_priv->current_mode)
		drm_mode_destroy(dev,
				hdmi_priv->current_mode);
	hdmi_priv->current_mode =
		drm_mode_duplicate(dev, adjusted_mode);

#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	/* Send MODE_CHANGE event to Audio driver */
	if (dev_priv->mdfld_had_event_callbacks && !dev_priv->bDVIport)
		(*dev_priv->mdfld_had_event_callbacks)(event_type,
				had_pvt_data);
#endif
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return;
}

static bool mdfld_hdmi_mode_fixup(struct drm_encoder *encoder,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc =
		to_psb_intel_crtc(encoder->crtc);
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct drm_connector *connector = &output->base;
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	uint32_t adjusted_mode_id = 0;
	struct drm_display_mode *temp_mode = NULL, *t = NULL;

	PSB_DEBUG_ENTRY("hdisplay = %d, vdisplay = %d. a_hdisplay = %d,"
			"a_vdisplay = %d.\n", mode->hdisplay, mode->vdisplay,
			adjusted_mode->hdisplay, adjusted_mode->vdisplay);

	/* Should never happen!! */
	if (IS_MID(dev) && psb_intel_crtc->pipe != 1) {
		printk(KERN_ERR
				"Only support HDMI on pipe B on MID\n");
	}

	list_for_each_entry_safe(temp_mode, t, &connector->modes, head) {
		if (drm_mode_equal(temp_mode, mode)) {
			/*update adjusted mode with kernel mode info,
			such as width_mm height_mm*/
			PSB_DEBUG_ENTRY("fix up enter.\n");
			adjusted_mode_id = adjusted_mode->base.id;
			*adjusted_mode = *temp_mode;
			adjusted_mode->base.id = adjusted_mode_id;

			/* Apply the adjusted mode to CRTC mode setting
			 * parameters.*/
			drm_mode_set_crtcinfo(adjusted_mode,
					CRTC_INTERLACE_HALVE_V);
		}
	}

#if 0 /* FIXME hard coded different HDMI timeings, remove them after the MSIC HDMI HW issue is fixed. */
#if 0 /* 720p - Adeel */
	adjusted_mode->hdisplay = 0x500;
	adjusted_mode->htotal = 0x672;
	adjusted_mode->hsync_start = 0x56e;
	adjusted_mode->hsync_end = 0x596;
	adjusted_mode->vdisplay = 0x2d0;
	adjusted_mode->vtotal = 0x2ee;
	adjusted_mode->vsync_start = 0x2d5;
	adjusted_mode->vsync_end = 0x2da;
#endif

#if 0 /* 1080p - Brian */
	adjusted_mode->hdisplay = 0x780;
	adjusted_mode->htotal = 0x898;
	adjusted_mode->hsync_start = 0x7d8;
	adjusted_mode->hsync_end = 0x804;
	adjusted_mode->vdisplay = 0x438;
	adjusted_mode->vtotal = 0x464;
	adjusted_mode->vsync_start = 0x43c;
	adjusted_mode->vsync_end = 0x446;
#endif
#if 0 /* 1080p - Adeel */
	adjusted_mode->hdisplay = 0x780;
	adjusted_mode->htotal = 0xabe;
	adjusted_mode->hsync_start = 0x9fe;
	adjusted_mode->hsync_end = 0xa2a;
	adjusted_mode->vdisplay = 0x438;
	adjusted_mode->vtotal = 0x465;
	adjusted_mode->vsync_start = 0x43c;
	adjusted_mode->vsync_end = 0x441;
#endif
#if 0 /* 480p - Adeel */
	adjusted_mode->hdisplay = 0x280;
	adjusted_mode->htotal = 0x320;
	adjusted_mode->hsync_start = 0x290;
	adjusted_mode->hsync_end = 0x2f0;
	adjusted_mode->vdisplay = 0x1e0;
	adjusted_mode->vtotal = 0x20d;
	adjusted_mode->vsync_start = 0x1ea;
	adjusted_mode->vsync_end = 0x1ec;
#endif
#if 0 /* 480p - icdk */
	adjusted_mode->hdisplay = 0x280;
	adjusted_mode->htotal = 0x35a;
	adjusted_mode->hsync_start = 0x2e0;
	adjusted_mode->hsync_end = 0x31e;
	adjusted_mode->vdisplay = 0x1e0;
	adjusted_mode->vtotal = 0x20e;
	adjusted_mode->vsync_start = 0x1ea;
	adjusted_mode->vsync_end = 0x1ec;
#endif
#endif
	return true;
}


static void mdfld_hdmi_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	u32 hdmip_enabled = 0;
	u32 hdmib, hdmi_phy_misc;

	PSB_DEBUG_ENTRY("%s\n", mode == DRM_MODE_DPMS_ON ?
		"on" : "off");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	hdmib = REG_READ(hdmi_priv->hdmib_reg) | HDMIB_PIPE_B_SELECT;

	if (dev_priv->bDVIport) {
		hdmib &= ~(HDMIB_NULL_PACKET | HDMI_AUDIO_ENABLE);
		REG_WRITE(VIDEO_DIP_CTL, 0x0);
		REG_WRITE(AUDIO_DIP_CTL, 0x0);
	} else
		hdmib |= (HDMIB_NULL_PACKET | HDMI_AUDIO_ENABLE);

	hdmi_phy_misc = REG_READ(HDMIPHYMISCCTL);
	hdmip_enabled = REG_READ(hdmi_priv->hdmib_reg) & HDMIB_PORT_EN;
	PSB_DEBUG_ENTRY("hdmip_enabled is %x\n", hdmip_enabled);

	if (mode != DRM_MODE_DPMS_ON) {
		if (dev_priv->mdfld_had_event_callbacks
			&& !dev_priv->bDVIport
			&& (hdmip_enabled != 0))
			(*dev_priv->mdfld_had_event_callbacks)
				(HAD_EVENT_HOT_UNPLUG, dev_priv->had_pvt_data);

		REG_WRITE(hdmi_priv->hdmib_reg,
				hdmib & ~HDMIB_PORT_EN & ~HDMI_AUDIO_ENABLE);
		psb_disable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc | HDMI_PHY_POWER_DOWN);
		REG_WRITE(VIDEO_DIP_CTL, 0x0);
		REG_WRITE(AUDIO_DIP_CTL, 0x0);
	} else {
		REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc & ~HDMI_PHY_POWER_DOWN);
	        psb_enable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		REG_WRITE(hdmi_priv->hdmib_reg,
				hdmib | HDMIB_PORT_EN);

		if (!dev_priv->bDVIport)
			mdfld_hdmi_set_avi_infoframe(dev, &output->base,
					hdmi_priv->current_mode);

		if (dev_priv->mdfld_had_event_callbacks
			&& !dev_priv->bDVIport
			&& (hdmip_enabled == 0))
			(*dev_priv->mdfld_had_event_callbacks)
				(HAD_EVENT_HOT_PLUG, dev_priv->had_pvt_data);
	}
	/* flush hdmi port register */
	REG_WRITE(hdmi_priv->hdmib_reg, REG_READ(hdmi_priv->hdmib_reg));

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mdfld_hdmi_encoder_save(struct drm_encoder *encoder)
{
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = MRST_DSPBBASE;
	struct drm_device *dev = encoder->dev;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	u32 temp;
	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	hdmi_priv->need_encoder_restore = true;

	/*Use Disable pipeB plane to turn off HDMI screen
	 in early_suspend  */
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
		REG_WRITE(dspcntr_reg,
				temp & ~DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mdfld_hdmi_encoder_restore(struct drm_encoder *encoder)
{
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = MRST_DSPBBASE;
	int dspbsurf_reg = DSPBSURF;
	int dspblinoff_reg = DSPBLINOFF;
	int dspbsize_reg = DSPBSIZE;
	int dspbstride_reg = DSPBSTRIDE;
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	u32 temp;
	PSB_DEBUG_ENTRY("\n");

	if(unlikely(!(hdmi_priv->need_encoder_restore)))
		return;

	hdmi_priv->need_encoder_restore = false;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	/*Set DSPBSURF to systemBuffer temporary to avoid hdmi display last picture*/
	REG_WRITE(dspbsurf_reg, dev_priv->init_screen_start);
	REG_WRITE(dspblinoff_reg, dev_priv->init_screen_offset);
	/* FIXME: this restore to init screen need to be replaced
	* by flushing from upper layer. */
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	REG_WRITE(dspbsize_reg, dev_priv->init_screen_size);
	REG_WRITE(dspbstride_reg, dev_priv->init_screen_stride);
#endif

	/*Restore pipe B plane to turn on HDMI screen
	in late_resume*/
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
		REG_WRITE(dspcntr_reg,
				temp | DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
	}
	/*restore avi infomation*/
	if (!dev_priv->bDVIport)
		mdfld_hdmi_set_avi_infoframe(dev,
			 &output->base, hdmi_priv->current_mode);
	else {
		REG_WRITE(VIDEO_DIP_CTL, 0x0);
		REG_WRITE(AUDIO_DIP_CTL, 0x0);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}


static void mdfld_hdmi_connector_save(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;

	PSB_DEBUG_ENTRY("\n");
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	hdmi_priv->save_HDMIB = REG_READ(hdmi_priv->hdmib_reg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static void mdfld_hdmi_connector_restore(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	REG_WRITE(hdmi_priv->hdmib_reg, hdmi_priv->save_HDMIB);
	REG_READ(hdmi_priv->hdmib_reg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/* HDMI DIP related stuff */
static int mdfld_hdmi_get_cached_edid_block(struct drm_connector *connector, uint32_t num_block, uint8_t *edid_block, uint32_t size)
{
    struct drm_display_info *displayinfo = &(connector->display_info);
    if (num_block >= MAX_EDID_BLOCKS)
    {
	DRM_ERROR("mdfld_hdmi_get_cached_edid_block() - Invalid EDID block\n");
        return 0;
    }

    edid_block = &displayinfo->raw_edid[EDID_BLOCK_SIZE*num_block];
    return 1;
}

/////////////////////////////////////////////////////////////////////////
//    INTHDMIENCODER_CreateEELDPacket():
//    This function parses v1.3 base EDID and CEA-861b EDID Timing Extension
//    Version3 and creates EELD (Enhanced EDID Like Data) packet. This EELD data contains
//    audio configuration information and other details read EDID.This can also contain Vendor specific Data
//
/////////////////////////////////////////////////////////////////////////
static int mdfld_hdmi_create_eeld_packet(struct drm_connector *connector)
{
    struct psb_intel_output *output = to_psb_intel_output(connector);
    struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
    uint8_t ucEdidBlock[128];
    hdmi_eeld_t *pEEld                = NULL;
    baseedid_1_x_t *pEdid        = NULL;
    ce_edid_t *pCeEdid        = NULL;
    int dwNumOfBytes        = 0;
    int sizeOfCEADataBlock    = 0;
    uint8_t * pDataBlock        = NULL;
    edid_dtd_timing_t *pDTD    = NULL;
    uint8_t *pData            = NULL;
    uint8_t ucDataBlockTag    = 0;
    cea_861b_adb_t *pADB        = NULL;
    uint8_t i                    = 0;
    uint8_t j                    = 0;
    uint8_t * pSADBlocks = NULL;
    uint8_t * pCurrentSADBlocks = NULL;
    uint32_t ulNumSADBytes = 0;
    //vsdb_byte6_to_byte8_t *pVSDB = NULL;
    uint32_t ulIndex = 0;
    //uint8_t b48kHzCADPresent = false;

    pEEld = (hdmi_eeld_t *) &hdmi_priv->eeld;

    // Fill Version info
    pEEld->cea_edid_rev_id = HDMI_EELD_CEA_EDID_VERSION;
    pEEld->eld_ver = HDMI_EELD_VERSION;

    // Fill BaseLine ELD length
    // This is 80 bytes as per EELD proposal
    pEEld->baseline_eld_length = HDMI_EELD_BASELINE_DATA_LENGTH;

    //Zero out EDID block buffer
    memset(ucEdidBlock, 0, sizeof(ucEdidBlock));

    // Get Extn EDID
    if(!mdfld_hdmi_get_cached_edid_block(connector, 1, ucEdidBlock, EDID_BLOCK_SIZE))
    {
        return 0;
    }

    pCeEdid = (ce_edid_t *) ucEdidBlock;

    //allocate memory (48 bytes) for SAD Blocks buffer
    pSADBlocks = kcalloc(1, 48, GFP_KERNEL);

    if(pSADBlocks == NULL)
    {
	DRM_ERROR("mdfld_hdmi_create_eld_packaet() - Failed to allocate mem for pSADBlocks\n");
        return 0;
    }

    pCurrentSADBlocks = pSADBlocks;

    // Now pull out data from CEA Extension EDID
    // If Offset <= 4, we will not have CEA DataBlocks
    if(pCeEdid->ucDTDOffset > CEA_EDID_HEADER_SZIE)
    {
        sizeOfCEADataBlock = pCeEdid->ucDTDOffset - CEA_EDID_HEADER_SZIE;

        pDataBlock = (uint8_t *)pCeEdid;

        // skip header (first 4 bytes) in CEA EDID Timing Extension
        // and set pointer to start of DataBlocks collection
        pDataBlock += CEA_EDID_HEADER_SZIE;

        // General Format of CEA Data Block Collection
        // -----------+--------------------+-----------------------------------------+
        //            |Byte#   |bits5-7    |       bits 0-4                          |
        // -----------|--------------------+-----------------------------------------+
        //            |  1     | Video Tag |Length = total #of video bytes following |
        //            |        |    Code   |this byte (L1)                           |
        //            |--------------------+-----------------------------------------+
        //  Video     |  2     | CEA Short Video Descriptor 1                        |
        //  Data      |--------+-----------------------------------------------------|
        //  Block     |  3     | CEA Short Video Descriptor 2                        |
        //            |--------+-----------------------------------------------------|
        //            | ...    | ...                                                 |
        //            |--------------------------------------------------------------+
        //            | 1+L1   | CEA Short Video Descriptor L1                       |
        // -----------+--------------------+-----------------------------------------+
        //            | 2+L1   | Audio Tag |Length = total #of audio bytes following |
        //            |        |    Code   |this byte (L2)                           |
        //            |--------------------+-----------------------------------------+
        //  Audio     | 3+L1   |                                                     |
        //  Data      |--------+                                                     |
        //  Block     | 4+L1   | CEA Short Audio Descriptor 1                        |
        //            |--------+                                                     |
        //            | 5+L1   |                                                     |
        //            |--------------------------------------------------------------+
        //            | ...    |                                                     |
        //            |        |                                                     |
        //            |        |                                                     |
        //            | ...    |                                                     |
        //            |---------------------------------------------------------------
        //            |L1+L2   |                                                     |
        //            |--------|                                                     |
        //            |1+L1+L2 | CEA Short Audio Descriptor L2/3                     |
        //            |--------|                                                     |
        //            |2+L1+L2 |                                                     |
        // -----------+--------------------------------------------------------------+
        //            |3+L1+L2 |  Speaker  |Length = total #of SA bytes following    |
        //            |        | Tag Code  |this byte (L1)                           |
        //  Speaker   |--------------------------------------------------------------+
        //  Allocation|4+L1+L2 |                                                     |
        //  Data      |--------|                                                     |
        //  Block     |5+L1+L2 | Speaker Allocation Data Block Payload(3 bytes)      |
        //            |--------|                                                     |
        //            |6+L1+L2 |                                                     |
        // -----------+--------------------------------------------------------------+
        //            |7+L1+L2 | VSDB  Tag |Length = total #of VSDB bytes following  |
        //            |        |    Code   |this byte (L1)                           |
        //  Vendor    |--------------------------------------------------------------+
        //  Specific  |8+L1+L2 |                                                     |
        //  Data      |--------|                                                     |
        //  Block     |9+L1+L2 | 24-bit IEEE Registration Identifier (LSB first)     |
        //            |--------|                                                     |
        //            |10+L1+L2|                                                     |
        //            |--------------------------------------------------------------+
        //            | ...    | Vendor Specific Data block Payload                  |
        // -----------+--------------------------------------------------------------+

        while(sizeOfCEADataBlock > 0)
        {
            // Get the Size of CEA DataBlock in bytes and TAG
            dwNumOfBytes = *pDataBlock & CEA_DATABLOCK_LENGTH_MASK;
            ucDataBlockTag = (*pDataBlock & CEA_DATABLOCK_TAG_MASK) >> 5;

            switch(ucDataBlockTag)
            {
                case CEA_AUDIO_DATABLOCK:
                    // move beyond tag/length byte
                    ++pDataBlock;
                    for (i = 0; i < (dwNumOfBytes / 3); ++i, pDataBlock += 3)
                    {
                        pADB = (cea_861b_adb_t*)pDataBlock;
                        switch(pADB->audio_format_code)
                        {
                            // uncompressed audio (Linear PCM)
                            case AUDIO_LPCM:
                                memcpy(&(hdmi_priv->lpcm_sad),pDataBlock,3);
                                //save these blocks
                                memcpy(pCurrentSADBlocks, pDataBlock, 3);
                                // move pointer in SAD blocks buffer
                                pCurrentSADBlocks += 3;
                                // update SADC field
                                pEEld->sadc += 1;
                                break;
                            // compressed audio
                            case AUDIO_AC3:
                            case AUDIO_MPEG1:
                            case AUDIO_MP3:
                            case AUDIO_MPEG2:
                            case AUDIO_AAC:
                            case AUDIO_DTS:
                            case AUDIO_ATRAC:
                            case AUDIO_OBA:
                            case AUDIO_DOLBY_DIGITAL:
                            case AUDIO_DTS_HD:
                            case AUDIO_MAT:
                            case AUDIO_DST:
                            case AUDIO_WMA_PRO:
                                //save these blocks
                                memcpy(pCurrentSADBlocks, pDataBlock, 3);
                                // move pointer in SAD blocks buffer
                                pCurrentSADBlocks += 3;
                                // update SADC field
                                pEEld->sadc += 1;
                                break;
                        }
                    }
                    break;

                case CEA_VENDOR_DATABLOCK:
                    // audio wants data from 6th byte of VSDB onwards
                    //Sighting 94842:

                    // | Byte # |    bits[7-0]                                              |
                    // |--------------------------------------------------------------------|
                    // | 1-3    |24-bit IEEE Registration Identifier (0x000C03)             |
                    // |--------------------------------------------------------------------|
                    // | 4-5    |       Source Physical Address                             |
                    // |--------------------------------------------------------------------|
                    // | 6      |SupportsAI|DC48bit|DC36bit|Dc30bit|DCY444|Rsvd|Rsvd|DVIDual|
                    // |--------------------------------------------------------------------|
                    // | 7      |   Max TMDS clock                                          |
                    // |--------------------------------------------------------------------|
                    // | 8      |Latency_Field |I_Latency_Field| Reserved bits 5-0          |
                    // |        |   _Present   |  _Present     |                            |
                    // |--------------------------------------------------------------------|
                    // | 9      |               Video Latency                               |
                    // |--------------------------------------------------------------------|
                    // | 10     |               Audio Latency                               |
                    // |--------------------------------------------------------------------|
                    // | 11     |            Interlaced Video Latency                       |
                    // |--------------------------------------------------------------------|
                    // | 12     |            Interlaced Audio Latency                       |
                    // |--------------------------------------------------------------------|

                    ++pDataBlock;
                    // move pointer to next CEA Datablock
                    pDataBlock += dwNumOfBytes;
                    break;

                case CEA_SPEAKER_DATABLOCK:
                    pEEld->speaker_allocation_block = *(++pDataBlock);
                    // move pointer to next CEA Datablock
                    pDataBlock += dwNumOfBytes;
                    break;

                default:
                    // Move pointer to next CEA DataBlock
                    pDataBlock += (dwNumOfBytes + 1);
            }
            // Decrement size of CEA DataBlock
            sizeOfCEADataBlock -= (dwNumOfBytes + 1);
        }
    }

    //Copy all the saved SAD blocks at the end of ELD
    //SAD blocks should be written after the Monitor name and VSDB.
    //See ELD definition in iHDMI.h
    ulNumSADBytes = (pEEld->sadc) * 3; //Size of each SAD block is 3 bytes

    //DCN 460119: Audio does not play on displays which do not provide SAB in EDID.
    //Solution: Graphics driver should create a default SAB in ELD with front left and front right
    //speakers enabled if the display supports basic audio.
    pDataBlock = (uint8_t *)pCeEdid;
    if((*(pDataBlock + HDMI_CEA_EXTENSION_BLOCK_BYTE_3) & HDMI_BASIC_AUDIO_SUPPORTED) && (pEEld->speaker_allocation_block == 0))
    {
        pEEld->flr = 1;
    }
    //End of DCN 460119

    // zero out local buffers
    memset(ucEdidBlock, 0, sizeof(ucEdidBlock));

    // Get base EDID
    if(!mdfld_hdmi_get_cached_edid_block(connector, 0, ucEdidBlock, EDID_BLOCK_SIZE))
    {
        return 0;
    }

    pEdid = (baseedid_1_x_t*) ucEdidBlock;
    pDTD = &pEdid->DTD[1];

    //Update the Manufacturer ID and Product Code here
    memcpy(pEEld->manufacturer_id,pEdid->ManufacturerID,2);
    memcpy(pEEld->product_id,pEdid->ProductID,2);

    // Now Fill the monitor string name
    // Search through DTD blocks, looking for monitor name
    for (i = 0; i < MAX_BASEEDID_DTD_BLOCKS - 1; ++i, ++pDTD)
    {
        // Set a uint8_t pointer to DTD data
        pData = (uint8_t *)pDTD;

        // Check the Flag (the first two bytes) to determine
        // if this block is used as descriptor
        if (pData[0] == 0x00 && pData[1] == 0x00)
        {
            // And now check Data Type Tag within this descriptor
            // Tag = 0xFC, then monitor name stored as ASCII
            if (pData[3] == 0xFC)
            {
                ulIndex = 0;
                // Copy monitor name
                for (j = 0; (j < 13) && (pData[j+5] != 0x0A); ++j)
                {
                    pEEld->mn_sand_sads[ulIndex] = pData[j+5];
                    ulIndex++;
                }
                pEEld->mnl = j;
                break;
            }
        }
    }

    //Check if number of SAD Bytes > 0 and for size within limits of allowed Base line Data size as per EELD spec
    if((ulNumSADBytes > 0) && (ulNumSADBytes <= 64))
    {
        //Copy the SADs immediately after the Monitor Name String
        memcpy(&pEEld->mn_sand_sads[j], pSADBlocks, ulNumSADBytes);
    }


    // Header = 4, Baseline Data = 60 and Vendor (INTEL) specific = 2
    // 4 + 60 + 2 = 66
    hdmi_priv->hdmi_eeld_size = HDMI_EELD_SIZE;

    //free the buffer allocated for SAD blocks
    kfree(pSADBlocks);
    pSADBlocks = NULL;
    pCurrentSADBlocks = NULL;
    return 1;
}
bool isHDMI(struct drm_connector *connector)
{
	int i = 0, n = 0;
	bool ret = false;
	baseedid_1_4_t *base_edid_block = NULL;
	char *edid_block = NULL;

	if (!connector->edid_blob_ptr)
		return ret;

	if (!connector->edid_blob_ptr->data)
		return ret;

	base_edid_block = (baseedid_1_4_t *)connector->edid_blob_ptr->data;
	edid_block = (char *)connector->edid_blob_ptr->data;
	if (!base_edid_block) {
		PSB_DEBUG_ENTRY("base_edid_block = NULL\n");
		return false;
	}

	if (base_edid_block->ucNumExtBlocks == 0) {
		PSB_DEBUG_ENTRY("ucNumExtBlocks = 0\n");
		/*print_hex_dump_bytes(KERN_ERR, DUMP_PREFIX_NONE,
				 edid_block, EDID_LENGTH);*/
		return false;
	}

	/*search VSDB in extend  edid */
	for (i = 0; i <= EDID_BLOCK_SIZE - 3; i++) {
		n = EDID_BLOCK_SIZE + i;
		if (edid_block[n] == 0x03 &&
		   edid_block[n+1] == 0x0c &&
		   edid_block[n+2] == 0x00)
			ret = true;
	}
	/*print_hex_dump_bytes(KERN_ERR, DUMP_PREFIX_NONE,
		  edid_block, 2*EDID_LENGTH);*/

	return ret;
}
static enum drm_connector_status
mdfld_hdmi_edid_detect(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	struct edid *edid = NULL;
	enum drm_connector_status status = connector_status_disconnected;
	int i = 0;
	int ret = 0;
	int monitor_number = sizeof(mdfld_hdmi_edid) / sizeof(struct hdmi_edid_info);
	static u8 last_mfg_id[2] = {0,0};
	static u8 last_prod_code[2] = {0,0};
	static u32 last_serial = 0; /* FIXME: byte order */
	int hdmi_change = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, i2c_adapter is NULL.\n");

		/* hard-coded the HDMI_I2C_ADAPTER_ID to be 8, Should get from GCT*/
		output->hdmi_i2c_adapter = i2c_get_adapter(8);
	}

	if (!output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, no valid i2c_adapter.\n");
		return ret;
	}

	/* MSIC HW issue would be fixed after C0. */
	if (!((IS_MDFLD_OLD(dev)) &&
		(dev_priv->platform_rev_id < MDFLD_PNW_C0)))
		edid = drm_get_edid(&output->base, output->hdmi_i2c_adapter);

	hdmi_priv->has_hdmi_sink = false;
	if (edid) {
		PSB_DEBUG_ENTRY("edid read successful .\n");
		if (edid->input & DRM_EDID_INPUT_DIGITAL) {
			status = connector_status_connected;
			hdmi_priv->has_hdmi_sink = drm_detect_hdmi_monitor(edid);
			mdfld_hdmi_create_eeld_packet(connector);
		}

		if(hdmi_change == 0)
		{
			if (!(((edid->mfg_id[0] == last_mfg_id[0])&&(edid->mfg_id[1] == last_mfg_id[1]))
				&&((edid->prod_code[0] == last_prod_code[0])&&(edid->prod_code[1] == last_prod_code[1]))
				&&(edid->serial == last_serial))) {
				hdmi_change = 1;
			}
				last_mfg_id[0] = edid->mfg_id[0];
				last_mfg_id[1] = edid->mfg_id[1];
				last_prod_code[0] = edid->prod_code[0];
				last_prod_code[1] = edid->prod_code[1];
				last_serial = edid->serial;
		}
		if (hdmi_change ==1) {
			drm_connector_property_set_value(connector,dev->mode_config.scaling_mode_property,DRM_MODE_SCALE_ASPECT);
		}

		DRM_INFO("hdmi_change =%d", hdmi_change);
		DRM_INFO("mfg_id: 0x%x,0x%x", edid->mfg_id[0],edid->mfg_id[1]);
		DRM_INFO("prod_code: 0x%x,0x%x", edid->prod_code[0],edid->prod_code[1]);
		DRM_INFO("serial: 0x%x", edid->serial);

		drm_mode_connector_update_edid_property(connector, edid);
		kfree(edid);
		dev_priv->hdmi_done_reading_edid = true;
		hdmi_priv->is_hardcode_edid = false;
	} else {
		/* Failed to read a valid EDID, so we're using a hardcoded one */
		if ((HDMI_EDID == NULL) || (strlen(HDMI_EDID) > HDMI_MONITOR_NAME_LENGTH))
			return status;

		DRM_DEBUG("Detected HDMI monitor %s. Using hardcoded EDID.\n",
				HDMI_EDID);

		for (i = 0; i < monitor_number; i++) {
			if (!strcasecmp(HDMI_EDID, mdfld_hdmi_edid[i].monitor_name)) {
				edid = (struct edid *)mdfld_hdmi_edid[i].edid_info;
				break;
			}
		}

		if (i == monitor_number) {
#if HDMI_COMPLIANCE_TEST
			if (!((IS_MDFLD_OLD(dev)) &&
				(dev_priv->platform_rev_id < MDFLD_PNW_C0))) {
				PSB_DEBUG_ENTRY(
					"hard code fix to DVI!\n");
				/*EDID_Samsung_2493HM*/
				edid =
				 (struct edid *)mdfld_hdmi_edid[1].edid_info;
			} else
#endif
			{
				PSB_DEBUG_ENTRY(
					"hard code fix to HDMI!\n");
				/*Use Toshiba Regza HDMI EDID as default data.*/
				edid =
				(struct edid *)mdfld_hdmi_edid[i - 1].edid_info;
			}
		}

		hdmi_priv->has_hdmi_sink = false;
		if (edid) {
			drm_mode_connector_update_edid_property(connector, edid);
			status = connector_status_connected;
			hdmi_priv->has_hdmi_sink = drm_detect_hdmi_monitor(edid);
			mdfld_hdmi_create_eeld_packet(connector);
		}

		dev_priv->hdmi_done_reading_edid = true;
		hdmi_priv->is_hardcode_edid = true;
	}
       /* Check whether it's DVI port or HDMI. */
	if (isHDMI(connector))
		dev_priv->bDVIport = false;
	else
		dev_priv->bDVIport = true;

	PSB_DEBUG_ENTRY("is DVI port ? %s", dev_priv->bDVIport ? "yes" : "no");
	return status;
}
void hdmi_unplug_prepare(struct drm_psb_private *dev_priv)
{
	int wait_count = 0;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, false)) {
		/*wait for overlay switch back to pipeA*/
		while ((PSB_RVDC32(OV_OVADD) & 0xc0) && wait_count < 5) {
			PSB_DEBUG_ENTRY(
			"wait for overlay switch to mipi 0x%x.\n",
					PSB_RVDC32(OV_OVADD));
			msleep(20);
			wait_count++;
		};
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else
		PSB_DEBUG_ENTRY("display controller is power off.\n");
}

static enum drm_connector_status mdfld_hdmi_detect(struct drm_connector
						*connector, bool force)
{
	(void) force;           /*  unused parameter */

#ifdef CONFIG_X86_MDFLD
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_output *psb_intel_output =
		to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = psb_intel_output->dev_priv;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	u8 data = 0;
	enum drm_connector_status connect_status =
			 connector_status_disconnected;
	static bool first_time_boot_detect = true;
	bool hdmi_hpd_connected = false;

	/* Check if monitor is attached to HDMI connector. */
	if (IS_MDFLD_OLD(dev)) {
		intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &data);

		if (data & HPD_SIGNAL_STATUS)
			hdmi_hpd_connected = true;
		else
			hdmi_hpd_connected = false;
	} else if (IS_CTP(dev)) {
		if (gpio_get_value(CLV_TI_HPD_GPIO_PIN) == 0)
			hdmi_hpd_connected = false;
		else
			hdmi_hpd_connected = true;
	}

	if (hdmi_hpd_connected) {
		DRM_DEBUG("%s: HPD connected data = 0x%x.\n", __func__, data);

		if (connector->status == connector_status_connected) {
			/*
			 * Don't un-gate Display B if HDMI is connected and in
			 * D0i3 state.
			 */
			connect_status =  connector_status_connected;
			goto fun_exit;
		}
		dev_priv->panel_desc |= DISPLAY_B;
		/*
		Handle Hot-plug of HDMI. Display B would be power-gated
		by ospm_post_init if HDMI is not detected during driver load.
		This will power-up Display B if HDMI is
		connected post driver load.
		*/
		/*
		If pmu_nc_set_power_state fails then accessing HW
		reg would result in a crash - IERR/Fabric error.
		*/
		if (pmu_nc_set_power_state(OSPM_DISPLAY_B_ISLAND,
				OSPM_ISLAND_UP, OSPM_REG_TYPE))
			BUG();

		if (dev_priv->hdmi_done_reading_edid ||
				(mdfld_hdmi_edid_detect(connector) ==
						connector_status_connected)) {
			connect_status = connector_status_connected;
		}
		else {
			connect_status = connector_status_disconnected;
		}

	} else {
		PSB_DEBUG_ENTRY("%s: HPD disconnected data = 0x%x.\n", __func__,
				data);
		connect_status = connector_status_disconnected;

		if (dev_priv->panel_desc & DISPLAY_B) {
			hdmi_unplug_prepare(dev_priv);
			/*
			 * Clean up the HDMI connector attached encoder, to make
			 * drm_crtc_helper_set_config() do mode setting
			 *each time, especially when plug out HDMI from
			 *a sink device and plug it
			 * in to another one with different EDID.
			 */
			drm_helper_disable_unused_functions(dev);
		}

	}
#else
	connect_status =  mdfld_hdmi_edid_detect(connector);
#endif

fun_exit:
	if (first_time_boot_detect) {
		if (connect_status == connector_status_connected) {
			hdmi_state = 1;
			dev_priv->bhdmiconnected = true;
		}
		first_time_boot_detect = false;
		PSB_DEBUG_ENTRY("first time boot detect state 0x%d.\n",
				connect_status);
	} else {
		/* bhdmiconnected indicates that the HDMI power island is up */
		if (connect_status == connector_status_connected)
			dev_priv->bhdmiconnected = true;
	}

	return connect_status;
}

static int mdfld_hdmi_set_property(struct drm_connector *connector,
				       struct drm_property *property,
				       uint64_t value)
{
	struct drm_encoder *pEncoder = connector->encoder;

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		PSB_DEBUG_ENTRY("scaling mode \n");
	} else if (!strcmp(property->name, "backlight") && pEncoder) {
		PSB_DEBUG_ENTRY("backlight \n");
	} else if (!strcmp(property->name, "DPMS") && pEncoder) {
		PSB_DEBUG_ENTRY("DPMS \n");
	}

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		struct psb_intel_crtc *pPsbCrtc = to_psb_intel_crtc(pEncoder->crtc);
		bool bTransitionFromToCentered = false;
		bool bTransitionFromToAspect = false;
		uint64_t curValue;

		if (!pPsbCrtc)
			goto set_prop_error;

		if (drm_connector_property_get_value(connector, property, &curValue))
			goto set_prop_error;

		if (drm_connector_property_set_value(connector, property, value))
			goto set_prop_error;

		bTransitionFromToCentered = (curValue == DRM_MODE_SCALE_NO_SCALE) ||
			(value == DRM_MODE_SCALE_NO_SCALE) || (curValue == DRM_MODE_SCALE_CENTER) || (value == DRM_MODE_SCALE_CENTER);
		bTransitionFromToAspect = (curValue == DRM_MODE_SCALE_ASPECT) ||
			(value == DRM_MODE_SCALE_ASPECT);

		if (pPsbCrtc->saved_mode.hdisplay != 0 &&
		    pPsbCrtc->saved_mode.vdisplay != 0) {
			if (bTransitionFromToCentered || bTransitionFromToAspect
				||(value > DRM_MODE_SCALE_NO_SCALE) || (value == DRM_MODE_SCALE_FULLSCREEN)){
				if (!drm_crtc_helper_set_mode(pEncoder->crtc, &pPsbCrtc->saved_mode,
					    pEncoder->crtc->x, pEncoder->crtc->y, pEncoder->crtc->fb))
					goto set_prop_error;
			} else {
				struct drm_encoder_helper_funcs *pEncHFuncs  = pEncoder->helper_private;
				pEncHFuncs->mode_set(pEncoder, &pPsbCrtc->saved_mode,
						     &pPsbCrtc->saved_adjusted_mode);
			}
		}
	}
set_prop_done:
    return 0;
set_prop_error:
    return -1;
}

/**
 * Return the list of HDMI DDC modes if available.
 */
static int mdfld_hdmi_get_hardcoded_edid_modes(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output = to_psb_intel_output(connector);
	struct edid *edid = NULL;
	int ret = 0;
	int i = 0;
	int monitor_number = sizeof(mdfld_hdmi_edid) / sizeof(struct hdmi_edid_info);

	PSB_DEBUG_ENTRY("\n");

	DRM_DEBUG("%s: fake edid info.\n", __func__);

	for (i = 0; i < monitor_number; i++) {
		if (!strcasecmp(HDMI_EDID, mdfld_hdmi_edid[i].monitor_name)) {
			edid = (struct edid *)mdfld_hdmi_edid[i].edid_info;
			break;
		}
	}

	if (i == monitor_number)
		/* Use Toshiba Regza HDMI EDID as default data. */
		edid = (struct edid *)mdfld_hdmi_edid[i - 1].edid_info;

	connector->display_info.raw_edid = (char *)edid;
	drm_mode_connector_update_edid_property(&psb_intel_output->
			base, edid);
	ret = drm_add_edid_modes(&psb_intel_output->base, edid);

	return ret;
}

int mdfld_add_eedid_video_block_modes(struct drm_connector *connector, struct edid *edid)
{
    struct drm_device *dev = connector->dev;
    int i, j, modes = 0;
    char *edid_ext = NULL;
    u32 quirks = 0;
    struct detailed_timing *timing;
    extention_block_cea_t eb;
    unsigned char *c = eb.data;
    int vic;
    mdfld_hdmi_timing_t *p_video_mode;
    struct detailed_timing * vblock_timings;
    struct drm_display_mode *mode;
    int block_type, payload_size;
    int start_offset, end_offset;

    if (edid->version == 1 && edid->revision < 3)
        return 0;
    if (!edid->extensions)
        return 0;

    /* Find CEA extension */
    for (i = 0; i < edid->extensions; i++) {
        edid_ext = (char *)edid + EDID_LENGTH * (i + 1);

        /* Extended EDID BLOCK */
        if (edid_ext[0] == 0x02) {
            memset(&eb, 0, sizeof(extention_block_cea_t));

            if (edid_ext != NULL)
                memcpy(&eb, edid_ext, sizeof(extention_block_cea_t));

            printk("CEA tag = %X, revision = %X, content_offset = %X, flags = %X\n", eb.tag, eb.revision, eb.content_offset, eb.flags);

            /*
             * Short descriptors section exists when:
             * - offset is not 4
             * - CEA extension version is 3
             */
            if ((eb.content_offset != 4) && (eb.revision >= 3)) {

                c = eb.data;
                /* All area before detailed descriptors should be filled
                 * TODO: Shall we change this to safer check?
                 */
                while (c < ((unsigned char *)&eb + eb.content_offset)) {
                    block_type = (*c & 0xE0) >> 5;
                    payload_size = *c & 0x1F;

                    printk("block_type: %d, payload_size: %d\n", block_type, payload_size);
                    /* Simple block types */
                    switch (block_type) {
                        case 0:
                            break;
                        case 1:
                            break;
                        case 2:
                            for (j = 1; j < payload_size+1; j++) {
                                vic = *(c + j) & 0x7F;

                                printk("vic: %d\n", vic);
                                if (vic == 34 || vic == 32 || vic == 33 ||
				    vic == 1 || vic == 2 || vic == 3 ||
				    vic == 4 || vic == 19) {
                                    p_video_mode = &mdfld_hdmi_video_mode_table[vic - 1];

                                    mode = drm_mode_create(dev);
                                    if (!mode)
					return 0;

                                    mode->type = DRM_MODE_TYPE_DRIVER;

                                    mode->vrefresh = p_video_mode->refresh;
                                    mode->clock = p_video_mode->dclk;

                                    mode->hdisplay = p_video_mode->width;
                                    mode->hsync_start = p_video_mode->hsync_start;
                                    mode->hsync_end = p_video_mode->hsync_end;
                                    mode->htotal = p_video_mode->htotal;

                                    mode->vdisplay = p_video_mode->height;
                                    mode->vsync_start = p_video_mode->vsync_start;
                                    mode->vsync_end = p_video_mode->vsync_end;
                                    mode->vtotal = p_video_mode->vtotal;

                                    mode->flags = 0;

                                    drm_mode_set_name(mode);

                                    mode->width_mm = 1600;
                                    mode->height_mm = 900;

                                    printk("VIDEO TIMING. hdisplay = %d, vdisplay = %d.\n",
                                            mode->hdisplay, mode->vdisplay);

                                    drm_mode_probed_add(connector, mode);
                                    modes++;
                                }
                            }
                            break;
                        default:
                            break;
                    }
                    c += (*c & 0x1F) + 1;
                } /* end of while */
            }
        }
    } /* end of for loop */

    return modes;
}

static int mdfld_hdmi_get_modes(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = psb_intel_output->dev_priv;
	struct edid *edid = NULL;
	int ret = 0;
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct drm_display_mode *mode, *t, *user_mode, *dup_mode;
	int found_it = 0;

	PSB_DEBUG_ENTRY("\n");

	/*
	 * Probe more modes from EDID data.
	 */
	if (!psb_intel_output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, i2c_adapter is NULL.\n");

		/* hard-coded the HDMI_I2C_ADAPTER_ID to be 8, Should get from GCT*/
		psb_intel_output->hdmi_i2c_adapter = i2c_get_adapter(8);
	}

	if (!psb_intel_output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, no valid i2c_adapter . \n");
		return ret;
	}

	/* MSIC HW issue would be fixed after C0. */
	if (!((IS_MDFLD_OLD(dev)) &&
		(dev_priv->platform_rev_id < MDFLD_PNW_C0))) {
		if (connector->edid_blob_ptr)
			edid = connector->edid_blob_ptr->data;
	}

	if (edid) {
		ret = drm_add_edid_modes(&psb_intel_output->base, edid);
		ret += mdfld_add_eedid_video_block_modes(&psb_intel_output->base, edid);
	} else if (IS_CTP(dev)) {
		/* try one more time to get edid here */
		edid = drm_get_edid(&psb_intel_output->base,
			psb_intel_output->hdmi_i2c_adapter);
		hdmi_priv->has_hdmi_sink = false;
		if (edid) {
			PSB_DEBUG_ENTRY("edid read successful .\n");
			if (edid->input & DRM_EDID_INPUT_DIGITAL) {
				hdmi_priv->has_hdmi_sink =
					drm_detect_hdmi_monitor(edid);
				mdfld_hdmi_create_eeld_packet(connector);
			}
			drm_mode_connector_update_edid_property(connector,
								edid);
			dev_priv->hdmi_done_reading_edid = true;
			ret = drm_add_edid_modes(&psb_intel_output->base, edid);
			kfree(edid);
		} else{
			ret = mdfld_hdmi_get_hardcoded_edid_modes(connector);
		}
	} else {
		ret = mdfld_hdmi_get_hardcoded_edid_modes(connector);
	}

	/* add user mode to connector->mode list to support DRM IOCTL attachmode */
	list_for_each_entry_safe(user_mode, t, &connector->user_modes, head) {
		found_it = 0;
		/* check for whether user_mode is already in the mode_list */
		list_for_each_entry(mode, &connector->modes, head) {
			if (drm_mode_equal(mode, user_mode)) {
				found_it = 1;
				mode->status = MODE_OK;
			}
		}

		PSB_DEBUG_ENTRY("user_mode ret: 0x%x, found_it: 0x%x\n", ret, found_it);
		if (!found_it) {
			dup_mode = drm_mode_duplicate(dev, user_mode);
			dup_mode->status = MODE_OK;
			list_add_tail(&dup_mode->head, &connector->modes);
			ret += 1;
		}
	}
	if (ret <= 0) {
		/*
		 * Didn't get an EDID, so set wide sync ranges so we get all
		 * modes handed to valid_mode for checking.
		 */
		connector->display_info.min_vfreq = 0;
		connector->display_info.max_vfreq = 200;
		connector->display_info.min_hfreq = 0;
		connector->display_info.max_hfreq = 200;

		return ret;
	}

	/* Reset preferred mode if NULL EDID detected on PNW C1 or latter. */
	if (hdmi_priv->is_hardcode_edid) {
		if ((IS_MDFLD_OLD(dev)) &&
			(dev_priv->platform_rev_id > MDFLD_PNW_C0)) {
			/*from C1, driver can get edid right
			  if not right,set prefer mode to 640*480p
			  it is required by HDMI compliance test.*/
			PSB_DEBUG_ENTRY(
					" set hardcode edid prefer to 640*480p\n");
			list_for_each_entry_safe(mode, t,
					&connector->probed_modes, head) {
#if HDMI_COMPLIANCE_TEST
				if (mode->hdisplay == 640
						&& mode->vdisplay == 480)
#else
				if (mode->hdisplay == 1280
						&& mode->vdisplay == 720)
#endif
					mode->type
						|= DRM_MODE_TYPE_PREFERRED;
				else if (mode->type & DRM_MODE_TYPE_PREFERRED)
					mode->type
						&= ~DRM_MODE_TYPE_PREFERRED;
			}
		}
	}
	/*record the preferred mode*/
	list_for_each_entry_safe(mode, t,
			&connector->probed_modes, head) {
		if ((mode->type & DRM_MODE_TYPE_PREFERRED)) {
			drm_mode_debug_printmodeline(mode);
			if (hdmi_priv->edid_preferred_mode)
				drm_mode_destroy(dev,
						hdmi_priv->edid_preferred_mode);

			hdmi_priv->edid_preferred_mode =
				drm_mode_duplicate(dev, mode);
		}
	}
	/*clear the mode ,which the same as preferred mode
	  otherwise the preferred mode will be updated by
	  the same mode in DRM */
    if (hdmi_priv->edid_preferred_mode != NULL) {
        list_for_each_entry_safe(mode, t, &connector->probed_modes, head) {
            if ((!(mode->type & DRM_MODE_TYPE_PREFERRED)) &&
                    drm_mode_equal(mode,
                        hdmi_priv->edid_preferred_mode)) {
                PSB_DEBUG_ENTRY(
                        "clear the same mode as preferred.\n");
                list_del(&mode->head);
                drm_mode_destroy(dev, mode);
            }
        }
    }

    return ret;
}

static int mdfld_hdmi_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;

	PSB_DEBUG_ENTRY("display info. hdisplay = %d, vdisplay = %d, vrefresh = %d.\n",
			mode->hdisplay, mode->vdisplay, mode->vrefresh);

	/* change the maximum clock to be 74.25mhz per DC spec */
	if (IS_CTP(dev)) {
		if (mode->clock > 165000)
			return MODE_CLOCK_HIGH;
	} else {
		if (mode->clock > 74250)
			return MODE_CLOCK_HIGH;
	}

	if (mode->clock < 20000)
		return MODE_CLOCK_LOW;

	if (mode->type == DRM_MODE_TYPE_USERDEF)
		return MODE_OK;

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	/* just in case */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static void mdfld_hdmi_connector_dpms(struct drm_connector *connector, int mode)
{
	struct drm_device * dev = connector->dev;
	struct drm_psb_private * dev_priv = dev->dev_private;
	int hdmi_audio_busy = 0;
	pm_event_t hdmi_audio_event;
	u32 dspcntr_val;
#ifdef CONFIG_PM_RUNTIME
	bool panel_on, panel_on2;
#endif

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON))
		return ;

	/*first, execute dpms*/
	/* using suspend to judge whether hdmi audio is playing */
	hdmi_audio_event.event = 0;
	if (dev_priv->had_interface && dev_priv->had_pvt_data)
		hdmi_audio_busy =
			dev_priv->had_interface->suspend(dev_priv->had_pvt_data,
						hdmi_audio_event);

	PSB_DEBUG_ENTRY("[DPMS] audio busy: 0x%x\n", hdmi_audio_busy);

	/* if hdmi audio is busy, just turn off hdmi display plane */
	if (hdmi_audio_busy) {
		dspcntr_val = PSB_RVDC32(DSPBCNTR);
		connector->dpms = mode;

		if (mode != DRM_MODE_DPMS_ON) {
			PSB_WVDC32(dspcntr_val & ~DISPLAY_PLANE_ENABLE, DSPBCNTR);
                        DISP_PLANEB_STATUS = DISPLAY_PLANE_DISABLE;
                }
		else {
			PSB_WVDC32(dspcntr_val | DISPLAY_PLANE_ENABLE, DSPBCNTR);
                        DISP_PLANEB_STATUS = DISPLAY_PLANE_ENABLE;
                }
	} else {
		/* Resume audio if audio is suspended */
		if (dev_priv->had_interface && dev_priv->had_pvt_data)
			dev_priv->had_interface->resume(dev_priv->had_pvt_data);

		drm_helper_connector_dpms(connector, mode);

                if (mode != DRM_MODE_DPMS_ON)
                        DISP_PLANEB_STATUS = DISPLAY_PLANE_DISABLE;
                else
                        DISP_PLANEB_STATUS = DISPLAY_PLANE_ENABLE;

	}


#ifdef CONFIG_PM_RUNTIME
	if(is_panel_vid_or_cmd(dev)) {
		/*DPI panel*/
		panel_on = dev_priv->dpi_panel_on;
		panel_on2 = dev_priv->dpi_panel_on2;
	} else {
		/*DBI panel*/
		panel_on = dev_priv->dbi_panel_on;
		panel_on2 = dev_priv->dbi_panel_on2;
	}

	/*then check all display panels + monitors status*/
	if(!panel_on && !panel_on2 && !(REG_READ(HDMIB_CONTROL) & HDMIB_PORT_EN)) {
		/*request rpm idle*/
		if(dev_priv->rpm_enabled) {
			pm_request_idle(&dev->pdev->dev);
		}
	}

	/**
	 * if rpm wasn't enabled yet, try to allow it
	 * FIXME: won't enable rpm for DPI since DPI
	 * CRTC setting is a little messy now.
	 * Enable it later!
	 */
#if 0 /* FIXME: won't enable rpm for DPI.  */
	if(!dev_priv->rpm_enabled && !is_panel_vid_or_cmd(dev))
		ospm_runtime_pm_allow(dev);
#endif
#endif
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static const struct drm_encoder_helper_funcs mdfld_hdmi_helper_funcs = {
	.dpms = mdfld_hdmi_dpms,
	.save = mdfld_hdmi_encoder_save,
	.restore = mdfld_hdmi_encoder_restore,
	.mode_fixup = mdfld_hdmi_mode_fixup,
	.prepare = psb_intel_encoder_prepare,
	.mode_set = mdfld_hdmi_mode_set,
	.commit = psb_intel_encoder_commit,
};

static const struct drm_connector_helper_funcs
    mdfld_hdmi_connector_helper_funcs = {
	.get_modes = mdfld_hdmi_get_modes,
	.mode_valid = mdfld_hdmi_mode_valid,
	.best_encoder = psb_intel_best_encoder,
};

static const struct drm_connector_funcs mdfld_hdmi_connector_funcs = {
	.dpms = mdfld_hdmi_connector_dpms,
	.save = mdfld_hdmi_connector_save,
	.restore = mdfld_hdmi_connector_restore,
	.detect = mdfld_hdmi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = mdfld_hdmi_set_property,
	.destroy = psb_intel_lvds_destroy,
};

void mdfld_hdmi_init(struct drm_device *dev,
		    struct psb_intel_mode_device *mode_dev)
{
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct mid_intel_hdmi_priv *hdmi_priv;

	PSB_DEBUG_ENTRY("\n");

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output) +
			       sizeof(struct mid_intel_hdmi_priv), GFP_KERNEL);
	if (!psb_intel_output)
		return;

	hdmi_priv = (struct mid_intel_hdmi_priv *)(psb_intel_output + 1);
	psb_intel_output->mode_dev = mode_dev;
	connector = &psb_intel_output->base;
	encoder = &psb_intel_output->enc;
	drm_connector_init(dev, &psb_intel_output->base,
			   &mdfld_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_DVID);

	drm_encoder_init(dev, &psb_intel_output->enc, &psb_intel_lvds_enc_funcs,
			 DRM_MODE_ENCODER_TMDS);

	drm_mode_connector_attach_encoder(&psb_intel_output->base,
					  &psb_intel_output->enc);
	psb_intel_output->type = INTEL_OUTPUT_HDMI;
	/*FIXME: May need to get this somewhere, but CG code seems hard coded it*/
	hdmi_priv->hdmib_reg = HDMIB_CONTROL;
	hdmi_priv->has_hdmi_sink = false;
	hdmi_priv->need_encoder_restore = false;
	psb_intel_output->dev_priv = hdmi_priv;

	drm_encoder_helper_add(encoder, &mdfld_hdmi_helper_funcs);
	drm_connector_helper_add(connector,
				 &mdfld_hdmi_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_connector_attach_property(connector, dev->mode_config.scaling_mode_property, DRM_MODE_SCALE_CENTER);

	/* hard-coded the HDMI_I2C_ADAPTER_ID to be 8, Should get from GCT*/
	psb_intel_output->hdmi_i2c_adapter = i2c_get_adapter(8);

	if (psb_intel_output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_init, i2c_adapter is availabe.\n");

	} else {
		printk(KERN_ALERT "No ddc adapter available!\n");
	}

	hdmi_priv->is_hdcp_supported = true;
	hdmi_priv->hdmi_i2c_adapter = psb_intel_output->hdmi_i2c_adapter;
	hdmi_priv->dev = dev;
	hdmi_priv->edid_preferred_mode = NULL;
	mdfld_hdcp_init(hdmi_priv);
#if (defined CONFIG_SND_INTELMID_HDMI_AUDIO)
	mdfld_hdmi_audio_init(hdmi_priv);
#endif /* if (defined CONFIG_SND_INTELMID_HDMI_AUDIO) */

	/* CTP has separate companion chip for HDMI. This is not required.
	   To be fixed : enable HPD for CTP.
	 */
	if (IS_MDFLD_OLD(dev)) {
		mdfld_msic_init(hdmi_priv);

		/* turn on HDMI power rails. These will be on in all non-S0iX
		states so that HPD and connection status will work. VCC330 will
		have ~1.7mW usage during idle states when the display is
		active.*/
		intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_ON);

		/* MSIC documentation requires that there be a 500us delay
		after enabling VCC330 before you can enable VHDMI */
		usleep_range(500, 1000);

		/* Extend VHDMI switch de-bounce time, to avoid redundant MSIC
		 * VREG/HDMI interrupt during HDMI cable plugged in/out. */
		intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_ON | VHDMI_DB_30MS);
	} else if (IS_CTP(dev))
		mdfld_ti_tpd_init(hdmi_priv);

	drm_sysfs_connector_add(connector);
	return;
}
