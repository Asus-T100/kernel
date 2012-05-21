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
#include "psb_intel_drv.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_intel_hdmi_reg.h"
#include "psb_intel_hdmi_edid.h"
#include "psb_intel_hdmi.h"
#include "mdfld_dsi_output.h"
#include "mdfld_hdmi_audio_if.h"
#include <linux/pm_runtime.h>
#include <linux/intel_mid_pm.h>

#include <asm/intel_scu_ipc.h>

/* FIXME_MDFLD HDMI EDID supports */
char EDID_Samsung[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0x2D, 0x10, 0x03,
	0x00, 0x00, 0x00, 0x00,
	0x14, 0x11, 0x01, 0x03, 0x80, 0x10, 0x09, 0x8C, 0x0A, 0xE2, 0xBD, 0xA1,
	0x5B, 0x4A, 0x98, 0x24,
	0x15, 0x47, 0x4A, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x6E, 0x28,
	0x55, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0xBC,
	0x52, 0xD0, 0x1E, 0x20,
	0xB8, 0x28, 0x55, 0x40, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00,
	0x00, 0xFD, 0x00, 0x31,
	0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x41, 0x4D, 0x53, 0x55, 0x4E, 0x47, 0x0A, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x01, 0xB3,
	0x02, 0x03, 0x1A, 0x71, 0x46, 0x84, 0x13, 0x05, 0x14, 0x03, 0x12, 0x23,
	0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0x66, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x80, 0x01, 0x1D,
	0x80, 0x18, 0x71, 0x1C,
	0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x9E,
	0x01, 0x1D, 0x80, 0xD0,
	0x72, 0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0xA0, 0x5A, 0x00, 0x00,
	0x00, 0x9E, 0x8C, 0x0A,
	0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xA0, 0x5A,
	0x00, 0x00, 0x00, 0x18,
	0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
	0xA0, 0x5A, 0x00, 0x00,
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xC0
};

char EDID_Samsung_2493HM[EDID_LENGTH] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x4c, 0x2d, 0x68, 0x03,
	0x34, 0x32, 0x49, 0x4b,
	0x0c, 0x12, 0x01, 0x03, 0x0e, 0x34, 0x20, 0xa0, 0x2a, 0xef, 0x91, 0xa3,
	0x54, 0x4c, 0x9b, 0x26,
	0x0f, 0x50, 0x54, 0xbf, 0xef, 0x80, 0xa9, 0x40, 0x81, 0x80, 0x81, 0x40,
	0x71, 0x4f, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x28, 0x3c, 0x80, 0xa0, 0x70, 0xb0,
	0x23, 0x40, 0x30, 0x20,
	0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0xfd,
	0x00, 0x38, 0x4b, 0x1e,
	0x51, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00,
	0x00, 0xfc, 0x00, 0x53,
	0x79, 0x6e, 0x63, 0x4d, 0x61, 0x73, 0x74, 0x65, 0x72, 0x0a, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xff,
	0x00, 0x48, 0x56, 0x52, 0x51, 0x33, 0x30, 0x30, 0x35, 0x35, 0x30, 0x0a,
	0x20, 0x20, 0x00, 0x20
};

char EDID_Dell[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0xac, 0x5d, 0xa0,
	0x55, 0x36, 0x4e, 0x32,
	0x23, 0x14, 0x01, 0x03, 0x80, 0x35, 0x1e, 0x78, 0xee, 0xee, 0x91, 0xa3,
	0x54, 0x4c, 0x99, 0x26,
	0x0f, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f, 0x81, 0x80, 0xd1, 0xc0,
	0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
	0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xff,
	0x00, 0x58, 0x31, 0x37,
	0x35, 0x52, 0x30, 0x38, 0x51, 0x32, 0x4e, 0x36, 0x55, 0x0a, 0x00, 0x00,
	0x00, 0xfc, 0x00, 0x44,
	0x45, 0x4c, 0x4c, 0x20, 0x53, 0x54, 0x32, 0x34, 0x31, 0x30, 0x0a, 0x20,
	0x00, 0x00, 0x00, 0xfd,
	0x00, 0x32, 0x4c, 0x1e, 0x53, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x01, 0x6d,
	0x02, 0x03, 0x1f, 0xf1, 0x4c, 0x90, 0x05, 0x04, 0x03, 0x02, 0x07, 0x16,
	0x01, 0x14, 0x1f, 0x12,
	0x13, 0x23, 0x09, 0x07, 0x07, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x83,
	0x01, 0x00, 0x00, 0x02,
	0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00,
	0x1e, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25,
	0x00, 0x13, 0x2b, 0x21,
	0x00, 0x00, 0x9e, 0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e,
	0x28, 0x55, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d,
	0x10, 0x10, 0x3e, 0x96,
	0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x1b
};

char EDID_Benq[EDID_LENGTH] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x09, 0xD1, 0x22, 0x79,
	0x45, 0x54, 0x00, 0x00,
	0x2A, 0x14, 0x01, 0x03, 0x0E, 0x35, 0x1E, 0x78, 0x2E, 0x60, 0x85, 0xA6,
	0x56, 0x4A, 0x9C, 0x25,
	0x12, 0x50, 0x54, 0xA5, 0x6B, 0x80, 0x81, 0x80, 0x81, 0x00, 0x81, 0xC0,
	0xA9, 0xC0, 0x81, 0x40,
	0xD1, 0xC0, 0x61, 0xC0, 0xB3, 0x00, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38,
	0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0x13, 0x2B, 0x21, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF,
	0x00, 0x50, 0x41, 0x41,
	0x30, 0x31, 0x39, 0x33, 0x38, 0x30, 0x32, 0x36, 0x0A, 0x20, 0x00, 0x00,
	0x00, 0xFD, 0x00, 0x32,
	0x4C, 0x18, 0x53, 0x15, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xFC,
	0x00, 0x42, 0x65, 0x6E, 0x51, 0x20, 0x45, 0x57, 0x32, 0x34, 0x32, 0x30,
	0x0A, 0x20, 0x00, 0x8E
};

char EDID_Toshiba_32RV525RZ[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0xac, 0x5d, 0xa0,
	0x55, 0x36, 0x4e, 0x32,
	0x23, 0x14, 0x01, 0x03, 0x80, 0x35, 0x1e, 0x78, 0xee, 0xee, 0x91, 0xa3,
	0x54, 0x4c, 0x99, 0x26,
	0x0f, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f, 0x81, 0x80, 0xd1, 0xc0,
	0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
	0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xff,
	0x00, 0x58, 0x31, 0x37,
	0x35, 0x52, 0x30, 0x38, 0x51, 0x32, 0x4e, 0x36, 0x55, 0x0a, 0x00, 0x00,
	0x00, 0xfc, 0x00, 0x44,
	0x45, 0x4c, 0x4c, 0x20, 0x53, 0x54, 0x32, 0x34, 0x31, 0x30, 0x0a, 0x20,
	0x00, 0x00, 0x00, 0xfd,
	0x00, 0x32, 0x4c, 0x1e, 0x53, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x01, 0x6d,
	0x02, 0x03, 0x1f, 0xf1, 0x4c, 0x90, 0x05, 0x04, 0x03, 0x02, 0x07, 0x16,
	0x01, 0x14, 0x1f, 0x12,
	0x13, 0x23, 0x09, 0x07, 0x07, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x83,
	0x01, 0x00, 0x00, 0x02,
	0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00,
	0x1e, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25,
	0x00, 0x13, 0x2b, 0x21,
	0x00, 0x00, 0x9e, 0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e,
	0x28, 0x55, 0x00, 0x13,
	0x2b, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d,
	0x10, 0x10, 0x3e, 0x96,
	0x00, 0x13, 0x2b, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x1b
};

char EDID_Toshiba_Regza[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x52, 0x62, 0x05, 0x02,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x11, 0x01, 0x03, 0x80, 0x69, 0x3b, 0x78, 0x0a, 0x0d, 0xc9, 0xa0,
	0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4c, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
	0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a,
	0x20, 0xe0, 0x2d, 0x10,
	0x10, 0x3e, 0x96, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00,
	0x00, 0xfc, 0x00, 0x54,
	0x53, 0x42, 0x2d, 0x54, 0x56, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xfd,
	0x00, 0x17, 0x3d, 0x0f, 0x44, 0x0f, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x01, 0x03,
	0x02, 0x03, 0x20, 0x77, 0x4a, 0x90, 0x05, 0x04, 0x03, 0x07, 0x02, 0x06,
	0x01, 0x20, 0x22, 0x23,
	0x09, 0x07, 0x07, 0x6c, 0x03, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x1e, 0xc0,
	0x2b, 0x2b, 0x33, 0x33,
	0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00,
	0xc4, 0x8e, 0x21, 0x00,
	0x00, 0x1e, 0x8c, 0x0a, 0xa0, 0x14, 0x51, 0xf0, 0x16, 0x00, 0x26, 0x7c,
	0x43, 0x00, 0xc4, 0x8e,
	0x21, 0x00, 0x00, 0x98, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10,
	0x10, 0x3e, 0x96, 0x00,
	0x13, 0x8e, 0x21, 0x00, 0x00, 0x18, 0x8c, 0x0a, 0xa0, 0x14, 0x51, 0xf0,
	0x16, 0x00, 0x26, 0x7c,
	0x43, 0x00, 0x13, 0x8e, 0x21, 0x00, 0x00, 0x98, 0x01, 0x1d, 0x80, 0x18,
	0x71, 0x1c, 0x16, 0x20,
	0x58, 0x2c, 0x25, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x9e, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xb3
};

char EDID_Samsung_UN40C7000WFXZA[EDID_LENGTH + HDMI_CEA_EDID_BLOCK_SIZE] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0x2D, 0xAC, 0x06,
	0x01, 0x00, 0x00, 0x00,
	0x34, 0x13, 0x01, 0x03, 0x80, 0x59, 0x32, 0x78, 0x0A, 0xEE, 0x91, 0xA3,
	0x54, 0x4C, 0x99, 0x26,
	0x0F, 0x50, 0x54, 0xBD, 0xEF, 0x80, 0x71, 0x4F, 0x81, 0x00, 0x81, 0x40,
	0x81, 0x80, 0x95, 0x00,
	0x95, 0x0F, 0xB3, 0x00, 0xA9, 0x40, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38,
	0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x66, 0x21, 0x50, 0xB0,
	0x51, 0x00, 0x1B, 0x30,
	0x40, 0x70, 0x36, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00,
	0x00, 0xFD, 0x00, 0x18,
	0x4B, 0x1A, 0x51, 0x17, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x41, 0x4D, 0x53, 0x55, 0x4E, 0x47, 0x0A, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x01, 0x6D,
	0x02, 0x03, 0x29, 0xF1, 0x46, 0x90, 0x04, 0x05, 0x03, 0x20, 0x22, 0x23,
	0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0xE2, 0x00, 0x0F, 0xE3, 0x05, 0x03, 0x01, 0x6E, 0x03,
	0x0C, 0x00, 0x10, 0x00,
	0xB8, 0x2D, 0x20, 0xD0, 0x04, 0x01, 0x40, 0x00, 0x37, 0x01, 0x1D, 0x00,
	0x72, 0x51, 0xD0, 0x1E,
	0x20, 0x6E, 0x28, 0x55, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x01,
	0x1D, 0x80, 0x18, 0x71,
	0x1C, 0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00,
	0x9E, 0x8C, 0x0A, 0xD0,
	0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xA0, 0x5A, 0x00,
	0x00, 0x00, 0x18, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x75,
};

static struct hdmi_edid_info mdfld_hdmi_edid[] = {
	{HDMI_EDID_INFO("SAMSUNG", EDID_Samsung)},
	{HDMI_EDID_INFO("SAMSUNG_2493HM", EDID_Samsung_2493HM)},
	{HDMI_EDID_INFO("DELL", EDID_Dell)},
	{HDMI_EDID_INFO("BENQ", EDID_Benq)},
	{HDMI_EDID_INFO("TOSHIBA_32RV525RZ", EDID_Toshiba_32RV525RZ)},
	{HDMI_EDID_INFO("TOSHIBA_REGZA", EDID_Toshiba_Regza)},
	{HDMI_EDID_INFO("SAMSUNG_UN40C", EDID_Samsung_UN40C7000WFXZA)},
};

/* VIC for AVI InfoFrame Data Byte 4 and CEA Short Descriptors */
struct hdmi_video_format_timing mdfld_hdmi_video_format_timing[] = {
	 /*NULL*/ {0, 0, 0, 0, 60, 1, 1, 0},
	/*640*480p@60hz 4:3 */
	{1, 640, 480, 60, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*480p@60hz 4:3 */
	{2, 720, 480, 60, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*480p@60hz 16:9 */
	{3, 720, 480, 60, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1280*720p@60hz 16:9 */
	{4, 1280, 720, 60, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*1920*1080i@60hz 16:9 */
	{5, 1920, 1080, 60, 1, 1, 1, HDMI_AVI_AFAR_16_9},
	/*720*480i@60hz 4:3 */
	{6, 720, 480, 60, 1, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*480i@60hz 16:9 */
	{7, 720, 480, 60, 1, 0, 0, HDMI_AVI_AFAR_16_9},
	/*720*240p@60hz 4:3 */
	{8, 720, 240, 60, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*240p@60hz 16:9 */
	{9, 720, 240, 60, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*2880*480i@60hz 4:3 */
	{10, 2880, 480, 60, 1, 0, 0, HDMI_AVI_AFAR_4_3},
	/*2880*480i@60hz 16:9 */
	{11, 2880, 480, 60, 1, 0, 0, HDMI_AVI_AFAR_16_9},
	/*2880*240p@60hz 4:3 */
	{12, 2880, 240, 60, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*2880*240p@60hz 16:9 */
	{13, 2880, 240, 60, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1440*480p@60hz 4:3 */
	{14, 1440, 480, 60, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*1440*480p@60hz 16:9 */
	{15, 1440, 480, 60, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1920*1080p@60hz 16:9 */
	{16, 1920, 1080, 60, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*720*5760p@50hz 4:3 */
	{17, 720, 576, 50, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*5760p@50hz 16:9 */
	{18, 720, 576, 50, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1280*720p@50hz 16:9 */
	{19, 1280, 720, 50, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*1920*1080i@50hz 16:9 */
	{20, 1920, 1080, 50, 1, 1, 1, HDMI_AVI_AFAR_16_9},
	/*720*576i@50hz 4:3 */
	{21, 720, 576, 50, 1, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*576i@50hz 16:9 */
	{22, 720, 576, 50, 1, 0, 0, HDMI_AVI_AFAR_16_9},
	/*720*288p@50hz 4:3 */
	{23, 720, 288, 50, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*720*288p@50hz 16:9 */
	{24, 720, 288, 50, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*2880*576i@50hz 4:3 */
	{25, 2880, 576, 50, 1, 0, 0, HDMI_AVI_AFAR_4_3},
	/*2880*576i@50hz 16:9 */
	{26, 2880, 576, 50, 1, 0, 0, HDMI_AVI_AFAR_16_9},
	/*2880*288p@50hz 4:3 */
	{27, 2880, 288, 50, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*2880*288p@50hz 16:9 */
	{28, 2880, 288, 50, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1440*576p@50hz 4:3 */
	{29, 1440, 576, 50, 0, 0, 0, HDMI_AVI_AFAR_4_3},
	/*1440*576p@50hz 16:9 */
	{30, 1440, 576, 50, 0, 0, 0, HDMI_AVI_AFAR_16_9},
	/*1920*1080p@50hz 16:9 */
	{31, 1920, 1080, 50, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*1920*1080p@24hz 16:9 */
	{32, 1920, 1080, 24, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*1920*1080p@25hz 16:9 */
	{33, 1920, 1080, 25, 0, 1, 1, HDMI_AVI_AFAR_16_9},
	/*1920*1080p@30hz 16:9 */
	{34, 1920, 1080, 30, 0, 1, 1, HDMI_AVI_AFAR_16_9},
};

static int mdfld_hdmi_get_cached_edid_block(struct drm_connector *connector,
					    uint32_t num_block,
					    uint8_t *edid_block,
					    uint32_t size);

static int mdfld_hdmi_timing_get_vic(struct drm_display_mode *mode)
{
	int i = 0;
	int vic = 0;
	u32 video_code_count = sizeof(mdfld_hdmi_video_format_timing) /
	    sizeof(struct hdmi_video_format_timing);
	struct hdmi_video_format_timing vft_table_entry = { 0 };

	PSB_DEBUG_ENTRY("%s\n", __func__);

	/*find video code for this mode */
	for (i = 0; i < video_code_count; i++) {
		vft_table_entry = mdfld_hdmi_video_format_timing[i];

		if (mode->hdisplay == vft_table_entry.hdisplay &&
		    mode->vdisplay == vft_table_entry.vdisplay &&
		    mode->vrefresh == vft_table_entry.refresh &&
		    (mode->flags & DRM_MODE_FLAG_INTERLACE)
		    == vft_table_entry.bInterlace) {
			vic = vft_table_entry.video_code;
			PSB_DEBUG_ENTRY("find hdmi mode video code %d\n",
					vft_table_entry.video_code);
			break;
		}
	}

	if (i == video_code_count)
		DRM_ERROR("Not supported HDMI mode: %s\n.", mode->name);

	return vic;
}

static int mdfld_hdmi_get_aspect_ratio(int h_active, int v_active)
{
	unsigned char aspect_ratio = 0xFF;

	if (v_active == h_active * 10 / 16)
		/* 16:10 aspect ratio for EDID 1.3 */
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_10;
	else if (v_active == h_active * 3 / 4)
		/* 4:3 aspect ratio */
		aspect_ratio = EDID_STD_ASPECT_RATIO_4_3;
	else if (v_active == h_active * 4 / 5)
		/* 5:4 aspect ratio */
		aspect_ratio = EDID_STD_ASPECT_RATIO_5_4;
	else if (v_active == h_active * 9 / 16)
		/* 16:9 aspect ratio */
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_9;
	else if (v_active == h_active)
		/*1:1 aspect ratio for EDID prior to EDID 1.3 */
		aspect_ratio = EDID_STD_ASPECT_RATIO_16_10;

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
					struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	u32 buf_size = 0;
	u32 viddipctl_val = 0;
	int i = 0;
	int aspect_ratio = 0xFF;
	avi_if_t avi_if;
	u32 *p_vsif = (u32 *) &avi_if;
	uint8_t edid_block[128];
	ce_edid_t *cea_edid_block = NULL;

	PSB_DEBUG_ENTRY("%s\n", __func__);

	memset(&avi_if, 0, sizeof(avi_if));
	avi_if.avi_info.avi_if_header.type = HDMI_AVI_TYPE;
	avi_if.avi_info.avi_if_header.version = HDMI_AVI_VERSION2;
	avi_if.avi_info.avi_if_header.length = HDMI_AVI_LENGTH;
	avi_if.avi_info.avi_if_header.ecc = 0;

	/* Data Byte 1 */
	memset(edid_block, 0, sizeof(edid_block));
	if (!mdfld_hdmi_get_cached_edid_block(connector, 1, edid_block,
					      EDID_BLOCK_SIZE)) {
		DRM_ERROR("Can't get the HDMI CEA EDID data\n");
		return -1;
	}

	cea_edid_block = (ce_edid_t *) edid_block;

	avi_if.avi_info.byte1_bits.scan_info = HDMI_AVI_SCAN_NODATA;
	if (cea_edid_block->ucCapabilty & BIT7)
		avi_if.avi_info.byte1_bits.scan_info = HDMI_AVI_SCAN_UNDERSCAN;

	avi_if.avi_info.byte1_bits.bar_info = HDMI_AVI_BAR_INVALID;
	avi_if.avi_info.byte1_bits.format = HDMI_AVI_AFI_VALID;

	avi_if.avi_info.byte1_bits.enc_mode = HDMI_AVI_RGB_MODE;
	if (cea_edid_block->ucCapabilty & BIT5)
		avi_if.avi_info.byte1_bits.enc_mode = HDMI_AVI_YCRCB444_MODE;
	else if (cea_edid_block->ucCapabilty & BIT4)
		avi_if.avi_info.byte1_bits.enc_mode = HDMI_AVI_YCRCB422_MODE;

	/* Data Byte 2 */
	/* Set aspect ratio */
	avi_if.avi_info.byte2_bits.afar = HDMI_AVI_AFAR_SAME;
	avi_if.avi_info.byte2_bits.par = HDMI_AVI_PAR_NODATA;
	aspect_ratio = mdfld_hdmi_get_aspect_ratio(mode->hdisplay * 2,
						   mode->vdisplay * 2);
	if (aspect_ratio != 0xFF) {
		switch (aspect_ratio) {
		case EDID_STD_ASPECT_RATIO_4_3:
			avi_if.avi_info.byte2_bits.par = HDMI_AVI_PAR_4_3;
			break;

		case EDID_STD_ASPECT_RATIO_16_9:
			avi_if.avi_info.byte2_bits.par = HDMI_AVI_PAR_16_9;
			break;

		default:
			avi_if.avi_info.byte2_bits.par = HDMI_AVI_PAR_NODATA;
			break;
		}
	}

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
	/* get hdmi mode video code */
	avi_if.avi_info.byte4_bits.vic = mdfld_hdmi_timing_get_vic(mode);

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

	avi_if.avi_info.chksum =
	    mdfld_hdmi_calc_hbuf_csum((uint8_t *) &avi_if,
				      HDMI_AVI_TOTAL_LENGTH);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return -1;

	/* Wait for 2 VSyncs. */
	mdelay(20);		/* msleep(20); */
	mdelay(20);		/* msleep(20); */
	/* psb_intel_wait_for_vblank(dev); */
	/* Wait for 3 HSync. */
	/* Disable the DIP type . */
	viddipctl_val = REG_READ(VIDEO_DIP_CTL);
	viddipctl_val &= ~(DIP_TYPE_MASK | DIP_BUFF_INDX_MASK |
			   DIP_RAM_ADDR_MASK | DIP_TX_FREQ_MASK);
	viddipctl_val |= DIP_BUFF_INDX_AVI | PORT_B_SELECT;
	PSB_DEBUG_ENTRY("[AVI INFO] viddipctl: 0x%x\n", viddipctl_val);
	REG_WRITE(VIDEO_DIP_CTL, viddipctl_val);

	/* Get the buffer size in DWORD. */
	buf_size = HDMI_AVI_TOTAL_LENGTH / 4 + 1;

	/* Write HDMI  InfoFrame. */
	for (i = 0; i < buf_size; i++)
		REG_WRITE(VIDEO_DIP_DATA, *(p_vsif++));

	/* Enable the DIP type and transmission frequency. */
	viddipctl_val |= DIP_TYPE_AVI | DIP_TX_FREQ_1VSNC;
	REG_WRITE(VIDEO_DIP_CTL, viddipctl_val);

	viddipctl_val |= EN_DIP;
	PSB_DEBUG_ENTRY("[AVI INFO] viddipctl: 0x%x\n", viddipctl_val);
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
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	int vic = 0;

	PSB_DEBUG_ENTRY("\n");

	/* Set panel fitting. */
	if ((adjusted_mode->hdisplay != mode->hdisplay)
	    || (adjusted_mode->vdisplay != mode->vdisplay))
		REG_WRITE(PFIT_CONTROL, PFIT_ENABLE | PFIT_PIPE_SELECT_B);

	hdmi_phy_misc = REG_READ(HDMIPHYMISCCTL) & ~HDMI_PHY_POWER_DOWN;
	REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc);

	hdmib = REG_READ(hdmi_priv->hdmib_reg) | HDMIB_PORT_EN |
	    HDMIB_PIPE_B_SELECT;

	if (dev_priv->bDVIport)
		hdmib &= ~HDMIB_NULL_PACKET;
	else {
		hdmib |= HDMIB_NULL_PACKET;
		/* Set AVI InfoFrame */
		mdfld_hdmi_set_avi_infoframe(dev, &output->base, adjusted_mode);
	}

	vic = mdfld_hdmi_timing_get_vic(adjusted_mode);

	/* Update hpolarity */
	if (mdfld_hdmi_video_format_timing[vic].hpolarity == 1)
		hdmib |= BIT3;
	else
		hdmib &= ~BIT3;

	/* Update vpolarity */
	if (mdfld_hdmi_video_format_timing[vic].vpolarity == 1)
		hdmib |= BIT4;
	else
		hdmib &= ~BIT4;

	REG_WRITE(hdmi_priv->hdmib_reg, hdmib);
	REG_READ(hdmi_priv->hdmib_reg);
}

static bool mdfld_hdmi_mode_fixup(struct drm_encoder *encoder,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct psb_intel_crtc *psb_intel_crtc =
	    to_psb_intel_crtc(encoder->crtc);
	struct psb_intel_output *output = enc_to_psb_intel_output(encoder);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	PSB_DEBUG_ENTRY("hdisplay = %d, vdisplay = %d. a_hdisplay = %d,"
			"a_vdisplay = %d.\n", mode->hdisplay, mode->vdisplay,
			adjusted_mode->hdisplay, adjusted_mode->vdisplay);

	/* Should never happen!! */
	if (IS_MID(dev) && psb_intel_crtc->pipe != 1)
		printk(KERN_ERR "Only support HDMI on pipe B on MID\n");

	/* FIXME: To make HDMI display with 1920x1080,
	 * in 864x480 (TPO), 480x864 (PYR) or 480x854 (TMD) fixed mode. */
	if ((hdmi_priv->mimic_mode != NULL) &&
	    (hdmi_priv->edid_preferred_mode != NULL)) {
		if (drm_mode_equal(mode, hdmi_priv->mimic_mode)) {
			adjusted_mode->hdisplay =
			    hdmi_priv->edid_preferred_mode->hdisplay;
			adjusted_mode->htotal =
			    hdmi_priv->edid_preferred_mode->htotal;
			adjusted_mode->hsync_start =
			    hdmi_priv->edid_preferred_mode->hsync_start;
			adjusted_mode->hsync_end =
			    hdmi_priv->edid_preferred_mode->hsync_end;
			adjusted_mode->hskew =
			    hdmi_priv->edid_preferred_mode->hskew;
			adjusted_mode->vdisplay =
			    hdmi_priv->edid_preferred_mode->vdisplay;
			adjusted_mode->vtotal =
			    hdmi_priv->edid_preferred_mode->vtotal;
			adjusted_mode->vsync_start =
			    hdmi_priv->edid_preferred_mode->vsync_start;
			adjusted_mode->vsync_end =
			    hdmi_priv->edid_preferred_mode->vsync_end;
			adjusted_mode->vscan =
			    hdmi_priv->edid_preferred_mode->vscan;
			adjusted_mode->clock =
			    hdmi_priv->edid_preferred_mode->clock;

			/* Apply the adjusted mode to CRTC mode setting
			 * parameters.*/
			drm_mode_set_crtcinfo(adjusted_mode,
					      CRTC_INTERLACE_HALVE_V);
		}
	}
#if 0
/* FIXME hard coded different HDMI timeings,
* remove them after the MSIC HDMI HW issue is fixed. */
#if 0				/* 720p - Adeel */
	adjusted_mode->hdisplay = 0x500;
	adjusted_mode->htotal = 0x672;
	adjusted_mode->hsync_start = 0x56e;
	adjusted_mode->hsync_end = 0x596;
	adjusted_mode->vdisplay = 0x2d0;
	adjusted_mode->vtotal = 0x2ee;
	adjusted_mode->vsync_start = 0x2d5;
	adjusted_mode->vsync_end = 0x2da;
#endif

#if 0				/* 1080p - Brian */
	adjusted_mode->hdisplay = 0x780;
	adjusted_mode->htotal = 0x898;
	adjusted_mode->hsync_start = 0x7d8;
	adjusted_mode->hsync_end = 0x804;
	adjusted_mode->vdisplay = 0x438;
	adjusted_mode->vtotal = 0x464;
	adjusted_mode->vsync_start = 0x43c;
	adjusted_mode->vsync_end = 0x446;
#endif
#if 0				/* 1080p - Adeel */
	adjusted_mode->hdisplay = 0x780;
	adjusted_mode->htotal = 0xabe;
	adjusted_mode->hsync_start = 0x9fe;
	adjusted_mode->hsync_end = 0xa2a;
	adjusted_mode->vdisplay = 0x438;
	adjusted_mode->vtotal = 0x465;
	adjusted_mode->vsync_start = 0x43c;
	adjusted_mode->vsync_end = 0x441;
#endif
#if 0				/* 480p - Adeel */
	adjusted_mode->hdisplay = 0x280;
	adjusted_mode->htotal = 0x320;
	adjusted_mode->hsync_start = 0x290;
	adjusted_mode->hsync_end = 0x2f0;
	adjusted_mode->vdisplay = 0x1e0;
	adjusted_mode->vtotal = 0x20d;
	adjusted_mode->vsync_start = 0x1ea;
	adjusted_mode->vsync_end = 0x1ec;
#endif
#if 0				/* 480p - icdk */
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
	u32 hdmib, hdmi_phy_misc;

	PSB_DEBUG_ENTRY("DPMS %s\n", mode == DRM_MODE_DPMS_ON ? "on" : "off");

	hdmib = REG_READ(hdmi_priv->hdmib_reg) | HDMIB_PIPE_B_SELECT;

	if (dev_priv->bDVIport)
		hdmib &= ~HDMIB_NULL_PACKET;
	else
		hdmib |= HDMIB_NULL_PACKET;

	hdmi_phy_misc = REG_READ(HDMIPHYMISCCTL);

	if (mode != DRM_MODE_DPMS_ON) {
		/* Disable HDMI audio Buffer Done and Underrun interrupts,
		 * in case that the HDMI PCM audio stream is running during
		 * mode setting / DPMS off process. */
		mdfld_irq_disable_hdmi_audio(dev);

		REG_WRITE(hdmi_priv->hdmib_reg,
			  hdmib & ~HDMIB_PORT_EN & ~HDMI_AUDIO_ENABLE);
		REG_WRITE(AUD_CONFIG,
			  REG_READ(AUD_CONFIG) & ~AUD_CONF_AUDIO_ENABLE);
		REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc | HDMI_PHY_POWER_DOWN);
	} else {
		REG_WRITE(HDMIPHYMISCCTL, hdmi_phy_misc & ~HDMI_PHY_POWER_DOWN);
		REG_WRITE(AUD_CONFIG,
			  REG_READ(AUD_CONFIG) | AUD_CONF_AUDIO_ENABLE);
		REG_WRITE(hdmi_priv->hdmib_reg,
			  hdmib | HDMIB_PORT_EN | HDMI_AUDIO_ENABLE);

		/* Enable HDMI audio Buffer Done and Underrun interrupts. */
		mdfld_irq_enable_hdmi_audio(dev);
	}
	REG_READ(hdmi_priv->hdmib_reg);
}

int lastpipebbase;

static void mdfld_hdmi_encoder_save(struct drm_encoder *encoder)
{
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = MRST_DSPBBASE;
	struct drm_device *dev = encoder->dev;
	u32 temp;
	PSB_DEBUG_ENTRY("\n");

	/*Use Disable pipeB plane to turn off HDMI screen
	   in early_suspend  */
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
		REG_WRITE(dspcntr_reg, temp & ~DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		lastpipebbase = REG_READ(dspbase_reg);
		REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		REG_READ(dspbase_reg);
	}
}

static void mdfld_hdmi_encoder_restore(struct drm_encoder *encoder)
{
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = MRST_DSPBBASE;
	struct drm_device *dev = encoder->dev;
	u32 temp;
	PSB_DEBUG_ENTRY("\n");

	/*Restore pipe B plane to turn on HDMI screen
	   in late_resume */
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
		REG_WRITE(dspcntr_reg, temp | DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		REG_WRITE(dspbase_reg, lastpipebbase);
		REG_READ(dspbase_reg);
	}
}

static void mdfld_hdmi_connector_save(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;

	PSB_DEBUG_ENTRY("\n");
	hdmi_priv->save_HDMIB = REG_READ(hdmi_priv->hdmib_reg);
}

static void mdfld_hdmi_connector_restore(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;

	PSB_DEBUG_ENTRY("\n");
	REG_WRITE(hdmi_priv->hdmib_reg, hdmi_priv->save_HDMIB);
	REG_READ(hdmi_priv->hdmib_reg);
}

/* HDMI DIP related stuff */
static int mdfld_hdmi_get_cached_edid_block(struct drm_connector *connector,
					    uint32_t num_block,
					    uint8_t *edid_block, uint32_t size)
{
	struct drm_display_info *displayinfo = &(connector->display_info);
	if (num_block >= MAX_EDID_BLOCKS) {
		DRM_ERROR
		    ("mdfld_hdmi_get_cached_edid_block() -"
			"Invalid EDID block\n");
		return 0;
	}
	edid_block = &displayinfo->raw_edid[EDID_BLOCK_SIZE * num_block];
	return 1;
}

/**
 * This function parses v1.3 base EDID and
 * CEA-861b EDID Timing Extension
 * Version3 and creates EELD (Enhanced EDID Like Data) packet.
 * This EELD data contains audio configuration information and
 * other details read EDID.This can also contain
 * Vendor specific Data
 */
static int mdfld_hdmi_create_eeld_packet(struct drm_connector *connector)
{
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	uint8_t ucEdidBlock[128];
	hdmi_eeld_t *pEEld = NULL;
	baseedid_1_x_t *pEdid = NULL;
	ce_edid_t *pCeEdid = NULL;
	int dwNumOfBytes = 0;
	int sizeOfCEADataBlock = 0;
	uint8_t *pDataBlock = NULL;
	edid_dtd_timing_t *pDTD = NULL;
	uint8_t *pData = NULL;
	uint8_t ucDataBlockTag = 0;
	cea_861b_adb_t *pADB = NULL;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t *pSADBlocks = NULL;
	uint8_t *pCurrentSADBlocks = NULL;
	uint32_t ulNumSADBytes = 0;
	/* vsdb_byte6_to_byte8_t *pVSDB = NULL; */
	uint32_t ulIndex = 0;
	/* uint8_t b48kHzCADPresent = false; */

	pEEld = (hdmi_eeld_t *) &hdmi_priv->eeld;

	/* Fill Version info */
	pEEld->cea_edid_rev_id = HDMI_EELD_CEA_EDID_VERSION;
	pEEld->eld_ver = HDMI_EELD_VERSION;

	/* Fill BaseLine ELD length
	 * This is 80 bytes as per EELD proposal */
	pEEld->baseline_eld_length = HDMI_EELD_BASELINE_DATA_LENGTH;

	/* Zero out EDID block buffer */
	memset(ucEdidBlock, 0, sizeof(ucEdidBlock));

	/* Get Extn EDID */
	if (!mdfld_hdmi_get_cached_edid_block
	    (connector, 1, ucEdidBlock, EDID_BLOCK_SIZE)) {
		return 0;
	}

	pCeEdid = (ce_edid_t *) ucEdidBlock;

	/* check if it is a CEA EDID EXTENSIO. */
	if (pCeEdid->ucTag != CEA_EDID_EXTENSION_TAG)
		return 0;

	/* allocate memory (48 bytes) for SAD Blocks buffer */
	pSADBlocks = kcalloc(1, 48, GFP_KERNEL);

	if (pSADBlocks == NULL) {
		DRM_ERROR
		    ("mdfld_hdmi_create_eld_packaet() - "
			"Failed to allocate mem for pSADBlocks\n");
		return 0;
	}

	pCurrentSADBlocks = pSADBlocks;

	/* Now pull out data from CEA Extension EDID
	 * If Offset <= 4, we will not have CEA DataBlocks */
	if (pCeEdid->ucDTDOffset > CEA_EDID_HEADER_SIZE) {
		sizeOfCEADataBlock =
		    pCeEdid->ucDTDOffset - CEA_EDID_HEADER_SIZE;

		pDataBlock = (uint8_t *) pCeEdid;

		/* skip header (first 4 bytes) in CEA EDID Timing Extension
		 * and set pointer to start of DataBlocks collection */
		pDataBlock += CEA_EDID_HEADER_SIZE;

/* General Format of CEA Data Block Collection
* -----------+--------------------+-----------------------------------------+
*            |Byte#   |bits5-7    |       bits 0-4                          |
* -----------|--------------------+-----------------------------------------+
*            |  1     | Video Tag |Length = total #of video bytes following |
*            |        |    Code   |this byte (L1)                           |
*            |--------------------+-----------------------------------------+
*  Video     |  2     | CEA Short Video Descriptor 1                        |
*  Data      |--------+-----------------------------------------------------|
*  Block     |  3     | CEA Short Video Descriptor 2                        |
*            |--------+-----------------------------------------------------|
*            | ...    | ...                                                 |
*            |--------------------------------------------------------------+
*            | 1+L1   | CEA Short Video Descriptor L1                       |
* -----------+--------------------+-----------------------------------------+
*            | 2+L1   | Audio Tag |Length = total #of audio bytes following |
*            |        |    Code   |this byte (L2)                           |
*            |--------------------+-----------------------------------------+
*  Audio     | 3+L1   |                                                     |
*  Data      |--------+                                                     |
*  Block     | 4+L1   | CEA Short Audio Descriptor 1                        |
*            |--------+                                                     |
*            | 5+L1   |                                                     |
*            |--------------------------------------------------------------+
*            | ...    |                                                     |
*            |        |                                                     |
*            |        |                                                     |
*            | ...    |                                                     |
*            |---------------------------------------------------------------
*            |L1+L2   |                                                     |
*            |--------|                                                     |
*            |1+L1+L2 | CEA Short Audio Descriptor L2/3                     |
*            |--------|                                                     |
*            |2+L1+L2 |                                                     |
* -----------+--------------------------------------------------------------+
*            |3+L1+L2 |  Speaker  |Length = total #of SA bytes following    |
*            |        | Tag Code  |this byte (L1)                           |
*  Speaker   |--------------------------------------------------------------+
*  Allocation|4+L1+L2 |                                                     |
*  Data      |--------|                                                     |
*  Block     |5+L1+L2 | Speaker Allocation Data Block Payload(3 bytes)      |
*            |--------|                                                     |
*            |6+L1+L2 |                                                     |
* -----------+--------------------------------------------------------------+
*            |7+L1+L2 | VSDB  Tag |Length = total #of VSDB bytes following  |
*            |        |    Code   |this byte (L1)                           |
*  Vendor    |--------------------------------------------------------------+
*  Specific  |8+L1+L2 |                                                     |
*  Data      |--------|                                                     |
*  Block     |9+L1+L2 | 24-bit IEEE Registration Identifier (LSB first)     |
*            |--------|                                                     |
*            |10+L1+L2|                                                     |
*            |--------------------------------------------------------------+
*            | ...    | Vendor Specific Data block Payload                  |
* -----------+--------------------------------------------------------------+
*/
		while (sizeOfCEADataBlock > 0) {
			/* Get the Size of CEA DataBlock in bytes and TAG */
			dwNumOfBytes = *pDataBlock & CEA_DATABLOCK_LENGTH_MASK;
			ucDataBlockTag =
			    (*pDataBlock & CEA_DATABLOCK_TAG_MASK) >> 5;

			switch (ucDataBlockTag) {
			case CEA_AUDIO_DATABLOCK:
				/* move beyond tag/length byte */
				++pDataBlock;
				for (i = 0; i < (dwNumOfBytes / 3);
				     ++i, pDataBlock += 3) {
					pADB = (cea_861b_adb_t *) pDataBlock;
					switch (pADB->audio_format_code) {
					/* uncompressed audio (Linear PCM) */
					case AUDIO_LPCM:
						memcpy(&(hdmi_priv->lpcm_sad),
						       pDataBlock, 3);
						/* save these blocks */
						memcpy(pCurrentSADBlocks,
						       pDataBlock, 3);
				/* move pointer in SAD blocks buffer */
						pCurrentSADBlocks += 3;
						/* update SADC field */
						pEEld->sadc += 1;
						break;
						/* compressed audio */
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
						/* save these blocks */
						memcpy(pCurrentSADBlocks,
						       pDataBlock, 3);
				/* move pointer in SAD blocks buffer */
						pCurrentSADBlocks += 3;
						/* update SADC field */
						pEEld->sadc += 1;
						break;
					}
				}
				break;

			case CEA_VENDOR_DATABLOCK:
/* audio wants data from 6th byte of VSDB onwards
* Sighting 94842:

* | Byte # |    bits[7-0]                                              |
* |--------------------------------------------------------------------|
* | 1-3    |24-bit IEEE Registration Identifier (0x000C03)             |
* |--------------------------------------------------------------------|
* | 4-5    |       Source Physical Address                             |
* |--------------------------------------------------------------------|
* | 6      |SupportsAI|DC48bit|DC36bit|Dc30bit|DCY444|Rsvd|Rsvd|DVIDual|
* |--------------------------------------------------------------------|
* | 7      |   Max TMDS clock                                          |
* |--------------------------------------------------------------------|
* | 8      |Latency_Field |I_Latency_Field| Reserved bits 5-0          |
* |        |   _Present   |  _Present     |                            |
* |--------------------------------------------------------------------|
* | 9      |               Video Latency                               |
* |--------------------------------------------------------------------|
* | 10     |               Audio Latency                               |
* |--------------------------------------------------------------------|
* | 11     |            Interlaced Video Latency                       |
* |--------------------------------------------------------------------|
* | 12     |            Interlaced Audio Latency                       |
* |--------------------------------------------------------------------|
*/
				++pDataBlock;
				/* move pointer to next CEA Datablock */
				pDataBlock += dwNumOfBytes;
				break;

			case CEA_SPEAKER_DATABLOCK:
				pEEld->speaker_allocation_block =
				    *(++pDataBlock);
				/* move pointer to next CEA Datablock */
				pDataBlock += dwNumOfBytes;
				break;

			default:
				/* Move pointer to next CEA DataBlock */
				pDataBlock += (dwNumOfBytes + 1);
			}
			/* Decrement size of CEA DataBlock */
			sizeOfCEADataBlock -= (dwNumOfBytes + 1);
		}
	}
	/* Copy all the saved SAD blocks at the end of ELD
	 * SAD blocks should be written after the Monitor
	 * name and VSDB. See ELD definition in iHDMI.h
	 * Size of each SAD block is 3 bytes */
	ulNumSADBytes = (pEEld->sadc) * 3;

	/* DCN 460119: Audio does not play on displays
	 * which do not provide SAB in EDID.
	 * Solution: Graphics driver should create a default SAB
	 * in ELD with front left and front right speakers enabled
	 * if the display supports basic audio. */
	pDataBlock = (uint8_t *) pCeEdid;
	if ((*(pDataBlock + HDMI_CEA_EXTENSION_BLOCK_BYTE_3) &
	     HDMI_BASIC_AUDIO_SUPPORTED)
	    && (pEEld->speaker_allocation_block == 0)) {
		pEEld->flr = 1;
	}
	/* End of DCN 460119 */

	/* zero out local buffers */
	memset(ucEdidBlock, 0, sizeof(ucEdidBlock));

	/* Get base EDID */
	if (!mdfld_hdmi_get_cached_edid_block
	    (connector, 0, ucEdidBlock, EDID_BLOCK_SIZE)) {
		return 0;
	}

	pEdid = (baseedid_1_x_t *) ucEdidBlock;
	pDTD = &pEdid->DTD[1];

	/* Update the Manufacturer ID and Product Code here */
	memcpy(pEEld->manufacturer_id, pEdid->ManufacturerID, 2);
	memcpy(pEEld->product_id, pEdid->ProductID, 2);

	/* Now Fill the monitor string name
	 * Search through DTD blocks, looking for monitor name */
	for (i = 0; i < MAX_BASEEDID_DTD_BLOCKS - 1; ++i, ++pDTD) {
		/* Set a uint8_t pointer to DTD data */
		pData = (uint8_t *) pDTD;

		/* Check the Flag (the first two bytes) to determine
		 * if this block is used as descriptor */
		if (pData[0] == 0x00 && pData[1] == 0x00) {
			/* And now check Data Type Tag within this descriptor
			 * Tag = 0xFC, then monitor name stored as ASCII */
			if (pData[3] == 0xFC) {
				ulIndex = 0;
				/* Copy monitor name */
				for (j = 0; (j < 13) && (pData[j + 5] != 0x0A);
				     ++j) {
					pEEld->mn_sand_sads[ulIndex] =
					    pData[j + 5];
					ulIndex++;
				}
				pEEld->mnl = j;
				break;
			}
		}
	}

	/* Check if number of SAD Bytes > 0 and for size within
	 * limits of allowed Base line Data size as
	 * per EELD spec */
	if ((ulNumSADBytes > 0) && (ulNumSADBytes <= 64)) {
		/* Copy the SADs immediately after
		 * the Monitor Name String */
		memcpy(&pEEld->mn_sand_sads[j], pSADBlocks, ulNumSADBytes);
	}
	/* Header = 4, Baseline Data = 60 and
	 * Vendor (INTEL) specific = 2
	 * 4 + 60 + 2 = 66 */
	hdmi_priv->hdmi_eeld_size = HDMI_EELD_SIZE;

	/* free the buffer allocated for SAD blocks */
	kfree(pSADBlocks);
	pSADBlocks = NULL;
	pCurrentSADBlocks = NULL;
	return 1;
}

/* start of S3D functions */

/**
 * Check if the HDMI display supports S3D. If so, report supported S3D formats
 *
 */
int mrfld_hdmi_s3d_query(struct drm_device *dev, struct drm_psb_s3d_query
			 *s3d_query)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct mid_intel_hdmi_priv *hdmi_priv = dev_priv->hdmi_priv;

	if (!hdmi_priv) {
		DRM_ERROR("%s, HDMI is not initialized.\n", __func__);
		return -EINVAL;
	}

	if (hdmi_priv->s3d_info.s3d_mandatory) {
		if ((s3d_query->mode_resolution_x == 1920) &&
		    (s3d_query->mode_resolution_y == 1080) &&
		    (s3d_query->mode_refresh_rate == 24)) {
			s3d_query->is_s3d_supported = 1;
			s3d_query->s3d_format = HDMI_3D_MANDATORY_1080P24;
		} else if ((s3d_query->mode_resolution_x == 1280) &&
			   (s3d_query->mode_resolution_y == 720)) {
			if ((s3d_query->mode_refresh_rate == 50) ||
			    (s3d_query->mode_refresh_rate == 60)) {
				s3d_query->is_s3d_supported = 1;
				s3d_query->s3d_format = HDMI_3D_MANDATORY_720P;
			}
		} else if ((s3d_query->mode_resolution_x == 1920) &&
			   (s3d_query->mode_resolution_y == 1080) &&
			   s3d_query->is_interleaving) {
			if ((s3d_query->mode_refresh_rate == 50) ||
			    (s3d_query->mode_refresh_rate == 60)) {
				s3d_query->is_s3d_supported = 1;
				s3d_query->s3d_format = HDMI_3D_MANDATORY_1080I;
			}
		}
	}

	return 0;
}

/**
 * This function parses v1.3 base EDID and CEA-861b EDID Timing Extension
 * Version3. It gets the VIC info for the first 16 HDMI display resolutions
 * and creates display 3D format packet. This packet contains the 3d
 * formats that are supported by the HDMI display device.
 */
static int mrfld_hdmi_get_s3d_formats(struct drm_connector *connector)
{
	struct psb_intel_output *output = to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = output->dev_priv;
	uint8_t ucEdidBlock[128];
	uint8_t *pDataBlock = NULL;
	uint8_t *pVideoDataBlock = NULL;
	uint8_t *pVSDB = NULL;
	uint8_t DataBlockLenth = 0;
	uint8_t DataBlockTag = 0;
	uint8_t hdmi_vic_len = 0;
	uint8_t VDB_VIC_len = 0;
	uint8_t hdmi_3d_multi_present = 0;
	uint8_t s3d_structure_all_h = 0;
	uint8_t s3d_structure_all_l = 0;
	uint8_t s3d_mask_h = 0;
	uint8_t s3d_mask_l = 0;
	uint8_t hdmi_2d_vic_order = 0;
	uint8_t s3d_structure = 0;
	uint8_t s3d_detail = 0;
	uint8_t reserved = 0;
	uint16_t s3d_structure_all = 0;
	uint16_t s3d_mask = 0xffff;
	int sizeOfCEADataBlock = 0;
	int hdmi_3d_len = 0;
	int i = 0;
	int result = 0;

	hdmi_3d_info_t *pS3D_info = NULL;
	ce_edid_t *pCeEdid = NULL;

	pS3D_info = (hdmi_3d_info_t *) &hdmi_priv->s3d_info;

	/* Fill Version info */
	pS3D_info->cea_edid_rev_id = HDMI_S3D_CEA_EDID_VERSION;
	pS3D_info->s3d_info_ver = HDMI_S3D_INFO_VERSION;

	/* Zero out EDID block buffer */
	memset(ucEdidBlock, 0, sizeof(ucEdidBlock));

	/* Get Extn EDID */
	if (!mdfld_hdmi_get_cached_edid_block
	    (connector, 1, ucEdidBlock, EDID_BLOCK_SIZE))
		return result;

	pCeEdid = (ce_edid_t *) ucEdidBlock;

	/* check if it is a CEA EDID EXTENSIO. */
	if (pCeEdid->ucTag != CEA_EDID_EXTENSION_TAG)
		return result;

    /**
    * Now pull out data from CEA Extension EDID. If Offset <= 4, we will not
    * have CEA DataBlocks
    */
	if (pCeEdid->ucDTDOffset > CEA_EDID_HEADER_SIZE) {
		sizeOfCEADataBlock =
		    pCeEdid->ucDTDOffset - CEA_EDID_HEADER_SIZE;

		pDataBlock = (uint8_t *) pCeEdid;

	/**
	* skip header (first 4 bytes) in CEA EDID Timing Extension
	* and set pointer to start of DataBlocks collection
	*/
		pDataBlock += CEA_EDID_HEADER_SIZE;

		/* Please see the General Format of CEA Data Block
		 * Collection in the function of
		 * mdfld_hdmi_create_eeld_packet */
		while (sizeOfCEADataBlock > 0) {
			/* Get the Size of CEA DataBlock in bytes and TAG */
			DataBlockLenth =
			    *pDataBlock & CEA_DATABLOCK_LENGTH_MASK;
			DataBlockTag =
			    (*pDataBlock & CEA_DATABLOCK_TAG_MASK) >> 5;

			switch (DataBlockTag) {
			case CEA_VENDOR_DATABLOCK:
				pVSDB = pDataBlock;
				break;

			case CEA_VIDEO_DATABLOCK:
				pVideoDataBlock = pDataBlock;
				break;
			default:
				break;
			}

			/* Move pointer to next CEA DataBlock */
			pDataBlock += (DataBlockLenth + 1);

			/* Decrement size of CEA DataBlock */
			sizeOfCEADataBlock -= (DataBlockLenth + 1);
		}
	}

	/* Get the first 16 VICs from the video data block. */
	VDB_VIC_len = *pVideoDataBlock & CEA_DATABLOCK_LENGTH_MASK;

	if (VDB_VIC_len > 16)
		VDB_VIC_len = 16;

	pVideoDataBlock++;

	for (i = 0; i < VDB_VIC_len; i++)
		hdmi_priv->vdb_vic[i] = pVideoDataBlock[i] & CEA_VDB_VIC_MASK;

	/* HDMI display 3D formats are included in HDMI VSDB. */

	/* return no 3D formats if there is no VSDB. */
	if (!pVSDB)
		return result;

	/* point to the VSDB. */
	pDataBlock = pVSDB;
	pDataBlock++;

	/* return no 3D formats if there is no HDMI VSDB. */
	if (!(*((uint32_t *) pDataBlock) & HDMI_VSDB_MASK) == HDMI_VSDB)
		return result;

	pDataBlock += HDMI_VIDEO_PRESENT_OFFSET - 1;

	/* return no 3D formats if HDMI video present bit is 0. */
	if (!(*pDataBlock & HDMI_VIDEO_PRESENT_BIT))
		return result;

	pDataBlock++;

	if (*pDataBlock & I_LATENCY_FIELDS_PRESENT_BIT)
		pDataBlock += 2;

	if (*pDataBlock & LATENCY_FIELDS_PRESENT_BIT)
		pDataBlock += 2;

	if (*pDataBlock & HDMI_3D_PRESENT_BIT) {
		/* Mandatory 3D display is supported by the HDMI display. */
		pS3D_info->s3d_mandatory = true;
		result = 1;
	}

	hdmi_3d_multi_present = (*pDataBlock & HDMI_3D_MULTI_PRESENT_BITS)
	    >> HDMI_3D_MULTI_PRESENT_BITS_POS;

	pDataBlock++;

	hdmi_3d_len = (int)(*pDataBlock & HDMI_3D_LEN_BITS);
	hdmi_vic_len = (*pDataBlock & HDMI_VIC_LEN_BITS)
	    >> HDMI_VIC_LEN_BITS_POS;

	/* no more 3D data block. */
	if (hdmi_3d_len == 0)
		return result;

	/* point to 3D data block. */
	pDataBlock += hdmi_vic_len + 1;

	if (hdmi_3d_multi_present == 1 || hdmi_3d_multi_present == 2) {
		s3d_structure_all_h = *(pDataBlock++);
		s3d_structure_all_l = *(pDataBlock++);
		s3d_structure_all =
		    (s3d_structure_all_h << 8) | s3d_structure_all_l;
		hdmi_3d_len -= 2;
		result = 1;
		pS3D_info->s3d_vic = true;
	}

	if (hdmi_3d_multi_present == 2) {
		s3d_mask_h = *(pDataBlock++);
		s3d_mask_l = *(pDataBlock++);
		s3d_mask = (s3d_mask_h << 8) | s3d_mask_l;
		hdmi_3d_len -= 2;
	}

	if (hdmi_3d_multi_present == 1 || hdmi_3d_multi_present == 2) {
		for (i = 0; i < 16; i++) {
			if (s3d_mask & (1 << i))
				pS3D_info->s3d_structure_all[i] =
				    s3d_structure_all;
		}
	}

	while (hdmi_3d_len > 0) {
		s3d_structure = *pDataBlock & HDMI_3D_STRUCTURE_BITS;
		hdmi_2d_vic_order = (*pDataBlock & HDMI_2D_VIC_ORDER_BITS)
		    >> HDMI_2D_VIC_ORDER_BITS_POS;

		pS3D_info->s3d_structure_all[hdmi_2d_vic_order] |=
		    1 << s3d_structure;

		hdmi_3d_len -= 1;
		pDataBlock++;
		result = 1;
		pS3D_info->s3d_vic = true;

		/* The following info is not needed in current HW/SW.
		 * May need it later. */
		if (s3d_structure & HDMI_3D_STRUCTURE_EXTRA_BIT) {
			s3d_detail = (*pDataBlock & HDMI_3D_DETAILS_BITS)
			    >> HDMI_3D_DETAILS_BITS_POS;

			hdmi_3d_len -= 1;
			pDataBlock++;
		}
	}

	return result;
}

/* end of S3D functions */

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
	int monitor_number =
	    sizeof(mdfld_hdmi_edid) / sizeof(struct hdmi_edid_info);
	uint8_t edid_block[128];
	baseedid_1_4_t *edid_block_data = NULL;

	PSB_DEBUG_ENTRY("\n");

	if (!output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, i2c_adapter is NULL.\n");

		/* hard-coded the HDMI_I2C_ADAPTER_ID to be 3,
		 * Should get from GCT */
		output->hdmi_i2c_adapter = i2c_get_adapter(3);
	}

	if (!output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, no valid i2c_adapter.\n");
#if 0				/* FIXME MRFLD */
		return ret;
#endif				/* FIXME MRFLD */
	}
#if 0				/* FIXME MRFLD */
	/* MSIC HW issue would be fixed after C0. */
	if (dev_priv->platform_rev_id >= MDFLD_PNW_C0)
		edid = drm_get_edid(&output->base, output->hdmi_i2c_adapter);
#endif				/* FIXME MRFLD */

	hdmi_priv->has_hdmi_sink = false;
	if (edid) {
		if (edid->input & DRM_EDID_INPUT_DIGITAL) {
			status = connector_status_connected;
			hdmi_priv->has_hdmi_sink =
			    drm_detect_hdmi_monitor(edid);
			mdfld_hdmi_create_eeld_packet(connector);
		}

		drm_mode_connector_update_edid_property(connector, edid);
		kfree(edid);
		dev_priv->hdmi_done_reading_edid = true;
	} else {
		/* Failed to read a valid EDID,
		 * so we're using a hardcoded one */
		if ((HDMI_EDID == NULL)
		    || (strlen(HDMI_EDID) > HDMI_MONITOR_NAME_LENGTH))
			return status;

#if 1				/* FIXME MRFLD */
		DRM_INFO("Detected HDMI monitor %s. Using hardcoded EDID.\n",
			 HDMI_EDID);
#endif				/* FIXME MRFLD */
		DRM_DEBUG("Detected HDMI monitor %s. Using hardcoded EDID.\n",
			  HDMI_EDID);

		for (i = 0; i < monitor_number; i++) {
			if (!strcasecmp
			    (HDMI_EDID, mdfld_hdmi_edid[i].monitor_name)) {
				edid =
				    (struct edid *)mdfld_hdmi_edid[i].edid_info;
				break;
			}
		}

		if (i == monitor_number)
			/* Use Toshiba Regza HDMI EDID as default data. */
			edid = (struct edid *)mdfld_hdmi_edid[i - 1].edid_info;

		hdmi_priv->has_hdmi_sink = false;
		if (edid) {
			drm_mode_connector_update_edid_property(connector,
								edid);
			status = connector_status_connected;
			hdmi_priv->has_hdmi_sink =
			    drm_detect_hdmi_monitor(edid);
			mdfld_hdmi_create_eeld_packet(connector);
		}

		dev_priv->hdmi_done_reading_edid = true;
	}

	/* Check whether it's DVI port or HDMI. */
	memset(edid_block, 0, sizeof(edid_block));
	if (!mdfld_hdmi_get_cached_edid_block(connector, 0, edid_block,
					      EDID_BLOCK_SIZE)) {
		DRM_ERROR("Can't get the HDMI EDID data\n");
		return -1;
	}

	edid_block_data = (baseedid_1_4_t *) edid_block;

	if (edid_block_data->ucNumExtBlocks == 0)
		dev_priv->bDVIport = true;
	else
		dev_priv->bDVIport = false;

	PSB_DEBUG_ENTRY("is DVI port ? %s", dev_priv->bDVIport ? "yes" : "no");

	return status;
}

static enum drm_connector_status mdfld_hdmi_detect(struct drm_connector
						   *connector)
{
#if 0				/* FIXME MRFLD */
/* #ifdef CONFIG_X86_MRST */
#endif				/* FIXME MRFLD */
#if 1
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = psb_intel_output->dev_priv;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	u8 data = 0;

#if 0	/* FIXME MRFLD */
	/* Check if monitor is attached to HDMI connector. */
	intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &data);

	if (data & HPD_SIGNAL_STATUS) {
#endif	/* FIXME MRFLD */
		if (1) {
			DRM_DEBUG("%s: HPD connected data = 0x%x.\n", __func__,
				  data);

			if (dsi_config) {
				hdmi_priv->mimic_mode =
				    drm_mode_duplicate(dev,
						       dsi_config->fixed_mode);
				drm_mode_set_name(hdmi_priv->mimic_mode);
			}

			if (connector->status == connector_status_connected) {
				/*
				 * Don't un-gate Display B if HDMI is connected
				 * and in D0i3 state.
				 */
				if (hdmi_state == 0)
					hdmi_state = 1;
				return connector_status_connected;
			}

			hdmi_state = 1;

			dev_priv->panel_desc |= DISPLAY_B;
			/*
			   Handle Hot-plug of HDMI. Display B would be
			   power-gated by ospm_post_init if HDMI is not
			   detected during driver load. This will power-up
			   Display B if HDMI is connected post driver load.
			 */
			/*
			   If pmu_nc_set_power_state fails then accessing HW
			   reg would result in a crash - IERR/Fabric error.
			 */
			if (pmu_nc_set_power_state(OSPM_DISPLAY_B_ISLAND,
						   OSPM_ISLAND_UP,
						   OSPM_REG_TYPE))
				BUG();

			if (dev_priv->hdmi_done_reading_edid ||
			    (mdfld_hdmi_edid_detect(connector) ==
			     connector_status_connected)) {
				if (hdmi_state == 0)
					hdmi_state = 1;
				return connector_status_connected;
			} else {
				hdmi_state = 0;
				return connector_status_disconnected;
			}

		} else {
			DRM_DEBUG("%s: HPD disconnected data = 0x%x.\n",
				  __func__, data);

			/*
			 * Clean up the HDMI connector attached encoder, to make
			 * drm_crtc_helper_set_config() do mode setting each
			 * time, especially when plug out HDMI from a sink
			 * device and plug it in to another one with different
			 * EDID.
			 */
			drm_helper_disable_unused_functions(dev);

			/* FIXME: Need to power gate Display B sub-system.
			 * Currently HDMI sometimes couldn't display after
			 * plugging in cable. */
			hdmi_state = 0;
			return connector_status_disconnected;
		}
#else
	return mdfld_hdmi_edid_detect(connector);
#endif
}

static int mdfld_hdmi_set_property(struct drm_connector *connector,
				   struct drm_property *property,
				   uint64_t value)
{
	struct drm_encoder *pEncoder = connector->encoder;

	if (!strcmp(property->name, "scaling mode") && pEncoder)
		PSB_DEBUG_ENTRY("scaling mode\n");
	else if (!strcmp(property->name, "backlight") && pEncoder)
		PSB_DEBUG_ENTRY("backlight\n");
	else if (!strcmp(property->name, "DPMS") && pEncoder)
		PSB_DEBUG_ENTRY("DPMS\n");

	if (!strcmp(property->name, "scaling mode") && pEncoder) {
		struct psb_intel_crtc *pPsbCrtc =
		    to_psb_intel_crtc(pEncoder->crtc);
		bool bTransitionFromToCentered;
		uint64_t curValue;

		if (!pPsbCrtc)
			goto set_prop_error;

		switch (value) {
		case DRM_MODE_SCALE_FULLSCREEN:
			break;
		case DRM_MODE_SCALE_CENTER:
			break;
		case DRM_MODE_SCALE_NO_SCALE:
			break;
		case DRM_MODE_SCALE_ASPECT:
			break;
		default:
			goto set_prop_error;
		}

		if (drm_connector_property_get_value
		    (connector, property, &curValue))
			goto set_prop_error;

		if (curValue == value)
			goto set_prop_done;

		if (drm_connector_property_set_value
		    (connector, property, value))
			goto set_prop_error;

		bTransitionFromToCentered =
		    (curValue == DRM_MODE_SCALE_NO_SCALE)
		    || (value == DRM_MODE_SCALE_NO_SCALE)
		    || (curValue == DRM_MODE_SCALE_CENTER)
		    || (value == DRM_MODE_SCALE_CENTER);

		if (pPsbCrtc->saved_mode.hdisplay != 0 &&
		    pPsbCrtc->saved_mode.vdisplay != 0) {
			if (bTransitionFromToCentered) {
				if (!drm_crtc_helper_set_mode
				    (pEncoder->crtc, &pPsbCrtc->saved_mode,
				     pEncoder->crtc->x, pEncoder->crtc->y,
				     pEncoder->crtc->fb))
					goto set_prop_error;
			} else {
				struct drm_encoder_helper_funcs *pEncHFuncs =
				    pEncoder->helper_private;
				pEncHFuncs->mode_set(pEncoder,
					&pPsbCrtc->saved_mode,
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
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct edid *edid = NULL;
	int ret = 0;
	int i = 0;
	int monitor_number =
	    sizeof(mdfld_hdmi_edid) / sizeof(struct hdmi_edid_info);

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
	drm_mode_connector_update_edid_property(&psb_intel_output->base, edid);
	ret = drm_add_edid_modes(&psb_intel_output->base, edid);

	return ret;
}

static int mdfld_hdmi_get_modes(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = psb_intel_output->dev_priv;
	struct edid *edid = NULL;
	int ret = 0;
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct drm_display_mode *mode, *t;

	PSB_DEBUG_ENTRY("\n");

	if (dsi_config) {
		/*
		 * In case that the HDMI mimic mode's status is reset to
		 * MODE_UNVERIFIED when getting the complete set of display
		 * modes.
		 */
		if ((hdmi_priv->mimic_mode) &&
		    (hdmi_priv->mimic_mode->status !=
		     dsi_config->fixed_mode->status))
			hdmi_priv->mimic_mode->status =
			    dsi_config->fixed_mode->status;
	}

	/*
	 * Insert MIPI's mode into HDMI connector's mode list as the mimic mode.
	 */
	if (hdmi_priv->mimic_mode)
		drm_mode_probed_add(connector, hdmi_priv->mimic_mode);

	/*
	 * Probe more modes from EDID data.
	 */
	if (!psb_intel_output->hdmi_i2c_adapter) {
		DRM_INFO("Enter mdfld_hdmi_get_modes, i2c_adapter is NULL.\n");

		/*
		 * hard-coded the HDMI_I2C_ADAPTER_ID to be 3,
		 * Should get from GCT
		*/
		psb_intel_output->hdmi_i2c_adapter = i2c_get_adapter(3);
	}

	if (!psb_intel_output->hdmi_i2c_adapter) {
		DRM_INFO
		    ("Enter mdfld_hdmi_get_modes, no valid i2c_adapter .\n");
#if 0				/* FIXME MRFLD */
		return ret;
#endif				/* FIXME MRFLD */
	}

	/* MSIC HW issue would be fixed after C0. */
	if (dev_priv->platform_rev_id >= MDFLD_PNW_C0)
		edid = connector->edid_blob_ptr->data;

	if (edid)
		ret = drm_add_edid_modes(&psb_intel_output->base, edid);
	else
		ret = mdfld_hdmi_get_hardcoded_edid_modes(connector);

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

	if ((!hdmi_priv->mimic_mode) || ((hdmi_priv->mimic_mode) &&
					 (!(hdmi_priv->mimic_mode->type &
					    DRM_MODE_TYPE_PREFERRED))))
		return ret;

	list_for_each_entry_safe(mode, t, &connector->probed_modes, head) {
		/*
		 * Don't set the mode (from EDID) such as 1920x1200 as
		 * preferred, to avoid creating the frame buffer which doesn't
		 * match MIPI panel's resolution. Otherwise MIPI couldn't
		 * display the whole UI region.
		 */
		if ((!drm_mode_equal(mode, hdmi_priv->mimic_mode)) &&
		    (mode->type & DRM_MODE_TYPE_PREFERRED)) {
			mode->type &= ~DRM_MODE_TYPE_PREFERRED;

			if ((!hdmi_priv->edid_preferred_mode) ||
			    /*
			     * In case that the HDMI EDID data has
			     * changed due to connecting to another
			     * HDMI device.
			     */
			    ((hdmi_priv->edid_preferred_mode) &&
			     (!drm_mode_equal(mode,
			     hdmi_priv->edid_preferred_mode)))) {
				if (hdmi_priv->edid_preferred_mode)
					drm_mode_destroy(dev,
						hdmi_priv->edid_preferred_mode);

				hdmi_priv->edid_preferred_mode =
				    drm_mode_duplicate(dev, mode);
			}
		}
	}

	return ret;
}

static int mdfld_hdmi_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct psb_intel_output *psb_intel_output =
	    to_psb_intel_output(connector);
	struct mid_intel_hdmi_priv *hdmi_priv = psb_intel_output->dev_priv;

	PSB_DEBUG_ENTRY("display info. hdisplay = %d, vdisplay = %d.\n",
			mode->hdisplay, mode->vdisplay);

	/*
	 * Skip the mimic mode (such as 480 x 854) valid checking with pixel
	 * clock.
	 */
	if ((hdmi_priv->mimic_mode == NULL) ||
	    ((hdmi_priv->mimic_mode != NULL) &&
	     (!drm_mode_equal(mode, hdmi_priv->mimic_mode)))) {
		if (mode->clock > 165000)
			return MODE_CLOCK_HIGH;
		if (mode->clock < 20000)
			return MODE_CLOCK_HIGH;
	}

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
#ifdef CONFIG_PM_RUNTIME
	struct drm_device *dev = connector->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	bool panel_on, panel_on2;
#endif
	/*first, execute dpms */

	drm_helper_connector_dpms(connector, mode);

#ifdef CONFIG_PM_RUNTIME
	if (is_panel_vid_or_cmd(dev)) {
		/*DPI panel */
		panel_on = dev_priv->dpi_panel_on;
		panel_on2 = dev_priv->dpi_panel_on2;
	} else {
		/*DBI panel */
		panel_on = dev_priv->dbi_panel_on;
		panel_on2 = dev_priv->dbi_panel_on2;
	}

	/*then check all display panels + monitors status */
	if (!panel_on && !panel_on2
	    && !(REG_READ(HDMIB_CONTROL) & HDMIB_PORT_EN)) {
		/*request rpm idle */
		if (dev_priv->rpm_enabled)
			pm_request_idle(&dev->pdev->dev);
	}

	/**
	 * if rpm wasn't enabled yet, try to allow it
	 * FIXME: won't enable rpm for DPI since DPI
	 * CRTC setting is a little messy now.
	 * Enable it later!
	 */
#if 0				/* FIXME: won't enable rpm for DPI.  */
	if (!dev_priv->rpm_enabled && !is_panel_vid_or_cmd(dev))
		ospm_runtime_pm_allow(dev);
#endif
#endif
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
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_intel_output *psb_intel_output;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct mid_intel_hdmi_priv *hdmi_priv;

	PSB_DEBUG_ENTRY("\n");

	psb_intel_output = kzalloc(sizeof(struct psb_intel_output) +
				   sizeof(struct mid_intel_hdmi_priv),
				   GFP_KERNEL);
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
	/*
	 * FIXME: May need to get this somewhere,
	 * but CG code seems hard coded it
	*/
	hdmi_priv->hdmib_reg = HDMIB_CONTROL;
	hdmi_priv->has_hdmi_sink = false;
	psb_intel_output->dev_priv = hdmi_priv;

	drm_encoder_helper_add(encoder, &mdfld_hdmi_helper_funcs);
	drm_connector_helper_add(connector, &mdfld_hdmi_connector_helper_funcs);
	connector->display_info.subpixel_order = SubPixelHorizontalRGB;
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	drm_connector_attach_property(connector,
				      dev->mode_config.scaling_mode_property,
				      DRM_MODE_SCALE_FULLSCREEN);

	/* hard-coded the HDMI_I2C_ADAPTER_ID to be 3, Should get from GCT */
	psb_intel_output->hdmi_i2c_adapter = i2c_get_adapter(3);

	if (psb_intel_output->hdmi_i2c_adapter)
		DRM_INFO("Enter mdfld_hdmi_init, i2c_adapter is availabe.\n");
	else
		printk(KERN_ALERT "No ddc adapter available!\n");

	dev_priv->hdmi_priv = hdmi_priv;
	hdmi_priv->is_hdcp_supported = true;
	hdmi_priv->hdmi_i2c_adapter = psb_intel_output->hdmi_i2c_adapter;
	hdmi_priv->dev = dev;
	hdmi_priv->mimic_mode = NULL;
	hdmi_priv->edid_preferred_mode = NULL;
	mdfld_hdcp_init(hdmi_priv);
	mdfld_hdmi_audio_init(hdmi_priv);
	mdfld_msic_init(hdmi_priv);

#ifdef CONFIG_X86_MRST
	/* turn on HDMI power rails. These will be on in all non-S0iX
	   states so that HPD and connection status will work. VCC330 will
	   have ~1.7mW usage during idle states when the display is active. */
	intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_ON);

	/* MSIC documentation requires that there be a 500us delay
	   after enabling VCC330 before you can enable VHDMI */
	usleep_range(500, 1000);

	/* Extend VHDMI switch de-bounce time, to avoid redundant MSIC
	 * VREG/HDMI interrupt during HDMI cable plugged in/out. */
	intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_ON | VHDMI_DB_30MS);
#endif
	drm_sysfs_connector_add(connector);
	return;
}
