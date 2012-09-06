/*
 * Copyright (c) 2009, Intel Corporation
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
 */

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include "psb_intel_drv.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"

/* Fixed name */
#define ACPI_EDID_LCD	"\\_SB_.PCI0.GFX0.DD04._DDC"
#define ACPI_DOD	"\\_SB_.PCI0.GFX0._DOD"

#include "psb_intel_i2c.c"
#include "psb_intel_sdvo.c"
#include "psb_intel_modes.c"
#include "psb_intel_lvds.c"
#include "psb_intel_dsi.c"
#include "psb_intel_dsi2.c"
#include "psb_intel_display.c"
