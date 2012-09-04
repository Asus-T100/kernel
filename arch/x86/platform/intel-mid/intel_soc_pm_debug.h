/*
 * intel_soc_pm_debug.h
 * Copyright (c) 2012, Intel Corporation.
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
 */
#ifndef _INTEL_SOC_PM_DEBUG_H
#define _INTEL_SOC_PM_DEBUG_H
#include <linux/intel_mid_pm.h>

#define NANO_SEC 1000000000UL /* 10^9 in sec */
#define PMU_LOG_INTERVAL_SECS	(60*5) /* 5 mins in secs */

struct island {
	int type;
	int index;
	char *name;
};

struct lss_definition {
	char *lss_name;
	char *block;
	char *subsystem;
};

/* platform dependency starts */
#define DEV_GFX		2
#define FUNC_GFX	0
#define ISLANDS_GFX	8
#define DEV_ISP		3
#define FUNC_ISP	0
#define ISLANDS_ISP	2
#define NC_DEVS		2

static struct lss_definition lsses[] = {
	{"Lss00", "Storage", "SDIO0 (HC2)"},
	{"Lss01", "Storage", "eMMC0 (HC0a)"},
	{"NA", "Storage", "ND_CTL (Note 5)"},
	{"Lss03", "H S I", "H S I DMA"},
	{"Lss04", "Security", "RNG"},
	{"Lss05", "Storage", "eMMC1 (HC0b)"},
	{"Lss06", "USB", "USB OTG (ULPI)"},
	{"Lss07", "USB", "USB_SPH"},
	{"Lss08", "Audio", ""},
	{"Lss09", "Audio", ""},
	{"Lss10", "SRAM", " SRAM CTL+SRAM_16KB"},
	{"Lss11", "SRAM", " SRAM CTL+SRAM_16KB"},
	{"Lss12", "SRAM", "SRAM BANK (16KB+3x32KBKB)"},
	{"Lss13", "SRAM", "SRAM BANK(4x32KB)"},
	{"Lss14", "SDIO COMMS", "SDIO2 (HC1b)"},
	{"Lss15", "PTI, DAFCA", " DFX Blocks"},
	{"Lss16", "SC", " DMA"},
	{"NA", "SC", "SPI0/MSIC"},
	{"Lss18", "GP", "SPI1"},
	{"Lss19", "GP", " SPI2"},
	{"Lss20", "GP", " I2C0"},
	{"Lss21", "GP", " I2C1"},
	{"NA", "Fabrics", " Main Fabric"},
	{"NA", "Fabrics", " Secondary Fabric"},
	{"NA", "SC", "SC Fabric"},
	{"Lss25", "Audio", " I-RAM BANK1 (32 + 256KB)"},
	{"NA", "SCU", " ROM BANK1 (18KB+18KB+18KB)"},
	{"Lss27", "GP", "I2C2"},
	{"NA", "SSC", "SSC (serial bus controller to FLIS)"},
	{"Lss29", "Security", "Chaabi AON Registers"},
	{"Lss30", "SDIO COMMS", "SDIO1 (HC1a)"},
	{"NA", "SCU", "I-RAM BANK0 (32KB)"},
	{"NA", "SCU", "I-RAM BANK1 (32KB)"},
	{"Lss33", "GP", "I2C3 (HDMI)"},
	{"Lss34", "GP", "I2C4"},
	{"Lss35", "GP", "I2C5"},
	{"Lss36", "GP", "SSP (SPI3)"},
	{"Lss37", "GP", "GPIO1"},
	{"NA", "GP", "GP Fabric"},
	{"Lss39", "SC", "GPIO0"},
	{"Lss40", "SC", "KBD"},
	{"Lss41", "SC", "UART2:0"},
	{"NA", "NA", "NA"},
	{"NA", "NA", "NA"},
	{"Lss44", "Security", " Security TAPC"},
	{"NA", "MISC", "AON Timers"},
	{"NA", "PLL", "LFHPLL and Spread Spectrum"},
	{"NA", "PLL", "USB PLL"},
	{"NA", "NA", "NA"},
	{"NA", "Audio", "SLIMBUS CTL 1 (note 5)"},
	{"NA", "Audio", "SLIMBUS CTL 2 (note 5)"},
	{"Lss51", "Audio", "SSP0"},
	{"Lss52", "Audio", "SSP1"},
	{"NA", "Bridge", "IOSF to OCP Bridge"},
	{"Lss54", "GP", "DMA"},
	{"NA", "SC", "SVID (Serial Voltage ID)"},
	{"NA", "SOC Fuse", "SoC Fuse Block (note 3)"},
	{"NA", "NA", "NA"},
};
/* platform dependency ends */

#endif
