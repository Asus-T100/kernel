/*
 * drivers/misc/intel_fabricerr_status.c
 *
 * Copyright (C) 2011 Intel Corp
 * Author: winson.w.yung@intel.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/types.h>
#include <asm/intel-mid.h>

#include "intel_fabricid_def.h"

static char *FullChip_FlagStatusLow32_pnw[] = {
	"cdmi_iocp (IA Burst Timeout)",		/* bit 0 */
	"cha_iahb (IA Burst Timeout)",		/* bit 1 */
	"nand_iaxi (IA Burst Timeout)",		/* bit 2 */
	"otg_iahb (IA Burst Timeout)",		/* bit 3 */
	"usb_iahb (IA Burst Timeout)",		/* bit 4 */
	"usc0a_iahb (IA Burst Timeout)",	/* bit 5 */
	"usc0b_iahb (IA Burst Timeout)",	/* bit 6 */
	"usc2_iahb (IA Burst Timeout)",		/* bit 7 */
	"tra0_iocp (IA Burst Timeout)",		/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"cdmi_iocp (IA Resp Timeout)",		/* bit 16 */
	"cha_iahb (IA Resp Timeout)",		/* bit 17 */
	"nand_iaxi (IA Resp Timeout)",		/* bit 18 */
	"otg_iahb (IA Resp Timeout)",		/* bit 19 */
	"usb_iahb (IA Resp Timeout)",		/* bit 20 */
	"usc0a_iahb (IA Resp Timeout)",		/* bit 21 */
	"usc0b_iahb (IA Resp Timeout)",		/* bit 22 */
	"usc2_iahb (IA Resp Timeout)",		/* bit 23 */
	"tra0_iocp (IA Resp Timeout)",		/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *FullChip_FlagStatusLow32_clv[] = {
	"cdmi_iocp (IA Burst Timeout)",		/* bit 0 */
	"cha_iahb (IA Burst Timeout)",		/* bit 1 */
	"",					/* bit 2 */
	"otg_iahb (IA Burst Timeout)",		/* bit 3 */
	"usb_iahb (IA Burst Timeout)",		/* bit 4 */
	"usc0a_iahb (IA Burst Timeout)",	/* bit 5 */
	"usc0b_iahb (IA Burst Timeout)",	/* bit 6 */
	"usc2_iahb (IA Burst Timeout)",		/* bit 7 */
	"tra0_iocp (IA Burst Timeout)",		/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"cdmi_iocp (IA Resp Timeout)",		/* bit 16 */
	"cha_iahb (IA Resp Timeout)",		/* bit 17 */
	"",					/* bit 18 */
	"otg_iahb (IA Resp Timeout)",		/* bit 19 */
	"usb_iahb (IA Resp Timeout)",		/* bit 20 */
	"usc0a_iahb (IA Resp Timeout)",		/* bit 21 */
	"usc0b_iahb (IA Resp Timeout)",		/* bit 22 */
	"usc2_iahb (IA Resp Timeout)",		/* bit 23 */
	"tra0_iocp (IA Resp Timeout)",		/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *FullChip_FlagStatusLow32_tng[] = {
	"iosf2ocp_i0 (IA Burst Timeout)",	/* bit 0 */
	"usb3_i0 (IA Burst Timeout)",		/* bit 1 */
	"usb3_i1 (IA Burst Timeout)",		/* bit 2 */
	"mfth_i0 (IA Burst Timeout)",		/* bit 3 */
	"cha_i0 (IA Burst Timeout)",		/* bit 4 */
	"otg_i0 (IA Burst Timeout)",		/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"iosf2ocp_i0 (IA Response Timeout)",	/* bit 10 */
	"usb3_i0 (IA Response Timeout)",	/* bit 11 */
	"usb3_i1 (IA Response Timeout)",	/* bit 12 */
	"mfth_i0 (IA Response Timeout)",	/* bit 13 */
	"cha_i0 (IA Response Timeout)",		/* bit 14 */
	"otg_i0 (IA Response Timeout)",		/* bit 15 */
	"",					/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"iosf2ocp_i0 (IA InBand Error)",	/* bit 20 */
	"usb3_i0 (IA InBand Error)",		/* bit 21 */
	"usb3_i1 (IA InBand Error)",		/* bit 22 */
	"mfth_i0 (IA InBand Error)",		/* bit 23 */
	"cha_i0 (IA InBand Error)",		/* bit 24 */
	"otg_i0 (IA InBand Error)",		/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"iosf2ocp_t0 (TA Request Timeout)",	/* bit 30 */
	"usb3_t0 (TA Request Timeout)"		/* bit 31 */
};

static char *FullChip_FlagStatusHi32_pnw[] = {
	"cdmi_iocp (IA Inband Error)",		/* bit 32 */
	"cha_iahb (IA Inband Error)",		/* bit 33 */
	"nand_iaxi (IA Inband Error)",		/* bit 34 */
	"otg_iahb (IA Inband Error)",		/* bit 35 */
	"usb_iahb (IA Inband Error)",		/* bit 36 */
	"usc0a_iahb (IA Inband Error)",		/* bit 37 */
	"usc0b_iahb (IA Inband Error)",		/* bit 38 */
	"usc2_iahb (IA Inband Error)",		/* bit 39 */
	"tra0_iocp (IA Inband Error)",		/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"cdmi_tocp (TA Req Timeout)",		/* bit 48 */
	"cha_tahb (TA Req Timeout)",		/* bit 49 */
	"nand_taxi (TA Req Timeout)",		/* bit 50 */
	"nandreg_taxi (TA Req Timeout)",	/* bit 51 */
	"otg_tahb (TA Req Timeout)",		/* bit 52 */
	"usb_tahb (TA Req Timeout)",		/* bit 53 */
	"usc0a_tahb (TA Req Timeout)",		/* bit 54 */
	"usc0b_tahb (TA Req Timeout)",		/* bit 55 */
	"usc2_tahb (TA Req Timeout)",		/* bit 56 */
	"pti_tocp (TA Req Timeout)",		/* bit 57 */
	"tra0_tocp (TA Req Timeout)",		/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *FullChip_FlagStatusHi32_clv[] = {
	"cdmi_iocp (IA Inband Error)",		/* bit 32 */
	"cha_iahb (IA Inband Error)",		/* bit 33 */
	"",					/* bit 34 */
	"otg_iahb (IA Inband Error)",		/* bit 35 */
	"usb_iahb (IA Inband Error)",		/* bit 36 */
	"usc0a_iahb (IA Inband Error)",		/* bit 37 */
	"usc0b_iahb (IA Inband Error)",		/* bit 38 */
	"usc2_iahb (IA Inband Error)",		/* bit 39 */
	"tra0_iocp (IA Inband Error)",		/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"cdmi_tocp (TA Req Timeout)",		/* bit 48 */
	"cha_tahb (TA Req Timeout)",		/* bit 49 */
	"",					/* bit 50 */
	"",					/* bit 51 */
	"otg_tahb (TA Req Timeout)",		/* bit 52 */
	"usb_tahb (TA Req Timeout)",		/* bit 53 */
	"usc0a_tahb (TA Req Timeout)",		/* bit 54 */
	"usc0b_tahb (TA Req Timeout)",		/* bit 55 */
	"usc2_tahb (TA Req Timeout)",		/* bit 56 */
	"pti_tocp (TA Req Timeout)",		/* bit 57 */
	"tra0_tocp (TA Req Timeout)",		/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *FullChip_FlagStatusHi32_tng[] = {
	"ptistm_t0 (TA Request Timeout)",	/* bit 32 */
	"ptistm_t1 (TA Request Timeout)",	/* bit 33 */
	"ptistm_t2 (TA Request Timeout)",	/* bit 34 */
	"mfth_t0 (TA Request Timeout)",		/* bit 35 */
	"cha_t0 (TA Request Timeout)",		/* bit 36 */
	"otg_t0 (TA Request Timeout)",		/* bit 37 */
	"runctl_t0 (TA Request Timeout)",	/* bit 38 */
	"usb3phy_t0 (TA Request Timeout)",	/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"",					/* bit 48 */
	"",					/* bit 49 */
	"iosf2ocp_t0 (Access Control Violation)",/* bit 50 */
	"usb3_t0 (Access Control Violation)",	/* bit 51 */
	"ptistm_t0 (Access Control Violation)",	/* bit 52 */
	"ptistm_t1 (Access Control Violation)",	/* bit 53 */
	"ptistm_t2 (Access Control Violation)",	/* bit 54 */
	"mfth_t0 (Access Control Violation)",	/* bit 55 */
	"cha_t0 (Access Control Violation)",	/* bit 56 */
	"otg_t0 (Access Control Violation)",	/* bit 57 */
	"runctl_t0 (Access Control Violation)",	/* bit 58 */
	"usb3phy_t0 (Access Control Violation)",/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *Secondary_FlagStatusLow32[] = {
	"usc1a_iahb (IA Burst Timeout)",	/* bit 0 */
	"usc1b_iahb (IA Burst Timeout)",	/* bit 1 */
	"hsidma_iahb (IA Burst Timeout)",	/* bit 2 */
	"tra1_iocp (IA Burst Timeout)",		/* bit 3 */
	"dfx_iahb (IA Burst Timeout)",		/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"usc1a_iahb (IA Resp Timeout)",		/* bit 16 */
	"usc1b_iahb (IA Resp Timeout)",		/* bit 17 */
	"hsidma_iahb (IA Resp Timeout)",	/* bit 18 */
	"tra1_iocp (IA Resp Timeout)",		/* bit 19 */
	"dfx_iahb (IA Resp Timeout)",		/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"",					/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *Secondary_FlagStatusLow32_tng[] = {
	"sdio0_i0 (IA Burst Timeout)",		/* bit 0 */
	"emmc01_i0 (IA Burst Timeout)",		/* bit 1 */
	"emmc01_i1 (IA Burst Timeout)",		/* bit 2 */
	"sdio1_i0 (IA Burst Timeout)",		/* bit 3 */
	"hsi_i0 (IA Burst Timeout)",		/* bit 4 */
	"mph_i0 (IA Burst Timeout)",		/* bit 5 */
	"sfth_i0 (IA Burst Timeout)",		/* bit 6 */
	"dfxsctap_i0 (IA Burst Timeout)",	/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"sdio0_i0 (IA Response Timeout)",	/* bit 10 */
	"emmc01_i0 (IA Response Timeout)",	/* bit 11 */
	"emmc01_i1 (IA Response Timeout)",	/* bit 12 */
	"sdio1_i0 (IA Response Timeout)",	/* bit 13 */
	"hsi_i0 (IA Response Timeout)",		/* bit 14 */
	"mph_i0 (IA Response Timeout)",		/* bit 15 */
	"sfth_i0 (IA Response Timeout)",	/* bit 16 */
	"dfxsctap_i0 (IA Response Timeout)",	/* bit 17 */
	"",					/* bit 18 */
	"sdio0_i0 (IA InBand Error)",		/* bit 19 */
	"emmc01_i0 (IA InBand Error)",		/* bit 20 */
	"emmc01_i1 (IA InBand Error)",		/* bit 21 */
	"sdio1_i0 (IA InBand Error)",		/* bit 22 */
	"hsi_i0 (IA InBand Error)",		/* bit 23 */
	"mph_i0 (IA InBand Error)",		/* bit 24 */
	"sfth_i0 (IA InBand Error)",		/* bit 25 */
	"dfxsctap_i0 (IA InBand Error)",	/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"sram_t0 (TA Request Timeout)",		/* bit 30 */
	"sdio0_t0 (TA Request Timeout)"		/* bit 31 */
};

static char *Secondary_FlagStatusHi32[] = {
	"usc1a_iahb (IA Inband Error)",		/* bit 32 */
	"usc1b_iahb (IA Inband Error)",		/* bit 33 */
	"hsidma_iahb (IA Inband Error)",	/* bit 34 */
	"tra1_iocp (IA Inband Error)",		/* bit 35 */
	"dfx_iahb (IA Inband Error)",		/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"usc1a_tahb (TA Req Timeout)",		/* bit 48 */
	"usc1b_tahb (TA Req Timeout)",		/* bit 49 */
	"hsi_tocp (TA Req Timeout)",		/* bit 50 */
	"hsidma_tahb (TA Req Timeout)",		/* bit 51 */
	"sram_tocp (TA Req Timeout)",		/* bit 52 */
	"tra1_tocp (TA Req Timeout)",		/* bit 53 */
	"i2c3ssc_tocp (TA Req Timeout)",	/* bit 54 */
	"",					/* bit 55 */
	"",					/* bit 56 */
	"",					/* bit 57 */
	"",					/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *Secondary_FlagStatusHi32_tng[] = {
	"emmc01_t0 (TA Request Timeout)",	/* bit 32 */
	"emmc01_t1 (TA Request Timeout)",	/* bit 33 */
	"sdio1_t0 (TA Request Timeout)",	/* bit 34 */
	"hsi_t0 (TA Request Timeout)",		/* bit 35 */
	"mph_t0 (TA Request Timeout)",		/* bit 36 */
	"sfth_t0 (TA Request Timeout)",		/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"",					/* bit 48 */
	"",					/* bit 49 */
	"sram_t0 (Access Control Violation)",	/* bit 50 */
	"sdio0_t0 (Access Control Violation)",	/* bit 51 */
	"emmc01_t0 (Access Control Violation)",	/* bit 52 */
	"emmc01_t1 (Access Control Violation)",	/* bit 53 */
	"sdio1_t0 (Access Control Violation)",	/* bit 54 */
	"hsi_t0 (Access Control Violation)",	/* bit 55 */
	"mph_t0 (Access Control Violation)",	/* bit 56 */
	"sfth_t0 (Access Control Violation)",	/* bit 57 */
	"",					/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *Audio_FlagStatusLow32[] = {
	"aes_iahb (IA Burst Timeout)",		/* bit 0 */
	"adma_iahb (IA Burst Timeout)",		/* bit 1 */
	"adma2_iahb (IA Burst Timeout)",	/* bit 2 */
	"",					/* bit 3 */
	"",					/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"aes_iahb (IA Resp Timeout)",		/* bit 16 */
	"adma_iahb (IA Resp Timeout)",		/* bit 17 */
	"adma2_iahb (IA Resp Timeout)",		/* bit 18 */
	"",					/* bit 19 */
	"",					/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"",					/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *Audio_FlagStatusLow32_tng[] = {
	"pifocp_i0 (IA Burst Timeout)",		/* bit 0 */
	"adma0_i0 (IA Burst Timeout)",		/* bit 1 */
	"adma0_i1 (IA Burst Timeout)",		/* bit 2 */
	"adma1_i0 (IA Burst Timeout)",		/* bit 3 */
	"adma1_i1 (IA Burst Timeout)",		/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"pifocp_i0 (IA Response Timeout)",	/* bit 10 */
	"adma0_i0 (IA Response Timeout)"	/* bit 11 */
	"adma0_i1 (IA Response Timeout)",	/* bit 12 */
	"adma1_i0 (IA Response Timeout)",	/* bit 13 */
	"adma1_i1 (IA Response Timeout)",	/* bit 14 */
	"",					/* bit 15 */
	"",					/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"pifocp_i0 (IA InBand Error)",		/* bit 20 */
	"adma0_i0 (IA InBand Error)",		/* bit 21 */
	"adma0_i1 (IA InBand Error)",		/* bit 22 */
	"adma1_i0 (IA InBand Error)",		/* bit 23 */
	"adma1_i1 (IA InBand Error)",		/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"ssp0_t0 (TA Request Timeout)",		/* bit 30 */
	"ssp1_t0 (TA Request Timeout)"		/* bit 31 */
};

static char *Audio_FlagStatusHi32_pnw[] = {
	"aes_iahb (IA Inband Error)",		/* bit 32 */
	"adma_iahb (IA Inband Error)",		/* bit 33 */
	"adma2_iahb (IA Inband Error)",		/* bit 34 */
	"",					/* bit 35 */
	"",					/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"aes_tahb (TA Req Timeout)",		/* bit 48 */
	"adma_tahb (TA Req Timeout)",		/* bit 49 */
	"adram2_tocp (TA Req Timeout)",		/* bit 50 */
	"adram_tocp (TA Req Timeout)",		/* bit 51 */
	"airam_tocp (TA Req Timeout)",		/* bit 52 */
	"assp1_1_tapb (TA Req Timeout)",	/* bit 53 */
	"assp2_2_tahb (TA Req Timeout)",	/* bit 54 */
	"adma2_tahb (TA Req Timeout)",		/* bit 55 */
	"slim0_iocp (TA Req Timeout)",		/* bit 56 */
	"slim1_iocp (TA Req Timeout)",		/* bit 57 */
	"slim2_iocp (TA Req Timeout)",		/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *Audio_FlagStatusHi32_clv[] = {
	"aes_iahb (IA Inband Error)",		/* bit 32 */
	"adma_iahb (IA Inband Error)",		/* bit 33 */
	"adma2_iahb (IA Inband Error)",		/* bit 34 */
	"",					/* bit 35 */
	"",					/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"aes_tahb (TA Req Timeout)",		/* bit 48 */
	"adma_tahb (TA Req Timeout)",		/* bit 49 */
	"adram2_tocp (TA Req Timeout)",		/* bit 50 */
	"adram_tocp (TA Req Timeout)",		/* bit 51 */
	"airam_tocp (TA Req Timeout)",		/* bit 52 */
	"assp1_1_tapb (TA Req Timeout)",	/* bit 53 */
	"assp_2_tapb (TA Req Timeout)",		/* bit 54 */
	"adma2_tahb (TA Req Timeout)",		/* bit 55 */
	"assp_3_tapb (TA Req Timeout)",		/* bit 56 */
	"",					/* bit 57 */
	"",					/* bit 58 */
	"assp_4_tapb (TA Req Timeout)",		/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *Audio_FlagStatusHi32_tng[] = {
	"ssp2_t0 (TA Request Timeout)",		/* bit 32 */
	"slim1_t0 (TA Request Timeout)",	/* bit 33 */
	"pifocp_t0 (TA Request Timeout)",	/* bit 34 */
	"adma0_t0 (TA Request Timeout)",	/* bit 35 */
	"adma1_t0 (TA Request Timeout)",	/* bit 36 */
	"mboxram_t0 (TA Request Timeout)",	/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"",					/* bit 48 */
	"",					/* bit 49 */
	"ssp0_t0 (Access Control Violation)",	/* bit 50 */
	"ssp1_t0 (Access Control Violation)",	/* bit 51 */
	"ssp2_t0 (Access Control Violation)",	/* bit 52 */
	"slim1_t0 (Access Control Violation)",	/* bit 53 */
	"pifocp_t0 (Access Control Violation)",	/* bit 54 */
	"adma0_t0 (Access Control Violation)",	/* bit 55 */
	"adma1_t0 (Access Control Violation)",	/* bit 56 */
	"mboxram_t0 (Access Control Violation)",/* bit 57 */
	"",					/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *GP_FlagStatusLow32[] = {
	"gpdma_iahb (IA Burst Timeout)",	/* bit 0 */
	"",					/* bit 1 */
	"",					/* bit 2 */
	"",					/* bit 3 */
	"",					/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"gpdma_iahb (IA Resp Timeout)",		/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"",					/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"",					/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *GP_FlagStatusLow32_tng[] = {
	"gpdma_i0 (IA Burst Timeout)",		/* bit 0 */
	"gpdma_i1 (IA Burst Timeout)",		/* bit 1 */
	"",					/* bit 2 */
	"",					/* bit 3 */
	"",					/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"gpdma_i0 (IA Response Timeout)",	/* bit 10 */
	"gpdma_i1 (IA Response Timeout)",	/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"",					/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"gpdma_i0 (IA InBand Error)",		/* bit 20 */
	"gpdma_i1 (IA InBand Error)",		/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"",					/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"spi5_t0 (TA Request Timeout)",		/* bit 30 */
	"ssp6_t0 (TA Request Timeout)"		/* bit 31 */
};

static char *GP_FlagStatusHi32[] = {
	"gpdma_iahb (IA Inband Error)",		/* bit 32 */
	"",					/* bit 33 */
	"",					/* bit 34 */
	"",					/* bit 35 */
	"",					/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"gpio1_tocp (TA Req Timeout)",		/* bit 48 */
	"i2c0_tocp (TA Req Timeout)",		/* bit 49 */
	"i2c1_tocp (TA Req Timeout)",		/* bit 50 */
	"i2c2_tocp (TA Req Timeout)",		/* bit 51 */
	"i2c3hdmi_tocp (TA Req Timeout)",	/* bit 52 */
	"i2c4_tocp (TA Req Timeout)",		/* bit 53 */
	"i2c5_tocp (TA Req Timeout)",		/* bit 54 */
	"spi1_tocp (TA Req Timeout)",		/* bit 55 */
	"spi2_tocp (TA Req Timeout)",		/* bit 56 */
	"spi3_tocp (TA Req Timeout)",		/* bit 57 */
	"gpdma_tahb (TA Req Timeout)",		/* bit 58 */
	"i2c3scc_tocp (TA Req Timeout)",	/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *GP_FlagStatusHi32_tng[] = {
	"gpdma_t0 (TA Request Timeout)",	/* bit 32 */
	"i2c12_t0 (TA Request Timeout)",	/* bit 33 */
	"i2c12_t1 (TA Request Timeout)",	/* bit 34 */
	"i2c3_t0 (TA Request Timeout)",		/* bit 35 */
	"i2c45_t0 (TA Request Timeout)",	/* bit 36 */
	"i2c45_t1 (TA Request Timeout)",	/* bit 37 */
	"i2c67_t0 (TA Request Timeout)",	/* bit 38 */
	"i2c67_t1 (TA Request Timeout)",	/* bit 39 */
	"ssp3_t0 (TA Request Timeout)",		/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"",					/* bit 48 */
	"",					/* bit 49 */
	"spi5_t0 (Access Control Violation)",	/* bit 50 */
	"ssp6_t0 (Access Control Violation)",	/* bit 51 */
	"gpdma_t0 (Access Control Violation)",	/* bit 52 */
	"i2c12_t0 (Access Control Violation)",	/* bit 53 */
	"i2c12_t1 (Access Control Violation)",	/* bit 54 */
	"i2c3_t0 (Access Control Violation)",	/* bit 55 */
	"i2c45_t0 (Access Control Violation)",	/* bit 56 */
	"i2c45_t1 Access Control Violation)",	/* bit 57 */
	"i2c67_t0 (Access Control Violation)",	/* bit 58 */
	"i2c67_t1 (Access Control Violation)",	/* bit 59 */
	"ssp3_t0 (Access Control Violation)",	/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *SC_FlagStatusLow32_pnw[] = {
	"MFlag0 (Audio)",			/* bit 0 */
	"MFlag1 (Secondary)",			/* bit 1 */
	"MFlag2 (FullChip)",			/* bit 2 */
	"MFlag3 (GP)",				/* bit 3 */
	"",					/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"arc_iocp (IA Burst Timeout)",		/* bit 8 */
	"scdma_iocp (IA Burst Timeout)",	/* bit 9 */
	"uart_iocp (IA Burst Timeout)",		/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"arc_iocp (IA Resp Timeout)",		/* bit 16 */
	"scdma_iocp (IA Resp Timeout)",		/* bit 17 */
	"uart_iocp (IA Resp Timeout)",		/* bit 18 */
	"",					/* bit 19 */
	"",					/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"arc_iocp (IA Inband Error)",		/* bit 24 */
	"scdma_iocp (IA Inband Error)",		/* bit 25 */
	"uart_iocp (IA Inband Error)",		/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *SC_FlagStatusLow32_clv[] = {
	"MFlag0 (Audio)",			/* bit 0 */
	"MFlag1 (Secondary)",			/* bit 1 */
	"MFlag2 (FullChip)",			/* bit 2 */
	"MFlag3 (GP)",				/* bit 3 */
	"",					/* bit 4 */
	"",					/* bit 5 */
	"",					/* bit 6 */
	"ilb_iocp (IA Burst Timeout)",		/* bit 7 */
	"arc_iocp (IA Burst Timeout)",		/* bit 8 */
	"scdma_iocp (IA Burst Timeout)",	/* bit 9 */
	"uart_iocp (IA Burst Timeout)",		/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"arc_iocp (IA Resp Timeout)",		/* bit 16 */
	"scdma_iocp (IA Resp Timeout)",		/* bit 17 */
	"uart_iocp (IA Resp Timeout)",		/* bit 18 */
	"ilb_iocp (IA Resp Timeout)",		/* bit 19 */
	"",					/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"arc_iocp (IA Inband Error)",		/* bit 24 */
	"scdma_iocp (IA Inband Error)",		/* bit 25 */
	"uart_iocp (IA Inband Error)",		/* bit 26 */
	"ilb_iocp (IA Inband Error)",		/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

static char *SC_FlagStatusLow32_tng[] = {
	"ADF Flag Status",			/* bit 0 */
	"SDF Flag Status",			/* bit 1 */
	"MNF Flag Status",			/* bit 2 */
	"GPF Flag Status",			/* bit 3 */
	"ilb_i0 (IA Burst Timeout)",		/* bit 4 */
	"scdma_i0 (IA Burst Timeout)",		/* bit 5 */
	"scdma_i1 (IA Burst Timeout)",		/* bit 6 */
	"arc_i0 (IA Burst Timeout)",		/* bit 7 */
	"uart_i0 (IA Burst Timeout)",		/* bit 8 */
	"psh_i0 (IA Burst Timeout)",		/* bit 9 */
	"ilb_i0 (IA Response Timeout)",		/* bit 10 */
	"scdma_i0 (IA Response Timeout)",	/* bit 11 */
	"scdma_i1 (IA Response Timeout)",	/* bit 12 */
	"arc_i0 (IA Response Timeout)",		/* bit 13 */
	"uart_i0 (IA Response Timeout)",	/* bit 14 */
	"psh_i0 (IA Response Timeout)",		/* bit 15 */
	"",					/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"ilb_i0 (IA InBand Error)",		/* bit 20 */
	"scdma_i0 (IA InBand Error)",		/* bit 21 */
	"scdma_i1 (IA InBand Error)",		/* bit 22 */
	"arc_i0 (IA InBand Error)",		/* bit 23 */
	"uart_i0 (IA InBand Error)",		/* bit 24 */
	"psh_i0 (IA InBand Error)",		/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"ilb_t0 (TA Request Timeout)",		/* bit 30 */
	"ipc1_t0 (TA Request Timeout)"		/* bit 31 */
};

static char *SC_FlagStatusHi32_pnw[] = {
	"",					/* bit 32 */
	"",					/* bit 33 */
	"",					/* bit 34 */
	"",					/* bit 35 */
	"",					/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"gpio_tocp (TA Req Timeout)",		/* bit 48 */
	"uart_tocp (TA Req Timeout)",		/* bit 49 */
	"ipc1_tocp (TA Req Timeout)",		/* bit 50 */
	"ipc2_tocp (TA Req Timeout)",		/* bit 51 */
	"kbd_tocp (TA Req Timeout)",		/* bit 52 */
	"pmu_tocp (TA Req Timeout)",		/* bit 53 */
	"scdma_tocp (TA Req Timeout)",		/* bit 54 */
	"spi0_tocp (TA Req Timeout)",		/* bit 55 */
	"tim_ocp (TA Req Timeout)",		/* bit 56 */
	"vrtc_tocp (TA Req Timeout)",		/* bit 57 */
	"arcs_tocp (TA Req Timeout)",		/* bit 58 */
	"",					/* bit 59 */
	"",					/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *SC_FlagStatusHi32_clv[] = {
	"",					/* bit 32 */
	"",					/* bit 33 */
	"",					/* bit 34 */
	"",					/* bit 35 */
	"",					/* bit 36 */
	"",					/* bit 37 */
	"",					/* bit 38 */
	"",					/* bit 39 */
	"",					/* bit 40 */
	"",					/* bit 41 */
	"",					/* bit 42 */
	"",					/* bit 43 */
	"",					/* bit 44 */
	"",					/* bit 45 */
	"",					/* bit 46 */
	"",					/* bit 47 */
	"gpio_tocp (TA Req Timeout)",		/* bit 48 */
	"uart_tocp (TA Req Timeout)",		/* bit 49 */
	"ipc1_tocp (TA Req Timeout)",		/* bit 50 */
	"ipc2_tocp (TA Req Timeout)",		/* bit 51 */
	"kbd_tocp (TA Req Timeout)",		/* bit 52 */
	"pmu_tocp (TA Req Timeout)",		/* bit 53 */
	"scdma_tocp (TA Req Timeout)",		/* bit 54 */
	"spi0_tocp (TA Req Timeout)",		/* bit 55 */
	"tim_ocp (TA Req Timeout)",		/* bit 56 */
	"vrtc_tocp (TA Req Timeout)",		/* bit 57 */
	"arcs_tocp (TA Req Timeout)",		/* bit 58 */
	"ilb_tocp (TA Req Timeout)",		/* bit 59 */
	"ilbmb0_tocp (TA Req Timeout)",		/* bit 60 */
	"",					/* bit 61 */
	"",					/* bit 62 */
	""					/* bit 63 */
};

static char *SC_FlagStatusHi32_tng[] = {
	"ipc2_t0 (TA Request Timeout)",		/* bit 32 */
	"mbb_t0 (TA Request Timeout)",		/* bit 33 */
	"spi4_t0 (TA Request Timeout)",		/* bit 34 */
	"scdma_t0 (TA Request Timeout)",	/* bit 35 */
	"kbd_t0 (TA Request Timeout)",		/* bit 36 */
	"sccb_t0 (TA Request Timeout)",		/* bit 37 */
	"timers_t0 (TA Request Timeout)",	/* bit 38 */
	"pmu_t0 (TA Request Timeout)",		/* bit 39 */
	"arc_t0 (TA Request Timeout)",		/* bit 40 */
	"gpio192_t0 (TA Request Timeout)",	/* bit 41 */
	"i2c0_t0 (TA Request Timeout)",		/* bit 42 */
	"uart_t0 (TA Request Timeout)",		/* bit 43 */
	"ssc_t0 (TA Request Timeout)",		/* bit 44 */
	"pwm_t0 (TA Request Timeout)",		/* bit 45 */
	"psh_t0 (TA Request Timeout)",		/* bit 46 */
	"pcache_t0 (TA Request Timeout)",	/* bit 47 */
	"i2c89_t0 (TA Request Timeout)",	/* bit 48 */
	"i2c89_t1 (TA Request Timeout)",	/* bit 49 */
	"ilb_t0 (Access Control Violation)",	/* bit 50 */
	"ipc1_t0 (Access Control Violation)",	/* bit 51 */
	"ipc2_t0 (Access Control Violation)",	/* bit 52 */
	"spi4_t0 (Access Control Violation)",	/* bit 53 */
	"sccb_t0 (Access Control Violation)",	/* bit 54 */
	"timers_t0 (Access Control Violation)",	/* bit 55 */
	"pmu_t0 (Access Control Violation)",	/* bit 56 */
	"arc_t0 (Access Control Violation)",	/* bit 57 */
	"gpio192_t0 (Access Control Violation)",/* bit 58 */
	"i2c0_t0 (Access Control Violation)",	/* bit 59 */
	"ssc_t0 (Access Control Violation)",	/* bit 60 */
	"pcache_t0 (Access Control Violation)",	/* bit 61 */
	"i2c89_t0 (Access Control Violation)",	/* bit 62 */
	"i2c89_t1 (Access Control Violation)"	/* bit 63 */
};

static char *SC_FlagStatus1Low32_tng[] = {
	"mbb_t0 (Access Control Violation)",	/* bit 0 */
	"scdma_t0 (Access Control Violation)",	/* bit 1 */
	"kbd_t0 (Access Control Violation)",	/* bit 2 */
	"uart_t0 (Access Control Violation)",	/* bit 3 */
	"pwm_t0 (Access Control Violation)",	/* bit 4 */
	"psh_t0 (Access Control Violation)",	/* bit 5 */
	"",					/* bit 6 */
	"",					/* bit 7 */
	"",					/* bit 8 */
	"",					/* bit 9 */
	"",					/* bit 10 */
	"",					/* bit 11 */
	"",					/* bit 12 */
	"",					/* bit 13 */
	"",					/* bit 14 */
	"",					/* bit 15 */
	"",					/* bit 16 */
	"",					/* bit 17 */
	"",					/* bit 18 */
	"",					/* bit 19 */
	"",					/* bit 20 */
	"",					/* bit 21 */
	"",					/* bit 22 */
	"",					/* bit 23 */
	"",					/* bit 24 */
	"",					/* bit 25 */
	"",					/* bit 26 */
	"",					/* bit 27 */
	"",					/* bit 28 */
	"",					/* bit 29 */
	"",					/* bit 30 */
	""					/* bit 31 */
};

char *fabric_error_lookup(u32 fab_id, u32 error_index, int use_hidword)
{
	if (error_index > 31) /* Out of range of 32bit */
		return NULL;

	switch (fab_id) {
	case FAB_ID_FULLCHIP:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_PENWELL:
			return use_hidword ?
				FullChip_FlagStatusHi32_pnw[error_index] :
				FullChip_FlagStatusLow32_pnw[error_index];
		case INTEL_MID_CPU_CHIP_CLOVERVIEW:
			return use_hidword ?
				FullChip_FlagStatusHi32_clv[error_index] :
				FullChip_FlagStatusLow32_clv[error_index];
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				FullChip_FlagStatusHi32_tng[error_index] :
				FullChip_FlagStatusLow32_tng[error_index];
		default:
			return NULL;
		}

	case FAB_ID_SECONDARY:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_PENWELL:
		case INTEL_MID_CPU_CHIP_CLOVERVIEW:
			return use_hidword ?
				Secondary_FlagStatusHi32[error_index] :
				Secondary_FlagStatusLow32[error_index];
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				Secondary_FlagStatusHi32_tng[error_index] :
				Secondary_FlagStatusLow32_tng[error_index];
		default:
			return NULL;
		}

	case FAB_ID_AUDIO:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_PENWELL:
			return use_hidword ?
				Audio_FlagStatusHi32_pnw[error_index] :
				Audio_FlagStatusLow32[error_index];
		case INTEL_MID_CPU_CHIP_CLOVERVIEW:
			return use_hidword ?
				Audio_FlagStatusHi32_clv[error_index] :
				Audio_FlagStatusLow32[error_index];
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				Audio_FlagStatusHi32_tng[error_index] :
				Audio_FlagStatusLow32_tng[error_index];
		default:
			return NULL;
		}

	case FAB_ID_GP:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_PENWELL:
		case INTEL_MID_CPU_CHIP_CLOVERVIEW:
			return use_hidword ?
				GP_FlagStatusHi32[error_index] :
				GP_FlagStatusLow32[error_index];
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				GP_FlagStatusHi32_tng[error_index] :
				GP_FlagStatusLow32_tng[error_index];
		default:
			return NULL;
		}

	case FAB_ID_SC:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_PENWELL:
			return use_hidword ?
				SC_FlagStatusHi32_pnw[error_index] :
				SC_FlagStatusLow32_pnw[error_index];
		case INTEL_MID_CPU_CHIP_CLOVERVIEW:
			return use_hidword ?
				SC_FlagStatusHi32_clv[error_index] :
				SC_FlagStatusLow32_clv[error_index];
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				SC_FlagStatusHi32_tng[error_index] :
				SC_FlagStatusLow32_tng[error_index];
		default:
			return NULL;
		}

	case FAB_ID_SC1:
		switch (intel_mid_identify_cpu()) {
		case INTEL_MID_CPU_CHIP_TANGIER:
			return use_hidword ?
				NULL :
				SC_FlagStatus1Low32_tng[error_index];
		default:
			return NULL;
		}

	default:
		return NULL;
	}

	return NULL;
}

enum ErrorType {
	FABRIC_ERR = 0,
	MEMORY_ERR,
	INSTRUCT_ERR,
	ECC_ERR,
	SCU_WDOG_FIRED,
	PLL_LOCKSLIP,
	KERN_WDOG_FIRED,
	UNKNOWN_ERRTYPE
};

static char *Scu_ErrorTypes[] = {
	"Fabric Error",
	"Memory Error",
	"Instruction Error",
	"Shared SRAM ECC Error",
	"SCU Watchdog expired",
	"PLL Lockslip",
	"Kernel Watchdog expired",
	"Unknown"
};

static char *FabricFlagStatusErrLogDetail[] = {
	"Main Fabric Flag Status",
	"Audio Fabric Flag Status",
	"Secondary Fabric Flag Status",
	"GP Fabric Flag Status",
	"Lower 64bit part SC Fabric Flag Status",
	"Upper 64bit part SC Fabric Flag Status",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"TA ERROR LOG register for initiator iosf2ocp_t0 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator iosf2ocp_i0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator psh_t0 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator psh_i0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator arc_t0 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator arc_i0 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator usb3_i0 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator usb3_i1 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator mfth_i0 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator cha_i0 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator otg_i0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator usb3_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator ptistm_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator ptistm_t1 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator ptistm_t2 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator mfth_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator cha_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator otg_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator runctl_t0 in Main Fabric @200MHz{mnf}",
	"TA ERROR LOG register for initiator usb3phy_t0 in Main Fabric @200MHz{mnf}",
	"IA ERROR LOG Register for initiator ilb_i0 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator scdma_i0 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator scdma_i1 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator uart_i0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator ilb_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator ipc1_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator ipc2_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator mbb_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator spi4_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator scdma_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator kbd_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator sccb_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator timers_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator pmu_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator gpio192_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator i2c0_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator uart_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator ssc_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator pwm_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator pcache_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator i2c89_t0 in SC Fabric @100MHz{scf}",
	"TA ERROR LOG register for initiator i2c89_t1 in SC Fabric @100MHz{scf}",
	"IA ERROR LOG Register for initiator gpdma_i0 in GP Fabric @100MHz{gpf}",
	"IA ERROR LOG Register for initiator gpdma_i1 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator spi5_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator ssp6_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator gpdma_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c12_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c12_t1 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c3_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c45_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c45_t1 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c67_t0 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator i2c67_t1 in GP Fabric @100MHz{gpf}",
	"TA ERROR LOG register for initiator ssp3_t0 in GP Fabric @100MHz{gpf}",
	"IA ERROR LOG Register for initiator pifocp_i0 in Audio Fabric @50MHz{adf}",
	"IA ERROR LOG Register for initiator adma0_i0 in Audio Fabric @50MHz{adf}",
	"IA ERROR LOG Register for initiator adma0_i1 in Audio Fabric @50MHz{adf}",
	"IA ERROR LOG Register for initiator adma1_i0 in Audio Fabric @50MHz{adf}",
	"IA ERROR LOG Register for initiator adma1_i1 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator ssp0_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator ssp1_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator ssp2_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator slim1_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator pifocp_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator adma0_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator adma1_t0 in Audio Fabric @50MHz{adf}",
	"TA ERROR LOG register for initiator mboxram_t0 in Audio Fabric @50MHz{adf}",
	"IA ERROR LOG Register for initiator sdio0_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator emmc01_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator emmc01_i1 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator sdio1_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator hsi_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator mph_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator sfth_i0 in Secondary Fabric @100MHz{sdf}",
	"IA ERROR LOG Register for initiator dfxsctap_i0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator sram_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator sdio0_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator emmc01_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator emmc01_t1 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator sdio1_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator hsi_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator mph_t0 in Secondary Fabric @100MHz{sdf}",
	"TA ERROR LOG register for initiator sfth_t0 in Secondary Fabric @100MHz{sdf}",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	""
};

char *get_element_errorlog_detail(
	u8 id, u32 first_dword, u32 second_dword, u32 third_dword)
{
	return FabricFlagStatusErrLogDetail[id];
}

char *get_element_flagsts_detail(
	u8 id, u32 first_dword, u32 second_dword)
{
	return FabricFlagStatusErrLogDetail[id];
}

char *get_errortype_str(u16 error_type)
{
	switch (error_type) {
	case 0xE103:
		return Scu_ErrorTypes[FABRIC_ERR];
	case 0xE101:
		return Scu_ErrorTypes[MEMORY_ERR];
	case 0xE102:
		return Scu_ErrorTypes[INSTRUCT_ERR];
	case 0xE104:
		return Scu_ErrorTypes[ECC_ERR];
	case 0xE107:
		return Scu_ErrorTypes[SCU_WDOG_FIRED];
	case 0xE108:
		return Scu_ErrorTypes[PLL_LOCKSLIP];
	case 0xE10A:
		return Scu_ErrorTypes[KERN_WDOG_FIRED];
	default:
		return Scu_ErrorTypes[UNKNOWN_ERRTYPE];
	}
}

int errorlog_element_type(u8 id_type)
{
	if (id_type >= 0 && id_type <= 15)
		return 0; /* flag_status */
	else
		return 1; /* ia_errorlog */
}
