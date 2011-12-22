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

#include "intel_fabricid_def.h"

static char *FullChip_FlagStatusLow32[] = {
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

static char *FullChip_FlagStatusHi32[] = {
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

static char *Audio_FlagStatusHi32[] = {
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

static char *SC_FlagStatusLow32[] = {
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

static char *SC_FlagStatusHi32[] = {
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

char *fabric_error_lookup(u32 fab_id, u32 error_index, int use_hidword)
{
	if (error_index > 31) /* Out of range of 32bit */
		return NULL;

	switch (fab_id) {
	case FAB_ID_FULLCHIP:
		return use_hidword ? FullChip_FlagStatusHi32[error_index] :
			FullChip_FlagStatusLow32[error_index];

	case FAB_ID_SECONDARY:
		return use_hidword ? Secondary_FlagStatusHi32[error_index] :
			Secondary_FlagStatusLow32[error_index];

	case FAB_ID_AUDIO:
		return use_hidword ? Audio_FlagStatusHi32[error_index] :
			Audio_FlagStatusLow32[error_index];

	case FAB_ID_GP:
		return use_hidword ? GP_FlagStatusHi32[error_index] :
			GP_FlagStatusLow32[error_index];

	case FAB_ID_SC:
		return use_hidword ? SC_FlagStatusHi32[error_index] :
			SC_FlagStatusLow32[error_index];

	default:
		return NULL;
	}

	return NULL;
}
