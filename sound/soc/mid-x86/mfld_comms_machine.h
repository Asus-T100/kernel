/*
 *  mfld_ssp_wl1273_machine.c - ASoC Machine driver for
 *  Intel Medfield MID platform
 *
 *  Copyright (C) 2011 Intel Corp
 *  Author: Selma Bensaid <selma.bensaidl@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef MFLD_SSP_WL1273_MACHINE_H_
#define MFLD_SSP_WL1273_MACHINE_H_
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mid_ssp.h"

/*
 * Structures Definition
 */

struct comms_mc_private {
	bool ssp_master_mode;
};

static int mfld_comms_dai_link_startup(struct snd_pcm_substream *substream);
static int mfld_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params);

/* Data path functionalities */
struct snd_pcm_hardware BT_soc_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_8000),
		.rate_min = 8000,
		.rate_max = 8000,
		.channels_min = 1,
		.channels_max = 1,
		.buffer_bytes_max = (320*1024),
		.period_bytes_min = 32,
		.period_bytes_max = (320*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

struct snd_pcm_hardware FM_soc_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware IFX_modem_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware VOIP_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};


enum {
	BT_SCO_DEV = 0,
	FM_DEV,
	MSIC_VOIP_DEV,
	IFX_MODEM_DEV,
};

/*
 * For FM 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_FM_SLOT_NB_SLOT		1
#define SSP_FM_SLOT_WIDTH		32
#define SSP_FM_SLOT_RX_MASK		0x1
#define SSP_FM_SLOT_TX_MASK		0x0
/*
 * For BT SCO 1 slot of 16 bits is used
 * to transfer mono 16 bits PCM samples
 */
#define SSP_BT_SLOT_NB_SLOT		1
#define SSP_BT_SLOT_WIDTH		16
#define SSP_BT_SLOT_RX_MASK		0x1
#define SSP_BT_SLOT_TX_MASK		0x1

/*
 * For VoIP 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_VOIP_SLOT_NB_SLOT	1
#define SSP_VOIP_SLOT_WIDTH		16
#define SSP_VOIP_SLOT_RX_MASK	0x1
#define SSP_VOIP_SLOT_TX_MASK	0x1

/*
 * For Modem IFX 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_IFX_SLOT_NB_SLOT	1
#define SSP_IFX_SLOT_WIDTH		32
#define SSP_IFX_SLOT_RX_MASK	0x1
#define SSP_IFX_SLOT_TX_MASK	0x1


#endif /* MFLD_SSP_WL1273_MACHINE_H_ */
