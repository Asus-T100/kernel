/*
 *  sst_platform.h - Intel MID Platform driver header file
 *
 *  Copyright (C) 2010 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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
 *
 *
 */

#ifndef __SST_PLATFORMDRV_H__
#define __SST_PLATFORMDRV_H__

#define SST_MONO		1
#define SST_STEREO		2


#define SST_MIN_RATE		8000
#define SST_MAX_RATE		48000
#define SST_MIN_CHANNEL		1
#define SST_MAX_CHANNEL		2

/**FIXME ***/
#ifdef CONFIG_SND_MFLD_MACHINE
#define SST_MAX_BUFFER		88200 /*500ms*/
#define SST_MIN_PERIOD_BYTES	1764  /*10ms@44.1,16bit,2ch*/
#define SST_MAX_PERIOD_BYTES	44100 /*250ms*/
#else
#define SST_MAX_BUFFER		96000 /*500ms*/
#define SST_MIN_PERIOD_BYTES	1920  /*10ms@44.1,16bit,2ch*/
#define SST_MAX_PERIOD_BYTES	48000 /*250ms*/
#endif

#define SST_MIN_PERIODS		2
#define SST_MAX_PERIODS		50
#define SST_FIFO_SIZE		0
#define SST_CARD_NAMES		"intel_mid_card"
#define MSIC_VENDOR_ID		3
#define SST_CLK_UNINIT		0x03

struct sst_runtime_stream {
	int     stream_status;
	struct pcm_stream_info stream_info;
	struct intel_sst_card_ops *sstdrv_ops;
	spinlock_t	status_lock;
};

enum sst_drv_status {
	SST_PLATFORM_INIT = 1,
	SST_PLATFORM_STARTED,
	SST_PLATFORM_RUNNING,
	SST_PLATFORM_PAUSED,
	SST_PLATFORM_DROPPED,
	SST_PLATFORM_SUSPENDED,
};

struct sst_platform_ctx {
	int active_nonvoice_cnt;
	int active_voice_cnt;
};
#endif
