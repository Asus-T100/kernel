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

enum sst_audio_device_type {
	SND_SST_DEVICE_HEADSET = 1,
	SND_SST_DEVICE_IHF,
	SND_SST_DEVICE_VIBRA,
	SND_SST_DEVICE_HAPTIC,
	SND_SST_DEVICE_CAPTURE,
};

enum snd_sst_input_stream {
	SST_INPUT_STREAM_PCM = 0x2,
	SST_INPUT_STREAM_COMPRESS = 0x8,
	SST_INPUT_STREAM_MIXED = 0xA,
};

enum snd_sst_stream_type {
	SST_STREAM_DEVICE_HS = 32,
	SST_STREAM_DEVICE_IHF = 33,
	SST_STREAM_DEVICE_MIC0 = 34,
	SST_STREAM_DEVICE_MIC1 = 35,
};

enum sst_controls {
	SST_SND_ALLOC =			0x1000,
	SST_SND_PAUSE =			0x1001,
	SST_SND_RESUME =		0x1002,
	SST_SND_DROP =			0x1003,
	SST_SND_FREE =			0x1004,
	SST_SND_BUFFER_POINTER =	0x1005,
	SST_SND_STREAM_INIT =		0x1006,
	SST_SND_START	 =		0x1007,
	SST_SND_STREAM_PROCESS =	0x1008,
	SST_CONTROL_BASE =		0x1009,
	SST_VMIC_CHANNEL_SELECT =	0x1010,
	SST_SET_RUNTIME_PARAMS =	0x1011,
	SST_SET_ALGO_PARAMS =		0x1012,
	SST_MAX_CONTROLS =		0x1012,
};

struct pcm_stream_info {
	int str_id;
	void *mad_substream;
	void (*period_elapsed) (void *mad_substream);
	unsigned long long buffer_ptr;
	unsigned long long pcm_delay;
	int sfreq;
};

enum sst_stream_ops {
	STREAM_OPS_PLAYBACK = 0,	/* Decode */
	STREAM_OPS_CAPTURE,		/* Encode */
	STREAM_OPS_PLAYBACK_DRM,	/* Play Audio/Voice */
	STREAM_OPS_PLAYBACK_ALERT,	/* Play Audio/Voice */
	STREAM_OPS_CAPTURE_VOICE_CALL,	/* CSV Voice recording */
};

/* PCM Parameters */
struct sst_pcm_params {
	u16 codec;	/* codec type */
	u8 num_chan;	/* 1=Mono, 2=Stereo */
	u8 pcm_wd_sz;	/* 16/24 - bit*/
	u32 reserved;	/* Bitrate in bits per second */
	u32 sfreq;	/* Sampling rate in Hz */
	u32 ring_buffer_size;
	u32 period_count;	/* period elapsed in samples*/
	u32 ring_buffer_addr;
};

struct sst_stream_params {
	u32 result;
	u32 stream_id;
	u8 codec;
	u8 ops;
	u8 stream_type;
	u8 device_type;
	struct sst_pcm_params sparams;
};

enum lpe_param_types_mixer {
	SST_ALGO_PARAM_MIXER_STREAM_CFG = 0x801,
};

struct mad_ops_wq {
	int stream_id;
	enum sst_controls control_op;
	struct work_struct wq;
};

struct sst_ops {
	int (*open) (struct sst_stream_params *str_param);
	int (*device_control) (int cmd, void *arg);
	int (*set_generic_params) (enum sst_controls cmd, void *arg);
	int (*close) (unsigned int str_id);
};

struct sst_runtime_stream {
	int     stream_status;
	struct pcm_stream_info stream_info;
	struct sst_ops *ops;
	spinlock_t	status_lock;
};

struct sst_device {
	char *name;
	struct device *dev;
	struct sst_ops *ops;
};

int sst_register_dsp(struct sst_device *sst);
int sst_unregister_dsp(struct sst_device *sst);
/* FIXME: remove once vibra becomes PCI driver */
void intel_sst_pwm_suspend(unsigned int suspend);
#endif
