#ifndef __INTEL_SST_IOCTL_H__
#define __INTEL_SST_IOCTL_H__
/*
 *  intel_sst_ioctl.h - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corporation
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com>
 *		KP Jeeja <jeeja.kp@intel.com>
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
 *  This file defines all sst ioctls
 */

/* codec and post/pre processing related info */

#include <linux/types.h>

enum sst_codec_types {
	/*  AUDIO/MUSIC	CODEC Type Definitions */
	SST_CODEC_TYPE_UNKNOWN = 0,
	SST_CODEC_TYPE_PCM,	/* Pass through Audio codec */
	SST_CODEC_TYPE_MP3,
	SST_CODEC_TYPE_MP24,
	SST_CODEC_TYPE_AAC,
	SST_CODEC_TYPE_AACP,
	SST_CODEC_TYPE_eAACP,
	SST_CODEC_TYPE_WMA9,
	SST_CODEC_TYPE_WMA10,
	SST_CODEC_TYPE_WMA10P,
	SST_CODEC_TYPE_RA,
	SST_CODEC_TYPE_DDAC3,
	SST_CODEC_TYPE_STEREO_TRUE_HD,
	SST_CODEC_TYPE_STEREO_HD_PLUS,

	/*  VOICE CODEC Type Definitions */
	SST_CODEC_TYPE_VOICE_PCM = 0x21, /* Pass through voice codec */
};

enum sst_algo_types {
	SST_CODEC_SRC = 0x64,
	SST_CODEC_MIXER = 0x65,
	SST_CODEC_DOWN_MIXER = 0x66,
	SST_CODEC_VOLUME_CONTROL = 0x67,
	SST_CODEC_OEM1 = 0xC8,
	SST_CODEC_OEM2 = 0xC9,
};

enum stream_mode {
	SST_STREAM_MODE_NONE = 0,
	SST_STREAM_MODE_DNR = 1,
	SST_STREAM_MODE_FNF = 2,
	SST_STREAM_MODE_CAPTURE = 3
};

enum stream_type {
	SST_STREAM_TYPE_NONE = 0,
	SST_STREAM_TYPE_MUSIC = 1,
	SST_STREAM_TYPE_NORMAL = 2,
	SST_STREAM_TYPE_LONG_PB = 3,
	SST_STREAM_TYPE_LOW_LATENCY = 4,
};

/* Firmware Version info */
struct snd_sst_fw_version {
	__u8 build;	/* build number*/
	__u8 minor;	/* minor number*/
	__u8 major;	/* major number*/
	__u8 type;	/* build type */
};

/* Port info structure */
struct snd_sst_port_info {
	__u16 port_type;
	__u16 reserved;
};

/* Mixer info structure */
struct snd_sst_mix_info {
	__u16 max_streams;
	__u16 reserved;
};

struct snd_pcm_params {
	__u16 codec;	/* codec type */
	__u8 num_chan;	/* 1=Mono, 2=Stereo */
	__u8 pcm_wd_sz;	/* 16/24 - bit*/
	__u32 reserved;	/* Bitrate in bits per second */
	__u32 sfreq;	/* Sampling rate in Hz */
	__u8 use_offload_path;	/* 0-PCM using period elpased & ALSA interfaces
				   1-PCM stream via compressed interface  */
	__u8 reserved2;
	__u16 reserved3;
	__u8 channel_map[8];
} __packed;

/* PCM Parameters */
struct snd_pcm_params_mfld {
	__u16 codec;	/* codec type */
	__u8 num_chan;	/* 1=Mono, 2=Stereo */
	__u8 pcm_wd_sz;	/* 16/24 - bit*/
	__u32 reserved;	/* Bitrate in bits per second */
	__u32 sfreq;	/* Sampling rate in Hz */
	__u32 ring_buffer_size;
	__u32 period_count;	/* period elapsed in samples*/
	__u32 ring_buffer_addr;
};

/* MP3 Music Parameters Message */
struct snd_mp3_params {
	__u16 codec;
	__u8  num_chan;	/* 1=Mono, 2=Stereo	*/
	__u8  pcm_wd_sz; /* 16/24 - bit*/
	__u8  crc_check; /* crc_check - disable (0) or enable (1) */
	__u8  reserved1; /* unused*/
	__u16 reserved2;	/* Unused */
};

#define AAC_BIT_STREAM_ADTS		0
#define AAC_BIT_STREAM_ADIF		1
#define AAC_BIT_STREAM_RAW		2

/* AAC Music Parameters Message */
struct snd_aac_params {
	__u16 codec;
	__u8 num_chan; /* 1=Mono, 2=Stereo*/
	__u8 pcm_wd_sz; /* 16/24 - bit*/
	__u8 bdownsample; /*SBR downsampling 0 - disable 1 -enabled AAC+ only */
	__u8 bs_format; /* input bit stream format adts=0, adif=1, raw=2 */
	__u16  reser2;
	__u32 externalsr; /*sampling rate of basic AAC raw bit stream*/
	__u8 sbr_signalling;/*disable/enable/set automode the SBR tool.AAC+*/
	__u8 reser1;
	__u16  reser3;
};

/* WMA Music Parameters Message */
struct snd_wma_params {
	__u16 codec;
	__u8  num_chan;	/* 1=Mono, 2=Stereo */
	__u8  pcm_wd_sz;	/* 16/24 - bit*/
	__u32 brate;	/* Use the hard coded value. */
	__u32 sfreq;	/* Sampling freq eg. 8000, 441000, 48000 */
	__u32 channel_mask;  /* Channel Mask */
	__u16 format_tag;	/* Format Tag */
	__u16 block_align;	/* packet size */
	__u16 wma_encode_opt;/* Encoder option */
	__u8 op_align;	/* op align 0- 16 bit, 1- MSB, 2 LSB */
	__u8 reserved;	/* reserved */
};

/* Pre processing param structure */
struct snd_prp_params {
	__u32 reserved;	/* No pre-processing defined yet */
};

/* Pre and post processing params structure */
struct snd_ppp_params {
	__u8			algo_id;/* Post/Pre processing algorithm ID  */
	__u8			str_id;	/*Only 5 bits used 0 - 31 are valid*/
	__u8			enable;	/* 0= disable, 1= enable*/
	__u8			reserved;
	__u32			size;	/*Size of parameters for all blocks*/
	void			*params;
} __packed;

struct snd_sst_postproc_info {
	__u32 src_min;		/* Supported SRC Min sampling freq */
	__u32 src_max;		/* Supported SRC Max sampling freq */
	__u8  src;		/* 0=Not supported, 1=Supported */
	__u8  bass_boost;		/* 0=Not Supported, 1=Supported */
	__u8  stereo_widening;	/* 0=Not Supported, 1=Supported */
	__u8  volume_control;	/* 0=Not Supported, 1=Supported */
	__s16 min_vol;		/* Minimum value of Volume in dB */
	__s16 max_vol;		/* Maximum value of Volume in dB */
	__u8 mute_control;	/* 0=No Mute, 1=Mute */
	__u8 reserved1;
	__u16 reserved2;
};

/* pre processing Capability info structure */
struct snd_sst_prp_info {
	__s16 min_vol;			/* Minimum value of Volume in dB */
	__s16 max_vol;			/* Maximum value of Volume in dB */
	__u8 volume_control;		/* 0=Not Supported, 1=Supported */
	__u8 reserved1;			/* for 32 bit alignment */
	__u16 reserved2;		/* for 32 bit alignment */
} __packed;

/*Pre / Post processing algorithms support*/
struct snd_sst_ppp_info {
	__u32 src:1;		/* 0=Not supported, 1=Supported */
	__u32 mixer:1;		/* 0=Not supported, 1=Supported */
	__u32 volume_control:1;	/* 0=Not Supported, 1=Supported */
	__u32 mute_control:1;	/* 0=Not Supported, 1=Supported */
	__u32 anc:1;		/* 0=Not Supported, 1=Supported */
	__u32 side_tone:1;	/* 0=Not Supported, 1=Supported */
	__u32 dc_removal:1;	/* 0=Not Supported, 1=Supported */
	__u32 equalizer:1;	/* 0=Not Supported, 1=Supported */
	__u32 spkr_prot:1;	/* 0=Not Supported, 1=Supported */
	__u32 bass_boost:1;	/* 0=Not Supported, 1=Supported */
	__u32 stereo_widening:1;/* 0=Not Supported, 1=Supported */
	__u32 rsvd1:21;
	__u32 rsvd2;
};

/* Firmware capabilities info */
struct snd_sst_fw_info {
	struct snd_sst_fw_version fw_version; /* Firmware version */
	__u8 audio_codecs_supported[8];	/* Codecs supported by FW */
	__u32 recommend_min_duration; /* Min duration for Lowpower Playback */
	__u8 max_pcm_streams_supported; /* Max num of PCM streams supported */
	__u8 max_enc_streams_supported;	/* Max number of Encoded streams  */
	__u16 reserved;		/* 32 bit alignment*/
	struct snd_sst_ppp_info ppp_info; /* pre_processing mod cap info */
	struct snd_sst_postproc_info pop_info; /* Post processing cap info*/
	struct snd_sst_port_info port_info[3]; /* Port info */
	struct snd_sst_mix_info mix_info;/* Mixer info */
	__u32 min_input_buf; /* minmum i/p buffer for decode */
};

/* Codec params struture */
union  snd_sst_codec_params {
	struct snd_pcm_params pcm_params;
	struct snd_mp3_params mp3_params;
	struct snd_aac_params aac_params;
	struct snd_wma_params wma_params;
};


/* Address and size info of a frame buffer in DDR */
struct sst_address_info {
	__u32 addr; /* Address at IA */
	__u32 size; /* Size of the buffer */
} __packed;

/* Additional params for Alloc struct*/
struct snd_sst_alloc_params_ext {
	struct sst_address_info  ring_buf_info[8];
	__u8 sg_count;
	__u8 reserved;
	__u16 reserved2;
	__u32 frag_size;	/*Number of samples after which period elapsed
				  message is sent valid only if path  = 0*/
};


struct snd_sst_stream_params {
	union snd_sst_codec_params uc;
} __packed;

struct snd_sst_params {
	__u32 result;
	__u32 stream_id;
	__u8 codec;
	__u8 ops;
	__u8 stream_type;
	__u8 device_type;
	struct snd_sst_stream_params sparams;
	struct snd_sst_alloc_params_ext aparams;
};

struct snd_sst_vol {
	__u32	stream_id;
	__s32	volume;
	__u32	ramp_duration;
	__u32	ramp_type;		/* Ramp type, default=0 */
};

struct snd_sst_mute {
	__u32	stream_id;
	__u32	mute;
};

/* ioctl related stuff here */
struct snd_sst_pmic_config {
	__u32  sfreq;                /* Sampling rate in Hz */
	__u16  num_chan;             /* Mono =1 or Stereo =2 */
	__u16  pcm_wd_sz;            /* Number of bits per sample */
} __packed;

struct snd_sst_get_stream_params {
	struct snd_sst_params codec_params;
	struct snd_sst_pmic_config pcm_params;
};

enum snd_sst_target_type {
	SND_SST_TARGET_PMIC = 1,
	SND_SST_TARGET_LPE,
	SND_SST_TARGET_MODEM,
	SND_SST_TARGET_BT,
	SND_SST_TARGET_FM,
	SND_SST_TARGET_NONE,
};

enum snd_sst_device_type {
	SND_SST_DEVICE_SSP = 1,
	SND_SST_DEVICE_PCM,
	SND_SST_DEVICE_OTHER,
};

enum snd_sst_device_mode {
	SND_SST_DEV_MODE_PCM_MODE1 = 1, /*(16-bit word, bit-length frame sync)*/
	SND_SST_DEV_MODE_PCM_MODE2,
	SND_SST_DEV_MODE_PCM_MODE3,
	SND_SST_DEV_MODE_PCM_MODE4_RIGHT_JUSTIFIED,
	SND_SST_DEV_MODE_PCM_MODE4_LEFT_JUSTIFIED,
	SND_SST_DEV_MODE_PCM_MODE4_I2S, /*(I2S mode, 16-bit words)*/
	SND_SST_DEV_MODE_PCM_MODE5,
	SND_SST_DEV_MODE_PCM_MODE6,
};

enum snd_sst_port_action {
	SND_SST_PORT_PREPARE = 1,
	SND_SST_PORT_ACTIVATE,
};

enum stream_param_type {
	SST_SET_TIME_SLOT = 0,
	SST_SET_CHANNEL_INFO = 1,
	OTHERS = 2, /*reserved for future params*/
};

/* Target selection per device structure */
struct snd_sst_slot_info {
	__u8 mix_enable;	/* Mixer enable or disable */
	__u8 device_type;
	__u8 device_instance;	/* 0, 1, 2 */
	__u8 target_device;
	__u16 target_sink;
	__u8 slot[2];
	__u8 master;
	__u8 action;
	__u8 device_mode;
	__u8 reserved;
	struct snd_sst_pmic_config pcm_params;
} __packed;

#define SST_MAX_TARGET_DEVICES 3
/* Target device list structure */
struct snd_sst_target_device  {
	__u32 device_route;
	struct snd_sst_slot_info devices[SST_MAX_TARGET_DEVICES];
} __packed;

struct snd_sst_driver_info {
	__u32 version;	/* Version of the driver */
	__u32 active_pcm_streams;
	__u32 active_enc_streams;
	__u32 max_pcm_streams;
	__u32 max_enc_streams;
	__u32 buf_per_stream;
};

enum snd_sst_buff_type {
	SST_BUF_USER = 1,
	SST_BUF_MMAP,
	SST_BUF_RAR,
};

struct snd_sst_mmap_buff_entry {
	unsigned int offset;
	unsigned int size;
};

struct snd_sst_mmap_buffs {
	unsigned int entries;
	enum snd_sst_buff_type type;
	struct snd_sst_mmap_buff_entry *buff;
};

struct snd_sst_buff_entry {
	void *buffer;
	unsigned int size;
};

struct snd_sst_buffs {
	unsigned int entries;
	__u8 type;
	struct snd_sst_buff_entry *buff_entry;
};

struct snd_sst_dbufs  {
	unsigned long long input_bytes_consumed;
	unsigned long long output_bytes_produced;
	struct snd_sst_buffs *ibufs;
	struct snd_sst_buffs *obufs;
};

struct snd_sst_tuning_params {
	__u8 type;
	__u8 str_id;
	__u8 size;
	__u8 rsvd;
	__u64 addr;
} __packed;

struct snd_sst_runtime_params {
	__u8 type;
	__u8 str_id;
	__u8 size;
	__u8 rsvd;
	void *addr;
} __packed;

/*IOCTL defined here */
/*SST common ioctls */
#define SNDRV_SST_DRIVER_INFO	_IOR('L', 0x10, struct snd_sst_driver_info *)

/*AM Ioctly only */
#define SNDRV_SST_FW_INFO	_IOR('L', 0x20,  struct snd_sst_fw_info *)

/*DSP Ioctls on /dev/intel_sst_ctrl only*/
#define SNDRV_SST_SET_ALGO	_IOW('L', 0x30,  struct snd_ppp_params *)
#define SNDRV_SST_GET_ALGO	_IOWR('L', 0x31,  struct snd_ppp_params *)
#define SNDRV_SST_TUNING_PARAMS	_IOW('L', 0x32,  struct snd_sst_tuning_params *)
#define SNDRV_SST_SET_RUNTIME_PARAMS _IOW \
			('L', 0x40, struct snd_sst_tuning_params *)

#endif /* __INTEL_SST_IOCTL_H__ */
