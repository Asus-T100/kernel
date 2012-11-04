#ifndef __INTEL_SST_FW_IPC_H__
#define __INTEL_SST_FW_IPC_H__
/*
*  intel_sst_fw_ipc.h - Intel SST Driver for audio engine
*
*  Copyright (C) 2008-10 Intel Corporation
*  Author:	Vinod Koul <vinod.koul@intel.com>
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
*  This driver exposes the audio engine functionalities to the ALSA
*	and middleware.
*  This file has definitions shared between the firmware and driver
*/

#define MAX_NUM_STREAMS_MRST 3
#define MAX_NUM_STREAMS_MFLD 5
#define MAX_NUM_STREAMS_MRFLD 23
#define MAX_NUM_STREAMS MAX_NUM_STREAMS_MRFLD
#define MAX_DBG_RW_BYTES 80
#define MAX_NUM_SCATTER_BUFFERS 8
#define MAX_LOOP_BACK_DWORDS 8
/* IPC base address and mailbox, timestamp offsets */
#define SST_MAILBOX_SIZE 0x0400
#define SST_MAILBOX_SEND 0x0000
#define SST_MAILBOX_RCV 0x0800
#define SST_TIME_STAMP 0x1800
#define SST_TIME_STAMP_MRFLD 0x800
#define SST_RESERVED_OFFSET 0x1A00
#define SST_CHEKPOINT_OFFSET 0x1C00
#define REPLY_MSG 0x80

/* Message ID's for IPC messages */
/* Bits B7: SST or IA/SC ; B6-B4: Msg Category; B3-B0: Msg Type */

/* I2L Firmware/Codec Download msgs */
#define IPC_IA_PREP_LIB_DNLD 0x01
#define IPC_IA_LIB_DNLD_CMPLT 0x02
#define IPC_IA_GET_FW_VERSION 0x04
#define IPC_IA_GET_FW_BUILD_INF 0x05
#define IPC_IA_GET_FW_INFO 0x06
#define IPC_IA_GET_FW_CTXT 0x07
#define IPC_IA_SET_FW_CTXT 0x08

/* I2L Codec Config/control msgs */
#define IPC_IA_SET_CODEC_PARAMS 0x10
#define IPC_IA_GET_CODEC_PARAMS 0x11
#define IPC_IA_SET_PPP_PARAMS 0x12
#define IPC_IA_GET_PPP_PARAMS 0x13
#define IPC_SST_PERIOD_ELAPSED_MRFLD 10
#define IPC_IA_ALG_PARAMS 0x1A
#define IPC_IA_TUNING_PARAMS 0x1B
#define IPC_IA_SET_RUNTIME_PARAMS 0x1C
#define IPC_IA_SET_PARAMS 0x1
#define IPC_IA_GET_PARAMS 0x2

/* I2L Stream config/control msgs */
#define IPC_IA_ALLOC_STREAM_MRFLD 0x2
#define IPC_IA_ALLOC_STREAM 0x20 /* Allocate a stream ID */
#define IPC_IA_FREE_STREAM_MRFLD 0x03
#define IPC_IA_FREE_STREAM 0x21 /* Free the stream ID */
#define IPC_IA_SET_STREAM_PARAMS 0x22
#define IPC_IA_GET_STREAM_PARAMS 0x23
#define IPC_IA_PAUSE_STREAM 0x24
#define IPC_IA_PAUSE_STREAM_MRFLD 0x24
#define IPC_IA_RESUME_STREAM 0x25
#define IPC_IA_RESUME_STREAM_MRFLD 0x25
#define IPC_IA_DROP_STREAM 0x26
#define IPC_IA_DROP_STREAM_MRFLD 0x07
#define IPC_IA_DRAIN_STREAM 0x27 /* Short msg with str_id */
#define IPC_IA_CONTROL_ROUTING 0x29

#define IPC_IA_START_STREAM_MRFLD 0X06
#define IPC_IA_START_STREAM 0x30 /* Short msg with str_id */

/* Debug msgs */
#define IPC_IA_DBG_MEM_READ 0x40
#define IPC_IA_DBG_MEM_WRITE 0x41
#define IPC_IA_DBG_LOOP_BACK 0x42

/* L2I Firmware/Codec Download msgs */
#define IPC_IA_FW_INIT_CMPLT 0x81
#define IPC_IA_FW_INIT_CMPLT_MRFLD 0x01

/* L2I Codec Config/control msgs */
#define IPC_SST_FRAGMENT_ELPASED 0x90 /* Request IA more data */

#define IPC_SST_BUF_UNDER_RUN 0x92 /* PB Under run and stopped */
#define IPC_SST_BUF_OVER_RUN 0x93 /* CAP Under run and stopped */
#define IPC_SST_DRAIN_END 0x94 /* PB Drain complete and stopped */
#define IPC_SST_CHNGE_SSP_PARAMS 0x95 /* PB SSP parameters changed */
#define IPC_SST_STREAM_PROCESS_FATAL_ERR 0x96/* error in processing a stream */
#define IPC_SST_PERIOD_ELAPSED 0x97 /* period elapsed */

#define IPC_SST_ERROR_EVENT 0x99 /* Buffer over run occurred */
/* L2S messages */
#define IPC_SC_DDR_LINK_UP 0xC0
#define IPC_SC_DDR_LINK_DOWN 0xC1
#define IPC_SC_SET_LPECLK_REQ 0xC2
#define IPC_SC_SSP_BIT_BANG 0xC3

/* L2I Error reporting msgs */
#define IPC_IA_MEM_ALLOC_FAIL 0xE0
#define IPC_IA_PROC_ERR 0xE1 /* error in processing a
					stream can be used by playback and
					capture modules */

/* L2I Debug msgs */
#define IPC_IA_PRINT_STRING 0xF0

/*TODO*/
#define IPC_QUE_ID_MED 0x03
#define SST_UNSOLICITED_ERROR_MSG 0xFFFF0000
#define SST_UNSOLICITED_MSG_ID 0x0000FFFF

/* Command Response or Acknowledge message to any IPC message will have
 * same message ID and stream ID information which is sent.
 * There is no specific Ack message ID. The data field is used as response
 * meaning.
 */
enum ackData {
	IPC_ACK_SUCCESS = 0,
	IPC_ACK_FAILURE
};
enum ipc_ia_msg_id {
	IPC_CMD = 1,		/*!< Task Control message ID */
	IPC_SET_PARAMS = 2,/*!< Task Set param message ID */
	IPC_GET_PARAMS = 3,	/*!< Task Get param message ID */
	IPC_INVALID = 0xFF	/*!<Task Get param message ID */
};

/* Pipe IDs ref: LPE DSP command interface spec v0.75 */
enum lpe_pipe_id {
/* Output pipeline IDs */
	PIPE_ID_OUT_START = 0x0,
	PIPE_MODEM_OUT = 0x0,
	PIPE_BT_OUT = 0x1,
	PIPE_CODEC_OUT0 = 0x2,
	PIPE_CODEC_OUT1 = 0x3,
	PIPE_SPROT_LOOP_OUT = 0x4,
	PIPE_MEDIA_LOOP1_OUT = 0x5,
	PIPE_MEDIA_LOOP2_OUT = 0x6,
	PIPE_PROBE_OUT = 0x7,
	PIPE_HF_SNS_OUT = 0x8, /* VOCIE_UPLINK_REF2 */
	PIPE_HF_OUT = 0x9, /* VOICE_UPLINK_REF1 */
	PIPE_SPEECH_OUT = 0xA, /* VOICE UPLINK */
	PIPE_RxSPEECH_OUT = 0xB, /* VOICE_DOWNLINK */
	PIPE_VOIP_OUT = 0xC,
	PIPE_PCM0_OUT = 0xD,
	PIPE_PCM1_OUT = 0xE,
	PIPE_PCM2_OUT = 0xF,
	PIPE_AWARE_OUT = 0x10,
	PIPE_VAD_OUT = 0x11,
	PIPE_MEDIA0_OUT = 0x12,
	PIPE_MEDIA1_OUT = 0x12,
	PIPE_FM_OUT = 0x14,
	PIPE_PROBE1_OUT = 0x15,
	PIPE_PROBE2_OUT = 0x16,
	PIPE_PROBE3_OUT = 0x17,
	PIPE_PROBE4_OUT = 0x18,
	PIPE_PROBE5_OUT = 0x19,
	PIPE_PROBE6_OUT = 0x1A,
	PIPE_PROBE7_OUT = 0x1B,
	PIPE_PROBE8_OUT = 0x1C,
/* Input Pipeline IDs */
	PIPE_ID_IN_START = 0x80,
	PIPE_MODEM_IN = 0x80,
	PIPE_BT_IN = 0x81,
	PIPE_CODEC_IN0 = 0x82,
	PIPE_CODEC_IN1 = 0x83,
	PIPE_SPROT_LOOP_IN = 0x84,
	PIPE_MEDIA_LOOP1_IN = 0x85,
	PIPE_MEDIA_LOOP2_IN = 0x86,
	PIPE_PROBE_IN = 0x87,
	PIPE_SIDETONE_IN = 0x88,
	PIPE_TxSPEECH_IN = 0x89,
	PIPE_SPEECH_IN = 0x8A,
	PIPE_TONE_IN = 0x8B,
	PIPE_VOIP_IN = 0x8C,
	PIPE_PCM0_IN = 0x8D,
	PIPE_PCM1_IN = 0x8E,
	PIPE_MEDIA0_IN = 0x8F,
	PIPE_MEDIA1_IN = 0x90,
	PIPE_MEDIA2_IN = 0x91,
	PIPE_FM_IN = 0x92,
	PIPE_PROBE1_IN = 0x93,
	PIPE_PROBE2_IN = 0x94,
	PIPE_PROBE3_IN = 0x95,
	PIPE_PROBE4_IN = 0x96,
	PIPE_PROBE5_IN = 0x97,
	PIPE_PROBE6_IN = 0x98,
	PIPE_PROBE7_IN = 0x99,
	PIPE_PROBE8_IN = 0x9A,
	PIPE_RSVD = 0xFF,
};

enum sst_error_codes {
	/* Error code,response to msgId: Description */
	/* Common error codes */
	SST_SUCCESS = 0,	/* Success */
	SST_ERR_INVALID_STREAM_ID = 1,
	SST_ERR_INVALID_MSG_ID = 2,
	SST_ERR_INVALID_STREAM_OP = 3,
	SST_ERR_INVALID_PARAMS = 4,
	SST_ERR_INVALID_CODEC = 5,
	SST_ERR_INVALID_MEDIA_TYPE = 6,
	SST_ERR_STREAM_ERR = 7,

	/* IPC specific error codes */
	SST_IPC_ERR_CALL_BACK_NOT_REGD = 8,
	SST_IPC_ERR_STREAM_NOT_ALLOCATED = 9,
	SST_IPC_ERR_STREAM_ALLOC_FAILED = 10,
	SST_IPC_ERR_GET_STREAM_FAILED = 11,
	SST_ERR_MOD_NOT_AVAIL = 12,
	SST_ERR_MOD_DNLD_RQD = 13,
	SST_ERR_STREAM_STOPPED = 14,
	SST_ERR_STREAM_IN_USE = 15,

	/* Capture specific error codes */
	SST_CAP_ERR_INCMPLTE_CAPTURE_MSG = 16,
	SST_CAP_ERR_CAPTURE_FAIL = 17,
	SST_CAP_ERR_GET_DDR_NEW_SGLIST = 18,
	SST_CAP_ERR_UNDER_RUN = 19,
	SST_CAP_ERR_OVERFLOW = 20,

	/* Playback specific error codes*/
	SST_PB_ERR_INCMPLTE_PLAY_MSG = 21,
	SST_PB_ERR_PLAY_FAIL = 22,
	SST_PB_ERR_GET_DDR_NEW_SGLIST = 23,

	/* Codec manager specific error codes */
	SST_LIB_ERR_LIB_DNLD_REQUIRED = 24,
	SST_LIB_ERR_LIB_NOT_SUPPORTED = 25,

	/* Library manager specific error codes */
	SST_SCC_ERR_PREP_DNLD_FAILED = 26,
	SST_SCC_ERR_LIB_DNLD_RES_FAILED = 27,
	/* Scheduler specific error codes */
	SST_SCH_ERR_FAIL = 28,

	/* DMA specific error codes */
	SST_DMA_ERR_NO_CHNL_AVAILABLE = 29,
	SST_DMA_ERR_INVALID_INPUT_PARAMS = 30,
	SST_DMA_ERR_CHNL_ALREADY_SUSPENDED = 31,
	SST_DMA_ERR_CHNL_ALREADY_STARTED = 32,
	SST_DMA_ERR_CHNL_NOT_ENABLED = 33,
	SST_DMA_ERR_TRANSFER_FAILED = 34,

	SST_SSP_ERR_ALREADY_ENABLED = 35,
	SST_SSP_ERR_ALREADY_DISABLED = 36,
	SST_SSP_ERR_NOT_INITIALIZED = 37,
	SST_SSP_ERR_SRAM_NO_DMA_DATA = 38,

	/* Other error codes */
	SST_ERR_MOD_INIT_FAIL = 39,

	/* FW init error codes */
	SST_RDR_ERR_IO_DEV_SEL_NOT_ALLOWED = 40,
	SST_RDR_ERR_ROUTE_ALREADY_STARTED = 41,
	SST_RDR_ERR_IO_DEV_SEL_FAILED = 42,
	SST_RDR_PREP_CODEC_DNLD_FAILED = 43,

	/* Memory debug error codes */
	SST_ERR_DBG_MEM_READ_FAIL = 44,
	SST_ERR_DBG_MEM_WRITE_FAIL = 45,
	SST_ERR_INSUFFICIENT_INPUT_SG_LIST = 46,
	SST_ERR_INSUFFICIENT_OUTPUT_SG_LIST = 47,

	SST_ERR_BUFFER_NOT_AVAILABLE = 48,
	SST_ERR_BUFFER_NOT_ALLOCATED = 49,
	SST_ERR_INVALID_REGION_TYPE = 50,
	SST_ERR_NULL_PTR = 51,
	SST_ERR_INVALID_BUFFER_SIZE = 52,
	SST_ERR_INVALID_BUFFER_INDEX = 53,

	/*IIPC specific error codes */
	SST_IIPC_QUEUE_FULL = 54,
	SST_IIPC_ERR_MSG_SND_FAILED = 55,
	SST_PB_ERR_UNDERRUN_OCCURED = 56,
	SST_RDR_INSUFFICIENT_MIXER_BUFFER = 57,
	SST_INVALID_TIME_SLOTS = 58,
};

enum dbg_mem_data_type {
	/* Data type of debug read/write */
	DATA_TYPE_U32,
	DATA_TYPE_U16,
	DATA_TYPE_U8,
};

struct ipc_dsp_hdr {
	u16 mod_index_id:8;		/*!< DSP Command ID specific to tasks */
	u16 pipe_id:8;	/*!< instance of the module in the pipeline */
	u16 mod_id;		/*!< Pipe_id */
	u16 cmd_id;		/*!< Module ID = lpe_algo_types_t */
	u16 length;		/*!< Length of the payload only */
} __packed;

union ipc_header_high {
	struct {
		u32  msg_id:8;	    /* Message ID - Max 256 Message Types */
		u32  task_id:4;	    /* Task ID associated with this comand */
		u32  str_id:4;    /* Identifier for the driver to track*/
		u32  rsvd1:8;	    /* Reserved */
		u32  result:4;	    /* Reserved */
		u32  res_rqd:1;	    /* Response rqd */
		u32  large:1;	    /* Large Message if large = 1 */
		u32  done:1;	    /* bit 30 - Done bit */
		u32  busy:1;	    /* bit 31 - busy bit*/
	} part;
	u32 full;
} __packed;

/* IPC header */
union ipc_header_mrfld {
	struct {
		u32 header_low_payload;
		union ipc_header_high header_high;
	} p;
	u64 full;
} __packed;

/* CAUTION NOTE: All IPC message body must be multiple of 32 bits.*/

/* IPC Header */
union ipc_header {
	struct {
		u32  msg_id:8; /* Message ID - Max 256 Message Types */
		u32  str_id:5;
		u32  large:1;	/* Large Message if large = 1 */
		u32  reserved:2;	/* Reserved for future use */
		u32  data:14;	/* Ack/Info for msg, size of msg in Mailbox */
		u32  done:1; /* bit 30 */
		u32  busy:1; /* bit 31 */
	} part;
	u32 full;
} __packed;

/* Firmware build info */
struct sst_fw_build_info {
	unsigned char  date[16]; /* Firmware build date */
	unsigned char  time[16]; /* Firmware build time */
} __packed;

struct ipc_header_fw_init {
	struct snd_sst_fw_version fw_version;/* Firmware version details */
	struct sst_fw_build_info build_info;
	u16 result;	/* Fw init result */
	u8 module_id; /* Module ID in case of error */
	u8 debug_info; /* Debug info from Module ID in case of fail */
} __packed;


/* Time stamp */
struct snd_sst_tstamp_mfld {
	u64 samples_processed;/* capture - data in DDR */
	u64 samples_rendered;/* playback - data rendered */
	u64 bytes_processed;/* bytes decoded or encoded */
	u32 sampling_frequency;/* eg: 48000, 44100 */
	u32 reserved1;
	u16 reserved2;
	u16 reserved;/* 32 bit alignment */
	u64 pcm_delay;
};

struct snd_sst_tstamp {
	u32 ring_buffer_counter;	/* PB/CP: Bytes copied from/to DDR. */
	u32 hardware_counter;	    /* PB/CP: Bytes DMAed to/from SSP. */
	u32 frames_decoded;
	u32 bytes_decoded;
	u32 bytes_copied;
	u32 sampling_frequency;
	u32 channel_peak[8];
};


/* SST to IA memory read debug message  */
struct ipc_sst_ia_dbg_mem_rw  {
	u16  num_bytes;/* Maximum of MAX_DBG_RW_BYTES */
	u16  data_type;/* enum: dbg_mem_data_type */
	u32  address;	/* Memory address of data memory of data_type */
	u8	rw_bytes[MAX_DBG_RW_BYTES];/* Maximum of 64 bytes can be RW */
} __packed;

struct ipc_sst_ia_dbg_loop_back {
	u16 num_dwords; /* Maximum of MAX_DBG_RW_BYTES */
	u16 increment_val;/* Increments dwords by this value, 0- no increment */
	u32 lpbk_dwords[MAX_LOOP_BACK_DWORDS];/* Maximum of 8 dwords loopback */
} __packed;

/* Stream type params struture for Alloc stream */
struct snd_sst_str_type {
	u8 codec_type;		/* Codec type */
	u8 str_type;		/* 1 = voice 2 = music */
	u8 operation;		/* Playback or Capture */
	u8 protected_str;	/* 0=Non DRM, 1=DRM */
	u8 time_slots;
	u8 reserved;		/* Reserved */
	u16 result;		/* Result used for acknowledgment */
} __packed;

/* Library info structure */
struct module_info {
	u32 lib_version;
	u32 lib_type;/*TBD- KLOCKWORK u8 lib_type;*/
	u32 media_type;
	u8  lib_name[12];
	u32 lib_caps;
	unsigned char  b_date[16]; /* Lib build date */
	unsigned char  b_time[16]; /* Lib build time */
} __packed;

/* Library slot info */
struct lib_slot_info {
	u8  slot_num; /* 1 or 2 */
	u8  reserved1;
	u16 reserved2;
	u32 iram_size; /* slot size in IRAM */
	u32 dram_size; /* slot size in DRAM */
	u32 iram_offset; /* starting offset of slot in IRAM */
	u32 dram_offset; /* starting offset of slot in DRAM */
} __packed;

struct snd_ppp_mixer_params {
	__u32			type; /*Type of the parameter */
	__u32			size;
	__u32			input_stream_bitmap; /*Input stream Bit Map*/
} __packed;

struct snd_sst_lib_download {
	struct module_info lib_info; /* library info type, capabilities etc */
	struct lib_slot_info slot_info; /* slot info to be downloaded */
	u32 mod_entry_pt;
};

struct snd_sst_lib_download_info {
	struct snd_sst_lib_download dload_lib;
	u16 result;	/* Result used for acknowledgment */
	u8 pvt_id; /* Private ID */
	u8 reserved;  /* for alignment */
};

/* Codec params struture */
union snd_sst_codec_params_mrfld {
	struct snd_pcm_params_mrfld pcm_params;
	struct snd_mp3_params_mrfld mp3_params;
	struct snd_aac_params_mrfld aac_params;
	struct snd_wma_params_mrfld wma_params;
	struct snd_mp3_get_params_mrfld get_mp3_params;
	struct snd_aac_get_params_mrfld get_aac_params;
};



union snd_sst_codec_params_mfld {
	struct snd_pcm_params_mfld pcm_params;
	struct snd_mp3_params mp3_params;
	struct snd_aac_params aac_params;
	struct snd_wma_params wma_params;
};

struct snd_sst_stream_params_mfld {
	union snd_sst_codec_params_mfld uc;
} __packed;

struct snd_sst_stream_params_mrfld {
	union snd_sst_codec_params_mrfld uc;
} __packed;

struct snd_sst_alloc_mrfld {
	u16 codec_type;
	u8 operation;
	u8 sg_count;
	struct sst_address_info ring_buf_info[8];
	u32 frag_size;
	struct snd_sst_tstamp *ts;
	struct snd_sst_stream_params_mrfld codec_params;
} __packed;

/* Alloc stream params structure */
struct snd_sst_alloc_params_mfld {
	struct snd_sst_str_type str_type;
	struct snd_sst_stream_params_mfld stream_params;
};

struct snd_sst_fw_get_stream_params_mfld {
	struct snd_sst_stream_params_mfld codec_params;
	struct snd_sst_pmic_config pcm_params;
};

/* Alloc stream params structure */
struct snd_sst_alloc_params {
	struct snd_sst_str_type str_type;
	struct snd_sst_stream_params stream_params;
	struct snd_sst_alloc_params_ext alloc_params;
} __packed;


struct snd_sst_fw_get_stream_params {
	struct snd_sst_stream_params codec_params;
	struct snd_sst_pmic_config pcm_params;
};

/* Alloc stream response message */
struct snd_sst_alloc_response {
	struct snd_sst_str_type str_type; /* Stream type for allocation */
	struct snd_sst_lib_download lib_dnld; /* Valid only for codec dnld */
};

/* Drop response */
struct snd_sst_drop_response {
	u32 result;
	u32 bytes;
};

/* CSV Voice call routing structure */
struct snd_sst_control_routing {
	u8 control; /* 0=start, 1=Stop */
	u8 reserved[3];	/* Reserved- for 32 bit alignment */
};


struct ipc_post {
	struct list_head node;
	union ipc_header header; /* driver specific */
	union ipc_header_mrfld mrfld_header;
	char *mailbox_data;
};

struct snd_sst_ctxt_params {
	u32 address; /* Physical Address in DDR where the context is stored */
	u32 size; /* size of the context */
};
#endif /* __INTEL_SST_FW_IPC_H__ */
