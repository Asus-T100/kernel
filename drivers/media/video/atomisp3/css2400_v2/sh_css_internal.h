/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
*
* Copyright (c) 2010 Intel Corporation. All Rights Reserved.
*
* Copyright (c) 2010 Silicon Hive www.siliconhive.com.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License version
* 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
* 02110-1301, USA.
*
*/

#ifndef _SH_CSS_INTERNAL_H_
#define _SH_CSS_INTERNAL_H_

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif

#include "input_formatter.h"
#include "input_system.h"

#include "pipeline.h"
#include "csc_kernel.h"

#include "ia_css_types.h"
#include "ia_css_acc_types.h"
//#include "sh_css_internal.h"
#include "sh_css_binary.h"
#include "sh_css_firmware.h"
#include "sh_css_legacy.h"
#include "sh_css_defs.h"
#include "dma.h"	/* N_DMA_CHANNEL_ID */

#define SH_CSS_MAX_BINARY_NAME	32

#define SP_DEBUG_NONE	(0)
#define SP_DEBUG_DUMP	(1)
#define SP_DEBUG_COPY	(2)
#define SP_DEBUG_TRACE	(3) /* not yet functional */
#define SP_DEBUG_MINIMAL (4)

#define SP_DEBUG SP_DEBUG_NONE
#define SP_DEBUG_MINIMAL_OVERWRITE 1


#ifdef __DISABLE_UNUSED_THREAD__
#define SH_CSS_MAX_SP_THREADS	1 /* preview */
#else
#define SH_CSS_MAX_SP_THREADS	4 /* raw_copy, preview, capture, acceleration */
#endif

#define NUM_REF_FRAMES		2

/* keep next up to date with the definition for MAX_CB_ELEMS_FOR_TAGGER in tagger.sp.c */
#if defined(HAS_SP_2400A0)
#define NUM_CONTINUOUS_FRAMES	5
#else
#define NUM_CONTINUOUS_FRAMES	10
#endif

#define NUM_MIPI_FRAMES		8

#define NUM_TNR_FRAMES		2

#define NUM_VIDEO_REF_FRAMES	2
#define NUM_VIDEO_TNR_FRAMES	2
#define NR_OF_PIPELINES			5 /* Must match with IA_CSS_PIPE_ID_NUM */

#define SH_CSS_MAX_IF_CONFIGS	3 /* Must match with IA_CSS_NR_OF_CONFIGS (not defined yet).*/
#define SH_CSS_IF_CONFIG_NOT_NEEDED	0xFF

/* MW: This has to be moved to the "pipeline" file set */
/*#define SH_CSS_MAX_STAGES 6 *//* copy, preisp, anr, postisp, capture_pp, vf_pp */

/* ISP parameter versions
 * If ISP_PIPE_VERSION is defined as 2 in isp_defs_for_hive.h,
 * SH_CSS_ISP_PARAMS_VERSION should be 2.
 * If ISP_PIPE_VERSION is defined as 1,
 * SH_CSS_ISP_PARAMS_VERSION can be either 1 or 2.
 * ISP_PIPE_VERSION is always defined as 1 for ISP2300,
 * so SH_CSS_ISP_PARAMS_VERSION can be defined as 1 for ISP2300
 * to avoid sched error by increase of the number of paramaters.
 *
#if defined(IS_ISP_2400_SYSTEM)
#define SH_CSS_ISP_PARAMS_VERSION	2
#define SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB	1
#else
#define SH_CSS_ISP_PARAMS_VERSION	1
#define SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB	0
#endif
 */

/*
 * JB: keep next enum in sync with thread id's
 * and pipe id's
 */
enum sh_css_pipe_config_override {
	SH_CSS_PIPE_CONFIG_OVRD_NONE     = 0,
	SH_CSS_PIPE_CONFIG_OVRD_NO_OVRD  = 0xffff
};

enum host2sp_commands {
	host2sp_cmd_error = 0,
	/*
	 * The host2sp_cmd_ready command is the only command written by the SP
	 * It acknowledges that is previous command has been received.
	 * (this does not mean that the command has been executed)
	 * It also indicates that a new command can be send (it is a queue
	 * with depth 1).
	 */
	host2sp_cmd_ready = 1,
	/* Command written by the Host */
	host2sp_cmd_dummy,		/* No action, can be used as watchdog */
	host2sp_cmd_terminate,		/* SP should terminate itself */
	N_host2sp_cmd
};

/** Enumeration used to indicate the events that are produced by
 *  the SP and consumed by the Host.
 */
enum sh_css_sp_event_type {
	SH_CSS_SP_EVENT_OUTPUT_FRAME_DONE,
	SH_CSS_SP_EVENT_VF_OUTPUT_FRAME_DONE,
	SH_CSS_SP_EVENT_3A_STATISTICS_DONE,
	SH_CSS_SP_EVENT_DIS_STATISTICS_DONE,
	SH_CSS_SP_EVENT_PIPELINE_DONE,
	SH_CSS_SP_EVENT_PORT_EOF,
	SH_CSS_SP_EVENT_NR_OF_TYPES,		/* must be last */
};

/* xmem address map allocation */
struct sh_css_ddr_address_map {
	hrt_vaddress isp_param;
	hrt_vaddress ctc_tbl;
	hrt_vaddress xnr_tbl;
	hrt_vaddress gamma_tbl;
	hrt_vaddress macc_tbl;
	hrt_vaddress fpn_tbl;
	hrt_vaddress sc_tbl;
	hrt_vaddress sdis_hor_coef;
	hrt_vaddress sdis_ver_coef;
	hrt_vaddress tetra_r_x;
	hrt_vaddress tetra_r_y;
	hrt_vaddress tetra_gr_x;
	hrt_vaddress tetra_gr_y;
	hrt_vaddress tetra_gb_x;
	hrt_vaddress tetra_gb_y;
	hrt_vaddress tetra_b_x;
	hrt_vaddress tetra_b_y;
	hrt_vaddress tetra_ratb_x;
	hrt_vaddress tetra_ratb_y;
	hrt_vaddress tetra_batr_x;
	hrt_vaddress tetra_batr_y;
	hrt_vaddress dvs_6axis_params_y;
	hrt_vaddress r_gamma_tbl;
	hrt_vaddress g_gamma_tbl;
	hrt_vaddress b_gamma_tbl;
	hrt_vaddress anr_thres;
};

/* xmem address map allocation */
struct sh_css_ddr_address_map_size {
	size_t isp_param;
	size_t ctc_tbl;
	size_t gamma_tbl;
	size_t xnr_tbl;
	size_t macc_tbl;
	size_t fpn_tbl;
	size_t sc_tbl;
	size_t sdis_hor_coef;
	size_t sdis_ver_coef;
	size_t tetra_r_x;
	size_t tetra_r_y;
	size_t tetra_gr_x;
	size_t tetra_gr_y;
	size_t tetra_gb_x;
	size_t tetra_gb_y;
	size_t tetra_b_x;
	size_t tetra_b_y;
	size_t tetra_ratb_x;
	size_t tetra_ratb_y;
	size_t tetra_batr_x;
	size_t tetra_batr_y;
	size_t dvs_6axis_params_y;
	size_t r_gamma_tbl;
	size_t g_gamma_tbl;
	size_t b_gamma_tbl;
	size_t anr_thres;
};

struct sh_css_ddr_address_map_compound {
	struct sh_css_ddr_address_map		map;
	struct sh_css_ddr_address_map_size	size;
};

/* this struct contains all arguments that can be passed to
   a binary. It depends on the binary which ones are used. */
struct sh_css_binary_args {
	struct ia_css_frame *cc_frame;       /* continuous capture frame */
	struct ia_css_frame *in_frame;	     /* input frame */
	struct ia_css_frame *in_ref_frame;   /* reference input frame */
	struct ia_css_frame *in_tnr_frame;   /* tnr input frame */
	struct ia_css_frame *out_frame;      /* output frame */
	struct ia_css_frame *out_ref_frame;  /* reference output frame */
	struct ia_css_frame *out_tnr_frame;  /* tnr output frame */
	struct ia_css_frame *extra_frame;    /* intermediate frame */
	struct ia_css_frame *out_vf_frame;   /* viewfinder output frame */
	bool                 copy_vf;
	bool                 copy_output;
	unsigned             vf_downscale_log2;
};

enum sh_css_sp_stage_func {
  SH_CSS_SP_RAW_COPY  = 0,
  SH_CSS_SP_BIN_COPY  = 1,
  SH_CSS_SP_ISYS_COPY = 2,
  SH_CSS_SP_NO_FUNC   = 3,
};
#define SH_CSS_NUM_STAGE_FUNCS 3

/* Pipeline stage to be executed on SP/ISP */
struct sh_css_pipeline_stage {
	unsigned int		     stage_num;
	struct sh_css_binary	    *binary;      /* built-in binary */
	struct ia_css_binary_info   *binary_info;
	const struct ia_css_fw_info *firmware; /* acceleration binary */
	enum sh_css_sp_stage_func    sp_func; /* SP function for SP stage */
	unsigned		     max_input_width; /* For SP raw copy */
	struct sh_css_binary_args    args;
	int			     mode;
	bool			     out_frame_allocated;
	bool			     vf_frame_allocated;
	/* Indicate which buffers require an IRQ */
	unsigned int		     irq_buf_flags;
	struct sh_css_pipeline_stage *next;
};

/* Pipeline of n stages to be executed on SP/ISP per stage */
struct sh_css_pipeline {
	enum ia_css_pipe_id pipe_id;
	struct sh_css_pipeline_stage *stages;
	bool reload;
	struct sh_css_pipeline_stage *current_stage;
	unsigned num_stages;
	struct ia_css_frame in_frame;
	struct ia_css_frame out_frame;
	struct ia_css_frame vf_frame;
};


#if SP_DEBUG == SP_DEBUG_DUMP

#define SH_CSS_NUM_SP_DEBUG 48

struct sh_css_sp_debug_state {
	unsigned int error;
	unsigned int debug[SH_CSS_NUM_SP_DEBUG];
};

#elif SP_DEBUG == SP_DEBUG_COPY

#define SH_CSS_SP_DBG_TRACE_DEPTH	(40)

struct sh_css_sp_debug_trace {
	uint16_t frame;
	uint16_t line;
	uint16_t pixel_distance;
	uint16_t mipi_used_dword;
	uint16_t sp_index;
};

struct sh_css_sp_debug_state {
	uint16_t if_start_line;
	uint16_t if_start_column;
	uint16_t if_cropped_height;
	uint16_t if_cropped_width;
	unsigned int index;
	struct sh_css_sp_debug_trace
		trace[SH_CSS_SP_DBG_TRACE_DEPTH];
};

#elif SP_DEBUG == SP_DEBUG_TRACE

#define SH_CSS_SP_DBG_NR_OF_TRACES	(4)
#define SH_CSS_SP_DBG_TRACE_DEPTH	(20)

#define SH_CSS_SP_DBG_TRACE_FILE_ID_BIT_POS (13)

/* trace id 0..3 are used by the SP threads */
#define SH_CSS_SP_DBG_TRACE_ID_CONTROL (3) /* Re-use accl thread */
#define SH_CSS_SP_DBG_TRACE_ID_TBD  (5)

struct sh_css_sp_debug_trace {
	uint16_t time_stamp;
	uint16_t location;	/* bit 15..13 = file_id, 12..0 = line */
	uint32_t data;
};

struct sh_css_sp_debug_state {
	unsigned int mipi_fifo_high_water;
	struct sh_css_sp_debug_trace
		trace[SH_CSS_SP_DBG_NR_OF_TRACES][SH_CSS_SP_DBG_TRACE_DEPTH];
	uint8_t index[SH_CSS_SP_DBG_NR_OF_TRACES];
};


#elif SP_DEBUG == SP_DEBUG_MINIMAL

#define SH_CSS_NUM_SP_DEBUG 128

struct sh_css_sp_debug_state {
	unsigned int error;
	unsigned int debug[SH_CSS_NUM_SP_DEBUG];
};

#endif


struct sh_css_sp_debug_command {
	/*
	 * The DMA software-mask,
	 *	Bit 31...24: unused.
	 *	Bit 23...16: unused.
	 *	Bit 15...08: reading-request enabling bits for DMA channel 7..0
	 *	Bit 07...00: writing-reqeust enabling bits for DMA channel 7..0
	 *
	 * For example, "0...0 0...0 11111011 11111101" indicates that the
	 * writing request through DMA Channel 1 and the reading request
	 * through DMA channel 2 are both disabled. The others are enabled.
	 */
	uint32_t dma_sw_reg;
};

/* SP input formatter configuration.*/
struct sh_css_sp_input_formatter_set {
	uint32_t				stream_format;
	input_formatter_cfg_t	config_a;
	input_formatter_cfg_t	config_b;
};

/* SP configuration information */
struct sh_css_sp_config {
	uint8_t			is_offline;  /* Run offline, with continuous copy */
	uint8_t			input_needs_raw_binning;
	uint8_t			no_isp_sync; /* Signal host immediately after start */
	struct {
		uint8_t					a_changed;
		uint8_t					b_changed;
		uint8_t					isp_2ppc;
		struct sh_css_sp_input_formatter_set	set[SH_CSS_MAX_IF_CONFIGS]; /* CSI-2 port is used as index. */
	} input_formatter;
	sync_generator_cfg_t	sync_gen;
	tpg_cfg_t				tpg;
	prbs_cfg_t				prbs;
	input_system_cfg_t		input_circuit;
	uint8_t					input_circuit_cfg_changed;
};

enum sh_css_stage_type {
  SH_CSS_SP_STAGE_TYPE  = 0,
  SH_CSS_ISP_STAGE_TYPE = 1
};
#define SH_CSS_NUM_STAGE_TYPES 2

#define SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS 	(1 << 0)
#define SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS_MASK \
	((SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS << SH_CSS_MAX_SP_THREADS)-1)

/* Information for a pipeline */
struct sh_css_sp_pipeline {
	uint32_t	pipe_id;	/* the pipe ID */
	uint32_t	pipe_num;	/* the dynamic pipe number */
	uint32_t	thread_id;	/* the sp thread ID */
	uint32_t	pipe_config;	/* the pipe config */
	uint32_t	input_system_mode;	/* enum ia_css_input_mode */
	mipi_port_ID_t	port_id;	/* port_id for input system */
	uint32_t	num_stages;		/* the pipe config */
	uint32_t	running;	/* needed for pipe termination */
	hrt_vaddress	sp_stage_addr[SH_CSS_MAX_STAGES];
	struct sh_css_sp_stage *stage; /* Current stage for this pipeline */
	union {
		struct {
			unsigned int	bytes_available;
		} bin;
		struct {
			unsigned int	height;
			unsigned int	width;
			unsigned int	padded_width;
			unsigned int	max_input_width;
			unsigned int	raw_bit_depth;
		} raw;
	} copy;
};

/*
 * These structs are derived from structs defined in ia_css_types.h
 * (just take out the "_sp" from the struct name to get the "original")
 * All the fields that are not needed by the SP are removed.
 */
struct sh_css_sp_frame_plane {
	unsigned int offset;	/* offset in bytes to start of frame data */
				/* offset is wrt data in sh_css_sp_sp_frame */
};

struct sh_css_sp_frame_binary_plane {
	unsigned int size;
	struct sh_css_sp_frame_plane data;
};

struct sh_css_sp_frame_yuv_planes {
	struct sh_css_sp_frame_plane y;
	struct sh_css_sp_frame_plane u;
	struct sh_css_sp_frame_plane v;
};

struct sh_css_sp_frame_nv_planes {
	struct sh_css_sp_frame_plane y;
	struct sh_css_sp_frame_plane uv;
};

struct sh_css_sp_frame_rgb_planes {
	struct sh_css_sp_frame_plane r;
	struct sh_css_sp_frame_plane g;
	struct sh_css_sp_frame_plane b;
};

struct sh_css_sp_frame_plane6_planes {
	struct sh_css_sp_frame_plane r;
	struct sh_css_sp_frame_plane r_at_b;
	struct sh_css_sp_frame_plane gr;
	struct sh_css_sp_frame_plane gb;
	struct sh_css_sp_frame_plane b;
	struct sh_css_sp_frame_plane b_at_r;
};

/* MW: ALL CAPS, and is it too much trouble to suffix an ID with ID ? */
enum sh_css_frame_id {
	sh_css_frame_in,		/* Dynamic */
	sh_css_frame_out,		/* Dynamic */
	sh_css_frame_out_vf,		/* Dynamic */
	sh_css_frame_s3a,		/* Dynamic */
	sh_css_frame_dis,		/* Dynamic */
	sh_css_frame_ref_in,
	sh_css_frame_ref_out,
	sh_css_frame_tnr_in,
	sh_css_frame_tnr_out,
	sh_css_frame_extra,
	sh_css_frame_raw_out,
	sh_css_frame_cust_in,
	sh_css_frame_cust_out,
};
/*
 * The first frames (with comment Dynamic) can be dynamic or static
 * The other frames (ref_in and below) can only be static
 * Static means that the data addres will not change during the life time
 * of the associated pipe. Dynamic means that the data address can
 * change with every (frame) iteration of the associated pipe
 *
 * s3a and dis are now also dynamic but (stil) handled seperately
 */
#define SH_CSS_NUM_FRAME_IDS (13)
#define SH_CSS_NUM_DYNAMIC_BUFFER_IDS (5)
#define SH_CSS_NUM_DYNAMIC_FRAME_IDS (3)
#define SH_CSS_INVALID_FRAME_ID (-1)


/** Frame info struct. This describes the contents of an image frame buffer.
  */
struct sh_css_sp_frame_info {
	uint16_t width;  /**< width of valid data in pixels */
	uint16_t height; /**< Height of valid data in lines */
	uint16_t padded_width; /**< stride of line in memory (in pixels) */
	unsigned char format; /**< format of the frame data */
	unsigned char raw_bit_depth; /**< number of valid bits per pixel,
					 only valid for RAW bayer frames */
	unsigned char raw_bayer_order; /**< bayer order, only valid
						      for RAW bayer frames */
	unsigned char padding;
};


struct sh_css_sp_frame {
	struct sh_css_sp_frame_info info;
	union {
		struct sh_css_sp_frame_plane raw;
		struct sh_css_sp_frame_plane rgb;
		struct sh_css_sp_frame_rgb_planes planar_rgb;
		struct sh_css_sp_frame_plane yuyv;
		struct sh_css_sp_frame_yuv_planes yuv;
		struct sh_css_sp_frame_nv_planes nv;
		struct sh_css_sp_frame_plane6_planes plane6;
		struct sh_css_sp_frame_binary_plane binary;
	} planes;
};

struct sh_css_sp_frames {
	struct sh_css_sp_frame	in;
	struct sh_css_sp_frame	out;
	struct sh_css_sp_frame	out_vf;
	struct sh_css_sp_frame	ref_in;
	/* ref_out_frame is same as ref_in_frame */
	struct sh_css_sp_frame	tnr_in;
	/* trn_out_frame is same as tnr_in_frame */
	struct sh_css_sp_frame	extra;
	struct sh_css_sp_frame_info internal_frame_info;
	hrt_vaddress static_frame_data[SH_CSS_NUM_FRAME_IDS];
};

/* Information for a single pipeline stage for an ISP */
struct sh_css_isp_stage {
	/*
	 * For compatability and portabilty, only types
	 * from "stdint.h" are allowed
	 *
	 * Use of "enum" and "bool" is prohibited
	 * Multiple boolean flags can be stored in an
	 * integer
	 */
	struct ia_css_blob_info	  blob_info;
	struct ia_css_binary_info binary_info;
	char			  binary_name[SH_CSS_MAX_BINARY_NAME];
	struct ia_css_data        mem_initializers[IA_CSS_NUM_ISP_MEMORIES];
};

/* Information for a single pipeline stage */
struct sh_css_sp_stage {
	/*
	 * For compatability and portabilty, only types
	 * from "stdint.h" are allowed
	 *
	 * Use of "enum" and "bool" is prohibited
	 * Multiple boolean flags can be stored in an
	 * integer
	 */
	uint8_t			num; /* Stage number */
	uint8_t			isp_online;
	uint8_t			isp_copy_vf;
	uint8_t			isp_copy_output;
	uint8_t			sp_enable_xnr;
	uint8_t			isp_deci_log_factor;
	uint8_t			isp_vf_downscale_bits;
	uint8_t			anr;
	uint8_t			deinterleaved;
/*
 * NOTE: Programming the input circuit can only be done at the
 * start of a session. It is illegal to program it during execution
 * The input circuit defines the connectivity
 */
	uint8_t			program_input_circuit;
/* enum sh_css_sp_stage_func	func; */
	uint8_t			func;
	/* The type of the pipe-stage */
	/* enum sh_css_stage_type	stage_type; */
	uint8_t			stage_type;
	uint8_t			num_stripes;
	uint8_t			isp_pipe_version;
	struct {
		uint8_t		vf_output;
		uint8_t		s3a;
		uint8_t		sdis;
	} enable;
	/* Add padding to come to a word boundary */
	/* unsigned char			padding[0]; */

	struct sh_css_crop_pos		sp_out_crop_pos;
	/* Indicate which buffers require an IRQ */
	uint32_t					irq_buf_flags;
	struct sh_css_sp_frames		frames;
	struct ia_css_resolution	dvs_envelope;
	struct sh_css_uds_info		uds;
	hrt_vaddress			isp_stage_addr;
	hrt_vaddress			xmem_bin_addr;
	hrt_vaddress			xmem_map_addr;

	uint8_t			if_config_index; /* Which should be applied by this stage. */
};

/*
 * Time: 2012-07-19, 17:40.
 * Author: zhengjie.lu@intel.com
 * Note: Add a new data memeber "debug" in "sh_css_sp_group". This
 * data member is used to pass the debugging command from the
 * Host to the SP.
 *
 * Time: Before 2012-07-19.
 * Author: unknown
 * Note:
 * Group all host initialized SP variables into this struct.
 * This is initialized every stage through dma.
 * The stage part itself is transfered through sh_css_sp_stage.
*/
struct sh_css_sp_group {
	struct sh_css_sp_config		config;
	struct sh_css_sp_pipeline	pipe[SH_CSS_MAX_SP_THREADS];

	struct sh_css_sp_debug_command	debug;
};

/* Data in SP dmem that is set from the host every stage. */
struct sh_css_sp_per_frame_data {
	/* ddr address of sp_group and sp_stage */
	hrt_vaddress			sp_group_addr;
};

#define SH_CSS_NUM_SDW_IRQS 3

/* Output data from SP to css */
struct sh_css_sp_output {
	unsigned int			bin_copy_bytes_copied;
#if SP_DEBUG != SP_DEBUG_NONE
	struct sh_css_sp_debug_state	debug;
#endif
	unsigned int		sw_interrupt_value[SH_CSS_NUM_SDW_IRQS];
};

/**
 * @brief Data structure for the circular buffer.
 * The circular buffer is empty if "start == end". The
 * circular buffer is full if "(end + 1) % size == start".
 */
#define  SH_CSS_CIRCULAR_BUF_NUM_ELEMS	6
struct sh_css_circular_buf {
	/*
	 * WARNING: Do NOT change the memeber orders below,
	 * unless you are the expert of either the CSS API
	 * or the SP code.
	 */

	uint8_t size;  /* maximum number of elements */
	uint8_t step;  /* number of bytes per element */
	uint8_t start; /* index of the oldest element */
	uint8_t end;   /* index at which to write the new element */

	uint32_t elems[SH_CSS_CIRCULAR_BUF_NUM_ELEMS]; /* array of elements */
};

struct sh_css_hmm_buffer {
	hrt_vaddress kernel_ptr;
	union {
		struct ia_css_isp_3a_statistics  s3a;
		struct ia_css_isp_dvs_statistics dis;
//		hrt_vaddress frame_data;
		struct {
			hrt_vaddress	frame_data;
			unsigned int	flashed;
			unsigned int	exp_id;
		} frame;
		hrt_vaddress ddr_ptrs;
	} payload;
};

enum sh_css_buffer_queue_id {
	sh_css_invalid_buffer_queue = -1,
	sh_css_input_buffer_queue,
	sh_css_output_buffer_queue,
	sh_css_vf_output_buffer_queue,
	sh_css_s3a_buffer_queue,
	sh_css_dis_buffer_queue,
	sh_css_param_buffer_queue,
	sh_css_tag_cmd_queue
};

struct sh_css_event_irq_mask {
	uint16_t or_mask;
	uint16_t and_mask;
};

#define SH_CSS_NUM_BUFFER_QUEUES 7

struct host_sp_communication {
	/*
	 * Don't use enum host2sp_commands, because the sizeof an enum is
	 * compiler dependant and thus non-portable
	 */
	uint32_t host2sp_command;

	/*
	 * The frame buffers that are reused by the
	 * copy pipe in the offline preview mode.
	 *
	 * host2sp_offline_frames[0]: the input frame of the preview pipe.
	 * host2sp_offline_frames[1]: the output frame of the copy pipe.
	 *
	 * TODO:
	 *   Remove it when the Host and the SP is decoupled.
	 */
	hrt_vaddress host2sp_offline_frames[NUM_CONTINUOUS_FRAMES];
	hrt_vaddress host2sp_mipi_frames[NUM_MIPI_FRAMES];
	uint32_t host2sp_cont_num_raw_frames;
	uint32_t host2sp_cont_num_mipi_frames;
	struct sh_css_event_irq_mask host2sp_event_irq_mask[NR_OF_PIPELINES];

};

struct host_sp_queues {
	/*
	 * Queues for the dynamic frame information,
	 * i.e. the "in_frame" buffer, the "out_frame"
	 * buffer and the "vf_out_frame" buffer.
	 */
	struct sh_css_circular_buf host2sp_buffer_queues
		[SH_CSS_MAX_SP_THREADS][SH_CSS_NUM_BUFFER_QUEUES];
	struct sh_css_circular_buf sp2host_buffer_queues
		[SH_CSS_NUM_BUFFER_QUEUES];

	/*
	 * The queue for the events.
	 */
	struct sh_css_circular_buf host2sp_event_queue;
	struct sh_css_circular_buf sp2host_event_queue;
};

struct ia_css_isp_gamma_table {
	uint16_t data[SH_CSS_ISP_GAMMA_TABLE_SIZE];
};

struct ia_css_isp_ctc_table {
	uint16_t data[SH_CSS_ISP_CTC_TABLE_SIZE];
};

struct ia_css_isp_rgb_gamma_table {
	uint16_t data[SH_CSS_ISP_RGB_GAMMA_TABLE_SIZE];
};

struct ia_css_isp_xnr_table {
	uint16_t data[SH_CSS_ISP_XNR_TABLE_SIZE];
};

extern int (*sh_css_printf) (const char *fmt, va_list args);

#ifdef __HIVECC
/* inline functions in hivecc cannot use varargs */
static void
#else
STORAGE_CLASS_INLINE void
#endif
sh_css_print(const char *fmt, ...)
{
	va_list ap;

	if (sh_css_printf) {
		va_start(ap, fmt);
		sh_css_printf(fmt, ap);
		va_end(ap);
	}
}

#ifdef __HIVECC
/* inline functions in hivecc cannot use varargs */
static void
#else
STORAGE_CLASS_INLINE void
#endif
sh_css_vprint(const char *fmt, va_list args)
{
	if (sh_css_printf)
		sh_css_printf(fmt, args);
}

hrt_vaddress
sh_css_params_ddr_address_map(void);

enum ia_css_err
sh_css_params_init(void);

void
sh_css_params_uninit(void);

void
sh_css_params_reconfigure_gdc_lut(void);

void *
sh_css_malloc(size_t size);

void *
sh_css_calloc(size_t N, size_t size);

void
sh_css_free(void *ptr);

/* For Acceleration API: Flush FW (shared buffer pointer) arguments */
extern void
sh_css_flush(struct ia_css_acc_fw *fw);

/* Check two frames for equality (format, resolution, bits per element) */
bool
sh_css_frame_equal_types(const struct ia_css_frame *frame_a,
			 const struct ia_css_frame *frame_b);

bool
sh_css_frame_info_equal_resolution(const struct ia_css_frame_info *info_a,
				   const struct ia_css_frame_info *info_b);

unsigned int
sh_css_input_format_bits_per_pixel(enum ia_css_stream_format format,
				   bool two_ppc);

enum ia_css_err
sh_css_vf_downscale_log2(const struct ia_css_frame_info *out_info,
			 const struct ia_css_frame_info *vf_info,
			 unsigned int *downscale_log2);

void
sh_css_capture_enable_bayer_downscaling(bool enable);

void
sh_css_binary_print(const struct sh_css_binary *binary);

#if SP_DEBUG !=SP_DEBUG_NONE

void
sh_css_print_sp_debug_state(const struct sh_css_sp_debug_state *state);

#endif

void
sh_css_frame_info_set_width(struct ia_css_frame_info *info,
			    unsigned int width);

/* Return whether the sp copy process should be started */
bool
sh_css_continuous_start_sp_copy(void);

bool
sh_css_pipe_uses_params(struct sh_css_pipeline *me);

hrt_vaddress
sh_css_store_sp_group_to_ddr(void);

hrt_vaddress
sh_css_store_sp_stage_to_ddr(unsigned pipe, unsigned stage);

hrt_vaddress
sh_css_store_isp_stage_to_ddr(unsigned pipe, unsigned stage);

void
sh_css_frame_info_init(struct ia_css_frame_info *info,
		       unsigned int width,
		       unsigned int height,
		       enum ia_css_frame_format format);

bool
sh_css_enqueue_frame(unsigned int pipe_num,
		     enum sh_css_frame_id frame_id,
		     struct ia_css_frame *frame);

/**
 * @brief Query the SP thread ID.
 *
 * @param[in]	key	The query key, typical use is pipe_num.
 * @param[out]	val	The query value.
 *
 * @return
 *	true, if the query succeeds;
 *	false, if the query fails.
 */
bool
sh_css_query_sp_thread_id(unsigned int key,
			  unsigned int *val);

/**
 * @brief Query the internal frame ID.
 *
 * @param[in]	key	The query key.
 * @param[out]	val	The query value.
 *
 * @return
 *	true, if the query succeeds;
 *	false, if the query fails.
 */
bool
sh_css_query_internal_queue_id(enum ia_css_buffer_type key,
		enum sh_css_buffer_queue_id *val);

void
sh_css_update_uds_and_crop_info(
		const struct ia_css_binary_info *info,
		const struct ia_css_frame_info *in_frame_info,
		const struct ia_css_frame_info *out_frame_info,
		const struct ia_css_resolution *dvs_env,
		bool preview_mode,
		const struct ia_css_dz_config *zoom,
		const struct ia_css_vector *motion_vector,
		struct sh_css_uds_info *uds,		/* out */
		struct sh_css_crop_pos *sp_out_crop_pos	/* out */
		);

void
sh_css_invalidate_shading_tables(struct ia_css_stream *stream);

struct sh_css_pipeline *
ia_css_pipe_get_pipeline(const struct ia_css_pipe *pipe);

unsigned int
ia_css_pipe_get_pipe_num(const struct ia_css_pipe *pipe);

bool
sh_css_continuous_is_enabled(uint8_t pipe_num);

#endif /* _SH_CSS_INTERNAL_H_ */
