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

/*! \file */
//#include "stdio.h"

#include "ia_css.h"
#include "sh_css_hrt.h"		/* only for file 2 MIPI */
#include "sh_css_binary.h"
#include "sh_css_internal.h"
#include "sh_css_sp.h"
#include "sh_css_sp_start.h"
#include "sh_css_rx.h"
#include "sh_css_defs.h"
#include "sh_css_firmware.h"
#include "ia_css_accelerate.h"
#include "sh_css_params.h"
#include "sh_css_params_internal.h"
#include "sh_css_param_shading.h"
#include "sh_css_refcount.h"
#include "ia_css_i_rmgr.h"
#include "sh_css_debug.h"
#include "sh_css_debug_internal.h"
#include "ia_css_memory_access.h"
#include "ia_css_device_access.h"
#include "sh_css_legacy.h"
#include "ia_css_acc.h"
#include "ia_css_stream.h"
//#include "ia_css_stream_manager.h"
//#include "ia_css_i_host_rmgr_gen_shared.h"
#include "input_system_init.h"

#include "memory_access.h"
#include "tag.h"
#include "assert_support.h"
#include "queue.h"			/* host2sp_enqueue_frame_data() */
#include "sw_event.h"			/* encode_sw_event */
#include "input_formatter.h"/* input_formatter_cfg_t,
	input_formatter_get_alignment(), ... */
#include "input_system.h"
#include "mmu_device.h"		/* mmu_set_page_table_base_index(), ... */
#include "gdc_device.h"		/* HRT_GDC_N */
#include "irq.h"			/* virq */
#include "sp.h"				/* cnd_sp_irq_enable() */
#include "isp.h"			/* cnd_isp_irq_enable, ISP_VEC_NELEMS */
#define __INLINE_GPIO__
#include "gpio.h"
#include "timed_ctrl.h"
#include "platform_support.h" /* hrt_sleep() */

#define WITH_PC_MONITORING  0
#define MAX_NUM_PIPES 20
#define MAX_NUM_SP_THREADS 4
#define PIPE_NUM_EMPTY_TOKEN 0xFFFF
#define PIPE_NUM_RESERVED_TOKEN 0x1
#define SP_THREAD_EMPTY_TOKEN 0x0
#define SP_THREAD_RESERVED_TOKEN 0x1


#if WITH_PC_MONITORING
#define MULTIPLE_SAMPLES 1
#define NOF_SAMPLES      60
#include "linux/kthread.h"
#include "linux/sched.h"
#include "linux/delay.h"
#include "sh_css_metrics.h"
static int thread_alive;
#endif /* WITH_PC_MONITORING */

#define DVS_REF_TESTING 0
#if DVS_REF_TESTING
#include <stdio.h>
#endif

/* Name of the sp program: should not be built-in */
#define SP_PROG_NAME "sp"

/* for JPEG, we don't know the length of the image upfront,
 * but since we support sensor upto 16MP, we take this as
 * upper limit.
 */
#define JPEG_BYTES (16 * 1024 * 1024)

#define IS_ODD(a)              ((a) & 0x1)

#define IMPLIES(a, b)           (!(a) || (b))   /* A => B */

#define STATS_ENABLED(stage) (stage && stage->binary && stage->binary->info && \
	(stage->binary->info->enable.s3a || stage->binary->info->enable.dis))

#define DEFAULT_IF_CONFIG \
{ \
	0,          /* start_line */\
	0,          /* start_column */\
	0,          /* left_padding */\
	0,          /* cropped_height */\
	0,          /* cropped_width */\
	0,          /* deinterleaving */\
	0,          /*.buf_vecs */\
	0,          /* buf_start_index */\
	0,          /* buf_increment */\
	0,          /* buf_eol_offset */\
	false,      /* is_yuv420_format */\
	false       /* block_no_reqs */\
}

#define DEFAULT_PLANES { {0, 0, 0, 0} }

#define DEFAULT_FRAME \
{ \
	DEFAULT_FRAME_INFO,            /* info */ \
	0,                             /* data */ \
	0,                             /* data_bytes */ \
	-1,                            /* dynamic_data_index */ \
	IA_CSS_FRAME_FLASH_STATE_NONE, /* flash_state */ \
	0,                             /* exp_id */ \
	false,                         /* valid */ \
	false,                         /* contiguous  */ \
	{ 0 }                          /* planes */ \
}

#define DEFAULT_PIPELINE \
{ \
	IA_CSS_PIPE_ID_PREVIEW, /* pipe_id */ \
	NULL,                   /* stages */ \
	true,                   /* reload */ \
	NULL,                   /* current_stage */ \
	0,                      /* num_stages */ \
	DEFAULT_FRAME,          /* in_frame */ \
	DEFAULT_FRAME,          /* out_frame */ \
	DEFAULT_FRAME           /* vf_frame */ \
}

#define DEFAULT_PIPE \
{ \
	IA_CSS_PIPE_ID_ACC, \
	0,                         /* pipe_num */ \
	false,                     /* zoom_changed */ \
	NULL,                      /* shading_table */ \
	DEFAULT_PIPELINE,          /* pipeline */ \
	DEFAULT_FRAME_INFO,        /* output_info */\
	DEFAULT_FRAME_INFO,        /* vf_output_info */ \
	DEFAULT_FRAME_INFO,        /* yuv_ds_input_info */\
	false,                     /* disable_vf_pp */\
	false,                     /* disable_capture_pp */ \
	false,                     /* input_needs_raw_binning */ \
	NULL,                      /* output_stage */\
	NULL,                      /* vf_stage */ \
	IA_CSS_CAPTURE_MODE_PRIMARY,/* capture_mode */ \
	false,                     /* xnr */ \
	{ 0, 0 },                  /* dvs_envelope */ \
	0,                         /* num_invalid_frames */ \
	false,                     /* enable_yuv_ds */ \
	false,                     /* enable_high_speed */ \
	false,                     /* enable_dvs_6axis */ \
	true,                      /* enable_viewfinder */ \
	true,                      /* enable_dz */ \
	false,                     /* enable_reduced_pipe */ \
	1,                         /* isp_pipe_version */ \
	NULL,                      /* stream */ \
	NULL,                      /* new_pipe */ \
	DEFAULT_FRAME,             /* out_frame */ \
	DEFAULT_FRAME,             /* vf_frame */ \
	{ NULL },                  /* continuous_frames */\
	{ DEFAULT_PREVIEW_SETTINGS } /* pipe */ \
}

struct sh_css_preview_settings {
	struct sh_css_binary copy_binary;
	struct sh_css_binary preview_binary;
	struct sh_css_binary vf_pp_binary;
	struct sh_css_pipe *copy_pipe;
	struct sh_css_pipe *capture_pipe;
};

#define DEFAULT_PREVIEW_SETTINGS \
{ \
	DEFAULT_BINARY_SETTINGS,  /* copy_binary */\
	DEFAULT_BINARY_SETTINGS,  /* preview_binary */\
	DEFAULT_BINARY_SETTINGS,  /* vf_pp_binary */\
	NULL,                     /* copy_pipe */\
	NULL,                     /* capture_pipe */\
}

struct sh_css_capture_settings {
	struct sh_css_binary copy_binary;
	struct sh_css_binary primary_binary;
	struct sh_css_binary pre_isp_binary;
	struct sh_css_binary gdc_binary;
	struct sh_css_binary post_isp_binary;
	struct sh_css_binary pre_anr_binary;
	struct sh_css_binary anr_binary;
	struct sh_css_binary post_anr_binary;
	struct sh_css_binary capture_pp_binary;
	struct sh_css_binary vf_pp_binary;
	struct ia_css_frame *capture_pp_frame;
};

#define DEFAULT_CAPTURE_SETTINGS \
{ \
	DEFAULT_BINARY_SETTINGS,     /* copy_binary */\
	DEFAULT_BINARY_SETTINGS,     /* primary_binary */\
	DEFAULT_BINARY_SETTINGS,     /* pre_isp_binary */\
	DEFAULT_BINARY_SETTINGS,     /* gdc_binary */\
	DEFAULT_BINARY_SETTINGS,     /* post_isp_binary */\
	DEFAULT_BINARY_SETTINGS,     /* pre_anr_binary */\
	DEFAULT_BINARY_SETTINGS,     /* anr_binary */\
	DEFAULT_BINARY_SETTINGS,     /* post_anr_binary */\
	DEFAULT_BINARY_SETTINGS,     /* capture_pp_binary */\
	DEFAULT_BINARY_SETTINGS,     /* vf_pp_binary */\
	NULL,                        /* capture_pp_frame */\
}

struct sh_css_video_settings {
	struct sh_css_binary copy_binary;
	struct sh_css_binary video_binary;
	struct sh_css_binary vf_pp_binary;
	struct ia_css_frame *ref_frames[NUM_VIDEO_REF_FRAMES];
	struct ia_css_frame *tnr_frames[NUM_VIDEO_TNR_FRAMES];
	struct ia_css_frame *vf_pp_in_frame;
	struct sh_css_pipe *copy_pipe;
	struct sh_css_pipe *capture_pipe;
};

#define DEFAULT_VIDEO_SETTINGS \
{ \
	DEFAULT_BINARY_SETTINGS,/* copy_binary */ \
	DEFAULT_BINARY_SETTINGS,/* video_binary */ \
	DEFAULT_BINARY_SETTINGS,/* vf_pp_binary */ \
	{ NULL },                /* ref_frames */ \
	{ NULL },                /* tnr_frames */ \
	NULL,                    /* vf_pp_in_frame */ \
	NULL,                    /* copy_pipe */ \
	NULL,                    /* capture_pipe */ \
}

struct ia_css_pipe {
	uint8_t                         pipe_num;
	bool                            stop_requested;
	struct ia_css_pipe_config       config;
	struct ia_css_pipe_extra_config extra_config;
	struct ia_css_pipe_info         info;
	struct sh_css_pipe             *old_pipe;
};

struct sh_css_pipe {
	enum ia_css_pipe_id          mode;
	uint8_t                      pipe_num;
	bool                         zoom_changed;
	struct ia_css_shading_table *shading_table;
	struct sh_css_pipeline       pipeline;
	struct ia_css_frame_info     output_info;
	struct ia_css_frame_info     vf_output_info;
	struct ia_css_frame_info     yuv_ds_input_info;
	bool                         disable_vf_pp;
	bool                         disable_capture_pp;
	bool                         input_needs_raw_binning;
	struct ia_css_fw_info	    *output_stage; /* extra output stage */
	struct ia_css_fw_info	    *vf_stage;     /* extra vf stage */
	enum ia_css_capture_mode     capture_mode;
	bool                         xnr;
	struct ia_css_resolution     dvs_envelope;
	int                          num_invalid_frames;
	bool                         enable_yuv_ds;
	bool                         enable_high_speed;
	bool                         enable_dvs_6axis;
	bool                         enable_viewfinder;
	bool                         enable_dz;
	bool                         enable_reduced_pipe;
	unsigned int                 isp_pipe_version;
	struct ia_css_stream         *stream;
	struct ia_css_pipe           *new_pipe;
	struct ia_css_frame          out_frame_struct;
	struct ia_css_frame          vf_frame_struct;
	struct ia_css_frame         *continuous_frames[NUM_CONTINUOUS_FRAMES];
	union {
		struct sh_css_preview_settings preview;
		struct sh_css_video_settings   video;
		struct sh_css_capture_settings capture;
	} pipe;
};

struct sh_css {
	struct ia_css_pipe            *active_pipes[MAX_NUM_PIPES];
	void *(*malloc) (size_t bytes, bool zero_mem);
	void (*free) (void *ptr);
	void (*flush) (struct ia_css_acc_fw *fw);
	bool                           check_system_idle;
	bool                           stop_copy_preview;
	unsigned int                   num_cont_raw_frames;
	unsigned int                   num_mipi_frames;
	struct ia_css_frame	      *mipi_frames[NUM_MIPI_FRAMES];
	bool                           start_sp_copy;
	hrt_vaddress                   sp_bin_addr;
	hrt_data                       page_table_base_index;
	unsigned int                   size_mem_words;
	bool                           contiguous;
	enum ia_css_irq_type           irq_type;
};

#define DEFAULT_CSS \
{ \
	{NULL, NULL, NULL, NULL },/* active_pipes */ \
	NULL,                     /* malloc */ \
	NULL,                     /* free */ \
	NULL,                     /* flush */ \
	true,                     /* check_system_idle */ \
	false,                    /* stop_copy_preview */ \
	NUM_CONTINUOUS_FRAMES,    /* num_cont_raw_frames */ \
	NUM_MIPI_FRAMES,	  /* num_mipi_frames */ \
	{ NULL }, 		  /* mipi_frames */ \
	false,                    /* start_sp_copy */ \
	0,                        /* sp_bin_addr */ \
	0,                        /* page_table_base_index */ \
	0,                        /* size_mem_words */ \
	true,                     /* contiguous */ \
	IA_CSS_IRQ_TYPE_EDGE      /* irq_type */ \
}

#if defined(HAS_RX_VERSION_2)

#define DEFAULT_MIPI_CONFIG \
{ \
	MONO_4L_1L_0L, \
	MIPI_PORT0_ID, \
	0xffff4, \
	0, \
	0x28282828, \
	0x04040404, \
	MIPI_PREDICTOR_NONE, \
	false \
}

#elif defined(HAS_RX_VERSION_1) || defined(HAS_NO_RX)

#define DEFAULT_MIPI_CONFIG \
{ \
	MIPI_PORT1_ID, \
	1, \
	0xffff4, \
	0, \
	0, \
	MIPI_PREDICTOR_NONE, \
	false \
}

#else
#error "sh_css.c: RX version must be one of {RX_VERSION_1, RX_VERSION_2, NO_RX}"
#endif

int (*sh_css_printf) (const char *fmt, va_list args) = NULL;

static struct sh_css my_css;
/* static variables, temporarily used in load_<mode>_binaries.
   Declaring these inside the functions increases the size of the
   stack frames beyond the acceptable 128 bytes. */
static struct sh_css_binary_descr preview_descr,
				  vf_pp_descr,
				  copy_descr,
				  prim_descr,
				  pre_gdc_descr,
				  gdc_descr,
				  post_gdc_descr,
				  pre_anr_descr,
				  anr_descr,
				  post_anr_descr,
				  video_descr,
				  capture_pp_descr;

/* pqiao NOTICE: this is for css internal buffer recycling when stopping pipeline,
   this array is temporary and will be replaced by resource manager*/
#define MAX_HMM_BUFFER_NUM (SH_CSS_NUM_BUFFER_QUEUES * (SH_CSS_CIRCULAR_BUF_NUM_ELEMS + 2))
static struct ia_css_i_host_rmgr_vbuf_handle *hmm_buffer_record_h[MAX_HMM_BUFFER_NUM];

static uint32_t pipe_num_counter = 0;
static uint32_t ref_count_mipi_allocation = 0;
static uint32_t pipe_num_list[MAX_NUM_PIPES];
static unsigned int sp_thread_list[MAX_NUM_SP_THREADS];
static unsigned int pipe_num_to_sp_thread[MAX_NUM_PIPES];

#define GPIO_FLASH_PIN_MASK (1 << HIVE_GPIO_STROBE_TRIGGER_PIN)

static enum sh_css_buffer_queue_id
	sh_css_buf_type_2_internal_queue_id[IA_CSS_BUFFER_TYPE_NUM] = {
		sh_css_s3a_buffer_queue,
		sh_css_dis_buffer_queue,
		sh_css_input_buffer_queue,
		sh_css_output_buffer_queue,
		sh_css_vf_output_buffer_queue,
		sh_css_output_buffer_queue,
		sh_css_input_buffer_queue,
		sh_css_output_buffer_queue,
		sh_css_param_buffer_queue };

/**
 * Local prototypes
 */
static void
init_pipe_number(void);

static enum ia_css_err
ia_css_pipe_dequeue_unused_buffer(struct ia_css_pipe *pipe);

static enum ia_css_err
ia_css_pipe_load_extension(struct ia_css_pipe *pipe,
			   struct ia_css_fw_info *firmware);

static void
ia_css_pipe_unload_extension(struct ia_css_pipe *pipe,
			     struct ia_css_fw_info *firmware);

static void
sh_css_init_host_sp_control_vars(void);

static void
sh_css_mmu_set_page_table_base_index(hrt_data base_index);

static bool
need_capture_pp(const struct sh_css_pipe *pipe);

static enum ia_css_err
sh_css_pipe_load_binaries(struct sh_css_pipe *pipe);

static
enum ia_css_err sh_css_pipe_get_viewfinder_frame_info(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *info);

static enum ia_css_err
sh_css_pipe_get_output_frame_info(struct sh_css_pipe *pipe,
				  struct ia_css_frame_info *info);

static enum ia_css_err
capture_start(struct sh_css_pipe *pipe);

static enum ia_css_err
video_start(struct sh_css_pipe *pipe);

static bool copy_on_sp(
	struct sh_css_pipe *pipe);

static enum ia_css_err
construct_copy_pipe(struct sh_css_pipe *pipe,
		    unsigned max_input_width,
		    struct ia_css_frame *out_frame);

static enum ia_css_err
construct_capture_pipe(struct sh_css_pipe *pipe);

static enum ia_css_err
init_frame_planes(struct ia_css_frame *frame);


static void
free_mipi_frames(struct sh_css_pipe *pipe);


#if 0
static enum ia_css_err
sh_css_pipeline_stop(struct sh_css_pipe *pipe);
#endif

static struct sh_css_binary *
ia_css_pipe_get_3a_binary (const struct ia_css_pipe *pipe);

static void
sh_css_pipe_free_shading_table(struct sh_css_pipe *pipe)
{
	if (pipe->shading_table)
		ia_css_shading_table_free(pipe->shading_table);
	pipe->shading_table = NULL;
}

static enum ia_css_err
check_frame_info(const struct ia_css_frame_info *info)
{
	if (info->res.width == 0 || info->res.height == 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
check_vf_info(const struct ia_css_frame_info *info)
{
	enum ia_css_err err;
	err = check_frame_info(info);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (info->res.width > sh_css_max_vf_width()*2)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
check_vf_out_info(const struct ia_css_frame_info *out_info,
		  const struct ia_css_frame_info *vf_info)
{
	enum ia_css_err err;
	err = check_frame_info(out_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = check_vf_info(vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (vf_info->res.width > out_info->res.width ||
	    vf_info->res.height > out_info->res.height)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
check_res(unsigned int width, unsigned int height)
{
	if (width  == 0   ||
	    height == 0   ||
	    IS_ODD(width) ||
	    IS_ODD(height)) {
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	return IA_CSS_SUCCESS;
}
#if 0
static enum ia_css_err
check_null_res(unsigned int width, unsigned int height)
{
	if (IS_ODD(width) || IS_ODD(height))
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	return IA_CSS_SUCCESS;
}
#endif
static bool
input_format_is_raw(enum ia_css_stream_format format)
{
	return format == IA_CSS_STREAM_FORMAT_RAW_6 ||
	    format == IA_CSS_STREAM_FORMAT_RAW_7 ||
	    format == IA_CSS_STREAM_FORMAT_RAW_8 ||
	    format == IA_CSS_STREAM_FORMAT_RAW_10 ||
	    format == IA_CSS_STREAM_FORMAT_RAW_12;
	/* raw_14 and raw_16 are not supported as input formats to the ISP.
	 * They can only be copied to a frame in memory using the
	 * copy binary.
	 */
}

static bool
input_format_is_yuv(enum ia_css_stream_format format)
{
	return format == IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY ||
	    format == IA_CSS_STREAM_FORMAT_YUV420_8 ||
	    format == IA_CSS_STREAM_FORMAT_YUV420_10 ||
	    format == IA_CSS_STREAM_FORMAT_YUV422_8 ||
	    format == IA_CSS_STREAM_FORMAT_YUV422_10;
}

static enum ia_css_err
check_input(struct sh_css_pipe *pipe, bool must_be_raw)
{
	assert(pipe != NULL);
	assert(pipe->stream != NULL);
	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe->stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe->stream->config.effective_res.width == 0 ||
	    pipe->stream->config.effective_res.height == 0) {
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (must_be_raw &&
	    !input_format_is_raw(pipe->stream->config.format)) {
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	return IA_CSS_SUCCESS;
}

static enum ia_css_frame_format yuv420_copy_formats[] = {
	IA_CSS_FRAME_FORMAT_NV12,
	IA_CSS_FRAME_FORMAT_NV21,
	IA_CSS_FRAME_FORMAT_YV12,
	IA_CSS_FRAME_FORMAT_YUV420,
	IA_CSS_FRAME_FORMAT_YUV420_16
};

static enum ia_css_frame_format yuv422_copy_formats[] = {
	IA_CSS_FRAME_FORMAT_NV12,
	IA_CSS_FRAME_FORMAT_NV16,
	IA_CSS_FRAME_FORMAT_NV21,
	IA_CSS_FRAME_FORMAT_NV61,
	IA_CSS_FRAME_FORMAT_YV12,
	IA_CSS_FRAME_FORMAT_YV16,
	IA_CSS_FRAME_FORMAT_YUV420,
	IA_CSS_FRAME_FORMAT_YUV420_16,
	IA_CSS_FRAME_FORMAT_YUV422,
	IA_CSS_FRAME_FORMAT_YUV422_16,
	IA_CSS_FRAME_FORMAT_UYVY,
	IA_CSS_FRAME_FORMAT_YUYV
};

#define array_length(array) (sizeof(array)/sizeof(array[0]))

/* Verify whether the selected output format is can be produced
 * by the copy binary given the stream format.
 * */
static enum ia_css_err
verify_copy_out_frame_format(struct sh_css_pipe *pipe)
{
	enum ia_css_frame_format out_fmt = pipe->output_info.format;
	unsigned int i, found = 0;	

	switch (pipe->stream->config.format) {
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
	case IA_CSS_STREAM_FORMAT_YUV420_8:
		for (i=0; i<array_length(yuv420_copy_formats) && !found; i++)
			found = (out_fmt == yuv420_copy_formats[i]);
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_10:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_YUV420_16);
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_8:
		for (i=0; i<array_length(yuv422_copy_formats) && !found; i++)
			found = (out_fmt == yuv422_copy_formats[i]);
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_10:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_YUV422_16 ||
			 out_fmt == IA_CSS_FRAME_FORMAT_YUV420_16);
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
	case IA_CSS_STREAM_FORMAT_RGB_555:
	case IA_CSS_STREAM_FORMAT_RGB_565:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RGBA888 ||
			 out_fmt == IA_CSS_FRAME_FORMAT_RGB565);
		break;
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RGB_888:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RGBA888);
		break;
	case IA_CSS_STREAM_FORMAT_RAW_6:
	case IA_CSS_STREAM_FORMAT_RAW_7:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_RAW_10:
	case IA_CSS_STREAM_FORMAT_RAW_12:
	case IA_CSS_STREAM_FORMAT_RAW_14:
	case IA_CSS_STREAM_FORMAT_RAW_16:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_RAW);
		break;
	case IA_CSS_STREAM_FORMAT_BINARY_8:
		found = (out_fmt == IA_CSS_FRAME_FORMAT_BINARY_8);
		break;
	default:
		break;
	}
	if (!found)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

/* next function takes care of getting the settings from kernel
 * commited to hmm / isp
 * TODO: see if needs to be made public
 */
static enum ia_css_err
sh_css_commit_isp_config(struct ia_css_stream *stream,
			 struct sh_css_pipeline *pipeline)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_pipeline_stage *stage;

	if (pipeline) {
		/* walk through pipeline and commit settings */
		/* TODO: check if this is needed (s3a is handled through this */
		for (stage = pipeline->stages; stage; stage = stage->next) {
			if (stage && stage->binary) {
				err = sh_css_params_write_to_ddr(stream,
								 stage->binary);
				if (err != IA_CSS_SUCCESS)
					return err;
			}
		}
	}
	return err;
}

static unsigned int
sh_css_pipe_input_format_bits_per_pixel(const struct sh_css_pipe *pipe)
{
	return sh_css_input_format_bits_per_pixel(pipe->stream->config.format,
						  pipe->stream->config.two_pixels_per_clock);
}

/* MW: Table look-up ??? */
unsigned int
sh_css_input_format_bits_per_pixel(enum ia_css_stream_format format,
	bool two_ppc)
{
	unsigned int rval = 0;
	switch (format) {
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
	case IA_CSS_STREAM_FORMAT_YUV420_8:
	case IA_CSS_STREAM_FORMAT_YUV422_8:
	case IA_CSS_STREAM_FORMAT_RGB_888:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_BINARY_8:
		rval = 8;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_10:
	case IA_CSS_STREAM_FORMAT_YUV422_10:
	case IA_CSS_STREAM_FORMAT_RAW_10:
		rval = 10;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
		rval = 4;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_555:
		rval = 5;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_565:
		rval = 65;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RAW_6:
		rval = 6;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_7:
		rval = 7;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_12:
		rval = 12;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_14:
		if (two_ppc)
			rval = 14;
		else
			rval = 12;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_16:
		if (two_ppc)
			rval = 16;
		else
			rval = 12;
		break;
	default:
		rval = 0;
		break;
		
	}
return rval;
}

unsigned int 
ia_css_stream_input_format_bits_per_pixel(struct ia_css_stream *stream)
{
	int bpp = 0;

	if (stream)
		bpp = sh_css_input_format_bits_per_pixel(stream->config.format,
						stream->config.two_pixels_per_clock);

	return bpp;
}
/* compute the log2 of the downscale factor needed to get closest
 * to the requested viewfinder resolution on the upper side. The output cannot
 * be smaller than the requested viewfinder resolution.
 */
enum ia_css_err
sh_css_vf_downscale_log2(const struct ia_css_frame_info *out_info,
			 const struct ia_css_frame_info *vf_info,
			 unsigned int *downscale_log2)
{
	unsigned int ds_log2 = 0;
	unsigned int out_width = out_info ? out_info->padded_width : 0;

	if (out_width == 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	/* downscale until width smaller than the viewfinder width. We don't
	 * test for the height since the vmem buffers only put restrictions on
	 * the width of a line, not on the number of lines in a frame.
	 */
	while (out_width >= vf_info->res.width) {
		ds_log2++;
		out_width /= 2;
	}
	/* now width is smaller, so we go up one step */
	if ((ds_log2 > 0) && (out_width < sh_css_max_vf_width()))
		ds_log2--;
	/* TODO: use actual max input resolution of vf_pp binary */
	if ((out_info->res.width >> ds_log2) >= 2*sh_css_max_vf_width())
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	*downscale_log2 = ds_log2;
	return IA_CSS_SUCCESS;
}

/* ISP expects GRBG bayer order, we skip one line and/or one row
 * to correct in case the input bayer order is different.
 */
static unsigned int
lines_needed_for_bayer_order(const struct sh_css_pipe *pipe)
{
	if (pipe->stream->config.bayer_order == IA_CSS_BAYER_ORDER_BGGR ||
	    pipe->stream->config.bayer_order == IA_CSS_BAYER_ORDER_GBRG) {
		return 1;
	}
	return 0;
}

static unsigned int
columns_needed_for_bayer_order(const struct sh_css_pipe *pipe)
{
	if (pipe->stream->config.bayer_order == IA_CSS_BAYER_ORDER_RGGB ||
	    pipe->stream->config.bayer_order == IA_CSS_BAYER_ORDER_GBRG) {
		return 1;
	}
	return 0;
}

static enum ia_css_err
input_start_column(struct sh_css_pipe *pipe,
		   unsigned int bin_in,
		   unsigned int *start_column)
{
	unsigned int in = pipe->stream->config.input_res.width,
	    for_bayer = columns_needed_for_bayer_order(pipe), start;

	if (bin_in + 2 * for_bayer > in)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	/* On the hardware, we want to use the middle of the input, so we
	 * divide the start column by 2. */
	start = (in - bin_in) / 2;
	/* in case the number of extra columns is 2 or odd, we round the start
	 * column down */
	start &= ~0x1;

	/* now we add the one column (if needed) to correct for the bayer
	 * order).
	 */
	start += for_bayer;
	*start_column = start;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
input_start_line(struct sh_css_pipe *pipe,
		 unsigned int bin_in,
		 unsigned int *start_line)
{
	unsigned int in = pipe->stream->config.input_res.height,
	    for_bayer = lines_needed_for_bayer_order(pipe), start;

	if (bin_in + 2 * for_bayer > in)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	/* On the hardware, we want to use the middle of the input, so we
	 * divide the start line by 2. On the simulator, we cannot handle extra
	 * lines at the end of the frame.
	 */
	start = (in - bin_in) / 2;
	/* in case the number of extra lines is 2 or odd, we round the start
	 * line down.
	 */
	start &= ~0x1;

	/* now we add the one line (if needed) to correct for the bayer order*/
	start += for_bayer;
	*start_line = start;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
program_input_formatter(struct sh_css_pipe *pipe,
			struct sh_css_binary *binary)
{
	unsigned int start_line, start_column = 0,
		     cropped_height = binary->in_frame_info.res.height,
		     cropped_width  = binary->in_frame_info.res.width,
		     num_vectors,
		     buffer_height = 2,
		     buffer_width = binary->info->max_input_width,
		     two_ppc = pipe->stream->config.two_pixels_per_clock,
		     vmem_increment = 0,
		     deinterleaving = 0,
		     deinterleaving_b = 0,
		     width_a = 0,
		     width_b = 0,
		     bits_per_pixel,
		     vectors_per_buffer,
		     vectors_per_line = 0,
		     buffers_per_line = 0,
		     buf_offset_b = 0,
		     line_width = 0,
		     width_b_factor = 1,
		     start_column_b,
		     left_padding = 0;
	input_formatter_cfg_t	if_a_config, if_b_config;
	enum ia_css_stream_format input_format = binary->input_format;
	enum ia_css_err err = IA_CSS_SUCCESS;
	uint8_t if_config_index;

	/* Determine which input formatter config set is targeted. */
	/* Index is equal to the CSI-2 port used. */
	enum ia_css_csi2_port port;
	
	if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_SENSOR
	 || pipe->stream->config.mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR) 
	{
		port = pipe->stream->config.source.port.port;
		if_config_index = (uint8_t) (port - IA_CSS_CSI2_PORT_4LANE);
	}
	else if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY){
		if_config_index = SH_CSS_IF_CONFIG_NOT_NEEDED;
	}
	else{
		if_config_index = 0;
	}

	assert(if_config_index <= SH_CSS_MAX_IF_CONFIGS || if_config_index == SH_CSS_IF_CONFIG_NOT_NEEDED);

	if (pipe->input_needs_raw_binning &&
	    binary->info->enable.raw_binning) {
		cropped_width *= 2;
		cropped_width -= binary->info->left_cropping;
		cropped_height *= 2;
		cropped_height -= binary->info->left_cropping;
	}

	/* TODO: check to see if input is RAW and if current mode interprets
	 * RAW data in any particular bayer order. copy binary with output
	 * format other than raw should not result in dropping lines and/or
	 * columns.
	 */
	err = input_start_line(pipe, cropped_height, &start_line);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = input_start_column(pipe, cropped_width, &start_column);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (!left_padding)
		left_padding = binary->left_padding;
	if (left_padding) {
		num_vectors = CEIL_DIV(cropped_width + left_padding,
				       ISP_VEC_NELEMS);
	} else {
		num_vectors = CEIL_DIV(cropped_width, ISP_VEC_NELEMS);
		num_vectors *= buffer_height;
		/* todo: in case of left padding,
		   num_vectors is vectors per line,
		   otherwise vectors per line * buffer_height. */
	}

	start_column_b = start_column;

	bits_per_pixel = input_formatter_get_alignment(INPUT_FORMATTER0_ID)
		*8 / ISP_VEC_NELEMS;
	switch (input_format) {
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
		if (two_ppc) {
			vmem_increment = 1;
			deinterleaving = 1;
			deinterleaving_b = 1;
			/* half lines */
			width_a = cropped_width * deinterleaving / 2;
			width_b_factor = 2;
			/* full lines */
			width_b = width_a * width_b_factor;
			buffer_width *= deinterleaving * 2;
			/* Patch from bayer to yuv */
			num_vectors *= deinterleaving;
			buf_offset_b = buffer_width / 2 / ISP_VEC_NELEMS;
			vectors_per_line = num_vectors / buffer_height;
			/* Even lines are half size */
			line_width = vectors_per_line *
				input_formatter_get_alignment(
				INPUT_FORMATTER0_ID) / 2;
			start_column /= 2;
		} else {
			vmem_increment = 1;
			deinterleaving = 3;
			width_a = cropped_width * deinterleaving / 2;
			buffer_width = buffer_width * deinterleaving / 2;
			/* Patch from bayer to yuv */
			num_vectors = num_vectors / 2 * deinterleaving;
			start_column = start_column * deinterleaving / 2;
		}
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_8:
	case IA_CSS_STREAM_FORMAT_YUV420_10:
		if (two_ppc) {
			vmem_increment = 1;
			deinterleaving = 1;
			width_a = width_b = cropped_width * deinterleaving / 2;
			buffer_width *= deinterleaving * 2;
			num_vectors *= deinterleaving;
			buf_offset_b = buffer_width / 2 / ISP_VEC_NELEMS;
			vectors_per_line = num_vectors / buffer_height;
			/* Even lines are half size */
			line_width = vectors_per_line *
				input_formatter_get_alignment(
				INPUT_FORMATTER0_ID) / 2;
			start_column *= deinterleaving;
			start_column /= 2;
			start_column_b = start_column;
		} else {
			vmem_increment = 1;
			deinterleaving = 1;
			width_a = cropped_width * deinterleaving;
			buffer_width  *= deinterleaving * 2;
			num_vectors  *= deinterleaving;
			start_column *= deinterleaving;
		}
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_8:
	case IA_CSS_STREAM_FORMAT_YUV422_10:
		if (two_ppc) {
			vmem_increment = 1;
			deinterleaving = 1;
			width_a = width_b = cropped_width * deinterleaving;
			buffer_width *= deinterleaving * 2;
			num_vectors  *= deinterleaving;
			start_column *= deinterleaving;
			buf_offset_b   = buffer_width / 2 / ISP_VEC_NELEMS;
			start_column_b = start_column;
		} else {
			vmem_increment = 1;
			deinterleaving = 2;
			width_a = cropped_width * deinterleaving;
			buffer_width *= deinterleaving;
			num_vectors  *= deinterleaving;
			start_column *= deinterleaving;
		}
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
	case IA_CSS_STREAM_FORMAT_RGB_555:
	case IA_CSS_STREAM_FORMAT_RGB_565:
	case IA_CSS_STREAM_FORMAT_RGB_666:
	case IA_CSS_STREAM_FORMAT_RGB_888:
		num_vectors *= 2;
		if (two_ppc) {
			deinterleaving = 2;	/* BR in if_a, G in if_b */
			deinterleaving_b = 1;	/* BR in if_a, G in if_b */
			buffers_per_line = 4;
			start_column_b = start_column;
			start_column *= deinterleaving;
			start_column_b *= deinterleaving_b;
		} else {
			deinterleaving = 3;	/* BGR */
			buffers_per_line = 3;
			start_column *= deinterleaving;
		}
		vmem_increment = 1;
		width_a = cropped_width * deinterleaving;
		width_b = cropped_width * deinterleaving_b;
		buffer_width *= buffers_per_line;
		/* Patch from bayer to rgb */
		num_vectors = num_vectors / 2 * deinterleaving;
		buf_offset_b = buffer_width / 2 / ISP_VEC_NELEMS;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_6:
	case IA_CSS_STREAM_FORMAT_RAW_7:
	case IA_CSS_STREAM_FORMAT_RAW_8:
	case IA_CSS_STREAM_FORMAT_RAW_10:
	case IA_CSS_STREAM_FORMAT_RAW_12:
		if (two_ppc) {
			vmem_increment = 2;
			deinterleaving = 1;
			width_a = width_b = cropped_width / 2;
			start_column /= 2;
			start_column_b = start_column;
			buf_offset_b = 1;
		} else {
			vmem_increment = 1;
			deinterleaving = 2;
			if (pipe->stream->config.continuous &&
			    binary->info->mode == SH_CSS_BINARY_MODE_COPY) {
				/* No deinterleaving for sp copy */
				deinterleaving = 1;
			}
			width_a = cropped_width;
			/* Must be multiple of deinterleaving */
			num_vectors = CEIL_MUL(num_vectors, deinterleaving);
		}
		buffer_height *= 2;
		if (pipe->stream->config.continuous)
			buffer_height *= 2;
		vectors_per_line = CEIL_DIV(cropped_width, ISP_VEC_NELEMS);
		vectors_per_line = CEIL_MUL(vectors_per_line, deinterleaving);
		break;
	case IA_CSS_STREAM_FORMAT_RAW_14:
	case IA_CSS_STREAM_FORMAT_RAW_16:
		if (two_ppc) {
			num_vectors *= 2;
			vmem_increment = 1;
			deinterleaving = 2;
			width_a = width_b = cropped_width;
			/* B buffer is one line further */
			buf_offset_b = buffer_width / ISP_VEC_NELEMS;
			bits_per_pixel *= 2;
		} else {
			vmem_increment = 1;
			deinterleaving = 2;
			width_a = cropped_width;
			start_column /= deinterleaving;
		}
		buffer_height *= 2;
		break;
	case IA_CSS_STREAM_FORMAT_BINARY_8:
		break;
	}
	if (width_a == 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (two_ppc)
		left_padding /= 2;

	/* Default values */
	if (left_padding)
		vectors_per_line = num_vectors;
	if (!vectors_per_line) {
		vectors_per_line = CEIL_MUL(num_vectors / buffer_height,
					    deinterleaving);
		line_width = 0;
	}
	if (!line_width)
		line_width = vectors_per_line *
		input_formatter_get_alignment(INPUT_FORMATTER0_ID);
	if (!buffers_per_line)
		buffers_per_line = deinterleaving;
	line_width = CEIL_MUL(line_width,
		input_formatter_get_alignment(INPUT_FORMATTER0_ID)
		* vmem_increment);

	vectors_per_buffer = buffer_height * buffer_width / ISP_VEC_NELEMS;

	if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_TPG &&
	    binary->info->mode == SH_CSS_BINARY_MODE_VIDEO) {
		/* workaround for TPG in video mode*/
		start_line = 0;
		start_column = 0;
		cropped_height -= start_line;
		width_a -= start_column;
	}

	if_a_config.start_line = start_line;
	if_a_config.start_column = start_column;
	if_a_config.left_padding = left_padding / deinterleaving;
	if_a_config.cropped_height = cropped_height;
	if_a_config.cropped_width = width_a;
	if_a_config.deinterleaving = deinterleaving;
	if_a_config.buf_vecs = vectors_per_buffer;
	if_a_config.buf_start_index = 0;
	if_a_config.buf_increment = vmem_increment;
	if_a_config.buf_eol_offset =
	    buffer_width * bits_per_pixel / 8 - line_width;
	if_a_config.is_yuv420_format =
		(input_format == IA_CSS_STREAM_FORMAT_YUV420_8)
		|| (input_format == IA_CSS_STREAM_FORMAT_YUV420_10);
	if_a_config.block_no_reqs =
		(pipe->stream->config.mode != IA_CSS_INPUT_MODE_SENSOR);

	if (two_ppc) {
		if (deinterleaving_b) {
			deinterleaving = deinterleaving_b;
			width_b = cropped_width * deinterleaving;
			buffer_width *= deinterleaving;
			/* Patch from bayer to rgb */
			num_vectors = num_vectors / 2 *
					deinterleaving * width_b_factor;
			vectors_per_line = num_vectors / buffer_height;
			line_width = vectors_per_line *
				input_formatter_get_alignment(
				INPUT_FORMATTER0_ID);
		}
		if_b_config.start_line = start_line;
		if_b_config.start_column = start_column_b;
		if_b_config.left_padding = left_padding / deinterleaving;
		if_b_config.cropped_height = cropped_height;
		if_b_config.cropped_width = width_b;
		if_b_config.deinterleaving = deinterleaving;
		if_b_config.buf_vecs = vectors_per_buffer;
		if_b_config.buf_start_index = buf_offset_b;
		if_b_config.buf_increment = vmem_increment;
		if_b_config.buf_eol_offset =
		    buffer_width * bits_per_pixel/8 - line_width;
		if_b_config.is_yuv420_format =
		    input_format == IA_CSS_STREAM_FORMAT_YUV420_8
		    || input_format == IA_CSS_STREAM_FORMAT_YUV420_10;
		if_b_config.block_no_reqs =
			(pipe->stream->config.mode != IA_CSS_INPUT_MODE_SENSOR);
		sh_css_sp_set_if_configs(&if_a_config, &if_b_config, if_config_index);
	} else {
		sh_css_sp_set_if_configs(&if_a_config, NULL, if_config_index);
	}
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_config_input_network(struct sh_css_pipe *pipe,
			    struct sh_css_binary *binary)
{
	unsigned int fmt_type;
	enum ia_css_err err = IA_CSS_SUCCESS;

	if (pipe && pipe->pipeline.stages)
		binary = pipe->pipeline.stages->binary;

	err = sh_css_input_format_type(pipe->stream->config.format,
				       pipe->stream->csi_rx_config.comp,
				       &fmt_type);
	if (err != IA_CSS_SUCCESS)
		return err;
	sh_css_sp_program_input_circuit(fmt_type,
					pipe->stream->config.channel_id,
					pipe->stream->config.mode);

	if (binary && (binary->online || pipe->stream->config.continuous)) {
		if (pipe->stream->config.continuous)
			my_css.start_sp_copy = true;
		err = program_input_formatter(pipe, binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_TPG ||
	    pipe->stream->config.mode == IA_CSS_INPUT_MODE_PRBS) {
		unsigned int hblank_cycles = 100,
			     vblank_lines = 6,
			     width,
			     height,
			     vblank_cycles;
		width  = (pipe->stream->config.input_res.width) / (1 + (pipe->stream->config.two_pixels_per_clock != 0));
		height = pipe->stream->config.input_res.height;
		vblank_cycles = vblank_lines * (width + hblank_cycles);
		sh_css_sp_configure_sync_gen(width, height, hblank_cycles,
					     vblank_cycles);
#if defined(IS_ISP_2400_SYSTEM)
		if (pipe->stream->config.mode == IA_CSS_INPUT_MODE_TPG) {
			/* TODO: move define to proper file in tools */
			#define GP_ISEL_TPG_MODE 0x90058
			device_store_uint32(GP_ISEL_TPG_MODE, 2);
		}
#endif
	}
	return IA_CSS_SUCCESS;
}

#if WITH_PC_MONITORING
static struct task_struct *my_kthread;    /* Handle for the monitoring thread */
static int sh_binary_running;         /* Enable sampling in the thread */

static void print_pc_histo(char *core_name, struct sh_css_pc_histogram *hist)
{
	unsigned i;
	unsigned cnt_run = 0;
	unsigned cnt_stall = 0;
	sh_css_print("%s histogram length = %d\n", core_name, hist->length);
	sh_css_print("%s PC\trun\tstall\n", core_name);

	for (i = 0; i < hist->length; i++) {
		if ((hist->run[i] == 0) && (hist->run[i] == hist->stall[i]))
			continue;
		sh_css_print("%s %d\t%d\t%d\n",
				core_name, i, hist->run[i], hist->stall[i]);
		cnt_run += hist->run[i];
		cnt_stall += hist->stall[i];
	}

	sh_css_print(" Statistics for %s, cnt_run = %d, cnt_stall = %d, "
	       "hist->length = %d\n",
			core_name, cnt_run, cnt_stall, hist->length);
}

static void print_pc_histogram(void)
{
	struct sh_css_binary_metrics *metrics;

	for (metrics = sh_css_metrics.binary_metrics;
	     metrics;
	     metrics = metrics->next) {
		if (metrics->mode == SH_CSS_BINARY_MODE_PREVIEW ||
		    metrics->mode == SH_CSS_BINARY_MODE_VF_PP) {
			sh_css_print("pc_histogram for binary %d is SKIPPED\n",
				metrics->id);
			continue;
		}

		sh_css_print(" pc_histogram for binary %d\n", metrics->id);
		print_pc_histo("  ISP", &metrics->isp_histogram);
		print_pc_histo("  SP",   &metrics->sp_histogram);
		sh_css_print("print_pc_histogram() done for binay->id = %d, "
			     "done.\n", metrics->id);
	}

	sh_css_print("PC_MONITORING:print_pc_histogram() -- DONE\n");
}

static int pc_monitoring(void *data)
{
	int i = 0;

	while (true) {
		if (sh_binary_running) {
			sh_css_metrics_sample_pcs();
#if MULTIPLE_SAMPLES
			for (i = 0; i < NOF_SAMPLES; i++)
				sh_css_metrics_sample_pcs();
#endif
		}
		usleep_range(10, 50);
	}
	return 0;
}

static void spying_thread_create(void)
{
	my_kthread = kthread_run(pc_monitoring, NULL, "sh_pc_monitor");
	sh_css_metrics_enable_pc_histogram(1);
}

static void input_frame_info(struct ia_css_frame_info frame_info)
{
	sh_css_print("SH_CSS:input_frame_info() -- frame->info.res.width = %d, "
	       "frame->info.res.height = %d, format = %d\n",
			frame_info.res.width, frame_info.res.height, frame_info.format);
}
#endif /* WITH_PC_MONITORING */

static void
start_binary(struct sh_css_pipe *pipe,
	     struct sh_css_binary *binary)
{
	struct ia_css_stream *stream = pipe->stream;

#ifdef THIS_CODE_IS_NO_LONGER_NEEDED_FOR_DUAL_STREAM
    if (stream && stream->reconfigure_css_rx)
		sh_css_rx_disable();
#endif

	sh_css_metrics_start_binary(&binary->metrics);

#if WITH_PC_MONITORING
	sh_css_print("PC_MONITORING: %s() -- binary id = %d , "
		     "enable_dvs_envelope = %d\n",
		     __func__, binary->info->id,
		     binary->info->enable.dvs_envelope);
	input_frame_info(binary->in_frame_info);

	if (binary->info->mode == SH_CSS_BINARY_MODE_VIDEO)
		sh_binary_running = true;
#endif

	//sh_css_sp_start_isp();

	if (stream && stream->reconfigure_css_rx) {
		sh_css_rx_configure(&pipe->stream->csi_rx_config, pipe->stream->config.mode);
		stream->reconfigure_css_rx = false;
	}
}

void
ia_css_frame_zero(struct ia_css_frame *frame)
{
	mmgr_clear(frame->data, frame->data_bytes);
}

/* start the copy function on the SP */
static enum ia_css_err
start_copy_on_sp(struct sh_css_pipe *pipe,
		 struct ia_css_frame *out_frame)
{
	struct ia_css_stream *stream = pipe->stream;

	if (stream && stream->reconfigure_css_rx)
		sh_css_rx_disable();

	if (pipe->stream->config.format != IA_CSS_STREAM_FORMAT_BINARY_8)
		return IA_CSS_ERR_INTERNAL_ERROR;
	sh_css_sp_start_binary_copy(pipe->pipe_num, out_frame, pipe->stream->config.two_pixels_per_clock);

	//sh_css_sp_start_isp();

	if (stream && stream->reconfigure_css_rx) {
		sh_css_rx_configure(&pipe->stream->csi_rx_config, pipe->stream->config.mode);
		stream->reconfigure_css_rx = false;
	}

	return IA_CSS_SUCCESS;
}

/* Pipeline:
 * To organize the several different binaries for each type of mode,
 * we use a pipeline. A pipeline contains a number of stages, each with
 * their own binary and frame pointers.
 * When stages are added to a pipeline, output frames that are not passed
 * from outside are automatically allocated.
 * When input frames are not passed from outside, each stage will use the
 * output frame of the previous stage as input (the full resolution output,
 * not the viewfinder output).
 * Pipelines must be cleaned and re-created when settings of the binaries
 * change.
 */
static void
sh_css_pipeline_stage_destroy(struct sh_css_pipeline_stage *me)
{
	if (me->out_frame_allocated) {
		ia_css_frame_free(me->args.out_frame);
		me->args.out_frame = NULL;
	}
	if (me->vf_frame_allocated) {
		ia_css_frame_free(me->args.out_vf_frame);
		me->args.out_vf_frame = NULL;
	}
	sh_css_free(me);
}

static void
sh_css_binary_args_reset(struct sh_css_binary_args *args)
{
	args->in_frame      = NULL;
	args->out_frame     = NULL;
	args->in_ref_frame  = NULL;
	args->out_ref_frame = NULL;
	args->in_tnr_frame  = NULL;
	args->out_tnr_frame = NULL;
	args->extra_frame   = NULL;
	args->out_vf_frame  = NULL;
	args->copy_vf       = false;
	args->copy_output   = true;
	args->vf_downscale_log2 = 0;
}

static enum ia_css_err
sh_css_pipeline_stage_create(struct sh_css_pipeline_stage **me,
			     struct sh_css_binary *binary,
			     const struct ia_css_fw_info *firmware,
			     enum sh_css_sp_stage_func    sp_func,
			     unsigned max_input_width,
			     int    mode,
			     struct ia_css_frame *cc_frame,
			     struct ia_css_frame *in_frame,
			     struct ia_css_frame *out_frame,
			     struct ia_css_frame *vf_frame)
{
	struct sh_css_pipeline_stage *stage = sh_css_malloc(sizeof(*stage));
	if (!stage)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	stage->binary = firmware ? NULL : binary;
	stage->binary_info = firmware ?
				(struct ia_css_binary_info *)
				&firmware->info.isp :
			     binary ?
				(struct ia_css_binary_info *)binary->info :
			     NULL;
	stage->firmware = firmware;
	stage->sp_func  = sp_func;
	stage->max_input_width  = max_input_width;
	stage->mode = mode;
	stage->out_frame_allocated = false;
	stage->vf_frame_allocated = false;
	stage->irq_buf_flags = 0x0;
	stage->next = NULL;
	sh_css_binary_args_reset(&stage->args);

	if (!in_frame && !firmware && binary && !binary->online)
		return IA_CSS_ERR_INTERNAL_ERROR;

	if (!out_frame && binary && binary->out_frame_info.res.width) {
		enum ia_css_err ret =
		    ia_css_frame_allocate_from_info(&out_frame,
						    &binary->out_frame_info);
		if (ret != IA_CSS_SUCCESS) {
			sh_css_free(stage);
			return ret;
		}
		stage->out_frame_allocated = true;
	}
	/* VF frame is not needed in case of need_pp
	   However, the capture binary needs a vf frame to write to.
	*/
	if (!vf_frame) {
		if ((binary && binary->vf_frame_info.res.width) ||
		    (firmware &&
		     firmware->info.isp.enable.vf_veceven)
		    ) {
			enum ia_css_err ret =
			    ia_css_frame_allocate_from_info(&vf_frame,
						    &binary->vf_frame_info);
			if (ret != IA_CSS_SUCCESS) {
				if (stage->out_frame_allocated) {
					ia_css_frame_free(out_frame);
					out_frame = NULL;
				}
				sh_css_free(stage);
				return ret;
			}
			stage->vf_frame_allocated = true;
		}
	} else if (vf_frame && binary && binary->vf_frame_info.res.width)
		stage->vf_frame_allocated = true;

	stage->args.cc_frame = cc_frame;
	stage->args.in_frame = in_frame;
	stage->args.out_frame = out_frame;
	stage->args.out_vf_frame = vf_frame;
	*me = stage;
	return IA_CSS_SUCCESS;
}

static void
sh_css_pipeline_init(struct sh_css_pipeline *me, enum ia_css_pipe_id pipe_id)
{
	struct ia_css_frame init_frame;

	assert(me != NULL);
	init_frame.dynamic_data_index = SH_CSS_INVALID_FRAME_ID;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipeline_init() enter:\n");
	me->pipe_id = pipe_id;
	me->stages = NULL;
	me->reload = true;
	me->current_stage = NULL;
	me->in_frame = init_frame;
	me->out_frame = init_frame;
	me->vf_frame = init_frame;
}

/** @brief Add a stage to pipeline.
 *
 * @param	me	Pointer to the pipeline to be added to.
 * @param[in]	binary		ISP binary of new stage.
 * @param[in]	firmware	ISP firmware of new stage.
 * @param[in]	mode		ISP mode of new stage.
 * @param[in]	cc_frame		The cc frame to the stage.
 * @param[in]	in_frame		The input frame to the stage.
 * @param[in]	out_frame		The output frame of the stage.
 * @param[in]	vf_frame		The viewfinder frame of the stage.
 * @param[in]	stage			The successor of the stage.
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * Add a new stage to a non-NULL pipeline.
 * The stage consists of an ISP binary or firmware and input and output arguments.
*/
static enum ia_css_err
sh_css_pipeline_add_stage(struct sh_css_pipeline *me,
			  struct sh_css_binary *binary,
			  const struct ia_css_fw_info *firmware,
			  unsigned int mode,
			  struct ia_css_frame *cc_frame,
			  struct ia_css_frame *in_frame,
			  struct ia_css_frame *out_frame,
			  struct ia_css_frame *vf_frame,
			  struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *last = me->stages, *new_stage = NULL;
	enum ia_css_err err;

/* other arguments can be NULL */
assert(me != NULL);
/* assert(stage != NULL); */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_add_stage() enter:\n");
	if (!binary && !firmware)
		return IA_CSS_ERR_INTERNAL_ERROR;

	while (last && last->next)
		last = last->next;

	/* if in_frame is not set, we use the out_frame from the previous
	 * stage, if no previous stage, it's an error.
	 */
	if (!in_frame && !firmware && !binary->online) {
		if (last)
			in_frame = last->args.out_frame;
		if (!in_frame)
			return IA_CSS_ERR_INTERNAL_ERROR;
	}
	err = sh_css_pipeline_stage_create(&new_stage, binary, firmware,
					   SH_CSS_SP_NO_FUNC, 0, mode, cc_frame,
					   in_frame, out_frame, vf_frame);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (last)
		last->next = new_stage;
	else
		me->stages = new_stage;
	if (stage)
		*stage = new_stage;
	return IA_CSS_SUCCESS;
}

/** @brief Add a stage to pipeline.
 *
 * @param	me	Pointer to the pipeline to be added to.
 * @param[in]	out_frame		The output frame of the stage.
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * Add a new stage to a non-NULL pipeline.
 * The stage consists of an SP function and input and output arguments.
*/
static enum ia_css_err
sh_css_pipeline_add_sp_stage(struct sh_css_pipeline *me,
			  enum sh_css_sp_stage_func func,
			  unsigned max_input_width,
			  struct ia_css_frame *out_frame)
{
	struct sh_css_pipeline_stage *last = me->stages, *new_stage = NULL;
	enum ia_css_err err;

/* other arguments can be NULL */
assert(me != NULL);
/* assert(stage != NULL); */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_add_sp_stage() enter:\n");

	while (last && last->next)
		last = last->next;

	err = sh_css_pipeline_stage_create(&new_stage, NULL, NULL,
					   func, max_input_width, -1, NULL,
					   NULL, out_frame, NULL);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (last)
		last->next = new_stage;
	else
		me->stages = new_stage;
	
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_pipeline_get_stage(struct sh_css_pipeline *me,
			  int mode,
			  struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *s;
assert(me != NULL);
assert(stage != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_get_stage() enter:\n");
	for (s = me->stages; s; s = s->next) {
		if (s->mode == mode) {
			*stage = s;
			return IA_CSS_SUCCESS;
		}
	}
	return IA_CSS_ERR_INTERNAL_ERROR;
}

static enum ia_css_err
sh_css_pipeline_get_output_stage(struct sh_css_pipeline *me,
				 int mode,
				 struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *s;
assert(me != NULL);
assert(stage != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_get_output_stage() enter:\n");

	*stage = NULL;
	/* First find acceleration firmware at end of pipe */
	for (s = me->stages; s; s = s->next) {
		if (s->firmware && s->mode == mode &&
		    s->firmware->info.isp.enable.output)
			*stage = s;
	}
	if (*stage)
		return IA_CSS_SUCCESS;
	/* If no firmware, find binary in pipe */
	return sh_css_pipeline_get_stage(me, mode, stage);
}

static void
sh_css_pipeline_restart(struct sh_css_pipeline *me)
{
assert(me != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_restart() enter:\n");
	me->current_stage = NULL;
}

static void
sh_css_pipeline_clean(struct sh_css_pipeline *me)
{
	struct sh_css_pipeline_stage *s = me->stages;
assert(me != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipeline_clean() enter:\n");

	while (s) {
		struct sh_css_pipeline_stage *next = s->next;
		sh_css_pipeline_stage_destroy(s);
		s = next;
	}
	sh_css_pipeline_init(me, me->pipe_id);
}

static void
pipe_start(struct sh_css_pipe *pipe)
{
	struct sh_css_pipeline_stage *stage = pipe->pipeline.stages;
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"pipe_start() enter:\n");
	if (!stage)
		return;
	pipe->pipeline.current_stage = stage;

	start_binary(pipe, stage->binary);
}

static void start_pipe(
	struct sh_css_pipe *me,
	enum sh_css_pipe_config_override copy_ovrd,
	enum ia_css_input_mode input_mode)
{
	bool low_light = me->mode == IA_CSS_PIPE_ID_CAPTURE &&
			 (me->capture_mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT ||
			  me->capture_mode == IA_CSS_CAPTURE_MODE_BAYER);
	bool is_preview = me->mode == IA_CSS_PIPE_ID_PREVIEW;
	assert(me != NULL);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"start_pipe() enter:\n");

	sh_css_sp_init_pipeline(&me->pipeline,
				me->mode,
				me->pipe_num,
				is_preview,
				low_light,
				me->xnr,
				me->stream->config.two_pixels_per_clock,
				me->stream->config.continuous,
				false,
				me->input_needs_raw_binning,
				copy_ovrd,
				input_mode,
				(input_mode==IA_CSS_INPUT_MODE_MEMORY)?
					(mipi_port_ID_t)0:
				me->stream->config.source.port.port);

	/* prepare update of params to ddr */
	sh_css_commit_isp_config(me->stream, &me->pipeline);

	pipe_start(me);
}

static void
sh_css_set_irq_buffer(struct sh_css_pipeline_stage *stage,
			enum sh_css_frame_id frame_id,
			struct ia_css_frame *frame)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_set_irq_buffer() enter:\n");
	if (stage && frame)
		stage->irq_buf_flags |= 1<<frame_id;
}

void sh_css_frame_info_set_width(
	struct ia_css_frame_info *info,
	unsigned int width)
{
assert(info != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_frame_info_set_width() enter: "
		"width=%d\n",
		width);
	info->res.width = width;
	/* frames with a U and V plane of 8 bits per pixel need to have
	   all planes aligned, this means double the alignment for the
	   Y plane if the horizontal decimation is 2. */
	if (info->format == IA_CSS_FRAME_FORMAT_YUV420 ||
	    info->format == IA_CSS_FRAME_FORMAT_YV12)
		info->padded_width = CEIL_MUL(width, 2*HIVE_ISP_DDR_WORD_BYTES);
	else if (info->format == IA_CSS_FRAME_FORMAT_YUV_LINE)
		info->padded_width = CEIL_MUL(width, 2*ISP_VEC_NELEMS);
	else if (info->format == IA_CSS_FRAME_FORMAT_RAW)
		info->padded_width = CEIL_MUL(width, 2*ISP_VEC_NELEMS);
	else
		info->padded_width = CEIL_MUL(width, HIVE_ISP_DDR_WORD_BYTES);
}

static void sh_css_frame_info_set_format(
	struct ia_css_frame_info *info,
	enum ia_css_frame_format format)
{
assert(info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_frame_info_set_format() enter:\n");
	/* yuv_line has 2*NWAY alignment */
	info->format = format;
	/* HACK: this resets the padded width incorrectly.
	   Lex needs to fix this in the vf_veceven module. */
	info->padded_width =  CEIL_MUL(info->padded_width, 2*ISP_VEC_NELEMS);
}

void sh_css_frame_info_init(
	struct ia_css_frame_info *info,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
assert(info != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_frame_info_init() enter: "
		"width=%d, "
		"height=%d, "
		"format=%d\n",
		width, height,
		format);
	info->res.height = height;
	info->format = format;
	sh_css_frame_info_set_width(info, width);
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_frame_info_init() leave: return_void\n");
}

static void invalidate_video_binaries(
	struct sh_css_pipe *pipe)
{
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"invalidate_video_binaries() enter:\n");
	pipe->pipeline.reload   = true;
	pipe->pipe.video.copy_binary.info = NULL;
	pipe->pipe.video.video_binary.info = NULL;
	pipe->pipe.video.vf_pp_binary.info = NULL;
	if (pipe->shading_table) {
		ia_css_shading_table_free(pipe->shading_table);
		pipe->shading_table = NULL;
	}
}

void
sh_css_invalidate_shading_tables(struct ia_css_stream *stream)
{
	int i;
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_invalidate_shading_tables() enter:\n");

	for (i=0; i<stream->num_pipes; i++) {
		struct sh_css_pipe *old_pipe = stream->pipes[i]->old_pipe;
		sh_css_pipe_free_shading_table(old_pipe);
	}

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_invalidate_shading_tables() leave: return_void\n");
}

/* CSS receiver programming */
/* currently, the capture pp binary requires an internal frame. This will
   be removed in the future. */
static enum ia_css_err alloc_capture_pp_frame(
	struct sh_css_pipe *pipe,
	const struct sh_css_binary *binary)
{
	struct ia_css_frame_info cpp_info;
	enum ia_css_err err = IA_CSS_SUCCESS;
assert(pipe != NULL);
assert(binary != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "alloc_capture_pp_frame() enter:\n");

	cpp_info = binary->internal_frame_info;
	cpp_info.format = IA_CSS_FRAME_FORMAT_YUV420;
	if (pipe->pipe.capture.capture_pp_frame) {
		ia_css_frame_free(pipe->pipe.capture.capture_pp_frame);
		pipe->pipe.capture.capture_pp_frame = NULL;
	}
	err = ia_css_frame_allocate_from_info(
			&pipe->pipe.capture.capture_pp_frame, &cpp_info);
	return err;
}

static void invalidate_preview_binaries(
	struct sh_css_pipe *pipe)
{
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "invalidate_preview_binaries() enter:\n");
	pipe->pipeline.reload     = true;
	pipe->pipe.preview.preview_binary.info = NULL;
	pipe->pipe.preview.vf_pp_binary.info   = NULL;
	pipe->pipe.preview.copy_binary.info    = NULL;
	if (pipe->shading_table) {
		ia_css_shading_table_free(pipe->shading_table);
		pipe->shading_table = NULL;
	}
}

static void invalidate_capture_binaries(
	struct sh_css_pipe *pipe)
{
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "invalidate_capture_binaries() enter:\n");
	pipe->pipeline.reload        = true;
	pipe->pipe.capture.copy_binary.info       = NULL;
	pipe->pipe.capture.primary_binary.info    = NULL;
	pipe->pipe.capture.pre_isp_binary.info    = NULL;
	pipe->pipe.capture.gdc_binary.info        = NULL;
	pipe->pipe.capture.post_isp_binary.info   = NULL;
	pipe->pipe.capture.pre_anr_binary.info    = NULL;
	pipe->pipe.capture.anr_binary.info        = NULL;
	pipe->pipe.capture.post_anr_binary.info   = NULL;
	pipe->pipe.capture.capture_pp_binary.info = NULL;
	pipe->pipe.capture.vf_pp_binary.info      = NULL;
	if (pipe->shading_table) {
		ia_css_shading_table_free(pipe->shading_table);
		pipe->shading_table = NULL;
	}
}

static void sh_css_pipe_invalidate_binaries(
	struct sh_css_pipe *pipe)
{
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_invalidate_binaries() enter:\n");
	switch (pipe->mode) {
	case IA_CSS_PIPE_ID_VIDEO:
		invalidate_video_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		invalidate_capture_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_PREVIEW:
		invalidate_preview_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_COPY:
		return;
	default:
		break;
	}
	/* Temporarily, not every sh_css_pipe has a new_pipe. */
	if (pipe->new_pipe && pipe->new_pipe->config.acc_extension) {
		ia_css_pipe_unload_extension(pipe->new_pipe,
				pipe->new_pipe->config.acc_extension);
	}
}

static void
enable_interrupts(enum ia_css_irq_type irq_type)
{
	bool enable_pulse = irq_type != IA_CSS_IRQ_TYPE_EDGE;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "enable_interrupts() enter:\n");
/* Enable IRQ on the SP which signals that SP goes to idle (aka ready state) */
	cnd_sp_irq_enable(SP0_ID, true);
/* Set the IRQ device 0 to either level or pulse */
	irq_enable_pulse(IRQ0_ID, enable_pulse);
	cnd_virq_enable_channel(virq_sp, true);
	/* Triggered by SP to signal Host that there are new statistics */
	cnd_virq_enable_channel((virq_id_t)(IRQ_SW_CHANNEL1_ID + IRQ_SW_CHANNEL_OFFSET), true);
	/* Triggered by SP to signal Host that there is data in one of the
	 * SP->Host queues.*/
#if !defined(HAS_IRQ_MAP_VERSION_2)
/* IRQ_SW_CHANNEL2_ID does not exist on 240x systems */
	cnd_virq_enable_channel((virq_id_t)(IRQ_SW_CHANNEL2_ID + IRQ_SW_CHANNEL_OFFSET), true);
	virq_clear_all();
#endif

	sh_css_rx_enable_all_interrupts();

#if defined(HRT_CSIM)
/*
 * Enable IRQ on the SP which signals that SP goes to idle 
 * to get statistics for each binary
 */
	cnd_isp_irq_enable(ISP0_ID, true);
	cnd_virq_enable_channel(virq_isp, true);
#endif
}

enum ia_css_err
ia_css_init(const struct ia_css_env *env,
	    const struct ia_css_fw  *fw,
	    uint32_t                 mmu_l1_base,
	    enum ia_css_irq_type     irq_type)
{
	enum ia_css_err err;
	//uint32_t i = 0;
	void *(*malloc_func) (size_t size, bool zero_mem) = env->cpu_mem_env.alloc;
	void (*free_func) (void *ptr) = env->cpu_mem_env.free;
	void (*flush_func) (struct ia_css_acc_fw *fw) = env->cpu_mem_env.flush;
	static struct sh_css default_css = DEFAULT_CSS;
	hrt_data select, enable;

	init_pipe_number();

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init() enter: void\n");

	ia_css_device_access_init(&env->hw_access_env);
	ia_css_memory_access_init(&env->css_mem_env);
	select = gpio_reg_load(GPIO0_ID, _gpio_block_reg_do_select)
						& (~GPIO_FLASH_PIN_MASK);
	enable = gpio_reg_load(GPIO0_ID, _gpio_block_reg_do_e)
							| GPIO_FLASH_PIN_MASK;
	sh_css_mmu_set_page_table_base_index(mmu_l1_base);

	if (malloc_func == NULL || free_func == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	memcpy(&my_css, &default_css, sizeof(my_css));

	my_css.malloc = malloc_func;
	my_css.free = free_func;
	my_css.flush = flush_func;
	sh_css_printf = env->print_env.debug_print;

	ia_css_i_host_rmgr_init();

	sh_css_set_dtrace_level(9);
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init()\n");

	ref_count_mipi_allocation = 0;
	/* In case this has been programmed already, update internal
	   data structure ... DEPRECATED */
	my_css.page_table_base_index = mmu_get_page_table_base_index(MMU0_ID);

	my_css.irq_type = irq_type;
	enable_interrupts(my_css.irq_type);

	/* configure GPIO to output mode */
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_select, select);
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_e, enable);
	gpio_reg_store(GPIO0_ID, _gpio_block_reg_do_0, 0);

	err = sh_css_refcount_init();
	if (err != IA_CSS_SUCCESS)
		return err;
	err = sh_css_params_init();
	if (err != IA_CSS_SUCCESS)
		return err;
	err = sh_css_sp_init();
	if (err != IA_CSS_SUCCESS)
		return err;
	err = sh_css_load_firmware(fw->data, fw->bytes);
	if (err != IA_CSS_SUCCESS)
		return err;
	sh_css_init_binary_infos();
	my_css.sp_bin_addr = sh_css_sp_load_program(&sh_css_sp_fw,
						    SP_PROG_NAME,
						    my_css.sp_bin_addr);
	if (!my_css.sp_bin_addr) {
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init() leave: return_err=%d\n",IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

#if defined(HRT_CSIM)
	/**
	 * In compiled simulator context include debug support by default.
	 * In all other cases (e.g. Android phone), the user (e.g. driver)
	 * must explicitly enable debug support by calling this function.
	 */
	if (!sh_css_debug_mode_init()) {
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init() leave: return_err=%d\n",IA_CSS_ERR_INTERNAL_ERROR);
		return IA_CSS_ERR_INTERNAL_ERROR;
	}
#endif

#if WITH_PC_MONITORING
	if (!thread_alive) {
		thread_alive++;
		sh_css_print("PC_MONITORING: %s() -- create thread DISABLED\n",
			     __func__);
		spying_thread_create();
	}
	sh_css_printf = printk;
#endif
	if (!sh_css_hrt_system_is_idle()) {
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init() leave: return_err=%d\n",IA_CSS_ERR_SYSTEM_NOT_IDLE);
		return IA_CSS_ERR_SYSTEM_NOT_IDLE;
	}
	/* can be called here, queuing works, but:
	   - when sp is started later, it will wipe queued items
	   so for now we leave it for later and make sure
	   updates are not called to frequently.
	sh_css_init_buffer_queues();
	*/

    //ia_css_stream_manager_init();

#ifdef HAS_INPUT_SYSTEM_VERSION_2
	if(ia_css_input_system_init() != INPUT_SYSTEM_ERR_NO_ERROR)
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
#endif
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init() leave: return_err=%d\n",err);

	return err;
}

/* Suspend does not need to do anything for now, this may change
   in the future though. */
void
ia_css_suspend(void)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_suspend() enter & leave\n");
}

void
ia_css_resume(void)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_resume() enter: void\n");

	sh_css_sp_set_sp_running(false);
	/* reload the SP binary. ISP binaries are automatically
	   reloaded by the ISP upon execution. */
	sh_css_mmu_set_page_table_base_index(my_css.page_table_base_index);
	sh_css_params_reconfigure_gdc_lut();

	sh_css_sp_activate_program(&sh_css_sp_fw, my_css.sp_bin_addr,
				   SP_PROG_NAME);

	enable_interrupts(my_css.irq_type);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_resume() leave: return_void\n");
}

void *
sh_css_malloc(size_t size)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_malloc() enter: size=%d\n",size);
	if (size > 0 && my_css.malloc)
		return my_css.malloc(size, false);
	return NULL;
}

void *
sh_css_calloc(size_t N, size_t size)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_calloc() enter: N=%d, size=%d\n",N,size);
	if (size > 0 && my_css.malloc) {
		return my_css.malloc(N*size, true);
	}
	return NULL;
}

void
sh_css_free(void *ptr)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_free() enter:\n");
	if (ptr && my_css.free)
		my_css.free(ptr);
}

/* For Acceleration API: Flush FW (shared buffer pointer) arguments */
void
sh_css_flush(struct ia_css_acc_fw *fw)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_flush() enter:\n");
	if ((fw != NULL) && (my_css.flush != NULL))
		my_css.flush(fw);
}

static void
init_sp_thread_mapping(void)
{
	unsigned int i;

	for (i = 1; i < MAX_NUM_SP_THREADS; i++) {
		sp_thread_list[i] = SP_THREAD_EMPTY_TOKEN;
	}

	for (i = 0; i < MAX_NUM_PIPES; i++) {
		pipe_num_to_sp_thread[i] = 0xFFFF;
	}
}

static void
map_pipe_num_to_sp_thread(unsigned int pipe_num)
{
	unsigned int i;
	assert(pipe_num_to_sp_thread[pipe_num] == 0xFFFF); /* pipe is not mapped to any thread */

	for (i = 0; i < MAX_NUM_SP_THREADS; i++) {
		if (sp_thread_list[i] == SP_THREAD_EMPTY_TOKEN) {
			sp_thread_list[i] = SP_THREAD_RESERVED_TOKEN;
			pipe_num_to_sp_thread[pipe_num] = i;
			break;
		}
	}
}

static void
unmap_pipe_num_to_sp_thread(unsigned int pipe_num)
{
	unsigned int thread_id;
	assert(pipe_num_to_sp_thread[pipe_num] != 0xFFFF);

	thread_id = pipe_num_to_sp_thread[pipe_num];
	pipe_num_to_sp_thread[pipe_num] = 0xFFFF;
	sp_thread_list[thread_id] = SP_THREAD_EMPTY_TOKEN;
}

static void
init_pipe_number(void)
{
	unsigned int i;
	pipe_num_counter = 0;
	for (i = 0; i < MAX_NUM_PIPES; i++)
		pipe_num_list[i] = PIPE_NUM_EMPTY_TOKEN;

	init_sp_thread_mapping();
}

static unsigned int
generate_pipe_number(void)
{
	unsigned int i;
	unsigned int pipe_num = 0;
	/*Assign a new pipe_num .... search for empty place */
	for (i = 0; i < MAX_NUM_PIPES; i++)
	{
		if (pipe_num_list[i] == PIPE_NUM_EMPTY_TOKEN){
			pipe_num_list[i] = PIPE_NUM_RESERVED_TOKEN; /*position is reserved */
			pipe_num = i;
			break;
		}
	}
	pipe_num_counter++;
	map_pipe_num_to_sp_thread(pipe_num);
	return pipe_num;
}

static void
release_pipe_num(unsigned int pipe_num)
{
	pipe_num_list[pipe_num] = PIPE_NUM_EMPTY_TOKEN;
	pipe_num_counter--;
	unmap_pipe_num_to_sp_thread(pipe_num);
}

static enum ia_css_err
create_old_pipe(enum ia_css_pipe_mode mode,
	       struct sh_css_pipe **pipe,
	       bool copy_pipe)
{
	struct sh_css_pipe *me = sh_css_malloc(sizeof(*me));
	static struct sh_css_pipe default_pipe = DEFAULT_PIPE;
	static struct sh_css_preview_settings prev  = DEFAULT_PREVIEW_SETTINGS;
	static struct sh_css_capture_settings capt  = DEFAULT_CAPTURE_SETTINGS;
	static struct sh_css_video_settings   video = DEFAULT_VIDEO_SETTINGS;

	if (!me)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	memset(me, 0, sizeof(*me));

	*me = default_pipe;
	/* TODO: JB should not be needed, but temporary backward reference */
	switch (mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		me->mode = IA_CSS_PIPE_ID_PREVIEW;
		me->pipe.preview = prev;
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
		if (copy_pipe) {
			me->mode = IA_CSS_PIPE_ID_COPY;
		} else {
			me->mode = IA_CSS_PIPE_ID_CAPTURE;
		}
		me->pipe.capture = capt;
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		me->mode = IA_CSS_PIPE_ID_VIDEO;
		me->pipe.video = video;
		break;
	case IA_CSS_PIPE_MODE_ACC:
		me->mode = IA_CSS_PIPE_ID_ACC;
		break;
	default:
		sh_css_free(me);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	sh_css_pipeline_init(&me->pipeline, me->mode);
	me->new_pipe = NULL;
	*pipe = me;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
create_pipe(enum ia_css_pipe_mode mode,
	    struct ia_css_pipe **pipe,
	    bool copy_pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_pipe *me = sh_css_malloc(sizeof(*me));

	if (!me)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	me->pipe_num = generate_pipe_number();
	err = create_old_pipe(mode, &me->old_pipe, copy_pipe);
	if (err != IA_CSS_SUCCESS) {
		sh_css_free(me);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

	my_css.active_pipes[me->pipe_num] = me;
	me->old_pipe->new_pipe = me;
	me->old_pipe->pipe_num = me->pipe_num;
	*pipe = me;
	return IA_CSS_SUCCESS;
}

static struct ia_css_pipe *
find_pipe_by_num(uint8_t pipe_num)
{
	unsigned int i;
	for (i = 0; i < pipe_num_counter; i++){
		if (my_css.active_pipes[i]->pipe_num == pipe_num){
			return  my_css.active_pipes[i];
		}
	}
	return NULL;
}

static void
destroy_frames(unsigned int num_frames, struct ia_css_frame **frames)
{
	unsigned int i;
	for (i = 0; i < num_frames; i++) {
		if (frames[i]) {
			ia_css_frame_free(frames[i]);
			frames[i] = NULL;
		}
	}
}

enum ia_css_err
ia_css_pipe_destroy(struct ia_css_pipe *pipe)
{
	struct sh_css_pipe *old_pipe = NULL;
	enum ia_css_err err = IA_CSS_SUCCESS;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_destroy() enter\n");

	if (pipe == NULL) return IA_CSS_ERR_INVALID_ARGUMENTS;

	old_pipe = pipe->old_pipe;

	if (old_pipe->stream != NULL) {
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_destroy(): "
				"ia_css_stream_destroy not called!\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	release_pipe_num(pipe->pipe_num);

	
	switch (pipe->config.mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		/* need to take into account that this function is also called
		   on the internal copy pipe */
		if (old_pipe->mode == IA_CSS_PIPE_ID_PREVIEW) {
			destroy_frames(NUM_CONTINUOUS_FRAMES,
				old_pipe->continuous_frames);
			if (old_pipe->pipe.preview.copy_pipe) {
				err = ia_css_pipe_destroy(old_pipe->pipe.preview.copy_pipe->new_pipe);
				sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_destroy(): "
					"destroyed internal copy pipe err=%d\n", err);
			}
		}
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		if (old_pipe->mode == IA_CSS_PIPE_ID_VIDEO) {
			destroy_frames(NUM_CONTINUOUS_FRAMES,
				old_pipe->continuous_frames);
			if (old_pipe->pipe.video.copy_pipe) {
				err = ia_css_pipe_destroy(old_pipe->pipe.video.copy_pipe->new_pipe);
				sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_destroy(): "
					"destroyed internal copy pipe err=%d\n", err);
			}
		}
		destroy_frames(NUM_TNR_FRAMES, old_pipe->pipe.video.tnr_frames);
		destroy_frames(NUM_REF_FRAMES, old_pipe->pipe.video.ref_frames);
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
		destroy_frames(1, &old_pipe->pipe.capture.capture_pp_frame);
#if 0
		/* Do not destroy, these are shared with preview */
		destroy_frames(NUM_CONTINUOUS_FRAMES,
				old_pipe->pipe.capture.continuous_frames);
#endif
		break;
	case IA_CSS_PIPE_MODE_ACC:
		break;
	}

	my_css.active_pipes[pipe->pipe_num] = NULL;
	sh_css_pipe_free_shading_table(old_pipe);
	sh_css_pipe_invalidate_binaries(old_pipe);
	sh_css_pipeline_clean(&old_pipe->pipeline);
	sh_css_free(old_pipe);
	old_pipe = NULL;
	sh_css_free(pipe);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_destroy() exit, err=%d\n", err);
	return err;
}

void
ia_css_uninit(void)
{
	int i = 0;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_uninit() enter: void\n");
#if WITH_PC_MONITORING
	sh_css_print("PC_MONITORING: %s() -- started\n", __func__);
	print_pc_histogram();
#endif
	/* TODO: JB: implement decent check and handling of freeing mipi frames */
	//assert(ref_count_mipi_allocation == 0); //mipi frames are not freed
	/* cleanup generic data */
	sh_css_params_uninit();
	sh_css_refcount_uninit();

	ia_css_i_host_rmgr_uninit();

	for (i = 0; i < my_css.num_mipi_frames; i++) {
		if (my_css.mipi_frames[i] != NULL)
		{
			ia_css_frame_free(my_css.mipi_frames[i]);
			my_css.mipi_frames[i] = NULL;
		}
	}
	sh_css_binary_uninit();
	sh_css_sp_uninit();
	sh_css_unload_firmware();
	if (my_css.sp_bin_addr) {
		mmgr_free(my_css.sp_bin_addr);
		my_css.sp_bin_addr = mmgr_NULL;
	}

	sh_css_sp_set_sp_running(false);
	sh_css_sp_reset_global_vars();
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_uninit() leave: return_void\n");
}

static unsigned int translate_sw_interrupt(unsigned value)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "translate_sw_interrupt() enter:\n");
	/* previous versions of sp would put info in the upper word
	   better safe than sorry so mask that away
	*/
	value = value & 0xffff;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "translate_sw_interrupt() leave: return %d\n", value);
	return value;
}

static unsigned int translate_sw_interrupt1(void)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "translate_sw_interrupt1() enter:\n");
	return translate_sw_interrupt(sh_css_get_sw_interrupt_value(1));
}

#if 0
static unsigned int translate_sw_interrupt2(void)
{
	/* By smart coding the flag/bits in value (on the SP side),
	 * no translation is required. The returned value can be
	 * binary ORed with existing interrupt info
	 * (it is compatible with enum ia_css_irq_info)
	 */
/* MW: No smart coding required, we should just keep interrupt info
   and local context info separated */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "translate_sw_interrupt2() enter:\n");
	return translate_sw_interrupt(sh_css_get_sw_interrupt_value(2));
}
#endif

/* Deprecated, this is an HRT backend function (memory_access.h) */
static void
sh_css_mmu_set_page_table_base_index(hrt_data base_index)
{
	int i;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_mmu_set_page_table_base_index() enter: base_index=0x%08x\n",base_index);
	my_css.page_table_base_index = base_index;
	for (i = 0; i < (int)N_MMU_ID; i++) {
		mmu_ID_t mmu_id = (mmu_ID_t)i;
		mmu_set_page_table_base_index(mmu_id, base_index);
		mmu_invalidate_cache(mmu_id);
	}
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_mmu_set_page_table_base_index() leave: return_void\n");
}

void
ia_css_mmu_invalidate_cache(void)
{
	const struct ia_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_invalidate_tlb;

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_mmu_invalidate_cache() enter\n");
	/* indicate to sp start that invalidation must occur */
	sh_css_sp_invalidate_mmu();

	HIVE_ADDR_sp_invalidate_tlb = fw->info.sp.invalidate_tlb;

	(void)HIVE_ADDR_sp_invalidate_tlb; /* Suppres warnings in CRUN */

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_invalidate_tlb),
		true);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_mmu_invalidate_cache() leave\n");
}

#if defined(HAS_IRQ_MAP_VERSION_1) || defined(HAS_IRQ_MAP_VERSION_1_DEMO)
enum ia_css_err ia_css_irq_translate(
	unsigned int *irq_infos)
{
	virq_id_t	irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

/* irq_infos can be NULL, but that would make the function useless */
/* assert(irq_infos != NULL); */
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_translate() enter: irq_infos=%p\n",irq_infos);

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = virq_get_channel_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return IA_CSS_ERR_INTERNAL_ERROR;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
		case virq_sp:
			infos |= IA_CSS_IRQ_INFO_EVENTS_READY;
			break;
		case virq_isp:
#ifdef HRT_CSIM
			/* Enable IRQ which signals that ISP goes to idle
			 * to get statistics for each binary */
			infos |= IA_CSS_IRQ_INFO_ISP_BINARY_STATISTICS_READY;
#endif
			break;
		case virq_isys_csi:
			/* css rx interrupt, read error bits from css rx */
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR;
			break;
		case virq_isys_fifo_full:
			infos |=
			    IA_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW;
			break;
		case virq_isys_sof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF;
			break;
		case virq_isys_eof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF;
			break;
/* Temporarily removed, until we have a seperate flag for FRAME_READY irq */
#if 0
/* hmm, an interrupt mask, why would we have that ? */
		case virq_isys_sol:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOL;
			break;
#endif
		case virq_isys_eol:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOL;
			break;
/*
 * MW: The 2300 demo system does not have a receiver, and it
 * does not have the following three IRQ channels defined
 */
#if defined(HAS_IRQ_MAP_VERSION_1)
		case virq_ifmt_sideband_changed:
			infos |=
			    IA_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED;
			break;
		case virq_gen_short_0:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0;
			break;
		case virq_gen_short_1:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1;
			break;
#endif
		case virq_ifmt0_id:
			infos |= IA_CSS_IRQ_INFO_IF_PRIM_ERROR;
			break;
		case virq_ifmt1_id:
			infos |= IA_CSS_IRQ_INFO_IF_PRIM_B_ERROR;
			break;
		case virq_ifmt2_id:
			infos |= IA_CSS_IRQ_INFO_IF_SEC_ERROR;
			break;
		case virq_ifmt3_id:
			infos |= IA_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR;
			break;
		case virq_sw_pin_0:
			infos |= IA_CSS_IRQ_INFO_SW_0;
			break;
		case virq_sw_pin_1:
			infos |= translate_sw_interrupt1();
			/* pqiao TODO: also assumption here */
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_translate() "
		"leave: irq_infos=0x%08x\n", infos);

	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_irq_enable(enum ia_css_irq_info info,
		  bool enable)
{
	virq_id_t	irq = N_virq_id;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() enter: info=%d, enable=%d\n",info,enable);

	switch (info) {
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR:
		irq = virq_isys_csi;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW:
		irq = virq_isys_fifo_full;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF:
		irq = virq_isys_sof;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF:
		irq = virq_isys_eof;
		break;
/* Temporarily removed, until we have a seperate flag for FRAME_READY irq */
#if 0
/* hmm, an interrupt mask, why would we have that ? */
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOL:
		irq = virq_isys_sol;
		break;
#endif
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOL:
		irq = virq_isys_eol;
		break;
#if defined(HAS_IRQ_MAP_VERSION_1)
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED:
		irq = virq_ifmt_sideband_changed;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0:
		irq = virq_gen_short_0;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1:
		irq = virq_gen_short_1;
		break;
#endif
	case IA_CSS_IRQ_INFO_IF_PRIM_ERROR:
		irq = virq_ifmt0_id;
		break;
	case IA_CSS_IRQ_INFO_IF_PRIM_B_ERROR:
		irq = virq_ifmt1_id;
		break;
	case IA_CSS_IRQ_INFO_IF_SEC_ERROR:
		irq = virq_ifmt2_id;
		break;
	case IA_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR:
		irq = virq_ifmt3_id;
		break;
	case IA_CSS_IRQ_INFO_SW_0:
		irq = virq_sw_pin_0;
		break;
	case IA_CSS_IRQ_INFO_SW_1:
		irq = virq_sw_pin_1;
		break;
	case IA_CSS_IRQ_INFO_SW_2:
		irq = virq_sw_pin_2;
		break;
	default:
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	cnd_virq_enable_channel(irq, enable);

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

#elif defined(HAS_IRQ_MAP_VERSION_2)

enum ia_css_err ia_css_irq_translate(
	unsigned int *irq_infos)
{
	virq_id_t	irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

/* irq_infos can be NULL, but that would make the function useless */
/* assert(irq_infos != NULL); */
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_translate() enter: irq_infos=%d\n",irq_infos);

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = virq_get_channel_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return IA_CSS_ERR_INTERNAL_ERROR;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
		case virq_sp:
			infos |= IA_CSS_IRQ_INFO_EVENTS_READY;
			break;
		case virq_isp:
#ifdef HRT_CSIM
			/* Enable IRQ which signals that ISP goes to idle
			 * to get statistics for each binary */
			infos |= IA_CSS_IRQ_INFO_ISP_BINARY_STATISTICS_READY;
#endif
			break;
		case virq_isys_sof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF;
			break;
		case virq_isys_eof:
			infos |= IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF;
			break;
		case virq_isys_csi:
			infos |= IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR;
			break;
		case virq_ifmt0_id:
			infos |= IA_CSS_IRQ_INFO_IF_ERROR;
			break;
		case virq_dma:
			infos |= IA_CSS_IRQ_INFO_DMA_ERROR;
			break;
		case virq_sw_pin_0:
			infos |= IA_CSS_IRQ_INFO_SW_0;
			break;
		case virq_sw_pin_1:
			infos |= translate_sw_interrupt1();
			/* pqiao TODO: also assumption here */
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_translate() "
		"leave: irq_infos=0x%08x\n", infos);

	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_irq_enable(
	enum ia_css_irq_info info,
	bool enable)
{
	virq_id_t	irq = N_virq_id;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() enter: info=%d, enable=%d\n",info,enable);

	switch (info) {
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF:
		irq = virq_isys_sof;
		break;
	case IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF:
		irq = virq_isys_eof;
		break;
	case IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR:
		irq = virq_isys_csi;
		break;
	case IA_CSS_IRQ_INFO_IF_ERROR:
		irq = virq_ifmt0_id;
		break;
	case IA_CSS_IRQ_INFO_DMA_ERROR:
		irq = virq_dma;
		break;
	case IA_CSS_IRQ_INFO_SW_0:
		irq = virq_sw_pin_0;
		break;
	case IA_CSS_IRQ_INFO_SW_1:
		irq = virq_sw_pin_1;
		break;
	default:
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	cnd_virq_enable_channel(irq, enable);

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_irq_enable() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

#else
#error "sh_css.c: IRQ MAP must be one of \
	{IRQ_MAP_VERSION_1, IRQ_MAP_VERSION_1_DEMO, IRQ_MAP_VERSION_2}"
#endif

unsigned int sh_css_get_sw_interrupt_value(
	unsigned int irq)
{
unsigned int	irq_value;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_get_sw_interrupt_value() enter: irq=%d\n",irq);
	irq_value = sh_css_sp_get_sw_interrupt_value(irq);
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_get_sw_interrupt_value() leave: irq_value=%d\n",irq_value);
return irq_value;
}

void
sh_css_uv_offset_is_zero(bool *uv_offset_is_zero)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_uv_offset_is_zero() enter:\n");
	if (uv_offset_is_zero != NULL) {
		*uv_offset_is_zero = SH_CSS_UV_OFFSET_IS_0;
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_uv_offset_is_zero() leave: uv_offset_is_zero=%d\n",
		*uv_offset_is_zero);
	}
}

#if 0
/* Disabled because it is currently unused. */
static void
sh_css_pipe_get_extra_pixels_count(const struct sh_css_pipe *pipe,
				   struct ia_css_resolution *extra)
{
	int rows = SH_CSS_MAX_LEFT_CROPPING,
	    cols = SH_CSS_MAX_LEFT_CROPPING;

	assert(pipe != NULL);
	assert(extra != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipe_get_extra_pixels_count() enter: void\n");

	if (lines_needed_for_bayer_order(pipe))
		rows += 2;

	if (columns_needed_for_bayer_order(pipe))
		cols  += 2;

	extra->width  = cols;
	extra->height = rows;

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_get_extra_pixels_count() leave: extra_rows=%d, extra_cols=%d\n",
		rows, cols);
}
#endif

static void init_copy_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
/* out_info can be NULL */
assert(pipe != NULL);
assert(in_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"init_copy_descr() enter:\n");

	*in_info = *out_info;

	copy_descr.mode          = SH_CSS_BINARY_MODE_COPY;
	copy_descr.online        = true;
	copy_descr.stream_format = pipe->stream->config.format;
	copy_descr.binning       = false;
	copy_descr.two_ppc       = pipe->stream->config.two_pixels_per_clock;
	copy_descr.in_info       = in_info;
	copy_descr.out_info      = out_info;
	copy_descr.vf_info       = NULL;
	copy_descr.isp_pipe_version = 1;
}

static void init_offline_descr(
	struct sh_css_pipe *pipe,
	struct sh_css_binary_descr *descr,
	int mode,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info,
	struct ia_css_frame_info *vf_info)
{
/* in_info, out_info, vf_info can be NULL */
assert(pipe != NULL);
assert(descr != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"init_offline_descr() enter:\n");

	descr->mode          = mode;
	descr->online        = false;
	descr->continuous    = pipe->stream->config.continuous;
	descr->stream_format = pipe->stream->config.format;
	descr->binning       = false;
	descr->two_ppc       = false;
	descr->in_info       = in_info;
	descr->out_info      = out_info;
	descr->vf_info       = vf_info;
	descr->isp_pipe_version = pipe->isp_pipe_version;
	descr->enable_yuv_ds = false;
	descr->enable_high_speed = false;
	descr->enable_reduced_pipe = false;
	descr->enable_dvs_6axis = false;
	descr->enable_dz     = false;
	descr->dvs_env.width = 0;
	descr->dvs_env.height = 0;
}

static void
init_vf_pp_descr(struct sh_css_pipe *pipe,
		 struct ia_css_frame_info *in_info,
		 struct ia_css_frame_info *out_info)
{
/* out_info can be NULL ??? */
assert(pipe != NULL);
assert(in_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"init_vf_pp_descr() enter:\n");

	in_info->raw_bit_depth = 0;
	init_offline_descr(pipe,
			   &vf_pp_descr, SH_CSS_BINARY_MODE_VF_PP,
			   in_info, out_info, NULL);
}

static void init_preview_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
	int mode = SH_CSS_BINARY_MODE_PREVIEW;

/* out_info can be NULL ??? */
assert(pipe != NULL);
assert(in_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"init_preview_descr() enter:\n");

	in_info->res = pipe->stream->info.effective_info;
	in_info->padded_width = in_info->res.width;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	if (input_format_is_yuv(pipe->stream->config.format))
		mode = SH_CSS_BINARY_MODE_COPY;
	else
		in_info->format = IA_CSS_FRAME_FORMAT_RAW;

	init_offline_descr(pipe,
			   &preview_descr, mode,
			   in_info, out_info, NULL);
	if (pipe->stream->config.online) {
		preview_descr.online	    = pipe->stream->config.online;
		preview_descr.two_ppc       = pipe->stream->config.two_pixels_per_clock;
	}
	preview_descr.stream_format = pipe->stream->config.format;
	preview_descr.binning	    = pipe->input_needs_raw_binning;
	preview_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

/* configure and load the copy binary, the next binary is used to
   determine whether the copy binary needs to do left padding. */
static enum ia_css_err load_copy_binary(
	struct sh_css_pipe *pipe,
	struct sh_css_binary *copy_binary,
	struct sh_css_binary *next_binary)
{
	struct ia_css_frame_info copy_out_info, copy_in_info;
	unsigned int left_padding;
	enum ia_css_err err;
	int mode = SH_CSS_BINARY_MODE_COPY;

/* next_binary can be NULL */
assert(pipe != NULL);
assert(copy_binary != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"load_copy_binary() enter:\n");

	if (next_binary != NULL) {
		copy_out_info = next_binary->in_frame_info;
		left_padding = next_binary->left_padding;
	} else {
		copy_out_info = pipe->output_info;
		left_padding = 0;
	}

	init_copy_descr(pipe, &copy_in_info, &copy_out_info);
	copy_descr.mode = mode;
	err = sh_css_binary_find(&copy_descr, copy_binary);
	if (err != IA_CSS_SUCCESS)
		return err;
	copy_binary->left_padding = left_padding;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
alloc_continuous_frames(
	struct sh_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame_info ref_info;
	enum ia_css_pipe_id pipe_id = pipe->mode;
	bool continuous = pipe->stream->config.continuous;
	bool input_needs_raw_binning = pipe->input_needs_raw_binning;
	unsigned int i;
	unsigned int left_cropping = 0;
	uint8_t raw_binning = 0;

	if (pipe_id == IA_CSS_PIPE_ID_PREVIEW) {
		left_cropping = pipe->pipe.preview.preview_binary.info->left_cropping;
		ref_info = pipe->pipe.preview.preview_binary.in_frame_info;
		raw_binning = pipe->pipe.preview.preview_binary.info->enable.raw_binning;
	} else if (pipe_id == IA_CSS_PIPE_ID_VIDEO) {
		left_cropping = pipe->pipe.video.video_binary.info->left_cropping;
		ref_info = pipe->pipe.video.video_binary.in_frame_info;
		raw_binning = pipe->pipe.video.video_binary.info->enable.raw_binning;
	}

	if (input_needs_raw_binning && raw_binning) {
		/* TODO: Remove this; when the decimated
		 * resolution is available */
		/* Only for continuous preview/video mode
		 * where we need 2xOut resolution */
		ref_info.padded_width *= 2;
		ref_info.res.width -= left_cropping;
		ref_info.res.width *= 2;
		/* In case of left-cropping, add 2 vectors */
		ref_info.res.width += left_cropping ? 2*ISP_VEC_NELEMS : 0;
		/* Must be even amount of vectors */
		ref_info.res.width  = CEIL_MUL(ref_info.res.width,2*ISP_VEC_NELEMS);
		ref_info.res.height -= left_cropping;
		ref_info.res.height *= 2;
		ref_info.res.height += left_cropping;
	} else if (continuous) {
		ref_info.res.width -= left_cropping;
		/* In case of left-cropping, add 2 vectors */
		ref_info.res.width += left_cropping ? 2*ISP_VEC_NELEMS : 0;
		/* Must be even amount of vectors */
		ref_info.res.width  = CEIL_MUL(ref_info.res.width,2*ISP_VEC_NELEMS);
	}

	ref_info.format = IA_CSS_FRAME_FORMAT_RAW;

	for (i = 0; i < NUM_CONTINUOUS_FRAMES; i++) {
		/* free previous frame */
		if (pipe->continuous_frames[i]) {
			ia_css_frame_free(pipe->continuous_frames[i]);
			pipe->continuous_frames[i] = NULL;
		}
		/* check if new frame needed */
		if (i < my_css.num_cont_raw_frames) {
			/* allocate new frame */
			err = ia_css_frame_allocate_from_info(
				&pipe->continuous_frames[i],
				&ref_info);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
	}
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
allocate_mipi_frames(struct sh_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int i;
	struct ia_css_frame_info mipi_intermediate_info;

	assert(pipe != NULL);
	assert(pipe->stream != NULL); 
	
	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
		return err;


	ref_count_mipi_allocation++;	
	if (ref_count_mipi_allocation > 1)
		return err;
	assert(ref_count_mipi_allocation == 1);
	
// This code needs to modified to allocate the MIPI frames in the correct normal way with a allocate from info
// by justin
	mipi_intermediate_info = pipe->pipe.video.video_binary.internal_frame_info;
	mipi_intermediate_info.res.width = 0;
	mipi_intermediate_info.res.height = 0;
	// To indicate it is not (yet) valid format.
	mipi_intermediate_info.format = IA_CSS_FRAME_FORMAT_NUM;
	mipi_intermediate_info.padded_width = 0;
	mipi_intermediate_info.raw_bit_depth = 0;
	//mipi_intermediate_info.data_bytes = my_css.size_mem_words * HIVE_ISP_DDR_WORD_BYTES;
	//mipi_intermediate_info.contiguous = my_css.contiguous;
	// To indicate it is not valid frame.
	//mipi_intermediate_info.dynamic_data_index = SH_CSS_INVALID_FRAME_ID;

	for (i = 0; i < my_css.num_mipi_frames; i++) {
		/* free previous frame */
		if (my_css.mipi_frames[i]) {
			ia_css_frame_free(my_css.mipi_frames[i]);
			my_css.mipi_frames[i] = NULL;
		}
		/* check if new frame needed */
		if (i < my_css.num_mipi_frames) {
			/* allocate new frame */
			err = ia_css_mipi_frame_allocate(
				&my_css.mipi_frames[i],
				my_css.size_mem_words * HIVE_ISP_DDR_WORD_BYTES,
				my_css.contiguous);
			if (err != IA_CSS_SUCCESS)
				return err;
			if (SH_CSS_PREVENT_UNINIT_READS)
				ia_css_frame_zero(my_css.mipi_frames[i]);
		}
	}

	return err;
}

static void
free_mipi_frames(struct sh_css_pipe *pipe)
{
	unsigned int i;
		
	assert(pipe != NULL); 
	assert(pipe->stream != NULL); 

	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
		return;

	assert(ref_count_mipi_allocation > 0);
	ref_count_mipi_allocation--;
	if (ref_count_mipi_allocation > 0)
		return; 
		

	for (i = 0; i < my_css.num_mipi_frames; i++) {
		if (my_css.mipi_frames[i] != NULL)
		{
			ia_css_frame_free(my_css.mipi_frames[i]);
			my_css.mipi_frames[i] = NULL;
		}
	}
}


static void
send_mipi_frames (struct sh_css_pipe *pipe)
{
	unsigned int i;
	/* multi stream video needs mipi buffers */
	if (pipe->stream->config.mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR)
		return;

	/* Hand-over the SP-internal mipi buffers */
	for (i = 0; i < my_css.num_mipi_frames; i++) {
		sh_css_update_host2sp_mipi_frame(i,
			my_css.mipi_frames[i]);
	}
	sh_css_update_host2sp_cont_num_mipi_frames
		(my_css.num_mipi_frames);

	/**********************************
	 *
	 * Hack for Baytrail.
	 *
	 * AUTHOR: zhengjie.lu@intel.com
	 * TIME: 2013-01-19, 14:38.
	 * LOCATION: Santa Clara, U.S.A.
	 * COMMENT:
	 * Send an event to inform the SP
	 * that all MIPI frames are passed.
	 *
	 **********************************/
	{
		sh_css_sp_snd_event(SP_SW_EVENT_ID_6,	/* the event ID  */
			0,				/* not used */
			0,				/* not used */
			0				/* not used */);
	}
	/** End of hack of Baytrail **/
}

static enum ia_css_err
load_preview_binaries(struct sh_css_pipe *pipe)
{
	struct ia_css_frame_info prev_in_info,
				 prev_out_info;
	bool online = pipe->stream->config.online;
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool continuous = pipe->stream->config.continuous;
	unsigned int left_cropping;

	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"load_preview_binaries() enter:\n");

	if (pipe->pipe.preview.preview_binary.info &&
	    pipe->pipe.preview.vf_pp_binary.info)
		return IA_CSS_SUCCESS;

	err = check_input(pipe, false);
	if (err != IA_CSS_SUCCESS)
		return err;
	err = check_frame_info(&pipe->output_info);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Preview */
	if (pipe->yuv_ds_input_info.res.width)
		prev_out_info = pipe->yuv_ds_input_info;
	else
		prev_out_info = pipe->output_info;
	sh_css_frame_info_set_format(&prev_out_info,
				     IA_CSS_FRAME_FORMAT_YUV_LINE);
	init_preview_descr(pipe, &prev_in_info, &prev_out_info);
	err = sh_css_binary_find(&preview_descr,
				 &pipe->pipe.preview.preview_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Viewfinder post-processing */
	init_vf_pp_descr(pipe,
			&pipe->pipe.preview.preview_binary.out_frame_info,
			&pipe->output_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &pipe->pipe.preview.vf_pp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Copy */
	if (!online && !continuous) {
		err = load_copy_binary(pipe,
				       &pipe->pipe.preview.copy_binary,
				       &pipe->pipe.preview.preview_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	left_cropping = pipe->pipe.preview.preview_binary.info->left_cropping;

	err = alloc_continuous_frames(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (SH_CSS_PREVENT_UNINIT_READS)
		ia_css_frame_zero(pipe->continuous_frames[0]);

	if (pipe->shading_table) {
		ia_css_shading_table_free(pipe->shading_table);
		pipe->shading_table = NULL;
	}

	err = allocate_mipi_frames(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	return IA_CSS_SUCCESS;
}

static const struct ia_css_fw_info *last_output_firmware(
	const struct ia_css_fw_info *fw)
{
	const struct ia_css_fw_info *last_fw = NULL;
/* fw can be NULL */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"last_output_firmware() enter:\n");

	for (; fw; fw = fw->next) {
		const struct ia_css_fw_info *info = fw;
		if (info->info.isp.enable.output)
			last_fw = fw;
	}
	return last_fw;
}

static enum ia_css_err add_firmwares(
	struct sh_css_pipeline *me,
	struct sh_css_binary *binary,
	const struct ia_css_fw_info *fw,
	const struct ia_css_fw_info *last_fw,
	unsigned int binary_mode,
	struct ia_css_frame *in_frame,
	struct ia_css_frame *out_frame,
	struct ia_css_frame *vf_frame,
	struct sh_css_pipeline_stage **my_stage,
	struct sh_css_pipeline_stage **vf_stage)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_pipeline_stage *extra_stage = NULL;

/* all args can be NULL ??? */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"add_firmwares() enter:\n");

	for (; fw; fw = fw->next) {
		struct ia_css_frame *out = NULL;
		struct ia_css_frame *in = NULL;
		struct ia_css_frame *vf = NULL;
		if ((fw == last_fw) && (fw->info.isp.enable.out_frame  != 0)) {
			out = out_frame;
		}
		if (fw->info.isp.enable.in_frame != 0) {
			in = in_frame;
		}
		if (fw->info.isp.enable.out_frame != 0) {
			vf = vf_frame;
		}

		err = sh_css_pipeline_add_stage(me, binary, fw,
				binary_mode, NULL,
				in, out,
				vf, &extra_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		if (fw->info.isp.enable.output != 0)
			in_frame = extra_stage->args.out_frame;
		if (my_stage && !*my_stage && extra_stage)
			*my_stage = extra_stage;
		if (vf_stage && !*vf_stage && extra_stage &&
		    fw->info.isp.enable.vf_veceven)
			*vf_stage = extra_stage;
	}
	return err;
}

static enum ia_css_err add_vf_pp_stage(
	struct sh_css_pipe *pipe,
	struct ia_css_frame *out_frame,
	struct sh_css_binary *vf_pp_binary,
	struct sh_css_pipeline_stage *post_stage,
	struct sh_css_pipeline_stage **vf_pp_stage)
{
	struct sh_css_pipeline *me = &pipe->pipeline;
	const struct ia_css_fw_info *last_fw;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *in_frame = post_stage->args.out_vf_frame;

/* out_frame can be NULL ??? */
assert(pipe != NULL);
assert(vf_pp_binary != NULL);
assert(post_stage != NULL);
assert(vf_pp_stage != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"add_vf_pp_stage() enter:\n");

	*vf_pp_stage = NULL;

	if (in_frame == NULL)
		in_frame = post_stage->args.out_frame;

	last_fw = last_output_firmware(pipe->vf_stage);
	if (!pipe->disable_vf_pp) {
		err = sh_css_pipeline_add_stage(me, vf_pp_binary, NULL,
				vf_pp_binary->info->mode, NULL,
				in_frame,
				last_fw ? NULL : out_frame,
				NULL, vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_frame = (*vf_pp_stage)->args.out_frame;
	}
	err = add_firmwares(me, vf_pp_binary, pipe->vf_stage, last_fw,
			    SH_CSS_BINARY_MODE_VF_PP,
			    in_frame, out_frame, NULL,
			    vf_pp_stage, NULL);
	return err;
}

static enum ia_css_err add_capture_pp_stage(
	struct sh_css_pipe *pipe,
	struct sh_css_pipeline *me,
	struct ia_css_frame *out_frame,
	struct sh_css_binary *capture_pp_binary,
	struct sh_css_pipeline_stage *capture_stage,
	struct sh_css_pipeline_stage **pre_vf_pp_stage)
{
	const struct ia_css_fw_info *last_fw;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *in_frame = capture_stage->args.out_frame;
	struct ia_css_frame *vf_frame = NULL;

/* out_frame can be NULL ??? */
assert(pipe != NULL);
assert(me != NULL);
assert(capture_pp_binary != NULL);
assert(capture_stage != NULL);
assert(pre_vf_pp_stage != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"add_capture_pp_stage() enter:\n");

	*pre_vf_pp_stage = NULL;

	if (in_frame == NULL)
		in_frame = capture_stage->args.out_frame;

	last_fw = last_output_firmware(pipe->output_stage);
	if (!pipe->disable_capture_pp &&
	    need_capture_pp(pipe)) {
		err = ia_css_frame_allocate_from_info(&vf_frame,
					    &capture_pp_binary->vf_frame_info);
		if (err != IA_CSS_SUCCESS)
			return err;
		err = sh_css_pipeline_add_stage(me, capture_pp_binary, NULL,
				capture_pp_binary->info->mode, NULL,
				NULL,
				last_fw ? NULL : out_frame,
				vf_frame, pre_vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_frame = (*pre_vf_pp_stage)->args.out_frame;
	}
	err = add_firmwares(me, capture_pp_binary, pipe->output_stage, last_fw,
			    SH_CSS_BINARY_MODE_CAPTURE_PP,
			    in_frame, out_frame, vf_frame,
			    NULL, pre_vf_pp_stage);
	/* If a firmware produce vf_pp output, we set that as vf_pp input */
	if (*pre_vf_pp_stage) {
		(*pre_vf_pp_stage)->args.extra_frame =
		  pipe->pipe.capture.capture_pp_frame;
		(*pre_vf_pp_stage)->args.vf_downscale_log2 =
		  capture_pp_binary->vf_downscale_log2;
	} else {
		*pre_vf_pp_stage = capture_stage;
	}
	return err;
}

static void
number_stages(
	struct sh_css_pipe *pipe)
{
	unsigned i = 0;
	struct sh_css_pipeline_stage *stage;
	for (stage = pipe->pipeline.stages; stage; stage = stage->next) {
		stage->stage_num = i;
		i++;
	}
	pipe->pipeline.num_stages = i;
}

void
sh_css_init_buffer_queues(void)
{
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_host_sp_queues_initialized;
	unsigned int i;

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init_buffer_queues() enter:\n");

	for (i = 0; i < MAX_HMM_BUFFER_NUM; i++)
		hmm_buffer_record_h[i] = NULL;


	sh_css_event_init_irq_mask();

	fw = &sh_css_sp_fw;
	HIVE_ADDR_host_sp_queues_initialized =
		fw->info.sp.host_sp_queues_initialized;

	/* initialize the "sp2host" queues */
	init_sp2host_queues();

	/* initialize the "host2sp" queues */
	init_host2sp_queues();

	/* set "host_sp_queues_initialized" to "true" */
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(host_sp_queues_initialized),
		(uint32_t)(1));

	

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_init_buffer_queues() leave:\n");
}

static enum ia_css_err
preview_start(struct sh_css_pipe *pipe)
{
	struct sh_css_pipeline *me = &pipe->pipeline;
	struct sh_css_pipeline_stage *preview_stage, *copy_stage;
	struct sh_css_pipeline_stage *vf_pp_stage;
	struct ia_css_frame *in_frame = NULL, *cc_frame = NULL;
	struct sh_css_binary *copy_binary, *preview_binary, *vf_pp_binary;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_pipe *copy_pipe, *capture_pipe;
	enum sh_css_pipe_config_override copy_ovrd;
	enum ia_css_input_mode preview_pipe_input_mode;

	/**
	 * rvanimme: raw_out_frame support is broken and forced to NULL
	 * TODO: add a way to tell the pipeline construction that a
	 * raw_out_frame is used.
	 */
	struct ia_css_frame *raw_out_frame = NULL;
	struct ia_css_frame *out_frame = &me->out_frame;
	
	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"preview_start() enter: void\n");

	preview_pipe_input_mode = pipe->stream->config.mode;

	copy_pipe    = pipe->pipe.preview.copy_pipe;
	capture_pipe = pipe->pipe.preview.capture_pipe;
	
	sh_css_pipeline_clean(me);

	sh_css_pipe_get_output_frame_info(pipe, &out_frame->info);
	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	out_frame->dynamic_data_index = sh_css_frame_out;
	err = init_frame_planes(out_frame);
	if (err != IA_CSS_SUCCESS)
		return err;

	copy_stage = NULL;

	if (pipe->zoom_changed) {
		sh_css_pipe_invalidate_binaries(pipe);
		pipe->zoom_changed = false;
	}

	err = sh_css_pipe_load_binaries(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	copy_binary    = &pipe->pipe.preview.copy_binary;
	preview_binary = &pipe->pipe.preview.preview_binary;
	vf_pp_binary   = &pipe->pipe.preview.vf_pp_binary;

	sh_css_metrics_start_frame();

	if (me->reload) {
		struct sh_css_pipeline_stage *post_stage;
		if (pipe->pipe.preview.copy_binary.info) {
			err = sh_css_pipeline_add_stage(me, copy_binary, NULL,
					copy_binary->info->mode,
					NULL, NULL, raw_out_frame, NULL,
					&post_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
			in_frame = me->stages->args.out_frame;
		} else {
			in_frame = pipe->continuous_frames[0];
		}
		err = sh_css_pipeline_add_stage(me, preview_binary, NULL,
						preview_binary->info->mode,
						cc_frame, in_frame, NULL, NULL,
						&post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		/* If we use copy iso preview, the input must be yuv iso raw */
		post_stage->args.copy_vf =
			preview_binary->info->mode == SH_CSS_BINARY_MODE_COPY;
		post_stage->args.copy_output = !post_stage->args.copy_vf;
		if (post_stage->args.copy_vf) {
			/* in case of copy, use the vf frame as output frame */
			post_stage->args.out_vf_frame =
				post_stage->args.out_frame;
		}

		err = add_vf_pp_stage(pipe, out_frame, vf_pp_binary,
				      post_stage, &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		number_stages(pipe);
	} else {
		sh_css_pipeline_restart(me);
	}

	err = sh_css_pipeline_get_output_stage(me, SH_CSS_BINARY_MODE_VF_PP,
					       &vf_pp_stage);

	if (err != IA_CSS_SUCCESS)
		return err;
	err = sh_css_pipeline_get_stage(me, preview_binary->info->mode,
					&preview_stage);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* multi stream video needs mipi buffers */
	send_mipi_frames(pipe);

	vf_pp_stage->args.out_frame = out_frame;

	if (pipe->stream->config.continuous) {
		int i;
		in_frame = pipe->continuous_frames[0];
		preview_stage->args.in_frame = in_frame;

		/* Hand-over all the SP-internal buffers */
		for (i = 0; i < NUM_CONTINUOUS_FRAMES; i++) {
			sh_css_update_host2sp_offline_frame(i,
				pipe->continuous_frames[i]);
		}
	}
	/* update the arguments with the latest info */

	sh_css_set_irq_buffer(preview_stage, sh_css_frame_in,  raw_out_frame);
	sh_css_set_irq_buffer(vf_pp_stage,   sh_css_frame_out, out_frame);

	{
		unsigned int thread_id;
		
		sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);
		copy_ovrd = 1 << thread_id;
		
		if (pipe->stream->cont_capt) {
			sh_css_query_sp_thread_id(capture_pipe->pipe_num, &thread_id);
			copy_ovrd |= 1 << thread_id;
		}
	}

	/* Construct and load the copy pipe */
	if (pipe->stream->config.continuous) {
		err = construct_copy_pipe(copy_pipe,
					  preview_binary->info->max_input_width,
					  pipe->continuous_frames[0]);
		if (err != IA_CSS_SUCCESS)
			return err;

		sh_css_sp_init_pipeline(&copy_pipe->pipeline, IA_CSS_PIPE_ID_COPY,
			copy_pipe->pipe_num,
			false, false, false,
			pipe->stream->config.two_pixels_per_clock, false,
			false, pipe->input_needs_raw_binning,
			copy_ovrd,
			pipe->stream->config.mode,
			pipe->stream->config.source.port.port);

		/* make the preview pipe start with mem mode input, copy handles
		   the actual mode */
		preview_pipe_input_mode = IA_CSS_INPUT_MODE_MEMORY;
	}
	
	/* Construct and load the capture pipe */
	if (pipe->stream->cont_capt) {
		bool low_light;

		err = construct_capture_pipe(capture_pipe);
		if (err != IA_CSS_SUCCESS)
			return err;

		low_light = (capture_pipe->capture_mode ==
				IA_CSS_CAPTURE_MODE_LOW_LIGHT) ||
				(capture_pipe->capture_mode ==
				IA_CSS_CAPTURE_MODE_BAYER);

		sh_css_sp_init_pipeline(&capture_pipe->pipeline, IA_CSS_PIPE_ID_CAPTURE,
			capture_pipe->pipe_num,
			false, low_light, pipe->xnr,
			capture_pipe->stream->config.two_pixels_per_clock,
			true, /* continuous */
			false, /* offline */
			capture_pipe->input_needs_raw_binning,
			0,
			IA_CSS_INPUT_MODE_MEMORY,
			(mipi_port_ID_t)0);
	}

	err = sh_css_config_input_network(pipe, copy_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	start_pipe(pipe, copy_ovrd, preview_pipe_input_mode);

	me->reload = false;

	return IA_CSS_SUCCESS;
}

static bool
sh_css_pipe_has_stopped(struct ia_css_pipe *pipe)
{
	unsigned int thread_id;
	struct sh_css_sp_group sp_group;
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_sp_group;
	
	fw = &sh_css_sp_fw;
	HIVE_ADDR_sp_group = fw->info.sp.group;

	sh_css_query_sp_thread_id(pipe->old_pipe->pipe_num, &thread_id);
	sp_dmem_load(SP0_ID,
		     (unsigned int)sp_address_of(sp_group),
		     &sp_group,
		     sizeof(struct sh_css_sp_group));
	return sp_group.pipe[thread_id].num_stages == 0;
}

static enum ia_css_err
sh_css_pipe_request_stop(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int thread_id;

	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_request_stop() enter: pipe=%p\n",
		pipe);
	pipe->stop_requested = true;
	
	// Send stop event to the sp
	// This needs improvement, stop on all the pipes available in the stream
	sh_css_query_sp_thread_id(pipe->old_pipe->pipe_num, &thread_id);
	sh_css_sp_snd_event(SP_SW_EVENT_ID_5, thread_id, 0,  0);
	sh_css_sp_uninit_pipeline(pipe->old_pipe->pipe_num);

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_request_stop() leave: return_err=%d\n",err);
	return err;
}

static enum ia_css_err
sh_css_pipe_stop(struct ia_css_pipe *pipe)
{
	enum ia_css_err err;

	err = sh_css_pipe_request_stop(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;
	/* synchronization here, otherwise we cannot map new pipe to the same thread */
	while (!sh_css_pipe_has_stopped(pipe))
		hrt_sleep();

	ia_css_pipe_dequeue_unused_buffer(pipe);
	
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_stop() leave: return_err=%d\n",err);
	return err;
}

enum ia_css_err
ia_css_pipe_enqueue_buffer(struct ia_css_pipe *pipe,
			   const struct ia_css_buffer *buffer)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	unsigned int thread_id, i;
	enum sh_css_buffer_queue_id queue_id;
	struct sh_css_pipeline *pipeline;
	struct sh_css_pipeline_stage *stage;
	struct ia_css_i_host_rmgr_vbuf_handle p_vbuf;
	struct ia_css_i_host_rmgr_vbuf_handle *h_vbuf;
	struct sh_css_hmm_buffer ddr_buffer;
	bool rc = true;
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id pipe_id;

	if (buffer == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (pipe != NULL && pipe->old_pipe)
		pipe_id = pipe->old_pipe->mode;
	else
		pipe_id = IA_CSS_PIPE_ID_COPY;

	assert(buffer != NULL);

	buf_type = buffer->type;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_enqueue_buffer() enter: pipe_id=%d, buf_type=%d, buffer=%p\n",
		pipe_id, buf_type, buffer);


	assert(pipe_id < IA_CSS_PIPE_ID_NUM);
	assert(buf_type < IA_CSS_BUFFER_TYPE_NUM);

	//sh_css_query_sp_thread_id(pipe_id, &thread_id);
	sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);
	
	sh_css_query_internal_queue_id(buf_type, &queue_id);

	if (pipe != NULL && pipe->old_pipe != NULL)
		pipeline = &pipe->old_pipe->pipeline;
	else
		pipeline = NULL;

	assert(pipeline != NULL ||
	       pipe_id == IA_CSS_PIPE_ID_COPY ||
	       pipe_id == IA_CSS_PIPE_ID_ACC);

	if (buf_type == IA_CSS_BUFFER_TYPE_3A_STATISTICS) {
		if (buffer->data.stats_3a == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
/* MW: I don't think "ddr_buffer.kernel_ptr" is an hrt_vaddress ?! */
		ddr_buffer.kernel_ptr = (hrt_vaddress)HOST_ADDRESS(buffer->data.stats_3a);
		ddr_buffer.payload.s3a = *buffer->data.stats_3a;
	} else if (buf_type == IA_CSS_BUFFER_TYPE_DIS_STATISTICS) {
		if (buffer->data.stats_dvs == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = (hrt_vaddress)HOST_ADDRESS(buffer->data.stats_dvs);
		ddr_buffer.payload.dis = *buffer->data.stats_dvs;
	} else if ((buf_type == IA_CSS_BUFFER_TYPE_INPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)) {
		if (buffer->data.frame == NULL)
			return IA_CSS_ERR_INVALID_ARGUMENTS;
		ddr_buffer.kernel_ptr = (hrt_vaddress)HOST_ADDRESS(buffer->data.frame);
		ddr_buffer.payload.frame.frame_data = buffer->data.frame->data;
		ddr_buffer.payload.frame.flashed = 0;
	}
/* start of test for using rmgr for acq/rel memory */
	p_vbuf.vptr = 0;
	p_vbuf.count = 0;
	p_vbuf.size = sizeof(struct sh_css_hmm_buffer);
	h_vbuf = &p_vbuf;
	// TODO: change next to correct pool for optimization
	ia_css_i_host_rmgr_acq_vbuf(hmm_buffer_pool, &h_vbuf);

	assert(h_vbuf != NULL);
	assert(h_vbuf->vptr != 0x0);
	if (h_vbuf == NULL || h_vbuf->vptr == mmgr_NULL)
		return IA_CSS_ERR_INTERNAL_ERROR;

	mmgr_store(h_vbuf->vptr,
				(void *)(&ddr_buffer),
				sizeof(struct sh_css_hmm_buffer));
	if ((buf_type == IA_CSS_BUFFER_TYPE_3A_STATISTICS)
		|| (buf_type == IA_CSS_BUFFER_TYPE_DIS_STATISTICS)) {
		for (stage = pipeline->stages; stage; stage = stage->next) {
			/* The SP will read the params
				after it got empty 3a and dis */
			if (STATS_ENABLED(stage)) {
				/* there is a stage that needs it */
				rc = host2sp_enqueue_buffer(thread_id, 0,
						queue_id,
						(uint32_t)h_vbuf->vptr);
			}
		}
	} else if ((buf_type == IA_CSS_BUFFER_TYPE_INPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_OUTPUT_FRAME)
		|| (buf_type == IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)) {
			rc = host2sp_enqueue_buffer(thread_id,
				0,
				queue_id,
				(uint32_t)h_vbuf->vptr);
	}

	err = (rc == true) ?
		IA_CSS_SUCCESS : IA_CSS_ERR_QUEUE_IS_FULL;

	if (err == IA_CSS_SUCCESS) {
		for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
			if (hmm_buffer_record_h[i] == NULL) {
				hmm_buffer_record_h[i] = h_vbuf;
				break;
			}
		}
	} else {
		ia_css_i_host_rmgr_rel_vbuf(hmm_buffer_pool, &h_vbuf);
	}

		/*
		 * Tell the SP which queues are not empty,
		 * by sending the software event.
		 */
	if (err == IA_CSS_SUCCESS)
		sh_css_sp_snd_event(SP_SW_EVENT_ID_1,
				thread_id,
				queue_id,
				0);

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_enqueue_buffer() leave: return_err=%d\n",err);
	return err;
}

/*
 * TODO: Free up the hmm memory space.
	 */
enum ia_css_err
ia_css_pipe_dequeue_buffer(struct ia_css_pipe *pipe,
			   struct ia_css_buffer *buffer)
{
	enum ia_css_err err;
	enum sh_css_buffer_queue_id queue_id;
	hrt_vaddress ddr_buffer_addr;
	struct sh_css_hmm_buffer ddr_buffer;
	bool rc;
	unsigned int i, found_record;
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id pipe_id;
	struct sh_css_pipe *old_pipe = NULL;

	assert(buffer != NULL);

	if (pipe)
		old_pipe = pipe->old_pipe;

	if (old_pipe)
		pipe_id = old_pipe->mode;
	else
		pipe_id = IA_CSS_PIPE_ID_COPY;

	buf_type = buffer->type;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_dequeue_buffer() enter: pipe_id=%d, buf_type=%d\n",
		(int)pipe_id, buf_type);

	ddr_buffer.kernel_ptr = 0;

	sh_css_query_internal_queue_id(buf_type, &queue_id);

	rc = sp2host_dequeue_buffer(0,
				0,
				queue_id,
				&ddr_buffer_addr);
	if (rc) {
		struct ia_css_frame *frame;
		mmgr_load(ddr_buffer_addr,
				&ddr_buffer,
				sizeof(struct sh_css_hmm_buffer));
		found_record = 0;
		for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
			if (hmm_buffer_record_h[i] != NULL && hmm_buffer_record_h[i]->vptr == ddr_buffer_addr) {
				ia_css_i_host_rmgr_rel_vbuf(hmm_buffer_pool, &hmm_buffer_record_h[i]);
				hmm_buffer_record_h[i] = NULL;
				found_record = 1;
				break;
			}
		}
		assert(found_record == 1);
		assert(ddr_buffer.kernel_ptr != 0);

		if (ddr_buffer.kernel_ptr == 0)
			rc = false;

		switch (buf_type) {
		case IA_CSS_BUFFER_TYPE_OUTPUT_FRAME:
			if (pipe->stop_requested == true)
			{
				free_mipi_frames(old_pipe);
				pipe->stop_requested = false;
			}
		case IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
			frame = (struct ia_css_frame*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
			buffer->data.frame = frame;
			if (ddr_buffer.payload.frame.exp_id)
				frame->exp_id = ddr_buffer.payload.frame.exp_id;
			if (ddr_buffer.payload.frame.flashed == 1)
				frame->flash_state =
					IA_CSS_FRAME_FLASH_STATE_PARTIAL;
			if (ddr_buffer.payload.frame.flashed == 2)
				frame->flash_state =
					IA_CSS_FRAME_FLASH_STATE_FULL;
			if (old_pipe) {
				frame->valid = old_pipe->num_invalid_frames == 0;
				if (!frame->valid)
					old_pipe->num_invalid_frames--;
				if (frame->info.format ==
						IA_CSS_FRAME_FORMAT_BINARY_8) {
					frame->planes.binary.size =
					    sh_css_sp_get_binary_copy_size();
				}
			}
		
			break;
		case IA_CSS_BUFFER_TYPE_3A_STATISTICS:
			buffer->data.stats_3a =
				(struct ia_css_isp_3a_statistics*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
			break;
		case IA_CSS_BUFFER_TYPE_DIS_STATISTICS:
			buffer->data.stats_dvs =
				(struct ia_css_isp_dvs_statistics*)HOST_ADDRESS(ddr_buffer.kernel_ptr);
			break;
		default:
			rc = false;
			break;
		}
	}

	err = rc ? IA_CSS_SUCCESS : IA_CSS_ERR_QUEUE_IS_EMPTY;

	/*
	 * Tell the SP which queues are not full,
	 * by sending the software event.
	 */
	if (err == IA_CSS_SUCCESS)
		sh_css_sp_snd_event(SP_SW_EVENT_ID_2,
				0,
				queue_id,
				0);

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_dequeue_buffer() leave: buffer=%p\n", buffer);

	return err;
}

static enum ia_css_err
ia_css_pipe_dequeue_unused_buffer(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	enum sh_css_buffer_queue_id queue_id;
	hrt_vaddress ddr_buffer_addr;
	struct sh_css_hmm_buffer ddr_buffer;
	bool rc;
	unsigned int i, found_record, thread_id;
	enum ia_css_pipe_id pipe_id;
	struct sh_css_pipe *old_pipe = NULL;
	struct sh_css_ddr_address_map map;
	hrt_vaddress *addrs = (hrt_vaddress *)&map;

	if (pipe)
	       old_pipe = pipe->old_pipe;

	if (old_pipe)
		pipe_id = old_pipe->mode;
	else
		pipe_id = IA_CSS_PIPE_ID_COPY;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_dequeue_unused_buffer() enter: pipe_id=%d\n",
		(int)pipe_id);

	ddr_buffer.kernel_ptr = 0;

	sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);

    /* for frame buffers */
    for (queue_id = 0; queue_id < (SH_CSS_NUM_BUFFER_QUEUES - 2); queue_id++) {
		rc = host2sp_dequeue_buffer(thread_id,
				0,
				queue_id,
				&ddr_buffer_addr);
		if (rc) {
			mmgr_load(ddr_buffer_addr,
				&ddr_buffer,
				sizeof(struct sh_css_hmm_buffer));
			found_record = 0;
			for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
				if (hmm_buffer_record_h[i] != NULL && hmm_buffer_record_h[i]->vptr == ddr_buffer_addr) {
					ia_css_i_host_rmgr_rel_vbuf(hmm_buffer_pool, &hmm_buffer_record_h[i]);
					hmm_buffer_record_h[i] = NULL;
					found_record = 1;
					break;
				}
			}
			assert(found_record == 1);
			assert(ddr_buffer.kernel_ptr != 0);

			if (ddr_buffer.kernel_ptr == 0)
				return IA_CSS_ERR_INTERNAL_ERROR;
		} else {
			continue;
		}
	}

	/* for param buffers */
	rc = host2sp_dequeue_buffer(thread_id,
				0,
				queue_id,
				&ddr_buffer_addr);
	if (rc) {
		mmgr_load(ddr_buffer_addr, &map, sizeof(struct sh_css_ddr_address_map));
		/* copy map using size info */
		for (i = 0; i < (sizeof(struct sh_css_ddr_address_map_size)/
						sizeof(size_t)); i++) {
			if (addrs[i] == mmgr_NULL)
				continue;
			sh_css_refcount_release(PARAM_BUFFER, addrs[i]);
		}
		sh_css_refcount_release(PARAM_SET_POOL, ddr_buffer_addr);
	}

	/* for tag command */

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_dequeue_buffer() leave:\n");

	return err;
}

static void decode_sp_event(
	uint32_t event,
	uint8_t *port_id,
	enum ia_css_pipe_id *pipe_id,
	uint8_t *pipe_num,
	enum ia_css_event_type *event_id)
{
	enum ia_css_event_type event_code = IA_CSS_EVENT_TYPE_NUM;
	enum sh_css_sp_event_type sh_event_id;

	assert(pipe_id != NULL);
	assert(event_id != NULL);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "decode_sp_event() enter:\n");

	/* First decode according to the common case
	 * In case of a PORT_EOF event we overwrite with 
	 * the specific values
	 * This is somewhat ugly but probably somewhat efficient 
	 * (and it avoids some code duplication)
	 */
	*port_id = 0;
	*pipe_id = (enum ia_css_pipe_id)((event >> 16) & 0xff);
	*pipe_num = (event >> 8) & 0xff;
	sh_event_id = event & 0xff;

	/* convert event_code from sp (SH) domain to host (IA) domain */
	switch (sh_event_id) {
	case SH_CSS_SP_EVENT_OUTPUT_FRAME_DONE:
		event_code = IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE;
		break;
	case SH_CSS_SP_EVENT_VF_OUTPUT_FRAME_DONE:
		event_code = IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE;
		break;
	case SH_CSS_SP_EVENT_3A_STATISTICS_DONE:
		event_code = IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE;
		break;
	case SH_CSS_SP_EVENT_DIS_STATISTICS_DONE:
		event_code = IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE;
		break;
	case SH_CSS_SP_EVENT_PIPELINE_DONE:
		event_code = IA_CSS_EVENT_TYPE_PIPELINE_DONE;
		break;
	case SH_CSS_SP_EVENT_PORT_EOF:
		*port_id = (event >> 8) & 0xff;
		*pipe_id = 0;
		*pipe_num = 0;
		event_code = IA_CSS_EVENT_TYPE_PORT_EOF;
		break;
	default:
		break;
	}
	*event_id = event_code;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "decode_sp_event() leave:\n");
}


enum ia_css_err
ia_css_dequeue_event(struct ia_css_event *event)
{
	bool is_event_available;
	uint32_t sp_event;
	enum ia_css_pipe_id pipe_id;
	uint8_t pipe_num = 0;

	assert(event);

	/* dequeue the IRQ event */
	is_event_available = sp2host_dequeue_irq_event(&sp_event);

	/* check whether the IRQ event is available or not */
	if (!is_event_available) {
		//sh_css_dtrace(SH_DBG_TRACE,
		//      "ia_css_dequeue_event() out: EVENT_QUEUE_EMPTY\n");
		return IA_CSS_ERR_QUEUE_IS_EMPTY;
	} else {
		/*
		 * Tell the SP which queues are not full,
		 * by sending the software event.
		 */
		sh_css_sp_snd_event(SP_SW_EVENT_ID_3, 0, 0, 0);
	}

	decode_sp_event(sp_event, &event->port, &pipe_id, &pipe_num, &event->type);
	
	if (event->type != IA_CSS_EVENT_TYPE_PORT_EOF) {
		/* pipe related events */
		event->pipe = find_pipe_by_num(pipe_num);
		if (!event->pipe)
		{
			/* an event is generated of a port that is not longer available*/
			return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
		}
	}
	else {
		/* event is not directly correlated to a pipe */
		event->pipe = NULL;
	}
#if 0
  /* temporary debug code to dump the intermediate vf buffer to raw file*/
	if (event->type == IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE)
	{
	  static int count = 0;
	  FILE* fp;
    char *xmem_y_addr  = (char*)(event->pipe->old_pipe->pipeline.stages->args.out_vf_frame->data + event->pipe->old_pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.offset);
    unsigned stride = event->pipe->old_pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.stride;
    unsigned height = event->pipe->old_pipe->pipeline.stages->args.out_vf_frame->planes.yuyv.height;
    //printf ("init_frame_pointers format %d\n", stage->frames.in.info.format);
    //if (stage->frames.in.info.format == IA_CSS_FRAME_FORMAT_YUV_LINE) {
    unsigned char pixel;
    unsigned char buf[5000];
    unsigned int i;
    mmgr_load(xmem_y_addr, &pixel, sizeof(char));
    printf ("vf_pp: line 0 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+stride, &pixel, sizeof(char));
    printf ("vf_pp: line 1 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+2*stride, &pixel, sizeof(char));
    printf ("vf_pp: line 2 pixel %d\n", pixel);
    mmgr_load(xmem_y_addr+3*stride, &pixel, sizeof(char));
    printf ("vf_pp: line 3 pixel %d\n", pixel);

    fp = fopen("dump.raw", "w+");

    for (i=0; i< height/2; i++)
    {
      mmgr_load(xmem_y_addr, buf, stride);
      fwrite(buf, 1, stride, fp);
      xmem_y_addr += stride;
      mmgr_load(xmem_y_addr, buf, stride);
      fwrite(buf, 1, stride, fp);
      xmem_y_addr += stride;
      xmem_y_addr += stride;

    }
    fclose(fp);
  
    count++;
	}
#endif
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_dequeue_event() leave: pipe_id=%d, event_id=%d\n",
				pipe_id, event->type);

	return IA_CSS_SUCCESS;
}

static void
acc_start(struct sh_css_pipe *pipe)
{
	sh_css_start_pipeline(pipe->mode, &pipe->pipeline);
//	while(!ia_css_sp_has_initialized())
//		hrt_sleep();
//	sh_css_init_host_sp_control_vars();
//	sh_css_init_buffer_queues();
	start_pipe(pipe, SH_CSS_PIPE_CONFIG_OVRD_NO_OVRD, pipe->stream->config.mode);
}

static enum ia_css_err
sh_css_pipe_start(struct ia_css_stream *stream)
{
	enum ia_css_err err;
	struct ia_css_pipe *pipe = stream->last_pipe;
	struct sh_css_pipe *old_pipe = pipe->old_pipe;
	enum ia_css_pipe_id pipe_id = old_pipe->mode;
	unsigned int thread_id;
	//static bool init_queues = true; /* Workaround */

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_start() enter: pipe_id=%d\n", pipe_id);
#ifdef __KERNEL__
	printk("sh_css_pipe_start() enter: pipe_id=%d\n", pipe_id);
#endif

	old_pipe->pipe_num = pipe->pipe_num;
	pipe->stop_requested = false;

	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		err = preview_start(old_pipe);
		break;
	case IA_CSS_PIPE_ID_VIDEO:
		err = video_start(old_pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		err = capture_start(old_pipe);
		break;
	case IA_CSS_PIPE_ID_ACC:
		acc_start(old_pipe);
		err = IA_CSS_SUCCESS;
		break;
	default:
		err = IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (err != IA_CSS_SUCCESS) {
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_start() leave: return_err=%d\n", err);
		return err;
	}
	
	/* Force ISP parameter calculation after a mode change 
	 * Acceleration API examples pass NULL for stream but they
	 * don't use ISP parameters anyway. So this should be okay.
	 * The SP binary (jpeg) copy does not use any parameters.
	 */
	if (stream && !copy_on_sp(old_pipe)) {
		sh_css_invalidate_params(stream);
		sh_css_param_update_isp_params(stream, true);
	}

	sh_css_debug_pipe_graph_dump_epilogue();

	sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);
	sh_css_sp_snd_event(SP_SW_EVENT_ID_4, thread_id, 0,  0);

	/* in case of continuous capture mode, we also start capture thread and copy thread*/
	if (old_pipe->stream->config.continuous) {
		struct sh_css_pipe *copy_pipe = NULL;
		if (pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			copy_pipe = pipe->old_pipe->pipe.preview.copy_pipe;
		else if (pipe_id == IA_CSS_PIPE_ID_VIDEO)
			copy_pipe = pipe->old_pipe->pipe.video.copy_pipe;
		
		assert(copy_pipe != NULL);
		sh_css_query_sp_thread_id(copy_pipe->pipe_num, &thread_id);
		sh_css_sp_snd_event(SP_SW_EVENT_ID_4, thread_id, 0,  0);
	}
	if (old_pipe->stream->cont_capt) {
		struct sh_css_pipe *capture_pipe = NULL;
		if (pipe_id == IA_CSS_PIPE_ID_PREVIEW)
			capture_pipe = pipe->old_pipe->pipe.preview.capture_pipe;
		else if (pipe_id == IA_CSS_PIPE_ID_VIDEO)
			capture_pipe = pipe->old_pipe->pipe.video.capture_pipe;

		assert(capture_pipe != NULL);
		sh_css_query_sp_thread_id(capture_pipe->pipe_num, &thread_id);
		sh_css_sp_snd_event(SP_SW_EVENT_ID_4, thread_id, 0,  0);
	}

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_start() leave: return_err=%d\n", err);
	return err;
}

#if 0
static enum ia_css_err sh_css_pipeline_stop(
	struct sh_css_pipe *pipe)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipeline_stop() enter\n");
	(void)pipe;
	/* TO BE IMPLEMENTED */
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipeline_stop() exit\n");
	return IA_CSS_SUCCESS;
}
#endif

static enum ia_css_err
sh_css_pipe_get_input_resolution(struct sh_css_pipe *pipe,
				 unsigned int *width,
				 unsigned int *height)
{
	enum ia_css_err err;

	assert(width != NULL);
	assert(height != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipe_get_input_resolution() enter: void\n");
	if (pipe->mode == IA_CSS_PIPE_ID_CAPTURE && copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		*width = JPEG_BYTES;
		*height = 1;
		return IA_CSS_SUCCESS;
	}


	err = sh_css_pipe_load_binaries(pipe);
	if (err == IA_CSS_SUCCESS) {
		const struct sh_css_binary *binary = NULL;
		if (pipe->mode == IA_CSS_PIPE_ID_PREVIEW) {
			sh_css_dtrace(SH_DBG_TRACE,
				"sh_css_pipe_get_input_resolution: preview\n");
			if (pipe->pipe.preview.copy_binary.info)
				binary = &pipe->pipe.preview.copy_binary;
			else
				binary = &pipe->pipe.preview.preview_binary;

		}
		else if (pipe->mode == IA_CSS_PIPE_ID_VIDEO) {
			sh_css_dtrace(SH_DBG_TRACE,
				"sh_css_pipe_get_input_resolution: video\n");
			if (pipe->pipe.video.copy_binary.info)
				binary = &pipe->pipe.video.copy_binary;
			else
				binary = &pipe->pipe.video.video_binary;
		}
		else if (pipe->mode == IA_CSS_PIPE_ID_CAPTURE) {
			sh_css_dtrace(SH_DBG_TRACE,
				"sh_css_pipe_get_input_resolution: capture\n");
			if (pipe->pipe.capture.copy_binary.info)
				binary = &pipe->pipe.capture.copy_binary;
			else if (pipe->pipe.capture.primary_binary.info)
				binary = &pipe->pipe.capture.primary_binary;
			else
				binary = &pipe->pipe.capture.pre_isp_binary;
		}
		*width  = binary->in_frame_info.res.width +
			columns_needed_for_bayer_order(pipe);
		*height = binary->in_frame_info.res.height +
			lines_needed_for_bayer_order(pipe);
	/* TODO: Remove this when the decimated resolution is available */
	/* Only for continuous preview mode where we need 2xOut resolution */
		if (pipe->input_needs_raw_binning &&
		pipe->pipe.preview.preview_binary.info->enable.raw_binning) {
			*width *= 2;
			*width -= binary->info->left_cropping;
			*height *= 2;
			*height -= binary->info->left_cropping;
		}
	}
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_get_input_resolution() leave: width=%d, height=%d err=%d\n",
		*width, *height, err);
	return err;
}

void
sh_css_enable_cont_capt(bool enable, bool stop_copy_preview)
{
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_enable_cont_capt() enter: enable=%d\n", enable);
	//my_css.cont_capt = enable;
	my_css.stop_copy_preview = stop_copy_preview;
}

bool
sh_css_continuous_is_enabled(uint8_t pipe_num)
{
	struct ia_css_pipe *pipe;
	bool continuous;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_continuous_is_enabled() enter: void\n");
	
	pipe = find_pipe_by_num(pipe_num);
	continuous = pipe && pipe->old_pipe->stream->config.continuous;
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_continuous_is_enabled() leave: enable=%d\n",
		continuous);
	return continuous;
}

enum ia_css_err
ia_css_stream_get_max_buffer_depth(struct ia_css_stream *stream, int *buffer_depth)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_get_max_buffer_depth() enter: void\n");
(void)stream;
	*buffer_depth = NUM_CONTINUOUS_FRAMES;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_set_buffer_depth(struct ia_css_stream *stream, int buffer_depth)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_set_buffer_depth() enter: num_frames=%d\n",buffer_depth);
(void)stream;
	if (buffer_depth > NUM_CONTINUOUS_FRAMES || buffer_depth < 1)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	/* ok, value allowed */
	my_css.num_cont_raw_frames = buffer_depth;
	// TODO: check what to regarding initialization
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_get_buffer_depth(struct ia_css_stream *stream, int *buffer_depth)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_get_buffer_depth() enter: void\n");
(void)stream;
	*buffer_depth = my_css.num_cont_raw_frames;
	return IA_CSS_SUCCESS;
}

bool
sh_css_continuous_start_sp_copy(void)
{
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_continuous_start_sp_copy() enter: void\n");
	return my_css.start_sp_copy;
}

static enum ia_css_err sh_css_pipe_configure_output(
	struct sh_css_pipe *pipe,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)

{
	enum ia_css_err err = IA_CSS_SUCCESS;
assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_configure_output() enter:\n");

	err = check_res(width, height);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (pipe->output_info.res.width != width ||
	    pipe->output_info.res.height != height ||
	    pipe->output_info.format != format) {
		sh_css_frame_info_init(&pipe->output_info, width, height, format);
		sh_css_pipe_invalidate_binaries(pipe);
	}
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_pipe_get_grid_info(struct sh_css_pipe *pipe,
			  struct ia_css_grid_info *info)
{
	enum ia_css_err err;
	struct sh_css_binary *s3a_binary = NULL;

	assert(pipe != NULL);
	assert(info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_get_grid_info() enter:\n");

	err = sh_css_pipe_load_binaries(pipe);
	if (err == IA_CSS_SUCCESS) {	
		s3a_binary = ia_css_pipe_get_3a_binary(pipe->new_pipe);
		if (s3a_binary)
			sh_css_binary_grid_info(s3a_binary, info);
		else
			memset(info, 0, sizeof(*info));
	}
	return err;
}

static void init_video_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *vf_info)
{
	int mode = SH_CSS_BINARY_MODE_VIDEO;
	bool stream_dz_config = false;
/* vf_info can be NULL */
assert(pipe != NULL);
assert(in_info != NULL);
/* assert(vf_info != NULL); */
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_video_descr() enter:\n");

	if (input_format_is_yuv(pipe->stream->config.format))
		mode = SH_CSS_BINARY_MODE_COPY;
	in_info->res = pipe->stream->info.effective_info;
	in_info->padded_width = in_info->res.width;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	init_offline_descr(pipe,
			   &video_descr,
			   mode,
			   in_info,
			   &pipe->output_info,
			   vf_info);
	if (pipe->stream->config.online) {
		video_descr.online	    = pipe->stream->config.online;
		video_descr.two_ppc       = pipe->stream->config.two_pixels_per_clock;
	}
	if (mode == SH_CSS_BINARY_MODE_VIDEO) {
		stream_dz_config = ((pipe->stream->isp_params_configs->dz_config.dx != HRT_GDC_N)
				 || (pipe->stream->isp_params_configs->dz_config.dy != HRT_GDC_N));

		video_descr.enable_dz           = pipe->enable_dz || stream_dz_config;
		video_descr.dvs_env             = pipe->dvs_envelope;
		video_descr.enable_yuv_ds       = pipe->enable_yuv_ds;
		video_descr.enable_high_speed   = pipe->enable_high_speed;
		video_descr.enable_dvs_6axis    = pipe->enable_dvs_6axis;
		video_descr.enable_reduced_pipe = pipe->enable_reduced_pipe;
		video_descr.isp_pipe_version    = pipe->isp_pipe_version;
		video_descr.binning             = pipe->input_needs_raw_binning;
	}
}


/*
 * @GC: TEMPORARY CODE TO TEST DVS AGAINST THE REFERENCE
 * PLEASE DO NOT REMOVE IT!
 */
#if DVS_REF_TESTING
static enum ia_css_err alloc_frame_from_file(
	struct sh_css_pipe *pipe,
	int width,
	int height)
{
	FILE *fp;
	int len = 0, err;
	int bytes_per_pixel;
	const char *file = "../File_input/dvs_input2.yuv";
	char *y_buf, *u_buf, *v_buf;
	char *uv_buf;
	int offset = 0;
	int h, w;
	hrt_vaddress out_base_addr = pipe->pipe.video.ref_frames[0]->data;
	hrt_vaddress out_y_addr  = out_base_addr
		+ pipe->pipe.video.ref_frames[0]->planes.yuv.y.offset;
	hrt_vaddress out_uv_addr = out_base_addr
		+ pipe->pipe.video.ref_frames[0]->planes.yuv.u.offset;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "alloc_frame_from_file() enter:\n");

	bytes_per_pixel = sizeof(char);

	if (!file) {printf("Error: Input file for dvs is not specified\n"); return 1;}
	fp = fopen(file, "rb");
	if (!fp) {printf("Error: Input file for dvs is not found\n"); return 1;}

	err = fseek(fp, 0, SEEK_END);
	if (err) {
		fclose(fp);
	  	printf("Error: Fseek error\n");
	  	return 1;
	}
	len = ftell(fp);

	err = fseek(fp, 0, SEEK_SET);
	if (err) {
		fclose(fp);
		printf("Error: Fseek error2\n");
		return 1;
	}

	len = 2 * len / 3;
	if (len != width * height * bytes_per_pixel) {
		fclose(fp);
		printf("Error: File size mismatches with the internal resolution\n");
		return 1;
	}

	y_buf = (char *) malloc(len);
	u_buf = (char *) malloc(len/4);
	v_buf = (char *) malloc(len/4);
	uv_buf= (char *) malloc(len/2);

	fread(y_buf, 1, len, fp);
	fread(u_buf, 1, len/4, fp);
	fread(v_buf, 1, len/4, fp);

	for (h=0; h<height/2; h++) {
		for (w=0; w<width/2; w++) {
			*(uv_buf + offset + w) = *(u_buf++);
			*(uv_buf + offset + w + width/2) = *(v_buf++);
			//printf("width: %d\n", width);
			//printf("offset_u: %d\n", offset+w);
			//printf("offset_v: %d\n", offset+w+width/2);
		}
		offset += width;
	}

	mmgr_store(out_y_addr, y_buf, len);
	mmgr_store(out_uv_addr, uv_buf, len/2);

	out_base_addr = pipe->pipe.video.ref_frames[1]->data;
	out_y_addr  = out_base_addr + pipe->pipe.video.ref_frames[1]->planes.yuv.y.offset;
	out_uv_addr = out_base_addr + pipe->pipe.video.ref_frames[1]->planes.yuv.u.offset;
	mmgr_store(out_y_addr, y_buf, len);
	mmgr_store(out_uv_addr, uv_buf, len/2);

	fclose(fp);

	return IA_CSS_SUCCESS;
}

/* MW: Why do we not pass the pointer to the struct ? */
static enum ia_css_err fill_ref_frame_for_dvs(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info ref_info)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "fill_ref_frame_for_dvs() enter:\n");
	/* Allocate tmp_frame which is used to store YUV420 input.
	 * Read YUV420 input from the file to tmp_frame.
	 * Convert from YUV420 to NV12 format */
	err = alloc_frame_from_file(pipe, ref_info.res.width, ref_info.res.height);

	return err;
}
#endif

#define SH_CSS_TNR_BIT_DEPTH 8
#define SH_CSS_REF_BIT_DEPTH 8

static enum ia_css_err load_video_binaries(
	struct sh_css_pipe *pipe)
{
	struct ia_css_frame_info video_in_info, ref_info, tnr_info,
				 *video_vf_info;
	bool online;
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool continuous = pipe->stream->config.continuous;
	unsigned int i;
	unsigned num_output_pins;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_video_binaries() enter:\n");
	/* we only test the video_binary because offline video doesn't need a
	 * vf_pp binary and online does not (always use) the copy_binary.
	 * All are always reset at the same time anyway.
	 */
	if (pipe->pipe.video.video_binary.info)
		return IA_CSS_SUCCESS;

	online = pipe->stream->config.online;
	err = check_input(pipe, !online);
	if (err != IA_CSS_SUCCESS)
		return err;
	/* cannot have online video and input_mode memory */
	if (online && pipe->stream->config.mode == IA_CSS_INPUT_MODE_MEMORY)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (pipe->enable_viewfinder) {
		err = check_vf_out_info(&pipe->output_info,
					&pipe->vf_output_info);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		err = check_frame_info(&pipe->output_info);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Video */
	if (pipe->enable_viewfinder)
		video_vf_info = &pipe->vf_output_info;
	else
		video_vf_info = NULL;
	init_video_descr(pipe, &video_in_info, video_vf_info);

	err = sh_css_binary_find(&video_descr,
				 &pipe->pipe.video.video_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	num_output_pins = pipe->pipe.video.video_binary.info->num_output_pins;
	/* This is where we set the flag for invalid first frame */
	if (video_vf_info)
		pipe->num_invalid_frames = 2;
	else
		pipe->num_invalid_frames = 1;

	/* Copy */
	if (!online && !continuous) {
		/* TODO: what exactly needs doing, prepend the copy binary to
		 *	 video base this only on !online?
		 */
		err = load_copy_binary(pipe,
				       &pipe->pipe.video.copy_binary,
				       &pipe->pipe.video.video_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* Viewfinder post-processing */
	if (pipe->enable_viewfinder && num_output_pins == 1) { // When the video binary has only one output pin, we need vf_pp to produce the viewfinder output.
		init_vf_pp_descr(pipe,
			&pipe->pipe.video.video_binary.vf_frame_info,
			&pipe->vf_output_info);
		err = sh_css_binary_find(&vf_pp_descr,
				&pipe->pipe.video.vf_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	/* yuv copy does not use reference frames */
	if (input_format_is_yuv(pipe->stream->config.format))
		return IA_CSS_SUCCESS;

	ref_info = pipe->pipe.video.video_binary.internal_frame_info;
	ref_info.format = IA_CSS_FRAME_FORMAT_YUV420;
	ref_info.raw_bit_depth = SH_CSS_REF_BIT_DEPTH;

	for (i = 0; i < NUM_REF_FRAMES; i++) {
		if (pipe->pipe.video.ref_frames[i]) {
			ia_css_frame_free(pipe->pipe.video.ref_frames[i]);
			pipe->pipe.video.ref_frames[i] = NULL;
		}
		err = ia_css_frame_allocate_from_info(
				&pipe->pipe.video.ref_frames[i],
				&ref_info);
		if (SH_CSS_PREVENT_UNINIT_READS) {
			ia_css_frame_zero(pipe->pipe.video.ref_frames[i]);
		}
		if (err != IA_CSS_SUCCESS)
			return err;
	}


#if DVS_REF_TESTING
	/* @GC: TEMPORARY CODE TO TEST DVS AGAINST THE REFERENCE
	 * To test dvs-6axis:
	 * 1. Enable this function call
	 * 2. Set "reqs.ref_out_requests" to "0" in lineloop.hive.c
	 */
	err = fill_ref_frame_for_dvs(pipe, ref_info);
	if (err != IA_CSS_SUCCESS)
		return err;
#endif


  if (pipe->pipe.video.video_binary.info->enable.block_output){
    tnr_info = pipe->pipe.video.video_binary.out_frame_info;
  }
  else {
    tnr_info = pipe->pipe.video.video_binary.internal_frame_info;
  }
	tnr_info.format = IA_CSS_FRAME_FORMAT_YUV_LINE;
	tnr_info.raw_bit_depth = SH_CSS_TNR_BIT_DEPTH;

	for (i = 0; i < NUM_TNR_FRAMES; i++) {
		if (pipe->pipe.video.tnr_frames[i]) {
			ia_css_frame_free(pipe->pipe.video.tnr_frames[i]);
			pipe->pipe.video.tnr_frames[i] = NULL;
		}
		err = ia_css_frame_allocate_from_info(
				&pipe->pipe.video.tnr_frames[i],
				&tnr_info);
		if (SH_CSS_PREVENT_UNINIT_READS) {
			ia_css_frame_zero(pipe->pipe.video.tnr_frames[i]);
		}
		
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	if (pipe->stream->cont_capt) {
		err = alloc_continuous_frames(pipe);
		if (err != IA_CSS_SUCCESS)
			return err;
	
		if (SH_CSS_PREVENT_UNINIT_READS)
			ia_css_frame_zero(pipe->continuous_frames[0]);
	}
	
	err = allocate_mipi_frames(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	return IA_CSS_SUCCESS;
}

static enum ia_css_err video_start(
	struct sh_css_pipe *pipe)
{
	struct sh_css_pipeline *me = &pipe->pipeline;
	struct sh_css_pipeline_stage *copy_stage  = NULL;
	struct sh_css_pipeline_stage *video_stage = NULL;
	struct sh_css_pipeline_stage *vf_pp_stage = NULL;
	struct sh_css_pipeline_stage *in_stage    = NULL;
	struct sh_css_binary *copy_binary, *video_binary, *vf_pp_binary;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_pipe *copy_pipe, *capture_pipe;
	enum sh_css_pipe_config_override copy_ovrd;
	enum ia_css_input_mode video_pipe_input_mode;
	/**
	 * rvanimme: in_frame support is broken and forced to NULL
	 * TODO: add a way to tell the pipeline construction that an in_frame
	 * is used.
	 */
	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *out_frame = &pipe->out_frame_struct;
	struct ia_css_frame *vf_frame = &pipe->vf_frame_struct;
	unsigned num_output_pins;

	pipe->out_frame_struct.data = 0;
	pipe->vf_frame_struct.data = 0;

	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "video_start() enter:\n");

	video_pipe_input_mode = pipe->stream->config.mode;

	copy_pipe    = pipe->pipe.video.copy_pipe;
	capture_pipe = pipe->pipe.video.capture_pipe;

	sh_css_pipe_get_output_frame_info(pipe, &out_frame->info);
	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	out_frame->dynamic_data_index = sh_css_frame_out;
	err = init_frame_planes(out_frame);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (!pipe->enable_viewfinder || in_frame) {
		/* These situations don't support viewfinder output */
		vf_frame = NULL;
	} else {
		sh_css_pipe_get_viewfinder_frame_info(pipe, &vf_frame->info);
		vf_frame->contiguous = false;
		vf_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
		vf_frame->dynamic_data_index = sh_css_frame_out_vf;
		err = init_frame_planes(vf_frame);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	copy_stage = NULL;
	in_stage = NULL;

	if (pipe->zoom_changed) {
		sh_css_pipe_invalidate_binaries(pipe);
		pipe->zoom_changed = false;
	}

	err = sh_css_pipe_load_binaries(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	copy_binary  = &pipe->pipe.video.copy_binary;
	video_binary = &pipe->pipe.video.video_binary;
	vf_pp_binary = &pipe->pipe.video.vf_pp_binary;
	num_output_pins = video_binary->info->num_output_pins;

	sh_css_metrics_start_frame();

	if (me->reload) {
		sh_css_pipeline_clean(me);
		if (pipe->pipe.video.copy_binary.info) {
			err = sh_css_pipeline_add_stage(me, copy_binary,
				/* TODO: check next params */
				/* const struct ia_css_acc_fw *firmware, */
				NULL,
				/* unsigned int mode, */
				copy_binary->info->mode, /* unsigned int mode,*/
				/* struct ia_css_frame *cc_frame, */
				NULL,
				/* struct ia_css_frame *in_frame, */
				NULL,
				/* struct ia_css_frame *out_frame, */
				NULL,
				/* struct ia_css_frame *vf_frame, */
				NULL,
				/* struct sh_css_pipeline_stage **stage) */
				&copy_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
			in_frame = me->stages->args.out_frame;
			in_stage = copy_stage;
		} else if (pipe->stream->cont_capt) {
			in_frame = pipe->continuous_frames[0];
		}
		err = sh_css_pipeline_add_stage(me, video_binary, NULL,
						video_binary->info->mode, NULL,
						in_frame, out_frame, num_output_pins > 1 ? vf_frame : NULL,// when the video binary supports a second output pin, it can directly produce the vf_frame.
						&video_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		/* If we use copy iso video, the input must be yuv iso raw */
		video_stage->args.copy_vf =
			video_binary->info->mode == SH_CSS_BINARY_MODE_COPY;
		video_stage->args.copy_output = video_stage->args.copy_vf;
		if (!in_frame && pipe->enable_viewfinder && num_output_pins == 1) { // when the video binary supports only 1 output pin, vf_pp is needed to produce the vf_frame.
			err = add_vf_pp_stage(pipe, vf_frame, vf_pp_binary,
					      video_stage, &vf_pp_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		number_stages(pipe);
	} else {
		sh_css_pipeline_restart(me);
	}

	err = sh_css_pipeline_get_stage(me, video_binary->info->mode,
					&video_stage);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (!in_stage)
		in_stage = video_stage;


	if (!in_frame && pipe->enable_viewfinder && num_output_pins == 1) {// when the video binary supports only 1 output pin, vf_pp is needed to produce the vf_frame.
		err = sh_css_pipeline_get_output_stage(me,
						       vf_pp_binary->info->mode,
						       &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	video_stage->args.in_ref_frame = pipe->pipe.video.ref_frames[0];
	video_stage->args.out_ref_frame = pipe->pipe.video.ref_frames[1];
	video_stage->args.in_tnr_frame = pipe->pipe.video.tnr_frames[0];
	video_stage->args.out_tnr_frame = pipe->pipe.video.tnr_frames[1];

	/* multi stream video needs mipi buffers */
	send_mipi_frames(pipe);

	/* update the arguments with the latest info */
	video_stage->args.out_frame = out_frame;

	if (vf_pp_stage)
		vf_pp_stage->args.out_frame = vf_frame;

	if (pipe->stream->config.continuous) {
		int i;
		in_frame = pipe->continuous_frames[0];
		video_stage->args.in_frame = in_frame;

		/* Hand-over all the SP-internal buffers */
		for (i = 0; i < NUM_CONTINUOUS_FRAMES; i++) {
			sh_css_update_host2sp_offline_frame(i,
				pipe->continuous_frames[i]);
		}
	}

	if (pipe->stream->config.online)
		sh_css_set_irq_buffer(in_stage, sh_css_frame_in, in_frame);
	sh_css_set_irq_buffer(video_stage, sh_css_frame_out,    out_frame);
	if (vf_pp_stage)
		sh_css_set_irq_buffer(vf_pp_stage, sh_css_frame_out_vf,
					vf_frame);

	{
		unsigned int thread_id;
		
		sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);
		copy_ovrd = 1 << thread_id;
		
		if (pipe->stream->cont_capt) {
			sh_css_query_sp_thread_id(capture_pipe->pipe_num, &thread_id);
			copy_ovrd |= 1 << thread_id;
		}
	}

	/* Construct and load the copy pipe */
	if (pipe->stream->config.continuous) {
		err = construct_copy_pipe(copy_pipe,
					  video_binary->info->max_input_width,
					  pipe->continuous_frames[0]);
		if (err != IA_CSS_SUCCESS)
			return err;

		sh_css_sp_init_pipeline(&copy_pipe->pipeline, IA_CSS_PIPE_ID_COPY,
			copy_pipe->pipe_num,
			false, false, false,
			pipe->stream->config.two_pixels_per_clock, false,
			false, pipe->input_needs_raw_binning,
			copy_ovrd,
			pipe->stream->config.mode,
			pipe->stream->config.source.port.port);

		/* make the video pipe start with mem mode input, copy handles
		   the actual mode */
		video_pipe_input_mode = IA_CSS_INPUT_MODE_MEMORY;
	}
	
	/* Construct and load the capture pipe */
	if (pipe->stream->cont_capt) {
		bool low_light;

		err = construct_capture_pipe(capture_pipe);
		if (err != IA_CSS_SUCCESS)
			return err;

		low_light = (capture_pipe->capture_mode ==
				IA_CSS_CAPTURE_MODE_LOW_LIGHT) ||
				(capture_pipe->capture_mode ==
				IA_CSS_CAPTURE_MODE_BAYER);

		sh_css_sp_init_pipeline(&capture_pipe->pipeline, IA_CSS_PIPE_ID_CAPTURE,
			capture_pipe->pipe_num,
			false, low_light, pipe->xnr,
			capture_pipe->stream->config.two_pixels_per_clock,
			true, /* continuous */
			false, /* offline */
			capture_pipe->input_needs_raw_binning,
			0,
			IA_CSS_INPUT_MODE_MEMORY,
			(mipi_port_ID_t)0);
	}

	err = sh_css_config_input_network(pipe, copy_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	start_pipe(pipe, copy_ovrd, video_pipe_input_mode);
	me->reload = false;

	return IA_CSS_SUCCESS;
}

static
enum ia_css_err sh_css_pipe_get_viewfinder_frame_info(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *info)
{
	enum ia_css_err err;
assert(info != NULL);
/* We could print the pointer as input arg, and the values as output */
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipe_get_viewfinder_frame_info() enter: void\n");

	err = sh_css_pipe_load_binaries(pipe);
	if (err != IA_CSS_SUCCESS) {
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_get_viewfinder_frame_info() leave: return_err=%d\n",
			err);
		return err;
	}
	if ( pipe->mode == IA_CSS_PIPE_ID_CAPTURE &&
	    (pipe->capture_mode == IA_CSS_CAPTURE_MODE_RAW ||
	     pipe->capture_mode == IA_CSS_CAPTURE_MODE_BAYER))
		return IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER;
	/* offline video does not generate viewfinder output */
	if ( pipe->mode == IA_CSS_PIPE_ID_VIDEO &&
	    !pipe->stream->config.online && !pipe->stream->config.continuous) {
		sh_css_dtrace(SH_DBG_TRACE,
			"sh_css_pipe_get_viewfinder_frame_info() leave: return_err=%d\n",
			IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER);
		return IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER;
	} else {
		*info = pipe->vf_output_info;
	}
		
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_get_viewfinder_frame_info() leave: \
		info.res.width=%d, info.res.height=%d, \
		info.padded_width=%d, info.format=%d, \
		info.raw_bit_depth=%d, info.raw_bayer_order=%d\n",
		info->res.width,info->res.height,
		info->padded_width,info->format,
		info->raw_bit_depth,info->raw_bayer_order);

	return IA_CSS_SUCCESS;
}

static enum ia_css_err
sh_css_pipe_configure_viewfinder(struct sh_css_pipe *pipe,
				 unsigned int width,
				 unsigned int height,
				 enum ia_css_frame_format format)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipe_configure_viewfinder() enter: \
		width=%d, height=%d format=%d\n",
		width, height, format);

	err = check_res(width, height);
	if (err != IA_CSS_SUCCESS) {
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipe_configure_viewfinder() leave: return_err=%d\n",err);
		return err;
	}
	if (pipe->vf_output_info.res.width != width ||
	    pipe->vf_output_info.res.height != height ||
	    pipe->vf_output_info.format != format) {
		sh_css_frame_info_init(&pipe->vf_output_info,
				       width, height, format);
		sh_css_pipe_invalidate_binaries(pipe);
	}
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_pipe_configure_viewfinder() leave: return_err=%d\n",IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

static enum ia_css_err load_copy_binaries(
	struct sh_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_copy_binaries() enter:\n");

	if (pipe->pipe.capture.copy_binary.info)
		return IA_CSS_SUCCESS;

	err = check_frame_info(&pipe->output_info);
	if (err != IA_CSS_SUCCESS)
		return err;

	err = verify_copy_out_frame_format(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	return load_copy_binary(pipe,
				&pipe->pipe.capture.copy_binary,
				NULL);
}

static bool need_capture_pp(
	const struct sh_css_pipe *pipe)
{
	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "need_capture_pp() enter:\n");
	/* determine whether we need to use the capture_pp binary.
	 * This is needed for:
	 *   1. XNR or
	 *   2. Digital Zoom or
	 *   3. YUV downscaling
	 *   4. in continuous capture mode
	 */
	if (pipe->yuv_ds_input_info.res.width &&
	    ((pipe->yuv_ds_input_info.res.width != pipe->output_info.res.width) ||
	     (pipe->yuv_ds_input_info.res.height != pipe->output_info.res.height)))
		return true;
	if (pipe->xnr)
		return true;

	if ((pipe->stream->isp_params_configs->dz_config.dx < HRT_GDC_N) ||
	    (pipe->stream->isp_params_configs->dz_config.dy < HRT_GDC_N))
		return true;

	/* check if we are trying to a 'digital zoom' */
	if (true == pipe->enable_dz)
		return true;

	/*if (my_css.cont_capt)
		return true;*/

	return false;
}

static void init_capture_pp_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *vf_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(vf_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_capture_pp_descr() enter:\n");

	/* the in_info is only used for resolution to enable
	   bayer down scaling. */
	if (pipe->yuv_ds_input_info.res.width)
		*in_info = pipe->yuv_ds_input_info;
	else
		*in_info = pipe->output_info;
	in_info->format = IA_CSS_FRAME_FORMAT_YUV420;
	in_info->raw_bit_depth = 0;
	sh_css_frame_info_set_width(in_info, in_info->res.width);
	init_offline_descr(pipe,
			   &capture_pp_descr,
			   SH_CSS_BINARY_MODE_CAPTURE_PP,
			   in_info,
			   &pipe->output_info,
			   vf_info);
}

static void init_primary_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info,
	struct ia_css_frame_info *vf_info)
{
	int mode = SH_CSS_BINARY_MODE_PRIMARY;

assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
assert(vf_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_primary_descr() enter:\n");

	if (input_format_is_yuv(pipe->stream->config.format))
		mode = SH_CSS_BINARY_MODE_COPY;

	in_info->res = pipe->stream->info.effective_info;
	in_info->padded_width = in_info->res.width;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	init_offline_descr(pipe,
			   &prim_descr,
			   mode,
			   in_info,
			   out_info,
			   vf_info);
	if (pipe->stream->config.online) {
		prim_descr.online        = true;
		prim_descr.two_ppc       = pipe->stream->config.two_pixels_per_clock;
		prim_descr.stream_format = pipe->stream->config.format;
	}
	if (mode == SH_CSS_BINARY_MODE_PRIMARY) {
		prim_descr.isp_pipe_version = pipe->isp_pipe_version;
	}
}

static void init_pre_gdc_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_pre_gdc_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	init_offline_descr(pipe,
			   &pre_gdc_descr, SH_CSS_BINARY_MODE_PRE_ISP,
			   in_info, out_info, NULL);
	pre_gdc_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

static void
init_gdc_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_gdc_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_QPLANE6;
	init_offline_descr(pipe,
			   &gdc_descr, SH_CSS_BINARY_MODE_GDC,
			   in_info, out_info, NULL);
}

static void init_post_gdc_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info,
	struct ia_css_frame_info *vf_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
assert(vf_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_post_gdc_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_YUV420_16;
	init_offline_descr(pipe,
			   &post_gdc_descr, SH_CSS_BINARY_MODE_POST_ISP,
			   in_info, out_info, vf_info);
	post_gdc_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

static void init_pre_anr_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_pre_anr_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	if (pipe->isp_pipe_version == 1) {
		init_offline_descr(pipe,
			   &pre_anr_descr, SH_CSS_BINARY_MODE_PRE_ISP,
			   in_info, out_info, NULL);
	} else {
		init_offline_descr(pipe,
			   &pre_anr_descr, SH_CSS_BINARY_MODE_PRE_ANR,
			   in_info, out_info, NULL);
	}
	if (pipe->stream->config.online) {
		pre_anr_descr.online        = true;
		pre_anr_descr.two_ppc       = pipe->stream->config.two_pixels_per_clock;
		pre_anr_descr.stream_format = pipe->stream->config.format;
	}
	pre_anr_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

static void init_anr_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_anr_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	init_offline_descr(pipe,
			   &anr_descr, SH_CSS_BINARY_MODE_ANR,
			   in_info, out_info, NULL);
	anr_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

static void init_post_anr_descr(
	struct sh_css_pipe *pipe,
	struct ia_css_frame_info *in_info,
	struct ia_css_frame_info *out_info,
	struct ia_css_frame_info *vf_info)
{
assert(pipe != NULL);
assert(in_info != NULL);
assert(out_info != NULL);
assert(vf_info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "init_post_anr_descr() enter:\n");

	*in_info = *out_info;
	in_info->format = IA_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth = sh_css_pipe_input_format_bits_per_pixel(pipe);
	if (pipe->isp_pipe_version == 1) {
		init_offline_descr(pipe,
			   &post_anr_descr, SH_CSS_BINARY_MODE_POST_ISP,
			   in_info, out_info, vf_info);
	} else {
		init_offline_descr(pipe,
			   &post_anr_descr, SH_CSS_BINARY_MODE_POST_ANR,
			   in_info, out_info, vf_info);
	}
	post_anr_descr.isp_pipe_version    = pipe->isp_pipe_version;
}

static enum ia_css_err load_primary_binaries(
	struct sh_css_pipe *pipe)
{
	bool online = pipe->stream->config.online;
	bool continuous = pipe->stream->config.continuous;
	bool need_pp = false;
	struct ia_css_frame_info prim_in_info,
				 prim_out_info, vf_info,
				 *vf_pp_in_info;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_capture_settings *mycs = &pipe->pipe.capture;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_primary_binaries() enter:\n");

	if (mycs->primary_binary.info)
		return IA_CSS_SUCCESS;

	err = check_vf_out_info(&pipe->output_info, &pipe->vf_output_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	/* we use the vf output info to get the primary/capture_pp binary
	   configured for vf_veceven. It will select the closest downscaling
	   factor. */
	vf_info = pipe->vf_output_info;
	sh_css_frame_info_set_format(&vf_info, IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		init_capture_pp_descr(pipe, &prim_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
					&mycs->capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		prim_out_info = pipe->output_info;
	}

	/* Primary */
	init_primary_descr(pipe, &prim_in_info, &prim_out_info, &vf_info);
	err = sh_css_binary_find(&prim_descr, &mycs->primary_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &mycs->capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &mycs->primary_binary.vf_frame_info;
	}

	init_vf_pp_descr(pipe, vf_pp_in_info, &pipe->vf_output_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &mycs->vf_pp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* ISP Copy */
	if (!online && !continuous) {
		err = load_copy_binary(pipe,
				       &mycs->copy_binary,
				       &mycs->primary_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	if (need_pp)
		return alloc_capture_pp_frame(pipe, &mycs->capture_pp_binary);
	else
		return IA_CSS_SUCCESS;
}

static enum ia_css_err load_advanced_binaries(
	struct sh_css_pipe *pipe)
{
	struct ia_css_frame_info pre_in_info, gdc_in_info,
				 post_in_info, post_out_info,
				 vf_info, *vf_pp_in_info;
	bool need_pp;
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_advanced_binaries() enter:\n");

	if (pipe->pipe.capture.pre_isp_binary.info)
		return IA_CSS_SUCCESS;

	vf_info = pipe->vf_output_info;
	err = check_vf_out_info(&pipe->output_info,
				&vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	sh_css_frame_info_set_format(&vf_info,
				     IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		init_capture_pp_descr(pipe, &post_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
				&pipe->pipe.capture.capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		post_out_info = pipe->output_info;
	}

	/* Post-gdc */
	init_post_gdc_descr(pipe, &post_in_info, &post_out_info, &vf_info);
	err = sh_css_binary_find(&post_gdc_descr,
				 &pipe->pipe.capture.post_isp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Gdc */
	init_gdc_descr(pipe, &gdc_in_info,
		       &pipe->pipe.capture.post_isp_binary.in_frame_info);
	err = sh_css_binary_find(&gdc_descr,
				 &pipe->pipe.capture.gdc_binary);
	if (err != IA_CSS_SUCCESS)
		return err;
	pipe->pipe.capture.gdc_binary.left_padding =
		pipe->pipe.capture.post_isp_binary.left_padding;

	/* Pre-gdc */
	init_pre_gdc_descr(pipe, &pre_in_info,
			   &pipe->pipe.capture.gdc_binary.in_frame_info);
	err = sh_css_binary_find(&pre_gdc_descr,
				 &pipe->pipe.capture.pre_isp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;
	pipe->pipe.capture.pre_isp_binary.left_padding =
		pipe->pipe.capture.gdc_binary.left_padding;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &pipe->pipe.capture.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &pipe->pipe.capture.post_isp_binary.vf_frame_info;
	}

	init_vf_pp_descr(pipe, vf_pp_in_info, &pipe->vf_output_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &pipe->pipe.capture.vf_pp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Copy */
	err = load_copy_binary(pipe,
			       &pipe->pipe.capture.copy_binary,
			       &pipe->pipe.capture.pre_isp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (need_pp)
		return alloc_capture_pp_frame(pipe,
				&pipe->pipe.capture.capture_pp_binary);
	else
		return IA_CSS_SUCCESS;
}

static enum ia_css_err load_pre_isp_binaries(
	struct sh_css_pipe *pipe)
{
	struct ia_css_frame_info pre_isp_in_info;
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_pre_isp_binaries() enter:\n");

	if (pipe->pipe.capture.pre_isp_binary.info)
		return IA_CSS_SUCCESS;

	err = check_frame_info(&pipe->output_info);
	if (err != IA_CSS_SUCCESS)
		return err;

	init_pre_anr_descr(pipe, &pre_isp_in_info,
			   &pipe->output_info);

	err = sh_css_binary_find(&pre_anr_descr,
				 &pipe->pipe.capture.pre_isp_binary);

	return err;
}

static enum ia_css_err load_low_light_binaries(
	struct sh_css_pipe *pipe)
{
	struct ia_css_frame_info pre_in_info, anr_in_info,
				 post_in_info, post_out_info,
				 vf_info, *vf_pp_in_info;
	bool need_pp;
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_low_light_binaries() enter:\n");

	if (pipe->isp_pipe_version == 1) {
		if (pipe->pipe.capture.pre_isp_binary.info)
			return IA_CSS_SUCCESS;
	} else {
		if (pipe->pipe.capture.pre_anr_binary.info)
			return IA_CSS_SUCCESS;
	}

	vf_info = pipe->vf_output_info;
	err = check_vf_out_info(&pipe->output_info,
				&vf_info);
	if (err != IA_CSS_SUCCESS)
		return err;
	need_pp = need_capture_pp(pipe);

	sh_css_frame_info_set_format(&vf_info,
				     IA_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	if (need_pp) {
		init_capture_pp_descr(pipe, &post_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
				&pipe->pipe.capture.capture_pp_binary);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		post_out_info = pipe->output_info;
	}

	/* Post-anr */
	init_post_anr_descr(pipe, &post_in_info, &post_out_info, &vf_info);
	if (pipe->isp_pipe_version == 1) {
		err = sh_css_binary_find(&post_anr_descr,
				 &pipe->pipe.capture.post_isp_binary);
	} else {
		err = sh_css_binary_find(&post_anr_descr,
				 &pipe->pipe.capture.post_anr_binary);
	}
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Anr */
	if (pipe->isp_pipe_version == 1) {
		init_anr_descr(pipe, &anr_in_info,
		       &pipe->pipe.capture.post_isp_binary.in_frame_info);
	} else {
		init_anr_descr(pipe, &anr_in_info,
		       &pipe->pipe.capture.post_anr_binary.in_frame_info);
	}
	err = sh_css_binary_find(&anr_descr,
				 &pipe->pipe.capture.anr_binary);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (pipe->isp_pipe_version == 1) {
		pipe->pipe.capture.anr_binary.left_padding =
			pipe->pipe.capture.post_isp_binary.left_padding;
	} else {
		pipe->pipe.capture.anr_binary.left_padding =
			pipe->pipe.capture.post_anr_binary.left_padding;
	}

	/* Pre-anr */
	init_pre_anr_descr(pipe, &pre_in_info,
			   &pipe->pipe.capture.anr_binary.in_frame_info);
	if (pipe->isp_pipe_version == 1) {
		err = sh_css_binary_find(&pre_anr_descr,
				 &pipe->pipe.capture.pre_isp_binary);
	} else {
		err = sh_css_binary_find(&pre_anr_descr,
				 &pipe->pipe.capture.pre_anr_binary);
	}
	if (err != IA_CSS_SUCCESS)
		return err;
	if (pipe->isp_pipe_version == 1) {
		pipe->pipe.capture.pre_isp_binary.left_padding =
			pipe->pipe.capture.anr_binary.left_padding;
	} else {
		pipe->pipe.capture.pre_anr_binary.left_padding =
			pipe->pipe.capture.anr_binary.left_padding;
	}

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &pipe->pipe.capture.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &pipe->pipe.capture.post_isp_binary.vf_frame_info;
	}

	init_vf_pp_descr(pipe, vf_pp_in_info, &pipe->vf_output_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &pipe->pipe.capture.vf_pp_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Copy */
	if (pipe->isp_pipe_version == 1) {
		err = load_copy_binary(pipe,
			       &pipe->pipe.capture.copy_binary,
			       &pipe->pipe.capture.pre_isp_binary);
	} else {
		err = load_copy_binary(pipe,
			       &pipe->pipe.capture.copy_binary,
			       &pipe->pipe.capture.pre_anr_binary);
	}
	if (err != IA_CSS_SUCCESS)
		return err;

	if (need_pp)
		return alloc_capture_pp_frame(pipe,
				&pipe->pipe.capture.capture_pp_binary);
	else
		return IA_CSS_SUCCESS;
}

static bool copy_on_sp(
	struct sh_css_pipe *pipe)
{

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "copy_on_sp() enter:\n");

	if (pipe->mode != IA_CSS_PIPE_ID_CAPTURE)
		return false;
	if (pipe->capture_mode != IA_CSS_CAPTURE_MODE_RAW)
		return false;
	return pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8;
}

static enum ia_css_err load_capture_binaries(
	struct sh_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	bool must_be_raw;

assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "load_capture_binaries() enter:\n");
	/* in primary, advanced,low light or bayer,
						the input format must be raw */
	must_be_raw =
		pipe->capture_mode == IA_CSS_CAPTURE_MODE_ADVANCED ||
		pipe->capture_mode == IA_CSS_CAPTURE_MODE_BAYER ||
		pipe->capture_mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT;
	err = check_input(pipe, must_be_raw);
	if (err != IA_CSS_SUCCESS)
		return err;
	if (copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		sh_css_frame_info_init(
			&pipe->output_info,
			JPEG_BYTES, 1, IA_CSS_FRAME_FORMAT_BINARY_8);
		return IA_CSS_SUCCESS;
	}

	switch (pipe->capture_mode) {
	case IA_CSS_CAPTURE_MODE_RAW:
		err = load_copy_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_BAYER:
		err = load_pre_isp_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_PRIMARY:
		err = load_primary_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_ADVANCED:
		err = load_advanced_binaries(pipe);
		break;
	case IA_CSS_CAPTURE_MODE_LOW_LIGHT:
		err = load_low_light_binaries(pipe);
		break;
	}
	if (err != IA_CSS_SUCCESS)
		return err;

	err = allocate_mipi_frames(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	return err;
}

static enum ia_css_err
sh_css_pipe_load_binaries(struct sh_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	assert(pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_load_binaries() enter:\n");

	switch (pipe->mode) {
	case IA_CSS_PIPE_ID_PREVIEW:
		err = load_preview_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_VIDEO:
		err = load_video_binaries(pipe);
		break;
	case IA_CSS_PIPE_ID_CAPTURE:
		err = load_capture_binaries(pipe);
		break;
	default:
		err = IA_CSS_ERR_INTERNAL_ERROR;
		break;
	}
	if (pipe->new_pipe->config.acc_extension) {
		ia_css_pipe_load_extension(pipe->new_pipe,
				pipe->new_pipe->config.acc_extension);
	}
	return err;
}

static enum ia_css_err
construct_copy_pipe(struct sh_css_pipe *pipe,
		    unsigned max_input_width,
		    struct ia_css_frame *out_frame)
{
	struct sh_css_pipeline *me = &pipe->pipeline;
	enum ia_css_err err = IA_CSS_SUCCESS;
	
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "construct_copy_pipe() enter:\n");
	sh_css_pipeline_clean(me);

	/* Construct out_frame info */
	sh_css_pipe_get_output_frame_info(pipe, &out_frame->info);

	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	out_frame->dynamic_data_index = sh_css_frame_out;

	me->num_stages = 1;
	me->pipe_id = IA_CSS_PIPE_ID_COPY;
	pipe->mode  = IA_CSS_PIPE_ID_COPY;

	err = sh_css_pipeline_add_sp_stage(me, SH_CSS_SP_RAW_COPY, max_input_width, out_frame);
	return err;
}

static enum ia_css_err
construct_capture_pipe(struct sh_css_pipe *pipe)
{
	struct sh_css_pipeline *me = &pipe->pipeline;
	enum ia_css_err err = IA_CSS_SUCCESS;
	enum ia_css_capture_mode mode = pipe->capture_mode;
	struct sh_css_pipeline_stage *out_stage = NULL,
				     *vf_pp_stage = NULL,
				     *copy_stage = NULL,
				     *in_stage = NULL,
				     *post_stage = NULL;
	struct ia_css_frame *cc_frame = NULL;
	struct sh_css_binary *copy_binary,
			     *primary_binary,
			     *vf_pp_binary,
			     *pre_isp_binary,
			     *gdc_binary,
			     *post_isp_binary,
			     *pre_anr_binary,
			     *anr_binary,
			     *post_anr_binary,
			     *capture_pp_binary,
			     *sc_binary = NULL;
	bool need_pp = false;
	bool raw = mode == IA_CSS_CAPTURE_MODE_RAW;

	/**
	 * rvanimme: in_frame support is broken and forced to NULL
	 * TODO: add a way to tell the pipeline construction that an in_frame
	 * is used.
	 */
	struct ia_css_frame *in_frame = NULL;
	struct ia_css_frame *out_frame = &me->out_frame;
	struct ia_css_frame *vf_frame = &me->vf_frame;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "construct_capture_pipe() enter:\n");
	sh_css_pipeline_clean(me);

	
	err = sh_css_pipe_load_binaries(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Construct out_frame info */
	sh_css_pipe_get_output_frame_info(pipe, &out_frame->info);

	out_frame->contiguous = false;
	out_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
	out_frame->dynamic_data_index = sh_css_frame_out;
	err = init_frame_planes(out_frame);
	if (err != IA_CSS_SUCCESS)
		return err;

	/* Construct vf_frame info (only in case we have VF) */
	if (mode == IA_CSS_CAPTURE_MODE_RAW ||
			mode == IA_CSS_CAPTURE_MODE_BAYER) {
		/* These modes don't support viewfinder output */
		vf_frame = NULL;
	} else {
		sh_css_pipe_get_viewfinder_frame_info(pipe, &vf_frame->info);
		vf_frame->contiguous = false;
		vf_frame->flash_state = IA_CSS_FRAME_FLASH_STATE_NONE;
		vf_frame->dynamic_data_index = sh_css_frame_out_vf;
		err = init_frame_planes(vf_frame);
		if (err != IA_CSS_SUCCESS)
			return err;
	}

	copy_stage = NULL;
	in_stage = NULL;

	copy_binary       = &pipe->pipe.capture.copy_binary;
	primary_binary    = &pipe->pipe.capture.primary_binary;
	vf_pp_binary      = &pipe->pipe.capture.vf_pp_binary;
	pre_isp_binary    = &pipe->pipe.capture.pre_isp_binary;
	gdc_binary        = &pipe->pipe.capture.gdc_binary;
	post_isp_binary   = &pipe->pipe.capture.post_isp_binary;
	pre_anr_binary    = &pipe->pipe.capture.pre_anr_binary;
	anr_binary        = &pipe->pipe.capture.anr_binary;
	post_anr_binary   = &pipe->pipe.capture.post_anr_binary;
	capture_pp_binary = &pipe->pipe.capture.capture_pp_binary;
	need_pp = need_capture_pp(pipe) || pipe->output_stage;

	if (pipe->pipe.capture.copy_binary.info) {
		err = sh_css_pipeline_add_stage(me, copy_binary, NULL,
				copy_binary->info->mode, NULL, NULL,
				raw ? out_frame : in_frame,
				NULL, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		in_stage = post_stage;
	} else if (pipe->stream->config.continuous) {
		in_frame = pipe->stream->last_pipe->old_pipe->continuous_frames[0];//pipe->pipe.capture.continuous_frames[0];
	}

	if (mode == IA_CSS_CAPTURE_MODE_PRIMARY) {
		err = sh_css_pipeline_add_stage(me, primary_binary,
				NULL, primary_binary->info->mode,
				cc_frame, in_frame,
				need_pp ? NULL : out_frame,
				NULL, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		/* If we use copy iso primary,
		   the input must be yuv iso raw */
		post_stage->args.copy_vf =
			primary_binary->info->mode ==
			SH_CSS_BINARY_MODE_COPY;
		post_stage->args.copy_output = post_stage->args.copy_vf;
		sc_binary = primary_binary;
	} else if (mode == IA_CSS_CAPTURE_MODE_ADVANCED) {
		err = sh_css_pipeline_add_stage(me, pre_isp_binary,
				NULL, pre_isp_binary->info->mode,
				cc_frame, in_frame, NULL, NULL, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
		err = sh_css_pipeline_add_stage(me, gdc_binary,
				NULL, gdc_binary->info->mode,
				NULL, NULL, NULL, NULL, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
		err = sh_css_pipeline_add_stage(me, post_isp_binary,
				NULL, post_isp_binary->info->mode,
				NULL, NULL,
				need_pp ? NULL : out_frame,
				NULL, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		sc_binary = pre_isp_binary;
	} else if (mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT) {
		if (pipe->isp_pipe_version == 1) {
			err = sh_css_pipeline_add_stage(me, pre_isp_binary,
				NULL, pre_isp_binary->info->mode,
				cc_frame, in_frame, NULL, NULL, NULL);
		} else {
			err = sh_css_pipeline_add_stage(me, pre_anr_binary,
				NULL, pre_anr_binary->info->mode,
				cc_frame, in_frame, NULL, NULL, NULL);
		}
		if (err != IA_CSS_SUCCESS)
			return err;
		err = sh_css_pipeline_add_stage(me, anr_binary,
				NULL, anr_binary->info->mode,
				NULL, NULL, NULL, NULL, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
		if (pipe->isp_pipe_version == 1) {
			err = sh_css_pipeline_add_stage(me, post_isp_binary,
				NULL, post_isp_binary->info->mode,
				NULL, NULL,
				need_pp ? NULL : out_frame,
				NULL, &post_stage);
		} else {
			err = sh_css_pipeline_add_stage(me, post_anr_binary,
				NULL, post_anr_binary->info->mode,
				NULL, NULL,
				need_pp ? NULL : out_frame,
				NULL, &post_stage);
		}
		if (err != IA_CSS_SUCCESS)
			return err;
		if (pipe->isp_pipe_version == 1) {
			sc_binary = pre_isp_binary;
		} else {
			sc_binary = pre_anr_binary;
		}
	} else if (mode == IA_CSS_CAPTURE_MODE_BAYER) {
		err = sh_css_pipeline_add_stage(me, pre_isp_binary,
				NULL, pre_isp_binary->info->mode,
				cc_frame, in_frame, out_frame,
				NULL, NULL);
		if (err != IA_CSS_SUCCESS)
			return err;
		sc_binary = pre_isp_binary;
	}
	if (!in_stage)
		in_stage = post_stage;

	if (need_pp) {
		err = add_capture_pp_stage(pipe, me, out_frame,
					   capture_pp_binary,
					   post_stage, &post_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	if (mode != IA_CSS_CAPTURE_MODE_RAW &&
		mode != IA_CSS_CAPTURE_MODE_BAYER) {
		err = add_vf_pp_stage(pipe, vf_frame, vf_pp_binary,
				      post_stage, &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	number_stages(pipe);

	/**
	 * Maybe we can return earlier but this was the original position
	 * in the original version of capture_start()
	 */
	if (pipe->capture_mode == IA_CSS_CAPTURE_MODE_RAW ||
	    pipe->capture_mode == IA_CSS_CAPTURE_MODE_BAYER) {
		if (copy_on_sp(pipe))
			return IA_CSS_SUCCESS;
	}

	if (mode == IA_CSS_CAPTURE_MODE_RAW) {
		err = sh_css_pipeline_get_stage(me, copy_binary->info->mode,
						&out_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
		copy_stage = out_stage;
	} else if (mode == IA_CSS_CAPTURE_MODE_BAYER) {
		err = sh_css_pipeline_get_stage(me,
				pre_isp_binary->info->mode,
				&out_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	} else {
		if (copy_binary->info) {
			err = sh_css_pipeline_get_stage(me,
							copy_binary->info->mode,
							&copy_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		if (capture_pp_binary->info) {
			err = sh_css_pipeline_get_stage(me,
					capture_pp_binary->info->mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		} else if (mode ==
			   IA_CSS_CAPTURE_MODE_PRIMARY) {
			err = sh_css_pipeline_get_stage(me,
					primary_binary->info->mode, &out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		} else if (mode ==
			   IA_CSS_CAPTURE_MODE_LOW_LIGHT) {
			if (pipe->isp_pipe_version == 1) {
				err = sh_css_pipeline_get_stage(me,
					post_isp_binary->info->mode,
					&out_stage);
			} else {
				err = sh_css_pipeline_get_stage(me,
					post_anr_binary->info->mode,
					&out_stage);
			}
			if (err != IA_CSS_SUCCESS)
				return err;
		} else {
			err = sh_css_pipeline_get_stage(me,
					post_isp_binary->info->mode,
					&out_stage);
			if (err != IA_CSS_SUCCESS)
				return err;
		}
		err = sh_css_pipeline_get_output_stage(me,
						       vf_pp_binary->info->mode,
						       &vf_pp_stage);
		if (err != IA_CSS_SUCCESS)
			return err;
	}
	if (mode != IA_CSS_CAPTURE_MODE_RAW &&
	    mode != IA_CSS_CAPTURE_MODE_BAYER)
		vf_pp_stage->args.out_frame = vf_frame;

	/* rvanimme: why is this? */
	/* TODO: investigate if this can be removed */
	if (!pipe->output_stage)
		out_stage->args.out_frame = out_frame;

	if (copy_stage && in_frame)
		copy_stage->args.out_frame = in_frame;

	sh_css_set_irq_buffer(in_stage,    sh_css_frame_in,  in_frame);
	sh_css_set_irq_buffer(out_stage,   sh_css_frame_out, out_frame);
	sh_css_set_irq_buffer(vf_pp_stage, sh_css_frame_in,  vf_frame);

	return IA_CSS_SUCCESS;

}

static enum ia_css_err capture_start(
	struct sh_css_pipe *pipe)
{
	struct sh_css_pipeline *me = &pipe->pipeline;

	enum ia_css_err err = IA_CSS_SUCCESS;
	enum sh_css_pipe_config_override copy_ovrd;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "capture_start() enter:\n");

	err = construct_capture_pipe(pipe);
	if (err != IA_CSS_SUCCESS)
		return err;

	err = sh_css_config_input_network(pipe, &pipe->pipe.capture.copy_binary);
	if (err != IA_CSS_SUCCESS)
		return err;

	if (pipe->capture_mode == IA_CSS_CAPTURE_MODE_RAW ||
	    pipe->capture_mode == IA_CSS_CAPTURE_MODE_BAYER) {
		if (copy_on_sp(pipe)) {
			return start_copy_on_sp(pipe,
				&me->out_frame);
		}
	}

	/* multi stream video needs mipi buffers */
	send_mipi_frames(pipe);

	{
		unsigned int thread_id;
		
		sh_css_query_sp_thread_id(pipe->pipe_num, &thread_id);
		copy_ovrd = 1 << thread_id;
		
	}
	start_pipe(pipe, copy_ovrd, pipe->stream->config.mode);

	return IA_CSS_SUCCESS;

}

static enum ia_css_err
sh_css_pipe_get_output_frame_info(struct sh_css_pipe *pipe,
				  struct ia_css_frame_info *info)
{
	enum ia_css_err err;

	assert(pipe != NULL);
	assert(info != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_get_output_frame_info() enter:\n");
	err = sh_css_pipe_load_binaries(pipe);
	if (err == IA_CSS_SUCCESS)
		*info = pipe->output_info;
	if (copy_on_sp(pipe) &&
	    pipe->stream->config.format == IA_CSS_STREAM_FORMAT_BINARY_8) {
		sh_css_frame_info_init(info, JPEG_BYTES, 1,
				IA_CSS_FRAME_FORMAT_BINARY_8);
	} else if (info->format == IA_CSS_FRAME_FORMAT_RAW) {
		info->raw_bit_depth =
			sh_css_pipe_input_format_bits_per_pixel(pipe);
	}
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_get_output_frame_info() leave:\n");
	return err;
}

void
ia_css_stream_send_input_frame(const struct ia_css_stream *stream,
			       const unsigned short *data,
			       unsigned int width,
			       unsigned int height)
{
	sh_css_hrt_send_input_frame(
			data, width, height,
			stream->config.channel_id,
			stream->config.format,
			stream->config.two_pixels_per_clock);
}

void
ia_css_stream_start_input_frame(const struct ia_css_stream *stream)
{
	sh_css_hrt_streaming_to_mipi_start_frame(
			stream->config.channel_id,
			stream->config.format,
			stream->config.two_pixels_per_clock);
}

void
ia_css_stream_send_input_line(const struct ia_css_stream *stream,
			      const unsigned short *data,
			      unsigned int width,
			      const unsigned short *data2,
			      unsigned int width2)
{
	sh_css_hrt_streaming_to_mipi_send_line(stream->config.channel_id,
					       data, width, data2, width2);
}


void
ia_css_stream_end_input_frame(const struct ia_css_stream *stream)
{
	sh_css_hrt_streaming_to_mipi_end_frame(stream->config.channel_id);
}

static enum ia_css_err allocate_frame_data(
	struct ia_css_frame *frame)
{
	frame->data = mmgr_alloc_attr(frame->data_bytes,
		frame->contiguous ?
			MMGR_ATTRIBUTE_CONTIGUOUS : MMGR_ATTRIBUTE_DEFAULT);

	if (frame->data == mmgr_NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	return IA_CSS_SUCCESS;
}

static void init_plane(
	struct ia_css_frame_plane *plane,
	unsigned int width,
	unsigned int stride,
	unsigned int height,
	unsigned int offset)
{
	plane->height = height;
	plane->width = width;
	plane->stride = stride;
	plane->offset = offset;
}

static void init_single_plane(
	struct ia_css_frame *frame,
	struct ia_css_frame_plane *plane,
	unsigned int height,
	unsigned int subpixels_per_line,
	unsigned int bytes_per_pixel)
{
	unsigned int stride;

	stride = subpixels_per_line * bytes_per_pixel;
	frame->data_bytes = stride * height;
	init_plane(plane, subpixels_per_line, stride, height, 0);
	return;
}


static void init_mipi_plane(
	struct ia_css_frame *frame,
	struct ia_css_frame_plane *plane,
	unsigned int height,
	unsigned int subpixels_per_line,
	unsigned int bytes_per_pixel)
{
	unsigned int stride;

	stride = subpixels_per_line * bytes_per_pixel;
	frame->data_bytes = 8388608;
	frame->valid = false;
	frame->contiguous = true;
	init_plane(plane, subpixels_per_line, stride, height, 0);
	return;
	}


static void init_nv_planes(
	struct ia_css_frame *frame,
	unsigned int horizontal_decimation,
	unsigned int vertical_decimation)
{
	unsigned int y_width = frame->info.padded_width,
		     y_height = frame->info.res.height,
		     uv_width = 2 * (y_width / horizontal_decimation),
		     uv_height = y_height / vertical_decimation,
		     y_bytes, uv_bytes;

	y_bytes   = y_width * y_height;
	uv_bytes  = uv_width * uv_height;

	frame->data_bytes = y_bytes + uv_bytes;
	init_plane(&frame->planes.nv.y, y_width, y_width, y_height, 0);
	init_plane(&frame->planes.nv.uv, uv_width,
			uv_width, uv_height, y_bytes);
	return;
}

static void init_yuv_planes(
	struct ia_css_frame *frame,
	unsigned int horizontal_decimation,
	unsigned int vertical_decimation,
	bool swap_uv,
	unsigned int bytes_per_element)
{
	unsigned int y_width = frame->info.padded_width,
		     y_height = frame->info.res.height,
		     uv_width = y_width / horizontal_decimation,
		     uv_height = y_height / vertical_decimation,
		     y_stride, y_bytes, uv_bytes, uv_stride;

	y_stride  = y_width * bytes_per_element;
	uv_stride = uv_width * bytes_per_element;
	y_bytes   = y_stride * y_height;
	uv_bytes  = uv_stride * uv_height;

	frame->data_bytes = y_bytes + 2 * uv_bytes;
	init_plane(&frame->planes.yuv.y, y_width, y_stride, y_height, 0);
		if (swap_uv) {
			init_plane(&frame->planes.yuv.v, uv_width, uv_stride,
				   uv_height, y_bytes);
			init_plane(&frame->planes.yuv.u, uv_width, uv_stride,
				   uv_height, y_bytes + uv_bytes);
		} else {
			init_plane(&frame->planes.yuv.u, uv_width, uv_stride,
				   uv_height, y_bytes);
			init_plane(&frame->planes.yuv.v, uv_width, uv_stride,
				   uv_height, y_bytes + uv_bytes);
		}
	return;
	}

static void init_rgb_planes(
	struct ia_css_frame *frame,
	unsigned int bytes_per_element)
{
	unsigned int width = frame->info.res.width,
		     height = frame->info.res.height, stride, bytes;

	stride = width * bytes_per_element;
	bytes  = stride * height;
	frame->data_bytes = 3 * bytes;
	init_plane(&frame->planes.planar_rgb.r,
			width, stride, height, 0);
	init_plane(&frame->planes.planar_rgb.g,
			width, stride, height, 1 * bytes);
	init_plane(&frame->planes.planar_rgb.b,
			width, stride, height, 2 * bytes);
	return;
	}

static void init_qplane6_planes(
	struct ia_css_frame *frame)
{
	unsigned int width = frame->info.padded_width / 2,
		     height = frame->info.res.height / 2,
		     bytes, stride;

	stride = width * 2;
	bytes  = stride * height;

	frame->data_bytes = 6 * bytes;
	init_plane(&frame->planes.plane6.r,
			width, stride, height, 0 * bytes);
	init_plane(&frame->planes.plane6.r_at_b,
			width, stride, height, 1 * bytes);
	init_plane(&frame->planes.plane6.gr,
			width, stride, height, 2 * bytes);
	init_plane(&frame->planes.plane6.gb,
			width, stride, height, 3 * bytes);
	init_plane(&frame->planes.plane6.b,
			width, stride, height, 4 * bytes);
	init_plane(&frame->planes.plane6.b_at_r,
			width, stride, height, 5 * bytes);
	return;
}

static enum ia_css_err init_frame_planes(
	struct ia_css_frame *frame)
{
assert(frame != NULL);

	switch (frame->info.format) {
	case IA_CSS_FRAME_FORMAT_MIPI:
		init_mipi_plane(frame, &frame->planes.raw,
				frame->info.res.height,
				frame->info.padded_width,
				frame->info.raw_bit_depth <= 8 ? 1 : 2);
		break;
	case IA_CSS_FRAME_FORMAT_RAW:
		init_single_plane(frame, &frame->planes.raw,
				frame->info.res.height,
				frame->info.padded_width,
				frame->info.raw_bit_depth <= 8 ? 1 : 2);
		break;
	case IA_CSS_FRAME_FORMAT_RGB565:
		init_single_plane(frame, &frame->planes.rgb,
				    frame->info.res.height,
				    frame->info.padded_width, 2);
		break;
	case IA_CSS_FRAME_FORMAT_RGBA888:
		init_single_plane(frame, &frame->planes.rgb,
				    frame->info.res.height,
				    frame->info.padded_width * 4, 1);
		break;
	case IA_CSS_FRAME_FORMAT_PLANAR_RGB888:
		init_rgb_planes(frame, 1);
		break;
		/* yuyv and uyvu have the same frame layout, only the data
		 * positioning differs.
		 */
	case IA_CSS_FRAME_FORMAT_YUYV:
	case IA_CSS_FRAME_FORMAT_UYVY:
		init_single_plane(frame, &frame->planes.yuyv,
				    frame->info.res.height,
				    frame->info.padded_width * 2, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YUV_LINE:
		/* Needs 3 extra lines to allow vf_pp prefetching */
		init_single_plane(frame, &frame->planes.yuyv,
				    frame->info.res.height * 3/2 + 3,
				    frame->info.padded_width, 1);
		break;
	case IA_CSS_FRAME_FORMAT_NV11:
		init_nv_planes(frame, 4, 1);
		break;
		/* nv12 and nv21 have the same frame layout, only the data
		 * positioning differs.
		 */
	case IA_CSS_FRAME_FORMAT_NV12:
	case IA_CSS_FRAME_FORMAT_NV21:
		init_nv_planes(frame, 2, 2);
		break;
		/* nv16 and nv61 have the same frame layout, only the data
		 * positioning differs.
		 */
	case IA_CSS_FRAME_FORMAT_NV16:
	case IA_CSS_FRAME_FORMAT_NV61:
		init_nv_planes(frame, 2, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YUV420:
		init_yuv_planes(frame, 2, 2, false, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YUV422:
		init_yuv_planes(frame, 2, 1, false, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YUV444:
		init_yuv_planes(frame, 1, 1, false, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YUV420_16:
		init_yuv_planes(frame, 2, 2, false, 2);
		break;
	case IA_CSS_FRAME_FORMAT_YUV422_16:
		init_yuv_planes(frame, 2, 1, false, 2);
		break;
	case IA_CSS_FRAME_FORMAT_YV12:
		init_yuv_planes(frame, 2, 2, true, 1);
		break;
	case IA_CSS_FRAME_FORMAT_YV16:
		init_yuv_planes(frame, 2, 1, true, 1);
		break;
	case IA_CSS_FRAME_FORMAT_QPLANE6:
		init_qplane6_planes(frame);
		break;
	case IA_CSS_FRAME_FORMAT_BINARY_8:
		init_single_plane(frame, &frame->planes.binary.data,
				    frame->info.res.height,
				    frame->info.padded_width, 1);
		frame->planes.binary.size = 0;
		break;
	default:
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	return IA_CSS_SUCCESS;
}


static enum ia_css_err allocate_frame_and_data(
	struct ia_css_frame **frame,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format,
	unsigned int padded_width,
	unsigned int raw_bit_depth,
	bool contiguous)
{
	enum ia_css_err err;
	struct ia_css_frame *me = sh_css_malloc(sizeof(*me));

	if (me == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	me->info.res.width = width;
	me->info.res.height = height;
	me->info.format = format;
	me->info.padded_width = padded_width;
	me->info.raw_bit_depth = raw_bit_depth;
	me->contiguous = contiguous;
	me->valid = true;
	me->dynamic_data_index = SH_CSS_INVALID_FRAME_ID;

	err = init_frame_planes(me);

	if (err == IA_CSS_SUCCESS)
		err = allocate_frame_data(me);

	if (err != IA_CSS_SUCCESS) {
		sh_css_free(me);
		return err;
	}

		*frame = me;

	return err;
}

enum ia_css_err ia_css_frame_allocate(
	struct ia_css_frame **frame,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format,
	unsigned int padded_width,
	unsigned int raw_bit_depth)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	if (frame == NULL || width == 0||height == 0) return IA_CSS_ERR_INVALID_ARGUMENTS;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate() enter: width=%d, height=%d, format=%d\n",
		width, height, format);


	err = allocate_frame_and_data(frame, width, height, format,
			      padded_width, raw_bit_depth, false);

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate() leave: frame=%p\n",
		frame ? *frame : (void *)-1);

	return err;
}

enum ia_css_err ia_css_frame_allocate_from_info(
	struct ia_css_frame **frame,
	const struct ia_css_frame_info *info)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	if (frame == NULL || info == NULL) return IA_CSS_ERR_INVALID_ARGUMENTS;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_frame_allocate_from_info() enter:\n");
	err = ia_css_frame_allocate(frame,
				     info->res.width,
				     info->res.height,
				     info->format,
				     info->padded_width,
				     info->raw_bit_depth);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_frame_allocate_from_info() leave:\n");
	return err;
}

enum ia_css_err
ia_css_frame_map(struct ia_css_frame **frame,
                 const struct ia_css_frame_info *info,
                 const void *data,
                 uint16_t attribute,
                 void *context)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_frame *me = sh_css_malloc(sizeof(*me));

	if (me == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	me->info.res.width = info->res.width;
	me->info.res.height = info->res.height;
	me->info.format = info->format;
	me->info.padded_width = info->padded_width;
	me->info.raw_bit_depth = info->raw_bit_depth;
	me->contiguous = false; /* doublecheck */
	me->valid = true;
	me->dynamic_data_index = SH_CSS_INVALID_FRAME_ID;

	err = init_frame_planes(me);

	if (err == IA_CSS_SUCCESS) {
		/* use mmgr_mmap to map */
		me->data = (ia_css_ptr)mmgr_mmap(
					     data,
					     me->data_bytes,
					     attribute,
					     context);
		if (me->data == mmgr_NULL)
			err = IA_CSS_ERR_INVALID_ARGUMENTS;
	};

	if (err != IA_CSS_SUCCESS) {
		sh_css_free(me);
		return err;
	}

	*frame = me;

	return err;
}

enum ia_css_err
ia_css_mipi_frame_allocate(struct	ia_css_frame **frame,
				const unsigned int	size_bytes,
				const bool			contiguous)
{
	/* AM: Body coppied from allocate_frame_and_data().*/
	enum ia_css_err err;
	struct ia_css_frame *me = sh_css_malloc(sizeof(*me));

	sh_css_dtrace(SH_DBG_TRACE, "ia_css_mipi_frame_allocate()\n");
	
	if (me == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;

	me->info.res.width = 0;
	me->info.res.height = 0;
	// To indicate it is not (yet) valid format.
	me->info.format = IA_CSS_FRAME_FORMAT_NUM;
	me->info.padded_width = 0;
	me->info.raw_bit_depth = 0;
	me->data_bytes = size_bytes;
	me->contiguous = contiguous;
	// To indicate it is not valid frame.
	me->dynamic_data_index = SH_CSS_INVALID_FRAME_ID;

	err = allocate_frame_data(me);

	if (err != IA_CSS_SUCCESS) {
		sh_css_free(me);
		return err;
	}

		*frame = me;

	return err;
}

enum ia_css_err
ia_css_mipi_frame_specify(const unsigned int size_mem_words,
				const bool contiguous)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	my_css.size_mem_words 	= size_mem_words;
	my_css.contiguous		= contiguous;		


	return err;
}

/* Assumptions:
 *	- A line is multiple of 4 bytes = 1 word.
 *	- Each frame has SOF and EOF (each 1 word).
 *	- Each line has format header and optionally SOL and EOL (each 1 word).
 *	- Odd and even lines of YUV420 format are different in bites per pixel size.
 *	- Custom size of embedded data.
 *  -- Interleaved frames are not taken into account.
 *  -- Lines are multiples of 8B, and not necessary of (custom 3B, or 7B
 *  etc.). 
 * Result is given in DDR mem words, 32B or 256 bits
 */
enum ia_css_err
ia_css_mipi_frame_calculate_size(const unsigned int width,
				const unsigned int height,
				const enum ia_css_stream_format format,
				const bool hasSOLandEOL,
				const unsigned int embedded_data_size_words,
				unsigned int *size_mem_words)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	unsigned int bits_per_pixel = 0; 
	unsigned int even_line_bytes = 0;
	unsigned int odd_line_bytes = 0;
	unsigned int words_per_odd_line = 0;
	unsigned int words_for_first_line = 0;
	unsigned int words_per_even_line = 0;
	unsigned int mem_words_per_even_line = 0;
	unsigned int mem_words_per_odd_line = 0;
	unsigned int mem_words_for_first_line = 0;
	unsigned int mem_words_for_EOF = 0;
	unsigned int mem_words = 0;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_mipi_frame_calculate_size() "
		"enter: width=%d, height=%d, format=%d, hasSOLandEOL=%d, embedded_data_size_words=%d\n",
		width, height, format, hasSOLandEOL, embedded_data_size_words);

	switch (format) {
		case IA_CSS_STREAM_FORMAT_RAW_6:			/* 4p, 3B, 24bits */
			bits_per_pixel = 6; 	break;
		case IA_CSS_STREAM_FORMAT_RAW_7:			/* 8p, 7B, 56bits */
			bits_per_pixel = 7;		break;
		case IA_CSS_STREAM_FORMAT_RAW_8:			/* 1p, 1B, 8bits */
		case IA_CSS_STREAM_FORMAT_BINARY_8:      /*  8bits, TODO: check. */
		case IA_CSS_STREAM_FORMAT_YUV420_8:		/* odd 2p, 2B, 16bits, even 2p, 4B, 32bits */
			bits_per_pixel = 8;		break;
		case IA_CSS_STREAM_FORMAT_YUV420_10:		/* odd 4p, 5B, 40bits, even 4p, 10B, 80bits */
		case IA_CSS_STREAM_FORMAT_RAW_10:		/* 4p, 5B, 40bits */
			bits_per_pixel = 10;	break;
		case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:	/* 2p, 3B, 24bits */	
		case IA_CSS_STREAM_FORMAT_RAW_12:			/* 2p, 3B, 24bits */			
			bits_per_pixel = 12;	break;
		case IA_CSS_STREAM_FORMAT_RAW_14:		/* 4p, 7B, 56bits */
			bits_per_pixel = 14;	break;
		case IA_CSS_STREAM_FORMAT_RGB_444:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_RGB_555:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_RGB_565:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_YUV422_8:		/* 2p, 4B, 32bits */
			bits_per_pixel = 16;	break;
		case IA_CSS_STREAM_FORMAT_RGB_666:		/* 4p, 9B, 72bits */
			bits_per_pixel = 18;	break;
		case IA_CSS_STREAM_FORMAT_YUV422_10:		/* 2p, 5B, 40bits */
			bits_per_pixel = 20;	break;
		case IA_CSS_STREAM_FORMAT_RGB_888:		/* 1p, 3B, 24bits */
			bits_per_pixel = 24;	break;
		
		case IA_CSS_STREAM_FORMAT_RAW_16:        /* TODO: not specified in MIPI SPEC, check */
		default:
			return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	
	odd_line_bytes = (width * bits_per_pixel + 7) >> 3; /* ceil ( bits per line / 8 ) */

	/* Even lines for YUV420 formats are double in bits_per_pixel. */
	if (format == IA_CSS_STREAM_FORMAT_YUV420_8
		|| format == IA_CSS_STREAM_FORMAT_YUV420_10) {
		even_line_bytes = (width * 2 * bits_per_pixel + 7) >> 3; /* ceil ( bits per line / 8 ) */
	} else {
		even_line_bytes = odd_line_bytes; 
	}

   /*  a frame represented in memory:  ()- optional; data - payload words.
	*  addr		0		1		2		3		4		5		6		7:
	*  first	SOF		(SOL)	PACK_H	data	data	data	data	data
	*          	data	data	data	data	data	data	data	data
	*           ...              
	*			data 	data	0		0		0		0		0		0
	*  second   (EOL)	(SOL)	PACK_H	data	data	data	data	data			
	*          	data	data	data	data	data	data	data	data
	*           ...              
	*			data 	data	0		0		0		0		0		0
	*  ...
	*  last		(EOL)	EOF		0		0		0		0		0		0
	*
	*  Embedded lines are regular lines stored before the first and after
	*  payload lines.
	*/


	words_per_odd_line 	 = ((odd_line_bytes   + 3) >> 2 ); 		/* ceil(odd_line_bytes/4); word = 4 bytes */
	words_per_even_line  = ((even_line_bytes  + 3) >> 2 );
    words_for_first_line = words_per_odd_line + 2 + (hasSOLandEOL ? 1 : 0); /* + SOF +packet header + optionally (SOL), but (EOL) is not in the first line */
	words_per_odd_line 	+= (1 + (hasSOLandEOL ? 2 : 0));  /* each non-first line has format header, and optionally (SOL) and (EOL). */
	words_per_even_line += (1 + (hasSOLandEOL ? 2 : 0));

	mem_words_per_odd_line 	 = ((words_per_odd_line + 7) >> 3); 	/* ceil(words_per_odd_line/8); mem_word = 32 bytes, 8 words */
	mem_words_for_first_line = ((words_for_first_line + 7) >> 3);
	mem_words_per_even_line  = ((words_per_even_line + 7) >> 3);
	mem_words_for_EOF        = 1; /* last line consisit of the optional (EOL) and EOF */

	mem_words = ((embedded_data_size_words + 7) >> 3) +
                mem_words_for_first_line + 
				(((height + 1) >> 1) - 1) * mem_words_per_odd_line + /* ceil (height/2) - 1 (first line is calculated separatelly) */
				(  height      >> 1     ) * mem_words_per_even_line + /* floor(height/2) */
				mem_words_for_EOF;
	
	*size_mem_words = mem_words; /* ceil(words/8); mem word is 32B = 8words. */ //Check if this is still needed.

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_mipi_frame_calculate_size() leave: return_err=%d\n",err);
	
	return err;
}

enum ia_css_err ia_css_frame_allocate_contiguous(
	struct ia_css_frame **frame,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format,
	unsigned int padded_width,
	unsigned int raw_bit_depth)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate_contiguous() "
		"enter: width=%d, height=%d, format=%d\n",
		width, height, format);

	err = allocate_frame_and_data(frame, width, height, format,
					padded_width, raw_bit_depth, true);

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate_contiguous() leave: frame=%p\n",
		frame ? *frame : (void *)-1);

	return err;
}

enum ia_css_err ia_css_frame_allocate_contiguous_from_info(
	struct ia_css_frame **frame,
	const struct ia_css_frame_info *info)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
assert(frame != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate_contiguous_from_info() enter:\n");
	err = ia_css_frame_allocate_contiguous(frame,
						info->res.width,
						info->res.height,
						info->format,
						info->padded_width,
						info->raw_bit_depth);
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_allocate_contiguous_from_info() leave:\n");
return err;
}

void
ia_css_frame_free(struct ia_css_frame *frame)
{
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_free() enter: frame=%p\n", frame);

	if (frame != NULL) {
		mmgr_free(frame->data);
		sh_css_free(frame);
	}
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_frame_free() leave: return_void\n");
}

bool sh_css_frame_info_equal_resolution(
	const struct ia_css_frame_info *info_a,
	const struct ia_css_frame_info *info_b)
{
	if (!info_a || !info_b)
		return false;
	return (info_a->res.width == info_b->res.width) &&
	    (info_a->res.height == info_b->res.height);
}

bool sh_css_frame_equal_types(
	const struct ia_css_frame *frame_a,
	const struct ia_css_frame *frame_b)
{
	bool is_equal = false;
	const struct ia_css_frame_info *info_a = &frame_a->info,
	    *info_b = &frame_b->info;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_frame_equal_types() enter:\n");

	if (!info_a || !info_b)
		return false;
	if (info_a->format != info_b->format)
		return false;
	if (info_a->padded_width != info_b->padded_width)
		return false;
	is_equal = sh_css_frame_info_equal_resolution(info_a, info_b);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_frame_equal_types() leave:\n");
return is_equal;
}

static void
append_firmware(struct ia_css_fw_info **l, struct ia_css_fw_info *firmware)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "append_firmware() enter:\n");
	while (*l)
		l = &(*l)->next;
	*l = firmware;
	firmware->next = NULL;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "append_firmware() leave:\n");
}

static void
remove_firmware(struct ia_css_fw_info **l, struct ia_css_fw_info *firmware)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "remove_firmware() enter:\n");
	while (*l && *l != firmware)
		l = &(*l)->next;
	if (!*l)
		return;
	*l = firmware->next;
	firmware->next = NULL;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "remove_firmware() leave:\n");
}

/* Load firmware for acceleration */

/* Unload firmware for acceleration */
//fix me, to be removed 
void
sh_css_unload_acceleration(struct ia_css_acc_fw *firmware)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_unload_acceleration() enter:\n");
	sh_css_acc_unload(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_unload_acceleration() leave:\n");
}

/* Load firmware for extension */
static enum ia_css_err
ia_css_pipe_load_extension(struct ia_css_pipe *pipe,
			   struct ia_css_fw_info *firmware)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_load_extension() enter:\n");
	if (firmware->info.isp.type == IA_CSS_ACC_OUTPUT)
		append_firmware(&pipe->old_pipe->output_stage, firmware);
	else if (firmware->info.isp.type == IA_CSS_ACC_VIEWFINDER)
		append_firmware(&pipe->old_pipe->vf_stage, firmware);
	err = sh_css_acc_load_extension(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_load_extension() leave:\n");
	return err;
}

/* Unload firmware for extension */
static void
ia_css_pipe_unload_extension(struct ia_css_pipe *pipe,
			     struct ia_css_fw_info *firmware)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_unload_extension() enter:\n");
	if (firmware->info.isp.type == IA_CSS_ACC_OUTPUT)
		remove_firmware(&pipe->old_pipe->output_stage, firmware);
	else if (firmware->info.isp.type == IA_CSS_ACC_VIEWFINDER)
		remove_firmware(&pipe->old_pipe->vf_stage, firmware);
	sh_css_acc_unload_extension(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_pipe_unload_extension() leave:\n");
}

/* Set acceleration parameter to value <val> */
enum ia_css_err
sh_css_set_acceleration_parameter(struct ia_css_acc_fw *firmware,
				  hrt_vaddress val, size_t size)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_data par; 
	par.address =  val;
	par.size = size ;
	
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_set_acceleration_parameter() enter:\n");
	err = sh_css_acc_set_parameter(firmware, par);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_set_acceleration_parameter() leave:\n");
return err;
}

/* Set acceleration parameters to value <val> */
enum ia_css_err
sh_css_set_firmware_dmem_parameters(struct ia_css_fw_info *firmware,
				    enum ia_css_isp_memories mem,
				  hrt_vaddress val, size_t size)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct ia_css_data par; 
	par.address =  val;
	par.size = size ;
	
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_set_firmware_dmem_parameters() enter:\n");
	err = sh_css_acc_set_firmware_parameters(firmware, mem, par);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_set_firmware_dmem_parameters() leave:\n");
return err;
}

/* Start acceleration of firmware with sp-args as SP arguments. */
enum ia_css_err
sh_css_start_acceleration(struct ia_css_acc_fw *firmware)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_start_acceleration() enter:\n");
	err = sh_css_acc_start(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_start_acceleration() leave:\n");
	return err;
}

/* To be called when acceleration has terminated.
*/
void
sh_css_acceleration_done(struct ia_css_acc_fw *firmware)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_acceleration_done() enter: firmware=%p\n", firmware);
	sh_css_acc_wait();
	sh_css_acc_done(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_acceleration_done() leave: return_void\n");
}

/* Abort acceleration within <deadline> microseconds
*/
void
sh_css_abort_acceleration(struct ia_css_acc_fw *firmware, unsigned deadline)
{
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_abort_acceleration() enter:\n");
	/* TODO: implement time-out */
	(void)deadline;
	sh_css_acc_abort(firmware);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_abort_acceleration() leave:\n");
}

bool
sh_css_pipe_uses_params(struct sh_css_pipeline *me)
{
	struct sh_css_pipeline_stage *stage;

assert(me != NULL);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipe_uses_params() enter: me=%p\n", me);

	for (stage = me->stages; stage; stage = stage->next)
		if (stage->binary_info && stage->binary_info->enable.params) {
			sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
				"sh_css_pipe_uses_params() leave: "
				"return_bool=true\n");
			return true;
		}
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_pipe_uses_params() leave: return_bool=false\n");
	return false;
}

/* Create a pipeline stage for firmware <isp_fw>
 * with input and output arguments.
*/
static enum ia_css_err sh_css_create_stage(
	struct sh_css_pipeline_stage **stage,
	const char *isp_fw,
	struct ia_css_frame *in,
	struct ia_css_frame *out,
	struct ia_css_frame *vf)
{
	struct sh_css_binary *binary;
	struct ia_css_blob_descr *blob;
	struct ia_css_binary_info *info;
	unsigned size;
	enum ia_css_err err = IA_CSS_SUCCESS;

assert(stage != NULL);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_create_stage() enter:\n");

	*stage = sh_css_malloc(sizeof(**stage));
	if (*stage == NULL) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_create_stage() leave: return_err=%d\n",
			IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

	binary = sh_css_malloc(sizeof(*binary));
	if (binary == NULL) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_create_stage() leave: return_err=%d\n",
			IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

	blob = sh_css_malloc(sizeof(*blob));
	if (blob == NULL) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_create_stage() leave: return_err=%d\n",
			IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}

	memset(&(*stage)->args, 0, sizeof((*stage)->args));
	(*stage)->args.in_frame = in;
	(*stage)->args.out_frame = out;
	(*stage)->args.out_vf_frame = vf;

	err = sh_css_load_blob_info(isp_fw, blob);
	if (err != IA_CSS_SUCCESS) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_create_stage() leave: return_err=%d\n",
			err);
		return err;
	}
	err = sh_css_fill_binary_info(&blob->header.info.isp, false, false,
			    IA_CSS_STREAM_FORMAT_RAW_10,
			    in  ? &in->info  : NULL,
			    out ? &out->info : NULL,
			    vf  ? &vf->info  : NULL,
			    binary, false, NULL);
	if (err != IA_CSS_SUCCESS) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_create_stage() leave: return_err=%d\n",
			err);
		return err;
	}
	blob->header.info.isp.xmem_addr = 0;
	size = blob->header.blob.size;
	if (size) {
		blob->header.info.isp.xmem_addr =
			sh_css_load_blob(blob->blob, size);
		if (!blob->header.info.isp.xmem_addr) {
			sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
				"sh_css_create_stage() leave: return_err=%d\n",
				IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
			return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
	}

	info = (struct ia_css_binary_info *)binary->info;
	info->blob = blob;
	(*stage)->binary = binary;
	(*stage)->binary_info = &blob->header.info.isp;
	(*stage)->firmware = NULL;
	(*stage)->mode = binary->info->mode;
	(*stage)->out_frame_allocated = false;
	(*stage)->vf_frame_allocated = false;
	(*stage)->next = NULL;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_create_stage() leave: return_err=%d\n", err);

	return err;
}

/* Append a new stage to *pipeline. When *pipeline is NULL, it will be created.
 * The stage consists of an ISP binary <isp_fw> and input and output arguments.
*/
enum ia_css_err
sh_css_append_stage(struct sh_css_pipeline **pipeline,
		    const char *isp_fw,
		    struct ia_css_frame *in,
		    struct ia_css_frame *out,
		    struct ia_css_frame *vf)
{
	struct sh_css_pipeline_stage *stage;
	enum ia_css_err err = IA_CSS_SUCCESS;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE, "sh_css_append_stage() enter: "
		"pipeline=%p, isp_f=%p, in=%p, out=%p, vf=%p\n",
		pipeline, isp_fw, in, out, vf);

	if (!*pipeline) {
		*pipeline = sh_css_create_pipeline();
		if (*pipeline == NULL) {

			sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
				"sh_css_append_stage() leave: return_err=%d\n",
				IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY);
			return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
	}

	err = sh_css_create_stage(&stage, isp_fw, in, out, vf);
	if (err != IA_CSS_SUCCESS) {
		sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
			"sh_css_append_stage() leave: return_err=%d\n", err);

		return err;
	}

	stage->stage_num = (*pipeline)->num_stages++;
	if ((*pipeline)->current_stage)
		(*pipeline)->current_stage->next = stage;
	else
		(*pipeline)->stages = stage;

	(*pipeline)->current_stage = stage;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_append_stage() leave: return_err=%d\n", err);

	return err;
}

/* #error return of function is not consistent with implementation */
struct sh_css_pipeline *
sh_css_create_pipeline(void)
{
	struct sh_css_pipeline *pipeline = sh_css_malloc(sizeof(struct sh_css_pipeline));

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_create_pipeline() enter:\n");

	if (pipeline != NULL) {
		pipeline->num_stages = 0;
		pipeline->stages = NULL;
		pipeline->current_stage = NULL;
	}

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_create_pipeline() leave: pipeline=%p\n");
	return pipeline;
}

enum ia_css_err
sh_css_pipeline_add_acc_stage(struct sh_css_pipeline *pipeline,
			      const void *acc_fw)
{
	struct ia_css_fw_info *fw = (struct ia_css_fw_info *)acc_fw;
	enum ia_css_err	err = sh_css_acc_load_extension(fw);

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipeline_add_acc_stage() enter: pipeline=%p,"
		" acc_fw=%p\n", pipeline, acc_fw);

	if (err == IA_CSS_SUCCESS) {
		err = sh_css_pipeline_add_stage(
			pipeline, NULL, fw,
			SH_CSS_BINARY_MODE_VF_PP, NULL,
			NULL, NULL,
			NULL,NULL);
	}

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_pipeline_add_acc_stage() leave: return_err=%d\n",err);
	return err;
}

/* Run a pipeline and wait till it completes. */
void
sh_css_start_pipeline(enum ia_css_pipe_id pipe_id, struct sh_css_pipeline *pipeline)
{
	uint8_t pipe_num = 0;
	unsigned int thread_id;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_start_pipeline() enter: pipe_id=%d, pipeline=%p\n",
		pipe_id, pipeline);
	pipeline->pipe_id = pipe_id;
	sh_css_sp_init_pipeline(pipeline, pipe_id, pipe_num,
				false, true, false, false, false, true, false,
				SH_CSS_PIPE_CONFIG_OVRD_NO_OVRD,
				IA_CSS_INPUT_MODE_MEMORY,
				(mipi_port_ID_t) 0);
	//sh_css_sp_start_isp();

	//TODO: fix here, pipe_num
	sh_css_query_sp_thread_id(pipe_num, &thread_id);
	sh_css_sp_snd_event(SP_SW_EVENT_ID_4, thread_id, 0,  0);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_start_pipeline() leave: return_void\n");
}

/* Run a pipeline and free all memory allocated to it. */
void
sh_css_close_pipeline(struct sh_css_pipeline *pipeline)
{
	struct sh_css_pipeline_stage *stage;
	struct sh_css_pipeline_stage *next;

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_close_pipeline() enter: pipeline=%p\n", pipeline);

	for (stage = pipeline->stages; stage; stage = next) {
		struct ia_css_blob_descr *blob;
		next = stage->next;
		blob = (struct ia_css_blob_descr *)stage->binary->info->blob;
		if (blob->header.info.isp.xmem_addr)
			mmgr_free(blob->header.info.isp.xmem_addr);
		sh_css_free(blob);
		sh_css_free(stage->binary);
		sh_css_pipeline_stage_destroy(stage);
	}
	sh_css_free(pipeline);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_close_pipeline() leave: return_void\n");
}

/**
 * @brief Query the SP thread ID.
 * Refer to "sh_css_internal.h" for details.
 */
bool
sh_css_query_sp_thread_id(unsigned int key,
			  unsigned int *val)
{
assert(key < MAX_NUM_PIPES);
assert(key < IA_CSS_PIPE_ID_NUM);
assert(val != NULL);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_query_sp_thread_id() enter: key=%d\n", key);
	*val = pipe_num_to_sp_thread[key];
		assert(*val != 0xFFFF);
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_query_sp_thread_id() leave: return_val=%d\n", *val);
	return true;
}

/**
 * @brief Query the internal frame ID.
 * Refer to "sh_css_internal.h" for details.
 */
bool sh_css_query_internal_queue_id(
	enum ia_css_buffer_type key,
	enum sh_css_buffer_queue_id *val)
{
assert(key < IA_CSS_BUFFER_TYPE_NUM);
assert(val != NULL);

	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_query_internal_queue_id() enter: key=%d\n", key);
	*val = sh_css_buf_type_2_internal_queue_id[key];
	sh_css_dtrace(SH_DBG_TRACE_PRIVATE,
		"sh_css_query_internal_queue_id() leave: return_val=%d\n",
		*val);
	return true;
}

/**
 * @brief Tag a specific frame in continuous capture.
 * Refer to "sh_css_internal.h" for details.
 */
enum ia_css_err ia_css_stream_capture_frame(struct ia_css_stream *stream,
				unsigned int exp_id)
{
	struct sh_css_tag_descr tag_descr;
	unsigned int encoded_tag_descr;

	bool enqueue_successful = false;
	assert(stream != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture_frame() enter: exp_id=%d\n",
		exp_id);

	if (exp_id == 0) {
		sh_css_dtrace(SH_DBG_TRACE,
			"ia_css_stream_capture_frame() "
			"leave: return_err=%d\n",
			IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	/* Create the tag descriptor from the parameters */
	sh_css_create_tag_descr(0, 0, 0, exp_id, &tag_descr);


	/* Encode the tag descriptor into a 32-bit value */
	encoded_tag_descr = sh_css_encode_tag_descr(&tag_descr);


	/* Enqueue the encoded tag to the host2sp queue.
	 * Note: The pipe and stage IDs for tag_cmd queue are hard-coded to 0
	 * on both host and the SP side.
	 * It is mainly because it is enough to have only one tag_cmd queue */
	enqueue_successful = host2sp_enqueue_buffer(0, 0,
				sh_css_tag_cmd_queue,
				(uint32_t)encoded_tag_descr);


	/* Give an error if the tag command cannot be issued
	 * (because the cmd queue is full) */
	if (!enqueue_successful) {
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture_frame() leave: return_err=%d\n",
		IA_CSS_ERR_QUEUE_IS_FULL);
		return IA_CSS_ERR_QUEUE_IS_FULL;
	}

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture_frame() leave: return_err=%d\n",
		IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

/**
 * @brief Configure the continuous capture.
 * Refer to "sh_css_internal.h" for details.
 */
enum ia_css_err ia_css_stream_capture(
	struct ia_css_stream *stream,
	int num_captures,
	unsigned int skip,
	int offset)
{
	struct sh_css_tag_descr tag_descr;
	unsigned int encoded_tag_descr;

	bool enqueue_successful = false;
	assert(stream != NULL);
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture() enter: num_captures=%d,"
		" skip=%d, offset=%d\n", num_captures, skip,offset);

	/* Check if the tag descriptor is valid */
	if (num_captures < SH_CSS_MINIMUM_TAG_ID) {
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture() leave: return_err=%d\n",
		IA_CSS_ERR_INVALID_ARGUMENTS);
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	/* Create the tag descriptor from the parameters */
	sh_css_create_tag_descr(num_captures, skip, offset, 0, &tag_descr);


	/* Encode the tag descriptor into a 32-bit value */
	encoded_tag_descr = sh_css_encode_tag_descr(&tag_descr);


	/* Enqueue the encoded tag to the host2sp queue.
	 * Note: The pipe and stage IDs for tag_cmd queue are hard-coded to 0
	 * on both host and the SP side.
	 * It is mainly because it is enough to have only one tag_cmd queue */
	enqueue_successful = host2sp_enqueue_buffer(0, 0,
				sh_css_tag_cmd_queue,
				(uint32_t)encoded_tag_descr);


	/* Give an error if the tag command cannot be issued
	 * (because the cmd queue is full) */
	if (!enqueue_successful) {
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture() leave: return_err=%d\n",
		IA_CSS_ERR_QUEUE_IS_FULL);
		return IA_CSS_ERR_QUEUE_IS_FULL;
	}

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_capture() leave: return_err=%d\n",
		IA_CSS_SUCCESS);
	return IA_CSS_SUCCESS;
}

void ia_css_stream_request_flash(struct ia_css_stream *stream)
{
	const struct ia_css_fw_info *fw= &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_request_flash;
	assert(stream != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_request_flash() enter: void\n");
	HIVE_ADDR_sp_request_flash = fw->info.sp.request_flash;

	(void)HIVE_ADDR_sp_request_flash;

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_request_flash),
		1);

	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_request_flash() leave: return_void\n");
}

enum ia_css_err
sh_css_acceleration_stop(struct ia_css_pipe *pipe)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_acceleration_stop() enter: void\n");
	err = sh_css_pipe_stop(pipe);
	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_acceleration_stop() leave: return_err=%d\n",err);
	return err;
}

static void
sh_css_init_host_sp_control_vars(void)
{
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_sp_isp_started;
	
	unsigned int HIVE_ADDR_host_sp_queues_initialized;
	unsigned int HIVE_ADDR_sp_sleep_mode;
	unsigned int HIVE_ADDR_sp_invalidate_tlb;
	unsigned int HIVE_ADDR_sp_request_flash;
	unsigned int HIVE_ADDR_sp_stop_copy_preview;
	unsigned int HIVE_ADDR_host_sp_com;
	unsigned int o = offsetof(struct host_sp_communication, host2sp_command)
				/ sizeof(int);

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_init_host_sp_control_vars() enter: void\n");

	fw = &sh_css_sp_fw;
	HIVE_ADDR_sp_isp_started = fw->info.sp.isp_started;
	
	HIVE_ADDR_host_sp_queues_initialized =
		fw->info.sp.host_sp_queues_initialized;
	HIVE_ADDR_sp_sleep_mode = fw->info.sp.sleep_mode;
	HIVE_ADDR_sp_invalidate_tlb = fw->info.sp.invalidate_tlb;
	HIVE_ADDR_sp_request_flash = fw->info.sp.request_flash;
	HIVE_ADDR_sp_stop_copy_preview = fw->info.sp.stop_copy_preview;
	HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;

	(void)HIVE_ADDR_sp_isp_started; /* Suppres warnings in CRUN */
	
	(void)HIVE_ADDR_sp_sleep_mode;
	(void)HIVE_ADDR_sp_invalidate_tlb;
	(void)HIVE_ADDR_sp_request_flash;
	(void)HIVE_ADDR_sp_stop_copy_preview;
	(void)HIVE_ADDR_host_sp_com;

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_isp_started),
		(uint32_t)(0));
	
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(host_sp_queues_initialized),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_sleep_mode),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_invalidate_tlb),
		(uint32_t)(false));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_request_flash),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_stop_copy_preview),
		my_css.stop_copy_preview?(uint32_t)(1):(uint32_t)(0));
	store_sp_array_uint(host_sp_com, o, host2sp_cmd_ready);
	sh_css_update_host2sp_cont_num_raw_frames
			(my_css.num_cont_raw_frames);
	sh_css_update_host2sp_cont_num_mipi_frames
			(my_css.num_mipi_frames);

	sh_css_dtrace(SH_DBG_TRACE,
		"sh_css_init_host_sp_control_vars() leave: return_void\n");
}

void
ia_css_get_properties(struct ia_css_properties *properties)
{
#if defined(HAS_GDC_VERSION_2)
/*
 * MW: We don't want to store the coordinates
 * full range in memory: Truncate
 */
	properties->gdc_coord_one = gdc_get_unity(GDC0_ID)/HRT_GDC_COORD_SCALE;
#elif defined(HAS_GDC_VERSION_1)
	properties->gdc_coord_one = gdc_get_unity(GDC0_ID);
#else
#error "Unknown GDC version"
#endif

#if defined(IS_ISP_2300_SYSTEM)
	properties->l1_base_is_index = false;
#else
	properties->l1_base_is_index = true;
#endif

#if defined(HAS_VAMEM_VERSION_1)
	properties->vamem_type = IA_CSS_VAMEM_TYPE_1;
#elif defined(HAS_VAMEM_VERSION_2)
	properties->vamem_type = IA_CSS_VAMEM_TYPE_2;
#else
#error "Unknown VAMEM version"
#endif
}

/**
 * create the internal structures and fill in the configuration data
 */
void ia_css_pipe_config_defaults(struct ia_css_pipe_config *pipe_config)
{
  struct ia_css_pipe_config def_config = {
    0,
    {0, 0},
    {0, 0},
    {0, 0},
    {{0, 0}, 0, 0, 0, 0}, /* output_info */
    {{0, 0}, 0, 0, 0, 0}, /* vf_output_info */
    NULL, /* acc_extension */
    NULL, /* acc_stages */
    0,    /* num_acc_stages */
    {
      0,
      false,
      false,
      false
    },
    {0, 0},
  };
  sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_config_defaults()\n");
  *pipe_config = def_config;
}

void
ia_css_pipe_extra_config_defaults(struct ia_css_pipe_extra_config *extra_config)
{
	extra_config->enable_raw_binning = false;
	extra_config->enable_yuv_ds = false;
	extra_config->enable_high_speed = false;
	extra_config->enable_dvs_6axis = false;
	extra_config->enable_reduced_pipe = false;
	extra_config->isp_pipe_version = 1;
	extra_config->disable_vf_pp = false;
	extra_config->disable_capture_pp = false;
	extra_config->enable_dz = true;
}

void ia_css_stream_config_defaults(struct ia_css_stream_config *stream_config)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_config_defaults()\n");
	assert(stream_config != NULL);
	memset(stream_config, 0, sizeof(*stream_config));
	stream_config->online = true;
}

static enum ia_css_err
ia_css_acc_pipe_create(struct ia_css_pipe *pipe)
{
	unsigned int i;
	enum ia_css_err err = IA_CSS_SUCCESS;
	struct sh_css_pipeline *pipeline = &pipe->old_pipe->pipeline;

	for (i=0; i<pipe->config.num_acc_stages; i++) {
		struct ia_css_fw_info *fw = pipe->config.acc_stages[i];
		err = sh_css_pipeline_add_acc_stage(pipeline, fw);
		if (err != IA_CSS_SUCCESS)
			break;
	}
	return err;
}

enum ia_css_err
ia_css_pipe_create(const struct ia_css_pipe_config *config,
		   struct ia_css_pipe **pipe)
{
	return ia_css_pipe_create_extra(config, NULL, pipe);
}

enum ia_css_err
ia_css_pipe_create_extra(const struct ia_css_pipe_config *config,
			 const struct ia_css_pipe_extra_config *extra_config,
			 struct ia_css_pipe **pipe)
{
	enum ia_css_err err = IA_CSS_ERR_INTERNAL_ERROR;
	struct ia_css_pipe *internal_pipe;

	(void)extra_config;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_pipe_create()\n");
	assert(pipe!=NULL);
	assert(*pipe==NULL);
	if (pipe == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (*pipe != NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	
	err = create_pipe(config->mode, &internal_pipe, false);
	if (internal_pipe == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	
	if (pipe_num_counter > MAX_NUM_PIPES) return IA_CSS_ERR_RESOURCE_LIST_TO_SMALL;

	/* now we have a pipe structure to fill */
	internal_pipe->config = *config;
	if (extra_config)
		internal_pipe->extra_config = *extra_config;
	else
		ia_css_pipe_extra_config_defaults(&internal_pipe->extra_config);

	if (config->mode == IA_CSS_PIPE_MODE_ACC) {
		/* Temporary hack to migrate acceleration to CSS 2.0.
		 * In the future the code for all pipe types should be
		 * unified. */
		*pipe = internal_pipe;
		return ia_css_acc_pipe_create(internal_pipe);
	}
	/* handle the mode, for now select the correct pipe instance */
	internal_pipe->old_pipe->capture_mode =
		internal_pipe->config.default_capture_config.mode;
	internal_pipe->old_pipe->xnr =
		(bool)internal_pipe->config.default_capture_config.enable_xnr;
	/* DVS envelope */
	internal_pipe->old_pipe->dvs_envelope =
		internal_pipe->config.dvs_envelope;
	/* YUV downscaling */
	if (internal_pipe->config.bayer_ds_out_res.width &&
	    (internal_pipe->config.mode == IA_CSS_PIPE_MODE_PREVIEW ||
	     internal_pipe->config.mode == IA_CSS_PIPE_MODE_CAPTURE)) {
		enum ia_css_frame_format format;
		if (internal_pipe->config.mode == IA_CSS_PIPE_MODE_PREVIEW)
			format = IA_CSS_FRAME_FORMAT_YUV_LINE;
		else
			format = IA_CSS_FRAME_FORMAT_YUV420;
		sh_css_frame_info_init(
				&internal_pipe->old_pipe->yuv_ds_input_info,
				internal_pipe->config.bayer_ds_out_res.width,
				internal_pipe->config.bayer_ds_out_res.height,
				format);
	}
	/* handle output info, asume always needed */
	if (internal_pipe->config.output_info.res.width) {
		err = sh_css_pipe_configure_output(
				internal_pipe->old_pipe,
				internal_pipe->config.output_info.res.width,
				internal_pipe->config.output_info.res.height,
				internal_pipe->config.output_info.format);
		if (err != IA_CSS_SUCCESS) {
			sh_css_dtrace(SH_DBG_ERROR, "ia_css_pipe_create: "
							"invalid output info\n");
			sh_css_free(internal_pipe);
			internal_pipe = NULL;
			return err;
		}
	}
	/* handle vf output info, when configured */
	internal_pipe->old_pipe->enable_viewfinder = (internal_pipe->config.vf_output_info.res.width != 0);
	if (internal_pipe->config.vf_output_info.res.width) {
		err = sh_css_pipe_configure_viewfinder(
				internal_pipe->old_pipe,
				internal_pipe->config.vf_output_info.res.width,
				internal_pipe->config.vf_output_info.res.height,
				internal_pipe->config.vf_output_info.format);
		if (err != IA_CSS_SUCCESS) {
			sh_css_dtrace(SH_DBG_ERROR, "ia_css_pipe_create: "
							"invalid vf output info\n");
			sh_css_free(internal_pipe);
			internal_pipe = NULL;
			return err;
		}
	}
	internal_pipe->old_pipe->input_needs_raw_binning =
		internal_pipe->extra_config.enable_raw_binning;
	internal_pipe->old_pipe->enable_yuv_ds =
		internal_pipe->extra_config.enable_yuv_ds;
	internal_pipe->old_pipe->enable_high_speed =
		internal_pipe->extra_config.enable_high_speed;
	internal_pipe->old_pipe->enable_dvs_6axis =
		internal_pipe->extra_config.enable_dvs_6axis;
	internal_pipe->old_pipe->enable_reduced_pipe =
		internal_pipe->extra_config.enable_reduced_pipe;
	internal_pipe->old_pipe->enable_dz =
		internal_pipe->extra_config.enable_dz;
	internal_pipe->old_pipe->isp_pipe_version =
		internal_pipe->extra_config.isp_pipe_version;
	internal_pipe->old_pipe->disable_vf_pp =
		internal_pipe->extra_config.disable_vf_pp;
	internal_pipe->old_pipe->disable_capture_pp =
		internal_pipe->extra_config.disable_capture_pp;
	if (internal_pipe->config.acc_extension) {
		ia_css_pipe_load_extension(internal_pipe,
			internal_pipe->config.acc_extension);
	}
	/* set all info to zeroes first */
	memset(&internal_pipe->info, 0, sizeof(internal_pipe->info));

	/* all went well, return the pipe */
	*pipe = internal_pipe;
	return IA_CSS_SUCCESS;
}


enum ia_css_err
ia_css_pipe_get_info(const struct ia_css_pipe *pipe,
		     struct ia_css_pipe_info *pipe_info)
{
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_pipe_get_info()\n");
	assert(pipe_info != NULL);
	if (pipe_info == NULL) {
		sh_css_dtrace(SH_DBG_ERROR,
			"ia_css_pipe_get_info: pipe_info cannot be NULL\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	if (pipe->old_pipe->stream == NULL) {
		sh_css_dtrace(SH_DBG_ERROR,
			"ia_css_pipe_get_info: ia_css_stream_create needs to"
			" be called before ia_css_[stream/pipe]_get_info\n");
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	/* we succeeded return the info */
	*pipe_info = pipe->info;
	return IA_CSS_SUCCESS;
}

static enum ia_css_err
ia_css_stream_configure_rx(struct ia_css_stream *stream)
{
#if defined(HAS_RX_VERSION_1)
	struct ia_css_input_port *config = &stream->config.source.port;

	if (config->port == IA_CSS_CSI2_PORT_1LANE)
		stream->csi_rx_config.port = MIPI_PORT1_ID;
	else if (config->port == IA_CSS_CSI2_PORT_4LANE)
		stream->csi_rx_config.port = MIPI_PORT0_ID;
	else
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	stream->csi_rx_config.num_lanes  = config->num_lanes;
	stream->csi_rx_config.timeout    = config->timeout;
	stream->csi_rx_config.uncomp_bpp =
		config->compression.uncompressed_bits_per_pixel;
	stream->csi_rx_config.comp_bpp   =
		config->compression.compressed_bits_per_pixel;
	if (config->compression.type == IA_CSS_CSI2_COMPRESSION_TYPE_NONE)
		stream->csi_rx_config.comp = MIPI_PREDICTOR_NONE;
	else if (config->compression.type == IA_CSS_CSI2_COMPRESSION_TYPE_1)
		stream->csi_rx_config.comp = MIPI_PREDICTOR_TYPE1;
	else if (config->compression.type == IA_CSS_CSI2_COMPRESSION_TYPE_2)
		stream->csi_rx_config.comp = MIPI_PREDICTOR_TYPE2;
	else
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	stream->csi_rx_config.is_two_ppc = stream->config.two_pixels_per_clock;
	stream->reconfigure_css_rx = true;
#elif defined(HAS_RX_VERSION_2)
	struct ia_css_input_port *config = &stream->config.source.port;

// AM: this code is not reliable, especially for 2400
	if (config->num_lanes == 1)
		stream->csi_rx_config.mode = MONO_1L_1L_0L;
	else if (config->num_lanes == 2)
		stream->csi_rx_config.mode = MONO_2L_1L_0L;
	else if (config->num_lanes == 3)
		stream->csi_rx_config.mode = MONO_3L_1L_0L;
	else if (config->num_lanes == 4)
		stream->csi_rx_config.mode = MONO_4L_1L_0L;
	else if (config->num_lanes != 0)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	if (config->port == IA_CSS_CSI2_PORT_1LANE)
		stream->csi_rx_config.port = MIPI_PORT1_ID;
	else if (config->port == IA_CSS_CSI2_PORT_2LANE)
		stream->csi_rx_config.port = MIPI_PORT2_ID;
	else if (config->port == IA_CSS_CSI2_PORT_4LANE)
		stream->csi_rx_config.port = MIPI_PORT0_ID;
	else
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	stream->csi_rx_config.timeout    = config->timeout;
	stream->csi_rx_config.initcount  = 0;
	stream->csi_rx_config.synccount  = 0x28282828;
	stream->csi_rx_config.rxcount    = 0x04040404;
	if (config->compression.type == IA_CSS_CSI2_COMPRESSION_TYPE_NONE)
		stream->csi_rx_config.comp = MIPI_PREDICTOR_NONE;
	else {
		/* not implemented yet, requires extension of the rx_cfg_t
		 * struct */
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	}
	stream->csi_rx_config.is_two_ppc = stream->config.two_pixels_per_clock;
	stream->reconfigure_css_rx = true;
#else
	/* No CSS receiver */
	(void)stream;
#endif
	return IA_CSS_SUCCESS;
}

static struct ia_css_pipe *
find_pipe(struct ia_css_pipe *pipes[],
		unsigned int num_pipes,
		enum ia_css_pipe_mode mode,
		bool copy_pipe)
{
	unsigned i;
	for (i = 0; i < num_pipes; i++) {
		if (pipes[i]->config.mode != mode)
			continue;
		if (copy_pipe && pipes[i]->old_pipe->mode != IA_CSS_PIPE_ID_COPY)
			continue;
		return pipes[i];
	}
	return NULL;
}

static enum ia_css_err
ia_css_acc_stream_create(struct ia_css_stream *stream)
{
	int i;
	for (i=0; i< stream->num_pipes; i++) {
		struct ia_css_pipe *pipe = stream->pipes[i];
		pipe->old_pipe->stream = stream;
	}
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_create(const struct ia_css_stream_config *stream_config,
					 int num_pipes,
					 struct ia_css_pipe *pipes[],
					 struct ia_css_stream **stream)
{
	struct ia_css_pipe *curr_pipe;
	struct ia_css_stream *curr_stream = NULL;
	bool sensor_binning_changed;
	int i;
	enum ia_css_err err = IA_CSS_ERR_INTERNAL_ERROR;
	sh_css_dtrace(SH_DBG_TRACE,
		"ia_css_stream_create() enter, num_pipes=%d\n", num_pipes);
	/* some checks */
	assert(num_pipes != 0);
	assert(stream != NULL);
	assert(*stream == NULL);
	assert(pipes != NULL);
	/* allocate the stream instance */
	curr_stream = sh_css_malloc(sizeof(struct ia_css_stream));
	if (curr_stream == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	/* default all to 0 */
	memset(curr_stream, 0, sizeof(struct ia_css_stream));
	/* default mipi config */
#if defined(HAS_RX_VERSION_2)
	curr_stream->csi_rx_config.mode = MONO_4L_1L_0L; /* The HW config */
	curr_stream->csi_rx_config.port = MIPI_PORT0_ID; /* The port ID to apply the control on */
	curr_stream->csi_rx_config.timeout = 0xffff4;
	curr_stream->csi_rx_config.initcount = 0;
	curr_stream->csi_rx_config.synccount = 0x28282828;
	curr_stream->csi_rx_config.rxcount = 0x04040404;
	curr_stream->csi_rx_config.comp = MIPI_PREDICTOR_NONE;	/* Just for backward compatibility */
	curr_stream->csi_rx_config.is_two_ppc = false;
    curr_stream->reconfigure_css_rx = true;
#else
	curr_stream->csi_rx_config = (rx_cfg_t)DEFAULT_MIPI_CONFIG;
#endif
	/* allocate pipes */
	curr_stream->num_pipes = num_pipes;
	curr_stream->pipes = sh_css_malloc(num_pipes * sizeof(struct ia_css_pipe *));
	if (curr_stream->pipes == NULL)
		return IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	/* store pipes */
	for (i = 0; i < num_pipes; i++)
		curr_stream->pipes [i] = pipes[i];
	curr_stream->last_pipe = curr_stream->pipes[0];
	/* take over stream config */
	curr_stream->config = *stream_config;
	
	/* copy mode specific stuff */
	switch (curr_stream->config.mode) {
		case IA_CSS_INPUT_MODE_SENSOR:
		case IA_CSS_INPUT_MODE_BUFFERED_SENSOR:
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: mode port\n");
		/* CSI RX configuration */
		ia_css_stream_configure_rx(curr_stream);
		break;
	case IA_CSS_INPUT_MODE_TPG:
		sh_css_dtrace(SH_DBG_TRACE,
			"ia_css_stream_create tpg_configuration: "
			"x_mask=%d, y_mask=%d, x_delta=%d, "
			"y_delta=%d, xy_mask=%d\n",
			curr_stream->config.source.tpg.x_mask,
			curr_stream->config.source.tpg.y_mask,
			curr_stream->config.source.tpg.x_delta,
			curr_stream->config.source.tpg.y_delta,
			curr_stream->config.source.tpg.xy_mask);
		sh_css_sp_configure_tpg(
			curr_stream->config.source.tpg.x_mask,
			curr_stream->config.source.tpg.y_mask,
			curr_stream->config.source.tpg.x_delta,
			curr_stream->config.source.tpg.y_delta,
			curr_stream->config.source.tpg.xy_mask);
		break;
	case IA_CSS_INPUT_MODE_PRBS:
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: mode prbs\n");
		sh_css_sp_configure_prbs(curr_stream->config.source.prbs.seed);
		break;
	default:
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: mode sensor/default\n");
	}
	err = ia_css_stream_isp_parameters_init(curr_stream);
	if (err != IA_CSS_SUCCESS)
		return err;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: isp_params_configs: %p\n",curr_stream->isp_params_configs);

	if (num_pipes == 1 && pipes[0]->config.mode == IA_CSS_PIPE_MODE_ACC) {
		*stream = curr_stream;
		return ia_css_acc_stream_create(curr_stream);
	}
	/* sensor binning */
	sensor_binning_changed =
		sh_css_params_set_binning_factor(curr_stream, curr_stream->config.sensor_binning_factor);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: sensor_binning=%d, changed=%d\n",
		curr_stream->config.sensor_binning_factor, sensor_binning_changed);
	/* loop over pipes */
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: num_pipes=%d\n",
		num_pipes);
	curr_stream->cont_capt = false;
	/* Temporary hack: we give the preview pipe a reference to the capture
	 * pipe in continuous capture mode. */
	if (curr_stream->config.continuous && num_pipes >= 2) {
		/* Search for the preview pipe and create the copy pipe */
		struct ia_css_pipe *preview_pipe;
		struct ia_css_pipe *video_pipe;
		struct ia_css_pipe *capture_pipe;
		struct ia_css_pipe *copy_pipe;

		curr_stream->cont_capt = true;

		/* Create copy pipe here, since it may not be exposed to the driver */
		preview_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_PREVIEW, false);
		video_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_VIDEO, false);
		capture_pipe = find_pipe(pipes, num_pipes,
						IA_CSS_PIPE_MODE_CAPTURE, false);
		/* We do not support preview and video pipe at the same time */
		if (preview_pipe && video_pipe)
			return IA_CSS_ERR_INVALID_ARGUMENTS;

		if (preview_pipe && !preview_pipe->old_pipe->pipe.preview.copy_pipe) {
			create_pipe(IA_CSS_PIPE_MODE_CAPTURE, &copy_pipe, true);
			ia_css_pipe_config_defaults(&copy_pipe->config);
			preview_pipe->old_pipe->pipe.preview.copy_pipe =
				copy_pipe->old_pipe;
			copy_pipe->old_pipe->stream = curr_stream;
		}
		if (preview_pipe) {
			preview_pipe->old_pipe->pipe.preview.capture_pipe =
				capture_pipe->old_pipe;
		}
		if (video_pipe && !video_pipe->old_pipe->pipe.video.copy_pipe) {
			create_pipe(IA_CSS_PIPE_MODE_CAPTURE, &copy_pipe, true);
			ia_css_pipe_config_defaults(&copy_pipe->config);
			video_pipe->old_pipe->pipe.video.copy_pipe =
				copy_pipe->old_pipe;
			copy_pipe->old_pipe->stream = curr_stream;
		}
		if (video_pipe) {
			video_pipe->old_pipe->pipe.video.capture_pipe =
				capture_pipe->old_pipe;
		}
	}
	for (i = 0; i < num_pipes; i++) {
		curr_pipe = pipes[i];
		/* set current stream */
		curr_pipe->old_pipe->stream = curr_stream;
		/* take over effective info */
		sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create: effective_res=%dx%d\n",
			curr_stream->config.effective_res.width,
			curr_stream->config.effective_res.height);
		err = check_res(curr_stream->config.effective_res.width,
			curr_stream->config.effective_res.height);
		if (err != IA_CSS_SUCCESS)
			return err;

		if (curr_stream->info.effective_info.width != curr_stream->config.effective_res.width ||
			curr_stream->info.effective_info.height != curr_stream->config.effective_res.height) {
			curr_stream->info.effective_info.width = curr_stream->config.effective_res.width;
			curr_stream->info.effective_info.height = curr_stream->config.effective_res.height;
			sh_css_pipe_invalidate_binaries(curr_pipe->old_pipe);
		}
		/* sensor binning per pipe */
		if (sensor_binning_changed)
			sh_css_pipe_free_shading_table(curr_pipe->old_pipe);
	}
	/* now pipes have been configured, info should be available */
	for (i = 0; i < num_pipes; i++) {
		struct sh_css_pipe *old_pipe = NULL;
		struct ia_css_pipe_info *pipe_info = NULL;
		curr_pipe = pipes[i];
		/* handle each pipe */
		old_pipe = curr_pipe->old_pipe;
		old_pipe->pipe_num = curr_pipe->pipe_num;
		pipe_info = &curr_pipe->info;
		sh_css_pipe_get_output_frame_info(old_pipe,
					&pipe_info->output_info);
		sh_css_pipe_get_grid_info(old_pipe,
					&pipe_info->grid_info);
		sh_css_pipe_get_viewfinder_frame_info(old_pipe,
					&pipe_info->vf_output_info);
	}
	/* stream has been configured, info should be available */
	{
		/* use first pipe in list, should be identical anyway */
		// TODO: should come from stream something
		struct sh_css_pipe *old_pipe = pipes[0]->old_pipe;
		struct ia_css_stream_info *stream_info = &curr_stream->info;
		// TODO: JB implement stream info
		sh_css_pipe_get_input_resolution(old_pipe,
			&stream_info->raw_info.width,
			&stream_info->raw_info.height);
		
	}

	/* assign curr_stream */
	*stream = curr_stream;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_create() leave, err=%d\n",
			err);
	return err;
}

enum ia_css_err
ia_css_stream_destroy(struct ia_css_stream *stream)
{
	int i;
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_destroy: enter\n");
	assert(stream != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	ia_css_stream_isp_parameters_uninit(stream);

	/* remove references from pipes to stream */
	for (i = 0; i < stream->num_pipes; i++) {
		struct ia_css_pipe *entry = stream->pipes[i];
		assert(entry != NULL);
		if (entry->old_pipe != NULL) {
			/* clear reference to stream */
			entry->old_pipe->stream = NULL;
			/* check internal copy pipe */
			if (entry->old_pipe->mode == IA_CSS_PIPE_ID_PREVIEW &&
			    entry->old_pipe->pipe.preview.copy_pipe) {
				sh_css_dtrace(SH_DBG_TRACE,
					"ia_css_stream_destroy: "
					"clearing stream on internal copy pipe\n");
				entry->old_pipe->pipe.preview.copy_pipe->stream = NULL;
			}
		}
	}
	/* free associated memory of stream struct */
	sh_css_free(stream->pipes);
	stream->pipes = NULL;
	stream->num_pipes = 0;
	sh_css_free(stream);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_destroy: leave\n");
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_get_info(const struct ia_css_stream *stream,
		       struct ia_css_stream_info *stream_info)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_get_info: enter/exit\n");
	assert(stream != NULL);
	assert(stream_info != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	if (stream_info == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	*stream_info = stream->info;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_load(struct ia_css_stream *stream)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_load() enter/exit\n");
	assert(stream != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_stream_start(struct ia_css_stream *stream)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_start()\n");
	assert(stream != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	/* for now simple implementation, just start what seems right */
	assert(stream->last_pipe != NULL);
	assert(stream->last_pipe->old_pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_start: starting %d\n",
		stream->last_pipe->old_pipe->mode);
	return sh_css_pipe_start(stream);
}

enum ia_css_err
ia_css_stream_stop(struct ia_css_stream *stream)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_stop() enter/exit\n");
	assert(stream != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	/* for now simple implementation, just start what seems right */
	assert(stream->last_pipe != NULL);
	assert(stream->last_pipe->old_pipe != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_stop: stopping %d\n",
		stream->last_pipe->old_pipe->mode);

	// Check if this is the last pipe, if not return else stop everything
	//if (pipe_num_counter > 1) return IA_CSS_SUCCESS;
	return sh_css_pipe_request_stop(stream->last_pipe);
}

bool
ia_css_stream_has_stopped(struct ia_css_stream *stream)
{
	return sh_css_pipe_has_stopped(stream->last_pipe);
}

enum ia_css_err
ia_css_stream_unload(struct ia_css_stream *stream)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_stream_unload() enter/exit\n");
	assert(stream != NULL);
	if (stream == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;
	return IA_CSS_SUCCESS;
}

enum ia_css_err
ia_css_temp_pipe_to_pipe_id(const struct ia_css_pipe *pipe, enum ia_css_pipe_id *pipe_id)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_temp_pipe_to_pipe_id() enter/exit\n");
	if (pipe != NULL && pipe->old_pipe != NULL)
		*pipe_id = pipe->old_pipe->mode;
	else
		*pipe_id = IA_CSS_PIPE_ID_COPY;

	return IA_CSS_SUCCESS;
}

enum ia_css_stream_format
ia_css_stream_get_format(const struct ia_css_stream *stream)
{
	return stream->config.format;
}

bool
ia_css_stream_get_two_pixels_per_clock(const struct ia_css_stream *stream)
{
	return stream->config.two_pixels_per_clock;
}

struct sh_css_binary *
ia_css_stream_get_dvs_binary(const struct ia_css_stream *stream)
{
	int i;
	struct ia_css_pipe *video_pipe = NULL;

	/* First we find the video pipe */
	for (i=0; i<stream->num_pipes; i++) {
		struct ia_css_pipe *pipe = stream->pipes[i];
		if (pipe->config.mode == IA_CSS_PIPE_MODE_VIDEO) {
			video_pipe = pipe;
			break;
		}
	}
	if (video_pipe)
		return &video_pipe->old_pipe->pipe.video.video_binary;
	return NULL;
}

struct sh_css_binary *
ia_css_stream_get_3a_binary(const struct ia_css_stream *stream)
{
	struct ia_css_pipe *pipe = stream->pipes[0];

	if (stream->num_pipes == 2) {
		if (stream->pipes[1]->config.mode == IA_CSS_PIPE_MODE_VIDEO ||
		    stream->pipes[1]->config.mode == IA_CSS_PIPE_MODE_PREVIEW)
			pipe = stream->pipes[1];
	}

	return ia_css_pipe_get_3a_binary(pipe);
}

static struct sh_css_binary *
ia_css_pipe_get_3a_binary (const struct ia_css_pipe *pipe)
{
	struct sh_css_binary *s3a_binary = NULL;
	switch (pipe->config.mode) {
	case IA_CSS_PIPE_MODE_PREVIEW:
		s3a_binary = &pipe->old_pipe->pipe.preview.preview_binary;
		break;
	case IA_CSS_PIPE_MODE_VIDEO:
		s3a_binary = &pipe->old_pipe->pipe.video.video_binary;
		break;
	case IA_CSS_PIPE_MODE_CAPTURE:
		if (pipe->old_pipe->capture_mode == IA_CSS_CAPTURE_MODE_PRIMARY)
			s3a_binary = &pipe->old_pipe->pipe.capture.primary_binary;
		else if (pipe->old_pipe->capture_mode == IA_CSS_CAPTURE_MODE_ADVANCED ||
			 pipe->old_pipe->capture_mode == IA_CSS_CAPTURE_MODE_LOW_LIGHT ||
			 pipe->old_pipe->capture_mode == IA_CSS_CAPTURE_MODE_BAYER) {
			if (pipe->extra_config.isp_pipe_version == 1) {
				s3a_binary 
				= &pipe->old_pipe->pipe.capture.pre_isp_binary;
			} else {
				s3a_binary
				= &pipe->old_pipe->pipe.capture.pre_anr_binary;
			}
			}
		break;
	default:
		break;
	}

	if (s3a_binary && s3a_binary->info->enable.s3a)
		return s3a_binary;

	return NULL;
}

struct sh_css_pipeline *
ia_css_pipe_get_pipeline(const struct ia_css_pipe *pipe)
{
	return &pipe->old_pipe->pipeline;
}

unsigned int
ia_css_pipe_get_pipe_num(const struct ia_css_pipe *pipe)
{
	return pipe->pipe_num;
}

enum ia_css_err
ia_css_start_sp(void)
{
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_start_sp() enter\n");
	sh_css_sp_start_isp();

	/* waiting for the SP is completely started */
	while (!ia_css_sp_has_initialized())
		hrt_sleep();

	//ia_css_stream_manager_create_id(&pipe->pipeline ,&stream_id);

	//ia_css_stream_manager_create(&pipe->pipeline ,stream_id);

	/* Workaround, in order to run two streams in parallel. See TASK 4271*/
	/* TODO: Fix this. */

	sh_css_init_host_sp_control_vars();

	/* buffers should be initialized only when sp is started */
	/* AM: At the moment it will be done only when there is no stream active. */
	sh_css_init_buffer_queues();
	sh_css_dtrace(SH_DBG_TRACE, "ia_css_start_sp() exit\n");
	return IA_CSS_SUCCESS;
}

/**
 *	Time to wait SP for termincate. Only condition when this can happen
 *	is a fatal hw failure, but we must be able to detect this and emit
 *	a proper error trace.
 */
#define SP_SHUTDOWN_TIMEOUT_US 200000

enum ia_css_err
ia_css_stop_sp(void)
{
	unsigned int i;
	unsigned long timeout;

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_stop_sp() enter\n");
	/* For now, stop whole SP */
	sh_css_write_host2sp_command(host2sp_cmd_terminate);
	sh_css_sp_set_sp_running(false);
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 1\n");
#endif
	timeout = SP_SHUTDOWN_TIMEOUT_US;
	while (!ia_css_sp_has_terminated() && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		sh_css_dump_debug_info("sh_css_stop_sp point1");
		sh_css_dump_sp_sw_debug_info();
#ifdef __KERNEL__
		printk(KERN_ERR "%s poll timeout point 1!!!\n", __func__);
#endif
	}
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 2\n");
#endif
	while (!isp_ctrl_getbit(ISP0_ID, ISP_SC_REG, ISP_IDLE_BIT) && timeout) {
		timeout--;
		hrt_sleep();
	}
	if (timeout == 0) {
		sh_css_dump_debug_info("sh_css_stop_sp point2");
		sh_css_dump_sp_sw_debug_info();
#ifdef __KERNEL__
		printk(KERN_ERR "%s poll timeout point 2!!!\n", __func__);
#endif
	}
#ifdef __KERNEL__
	printk("STOP_FUNC: reach point 3\n");
#endif

	for (i = 0; i < MAX_HMM_BUFFER_NUM; i++) {
		if (hmm_buffer_record_h[i] != NULL) {
			ia_css_i_host_rmgr_rel_vbuf(hmm_buffer_pool, &hmm_buffer_record_h[i]);
		}
	}

	/* clear pending param sets from refcount */
	sh_css_param_clear_param_sets();

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_stop_sp() exit\n");
	return IA_CSS_SUCCESS;
}
