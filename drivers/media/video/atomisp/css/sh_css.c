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
#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#include "sh_css.h"
#include "sh_css_hrt.h"
#include "sh_css_binary.h"
#include "sh_css_internal.h"
#include "sh_css_sp.h"
#include "sh_css_hw.h"
#include "sh_css_rx.h"
#include "sh_css_defs.h"
#include "sh_css_firmware.h"
#include "sh_css_binary_info.h"
#include "sh_css_accelerate.h"

#define WITH_PC_MONITORING  0

#if WITH_PC_MONITORING
#define MULTIPLE_SAMPLES 1
#define NOF_SAMPLES      60
#include "linux/kthread.h"
#include "linux/sched.h"
#include "linux/delay.h"
#include "sh_css_metrics.h"
#endif

/* Name of the sp program: should not be built-in */
#define SP_PROG_NAME "sp"

/* for JPEG, we don't know the length of the image upfront,
 * but since we support sensor upto 16MP, we take this as
 * upper limit.
 */
#define JPEG_BYTES (16 * 1024 * 1024)

#define NUM_REF_FRAMES         2
#define NUM_CONTINUOUS_FRAMES  2
#define NUM_TNR_FRAMES         2

#define IS_ODD(a)              ((a) & 0x1)

#define DEFAULT_FRAME_INFO \
{ \
	.width           = 0, \
	.height          = 0, \
	.format          = SH_CSS_FRAME_FORMAT_YUV420, \
	.raw_bit_depth   = 0, \
	.raw_bayer_order = sh_css_bayer_order_grbg, \
}

#define DEFAULT_IF_CONFIG \
{ \
	.start_line      = 0, \
	.start_column    = 0, \
	.left_padding    = 0, \
	.cropped_height  = 0, \
	.cropped_width   = 0, \
	.deinterleaving  = 0, \
	.buf_vecs        = 0, \
	.buf_start_index = 0, \
	.buf_increment   = 0, \
	.buf_eol_offset  = 0, \
	.yuv420          = false, \
	.block_no_reqs   = false, \
}

enum sh_css_mode {
	sh_css_mode_preview,
	sh_css_mode_video,
	sh_css_mode_capture
};

enum sh_css_state {
	sh_css_state_idle,
	sh_css_state_executing_isp,
	sh_css_state_executing_sp_bin_copy,
};

struct sh_css_pipeline_stage {
	struct sh_css_binary        *binary;      /* built-in binary */
	struct sh_css_binary_args    args;
	const struct sh_css_acc_fw  *firmware; /* acceleration binary */
	int                          mode;
	bool                         out_frame_allocated;
	bool                         vf_frame_allocated;
	struct sh_css_pipeline_stage *next;
};

struct sh_css_pipeline {
	unsigned int length;
	struct sh_css_pipeline_stage *stages;
	bool reload;
	struct sh_css_pipeline_stage *current_stage;
};

struct sh_css_preview_settings {
	struct sh_css_frame_info output_info;
	/* resolution before yuv downscaling */
	struct sh_css_frame_info pp_input_info;
	struct sh_css_pipeline pipeline;
	struct sh_css_binary copy_binary;
	struct sh_css_binary preview_binary;
	struct sh_css_binary vf_pp_binary;
	struct sh_css_frame *ref_frames[NUM_REF_FRAMES];
	unsigned int prev_ref_frame;
	struct sh_css_frame *continuous_frames[NUM_CONTINUOUS_FRAMES];
	unsigned int continuous_frame;
	bool online;
	struct sh_css_shading_table *shading_table;
	bool zoom_changed;
};

#define DEFAULT_PREVIEW_SETTINGS \
{ \
	.output_info       = DEFAULT_FRAME_INFO, \
	.pp_input_info     = DEFAULT_FRAME_INFO, \
	.online            = true, \
}

struct sh_css_capture_settings {
	enum sh_css_capture_mode mode;
	bool xnr;
	bool bayer_ds;
	struct sh_css_binary copy_binary;
	struct sh_css_binary primary_binary;
	struct sh_css_binary pre_isp_binary;
	struct sh_css_binary gdc_binary;
	struct sh_css_binary post_isp_binary;
	struct sh_css_binary anr_binary;
	struct sh_css_binary capture_pp_binary;
	struct sh_css_binary vf_pp_binary;
	/* resolution before yuv downscaling */
	struct sh_css_frame_info pp_input_info;
	struct sh_css_frame_info output_info;
	struct sh_css_frame_info vf_info;
	struct sh_css_pipeline pipeline;
	struct sh_css_frame *capture_pp_frame;
	struct sh_css_frame *output_frame;
	bool online;
	bool need_pp;
	struct sh_css_shading_table *shading_table;
	bool zoom_changed;
	bool anr;
};

#define DEFAULT_CAPTURE_SETTINGS \
{ \
	.mode              = SH_CSS_CAPTURE_MODE_PRIMARY, \
	.xnr               = false, \
	.bayer_ds          = false, \
	.pp_input_info     = DEFAULT_FRAME_INFO, \
	.output_info       = DEFAULT_FRAME_INFO, \
	.vf_info           = DEFAULT_FRAME_INFO, \
	.online            = true, \
	.need_pp           = false, \
}

#define NUM_VIDEO_REF_FRAMES 2
#define NUM_VIDEO_TNR_FRAMES 2

struct sh_css_video_settings {
	struct sh_css_binary video_binary;
	struct sh_css_binary vf_pp_binary;
	struct sh_css_frame_info output_info;
	struct sh_css_frame_info vf_info;
	struct sh_css_frame *ref_frames[NUM_VIDEO_REF_FRAMES];
	unsigned int         prev_ref_frame;
	struct sh_css_frame *tnr_frames[NUM_VIDEO_TNR_FRAMES];
	unsigned int         prev_tnr_frame;
	int dvs_vector_x;
	int dvs_vector_y;
	unsigned int dvs_envelope_width;
	unsigned int dvs_envelope_height;
	struct sh_css_frame *vf_pp_in_frame;
	struct sh_css_pipeline pipeline;
	struct sh_css_shading_table *shading_table;
	bool zoom_changed;
	bool invalid_first_frame;
};

#define DEFAULT_VIDEO_SETTINGS \
{ \
	.output_info         = DEFAULT_FRAME_INFO, \
	.vf_info             = DEFAULT_FRAME_INFO, \
	.invalid_first_frame = true,               \
}

struct sh_css {
	struct sh_css_preview_settings preview_settings;
	struct sh_css_capture_settings capture_settings;
	struct sh_css_video_settings   video_settings;
	unsigned int                   ch_id;
	unsigned int                   input_width;
	unsigned int                   input_height;
	struct sh_css_frame_info       input_effective_info;
	enum sh_css_input_format       input_format;
	enum sh_css_mode               mode;
	struct sh_css_mipi_config      mipi_config;
	bool                           reconfigure_css_rx;
	bool                           invalidate;
	/* Register functions that are only provided by the OS */
	void *(*malloc) (size_t size);
	void (*free) (void *ptr);
	void (*flush) (struct sh_css_acc_fw *fw);
	enum sh_css_state              state;
	bool                           two_ppc;
	enum sh_css_bayer_order        bayer_order;
	unsigned int                   sensor_binning;
	bool                           irq_edge;
	const struct sh_css_overlay   *vf_overlay;
	bool                           vf_overlay_changed;
	struct sh_css_if_config        curr_if_a_config;
	struct sh_css_if_config        curr_if_b_config;
	unsigned int                   curr_fmt_type;
	unsigned int                   curr_ch_id;
	enum sh_css_input_mode         curr_input_mode;
	enum sh_css_input_mode         input_mode;
	bool                           check_system_idle;
	unsigned int                   curr_dx;
	unsigned int                   curr_dy;
	bool                           continuous;
	bool                           start_sp_copy;
	bool                           disable_vf_pp;
	bool                           disable_capture_pp;
	const struct sh_css_shading_table *shading_table;
	void                          *sp_bin_addr;
	struct sh_css_acc_fw	      *output_stage;	/* extra output stage */
	struct sh_css_acc_fw	      *vf_stage;	/* extra vf stage */
	struct sh_css_acc_fw	      *standalone_acc;	/* standalone accel */
	void                          *page_table_base_address;
	bool capture_zoom_update;
	bool preview_zoom_update;
	bool video_zoom_update;
};

#define DEFAULT_MIPI_CONFIG \
{ \
	.port            = SH_CSS_MIPI_PORT_1LANE, \
	.num_lanes       = 1, \
	.timeout         = 0xffff4, \
	.comp_bpp        = 0, \
	.uncomp_bpp      = 0, \
	.comp            = SH_CSS_MIPI_COMPRESSION_NONE, \
	.two_ppc         = false, \
}

#define DEFAULT_CSS \
{ \
	.preview_settings     = DEFAULT_PREVIEW_SETTINGS, \
	.capture_settings     = DEFAULT_CAPTURE_SETTINGS, \
	.video_settings       = DEFAULT_VIDEO_SETTINGS, \
	.ch_id                = 0, \
	.input_width          = 0, \
	.input_height         = 0, \
	.input_effective_info = DEFAULT_FRAME_INFO, \
	.input_format         = SH_CSS_INPUT_FORMAT_RAW_10, \
	.mode                 = -1, \
	.mipi_config          = DEFAULT_MIPI_CONFIG, \
	.reconfigure_css_rx   = true, \
	.invalidate           = false, \
	.malloc               = NULL, \
	.free                 = NULL, \
	.flush                = NULL, \
	.state                = sh_css_state_idle, \
	.two_ppc              = false, \
	.bayer_order          = sh_css_bayer_order_grbg, \
	.sensor_binning       = 0, \
	.vf_overlay           = NULL, \
	.vf_overlay_changed   = false, \
	.curr_if_a_config     = DEFAULT_IF_CONFIG, \
	.curr_if_b_config     = DEFAULT_IF_CONFIG, \
	.curr_fmt_type        = -1, /* trigger first configuration */ \
	.curr_ch_id           = 0, \
	.curr_input_mode      = SH_CSS_INPUT_MODE_SENSOR, \
	.input_mode           = SH_CSS_INPUT_MODE_SENSOR, \
	.check_system_idle    = true, \
	.curr_dx              = UDS_SCALING_N, \
	.curr_dy              = UDS_SCALING_N, \
	.continuous           = false, \
	.start_sp_copy        = false, \
	.disable_vf_pp        = false, \
	.disable_capture_pp   = false, \
	.shading_table        = NULL, \
	.sp_bin_addr          = NULL, \
	.page_table_base_address = NULL, \
	.capture_zoom_update = false, \
	.preview_zoom_update = false, \
	.video_zoom_update = false, \
	.output_stage	      = NULL, \
	.vf_stage	      = NULL, \
	.standalone_acc	      = NULL, \
}

int (*sh_css_printf) (const char *fmt, ...) = NULL;

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
#ifndef SYSTEM_hive_isp_css_2400_system
				  pre_anr_descr,
				  anr_descr,
				  post_anr_descr,
#endif
				  video_descr,
				  capture_pp_descr;

static void reset_mode_shading_tables(void)
{
	if (my_css.preview_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.preview_settings.shading_table);
		my_css.preview_settings.shading_table = NULL;
	}

	if (my_css.capture_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.capture_settings.shading_table);
		my_css.capture_settings.shading_table = NULL;
	}

	if (my_css.video_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.video_settings.shading_table);
		my_css.video_settings.shading_table = NULL;
	}
}

static enum sh_css_err
check_frame_info(struct sh_css_frame_info *info)
{
	if (info->width == 0 || info->height == 0)
		return sh_css_err_illegal_resolution;
	return sh_css_success;
}

static enum sh_css_err
check_vf_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;
	err = check_frame_info(info);
	if (err != sh_css_success)
		return err;
	if (info->width > ISP_VF_PP_MAX_OUTPUT_WIDTH*2)
		return sh_css_err_viewfinder_resolution_too_wide;
	return sh_css_success;
}

static enum sh_css_err
check_vf_out_info(struct sh_css_frame_info *out_info,
		  struct sh_css_frame_info *vf_info)
{
	enum sh_css_err err;
	err = check_frame_info(out_info);
	if (err != sh_css_success)
		return err;
	err = check_vf_info(vf_info);
	if (err != sh_css_success)
		return err;
	if (vf_info->width > out_info->width ||
	    vf_info->height > out_info->height)
		return sh_css_err_viewfinder_resolution_exceeds_output;
	return sh_css_success;
}

static enum sh_css_err
check_res(unsigned int width, unsigned int height)
{
	if (width  == 0   ||
	    height == 0   ||
	    IS_ODD(width) ||
	    IS_ODD(height)) {
		return sh_css_err_illegal_resolution;
	}
	return sh_css_success;
}

static enum sh_css_err
check_null_res(unsigned int width, unsigned int height)
{
	if (IS_ODD(width) || IS_ODD(height))
		return sh_css_err_illegal_resolution;

	return sh_css_success;
}

static bool
input_format_is_raw(enum sh_css_input_format format)
{
	return format == SH_CSS_INPUT_FORMAT_RAW_6 ||
	    format == SH_CSS_INPUT_FORMAT_RAW_7 ||
	    format == SH_CSS_INPUT_FORMAT_RAW_8 ||
	    format == SH_CSS_INPUT_FORMAT_RAW_10 ||
	    format == SH_CSS_INPUT_FORMAT_RAW_12;
	/* raw_14 and raw_16 are not supported as input formats to the ISP.
	 * They can only be copied to a frame in memory using the
	 * copy binary.
	 */
}

static bool
input_format_is_yuv(enum sh_css_input_format format)
{
	return format == SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY ||
	    format == SH_CSS_INPUT_FORMAT_YUV420_8 ||
	    format == SH_CSS_INPUT_FORMAT_YUV420_10 ||
	    format == SH_CSS_INPUT_FORMAT_YUV422_8 ||
	    format == SH_CSS_INPUT_FORMAT_YUV422_10;
}

static enum sh_css_err
check_input(bool must_be_raw)
{
	if (my_css.input_effective_info.width == 0 ||
	    my_css.input_effective_info.height == 0) {
		return sh_css_err_effective_input_resolution_not_set;
	}
	if (must_be_raw &&
	    !input_format_is_raw(my_css.input_format)) {
		return sh_css_err_unsupported_input_format;
	}
	return sh_css_success;
}

/* we don't compare the height here since the output frame is usually
   a couple of lines bigger than the height of the binary info.
   For the padded width however, we do check equility because this is
   not expected to differ. A difference there would indicate an erroneous
   situation. */
static enum sh_css_err
check_infos_match(struct sh_css_frame_info *frame_info,
		  struct sh_css_frame_info *binary_info)
{
	if (frame_info->padded_width != binary_info->padded_width ||
	    frame_info->height       <  binary_info->height       ||
	    frame_info->format       != binary_info->format) {
		return sh_css_err_frames_mismatch;
	}
	return sh_css_success;
}

/* Input network configuration functions */
static void
get_copy_out_frame_format(enum sh_css_frame_format *format)
{
	switch (my_css.input_format) {
	case SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY:
	case SH_CSS_INPUT_FORMAT_YUV420_8:
		*format = SH_CSS_FRAME_FORMAT_YUV420;
		break;
	case SH_CSS_INPUT_FORMAT_YUV420_10:
		*format = SH_CSS_FRAME_FORMAT_YUV420_16;
		break;
	case SH_CSS_INPUT_FORMAT_YUV422_8:
		*format = SH_CSS_FRAME_FORMAT_YUV422;
		break;
	case SH_CSS_INPUT_FORMAT_YUV422_10:
		*format = SH_CSS_FRAME_FORMAT_YUV422_16;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_444:
	case SH_CSS_INPUT_FORMAT_RGB_555:
	case SH_CSS_INPUT_FORMAT_RGB_565:
		if (*format != SH_CSS_FRAME_FORMAT_RGBA888)
			*format = SH_CSS_FRAME_FORMAT_RGB565;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_666:
	case SH_CSS_INPUT_FORMAT_RGB_888:
		*format = SH_CSS_FRAME_FORMAT_RGBA888;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_6:
	case SH_CSS_INPUT_FORMAT_RAW_7:
	case SH_CSS_INPUT_FORMAT_RAW_8:
	case SH_CSS_INPUT_FORMAT_RAW_10:
	case SH_CSS_INPUT_FORMAT_RAW_12:
	case SH_CSS_INPUT_FORMAT_RAW_14:
	case SH_CSS_INPUT_FORMAT_RAW_16:
		if (*format != SH_CSS_FRAME_FORMAT_RAW)
			*format = SH_CSS_FRAME_FORMAT_RAW;
		break;
	case SH_CSS_INPUT_FORMAT_BINARY_8:
		*format = SH_CSS_FRAME_FORMAT_BINARY_8;
		break;
	}
}

unsigned int
sh_css_input_format_bits_per_pixel(enum sh_css_input_format format,
				   bool two_ppc)
{
	switch (format) {
	case SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY:
	case SH_CSS_INPUT_FORMAT_YUV420_8:
	case SH_CSS_INPUT_FORMAT_YUV422_8:
	case SH_CSS_INPUT_FORMAT_RGB_888:
	case SH_CSS_INPUT_FORMAT_RAW_8:
	case SH_CSS_INPUT_FORMAT_BINARY_8:
		return 8;
	case SH_CSS_INPUT_FORMAT_YUV420_10:
	case SH_CSS_INPUT_FORMAT_YUV422_10:
	case SH_CSS_INPUT_FORMAT_RAW_10:
		return 10;
	case SH_CSS_INPUT_FORMAT_RGB_444:
		return 4;
	case SH_CSS_INPUT_FORMAT_RGB_555:
		return 5;
	case SH_CSS_INPUT_FORMAT_RGB_565:
		return 565;
	case SH_CSS_INPUT_FORMAT_RGB_666:
	case SH_CSS_INPUT_FORMAT_RAW_6:
		return 6;
	case SH_CSS_INPUT_FORMAT_RAW_7:
		return 7;
	case SH_CSS_INPUT_FORMAT_RAW_12:
		return 12;
	case SH_CSS_INPUT_FORMAT_RAW_14:
		if (two_ppc)
			return 14;
		else
			return 12;
	case SH_CSS_INPUT_FORMAT_RAW_16:
		if (two_ppc)
			return 16;
		else
			return 12;
	}
	return 0;
}

/* compute the log2 of the downscale factor needed to get closest
 * to the requested viewfinder resolution on the upper side. The output cannot
 * be smaller than the requested viewfinder resolution.
 */
enum sh_css_err
sh_css_vf_downscale_log2(const struct sh_css_frame_info *out_info,
			 const struct sh_css_frame_info *vf_info,
			 unsigned int *downscale_log2)
{
	unsigned int ds_log2 = 0, out_width = out_info->padded_width;

	if (out_width == 0)
		return 0;

	/* downscale until width smaller than the viewfinder width. We don't
	 * test for the height since the vmem buffers only put restrictions on
	 * the width of a line, not on the number of lines in a frame.
	 */
	while (out_width >= vf_info->width) {
		ds_log2++;
		out_width /= 2;
	}
	/* now width is smaller, so we go up one step */
	if ((ds_log2 > 0) && (out_width < ISP_VF_PP_MAX_OUTPUT_WIDTH))
		ds_log2--;
	/* TODO: use actual max input resolution of vf_pp binary */
	if ((out_info->width >> ds_log2) >= 2*ISP_VF_PP_MAX_OUTPUT_WIDTH)
		return sh_css_err_viewfinder_resolution_too_wide;
	*downscale_log2 = ds_log2;
	return sh_css_success;
}

/* ISP expects GRBG bayer order, we skip one line and/or one row
 * to correct in case the input bayer order is different.
 */
static unsigned int
lines_needed_for_bayer_order(void)
{
	if (my_css.bayer_order == sh_css_bayer_order_bggr ||
	    my_css.bayer_order == sh_css_bayer_order_gbrg) {
		return 1;
	}
	return 0;
}

static unsigned int
columns_needed_for_bayer_order(void)
{
	if (my_css.bayer_order == sh_css_bayer_order_rggb ||
	    my_css.bayer_order == sh_css_bayer_order_gbrg) {
		return 1;
	}
	return 0;
}

static enum sh_css_err
input_start_column(unsigned int bin_in,
		   unsigned int *start_column)
{
	unsigned int in = my_css.input_width,
	    for_bayer = columns_needed_for_bayer_order(), start;

	if (bin_in + 2 * for_bayer > in)
		return sh_css_err_not_enough_input_columns;

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
	return sh_css_success;
}

static enum sh_css_err
input_start_line(unsigned int bin_in, unsigned int *start_line)
{
	unsigned int in = my_css.input_height,
	    for_bayer = lines_needed_for_bayer_order(), start;

	if (bin_in + 2 * for_bayer > in)
		return sh_css_err_not_enough_input_lines;

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
	return sh_css_success;
}

static enum sh_css_err
program_input_formatter(struct sh_css_binary *binary)
{
	unsigned int start_line, start_column = 0,
		     cropped_height = binary->in_frame_info.height,
		     cropped_width  = binary->in_frame_info.width,
		     left_padding = binary->left_padding,
		     num_vectors,
		     buffer_height = 2,
		     buffer_width = binary->info->max_input_width,
		     two_ppc = my_css.two_ppc,
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
		     start_column_b;
	struct sh_css_if_config if_a_config, if_b_config;
	enum sh_css_input_format input_format = binary->input_format;
	enum sh_css_err err = sh_css_success;

	/* TODO: check to see if input is RAW and if current mode interprets
	 * RAW data in any particular bayer order. copy binary with output
	 * format other than raw should not result in dropping lines and/or
	 * columns.
	 */
	err = input_start_line(cropped_height, &start_line);
	if (err != sh_css_success)
		return err;

	err = input_start_column(cropped_width, &start_column);
	if (err != sh_css_success)
		return err;

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

	bits_per_pixel = sh_css_hrt_if_prim_vec_align()*8 / ISP_VEC_NELEMS;
	switch (input_format) {
	case SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY:
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
			line_width =
			    vectors_per_line * sh_css_hrt_if_prim_vec_align() /
			    2;
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
	case SH_CSS_INPUT_FORMAT_YUV420_8:
	case SH_CSS_INPUT_FORMAT_YUV420_10:
		if (two_ppc) {
			vmem_increment = 1;
			deinterleaving = 1;
			width_a = width_b = cropped_width * deinterleaving / 2;
			buffer_width *= deinterleaving * 2;
			num_vectors *= deinterleaving;
			buf_offset_b = buffer_width / 2 / ISP_VEC_NELEMS;
			vectors_per_line = num_vectors / buffer_height;
			/* Even lines are half size */
			line_width =
			    vectors_per_line * sh_css_hrt_if_prim_vec_align() /
			    2;
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
	case SH_CSS_INPUT_FORMAT_YUV422_8:
	case SH_CSS_INPUT_FORMAT_YUV422_10:
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
	case SH_CSS_INPUT_FORMAT_RGB_444:
	case SH_CSS_INPUT_FORMAT_RGB_555:
	case SH_CSS_INPUT_FORMAT_RGB_565:
	case SH_CSS_INPUT_FORMAT_RGB_666:
	case SH_CSS_INPUT_FORMAT_RGB_888:
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
	case SH_CSS_INPUT_FORMAT_RAW_6:
	case SH_CSS_INPUT_FORMAT_RAW_7:
	case SH_CSS_INPUT_FORMAT_RAW_8:
	case SH_CSS_INPUT_FORMAT_RAW_10:
	case SH_CSS_INPUT_FORMAT_RAW_12:
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
			if (my_css.continuous &&
			    binary->info->mode == SH_CSS_BINARY_MODE_COPY) {
				/* No deinterleaving for sp copy */
				deinterleaving = 1;
			}
			width_a = cropped_width;
			/* Must be multiple of deinterleaving */
			num_vectors = CEIL_MUL(num_vectors, deinterleaving);
		}
		buffer_height *= 2;
		vectors_per_line = CEIL_DIV(cropped_width, ISP_VEC_NELEMS);
		vectors_per_line = CEIL_MUL(vectors_per_line, deinterleaving);
		break;
	case SH_CSS_INPUT_FORMAT_RAW_14:
	case SH_CSS_INPUT_FORMAT_RAW_16:
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
	case SH_CSS_INPUT_FORMAT_BINARY_8:
		break;
	}
	if (width_a == 0)
		return sh_css_err_unsupported_input_mode;

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
		line_width = vectors_per_line * sh_css_hrt_if_prim_vec_align();
	if (!buffers_per_line)
		buffers_per_line = deinterleaving;
	line_width = CEIL_MUL(line_width,
			      sh_css_hrt_if_prim_vec_align()*vmem_increment);

	vectors_per_buffer = buffer_height * buffer_width / ISP_VEC_NELEMS;

	if (my_css.input_mode == SH_CSS_INPUT_MODE_TPG &&
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
	    buffer_width * bits_per_pixel/8 - line_width;
	if_a_config.yuv420 = input_format == SH_CSS_INPUT_FORMAT_YUV420_8 ||
			     input_format == SH_CSS_INPUT_FORMAT_YUV420_10;
	if_a_config.block_no_reqs =
		my_css.input_mode != SH_CSS_INPUT_MODE_SENSOR;

	if (two_ppc) {
		if (deinterleaving_b) {
			deinterleaving = deinterleaving_b;
			width_b = cropped_width * deinterleaving;
			buffer_width *= deinterleaving;
			/* Patch from bayer to rgb */
			num_vectors =
			    num_vectors / 2 * deinterleaving * width_b_factor;
			vectors_per_line = num_vectors / buffer_height;
			line_width =
			    vectors_per_line * sh_css_hrt_if_prim_vec_align();
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
		if_b_config.yuv420 =
		    input_format == SH_CSS_INPUT_FORMAT_YUV420_8
		    || input_format == SH_CSS_INPUT_FORMAT_YUV420_10;
		if_b_config.block_no_reqs =
			my_css.input_mode != SH_CSS_INPUT_MODE_SENSOR;
		if (memcmp(&if_a_config, &my_css.curr_if_a_config,
			   sizeof(if_a_config)) ||
		    memcmp(&if_b_config, &my_css.curr_if_b_config,
			   sizeof(if_b_config))) {
			my_css.curr_if_a_config = if_a_config;
			my_css.curr_if_b_config = if_b_config;
			sh_css_sp_set_if_configs(&if_a_config, &if_b_config);
		}
	} else {
		if (memcmp(&if_a_config, &my_css.curr_if_a_config,
			   sizeof(if_a_config))) {
			my_css.curr_if_a_config = if_a_config;
			sh_css_sp_set_if_configs(&if_a_config, NULL);
		}
	}
	return sh_css_success;
}

static enum sh_css_err
sh_css_config_input_network(struct sh_css_pipeline *pipeline,
			    struct sh_css_binary *binary)
{
	unsigned int fmt_type;
	enum sh_css_err err = sh_css_success;

	if (pipeline && pipeline->stages)
		binary = pipeline->stages->binary;

	err = sh_css_input_format_type(my_css.input_format,
				       my_css.mipi_config.comp,
				       &fmt_type);
	if (err != sh_css_success)
		return err;
	if (fmt_type != my_css.curr_fmt_type ||
	    my_css.ch_id != my_css.curr_ch_id ||
	    my_css.input_mode != my_css.curr_input_mode) {
		my_css.curr_fmt_type = fmt_type;
		my_css.curr_ch_id = my_css.ch_id;
		my_css.curr_input_mode = my_css.input_mode;
		sh_css_sp_program_input_circuit(fmt_type,
						my_css.ch_id,
						my_css.input_mode);
	}

	if (binary && (binary->online || my_css.continuous)) {
		if (my_css.continuous)
			my_css.start_sp_copy = true;
		err = program_input_formatter(binary);
		if (err != sh_css_success)
			return err;
	}

	if (my_css.input_mode == SH_CSS_INPUT_MODE_TPG ||
	    my_css.input_mode == SH_CSS_INPUT_MODE_PRBS) {
		unsigned int hblank_cycles = 100,
			     vblank_lines = 6,
			     width,
			     height,
			     vblank_cycles;
		width  = my_css.input_width;
		height = my_css.input_height;
		vblank_cycles = vblank_lines * (width + hblank_cycles);
		sh_css_sp_configure_sync_gen(width, height, hblank_cycles,
					     vblank_cycles);
	}
	return sh_css_success;
}

#if WITH_PC_MONITORING
static struct task_struct *mon_thread;	/* Handle for the monitoring thread */
static int sh_binary_running;		/* Enable sampling in the thread */

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
		if (metrics->mode == SH_CSS_BINARY_ID_PREVIEW_DZ ||
		    metrics->mode == SH_CSS_BINARY_ID_VF_PP) {
			sh_css_print("pc_histogram for binary %d is SKIPPED\n",
				metrics->mode);
			continue;
		}

		sh_css_print(" pc_histogram for binary %d\n", metrics->mode);
		print_pc_histo("  ISP", &metrics->isp_histogram);
		print_pc_histo("  SP",   &metrics->sp_histogram);
		sh_css_print("print_pc_histogram() done for binay->id = %d, "
			     "done.\n", metrics->mode);
	}

	sh_css_print("PC_MONITORING:print_pc_histogram() -- DONE\n");
}

static int pc_monitoring(void *data)
{
	int i = 0;

	while (true) {
		if (kthread_should_stop())
			break;

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

static inline struct task_struct *spying_thread_create(void)
{
	struct task_struct *my_kthread;

	my_kthread = kthread_run(pc_monitoring, NULL, "sh_pc_monitor");
	sh_css_metrics_enable_pc_histogram(1);
	return my_kthread;
}

static inline int spying_thread_destroy(struct task_struct *my_kthread)
{
	sh_css_metrics_enable_pc_histogram(0);
	return kthread_stop(my_kthread);
}

static void input_frame_info(struct sh_css_frame_info frame_info)
{
	sh_css_print("SH_CSS:input_frame_info() -- frame->info.width = %d, "
	       "frame->info.height = %d, format = %d\n",
			frame_info.width, frame_info.height, frame_info.format);
}
#endif /* WITH_PC_MONITORING */

static enum sh_css_err
start_binary(struct sh_css_binary *binary,
	     struct sh_css_binary_args *args,
	     bool preview_mode)
{
	enum sh_css_err err = sh_css_success;

	if (my_css.reconfigure_css_rx)
		sh_css_rx_disable();

	sh_css_metrics_start_binary(&binary->metrics);

#if WITH_PC_MONITORING
	sh_css_print("PC_MONITORING: %s() -- binary id = %d , "
		     "enable_dvs_envelope = %d\n",
		     __func__, binary->info->id,
		     binary->info->enable_dvs_envelope);
	input_frame_info(binary->in_frame_info);

	if (binary->info->id == SH_CSS_BINARY_ID_VIDEO_DZ)
		sh_binary_running = 1;
#endif

	if (binary->info->mode == SH_CSS_BINARY_MODE_VF_PP &&
	    my_css.vf_overlay_changed) {
		sh_css_sp_set_overlay(my_css.vf_overlay);
		my_css.vf_overlay_changed = false;
	}

	if (my_css.continuous && binary->info->mode ==
	    SH_CSS_BINARY_MODE_PREVIEW) {
		sh_css_sp_start_raw_copy(binary, args->cc_frame,
					 my_css.two_ppc, false);
	}
	my_css.state = sh_css_state_executing_isp;
	err = sh_css_sp_start_isp(binary, args, preview_mode,
		my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_LOW_LIGHT);
	if (err != sh_css_success)
		return err;

	if (my_css.reconfigure_css_rx) {
		my_css.mipi_config.two_ppc = my_css.two_ppc;
		sh_css_rx_configure(&my_css.mipi_config);
		my_css.reconfigure_css_rx = false;
	}
	return sh_css_success;
}

static enum sh_css_err
start_firmware(const struct sh_css_acc_fw *fw,
	       struct sh_css_binary_args *args)
{
	enum sh_css_err err;
	struct sh_css_acc_fw *firmware = (struct sh_css_acc_fw *)fw;
	my_css.state = sh_css_state_executing_isp;
	err = sh_css_acc_start(firmware, args);
	return err;
}

static void
sh_css_frame_zero(struct sh_css_frame *frame)
{
	unsigned int i, words = frame->data_bytes / sizeof(int);
	int *ptr = frame->data;

	for (i = 0; i < words; i++)
		hrt_isp_css_mm_store_int(ptr + i, 0);
}

/* start the copy function on the SP */
static enum sh_css_err
start_copy_on_sp(struct sh_css_binary *binary,
		 struct sh_css_frame *out_frame)
{
	my_css.capture_settings.output_frame = out_frame;

	if (my_css.reconfigure_css_rx)
		sh_css_rx_disable();

#if defined(SYSTEM_hive_isp_css_2400_system)
	sh_css_hrt_irq_enable(hrt_isp_css_irq_sw_pin_0, true, false);
	sh_css_hrt_irq_enable(hrt_isp_css_irq_sw_pin_1, true, false);
#else  /* defined(SYSTEM_hive_isp_css_2400_system) */
	sh_css_hrt_irq_enable(hrt_isp_css_irq_sw_0, true, false);
	sh_css_hrt_irq_enable(hrt_isp_css_irq_sw_1, true, false);
	sh_css_hrt_irq_enable(hrt_isp_css_irq_sw_2, true, false);
#endif /* defined(SYSTEM_hive_isp_css_2400_system) */
	if (my_css.input_format == SH_CSS_INPUT_FORMAT_BINARY_8) {
		sh_css_sp_start_binary_copy(out_frame, my_css.two_ppc);
	} else {
		sh_css_sp_start_raw_copy(binary, out_frame,
					 my_css.two_ppc, true);
	}

	if (my_css.reconfigure_css_rx) {
		/* do we need to wait for the IF do be ready? */
		my_css.mipi_config.two_ppc = my_css.two_ppc;
		sh_css_rx_configure(&my_css.mipi_config);
		my_css.reconfigure_css_rx = false;
	}

	return sh_css_success;
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
	if (me->out_frame_allocated)
		sh_css_frame_free(me->args.out_frame);
	if (me->vf_frame_allocated)
		sh_css_frame_free(me->args.out_vf_frame);
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
	args->dvs_vector_x  = 0;
	args->dvs_vector_y  = 0;
	args->enable_xnr    = my_css.capture_settings.xnr;
	args->two_ppc       = my_css.two_ppc;
	args->copy_vf       = false;
	args->copy_output   = true;
	args->vf_downscale_log2 = 0;
}

static enum sh_css_err
sh_css_pipeline_stage_create(struct sh_css_pipeline_stage **me,
			     struct sh_css_binary *binary,
			     const struct sh_css_acc_fw *firmware,
			     int    mode,
			     struct sh_css_frame *cc_frame,
			     struct sh_css_frame *in_frame,
			     struct sh_css_frame *out_frame,
			     struct sh_css_frame *vf_frame)
{
	struct sh_css_pipeline_stage *stage = sh_css_malloc(sizeof(*stage));
	if (!stage)
		return sh_css_err_cannot_allocate_memory;
	stage->binary = firmware ? NULL : binary;
	stage->firmware = firmware;
	stage->mode = mode;
	stage->out_frame_allocated = false;
	stage->vf_frame_allocated = false;
	stage->next = NULL;
	sh_css_binary_args_reset(&stage->args);

	if (!in_frame && !firmware && !binary->online)
		return sh_css_err_internal_error;

	if (!out_frame && !firmware) {
		enum sh_css_err ret =
		    sh_css_frame_allocate_from_info(&out_frame,
						    &binary->out_frame_info);
		if (ret != sh_css_success) {
			sh_css_free(stage);
			return ret;
		}
		stage->out_frame_allocated = true;
	}
	/* VF frame is not needed in case of my_css.capture_settings.need_pp
	   However, the capture binary needs a vf frame to write to.
	*/
	if (!vf_frame) {
		if ((binary && binary->vf_frame_info.width) ||
		    (firmware && firmware->header.sp.out_vf)) {
			enum sh_css_err ret =
			    sh_css_frame_allocate_from_info(&vf_frame,
						    &binary->vf_frame_info);
			if (ret != sh_css_success) {
				if (stage->out_frame_allocated)
					sh_css_frame_free(out_frame);
				sh_css_free(stage);
				return ret;
			}
			stage->vf_frame_allocated = true;
		}
	} else if (vf_frame && binary && binary->vf_frame_info.width)
		stage->vf_frame_allocated = true;

	stage->args.cc_frame = cc_frame;
	stage->args.in_frame = in_frame;
	stage->args.out_frame = out_frame;
	stage->args.out_vf_frame = vf_frame;
	*me = stage;
	return sh_css_success;
}

static void
sh_css_pipeline_init(struct sh_css_pipeline *me)
{
	me->length = 0;
	me->stages = NULL;
	me->reload = true;
	me->current_stage = NULL;
}

static enum sh_css_err
sh_css_pipeline_add_stage(struct sh_css_pipeline *me,
			  struct sh_css_binary *binary,
			  const struct sh_css_acc_fw *firmware,
			  int    mode,
			  struct sh_css_frame *cc_frame,
			  struct sh_css_frame *in_frame,
			  struct sh_css_frame *out_frame,
			  struct sh_css_frame *vf_frame,
			  struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *last = me->stages, *new_stage = NULL;
	enum sh_css_err err;

	if (!binary && !firmware)
		return sh_css_err_internal_error;

	while (last && last->next)
		last = last->next;

	/* if in_frame is not set, we use the out_frame from the previous
	 * stage, if no previous stage, it's an error.
	 */
	if (!in_frame && !firmware && !binary->online) {
		if (last)
			in_frame = last->args.out_frame;
		if (!in_frame)
			return sh_css_err_internal_error;
	}
	err = sh_css_pipeline_stage_create(&new_stage, binary, firmware,
					   mode, cc_frame,
					   in_frame, out_frame, vf_frame);
	if (err != sh_css_success)
		return err;
	if (last)
		last->next = new_stage;
	else
		me->stages = new_stage;
	me->length++;
	if (stage)
		*stage = new_stage;
	return sh_css_success;
}

static enum sh_css_err
sh_css_pipeline_get_stage(struct sh_css_pipeline *me,
			  int mode,
			  struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *s;
	for (s = me->stages; s; s = s->next) {
		if (s->mode == mode) {
			*stage = s;
			return sh_css_success;
		}
	}
	return sh_css_err_internal_error;
}

static enum sh_css_err
sh_css_pipeline_get_output_stage(struct sh_css_pipeline *me,
				 int mode,
				 struct sh_css_pipeline_stage **stage)
{
	struct sh_css_pipeline_stage *s;

	*stage = NULL;
	/* First find acceleration firmware at end of pipe */
	for (s = me->stages; s; s = s->next) {
		if (s->firmware && s->mode == mode &&
		    s->firmware->header.sp.output)
			*stage = s;
	}
	if (*stage)
		return sh_css_success;
	/* If no firmware, find binary in pipe */
	return sh_css_pipeline_get_stage(me, mode, stage);
}

static void
sh_css_pipeline_restart(struct sh_css_pipeline *me)
{
	me->current_stage = NULL;
}

static void
sh_css_pipeline_clean(struct sh_css_pipeline *me)
{
	struct sh_css_pipeline_stage *s = me->stages;

	while (s) {
		struct sh_css_pipeline_stage *next = s->next;
		sh_css_pipeline_stage_destroy(s);
		s = next;
	}
	sh_css_pipeline_init(me);
}

static bool
sh_css_pipeline_done(struct sh_css_pipeline *me)
{
	return me->current_stage && !me->current_stage->next;
}

static struct sh_css_pipeline *
get_current_pipeline(void)
{
	if (my_css.mode == sh_css_mode_preview)
		return &my_css.preview_settings.pipeline;
	else if (my_css.mode == sh_css_mode_video)
		return &my_css.video_settings.pipeline;
	else
		return &my_css.capture_settings.pipeline;
}

void
sh_css_terminate_firmware(void)
{
	struct sh_css_pipeline *me = get_current_pipeline();
	struct sh_css_acc_fw *firmware;

	if (!me->current_stage)
		return;
	firmware = my_css.standalone_acc ? my_css.standalone_acc :
				(struct sh_css_acc_fw *)me->current_stage->firmware;
	if (!firmware)
		return;

	sh_css_acceleration_done(firmware);
	/* reload SP firmmware */
	my_css.sp_bin_addr = sh_css_sp_load_program(&sh_css_sp_fw,
						    SP_PROG_NAME,
						    my_css.sp_bin_addr, true);
#if 0
	/* restore SP state */
	if (my_css.two_ppc) {
		sh_css_sp_set_if_configs(&my_css.curr_if_a_config,
					 &my_css.curr_if_b_config);
	} else {
		sh_css_sp_set_if_configs(&my_css.curr_if_a_config, NULL);
	}
#endif
}

bool
sh_css_next_stage_is_acc(void)
{
	struct sh_css_pipeline_stage *stage;
	struct sh_css_pipeline *pipeline;

	if (my_css.mode == sh_css_mode_preview)
		pipeline = &my_css.preview_settings.pipeline;
	else if (my_css.mode == sh_css_mode_video)
		pipeline = &my_css.video_settings.pipeline;
	else
		pipeline = &my_css.capture_settings.pipeline;

	if (pipeline->current_stage)
		stage = pipeline->current_stage->next;
	else
		stage = pipeline->stages;

	return stage != NULL ? !!stage->firmware : false;
}

static enum sh_css_err
sh_css_pipeline_start_next_stage(struct sh_css_pipeline *me)
{
	struct sh_css_pipeline_stage *stage;
	enum sh_css_err err;

	if (me->current_stage)
		stage = me->current_stage->next;
	else
		stage = me->stages;
	if (!stage)
		return sh_css_success;

	me->current_stage = stage;

	if (stage->firmware)
		err = start_firmware(stage->firmware, &stage->args);
	else
		err = start_binary(stage->binary, &stage->args,
				   me == &my_css.preview_settings.pipeline);
	return err;
}

void
sh_css_frame_info_set_width(struct sh_css_frame_info *info,
			    unsigned int width)
{
	info->width = width;
	/* frames with a U and V plane of 8 bits per pixel need to have
	   all planes aligned, this means double the alignment for the
	   Y plane if the horizontal decimation is 2. */
	if (info->format == SH_CSS_FRAME_FORMAT_YUV420 ||
	    info->format == SH_CSS_FRAME_FORMAT_YV12)
		info->padded_width = CEIL_MUL(width, 2*HIVE_ISP_DDR_WORD_BYTES);
	else if (info->format == SH_CSS_FRAME_FORMAT_YUV_LINE)
		info->padded_width = CEIL_MUL(width, 2*ISP_VEC_NELEMS);
	else
		info->padded_width = CEIL_MUL(width, HIVE_ISP_DDR_WORD_BYTES);
}

static void
sh_css_frame_info_set_format(struct sh_css_frame_info *info,
			     enum sh_css_frame_format format)
{
	/* yuv_line has 2*NWAY alignment */
	info->format = format;
	/* HACK: this resets the padded width incorrectly.
	   Lex needs to fix this in the vf_veceven module. */
	info->padded_width =  CEIL_MUL(info->padded_width, 2*ISP_VEC_NELEMS);
}

static void
sh_css_frame_info_init(struct sh_css_frame_info *info,
		       unsigned int width,
		       unsigned int height,
		       enum sh_css_frame_format format)
{
	info->height = height;
	info->format = format;
	sh_css_frame_info_set_width(info, width);
}

static void
invalidate_video_binaries(void)
{
	my_css.video_settings.pipeline.reload   = true;
	my_css.video_settings.video_binary.info = NULL;
	my_css.video_settings.vf_pp_binary.info = NULL;
	if (my_css.video_settings.shading_table) {
		sh_css_shading_table_free(my_css.video_settings.shading_table);
		my_css.video_settings.shading_table = NULL;
	}
}

void
sh_css_overlay_set_for_viewfinder(const struct sh_css_overlay *overlay)
{
	my_css.vf_overlay = overlay;
	my_css.vf_overlay_changed = true;
}

void
sh_css_set_shading_table(const struct sh_css_shading_table *table)
{
	if (table != my_css.shading_table)
		reset_mode_shading_tables();

	my_css.shading_table = table;
}

/* CSS receiver programming */
enum sh_css_err
sh_css_input_configure_port(enum sh_css_mipi_port port,
			    unsigned int num_lanes,
			    unsigned int timeout)
{
	if (port == SH_CSS_MIPI_PORT_1LANE && num_lanes > 1)
		return sh_css_err_conflicting_mipi_settings;
	if (num_lanes > 4)
		return sh_css_err_conflicting_mipi_settings;
	my_css.mipi_config.port = port;
	my_css.mipi_config.num_lanes = num_lanes;
	my_css.mipi_config.timeout = timeout;
	my_css.reconfigure_css_rx = true;

	return sh_css_success;
}

enum sh_css_err
sh_css_input_set_compression(enum sh_css_mipi_compression comp,
			     unsigned int compressed_bits_per_pixel,
			     unsigned int uncompressed_bits_per_pixel)
{
	if (comp == SH_CSS_MIPI_COMPRESSION_NONE) {
		if (compressed_bits_per_pixel || uncompressed_bits_per_pixel)
			return sh_css_err_conflicting_mipi_settings;
	} else {
		if (compressed_bits_per_pixel < 6 ||
		    compressed_bits_per_pixel > 8)
			return sh_css_err_conflicting_mipi_settings;
		if (uncompressed_bits_per_pixel != 10 &&
		    uncompressed_bits_per_pixel != 12)
			return sh_css_err_conflicting_mipi_settings;
	}
	my_css.mipi_config.comp = comp;
	my_css.mipi_config.comp_bpp = compressed_bits_per_pixel;
	my_css.mipi_config.uncomp_bpp = uncompressed_bits_per_pixel;
	my_css.reconfigure_css_rx = true;
	return sh_css_success;
}

void
sh_css_tpg_configure(unsigned int x_mask,
		     int x_delta,
		     unsigned int y_mask,
		     int y_delta,
		     unsigned int xy_mask)
{
	sh_css_sp_configure_tpg(x_mask, y_mask, x_delta, y_delta, xy_mask);
}

void
sh_css_prbs_set_seed(int seed)
{
	sh_css_sp_configure_prbs(seed);
}

/* currently, the capture pp binary requires an internal frame. This will
   be removed in the future. */
static enum sh_css_err
alloc_capture_pp_frame(const struct sh_css_binary *binary)
{

	struct sh_css_frame_info cpp_info;
	enum sh_css_err err = sh_css_success;

	cpp_info = binary->internal_frame_info;
	cpp_info.format = SH_CSS_FRAME_FORMAT_YUV420;
	if (my_css.capture_settings.capture_pp_frame)
		sh_css_frame_free(my_css.capture_settings.capture_pp_frame);
	err = sh_css_frame_allocate_from_info(
			&my_css.capture_settings.capture_pp_frame, &cpp_info);
	return err;
}

static void
invalidate_preview_binaries(void)
{
	my_css.preview_settings.pipeline.reload     = true;
	my_css.preview_settings.preview_binary.info = NULL;
	my_css.preview_settings.vf_pp_binary.info   = NULL;
	my_css.preview_settings.copy_binary.info    = NULL;
	if (my_css.preview_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.preview_settings.shading_table);
		my_css.preview_settings.shading_table = NULL;
	}
}

static void
invalidate_capture_binaries(void)
{
	my_css.capture_settings.pipeline.reload        = true;
	my_css.capture_settings.copy_binary.info       = NULL;
	my_css.capture_settings.primary_binary.info    = NULL;
	my_css.capture_settings.pre_isp_binary.info    = NULL;
	my_css.capture_settings.gdc_binary.info        = NULL;
	my_css.capture_settings.post_isp_binary.info   = NULL;
	my_css.capture_settings.anr_binary.info        = NULL;
	my_css.capture_settings.capture_pp_binary.info = NULL;
	my_css.capture_settings.vf_pp_binary.info      = NULL;
	my_css.capture_settings.need_pp = false;
	if (my_css.capture_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.capture_settings.shading_table);
		my_css.capture_settings.shading_table = NULL;
	}
}

static void
enable_interrupts(void)
{
	/* Enable IRQ when SP goes to idle */
	sh_css_hrt_irq_enable_sp(true);
	/* Enabling this triggers an interrupt immediately, clear it */
	sh_css_hrt_irq_clear_sp();

	sh_css_hrt_irq_enable(hrt_isp_css_irq_sp, true, my_css.irq_edge);
	sh_css_hrt_irq_clear_all();
	sh_css_rx_enable_all_interrupts();
}

enum sh_css_err
sh_css_init(void *(*malloc_func) (size_t size),
	    void (*free_func) (void *ptr),
	    void (*flush_func) (struct sh_css_acc_fw *fw),
	    enum sh_css_interrupt_setting irq_setting,
	    const char *fw_data,
	    unsigned int fw_size)
{
	static struct sh_css default_css = DEFAULT_CSS;
	enum sh_css_err err;

	/* "flush()" for cache control of accelrator API
	 * (shared buffer pointer) arguments
	 */
	if (malloc_func == NULL || free_func == NULL || flush_func == NULL)
		return sh_css_err_invalid_arguments;

	memcpy(&my_css, &default_css, sizeof(my_css));

	my_css.malloc = malloc_func;
	my_css.free = free_func;
	my_css.flush = flush_func;
	my_css.irq_edge = (irq_setting == SH_CSS_INTERRUPT_SETTING_EDGE);

	/* In case this has been programmed already, update internal
	   data structure */
	my_css.page_table_base_address =
		sh_css_mmu_get_page_table_base_address();

	enable_interrupts();

	err = sh_css_params_init();
	if (err != sh_css_success)
		return err;
	err = sh_css_sp_init();
	if (err != sh_css_success)
		return err;
	err = sh_css_load_firmware(fw_data, fw_size);
	if (err != sh_css_success)
		return err;
	sh_css_init_binary_infos();
	my_css.sp_bin_addr = sh_css_sp_load_program(&sh_css_sp_fw,
						    SP_PROG_NAME,
						    my_css.sp_bin_addr, true);
	if (!my_css.sp_bin_addr)
		return sh_css_err_cannot_allocate_memory;
	sh_css_pipeline_init(&my_css.preview_settings.pipeline);
	sh_css_pipeline_init(&my_css.video_settings.pipeline);
	sh_css_pipeline_init(&my_css.capture_settings.pipeline);

#if WITH_PC_MONITORING
	if (!mon_thread) {
		sh_css_print("PC_MONITORING: %s() -- create thread DISABLED\n",
			     __func__);
		mon_thread = spying_thread_create();
	}
	sh_css_printf = printk;
#endif

	return err;
}

/* Suspend does not need to do anything for now, this may change
   in the future though. */
void
sh_css_suspend(void)
{
}

void
sh_css_resume(void)
{
	/* trigger reconfiguration of necessary hardware */
	my_css.reconfigure_css_rx = true;
	my_css.curr_if_a_config.cropped_width  = 0;
	my_css.curr_if_a_config.cropped_height = 0;
	my_css.curr_if_b_config.cropped_width  = 0;
	my_css.curr_if_b_config.cropped_height = 0;
	my_css.curr_fmt_type = -1;
	/* reload the SP binary. ISP binaries are automatically
	   reloaded by the ISP upon execution. */
	sh_css_hrt_mmu_set_page_table_base_address(
			my_css.page_table_base_address);
	sh_css_params_reconfigure_gdc_lut();

	sh_css_sp_activate_program(&sh_css_sp_fw, my_css.sp_bin_addr,
				   SP_PROG_NAME);

	enable_interrupts();
}

void *
sh_css_malloc(size_t size)
{
	if (size > 0 && my_css.malloc)
		return my_css.malloc(size);
	return NULL;
}

void
sh_css_free(void *ptr)
{
	if (ptr && my_css.free)
		my_css.free(ptr);
}

/* For Acceleration API: Flush FW (shared buffer pointer) arguments */
void
sh_css_flush(struct sh_css_acc_fw *fw)
{
	if ((fw != NULL) && (my_css.flush != NULL))
		my_css.flush(fw);
}

void
sh_css_set_print_function(int (*func) (const char *fmt, ...))
{
	sh_css_printf = func;
}

void
sh_css_uninit(void)
{
	int i;

#if WITH_PC_MONITORING
	if (mon_thread) {
		spying_thread_destroy(mon_thread);
		mon_thread = NULL;
	}
	sh_css_print("PC_MONITORING: %s() -- started\n", __func__);
	print_pc_histogram();
#endif

	/* cleanup generic data */
	sh_css_params_uninit();
	sh_css_binary_uninit();
	sh_css_sp_uninit();
	if (my_css.sp_bin_addr) {
		hrt_isp_css_mm_free(my_css.sp_bin_addr);
		my_css.sp_bin_addr = NULL;
	}

	/* cleanup preview data */
	invalidate_preview_binaries();
	sh_css_pipeline_clean(&my_css.preview_settings.pipeline);
	for (i = 0; i < NUM_REF_FRAMES; i++) {
		if (my_css.preview_settings.ref_frames[i]) {
			sh_css_frame_free(
					my_css.preview_settings.ref_frames[i]);
			my_css.preview_settings.ref_frames[i] = NULL;
		}
	}
	for (i = 0; i < NUM_CONTINUOUS_FRAMES; i++) {
		if (my_css.preview_settings.continuous_frames[i]) {
			sh_css_frame_free(
				my_css.preview_settings.continuous_frames[i]);
			my_css.preview_settings.continuous_frames[i] = NULL;
		}
	}

	/* cleanup video data */
	invalidate_video_binaries();
	sh_css_pipeline_clean(&my_css.video_settings.pipeline);
	for (i = 0; i < NUM_TNR_FRAMES; i++) {
		if (my_css.video_settings.tnr_frames[i])
			sh_css_frame_free(my_css.video_settings.tnr_frames[i]);
		my_css.video_settings.tnr_frames[i] = NULL;
	}
	for (i = 0; i < NUM_REF_FRAMES; i++) {
		if (my_css.video_settings.ref_frames[i])
			sh_css_frame_free(my_css.video_settings.ref_frames[i]);
		my_css.video_settings.ref_frames[i] = NULL;
	}

	/* cleanup capture data */
	invalidate_capture_binaries();
	sh_css_pipeline_clean(&my_css.capture_settings.pipeline);
	if (my_css.capture_settings.capture_pp_frame) {
		sh_css_frame_free(my_css.capture_settings.capture_pp_frame);
		my_css.capture_settings.capture_pp_frame = NULL;
	}
}

enum sh_css_err
sh_css_start_next_stage(void)
{
	struct sh_css_pipeline *pipeline = NULL;
	enum sh_css_err err = sh_css_success;

	/* LA: to be removed, now only first binary can run it */
	my_css.start_sp_copy = false;

	if (my_css.state != sh_css_state_idle)
		return sh_css_err_system_not_idle;

	if (my_css.check_system_idle && !sh_css_hrt_system_is_idle())
		return sh_css_err_system_not_idle;

	if (my_css.mode == sh_css_mode_preview)
		pipeline = &my_css.preview_settings.pipeline;
	else if (my_css.mode == sh_css_mode_video)
		pipeline = &my_css.video_settings.pipeline;
	else
		pipeline = &my_css.capture_settings.pipeline;
	err = sh_css_pipeline_start_next_stage(pipeline);
	return err;
}

static bool
sh_css_frame_done(void)
{
	bool done = true;
	if (my_css.state == sh_css_state_executing_isp)
		done = sh_css_pipeline_done(get_current_pipeline());
	else if (my_css.state == sh_css_state_executing_sp_bin_copy) {
		my_css.capture_settings.output_frame->planes.binary.size =
			sh_css_sp_get_binary_copy_size();
	}
	return done;
}

static bool
sh_css_statistics_ready(void)
{
	struct sh_css_pipeline *pipeline = NULL;

	if (my_css.state != sh_css_state_executing_isp)
		return false;

	pipeline = get_current_pipeline();
	if (pipeline->current_stage->binary)
		return pipeline->current_stage->binary->info->enable_s3a;
	return false;
}

static bool
acceleration_done(void)
{
	struct sh_css_pipeline *pipeline = get_current_pipeline();

	if (my_css.standalone_acc)
		return true;

	return pipeline &&
	       pipeline->current_stage &&
	       pipeline->current_stage->firmware;
}
/* MW_R1MRFLD : 2300 and 2400 have a different IRQ enumeration */
#if defined(SYSTEM_hive_isp_css_2400_system)

enum sh_css_err
sh_css_translate_interrupt(unsigned int *irq_infos)
{
	enum hrt_isp_css_irq irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = sh_css_hrt_irq_get_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return sh_css_err_interrupt_error;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
		case hrt_isp_css_irq_sp:
			sh_css_hrt_irq_clear_sp();
			if (sh_css_frame_done())
				infos |= SH_CSS_IRQ_INFO_FRAME_DONE;
			else
				infos |= SH_CSS_IRQ_INFO_START_NEXT_STAGE;
			if (sh_css_statistics_ready())
				infos |= SH_CSS_IRQ_INFO_STATISTICS_READY;
			if (acceleration_done())
				infos |= SH_CSS_IRQ_INFO_FW_ACC_DONE;
			my_css.state = sh_css_state_idle;
			break;
		case hrt_isp_css_irq_isp:
			/* nothing */
			break;
		case hrt_isp_css_irq_isys:
			infos |= SH_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR;
			break;
		case hrt_isp_css_irq_ifmt:
			infos |= SH_CSS_IRQ_INFO_IF_ERROR;
			break;
		case hrt_isp_css_irq_dma:
			infos |= SH_CSS_IRQ_INFO_DMA_ERROR;
			break;
		case hrt_isp_css_irq_sw_pin_0:
			infos |= SH_CSS_IRQ_INFO_SW_0;
			break;
		case hrt_isp_css_irq_sw_pin_1:
			infos |= SH_CSS_IRQ_INFO_SW_1;
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;
	return sh_css_success;
}

enum sh_css_err
sh_css_enable_interrupt(enum sh_css_interrupt_info info,
			bool enable)
{
	enum hrt_isp_css_irq irq = -1;
	bool edge = my_css.irq_edge;

	switch (info) {
	case SH_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR:
		irq = hrt_isp_css_irq_isys;
		break;
	case SH_CSS_IRQ_INFO_IF_ERROR:
		irq = hrt_isp_css_irq_ifmt;
		break;
	case SH_CSS_IRQ_INFO_DMA_ERROR:
		irq = hrt_isp_css_irq_dma;
		break;
	case SH_CSS_IRQ_INFO_SW_0:
		irq = hrt_isp_css_irq_sw_pin_0;
		edge = false;
		break;
	case SH_CSS_IRQ_INFO_SW_1:
		irq = hrt_isp_css_irq_sw_pin_1;
		edge = false;
		break;
	default:
		return sh_css_err_invalid_arguments;
	}

	if (enable)
		sh_css_hrt_irq_enable(irq, true, edge);
	else
		sh_css_hrt_irq_disable(irq);

	return sh_css_success;
}

#else  /* defined(SYSTEM_hive_isp_css_2400_system) */

enum sh_css_err
sh_css_translate_interrupt(unsigned int *irq_infos)
{
	enum hrt_isp_css_irq irq;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_more_irqs;
	unsigned int infos = 0;

	while (status == hrt_isp_css_irq_status_more_irqs) {
		status = sh_css_hrt_irq_get_id(&irq);
		if (status == hrt_isp_css_irq_status_error)
			return sh_css_err_interrupt_error;

#if WITH_PC_MONITORING
		sh_css_print("PC_MONITORING: %s() irq = %d, "
			     "sh_binary_running set to 0\n", __func__, irq);
		sh_binary_running = 0 ;
#endif

		switch (irq) {
		case hrt_isp_css_irq_sp:
			sh_css_hrt_irq_clear_sp();
			if (sh_css_frame_done()) {
				infos |= SH_CSS_IRQ_INFO_FRAME_DONE;
				if (my_css.mode == sh_css_mode_video &&
				    my_css.video_settings.invalid_first_frame) {
					infos |=
					  SH_CSS_IRQ_INFO_INVALID_FIRST_FRAME;
					my_css.video_settings.
						invalid_first_frame = false;
				}
			} else {
				infos |= SH_CSS_IRQ_INFO_START_NEXT_STAGE;
			}
			if (sh_css_statistics_ready()) {
				infos |= SH_CSS_IRQ_INFO_STATISTICS_READY;
				sh_css_params_swap_3a_buffers();
			}
			if (acceleration_done())
				infos |= SH_CSS_IRQ_INFO_FW_ACC_DONE;
			my_css.state = sh_css_state_idle;
			break;
		case hrt_isp_css_irq_isp:
			/* nothing */
			break;
		case hrt_isp_css_irq_mipi:
			/* css rx interrupt, read error bits from css rx */
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR;
			break;
		case hrt_isp_css_irq_mipi_fifo_full:
			infos |=
			    SH_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW;
			break;
		case hrt_isp_css_irq_mipi_sof:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF;
			break;
		case hrt_isp_css_irq_mipi_eof:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_EOF;
			break;
		case hrt_isp_css_irq_mipi_sol:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_SOL;
			break;
		case hrt_isp_css_irq_mipi_eol:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_EOL;
			break;
		case hrt_isp_css_irq_sideband_changed:
			infos |=
			    SH_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED;
			break;
		case hrt_isp_css_irq_css_gen_short_0:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0;
			break;
		case hrt_isp_css_irq_css_gen_short_1:
			infos |= SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1;
			break;
		case hrt_isp_css_irq_ift_prim:
			infos |= SH_CSS_IRQ_INFO_IF_PRIM_ERROR;
			break;
		case hrt_isp_css_irq_ift_prim_b:
			infos |= SH_CSS_IRQ_INFO_IF_PRIM_B_ERROR;
			break;
		case hrt_isp_css_irq_ift_sec:
			infos |= SH_CSS_IRQ_INFO_IF_SEC_ERROR;
			break;
		case hrt_isp_css_irq_ift_mem_cpy:
			infos |= SH_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR;
			break;
		case hrt_isp_css_irq_sw_0:
			infos |= SH_CSS_IRQ_INFO_SW_0;
			break;
		case hrt_isp_css_irq_sw_1:
			infos |= SH_CSS_IRQ_INFO_SW_1;
			break;
		case hrt_isp_css_irq_sw_2:
			infos |= SH_CSS_IRQ_INFO_SW_2;
			break;
		default:
			break;
		}
	}

	if (irq_infos)
		*irq_infos = infos;
	return sh_css_success;
}
#endif
void
sh_css_mmu_set_page_table_base_address(void *base_address)
{
	my_css.page_table_base_address = base_address;
	sh_css_hrt_mmu_set_page_table_base_address(base_address);
	sh_css_mmu_invalidate_cache();
}

void *
sh_css_mmu_get_page_table_base_address(void)
{
	return sh_css_hrt_mmu_get_page_table_base_address();
}

void
sh_css_mmu_invalidate_cache(void)
{
#ifdef SYSTEM_hive_isp_css_2400_system
	sh_css_hrt_mmu_invalidate_cache();
#else
	sh_css_sp_invalidate_mmu();
#endif
}

#ifndef SYSTEM_hive_isp_css_2400_system
enum sh_css_err
sh_css_enable_interrupt(enum sh_css_interrupt_info info,
			bool enable)
{
	enum hrt_isp_css_irq irq = -1;
	bool edge = my_css.irq_edge;

	switch (info) {
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR:
		irq = hrt_isp_css_irq_mipi;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW:
		irq = hrt_isp_css_irq_mipi_fifo_full;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF:
		irq = hrt_isp_css_irq_mipi_sof;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_EOF:
		irq = hrt_isp_css_irq_mipi_eof;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_SOL:
		irq = hrt_isp_css_irq_mipi_sol;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_EOL:
		irq = hrt_isp_css_irq_mipi_eol;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED:
		irq = hrt_isp_css_irq_sideband_changed;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0:
		irq = hrt_isp_css_irq_css_gen_short_0;
		break;
	case SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1:
		irq = hrt_isp_css_irq_css_gen_short_1;
		break;
	case SH_CSS_IRQ_INFO_IF_PRIM_ERROR:
		irq = hrt_isp_css_irq_ift_prim;
		break;
	case SH_CSS_IRQ_INFO_IF_PRIM_B_ERROR:
		irq = hrt_isp_css_irq_ift_prim_b;
		break;
	case SH_CSS_IRQ_INFO_IF_SEC_ERROR:
		irq = hrt_isp_css_irq_ift_sec;
		break;
	case SH_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR:
		irq = hrt_isp_css_irq_ift_mem_cpy;
		break;
	case SH_CSS_IRQ_INFO_SW_0:
		irq = hrt_isp_css_irq_sw_0;
		edge = false;
		break;
	case SH_CSS_IRQ_INFO_SW_1:
		irq = hrt_isp_css_irq_sw_1;
		edge = false;
		break;
	case SH_CSS_IRQ_INFO_SW_2:
		irq = hrt_isp_css_irq_sw_2;
		edge = false;
		break;
	default:
		return sh_css_err_invalid_arguments;
	}

	if (enable)
		sh_css_hrt_irq_enable(irq, true, edge);
	else
		sh_css_hrt_irq_disable(irq);

	return sh_css_success;
}
#endif
unsigned int
sh_css_get_sw_interrupt_value(void)
{
	return sh_css_sp_get_sw_interrupt_value();
}

enum sh_css_err
sh_css_wait_for_completion(void)
{
	enum sh_css_err err;
	unsigned int irq_infos = 0;

	while (true) {
		err = sh_css_hrt_sp_wait();
		if (err != sh_css_success)
			return err;
		/* just in case we're not using interrupts */
		sh_css_translate_interrupt(&irq_infos);
		if (irq_infos & SH_CSS_IRQ_INFO_START_NEXT_STAGE) {
			err = sh_css_start_next_stage();
			if (err != sh_css_success)
				return err;
		} else {
			break;
		}
	}
	return sh_css_success;
}

void
sh_css_uv_offset_is_zero(bool *uv_offset_is_zero)
{
	if (uv_offset_is_zero)
		*uv_offset_is_zero = SH_CSS_UV_OFFSET_IS_0;
}

enum sh_css_err
sh_css_input_set_resolution(unsigned int width, unsigned int height)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.input_width != width || my_css.input_height != height)
		sh_css_invalidate_morph_table();

	my_css.input_width  = width;
	my_css.input_height = height;
	return sh_css_success;
}

enum sh_css_err
sh_css_input_set_effective_resolution(unsigned int width, unsigned int height)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	my_css.input_effective_info.width = width;
	my_css.input_effective_info.padded_width = width;
	my_css.input_effective_info.height = height;
	return sh_css_success;
}

void
sh_css_input_set_format(enum sh_css_input_format format)
{
	my_css.input_format = format;
}

void
sh_css_input_get_format(enum sh_css_input_format *format)
{
	*format = my_css.input_format;
}


void
sh_css_input_set_binning_factor(unsigned int binning_factor)
{
	if (binning_factor != my_css.sensor_binning)
		reset_mode_shading_tables();
	my_css.sensor_binning = binning_factor;
}

void
sh_css_input_set_two_pixels_per_clock(bool two_pixels_per_clock)
{
	if (my_css.two_ppc != two_pixels_per_clock) {
		my_css.two_ppc = two_pixels_per_clock;
		my_css.reconfigure_css_rx = true;
	}
}

void
sh_css_input_get_two_pixels_per_clock(bool *two_pixels_per_clock)
{
	*two_pixels_per_clock = my_css.two_ppc;
}


void
sh_css_input_set_bayer_order(enum sh_css_bayer_order bayer_order)
{
	my_css.bayer_order = bayer_order;
}

int
sh_css_get_extra_pixels_count(int *extra_rows, int *extra_cols)
{
	int rows = SH_CSS_MAX_LEFT_CROPPING,
	    cols = SH_CSS_MAX_LEFT_CROPPING;

	if (lines_needed_for_bayer_order())
		rows += 2;

	if (columns_needed_for_bayer_order())
		cols  += 2;

	*extra_rows = rows;
	*extra_cols = cols;

	return 0;
}


void
sh_css_input_set_channel(unsigned int channel_id)
{
	my_css.ch_id = channel_id;
}

void
sh_css_input_set_mode(enum sh_css_input_mode mode)
{
	enum sh_css_input_mode prev = my_css.input_mode;

	if (prev != mode) {
		if (mode == SH_CSS_INPUT_MODE_MEMORY
		    || prev == SH_CSS_INPUT_MODE_MEMORY) {
			/* if we switch from online to offline, we need to
			   reload the binary */
			invalidate_video_binaries();
		}
	}
	my_css.input_mode = mode;
}

static void
init_copy_descr(struct sh_css_frame_info *in_info,
		struct sh_css_frame_info *out_info)
{
	*in_info = *out_info;

	copy_descr.mode          = SH_CSS_BINARY_MODE_COPY;
	copy_descr.online        = true;
	copy_descr.stream_format = my_css.input_format;
	copy_descr.two_ppc       = my_css.two_ppc;
	copy_descr.in_info       = in_info;
	copy_descr.out_info      = out_info;
	copy_descr.vf_info       = NULL;
}

/* Bilinear interpolation on shading tables:
 * For each target point T, we calculate the 4 surrounding source points:
 * ul (upper left), ur (upper right), ll (lower left) and lr (lower right).
 * We then calculate the distances from the T to the source points: x0, x1,
 * y0 and y1.
 * We then calculate the value of T:
 *   dx0*dy0*Slr + dx0*dy1*Sur + dx1*dy0*Sll + dx1*dy1*Sul.
 * We choose a grid size of 1x1 which means:
 *   dx1 = 1-dx0
 *   dy1 = 1-dy0
 *
 *   Sul dx0         dx1      Sur
 *    .<----->|<------------->.
 *    ^
 * dy0|
 *    v        T
 *    -        .
 *    ^
 *    |
 * dy1|
 *    v
 *    .                        .
 *   Sll                      Slr
 *
 * Padding:
 * The area that the ISP operates on can include padding both on the left
 * and the right. We need to padd the shading table such that the shading
 * values end up on the correct pixel values. This means we must padd the
 * shading table to match the ISP padding.
 * We can have 5 cases:
 * 1. All 4 points fall in the left padding.
 * 2. The left 2 points fall in the left padding.
 * 3. All 4 points fall in the cropped (target) region.
 * 4. The right 2 points fall in the right padding.
 * 5. All 4 points fall in the right padding.
 * Cases 1 and 5 are easy to handle: we simply use the
 * value 1 in the shading table.
 * Cases 2 and 4 require interpolation that takes into
 * account how far into the padding area the pixels
 * fall. We extrapolate the shading table into the
 * padded area and then interpolate.
 */
static void
crop_and_interpolate(unsigned int cropped_width,
		     unsigned int cropped_height,
		     unsigned int left_padding,
		     unsigned int right_padding,
		     struct sh_css_shading_table *out_table,
		     enum sh_css_sc_color color)
{
	unsigned int i, j,
		     sensor_width  = my_css.shading_table->sensor_width,
		     sensor_height = my_css.shading_table->sensor_height,
		     table_width   = my_css.shading_table->width,
		     table_height  = my_css.shading_table->height,
		     table_cell_h,
		     out_cell_size,
		     in_cell_size,
		     out_start_row,
		     padded_width;
	int out_start_col, /* can be negative to indicate padded space */
	    table_cell_w;
	unsigned short *in_ptr = my_css.shading_table->data[color],
		       *out_ptr = out_table->data[color];

	padded_width = cropped_width + left_padding + right_padding;
	out_cell_size = CEIL_DIV(padded_width, out_table->width - 1);
	in_cell_size  = CEIL_DIV(sensor_width, table_width - 1);

	out_start_col = (sensor_width - cropped_width)/2 - left_padding;
	out_start_row = (sensor_height - cropped_height)/2;
	table_cell_w = (int)((table_width-1) * in_cell_size);
	table_cell_h = (table_height-1) * in_cell_size;

	for (i = 0; i < out_table->height; i++) {
		unsigned int ty, src_y0, src_y1, sy0, sy1, dy0, dy1, divy;

		/* calculate target point and make sure it falls within
		   the table */
		ty = out_start_row + i * out_cell_size;
		ty = min(ty, sensor_height-1);
		ty = min(ty, table_cell_h);
		/* calculate closest source points in shading table and
		   make sure they fall within the table */
		src_y0 = ty / in_cell_size;
		if (in_cell_size < out_cell_size)
			src_y1 = (ty + out_cell_size) / in_cell_size;
		else
			src_y1 = src_y0 + 1;
		src_y0 = min(src_y0, table_height-1);
		src_y1 = min(src_y1, table_height-1);
		/* calculate closest source points for distance computation */
		sy0 = min(src_y0 * in_cell_size, sensor_height-1);
		sy1 = min(src_y1 * in_cell_size, sensor_height-1);
		/* calculate distance between source and target pixels */
		dy0 = ty - sy0;
		dy1 = sy1 - ty;
		divy = sy1 - sy0;
		if (divy == 0) {
			dy0 = 1;
			divy = 1;
		}

		for (j = 0; j < out_table->width; j++, out_ptr++) {
			int tx, src_x0, src_x1;
			unsigned int sx0, sx1, dx0, dx1, divx;
			unsigned short s_ul, s_ur, s_ll, s_lr;

			/* calculate target point */
			tx = out_start_col + j * out_cell_size;
			/* calculate closest source points. */
			src_x0 = tx / (int)in_cell_size;
			if (in_cell_size < out_cell_size) {
				src_x1 = (tx + out_cell_size) /
					 (int)in_cell_size;
			} else {
				src_x1 = src_x0 + 1;
			}
			/* if src points fall in padding, select closest ones.*/
			src_x0 = clamp(src_x0, 0, (int)table_width-1);
			src_x1 = clamp(src_x1, 0, (int)table_width-1);
			tx = min(clamp(tx, 0, (int)sensor_width-1),
				 (int)table_cell_w);
			/* calculate closest source points for distance
			   computation */
			sx0 = min(src_x0 * in_cell_size, sensor_width-1);
			sx1 = min(src_x1 * in_cell_size, sensor_width-1);
			/* calculate distances between source and target
			   pixels */
			dx0 = tx - sx0;
			dx1 = sx1 - tx;
			divx = sx1 - sx0;
			/* if we're at the edge, we just use the closest
			   point still in the grid. We make up for the divider
			   in this case by setting the distance to
			   out_cell_size, since it's actually 0. */
			if (divx == 0) {
				dx0 = 1;
				divx = 1;
			}

			/* get source pixel values */
			s_ul = in_ptr[(table_width*src_y0)+src_x0];
			s_ur = in_ptr[(table_width*src_y0)+src_x1];
			s_ll = in_ptr[(table_width*src_y1)+src_x0];
			s_lr = in_ptr[(table_width*src_y1)+src_x1];

			*out_ptr = (dx0*dy0*s_lr +
				    dx0*dy1*s_ur +
				    dx1*dy0*s_ll +
				    dx1*dy1*s_ul) / (divx*divy);
		}
	}
}

static void
generate_id_shading_table(struct sh_css_shading_table **target_table,
			  const struct sh_css_binary *binary)
{
	/* initialize table with ones, shift becomes zero */
	unsigned int i, j, table_width, table_height;
	struct sh_css_shading_table *result;

	table_width  = binary->sctbl_width_per_color;
	table_height = binary->sctbl_height;
	result = sh_css_shading_table_alloc(table_width, table_height);
	if (result == NULL) {
		*target_table = NULL;
		return;
	}

	for (i = 0; i < SH_CSS_SC_NUM_COLORS; i++) {
		for (j = 0; j < table_height * table_width; j++)
			result->data[i][j] = 1;
	}
	result->fraction_bits = 0;
	*target_table = result;
}

static void
prepare_shading_table(struct sh_css_shading_table **target_table,
		      const struct sh_css_binary *binary)
{
	unsigned int input_width,
		     input_height,
		     table_width,
		     table_height,
		     left_padding,
		     right_padding,
		     i;
	struct sh_css_shading_table *result;

	if (!my_css.shading_table) {
		generate_id_shading_table(target_table, binary);
		return;
	}

	left_padding = my_css.curr_if_a_config.left_padding;
	if (my_css.two_ppc)
		left_padding += my_css.curr_if_a_config.left_padding;
	/* We use the ISP input resolution for the shading table because
	   shading correction is performed in the bayer domain (before bayer
	   down scaling). */
	input_height  = binary->in_frame_info.height;
	input_width   = binary->in_frame_info.width;
	left_padding  = binary->left_padding;
	right_padding = binary->in_frame_info.padded_width -
			(input_width + left_padding);

	/* We take into account the binning done by the sensor. We do this
	   by cropping the non-binned part of the shading table and then
	   increasing the size of a grid cell with this same binning factor. */
	input_width  <<= my_css.sensor_binning;
	input_height <<= my_css.sensor_binning;
	/* We also scale the padding by the same binning factor. This will
	   make it much easier later on to calculate the padding of the
	   shading table. */
	left_padding  <<= my_css.sensor_binning;
	right_padding <<= my_css.sensor_binning;

	/* during simulation, the used resolution can exceed the sensor
	   resolution, so we clip it. */
	input_width  = min(input_width,  my_css.shading_table->sensor_width);
	input_height = min(input_height, my_css.shading_table->sensor_height);

	table_width  = binary->sctbl_width_per_color;
	table_height = binary->sctbl_height;

	result = sh_css_shading_table_alloc(table_width, table_height);
	if (result == NULL) {
		*target_table = NULL;
		return;
	}
	result->sensor_width  = my_css.shading_table->sensor_width;
	result->sensor_height = my_css.shading_table->sensor_height;
	result->fraction_bits = my_css.shading_table->fraction_bits;

	/* now we crop the original shading table and then interpolate to the
	   requested resolution and decimation factor. */
	for (i = 0; i < SH_CSS_SC_NUM_COLORS; i++) {
		crop_and_interpolate(input_width, input_height,
				     left_padding, right_padding, result, i);
	}
	*target_table = result;
}

static void
init_offline_descr(struct sh_css_binary_descr *descr,
		   int mode,
		   struct sh_css_frame_info *in_info,
		   struct sh_css_frame_info *out_info,
		   struct sh_css_frame_info *vf_info)
{
	descr->mode          = mode;
	descr->online        = false;
	descr->stream_format = my_css.input_format;
	descr->two_ppc       = false;
	descr->in_info       = in_info;
	descr->out_info      = out_info;
	descr->vf_info       = vf_info;
}

static void
init_vf_pp_descr(struct sh_css_frame_info *in_info,
		 struct sh_css_frame_info *out_info)
{
	in_info->raw_bit_depth = 0;
	init_offline_descr(&vf_pp_descr, SH_CSS_BINARY_MODE_VF_PP,
			   in_info, out_info, NULL);
}

static void
init_preview_descr(struct sh_css_frame_info *in_info,
		   struct sh_css_frame_info *out_info)
{
	int mode = SH_CSS_BINARY_MODE_PREVIEW;

	*in_info = my_css.input_effective_info;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	if (input_format_is_yuv(my_css.input_format))
		mode = SH_CSS_BINARY_MODE_COPY;
	else
		in_info->format = SH_CSS_FRAME_FORMAT_RAW;

	init_offline_descr(&preview_descr, mode,
			   in_info, out_info, NULL);
	preview_descr.online	    = my_css.preview_settings.online;
	preview_descr.stream_format = my_css.input_format;
}

/* configure and load the copy binary, the next binary is used to
   determine whether the copy binary needs to do left padding. */
static enum sh_css_err
load_copy_binary(struct sh_css_binary *copy_binary,
		 struct sh_css_binary *next_binary)
{
	struct sh_css_frame_info copy_out_info, copy_in_info;
	unsigned int left_padding;
	enum sh_css_err err;
	int mode = SH_CSS_BINARY_MODE_COPY;

	if (next_binary) {
		copy_out_info = next_binary->in_frame_info;
		left_padding = next_binary->left_padding;
	} else {
		copy_out_info = my_css.capture_settings.output_info;
		left_padding = 0;
		if (my_css.capture_settings.bayer_ds)
			mode = SH_CSS_BINARY_MODE_BAYER_DS;
	}

	init_copy_descr(&copy_in_info, &copy_out_info);
	copy_descr.mode = mode;
	err = sh_css_binary_find(&copy_descr, copy_binary);
	if (err != sh_css_success)
		return err;
	copy_binary->left_padding = left_padding;
	return sh_css_success;
}

static enum sh_css_err
load_preview_binaries(void)
{
	struct sh_css_frame_info prev_in_info,
				 prev_out_info,
				 ref_info;
	enum sh_css_err err = sh_css_success;
	bool online = my_css.preview_settings.online;
	bool continuous = my_css.continuous;
	unsigned int i;

	if (my_css.preview_settings.preview_binary.info &&
	    my_css.preview_settings.vf_pp_binary.info)
		return sh_css_success;

	err = check_input(false);
	if (err != sh_css_success)
		return err;
	err = check_frame_info(&my_css.preview_settings.output_info);
	if (err != sh_css_success)
		return err;

	/* Preview */
	if (my_css.preview_settings.pp_input_info.width)
		prev_out_info = my_css.preview_settings.pp_input_info;
	else
		prev_out_info = my_css.preview_settings.output_info;
	sh_css_frame_info_set_format(&prev_out_info,
				     SH_CSS_FRAME_FORMAT_YUV_LINE);
	init_preview_descr(&prev_in_info, &prev_out_info);
	err = sh_css_binary_find(&preview_descr,
				 &my_css.preview_settings.preview_binary);
	if (err != sh_css_success)
		return err;

	/* Viewfinder post-processing */
	init_vf_pp_descr(
			&my_css.preview_settings.preview_binary.out_frame_info,
			&my_css.preview_settings.output_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &my_css.preview_settings.vf_pp_binary);
	if (err != sh_css_success)
		return err;

	/* Copy */
	if (!online && !continuous) {
		err = load_copy_binary(&my_css.preview_settings.copy_binary,
				       &my_css.preview_settings.preview_binary);
		if (err != sh_css_success)
			return err;
	}

	ref_info = my_css.preview_settings.preview_binary.internal_frame_info;
	ref_info.format = SH_CSS_FRAME_FORMAT_YUV420;

	for (i = 0; i < NUM_REF_FRAMES; i++) {
		if (my_css.preview_settings.ref_frames[i]) {
			sh_css_frame_free(
				my_css.preview_settings.ref_frames[i]);
		}
		err = sh_css_frame_allocate_from_info(
				&my_css.preview_settings.ref_frames[i],
				&ref_info);
		if (err != sh_css_success)
			return err;
	}

	my_css.preview_settings.prev_ref_frame = 0;
	if (SH_CSS_PREVENT_UNINIT_READS)
		sh_css_frame_zero(my_css.preview_settings.ref_frames[0]);

	if (!continuous)
		ref_info = my_css.preview_settings.preview_binary.in_frame_info;
	else
		ref_info =
		  my_css.preview_settings.preview_binary.internal_frame_info;
	ref_info.format = SH_CSS_FRAME_FORMAT_RAW;

	for (i = 0; i < NUM_CONTINUOUS_FRAMES; i++) {
		if (my_css.preview_settings.continuous_frames[i])
			sh_css_frame_free(
				my_css.preview_settings.continuous_frames[i]);
		err = sh_css_frame_allocate_from_info(
			&my_css.preview_settings.continuous_frames[i],
			&ref_info);
		if (err != sh_css_success)
			return err;
	}

	my_css.preview_settings.continuous_frame = 0;
	if (SH_CSS_PREVENT_UNINIT_READS)
		sh_css_frame_zero(my_css.preview_settings.continuous_frames[0]);

	if (my_css.preview_settings.shading_table) {
		sh_css_shading_table_free(
				my_css.preview_settings.shading_table);
		my_css.preview_settings.shading_table = NULL;
	}
	return sh_css_success;
}

static const struct sh_css_acc_fw *
last_output_firmware(const struct sh_css_acc_fw *fw)
{
	const struct sh_css_acc_fw *last_fw = NULL;
	for (; fw; fw = fw->header.next) {
		if (fw->header.sp.output)
			last_fw = fw;
	}
	return last_fw;
}

static enum sh_css_err
add_firmwares(struct sh_css_pipeline *me,
	      struct sh_css_binary *binary,
	      const struct sh_css_acc_fw *fw,
	      const struct sh_css_acc_fw *last_fw,
	      unsigned int binary_mode,
	      struct sh_css_frame *in_frame,
	      struct sh_css_frame *out_frame,
	      struct sh_css_frame *vf_frame,
	      struct sh_css_pipeline_stage **my_stage,
	      struct sh_css_pipeline_stage **vf_stage)
{
	enum sh_css_err err = sh_css_success;
	struct sh_css_pipeline_stage *extra_stage = NULL;
	for (; fw; fw = fw->header.next) {
		struct sh_css_frame *out = NULL;
		if (fw == last_fw)
			out = out_frame;
		err = sh_css_pipeline_add_stage(me, binary, fw,
				binary_mode, NULL,
				in_frame, out,
				vf_frame, &extra_stage);
		if (err != sh_css_success)
			return err;
		if (fw->header.sp.output)
			in_frame = extra_stage->args.out_frame;
		if (my_stage && !*my_stage && extra_stage)
			*my_stage = extra_stage;
		if (vf_stage && !*vf_stage && extra_stage &&
		    fw->header.sp.out_vf)
			*vf_stage = extra_stage;
	}
	return err;
}

static enum sh_css_err
add_vf_pp_stage(struct sh_css_pipeline *me,
		struct sh_css_frame *out_frame,
		struct sh_css_binary *vf_pp_binary,
		struct sh_css_pipeline_stage *post_stage,
		struct sh_css_pipeline_stage **vf_pp_stage)
{
	const struct sh_css_acc_fw *last_fw;
	enum sh_css_err err = sh_css_success;
	struct sh_css_frame *in_frame = post_stage->args.out_vf_frame;

	*vf_pp_stage = NULL;

	if (!in_frame)
		in_frame = post_stage->args.out_frame;

	last_fw = last_output_firmware(my_css.vf_stage);
	if (!my_css.disable_vf_pp) {
		err = sh_css_pipeline_add_stage(me, vf_pp_binary, NULL,
				vf_pp_binary->info->mode, NULL,
				in_frame,
				last_fw ? NULL : out_frame,
				NULL, vf_pp_stage);
		if (err != sh_css_success)
			return err;
		in_frame = (*vf_pp_stage)->args.out_frame;
	}
	err = add_firmwares(me, vf_pp_binary, my_css.vf_stage, last_fw,
			    SH_CSS_BINARY_MODE_VF_PP,
			    in_frame, out_frame, NULL,
			    vf_pp_stage, NULL);
	return err;
}

static enum sh_css_err
add_capture_pp_stage(struct sh_css_pipeline *me,
		     struct sh_css_frame *out_frame,
		     struct sh_css_binary *capture_pp_binary,
		     struct sh_css_pipeline_stage *capture_stage,
		     struct sh_css_pipeline_stage **pre_vf_pp_stage)
{
	const struct sh_css_acc_fw *last_fw;
	enum sh_css_err err = sh_css_success;
	struct sh_css_frame *in_frame = capture_stage->args.out_frame;
	struct sh_css_frame *vf_frame = NULL;

	*pre_vf_pp_stage = NULL;

	if (!in_frame)
		in_frame = capture_stage->args.out_frame;

	last_fw = last_output_firmware(my_css.output_stage);
	if (!my_css.disable_capture_pp && my_css.capture_settings.need_pp) {
		err = sh_css_frame_allocate_from_info(&vf_frame,
					    &capture_pp_binary->vf_frame_info);
		if (err != sh_css_success)
			return err;
		err = sh_css_pipeline_add_stage(me, capture_pp_binary, NULL,
				capture_pp_binary->info->mode, NULL,
				NULL,
				last_fw ? NULL : out_frame,
				vf_frame, pre_vf_pp_stage);
		if (err != sh_css_success)
			return err;
		in_frame = (*pre_vf_pp_stage)->args.out_frame;
	}
	err = add_firmwares(me, capture_pp_binary, my_css.output_stage, last_fw,
			    SH_CSS_BINARY_MODE_CAPTURE_PP,
			    in_frame, out_frame, vf_frame,
			    NULL, pre_vf_pp_stage);
	/* If a firmware produce vf_pp output, we set that as vf_pp input */
	if (*pre_vf_pp_stage) {
		(*pre_vf_pp_stage)->args.extra_frame =
		  my_css.capture_settings.capture_pp_frame;
		(*pre_vf_pp_stage)->args.vf_downscale_log2 =
		  capture_pp_binary->vf_downscale_log2;
	} else {
		*pre_vf_pp_stage = capture_stage;
	}
	return err;
}


enum sh_css_err
sh_css_preview_start(struct sh_css_frame *raw_out_frame,
		     struct sh_css_frame *out_frame)
{
	struct sh_css_pipeline *me = &my_css.preview_settings.pipeline;
	struct sh_css_pipeline_stage *preview_stage, *copy_stage;
	struct sh_css_pipeline_stage *vf_pp_stage;
	struct sh_css_frame *in_frame = NULL, *in_ref_frame, *cc_frame = NULL;
	struct sh_css_binary *copy_binary, *preview_binary, *vf_pp_binary;
	enum sh_css_err err = sh_css_success;

	if (!out_frame)
		return sh_css_err_invalid_arguments;
	copy_stage = NULL;

	if (my_css.invalidate || my_css.preview_settings.zoom_changed) {
		invalidate_preview_binaries();
		my_css.preview_settings.zoom_changed = false;
		my_css.invalidate = false;
	}

	err = load_preview_binaries();
	if (err != sh_css_success)
		return err;

	err = check_infos_match(&out_frame->info,
			&my_css.preview_settings.vf_pp_binary.out_frame_info);
	if (err != sh_css_success)
		return err;

	copy_binary    = &my_css.preview_settings.copy_binary;
	preview_binary = &my_css.preview_settings.preview_binary;
	vf_pp_binary   = &my_css.preview_settings.vf_pp_binary;

	sh_css_metrics_start_frame();

	if (me->reload) {
		struct sh_css_pipeline_stage *post_stage;
		sh_css_pipeline_clean(me);
		if (my_css.preview_settings.copy_binary.info) {
			err = sh_css_pipeline_add_stage(me, copy_binary, NULL,
					copy_binary->info->mode,
					NULL, NULL, raw_out_frame, NULL,
					&post_stage);
			if (err != sh_css_success)
				return err;
			in_frame = me->stages->args.out_frame;
		} else {
			in_frame = my_css.preview_settings.continuous_frames
				[my_css.preview_settings.continuous_frame];
		}
		err = sh_css_pipeline_add_stage(me, preview_binary, NULL,
						preview_binary->info->mode,
						cc_frame, in_frame, NULL, NULL,
						&post_stage);
		if (err != sh_css_success)
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

		err = add_vf_pp_stage(me, out_frame, vf_pp_binary,
				      post_stage, &vf_pp_stage);
		if (err != sh_css_success)
			return err;
		me->reload = false;
	} else {
		sh_css_pipeline_restart(me);
	}
	err = sh_css_config_input_network(me, copy_binary);
	if (err != sh_css_success)
		return err;
	if (!my_css.preview_settings.shading_table) {
		prepare_shading_table(
				&my_css.preview_settings.shading_table,
				preview_binary);
	}
	sh_css_params_set_shading_table(my_css.preview_settings.shading_table);

	err = sh_css_pipeline_get_output_stage(me, SH_CSS_BINARY_MODE_VF_PP,
					       &vf_pp_stage);

	if (err != sh_css_success)
		return err;
	err = sh_css_pipeline_get_stage(me, preview_binary->info->mode,
					&preview_stage);
	if (err != sh_css_success)
		return err;

	vf_pp_stage->args.out_frame = out_frame;

	in_ref_frame = my_css.preview_settings.ref_frames
		[my_css.preview_settings.prev_ref_frame];

	/* switch the reference frame buffers */
	my_css.preview_settings.prev_ref_frame++;
	if (my_css.preview_settings.prev_ref_frame == NUM_REF_FRAMES)
		my_css.preview_settings.prev_ref_frame = 0;

	if (my_css.continuous) {
		in_frame = my_css.preview_settings.continuous_frames
			[my_css.preview_settings.continuous_frame];
		my_css.preview_settings.continuous_frame++;
		if (my_css.preview_settings.continuous_frame ==
		    NUM_CONTINUOUS_FRAMES) {
			my_css.preview_settings.continuous_frame = 0;
		}
		cc_frame = my_css.preview_settings.continuous_frames
			[my_css.preview_settings.continuous_frame];
		preview_stage->args.cc_frame = cc_frame;
		preview_stage->args.in_frame = in_frame;
	}
	/* update the arguments with the latest info */
	preview_stage->args.in_ref_frame = in_ref_frame;
	preview_stage->args.out_ref_frame =
		my_css.preview_settings.ref_frames
			[my_css.preview_settings.prev_ref_frame];

	my_css.mode = sh_css_mode_preview;
	return sh_css_pipeline_start_next_stage(me);
}

enum sh_css_err
sh_css_preview_get_output_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;
	err = load_preview_binaries();
	if (err == sh_css_success)
		*info = my_css.preview_settings.output_info;
	return err;
}

enum sh_css_err
sh_css_preview_configure_pp_input(unsigned int width, unsigned int height)
{
	enum sh_css_err err = sh_css_success;

	err = check_null_res(width, height);
	if (err != sh_css_success)
		return err;

	if (my_css.preview_settings.pp_input_info.width != width ||
	    my_css.preview_settings.pp_input_info.height != height) {
		sh_css_frame_info_init(&my_css.preview_settings.pp_input_info,
				       width, height,
				       SH_CSS_FRAME_FORMAT_YUV_LINE);
		invalidate_preview_binaries();
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_preview_get_input_resolution(unsigned int *width,
				    unsigned int *height)
{
	enum sh_css_err err;
	err = load_preview_binaries();
	if (err == sh_css_success) {
		const struct sh_css_binary *binary;
		if (my_css.preview_settings.copy_binary.info)
			binary = &my_css.preview_settings.copy_binary;
		else
			binary = &my_css.preview_settings.preview_binary;
		*width  = binary->in_frame_info.width +
			  columns_needed_for_bayer_order();
		*height = binary->in_frame_info.height +
			  lines_needed_for_bayer_order();
	}
	return err;
}

void
sh_css_preview_enable_online(bool enable)
{
	my_css.preview_settings.online = enable;
}

void
sh_css_enable_continuous(bool enable)
{
	my_css.continuous = enable;
}

bool
sh_css_continuous_is_enabled(void)
{
	return my_css.continuous;
}

bool
sh_css_continuous_start_sp_copy(void)
{
	return my_css.start_sp_copy;
}

void
sh_css_disable_vf_pp(bool disable)
{
	my_css.disable_vf_pp = disable;
}

void
sh_css_disable_capture_pp(bool disable)
{
	my_css.disable_capture_pp = disable;
}

enum sh_css_err
sh_css_preview_configure_output(unsigned int width,
				unsigned int height,
				enum sh_css_frame_format format)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.preview_settings.output_info.width != width ||
	    my_css.preview_settings.output_info.height != height ||
	    my_css.preview_settings.output_info.format != format) {
		sh_css_frame_info_init(&my_css.preview_settings.output_info,
				       width, height, format);
		invalidate_preview_binaries();
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_preview_get_grid_info(struct sh_css_grid_info *info)
{
	enum sh_css_err err;
	err = load_preview_binaries();
	if (err == sh_css_success) {
		err = sh_css_binary_grid_info(
				&my_css.preview_settings.preview_binary,
				info);
	}
	return err;
}

bool
sh_css_preview_next_stage_needs_alloc(void)
{
	return my_css.invalidate ||
	       my_css.preview_settings.pipeline.reload ||
	       my_css.preview_settings.zoom_changed ||
	       my_css.preview_settings.preview_binary.info == NULL ||
	       my_css.preview_settings.shading_table == NULL;
}

static void
init_video_descr(struct sh_css_frame_info *in_info,
		 struct sh_css_frame_info *vf_info)
{
	int mode = SH_CSS_BINARY_MODE_VIDEO;

	if (input_format_is_yuv(my_css.input_format))
		mode = SH_CSS_BINARY_MODE_COPY;
	*in_info = my_css.input_effective_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&video_descr, mode,
			   in_info, &my_css.video_settings.output_info,
			   vf_info);
	video_descr.online = my_css.input_mode != SH_CSS_INPUT_MODE_MEMORY;
}

static enum sh_css_err
load_video_binaries(void)
{
	struct sh_css_frame_info video_in_info, ref_info, tnr_info,
				 *video_vf_info;
	bool online;
	enum sh_css_err err = sh_css_success;
	int i;

	/* we only test the video_binary because offline video doesn't need a
	 * vf_pp binary. Both are always reset together anyway.
	 */
	if (my_css.video_settings.video_binary.info)
		return sh_css_success;

	online = my_css.input_mode != SH_CSS_INPUT_MODE_MEMORY;
	err = check_input(!online);
	if (err != sh_css_success)
		return err;
	if (online) {
		err = check_vf_out_info(&my_css.video_settings.output_info,
					&my_css.video_settings.vf_info);
	} else {
		err = check_frame_info(&my_css.video_settings.output_info);
	}
	if (err != sh_css_success)
		return err;

	/* Video */
	if (online)
		video_vf_info = &my_css.video_settings.vf_info;
	else
		video_vf_info = NULL;
	init_video_descr(&video_in_info, video_vf_info);
	err = sh_css_binary_find(&video_descr,
				 &my_css.video_settings.video_binary);
	if (err != sh_css_success)
		return err;

	/* This is where we set the flag for invalid first frame */
	my_css.video_settings.invalid_first_frame = true;

	/* Viewfinder post-processing */
	if (online) {
		init_vf_pp_descr(
			&my_css.video_settings.video_binary.vf_frame_info,
			&my_css.video_settings.vf_info);
		err = sh_css_binary_find(&vf_pp_descr,
				&my_css.video_settings.vf_pp_binary);
		if (err != sh_css_success)
			return err;
	}

	/* yuv copy does not use reference frames */
	if (input_format_is_yuv(my_css.input_format))
		return sh_css_success;

	ref_info = my_css.video_settings.video_binary.internal_frame_info;
	ref_info.format = SH_CSS_FRAME_FORMAT_YUV420;

	for (i = 0; i < NUM_REF_FRAMES; i++) {
		if (my_css.video_settings.ref_frames[i])
			sh_css_frame_free(my_css.video_settings.ref_frames[i]);
		err = sh_css_frame_allocate_from_info(
				&my_css.video_settings.ref_frames[i],
				&ref_info);
		if (err != sh_css_success)
			return err;
	}
	my_css.video_settings.prev_ref_frame = 0;
	if (SH_CSS_PREVENT_UNINIT_READS)
		sh_css_frame_zero(my_css.video_settings.ref_frames[0]);

	tnr_info = my_css.video_settings.video_binary.internal_frame_info;
	tnr_info.format = SH_CSS_FRAME_FORMAT_YUV420;

	for (i = 0; i < NUM_TNR_FRAMES; i++) {
		if (my_css.video_settings.tnr_frames[i])
			sh_css_frame_free(my_css.video_settings.tnr_frames[i]);
		err = sh_css_frame_allocate_from_info(
				&my_css.video_settings.tnr_frames[i],
				&tnr_info);
		if (err != sh_css_success)
			return err;
	}
	my_css.video_settings.prev_tnr_frame = 0;
	if (SH_CSS_PREVENT_UNINIT_READS) {
		sh_css_frame_zero(my_css.video_settings.tnr_frames[0]);
		sh_css_frame_zero(my_css.video_settings.tnr_frames[1]);
	}

	return sh_css_success;
}

enum sh_css_err
sh_css_video_start(struct sh_css_frame *in_frame,
		   struct sh_css_frame *out_frame,
		   struct sh_css_frame *vf_frame)
{
	struct sh_css_pipeline *me = &my_css.video_settings.pipeline;
	struct sh_css_pipeline_stage *video_stage, *vf_pp_stage;
	struct sh_css_frame *in_ref_frame, *in_tnr_frame;
	struct sh_css_binary *video_binary, *vf_pp_binary;
	enum sh_css_err err = sh_css_success;

	if (!out_frame)
		return sh_css_err_invalid_arguments;
	if (my_css.invalidate || my_css.video_settings.zoom_changed) {
		invalidate_video_binaries();
		my_css.video_settings.zoom_changed = false;
		my_css.invalidate = false;
	}

	err = load_video_binaries();
	if (err != sh_css_success)
		return err;
	video_binary = &my_css.video_settings.video_binary;
	vf_pp_binary = &my_css.video_settings.vf_pp_binary;
	if (in_frame) {
		if (vf_frame)
			return sh_css_err_mode_does_not_have_viewfinder;
	} else {
		if (!vf_frame)
			return sh_css_err_invalid_arguments;
		err = check_infos_match(&vf_frame->info,
			&my_css.video_settings.vf_pp_binary.out_frame_info);
		if (err != sh_css_success)
			return err;
	}
	err = check_infos_match(&out_frame->info,
			&my_css.video_settings.video_binary.out_frame_info);
	if (err != sh_css_success)
		return err;

	sh_css_metrics_start_frame();

	if (me->reload) {
		sh_css_pipeline_clean(me);
		err = sh_css_pipeline_add_stage(me, video_binary, NULL,
						video_binary->info->mode, NULL,
						in_frame, out_frame, NULL,
						&video_stage);
		if (err != sh_css_success)
			return err;
		/* If we use copy iso video, the input must be yuv iso raw */
		video_stage->args.copy_vf =
			video_binary->info->mode == SH_CSS_BINARY_MODE_COPY;
		video_stage->args.copy_output = video_stage->args.copy_vf;
		if (!in_frame) {
			err = add_vf_pp_stage(me, vf_frame, vf_pp_binary,
					      video_stage, &vf_pp_stage);
			if (err != sh_css_success)
				return err;
		}
		me->reload = false;
	} else {
		sh_css_pipeline_restart(me);
	}
	err = sh_css_config_input_network(me, NULL);
	if (err != sh_css_success)
		return err;
	if (!my_css.video_settings.shading_table) {
		prepare_shading_table(
				&my_css.video_settings.shading_table,
				video_binary);
	}
	sh_css_params_set_shading_table(my_css.video_settings.shading_table);

	err = sh_css_pipeline_get_stage(me, video_binary->info->mode,
					&video_stage);
	if (err != sh_css_success)
		return err;
	if (!in_frame) {
		err = sh_css_pipeline_get_output_stage(me,
						       vf_pp_binary->info->mode,
						       &vf_pp_stage);
		if (err != sh_css_success)
			return err;
	}

	in_ref_frame = my_css.video_settings.ref_frames
		[my_css.video_settings.prev_ref_frame];
	/* switch the reference frame buffers */
	my_css.video_settings.prev_ref_frame++;
	if (my_css.video_settings.prev_ref_frame == NUM_VIDEO_REF_FRAMES)
		my_css.video_settings.prev_ref_frame = 0;

	in_tnr_frame = my_css.video_settings.tnr_frames
		[my_css.video_settings.prev_tnr_frame];
	/* switch the tnr frame buffers */
	my_css.video_settings.prev_tnr_frame++;
	if (my_css.video_settings.prev_tnr_frame == NUM_VIDEO_TNR_FRAMES)
		my_css.video_settings.prev_tnr_frame = 0;

	/* update the arguments with the latest info */
	video_stage->args.in_ref_frame = in_ref_frame;
	video_stage->args.in_tnr_frame = in_tnr_frame;
	video_stage->args.out_frame = out_frame;
	video_stage->args.out_ref_frame =
		my_css.video_settings.ref_frames
			[my_css.video_settings.prev_ref_frame];
	video_stage->args.out_tnr_frame =
		my_css.video_settings.tnr_frames
			[my_css.video_settings.prev_tnr_frame];
	video_stage->args.dvs_vector_x = my_css.video_settings.dvs_vector_x;
	video_stage->args.dvs_vector_y = my_css.video_settings.dvs_vector_y;
	if (!in_frame)
		vf_pp_stage->args.out_frame = vf_frame;

	my_css.mode = sh_css_mode_video;
	return sh_css_pipeline_start_next_stage(me);
}

enum sh_css_err
sh_css_video_get_output_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;

	err = load_video_binaries();
	if (err == sh_css_success)
		*info = my_css.video_settings.output_info;
	return err;
}

enum sh_css_err
sh_css_video_get_viewfinder_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;

	err = load_video_binaries();
	if (err != sh_css_success)
		return err;
	/* offline video does not generate viewfinder output */
	if (my_css.input_mode == SH_CSS_INPUT_MODE_MEMORY)
		return sh_css_err_mode_does_not_have_viewfinder;
	else
		*info = my_css.video_settings.vf_info;
	return sh_css_success;
}

enum sh_css_err
sh_css_video_get_grid_info(struct sh_css_grid_info *info)
{
	enum sh_css_err err;

	err = load_video_binaries();
	if (err != sh_css_success)
		return err;
	err = sh_css_binary_grid_info(&my_css.video_settings.video_binary,
				      info);
	return err;
}

enum sh_css_err
sh_css_video_get_input_resolution(unsigned int *width, unsigned int *height)
{
	enum sh_css_err err;

	err = load_video_binaries();
	if (err == sh_css_success) {
		const struct sh_css_binary *binary;
		binary = &my_css.video_settings.video_binary;
		*width  = binary->in_frame_info.width +
			  columns_needed_for_bayer_order();
		*height = binary->in_frame_info.height +
			  lines_needed_for_bayer_order();
	}

	return err;
}

enum sh_css_err
sh_css_video_configure_output(unsigned int width,
			      unsigned int height,
			      enum sh_css_frame_format format)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.video_settings.output_info.width != width ||
	    my_css.video_settings.output_info.height != height ||
	    my_css.video_settings.output_info.format != format) {
		sh_css_frame_info_init(&my_css.video_settings.output_info,
				       width, height, format);
		invalidate_video_binaries();
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_video_configure_viewfinder(unsigned int width,
				  unsigned int height,
				  enum sh_css_frame_format format)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.video_settings.vf_info.width != width ||
	    my_css.video_settings.vf_info.height != height ||
	    my_css.video_settings.vf_info.format != format) {
		sh_css_frame_info_init(&my_css.video_settings.vf_info,
				       width, height, format);
		invalidate_video_binaries();
	}
	return sh_css_success;
}

void
sh_css_video_set_dis_vector(int x, int y)
{
	my_css.video_settings.dvs_vector_x = x;
	my_css.video_settings.dvs_vector_y = y;
}

/* Specify the envelope to be used for DIS. */
void
sh_css_video_set_dis_envelope(unsigned int width, unsigned int height)
{
	if (width != my_css.video_settings.dvs_envelope_width ||
	    height != my_css.video_settings.dvs_envelope_height) {
		my_css.video_settings.dvs_envelope_width = width;
		my_css.video_settings.dvs_envelope_height = height;
		invalidate_video_binaries();
	}
}

void
sh_css_video_get_dis_envelope(unsigned int *width, unsigned int *height)
{
	*width = my_css.video_settings.dvs_envelope_width;
	*height = my_css.video_settings.dvs_envelope_height;
}

bool
sh_css_video_next_stage_needs_alloc(void)
{
	return my_css.invalidate ||
	       my_css.video_settings.pipeline.reload ||
	       my_css.video_settings.zoom_changed ||
	       my_css.video_settings.video_binary.info == NULL ||
	       my_css.video_settings.shading_table == NULL;
}

void
sh_css_set_zoom_factor(unsigned int dx, unsigned int dy)
{
	bool is_zoomed  = dx < UDS_SCALING_N || dy < UDS_SCALING_N;
	bool was_zoomed = my_css.curr_dx < UDS_SCALING_N ||
			  my_css.curr_dy < UDS_SCALING_N;

	if (is_zoomed != was_zoomed) {
		/* for with/without zoom, we use different binaries */
		my_css.video_settings.zoom_changed   = true;
		my_css.preview_settings.zoom_changed = true;
		my_css.capture_settings.zoom_changed = true;
	}
	my_css.curr_dx = dx;
	my_css.curr_dy = dy;
}

void
sh_css_get_zoom_factor(unsigned int *dx, unsigned int *dy)
{
	*dx = my_css.curr_dx;
	*dy = my_css.curr_dy;
}

static enum sh_css_err
load_copy_binaries(void)
{
	enum sh_css_err err = sh_css_success;

	if (my_css.capture_settings.copy_binary.info)
		return sh_css_success;

	err = check_frame_info(&my_css.capture_settings.output_info);
	if (err != sh_css_success)
		return err;

	get_copy_out_frame_format(&my_css.capture_settings.output_info.format);
	return load_copy_binary(&my_css.capture_settings.copy_binary, NULL);
}

static bool
need_capture_pp(void)
{
	/* determine whether we need to use the capture_pp binary.
	 * This is needed for:
	 *   1. XNR or
	 *   2. Digital Zoom or
	 *   3. YUV downscaling
	 */
	if (my_css.capture_settings.pp_input_info.width &&
	    ((my_css.capture_settings.pp_input_info.width !=
	      my_css.capture_settings.output_info.width) ||
	     (my_css.capture_settings.pp_input_info.height !=
	      my_css.capture_settings.output_info.height)))
		return true;
	if (my_css.capture_settings.xnr)
		return true;
	if (my_css.curr_dx < UDS_SCALING_N ||
	    my_css.curr_dy < UDS_SCALING_N)
		return true;
	return false;
}

static void
init_capture_pp_descr(struct sh_css_frame_info *in_info,
		      struct sh_css_frame_info *vf_info)
{
	/* the in_info is only used for resolution to enable
	   bayer down scaling. */
	if (my_css.capture_settings.pp_input_info.width)
		*in_info = my_css.capture_settings.pp_input_info;
	else
		*in_info = my_css.capture_settings.output_info;
	in_info->format = SH_CSS_FRAME_FORMAT_YUV420;
	in_info->raw_bit_depth = 0;
	sh_css_frame_info_set_width(in_info, in_info->width);
	init_offline_descr(&capture_pp_descr, SH_CSS_BINARY_MODE_CAPTURE_PP,
			   in_info, &my_css.capture_settings.output_info,
			   vf_info);
}

static void
init_primary_descr(struct sh_css_frame_info *in_info,
		   struct sh_css_frame_info *out_info,
		   struct sh_css_frame_info *vf_info)
{
	int mode = SH_CSS_BINARY_MODE_PRIMARY;

	if (input_format_is_yuv(my_css.input_format))
		mode = SH_CSS_BINARY_MODE_COPY;

	*in_info = my_css.input_effective_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&prim_descr, mode,
			   in_info, out_info, vf_info);
	if (my_css.capture_settings.online) {
		prim_descr.online        = true;
		prim_descr.stream_format = my_css.input_format;
	}
}

static void
init_pre_gdc_descr(struct sh_css_frame_info *in_info,
		   struct sh_css_frame_info *out_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&pre_gdc_descr, SH_CSS_BINARY_MODE_PRE_ISP,
			   in_info, out_info, NULL);
}

static void
init_gdc_descr(struct sh_css_frame_info *in_info,
	       struct sh_css_frame_info *out_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_QPLANE6;
	init_offline_descr(&gdc_descr, SH_CSS_BINARY_MODE_GDC,
			   in_info, out_info, NULL);
}

static void
init_post_gdc_descr(struct sh_css_frame_info *in_info,
		    struct sh_css_frame_info *out_info,
		    struct sh_css_frame_info *vf_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_YUV420_16;
	init_offline_descr(&post_gdc_descr, SH_CSS_BINARY_MODE_POST_ISP,
			   in_info, out_info, vf_info);
}

#ifndef SYSTEM_hive_isp_css_2400_system
static void
init_pre_anr_descr(struct sh_css_frame_info *in_info,
		   struct sh_css_frame_info *out_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&pre_anr_descr, SH_CSS_BINARY_MODE_PRE_ISP,
			   in_info, out_info, NULL);
}

static void
init_anr_descr(struct sh_css_frame_info *in_info,
	       struct sh_css_frame_info *out_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&anr_descr, SH_CSS_BINARY_MODE_ANR,
			   in_info, out_info, NULL);
}

static void
init_post_anr_descr(struct sh_css_frame_info *in_info,
		    struct sh_css_frame_info *out_info,
		    struct sh_css_frame_info *vf_info)
{
	*in_info = *out_info;
	in_info->format = SH_CSS_FRAME_FORMAT_RAW;
	in_info->raw_bit_depth =
		sh_css_input_format_bits_per_pixel(my_css.input_format,
						   my_css.two_ppc);
	init_offline_descr(&post_anr_descr, SH_CSS_BINARY_MODE_POST_ISP,
			   in_info, out_info, vf_info);
}
#endif

static enum sh_css_err
load_primary_binaries(void)
{
	bool online = my_css.capture_settings.online;
	bool continuous = my_css.continuous;
	bool need_pp = false;
	struct sh_css_frame_info prim_in_info,
				 prim_out_info, vf_info,
				 *vf_pp_in_info;
	enum sh_css_err err = sh_css_success;

	if (my_css.capture_settings.primary_binary.info)
		return sh_css_success;

	err = check_vf_out_info(&my_css.capture_settings.output_info,
				&my_css.capture_settings.vf_info);
	if (err != sh_css_success)
		return err;
	need_pp = need_capture_pp();

	/* we use the vf output info to get the primary/capture_pp binary
	   configured for vf_veceven. It will select the closest downscaling
	   factor. */
	vf_info = my_css.capture_settings.vf_info;
	sh_css_frame_info_set_format(&vf_info,
				     SH_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	my_css.capture_settings.need_pp = need_pp;
	if (need_pp) {
		init_capture_pp_descr(&prim_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
				&my_css.capture_settings.capture_pp_binary);
		if (err != sh_css_success)
			return err;
	} else {
		prim_out_info = my_css.capture_settings.output_info;
	}

	/* Primary */
	init_primary_descr(&prim_in_info, &prim_out_info, &vf_info);
	err = sh_css_binary_find(&prim_descr,
				 &my_css.capture_settings.primary_binary);
	if (err != sh_css_success)
		return err;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &my_css.capture_settings.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &my_css.capture_settings.primary_binary.vf_frame_info;
	}

	init_vf_pp_descr(vf_pp_in_info, &my_css.capture_settings.vf_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &my_css.capture_settings.vf_pp_binary);
	if (err != sh_css_success)
		return err;

	/* Copy */
	if (!online && !continuous) {
		err = load_copy_binary(&my_css.capture_settings.copy_binary,
				       &my_css.capture_settings.primary_binary);
		if (err != sh_css_success)
			return err;
	}

	if (need_pp)
		return alloc_capture_pp_frame(
				&my_css.capture_settings.capture_pp_binary);
	else
		return sh_css_success;
}

static enum sh_css_err
load_advanced_binaries(void)
{
	struct sh_css_frame_info pre_in_info, gdc_in_info,
				 post_in_info, post_out_info,
				 vf_info, *vf_pp_in_info;
	bool need_pp;
	enum sh_css_err err = sh_css_success;

	if (my_css.capture_settings.pre_isp_binary.info)
		return sh_css_success;

	err = check_vf_out_info(&my_css.capture_settings.output_info,
				&my_css.capture_settings.vf_info);
	if (err != sh_css_success)
		return err;
	need_pp = need_capture_pp();

	vf_info = my_css.capture_settings.vf_info;
	sh_css_frame_info_set_format(&vf_info,
				     SH_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	my_css.capture_settings.need_pp = need_pp;
	if (need_pp) {
		init_capture_pp_descr(&post_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
				&my_css.capture_settings.capture_pp_binary);
		if (err != sh_css_success)
			return err;
	} else {
		post_out_info = my_css.capture_settings.output_info;
	}

	/* Post-gdc */
	init_post_gdc_descr(&post_in_info, &post_out_info, &vf_info);
	err = sh_css_binary_find(&post_gdc_descr,
				 &my_css.capture_settings.post_isp_binary);
	if (err != sh_css_success)
		return err;

	/* Gdc */
	init_gdc_descr(&gdc_in_info,
		       &my_css.capture_settings.post_isp_binary.in_frame_info);
	err = sh_css_binary_find(&gdc_descr,
				 &my_css.capture_settings.gdc_binary);
	if (err != sh_css_success)
		return err;
	my_css.capture_settings.gdc_binary.left_padding =
		my_css.capture_settings.post_isp_binary.left_padding;

	/* Pre-gdc */
	init_pre_gdc_descr(&pre_in_info,
			   &my_css.capture_settings.gdc_binary.in_frame_info);
	err = sh_css_binary_find(&pre_gdc_descr,
				 &my_css.capture_settings.pre_isp_binary);
	if (err != sh_css_success)
		return err;
	my_css.capture_settings.pre_isp_binary.left_padding =
		my_css.capture_settings.gdc_binary.left_padding;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &my_css.capture_settings.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &my_css.capture_settings.post_isp_binary.vf_frame_info;
	}

	init_vf_pp_descr(vf_pp_in_info, &my_css.capture_settings.vf_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &my_css.capture_settings.vf_pp_binary);
	if (err != sh_css_success)
		return err;

	/* Copy */
	err = load_copy_binary(&my_css.capture_settings.copy_binary,
			       &my_css.capture_settings.pre_isp_binary);
	if (err != sh_css_success)
		return err;

	if (need_pp)
		return alloc_capture_pp_frame(
				&my_css.capture_settings.capture_pp_binary);
	else
		return sh_css_success;
}

#ifndef SYSTEM_hive_isp_css_2400_system
static enum sh_css_err
load_low_light_binaries(void)
{
	struct sh_css_frame_info pre_in_info, anr_in_info,
				 post_in_info, post_out_info,
				 vf_info, *vf_pp_in_info;
	bool need_pp;
	enum sh_css_err err = sh_css_success;

	if (my_css.capture_settings.pre_isp_binary.info)
		return sh_css_success;

	err = check_vf_out_info(&my_css.capture_settings.output_info,
				&my_css.capture_settings.vf_info);
	if (err != sh_css_success)
		return err;
	need_pp = need_capture_pp();

	vf_info = my_css.capture_settings.vf_info;
	sh_css_frame_info_set_format(&vf_info,
				     SH_CSS_FRAME_FORMAT_YUV_LINE);

	/* we build up the pipeline starting at the end */
	/* Capture post-processing */
	my_css.capture_settings.need_pp = need_pp;
	if (need_pp) {
		init_capture_pp_descr(&post_out_info, &vf_info);
		err = sh_css_binary_find(&capture_pp_descr,
				&my_css.capture_settings.capture_pp_binary);
		if (err != sh_css_success)
			return err;
	} else {
		post_out_info = my_css.capture_settings.output_info;
	}

	/* Post-anr */
	init_post_anr_descr(&post_in_info, &post_out_info, &vf_info);
	err = sh_css_binary_find(&post_anr_descr,
				 &my_css.capture_settings.post_isp_binary);
	if (err != sh_css_success)
		return err;

	/* Anr */
	init_anr_descr(&anr_in_info,
		       &my_css.capture_settings.post_isp_binary.in_frame_info);
	err = sh_css_binary_find(&anr_descr,
				 &my_css.capture_settings.anr_binary);
	if (err != sh_css_success)
		return err;
	my_css.capture_settings.anr_binary.left_padding =
		my_css.capture_settings.post_isp_binary.left_padding;

	/* Pre-anr */
	init_pre_anr_descr(&pre_in_info,
			   &my_css.capture_settings.anr_binary.in_frame_info);
	err = sh_css_binary_find(&pre_anr_descr,
				 &my_css.capture_settings.pre_isp_binary);
	if (err != sh_css_success)
		return err;
	my_css.capture_settings.pre_isp_binary.left_padding =
		my_css.capture_settings.anr_binary.left_padding;

	/* Viewfinder post-processing */
	if (need_pp) {
		vf_pp_in_info =
		    &my_css.capture_settings.capture_pp_binary.vf_frame_info;
	} else {
		vf_pp_in_info =
		    &my_css.capture_settings.post_isp_binary.vf_frame_info;
	}

	init_vf_pp_descr(vf_pp_in_info, &my_css.capture_settings.vf_info);
	err = sh_css_binary_find(&vf_pp_descr,
				 &my_css.capture_settings.vf_pp_binary);
	if (err != sh_css_success)
		return err;

	/* Copy */
	err = load_copy_binary(&my_css.capture_settings.copy_binary,
			       &my_css.capture_settings.pre_isp_binary);
	if (err != sh_css_success)
		return err;

	if (need_pp)
		return alloc_capture_pp_frame(
				&my_css.capture_settings.capture_pp_binary);
	else
		return sh_css_success;
}
#endif

static bool
copy_on_sp(void)
{
	if (my_css.capture_settings.mode != SH_CSS_CAPTURE_MODE_RAW)
		return false;
	return my_css.continuous ||
		my_css.input_format == SH_CSS_INPUT_FORMAT_BINARY_8;
}

static enum sh_css_err
load_capture_binaries(void)
{
	enum sh_css_err err = sh_css_success;
	bool must_be_raw;

	/* in primary, advanced or low light, the input format must be raw */
	must_be_raw =
		my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_ADVANCED ||
		my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_LOW_LIGHT;
	err = check_input(must_be_raw);
	if (err != sh_css_success)
		return err;
	if (copy_on_sp()) {
		/* this is handled by the SP, no ISP binaries needed. */
		if (my_css.input_format == SH_CSS_INPUT_FORMAT_BINARY_8) {
			sh_css_frame_info_init(
				&my_css.capture_settings.output_info,
				JPEG_BYTES, 1, SH_CSS_FRAME_FORMAT_BINARY_8);
			return sh_css_success;
		}
	}

	switch (my_css.capture_settings.mode) {
	case SH_CSS_CAPTURE_MODE_RAW:
		return load_copy_binaries();
	case SH_CSS_CAPTURE_MODE_PRIMARY:
		return load_primary_binaries();
	case SH_CSS_CAPTURE_MODE_ADVANCED:
		return load_advanced_binaries();
	case SH_CSS_CAPTURE_MODE_LOW_LIGHT:
#ifndef SYSTEM_hive_isp_css_2400_system
		return load_low_light_binaries();
#else
		return sh_css_err_unsupported_configuration;
#endif
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_capture_start(struct sh_css_frame *raw_out_frame,
		     struct sh_css_frame *out_frame,
		     struct sh_css_frame *vf_frame)
{
	bool raw = my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW;
	struct sh_css_pipeline *me = &my_css.capture_settings.pipeline;
	struct sh_css_pipeline_stage *out_stage, *vf_pp_stage, *copy_stage;
	struct sh_css_binary *copy_binary,
			     *primary_binary,
			     *vf_pp_binary,
			     *pre_isp_binary,
			     *gdc_binary,
			     *post_isp_binary,
			     *anr_binary,
			     *capture_pp_binary,
			     *sc_binary = NULL;
	bool need_pp = false;
	enum sh_css_err err = sh_css_success;
	bool raw_copy = my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW
			&& copy_on_sp();

	if (!out_frame)
		return sh_css_err_invalid_arguments;
	copy_stage = NULL;

	if (my_css.invalidate || my_css.capture_settings.zoom_changed) {
		invalidate_capture_binaries();
		my_css.capture_settings.zoom_changed = false;
		my_css.invalidate = false;
	}

	err = load_capture_binaries();
	if (err != sh_css_success)
		return err;

	err = check_infos_match(&out_frame->info,
			&my_css.capture_settings.output_info);
	if (err != sh_css_success)
		return err;
	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW && vf_frame)
		return sh_css_err_mode_does_not_have_viewfinder;

	if (vf_frame) {
		err = check_infos_match(&vf_frame->info,
					&my_css.capture_settings.vf_info);
		if (err != sh_css_success)
			return err;
	}

	copy_binary       = &my_css.capture_settings.copy_binary;
	primary_binary    = &my_css.capture_settings.primary_binary;
	vf_pp_binary      = &my_css.capture_settings.vf_pp_binary;
	pre_isp_binary    = &my_css.capture_settings.pre_isp_binary;
	gdc_binary        = &my_css.capture_settings.gdc_binary;
	post_isp_binary   = &my_css.capture_settings.post_isp_binary;
	anr_binary        = &my_css.capture_settings.anr_binary;
	capture_pp_binary = &my_css.capture_settings.capture_pp_binary;
	need_pp = my_css.capture_settings.need_pp || my_css.output_stage;

	sh_css_metrics_start_frame();

	if (me->reload) {
		struct sh_css_pipeline_stage *post_stage;
		sh_css_pipeline_clean(me);
		if (my_css.capture_settings.copy_binary.info && !raw_copy) {
			err = sh_css_pipeline_add_stage(me, copy_binary, NULL,
					copy_binary->info->mode, NULL, NULL,
					raw ? out_frame : raw_out_frame,
					NULL, &post_stage);
			if (err != sh_css_success)
				return err;
		}
		if (my_css.capture_settings.mode ==
		    SH_CSS_CAPTURE_MODE_PRIMARY) {
			err = sh_css_pipeline_add_stage(me, primary_binary,
					NULL, primary_binary->info->mode,
					NULL, NULL,
					need_pp ? NULL : out_frame,
					NULL, &post_stage);
			if (err != sh_css_success)
				return err;
			/* If we use copy iso primary,
			   the input must be yuv iso raw */
			post_stage->args.copy_vf =
				primary_binary->info->mode ==
				SH_CSS_BINARY_MODE_COPY;
			post_stage->args.copy_output = post_stage->args.copy_vf;
			sc_binary = primary_binary;
		} else if (my_css.capture_settings.mode ==
			   SH_CSS_CAPTURE_MODE_ADVANCED) {
			err = sh_css_pipeline_add_stage(me, pre_isp_binary,
					NULL, pre_isp_binary->info->mode,
					NULL, NULL, NULL, NULL, NULL);
			if (err != sh_css_success)
				return err;
			err = sh_css_pipeline_add_stage(me, gdc_binary,
					NULL, gdc_binary->info->mode,
					NULL, NULL, NULL, NULL, NULL);
			if (err != sh_css_success)
				return err;
			err = sh_css_pipeline_add_stage(me, post_isp_binary,
					NULL, post_isp_binary->info->mode,
					NULL, NULL,
					need_pp ? NULL : out_frame,
					NULL, &post_stage);
			if (err != sh_css_success)
				return err;
			sc_binary = pre_isp_binary;
		} else if (my_css.capture_settings.mode ==
			   SH_CSS_CAPTURE_MODE_LOW_LIGHT) {
			err = sh_css_pipeline_add_stage(me, pre_isp_binary,
					NULL, pre_isp_binary->info->mode,
					NULL, NULL, NULL, NULL, NULL);
			if (err != sh_css_success)
				return err;
			err = sh_css_pipeline_add_stage(me, anr_binary,
					NULL, anr_binary->info->mode,
					NULL, NULL, NULL, NULL, NULL);
			if (err != sh_css_success)
				return err;
			err = sh_css_pipeline_add_stage(me, post_isp_binary,
					NULL, post_isp_binary->info->mode,
					NULL, NULL,
					need_pp ? NULL : out_frame,
					NULL, &post_stage);
			if (err != sh_css_success)
				return err;
			sc_binary = pre_isp_binary;
		}

		if (need_pp) {
			err = add_capture_pp_stage(me, out_frame,
						   capture_pp_binary,
						   post_stage, &post_stage);
			if (err != sh_css_success)
				return err;
		}
		if (my_css.capture_settings.mode != SH_CSS_CAPTURE_MODE_RAW) {
			err = add_vf_pp_stage(me, vf_frame, vf_pp_binary,
					      post_stage, &vf_pp_stage);
			if (err != sh_css_success)
				return err;
		}
		me->reload = false;
	} else {
		sh_css_pipeline_restart(me);
	}

	err = sh_css_config_input_network(me, copy_binary);
	if (err != sh_css_success)
		return err;
	if (!my_css.capture_settings.shading_table && sc_binary) {
		prepare_shading_table(
				&my_css.capture_settings.shading_table,
				sc_binary);
	}
	sh_css_params_set_shading_table(my_css.capture_settings.shading_table);

	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW) {
		if (copy_on_sp()) {
			my_css.mode = sh_css_mode_capture;
			my_css.state = sh_css_state_executing_sp_bin_copy;
			return start_copy_on_sp(copy_binary, out_frame);
		}
	} else {
		if (!vf_frame)
			return sh_css_err_invalid_arguments;
	}
	if (err != sh_css_success)
		return err;

	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW) {
		err = sh_css_pipeline_get_stage(me, copy_binary->info->mode,
						&out_stage);
		if (err != sh_css_success)
			return err;
		copy_stage = out_stage;
	} else {
		if (copy_binary->info) {
			err = sh_css_pipeline_get_stage(me,
							copy_binary->info->mode,
							&copy_stage);
			if (err != sh_css_success)
				return err;
		}
		if (capture_pp_binary->info) {
			err = sh_css_pipeline_get_stage(me,
					capture_pp_binary->info->mode,
					&out_stage);
			if (err != sh_css_success)
				return err;
		} else if (my_css.capture_settings.mode ==
			   SH_CSS_CAPTURE_MODE_PRIMARY) {
			err = sh_css_pipeline_get_stage(me,
					primary_binary->info->mode, &out_stage);
			if (err != sh_css_success)
				return err;
		} else if (my_css.capture_settings.mode ==
			   SH_CSS_CAPTURE_MODE_LOW_LIGHT) {
			err = sh_css_pipeline_get_stage(me,
					post_isp_binary->info->mode,
					&out_stage);
			if (err != sh_css_success)
				return err;
		} else {
			err = sh_css_pipeline_get_stage(me,
					post_isp_binary->info->mode,
					&out_stage);
			if (err != sh_css_success)
				return err;
		}
		err = sh_css_pipeline_get_output_stage(me,
						       vf_pp_binary->info->mode,
						       &vf_pp_stage);
		if (err != sh_css_success)
			return err;
	}
	if (my_css.capture_settings.mode != SH_CSS_CAPTURE_MODE_RAW)
		vf_pp_stage->args.out_frame = vf_frame;

	if (!my_css.output_stage)
		out_stage->args.out_frame = out_frame;

	if (copy_stage && raw_out_frame)
		copy_stage->args.out_frame = raw_out_frame;
	my_css.mode = sh_css_mode_capture;
	return sh_css_pipeline_start_next_stage(me);
}

enum sh_css_err
sh_css_capture_get_output_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;

	err = load_capture_binaries();
	if (err != sh_css_success)
		return err;

	if (copy_on_sp() &&
	    my_css.input_format == SH_CSS_INPUT_FORMAT_BINARY_8) {
		sh_css_frame_info_init(info, JPEG_BYTES, 1,
				       SH_CSS_FRAME_FORMAT_BINARY_8);
	} else {
		*info = my_css.capture_settings.output_info;
		if (info->format == SH_CSS_FRAME_FORMAT_RAW)
			info->raw_bit_depth =
				sh_css_input_format_bits_per_pixel(
					my_css.input_format,
					my_css.two_ppc);
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_capture_get_viewfinder_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;

	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW)
		return sh_css_err_mode_does_not_have_viewfinder;
	err = load_capture_binaries();
	if (err == sh_css_success)
		*info = my_css.capture_settings.vf_info;
	return err;
}

enum sh_css_err
sh_css_capture_get_output_raw_frame_info(struct sh_css_frame_info *info)
{
	enum sh_css_err err;

	if (my_css.capture_settings.online ||
	    copy_on_sp()) {
		return sh_css_err_mode_does_not_have_raw_output;
	}
	err = load_capture_binaries();
	if (err == sh_css_success)
		*info = my_css.capture_settings.copy_binary.out_frame_info;
	return err;
}

enum sh_css_err
sh_css_capture_get_grid_info(struct sh_css_grid_info *info)
{
	enum sh_css_err err;

	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW)
		return sh_css_err_mode_does_not_have_grid;
	err = load_capture_binaries();
	if (err != sh_css_success)
		return err;
	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_PRIMARY) {
		err = sh_css_binary_grid_info(
				&my_css.capture_settings.primary_binary, info);
	} else {
		err = sh_css_binary_grid_info(
				&my_css.capture_settings.pre_isp_binary, info);
	}
	return err;
}

enum sh_css_err
sh_css_capture_get_input_resolution(unsigned int *width, unsigned int *height)
{
	enum sh_css_err err;

	if (copy_on_sp() &&
	    my_css.input_format == SH_CSS_INPUT_FORMAT_BINARY_8) {
		*width = JPEG_BYTES;
		*height = 1;
		return sh_css_success;
	}

	err = load_capture_binaries();
	if (err == sh_css_success) {
		const struct sh_css_binary *binary;
		if (my_css.capture_settings.copy_binary.info)
			binary = &my_css.capture_settings.copy_binary;
		else
			binary = &my_css.capture_settings.primary_binary;
		*width  = binary->in_frame_info.width +
			  columns_needed_for_bayer_order();
		*height = binary->in_frame_info.height +
			  lines_needed_for_bayer_order();
	}
	return err;
}

void
sh_css_capture_set_mode(enum sh_css_capture_mode mode)
{
	if (mode != my_css.capture_settings.mode) {
		my_css.capture_settings.mode = mode;
		invalidate_capture_binaries();
	}
}

void
sh_css_capture_enable_xnr(bool enable)
{
	if (my_css.capture_settings.xnr != enable) {
		invalidate_capture_binaries();
		my_css.capture_settings.xnr = enable;
	}
}

void
sh_css_capture_enable_bayer_downscaling(bool enable)
{
	if (my_css.capture_settings.bayer_ds != enable) {
		invalidate_capture_binaries();
		my_css.capture_settings.bayer_ds = enable;
	}
}

enum sh_css_err
sh_css_capture_configure_output(unsigned int width,
				unsigned int height,
				enum sh_css_frame_format format)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.capture_settings.output_info.width != width ||
	    my_css.capture_settings.output_info.height != height ||
	    my_css.capture_settings.output_info.format != format) {
		sh_css_frame_info_init(&my_css.capture_settings.output_info,
				       width, height, format);
		invalidate_capture_binaries();
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_capture_configure_viewfinder(unsigned int width,
				    unsigned int height,
				    enum sh_css_frame_format format)
{
	enum sh_css_err err = sh_css_success;
	err = check_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.capture_settings.vf_info.width != width ||
	    my_css.capture_settings.vf_info.height != height ||
	    my_css.capture_settings.vf_info.format != format) {
		sh_css_frame_info_init(&my_css.capture_settings.vf_info,
				       width, height, format);
		invalidate_capture_binaries();
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_capture_configure_pp_input(unsigned int width,
				  unsigned int height)
{
	enum sh_css_err err = sh_css_success;
	err = check_null_res(width, height);
	if (err != sh_css_success)
		return err;
	if (my_css.capture_settings.pp_input_info.width != width ||
	    my_css.capture_settings.pp_input_info.height != height) {
		sh_css_frame_info_init(&my_css.capture_settings.pp_input_info,
				       width, height,
				       SH_CSS_FRAME_FORMAT_YUV420);
		invalidate_capture_binaries();
	}
	return sh_css_success;
}

void
sh_css_capture_enable_online(bool enable)
{
	my_css.capture_settings.online = enable;
}

bool
sh_css_capture_next_stage_needs_alloc(void)
{
	struct sh_css_binary *main_binary;

	main_binary = &my_css.capture_settings.primary_binary;
	if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_RAW)
		main_binary = &my_css.capture_settings.copy_binary;
	else if (my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_ADVANCED ||
		 my_css.capture_settings.mode == SH_CSS_CAPTURE_MODE_LOW_LIGHT)
		main_binary = &my_css.capture_settings.pre_isp_binary;

	return my_css.invalidate ||
	       my_css.capture_settings.pipeline.reload ||
	       my_css.capture_settings.zoom_changed ||
	       main_binary->info == NULL ||
	       my_css.capture_settings.shading_table == NULL;
}

enum sh_css_err
sh_css_histogram_allocate(unsigned int num_elements,
			  struct sh_css_histogram **histogram)
{
	struct sh_css_histogram *me = sh_css_malloc(sizeof(*me));

	if (me == NULL)
		return sh_css_err_cannot_allocate_memory;
	me->num_elements = num_elements;
	me->data = hrt_isp_css_mm_alloc(num_elements * sizeof(unsigned int));
	if (me->data == NULL) {
		sh_css_free(me);
		return sh_css_err_cannot_allocate_memory;
	}
	*histogram = me;
	return sh_css_success;
}

void
sh_css_histogram_free(struct sh_css_histogram *me)
{
	if (me->data)
		hrt_isp_css_mm_free(me->data);
	sh_css_free(me);
}

enum sh_css_err
sh_css_histogram_start(const struct sh_css_frame *input_frame,
		       struct sh_css_histogram *histogram)
{
	if (my_css.state != sh_css_state_idle)
		return sh_css_err_system_not_idle;
	if (input_frame->info.padded_width > SH_CSS_SP_MAX_WIDTH)
		return sh_css_err_overlay_frames_too_big;
	if (input_frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_frames_mismatch;
	sh_css_sp_start_histogram(histogram, input_frame);
	return sh_css_success;
}

void
sh_css_send_input_frame(unsigned short *data,
			unsigned int width,
			unsigned int height)
{
	sh_css_hrt_send_input_frame(data, width, height, my_css.ch_id,
				    my_css.input_format,
				    my_css.two_ppc);
}


void
sh_css_streaming_to_mipi_start_frame(unsigned int channel_id,
				enum sh_css_input_format input_format,
				bool two_pixels_per_clock)
{
	sh_css_hrt_streaming_to_mipi_start_frame(channel_id,
						input_format,
						two_pixels_per_clock);
}


void
sh_css_streaming_to_mipi_send_line(unsigned int channel_id,
					unsigned short *data,
					unsigned int width,
						unsigned short *data2,
						unsigned int width2)
{
	sh_css_hrt_streaming_to_mipi_send_line(channel_id,
						data, width,
						data2, width2);
}


void
sh_css_streaming_to_mipi_end_frame(unsigned int channel_id)
{
	sh_css_hrt_streaming_to_mipi_end_frame(channel_id);
}


static enum sh_css_err
allocate_frame_data(struct sh_css_frame *frame, unsigned int bytes)
{
#ifndef __KERNEL__
	/* Physically contiguous memory allocation only for FPGA. */
	if (frame->contiguous)
		frame->data = hrt_isp_css_mm_alloc_contiguous(bytes);
	else
#endif
		frame->data = hrt_isp_css_mm_alloc(bytes);
	if (frame->data == NULL)
		return sh_css_err_cannot_allocate_memory;
	frame->data_bytes = bytes;
	return sh_css_success;
}

static void
init_plane(struct sh_css_frame_plane *plane,
	   unsigned int width,
	   unsigned int stride,
	   unsigned int height,
	   void *data)
{
	plane->height = height;
	plane->width = width;
	plane->stride = stride;
	plane->data = data;
}

static enum sh_css_err
allocate_single_plane(struct sh_css_frame *frame,
		      struct sh_css_frame_plane *plane,
		      unsigned int height,
		      unsigned int subpixels_per_line,
		      unsigned int bytes_per_pixel)
{
	enum sh_css_err err;
	unsigned int stride;

	stride = subpixels_per_line * bytes_per_pixel;
	err = allocate_frame_data(frame, stride * height);
	if (err == sh_css_success) {
		init_plane(plane, subpixels_per_line, stride,
			   height, frame->data);
	}
	return err;
}

static enum sh_css_err
allocate_nv_planes(struct sh_css_frame *frame,
		   unsigned int horizontal_decimation,
		   unsigned int vertical_decimation)
{
	enum sh_css_err err;
	unsigned int y_width = frame->info.padded_width,
		     y_height = frame->info.height,
		     uv_width = 2 * (y_width / horizontal_decimation),
		     uv_height = y_height / vertical_decimation,
		     y_bytes, uv_bytes;

	y_bytes   = y_width * y_height;
	uv_bytes  = uv_width * uv_height;

	err = allocate_frame_data(frame, y_bytes + uv_bytes);
	if (err == sh_css_success) {
		init_plane(&frame->planes.nv.y, y_width, y_width,
			   y_height, frame->data);
		init_plane(&frame->planes.nv.uv, uv_width, uv_width,
			   uv_height, frame->data + y_bytes);
	}
	return err;
}

static enum sh_css_err
allocate_yuv_planes(struct sh_css_frame *frame,
		    unsigned int horizontal_decimation,
		    unsigned int vertical_decimation,
		    bool swap_uv,
		    unsigned int bytes_per_element)
{
	enum sh_css_err err;
	unsigned int y_width = frame->info.padded_width,
		     y_height = frame->info.height,
		     uv_width = y_width / horizontal_decimation,
		     uv_height = y_height / vertical_decimation,
		     y_stride, y_bytes, uv_bytes, uv_stride;

	y_stride  = y_width * bytes_per_element;
	uv_stride = uv_width * bytes_per_element;
	y_bytes   = y_stride * y_height;
	uv_bytes  = uv_stride * uv_height;

	err = allocate_frame_data(frame, y_bytes + 2 * uv_bytes);
	if (err == sh_css_success) {
		init_plane(&frame->planes.yuv.y, y_width, y_stride, y_height,
				frame->data);
		if (swap_uv) {
			init_plane(&frame->planes.yuv.v, uv_width, uv_stride,
				   uv_height, frame->data + y_bytes);
			init_plane(&frame->planes.yuv.u, uv_width, uv_stride,
				   uv_height, frame->data + y_bytes + uv_bytes);
		} else {
			init_plane(&frame->planes.yuv.u, uv_width, uv_stride,
				   uv_height, frame->data + y_bytes);
			init_plane(&frame->planes.yuv.v, uv_width, uv_stride,
				   uv_height, frame->data + y_bytes + uv_bytes);
		}
	}
	return err;
}

static enum sh_css_err
allocate_rgb_planes(struct sh_css_frame *frame, unsigned int bytes_per_element)
{
	enum sh_css_err err;
	unsigned int width = frame->info.width,
		     height = frame->info.height, stride, bytes;

	stride = width * bytes_per_element;
	bytes  = stride * height;
	err = allocate_frame_data(frame, 3 * bytes);
	if (err == sh_css_success) {
		init_plane(&frame->planes.planar_rgb.r, width, stride,
			   height, frame->data);
		init_plane(&frame->planes.planar_rgb.g, width, stride,
			   height, frame->data + 1 * bytes);
		init_plane(&frame->planes.planar_rgb.b, width, stride,
			   height, frame->data + 2 * bytes);
	}
	return err;
}

static enum sh_css_err
allocate_qplane6_planes(struct sh_css_frame *frame)
{
	enum sh_css_err err;
	unsigned int width = frame->info.padded_width / 2,
		     height = frame->info.height / 2,
		     bytes, stride;

	stride = width * 2;
	bytes  = stride * height;

	err = allocate_frame_data(frame, 6 * bytes);
	if (err == sh_css_success) {
		init_plane(&frame->planes.plane6.r, width, stride,
			   height, frame->data + 0 * bytes);
		init_plane(&frame->planes.plane6.r_at_b, width, stride,
			   height, frame->data + 1 * bytes);
		init_plane(&frame->planes.plane6.gr, width, stride,
			   height, frame->data + 2 * bytes);
		init_plane(&frame->planes.plane6.gb, width, stride,
			   height, frame->data + 3 * bytes);
		init_plane(&frame->planes.plane6.b, width, stride,
			   height, frame->data + 4 * bytes);
		init_plane(&frame->planes.plane6.b_at_r, width, stride,
			   height, frame->data + 5 * bytes);
	}
	return err;
}

static enum sh_css_err
allocate_frame(struct sh_css_frame **frame,
	       unsigned int width,
	       unsigned int height,
	       enum sh_css_frame_format format,
	       unsigned int padded_width,
	       unsigned int raw_bit_depth,
	       bool contiguous)
{
	enum sh_css_err err;
	struct sh_css_frame *me = sh_css_malloc(sizeof(*me));
	unsigned int bytes_per_pixel = raw_bit_depth <= 8 ? 1 : 2;

	if (!me)
		return sh_css_err_cannot_allocate_memory;
	me->info.width = width;
	me->info.height = height;
	me->info.format = format;
	me->info.padded_width = padded_width;
	me->info.raw_bit_depth = raw_bit_depth;
	me->contiguous = contiguous;

	switch (me->info.format) {
	case SH_CSS_FRAME_FORMAT_RAW:
		err = allocate_single_plane(me, &me->planes.raw,
					    me->info.height,
					    padded_width,
					    bytes_per_pixel);
		break;
	case SH_CSS_FRAME_FORMAT_RGB565:
		err = allocate_single_plane(me, &me->planes.rgb,
					    me->info.height,
					    padded_width, 2);
		break;
	case SH_CSS_FRAME_FORMAT_RGBA888:
		err = allocate_single_plane(me, &me->planes.rgb,
					    me->info.height,
					    padded_width * 4, 1);
		break;
	case SH_CSS_FRAME_FORMAT_PLANAR_RGB888:
		err = allocate_rgb_planes(me, 1);
		break;
		/* yuyv and uyvu have the same frame layout, only the data
		 * positioning differs.
		 */
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_UYVY:
		err = allocate_single_plane(me, &me->planes.yuyv,
					    me->info.height,
					    padded_width * 2, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
		/* Needs 3 extra lines to allow vf_pp prefetching */
		err = allocate_single_plane(me, &me->planes.yuyv,
					    me->info.height * 3/2 + 3,
					    padded_width, 1);
		break;
	case SH_CSS_FRAME_FORMAT_NV11:
		err = allocate_nv_planes(me, 4, 1);
		break;
		/* nv12 and nv21 have the same frame layout, only the data
		 * positioning differs.
		 */
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV21:
		err = allocate_nv_planes(me, 2, 2);
		break;
		/* nv16 and nv61 have the same frame layout, only the data
		 * positioning differs.
		 */
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV61:
		err = allocate_nv_planes(me, 2, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YUV420:
		err = allocate_yuv_planes(me, 2, 2, false, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YUV422:
		err = allocate_yuv_planes(me, 2, 1, false, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YUV444:
		err = allocate_yuv_planes(me, 1, 1, false, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YUV420_16:
		err = allocate_yuv_planes(me, 2, 2, false, 2);
		break;
	case SH_CSS_FRAME_FORMAT_YUV422_16:
		err = allocate_yuv_planes(me, 2, 1, false, 2);
		break;
	case SH_CSS_FRAME_FORMAT_YV12:
		err = allocate_yuv_planes(me, 2, 2, true, 1);
		break;
	case SH_CSS_FRAME_FORMAT_YV16:
		err = allocate_yuv_planes(me, 2, 1, true, 1);
		break;
	case SH_CSS_FRAME_FORMAT_QPLANE6:
		err = allocate_qplane6_planes(me);
		break;
	case SH_CSS_FRAME_FORMAT_BINARY_8:
		err = allocate_single_plane(me, &me->planes.binary.data,
					    me->info.height,
					    padded_width, 1);
		me->planes.binary.size = 0;
		break;
	default:
		sh_css_free(me);
		return sh_css_err_invalid_frame_format;
	}
	if (err == sh_css_success)
		*frame = me;
	return err;
}

enum sh_css_err
sh_css_frame_allocate(struct sh_css_frame **frame,
		      unsigned int width,
		      unsigned int height,
		      enum sh_css_frame_format format,
		      unsigned int padded_width,
		      unsigned int raw_bit_depth)
{
	return allocate_frame(frame, width, height, format,
			      padded_width, raw_bit_depth, false);
}

enum sh_css_err
sh_css_frame_allocate_from_info(struct sh_css_frame **frame,
				const struct sh_css_frame_info *info)
{
	return sh_css_frame_allocate(frame,
				     info->width,
				     info->height,
				     info->format,
				     info->padded_width,
				     info->raw_bit_depth);
}

enum sh_css_err
sh_css_frame_allocate_contiguous(struct sh_css_frame **frame,
				 unsigned int width,
				 unsigned int height,
				 enum sh_css_frame_format format,
				 unsigned int padded_width,
				 unsigned int raw_bit_depth)
{
	return allocate_frame(frame, width, height, format, padded_width,
			      raw_bit_depth, true);
}

enum sh_css_err
sh_css_frame_allocate_contiguous_from_info(struct sh_css_frame **frame,
					   const struct sh_css_frame_info
						*info)
{
	return sh_css_frame_allocate_contiguous(frame,
						info->width,
						info->height,
						info->format,
						info->padded_width,
						info->raw_bit_depth);
}

void
sh_css_frame_free(struct sh_css_frame *frame)
{
	if (frame) {
		hrt_isp_css_mm_free(frame->data);
		sh_css_free(frame);
	}
}

bool
sh_css_frame_info_equal_resolution(const struct sh_css_frame_info *info_a,
				   const struct sh_css_frame_info *info_b)
{
	if (!info_a || !info_b)
		return false;
	return (info_a->width == info_b->width) &&
	    (info_a->height == info_b->height);
}

bool
sh_css_frame_equal_types(const struct sh_css_frame *frame_a,
			 const struct sh_css_frame *frame_b)
{
	const struct sh_css_frame_info *info_a = &frame_a->info,
	    *info_b = &frame_b->info;
	if (!info_a || !info_b)
		return false;
	if (info_a->format != info_b->format)
		return false;
	if (info_a->padded_width != info_b->padded_width)
		return false;
	return sh_css_frame_info_equal_resolution(info_a, info_b);
}

static void
append_firmware(struct sh_css_acc_fw **l, struct sh_css_acc_fw *firmware)
{
	while (*l)
		l = &(*l)->header.next;
	*l = firmware;
	firmware->header.next = NULL;
}

static void
remove_firmware(struct sh_css_acc_fw **l, struct sh_css_acc_fw *firmware)
{
	while (*l && *l != firmware)
		l = &(*l)->header.next;
	if (!*l)
		return;
	*l = firmware->header.next;
	firmware->header.next = NULL;
}

/* Load firmware for acceleration */
enum sh_css_err
sh_css_load_acceleration(struct sh_css_acc_fw *firmware)
{
	my_css.invalidate = true;
	if (firmware->header.type == SH_CSS_ACC_OUTPUT)
		append_firmware(&my_css.output_stage, firmware);
	else if (firmware->header.type == SH_CSS_ACC_VIEWFINDER)
		append_firmware(&my_css.vf_stage, firmware);
	return sh_css_acc_load(firmware);
}

/* Unload firmware for acceleration */
void
sh_css_unload_acceleration(struct sh_css_acc_fw *firmware)
{
	my_css.invalidate = true;
	if (firmware->header.type == SH_CSS_ACC_OUTPUT)
		remove_firmware(&my_css.output_stage, firmware);
	else if (firmware->header.type == SH_CSS_ACC_VIEWFINDER)
		remove_firmware(&my_css.vf_stage, firmware);
	my_css.standalone_acc = NULL;
}

/* Set argument <num> of size <size> to value <val> */
enum sh_css_err
sh_css_set_acceleration_argument(struct sh_css_acc_fw *firmware,
				 unsigned num, void *val, size_t size)
{
	return sh_css_acc_set_argument(firmware, num, val, size);
}

/* Start acceleration of firmware with sp-args as SP arguments. */
enum sh_css_err
sh_css_start_acceleration(struct sh_css_acc_fw *firmware)
{
	my_css.standalone_acc = firmware;
	return sh_css_acc_start(firmware, NULL);
}

/* To be called when acceleration has terminated.
*/
void
sh_css_acceleration_done(struct sh_css_acc_fw *firmware)
{
	my_css.standalone_acc = NULL;
	sh_css_acc_done(firmware);
}

/* Abort acceleration within <deadline> microseconds
*/
void
sh_css_abort_acceleration(struct sh_css_acc_fw *firmware, unsigned deadline)
{
	/* TODO: implement time-out */
	(void)deadline;
	sh_css_acc_abort(firmware);
}
