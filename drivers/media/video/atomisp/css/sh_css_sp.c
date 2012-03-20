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

#include "sh_css.h"
#include "sh_css_binary.h"
#include "sh_css_sp.h"
#include "sh_css_sp_start.h"
#include "sh_css_hrt.h"
#include "sh_css_defs.h"
#include "sh_css_internal.h"
#include "sh_css_hw.h"
#define HRT_NO_BLOB_sp
#include "sp.map.h"

/* Convenience macro to force a value to a lower even value.
 *  We do not want to (re)use the kernel macro round_down here
 *  because the same code base is used internally by Silicon Hive
 *  simulation environment, where the kernel macro is not available
 */
#define EVEN_FLOOR(x)	(x & ~1)

struct sh_css_sp_group		sh_css_sp_group;
static struct sh_css_sp_per_frame_data per_frame_data;

/* This data is stored every frame */
void
sh_css_store_sp_per_frame_data(void)
{
	per_frame_data.sp_group_addr = sh_css_store_sp_group_to_ddr();
	per_frame_data.read_sp_group_from_ddr = true;

	store_sp_var(sp_per_frame_data, &per_frame_data,
			sizeof(per_frame_data));
	/* Clear for next frame */
	memset(&per_frame_data, 0, sizeof(per_frame_data));
}

static void *init_dmem_ddr;

/* Initialize the entire contents of the DMEM at once -- does not need to
 * do this from the host
 */
void
sh_css_sp_store_init_dmem(const struct sh_css_sp_fw *fw)
{
	struct sh_css_sp_init_dmem_cfg init_dmem_cfg;

	hrt_isp_css_mm_store(init_dmem_ddr, fw->data, fw->data_size);

	/* Configure the data structure to initialize dmem */
	init_dmem_cfg.done	     = false;
	init_dmem_cfg.ddr_data_addr  = init_dmem_ddr;
	init_dmem_cfg.dmem_data_addr = (void *) fw->data_target;
	init_dmem_cfg.data_size      = fw->data_size;
	init_dmem_cfg.dmem_bss_addr  = (void *) fw->bss_target;
	init_dmem_cfg.bss_size       = fw->bss_size;

	sh_css_sp_dmem_store((unsigned)fw->dmem_init_data, &init_dmem_cfg,
				sizeof(init_dmem_cfg));

}

enum sh_css_err
sh_css_sp_init(void)
{
	init_dmem_ddr = hrt_isp_css_mm_alloc(SP_DMEM_SIZE);
	if (!init_dmem_ddr)
		return sh_css_err_cannot_allocate_memory;

	return sh_css_success;
}

void
sh_css_sp_uninit(void)
{
	if (init_dmem_ddr) {
		hrt_isp_css_mm_free(init_dmem_ddr);
		init_dmem_ddr = NULL;
	}
}

void
sh_css_sp_start_histogram(struct sh_css_histogram *histogram,
			  const struct sh_css_frame *frame)
{
	sh_css_sp_group.histo_addr = histogram->data;
	sh_css_sp_group.sp_in_frame = *frame;
	sh_css_sp_group.dma_proxy.busy_wait = false;

	sh_css_store_sp_per_frame_data();
	sh_css_hrt_sp_start_histogram();
}

void
sh_css_sp_get_debug_state(struct sh_css_sp_debug_state *state)
{
	int i;

	state->error = load_sp_uint(sp_error);

	for (i = 0; i < 16; i++)
		state->debug[i] = load_sp_array_uint(sp_debug, i);
}

void
sh_css_sp_start_binary_copy(struct sh_css_frame *out_frame,
			    unsigned two_ppc)
{
	sh_css_sp_group.bin_copy.out = out_frame->planes.binary.data.data;
	sh_css_sp_group.bin_copy.bytes_available = out_frame->data_bytes;
	sh_css_sp_group.isp_2ppc = two_ppc;
	sh_css_sp_group.dma_proxy.busy_wait = false;

	sh_css_store_sp_per_frame_data();
	sh_css_hrt_sp_start_copy_binary_data();
}

void
sh_css_sp_start_raw_copy(struct sh_css_binary *binary,
			 struct sh_css_frame *out_frame,
			 unsigned two_ppc,
			 bool start)
{
	sh_css_sp_group.raw_copy.out	= out_frame->planes.raw.data;
	sh_css_sp_group.raw_copy.height	= out_frame->info.height;
	sh_css_sp_group.raw_copy.width	= out_frame->info.width;
	sh_css_sp_group.raw_copy.padded_width  = out_frame->info.padded_width;
	sh_css_sp_group.raw_copy.raw_bit_depth = out_frame->info.raw_bit_depth;
	sh_css_sp_group.raw_copy.max_input_width =
		     binary->info->max_input_width;
	sh_css_sp_group.dma_proxy.busy_wait	= true;
	sh_css_sp_group.isp_2ppc = two_ppc;
	sh_css_sp_group.xmem_bin_addr = binary->info->xmem_addr;

	if (start) {
		sh_css_store_sp_per_frame_data();
		sh_css_hrt_sp_start_copy_raw_data();
	}
}

unsigned int
sh_css_sp_get_binary_copy_size(void)
{
	return load_sp_uint(sp_bin_copy_bytes_copied);
}

unsigned int
sh_css_sp_get_sw_interrupt_value(void)
{
	return load_sp_uint(sp_sw_interrupt_value);
}

static enum sh_css_err
set_input_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	switch (frame->info.format) {
	case SH_CSS_FRAME_FORMAT_QPLANE6:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_RAW:
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
		break;
	default:
		return sh_css_err_unsupported_frame_format;
	}
	sh_css_sp_group.sp_in_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_output_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	switch (frame->info.format) {
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV422:
	case SH_CSS_FRAME_FORMAT_YUV444:
	case SH_CSS_FRAME_FORMAT_YV12:
	case SH_CSS_FRAME_FORMAT_YV16:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_YUV422_16:
	case SH_CSS_FRAME_FORMAT_NV11:
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV21:
	case SH_CSS_FRAME_FORMAT_NV61:
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_UYVY:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
	case SH_CSS_FRAME_FORMAT_RGB565:
	case SH_CSS_FRAME_FORMAT_RGBA888:
	case SH_CSS_FRAME_FORMAT_PLANAR_RGB888:
	case SH_CSS_FRAME_FORMAT_RAW:
	case SH_CSS_FRAME_FORMAT_QPLANE6:
		break;
	default:
		return sh_css_err_unsupported_frame_format;
	}
	sh_css_sp_group.sp_out_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_ref_in_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_ref_in_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_ref_out_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_ref_out_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_tnr_in_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_tnr_in_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_tnr_out_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_tnr_out_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_capture_pp_frame_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_extra_frame = *frame;
	return sh_css_success;
}

static enum sh_css_err
set_view_finder_buffer(const struct sh_css_frame *frame)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV_LINE)
		return sh_css_err_unsupported_frame_format;
	sh_css_sp_group.sp_out_vf_frame = *frame;
	return sh_css_success;
}

static void
sh_css_sp_configure_cropping(const struct sh_css_binary *binary)
{
	sh_css_sp_group.sp_uds_xc = binary->in_frame_info.width / 2;
	sh_css_sp_group.sp_uds_yc = binary->in_frame_info.height / 2;
	sh_css_sp_group.sp_out_crop_pos_x = binary->info->left_cropping;
	sh_css_sp_group.sp_out_crop_pos_y = binary->info->top_cropping;
}

static void
sh_css_sp_configure_dvs(const struct sh_css_binary *binary,
			const struct sh_css_binary_args *args)
{
	unsigned int crop_x = 0,
		     crop_y = 0,
		     uds_xc = 0,
		     uds_yc = 0,
		     env_width, env_height;
	int half_env_x, half_env_y,
	    motion_x = args->dvs_vector_x,
	    motion_y = args->dvs_vector_y;

	if (binary->info->enable_uds) {
		/* we calculate with the envelope that we can actually use,
		   the min dvs envelope is for the filter initialization. */
		env_width  = binary->dvs_envelope_width -
				SH_CSS_MIN_DVS_ENVELOPE;
		env_height = binary->dvs_envelope_height -
				SH_CSS_MIN_DVS_ENVELOPE;
		half_env_x = env_width / 2;
		half_env_y = env_height / 2;
		/* for digital zoom, we use the dvs envelope and make sure
		   that we don't include the 8 leftmost pixels or 8 topmost
		   rows. */
		uds_xc = (binary->out_frame_info.width + env_width) / 2 +
			 SH_CSS_MIN_DVS_ENVELOPE;
		uds_yc = (binary->out_frame_info.height + env_height) / 2 +
			 SH_CSS_MIN_DVS_ENVELOPE;
		/* clip the motion vector to +/- half the envelope */
		motion_x = clamp(motion_x, -half_env_x, half_env_x);
		motion_y = clamp(motion_y, -half_env_y, half_env_y);
		uds_xc += motion_x;
		uds_yc += motion_y;
		/* uds can be pipelined, remove top lines */
		crop_y = 2;
	} else if (binary->info->enable_ds) {
		env_width  = binary->dvs_envelope_width;
		env_height = binary->dvs_envelope_height;
		half_env_x = env_width / 2;
		half_env_y = env_height / 2;
		/* clip the motion vector to +/- half the envelope */
		motion_x = clamp(motion_x, -half_env_x, half_env_x);
		motion_y = clamp(motion_y, -half_env_y, half_env_y);
		/* for video with downscaling, the envelope is included in
		   the input resolution. */
		uds_xc = binary->in_frame_info.width/2 + motion_x;
		uds_yc = binary->in_frame_info.height/2 + motion_y;
		crop_x = binary->info->left_cropping;
		crop_y = binary->info->top_cropping;
	} else {
		/* video nodz: here we can only crop. We make sure we crop at
		   least the first 8x8 pixels away. */
		env_width  = binary->dvs_envelope_width -
				SH_CSS_MIN_DVS_ENVELOPE;
		env_height = binary->dvs_envelope_height -
				SH_CSS_MIN_DVS_ENVELOPE;
		half_env_x = env_width / 2;
		half_env_y = env_height / 2;
		motion_x = clamp(motion_x, -half_env_x, half_env_x);
		motion_y = clamp(motion_y, -half_env_y, half_env_y);
		crop_x = SH_CSS_MIN_DVS_ENVELOPE + half_env_x + motion_x;
		crop_y = SH_CSS_MIN_DVS_ENVELOPE + half_env_y + motion_y;
	}

	/* Must enforce that the crop position is even */
	crop_x = EVEN_FLOOR(crop_x);
	crop_y = EVEN_FLOOR(crop_y);
	uds_xc = EVEN_FLOOR(uds_xc);
	uds_yc = EVEN_FLOOR(uds_yc);

	sh_css_sp_group.sp_uds_xc = uds_xc;
	sh_css_sp_group.sp_uds_yc = uds_yc;
	sh_css_sp_group.sp_out_crop_pos_x = crop_x;
	sh_css_sp_group.sp_out_crop_pos_y = crop_y;
}

static void
sh_css_sp_set_vf_zoom_position(const struct sh_css_binary *binary, bool zoom)
{
	/* for down scaling, we always use the center of the image */
	unsigned uds_xc = 0;
	unsigned uds_yc = 0;
	if (zoom) {
		uds_xc = (binary->in_frame_info.width) / 2;
		uds_yc = (binary->in_frame_info.height) / 2;
	}
	sh_css_sp_group.sp_uds_xc = uds_xc;
	sh_css_sp_group.sp_uds_yc = uds_yc;
	sh_css_sp_group.sp_out_crop_pos_x = binary->info->left_cropping;
	sh_css_sp_group.sp_out_crop_pos_y = binary->info->top_cropping;
}

void
sh_css_sp_set_if_configs(const struct sh_css_if_config *config_a,
			 const struct sh_css_if_config *config_b)
{
	bool block_a = config_a->block_no_reqs,
	     block_b = false;
	if (config_b)
		block_b = config_b->block_no_reqs;
	sh_css_hrt_if_reset();
	sh_css_hrt_if_set_block_fifo_no_reqs(block_a, block_b);
	sh_css_sp_group.if_config_a = *config_a;
	per_frame_data.if_a_changed = true;

	if (config_b) {
		sh_css_sp_group.if_config_b = *config_b;
		per_frame_data.if_b_changed = true;
	}
}

void
sh_css_sp_program_input_circuit(int fmt_type,
				int ch_id,
				enum sh_css_input_mode input_mode)
{
	sh_css_sp_group.input_circuit.no_side_band = false;
	sh_css_sp_group.input_circuit.fmt_type     = fmt_type;
	sh_css_sp_group.input_circuit.ch_id	   = ch_id;
	sh_css_sp_group.input_circuit.input_mode   = input_mode;
	per_frame_data.program_input_circuit = true;
}

void
sh_css_sp_configure_sync_gen(int width, int height,
			     int hblank_cycles,
			     int vblank_cycles)
{
	sh_css_sp_group.sync_gen.width	       = width;
	sh_css_sp_group.sync_gen.height	       = height;
	sh_css_sp_group.sync_gen.hblank_cycles = hblank_cycles;
	sh_css_sp_group.sync_gen.vblank_cycles = vblank_cycles;
}

void
sh_css_sp_configure_tpg(int x_mask,
			int y_mask,
			int x_delta,
			int y_delta,
			int xy_mask)
{
	sh_css_sp_group.tpg.x_mask  = x_mask;
	sh_css_sp_group.tpg.y_mask  = y_mask;
	sh_css_sp_group.tpg.x_delta = x_delta;
	sh_css_sp_group.tpg.y_delta = y_delta;
	sh_css_sp_group.tpg.xy_mask = xy_mask;
}

void
sh_css_sp_configure_prbs(int seed)
{
	sh_css_sp_group.prbs.seed = seed;
}

static enum sh_css_err
sh_css_sp_write_frame_pointers(const struct sh_css_binary_args *args)
{
	enum sh_css_err err = sh_css_success;
	if (args->in_frame)
		err = set_input_frame_buffer(args->in_frame);
	if (err == sh_css_success && args->in_ref_frame)
		err = set_ref_in_frame_buffer(args->in_ref_frame);
	if (err == sh_css_success && args->in_tnr_frame)
		err = set_tnr_in_frame_buffer(args->in_tnr_frame);
	if (err == sh_css_success && args->out_vf_frame)
		err = set_view_finder_buffer(args->out_vf_frame);
	if (err == sh_css_success && args->extra_frame)
		err = set_capture_pp_frame_buffer(args->extra_frame);
	if (err == sh_css_success && args->out_ref_frame)
		err = set_ref_out_frame_buffer(args->out_ref_frame);
	if (err == sh_css_success && args->out_tnr_frame)
		err = set_tnr_out_frame_buffer(args->out_tnr_frame);
	if (err == sh_css_success && args->out_frame)
		err = set_output_frame_buffer(args->out_frame);
	return err;
}

static void
sh_css_sp_compute_overlay(const struct sh_css_overlay *overlay,
			  struct sh_css_sp_overlay *sp_overlay)
{
	if (overlay == NULL) {
		sp_overlay->overlay_width  = 0;
		sp_overlay->overlay_height = 0;
	} else {
		int blend_shift,
		    ratio100,
		    blend_input_y,
		    blend_input_u,
		    blend_input_v,
		    blend_overlay_y, blend_overlay_u, blend_overlay_v;

		blend_shift = 12;
		ratio100 = 1 << blend_shift;
		blend_input_y =
		    (ratio100 * overlay->blend_input_perc_y + 50) / 100;
		blend_input_u =
		    (ratio100 * overlay->blend_input_perc_u + 50) / 100;
		blend_input_v =
		    (ratio100 * overlay->blend_input_perc_v + 50) / 100;
		blend_overlay_y =
		    (ratio100 * overlay->blend_overlay_perc_y + 50) / 100;
		blend_overlay_u =
		    (ratio100 * overlay->blend_overlay_perc_u + 50) / 100;
		blend_overlay_v =
		    (ratio100 * overlay->blend_overlay_perc_v + 50) / 100;
		sp_overlay->bg_y = (int) overlay->bg_y;
		sp_overlay->bg_u = (int) overlay->bg_u;
		sp_overlay->bg_v = (int) overlay->bg_v;
		sp_overlay->blend_shift = blend_shift;
		sp_overlay->blend_input_y = blend_input_y;
		sp_overlay->blend_input_u = blend_input_u;
		sp_overlay->blend_input_v = blend_input_v;
		sp_overlay->blend_overlay_y = blend_overlay_y;
		sp_overlay->blend_overlay_u = blend_overlay_u;
		sp_overlay->blend_overlay_v = blend_overlay_v;
		sp_overlay->overlay_width =
			(int) overlay->frame->info.width;
		sp_overlay->overlay_height =
			(int) overlay->frame->info.height;
		sp_overlay->overlay_start_x =
			(int) overlay->overlay_start_x;
		sp_overlay->overlay_start_y =
			(int) overlay->overlay_start_y;
		sp_overlay->frame_ptr_overlay_y =
			overlay->frame->planes.yuv.y.data;
		sp_overlay->frame_ptr_overlay_u =
			overlay->frame->planes.yuv.u.data;
		sp_overlay->frame_ptr_overlay_v =
			overlay->frame->planes.yuv.v.data;
	}
}

void
sh_css_sp_set_overlay(const struct sh_css_overlay *overlay)
{
	sh_css_sp_compute_overlay(overlay, &sh_css_sp_group.overlay);
}

enum sh_css_err
sh_css_sp_start_isp(struct sh_css_binary *binary,
		    const struct sh_css_binary_args *args,
		    bool preview_mode,
		    bool low_light)
{
	unsigned int dx, dy;
	enum sh_css_err err = sh_css_success;
	bool start_copy = sh_css_continuous_start_sp_copy();

	if (binary->info->mode == SH_CSS_BINARY_MODE_VF_PP) {
		bool zoom = false;
		if (preview_mode) {
			sh_css_get_zoom_factor(&dx, &dy);
			zoom = ((dx != UDS_SCALING_N) || (dy != UDS_SCALING_N));
		} else {
			/* in non-preview modes, VF_PP does not do
			   the zooming, capture_pp or video do. */
			dx = UDS_SCALING_N;
			dy = UDS_SCALING_N;
		}
		sh_css_sp_set_vf_zoom_position(binary, zoom);
	} else {
		sh_css_get_zoom_factor(&dx, &dy);
	}

 /* Copy the frame infos first, to be overwritten by the frames,
	   if these are present.
	*/
	sh_css_sp_group.sp_in_frame.info = binary->in_frame_info;
	sh_css_sp_group.sp_out_frame.info = binary->out_frame_info;
	sh_css_sp_group.sp_internal_frame_info = binary->internal_frame_info;

	sh_css_sp_group.sp_isp_binary_id         = binary->info->id;
	sh_css_sp_group.sp_enable_xnr            = args->enable_xnr;
	sh_css_sp_group.sp_uds_curr_dx           = dx;
	sh_css_sp_group.sp_uds_curr_dy           = dy;
	sh_css_sp_group.sp_input_stream_format   = binary->input_format;
	sh_css_sp_group.isp_dvs_envelope_width   = binary->dvs_envelope_width;
	sh_css_sp_group.isp_dvs_envelope_height  = binary->dvs_envelope_height;
	sh_css_sp_group.isp_deci_log_factor      = binary->deci_factor_log2;
	sh_css_sp_group.isp_vf_downscale_bits    = binary->vf_downscale_log2;
	sh_css_sp_group.isp_online               = binary->online;
	sh_css_sp_group.isp_copy_vf              = args->copy_vf;
	sh_css_sp_group.isp_copy_output          = args->copy_output;
	sh_css_sp_group.isp_2ppc                 = args->two_ppc;
	sh_css_sp_group.sp_run_copy              = start_copy;
	sh_css_sp_group.xmem_bin_addr            = binary->info->xmem_addr;
	sh_css_sp_group.xmem_map_addr            =
					sh_css_params_ddr_address_map();
	sh_css_sp_group.anr			 = low_light;
	sh_css_sp_group.dma_proxy.busy_wait	 = start_copy;

	store_sp_int(sp_isp_started, 0);

	if (binary->info->enable_dvs_envelope)
		sh_css_sp_configure_dvs(binary, args);
	else if (binary->info->mode != SH_CSS_BINARY_MODE_VF_PP)
		sh_css_sp_configure_cropping(binary);
	sh_css_params_set_current_binary(binary);
	err = sh_css_params_write_to_ddr(binary);
	if (err != sh_css_success)
		return err;
	err = sh_css_sp_write_frame_pointers(args);
	if (err != sh_css_success)
		return err;

	sh_css_store_sp_per_frame_data();

	sh_css_hrt_sp_start_isp();
	return err;
}

bool
sh_css_isp_has_started(void)
{
	return load_sp_uint(sp_isp_started);
}
