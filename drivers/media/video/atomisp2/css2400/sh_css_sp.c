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

#include "sh_css_sp.h"

#include "input_formatter.h"

#include "dma.h"	/* N_DMA_CHANNEL_ID */

#include "sh_css.h"
#include "sh_css_binary.h"
#include "sh_css_sp_start.h"
#include "sh_css_hrt.h"
#include "sh_css_defs.h"
#include "sh_css_internal.h"
#include "sh_css_debug.h"
#include "sh_css_debug_internal.h"

#include "gdc_device.h"				/* HRT_GDC_N */

/*#include "sp.h"*/	/* host2sp_enqueue_frame_data() */

#include "memory_access.h"

#include "assert_support.h"
#include "platform_support.h"	/* hrt_sleep() */

#include "queue.h"	/* host_sp_enqueue_XXX */
#include "sw_event.h"	/* encode_sw_event */
#include "mmu_local.h"

#ifndef offsetof
#define offsetof(T, x) ((unsigned)&(((T *)0)->x))
#endif

struct sh_css_sp_group		sh_css_sp_group;
struct sh_css_sp_stage		sh_css_sp_stage;
struct sh_css_sp_output		sh_css_sp_output;
static struct sh_css_sp_per_frame_data per_frame_data;

static hrt_vaddress init_dmem_ddr;

/* true if SP supports frame loop and host2sp_commands */
/* For the moment there is only code that sets this bool to true */
/* TODO: add code that sets this bool to false */
static bool sp_running;

static enum sh_css_err
set_output_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num);

/* This data is stored every frame */
void
store_sp_group_data(void)
{
	per_frame_data.sp_group_addr = sh_css_store_sp_group_to_ddr();
}

void
store_sp_stage_data(enum sh_css_pipe_id id, unsigned stage)
{
	enum sh_css_pipe_id pipe_id = id;
	unsigned int thread_id;
	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	sh_css_sp_group.pipe[thread_id].sp_stage_addr[stage] =
		sh_css_store_sp_stage_to_ddr(pipe_id, stage);

	/* Clear for next frame */
	sh_css_sp_stage.program_input_circuit = false;
}

static void
store_sp_per_frame_data(const struct sh_css_fw_info *fw)
{
	unsigned int HIVE_ADDR_sp_per_frame_data = 0;
	switch (fw->type) {
	case sh_css_sp_firmware:
		HIVE_ADDR_sp_per_frame_data = fw->info.sp.per_frame_data;
		break;
	case sh_css_acc_firmware:
		HIVE_ADDR_sp_per_frame_data = fw->info.acc.per_frame_data;
		break;
	case sh_css_isp_firmware:
		return;
	}

	sp_dmem_store(SP0_ID,
		(unsigned int)sp_address_of(sp_per_frame_data),
		&per_frame_data,
			sizeof(per_frame_data));
}

static void
sh_css_store_sp_per_frame_data(enum sh_css_pipe_id pipe_id,
			       const struct sh_css_fw_info *sp_fw)
{
	if (!sp_fw)
		sp_fw = &sh_css_sp_fw;

	store_sp_stage_data(pipe_id, 0);
	store_sp_group_data();
	store_sp_per_frame_data(sp_fw);
}

/* Initialize the entire contents of the DMEM at once -- does not need to
 * do this from the host
 */
void
sh_css_sp_store_init_dmem(const struct sh_css_fw_info *fw)
{
	struct sh_css_sp_init_dmem_cfg init_dmem_cfg;

	mmgr_store(init_dmem_ddr, fw->blob.data, fw->blob.data_size);

	/* Configure the data structure to initialize dmem */
	init_dmem_cfg.done	     = false;
	init_dmem_cfg.ddr_data_addr  = init_dmem_ddr;
	init_dmem_cfg.dmem_data_addr = (hrt_vaddress)fw->blob.data_target;
	init_dmem_cfg.data_size      = fw->blob.data_size;
	init_dmem_cfg.dmem_bss_addr  = (hrt_vaddress)fw->blob.bss_target;
	init_dmem_cfg.bss_size       = fw->blob.bss_size;

	sp_dmem_store(SP0_ID, (unsigned)fw->info.sp.init_dmem_data,
				&init_dmem_cfg,
				sizeof(init_dmem_cfg));

}

enum sh_css_err
sh_css_sp_init(void)
{
	init_dmem_ddr = mmgr_malloc(SP_DMEM_SIZE);
	if (init_dmem_ddr == mmgr_NULL)
		return sh_css_err_cannot_allocate_memory;

	return sh_css_success;
}

void
sh_css_sp_uninit(void)
{
	mmgr_free(init_dmem_ddr);
	init_dmem_ddr = mmgr_NULL;
}

void
sh_css_sp_get_debug_state(struct sh_css_sp_debug_state *state)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_output = fw->info.sp.output;
	unsigned i;
	unsigned o = offsetof(struct sh_css_sp_output, debug)/sizeof(int);
	(void)HIVE_ADDR_sp_output; /* To get rid of warning in CRUN */
	for (i = 0; i < sizeof(*state)/sizeof(int); i++)
		((unsigned *)state)[i] = load_sp_array_uint(sp_output, i+o);
}

void
sh_css_sp_start_binary_copy(struct sh_css_frame *out_frame,
			    unsigned two_ppc)
{
	enum sh_css_pipe_id pipe_id;
	unsigned int thread_id;
	struct sh_css_sp_pipeline *pipe;
	unsigned stage_num = 0;

assert(out_frame != NULL);
	pipe_id = SH_CSS_CAPTURE_PIPELINE;
	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	pipe = &sh_css_sp_group.pipe[thread_id];

	pipe->copy.bin.bytes_available = out_frame->data_bytes;
	pipe->num_stages = 1;
	pipe->pipe_id = pipe_id;
	/* TODO: next indicates from which queues parameters need to be
		 sampled, needs checking/improvement */
	pipe->pipe_config = SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS << thread_id;

	sh_css_sp_group.config.input_formatter.isp_2ppc = two_ppc;

	sh_css_sp_stage.num = stage_num;
	sh_css_sp_stage.irq_buf_flags = 1 << sh_css_frame_out;
	sh_css_sp_stage.stage_type = SH_CSS_SP_STAGE_TYPE;
	sh_css_sp_stage.func =
		(unsigned int)SH_CSS_SP_BIN_COPY;

	set_output_frame_buffer(out_frame,
						(unsigned)pipe_id, stage_num);

	/* sp_bin_copy_init on the SP does not deal with dynamica/static yet */
	/* For now always update the dynamic data from out frames. */
	sh_css_store_sp_per_frame_data(pipe_id, &sh_css_sp_fw);
}

void
sh_css_sp_start_raw_copy(struct sh_css_binary *binary,
			 struct sh_css_frame *out_frame,
			 unsigned two_ppc,
			 bool input_is_raw_reordered,
			 enum sh_css_pipe_config_override pipe_conf_override)
{
	enum sh_css_pipe_id pipe_id;
	unsigned int thread_id;
	unsigned stage_num = 0;
	struct sh_css_sp_pipeline *pipe;

assert(out_frame != NULL);

	{
		/**
		 * Clear sh_css_sp_stage for easy debugging.
		 * program_input_circuit must be saved as it is set outside
		 * this function.
		 */
		unsigned int program_input_circuit;
		program_input_circuit = sh_css_sp_stage.program_input_circuit;
		memset(&sh_css_sp_stage, 0, sizeof(sh_css_sp_stage));
		sh_css_sp_stage.program_input_circuit = program_input_circuit;
	}

	pipe_id = SH_CSS_COPY_PIPELINE;
	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	pipe = &sh_css_sp_group.pipe[thread_id];

	pipe->copy.raw.height	    = out_frame->info.height;
	pipe->copy.raw.width	    = out_frame->info.width;
	pipe->copy.raw.padded_width  = out_frame->info.padded_width;
	pipe->copy.raw.raw_bit_depth = out_frame->info.raw_bit_depth;
	pipe->copy.raw.max_input_width = binary->info->max_input_width;
	pipe->num_stages = 1;
	pipe->pipe_id = pipe_id;
	/* TODO: next indicates from which queues parameters need to be
		 sampled, needs checking/improvement */
	if (pipe_conf_override == SH_CSS_PIPE_CONFIG_OVRD_NO_OVRD)
		pipe->pipe_config =
			(SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS << thread_id);
	else
		pipe->pipe_config = pipe_conf_override;

	sh_css_sp_group.config.input_formatter.isp_2ppc = two_ppc;
	sh_css_sp_group.config.input_is_raw_reordered =
						input_is_raw_reordered;

	sh_css_sp_stage.num = stage_num;
	sh_css_sp_stage.irq_buf_flags = 1 << sh_css_frame_out;
	sh_css_sp_stage.xmem_bin_addr = binary->info->xmem_addr;
	sh_css_sp_stage.stage_type = SH_CSS_SP_STAGE_TYPE;
	sh_css_sp_stage.func =
		(unsigned int)SH_CSS_SP_RAW_COPY;

	set_output_frame_buffer(out_frame,
						(unsigned)pipe_id, stage_num);

	/* sp_raw_copy_init on the SP does not deal with dynamica/static yet */
	/* For now always update the dynamic data from out frames. */
	sh_css_store_sp_per_frame_data(pipe_id, &sh_css_sp_fw);
}

unsigned int
sh_css_sp_get_binary_copy_size(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_output = fw->info.sp.output;
	unsigned int o = offsetof(struct sh_css_sp_output,
				bin_copy_bytes_copied) / sizeof(int);
	(void)HIVE_ADDR_sp_output; /* To get rid of warning in CRUN */
	return load_sp_array_uint(sp_output, o);
}

unsigned int
sh_css_sp_get_sw_interrupt_value(unsigned int irq)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_output = fw->info.sp.output;
	unsigned int o = offsetof(struct sh_css_sp_output, sw_interrupt_value)
				/ sizeof(int);
	(void)HIVE_ADDR_sp_output; /* To get rid of warning in CRUN */
	return load_sp_array_uint(sp_output, o+irq);
}

static void
sh_css_frame_info_to_sp(struct sh_css_sp_frame_info *sp,
			const struct sh_css_frame_info *host)
{
  sp->width	      = host->width;
  sp->height	      = host->height;
  sp->padded_width    = host->padded_width;
  sp->format	      = host->format;
  sp->raw_bit_depth   = host->raw_bit_depth;
  sp->raw_bayer_order = host->raw_bayer_order;
}

static void
sh_css_copy_frame_to_spframe(struct sh_css_sp_frame *sp_frame_out,
				const struct sh_css_frame *frame_in,
				unsigned pipe_num, unsigned stage_num,
				enum sh_css_frame_id id)
{
	/* TODO: remove pipe and stage from interface */
	(void)pipe_num;
	(void)stage_num;

	if (frame_in->dynamic_data_index >= 0) {
		assert((id == sh_css_frame_in) ||
				(id == sh_css_frame_out) ||
				(id == sh_css_frame_out_vf));
		/*
		 * value >=0 indicates that function init_frame_pointers()
		 * should use the dynamic data address
		 */
		assert(frame_in->dynamic_data_index <
					SH_CSS_NUM_DYNAMIC_FRAME_IDS);
		/*
		 * static_frame_data is overloaded, small values (<3) are
		 * the dynamic index, large values are the static address
		 */
		sh_css_sp_stage.frames.static_frame_data[id] =
						frame_in->dynamic_data_index;
	} else {
		sh_css_sp_stage.frames.static_frame_data[id] = frame_in->data;
	}

	if (!sp_frame_out)
		return;

	sh_css_frame_info_to_sp(&sp_frame_out->info, &frame_in->info);

	switch (frame_in->info.format) {
	case SH_CSS_FRAME_FORMAT_RAW:
	case SH_CSS_FRAME_FORMAT_RAW_REORDERED:
		sp_frame_out->planes.raw.offset = frame_in->planes.raw.offset;
		break;
	case SH_CSS_FRAME_FORMAT_RGB565:
	case SH_CSS_FRAME_FORMAT_RGBA888:
		sp_frame_out->planes.rgb.offset = frame_in->planes.rgb.offset;
		break;
	case SH_CSS_FRAME_FORMAT_PLANAR_RGB888:
		sp_frame_out->planes.planar_rgb.r.offset =
			frame_in->planes.planar_rgb.r.offset;
		sp_frame_out->planes.planar_rgb.g.offset =
			frame_in->planes.planar_rgb.g.offset;
		sp_frame_out->planes.planar_rgb.b.offset =
			frame_in->planes.planar_rgb.b.offset;
		break;
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_UYVY:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
		sp_frame_out->planes.yuyv.offset = frame_in->planes.yuyv.offset;
		break;
	case SH_CSS_FRAME_FORMAT_NV11:
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV21:
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV61:
		sp_frame_out->planes.nv.y.offset =
			frame_in->planes.nv.y.offset;
		sp_frame_out->planes.nv.uv.offset =
			frame_in->planes.nv.uv.offset;
		break;
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV422:
	case SH_CSS_FRAME_FORMAT_YUV444:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_YUV422_16:
	case SH_CSS_FRAME_FORMAT_YV12:
	case SH_CSS_FRAME_FORMAT_YV16:
		sp_frame_out->planes.yuv.y.offset =
			frame_in->planes.yuv.y.offset;
		sp_frame_out->planes.yuv.u.offset =
			frame_in->planes.yuv.u.offset;
		sp_frame_out->planes.yuv.v.offset =
			frame_in->planes.yuv.v.offset;
		break;
	case SH_CSS_FRAME_FORMAT_QPLANE6:
		sp_frame_out->planes.plane6.r.offset =
			frame_in->planes.plane6.r.offset;
		sp_frame_out->planes.plane6.r_at_b.offset =
			frame_in->planes.plane6.r_at_b.offset;
		sp_frame_out->planes.plane6.gr.offset =
			frame_in->planes.plane6.gr.offset;
		sp_frame_out->planes.plane6.gb.offset =
			frame_in->planes.plane6.gb.offset;
		sp_frame_out->planes.plane6.b.offset =
			frame_in->planes.plane6.b.offset;
		sp_frame_out->planes.plane6.b_at_r.offset =
			frame_in->planes.plane6.b_at_r.offset;
		break;
	case SH_CSS_FRAME_FORMAT_BINARY_8:
		sp_frame_out->planes.binary.data.offset =
			frame_in->planes.binary.data.offset;
		break;
	default:
		/* This should not happen, but in case it does,
		 * nullify the planes
		 */
		memset(&sp_frame_out->planes, 0, sizeof(sp_frame_out->planes));
		break;
	}

}

static enum sh_css_err
set_input_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	switch (frame->info.format) {
	case SH_CSS_FRAME_FORMAT_QPLANE6:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_RAW:
	case SH_CSS_FRAME_FORMAT_RAW_REORDERED:
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
	case SH_CSS_FRAME_FORMAT_NV12:
		break;
	default:
		return sh_css_err_unsupported_frame_format;
	}
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.in, frame,
					pipe_num, stage_num,
					sh_css_frame_in);

	return sh_css_success;
}

static enum sh_css_err
set_output_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
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
	case SH_CSS_FRAME_FORMAT_RAW_REORDERED:
	case SH_CSS_FRAME_FORMAT_QPLANE6:
	case SH_CSS_FRAME_FORMAT_BINARY_8:
		break;
	default:
		return sh_css_err_unsupported_frame_format;
	}
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.out, frame,
					pipe_num, stage_num,
					sh_css_frame_out);
	return sh_css_success;
}

static enum sh_css_err
set_ref_in_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.ref_in, frame,
					pipe_num, stage_num,
					sh_css_frame_ref_in);
	return sh_css_success;
}

static enum sh_css_err
set_ref_out_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(NULL, frame,
					pipe_num, stage_num,
					sh_css_frame_ref_out);
	return sh_css_success;
}

static enum sh_css_err
set_tnr_in_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV_LINE)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.tnr_in, frame,
					pipe_num, stage_num,
					sh_css_frame_tnr_in);
	return sh_css_success;
}

static enum sh_css_err
set_tnr_out_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV_LINE)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(NULL, frame,
					pipe_num, stage_num,
					sh_css_frame_tnr_out);
	return sh_css_success;
}

static enum sh_css_err
set_capture_pp_frame_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV420)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.extra, frame,
					pipe_num, stage_num,
					sh_css_frame_extra);
	return sh_css_success;
}

static enum sh_css_err
set_view_finder_buffer(const struct sh_css_frame *frame,
			unsigned pipe_num, unsigned stage_num)
{
	if (frame == NULL)
		return sh_css_err_invalid_arguments;

	if (frame->info.format != SH_CSS_FRAME_FORMAT_YUV_LINE)
		return sh_css_err_unsupported_frame_format;
	sh_css_copy_frame_to_spframe(&sh_css_sp_stage.frames.out_vf, frame,
					pipe_num, stage_num,
					sh_css_frame_out_vf);
	return sh_css_success;
}

void sh_css_sp_set_if_configs(
	const input_formatter_cfg_t		*config_a,
	const input_formatter_cfg_t		*config_b)
{
	input_formatter_ID_t	id;
#if defined(IS_ISP_2300_SYSTEM)
	bool	block[N_INPUT_FORMATTER_ID] = {false, false};
#elif defined(IS_ISP_2400_SYSTEM)
	bool	block[N_INPUT_FORMATTER_ID] = {false, false, false};
#else
#error "sh_css_sp_set_if_configs: ISP_SYSTEM must be one of \
	{IS_ISP_2300_SYSTEM, IS_ISP_2400_SYSTEM}"
#endif

	block[INPUT_FORMATTER0_ID] = config_a->block_no_reqs;
	if (config_b != NULL)
		block[INPUT_FORMATTER1_ID] = config_b->block_no_reqs;

	for (id = (input_formatter_ID_t)0; id < N_INPUT_FORMATTER_ID; id++) {
		input_formatter_rst(id);
		input_formatter_set_fifo_blocking_mode(id, block[id]);
	}

	sh_css_sp_group.config.input_formatter.config_a = *config_a;
	sh_css_sp_group.config.input_formatter.a_changed = true;

	if (config_b != NULL) {
		sh_css_sp_group.config.input_formatter.config_b = *config_b;
		sh_css_sp_group.config.input_formatter.b_changed = true;
	}

return;
}

void
sh_css_sp_program_input_circuit(int fmt_type,
				int ch_id,
				enum sh_css_input_mode input_mode)
{
	sh_css_sp_group.config.input_circuit.no_side_band = false;
	sh_css_sp_group.config.input_circuit.fmt_type     = fmt_type;
	sh_css_sp_group.config.input_circuit.ch_id	      = ch_id;
	sh_css_sp_group.config.input_circuit.input_mode   = input_mode;
/*
 * The SP group is only loaded at SP boot time and is read once
 * change flags as "input_circuit_cfg_changed" must be reset on the SP
 */
	sh_css_sp_group.config.input_circuit_cfg_changed = true;
	sh_css_sp_stage.program_input_circuit = true;
}

void
sh_css_sp_configure_sync_gen(int width, int height,
			     int hblank_cycles,
			     int vblank_cycles)
{
	sh_css_sp_group.config.sync_gen.width	       = width;
	sh_css_sp_group.config.sync_gen.height	       = height;
	sh_css_sp_group.config.sync_gen.hblank_cycles = hblank_cycles;
	sh_css_sp_group.config.sync_gen.vblank_cycles = vblank_cycles;
}

void
sh_css_sp_configure_tpg(int x_mask,
			int y_mask,
			int x_delta,
			int y_delta,
			int xy_mask)
{
	sh_css_sp_group.config.tpg.x_mask  = x_mask;
	sh_css_sp_group.config.tpg.y_mask  = y_mask;
	sh_css_sp_group.config.tpg.x_delta = x_delta;
	sh_css_sp_group.config.tpg.y_delta = y_delta;
	sh_css_sp_group.config.tpg.xy_mask = xy_mask;
}

void
sh_css_sp_configure_prbs(int seed)
{
	sh_css_sp_group.config.prbs.seed = seed;
}

enum sh_css_err
sh_css_sp_write_frame_pointers(const struct sh_css_binary_args *args,
				unsigned pipe_num, unsigned stage_num)
{
	enum sh_css_err err = sh_css_success;
	if (args->in_frame)
		err = set_input_frame_buffer(args->in_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->in_ref_frame)
		err = set_ref_in_frame_buffer(args->in_ref_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->in_tnr_frame)
		err = set_tnr_in_frame_buffer(args->in_tnr_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->out_vf_frame)
		err = set_view_finder_buffer(args->out_vf_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->extra_frame)
		err = set_capture_pp_frame_buffer(args->extra_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->out_ref_frame)
		err = set_ref_out_frame_buffer(args->out_ref_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->out_tnr_frame)
		err = set_tnr_out_frame_buffer(args->out_tnr_frame,
						pipe_num, stage_num);
	if (err == sh_css_success && args->out_frame)
		err = set_output_frame_buffer(args->out_frame,
						pipe_num, stage_num);
	return err;
}

void
sh_css_sp_init_group(bool two_ppc, enum sh_css_input_format input_format,
		     bool no_isp_sync)
{
	sh_css_sp_group.config.input_formatter.stream_format = input_format;
	sh_css_sp_group.config.input_formatter.isp_2ppc = two_ppc;

	sh_css_sp_group.config.no_isp_sync = no_isp_sync;
	/* decide whether the frame is processed online or offline */
	sh_css_sp_group.config.is_offline  = sh_css_continuous_start_sp_copy();
}

void
sh_css_stage_write_binary_info(struct sh_css_binary_info *info)
{
	sh_css_sp_stage.binary_info = *info;
}

static enum sh_css_err
sh_css_sp_init_stage(struct sh_css_binary *binary,
		    const char *binary_name,
		    const struct sh_css_blob_info *blob_info,
		    const struct sh_css_binary_args *args,
		    enum sh_css_pipe_id id,
		    unsigned stage,
		    bool preview_mode,
		    bool low_light,
		    bool xnr,
		    unsigned irq_buf_flags,
		    const struct sh_css_hmm_isp_interface *isp_mem_if)
{
	const struct sh_css_binary_info *info = binary->info;
	enum sh_css_err err = sh_css_success;
	bool start_copy;
	int i;

	enum sh_css_pipe_id pipe_id = id;
	unsigned int thread_id;
	bool continuous = sh_css_continuous_is_enabled();
	{
		/**
		 * Clear sh_css_sp_stage for easy debugging.
		 * program_input_circuit must be saved as it is set outside
		 * this function.
		 */
		unsigned int program_input_circuit;
		program_input_circuit = sh_css_sp_stage.program_input_circuit;
		memset(&sh_css_sp_stage, 0, sizeof(sh_css_sp_stage));
		sh_css_sp_stage.program_input_circuit = program_input_circuit;
	}

	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	sh_css_sp_group.pipe[thread_id].num_stages++;

	if (info == NULL) {
		sh_css_sp_group.pipe[thread_id].sp_stage_addr[stage] =
			(hrt_vaddress)HOST_ADDRESS(NULL);
		return sh_css_success;
	}

	start_copy = stage == 0 && sh_css_continuous_start_sp_copy();
	sh_css_sp_stage.deinterleaved = start_copy;

	/*
	 * TODO: Make the Host dynamically determine
	 * the stage type.
	 */
	sh_css_sp_stage.stage_type = SH_CSS_ISP_STAGE_TYPE;
	sh_css_sp_stage.num		= stage;
	sh_css_sp_stage.isp_online	= binary && binary->online;
	sh_css_sp_stage.isp_copy_vf     = args->copy_vf;
	sh_css_sp_stage.isp_copy_output = args->copy_output;
	/* These flags wil be set from the css top level */
	sh_css_sp_stage.irq_buf_flags   = irq_buf_flags;

	/* Copy the frame infos first, to be overwritten by the frames,
	   if these are present.
	*/
	sh_css_frame_info_to_sp(&sh_css_sp_stage.frames.in.info,
				&binary->in_frame_info);
	sh_css_frame_info_to_sp(&sh_css_sp_stage.frames.out.info,
				&binary->out_frame_info);
	sh_css_frame_info_to_sp(&sh_css_sp_stage.frames.internal_frame_info,
				&binary->internal_frame_info);
	sh_css_sp_stage.dvs_envelope.width    = binary->dvs_envelope.width;
	sh_css_sp_stage.dvs_envelope.height   = binary->dvs_envelope.height;
	sh_css_sp_stage.isp_deci_log_factor   = binary->deci_factor_log2;
	sh_css_sp_stage.isp_vf_downscale_bits = binary->vf_downscale_log2;

	sh_css_sp_stage.sp_enable_xnr = xnr;
/*	sh_css_sp_stage.uds.extra_vectors   = info->uds.extra_vectors; */ /* task 3340 */
	sh_css_sp_stage.xmem_bin_addr = info->xmem_addr;
	sh_css_sp_stage.xmem_map_addr = sh_css_params_ddr_address_map();
	sh_css_sp_stage.anr	      = low_light;
	sh_css_sp_stage.isp_blob_info = *blob_info;
	sh_css_stage_write_binary_info((struct sh_css_binary_info *)info);
	memcpy(sh_css_sp_stage.binary_name, binary_name,
		strlen(binary_name)+1);
	memcpy(&sh_css_sp_stage.isp_mem_interface, isp_mem_if,
		sizeof(sh_css_sp_stage.isp_mem_interface));

#if 0
	{
		struct sh_css_vector motion;
		struct sh_css_zoom zoom;

		sh_css_get_zoom(&zoom);
		sh_css_get_dis_motion(&motion);

		sh_css_update_uds_and_crop_info(binary->info,
					&binary->in_frame_info,
					&binary->out_frame_info,
					&binary->dvs_envelope,
					preview_mode,
					&zoom,
					&motion,
					&sh_css_sp_stage.uds,
					&sh_css_sp_stage.sp_out_crop_pos);
	}
#else
	/**
	 * Even when a stage does not need uds and does not params,
	 * sp_uds_init() seems to be called (needs further investigation)
	 * This function can not deal with dx, dy = {0, 0}
	 */
	(void)preview_mode;
#if 0
	sh_css_sp_stage.uds =
		(struct sh_css_uds_info){HRT_GDC_N, HRT_GDC_N, 0, 0};
#endif

#endif

	sh_css_params_set_current_binary(binary);

	/* Clean static frame info before we update it */
	/*
	 * TODO: Initialize the static frame data with
	 * "sh_css_frame_null".
	 */
	for (i = 0; i < SH_CSS_NUM_FRAME_IDS; i++)
		/* Here, we do not initialize it to zero for now */
		/* to be able to recognize non-updated elements  */
		sh_css_sp_stage.frames.static_frame_data[i] = -1;

	err = sh_css_sp_write_frame_pointers(args, pipe_id, stage);
	if (err != sh_css_success)
		return err;


	/* TODO: @GC Remove this code after the proper mechanism is placed! */
	if (binary->in_frame_info.format == SH_CSS_FRAME_FORMAT_RAW_REORDERED
	&& binary->info->enable.rawdeci) {
		unsigned int left_cropping = binary->info->left_cropping;
		/* It takes back the undesired effect (circular data flow,
		 * preview->copy->preview) due to the call of function
		 * sh_css_sp_write_frame_pointers */
		sh_css_sp_stage.frames.in.info.padded_width /= 2;
		sh_css_sp_stage.frames.in.info.width += left_cropping;
		sh_css_sp_stage.frames.in.info.width /= 2;
		sh_css_sp_stage.frames.in.info.height += left_cropping;
		sh_css_sp_stage.frames.in.info.height /= 2;
	}

	if (continuous &&  binary->info->enable.rawdeci) {
		/* TODO: Remove this after preview output decimation is fixed
		 * by configuring out&vf info fiels properly */
		sh_css_sp_stage.frames.out.info.padded_width
			<<= binary->vf_downscale_log2;
		sh_css_sp_stage.frames.out.info.width
			<<= binary->vf_downscale_log2;
		sh_css_sp_stage.frames.out.info.height
			<<= binary->vf_downscale_log2;
	}

	store_sp_stage_data(pipe_id, stage);

	return sh_css_success;
}

static enum sh_css_err
sp_init_stage(struct sh_css_pipeline_stage *stage,
		    enum sh_css_pipe_id id,
		    unsigned stage_num,
		    bool preview_mode,
	      bool low_light,
	      bool xnr)
{
	struct sh_css_binary *binary = stage->binary;
	const struct sh_css_fw_info *firmware = stage->firmware;
	const struct sh_css_binary_args *args = &stage->args;
	const unsigned char *binary_name;
	const struct sh_css_binary_info *info = NULL;
	struct sh_css_binary tmp_binary;
	const struct sh_css_blob_info *blob_info;
	struct sh_css_hmm_isp_interface isp_mem_if[SH_CSS_NUM_ISP_MEMORIES];
	const struct sh_css_hmm_isp_interface *mem_if = isp_mem_if;
	enum sh_css_pipe_id pipe_id = id;

	memset(isp_mem_if, 0, sizeof(isp_mem_if));

	if (binary) {
		info = binary->info;
		binary_name =
			(const unsigned char *)(info->blob->name);
		blob_info = &info->blob->header.blob;
	} else {
		info = &firmware->info.isp;
		sh_css_fill_binary_info(info, false, false,
			    SH_CSS_INPUT_FORMAT_RAW_10,
			    args->in_frame  ? &args->in_frame->info  : NULL,
			    args->out_frame ? &args->out_frame->info : NULL,
			    args->out_vf_frame ? &args->out_vf_frame->info
						: NULL,
			    &tmp_binary,
			    false);
		binary = &tmp_binary;
		binary->info = info;
		binary_name = SH_CSS_EXT_ISP_PROG_NAME(firmware);
		blob_info = &firmware->blob;
		mem_if = firmware->memory_interface;
	}

	sh_css_sp_init_stage(binary,
			     (const char *)binary_name,
			     blob_info,
			     args,
			     pipe_id,
			     stage_num,
			     preview_mode,
			     low_light,
			     xnr,
				stage->irq_buf_flags,
				mem_if);
	return sh_css_success;
}

void
sh_css_sp_init_pipeline(struct sh_css_pipeline *me,
			enum sh_css_pipe_id id,
			bool preview_mode,
			bool low_light,
			bool xnr,
			bool two_ppc,
			bool continuous,
			bool offline,
			bool input_is_raw_reordered,
			enum sh_css_pipe_config_override copy_ovrd)
{
	/* Get first stage */
	struct sh_css_pipeline_stage *stage;
	struct sh_css_binary	     *first_binary = me->stages->binary;
	struct sh_css_binary_args    *first_args   = &me->stages->args;
	unsigned num;

	enum sh_css_pipe_id pipe_id = id;
	unsigned int thread_id;

	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	memset(&sh_css_sp_group.pipe[thread_id], 0, sizeof(struct sh_css_sp_pipeline));

	/* Count stages */
	for (stage = me->stages, num = 0; stage; stage = stage->next, num++) {
		stage->stage_num = num;
		sh_css_debug_pipe_graph_dump_stage(stage, id);
	}
	me->num_stages = num;

	if (first_binary != NULL) {
	/* Init pipeline data */
		sh_css_sp_init_group(two_ppc, first_binary->input_format, offline);
	/* for Capture, do we need to add more modes like */
		if (continuous &&
			(first_binary->info->mode == SH_CSS_BINARY_MODE_PREVIEW ||
			 first_binary->info->mode == SH_CSS_BINARY_MODE_PRIMARY)) {
#if 0
			sh_css_queue_buffer(SH_CSS_COPY_PIPELINE,
				SH_CSS_BUFFER_TYPE_OUTPUT_FRAME,
				(void *)first_args->cc_frame);
#endif
			sh_css_sp_start_raw_copy(first_binary, first_args->cc_frame,
				two_ppc, input_is_raw_reordered,
				copy_ovrd);

			sh_css_debug_pipe_graph_dump_sp_raw_copy(first_args->cc_frame);
		}
	} /* if (first_binary != NULL) */

	/* Init stage data */
	sh_css_init_host2sp_frame_data();

	sh_css_sp_group.pipe[thread_id].num_stages = 0;
	sh_css_sp_group.pipe[thread_id].pipe_id = pipe_id;
	/* TODO: next indicates from which queues parameters need to be
		 sampled, needs checking/improvement */
	if (sh_css_pipe_uses_params(me)) {
		sh_css_sp_group.pipe[thread_id].pipe_config =
			SH_CSS_PIPE_CONFIG_SAMPLE_PARAMS << thread_id;
	}

	/* For continuous use-cases, SP copy is responsible for sampling the
	 * parameters */
	if (sh_css_continuous_is_enabled())
		sh_css_sp_group.pipe[thread_id].pipe_config = 0;

	for (stage = me->stages, num = 0; stage; stage = stage->next, num++)
		sp_init_stage(stage, pipe_id, num, preview_mode, low_light, xnr);

	store_sp_group_data();

}

void
sh_css_sp_uninit_pipeline(enum sh_css_pipe_id pipe_id)
{
	unsigned int thread_id;
	sh_css_query_sp_thread_id(pipe_id, &thread_id);
	/*memset(&sh_css_sp_group.pipe[thread_id], 0, sizeof(struct sh_css_sp_pipeline));*/
	sh_css_sp_group.pipe[thread_id].num_stages = 0;
}

static void
init_host2sp_command(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int o = offsetof(struct host_sp_communication, host2sp_command)
				/ sizeof(int);
	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */
	store_sp_array_uint(host_sp_com, o, host2sp_cmd_ready);
}

void
sh_css_write_host2sp_command(enum host2sp_commands host2sp_command)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int o = offsetof(struct host_sp_communication, host2sp_command)
				/ sizeof(int);
	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

	/* Previous command must be handled by SP (by design) */
assert(load_sp_array_uint(host_sp_com, o) == host2sp_cmd_ready);

	store_sp_array_uint(host_sp_com, o, host2sp_command);
}

enum host2sp_commands
sh_css_read_host2sp_command(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int o = offsetof(struct host_sp_communication, host2sp_command)
				/ sizeof(int);
	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */
	return load_sp_array_uint(host_sp_com, o);
}


/*
 * Frame data is no longer part of the sp_stage structure but part of a
 * seperate structure. The aim is to make the sp_data struct static
 * (it defines a pipeline) and that the dynamic (per frame) data is stored
 * separetly.
 *
 * This function must be called first every where were you start constructing
 * a new pipeline by defining one or more stages with use of variable
 * sh_css_sp_stage. Even the special cases like accelerator and copy_frame
 * These have a pipeline of just 1 stage.
 */
void
sh_css_init_host2sp_frame_data(void)
{
	/* Clean table */
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */
	/*
	 * rvanimme: don't clean it to save static frame info line ref_in
	 * ref_out, tnr_in and tnr_out. Once this static data is in a
	 * seperate data struct, this may be enable (but still, there is
	 * no need for it)
	 */
#if 0
	unsigned i;
	for (i = 0; i < SH_CSS_MAX_PIPELINES*SH_CSS_NUM_FRAME_IDS; i++)
		store_sp_array_uint(host_sp_com, i+o, 0);
#endif
}


/**
 * @brief Update the offline frame information in host_sp_communication.
 * Refer to "sh_css_sp.h" for more details.
 */
void
sh_css_update_host2sp_offline_frame(
				unsigned frame_num,
				struct sh_css_frame *frame)
{
	const struct sh_css_fw_info *fw;
	unsigned int HIVE_ADDR_host_sp_com;
	unsigned int o;

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

assert(frame_num < NUM_CONTINUOUS_FRAMES);

	/* Write new frame data into SP DMEM */
	fw = &sh_css_sp_fw;
	HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	o = offsetof(struct host_sp_communication, host2sp_offline_frames)
		/ sizeof(int);
	o += frame_num;

	store_sp_array_uint(host_sp_com, o,
				frame ? frame->data : 0);
}

void
sh_css_update_host2sp_cont_num_raw_frames(unsigned num_frames)
{
	const struct sh_css_fw_info *fw;
	unsigned int HIVE_ADDR_host_sp_com;
	unsigned int o;

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

	/* Write new frame data into SP DMEM */
	fw = &sh_css_sp_fw;
	HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	o = offsetof(struct host_sp_communication, host2sp_cont_num_raw_frames)
		/ sizeof(int);

	store_sp_array_uint(host_sp_com, o,
				num_frames);
}

void
sh_css_event_init_irq_mask(void)
{
	int i;
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int offset;

	struct sh_css_event_irq_mask event_irq_mask_init = {
		.or_mask  = SH_CSS_EVENT_IRQ_MASK_ALL,
		.and_mask = SH_CSS_EVENT_IRQ_MASK_ALL };

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

	assert(sizeof(event_irq_mask_init) % HRT_BUS_BYTES == 0);
	for (i = 0; i < SH_CSS_NR_OF_PIPELINES; i++) {
		offset = offsetof(struct host_sp_communication,
						host2sp_event_irq_mask[i]);
		assert(offset % HRT_BUS_BYTES == 0);
		sp_dmem_store(SP0_ID,
			(unsigned int)sp_address_of(host_sp_com) + offset,
			&event_irq_mask_init, sizeof(event_irq_mask_init));
	}

}

enum sh_css_err
sh_css_event_set_irq_mask(
	enum sh_css_pipe_id pipe_id,
	unsigned int or_mask,
	unsigned int and_mask)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int offset;
	struct sh_css_event_irq_mask event_irq_mask;

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_event_set_irq_mask()\n");

	assert(SH_CSS_NR_OF_PIPELINES == NR_OF_PIPELINES);
	assert(pipe_id < SH_CSS_NR_OF_PIPELINES);
	assert(or_mask <= UINT16_MAX);
	assert(and_mask <= UINT16_MAX);

	event_irq_mask.or_mask  = (uint16_t)or_mask;
	event_irq_mask.and_mask = (uint16_t)and_mask;

	offset = offsetof(struct host_sp_communication,
					host2sp_event_irq_mask[pipe_id]);
	assert(offset % HRT_BUS_BYTES == 0);
	sp_dmem_store(SP0_ID,
		(unsigned int)sp_address_of(host_sp_com) + offset,
		&event_irq_mask, sizeof(event_irq_mask));

	return sh_css_success;
}

enum sh_css_err
sh_css_event_get_irq_mask(
	enum sh_css_pipe_id pipe_id,
	unsigned int *or_mask,
	unsigned int *and_mask)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_host_sp_com = fw->info.sp.host_sp_com;
	unsigned int offset;
	struct sh_css_event_irq_mask event_irq_mask;

	(void)HIVE_ADDR_host_sp_com; /* Suppres warnings in CRUN */

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_event_get_irq_mask()\n");

	assert(SH_CSS_NR_OF_PIPELINES == NR_OF_PIPELINES);
	assert(pipe_id < SH_CSS_NR_OF_PIPELINES);

	offset = offsetof(struct host_sp_communication,
					host2sp_event_irq_mask[pipe_id]);
	assert(offset % HRT_BUS_BYTES == 0);
	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(host_sp_com) + offset,
		&event_irq_mask, sizeof(event_irq_mask));

	if (or_mask)
		*or_mask = event_irq_mask.or_mask;

	if (and_mask)
		*and_mask = event_irq_mask.and_mask;

	return sh_css_success;
}

void
sh_css_sp_set_sp_running(bool flag)
{
	sp_running = flag;
}

void
sh_css_sp_start_isp(void)
{
	const struct sh_css_fw_info *fw;
	unsigned int HIVE_ADDR_sp_isp_started;
	unsigned int HIVE_ADDR_sp_sw_state;
	unsigned int HIVE_ADDR_host_sp_queues_initialized;
	unsigned int HIVE_ADDR_sp_sleep_mode;
	unsigned int HIVE_ADDR_sp_invalidate_tlb;
	unsigned int HIVE_ADDR_sp_request_flash;

	fw = &sh_css_sp_fw;
	HIVE_ADDR_sp_isp_started = fw->info.sp.isp_started;
	HIVE_ADDR_sp_sw_state = fw->info.sp.sw_state;
	HIVE_ADDR_host_sp_queues_initialized =
		fw->info.sp.host_sp_queues_initialized;
	HIVE_ADDR_sp_sleep_mode = fw->info.sp.sleep_mode;
	HIVE_ADDR_sp_invalidate_tlb = fw->info.sp.invalidate_tlb;
	HIVE_ADDR_sp_request_flash = fw->info.sp.request_flash;

	if (sp_running)
		return;

	(void)HIVE_ADDR_sp_isp_started; /* Suppres warnings in CRUN */
	(void)HIVE_ADDR_sp_sw_state; /* Suppres warnings in CRUN */
	(void)HIVE_ADDR_sp_sleep_mode;
	(void)HIVE_ADDR_sp_invalidate_tlb;
	(void)HIVE_ADDR_sp_request_flash;

	sh_css_debug_pipe_graph_dump_epilogue();

	store_sp_per_frame_data(fw);
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_isp_started),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_sw_state),
		(uint32_t)(SP_SW_STATE_NULL));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(host_sp_queues_initialized),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_sleep_mode),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_invalidate_tlb),
		(uint32_t)(0));
	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_request_flash),
		(uint32_t)(0));


	init_host2sp_command();
	/* Note 1: The sp_start_isp function contains a wait till
	 * the input network is configured by the SP.
	 * Note 2: Not all SP binaries supports host2sp_commands.
	 * In case a binary does support it, the host2sp_command
	 * will have status cmd_ready after return of the function
	 * sh_css_hrt_sp_start_isp. There is no race-condition here
	 * because only after the process_frame command has been
	 * received, the SP starts configuring the input network.
	 */
	sh_css_mmu_invalidate_cache();
	sh_css_hrt_sp_start_isp();

	sp_running = true;
}

bool
sh_css_isp_has_started(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_isp_started = fw->info.sp.isp_started;
	(void)HIVE_ADDR_sp_isp_started; /* Suppres warnings in CRUN */

	return load_sp_uint(sp_isp_started);
}

bool
sh_css_sp_has_initialized(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_sw_state = fw->info.sp.sw_state;
	(void)HIVE_ADDR_sp_sw_state; /* Suppres warnings in CRUN */

	return (load_sp_uint(sp_sw_state) == SP_SW_INITIALIZED);
}

bool
sh_css_sp_has_terminated(void)
{
	const struct sh_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int HIVE_ADDR_sp_sw_state = fw->info.sp.sw_state;
	(void)HIVE_ADDR_sp_sw_state; /* Suppres warnings in CRUN */
	return (load_sp_uint(sp_sw_state) == SP_SW_TERMINATED);
}

/**
 * @brief Initialize the DMA software-mask in the debug mode.
 * Refer to "sh_css_sp.h" for more details.
 */
bool
sh_css_sp_init_dma_sw_reg(int dma_id)
{
	int i;

	/* enable all the DMA channels */
	for (i = 0; i < N_DMA_CHANNEL_ID; i++) {
		/* enable the writing request */
		sh_css_sp_set_dma_sw_reg(dma_id,
				i,
				0,
				true);
		/* enable the reading request */
		sh_css_sp_set_dma_sw_reg(dma_id,
				i,
				1,
				true);
	}

	return true;
}

/**
 * @brief Set the DMA software-mask in the debug mode.
 * Refer to "sh_css_sp.h" for more details.
 */
bool
sh_css_sp_set_dma_sw_reg(int dma_id,
		int channel_id,
		int request_type,
		bool enable)
{
	uint32_t sw_reg;
	uint32_t bit_val;
	uint32_t bit_offset;
	uint32_t bit_mask;

	(void)dma_id;

assert(channel_id >= 0 && channel_id < N_DMA_CHANNEL_ID);
assert(request_type >= 0);

	/* get the software-mask */
	sw_reg =
		sh_css_sp_group.debug.dma_sw_reg;

	/* get the offest of the target bit */
	bit_offset = (8 * request_type) + channel_id;

	/* clear the value of the target bit */
	bit_mask = ~(1 << bit_offset);
	sw_reg &= bit_mask;

	/* set the value of the bit for the DMA channel */
	bit_val = enable ? 1 : 0;
	bit_val <<= bit_offset;
	sw_reg |= bit_val;

	/* update the software status of DMA channels */
	sh_css_sp_group.debug.dma_sw_reg = sw_reg;

	return true;
}

/**
 * @brief The Host sends the event to the SP.
 * Refer to "sh_css_sp.h" for details.
 */
void
sh_css_sp_snd_event(int evt_id, int evt_payload_0, int evt_payload_1, int evt_payload_2)
{
	uint32_t tmp[4];
	uint32_t sw_event;

	/*
	 * Encode the queue type, the thread ID and
	 * the queue ID into the event.
	 */
	tmp[0] = (uint32_t)evt_id;
	tmp[1] = (uint32_t)evt_payload_0;
	tmp[2] = (uint32_t)evt_payload_1;
	tmp[3] = (uint32_t)evt_payload_2;
	encode_sw_event(tmp, 4, &sw_event);

	/* queue the software event (busy-waiting) */
	while (!host2sp_enqueue_sp_event(sw_event))
		hrt_sleep();
}
