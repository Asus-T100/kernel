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

#if defined(SYSTEM_hive_isp_css_2400_system)
#include <hmm_2400/hmm_2400.h>
#endif
#include "sh_css_binary.h"
#include "sh_css.h"
#include "sh_css_internal.h"
#include "sh_css_hrt.h"
#include "sh_css_sp.h"
#include "sh_css_firmware.h"
#include "sh_css_defs.h"
#include "sh_css_binary_info.h"

static struct sh_css_binary_info all_binaries[SH_CSS_BINARY_NUM_IDS];
static struct sh_css_binary_info *binary_infos[SH_CSS_BINARY_NUM_MODES];

enum sh_css_err
sh_css_binary_grid_info(struct sh_css_binary *binary,
			struct sh_css_grid_info *info)
{
	info->dis_width             = binary->dis_ver_proj_num_3a;
	info->dis_aligned_width     = binary->dis_ver_proj_num_isp;
	info->dis_height            = binary->dis_hor_proj_num_3a;
	info->dis_aligned_height    = binary->dis_hor_proj_num_isp;
	info->dis_bqs_per_grid_cell = 1 << binary->dis_deci_factor_log2;
	info->dis_hor_coef_num      = binary->dis_hor_coef_num_3a;
	info->dis_ver_coef_num      = binary->dis_ver_coef_num_3a;
	/* 3A statistics grid */
	info->isp_in_width = binary->internal_frame_info.width;
	info->isp_in_height = binary->internal_frame_info.height;
	info->s3a_width = binary->s3atbl_width;
	info->s3a_height = binary->s3atbl_height;
	info->s3a_bqs_per_grid_cell = (1 << binary->deci_factor_log2);

	return sh_css_success;
}

static enum sh_css_err
load_binary(struct sh_css_binary_info *binary, bool *binary_found)
{
	const unsigned char *blob = sh_css_blob_info[binary->id].blob;
	unsigned size = sh_css_blob_info[binary->id].size;

	*binary_found = blob != NULL;
	/* we don't have this binary, skip it */
	if (!size)
		return sh_css_success;

	binary->xmem_addr = sh_css_load_blob(blob, size);
	if (!binary->xmem_addr)
		return sh_css_err_cannot_allocate_memory;
	return sh_css_success;
}

static void init_pc_histogram(struct sh_css_pc_histogram *histo)
{
	histo->length = 0;
	histo->run = NULL;
	histo->stall = NULL;
}

static void init_metrics(struct sh_css_binary_metrics *metrics,
			 unsigned int binary_id)
{
	metrics->mode = binary_id;
	metrics->next = NULL;
	init_pc_histogram(&metrics->isp_histogram);
	init_pc_histogram(&metrics->sp_histogram);
}

#define _init_binary_info(binary, prefix) { \
	int i; \
	enum sh_css_frame_format out_fmts[] = prefix ## OUTPUT_FORMATS; \
	binary->num_output_formats = sizeof(out_fmts)/sizeof(*out_fmts); \
	for (i = 0; i < binary->num_output_formats; i++) \
		binary->output_formats[i] = out_fmts[i]; \
	for (i = binary->num_output_formats; \
	     i < SH_CSS_MAX_NUM_FRAME_FORMATS; \
	     i++) \
		binary->output_formats[i] = -1; \
	binary->mode                    = prefix ## MODE; \
	binary->enable_sc               = prefix ## ENABLE_SC; \
	binary->enable_fpnr             = prefix ## ENABLE_FPNR; \
	binary->enable_s3a              = prefix ## ENABLE_S3A; \
	binary->enable_ds               = prefix ## ENABLE_DS; \
	binary->enable_uds              = prefix ## ENABLE_UDS; \
	binary->enable_dis              = prefix ## ENABLE_SDIS; \
	binary->enable_dvs_envelope     = prefix ## ENABLE_DVS_ENVELOPE; \
	binary->left_cropping           = prefix ## LEFT_CROPPING; \
	binary->top_cropping            = prefix ## TOP_CROPPING; \
	binary->c_subsampling           = prefix ## C_SUBSAMPLING; \
	binary->pipelining              = prefix ## PIPELINING; \
	binary->fixed_s3a_deci_log      = prefix ## FIXED_S3A_DECI_LOG; \
	binary->input                   = prefix ## INPUT; \
	binary->max_dvs_envelope_width  = prefix ## MAX_DVS_ENVELOPE_WIDTH; \
	binary->max_dvs_envelope_height = prefix ## MAX_DVS_ENVELOPE_HEIGHT; \
	binary->min_output_width        = prefix ## MIN_OUTPUT_WIDTH; \
	binary->max_output_width        = prefix ## MAX_OUTPUT_WIDTH; \
	binary->max_vf_log_downscale    = prefix ## MAX_VF_LOG_DOWNSCALE; \
	binary->enable_vf_veceven       = prefix ## ENABLE_VF_VECEVEN; \
	binary->output_num_chunks       = prefix ## OUTPUT_NUM_CHUNKS; \
}

static bool
supports_output_format(const struct sh_css_binary_info *info,
		       enum sh_css_frame_format format)
{
	int i;

	for (i = 0; i < info->num_output_formats; i++) {
		if (info->output_formats[i] == format)
			return true;
	}
	return false;
}

static enum sh_css_err
init_binary_info(struct sh_css_binary_info *info, bool *binary_found)
{
	unsigned int max_internal_width;

	switch (info->id) {
	case SH_CSS_BINARY_ID_COPY:
		_init_binary_info(info, ISP_COPY_);
		break;
	case SH_CSS_BINARY_ID_VF_PP:
		_init_binary_info(info, ISP_VF_PP_);
		break;
	case SH_CSS_BINARY_ID_CAPTURE_PP:
		_init_binary_info(info, ISP_CAPTURE_PP_);
		break;
	case SH_CSS_BINARY_ID_PRE_ISP:
		_init_binary_info(info, ISP_PRE_ISP_);
		break;
	case SH_CSS_BINARY_ID_GDC:
		_init_binary_info(info, ISP_GDC_);
		break;
	case SH_CSS_BINARY_ID_POST_ISP:
		_init_binary_info(info, ISP_POST_ISP_);
		break;
#ifndef SYSTEM_hive_isp_css_2400_system
	case SH_CSS_BINARY_ID_ANR:
		_init_binary_info(info, ISP_ANR_);
#endif
		break;
	case SH_CSS_BINARY_ID_PREVIEW_DZ:
		_init_binary_info(info, ISP_PREVIEW_DZ_);
		break;
	case SH_CSS_BINARY_ID_PREVIEW_DS:
		_init_binary_info(info, ISP_PREVIEW_DS_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_SMALL:
		_init_binary_info(info, ISP_PRIMARY_SMALL_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_DS:
		_init_binary_info(info, ISP_PRIMARY_DS_);
		break;
	case SH_CSS_BINARY_ID_BAYER_DS:
		_init_binary_info(info, ISP_BAYER_DS_);
		break;
	case SH_CSS_BINARY_ID_VIDEO_OFFLINE:
		_init_binary_info(info, ISP_VIDEO_OFFLINE_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_VAR:
		_init_binary_info(info, ISP_PRIMARY_VAR_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_8MP:
		_init_binary_info(info, ISP_PRIMARY_8MP_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_14MP:
		_init_binary_info(info, ISP_PRIMARY_14MP_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_16MP:
		_init_binary_info(info, ISP_PRIMARY_16MP_);
		break;
	case SH_CSS_BINARY_ID_PRIMARY_REF:
		_init_binary_info(info, ISP_PRIMARY_REF_);
		break;
	case SH_CSS_BINARY_ID_VIDEO_DZ:
		_init_binary_info(info, ISP_VIDEO_DZ_);
		break;
	case SH_CSS_BINARY_ID_VIDEO_NODZ:
		_init_binary_info(info, ISP_VIDEO_NODZ_);
		break;
	case SH_CSS_BINARY_ID_VIDEO_DS:
		_init_binary_info(info, ISP_VIDEO_DS_);
		break;
	default:
		return sh_css_err_invalid_arguments;
	}
	info->s3atbl_use_dmem = _S3ATBL_USE_DMEM(info->min_output_width !=
						 info->max_output_width);
	/* The ISP uses the veceven module for output, on the host however
	 * we don't want to know about it. We treat preview output as regular
	 * output, not as viewfinder output. */
	if (info->mode == SH_CSS_BINARY_MODE_PREVIEW)
		info->enable_vf_veceven = false;
	info->variable_vf_veceven = info->mode == SH_CSS_BINARY_MODE_COPY;
	max_internal_width =
		__ISP_INTERNAL_WIDTH(info->max_output_width,
				     info->max_dvs_envelope_width,
				     info->left_cropping,
				     info->mode,
				     info->c_subsampling,
				     info->output_num_chunks,
				     info->pipelining,
				     supports_output_format(info,
				       SH_CSS_FRAME_FORMAT_RGBA888));
	info->max_input_width = _ISP_MAX_INPUT_WIDTH(max_internal_width,
						     info->enable_ds);
	info->xmem_addr = NULL;
	info->next      = NULL;
	return load_binary(info, binary_found);
}

/* When binaries are put at the beginning, they will only
 * be selected if no other primary matches.
 */
enum sh_css_err
sh_css_init_binary_infos(void)
{
	int i;

	for (i = 0; i < SH_CSS_BINARY_NUM_IDS; i++) {
		enum sh_css_err ret;
		struct sh_css_binary_info *binary = &all_binaries[i];
		bool binary_found;

		binary->id = (enum sh_css_binary_id) i;
		ret = init_binary_info(binary, &binary_found);
		if (ret != sh_css_success)
			return ret;
		if (!binary_found)
			continue;
		/* Prepend new binary information */
		binary->next = binary_infos[binary->mode];
		binary_infos[binary->mode] = binary;
	}
	return sh_css_success;
}

enum sh_css_err
sh_css_binary_uninit(void)
{
	unsigned int i;
	struct sh_css_binary_info *b;

	for (i = 0; i < SH_CSS_BINARY_NUM_MODES; i++) {
		for (b = binary_infos[i]; b; b = b->next) {
			if (b->xmem_addr)
				hrt_isp_css_mm_free(b->xmem_addr);
			b->xmem_addr = NULL;
		}
		binary_infos[i] = NULL;
	}
	return sh_css_success;
}

static int
sh_css_grid_deci_factor_log2(int width, int height)
{
	int fact, fact1;
	fact = 5;
	while (ISP_BQ_GRID_WIDTH(width, fact - 1) <= SH_CSS_MAX_BQ_GRID_WIDTH &&
	       ISP_BQ_GRID_HEIGHT(height, fact - 1) <= SH_CSS_MAX_BQ_GRID_HEIGHT
	       && fact > 3)
		fact--;

	/* fact1 satisfies the specification of grid size. fact and fact1 is
	   not the same for some resolution (fact=4 and fact1=5 for 5mp). */
	if (width >= 2560)
		fact1 = 5;
	else if (width >= 1280)
		fact1 = 4;
	else
		fact1 = 3;
	return max(fact, fact1);
}

static enum sh_css_err
fill_binary_info(const struct sh_css_binary_info *info,
		 bool online,
		 bool two_ppc,
		 enum sh_css_input_format stream_format,
		 const struct sh_css_frame_info *in_info,
		 const struct sh_css_frame_info *out_info,
		 const struct sh_css_frame_info *vf_info,
		 struct sh_css_binary *binary)
{
	unsigned int dvs_env_width = 0,
		     dvs_env_height = 0,
		     vf_log_ds = 0,
		     s3a_log_deci = 0,
		     bits_per_pixel = in_info->raw_bit_depth,
		     ds_input_width = 0,
		     ds_input_height = 0,
		     isp_input_width,
		     isp_input_height,
		     isp_internal_width,
		     isp_internal_height,
		     isp_output_width  = out_info->padded_width,
		     isp_output_height = out_info->height,
		     s3a_isp_width;
	bool enable_ds = info->enable_ds;
	bool enable_hus = in_info->width < out_info->width;
	bool enable_vus = in_info->height < out_info->height;

	if (info->enable_dvs_envelope) {
		sh_css_video_get_dis_envelope(&dvs_env_width, &dvs_env_height);
		dvs_env_width  = MAX(dvs_env_width, SH_CSS_MIN_DVS_ENVELOPE);
		dvs_env_height = MAX(dvs_env_height, SH_CSS_MIN_DVS_ENVELOPE);
		binary->dvs_envelope_width  = dvs_env_width;
		binary->dvs_envelope_height = dvs_env_height;
	}
	if (vf_info) {
		enum sh_css_err err;
		err = sh_css_vf_downscale_log2(out_info, vf_info, &vf_log_ds);
		if (err != sh_css_success)
			return err;
		vf_log_ds = min(vf_log_ds, info->max_vf_log_downscale);
	}
	if (online) {
		bits_per_pixel = sh_css_input_format_bits_per_pixel(
					stream_format, two_ppc);
	}
	ds_input_width  = in_info->padded_width + info->left_cropping;
	ds_input_height = in_info->height + info->top_cropping;
	if (enable_hus)
		ds_input_width  += dvs_env_width;
	if (enable_vus)
		ds_input_height += dvs_env_height;

	/* We first calculate the resolutions used by the ISP. After that,
	 * we use those resolutions to compute sizes for tables etc. */
	isp_internal_width  =
		__ISP_INTERNAL_WIDTH(isp_output_width, dvs_env_width,
				     info->left_cropping, info->mode,
				     info->c_subsampling,
				     info->output_num_chunks, info->pipelining,
				     out_info->format ==
				       SH_CSS_FRAME_FORMAT_RGBA888);
	isp_internal_height =
		__ISP_INTERNAL_HEIGHT(isp_output_height, info->top_cropping,
				      dvs_env_height);
	isp_input_width = _ISP_INPUT_WIDTH(isp_internal_width,
					   ds_input_width,
					   enable_ds || enable_hus);
	isp_input_height = _ISP_INPUT_HEIGHT(isp_internal_height,
					     ds_input_height,
					     enable_ds || enable_vus);

	s3a_isp_width = _ISP_S3A_ELEMS_ISP_WIDTH(isp_input_width,
		isp_internal_width, enable_hus, info->left_cropping);
	if (info->fixed_s3a_deci_log)
		s3a_log_deci = info->fixed_s3a_deci_log;
	else
		s3a_log_deci = sh_css_grid_deci_factor_log2(s3a_isp_width,
							    isp_input_height);

	binary->vf_downscale_log2 = vf_log_ds;
	binary->deci_factor_log2  = s3a_log_deci;
	binary->input_buf_vectors =
			SH_CSS_NUM_INPUT_BUF_LINES * _ISP_VECS(isp_input_width);
	binary->online            = online;
	binary->input_format      = stream_format;
	/* input info */
	binary->in_frame_info.format = in_info->format;
	binary->in_frame_info.width = in_info->width + info->left_cropping +
				      dvs_env_width;
	binary->in_frame_info.padded_width  = isp_input_width;
	binary->in_frame_info.height        = isp_input_height;
	binary->in_frame_info.raw_bit_depth = bits_per_pixel;
	/* internal frame info */
	binary->internal_frame_info.format          = out_info->format;
	binary->internal_frame_info.width           = isp_internal_width;
	binary->internal_frame_info.padded_width    = isp_internal_width;
	binary->internal_frame_info.height          = isp_internal_height;
	binary->internal_frame_info.raw_bit_depth   = bits_per_pixel;
	/* output info */
	binary->out_frame_info.format        = out_info->format;
	binary->out_frame_info.width         = out_info->width;
	binary->out_frame_info.padded_width  = isp_output_width;
	binary->out_frame_info.height        = isp_output_height;
	binary->out_frame_info.raw_bit_depth = bits_per_pixel;

	/* viewfinder output info */
	binary->vf_frame_info.format = SH_CSS_FRAME_FORMAT_YUV_LINE;
	if (vf_info) {
		unsigned int vf_out_vecs, vf_out_width, vf_out_height;
		vf_out_vecs = __ISP_VF_OUTPUT_WIDTH_VECS(isp_output_width,
							 vf_log_ds);
		vf_out_width = _ISP_VF_OUTPUT_WIDTH(vf_out_vecs);
		vf_out_height = _ISP_VF_OUTPUT_HEIGHT(isp_output_height,
						      vf_log_ds);
		/* we also store the raw downscaled width. This is used for
		 * digital zoom in preview to zoom only on the width that
		 * we actually want to keep, not on the aligned width. */
		binary->vf_frame_info.width = (out_info->width >> vf_log_ds);
		binary->vf_frame_info.padded_width = vf_out_width;
		binary->vf_frame_info.height       = vf_out_height;
	} else {
		binary->vf_frame_info.width        = 0;
		binary->vf_frame_info.padded_width = 0;
		binary->vf_frame_info.height       = 0;
	}
	if (info->mode == SH_CSS_BINARY_MODE_GDC) {
		binary->morph_tbl_width =
			_ISP_MORPH_TABLE_WIDTH(isp_internal_width);
		binary->morph_tbl_aligned_width  =
			_ISP_MORPH_TABLE_ALIGNED_WIDTH(isp_internal_width);
		binary->morph_tbl_height =
			_ISP_MORPH_TABLE_HEIGHT(isp_internal_height);
	} else {
		binary->morph_tbl_width  = 0;
		binary->morph_tbl_aligned_width  = 0;
		binary->morph_tbl_height = 0;
	}
	if (info->enable_sc)
		binary->sctbl_width_per_color =
			SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR;
	else
		binary->sctbl_width_per_color = 0;

	if (info->enable_s3a) {
		binary->s3atbl_width  =
			_ISP_S3ATBL_WIDTH(binary->in_frame_info.width,
				s3a_log_deci);
		binary->s3atbl_height =
			_ISP_S3ATBL_HEIGHT(binary->in_frame_info.height,
				s3a_log_deci);
		binary->s3atbl_isp_width =
			_ISP_S3ATBL_ISP_WIDTH(
				_ISP_S3A_ELEMS_ISP_WIDTH(isp_input_width,
				isp_internal_width, enable_hus,
				info->left_cropping),
				s3a_log_deci);
		binary->s3atbl_isp_height =
			_ISP_S3ATBL_ISP_HEIGHT(
				_ISP_S3A_ELEMS_ISP_HEIGHT(isp_input_height,
				isp_internal_height, enable_vus), s3a_log_deci);
	} else {
		binary->s3atbl_width  = 0;
		binary->s3atbl_height = 0;
		binary->s3atbl_isp_width  = 0;
		binary->s3atbl_isp_height = 0;
	}

	if (info->enable_sc) {
		binary->sctbl_width_per_color  =
			_ISP_SCTBL_WIDTH_PER_COLOR(isp_input_width,
						   s3a_log_deci);
		binary->sctbl_aligned_width_per_color =
			SH_CSS_MAX_SCTBL_ALIGNED_WIDTH_PER_COLOR;
		binary->sctbl_height =
			_ISP_SCTBL_HEIGHT(isp_input_height, s3a_log_deci);
	} else {
		binary->sctbl_width_per_color         = 0;
		binary->sctbl_aligned_width_per_color = 0;
		binary->sctbl_height                  = 0;
	}
	if (info->enable_dis) {
		binary->dis_deci_factor_log2 = SH_CSS_DIS_DECI_FACTOR_LOG2;
		binary->dis_hor_coef_num_3a  =
			_ISP_SDIS_HOR_COEF_NUM_3A(binary->in_frame_info.width,
						  SH_CSS_DIS_DECI_FACTOR_LOG2);
		binary->dis_ver_coef_num_3a  =
			_ISP_SDIS_VER_COEF_NUM_3A(binary->in_frame_info.height,
						  SH_CSS_DIS_DECI_FACTOR_LOG2);
		binary->dis_hor_coef_num_isp =
			_ISP_SDIS_HOR_COEF_NUM_ISP(
				_ISP_SDIS_ELEMS_ISP(isp_input_width,
				isp_internal_width, enable_hus));
		binary->dis_ver_coef_num_isp =
			_ISP_SDIS_VER_COEF_NUM_ISP(
				_ISP_SDIS_ELEMS_ISP(isp_input_height,
				isp_internal_height, enable_vus));
		binary->dis_hor_proj_num_3a  =
			_ISP_SDIS_HOR_PROJ_NUM_3A(binary->in_frame_info.height,
						  SH_CSS_DIS_DECI_FACTOR_LOG2);
		binary->dis_ver_proj_num_3a  =
			_ISP_SDIS_VER_PROJ_NUM_3A(binary->in_frame_info.width,
						  SH_CSS_DIS_DECI_FACTOR_LOG2);
		binary->dis_hor_proj_num_isp =
			__ISP_SDIS_HOR_PROJ_NUM_ISP(
				_ISP_SDIS_ELEMS_ISP(isp_input_height,
				isp_internal_height, enable_vus),
						SH_CSS_DIS_DECI_FACTOR_LOG2);
		binary->dis_ver_proj_num_isp =
			__ISP_SDIS_VER_PROJ_NUM_ISP(
				_ISP_SDIS_ELEMS_ISP(isp_input_width,
				isp_internal_width, enable_hus),
						SH_CSS_DIS_DECI_FACTOR_LOG2);
	} else {
		binary->dis_deci_factor_log2 = 0;
		binary->dis_hor_coef_num_3a  = 0;
		binary->dis_ver_coef_num_3a  = 0;
		binary->dis_hor_coef_num_isp = 0;
		binary->dis_ver_coef_num_isp = 0;
		binary->dis_hor_proj_num_3a  = 0;
		binary->dis_ver_proj_num_3a  = 0;
		binary->dis_hor_proj_num_isp = 0;
		binary->dis_ver_proj_num_isp = 0;
	}
	if (info->left_cropping)
		binary->left_padding = 2 * ISP_VEC_NELEMS - info->left_cropping;
	else
		binary->left_padding = 0;

	binary->info = info;
	return sh_css_success;
}
enum sh_css_err
sh_css_binary_find(struct sh_css_binary_descr *descr,
		   struct sh_css_binary *binary)
{
	int mode = descr->mode;
	bool online = descr->online;
	bool two_ppc = descr->two_ppc;
	enum sh_css_input_format stream_format = descr->stream_format;
	const struct sh_css_frame_info *req_in_info = descr->in_info,
				       *req_out_info = descr->out_info,
				       *req_vf_info = descr->vf_info;
	struct sh_css_binary_info *candidate;
	unsigned int dvs_envelope_width = 0,
		     dvs_envelope_height = 0;
	bool need_ds = false,
	     need_dz = false,
	     need_dvs = false;
	enum sh_css_err err = sh_css_success;

	if (mode == SH_CSS_BINARY_MODE_VIDEO) {
		unsigned int dx, dy;
		sh_css_get_zoom_factor(&dx, &dy);
		sh_css_video_get_dis_envelope(&dvs_envelope_width,
					      &dvs_envelope_height);

		/* Video is the only mode that has a nodz variant. */
		need_dz = ((dx != UDS_SCALING_N) || (dy != UDS_SCALING_N));
		need_dvs = dvs_envelope_width || dvs_envelope_height;
	}

	need_ds = req_in_info->width > req_out_info->width ||
		  req_in_info->height > req_out_info->height;

	for (candidate = binary_infos[mode]; candidate;
	     candidate = candidate->next) {
		if (candidate->enable_vf_veceven && !req_vf_info)
			continue;
		if (req_vf_info && !(candidate->enable_vf_veceven ||
				     candidate->variable_vf_veceven))
			continue;
		if (!candidate->enable_dvs_envelope && need_dvs)
			continue;
		if (dvs_envelope_width > candidate->max_dvs_envelope_width)
			continue;
		if (dvs_envelope_height > candidate->max_dvs_envelope_height)
			continue;
		if (!candidate->enable_ds && need_ds)
			continue;
		if (!candidate->enable_uds && need_dz)
			continue;
		if (online && candidate->input == SH_CSS_BINARY_INPUT_MEMORY)
			continue;
		if (!online && candidate->input == SH_CSS_BINARY_INPUT_SENSOR)
			continue;
		if (req_out_info->padded_width < candidate->min_output_width ||
		    req_out_info->padded_width > candidate->max_output_width)
			continue;

		if (req_in_info->padded_width > candidate->max_input_width)
			continue;

		if (!supports_output_format(candidate, req_out_info->format))
			continue;

		/* reconfigure any variable properties of the binary */
		err = fill_binary_info(candidate, online, two_ppc,
				       stream_format, req_in_info,
				       req_out_info, req_vf_info,
				       binary);
		if (err)
			return err;
		init_metrics(&binary->metrics, binary->info->id);
		return sh_css_success;
	}
	return sh_css_err_internal_error;
}
