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

#ifndef _SH_CSS_BINARY_H_
#define _SH_CSS_BINARY_H_

#include "sh_css_types.h"
#include "sh_css_metrics.h"

/* The binary mode is used in pre-processor expressions so we cannot
 * use an enum here. */
#define SH_CSS_BINARY_MODE_COPY       0
#define SH_CSS_BINARY_MODE_PREVIEW    1
#define SH_CSS_BINARY_MODE_PRIMARY    2
#define SH_CSS_BINARY_MODE_VIDEO      3
#define SH_CSS_BINARY_MODE_PRE_ISP    4
#define SH_CSS_BINARY_MODE_GDC        5
#define SH_CSS_BINARY_MODE_POST_ISP   6
#define SH_CSS_BINARY_MODE_ANR        7
#define SH_CSS_BINARY_MODE_CAPTURE_PP 8
#define SH_CSS_BINARY_MODE_BAYER_DS   9
#define SH_CSS_BINARY_MODE_VF_PP      10
#define SH_CSS_BINARY_NUM_MODES       11
#define SH_CSS_BINARY_MODE_NONE       12

/* Indicate where binaries can read input from */
#define SH_CSS_BINARY_INPUT_SENSOR   0
#define SH_CSS_BINARY_INPUT_MEMORY   1
#define SH_CSS_BINARY_INPUT_VARIABLE 2

/* ISP binary identifiers.
   these determine the order in which the binaries are looked up, do not change
   this!
   Also, the SP firmware uses this same order (sp_main_funcs in sp.hive.c).
   Also, gen_firmware.c uses this order in its firmware_header.
*/
enum sh_css_binary_id {
	SH_CSS_BINARY_ID_COPY,
	SH_CSS_BINARY_ID_BAYER_DS,
	SH_CSS_BINARY_ID_VF_PP,
	SH_CSS_BINARY_ID_CAPTURE_PP,
	SH_CSS_BINARY_ID_PRE_ISP,
	SH_CSS_BINARY_ID_GDC,
	SH_CSS_BINARY_ID_POST_ISP,
	SH_CSS_BINARY_ID_ANR,
	SH_CSS_BINARY_ID_PREVIEW_DS,
	SH_CSS_BINARY_ID_PREVIEW_DZ,
	SH_CSS_BINARY_ID_PRIMARY_DS,
	SH_CSS_BINARY_ID_PRIMARY_VAR,
	SH_CSS_BINARY_ID_PRIMARY_SMALL,
	SH_CSS_BINARY_ID_PRIMARY_8MP,
	SH_CSS_BINARY_ID_PRIMARY_14MP,
	SH_CSS_BINARY_ID_PRIMARY_16MP,
	SH_CSS_BINARY_ID_PRIMARY_REF,
	SH_CSS_BINARY_ID_VIDEO_OFFLINE,
	SH_CSS_BINARY_ID_VIDEO_DS,
	SH_CSS_BINARY_ID_VIDEO_DZ,
	SH_CSS_BINARY_ID_VIDEO_NODZ,
	SH_CSS_BINARY_NUM_IDS,
};

#define SH_CSS_BINARY_ID(BINARY) SH_CSS_BINARY_##BINARY
/* The maximum number of different frame formats any binary can support */
#define SH_CSS_MAX_NUM_FRAME_FORMATS 18

struct sh_css_binary_descr {
	int mode;
	bool online;
	bool continuous;
	bool two_ppc;
	enum sh_css_input_format stream_format;
	struct sh_css_frame_info *in_info;
	struct sh_css_frame_info *out_info;
	struct sh_css_frame_info *vf_info;
};

struct sh_css_binary;

struct sh_css_binary_info {
	enum sh_css_binary_id    id;
	int                      mode;
	int                      num_output_formats;
	enum sh_css_frame_format output_formats[SH_CSS_MAX_NUM_FRAME_FORMATS];
	unsigned int             max_input_width;
	unsigned int             min_output_width;
	unsigned int             max_output_width;
	unsigned int             max_dvs_envelope_width;
	unsigned int             max_dvs_envelope_height;
	bool                     variable_vf_veceven;
	unsigned int             max_vf_log_downscale;
	bool                     enable_vf_veceven;
	bool                     enable_dis;
	bool                     enable_dvs_envelope;
	bool                     enable_uds;
	bool                     enable_ds;
	bool                     enable_s3a;
	bool                     enable_fpnr;
	bool                     enable_sc;
	unsigned int             top_cropping;
	unsigned int             left_cropping;
	int                      s3atbl_use_dmem;
	int                      input;
	void                    *xmem_addr;
	unsigned int             c_subsampling;
	unsigned int             output_num_chunks;
	unsigned int             pipelining;
	unsigned int             fixed_s3a_deci_log;
	struct sh_css_binary_info *next;
};

struct sh_css_binary {
	const struct sh_css_binary_info *info;
	enum sh_css_input_format input_format;
	struct sh_css_frame_info in_frame_info;
	struct sh_css_frame_info internal_frame_info;
	struct sh_css_frame_info out_frame_info;
	struct sh_css_frame_info vf_frame_info;
	int                      input_buf_vectors;
	int                      deci_factor_log2;
	int                      dis_deci_factor_log2;
	int                      vf_downscale_log2;
	int                      s3atbl_width;
	int                      s3atbl_height;
	int                      s3atbl_isp_width;
	int                      s3atbl_isp_height;
	unsigned int             morph_tbl_width;
	unsigned int             morph_tbl_aligned_width;
	unsigned int             morph_tbl_height;
	int                      sctbl_width_per_color;
	int                      sctbl_aligned_width_per_color;
	int                      sctbl_height;
	int                      dis_hor_coef_num_3a;
	int                      dis_ver_coef_num_3a;
	int                      dis_hor_coef_num_isp;
	int                      dis_ver_coef_num_isp;
	int                      dis_hor_proj_num_3a;
	int                      dis_ver_proj_num_3a;
	int                      dis_hor_proj_num_isp;
	int                      dis_ver_proj_num_isp;
	unsigned int             dvs_envelope_width;
	unsigned int             dvs_envelope_height;
	bool                     online;
	unsigned int             uds_xc;
	unsigned int             uds_yc;
	unsigned int             left_padding;
	struct sh_css_binary_metrics metrics;
};

enum sh_css_err
sh_css_init_binary_infos(void);

enum sh_css_err
sh_css_binary_uninit(void);

enum sh_css_err
sh_css_binary_find(struct sh_css_binary_descr *descr,
		   struct sh_css_binary *binary);

enum sh_css_err
sh_css_binary_grid_info(struct sh_css_binary *binary,
			struct sh_css_grid_info *info);

void sh_css_binary_init(struct sh_css_binary *binary);

#endif /* _SH_CSS_BINARY_H_ */
