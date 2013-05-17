#ifndef _SH_CSS_PARAMS_H_
#define _SH_CSS_PARAMS_H_

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

#include "ia_css_types.h"
#include "ia_css.h"
#include "sh_css_internal.h"
#include "sh_css_legacy.h"

struct ia_css_isp_parameters {
	struct sh_css_isp_params isp_parameters;
	struct ia_css_fpn_table fpn_table;
	struct ia_css_vector motion_config;
	const struct ia_css_morph_table   *morph_table;
	const struct ia_css_shading_table *sc_table;
	struct ia_css_macc_table    macc_table;
	struct ia_css_gamma_table   gamma_table;
	struct ia_css_ctc_table     ctc_table;
	struct ia_css_xnr_table     xnr_table;
	struct ia_css_dz_config     dz_config;
	struct ia_css_3a_config     s3a_config;
	struct ia_css_wb_config     wb_config;
	struct ia_css_cc_config     cc_config[N_CSC_KERNEL_PARAM_SET];
	struct ia_css_tnr_config    tnr_config;
	struct ia_css_ob_config     ob_config;
	struct ia_css_dp_config     dp_config;
	struct ia_css_nr_config     nr_config;
	struct ia_css_ee_config     ee_config;
	struct ia_css_de_config     de_config;
	struct ia_css_gc_config     gc_config;
	struct ia_css_anr_config    anr_config;
	struct ia_css_ce_config     ce_config;
	
	struct ia_css_dvs_6axis_config	*dvs_6axis_config;
	
	struct ia_css_ecd_config    ecd_config;
	struct ia_css_ynr_config    ynr_config;
	struct ia_css_fc_config     fc_config;
	struct ia_css_cnr_config    cnr_config;
	struct ia_css_macc_config   macc_config;
	struct ia_css_ctc_config    ctc_config;
	struct ia_css_aa_config     aa_config;
	struct ia_css_rgb_gamma_table     r_gamma_table;
	struct ia_css_rgb_gamma_table     g_gamma_table;
	struct ia_css_rgb_gamma_table     b_gamma_table;
	struct ia_css_anr_thres     anr_thres;
/*
	struct ia_css_yuv2rgb_cc_config   yuv2rgb_cc_config;
	struct ia_css_rgb2yuv_cc_config   rgb2yuv_cc_config;
*/
	bool isp_params_changed;
	bool fpn_table_changed;
	bool dz_config_changed;
	bool motion_config_changed;
	bool dis_coef_table_changed;
	bool dvs2_coef_table_changed;
	bool morph_table_changed;
	bool sc_table_changed;
	bool macc_table_changed;
	bool gamma_table_changed;
	bool ctc_table_changed;
	bool xnr_table_changed;
	bool s3a_config_changed;
	bool wb_config_changed;
	bool cc_config_changed[N_CSC_KERNEL_PARAM_SET];
	bool tnr_config_changed;
	bool ob_config_changed;
	bool dp_config_changed;
	bool nr_config_changed;
	bool ee_config_changed;
	bool de_config_changed;
	bool gc_config_changed;
	bool anr_config_changed;
	bool ce_config_changed;
	bool dvs_6axis_config_changed;
	bool ecd_config_changed;
	bool ynr_config_changed;
	bool fc_config_changed;
	bool cnr_config_changed;
	bool macc_config_changed;
	bool ctc_config_changed;
	bool aa_config_changed;
	bool r_gamma_table_changed;
	bool g_gamma_table_changed;
	bool b_gamma_table_changed;
	bool anr_thres_changed;
/*
	bool yuv2rgb_cc_config_changed;
	bool rgb2yuv_cc_config_changed;
*/
	unsigned int sensor_binning;
	/* local buffers, used to re-order the 3a statistics in vmem-format */
	const short *dis_hor_coef_tbl;
	const short *dis_ver_coef_tbl;
	struct ia_css_dvs2_coef_types dvs2_hor_coefs;
	struct ia_css_dvs2_coef_types dvs2_ver_coefs;
	struct sh_css_ddr_address_map pipe_ddr_ptrs[IA_CSS_PIPE_ID_NUM];
	struct sh_css_ddr_address_map_size pipe_ddr_ptrs_size[IA_CSS_PIPE_ID_NUM];
	struct sh_css_ddr_address_map ddr_ptrs;
	struct sh_css_ddr_address_map_size ddr_ptrs_size;
};

#endif /* _SH_CSS_PARAMS_H_ */
