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

#ifndef _SH_CSS_PARAMS_H_
#define _SH_CSS_PARAMS_H_

/*! \file */

#include "sh_css.h"

/* TODO: remove most of the individual parameter functions.
 * These will be combined in sh_css_set_isp_config.
 * The remaining function should get an extra pipe-id argument.
 */

int sh_css_get_gdc_coord_one(void);

/* DIS */
/* get the pointers to the dis coefficient tables.
 * These tables will then be written by the caller and the
 * values will be sent to the ISP upon the next start of frame.
 */
void sh_css_set_dis_coefficients(
	const short *horizontal_coefs,
	const short *vertical_coefs);

void sh_css_get_dis_projections(
	int *horizontal_projections,
	int *vertical_projections,
	struct sh_css_dis_data *dis_data);

/* 3A */
enum sh_css_err sh_css_get_3a_statistics(
	struct sh_css_3a_output *output,
	bool use_dmem,
	union sh_css_s3a_data *s3a_data);

void sh_css_set_3a_config(
	const struct sh_css_3a_config *config);

void sh_css_get_3a_config(
	const struct sh_css_3a_config **config);

/** @brief Configure an image pipe with filter coefficients.
 *
 * @param[in]	pipe	The pipe to be configured.
 * @param[in]	config	The set of filter coefficients.
 * @return		IA_CSS_SUCCESS or error code upon error.
 *
 * This function configures the filter coefficients for an image
 * pipe. For image pipes that do not execute any ISP filters, this
 * function will have no effect.
 * It is safe to call this function while the image pipe is running,
 * in fact this is the expected behavior most of the time. Proper
 * resource locking and double buffering is in place to allow for this.
 */
void
sh_css_set_isp_config(enum sh_css_pipe_id pipe,
		      const struct sh_css_isp_config *config);

void
sh_css_get_isp_config(enum sh_css_pipe_id pipe,
		      const struct sh_css_isp_config **config);

/* FPN */
enum sh_css_err
sh_css_set_black_frame(const struct sh_css_frame *raw_black_frame);

void
sh_css_set_morph_table(const struct sh_css_morph_table *table);

void
sh_css_get_morph_table(const struct sh_css_morph_table **table);

struct sh_css_morph_table *
sh_css_morph_table_allocate(unsigned int width, unsigned int height);

void
sh_css_morph_table_free(struct sh_css_morph_table *me);

/* White Balance */
void
sh_css_set_wb_config(const struct sh_css_wb_config *wb_config);

void
sh_css_get_wb_config(const struct sh_css_wb_config **wb_config);

/* Color Correction */
void
sh_css_set_cc_config(const struct sh_css_cc_config *cc_config);

void
sh_css_get_cc_config(const struct sh_css_cc_config **cc_config);

/* TNR */
void
sh_css_set_tnr_config(const struct sh_css_tnr_config *tnr_config);

void
sh_css_get_tnr_config(const struct sh_css_tnr_config **tnr_config);

/* ANR */
void
sh_css_set_anr_config(const struct sh_css_anr_config *anr_config);

void
sh_css_get_anr_config(const struct sh_css_anr_config **anr_config);

/* Objective Black */
void
sh_css_set_ob_config(const struct sh_css_ob_config *ob_config);

void
sh_css_get_ob_config(const struct sh_css_ob_config **ob_config);

/* Dead Pixel */
void
sh_css_set_dp_config(const struct sh_css_dp_config *dp_config);

void
sh_css_get_dp_config(const struct sh_css_dp_config **dp_config);

/* Noise Reduction */
void
sh_css_set_nr_config(const struct sh_css_nr_config *nr_config);

void
sh_css_get_nr_config(const struct sh_css_nr_config **nr_config);

/* Edge Enhancement */
void
sh_css_set_ee_config(const struct sh_css_ee_config *ee_config);

void
sh_css_get_ee_config(const struct sh_css_ee_config **ee_config);

/* Demosaic */
void
sh_css_set_de_config(const struct sh_css_de_config *de_config);

void
sh_css_get_de_config(const struct sh_css_de_config **de_config);

/* Color Enhancement */
void
sh_css_set_ce_config(const struct sh_css_ce_config *ce_config);

void
sh_css_get_ce_config(const struct sh_css_ce_config **ce_config);

/* Gamma Correction */
void
sh_css_set_gc_config(const struct sh_css_gc_config *gc_config);

void
sh_css_get_gc_config(const struct sh_css_gc_config **gc_config);

void
sh_css_set_gamma_table(const struct sh_css_gamma_table *table);

void
sh_css_get_gamma_table(const struct sh_css_gamma_table **table);

void
sh_css_set_ctc_table(const struct sh_css_ctc_table *table);

void
sh_css_get_ctc_table(const struct sh_css_ctc_table **table);

void
sh_css_set_xnr_table(const struct sh_css_xnr_table *table);

void
sh_css_get_xnr_table(const struct sh_css_xnr_table **table);

/* Multi-Access Color Correction */
void
sh_css_set_macc_table(const struct sh_css_macc_table *table);

void
sh_css_get_macc_table(const struct sh_css_macc_table **table);

/* Shading Correction */
bool
sh_css_params_set_binning_factor(unsigned int sensor_binning);

bool
sh_css_params_set_shading_table(
	const struct sh_css_shading_table *table);

void sh_css_shading_table_free(
	struct sh_css_shading_table *table);

struct sh_css_shading_table *
sh_css_shading_table_alloc(unsigned int width,
			   unsigned int height);

struct sh_css_s3a_dis_buffer_info {
	bool enable_s3a;
	bool enable_dis;
	bool s3atbl_use_dmem;
	int  s3atbl_width;
	int  s3atbl_height;
	int  s3atbl_isp_width;
	int  s3atbl_isp_height;
	int  dis_hor_proj_num_isp;
	int  dis_ver_proj_num_isp;
	int  dis_hor_proj_num_3a;
	int  dis_ver_proj_num_3a;
	int  deci_factor_log2;
};

extern enum sh_css_err sh_css_reallocate_stat_buffers_from_info(
	union sh_css_s3a_data *s3a_ptr,
	struct sh_css_dis_data *dvs_ptr,
	size_t *curr_size_s3a,
	size_t *curr_size_dvs,
	const struct sh_css_grid_info *grid);

enum sh_css_err sh_css_allocate_stat_buffers_from_info(
	union sh_css_s3a_data *s3a_ptr,
	struct sh_css_dis_data *dvs_ptr,
	const struct sh_css_grid_info *info);

void
sh_css_free_stat_buffers(union sh_css_s3a_data *s3a_ptr,
	struct sh_css_dis_data *dis_ptr);

#endif /* _SH_CSS_PARAMS_H_ */
