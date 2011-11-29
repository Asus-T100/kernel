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

#include "sh_css_types.h"

int sh_css_get_gdc_coord_one(void);

/* DIS */
/* get the pointers to the dis coefficient tables.
 * These tables will then be written by the caller and the
 * values will be sent to the ISP upon the next start of frame.
 */
void
sh_css_set_dis_coefficients(const short *horizontal_coefs,
			    const short *vertical_coefs);

void
sh_css_get_dis_projections(int *horizontal_projections,
			   int *vertical_projections);

/* 3A */
enum sh_css_err
sh_css_get_3a_statistics(struct sh_css_3a_output *output);

void
sh_css_set_3a_config(const struct sh_css_3a_config *config);

void
sh_css_get_3a_config(const struct sh_css_3a_config **config);

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

/* Multi-Access Color Correction */
void
sh_css_set_macc_table(const struct sh_css_macc_table *table);

void
sh_css_get_macc_table(const struct sh_css_macc_table **table);

/* Shading Correction */
void
sh_css_params_set_shading_table(const struct sh_css_shading_table *table);

void
sh_css_shading_table_free(struct sh_css_shading_table *table);

struct sh_css_shading_table *
sh_css_shading_table_alloc(unsigned int width,
			   unsigned int height);

#endif /* _SH_CSS_PARAMS_H_ */
