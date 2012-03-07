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

#include "sh_css_types.h"
#include "sh_css_binary.h"

#define sh_css_print(fmt, s...) \
	do { \
		if (sh_css_printf) { \
			sh_css_printf(fmt, ## s); \
		} \
	} while (0)

/* Data structures shared with ISP */
struct sh_css_isp_params {
	/* FPNR (Fixed Pattern Noise Reduction) */
	int fpn_shift;
	int fpn_enabled;

	/* OB (Optical Black) */
	int ob_blacklevel_gr;
	int ob_blacklevel_r;
	int ob_blacklevel_b;
	int ob_blacklevel_gb;
	int obarea_start_bq;
	int obarea_length_bq;
	int obarea_length_bq_inverse;

	/* SC (Shading Corrction) */
	int sc_gain_shift;

	/* WB (White Balance) */
	int wb_gain_shift;
	int wb_gain_gr;
	int wb_gain_r;
	int wb_gain_b;
	int wb_gain_gb;

	/* DP (Defect Pixel Correction) */
	int dp_threshold_single_when_2adjacent_on;
	int dp_threshold_2adjacent_when_2adjacent_on;
	int dp_threshold_single_when_2adjacent_off;
	int dp_threshold_2adjacent_when_2adjacent_off;
	int dp_gain;

	/* BNR (Bayer Noise Reduction) */
	int bnr_gain_all;
	int bnr_gain_dir;
	int bnr_threshold_low;
	int bnr_threshold_width_log2;
	int bnr_threshold_width;
	int bnr_clip;

	/* S3A (3A Support): coefficients to calculate Y */
	int ae_y_coef_r;
	int ae_y_coef_g;
	int ae_y_coef_b;

	/* S3A (3A Support): af fir coefficients */
	int af_fir1[7];
	int af_fir2[7];

	/* DE (Demosaic) */
	int de_pixelnoise;
	int de_c1_coring_threshold;
	int de_c2_coring_threshold;

	/* YNR (Y Noise Reduction), YEE (Y Edge Enhancement) */
	int ynr_threshold;
	int ynr_gain_all;
	int ynr_gain_dir;
	int ynryee_dirthreshold_s;
	int ynryee_dirthreshold_g;
	int ynryee_dirthreshold_width_log2;
	int ynryee_dirthreshold_width;
	int yee_detailgain;
	int yee_coring_s;
	int yee_coring_g;
	int yee_scale_plus_s;
	int yee_scale_plus_g;
	int yee_scale_minus_s;
	int yee_scale_minus_g;
	int yee_clip_plus_s;
	int yee_clip_plus_g;
	int yee_clip_minus_s;
	int yee_clip_minus_g;
	int ynryee_Yclip;

	/* CSC (Color Space Conversion) */
	/* YC1C2->YCbCr */
	int csc_coef_shift;
	int yc1c2_to_ycbcr_00;
	int yc1c2_to_ycbcr_01;
	int yc1c2_to_ycbcr_02;
	int yc1c2_to_ycbcr_10;
	int yc1c2_to_ycbcr_11;
	int yc1c2_to_ycbcr_12;
	int yc1c2_to_ycbcr_20;
	int yc1c2_to_ycbcr_21;
	int yc1c2_to_ycbcr_22;

	/* GC (Gamma Correction) */
	int gamma_gain_k1;
	int gamma_gain_k2;

	/* TNR (Temporal Noise Reduction) */
	int tnr_coef;
	int tnr_threshold_Y;
	int tnr_threshold_C;

	/* ANR (Advance Noise Reduction) */
	int anr_threshold;

	/* CE (Chroma Enhancement) */
	int ce_uv_level_min;
	int ce_uv_level_max;
};

/* xmem address map allocation */
struct sh_css_ddr_address_map {
	void *isp_param;
	void *ctc_tbl;
	void *gamma_tbl;
	void *macc_tbl;
	void *fpn_tbl;
	void *sc_tbl;
	void *s3a_tbl;
	void *s3a_tbl_hi;
	void *s3a_tbl_lo;
	void *sdis_hor_coef;
	void *sdis_ver_coef;
	void *sdis_hor_proj;
	void *sdis_ver_proj;
	void *tetra_r_x;
	void *tetra_r_y;
	void *tetra_gr_x;
	void *tetra_gr_y;
	void *tetra_gb_x;
	void *tetra_gb_y;
	void *tetra_b_x;
	void *tetra_b_y;
	void *tetra_ratb_x;
	void *tetra_ratb_y;
	void *tetra_batr_x;
	void *tetra_batr_y;
};

struct sh_css_sp_debug_state {
	unsigned int error;
	unsigned int debug[16];
};

/* Input formatter descriptor */
struct sh_css_if_config {
	unsigned int start_line;
	unsigned int start_column;
	unsigned int left_padding;
	unsigned int cropped_height;
	unsigned int cropped_width;
	unsigned int deinterleaving;
	unsigned int buf_vecs;
	unsigned int buf_start_index;
	unsigned int buf_increment;
	unsigned int buf_eol_offset;
	unsigned int yuv420;
	unsigned int block_no_reqs;
};

/* Group all host initialized SP variables into this struct. */
struct sh_css_sp_group {
	int				sp_isp_binary_id;
	int				sp_enable_xnr;
	unsigned int			sp_uds_curr_dx;
	unsigned int			sp_uds_curr_dy;
	unsigned int			sp_uds_xc;
	unsigned int			sp_uds_yc;
	int				sp_input_stream_format;
	unsigned int			isp_dvs_envelope_width;
	unsigned int			isp_dvs_envelope_height;
	unsigned int			isp_deci_log_factor;
	unsigned int			isp_vf_downscale_bits;
	unsigned int			isp_online;
	unsigned int			isp_copy_vf;
	unsigned int			isp_copy_output;
	unsigned int			isp_2ppc;
	unsigned int			sp_out_crop_pos_x;
	unsigned int			sp_out_crop_pos_y;
	unsigned int			sp_run_copy;
	void				*xmem_bin_addr;
	void				*xmem_map_addr;
	char				*histo_addr;
	unsigned			anr;
	struct sh_css_frame		sp_in_frame;
	struct sh_css_frame		sp_out_frame;
	struct sh_css_frame		sp_ref_in_frame;
	struct sh_css_frame		sp_tnr_in_frame;
	struct sh_css_frame		sp_out_vf_frame;
	struct sh_css_frame		sp_extra_frame;
	struct sh_css_frame		sp_ref_out_frame;
	struct sh_css_frame		sp_tnr_out_frame;
	struct sh_css_frame_info	sp_internal_frame_info;
	struct {
		char			*out;
		unsigned int		bytes_available;
	} bin_copy;
	struct {
		char			*out;
		unsigned int		height;
		unsigned int		width;
		unsigned int		padded_width;
		unsigned int		max_input_width;
		unsigned int		raw_bit_depth;
	} raw_copy;
	struct {
		unsigned int		busy_wait;
	} dma_proxy;
	struct sh_css_if_config		if_config_a;
	struct sh_css_if_config		if_config_b;
	struct {
		unsigned int		width;
		unsigned int		height;
		unsigned int		hblank_cycles;
		unsigned int		vblank_cycles;
	} sync_gen;
	struct {
		unsigned int		x_mask;
		unsigned int		y_mask;
		unsigned int		x_delta;
		unsigned int		y_delta;
		unsigned int		xy_mask;
	} tpg;
	struct {
		unsigned int		seed;
	} prbs;
	struct {
		unsigned int		no_side_band;
		unsigned int		fmt_type;
		unsigned int		ch_id;
		enum sh_css_input_mode	input_mode;
	} input_circuit;
	struct sh_css_sp_overlay	overlay;
};

/* Data in SP dmem that is set from the host every frame. */

struct sh_css_sp_per_frame_data {
	 /* ddr address of sp_group */
	 struct sh_css_sp_group *sp_group_addr;
	 unsigned read_sp_group_from_ddr;
	 unsigned if_a_changed;
	 unsigned if_b_changed;
	 unsigned program_input_circuit;
};

extern struct sh_css_sp_group sp_group;
extern struct sh_css_frame sp_in_frame;
extern struct sh_css_frame sp_out_frame;

extern int (*sh_css_printf) (const char *fmt, ...);

void *
sh_css_params_ddr_address_map(void);

enum sh_css_err
sh_css_params_write_to_ddr(const struct sh_css_binary *binary_info);

void
sh_css_params_set_current_binary(const struct sh_css_binary *binary);

/* swap 3a double buffers. This should be called when handling an
   interrupt that indicates that statistics are ready.
   This also swaps the DIS buffers. */
void
sh_css_params_swap_3a_buffers(void);

enum sh_css_err
sh_css_params_init(void);

void
sh_css_params_uninit(void);

void
sh_css_params_reconfigure_gdc_lut(void);

void *
sh_css_malloc(size_t size);

void
sh_css_free(void *ptr);

/* For Acceleration API: Flush FW (shared buffer pointer) arguments */
extern void
sh_css_flush(struct sh_css_acc_fw *fw);

/* Check two frames for equality (format, resolution, bits per element) */
bool
sh_css_frame_equal_types(const struct sh_css_frame *frame_a,
			 const struct sh_css_frame *frame_b);

bool
sh_css_frame_info_equal_resolution(const struct sh_css_frame_info *info_a,
				   const struct sh_css_frame_info *info_b);

unsigned int
sh_css_input_format_bits_per_pixel(enum sh_css_input_format format,
				   bool two_ppc);

enum sh_css_err
sh_css_vf_downscale_log2(const struct sh_css_frame_info *out_info,
			 const struct sh_css_frame_info *vf_info,
			 unsigned int *downscale_log2);

void
sh_css_capture_enable_bayer_downscaling(bool enable);

void
sh_css_binary_print(const struct sh_css_binary *binary);

void
sh_css_print_sp_debug_state(const struct sh_css_sp_debug_state *state);

void
sh_css_frame_info_set_width(struct sh_css_frame_info *info,
			    unsigned int width);

/* Return whether the sp copy process should be started */
bool
sh_css_continuous_start_sp_copy(void);

/* The following functions are used for testing purposes only */
const struct sh_css_fpn_table *
sh_css_get_fpn_table(void);

const struct sh_css_shading_table *
sh_css_get_shading_table(void);

const struct sh_css_isp_params *
sh_css_get_isp_params(void);

void
sh_css_invalidate_morph_table(void);

const struct sh_css_binary *
sh_css_get_3a_binary(void);

void
sh_css_get_isp_dis_coefficients(short *horizontal_coefficients,
				short *vertical_coefficients);

void
sh_css_get_isp_dis_projections(int *horizontal_projections,
			       int *vertical_projections);

void *
sh_css_store_sp_group_to_ddr(void);

#endif /* _SH_CSS_INTERNAL_H_ */
