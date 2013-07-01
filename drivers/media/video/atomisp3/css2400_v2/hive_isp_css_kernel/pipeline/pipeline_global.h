#ifndef __PIPELINE_GLOBAL_H_INCLUDED__
#define __PIPELINE_GLOBAL_H_INCLUDED__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "system_types.h"	/* To get the DLI version, we should not need that */

#if defined(IS_ISP_2400_SYSTEM)
#define SH_CSS_ISP_PARAMS_VERSION	2
#define SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB	1
#else
#define SH_CSS_ISP_PARAMS_VERSION	1
#define SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB	0
#endif

#define SH_CSS_MAX_STAGES	6

typedef struct sh_css_isp_params		pipeline_param_t;
typedef const struct sh_css_isp_params	*pipeline_param_h;
//typedef const pipeline_param_t		*pipeline_param_h;

typedef struct sh_css_uds_info			sh_css_uds_info_t;
typedef struct sh_css_crop_pos			sh_css_crop_pos_t;

/*
 * Compose the pipeline (parameters) from one or more module parameter sets
 *
 * This file should be sperated for pipeline versions, but separation is not
 * yet possible at CSS API level. So
 *
 * For ISP 1.0:
 *	CSC_KERNEL_PARAM_SET0: CSC conversion matrix
 *	CSC_KERNEL_PARAM_SET1: Not used
 *
 * For ISP 1.0:
 *	CSC_KERNEL_PARAM_SET0: RGB 2 YUV
 *	CSC_KERNEL_PARAM_SET1: YUV 2 RGB
 *
 * becomes
 *	CSC_KERNEL_PARAM_SET0: CSC conversion matrix
 *	CSC_KERNEL_PARAM_SET1: RGB 2 YUV
 *	CSC_KERNEL_PARAM_SET2: YUV 2 RGB
 *
 * Using enums to create constants is not supported in the pipeline generator flow
 *
typedef enum {
	CSC_KERNEL_PARAM_SET0 = 0,
	CSC_KERNEL_PARAM_SET1,
	CSC_KERNEL_PARAM_SET2,
	N_CSC_KERNEL_PARAM_SET
} csc_kernel_param_set_t;
 */

#define CSC_KERNEL_PARAM_SET0	0
#define CSC_KERNEL_PARAM_SET1	1
#define CSC_KERNEL_PARAM_SET2	2
#define N_CSC_KERNEL_PARAM_SET	3
typedef uint16_t	csc_kernel_param_set_t;

#define pipeline_get_csc_param_set(pipeline_param, csc_kernel_param_set) csc_kernel_param_set
#define g_ispparm_csc_param_set(csc_kernel_param) g_ispparm.csc_kernel_param[csc_kernel_param]

#include "csc_kernel.h"

struct sh_css_uds_info {
	uint16_t curr_dx;
	uint16_t curr_dy;
	uint16_t xc;
	uint16_t yc;
};

struct sh_css_crop_pos {
	uint16_t x;
	uint16_t y;
};

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
#if SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB
	int dpc_coef_rr_gr;
	int dpc_coef_rr_gb;
	int dpc_coef_bb_gb;
	int dpc_coef_bb_gr;
	int dpc_coef_gr_rr;
	int dpc_coef_gr_bb;
	int dpc_coef_gb_bb;
	int dpc_coef_gb_rr;
#endif

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

	/* S3A (3A Support): AWB level gate */
	int awb_lg_high_raw;
	int awb_lg_low;
	int awb_lg_high;

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
	csc_kernel_param_t	csc_kernel_param[N_CSC_KERNEL_PARAM_SET];

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

	sh_css_crop_pos_t sp_out_crop_pos[SH_CSS_MAX_STAGES];
	sh_css_uds_info_t uds[SH_CSS_MAX_STAGES];

/* parameters for ISP pipe version 2 */
#if SH_CSS_ISP_PARAMS_VERSION == 2
	/* DE (Demosaic) */
	int ecd_zip_strength;
	int ecd_fc_strength;
	int ecd_fc_debias;

	/* YNR (Y Noise Reduction), YEE (Y Edge Enhancement) */
	int yee_edge_sense_gain_0;
	int yee_edge_sense_gain_1;
	int yee_corner_sense_gain_0;
	int yee_corner_sense_gain_1;

	/* Fringe Control */
	int fc_gain_exp;
	int fc_gain_pos_0;
	int fc_gain_pos_1;
	int fc_gain_neg_0;
	int fc_gain_neg_1;
	int fc_crop_pos_0;
	int fc_crop_pos_1;
	int fc_crop_neg_0;
	int fc_crop_neg_1;

	/* CNR */
	int cnr_coring_u;
	int cnr_coring_v;
	int cnr_sense_gain_vy;
	int cnr_sense_gain_vu;
	int cnr_sense_gain_vv;
	int cnr_sense_gain_hy;
	int cnr_sense_gain_hu;
	int cnr_sense_gain_hv;
#endif /* SH_CSS_ISP_PARAMS_VERSION == 2 */

	/* MACC */
	int exp;

/* parameters for ISP pipe version 2 */
#if SH_CSS_ISP_PARAMS_VERSION == 2
	/* CTC */
	int ctc_y0;
	int ctc_y1;
	int ctc_y2;
	int ctc_y3;
	int ctc_y4;
	int ctc_y5;
	int ctc_ce_gain_exp;
	int ctc_x1;
	int ctc_x2;
	int ctc_x3;
	int ctc_x4;
	int ctc_dydx0;
	int ctc_dydx0_shift;
	int ctc_dydx1;
	int ctc_dydx1_shift;
	int ctc_dydx2;
	int ctc_dydx2_shift;
	int ctc_dydx3;
	int ctc_dydx3_shift;
	int ctc_dydx4;
	int ctc_dydx4_shift;

#endif /* SH_CSS_ISP_PARAMS_VERSION == 2 */
	/* Anti-Aliasing */
	int aa_scale;
};

#endif /* __CSC_KERNEL_GLOBAL_H_INCLUDED__ */
