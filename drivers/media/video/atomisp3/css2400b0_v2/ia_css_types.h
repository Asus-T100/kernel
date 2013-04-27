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

#ifndef _IA_CSS_TYPES_H_
#define _IA_CSS_TYPES_H_

/*! \file */

/** @file ia_css_types.h
 * This file contains types used for the ia_css parameters.
 * These types are in a separate file because they are expected
 * to be used in software layers that do not access the CSS API
 * directly but still need to forward parameters for it.
 */

/* This code is also used by Silicon Hive in a simulation environment
 * Therefore, the following macro is used to differentiate when this
 * code is being included from within the Linux kernel source
 */

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#endif

#define IA_CSS_VERSION_MAJOR    2
#define IA_CSS_VERSION_MINOR    0
#define IA_CSS_VERSION_REVISION 2

/** Number of axes in the MACC table. */
#define IA_CSS_MACC_NUM_AXES           16
/** Number of coefficients per MACC axes. */
#define IA_CSS_MACC_NUM_COEFS          4
/** The number of planes in the morphing table. */
#define IA_CSS_MORPH_TABLE_NUM_PLANES  6
/** Number of color planes in the shading table. */
#define IA_CSS_SC_NUM_COLORS           4
/** Number of DVS coefficient types */
#define IA_CSS_DVS_NUM_COEF_TYPES      6
#define IA_CSS_DVS_COEF_TYPES_ON_DMEM  2
#define IA_CSS_DVS2_NUM_COEF_TYPES     4

/** Fractional bits for CTC gain */
#define IA_CSS_CTC_COEF_SHIFT          13
/** Fractional bits for GAMMA gain */
#define IA_CSS_GAMMA_GAIN_K_SHIFT      13

/** Number of elements in the CTC table. */
#define IA_CSS_VAMEM_2_CTC_TABLE_SIZE_LOG2      8
#define IA_CSS_VAMEM_2_CTC_TABLE_SIZE           ((1U<<IA_CSS_VAMEM_2_CTC_TABLE_SIZE_LOG2) + 1)
/** Number of elements in the gamma table. */
#define IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE_LOG2    8
#define IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE         ((1U<<IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE_LOG2) + 1)
/** Number of elements in the xnr table. */
#define IA_CSS_VAMEM_2_XNR_TABLE_SIZE_LOG2      6
#define IA_CSS_VAMEM_2_XNR_TABLE_SIZE	        (1U<<IA_CSS_VAMEM_2_XNR_TABLE_SIZE_LOG2)
/** Number of elements in the sRGB gamma table. */
#define IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE_LOG2    8
#define IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE     ((1U<<IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE_LOG2) + 1)

/** Number of elements in the CTC table. */
#define IA_CSS_VAMEM_1_CTC_TABLE_SIZE_LOG2      10
#define IA_CSS_VAMEM_1_CTC_TABLE_SIZE           (1U<<IA_CSS_VAMEM_1_CTC_TABLE_SIZE_LOG2)
/** Number of elements in the gamma table. */
#define IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE_LOG2    10
#define IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE         (1U<<IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE_LOG2)
/** Number of elements in the xnr table. */
#define IA_CSS_VAMEM_1_XNR_TABLE_SIZE_LOG2      6
#define IA_CSS_VAMEM_1_XNR_TABLE_SIZE           (1U<<IA_CSS_VAMEM_1_XNR_TABLE_SIZE_LOG2)
/** Number of elements in the sRGB gamma table. */
#define IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE_LOG2 8
#define IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE      (1U<<IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE_LOG2)

/* Fixed point types.
 * To comply with Linux coding standards these are #defines instead
 * of typedefs.
 * NOTE: the 16 bit fixed point types actually occupy 32 bits
 * to save on extension operations in the ISP code.
 */
/** Unsigned fixed point value, 0 integer bits, 16 fractional bits */
typedef uint32_t ia_css_u0_16;
/** Unsigned fixed point value, 5 integer bits, 11 fractional bits */
typedef uint32_t ia_css_u5_11;
/** Unsigned fixed point value, 8 integer bits, 8 fractional bits */
typedef uint32_t ia_css_u8_8;
/** Signed fixed point value, 0 integer bits, 15 fractional bits */
typedef int32_t ia_css_s0_15;

/* Virtual address within the CSS address space. */
typedef uint32_t ia_css_ptr;

enum ia_css_vamem_type {
	IA_CSS_VAMEM_TYPE_1,
	IA_CSS_VAMEM_TYPE_2,
};

/** Vector with signed values. This is used to indicate motion for
 * Digital Image Stabilization.
 */
struct ia_css_vector {
	int32_t x; /**< horizontal motion (in pixels) */
	int32_t y; /**< vertical motion (in pixels) */
};

/** 3A statistics grid */
struct ia_css_3a_grid_info {
	uint32_t enable;            /**< 3A statistics enabled */
	uint32_t use_dmem;          /**< DMEM or VMEM determines layout */
	uint32_t has_histogram;     /**< Statistics include histogram */
	uint32_t width;	            /**< Width of 3A grid */
	uint32_t height;	    /**< Height of 3A grid */
	uint32_t aligned_width;     /**< Horizontal stride (for alloc) */
	uint32_t aligned_height;    /**< Vertical stride (for alloc) */
	uint32_t bqs_per_grid_cell; /**< Grid cell size */
	uint32_t deci_factor_log2;  /**< log2 of bqs_per_grid_cell */
	uint32_t elem_bit_depth;    /**< Bit depth of element used to calculate 3A statistics */	
};

/** DVS statistics grid */
struct ia_css_dvs_grid_info {
	uint32_t enable;        /**< DVS statistics enabled */
	uint32_t width;	    /**< Width of DVS grid, this is equal to the
					 the number of vertical statistics. */
	uint32_t aligned_width; /**< Stride of each grid line */
	uint32_t height;	    /**< Height of DVS grid, this is equal
					 to the number of horizontal statistics.
				     */
	uint32_t aligned_height;/**< Stride of each grid column */
	uint32_t bqs_per_grid_cell; /**< Grid cell size */

	uint32_t num_hor_coefs;	/**< Number of horizontal coefficients. */
	uint32_t num_ver_coefs;	/**< Number of vertical coefficients. */
};

/** structure that describes the 3A and DIS grids */
struct ia_css_grid_info {
	/** \name ISP input size
	  * that is visible for user
	  * @{
	  */
	uint32_t isp_in_width;
	uint32_t isp_in_height;
	/** @}*/

	struct ia_css_3a_grid_info  s3a_grid; /**< 3A grid info */
	struct ia_css_dvs_grid_info dvs_grid; /**< DVS grid info */

	enum ia_css_vamem_type vamem_type;
};

/** Optical black mode.
 */
enum ia_css_ob_mode {
	IA_CSS_OB_MODE_NONE,
	IA_CSS_OB_MODE_FIXED,
	IA_CSS_OB_MODE_RASTER
};

/** The 4 colors that a shading table consists of.
 *  For each color we store a grid of values.
 */
enum ia_css_sc_color {
	IA_CSS_SC_COLOR_GR, /**< Green on a green-red line */
	IA_CSS_SC_COLOR_R,  /**< Red */
	IA_CSS_SC_COLOR_B,  /**< Blue */
	IA_CSS_SC_COLOR_GB  /**< Green on a green-blue line */
};

/** White Balance configuration (Gain Adjust).
 *  All values are uinteger_bits.16-integer_bits fixed point values.
 */
struct ia_css_wb_config {
	uint32_t integer_bits; /**< */
	uint32_t gr;	/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t r;		/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t b;		/* unsigned <integer_bits>.<16-integer_bits> */
	uint32_t gb;	/* unsigned <integer_bits>.<16-integer_bits> */
};

/** Color Space Conversion settings.
 *  The data is s13-fraction_bits.fraction_bits fixed point.
 */
struct ia_css_cc_config {
	uint32_t fraction_bits;
	int32_t matrix[3 * 3]; /**< RGB2YUV conversion matrix, signed
				   <13-fraction_bits>.<fraction_bits> */
};

/** Morping table, used for geometric distortion and chromatic abberration
 *  correction (GDCAC, also called GDC).
 *  This table describes the imperfections introduced by the lens, the
 *  advanced ISP can correct for these imperfections using this table.
 */
struct ia_css_morph_table {
	uint32_t enable; /**< To disable GDC, set this field to false. The
		          coordinates fields can be set to NULL in this case. */
	uint32_t height; /**< Table height */
	uint32_t width;  /**< Table width */
	uint16_t *coordinates_x[IA_CSS_MORPH_TABLE_NUM_PLANES];
	/**< X coordinates that describe the sensor imperfection */
	uint16_t *coordinates_y[IA_CSS_MORPH_TABLE_NUM_PLANES];
	/**< Y coordinates that describe the sensor imperfection */
};

/** Fixed pattern noise table. This contains the fixed patterns noise values
 *  obtained from a black frame capture.
 */
struct ia_css_fpn_table {
	int16_t *data;		/**< Table content */
	uint32_t width;	/**< Table height */
	uint32_t height;	/**< Table width */
	uint32_t shift;	/**< */
};

/** Lens color shading table. This describes the color shading artefacts
 *  introduced by lens imperfections.
 */
struct ia_css_shading_table {
	uint32_t enable; /**< set to false for no shading correction.
		          The data field can be NULL when enable == true */
	uint32_t sensor_width;  /**< Native sensor width in pixels */
	uint32_t sensor_height; /**< Native sensor height in lines */
	uint32_t width;  /**< Number of data points per line per color */
	uint32_t height; /**< Number of lines of data points per color */
	uint32_t fraction_bits; /**< Bits of fractional part in the data
					    points */
	uint16_t *data[IA_CSS_SC_NUM_COLORS];
	/**< Table data, one array for each color. Use ia_css_sc_color to
	     index this array */
};

/** Gamma table, used for gamma correction.
 */
struct ia_css_gamma_table {
	enum ia_css_vamem_type vamem_type;
	union {
		uint16_t vamem_1[IA_CSS_VAMEM_1_GAMMA_TABLE_SIZE];
		uint16_t vamem_2[IA_CSS_VAMEM_2_GAMMA_TABLE_SIZE];
	} data;
};

/** CTC table (need to explain CTC)
 */
struct ia_css_ctc_table {
	enum ia_css_vamem_type vamem_type;
	union {
		uint16_t vamem_1[IA_CSS_VAMEM_1_CTC_TABLE_SIZE];
		uint16_t vamem_2[IA_CSS_VAMEM_2_CTC_TABLE_SIZE];
	} data;
};

/** sRGB Gamma table, used for sRGB gamma correction.
 */
struct ia_css_rgb_gamma_table {
	enum ia_css_vamem_type vamem_type;
	union {
		uint16_t vamem_1[IA_CSS_VAMEM_1_RGB_GAMMA_TABLE_SIZE];
		uint16_t vamem_2[IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE];
	} data;
};

/** XNR table
 */
struct ia_css_xnr_table {
	enum ia_css_vamem_type vamem_type;
	union {
		uint16_t vamem_1[IA_CSS_VAMEM_1_XNR_TABLE_SIZE];
		uint16_t vamem_2[IA_CSS_VAMEM_2_XNR_TABLE_SIZE];
	} data;
};

/** Multi-Axes Color Correction (MACC) table. */
struct ia_css_macc_table {
	int16_t data[IA_CSS_MACC_NUM_COEFS * IA_CSS_MACC_NUM_AXES];
};

/** Advanced Noise Reduction (ANR) thresholds */
struct ia_css_anr_thres {
	int16_t data[13*64];
};

/** Temporal noise reduction (TNR) configuration.
 */
struct ia_css_tnr_config {
	ia_css_u0_16 gain;		/**< Gain (strength) of NR */
	ia_css_u0_16 threshold_y;	/**< Motion sensitivity for Y */
	ia_css_u0_16 threshold_uv;	/**< Motion sensitivity for U/V */
};

/** Optical black level configuration.
 */
struct ia_css_ob_config {
	enum ia_css_ob_mode mode; /**< Mode (Fixed / Raster) */
	ia_css_u0_16 level_gr;    /**< Black level for GR pixels */
	ia_css_u0_16 level_r;     /**< Black level for R pixels */
	ia_css_u0_16 level_b;     /**< Black level for B pixels */
	ia_css_u0_16 level_gb;    /**< Black level for GB pixels */
	uint16_t start_position; /**< Start position of OB area (used for
				      raster mode only). Valid range is [0..63]. */
	uint16_t end_position;  /**< End position of OB area (used for
				      raster mode only).
				      Valid range is [start_pos..64]. */
};

/** Defect pixel correction configuration.
 */
struct ia_css_dp_config {
	ia_css_u0_16 threshold; /**< The threshold of defect Pixel Correction,
			      representing the permissible difference of
			      intensity between one pixel and its
			      surrounding pixels. Smaller values result
			      in more frequent pixel corrections. */
	ia_css_u8_8 gain;	 /**< The sensitivity of mis-correction. ISP will
			      miss a lot of defects if the value is set
			      too large. */
};

/** Configuration used by Bayer Noise Reduction (BNR) and
 *  YCC noise reduction (YNR).
 */
struct ia_css_nr_config {
	ia_css_u0_16 bnr_gain;	    /**< Strength of noise reduction (BNR) */
	ia_css_u0_16 ynr_gain;	    /**< Strength of noise reduction (YNR */
	ia_css_u0_16 direction;    /**< Sensitivity of Edge (BNR) */
	ia_css_u0_16 threshold_cb; /**< Coring threshold for Cb (YNR) */
	ia_css_u0_16 threshold_cr; /**< Coring threshold for Cr (YNR) */
};

/** Edge Enhancement (sharpen) configuration.
 */
struct ia_css_ee_config {
	ia_css_u5_11 gain;	   /**< The strength of sharpness. */
	ia_css_u8_8 threshold;    /**< The threshold that divides noises from
				       edge. */
	ia_css_u5_11 detail_gain; /**< The strength of sharpness in pell-mell
				       area. */
};

/** Demosaic (bayer-to-rgb) configuration.
 */
struct ia_css_de_config {
	ia_css_u0_16 pixelnoise;	   /**< Pixel noise used in moire elimination */
	ia_css_u0_16 c1_coring_threshold; /**< Coring threshold for C1 */
	ia_css_u0_16 c2_coring_threshold; /**< Coring threshold for C2 */
};

/** Gamma Correction configuration.
  */
struct ia_css_gc_config {
	uint16_t gain_k1; /**< */
	uint16_t gain_k2; /**< */
};

struct ia_css_dvs_6axis_config {
	uint32_t width_y;
	uint32_t height_y;
	uint32_t width_uv;
	uint32_t height_uv;
	uint32_t *xcoords_y;
	uint32_t *ycoords_y;
	uint32_t *xcoords_uv;
	uint32_t *ycoords_uv;   
};

/** Advanced Noise Reduction configuration.
 *  This is also known as Low-Light.
 */
struct ia_css_anr_config {
	int32_t threshold; /**< Threshold */
	int32_t thresholds[4*4*4];
	int32_t factors[3];
};

/** Eigen Color Demosaicing configuration.
 */
struct ia_css_ecd_config {
	uint16_t zip_strength;
	uint16_t fc_strength;
	uint16_t fc_debias;
};

/** Y(Luma) Noise Reduction configuration.
 */
struct ia_css_ynr_config {
	uint16_t edge_sense_gain_0;
	uint16_t edge_sense_gain_1;
	uint16_t corner_sense_gain_0;
	uint16_t corner_sense_gain_1;
};

/** Fringe Control configuration.
 */
struct ia_css_fc_config {
	uint8_t  gain_exp;
	uint16_t gain_pos_0;
	uint16_t gain_pos_1;
	uint16_t gain_neg_0;
	uint16_t gain_neg_1;
	uint16_t crop_pos_0;
	uint16_t crop_pos_1;
	uint16_t crop_neg_0;
	uint16_t crop_neg_1;
};

/** Chroma Noise Reduction configuration.
 */
struct ia_css_cnr_config {
	uint8_t coring_u;
	uint8_t coring_v;
	uint8_t sense_gain_vy;
	uint8_t sense_gain_vu;
	uint8_t sense_gain_vv;
	uint8_t sense_gain_hy;
	uint8_t sense_gain_hu;
	uint8_t sense_gain_hv;
};

/** MACC
 */
struct ia_css_macc_config {
	uint8_t exp; /**< */
};

/** Chroma Tone Control configuration.
 */
struct ia_css_ctc_config {
	uint16_t y0;
	uint16_t y1;
	uint16_t y2;
	uint16_t y3;
	uint16_t y4;
	uint16_t y5;
	uint16_t ce_gain_exp;
	uint16_t x1;
	uint16_t x2;
	uint16_t x3;
	uint16_t x4;
};

/** Anti-Aliasing configuration.
 */
struct ia_css_aa_config {
	uint16_t scale;
};

/** Chroma Enhancement configuration.
 */
struct ia_css_ce_config {
	ia_css_u0_16 uv_level_min; /**< */
	ia_css_u0_16 uv_level_max; /**< */
};

/** Color Correction Matrix (YCgCo to RGB) settings.
 *  The data is
 *  s(13-IA_CSS_YUV2RGB_CCM_COEF_SHIFT).IA_CSS_YUV2RGB_CCM_COEF_SHIFT
 *  fixed point.
 */
struct ia_css_yuv2rgb_cc_config {
	int32_t matrix[3 * 3]; /**< YUV2RGB conversion matrix, signed
	<13-IA_CSS_YUV2RGB_CCM_COEF_SHIFT>.<IA_CSS_YUV2RGB_CCM_COEF_SHIFT> */
};

/** Color Space Conversion (RGB to YUV) settings.
 *  The data is
 *  s(13-IA_CSS_RGB2YUV_CSC_COEF_SHIFT).IA_CSS_RGB2YUV_CSC_COEF_SHIFT
 *  fixed point.
 */
struct ia_css_rgb2yuv_cc_config {
	int32_t matrix[3 * 3]; /**< RGB2YUV conversion matrix, signed
	<13-IA_CSS_RGB2YUV_CSC_COEF_SHIFT>.<IA_CSS_RGB2YUV_CSC_COEF_SHIFT> */
};

/** 3A configuration. This configures the 3A statistics collection
 *  module.
 */
struct ia_css_3a_config {
	ia_css_u0_16 ae_y_coef_r;	/**< Weight of R for Y */
	ia_css_u0_16 ae_y_coef_g;	/**< Weight of G for Y */
	ia_css_u0_16 ae_y_coef_b;	/**< Weight of B for Y */
	ia_css_u0_16 awb_lg_high_raw;	/**< AWB level gate high for raw */
	ia_css_u0_16 awb_lg_low;	/**< AWB level gate low */
	ia_css_u0_16 awb_lg_high;	/**< AWB level gate high */
	ia_css_s0_15 af_fir1_coef[7];	/**< AF FIR coefficients of fir1 */
	ia_css_s0_15 af_fir2_coef[7];	/**< AF FIR coefficients of fir2 */
};

/** eXtra Noise Reduction configuration.
 */
struct ia_css_xnr_config {
	uint32_t threshold;  /**< Threshold */
};

/**
 * Digital zoom:
 * This feature is currently available only for video, but will become
 * available for preview and capture as well.
 * Set the digital zoom factor, this is a logarithmic scale. The actual zoom
 * factor will be 64/x.
 * Setting dx or dy to 0 disables digital zoom for that direction.
 */
struct ia_css_dz_config {
	uint32_t dx;
	uint32_t dy;
};

/** The still capture mode, this can be RAW (simply copy sensor input to DDR),
 *  Primary ISP, the Advanced ISP (GDC) or the low-light ISP (ANR).
 */
enum ia_css_capture_mode {
	IA_CSS_CAPTURE_MODE_RAW,      /**< no processing, copy data only */
	IA_CSS_CAPTURE_MODE_BAYER,    /**< pre ISP (bayer capture) */
	IA_CSS_CAPTURE_MODE_PRIMARY,  /**< primary ISP */
	IA_CSS_CAPTURE_MODE_ADVANCED, /**< advanced ISP (GDC) */
	IA_CSS_CAPTURE_MODE_LOW_LIGHT /**< low light ISP (ANR) */
};

struct ia_css_capture_config {
	enum ia_css_capture_mode mode; /**< Still capture mode */
	uint32_t enable_xnr;	       /**< Enable/disable XNR */
	uint32_t enable_capture_pp;
	/**< Enable/disable the post-processing binary.
	     This is for testing purposes only! */
	uint32_t enable_raw_output;
};

/** ISP filter configuration. This is a collection of configurations
 *  for each of the ISP filters (modules).
 *
 *  NOTE! The contents of all pointers is copied when get or set with the
 *  exception of the shading and morph tables. For these we only copy the
 *  pointer, so the caller must make sure the memory contents of these pointers
 *  remain valid as long as they are used by the CSS. This will be fixed in the
 *  future by copying the contents instead of just the pointer.
 */
struct ia_css_isp_config {
	struct ia_css_wb_config   *wb_config;  /**< White Balance config */
	struct ia_css_cc_config   *cc_config;  /**< Color Correction config */
	struct ia_css_tnr_config  *tnr_config; /**< Temporal Noise Reduction */
	struct ia_css_ecd_config  *ecd_config; /**< Eigen Color Demosaicing */
	struct ia_css_ynr_config  *ynr_config; /**< Y(Luma) Noise Reduction */
	struct ia_css_fc_config   *fc_config;  /**< Fringe Control */
	struct ia_css_cnr_config  *cnr_config; /**< Chroma Noise Reduction */
	struct ia_css_macc_config *macc_config;  /**< MACC */
	struct ia_css_ctc_config  *ctc_config; /**< Chroma Tone Control */
	struct ia_css_aa_config   *aa_config;  /**< Anti-Aliasing */
	struct ia_css_ce_config   *ce_config;
	struct ia_css_dvs_6axis_config *dvs_6axis_config;
	struct ia_css_ob_config   *ob_config;  /**< Objective Black config */
	struct ia_css_dp_config   *dp_config;  /**< Dead Pixel config */
	struct ia_css_nr_config   *nr_config;  /**< Noise Reduction config */
	struct ia_css_ee_config   *ee_config;  /**< Edge Enhancement config */
	struct ia_css_de_config   *de_config;  /**< Demosaic config */
	struct ia_css_gc_config   *gc_config;  /**< Gamma Correction config */
	struct ia_css_anr_config  *anr_config; /**< Advanced Noise Reduction */
	struct ia_css_3a_config   *s3a_config; /**< 3A Statistics config */
	struct ia_css_xnr_config  *xnr_config; /**< eXtra Noise Reduction */
	struct ia_css_dz_config   *dz_config;  /**< Digital Zoom */
	struct ia_css_cc_config *yuv2rgb_cc_config; /**< Color
							Correction config */
	struct ia_css_cc_config *rgb2yuv_cc_config; /**< Color
							Correction config */
	struct ia_css_macc_table  *macc_table;
	struct ia_css_gamma_table *gamma_table;
	struct ia_css_ctc_table   *ctc_table;
	struct ia_css_xnr_table   *xnr_table;
	struct ia_css_rgb_gamma_table *r_gamma_table;
	struct ia_css_rgb_gamma_table *g_gamma_table;
	struct ia_css_rgb_gamma_table *b_gamma_table;
	struct ia_css_vector      *motion_vector; /**< For 2-axis DVS */
	struct ia_css_shading_table *shading_table;
	struct ia_css_morph_table   *morph_table;
	struct ia_css_dvs_coefficients *dvs_coefs; /**< DVS 1.0 coefficients */
	struct ia_css_dvs2_coefficients *dvs2_coefs; /**< DVS 2.0 coefficients */
	struct ia_css_capture_config   *capture_config;
	struct ia_css_anr_thres   *anr_thres;
};

/** 3A statistics point. This structure describes the data stored
 *  in each 3A grid point.
 */
struct ia_css_3a_output {
	int32_t ae_y;    /**< */
	int32_t awb_cnt; /**< */
	int32_t awb_gr;  /**< */
	int32_t awb_r;   /**< */
	int32_t awb_b;   /**< */
	int32_t awb_gb;  /**< */
	int32_t af_hpf1; /**< */
	int32_t af_hpf2; /**< */
};

struct ia_css_3a_rgby_output {
	uint32_t r;    /**< */
	uint32_t g; /**< */
	uint32_t b;  /**< */
	uint32_t y;   /**< */
};

/** DVS 1.0 Coefficients.
 */

struct ia_css_dvs_coefficients {
	struct ia_css_dvs_grid_info grid;
	int16_t *hor_coefs;
	int16_t *ver_coefs;
};

/** DVS 1.0 Statistics
 */

struct ia_css_dvs_statistics {
	struct ia_css_dvs_grid_info grid;
	int32_t *hor_proj;
	int32_t *ver_proj;
};

/** DVS 2.0 Coefficient types. This structure contains 4 pointers to
 *  arrays that contain the coeffients for each type.
 */
struct ia_css_dvs2_coef_types {
	int16_t *odd_real; /**< real part of the odd coefficients*/
	int16_t *odd_imag; /**< imaginary part of the odd coefficients*/
	int16_t *even_real;/**< real part of the even coefficients*/
	int16_t *even_imag;/**< imaginary part of the even coefficients*/
};

/** DVS 2.0 Coefficients. This structure describes the coefficients that are needed for the dvs statistics.
 *  e.g. hor_coefs.odd_real is the pointer to int16_t[grid.num_hor_coefs] containing the horizontal odd real 
 *  coefficients.
 */
struct ia_css_dvs2_coefficients {
	struct ia_css_dvs_grid_info grid;        /**< grid info contains the dimensions of the dvs grid */
	struct ia_css_dvs2_coef_types hor_coefs; /**< struct with pointers that contain the horizontal coefficients */
	struct ia_css_dvs2_coef_types ver_coefs; /**< struct with pointers that contain the vertical coefficients */
};

/** DVS 2.0 Statistic types. This structure contains 4 pointers to
 *  arrays that contain the statistics for each type.
 */
struct ia_css_dvs2_stat_types {
	int32_t *odd_real; /**< real part of the odd statistics*/
	int32_t *odd_imag; /**< imaginary part of the odd statistics*/
	int32_t *even_real;/**< real part of the even statistics*/
	int32_t *even_imag;/**< imaginary part of the even statistics*/
};

/** DVS 2.0 Statistics. This structure describes the statistics that are generated using the provided coefficients.
 *  e.g. hor_prod.odd_real is the pointer to int16_t[grid.aligned_height][grid.aligned_width] containing 
 *  the horizontal odd real statistics. Valid statistics data area is int16_t[0..grid.height-1][0..grid.width-1]
 */
struct ia_css_dvs2_statistics {
	struct ia_css_dvs_grid_info grid;       /**< grid info contains the dimensions of the dvs grid */
	struct ia_css_dvs2_stat_types hor_prod; /**< struct with pointers that contain the horizontal statistics */
	struct ia_css_dvs2_stat_types ver_prod; /**< struct with pointers that contain the vertical statistics */
};

struct ia_css_3a_statistics {
	struct ia_css_3a_grid_info    grid;
	struct ia_css_3a_output      *data;
	struct ia_css_3a_rgby_output *rgby_data;
};

#endif /* _IA_CSS_TYPES_H_ */
