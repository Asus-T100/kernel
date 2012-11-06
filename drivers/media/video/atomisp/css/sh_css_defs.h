#ifndef _SH_CSS_DEFS_H_
#define _SH_CSS_DEFS_H_

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

#if defined(SYSTEM_hive_isp_css_2400_system) || defined(__isp2400_mamoiada) || \
	defined(__scalar_processor_2400)
#include <isp2400_mamoiada_params.h>         /* ISP_VEC_NELEMS */
#include "gdc_v2_defs.h"                     /* HRT_GDC_N */
#include <hive_isp_css_2400_defs.h>              /* HIVE_ISP_DDR_WORD_BYTES */
#elif defined(SYSTEM_hive_isp_css_system) || defined(__isp2300_medfield) || \
	defined(__scalar_processor)
#include <isp2300_medfield_params.h>
#include "gdc_defs.h"
#include <hive_isp_css_defs.h>              /* HIVE_ISP_DDR_WORD_BYTES */
#elif defined(SYSTEM_hive_isp_css_large_system) || \
	defined(__isp2300_medfield_large) || defined(__scalar_processor_large)
#include <isp2300_medfield_large_params.h>
#include "gdc_defs.h"
#include <hive_isp_css_defs.h>              /* HIVE_ISP_DDR_WORD_BYTES */
#elif defined(SYSTEM_isp_css_dev_tb) || defined(__isp2300_medfield_demo) || \
	defined(__scalar_processor_demo)
#include <isp2300_medfield_demo_params.h>
#include "gdc_defs.h"
#include <hive_isp_css_defs.h>              /* HIVE_ISP_DDR_WORD_BYTES */
#else
/* pipeline generator does not accept -D<var> */
/*#error "sh_css_defs.h: Unknown system" */
#endif
/* Width of a DDR word in bytes */
#define HIVE_ISP_DDR_WORD_BYTES           (HIVE_ISP_DDR_WORD_BITS/8)


/* Circular dependencies copy the definition in isp_defs.h */
/*#include <isp_defs.h>*/
/* System dependent and used inconsistently */
#define UDS_SCALING_N                 HRT_GDC_N

/* the max macro from the kernel only works within function context. We use
   these macros also as global initializers (for now). for this, we need
   the MAX macro. */
#define MAX(a, b)        ((a) > (b) ? (a) : (b))
#define CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))

/* Digital Image Stabilization */
#define SH_CSS_DIS_DECI_FACTOR_LOG2       6

/* UV offset: 1:uv=-128...127, 0:uv=0...255 */
#define SH_CSS_UV_OFFSET_IS_0             0

/* Bits of bayer is adjusted as 13 in ISP */
#define SH_CSS_BAYER_BITS                 13
/* Max value of bayer data (unsigned 13bit in ISP) */
#define SH_CSS_BAYER_MAXVAL               ((1U << SH_CSS_BAYER_BITS) - 1)

/* Bits of yuv in ISP */
#define SH_CSS_ISP_YUV_BITS               8

#define SH_CSS_DP_GAIN_SHIFT              5
#define SH_CSS_BNR_GAIN_SHIFT             13
#define SH_CSS_YNR_GAIN_SHIFT             13
#define SH_CSS_AE_YCOEF_SHIFT             13
#define SH_CSS_AF_FIR_SHIFT               13
#define SH_CSS_YEE_DETAIL_GAIN_SHIFT      8  /* [u5.8] */
#define SH_CSS_YEE_SCALE_SHIFT            8
#define SH_CSS_TNR_COEF_SHIFT                    13
#define SH_CSS_MACC_COEF_SHIFT            11 /* [s2.11] */

#define SH_CSS_NUM_INPUT_BUF_LINES        4

/* Left cropping only applicable for sufficiently large nway */
#if ISP_VEC_NELEMS == 16
#define SH_CSS_MAX_LEFT_CROPPING          0
#else
#ifdef SYSTEM_hive_isp_css_2400_system
#define SH_CSS_MAX_LEFT_CROPPING          8
#else
#define SH_CSS_MAX_LEFT_CROPPING          12
#endif
#endif

#define	SH_CSS_SP_MAX_WIDTH               1280

/* This is the maximum grid we can handle in the ISP binaries.
 * The host code makes sure no bigger grid is ever selected. */
#define SH_CSS_MAX_BQ_GRID_WIDTH          80
#define SH_CSS_MAX_BQ_GRID_HEIGHT         60

/* The minimum dvs envelope is 8x8 to make sure the invalid rows/columns
   that result from filter initialization are skipped. */
#ifdef SYSTEM_hive_isp_css_2400_system
#define SH_CSS_MIN_DVS_ENVELOPE           8
#else
#define SH_CSS_MIN_DVS_ENVELOPE           12
#endif

/* The FPGA system (vec_nelems == 16) only supports upto 5MP */
#if ISP_VEC_NELEMS == 16
#define SH_CSS_MAX_SENSOR_WIDTH           2560
#define SH_CSS_MAX_SENSOR_HEIGHT          1920
#else
#define SH_CSS_MAX_SENSOR_WIDTH           4608
#define SH_CSS_MAX_SENSOR_HEIGHT          3450
#endif

#define SH_CSS_MIN_SENSOR_WIDTH           2
#define SH_CSS_MIN_SENSOR_HEIGHT          2

#define SH_CSS_MAX_VF_WIDTH               1280
#define SH_CSS_MAX_VF_HEIGHT              960

#define SH_CSS_DEFAULT_C_SUBSAMPLING      2

/* We use 16 bits per coordinate component, including integer
   and fractional bits */
#define SH_CSS_MORPH_TABLE_GRID               ISP_VEC_NELEMS
#define SH_CSS_MORPH_TABLE_ELEM_BYTES         2
#define SH_CSS_MORPH_TABLE_ELEMS_PER_DDR_WORD \
	(HIVE_ISP_DDR_WORD_BYTES/SH_CSS_MORPH_TABLE_ELEM_BYTES)

#define SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR   (SH_CSS_MAX_BQ_GRID_WIDTH + 1)
#define SH_CSS_MAX_SCTBL_HEIGHT_PER_COLOR   (SH_CSS_MAX_BQ_GRID_HEIGHT + 1)
#define SH_CSS_MAX_SCTBL_ALIGNED_WIDTH_PER_COLOR \
	CEIL_MUL(SH_CSS_MAX_SCTBL_WIDTH_PER_COLOR, ISP_VEC_NELEMS)

/* Each line of this table is aligned to the maximum line width. */
#define SH_CSS_MAX_S3ATBL_WIDTH              SH_CSS_MAX_BQ_GRID_WIDTH

/* Rules: these implement logic shared between the host code and ISP firmware.
   The ISP firmware needs these rules to be applied at pre-processor time,
   that's why these are macros, not functions. */
#define _ISP_BQS(num)  ((num)/2)
#define _ISP_VECS(width) CEIL_DIV(width, ISP_VEC_NELEMS)

#define ISP_BQ_GRID_WIDTH(elements_per_line, deci_factor_log2) \
	CEIL_SHIFT(elements_per_line/2,  deci_factor_log2)
#define ISP_BQ_GRID_HEIGHT(lines_per_frame, deci_factor_log2) \
	CEIL_SHIFT(lines_per_frame/2,  deci_factor_log2)
#define ISP_C_VECTORS_PER_LINE(elements_per_line) \
	_ISP_VECS(elements_per_line/2)

/* The morphing table is similar to the shading table in the sense that we
   have 1 more value than we have cells in the grid. */
#define _ISP_MORPH_TABLE_WIDTH(int_width) \
	(CEIL_DIV(int_width, SH_CSS_MORPH_TABLE_GRID) + 1)
#define _ISP_MORPH_TABLE_HEIGHT(int_height) \
	(CEIL_DIV(int_height, SH_CSS_MORPH_TABLE_GRID) + 1)
#define _ISP_MORPH_TABLE_ALIGNED_WIDTH(width) \
	CEIL_MUL(_ISP_MORPH_TABLE_WIDTH(width), \
		 SH_CSS_MORPH_TABLE_ELEMS_PER_DDR_WORD)

#define _ISP_SCTBL_WIDTH_PER_COLOR(input_width, deci_factor_log2) \
	(ISP_BQ_GRID_WIDTH(input_width, deci_factor_log2) + 1)
#define _ISP_SCTBL_HEIGHT(input_height, deci_factor_log2) \
	(ISP_BQ_GRID_HEIGHT(input_height, deci_factor_log2) + 1)
#define _ISP_SCTBL_ALIGNED_WIDTH_PER_COLOR(input_width, deci_factor_log2) \
	CEIL_MUL(_ISP_SCTBL_WIDTH_PER_COLOR(input_width, deci_factor_log2), \
		 ISP_VEC_NELEMS)

/* As an optimization we use DMEM to store the 3A statistics for fixed
 * resolution primary binaries on the ASIC system (not on FPGA). */
#define _S3ATBL_USE_DMEM(var_res)       (!(var_res))

/* ********************************************************
 * Statistics for Digital Image Stabilization
 * ********************************************************/
/* Some binaries put the vertical coefficients in DMEM instead
   of VMEM to save VMEM. */
#define _SDIS_VER_COEF_TBL_USE_DMEM(mode, enable_sdis) \
	(mode == SH_CSS_BINARY_MODE_VIDEO && enable_sdis)

/* For YUV upscaling, the internal size is used for DIS statistics */
#define _ISP_SDIS_ELEMS_ISP(input, internal, enable_us) \
	((enable_us) ? (internal) : (input))

/* SDIS Projections:
 * Horizontal projections are calculated for each line.
 * Vertical projections are calculated for each column.
 * Grid cells that do not fall completely within the image are not
 * valid. The host needs to use the bigger one for the stride but
 * should only return the valid ones to the 3A. */
#define __ISP_SDIS_HOR_PROJ_NUM_ISP(in_height, deci_factor_log2) \
	CEIL_SHIFT(_ISP_BQS(in_height), deci_factor_log2)
#define __ISP_SDIS_VER_PROJ_NUM_ISP(in_width, deci_factor_log2) \
	CEIL_SHIFT(_ISP_BQS(in_width), deci_factor_log2)

#define _ISP_SDIS_HOR_PROJ_NUM_3A(in_height, deci_factor_log2) \
	(_ISP_BQS(in_height) >> deci_factor_log2)
#define _ISP_SDIS_VER_PROJ_NUM_3A(in_width, deci_factor_log2) \
	(_ISP_BQS(in_width) >> deci_factor_log2)

/* SDIS Coefficients: */
/* The ISP uses vectors to store the coefficients, so we round
   the number of coefficients up to vectors. */
#define __ISP_SDIS_HOR_COEF_NUM_VECS(in_width)  _ISP_VECS(_ISP_BQS(in_width))
#define __ISP_SDIS_VER_COEF_NUM_VECS(in_height) _ISP_VECS(_ISP_BQS(in_height))

/* The number of coefficients produced by the ISP */
#define _ISP_SDIS_HOR_COEF_NUM_ISP(in_width) \
	(__ISP_SDIS_HOR_COEF_NUM_VECS(in_width) * ISP_VEC_NELEMS)
#define _ISP_SDIS_VER_COEF_NUM_ISP(in_height) \
	(__ISP_SDIS_VER_COEF_NUM_VECS(in_height) * ISP_VEC_NELEMS)

/* The number of coefficients used by the 3A library. This excludes
   coefficients from grid cells that do not fall completely within the image. */
#define _ISP_SDIS_HOR_COEF_NUM_3A(in_width, deci_factor_log2) \
	((_ISP_BQS(in_width) >> deci_factor_log2) << deci_factor_log2)
#define _ISP_SDIS_VER_COEF_NUM_3A(in_height, deci_factor_log2) \
	((_ISP_BQS(in_height) >> deci_factor_log2) << deci_factor_log2)

/* *****************************************************************
 * Statistics for 3A (Auto Focus, Auto White Balance, Auto Exposure)
 * *****************************************************************/
/* if left cropping is used, 3A statistics are also cropped by 2 vectors. */
#define _ISP_S3ATBL_WIDTH(in_width, deci_factor_log2) \
	(_ISP_BQS(in_width) >> deci_factor_log2)
#define _ISP_S3ATBL_HEIGHT(in_height, deci_factor_log2) \
	(_ISP_BQS(in_height) >> deci_factor_log2)

#define _ISP_S3A_ELEMS_ISP_WIDTH(in_width, int_width, enable_hus, left_crop) \
	(((enable_hus) ? (int_width) : (in_width)) \
	 - ((left_crop) ? 2 * ISP_VEC_NELEMS : 0))
#define _ISP_S3A_ELEMS_ISP_HEIGHT(in_height, int_height, enable_vus) \
	((enable_vus) ? (int_height) : (in_height))

#define _ISP_S3ATBL_ISP_WIDTH(in_width, deci_factor_log2) \
	CEIL_SHIFT(_ISP_BQS(in_width), deci_factor_log2)
#define _ISP_S3ATBL_ISP_HEIGHT(in_height, deci_factor_log2) \
	CEIL_SHIFT(_ISP_BQS(in_height), deci_factor_log2)
#define ISP_S3ATBL_VECTORS \
	_ISP_VECS(SH_CSS_MAX_S3ATBL_WIDTH * \
		  (sizeof(struct sh_css_3a_output)/sizeof(int)))
#define ISP_S3ATBL_HI_LO_STRIDE \
	(ISP_S3ATBL_VECTORS * ISP_VEC_NELEMS)
#define ISP_S3ATBL_HI_LO_STRIDE_BYTES \
	(sizeof(unsigned short) * ISP_S3ATBL_HI_LO_STRIDE)

/* Viewfinder support */
#define __ISP_MAX_VF_OUTPUT_WIDTH(width, left_crop) \
	(width - 2*ISP_VEC_NELEMS + ((left_crop) ? 2 * ISP_VEC_NELEMS : 0))

/* Number of vectors per vf line is determined by the chroma width,
 * the luma width is derived from that. That's why we have the +1. */
#define __ISP_VF_OUTPUT_WIDTH_VECS(out_width, vf_log_downscale) \
	(_ISP_VECS((out_width) >> ((vf_log_downscale)+1)) * 2)

#define _ISP_VF_OUTPUT_WIDTH(vf_out_vecs) ((vf_out_vecs) * ISP_VEC_NELEMS)
#define _ISP_VF_OUTPUT_HEIGHT(out_height, vf_log_ds) \
	((out_height) >> (vf_log_ds))

#define _ISP_LOG_VECTOR_STEP(mode) \
	((mode) == SH_CSS_BINARY_MODE_CAPTURE_PP ? 2 : 1)

/* Rules for computing the internal width. This is extremely complicated
 * and definitely needs to be commented and explained. */
#define _ISP_LEFT_CROP_EXTRA(left_crop) ((left_crop) > 0 ? 2*ISP_VEC_NELEMS : 0)

#define __ISP_MIN_INTERNAL_WIDTH(num_chunks, pipelining, mode) \
	((num_chunks) * (pipelining) * (1<<_ISP_LOG_VECTOR_STEP(mode)) * \
	 ISP_VEC_NELEMS)
#define __ISP_PADDED_OUTPUT_WIDTH(out_width, dvs_env_width, left_crop) \
	((out_width) + MAX(dvs_env_width, _ISP_LEFT_CROP_EXTRA(left_crop)))
#define __ISP_CHUNK_STRIDE_ISP(mode) \
	((1<<_ISP_LOG_VECTOR_STEP(mode)) * ISP_VEC_NELEMS)
#define __ISP_CHUNK_STRIDE_DDR(c_subsampling, num_chunks) \
	((c_subsampling) * (num_chunks) * HIVE_ISP_DDR_WORD_BYTES)
#if 0
#define __ISP_RGBA_WIDTH(rgba, num_chunks) \
	((rgba) ? (num_chunks)*4*2*ISP_VEC_NELEMS : 0)
#else
#define __ISP_RGBA_WIDTH(rgba, num_chunks) \
	(0)
#endif
#define __ISP_INTERNAL_WIDTH(out_width, \
			     dvs_env_width, \
			     left_crop, \
			     mode, \
			     c_subsampling, \
			     num_chunks, \
			     pipelining, \
			     rgba) \
	CEIL_MUL(CEIL_MUL(MAX(MAX(__ISP_PADDED_OUTPUT_WIDTH(out_width, \
							    dvs_env_width, \
							    left_crop), \
				  __ISP_MIN_INTERNAL_WIDTH(num_chunks, \
							   pipelining, \
							   mode) \
				 ), \
			      __ISP_RGBA_WIDTH(rgba, num_chunks) \
			     ), \
			  __ISP_CHUNK_STRIDE_ISP(mode) \
			 ), \
		 __ISP_CHUNK_STRIDE_DDR(c_subsampling, num_chunks) \
		)

#define __ISP_INTERNAL_HEIGHT(out_height, dvs_env_height, top_crop) \
	((out_height) + (dvs_env_height) + top_crop)

#define _ISP_MAX_INPUT_WIDTH(max_internal_width, enable_ds) \
	((enable_ds) ? SH_CSS_MAX_SENSOR_WIDTH : max_internal_width)

#define _ISP_INPUT_WIDTH(internal_width, ds_input_width, enable_ds) \
	((enable_ds) ? (ds_input_width) : (internal_width))

#define _ISP_INPUT_HEIGHT(internal_height, ds_input_height, enable_ds) \
	((enable_ds) ? (ds_input_height) : (internal_height))

#define SH_CSS_VF_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_NV11, \
		SH_CSS_FRAME_FORMAT_NV12, \
		SH_CSS_FRAME_FORMAT_NV16, \
		SH_CSS_FRAME_FORMAT_NV21, \
		SH_CSS_FRAME_FORMAT_NV61, \
		SH_CSS_FRAME_FORMAT_YV12, \
		SH_CSS_FRAME_FORMAT_YV16, \
		SH_CSS_FRAME_FORMAT_YUV420, \
		SH_CSS_FRAME_FORMAT_YUV420_16, \
		SH_CSS_FRAME_FORMAT_YUV422, \
		SH_CSS_FRAME_FORMAT_YUV422_16, \
		SH_CSS_FRAME_FORMAT_UYVY, \
		SH_CSS_FRAME_FORMAT_YUYV, \
		SH_CSS_FRAME_FORMAT_RGB565, \
		SH_CSS_FRAME_FORMAT_PLANAR_RGB888, \
		SH_CSS_FRAME_FORMAT_RGBA888 \
	}

#define SH_CSS_VIDEO_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_NV11, \
		SH_CSS_FRAME_FORMAT_NV12, \
		SH_CSS_FRAME_FORMAT_NV16, \
		SH_CSS_FRAME_FORMAT_NV21, \
		SH_CSS_FRAME_FORMAT_NV61, \
		SH_CSS_FRAME_FORMAT_YV12, \
		SH_CSS_FRAME_FORMAT_YV16, \
		SH_CSS_FRAME_FORMAT_YUV420, \
		SH_CSS_FRAME_FORMAT_YUV420_16, \
		SH_CSS_FRAME_FORMAT_YUV422, \
		SH_CSS_FRAME_FORMAT_YUV422_16, \
		SH_CSS_FRAME_FORMAT_UYVY, \
		SH_CSS_FRAME_FORMAT_YUYV \
	}

#define SH_CSS_CAPTURE_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_NV12, \
		SH_CSS_FRAME_FORMAT_NV16, \
		SH_CSS_FRAME_FORMAT_NV21, \
		SH_CSS_FRAME_FORMAT_NV61, \
		SH_CSS_FRAME_FORMAT_YV12, \
		SH_CSS_FRAME_FORMAT_YV16, \
		SH_CSS_FRAME_FORMAT_YUV420, \
		SH_CSS_FRAME_FORMAT_YUV420_16, \
		SH_CSS_FRAME_FORMAT_YUV422, \
		SH_CSS_FRAME_FORMAT_YUV422_16, \
		SH_CSS_FRAME_FORMAT_UYVY, \
		SH_CSS_FRAME_FORMAT_YUYV, \
		SH_CSS_FRAME_FORMAT_YUV444, \
		SH_CSS_FRAME_FORMAT_RGB565, \
		SH_CSS_FRAME_FORMAT_PLANAR_RGB888, \
		SH_CSS_FRAME_FORMAT_RGBA888 \
	}

#define SH_CSS_CAPTURE_DS_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_NV12, \
		SH_CSS_FRAME_FORMAT_NV16, \
		SH_CSS_FRAME_FORMAT_NV21, \
		SH_CSS_FRAME_FORMAT_NV61, \
		SH_CSS_FRAME_FORMAT_YV12, \
		SH_CSS_FRAME_FORMAT_YV16, \
		SH_CSS_FRAME_FORMAT_YUV420, \
		SH_CSS_FRAME_FORMAT_YUV420_16, \
		SH_CSS_FRAME_FORMAT_YUV422, \
		SH_CSS_FRAME_FORMAT_YUV422_16, \
		SH_CSS_FRAME_FORMAT_UYVY, \
		SH_CSS_FRAME_FORMAT_YUYV, \
		SH_CSS_FRAME_FORMAT_YUV444, \
		SH_CSS_FRAME_FORMAT_RGB565, \
		SH_CSS_FRAME_FORMAT_PLANAR_RGB888 \
	}

#define SH_CSS_YUV422_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_NV12, \
		SH_CSS_FRAME_FORMAT_NV16, \
		SH_CSS_FRAME_FORMAT_NV21, \
		SH_CSS_FRAME_FORMAT_NV61, \
		SH_CSS_FRAME_FORMAT_YV12, \
		SH_CSS_FRAME_FORMAT_YV16, \
		SH_CSS_FRAME_FORMAT_YUV420, \
		SH_CSS_FRAME_FORMAT_YUV420_16, \
		SH_CSS_FRAME_FORMAT_YUV422, \
		SH_CSS_FRAME_FORMAT_YUV422_16, \
		SH_CSS_FRAME_FORMAT_UYVY, \
		SH_CSS_FRAME_FORMAT_YUYV \
	}

#define SH_CSS_PRE_ISP_OUTPUT_FORMATS \
	{ \
		SH_CSS_FRAME_FORMAT_RAW, \
		SH_CSS_FRAME_FORMAT_QPLANE6 \
	}

/* Fixed resolution primaries only output this format */
#define SH_CSS_FIXED_PRIMARY_FORMAT SH_CSS_FRAME_FORMAT_NV12

#endif /* _SH_CSS_DEFS_H_ */
