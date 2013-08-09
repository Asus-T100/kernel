/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#ifndef _SH_CSS_PARAMS_DVS_H_
#define _SH_CSS_PARAMS_DVS_H_

#include "ia_css.h"

#define DVS_ENV_MIN_X (12)
#define DVS_ENV_MIN_Y (12)
 
#define DVS_BLOCKDIM_X (64)        /* X block height*/
#define DVS_BLOCKDIM_Y_LUMA (64)   /* Y block height*/
#define DVS_BLOCKDIM_Y_CHROMA (32) /* UV height block size is half the Y block height*/

#define DVS_NUM_BLOCKS_X(X)    (CEIL_MUL(CEIL_DIV((X), DVS_BLOCKDIM_X), 2))      // horizontal 64x64 blocks round up to DVS_BLOCKDIM_X, make even
#define DVS_NUM_BLOCKS_Y(X)             (CEIL_DIV((X), DVS_BLOCKDIM_Y_LUMA))     // vertical   64x64 blocks round up to DVS_BLOCKDIM_Y
#define DVS_NUM_BLOCKS_X_CHROMA(X)      (CEIL_DIV((X), DVS_BLOCKDIM_X))
#define DVS_NUM_BLOCKS_Y_CHROMA(X)      (CEIL_DIV((X), DVS_BLOCKDIM_Y_CHROMA))


#define DVS_TABLE_IN_BLOCKDIM_X_LUMA(X)   	(DVS_NUM_BLOCKS_X(X) + 1)  // N blocks have N + 1 set of coords 
#define DVS_TABLE_IN_BLOCKDIM_X_CHROMA(X)   (DVS_NUM_BLOCKS_X_CHROMA(X) + 1)
#define DVS_TABLE_IN_BLOCKDIM_Y_LUMA(X)		(DVS_NUM_BLOCKS_Y(X) + 1)
#define DVS_TABLE_IN_BLOCKDIM_Y_CHROMA(X)	(DVS_NUM_BLOCKS_Y_CHROMA(X) + 1)

#define DVS_ENVELOPE_X(X) (((X) == 0) ? (DVS_ENV_MIN_X) : (X))
#define DVS_ENVELOPE_Y(X) (((X) == 0) ? (DVS_ENV_MIN_Y) : (X))

#define DVS_COORD_FRAC_BITS (10)
#define DVS_INPUT_BYTES_PER_PIXEL (1)
#define XMEM_ALIGN_LOG2 (5)

#define DVS_6AXIS_COORDS_ELEMS CEIL_MUL(sizeof(gdc_warp_param_mem_t) \
					, HIVE_ISP_DDR_WORD_BYTES)

#define DVS_6AXIS_BYTES(binary) \
	(DVS_6AXIS_COORDS_ELEMS \
     *  DVS_NUM_BLOCKS_X((binary)->out_frame_info.res.width) \
     *  DVS_NUM_BLOCKS_Y((binary)->out_frame_info.res.height)   )

struct ia_css_dvs_6axis_config *
generate_dvs_6axis_table(const struct ia_css_resolution	*frame_res, const struct ia_css_resolution *dvs_offset);

void
free_dvs_6axis_table(struct ia_css_dvs_6axis_config  **dvs_6axis_config);

void 
copy_dvs_6axis_table(struct ia_css_dvs_6axis_config *dvs_config_dst,
			 const struct ia_css_dvs_6axis_config *dvs_config_src);


#endif
