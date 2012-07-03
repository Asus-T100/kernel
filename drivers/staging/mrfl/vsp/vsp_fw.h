/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
 * All Rights Reserved.
 * Copyright (c) 2008, Tungsten Graphics Inc.  Cedar Park, TX., USA.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#ifndef _VSP_FW_H_
#define _VSP_FW_H_

#define VssProcPipelineMaxNumFilters 5
#define VSS_PROC_MAX_INPUT_PICTURES  1
#define VSS_PROC_MAX_OUTPUT_PICTURES 4

enum VssProcFilterType {
	VssProcFilterDenoise,
	VssProcFilterSharpening,
	VssProcFilterColorEnhancement,
	VssProcFilterFrameRateConversion
};

enum VssDenoiseType {
	VssProcDegrain,
	VssProcDeblock
};

enum VssFrcQuality {
	/* VssFrcLowQuality, */
	VssFrcMediumQuality,
	VssFrcHighQuality
};

enum VssFrcConversionRate {
	VssFrc2xConversionRate,
	VssFrc2_5xConversionRate,
	VssFrc4xConversionRate
};

struct VssProcPipelineParameterBuffer {
	unsigned int      num_filters;
	enum VssProcFilterType filter_pipeline[VssProcPipelineMaxNumFilters];
	/* VssRectangle      output_region; */
	/* unsigned int      output_background_color; */
	/* VssColorPrimaries output_color_primaries; */
};

struct VssProcSharpenParameterBuffer {
	int quality;
};

struct VssProcDenoiseParameterBuffer {
	enum VssDenoiseType     type;
	int                value_thr;
	int                cnt_thr;
	int                coef;
	int                temp_thr1;
	int                temp_thr2;
};

struct VssProcColorEnhancementParameterBuffer {
	int                temp_detect;
	int                temp_correct;
	int                clip_thr;
	int                mid_thr;
	int                luma_amm;
	int                chroma_amm;
};

struct VssProcFrcParameterBuffer {
	enum VssFrcQuality quality;
	enum VssFrcConversionRate conversion_rate;
};

struct VssProcPicture {
	unsigned int surface_id;
	/* send interupt when input or output surface is ready */
	unsigned int irq;
	unsigned int base;
	unsigned int height;
	unsigned int width;
	unsigned int stride;
	unsigned int format;
};

struct VssProcPictureParameterBuffer {
	unsigned int num_input_pictures;
	unsigned int num_output_pictures;
	struct VssProcPicture input_picture[VSS_PROC_MAX_INPUT_PICTURES];
	struct VssProcPicture output_picture[VSS_PROC_MAX_OUTPUT_PICTURES];
};

union VssProcBuffer {
	struct VssProcPipelineParameterBuffer         pipeline;
	struct VssProcSharpenParameterBuffer          sharpen_base;
	struct VssProcDenoiseParameterBuffer          denoiser_base;
	struct VssProcColorEnhancementParameterBuffer enhancer_base;
	struct VssProcFrcParameterBuffer              frc;
	struct VssProcPictureParameterBuffer          picture;
};

enum VssProcCommandType {
	VssProcPipelineParameterCommand = 0xFFFE,
	VssProcSharpenParameterCommand = 0xFFFD,
	VssProcDenoiseParameterCommand = 0xFFFC,
	VssProcColorEnhancementParameterCommand = 0xFFFB,
	VssProcFrcParameterCommand = 0xFFFA,
	VssProcPictureCommand = 0xFFF9,
	VspFencePictureParamCommand = 0xEBEC
};


/*
 * Command types and data structure.
 * Each command has a type. Depending on the type there is some kind
 * of data in external memory,
 * The VSS will use its DMA to load data from the buffer into local memory.
 */
struct vss_command_t {
	unsigned int       type;
	unsigned int       buffer;
	unsigned int       size;
	unsigned int       buffer_id;
	unsigned int       irq;
};

struct vss_response_t {
	unsigned int       type;
	unsigned int       buffer;
	unsigned int       size;
};


/*
 * Response types
 */
enum VssResponseType {
	VssCommandBufferReadyResponse,
	VssInputSurfaceReadyResponse,
	VssOutputSurfaceReadyResponse,
	VssEndOfSequenceResponse,
	VssErrorResponse
};

enum VssStatus {
	VssOK,
	VssInvalidCommandType,
	VssInvalidCommandArgument,
	VssInvalidProcPictureCommand
};

struct vss_queue {
	unsigned int wr;
	unsigned int rd;
	unsigned int size;
	unsigned int buffer;
};

enum vsp_format {
	VSP_NV12,
	VSP_YV12,
	VSP_UYVY,
	VSP_YUY2,
	VSP_NV11,
	VSP_NV16,
	VSP_TYPE_ERROR
};

#endif
