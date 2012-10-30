/**
 * file vsp.h
 * Author: Binglin Chen <binglin.chen@intel.com>
 *
 */

/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
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
	VspFencePictureParamCommand = 0xEBEC,
	VspSetContextCommand = 0xEBED
};

#define VSP_CMD_QUEUE_SIZE (64)
#define VSP_ACK_QUEUE_SIZE (64)

/*
 * Command types and data structure.
 * Each command has a type. Depending on the type there is some kind
 * of data in external memory,
 * The VSS will use its DMA to load data from the buffer into local memory.
 */
struct vss_command_t {
	unsigned int       context;
	unsigned int       type;
	unsigned int       buffer;
	unsigned int       size;
	unsigned int       buffer_id;
	unsigned int       irq;
	unsigned int       reserved6;
	unsigned int       reserved7;
};

struct vss_response_t {
	unsigned int       context;
	unsigned int       type;
	unsigned int       buffer;
	unsigned int       size;
	unsigned int       vss_cc;
	unsigned int       reserved5;
	unsigned int       reserved6;
	unsigned int       reserved7;
};

/* Default initial values for vsp-command and vsp-response
* Using those avoids the risk of uninitialized warnings when
* the definition changes.
*/
#define VSP_COMMAND_INITIALIZER {0, 0, 0, 0, 0, 0, 0, 0}
#define VSP_RESPONSE_INITIALIZER {0, 0, 0, 0, 0, 0, 0, 0}

/*
 * Response types
 */
enum VssResponseType {
	VssCommandBufferReadyResponse,
	VssInputSurfaceReadyResponse,
	VssOutputSurfaceReadyResponse,
	VssOutputSurfaceFreeResponse,
	VssOutputSurfaceCrcResponse,
	VssEndOfSequenceResponse,
	VssErrorResponse,
	VssIdleResponse
};

enum VssStatus {
	VssOK,
	VssInvalidCommandType,
	VssInvalidCommandArgument,
	VssInvalidProcPictureCommand
};

enum vsp_format {
	VSP_NV12,
	VSP_YV12,
	VSP_UYVY,
	VSP_YUY2,
	VSP_NV11,
	VSP_NV16,
	VSP_IYUV,
	VSP_TYPE_ERROR
};

struct vsp_data {
	unsigned int fw_state;
	unsigned int uninit_req;
};

#define VSP_FIRMWARE_MAGIC_NUMBER 0x45BF1835
#define VSP_MAX_PROGRAMS 16

enum vsp_processor {
	vsp_sp0 = 0,
	vsp_sp1 = 1
};

struct vsp_config {
	unsigned int magic_number;
	unsigned int num_programs;
	/* array of program offsets */
	unsigned int program_offset[VSP_MAX_PROGRAMS];
	/* offsets in string table */
	unsigned int program_name_offset[VSP_MAX_PROGRAMS];
	char string_table[256];

	unsigned int boot_processor;
	unsigned int api_processor;

	/* boot program info */
	unsigned int text_src;
	unsigned int data_src;
	unsigned int data_dst;
	unsigned int data_size;
	unsigned int bss_dst;
	unsigned int bss_size;

	/* PC of init entry function */
	unsigned int init_addr;
	/* PC of main entry function */
	unsigned int main_addr;
};

struct vsp_ctrl_reg {
	/* VSP_FIRMWARE_ADDR_REG */
	unsigned int firmware_addr;

	/* VSP_CMD_QUEUE_ADDR_REG */
	unsigned int cmd_queue_addr;

	/* VSP_ACK_QUEUE_ADDR_REG */
	unsigned int ack_queue_addr;

	union {
		/* VSP_ENTRY_KIND_REG */
		unsigned int entry_kind;

		/* VSP_UNINIT_REQ_REG */
		unsigned int uninit_req;
	};

	/* VSP_CONTEXT_INIT_REQ_REG */
	unsigned int context_init_req;

	/* VSP_CONTEXT_INIT_ACK_REG */
	unsigned int context_init_ack;

	/* VSP_CONTEXT_UNINIT_REQ_REG */
	unsigned int context_uninit_req;

	/* VSP_CONTEXT_UNINIT_ACK_REG */
	unsigned int context_uninit_ack;

	/* VSP_CONTEXT_BUFFER_ADDR_REG */
	unsigned int context_buf_addr;

	/* VSP_CONTEXT_BUFFER_SIZE_REG */
	unsigned int context_buf_sz;

	/* VSP_CMD_QUEUE_RD_REG */
	unsigned int cmd_rd;

	/* VSP_CMD_QUEUE_WR_REG */
	unsigned int cmd_wr;

	/* VSP_ACK_QUEUE_RD_REG */
	unsigned int ack_rd;

	/* VSP_ACK_QUEUE_WR_REG */
	unsigned int ack_wr;
};

/* values passed via VSP_ENTRY_TYPE_REG */
enum vsp_entry_kind {
	vsp_entry_init = 0,
	vsp_entry_continue = 1,
	vsp_entry_resume = 2,
	vsp_exit = 3
};

#endif
