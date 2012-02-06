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

#ifndef _SH_CSS_TYPES_H_
#define _SH_CSS_TYPES_H_

/* This code is also used by Silicon Hive in a simulation environment
 * Therefore, the following macro is used to differentiate when this
 * code is being included from within the Linux kernel source
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>       /* for memcpy */
#else
#include <stdarg.h>             /* for the print function */
#include <stdlib.h>             /* for size_t */
#include <string.h>             /* for memcpy */
#include "sh_css_min_max.h"
#ifdef STDC99
#include <stdbool.h>
#else
#include "sh_css_bool.h"
#endif /* STDC99 */
#endif

#define SH_CSS_MAJOR    0
#define SH_CSS_MINOR    2
#define SH_CSS_REVISION 5

#define SH_CSS_MACC_NUM_AXES           16
#define SH_CSS_MACC_NUM_COEFS          4
#define SH_CSS_MORPH_TABLE_NUM_PLANES  6
#define SH_CSS_SC_NUM_COLORS           4
#define SH_CSS_CTC_TABLE_SIZE          1024
#define SH_CSS_GAMMA_TABLE_SIZE        1024
#define SH_CSS_DIS_NUM_COEF_TYPES      6
#define SH_CSS_DIS_COEF_TYPES_ON_DMEM  2
#define SH_CSS_CTC_COEF_SHIFT          13
#define SH_CSS_GAMMA_GAIN_K_SHIFT      13

/* Fixed point types.
 * NOTE: the 16 bit fixed point types actually occupy 32 bits
 * to save on extension operations in the ISP code.
 */
#define u0_16 unsigned int	/* unsigned 0.16 fixed point type */
#define u2_14 unsigned int	/* unsigned 2.14 fixed point type */
#define u5_11 unsigned int	/* unsigned 5.11 fixed point type */
#define u8_8  unsigned int	/* unsigned 8.8 fixed point type */
#define s0_15 signed int	/* signed 0.15 fixed point type */

/* Errors, these values are used as the return value for most
   functions in this API. These can be translated into a human readable
   string (see below). */
enum sh_css_err {
	sh_css_success,
	sh_css_err_internal_error,
	sh_css_err_conflicting_mipi_settings,
	sh_css_err_unsupported_configuration,
	sh_css_err_mode_does_not_have_viewfinder,
	sh_css_err_input_resolution_not_set,
	sh_css_err_unsupported_input_mode,
	sh_css_err_cannot_allocate_memory,
	sh_css_err_invalid_arguments,
	sh_css_err_too_may_colors,
	sh_css_err_overlay_frame_missing,
	sh_css_err_overlay_frames_too_big,
	sh_css_err_unsupported_frame_format,
	sh_css_err_frames_mismatch,
	sh_css_err_overlay_not_set,
	sh_css_err_not_implemented,
	sh_css_err_invalid_frame_format,
	sh_css_err_unsupported_resolution,
	sh_css_err_scaling_factor_out_of_range,
	sh_css_err_cannot_obtain_shading_table,
	sh_css_err_interrupt_error,
	sh_css_err_unexpected_interrupt,
	sh_css_err_interrupts_not_enabled,
	sh_css_err_system_not_idle,
	sh_css_err_unsupported_input_format,
	sh_css_err_not_enough_input_lines,
	sh_css_err_not_enough_input_columns,
	sh_css_err_illegal_resolution,
	sh_css_err_effective_input_resolution_not_set,
	sh_css_err_viewfinder_resolution_too_wide,
	sh_css_err_viewfinder_resolution_exceeds_output,
	sh_css_err_mode_does_not_have_grid,
	sh_css_err_mode_does_not_have_raw_output
};

/* Input modes, these enumerate all supported input modes:
 *  - Sensor: data from a sensor coming into the MIPI interface
 *  - FIFO:   data from the host coming into the GP FIFO
 *  - TPG:    data coming from the test pattern generator
 *  - PRBS:   data coming from the Pseudo Random Bit Sequence
 *  - Memory: data coming from DDR
 */
enum sh_css_input_mode {
	SH_CSS_INPUT_MODE_SENSOR,
	SH_CSS_INPUT_MODE_FIFO,
	SH_CSS_INPUT_MODE_TPG,
	SH_CSS_INPUT_MODE_PRBS,
	SH_CSS_INPUT_MODE_MEMORY
};

/* The MIPI interface can be used in two modes, one with one
 * lane and one with 4 lanes.
 */
enum sh_css_mipi_port {
	SH_CSS_MIPI_PORT_1LANE,
	SH_CSS_MIPI_PORT_4LANE
};

/* The MIPI interface supports 2 types of compression or can
 * be run without compression.
 */
enum sh_css_mipi_compression {
	SH_CSS_MIPI_COMPRESSION_NONE,
	SH_CSS_MIPI_COMPRESSION_1,
	SH_CSS_MIPI_COMPRESSION_2
};

/* The ISP streaming input interface supports the following formats.
 * These match the corresponding MIPI formats.
 */
enum sh_css_input_format {
	SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY,    /* 8 bits per subpixel */
	SH_CSS_INPUT_FORMAT_YUV420_8,   /* 8 bits per subpixel */
	SH_CSS_INPUT_FORMAT_YUV420_10,  /* 10 bits per subpixel */
	SH_CSS_INPUT_FORMAT_YUV422_8,   /* UYVY..UVYV, 8 bits per subpixel */
	SH_CSS_INPUT_FORMAT_YUV422_10,  /* UYVY..UVYV, 10 bits per subpixel */
	SH_CSS_INPUT_FORMAT_RGB_444,    /* BGR..BGR, 4 bits per subpixel */
	SH_CSS_INPUT_FORMAT_RGB_555,    /* BGR..BGR, 5 bits per subpixel */
	SH_CSS_INPUT_FORMAT_RGB_565,    /* BGR..BGR, 5 bits B and $, 6 bits G */
	SH_CSS_INPUT_FORMAT_RGB_666,    /* BGR..BGR, 6 bits per subpixel */
	SH_CSS_INPUT_FORMAT_RGB_888,    /* BGR..BGR, 8 bits per subpixel */
	SH_CSS_INPUT_FORMAT_RAW_6,      /* RAW data, 6 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_7,      /* RAW data, 7 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_8,      /* RAW data, 8 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_10,     /* RAW data, 10 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_12,     /* RAW data, 12 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_14,     /* RAW data, 14 bits per pixel */
	SH_CSS_INPUT_FORMAT_RAW_16,     /* RAW data, 16 bits per pixel */
	SH_CSS_INPUT_FORMAT_BINARY_8,   /* Binary byte stream. */
};

/* Specify the capture mode, this can be RAW (simply copy sensor input to DDR),
 * Primary ISP or the Advanced ISP.
 */
enum sh_css_capture_mode {
	SH_CSS_CAPTURE_MODE_RAW,        /* no processing, only copy input to
					   output, no viewfinder output */
	SH_CSS_CAPTURE_MODE_PRIMARY,    /* primary ISP */
	SH_CSS_CAPTURE_MODE_ADVANCED,   /* advanced ISP */
	SH_CSS_CAPTURE_MODE_LOW_LIGHT,   /* low light ISP */
};

/* Interrupt info enumeration.
 * This lists all possible interrupts for use by the appliation layer.
 * Note that the sh_css API uses some internal interrupts, these are not listed
 * here.
 */
enum sh_css_interrupt_info {
	/* the current frame is done and a new one can be started */
	SH_CSS_IRQ_INFO_FRAME_DONE = 1 << 0,
	/* another stage (ISP binary) needs to be started. */
	SH_CSS_IRQ_INFO_START_NEXT_STAGE = 1 << 1,
	/* 3A + DIS statistics are ready. */
	SH_CSS_IRQ_INFO_STATISTICS_READY = 1 << 2,
	/* the css receiver has encountered an error */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR = 1 << 3,
	/* the FIFO in the csi receiver has overflown */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW = 1 << 4,
	/* the css receiver received the start of frame */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_SOF = 1 << 5,
	/* the css receiver received the end of frame */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_EOF = 1 << 6,
	/* the css receiver received the start of line */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_SOL = 1 << 7,
	/* the css receiver received the end of line */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_EOL = 1 << 8,
	/* the css receiver received a change in side band signals */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED = 1 << 9,
	/* generic short packets (0) */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0 = 1 << 10,
	/* generic short packets (1) */
	SH_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1 = 1 << 11,
	/* the primary input formatter (A) has encountered an error */
	SH_CSS_IRQ_INFO_IF_PRIM_ERROR = 1 << 12,
	/* the primary input formatter (B) has encountered an error */
	SH_CSS_IRQ_INFO_IF_PRIM_B_ERROR = 1 << 13,
	/* the secondary input formatter has encountered an error */
	SH_CSS_IRQ_INFO_IF_SEC_ERROR = 1 << 14,
	/* the stream-to-memory device has encountered an error */
	SH_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR = 1 << 15,
	/* A firmware accelerator has terminated */
	SH_CSS_IRQ_INFO_FW_ACC_DONE = 1 << 16,
	/* software interrupts */
	SH_CSS_IRQ_INFO_SW_0 = 1 << 17,
	SH_CSS_IRQ_INFO_SW_1 = 1 << 18,
	SH_CSS_IRQ_INFO_SW_2 = 1 << 19,
	/* Inform the ISR that there is an invalid first frame */
	SH_CSS_IRQ_INFO_INVALID_FIRST_FRAME = 1 << 20,
};

enum sh_css_rx_irq_info {
	SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN   = 1 << 0,
	SH_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE = 1 << 1,
	SH_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE  = 1 << 2,
	SH_CSS_RX_IRQ_INFO_ECC_CORRECTED    = 1 << 3,
	SH_CSS_RX_IRQ_INFO_ERR_SOT          = 1 << 4,
	SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC     = 1 << 5,
	SH_CSS_RX_IRQ_INFO_ERR_CONTROL      = 1 << 6,
	SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE   = 1 << 7,
	SH_CSS_RX_IRQ_INFO_ERR_CRC          = 1 << 8,
	SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID   = 1 << 9,
	SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC   = 1 << 10,
	SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA   = 1 << 11,
	SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT = 1 << 12,
	SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC  = 1 << 13,
	SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC    = 1 << 14,
};

/* Enumeration used to select whether interrupts should be used, and if so,
 * whether they are edge or pulse triggered.
 * If interrupts are not used, the blocking function
 * sh_css_wait_for_completion() must be used.
 */
enum sh_css_interrupt_setting {
	SH_CSS_INTERRUPT_SETTING_EDGE,
	SH_CSS_INTERRUPT_SETTING_PULSE
};

/* Frame formats, some of these come from fourcc.org, others are
   better explained by video4linux2. The NV11 seems to be described only
   on MSDN pages, but even those seem to be gone now.
   Frames can come in many forms, the main categories are RAW, RGB and YUV
   (or YCbCr). The YUV frames come in 4 flavors, determined by how the U and V
   values are subsampled:
   1. YUV420: hor = 2, ver = 2
   2. YUV411: hor = 4, ver = 1
   3. YUV422: hor = 2, ver = 1
   4. YUV444: hor = 1, ver = 1
 */
enum sh_css_frame_format {
	SH_CSS_FRAME_FORMAT_NV11,       /* 12 bit YUV 411, Y, UV plane */
	SH_CSS_FRAME_FORMAT_NV12,       /* 12 bit YUV 420, Y, UV plane */
	SH_CSS_FRAME_FORMAT_NV16,       /* 16 bit YUV 422, Y, UV plane */
	SH_CSS_FRAME_FORMAT_NV21,       /* 12 bit YUV 420, Y, VU plane */
	SH_CSS_FRAME_FORMAT_NV61,       /* 16 bit YUV 422, Y, VU plane */
	SH_CSS_FRAME_FORMAT_YV12,       /* 12 bit YUV 420, Y, V, U plane */
	SH_CSS_FRAME_FORMAT_YV16,       /* 16 bit YUV 422, Y, V, U plane */
	SH_CSS_FRAME_FORMAT_YUV420,     /* 12 bit YUV 420, Y, U, V plane */
	SH_CSS_FRAME_FORMAT_YUV420_16,  /* yuv420, 16 bits per subpixel */
	SH_CSS_FRAME_FORMAT_YUV422,     /* 16 bit YUV 422, Y, U, V plane */
	SH_CSS_FRAME_FORMAT_YUV422_16,  /* yuv422, 16 bits per subpixel */
	SH_CSS_FRAME_FORMAT_UYVY,       /* 16 bit YUV 422, UYVY interleaved */
	SH_CSS_FRAME_FORMAT_YUYV,       /* 16 bit YUV 422, YUYV interleaved */
	SH_CSS_FRAME_FORMAT_YUV444,     /* 24 bit YUV 444, Y, U, V plane */
	SH_CSS_FRAME_FORMAT_YUV_LINE,   /* Internal format, e.g. for VBF frame.
					   2 y lines followed by a uv
					   interleaved line */
	SH_CSS_FRAME_FORMAT_RAW,        /* RAW, 1 plane */
	SH_CSS_FRAME_FORMAT_RGB565,     /* 16 bit RGB, 1 plane. Each 3 sub
					   pixels are packed into one 16 bit
					   value, 5 bits for R, 6 bits for G
					   and 5 bits for B. */
	SH_CSS_FRAME_FORMAT_PLANAR_RGB888, /* 24 bit RGB, 3 planes */
	SH_CSS_FRAME_FORMAT_RGBA888,    /* 32 bit RGBA, 1 plane, A=Alpha unused
					 */
	SH_CSS_FRAME_FORMAT_QPLANE6,    /* Internal, for advanced ISP */
	SH_CSS_FRAME_FORMAT_BINARY_8,   /* byte stream, used for jpeg. For
					   frames of this type, we set the
					   height to 1 and the width to the
					   number of allocated bytes. */
};

struct sh_css_frame_plane {
	unsigned int height;	/* height of a plane in lines */
	unsigned int width;	/* width of a line, in DMA elements, note that
				   for RGB565 the three subpixels are stored in
				   one element. For all other formats this is
				   the number of subpixels per line. */
	unsigned int stride;	/* stride of a line in bytes */
	void *data;		/* pointer that points into frame data */
};

struct sh_css_frame_binary_plane {
	unsigned int size;
	struct sh_css_frame_plane data;
};

struct sh_css_frame_yuv_planes {
	struct sh_css_frame_plane y;
	struct sh_css_frame_plane u;
	struct sh_css_frame_plane v;
};

struct sh_css_frame_nv_planes {
	struct sh_css_frame_plane y;
	struct sh_css_frame_plane uv;
};

struct sh_css_frame_rgb_planes {
	struct sh_css_frame_plane r;
	struct sh_css_frame_plane g;
	struct sh_css_frame_plane b;
};

struct sh_css_frame_plane6_planes {
	struct sh_css_frame_plane r;
	struct sh_css_frame_plane r_at_b;
	struct sh_css_frame_plane gr;
	struct sh_css_frame_plane gb;
	struct sh_css_frame_plane b;
	struct sh_css_frame_plane b_at_r;
};

/* For RAW input, the bayer order needs to be specified separately. There
   are 4 possible orders. The name is constructed by taking the first two
   colors on the first line and the first two colors from the second line.
grbg: GRGRGRGR
      BGBGBGBG
rgbg: RGRGRGRG
      GBGBGBGB
bggr: BGBGBGBG
      GRGRGRGR
gbrg: GBGBGBGB
      RGRGRGRG
 */
enum sh_css_bayer_order {
	sh_css_bayer_order_grbg,
	sh_css_bayer_order_rggb,
	sh_css_bayer_order_bggr,
	sh_css_bayer_order_gbrg
};

/* Frame info struct:
   This structure describes a frame. It contains the resolution and strides.
 */
struct sh_css_frame_info {
	/* width in valid data in pixels (not subpixels) */
	unsigned int width;
	/* height in lines of valid image data */
	unsigned int height;
	/* width of a line in memory, in pixels */
	unsigned int padded_width;
	/* format of the data in this frame */
	enum sh_css_frame_format format;
	/* number of valid bits per pixel, only valid for raw frames. */
	unsigned int raw_bit_depth;
	/* bayer order of raw data, only valid for raw frames. */
	enum sh_css_bayer_order raw_bayer_order;
};

struct sh_css_frame {
	struct sh_css_frame_info info;
	/* pointer to start of image data in memory */
	void *data;
	/* size of data pointer in bytes */
	unsigned int data_bytes;
	/* indicate whether memory is allocated physically contiguously */
	bool contiguous;
	union {
		struct sh_css_frame_plane raw;
		struct sh_css_frame_plane rgb;
		struct sh_css_frame_rgb_planes planar_rgb;
		struct sh_css_frame_plane yuyv;
		struct sh_css_frame_yuv_planes yuv;
		struct sh_css_frame_nv_planes nv;
		struct sh_css_frame_plane6_planes plane6;
		struct sh_css_frame_binary_plane binary;
	} planes;
};

/* Histogram. This contains num_elements values of type unsigned int.
 * The data pointer is a DDR pointer (virtual address).
 */
struct sh_css_histogram {
	unsigned int num_elements;
	void *data;
};

/* Overlay:
 * this is the structure describing the entire overlay.
 * An overlay consists of a frame (of type sh_css_frame_format_yuv420),
 * the background color (yuv) and the blending ratios for the subpixels
 * of the input data and the overlay data.
 * All pixels in the overlay that are not equal to the background are
 * overlaid, taking their blending ratio into account. The blending ratio
 * should be specified between 0 and 100.
 */
struct sh_css_overlay {
	/* the frame containing the overlay data The overlay frame width should
	 * be the multiples of 2*ISP_VEC_NELEMS. The overlay frame height
	 * should be the multiples of 2.
	 */
	struct sh_css_frame *frame;
	/* Y value of overlay background */
	unsigned char bg_y;
	/* U value of overlay background */
	char bg_u;
	/* V value of overlay background */
	char bg_v;
	/* the blending percent of input data for Y subpixels */
	unsigned char blend_input_perc_y;
	/* the blending percent of input data for U subpixels */
	unsigned char blend_input_perc_u;
	/* the blending percent of input data for V subpixels */
	unsigned char blend_input_perc_v;
	/* the blending percent of overlay data for Y subpixels */
	unsigned char blend_overlay_perc_y;
	/* the blending percent of overlay data for U subpixels */
	unsigned char blend_overlay_perc_u;
	/* the blending percent of overlay data for V subpixels */
	unsigned char blend_overlay_perc_v;
	/* the overlay start x pixel position on output frame It should be the
	   multiples of 2*ISP_VEC_NELEMS. */
	unsigned int overlay_start_x;
	/* the overlay start y pixel position on output frame It should be the
	   multiples of 2. */
	unsigned int overlay_start_y;
};

/* SP struct describing overlay properties */
struct sh_css_sp_overlay {
	int bg_y;
	int bg_u;
	int bg_v;
	int blend_shift;
	int blend_input_y;
	int blend_input_u;
	int blend_input_v;
	int blend_overlay_y;
	int blend_overlay_u;
	int blend_overlay_v;
	int overlay_width;
	int overlay_height;
	int overlay_start_x;
	int overlay_start_y;
	const char *frame_ptr_overlay_y;
	const char *frame_ptr_overlay_u;
	const char *frame_ptr_overlay_v;
};

/* structure that describes the 3A and DIS grids */
struct sh_css_grid_info {
	/* ISP input size that is visible for user */
	unsigned int isp_in_width;
	unsigned int isp_in_height;
	/* 3A statistics grid: */
	unsigned int s3a_width;
	unsigned int s3a_height;
	unsigned int s3a_bqs_per_grid_cell;
	/* DIS grid: */
	unsigned int dis_width;  /* also used for vertical projections */
	unsigned int dis_aligned_width;
	unsigned int dis_height; /* also used for horizontal projections */
	unsigned int dis_aligned_height;
	unsigned int dis_bqs_per_grid_cell;
	unsigned int dis_hor_coef_num;
	unsigned int dis_ver_coef_num;
};

enum sh_css_ob_mode {
	sh_css_ob_mode_none,
	sh_css_ob_mode_fixed,
	sh_css_ob_mode_raster
};

/* Shading correction */
enum sh_css_sc_color {
	SH_CSS_SC_COLOR_GR,
	SH_CSS_SC_COLOR_R,
	SH_CSS_SC_COLOR_B,
	SH_CSS_SC_COLOR_GB
};

/* White Balance (Gain Adjust) */
struct sh_css_wb_config {
	unsigned int integer_bits;
	unsigned int gr;	/* unsigned <integer_bits>.<16-integer_bits> */
	unsigned int r;		/* unsigned <integer_bits>.<16-integer_bits> */
	unsigned int b;		/* unsigned <integer_bits>.<16-integer_bits> */
	unsigned int gb;	/* unsigned <integer_bits>.<16-integer_bits> */
};

/* Color Space Conversion settings */
struct sh_css_cc_config {
	unsigned int fraction_bits;
	int matrix[3 * 3];	/* RGB2YUV Color matrix, signed
				   <13-fraction_bits>.<fraction_bits> */
};

/* Morphing table for advanced ISP.
 * Each line of width elements takes up COORD_TABLE_EXT_WIDTH elements
 * in memory.
 */
struct sh_css_morph_table {
	unsigned int height;
	unsigned int width;	/* number of valid elements per line */
	unsigned short *coordinates_x[SH_CSS_MORPH_TABLE_NUM_PLANES];
	unsigned short *coordinates_y[SH_CSS_MORPH_TABLE_NUM_PLANES];
};

struct sh_css_fpn_table {
	short *data;
	unsigned int width;
	unsigned int height;
	unsigned int shift;
};

struct sh_css_shading_table {
	/* native sensor resolution */
	unsigned int sensor_width;
	unsigned int sensor_height;
	/* number of data points per line per color (bayer quads) */
	unsigned int width;
	/* number of lines of data points per color (bayer quads) */
	unsigned int height;
	/* bits of fraction part for shading table values */
	unsigned int fraction_bits;
	/* one table for each color (use sh_css_sc_color to index) */
	unsigned short *data[SH_CSS_SC_NUM_COLORS];
};

struct sh_css_gamma_table {
	unsigned short data[SH_CSS_GAMMA_TABLE_SIZE];
};

struct sh_css_ctc_table {
	unsigned short data[SH_CSS_CTC_TABLE_SIZE];
};

struct sh_css_macc_table {
	short data[SH_CSS_MACC_NUM_COEFS * SH_CSS_MACC_NUM_AXES];
};

/* Temporal noise reduction configuration */
struct sh_css_tnr_config {
	u0_16 gain;		/* [gain] Strength of NR */
	u0_16 threshold_y;	/* [intensity] Motion sensitivity for Y */
	u0_16 threshold_uv;	/* [intensity] Motion sensitivity for U/V */
};

/* Optical black level configuration */
struct sh_css_ob_config {
	/* Obtical black level mode (Fixed / Raster) */
	enum sh_css_ob_mode mode;
	/* [intensity] optical black level for GR (relevant for fixed mode) */
	u0_16 level_gr;
	/* [intensity] optical black level for R (relevant for fixed mode) */
	u0_16 level_r;
	/* [intensity] optical black level for B (relevant for fixed mode) */
	u0_16 level_b;
	/* [intensity] optical black level for GB (relevant for fixed mode) */
	u0_16 level_gb;
	/* [BQ] 0..63 start position of OB area (relevant for raster mode) */
	unsigned short start_position;
	/* [BQ] start..63 end position of OB area (relevant for raster mode) */
	unsigned short end_position;
};

/* Defect pixel correction configuration */
struct sh_css_dp_config {
	/* [intensity] The threshold of defect Pixel Correction, representing
	 * the permissible difference of intensity between one pixel and its
	 * surrounding pixels. Smaller values result in more frequent pixel
	 * corrections.
	 */
	u0_16 threshold;
	/* [gain] The sensitivity of mis-correction. ISP will miss a lot of
	 * defects if the value is set too large.
	 */
	u8_8 gain;
};

/* Configuration used by Bayer noise reduction and YCC noise reduction */
struct sh_css_nr_config {
	/* [gain] Strength of noise reduction for Bayer NR (Used by Bayer NR) */
	u0_16 bnr_gain;
	/* [gain] Strength of noise reduction for YCC NR (Used by YCC NR) */
	u0_16 ynr_gain;
	/* [intensity] Sensitivity of Edge (Used by Bayer NR) */
	u0_16 direction;
	/* [intensity] coring threshold for Cb (Used by YCC NR) */
	u0_16 threshold_cb;
	/* [intensity] coring threshold for Cr (Used by YCC NR) */
	u0_16 threshold_cr;
};

/* Edge enhancement (sharpen) configuration */
struct sh_css_ee_config {
	/* [gain] The strength of sharpness. */
	u5_11 gain;
	/* [intensity] The threshold that divides noises from edge. */
	u8_8 threshold;
	/* [gain] The strength of sharpness in pell-mell area. */
	u5_11 detail_gain;
};

struct sh_css_de_config {
	u0_16 pixelnoise;
	u0_16 c1_coring_threshold;
	u0_16 c2_coring_threshold;
};

struct sh_css_gc_config {
	unsigned short gain_k1;
	unsigned short gain_k2;
};

struct sh_css_anr_config {
	int threshold;
};

struct sh_css_ce_config {
	u0_16 uv_level_min;
	u0_16 uv_level_max;
};

struct sh_css_3a_config {
	u0_16 ae_y_coef_r;	/* [gain] Weight of R for Y */
	u0_16 ae_y_coef_g;	/* [gain] Weight of G for Y */
	u0_16 ae_y_coef_b;	/* [gain] Weight of B for Y */
	s0_15 af_fir1_coef[7];	/* [factor] AF FIR coefficients of fir1 */
	s0_15 af_fir2_coef[7];	/* [factor] AF FIR coefficients of fir2 */
};

/* Guard this declaration, because this struct is also defined by
 * Sh3a_Types.h now
 */
#ifndef __SH_CSS_3A_OUTPUT__
#define __SH_CSS_3A_OUTPUT__

/* Workaround: hivecc complains about "tag "sh_css_3a_output" already declared"
   without this extra decl. */
struct sh_css_3a_output;

struct sh_css_3a_output {
	int ae_y;
	int awb_cnt;
	int awb_gr;
	int awb_r;
	int awb_b;
	int awb_gb;
	int af_hpf1;
	int af_hpf2;
};

#endif /* End of guard */

/* Descriptor of sp firmware blob */
struct sh_css_sp_fw {
	const void  *text;		/* Sp text section */
	unsigned int text_source;	/* Position of text in blob */
	unsigned int text_size;		/* Size of text section */
	const void  *data;		/* Sp data section */
	unsigned int data_source;	/* Position of data in blob */
	unsigned int data_target;	/* Start position of data in SP dmem */
	unsigned int data_size;		/* Size of text section */
	unsigned int bss_target;	/* Start position of bss in SP dmem */
	unsigned int bss_size;		/* Size of bss section */
	void *dmem_init_data;		/* Addr sp init data */
};

/* this struct contains all arguments that can be passed to
   a binary. It depends on the binary which ones are used. */
struct sh_css_binary_args {
	struct sh_css_frame *cc_frame;       /* continuous capture frame */
	struct sh_css_frame *in_frame;	     /* input frame */
	struct sh_css_frame *in_ref_frame;   /* reference input frame */
	struct sh_css_frame *in_tnr_frame;   /* tnr input frame */
	struct sh_css_frame *out_frame;      /* output frame */
	struct sh_css_frame *out_ref_frame;  /* reference output frame */
	struct sh_css_frame *out_tnr_frame;  /* tnr output frame */
	struct sh_css_frame *extra_frame;    /* intermediate frame */
	struct sh_css_frame *out_vf_frame;   /* viewfinder output frame */
	int                  dvs_vector_x;
	int                  dvs_vector_y;
	bool                 enable_xnr;
	bool                 two_ppc;
	bool                 copy_vf;
	bool                 copy_output;
	unsigned             vf_downscale_log2;
};

/* Type of acceleration */
enum sh_css_acc_type {
	SH_CSS_ACC_STANDALONE,	/* Stand-alone acceleration */
	SH_CSS_ACC_OUTPUT,	/* Accelerator stage on output frame */
	SH_CSS_ACC_VIEWFINDER	/* Accelerator stage on viewfinder frame */
};

/* Type of acceleration argument */
enum sh_css_acc_arg_type {
	SH_CSS_ACC_ARG_SCALAR_IN,    /* Scalar input argument */
	SH_CSS_ACC_ARG_SCALAR_OUT,   /* Scalar output argument */
	SH_CSS_ACC_ARG_SCALAR_IO,    /* Scalar in/output argument */
	SH_CSS_ACC_ARG_PTR_IN,	     /* Pointer input argument */
	SH_CSS_ACC_ARG_PTR_OUT,	     /* Pointer output argument */
	SH_CSS_ACC_ARG_PTR_IO,	     /* Pointer in/output argument */
	SH_CSS_ACC_ARG_PTR_NOFLUSH,  /* Pointer argument will not be flushed */
	SH_CSS_ACC_ARG_PTR_STABLE,   /* Pointer input argument that is stable */
	SH_CSS_ACC_ARG_FRAME	     /* Frame argument */
};

/* Descriptor for an SP argument */
struct sh_css_sp_arg {
	enum sh_css_acc_arg_type type;	 /* Type  of SP argument */
	void			*value;	 /* Value of SP argument */
	bool			 stable; /* Pointer is stable */
	unsigned int		 size;	 /* Size  of SP argument */
	void			*host;	 /* Private data used by host */
};

struct sh_css_acc_fw;

/* Firmware descriptor */
struct sh_css_acc_fw_hdr {
	enum sh_css_acc_type type;      /* Type of accelerator */
	bool		     loaded;    /* Firmware has been loaded */
	struct sh_css_sp_arg *sp_args;  /* Current SP argument */
	unsigned	     prog_name_offset; /* offset wrt hdr in bytes */
	unsigned	     arg_types_offset; /* offset wrt hdr in bytes */
	unsigned	     sp_blob_offset;   /* offset wrt hdr in bytes */
	unsigned	     isp_blob_offset;  /* offset wrt hdr in bytes */
	struct {
		unsigned int	     size;       /* Size of sp blob */
		void (*init) (struct sh_css_acc_fw *); /* init for crun */
		void		    *entry;      /* Address of sp entry point */
		unsigned int	    *args;       /* Address of sp_args */
		unsigned int	     args_cnt;   /* Number  of sp_args */
		unsigned int	     args_size;  /* Size    of sp_args */
		unsigned int	    *css_abort;  /* SP dmem abort flag */
		struct sh_css_frame *input;      /* SP dmem input frame */
		struct sh_css_frame *output;     /* SP dmem output frame */
		struct sh_css_frame *out_vf;     /* SP dmem vf frame */
		struct sh_css_frame *extra;      /* SP dmem extra frame */
		unsigned int	    *vf_downscale_bits;
		void		    *isp_code;   /* SP dmem address holding xmem
						    address of isp code */
		struct sh_css_sp_fw fw;		 /* SP fw descriptor */
	} sp;
	struct {
		unsigned int size;       /* Size of isp blob */
	} isp;
	/* To create a sequence of accelerators */
	struct sh_css_acc_fw *next;
	/* Firmware handle between user space and kernel */
	unsigned int handle;
	/* Hmm pointer of allocated SP code */
	const unsigned char *sp_code;
	/* Hmm pointer of allocated ISP code */
	const unsigned char *isp_code;
};

/* Firmware. Containing header and actual blobs */
struct sh_css_acc_fw {
	/* firmware header */
	struct sh_css_acc_fw_hdr header;
	/* followed by prog_name, sp arg types, sp blob and isp blob */
#ifdef __HIVECC
	unsigned char		 data[1]; /* Not C89 */
#else
	unsigned char		 data[];
#endif
};

/* Access macros for firmware */
#define SH_CSS_ACC_OFFSET(t, f, n) ((t)((unsigned char *)(f)+(f->header.n)))
#define SH_CSS_ACC_PROG_NAME(f)    SH_CSS_ACC_OFFSET(const char *, f, \
						 prog_name_offset)
#define SH_CSS_ACC_SP_ARGS(f)      SH_CSS_ACC_OFFSET(enum sh_css_acc_arg_type*,\
						 f, arg_types_offset)
#define SH_CSS_ACC_SP_CODE(f)      SH_CSS_ACC_OFFSET(unsigned char *, f, \
						 sp_blob_offset)
#define SH_CSS_ACC_SP_SIZE(f)      ((f)->header.sp.size)
#define SH_CSS_ACC_SP_DATA(f)      (SH_CSS_ACC_SP_CODE(f) + \
					(f)->header.sp.fw.data_source)
#define SH_CSS_ACC_ISP_CODE(f)     SH_CSS_ACC_OFFSET(unsigned char*, f,\
						 isp_blob_offset)
#define SH_CSS_ACC_ISP_SIZE(f)     ((f)->header.isp.size)
#define SH_CSS_ACC_SIZE(f)         ((f)->header.isp_blob_offset + \
					SH_CSS_ACC_ISP_SIZE(f))

/* Structure to encapsulate required arguments for
 * initialization of SP DMEM using the SP itself
 */
struct sh_css_sp_init_dmem_cfg {
	unsigned      done;	      /* Init has been done */
	void         *ddr_data_addr;  /* data segment address in ddr  */
	void         *dmem_data_addr; /* data segment address in dmem */
	unsigned int  data_size;      /* data segment size            */
	void         *dmem_bss_addr;  /* bss segment address in dmem  */
	unsigned int  bss_size;       /* bss segment size             */
};

#endif /* _SH_CSS_TYPES_H_ */
