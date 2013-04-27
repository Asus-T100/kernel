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

/*! \file */

#ifndef _IA_CSS_H_
#define _IA_CSS_H_

/* Move to "platform_support.h" */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stdint.h>
//#include <stdarg.h>
#endif

#include "ia_css_types.h"

/** Enumeration of pipe modes. This mode can be used to create
 *  an image pipe for this mode. These pipes can be combined
 *  to configure and run streams on the ISP.
 *
 *  For example, one can create a preview and capture pipe to
 *  create a continuous capture stream.
 */
enum ia_css_pipe_mode {
	IA_CSS_PIPE_MODE_PREVIEW,      /**< Preview pipe */
	IA_CSS_PIPE_MODE_VIDEO,        /**< Video pipe */
	IA_CSS_PIPE_MODE_CAPTURE,      /**< Still capture pipe */
	IA_CSS_PIPE_MODE_ACC,          /**< Accelerated pipe */
};
/* Temporary define  */
#define IA_CSS_PIPE_MODE_NUM (IA_CSS_PIPE_MODE_ACC + 1)

/** Input modes, these enumerate all supported input modes.
 *  Note that not all ISP modes support all input modes.
 */
enum ia_css_input_mode {
	IA_CSS_INPUT_MODE_SENSOR, /**< data from sensor */
	IA_CSS_INPUT_MODE_FIFO,   /**< data from input-fifo */
	IA_CSS_INPUT_MODE_TPG,    /**< data from test-pattern generator */
	IA_CSS_INPUT_MODE_PRBS,   /**< data from pseudo-random bit stream */
	IA_CSS_INPUT_MODE_MEMORY, /**< data from a frame in memory */
	IA_CSS_INPUT_MODE_BUFFERED_SENSOR /**< data is sent through mipi buffer */ 
};

enum ia_css_irq_type {
	IA_CSS_IRQ_TYPE_EDGE,  /**< Edge (level) sensitive interrupt */
	IA_CSS_IRQ_TYPE_PULSE  /**< Pulse-shaped interrupt */
};

/** The ISP streaming input interface supports the following formats.
 *  These match the corresponding MIPI formats.
 */
enum ia_css_stream_format {
	IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY,    /**< 8 bits per subpixel */
	IA_CSS_STREAM_FORMAT_YUV420_8,  /**< 8 bits per subpixel */
	IA_CSS_STREAM_FORMAT_YUV420_10, /**< 10 bits per subpixel */
	IA_CSS_STREAM_FORMAT_YUV422_8,  /**< UYVY..UVYV, 8 bits per subpixel */
	IA_CSS_STREAM_FORMAT_YUV422_10, /**< UYVY..UVYV, 10 bits per subpixel */
	IA_CSS_STREAM_FORMAT_RGB_444,  /**< BGR..BGR, 4 bits per subpixel */
	IA_CSS_STREAM_FORMAT_RGB_555,  /**< BGR..BGR, 5 bits per subpixel */
	IA_CSS_STREAM_FORMAT_RGB_565,  /**< BGR..BGR, 5 bits B and R, 6 bits G */
	IA_CSS_STREAM_FORMAT_RGB_666,  /**< BGR..BGR, 6 bits per subpixel */
	IA_CSS_STREAM_FORMAT_RGB_888,  /**< BGR..BGR, 8 bits per subpixel */
	IA_CSS_STREAM_FORMAT_RAW_6,    /**< RAW data, 6 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_7,    /**< RAW data, 7 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_8,    /**< RAW data, 8 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_10,   /**< RAW data, 10 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_12,   /**< RAW data, 12 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_14,   /**< RAW data, 14 bits per pixel */
	IA_CSS_STREAM_FORMAT_RAW_16,   /**< RAW data, 16 bits per pixel */
	IA_CSS_STREAM_FORMAT_BINARY_8, /**< Binary byte stream. */
};

/** For RAW input, the bayer order needs to be specified separately. There
 *  are 4 possible orders. The name is constructed by taking the first two
 *  colors on the first line and the first two colors from the second line.
 */
enum ia_css_bayer_order {
	IA_CSS_BAYER_ORDER_GRBG, /**< GRGRGRGRGR .. BGBGBGBGBG */
	IA_CSS_BAYER_ORDER_RGGB, /**< RGRGRGRGRG .. GBGBGBGBGB */
	IA_CSS_BAYER_ORDER_BGGR, /**< BGBGBGBGBG .. GRGRGRGRGR */
	IA_CSS_BAYER_ORDER_GBRG, /**< GBGBGBGBGB .. RGRGRGRGRG */
};
#define IA_CSS_BAYER_ORDER_NUM (IA_CSS_BAYER_ORDER_GBRG + 1)

/** Frame formats, some of these come from fourcc.org, others are
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
enum ia_css_frame_format {
	IA_CSS_FRAME_FORMAT_NV11,       /**< 12 bit YUV 411, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV12,       /**< 12 bit YUV 420, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV16,       /**< 16 bit YUV 422, Y, UV plane */
	IA_CSS_FRAME_FORMAT_NV21,       /**< 12 bit YUV 420, Y, VU plane */
	IA_CSS_FRAME_FORMAT_NV61,       /**< 16 bit YUV 422, Y, VU plane */
	IA_CSS_FRAME_FORMAT_YV12,       /**< 12 bit YUV 420, Y, V, U plane */
	IA_CSS_FRAME_FORMAT_YV16,       /**< 16 bit YUV 422, Y, V, U plane */
	IA_CSS_FRAME_FORMAT_YUV420,     /**< 12 bit YUV 420, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV420_16,  /**< yuv420, 16 bits per subpixel */
	IA_CSS_FRAME_FORMAT_YUV422,     /**< 16 bit YUV 422, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV422_16,  /**< yuv422, 16 bits per subpixel */
	IA_CSS_FRAME_FORMAT_UYVY,       /**< 16 bit YUV 422, UYVY interleaved */
	IA_CSS_FRAME_FORMAT_YUYV,       /**< 16 bit YUV 422, YUYV interleaved */
	IA_CSS_FRAME_FORMAT_YUV444,     /**< 24 bit YUV 444, Y, U, V plane */
	IA_CSS_FRAME_FORMAT_YUV_LINE,   /**< Internal format, 2 y lines followed
					     by a uvinterleaved line */
	IA_CSS_FRAME_FORMAT_RAW,	/**< RAW, 1 plane */
	IA_CSS_FRAME_FORMAT_RGB565,     /**< 16 bit RGB, 1 plane. Each 3 sub
					     pixels are packed into one 16 bit
					     value, 5 bits for R, 6 bits for G
					     and 5 bits for B. */
	IA_CSS_FRAME_FORMAT_PLANAR_RGB888, /**< 24 bit RGB, 3 planes */
	IA_CSS_FRAME_FORMAT_RGBA888,	/**< 32 bit RGBA, 1 plane, A=Alpha
					     (alpha is unused) */
	IA_CSS_FRAME_FORMAT_QPLANE6, /**< Internal, for advanced ISP */
	IA_CSS_FRAME_FORMAT_BINARY_8,	/**< byte stream, used for jpeg. For
					     frames of this type, we set the
					     height to 1 and the width to the
					     number of allocated bytes. */
	IA_CSS_FRAME_FORMAT_MIPI,	/**< MIPI frame, 1 plane */     
};
/* This one is hardcoded because the ISP firmware requires it known at
 * compile time (preprocessor time in fact). */
#define IA_CSS_FRAME_FORMAT_NUM 21

/* We include acc_types.h here because it uses the frame_format enum above.
 * This needs to be fixed, we do not want to have #include statements halfway
 * a file.
 */
#include "ia_css_acc_types.h"

enum ia_css_tpg_mode {
	IA_CSS_TPG_MODE_RAMP,
	IA_CSS_TPG_MODE_CHECKERBOARD,
	IA_CSS_TPG_MODE_FRAME_BASED_COLOR
};

/** @brief Configure the test pattern generator.
 *
 * Configure the Test Pattern Generator, the way these values are used to
 * generate the pattern can be seen in the HRT extension for the test pattern
 * generator:
 * devices/test_pat_gen/hrt/include/test_pat_gen.h: hrt_calc_tpg_data().
 *
 * This interface is deprecated, it is not portable -> move to input system API
 *
@code
unsigned int test_pattern_value(unsigned int x, unsigned int y)
{
 unsigned int x_val, y_val;
 if (x_delta > 0) (x_val = (x << x_delta) & x_mask;
 else (x_val = (x >> -x_delta) & x_mask;
 if (y_delta > 0) (y_val = (y << y_delta) & y_mask;
 else (y_val = (y >> -y_delta) & x_mask;
 return (x_val + y_val) & xy_mask;
}
@endcode
 */
struct ia_css_tpg_config {
	enum ia_css_tpg_mode mode;
	unsigned int         x_mask;
	int                  x_delta;
	unsigned int         y_mask;
	int                  y_delta;
	unsigned int         xy_mask;
};

/**
 * PRBS configuration structure.
 *
 * Seed the for the Pseudo Random Bit Sequence.
 *
 * This interface is deprecated, it is not portable -> move to input system API
 */
struct ia_css_prbs_config {
	int seed;
};

/** Generic resolution structure.
 */
struct ia_css_resolution {
	unsigned int width;  /**< Width */
	unsigned int height; /**< Height */
};

/** Enumeration of the physical input ports on the CSS hardware.
 *  There are 2 MIPI CSI-2 ports, a 1-lane port and a 4-lane port.
 */
enum ia_css_csi2_port {
	IA_CSS_CSI2_PORT_4LANE, /* 4-lane MIPI CSI-2 port */
	IA_CSS_CSI2_PORT_1LANE, /* 1-lane MIPI CSI-2 port */
	IA_CSS_CSI2_PORT_2LANE  /* 2-lane MIPI CSI-2 port */
};

/** The CSI2 interface supports 2 types of compression or can
 *  be run without compression.
 */
enum ia_css_csi2_compression_type {
	IA_CSS_CSI2_COMPRESSION_TYPE_NONE, /**< No compression */
	IA_CSS_CSI2_COMPRESSION_TYPE_1,    /**< Compression scheme 1 */
	IA_CSS_CSI2_COMPRESSION_TYPE_2     /**< Compression scheme 2 */
};

struct ia_css_csi2_compression {
	enum ia_css_csi2_compression_type type;
	/**< Compression used */
	unsigned int                      compressed_bits_per_pixel;
	/**< Compressed bits per pixel (only when compression is enabled) */
	unsigned int                      uncompressed_bits_per_pixel;
	/**< Uncompressed bits per pixel (only when compression is enabled) */
};

/** Input port structure.
 */
struct ia_css_input_port {
	enum ia_css_csi2_port port; /**< Physical CSI-2 port */
	unsigned int num_lanes; /**< Number of lanes used (4-lane port only) */
	unsigned int timeout;   /**< Timeout value */
	struct ia_css_csi2_compression compression; /**< Compression used */
};

/** Input stream description. This describes how input will flow into the
 *  CSS. This is used to program the CSS hardware.
 */
struct ia_css_stream_config {
	enum ia_css_input_mode    mode; /**< Input mode */
	union {
		struct ia_css_input_port  port; /**< Port, for sensor only. */
		struct ia_css_tpg_config  tpg;  /**< TPG configuration */
		struct ia_css_prbs_config prbs; /**< PRBS configuration */
	} source; /**< Source of input data */
	unsigned int              channel_id; /**< Channel on which input data
						   will arrive */
	struct ia_css_resolution  input_res; /**< Resolution of input data */
	struct ia_css_resolution  effective_res; /**< Resolution of input data */
	enum ia_css_stream_format format; /**< Format of input stream */
	enum ia_css_bayer_order bayer_order; /**< Bayer order for RAW streams */
	unsigned int sensor_binning_factor; /**< Binning factor used by sensor
					         to produce image data. This is
						 used for shading correction. */
	bool two_pixels_per_clock; /**< Enable/disable 2 pixels per clock */
	bool online; /**< offline will activate RAW copy on SP, use this for
		          continuous capture. */
	bool continuous; /**< Use SP copy feature to continuously capture frames
			      to system memory and run pipes in offline mode */
	int32_t flash_gpio_pin; /**< pin on which the flash is connected, -1 for no flash */
};

struct ia_css_stream;
struct ia_css_pipe;

/** Interrupt request type.
 *  When the CSS hardware generates an interrupt, a function in this API
 *  needs to be called to retrieve information about the interrupt.
 *  This interrupt type is part of this information and indicates what
 *  type of information the interrupt signals.
 *
 *  Note that one interrupt can carry multiple interrupt types. For
 *  example: the online video ISP will generate only 2 interrupts, one to
 *  signal that the statistics (3a and DIS) are ready and one to signal
 *  that all output frames are done (output and viewfinder).
 *
 * DEPRECATED, this interface is not portable it should only define user
 * (SW) interrupts
 */
enum ia_css_irq_info {
	IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR            = 1 << 0,
	/**< the css receiver has encountered an error */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_FIFO_OVERFLOW    = 1 << 1,
	/**< the FIFO in the csi receiver has overflown */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_SOF              = 1 << 2,
	/**< the css receiver received the start of frame */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_EOF              = 1 << 3,
	/**< the css receiver received the end of frame */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_SOL              = 1 << 4,
	/**< the css receiver received the start of line */
	IA_CSS_IRQ_INFO_EVENTS_READY                  = 1 << 5,
	/**< One or more events are available in the event queue */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_EOL              = 1 << 6,
	/**< the css receiver received the end of line */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_SIDEBAND_CHANGED = 1 << 7,
	/**< the css receiver received a change in side band signals */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_0      = 1 << 8,
	/**< generic short packets (0) */
	IA_CSS_IRQ_INFO_CSS_RECEIVER_GEN_SHORT_1      = 1 << 9,
	/**< generic short packets (1) */
	IA_CSS_IRQ_INFO_IF_PRIM_ERROR                 = 1 << 10,
	/**< the primary input formatter (A) has encountered an error */
	IA_CSS_IRQ_INFO_IF_PRIM_B_ERROR               = 1 << 11,
	/**< the primary input formatter (B) has encountered an error */
	IA_CSS_IRQ_INFO_IF_SEC_ERROR                  = 1 << 12,
	/**< the secondary input formatter has encountered an error */
	IA_CSS_IRQ_INFO_STREAM_TO_MEM_ERROR           = 1 << 13,
	/**< the stream-to-memory device has encountered an error */
	IA_CSS_IRQ_INFO_SW_0                          = 1 << 14,
	/**< software interrupt 0 */
	IA_CSS_IRQ_INFO_SW_1                          = 1 << 15,
	/**< software interrupt 1 */
	IA_CSS_IRQ_INFO_SW_2                          = 1 << 16,
	/**< software interrupt 2 */
	IA_CSS_IRQ_INFO_ISP_BINARY_STATISTICS_READY   = 1 << 17,
	/**< ISP binary statistics are ready */
	IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR            = 1 << 18,
	/**< the input system in in error */
	IA_CSS_IRQ_INFO_IF_ERROR                      = 1 << 19,
	/**< the input formatter in in error */
	IA_CSS_IRQ_INFO_DMA_ERROR                     = 1 << 20,
	/**< the dma in in error */
};

/** CSS receiver error types. Whenever the CSS receiver has encountered
 *  an error, this enumeration is used to indicate which errors have occurred.
 *
 *  Note that multiple error flags can be enabled at once and that this is in
 *  fact common (whenever an error occurs, it usually results in multiple
 *  errors).
 *
 * DEPRECATED: This interface is not portable, different systems have
 * different receiver types, or possibly none in case of tests systems.
 */
enum ia_css_rx_irq_info {
	IA_CSS_RX_IRQ_INFO_BUFFER_OVERRUN   = 1U << 0, /**< buffer overrun */
	IA_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE = 1U << 1, /**< entering sleep mode */
	IA_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE  = 1U << 2, /**< exited sleep mode */
	IA_CSS_RX_IRQ_INFO_ECC_CORRECTED    = 1U << 3, /**< ECC corrected */
	IA_CSS_RX_IRQ_INFO_ERR_SOT          = 1U << 4,
						/**< Start of transmission */
	IA_CSS_RX_IRQ_INFO_ERR_SOT_SYNC     = 1U << 5, /**< SOT sync (??) */
	IA_CSS_RX_IRQ_INFO_ERR_CONTROL      = 1U << 6, /**< Control (??) */
	IA_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE   = 1U << 7, /**< Double ECC */
	IA_CSS_RX_IRQ_INFO_ERR_CRC          = 1U << 8, /**< CRC error */
	IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID   = 1U << 9, /**< Unknown ID */
	IA_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC   = 1U << 10,/**< Frame sync error */
	IA_CSS_RX_IRQ_INFO_ERR_FRAME_DATA   = 1U << 11,/**< Frame data error */
	IA_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT = 1U << 12,/**< Timeout occurred */
	IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC  = 1U << 13,/**< Unknown escape seq. */
	IA_CSS_RX_IRQ_INFO_ERR_LINE_SYNC    = 1U << 14,/**< Line Sync error */
	IA_CSS_RX_IRQ_INFO_INIT_TIMEOUT     = 1U << 15,
};

/** Errors, these values are used as the return value for most
 *  functions in this API.
 */
enum ia_css_err {
	IA_CSS_SUCCESS,
	IA_CSS_ERR_INTERNAL_ERROR,
	IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY,
	IA_CSS_ERR_INVALID_ARGUMENTS,
	IA_CSS_ERR_SYSTEM_NOT_IDLE,
	IA_CSS_ERR_MODE_HAS_NO_VIEWFINDER,
	IA_CSS_ERR_QUEUE_IS_FULL,
	IA_CSS_ERR_QUEUE_IS_EMPTY,
	IA_CSS_ERR_RESOURCE_NOT_AVAILABLE,
	IA_CSS_ERR_RESOURCE_LIST_TO_SMALL,
	IA_CSS_ERR_RESOURCE_ITEMS_STILL_ALLOCATED,
	IA_CSS_ERR_RESOURCE_EXHAUSTED,
	IA_CSS_ERR_RESOURCE_ALREADY_ALLOCATED
};

/** Frame plane structure. This describes one plane in an image
 *  frame buffer.
 */
struct ia_css_frame_plane {
	unsigned int height; /**< height of a plane in lines */
	unsigned int width;  /**< width of a line, in DMA elements, note that
				  for RGB565 the three subpixels are stored in
				  one element. For all other formats this is
				  the number of subpixels per line. */
	unsigned int stride; /**< stride of a line in bytes */
	unsigned int offset; /**< offset in bytes to start of frame data.
				  offset is wrt data field in ia_css_frame */
};

/** Binary "plane". This is used to story binary streams such as jpeg
 *  images. This is not actually a real plane.
 */
struct ia_css_frame_binary_plane {
	unsigned int		  size; /**< number of bytes in the stream */
	struct ia_css_frame_plane data; /**< plane */
};

/** Container for planar YUV frames. This contains 3 planes.
 */
struct ia_css_frame_yuv_planes {
	struct ia_css_frame_plane y; /**< Y plane */
	struct ia_css_frame_plane u; /**< U plane */
	struct ia_css_frame_plane v; /**< V plane */
};

/** Container for semi-planar YUV frames.
  */
struct ia_css_frame_nv_planes {
	struct ia_css_frame_plane y;  /**< Y plane */
	struct ia_css_frame_plane uv; /**< UV plane */
};

/** Container for planar RGB frames. Each color has its own plane.
 */
struct ia_css_frame_rgb_planes {
	struct ia_css_frame_plane r; /**< Red plane */
	struct ia_css_frame_plane g; /**< Green plane */
	struct ia_css_frame_plane b; /**< Blue plane */
};

/** Container for 6-plane frames. These frames are used internally
 *  in the advanced ISP only.
 */
struct ia_css_frame_plane6_planes {
	struct ia_css_frame_plane r;	  /**< Red plane */
	struct ia_css_frame_plane r_at_b; /**< Red at blue plane */
	struct ia_css_frame_plane gr;	  /**< Red-green plane */
	struct ia_css_frame_plane gb;	  /**< Blue-green plane */
	struct ia_css_frame_plane b;	  /**< Blue plane */
	struct ia_css_frame_plane b_at_r; /**< Blue at red plane */
};

/** Frame info struct. This describes the contents of an image frame buffer.
  */
struct ia_css_frame_info {
	struct ia_css_resolution res; /**< Frame resolution (valid data) */
	unsigned int padded_width; /**< stride of line in memory (in pixels) */
	enum ia_css_frame_format format; /**< format of the frame data */
	unsigned int raw_bit_depth; /**< number of valid bits per pixel,
					 only valid for RAW bayer frames */
	enum ia_css_bayer_order raw_bayer_order; /**< bayer order, only valid
						      for RAW bayer frames */
};

/* Temporary hack, hivecc fails to properly compile if this struct is
 * included. */
#ifndef __HIVECC__

/**
 * Pipe configuration structure.
 */
struct ia_css_pipe_config {
	enum ia_css_pipe_mode mode;
	struct ia_css_resolution bin_out_res;
	/**< binning, used in continuous capture */
	struct ia_css_resolution bayer_ds_out_res;
	/**< bayer down scaling */
	struct ia_css_resolution dvs_crop_out_res;
	/**< dvs crop, video only, not in use yet. Use dvs_envelope below. */
	struct ia_css_frame_info output_info;
	/**< output of YUV scaling */
	struct ia_css_frame_info vf_output_info;
	/**< output of VF YUV scaling */
	struct ia_css_fw_info *acc_extension;
	/**< Pipeline extension accelerator */
	struct ia_css_fw_info **acc_stages;
	/**< Standalone accelerator stages */
	uint32_t num_acc_stages;
	/**< Number of standalone accelerator stages */
	struct ia_css_capture_config default_capture_config;
	/**< Default capture config for initial capture pipe configuration. */
	struct ia_css_resolution dvs_envelope; /**< temporary */
};
#else
struct ia_css_pipe_config;
#endif

enum ia_css_frame_flash_state {
	IA_CSS_FRAME_FLASH_STATE_NONE,
	IA_CSS_FRAME_FLASH_STATE_PARTIAL,
	IA_CSS_FRAME_FLASH_STATE_FULL
};

/** Frame structure. This structure describes an image buffer or frame.
 *  This is the main structure used for all input and output images.
 */
struct ia_css_frame {
	struct ia_css_frame_info info; /**< info struct describing the frame */
	ia_css_ptr   data;	       /**< pointer to start of image data */
	unsigned int data_bytes;       /**< size of image data in bytes */
	/* LA: move this to ia_css_buffer */
	/*
	 * -1 if data address is static during life time of pipeline
	 * >=0 if data address can change per pipeline/frame iteration
	 *     index to dynamic data: ia_css_frame_in, ia_css_frame_out
	 *                            ia_css_frame_out_vf
	 */
	int dynamic_data_index;
	enum ia_css_frame_flash_state flash_state;
	unsigned int exp_id; /**< exposure id, only valid for continuous
				capture cases */
	bool valid; /**< First video output frame is not valid */
	bool contiguous; /**< memory is allocated physically contiguously */
	union {
		unsigned int	_initialisation_dummy;
		struct ia_css_frame_plane raw;
		struct ia_css_frame_plane rgb;
		struct ia_css_frame_rgb_planes planar_rgb;
		struct ia_css_frame_plane yuyv;
		struct ia_css_frame_yuv_planes yuv;
		struct ia_css_frame_nv_planes nv;
		struct ia_css_frame_plane6_planes plane6;
		struct ia_css_frame_binary_plane binary;
	} planes; /**< frame planes, select the right one based on
		       info.format */
};

/** CSS firmware package structure.
 */
struct ia_css_fw {
	void	    *data;  /**< pointer to the firmware data */
	unsigned int bytes; /**< length in bytes of firmware data */
};

/** Structure that holds 3A statistics in the ISP internal
 * format. Use ia_css_get_3a_statistics() to translate
 * this to the format used on the host (3A library).
 * */
struct ia_css_isp_3a_statistics {
	union {
		struct {
			ia_css_ptr s3a_tbl;
		} dmem;
		struct {
			ia_css_ptr s3a_tbl_hi;
			ia_css_ptr s3a_tbl_lo;
		} vmem;
	} data;
	struct {
		ia_css_ptr rgby_tbl;
	} data_hmem;
};

/** Structure that holds DVS statistics in the ISP internal
 * format. Use ia_css_get_dvs_statistics() to translate
 * this to the format used on the host (DVS engine).
 * */
struct ia_css_isp_dvs_statistics {
	ia_css_ptr hor_proj;
	ia_css_ptr ver_proj;
};

struct ia_css_properties {
	int  gdc_coord_one;
	bool l1_base_is_index; /**< Indicate whether the L1 page base
				    is a page index or a byte address. */
	enum ia_css_vamem_type vamem_type;
};

/** Enumeration of buffer types. Buffers can be queued and de-queued
 *  to hand them over between IA and ISP.
 */
enum ia_css_buffer_type {
	IA_CSS_BUFFER_TYPE_3A_STATISTICS,
	IA_CSS_BUFFER_TYPE_DIS_STATISTICS,
	IA_CSS_BUFFER_TYPE_INPUT_FRAME,
	IA_CSS_BUFFER_TYPE_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_CUSTOM_INPUT,
	IA_CSS_BUFFER_TYPE_CUSTOM_OUTPUT,
	IA_CSS_BUFFER_TYPE_PARAMETER_SET,
};
#define IA_CSS_BUFFER_TYPE_NUM (IA_CSS_BUFFER_TYPE_PARAMETER_SET + 1)

/** Pipe info, this struct describes properties of a pipe after it's stream has
 * been created.
 */
struct ia_css_pipe_info {
	struct ia_css_frame_info output_info;
	/**< Info about output resolution. This contains the stride which
	     should be used for memory allocation. */
	struct ia_css_frame_info vf_output_info;
	/**< Info about viewfinder output resolution (optional). This contains
	     the stride that should be used for memory allocation. */
	struct ia_css_frame_info raw_output_info;
	/**< Raw output resolution. This indicates the resolution of the
	     RAW bayer output for pipes that support this. Currently, only the
	     still capture pipes support this feature. When this resolution is
	     smaller than the input resolution, cropping will be performed by
	     the ISP. The first cropping that will be performed is on the upper
	     left corner where we crop 8 lines and 8 columns to remove the
	     pixels normally used to initialize the ISP filters.
	     This is why the raw output resolution should normally be set to
	     the input resolution - 8x8. */
	struct ia_css_grid_info  grid_info;
	/**< After register an image pipe, this field will contain the grid
	     info for 3A and DVS. */
};

/** Stream info, this struct describes properties of a stream after it has been
 *  created.
 */
struct ia_css_stream_info {
	struct ia_css_resolution raw_info;
	/**< Info about raw buffer resolution. Mainly for continuous capture */
	struct ia_css_resolution effective_info;
	/**< Info about effective input buffer resolution. */
};

/** Memory allocation attributes, for use in ia_css_css_mem_env. */
enum ia_css_mem_attr {
	IA_CSS_MEM_ATTR_CACHED = 1 << 0,
	IA_CSS_MEM_ATTR_ZEROED = 1 << 1,
	IA_CSS_MEM_ATTR_PAGEALIGN = 1 << 2,
	IA_CSS_MEM_ATTR_CONTIGUOUS = 1 << 3,
};

struct ia_css_acc_fw;

/** Environment with function pointers for local IA memory allocation.
 *  This provides the CSS code with environment specific functionality
 *  for memory allocation of small local buffers such as local data structures.
 *  This is never expected to allocate more than one page of memory (4K bytes).
 */
struct ia_css_cpu_mem_env {
	void *(*alloc)(size_t bytes, bool zero_mem);
	/**< Allocation function with boolean argument to indicate whether
	     the allocated memory should be zeroed out or not. */
	void (*free)(void *ptr); /**< Corresponding free function. */
	void (*flush) (struct ia_css_acc_fw *fw);
	/**< Flush function to flush the cache for given accelerator. */
};

/** Environment with function pointers for allocation of memory for the CSS.
 *  The CSS uses its own MMU which has its own set of page tables. These
 *  functions are expected to use and/or update those page tables.
 *  This type of memory allocation is expected to be used for large buffers
 *  for images and statistics.
 *  ISP pointers are always 32 bits whereas IA pointer widths will depend
 *  on the platform.
 *  Attributes can be a combination (OR'ed) of ia_css_mem_attr values.
 */
struct ia_css_css_mem_env {
	ia_css_ptr (*alloc)(size_t bytes, uint32_t attributes);
	/**< Allocate memory, cached or uncached, zeroed out or not. */
	void     (*free)(ia_css_ptr ptr);
	/**< Free ISP shared memory. */
	int      (*load)(ia_css_ptr ptr, void *data, size_t bytes);
	/**< Load from ISP shared memory. This function is necessary because
	     the IA MMU does not share page tables with the ISP MMU. This means
	     that the IA needs to do the virtual-to-physical address
	     translation in software. This function performs this translation.*/
	int      (*store)(ia_css_ptr ptr, const void *data, size_t bytes);
	/**< Same as the above load function but then to write data into ISP
	     shared memory. */
	int      (*set)(ia_css_ptr ptr, int c, size_t bytes);
	/**< Set an ISP shared memory region to a particular value. Each byte
	     in this region will be set to this value. In most cases this is
	     used to zero-out memory sections in which case the argument c
	     would have the value zero. */
	ia_css_ptr (*mmap)(const void *ptr, const size_t size,
			   uint16_t attribute, void *context);
	/**< Map an pre-allocated memory region to an address. */
};

/** Environment with function pointers to access the CSS hardware. This includes
 *  registers and local memories.
 */
struct ia_css_hw_access_env {
	void     (*store_8)(hrt_address addr, uint8_t data);
	/**< Store an 8 bit value into an address in the CSS HW address space.
	     The address must be an 8 bit aligned address. */
	void     (*store_16)(hrt_address addr, uint16_t data);
	/**< Store a 16 bit value into an address in the CSS HW address space.
	     The address must be a 16 bit aligned address. */
	void     (*store_32)(hrt_address addr, uint32_t data);
	/**< Store a 32 bit value into an address in the CSS HW address space.
	     The address must be a 32 bit aligned address. */
	uint8_t (*load_8)(hrt_address addr);
	/**< Load an 8 bit value from an address in the CSS HW address
	     space. The address must be an 8 bit aligned address. */
	uint16_t (*load_16)(hrt_address addr);
	/**< Load a 16 bit value from an address in the CSS HW address
	     space. The address must be a 16 bit aligned address. */
	uint32_t (*load_32)(hrt_address addr);
	/**< Load a 32 bit value from an address in the CSS HW address
	     space. The address must be a 32 bit aligned address. */
	void     (*store)(hrt_address addr, const void *data, uint32_t bytes);
	/**< Store a number of bytes into a byte-aligned address in the CSS HW
	     address space. */
	void     (*load)(hrt_address addr, void *data, uint32_t bytes);
	/**< Load a number of bytes from a byte-aligned address in the CSS HW
	     address space. */
};

/** Environment with function pointers to print error and debug messages.
 */
struct ia_css_print_env {
	int (*debug_print)(const char *fmt, va_list args);
	/**< Print a debug message. */
	int (*error_print)(const char *fmt, va_list args);
	/**< Print an error message.*/
};

/** Environment structure. This includes function pointers to access several
 *  features provided by the environment in which the CSS API is used.
 *  This is used to run the camera IP in multiple platforms such as Linux,
 *  Windows and several simulation environments.
 */
struct ia_css_env {
	struct ia_css_cpu_mem_env   cpu_mem_env;   /**< local malloc and free. */
	struct ia_css_css_mem_env   css_mem_env;   /**< CSS/ISP buffer alloc/free */
	struct ia_css_hw_access_env hw_access_env; /**< CSS HW access functions */
	struct ia_css_print_env     print_env;     /**< Message printing env. */
};

/** Buffer structure. This is a container structure that enables content
 *  independent buffer queues and access functions.
 */
struct ia_css_buffer {
	enum ia_css_buffer_type type; /**< Buffer type. */
	union {
		struct ia_css_isp_3a_statistics  *stats_3a; /**< 3A statistics & optionally RGBY statistics. */
		struct ia_css_isp_dvs_statistics *stats_dvs;/**< DVS statistics. */
		struct ia_css_frame	         *frame;    /**< Frame buffer. */
		struct ia_css_acc_param          *custom_data; /**< Custom buffer. */
	} data; /**< Buffer data pointer. */
};

/** The event type, distinguishes the kind of events that
 * can are generated by the CSS system.
 */
enum ia_css_event_type {
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE    = 1 << 0,
	IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE = 1 << 1,
	IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE   = 1 << 2,
	IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE  = 1 << 3,
	IA_CSS_EVENT_TYPE_PIPELINE_DONE        = 1 << 4,
	IA_CSS_EVENT_TYPE_PORT_EOF	       = 1 << 5,
};
#define IA_CSS_EVENT_TYPE_NUM (IA_CSS_EVENT_TYPE_PORT_EOF + 1)
#define IA_CSS_EVENT_TYPE_NONE 0
#define IA_CSS_EVENT_TYPE_ALL  0xFFFF

struct ia_css_event {
	struct ia_css_pipe    *pipe;
	enum ia_css_event_type type;
	uint8_t                port;
};

/** Interrupt info structure. This structure contains information about an
 *  interrupt. This needs to be used after an interrupt is received on the IA
 *  to perform the correct action.
 */
struct ia_css_irq {
	enum ia_css_irq_info type; /**< Interrupt type. */
	unsigned int sw_irq_0_val; /**< In case of SW interrupt 0, value. */
	unsigned int sw_irq_1_val; /**< In case of SW interrupt 1, value. */
	unsigned int sw_irq_2_val; /**< In case of SW interrupt 2, value. */
	struct ia_css_pipe *pipe;
	/**< The image pipe that generated the interrupt. */
};

/* ===== GENERIC ===== */

/** @brief Initialize the CSS API.
 * @param[in]	env		Environment, provides functions to access the
 *				environment in which the CSS code runs. This is
 *				used for host side memory access and message
 *				printing.
 * @param[in]	fw		Firmware package containing the firmware for all
 *				predefined ISP binaries.
 * @param[in]	l1_base         Base address (isp2300) or base index (isp2400)
 *                              of the L1 page table. This is a physical
 *                              address or index.
 * @return			Returns IA_CSS_ERR_INTERNAL_ERROR in case of any
 *				errors and IA_CSS_SUCCESS otherwise.
 *
 * This function initializes the API which includes allocating and initializing
 * internal data structures. This also interprets the firmware package. All
 * contents of this firmware package are copied into local data structures, so
 * this pointer could be freed after this function completes.
 */
enum ia_css_err ia_css_init(
	const struct ia_css_env *env,
	const struct ia_css_fw  *fw,
	uint32_t                 l1_base,
	enum ia_css_irq_type     irq_type);

/** @brief Un-initialize the CSS API.
 *
 * This function deallocates all memory that has been allocated by the CSS
 * API. After this function is called, no other CSS functions should be called
 * with the exception of ia_css_init which will re-initialize the CSS code.
 *
 * @return None
 */
void
ia_css_uninit(void);

/** @brief Suspend CSS API for power down.
 *
 * This function prepares the CSS API for a power down of the CSS hardware.
 * This will make sure the hardware is idle. After this function is called,
 * always call ia_css_resume before calling any other CSS functions.
 * This assumes that all buffers allocated in DDR will remain alive during
 * power down. If this is not the case, use ia_css_unit() followed by
 * ia_css_init() at power up.
 */
void
ia_css_suspend(void);

/** @brief Resume CSS API from power down
 *
 * After a power cycle, this function will bring the CSS API back into
 * a state where it can be started. This will re-initialize the hardware.
 * Call this function only after ia_css_suspend() has been called.
 */
void
ia_css_resume(void);

/** @brief Get hardware properties
 *
 * This function returns a number of hardware properties.
 */
void
ia_css_get_properties(struct ia_css_properties *properties);

/** @brief Obtain interrupt information.
 *
 * @param[out] info	Pointer to the interrupt info. The interrupt
 *			information wil be written to this info.
 * @return		If an error is encountered during the interrupt info
 *			and no interrupt could be translated successfully, this
 *			will return IA_CSS_INTERNAL_ERROR. Otherwise
 *			IA_CSS_SUCCESS.
 *
 * This function is expected to be executed after an interrupt has been sent
 * to the IA from the CSS. This function returns information about the interrupt
 * which is needed by the IA code to properly handle the interrupt. This
 * information includes the image pipe, buffer type etc.
 */
enum ia_css_err
ia_css_irq_translate(unsigned int *info);

/** @brief Get CSI receiver error info.
 *
 * @param[out] irq_bits	Pointer to the interrupt bits. The interrupt
 *			bits will be written this info.
 *			This will be the error bits that are enabled in the CSI
 *			receiver error register.
 * This function should be used whenever a CSI receiver error interrupt is
 * generated. It provides the detailed information (bits) on the exact error
 * that occurred.
 */
void
ia_css_rx_get_irq_info(unsigned int *irq_bits);

/** @brief Clear CSI receiver error info.
 *
 * @param[in] irq_bits	The bits that should be cleared from the CSI receiver
 *			interrupt bits register.
 *
 * This function should be called after ia_css_rx_get_irq_info has been called
 * and the error bits have been interpreted. It is advised to use the return
 * value of that function as the argument to this function to make sure no new
 * error bits get overwritten.
 */
void
ia_css_rx_clear_irq_info(unsigned int irq_bits);

/** @brief Enable or disable specific interrupts.
 *
 * @param[in] type	The interrupt type that will be enabled/disabled.
 * @param[in] enable	enable or disable.
 * @return		Returns IA_CSS_INTERNAL_ERROR if this interrupt
 *			type cannot be enabled/disabled which is true for
 *			CSS internal interrupts. Otherwise returns
 *			IA_CSS_SUCCESS.
 */
enum ia_css_err
ia_css_irq_enable(enum ia_css_irq_info type, bool enable);

/** @brief Invalidate the MMU internal cache.
 *
 * This function triggers an invalidation of the translate-look-aside
 * buffer (TLB) that's inside the CSS MMU. This function should be called
 * every time the page tables used by the MMU change.
 */
void
ia_css_mmu_invalidate_cache(void);

/**
 * create the internal structures and fill in the configuration data
 */
void ia_css_pipe_config_defaults(struct ia_css_pipe_config *pipe_config);

enum ia_css_err
ia_css_pipe_create(const struct ia_css_pipe_config *config,
		   struct ia_css_pipe **pipe);

enum ia_css_err
ia_css_pipe_destroy(struct ia_css_pipe *pipe);

enum ia_css_err
ia_css_pipe_get_info(const struct ia_css_pipe *pipe,
		     struct ia_css_pipe_info *pipe_info);

void ia_css_stream_config_defaults(struct ia_css_stream_config *stream_config);

/**
 * create the internal structures and fill in the configuration data and pipes
 */
enum ia_css_err
ia_css_stream_create(const struct ia_css_stream_config *stream_config,
					 int num_pipes,
					 struct ia_css_pipe *pipes[],
					 struct ia_css_stream **stream);

enum ia_css_err
ia_css_stream_destroy(struct ia_css_stream *stream);

enum ia_css_err
ia_css_stream_get_info(const struct ia_css_stream *stream,
		       struct ia_css_stream_info *stream_info);

enum ia_css_err
ia_css_stream_load(struct ia_css_stream *stream);

/** @brief Starts the stream.
 *
 * The dynamic data in
 * the buffers are not used and need to be queued with a seperate call
 * to ia_css_pipe_enqueue_buffer.
 * NOTE: this function will only send start event to corresponding
 * thread and will not start SP any more.
 */
enum ia_css_err
ia_css_stream_start(struct ia_css_stream *stream);

/** @brief Stop the stream.
 *
 * NOTE: this function will send stop event to pipes belong to this
 * stream but will not terminate threads.
 */

enum ia_css_err
ia_css_stream_stop(struct ia_css_stream *stream);

bool
ia_css_stream_has_stopped(struct ia_css_stream *stream);

enum ia_css_err
ia_css_stream_unload(struct ia_css_stream *stream);

enum ia_css_stream_format
ia_css_stream_get_format(const struct ia_css_stream *stream);

bool
ia_css_stream_get_two_pixels_per_clock(const struct ia_css_stream *stream);

/** @brief Queue a buffer for an image pipe.
 *
 * @param[in] pipe	The pipe that will own the buffer.
 * @param[in] buffer	Pointer to the buffer.
 *                      Note that the caller remains owner of the buffer
 *                      structure. Only the data pointer within it will
 *                      be passed into the internal queues.
 * @return		IA_CSS_INTERNAL_ERROR in case of unexpected errors,
 *			IA_CSS_SUCCESS otherwise.
 *
 * This function adds a buffer (which has a certain buffer type) to the queue
 * for this type. This queue is owned by the image pipe. After this function
 * completes successfully, the buffer is now owned by the image pipe and should
 * no longer be accessed by any other code until it gets dequeued. The image
 * pipe will dequeue buffers from this queue, use them and return them to the
 * host code via an interrupt. Buffers will be consumed in the same order they
 * get queued, but may be returned to the host out of order.
 */
enum ia_css_err
ia_css_pipe_enqueue_buffer(struct ia_css_pipe *pipe,
			   const struct ia_css_buffer *buffer);

/** @brief Dequeue a buffer from an image pipe.
 *
 * @param[in]    pipe	The pipeline that the buffer queue belongs to.
 * @param[inout] buffer The buffer is used to lookup the type which determines
 *                      which internal queue to use.
 *                      The resulting buffer pointer is written into the dta
 *                      field.
 * @return		IA_CSS_ERR_NO_BUFFER if the queue is empty or
 *			IA_CSS_SUCCESS otherwise.
 *
 * This function dequeues a buffer from a buffer queue. The queue is indicated
 * by the buffer type argument. This function can be called after an interrupt
 * has been generated that signalled that a new buffer was available and can
 * be used in a polling-like situation where the NO_BUFFER return value is used
 * to determine whether a buffer was available or not.
 */
enum ia_css_err
ia_css_pipe_dequeue_buffer(struct ia_css_pipe *pipe,
			   struct ia_css_buffer *buffer);

/** @brief Dequeue an event from the CSS system. An event consists of pipe
 * and event_id.
 *
 * @param[out]	event   Pointer to the event struct which will be filled by
 *                      this function if an event is available.
 * @return		IA_CSS_ERR_QUEUE_IS_EMPTY if no events are
 *			available or
 *			IA_CSS_SUCCESS otherwise.
 *
 * This function dequeues an event from an event queue. The queue is inbetween
 * the Host (i.e. the Atom processosr) and the CSS system. This function can be
 * called after an interrupt has been generated that signalled that a new event
 * was available and can be used in a polling-like situation where the NO_EVENT
 * return value is used to determine whether an event was available or not.
 */
enum ia_css_err
ia_css_dequeue_event(struct ia_css_event *event);

/** @brief Controls when the Event generator raises an IRQ to the Host.
 *
 * @param[in]	pipe	The pipe.
 * @param[in]	or_mask	Binary or of enum ia_css_event_irq_mask_type. Each event
			that is part of this mask will directly raise an IRQ to
			the Host when the event occurs in the CSS.
 * @param[in]	and_mask Binary or of enum ia_css_event_irq_mask_type. An event
			IRQ for the Host is only raised after all events have
			occurred at least once for all the active pipes. Events
			are remembered and don't need to occure at the same
			moment in time. There is no control over the order of
			these events. Once an IRQ has been raised all
			remembered events are reset.
 * @return		IA_CSS_SUCCESS.
 *
 Controls when the Event generator in the CSS raises an IRQ to the Host.
 The main purpose of this function is to reduce the amount of interrupts
 between the CSS and the Host. This will help saving power as it wakes up the
 Host less often. In case both or_mask and and_mask are
 IA_CSS_EVENT_TYPE_NONE for all pipes, no event IRQ's will be raised.
 Note that events are still queued and the Host can poll for them. The
 or_mask and and_mask may be be active at the same time\n
 \n
 Default values, for all pipe id's, after ia_css_init:\n
 or_mask = IA_CSS_EVENT_TYPE_ALL\n
 and_mask = IA_CSS_EVENT_TYPE_NONE\n
 \n
 Examples\n
 \code
 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_PREVIEW,
 IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE |
 IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE ,
 IA_CSS_EVENT_TYPE_NONE);
 \endcode
 The event generator will only raise an interrupt to the Host when there are
 3A or DIS statistics available from the preview pipe. It will not generate
 an interrupt for any other event of the preview pipe e.g when there is an
 output frame available.

 \code
 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_PREVIEW,
	IA_CSS_EVENT_TYPE_NONE,
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE |
	IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE );

 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_CAPTURE,
	IA_CSS_EVENT_TYPE_NONE,
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE );
 \endcode
 The event generator will only raise an interrupt to the Host when there is
 both a frame done and 3A event available from the preview pipe AND when there
 is a frame done available from the capture pipe. Note that these events
 may occur at different moments in time. Also the order of the events is not
 relevant.

 \code
 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_PREVIEW,
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE,
	IA_CSS_EVENT_TYPE_ALL );

 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_COPY,
	IA_CSS_EVENT_TYPE_NONE,
	IA_CSS_EVENT_TYPE_NONE );

 ia_css_pipe_set_irq_mask(IA_CSS_PIPE_ID_CAPTURE,
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE,
	IA_CSS_EVENT_TYPE_ALL );
 \endcode
 The event generator will only raise an interrupt to the Host when there is an
 output frame from the preview pipe OR an output frame from the capture pipe.
 All other events (3A, VF output, pipeline done) will not raise an interrupt
 to the Host. These events are not lost but always stored in the event queue.
 */
enum ia_css_err
ia_css_pipe_set_irq_mask(struct ia_css_pipe *pipe,
			 unsigned int or_mask,
			 unsigned int and_mask);

/** @brief Reads the current event IRQ mask from the CSS.
 *
 * @param[in]	The pipe.
 * @param[out]	or_mask	Current or_mask. The bits in this mask are a binary or
		of enum ia_css_event_irq_mask_type. Pointer may be NULL.
 * @param[out]	and_mask Current and_mask.The bits in this mask are a binary or
		of enum ia_css_event_irq_mask_type. Pointer may be NULL.
 * @return	IA_CSS_SUCCESS.
 *
 Reads the current event IRQ mask from the CSS. Reading returns the actual
 values as used by the SP and not any mirrored values stored at the Host.\n
\n
Precondition:\n
SP must be running.\n

*/
enum ia_css_err
ia_css_event_get_irq_mask(const struct ia_css_pipe *pipe,
			  unsigned int *or_mask,
			  unsigned int *and_mask);

/* ===== FRAMES ===== */

/** @brief Fill a frame with zeros
 *
 * @param	frame		The frame.
 *
 * Fill a frame with pixel values of zero
 */
void ia_css_frame_zero(struct ia_css_frame *frame);

/** @brief Allocate a CSS frame structure
 *
 * @param	frame		The allocated frame.
 * @param	width		The width (in pixels) of the frame.
 * @param	height		The height (in lines) of the frame.
 * @param	format		The frame format.
 * @param	stride		The padded stride, in pixels.
 * @param	raw_bit_depth	The raw bit depth, in bits.
 * @return			The error code.
 *
 * Allocate a CSS frame structure. The memory for the frame data will be
 * allocated in the CSS address space.
 */
enum ia_css_err
ia_css_frame_allocate(struct ia_css_frame **frame,
		      unsigned int width,
		      unsigned int height,
		      enum ia_css_frame_format format,
		      unsigned int stride,
		      unsigned int raw_bit_depth);

/** @brief Allocate a CSS frame structure using a frame info structure.
 *
 * @param	frame	The allocated frame.
 * @param[in]	info	The frame info structure.
 * @return		The error code.
 *
 * Allocate a frame using the resolution and format from a frame info struct.
 * This is a convenience function, implemented on top of
 * ia_css_frame_allocate().
 */
enum ia_css_err
ia_css_frame_allocate_from_info(struct ia_css_frame **frame,
				const struct ia_css_frame_info *info);
/** @brief Free a CSS frame structure.
 *
 * @param[in]	frame	Pointer to the frame.
 *
 * Free a CSS frame structure. This will free both the frame structure
 * and the pixel data pointer contained within the frame structure.
 */
void
ia_css_frame_free(struct ia_css_frame *frame);

/* ===== FPGA display frames ====== */

/** @brief Allocate a contiguous CSS frame structure
 *
 * @param	frame		The allocated frame.
 * @param	width		The width (in pixels) of the frame.
 * @param	height		The height (in lines) of the frame.
 * @param	format		The frame format.
 * @param	stride		The padded stride, in pixels.
 * @param	raw_bit_depth	The raw bit depth, in bits.
 * @return			The error code.
 *
 * Contiguous frame allocation, only for FPGA display driver which needs
 * physically contiguous memory.
 */
enum ia_css_err
ia_css_frame_allocate_contiguous(struct ia_css_frame **frame,
				 unsigned int width,
				 unsigned int height,
				 enum ia_css_frame_format format,
				 unsigned int stride,
				 unsigned int raw_bit_depth);

/** @brief Allocate a contiguous CSS frame from a frame info structure.
 *
 * @param	frame	The allocated frame.
 * @param[in]	info	The frame info structure.
 * @return		The error code.
 *
 * Allocate a frame using the resolution and format from a frame info struct.
 * This is a convenience function, implemented on top of
 * ia_css_frame_allocate_contiguous().
 * Only for FPGA display driver which needs physically contiguous memory.
 */
enum ia_css_err
ia_css_frame_allocate_contiguous_from_info(struct ia_css_frame **frame,
					  const struct ia_css_frame_info *info);

/** @brief Map an existing frame data pointer to a CSS frame.
 *
 * @param[in]	info		The frame info.
 * @param[in]	data		Pointer to the allocated frame data.
 * @param[in]	attribute	Attributes to be passed to mmgr_mmap.
 * @param[in]	context		Pointer to the a context to be passed to mmgr_mmap.
 * @return			The allocated frame structure.
 *
 * This function maps a pre-allocated pointer into a CSS frame. This can be
 * used when an upper software layer is responsible for allocating the frame
 * data and it wants to share that frame pointer with the CSS code.
 * This function will fill the CSS frame structure just like
 * ia_css_frame_allocate() does, but instead of allocating the memory, it will
 * map the pre-allocated memory into the CSS address space.
 */
enum ia_css_err
ia_css_frame_map(struct ia_css_frame **frame,
                 const struct ia_css_frame_info *info,
                 const void *data,
                 uint16_t attribute,
                 void *context);

/** @brief Unmap a CSS frame structure.
 *
 * @param[in]	frame	Pointer to the CSS frame.
 *
 * This function unmaps the frame data pointer within a CSS frame and
 * then frees the CSS frame structure. Use this for frame pointers created
 * using ia_css_frame_map().
 */
void
ia_css_frame_unmap(struct ia_css_frame *frame);

/** @brief Return max nr of continuous RAW frames.
 *
 * @return	Max nr of continuous RAW frames.
 *
 * Return the maximum nr of continuous RAW frames the system can support.
 */
enum ia_css_err
ia_css_stream_get_max_buffer_depth(struct ia_css_stream *stream, int *buffer_depth);

/** @brief Set nr of continuous RAW frames to use.
 *
 * @param 	num_frames	Number of frames.
 * @return	IA_CSS_SUCCESS or error code upon error.
 *
 * Set the number of continuous frames to use during continuous modes.
 */
enum ia_css_err
ia_css_stream_set_buffer_depth(struct ia_css_stream *stream, int buffer_depth);

/** @brief Get nr of continuous RAW frames to use.
 *
 * @return 	Number of frames to use.

 *
 * Get the currently set number of continuous frames
 * to use during continuous modes.
 */
enum ia_css_err
ia_css_stream_get_buffer_depth(struct ia_css_stream *stream, int *buffer_depth);

/* ===== CAPTURE ===== */

/** @brief Configure the continuous capture
 *
 * @param	num_captures	The number of RAW frames to be processed to
 *                              YUV. Setting this to -1 will make continuous
 *                              capture run until it is stopped.
 *                              This number will also be used to allocate RAW
 *                              buffers. To allow the viewfinder to also
 *                              keep operating, 2 extra buffers will always be
 *                              allocated.
 *                              If the offset is negative and the skip setting
 *                              is greater than 0, additional buffers may be
 *                              needed.
 * @param	skip		Skip N frames in between captures. This can be
 *                              used to select a slower capture frame rate than
 *                              the sensor output frame rate.
 * @param	offset		Start the RAW-to-YUV processing at RAW buffer
 *                              with this offset. This allows the user to
 *                              process RAW frames that were captured in the
 *                              past or future.
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 *  For example, to capture the current frame plus the 2 previous
 *  frames and 2 subsequent frames, you would call
 *  ia_css_stream_capture(5, 0, -2).
 */
enum ia_css_err
ia_css_stream_capture(struct ia_css_stream *stream,
			int num_captures,
			unsigned int skip,
			int offset);

/** @brief Specify which raw frame to tag based on exp_id found in frame info
 *
 * @param	exp_id	The exposure id of the raw frame to tag.
 *
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * This function allows the user to tag a raw frame based on the exposure id
 * found in the viewfinder frames' frame info.
 */
enum ia_css_err
ia_css_stream_capture_frame(struct ia_css_stream *stream,
			unsigned int exp_id);

/* ===== VIDEO ===== */

/** @brief Send streaming data into the css input FIFO
 *
 * @param	data	Pointer to the pixels to be send.
 * @param	width	Width of the input frame.
 * @param	height	Height of the input frame.
 *
 * Send streaming data into the css input FIFO. This is for testing purposes
 * only. This uses the channel ID and input format as set by the user with
 * the regular functions for this.
 * This function blocks until the entire frame has been written into the
 * input FIFO.
 */
void
ia_css_stream_send_input_frame(const struct ia_css_stream *stream,
			       const unsigned short *data,
			       unsigned int width,
			       unsigned int height);

/*
 * For higher flexibility the ia_css_stream_send_input_frame is replaced by
 * three seperate functions:
 * 1) ia_css_stream_start_input_frame
 * 2) ia_css_stream_send_input_line
 * 3) ia_css_stream_end_input_frame
 * In this way it is possible to stream multiple frames on different
 * channel ID's on a line basis. It will be possible to simulate
 * line-interleaved Stereo 3D muxed on 1 mipi port.
 * These 3 functions are for testing purpose only and can be used in
 * conjunction with ia_css_stream_send_input_frame
 */

/** @brief Start an input frame on the CSS input FIFO.
 *
 * @param[in]	channel_id		The channel id.
 * @param[in]	input_format		The input format.
 * @param[in]	two_pixels_per_clock	Use 2 pixels per clock.
 *
 * Starts the streaming to mipi frame by sending SoF for channel channel_id.
 * It will use the input_format and two_pixels_per_clock as provided by
 * the user.
 * For the "correct" use-case, input_format and two_pixels_per_clock must match
 * with the values as set by the user with the regular functions.
 * To simulate an error, the user can provide "incorrect" values for
 * input_format and/or two_pixels_per_clock.
 */
void
ia_css_stream_start_input_frame(const struct ia_css_stream *stream);

/** @brief Send a line of input data into the CSS input FIFO.
 *
 * @param[in]	channel_id		The channel id.
 * @param[in]	data	Array of the first line of image data.
 * @param	width	The width (in pixels) of the first line.
 * @param[in]	data2	Array of the second line of image data.
 * @param	width2	The width (in pixels) of the second line.
 *
 * Sends 1 frame line. Start with SoL followed by width bytes of data, followed
 * by width2 bytes of data2 and followed by and EoL
 * It will use the input_format and two_pixels_per_clock settings as provided
 * with the ia_css_stream_start_input_frame function call.
 *
 * This function blocks until the entire line has been written into the
 * input FIFO.
 */
void
ia_css_stream_send_input_line(const struct ia_css_stream *stream,
			      const unsigned short *data,
			      unsigned int width,
			      const unsigned short *data2,
			      unsigned int width2);


/** @brief End an input frame on the CSS input FIFO.
 *
 * @param[in]	channel_id	The channel id.
 *
 * Send the end-of-frame signal into the CSS input FIFO.
 */
void
ia_css_stream_end_input_frame(const struct ia_css_stream *stream);

/** @brief Test whether the ISP has started.
 *
 * @return	The ISP has started.
 *
 * Temporary function to poll whether the ISP has been started. Once it has,
 * the sensor can also be started. */
bool
ia_css_isp_has_started(void);

/** @brief Test whether the SP has initialized.
 *
 * @return	The SP has initialized.
 *
 * Temporary function to poll whether the SP has been initilized. Once it has,
 * we can enqueue buffers. */
bool
ia_css_sp_has_initialized(void);

/** @brief Test whether the SP has terminated.
 *
 * @return	The SP has terminated.
 *
 * Temporary function to poll whether the SP has been terminated. Once it has,
 * we can switch mode. */
bool
ia_css_sp_has_terminated(void);

/** @brief send a request flash command to SP
 *
 * Driver needs to call this function to send a flash request command
 * to SP, SP will be responsible for switching on/off the flash at proper
 * time. Due to the SP multi-threading environment, this request may have
 * one-frame delay, the driver needs to check the flashed flag in frame info
 * to determine which frame is being flashed.
 */
void
ia_css_stream_request_flash(struct ia_css_stream *stream);

/** @brief Configure a stream with filter coefficients.
 *
 * @param[in]	config	The set of filter coefficients.
 * @return		IA_CSS_SUCCESS or error code upon error.
 *
 * This function configures the filter coefficients for an image
 * stream. For image pipes that do not execute any ISP filters, this
 * function will have no effect.
 * It is safe to call this function while the image stream is running,
 * in fact this is the expected behavior most of the time. Proper
 * resource locking and double buffering is in place to allow for this.
 */
void
ia_css_stream_set_isp_config(struct ia_css_stream *stream,
			     const struct ia_css_isp_config *config);

/** @brief Get selected configuration settings
 */
void
ia_css_stream_get_isp_config(const struct ia_css_stream *stream,
			     struct ia_css_isp_config *config);

/* Copy DVS statistics from an ISP buffer to a host buffer.
 * This may include a translation step as well depending
 * on the ISP version.
 * Always use this function, never copy the buffer directly.
 */
void
ia_css_get_dvs_statistics(struct ia_css_dvs_statistics           *host_stats,
			  const struct ia_css_isp_dvs_statistics *isp_stats);

/* Copy DVS 2.0 statistics from an ISP buffer to a host buffer.
 * This may include a translation step as well depending
 * on the ISP version.
 * Always use this function, never copy the buffer directly.
 */
void
ia_css_get_dvs2_statistics(struct ia_css_dvs2_statistics           *host_stats,
			  const struct ia_css_isp_dvs_statistics *isp_stats);

/* Copy 3A statistics from an ISP buffer to a host buffer.
 * This may include a translation step as well depending
 * on the ISP version.
 * Always use this function, never copy the buffer directly.
 */
void
ia_css_get_3a_statistics(struct ia_css_3a_statistics           *host_stats,
			 const struct ia_css_isp_3a_statistics *isp_stats);

/* Convenience functions for alloc/free of certain datatypes */

/* Morphing table */
struct ia_css_morph_table *
ia_css_morph_table_allocate(unsigned int width, unsigned int height);

void
ia_css_morph_table_free(struct ia_css_morph_table *me);

/* Shading table */
void
ia_css_shading_table_free(struct ia_css_shading_table *table);

struct ia_css_shading_table *
ia_css_shading_table_alloc(unsigned int width,
			   unsigned int height);

struct ia_css_isp_3a_statistics *
ia_css_isp_3a_statistics_allocate(const struct ia_css_3a_grid_info *grid);

void
ia_css_isp_3a_statistics_free(struct ia_css_isp_3a_statistics *me);

struct ia_css_isp_dvs_statistics *
ia_css_isp_dvs_statistics_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_isp_dvs_statistics_free(struct ia_css_isp_dvs_statistics *me);

struct ia_css_isp_dvs_statistics *
ia_css_isp_dvs2_statistics_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_isp_dvs2_statistics_free(struct ia_css_isp_dvs_statistics *me);

struct ia_css_3a_statistics *
ia_css_3a_statistics_allocate(const struct ia_css_3a_grid_info *grid);

void
ia_css_3a_statistics_free(struct ia_css_3a_statistics *me);

struct ia_css_dvs_statistics *
ia_css_dvs_statistics_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_dvs_statistics_free(struct ia_css_dvs_statistics *me);

struct ia_css_dvs_coefficients *
ia_css_dvs_coefficients_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_dvs_coefficients_free(struct ia_css_dvs_coefficients *me);

struct ia_css_dvs2_statistics *
ia_css_dvs2_statistics_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_dvs2_statistics_free(struct ia_css_dvs2_statistics *me);

struct ia_css_dvs2_coefficients *
ia_css_dvs2_coefficients_allocate(const struct ia_css_dvs_grid_info *grid);

void
ia_css_dvs2_coefficients_free(struct ia_css_dvs2_coefficients *me);

/** @brief start SP hardware
 *
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * It will boot the SP hardware and start multi-threading infrastructure.
 * All threads will be started and blocked by semaphore. This function should
 * be called before any ia_css_stream_start().
 */
enum ia_css_err
ia_css_start_sp(void);


/** @brief stop SP hardware
 *
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * This function will terminate all threads and shut down SP. It should be
 * called after all ia_css_stream_stop().
 */
enum ia_css_err
ia_css_stop_sp(void);

#endif /* _IA_CSS_H_ */
