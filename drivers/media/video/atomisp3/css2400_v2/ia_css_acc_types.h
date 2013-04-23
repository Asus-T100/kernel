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

#ifndef _IA_CSS_ACC_TYPES_H_
#define _IA_CSS_ACC_TYPES_H_

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
#include <system_types.h>	/* HAS_IRQ_MAP_VERSION_# */

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>       /* memcpy() */
#else
#include <stdlib.h>             /* size_t */
#include <string.h>             /* memcpy() */
#include "math_support.h"		/* min(), max() */
#endif

#include "ia_css.h"
#include "ia_css_types.h"

/* Types for the acceleration API.
 * These should be moved to sh_css_internal.h once the old acceleration
 * argument handling has been completed.
 * After that, interpretation of these structures is no longer needed
 * in the kernel and HAL.
*/

/** Blob descriptor.
 * This structure describes an SP or ISP blob.
 * It describes the test, data and bss sections as well as position in a
 * firmware file.
 * For convenience, it contains dynamic data after loading.
 */
struct ia_css_blob_info {
	/**< Static blob data */
	uint32_t offset;		/**< Blob offset in fw file */
	uint32_t size;		/**< Size of blob */
	uint32_t prog_name_offset;  /**< offset wrt hdr in bytes */
	uint32_t text_source;	/**< Position of text in blob */
	uint32_t text_size;		/**< Size of text section */
	uint32_t icache_source;	/**< Position of icache in blob */
	uint32_t icache_size;		/**< Size of icache section */
	uint32_t data_source;	/**< Position of data in blob */
	uint32_t data_target;	/**< Start of data in SP dmem */
	uint32_t data_size;		/**< Size of text section */
	uint32_t bss_target;	/**< Start position of bss in SP dmem */
	uint32_t bss_size;		/**< Size of bss section */
	/**< Dynamic data filled by loader */
	const void  *text;		/**< Text section within fw */
	const void  *data;		/**< Sp data section */
};

/** Type of acceleration.
 */
enum ia_css_acc_type {
	IA_CSS_ACC_NONE,	/**< Normal binary */
	IA_CSS_ACC_OUTPUT,	/**< Accelerator stage on output frame */
	IA_CSS_ACC_VIEWFINDER,	/**< Accelerator stage on viewfinder frame */
	IA_CSS_ACC_STANDALONE,	/**< Stand-alone acceleration */
};

/** Firmware types.
 */
enum ia_css_fw_type {
	ia_css_sp_firmware,	/**< Firmware for the SP */
	ia_css_isp_firmware,	/**< Firmware for the ISP */
	ia_css_acc_firmware	/**< Firmware for accelrations */
};

#if defined(IS_ISP_2300_SYSTEM)
enum ia_css_isp_memories {
	IA_CSS_ISP_PMEM0 = 0,
	IA_CSS_ISP_DMEM0,
	IA_CSS_ISP_VMEM0,
	IA_CSS_ISP_VAMEM0,
	IA_CSS_ISP_VAMEM1,
	N_IA_CSS_ISP_MEMORIES
};

#define IA_CSS_NUM_ISP_MEMORIES 5

#elif defined(IS_ISP_2400_SYSTEM)
enum ia_css_isp_memories {
	IA_CSS_ISP_PMEM0 = 0,
	IA_CSS_ISP_DMEM0,
	IA_CSS_ISP_VMEM0,
	IA_CSS_ISP_VAMEM0,
	IA_CSS_ISP_VAMEM1,
	IA_CSS_ISP_VAMEM2,
	IA_CSS_ISP_HMEM0,
	N_IA_CSS_ISP_MEMORIES
};

#define IA_CSS_NUM_ISP_MEMORIES 7

#else
#error "ia_css_types.h:  SYSTEM must be one of {ISP_2300_SYSTEM, ISP_2400_SYSTEM}"
#endif

/** CSS data descriptor */
struct ia_css_data {
	ia_css_ptr address;
	uint32_t   size; /* Disabled if 0 */
};

struct ia_css_blob_descr;

/** Structure describing an ISP binary.
 * It describes the capabilities of a binary, like the maximum resolution,
 * support features, dma channels, uds features, etc.
 */
struct ia_css_binary_info {
	uint32_t		id; /* IA_CSS_BINARY_ID_* */
	uint32_t		mode;
	enum ia_css_acc_type	 type;
	const struct ia_css_blob_descr *blob;
	int32_t			num_output_formats;
	enum ia_css_frame_format output_formats[IA_CSS_FRAME_FORMAT_NUM];
	uint32_t		min_input_width;
	uint32_t		min_input_height;
	uint32_t		max_input_width;
	uint32_t		max_input_height;
	uint32_t		min_output_width;
	uint32_t		min_output_height;
	uint32_t		max_output_width;
	uint32_t		max_output_height;
	uint32_t		max_internal_width;
	uint32_t		max_internal_height;
	uint32_t		max_dvs_envelope_width;
	uint32_t		max_dvs_envelope_height;
	uint32_t		variable_resolution;
	uint32_t		variable_output_format;
	uint32_t		variable_vf_veceven;
	uint32_t		max_vf_log_downscale;
	uint32_t		top_cropping;
	uint32_t		left_cropping;
	uint32_t		s3atbl_use_dmem;
	int32_t			input;
	ia_css_ptr		xmem_addr;
	uint32_t		c_subsampling;
	uint32_t		output_num_chunks;
	uint32_t		num_stripes;
	uint32_t		pipelining;
	uint32_t		fixed_s3a_deci_log;
	uint32_t		isp_addresses; /* Address in ISP dmem */
	uint32_t		main_entry;    /* Address of entry fct */
	uint32_t		in_frame;  /* Address in ISP dmem */
	uint32_t		out_frame; /* Address in ISP dmem */
	uint32_t		in_data;  /* Address in ISP dmem */
	uint32_t		out_data; /* Address in ISP dmem */
	uint8_t			block_width;
	uint8_t			block_height;
	uint8_t			output_block_height;
	uint8_t			num_output_pins;
	uint32_t		dvs_in_block_width;
	uint32_t		dvs_in_block_height;
	struct ia_css_data	mem_initializers[IA_CSS_NUM_ISP_MEMORIES];
	uint32_t		sh_dma_cmd_ptr;     /* In ISP dmem */
	uint32_t		isp_pipe_version;
	struct {
		uint8_t	ctc;   /* enum sh_css_isp_memories */
		uint8_t	gamma; /* enum sh_css_isp_memories */
		uint8_t	xnr;   /* enum sh_css_isp_memories */
		uint8_t	r_gamma; /* enum sh_css_isp_memories */
		uint8_t	g_gamma; /* enum sh_css_isp_memories */
		uint8_t	b_gamma; /* enum sh_css_isp_memories */
		uint8_t	rgby; /* enum sh_css_isp_memories */
	} memories;
/* MW: Packing (related) bools in an integer ?? */
	struct {
		uint8_t	reduced_pipe;
		uint8_t	vf_veceven;
		uint8_t	dis;
		uint8_t	dvs_envelope;
		uint8_t	uds;
		uint8_t	dvs_6axis;
		uint8_t	block_output;
		uint8_t	streaming_dma;
		uint8_t	ds;
		uint8_t	fixed_bayer_ds;
		uint8_t	bayer_fir_6db;
		uint8_t	raw_binning;
		uint8_t	continuous;
		uint8_t	s3a;
		uint8_t	fpnr;
		uint8_t	sc;
		uint8_t	dis_crop;
		uint8_t	dp_2adjacent;
		uint8_t	macc;
		uint8_t	ss;
		uint8_t	output;
		uint8_t	ref_frame;
		uint8_t	tnr;
		uint8_t	xnr;
		uint8_t	raw;
		uint8_t	params;
		uint8_t	gamma;
		uint8_t	ctc;
		uint8_t	ca_gdc;
		uint8_t	isp_addresses;
		uint8_t	in_frame;
		uint8_t	out_frame;
		uint8_t	high_speed;
		uint8_t	input_chunking;
		/* uint8_t padding[2]; */
	} enable;
	struct {
/* DMA channel ID: [0,...,HIVE_ISP_NUM_DMA_CHANNELS> */
		uint8_t	crop_channel;
		uint8_t	fpntbl_channel;
		uint8_t	multi_channel;
		uint8_t	raw_out_channel;
		uint8_t	sctbl_channel;
		uint8_t	ref_y_channel;
		uint8_t	ref_c_channel;
		uint8_t	tnr_channel;
		uint8_t	tnr_out_channel;
		uint8_t	dvs_in_channel;
		uint8_t	dvs_coords_channel;
		uint8_t	output_channel;
		uint8_t	c_channel;
		uint8_t	vfout_channel;
		uint8_t	vfout_c_channel;
		uint8_t	claimed_by_isp;
		/* uint8_t padding[0]; */
		struct {
			uint8_t		channel;  /* Dma channel used */
			uint8_t		height;   /* Buffer height */
			uint16_t	stride;   /* Buffer stride */
		} raw;
	} dma;
	struct {
		uint16_t	bpp;
		uint16_t	use_bci;
		uint16_t	woix;
		uint16_t	woiy;
		uint16_t	extra_out_vecs;
		uint16_t	vectors_per_line_in;
		uint16_t	vectors_per_line_out;
		uint16_t	vectors_c_per_line_in;
		uint16_t	vectors_c_per_line_out;
		uint16_t	vmem_gdc_in_block_height_y;
		uint16_t	vmem_gdc_in_block_height_c;
		/* uint16_t padding; */
	} uds;
	uint32_t	blob_index;
	struct ia_css_binary_info *next;
};

/** Structure describing the SP binary.
 * It contains several address, either in ddr, sp_dmem or
 * the entry function in pmem.
 */
struct ia_css_sp_info {
	uint32_t init_dmem_data; /**< data sect config, stored to dmem */
	uint32_t per_frame_data; /**< Per frame data, stored to dmem */
	uint32_t group;		/**< Per pipeline data, loaded by dma */
	uint32_t output;		/**< SP output data, loaded by dmem */
	uint32_t host_sp_queue;	/**< Host <-> SP queues */
	uint32_t host_sp_com;/**< Host <-> SP commands */
	uint32_t isp_started;	/**< Polled from sensor thread, csim only */
	uint32_t sw_state;	/**< Polled from css */
	uint32_t host_sp_queues_initialized; /**< Polled from the SP */
	uint32_t sleep_mode;  /**< different mode to halt SP */
	uint32_t invalidate_tlb;		/**< inform SP to invalidate mmu TLB */
	uint32_t request_flash;	/**< inform SP to switch on flash for next frame */
	uint32_t stop_copy_preview;
	uint32_t debug_buffer_ddr_address;	/**< inform SP the address
	of DDR debug queue */
	uint32_t ddr_parameter_address; /**< acc param ddrptr, sp dmem */
	uint32_t ddr_parameter_size;    /**< acc param size, sp dmem */
	/* Entry functions */
	uint32_t sp_entry;	/**< The SP entry function */
};

/** Accelerator firmware information.
 */
struct ia_css_acc_info {
	uint32_t per_frame_data; /**< Dummy for now */
};

/** Firmware information.
 */
union ia_css_fw_union {
	struct ia_css_binary_info	isp; /**< ISP info */
	struct ia_css_sp_info		sp;  /**< SP info */
	struct ia_css_acc_info		acc; /**< Accelerator info */
};

/** Firmware information.
 */
struct ia_css_fw_info {
	size_t			header_size; /**< size of fw header */
	enum ia_css_fw_type	type; /**< FW type */
	union ia_css_fw_union	info; /**< Binary info */
	struct ia_css_blob_info blob; /**< Blob info */
	/* Dynamic part */
	struct ia_css_fw_info  *next;
	uint32_t                loaded;    /**< Firmware has been loaded */
	const uint8_t          *isp_code;  /**< ISP pointer to code */
	/**< Firmware handle between user space and kernel */
	uint32_t		handle;
	/**< Sections to copy from/to ISP */
	struct ia_css_data      mem_initializers[IA_CSS_NUM_ISP_MEMORIES];
	/**< Initializer for local ISP memories */
};

struct ia_css_blob_descr {
	const unsigned char  *blob;
	struct ia_css_fw_info header;
	const char	     *name;
};

struct ia_css_acc_fw;

/** Structure describing the SP binary of a stand-alone accelerator.
 */
 struct ia_css_acc_sp {
	void (*init) (struct ia_css_acc_fw *); /**< init for crun */
	uint32_t      sp_prog_name_offset; /**< program name offset wrt hdr
						in bytes */
	uint32_t      sp_blob_offset;	   /**< blob offset wrt hdr in bytes */
	void	     *entry;		   /**< Address of sp entry point */
	uint32_t *css_abort;	   /**< SP dmem abort flag */
	void	     *isp_code;		   /**< SP dmem address holding xmem
						address of isp code */
	struct ia_css_fw_info fw;	   /**< SP fw descriptor */
	const uint8_t *code;	   /**< ISP pointer of allocated
						SP code */
};

/** Acceleration firmware descriptor.
  * This descriptor descibes either SP code (stand-alone), or
  * ISP code (a separate pipeline stage).
  */
struct ia_css_acc_fw_hdr {
	enum ia_css_acc_type type;	/**< Type of accelerator */
	uint32_t	isp_prog_name_offset; /**< program name offset wrt
						   header in bytes */
	uint32_t	isp_blob_offset;      /**< blob offset wrt header
						   in bytes */
	uint32_t	isp_size;	      /**< Size of isp blob */
	const uint8_t  *isp_code;	      /**< ISP pointer to code */
	struct ia_css_acc_sp  sp;  /**< Standalone sp code */
	/**< Firmware handle between user space and kernel */
	uint32_t	handle;
	struct ia_css_data parameters; /**< Current SP parameters */
};

/** Firmware structure.
  * This contains the header and actual blobs.
  * For standalone, it contains SP and ISP blob.
  * For a pipeline stage accelerator, it contains ISP code only.
  * Since its members are variable size, their offsets are described in the
  * header and computed using the access macros below.
  */
struct ia_css_acc_fw {
	struct ia_css_acc_fw_hdr header; /**< firmware header */
	/*
	int8_t   isp_progname[];	  **< ISP program name
	int8_t   sp_progname[];	  **< SP program name, stand-alone only
	uint8_t sp_code[];  **< SP blob, stand-alone only
	uint8_t isp_code[]; **< ISP blob
	*/
};

/* Access macros for firmware */
#define IA_CSS_ACC_OFFSET(t, f, n) ((t)((uint8_t *)(f)+(f->header.n)))
#define IA_CSS_ACC_SP_PROG_NAME(f) IA_CSS_ACC_OFFSET(const char *, f, \
						 sp.sp_prog_name_offset)
#define IA_CSS_ACC_ISP_PROG_NAME(f) IA_CSS_ACC_OFFSET(const char *, f, \
						 isp_prog_name_offset)
#define IA_CSS_ACC_SP_CODE(f)      IA_CSS_ACC_OFFSET(uint8_t *, f, \
						 sp.sp_blob_offset)
#define IA_CSS_ACC_SP_DATA(f)      (IA_CSS_ACC_SP_CODE(f) + \
					(f)->header.sp.fw.blob.data_source)
#define IA_CSS_ACC_ISP_CODE(f)     IA_CSS_ACC_OFFSET(uint8_t*, f,\
						 isp_blob_offset)
#define IA_CSS_ACC_ISP_SIZE(f)     ((f)->header.isp_size)

/* Binary name follows header immediately */
#define IA_CSS_EXT_ISP_PROG_NAME(f) ((const char *)(f)+sizeof(*f))

/** Structure to encapsulate required arguments for
 * initialization of SP DMEM using the SP itself
 * This is exported for accelerators implementing their own SP code.
 */
struct ia_css_sp_init_dmem_cfg {
	uint32_t   done;	      /**< Init has been done */
	ia_css_ptr ddr_data_addr;  /**< data segment address in ddr  */
	ia_css_ptr dmem_data_addr; /**< data segment address in dmem */
	ia_css_ptr dmem_bss_addr;  /**< bss segment address in dmem  */
	uint32_t   data_size;      /**< data segment size            */
	uint32_t   bss_size;       /**< bss segment size             */
};

enum ia_css_sp_sleep_mode {
	SP_DISABLE_SLEEP_MODE = 0,
	SP_SLEEP_AFTER_FRAME = 1 << 0,
	SP_SLEEP_AFTER_IRQ = 1 << 1
};

enum ia_css_sp_sw_state {
	SP_SW_STATE_NULL = 0,
	SP_SW_INITIALIZED,
	SP_SW_TERMINATED
};

#endif /* _IA_CSS_TYPES_H_ */
