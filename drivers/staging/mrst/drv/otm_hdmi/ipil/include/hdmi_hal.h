/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * GENERAL ASSUMPTIONS / REQUIREMENTS
 *
 * 1. HAL entries mentioned in this document are intended to be simple wrappers
 *    on top of register reads and writes. They should not keep any state
 *    and should not implement any policies. It is completely up to higher
 *    levels when and how to use these entries. The only safety checks HAL
 *    entries should do is for NULL pointers and (in very few cases) for
 *    correctness of values supplied for register writes (range and alignment
 *    checks).
 * 2. HAL should be implemented in such a way that they can be used in both
 *    both user and kernel space code.
 * 3. HAL should hide actual register layout where appropriate (infoframes, csc
 *    coefficients, etc). This objective is not met at the moment since in some
 *    cases (DMA configuration) we assume that end user knows the register
 *    layout (at least for now).
 *
 * ABBREVIATIONS
 *
 * GCP  - General Control Packet
 * AVI  - Auxiliaty Video Information
 * ACP  - Audio Content Protection
 * ISRC - International Standard Recording Code
 * SPD  - Source Product Description
 */

#ifndef __HDMI_HAL_H__
#define __HDMI_HAL_H__


#include <linux/types.h>

#include "otm_hdmi_types.h"
#include "otm_hdmi_defs.h"

/*
 * This enumeration represents HDMI unit revision
 */
typedef enum {
	HDMI_PCI_REV_CE3100 = 0,
	HDMI_PCI_REV_CE4100_A0 = 1,
	HDMI_PCI_REV_CE4100_B012 = 2,
	HDMI_PCI_REV_CE4200_A0 = 3,
	HDMI_PCI_REV_CE4200_B0 = 4,
} hdmi_unit_revision_id_t;

/*
 * HDMI register state
 */
typedef struct {
	bool valid;
	uint32_t saveDPLL;
	uint32_t saveFPA0;
	uint32_t savePIPEBCONF;
	uint32_t saveHTOTAL_B;
	uint32_t saveHBLANK_B;
	uint32_t saveHSYNC_B;
	uint32_t saveVTOTAL_B;
	uint32_t saveVBLANK_B;
	uint32_t saveVSYNC_B;
	uint32_t savePIPEBSRC;
	uint32_t saveDSPBSTRIDE;
	uint32_t saveDSPBLINOFF;
	uint32_t saveDSPBTILEOFF;
	uint32_t saveDSPBSIZE;
	uint32_t saveDSPBPOS;
	uint32_t saveDSPBSURF;
	uint32_t saveDSPBCNTR;
	uint32_t saveDSPBSTATUS;
	uint32_t save_palette_b[256];
	uint32_t savePFIT_CONTROL;
	uint32_t savePFIT_PGM_RATIOS;
	uint32_t saveHDMIPHYMISCCTL;
	uint32_t saveHDMIB_CONTROL;
} hdmi_register_state_t;

/*
 * HDMI infoframe information
 */
struct hdmi_infoframe_info_t {
	bool valid;
	uint32_t freq;
	otm_hdmi_packet_t pkt;
};

/*
 * This structure is used by HAL user to configure and use HAL
 */
typedef struct {
	/* Base address of mapped registers */
	unsigned int io_address;

	/* Base address of mapped interrupt registers */
	void __iomem *irq_io_address;

	/* Pointer to register read routine */
	unsigned int (*io_read) (void *uhandle,	/* User provided data */
				unsigned int base, /* Base address */
				unsigned int offset);	/* Register offset */

	/* Pointer to register write routine */
	void (*io_write) (void *uhandle, /* User provided data */
			unsigned int base, /* Base address */
			unsigned int offset, /* Register offset */
			unsigned int value); /* Value */


	/* Pointer to the data that will be
	 * passed to both io_read and io_write */
	void *uhandle;

	/* Pointer to the routine invoked at the beginning of every
	 * HAL call */
	void (*log_entry) (void *uhandle, /* User provided data */
			   char *foo);	/* Name of the routine */

	/* Pointer to the routine invoked at the end of every
	 * HAL call */
	void (*log_exit) (void *uhandle, /* User provided data */
			  char *foo, /* Name of the routine */
			  int rc); /* Return code */

	/* HDMI unit identifier */
	hdmi_unit_revision_id_t id;

	/* Pointer to opaque polling timer */
	void *poll_timer;

	/* Pointer to the polling timer initialization routine */
	void (*poll_start) (void *poll_timer);

	/* Pointer to the timeout verification routine */
	 bool(*poll_timeout) (void *poll_timer);

	/* Interrupt status to interrupt handling function */
	unsigned int isr_status;

	/*
	 * TODO: tmds clk value for the best pll found and is needed for audio.
	 * This field has to be moved into OTM audio interfaces
	 * when implemented.
	 */
	uint32_t clock_khz;

	/* HDMI register value */
	hdmi_register_state_t reg_state;
	/* AVI Infoframe - used for suspend resume */
	struct hdmi_infoframe_info_t avi;
} hdmi_device_t;

/*
 * Description: Infoframe handling facility supports two sending rates: one time
 *              send and every frame send
 */
typedef enum {
	HDMI_INFOFRAME_SEND_ONCE,
	HDMI_INFOFRAME_SEND_EVERY
} hdmi_infoframe_frequency_t;

/*
 * Description: CE 3100 provides SW with 4 slots for infoframes which can be
 *              sent as often as every frame. This routine loads given
 *              infoframe into given slot.
 *
 * @param [in] slot_number : number of slot; valid range is [1..4]
 * @param [in] frame       : pointer to the structure representing infoframe
 */
otm_hdmi_ret_t hdmi_infoframe_set(hdmi_device_t *dev,
				unsigned int slot_number,
				otm_hdmi_packet_t *frame);

/*
 * Description: Setting up sending frequency attribute for given slot
 *
 * @param [in] slot_number : slot number; valid range is [1..4]
 * @param [in] frequency   : sending frequency
 */
otm_hdmi_ret_t hdmi_infoframe_set_frequency(hdmi_device_t *dev,
				unsigned int slot_number,
				hdmi_infoframe_frequency_t frequency);

/*
 * Description: Enables/disables sending of infoframe from the given slot
 *
 * @param [in] slot_number : slot number; valid range is [1..4]
 */
otm_hdmi_ret_t hdmi_infoframe_enable(hdmi_device_t *dev,
					unsigned int slot_number);
otm_hdmi_ret_t hdmi_infoframe_disable(hdmi_device_t *dev,
					unsigned int slot_number);

/*
 * AUDIO UNIT
 */

/*
 * Description: Turns transmission of audio data and CTS/N packets on/off
 */
otm_hdmi_ret_t hdmi_audio_enable(hdmi_device_t *dev);
otm_hdmi_ret_t hdmi_audio_disable(hdmi_device_t *dev);

/*
 * Description: List of audio formats supported
 */
typedef enum {
	HDMI_AUDIO_FORMAT_LPCM,
	HDMI_AUDIO_FORMAT_HBR,
	HDMI_AUDIO_FORMAT_OBA,
	HDMI_AUDIO_FORMAT_DTS
} hdmi_audio_format_t;

/*
 * Description: Setting up given format
 *
 * @param [in] format : audio format
 */
otm_hdmi_ret_t hdmi_audio_set_format(hdmi_device_t *dev,
				hdmi_audio_format_t format);

/*
 * Description: Number of channels supported
 */
typedef enum {
	HDMI_AUDIO_NUM_CHANNELS_2,
	HDMI_AUDIO_NUM_CHANNELS_4,
	HDMI_AUDIO_NUM_CHANNELS_6,
	HDMI_AUDIO_NUM_CHANNELS_8
} hdmi_audio_num_channels_t;

/*
 * Description: Setting up number of channels
 *
 * @param [in] nchannels : number of channels
 */
otm_hdmi_ret_t hdmi_audio_set_num_channels(hdmi_device_t *dev,
					hdmi_audio_num_channels_t nchannels);

/*
 * Description: Specifying if sampling frequensy is larger than 192KHz
 *              Looks like this call is not really needed at the moment.
 *
 * @param [in] high_freq : GDL_TRUE  - sampling frequency is larger than
 *					192 KHz
 *                         GDL_FALSE - sampling frequency is smaller that
 *					192 KHz
 */
otm_hdmi_ret_t hdmi_audio_set_high_frequency(hdmi_device_t *dev,
						bool high_freq);

/*
 * Description: Setting up clock recovery value N
 *
 * @param [in] n - N value
 */
otm_hdmi_ret_t hdmi_audio_set_clock_recovery_n(hdmi_device_t *dev,
						unsigned int n);

/*
 * Description: HW computes CTS value based on current N value. CE3100 can
 *              bypass automatic CTS calculation.
 *
 * @param [in] bypass : GDL_FALSE - CTS calculation is done by HW
 *                      GDL_TRUE  - CTS calculation is done by SW
 */
otm_hdmi_ret_t hdmi_audio_set_clock_recovery_cts_bypass(hdmi_device_t *dev,
							bool bypass);

/*
 * Description: Setting up clock recovery CTS value when CTS bypass is enabled
 *
 * @param [in] cts : CTS value
 */
otm_hdmi_ret_t hdmi_audio_set_clock_recovery_cts(hdmi_device_t *dev,
						unsigned int cts);

/*
 * Description: Setting up audio clock divider register
 *              End user should compute it as (128 * FS) / 1000
 *
 * @param [in] clock_divider : clock divider
 */
otm_hdmi_ret_t hdmi_audio_set_clock_divider(hdmi_device_t *dev,
				       unsigned int clock_divider);

/*
 * Description: Setting up FIFO threshold level (in 32 bit words)
 *              Should be set to FIFO_SIZE - DMA_BURST_SIZE
 *
 * @param [in] threshold : FIFO threshold level
 */
otm_hdmi_ret_t hdmi_audio_set_fifo_threshold(hdmi_device_t *dev,
					unsigned int threshold);

/*
 * Description: Audio FIFO size is 192 32 bit words. This routine reads current
 *              number of words in the FIFO.
 *
 * @param [out] level : valid buffer to store number of 32bit words in the FIFO
 */
otm_hdmi_ret_t hdmi_audio_get_fifo_level(hdmi_device_t *dev,
					unsigned int *level);

/*
 * Description: Layout of subpackets in the audio packet
 */
typedef enum {
	HDMI_AUDIO_LAYOUT_0,
	HDMI_AUDIO_LAYOUT_1
} hdmi_audio_packet_layout_t;

/*
 * Description: Setting up subpackets layout bit. Refer to HDMI specs
 *              version 1.3 section 5.3.4 for details.
 *
 * @param [in] layout : audio subpackets layout
 */
otm_hdmi_ret_t hdmi_audio_set_packets_layout(hdmi_device_t *dev,
					hdmi_audio_packet_layout_t layout);

/*
 * Description: Setting up VR and VL bits in all audio subpackets
 *              Refer to HDMI specs version 1.3 section 5.3.4 for details
 *
 * @param [in] valid : GDL_TRUE - bit is on, GDL_FALSE - bit is off
 */
otm_hdmi_ret_t hdmi_audio_set_packets_valid_bits(hdmi_device_t *dev,
					    bool valid);

/*
 * Descripton: Setting up B bit for the first or for all subpackets
 *
 * @param [in] all : GDL_TRUE  - sets B bit for all valid subpackets
 *                   GDL_FALSE - sets B bit for the first subpacket only
 */
otm_hdmi_ret_t hdmi_audio_set_packets_b_bits(hdmi_device_t *dev,
						bool all);

/*
 * Description: Setting up flat bits in all audio packets header
 *              Refer to HDMI specs version 1.3 section 5.3.4 for details
 *
 * @param [in] flat : GDL_TRUE - bit is on, GDL_FALSE - bit is off
 */
otm_hdmi_ret_t hdmi_audio_set_packets_flat_bits(hdmi_device_t *dev,
						bool flat);

/*
 * Description: Setting up UR and UL bits in outgoing packets
 *              When packet layout is set to 0 then UR and UL in all subpackets
 *              are affected by a pair of bits. Hence 192 packets are affected
 *              in total. When packet layout is set to 1 then UR and UL in
 *              individual subpackets are affected. Hence at least 48 packets
 *              are affected.
 *
 * @param [in] data : valid buffer with 12 32-bit words each representing
 *                    16 packets
 */
otm_hdmi_ret_t hdmi_audio_set_packets_user_bits(hdmi_device_t *dev,
						unsigned int *data);

/*
 * Description: Triggers insertion of user bits into outgoing audio packets
 */
otm_hdmi_ret_t hdmi_audio_send_packets_user_bits(hdmi_device_t *dev);

/*
 * Description: Setting up CR and CL bits in outgoing packets.
 *              Rules of how many packets are affected are the same as in
 *              hdmi_audio_set_packets_user_bits call.
 *              152 subsequent subpackets will have CR and CL set to 0 (??)
 *
 * @param [in] data : valid buffer with 2 32-bit words lower 40 bits of which
 *                    are considered
 */
otm_hdmi_ret_t hdmi_audio_set_packets_channel_bits(hdmi_device_t *dev,
						unsigned int *data);

/*
 * Description: Reading the CR and CL bits in channel status registers.
 *              152 subsequent subpackets will have CR and CL set to 0 (??)
 *
 * @param [out] data : valid buffer with 2 32-bit words lower 40 bits of which
 *                     are considered
 */
otm_hdmi_ret_t hdmi_audio_get_packets_channel_bits(hdmi_device_t *dev,
						unsigned int *data);

/*
 * Description: Triggers insertion of channle bits into outgoing audio packets
 */
otm_hdmi_ret_t hdmi_audio_send_packets_channel_bits(hdmi_device_t *dev);

/*
 * DMA UNIT
 *
 * DMA can be configured to program itself by fetching series of
 * descriptors. Each descriptor represents data transfer to be done. So
 * transfering of audio data from audio buffers to the HDMI FIFO involves the
 * following:
 * - Audio buffer is devided into number of transfers
 * - Each transfer is represented by the node in the linked list of descriptors
 * - DMA NEXT_DESCR reg is set to the address of first descriptor in the list
 * - Linked-list mode, burst size and other stuff is set in DMA FLAGS register
 *
 * SRC_INT and DST_INT interrupts are available to help HAL user monitoring
 * what transfer from the list is currently in progress. Enabling them requires
 * hitting SRC_INT/DST_INT bits in FLAGS_MODE register of DMA unit and
 * bits 5/6 in HDMI interrupt control register.
 *
 * HAL provides three levels of DMA access.
 * 1. End user controls address of first descriptor
 * 2. End user controls flags register and address of first descriptor
 * 3. End user has complete control
 *
 * Levels 1 and 2 are enough for DMA usage. Level 3 is included in case there
 * are some cases where levels 1 and 2 are not covering the needs.
 */

/*
 * Description: DMA can be programmed to fetch series of descriptors, each of
 *              which specifies data transfer parameters. The following
 *              structure represents such a descriptor.
 */
typedef struct {
	/* address of the the next descriptor */
	unsigned int next_address;
	/* number of bytes in the current transfer */
	unsigned int transfer_size;
	/* address of the user input buffer */
	unsigned int src_start_address;
	/* HDMI FIFO address, should be 0x10101010 */
	unsigned int dst_start_address;
	unsigned int flags;
	/* Misc data associated with descriptor */
	unsigned int misc_data;
} hdmi_dma_descriptor_t;

/*
 * Description: Programs DMA with default configuration and start fetching
 *              sequence. This call is supposed to hide all configuration
 *              details from end user.
 *
 * @param [in] dscr_ph_addr : physical address of first descriptor to fetch
 */
otm_hdmi_ret_t hdmi_dma_default_start(hdmi_device_t *dev,
				unsigned int dscr_ph_addr);

/*
 * Description: Stop transfer sequence by hitting BIT0 of OTHER_MODE register
 *              in DMA unit.
 */
otm_hdmi_ret_t hdmi_dma_stop(hdmi_device_t *dev);

/*
 * Description: Gives end user a default configuration (i.e. the same
 *              configuration as hdmi_dma_defalt_start sets) so he can apply
 *              it to every entry of linked list of descriptors.
 */
otm_hdmi_ret_t hdmi_dma_get_default_flags(hdmi_device_t *dev,
						unsigned int *flags);

/*
 * Description: Setting up given configuration of DMA unit and starting the
 *              fetching sequence. The intention of this call is to give
 *              end user more control over the DMA unit than in
 *              hdmi_dma_default_start so user can tweak things a little bit
 *
 * @param [in] descr_phys_addr : physical address of first descriptor to fetch
 * @param [in] flags           : setting for the DMA flags register
 */
otm_hdmi_ret_t hdmi_dma_custom_start(hdmi_device_t *dev,
				unsigned int descr_phys_addr,
				unsigned int flags);

/*
 * Description: Structure below represtent DMA configuration
 */
typedef struct {
	unsigned int curr_descr;
	unsigned int next_descr;
	unsigned int srcdma_start;
	unsigned int dstdma_start;
	unsigned int srcdma_size;
	unsigned int flags_mode;
	unsigned int other_mode;
	unsigned int srcdma_bot;
	unsigned int srcdma_top;
	unsigned int dstdma_bot;
	unsigned int dstdma_top;
	unsigned int dstdma_size;
	unsigned int srcdma_stop;
	unsigned int dstdma_stop;
} hdmi_dma_config_t;

/*
 * Description: Providing end user with full control over the DMA
 *
 * @param [in] dma_config : pointer to dma configuration structure
 */
otm_hdmi_ret_t hdmi_dma_config(hdmi_device_t *dev,
				hdmi_dma_config_t *dma_config);

/*
 * VIDEO
 */

/*
 * Description: HDMI unit can operate in both HDMI and DVI modes
 */
typedef enum {MODE_HDMI, MODE_DVI} hdmi_operation_mode_t;

/*
 * Description: Selecting mode of operation
 *
 * @param [in] mode: mode of operation
 */
otm_hdmi_ret_t hdmi_video_set_operation_mode(hdmi_device_t *dev,
					hdmi_operation_mode_t mode);

/*
 * Description: Setting up pixel depth
 *
 * @param [in] pixel_depth : pixel depth
 */
otm_hdmi_ret_t hdmi_video_set_pixel_depth(hdmi_device_t *dev,
				otm_hdmi_output_pixel_depth_t pixel_depth);

/*
 * Description: HDMI unit supports dithering from 12 to 10 and 8 bits
 */
typedef enum {
	HDMI_DITHER_OUTPUT_8BITS,
	HDMI_DITHER_OUTPUT_10BITS
} hdmi_dither_output_t;

/*
 * Description: Selecting dither output
 *
 * @param [in] output : rounding from 12 bits to 10 or 8 bits
 */
otm_hdmi_ret_t hdmi_video_set_dither_output(hdmi_device_t *dev,
					hdmi_dither_output_t output);

/*
 * Description: HDMI unit can dither, round or don't do anything at all
 */
typedef enum {
	HDMI_DITHER_DITHER,
	HDMI_DITHER_ROUND,
	HDMI_DITHER_BYPASS
} hdmi_dither_t;

/*
 * Description: Choosing dithering
 *
 * @param [in] dither - selection of particular dithering
 */
otm_hdmi_ret_t hdmi_video_set_dither(hdmi_device_t *dev, hdmi_dither_t dither);

/*
 * Description: Enables/disables CSC
 *
 * @param [in] state : GDL_TRUE - CSC is ON, GDL_FALSE - CSC is off
 */
otm_hdmi_ret_t hdmi_video_set_csc(hdmi_device_t *dev, bool state);

/*
 * Descripton: Set additional 444 to 422 conversion. Assumption is that either
 *             pipe outputs 444 or we already convert to 444 prior to this 422
 *             conversion. Note that dithering is bypassed when 422 conversion
 *             is on.
 *
 * @param [in] state : GDL_TRUE  - perform additional 422 conversion
 *                     GDL_FALSE - do not perform additional 422 conversion
 *
 * @param [in] sd    : GDL_TRUE  - Current mode is SD
 *                     GDL_FALSE - Current mode is HD
 */
otm_hdmi_ret_t hdmi_video_set_csc_422(hdmi_device_t *dev,
				bool state,
				bool sd);

/*
 * Description: offset setup
 *
 * @param [in] in  :
 * @param [in] out :
 */
otm_hdmi_ret_t hdmi_video_set_csc_offset_YG(hdmi_device_t *dev,
					int in, int out);
otm_hdmi_ret_t hdmi_video_set_csc_offset_CbB(hdmi_device_t *dev,
					int in, int out);
otm_hdmi_ret_t hdmi_video_set_csc_offset_CrR(hdmi_device_t *dev,
					int in, int out);

/*
 * Description: this structure reflects clamp configuration register
 */
typedef struct {
	bool full_overrun;
	bool shift_output;
	bool output_clamp_960;
	bool output_clamp_enable;
	bool input_clamp_960;
	bool input_clamp_enable;
} hdmi_csc_clamp_t;

/*
 * Description: Clamp setup
 *
 * @param [in] clamp : user supplied buffer with clamp configuration
 */
otm_hdmi_ret_t hdmi_video_set_csc_clamp(hdmi_device_t *dev,
				hdmi_csc_clamp_t *clamp);

/*
 * Color space conversion coefficients are exposed keeping the following
 * conversion formula in mind (in matrix representation):
 *
 * | R/Cr |   | CC0 CC1 CC2 | | Cr/R |
 * | G/Y  | = | CC3 CC4 CC5 | | Y /G |
 * | B/Cb |   | CC6 CC7 CC8 | | Cb/B |
 *
 * Description: conversion coefficients setup
 *
 * @param [in] c0..c8 : coefficients from formula above
 */
otm_hdmi_ret_t hdmi_video_set_csc_012(hdmi_device_t *dev, float c0, float c1,
					float c2);
otm_hdmi_ret_t hdmi_video_set_csc_345(hdmi_device_t *dev, float c3, float c4,
					float c5);
otm_hdmi_ret_t hdmi_video_set_csc_678(hdmi_device_t *dev, float c6, float c7,
					float c8);

/*
 * Description: conversion coefficients setup. Format details are specified
 *		near each argument
 *
 * @param [in] c0..c8 : coefficients from folmula above (signed fixed point)
 */
otm_hdmi_ret_t hdmi_video_set_csc_012_fixed(hdmi_device_t *dev,
					unsigned int c0,	/* 3.10 */
					unsigned int c1,	/* 3.10 */
					unsigned int c2);	/* 3.10 */
otm_hdmi_ret_t hdmi_video_set_csc_345_fixed(hdmi_device_t *dev,
					unsigned int c3,	/* 3.10 */
					unsigned int c4,	/* 3.10 */
					unsigned int c5);	/* 3.10 */
otm_hdmi_ret_t hdmi_video_set_csc_678_fixed(hdmi_device_t *dev,
					unsigned int c6,	/* 3.10 */
					unsigned int c7,	/* 3.10 */
					unsigned int c8);	/* 3.10 */

/*
 * Description: Programs htotal/hblank values. See EAS for details as
 *              to how exactly to program these values
 */
otm_hdmi_ret_t hdmi_video_set_tv_timings(hdmi_device_t *dev,
					unsigned int hactive,
					unsigned int hblank);

/*
 * Description: Programs VSYNC and HSYNC polarity bit of Video Output Format
 *              Register
 *
 * @param [in] v : vertical polarity;   GDL_TRUE -positive,
					GDL_FALSE -negative
 * @param [in] h : horizontal polarity; GDL_TRUE -positive,
					GDL_FALSE -negative
 */
otm_hdmi_ret_t hdmi_video_set_output_polarity(hdmi_device_t *dev,
					bool v, bool h);

/*
 * Description: Controls settings of PP (pixel packing) bits in GCP packets
 *
 * @param [in] pp : GDL_FALSE - pp controlled phase is 0;
 *                  GDL_TRUE  - pp controlled phase is 1;
 */
otm_hdmi_ret_t hdmi_video_set_gcp_packet_pp_bits(hdmi_device_t *dev,
						bool pp);

/*
 * Description: Controls setting of CD (color depth) bits in GCP packet
 *
 * @param [in] cd : GDL_FALSE - cd bits are zero;
 *                  GDL_TRUE  - cd bits depend on color depth settings;
 */
otm_hdmi_ret_t hdmi_video_set_gcp_packet_cd_bits(hdmi_device_t *dev,
					    bool cd);

/*
 * Description: Controls pixel data source
 *
 * @param [in] pipe : GDL_TRUE  - pixel data comes from the pipe
 *                    GDL_FALSE - pixel data comes from the register
 */
otm_hdmi_ret_t hdmi_video_set_pixel_source(hdmi_device_t *dev,
						bool pipe);

/*
 * Description: Sets the color of the fixed pixel source
 *
 * @param [in] color : lower 24 bits represent RGB / CrYCb data to be shown
 */
otm_hdmi_ret_t hdmi_video_set_pixel_color(hdmi_device_t *dev,
						unsigned int color);

/*
 * hdmi_video_set_video_indicator()
 *
 * @param [in] on : GDL_TRUE  - enable video indicator bit
 *                : GDL_FALSE - disable video indicator bit
 */
otm_hdmi_ret_t hdmi_video_set_video_indicator(hdmi_device_t *dev,
						bool on);

/*
 * INTERRUPTS
 *
 * Interrupts listed in the enumeration represent actual bits to be written in
 * in the Interrupt Control Register. As of now interrupt status register has
 * the same layout and these value can be used directly to look for incoming
 * interupts in the interrupt handler.
 */

/*
 * Description: The following interrupts are avalable
 */
typedef enum {
	/* Keys are loaded from SEC */
	HDMI_INTERRUPT_HDCP_KEYS_READY = 0x00100000,
	/* Ri is available (128th frame) */
	HDMI_INTERRUPT_HDCP_RI = 0x00080000,
	/* Pre Ri is available (127th frame) */
	HDMI_INTERRUPT_HDCP_RI_PRE = 0x00040000,
	/* Pi is available ( 16th frame) */
	HDMI_INTERRUPT_HDCP_PI = 0x00020000,
	/* Pre Pi is available ( 15th frame) */
	HDMI_INTERRUPT_HDCP_PI_PRE = 0x00010000,
	/* Encrypted frame is sent */
	HDMI_INTERRUPT_HDCP_FRAME = 0x00008000,
	/* M0 is available for SEC access */
	HDMI_INTERRUPT_HDCP_M0 = 0x00004000,
	/* R0 computation is done */
	HDMI_INTERRUPT_HDCP_R0 = 0x00002000,
	/* Key buffer is ready after reset */
	HDMI_INTERRUPT_HDCP_KEY_BUFFER_RESET = 0x00001000,
	/* HW has transmitted 192 bytes block */
	HDMI_INTERRUPT_AUDIO_BLOCK_DONE = 0x00000100,
	/* FIFO is emtpy */
	HDMI_INTERRUPT_AUDIO_FIFO_UNDERFLOW = 0x00000080,
	/*  DMA completed write to DST */
	HDMI_INTERRUPT_DMA_DST_COMPLETE = 0x00000040,
	/* DMA completed read from SRC */
	HDMI_INTERRUPT_DMA_SRC_COMPLETE = 0x00000020,
	/* I2C bus error */
	HDMI_INTERRUPT_I2C_BUS_ERROR = 0x00000010,
	/* Internal 64 bytes buffer is full */
	HDMI_INTERRUPT_I2C_BUFFER_FULL = 0x00000008,
	/* Current I2C transaction done */
	HDMI_INTERRUPT_I2C_TRANSACTION_DONE = 0x00000004,
	/* Detection of Hot Plug */
	HDMI_INTERRUPT_HOTPLUG_DETECT = 0x00000001
} hdmi_interrupt_t;

/*
 * Description: Enabling/disabling interrupts of interest.
 *
 * @param [in] mask : user provided mask with OR'ed hdmi_interrupt_t values
 */
otm_hdmi_ret_t hdmi_interrupts_set_mask(hdmi_device_t *dev,
					unsigned int mask);

/*
 * Description: Reading current interrupts settings
 *
 * @param [in] mask : user provided mask to store current interupts settings
 */
otm_hdmi_ret_t hdmi_interrupts_get_mask(hdmi_device_t *dev,
					unsigned int *mask);

/*
 * Description: Reading current interrupts status
 *
 * @param [in] mask : user provided buffer to store current interrupts status
 */
otm_hdmi_ret_t hdmi_interrupts_get_status(hdmi_device_t *dev,
					unsigned int *mask);

/*
 * Description: Acknowledgement of given interrupts.
 *
 * @param [in] mask : user provided mask with OR'ed hdmi_interrupt_t values
 */
otm_hdmi_ret_t hdmi_interrupts_acknowledge(hdmi_device_t *dev,
					unsigned int mask);

/*
 * I2C
 *
 * Model of use:
 * - higher level uses hdmi_i2c_transaction_* and hdmi_i2c_buffer_* calls
 *   to manipulate data and start/stop transfers
 * - higher level is responsible for handling reads more than 64 bytes
 */

/*
 * Description: HDMI I2C unit support the following transactions
 */
typedef enum {
	HDMI_I2C_TRANSACTION_HDCP_READ,
	HDMI_I2C_TRANSACTION_HDCP_READ_RI,
	HDMI_I2C_TRANSACTION_HDCP_WRITE,
	HDMI_I2C_TRANSACTION_EDID_READ
} hdmi_i2c_transaction_type_t;

/*
 * Description: This structure describes I2C transaction
 */
typedef struct {
	hdmi_i2c_transaction_type_t type;
	unsigned int sp;
	unsigned int offset;
	unsigned int size;
} hdmi_i2c_transaction_t;

/*
 * Description: Size of HW I2C buffer. Higher level routines should use this
 *              constant when implementing reads larger than buffer size
 */
#define HDMI_I2C_BUFFER_SIZE 64

/*
 * Description: Initiates HW accelerated I2C transaction
 *
 * @param [in] transaction : pointer to transaction descriptor
 */
otm_hdmi_ret_t hdmi_i2c_transaction_start(hdmi_device_t *dev,
				hdmi_i2c_transaction_t *transaction);

/*
 * Description: Continues current transaction
 */
otm_hdmi_ret_t hdmi_i2c_transaction_continue(hdmi_device_t *dev);

/*
 * Description: Stops current transaction
 */
otm_hdmi_ret_t hdmi_i2c_transaction_stop(hdmi_device_t *dev);

/*
 * Description: Writes user supplied data into the internal i2c unit buffer
 *              User supplies up to 8 consecutive bytes. This call is
 *              responsible for positioning these bytes properly in the
 *              internal i2c buffer.
 *
 *              The following data can be written into hdcp port: An [8 bytes],
 *              Ainfo [1 byte] and Aksv  [5 bytes]. Keeping this in mind
 *              supported write size is limited to 1, 5 and 8 bytes
 *
 *              Aksv should be provided as 5 consecutive bytes.
 *              An should be provided as 8 bytes word
 *
 * @param [in] buffer : user supplied buffer with data
 * @param [in] size   : size of content in user buffer
 */
otm_hdmi_ret_t hdmi_i2c_buffer_write(hdmi_device_t *dev,
				void *buffer, unsigned int size);

/*
 * Description: Reads data from internal i2c buffer into user specified location
 *              Bytes are positioned in the user supplied buffer in the same
 *              order they reside at the specified offset.
 *
 * @param [out] buffer : user supplied buffer for data
 * @param [in]  size   : number of requested bytes; valid range is [1..64]
 */
otm_hdmi_ret_t hdmi_i2c_buffer_read(hdmi_device_t *dev,
			       void *buffer, unsigned int size);

/*
 * Description: I2C speed
 */
typedef enum {HDMI_I2C_SPEED_100KBPS,
	HDMI_I2C_SPEED_400KBPS
} hdmi_i2c_speed_t;

/*
 * Description: Resets, configures and enables I2C unit.
 *
 * @param [in] speed : operation speed
 */
otm_hdmi_ret_t hdmi_i2c_enable(hdmi_device_t *dev, hdmi_i2c_speed_t speed);

/*
 * Description: Disables I2C unit.
 */
otm_hdmi_ret_t ipil_hdmi_i2c_disable(hdmi_device_t *dev);

/*
 * Description: Alters DDC bus speed
 */
otm_hdmi_ret_t hdmi_i2c_set_ddc_speed(hdmi_device_t *dev,
					bool slow);

/*
 * Description: Gets I2C unit bus monitor
 */
otm_hdmi_ret_t hdmi_i2c_get_bus_monitor(hdmi_device_t *dev,
					unsigned int *value);

/*
 * Description: Gets I2C unis status
 */
otm_hdmi_ret_t hdmi_i2c_get_status(hdmi_device_t *dev, unsigned int *status);

/*
 * Non HW accelerated I2C read
 */
otm_hdmi_ret_t hdmi_i2c_sw_read(hdmi_device_t *dev,
			   hdmi_i2c_transaction_t *t, unsigned char *buffer);

/*
 * Non HW accelerated I2C write
 */
otm_hdmi_ret_t hdmi_i2c_sw_write(hdmi_device_t *dev,
			    hdmi_i2c_transaction_t *t, unsigned char *buffer);

/*
 * Configure SCL line
 */
otm_hdmi_ret_t hdmi_hal_i2c_scl_line_config(hdmi_device_t *dev,
						bool bool);

/*
 * Set SCL line
 */
otm_hdmi_ret_t hdmi_hal_i2c_scl_line_set(hdmi_device_t *dev, bool b);

/*
 * Set GCP line number and pixel offset in GCPLINE register.
 */
otm_hdmi_ret_t hdmi_hal_gcp_line_set(hdmi_device_t *dev, int line_no,
							int pel_off);

/*
 * Set / clear avmute flag in GCP  descriptor register.
 */
otm_hdmi_ret_t hdmi_hal_gcp_avmute_set(hdmi_device_t *dev,
					bool flag);

/*
 * HDCP
 */

/*
 * Description: Sets RND register of HDMI Tx
 *
 * @param [in] rnd : random 64 bits value
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_rnd(hdmi_device_t *dev, void *rnd);

/*
 * Description: Triggers computation of An
 */
otm_hdmi_ret_t hdmi_hdcp_tx_compute_an(hdmi_device_t *dev);

/*
 * Description: Reads An from the HDMI Tx
 *
 * @param [out] an : user supplied 64 bit buffer for An
 */
otm_hdmi_ret_t hdmi_hdcp_tx_get_an(hdmi_device_t *dev, void *an);

/*
 * Description: Writes An to the HDMI Tx
 *
 * @param [in] an : user supplied 64 bit buffer with An
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_an(hdmi_device_t *dev, void *an);

/*
 * Description: Writes HDMI Tx KSV
 *
 * @param [out] aksv : user supplied buffer with 40bit KSV
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_aksv(hdmi_device_t *dev, void *aksv);

/*
 * Description: Toggles REPEATER bit in HDMI Tx
 *
 * @param [in] repeater : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_repeater(hdmi_device_t *dev,
					bool repeater);

/*
 * Descriptor: Loads BKSV into the HDMI Tx
 *
 * @param [in] bksv : user supplied buffer with BKSV
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_bksv(hdmi_device_t *dev, void *bksv);

/*
 * Descriptor: Triggers computation of M0, R0 and K0
 */
otm_hdmi_ret_t hdmi_hdcp_tx_compute_mrk(hdmi_device_t *dev);

/*
 * Description: Makes M0 available for SEC usage
 */
otm_hdmi_ret_t hdmi_hdcp_tx_push_m0(hdmi_device_t *dev);

/*
 * Description: Toggles ENCRYPTION bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_encryption(hdmi_device_t *dev,
						bool state);

/*
 * Description: Queries current state of ENCRYPTION bit in Tx
 *
 * @param [out] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_get_encryption(hdmi_device_t *dev,
					bool *state);

/*
 * Description: Toggles AUTHENTICATON_STATE bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_auth(hdmi_device_t *dev, bool state);

/*
 * Description: Toggles 1.1_FEATURES bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_1p1_features(hdmi_device_t *dev,
						bool state);

/*
 * Description: Toggles EESS bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_eess(hdmi_device_t *dev, bool state);

/*
 * Description: Toggles OESS bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_oess(hdmi_device_t *dev, bool state);

/*
 * Description: Toggles RING_CIPHER bit in Tx
 *
 * @param [in] state : GDL_TRUE - on, GDL_FALSE - off
 */
otm_hdmi_ret_t hdmi_hdcp_tx_set_rc(hdmi_device_t *dev, bool state);

/*
 * Description: Reads Ri from Tx
 *
 * @param [out] r : user supplied buffer for Ri
 */
otm_hdmi_ret_t hdmi_hdcp_tx_get_r(hdmi_device_t *dev, unsigned int *r);
otm_hdmi_ret_t hdmi_hdcp_tx_get_r_pre(hdmi_device_t *dev, unsigned int *r_pre);

/*
 * Description: Reads Pi from Tx
 *
 * @param [out] p : user supplied buffer for Pi
 */
otm_hdmi_ret_t hdmi_hdcp_tx_get_p(hdmi_device_t *dev, unsigned int *p);
otm_hdmi_ret_t hdmi_hdcp_tx_get_p_pre(hdmi_device_t *dev, unsigned int *p_pre);

/*
 * Description: Clears M0, R0 and An related triggers
 */
otm_hdmi_ret_t hdmi_hdcp_reset_triggers(hdmi_device_t *dev);

/*
 * OTHER STUFF
 */

/*
 * Description: Querying of unit status register
 */
otm_hdmi_ret_t hdmi_general_get_status(hdmi_device_t *dev,
					unsigned int *status);

/*
 * Description: Set equalization
 */
otm_hdmi_ret_t hdmi_phy_set_equalization(hdmi_device_t *dev,
				    otm_hdmi_equalize_t eq1,
				    otm_hdmi_equalize_t eq2);

/*
 * Description: Set transmit level
 */
otm_hdmi_ret_t hdmi_phy_set_transmit_level(hdmi_device_t *dev,
				      otm_hdmi_transmit_level_t tl1,
				      otm_hdmi_transmit_level_t tl2);

/*
 * Description: Set termination
 */
otm_hdmi_ret_t hdmi_phy_set_termination(hdmi_device_t *dev,
				   otm_hdmi_termination_t t1,
				   otm_hdmi_termination_t t2);

/*
 * Description: Set Current Adjustment
 */
otm_hdmi_ret_t hdmi_phy_set_current(hdmi_device_t *dev,
				otm_hdmi_current_t c1,
				otm_hdmi_current_t c2);

/*
 * Description: Set band gap level
 */
otm_hdmi_ret_t hdmi_phy_set_bglvl(hdmi_device_t *dev, otm_hdmi_bglvl_t t);

/*
 * Description: Alternates between sending real signal and dummy pattern
 */
otm_hdmi_ret_t hdmi_misc_set_output(hdmi_device_t *dev, bool output);

#endif /* __HDMI_HAL_H__*/
