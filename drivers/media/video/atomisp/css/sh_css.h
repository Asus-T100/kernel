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

#ifndef _SH_CSS_H_
#define _SH_CSS_H_

#include "sh_css_types.h"
#include "sh_css_params.h"

/* ===== GENERIC ===== */

/* Initialize the API. This is required before any other function in this
 * API can be called.
 * Arguments:
 *  - malloc_func: memory allocation function such as malloc or kalloc.
 *  - free_func:   memory free function such as free or kfree
 *  - irq_setting: the way interrupts should be used (or not)
 */
enum sh_css_err
sh_css_init(void *(*malloc_func) (size_t size),
	    void (*free_func) (void *ptr),
	    void (*flush_func) (struct sh_css_acc_fw *fw),
	    enum sh_css_interrupt_setting irq_setting,
	    const char *fw_data,
	    unsigned int fw_size);

/* Uninitialize the API. This cleans up all internal data structures. */
void
sh_css_uninit(void);

/* Suspend/resume: use suspend before powering the CSS and use resume
   after powering it up again. The resume function will make sure the
   CSS is correctly reprogrammed. This assumes that all buffers allocated
   in DDR will remain alive during power down. If this is not the case,
   use sh_css_unit() followed by sh_css_init() at power up. */
void
sh_css_suspend(void);

void
sh_css_resume(void);

/* Set the print function. This function is used to print debug information
 * if debugging is enable. An example of the argument is printf.
 */
void
sh_css_set_print_function(int (*func) (const char *fmt, ...));

/* When an interrupt occurs from the ISP_CSS, use this function to get
 * information about this interrupt. Since multiple interrupts can occur
 * at the same time, this function returns whether there are more interrupts
 * pending. This function should be called until this return value becomes
 * false.
 */
enum sh_css_err
sh_css_translate_interrupt(unsigned int *irq_infos);

/* Get all interrupt bits from the CSS receiver. */
void
sh_css_rx_get_interrupt_info(unsigned int *irq_infos);

/* Reset all bits in the irq_infos variable. */
void
sh_css_rx_clear_interrupt_info(unsigned int irq_infos);

/* To be called on a SH_CSS_IRQ_INFO_FW_ACC_DONE interrupt */
void
sh_css_terminate_firmware(void);

bool
sh_css_next_stage_is_acc(void);

enum sh_css_err
sh_css_start_next_stage(void);

void
sh_css_mmu_set_page_table_base_address(void *base_address);

void *
sh_css_mmu_get_page_table_base_address(void);

void
sh_css_mmu_invalidate_cache(void);

/* Enable or disable certain interrupts. The interrupt info type is used
 * here to indicate the interrupt to enable or disable.
 */
enum sh_css_err
sh_css_enable_interrupt(enum sh_css_interrupt_info info, bool enable);

/* Return the value of a SW interrupt */
unsigned int
sh_css_get_sw_interrupt_value(void);

/* Allocate a histogram. */
enum sh_css_err
sh_css_histogram_allocate(unsigned int num_elements,
			  struct sh_css_histogram **histogram);

/* Free a histogram */
void
sh_css_histogram_free(struct sh_css_histogram *histogram);

/* Return true if UV values range from 0 to 255 and false if UV values
 *  range from -127 to 128.
 */
void
sh_css_uv_offset_is_zero(bool *uv_offset_is_zero);

/* When interrupts are disabled, use this function to wait for a particular
 * ISP mode to complete.
 */
enum sh_css_err
sh_css_wait_for_completion(void);

/* Set the current input resolution. This needs to be called every time the
 * sensor resolution changes.
 */
enum sh_css_err
sh_css_input_set_resolution(unsigned int width, unsigned int height);

/* Set the part of the input resolution that will be the input to the ISP.
 * The difference between the input resolution and effective input resolution
 * will be cropped off. When the effective input resolution is exceeds the
 * output resolution, the ISP will downscale the input to the output resolution
 * in the domain.
 * Note that the effective input resolution cannot be smaller than the output
 * resolution.
 */
enum sh_css_err
sh_css_input_set_effective_resolution(unsigned int width, unsigned int height);

/* Specify the format of the input data. This format is used for all input
 * sources except memory (mipi receiver, prbs, tpg, fifo).
 */
void
sh_css_input_set_format(enum sh_css_input_format format);

/*
 * Return the last set format of the input data
 */
void
sh_css_input_get_format(enum sh_css_input_format *format);

void
sh_css_input_set_binning_factor(unsigned int binning_factor);

/* Translate an input format and mipi compression pair to the fmt_type.
 * This is normally done by the sensor, but when using the input fifo, this
 * format type must be sumitted correctly by the application.
 */
enum sh_css_err
sh_css_input_format_type(enum sh_css_input_format input_format,
			 enum sh_css_mipi_compression compression,
			 unsigned int *fmt_type);

/* Specify that the input will be sent as 2 pixels per clock.
 * The default is one pixel per clock.
 */
void
sh_css_input_set_two_pixels_per_clock(bool two_pixels_per_clock);

/*
 * Return the last set "2 pixels per clock" setting
 */
void
sh_css_input_get_two_pixels_per_clock(bool *two_pixels_per_clock);

/* Specify the bayer order of the input. The default is grbg. */
void
sh_css_input_set_bayer_order(enum sh_css_bayer_order bayer_order);

/* Get the number of extra rows and columns needed to program the
 * sensor driver with the correct resolution.
 * This is dependent upon the bayer order which is assumed to have
 * been already set using the API sh_css_input_set_bayer_order
 */
int
sh_css_get_extra_pixels_count(int *extra_rows, int *extra_columns);

/* Specify which channel carries the input for the CSS. */
void
sh_css_input_set_channel(unsigned int channel_id);

/* Set the input mode to be used. */
void
sh_css_input_set_mode(enum sh_css_input_mode mode);

/* Configure the MIPI receiver:
 *  - port: select the 1lane or 4lane port
 *  - num_lanes: this argument is only valid for the 4lane port. it specifies
 *               how many of these 4 lanes are in use. Valid values are 1, 2,
 *               3 or 4.
 *  - timeout: this specifies the timeout after which a timeout interrupt is
 *             generated.
 *             The timeout is specified in terms of <TO BE CLARIFIED>.
 */
enum sh_css_err
sh_css_input_configure_port(enum sh_css_mipi_port port,
			    unsigned int num_lanes, unsigned int timeout);

/* Specify the number of bits per compressed and uncompressed pixel for a given
 * compression mode.
 */
enum sh_css_err
sh_css_input_set_compression(enum sh_css_mipi_compression comp,
			     unsigned int compressed_bits_per_pixel,
			     unsigned int uncompressed_bits_per_pixel);

/* Configure the Test Pattern Generator, the way these values are used to
 * generate the pattern can be seen in the HRT extension for the test pattern
 * generator:
 * devices/test_pat_gen/hrt/include/test_pat_gen.h: hrt_calc_tpg_data().
 */
void
sh_css_tpg_configure(unsigned int x_mask, int x_delta,
		     unsigned int y_mask, int y_delta, unsigned int xy_mask);

/* Seed the for the Pseudo Random Bit Sequence */
void
sh_css_prbs_set_seed(int seed);

/* Digital zoom: this feature can be configured with a zoom factor
 * which determines the amount of zoom and a zoom center which determines
 * the point to zoom in at.
 * This feature is currently available only for video, but will become
 * available for preview and capture as well.
 * Set the digital zoom factor, this is a logarithmic scale. The actual zoom
 * factor will be 64/x.
 * Setting dx or dy to 0 disables digital zoom for that direction.
 */
void
sh_css_set_zoom_factor(unsigned int dx, unsigned int dy);

/* Get the current zoom factor. This will return the same values as were set
 * during the last video_set_zoom_factor() call.
 */
void
sh_css_get_zoom_factor(unsigned int *dx, unsigned int *dy);

/* Specify the overlay to be used for each viewfinder frame generated.
 * This overlay will remain active until it is reset by passing NULL to
 * this same function.
 */
void
sh_css_overlay_set_for_viewfinder(const struct sh_css_overlay *overlay);

/* Set the shading table for the current sensor module. This table will be
 * used for shading correction in each mode that supports this feature.
 */
void
sh_css_set_shading_table(const struct sh_css_shading_table *table);

/* ===== FRAMES ===== */

/* Allocate a frame of a certain resolution and format. */
enum sh_css_err
sh_css_frame_allocate(struct sh_css_frame **frame,
		      unsigned int width,
		      unsigned int height,
		      enum sh_css_frame_format format,
		      unsigned int padded_width,
		      unsigned int raw_bit_depth);

/* Allocate a frame using the resolution and format from a frame info struct. */
enum sh_css_err
sh_css_frame_allocate_from_info(struct sh_css_frame **frame,
				const struct sh_css_frame_info *info);

/* Free a frame */
void
sh_css_frame_free(struct sh_css_frame *frame);

/* ===== FPGA display frames ====== */
/* Contiguous frame allocation, only for FPGA display driver which needs
   physically contiguous memory. */
enum sh_css_err
sh_css_frame_allocate_contiguous(struct sh_css_frame **frame,
				 unsigned int width,
				 unsigned int height,
				 enum sh_css_frame_format format,
				 unsigned int padded_width,
				 unsigned int raw_bit_depth);

enum sh_css_err
sh_css_frame_allocate_contiguous_from_info(struct sh_css_frame **frame,
					  const struct sh_css_frame_info *info);

/* ===== PREVIEW ===== */

/* Start the ISP in preview mode, this will run the preview ISP on one frame.
 * After this has completed, it needs to be started again for the next frame.
 */
enum sh_css_err
sh_css_preview_start(struct sh_css_frame *raw_out_frame,
		     struct sh_css_frame *out_frame);

/* Enable or disable online binaries if available. Default is enabled. */
void
sh_css_preview_enable_online(bool enable);

/* Enable or disable continuous binaries if available. Default is disabled. */
void
sh_css_enable_continuous(bool enable);

/* Return whether continuous binaries are enabled */
bool
sh_css_continuous_is_enabled(void);

/* Disable vf_pp: used to replace by dynamic binary */
void
sh_css_disable_vf_pp(bool disable);

/* Disable capture_pp: used to replace by dynamic binary */
void
sh_css_disable_capture_pp(bool disable);

/* Specify the output resolution to be used by the preview ISP. */
enum sh_css_err
sh_css_preview_configure_output(unsigned int width,
				unsigned int height,
				enum sh_css_frame_format format);

/* Get the information about the output frames, this contains the resolution
 * and the stride. To allocate frames, use the information returned here.
 */
enum sh_css_err
sh_css_preview_get_output_frame_info(struct sh_css_frame_info *info);

enum sh_css_err
sh_css_preview_get_grid_info(struct sh_css_grid_info *info);

enum sh_css_err
sh_css_preview_get_input_resolution(unsigned int *width,
				    unsigned int *height);

/* Set the input resolution for viewfinder post processing.
   When this is not set, the input resolution is equal to the input
   resolution of the preview pipeline. When this is set, the YUV scaler
   in the viewfinder post processing step will be activated. */
enum sh_css_err
sh_css_preview_configure_pp_input(unsigned int width, unsigned int height);

/* Return whether the next preview stage (or first stage of next
   frame) needs to perform memory allocation or not.
   This can be used to determine whether the next stage can be
   started from an atomic context.
   Since no CSS functionality can ever perform blocking operations,
   memory allocation is the only thing that would not allow us to
   start the next stage from an atomic context. */
bool
sh_css_preview_next_stage_needs_alloc(void);

/* ===== CAPTURE ===== */

/* Start the ISP in capture mode:
 *  - out_frame: pointer to the output frame
 *  - vf_frame: pointer to the viewfinder frame
 */
enum sh_css_err
sh_css_capture_start(struct sh_css_frame *raw_out_frame,
		     struct sh_css_frame *out_frame,
		     struct sh_css_frame *vf_frame);

/* Specify the mode used for capturing. */
void
sh_css_capture_set_mode(enum sh_css_capture_mode mode);

/* Enable the eXtra Noise Reduction as a post processing step. This will be
 * run on both the captured output and the viewfinder output.
 */
void
sh_css_capture_enable_xnr(bool enable);

/* Specify the output resolution for captured images. */
enum sh_css_err
sh_css_capture_configure_output(unsigned int width,
				unsigned int height,
				enum sh_css_frame_format format);

/* Specify the viewfinder resolution. Note that this resolution currently
 * has to be a division of the captured output by a power of 2. The API will
 * automatically select the resolution that's closest to the one requested
 * here.
 */
enum sh_css_err
sh_css_capture_configure_viewfinder(unsigned int width,
				    unsigned int height,
				    enum sh_css_frame_format format);

/* For non-raw still captures, downscaling in the YUV domain can be done
 * during post processing. This function specifies the input resolution
 * for the YUV downscaling step. If this resolution is not set, the YUV
 * downscaling will not be done. The output resolution of the YUV
 * downscaling is taken from the configure_output function above.
 */
enum sh_css_err
sh_css_capture_configure_pp_input(unsigned int width,
				  unsigned int height);

/* Enable or disable online binaries if available. Default is enabled. */
void
sh_css_capture_enable_online(bool enable);

/* Retrieve the format and resolution of the output frames. Note that this
 * can differ from the requested resolution.
 */
enum sh_css_err
sh_css_capture_get_output_frame_info(struct sh_css_frame_info *info);

/* Retrieve the format and resolution of the viewfinder frames. Note that this
 * can differ from the requested resolution.
 */
enum sh_css_err
sh_css_capture_get_viewfinder_frame_info(struct sh_css_frame_info *info);

/* Retrieve the format and resolution of the RAW frame. This is only available
 * when the capture is set to offline or continuous using:
 * sh_css_capture_enable_online(false) or
 * sh_css_enable_continuous(true).
 */
enum sh_css_err
sh_css_capture_get_output_raw_frame_info(struct sh_css_frame_info *info);

enum sh_css_err
sh_css_capture_get_grid_info(struct sh_css_grid_info *info);

enum sh_css_err
sh_css_capture_get_input_resolution(unsigned int *width,
				    unsigned int *height);

/* See comments for sh_css_preview_next_stage_needs_alloc() */
bool
sh_css_capture_next_stage_needs_alloc(void);

/* ===== VIDEO ===== */

/* Start the video ISP for one frame:
 *  - in_frame: pointer to the input frame, this argument is only used if the
 *              input_mode is set to sh_css_input_mode_memory.
 *  - out_frame: pointer to the output frame.
 *  - vf_frame: pointer to the viewfinder output frame.
 */
enum sh_css_err
sh_css_video_start(struct sh_css_frame *in_frame,
		   struct sh_css_frame *out_frame,
		   struct sh_css_frame *vf_frame);

/* Specify the output resolution for output frames. Note that the actual
 * resolution can be different from the requested resolution.
 */
enum sh_css_err
sh_css_video_configure_output(unsigned int width,
			      unsigned int height,
			      enum sh_css_frame_format format);

/* Specify the viewfinder resolution. Note that this resolution currently has
 * to be a division of the captured output by a power of 2. The API will
 * automatically select the resolution that's closest to the one requested
 * here.
 */
enum sh_css_err
sh_css_video_configure_viewfinder(unsigned int width,
				  unsigned int height,
				  enum sh_css_frame_format format);

/* Set the motion vector for Digital Image Stabilization (DIS).
 * These positions are normally calculated using the DIS statistics.
 */
void
sh_css_video_set_dis_vector(int x, int y);

/* Specify the envelope to be used for DIS. */
void
sh_css_video_set_dis_envelope(unsigned int width, unsigned int height);

/* Retrieve the envelope to be used for DIS. */
void
sh_css_video_get_dis_envelope(unsigned int *width, unsigned int *height);

/* Retrieve the format and resolution of the output frames. Note that this
 * can differ from the requested resolution.
 */
enum sh_css_err
sh_css_video_get_output_frame_info(struct sh_css_frame_info *info);

/* Retrieve the format and resolution of the viewfinder frames. Note that this
 * can differ from the requested resolution.
 */
enum sh_css_err
sh_css_video_get_viewfinder_frame_info(struct sh_css_frame_info *info);

enum sh_css_err
sh_css_video_get_grid_info(struct sh_css_grid_info *info);

enum sh_css_err
sh_css_video_get_input_resolution(unsigned int *width,
				  unsigned int *height);

/* See comments for sh_css_preview_next_stage_needs_alloc() */
bool
sh_css_video_next_stage_needs_alloc(void);

/* Generate a luminance histogram from a frame. The width of the frame
 * cannot exceed 640 pixels and the frame must be a yuv420 frame.
 */
enum sh_css_err
sh_css_histogram_start(const struct sh_css_frame *input_frame,
		       struct sh_css_histogram *histogram);

/* Send streaming data into the css input FIFO. This is for testing purposes
 * only. This uses the channel ID and input format as set by the user with
 * the regular functions for this.
 * This function blocks until the entire frame has been written into the
 * input FIFO.
 */
void
sh_css_send_input_frame(unsigned short *data,
			unsigned int width,
			unsigned int height);

/*
 * For higher flexibility the sh_css_send_input_frame is replaced by
 * three seperate functions:
 * 1) sh_css_streaming_to_mipi_start_frame
 * 2) sh_css_streaming_to_mipi_send_line
 * 3) sh_css_streaming_to_mipi_end_frame
 * In this way it is possible to stream multiple frames on different
 * channel ID's on a line basis. It will be possible to simulate
 * line-interleaved Stereo 3D muxed on 1 mipi port.
 * These 3 functions are for testing purpose only and can be used in
 * conjunction with sh_css_send_input_frame
 */

/*
 * Starts the streaming to mipi frame by sending SoF for channel channel_id.
 * It will use the input_format and two_pixels_per_clock as provided by
 * the user.
 * For the "correct" use-case, input_format and two_pixels_per_clock must match
 * with the values as set by the user with the regular functions.
 * To simulate an error, the user can provide "incorrect" values for
 * input_format and/or two_pixels_per_clock.
 */
void
sh_css_streaming_to_mipi_start_frame(unsigned int channel_id,
				enum sh_css_input_format input_format,
				bool two_pixels_per_clock);


/*
 * Sends 1 frame line. Start with SoL followed by width bytes of data, folowed
 * by width2 bytes of data2 and followed by and EoL
 * It will use the input_format and two_pixels_per_clock settings as provided
 * with the sh_css_streaming_to_mipi_start_frame function call.
 *
 * This function blocks until the entire line has been written into the
 * input FIFO.
 */
void
sh_css_streaming_to_mipi_send_line(unsigned int channel_id,
						unsigned short *data,
						unsigned int width,
						unsigned short *data2,
						unsigned int width2);


/*
 * Stops the streaming to mipi frame by sending EoF for channel channel_id.
 */
void
sh_css_streaming_to_mipi_end_frame(unsigned int channel_id);

/* Temporary function to poll whether the ISP has been started. Once it has,
 * the sensor can also be started. */
bool
sh_css_isp_has_started(void);

/* Load firmware for acceleration */
enum sh_css_err
sh_css_load_acceleration(struct sh_css_acc_fw *firmware);

/* Unload firmware for acceleration */
void
sh_css_unload_acceleration(struct sh_css_acc_fw *firmware);

/* Set argument <num> of size <size> to value <val> */
enum sh_css_err
sh_css_set_acceleration_argument(struct sh_css_acc_fw *firmware,
				 unsigned num, void *val, size_t size);

/* Start acceleration of firmware.
   Load the firmware if not yet loaded.
*/
enum sh_css_err
sh_css_start_acceleration(struct sh_css_acc_fw *firmware);

/* To be called when acceleration has terminated.
*/
void
sh_css_acceleration_done(struct sh_css_acc_fw *firmware);

/* Abort acceleration within <deadline> microseconds
*/
void
sh_css_abort_acceleration(struct sh_css_acc_fw *firmware, unsigned deadline);

#endif /* _SH_CSS_H_ */
