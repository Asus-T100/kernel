#ifndef _SH_CSS_LEGACY_H_
#define _SH_CSS_LEGACY_H_

#include "ia_css.h"

/** The pipe id type, distinguishes the kind of pipes that
 *  can be run in parallel.
 */
enum ia_css_pipe_id {
	IA_CSS_PIPE_ID_PREVIEW,
	IA_CSS_PIPE_ID_COPY,
	IA_CSS_PIPE_ID_VIDEO,
	IA_CSS_PIPE_ID_CAPTURE,
	IA_CSS_PIPE_ID_ACC,
};
#define IA_CSS_PIPE_ID_NUM (IA_CSS_PIPE_ID_ACC + 1)

struct ia_css_pipe_extra_config {
	bool enable_raw_binning;
	bool enable_yuv_ds;
	bool enable_high_speed;
	bool enable_dvs_6axis;
	bool enable_reduced_pipe;
	bool enable_dz;
	unsigned int isp_pipe_version;
	bool disable_vf_pp;
	bool disable_capture_pp;
};

enum ia_css_err
ia_css_pipe_create_extra(const struct ia_css_pipe_config *config,
			 const struct ia_css_pipe_extra_config *extra_config,
			 struct ia_css_pipe **pipe);

void
ia_css_pipe_extra_config_defaults(struct ia_css_pipe_extra_config *extra_config);

enum ia_css_err
ia_css_temp_pipe_to_pipe_id(const struct ia_css_pipe *pipe,
			    enum ia_css_pipe_id *pipe_id);

/** @brief Return the value of a SW interrupt.
 *
 * @param[in] irq	The software interrupt id.
 * @return		The value for the software interrupt.
 */
unsigned int
sh_css_get_sw_interrupt_value(unsigned int irq);

/** @brief Return whether UV range starts at 0.
 *
 * @param[out]	uv_offset_is_zero	Pointer to the result value.

 *  Return true if UV values range from 0 to 255 and false if UV values
 *  range from -127 to 128.
 */
void
sh_css_uv_offset_is_zero(bool *uv_offset_is_zero);

/** @brief Enable cont_capt mode (continuous preview+capture running together).
 *
 * @param	enable	Enabling value.
 *
 * Enable or disable continuous binaries if available. Default is disabled.
 */
void
sh_css_enable_cont_capt(bool enable, bool stop_copy_preview);

/** @brief Initialize the buffer queues in SP dmem
 *
 */
void
sh_css_init_buffer_queues(void);

enum ia_css_err
ia_css_pipe_id_set_irq_mask(enum ia_css_pipe_id pipe_id,
			    unsigned int or_mask,
			    unsigned int and_mask);

/* DEPRECATED. FPN is not supported. */
enum ia_css_err
sh_css_set_black_frame(struct ia_css_stream *stream,
			const struct ia_css_frame *raw_black_frame);

/** @brief Allocate a CSS MIPI frame structure of given size in bytes..
 *
 * @param	frame	The allocated frame.
 * @param[in]	size_bytes	The frame size in bytes.
 * @param[in]	contiguous	Allocate memory physically contiguously or not.
 * @return		The error code.
 *
 * Allocate a frame using the given size in bytes.
 * The frame structure is partially null initialized.
 */
enum ia_css_err
ia_css_mipi_frame_allocate(struct	ia_css_frame **frame,
				const unsigned int	size_bytes,
				const bool			contiguous);

/** @brief Specify a CSS MIPI frame buffer.
 *
 * @param[in]	size_bytes	The frame size in memory words (32B).
 * @param[in]	contiguous	Allocate memory physically contiguously or not.
 * @return		The error code.
 *
 * Specifies a CSS MIPI frame buffer: size in memory words (32B).
 */
enum ia_css_err
ia_css_mipi_frame_specify(const unsigned int	size_mem_words,
				const bool contiguous);

/** @brief Calculate the size of a mipi frame.
 *
 * @param[in]	width		The width (in pixels) of the frame.
 * @param[in]	height		The height (in lines) of the frame.
 * @param[in]	format		The frame (MIPI) format.
 * @param[in]	hasSOLandEOL	Whether frame (MIPI) contains (optional) SOL and EOF packets.
 * @param[in]	embedded_data_size_words		Embedded data size in memory words.
 * @param		size_mem_words					The mipi frame size in memory words (32B).
 * @return		The error code.
 *
 * Calculate the size of a mipi frame, based on the resolution and format. 
 */
enum ia_css_err
ia_css_mipi_frame_calculate_size(const unsigned int width,
				const unsigned int height,
				const enum ia_css_stream_format format,
				const bool hasSOLandEOL,
				const unsigned int embedded_data_size_words,
				unsigned int *size_mem_words);

#endif /* _SH_CSS_LEGACY_H_ */
