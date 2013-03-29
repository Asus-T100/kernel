#ifndef _IA_CSS_ACC_H_
#define _IA_CSS_ACC_H_

#include "ia_css.h"
#include "ia_css_acc_types.h"
#include "sh_css_legacy.h"

/* Acceleration API.
 * This file is still in CSS 1.5 style, it will be converted to the
 * 2.0 API and naming conventions soon.
 * Do not start any new code on this API until the 2.0 version is
 * available.
 */

#include "ia_css_acc_v1.h"

struct ia_css_pipe;
struct sh_css_pipeline;


/** @brief Unload firmware for acceleration.
 *
 * @param	firmware	Firmware to be unloaded.
 *
 * Unload firmware for acceleration.
 */
void
sh_css_unload_acceleration(struct ia_css_acc_fw *firmware);

/** @brief Stop the acceleration pipe
 *
 * @return	       IA_CSS_SUCCESS or error code upon error.
 *
 * This function stops the acceleration pipe that's running. Note that any
 * dependent pipes will also be stopped automatically since otherwise
 * they would starve because they no longer receive input data.
 */
enum ia_css_err
sh_css_acceleration_stop(struct ia_css_pipe *pipe);


/** @brief Set isp dmem parameters for acceleration.
 *
 * @param       firmware        Firmware of acceleration.
 * @param       val             Parameter value.
 * @return                      IA_CSS_SUCCESS or error code upon error.
 *
 * Set acceleration parameter to value <val>.
 * The parameter value is an isp pointer, i.e. allocated in DDR and mapped
 * to the CSS virtual address space.
 */
enum ia_css_err
sh_css_set_firmware_dmem_parameters(struct ia_css_fw_info *firmware,
                                    enum ia_css_isp_memories mem,
                                    ia_css_ptr val, size_t size);

/** @brief Append a stage to pipeline.
 *
 * @param	pipeline	Pointer to the pipeline to be extended.
 * @param[in]	isp_fw		ISP firmware of new stage.
 * @param[in]	in		The input frame to the stage.
 * @param[in]	out		The output frame of the stage.
 * @param[in]	vf		The viewfinder frame of the stage.
 * @return			IA_CSS_SUCCESS or error code upon error.
 *
 * Append a new stage to *pipeline. When *pipeline is NULL, it will be created.
 * The stage consists of an ISP binary <isp_fw> and input and output arguments.
*/
enum ia_css_err
sh_css_append_stage(struct sh_css_pipeline **pipeline,
		    const char *isp_fw,
		    struct ia_css_frame *in,
		    struct ia_css_frame *out,
		    struct ia_css_frame *vf);

/** @brief Create an empty pipeline.
 */
struct sh_css_pipeline *
sh_css_create_pipeline(void);

/** @brief Add an accelerator stage to a pipeline.
 *
 * @param	pipeline	The pipeline to be appended to.
 * @param	acc_fw		The fw descriptor of the new stage
 */
enum ia_css_err
sh_css_pipeline_add_acc_stage(struct sh_css_pipeline *pipeline,
			      const void             *acc_fw);
/** @brief Start a pipeline.
 *
 * @param	pipe_id		The pipe id where to run the pipeline. (Huh ?)
 * @param	pipeline	The pipeline to be executed.
 *
 * Start a pipeline, does not wait until the pipeline completes.
 */
void
sh_css_start_pipeline(enum ia_css_pipe_id pipe_id, struct sh_css_pipeline *pipeline);

/** @brief Close a pipeline.
 *
 * @param	pipeline	The pipeline to be closed.
 *
 * Close a pipeline and free all memory allocated to it.
 */
void
sh_css_close_pipeline(struct sh_css_pipeline *pipeline);

#endif /* _IA_CSS_ACC_H_ */
