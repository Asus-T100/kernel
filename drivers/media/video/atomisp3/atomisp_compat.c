/*
 * Support for Merrifield Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#include "ia_css.h"
#include "atomisp_internal.h"
#include "atomisp_fops.h"
#include "ia_css_types.h"

#include <asm/intel-mid.h>

#include <linux/delay.h>
enum frame_info_type {
	VF_FRAME,
	OUTPUT_FRAME,
	RAW_FRAME,
};
static inline
enum ia_css_pipe_mode __pipe_id_to_pipe_mode(enum ia_css_pipe_id pipe_id)
{
	switch (pipe_id) {
	case IA_CSS_PIPE_ID_PREVIEW:
		return IA_CSS_PIPE_MODE_PREVIEW;
	case IA_CSS_PIPE_ID_CAPTURE:
		return IA_CSS_PIPE_MODE_CAPTURE;
	case IA_CSS_PIPE_ID_VIDEO:
		return IA_CSS_PIPE_MODE_VIDEO;
	case IA_CSS_PIPE_ID_ACC:
		return IA_CSS_PIPE_MODE_ACC;
	default:
		return IA_CSS_PIPE_MODE_NUM;
	}

}
static void __configure_output(struct atomisp_sub_device *isp_subdev,
			       unsigned int width,
			       unsigned int height,
			       enum ia_css_frame_format format,
			       enum ia_css_pipe_id pipe_id)
{
	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode =
	    __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.width =
	    width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.res.height =
	    height;
	isp_subdev->css2_basis.pipe_configs[pipe_id].output_info.format =
	    format;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d] output info w=%d.h=%d.f=%d.\n",
		 pipe_id, width, height, format);
}
static void __configure_pp_input(struct atomisp_sub_device *isp_subdev,
				 unsigned int width,
				 unsigned int height,
				 enum ia_css_pipe_id pipe_id)
{
	if (width == 0 && height == 0)
		return;

	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode =
	    __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	isp_subdev->css2_basis.pipe_extra_configs[pipe_id].enable_yuv_ds = true;
	isp_subdev->css2_basis.pipe_configs[pipe_id].bayer_ds_out_res.width =
	    width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].bayer_ds_out_res.height =
	    height;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d]capture pp input w=%d.h=%d.\n",
		 pipe_id, width, height);
}
static void __configure_vf_output(struct atomisp_sub_device *isp_subdev,
				  unsigned int width,
				  unsigned int height,
				  enum ia_css_frame_format format,
				  enum ia_css_pipe_id pipe_id)
{
	isp_subdev->css2_basis.curr_pipe = pipe_id;
	isp_subdev->css2_basis.pipe_configs[pipe_id].mode =
	    __pipe_id_to_pipe_mode(pipe_id);
	isp_subdev->css2_basis.update_pipe[pipe_id] = true;

	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.res.width =
	    width;
	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.res.height =
	    height;
	isp_subdev->css2_basis.pipe_configs[pipe_id].vf_output_info.format =
	    format;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "configuring pipe[%d] vf output info w=%d.h=%d.f=%d.\n",
		 pipe_id, width, height, format);
}

static enum ia_css_err __destroy_pipes(struct atomisp_sub_device *isp_subdev,
				       bool force)
{
	int i;
	enum ia_css_err ret = IA_CSS_SUCCESS;

	if (isp_subdev->css2_basis.stream) {
		dev_dbg(isp_subdev->isp->dev, "destroy css stream first.\n");
		return IA_CSS_ERR_INTERNAL_ERROR;
	}
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i]
				&& (force ||
				    isp_subdev->css2_basis.update_pipe[i])) {
			ret |= ia_css_pipe_destroy(
					isp_subdev->css2_basis.pipes[i]);
			if (ret) {
				v4l2_err(&atomisp_dev,
					 "destroy pipe[%d]failed.\
					 cannot recover\n", i);
			}
			isp_subdev->css2_basis.pipes[i] = NULL;
			isp_subdev->css2_basis.update_pipe[i] = false;
		}
	}

	return ret;
}
static enum ia_css_err __create_pipe(struct atomisp_sub_device *isp_subdev)
{

	int i, j;
	enum ia_css_err ret;
	struct ia_css_pipe_extra_config extra_config;

	v4l2_dbg(5, dbg_level, &atomisp_dev, ">%s\n", __func__);
	ia_css_pipe_extra_config_defaults(&extra_config);
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipe_configs[i].output_info.
		    res.width) {
			if (!memcmp(&extra_config,
					&isp_subdev->css2_basis.
					pipe_extra_configs[i],
				    sizeof(extra_config)))
				ret = ia_css_pipe_create(
					&isp_subdev->css2_basis.pipe_configs[i],
					&isp_subdev->css2_basis.pipes[i]);
			else
				ret = ia_css_pipe_create_extra(
					&isp_subdev->css2_basis.pipe_configs[i],
					&isp_subdev->css2_basis.
					pipe_extra_configs[i],
					&isp_subdev->css2_basis.pipes[i]);
			if (ret) {
				v4l2_err(&atomisp_dev, "create pipe[%d] error.\n", i);
				goto pipe_err;
			}
			v4l2_dbg(5, dbg_level, &atomisp_dev,
				 "dump pipe[%d] info w=%d, h=%d,f=%d vf_w=%d vf_h=%d vf_f=%d.\n",
				 i,
				 isp_subdev->css2_basis.pipe_configs[i].
				 output_info.res.width,
				 isp_subdev->css2_basis.pipe_configs[i].
				 output_info.res.height,
				 isp_subdev->css2_basis.pipe_configs[i].
				 output_info.format,
				 isp_subdev->css2_basis.pipe_configs[i].
				 vf_output_info.res.width,
				 isp_subdev->css2_basis.pipe_configs[i].
				 vf_output_info.res.height,
				 isp_subdev->css2_basis.pipe_configs[i].
				 vf_output_info.format);
		}
	}

	return IA_CSS_SUCCESS;
pipe_err:
	for (j = i; j >= 0; j--)
		if (isp_subdev->css2_basis.pipes[j]) {
			ia_css_pipe_destroy(isp_subdev->css2_basis.pipes[j]);
			isp_subdev->css2_basis.pipes[j] = NULL;
		}

	return ret;
}
static void __dump_stream_pipe_config(struct atomisp_sub_device *isp_subdev)
{
	struct ia_css_pipe_config *p_config;
	struct ia_css_pipe_extra_config *pe_config;
	struct ia_css_stream_config *s_config;
	int i = 0;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i]) {
			p_config = &isp_subdev->css2_basis.pipe_configs[i];
			pe_config = &isp_subdev->css2_basis.
			    pipe_extra_configs[i];
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "dumping pipe[%d] config:\n", i);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.pipe_id:%d.\n", p_config->mode);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.output_info w=%d, h=%d.\n",
				p_config->output_info.res.width,
				p_config->output_info.res.height);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.vf_output_info w=%d, h=%d.\n",
				p_config->vf_output_info.res.width,
				p_config->vf_output_info.res.height);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.bayer_ds_out_res w=%d, h=%d.\n",
				p_config->bayer_ds_out_res.width,
				p_config->bayer_ds_out_res.height);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.envelope w=%d, h=%d.\n",
				p_config->dvs_envelope.width,
				p_config->dvs_envelope.height);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_config.default_capture_config.capture_mode=%d.\n",
				p_config->default_capture_config.mode);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "dumping pipe[%d] extra config:\n", i);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_raw_binning:%d.\n",
				pe_config->enable_raw_binning);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_yuv_ds:%d.\n",
				pe_config->enable_yuv_ds);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_high_speed:%d.\n",
				pe_config->enable_high_speed);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_dvs_6axis:%d.\n",
				pe_config->enable_dvs_6axis);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_reduced_pipe:%d.\n",
				pe_config->enable_reduced_pipe);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.enable_dz:%d.\n",
				pe_config->enable_dz);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.disable_vf_pp:%d.\n",
				pe_config->disable_vf_pp);
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "pipe_extra_config.disable_capture_pp:%d.\n",
				pe_config->disable_capture_pp);
		}
	}

	s_config = &isp_subdev->css2_basis.stream_config;
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "dumping stream config:\n");
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.mode=%d.\n",
		 s_config->mode);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.input_res w=%d, h=%d.\n",
		 s_config->input_res.width,
		 s_config->input_res.height);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.effective_res w=%d, h=%d.\n",
		 s_config->effective_res.width,
		 s_config->effective_res.height);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.format=%d.\n",
		 s_config->format);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.bayer_order=%d.\n",
		 s_config->bayer_order);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.2ppc=%d.\n",
		 s_config->two_pixels_per_clock);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.online=%d.\n",
		 s_config->online);
	v4l2_dbg(3, dbg_level, &atomisp_dev,
		 "stream_config.continuous=%d.\n",
		 s_config->continuous);
}
static enum ia_css_err __destroy_stream(struct atomisp_sub_device *isp_subdev,
					bool force)
{
	int i;
	enum ia_css_err ret;
	bool pipe_updated = false;
	unsigned int streaming;

	if (!isp_subdev->css2_basis.stream)
		return IA_CSS_SUCCESS;

	if (!force) {
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			if (isp_subdev->css2_basis.update_pipe[i]) {
				pipe_updated = true;
				break;
			}
	}

	if (!(force || pipe_updated))
		return IA_CSS_SUCCESS;

	streaming = atomisp_subdev_streaming_count(isp_subdev->isp);

	if (isp_subdev->css2_basis.stream_state == CSS2_STREAM_STARTED
	    && !streaming) { /* single stream mode */
		ret = ia_css_stream_stop(isp_subdev->css2_basis.stream);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev,
				 "stop stream failed.\n");
			return ret;
		}
	} else if (isp_subdev->css2_basis.stream_state == CSS2_STREAM_STARTED
	    && streaming) { /* multi stream mode */
		unsigned int cnt;
		/* CSS2.0 bug:
		 * 1: it requires no buffer in css buffer queue after
		 * stream stop
		 * 2: it does not provide the buffer queue flush
		 * machenism
		 *
		 * If there are multiple buffers in css at the time
		 * calling ia_css_stream_stop, there will buffers
		 * remaining in css, which will be dequeued next time
		 * stream start, which will causes serious mm issue
		 * since buffers are already invalid.
		 *
		 * So before calling ia_css_stream stop, we will wait
		 * until all the buffers are dequeued from css.
		 *
		 * Note that there would have big pnp impact to wait
		 * all the buffers dequeued from css, so we will only do
		 * this in multiple stream mode, since in single
		 * stream mode, no care on whether buffer remain in
		 * css, as the css will be un-initialized after it.
		 */
		/*
		 * check whether there is other stream in streaming
		 * state. if so, we need to to dequeue all the buffers
		 * in this stream, and also need to workaround to avoid
		 * sp stuck.
		 */
		mutex_unlock(&isp_subdev->isp->mutex);
		if (!wait_for_completion_timeout(&isp_subdev->buf_done,
						 2 * HZ)) {
			dev_warn(isp_subdev->isp->dev,
				 "%s: wait buf clean timeout!.\n",
				 __func__);
		}
		mutex_lock(&isp_subdev->isp->mutex);
		ret = ia_css_stream_stop(isp_subdev->css2_basis.stream);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev,
				 "stop stream failed.\n");
			return ret;
		}
		/*
		 * There is another issue if there is no buffer
		 * remaining in css after calling ia_css_stream_stop:
		 * the SP will get stuck, because it is blocking on
		 * the empty queue. So after calling
		 * ia_css_stream_stop(), we will queue only 1 buffer
		 * to css to recover sp, then wait for
		 * ia_css_stream_has_stopped(). The css will process the
		 * final buffer and exit gracefully.
		 */
		/* queue 1 buffer to css to recover sp */
		atomisp_qbuffers_to_css(isp_subdev, true);
		cnt = 50;
		/*
		 * wait for 1s for stream stop
		 * In the worst case, like ULL mode, the fps will be very low
		 * and need to wait ISP to finish procesing the frame
		 */
		while (--cnt) {
			mutex_unlock(&isp_subdev->isp->mutex);
			msleep(20);
			mutex_lock(&isp_subdev->isp->mutex);
			if (ia_css_stream_has_stopped(
					isp_subdev->css2_basis.stream))
				break;
		}
		if (!cnt)
			dev_warn(isp_subdev->isp->dev,
				 "%s: wait stream off timeout!.\n",
				 __func__);

	}

	ret = ia_css_stream_destroy(isp_subdev->css2_basis.stream);
	if (ret != IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev,
			 "destroy stream failed.\n");
		return ret;
	}
	isp_subdev->css2_basis.stream = NULL;

	return IA_CSS_SUCCESS;
}
static enum ia_css_err __create_stream(struct atomisp_sub_device *isp_subdev)
{
	int pipe_index = 0, i;
	struct ia_css_pipe *multi_pipes[IA_CSS_PIPE_ID_NUM];
	const struct ia_css_stream_config *s_config =
			&isp_subdev->css2_basis.stream_config;

	__dump_stream_pipe_config(isp_subdev);
	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipes[i])
			multi_pipes[pipe_index++] =
			    isp_subdev->css2_basis.pipes[i];
	}
	return ia_css_stream_create(s_config, pipe_index, multi_pipes,
				    &isp_subdev->css2_basis.stream);
}

enum ia_css_err ia_css_update_stream(struct atomisp_sub_device *isp_subdev)
{
	enum ia_css_err ret;

	if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy stream failed.\n");

	if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy pipe failed.\n");

	ret = __create_pipe(isp_subdev);
	if (ret != IA_CSS_SUCCESS) {
		dev_err(isp_subdev->isp->dev, "create pipe failed.\n");
		return ret;
	}

	ret = __create_stream(isp_subdev);
	if (ret != IA_CSS_SUCCESS) {
		dev_warn(isp_subdev->isp->dev, "create stream failed.\n");
		__destroy_pipes(isp_subdev, true);
		return ret;
	}

	return ret;
}

static enum ia_css_err __get_frame_info(struct atomisp_sub_device *isp_subdev,
				struct ia_css_frame_info *info,
				enum frame_info_type type)
{
	enum ia_css_err ret;
	struct ia_css_pipe_info p_info;
	unsigned int pipe_id = isp_subdev->css2_basis.curr_pipe;

	v4l2_dbg(5, dbg_level, &atomisp_dev,
		 ">%s.\n", __func__);

	if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy stream failed.\n");

	if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
		dev_warn(isp_subdev->isp->dev, "destroy pipe failed.\n");

	ret = __create_pipe(isp_subdev);
	if (ret != IA_CSS_SUCCESS)
		goto pipe_err;

	ret = __create_stream(isp_subdev);
	if (ret != IA_CSS_SUCCESS)
		goto stream_err;

	ret = ia_css_pipe_get_info(
			isp_subdev->css2_basis.pipes[pipe_id], &p_info);
	if (ret == IA_CSS_SUCCESS) {
		switch (type) {
		case VF_FRAME:
			*info = p_info.vf_output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting vf frame info.\n");
			break;
		case OUTPUT_FRAME:
			*info = p_info.output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting main frame info.\n");
			break;
		case RAW_FRAME:
			*info = p_info.raw_output_info;
			v4l2_dbg(3, dbg_level, &atomisp_dev,
				 "getting raw frame info.\n");
			break;
		default:
			info = NULL;
			v4l2_err(&atomisp_dev,
				  "wrong type for getting frame info");
		}
		v4l2_dbg(5, dbg_level, &atomisp_dev,
			 "<%s.\n", __func__);
		return IA_CSS_SUCCESS;
	}

stream_err:
	__destroy_pipes(isp_subdev, true);
pipe_err:

	return ret;
}

enum ia_css_err ia_css_preview_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format,
			   IA_CSS_PIPE_ID_PREVIEW);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_preview_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_PREVIEW;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_capture_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format,
			   IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_capture_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_vf_output(isp_subdev, width, height, format,
			      IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	__get_frame_info(isp_subdev, info, VF_FRAME);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_configure_output(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_output(isp_subdev, width, height, format,
			   IA_CSS_PIPE_ID_VIDEO);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_get_output_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_VIDEO;
	return __get_frame_info(isp_subdev, info, OUTPUT_FRAME);
}

enum ia_css_err ia_css_video_configure_viewfinder(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format)
{
	__configure_vf_output(isp_subdev, width, height, format,
			      IA_CSS_PIPE_ID_VIDEO);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_video_get_viewfinder_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_VIDEO;
	return __get_frame_info(isp_subdev, info, VF_FRAME);
}

enum ia_css_err ia_css_preview_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height)
{
	__configure_pp_input(isp_subdev, width, height, IA_CSS_PIPE_ID_PREVIEW);
	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_capture_configure_pp_input(
	struct atomisp_sub_device *isp_subdev,
	unsigned int width,
	unsigned int height)
{
	__configure_pp_input(isp_subdev, width, height, IA_CSS_PIPE_ID_CAPTURE);
	return IA_CSS_SUCCESS;
}
void
ia_css_capture_set_mode(struct atomisp_sub_device *isp_subdev,
			enum ia_css_capture_mode mode)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	if (isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_CAPTURE].
	    default_capture_config.mode != mode) {
		isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_CAPTURE].
		    default_capture_config.mode = mode;
		isp_subdev->css2_basis.update_pipe[IA_CSS_PIPE_ID_CAPTURE] =
		    true;
	}
}

void
ia_css_capture_enable_online(struct atomisp_sub_device *isp_subdev,
			     bool enable)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	if (isp_subdev->css2_basis.stream_config.online != enable) {
		isp_subdev->css2_basis.stream_config.online = enable;
		isp_subdev->css2_basis.update_pipe[IA_CSS_PIPE_ID_CAPTURE] =
		    true;
	}
}

void
ia_css_input_set_two_pixels_per_clock(struct atomisp_sub_device *isp_subdev,
					   bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.two_pixels_per_clock !=
	    enable) {
		isp_subdev->css2_basis.stream_config.two_pixels_per_clock =
		    enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}

void
ia_css_enable_raw_binning(struct atomisp_sub_device *isp_subdev,
			     bool enable)
{
	int i;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
		if (isp_subdev->css2_basis.pipe_extra_configs[i].
		    enable_raw_binning != enable) {
			isp_subdev->css2_basis.pipe_extra_configs[i].
			    enable_raw_binning = enable;
			isp_subdev->css2_basis.update_pipe[i] = true;
		}
	}
}

void ia_css_enable_continuous(struct atomisp_sub_device *isp_subdev,
				  bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.continuous != enable) {
		isp_subdev->css2_basis.stream_config.continuous = enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}

void ia_css_preview_enable_online(struct atomisp_sub_device *isp_subdev,
				  bool enable)
{
	int i;

	if (isp_subdev->css2_basis.stream_config.online != enable) {
		isp_subdev->css2_basis.stream_config.online = enable;
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
			isp_subdev->css2_basis.update_pipe[i] = true;
	}
}

enum ia_css_err ia_css_capture_get_output_raw_frame_info(
	struct atomisp_sub_device *isp_subdev,
	struct ia_css_frame_info *info)
{
	isp_subdev->css2_basis.curr_pipe = IA_CSS_PIPE_ID_CAPTURE;
	return __get_frame_info(isp_subdev, info, RAW_FRAME);
}

void atomisp_sh_css_mmu_set_page_table_base_index(unsigned int base_index)
{
#ifndef ATOMISP_CSS2
	sh_css_mmu_set_page_table_base_index((hrt_data) base_index);
#endif
}
enum ia_css_err ia_css_start(struct atomisp_sub_device *isp_subdev,
			     bool in_reset)
{
	enum ia_css_err ret;

	if (in_reset) {
		if (__destroy_stream(isp_subdev, true) != IA_CSS_SUCCESS)
			dev_warn(isp_subdev->isp->dev, "destroy stream failed.\n");

		if (__destroy_pipes(isp_subdev, true) != IA_CSS_SUCCESS)
			dev_warn(isp_subdev->isp->dev, "destroy pipe failed.\n");

		ret = __create_pipe(isp_subdev);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev, "create pipe error.\n");
			goto pipe_err;
		}
		ret = __create_stream(isp_subdev);
		if (ret != IA_CSS_SUCCESS) {
			v4l2_err(&atomisp_dev, "create stream error.\n");
			goto stream_err;
		}
	}
	/*
	 * SP can only be started one time
	 * if atomisp_subdev_streaming_count() tell there aleady has some subdev
	 * at streamming, then SP should already be started previously, so
	 * need to skip start sp procedure
	 */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)) {
		dev_dbg(isp_subdev->isp->dev, "skip start sp.\n");
	} else {
		ret = ia_css_start_sp();
		if (ret != IA_CSS_SUCCESS) {
			dev_err(isp_subdev->isp->dev, "start sp error.\n");
			goto start_err;
		}
	}
	ret = ia_css_stream_start(isp_subdev->css2_basis.stream);
	if (ret != IA_CSS_SUCCESS) {
		dev_err(isp_subdev->isp->dev, "stream start error.\n");
		goto start_err;
	}

	isp_subdev->css2_basis.stream_state = CSS2_STREAM_STARTED;
	return IA_CSS_SUCCESS;

start_err:
	__destroy_stream(isp_subdev, true);
stream_err:
	__destroy_pipes(isp_subdev, true);

	/* css 2.0 API limitation: ia_css_stop_sp() could be only called after
	 * destroy all pipes
	 * */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)) {
		dev_dbg(isp_subdev->isp->dev, "can not stop sp.\n");
	} else if (ia_css_isp_has_started())
		if (ia_css_stop_sp() != IA_CSS_SUCCESS)
			dev_warn(isp_subdev->isp->dev, "stop sp failed.\n");
pipe_err:
	return ret;
}
enum ia_css_err ia_css_stop(struct atomisp_sub_device *isp_subdev,
			    bool in_reset)
{
	int i = 0;
	enum ia_css_err ret = IA_CSS_SUCCESS;

	ret = __destroy_stream(isp_subdev, true);
	if (ret != IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev, "destroy stream failed.\n");
		goto err;
	}
	ret = __destroy_pipes(isp_subdev, true);
	if (ret != IA_CSS_SUCCESS) {
		v4l2_err(&atomisp_dev, "destroy pipes failed.\n");
		goto err;
	}
	/*
	 * SP can not be stopped if other streams are still running
	 * if atomisp_subdev_streaming_count() tell there aleady has some subdev
	 * at streamming, then SP can not be stopped, so
	 * need to skip start sp procedure
	 */
	if (atomisp_subdev_streaming_count(isp_subdev->isp)) {
		v4l2_info(&atomisp_dev, "skip stop sp.\n");
	} else if (ia_css_isp_has_started()) {
		if (ia_css_stop_sp() != IA_CSS_SUCCESS) {
			dev_dbg(isp_subdev->isp->dev, "stop sp failed.\n");
			goto err;
		}
	}

	isp_subdev->css2_basis.stream_state = CSS2_STREAM_STOPPED;

	/* FIXME: Current code would cause streamon, then streamoff failed
	 * If configs are not cleared, it would create wrong pipe/stream in
	 * set format. No better solution has found yet.*/
	if (!in_reset) {
		for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++) {
			ia_css_pipe_config_defaults(&isp_subdev->css2_basis.
						    pipe_configs[i]);
			ia_css_pipe_extra_config_defaults(
						&isp_subdev->css2_basis.
						pipe_extra_configs[i]);
		}
		ia_css_stream_config_defaults(&isp_subdev->css2_basis.
					      stream_config);
	}

	return IA_CSS_SUCCESS;

err:
	v4l2_err(&atomisp_dev, "stop css fatal error. cannot recover\n");
	return ret;
}

void
ia_css_disable_vf_pp(struct atomisp_sub_device *isp_subdev, bool disable)
{
	int i;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
		isp_subdev->css2_basis.pipe_extra_configs[i].disable_vf_pp
							= disable;
}

void
ia_css_enable_high_speed(struct atomisp_sub_device *isp_subdev, bool enable)
{
	int i;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
		isp_subdev->css2_basis.pipe_extra_configs[i].enable_high_speed
							= enable;
}

void
ia_css_enbale_dz(struct atomisp_sub_device *isp_subdev, bool enable)
{
	int i;

	for (i = 0; i < IA_CSS_PIPE_ID_NUM; i++)
		isp_subdev->css2_basis.pipe_extra_configs[i].enable_dz = enable;
}

void
ia_css_video_set_dis_envelope(struct atomisp_sub_device *isp_subdev,
			      unsigned int dvs_w,
			      unsigned int dvs_h)
{
	unsigned int pipe_id = isp_subdev->css2_basis.curr_pipe;

	isp_subdev->css2_basis.pipe_configs[pipe_id].dvs_envelope.width = dvs_w;
	isp_subdev->css2_basis.pipe_configs[pipe_id].dvs_envelope.height =
	    dvs_h;
}
void
ia_css_input_set_effective_resolution(struct atomisp_sub_device *isp_subdev,
			unsigned int width, unsigned int height)
{
	isp_subdev->css2_basis.stream_config.effective_res.width = width;
	isp_subdev->css2_basis.stream_config.effective_res.height = height;
}
void
ia_css_input_set_resolution(struct atomisp_sub_device *isp_subdev,
			unsigned int width, unsigned int height)
{
	isp_subdev->css2_basis.stream_config.input_res.width = width;
	isp_subdev->css2_basis.stream_config.input_res.height = height;
}

void
ia_css_input_set_binning_factor(struct atomisp_sub_device *isp_subdev,
				unsigned int binning)
{
	isp_subdev->css2_basis.stream_config.sensor_binning_factor = binning;
}

void
ia_css_input_set_bayer_order(struct atomisp_sub_device *isp_subdev,
			     enum ia_css_bayer_order order)
{
	isp_subdev->css2_basis.stream_config.bayer_order = order;
}

void
ia_css_input_set_format(struct atomisp_sub_device *isp_subdev,
			     enum ia_css_stream_format format)
{
	isp_subdev->css2_basis.stream_config.format = format;
}

void
ia_css_input_configure_port(struct atomisp_sub_device *isp_subdev,
			    const mipi_port_ID_t port,
			    const unsigned int	 num_lanes,
			    const unsigned int	 timeout)
{
	isp_subdev->css2_basis.stream_config.source.port.port = port;
	isp_subdev->css2_basis.stream_config.source.port.num_lanes = num_lanes;
	isp_subdev->css2_basis.stream_config.source.port.timeout = timeout;
}
void
ia_css_input_set_mode(struct atomisp_sub_device *isp_subdev,
		      enum ia_css_input_mode mode)
{
	isp_subdev->css2_basis.stream_config.mode = mode;
	if (mode == IA_CSS_INPUT_MODE_BUFFERED_SENSOR) {
		if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
			ia_css_mipi_frame_specify(0x80000, false);
		else
			ia_css_mipi_frame_specify(0x60000, false);
	}
}
