/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
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

#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf-vmalloc.h>

#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_compat.h"
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp_subdev.h"
#include "atomisp-regs.h"

#include "hrt/hive_isp_css_mm_hrt.h"
#include "hrt/hive_isp_css_custom_host_hrt.h"

#include "sh_css_debug.h"
#include "host/mmu_local.h"
#include "device_access/device_access.h"
#include "memory_access/memory_access.h"

#include "atomisp_acc.h"
#include "atomisp_fw.h"

#define ISP_LEFT_PAD			128	/* equal to 2*NWAY */
#define CSS_DTRACE_VERBOSITY_LEVEL	5	/* Controls trace verbosity */

/*
 * input image data, and current frame resolution for test
 */
#define	ISP_PARAM_MMAP_OFFSET	0xfffff000

#define MAGIC_CHECK(is, should)	\
	if (unlikely((is) != (should))) { \
		printk(KERN_ERR "magic mismatch: %x (expected %x)\n", \
			is, should); \
		BUG(); \
	}

/*
 * Videobuf ops
 */
static int atomisp_buf_setup(struct videobuf_queue *vq, unsigned int *count,
			     unsigned int *size)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	*size = pipe->pix.sizeimage;

	return 0;
}

static int atomisp_buf_prepare(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb,
			       enum v4l2_field field)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	vb->size = pipe->pix.sizeimage;
	vb->width = pipe->pix.width;
	vb->height = pipe->pix.height;
	vb->field = field;
	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

int atomisp_q_video_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			     struct atomisp_video_pipe *pipe,
			     enum ia_css_buffer_type css_buf_type,
			     enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct videobuf_buffer *vb;
	struct videobuf_vmalloc_memory *vm_mem;
	unsigned long irqflags;
	int err;

	while (pipe->buffers_in_css < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer css_buf = {0};
		spin_lock_irqsave(&pipe->irq_lock, irqflags);
		if (list_empty(&pipe->activeq)) {
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
			return -EINVAL;
		}
		vb = list_entry(pipe->activeq.next,
				struct videobuf_buffer, queue);
		list_del_init(&vb->queue);
		vb->state = VIDEOBUF_ACTIVE;
		spin_unlock_irqrestore(&pipe->irq_lock, irqflags);

		vm_mem = vb->priv;
		css_buf.type = css_buf_type;
		css_buf.data.frame = vm_mem->vaddr;
		v4l2_dbg(5, dbg_level, &atomisp_dev,
			 "queue video buffer[%d] type[%d] into css pipe[%d].\n",
			 vb->i, css_buf_type, css_pipe_id);

		err = ia_css_pipe_enqueue_buffer(isp_subdev->css2_basis.
						 pipes[css_pipe_id],
						 &css_buf);
		if (err) {
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			list_add_tail(&vb->queue, &pipe->activeq);
			vb->state = VIDEOBUF_QUEUED;
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
			dev_err(isp_subdev->isp->dev, "%s, css q fails: %d\n",
					__func__, err);
			return -EINVAL;
		}
		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */
		if (streamoff)
			return 0;

		pipe->buffers_in_css++;
		vb = NULL;
	}
	return 0;
}

int atomisp_q_s3a_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			   enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct atomisp_s3a_buf *s3a_buf;
	int pipe_index = 0;

	if (list_empty(&isp_subdev->s3a_stats)) {
		WARN(1, "%s: No s3a buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp_subdev->s3a_bufs_in_css[css_pipe_id] < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer buffer = {0};
		s3a_buf = list_entry(isp_subdev->s3a_stats.next,
				struct atomisp_s3a_buf, list);
		list_move_tail(&s3a_buf->list, &isp_subdev->s3a_stats);

		buffer.type = IA_CSS_BUFFER_TYPE_3A_STATISTICS;
		buffer.data.stats_3a = s3a_buf->s3a_stat;
		if (ia_css_pipe_enqueue_buffer(
				isp_subdev->css2_basis.pipes[css_pipe_id],
					&buffer)) {
			dev_err(isp_subdev->isp->dev, "failed to q s3a stat buffer\n");
			return -EINVAL;
		}
		v4l2_dbg(5, dbg_level, &atomisp_dev, "q s3a buffer into css pipe[%d].\n", pipe_index);
		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */
		if (streamoff)
			return 0;

		isp_subdev->s3a_bufs_in_css[css_pipe_id]++;
	}
	return 0;
}

int atomisp_q_dis_buffers_to_css(struct atomisp_sub_device *isp_subdev,
			   enum ia_css_pipe_id css_pipe_id, bool streamoff)
{
	struct atomisp_dvs_buf *dvs_buf;

	if (list_empty(&isp_subdev->dvs_stats)) {
		WARN(1, "%s: No dis buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp_subdev->dis_bufs_in_css < ATOMISP_CSS_Q_DEPTH) {
		struct ia_css_buffer buffer = {0};
		dvs_buf = list_entry(isp_subdev->dvs_stats.next,
				struct atomisp_dvs_buf, list);
		list_move_tail(&dvs_buf->list, &isp_subdev->dvs_stats);

		buffer.type = IA_CSS_BUFFER_TYPE_DIS_STATISTICS;
		buffer.data.stats_dvs = dvs_buf->dvs_stat;
		if (ia_css_pipe_enqueue_buffer(isp_subdev->css2_basis.
					       pipes[css_pipe_id],
					&buffer)) {
			dev_err(isp_subdev->isp->dev, "failed to q dvs stat buffer\n");
			return -EINVAL;
		}
		/*
		 * CSS2.0 Issue: after stream off, need to queue 1 buffer to CSS
		 * to recover SP.
		 * So if it is in stream off state, only need to queue 1 buffer
		 */
		if (streamoff)
			return 0;

		isp_subdev->dis_bufs_in_css++;
		dvs_buf = NULL;
	}
	return 0;
}

static int atomisp_get_css_buf_type(struct atomisp_sub_device *isp_subdev,
				    uint16_t source_pad)
{
	if (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE ||
	    (source_pad == ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW &&
	     isp_subdev->run_mode->val != ATOMISP_RUN_MODE_VIDEO))
		return IA_CSS_BUFFER_TYPE_OUTPUT_FRAME;
	else
		return IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME;
}

/* queue all available buffers to css */
int atomisp_qbuffers_to_css(struct atomisp_sub_device *isp_subdev,
			    bool streamoff)
{
	enum ia_css_buffer_type buf_type;
	enum ia_css_pipe_id css_capture_pipe_id = IA_CSS_PIPE_ID_NUM;
	enum ia_css_pipe_id css_preview_pipe_id = IA_CSS_PIPE_ID_NUM;
	struct atomisp_video_pipe *capture_pipe = NULL;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *preview_pipe = NULL;

	if (!isp_subdev->enable_vfpp->val) {
		preview_pipe = &isp_subdev->video_out_capture;
		css_preview_pipe_id = IA_CSS_PIPE_ID_VIDEO;
	} else if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		capture_pipe = &isp_subdev->video_out_capture;
		preview_pipe = &isp_subdev->video_out_preview;
		css_capture_pipe_id = IA_CSS_PIPE_ID_VIDEO;
		css_preview_pipe_id = IA_CSS_PIPE_ID_VIDEO;
	} else if (isp_subdev->continuous_mode->val) {
		capture_pipe = &isp_subdev->video_out_capture;
		vf_pipe = &isp_subdev->video_out_vf;
		preview_pipe = &isp_subdev->video_out_preview;
		css_capture_pipe_id = IA_CSS_PIPE_ID_CAPTURE;
		css_preview_pipe_id = IA_CSS_PIPE_ID_PREVIEW;
	} else if (isp_subdev->run_mode->val == ATOMISP_RUN_MODE_PREVIEW) {
		preview_pipe = &isp_subdev->video_out_preview;
		css_preview_pipe_id = IA_CSS_PIPE_ID_PREVIEW;
	} else {
		/* ATOMISP_RUN_MODE_STILL_CAPTURE */
		capture_pipe = &isp_subdev->video_out_capture;
		if (!atomisp_is_mbuscode_raw(
			    isp_subdev->
			    fmt[isp_subdev->capture_pad].fmt.code))
			vf_pipe = &isp_subdev->video_out_vf;
		css_capture_pipe_id = IA_CSS_PIPE_ID_CAPTURE;
	}

	if (capture_pipe) {
		buf_type = atomisp_get_css_buf_type(
			isp_subdev,
			atomisp_subdev_source_pad(&capture_pipe->vdev));
		atomisp_q_video_buffers_to_css(isp_subdev, capture_pipe,
					       buf_type, css_capture_pipe_id,
					       streamoff);
	}

	if (vf_pipe) {
		buf_type = atomisp_get_css_buf_type(
			isp_subdev, atomisp_subdev_source_pad(&vf_pipe->vdev));
		atomisp_q_video_buffers_to_css(isp_subdev, vf_pipe, buf_type,
					 css_capture_pipe_id, streamoff);
	}

	if (preview_pipe) {
		buf_type = atomisp_get_css_buf_type(
			isp_subdev,
			atomisp_subdev_source_pad(&preview_pipe->vdev));
		atomisp_q_video_buffers_to_css(isp_subdev, preview_pipe,
					       buf_type, css_preview_pipe_id,
					       streamoff);
	}

	if (isp_subdev->params.curr_grid_info.s3a_grid.enable) {
		if (css_capture_pipe_id < IA_CSS_PIPE_ID_NUM)
			atomisp_q_s3a_buffers_to_css(isp_subdev,
						     css_capture_pipe_id,
						     streamoff);
		if (css_preview_pipe_id != css_capture_pipe_id &&
		   css_preview_pipe_id < IA_CSS_PIPE_ID_NUM)
			atomisp_q_s3a_buffers_to_css(isp_subdev,
						     css_preview_pipe_id,
						     streamoff);
	}

	if (isp_subdev->params.curr_grid_info.dvs_grid.enable)
		atomisp_q_dis_buffers_to_css(isp_subdev, css_capture_pipe_id,
					     streamoff);

	return 0;
}

static void atomisp_buf_queue(struct videobuf_queue *vq,
			      struct videobuf_buffer *vb)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	list_add_tail(&vb->queue, &pipe->activeq);
	vb->state = VIDEOBUF_QUEUED;
}

static void atomisp_buf_release(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	vb->state = VIDEOBUF_NEEDS_INIT;
	atomisp_videobuf_free_buf(vb);
}

static int atomisp_buf_setup_output(struct videobuf_queue *vq,
				    unsigned int *count,
				    unsigned int *size)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	*size = pipe->pix.sizeimage;

	return 0;
}

static int atomisp_buf_prepare_output(struct videobuf_queue *vq,
				      struct videobuf_buffer *vb,
				      enum v4l2_field field)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	vb->size = pipe->pix.sizeimage;
	vb->width = pipe->pix.width;
	vb->height = pipe->pix.height;
	vb->field = field;
	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

static void atomisp_buf_queue_output(struct videobuf_queue *vq,
				     struct videobuf_buffer *vb)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	list_add_tail(&vb->queue, &pipe->activeq_out);
	vb->state = VIDEOBUF_QUEUED;
}

static void atomisp_buf_release_output(struct videobuf_queue *vq,
				       struct videobuf_buffer *vb)
{
	videobuf_vmalloc_free(vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops videobuf_qops = {
	.buf_setup	= atomisp_buf_setup,
	.buf_prepare	= atomisp_buf_prepare,
	.buf_queue	= atomisp_buf_queue,
	.buf_release	= atomisp_buf_release,
};

static struct videobuf_queue_ops videobuf_qops_output = {
	.buf_setup	= atomisp_buf_setup_output,
	.buf_prepare	= atomisp_buf_prepare_output,
	.buf_queue	= atomisp_buf_queue_output,
	.buf_release	= atomisp_buf_release_output,
};

static int atomisp_init_pipe(struct atomisp_video_pipe *pipe)
{
	/* init locks */
	spin_lock_init(&pipe->irq_lock);

	videobuf_queue_vmalloc_init(&pipe->capq, &videobuf_qops, NULL,
				    &pipe->irq_lock,
				    V4L2_BUF_TYPE_VIDEO_CAPTURE,
				    V4L2_FIELD_NONE,
				    sizeof(struct atomisp_buffer),
				    pipe,
				    NULL);	/* ext_lock: NULL */

	videobuf_queue_vmalloc_init(&pipe->outq,
				    &videobuf_qops_output, NULL,
				    &pipe->irq_lock,
				    V4L2_BUF_TYPE_VIDEO_OUTPUT,
				    V4L2_FIELD_NONE,
				    sizeof(struct atomisp_buffer),
				    pipe,
				    NULL);	/* ext_lock: NULL */

	INIT_LIST_HEAD(&pipe->activeq);
	INIT_LIST_HEAD(&pipe->activeq_out);

	return 0;
}

int atomisp_dev_init_struct(struct atomisp_device *isp)
{
	int i;

	if (isp == NULL)
		return -EINVAL;

	isp->sw_contex.file_input = 0;
	isp->need_gfx_throttle = true;
	isp->isp_fatal_error = false;
	isp->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;

	for (i = 0; i < ATOM_ISP_MAX_INPUTS; i++)
		isp->inputs[i].used_by = -1;
	/*
	 * For Merrifield, frequency is scalable.
	 * After boot-up, the default frequency is 200MHz.
	 * For Medfield/Clovertrail, all running at 320MHz
	 */
	if (IS_ISP2400)
		isp->sw_contex.running_freq = ISP_FREQ_200MHZ;
	else
		isp->sw_contex.running_freq = ISP_FREQ_320MHZ;

	return 0;
}

int atomisp_subdev_init_struct(struct atomisp_sub_device *isp_subdev)
{
	unsigned int i = 0;

	v4l2_ctrl_s_ctrl(isp_subdev->run_mode,
			 ATOMISP_RUN_MODE_STILL_CAPTURE);
	isp_subdev->params.color_effect = V4L2_COLORFX_NONE;
	isp_subdev->params.bad_pixel_en = 1;
	isp_subdev->params.gdc_cac_en = 0;
	isp_subdev->params.video_dis_en = 0;
	isp_subdev->params.sc_en = 0;
	isp_subdev->params.fpn_en = 0;
	isp_subdev->params.xnr_en = 0;
	isp_subdev->params.false_color = 0;
	isp_subdev->params.online_process = 1;
	isp_subdev->params.yuv_ds_en = 0;
	isp_subdev->params.offline_parm.num_captures = 1;
	isp_subdev->params.offline_parm.skip_frames = 0;
	isp_subdev->params.offline_parm.offset = 0;

	isp_subdev->css2_basis.stream = NULL;
	for (i = 0; i < IA_CSS_PIPE_MODE_NUM; i++) {
		isp_subdev->css2_basis.pipes[i] = NULL;
		isp_subdev->css2_basis.update_pipe[i] = false;
		ia_css_pipe_config_defaults(&isp_subdev->css2_basis.
					    pipe_configs[i]);
		ia_css_pipe_extra_config_defaults(
				&isp_subdev->css2_basis.pipe_extra_configs[i]);
	}
	ia_css_stream_config_defaults(&isp_subdev->css2_basis.stream_config);
	isp_subdev->css2_basis.curr_pipe = 0;

	/* Add for channel */
	if (isp_subdev->isp->inputs[0].camera)
		isp_subdev->input_curr = 0;

	init_completion(&isp_subdev->buf_done);

	return 0;
}

static void *my_kernel_malloc(size_t bytes, bool zero_mem)
{
	void *ptr = atomisp_kernel_malloc(bytes);

	if (ptr && zero_mem)
		memset(ptr, 0, bytes);

	return ptr;
}
/*
 * Return the number of concurrent running streams.
 */
int atomisp_subdev_streaming_count(struct atomisp_device *isp)
{
	int i, sum;

	for (i = 0, sum = 0; i < isp->num_of_streams; i++)
		sum += isp->isp_subdev[i].streaming ==
		    ATOMISP_DEVICE_STREAMING_ENABLED;

	return sum;
}
/*
 * file operation functions
 */
unsigned int atomisp_subdev_users(struct atomisp_sub_device *isp_subdev)
{
	return isp_subdev->video_out_preview.users +
	       isp_subdev->video_out_vf.users +
	       isp_subdev->video_out_capture.users +
	       isp_subdev->video_in.users;
}

unsigned int atomisp_dev_users(struct atomisp_device *isp)
{
	unsigned int i, sum = 0;

	for (i = 0; i < isp->num_of_streams; i++)
		sum += atomisp_subdev_users(&isp->isp_subdev[i]);

	return sum;
}

int atomisp_css2_dbg_print(const char *fmt, va_list args)
{
	if (dbg_level > 5)
		vprintk(fmt, args);
	return 0;
}

int atomisp_css2_err_print(const char *fmt, va_list args)
{
	vprintk(fmt, args);
	return 0;
}


struct ia_css_env css_env = {
	.cpu_mem_env.alloc = my_kernel_malloc,
	.cpu_mem_env.free = atomisp_kernel_free,

	.css_mem_env.alloc = atomisp_css2_mm_alloc,
	.css_mem_env.free = atomisp_css2_mm_free,
	.css_mem_env.load = atomisp_css2_mm_load,
	.css_mem_env.store = atomisp_css2_mm_store,
	.css_mem_env.set = atomisp_css2_mm_set,
	.css_mem_env.mmap = atomisp_css2_mm_mmap,

	.hw_access_env.store_8 = atomisp_css2_hw_store_8,
	.hw_access_env.store_16 = atomisp_css2_hw_store_16,
	.hw_access_env.store_32 = atomisp_css2_hw_store_32,

	.hw_access_env.load_8 = atomisp_css2_hw_load_8,
	.hw_access_env.load_16 = atomisp_css2_hw_load_16,
	.hw_access_env.load_32 = atomisp_css2_hw_load_32,

	.hw_access_env.load = atomisp_css2_hw_load,
	.hw_access_env.store = atomisp_css2_hw_store,

	.print_env.debug_print = atomisp_css2_dbg_print,
	.print_env.error_print = atomisp_css2_err_print,
};

static int atomisp_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret, mmu_base_addr;
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);


	dev_dbg(isp->dev, "open device %s\n", vdev->name);

	mutex_lock(&isp->mutex);

	if (!isp->input_cnt) {
		dev_err(isp->dev, "no camera attached\n");
		ret = -EINVAL;
		goto error;
	}

	if (pipe->users)
		goto done;

	ret = atomisp_init_pipe(pipe);
	if (ret)
		goto error;

	/* Init subdev part */
	if (atomisp_subdev_users(isp_subdev))
		goto done;

	atomisp_subdev_init_struct(isp_subdev);

	/* Init ISP Global part */
	if (atomisp_dev_users(isp))
		goto done;

	hrt_isp_css_mm_init();
	mmu_base_addr = hrt_isp_get_mmu_base_address();
	if (mmu_base_addr < 0) {
		hrt_isp_css_mm_clear();
		goto error;
	}

	/* runtime power management, turn on ISP */
	ret = pm_runtime_get_sync(vdev->v4l2_dev->dev);
	if (ret < 0) {
		dev_err(isp->dev, "Failed to power on device\n");
		goto error;
	}

	isp->css_fw.data = (void *)isp->firmware->data;
	isp->css_fw.bytes = isp->firmware->size;

	/* Init ISP */
	if (ia_css_init(&css_env,
		&isp->css_fw,
		(uint32_t)mmu_base_addr,
		IA_CSS_IRQ_TYPE_PULSE)) {
		ret = -EINVAL;
		goto css_init_failed;
	}

	atomisp_dev_init_struct(isp);

done:
	pipe->users++;
	mutex_unlock(&isp->mutex);
	return 0;

css_init_failed:
	dev_err(isp->dev, "css init failed --- bad firmware?\n");
error:
	pm_runtime_put(vdev->v4l2_dev->dev);
	mutex_unlock(&isp->mutex);
	return ret;
}

static int atomisp_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct v4l2_requestbuffers req;
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);

	dev_dbg(isp->dev, "release device %s\n", vdev->name);

	req.count = 0;
	if (isp == NULL)
		return -EBADF;

	mutex_lock(&isp->streamoff_mutex);
	mutex_lock(&isp->mutex);

	pipe->users--;

	if (pipe->capq.streaming &&
	    __atomisp_streamoff(file, NULL, V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		dev_err(isp->dev,
			"atomisp_streamoff failed on release, driver bug");
		goto done;
	}

	if (pipe->users)
		goto done;

	if (__atomisp_reqbufs(file, NULL, &req)) {
		dev_err(isp->dev,
			"atomisp_reqbufs failed on release, driver bug");
		goto done;
	}

	if (pipe->outq.bufs[0]) {
		mutex_lock(&pipe->outq.vb_lock);
		videobuf_queue_cancel(&pipe->outq);
		mutex_unlock(&pipe->outq.vb_lock);
	}

	/*
	 * A little trick here:
	 * file injection input resolution is recorded in the sink pad,
	 * therefore can not be cleared when releaseing one device node.
	 * The sink pad setting can only be cleared when all device nodes
	 * get released.
	 */
	if (!isp->sw_contex.file_input && isp_subdev->fmt_auto->val) {
		struct v4l2_mbus_framefmt isp_sink_fmt = { 0 };
		atomisp_subdev_set_ffmt(
			&isp_subdev->subdev, NULL,
			V4L2_SUBDEV_FORMAT_ACTIVE, ATOMISP_SUBDEV_PAD_SINK,
			&isp_sink_fmt);
	}

	if (atomisp_subdev_users(isp_subdev))
		goto done;

	/* clear the sink pad for file input */
	if (isp->sw_contex.file_input && isp_subdev->fmt_auto->val) {
		struct v4l2_mbus_framefmt isp_sink_fmt = { 0 };
		atomisp_subdev_set_ffmt(&isp_subdev->subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &isp_sink_fmt);
	}

	atomisp_ISP_parameters_clean_up(isp_subdev, &isp_subdev->params.config);
	isp_subdev->params.css_update_params_needed = false;

	atomisp_free_3a_dvs_buffers(isp_subdev);
	atomisp_free_internal_buffers(isp_subdev);

	if (atomisp_dev_users(isp))
		goto done;

	del_timer_sync(&isp->wdt);
	atomisp_acc_release(isp);
	atomisp_free_all_shading_tables(isp);
	ia_css_uninit();
	hrt_isp_css_mm_clear();

	/*
	 * FIXME! Workaround due to HW limitation
	 * Cameras are using shared V1.8 and V2.8 power pins. So we can not
	 * power off one sensor when one stream is closed, while the other
	 * stream is still using other camera sensors.
	 * So power off all sensors in the finall unitialization step.
	 *
	 * Currently there don't have VRF(Voltage regulator framework) for
	 * Merrifield/BTY, will add workaround in platform_camera.c for sensor
	 * power control, and remove the WA when VRF is ready.
	 *
	 * The following WA in driver will not be in atomisp2, but first let
	 * it at atomisp3 at present.
	 */
	v4l2_subdev_call(isp->inputs[0].camera, core, s_power, 0);
	v4l2_subdev_call(isp->inputs[1].camera, core, s_power, 0);

	if (pm_runtime_put_sync(vdev->v4l2_dev->dev) < 0)
		dev_err(isp->dev, "Failed to power off device\n");
done:
	mutex_unlock(&isp->mutex);
	mutex_unlock(&isp->streamoff_mutex);

	return 0;
}

/*
 * Memory help functions for image frame and private parameters
 */
static int do_isp_mm_remap(struct atomisp_device *isp,
			   struct vm_area_struct *vma,
			   void *isp_virt, u32 host_virt, u32 pgnr)
{
	u32 pfn;

	while (pgnr) {
		pfn = hmm_virt_to_phys(isp_virt) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, host_virt, pfn,
				    PAGE_SIZE, PAGE_SHARED)) {
			dev_err(isp->dev, "remap_pfn_range err.\n");
			return -EAGAIN;
		}

		isp_virt += PAGE_SIZE;
		host_virt += PAGE_SIZE;
		pgnr--;
	}

	return 0;
}

static int frame_mmap(struct atomisp_device *isp,
	const struct ia_css_frame *frame, struct vm_area_struct *vma)
{
	void *isp_virt;
	u32 host_virt;
	u32 pgnr;

	if (!frame) {
		dev_err(isp->dev, "%s: NULL frame pointer.\n", __func__);
		return -EINVAL;
	}

	host_virt = vma->vm_start;
	isp_virt = (void *)frame->data;
	atomisp_get_frame_pgnr(isp, frame, &pgnr);

	if (do_isp_mm_remap(isp, vma, isp_virt, host_virt, pgnr))
		return -EAGAIN;

	return 0;
}

int atomisp_videobuf_mmap_mapper(struct videobuf_queue *q,
	struct vm_area_struct *vma)
{
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL, i;
	struct atomisp_device *isp =
		((struct atomisp_video_pipe *)(q->priv_data))->isp;
	struct videobuf_vmalloc_memory *vm_mem;
	struct videobuf_mapping *map;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);
	if (!(vma->vm_flags & VM_WRITE) || !(vma->vm_flags & VM_SHARED)) {
		dev_err(isp->dev, "map appl bug: PROT_WRITE and MAP_SHARED "
				  "are required\n");
		return -EINVAL;
	}

	mutex_lock(&q->vb_lock);
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		struct videobuf_buffer *buf = q->bufs[i];
		if (buf == NULL)
			continue;

		map = kzalloc(sizeof(struct videobuf_mapping), GFP_KERNEL);
		if (map == NULL) {
			mutex_unlock(&q->vb_lock);
			return -ENOMEM;
		}

		buf->map = map;
		map->q = q;

		buf->baddr = vma->vm_start;

		if (buf && buf->memory == V4L2_MEMORY_MMAP &&
		    buf->boff == offset) {
			vm_mem = buf->priv;
			ret = frame_mmap(isp, vm_mem->vaddr, vma);
			vma->vm_flags |= VM_DONTEXPAND | VM_RESERVED;
			break;
		}
	}
	mutex_unlock(&q->vb_lock);

	return ret;
}

/* The input frame contains left and right padding that need to be removed.
 * There is always ISP_LEFT_PAD padding on the left side.
 * There is also padding on the right (padded_width - width).
 */
static int remove_pad_from_frame(struct atomisp_device *isp,
		struct ia_css_frame *in_frame, __u32 width, __u32 height)
{
	unsigned int i;
	unsigned short *buffer;
	int ret = 0;
	unsigned short *load = (unsigned short *)in_frame->data;
	unsigned short *store = load;

	buffer = kmalloc(width*sizeof(*load), GFP_KERNEL);
	if (!buffer) {
		dev_err(isp->dev, "out of memory.\n");
		return -ENOMEM;
	}

//#define ISP_LEFT_PAD			128	/* equal to 2*NWAY */
	load += ISP_LEFT_PAD;
	for (i = 0; i < height; i++) {
		ret = hrt_isp_css_mm_load(load, buffer, width*sizeof(*load));
		if (ret < 0)
			goto remove_pad_error;

		ret = hrt_isp_css_mm_store(store, buffer, width*sizeof(*store));
		if (ret < 0)
			goto remove_pad_error;

		load  += in_frame->info.padded_width;
		store += width;
	}

remove_pad_error:
	kfree(buffer);
	return ret;
}

static int atomisp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	struct ia_css_frame *raw_virt_addr;
	u32 start = vma->vm_start;
	u32 end = vma->vm_end;
	u32 size = end - start;
	u32 origin_size, new_size;
	int ret;
	struct atomisp_sub_device *isp_subdev = atomisp_to_sub_device(pipe);

	if (!(vma->vm_flags & (VM_WRITE | VM_READ)))
		return -EACCES;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	mutex_lock(&isp->mutex);
	new_size = pipe->pix.width * pipe->pix.height * 2;

	/* mmap for ISP offline raw data */
	if (atomisp_subdev_source_pad(vdev)
	    == ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE &&
	    vma->vm_pgoff == (ISP_PARAM_MMAP_OFFSET >> PAGE_SHIFT)) {
		if (isp_subdev->params.online_process != 0) {
			ret = -EINVAL;
			goto error;
		}
		raw_virt_addr = isp_subdev->raw_output_frame;
		if (raw_virt_addr == NULL) {
			dev_err(isp->dev, "Failed to request RAW frame\n");
			ret = -EINVAL;
			goto error;
		}

		ret = remove_pad_from_frame(isp, raw_virt_addr,
				      pipe->pix.width,
				      pipe->pix.height);
		if (ret < 0) {
			dev_err(isp->dev, "remove pad failed.\n");
			goto error;
		}
		origin_size = raw_virt_addr->data_bytes;
		raw_virt_addr->data_bytes = new_size;

		if (size != PAGE_ALIGN(new_size)) {
			dev_err(isp->dev, "incorrect size for mmap ISP"
				 " Raw Frame\n");
			ret = -EINVAL;
			goto error;
		}

		if (frame_mmap(isp, raw_virt_addr, vma)) {
			dev_err(isp->dev, "frame_mmap failed.\n");
			raw_virt_addr->data_bytes = origin_size;
			ret = -EAGAIN;
			goto error;
		}
		raw_virt_addr->data_bytes = origin_size;
		vma->vm_flags |= VM_RESERVED;
		mutex_unlock(&isp->mutex);
		return 0;
	}

	/*
	 * mmap for normal frames
	 */
	if (size != pipe->pix.sizeimage) {
		dev_err(isp->dev, "incorrect size for mmap ISP frames\n");
		ret = -EINVAL;
		goto error;
	}
	mutex_unlock(&isp->mutex);

	return atomisp_videobuf_mmap_mapper(&pipe->capq, vma);

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

static int atomisp_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return videobuf_mmap_mapper(&pipe->outq, vma);
}

static unsigned int atomisp_poll(struct file *file,
				 struct poll_table_struct *pt)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	mutex_lock(&isp->mutex);
	if (pipe->capq.streaming != 1) {
		mutex_unlock(&isp->mutex);
		return POLLERR;
	}
	mutex_unlock(&isp->mutex);

	return videobuf_poll_stream(file, &pipe->capq, pt);
}

const struct v4l2_file_operations atomisp_fops = {
	.owner = THIS_MODULE,
	.open = atomisp_open,
	.release = atomisp_release,
	.mmap = atomisp_mmap,
	.ioctl = video_ioctl2,
	.poll = atomisp_poll,
};

const struct v4l2_file_operations atomisp_file_fops = {
	.owner = THIS_MODULE,
	.open = atomisp_open,
	.release = atomisp_release,
	.mmap = atomisp_file_mmap,
	.ioctl = video_ioctl2,
	.poll = atomisp_poll,
};
