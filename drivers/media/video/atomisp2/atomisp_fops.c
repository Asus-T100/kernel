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
#include "atomisp_fops.h"
#include "atomisp_internal.h"
#include "atomisp_ioctl.h"
#include "atomisp_subdev.h"
#include "atomisp-regs.h"

#include "hrt/hive_isp_css_mm_hrt.h"

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
int atomisp_buf_setup(struct videobuf_queue *vq,
		      unsigned int *count,
		      unsigned int *size)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	*size = pipe->pix.sizeimage;

	return 0;
}

int atomisp_buf_prepare(struct videobuf_queue *vq,
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

int atomisp_q_video_buffers_to_css(struct atomisp_device *isp,
			     struct atomisp_video_pipe *pipe,
			     enum sh_css_buffer_type css_buf_type,
			     enum sh_css_pipe_id css_pipe_id)
{
	struct videobuf_buffer *vb;
	struct videobuf_vmalloc_memory *vm_mem;
	unsigned long irqflags;
	int err;

	while (pipe->buffers_in_css < ATOMISP_CSS_Q_DEPTH) {
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
		err = sh_css_queue_buffer(css_pipe_id, css_buf_type,
					  vm_mem->vaddr);
		if (err) {
			spin_lock_irqsave(&pipe->irq_lock, irqflags);
			list_add_tail(&vb->queue, &pipe->activeq);
			vb->state = VIDEOBUF_QUEUED;
			spin_unlock_irqrestore(&pipe->irq_lock, irqflags);
			dev_err(isp->dev, "%s, css q fails: %d\n",
					__func__, err);
			return -EINVAL;
		}
		pipe->buffers_in_css++;
		vb = NULL;
	}
	return 0;
}

int atomisp_q_s3a_buffers_to_css(struct atomisp_device *isp,
			   enum sh_css_pipe_id css_pipe_id)
{
	struct atomisp_s3a_buf *s3a_buf;

	if (list_empty(&isp->s3a_stats)) {
		WARN(1, "%s: No s3a buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp->s3a_bufs_in_css[css_pipe_id] < ATOMISP_CSS_Q_DEPTH) {
		s3a_buf = list_entry(isp->s3a_stats.next,
				struct atomisp_s3a_buf, list);
		list_move_tail(&s3a_buf->list, &isp->s3a_stats);

		if (sh_css_queue_buffer(css_pipe_id,
					SH_CSS_BUFFER_TYPE_3A_STATISTICS,
					&s3a_buf->s3a_data)) {
			dev_err(isp->dev, "failed to q s3a stat buffer\n");
			return -EINVAL;
		}
		isp->s3a_bufs_in_css[css_pipe_id]++;
	}
	return 0;
}

int atomisp_q_dis_buffers_to_css(struct atomisp_device *isp,
			   enum sh_css_pipe_id css_pipe_id)
{
	struct atomisp_dis_buf *dis_buf;

	if (list_empty(&isp->dis_stats)) {
		WARN(1, "%s: No dis buffers available!\n", __func__);
		return -EINVAL;
	}

	while (isp->dis_bufs_in_css < ATOMISP_CSS_Q_DEPTH) {
		dis_buf = list_entry(isp->dis_stats.next,
				struct atomisp_dis_buf, list);
		list_move_tail(&dis_buf->list, &isp->dis_stats);

		if (sh_css_queue_buffer(css_pipe_id,
					SH_CSS_BUFFER_TYPE_DIS_STATISTICS,
					&dis_buf->dis_data)) {
			dev_err(isp->dev, "failed to q dis stat buffer\n");
			return -EINVAL;
		}
		isp->dis_bufs_in_css++;
		dis_buf = NULL;
	}
	return 0;
}

/* queue all available buffers to css */
int atomisp_qbuffers_to_css(struct atomisp_device *isp)
{
	enum sh_css_buffer_type buf_type;
	enum sh_css_pipe_id css_capture_pipe_id = SH_CSS_NR_OF_PIPELINES;
	enum sh_css_pipe_id css_preview_pipe_id = SH_CSS_NR_OF_PIPELINES;
	struct atomisp_video_pipe *capture_pipe = NULL;
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *preview_pipe = NULL;

	if (!isp->isp_subdev.enable_vfpp->val) {
		preview_pipe = &isp->isp_subdev.video_out_capture;
		css_preview_pipe_id = SH_CSS_VIDEO_PIPELINE;
	} else if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
		capture_pipe = &isp->isp_subdev.video_out_capture;
		preview_pipe = &isp->isp_subdev.video_out_preview;
		css_capture_pipe_id = SH_CSS_VIDEO_PIPELINE;
		css_preview_pipe_id = SH_CSS_VIDEO_PIPELINE;
	} else if (isp->params.continuous_vf) {
		capture_pipe = &isp->isp_subdev.video_out_capture;
		vf_pipe = &isp->isp_subdev.video_out_vf;
		preview_pipe = &isp->isp_subdev.video_out_preview;

		css_preview_pipe_id = SH_CSS_PREVIEW_PIPELINE;
		css_capture_pipe_id = SH_CSS_CAPTURE_PIPELINE;
	} else if (isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_PREVIEW) {
		preview_pipe = &isp->isp_subdev.video_out_preview;
		css_preview_pipe_id = SH_CSS_PREVIEW_PIPELINE;
	} else {
		/* ATOMISP_RUN_MODE_STILL_CAPTURE */
		capture_pipe = &isp->isp_subdev.video_out_capture;
		if (!atomisp_is_mbuscode_raw(
			    isp->isp_subdev.
			    fmt[isp->isp_subdev.capture_pad].fmt.code))
			vf_pipe = &isp->isp_subdev.video_out_vf;
		css_capture_pipe_id = SH_CSS_CAPTURE_PIPELINE;
	}

	if (capture_pipe) {
		buf_type = atomisp_get_css_buf_type(isp, capture_pipe);
		atomisp_q_video_buffers_to_css(isp, capture_pipe, buf_type,
					 css_capture_pipe_id);
	}

	if (vf_pipe) {
		buf_type = atomisp_get_css_buf_type(isp, vf_pipe);
		atomisp_q_video_buffers_to_css(isp, vf_pipe, buf_type,
					 css_capture_pipe_id);
	}

	if (preview_pipe) {
		buf_type = atomisp_get_css_buf_type(isp, preview_pipe);
		atomisp_q_video_buffers_to_css(isp, preview_pipe, buf_type,
					 css_preview_pipe_id);
	}

	if (isp->params.curr_grid_info.s3a_grid.enable) {
		if (css_capture_pipe_id < SH_CSS_NR_OF_PIPELINES)
			atomisp_q_s3a_buffers_to_css(isp, css_capture_pipe_id);
		if (css_preview_pipe_id < SH_CSS_NR_OF_PIPELINES)
			atomisp_q_s3a_buffers_to_css(isp, css_preview_pipe_id);
	}

	if (isp->params.curr_grid_info.dvs_grid.enable)
		atomisp_q_dis_buffers_to_css(isp, css_capture_pipe_id);

	return 0;
}

void atomisp_buf_queue(struct videobuf_queue *vq,
		       struct videobuf_buffer *vb)
{
	struct atomisp_video_pipe *pipe = vq->priv_data;

	list_add_tail(&vb->queue, &pipe->activeq);
	vb->state = VIDEOBUF_QUEUED;
}

void atomisp_buf_release(struct videobuf_queue *vq,
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

int atomisp_init_struct(struct atomisp_device *isp)
{
	if (isp == NULL)
		return -EINVAL;

	v4l2_ctrl_s_ctrl(isp->isp_subdev.run_mode,
			 ATOMISP_RUN_MODE_STILL_CAPTURE);
	isp->params.color_effect = V4L2_COLORFX_NONE;
	isp->params.bad_pixel_en = 1;
	isp->params.gdc_cac_en = 0;
	isp->params.video_dis_en = 0;
	isp->params.sc_en = 0;
	isp->params.fpn_en = 0;
	isp->params.xnr_en = 0;
	isp->params.false_color = 0;
	isp->params.online_process = 1;
	isp->params.yuv_ds_en = 0;
	//isp->params.vf_overlay = NULL;
	isp->params.offline_parm.num_captures = 1;
	isp->params.offline_parm.skip_frames = 0;
	isp->params.offline_parm.offset = 0;
	isp->params.continuous_vf = false;
	isp->sw_contex.file_input = 0;
	isp->need_gfx_throttle = true;
	isp->isp_fatal_error = false;
	isp->delayed_init = ATOMISP_DELAYED_INIT_NOT_QUEUED;

	/*
	 * For Merrifield, frequency is scalable.
	 * After boot-up, the default frequency is 200MHz.
	 * For Medfield/Clovertrail, all running at 320MHz
	 */
	if (IS_MRFLD)
		isp->sw_contex.running_freq = ISP_FREQ_200MHZ;
	else
		isp->sw_contex.running_freq = ISP_FREQ_320MHZ;

	/* Add for channel */
	if (isp->inputs[0].camera)
		isp->input_curr = 0;

	/* obtain the pointers to the default configurations */
	sh_css_get_tnr_config(&isp->params.default_tnr_config);
	sh_css_get_nr_config(&isp->params.default_nr_config);
	sh_css_get_ee_config(&isp->params.default_ee_config);
	sh_css_get_ob_config(&isp->params.default_ob_config);
	sh_css_get_dp_config(&isp->params.default_dp_config);
	sh_css_get_wb_config(&isp->params.default_wb_config);
	sh_css_get_cc_config(&isp->params.default_cc_config);
	sh_css_get_de_config(&isp->params.default_de_config);
	sh_css_get_gc_config(&isp->params.default_gc_config);
	sh_css_get_3a_config(&isp->params.default_3a_config);
	sh_css_get_macc_table(&isp->params.default_macc_table);
	sh_css_get_ctc_table(&isp->params.default_ctc_table);
	sh_css_get_gamma_table(&isp->params.default_gamma_table);

	/* we also initialize our configurations with the defaults */
	isp->params.tnr_config  = *isp->params.default_tnr_config;
	isp->params.nr_config   = *isp->params.default_nr_config;
	isp->params.ee_config   = *isp->params.default_ee_config;
	isp->params.ob_config   = *isp->params.default_ob_config;
	isp->params.dp_config   = *isp->params.default_dp_config;
	isp->params.wb_config   = *isp->params.default_wb_config;
	isp->params.cc_config   = *isp->params.default_cc_config;
	isp->params.de_config   = *isp->params.default_de_config;
	isp->params.gc_config   = *isp->params.default_gc_config;
	isp->params.s3a_config  = *isp->params.default_3a_config;
	isp->params.macc_table  = *isp->params.default_macc_table;
	isp->params.ctc_table   = *isp->params.default_ctc_table;
	isp->params.gamma_table = *isp->params.default_gamma_table;

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
 * file operation functions
 */
unsigned int atomisp_users(struct atomisp_device *isp)
{
	return isp->isp_subdev.video_out_preview.users +
	       isp->isp_subdev.video_out_vf.users +
	       isp->isp_subdev.video_out_capture.users +
	       isp->isp_subdev.video_in.users;
}

static int atomisp_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret;
	struct sh_css_env my_env = sh_css_default_env();

	my_env.sh_env.alloc = my_kernel_malloc;
	my_env.sh_env.free = atomisp_kernel_free;
	/* FIXME: my_env.sh_env.flush needs to be implemented */

	v4l2_dbg(2, dbg_level, &atomisp_dev, ">%s [%d]\n",
				__func__, pipe->pipe_type);

	mutex_lock(&isp->mutex);

	if (!isp->input_cnt) {
		v4l2_err(&atomisp_dev, "no camera attached\n");
		ret = -EINVAL;
		goto error;
	}

	if (pipe->users)
		goto done;

	ret = atomisp_init_pipe(pipe);
	if (ret)
		goto error;

	if (atomisp_users(isp)) {
		v4l2_err(&atomisp_dev, "skip init isp in open\n");
		goto done;
	}

	/* runtime power management, turn on ISP */
	ret = pm_runtime_get_sync(vdev->v4l2_dev->dev);
	if (ret < 0) {
		v4l2_err(&atomisp_dev,
				"Failed to power on device\n");
		goto error;
	}

	device_set_base_address(0);

	/*
	 * if the driver gets closed and reopened, the HMM is not reinitialized
	 * This means we need to put the L1 page table base address back into
	 * the ISP
	 */
	if (isp->mmu_l1_base)
		/*
		 * according to sh_css.c sh_css_mmu_set_page_table_base_index
		 * is deprecated and mmgr_set_base_address should be used
		 * instead. But just for now (with CSS "alpha") replacing
		 * all sh_cssh_mmu_set_page_table_base_index() -calls
		 * with mmgr_set_base_address() is not working.
		 */
		sh_css_mmu_set_page_table_base_index(
				HOST_ADDRESS(isp->mmu_l1_base));

	/* With CSS "alpha" it is mandatory to set base address always */
	mmgr_set_base_address(HOST_ADDRESS(isp->mmu_l1_base));

	/* Init ISP */
	if (sh_css_init(&my_env, isp->firmware->data, isp->firmware->size)) {
		ret = -EINVAL;
		goto css_init_failed;
	}
	/*uncomment the following to lines to enable offline preview*/
	/*sh_css_enable_continuous(true);*/
	/*sh_css_preview_enable_online(false);*/

	/* CSS has default zoom factor of 61x61, we want no zoom
	   because the zoom binary for capture is broken (XNR). */
	v4l2_dbg(2, dbg_level, &atomisp_dev, "sh_css_init success\n");
	if (IS_MRFLD)
		sh_css_set_zoom_factor(MRFLD_MAX_ZOOM_FACTOR,
					MRFLD_MAX_ZOOM_FACTOR);
	else
		sh_css_set_zoom_factor(MFLD_MAX_ZOOM_FACTOR,
					MFLD_MAX_ZOOM_FACTOR);

	/* Initialize the CSS debug trace verbosity level. To change
	 * the verbosity level, change the definition of this macro
	 * up in the file
	 */
	sh_css_set_dtrace_level(CSS_DTRACE_VERBOSITY_LEVEL);

	atomisp_init_struct(isp);

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
	struct v4l2_mbus_framefmt isp_sink_fmt;
	struct v4l2_requestbuffers req;
	int ret = 0;

	v4l2_dbg(2, dbg_level, &atomisp_dev, ">%s [%d]\n",
				__func__, pipe->pipe_type);

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
	if (!isp->sw_contex.file_input) {
		memset(&isp_sink_fmt, 0, sizeof(isp_sink_fmt));
		atomisp_subdev_set_ffmt(&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &isp_sink_fmt);
	}

	if (atomisp_users(isp))
		goto done;

	/* clear the sink pad for file input */
	if (isp->sw_contex.file_input) {
		memset(&isp_sink_fmt, 0, sizeof(isp_sink_fmt));
		atomisp_subdev_set_ffmt(&isp->isp_subdev.subdev, NULL,
				V4L2_SUBDEV_FORMAT_ACTIVE,
				ATOMISP_SUBDEV_PAD_SINK, &isp_sink_fmt);
	}

	del_timer_sync(&isp->wdt);
	atomisp_acc_release(isp);
	atomisp_free_3a_dis_buffers(isp);
	atomisp_free_all_shading_tables(isp);
	atomisp_free_internal_buffers(isp);
	sh_css_uninit();
	hrt_isp_css_mm_clear();

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, s_power, 0);
	if (ret)
		v4l2_warn(&atomisp_dev, "Failed to power-off sensor\n");

	/* store L1 base address for next time we init the CSS */
/*
	isp->mmu_l1_base =
			(void *)mmgr_get_base_address();
*/
	isp->mmu_l1_base =
			(void *)sh_css_mmu_get_page_table_base_index();

	if (pm_runtime_put_sync(vdev->v4l2_dev->dev) < 0)
		v4l2_err(&atomisp_dev, "Failed to power off device\n");
done:
	mutex_unlock(&isp->mutex);
	mutex_unlock(&isp->streamoff_mutex);

	return 0;
}

/*
 * Memory help functions for image frame and private parameters
 */
static int do_isp_mm_remap(struct vm_area_struct *vma,
		    void *isp_virt, u32 host_virt, u32 pgnr)
{
	u32 pfn;

	while (pgnr) {
		pfn = hmm_virt_to_phys(isp_virt) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, host_virt, pfn,
				    PAGE_SIZE, PAGE_SHARED)) {
			v4l2_err(&atomisp_dev,
				    "remap_pfn_range err.\n");
			return -EAGAIN;
		}

		isp_virt += PAGE_SIZE;
		host_virt += PAGE_SIZE;
		pgnr--;
	}

	return 0;
}

static int frame_mmap(const struct sh_css_frame *frame,
	struct vm_area_struct *vma)
{
	void *isp_virt;
	u32 host_virt;
	u32 pgnr;

	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer.\n", __func__);
		return -EINVAL;
	}

	host_virt = vma->vm_start;
	isp_virt = (void *)frame->data;
	atomisp_get_frame_pgnr(frame, &pgnr);

	if (do_isp_mm_remap(vma, isp_virt, host_virt, pgnr))
		return -EAGAIN;

	return 0;
}

int atomisp_videobuf_mmap_mapper(struct videobuf_queue *q,
	struct vm_area_struct *vma)
{
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL, i;
	struct videobuf_vmalloc_memory *vm_mem;
	struct videobuf_mapping *map;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);
	if (!(vma->vm_flags & VM_WRITE) || !(vma->vm_flags & VM_SHARED)) {
		v4l2_err(&atomisp_dev, "map appl bug:"
			    " PROT_WRITE and MAP_SHARED are required\n");
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
			ret = frame_mmap(vm_mem->vaddr, vma);
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
static int
remove_pad_from_frame(struct sh_css_frame *in_frame, __u32 width, __u32 height)
{
	unsigned int i;
	unsigned short *buffer;
	int ret = 0;
	unsigned short *load = (unsigned short *)in_frame->data;
	unsigned short *store = load;

	buffer = kmalloc(width*sizeof(*load), GFP_KERNEL);
	if (!buffer) {
		v4l2_err(&atomisp_dev, "out of memory.\n");
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
	struct sh_css_frame *raw_virt_addr;
	u32 start = vma->vm_start;
	u32 end = vma->vm_end;
	u32 size = end - start;
	u32 origin_size, new_size;
	int ret;

	if (!(vma->vm_flags & (VM_WRITE | VM_READ)))
		return -EACCES;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	mutex_lock(&isp->mutex);
	new_size = pipe->pix.width * pipe->pix.height * 2;

	/* mmap for ISP offline raw data */
	if ((pipe->pipe_type == ATOMISP_PIPE_CAPTURE) &&
	    (vma->vm_pgoff == (ISP_PARAM_MMAP_OFFSET >> PAGE_SHIFT))) {
		if (isp->params.online_process != 0) {
			ret = -EINVAL;
			goto error;
		}
		raw_virt_addr = isp->raw_output_frame;
		if (raw_virt_addr == NULL) {
			v4l2_err(&atomisp_dev,
				 "Failed to request RAW frame\n");
			ret = -EINVAL;
			goto error;
		}

		ret = remove_pad_from_frame(raw_virt_addr,
				      pipe->pix.width,
				      pipe->pix.height);
		if (ret < 0) {
			v4l2_err(&atomisp_dev, "remove pad failed.\n");
			goto error;
		}
		origin_size = raw_virt_addr->data_bytes;
		raw_virt_addr->data_bytes = new_size;

		if (size != PAGE_ALIGN(new_size)) {
			v4l2_err(&atomisp_dev,
				 "incorrect size for mmap ISP"
				 " Raw Frame\n");
			ret = -EINVAL;
			goto error;
		}

		if (frame_mmap(raw_virt_addr, vma)) {
			v4l2_err(&atomisp_dev,
				 "frame_mmap failed.\n");
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
		v4l2_err(&atomisp_dev,
			    "incorrect size for mmap ISP frames\n");
		ret = -EINVAL;
		goto error;
	}
	mutex_unlock(&isp->mutex);

	return atomisp_videobuf_mmap_mapper(&pipe->capq, vma);

error:
	mutex_unlock(&isp->mutex);

	return ret;
}

int atomisp_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return videobuf_mmap_mapper(&pipe->outq, vma);
}

static unsigned int
atomisp_poll(struct file *file, struct poll_table_struct *pt)
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

