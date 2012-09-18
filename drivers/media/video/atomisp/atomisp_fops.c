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

#include "atomisp_ioctl.h"
#include "atomisp_cmd.h"
#include "atomisp_fops.h"
#include "atomisp_common.h"
#include "hrt/hive_isp_css_mm_hrt.h"
#include "css/sh_css_debug.h"

#include "css/sh_css_firmware.h"
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
 * Implementation in atomisp_cmd.c only required to be known here as
 * it is registered in CSS with "sh_css_init()" which is called from
 * here. So "flush_acc_api_arguments()" is not a true public function
 *
 * "flush_acc_api_arguments()" needs to be accessible in CSS for the
 * extension type accelerator cache control of (shared buffer pointer)
 * arguments
 */
extern void flush_acc_api_arguments(struct sh_css_acc_fw *fw);

/*
 * Videobuf2 ops
 */
static int atomisp_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				unsigned int *nplanes, unsigned long sizes[],
				void *alloc_ctxs[])
{
	struct atomisp_video_pipe *pipe = vq->drv_priv;
#ifdef MULTI_PLANE_SUPPORT
	int i;
#endif
	/*
	 * if number of buffers < 1, driver cannot handle them, need to
	 * set number of buffers afresh.
	 */
	*nbuffers = clamp((unsigned int)*nbuffers, (unsigned int)1,
			  (unsigned int)VIDEO_MAX_FRAME);
	/* currently only one plane gets supported */
	*nplanes = 1;
#ifdef MULTI_PLANE_SUPPORT
	for (i = 0; i < *nplanes; i++) {
		sizes[i] = pipe->format->out.sizeimage;

		/* alloc_ctxs is used to store allocator private data */
		if (pipe->is_main)
			alloc_ctxs[i] = &pipe->main_info;
		else
			alloc_ctxs[i] = &pipe->vf_info;
	}
#else
	sizes[0] = pipe->format->out.sizeimage;

	/* alloc_ctxs is used to store allocator private data */
	if (pipe->is_main)
		alloc_ctxs[0] = &pipe->main_info;
	else
		alloc_ctxs[0] = &pipe->vf_info;
#endif
	return 0;
}

static int atomisp_buf_init(struct vb2_buffer *vb)
{
	/* reserved */
	return 0;
}

static void atomisp_buf_queue(struct vb2_buffer *vb)
{
	unsigned long flags;
	struct vb2_queue *vq = vb->vb2_queue;
	struct atomisp_video_pipe *pipe = vq->drv_priv;

	spin_lock_irqsave(&pipe->irq_lock, flags);
	list_add_tail(&vb->done_entry, &pipe->activeq);
	spin_unlock_irqrestore(&pipe->irq_lock, flags);
	wake_up(&pipe->vb_queued);
}

static int atomisp_start_streaming(struct vb2_queue *vq)
{
	struct atomisp_video_pipe *pipe = vq->drv_priv;
	struct video_device *vdev = &pipe->vdev;
	struct atomisp_device *isp = video_get_drvdata(vdev);
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif

	isp->sw_contex.isp_streaming = true;

	if (isp->sw_contex.work_queued)
		goto done;

#ifdef PUNIT_CAMERA_BUSY
	/*
	 * As per h/w architect and ECO 697611 we need to throttle the
	 * GFX performance (freq) while camera is up to prevent peak
	 * current issues. this is done by setting the camera busy bit.
	 */
	msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, OR1);
	msg_ret |= 0x100;
	intel_mid_msgbus_write32(PUNIT_PORT, OR1, msg_ret);
#endif

	/*
	 * WQ is about get launched. So reset this flag here. atomisp_dqbuf()
	 * checks this flag and returns ISP Error if this flag is set. There
	 * can be situations where atomisp_dqbuf() gets called even before
	 * WQ is scheduled.
	 */
	isp->sw_contex.error = false;

	/*stream on sensor in work thread*/
	queue_work(isp->work_queue, &isp->work);
	isp->sw_contex.work_queued = true;

done:
	return 0;
}

static int atomisp_stop_streaming(struct vb2_queue *vq)
{
	struct atomisp_video_pipe *pipe = vq->drv_priv;
	struct video_device *vdev = &pipe->vdev;
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *vf_pipe = NULL;
	struct atomisp_video_pipe *mo_pipe = NULL;
	int ret = 0;
	unsigned long flags;
#ifdef PUNIT_CAMERA_BUSY
	u32 msg_ret;
#endif
	spin_lock_irqsave(&isp->irq_lock, flags);
	isp->sw_contex.isp_streaming = false;
	spin_unlock_irqrestore(&isp->irq_lock, flags);

	/* cancel work queue*/
	if (isp->sw_contex.work_queued) {
		mo_pipe = &isp->isp_subdev.video_out_mo;
		wake_up_interruptible(&mo_pipe->vb_queued);
		if (atomisp_is_viewfinder_support(isp)) {
			vf_pipe = &isp->isp_subdev.video_out_vf;
			wake_up_interruptible(&vf_pipe->vb_queued);
		}
		cancel_work_sync(&isp->work);
		isp->sw_contex.work_queued = false;

		spin_lock_irqsave(&mo_pipe->irq_lock, flags);
		INIT_LIST_HEAD(&mo_pipe->activeq);
		spin_unlock_irqrestore(&mo_pipe->irq_lock, flags);
		if (atomisp_is_viewfinder_support(isp)) {
			vf_pipe = &isp->isp_subdev.video_out_vf;
			spin_lock_irqsave(&vf_pipe->irq_lock, flags);
			INIT_LIST_HEAD(&vf_pipe->activeq);
			spin_unlock_irqrestore(&vf_pipe->irq_lock, flags);
		}
	}


	if (!IS_MRFLD)
		atomisp_wdt_lock_dog(isp);

	/*stream off sensor, power off is called in senor driver*/
	if (!pipe->is_main)
		return 0;
	if (!isp->sw_contex.file_input)
		ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       video, s_stream, 0);

	isp->sw_contex.sensor_streaming = false;

#ifdef PUNIT_CAMERA_BUSY
	/* Free camera_busy bit */
	msg_ret = intel_mid_msgbus_read32(PUNIT_PORT, OR1);
	msg_ret &= ~0x100;
	intel_mid_msgbus_write32(PUNIT_PORT, OR1, msg_ret);
#endif

	/* ISP work around, need to reset isp */
	if (pipe->is_main && isp->sw_contex.power_state == ATOM_ISP_POWER_UP)
		atomisp_reset(isp);

	return ret;
}

static int atomisp_queue_setup_output(struct vb2_queue *vq,
				      unsigned int *nbuffers,
				      unsigned int *nplanes,
				      unsigned long sizes[],
				      void *alloc_ctxs[])
{
	struct atomisp_video_pipe *pipe = vq->drv_priv;
#ifdef MULTI_PLANE_SUPPORT
	int i;
#endif
	/*
	 * if number of buffers < 1, driver cannot handle them, need to
	 * set number of buffers afresh.
	 */
	*nbuffers = clamp((unsigned int)*nbuffers, (unsigned int)1,
			  (unsigned int)VIDEO_MAX_FRAME);
	/* currently only one plane gets supported */
	*nplanes = 1;
#ifdef MULTI_PLANE_SUPPORT
	for (i = 0; i < *nplanes; i++)
		sizes[i] = pipe->out_fmt->imagesize;
#else
		sizes[0] = pipe->out_fmt->imagesize;
#endif
	return 0;
}

static int atomisp_buf_init_output(struct vb2_buffer *vb)
{
	/* reserved */
	return 0;
}

static void atomisp_buf_queue_output(struct vb2_buffer *vb)
{

}

static struct vb2_ops vb2_qops = {
	.queue_setup	= atomisp_queue_setup,
	.buf_init	= atomisp_buf_init,
	.buf_queue	= atomisp_buf_queue,
	.start_streaming = atomisp_start_streaming,
	.stop_streaming	= atomisp_stop_streaming,
};

static struct vb2_ops vb2_qops_output = {
	.queue_setup	= atomisp_queue_setup_output,
	.buf_init	= atomisp_buf_init_output,
	.buf_queue	= atomisp_buf_queue_output,
};

static int atomisp_uninit_pipe(struct atomisp_video_pipe *pipe)
{
	kfree(pipe->format);
	pipe->format = NULL;

	kfree(pipe->out_fmt);
	pipe->out_fmt = NULL;

	kfree(pipe->main_info.frame_info);
	pipe->main_info.frame_info = NULL;

	kfree(pipe->vf_info.frame_info);
	pipe->vf_info.frame_info = NULL;

	pipe->opened = false;
	return 0;
}

static int atomisp_init_pipe(struct atomisp_video_pipe *pipe)
{

	pipe->out_fmt = kzalloc(sizeof(struct atomisp_fmt), GFP_KERNEL);
	if (pipe->out_fmt == NULL)
		goto nomem;


	pipe->format =
	kzalloc(sizeof(struct atomisp_video_pipe_format), GFP_KERNEL);
	if (pipe->format == NULL)
		goto nomem;

	pipe->main_info.frame_info =
	kzalloc(sizeof(*pipe->main_info.frame_info), GFP_KERNEL);
	if (pipe->main_info.frame_info == NULL)
		goto nomem;

	/* pipe->vf_info.frame_info is void * */
	pipe->vf_info.frame_info =
	kzalloc(sizeof(*pipe->vf_info.frame_info), GFP_KERNEL);
	if (pipe->vf_info.frame_info == NULL)
		goto nomem;

	/* init locks */
	spin_lock_init(&pipe->irq_lock);
	mutex_init(&pipe->mutex);

	atomisp_vb2_queue_init(&pipe->capq, &vb2_qops, pipe,
				V4L2_MEMORY_USERPTR,
				V4L2_BUF_TYPE_VIDEO_CAPTURE,
				sizeof(struct atomisp_buffer),
				VB2_MMAP | VB2_USERPTR);

	atomisp_vb2_queue_init(&pipe->outq,
				&vb2_qops_output, pipe,
				V4L2_MEMORY_MMAP,
				V4L2_BUF_TYPE_VIDEO_OUTPUT,
				sizeof(struct atomisp_buffer),
				VB2_MMAP | VB2_USERPTR);

	INIT_LIST_HEAD(&pipe->activeq);
	INIT_LIST_HEAD(&pipe->activeq_out);
	init_waitqueue_head(&pipe->vb_queued);
	pipe->opened = true;

	return 0;
nomem:
	kfree(pipe->out_fmt);
	kfree(pipe->format);
	kfree(pipe->main_info.frame_info);
	kfree(pipe->vf_info.frame_info);
	return -ENOMEM;
}

int atomisp_init_struct(struct atomisp_device *isp)
{
	if (isp == NULL)
		return -EINVAL;

	isp->main_format = NULL;
	isp->vf_format = NULL;
	isp->input_format = NULL;
	isp->sw_contex.run_mode = CI_MODE_STILL_CAPTURE;
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
	isp->params.vf_overlay = NULL;
	isp->sw_contex.sensor_streaming = false;
	isp->sw_contex.isp_streaming = false;
	isp->sw_contex.work_queued = false;
	isp->sw_contex.error = false;
	isp->sw_contex.file_input = 0;
	isp->isp_timeout = false;

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

/*
 * file operation functions
 */
static int atomisp_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret = -EINVAL;

	if (!isp || isp->sw_contex.probed == 0)
		return -ENODEV;

	/* if (atomisp_get(isp) == NULL) { */
	mutex_lock(&isp->input_lock);

	if (!isp->input_cnt) {
		v4l2_err(&atomisp_dev, "no camera attached\n");
		goto error;
	}

	if (pipe->opened)
		goto done;

	if (atomisp_init_pipe(pipe) < 0)
		goto error;

	if (isp->sw_contex.init) {
		v4l2_err(&atomisp_dev, "skip init isp in open\n");
		goto done;
	}

#ifdef CONFIG_PM
	if (!IS_MRFLD) {
		/* runtime power management, turn on ISP */
		if (pm_runtime_get_sync(vdev->v4l2_dev->dev) < 0) {
			v4l2_err(&atomisp_dev,
				 "Failed to power on device\n");
			goto runtime_get_failed;
		}
	}
#endif
	/* if the driver gets closed and reopened, the HMM is not reinitialized
	 * This means we need to put the L1 page table base address back into
	 * the ISP. */
	if (isp->hw_contex.mmu_l1_base)
		sh_css_mmu_set_page_table_base_address(
						isp->hw_contex.mmu_l1_base);

	if (IS_MRFLD) {
		/* Init ISP */
		if (sh_css_init(atomisp_kernel_malloc,
				atomisp_kernel_free,
				flush_acc_api_arguments, /*NULL,*/
				SH_CSS_INTERRUPT_SETTING_PULSE,
				isp2400_fw_data_sim,
				sizeof(isp2400_fw_data_sim)))
			goto css_init_failed;
		/* CSS has default zoom factor of 61x61, we want no zoom
		   because the zoom binary for capture is broken (XNR). */
		sh_css_set_zoom_factor(1024, 1024);
	} else {
		/* Init ISP */
		if (sh_css_init(atomisp_kernel_malloc,
				atomisp_kernel_free,
				flush_acc_api_arguments, /*NULL,*/
				SH_CSS_INTERRUPT_SETTING_PULSE,
				isp->firmware->data,
				isp->firmware->size))
			goto css_init_failed;
		/* CSS has default zoom factor of 61x61, we want no zoom
		   because the zoom binary for capture is broken (XNR). */
		sh_css_set_zoom_factor(64, 64);
	}
	/* Initialize the CSS debug trace verbosity level. To change
	 * the verbosity level, change the definition of this macro
	 * up in the file
	 */
	sh_css_set_dtrace_level(CSS_DTRACE_VERBOSITY_LEVEL);

	atomisp_init_struct(isp);

	isp->sw_contex.init = true;
done:
	mutex_unlock(&isp->input_lock);

	return 0;

css_init_failed:
#ifdef CONFIG_PM
	if (!IS_MRFLD)
		pm_runtime_put(vdev->v4l2_dev->dev);
#endif
runtime_get_failed:
	atomisp_uninit_pipe(pipe);
error:
	mutex_unlock(&isp->input_lock);
	return ret;
}

static int atomisp_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_device *isp = video_get_drvdata(vdev);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);
	int ret = 0;

	if (isp == NULL)
		return -EBADF;

	mutex_lock(&isp->input_lock);

	if (pipe->capq.streaming &&
	    atomisp_streamoff(file, NULL,
			      V4L2_BUF_TYPE_VIDEO_CAPTURE))
		goto error;

	if (pipe->opened == false)
		goto done;

	/* in case image buf is not freed */
	if (pipe->capq.bufs[0])
		vb2_queue_release(&pipe->capq);

	if (pipe->format) {
		kfree(pipe->format);
		pipe->format = NULL;
		isp->main_format = NULL;
	}

	if (pipe->outq.bufs[0])
		vb2_queue_release(&pipe->outq);

	if (pipe->out_fmt) {
		kfree(pipe->out_fmt);
		pipe->out_fmt = NULL;
	}

	if (isp->vf_format) {
		kfree(isp->vf_format);
		isp->vf_format = NULL;
	}

	if (isp->input_format) {
		kfree(isp->input_format);
		isp->input_format = NULL;
	}

	if (!pipe->is_main)
		goto done;

	if (isp->sw_contex.init == false)
		goto done;

	if ((!isp->isp_subdev.video_out_vf.opened) &&
	    (isp->vf_frame)) {
		sh_css_frame_free(isp->vf_frame);
		isp->vf_frame = NULL;
	}


	atomisp_free_3a_buffers(isp);
	atomisp_free_dis_buffers(isp);
	atomisp_free_all_shading_tables(isp);
	atomisp_free_internal_buffers(isp);
	sh_css_uninit();
	hrt_isp_css_mm_clear();

	/*uninit the camera subdev*/
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
			       core, init, 0);
	if (ret == -1 || ret == -EINVAL)
		v4l2_err(&atomisp_dev, "sensor firmware failed\n");

	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera,
				       core, s_power, 0);
	if (ret)
		v4l2_warn(&atomisp_dev, "Failed to power-off sensor\n");

	/* store L1 base address for next time we init the CSS */
	isp->hw_contex.mmu_l1_base = sh_css_mmu_get_page_table_base_address();
	isp->sw_contex.init = false;

	atomisp_acc_unload_all(isp);

#ifdef CONFIG_PM
	if (pm_runtime_put_sync(vdev->v4l2_dev->dev))
		v4l2_err(&atomisp_dev,
			 "Failed to power off device\n");
#endif
done:
	pipe->opened = false;
	mutex_unlock(&isp->input_lock);

	return 0;
error:
	mutex_unlock(&isp->input_lock);
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
	unsigned short *load = in_frame->data;
	unsigned short *store = load;

	buffer = kmalloc(width*sizeof(*load), GFP_KERNEL);
	if (!buffer) {
		v4l2_err(&atomisp_dev, "out of memory.\n");
		return -ENOMEM;
	}

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

	if (!(vma->vm_flags & (VM_WRITE | VM_READ)))
		return -EACCES;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	new_size = pipe->format->out.width *
		pipe->format->out.height * 2;

	/* mmap for ISP offline raw data */
	if ((pipe->is_main) &&
	    (vma->vm_pgoff == (ISP_PARAM_MMAP_OFFSET >> PAGE_SHIFT))) {
		int ret;
		if (isp->params.online_process != 0)
			return -EINVAL;
		raw_virt_addr = isp->raw_output_frame;
		if (raw_virt_addr == NULL) {
			v4l2_err(&atomisp_dev,
				 "Failed to request RAW frame\n");
			return -EINVAL;
		}

		ret = remove_pad_from_frame(raw_virt_addr,
				      pipe->format->out.width,
				      pipe->format->out.height);
		if (ret < 0) {
			v4l2_err(&atomisp_dev, "remove pad failed.\n");
			return ret;
		}
		origin_size = raw_virt_addr->data_bytes;
		raw_virt_addr->data_bytes = new_size;

		if (size != PAGE_ALIGN(new_size)) {
			v4l2_err(&atomisp_dev,
				 "incorrect size for mmap ISP"
				 " Raw Frame\n");
			return -EINVAL;
		}

		if (frame_mmap(raw_virt_addr, vma)) {
			v4l2_err(&atomisp_dev,
				 "frame_mmap failed.\n");
			raw_virt_addr->data_bytes = origin_size;
			return -EAGAIN;
		}
		raw_virt_addr->data_bytes = origin_size;
		vma->vm_flags |= VM_RESERVED;
		return 0;
	}

	/*
	 * mmap for normal frames
	 */
	if (size != pipe->format->out.sizeimage) {
		v4l2_err(&atomisp_dev,
			    "incorrect size for mmap ISP frames\n");
		return -EINVAL;
	}

	return vb2_mmap(&pipe->capq, vma);
}

int atomisp_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_mmap(&pipe->outq, vma);
}

static unsigned int
atomisp_poll(struct file *file, struct poll_table_struct *pt)
{
	struct video_device *vdev = video_devdata(file);
	struct atomisp_video_pipe *pipe = atomisp_to_video_pipe(vdev);

	return vb2_poll(&pipe->capq, file, pt);
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

