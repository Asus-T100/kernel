/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#include <media/videobuf-vmalloc.h>

#include "sh_css_debug.h"
#include "host/mmu_local.h"
#include "device_access/device_access.h"
#include "memory_access/memory_access.h"

#include "atomisp_compat.h"
#include "atomisp_internal.h"
#include "atomisp_cmd.h"

#define CSS_DTRACE_VERBOSITY_LEVEL	5	/* Controls trace verbosity */

int atomisp_css_init(struct atomisp_device *isp)
{
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
	if (sh_css_init(&isp->css_env.isp_css_env,
			isp->firmware->data, isp->firmware->size)) {
		dev_err(isp->dev, "css init failed --- bad firmware?\n");
		return -EINVAL;
	}

	/* CSS has default zoom factor of 61x61, we want no zoom
	   because the zoom binary for capture is broken (XNR). */
	if (IS_ISP2400(isp))
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

	dev_dbg(isp->dev, "sh_css_init success\n");

	return 0;
}

void atomisp_css_uninit(struct atomisp_device *isp)
{
	sh_css_uninit();

	/* store L1 base address for next time we init the CSS */
	isp->mmu_l1_base = (void *)sh_css_mmu_get_page_table_base_index();
}

void atomisp_css_suspend(void)
{
	sh_css_suspend();
}

int atomisp_css_resume(struct atomisp_device *isp)
{
	sh_css_resume();

	return 0;
}

int atomisp_css_irq_translate(struct atomisp_device *isp,
			      unsigned int *infos)
{
	int err;

	err = sh_css_translate_interrupt(infos);
	if (err != sh_css_success) {
		dev_warn(isp->dev,
			  "%s:failed to translate irq (err = %d,infos = %d)\n",
			  __func__, err, *infos);
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_rx_get_irq_info(unsigned int *infos)
{
	sh_css_rx_get_interrupt_info(infos);
}

void atomisp_css_rx_clear_irq_info(unsigned int infos)
{
	sh_css_rx_clear_interrupt_info(infos);
}

int atomisp_css_irq_enable(struct atomisp_device *isp,
			    enum atomisp_css_irq_info info, bool enable)
{
	if (sh_css_enable_interrupt(info, enable) != sh_css_success) {
		dev_warn(isp->dev, "%s:Invalid irq info.\n", __func__);
		return -EINVAL;
	}

	return 0;
}

void atomisp_set_css_env(struct atomisp_device *isp)
{
	isp->css_env.isp_css_env = sh_css_default_env();
	isp->css_env.isp_css_env.sh_env.alloc = atomisp_kernel_zalloc;
	isp->css_env.isp_css_env.sh_env.free = atomisp_kernel_free;
}

void atomisp_css_init_struct(struct atomisp_device *isp)
{
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
}

int atomisp_q_video_buffer_to_css(struct atomisp_device *isp,
			struct videobuf_vmalloc_memory *vm_mem,
			enum atomisp_css_buffer_type css_buf_type,
			enum atomisp_css_pipe_id css_pipe_id)
{
	enum sh_css_err err;

	err = sh_css_queue_buffer(css_pipe_id, css_buf_type,
						vm_mem->vaddr);
	if (err != sh_css_success)
		return -EINVAL;

	return 0;
}

int atomisp_q_s3a_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_s3a_buf *s3a_buf,
			enum atomisp_css_pipe_id css_pipe_id)
{
	if (sh_css_queue_buffer(css_pipe_id,
				SH_CSS_BUFFER_TYPE_3A_STATISTICS,
				&s3a_buf->s3a_data)) {
		dev_dbg(isp->dev, "failed to q s3a stat buffer\n");
		return -EINVAL;
	}

	return 0;
}

int atomisp_q_dis_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_dis_buf *dis_buf,
			enum atomisp_css_pipe_id css_pipe_id)
{
	if (sh_css_queue_buffer(css_pipe_id,
				SH_CSS_BUFFER_TYPE_DIS_STATISTICS,
				&dis_buf->dis_data)) {
		dev_dbg(isp->dev, "failed to q dis stat buffer\n");
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_mmu_invalidate_cache(void)
{
	sh_css_mmu_invalidate_cache();
}

void atomisp_css_mmu_invalidate_tlb(void)
{
	sh_css_enable_sp_invalidate_tlb();
}

int atomisp_css_start(struct atomisp_device *isp,
			enum atomisp_css_pipe_id pipe_id, bool in_reset)
{
	enum sh_css_err err;

	err = sh_css_start(pipe_id);
	if (err != sh_css_success) {
		dev_err(isp->dev, "sh_css_start error:%d.\n", err);
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_update_isp_params(struct atomisp_device *isp)
{
	sh_css_update_isp_params();
}

int atomisp_css_queue_buffer(struct atomisp_device *isp,
			     enum atomisp_css_pipe_id pipe_id,
			     enum atomisp_css_buffer_type buf_type,
			     struct atomisp_css_buffer *isp_css_buffer)
{
	void *buffer;

	switch (buf_type) {
	case SH_CSS_BUFFER_TYPE_3A_STATISTICS:
		buffer = isp_css_buffer->s3a_data;
		break;
	case SH_CSS_BUFFER_TYPE_DIS_STATISTICS:
		buffer = isp_css_buffer->dis_data;
		break;
	case SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
		buffer = isp_css_buffer->css_buffer.data.frame;
		break;
	case SH_CSS_BUFFER_TYPE_OUTPUT_FRAME:
		buffer = isp_css_buffer->css_buffer.data.frame;
		break;
	default:
		return -EINVAL;
	}

	if (sh_css_queue_buffer(pipe_id, buf_type, buffer) != sh_css_success)
		return -EINVAL;

	return 0;
}

int atomisp_css_dequeue_buffer(struct atomisp_device *isp,
				enum atomisp_css_pipe_id pipe_id,
				enum atomisp_css_buffer_type buf_type,
				struct atomisp_css_buffer *isp_css_buffer)
{
	enum sh_css_err err;
	void *buffer;

	err = sh_css_dequeue_buffer(pipe_id, buf_type, (void **)&buffer);
	if (err != sh_css_success) {
		dev_err(isp->dev,
			"sh_css_dequeue_buffer failed: 0x%x\n", err);
		return -EINVAL;
	}

	switch (buf_type) {
	case SH_CSS_BUFFER_TYPE_3A_STATISTICS:
		isp_css_buffer->s3a_data = buffer;
		break;
	case SH_CSS_BUFFER_TYPE_DIS_STATISTICS:
		isp_css_buffer->dis_data = buffer;
		break;
	case SH_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME:
		isp_css_buffer->css_buffer.data.frame = buffer;
		break;
	case SH_CSS_BUFFER_TYPE_OUTPUT_FRAME:
		isp_css_buffer->css_buffer.data.frame = buffer;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int atomisp_css_get_3a_statistics(struct atomisp_device *isp,
				  struct atomisp_css_buffer *isp_css_buffer)
{
	enum sh_css_err err;

	err = sh_css_get_3a_statistics(isp->params.s3a_output_buf,
				isp->params.curr_grid_info.s3a_grid.use_dmem,
				isp_css_buffer->s3a_data);
	if (err != sh_css_success) {
		dev_err(isp->dev,
			"sh_css_get_3a_statistics failed: 0x%x\n", err);
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_get_dis_statistics(struct atomisp_device *isp,
				    struct atomisp_css_buffer *isp_css_buffer)
{
	sh_css_get_dis_projections(isp->params.dis_hor_proj_buf,
				   isp->params.dis_ver_proj_buf,
				   isp_css_buffer->dis_data);
}

int atomisp_css_dequeue_event(struct atomisp_css_event *current_event)
{
	if (sh_css_dequeue_event(&current_event->pipe, &current_event->event)
	    != sh_css_success)
		return -EINVAL;

	return 0;
}
