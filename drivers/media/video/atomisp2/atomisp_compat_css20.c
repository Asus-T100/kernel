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

#include "mmu/isp_mmu.h"
#include "mmu/sh_mmu_mrfld.h"
#include "hmm/hmm.h"

#include "atomisp_compat.h"
#include "atomisp_internal.h"
#include "atomisp_cmd.h"

static ia_css_ptr atomisp_css2_mm_alloc(size_t bytes, uint32_t attr)
{
	if (attr & IA_CSS_MEM_ATTR_CACHED)
		return (ia_css_ptr) hrt_isp_css_mm_calloc_cached(bytes);
	else if (attr & IA_CSS_MEM_ATTR_ZEROED)
		return (ia_css_ptr) hrt_isp_css_mm_calloc(bytes);
	else if (attr & IA_CSS_MEM_ATTR_CONTIGUOUS)
		return (ia_css_ptr) hrt_isp_css_mm_calloc_contiguous(bytes);
	else
		return (ia_css_ptr) hrt_isp_css_mm_calloc(bytes);
}

static void atomisp_css2_mm_free(ia_css_ptr ptr)
{
	hrt_isp_css_mm_free((void *)ptr);
}

static int atomisp_css2_mm_load(ia_css_ptr ptr, void *data, size_t bytes)
{
	return hrt_isp_css_mm_load((void *)ptr, data, bytes);
}

static int atomisp_css2_mm_store(ia_css_ptr ptr, const void *data, size_t bytes)
{
	return hrt_isp_css_mm_store((void *)ptr, data, bytes);
}

static int atomisp_css2_mm_set(ia_css_ptr ptr, int c, size_t bytes)
{
	return hrt_isp_css_mm_set((void *)ptr, c, bytes);
}

static ia_css_ptr atomisp_css2_mm_mmap(const void *ptr, const size_t size,
		   uint16_t attribute, void *context)
{
	struct hrt_userbuffer_attr *userbuffer_attr = context;
	return (ia_css_ptr)hrt_isp_css_mm_alloc_user_ptr(
			size, (unsigned int)ptr,
			userbuffer_attr->pgnr,
			userbuffer_attr->type,
			attribute & HRT_BUF_FLAG_CACHED);
}

static void atomisp_css2_hw_store_8(hrt_address addr, uint8_t data)
{
	_hrt_master_port_store_8(addr, data);
}

static void atomisp_css2_hw_store_16(hrt_address addr, uint16_t data)
{
	_hrt_master_port_store_16(addr, data);
}

static void atomisp_css2_hw_store_32(hrt_address addr, uint32_t data)
{
	_hrt_master_port_store_32(addr, data);
}

static uint8_t atomisp_css2_hw_load_8(hrt_address addr)
{
	return _hrt_master_port_load_8(addr);
}

static uint16_t atomisp_css2_hw_load_16(hrt_address addr)
{
	return _hrt_master_port_load_16(addr);
}

static uint32_t atomisp_css2_hw_load_32(hrt_address addr)
{
	return _hrt_master_port_load_32(addr);
}

static void atomisp_css2_hw_store(hrt_address addr,
				  const void *from, uint32_t n)
{
	unsigned i;
	unsigned int _to = (unsigned int)addr;
	const char *_from = (const char *)from;
	for (i = 0; i < n; i++, _to++, _from++)
		_hrt_master_port_store_8(_to , *_from);
}

static void atomisp_css2_hw_load(hrt_address addr, void *to, uint32_t n)
{
	unsigned i;
	char *_to = (char *)to;
	unsigned int _from = (unsigned int)addr;
	for (i = 0; i < n; i++, _to++, _from++)
		*_to = _hrt_master_port_load_8(_from);
}

static int hmm_get_mmu_base_addr(unsigned int *mmu_base_addr)
{
	if (sh_mmu_mrfld.get_pd_base == NULL) {
		v4l2_err(&atomisp_dev, "get mmu base address failed.\n");
		return -EINVAL;
	}

	*mmu_base_addr = sh_mmu_mrfld.get_pd_base(&bo_device.mmu,
					bo_device.mmu.base_address);
	return 0;
}

int atomisp_css_init(struct atomisp_device *isp)
{
	unsigned int mmu_base_addr;
	int ret;
	enum ia_css_err err;

	hrt_isp_css_mm_init();
	ret = hmm_get_mmu_base_addr(&mmu_base_addr);
	if (ret) {
		hrt_isp_css_mm_clear();
		return ret;
	}

	/* Init ISP */
	err = ia_css_init(&isp->css_env.isp_css_env, &isp->css_env.isp_css_fw,
			  (uint32_t)mmu_base_addr, IA_CSS_IRQ_TYPE_PULSE);
	if (err != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "css init failed --- bad firmware?\n");
		return -EINVAL;
	}

	dev_dbg(isp->dev, "sh_css_init success\n");

	return 0;
}

void atomisp_css_uninit(struct atomisp_device *isp)
{
	ia_css_uninit();
}

void atomisp_css_suspend(void)
{
	ia_css_uninit();
}

int atomisp_css_resume(struct atomisp_device *isp)
{
	unsigned int mmu_base_addr;
	int ret;

	ret = hmm_get_mmu_base_addr(&mmu_base_addr);
	if (ret) {
		dev_err(isp->dev, "get base address error.\n");
		return -EINVAL;
	}

	ret = ia_css_init(&isp->css_env.isp_css_env, &isp->css_env.isp_css_fw,
			  mmu_base_addr, IA_CSS_IRQ_TYPE_PULSE);
	if (ret) {
		dev_err(isp->dev, "re-init css failed.\n");
		return -EINVAL;
	}

	return 0;
}

int atomisp_css_irq_translate(struct atomisp_device *isp,
			      unsigned int *infos)
{
	int err;

	err = ia_css_irq_translate(infos);
	if (err != IA_CSS_SUCCESS) {
		dev_warn(isp->dev,
			  "%s:failed to translate irq (err = %d,infos = %d)\n",
			  __func__, err, *infos);
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_rx_get_irq_info(unsigned int *infos)
{
	ia_css_rx_get_irq_info(infos);
}

void atomisp_css_rx_clear_irq_info(unsigned int infos)
{
	ia_css_rx_clear_irq_info(infos);
}

int atomisp_css_irq_enable(struct atomisp_device *isp,
			    enum atomisp_css_irq_info info, bool enable)
{
	if (ia_css_irq_enable(info, enable) != IA_CSS_SUCCESS) {
		dev_warn(isp->dev, "%s:Invalid irq info.\n", __func__);
		return -EINVAL;
	}

	return 0;
}

void atomisp_set_css_env(struct atomisp_device *isp)
{
	isp->css_env.isp_css_fw = {
		.data = isp->firmware->data,
		.bytes = isp->firmware->size,
	};

	isp->css_env.isp_css_env = {
		.cpu_mem_env.alloc = atomisp_kernel_zalloc,
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
}

void atomisp_css_init_struct(struct atomisp_device *isp)
{
	isp->css_env.stream = NULL;
	for (i = 0; i < IA_CSS_PIPE_MODE_NUM; i++) {
		isp->css_env.pipes[i] = NULL;
		ia_css_pipe_config_defaults(&isp->css_env.pipe_configs[i]);
		ia_css_pipe_extra_config_defaults(
				&isp->css_env.pipe_extra_configs[i]);
	}
	ia_css_stream_config_defaults(&isp->css_env.stream_config);
	isp->css_env.curr_pipe = 0;
}

int atomisp_q_video_buffer_to_css(struct atomisp_device *isp,
			struct videobuf_vmalloc_memory *vm_mem,
			enum atomisp_css_buffer_type css_buf_type,
			enum atomisp_css_pipe_id css_pipe_id)
{
	struct ia_css_buffer css_buf = {0};
	enum ia_css_err err;

	css_buf.type = css_buf_type;
	css_buf.data.frame = vm_mem->vaddr;

	err = ia_css_pipe_enqueue_buffer(isp->css_env.pipes[css_pipe_id],
					 &css_buf);
	if (err != IA_CSS_SUCCESS)
		return -EINVAL;

	return 0;
}

int atomisp_q_s3a_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_s3a_buf *s3a_buf,
			enum atomisp_css_pipe_id css_pipe_id)
{
	struct ia_css_buffer buffer = {0};

	buffer.type = IA_CSS_BUFFER_TYPE_3A_STATISTICS;
	buffer.data.stats_3a = s3a_buf->s3a_data;
	if (ia_css_pipe_enqueue_buffer(isp->css_env.pipes[css_pipe_id],
					&buffer)) {
		dev_dbg(isp->dev, "failed to q s3a stat buffer\n");
		return -EINVAL;
	}

	return 0;
}

int atomisp_q_dis_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_dis_buf *dis_buf,
			enum atomisp_css_pipe_id css_pipe_id)
{
	struct ia_css_buffer buffer = {0};

	buffer.type = IA_CSS_BUFFER_TYPE_DIS_STATISTICS;
	buffer.data.stats_dvs = dis_buf->dis_data;
	if (ia_css_pipe_enqueue_buffer(isp->css_env.pipes[css_pipe_id],
					&buffer)) {
		dev_dbg(isp->dev, "failed to q dvs stat buffer\n");
		return -EINVAL;
	}

	return 0;
}

void atomisp_css_mmu_invalidate_cache(void)
{
	ia_css_mmu_invalidate_cache();
}

int atomisp_css_start(struct atomisp_device *isp,
			enum atomisp_css_pipe_id pipe_id, bool in_reset)
{
	int ret = 0;

	if (in_reset) {
		if (__destroy_stream(isp, true) != IA_CSS_SUCCESS)
			dev_warn(isp->dev, "destroy stream failed.\n");

		if (__destroy_pipes(isp, true) != IA_CSS_SUCCESS)
			dev_warn(isp->dev, "destroy pipe failed.\n");

		if (__create_pipe(isp) != IA_CSS_SUCCESS) {
			dev_err(isp->dev, "create pipe error.\n");
			return -EINVAL;
		}
		if (__create_stream(isp) != IA_CSS_SUCCESS) {
			dev_err(isp->dev, "create stream error.\n");
			return -EINVAL;
		}
	}

	if (ia_css_start_sp() != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "start sp error.\n");
		ret = -EINVAL;
		goto start_err;
	}

	if (ia_css_stream_start(isp->css_env.stream) != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "stream start error.\n");
		ret = -EINVAL;
		goto start_err;
	}

	isp->css_env.stream_state = CSS_STREAM_CREATED;
	return 0;

start_err:
	__destroy_stream(isp, true);
stream_err:
	__destroy_pipes(isp, true);

	/* css 2.0 API limitation: ia_css_stop_sp() could be only called after
	 * destroy all pipes
	 */
	if (ia_css_isp_has_started() &&
	   (ia_css_stop_sp() != IA_CSS_SUCCESS))
		dev_err(isp->dev, "stop sp failed.\n");
pipe_err:
	return ret;
}

static void atomisp_isp_parameters_clean_up(
				struct atomisp_css_isp_config *config)
{
	if (config->shading_table)
		ia_css_shading_table_free(config->shading_table);
	if (config->morph_table)
		ia_css_morph_table_free(config->morph_table);

	/*
	 * Set NULL to configs pointer to avoid they are set into isp again when
	 * some configs are changed and need to be updated later.
	 */
	memset(config, 0, sizeof(struct atomisp_css_isp_config));
}

void atomisp_css_update_isp_params(struct atomisp_device *isp)
{
	ia_css_stream_set_isp_config(isp->css_env.stream, &isp->params.config);
	atomisp_isp_parameters_clean_up(&isp->params.config);
}

int atomisp_css_queue_buffer(struct atomisp_device *isp,
			     enum atomisp_css_pipe_id pipe_id,
			     enum atomisp_css_buffer_type buf_type,
			     struct atomisp_css_buffer *isp_css_buffer)
{
	if (ia_css_pipe_enqueue_buffer(isp->css_env.pipes[pipe_id],
					&isp_css_buffer->css_buffer)
					!= IA_CSS_SUCCESS)
		return -EINVAL;

	return 0;
}

int atomisp_css_dequeue_buffer(struct atomisp_device *isp,
				enum atomisp_css_pipe_id pipe_id,
				enum atomisp_css_buffer_type buf_type,
				struct atomisp_css_buffer *isp_css_buffer)
{
	enum ia_css_err err;

	err = ia_css_pipe_dequeue_buffer(isp->css_env.pipes[pipe_id],
					&isp_css_buffer->css_buffer);
	if (err != IA_CSS_SUCCESS) {
		dev_err(isp->dev,
			"ia_css_pipe_dequeue_buffer failed: 0x%x\n", err);
		return -EINVAL;
	}

	return 0;
}

int atomisp_css_get_3a_statistics(struct atomisp_device *isp,
				  struct atomisp_css_buffer *isp_css_buffer)
{
	ia_css_get_3a_statistics(isp->params.s3a_user_stat,
				 isp_css_buffer->css_buffer.data.stats_3a);

	return 0;
}

void atomisp_css_get_dis_statistics(struct atomisp_device *isp,
				    struct atomisp_css_buffer *isp_css_buffer)
{
	ia_css_get_dvs_statistics(isp->params.dvs_stat,
				  isp_css_buffer->css_buffer.data.stats_dvs);
}

int atomisp_css_dequeue_event(struct atomisp_css_event *current_event)
{
	if (ia_css_dequeue_event(&current_event->event) != IA_CSS_SUCCESS)
		return -EINVAL;

	return 0;
}

int atomisp_css_input_set_resolution(struct atomisp_device *isp,
					struct v4l2_mbus_framefmt *ffmt)
{
	isp->css_env.stream_config.input_res.width = ffmt->width;
	isp->css_env.stream_config.input_res.height = ffmt->height;
	return 0;
}

void atomisp_css_input_set_binning_factor(struct atomisp_device *isp,
						unsigned int bin_factor)
{
	isp->css_env.stream_config.sensor_binning_factor = bin_factor;
}

void atomisp_css_input_set_bayer_order(struct atomisp_device *isp,
				enum atomisp_css_bayer_order bayer_order)
{
	isp->css_env.stream_config.bayer_order = bayer_order;
}

void atomisp_css_input_set_format(struct atomisp_device *isp,
					enum atomisp_css_stream_format format)
{
	isp->css_env.stream_config.format = format;
}

int atomisp_css_input_set_effective_resolution(struct atomisp_device *isp,
					unsigned int width, unsigned int height)
{
	isp->css_env.stream_config.effective_res.width = width;
	isp->css_env.stream_config.effective_res.height = height;

	return 0;
}

void atomisp_css_video_set_dis_envelope(struct atomisp_device *isp,
					unsigned int dvs_w, unsigned int dvs_h)
{
	isp->css_env.pipe_configs[0].dvs_envelope.width = dvs_w;
	isp->css_env.pipe_configs[0].dvs_envelope.height = dvs_h;
}
