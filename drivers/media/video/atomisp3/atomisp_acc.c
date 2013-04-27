/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
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

/*
 * This file implements loadable acceleration firmware API,
 * including ioctls to map and unmap acceleration parameters and buffers.
 */

#include <linux/init.h>

#include "atomisp_acc.h"
#include "atomisp_internal.h"
#include "atomisp_compat.h"

#include "hrt/hive_isp_css_mm_hrt.h"
#include "memory_access/memory_access.h"
#include "ia_css.h"
#include "ia_css_accelerate.h"

#ifdef CONFIG_X86_MRFLD
#define sh_css_load_extension(fw, pipe_id, acc_type) sh_css_load_extension(fw)
#define sh_css_unload_extension(fw, pipe_id) sh_css_unload_extension(fw)
#endif

static const struct {
	unsigned int flag;
	enum ia_css_pipe_id pipe_id;
} acc_flag_to_pipe[] = {
	{ ATOMISP_ACC_FW_LOAD_FL_PREVIEW, IA_CSS_PIPE_ID_PREVIEW },
	{ ATOMISP_ACC_FW_LOAD_FL_COPY, IA_CSS_PIPE_ID_COPY },
	{ ATOMISP_ACC_FW_LOAD_FL_VIDEO, IA_CSS_PIPE_ID_VIDEO },
	{ ATOMISP_ACC_FW_LOAD_FL_CAPTURE, IA_CSS_PIPE_ID_CAPTURE },
};
#if 0
/*
 * Allocate struct atomisp_acc_fw along with space for firmware.
 * The returned struct atomisp_acc_fw is cleared (firmware region is not).
 */
static struct atomisp_acc_fw *acc_alloc_fw(unsigned int fw_size)
{
	/*struct atomisp_acc_fw *acc_fw;*/

	/*acc_fw = kzalloc(sizeof(*acc_fw), GFP_KERNEL);*/
	/*if (!acc_fw)*/
		/*return NULL;*/

	/*acc_fw->fw = vmalloc(fw_size);*/
	/*if (!acc_fw->fw) {*/
		/*kfree(acc_fw);*/
		/*return NULL;*/
	/*}*/

	/*return acc_fw;*/
}

static void acc_free_fw(struct atomisp_acc_fw *acc_fw)
{
	/*vfree(acc_fw->fw);*/
	/*kfree(acc_fw);*/
}

static struct atomisp_acc_fw *
acc_get_fw(struct atomisp_device *isp, unsigned int handle)
{
	/*struct atomisp_acc_fw *acc_fw;*/

	/*list_for_each_entry(acc_fw, &isp->acc.fw, list)*/
		/*if (acc_fw->handle == handle)*/
			/*return acc_fw;*/

	/*return NULL;*/
}

static struct atomisp_map *acc_get_map(struct atomisp_device *isp,
				       unsigned long css_ptr, size_t length)
{
	/*struct atomisp_map *atomisp_map;*/

	/*list_for_each_entry(atomisp_map, &isp->acc.memory_maps, list) {*/
		/*if (atomisp_map->ptr == css_ptr &&*/
		    /*atomisp_map->length == length)*/
			/*return atomisp_map;*/
	/*}*/
	/*return NULL;*/
}
#endif
/*static void acc_stop_acceleration(struct atomisp_device *isp)*/
/*{*/
	/*[>if (ia_css_acceleration_stop() != IA_CSS_SUCCESS)<]*/
		/*[>dev_err(isp->dev, "cannot stop acceleration pipeline\n");<]*/

	/*[>sh_css_destroy_pipeline(isp->acc.pipeline);<]*/
	/*[>isp->acc.pipeline = NULL;<]*/
/*}*/

void atomisp_acc_init(struct atomisp_device *isp)
{
	/*INIT_LIST_HEAD(&isp->acc.fw);*/
	/*INIT_LIST_HEAD(&isp->acc.memory_maps);*/
	/*ida_init(&isp->acc.ida);*/
}

void atomisp_acc_cleanup(struct atomisp_device *isp)
{
	/*ida_destroy(&isp->acc.ida);*/
}

void atomisp_acc_release(struct atomisp_device *isp)
{
#if 0
	struct atomisp_acc_fw *acc_fw, *ta;
	struct atomisp_map *atomisp_map, *tm;

	/* Stop acceleration if already running */
	if (isp->acc.pipeline)
		acc_stop_acceleration(isp);

	/* Unload all loaded acceleration binaries */
	list_for_each_entry_safe(acc_fw, ta, &isp->acc.fw, list) {
		list_del(&acc_fw->list);
		ida_remove(&isp->acc.ida, acc_fw->handle);
		acc_free_fw(acc_fw);
	}

	/* Free all mapped memory blocks */
	list_for_each_entry_safe(atomisp_map, tm, &isp->acc.memory_maps, list) {
		list_del(&atomisp_map->list);
		mmgr_free(atomisp_map->ptr);
		kfree(atomisp_map);
	}
#endif
}

int atomisp_acc_load_to_pipe(struct atomisp_device *isp,
			     struct atomisp_acc_fw_load_to_pipe *user_fw)
{
#if 0
	static const unsigned int pipeline_flags =
		ATOMISP_ACC_FW_LOAD_FL_PREVIEW |
		ATOMISP_ACC_FW_LOAD_FL_COPY |
		ATOMISP_ACC_FW_LOAD_FL_VIDEO |
		ATOMISP_ACC_FW_LOAD_FL_CAPTURE |
		ATOMISP_ACC_FW_LOAD_FL_ACC;

	struct atomisp_acc_fw *acc_fw;
	int handle;

	if (!user_fw->data || user_fw->size == 0)
		return -EINVAL;

	/* Binary has to be enabled at least for one pipeline */
	if (!(user_fw->flags & pipeline_flags))
		return -EINVAL;

	/* We do not support other flags yet */
	if (user_fw->flags & ~pipeline_flags)
		return -EINVAL;

	if (user_fw->type < ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT ||
	    user_fw->type > ATOMISP_ACC_FW_LOAD_TYPE_STANDALONE)
		return -EINVAL;

	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	acc_fw = acc_alloc_fw(user_fw->size);
	if (!acc_fw)
		return -ENOMEM;

	if (copy_from_user(acc_fw->fw, user_fw->data, user_fw->size)) {
		acc_free_fw(acc_fw);
		return -EFAULT;
	}

	if (!ida_pre_get(&isp->acc.ida, GFP_KERNEL) ||
	    ida_get_new_above(&isp->acc.ida, 1, &handle)) {
		acc_free_fw(acc_fw);
		return -ENOSPC;
	}

	user_fw->fw_handle = handle;
	acc_fw->handle = handle;
	acc_fw->flags = user_fw->flags;
	acc_fw->type = user_fw->type;
	list_add_tail(&acc_fw->list, &isp->acc.fw);
#endif
	return 0;
}

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
#if 0
	struct atomisp_acc_fw_load_to_pipe ltp;
	int r;

	memset(&ltp, 0, sizeof(ltp));
	ltp.flags = ATOMISP_ACC_FW_LOAD_FL_ACC;
	ltp.type = ATOMISP_ACC_FW_LOAD_TYPE_STANDALONE;
	ltp.size = user_fw->size;
	ltp.data = user_fw->data;
	r = atomisp_acc_load_to_pipe(isp, &ltp);
	user_fw->fw_handle = ltp.fw_handle;
	return r;
#endif
	return 0;
}

int atomisp_acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
#if 0
	struct atomisp_acc_fw *acc_fw;

	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	acc_fw = acc_get_fw(isp, *handle);
	if (!acc_fw)
		return -EINVAL;

	list_del(&acc_fw->list);
	ida_remove(&isp->acc.ida, acc_fw->handle);
	acc_free_fw(acc_fw);
#endif
	return 0;
}

/*[> Set the binary arguments <]*/
/*static int acc_set_parameters(struct atomisp_acc_fw *acc_fw)*/
/*{*/
/*#if 0*/
	/*struct sh_css_hmm_section sec;*/
	/*unsigned int mem;*/

	/*for (mem = 0; mem < ATOMISP_ACC_NR_MEMORY; mem++) {*/
		/*if (acc_fw->args[mem].length == 0)*/
			/*continue;*/

		/*sec.ddr_address = acc_fw->args[mem].css_ptr;*/
		/*sec.ddr_size = acc_fw->args[mem].length;*/
		/*if (sh_css_acc_set_firmware_parameters(acc_fw->fw, mem, sec)*/
		    /*!= sh_css_success)*/
			/*return -EIO;*/
	/*}*/
/*#endif*/
	/*return 0;*/
/*}*/
int atomisp_acc_start(struct atomisp_device *isp, unsigned int *handle)
{
#if 0
	struct atomisp_acc_fw *acc_fw;
	int ret;
	unsigned int nbin;

	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	/* Invalidate caches. FIXME: should flush only necessary buffers */
	wbinvd();

	isp->acc.pipeline = sh_css_create_pipeline();
	if (!isp->acc.pipeline)
		return -EBADE;

	nbin = 0;
	list_for_each_entry(acc_fw, &isp->acc.fw, list) {
		if (*handle != 0 && *handle != acc_fw->handle)
			continue;

		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_STANDALONE)
			continue;

		/* Add the binary into the pipeline */
		if (sh_css_pipeline_add_acc_stage(
		    isp->acc.pipeline, acc_fw->fw) != sh_css_success) {
			ret = -EBADSLT;
			goto err_stage;
		}
		nbin++;

		ret = acc_set_parameters(acc_fw);
		if (ret < 0)
			goto err_stage;
	}
	if (nbin < 1) {
		/* Refuse creating pipelines with no binaries */
		ret = -EINVAL;
		goto err_stage;
	}

	sh_css_start_pipeline(SH_CSS_ACC_PIPELINE, isp->acc.pipeline);
	while (!sh_css_sp_has_booted())
		cpu_relax();
	sh_css_init_buffer_queues();
	return 0;

err_stage:
	sh_css_destroy_pipeline(isp->acc.pipeline);
	isp->acc.pipeline = NULL;
	return ret;
#endif
	return 0;
}

int atomisp_acc_wait(struct atomisp_device *isp, unsigned int *handle)
{
#if 0
	if (!isp->acc.pipeline)
		return -ENOENT;

	if (*handle && !acc_get_fw(isp, *handle))
		return -EINVAL;

	if (sh_css_wait_for_completion(SH_CSS_ACC_PIPELINE) != sh_css_success)
		return -EIO;
	acc_stop_acceleration(isp);
#endif
	return 0;
}

int atomisp_acc_map(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
#if 0
	int pgnr;
	hrt_vaddress cssptr;
	struct atomisp_map *atomisp_map;

	if (map->flags || !map->user_ptr || map->css_ptr)
		return -EINVAL;

	if (isp->acc.pipeline)
		return -EBUSY;

	pgnr = DIV_ROUND_UP(map->length, PAGE_SIZE);
	cssptr = (hrt_vaddress)hrt_isp_css_mm_alloc_user_ptr(
			map->length, (unsigned int)map->user_ptr,
			pgnr, HRT_USR_PTR, false);
	if (!cssptr)
		return -ENOMEM;

	atomisp_map = kmalloc(sizeof(*atomisp_map), GFP_KERNEL);
	if (!atomisp_map) {
		mmgr_free(cssptr);
		return -ENOMEM;
	}
	atomisp_map->ptr = cssptr;
	atomisp_map->length = map->length;
	list_add(&atomisp_map->list, &isp->acc.memory_maps);

	map->css_ptr = cssptr;
#endif
	return 0;
}

int atomisp_acc_unmap(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
#if 0
	struct atomisp_map *atomisp_map;

	if (map->flags)
		return -EINVAL;

	if (isp->acc.pipeline)
		return -EBUSY;

	atomisp_map = acc_get_map(isp, map->css_ptr, map->length);
	if (!atomisp_map)
		return -EINVAL;

	list_del(&atomisp_map->list);
	mmgr_free(atomisp_map->ptr);
	kfree(atomisp_map);
#endif
	return 0;
}

int atomisp_acc_s_mapped_arg(struct atomisp_device *isp,
			     struct atomisp_acc_s_mapped_arg *arg)
{
#if 0
	struct atomisp_acc_fw *acc_fw;

	if (arg->memory >= ATOMISP_ACC_NR_MEMORY)
		return -EINVAL;

	if (isp->acc.pipeline)
		return -EBUSY;

	acc_fw = acc_get_fw(isp, arg->fw_handle);
	if (!acc_fw)
		return -EINVAL;

	if (arg->css_ptr != 0 || arg->length != 0) {
		/* Unless the parameter is cleared, check that it exists */
		if (!acc_get_map(isp, arg->css_ptr, arg->length))
			return -EINVAL;
	}

	acc_fw->args[arg->memory].length = arg->length;
	acc_fw->args[arg->memory].css_ptr = arg->css_ptr;
#endif
	return 0;
}

/*
 * Appends the loaded acceleration binary extensions to the
 * current ISP mode. Must be called just before sh_css_start().
 */
int atomisp_acc_load_extensions(struct atomisp_device *isp)
{
#if 0
	struct atomisp_acc_fw *acc_fw;
	int ret, i;

	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	/* Invalidate caches. FIXME: should flush only necessary buffers */
	wbinvd();

	list_for_each_entry(acc_fw, &isp->acc.fw, list) {
		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT &&
		    acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER)
			continue;

		for (i = 0; i < ARRAY_SIZE(acc_flag_to_pipe); i++) {
			if (acc_fw->flags & acc_flag_to_pipe[i].flag) {
				/* Add the binary into the pipeline */
				if (sh_css_load_extension(acc_fw->fw,
					     acc_flag_to_pipe[i].pipe_id,
					     acc_fw->type) != sh_css_success) {
					i--;
					ret = -EBADSLT;
					goto error;
				}
			}
		}

		ret = acc_set_parameters(acc_fw);
		if (ret < 0)
			goto error;
	}

	isp->acc.extension_mode = true;
	return 0;

error:
	for (; i >= 0; i--) {
		if (acc_fw->flags & acc_flag_to_pipe[i].flag) {
			sh_css_unload_extension(acc_fw->fw,
				acc_flag_to_pipe[i].pipe_id);
		}
	}

	list_for_each_entry_continue_reverse(acc_fw, &isp->acc.fw, list) {
		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT &&
		    acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER)
			continue;

		for (i = ARRAY_SIZE(acc_flag_to_pipe) - 1; i >= 0; i--) {
			if (acc_fw->flags & acc_flag_to_pipe[i].flag) {
				sh_css_unload_extension(acc_fw->fw,
					acc_flag_to_pipe[i].pipe_id);
			}
		}
	}
	return ret;
#endif
	return 0;
}

void atomisp_acc_unload_extensions(struct atomisp_device *isp)
{
#if 0
	struct atomisp_acc_fw *acc_fw;
	int i;

	if (!isp->acc.extension_mode)
		return;

	list_for_each_entry_reverse(acc_fw, &isp->acc.fw, list) {
		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT &&
		    acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER)
			continue;

		for (i = ARRAY_SIZE(acc_flag_to_pipe) - 1; i >= 0; i--) {
			if (acc_fw->flags & acc_flag_to_pipe[i].flag) {
				/* Remove the binary from the pipeline */
				sh_css_unload_extension(acc_fw->fw,
					acc_flag_to_pipe[i].pipe_id);
			}
		}
	}

	isp->acc.extension_mode = false;
#endif
}
