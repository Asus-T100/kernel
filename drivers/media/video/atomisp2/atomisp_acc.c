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
#include "sh_css.h"
#include "sh_css_accelerate.h"

#define ATOMISP_TO_CSS_MEMORY(m)	((enum sh_css_isp_memories)(m))

/*
 * Allocate struct atomisp_acc_fw along with space for firmware.
 * The returned struct atomisp_acc_fw is cleared (firmware region is not).
 */
static struct atomisp_acc_fw *acc_alloc_fw(unsigned int fw_size)
{
	struct atomisp_acc_fw *acc_fw;

	acc_fw = kzalloc(sizeof(*acc_fw), GFP_KERNEL);
	if (!acc_fw)
		return NULL;

	acc_fw->fw = vmalloc(fw_size);
	if (!acc_fw->fw) {
		kfree(acc_fw);
		return NULL;
	}

	return acc_fw;
}

static void acc_free_fw(struct atomisp_acc_fw *acc_fw)
{
	vfree(acc_fw->fw);
	kfree(acc_fw);
}

static struct atomisp_acc_fw *
acc_get_fw(struct atomisp_device *isp, unsigned int handle)
{
	struct atomisp_acc_fw *acc_fw;

	list_for_each_entry(acc_fw, &isp->acc.fw, list)
		if (acc_fw->handle == handle)
			return acc_fw;

	return NULL;
}

static struct atomisp_map *acc_get_map(struct atomisp_device *isp,
				       unsigned long css_ptr, size_t length)
{
	struct atomisp_map *atomisp_map;

	list_for_each_entry(atomisp_map, &isp->acc.memory_maps, list) {
		if (atomisp_map->ptr == css_ptr &&
		    atomisp_map->length == length)
			return atomisp_map;
	}
	return NULL;
}

static void acc_stop_acceleration(struct atomisp_device *isp)
{
	if (sh_css_acceleration_stop() != sh_css_success)
		dev_err(isp->dev, "cannot stop acceleration pipeline\n");

	sh_css_destroy_pipeline(isp->acc.pipeline);
	isp->acc.pipeline = NULL;
}

void atomisp_acc_init(struct atomisp_device *isp)
{
	INIT_LIST_HEAD(&isp->acc.fw);
	INIT_LIST_HEAD(&isp->acc.memory_maps);
	ida_init(&isp->acc.ida);
}

void atomisp_acc_release(struct atomisp_device *isp)
{
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

	ida_destroy(&isp->acc.ida);

	/* Free all mapped memory blocks */
	list_for_each_entry_safe(atomisp_map, tm, &isp->acc.memory_maps, list) {
		list_del(&atomisp_map->list);
		mmgr_free(atomisp_map->ptr);
		kfree(atomisp_map);
	}
}

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
	struct atomisp_acc_fw *acc_fw;
	int handle;

	if (!user_fw->data || user_fw->size == 0)
		return -EINVAL;

	if (isp->acc.pipeline)
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
	list_add_tail(&acc_fw->list, &isp->acc.fw);
	return 0;
}

int atomisp_acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
	struct atomisp_acc_fw *acc_fw;

	if (isp->acc.pipeline)
		return -EBUSY;

	acc_fw = acc_get_fw(isp, *handle);
	if (!acc_fw)
		return -EINVAL;

	list_del(&acc_fw->list);
	ida_remove(&isp->acc.ida, acc_fw->handle);
	acc_free_fw(acc_fw);
	return 0;
}

int atomisp_acc_start(struct atomisp_device *isp, unsigned int *handle)
{
	struct atomisp_acc_fw *acc_fw;
	struct sh_css_hmm_section sec;
	int ret;
	unsigned int mem;

	if (isp->sw_contex.isp_streaming || isp->acc.pipeline)
		return -EBUSY;

	/* Invalidate caches. FIXME: should flush only necessary buffers */
	wbinvd();

	isp->acc.pipeline = sh_css_create_pipeline();
	if (!isp->acc.pipeline)
		return -EBADE;

	list_for_each_entry(acc_fw, &isp->acc.fw, list) {
		if (*handle != 0 && *handle != acc_fw->handle)
			continue;

		/* Add the binary into the pipeline */
		if (sh_css_pipeline_add_acc_stage(
		    isp->acc.pipeline, acc_fw->fw) != sh_css_success) {
			ret = -EBADSLT;
			goto err_stage;
		}

		/* Set the binary arguments */
		for (mem = 0; mem < ATOMISP_ACC_NR_MEMORY; mem++) {
			if (acc_fw->args[mem].length == 0)
				continue;

			sec.ddr_address = acc_fw->args[mem].css_ptr;
			sec.ddr_size = acc_fw->args[mem].length;
			if (sh_css_acc_set_firmware_parameters(acc_fw->fw,
			   ATOMISP_TO_CSS_MEMORY(mem), sec) != sh_css_success) {
				ret = -EIO;
				goto err_stage;
			}
		}
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
}

int atomisp_acc_wait(struct atomisp_device *isp, unsigned int *handle)
{
	if (!isp->acc.pipeline)
		return -ENOENT;

	if (*handle && !acc_get_fw(isp, *handle))
		return -EINVAL;

	if (sh_css_wait_for_completion(SH_CSS_ACC_PIPELINE) != sh_css_success)
		return -EIO;
	acc_stop_acceleration(isp);

	return 0;
}

int atomisp_acc_map(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
	int pgnr;
	hrt_vaddress cssptr;
	struct atomisp_map *atomisp_map;

	if (map->flags || !map->user_ptr || map->css_ptr)
		return -EINVAL;

	if (isp->acc.pipeline)
		return -EBUSY;

	/* FIXME: should implement mmgr_map() and use it instead */
	pgnr = DIV_ROUND_UP(map->length, PAGE_SIZE);
	hrt_isp_css_mm_set_user_ptr((unsigned int)map->user_ptr, pgnr,
				    HRT_USR_PTR);
	cssptr = mmgr_malloc(map->length);
	hrt_isp_css_mm_set_user_ptr(0, 0, HRT_USR_PTR);
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
	return 0;
}

int atomisp_acc_unmap(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
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
	return 0;
}

int atomisp_acc_s_mapped_arg(struct atomisp_device *isp,
			     struct atomisp_acc_s_mapped_arg *arg)
{
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
	return 0;
}
