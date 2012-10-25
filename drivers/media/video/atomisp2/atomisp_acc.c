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

#include <linux/errno.h>
#include <linux/atomisp.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <asm/page.h>

#include "sh_css.h"
#include "sh_css_accelerate.h"
#include "memory_access/memory_access.h"
#include "hrt/hive_isp_css_mm_hrt.h"
#include "atomisp_acc.h"

#define ATOMISP_TO_CSS_MEMORY(m)	((enum sh_css_isp_memories)(m))

/*
 * __acc_get_fw - Return pointer to firmware with given handle
 */
static struct sh_css_fw_info *
__acc_get_fw(struct atomisp_device *isp, unsigned int handle)
{
	if (handle >= ATOMISP_ACC_FW_MAX)
		return NULL;

	return isp->acc.fw[handle];
}

static void
__acc_fw_free(struct atomisp_device *isp, struct sh_css_fw_info *fw)
{
	unsigned int i = fw->handle;

	/* Sanity check */
	if (i >= ATOMISP_ACC_FW_MAX || isp->acc.fw[i] != fw) {
		WARN_ON(1);
		return;
	}

	isp->acc.fw[i] = NULL;
	isp->acc.fw_count--;
	vfree(fw);
}

static struct sh_css_fw_info *
__acc_fw_alloc(struct atomisp_device *isp, struct atomisp_acc_fw_load *user_fw)
{
	struct sh_css_fw_info *fw;
	int ret;
	int i;

	if (user_fw->data == NULL || user_fw->size <= 0)
		return ERR_PTR(-EINVAL);

	if (isp->acc.fw_count >= ATOMISP_ACC_FW_MAX)
		return ERR_PTR(-E2BIG);

	fw = vmalloc(user_fw->size);
	if (fw == NULL) {
		v4l2_err(&atomisp_dev, "%s: Failed to alloc acc fw blob\n",
			 __func__);
		return ERR_PTR(-ENOMEM);
	}

	ret = copy_from_user(fw, user_fw->data, user_fw->size);
	if (ret) {
		v4l2_err(&atomisp_dev, "%s: Failed to copy acc fw blob\n",
			 __func__);
		ret = -EFAULT;
		goto err;
	}

	/* Find first free slot */
	for (i = 0; i < ATOMISP_ACC_FW_MAX - 1; i++) {
		if (isp->acc.fw[i] == NULL)
			break;
	}
	BUG_ON(i >= ATOMISP_ACC_FW_MAX);
	user_fw->fw_handle = i;
	fw->handle = i;
	isp->acc.fw[i] = fw;
	isp->acc.fw_count++;

	return fw;

err:
	vfree(fw);
	return ERR_PTR(ret);
}

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
	struct sh_css_fw_info *fw;
	int ret = 0;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	if (isp->acc.fw_count == 0) {
		/* First firmware, create pipeline */
		isp->acc.pipeline = sh_css_create_pipeline();
		if (!isp->acc.pipeline) {
			ret = -EBADE;
			goto out;
		}
	} else {
		/* FIXME: allow only one firmware for now */
		ret = -E2BIG;
		goto out;
	}

	fw = __acc_fw_alloc(isp, user_fw);
	if (IS_ERR(fw)) {
		v4l2_err(&atomisp_dev,
			 "%s: Acceleration firmware allocation failed\n",
			 __func__);
		ret = PTR_ERR(fw);
		goto out;
	}

	if (sh_css_pipeline_add_acc_stage(isp->acc.pipeline, fw) !=
	    sh_css_success) {
		__acc_fw_free(isp, fw);
		if (isp->acc.fw_count == 0) {
			sh_css_destroy_pipeline(isp->acc.pipeline);
			isp->acc.pipeline = NULL;
		}
		ret = -EINVAL;
	}

out:
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

static int __acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
	struct sh_css_fw_info *fw;
	int ret = 0;

	fw = __acc_get_fw(isp, *handle);
	if (!fw)
		return -EINVAL;

	if (isp->acc.fw_count == 1) {
		sh_css_destroy_pipeline(isp->acc.pipeline);
		isp->acc.pipeline = NULL;
	}

	__acc_fw_free(isp, fw);

	return ret;
}

int atomisp_acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
	int ret;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);
	ret = __acc_unload(isp, handle);
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

void atomisp_acc_unload_all(struct atomisp_device *isp)
{
	struct sh_css_fw_info *fw;
	int i, ret = 0;

	if (!isp)
		return;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	for (i = 0; i < ATOMISP_ACC_FW_MAX; i++) {
		fw = isp->acc.fw[i];
		if (!fw)
			continue;

		ret = __acc_unload(isp, &fw->handle);
		if (ret) {
			v4l2_err(&atomisp_dev,
				 "%s: failed to unload firmware\n", __func__);
			break;
		}

	}

	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
}

int atomisp_acc_start(struct atomisp_device *isp, unsigned int *handle)
{
	unsigned int ret = 0;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	sh_css_start_pipeline(SH_CSS_ACC_PIPELINE, isp->acc.pipeline);
	while (!sh_css_sp_has_initialized())
		hrt_sleep();
	sh_css_init_buffer_queues();

	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_wait(struct atomisp_device *isp, unsigned int *handle)
{
	int ret = 0;
	int err;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	err = sh_css_wait_for_completion(SH_CSS_ACC_PIPELINE);
	if (err != sh_css_success)
		ret = -EIO;

	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

static struct atomisp_map *__find_map(struct atomisp_device *isp,
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

int atomisp_acc_map(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
	int pgnr;
	hrt_vaddress cssptr;
	struct atomisp_map *atomisp_map;
	int ret = 0;

	if (map->flags || !map->user_ptr || map->css_ptr)
		return -EINVAL;

	pgnr = PAGE_ALIGN(map->length) >> PAGE_SHIFT;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	if (__find_map(isp, map->css_ptr, map->length)) {
		ret = -EEXIST;
		goto out;
	}

	hrt_isp_css_mm_set_user_ptr((unsigned int)map->user_ptr, pgnr,
				    HRT_USR_PTR);
	cssptr = mmgr_malloc(map->length);
	hrt_isp_css_mm_set_user_ptr(0, 0, HRT_USR_PTR);
	if (!cssptr) {
		ret = -ENOMEM;
		goto out;
	}

	atomisp_map = kmalloc(sizeof(*atomisp_map), GFP_KERNEL);
	if (!atomisp_map) {
		ret = -ENOMEM;
		mmgr_free(cssptr);
		goto out;
	}
	atomisp_map->ptr = cssptr;
	atomisp_map->length = map->length;
	list_add(&atomisp_map->list, &isp->acc.memory_maps);

	map->css_ptr = cssptr;
out:
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

int atomisp_acc_unmap(struct atomisp_device *isp, struct atomisp_acc_map *map)
{
	struct atomisp_map *atomisp_map;
	int ret = 0;

	if (map->flags)
		return -EINVAL;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	atomisp_map = __find_map(isp, map->css_ptr, map->length);
	if (!atomisp_map) {
		ret = -EINVAL;
		goto out;
	}
	list_del(&atomisp_map->list);
	mmgr_free(atomisp_map->ptr);
	kfree(atomisp_map);
out:
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;
}

void atomisp_acc_unmap_all(struct atomisp_device *isp)
{
	struct atomisp_map *atomisp_map, *t;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	list_for_each_entry_safe(atomisp_map, t, &isp->acc.memory_maps, list) {
		list_del(&atomisp_map->list);
		mmgr_free(atomisp_map->ptr);
		kfree(atomisp_map);
	}

	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return;
}

int atomisp_acc_s_mapped_arg(struct atomisp_device *isp,
			     struct atomisp_acc_s_mapped_arg *arg)
{
	struct sh_css_fw_info *fw;
	struct atomisp_map *atomisp_map;
	struct sh_css_hmm_section par;
	enum sh_css_err err;
	int ret = 0;

	if (arg->memory > ATOMISP_ACC_MEMORY_VAMEM2)
		return -EINVAL;

	mutex_lock(&isp->input_lock);
	mutex_lock(&isp->isp_lock);

	atomisp_map = __find_map(isp, arg->css_ptr, arg->length);
	if (!atomisp_map) {
		ret = -EINVAL;
		goto out;
	}

	fw = __acc_get_fw(isp, arg->fw_handle);
	if (!fw) {
		ret = -EINVAL;
		goto out;
	}

	par.ddr_address = atomisp_map->ptr;
	par.ddr_size = atomisp_map->length;
	err = sh_css_acc_set_firmware_parameters(fw,
			ATOMISP_TO_CSS_MEMORY(arg->memory), par);
	if (err != sh_css_success)
		ret = -EIO;

out:
	mutex_unlock(&isp->isp_lock);
	mutex_unlock(&isp->input_lock);
	return ret;

}
