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

/* Assume max number of ACC stages */
#define MAX_ACC_STAGES	20

static const struct {
	unsigned int flag;
	enum ia_css_pipe_id pipe_id;
} acc_flag_to_pipe[] = {
	{ ATOMISP_ACC_FW_LOAD_FL_PREVIEW, IA_CSS_PIPE_ID_PREVIEW },
	{ ATOMISP_ACC_FW_LOAD_FL_COPY, IA_CSS_PIPE_ID_COPY },
	{ ATOMISP_ACC_FW_LOAD_FL_VIDEO, IA_CSS_PIPE_ID_VIDEO },
	{ ATOMISP_ACC_FW_LOAD_FL_CAPTURE, IA_CSS_PIPE_ID_CAPTURE }
};

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

static int acc_stop_acceleration(struct atomisp_device *isp)
{
	void *acc_stages;
	/* FIXME: Function should take isp_subdev as parameter */
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[0];


	dev_dbg(isp->dev, ">%s\n", __func__);
	if (!isp->acc.pipeline) {
		dev_dbg(isp->dev, "<%s: no acc pipe to stop\n", __func__);
		return -ENOENT;
	}

	if (isp->streaming != ATOMISP_DEVICE_STREAMING_ENABLED) {
		dev_dbg(isp->dev, "<%s: acc stream not started\n", __func__);
		return -ENOENT;
	}

	if (wait_for_completion_interruptible(&isp->acc.acc_done) != 0) {
		dev_dbg(isp->dev, "<%s: completion interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	dev_dbg(isp->dev, "wait for acc pipeline done finished\n");

	del_timer_sync(&isp->wdt);
	cancel_work_sync(&isp->wdt_work);

	/*
	 * back up the acc stages pointer and release the memory
	 * after css stop.
	 */
	acc_stages =
		isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_ACC].
		acc_stages;
	ia_css_stop(isp, false);
	isp->streaming = ATOMISP_DEVICE_STREAMING_DISABLED;
	kfree(acc_stages);

	isp->acc.pipeline = NULL;
	dev_dbg(isp->dev, "<%s\n", __func__);
	return 0;
}

void atomisp_acc_init(struct atomisp_device *isp)
{
	INIT_LIST_HEAD(&isp->acc.fw);
	INIT_LIST_HEAD(&isp->acc.memory_maps);
	ida_init(&isp->acc.ida);
}

void atomisp_acc_cleanup(struct atomisp_device *isp)
{
	ida_destroy(&isp->acc.ida);
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

	/* Free all mapped memory blocks */
	list_for_each_entry_safe(atomisp_map, tm, &isp->acc.memory_maps, list) {
		list_del(&atomisp_map->list);
		mmgr_free(atomisp_map->ptr);
		kfree(atomisp_map);
	}
}

int atomisp_acc_load_to_pipe(struct atomisp_device *isp,
			     struct atomisp_acc_fw_load_to_pipe *user_fw)
{
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
	/*
	 * correct isp firmware type in order ISP firmware can be appended
	 * to correct pipe properly
	 */
	if (acc_fw->fw->type == ia_css_isp_firmware) {
		switch (acc_fw->type) {
		case ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT:
			acc_fw->fw->info.isp.type = IA_CSS_ACC_OUTPUT;
			break;

		case ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER:
			acc_fw->fw->info.isp.type = IA_CSS_ACC_VIEWFINDER;
			break;

		case ATOMISP_ACC_FW_LOAD_TYPE_STANDALONE:
			acc_fw->fw->info.isp.type = IA_CSS_ACC_STANDALONE;
			break;
		}
	}

	list_add_tail(&acc_fw->list, &isp->acc.fw);
	return 0;
}

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *user_fw)
{
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
}

int atomisp_acc_unload(struct atomisp_device *isp, unsigned int *handle)
{
	struct atomisp_acc_fw *acc_fw;

	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	acc_fw = acc_get_fw(isp, *handle);
	if (!acc_fw)
		return -EINVAL;

	list_del(&acc_fw->list);
	ida_remove(&isp->acc.ida, acc_fw->handle);
	acc_free_fw(acc_fw);

	return 0;
}

/*[> Set the binary arguments <]*/
static int acc_set_parameters(struct atomisp_acc_fw *acc_fw)
{
	struct ia_css_data sec;
	enum ia_css_isp_memories mem;

	for (mem = 0; mem < N_IA_CSS_ISP_MEMORIES; mem++) {
		if (acc_fw->args[mem].length == 0)
			continue;

		sec.address = acc_fw->args[mem].css_ptr;
		sec.size = acc_fw->args[mem].length;
		if (sh_css_acc_set_firmware_parameters(acc_fw->fw, mem, sec)
			!= IA_CSS_SUCCESS)
			return -EIO;
	}

	return 0;
}

int atomisp_acc_start(struct atomisp_device *isp, unsigned int *handle)
{
	struct atomisp_acc_fw *acc_fw;
	enum ia_css_err err;
	struct ia_css_pipe_config *pipe_config;
	/* FIXME: Function should take isp_subdev as parameter */
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[0];
	const struct ia_css_stream_config *s_config =
			&isp_subdev->css2_basis.stream_config;
	int ret = 0;

	dev_dbg(isp->dev, ">%s\n", __func__);
	if (isp->acc.pipeline || isp->acc.extension_mode)
		return -EBUSY;

	/* Invalidate caches. FIXME: should flush only necessary buffers */
	wbinvd();

	if (isp_subdev->css2_basis.stream) {
		dev_dbg(isp->dev, "%s: destroy basis stream\n", __func__);
		if (isp_subdev->css2_basis.stream_state ==
		    CSS2_STREAM_STARTED) {
			ret = ia_css_stream_stop(isp_subdev->css2_basis.stream);
			if (ret != IA_CSS_SUCCESS) {
				dev_err(isp->dev, "stop stream failed.\n");
				return -EBUSY;
			}
		}

		ret = ia_css_stream_destroy(isp_subdev->css2_basis.stream);
		if (ret != IA_CSS_SUCCESS) {
			dev_err(isp->dev, "destroy stream failed.\n");
			return -EBUSY;
		}
		isp_subdev->css2_basis.stream = NULL;
	}

	pipe_config = &isp_subdev->css2_basis.pipe_configs[IA_CSS_PIPE_ID_ACC];
	ia_css_pipe_config_defaults(pipe_config);
	pipe_config->acc_stages = kzalloc(MAX_ACC_STAGES *
				sizeof(struct ia_css_fw_info *), GFP_KERNEL);
	if (!pipe_config->acc_stages)
		return -ENOMEM;
	pipe_config->mode = IA_CSS_PIPE_MODE_ACC;
	pipe_config->num_acc_stages = 0;

	list_for_each_entry(acc_fw, &isp->acc.fw, list) {
		if (*handle != 0 && *handle != acc_fw->handle)
			continue;

		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_STANDALONE)
			continue;

		/* Add the binary into the pipeline */
		pipe_config->acc_stages[pipe_config->num_acc_stages++] =
								acc_fw->fw;
		if (acc_set_parameters(acc_fw)) {
			dev_err(isp->dev, "acc_set_parameters failed\n");
			ret = -EINVAL;
			goto err;
		}
	}

	if (pipe_config->num_acc_stages < 1) {
		/* Refuse creating pipelines with no binaries */
		dev_err(isp->dev, "%s: no acc binary available\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	err = ia_css_pipe_create(pipe_config,
			&isp_subdev->css2_basis.pipes[IA_CSS_PIPE_ID_ACC]);
	if (err) {
		dev_err(isp->dev, "%s: ia_css_pipe_create failed %d\n",
				__func__, err);
		ret = -EBADE;
		goto err;
	}

	memset(&isp_subdev->css2_basis.stream_config, 0,
		sizeof(struct ia_css_stream_config));
	err = ia_css_stream_create(s_config, 1,
				&isp_subdev->css2_basis.
				pipes[IA_CSS_PIPE_ID_ACC],
				&isp_subdev->css2_basis.stream);
	if (err != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "%s: create stream error %d.\n",
			__func__, err);
		ia_css_pipe_destroy(isp_subdev->css2_basis.
				    pipes[IA_CSS_PIPE_ID_ACC]);
		isp_subdev->css2_basis.pipes[IA_CSS_PIPE_ID_ACC] = NULL;
		err = -EINVAL;
		goto err;
	}
	dev_dbg(isp->dev, ">ia_css_start\n");
	err = ia_css_start(isp, false);
	if (err != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "%s: start css error %d.\n", __func__, err);
		ret = ia_css_stream_destroy(isp_subdev->css2_basis.stream);
		isp_subdev->css2_basis.stream = NULL;
		ia_css_pipe_destroy(isp_subdev->css2_basis.
				    pipes[IA_CSS_PIPE_ID_ACC]);
		isp_subdev->css2_basis.pipes[IA_CSS_PIPE_ID_ACC] = NULL;
		err = -EINVAL;
		goto err;
	}
	dev_dbg(isp->dev, "<ia_css_start\n");

	isp->streaming = ATOMISP_DEVICE_STREAMING_ENABLED;
	init_completion(&isp->acc.acc_done);
	atomic_set(&isp->wdt_count, 0);
	mod_timer(&isp->wdt, jiffies + ATOMISP_ISP_TIMEOUT_DURATION);
	isp->fr_status = ATOMISP_FRAME_STATUS_OK;
	isp->sw_contex.invalid_frame = false;
	isp_subdev->params.dvs_proj_data_valid = false;
	isp->acc.pipeline = isp_subdev->css2_basis.pipes[IA_CSS_PIPE_ID_ACC];

	dev_dbg(isp->dev, "<%s\n", __func__);
	return 0;
err:
	kfree(pipe_config->acc_stages);
	dev_dbg(isp->dev, "<%s err %d\n", __func__, ret);
	return ret;
}

int atomisp_acc_wait(struct atomisp_device *isp, unsigned int *handle)
{
	if (*handle && !acc_get_fw(isp, *handle))
		return -EINVAL;

	return acc_stop_acceleration(isp);
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

	dev_dbg(isp->dev, "%s: userptr 0x%x, css_address 0x%x, size %d\n",
		__func__, (unsigned int)map->user_ptr, cssptr, map->length);
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

	if (arg->memory >= N_IA_CSS_ISP_MEMORIES)
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

	dev_dbg(isp->dev, "%s: mem %d, address 0x%x, size %d\n",
		__func__, arg->memory, (unsigned int)arg->css_ptr, arg->length);
	return 0;
}

/*
 * Appends the loaded acceleration binary extensions to the
 * current ISP mode. Must be called just before ia_css_start().
 */
int atomisp_acc_load_extensions(struct atomisp_device *isp)
{
	struct atomisp_acc_fw *acc_fw;
	/* FIXME: Function should take isp_subdev as parameter */
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[0];

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
			if  (acc_fw->flags & acc_flag_to_pipe[i].flag) {
				unsigned int pipe_id =
					acc_flag_to_pipe[i].pipe_id;
				struct ia_css_pipe_config *pipe_cfg =
					&isp_subdev->css2_basis.
					pipe_configs[pipe_id];

				pipe_cfg->acc_extension = acc_fw->fw;
				isp_subdev->css2_basis.update_pipe[pipe_id] =
				    true;
			}
		}

		ret = acc_set_parameters(acc_fw);
		if (ret < 0)
			goto error;
	}

	if (ia_css_update_stream(isp) != IA_CSS_SUCCESS) {
		dev_err(isp->dev, "%s: update stream failed.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	isp->acc.extension_mode = true;
	return 0;

error:
	list_for_each_entry_continue_reverse(acc_fw, &isp->acc.fw, list) {
		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT &&
		    acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER)
			continue;

		for (i = ARRAY_SIZE(acc_flag_to_pipe) - 1; i >= 0; i--) {
			unsigned int pipe_id =
				acc_flag_to_pipe[i].pipe_id;
			struct ia_css_pipe_config *pipe_cfg =
				&isp_subdev->css2_basis.pipe_configs[pipe_id];

			if  (acc_fw->flags & acc_flag_to_pipe[i].flag)
				pipe_cfg->acc_extension = NULL;
		}
	}
	return ret;
}

void atomisp_acc_unload_extensions(struct atomisp_device *isp)
{
	struct atomisp_acc_fw *acc_fw;
	/* FIXME: Function should take isp_subdev as parameter */
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev[0];

	int i;

	if (!isp->acc.extension_mode)
		return;

	list_for_each_entry_reverse(acc_fw, &isp->acc.fw, list) {
		if (acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_OUTPUT &&
		    acc_fw->type != ATOMISP_ACC_FW_LOAD_TYPE_VIEWFINDER)
			continue;

		for (i = ARRAY_SIZE(acc_flag_to_pipe) - 1; i >= 0; i--) {
			unsigned int pipe_id =
				acc_flag_to_pipe[i].pipe_id;
			struct ia_css_pipe_config *pipe_cfg =
				&isp_subdev->css2_basis.pipe_configs[pipe_id];

			if  (acc_fw->flags & acc_flag_to_pipe[i].flag)
				pipe_cfg->acc_extension = NULL;
		}
	}

	isp->acc.extension_mode = false;
}
