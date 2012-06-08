/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_esd.h"
#include "mdfld_dsi_dbi.h"

#define MDFLD_ESD_SLEEP_MSECS	8000

static int __esd_thread(void *data)
{
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct panel_funcs *p_funcs  = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct mdfld_dsi_error_detector *err_detector =
		(struct mdfld_dsi_error_detector *)data;
	struct drm_device *dev = err_detector->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	set_freezable();

	while (!kthread_should_stop()) {
		wait_event_freezable(err_detector->esd_thread_wq,
			((dev_priv->dbi_panel_on) || kthread_should_stop()));

		PSB_DEBUG_ENTRY("%s: executed on %u\n", __func__,
			jiffies_to_msecs(jiffies));

		if (dev_priv->cur_pipe == 0) {
			dbi_output = dev_priv->dbi_output;
			dsi_config = dev_priv->dsi_configs[0];
		} else {
			dbi_output = dev_priv->dbi_output2;
			dsi_config = dev_priv->dsi_configs[1];
		}

		if (!dbi_output)
			goto esd_exit;

		p_funcs = dbi_output->p_funcs;
		if (p_funcs && (p_funcs->esd_detection)
			&& dev_priv->dbi_panel_on) {
			if (dev_priv->b_dsr_enable) {
				dev_priv->exit_idle(dev,
						MDFLD_DSR_2D_3D,
						NULL,
						0);
				/*make sure, during esd no DSR again*/
				dbi_output->mode_flags |= MODE_SETTING_ON_GOING;
			}
			if (p_funcs->esd_detection(dsi_config)) {
				printk(KERN_ALERT"ESD\n");
				schedule_work(&dev_priv->reset_panel_work);
			}
			if (dev_priv->b_dsr_enable) {
				dbi_output->mode_flags &= ~MODE_SETTING_ON_GOING;
			}

		}
esd_exit:
		schedule_timeout_interruptible(
			msecs_to_jiffies(MDFLD_ESD_SLEEP_MSECS));
	}

	DRM_INFO("ESD exited\n");
	return 0;
}

/**
 * Wake up the error detector
 */
void mdfld_dsi_error_detector_wakeup(struct mdfld_dsi_connector *dsi_connector)
{
	struct mdfld_dsi_error_detector *err_detector;

	if (!dsi_connector || !dsi_connector->err_detector)
		return;

	err_detector = dsi_connector->err_detector;
	wake_up_interruptible(&err_detector->esd_thread_wq);
}

/**
 * initialize DSI panel error detector
 */
int mdfld_dsi_error_detector_init(struct drm_device *dev,
	struct mdfld_dsi_connector *dsi_connector)
{
	struct mdfld_dsi_error_detector *err_detector;
	const char *fmt = "%s";
	struct task_struct *p;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO - 1};

	if (!dsi_connector || !dev) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	if (dsi_connector->err_detector)
		return 0;

	/*create a new error detector*/
	err_detector = kzalloc(sizeof(struct mdfld_dsi_error_detector),
				GFP_KERNEL);
	if (!err_detector) {
		DRM_ERROR("Failed to allocate ESD\n");
		return -ENOMEM;
	}

	/*init detector thread wait queue*/
	init_waitqueue_head(&err_detector->esd_thread_wq);

	/*init detector thread*/
	p = kthread_create(__esd_thread, err_detector, fmt, "dsi_esd", 0);
	if (IS_ERR(p)) {
		DRM_ERROR("Failed to create ESD thread\n");
		goto esd_thread_err;
	}
	/*use FIFO scheduler*/
	sched_setscheduler_nocheck(p, SCHED_FIFO, &param);

	err_detector->esd_thread = p;
	err_detector->dev = dev;

	/*attach it to connector*/
	dsi_connector->err_detector = err_detector;

	/*time to start detection*/
	wake_up_process(p);

	DRM_INFO("%s: started\n", __func__);

	return 0;
esd_thread_err:
	kfree(err_detector);
	return -EAGAIN;
}

void mdfld_dsi_error_detector_exit(struct mdfld_dsi_connector *dsi_connector)
{
	struct mdfld_dsi_error_detector *err_detector;

	if (!dsi_connector || !dsi_connector->err_detector)
		return;

	err_detector = dsi_connector->err_detector;

	/*stop & destroy detector thread*/
	if (err_detector->esd_thread) {
		kthread_stop(err_detector->esd_thread);
		err_detector->esd_thread = NULL;
	}

	/*delete it*/
	kfree(err_detector);

	dsi_connector->err_detector = NULL;
}
