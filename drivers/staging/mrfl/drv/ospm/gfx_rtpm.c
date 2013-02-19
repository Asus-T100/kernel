/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 */

#include <linux/mutex.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>

#include <linux/pm_runtime.h>
#include "psb_drv.h"


int rtpm_suspend(struct device *dev)
{
	OSPM_DPF("%s\n", __func__);
	return -EBUSY;
}

int rtpm_resume(struct device *dev)
{
	OSPM_DPF("%s\n", __func__);
	return -EBUSY;
}

int rtpm_idle(struct device *dev)
{
	OSPM_DPF("%s\n", __func__);
	return -EBUSY;
}

int rtpm_allow(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	dev_priv->rpm_enabled = 1;
	return -EBUSY;
}

void rtpm_forbid(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	pm_runtime_forbid(&dev->pdev->dev);
	dev_priv->rpm_enabled = 0;
	return;
}

void rtpm_init(struct drm_device *dev)
{
}

void rtpm_enable(struct drm_device *dev)
{
	/*enable runtime pm at last */
	pm_runtime_enable(&dev->pdev->dev);
	pm_runtime_set_active(&dev->pdev->dev);
}

void rtpm_uninit(struct drm_device *dev)
{
	pm_runtime_disable(&dev->pdev->dev);
	pm_runtime_set_suspended(&dev->pdev->dev);
}
