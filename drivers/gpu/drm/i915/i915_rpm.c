/*
 * Copyright 2013 Intel Corporation
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
 * Author:
 * Naresh Kumar Kachhi <naresh.kumar.kachhi@intel.com>
 */
#include "i915_drv.h"
#include "i915_reg.h"
#include "intel_drv.h"
#include "drmP.h"
#include <linux/console.h>
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#define RPM_AUTOSUSPEND_DELAY 500
#define RPM_AUTOSUSPEND 0x1
#define RPM_SYNC		0x2
#define RPM_SYNC_STRICT	0x4
#define RPM_NO_ADD_REQ	0x8
#define RPM_NOIDLE		0x10
#define RPM_NORESUME	0x20

#ifdef CONFIG_PM_RUNTIME
/* To make sure ring get/put are in pair */
bool ring_active;

int i915_rpm_init(struct drm_device *drm_dev)
{
	int ret = 0;
	struct device *dev = drm_dev->dev;
	ret = pm_runtime_set_active(dev);
	ring_active = false;
	pm_runtime_enable(dev);
	pm_runtime_allow(dev);
	/* enable Auto Suspend */
	pm_runtime_set_autosuspend_delay(dev, RPM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	if (dev->power.runtime_error)
		DRM_ERROR("rpm init: error = %d\n", dev->power.runtime_error);

	return ret;
}

int i915_rpm_deinit(struct drm_device *drm_dev)
{
	struct device *dev = drm_dev->dev;
	pm_runtime_forbid(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_get_noresume(dev);
	if (dev->power.runtime_error)
		DRM_ERROR("rpm init: error = %d\n", dev->power.runtime_error);

	return 0;
}

int check_mutex(struct drm_device *drm_dev)
{
	int ret = 0;
#ifdef RPM_DEBUG
	if ((mutex_is_locked(&drm_dev->mode_config.mutex)) &&
			(drm_dev->mode_config.mutex.owner == current)) {
		DRM_ERROR("config mutex locked by current thread\n");
		dump_stack();
		ret = -1;
	}
	if ((mutex_is_locked(&drm_dev->struct_mutex)) &&
			(drm_dev->struct_mutex.owner == current)) {
		DRM_ERROR("struct mutex locked by current thread\n");
		dump_stack();
		ret = -2;
	}
#endif
	return ret;
}

int i915_rpm_get(struct drm_device *drm_dev, u32 flags)
{
	struct device *dev = drm_dev->dev;
	int ret = 0;

	if (flags & RPM_SYNC) {
		if (dev->power.runtime_status == RPM_SUSPENDED) {
			/* Before doing a sync get make sure mutex is not
			 * locked
			 */
			check_mutex(drm_dev);
			ret = pm_runtime_get_sync(dev);
		} else {
			/* Don't call sync get if we are not in suspended
			 * state
			 */
			pm_runtime_get_noresume(dev);
		}
	} else if (flags & RPM_SYNC_STRICT) {
		ret = pm_runtime_get_sync(dev);
	} else if (flags & RPM_NORESUME) {
		pm_runtime_get_noresume(dev);
		ret = 0;
	} else {
		ret = pm_runtime_get(dev);
	}
	return ret;
}

int i915_rpm_put(struct drm_device *drm_dev, u32 flags)
{
	struct device *dev = drm_dev->dev;
	int ret = 0;
	if (flags & RPM_SYNC) {
		ret = pm_runtime_put_sync(dev);
	} else if (flags & RPM_AUTOSUSPEND) {
		/* Mark last time it was busy and schedule a autosuspend */
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
	} else if (flags & RPM_NOIDLE) {
		pm_runtime_put_noidle(dev);
	} else {
		/* Normal put */
		ret = pm_runtime_put(dev);
	}
	return ret;
}

/**
 * following is done to make sure Gfx is in D0i0 while
 * GPU accesses are being done
 * 1. For IOCTLS make sure we are in D0i0 by calling "get_ioctl".
 * 2. if IOCTL scheudles GPU commands using rings do the following
 *  a. For all ring accesses make sure we add a request in the request
 *     list and schedule a work item to track the "seq no". This
 *     is done by using "i915_add_request" or
 *     "i915_add_request_no_flush" functions.
 *  b. If request list was empty, we do a "get_ring". This will increment
 *     ref count to make sure GPU is kept on
 *  c. Once the list becomes empty call put_ring
 *
 * Note: All the ring accesses are covered with struct_mutex. So we
 * don't need any special synchronization here.
 */
int i915_rpm_get_ring(struct intel_ring_buffer *ring)
{
	struct drm_device *drm_dev = ring->dev;
	struct drm_i915_private *dev_priv = drm_dev->dev_private;
	int i;
	bool idle = true;

	for_each_ring(ring, dev_priv, i)
		idle &= list_empty(&ring->request_list);

	if (idle) {
		if (!ring_active) {
			ring_active = true;
			i915_rpm_get(drm_dev, RPM_NORESUME);
		}
	}

	return 0;
}

int i915_rpm_put_ring(struct intel_ring_buffer *ring)
{
	struct drm_device *drm_dev = ring->dev;
	if (ring_active) {
		i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
		ring_active = false;
	}
	return 0;
}

/**
 * To cover the function pointers that are assigned to drm structures
 * and can be called from drm
 */
int i915_rpm_get_callback(struct drm_device *drm_dev)
{
	i915_rpm_get(drm_dev, RPM_SYNC);
	return 0;
}

int i915_rpm_put_callback(struct drm_device *drm_dev)
{
	i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
	return 0;
}

/**
 * early_suspend/DSR should call this function to notify PM Core about
 * display idleness
 */
int i915_rpm_get_disp(struct drm_device *drm_dev)
{
	i915_rpm_get(drm_dev, RPM_SYNC_STRICT);
	return 0;
}

int i915_rpm_put_disp(struct drm_device *drm_dev)
{
	i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
	return 0;
}

/** to cover the ioctls with get/put*/
int i915_rpm_get_ioctl(struct drm_device *drm_dev)
{
	/* Don't do anything if device is not ready */
	if (drm_device_is_unplugged(drm_dev))
		return 0;

	i915_rpm_get(drm_dev, RPM_SYNC_STRICT);
	return 0;
}

int i915_rpm_put_ioctl(struct drm_device *drm_dev)
{
	/* Don't do anything if device is not ready */
	if (drm_device_is_unplugged(drm_dev))
		return 0;

	i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
	return 0;
}

/**
 * VXD driver need to call this to make sure Gfx is in D0i0
 * while VXD is on
 */
#ifdef CONFIG_DRM_VXD_BYT
int i915_rpm_get_vxd(struct drm_device *drm_dev)
{
	i915_rpm_get(drm_dev, RPM_SYNC);
	return 0;
}
EXPORT_SYMBOL(i915_rpm_get_vxd);

/**
 * VXD driver need to call this to notify Gfx that it is
 * done with HW accesses
 */
int i915_rpm_put_vxd(struct drm_device *drm_dev)
{
	i915_rpm_put(drm_dev, RPM_AUTOSUSPEND);
	return 0;
}
EXPORT_SYMBOL(i915_rpm_put_vxd);
#endif


bool i915_rpm_access_check(struct drm_device *dev)
{
#ifdef RPM_DEBUG
	if (dev->dev->power.runtime_status == RPM_SUSPENDED) {
		DRM_ERROR("invalid access, will cause Hard Hang\n");
		dump_stack();
		return false;
	}
#endif
	return true;
}

bool i915_is_device_active(struct drm_device *dev)
{
	return (dev->dev->power.runtime_status == RPM_ACTIVE);
}

bool i915_is_device_resuming(struct drm_device *dev)
{
	return (dev->dev->power.runtime_status == RPM_RESUMING);
}

bool i915_is_device_suspended(struct drm_device *dev)
{
	return (dev->dev->power.runtime_status == RPM_SUSPENDED);
}

bool i915_is_device_suspending(struct drm_device *dev)
{
	return (dev->dev->power.runtime_status == RPM_SUSPENDING);
}

#else /*CONFIG_PM_RUNTIME*/
int i915_rpm_init(struct drm_device *dev) {return 0; }
int i915_rpm_deinit(struct drm_device *dev) {return 0; }
int i915_rpm_get(struct drm_device *dev, u32 flags) {return 0; }
int i915_rpm_put(struct drm_device *dev, u32 flags) {return 0; }
int i915_rpm_get_ring(struct intel_ring_buffer *ring) {return 0; }
int i915_rpm_put_ring(struct intel_ring_buffer *ring) {return 0; }
int i915_rpm_get_callback(struct drm_device *dev) {return 0; }
int i915_rpm_put_callback(struct drm_device *dev) {return 0; }
int i915_rpm_get_ioctl(struct drm_device *dev) {return 0; }
int i915_rpm_put_ioctl(struct drm_device *dev) {return 0; }
int i915_rpm_get_disp(struct drm_device *dev) {return 0; }
int i915_rpm_put_disp(struct drm_device *dev) {return 0; }
#ifdef CONFIG_DRM_VXD_BYT
int i915_rpm_get_vxd(struct drm_device *dev) {return 0; }
int i915_rpm_put_vxd(struct drm_device *dev) {return 0; }
#endif

bool i915_is_device_active(struct drm_device *dev)
{
	return true;
}

bool i915_is_device_resuming(struct drm_device *dev)
{
	return false;
}

bool i915_is_device_suspended(struct drm_device *dev)
{
	return false;
}

bool i915_is_device_suspending(struct drm_device *dev)
{
	return false;
}

bool i915_rpm_access_check(struct drm_device *dev)
{
	return true;
}
#endif /*CONFIG_PM_RUNTIME*/

