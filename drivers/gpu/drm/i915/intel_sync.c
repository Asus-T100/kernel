/**************************************************************************
 * Copyright © 2013-2013 Intel Corporation
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
 *      Satyanantha RamaGopal M <rama.gopal.m.satyanantha@intel.com>
 */

#include <linux/device.h>
#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_sync.h"

static int i915_sync_pt_has_signaled(struct sync_pt *sync_pt)
{
	drm_i915_private_t *dev_priv = NULL;
	struct i915_sync_pt *pt = (struct i915_sync_pt *)sync_pt;
	struct i915_sync_timeline *obj =
		(struct i915_sync_timeline *)sync_pt->parent;

	dev_priv = (drm_i915_private_t *)obj->pvt.dev->dev_private;

	/* Upon TDR, fail the status of pending sync_pts.
	* This callback is synchronous with the thread which calls
	* sync_timeline_signal. If this has been signaled from TDR due
	* to an error then the TDR will have set ring->tdr_seqno
	* to the failing seqno (otherwise it will be 0). Compare the
	* sync point seqno with the failing seqno to detect errors */
	if (!obj->pvt.ring)
		return -ENODEV;
	else if (pt->pvt.value == obj->pvt.ring->tdr_seqno)
		return -ETIMEDOUT;
	else if (pt->pvt.value == 0)
		/* It hasn't yet been assigned a sequence number which means
		* it can't have finished */
		return 0;
	else
		/* This shouldn't require locking as it is synchronous
		* with the timeline signal function which is the only updater
		* of these fields*/
		return (obj->pvt.value >= pt->pvt.value) ? 1 : 0;

	return 0;
}

static int i915_sync_pt_compare(struct sync_pt *a, struct sync_pt *b)
{
	struct i915_sync_pt *pt_a = (struct i915_sync_pt *)a;
	struct i915_sync_pt *pt_b = (struct i915_sync_pt *)b;

	if (pt_a->pvt.value == pt_b->pvt.value)
		return 0;
	else
		return ((pt_a->pvt.value > pt_b->pvt.value) ? 1 : -1);
}

static int i915_sync_fill_driver_data(struct sync_pt *sync_pt,
				    void *data, int size)
{
	struct i915_sync_pt *pt = (struct i915_sync_pt *)sync_pt;

	if (size < sizeof(pt->pvt))
		return -ENOMEM;

	memcpy(data, &pt->pvt, sizeof(pt->pvt));

	return sizeof(pt->pvt);
}

static struct sync_pt *i915_sync_pt_dup(struct sync_pt *sync_pt)
{
	struct i915_sync_pt *pt = (struct i915_sync_pt *) sync_pt;
	struct i915_sync_timeline *obj =
		(struct i915_sync_timeline *)sync_pt->parent;

	return (struct sync_pt *) i915_sync_pt_create(obj, pt->pvt.value);
}

struct sync_timeline_ops i915_sync_timeline_ops = {
	.driver_name = "i915_sync",
	.dup = i915_sync_pt_dup,
	.has_signaled = i915_sync_pt_has_signaled,
	.compare = i915_sync_pt_compare,
	.fill_driver_data = i915_sync_fill_driver_data,
	.free_pt = NULL,
};

struct i915_sync_timeline *i915_sync_timeline_create(struct drm_device *dev,
							const char *name,
						struct intel_ring_buffer *ring)
{
	struct i915_sync_timeline *obj = (struct i915_sync_timeline *)
		sync_timeline_create(&i915_sync_timeline_ops,
				     sizeof(struct i915_sync_timeline),
				     name);

	obj->pvt.dev = dev;
	obj->pvt.ring = ring;

	/* Start the timeline from seqno 0 as this is a special value
	* that is never assigned to a batch buffer. */
	obj->pvt.value = 0;

	return obj;
}

void i915_sync_timeline_destroy(struct i915_sync_timeline *obj)
{
	if (obj)
		sync_timeline_destroy(&obj->obj);
}

struct sync_pt *i915_sync_pt_create(struct i915_sync_timeline *obj, u32 value)
{
	struct i915_sync_pt *pt;

	if (!obj)
		return NULL;

	pt = (struct i915_sync_pt *)
		sync_pt_create(&obj->obj, sizeof(struct i915_sync_pt));

	pt->pvt.value = value;

	return (struct sync_pt *)pt;
}

int i915_sync_fence_create(struct i915_sync_timeline *obj,
				const char *name,
				u32 value)
{
	int fd = get_unused_fd();
	int err;
	struct sync_pt *pt;
	struct sync_fence *fence;

	if (fd < 0) {
		DRM_ERROR("not able to get FD\n");
		return fd;
	}

	if (!obj) {
		DRM_ERROR("NULL timeline\n");
		err = -EFAULT;
		goto error;
	}

	pt = i915_sync_pt_create(obj, value);
	if (pt == NULL) {
		DRM_ERROR("sync pt creation failed\n");
		err = -ENOMEM;
		goto error;
	}

	fence = sync_fence_create(name, pt);
	if (fence == NULL) {
		DRM_ERROR("Fence creation failed\n");
		sync_pt_free(pt);
		err = -ENOMEM;
		goto error;
	}

	sync_fence_install(fence, fd);

	return fd;

error:
	put_unused_fd(fd);
	return err;
}


void i915_sync_timeline_signal(struct i915_sync_timeline *obj, u32 value,
				int own_pt)
{
	/* Update the timeline to notify it that the monotonic seqno counter
	* has advanced */

	if (obj) {
		obj->pvt.value = value;

		/* Only process the timeline if we own the sync point */
		if (own_pt)
			sync_timeline_signal(&obj->obj);
	}
}

