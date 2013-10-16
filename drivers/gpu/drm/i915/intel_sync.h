/*
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
#ifndef _INTEL_SYNC_H_
#define _INTEL_SYNC_H_

#include <linux/sync.h>

struct i915_sync_timeline {
	struct	sync_timeline	obj;

	struct {
		struct drm_device	*dev;

		u32			value;
		struct intel_ring_buffer *ring;
	} pvt;
};

struct i915_sync_pt {
	struct sync_pt		pt;

	struct {
		u32		value;
	} pvt;
};

struct i915_sync_timeline *i915_sync_timeline_create(struct drm_device *dev,
						const char *name,
						struct intel_ring_buffer *ring);
void i915_sync_timeline_destroy(struct i915_sync_timeline *obj);
struct sync_pt *i915_sync_pt_create(struct i915_sync_timeline *obj, u32 value);
int i915_sync_fence_create(struct i915_sync_timeline *obj,
				const char *name,
				u32 value);
void i915_sync_timeline_signal(struct i915_sync_timeline *obj, u32 value,
				int own_pt);

#endif /* _INTEL_SYNC_H_ */
