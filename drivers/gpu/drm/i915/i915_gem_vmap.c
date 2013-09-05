/*
 * Copyright © 2013 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    ?????? <?????>
 */
#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include <linux/swap.h>
static
struct i915_gem_vmap_object *to_vmap_object(struct drm_i915_gem_object *obj)
{
	return container_of(obj, struct i915_gem_vmap_object, gem);
}

#if defined(CONFIG_MMU_NOTIFIER)
struct i915_vmap_mn_object {
	struct mmu_notifier mn;
	struct i915_gem_vmap_object *vmap;
	struct mm_struct    *mm;
};
struct i915_gem_mn_unregister_work {
	struct i915_vmap_mn_object *vmap_mn_obj;
	struct work_struct  work;
};
static void i915_gem_userptr_mn_unregister_work_fn(struct work_struct *__work)
{
	struct i915_gem_mn_unregister_work *work =
		container_of(__work, struct i915_gem_mn_unregister_work, work);
	struct i915_vmap_mn_object *vmap_mn_object =
		work->vmap_mn_obj;
	struct mmu_notifier *mn = &(vmap_mn_object->mn);
	struct mm_struct *mm = vmap_mn_object->mm;

	/*BUG_ON(mn->ops->invalidate_range_start);*/

	BUG_ON(vmap_mn_object->vmap);

	/* The following check is not really required, as even
	   though the release callback method of notfifier could
	   have already been called through __mmu_notifier_release,
	   we can still safely invoke the notifier_unregister function.
	   But this notifier unregistration has a considerable overhead,
	   so it is better to avoid this whenever possible */
	if (mm != NULL)	{
		/* Assuming till we have a single registered notifier,
		   kernel will not free up the 'mm' structure and
		   hence we may not have this check here. */
		BUG_ON(atomic_read(&mm->mm_count) <= 0);

		/* Even though the release callback method of
		   notifier & this work function could get executed
		   concurrently, we don't need an explicit synchronization
		   The kernel ensures the serialization between
		   the mmu_notifier_unregister & __mmu_notifier_release
		   functions, and also our release callback method would get
		   called from one of them only, as kernel de-links the
		   notifier structure from both the functions, so whichever
		   is executed first will also de-link the notifier structure */

		/* Do the real work now */
		mmu_notifier_unregister(mn, mm);
		BUG_ON(vmap_mn_object->mm);
	}

	/* Finally release the wrapper object & thus the
	   also memory for the embedded notifier object*/
	kfree(vmap_mn_object);

	/* Release the work object */
	kfree(work);
}

static void i915_gem_userptr_mn_invalidate_range_start(struct mmu_notifier *mn,
						       struct mm_struct *mm,
						       unsigned long start,
						       unsigned long end)
{
	struct i915_gem_vmap_object *vmap;
	struct drm_device *dev;

	/* XXX race between obj unref and mmu notifier? */
	vmap = container_of(mn, struct i915_vmap_mn_object, mn)->vmap;
	/* Check whether if the vmap obj was already destroyed for mn */
	if (vmap == NULL)
		return;

	BUG_ON(vmap->mm != mm);

	if (vmap->user_ptr >= end || vmap->user_ptr + vmap->user_size <= start)
		return;
	if (vmap->gem.pages == NULL) /* opportunistic check */
		return;

	dev = vmap->gem.base.dev;
	mutex_lock(&dev->struct_mutex);
	if (vmap->gem.gtt_space) {
		struct drm_i915_private *dev_priv = dev->dev_private;
		bool was_interruptible;
		int ret;

		was_interruptible = dev_priv->mm.interruptible;
		dev_priv->mm.interruptible = false;

		ret = i915_gem_object_unbind(&vmap->gem);
		BUG_ON(ret && ret != -EIO);

		dev_priv->mm.interruptible = was_interruptible;
	}

	/* Commented as not really needed*/
	/*BUG_ON(i915_gem_object_put_pages(&vmap->gem));*/
	mutex_unlock(&dev->struct_mutex);
}

static void i915_gem_userptr_mn_release(struct mmu_notifier *mn,
					struct mm_struct *mm)
{
	struct i915_gem_vmap_object *vmap;
	struct i915_vmap_mn_object *vmap_mn_object;

	vmap_mn_object =
		container_of(mn, struct i915_vmap_mn_object, mn);

	BUG_ON(vmap_mn_object->mm != mm);

	/* Set the mm pointer to NULL, as this release method
	   getting called now, probably indicates that process is
	   making an exit. So our work function which may
	   be invoked later shall not unregister the notifier now */
	vmap_mn_object->mm = NULL;

	vmap = vmap_mn_object->vmap;
	/* Check if the vmap object has already been released as
	   as this release method could be getting called after
	   the object was already freed, as now we do not unregister
	   the notifier immediately when the object is freed */
	if (vmap == NULL)
		return;

	vmap->mm = NULL;

	/* XXX Schedule an eventual unbind? E.g. hook into require request?
	 * However, locking will be complicated.
	 */
}

static const struct mmu_notifier_ops i915_gem_vmap_notifier = {
	.invalidate_range_start = i915_gem_userptr_mn_invalidate_range_start,
	.release = i915_gem_userptr_mn_release, };

static void
i915_gem_userptr_release__mmu_notifier(struct i915_gem_vmap_object *vmap) {

	struct mmu_notifier *mn = vmap->mn;
	struct i915_vmap_mn_object *vmap_mn_object;

	/* Check whether if a notifier was registered for this vmap
	   object or not because the User could have created this object
	   with I915_USERPTR_UNSYNCHRONIZED flag, if it does not want an
	   extra synchronization from the driver side */
	if (mn == NULL)
		return;

	vmap_mn_object =
		container_of(mn, struct i915_vmap_mn_object, mn);

	/* check if the mn's release function has already been
	   called or not due to process crash or exit, so
	   may not unregister the MMU notifier now.
	   In case of process crash/exit, first the OS
	   will release the notfifiers & then close the
	   DRM device file, causing the freeing of vmap objects */
	if (vmap->mm) {
		struct i915_gem_mn_unregister_work *work;
		struct drm_device *dev = vmap->gem.base.dev;
		struct drm_i915_private *dev_priv = dev->dev_private;

		/* set the pointer to NULL as the vmap
		   object would get destroyed shortly */
		vmap_mn_object->vmap = NULL;

		/* Allocate a new work item to take care of the
		   notifier unregistration */
		work = kzalloc(sizeof *work, GFP_KERNEL);
		if (work == NULL)
			return;

		work->vmap_mn_obj = vmap_mn_object;
		INIT_WORK(&work->work, i915_gem_userptr_mn_unregister_work_fn);

		/* Queue the unregistration work in our private workqueue,
		   from which the work items will be dequeued sequentially
		   one by one */
		queue_work(dev_priv->vmap_mn_unregister_wq, &work->work);

	} else {
		BUG_ON(vmap->mn == NULL);
		/* Release the wrapper object & thus also the memory
		   for the notfier object, as by now the release method
		   of notifier would have already been called by kernel */
		kfree(vmap_mn_object);
	}
}

static int
i915_gem_userptr_init__mmu_notifier(struct i915_gem_vmap_object *vmap,
				    unsigned flags)
{
	struct i915_vmap_mn_object *vmap_mn_object;

	if (flags & I915_USERPTR_UNSYNCHRONIZED)
		return capable(CAP_SYS_ADMIN) ? 0 : -EPERM;

	/* Allocate a new wrapper object*/
	vmap_mn_object =
		kzalloc(sizeof(struct i915_vmap_mn_object), GFP_KERNEL);

	if (vmap_mn_object == NULL)
		return -ENOMEM;
	vmap_mn_object->vmap = vmap;
	vmap_mn_object->mm = vmap->mm;
	vmap->mn = &vmap_mn_object->mn;
	vmap->mn->ops = &i915_gem_vmap_notifier;
	return mmu_notifier_register(vmap->mn, vmap->mm);
}

#else

static void
i915_gem_userptr_release__mmu_notifier(struct i915_gem_vmap_object *vmap)
{
	return 0;
}

static int
i915_gem_userptr_init__mmu_notifier(struct i915_gem_vmap_object *vmap,
				    unsigned flags)
{
	if ((flags & I915_USERPTR_UNSYNCHRONIZED) == 0)
		return -ENODEV;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	return 0;
}
#endif

static int
i915_gem_vmap_get_pages(struct drm_i915_gem_object *obj,
			struct page **pages,
			gfp_t gfpmask,
			u32 *offset)
{
	struct i915_gem_vmap_object *vmap = to_vmap_object(obj);
	int num_pages = vmap->gem.base.size >> PAGE_SHIFT, n;
	int pinned, ret;
	int i;

#if defined(CONFIG_MMU_NOTIFIER)
	if (vmap->mm == NULL)
		return -EFAULT;
	if (vmap->mm != current->mm)
		return -EFAULT;
#endif

	if (!access_ok(vmap->read_only ? VERIFY_READ : VERIFY_WRITE,
		       (char __user *)vmap->user_ptr, vmap->user_size))
		return -EFAULT;

	n = num_pages;
	pinned = __get_user_pages_fast(vmap->user_ptr, num_pages,
				       !vmap->read_only, pages);

	if (pinned < num_pages) {
		struct mm_struct *mm = current->mm;

		mutex_unlock(&obj->base.dev->struct_mutex);
		down_read(&mm->mmap_sem);
		ret = get_user_pages(current, mm,
				     vmap->user_ptr + (pinned << PAGE_SHIFT),
				     num_pages - pinned,
				     !vmap->read_only, 0,
				     pages + pinned,
				     NULL);
		up_read(&mm->mmap_sem);
		mutex_lock(&obj->base.dev->struct_mutex);
		if (ret > 0)
			pinned += ret;

		if (pinned < num_pages) {
			for (i = 0; i < pinned; i++)
				page_cache_release(pages[i]);
			return -EFAULT;
		}
	}

	obj->dirty = 0;
	*offset = offset_in_page(vmap->user_ptr);
	return 0;
}

static int
i915_gem_vmap_put_pages(struct drm_i915_gem_object *obj)
{
	int num_pages = obj->base.size >> PAGE_SHIFT;
	int i;

	for (i = 0; i < num_pages; i++) {
		if (obj->dirty)
			set_page_dirty(obj->pages[i]);

		mark_page_accessed(obj->pages[i]);
		page_cache_release(obj->pages[i]);
	}

	obj->dirty = 0;
	return 0;
}

static void
i915_gem_vmap_object_release(struct drm_i915_gem_object *obj)
{
	struct i915_gem_vmap_object *vmap = to_vmap_object(obj);
	i915_gem_userptr_release__mmu_notifier(vmap);
}

static bool
i915_gem_vmap_object_fn(void)
{
	return 1;
}

static const struct drm_i915_gem_object_ops i915_gem_vmap_ops = {
.get_pages   = i915_gem_vmap_get_pages,
.put_pages   = i915_gem_vmap_put_pages,
.release     = i915_gem_vmap_object_release,
.is_vmap_obj = i915_gem_vmap_object_fn,
};

/**
 * Creates a new mm object that wraps some user memory.
 */
int
i915_gem_vmap_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_vmap *args = data;
	struct i915_gem_vmap_object *obj;
	loff_t first_data_page, last_data_page;
	int num_pages;
	int ret;
	u32 handle;

	first_data_page = args->user_ptr / PAGE_SIZE;
	last_data_page = (args->user_ptr + args->user_size - 1) / PAGE_SIZE;
	num_pages = last_data_page - first_data_page + 1;
	if (num_pages * PAGE_SIZE > dev_priv->mm.gtt_total)
		return -E2BIG;

	ret = fault_in_pages_readable((char __user *)(uintptr_t)args->user_ptr,
				      args->user_size);
	if (ret)
		return ret;

	/* Allocate the new object */
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (obj == NULL)
		return -ENOMEM;

	obj->gem.base.dev = dev;
	obj->gem.base.size = num_pages * PAGE_SIZE;

	kref_init(&obj->gem.base.refcount);
	atomic_set(&obj->gem.base.handle_count, 0);

	i915_gem_object_init(dev, &obj->gem, &i915_gem_vmap_ops);
	/* To be checked, what is the optimal Cache policy for vmap objects */
	if (IS_VALLEYVIEW(dev))
		obj->gem.cache_level = I915_CACHE_LLC;
	else
		obj->gem.cache_level = I915_CACHE_LLC_MLC;

	obj->user_ptr = args->user_ptr;
	obj->user_size = args->user_size;
	obj->read_only = args->flags & I915_VMAP_READ_ONLY;

	/* And keep a pointer to the current->mm for resolving the user pages
	 * at binding. This means that we need to hook into the mmu_notifier
	 * in order to detect if the mmu is destroyed.
	 */
	obj->mm = current->mm;
	ret = i915_gem_userptr_init__mmu_notifier(obj, args->flags);
	if (ret) {
		drm_gem_object_release(&obj->gem.base);
		dev_priv->mm.object_count--;
		dev_priv->mm.object_memory -= obj->gem.base.size;
		kfree(obj);
		return ret;
	}

	ret = drm_gem_handle_create(file, &obj->gem.base, &handle);
	if (ret) {
		i915_gem_vmap_object_release((struct drm_i915_gem_object *)obj);
		drm_gem_object_release(&obj->gem.base);
		dev_priv->mm.object_count--;
		dev_priv->mm.object_memory -= obj->gem.base.size;
		kfree(obj);
		return ret;
	}

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference(&obj->gem.base);

	args->handle = handle;
	return 0;
}
