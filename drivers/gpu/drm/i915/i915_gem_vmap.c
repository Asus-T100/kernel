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
static void i915_gem_userptr_mn_invalidate_range_start(struct mmu_notifier *mn,
						       struct mm_struct *mm,
						       unsigned long start,
						       unsigned long end)
{
	struct i915_gem_vmap_object *vmap;
	struct drm_device *dev;

	/* XXX race between obj unref and mmu notifier? */
	vmap = container_of(mn, struct i915_gem_vmap_object, mn);
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
	vmap = container_of(mn, struct i915_gem_vmap_object, mn);

	BUG_ON(vmap->mm != mm);
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

	if (vmap->mn.ops && vmap->mm) {
		mmu_notifier_unregister(&vmap->mn, vmap->mm);
		BUG_ON(vmap->mm);
	}
}

static int
i915_gem_userptr_init__mmu_notifier(struct i915_gem_vmap_object *vmap,
				    unsigned flags)
{
	if (flags & I915_USERPTR_UNSYNCHRONIZED)
		return capable(CAP_SYS_ADMIN) ? 0 : -EPERM;

	vmap->mn.ops = &i915_gem_vmap_notifier;
	return mmu_notifier_register(&vmap->mn, vmap->mm);
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
