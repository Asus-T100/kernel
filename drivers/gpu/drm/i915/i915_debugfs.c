/*
 * Copyright © 2008 Intel Corporation
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
 *    Eric Anholt <eric@anholt.net>
 *    Keith Packard <keithp@keithp.com>
 */

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/string.h>
#include "drmP.h"
#include "drm.h"
#include "intel_drv.h"
#include "intel_ringbuffer.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_debugfs.h"

#define DRM_I915_RING_DEBUG 1

#if defined(CONFIG_DEBUG_FS)

static const char *yesno(int v)
{
	return v ? "Yes" : "No";
}

static int i915_capabilities(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	const struct intel_device_info *info = INTEL_INFO(dev);

	seq_printf(m, "gen: %d\n", info->gen);
	seq_printf(m, "pch: %d\n", INTEL_PCH_TYPE(dev));
#define DEV_INFO_FLAG(x) seq_printf(m, #x ": %s\n", yesno(info->x))
#define DEV_INFO_SEP ;
	DEV_INFO_FLAGS;
#undef DEV_INFO_FLAG
#undef DEV_INFO_SEP

	return 0;
}

static const char *get_pin_flag(struct drm_i915_gem_object *obj)
{
	if (obj->user_pin_count > 0)
		return "P";
	else if (obj->pin_count > 0)
		return "p";
	else
		return " ";
}

static const char *get_tiling_flag(struct drm_i915_gem_object *obj)
{
	switch (obj->tiling_mode) {
	default:
	case I915_TILING_NONE: return " ";
	case I915_TILING_X: return "X";
	case I915_TILING_Y: return "Y";
	}
}

static const char *cache_level_str(int type)
{
	switch (type) {
	case I915_CACHE_NONE: return " uncached";
	case I915_CACHE_LLC: return " snooped (LLC)";
	case I915_CACHE_LLC_MLC: return " snooped (LLC+MLC)";
	default: return "";
	}
}

ssize_t i915_gamma_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Gamma adjust: Not implemented\n");
	return -EINVAL;
}

ssize_t i915_gamma_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Gamma adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Gamma adjust: insufficient memory\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Gamma adjust: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Parse data and load the gamma  table */
	ret = parse_clrmgr_input(gammaSoftlut, buf,
		GAMMA_CORRECT_MAX_COUNT, count);
	if (ret < 0)
		DRM_ERROR("Gamma table loading failed\n");
	else
		DRM_DEBUG("Gamma table loading done\n");
EXIT:
	kfree(buf);
	/* If error, return error*/
	if (ret < 0)
		return ret;

	return count;
}

ssize_t i915_gamma_enable_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	int len = 0;
	char buf[10] = {0,};
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	len = sprintf(buf, "%s\n",
		dev_priv->gamma_enabled ? "Enabled" : "Disabled");
	return simple_read_from_buffer(ubuf, max, ppos,
	(const void *) buf, 10);
}

ssize_t i915_gamma_enable_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	unsigned int status = 0;
	struct drm_crtc *crtc = NULL;
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Gamma adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Gamma enable: Out of mem\n");
		return  -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Gamma adjust: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Finally, get the status */
	if (kstrtoul((const char *)buf, 10,
		&status)) {
		DRM_ERROR("Gamma enable: Invalid limit\n");
		ret = -EINVAL;
		goto EXIT;
	}
	dev_priv->gamma_enabled = status;

	/* Search for a CRTC,
	Assumption: Either MIPI or EDP is fix panel */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (intel_pipe_has_type(crtc, dev_priv->is_mipi ?
			INTEL_OUTPUT_DSI : INTEL_OUTPUT_EDP))
			break;
	}

	/* No CRTC */
	if (!crtc) {
		DRM_ERROR("Gamma adjust: No local panel found\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* if gamma enabled, apply gamma correction on PIPE */
	if (dev_priv->gamma_enabled) {
		if (intel_crtc_enable_gamma(crtc, PIPEA)) {
			DRM_ERROR("Apply gamma correction failed\n");
			ret = -EINVAL;
		} else
			ret = count;
	} else {
		/* Disable gamma on this plane */
		intel_crtc_disable_gamma(crtc, PIPEA);
		ret = count;
	}

EXIT:
	kfree(buf);
	return ret;
}

const struct file_operations i915_gamma_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_gamma_adjust_read,
	.write = i915_gamma_adjust_write,
	.llseek = default_llseek,
};

const struct file_operations i915_gamma_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_gamma_enable_read,
	.write = i915_gamma_enable_write,
	.llseek = default_llseek,
};

ssize_t i915_cb_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Contrast Brightness adjust: Read Not implemented\n");
	return -EINVAL;
}

ssize_t i915_cb_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = count;
	u32 val = 0;
	struct drm_device *dev = filp->private_data;
	struct ContBrightlut *cb_ptr = NULL;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Contrast Brightness: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Contrast Brightness adjust: insufficient memory\n");
		return -ENOMEM;
	}

	cb_ptr = kzalloc(sizeof(struct ContBrightlut), GFP_KERNEL);
	if (!cb_ptr) {
		DRM_ERROR("Contrast Brightness adjust: insufficient memory\n");
		kfree(buf);
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Contrast Brightness: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Parse input data */
	ret = parse_clrmgr_input(cb_ptr, buf, 2, count);
	if (ret < 0)
		DRM_ERROR("Contrast Brightness loading failed\n");
	else
		DRM_DEBUG("Contrast Brightness loading done\n");

	if (cb_ptr->sprite_no < SPRITEA || cb_ptr->sprite_no > SPRITED ||
			cb_ptr->sprite_no == PLANEB) {
		DRM_ERROR("Sprite value out of range. Enter 2,3, 5 or 6\n");
		goto EXIT;
	}

	DRM_DEBUG("sprite = %d Val=0x%x,\n", cb_ptr->sprite_no, cb_ptr->val);

	if (intel_sprite_cb_adjust(dev_priv, cb_ptr))
		DRM_ERROR("Contrast Brightness update failed\n");

EXIT:
	kfree(cb_ptr);
	kfree(buf);
	/* If cant read the full buffer, read from last left */
	if (ret < count-1)
		return ret;

	return count;
}

ssize_t i915_hs_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("Hue Saturation adjust: Read Not implemented\n");
	return -EINVAL;
}
ssize_t i915_hs_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = count;
	struct drm_device *dev = filp->private_data;
	struct HueSaturationlut *hs_ptr = NULL;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char *buf = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("Hue Saturation: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("Hue Saturation adjust: insufficient memory\n");
		return -ENOMEM;
	}

	hs_ptr = kzalloc(sizeof(struct HueSaturationlut), GFP_KERNEL);
	if (!hs_ptr) {
		DRM_ERROR("Hue Saturation adjust: insufficient memory\n");
		kfree(buf);
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("Hue Saturation: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Parse input data */
	ret = parse_clrmgr_input(hs_ptr, buf, 2, count);
	if (ret < 0)
		DRM_ERROR("Hue Saturation loading failed\n");
	else
		DRM_DEBUG("Hue Saturation loading done\n");

	if (hs_ptr->sprite_no < SPRITEA || hs_ptr->sprite_no > SPRITED ||
			hs_ptr->sprite_no == PLANEB) {
		DRM_ERROR("sprite = %d Val=0x%x,\n", hs_ptr->sprite_no,
					hs_ptr->val);
		goto EXIT;
	}

	DRM_DEBUG("sprite = %d Val=0x%x,\n", hs_ptr->sprite_no, hs_ptr->val);

	if (intel_sprite_hs_adjust(dev_priv, hs_ptr))
		DRM_ERROR("Hue Saturation update failed\n");

EXIT:
	kfree(hs_ptr);
	kfree(buf);
	/* If cant read the full buffer, read from last left */
	if (ret < count-1)
		return ret;

	return count;
}

ssize_t i915_csc_adjust_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	/* To do: Not implemented yet */
	DRM_ERROR("CSC adjust: Not implemented\n");
	return -EINVAL;
}

ssize_t i915_csc_adjust_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	char *buf  = NULL;

	/* Validate input */
	if (!count) {
		DRM_ERROR("CSC adjust: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("CSC adjust: insufficient memory\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("CSC adjust: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Parse data and load the csc  table */
	ret = parse_clrmgr_input(CSCSoftlut, buf,
		CSC_MAX_COEFF_COUNT, count);
	if (ret < 0)
		DRM_ERROR("CSC table loading failed\n");
	else
		DRM_DEBUG("CSC table loading done\n");
EXIT:
	kfree(buf);
	/* If cant read the full buffer, read from last left */
	if (ret < 0)
		return ret;

	return count;
}


ssize_t i915_csc_enable_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	int len = 0;
	char buf[10] = {0,};
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	len = sprintf(buf, "%s\n",
		dev_priv->csc_enabled ? "Enabled" : "Disabled");
	return simple_read_from_buffer(ubuf, max, ppos,
	(const void *) buf, 10);
}

ssize_t i915_csc_enable_write(struct file *filp,
		  const char __user *ubuf,
		  size_t count,
		  loff_t *ppos)
{
	int ret = 0;
	unsigned int status = 0;
	char *buf = NULL;
	struct drm_crtc *crtc = NULL;
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;

	/* Validate input */
	if (!count) {
		DRM_ERROR("CSC enable: insufficient data\n");
		return -EINVAL;
	}

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf) {
		DRM_ERROR("CSC enable: Out of mem\n");
		return -ENOMEM;
	}

	/* Get the data */
	if (copy_from_user(buf, ubuf, count)) {
		DRM_ERROR("CSC enable: copy failed\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* Finally, get the status */
	if (kstrtoul((const char *)buf, 10,
		&status)) {
		DRM_ERROR("CSC enable: Invalid limit\n");
		ret = -EINVAL;
		goto EXIT;
	}

	dev_priv->csc_enabled = status;

	/* Search for a CRTC,
	Assumption: Either MIPI or EDP is fix panel */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (intel_pipe_has_type(crtc, dev_priv->is_mipi ?
			INTEL_OUTPUT_DSI : INTEL_OUTPUT_EDP))
			break;
	}

	/* No CRTC */
	if (!crtc) {
		DRM_ERROR("CSC enable: No local panel found\n");
		ret = -EINVAL;
		goto EXIT;
	}

	/* if CSC enabled, apply CSC correction */
	if (dev_priv->csc_enabled) {
		if (do_intel_enable_CSC(dev,
			(void *) CSCSoftlut, crtc)) {
			DRM_ERROR("CSC correction failed\n");
			ret = -EINVAL;
		} else
			ret = count;
	} else {
		/* Disable CSC on this CRTC */
		do_intel_disable_CSC(dev, crtc);
		ret = count;
	}

EXIT:
	kfree(buf);
	return ret;
}

static const struct file_operations i915_cb_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_cb_adjust_read,
	.write = i915_cb_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_hs_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_hs_adjust_read,
	.write = i915_hs_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_csc_adjust_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_csc_adjust_read,
	.write = i915_csc_adjust_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_csc_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_csc_enable_read,
	.write = i915_csc_enable_write,
	.llseek = default_llseek,
};


static void
describe_obj(struct seq_file *m, struct drm_i915_gem_object *obj)
{
	seq_printf(m, "%p: %s%s %8zdKiB %04x %04x %d %d %d%s%s%s",
		   &obj->base,
		   get_pin_flag(obj),
		   get_tiling_flag(obj),
		   obj->base.size / 1024,
		   obj->base.read_domains,
		   obj->base.write_domain,
		   obj->last_read_seqno,
		   obj->last_write_seqno,
		   obj->last_fenced_seqno,
		   cache_level_str(obj->cache_level),
		   obj->dirty ? " dirty" : "",
		   obj->madv == I915_MADV_DONTNEED ? " purgeable" : "");
	if (obj->base.name)
		seq_printf(m, " (name: %d)", obj->base.name);
	if (obj->fence_reg != I915_FENCE_REG_NONE)
		seq_printf(m, " (fence: %d)", obj->fence_reg);
	if (obj->gtt_space != NULL)
		seq_printf(m, " (gtt offset: %08x, size: %08x)",
			   obj->gtt_offset, (unsigned int)obj->gtt_space->size);
	if (obj->pin_mappable || obj->fault_mappable) {
		char s[3], *t = s;
		if (obj->pin_mappable)
			*t++ = 'p';
		if (obj->fault_mappable)
			*t++ = 'f';
		*t = '\0';
		seq_printf(m, " (%s mappable)", s);
	}
	if (obj->ring != NULL)
		seq_printf(m, " (%s)", obj->ring->name);
}

static int i915_gem_object_list_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct list_head *head;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	switch (list) {
	case ACTIVE_LIST:
		seq_printf(m, "Active:\n");
		head = &dev_priv->mm.active_list;
		break;
	case INACTIVE_LIST:
		seq_printf(m, "Inactive:\n");
		head = &dev_priv->mm.inactive_list;
		break;
	default:
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, head, mm_list) {
		seq_printf(m, "   ");
		describe_obj(m, obj);
		seq_printf(m, "\n");
		total_obj_size += obj->base.size;
		total_gtt_size += obj->gtt_space->size;
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);
	return 0;
}

#define count_objects(list, member) do { \
	list_for_each_entry(obj, list, member) { \
		size += obj->gtt_space->size; \
		++count; \
		if (obj->map_and_fenceable) { \
			mappable_size += obj->gtt_space->size; \
			++mappable_count; \
		} \
	} \
} while (0)

static int i915_gem_object_info(struct seq_file *m, void* data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 count, mappable_count;
	size_t size, mappable_size;
	struct drm_i915_gem_object *obj;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "%u objects, %zu bytes\n",
		   dev_priv->mm.object_count,
		   dev_priv->mm.object_memory);

	size = count = mappable_size = mappable_count = 0;
	count_objects(&dev_priv->mm.gtt_list, gtt_list);
	seq_printf(m, "%u [%u] objects, %zu [%zu] bytes in gtt\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_objects(&dev_priv->mm.active_list, mm_list);
	seq_printf(m, "  %u [%u] active objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	count_objects(&dev_priv->mm.inactive_list, mm_list);
	seq_printf(m, "  %u [%u] inactive objects, %zu [%zu] bytes\n",
		   count, mappable_count, size, mappable_size);

	size = count = mappable_size = mappable_count = 0;
	list_for_each_entry(obj, &dev_priv->mm.gtt_list, gtt_list) {
		if (obj->fault_mappable) {
			size += obj->gtt_space->size;
			++count;
		}
		if (obj->pin_mappable) {
			mappable_size += obj->gtt_space->size;
			++mappable_count;
		}
	}
	seq_printf(m, "%u pinned mappable objects, %zu bytes\n",
		   mappable_count, mappable_size);
	seq_printf(m, "%u fault mappable objects, %zu bytes\n",
		   count, size);

	seq_printf(m, "%zu [%zu] gtt total\n",
		   dev_priv->mm.gtt_total, dev_priv->mm.mappable_gtt_total);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_gtt_info(struct seq_file *m, void* data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	uintptr_t list = (uintptr_t) node->info_ent->data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	size_t total_obj_size, total_gtt_size;
	int count, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	total_obj_size = total_gtt_size = count = 0;
	list_for_each_entry(obj, &dev_priv->mm.gtt_list, gtt_list) {
		if (list == PINNED_LIST && obj->pin_count == 0)
			continue;

		seq_printf(m, "   ");
		describe_obj(m, obj);
		seq_printf(m, "\n");
		total_obj_size += obj->base.size;
		total_gtt_size += obj->gtt_space->size;
		count++;
	}

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "Total %d objects, %zu bytes, %zu GTT size\n",
		   count, total_obj_size, total_gtt_size);

	return 0;
}

static int i915_gem_pageflip_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	unsigned long flags;
	struct intel_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, base.head) {
		const char pipe = pipe_name(crtc->pipe);
		const char plane = plane_name(crtc->plane);
		struct intel_unpin_work *work;

		spin_lock_irqsave(&dev->event_lock, flags);
		work = crtc->unpin_work;
		if (work == NULL) {
			seq_printf(m, "No flip due on pipe %c (plane %c)\n",
				   pipe, plane);
		} else {
			if (!work->pending) {
				seq_printf(m, "Flip queued on pipe %c (plane %c)\n",
					   pipe, plane);
			} else {
				seq_printf(m, "Flip pending (waiting for vsync) on pipe %c (plane %c)\n",
					   pipe, plane);
			}
			if (work->enable_stall_check)
				seq_printf(m, "Stall check enabled, ");
			else
				seq_printf(m, "Stall check waiting for page flip ioctl, ");
			seq_printf(m, "%d prepares\n", work->pending);

			if (work->old_fb_obj) {
				struct drm_i915_gem_object *obj = work->old_fb_obj;
				if (obj)
					seq_printf(m, "Old framebuffer gtt_offset 0x%08x\n", obj->gtt_offset);
			}
			if (work->pending_flip_obj) {
				struct drm_i915_gem_object *obj = work->pending_flip_obj;
				if (obj)
					seq_printf(m, "New framebuffer gtt_offset 0x%08x\n", obj->gtt_offset);
			}
		}
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	return 0;
}

static int i915_gem_request_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_gem_request *gem_request;
	int ret, count;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	count = 0;
	if (!list_empty(&dev_priv->ring[RCS].request_list)) {
		seq_printf(m, "Render requests:\n");
		list_for_each_entry(gem_request,
				    &dev_priv->ring[RCS].request_list,
				    list) {
			seq_printf(m, "    %d @ %d\n",
				   gem_request->seqno,
				   (int) (jiffies - gem_request->emitted_jiffies));
		}
		count++;
	}
	if (!list_empty(&dev_priv->ring[VCS].request_list)) {
		seq_printf(m, "BSD requests:\n");
		list_for_each_entry(gem_request,
				    &dev_priv->ring[VCS].request_list,
				    list) {
			seq_printf(m, "    %d @ %d\n",
				   gem_request->seqno,
				   (int) (jiffies - gem_request->emitted_jiffies));
		}
		count++;
	}
	if (!list_empty(&dev_priv->ring[BCS].request_list)) {
		seq_printf(m, "BLT requests:\n");
		list_for_each_entry(gem_request,
				    &dev_priv->ring[BCS].request_list,
				    list) {
			seq_printf(m, "    %d @ %d\n",
				   gem_request->seqno,
				   (int) (jiffies - gem_request->emitted_jiffies));
		}
		count++;
	}
	mutex_unlock(&dev->struct_mutex);

	if (count == 0)
		seq_printf(m, "No requests\n");

	return 0;
}

static void i915_ring_seqno_info(struct seq_file *m,
				 struct intel_ring_buffer *ring)
{
	if (ring->get_seqno) {
		seq_printf(m, "Current sequence (%s): %d\n",
			   ring->name, ring->get_seqno(ring, false));
	}
}

static int i915_gem_seqno_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 0; i < I915_NUM_RINGS; i++)
		i915_ring_seqno_info(m, &dev_priv->ring[i]);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}


static int i915_interrupt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret, i, pipe;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "Display IER:\t%08x\n",
			   I915_READ(VLV_IER));
		seq_printf(m, "Display IIR:\t%08x\n",
			   I915_READ(VLV_IIR));
		seq_printf(m, "Display IIR_RW:\t%08x\n",
			   I915_READ(VLV_IIR_RW));
		seq_printf(m, "Display IMR:\t%08x\n",
			   I915_READ(VLV_IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:\t%08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));

		seq_printf(m, "Master IER:\t%08x\n",
			   I915_READ(VLV_MASTER_IER));

		seq_printf(m, "Render IER:\t%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Render IIR:\t%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Render IMR:\t%08x\n",
			   I915_READ(GTIMR));

		seq_printf(m, "PM IER:\t\t%08x\n",
			   I915_READ(GEN6_PMIER));
		seq_printf(m, "PM IIR:\t\t%08x\n",
			   I915_READ(GEN6_PMIIR));
		seq_printf(m, "PM IMR:\t\t%08x\n",
			   I915_READ(GEN6_PMIMR));

		seq_printf(m, "Port hotplug:\t%08x\n",
			   I915_READ(PORT_HOTPLUG_EN));
		seq_printf(m, "DPFLIPSTAT:\t%08x\n",
			   I915_READ(VLV_DPFLIPSTAT));
		seq_printf(m, "DPINVGTT:\t%08x\n",
			   I915_READ(DPINVGTT));

	} else if (!HAS_PCH_SPLIT(dev)) {
		seq_printf(m, "Interrupt enable:    %08x\n",
			   I915_READ(IER));
		seq_printf(m, "Interrupt identity:  %08x\n",
			   I915_READ(IIR));
		seq_printf(m, "Interrupt mask:      %08x\n",
			   I915_READ(IMR));
		for_each_pipe(pipe)
			seq_printf(m, "Pipe %c stat:         %08x\n",
				   pipe_name(pipe),
				   I915_READ(PIPESTAT(pipe)));
	} else {
		seq_printf(m, "North Display Interrupt enable:		%08x\n",
			   I915_READ(DEIER));
		seq_printf(m, "North Display Interrupt identity:	%08x\n",
			   I915_READ(DEIIR));
		seq_printf(m, "North Display Interrupt mask:		%08x\n",
			   I915_READ(DEIMR));
		seq_printf(m, "South Display Interrupt enable:		%08x\n",
			   I915_READ(SDEIER));
		seq_printf(m, "South Display Interrupt identity:	%08x\n",
			   I915_READ(SDEIIR));
		seq_printf(m, "South Display Interrupt mask:		%08x\n",
			   I915_READ(SDEIMR));
		seq_printf(m, "Graphics Interrupt enable:		%08x\n",
			   I915_READ(GTIER));
		seq_printf(m, "Graphics Interrupt identity:		%08x\n",
			   I915_READ(GTIIR));
		seq_printf(m, "Graphics Interrupt mask:		%08x\n",
			   I915_READ(GTIMR));
	}
	seq_printf(m, "Interrupts received: %d\n",
		   atomic_read(&dev_priv->irq_received));
	for (i = 0; i < I915_NUM_RINGS; i++) {
		if (IS_GEN6(dev) || IS_GEN7(dev)) {
			seq_printf(m, "Graphics Interrupt mask (%s):	%08x\n",
				   dev_priv->ring[i].name,
				   I915_READ_IMR(&dev_priv->ring[i]));
		}
		i915_ring_seqno_info(m, &dev_priv->ring[i]);
	}
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_gem_fence_regs_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int i, ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "Reserved fences = %d\n", dev_priv->fence_reg_start);
	seq_printf(m, "Total fences = %d\n", dev_priv->num_fence_regs);
	for (i = 0; i < dev_priv->num_fence_regs; i++) {
		struct drm_i915_gem_object *obj = dev_priv->fence_regs[i].obj;

		seq_printf(m, "Fenced object[%2d] = ", i);
		if (obj == NULL)
			seq_printf(m, "unused");
		else
			describe_obj(m, obj);
		seq_printf(m, "\n");
	}

	mutex_unlock(&dev->struct_mutex);
	return 0;
}

static int i915_hws_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	const volatile u32 __iomem *hws;
	int i;

	ring = &dev_priv->ring[(uintptr_t)node->info_ent->data];
	hws = (volatile u32 __iomem *)ring->status_page.page_addr;
	if (hws == NULL)
		return 0;

	for (i = 0; i < 4096 / sizeof(u32) / 4; i += 4) {
		seq_printf(m, "0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			   i * 4,
			   hws[i], hws[i + 1], hws[i + 2], hws[i + 3]);
	}
	return 0;
}

static const char *ring_str(int ring)
{
	switch (ring) {
	case RCS: return "render";
	case VCS: return "bsd";
	case BCS: return "blt";
	default: return "";
	}
}

static const char *pin_flag(int pinned)
{
	if (pinned > 0)
		return " P";
	else if (pinned < 0)
		return " p";
	else
		return "";
}

static const char *tiling_flag(int tiling)
{
	switch (tiling) {
	default:
	case I915_TILING_NONE: return "";
	case I915_TILING_X: return " X";
	case I915_TILING_Y: return " Y";
	}
}

static const char *dirty_flag(int dirty)
{
	return dirty ? " dirty" : "";
}

static const char *purgeable_flag(int purgeable)
{
	return purgeable ? " purgeable" : "";
}

static void print_error_buffers(struct seq_file *m,
				const char *name,
				struct drm_i915_error_buffer *err,
				int count)
{
	seq_printf(m, "%s [%d]:\n", name, count);

	while (count--) {
		seq_printf(m, "  %08x %8u %04x %04x %x %x%s%s%s%s%s%s%s",
			   err->gtt_offset,
			   err->size,
			   err->read_domains,
			   err->write_domain,
			   err->rseqno, err->wseqno,
			   pin_flag(err->pinned),
			   tiling_flag(err->tiling),
			   dirty_flag(err->dirty),
			   purgeable_flag(err->purgeable),
			   err->ring != -1 ? " " : "",
			   ring_str(err->ring),
			   cache_level_str(err->cache_level));

		if (err->name)
			seq_printf(m, " (name: %d)", err->name);
		if (err->fence_reg != I915_FENCE_REG_NONE)
			seq_printf(m, " (fence: %d)", err->fence_reg);

		seq_printf(m, "\n");
		err++;
	}
}

static void i915_ring_error_state(struct seq_file *m,
				  struct drm_device *dev,
				  struct drm_i915_error_state *error,
				  unsigned ring)
{
	BUG_ON(ring >= I915_NUM_RINGS); /* shut up confused gcc */
	seq_printf(m, "%s command stream:\n", ring_str(ring));
	seq_printf(m, "  HEAD: 0x%08x\n", error->head[ring]);
	seq_printf(m, "  TAIL: 0x%08x\n", error->tail[ring]);
	seq_printf(m, "  ACTHD: 0x%08x\n", error->acthd[ring]);
	seq_printf(m, "  IPEIR: 0x%08x\n", error->ipeir[ring]);
	seq_printf(m, "  IPEHR: 0x%08x\n", error->ipehr[ring]);
	seq_printf(m, "  INSTDONE: 0x%08x\n", error->instdone[ring][0]);
	if (ring == RCS) {
		switch (INTEL_INFO(dev)->gen) {
		case 4:
		case 5:
		case 6:
			seq_printf(m, "  INSTDONE1: 0x%08x\n",
				error->instdone[ring][1]);
			break;

		case 7:
			seq_printf(m, "  INSTDONE1: 0x%08x\n",
				error->instdone[ring][1]);
			seq_printf(m, "  INSTDONE2: 0x%08x\n",
				error->instdone[ring][2]);
			seq_printf(m, "  INSTDONE3: 0x%08x\n",
				error->instdone[ring][3]);
			break;
		}

		if (INTEL_INFO(dev)->gen >= 4) {
			seq_printf(m, "  BBADDR: 0x%08llx\n",
				error->bbaddr);
		}
	}
	if (INTEL_INFO(dev)->gen >= 4)
		seq_printf(m, "  INSTPS: 0x%08x\n", error->instps[ring]);
	seq_printf(m, "  INSTPM: 0x%08x\n", error->instpm[ring]);
	seq_printf(m, "  FADDR: 0x%08x\n", error->faddr[ring]);
	if (INTEL_INFO(dev)->gen >= 6) {
		seq_printf(m, "  RC PSMI: 0x%08x\n", error->rc_psmi[ring]);
		seq_printf(m, "  FAULT_REG: 0x%08x\n", error->fault_reg[ring]);
		seq_printf(m, "  SYNC_0: 0x%08x\n",
			   error->semaphore_mboxes[ring][0]);
		seq_printf(m, "  SYNC_1: 0x%08x\n",
			   error->semaphore_mboxes[ring][1]);
	}
	seq_printf(m, "  seqno: 0x%08x\n", error->seqno[ring]);
	seq_printf(m, "  waiting: %s\n", yesno(error->waiting[ring]));
	seq_printf(m, "  ring->head: 0x%08x\n", error->cpu_ring_head[ring]);
	seq_printf(m, "  ring->tail: 0x%08x\n", error->cpu_ring_tail[ring]);
}

struct i915_error_state_file_priv {
	struct drm_device *dev;
	struct drm_i915_error_state *error;
};

static int i915_error_state(struct seq_file *m, void *unused)
{
	struct i915_error_state_file_priv *error_priv = m->private;
	struct drm_device *dev = error_priv->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_error_state *error = error_priv->error;
	struct intel_ring_buffer *ring;
	int i, j, page, offset, elt;

	if (!error) {
		seq_printf(m, "no error state collected\n");
		return 0;
	}

	seq_printf(m, "Time: %ld s %ld us\n", error->time.tv_sec,
		   error->time.tv_usec);
	seq_printf(m, "PCI ID: 0x%04x\n", dev->pci_device);
	seq_printf(m, "EIR: 0x%08x\n", error->eir);
	seq_printf(m, "IER: 0x%08x\n", error->ier);
	seq_printf(m, "PGTBL_ER: 0x%08x\n", error->pgtbl_er);
	seq_printf(m, "CCID: 0x%08x\n", error->ccid);

	for (i = 0; i < dev_priv->num_fence_regs; i++)
		seq_printf(m, "  fence[%d] = %08llx\n", i, error->fence[i]);

	if (INTEL_INFO(dev)->gen >= 6) {
		seq_printf(m, "ERROR: 0x%08x\n", error->error);
		seq_printf(m, "DONE_REG: 0x%08x\n", error->done_reg);
	}

	for_each_ring(ring, dev_priv, i)
		i915_ring_error_state(m, dev, error, i);

	if (error->active_bo)
		print_error_buffers(m, "Active",
				    error->active_bo,
				    error->active_bo_count);

	if (error->pinned_bo)
		print_error_buffers(m, "Pinned",
				    error->pinned_bo,
				    error->pinned_bo_count);

	for (i = 0; i < ARRAY_SIZE(error->ring); i++) {
		struct drm_i915_error_object *obj;

		if ((obj = error->ring[i].batchbuffer)) {
			seq_printf(m, "%s --- gtt_offset = 0x%08x\n",
				   dev_priv->ring[i].name,
				   obj->gtt_offset);
			offset = 0;
			for (page = 0; page < obj->page_count; page++) {
				for (elt = 0; elt < PAGE_SIZE/4; elt++) {
					seq_printf(m, "%08x :  %08x\n", offset, obj->pages[page][elt]);
					offset += 4;
				}
			}
		}

		if (error->ring[i].num_requests) {
			seq_printf(m, "%s --- %d requests\n",
				   dev_priv->ring[i].name,
				   error->ring[i].num_requests);
			for (j = 0; j < error->ring[i].num_requests; j++) {
				seq_printf(m, "  seqno 0x%08x, emitted %ld, tail 0x%08x\n",
					   error->ring[i].requests[j].seqno,
					   error->ring[i].requests[j].jiffies,
					   error->ring[i].requests[j].tail);
			}
		}

		if ((obj = error->ring[i].ringbuffer)) {
			seq_printf(m, "%s --- ringbuffer = 0x%08x\n",
				   dev_priv->ring[i].name,
				   obj->gtt_offset);
			offset = 0;
			for (page = 0; page < obj->page_count; page++) {
				for (elt = 0; elt < PAGE_SIZE/4; elt++) {
					seq_printf(m, "%08x :  %08x\n",
						   offset,
						   obj->pages[page][elt]);
					offset += 4;
				}
			}
		}
	}

	if (error->overlay)
		intel_overlay_print_error_state(m, error->overlay);

	if (error->display)
		intel_display_print_error_state(m, dev, error->display);

	return 0;
}

static ssize_t
i915_error_state_write(struct file *filp,
		       const char __user *ubuf,
		       size_t cnt,
		       loff_t *ppos)
{
	struct seq_file *m = filp->private_data;
	struct i915_error_state_file_priv *error_priv = m->private;
	struct drm_device *dev = error_priv->dev;
	int ret;

	DRM_DEBUG_DRIVER("Resetting error state\n");

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	i915_destroy_error_state(dev);
	mutex_unlock(&dev->struct_mutex);

	return cnt;
}

static int i915_error_state_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct i915_error_state_file_priv *error_priv;
	unsigned long flags;

	error_priv = kzalloc(sizeof(*error_priv), GFP_KERNEL);
	if (!error_priv)
		return -ENOMEM;

	error_priv->dev = dev;

	spin_lock_irqsave(&dev_priv->error_lock, flags);
	error_priv->error = dev_priv->first_error;
	if (error_priv->error)
		kref_get(&error_priv->error->ref);
	spin_unlock_irqrestore(&dev_priv->error_lock, flags);

	return single_open(file, i915_error_state, error_priv);
}

static int i915_error_state_release(struct inode *inode, struct file *file)
{
	struct seq_file *m = file->private_data;
	struct i915_error_state_file_priv *error_priv = m->private;

	if (error_priv->error)
		kref_put(&error_priv->error->ref, i915_error_state_free);
	kfree(error_priv);

	return single_release(inode, file);
}

static const struct file_operations i915_error_state_fops = {
	.owner = THIS_MODULE,
	.open = i915_error_state_open,
	.read = seq_read,
	.write = i915_error_state_write,
	.llseek = default_llseek,
	.release = i915_error_state_release,
};

static int i915_rstdby_delays(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u16 crstanddelay;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	crstanddelay = I915_READ16(CRSTANDVID);

	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "w/ctx: %d, w/o ctx: %d\n", (crstanddelay >> 8) & 0x3f, (crstanddelay & 0x3f));

	return 0;
}

static int i915_cur_delayinfo(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;
	u32 pval = 0;

	if (IS_GEN5(dev)) {
		u16 rgvswctl = I915_READ16(MEMSWCTL);
		u16 rgvstat = I915_READ16(MEMSTAT_ILK);

		seq_printf(m, "Requested P-state: %d\n", (rgvswctl >> 8) & 0xf);
		seq_printf(m, "Requested VID: %d\n", rgvswctl & 0x3f);
		seq_printf(m, "Current VID: %d\n", (rgvstat & MEMSTAT_VID_MASK) >>
			   MEMSTAT_VID_SHIFT);
		seq_printf(m, "Current P-state: %d\n",
			   (rgvstat & MEMSTAT_PSTATE_MASK) >> MEMSTAT_PSTATE_SHIFT);
	} else if (IS_VALLEYVIEW(dev)) {
		seq_printf(m, "Max Gpu Freq _max_delay_: %d\n",
				dev_priv->rps.max_delay);
		seq_printf(m, "Min Gpu Freq _min_delay_: %d\n",
				dev_priv->rps.min_delay);
		intel_punit_read32(dev_priv, PUNIT_REG_GPU_FREQ_STS,
					&pval);
		seq_printf(m, "Cur Gpu Freq _cur_delay_: %d\n",
				pval >> 8);
		seq_printf(m, "Last Requested Gpu Freq _requested_delay_: %d\n",
				dev_priv->rps.requested_delay);
		seq_printf(m, "Up Threshold: %ld\n",
			atomic_read(&dev_priv->turbodebug.up_threshold));
		seq_printf(m, "Down Threshold: %d\n",
			atomic_read(&dev_priv->turbodebug.down_threshold));
		seq_printf(m, "RP_UP: %d\nRP_DOWN:%d\n",
				dev_priv->rps.rp_up_masked,
					dev_priv->rps.rp_down_masked);
	} else if (IS_GEN6(dev) || IS_GEN7(dev)) {
		u32 gt_perf_status = I915_READ(GEN6_GT_PERF_STATUS);
		u32 rp_state_limits = I915_READ(GEN6_RP_STATE_LIMITS);
		u32 rp_state_cap = I915_READ(GEN6_RP_STATE_CAP);
		u32 rpstat;
		u32 rpupei, rpcurup, rpprevup;
		u32 rpdownei, rpcurdown, rpprevdown;
		int max_freq;

		/* RPSTAT1 is in the GT power well */
		ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
		if (ret)
			return ret;

		/* TBD: Wake up relevant engine for VLV rather all */
		gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

		rpstat = I915_READ(GEN6_RPSTAT1);
		rpupei = I915_READ(GEN6_RP_CUR_UP_EI);
		rpcurup = I915_READ(GEN6_RP_CUR_UP);
		rpprevup = I915_READ(GEN6_RP_PREV_UP);
		rpdownei = I915_READ(GEN6_RP_CUR_DOWN_EI);
		rpcurdown = I915_READ(GEN6_RP_CUR_DOWN);
		rpprevdown = I915_READ(GEN6_RP_PREV_DOWN);

		/* TBD: Wake up relevant engine for VLV rather all */
		gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);
		mutex_unlock(&dev_priv->rps.rps_mutex);

		seq_printf(m, "GT_PERF_STATUS: 0x%08x\n", gt_perf_status);
		seq_printf(m, "RPSTAT1: 0x%08x\n", rpstat);
		seq_printf(m, "Render p-state ratio: %d\n",
			   (gt_perf_status & 0xff00) >> 8);
		seq_printf(m, "Render p-state VID: %d\n",
			   gt_perf_status & 0xff);
		seq_printf(m, "Render p-state limit: %d\n",
			   rp_state_limits & 0xff);
		seq_printf(m, "CAGF: %dMHz\n", ((rpstat & GEN6_CAGF_MASK) >>
						GEN6_CAGF_SHIFT) * 50);
		seq_printf(m, "RP CUR UP EI: %dus\n", rpupei &
			   GEN6_CURICONT_MASK);
		seq_printf(m, "RP CUR UP: %dus\n", rpcurup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV UP: %dus\n", rpprevup &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP CUR DOWN EI: %dus\n", rpdownei &
			   GEN6_CURIAVG_MASK);
		seq_printf(m, "RP CUR DOWN: %dus\n", rpcurdown &
			   GEN6_CURBSYTAVG_MASK);
		seq_printf(m, "RP PREV DOWN: %dus\n", rpprevdown &
			   GEN6_CURBSYTAVG_MASK);

		max_freq = (rp_state_cap & 0xff0000) >> 16;
		seq_printf(m, "Lowest (RPN) frequency: %dMHz\n",
			   max_freq * 50);

		max_freq = (rp_state_cap & 0xff00) >> 8;
		seq_printf(m, "Nominal (RP1) frequency: %dMHz\n",
			   max_freq * 50);

		max_freq = rp_state_cap & 0xff;
		seq_printf(m, "Max non-overclocked (RP0) frequency: %dMHz\n",
			   max_freq * 50);
	} else {
		seq_printf(m, "no P-state info available\n");
	}

	return 0;
}

static int i915_delayfreq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 delayfreq;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 0; i < 16; i++) {
		delayfreq = I915_READ(PXVFREQ_BASE + i * 4);
		seq_printf(m, "P%02dVIDFREQ: 0x%08x (VID: %d)\n", i, delayfreq,
			   (delayfreq & PXVFREQ_PX_MASK) >> PXVFREQ_PX_SHIFT);
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static inline int MAP_TO_MV(int map)
{
	return 1250 - (map * 25);
}

static int i915_inttoext_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 inttoext;
	int ret, i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 1; i <= 32; i++) {
		inttoext = I915_READ(INTTOEXT_BASE_ILK + i * 4);
		seq_printf(m, "INTTOEXT%02d: 0x%08x\n", i, inttoext);
	}

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int ironlake_drpc_info(struct seq_file *m)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	u32 rgvmodectl, rstdbyctl;
	u16 crstandvid;
	int ret;

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	rgvmodectl = I915_READ(MEMMODECTL);
	rstdbyctl = I915_READ(RSTDBYCTL);
	crstandvid = I915_READ16(CRSTANDVID);

	mutex_unlock(&dev_priv->rps.rps_mutex);

	seq_printf(m, "HD boost: %s\n", (rgvmodectl & MEMMODE_BOOST_EN) ?
		   "yes" : "no");
	seq_printf(m, "Boost freq: %d\n",
		   (rgvmodectl & MEMMODE_BOOST_FREQ_MASK) >>
		   MEMMODE_BOOST_FREQ_SHIFT);
	seq_printf(m, "HW control enabled: %s\n",
		   rgvmodectl & MEMMODE_HWIDLE_EN ? "yes" : "no");
	seq_printf(m, "SW control enabled: %s\n",
		   rgvmodectl & MEMMODE_SWMODE_EN ? "yes" : "no");
	seq_printf(m, "Gated voltage change: %s\n",
		   rgvmodectl & MEMMODE_RCLK_GATE ? "yes" : "no");
	seq_printf(m, "Starting frequency: P%d\n",
		   (rgvmodectl & MEMMODE_FSTART_MASK) >> MEMMODE_FSTART_SHIFT);
	seq_printf(m, "Max P-state: P%d\n",
		   (rgvmodectl & MEMMODE_FMAX_MASK) >> MEMMODE_FMAX_SHIFT);
	seq_printf(m, "Min P-state: P%d\n", (rgvmodectl & MEMMODE_FMIN_MASK));
	seq_printf(m, "RS1 VID: %d\n", (crstandvid & 0x3f));
	seq_printf(m, "RS2 VID: %d\n", ((crstandvid >> 8) & 0x3f));
	seq_printf(m, "Render standby enabled: %s\n",
		   (rstdbyctl & RCX_SW_EXIT) ? "no" : "yes");
	seq_printf(m, "Current RS state: ");
	switch (rstdbyctl & RSX_STATUS_MASK) {
	case RSX_STATUS_ON:
		seq_printf(m, "on\n");
		break;
	case RSX_STATUS_RC1:
		seq_printf(m, "RC1\n");
		break;
	case RSX_STATUS_RC1E:
		seq_printf(m, "RC1E\n");
		break;
	case RSX_STATUS_RS1:
		seq_printf(m, "RS1\n");
		break;
	case RSX_STATUS_RS2:
		seq_printf(m, "RS2 (RC6)\n");
		break;
	case RSX_STATUS_RS3:
		seq_printf(m, "RC3 (RC6+)\n");
		break;
	default:
		seq_printf(m, "unknown\n");
		break;
	}

	return 0;
}

static int gen6_drpc_info(struct seq_file *m)
{

	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 rpmodectl1, gt_core_status, rcctl1;
	unsigned forcewake_count;
	int count=0, ret;


	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	spin_lock_irq(&dev_priv->gt_lock);
	forcewake_count = dev_priv->forcewake_count;
	spin_unlock_irq(&dev_priv->gt_lock);

	if (forcewake_count) {
		seq_printf(m, "RC information inaccurate because somebody "
			      "holds a forcewake reference \n");
	} else {
		/* NB: we cannot use forcewake, else we read the wrong values */
		while (count++ < 50 && (I915_READ_NOTRACE(FORCEWAKE_ACK) & 1))
			udelay(10);
		seq_printf(m, "RC information accurate: %s\n", yesno(count < 51));
	}

	gt_core_status = readl(dev_priv->regs + GEN6_GT_CORE_STATUS);
	trace_i915_reg_rw(false, GEN6_GT_CORE_STATUS, gt_core_status, 4);

	rpmodectl1 = I915_READ(GEN6_RP_CONTROL);
	rcctl1 = I915_READ(GEN6_RC_CONTROL);
	mutex_unlock(&dev_priv->rps.rps_mutex);

	seq_printf(m, "Video Turbo Mode: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_MEDIA_TURBO));
	seq_printf(m, "HW control enabled: %s\n",
		   yesno(rpmodectl1 & GEN6_RP_ENABLE));
	seq_printf(m, "SW control enabled: %s\n",
		   yesno((rpmodectl1 & GEN6_RP_MEDIA_MODE_MASK) ==
			  GEN6_RP_MEDIA_SW_MODE));
	seq_printf(m, "RC1e Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC1e_ENABLE));
	seq_printf(m, "RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6_ENABLE));
	seq_printf(m, "Deep RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6p_ENABLE));
	seq_printf(m, "Deepest RC6 Enabled: %s\n",
		   yesno(rcctl1 & GEN6_RC_CTL_RC6pp_ENABLE));
	seq_printf(m, "Current RC state: ");
	switch (gt_core_status & GEN6_RCn_MASK) {
	case GEN6_RC0:
		if (gt_core_status & GEN6_CORE_CPD_STATE_MASK)
			seq_printf(m, "Core Power Down\n");
		else
			seq_printf(m, "on\n");
		break;
	case GEN6_RC3:
		seq_printf(m, "RC3\n");
		break;
	case GEN6_RC6:
		seq_printf(m, "RC6\n");
		break;
	case GEN6_RC7:
		seq_printf(m, "RC7\n");
		break;
	default:
		seq_printf(m, "Unknown\n");
		break;
	}

	seq_printf(m, "Core Power Down: %s\n",
		   yesno(gt_core_status & GEN6_CORE_CPD_STATE_MASK));

	/* Not exactly sure what this is */
	seq_printf(m, "RC6 \"Locked to RPn\" residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6_LOCKED));
	seq_printf(m, "RC6 residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6));
	seq_printf(m, "RC6+ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6p));
	seq_printf(m, "RC6++ residency since boot: %u\n",
		   I915_READ(GEN6_GT_GFX_RC6pp));

	return 0;
}

static int i915_drpc_info(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;

	if (IS_GEN6(dev) || IS_GEN7(dev))
		return gen6_drpc_info(m);
	else
		return ironlake_drpc_info(m);
}

static int i915_fbc_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!I915_HAS_FBC(dev)) {
		seq_printf(m, "FBC unsupported on this chipset\n");
		return 0;
	}

	if (intel_fbc_enabled(dev)) {
		seq_printf(m, "FBC enabled\n");
	} else {
		seq_printf(m, "FBC disabled: ");
		switch (dev_priv->no_fbc_reason) {
		case FBC_NO_OUTPUT:
			seq_printf(m, "no outputs");
			break;
		case FBC_STOLEN_TOO_SMALL:
			seq_printf(m, "not enough stolen memory");
			break;
		case FBC_UNSUPPORTED_MODE:
			seq_printf(m, "mode not supported");
			break;
		case FBC_MODE_TOO_LARGE:
			seq_printf(m, "mode too large");
			break;
		case FBC_BAD_PLANE:
			seq_printf(m, "FBC unsupported on plane");
			break;
		case FBC_NOT_TILED:
			seq_printf(m, "scanout buffer not tiled");
			break;
		case FBC_MULTIPLE_PIPES:
			seq_printf(m, "multiple pipes are enabled");
			break;
		case FBC_MODULE_PARAM:
			seq_printf(m, "disabled per module param (default off)");
			break;
		default:
			seq_printf(m, "unknown reason");
		}
		seq_printf(m, "\n");
	}
	return 0;
}

static int i915_sr_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	bool sr_enabled = false;

	if (HAS_PCH_SPLIT(dev))
		sr_enabled = I915_READ(WM1_LP_ILK) & WM1_LP_SR_EN;
	else if (IS_CRESTLINE(dev) || IS_I945G(dev) || IS_I945GM(dev))
		sr_enabled = I915_READ(FW_BLC_SELF) & FW_BLC_SELF_EN;
	else if (IS_I915GM(dev))
		sr_enabled = I915_READ(INSTPM) & INSTPM_SELF_EN;
	else if (IS_PINEVIEW(dev))
		sr_enabled = I915_READ(DSPFW3) & PINEVIEW_SELF_REFRESH_EN;

	seq_printf(m, "self-refresh: %s\n",
		   sr_enabled ? "enabled" : "disabled");

	return 0;
}

static int i915_emon_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	unsigned long temp, chipset, gfx;
	int ret;

	if (!IS_GEN5(dev))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	temp = i915_mch_val(dev_priv);
	chipset = i915_chipset_val(dev_priv);
	gfx = i915_gfx_val(dev_priv);
	mutex_unlock(&dev->struct_mutex);

	seq_printf(m, "GMCH temp: %ld\n", temp);
	seq_printf(m, "Chipset power: %ld\n", chipset);
	seq_printf(m, "GFX power: %ld\n", gfx);
	seq_printf(m, "Total power: %ld\n", chipset + gfx);

	return 0;
}

static int i915_ring_freq_table(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;
	int gpu_freq, ia_freq;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)) || IS_VALLEYVIEW(dev)) {
		seq_printf(m, "unsupported on this chipset\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	seq_printf(m, "GPU freq (MHz)\tEffective CPU freq (MHz)\n");

	for (gpu_freq = dev_priv->rps.min_delay;
	     gpu_freq <= dev_priv->rps.max_delay;
	     gpu_freq++) {
		I915_WRITE(GEN6_PCODE_DATA, gpu_freq);
		I915_WRITE(GEN6_PCODE_MAILBOX, GEN6_PCODE_READY |
			   GEN6_PCODE_READ_MIN_FREQ_TABLE);
		if (wait_for((I915_READ(GEN6_PCODE_MAILBOX) &
			      GEN6_PCODE_READY) == 0, 10)) {
			DRM_ERROR("pcode read of freq table timed out\n");
			continue;
		}
		ia_freq = I915_READ(GEN6_PCODE_DATA);
		seq_printf(m, "%d\t\t%d\n", gpu_freq * 50, ia_freq * 100);
	}

	mutex_unlock(&dev_priv->rps.rps_mutex);

	return 0;
}

static int i915_gfxec(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "GFXEC: %ld\n", (unsigned long)I915_READ(0x112f4));

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_opregion(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_opregion *opregion = &dev_priv->opregion;
	void *data = kmalloc(OPREGION_SIZE, GFP_KERNEL);
	int ret;

	if (data == NULL)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		goto out;

	if (opregion->header) {
		memcpy_fromio(data, opregion->header, OPREGION_SIZE);
		seq_write(m, data, OPREGION_SIZE);
	}

	mutex_unlock(&dev->struct_mutex);

out:
	kfree(data);
	return 0;
}

static int i915_gem_framebuffer_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_fbdev *ifbdev;
	struct intel_framebuffer *fb;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	ifbdev = dev_priv->fbdev;
	fb = to_intel_framebuffer(ifbdev->helper.fb);

	seq_printf(m, "fbcon size: %d x %d, depth %d, %d bpp, obj ",
		   fb->base.width,
		   fb->base.height,
		   fb->base.depth,
		   fb->base.bits_per_pixel);
	describe_obj(m, fb->obj);
	seq_printf(m, "\n");

	list_for_each_entry(fb, &dev->mode_config.fb_list, base.head) {
		if (&fb->base == ifbdev->helper.fb)
			continue;

		seq_printf(m, "user size: %d x %d, depth %d, %d bpp, obj ",
			   fb->base.width,
			   fb->base.height,
			   fb->base.depth,
			   fb->base.bits_per_pixel);
		describe_obj(m, fb->obj);
		seq_printf(m, "\n");
	}

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static int i915_context_status(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	if (dev_priv->pwrctx) {
		seq_printf(m, "power context ");
		describe_obj(m, dev_priv->pwrctx);
		seq_printf(m, "\n");
	}

	if (dev_priv->renderctx) {
		seq_printf(m, "render context ");
		describe_obj(m, dev_priv->renderctx);
		seq_printf(m, "\n");
	}

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static int i915_gen6_forcewake_count_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned forcewake_count;

	spin_lock_irq(&dev_priv->gt_lock);
	forcewake_count = dev_priv->forcewake_count;
	spin_unlock_irq(&dev_priv->gt_lock);

	seq_printf(m, "forcewake count = %u\n", forcewake_count);

	return 0;
}

static const char *swizzle_string(unsigned swizzle)
{
	switch(swizzle) {
	case I915_BIT_6_SWIZZLE_NONE:
		return "none";
	case I915_BIT_6_SWIZZLE_9:
		return "bit9";
	case I915_BIT_6_SWIZZLE_9_10:
		return "bit9/bit10";
	case I915_BIT_6_SWIZZLE_9_11:
		return "bit9/bit11";
	case I915_BIT_6_SWIZZLE_9_10_11:
		return "bit9/bit10/bit11";
	case I915_BIT_6_SWIZZLE_9_17:
		return "bit9/bit17";
	case I915_BIT_6_SWIZZLE_9_10_17:
		return "bit9/bit10/bit17";
	case I915_BIT_6_SWIZZLE_UNKNOWN:
		return "unkown";
	}

	return "bug";
}

static int i915_swizzle_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	seq_printf(m, "bit6 swizzle for X-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_x));
	seq_printf(m, "bit6 swizzle for Y-tiling = %s\n",
		   swizzle_string(dev_priv->mm.bit_6_swizzle_y));

	if (IS_GEN3(dev) || IS_GEN4(dev)) {
		seq_printf(m, "DDC = 0x%08x\n",
			   I915_READ(DCC));
		seq_printf(m, "C0DRB3 = 0x%04x\n",
			   I915_READ16(C0DRB3));
		seq_printf(m, "C1DRB3 = 0x%04x\n",
			   I915_READ16(C1DRB3));
	} else if (IS_GEN6(dev) || IS_GEN7(dev)) {
		seq_printf(m, "MAD_DIMM_C0 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C0));
		seq_printf(m, "MAD_DIMM_C1 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C1));
		seq_printf(m, "MAD_DIMM_C2 = 0x%08x\n",
			   I915_READ(MAD_DIMM_C2));
		seq_printf(m, "TILECTL = 0x%08x\n",
			   I915_READ(TILECTL));
		seq_printf(m, "ARB_MODE = 0x%08x\n",
			   I915_READ(ARB_MODE));
		seq_printf(m, "DISP_ARB_CTL = 0x%08x\n",
			   I915_READ(DISP_ARB_CTL));
	}
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_ppgtt_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_ring_buffer *ring;
	int i, ret;


	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;
	if (INTEL_INFO(dev)->gen == 6)
		seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(GFX_MODE));

	for (i = 0; i < I915_NUM_RINGS; i++) {
		ring = &dev_priv->ring[i];

		seq_printf(m, "%s\n", ring->name);
		if (INTEL_INFO(dev)->gen == 7)
			seq_printf(m, "GFX_MODE: 0x%08x\n", I915_READ(RING_MODE_GEN7(ring)));
		seq_printf(m, "PP_DIR_BASE: 0x%08x\n", I915_READ(RING_PP_DIR_BASE(ring)));
		seq_printf(m, "PP_DIR_BASE_READ: 0x%08x\n", I915_READ(RING_PP_DIR_BASE_READ(ring)));
		seq_printf(m, "PP_DIR_DCLV: 0x%08x\n", I915_READ(RING_PP_DIR_DCLV(ring)));
	}
	if (dev_priv->mm.aliasing_ppgtt) {
		struct i915_hw_ppgtt *ppgtt = dev_priv->mm.aliasing_ppgtt;

		seq_printf(m, "aliasing PPGTT:\n");
		seq_printf(m, "pd gtt offset: 0x%08x\n", ppgtt->pd_offset);
	}
	seq_printf(m, "ECOCHK: 0x%08x\n", I915_READ(GAM_ECOCHK));
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int i915_dpio_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;


	if (!IS_VALLEYVIEW(dev)) {
		seq_printf(m, "unsupported\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	seq_printf(m, "DPIO_CTL: 0x%08x\n", I915_READ(DPIO_CTL));

	seq_printf(m, "DPIO_DIV_A: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_DIV_A));
	seq_printf(m, "DPIO_DIV_B: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_DIV_B));

	seq_printf(m, "DPIO_REFSFR_A: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_REFSFR_A));
	seq_printf(m, "DPIO_REFSFR_B: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_REFSFR_B));

	seq_printf(m, "DPIO_CORE_CLK_A: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_CORE_CLK_A));
	seq_printf(m, "DPIO_CORE_CLK_B: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_CORE_CLK_B));

	seq_printf(m, "DPIO_LFP_COEFF_A: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_LFP_COEFF_A));
	seq_printf(m, "DPIO_LFP_COEFF_B: 0x%08x\n",
		   intel_dpio_read(dev_priv, _DPIO_LFP_COEFF_B));

	seq_printf(m, "DPIO_FASTCLK_DISABLE: 0x%08x\n",
		   intel_dpio_read(dev_priv, DPIO_FASTCLK_DISABLE));

	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

static ssize_t
i915_wedged_read(struct file *filp,
		 char __user *ubuf,
		 size_t max,
		 loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[80];
	int len;

	len = snprintf(buf, sizeof(buf),
		       "wedged :  %d\n",
		       atomic_read(&dev_priv->mm.wedged));

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_wedged_write(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[20];
	int val = 1;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		val = simple_strtoul(buf, NULL, 0);
	}

	if (val) {
		if (!atomic_read(&dev_priv->full_reset))
			i915_handle_error(dev, NULL, 0);
	}

	return cnt;
}

static const struct file_operations i915_wedged_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_wedged_read,
	.write = i915_wedged_write,
	.llseek = default_llseek,
};

static ssize_t
i915_ring_stop_read(struct file *filp,
		    char __user *ubuf,
		    size_t max,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[20];
	int len;

	len = snprintf(buf, sizeof(buf),
		       "0x%08x\n", dev_priv->stop_rings);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_ring_stop_write(struct file *filp,
		     const char __user *ubuf,
		     size_t cnt,
		     loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[20];
	int val = 0;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		val = simple_strtoul(buf, NULL, 0);
	}

	DRM_DEBUG_DRIVER("Stopping rings 0x%08x\n", val);

	if (val < I915_NUM_RINGS)
		intel_ring_disable(&dev_priv->ring[val]);

	return cnt;
}

static ssize_t
i915_ring_hangcheck_read(struct file *filp,
			char __user *ubuf,
			size_t max,
			loff_t *ppos)
{
	/* Returns the total number of times the rings
	 * have hung and been reset since boot */
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[100];
	int len;

	len = snprintf(buf, sizeof(buf),
		       "GPU=0x%08x,RCS=0x%08x,VCS=0x%08x,BCS=0x%08x\n",
			dev_priv->total_resets,
			dev_priv->hangcheck[RCS].total,
			dev_priv->hangcheck[VCS].total,
			dev_priv->hangcheck[BCS].total);


	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_ring_hangcheck_write(struct file *filp,
			const char __user *ubuf,
			size_t cnt,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;
	uint32_t i;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	for (i = 0; i < I915_NUM_RINGS; i++) {
		/* Reset the hangcheck counters */
		dev_priv->hangcheck[i].total = 0;
	}

	dev_priv->total_resets = 0;

	mutex_unlock(&dev->struct_mutex);

	return cnt;
}

static const struct file_operations i915_ring_hangcheck_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_ring_hangcheck_read,
	.write = i915_ring_hangcheck_write,
	.llseek = default_llseek,
};

static const struct file_operations i915_ring_stop_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_ring_stop_read,
	.write = i915_ring_stop_write,
	.llseek = default_llseek,
};


/* Helper function to set max freq turbo based on input */
static int
i915_set_max_freq(struct drm_device *dev, int val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("Manually setting max freq to %d\n", val);

	/*
	 * Turbo will still be enabled, but won't go above the set value.
	 */
	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev)) {
		dev_priv->rps.max_delay = val;
		valleyview_set_rps(dev, val);
	} else {
		dev_priv->rps.max_delay = val / 50;
		gen6_set_rps(dev, val / 50);
	}

	mutex_unlock(&dev_priv->rps.rps_mutex);
	return 0;
}

/* Helper function to get max freq turbo based on input */
static int
i915_get_max_freq(struct drm_device *dev, int *val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev))
		*val = dev_priv->rps.max_delay;
	else
		*val = dev_priv->rps.max_delay * 50;

	mutex_unlock(&dev_priv->rps.rps_mutex);
	return 0;
}

static ssize_t
i915_max_freq_read(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[80];
	int len, ret, val;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	ret = i915_get_max_freq(dev, &val);
	if (ret)
		return ret;

	len = snprintf(buf, sizeof(buf),
		       "max freq: %d\n", val);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_max_freq_write(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	int val = 1, ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		val = simple_strtoul(buf, NULL, 0);
	}

	ret = i915_set_max_freq(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_max_freq_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_max_freq_read,
	.write = i915_max_freq_write,
	.llseek = default_llseek,
};


/* Helper function to set min freq turbo based on input */
static int
i915_set_min_freq(struct drm_device *dev, int val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("Manually setting min freq to %d\n", val);

	/*
	 * Turbo will still be enabled, but won't go below the set value.
	 */
	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev)) {
		dev_priv->rps.min_delay = val;
		valleyview_set_rps(dev, val);
	} else {
		dev_priv->rps.min_delay = val / 50;
		gen6_set_rps(dev, val / 50);
	}

	mutex_unlock(&dev_priv->rps.rps_mutex);
	return 0;
}

/* Helper function to get min freq turbo based on input */
static int
i915_get_min_freq(struct drm_device *dev, int *val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	if (IS_VALLEYVIEW(dev))
		*val = dev_priv->rps.min_delay;
	else
		*val = dev_priv->rps.min_delay * 50;

	mutex_unlock(&dev_priv->rps.rps_mutex);
	return 0;
}

static ssize_t
i915_min_freq_read(struct file *filp, char __user *ubuf, size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[80];
	int len, ret, val;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	ret = i915_get_min_freq(dev, &val);
	if (ret)
		return ret;

	len = snprintf(buf, sizeof(buf),
		       "min freq: %d\n", val);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_min_freq_write(struct file *filp, const char __user *ubuf, size_t cnt,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	int val = 1, ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		val = simple_strtoul(buf, NULL, 0);
	}

	ret = i915_set_min_freq(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_min_freq_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_min_freq_read,
	.write = i915_min_freq_write,
	.llseek = default_llseek,
};

/* Helper function to enable and disable turbo based on input */
static int
i915_rps_enable_disable(struct drm_device *dev, long unsigned int val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	/* 1=> Enable Turbo, else disable. */

	if (val == 1)
		vlv_turbo_initialize(dev);
	else
		vlv_turbo_disable(dev);

	mutex_unlock(&dev_priv->rps.rps_mutex);

	return 0;
}

static ssize_t
i915_rps_init_read(struct file *filp, char __user *ubuf, size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[] = "rps init read is not defined";
	int len;
	u32 rval;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	rval = I915_READ(GEN6_RP_CONTROL);
	len = snprintf(buf, sizeof(buf),
		       "Turbo Enabled: %s\n", yesno(rval & GEN6_RP_ENABLE));
	if (len > sizeof(buf))
		len = sizeof(buf);
	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}


static ssize_t
i915_rps_init_write(struct file *filp, const char __user *ubuf, size_t cnt,
		    loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	long unsigned int val = 1;
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;

		buf[cnt] = 0;

		ret = kstrtoul(buf, 0, (unsigned long *)&val);
		if (ret)
			return -EINVAL;
	}

	ret = i915_rps_enable_disable(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_rps_init_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_rps_init_read,
	.write = i915_rps_init_write,
	.llseek = default_llseek,
};

/* Common Debugfs apis for all the PC features files */

/* Debugfs mmio apis implementation */

static ssize_t
i915_mmio_read_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], offset[20], operation[10], format[20];
	int len = 0, ret, noOfTokens;
	u32 mmio_to_read;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.mmio.mmio_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%ds %%%ds",
				sizeof(operation), sizeof(offset));

	noOfTokens = sscanf(i915_debugfs_vars.mmio.mmio_vars,
					format, operation, offset);
	if (noOfTokens < 2)
		return len;

	len = sizeof(i915_debugfs_vars.mmio.mmio_vars);

	if (strcmp(operation, READ_TOKEN) == 0) {
		ret = kstrtoul(offset, 16, &mmio_to_read);
		if (ret)
			return -EINVAL;

		len = snprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n",
				(unsigned int) mmio_to_read,
				(unsigned int) I915_READ(mmio_to_read));
	} else
		len = snprintf(buf, sizeof(buf),
				"MMIO WRITE not supported\n");

	if (len > sizeof(buf))
		len = sizeof(buf);

	i915_debugfs_vars.mmio.mmio_input = 0;

	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_mmio_write_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.mmio.mmio_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.mmio.mmio_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.mmio.mmio_vars, ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.mmio.mmio_vars[cnt] = 0;

		/* Enable Read */
		i915_debugfs_vars.mmio.mmio_input = 1;
	}

	return cnt;
}
static const struct file_operations i915_mmio_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_mmio_read_api,
	.write = i915_mmio_write_api,
	.llseek = default_llseek,
};

/* Debugfs iosf apis implementation */

static ssize_t
i915_iosf_read_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], operation[10], port[10], offset[20], format[20];
	int len = 0, ret, noOfTokens;
	u32 iosf_reg, iosf_val;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.iosf.iosf_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%ds %%%ds %%%ds",
			sizeof(operation), sizeof(port), sizeof(offset));

	noOfTokens = sscanf(i915_debugfs_vars.iosf.iosf_vars,
				format, operation, port, offset);

	if (noOfTokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.iosf.iosf_vars);

	ret = kstrtoul(offset, 16, &iosf_reg);
	if (ret)
		return -EINVAL;

	if (strcmp(operation, READ_TOKEN) == 0) {
		if (strcmp(port, IOSF_PUNIT_TOKEN) == 0) {
			intel_punit_read32(dev_priv, iosf_reg, &iosf_val);
			len = snprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n", (unsigned int) iosf_reg,
						(unsigned int) iosf_val);
		} else if (strcmp(port, IOSF_FUSE_TOKEN) == 0) {
			intel_fuse_read32(dev_priv, iosf_reg, &iosf_val);
			len = snprintf(buf, sizeof(buf),
				"0x%x: 0x%x\n", (unsigned int) iosf_reg,
						(unsigned int) iosf_val);
		}
	} else {
		len = snprintf(buf, sizeof(buf),
				"IOSF WRITE not supported\n");
	}

	if (len > sizeof(buf))
		len = sizeof(buf);

	i915_debugfs_vars.iosf.iosf_input = 0;

	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_iosf_write_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.iosf.iosf_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.iosf.iosf_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.iosf.iosf_vars, ubuf, cnt))
			return -EFAULT;
		i915_debugfs_vars.iosf.iosf_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.iosf.iosf_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_iosf_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_iosf_read_api,
	.write = i915_iosf_write_api,
	.llseek = default_llseek,
};

static ssize_t
i915_cache_sharing_read(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[80];
	u32 snpcr;
	int len, ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);
	mutex_unlock(&dev_priv->dev->struct_mutex);

	len = snprintf(buf, sizeof(buf),
		       "%d\n", (snpcr & GEN6_MBC_SNPCR_MASK) >>
		       GEN6_MBC_SNPCR_SHIFT);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_cache_sharing_write(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	struct drm_i915_private *dev_priv = dev->dev_private;
	char buf[20];
	u32 snpcr;
	int val = 1, ret;

	if (!(IS_GEN6(dev) || IS_GEN7(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		ret = kstrtoul(buf, 0, &val);
		if (ret)
			return -EINVAL;
	}

	if (val < 0 || val > 3)
		return -EINVAL;

	DRM_DEBUG_DRIVER("Manually setting uncore sharing to %d\n", val);

	/* Update the cache sharing policy here as well */
	snpcr = I915_READ(GEN6_MBCUNIT_SNPCR);
	snpcr &= ~GEN6_MBC_SNPCR_MASK;
	snpcr |= (val << GEN6_MBC_SNPCR_SHIFT);
	I915_WRITE(GEN6_MBCUNIT_SNPCR, snpcr);

	return cnt;
}

static const struct file_operations i915_cache_sharing_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_cache_sharing_read,
	.write = i915_cache_sharing_write,
	.llseek = default_llseek,
};

/* As the drm_debugfs_init() routines are called before dev->dev_private is
 * allocated we need to hook into the minor for release. */
static int
drm_add_fake_info_node(struct drm_minor *minor,
		       struct dentry *ent,
		       const void *key)
{
	struct drm_info_node *node;

	node = kmalloc(sizeof(struct drm_info_node), GFP_KERNEL);
	if (node == NULL) {
		debugfs_remove(ent);
		return -ENOMEM;
	}

	node->minor = minor;
	node->dent = ent;
	node->info_ent = (void *) key;

	mutex_lock(&minor->debugfs_lock);
	list_add(&node->list, &minor->debugfs_list);
	mutex_unlock(&minor->debugfs_lock);

	return 0;
}

static int i915_forcewake_open(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	/* This will wake all engines on VLV */
	gen6_gt_force_wake_get(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static int i915_forcewake_release(struct inode *inode, struct file *file)
{
	struct drm_device *dev = inode->i_private;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	/* This will wake all engines on VLV */
	gen6_gt_force_wake_put(dev_priv, FORCEWAKE_ALL);

	return 0;
}

static const struct file_operations i915_forcewake_fops = {
	.owner = THIS_MODULE,
	.open = i915_forcewake_open,
	.release = i915_forcewake_release,
};

/* Debugfs rc6 apis implementation */

static inline bool is_rc6_enabled(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;

	return I915_READ(VLV_RENDER_C_STATE_CONTROL_1_REG)
			& (VLV_EVAL_METHOD_ENABLE_BIT
			| VLV_TIMEOUT_METHOD_ENABLE_BIT);
}

static int
rc6_status(struct drm_device *dev, char *buf, int *len)
{
	*len = snprintf(buf, MAX_BUFFER_STR_LEN,
			"RC6 ENABLED: %s\n",
			yesno(is_rc6_enabled(dev)));
	return 0;
}

static int
rc6_enable_disable(struct drm_device *dev, long unsigned int val)
{
	int ret;
	drm_i915_private_t *dev_priv = dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (is_rc6_enabled(dev)) {
		if (val > 0)
			return 0;
	} else if (val == 0)
		return 0;

	ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
	if (ret)
		return ret;

	vlv_rs_setstate(dev, (val > 0 ? true : false));
	mutex_unlock(&dev_priv->rps.rps_mutex);
	DRM_DEBUG_DRIVER("RC6 feature status is %ld\n", val);

	return 0;
}


static int
i915_read_rc6_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], control[10], operation[20], format[20];
	int len = 0, ret, noOfTokens;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.rc6.rc6_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%ds %%%ds",
				sizeof(control), sizeof(operation));

	noOfTokens = sscanf(i915_debugfs_vars.rc6.rc6_vars,
					format, control, operation);

	if (noOfTokens < 2)
		return len;

	len = sizeof(i915_debugfs_vars.rc6.rc6_vars);

	if (strcmp(operation, STATUS_TOKEN) == 0) {
		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RC6, else disable.
		*/
		ret = rc6_enable_disable(dev, 1);
		if (ret)
			return ret;

		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {
		/*
		* 1=> Enable RC6, else disable.
		*/
		ret = rc6_enable_disable(dev, 0);
		if (ret)
			return ret;

		rc6_status(dev, buf, &len);

	} else if (strcmp(operation, RC6_POWER_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
				"RENDER WELL: %s & MEDIA WELL: %s\n",
				(I915_READ(VLV_POWER_WELL_STATUS_REG) &
					VLV_RENDER_POWER_WELL_STATUS_BIT)
					? "UP" : "DOWN",
				(I915_READ(VLV_POWER_WELL_STATUS_REG) &
					VLV_MEDIA_POWER_WELL_STATUS_BIT)
					? "UP" : "DOWN");

	} else if (strcmp(operation, READ_COUNTER_0_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
				"RENDER WELL C0 COUNTER: 0x%x & ",
				(unsigned int) I915_READ(GEN6_GT_GFX_RC6));
		if (len < 0)
			return len;

		len += snprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C1 COUNTER: 0x%x\n",
				(unsigned int) I915_READ(GEN6_GT_GFX_RC6p));

	} else if (strcmp(operation, READ_COUNTER_1_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
				"RENDER WELL C1 COUNTER: 0x%x & ",
				(unsigned int)I915_READ(GEN6_GT_GFX_RC6pp));

		if (len < 0)
			return len;

		len += snprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C1 COUNTER: 0x%x\n",
				(unsigned int)
					I915_READ(VLV_MEDIA_C1_COUNT_REG));

	} else if (strcmp(operation, READ_COUNTER_6_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
				"RENDER WELL C6 COUNTER: 0x%x & ",
				(unsigned int)
					I915_READ(VLV_RENDER_C0_COUNT_REG));

		if (len < 0)
			return len;

		len += snprintf(&buf[len], (sizeof(buf) - len),
				"MEDIA WELL C6 COUNTER: 0x%x\n",
				(unsigned int)
					I915_READ(VLV_MEDIA_C0_COUNT_REG));

	} else if (strcmp(operation, MULTITHREAD_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	} else if (strcmp(operation, RC6_SINGLETHREAD_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
			"SINGLE THREAD ENABLED: Yes\n");
	} else
		len = snprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	if (len > sizeof(buf))
		len = sizeof(buf);

	i915_debugfs_vars.rc6.rc6_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}

static ssize_t
i915_write_rc6_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* reset the string */
	memset(i915_debugfs_vars.rc6.rc6_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.rc6.rc6_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.rc6.rc6_vars, ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.rc6.rc6_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.rc6.rc6_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_rc6_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_rc6_api,
	.write = i915_write_rc6_api,
	.llseek = default_llseek,
};

static int
i915_read_turbo_api(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[200], control[10], operation[20], val[20], format[20];
	int len = 0, ret, noOfTokens;
	u32 pval = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.turbo.turbo_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%ds %%%ds %%%ds",
			sizeof(control), sizeof(operation), sizeof(val));

	noOfTokens = sscanf(i915_debugfs_vars.turbo.turbo_vars,
				format, control, operation, val);

	if (noOfTokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.turbo.turbo_vars);

	if (strcmp(operation, DETAILS_TOKEN) == 0) {
		ret = mutex_lock_interruptible(&dev_priv->rps.rps_mutex);
		if (ret)
			return ret;

		pval = I915_READ(GEN6_RP_CONTROL);
		len = snprintf(buf, sizeof(buf),
				"Turbo Enabled: %s\n",
				yesno(pval & GEN6_RP_ENABLE));

		if (len < 0)
			return len;

		len += snprintf(&buf[len], (sizeof(buf) - len),
				"Max Gpu Freq _max_delay_: %d\n",
				dev_priv->rps.max_delay);
		len += snprintf(&buf[len], (sizeof(buf) - len),
				"Min Gpu Freq _min_delay_: %d\n",
				dev_priv->rps.min_delay);

		intel_punit_read32(dev_priv, PUNIT_REG_GPU_FREQ_STS,	&pval);
		len += snprintf(&buf[len], (sizeof(buf) - len),
				"Cur Gpu Freq _cur_delay_: %d\n", pval >> 8);
		len += snprintf(&buf[len], (sizeof(buf) - len),
				"Up Threshold: %d\n", atomic_read(
					&dev_priv->turbodebug.up_threshold));
		len += snprintf(&buf[len], (sizeof(buf) - len),
				"Down Threshold: %d\n",	atomic_read(
					&dev_priv->turbodebug.down_threshold));
		len += snprintf(&buf[len], (sizeof(buf) - len),
				"RP_UP: %d\nRP_DOWN:%d\n",
				dev_priv->rps.rp_up_masked,
				dev_priv->rps.rp_down_masked);

		mutex_unlock(&dev_priv->rps.rps_mutex);

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {

		/* 1=> Enable Turbo, else disable. */

		ret = i915_rps_enable_disable(dev, 1);
		if (ret)
			return ret;

		len = snprintf(buf, sizeof(buf),
				"Turbo Enabled: Yes\n");

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {

		/* 1=> Enable Turbo, else disable. */

		ret = i915_rps_enable_disable(dev, 0);
		if (ret)
			return ret;

		len = snprintf(buf, sizeof(buf),
				"Turbo Enabled: No\n");

	} else if (strcmp(operation, RP_MAXFREQ_TOKEN) == 0) {

		ret = kstrtoul(val, 0, &pval);
		if (ret)
			return -EINVAL;

		ret = i915_set_max_freq(dev, pval);
		if (ret)
			return ret;

		len = snprintf(buf, sizeof(buf),
				"OPERATION: SUCCESSFUL\n");
	} else if (strcmp(operation, RP_MINFREQ_TOKEN) == 0) {

		ret = kstrtoul(val, 0, &pval);
		if (ret)
			return -EINVAL;

		ret = i915_set_min_freq(dev, pval);
		if (ret)
			return ret;

		len = snprintf(buf, sizeof(buf),
				"OPERATION: SUCCESSFUL\n");

	} else
		len = snprintf(buf, sizeof(buf),
				"NOTSUPPORTED\n");

	if (len > sizeof(buf))
		len = sizeof(buf);

	i915_debugfs_vars.turbo.turbo_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}


static ssize_t
i915_write_turbo_api(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* Reset the string */
	memset(i915_debugfs_vars.turbo.turbo_vars, 0, MAX_BUFFER_STR_LEN);

	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.turbo.turbo_vars) - 1)
			return -EINVAL;

		if (copy_from_user(i915_debugfs_vars.turbo.turbo_vars,
					ubuf, cnt))
			return -EFAULT;

		i915_debugfs_vars.turbo.turbo_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.turbo.turbo_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_turbo_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_turbo_api,
	.write = i915_write_turbo_api,
	.llseek = default_llseek,
};

/* Helper function to enable and disable dpst based on input */
static int
i915_dpst_status(struct drm_device *dev, char *buf, int *len)
{
	int ret;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = snprintf(buf, MAX_BUFFER_STR_LEN, "DPST Enabled: %s\n",
			yesno(dev_priv->is_dpst_enabled ? 1 : 0));

	return 0;
}

static int
i915_dpst_irq_count(struct drm_device *dev, char *buf, int *len)
{
	int ret;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = snprintf(buf, MAX_BUFFER_STR_LEN, "DPST Interrupt Count: %d\n",
					dev_priv->dpst.num_interrupt);

	return 0;
}

static int
i915_dpst_enable_disable(struct drm_device *dev, unsigned int val)
{
	int ret;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* 1=> Enable DPST, else disable. */

	if (val == 1)
		i915_dpst_enable_hist_interrupt(dev, true);
	else
		i915_dpst_enable_hist_interrupt(dev, false);

	return 0;
}

static int
i915_dpst_dump_reg(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	u32 btgr_data,  hcr_data, bpcr_data, dpst_set_level;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	btgr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BTGR_REG);
	hcr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_IEHCR_REG);
	bpcr_data = I915_READ(VLV_DISPLAY_BASE + DPST_VLV_BPCR_REG);
	dpst_set_level = I915_READ(
			VLV_DISPLAY_BASE + DPST_VLV_BPCR_REG) & 0xffff;

	*len = snprintf(buf, MAX_BUFFER_STR_LEN, "IEBTGR: 0x%x & IEHCR: 0x%x",
				btgr_data, hcr_data);

	*len += snprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
			" & IEBPCR: 0x%x DPST_SET_LEVEL: 0x%x\n",
			 bpcr_data, dpst_set_level);

	return 0;
}

static int
i915_dpst_get_bin_data(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int ret, index;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = snprintf(buf, MAX_BUFFER_STR_LEN, "Bin Data:\n");

	for (index = 0; index < DPST_BIN_COUNT; index++)
		*len += snprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
				"%d ", dev_priv->dpst.bin_data[index]);

	*len += snprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len), "\n");

	return 0;
}

static int
i915_dpst_get_luma_data(struct drm_device *dev, char *buf, int *len)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	int ret, index;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	*len = snprintf(buf, MAX_BUFFER_STR_LEN, "LUMA Data:\n");

	for (index = 0; index < DPST_LUMA_COUNT; index++)
		*len += snprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len),
				"%d ", dev_priv->dpst.luma_data[index]);

	*len += snprintf(&buf[*len], (MAX_BUFFER_STR_LEN - *len), "\n");

	return 0;
}



static int
i915_read_dpst_api(struct file *filp,
			char __user *ubuf,
			size_t max,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	char buf[200], control[10], operation[20], val[20], format[20];
	int len = 0, ret, noOfTokens;
	u32 pval = 0, dpst_set_level;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (i915_debugfs_vars.dpst.dpst_input == 0)
		return len;

	snprintf(format, sizeof(format), "%%%ds %%%ds %%%ds",
			sizeof(control), sizeof(operation), sizeof(val));

	noOfTokens = sscanf(i915_debugfs_vars.dpst.dpst_vars,
				format, control, operation, val);
	if (noOfTokens < 3)
		return len;

	len = sizeof(i915_debugfs_vars.dpst.dpst_vars);

	if (strcmp(operation, STATUS_TOKEN) == 0) {
		ret = i915_dpst_status(dev, buf, &len);
		if (ret)
			return ret;

	} else if (strcmp(operation, ENABLE_TOKEN) == 0) {

		/* 1 => Enable DPST. */
		ret = i915_dpst_enable_disable(dev, 1);
		if (ret)
			return ret;

		i915_dpst_status(dev, buf, &len);

	} else if (strcmp(operation, DISABLE_TOKEN) == 0) {

		/* 0 => Disable DPST */

		ret = i915_dpst_enable_disable(dev, 0);
		if (ret)
			return ret;

		i915_dpst_status(dev, buf, &len);

	} else if (strcmp(operation, DPST_DUMP_REG_TOKEN) == 0) {
		i915_dpst_dump_reg(dev, buf, &len);

	} else if (strcmp(operation, DPST_GET_BIN_DATA_TOKEN) == 0) {
		i915_dpst_get_bin_data(dev, buf, &len);

	} else if (strcmp(operation, DPST_GET_LUMA_DATA_TOKEN) == 0) {
		i915_dpst_get_luma_data(dev, buf, &len);

	} else if (strcmp(operation, DPST_IRQ_COUNT_TOKEN) == 0) {
		i915_dpst_irq_count(dev, buf, &len);

	} else if (strcmp(operation, DPST_FACTOR_TOKEN) == 0) {
		len = snprintf(buf, sizeof(buf),
				"DPST Backlight Factor: %d\n",
				(dev_priv->dpst_backlight_factor / 100));

	} else if (strcmp(operation, DPST_LEVEL_TOKEN) == 0) {
		dpst_set_level = I915_READ(
				VLV_DISPLAY_BASE + DPST_VLV_BPCR_REG) & 0xffff;

		len = snprintf(buf, sizeof(buf),
				"DPST Current User Level: 0x%x\n",
				(dev_priv->backlight_level));

		if (len < 0)
			return len;

		len += snprintf(&buf[len], (sizeof(buf) - len),
				"DPST Current Backlight Level: 0x%x\n",
				dpst_set_level);
	} else
		len = snprintf(buf, sizeof(buf), "NOTSUPPORTED\n");

	if (len > sizeof(buf))
		len = sizeof(buf);

	i915_debugfs_vars.dpst.dpst_input = 0;
	simple_read_from_buffer(ubuf, max, ppos, buf, len);

	return len;
}


static ssize_t
i915_write_dpst_api(struct file *filp,
			const char __user *ubuf,
			size_t cnt,
			loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	/* Reset the string */
	memset(i915_debugfs_vars.dpst.dpst_vars, 0, MAX_BUFFER_STR_LEN);
	if (cnt > 0) {
		if (cnt > sizeof(i915_debugfs_vars.dpst.dpst_vars) - 1)
			return -EINVAL;
		if (copy_from_user(i915_debugfs_vars.dpst.dpst_vars,
						ubuf, cnt))
			return -EFAULT;
		i915_debugfs_vars.dpst.dpst_vars[cnt] = 0;

		/* Enable read */
		i915_debugfs_vars.dpst.dpst_input = 1;
	}

	return cnt;
}

static const struct file_operations i915_dpst_fops = {
.owner = THIS_MODULE,
.open = simple_open,
.read = i915_read_dpst_api,
.write = i915_write_dpst_api,
.llseek = default_llseek,
};


static int
i915_read_rc6_status(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	char buf[80];
	int len = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	len = snprintf(buf, sizeof(buf),
		"RC6 is %s\n",
		(is_rc6_enabled(dev)) ?
				"enabled" : "disabled");
	if (len < 0)
		return len;

	len += snprintf(&buf[len], (sizeof(buf) - len),
		"Render well is %s & Media well is %s\n",
		(I915_READ(VLV_POWER_WELL_STATUS_REG) &
			VLV_RENDER_POWER_WELL_STATUS_BIT) ? "UP" : "DOWN",
		(I915_READ(VLV_POWER_WELL_STATUS_REG) &
			VLV_MEDIA_POWER_WELL_STATUS_BIT) ? "UP" : "DOWN");


	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(ubuf, max, ppos, buf, len);
}

static ssize_t
i915_write_rc6_status(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	int ret = 0;
	long unsigned int val = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		ret = kstrtoul(buf, 0, (unsigned long *)&val);
		if (ret)
			return -EINVAL;
	}



	/*
	* 1=> Enable RC6, else disable.
	*/
	ret = rc6_enable_disable(dev, val);
	if (ret)
		return ret;

	return cnt;
}

static const struct file_operations i915_rc6_status_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_read_rc6_status,
	.write = i915_write_rc6_status,
	.llseek = default_llseek,
};

static int i915_forcewake_create(struct dentry *root, struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file("i915_forcewake_user",
				  S_IRUSR,
				  root, dev,
				  &i915_forcewake_fops);
	if (IS_ERR(ent))
		return PTR_ERR(ent);

	return drm_add_fake_info_node(minor, ent, &i915_forcewake_fops);
}

static ssize_t
i915_rpm_debug_read(struct file *filp,
		   char __user *ubuf,
		   size_t max,
		   loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	return 0;
}

static ssize_t
i915_rpm_debug_write(struct file *filp,
		  const char __user *ubuf,
		  size_t cnt,
		  loff_t *ppos)
{
	struct drm_device *dev = filp->private_data;
	char buf[20];
	long unsigned int val = 0;

	if (!(IS_VALLEYVIEW(dev)))
		return -ENODEV;

	if (cnt > 0) {
		if (cnt > sizeof(buf) - 1)
			return -EINVAL;

		if (copy_from_user(buf, ubuf, cnt))
			return -EFAULT;
		buf[cnt] = 0;

		if (kstrtoul(buf, 0, (unsigned long *)&val))
			return -EINVAL;
	}

	if (val > 1)
		return -EINVAL;

	if (val)
		display_runtime_suspend(dev);
	else
		display_runtime_resume(dev);

	return cnt;
}

static const struct file_operations i915_rpm_debug_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = i915_rpm_debug_read,
	.write = i915_rpm_debug_write,
	.llseek = default_llseek,
};

static int i915_debugfs_create(struct dentry *root,
			       struct drm_minor *minor,
			       const char *name,
			       const struct file_operations *fops)
{
	struct drm_device *dev = minor->dev;
	struct dentry *ent;

	ent = debugfs_create_file(name,
				  S_IRUGO | S_IWUSR,
				  root, dev,
				  fops);
	if (IS_ERR(ent))
		return PTR_ERR(ent);

	return drm_add_fake_info_node(minor, ent, fops);
}

static struct drm_info_list i915_debugfs_list[] = {
	{"i915_capabilities", i915_capabilities, 0},
	{"i915_gem_objects", i915_gem_object_info, 0},
	{"i915_gem_gtt", i915_gem_gtt_info, 0},
	{"i915_gem_pinned", i915_gem_gtt_info, 0, (void *) PINNED_LIST},
	{"i915_gem_active", i915_gem_object_list_info, 0, (void *) ACTIVE_LIST},
	{"i915_gem_inactive", i915_gem_object_list_info, 0, (void *) INACTIVE_LIST},
	{"i915_gem_pageflip", i915_gem_pageflip_info, 0},
	{"i915_gem_request", i915_gem_request_info, 0},
	{"i915_gem_seqno", i915_gem_seqno_info, 0},
	{"i915_gem_fence_regs", i915_gem_fence_regs_info, 0},
	{"i915_gem_interrupt", i915_interrupt_info, 0},
	{"i915_gem_hws", i915_hws_info, 0, (void *)RCS},
	{"i915_gem_hws_blt", i915_hws_info, 0, (void *)BCS},
	{"i915_gem_hws_bsd", i915_hws_info, 0, (void *)VCS},
	{"i915_rstdby_delays", i915_rstdby_delays, 0},
	{"i915_cur_delayinfo", i915_cur_delayinfo, 0},
	{"i915_delayfreq_table", i915_delayfreq_table, 0},
	{"i915_inttoext_table", i915_inttoext_table, 0},
	{"i915_drpc_info", i915_drpc_info, 0},
	{"i915_emon_status", i915_emon_status, 0},
	{"i915_ring_freq_table", i915_ring_freq_table, 0},
	{"i915_gfxec", i915_gfxec, 0},
	{"i915_fbc_status", i915_fbc_status, 0},
	{"i915_sr_status", i915_sr_status, 0},
	{"i915_opregion", i915_opregion, 0},
	{"i915_gem_framebuffer", i915_gem_framebuffer_info, 0},
	{"i915_context_status", i915_context_status, 0},
	{"i915_gen6_forcewake_count", i915_gen6_forcewake_count_info, 0},
	{"i915_swizzle_info", i915_swizzle_info, 0},
	{"i915_ppgtt_info", i915_ppgtt_info, 0},
	{"i915_dpio", i915_dpio_info, 0},
};
#define I915_DEBUGFS_ENTRIES ARRAY_SIZE(i915_debugfs_list)

int i915_debugfs_init(struct drm_minor *minor)
{
	int ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_wedged",
					&i915_wedged_fops);
	if (ret)
		return ret;

	ret = i915_forcewake_create(minor->debugfs_root, minor);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_max_freq",
					&i915_max_freq_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_min_freq",
					&i915_min_freq_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_rps_init",
					&i915_rps_init_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_mmio_api",
					&i915_mmio_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_iosf_api",
					&i915_iosf_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_rc6_api",
					&i915_rc6_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
				  "i915_ring_hangcheck",
				  &i915_ring_hangcheck_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
				  "i915_error_state",
				  &i915_error_state_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_turbo_api",
					&i915_turbo_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_dpst_api",
					&i915_dpst_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_cache_sharing",
					&i915_cache_sharing_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_ring_stop",
					&i915_ring_stop_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_error_state",
					&i915_error_state_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_rc6_status",
					&i915_rc6_status_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"i915_rpm_debug",
					&i915_rpm_debug_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"cb_adjust",
					&i915_cb_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"hs_adjust",
					&i915_hs_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"csc_adjust",
					&i915_csc_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"csc_enable",
					&i915_csc_enable_fops);

	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"gamma_adjust",
					&i915_gamma_adjust_fops);
	if (ret)
		return ret;

	ret = i915_debugfs_create(minor->debugfs_root, minor,
					"gamma_enable",
					&i915_gamma_enable_fops);
	if (ret)
		return ret;

	return drm_debugfs_create_files(i915_debugfs_list,
					I915_DEBUGFS_ENTRIES,
					minor->debugfs_root, minor);
}

void i915_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(i915_debugfs_list,
				 I915_DEBUGFS_ENTRIES, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_forcewake_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_wedged_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_max_freq_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_min_freq_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_rps_init_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_iosf_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_mmio_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_rc6_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_turbo_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_dpst_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_cache_sharing_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_ring_stop_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *)
				&i915_ring_hangcheck_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_error_state_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_rc6_status_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_cb_adjust_fops,
				 1, minor);
	drm_debugfs_remove_files((struct drm_info_list *) &i915_hs_adjust_fops,
				 1, minor);
}

#endif /* CONFIG_DEBUG_FS */
