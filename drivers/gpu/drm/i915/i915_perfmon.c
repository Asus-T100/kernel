/*
 * Copyright  2013 Intel Corporation
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
 *	Adam Rutkowski <adam.j.rutkowski@intel.com>
 */
#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"
#include "drm_fb_helper.h"
#include "intel_drv.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "i915_perfmon.h"

/**
 * valleyview_rp_to_mhz - convert RP freq encoding to MHz
 */
int valleyview_rp_to_mhz(struct drm_i915_private *dev_priv, __u32 rp_freq,
	__u32 *freq)
{
	if (dev_priv->gpll) {
		/* GPLL enabled */
		if (rp_freq < 0xb7 || rp_freq > 0xff)
			return -EINVAL;

		switch (dev_priv->mem_freq) {
		case 800:
			*freq = 20 * (rp_freq - 0xb7);
			break;
		case 1066:
			*freq = (200 * (rp_freq - 0xb7) + 4) / 9;
			break;
		case 1333:
			*freq = (125 * (rp_freq - 0xb7) + 3) / 6;
			break;
		default:
			return -EINVAL;
		}
	} else {
		/* GPLL disabled */
		if (rp_freq < 0x1 || rp_freq > 0x1f)
			return -EINVAL;
		*freq = (2 * dev_priv->cck_freq + (rp_freq + 1) / 2) /
			(rp_freq + 1);
	}
	return 0;
}

/**
 * intel_get_freq_info - return GPU frequency in MHz
 *
 * Returns minimum, maximum and current turbo frequency
 * in units of MHz.
 */
int intel_get_freq_info(struct drm_device *dev,
			__u32 *min_freq,
			__u32 *max_freq,
			__u32 *cur_freq)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int retcode = 0;
	int freq_sts = 0;

	*min_freq = 0;
	*max_freq = 0;
	*cur_freq = 0;

	if (!IS_VALLEYVIEW(dev))
		return -EINVAL;

	retcode = intel_punit_read32(dev_priv, PUNIT_REG_GPU_FREQ_STS,
		&freq_sts);
	if (!retcode)
		retcode = valleyview_rp_to_mhz(dev_priv,
			(freq_sts >> 8) & 0xff, cur_freq);
	if (!retcode)
		retcode = valleyview_rp_to_mhz(dev_priv,
			dev_priv->rps.min_delay, min_freq);
	if (!retcode)
		retcode = valleyview_rp_to_mhz(dev_priv,
			dev_priv->rps.max_delay, max_freq);
	return retcode;
}


/**
 * i915_perfmon_update_override_counter - update override state counter
 *
 * Implements overrides reference counting. For each override there
 * is a reference counter in both device (dev_priv) and file (file_priv).
 * Enabling(disabling) override increments these two counters by +1 (-1).
 * When device counter has incremented from 0, 'toggle' is set to 1 indicating
 * that action needs to be taken to turn the override on. Similarly
 * when device counter drops to 0 'toggle' is set to -1 to indicate that
 * caller must turn the override off.
 * When file is being closed, device counter is decremented by the value of
 * counter in file_priv corresponding to the file being closed.
 */
int i915_perfmon_update_override_counter(int *device_counter,
	int *file_counter,
	int increment,
	int *toggle)
{
	*toggle = 0;
	if (*file_counter + increment < 0) {
		if (*file_counter > 0)
			increment = -1 * (*file_counter);
		else
			return -EINVAL;
	}
	if (increment > 0 && *device_counter == 0)
		*toggle = 1;
	*device_counter += increment;
	*file_counter += increment;
	if (increment < 0 && *device_counter == 0)
		*toggle = -1;
	return 0;
}

/**
 * i915_perfmon_update_max_freq_override - enable max GPU frequency override
 *
 * Overrides turbo algorithm to switch GPU to maximum frequency.
 */
static int i915_perfmon_update_max_freq_override(struct drm_device *dev,
	struct drm_file *file,
	int increment)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_file_private *file_priv = file->driver_priv;
	int ret_val;
	int toggle = 0;

	if (!IS_VALLEYVIEW(dev))
		return -EINVAL;

	mutex_lock(&dev_priv->rps.rps_mutex);
	ret_val = i915_perfmon_update_override_counter(
				&dev_priv->max_freq_enable_count,
				&file_priv->perfmon_override_counter.max_freq,
				increment,
				&toggle);
	if (!ret_val && toggle) {
		if (toggle == 1) {
			vlv_turbo_disable(dev);
			valleyview_set_rps(dev, dev_priv->rps.max_delay);
		} else
			vlv_turbo_initialize(dev);
	}
	mutex_unlock(&dev_priv->rps.rps_mutex);

	return ret_val;
}

/**
 * i915_perfmon_update_rc6_disable_override - set RC6 state
 *
 * Enable and re-enable RC6 on demand in runtime.
 */
static int i915_perfmon_update_rc6_disable_override(struct drm_device *dev,
	struct drm_file *file,
	int increment)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_file_private *file_priv = file->driver_priv;

	int ret_val;
	int toggle = 0;

	if (!IS_VALLEYVIEW(dev))
		return -EINVAL;

	if (i915_enable_rc6 >= 0) {
		int rc6_module_setting =
			((i915_enable_rc6 & INTEL_RC6_ENABLE) != 0);
		int rc6_enable = (increment < 0);
		if (rc6_module_setting == rc6_enable)
			return 0;
		else
			return -EINVAL;
	}

	mutex_lock(&dev_priv->rps.rps_mutex);
	ret_val = i915_perfmon_update_override_counter(
			&dev_priv->rc6_user_disable_count,
			&file_priv->perfmon_override_counter.rc6_disable,
			increment,
			&toggle);
	if (!ret_val && toggle) {
		if (toggle == 1)
			vlv_rs_setstate(dev, false);
		else
			vlv_rs_setstate(dev, true);
	}
	mutex_unlock(&dev_priv->rps.rps_mutex);

	return ret_val;
}


/**
 * i915_perfmon_ioctl - performance monitoring support
 *
 * Main entry point to performance monitoring support
 * IOCTLs.
 */
int i915_perfmon_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file)
{
	struct drm_i915_perfmon *perfmon = data;
	int retcode = 0;

	switch (perfmon->op) {
	case I915_PERFMON_SET_RC6:
		retcode = i915_perfmon_update_rc6_disable_override(dev,
			file,
			perfmon->data.set_rc6.enable ? -1 : 1);
		break;
	case I915_PERFMON_GET_FREQ_INFO:
		retcode = intel_get_freq_info(dev,
			&perfmon->data.freq_info.min_gpu_freq,
			&perfmon->data.freq_info.max_gpu_freq,
			&perfmon->data.freq_info.cur_gpu_freq);
		break;
	case I915_PERFMON_SET_MAX_FREQ:
		retcode = i915_perfmon_update_max_freq_override(dev,
			file,
			perfmon->data.set_max_freq.enable ? 1 : -1);
		break;
	case I915_PERFMON_ALLOC_OA_BUFFER:
	case I915_PERFMON_FREE_OA_BUFFER:
	case I915_PERFMON_SET_OA_IRQS:
		/* not supported yet */
		retcode = -EINVAL;
		break;
	default:
		/* unknown operation */
		retcode = -EINVAL;
		break;
	}

	return retcode;
}

/**
 * i915_perfmon_init - initialize perfmon overrides
 *
 * Initialize perfmon overrides per-file data.
 */
void i915_perfmon_init(struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	file_priv->perfmon_override_counter.rc6_disable = 0;
	file_priv->perfmon_override_counter.max_freq = 0;
}

/**
 * i915_perfmon_close - clean up perfmon upon driver close
 *
 * Perfmon overrides are being disabled here.
 */
void i915_perfmon_close(struct drm_device *dev, struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	if (file_priv->perfmon_override_counter.rc6_disable > 0)
		i915_perfmon_update_rc6_disable_override(dev, file,
			-1 * file_priv->perfmon_override_counter.rc6_disable);

	if (file_priv->perfmon_override_counter.max_freq > 0)
		i915_perfmon_update_max_freq_override(dev, file,
			-1 * file_priv->perfmon_override_counter.max_freq);
}
