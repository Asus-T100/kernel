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
 *	Jesse Barnes <jesse.barnes@intel.com>
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 *	Yogesh Mohan Marimuthu <yogesh.mohan.marimuthu@intel.com>
 */

#include <linux/kernel.h>
#include "intel_drv.h"
#include "i915_drv.h"


int intel_sideband_read32(struct drm_i915_private *dev_priv, u8 port_id,
		u32 reg, u32 *val)
{
	unsigned long flags;
	u32 cmd, devfn, opcode, port, be, bar;

	bar = 0;
	be = 0xf;
	port = port_id;
	opcode = IOSF_OPCODE_REG_READ;
	devfn = 0x00;

	if (port_id == IOSF_PORT_PMC)
		opcode = 0x0;

	cmd = (devfn << IOSF_DEVFN_SHIFT) | (opcode << IOSF_OPCODE_SHIFT) |
		(port << IOSF_PORT_SHIFT) | (be << IOSF_BYTE_ENABLES_SHIFT) |
		(bar << IOSF_BAR_SHIFT);

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for_atomic_us((I915_READ(IOSF_DOORBELL_REQ) & IOSF_SB_BUSY)
							== 0, 100)) {
		DRM_ERROR("IOSF sideband idle wait timed out\n");
		spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
		return -EAGAIN;
	}


	I915_WRITE(IOSF_ADDR, reg);
	I915_WRITE(IOSF_DOORBELL_REQ, cmd);

	if (wait_for_atomic_us((I915_READ(IOSF_DOORBELL_REQ) &
					IOSF_SB_BUSY) == 0, 100)) {
		DRM_ERROR("IOSF sideband read wait timed out\n");
		spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
		return -ETIMEDOUT;
	}

	*val = I915_READ(IOSF_DATA);
	I915_WRITE(IOSF_DATA, 0);

	spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
	return 0;
}

int intel_sideband_write32(struct drm_i915_private *dev_priv, u8 port_id,
		u32 reg, u32 val)
{
	unsigned long flags;
	u32 cmd, devfn, opcode, port, be, bar;

	bar = 0;
	be = 0xf;
	port = port_id;
	opcode = IOSF_OPCODE_REG_WRITE;
	devfn = 0x00;

	if (port_id == IOSF_PORT_PMC)
		opcode = 0x1;

	cmd = (devfn << IOSF_DEVFN_SHIFT) | (opcode << IOSF_OPCODE_SHIFT) |
		(port << IOSF_PORT_SHIFT) | (be << IOSF_BYTE_ENABLES_SHIFT) |
		(bar << IOSF_BAR_SHIFT);

	spin_lock_irqsave(&dev_priv->dpio_lock, flags);
	if (wait_for_atomic_us((I915_READ(IOSF_DOORBELL_REQ) &
						IOSF_SB_BUSY) == 0, 100)) {
		DRM_ERROR("IOSF Sideband idle wait timed out\n");
		spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
		return -EAGAIN;
	}

	I915_WRITE(IOSF_ADDR, reg);
	I915_WRITE(IOSF_DATA, val);
	I915_WRITE(IOSF_DOORBELL_REQ, cmd);

	if (wait_for_atomic_us((I915_READ(IOSF_DOORBELL_REQ) &
						IOSF_SB_BUSY) == 0, 100)) {
		DRM_ERROR("IOSF Sideband write wait timed out\n");
		spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
		return -ETIMEDOUT;
	}

	I915_WRITE(IOSF_DATA, 0);

	spin_unlock_irqrestore(&dev_priv->dpio_lock, flags);
	return 0;
}

int intel_sideband_write32_bits(struct drm_i915_private *dev_priv, int port_id,
			u32 reg, u32 val, u32 mask)
{
	u32 tmp;
	int status;

	status = intel_sideband_read32(dev_priv, port_id, reg, &tmp);
	if (status != 0)
		return status;

	tmp = tmp & ~mask;
	val = val & mask;
	tmp = val | tmp;

	return intel_sideband_write32(dev_priv, port_id, reg, tmp);
}

u32 intel_dpio_read32_tmp(struct drm_i915_private *dev_priv, u32 reg)
{
	u32 val;
	val = 0;

	intel_sideband_read32(dev_priv, IOSF_PORT_DPIO, reg, &val);
	return val;
}

int intel_dpio_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val)
{
	return intel_sideband_read32(dev_priv, IOSF_PORT_DPIO, reg, val);
}

int intel_dpio_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_DPIO, reg, val);
}

int intel_dpio_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_DPIO, reg, val, mask);
}

int intel_punit_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_PUNIT, reg, val);
}

int intel_punit_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_PUNIT, reg, val);
}

int intel_punit_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_PUNIT, reg, val, mask);
}

int intel_gpio_nc_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_GPIO_NC, reg, val);
}

int intel_gpio_nc_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_GPIO_NC, reg, val);
}

int intel_gpio_nc_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_GPIO_NC, reg, val, mask);
}

int intel_cck_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_CCK, reg, val);
}

int intel_cck_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_CCK, reg, val);
}

int intel_cck_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_CCK, reg, val, mask);
}

int intel_ccu_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_CCU, reg, val);
}

int intel_ccu_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_CCU, reg, val);
}

int intel_ccu_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_CCU, reg, val, mask);
}

int intel_gps_core_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{
	return intel_sideband_read32(dev_priv, IOSF_PORT_GPS_CORE, reg, val);
}

int intel_gps_core_write32(struct drm_i915_private *dev_priv, u32 reg,
				u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_GPS_CORE, reg, val);
}

int intel_gps_core_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_GPS_CORE, reg, val, mask);
}

int intel_pmc_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_PMC, reg, val);
}

int intel_pmc_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_PMC, reg, val);
}

int intel_pmc_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_PMC, reg, val, mask);
}

int intel_flisdsi_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{

	return intel_sideband_read32(dev_priv, IOSF_PORT_FLISDSI, reg, val);
}

int intel_flisdsi_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val)
{
	return intel_sideband_write32(dev_priv, IOSF_PORT_FLISDSI, reg, val);
}

int intel_flisdsi_write32_bits(struct drm_i915_private *dev_priv, u32 reg,
				u32 val, u32 mask)
{
	return intel_sideband_write32_bits(dev_priv,
				IOSF_PORT_FLISDSI, reg, val, mask);
}

/* Function reads the fuse port */
int intel_fuse_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val)
{
	return intel_sideband_read32(dev_priv, IOSF_PORT_FUSE, reg, val);
}
