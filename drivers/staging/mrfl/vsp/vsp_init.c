/**
 * file vsp_init.c
 * VSP initialization and firmware upload
 * Author: Binglin Chen <binglin.chen@intel.com>
 *
 */

/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#include <linux/firmware.h>

#include <drm/drmP.h>
#include <drm/drm.h>

#include "vsp.h"

static void vsp_enter_start_mode(struct drm_psb_private *dev_priv);

static ssize_t psb_vsp_pmstate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	ret = snprintf(buf, 64, "VSP Power state 0x%s\n",
			ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND)
			? "ON" : "OFF");

	return ret;
}
static DEVICE_ATTR(vsp_pmstate, 0444, psb_vsp_pmstate_show, NULL);

int vsp_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	struct vsp_private *vsp_priv;
	int ret;

	VSP_DEBUG("init vsp private data structure\n");
	vsp_priv = kmalloc(sizeof(struct vsp_private), GFP_KERNEL);
	if (vsp_priv == NULL)
		return -1;

	memset(vsp_priv, 0, sizeof(*vsp_priv));

#ifdef MRFL_VSP_POWER_MANAGEMENT
	/* get device --> drm_device --> drm_psb_private --> vsp_priv
	 * for psb_vsp_pmstate_show: vsp_pmpolicy
	 * if not pci_set_drvdata, can't get drm_device from device
	 */
	/* pci_set_drvdata(dev->pdev, dev); */
	if (device_create_file(&dev->pdev->dev,
			       &dev_attr_vsp_pmstate))
		DRM_ERROR("TOPAZ: could not create sysfs file\n");

	vsp_priv->sysfs_pmstate = sysfs_get_dirent(
					    dev->pdev->dev.kobj.sd,
					    "vsp_pmstate");
#endif

	vsp_priv->fw_loaded = 0;
	vsp_priv->needs_reset = 1;
	vsp_priv->current_sequence = 0;
	vsp_priv->vsp_busy = 0;

	dev_priv->vsp_private = vsp_priv;

	VSP_DEBUG("allocate buffer for fw\n");
	/* FIXME: assume 1 page, will modify to a proper value */
	vsp_priv->ppc_vp0_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->ppc_vp0_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->ppc_vp0_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer "
			  "for ppc_vp0_fw\n");
		goto out_clean;
	}

	vsp_priv->ppc_vp1_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->ppc_vp1_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->ppc_vp1_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer "
			  "for ppc_vp1_fw\n");
		goto out_clean;
	}

	vsp_priv->vpp_chain_vp0_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->vpp_chain_vp0_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->vpp_chain_vp0_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer "
			  "for vpp_chain_vp0_fw\n");
		goto out_clean;
	}

	vsp_priv->vpp_chain_sp1_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->vpp_chain_sp1_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->vpp_chain_sp1_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer for "
			  "vpp_chain_sp1_fw\n");
		goto out_clean;
	}

	vsp_priv->vpp_chain_vp1_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->vpp_chain_vp1_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->vpp_chain_vp1_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer "
			  "for vpp_chain_vp1_fw\n");
		goto out_clean;
	}

	vsp_priv->memc_app_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->memc_app_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->memc_app_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer "
			  "for memc_app_fw\n");
		goto out_clean;
	}

	vsp_priv->proc_fw_sz = 4096;
	ret = ttm_buffer_object_create(bdev, vsp_priv->proc_fw_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->proc_fw);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer"
			  " for proc_fw\n");
		goto out_clean;
	}

	/* 1M for context */
	vsp_priv->context_size = 8 * 1024 * 1024;
	ret = ttm_buffer_object_create(bdev, vsp_priv->context_size,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->context_buf);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP context buf\n");
		goto out_clean;
	}

	spin_lock_init(&vsp_priv->lock);

	return 0;
out_clean:
	vsp_deinit(dev);
	return -1;
}

int vsp_deinit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	VSP_DEBUG("free VSP firmware/context buffer\n");
	if (vsp_priv->context_buf)
		ttm_bo_unref(&vsp_priv->context_buf);
	if (vsp_priv->ppc_vp0_fw)
		ttm_bo_unref(&vsp_priv->ppc_vp0_fw);
	if (vsp_priv->ppc_vp1_fw)
		ttm_bo_unref(&vsp_priv->ppc_vp1_fw);
	if (vsp_priv->vpp_chain_vp0_fw)
		ttm_bo_unref(&vsp_priv->vpp_chain_vp0_fw);
	if (vsp_priv->vpp_chain_vp1_fw)
		ttm_bo_unref(&vsp_priv->vpp_chain_vp1_fw);
	if (vsp_priv->vpp_chain_sp1_fw)
		ttm_bo_unref(&vsp_priv->vpp_chain_sp1_fw);
	if (vsp_priv->memc_app_fw)
		ttm_bo_unref(&vsp_priv->memc_app_fw);
	if (vsp_priv->proc_fw)
		ttm_bo_unref(&vsp_priv->proc_fw);

	device_remove_file(&dev->pdev->dev, &dev_attr_vsp_pmstate);
	sysfs_put(vsp_priv->sysfs_pmstate);

	VSP_DEBUG("free VSP private structure\n");
	kfree(dev_priv->vsp_private);

	return 0;
}

void vsp_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int mask;
	unsigned int enable;
	unsigned int clear;

	VSP_DEBUG("will enable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);
	clear = 0;

	VSP_SET_FLAG(mask, VSP_SP1_IRQ_SHIFT);
	VSP_SET_FLAG(enable, VSP_SP1_IRQ_SHIFT);
	VSP_SET_FLAG(clear, VSP_SP1_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_EDGE, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_CLR, clear);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
}

void vsp_disableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int mask, enable;

	VSP_DEBUG("will disable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);

	VSP_CLEAR_FLAG(mask, VSP_SP1_IRQ_SHIFT);
	VSP_CLEAR_FLAG(enable, VSP_SP1_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
}

int vsp_init_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	VSP_DEBUG("read fimware into buffer\n");

	vsp_priv->fw_loaded = 1;
	vsp_priv->needs_reset = 1;

	return 0;
}

int vsp_reset(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret;

	ret = vsp_setup_fw(dev_priv);

	vsp_priv->needs_reset = 0;

	return ret;
}

int vsp_setup_fw(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	/* only for csim on VP */
	char *exe_prefix = "csim_executable=";
	char *exe_file_name = "proc";
	char *input;
	unsigned int val;
	int queue_num;

	VSP_DEBUG("clean HW before firmware uploaded\n");
	VSP_DEBUG("FIXME: not sure if this is required before "
		  "firmware upload every time\n");
	/* clean ECA */

	/* clean SP0 */
	SP0_REG_WRITE32(SP0_CFG_PMEM_MASTER, 0);
	SP0_REG_WRITE32(SP0_XMEM_MASTER, 0);

	/* clean SP1 */
	SP1_REG_WRITE32(SP1_CFG_PMEM_MASTER, 0);
	SP1_REG_WRITE32(SP1_XMEM_MASTER, 0);

	/* clean VP0 */
	VP0_REG_WRITE32(VP0_CFG_PMEM_MASTER, 0);
	VP0_REG_WRITE32(VP0_XD2MEM_MASTER, 0);
	VP0_REG_WRITE32(VP0_XD2MEMW_MASTER, 0);
	VP0_REG_WRITE32(VP0_XMEM_MASTER, 0);

	/* clean VP1 */
	VP1_REG_WRITE32(VP1_CFG_PMEM_MASTER, 0);
	VP1_REG_WRITE32(VP1_XD2MEM_MASTER, 0);
	VP1_REG_WRITE32(VP1_XD2MEMW_MASTER, 0);
	VP1_REG_WRITE32(VP1_XMEM_MASTER, 0);

	/* clean MEA */
	MEA_REG_WRITE32(MEA_CFG_PMEM_MASTER, 0);
	MEA_REG_WRITE32(MEA_XMEM_MASTER, 0);
	MEA_REG_WRITE32(MEA_XD2MEM_MASTER, 0);

	/* clean mmu */
	SET_MMU_PTD(
		psb_get_default_pd_addr(dev_priv->vsp_mmu) >> PAGE_TABLE_SHIFT);

	VSP_DEBUG("setup firmware\n");

	VSP_DEBUG("ppc_vp0_fw offset %lx\n", vsp_priv->ppc_vp0_fw->offset);
	SP1_DMEM_WRITE32(SP1_PPC_VP0_ADDR, vsp_priv->ppc_vp0_fw->offset);

	VSP_DEBUG("ppc_vp1_fw offset %lx\n", vsp_priv->ppc_vp1_fw->offset);
	SP1_DMEM_WRITE32(SP1_PPC_VP1_ADDR, vsp_priv->ppc_vp1_fw->offset);

	VSP_DEBUG("vpp_chain_vp0_fw offset %lx\n",
		  vsp_priv->vpp_chain_vp0_fw->offset);
	SP1_DMEM_WRITE32(SP1_VPP_CHAIN_VP0_ADDR,
			 vsp_priv->vpp_chain_vp0_fw->offset);

	VSP_DEBUG("vpp_chain_vp1_fw offset %lx\n",
		  vsp_priv->vpp_chain_vp1_fw->offset);
	SP1_DMEM_WRITE32(SP1_VPP_CHAIN_VP1_ADDR,
			 vsp_priv->vpp_chain_vp1_fw->offset);

	VSP_DEBUG("vpp_chain_vp1_fw offset %lx\n",
		  vsp_priv->vpp_chain_sp1_fw->offset);
	SP1_DMEM_WRITE32(SP1_VPP_CHAIN_SP1_ADDR,
			 vsp_priv->vpp_chain_sp1_fw->offset);

	VSP_DEBUG("memc_app_fw offset %lx\n",
		  vsp_priv->memc_app_fw->offset);
	SP1_DMEM_WRITE32(SP1_MEMC_APP_ADDR,
			 vsp_priv->memc_app_fw->offset);

	VSP_DEBUG("proc_fw offset %lx\n",
		  vsp_priv->proc_fw->offset);
	VSP_DEBUG("setup proc fw\n");

	/* CSIM section on VP */
	VSP_DEBUG("enter sp1 control mode\n");
	vsp_enter_start_mode(dev_priv);

	input = exe_prefix;
	while (*input != '\0') {
		SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, *input);
		input++;
	}
	input = exe_file_name;
	while (*input != '\0') {
		SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, *input);
		input++;
	}
	SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, '\0');

	/* invalide loop cache */
	SP1_REG_READ32(SP_LOOP_CACHE_INVALIDATE_REG, &val);
	VSP_SET_FLAG(val, SP_LOOP_CACHE_INVALIDATE_REG_INVALIDATE_FLAG);
	SP1_REG_WRITE32(SP_LOOP_CACHE_INVALIDATE_REG, val);
	udelay(100);
	VSP_CLEAR_FLAG(val, SP_LOOP_CACHE_INVALIDATE_REG_INVALIDATE_FLAG);
	SP1_REG_WRITE32(SP_LOOP_CACHE_INVALIDATE_REG, val);

	/* start sp */
	SP1_REG_WRITE32(SP_BASE_ADDR_REG, SP_MAIN_ENTRY);
	/* kick */
	SP1_REG_READ32(SP_STAT_AND_CTRL_REG, &val);
	VSP_SET_FLAG(val, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, val);

	SP1_REG_READ32(SP_STAT_AND_CTRL_REG, &val);
	VSP_SET_FLAG(val, SP_STAT_AND_CTRL_REG_START_FLAG);
	SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, val);

	VSP_DEBUG("check number of queues\n");
	SP1_DMEM_READ32(SP1_VSS_NUM_QUEUE_ADDR, &queue_num);
	VSP_DEBUG("queue number is %d\n", queue_num);
	BUG_ON(queue_num < 2);

	VSP_DEBUG("setup context, base %lx\n",
		  vsp_priv->context_buf->offset);
	SP1_DMEM_WRITE32(SP1_CONTEXT_BASE_ADDR,
			 vsp_priv->context_buf->offset);
	SP1_DMEM_WRITE32(SP1_CONTEXT_SIZE_ADDR,
			 vsp_priv->context_size);

	/* enable irq */
	psb_irq_preinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);
	psb_irq_postinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);

	SET_MMU_PTD(psb_get_default_pd_addr(dev_priv->vsp_mmu) >> PAGE_SHIFT);

	return 0;
}

void vsp_enter_start_mode(struct drm_psb_private *dev_priv)
{
	unsigned int val;

	VSP_DEBUG("enter start mode\n");
	val = 0;
	SP1_REG_READ32(SP_STAT_AND_CTRL_REG, &val);
	VSP_SET_FLAG(val, SP_STAT_AND_CTRL_REG_START_FLAG);
	VSP_CLEAR_FLAG(val, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	/* FIXME: program filename register for csim */
	SP1_REG_WRITE32(SP_STAT_AND_CTRL_REG, val);

	return;
}
