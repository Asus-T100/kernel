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

#define FW_SZ (2 * 800 * 1024)

#define FW_NAME "vsp_VPP_sle.bin"
#define FW_VP8_NAME "vsp_vp8_enc.bin"

static inline unsigned int vsp_set_firmware(struct drm_psb_private *dev_priv,
					    unsigned int processor);

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
	bool is_iomem;
	int ret;
	unsigned int context_size;

	VSP_DEBUG("init vsp private data structure\n");
	vsp_priv = kmalloc(sizeof(struct vsp_private), GFP_KERNEL);
	if (vsp_priv == NULL)
		return -1;

	memset(vsp_priv, 0, sizeof(*vsp_priv));

	/* get device --> drm_device --> drm_psb_private --> vsp_priv
	 * for psb_vsp_pmstate_show: vsp_pmpolicy
	 * if not pci_set_drvdata, can't get drm_device from device
	 */
	/* pci_set_drvdata(dev->pdev, dev); */
	if (device_create_file(&dev->pdev->dev,
			       &dev_attr_vsp_pmstate))
		DRM_ERROR("TOPAZ: could not create sysfs file\n");

	vsp_priv->sysfs_pmstate = sysfs_get_dirent(
		dev->pdev->dev.kobj.sd, NULL,
		"vsp_pmstate");

	vsp_priv->vsp_cmd_num = 0;
	vsp_priv->fw_loaded = 0;
	vsp_priv->current_sequence = 0;
	vsp_priv->vsp_state = VSP_STATE_DOWN;
	vsp_priv->dev = dev;

	dev_priv->vsp_private = vsp_priv;

	VSP_DEBUG("allocate buffer for fw\n");
	/* FIXME: assume 1 page, will modify to a proper value */
	vsp_priv->firmware_sz = FW_SZ;
	ret = ttm_buffer_object_create(bdev, vsp_priv->firmware_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->firmware);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP buffer for firmware\n");
		goto out_clean;
	}

	vsp_priv->cmd_queue_sz = VSP_CMD_QUEUE_SIZE *
		sizeof(struct vss_command_t);
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->cmd_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->cmd_queue_bo);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP cmd queue\n");
		goto out_clean;
	}

	vsp_priv->ack_queue_sz = VSP_ACK_QUEUE_SIZE *
		sizeof(struct vss_response_t);
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->ack_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->ack_queue_bo);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP cmd ack queue\n");
		goto out_clean;
	}

	/* Create setting buffer */
	ret =  ttm_buffer_object_create(bdev,
				       sizeof(struct vsp_settings_t),
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->setting_bo);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP setting buffer\n");
		goto out_clean;
	}

	/* Create context buffer */
	context_size = VSP_CONTEXT_NUM_MAX *
			sizeof(struct vsp_context_settings_t);
	ret =  ttm_buffer_object_create(bdev,
				       context_size,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &vsp_priv->context_setting_bo);
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate context setting buffer\n");
		goto out_clean;
	}

	/* map cmd queue */
	ret = ttm_bo_kmap(vsp_priv->cmd_queue_bo, 0,
			  vsp_priv->cmd_queue_bo->num_pages,
			  &vsp_priv->cmd_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->cmd_queue_bo);
		ttm_bo_kunmap(&vsp_priv->cmd_kmap);
		goto out_clean;
	}

	vsp_priv->cmd_queue = ttm_kmap_obj_virtual(&vsp_priv->cmd_kmap,
						   &is_iomem);


	/* map ack queue */
	ret = ttm_bo_kmap(vsp_priv->ack_queue_bo, 0,
			  vsp_priv->ack_queue_bo->num_pages,
			  &vsp_priv->ack_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->ack_queue_bo);
		ttm_bo_kunmap(&vsp_priv->ack_kmap);
		goto out_clean;
	}

	vsp_priv->ack_queue = ttm_kmap_obj_virtual(&vsp_priv->ack_kmap,
						   &is_iomem);

	/* map vsp setting */
	ret = ttm_bo_kmap(vsp_priv->setting_bo, 0,
			  vsp_priv->setting_bo->num_pages,
			  &vsp_priv->setting_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap setting_bo failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->setting_bo);
		ttm_bo_kunmap(&vsp_priv->setting_kmap);
		goto out_clean;
	}
	vsp_priv->setting = ttm_kmap_obj_virtual(&vsp_priv->setting_kmap,
						 &is_iomem);

	/* map vsp context setting */
	ret = ttm_bo_kmap(vsp_priv->context_setting_bo, 0,
			  vsp_priv->context_setting_bo->num_pages,
			  &vsp_priv->context_setting_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap context_setting_bo failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->context_setting_bo);
		ttm_bo_kunmap(&vsp_priv->context_setting_kmap);
		goto out_clean;
	}
	vsp_priv->context_setting = ttm_kmap_obj_virtual(
						&vsp_priv->context_setting_kmap,
						&is_iomem);

	spin_lock_init(&vsp_priv->lock);
	mutex_init(&vsp_priv->vsp_mutex);

	INIT_DELAYED_WORK(&vsp_priv->vsp_suspend_wq,
			&psb_powerdown_vsp);

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
	if (vsp_priv->firmware)
		ttm_bo_unref(&vsp_priv->firmware);

	if (vsp_priv->cmd_queue) {
		ttm_bo_kunmap(&vsp_priv->cmd_kmap);
		vsp_priv->cmd_queue = NULL;
	}

	if (vsp_priv->ack_queue) {
		ttm_bo_kunmap(&vsp_priv->ack_kmap);
		vsp_priv->ack_queue = NULL;
	}
	if (vsp_priv->setting) {
		ttm_bo_kunmap(&vsp_priv->setting);
		vsp_priv->setting = NULL;
	}

	if (vsp_priv->context_setting) {
		ttm_bo_kunmap(&vsp_priv->context_setting);
		vsp_priv->context_setting = NULL;
	}

	if (vsp_priv->ack_queue_bo)
		ttm_bo_unref(&vsp_priv->ack_queue_bo);
	if (vsp_priv->cmd_queue_bo)
		ttm_bo_unref(&vsp_priv->cmd_queue_bo);
	if (vsp_priv->setting_bo)
		ttm_bo_unref(&vsp_priv->setting_bo);
	if (vsp_priv->context_setting_bo)
		ttm_bo_unref(&vsp_priv->context_setting_bo);


	device_remove_file(&dev->pdev->dev, &dev_attr_vsp_pmstate);
	sysfs_put(vsp_priv->sysfs_pmstate);

	VSP_DEBUG("free VSP private structure\n");
	kfree(dev_priv->vsp_private);

	return 0;
}

void vsp_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned int mask;
	unsigned int enable;
	unsigned int clear;

	VSP_DEBUG("will enable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);
	clear = 0;

	VSP_SET_FLAG(mask, VSP_SP0_IRQ_SHIFT);
	VSP_SET_FLAG(enable, VSP_SP0_IRQ_SHIFT);
	VSP_SET_FLAG(clear, VSP_SP0_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_EDGE, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_CLR, clear);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
}

void vsp_disableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned int mask, enable;

	VSP_DEBUG("will disable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);

	VSP_CLEAR_FLAG(mask, VSP_SP0_IRQ_SHIFT);
	VSP_CLEAR_FLAG(enable, VSP_SP0_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
	return;
}

int vsp_init_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	int ret;
	const struct firmware *raw;
	unsigned char *ptr, *bo_ptr;
	struct vsp_config *config;
	struct vsp_secure_boot_header *boot_header;
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;
	int i;

	PSB_DEBUG_GENERAL("read firmware into buffer\n");

	/* read firmware img */
	if (vsp_priv->fw_type == VSP_FW_TYPE_VP8) {
		VSP_DEBUG("load vp8 fw\n");
		ret = request_firmware(&raw, FW_VP8_NAME, &dev->pdev->dev);
	} else if (vsp_priv->fw_type == VSP_FW_TYPE_VPP) {
		VSP_DEBUG("load vpp fw\n");
		ret = request_firmware(&raw, FW_NAME, &dev->pdev->dev);
	} else {
		DRM_ERROR("Don't support this fw type=%d!\n",
			vsp_priv->fw_type);
		ret = -1;
	}

	if (ret < 0) {
		DRM_ERROR("VSP: request_firmware failed: reason %d\n", ret);
		return -1;
	}

	if (raw->size < sizeof(struct vsp_secure_boot_header)) {
		DRM_ERROR("VSP: %s is not a correct firmware (size %d)\n",
				FW_NAME, raw->size);
		ret = -1;
		goto out;
	}

	ptr = (void *)raw->data;
	boot_header = (struct vsp_secure_boot_header *)ptr;
	/* get firmware header */
	memcpy(&vsp_priv->boot_header, ptr, sizeof(vsp_priv->boot_header));

	if (vsp_priv->boot_header.magic_number != VSP_SECURE_BOOT_MAGIC_NR) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->boot_header.magic_number,
			  VSP_SECURE_BOOT_MAGIC_NR);
		ret = -1;
		goto out;
	}

	VSP_DEBUG("firmware secure header:\n");
	VSP_DEBUG("magic number %x\n", boot_header->magic_number);
	VSP_DEBUG("boot_text_offset %x\n", boot_header->boot_text_offset);
	VSP_DEBUG("boot_text_reg %x\n", boot_header->boot_text_reg);
	VSP_DEBUG("boot_icache_value %x\n", boot_header->boot_icache_value);
	VSP_DEBUG("boot_icache_reg %x\n", boot_header->boot_icache_reg);
	VSP_DEBUG("boot_pc_value %x\n", boot_header->boot_pc_value);
	VSP_DEBUG("boot_pc_reg %x\n", boot_header->boot_pc_reg);
	VSP_DEBUG("ma_header_offset %x\n", boot_header->ma_header_offset);
	VSP_DEBUG("ma_header_reg %x\n", boot_header->ma_header_reg);
	VSP_DEBUG("boot_start_value %x\n", boot_header->boot_start_value);
	VSP_DEBUG("boot_start_reg %x\n", boot_header->boot_start_reg);

	/* load firmware to dmem */
	if (raw->size > vsp_priv->firmware_sz) {
		if (vsp_priv->firmware)
			ttm_bo_unref(&vsp_priv->firmware);

		VSP_DEBUG("allocate a new bo from size %d to size %d\n",
			  vsp_priv->firmware_sz, raw->size);

		vsp_priv->firmware_sz = raw->size;
		ret = ttm_buffer_object_create(bdev, vsp_priv->firmware_sz,
					       ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU |
					       TTM_PL_FLAG_NO_EVICT,
					       0, 0, 0, NULL,
					       &vsp_priv->firmware);
		if (ret != 0) {
			DRM_ERROR("VSP: failed to allocate firmware buffer\n");
			return -1;
			goto out;
		}
	}

	ret = ttm_bo_kmap(vsp_priv->firmware, 0,
			  vsp_priv->firmware->num_pages, &tmp_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->firmware);
		ttm_bo_kunmap(&tmp_kmap);
		ret = -1;
		goto out;
	}

	bo_ptr = ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem);
	memcpy(bo_ptr, ptr, raw->size);

	ttm_bo_kunmap(&tmp_kmap);

	vsp_priv->fw_loaded = 1;
	vsp_priv->vsp_state = VSP_STATE_DOWN;

	vsp_priv->ctrl = (struct vsp_ctrl_reg *) (dev_priv->vsp_reg +
						  VSP_CONFIG_REG_SDRAM_BASE +
						  VSP_CONFIG_REG_START);
out:
	release_firmware(raw);

	return ret;
}

int vsp_reset(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret;

	ret = vsp_setup_fw(dev_priv);

	return ret;
}

int vsp_setup_fw(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret;
	uint32_t pd_addr;

	/* set MMU */
	pd_addr = psb_get_default_pd_addr(dev_priv->vsp_mmu);
	SET_MMU_PTD(pd_addr >> PAGE_TABLE_SHIFT);
	SET_MMU_PTD(pd_addr >> PAGE_SHIFT);

	/* vsp setting */
	vsp_priv->setting->command_queue_size = VSP_CMD_QUEUE_SIZE;
	vsp_priv->setting->command_queue_addr = vsp_priv->cmd_queue_bo->offset;
	vsp_priv->setting->response_queue_size = VSP_ACK_QUEUE_SIZE;
	vsp_priv->setting->response_queue_addr = vsp_priv->ack_queue_bo->offset;

	vsp_priv->setting->max_contexts = 1;
	vsp_priv->setting->contexts_array_addr =
				vsp_priv->context_setting_bo->offset;

	vsp_priv->ctrl->setting_addr = vsp_priv->setting_bo->offset;

	vsp_priv->ctrl->cmd_rd = 0;
	vsp_priv->ctrl->cmd_wr = 0;
	vsp_priv->ctrl->ack_rd = 0;
	vsp_priv->ctrl->ack_wr = 0;

	VSP_DEBUG("setup firmware\n");
	vsp_set_firmware(dev_priv, vsp_vp0);

	/* Set power-saving mode */
	if (drm_vsp_pmpolicy == PSB_PMPOLICY_NOPM)
		vsp_priv->ctrl->power_saving_mode = vsp_always_on;
	else
		vsp_priv->ctrl->power_saving_mode = vsp_suspend_on_empty_queue;

	/* communicate the type of init
	 * this is the last value to write
	 * it will cause the VSP to read all other settings as wll
	 */
	vsp_priv->ctrl->entry_kind = vsp_entry_init;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;

	/* enable irq */
	psb_irq_preinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);
	psb_irq_postinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);

	return 0;
}

void vsp_start_function(struct drm_psb_private *dev_priv, unsigned int pc,
		    unsigned int processor)
{
	unsigned int reg;

	/* set the start addr */
	SP_REG_WRITE32(VSP_START_PC_REG_OFFSET, pc, processor);

	/* set start command */
	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &reg, processor);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_START_FLAG);
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);
	return;
}

unsigned int vsp_set_firmware(struct drm_psb_private *dev_priv,
			      unsigned int processor)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int reg = 0;

	/* config icache */
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_INVALID_FLAG);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_PREFETCH_FLAG);
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);

	/* set icache base address: point to instructions in DDR */
	SP_REG_WRITE32(VSP_ICACHE_BASE_REG_OFFSET,
		       vsp_priv->firmware->offset +
		       vsp_priv->boot_header.boot_text_offset,
		       processor);

	/* write ma_header_address to the variable allocated for it*/
	MM_WRITE32(vsp_priv->boot_header.ma_header_reg,
		   0,
		   vsp_priv->firmware->offset +
		   vsp_priv->boot_header.ma_header_offset);

	/* start the secure boot program */
	vsp_start_function(dev_priv,
			   vsp_priv->boot_header.boot_pc_value,
			   processor);

	return 0;
}

void vsp_init_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	vsp_priv->ctrl->entry_kind = vsp_entry_init;
}

void vsp_continue_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int i;

	vsp_set_firmware(dev_priv, vsp_vp0);

	vsp_priv->ctrl->entry_kind = vsp_entry_resume;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;
}

void vsp_resume_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	uint32_t pd_addr;

	vsp_priv->ctrl = (struct vsp_ctrl_reg *) (dev_priv->vsp_reg +
						  VSP_CONFIG_REG_SDRAM_BASE +
						  VSP_CONFIG_REG_START);

	/* Set MMU */
	pd_addr = psb_get_default_pd_addr(dev_priv->vsp_mmu);
	SET_MMU_PTD(pd_addr >> PAGE_TABLE_SHIFT);
	SET_MMU_PTD(pd_addr >> PAGE_SHIFT);

	/* restore the config regs */
	CONFIG_REG_WRITE32(VSP_SETTING_ADDR_REG,
			vsp_priv->saved_config_regs[VSP_SETTING_ADDR_REG]);
	CONFIG_REG_WRITE32(VSP_POWER_SAVING_MODE_REG,
			vsp_priv->saved_config_regs[VSP_POWER_SAVING_MODE_REG]);
	CONFIG_REG_WRITE32(VSP_CMD_QUEUE_RD_REG,
			vsp_priv->saved_config_regs[VSP_CMD_QUEUE_RD_REG]);
	CONFIG_REG_WRITE32(VSP_CMD_QUEUE_WR_REG,
			vsp_priv->saved_config_regs[VSP_CMD_QUEUE_WR_REG]);
	CONFIG_REG_WRITE32(VSP_ACK_QUEUE_RD_REG,
			vsp_priv->saved_config_regs[VSP_ACK_QUEUE_RD_REG]);
	CONFIG_REG_WRITE32(VSP_ACK_QUEUE_WR_REG,
			vsp_priv->saved_config_regs[VSP_ACK_QUEUE_WR_REG]);

	/* setup firmware */
	vsp_set_firmware(dev_priv, vsp_vp0);

	vsp_priv->ctrl->entry_kind = vsp_entry_resume;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;
}

