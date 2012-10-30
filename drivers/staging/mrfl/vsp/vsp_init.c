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

#define FW_SZ (800 * 1024)

#ifdef CONFIG_BOARD_MRFLD_VP
#define VSP_RUNNING_ON_VP
#define FW_NAME "vsp_VPP.bin"
#else
#define FW_NAME "vsp_VPP_sle.bin"
#endif

static inline void vsp_enter_start_mode(struct drm_psb_private *dev_priv,
					unsigned int processor);
static inline void vsp_leave_start_mode(struct drm_psb_private *dev_priv,
					unsigned int processor);
static inline void vsp_kick(struct drm_psb_private *dev_priv,
			    unsigned int processor);
static inline void vsp_start_func(struct drm_psb_private *dev_priv,
				  unsigned int pc, unsigned int processor);
static inline unsigned int vsp_set_firmware(struct drm_psb_private *dev_priv,
					    unsigned int processor);
static inline unsigned int vsp_set_firmware_vp(struct drm_psb_private *dev_priv,
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

	vsp_priv->fw_loaded = 0;
	vsp_priv->needs_reset = 1;
	vsp_priv->current_sequence = 0;
	vsp_priv->vsp_busy = 0;

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

	if (vsp_priv->ack_queue_bo)
		ttm_bo_unref(&vsp_priv->ack_queue_bo);
	if (vsp_priv->cmd_queue_bo)
		ttm_bo_unref(&vsp_priv->cmd_queue_bo);

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
	unsigned int mask, enable;

	VSP_DEBUG("will disable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);

	VSP_CLEAR_FLAG(mask, VSP_SP1_IRQ_SHIFT);
	VSP_CLEAR_FLAG(enable, VSP_SP1_IRQ_SHIFT);

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
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;
	int i;

	VSP_DEBUG("read firmware into buffer\n");

	/* read firmware img */
	ret = request_firmware(&raw, FW_NAME, &dev->pdev->dev);
	if (ret < 0) {
		DRM_ERROR("VSP: %s request_firmware failed: reason %d\n",
			  FW_NAME, ret);
		return -1;
	}

	if (raw->size < sizeof(struct vsp_config)) {
		DRM_ERROR("VSP: %s is not a correct firmware (size %d)\n",
			  FW_NAME, raw->size);
		ret = -1;
		goto out;
	}

	ptr = (void *)raw->data;
	config = (struct vsp_config *)ptr;
	/* get firmware configuration */
	memcpy(&vsp_priv->config, ptr, sizeof(vsp_priv->config));

	if (vsp_priv->config.magic_number != VSP_FIRMWARE_MAGIC_NUMBER) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->config.magic_number,
			  VSP_FIRMWARE_MAGIC_NUMBER);
		ret = -1;
		goto out;
	}

	VSP_DEBUG("firmware configuration:\n");
	VSP_DEBUG("magic number %x\n", config->magic_number);
	VSP_DEBUG("num_programs %d\n", config->num_programs);
	VSP_DEBUG("program offset:\n");
	for (i = 0; i < VSP_MAX_PROGRAMS; ++i)
		VSP_DEBUG("%x ", config->program_offset[i]);
	VSP_DEBUG("\n");
	VSP_DEBUG("program name offset:\n");
	for (i = 0; i < VSP_MAX_PROGRAMS; ++i)
		VSP_DEBUG("%x ", config->program_name_offset[i]);
	VSP_DEBUG("\n");
	VSP_DEBUG("programe name:\n");
	for (i = 0; i < 256; ++i)
		VSP_DEBUG("%c", config->string_table[i]);
	VSP_DEBUG("\n");
	VSP_DEBUG("boot processor %x\n", config->boot_processor);
	VSP_DEBUG("api processor %x\n", config->api_processor);
	VSP_DEBUG("boot program info:\n");
	VSP_DEBUG("text_src %x\n", config->text_src);
	VSP_DEBUG("data_src %x\n", config->data_src);
	VSP_DEBUG("data_dst %x\n", config->data_dst);
	VSP_DEBUG("data_size %x\n", config->data_size);
	VSP_DEBUG("bss_dst %x\n", config->bss_dst);
	VSP_DEBUG("bss_size %x\n", config->bss_size);
	VSP_DEBUG("init_addr %x\n", config->init_addr);
	VSP_DEBUG("main_addr %x\n", config->main_addr);

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
	VSP_DEBUG("vsp_config size %d, program_offset %d\n",
		  sizeof(struct vsp_config),
		  vsp_priv->config.program_offset[0]);
	memcpy(bo_ptr, ptr, raw->size);

	ttm_bo_kunmap(&tmp_kmap);

	vsp_priv->fw_loaded = 1;
	vsp_priv->needs_reset = 1;

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

	vsp_priv->needs_reset = 0;

	return ret;
}

int vsp_setup_fw(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret;

	VSP_DEBUG("clean HW before firmware uploaded\n");

	SET_MMU_PTD(
		psb_get_default_pd_addr(dev_priv->vsp_mmu) >> PAGE_TABLE_SHIFT);
	SET_MMU_PTD(psb_get_default_pd_addr(dev_priv->vsp_mmu) >> PAGE_SHIFT);

	/* set cmd/ack queue address */
	vsp_priv->ctrl->cmd_queue_addr = vsp_priv->cmd_queue_bo->offset;
	vsp_priv->ctrl->ack_queue_addr = vsp_priv->ack_queue_bo->offset;

	/* init queue */
	vsp_priv->ctrl->cmd_rd = 0;
	vsp_priv->ctrl->cmd_wr = 0;
	vsp_priv->ctrl->ack_rd = 0;
	vsp_priv->ctrl->ack_wr = 0;

	/* init context */
	vsp_priv->ctrl->context_init_req = 0;
	vsp_priv->ctrl->context_init_ack = 0;
	vsp_priv->ctrl->context_uninit_req = 0;
	vsp_priv->ctrl->context_uninit_ack = 0;

	VSP_DEBUG("setup firmware\n");
#ifdef VSP_RUNNING_ON_VP
	ret = vsp_set_firmware_vp(dev_priv, vsp_priv->config.boot_processor);
#else
	ret = vsp_set_firmware(dev_priv, vsp_priv->config.boot_processor);
#endif
	if (ret != 0) {
		DRM_ERROR("VSP: failed to set firmware register\n");
		return -1;
	}

	vsp_priv->ctrl->entry_kind = vsp_entry_init;

	/* init fw */
	vsp_start_func(dev_priv, vsp_priv->config.init_addr,
		       vsp_priv->config.boot_processor);
	vsp_kick(dev_priv, vsp_priv->config.boot_processor);

	/* wait it finished */
	do {
		udelay(10);
	} while (!vsp_is_idle(dev_priv, vsp_priv->config.boot_processor));

	/* enable irq */
	psb_irq_preinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);
	psb_irq_postinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);

	return 0;
}

void vsp_enter_start_mode(struct drm_psb_private *dev_priv,
			  unsigned int processor)
{
	unsigned int val;

	VSP_DEBUG("enter start mode\n");
	val = 0;
	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &val, processor);
	VSP_SET_FLAG(val, SP_STAT_AND_CTRL_REG_START_FLAG);
	VSP_CLEAR_FLAG(val, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	/* FIXME: program filename register for csim */
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, val, processor);

	return;
}

void vsp_leave_start_mode(struct drm_psb_private *dev_priv,
			  unsigned int processor)
{
	unsigned int val;

	VSP_DEBUG("leave start mode\n");
	val = 0;
	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &val, processor);
	VSP_CLEAR_FLAG(val, SP_STAT_AND_CTRL_REG_START_FLAG);
	VSP_CLEAR_FLAG(val, SP_STAT_AND_CTRL_REG_RUN_FLAG);

	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, val, processor);

	return;
}

void vsp_kick(struct drm_psb_private *dev_priv,
	      unsigned int processor)
{
	unsigned int reg;

	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &reg, processor);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_START_FLAG);
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);

	return;
}

void vsp_start_func(struct drm_psb_private *dev_priv, unsigned int pc,
		    unsigned int processor)
{
	SP_REG_WRITE32(SP_BASE_ADDR_REG, pc, processor);
	return;
}

unsigned int vsp_set_firmware_vp(struct drm_psb_private *dev_priv,
				 unsigned int processor)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	char *exe_prefix = "csim_executable=";
	char *input;

	VSP_DEBUG("set firmware address in config reg 2\n");
	vsp_priv->ctrl->firmware_addr = vsp_priv->firmware->offset;

	VSP_DEBUG("enter special mode\n");
	/* CSIM section on VP */
	VSP_DEBUG("enter sp1 control mode\n");

	vsp_enter_start_mode(dev_priv, processor);

	input = exe_prefix;
	while (*input != '\0') {
		SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, *input, processor);
		input++;
	}
	input = vsp_priv->config.string_table;
	while (*input != '\0') {
		SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, *input, processor);
		input++;
	}
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, '\0', processor);

	vsp_leave_start_mode(dev_priv, processor);

	return 0;
}

unsigned int vsp_set_firmware(struct drm_psb_private *dev_priv,
			      unsigned int processor)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	/* enable icache */
	vsp_config_icache(dev_priv, processor);

	/* set icache base address */
	SP_REG_WRITE32(SP_CFG_PMEM_MASTER,
		       vsp_priv->firmware->offset +
		       vsp_priv->config.program_offset[0] +
		       vsp_priv->config.text_src,
		       processor);

	vsp_priv->ctrl->firmware_addr = vsp_priv->firmware->offset;

	return 0;
}
