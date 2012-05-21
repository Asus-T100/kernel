/**
 * file lnc_topazinit.c
 * TOPAZ initialization and mtx-firmware upload
 *
 */

/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
 * Copyright (c) Imagination Technologies Limited, UK
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

/* NOTE: (READ BEFORE REFINE CODE)
 * 1. The FIRMWARE's SIZE is measured by byte, we have to pass the size
 * measured by word to DMAC.
 *
 *
 *
 */

/* include headers */

/* #define DRM_DEBUG_CODE 2 */

#include <linux/firmware.h>

#include <drm/drmP.h>
#include <drm/drm.h>

#include "psb_drv.h"
#include "lnc_topaz.h"
#include "lnc_topaz_hw_reg.h"
#include "psb_powermgmt.h"

/* WARNING: this define is very important */
#define RAM_SIZE (1024 * 24)

/* register default values
 * THIS HEADER IS ONLY INCLUDE ONCE*/
static unsigned long topaz_default_regs[183][3] = {
	{MVEA_START, 0x00000000, 0x00000000},
	{MVEA_START, 0x00000004, 0x00000400},
	{MVEA_START, 0x00000008, 0x00000000},
	{MVEA_START, 0x0000000C, 0x00000000},
	{MVEA_START, 0x00000010, 0x00000000},
	{MVEA_START, 0x00000014, 0x00000000},
	{MVEA_START, 0x00000018, 0x00000000},
	{MVEA_START, 0x0000001C, 0x00000000},
	{MVEA_START, 0x00000020, 0x00000120},
	{MVEA_START, 0x00000024, 0x00000000},
	{MVEA_START, 0x00000028, 0x00000000},
	{MVEA_START, 0x00000100, 0x00000000},
	{MVEA_START, 0x00000104, 0x00000000},
	{MVEA_START, 0x00000108, 0x00000000},
	{MVEA_START, 0x0000010C, 0x00000000},
	{MVEA_START, 0x0000011C, 0x00000001},
	{MVEA_START, 0x0000012C, 0x00000000},
	{MVEA_START, 0x00000180, 0x00000000},
	{MVEA_START, 0x00000184, 0x00000000},
	{MVEA_START, 0x00000188, 0x00000000},
	{MVEA_START, 0x0000018C, 0x00000000},
	{MVEA_START, 0x00000190, 0x00000000},
	{MVEA_START, 0x00000194, 0x00000000},
	{MVEA_START, 0x00000198, 0x00000000},
	{MVEA_START, 0x0000019C, 0x00000000},
	{MVEA_START, 0x000001A0, 0x00000000},
	{MVEA_START, 0x000001A4, 0x00000000},
	{MVEA_START, 0x000001A8, 0x00000000},
	{MVEA_START, 0x000001AC, 0x00000000},
	{MVEA_START, 0x000001B0, 0x00000000},
	{MVEA_START, 0x000001B4, 0x00000000},
	{MVEA_START, 0x000001B8, 0x00000000},
	{MVEA_START, 0x000001BC, 0x00000000},
	{MVEA_START, 0x000001F8, 0x00000000},
	{MVEA_START, 0x000001FC, 0x00000000},
	{MVEA_START, 0x00000200, 0x00000000},
	{MVEA_START, 0x00000204, 0x00000000},
	{MVEA_START, 0x00000208, 0x00000000},
	{MVEA_START, 0x0000020C, 0x00000000},
	{MVEA_START, 0x00000210, 0x00000000},
	{MVEA_START, 0x00000220, 0x00000001},
	{MVEA_START, 0x00000224, 0x0000001F},
	{MVEA_START, 0x00000228, 0x00000100},
	{MVEA_START, 0x0000022C, 0x00001F00},
	{MVEA_START, 0x00000230, 0x00000101},
	{MVEA_START, 0x00000234, 0x00001F1F},
	{MVEA_START, 0x00000238, 0x00001F01},
	{MVEA_START, 0x0000023C, 0x0000011F},
	{MVEA_START, 0x00000240, 0x00000200},
	{MVEA_START, 0x00000244, 0x00001E00},
	{MVEA_START, 0x00000248, 0x00000002},
	{MVEA_START, 0x0000024C, 0x0000001E},
	{MVEA_START, 0x00000250, 0x00000003},
	{MVEA_START, 0x00000254, 0x0000001D},
	{MVEA_START, 0x00000258, 0x00001F02},
	{MVEA_START, 0x0000025C, 0x00000102},
	{MVEA_START, 0x00000260, 0x0000011E},
	{MVEA_START, 0x00000264, 0x00000000},
	{MVEA_START, 0x00000268, 0x00000000},
	{MVEA_START, 0x0000026C, 0x00000000},
	{MVEA_START, 0x00000270, 0x00000000},
	{MVEA_START, 0x00000274, 0x00000000},
	{MVEA_START, 0x00000278, 0x00000000},
	{MVEA_START, 0x00000280, 0x00008000},
	{MVEA_START, 0x00000284, 0x00000000},
	{MVEA_START, 0x00000288, 0x00000000},
	{MVEA_START, 0x0000028C, 0x00000000},
	{MVEA_START, 0x00000314, 0x00000000},
	{MVEA_START, 0x00000318, 0x00000000},
	{MVEA_START, 0x0000031C, 0x00000000},
	{MVEA_START, 0x00000320, 0x00000000},
	{MVEA_START, 0x00000324, 0x00000000},
	{MVEA_START, 0x00000348, 0x00000000},
	{MVEA_START, 0x00000380, 0x00000000},
	{MVEA_START, 0x00000384, 0x00000000},
	{MVEA_START, 0x00000388, 0x00000000},
	{MVEA_START, 0x0000038C, 0x00000000},
	{MVEA_START, 0x00000390, 0x00000000},
	{MVEA_START, 0x00000394, 0x00000000},
	{MVEA_START, 0x00000398, 0x00000000},
	{MVEA_START, 0x0000039C, 0x00000000},
	{MVEA_START, 0x000003A0, 0x00000000},
	{MVEA_START, 0x000003A4, 0x00000000},
	{MVEA_START, 0x000003A8, 0x00000000},
	{MVEA_START, 0x000003B0, 0x00000000},
	{MVEA_START, 0x000003B4, 0x00000000},
	{MVEA_START, 0x000003B8, 0x00000000},
	{MVEA_START, 0x000003BC, 0x00000000},
	{MVEA_START, 0x000003D4, 0x00000000},
	{MVEA_START, 0x000003D8, 0x00000000},
	{MVEA_START, 0x000003DC, 0x00000000},
	{MVEA_START, 0x000003E0, 0x00000000},
	{MVEA_START, 0x000003E4, 0x00000000},
	{MVEA_START, 0x000003EC, 0x00000000},
	{MVEA_START, 0x000002D0, 0x00000000},
	{MVEA_START, 0x000002D4, 0x00000000},
	{MVEA_START, 0x000002D8, 0x00000000},
	{MVEA_START, 0x000002DC, 0x00000000},
	{MVEA_START, 0x000002E0, 0x00000000},
	{MVEA_START, 0x000002E4, 0x00000000},
	{MVEA_START, 0x000002E8, 0x00000000},
	{MVEA_START, 0x000002EC, 0x00000000},
	{MVEA_START, 0x000002F0, 0x00000000},
	{MVEA_START, 0x000002F4, 0x00000000},
	{MVEA_START, 0x000002F8, 0x00000000},
	{MVEA_START, 0x000002FC, 0x00000000},
	{MVEA_START, 0x00000300, 0x00000000},
	{MVEA_START, 0x00000304, 0x00000000},
	{MVEA_START, 0x00000308, 0x00000000},
	{MVEA_START, 0x0000030C, 0x00000000},
	{MVEA_START, 0x00000290, 0x00000000},
	{MVEA_START, 0x00000294, 0x00000000},
	{MVEA_START, 0x00000298, 0x00000000},
	{MVEA_START, 0x0000029C, 0x00000000},
	{MVEA_START, 0x000002A0, 0x00000000},
	{MVEA_START, 0x000002A4, 0x00000000},
	{MVEA_START, 0x000002A8, 0x00000000},
	{MVEA_START, 0x000002AC, 0x00000000},
	{MVEA_START, 0x000002B0, 0x00000000},
	{MVEA_START, 0x000002B4, 0x00000000},
	{MVEA_START, 0x000002B8, 0x00000000},
	{MVEA_START, 0x000002BC, 0x00000000},
	{MVEA_START, 0x000002C0, 0x00000000},
	{MVEA_START, 0x000002C4, 0x00000000},
	{MVEA_START, 0x000002C8, 0x00000000},
	{MVEA_START, 0x000002CC, 0x00000000},
	{MVEA_START, 0x00000080, 0x00000000},
	{MVEA_START, 0x00000084, 0x80705700},
	{MVEA_START, 0x00000088, 0x00000000},
	{MVEA_START, 0x0000008C, 0x00000000},
	{MVEA_START, 0x00000090, 0x00000000},
	{MVEA_START, 0x00000094, 0x00000000},
	{MVEA_START, 0x00000098, 0x00000000},
	{MVEA_START, 0x0000009C, 0x00000000},
	{MVEA_START, 0x000000A0, 0x00000000},
	{MVEA_START, 0x000000A4, 0x00000000},
	{MVEA_START, 0x000000A8, 0x00000000},
	{MVEA_START, 0x000000AC, 0x00000000},
	{MVEA_START, 0x000000B0, 0x00000000},
	{MVEA_START, 0x000000B4, 0x00000000},
	{MVEA_START, 0x000000B8, 0x00000000},
	{MVEA_START, 0x000000BC, 0x00000000},
	{MVEA_START, 0x000000C0, 0x00000000},
	{MVEA_START, 0x000000C4, 0x00000000},
	{MVEA_START, 0x000000C8, 0x00000000},
	{MVEA_START, 0x000000CC, 0x00000000},
	{MVEA_START, 0x000000D0, 0x00000000},
	{MVEA_START, 0x000000D4, 0x00000000},
	{MVEA_START, 0x000000D8, 0x00000000},
	{MVEA_START, 0x000000DC, 0x00000000},
	{MVEA_START, 0x000000E0, 0x00000000},
	{MVEA_START, 0x000000E4, 0x00000000},
	{MVEA_START, 0x000000E8, 0x00000000},
	{MVEA_START, 0x000000EC, 0x00000000},
	{MVEA_START, 0x000000F0, 0x00000000},
	{MVEA_START, 0x000000F4, 0x00000000},
	{MVEA_START, 0x000000F8, 0x00000000},
	{MVEA_START, 0x000000FC, 0x00000000},
	{TOPAZ_VLC_START, 0x00000000, 0x00000000},
	{TOPAZ_VLC_START, 0x00000004, 0x00000000},
	{TOPAZ_VLC_START, 0x00000008, 0x00000000},
	{TOPAZ_VLC_START, 0x0000000C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000010, 0x00000000},
	{TOPAZ_VLC_START, 0x00000014, 0x00000000},
	{TOPAZ_VLC_START, 0x0000001C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000020, 0x00000000},
	{TOPAZ_VLC_START, 0x00000024, 0x00000000},
	{TOPAZ_VLC_START, 0x0000002C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000034, 0x00000000},
	{TOPAZ_VLC_START, 0x00000038, 0x00000000},
	{TOPAZ_VLC_START, 0x0000003C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000040, 0x00000000},
	{TOPAZ_VLC_START, 0x00000044, 0x00000000},
	{TOPAZ_VLC_START, 0x00000048, 0x00000000},
	{TOPAZ_VLC_START, 0x0000004C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000050, 0x00000000},
	{TOPAZ_VLC_START, 0x00000054, 0x00000000},
	{TOPAZ_VLC_START, 0x00000058, 0x00000000},
	{TOPAZ_VLC_START, 0x0000005C, 0x00000000},
	{TOPAZ_VLC_START, 0x00000060, 0x00000000},
	{TOPAZ_VLC_START, 0x00000064, 0x00000000},
	{TOPAZ_VLC_START, 0x00000068, 0x00000000},
	{TOPAZ_VLC_START, 0x0000006C, 0x00000000}
};

#define FIRMWARE_NAME "topaz_fw.bin"

/* static function define */
static int topaz_upload_fw(struct drm_device *dev,
			   enum drm_lnc_topaz_codec codec);
static inline void topaz_set_default_regs(struct drm_psb_private
					  *dev_priv);

#define UPLOAD_FW_BY_DMA 1

#if UPLOAD_FW_BY_DMA
static void topaz_dma_transfer(struct drm_psb_private *dev_priv,
			       uint32_t channel, uint32_t src_phy_addr,
			       uint32_t offset, uint32_t dst_addr,
			       uint32_t byte_num, uint32_t is_increment,
			       uint32_t is_write);
#else
static void topaz_mtx_upload_by_register(struct drm_device *dev,
					 uint32_t mtx_mem, uint32_t addr,
					 uint32_t size,
					 struct ttm_buffer_object *buf);
#endif

static void topaz_write_core_reg(struct drm_psb_private *dev_priv,
				 uint32_t reg, const uint32_t val);
static void topaz_read_core_reg(struct drm_psb_private *dev_priv,
				uint32_t reg, uint32_t *ret_val);
static void get_mtx_control_from_dash(struct drm_psb_private *dev_priv);
static void release_mtx_control_from_dash(struct drm_psb_private
					  *dev_priv);
static void topaz_mmu_hwsetup(struct drm_psb_private *dev_priv);
static void mtx_dma_read(struct drm_device *dev, uint32_t source_addr,
			 uint32_t size);
static void mtx_dma_write(struct drm_device *dev);

#define DEBUG_FUNCTION 0

#if  DEBUG_FUNCTION
static int topaz_test_null(struct drm_device *dev, uint32_t seq);
static int topaz_test_sync(struct drm_device *dev, uint32_t seq,
			   uint32_t sync_seq);
static void topaz_mmu_test(struct drm_device *dev, uint32_t sync_value);
static void topaz_save_default_regs(struct drm_psb_private *dev_priv,
				    uint32_t *data);
static void topaz_restore_default_regs(struct drm_psb_private *dev_priv,
				       uint32_t *data);
static int topaz_test_sync_manual_alloc_page(struct drm_device *dev,
					     uint32_t seq,
					     uint32_t sync_seq,
					     uint32_t offset);
static int topaz_test_sync_tt_test(struct drm_device *dev,
				   uint32_t seq, uint32_t sync_seq);
#endif

uint32_t topaz_read_mtx_mem(struct drm_psb_private *dev_priv,
			    uint32_t byte_addr)
{
	uint32_t read_val;
	uint32_t reg, bank_size, ram_bank_size, ram_id;

	TOPAZ_READ32(0x3c, &reg);
	reg = 0x0a0a0606;
	bank_size = (reg & 0xF0000) >> 16;

	ram_bank_size = (uint32_t) (1 << (bank_size + 2));
	ram_id = (byte_addr - MTX_DATA_MEM_BASE) / ram_bank_size;

	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_CONTROL,
		    F_ENCODE(0x18 + ram_id, MTX_MTX_MCMID) |
		    F_ENCODE(byte_addr >> 2, MTX_MTX_MCM_ADDR) |
		    F_ENCODE(1, MTX_MTX_MCMR));

	/* ?? poll this reg? */
	topaz_wait_for_register(dev_priv,
				MTX_START + MTX_CR_MTX_RAM_ACCESS_STATUS, 1, 1);

	MTX_READ32(MTX_CR_MTX_RAM_ACCESS_DATA_TRANSFER, &read_val);

	return read_val;
}

void topaz_write_mtx_mem(struct drm_psb_private *dev_priv,
			 uint32_t byte_addr, uint32_t val)
{
	uint32_t ram_id = 0;
	uint32_t reg, bank_size, ram_bank_size;

	TOPAZ_READ32(0x3c, &reg);

	/* PSB_DEBUG_GENERAL ("TOPAZ: DEBUG REG(%x)\n", reg); */
	reg = 0x0a0a0606;

	bank_size = (reg & 0xF0000) >> 16;

	ram_bank_size = (uint32_t) (1 << (bank_size + 2));
	ram_id = (byte_addr - MTX_DATA_MEM_BASE) / ram_bank_size;

	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_CONTROL,
		    F_ENCODE(0x18 + ram_id, MTX_MTX_MCMID) |
		    F_ENCODE(byte_addr >> 2, MTX_MTX_MCM_ADDR));

	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_DATA_TRANSFER, val);

	/* ?? poll this reg? */
	topaz_wait_for_register(dev_priv,
				MTX_START + MTX_CR_MTX_RAM_ACCESS_STATUS, 1, 1);

	return;
}

void topaz_write_mtx_mem_multiple_setup(struct drm_psb_private *dev_priv,
					uint32_t byte_addr)
{
	uint32_t ram_id = 0;
	uint32_t reg, bank_size, ram_bank_size;

	TOPAZ_READ32(0x3c, &reg);

	reg = 0x0a0a0606;

	bank_size = (reg & 0xF0000) >> 16;

	ram_bank_size = (uint32_t) (1 << (bank_size + 2));
	ram_id = (byte_addr - MTX_DATA_MEM_BASE) / ram_bank_size;

	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_CONTROL,
		    F_ENCODE(0x18 + ram_id, MTX_MTX_MCMID) |
		    F_ENCODE(1, MTX_MTX_MCMAI) |
		    F_ENCODE(byte_addr >> 2, MTX_MTX_MCM_ADDR));
}

void topaz_write_mtx_mem_multiple(struct drm_psb_private *dev_priv,
				  uint32_t val)
{
	MTX_WRITE32(MTX_CR_MTX_RAM_ACCESS_DATA_TRANSFER, val);
}

int topaz_wait_for_register(struct drm_psb_private *dev_priv,
			    uint32_t addr, uint32_t value, uint32_t mask)
{
	uint32_t tmp;
	uint32_t count = 10000;

	/* # poll topaz register for certain times */
	while (count) {
		/* #.# read */
		MM_READ32(addr, 0, &tmp);

		if (value == (tmp & mask))
			return 0;

		/* #.# delay and loop */
		DRM_UDELAY(100);
		--count;
	}

	/* # now waiting is timeout, return 1 indicat failed */
	/* XXX: testsuit means a timeout 10000 */

	DRM_ERROR("TOPAZ:time out to poll addr(0x%x) expected value(0x%08x), "
		  "actual 0x%08x (0x%08x & 0x%08x)\n",
		  addr, value, tmp & mask, tmp, mask);

	return -EBUSY;

}

static ssize_t psb_topaz_pmstate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct drm_psb_private *dev_priv;
	struct topaz_private *topaz_priv;
	unsigned int pmstate;
	unsigned long flags;
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	dev_priv = drm_dev->dev_private;
	topaz_priv = dev_priv->topaz_private;
	pmstate = topaz_priv->pmstate;

	pmstate = topaz_priv->pmstate;
	spin_lock_irqsave(&topaz_priv->topaz_lock, flags);
	ret = snprintf(buf, 32, "%s\n",
		       (pmstate == PSB_PMSTATE_POWERUP) ? "powerup"
		       : ((pmstate == PSB_PMSTATE_POWERDOWN) ? "powerdown"
			  : "clockgated"));
	spin_unlock_irqrestore(&topaz_priv->topaz_lock, flags);

	return ret;
}

static DEVICE_ATTR(topaz_pmstate, 0444, psb_topaz_pmstate_show, NULL);

/* this function finish the first part of initialization, the rest
 * should be done in topaz_setup_fw
 */
int lnc_topaz_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	uint32_t core_id, core_rev;
	int ret = 0, n;
	bool is_iomem;
	struct topaz_private *topaz_priv;
	void *topaz_bo_virt;

	PSB_DEBUG_GENERAL("TOPAZ: init topaz data structures\n");
	topaz_priv = kmalloc(sizeof(struct topaz_private), GFP_KERNEL);
	if (topaz_priv == NULL)
		return -1;

	dev_priv->topaz_private = topaz_priv;
	memset(topaz_priv, 0, sizeof(struct topaz_private));

	/* get device --> drm_device --> drm_psb_private --> topaz_priv
	 * for psb_topaz_pmstate_show: topaz_pmpolicy
	 * if not pci_set_drvdata, can't get drm_device from device
	 */
	pci_set_drvdata(dev->pdev, dev);
	if (device_create_file(&dev->pdev->dev, &dev_attr_topaz_pmstate))
		DRM_ERROR("TOPAZ: could not create sysfs file\n");
	topaz_priv->sysfs_pmstate = sysfs_get_dirent(dev->pdev->dev.kobj.sd,
						     NULL,
						     "topaz_pmstate");

	topaz_priv = dev_priv->topaz_private;

	/* # initialize comand topaz queueing [msvdx_queue] */
	INIT_LIST_HEAD(&topaz_priv->topaz_queue);
	/* # init mutex? CHECK: mutex usage [msvdx_mutex] */
	mutex_init(&topaz_priv->topaz_mutex);
	/* # spin lock init? CHECK spin lock usage [msvdx_lock] */
	spin_lock_init(&topaz_priv->topaz_lock);

	/* # topaz status init. [msvdx_busy] */
	topaz_priv->topaz_busy = 0;
	topaz_priv->topaz_cmd_seq = 0;
	topaz_priv->topaz_fw_loaded = 0;
	/* FIXME: workaround since JPEG firmware is not ready */
	topaz_priv->topaz_cur_codec = 1;
	topaz_priv->cur_mtx_data_size = 0;
	topaz_priv->topaz_hw_busy = 1;

	topaz_priv->topaz_mtx_reg_state = kmalloc(TOPAZ_MTX_REG_SIZE,
						  GFP_KERNEL);
	if (topaz_priv->topaz_mtx_reg_state == NULL) {
		DRM_ERROR("TOPAZ: failed to allocate space "
			  "for mtx register\n");
		return -1;
	}

	/* # gain write back structure,we may only need 32+4=40DW */
	ret = ttm_buffer_object_create(bdev, 4096,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT, 0, 0, 0, NULL,
				       &(topaz_priv->topaz_bo));
	if (ret != 0) {
		DRM_ERROR("TOPAZ: failed to allocate topaz BO.\n");
		return ret;
	}

	ret = ttm_bo_kmap(topaz_priv->topaz_bo, 0,
			  topaz_priv->topaz_bo->num_pages,
			  &topaz_priv->topaz_bo_kmap);
	if (ret) {
		DRM_ERROR("TOPAZ: map topaz BO bo failed......\n");
		ttm_bo_unref(&topaz_priv->topaz_bo);
		return ret;
	}

	topaz_bo_virt = ttm_kmap_obj_virtual(&topaz_priv->topaz_bo_kmap,
					     &is_iomem);
	topaz_priv->topaz_ccb_wb = (void *)topaz_bo_virt;
	topaz_priv->topaz_wb_offset = topaz_priv->topaz_bo->offset;
	topaz_priv->topaz_sync_addr = (uint32_t *) (topaz_bo_virt + 2048);
	topaz_priv->topaz_sync_offset = topaz_priv->topaz_wb_offset + 2048;
	PSB_DEBUG_GENERAL("TOPAZ: alloc BO for WriteBack and SYNC\n");
	PSB_DEBUG_GENERAL("TOPAZ: WB offset=0x%08x\n",
			  topaz_priv->topaz_wb_offset);
	PSB_DEBUG_GENERAL("TOPAZ: SYNC offset=0x%08x\n",
			  topaz_priv->topaz_sync_offset);

	*(topaz_priv->topaz_sync_addr) = ~0;	/* reset sync seq */

	/* # reset topaz */
	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	/* # set up MMU */
	topaz_mmu_hwsetup(dev_priv);

	PSB_DEBUG_GENERAL("TOPAZ: defer firmware loading to the place"
			  "when receiving user space commands\n");

#if 0				/* can't load FW here */
	/* #.# load fw to driver */
	PSB_DEBUG_GENERAL("TOPAZ: will init firmware\n");
	ret = topaz_init_fw(dev);
	if (ret != 0)
		return -1;

	topaz_setup_fw(dev, IMG_CODEC_MPEG4_NO_RC);	/* just for test */
#endif
	/* <msvdx does> # minimal clock */

	/* <msvdx does> # return 0 */
	TOPAZ_READ32(TOPAZ_CR_IMG_TOPAZ_CORE_ID, &core_id);
	TOPAZ_READ32(TOPAZ_CR_IMG_TOPAZ_CORE_REV, &core_rev);

	PSB_DEBUG_GENERAL("TOPAZ: core_id(%x) core_rev(%x)\n",
			  core_id, core_rev);

	/* create firmware storage */
	for (n = 1; n < IMG_CODEC_NUM; ++n) {
		/* #.# malloc DRM object for fw storage */
		ret = ttm_buffer_object_create(bdev, 12 * 4096,
					       ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU |
					       TTM_PL_FLAG_NO_EVICT, 0, 0, 0,
					       NULL,
					       &topaz_priv->topaz_fw[n].text);
		if (ret) {
			DRM_ERROR("Failed to allocate firmware.\n");
			goto out;
		}

		/* #.# malloc DRM object for fw storage */
		ret = ttm_buffer_object_create(bdev, 12 * 4096,
					       ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU |
					       TTM_PL_FLAG_NO_EVICT, 0, 0, 0,
					       NULL,
					       &topaz_priv->topaz_fw[n].data);
		if (ret) {
			DRM_ERROR("Failed to allocate firmware.\n");
			goto out;
		}
	}

	ret = ttm_buffer_object_create(bdev,
				       12 * 4096,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL,
				       &topaz_priv->topaz_mtx_data_mem);
	if (ret) {
		DRM_ERROR("TOPAZ: failed to allocate ttm buffer for "
			  "mtx data save\n");
		goto out;
	}
	topaz_priv->cur_mtx_data_size = 0;

	if (drm_psb_topaz_clockgating == 0) {
		PSB_DEBUG_INIT("TOPAZ:old clock gating disable = 0x%08x\n",
			       PSB_RVDC32(PSB_TOPAZ_CLOCKGATING));

		PSB_DEBUG_INIT("TOPAZ:reset to disable clock gating\n");

		PSB_WVDC32(0x00011fff, PSB_TOPAZ_CLOCKGATING);

		PSB_DEBUG_INIT("MSDVX:new clock gating disable = 0x%08x\n",
			       PSB_RVDC32(PSB_TOPAZ_CLOCKGATING));
	}

	return 0;

 out:
	for (n = 1; n < IMG_CODEC_NUM; ++n) {
		if (topaz_priv->topaz_fw[n].text != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].text);
		if (topaz_priv->topaz_fw[n].data != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].data);
	}

	if (topaz_priv->topaz_mtx_data_mem != NULL)
		ttm_bo_unref(&topaz_priv->topaz_mtx_data_mem);

	return ret;
}

int lnc_topaz_uninit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	int n;

	/* flush MMU */
	PSB_DEBUG_GENERAL("XXX: need to flush mmu cache here??\n");
	/* topaz_mmu_flushcache (dev_priv); */

	/* # reset TOPAZ chip */
	lnc_topaz_reset(dev_priv);

	if (NULL == topaz_priv) {
		DRM_ERROR("TOPAZ: topaz_priv is NULL!\n");
		return -1;
	}
	/* release resources */
	/* # release write back memory */
	topaz_priv->topaz_ccb_wb = NULL;

	/* release mtx register save space */
	kfree(topaz_priv->topaz_mtx_reg_state);

	/* release mtx data memory save space */
	if (topaz_priv->topaz_mtx_data_mem)
		ttm_bo_unref(&topaz_priv->topaz_mtx_data_mem);

	/* # release firmware storage */
	for (n = 1; n < IMG_CODEC_NUM; ++n) {
		if (topaz_priv->topaz_fw[n].text != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].text);
		if (topaz_priv->topaz_fw[n].data != NULL)
			ttm_bo_unref(&topaz_priv->topaz_fw[n].data);
	}

	ttm_bo_kunmap(&topaz_priv->topaz_bo_kmap);
	ttm_bo_unref(&topaz_priv->topaz_bo);

	if (topaz_priv) {
		pci_set_drvdata(dev->pdev, NULL);
		device_remove_file(&dev->pdev->dev, &dev_attr_topaz_pmstate);
		sysfs_put(topaz_priv->sysfs_pmstate);
		topaz_priv->sysfs_pmstate = NULL;

		kfree(topaz_priv);
		dev_priv->topaz_private = NULL;
	}

	return 0;
}

int lnc_topaz_reset(struct drm_psb_private *dev_priv)
{
	struct topaz_private *topaz_priv;

	topaz_priv = dev_priv->topaz_private;
	topaz_priv->topaz_busy = 0;
	topaz_priv->topaz_cmd_seq = 0;
	topaz_priv->cur_mtx_data_size = 0;
	topaz_priv->topaz_cmd_windex = 0;
	topaz_priv->topaz_needs_reset = 0;

	/* # reset topaz */
	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	/* # set up MMU */
	topaz_mmu_hwsetup(dev_priv);

	return 0;
}

/* read firmware bin file and load all data into driver */
int topaz_init_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	const struct firmware *raw = NULL;
	unsigned char *ptr;
	int ret = 0;
	int n;
	struct topaz_fwinfo *cur_fw;
	int cur_size;
	struct topaz_codec_fw *cur_codec;
	struct ttm_buffer_object **cur_drm_obj;
	struct ttm_bo_kmap_obj tmp_kmap;
	bool is_iomem;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	topaz_priv->stored_initial_qp = 0;

	/* # get firmware */
	ret = request_firmware(&raw, FIRMWARE_NAME, &dev->pdev->dev);
	if (ret != 0) {
		DRM_ERROR("TOPAZ: request_firmware failed: %d\n", ret);
		return ret;
	}

	PSB_DEBUG_GENERAL("TOPAZ: opened firmware\n");

	if ((raw == NULL) || (raw->size < sizeof(struct topaz_fwinfo))) {
		DRM_ERROR("TOPAZ: firmware file is not correct size.\n");
		goto out;
	}

	ptr = (unsigned char *)raw->data;

	if (!ptr) {
		DRM_ERROR("TOPAZ: failed to load firmware.\n");
		goto out;
	}

	/* # load fw from file */
	PSB_DEBUG_GENERAL("TOPAZ: load firmware.....\n");
	cur_fw = NULL;
	/* didn't use the first element */
	for (n = 1; n < IMG_CODEC_NUM; ++n) {
		cur_fw = (struct topaz_fwinfo *)ptr;

		cur_codec = &topaz_priv->topaz_fw[cur_fw->codec];
		cur_codec->ver = cur_fw->ver;
		cur_codec->codec = cur_fw->codec;
		cur_codec->text_size = cur_fw->text_size;
		cur_codec->data_size = cur_fw->data_size;
		cur_codec->data_location = cur_fw->data_location;

		PSB_DEBUG_GENERAL("TOPAZ: load firemware %s.\n",
				  codec_to_string(cur_fw->codec));

		/* #.# handle text section */
		ptr += sizeof(struct topaz_fwinfo);
		cur_drm_obj = &cur_codec->text;
		cur_size = cur_fw->text_size;

		/* #.# fill DRM object with firmware data */
		ret = ttm_bo_kmap(*cur_drm_obj, 0, (*cur_drm_obj)->num_pages,
				  &tmp_kmap);
		if (ret) {
			PSB_DEBUG_GENERAL("drm_bo_kmap failed: %d\n", ret);
			ttm_bo_unref(cur_drm_obj);
			*cur_drm_obj = NULL;
			goto out;
		}

		memcpy(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), ptr,
		       cur_size);

		ttm_bo_kunmap(&tmp_kmap);

		/* #.# handle data section */
		ptr += cur_fw->text_size;
		cur_drm_obj = &cur_codec->data;
		cur_size = cur_fw->data_size;

		/* #.# fill DRM object with firmware data */
		ret = ttm_bo_kmap(*cur_drm_obj, 0, (*cur_drm_obj)->num_pages,
				  &tmp_kmap);
		if (ret) {
			PSB_DEBUG_GENERAL("drm_bo_kmap failed: %d\n", ret);
			ttm_bo_unref(cur_drm_obj);
			*cur_drm_obj = NULL;
			goto out;
		}

		memcpy(ttm_kmap_obj_virtual(&tmp_kmap, &is_iomem), ptr,
		       cur_size);

		ttm_bo_kunmap(&tmp_kmap);

		/* #.# validate firmware */

		/* #.# update ptr */
		ptr += cur_fw->data_size;
	}

	release_firmware(raw);

	PSB_DEBUG_GENERAL("TOPAZ: return from firmware init\n");

	return 0;

 out:
	if (raw) {
		PSB_DEBUG_GENERAL("release firmware....\n");
		release_firmware(raw);
	}

	return -1;
}

/* setup fw when start a new context */
int topaz_setup_fw(struct drm_device *dev, enum drm_lnc_topaz_codec codec)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t mem_size = RAM_SIZE;	/* follow DDK */
	uint32_t verify_pc;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

#if 0
	if (codec == topaz_priv->topaz_current_codec) {
		LNC_TRACEL("TOPAZ: reuse previous codec\n");
		return 0;
	}
#endif

	/* XXX: need to rest topaz? */
	PSB_DEBUG_GENERAL("XXX: should reset topaz when context change?\n");

	/* XXX: interrupt enable shouldn't be enable here,
	 * this funtion is called when interrupt is enable,
	 * but here, we've no choice since we have to call setup_fw by
	 * manual */
	/* # upload firmware, clear interruputs and start the firmware
	 * -- from  hostutils.c in TestSuits*/

	/* # reset MVEA */
	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(1, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	MVEA_WRITE32(MVEA_CR_IMG_MVEA_SRST,
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_CMC_SOFT_RESET) |
		     F_ENCODE(0, MVEA_CR_IMG_MVEA_DCF_SOFT_RESET));

	topaz_mmu_hwsetup(dev_priv);

#if !LNC_TOPAZ_NO_IRQ
	psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
#endif

	PSB_DEBUG_GENERAL("TOPAZ: will setup firmware....\n");

	topaz_set_default_regs(dev_priv);

	/* # reset mtx */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_SRST,
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_MVEA_SOFT_RESET) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_VLC_SOFT_RESET));

	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_SRST, 0x0);

	/* # upload fw by drm */
	PSB_DEBUG_GENERAL("TOPAZ: will upload firmware\n");

	topaz_upload_fw(dev, codec);
#if 0
	/* allocate the space for context save & restore if needed */
	if (topaz_priv->topaz_mtx_data_mem == NULL) {
		ret = ttm_buffer_object_create(bdev,
					       topaz_priv->cur_mtx_data_size *
					       4, ttm_bo_type_kernel,
					       DRM_PSB_FLAG_MEM_MMU |
					       TTM_PL_FLAG_NO_EVICT, 0, 0, 0,
					       NULL,
					       &topaz_priv->topaz_mtx_data_mem);
		if (ret) {
			DRM_ERROR("TOPAZ: failed to allocate ttm buffer for "
				  "mtx data save\n");
			return -1;
		}
	}
	PSB_DEBUG_GENERAL("TOPAZ: after upload fw ....\n");
#endif

	/* XXX: In power save mode, need to save the complete data memory
	 * and restore it. MTX_FWIF.c record the data size  */
	PSB_DEBUG_GENERAL("TOPAZ:in power save mode need to save memory?\n");

	PSB_DEBUG_GENERAL("TOPAZ: setting up pc address\n");
	topaz_write_core_reg(dev_priv, TOPAZ_MTX_PC, PC_START_ADDRESS);

	PSB_DEBUG_GENERAL("TOPAZ: verify pc address\n");

	topaz_read_core_reg(dev_priv, TOPAZ_MTX_PC, &verify_pc);

	/* enable auto clock is essential for this driver */
	TOPAZ_WRITE32(TOPAZ_CR_TOPAZ_AUTO_CLK_GATE,
		      F_ENCODE(1, TOPAZ_CR_TOPAZ_VLC_AUTO_CLK_GATE) |
		      F_ENCODE(1, TOPAZ_CR_TOPAZ_DB_AUTO_CLK_GATE));
	MVEA_WRITE32(MVEA_CR_MVEA_AUTO_CLOCK_GATING,
		     F_ENCODE(1, MVEA_CR_MVEA_IPE_AUTO_CLK_GATE) |
		     F_ENCODE(1, MVEA_CR_MVEA_SPE_AUTO_CLK_GATE) |
		     F_ENCODE(1, MVEA_CR_MVEA_CMPRS_AUTO_CLK_GATE) |
		     F_ENCODE(1, MVEA_CR_MVEA_JMCOMP_AUTO_CLK_GATE));

	PSB_DEBUG_GENERAL("TOPAZ: current pc(%08X) vs %08X\n",
			  verify_pc, PC_START_ADDRESS);

	/* # turn on MTX */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTCLEAR,
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX));

	MTX_WRITE32(MTX_CORE_CR_MTX_ENABLE_OFFSET,
		    MTX_CORE_CR_MTX_ENABLE_MTX_ENABLE_MASK);

	/* # poll on the interrupt which the firmware will generate */
	topaz_wait_for_register(dev_priv,
				TOPAZ_START + TOPAZ_CR_IMG_TOPAZ_INTSTAT,
				F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTS_MTX),
				F_MASK(TOPAZ_CR_IMG_TOPAZ_INTS_MTX));

	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTCLEAR,
		      F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX));

	PSB_DEBUG_GENERAL("TOPAZ: after topaz mtx setup ....\n");

	/* # get ccb buffer addr -- file hostutils.c */
	topaz_priv->topaz_ccb_buffer_addr =
	    topaz_read_mtx_mem(dev_priv, MTX_DATA_MEM_BASE + mem_size - 4);
	topaz_priv->topaz_ccb_ctrl_addr =
	    topaz_read_mtx_mem(dev_priv, MTX_DATA_MEM_BASE + mem_size - 8);
	topaz_priv->topaz_ccb_size =
	    topaz_read_mtx_mem(dev_priv,
			       topaz_priv->topaz_ccb_ctrl_addr +
			       MTX_CCBCTRL_CCBSIZE);

	topaz_priv->topaz_cmd_windex = 0;

	PSB_DEBUG_GENERAL("TOPAZ:ccb_buffer_addr(%x),ctrl_addr(%x) size(%d)\n",
			  topaz_priv->topaz_ccb_buffer_addr,
			  topaz_priv->topaz_ccb_ctrl_addr,
			  topaz_priv->topaz_ccb_size);

	/* # write back the initial QP Value */
	topaz_write_mtx_mem(dev_priv,
			    topaz_priv->topaz_ccb_ctrl_addr +
			    MTX_CCBCTRL_INITQP, topaz_priv->stored_initial_qp);

	PSB_DEBUG_GENERAL("TOPAZ: write WB mem address 0x%08x\n",
			  topaz_priv->topaz_wb_offset);
	topaz_write_mtx_mem(dev_priv, MTX_DATA_MEM_BASE + mem_size - 12,
			    topaz_priv->topaz_wb_offset);

	/* this kick is essential for mtx.... */
	*((uint32_t *) topaz_priv->topaz_ccb_wb) = 0x01020304;
	topaz_mtx_kick(dev_priv, 1);
	DRM_UDELAY(1000);
	PSB_DEBUG_GENERAL("TOPAZ: DDK expected 0x12345678 in WB memory,"
			  " and here it is 0x%08x\n",
			  *((uint32_t *) topaz_priv->topaz_ccb_wb));

	*((uint32_t *) topaz_priv->topaz_ccb_wb) = 0x0;	/* reset it to 0 */
	PSB_DEBUG_GENERAL("TOPAZ: firmware uploaded.\n");

	/* XXX: is there any need to record next cmd num??
	 * we use fence seqence number to record it
	 */
	topaz_priv->topaz_busy = 0;
	topaz_priv->topaz_cmd_seq = 0;

#if !LNC_TOPAZ_NO_IRQ
	psb_irq_preinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
	psb_irq_postinstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
	lnc_topaz_enableirq(dev);
#endif

#if 0
	topaz_mmu_flushcache(dev_priv);
	topaz_test_null(dev, 0xe1e1);
	topaz_test_null(dev, 0xe2e2);
	topaz_test_sync(dev, 0xe2e2, 0x87654321);

	topaz_mmu_test(dev, 0x12345678);
	topaz_test_null(dev, 0xe3e3);
	topaz_mmu_test(dev, 0x8764321);

	topaz_test_null(dev, 0xe4e4);
	topaz_test_null(dev, 0xf3f3);
#endif

	return 0;
}

#if UPLOAD_FW_BY_DMA
int topaz_upload_fw(struct drm_device *dev, enum drm_lnc_topaz_codec codec)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	const struct topaz_codec_fw *cur_codec_fw;
	uint32_t text_size, data_size;
	uint32_t data_location;
	uint32_t cur_mtx_data_size;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	/* # refer HLD document */

	/* # MTX reset */
	PSB_DEBUG_GENERAL("TOPAZ: mtx reset.\n");
	MTX_WRITE32(MTX_CORE_CR_MTX_SOFT_RESET_OFFSET,
		    MTX_CORE_CR_MTX_SOFT_RESET_MTX_RESET_MASK);

	DRM_UDELAY(6000);

	/* # upload the firmware by DMA */
	cur_codec_fw = &topaz_priv->topaz_fw[codec];

	PSB_DEBUG_GENERAL("Topaz:upload codec %s(%d) text sz=%d data sz=%d"
			  " data location(0x%08x)\n", codec_to_string(codec),
			  codec, cur_codec_fw->text_size,
			  cur_codec_fw->data_size, cur_codec_fw->data_location);

	/* # upload text */
	text_size = cur_codec_fw->text_size / 4;

	/* setup the MTX to start recieving data:
	   use a register for the transfer which will point to the source
	   (MTX_CR_MTX_SYSC_CDMAT) */
	/* #.# fill the dst addr */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA, 0x80900000);
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(2, MTX_BURSTSIZE) |
		    F_ENCODE(0, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) | F_ENCODE(text_size, MTX_LENGTH));

	/* #.# set DMAC access to host memory via BIF */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1);

	/* #.# transfer the codec */
	topaz_dma_transfer(dev_priv, 0, cur_codec_fw->text->offset, 0,
			   MTX_CR_MTX_SYSC_CDMAT, text_size, 0, 0);

	/* #.# wait dma finish */
	topaz_wait_for_register(dev_priv,
				DMAC_START + IMG_SOC_DMAC_IRQ_STAT(0),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN));

	/* #.# clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	/* # return access to topaz core */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 0);

	/* # upload data */
	data_size = cur_codec_fw->data_size / 4;
	data_location = cur_codec_fw->data_location;

	/* #.# fill the dst addr */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA,
		    0x80900000 + (data_location - 0x82880000));
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(2, MTX_BURSTSIZE) |
		    F_ENCODE(0, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) | F_ENCODE(data_size, MTX_LENGTH));

	/* #.# set DMAC access to host memory via BIF */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1);

	/* #.# transfer the codec */
	topaz_dma_transfer(dev_priv, 0, cur_codec_fw->data->offset, 0,
			   MTX_CR_MTX_SYSC_CDMAT, data_size, 0, 0);

	/* #.# wait dma finish */
	topaz_wait_for_register(dev_priv,
				DMAC_START + IMG_SOC_DMAC_IRQ_STAT(0),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN));

	/* #.# clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	/* # return access to topaz core */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 0);

	/* record this codec's mtx data size for
	 * context save & restore */
	/* FIXME: since non-root sighting fixed by pre allocated,
	 * only need to correct the buffer size
	 */
	cur_mtx_data_size = data_size;
	if (topaz_priv->cur_mtx_data_size != cur_mtx_data_size)
		topaz_priv->cur_mtx_data_size = cur_mtx_data_size;

	return 0;
}

#else

void topaz_mtx_upload_by_register(struct drm_device *dev, uint32_t mtx_mem,
				  uint32_t addr, uint32_t size,
				  struct ttm_buffer_object *buf)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t *buf_p;
	uint32_t debug_reg, bank_size, bank_ram_size, bank_count;
	uint32_t cur_ram_id, ram_addr, ram_id;
	int map_ret, lp;
	struct ttm_bo_kmap_obj bo_kmap;
	bool is_iomem;
	uint32_t cur_addr;

	get_mtx_control_from_dash(dev_priv);

	map_ret = ttm_bo_kmap(buf, 0, buf->num_pages, &bo_kmap);
	if (map_ret) {
		DRM_ERROR("TOPAZ: drm_bo_kmap failed: %d\n", map_ret);
		return;
	}
	buf_p = (uint32_t *) ttm_kmap_obj_virtual(&bo_kmap, &is_iomem);

	TOPAZ_READ32(TOPAZ_CORE_CR_MTX_DEBUG_OFFSET, &debug_reg);
	debug_reg = 0x0a0a0606;
	bank_size = (debug_reg & 0xf0000) >> 16;
	bank_ram_size = 1 << (bank_size + 2);

	bank_count = (debug_reg & 0xf00) >> 8;

	topaz_wait_for_register(dev_priv,
		MTX_START +
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_OFFSET,
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_MTX_MTX_MCM_STAT_MASK,
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_MTX_MTX_MCM_STAT_MASK);

	cur_ram_id = -1;
	cur_addr = addr;
	for (lp = 0; lp < size / 4; ++lp) {
		ram_id = mtx_mem + (cur_addr / bank_ram_size);

		if (cur_ram_id != ram_id) {
			ram_addr = cur_addr >> 2;

			MTX_WRITE32(MTX_CORE_CR_MTX_RAM_ACCESS_CONTROL_OFFSET,
				    F_ENCODE(ram_id, MTX_MTX_MCMID) |
				    F_ENCODE(ram_addr, MTX_MTX_MCM_ADDR) |
				    F_ENCODE(1, MTX_MTX_MCMAI));

			cur_ram_id = ram_id;
		}
		cur_addr += 4;

		MTX_WRITE32(MTX_CORE_CR_MTX_RAM_ACCESS_DATA_TRANSFER_OFFSET,
			    *(buf_p + lp));

		topaz_wait_for_register(dev_priv,
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_OFFSET
		+ MTX_START,
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_MTX_MTX_MCM_STAT_MASK,
		MTX_CORE_CR_MTX_RAM_ACCESS_STATUS_MTX_MTX_MCM_STAT_MASK);
	}

	ttm_bo_kunmap(&bo_kmap);

	PSB_DEBUG_GENERAL("TOPAZ: register data upload done\n");
	return;
}

int topaz_upload_fw(struct drm_device *dev, enum drm_lnc_topaz_codec codec)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	const struct topaz_codec_fw *cur_codec_fw;
	uint32_t text_size, data_size;
	uint32_t data_location;

	/* # refer HLD document */
	/* # MTX reset */
	PSB_DEBUG_GENERAL("TOPAZ: mtx reset.\n");
	MTX_WRITE32(MTX_CORE_CR_MTX_SOFT_RESET_OFFSET,
		    MTX_CORE_CR_MTX_SOFT_RESET_MTX_RESET_MASK);

	DRM_UDELAY(6000);

	/* # upload the firmware by DMA */
	cur_codec_fw = &topaz_priv->topaz_fw[codec];

	PSB_DEBUG_GENERAL("Topaz: upload codec %s text size(%d) data size(%d)"
			  " data location(0x%08x)\n", codec_to_string(codec),
			  cur_codec_fw->text_size, cur_codec_fw->data_size,
			  cur_codec_fw->data_location);

	/* # upload text */
	text_size = cur_codec_fw->text_size;

	topaz_mtx_upload_by_register(dev, LNC_MTX_CORE_CODE_MEM,
				     PC_START_ADDRESS - MTX_MEMORY_BASE,
				     text_size, cur_codec_fw->text);

	/* # upload data */
	data_size = cur_codec_fw->data_size;
	data_location = cur_codec_fw->data_location;

	topaz_mtx_upload_by_register(dev, LNC_MTX_CORE_DATA_MEM,
				     data_location - 0x82880000, data_size,
				     cur_codec_fw->data);

	return 0;
}

#endif				/* UPLOAD_FW_BY_DMA */

void
topaz_dma_transfer(struct drm_psb_private *dev_priv, uint32_t channel,
		   uint32_t src_phy_addr, uint32_t offset,
		   uint32_t soc_addr, uint32_t byte_num,
		   uint32_t is_increment, uint32_t is_write)
{
	uint32_t dmac_count;
	uint32_t irq_stat;
	uint32_t count;

	PSB_DEBUG_GENERAL("TOPAZ: using dma to transfer firmware\n");
	/* # check that no transfer is currently in progress and no
	   interrupts are outstanding ?? (why care interrupt) */
	DMAC_READ32(IMG_SOC_DMAC_COUNT(channel), &dmac_count);
	if (0 != (dmac_count & (MASK_IMG_SOC_EN | MASK_IMG_SOC_LIST_EN)))
		DRM_ERROR("TOPAZ: there is tranfer in progress\n");


	/* no hold off period */
	DMAC_WRITE32(IMG_SOC_DMAC_PER_HOLD(channel), 0);
	/* clear previous interrupts */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(channel), 0);
	/* check irq status */
	DMAC_READ32(IMG_SOC_DMAC_IRQ_STAT(channel), &irq_stat);
	/* assert(0 == irq_stat); */
	if (0 != irq_stat)
		DRM_ERROR("TOPAZ: there is hold up\n");

	DMAC_WRITE32(IMG_SOC_DMAC_SETUP(channel), (src_phy_addr + offset));
	count = DMAC_VALUE_COUNT(DMAC_BSWAP_NO_SWAP, DMAC_PWIDTH_32_BIT,
				 is_write, DMAC_PWIDTH_32_BIT, byte_num);
	/* generate an interrupt at the end of transfer */
	count |= MASK_IMG_SOC_TRANSFER_IEN;
	count |= F_ENCODE(is_write, IMG_SOC_DIR);
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel), count);

	DMAC_WRITE32(IMG_SOC_DMAC_PERIPH(channel),
		     DMAC_VALUE_PERIPH_PARAM(DMAC_ACC_DEL_0,
					     is_increment, DMAC_BURST_2));

	DMAC_WRITE32(IMG_SOC_DMAC_PERIPHERAL_ADDR(channel), soc_addr);

	/* Finally, rewrite the count register with
	 * the enable bit set to kick off the transfer
	 */
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel), count | MASK_IMG_SOC_EN);

	PSB_DEBUG_GENERAL("TOPAZ: dma transfer started.\n");

	return;
}

void topaz_set_default_regs(struct drm_psb_private *dev_priv)
{
	int n;
	int count = sizeof(topaz_default_regs) / (sizeof(unsigned long) * 3);

	for (n = 0; n < count; n++)
		MM_WRITE32(topaz_default_regs[n][0],
			   topaz_default_regs[n][1], topaz_default_regs[n][2]);

}

void topaz_write_core_reg(struct drm_psb_private *dev_priv, uint32_t reg,
			  const uint32_t val)
{
	uint32_t tmp;
	get_mtx_control_from_dash(dev_priv);

	/* put data into MTX_RW_DATA */
	MTX_WRITE32(MTX_CORE_CR_MTX_REGISTER_READ_WRITE_DATA_OFFSET, val);

	/* request a write */
	tmp = reg &
	    ~MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK;
	MTX_WRITE32(MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_OFFSET, tmp);

	/* wait for operation finished */
	topaz_wait_for_register(dev_priv,
		MTX_START +
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_OFFSET,
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK,
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK);

	release_mtx_control_from_dash(dev_priv);
}

void topaz_read_core_reg(struct drm_psb_private *dev_priv, uint32_t reg,
			 uint32_t *ret_val)
{
	uint32_t tmp;

	get_mtx_control_from_dash(dev_priv);

	/* request a write */
	tmp = (reg &
	       ~MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK);
	MTX_WRITE32(MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_OFFSET,
		    MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_RNW_MASK |
		    tmp);

	/* wait for operation finished */
	topaz_wait_for_register(dev_priv,
		MTX_START +
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_OFFSET,
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK,
		MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK);

	/* read  */
	MTX_READ32(MTX_CORE_CR_MTX_REGISTER_READ_WRITE_DATA_OFFSET, ret_val);

	release_mtx_control_from_dash(dev_priv);
}

void get_mtx_control_from_dash(struct drm_psb_private *dev_priv)
{
	int debug_reg_slave_val;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	/* GetMTXControlFromDash */
	TOPAZ_WRITE32(TOPAZ_CORE_CR_MTX_DEBUG_OFFSET,
		      F_ENCODE(1, TOPAZ_CR_MTX_DBG_IS_SLAVE) |
		      F_ENCODE(2, TOPAZ_CR_MTX_DBG_GPIO_OUT));
	do {
		TOPAZ_READ32(TOPAZ_CORE_CR_MTX_DEBUG_OFFSET,
			     &debug_reg_slave_val);
	} while ((debug_reg_slave_val & 0x18) != 0);

	/* save access control */
	TOPAZ_READ32(MTX_CORE_CR_MTX_RAM_ACCESS_CONTROL_OFFSET,
		     &topaz_priv->topaz_dash_access_ctrl);
}

void release_mtx_control_from_dash(struct drm_psb_private *dev_priv)
{
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	/* restore access control */
	TOPAZ_WRITE32(MTX_CORE_CR_MTX_RAM_ACCESS_CONTROL_OFFSET,
		      topaz_priv->topaz_dash_access_ctrl);

	/* release bus */
	TOPAZ_WRITE32(TOPAZ_CORE_CR_MTX_DEBUG_OFFSET,
		      F_ENCODE(1, TOPAZ_CR_MTX_DBG_IS_SLAVE));
}

void topaz_mmu_hwsetup(struct drm_psb_private *dev_priv)
{
	uint32_t pd_addr = psb_get_default_pd_addr(dev_priv->mmu);

	/* bypass all request while MMU is being configured */
	TOPAZ_WRITE32(TOPAZ_CR_MMU_CONTROL0, F_ENCODE(1, TOPAZ_CR_MMU_BYPASS));

	/* set MMU hardware at the page table directory */
	PSB_DEBUG_GENERAL("TOPAZ: write PD phyaddr=0x%08x "
			  "into MMU_DIR_LIST0/1\n", pd_addr);
	TOPAZ_WRITE32(TOPAZ_CR_MMU_DIR_LIST_BASE(0), pd_addr);
	TOPAZ_WRITE32(TOPAZ_CR_MMU_DIR_LIST_BASE(1), 0);

	/* setup index register, all pointing to directory bank 0 */
	TOPAZ_WRITE32(TOPAZ_CR_MMU_BANK_INDEX, 0);

	/* now enable MMU access for all requestors */
	TOPAZ_WRITE32(TOPAZ_CR_MMU_CONTROL0, 0);
}

void topaz_mmu_flushcache(struct drm_psb_private *dev_priv)
{
	uint32_t mmu_control;

	if (dev_priv->topaz_disabled)
		return;

#if 0
	PSB_DEBUG_GENERAL("XXX: Only one PTD/PTE cache"
			  " so flush using the master core\n");
#endif
	/* XXX: disable interrupt */

	TOPAZ_READ32(TOPAZ_CR_MMU_CONTROL0, &mmu_control);
	mmu_control |= F_ENCODE(1, TOPAZ_CR_MMU_INVALDC);
	mmu_control |= F_ENCODE(1, TOPAZ_CR_MMU_FLUSH);

#if 0
	PSB_DEBUG_GENERAL("Set Invalid flag (this causes a flush with MMU\n"
			  "still operating afterwards even if not cleared,\n"
			  "but may want to replace with MMU_FLUSH?\n");
#endif
	TOPAZ_WRITE32(TOPAZ_CR_MMU_CONTROL0, mmu_control);

	/* clear it */
	mmu_control &= (~F_ENCODE(1, TOPAZ_CR_MMU_INVALDC));
	mmu_control &= (~F_ENCODE(1, TOPAZ_CR_MMU_FLUSH));
	TOPAZ_WRITE32(TOPAZ_CR_MMU_CONTROL0, mmu_control);
}

#if DEBUG_FUNCTION

static int topaz_test_sync(struct drm_device *dev, uint32_t seq,
			   uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	uint32_t sync_cmd[3];
	struct topaz_cmd_header *cmd_hdr;
	uint32_t *sync_p = (uint32_t *) topaz_priv->topaz_sync_addr;
	int count = 1000;
	uint32_t clr_flag;

	cmd_hdr = (struct topaz_cmd_header *)&sync_cmd[0];

	/* reset sync area */
	*sync_p = 0;

	/* insert a SYNC command here */
	cmd_hdr->id = MTX_CMDID_SYNC;
	cmd_hdr->size = 3;
	cmd_hdr->seq = seq;

	sync_cmd[1] = topaz_priv->topaz_sync_offset;
	sync_cmd[2] = sync_seq;

	TOPAZ_BEGIN_CCB(dev_priv);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[0]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[1]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[2]);
	TOPAZ_END_CCB(dev_priv, 1);

	PSB_DEBUG_GENERAL("Topaz: Sent SYNC with cmd seq=0x%08x,"
			  "sync_seq=0x%08x\n", seq, sync_seq);

	while (count && *sync_p != sync_seq) {
		DRM_UDELAY(100);
		--count;
	}
	if ((count == 0) && (*sync_p != sync_seq)) {
		DRM_ERROR("TOPAZ: wait sycn timeout, expect sync seq 0x%08x,"
			  "actual 0x%08x\n", sync_seq, *sync_p);
	}
	PSB_DEBUG_GENERAL("TOPAZ: SYNC succeed, sync seq=0x%08x\n", *sync_p);
	PSB_DEBUG_GENERAL("Topaz: after SYNC test, query IRQ and clear it\n");

	clr_flag = lnc_topaz_queryirq(dev);
	lnc_topaz_clearirq(dev, clr_flag);

	return 0;
}

static int topaz_test_sync_tt_test(struct drm_device *dev,
				   uint32_t seq, uint32_t sync_seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	int ret;
	bool is_iomem;
	struct ttm_buffer_object *test_obj;
	struct ttm_bo_kmap_obj test_kmap;
	unsigned int *test_adr;
	uint32_t sync_cmd[3];
	int count = 1000;
	unsigned long pfn;

	ret = ttm_buffer_object_create(bdev, 4096,
				       ttm_bo_type_kernel,
				       TTM_PL_FLAG_TT | TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &test_obj);
	if (ret) {
		DRM_ERROR("failed create test object buffer\n");
		return -1;
	}

	ret = psb_mmu_virtual_to_pfn(psb_mmu_get_default_pd(dev_priv->mmu),
				     test_obj->offset, &pfn);
	if (ret) {
		DRM_ERROR("failed to get pfn from virtual\n");
		return -1;
	}

	PSB_DEBUG_GENERAL("Topaz:offset %lx, pfn %lx\n", test_obj->offset, pfn);

	ret = ttm_bo_kmap(test_obj, 0, test_obj->num_pages, &test_kmap);
	if (ret) {
		DRM_ERROR("failed map buffer\n");
		return -1;
	}
	test_adr = ttm_kmap_obj_virtual(&test_kmap, &is_iomem);
	*test_adr = 0xff55;
	ttm_bo_kunmap(&test_kmap);

	/* insert a SYNC command here */
	sync_cmd[0] = (MTX_CMDID_SYNC << 1) | (3 << 8) | (seq << 16);
	sync_cmd[1] = test_obj->offset;
	sync_cmd[2] = sync_seq;

	TOPAZ_BEGIN_CCB(dev_priv);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[0]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[1]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[2]);
	TOPAZ_END_CCB(dev_priv, 1);

	ret = ttm_bo_kmap(test_obj, 0, test_obj->num_pages, &test_kmap);
	if (ret) {
		DRM_ERROR("failed map buffer\n");
		return -1;
	}
	test_adr = ttm_kmap_obj_virtual(&test_kmap, &is_iomem);

	while (count && *test_adr != sync_seq) {
		DRM_UDELAY(100);
		--count;
	}
	if ((count == 0) && (*test_adr != sync_seq)) {
		DRM_ERROR("TOPAZ: wait sycn timeout (0x%08x),"
			  "actual 0x%08x\n", sync_seq, *test_adr);
	}
	PSB_DEBUG_GENERAL("TOPAZ: SYNC done, seq=0x%08x\n", *test_adr);
	ttm_bo_kunmap(&test_kmap);
	ttm_bo_unref(&test_obj);

	return 0;
}

static int topaz_test_sync_manual_alloc_page(struct drm_device *dev,
					     uint32_t seq,
					     uint32_t sync_seq, uint32_t offset)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret;
	uint32_t sync_cmd[3];
	int count = 1000;
	unsigned long pfn;

	struct page *p;
	uint32_t *v;
	/*      uint32_t offset = 0xd0000000; */

	p = alloc_page(GFP_DMA32);
	if (!p) {
		DRM_ERROR("Topaz:Failed allocating page\n");
		return -1;
	}

	v = kmap(p);
	memset(v, 0x67, PAGE_SIZE);
	pfn = (offset >> PAGE_SHIFT);
	kunmap(p);

	ret = psb_mmu_insert_pages(psb_mmu_get_default_pd(dev_priv->mmu),
				   &p, pfn << PAGE_SHIFT, 1, 0, 0, 0);
	if (ret) {
		DRM_ERROR("Topaz:Failed inserting mmu page\n");
		return -1;
	}

	/* insert a SYNC command here */
	sync_cmd[0] = (MTX_CMDID_SYNC << 1) | (3 << 8) | (0x5b << 16);
	sync_cmd[1] = pfn << PAGE_SHIFT;
	sync_cmd[2] = seq;

	TOPAZ_BEGIN_CCB(dev_priv);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[0]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[1]);
	TOPAZ_OUT_CCB(dev_priv, sync_cmd[2]);
	TOPAZ_END_CCB(dev_priv, 1);

	v = kmap(p);
	while (count && *v != sync_seq) {
		DRM_UDELAY(100);
		--count;
	}
	if ((count == 0) && (*v != sync_seq)) {
		DRM_ERROR("TOPAZ: wait sycn timeout (0x%08x),"
			  "actual 0x%08x\n", sync_seq, *v);
	}
	PSB_DEBUG_GENERAL("TOPAZ: SYNC done, seq=0x%08x\n", *v);
	kunmap(p);

	return 0;
}

static int topaz_test_null(struct drm_device *dev, uint32_t seq)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_cmd_header null_cmd;
	uint32_t clr_flag;

	/* XXX: here we finished firmware setup....
	 * using a NULL command to verify the
	 * correctness of firmware
	 */

	null_cmd.id = MTX_CMDID_NULL;
	null_cmd.size = 1;
	null_cmd.seq = seq;

	TOPAZ_BEGIN_CCB(dev_priv);
	TOPAZ_OUT_CCB(dev_priv, *((uint32_t *) &null_cmd));
	TOPAZ_END_CCB(dev_priv, 1);

	DRM_UDELAY(1000);	/* wait to finish */

	PSB_DEBUG_GENERAL("Topaz: Sent NULL with sequence=0x%08x,"
			  " got sequence=0x%08x (WB_seq=0x%08x,WB_roff=%d)\n",
			  seq, CCB_CTRL_SEQ(dev_priv),
			  WB_CCB_CTRL_SEQ(dev_priv),
			  WB_CCB_CTRL_RINDEX(dev_priv));

	PSB_DEBUG_GENERAL("Topaz: after NULL test, query IRQ and clear it\n");

	clr_flag = lnc_topaz_queryirq(dev);
	lnc_topaz_clearirq(dev, clr_flag);

	return 0;
}

/*
 * this function will test whether the mmu is correct:
 * it get a drm_buffer_object and use CMD_SYNC to write
 * certain value into this buffer.
 */
static void topaz_mmu_test(struct drm_device *dev, uint32_t sync_value)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	unsigned long real_pfn;
	int ret;

	/* topaz_mmu_flush(dev); */
	topaz_test_sync(dev, 0x55, sync_value);

	ret = psb_mmu_virtual_to_pfn(psb_mmu_get_default_pd(dev_priv->mmu),
				     topaz_priv->topaz_sync_offset, &real_pfn);
	if (ret != 0) {
		PSB_DEBUG_GENERAL("psb_mmu_virtual_to_pfn failed,exit\n");
		return;
	}
	PSB_DEBUG_GENERAL("TOPAZ: issued SYNC command, "
			  "BO offset=0x%08x (pfn=%lu), synch value=0x%08x\n",
			  topaz_priv->topaz_sync_offset, real_pfn, sync_value);
}

void topaz_save_default_regs(struct drm_psb_private *dev_priv, uint32_t * data)
{
	int n;
	int count;

	count = sizeof(topaz_default_regs) / (sizeof(unsigned long) * 3);
	for (n = 0; n < count; n++, ++data)
		MM_READ32(topaz_default_regs[n][0],
			  topaz_default_regs[n][1], data);

}

void topaz_restore_default_regs(struct drm_psb_private *dev_priv,
				uint32_t *data)
{
	int n;
	int count;

	count = sizeof(topaz_default_regs) / (sizeof(unsigned long) * 3);
	for (n = 0; n < count; n++, ++data)
		MM_WRITE32(topaz_default_regs[n][0],
			   topaz_default_regs[n][1], *data);

}

#endif

int lnc_topaz_restore_mtx_state(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	uint32_t reg_val;
	uint32_t *mtx_reg_state;
	int i;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	if (!topaz_priv->topaz_mtx_saved)
		return -1;

	if (topaz_priv->topaz_mtx_data_mem == NULL) {
		PSB_DEBUG_GENERAL("TOPAZ: try to restore context without "
				  "space allocated, return directly without restore\n");
		return -1;
	}

	/* turn on mtx clocks */
	MTX_READ32(TOPAZ_CR_TOPAZ_MAN_CLK_GATE, &reg_val);
	MTX_WRITE32(TOPAZ_CR_TOPAZ_MAN_CLK_GATE,
		    reg_val & (~MASK_TOPAZ_CR_TOPAZ_MTX_MAN_CLK_GATE));

	/* reset mtx */
	/* FIXME: should use core_write??? */
	MTX_WRITE32(MTX_CORE_CR_MTX_SOFT_RESET_OFFSET,
		    MTX_CORE_CR_MTX_SOFT_RESET_MTX_RESET_MASK);
	DRM_UDELAY(6000);

	topaz_mmu_hwsetup(dev_priv);
	/* upload code, restore mtx data */
	mtx_dma_write(dev);

	mtx_reg_state = topaz_priv->topaz_mtx_reg_state;
	/* restore register */
	/* FIXME: conside to put read/write into one function */
	/* Saves 8 Registers of D0 Bank  */
	/* DoRe0, D0Ar6, D0Ar4, D0Ar2, D0FrT, D0.5, D0.6 and D0.7 */
	for (i = 0; i < 8; i++) {
		topaz_write_core_reg(dev_priv, 0x1 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 8 Registers of D1 Bank  */
	/* D1Re0, D1Ar5, D1Ar3, D1Ar1, D1RtP, D1.5, D1.6 and D1.7 */
	for (i = 0; i < 8; i++) {
		topaz_write_core_reg(dev_priv, 0x2 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 4 Registers of A0 Bank  */
	/* A0StP, A0FrP, A0.2 and A0.3 */
	for (i = 0; i < 4; i++) {
		topaz_write_core_reg(dev_priv, 0x3 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 4 Registers of A1 Bank  */
	/* A1GbP, A1LbP, A1.2 and A1.3 */
	for (i = 0; i < 4; i++) {
		topaz_write_core_reg(dev_priv, 0x4 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves PC and PCX  */
	for (i = 0; i < 2; i++) {
		topaz_write_core_reg(dev_priv, 0x5 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 8 Control Registers */
	/* TXSTAT, TXMASK, TXSTATI, TXMASKI, TXPOLL, TXGPIOI, TXPOLLI,
	 * TXGPIOO */
	for (i = 0; i < 8; i++) {
		topaz_write_core_reg(dev_priv, 0x7 | (i << 4), *mtx_reg_state);
		mtx_reg_state++;
	}

	/* turn on MTX */
	MTX_WRITE32(MTX_CORE_CR_MTX_ENABLE_OFFSET,
		    MTX_CORE_CR_MTX_ENABLE_MTX_ENABLE_MASK);

	topaz_priv->topaz_mtx_saved = 0;

	return 0;
}

int lnc_topaz_save_mtx_state(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	uint32_t *mtx_reg_state;
	int i;
	struct topaz_codec_fw *cur_codec_fw;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	/* FIXME: make sure the topaz_mtx_data_mem is allocated */
	if (topaz_priv->topaz_mtx_data_mem == NULL) {
		PSB_DEBUG_GENERAL("TOPAZ: try to save context without space "
				  "allocated, return directly without save\n");
		return -1;
	}
	if (topaz_priv->topaz_fw_loaded == 0) {
		PSB_DEBUG_GENERAL("TOPAZ: try to save context without firmware "
				  "uploaded\n");
		return -1;
	}

	topaz_wait_for_register(dev_priv,
				MTX_START + MTX_CORE_CR_MTX_TXRPT_OFFSET,
				TXRPT_WAITONKICK_VALUE, 0xffffffff);

	/* stop mtx */
	MTX_WRITE32(MTX_CORE_CR_MTX_ENABLE_OFFSET,
		    MTX_CORE_CR_MTX_ENABLE_MTX_TOFF_MASK);

	mtx_reg_state = topaz_priv->topaz_mtx_reg_state;

	/* FIXME: conside to put read/write into one function */
	/* Saves 8 Registers of D0 Bank  */
	/* DoRe0, D0Ar6, D0Ar4, D0Ar2, D0FrT, D0.5, D0.6 and D0.7 */
	for (i = 0; i < 8; i++) {
		topaz_read_core_reg(dev_priv, 0x1 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 8 Registers of D1 Bank  */
	/* D1Re0, D1Ar5, D1Ar3, D1Ar1, D1RtP, D1.5, D1.6 and D1.7 */
	for (i = 0; i < 8; i++) {
		topaz_read_core_reg(dev_priv, 0x2 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 4 Registers of A0 Bank  */
	/* A0StP, A0FrP, A0.2 and A0.3 */
	for (i = 0; i < 4; i++) {
		topaz_read_core_reg(dev_priv, 0x3 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 4 Registers of A1 Bank  */
	/* A1GbP, A1LbP, A1.2 and A1.3 */
	for (i = 0; i < 4; i++) {
		topaz_read_core_reg(dev_priv, 0x4 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves PC and PCX  */
	for (i = 0; i < 2; i++) {
		topaz_read_core_reg(dev_priv, 0x5 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}
	/* Saves 8 Control Registers */
	/* TXSTAT, TXMASK, TXSTATI, TXMASKI, TXPOLL, TXGPIOI, TXPOLLI,
	 * TXGPIOO */
	for (i = 0; i < 8; i++) {
		topaz_read_core_reg(dev_priv, 0x7 | (i << 4), mtx_reg_state);
		mtx_reg_state++;
	}

	/* save mtx data memory */
	cur_codec_fw = &topaz_priv->topaz_fw[topaz_priv->topaz_cur_codec];

	mtx_dma_read(dev, cur_codec_fw->data_location + 0x80900000 - 0x82880000,
		     topaz_priv->cur_mtx_data_size);

	/* turn off mtx clocks */
	MTX_WRITE32(TOPAZ_CR_TOPAZ_MAN_CLK_GATE,
		    MASK_TOPAZ_CR_TOPAZ_MTX_MAN_CLK_GATE);

	topaz_priv->topaz_mtx_saved = 1;

	return 0;
}

void mtx_dma_read(struct drm_device *dev, uint32_t source_addr, uint32_t size)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct ttm_buffer_object *target;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	/* setup mtx DMAC registers to do transfer */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA, source_addr);
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(2, MTX_BURSTSIZE) |
		    F_ENCODE(1, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) | F_ENCODE(size, MTX_LENGTH));

	/* give the DMAC access to the host memory via BIF */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1);

	target = topaz_priv->topaz_mtx_data_mem;
	/* transfert the data */
	/* FIXME: size is meaured by bytes? */
	topaz_dma_transfer(dev_priv, 0, target->offset, 0,
			   MTX_CR_MTX_SYSC_CDMAT, size, 0, 1);

	/* wait for it transfer */
	topaz_wait_for_register(dev_priv, IMG_SOC_DMAC_IRQ_STAT(0) + DMAC_START,
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	/* clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);
	/* give access back to topaz core */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 0);
}

void dmac_transfer(struct drm_device *dev, uint32_t channel, uint32_t dst_addr,
		   uint32_t soc_addr, uint32_t bytes_num,
		   int increment, int rnw)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	uint32_t count_reg;
	uint32_t irq_state;

	/* check no transfer is in progress */
	DMAC_READ32(IMG_SOC_DMAC_COUNT(channel), &count_reg);
	if (0 != (count_reg & (MASK_IMG_SOC_EN | MASK_IMG_SOC_LIST_EN))) {
		DRM_ERROR("TOPAZ: there's transfer in progress when wanna "
			  "save mtx data\n");
		/* FIXME: how to handle this error */
		return;
	}

	/* no hold off period */
	DMAC_WRITE32(IMG_SOC_DMAC_PER_HOLD(channel), 0);
	/* cleare irq state */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(channel), 0);
	DMAC_READ32(IMG_SOC_DMAC_IRQ_STAT(channel), &irq_state);
	if (0 != irq_state) {
		DRM_ERROR("TOPAZ: there's irq cann't clear\n");
		return;
	}

	DMAC_WRITE32(IMG_SOC_DMAC_SETUP(channel), dst_addr);
	count_reg = DMAC_VALUE_COUNT(DMAC_BSWAP_NO_SWAP,
				     DMAC_PWIDTH_32_BIT, rnw,
				     DMAC_PWIDTH_32_BIT, bytes_num);
	/* generate an interrupt at end of transfer */
	count_reg |= MASK_IMG_SOC_TRANSFER_IEN;
	count_reg |= F_ENCODE(rnw, IMG_SOC_DIR);
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel), count_reg);

	DMAC_WRITE32(IMG_SOC_DMAC_PERIPH(channel),
		     DMAC_VALUE_PERIPH_PARAM(DMAC_ACC_DEL_0, increment,
					     DMAC_BURST_2));
	DMAC_WRITE32(IMG_SOC_DMAC_PERIPHERAL_ADDR(channel), soc_addr);

	/* Finally, rewrite the count register with the enable
	 * bit set to kick off the transfer */
	DMAC_WRITE32(IMG_SOC_DMAC_COUNT(channel), count_reg | MASK_IMG_SOC_EN);
}

void mtx_dma_write(struct drm_device *dev)
{
	struct topaz_codec_fw *cur_codec_fw;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	cur_codec_fw = &topaz_priv->topaz_fw[topaz_priv->topaz_cur_codec];

	/* upload code */
	/* setup mtx DMAC registers to recieve transfer */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA, 0x80900000);
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(2, MTX_BURSTSIZE) |
		    F_ENCODE(0, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) |
		    F_ENCODE(cur_codec_fw->text_size / 4, MTX_LENGTH));

	/* give DMAC access to host memory */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1);

	/* transfer code */
	topaz_dma_transfer(dev_priv, 0, cur_codec_fw->text->offset, 0,
			   MTX_CR_MTX_SYSC_CDMAT, cur_codec_fw->text_size / 4,
			   0, 0);
	/* wait finished */
	topaz_wait_for_register(dev_priv, IMG_SOC_DMAC_IRQ_STAT(0) + DMAC_START,
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	/* clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	/* setup mtx start recieving data */
	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAA, 0x80900000 +
		    (cur_codec_fw->data_location) - 0x82880000);

	MTX_WRITE32(MTX_CR_MTX_SYSC_CDMAC,
		    F_ENCODE(2, MTX_BURSTSIZE) |
		    F_ENCODE(0, MTX_RNW) |
		    F_ENCODE(1, MTX_ENABLE) |
		    F_ENCODE(topaz_priv->cur_mtx_data_size, MTX_LENGTH));

	/* give DMAC access to host memory */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 1);

	/* transfer data */
	topaz_dma_transfer(dev_priv, 0, topaz_priv->topaz_mtx_data_mem->offset,
			   0, MTX_CR_MTX_SYSC_CDMAT,
			   topaz_priv->cur_mtx_data_size, 0, 0);
	/* wait finished */
	topaz_wait_for_register(dev_priv, IMG_SOC_DMAC_IRQ_STAT(0) + DMAC_START,
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN),
				F_ENCODE(1, IMG_SOC_TRANSFER_FIN));
	/* clear interrupt */
	DMAC_WRITE32(IMG_SOC_DMAC_IRQ_STAT(0), 0);

	/* give access back to Topaz Core */
	TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_DMAC_MODE, 0);
}
