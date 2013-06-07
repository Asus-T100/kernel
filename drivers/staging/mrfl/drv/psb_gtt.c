/*
 * Copyright (c) 2007, Intel Corporation.
 * All Rights Reserved.
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
 * Authors: Thomas Hellstrom <thomas-at-tungstengraphics.com>
 */

#include <drm/drmP.h>
#include "psb_drv.h"
#include "pvrsrv_interface.h"
#include "display_callbacks.h"

static inline uint32_t psb_gtt_mask_pte(uint32_t pfn, int type)
{
	uint32_t mask = PSB_PTE_VALID;

	if (type & PSB_MMU_CACHED_MEMORY)
		mask |= PSB_PTE_CACHED;
	if (type & PSB_MMU_RO_MEMORY)
		mask |= PSB_PTE_RO;
	if (type & PSB_MMU_WO_MEMORY)
		mask |= PSB_PTE_WO;

	return (pfn << PAGE_SHIFT) | mask;
}

struct psb_gtt *psb_gtt_alloc(struct drm_device *dev)
{
	struct psb_gtt *tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);

	if (!tmp)
		return NULL;

	init_rwsem(&tmp->sem);
	tmp->dev = dev;

	return tmp;
}

void mrfld_gtt_takedown(struct psb_gtt *pg, int free)
{
	if (!pg)
		return;

	if (pg->gtt_map) {
		iounmap(pg->gtt_map);
		pg->gtt_map = NULL;
	}
	if (free)
		kfree(pg);
}

int mrfld_gtt_init(struct psb_gtt *pg, int resume)
{
	struct drm_device *dev = pg->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned gtt_pages;
	unsigned long stolen_size, vram_stolen_size, ci_stolen_size;
	unsigned long rar_stolen_size;
	unsigned i, num_pages;
	unsigned pfn_base;
	uint32_t ci_pages, vram_pages;
	uint32_t tt_pages;
	uint32_t *ttm_gtt_map;

	int ret = 0;
	uint32_t pte;

	pg->initialized = 1;

	pg->gatt_start = pci_resource_start(dev->pdev, PSB_GATT_RESOURCE);
	/* fix me: video mmu has hw bug to access 0x0D0000000,
	 * then make gatt start at 0x0e000,0000 */
	pg->mmu_gatt_start = PSB_MEM_TT_START;
	pg->gatt_pages = pci_resource_len(dev->pdev, PSB_GATT_RESOURCE)
	    >> PAGE_SHIFT;

	pci_read_config_dword(dev->pdev, MRFLD_BGSM, &pg->pge_ctl);
	pg->gtt_phys_start = pg->pge_ctl & PAGE_MASK;

	pci_read_config_dword(dev->pdev, MRFLD_MSAC, &gtt_pages);
	printk(KERN_INFO "01 gtt_pages = 0x%x \n", gtt_pages);
	gtt_pages &= _APERTURE_SIZE_MASK;
	gtt_pages >>= _APERTURE_SIZE_POS;

	printk(KERN_INFO "02 gtt_pages = 0x%x \n", gtt_pages);
	switch (gtt_pages) {
	case _1G_APERTURE:
		gtt_pages = _1G_APERTURE_SIZE >> PAGE_SHIFT;
		break;
	case _512M_APERTURE:
		gtt_pages = _512M_APERTURE_SIZE >> PAGE_SHIFT;
		break;
	case _256M_APERTURE:
		gtt_pages = _256M_APERTURE_SIZE >> PAGE_SHIFT;
		break;
	default:
		DRM_ERROR("%s, invalded aperture size.\n", __func__);
		gtt_pages = _1G_APERTURE_SIZE >> PAGE_SHIFT;
	}

	gtt_pages >>= PAGE_SHIFT;
	gtt_pages *= 4;

	printk(KERN_INFO "03 gtt_pages = 0x%x \n", gtt_pages);
	/* HW removed the PSB_BSM, SW/FW needs it. */
	pci_read_config_dword(dev->pdev, PSB_BSM, &pg->stolen_base);
	vram_stolen_size = pg->gtt_phys_start - pg->stolen_base - PAGE_SIZE;

	/* CI is not included in the stolen size since the TOPAZ MMU bug */
	ci_stolen_size = dev_priv->ci_region_size;
	/* Don't add CI & RAR share buffer space
	 * managed by TTM to stolen_size */
	stolen_size = vram_stolen_size;

	rar_stolen_size = dev_priv->rar_region_size;

	printk(KERN_INFO "GMMADR(region 0) start: 0x%08x (%dM).\n",
	       pg->gatt_start, pg->gatt_pages / 256);
	printk(KERN_INFO "GTT (can map %dM RAM), and actual RAM base 0x%08x.\n",
	       gtt_pages * 4, pg->gtt_phys_start);
	printk(KERN_INFO "Stole memory information \n");
	printk(KERN_INFO "      base in RAM: 0x%x \n", pg->stolen_base);
	printk(KERN_INFO
	       "      size: %luK, calculated by (GTT RAM base) - (Stolen base).\n",
	       vram_stolen_size / 1024);

	if (ci_stolen_size > 0)
		printk(KERN_INFO
		       "CI Stole memory: RAM base = 0x%08x, size = %lu M \n",
		       dev_priv->ci_region_start, ci_stolen_size / 1024 / 1024);
	if (rar_stolen_size > 0)
		printk(KERN_INFO
		       "RAR Stole memory: RAM base = 0x%08x, size = %lu M \n",
		       dev_priv->rar_region_start,
		       rar_stolen_size / 1024 / 1024);

	if (resume && (gtt_pages != pg->gtt_pages) &&
	    (stolen_size != pg->stolen_size)) {
		DRM_ERROR("GTT resume error.\n");
		ret = -EINVAL;
		goto out_err;
	}

	pg->gtt_pages = gtt_pages;
	pg->stolen_size = stolen_size;
	pg->vram_stolen_size = vram_stolen_size;
	pg->ci_stolen_size = ci_stolen_size;
	pg->rar_stolen_size = rar_stolen_size;
	pg->gtt_map =
	    ioremap_nocache(pg->gtt_phys_start, gtt_pages << PAGE_SHIFT);
	if (!pg->gtt_map) {
		DRM_ERROR("Failure to map gtt.\n");
		ret = -ENOMEM;
		goto out_err;
	}

	pg->vram_addr = ioremap_wc(pg->stolen_base, stolen_size);
	if (!pg->vram_addr) {
		DRM_ERROR("Failure to map stolen base.\n");
		ret = -ENOMEM;
		goto out_err;
	}

	DRM_INFO("%s: vram kernel virtual address %p\n", __FUNCTION__,
		 pg->vram_addr);

	tt_pages = (pg->gatt_pages < PSB_TT_PRIV0_PLIMIT) ?
	    (pg->gatt_pages) : PSB_TT_PRIV0_PLIMIT;

	ttm_gtt_map = pg->gtt_map + tt_pages / 2;

	/*
	 * insert vram stolen pages.
	 */

	pfn_base = pg->stolen_base >> PAGE_SHIFT;
	vram_pages = num_pages = vram_stolen_size >> PAGE_SHIFT;
	printk(KERN_INFO
	       "Set up %d stolen pages starting at 0x%08x, GTT offset %dK\n",
	       num_pages, pfn_base, 0);
	for (i = 0; i < num_pages; ++i) {
		pte = psb_gtt_mask_pte(pfn_base + i, 0);
		iowrite32(pte, pg->gtt_map + i);
	}

	/*
	 * Init rest of gtt managed by IMG.
	 */
	pfn_base = page_to_pfn(dev_priv->scratch_page);
	pte = psb_gtt_mask_pte(pfn_base, 0);
	for (; i < tt_pages / 2 - 1; ++i)
		iowrite32(pte, pg->gtt_map + i);

	/*
	 * Init rest of gtt managed by TTM.
	 */

	pfn_base = page_to_pfn(dev_priv->scratch_page);
	pte = psb_gtt_mask_pte(pfn_base, 0);
	printk(KERN_INFO
	       "Initializing the rest of a total "
	       "of %d gtt pages.\n", pg->gatt_pages);

	for (i = 0; i < pg->gatt_pages - tt_pages / 2; ++i)
		iowrite32(pte, ttm_gtt_map + i);
	(void)ioread32(pg->gtt_map + i - 1);

	return 0;

 out_err:
	mrfld_gtt_takedown(pg, 0);
	return ret;
}

void psb_gtt_takedown(struct psb_gtt *pg, int free)
{
	struct drm_psb_private *dev_priv = pg->dev->dev_private;

	if (!pg)
		return;

	if (pg->gtt_map) {
		iounmap(pg->gtt_map);
		pg->gtt_map = NULL;
	}
	if (pg->initialized) {
		pci_write_config_word(pg->dev->pdev, PSB_GMCH_CTRL,
				      pg->gmch_ctrl);
		PSB_WVDC32(pg->pge_ctl, PSB_PGETBL_CTL);
		(void)PSB_RVDC32(PSB_PGETBL_CTL);
	}
	if (free)
		kfree(pg);
}

int psb_gtt_init(struct psb_gtt *pg, int resume)
{
	struct drm_device *dev = pg->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned gtt_pages;
	unsigned long stolen_size, vram_stolen_size, ci_stolen_size;
	unsigned long rar_stolen_size;
	unsigned i, num_pages;
	unsigned pfn_base;
	uint32_t ci_pages, vram_pages;
	uint32_t tt_pages;
	uint32_t *ttm_gtt_map;
	uint32_t dvmt_mode = 0;

	int ret = 0;
	uint32_t pte;

	pci_read_config_word(dev->pdev, PSB_GMCH_CTRL, &pg->gmch_ctrl);
	pci_write_config_word(dev->pdev, PSB_GMCH_CTRL,
			      pg->gmch_ctrl | _PSB_GMCH_ENABLED);

	pg->pge_ctl = PSB_RVDC32(PSB_PGETBL_CTL);
	PSB_WVDC32(pg->pge_ctl | _PSB_PGETBL_ENABLED, PSB_PGETBL_CTL);
	(void)PSB_RVDC32(PSB_PGETBL_CTL);

	pg->initialized = 1;

	pg->gtt_phys_start = pg->pge_ctl & PAGE_MASK;

	pg->gatt_start = pci_resource_start(dev->pdev, PSB_GATT_RESOURCE);
	/* fix me: video mmu has hw bug to access 0x0D0000000,
	 * then make gatt start at 0x0e000,0000 */
	pg->mmu_gatt_start = PSB_MEM_TT_START;
	pg->gtt_start = pci_resource_start(dev->pdev, PSB_GTT_RESOURCE);
	gtt_pages = pci_resource_len(dev->pdev, PSB_GTT_RESOURCE) >> PAGE_SHIFT;
	pg->gatt_pages = pci_resource_len(dev->pdev, PSB_GATT_RESOURCE)
	    >> PAGE_SHIFT;

	pci_read_config_dword(dev->pdev, PSB_BSM, &pg->stolen_base);
	vram_stolen_size = pg->gtt_phys_start - pg->stolen_base - PAGE_SIZE;

	/* CI is not included in the stolen size since the TOPAZ MMU bug */
	ci_stolen_size = dev_priv->ci_region_size;
	/* Don't add CI & RAR share buffer space
	 * managed by TTM to stolen_size */
	stolen_size = vram_stolen_size;

	rar_stolen_size = dev_priv->rar_region_size;

	printk(KERN_INFO "GMMADR(region 0) start: 0x%08x (%dM).\n",
	       pg->gatt_start, pg->gatt_pages / 256);
	printk(KERN_INFO
	       "GTTADR(region 3) start: 0x%08x (can map %dM RAM), and actual RAM base 0x%08x.\n",
	       pg->gtt_start, gtt_pages * 4, pg->gtt_phys_start);
	printk(KERN_INFO "Stole memory information \n");
	printk(KERN_INFO "      base in RAM: 0x%x \n", pg->stolen_base);
	printk(KERN_INFO
	       "      size: %luK, calculated by (GTT RAM base) - (Stolen base), seems wrong\n",
	       vram_stolen_size / 1024);
	dvmt_mode = (pg->gmch_ctrl >> 4) & 0x7;
	printk(KERN_INFO
	       "      the correct size should be: %dM(dvmt mode=%d) \n",
	       (dvmt_mode == 1) ? 1 : (2 << (dvmt_mode - 1)), dvmt_mode);

	if (ci_stolen_size > 0)
		printk(KERN_INFO
		       "CI Stole memory: RAM base = 0x%08x, size = %lu M \n",
		       dev_priv->ci_region_start, ci_stolen_size / 1024 / 1024);
	if (rar_stolen_size > 0)
		printk(KERN_INFO
		       "RAR Stole memory: RAM base = 0x%08x, size = %lu M \n",
		       dev_priv->rar_region_start,
		       rar_stolen_size / 1024 / 1024);

	if (resume && (gtt_pages != pg->gtt_pages) &&
	    (stolen_size != pg->stolen_size)) {
		DRM_ERROR("GTT resume error.\n");
		ret = -EINVAL;
		goto out_err;
	}

	pg->gtt_pages = gtt_pages;
	pg->stolen_size = stolen_size;
	pg->vram_stolen_size = vram_stolen_size;
	pg->ci_stolen_size = ci_stolen_size;
	pg->rar_stolen_size = rar_stolen_size;
	pg->gtt_map =
	    ioremap_nocache(pg->gtt_phys_start, gtt_pages << PAGE_SHIFT);
	if (!pg->gtt_map) {
		DRM_ERROR("Failure to map gtt.\n");
		ret = -ENOMEM;
		goto out_err;
	}

	pg->vram_addr = ioremap_wc(pg->stolen_base, stolen_size);
	if (!pg->vram_addr) {
		DRM_ERROR("Failure to map stolen base.\n");
		ret = -ENOMEM;
		goto out_err;
	}

	DRM_INFO("%s: vram kernel virtual address %p\n", __FUNCTION__,
		 pg->vram_addr);

	tt_pages = (pg->gatt_pages < PSB_TT_PRIV0_PLIMIT) ?
	    (pg->gatt_pages) : PSB_TT_PRIV0_PLIMIT;

	ttm_gtt_map = pg->gtt_map + tt_pages / 2;

	/*
	 * insert vram stolen pages.
	 */

	pfn_base = pg->stolen_base >> PAGE_SHIFT;
	vram_pages = num_pages = vram_stolen_size >> PAGE_SHIFT;
	printk(KERN_INFO
	       "Set up %d stolen pages starting at 0x%08x, GTT offset %dK\n",
	       num_pages, pfn_base, 0);
	for (i = 0; i < num_pages; ++i) {
		pte = psb_gtt_mask_pte(pfn_base + i, 0);
		iowrite32(pte, pg->gtt_map + i);
	}

	/*
	 * Init rest of gtt managed by IMG.
	 */
	pfn_base = page_to_pfn(dev_priv->scratch_page);
	pte = psb_gtt_mask_pte(pfn_base, 0);
	for (; i < tt_pages / 2 - 1; ++i)
		iowrite32(pte, pg->gtt_map + i);

	/*
	 * Init rest of gtt managed by TTM.
	 */

	pfn_base = page_to_pfn(dev_priv->scratch_page);
	pte = psb_gtt_mask_pte(pfn_base, 0);
	PSB_DEBUG_INIT("Initializing the rest of a total "
		       "of %d gtt pages.\n", pg->gatt_pages);

	for (i = 0; i < pg->gatt_pages - tt_pages / 2; ++i)
		iowrite32(pte, ttm_gtt_map + i);
	(void)ioread32(pg->gtt_map + i - 1);

	return 0;

 out_err:
	psb_gtt_takedown(pg, 0);
	return ret;
}

/*********************************
 *  Added  (imported from 'mrst')  because memory allocated by PVRSRV
 *      won't succeed using 'get_user_pages' in PVRSRVGetMeminfoPages
 *      So, we will follow pfn's obtained by PVRSRVGetMeminfoPfn.
 *
 *  williamx.f.schmidt@intel.com
 */
static int psb_gtt_insert_pfn_list(struct psb_gtt *pg, unsigned long * pfn_list,
				   unsigned offset_pages, unsigned num_pages,
				   unsigned desired_tile_stride,
				   unsigned hw_tile_stride, int type)
{
	unsigned rows = 1;
	unsigned add;
	unsigned row_add;
	unsigned i;
	unsigned j;
	uint32_t *cur_page = NULL;
	uint32_t pte;

	if (!pg || !pfn_list)
		return -EINVAL;

	if (hw_tile_stride)
		rows = num_pages / desired_tile_stride;
	else
		desired_tile_stride = num_pages;

	add = desired_tile_stride;
	row_add = hw_tile_stride;

	down_read(&pg->sem);
	for (i = 0; i < rows; ++i) {
		cur_page = pg->gtt_map + offset_pages;
		for (j = 0; j < desired_tile_stride; ++j) {
			pte = psb_gtt_mask_pte(*pfn_list++, type);
			iowrite32(pte, cur_page++);
		}
		offset_pages += add;
	}
	(void)ioread32(cur_page - 1);
	up_read(&pg->sem);

	return 0;
}

int psb_gtt_insert_pages(struct psb_gtt *pg, struct page **pages,
			 unsigned offset_pages, unsigned num_pages,
			 unsigned desired_tile_stride,
			 unsigned hw_tile_stride, int type)
{
	unsigned rows = 1;
	unsigned add;
	unsigned row_add;
	unsigned i;
	unsigned j;
	uint32_t *cur_page = NULL;
	uint32_t pte;

	if (hw_tile_stride)
		rows = num_pages / desired_tile_stride;
	else
		desired_tile_stride = num_pages;

	add = desired_tile_stride;
	row_add = hw_tile_stride;

	down_read(&pg->sem);
	for (i = 0; i < rows; ++i) {
		cur_page = pg->gtt_map + offset_pages;
		for (j = 0; j < desired_tile_stride; ++j) {
			pte = psb_gtt_mask_pte(page_to_pfn(*pages++), type);
			iowrite32(pte, cur_page++);
		}
		offset_pages += add;
	}
	(void)ioread32(cur_page - 1);
	up_read(&pg->sem);

	return 0;
}

int psb_gtt_insert_phys_addresses(struct psb_gtt *pg, uintptr_t * pPhysFrames,
				  unsigned offset_pages, unsigned num_pages,
				  int type)
{
	unsigned j;
	uint32_t *cur_page = NULL;
	uint32_t pte;

	//printk("Allocatng IMG GTT mem at %x (pages %d)\n",offset_pages,num_pages);
	down_read(&pg->sem);

	cur_page = pg->gtt_map + offset_pages;
	for (j = 0; j < num_pages; ++j) {
		pte = psb_gtt_mask_pte(*(pPhysFrames++) >> PAGE_SHIFT, type);
		iowrite32(pte, cur_page++);
		//printk("PTE %d: %x/%x\n",j,(pPhysFrames-1)->uiAddr,pte);
	}
	(void)ioread32(cur_page - 1);

	up_read(&pg->sem);

	return 0;
}

int psb_gtt_remove_pages(struct psb_gtt *pg, unsigned offset_pages,
			 unsigned num_pages, unsigned desired_tile_stride,
			 unsigned hw_tile_stride, int rc_prot)
{
	struct drm_psb_private *dev_priv = pg->dev->dev_private;
	unsigned rows = 1;
	unsigned add;
	unsigned row_add;
	unsigned i;
	unsigned j;
	uint32_t *cur_page = NULL;
	unsigned pfn_base = page_to_pfn(dev_priv->scratch_page);
	uint32_t pte = psb_gtt_mask_pte(pfn_base, 0);

	if (hw_tile_stride)
		rows = num_pages / desired_tile_stride;
	else
		desired_tile_stride = num_pages;

	add = desired_tile_stride;
	row_add = hw_tile_stride;

	if (rc_prot)
		down_read(&pg->sem);
	for (i = 0; i < rows; ++i) {
		cur_page = pg->gtt_map + offset_pages;
		for (j = 0; j < desired_tile_stride; ++j)
			iowrite32(pte, cur_page++);

		offset_pages += add;
	}
	(void)ioread32(cur_page - 1);
	if (rc_prot)
		up_read(&pg->sem);

	return 0;
}

int psb_gtt_mm_init(struct psb_gtt *pg)
{
	struct psb_gtt_mm *gtt_mm;
	struct drm_psb_private *dev_priv = pg->dev->dev_private;
	struct drm_open_hash *ht;
	struct drm_mm *mm;
	int ret;
	uint32_t tt_start;
	uint32_t tt_size;

	if (!pg || !pg->initialized) {
		DRM_DEBUG("Invalid gtt struct\n");
		return -EINVAL;
	}

	gtt_mm = kzalloc(sizeof(struct psb_gtt_mm), GFP_KERNEL);
	if (!gtt_mm)
		return -ENOMEM;

	spin_lock_init(&gtt_mm->lock);

	ht = &gtt_mm->hash;
	ret = drm_ht_create(ht, 20);
	if (ret) {
		DRM_DEBUG("Create hash table failed(%d)\n", ret);
		goto err_free;
	}

	tt_start = (pg->stolen_size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	tt_start = (tt_start < pg->gatt_pages) ? tt_start : pg->gatt_pages;
	tt_size = (pg->gatt_pages < PSB_TT_PRIV0_PLIMIT) ?
	    (pg->gatt_pages) : PSB_TT_PRIV0_PLIMIT;

	mm = &gtt_mm->base;

	/*will use tt_start ~ 128M for IMG TT buffers */
	ret = drm_mm_init(mm, tt_start, ((tt_size / 2) - tt_start));
	if (ret) {
		DRM_DEBUG("drm_mm_int error(%d)\n", ret);
		goto err_mm_init;
	}

	gtt_mm->count = 0;

	dev_priv->gtt_mm = gtt_mm;

	DRM_INFO("PSB GTT mem manager ready, tt_start %ld, tt_size %ld pages\n",
		 (unsigned long)tt_start,
		 (unsigned long)((tt_size / 2) - tt_start));
	return 0;
 err_mm_init:
	drm_ht_remove(ht);

 err_free:
	kfree(gtt_mm);
	return ret;
}

/**
 * Delete all hash entries;
 */
void psb_gtt_mm_takedown(void)
{
	return;
}

static int psb_gtt_mm_get_ht_by_pid_locked(struct psb_gtt_mm *mm,
					   u32 tgid,
					   struct psb_gtt_hash_entry **hentry)
{
	struct drm_hash_item *entry;
	struct psb_gtt_hash_entry *psb_entry;
	int ret;

	ret = drm_ht_find_item(&mm->hash, tgid, &entry);
	if (ret) {
		DRM_DEBUG("Cannot find entry pid=%d\n", tgid);
		return ret;
	}

	psb_entry = container_of(entry, struct psb_gtt_hash_entry, item);
	if (!psb_entry) {
		DRM_DEBUG("Invalid entry");
		return -EINVAL;
	}

	*hentry = psb_entry;
	return 0;
}

static int psb_gtt_mm_insert_ht_locked(struct psb_gtt_mm *mm,
				       u32 tgid,
				       struct psb_gtt_hash_entry *hentry)
{
	struct drm_hash_item *item;
	int ret;

	if (!hentry) {
		DRM_DEBUG("Invalid parameters\n");
		return -EINVAL;
	}

	item = &hentry->item;
	item->key = tgid;

	/**
	 * NOTE: drm_ht_insert_item will perform such a check
	ret = psb_gtt_mm_get_ht_by_pid(mm, tgid, &tmp);
	if (!ret) {
		DRM_DEBUG("Entry already exists for pid %ld\n", tgid);
		return -EAGAIN;
	}
	*/

	/*Insert the given entry */
	ret = drm_ht_insert_item(&mm->hash, item);
	if (ret) {
		DRM_DEBUG("Insert failure\n");
		return ret;
	}

	mm->count++;

	return 0;
}

static int psb_gtt_mm_alloc_insert_ht(struct psb_gtt_mm *mm,
				      u32 tgid,
				      struct psb_gtt_hash_entry **entry)
{
	struct psb_gtt_hash_entry *hentry;
	int ret;

	/*if the hentry for this tgid exists, just get it and return */
	spin_lock(&mm->lock);
	ret = psb_gtt_mm_get_ht_by_pid_locked(mm, tgid, &hentry);
	if (!ret) {
		DRM_DEBUG("Entry for tgid %d exist, hentry %p\n",
			  tgid, hentry);
		*entry = hentry;
		spin_unlock(&mm->lock);
		return 0;
	}
	spin_unlock(&mm->lock);

	DRM_DEBUG("Entry for tgid %d doesn't exist, will create it\n", tgid);

	hentry = kzalloc(sizeof(struct psb_gtt_hash_entry), GFP_KERNEL);
	if (!hentry) {
		DRM_DEBUG("Kmalloc failled\n");
		return -ENOMEM;
	}

	ret = drm_ht_create(&hentry->ht, 20);
	if (ret) {
		DRM_DEBUG("Create hash table failed\n");
		return ret;
	}

	spin_lock(&mm->lock);
	ret = psb_gtt_mm_insert_ht_locked(mm, tgid, hentry);
	spin_unlock(&mm->lock);

	if (!ret)
		*entry = hentry;

	return ret;
}

static struct psb_gtt_hash_entry *psb_gtt_mm_remove_ht_locked(struct psb_gtt_mm
							      *mm, u32 tgid)
{
	struct psb_gtt_hash_entry *tmp;
	int ret;

	ret = psb_gtt_mm_get_ht_by_pid_locked(mm, tgid, &tmp);
	if (ret) {
		DRM_DEBUG("Cannot find entry pid %d\n", tgid);
		return NULL;
	}

	/*remove it from ht */
	drm_ht_remove_item(&mm->hash, &tmp->item);

	mm->count--;

	return tmp;
}

static int psb_gtt_mm_remove_free_ht_locked(struct psb_gtt_mm *mm, u32 tgid)
{
	struct psb_gtt_hash_entry *entry;

	entry = psb_gtt_mm_remove_ht_locked(mm, tgid);

	if (!entry) {
		DRM_DEBUG("Invalid entry");
		return -EINVAL;
	}

	/*delete ht */
	drm_ht_remove(&entry->ht);

	/*free this entry */
	kfree(entry);
	return 0;
}

static int
psb_gtt_mm_get_mem_mapping_locked(struct drm_open_hash *ht,
				  u32 key, struct psb_gtt_mem_mapping **hentry)
{
	struct drm_hash_item *entry;
	struct psb_gtt_mem_mapping *mapping;
	int ret;

	ret = drm_ht_find_item(ht, key, &entry);
	if (ret) {
		DRM_DEBUG("Cannot find key %d\n", key);
		return ret;
	}

	mapping = container_of(entry, struct psb_gtt_mem_mapping, item);
	if (!mapping) {
		DRM_DEBUG("Invalid entry\n");
		return -EINVAL;
	}

	*hentry = mapping;
	return 0;
}

static int
psb_gtt_mm_insert_mem_mapping_locked(struct drm_open_hash *ht,
				     u32 key,
				     struct psb_gtt_mem_mapping *hentry)
{
	struct drm_hash_item *item;
	struct psb_gtt_hash_entry *entry;
	int ret;

	if (!hentry || !ht) {
		DRM_DEBUG("parameter error\n");
		return -EINVAL;
	}

	item = &hentry->item;
	item->key = key;

	ret = drm_ht_insert_item(ht, item);
	if (ret) {
		DRM_DEBUG("insert_item failed\n");
		return ret;
	}

	entry = container_of(ht, struct psb_gtt_hash_entry, ht);
	if (entry)
		entry->count++;

	return 0;
}

static int
psb_gtt_mm_alloc_insert_mem_mapping(struct psb_gtt_mm *mm,
				    struct drm_open_hash *ht,
				    u32 key,
				    struct drm_mm_node *node,
				    struct psb_gtt_mem_mapping **entry)
{
	struct psb_gtt_mem_mapping *mapping;
	int ret;

	if (!node || !ht) {
		DRM_DEBUG("parameter error\n");
		return -EINVAL;
	}

	/*try to get this mem_map */
	spin_lock(&mm->lock);
	ret = psb_gtt_mm_get_mem_mapping_locked(ht, key, &mapping);
	if (!ret) {
		DRM_DEBUG("mapping entry for key %d exists, entry %p\n",
			  key, mapping);
		*entry = mapping;
		spin_unlock(&mm->lock);
		return 0;
	}
	spin_unlock(&mm->lock);

	DRM_DEBUG("Mapping entry for key %d doesn't exist, will create it\n",
		  key);

	mapping = kzalloc(sizeof(struct psb_gtt_mem_mapping), GFP_KERNEL);
	if (!mapping) {
		DRM_DEBUG("kmalloc failed\n");
		return -ENOMEM;
	}

	mapping->node = node;

	spin_lock(&mm->lock);
	ret = psb_gtt_mm_insert_mem_mapping_locked(ht, key, mapping);
	spin_unlock(&mm->lock);

	if (!ret)
		*entry = mapping;

	return ret;
}

static struct psb_gtt_mem_mapping *psb_gtt_mm_remove_mem_mapping_locked(struct
									drm_open_hash
									*ht,
									u32 key)
{
	struct psb_gtt_mem_mapping *tmp;
	struct psb_gtt_hash_entry *entry;
	int ret;

	if (!ht) {
		DRM_DEBUG("ht is NULL\n");
		return NULL;
	}

	ret = psb_gtt_mm_get_mem_mapping_locked(ht, key, &tmp);
	if (ret) {
		DRM_DEBUG("Cannot find key %d\n", key);
		return NULL;
	}

	drm_ht_remove_item(ht, &tmp->item);

	entry = container_of(ht, struct psb_gtt_hash_entry, ht);
	if (entry)
		entry->count--;

	return tmp;
}

static int psb_gtt_mm_remove_free_mem_mapping_locked(struct drm_open_hash *ht,
						     u32 key,
						     struct drm_mm_node **node)
{
	struct psb_gtt_mem_mapping *entry;

	entry = psb_gtt_mm_remove_mem_mapping_locked(ht, key);
	if (!entry) {
		DRM_DEBUG("entry is NULL\n");
		return -EINVAL;
	}

	*node = entry->node;

	kfree(entry);
	return 0;
}

static int psb_gtt_add_node(struct psb_gtt_mm *mm,
			    u32 tgid,
			    u32 key,
			    struct drm_mm_node *node,
			    struct psb_gtt_mem_mapping **entry)
{
	struct psb_gtt_hash_entry *hentry;
	struct psb_gtt_mem_mapping *mapping;
	int ret;

	ret = psb_gtt_mm_alloc_insert_ht(mm, tgid, &hentry);
	if (ret) {
		DRM_DEBUG("alloc_insert failed\n");
		return ret;
	}

	ret = psb_gtt_mm_alloc_insert_mem_mapping(mm,
						  &hentry->ht,
						  key, node, &mapping);
	if (ret) {
		DRM_DEBUG("mapping alloc_insert failed\n");
		return ret;
	}

	*entry = mapping;

	return 0;
}

static int psb_gtt_remove_node(struct psb_gtt_mm *mm,
			       u32 tgid, u32 key, struct drm_mm_node **node)
{
	struct psb_gtt_hash_entry *hentry;
	struct drm_mm_node *tmp;
	int ret;

	spin_lock(&mm->lock);
	ret = psb_gtt_mm_get_ht_by_pid_locked(mm, tgid, &hentry);
	if (ret) {
		DRM_DEBUG("Cannot find entry for pid %d\n", tgid);
		spin_unlock(&mm->lock);
		return ret;
	}
	spin_unlock(&mm->lock);

	/*remove mapping entry */
	spin_lock(&mm->lock);
	ret = psb_gtt_mm_remove_free_mem_mapping_locked(&hentry->ht, key, &tmp);
	if (ret) {
		DRM_DEBUG("remove_free failed\n");
		spin_unlock(&mm->lock);
		return ret;
	}

	*node = tmp;

	/*check the count of mapping entry */
	if (!hentry->count) {
		DRM_DEBUG("count of mapping entry is zero, tgid=%d\n", tgid);
		psb_gtt_mm_remove_free_ht_locked(mm, tgid);
	}

	spin_unlock(&mm->lock);

	return 0;
}

static int psb_gtt_mm_alloc_mem(struct psb_gtt_mm *mm,
				uint32_t pages,
				uint32_t align, struct drm_mm_node **node)
{
	struct drm_mm_node *tmp_node;
	int ret;

	do {
		ret = drm_mm_pre_get(&mm->base);
		if (unlikely(ret)) {
			DRM_DEBUG("drm_mm_pre_get error\n");
			return ret;
		}

		spin_lock(&mm->lock);
		tmp_node = drm_mm_search_free(&mm->base, pages, align, 1);
		if (unlikely(!tmp_node)) {
			DRM_DEBUG("No free node found\n");
			spin_unlock(&mm->lock);
			break;
		}

		tmp_node = drm_mm_get_block_atomic(tmp_node, pages, align);
		spin_unlock(&mm->lock);
	} while (!tmp_node);

	if (!tmp_node) {
		DRM_DEBUG("Node allocation failed\n");
		return -ENOMEM;
	}

	*node = tmp_node;
	return 0;
}

static void psb_gtt_mm_free_mem(struct psb_gtt_mm *mm, struct drm_mm_node *node)
{
	spin_lock(&mm->lock);
	drm_mm_put_block(node);
	spin_unlock(&mm->lock);
}

static u32 gtt_get_tgid(void)
{
	if (in_interrupt()) {
		return -1;
	}

	return task_tgid_nr(current);
}

int psb_gtt_map_meminfo(struct drm_device *dev,
			void *hKernelMemInfo,
			uint32_t page_align, uint32_t * offset)
{
	struct drm_psb_private *dev_priv;
	struct psb_gtt_mm *mm;
	struct psb_gtt *pg;
	uint32_t size, pages, offset_pages;
	void *kmem;
	struct drm_mm_node *node;
	struct page **page_list;
	unsigned long *pfn_list;
	struct psb_gtt_mem_mapping *mapping;
	int ret;

	/*
	 *  Initialize locals then get  the allocation size and calculate
	 *  the number of pages that this requires. To do this we use the
	 *  interface routines provided in pvr_drm.c, a module.
	 */
	dev_priv = (struct drm_psb_private *)dev->dev_private;
	mm = dev_priv->gtt_mm;
	pg = dev_priv->pg;
	mapping = NULL;

	size = PVRSRVGetMeminfoSize(hKernelMemInfo);
	kmem = PVRSRVGetMeminfoCPUAddr(hKernelMemInfo);
	pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	/*
	 *  Alloc memory in TT apeture and update the GTT mm...
	 */
	if ((ret = psb_gtt_mm_alloc_mem(mm, pages, page_align, &node)) != 0) {
		DRM_DEBUG("alloc TT memory error\n");

		return ret;
	}

	if ((ret = psb_gtt_add_node(mm,
				    (u32) gtt_get_tgid(),
				    (u32) hKernelMemInfo,
				    node, &mapping)) != 0) {
		DRM_DEBUG("add_node failed");

		psb_gtt_mm_free_mem(mm, node);
		return ret;
	}

	node = mapping->node;
	offset_pages = node->start;

	/*
	 *  Memory represented by  'hKernelMemInfo'  should either map by
	 *  page records, or by pfn.  It will map user allocated memory -
	 *  i.e., with malloc - by pages. PVRSRV allocations from general
	 *  heap map via their pfn's.  Insert items that map into the gtt
	 *  page table.
	 */
	if (PVRSRVGetMeminfoPages(hKernelMemInfo, pages, &page_list) == 0) {	/* Works with user space 'malloc' */
		psb_gtt_insert_pages(pg, page_list,
				     (unsigned)offset_pages,
				     (unsigned)pages, 0, 0, 0);
		kfree(page_list);
	} else if ((ret = PVRSRVGetMeminfoPfn(hKernelMemInfo, pages, &pfn_list)) == 0) {	/* Works with 'PVRSRVAllocDeviceMemMIW' */
		psb_gtt_insert_pfn_list(pg, pfn_list,
					(unsigned)offset_pages,
					(unsigned)pages, 0, 0, 0);
		kfree(pfn_list);
	} else {
		psb_gtt_remove_node(mm,
				    (u32) gtt_get_tgid(),
				    (u32) hKernelMemInfo, &node);
		psb_gtt_mm_free_mem(mm, node);

		return ret;
	}

	*offset = offset_pages;
	return 0;
}				/* psb_gtt_map_meminfo */

int psb_gtt_unmap_meminfo(struct drm_device *dev, void *hKernelMemInfo)
{
	struct drm_psb_private *dev_priv
	    = (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt_mm *mm = dev_priv->gtt_mm;
	struct psb_gtt *pg = dev_priv->pg;
	uint32_t pages, offset_pages;
	struct drm_mm_node *node;
	int ret;

	ret = psb_gtt_remove_node(mm,
				  (u32) gtt_get_tgid(),
				  (u32) hKernelMemInfo, &node);
	if (ret) {
		DRM_DEBUG("remove node failed\n");
		return ret;
	}

	/*remove gtt entries */
	offset_pages = node->start;
	pages = node->size;

	psb_gtt_remove_pages(pg, offset_pages, pages, 0, 0, 1);

	/*free tt node */

	psb_gtt_mm_free_mem(mm, node);
	return 0;
}

static int psb_gtt_unmap_common(struct drm_device *dev,
			unsigned int ui32TaskId,
			unsigned int hHandle)
{
	struct drm_psb_private *dev_priv
	= (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt_mm *mm = dev_priv->gtt_mm;
	struct psb_gtt *pg = dev_priv->pg;
	uint32_t pages, offset_pages;
	struct drm_mm_node *node;
	int ret;

	ret = psb_gtt_remove_node(mm,
				  (u32)ui32TaskId,
				  (u32)hHandle,
				  &node);
	if (ret) {
		DRM_DEBUG("remove node failed\n");
		return ret;
	}

	/*remove gtt entries*/
	offset_pages = node->start;
	pages = node->size;

	psb_gtt_remove_pages(pg, offset_pages, pages, 0, 0, 1);


	/*free tt node*/

	psb_gtt_mm_free_mem(mm, node);
	return 0;

}

static int psb_get_vaddr_pages(u32 vaddr, u32 size,
				unsigned long **pfn_list, int *page_count)
{
	u32 num_pages;
	struct page **pages = 0;
	struct task_struct *task = current;
	struct mm_struct *mm = task->mm;
	struct vm_area_struct *vma;
	unsigned long *pfns = 0;
	int ret;
	int i;

	if (unlikely(!pfn_list || !page_count || !vaddr || !size))
		return -EINVAL;

	num_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	pages = kzalloc(num_pages * sizeof(struct page *), GFP_KERNEL);
	if (unlikely(!pages)) {
		DRM_ERROR("Failed to allocate page list\n");
		return -ENOMEM;
	}

	down_read(&mm->mmap_sem);
	ret = get_user_pages(task, mm, vaddr, num_pages, 0, 0, pages, NULL);
	up_read(&mm->mmap_sem);

	if (ret <= 0) {
		DRM_DEBUG("failed to get user pages\n");
		kfree(pages);
		pages = 0;
	} else {
		DRM_ERROR("num_pages %d, ret %d\n", num_pages, ret);
		num_pages = ret;
	}

	/*allocate page list*/
	pfns = kzalloc(num_pages * sizeof(u32), GFP_KERNEL);
	if (!pfns) {
		DRM_ERROR("No memory\n");
		goto get_page_err;
	}

	if (!pages) {
		DRM_DEBUG("No pages found, trying to follow pfn\n");
		for (i = 0; i < num_pages; i++) {
			vma = find_vma(mm, vaddr + i * PAGE_SIZE);
			if (!vma) {
				DRM_ERROR("failed to find vma\n");
				goto find_vma_err;
			}

			ret = follow_pfn(vma,
				(unsigned long)(vaddr + i * PAGE_SIZE),
				&pfns[i]);
			if (ret) {
				DRM_ERROR("failed to follow pfn\n");
				goto follow_pfn_err;
			}
		}
	} else {
		DRM_DEBUG("Found pages\n");
		for (i = 0; i < num_pages; i++)
			pfns[i] = page_to_pfn(pages[i]);
	}

	*pfn_list = pfns;
	*page_count = num_pages;

	kfree(pages);

	return 0;
find_vma_err:
follow_pfn_err:
	kfree(pfns);
get_page_err:
	if (pages) {
		for (i = 0; i < num_pages; i++)
			put_page(pages[i]);
		kfree(pages);
	}
	return -EINVAL;
}

static int psb_gtt_map_vaddr(struct drm_device *dev,
			uint32_t vaddr,
			uint32_t size,
			uint32_t page_align,
			uint32_t *offset)
{
	struct drm_psb_private *dev_priv
		= (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt_mm *mm = dev_priv->gtt_mm;
	struct psb_gtt *pg = dev_priv->pg;
	uint32_t pages, offset_pages;
	struct drm_mm_node *node;
	unsigned long *pfn_list = 0;
	struct psb_gtt_mem_mapping *mapping = NULL;
	int ret;

	/*get pages*/
	ret = psb_get_vaddr_pages(vaddr, size, &pfn_list, &pages);
	if (ret) {
		DRM_DEBUG("get pages error\n");
		return ret;
	}

	DRM_DEBUG("get %d pages\n", pages);

	/*alloc memory in TT apeture*/
	ret = psb_gtt_mm_alloc_mem(mm, pages, page_align, &node);
	if (ret) {
		DRM_DEBUG("alloc TT memory error\n");
		goto failed_pages_alloc;
	}

	/*update psb_gtt_mm*/
	ret = psb_gtt_add_node(mm,
			       (u32)gtt_get_tgid(),
			       vaddr,
			       node,
			       &mapping);
	if (ret) {
		DRM_DEBUG("add_node failed");
		goto failed_add_node;
	}

	node = mapping->node;
	offset_pages = node->start;

	DRM_DEBUG("get free node for %d pages, offset %d pages",
		  pages, offset_pages);

	/*update gtt*/
	psb_gtt_insert_pfn_list(pg, pfn_list,
			     (unsigned)offset_pages,
			     (unsigned)pages,
			     0,
			     0,
			     0);

	/*free pfn_list if allocated*/
	kfree(pfn_list);

	*offset = offset_pages;
	return 0;

failed_add_node:
	psb_gtt_mm_free_mem(mm, node);
failed_pages_alloc:
	kfree(pfn_list);
	return ret;
}

static int psb_gtt_unmap_vaddr(struct drm_device *dev,
			uint32_t vaddr,
			uint32_t size)
{
	return psb_gtt_unmap_common(dev, gtt_get_tgid(), vaddr);

}

int psb_gtt_map_meminfo_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct psb_gtt_mapping_arg *arg
	= (struct psb_gtt_mapping_arg *)data;
	uint32_t *offset_pages = &arg->offset_pages;
	uint32_t page_align = arg->page_align;
	uint32_t vaddr = arg->vaddr;
	uint32_t size = arg->size;
	uint32_t type = arg->type;

	DRM_DEBUG("\n");

	switch (type) {
	case PSB_GTT_MAP_TYPE_MEMINFO:
		return psb_gtt_map_meminfo(dev,
				arg->hKernelMemInfo,
				page_align,
				offset_pages);
	case PSB_GTT_MAP_TYPE_VIRTUAL:
		return psb_gtt_map_vaddr(dev,
					vaddr,
					size,
					page_align,
					offset_pages);
	default:
		DRM_ERROR("unsupported buffer type %d\n", type);
		return -EINVAL;
	}
}

int psb_gtt_unmap_meminfo_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{

	struct psb_gtt_mapping_arg *arg
		= (struct psb_gtt_mapping_arg *)data;
	uint32_t vaddr = arg->vaddr;
	uint32_t size = arg->size;
	uint32_t type = arg->type;

	DRM_DEBUG("\n");

	switch (type) {
	case PSB_GTT_MAP_TYPE_MEMINFO:
		return psb_gtt_unmap_meminfo(dev, arg->hKernelMemInfo);
	case PSB_GTT_MAP_TYPE_VIRTUAL:
		return psb_gtt_unmap_vaddr(dev, vaddr, size);
	default:
		DRM_ERROR("unsupported buffer type %d\n", type);
		return -EINVAL;
	}
}

int DCCBgttMapMemory(struct drm_device *dev,
		     unsigned int hHandle,
		     unsigned int ui32TaskId,
		     uintptr_t * pPages,
		     unsigned int ui32PagesNum, unsigned int *ui32Offset)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt_mm *mm = dev_priv->gtt_mm;
	struct psb_gtt *pg = dev_priv->pg;

	uint32_t size, pages, offset_pages;
	struct drm_mm_node *node = NULL;
	struct psb_gtt_mem_mapping *mapping = NULL;
	int ret;

	size = ui32PagesNum * PAGE_SIZE;
	pages = 0;

	/*alloc memory in TT apeture */
	ret = psb_gtt_mm_alloc_mem(mm, ui32PagesNum, 0, &node);
	if (ret) {
		DRM_DEBUG("alloc TT memory error\n");
		goto failed_pages_alloc;
	}

	/*update psb_gtt_mm */
	ret = psb_gtt_add_node(mm,
			       (u32) ui32TaskId, (u32) hHandle, node, &mapping);
	if (ret) {
		DRM_DEBUG("add_node failed");
		goto failed_add_node;
	}

	node = mapping->node;
	offset_pages = node->start;

	DRM_DEBUG("get free node for %d pages, offset %d pages", pages,
		  offset_pages);

	/*update gtt */
	psb_gtt_insert_phys_addresses(pg, pPages, (unsigned)offset_pages,
				      (unsigned)ui32PagesNum, 0);

	*ui32Offset = offset_pages;
	return 0;

 failed_add_node:
	psb_gtt_mm_free_mem(mm, node);
 failed_pages_alloc:
	return ret;
}

int DCCBgttUnmapMemory(struct drm_device *dev, unsigned int hHandle,
		       unsigned int ui32TaskId)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt_mm *mm = dev_priv->gtt_mm;
	struct psb_gtt *pg = dev_priv->pg;
	uint32_t pages, offset_pages;
	struct drm_mm_node *node;
	int ret;

	ret = psb_gtt_remove_node(mm, (u32) ui32TaskId, (u32) hHandle, &node);
	if (ret) {
		printk(KERN_ERR "remove node failed\n");
		return ret;
	}

	/*remove gtt entries */
	offset_pages = node->start;
	pages = node->size;

	psb_gtt_remove_pages(pg, offset_pages, pages, 0, 0, 1);

	/*free tt node */
	psb_gtt_mm_free_mem(mm, node);
	return 0;
}
