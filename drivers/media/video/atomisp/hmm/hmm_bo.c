/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * This file contains functions for buffer object structure management
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/gfp.h>		/* for GFP_ATOMIC */
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/hugetlb.h>
#include <linux/highmem.h>
#include <linux/slab.h>		/* for kmalloc */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <asm/cacheflush.h>
#include <linux/io.h>
#include <asm/current.h>
#include <linux/sched.h>
#include "hmm/hmm_vm.h"
#include "hmm/hmm_bo.h"
#include "hmm/hmm_bo_dev.h"
#include "hmm/hmm_common.h"
#include "atomisp_internal.h"

static inline unsigned int order_to_nr(unsigned int order)
{
	return 1U << order;
}

static inline unsigned int nr_to_order_ceil(unsigned int nr)
{
	unsigned int order = 0;

	for (; nr / 2; nr = nr / 2 + nr % 2)
		order++;

	return order;
}

static inline unsigned int nr_to_order_bottom(unsigned int nr)
{
	unsigned int order = 0;

	while (nr /= 2)
		order++;

	return order;
}

static void free_bo_internal(struct hmm_buffer_object *bo)
{
	kfree(bo);
}

/*
 * use these functions to dynamically alloc hmm_buffer_object.
 * hmm_bo_init will called for that allocated buffer object, and
 * the release callback is set to kfree.
 */
struct hmm_buffer_object *hmm_bo_create(struct hmm_bo_device *bdev, int pgnr)
{
	struct hmm_buffer_object *bo;
	int ret;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo) {
		v4l2_err(&atomisp_dev, "out of memory for bo\n");
		return NULL;
	}

	ret = hmm_bo_init(bdev, bo, pgnr, free_bo_internal);
	if (ret) {
		v4l2_err(&atomisp_dev, "hmm_bo_init failed\n");
		kfree(bo);
		return NULL;
	}

	return bo;
}

/*
 * use this function to initialize pre-allocated hmm_buffer_object.
 * as hmm_buffer_object may be used as an embedded object in an upper
 * level object, a release callback must be provided. if it is
 * embedded in upper level object, set release call back to release
 * function of that object. if no upper level object, set release
 * callback to NULL.
 *
 * bo->kref is inited to 1.
 */
int hmm_bo_init(struct hmm_bo_device *bdev,
		struct hmm_buffer_object *bo,
		unsigned int pgnr, void (*release) (struct hmm_buffer_object *))
{
	if (bdev == NULL) {
		v4l2_warn(&atomisp_dev,
			    "NULL hmm_bo_device.\n");
		return -EINVAL;
	}

	/* hmm_bo_device must be already inited */
	var_equal_return(hmm_bo_device_inited(bdev), 0, -EINVAL,
			   "hmm_bo_device not inited yet.\n");

	/* prevent zero size buffer object */
	if (pgnr == 0) {
		v4l2_err(&atomisp_dev,
			    "0 size buffer is not allowed.\n");
		return -EINVAL;
	}

	memset(bo, 0, sizeof(*bo));

	kref_init(&bo->kref);

	mutex_init(&bo->mutex);

	INIT_LIST_HEAD(&bo->list);

	bo->pgnr = pgnr;
	bo->bdev = bdev;

	INIT_LIST_HEAD(&bo->pgblocks);

	bo->release = release;

	if (!bo->release)
		v4l2_warn(&atomisp_dev,
			    "no release callback specified.\n");

	/*
	 * add to active_bo_list
	 */
	mutex_lock(&bdev->ablist_mutex);
	list_add_tail(&bo->list, &bdev->active_bo_list);
	bo->status |= HMM_BO_ACTIVE;
	mutex_unlock(&bdev->ablist_mutex);

	return 0;
}

static void hmm_bo_release(struct hmm_buffer_object *bo)
{
	struct hmm_bo_device *bdev;

	check_bo_null_return(bo, (void)0);

	bdev = bo->bdev;

	/*
	 * FIX ME:
	 *
	 * how to destroy the bo when it is stilled MMAPED?
	 *
	 * ideally, this will not happened as hmm_bo_release
	 * will only be called when kref reaches 0, and in mmap
	 * operation the hmm_bo_ref will eventually be called.
	 * so, if this happened, something goes wrong.
	 */
	if (bo->status & HMM_BO_MMAPED) {
		v4l2_err(&atomisp_dev,
			     "destroy bo which is MMAPED, do nothing\n");
		goto err;
	}

	if (bo->status & HMM_BO_BINDED) {
		v4l2_warn(&atomisp_dev,
			     "the bo is still binded, unbind it first...\n");
		hmm_bo_unbind(bo);
	}
	if (bo->status & HMM_BO_PAGE_ALLOCED) {
		v4l2_warn(&atomisp_dev,
			     "the pages is not freed, free pages first\n");
		hmm_bo_free_pages(bo);
	}
	if (bo->status & HMM_BO_VM_ALLOCED) {
		v4l2_warn(&atomisp_dev,
			     "the vm is still not freed, free vm first...\n");
		hmm_bo_free_vm(bo);
	}

	/*
	 * remove it from buffer device's buffer object list.
	 */
	if (hmm_bo_activated(bo)) {
		mutex_lock(&bdev->ablist_mutex);
		list_del(&bo->list);
		mutex_unlock(&bdev->ablist_mutex);
	} else {
		mutex_lock(&bdev->fblist_mutex);
		list_del(&bo->list);
		mutex_unlock(&bdev->fblist_mutex);
	}

	if (bo->release)
		bo->release(bo);
err:
	return;
}

int hmm_bo_activated(struct hmm_buffer_object *bo)
{
	check_bo_null_return(bo, 0);

	return bo->status & HMM_BO_ACTIVE;
}

void hmm_bo_unactivate(struct hmm_buffer_object *bo)
{
	struct hmm_bo_device *bdev;

	check_bo_null_return(bo, (void)0);

	check_bo_status_no_goto(bo, HMM_BO_ACTIVE, status_err);

	bdev = bo->bdev;

	mutex_lock(&bdev->ablist_mutex);
	list_del(&bo->list);
	mutex_unlock(&bdev->ablist_mutex);

	mutex_lock(&bdev->fblist_mutex);
	list_add_tail(&bo->list, &bdev->free_bo_list);
	bo->status &= (~HMM_BO_ACTIVE);
	mutex_unlock(&bdev->fblist_mutex);

	return;

status_err:
	v4l2_err(&atomisp_dev,
			"buffer object already unactivated.\n");
	return;
}

int hmm_bo_alloc_vm(struct hmm_buffer_object *bo)
{
	struct hmm_bo_device *bdev;

	check_bo_null_return(bo, -EINVAL);

	mutex_lock(&bo->mutex);

	check_bo_status_no_goto(bo, HMM_BO_VM_ALLOCED, status_err);

	bdev = bo->bdev;

	bo->vm_node = hmm_vm_alloc_node(&bdev->vaddr_space, bo->pgnr);
	if (unlikely(!bo->vm_node)) {
		v4l2_err(&atomisp_dev,
				"hmm_vm_alloc_node err.\n");
		goto null_vm;
	}

	bo->status |= HMM_BO_VM_ALLOCED;

	mutex_unlock(&bo->mutex);

	return 0;
null_vm:
	mutex_unlock(&bo->mutex);
	return -ENOMEM;

status_err:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
			"buffer object already has vm allocated.\n");
	return -EINVAL;
}

void hmm_bo_free_vm(struct hmm_buffer_object *bo)
{
	struct hmm_bo_device *bdev;

	check_bo_null_return(bo, (void)0);

	mutex_lock(&bo->mutex);

	check_bo_status_yes_goto(bo, HMM_BO_VM_ALLOCED, status_err);

	bdev = bo->bdev;

	hmm_vm_free_node(bo->vm_node);
	bo->vm_node = NULL;
	bo->status &= (~HMM_BO_VM_ALLOCED);
	mutex_unlock(&bo->mutex);

	return;

status_err:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
			"buffer object has no vm allocated.\n");
}

int hmm_bo_vm_allocated(struct hmm_buffer_object *bo)
{
	int ret;

	check_bo_null_return(bo, 0);

	mutex_lock(&bo->mutex);

	ret = (bo->status & HMM_BO_VM_ALLOCED);

	mutex_unlock(&bo->mutex);

	return ret;
}

/*Allocate pages which will be used only by ISP*/
static int alloc_private_pages(struct hmm_buffer_object *bo, int from_highmem,
				bool cached)
{
	int ret;
	unsigned int pgnr, order, blk_pgnr;
	struct page *pages;
	struct page_block *pgblk;
	gfp_t gfp;
	int i, j;

	gfp = GFP_KERNEL;
	if (from_highmem)
		gfp |= __GFP_HIGHMEM;

	pgnr = bo->pgnr;

	bo->pages = kzalloc(sizeof(struct page *) * pgnr, GFP_KERNEL);
	if (unlikely(!bo->pages)) {
		v4l2_err(&atomisp_dev, "out of memory for bo->pages\n");
		return -ENOMEM;
	}

	i = 0;
	while (pgnr) {
		order = nr_to_order_bottom(pgnr);
		if (order > HMM_MAX_ORDER)
			order = HMM_MAX_ORDER;
retry:
		pages = alloc_pages(gfp, order);
		if (unlikely(!pages)) {
			/*
			 * in low memory case, if allocation page fails,
			 * we turn to try if order=0 allocation could
			 * succeed. if order=0 fails too, that means there is
			 * no memory left.
			 */
			if (order == HMM_MIN_ORDER) {
				v4l2_err(&atomisp_dev,
					 "out of memory in alloc_pages\n");
				goto cleanup;
			}
			v4l2_warn(&atomisp_dev,
				  "allocate order=%d pages failed."
				  "reduing page order to %d.\n",
				  order, HMM_MIN_ORDER);
			order = HMM_MIN_ORDER;
			goto retry;
		} else {
			blk_pgnr = order_to_nr(order);

			pgblk = kzalloc(sizeof(*pgblk), GFP_KERNEL);
			if (unlikely(!pgblk)) {
				v4l2_err(&atomisp_dev,
						"out of memory for pgblk\n");
				goto out_of_mem;
			}

			INIT_LIST_HEAD(&pgblk->list);
			pgblk->pages = pages;
			pgblk->order = order;

			list_add_tail(&pgblk->list, &bo->pgblocks);

			for (j = 0; j < blk_pgnr; j++)
				bo->pages[i++] = pages + j;

			pgnr -= blk_pgnr;

			if (!cached) {
				/*
				 * set memory to uncacheable -- UC_MINUS
				 */
				ret = set_pages_uc(pages, blk_pgnr);
				if (ret) {
					v4l2_err(&atomisp_dev,
						     "set page uncacheable"
							"failed.\n");
					goto cleanup;
				}
			}
		}
	}

	return 0;
out_of_mem:
	__free_pages(pages, order);
cleanup:
	ret = -ENOMEM;
	while (!list_empty(&bo->pgblocks)) {
		pgblk = list_first_entry(&bo->pgblocks,
					 struct page_block, list);

		list_del(&pgblk->list);

		ret = set_pages_wb(pgblk->pages, order_to_nr(pgblk->order));
		if (ret)
			v4l2_err(&atomisp_dev,
					"set page to WB err...\n");

		__free_pages(pgblk->pages, pgblk->order);
		kfree(pgblk);
	}
	kfree(bo->pages);

	return ret;
}

static void free_private_pages(struct hmm_buffer_object *bo)
{
	struct page_block *pgblk;
	int ret;

	while (!list_empty(&bo->pgblocks)) {
		pgblk = list_first_entry(&bo->pgblocks,
					 struct page_block, list);

		list_del(&pgblk->list);

		ret = set_pages_wb(pgblk->pages, order_to_nr(pgblk->order));
		if (ret)
			v4l2_err(&atomisp_dev,
					"set page to WB err...\n");

		__free_pages(pgblk->pages, pgblk->order);
		kfree(pgblk);
	}

	kfree(bo->pages);
}

/*
 * Hacked from kernel function __get_user_pages in mm/memory.c
 *
 * Handle buffers allocated by other kernel space driver and mmaped into user
 * space, function Ignore the VM_PFNMAP and VM_IO flag in VMA structure
 *
 * Get physical pages from user space virtual address and update into page list
 */
static int __get_pfnmap_pages(struct task_struct *tsk, struct mm_struct *mm,
			      unsigned long start, int nr_pages,
			      unsigned int gup_flags, struct page **pages,
			      struct vm_area_struct **vmas)
{
	int i, ret;
	unsigned long vm_flags;

	if (nr_pages <= 0)
		return 0;

	VM_BUG_ON(!!pages != !!(gup_flags & FOLL_GET));

	/*
	 * Require read or write permissions.
	 * If FOLL_FORCE is set, we only require the "MAY" flags.
	 */
	vm_flags  = (gup_flags & FOLL_WRITE) ?
			(VM_WRITE | VM_MAYWRITE) : (VM_READ | VM_MAYREAD);
	vm_flags &= (gup_flags & FOLL_FORCE) ?
			(VM_MAYREAD | VM_MAYWRITE) : (VM_READ | VM_WRITE);
	i = 0;

	do {
		struct vm_area_struct *vma;

		vma = find_vma(mm, start);
		if (!vma) {
			v4l2_err(&atomisp_dev, "find_vma failed\n");
			return i ? : -EFAULT;
		}

		if (is_vm_hugetlb_page(vma)) {
			/*
			i = follow_hugetlb_page(mm, vma, pages, vmas,
					&start, &nr_pages, i, gup_flags);
			*/
			continue;
		}

		do {
			struct page *page;
			unsigned long pfn;

			/*
			 * If we have a pending SIGKILL, don't keep faulting
			 * pages and potentially allocating memory.
			 */
			if (unlikely(fatal_signal_pending(current))) {
				v4l2_err(&atomisp_dev,
					"fatal_signal_pending in %s\n",
					__func__);
				return i ? i : -ERESTARTSYS;
			}

			ret = follow_pfn(vma, start, &pfn);
			if (ret) {
				v4l2_err(&atomisp_dev,
					"follow_pfn() failed\n");
				return i ? : -EFAULT;
			}

			page = pfn_to_page(pfn);
			if (IS_ERR(page))
				return i ? i : PTR_ERR(page);
			if (pages) {
				pages[i] = page;

				flush_anon_page(vma, page, start);
				flush_dcache_page(page);
			}
			if (vmas)
				vmas[i] = vma;
			i++;
			start += PAGE_SIZE;
			nr_pages--;
		} while (nr_pages && start < vma->vm_end);
	} while (nr_pages);
	return i;
}

static int get_pfnmap_pages(struct task_struct *tsk, struct mm_struct *mm,
		     unsigned long start, int nr_pages, int write, int force,
		     struct page **pages, struct vm_area_struct **vmas)
{
	int flags = FOLL_TOUCH;

	if (pages)
		flags |= FOLL_GET;
	if (write)
		flags |= FOLL_WRITE;
	if (force)
		flags |= FOLL_FORCE;

	return __get_pfnmap_pages(tsk, mm, start, nr_pages, flags, pages, vmas);
}

/*
 * Convert user space virtual address into pages list
 */
static int alloc_user_pages(struct hmm_buffer_object *bo,
			      unsigned int userptr, bool cached)
{
	unsigned int page_nr;
	unsigned int i;
	struct page_block *pgblk;
	struct vm_area_struct *vma;
	int ret;

	bo->pages = kzalloc(sizeof(struct page *) * bo->pgnr, GFP_KERNEL);
	if (unlikely(!bo->pages)) {
		v4l2_err(&atomisp_dev, "out of memory for bo->pages...\n");
		return -ENOMEM;
	}

	mutex_unlock(&bo->mutex);
	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, userptr);
	up_read(&current->mm->mmap_sem);
	if (vma == NULL) {
		v4l2_err(&atomisp_dev, "find_vma failed\n");
		kfree(bo->pages);
		return -EFAULT;
	}
	mutex_lock(&bo->mutex);
	/*
	 * Handle frame buffer allocated in other kerenl space driver
	 * and map to user space
	 */
	if (vma->vm_flags & (VM_IO | VM_PFNMAP)) {
		page_nr = get_pfnmap_pages(current, current->mm,
					   (unsigned long)userptr,
					   (int)(bo->pgnr), 1, 0,
					   bo->pages, NULL);
		bo->mem_type = HMM_BO_MEM_TYPE_PFN;
	} else {
		/*Handle frame buffer allocated in user space*/
		mutex_unlock(&bo->mutex);
		down_read(&current->mm->mmap_sem);
		page_nr = get_user_pages(current, current->mm,
					 (unsigned long)userptr,
					 (int)(bo->pgnr), 1, 0, bo->pages,
					 NULL);
		up_read(&current->mm->mmap_sem);
		mutex_lock(&bo->mutex);
		bo->mem_type = HMM_BO_MEM_TYPE_USER;
	}

	/* can be written by caller, not forced */
	if (page_nr != bo->pgnr) {
		v4l2_err(&atomisp_dev,
				"get_user_pages err: bo->pgnr = %d, "
				"pgnr actually pinned = %d.\n",
				bo->pgnr, page_nr);
		goto out_of_mem;
	}

	pgblk = kzalloc(sizeof(*pgblk) * bo->pgnr, GFP_KERNEL);
	if (unlikely(!pgblk)) {
		v4l2_err(&atomisp_dev, "out of memory for pgblk\n");
		goto out_of_mem;
	}

	for (i = 0; i < bo->pgnr; i++) {
		INIT_LIST_HEAD(&pgblk->list);
		pgblk->pages = bo->pages[i];
		pgblk->order = 0;

		list_add_tail(&pgblk->list, &bo->pgblocks);
		pgblk++;
	}
	return 0;

out_of_mem:
	ret = -ENOMEM;
	while (!list_empty(&bo->pgblocks)) {
		pgblk = list_first_entry(&bo->pgblocks,
					 struct page_block, list);

		list_del(&pgblk->list);

		kfree(pgblk);
	}

	if (bo->mem_type == HMM_BO_MEM_TYPE_USER)
		for (i = 0; i < page_nr; i++)
			put_page(bo->pages[i]);
	kfree(bo->pages);

	return ret;
}

static void free_user_pages(struct hmm_buffer_object *bo)
{
	struct page_block *pgblk, *head;
	head = list_first_entry(&bo->pgblocks,
				struct page_block, list);

	while (!list_empty(&bo->pgblocks)) {
		pgblk = list_first_entry(&bo->pgblocks,
					 struct page_block, list);
		if (bo->mem_type == HMM_BO_MEM_TYPE_USER)
			put_page(pgblk->pages);

		list_del(&pgblk->list);
	}

	kfree(head);
	kfree(bo->pages);
}

/*
 * allocate/free physical pages for the bo.
 *
 * type indicate where are the pages from. currently we have 3 types
 * of memory: HMM_BO_PRIVATE, HMM_BO_USER, HMM_BO_SHARE.
 *
 * from_highmem is only valid when type is HMM_BO_PRIVATE, it will
 * try to alloc memory from highmem if from_highmem is set.
 *
 * userptr is only valid when type is HMM_BO_USER, it indicates
 * the start address from user space task.
 *
 * from_highmem and userptr will both be ignored when type is
 * HMM_BO_SHARE.
 */
int hmm_bo_alloc_pages(struct hmm_buffer_object *bo,
		       enum hmm_bo_type type, int from_highmem,
		       unsigned int userptr, bool cached)
{
	int ret;

	check_bo_null_return(bo, -EINVAL);

	mutex_lock(&bo->mutex);

	check_bo_status_no_goto(bo, HMM_BO_PAGE_ALLOCED, status_err);

	/*
	 * TO DO:
	 * add HMM_BO_USER type
	 */
	if (type == HMM_BO_PRIVATE)
		ret = alloc_private_pages(bo, from_highmem, cached);
	else if (type == HMM_BO_USER)
		ret = alloc_user_pages(bo, userptr, cached);
	else {
		v4l2_err(&atomisp_dev, "invalid buffer type.\n");
		ret = -EINVAL;
	}

	if (ret)
		goto alloc_err;

	bo->type = type;

	bo->status |= HMM_BO_PAGE_ALLOCED;

	mutex_unlock(&bo->mutex);

	return 0;

alloc_err:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev, "alloc pages err...\n");
	return ret;
status_err:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
			"buffer object has already page allocated.\n");
	return -EINVAL;
}

/*
 * free physical pages of the bo.
 */
void hmm_bo_free_pages(struct hmm_buffer_object *bo)
{
	check_bo_null_return(bo, (void)0);

	mutex_lock(&bo->mutex);

	check_bo_status_yes_goto(bo, HMM_BO_PAGE_ALLOCED, status_err2);

	if (bo->type == HMM_BO_PRIVATE)
		free_private_pages(bo);
	else if (bo->type == HMM_BO_USER)
		free_user_pages(bo);
	else
		v4l2_err(&atomisp_dev, "invalid buffer type.\n");
	/* clear the flag anyway. */
	bo->status &= (~HMM_BO_PAGE_ALLOCED);

	mutex_unlock(&bo->mutex);

	return;

status_err2:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
			"buffer object not page allocated yet.\n");

}

int hmm_bo_page_allocated(struct hmm_buffer_object *bo)
{
	int ret;

	check_bo_null_return(bo, 0);

	mutex_lock(&bo->mutex);

	ret = bo->status & HMM_BO_PAGE_ALLOCED;

	mutex_unlock(&bo->mutex);

	return ret;
}

/*
 * get physical page info of the bo.
 */
int hmm_bo_get_page_info(struct hmm_buffer_object *bo,
			 struct page ***pages, int *pgnr)
{
	check_bo_null_return(bo, -EINVAL);

	mutex_lock(&bo->mutex);

	check_bo_status_yes_goto(bo, HMM_BO_PAGE_ALLOCED, status_err);

	*pages = bo->pages;
	*pgnr = bo->pgnr;

	mutex_unlock(&bo->mutex);

	return 0;

status_err:
	v4l2_err(&atomisp_dev,
			"buffer object not page allocated yet.\n");
	mutex_unlock(&bo->mutex);
	return -EINVAL;
}

/*
 * bind the physical pages to a virtual address space.
 */
int hmm_bo_bind(struct hmm_buffer_object *bo)
{
	int ret;
	unsigned int virt;
	struct hmm_bo_device *bdev;
	unsigned int i;

	check_bo_null_return(bo, -EINVAL);

	mutex_lock(&bo->mutex);

	check_bo_status_yes_goto(bo,
				   HMM_BO_PAGE_ALLOCED | HMM_BO_VM_ALLOCED,
				   status_err1);

	check_bo_status_no_goto(bo, HMM_BO_BINDED, status_err2);

	bdev = bo->bdev;

	virt = bo->vm_node->start;

	for (i = 0; i < bo->pgnr; i++) {
		ret =
		    isp_mmu_map(&bdev->mmu, virt, page_to_phys(bo->pages[i]),
				1);
		if (ret)
			goto map_err;
		virt += (1 << PAGE_SHIFT);
	}

	/*
	 * flush TBL here.
	 *
	 * theoretically, we donot need to flush TLB as we didnot change
	 * any existed address mappings, but for Silicon Hive's MMU, its
	 * really a bug here. I guess when fetching PTEs (page table entity)
	 * to TLB, its MMU will fetch additional INVALID PTEs automatically
	 * for performance issue. EX, we only set up 1 page address mapping,
	 * meaning updating 1 PTE, but the MMU fetches 4 PTE at one time,
	 * so the additional 3 PTEs are invalid.
	 */
	isp_mmu_flush_tlb_range(&bdev->mmu, bo->vm_node->start,
				(bo->pgnr << PAGE_SHIFT));

	bo->status |= HMM_BO_BINDED;

	mutex_unlock(&bo->mutex);

	return 0;

map_err:
	/* unbind the physical pages with related virtual address space */
	virt = bo->vm_node->start;
	for ( ; i > 0; i--) {
		isp_mmu_unmap(&bdev->mmu, virt, 1);
		virt += pgnr_to_size(1);
	}

	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
			"setup MMU address mapping failed.\n");
	return ret;

status_err2:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev, "buffer object already binded.\n");
	return -EINVAL;
status_err1:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
		     "buffer object vm_node or page not allocated.\n");
	return -EINVAL;
}

/*
 * unbind the physical pages with related virtual address space.
 */
void hmm_bo_unbind(struct hmm_buffer_object *bo)
{
	unsigned int virt;
	struct hmm_bo_device *bdev;
	unsigned int i;

	check_bo_null_return(bo, (void)0);

	mutex_lock(&bo->mutex);

	check_bo_status_yes_goto(bo,
				   HMM_BO_PAGE_ALLOCED |
				   HMM_BO_VM_ALLOCED |
				   HMM_BO_BINDED, status_err);

	bdev = bo->bdev;

	virt = bo->vm_node->start;

	for (i = 0; i < bo->pgnr; i++) {
		isp_mmu_unmap(&bdev->mmu, virt, 1);
		virt += pgnr_to_size(1);
	}

	/*
	 * flush TLB as the address mapping has been removed and
	 * related TLBs should be invalidated.
	 */
	isp_mmu_flush_tlb_range(&bdev->mmu, bo->vm_node->start,
				(bo->pgnr << PAGE_SHIFT));

	bo->status &= (~HMM_BO_BINDED);

	mutex_unlock(&bo->mutex);

	return;

status_err:
	mutex_unlock(&bo->mutex);
	v4l2_err(&atomisp_dev,
		     "buffer vm or page not allocated or not binded yet.\n");
}

int hmm_bo_binded(struct hmm_buffer_object *bo)
{
	int ret;

	check_bo_null_return(bo, 0);

	mutex_lock(&bo->mutex);

	ret = bo->status & HMM_BO_BINDED;

	mutex_unlock(&bo->mutex);

	return ret;
}

void *hmm_bo_vmap(struct hmm_buffer_object *bo)
{
	check_bo_null_return(bo, NULL);

	return vmap(bo->pages, bo->pgnr, VM_MAP, PAGE_KERNEL_NOCACHE);
}

void hmm_bo_ref(struct hmm_buffer_object *bo)
{
	check_bo_null_return(bo, (void)0);

	kref_get(&bo->kref);
}

static void kref_hmm_bo_release(struct kref *kref)
{
	if (!kref)
		return;

	hmm_bo_release(kref_to_hmm_bo(kref));
}

void hmm_bo_unref(struct hmm_buffer_object *bo)
{
	check_bo_null_return(bo, (void)0);

	kref_put(&bo->kref, kref_hmm_bo_release);
}

static void hmm_bo_vm_open(struct vm_area_struct *vma)
{
	struct hmm_buffer_object *bo =
	    (struct hmm_buffer_object *)vma->vm_private_data;

	check_bo_null_return(bo, (void)0);

	hmm_bo_ref(bo);

	mutex_lock(&bo->mutex);

	bo->status |= HMM_BO_MMAPED;

	bo->mmap_count++;

	mutex_unlock(&bo->mutex);
}

static void hmm_bo_vm_close(struct vm_area_struct *vma)
{
	struct hmm_buffer_object *bo =
	    (struct hmm_buffer_object *)vma->vm_private_data;

	check_bo_null_return(bo, (void)0);

	hmm_bo_unref(bo);

	mutex_lock(&bo->mutex);

	bo->mmap_count--;

	if (!bo->mmap_count) {
		bo->status &= (~HMM_BO_MMAPED);
		vma->vm_private_data = NULL;
	}

	mutex_unlock(&bo->mutex);
}

static const struct vm_operations_struct hmm_bo_vm_ops = {
	.open = hmm_bo_vm_open,
	.close = hmm_bo_vm_close,
};

/*
 * mmap the bo to user space.
 */
int hmm_bo_mmap(struct vm_area_struct *vma, struct hmm_buffer_object *bo)
{
	unsigned int start, end;
	unsigned int virt;
	unsigned int pgnr, i;
	unsigned int pfn;

	check_bo_null_return(bo, -EINVAL);

	check_bo_status_yes_goto(bo, HMM_BO_PAGE_ALLOCED, status_err);

	pgnr = bo->pgnr;
	start = vma->vm_start;
	end = vma->vm_end;

	/*
	 * check vma's virtual address space size and buffer object's size.
	 * must be the same.
	 */
	if ((start + pgnr_to_size(pgnr)) != end) {
		v4l2_warn(&atomisp_dev,
			     "vma's address space size not equal"
			     " to buffer object's size");
		return -EINVAL;
	}

	virt = vma->vm_start;
	for (i = 0; i < pgnr; i++) {
		pfn = page_to_pfn(bo->pages[i]);
		if (remap_pfn_range(vma, virt, pfn, PAGE_SIZE, PAGE_SHARED)) {
			v4l2_warn(&atomisp_dev,
					"remap_pfn_range failed:"
					" virt = 0x%x, pfn = 0x%x,"
					" mapped_pgnr = %d\n", virt, pfn, 1);
			return -EINVAL;
		}
		virt += PAGE_SIZE;
	}

	vma->vm_private_data = bo;

	vma->vm_ops = &hmm_bo_vm_ops;
	vma->vm_flags |= (VM_RESERVED | VM_IO);

	/*
	 * call hmm_bo_vm_open explictly.
	 */
	hmm_bo_vm_open(vma);

	return 0;

status_err:
	v4l2_err(&atomisp_dev,
			"buffer page not allocated yet.\n");
	return -EINVAL;
}
