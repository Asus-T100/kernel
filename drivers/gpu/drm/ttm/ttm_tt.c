/**************************************************************************
 *
 * Copyright (c) 2006-2009 VMware, Inc., Palo Alto, CA., USA
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/
/*
 * Authors: Thomas Hellstrom <thellstrom-at-vmware-dot-com>
 */

#include <linux/sched.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/shmem_fs.h>
#include <linux/file.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include "drm_cache.h"
#include "drm_mem_util.h"
#include "ttm/ttm_module.h"
#include "ttm/ttm_bo_driver.h"
#include "ttm/ttm_placement.h"
#include "ttm/ttm_page_alloc.h"
#include <linux/hugetlb.h>

static int ttm_tt_swapin(struct ttm_tt *ttm);

/**
 * Allocates storage for pointers to the pages that back the ttm.
 */
static void ttm_tt_alloc_page_directory(struct ttm_tt *ttm)
{
	ttm->pages = drm_calloc_large(ttm->num_pages, sizeof(*ttm->pages));
	ttm->dma_address = drm_calloc_large(ttm->num_pages,
					    sizeof(*ttm->dma_address));
}

static void ttm_tt_free_page_directory(struct ttm_tt *ttm)
{
	drm_free_large(ttm->pages);
	ttm->pages = NULL;
	drm_free_large(ttm->dma_address);
	ttm->dma_address = NULL;
}

static void ttm_tt_free_user_pages(struct ttm_tt *ttm)
{
	int write;
	int dirty;
	struct page *page;
	struct page **pages_to_wb;
	int i;
	struct ttm_backend *be = ttm->be;

	BUG_ON(!(ttm->page_flags & TTM_PAGE_FLAG_USER));
	write = ((ttm->page_flags & TTM_PAGE_FLAG_WRITE) != 0);
	dirty = ((ttm->page_flags & TTM_PAGE_FLAG_USER_DIRTY) != 0);

	if (be)
		be->func->clear(be);

	pages_to_wb = kmalloc(ttm->num_pages * sizeof(struct page *),
			GFP_KERNEL);

	if (pages_to_wb && ttm->caching_state != tt_cached) {
		int num_pages_wb = 0;

		for (i = 0; i < ttm->num_pages; ++i) {
			page = ttm->pages[i];
			if (page == NULL)
				continue;
			pages_to_wb[num_pages_wb++] = page;
		}

		if (set_pages_array_wb(pages_to_wb, num_pages_wb))
			printk(KERN_ERR TTM_PFX "Failed to set pages to wb\n");

	} else if (NULL == pages_to_wb) {
		printk(KERN_ERR TTM_PFX
		       "Failed to allocate memory for set wb operation.\n");
	}

	for (i = 0; i < ttm->num_pages; ++i) {
		page = ttm->pages[i];
		if (page == NULL)
			continue;

		if (page == ttm->dummy_read_page) {
			BUG_ON(write);
			continue;
		}

		if (write && dirty && !PageReserved(page))
			set_page_dirty_lock(page);

		ttm->pages[i] = NULL;
		ttm_mem_global_free(ttm->glob->mem_glob, PAGE_SIZE);
		put_page(page);
	}
	ttm->state = tt_unpopulated;
	ttm->first_himem_page = ttm->num_pages;
	ttm->last_lomem_page = -1;
	kfree(pages_to_wb);
}

static struct page *__ttm_tt_get_page(struct ttm_tt *ttm, int index)
{
	struct page *p;
	struct list_head h;
	struct ttm_mem_global *mem_glob = ttm->glob->mem_glob;
	int ret;

	while (NULL == (p = ttm->pages[index])) {

		INIT_LIST_HEAD(&h);

		ret = ttm_get_pages(&h, ttm->page_flags, ttm->caching_state, 1,
				    &ttm->dma_address[index]);

		if (ret != 0)
			return NULL;

		p = list_first_entry(&h, struct page, lru);

		ret = ttm_mem_global_alloc_page(mem_glob, p, false, false);
		if (unlikely(ret != 0))
			goto out_err;

		if (PageHighMem(p))
			ttm->pages[--ttm->first_himem_page] = p;
		else
			ttm->pages[++ttm->last_lomem_page] = p;
	}
	return p;
out_err:
	put_page(p);
	return NULL;
}

struct page *ttm_tt_get_page(struct ttm_tt *ttm, int index)
{
	int ret;

	if (unlikely(ttm->page_flags & TTM_PAGE_FLAG_SWAPPED)) {
		ret = ttm_tt_swapin(ttm);
		if (unlikely(ret != 0))
			return NULL;
	}
	return __ttm_tt_get_page(ttm, index);
}

int ttm_tt_populate(struct ttm_tt *ttm)
{
	struct page *page;
	unsigned long i;
	struct ttm_backend *be;
	int ret;
	be = ttm->be;
	if (ttm->state == tt_unbound) {
		/*when bo type is ttm_bo_type_user, also need call populate*/
		be->func->populate(be, ttm->num_pages, ttm->pages,
				   ttm->dummy_read_page, ttm->dma_address);
		return 0;
	}

	if (ttm->state != tt_unpopulated)
		return 0;

	if (unlikely(ttm->page_flags & TTM_PAGE_FLAG_SWAPPED)) {
		ret = ttm_tt_swapin(ttm);
		if (unlikely(ret != 0))
			return ret;
	}

	for (i = 0; i < ttm->num_pages; ++i) {
		page = __ttm_tt_get_page(ttm, i);
		if (!page)
			return -ENOMEM;
	}

	be->func->populate(be, ttm->num_pages, ttm->pages,
			   ttm->dummy_read_page, ttm->dma_address);
	ttm->state = tt_unbound;
	return 0;
}
EXPORT_SYMBOL(ttm_tt_populate);

#ifdef CONFIG_X86
static inline int ttm_tt_set_page_caching(struct page *p,
					  enum ttm_caching_state c_old,
					  enum ttm_caching_state c_new)
{
	int ret = 0;

	if (PageHighMem(p))
		return 0;

	if (c_old != tt_cached) {
		/* p isn't in the default caching state, set it to
		 * writeback first to free its current memtype. */

		ret = set_pages_wb(p, 1);
		if (ret)
			return ret;
	}

	if (c_new == tt_wc)
		ret = set_memory_wc((unsigned long) page_address(p), 1);
	else if (c_new == tt_uncached)
		ret = set_pages_uc(p, 1);

	return ret;
}
#else /* CONFIG_X86 */
static inline int ttm_tt_set_page_caching(struct page *p,
					  enum ttm_caching_state c_old,
					  enum ttm_caching_state c_new)
{
	return 0;
}
#endif /* CONFIG_X86 */

/*
 * Change caching policy for the linear kernel map
 * for range of pages in a ttm.
 */

static int ttm_tt_set_caching(struct ttm_tt *ttm,
			      enum ttm_caching_state c_state)
{
	int i, j;
	struct page *cur_page;
	int ret;

	if (ttm->caching_state == c_state)
		return 0;

	if (ttm->state == tt_unpopulated) {
		/* Change caching but don't populate */
		ttm->caching_state = c_state;
		return 0;
	}

	if (ttm->caching_state == tt_cached)
		drm_clflush_pages(ttm->pages, ttm->num_pages);

	for (i = 0; i < ttm->num_pages; ++i) {
		cur_page = ttm->pages[i];
		if (likely(cur_page != NULL)) {
			ret = ttm_tt_set_page_caching(cur_page,
						      ttm->caching_state,
						      c_state);
			if (unlikely(ret != 0))
				goto out_err;
		}
	}

	ttm->caching_state = c_state;

	return 0;

out_err:
	for (j = 0; j < i; ++j) {
		cur_page = ttm->pages[j];
		if (likely(cur_page != NULL)) {
			(void)ttm_tt_set_page_caching(cur_page, c_state,
						      ttm->caching_state);
		}
	}

	return ret;
}

int ttm_tt_set_placement_caching(struct ttm_tt *ttm, uint32_t placement)
{
	enum ttm_caching_state state;

	if (placement & TTM_PL_FLAG_WC)
		state = tt_wc;
	else if (placement & TTM_PL_FLAG_UNCACHED)
		state = tt_uncached;
	else
		state = tt_cached;

	return ttm_tt_set_caching(ttm, state);
}
EXPORT_SYMBOL(ttm_tt_set_placement_caching);

static void ttm_tt_free_alloced_pages(struct ttm_tt *ttm)
{
	int i;
	unsigned count = 0;
	struct list_head h;
	struct page *cur_page;
	struct ttm_backend *be = ttm->be;

	INIT_LIST_HEAD(&h);

	if (be)
		be->func->clear(be);
	for (i = 0; i < ttm->num_pages; ++i) {

		cur_page = ttm->pages[i];
		ttm->pages[i] = NULL;
		if (cur_page) {
			if (page_count(cur_page) != 1)
				printk(KERN_ERR TTM_PFX
				       "Erroneous page count. "
				       "Leaking pages.\n");
			ttm_mem_global_free_page(ttm->glob->mem_glob,
						 cur_page);
			list_add(&cur_page->lru, &h);
			count++;
		}
	}
	ttm_put_pages(&h, count, ttm->page_flags, ttm->caching_state,
		      ttm->dma_address);
	ttm->state = tt_unpopulated;
	ttm->first_himem_page = ttm->num_pages;
	ttm->last_lomem_page = -1;
}

void ttm_tt_destroy(struct ttm_tt *ttm)
{
	struct ttm_backend *be;

	if (unlikely(ttm == NULL))
		return;

	be = ttm->be;
	if (likely(be != NULL)) {
		be->func->destroy(be);
		ttm->be = NULL;
	}

	if (likely(ttm->pages != NULL)) {
		if (ttm->page_flags & TTM_PAGE_FLAG_USER)
			ttm_tt_free_user_pages(ttm);
		else
			ttm_tt_free_alloced_pages(ttm);

		ttm_tt_free_page_directory(ttm);
	}

	if (!(ttm->page_flags & TTM_PAGE_FLAG_PERSISTENT_SWAP) &&
	    ttm->swap_storage)
		fput(ttm->swap_storage);

	kfree(ttm);
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
		if (!vma)
			return i ? : -EFAULT;

		if (is_vm_hugetlb_page(vma)) {
			i = follow_hugetlb_page(mm, vma, pages, vmas,
					&start, &nr_pages, i, gup_flags);
			continue;
		}

		do {
			struct page *page;
			unsigned long pfn;

			/*
			* If we have a pending SIGKILL, don't keep faulting
			* pages and potentially allocating memory.
			*/
			if (unlikely(fatal_signal_pending(current)))
				return i ? i : -ERESTARTSYS;

			if ((ret = follow_pfn(vma, start, &pfn)))
				return i ? : -EFAULT;

			page = pfn_to_page(pfn);
			if (IS_ERR(page))
				return i ? i : PTR_ERR(page);
			if (pages) {
				pages[i] = page;
				get_page(page);
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
static int alloc_user_pages_ttm(struct ttm_tt *ttm,
				unsigned int userptr)
{
	unsigned int page_nr;
	unsigned int i;
	struct page_block *pgblk;
	struct vm_area_struct *vma;
	int ret;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, userptr);
	up_read(&current->mm->mmap_sem);
	if (vma == NULL) {
		printk(KERN_ERR "find_vma failed\n");
		return -EFAULT;
	}

	/*
	 * Handle frame buffer allocated in other kerenl space driver
	 * and map to user space
	 */
	if (vma->vm_flags & (VM_IO | VM_PFNMAP)) {
		down_read(&current->mm->mmap_sem);
		page_nr = get_pfnmap_pages(current, current->mm,
					   (unsigned long)userptr,
					   (int)(ttm->num_pages), 1, 0,
					   ttm->pages, NULL);
		up_read(&current->mm->mmap_sem);
	} else {
		/*Handle frame buffer allocated in user space*/
		down_read(&current->mm->mmap_sem);
		page_nr = get_user_pages(current, current->mm,
					 (unsigned long)userptr,
					 (int)(ttm->num_pages), 1, 0, ttm->pages,
					 NULL);
		up_read(&current->mm->mmap_sem);
	}

	/* can be written by caller, not forced */
	if (page_nr != ttm->num_pages) {
		printk(KERN_ERR "get_user_pages err: bo->pgnr = %lu, "
				"pgnr actually pinned = %u.\n",
				ttm->num_pages, page_nr);
		return -ENOMEM;
	}

	return 0;

out_of_mem:
	ret = -ENOMEM;

	return ret;
}

int ttm_tt_set_user(struct ttm_tt *ttm,
		    struct task_struct *tsk,
		    unsigned long start, unsigned long num_pages)
{
	struct mm_struct *mm = tsk->mm;
	int ret, i;
	int write = (ttm->page_flags & TTM_PAGE_FLAG_WRITE) != 0;
	struct ttm_mem_global *mem_glob = ttm->glob->mem_glob;

	BUG_ON(num_pages != ttm->num_pages);
	BUG_ON((ttm->page_flags & TTM_PAGE_FLAG_USER) == 0);

	/**
	 * Account user pages as lowmem pages for now.
	 */

	ret = ttm_mem_global_alloc(mem_glob, num_pages * PAGE_SIZE,
				   false, false);
	if (unlikely(ret != 0))
		return ret;

	/**
	 * get_user_pages can only handle the buffer allocated from user space,
	 * but we need handle the buffer allocated from other kernel driver
	 */
#if 0
	down_read(&mm->mmap_sem);

	ret = get_user_pages(tsk, mm, start, num_pages,
			     write, 0, ttm->pages, NULL);
	up_read(&mm->mmap_sem);
#else
	ret = alloc_user_pages_ttm(ttm, start);
#endif


	if (ret != num_pages && write) {
		ttm_tt_free_user_pages(ttm);
		ttm_mem_global_free(mem_glob, num_pages * PAGE_SIZE);
		return -ENOMEM;
	}

	ttm->tsk = tsk;
	ttm->start = start;
	ttm->state = tt_unbound;

	return 0;
}

struct ttm_tt *ttm_tt_create(struct ttm_bo_device *bdev, unsigned long size,
			     uint32_t page_flags, struct page *dummy_read_page)
{
	struct ttm_bo_driver *bo_driver = bdev->driver;
	struct ttm_tt *ttm;

	if (!bo_driver)
		return NULL;

	ttm = kzalloc(sizeof(*ttm), GFP_KERNEL);
	if (!ttm)
		return NULL;

	ttm->glob = bdev->glob;
	ttm->num_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	ttm->first_himem_page = ttm->num_pages;
	ttm->last_lomem_page = -1;
	ttm->caching_state = tt_cached;
	ttm->page_flags = page_flags;

	ttm->dummy_read_page = dummy_read_page;

	ttm_tt_alloc_page_directory(ttm);
	if (!ttm->pages) {
		ttm_tt_destroy(ttm);
		printk(KERN_ERR TTM_PFX "Failed allocating page table\n");
		return NULL;
	}
	ttm->be = bo_driver->create_ttm_backend_entry(bdev);
	if (!ttm->be) {
		ttm_tt_destroy(ttm);
		printk(KERN_ERR TTM_PFX "Failed creating ttm backend entry\n");
		return NULL;
	}
	ttm->state = tt_unpopulated;
	return ttm;
}

void ttm_tt_unbind(struct ttm_tt *ttm)
{
	int ret;
	struct ttm_backend *be = ttm->be;

	if (ttm->state == tt_bound) {
		ret = be->func->unbind(be);
		BUG_ON(ret);
		ttm->state = tt_unbound;
	}
}

int ttm_tt_bind(struct ttm_tt *ttm, struct ttm_mem_reg *bo_mem)
{
	int ret = 0;
	struct ttm_backend *be;

	if (!ttm)
		return -EINVAL;

	if (ttm->state == tt_bound)
		return 0;

	be = ttm->be;

	ret = ttm_tt_populate(ttm);
	if (ret)
		return ret;

	ret = be->func->bind(be, bo_mem);
	if (unlikely(ret != 0))
		return ret;

	ttm->state = tt_bound;

	if (ttm->page_flags & TTM_PAGE_FLAG_USER)
		ttm->page_flags |= TTM_PAGE_FLAG_USER_DIRTY;
	return 0;
}
EXPORT_SYMBOL(ttm_tt_bind);

static int ttm_tt_swapin(struct ttm_tt *ttm)
{
	struct address_space *swap_space;
	struct file *swap_storage;
	struct page *from_page;
	struct page *to_page;
	void *from_virtual;
	void *to_virtual;
	int i;
	int ret = -ENOMEM;

	if (ttm->page_flags & TTM_PAGE_FLAG_USER) {
		ret = ttm_tt_set_user(ttm, ttm->tsk, ttm->start,
				      ttm->num_pages);
		if (unlikely(ret != 0))
			return ret;

		ttm->page_flags &= ~TTM_PAGE_FLAG_SWAPPED;
		return 0;
	}

	swap_storage = ttm->swap_storage;
	BUG_ON(swap_storage == NULL);

	swap_space = swap_storage->f_path.dentry->d_inode->i_mapping;

	for (i = 0; i < ttm->num_pages; ++i) {
		from_page = shmem_read_mapping_page(swap_space, i);
		if (IS_ERR(from_page)) {
			ret = PTR_ERR(from_page);
			goto out_err;
		}
		to_page = __ttm_tt_get_page(ttm, i);
		if (unlikely(to_page == NULL))
			goto out_err;

		preempt_disable();
		from_virtual = kmap_atomic(from_page, KM_USER0);
		to_virtual = kmap_atomic(to_page, KM_USER1);
		memcpy(to_virtual, from_virtual, PAGE_SIZE);
		kunmap_atomic(to_virtual, KM_USER1);
		kunmap_atomic(from_virtual, KM_USER0);
		preempt_enable();
		page_cache_release(from_page);
	}

	if (!(ttm->page_flags & TTM_PAGE_FLAG_PERSISTENT_SWAP))
		fput(swap_storage);
	ttm->swap_storage = NULL;
	ttm->page_flags &= ~TTM_PAGE_FLAG_SWAPPED;

	return 0;
out_err:
	ttm_tt_free_alloced_pages(ttm);
	return ret;
}

int ttm_tt_swapout(struct ttm_tt *ttm, struct file *persistent_swap_storage)
{
	struct address_space *swap_space;
	struct file *swap_storage;
	struct page *from_page;
	struct page *to_page;
	void *from_virtual;
	void *to_virtual;
	int i;
	int ret = -ENOMEM;

	BUG_ON(ttm->state != tt_unbound && ttm->state != tt_unpopulated);
	BUG_ON(ttm->caching_state != tt_cached);

	/*
	 * For user buffers, just unpin the pages, as there should be
	 * vma references.
	 */

	if (ttm->page_flags & TTM_PAGE_FLAG_USER) {
		ttm_tt_free_user_pages(ttm);
		ttm->page_flags |= TTM_PAGE_FLAG_SWAPPED;
		ttm->swap_storage = NULL;
		return 0;
	}

	if (!persistent_swap_storage) {
		swap_storage = shmem_file_setup("ttm swap",
						ttm->num_pages << PAGE_SHIFT,
						0);
		if (unlikely(IS_ERR(swap_storage))) {
			printk(KERN_ERR "Failed allocating swap storage.\n");
			return PTR_ERR(swap_storage);
		}
	} else
		swap_storage = persistent_swap_storage;

	swap_space = swap_storage->f_path.dentry->d_inode->i_mapping;

	for (i = 0; i < ttm->num_pages; ++i) {
		from_page = ttm->pages[i];
		if (unlikely(from_page == NULL))
			continue;
		to_page = shmem_read_mapping_page(swap_space, i);
		if (unlikely(IS_ERR(to_page))) {
			ret = PTR_ERR(to_page);
			goto out_err;
		}
		preempt_disable();
		from_virtual = kmap_atomic(from_page, KM_USER0);
		to_virtual = kmap_atomic(to_page, KM_USER1);
		memcpy(to_virtual, from_virtual, PAGE_SIZE);
		kunmap_atomic(to_virtual, KM_USER1);
		kunmap_atomic(from_virtual, KM_USER0);
		preempt_enable();
		set_page_dirty(to_page);
		mark_page_accessed(to_page);
		page_cache_release(to_page);
	}

	ttm_tt_free_alloced_pages(ttm);
	ttm->swap_storage = swap_storage;
	ttm->page_flags |= TTM_PAGE_FLAG_SWAPPED;
	if (persistent_swap_storage)
		ttm->page_flags |= TTM_PAGE_FLAG_PERSISTENT_SWAP;

	return 0;
out_err:
	if (!persistent_swap_storage)
		fput(swap_storage);

	return ret;
}
