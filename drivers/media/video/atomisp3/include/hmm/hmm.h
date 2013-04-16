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

#ifndef	__HMM_H__
#define	__HMM_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/mm.h>

#include "hmm/hmm_bo.h"
#include "hmm/hmm_pool.h"

#define HMM_CACHED true
#define HMM_UNCACHED false

typedef void * hmm_ptr;
int hmm_pool_register(unsigned int pool_size, enum hmm_pool_type pool_type);
void hmm_pool_unregister(enum hmm_pool_type pool_type);

int hmm_init(void);
void hmm_cleanup(void);

void *hmm_alloc(size_t bytes, enum hmm_bo_type type,
		int from_highmem, unsigned int userptr, bool cached);
void hmm_free(void *ptr);
int hmm_load(void *virt, void *data, unsigned int bytes);
int hmm_store(void *virt, const void *data, unsigned int bytes);
int hmm_set(void *virt, int c, unsigned int bytes);
int hmm_flush(void *virt, unsigned int bytes);
int hmm_get_mmu_base_addr(void);

/*
 * get kernel memory physical address from ISP virtual address.
 */
phys_addr_t hmm_virt_to_phys(void *virt);

/*
 * map ISP memory starts with virt to kernel virtual address
 * by using vmap. return NULL if failed.
 *
 * !! user needs to use vunmap to unmap it manually before calling
 * hmm_free to free the memory.
 *
 * virt must be the start address of ISP memory (return by hmm_alloc),
 * do not pass any other address.
 */
void *hmm_vmap(void *virt);

/*
 * map ISP memory starts with virt to specific vma.
 *
 * used for mmap operation.
 *
 * virt must be the start address of ISP memory (return by hmm_alloc),
 * do not pass any other address.
 */
int hmm_mmap(struct vm_area_struct *vma, void *virt);

extern bool dypool_enable;
extern struct hmm_bo_device bo_device;

#endif
