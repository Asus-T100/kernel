/*
 * Support for Medfield PNW Camera Imaging ISP subsystem.
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

#ifndef _hmm_64_h_
#define _hmm_64_h_

//#include <stdlib.h>
//#include <stdio.h>
//#include <stdint.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/mm.h>

#include "hmm/hmm_bo.h"

typedef enum {
  hmm_esuccess,
  hmm_enomem,
  hmm_ezeroalloc,
  hmm_ebadvaddr,
  hmm_einternalerror,
  hmm_ecorruption,
  hmm_enocontiguousmem,
  hmm_enolocmem,
  hmm_emultiplefree,
} hmm_error;

typedef struct hmm_memory_map_s hmm_memory_map_t, *hmm_memory_map;

extern hmm_error hmm_errno;

typedef uint64_t hmm_pptr; /* Physical memory address type */
typedef uint64_t hmm_vptr; /* Virtual  memory address type */

//add by gsg
typedef void * hmm_ptr;
//typedef uint32_t hmm_ptr; /* Address type used by applications and host */

typedef struct hmm_config_s hmm_config_t, *hmm_config;
typedef void (*hmm_invalidate_mmu_tlb)(void);

typedef unsigned long long ull;

hmm_config hmm_create_config(void);
hmm_config hmm_default_config(void);

void     hmm_init_virt_mem(unsigned int ps, hmm_invalidate_mmu_tlb inv_tlb, unsigned int alignment, unsigned int page_numbers_iso_addresses);
void     hmm_init_phys_mem(unsigned int alignment);
hmm_pptr hmm_l1_page_mmu_address(void);
void     hmm_init_l_pages(int enable); /* used for simulations to prevent the mmu from reading
                                         uninitialized data when fetching a block of L2 data. */

void     hmm_cleanup(void);
void     hmm_register_zone(ull host_addr, ull base_addr, ull bytes);
void     hmm_config_register_zone(hmm_config config, ull host_addr, ull base_addr, ull bytes);

const char *hmm_strerror(hmm_error error_number); /* argument should be hmm_errno always */

// Client code should free the pointer after use
const char *hmm_ptr_str(const hmm_ptr ptr);


hmm_ptr hmm_calloc(size_t bytes);
/* allocate physically contiguous memory */
hmm_ptr hmm_calloc_contiguous(size_t bytes);
void    hmm_free(hmm_ptr ptr);

hmm_ptr hmm_config_alloc(hmm_config config, size_t bytes);
hmm_ptr hmm_config_calloc(hmm_config config, size_t bytes);
hmm_ptr hmm_config_alloc_contiguous(hmm_config config, size_t bytes);
hmm_ptr hmm_config_calloc_contiguous(hmm_config config, size_t bytes);

//modify by shuguang
#if 0
hmm_ptr hmm_alloc(size_t bytes);
void     hmm_load(hmm_pptr addr, void *data, unsigned int bytes);
void     hmm_store(hmm_pptr addr, const void *data, unsigned int bytes);
void     hmm_set(hmm_pptr addr, int c, size_t bytes);
#else

int hmm_init(void);
void hmm_cleanup(void);
void *hmm_alloc(size_t bytes, enum hmm_bo_type type,
		int from_highmem, unsigned int userptr, bool cached);
void hmm_free(void *ptr);
int hmm_load(void *virt, void *data, unsigned int bytes);
int hmm_store(void *virt, const void *data, unsigned int bytes);
int hmm_set(void *virt, int c, unsigned int bytes);
#endif

uint8_t  hmm_load_8(hmm_pptr addr);
uint16_t hmm_load_16(hmm_pptr addr);
uint32_t hmm_load_32(hmm_pptr addr);

void     hmm_store_8(hmm_pptr addr, uint8_t data);
void     hmm_store_16(hmm_pptr addr, uint16_t data);
void     hmm_store_32(hmm_pptr addr, uint32_t data);

#if 0
hmm_pptr hmm_virt_to_phys(hmm_vptr virt_addr);
#else
phys_addr_t hmm_virt_to_phys(void *virt);
#endif	

void    hmm_vmem_check(void);

#endif /* _hmm_h_ */
