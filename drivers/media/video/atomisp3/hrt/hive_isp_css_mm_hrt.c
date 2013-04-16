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
#include <hmm/hmm.h>
#include <ia_css.h>
/* not sure if we need these two for page related macros,
 * need to double check */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <hive_isp_css_mm_hrt.h>
#include "atomisp_internal.h"

#define __page_align(size)	(((size) + (PAGE_SIZE-1)) & (~(PAGE_SIZE-1)))

static unsigned init_done;
void hrt_isp_css_mm_init(void)
{
	if (!init_done) {
		hmm_init();
		init_done = 1;
	}
}

int hrt_isp_get_mmu_base_address(void)
{
	if (init_done)
		return hmm_get_mmu_base_addr();
	return -EINVAL;

}
int hrt_isp_css_mm_set(void *virt_addr, int c, size_t bytes)
{
	if (virt_addr)
		return hmm_set(virt_addr, c, bytes);

	return -EINVAL;
}

int hrt_isp_css_mm_load(void *virt_addr, void *data, size_t bytes)
{
	if (virt_addr)
		return hmm_load(virt_addr, data, bytes);
	return -EINVAL;
}

int hrt_isp_css_mm_store(void *virt_addr, const void *data, size_t bytes)
{
	if (virt_addr)
		return hmm_store(virt_addr, data, bytes);
	return -EINVAL;
}

void hrt_isp_css_mm_free(void *virt_addr)
{
	if (virt_addr)
		hmm_free(virt_addr);
}

void hrt_isp_css_mm_clear(void)
{
	if (init_done) {
		hmm_cleanup();
		init_done = 0;
	}
}

static unsigned int my_userptr, my_num_pages;
static enum hrt_userptr_type my_usr_type;

void hrt_isp_css_mm_set_user_ptr(unsigned int userptr,
				 unsigned int num_pages,
				 enum hrt_userptr_type type)
{
	my_userptr = userptr;
	my_num_pages = num_pages;
	my_usr_type = type;
}

static void *__hrt_isp_css_mm_alloc(size_t bytes, unsigned int userptr,
				    unsigned int num_pages,
				    enum hrt_userptr_type type,
				    bool cached)
{
	if (!init_done)
		hrt_isp_css_mm_init();
#ifdef CONFIG_ION
	if (type == HRT_USR_ION)
		return (void *)hmm_alloc(bytes, HMM_BO_ION, 0,
					 userptr, cached);

#endif
	if (type == HRT_USR_PTR) {
		if (userptr == 0)
			return (void *)hmm_alloc(bytes, HMM_BO_PRIVATE, 0,
						 0, cached);
		else {
			if (num_pages < ((__page_align(bytes)) >> PAGE_SHIFT))
				v4l2_err(&atomisp_dev,
					 "user space memory size is less"
					 " than the expected size)..\n");
			else if (num_pages > ((__page_align(bytes))
					      >> PAGE_SHIFT))
				v4l2_err(&atomisp_dev,
					 "user space memory size is"
					 " large than the expected size)..\n");

			return (void *)hmm_alloc(bytes, HMM_BO_USER, 0,
						 userptr, cached);
		}
	} else {
		v4l2_err(&atomisp_dev, "user ptr type is incorrect.\n");
		return NULL;
	}
}

void *hrt_isp_css_mm_alloc(size_t bytes)
{
	return __hrt_isp_css_mm_alloc(bytes, my_userptr,
				      my_num_pages, my_usr_type, false);
}

void *hrt_isp_css_mm_alloc_user_ptr(size_t bytes, unsigned int userptr,
				    unsigned int num_pages,
				    enum hrt_userptr_type type,
				    bool cached)
{
	return __hrt_isp_css_mm_alloc(bytes, userptr, num_pages,
				      type, cached);
}

void *hrt_isp_css_mm_alloc_cached(size_t bytes)
{
	if (!init_done)
		hrt_isp_css_mm_init();

	if (my_userptr == 0)
		return (void *)hmm_alloc(bytes, HMM_BO_PRIVATE, 0, 0,
						HMM_CACHED);
	else {
		if (my_num_pages < ((__page_align(bytes)) >> PAGE_SHIFT))
			v4l2_err(&atomisp_dev,
					"user space memory size is less"
					" than the expected size..\n");
		else if (my_num_pages > ((__page_align(bytes)) >> PAGE_SHIFT))
			v4l2_err(&atomisp_dev,
					"user space memory size is"
					" large than the expected size..\n");

		return (void *)hmm_alloc(bytes, HMM_BO_USER, 0,
						my_userptr, HMM_CACHED);
	}
}

void *hrt_isp_css_mm_calloc(size_t bytes)
{
	void *ptr = hrt_isp_css_mm_alloc(bytes);
	if (!ptr)
		hmm_set(ptr, 0, bytes);
	return ptr;
}

void *hrt_isp_css_mm_calloc_cached(size_t bytes)
{
	void *ptr = hrt_isp_css_mm_alloc_cached(bytes);
	if (!ptr)
		hmm_set(ptr, 0, bytes);
	return ptr;
}

void *hrt_isp_css_virt_to_phys(void *virt_addr)
{
	return (void *)(u32)hmm_virt_to_phys(virt_addr);
}

void *hrt_isp_css_mm_alloc_contiguous(size_t bytes)
{
	BUG_ON(false);
	return NULL;
}
void *hrt_isp_css_mm_calloc_contiguous(size_t bytes)
{
	BUG_ON(false);
	return NULL;
}
#ifdef ATOMISP_CSS2
ia_css_ptr
atomisp_css2_mm_alloc(size_t bytes, uint32_t attr)
{
	if (attr & IA_CSS_MEM_ATTR_CACHED)
		return (ia_css_ptr) hrt_isp_css_mm_calloc_cached(bytes);
	else if (attr & IA_CSS_MEM_ATTR_ZEROED)
		return (ia_css_ptr) hrt_isp_css_mm_calloc(bytes);
	else if (attr & IA_CSS_MEM_ATTR_CONTIGUOUS)
		return (ia_css_ptr) hrt_isp_css_mm_calloc_contiguous(bytes);
	else
		return (ia_css_ptr) hrt_isp_css_mm_calloc(bytes);
}

void atomisp_css2_mm_free(ia_css_ptr ptr)
{
	hrt_isp_css_mm_free((void *)ptr);
}
int atomisp_css2_mm_load(ia_css_ptr ptr, void *data, size_t bytes)
{
	return hrt_isp_css_mm_load((void *)ptr, data, bytes);
}
int atomisp_css2_mm_store(ia_css_ptr ptr, const void *data, size_t bytes)
{
	return hrt_isp_css_mm_store((void *)ptr, data, bytes);
}
int atomisp_css2_mm_set(ia_css_ptr ptr, int c, size_t bytes)
{
	return hrt_isp_css_mm_set((void *)ptr, c, bytes);
}

ia_css_ptr atomisp_css2_mm_mmap(const void *ptr, const size_t size,
		   uint16_t attribute, void *context)
{
	struct hrt_userbuffer_attr *userbuffer_attr = context;
	return (ia_css_ptr)hrt_isp_css_mm_alloc_user_ptr(
			size, (unsigned int)ptr,
			userbuffer_attr->pgnr,
			userbuffer_attr->type,
			attribute & HRT_BUF_FLAG_CACHED);
}
void atomisp_css2_hw_store_8(hrt_address addr, uint8_t data)
{
	_hrt_master_port_store_8(addr, data);
}
void atomisp_css2_hw_store_16(hrt_address addr, uint16_t data)
{
	_hrt_master_port_store_16(addr, data);
}
void atomisp_css2_hw_store_32(hrt_address addr, uint32_t data)
{
	_hrt_master_port_store_32(addr, data);
}
uint8_t atomisp_css2_hw_load_8(hrt_address addr)
{
	return _hrt_master_port_load_8(addr);
}
uint16_t atomisp_css2_hw_load_16(hrt_address addr)
{
	return _hrt_master_port_load_16(addr);
}
uint32_t atomisp_css2_hw_load_32(hrt_address addr)
{
	return _hrt_master_port_load_32(addr);
}

void atomisp_css2_hw_store(hrt_address addr, const void *from, uint32_t n)
{
	unsigned i;
	unsigned int _to = (unsigned int)addr;
	const char *_from = (const char *)from;
	for (i = 0; i < n; i++, _to++, _from++)
		_hrt_master_port_store_8(_to , *_from);
}

void atomisp_css2_hw_load(hrt_address addr, void *to, uint32_t n)
{
	unsigned i;
	char *_to = (char *)to;
	unsigned int _from = (unsigned int)addr;
	for (i = 0; i < n; i++, _to++, _from++)
		*_to = _hrt_master_port_load_8(_from);
}
#endif
