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

/* not sure if we need these two for page related macros,
 * need to double check */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <hive_isp_css_mm_hrt.h>
#include "atomisp_internal.h"

#define __page_align(size)	(((size) + (PAGE_SIZE-1)) & (~(PAGE_SIZE-1)))

static unsigned init_done;
static void hrt_isp_css_mm_init(void)
{
	hmm_init();
	init_done = 1;
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
					 " than the expected size..\n");
			else if (num_pages > ((__page_align(bytes))
					      >> PAGE_SHIFT))
				v4l2_err(&atomisp_dev,
					 "user space memory size is"
					 " large than the expected size..\n");

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
				    bool cached)
{
	return __hrt_isp_css_mm_alloc(bytes, userptr, num_pages,
				      HRT_USR_PTR, cached);
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

#if 0
int hrt_isp_css_mm_load_int(void *virt_addr, int *data)
{
	return hrt_isp_css_mm_load(virt_addr, data, sizeof(*data));
}

int hrt_isp_css_mm_load_short(void *virt_addr, short *data)
{
	return hrt_isp_css_mm_load(virt_addr, data, sizeof(*data));
}

int hrt_isp_css_mm_load_char(void *virt_addr, char *data)
{
	return hrt_isp_css_mm_load(virt_addr, data, sizeof(*data));
}

int hrt_isp_css_mm_store_char(void *virt_addr, char data)
{
	return hrt_isp_css_mm_store(virt_addr, &data, sizeof(data));
}

int hrt_isp_css_mm_store_short(void *virt_addr, short data)
{
	return hrt_isp_css_mm_store(virt_addr, &data, sizeof(data));
}

int hrt_isp_css_mm_store_int(void *virt_addr, int data)
{
	return hrt_isp_css_mm_store(virt_addr, &data, sizeof(data));
}
#endif
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

