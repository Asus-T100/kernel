
#include "memory_access.h"

#include <stddef.h>		/* NULL */

#include "device_access.h"

#include "mmu_device.h"

#include "assert_support.h"

/* Presently system specific */
/* #include <hmm_64/hmm.h> */
/* Presently system specific */
#include "hive_isp_css_mm_hrt.h"

/*
 * This is an HRT backend implementation for CSIM
 */

static sys_address		page_table_base_address = (sys_address)-1;
static hrt_data			page_table_base_index = (hrt_data)-1;

const hrt_vaddress	mmgr_NULL = (hrt_vaddress)0;

void mmgr_set_base_address(
	const sys_address		base_addr)
{
	hrt_data	base_index;
	page_table_base_address = base_addr;
/* This is part of "device_access.h", but it may be that "hive_isp_css_mm_hrt.h" requires it */
/* hrt_isp_css_mm_set_ddr_address_offset(offset); */
/* HIVE_ISP_PAGE_SHIFT is a system property, not defined with the MMU */
	base_index = page_table_base_address >> HIVE_ISP_PAGE_SHIFT;
//	mmu_set_page_table_base_index(MMU0_ID, base_index);
return;
}

sys_address mmgr_get_base_address(void)
{
return page_table_base_address;
}

void mmgr_set_base_index(
	const hrt_data		base_index)
{
	page_table_base_index = base_index;
	mmu_set_page_table_base_index(MMU0_ID, base_index);
return;
}

hrt_data mmgr_get_base_index(void)
{
return page_table_base_index;
}

hrt_vaddress mmgr_malloc(
	const size_t			size)
{
return mmgr_alloc_attr(size, MMGR_ATTRIBUTE_DEFAULT);
}

hrt_vaddress mmgr_calloc(
	const size_t			N,
	const size_t			size)
{
return mmgr_alloc_attr(N * size, MMGR_ATTRIBUTE_CLEARED);
}

hrt_vaddress mmgr_realloc(
	hrt_vaddress			vaddr,
	const size_t			size)
{
return mmgr_realloc_attr(vaddr, size, MMGR_ATTRIBUTE_DEFAULT);
}

void mmgr_free(
	hrt_vaddress			vaddr)
{
/* "free()" should accept NULL, "hrt_isp_css_mm_free()" may not */
	if (vaddr != mmgr_NULL)
		hrt_isp_css_mm_free((hmm_ptr)HOST_ADDRESS(vaddr));
return;
}

hrt_vaddress mmgr_alloc_attr(
	const size_t			size,
	const uint16_t			attribute)
{
	hmm_ptr	ptr;
assert(page_table_base_address != (sys_address)-1);
assert((attribute & MMGR_ATTRIBUTE_UNUSED) == 0);
/* assert(attribute == MMGR_ATTRIBUTE_DEFAULT); */
	if (attribute & MMGR_ATTRIBUTE_CLEARED) {
		if (attribute & MMGR_ATTRIBUTE_CACHED) {
			if (attribute & MMGR_ATTRIBUTE_CONTIGUOUS) {
				ptr = hrt_isp_css_mm_calloc_contiguous(size);
			} else {
				ptr = hrt_isp_css_mm_calloc(size);
			}
		} else { /* !MMGR_ATTRIBUTE_CACHED */
			if (attribute & MMGR_ATTRIBUTE_CONTIGUOUS) {
				ptr = hrt_isp_css_mm_calloc_contiguous(size);
			} else {
				ptr = hrt_isp_css_mm_calloc(size);
			}
		}
	} else { /* MMGR_ATTRIBUTE_CLEARED */
		if (attribute & MMGR_ATTRIBUTE_CACHED) {
			if (attribute & MMGR_ATTRIBUTE_CONTIGUOUS) {
				ptr = hrt_isp_css_mm_alloc_contiguous(size);
			} else {
				ptr = hrt_isp_css_mm_alloc(size);
			}
		} else { /* !MMGR_ATTRIBUTE_CACHED */
			if (attribute & MMGR_ATTRIBUTE_CONTIGUOUS) {
				ptr = hrt_isp_css_mm_alloc_contiguous(size);
			} else {
				ptr = hrt_isp_css_mm_alloc(size);
			}
		}
	}
return HOST_ADDRESS(ptr);
}

hrt_vaddress mmgr_realloc_attr(
	hrt_vaddress			vaddr,
	const size_t			size,
	const uint16_t			attribute)
{
assert(page_table_base_address != (sys_address)-1);
assert((attribute & MMGR_ATTRIBUTE_UNUSED) == 0);
/* assert(attribute == MMGR_ATTRIBUTE_DEFAULT); */
/* Apparently we don't have this one */
assert(0);
(void)vaddr;
(void)size;
(void)attribute;
return mmgr_NULL;
}

hrt_vaddress mmgr_mmap(
	const void				*ptr)
{
assert(page_table_base_address != (sys_address)-1);
assert(ptr != NULL);
/*assert(isPageAligned(ptr)); */
/* We don't have this one for sure */
assert(0);
(void)ptr;
return mmgr_NULL;
}

void mmgr_clear(
	hrt_vaddress			vaddr,
	const size_t			size)
{
	mmgr_set(vaddr, (uint8_t)0, size);
}

void mmgr_set(
	hrt_vaddress			vaddr,
	const uint8_t			data,
	const size_t			size)
{
	hrt_isp_css_mm_set((hmm_ptr)HOST_ADDRESS(vaddr), (int)data, size);
return;
}

void mmgr_load(
	const hrt_vaddress		vaddr,
	void					*data,
	const size_t			size)
{
	hrt_isp_css_mm_load((hmm_ptr)HOST_ADDRESS(vaddr), data, size);
return;
}

void mmgr_store(
	const hrt_vaddress		vaddr,
	const void				*data,
	const size_t			size)
{
	hrt_isp_css_mm_store((hmm_ptr)HOST_ADDRESS(vaddr), data, size);
return;
}
