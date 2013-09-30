/* Release Version: ci_master_byt_20130905_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#include "sh_css_refcount.h"
#include "memory_access/memory_access.h"
#include "sh_css_defs.h"

#include "platform_support.h"

#include "assert_support.h"

#include "sh_css_debug.h"

/* TODO: enable for other memory aswell
	 now only for hrt_vaddress */
struct sh_css_refcount_entry {
	uint32_t count;
	hrt_vaddress data;
	int32_t id;
};

struct sh_css_refcount_list {
	uint32_t size;
	struct sh_css_refcount_entry *items;
};

static struct sh_css_refcount_list myrefcount;

int sh_css_refcount_used(void)
{
	uint32_t i;
	int used = 0;
	for (i = 0; i < myrefcount.size; i++) {
		if ((&myrefcount.items[i])->data != mmgr_NULL)
			++used;
	}
	return used;
}

static struct sh_css_refcount_entry *find_entry(hrt_vaddress ptr,
		bool firstfree)
{
	uint32_t i;

	assert(ptr != 0);
	assert(myrefcount.items != NULL);

	for (i = 0; i < myrefcount.size; i++) {

		if ((&myrefcount.items[i])->data == 0) {
			if (firstfree) {
				/* for new entry */
				return &myrefcount.items[i];
			}
		}
		if ((&myrefcount.items[i])->data == ptr) {
			/* found entry */
			return &myrefcount.items[i];
		}
	}
	return NULL;
}

enum ia_css_err sh_css_refcount_init(void)
{
	enum ia_css_err err = IA_CSS_SUCCESS;
	int size = 1000;

	assert(myrefcount.items == NULL);

	myrefcount.items =
		sh_css_malloc(sizeof(struct sh_css_refcount_entry)*size);
	if (!myrefcount.items)
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	if (err == IA_CSS_SUCCESS) {
		memset(myrefcount.items, 0,
			   sizeof(struct sh_css_refcount_entry)*size);
		myrefcount.size = size;
	}
	return err;
}

void sh_css_refcount_uninit(void)
{
	struct sh_css_refcount_entry *entry;
	uint32_t i;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_uninit() enter\n");
	for (i = 0; i < myrefcount.size; i++) {
		entry = &myrefcount.items[i];
		if (entry->data != mmgr_NULL) {
/*			sh_css_dtrace(SH_DBG_TRACE,
				"sh_css_refcount_uninit: freeing (%x)\n",
				entry->data);*/
			mmgr_free(entry->data);
			entry->data = mmgr_NULL;
			entry->count = 0;
			entry->id = 0;
		}
	}
	sh_css_free(myrefcount.items);
	myrefcount.items = NULL;
	myrefcount.size = 0;
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_uninit() leave\n");
}

hrt_vaddress sh_css_refcount_retain(int32_t id, hrt_vaddress ptr)
{
	struct sh_css_refcount_entry *entry;

	entry = find_entry(ptr, false);

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_retain(%x) 0x%x\n", id, ptr);

	if (!entry) {
		entry = find_entry(ptr, true);
		if (entry == NULL) /* should not happen */
			return mmgr_NULL;
		entry->id = id;
	}

	assert(entry->id == id);

	if (entry->data == ptr)
		entry->count += 1;
	else if (entry->data == mmgr_NULL) {
		entry->data = ptr;
		entry->count = 1;
	} else
		return mmgr_NULL;

	return ptr;
}

bool sh_css_refcount_release(int32_t id, hrt_vaddress ptr)
{
	struct sh_css_refcount_entry *entry;

	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_release(%x) 0x%x\n", id, ptr);

	if (ptr == mmgr_NULL)
		return false;

	entry = find_entry(ptr, false);

	if (entry) {
		assert(entry->id == id);
		if (entry->count > 0) {
			entry->count -= 1;
			if (entry->count == 0) {
/*				sh_css_dtrace(SH_DBG_TRACE,
					"sh_css_refcount_release: freeing\n");*/
				mmgr_free(ptr);
				entry->data = mmgr_NULL;
				entry->id = 0;
			}
			return true;
		}
	}

	/* SHOULD NOT HAPPEN: ptr not managed by refcount, or not valid anymore */
	assert(false);

	return false;
}

bool sh_css_refcount_is_single(hrt_vaddress ptr)
{
	struct sh_css_refcount_entry *entry;

	if (ptr == mmgr_NULL)
		return false;

	entry = find_entry(ptr, false);

	if (entry)
		return (entry->count == 1);

	return true;
}

int32_t sh_css_refcount_get_id(hrt_vaddress ptr)
{
	struct sh_css_refcount_entry *entry;
	assert(ptr != mmgr_NULL);
	entry = find_entry(ptr, false);
	assert(entry != NULL);
	if (entry == NULL)  /* should not happen */
		return 0;   /* defensive action */
	return entry->id;
}

void sh_css_refcount_clear(int32_t id, void (*clear_func)(hrt_vaddress ptr))
{
	struct sh_css_refcount_entry *entry;
	uint32_t i;
	uint32_t count = 0;

	assert(clear_func != NULL);
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_clear(%x)\n", id);
	for (i = 0; i < myrefcount.size; i++) {
		entry = &myrefcount.items[i];
		if ((entry->data != mmgr_NULL) && (entry->id == id)) {
			sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_clear:"
					" %x: 0x%x\n", id, entry->data);
			if (clear_func) {
				/* clear using provided function */
				clear_func(entry->data);
			}
			else {
				sh_css_dtrace(SH_DBG_TRACE,
						"sh_css_refcount_clear: "
						"using mmgr_free: no clear_func\n");
				mmgr_free(entry->data);
			}
			assert(entry->count == 0);
			entry->data = mmgr_NULL;
			entry->count = 0;
			entry->id = 0;
			count++;
		}
	}
	sh_css_dtrace(SH_DBG_TRACE, "sh_css_refcount_clear(%x): cleared %d\n",
		id, count);
}
