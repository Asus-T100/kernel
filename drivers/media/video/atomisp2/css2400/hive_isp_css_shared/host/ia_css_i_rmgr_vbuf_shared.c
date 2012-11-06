#include "ia_css_i_rmgr.h"

#include <stdbool.h>
#include <assert_support.h>

#include "memory_access.h"

struct ia_css_i_host_rmgr_vbuf_handle handle_table[100];

struct ia_css_i_host_rmgr_vbuf_pool refpool = {
	.copy_on_write = false,
};
struct ia_css_i_host_rmgr_vbuf_pool writepool = {
	.copy_on_write = true,
};

struct ia_css_i_host_rmgr_vbuf_pool *vbuf_ref = &refpool;
struct ia_css_i_host_rmgr_vbuf_pool *vbuf_write = &writepool;



static void ia_css_i_host_refcount_init_vbuf(void)
{
	memset(&handle_table, 0, sizeof(handle_table));
}

void ia_css_i_host_refcount_retain_vbuf(
		struct ia_css_i_host_rmgr_vbuf_handle **handle)
{
	int i;
	struct ia_css_i_host_rmgr_vbuf_handle *h;
	assert(handle != NULL);
	assert(*handle != NULL);
	/* new vbuf to count on */
	if ((*handle)->count == 0) {
		h = *handle;
		*handle = NULL;
		for (i = 0; i < 100; i++) {
			if (handle_table[i].count == 0) {
				*handle = &handle_table[i];
				break;
			}
		}
		assert(*handle != NULL);
		(*handle)->vptr = h->vptr;
		(*handle)->size = h->size;
	}
	(*handle)->count++;
}


void ia_css_i_host_refcount_release_vbuf(
		struct ia_css_i_host_rmgr_vbuf_handle **handle)
{
	assert(handle != NULL);
	assert(*handle != NULL);
	assert((*handle)->count != 0);
	/* decrease reference count */
	(*handle)->count--;
	/* remove from admin */
	if ((*handle)->count == 0) {
		(*handle)->vptr = 0x0;
		(*handle)->size = 0;
		*handle = NULL;
	}
}

void ia_css_i_host_rmgr_init_vbuf(struct ia_css_i_host_rmgr_vbuf_pool *pool)
{
	ia_css_i_host_refcount_init_vbuf();
	(void)pool;
}

void ia_css_i_host_rmgr_uninit_vbuf(struct ia_css_i_host_rmgr_vbuf_pool *pool)
{
	(void)pool;
}

void ia_css_i_host_rmgr_acq_vbuf(
	struct ia_css_i_host_rmgr_vbuf_pool *pool,
	struct ia_css_i_host_rmgr_vbuf_handle **handle)
{
	struct ia_css_i_host_rmgr_vbuf_handle h;
	assert(pool != NULL);
	assert(handle != NULL);
	if (pool->copy_on_write) {
		/* only one reference, reuse (no new retain) */
		if ((*handle)->count == 1)
			return;
		/* more than one reference, release current buffer */
		if ((*handle)->count > 1) {
			/* store current values */
			h.vptr = 0x0;
			h.size = (*handle)->size;
			/* release ref to current buffer */
			ia_css_i_host_refcount_release_vbuf(handle);
			*handle = &h;
		}
		/* get new buffer for needed size */
		if ((*handle)->vptr == 0x0)
			(*handle)->vptr = mmgr_alloc_attr((*handle)->size, 0);
	}
	/* Note that handle will change to an internally maintained one */
	ia_css_i_host_refcount_retain_vbuf(handle);
}

void ia_css_i_host_rmgr_rel_vbuf(
	struct ia_css_i_host_rmgr_vbuf_pool *pool,
	struct ia_css_i_host_rmgr_vbuf_handle **handle)
{
	assert(pool != NULL);
	assert(handle != NULL);
	assert(*handle != NULL);
	/* release the handle */
	ia_css_i_host_refcount_release_vbuf(handle);
	*handle = NULL;
}
