/*  This file contains definitions from kernel 3.6 drivers/base/dma-buf.c */

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
#error Please remove this file and references to it; backport code no longer needed
#endif

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/anon_inodes.h>
#include <linux/export.h>


/**
 * dma_buf_vmap - Create virtual mapping for the buffer object into kernel
 * address space. Same restrictions as for vmap and friends apply.
 * @dmabuf:	[in]	buffer to vmap
 *
 * This call may fail due to lack of virtual mapping address space.
 * These calls are optional in drivers. The intended use for them
 * is for mapping objects linear in kernel space for high use objects.
 * Please attempt to use kmap/kunmap before thinking about these interfaces.
 */
void *dma_buf_vmap(struct dma_buf *dmabuf)
{
	if (WARN_ON(!dmabuf))
		return NULL;

	if (dmabuf->ops->vmap)
		return dmabuf->ops->vmap(dmabuf);
	return NULL;
}
EXPORT_SYMBOL_GPL(dma_buf_vmap);

/**
 * dma_buf_vunmap - Unmap a vmap obtained by dma_buf_vmap.
 * @dmabuf:	[in]	buffer to vunmap
 * @vaddr:	[in]	vmap to vunmap
 */
void dma_buf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	if (WARN_ON(!dmabuf))
		return;

	if (dmabuf->ops->vunmap)
		dmabuf->ops->vunmap(dmabuf, vaddr);
}
EXPORT_SYMBOL_GPL(dma_buf_vunmap);
