#include <linux/vmalloc.h>
#include "atomisp_mem_ops.h"
#include "atomisp_cmd.h"
#include "hive_isp_css_mm_hrt.h"
/*
 * videobuf2 memory ops
 */

/*
 * called to free mmap type frame buffer by videobuf2 core
 *
 * @buf_priv: pointer to atomisp_malloc_buf pointer
 * buf->vaddr point to sh_css_frame pointer
 */
static void atomisp_put(void *buf_priv)
{
	struct atomisp_malloc_buf *buf = buf_priv;

	if (atomic_dec_and_test(&buf->refcount)) {
		v4l2_dbg(1, dbg_level, &atomisp_dev,
			"%s: Freeing frame mem at vaddr=%p\n",
			__func__, buf->vaddr);
		if (buf->vaddr)
			sh_css_frame_free(buf->vaddr);
		atomisp_kernel_free(buf);
	}
}

/*
 * called to free usptr type frame buffer by videobuf2 core
 *
 * @buf_priv: pointer to atomisp_malloc_buf pointer
 * buf->vaddr point to sh_css_frame pointer
 */
static void atomisp_put_userptr(void *buf_priv)
{
	struct atomisp_malloc_buf *buf = buf_priv;

	v4l2_dbg(1, dbg_level, &atomisp_dev,
		"%s: Freeing frame mem at vaddr=%p\n",
		__func__, buf->vaddr);
	if (buf->vaddr)
		sh_css_frame_free(buf->vaddr);
	atomisp_kernel_free(buf);
}

/*
 * Be called to allocate per-buffer and frame buffer when memory type
 * is V4L2_MEMORY_MMAP by videobuf2 core.
 * @alloc_ctx: point to allocator private data.
 * @size:      frame image size.
 *
 * return per-buffer if succeed, return NULL if fail.
 */
static void *atomisp_alloc(void *alloc_ctx, unsigned long size)
{
	struct atomisp_frame_info *info = alloc_ctx;
	struct sh_css_frame_info *frame_info = info->frame_info;
	struct sh_css_frame *frame;
	struct atomisp_malloc_buf *buf;

	buf = atomisp_kernel_malloc(sizeof *buf);
	if (!buf)
		return NULL;

	buf->size = size;

	if (sh_css_frame_allocate_from_info(&frame, frame_info))
		return NULL;

	buf->vaddr = frame;

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = atomisp_put;
	buf->handler.arg = buf;

	atomic_inc(&buf->refcount);

	return buf;
}

/*
 * Be called to allocate per-buffer and frame buffer when memory type
 * is V4L2_MEMORY_USERPTR by videobuf2 core.
 * @alloc_ctx: point to allocator private data.
 * @vaddr:     point to the frame buffer virtual address in user space.
 * @size:      frame image size.
 * @write:     v4l2_buf_type is input or output. 0 indicates output type,
 * 1 indicates input type.
 *
 * return per-buffer if succeed, return NULL if fail.
 */
static void *atomisp_get_userptr(void *alloc_ctx, unsigned long vaddr,
		unsigned long size, int write)
{
	struct atomisp_frame_info *info = alloc_ctx;
	struct sh_css_frame_info *frame_info = info->frame_info;
	struct sh_css_frame *frame;
	struct atomisp_malloc_buf *buf;
	unsigned int pgnr;

	buf = atomisp_kernel_malloc(sizeof *buf);
	if (!buf)
		return NULL;

	buf->size = size;
	pgnr = (buf->size + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
#ifdef CONFIG_ION
	hrt_isp_css_mm_set_user_ptr(vaddr, pgnr,
			info->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_ION
			? HRT_USR_ION : HRT_USR_PTR);
#else
	hrt_isp_css_mm_set_user_ptr(vaddr, pgnr, HRT_USR_PTR);
#endif
	if (sh_css_frame_allocate_from_info(&frame, frame_info))
		return NULL;

	buf->vaddr = frame;

	hrt_isp_css_mm_set_user_ptr(0, 0, HRT_USR_PTR);

	return buf;
}

void *atomisp_vaddr(void *buf_priv)
{
	struct atomisp_malloc_buf *buf = buf_priv;

	WARN_ON(!buf);

	if (!buf)
		return NULL;

	if (!buf->vaddr) {
		v4l2_err(&atomisp_dev,
			"Address of an unallocated plane requested\n");
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int atomisp_num_users(void *buf_priv)
{
	struct atomisp_malloc_buf *buf = buf_priv;

	return atomic_read(&buf->refcount);
}

/*
 * Memory help functions for image frame and private parameters
 */
int do_isp_mm_remap(struct vm_area_struct *vma,
		    void *isp_virt, u32 host_virt, u32 pgnr)
{
	u32 pfn;

	while (pgnr) {
		pfn = hmm_virt_to_phys(isp_virt) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, host_virt, pfn,
				    PAGE_SIZE, PAGE_SHARED)) {
			v4l2_err(&atomisp_dev,
				    "remap_pfn_range err.\n");
			return -EAGAIN;
		}

		isp_virt += PAGE_SIZE;
		host_virt += PAGE_SIZE;
		pgnr--;
	}

	return 0;
}

int frame_mmap(const struct sh_css_frame *frame,
		struct vm_area_struct *vma)
{
	void *isp_virt;
	u32 host_virt;
	u32 pgnr;

	if (!frame) {
		v4l2_err(&atomisp_dev,
			    "%s: NULL frame pointer.\n", __func__);
		return -EINVAL;
	}

	host_virt = vma->vm_start;
	isp_virt = frame->data;
	atomisp_get_frame_pgnr(frame, &pgnr);

	if (do_isp_mm_remap(vma, isp_virt, host_virt, pgnr))
		return -EAGAIN;

	return 0;
}

static int atomisp_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct atomisp_malloc_buf *buf = buf_priv;
	int ret;

	if (!buf) {
		v4l2_err(&atomisp_dev, "No memory to map\n");
		return -EINVAL;
	}

	ret = frame_mmap(buf->vaddr, vma);
	if (ret) {
		v4l2_err(&atomisp_dev,
			"mapping frame memory, error: %d\n", ret);
		return ret;
	}

	/*
	 * Make sure that vm_areas for 2 buffers won't be merged together
	 */
	vma->vm_flags		|= VM_DONTEXPAND;

	/*
	 * Use common vm_area operations to track buffer refcount.
	 */
	vma->vm_private_data	= &buf->handler;
	vma->vm_ops		= &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

static void atomisp_put_output(void *buf_priv)
{
	struct atomisp_malloc_buf *buf = buf_priv;

	if (atomic_dec_and_test(&buf->refcount)) {
		v4l2_dbg(1, dbg_level, &atomisp_dev,
			"%s: Freeing vmalloc mem at vaddr=%p\n",
			__func__, buf->vaddr);
		vfree(buf->vaddr);
		atomisp_kernel_free(buf);
	}
}

static void *atomisp_alloc_output(void *alloc_ctx, unsigned long size)
{
	struct atomisp_malloc_buf *buf;

	buf = atomisp_kernel_malloc(sizeof *buf);
	if (!buf)
		return NULL;

	buf->size = size;
	buf->vaddr = vmalloc_user(buf->size);
	buf->handler.refcount = &buf->refcount;
	buf->handler.put = atomisp_put_output;
	buf->handler.arg = buf;

	if (!buf->vaddr) {
		v4l2_err(&atomisp_dev,
			"vmalloc of size %ld failed\n", buf->size);
		atomisp_kernel_free(buf);
		return NULL;
	}

	atomic_inc(&buf->refcount);
	v4l2_dbg(1, dbg_level, &atomisp_dev,
		"Allocated vmalloc buffer of size %ld at vaddr=%p\n",
		buf->size, buf->vaddr);

	return buf;
}

static void atomisp_put_userptr_output(void *buf_priv)
{

}

static void *atomisp_get_userptr_output(void *alloc_ctx, unsigned long vaddr,
		unsigned long size, int write)
{
	/*  reserved */
	return NULL;
}

static int atomisp_mmap_output(void *buf_priv, struct vm_area_struct *vma)
{
	struct atomisp_malloc_buf *buf = buf_priv;
	int ret;

	if (!buf) {
		v4l2_err(&atomisp_dev, "No memory to map\n");
		return -EINVAL;
	}

	ret = remap_vmalloc_range(vma, buf->vaddr, 0);
	if (ret) {
		v4l2_err(&atomisp_dev,
			"Remapping vmalloc memory, error: %d\n", ret);
		return ret;
	}

	/*
	 * Make sure that vm_areas for 2 buffers won't be merged together
	 */
	vma->vm_flags		|= VM_DONTEXPAND;

	/*
	 * Use common vm_area operations to track buffer refcount.
	 */
	vma->vm_private_data	= &buf->handler;
	vma->vm_ops		= &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

const struct vb2_mem_ops atomisp_memops = {
	.alloc		= atomisp_alloc,
	.put		= atomisp_put,
	.get_userptr	= atomisp_get_userptr,
	.put_userptr	= atomisp_put_userptr,
	.vaddr		= atomisp_vaddr,
	.mmap		= atomisp_mmap,
	.num_users	= atomisp_num_users,
};
EXPORT_SYMBOL_GPL(atomisp_memops);

const struct vb2_mem_ops atomisp_memops_output = {
	.alloc		= atomisp_alloc_output,
	.put		= atomisp_put_output,
	.get_userptr	= atomisp_get_userptr_output,
	.put_userptr	= atomisp_put_userptr_output,
	.vaddr		= atomisp_vaddr,
	.mmap		= atomisp_mmap_output,
	.num_users	= atomisp_num_users,
};
EXPORT_SYMBOL_GPL(atomisp_memops_output);

/*
 * Initialize vb2_queue statically.
 */
void atomisp_vb2_queue_init(struct vb2_queue *q,
			const struct vb2_ops *ops,
			void *priv,
			enum v4l2_memory memory,
			enum v4l2_buf_type type,
			unsigned int msize,
			unsigned int io_modes)
{
	BUG_ON(!q);
	memset(q, 0, sizeof(struct vb2_queue));
	q->type = type;
	q->io_modes = io_modes;
	q->drv_priv = priv;
	q->buf_struct_size = msize;
	q->memory = memory;
	q->ops = ops;
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		q->mem_ops = &atomisp_memops;
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		q->mem_ops = &atomisp_memops_output;

	vb2_queue_init(q);
}
