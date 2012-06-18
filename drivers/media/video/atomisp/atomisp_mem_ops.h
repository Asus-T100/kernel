#ifndef _ATOMISP_MEMOPS_H_
#define _ATOMISP_MEMOPS_H_

#include "atomisp_common.h"

void atomisp_vb2_queue_init(struct vb2_queue *q, const struct vb2_ops *ops,
			    void *priv, enum v4l2_memory memory,
			    enum v4l2_buf_type type, unsigned int msize,
			    unsigned int io_modes);

struct atomisp_malloc_buf {
	void	*vaddr;
	unsigned long	size;
	atomic_t refcount;
	struct vb2_vmarea_handler handler;
};

void *atomisp_vaddr(void *buf_priv);

int frame_mmap(const struct sh_css_frame *frame,
		struct vm_area_struct *vma);

#endif
