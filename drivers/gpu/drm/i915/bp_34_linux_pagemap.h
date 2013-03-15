/*  This file contains definitions from kernel 3.6 include/linux/pagemap.h */

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
#error Please remove this file and references to it; backport code no longer needed
#endif

#include <linux/compiler.h>
#include <linux/pagemap.h>


/*
 * Multipage variants of the above prefault helpers, useful if more than
 * PAGE_SIZE of data needs to be prefaulted. These are separate from the above
 * functions (which only handle up to PAGE_SIZE) to avoid clobbering the
 * filemap.c hotpaths.
 */
static inline int fault_in_multipages_writeable(char __user *uaddr, int size)
{
	int ret = 0;
	char __user *end = uaddr + size - 1;

	if (unlikely(size == 0))
		return ret;

	/*
	 * Writing zeroes into userspace here is OK, because we know that if
	 * the zero gets there, we'll be overwriting it.
	 */
	while (uaddr <= end) {
		ret = __put_user(0, uaddr);
		if (ret != 0)
			return ret;
		uaddr += PAGE_SIZE;
	}

	/* Check whether the range spilled into the next page. */
	if (((unsigned long)uaddr & PAGE_MASK) ==
			((unsigned long)end & PAGE_MASK))
		ret = __put_user(0, end);

	return ret;
}

static inline int fault_in_multipages_readable(const char __user *uaddr,
					       int size)
{
	volatile char c;
	int ret = 0;
	const char __user *end = uaddr + size - 1;

	if (unlikely(size == 0))
		return ret;

	while (uaddr <= end) {
		ret = __get_user(c, uaddr);
		if (ret != 0)
			return ret;
		uaddr += PAGE_SIZE;
	}

	/* Check whether the range spilled into the next page. */
	if (((unsigned long)uaddr & PAGE_MASK) ==
			((unsigned long)end & PAGE_MASK)) {
		ret = __get_user(c, end);
		(void)c;
	}

	return ret;
}
