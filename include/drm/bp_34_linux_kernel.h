/*  This file contains definitions from kernel 3.6 include/linux/kernel.h */

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
#error Please remove this file and references to it; backport code no longer needed
#endif


#define SIZE_MAX       (~(size_t)0)
