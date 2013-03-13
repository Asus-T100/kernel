/*
 * ssse3_memcpy_32.c: ssse3 based memcpy
 *
 * (C) Copyright 2011 Intel Corporation
 * Author: Anand Bodas <anand.v.bodas@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/hardirq.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <asm/i387.h>
#include <asm/asm.h>
#include <asm/ssse3.h>

void *_ssse3_memcpy(void *to, const void *from, size_t n)
{
	int d0 = 0;

	if (unlikely(in_interrupt()))
		return __memcpy(to, from, n);
	if (unlikely(system_state != SYSTEM_RUNNING))
		return __memcpy(to, from, n);

	kernel_fpu_begin();
	asm volatile("pushl %0\n\t"
			"pushl %1\n\t"
			"pushl %2\n\t"
			"call _memcpy_ssse3\n\t"
			"popl %3\n\t"
			"popl %3\n\t"
			"popl %3\n\t"
			:
			: "g" (n), "g" (from), "g" (to), "b" (d0)
			: "memory");
	kernel_fpu_end();
	return to;
}
EXPORT_SYMBOL(_ssse3_memcpy);
