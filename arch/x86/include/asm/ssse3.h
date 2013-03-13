/*
 * ssse3.h: Header file for sse3 helper functions
 *
 * (C) Copyright 2011 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef _ASM_X86_SSSE3_H
#define _ASM_X86_SSSE3_H

#include <linux/types.h>

extern void *_ssse3_memcpy(void *to, const void *from, size_t size);
extern void *_memcpy_ssse3(void *to, const void *from, size_t n);

#endif /* _ASM_X86_SSSE3_H */
