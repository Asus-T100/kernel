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

#ifndef __CSC_KERNEL_GLOBAL_H_INCLUDED__
#define __CSC_KERNEL_GLOBAL_H_INCLUDED__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/* Can we pass a pointer (that can be resolved to a constant) without ill effects */
//typedef const struct csc_kernel_param_s	*csc_kernel_param_h;
/* Else, can we pass a register struct (defined in csc_kernel_local.h) */

/* Else, can we only pass an ID / index ? */
typedef csc_kernel_param_set_t			csc_kernel_param_h;

typedef struct csc_kernel_param_s		csc_kernel_param_t;

struct csc_kernel_param_s {
	uint16_t	m_shift;
	int16_t		m00;
	int16_t		m01;
	int16_t		m02;
	int16_t		m10;
	int16_t		m11;
	int16_t		m12;
	int16_t		m20;
	int16_t		m21;
	int16_t		m22;
};

#endif /* __CSC_KERNEL_GLOBAL_H_INCLUDED__ */
