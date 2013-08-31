/* Release Version: ci_master_byt_20130823_2200 */
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

#ifndef _SH_CSS_REFCOUNT_H_
#define _SH_CSS_REFCOUNT_H_

#include "ia_css.h"
#include "sh_css_binary.h"
#include "sh_css_internal.h"

#define PARAM_SET_POOL  0xCAFE0001
#define PARAM_BUFFER    0xCAFE0002

enum ia_css_err sh_css_refcount_init(void);

void sh_css_refcount_uninit(void);

hrt_vaddress sh_css_refcount_retain(int32_t id, hrt_vaddress ptr);

bool sh_css_refcount_release(int32_t id, hrt_vaddress ptr);

bool sh_css_refcount_is_single(hrt_vaddress ptr);

int32_t sh_css_refcount_get_id(hrt_vaddress ptr);

void sh_css_refcount_clear(int32_t id, void (*clear_func)(hrt_vaddress ptr));

int sh_css_refcount_used(void);

#endif
