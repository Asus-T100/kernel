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

#ifndef __IA_CSS_I_RMGR_GLOBAL_H_INCLUDED__
#define __IA_CSS_I_RMGR_GLOBAL_H_INCLUDED__

#include "storage_class.h"

#ifndef __INLINE_RMGR__
#define STORAGE_CLASS_RMGR_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_RMGR_C
#else  /* __INLINE_RMGR__ */
#define STORAGE_CLASS_RMGR_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_RMGR_C STORAGE_CLASS_INLINE
#endif /* __INLINE_RMGR__ */

/* definition of init uninit function that needs to be implemented
 * per system?
 */
#define IA_CSS_I_RMGR_INIT(proc) \
void ia_css_i_ ## proc ## _rmgr_init(void); \
\
void ia_css_i_ ## proc ## _rmgr_uninit(void);

/*
 * macro to define the interface for resource type
 */
#define IA_CSS_I_RMGR_TYPE(proc, type) \
\
struct ia_css_i_ ## proc ## _rmgr_ ## type ## _pool; \
\
struct ia_css_i_ ## proc ## _rmgr_ ## type ## _handle; \
\
STORAGE_CLASS_RMGR_H void ia_css_i_ ## proc ## _rmgr_init_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _pool *pool); \
\
STORAGE_CLASS_RMGR_H void ia_css_i_ ## proc ## _rmgr_uninit_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _pool *pool); \
\
STORAGE_CLASS_RMGR_H void ia_css_i_ ## proc ## _rmgr_acq_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _pool *pool, \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _handle **handle); \
\
STORAGE_CLASS_RMGR_H void ia_css_i_ ## proc ## _rmgr_rel_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _pool *pool, \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _handle **handle); \

/*
 * macro to define the interface for refcounting
 */
#define IA_CSS_I_REFCOUNT(proc, type) \
void ia_css_i_ ## proc ## _refcount_retain_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _handle **handle); \
\
void ia_css_i_ ## proc ## _refcount_release_ ## type( \
	struct ia_css_i_ ## proc ## _rmgr_ ## type ## _handle **handle);


#include "ia_css_i_rmgr_public.h"


#endif /* __IA_CSS_I_RMGR_GLOBAL_H_INCLUDED__ */
