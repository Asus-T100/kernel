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

#ifndef __ASSERT_SUPPORT_H_INCLUDED__
#define __ASSERT_SUPPORT_H_INCLUDED__

#ifdef __KLOCWORK__
/* Klocwork does not see that assert will lead to abortion
 * as there is no good way to tell this to KW and the code
 * should not depend on assert to function (actually the assert
 * could be disabled in a release build) it was decided to
 * disable the assert for KW scans (by defining NDEBUG)
 * see also: http://www.klocwork.com/products/documentation/current/Tuning_C/C%2B%2B_analysis#Assertions
 */
#define NDEBUG
#endif /* __KLOCWORK__ */

#ifdef NDEBUG

#define assert(cnd) ((void)0)
#define OP___assert(cnd) ((void)0)

#else

#if defined(_MSC_VER)
#include <wdm.h>
#define assert(cnd) ASSERT(cnd)

#define OP___assert(cnd) assert(cnd)
#elif defined(__HIVECC)

/*
 * Enabling assert on cells has too many side effects, it should
 * by default be limited to the unsched CSIM mode, or to only
 * controller type processors. Presently there are not controls
 * in place for that
 */
/* #define OP___assert(cnd) OP___csim_assert(cnd) */
//#if defined(__SP)
//#define OP___assert(cnd) OP___csim_assert(cnd)
//#else
#define OP___assert(cnd) ((void)0)
//#endif

#elif defined(__KERNEL__) /* a.o. Android builds */

#include <linux/bug.h>
#include "sh_css_debug.h"
#define __symbol2value( x ) #x
#define __symbol2string( x ) __symbol2value( x )
#define assert(cnd)							\
	do {								\
		if (!(cnd)) {						\
			sh_css_dtrace(SH_DBG_ERROR, "%s",		\
				"Assertion failed: " #cnd		\
				  ", file " __FILE__			\
				  ", line " __symbol2string( __LINE__ )	\
				  ".\n" );				\
			BUG();						\
		}							\
	} while (0)

#define OP___assert(cnd) assert(cnd)

#elif defined(__FIST__)

#include "assert.h"
#define OP___assert(cnd) assert(cnd)

#elif defined(__GNUC__)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#else /* default is for unknown environments */
#define assert(cnd) ((void)0)
#endif

#endif /* NDEBUG */

#endif /* __ASSERT_SUPPORT_H_INCLUDED__ */
