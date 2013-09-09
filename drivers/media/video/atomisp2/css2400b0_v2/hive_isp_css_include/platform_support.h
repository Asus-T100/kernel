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

#ifndef __PLATFORM_SUPPORT_H_INCLUDED__
#define __PLATFORM_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER)
#include "storage_class.h"
/*
 * Put here everything _MSC_VER specific not covered in
 * "assert_support.h", "math_support.h", etc
 */
STORAGE_CLASS_INLINE void
hrt_sleep(void)
{
	/* Empty for now. Polling is not used in many places */
}

#elif defined(__HIVECC)
/*
 * Put here everything __HIVECC specific not covered in
 * "assert_support.h", "math_support.h", etc
 */
#include "hrt/host.h"

#elif defined(__KERNEL__)
#include "storage_class.h"
#include <linux/delay.h>
STORAGE_CLASS_INLINE void hrt_sleep(void)
{
       udelay(1);
}
/*
 * Put here everything __KERNEL__ specific not covered in
 * "assert_support.h", "math_support.h", etc
 */
#elif defined(__FIST__)
/*
 * Put here everything __FIST__ specific not covered in
 * "assert_support.h", "math_support.h", etc
 */

#elif defined(__GNUC__)
/*
 * Put here everything __GNUC__ specific not covered in
 * "assert_support.h", "math_support.h", etc
 */
#include "hrt/host.h"


#else /* default is for unknwn environments */
/*
 * Put here everything specific not covered in
 * "assert_support.h", "math_support.h", etc
 */

#endif

#endif /* __PLATFORM_SUPPORT_H_INCLUDED__ */
