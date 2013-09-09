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

/****************************************************************
 *
 * Time   : 2012-09-06, 11:16.
 * Author : zhengjie.lu@intel.com
 * Comment:
 * - Initial version.
 *
 ****************************************************************/

#ifndef __SW_EVENT_PUBLIC_H_INCLUDED__
#define __SW_EVENT_PUBLIC_H_INCLUDED__

#ifndef __KERNEL__
#include <stdbool.h>
#endif
#include "system_types.h"

/**
 * @brief Encode the information into the software-event.
 * Encode a certain amount of information into a signel software-event.
 *
 * @param[in]	in	The inputs of the encoder.
 * @param[in]	nr	The number of inputs.
 * @param[out]	out	The output of the encoder.
 *
 * @return true if it is successfull.
 */
STORAGE_CLASS_SW_EVENT_H bool encode_sw_event(
	uint32_t	*in,
	uint32_t	nr,
	uint32_t	*out);
#endif /* __SW_EVENT_PUBLIC_H_INCLUDED__ */

