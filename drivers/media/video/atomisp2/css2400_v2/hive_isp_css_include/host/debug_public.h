/* Release Version: ci_master_byt_20130820_2200 */
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

#ifndef __DEBUG_PUBLIC_H_INCLUDED__
#define __DEBUG_PUBLIC_H_INCLUDED__

#ifndef __KERNEL__
#include "stdbool.h"
#endif

#include "system_types.h"

/*! brief
 *
 * Simple queuing trace buffer for debug data
 * instantiatable in SP DMEM
 *
 * The buffer has a remote and and a local store
 * which contain duplicate data (when in sync).
 * The buffers are automatically synched when the
 * user dequeues, or manualy using the synch function
 *
 * An alternative (storage efficient) implementation
 * could manage the buffers to contain unique data
 *
 * The buffer empty status is computed from local
 * state which does not reflect the presence of data
 * in the remote buffer (unless the alternative
 * implementation is followed)
 */

typedef struct debug_data_s		debug_data_t;
typedef struct debug_data_ddr_s	debug_data_ddr_t;

extern debug_data_t				*debug_data_ptr;
extern hrt_address				debug_buffer_address;
extern hrt_vaddress				debug_buffer_ddr_address;

/*! Check the empty state of the local debug data buffer
 
 \return isEmpty(buffer)
 */
STORAGE_CLASS_DEBUG_H bool is_debug_buffer_empty(void);

/*! Dequeue a token from the debug data buffer
 
 \return isEmpty(buffer)?0:buffer[head]
 */
STORAGE_CLASS_DEBUG_H hrt_data debug_dequeue(void);

/*! Synchronise the remote buffer to the local buffer
 
 \return none
 */
STORAGE_CLASS_DEBUG_H void debug_synch_queue(void);

/*! Synchronise the remote buffer to the local buffer
 
 \return none
 */
STORAGE_CLASS_DEBUG_H void debug_synch_queue_isp(void);


/*! Synchronise the remote buffer to the local buffer
 
 \return none
 */
STORAGE_CLASS_DEBUG_H void debug_synch_queue_ddr(void);

/*! Set the offset/address of the (remote) debug buffer
 
 \return none
 */
extern void debug_buffer_init(
	const hrt_address		addr);

/*! Set the offset/address of the (remote) debug buffer
 
 \return none
 */
extern void debug_buffer_ddr_init(
	const hrt_vaddress		addr);

/*! Set the (remote) operating mode of the debug buffer
 
 \return none
 */
extern void debug_buffer_setmode(
	const debug_buf_mode_t	mode);

#endif /* __DEBUG_PUBLIC_H_INCLUDED__ */
