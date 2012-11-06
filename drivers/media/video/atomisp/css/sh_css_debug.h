/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
*
* Copyright (c) 2010 Intel Corporation. All Rights Reserved.
*
* Copyright (c) 2010 Silicon Hive www.siliconhive.com.
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

#ifndef _SH_CSS_DEBUG_H_
#define _SH_CSS_DEBUG_H_

#include "sh_css_types.h"

/* Global variable which controls the verbosity levels of the debug tracing */
extern unsigned int sh_css_trace_level;

/* The following macro definition allows us to direct the debug trace messages
 * to the Linux kernel trace buffer using the ftrace mechanism. The ftrace
 * mechanism is preferred over the usual prink based messages because it is
 * not bound to a console and its efficiency is many holds higher than the
 * printk mechanism. When we are not in the Linux kernel, we depend upon the
 * CSS print function to dump these traces. We need the __KERNEL__ macro
 * because the same code is compiled also in the Silicon Hive simulation
 * environnment
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#define sh_css_dtrace(level, format, args...)          \
	do {                                           \
		if (sh_css_trace_level >= level)       \
			trace_printk(format, ## args); \
	} while (0)
#else
#define sh_css_dtrace(level, format, args...)          \
	do {                                           \
		if (sh_css_trace_level >= level)       \
			sh_css_print(format, ## args); \
	} while (0)
#endif

/* Function to set the global dtrace verbosity level */
void sh_css_set_dtrace_level(unsigned int trace_level);

void sh_css_dump_if_state(void);
void sh_css_dump_isp_state(void);
void sh_css_dump_sp_state(void);
void sh_css_dump_dma_state(void);
void sh_css_dump_debug_info(const char *context);
void sh_css_dump_isp_gdc_fifo_state(void);
void sh_css_dump_dma_isp_fifo_state(void);
void sh_css_dump_dma_sp_fifo_state(void);
void sh_css_dump_pif_isp_fifo_state(void);
void sh_css_dump_isp_sp_fifo_state(void);
void sh_css_dump_rx_state(void);
void sh_css_frame_print(const struct sh_css_frame *frame,
			const char *descr);

#endif /* _SH_CSS_DEBUG_H_ */
