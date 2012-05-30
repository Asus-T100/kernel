/*
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "psb_drv.h"
#include "services_headers.h"
#include "bufferclass_video.h"

int BCGetBuffer(int devId, int bufferId, BC_VIDEO_BUFFER **bc_buffer);
int BCGetDeviceBufferCount(int devId);
int BCGetDeviceStride(int devId);

extern int psb_get_meminfo_by_handle(IMG_HANDLE hKernelMemInfo,
				PVRSRV_KERNEL_MEM_INFO **ppsKernelMemInfo);
extern IMG_UINT32 psb_get_tgid(void);
extern int psb_get_pages_by_mem_handle(IMG_HANDLE hOSMemHandle,
					u32 **pfn_list,
					int page_count);
extern int psb_get_bcd_pages(u32 device_id, u32 buffer_id, u32 **pfn_list,
				int *pages);
extern int psb_get_vaddr_pages(u32 vaddr, u32 size, u32 **pfn_list, int *pages);
extern int psb_get_bcd_buffer_count(uint32_t bcd_id);
extern int psb_get_bcd_buffer_stride(uint32_t bcd_id);
