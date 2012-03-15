/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#ifndef __BC_EXAMPLE_H__
#define __BC_EXAMPLE_H__

#include "img_defs.h"
#include "servicesext.h"
#include "kernelbuffer.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define BC_EXAMPLE_NUM_BUFFERS  3

#define NV12 1
#ifdef NV12

#define BC_EXAMPLE_WIDTH        (320)
#define BC_EXAMPLE_HEIGHT       (160)
#define BC_EXAMPLE_STRIDE       (320)
#define BC_EXAMPLE_PIXELFORMAT	(PVRSRV_PIXEL_FORMAT_NV12)

#else
#ifdef I420

#define BC_EXAMPLE_WIDTH        (320)
#define BC_EXAMPLE_HEIGHT       (160)
#define BC_EXAMPLE_STRIDE       (320)
#define BC_EXAMPLE_PIXELFORMAT	(PVRSRV_PIXEL_FORMAT_I420)

#else
#ifdef YUV422

#define BC_EXAMPLE_WIDTH        (320)
#define BC_EXAMPLE_HEIGHT       (160)
#define BC_EXAMPLE_STRIDE       (320*2)
#define BC_EXAMPLE_PIXELFORMAT	(PVRSRV_PIXEL_FORMAT_FOURCC_ORG_UYVY)

#else

#define BC_EXAMPLE_WIDTH        (320)
#define BC_EXAMPLE_HEIGHT       (160)
#define BC_EXAMPLE_STRIDE       (320*2)
#define BC_EXAMPLE_PIXELFORMAT  (PVRSRV_PIXEL_FORMAT_RGB565)

#endif
#endif
#endif

#define BC_EXAMPLE_DEVICEID      0

typedef void *       BCE_HANDLE;

typedef enum tag_bce_bool
{
	BCE_FALSE = 0,
	BCE_TRUE  = 1,
} BCE_BOOL, *BCE_PBOOL;

typedef struct BC_EXAMPLE_BUFFER_TAG
{
	unsigned long           ulSize;
	BCE_HANDLE              hMemHandle;



#if defined(BC_DISCONTIG_BUFFERS)
	IMG_SYS_PHYADDR				*psSysAddr;
#else
	IMG_SYS_PHYADDR				sSysAddr;
	IMG_SYS_PHYADDR         sPageAlignSysAddr;
#endif
	IMG_CPU_VIRTADDR        sCPUVAddr;
	PVRSRV_SYNC_DATA        *psSyncData;

	struct BC_EXAMPLE_BUFFER_TAG *psNext;
} BC_EXAMPLE_BUFFER;


typedef struct BC_EXAMPLE_DEVINFO_TAG
{
	unsigned long           ulDeviceID;

	BC_EXAMPLE_BUFFER       *psSystemBuffer;


	unsigned long           ulNumBuffers;


	PVRSRV_BC_BUFFER2SRV_KMJTABLE sPVRJTable;


	PVRSRV_BC_SRV2BUFFER_KMJTABLE sBCJTable;




	BCE_HANDLE              hPVRServices;


	unsigned long           ulRefCount;



	BUFFER_INFO             sBufferInfo;

}  BC_EXAMPLE_DEVINFO;


typedef enum _BCE_ERROR_
{
	BCE_OK                             =  0,
	BCE_ERROR_GENERIC                  =  1,
	BCE_ERROR_OUT_OF_MEMORY            =  2,
	BCE_ERROR_TOO_FEW_BUFFERS          =  3,
	BCE_ERROR_INVALID_PARAMS           =  4,
	BCE_ERROR_INIT_FAILURE             =  5,
	BCE_ERROR_CANT_REGISTER_CALLBACK   =  6,
	BCE_ERROR_INVALID_DEVICE           =  7,
	BCE_ERROR_DEVICE_REGISTER_FAILED   =  8,
	BCE_ERROR_NO_PRIMARY               =  9
} BCE_ERROR;


#ifndef UNREFERENCED_PARAMETER
#define	UNREFERENCED_PARAMETER(param) (param) = (param)
#endif

#ifndef NULL
#define NULL 0
#endif

BCE_ERROR BC_Example_Register(void);
BCE_ERROR BC_Example_Unregister(void);
BCE_ERROR BC_Example_Buffers_Create(void);
BCE_ERROR BC_Example_Buffers_Destroy(void);
BCE_ERROR BC_Example_Init(void);
BCE_ERROR BC_Example_Deinit(void);

BCE_ERROR BCOpenPVRServices(BCE_HANDLE *phPVRServices);
BCE_ERROR BCClosePVRServices(BCE_HANDLE hPVRServices);

void *BCAllocKernelMem(unsigned long ulSize);
void BCFreeKernelMem(void *pvMem);
#if defined(BC_DISCONTIG_BUFFERS)
BCE_ERROR BCAllocDiscontigMemory(unsigned long ulSize,
                              BCE_HANDLE unref__ *phMemHandle,
                              IMG_CPU_VIRTADDR *pLinAddr,
                              IMG_SYS_PHYADDR **ppPhysAddr);

void BCFreeDiscontigMemory(unsigned long ulSize,
                         BCE_HANDLE unref__ hMemHandle,
                         IMG_CPU_VIRTADDR LinAddr,
                         IMG_SYS_PHYADDR *pPhysAddr);

#else

BCE_ERROR BCAllocContigMemory(unsigned long    ulSize,
                              BCE_HANDLE       *phMemHandle,
                              IMG_CPU_VIRTADDR *pLinAddr,
                              IMG_CPU_PHYADDR  *pPhysAddr);

void BCFreeContigMemory(unsigned long ulSize,
                        BCE_HANDLE hMemHandle,
                        IMG_CPU_VIRTADDR LinAddr,
                        IMG_CPU_PHYADDR PhysAddr);
#endif

IMG_SYS_PHYADDR CpuPAddrToSysPAddrBC(IMG_CPU_PHYADDR cpu_paddr);
IMG_CPU_PHYADDR SysPAddrToCpuPAddrBC(IMG_SYS_PHYADDR sys_paddr);

void *MapPhysAddr(IMG_SYS_PHYADDR sSysAddr, unsigned long ulSize);
void UnMapPhysAddr(void *pvAddr, unsigned long ulSize);

BCE_ERROR BCGetLibFuncAddr (BCE_HANDLE hExtDrv, char *szFunctionName, PFN_BC_GET_PVRJTABLE *ppfnFuncTable);
BC_EXAMPLE_DEVINFO * GetAnchorPtr(void);

#if defined(__cplusplus)
}
#endif

#endif

