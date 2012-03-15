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

#if defined(__linux__)
#include <linux/string.h>
#else
#include <string.h>
#endif

#include "bufferclass_example.h"



#define BUFFERCLASS_DEVICE_NAME "Example Bufferclass Device (SW)"

static void *gpvAnchor = NULL;
static PFN_BC_GET_PVRJTABLE pfnGetPVRJTable = IMG_NULL;

BC_EXAMPLE_DEVINFO * GetAnchorPtr(void)
{
	return (BC_EXAMPLE_DEVINFO *)gpvAnchor;
}

static void SetAnchorPtr(BC_EXAMPLE_DEVINFO *psDevInfo)
{
	gpvAnchor = (void *)psDevInfo;
}


static PVRSRV_ERROR OpenBCDevice(IMG_UINT32 ui32DeviceID, IMG_HANDLE *phDevice)
{
	BC_EXAMPLE_DEVINFO *psDevInfo;




	UNREFERENCED_PARAMETER(ui32DeviceID);

	psDevInfo = GetAnchorPtr();


	*phDevice = (IMG_HANDLE)psDevInfo;

	return (PVRSRV_OK);
}


static PVRSRV_ERROR CloseBCDevice(IMG_UINT32 ui32DeviceID, IMG_HANDLE hDevice)
{
	UNREFERENCED_PARAMETER(hDevice);

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetBCBuffer(IMG_HANDLE          hDevice,
                                IMG_UINT32          ui32BufferNumber,
                                PVRSRV_SYNC_DATA   *psSyncData,
                                IMG_HANDLE         *phBuffer)
{
	BC_EXAMPLE_DEVINFO	*psDevInfo;

	if(!hDevice || !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (BC_EXAMPLE_DEVINFO*)hDevice;

	if( ui32BufferNumber < psDevInfo->sBufferInfo.ui32BufferCount )
	{
		psDevInfo->psSystemBuffer[ui32BufferNumber].psSyncData = psSyncData;
		*phBuffer = (IMG_HANDLE)&psDevInfo->psSystemBuffer[ui32BufferNumber];
	}
	else
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetBCInfo(IMG_HANDLE hDevice, BUFFER_INFO *psBCInfo)
{
	BC_EXAMPLE_DEVINFO	*psDevInfo;

	if(!hDevice || !psBCInfo)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (BC_EXAMPLE_DEVINFO*)hDevice;

	*psBCInfo = psDevInfo->sBufferInfo;

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetBCBufferAddr(IMG_HANDLE      hDevice,
                                    IMG_HANDLE      hBuffer,
                                    IMG_SYS_PHYADDR **ppsSysAddr,
                                    IMG_UINT32      *pui32ByteSize,
                                    IMG_VOID        **ppvCpuVAddr,
                                    IMG_HANDLE      *phOSMapInfo,
                                    IMG_BOOL        *pbIsContiguous,
                                    IMG_UINT32      *pui32TilingStride)
{
	BC_EXAMPLE_BUFFER *psBuffer;

	PVR_UNREFERENCED_PARAMETER(pui32TilingStride);

	if(!hDevice || !hBuffer || !ppsSysAddr || !pui32ByteSize)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psBuffer = (BC_EXAMPLE_BUFFER *) hBuffer;

	*ppvCpuVAddr = psBuffer->sCPUVAddr;

	*phOSMapInfo    = IMG_NULL;
	*pui32ByteSize = (IMG_UINT32)psBuffer->ulSize;

#if defined(BC_DISCONTIG_BUFFERS)
	*ppsSysAddr = psBuffer->psSysAddr;
	*pbIsContiguous = IMG_FALSE;
#else
	*ppsSysAddr = &psBuffer->sPageAlignSysAddr;
	*pbIsContiguous = IMG_TRUE;
#endif

	return (PVRSRV_OK);
}


BCE_ERROR BC_Example_Register(void)
{
	BC_EXAMPLE_DEVINFO	*psDevInfo;











	psDevInfo = GetAnchorPtr();

	if (psDevInfo == NULL)
	{

		psDevInfo = (BC_EXAMPLE_DEVINFO *)BCAllocKernelMem(sizeof(BC_EXAMPLE_DEVINFO));

		if(!psDevInfo)
		{
			return (BCE_ERROR_OUT_OF_MEMORY);
		}


		SetAnchorPtr((void*)psDevInfo);


		psDevInfo->ulRefCount = 0;


		if(BCOpenPVRServices(&psDevInfo->hPVRServices) != BCE_OK)
		{
			return (BCE_ERROR_INIT_FAILURE);
		}
		if(BCGetLibFuncAddr (psDevInfo->hPVRServices, "PVRGetBufferClassJTable", &pfnGetPVRJTable) != BCE_OK)
		{
			return (BCE_ERROR_INIT_FAILURE);
		}


		if(!(*pfnGetPVRJTable)(&psDevInfo->sPVRJTable))
		{
			return (BCE_ERROR_INIT_FAILURE);
		}



		psDevInfo->ulNumBuffers = 0;

		psDevInfo->psSystemBuffer = BCAllocKernelMem(sizeof(BC_EXAMPLE_BUFFER) * BC_EXAMPLE_NUM_BUFFERS);

		if(!psDevInfo->psSystemBuffer)
		{
			return (BCE_ERROR_OUT_OF_MEMORY);
		}


		psDevInfo->sBufferInfo.pixelformat        = PVRSRV_PIXEL_FORMAT_UNKNOWN;
		psDevInfo->sBufferInfo.ui32Width          = 0;
		psDevInfo->sBufferInfo.ui32Height         = 0;
		psDevInfo->sBufferInfo.ui32ByteStride     = 0;
		psDevInfo->sBufferInfo.ui32BufferDeviceID = BC_EXAMPLE_DEVICEID;
		psDevInfo->sBufferInfo.ui32Flags          = 0;
		psDevInfo->sBufferInfo.ui32BufferCount    = (IMG_UINT32)psDevInfo->ulNumBuffers;

		strncpy(psDevInfo->sBufferInfo.szDeviceName, BUFFERCLASS_DEVICE_NAME, MAX_BUFFER_DEVICE_NAME_SIZE);



		psDevInfo->sBCJTable.ui32TableSize    = sizeof(PVRSRV_BC_SRV2BUFFER_KMJTABLE);
		psDevInfo->sBCJTable.pfnOpenBCDevice  = OpenBCDevice;
		psDevInfo->sBCJTable.pfnCloseBCDevice = CloseBCDevice;
		psDevInfo->sBCJTable.pfnGetBCBuffer   = GetBCBuffer;
		psDevInfo->sBCJTable.pfnGetBCInfo     = GetBCInfo;
		psDevInfo->sBCJTable.pfnGetBufferAddr = GetBCBufferAddr;




		if(psDevInfo->sPVRJTable.pfnPVRSRVRegisterBCDevice (&psDevInfo->sBCJTable,
															(IMG_UINT32*)&psDevInfo->ulDeviceID ) != PVRSRV_OK)
		{
			return (BCE_ERROR_DEVICE_REGISTER_FAILED);
		}
	}


	psDevInfo->ulRefCount++;


	return (BCE_OK);
}

BCE_ERROR BC_Example_Unregister(void)
{
	BC_EXAMPLE_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr();


	if (psDevInfo == NULL)
	{
		return (BCE_ERROR_GENERIC);
	}

	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0)
	{

		PVRSRV_BC_BUFFER2SRV_KMJTABLE	*psJTable = &psDevInfo->sPVRJTable;



		if (psJTable->pfnPVRSRVRemoveBCDevice(psDevInfo->ulDeviceID) != PVRSRV_OK)
		{
			return (BCE_ERROR_GENERIC);
		}

		if (BCClosePVRServices(psDevInfo->hPVRServices) != BCE_OK)
		{
			psDevInfo->hPVRServices = NULL;
			return (BCE_ERROR_GENERIC);
		}

		if (psDevInfo->psSystemBuffer)
		{
			BCFreeKernelMem(psDevInfo->psSystemBuffer);
		}


		BCFreeKernelMem(psDevInfo);


		SetAnchorPtr(NULL);
	}


	return (BCE_OK);
}


BCE_ERROR BC_Example_Buffers_Create(void)
{
	BC_EXAMPLE_DEVINFO  *psDevInfo;
	unsigned long        i;
#if !defined(BC_DISCONTIG_BUFFERS)
	IMG_CPU_PHYADDR      sSystemBufferCPUPAddr;
#endif
	PVRSRV_PIXEL_FORMAT  pixelformat    = BC_EXAMPLE_PIXELFORMAT;
	static IMG_UINT32    ui32Width      = BC_EXAMPLE_WIDTH;
	static IMG_UINT32    ui32Height     = BC_EXAMPLE_HEIGHT;
	static IMG_UINT32    ui32ByteStride = BC_EXAMPLE_STRIDE;

	IMG_UINT32 ui32MaxWidth = 320 * 4;



	psDevInfo = GetAnchorPtr();
	if (psDevInfo == NULL)
	{

		return (BCE_ERROR_DEVICE_REGISTER_FAILED);
	}
	if (psDevInfo->ulNumBuffers)
	{

		return (BCE_ERROR_GENERIC);
	}


	psDevInfo->sBufferInfo.pixelformat        = BC_EXAMPLE_PIXELFORMAT;
	psDevInfo->sBufferInfo.ui32Width          = ui32Width;
	psDevInfo->sBufferInfo.ui32Height         = ui32Height;
	psDevInfo->sBufferInfo.ui32ByteStride     = ui32ByteStride;
	psDevInfo->sBufferInfo.ui32BufferDeviceID = BC_EXAMPLE_DEVICEID;
	psDevInfo->sBufferInfo.ui32Flags          = PVRSRV_BC_FLAGS_YUVCSC_FULL_RANGE | PVRSRV_BC_FLAGS_YUVCSC_BT601;

	for(i=psDevInfo->ulNumBuffers; i < BC_EXAMPLE_NUM_BUFFERS; i++)
	{
		unsigned long ulSize = (unsigned long)(ui32Height * ui32ByteStride);

		if(psDevInfo->sBufferInfo.pixelformat == PVRSRV_PIXEL_FORMAT_NV12)
		{

			ulSize += ((ui32ByteStride >> 1) * (ui32Height >> 1) << 1);
		}
		else if(psDevInfo->sBufferInfo.pixelformat == PVRSRV_PIXEL_FORMAT_I420)
		{

			ulSize += (ui32ByteStride >> 1) * (ui32Height >> 1);


			ulSize += (ui32ByteStride >> 1) * (ui32Height >> 1);
		}

#if defined(BC_DISCONTIG_BUFFERS)
		if (BCAllocDiscontigMemory(ulSize,
								   &psDevInfo->psSystemBuffer[i].hMemHandle,
								   &psDevInfo->psSystemBuffer[i].sCPUVAddr,
								   &psDevInfo->psSystemBuffer[i].psSysAddr) != BCE_OK)
		{
			break;
		}
#else

		if (BCAllocContigMemory(ulSize,
		                        &psDevInfo->psSystemBuffer[i].hMemHandle,
		                        &psDevInfo->psSystemBuffer[i].sCPUVAddr,
		                        &sSystemBufferCPUPAddr) != BCE_OK)
		{
			break;
		}
		psDevInfo->psSystemBuffer[i].sSysAddr = CpuPAddrToSysPAddrBC(sSystemBufferCPUPAddr);
		psDevInfo->psSystemBuffer[i].sPageAlignSysAddr.uiAddr = (psDevInfo->psSystemBuffer[i].sSysAddr.uiAddr & 0xFFFFF000);
#endif

		psDevInfo->ulNumBuffers++;

		psDevInfo->psSystemBuffer[i].ulSize = ulSize;
		psDevInfo->psSystemBuffer[i].psSyncData = NULL;
	}

	psDevInfo->sBufferInfo.ui32BufferCount = (IMG_UINT32)psDevInfo->ulNumBuffers;



	psDevInfo->sBCJTable.ui32TableSize    = sizeof(PVRSRV_BC_SRV2BUFFER_KMJTABLE);
	psDevInfo->sBCJTable.pfnOpenBCDevice  = OpenBCDevice;
	psDevInfo->sBCJTable.pfnCloseBCDevice = CloseBCDevice;
	psDevInfo->sBCJTable.pfnGetBCBuffer   = GetBCBuffer;
	psDevInfo->sBCJTable.pfnGetBCInfo     = GetBCInfo;
	psDevInfo->sBCJTable.pfnGetBufferAddr = GetBCBufferAddr;




	if (ui32Width < ui32MaxWidth)
	{
		switch(pixelformat)
		{
		    case PVRSRV_PIXEL_FORMAT_NV12:
		    case PVRSRV_PIXEL_FORMAT_I420:
			{
			    ui32Width += 320;
				ui32Height += 160;
				ui32ByteStride = ui32Width;
				break;
			}
		    case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_VYUY:
		    case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_UYVY:
		    case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YUYV:
		    case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YVYU:
			{
			    ui32Width += 320;
				ui32Height += 160;
				ui32ByteStride = ui32Width*2;
				break;
			}
		    case PVRSRV_PIXEL_FORMAT_RGB565:
			{
			    ui32Width += 320;
				ui32Height += 160;
				ui32ByteStride = ui32Width*2;
				break;
			}
		    default:
			{
				return (BCE_ERROR_INVALID_PARAMS);
			}
		}
	}
	else
	{
		ui32Width      = BC_EXAMPLE_WIDTH;
		ui32Height     = BC_EXAMPLE_HEIGHT;
		ui32ByteStride = BC_EXAMPLE_STRIDE;
	}


	return (BCE_OK);
}


BCE_ERROR BC_Example_Buffers_Destroy(void)
{
	BC_EXAMPLE_DEVINFO *psDevInfo;
	unsigned long       i;

	psDevInfo = GetAnchorPtr();


	if (psDevInfo == NULL)
	{


		return (BCE_ERROR_DEVICE_REGISTER_FAILED);
	}



	for(i = 0; i < psDevInfo->ulNumBuffers; i++)
	{
#if defined(BC_DISCONTIG_BUFFERS)
		BCFreeDiscontigMemory(psDevInfo->psSystemBuffer[i].ulSize,
			 psDevInfo->psSystemBuffer[i].hMemHandle,
			 psDevInfo->psSystemBuffer[i].sCPUVAddr,
			 psDevInfo->psSystemBuffer[i].psSysAddr);
#else
		BCFreeContigMemory(psDevInfo->psSystemBuffer[i].ulSize,
				psDevInfo->psSystemBuffer[i].hMemHandle,
				psDevInfo->psSystemBuffer[i].sCPUVAddr,
				SysPAddrToCpuPAddrBC(psDevInfo->psSystemBuffer[i].sSysAddr));
#endif
	}
	psDevInfo->ulNumBuffers = 0;


	psDevInfo->sBufferInfo.pixelformat        = PVRSRV_PIXEL_FORMAT_UNKNOWN;
	psDevInfo->sBufferInfo.ui32Width          = 0;
	psDevInfo->sBufferInfo.ui32Height         = 0;
	psDevInfo->sBufferInfo.ui32ByteStride     = 0;
	psDevInfo->sBufferInfo.ui32BufferDeviceID = BC_EXAMPLE_DEVICEID;
	psDevInfo->sBufferInfo.ui32Flags          = 0;
	psDevInfo->sBufferInfo.ui32BufferCount    = (IMG_UINT32)psDevInfo->ulNumBuffers;


	return (BCE_OK);
}


BCE_ERROR  BC_Example_Init(void)
{
	BCE_ERROR eError;

	eError = BC_Example_Register();
	if (eError != BCE_OK)
	{
		return eError;
	}

	eError = BC_Example_Buffers_Create();
	if (eError != BCE_OK)
	{
		return eError;
	}

	return (BCE_OK);
}

BCE_ERROR BC_Example_Deinit(void)
{
	BCE_ERROR eError;

	eError = BC_Example_Buffers_Destroy();
	if (eError != BCE_OK)
	{
		return eError;
	}

	eError = BC_Example_Unregister();
	if (eError != BCE_OK)
	{
		return eError;
	}

	return (BCE_OK);
}

