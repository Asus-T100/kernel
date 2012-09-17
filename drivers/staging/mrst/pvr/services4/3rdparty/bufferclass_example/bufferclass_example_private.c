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

#include "bufferclass_example.h"
#include "bufferclass_example_private.h"

#define MIN(a,b) ((a)<(b)?(a):(b))

static void FillNV12Image(void *pvDest, int width, int height, int bytestride)
{
	static int iPhase = 0;
	int             i, j;
	unsigned char   u,v,y;
	unsigned char  *pui8y = (unsigned char *)pvDest;
	unsigned short *pui16uv;
	unsigned int    count = 0;

	for(j=0;j<height;j++)
	{
		pui8y = (unsigned char *)pvDest + j * bytestride;
		count = 0;

		for(i=0;i<width;i++)
		{
			y = (((i+iPhase)>>6)%(2)==0)? 0x7f:0x00;

			pui8y[count++] = y;
		}
	}

	for(j=0;j<height;j+=2)
	{
		pui16uv = (unsigned short *)((unsigned char *)pvDest + height * bytestride + (j / 2) * bytestride);
		count = 0;

		for(i=0;i<width;i+=2)
		{
			u = (j<(height/2))? ((i<(width/2))? 0xFF:0x33) : ((i<(width/2))? 0x33:0xAA);
			v = (j<(height/2))? ((i<(width/2))? 0xAC:0x0) : ((i<(width/2))? 0x03:0xEE);


			pui16uv[count++] = (v << 8) | u;

		}
	}

	iPhase++;
}

static void FillI420Image(void *pvDest, int width, int height, int bytestride)
{
	static int iPhase = 0;
	int             i, j;
	unsigned char   u,v,y;
	unsigned char  *pui8y = (unsigned char *)pvDest;
	unsigned char *pui8u, *pui8v;
	unsigned int    count = 0;
	int uvplanestride = bytestride / 2;
	int uvplaneheight = height / 2;

	for(j=0;j<height;j++)
	{
		pui8y = (unsigned char *)pvDest + j * bytestride;
		count = 0;

		for(i=0;i<width;i++)
		{
			y = (((i+iPhase)>>6)%(2)==0)? 0x7f:0x00;

			pui8y[count++] = y;
		}
	}

	for(j=0;j<height;j+=2)
	{
		pui8u = (unsigned char *)pvDest + (height * bytestride) + ((j / 2) * uvplanestride);
		count = 0;

		for(i=0;i<width;i+=2)
		{
			u = (j<(height/2))? ((i<(width/2))? 0xFF:0x33) : ((i<(width/2))? 0x33:0xAA);

			pui8u[count++] = u;

		}
	}

	for(j=0;j<height;j+=2)
	{
		pui8v = (unsigned char *)pvDest + (height * bytestride) + (uvplaneheight * uvplanestride) + (j / 2) * uvplanestride;
		count = 0;

		for(i=0;i<width;i+=2)
		{
			v = (j<(height/2))? ((i<(width/2))? 0xAC:0x0) : ((i<(width/2))? 0x03:0xEE);

			pui8v[count++] = v;
		}
	}

	iPhase++;
}

static void FillYUV422Image(void *pvDest, int width, int height, int bytestride, PVRSRV_PIXEL_FORMAT pixelformat)
{
	static int iPhase = 0;
	int           x, y;
	unsigned char u,v,y0,y1;
	unsigned long *pui32yuv = (unsigned long *)pvDest;
	unsigned int  count = 0;

	for(y=0;y<height;y++)
	{
		pui32yuv = (unsigned long *)((unsigned char *)pvDest + y * bytestride);
		count = 0;

		for(x=0;x<width; x+=2)
		{
			u = (y<(height/2))? ((x<(width/2))? 0xFF:0x33) : ((x<(width/2))? 0x33:0xAA);
			v = (y<(height/2))? ((x<(width/2))? 0xAA:0x0) : ((x<(width/2))? 0x03:0xEE);

			y0 = y1 = (((x+iPhase)>>6)%(2)==0)? 0x7f:0x00;

			switch(pixelformat)
			{
				case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_VYUY:
					pui32yuv[count++] = (y1 << 24) | (u << 16) | (y0 << 8) | v;
					break;
				case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_UYVY:
					pui32yuv[count++] = (y1 << 24) | (v << 16) | (y0 << 8) | u;
					break;
				case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YUYV:
					pui32yuv[count++] = (v << 24) | (y1 << 16) | (u << 8) | y0;
					break;
				case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YVYU:
					pui32yuv[count++] = (u << 24) | (y1 << 16) | (v << 8) | y0;
					break;

				default:
					break;

			}

		}
	}

	iPhase++;
}

static void FillRGB565Image(void *pvDest, int width, int height, int bytestride)
{
	int i, Count;
	unsigned long *pui32Addr  = (unsigned long *)pvDest;
	unsigned short *pui16Addr = (unsigned short *)pvDest;
	unsigned long   Colour32;
	unsigned short  Colour16;
	static unsigned char Colour8 = 0;

	Colour16 = (Colour8>>3) | ((Colour8>>2)<<5) | ((Colour8>>3)<<11);
	Colour32 = Colour16 | Colour16 << 16;

	Count = (height * bytestride)>>2;

	for(i=0; i<Count; i++)
	{
		pui32Addr[i] = Colour32;
	}

	Count =  height;

	pui16Addr = (unsigned short *)((unsigned char *)pvDest + (2 * Colour8));

	for(i=0; i<Count; i++)
	{
		*pui16Addr = 0xF800U;

		pui16Addr = (unsigned short *)((unsigned char *)pui16Addr + bytestride);
	}
	Count = bytestride >> 2;

	pui32Addr = (unsigned long *)((unsigned char *)pvDest + (bytestride * (MIN(height - 1, 0xFF) - Colour8)));

	for(i=0; i<Count; i++)
	{
		pui32Addr[i] = 0x001F001FUL;
	}


	Colour8 = (Colour8 + 1) % MIN(height - 1, 0xFFU);
}


int FillBuffer(unsigned int uiBufferIndex)
{
	BC_EXAMPLE_DEVINFO  *psDevInfo = GetAnchorPtr();
	BC_EXAMPLE_BUFFER   *psBuffer;
	BUFFER_INFO         *psBufferInfo;
	PVRSRV_SYNC_DATA    *psSyncData;


	if(psDevInfo == NULL)
	{
		return -1;
	}

	psBuffer = &psDevInfo->psSystemBuffer[uiBufferIndex];
	psBufferInfo = &psDevInfo->sBufferInfo;


	psSyncData = psBuffer->psSyncData;

	if(psSyncData)
	{

		if(psSyncData->ui32ReadOpsPending != psSyncData->ui32ReadOpsComplete)
		{
			return -1;
		}


		psSyncData->ui32WriteOpsPending++;
	}

	switch(psBufferInfo->pixelformat)
	{
		case PVRSRV_PIXEL_FORMAT_RGB565:
		default:
		{
			FillRGB565Image(psBuffer->sCPUVAddr,
							psBufferInfo->ui32Width,
							psBufferInfo->ui32Height,
							psBufferInfo->ui32ByteStride);
			break;
		}
		case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_VYUY:
		case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_UYVY:
		case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YUYV:
		case PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YVYU:
		{
			FillYUV422Image(psBuffer->sCPUVAddr,
							psBufferInfo->ui32Width,
							psBufferInfo->ui32Height,
							psBufferInfo->ui32ByteStride,
							psBufferInfo->pixelformat);
			break;
		}
		case PVRSRV_PIXEL_FORMAT_NV12:
		{
			FillNV12Image(psBuffer->sCPUVAddr,
							psBufferInfo->ui32Width,
							psBufferInfo->ui32Height,
							psBufferInfo->ui32ByteStride);
			break;
		}
		case PVRSRV_PIXEL_FORMAT_I420:
		{
			FillI420Image(psBuffer->sCPUVAddr,
							psBufferInfo->ui32Width,
							psBufferInfo->ui32Height,
							psBufferInfo->ui32ByteStride);
			break;
		}
	}


	if(psSyncData)
	{
		psSyncData->ui32WriteOpsComplete++;

		if (NULL != psDevInfo->sPVRJTable.pfnPVRSRVScheduleDevices)
		{
			(*psDevInfo->sPVRJTable.pfnPVRSRVScheduleDevices)();
		}
	}

	return 0;
}


int GetBufferCount(unsigned int *puiBufferCount)
{
	BC_EXAMPLE_DEVINFO *psDevInfo = GetAnchorPtr();


	if(psDevInfo == IMG_NULL)
	{
		return -1;
	}


	*puiBufferCount = (unsigned int)psDevInfo->sBufferInfo.ui32BufferCount;

	return 0;
}



int ReconfigureBuffer(unsigned int *uiSucceed)
{
	BCE_ERROR eError;


	eError = BC_Example_Buffers_Destroy();

	if (eError != BCE_OK)
	{
	    *uiSucceed = 0;
		return -1;
	}





	eError = BC_Example_Buffers_Create();

	if (eError != BCE_OK)
	{
	    *uiSucceed = 0;
		return -1;
	}


	*uiSucceed = 1;
	return 0;
}
