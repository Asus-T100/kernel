/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_drv.h"
#include "drmlfb.h"

/* TODO: revert it back to static, once psb dependency is resolved. */
MRSTLFB_BUFFER **ppsHdmiBuffer = NULL;

static int android_hdmi_alloc_hdmibuffers(struct drm_device *psDrmDevice,
					u32 ui32Size,
					MRSTLFB_BUFFER **ppBuffer);
/* TODO: revert it back to static, once psb dependency is resolved. */
void android_hdmi_free_hdmibuffers(struct drm_device *psDrmDevice,
					MRSTLFB_BUFFER **ppBuffer);

typedef struct {
	int srcWidth;
	int srcAllignedWidth;
	int srcHeight;
	int destWidth;
	int destAllignedWidth;
	int destHeight;

	int wRatioIntVal;
	int wRatioFracVal;
	int hRatioIntVal;
	int hRatioFracVal;
}scalingInfo_t;
static scalingInfo_t sScaleInfo;
#define ANDROID_HDMI_SCALE_PRECISION 1000
/**
 * allocates the HDMI buffers and performs psb_gtt_map_pvr_memory
 * this function is taken from MRSTLFBAllocBuffer() in drmlfb_displayclass.c.
 */
static int android_hdmi_alloc_hdmibuffers(struct drm_device *psDrmDevice,
					u32 ui32Size, MRSTLFB_BUFFER **ppBuffer)
{
	void *pvBuf;
	u32 ulPagesNumber;
	u32 ulCounter;
	int i;

	pvBuf = __vmalloc( ui32Size, GFP_KERNEL | __GFP_HIGHMEM,
			__pgprot((pgprot_val(PAGE_KERNEL ) &
				~_PAGE_CACHE_MASK) | _PAGE_CACHE_WC));
	if (pvBuf == NULL)
	{
		DRM_INFO("%s: Failed to allocate buffer", __func__);
		return -ENOMEM;
	}

	ulPagesNumber = (ui32Size + PAGE_SIZE -1) / PAGE_SIZE;

	*ppBuffer = MRSTLFBAllocKernelMem( sizeof( MRSTLFB_BUFFER ) );
	if (NULL == *ppBuffer) {
		DRM_INFO("%s: Failed to allocate buffer", __func__);
		return -ENOMEM;
	}
	(*ppBuffer)->sCPUVAddr = pvBuf;
	(*ppBuffer)->ui32BufferSize = ui32Size;
	(*ppBuffer)->uSysAddr.psNonCont =
		MRSTLFBAllocKernelMem(sizeof(IMG_SYS_PHYADDR) * ulPagesNumber);
	if (NULL == (*ppBuffer)->uSysAddr.psNonCont) {
		DRM_INFO("%s: Failed to allocate buffer", __func__);
		return -ENOMEM;
	}
	(*ppBuffer)->bIsAllocated = MRST_TRUE;
	(*ppBuffer)->bIsContiguous = MRST_FALSE;
	(*ppBuffer)->ui32OwnerTaskID = task_tgid_nr(current);

	i = 0;
	for (ulCounter = 0; ulCounter < ui32Size; ulCounter += PAGE_SIZE)
	{
		(*ppBuffer)->uSysAddr.psNonCont[i++].uiAddr =
			vmalloc_to_pfn( pvBuf + ulCounter ) << PAGE_SHIFT;
	}

	psb_gtt_map_pvr_memory(psDrmDevice,
			(unsigned int)*ppBuffer,
			(*ppBuffer)->ui32OwnerTaskID,
			(IMG_CPU_PHYADDR*) (*ppBuffer)->uSysAddr.psNonCont,
			ulPagesNumber,
			&(*ppBuffer)->sDevVAddr.uiAddr );

	(*ppBuffer)->sDevVAddr.uiAddr <<= PAGE_SHIFT;

	return 0;
}
/**
 * To free and unmap the HDMI buffers allocated.
 * this function is taken from MRSTLFBFreeBuffer() in drmlfb_displayclass.c.
 */
/* TODO: revert it back to static, once psb dependency is resolved. */
void android_hdmi_free_hdmibuffers(struct drm_device *psDrmDevice,
					MRSTLFB_BUFFER **ppBuffer)
{
	if (!(*ppBuffer)->bIsAllocated)
		return;

	psb_gtt_unmap_pvr_memory( psDrmDevice, (unsigned int)*ppBuffer,
					  (*ppBuffer)->ui32OwnerTaskID);

	vfree( (*ppBuffer)->sCPUVAddr );

	MRSTLFBFreeKernelMem( (*ppBuffer)->uSysAddr.psNonCont );

	MRSTLFBFreeKernelMem( *ppBuffer);

	*ppBuffer = NULL;
}
/**
 * scaling algorithm. This function is specific to RGBA8888 format.
 *	It won't be right for other formats.
 * Input Paramters:
 *	srcAddr: source address
 *	dstAddr: destination address
 *	src_width, src_width_allign, src_height: source buffer width,
 *					aligned width and height.
 *	dest_width, dest_width_allign, dest_height: destination buffer width,
 *					aligned width and height.
 * Return Value:
 *	Nothing.
 */
static void android_hdmi_hdmi_do_scaling(unsigned int srcAddr,
				unsigned int dstAddr,
				int src_width, int src_width_allign,
				int src_height, int dest_width,
				int dest_width_allign, int dest_height)
{
        unsigned int *pu32src = (unsigned int *)srcAddr;
        unsigned int *pu32dst = (unsigned int *)dstAddr;
        unsigned int *pu32src_t, *pu32dst_t;
        unsigned int *pu32srctmp;
        int dw, dh, exp;
	int wStep;
	int hStep = 1;

	/* This is a very simple algorithm.
	 */
        pu32src_t = pu32src;
        pu32dst_t = pu32dst;
        for (dh = 0; dh < dest_height; dh++) {
                pu32dst = pu32dst_t + (dh * dest_width_allign);
		exp = sScaleInfo.hRatioFracVal * dh -
				ANDROID_HDMI_SCALE_PRECISION * hStep;
		if (exp > 0) hStep++;
                pu32src = pu32src_t +
			((dh * sScaleInfo.hRatioIntVal +(hStep - 1)) *
				src_width_allign);
		wStep = 1;
                for (dw = 0; dw < dest_width; dw++) {
			exp = sScaleInfo.wRatioFracVal * dw -
				ANDROID_HDMI_SCALE_PRECISION * wStep;
			if (exp > 0) wStep++;
                        pu32srctmp = pu32src +
				(sScaleInfo.wRatioIntVal * dw + (wStep-1));
                        *pu32dst = *pu32srctmp;
                        pu32dst++;
                }
        }
}
/**
 * Scales the buffer passed as parameter.
 * Input Parameter:
 *	ulAddr: Input Buffer address.
 * Return value:
 *	on Success: Hdmi Buffer address to render.
 *	on Failure: 0.
 */
unsigned long android_hdmi_hdmi_scale_buffer(unsigned long ulAddr)
{
	unsigned long sCPUVAddrHdmi;
	unsigned long sCPUVAddrLVDS;
	static int HdmiBufferIndex = 0;

	/* check whether HDMI buffers are allocated or not */
	if (ppsHdmiBuffer == NULL)
		return 0;

	/* Toggle between the dual buffers. change this logic if
	 * number of hdmi buffers are more than two.
	 */
	HdmiBufferIndex ^= 1;

	/* get the sourc and destination addresses */
	sCPUVAddrHdmi =(unsigned long)ppsHdmiBuffer[HdmiBufferIndex]->sCPUVAddr;
	sCPUVAddrLVDS = ulAddr;

	/* perform scaling.*/
	android_hdmi_hdmi_do_scaling(sCPUVAddrLVDS, sCPUVAddrHdmi,
			sScaleInfo.srcWidth, sScaleInfo.srcAllignedWidth,
			sScaleInfo.srcHeight, sScaleInfo.destWidth,
			sScaleInfo.destAllignedWidth, sScaleInfo.destHeight);

	/* return the hdmi buffer to display */
	return (unsigned long)ppsHdmiBuffer[HdmiBufferIndex]->sDevVAddr.uiAddr;
}
/**
 * Allocates the hdmi buffers of specified dimensions.
 * Initializes Scaling related paramters.
 * input parameters:
 *	psDrmDev: Drm Device.
 *	ui32HdmiWidth, ui32HdmiHeight: HDMI mode
 *	ui32BufferCount,: number of HDMI buffers to allocate.
 *	bpp: bits per pixel to consider for allocating the buffer.
 *	ui32LvdsWidth, ui32LvdsHeight: Native display (LVDS) Mode.
 * returns '0' on success and other values on failure.
 */
int android_hdmi_setup_hdmibuffers(struct drm_device *psDrmDev,
				u32 ui32HdmiWidth, u32 ui32HdmiHeight,
				u32 ui32BufferCount, int bpp, u32 ui32LvdsWidth,
				u32 ui32LvdsHeight)
{
	int i;
	u32 width_align, ui32ByteStride;

	/* clear any previous allocated buffers. This should have been done
	 * as part of unplug processing. Do a safe check one more time.
	 */
	if (ppsHdmiBuffer != NULL) {
		for (i = 0; i < 2; i++)
		{
			if(ppsHdmiBuffer[i] != NULL)
				android_hdmi_free_hdmibuffers(psDrmDev,
						&ppsHdmiBuffer[i]);
		}
		MRSTLFBFreeKernelMem(ppsHdmiBuffer);
		ppsHdmiBuffer = NULL;
	}
	/* allocate MRSTLFB_BUFFER handle */
	ppsHdmiBuffer = (MRSTLFB_BUFFER**)MRSTLFBAllocKernelMem(
				sizeof(MRSTLFB_BUFFER*) * ui32BufferCount);
	if (!ppsHdmiBuffer)
	{
		DRM_INFO("%s falied to allocate ppsHdmiBuffer\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < ui32BufferCount; i++) ppsHdmiBuffer[i] = NULL;
	/* HW alignment requires that the width should be multiple of 32.
	 */
	width_align = (ui32HdmiWidth + 31) & ~31;
	ui32ByteStride = (bpp / 8) * width_align;
	for (i = 0; i < ui32BufferCount; i++)
	{
		unsigned long bufSize = ui32ByteStride * ui32HdmiHeight;
		DRM_INFO("%s: allocating ppsHdmiBuffer[%d] of %lu size",
					__func__, i, bufSize);
		if (android_hdmi_alloc_hdmibuffers(psDrmDev, bufSize,
					&ppsHdmiBuffer[i] ) != MRST_OK ) {
			DRM_INFO("%s falied to allocate hdmi buffers\n",
							__func__);
			goto ErrorFreeBuffersAllocated;
		}
	}
	/* Initialize the scaling related info */
	sScaleInfo.wRatioIntVal = ui32LvdsWidth / ui32HdmiWidth;
	sScaleInfo.wRatioFracVal = ((int)(ui32LvdsWidth *
			ANDROID_HDMI_SCALE_PRECISION / ui32HdmiWidth)) -
		(sScaleInfo.wRatioIntVal * ANDROID_HDMI_SCALE_PRECISION);
	sScaleInfo.hRatioIntVal = ui32LvdsHeight / ui32HdmiHeight;
	sScaleInfo.hRatioFracVal = ((int)(ui32LvdsHeight *
			ANDROID_HDMI_SCALE_PRECISION / ui32HdmiHeight)) -
		(sScaleInfo.hRatioIntVal * ANDROID_HDMI_SCALE_PRECISION);

	sScaleInfo.srcWidth = ui32LvdsWidth;
	sScaleInfo.srcAllignedWidth = (sScaleInfo.srcWidth + 31) & ~31;
	sScaleInfo.srcHeight = ui32LvdsHeight;
	sScaleInfo.destWidth = ui32HdmiWidth;
	sScaleInfo.destAllignedWidth = (sScaleInfo.destWidth + 31) & ~31;
	sScaleInfo.destHeight = ui32HdmiHeight;

	return 0;
ErrorFreeBuffersAllocated:
        for (i = 0; i < ui32BufferCount; i++)
        {
                if(ppsHdmiBuffer[i] != NULL)
                        android_hdmi_free_hdmibuffers(psDrmDev,
							&ppsHdmiBuffer[i]);
        }
        MRSTLFBFreeKernelMem(ppsHdmiBuffer);
	return -ENOMEM;
}
