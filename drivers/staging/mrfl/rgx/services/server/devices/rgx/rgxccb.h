/*************************************************************************/ /*!
@File
@Title          RGX Circular Command Buffer functionality.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX Circular Command Buffer functionality.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(__RGXCCB_H__)
#define __RGXCCB_H__

#include "devicemem.h"
#include "device.h"

#if defined (__cplusplus)
extern "C" {
#endif

typedef struct {
	DEVMEM_MEMDESC *psClientCCBMemDesc;
	DEVMEM_MEMDESC *psClientCCBCtlMemDesc;
	DEVMEM_EXPORTCOOKIE sClientCCBExportCookie;
	DEVMEM_EXPORTCOOKIE sClientCCBCtlExportCookie;
} RGX_CCB_CLEANUP_DATA;

/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateCCBKM

 @Description
	Server-side implementation of RGXCreateCCB

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_IMPORT
PVRSRV_ERROR PVRSRVRGXCreateCCBKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
								  IMG_UINT32			ui32AllocSize,
								  IMG_UINT32			ui32AllocAlignment,
								  RGX_CCB_CLEANUP_DATA	**ppsCleanupData,
								  DEVMEM_MEMDESC 		**ppsClientCCBMemDesc,
								  DEVMEM_MEMDESC 		**ppsClientCCBCtlMemDesc,
								  DEVMEM_EXPORTCOOKIE 	**psClientCCBExportCookie,
								  DEVMEM_EXPORTCOOKIE 	**psClientCCBCtlExportCookie);


/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyCCBKM

 @Description
	Server-side implementation of RGXDestroyCCB

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_IMPORT
PVRSRV_ERROR PVRSRVRGXDestroyCCBKM(RGX_CCB_CLEANUP_DATA *psCleanupData);

#endif /* __RGXCCB_H__ */
