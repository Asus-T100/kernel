/*!****************************************************************************
@File           osfunc.h

@Title          OS functions header

@Author         Imagination Technologies

@Date           23rd May 2002

@Copyright      Copyright 2002-2008 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or otherwise, or disclosed
                to third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform       Generic

@Description    OS specific API definitions

@DoxygenVer

******************************************************************************/

#ifdef DEBUG_RELEASE_BUILD
#pragma optimize( "", off )
#define DEBUG		1
#endif

#ifndef __OSFUNC_H__
#define __OSFUNC_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "img_types.h"
#include "pvrsrv_device.h"
#include "device.h"

/******************************************************************************
 * Static defines
 *****************************************************************************/
#define KERNEL_ID			0xffffffffL
#define ISR_ID				0xfffffffdL

	IMG_UINT32 OSClockus(IMG_VOID);

	IMG_SIZE_T OSGetPageSize(IMG_VOID);
	IMG_SIZE_T OSGetPageShift(IMG_VOID);
	IMG_SIZE_T OSGetPageMask(IMG_VOID);

	PVRSRV_ERROR OSInstallDeviceLISR(PVRSRV_DEVICE_CONFIG * psDevConfig,
					 IMG_HANDLE * hLISRData,
					 PFN_LISR pfnLISR, IMG_VOID * hData);
	PVRSRV_ERROR OSUninstallDeviceLISR(IMG_HANDLE hLISRData);

	PVRSRV_ERROR OSInstallMISR(IMG_HANDLE * hMISRData,
				   PFN_MISR pfnMISR, IMG_VOID * hData);
	PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData);
	PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData);

	IMG_VOID OSMemCopy(IMG_VOID * pvDst, IMG_VOID * pvSrc,
			   IMG_SIZE_T ui32Size);
	IMG_VOID *OSMapPhysToLin(IMG_CPU_PHYADDR BasePAddr,
				 IMG_SIZE_T ui32Bytes, IMG_UINT32 ui32Flags);
	IMG_BOOL OSUnMapPhysToLin(IMG_VOID * pvLinAddr, IMG_SIZE_T ui32Bytes,
				  IMG_UINT32 ui32Flags);

	IMG_VOID OSCPUOperation(PVRSRV_CACHE_OP eCacheOp);

	IMG_PID OSGetCurrentProcessIDKM(IMG_VOID);
	IMG_UINTPTR_T OSGetCurrentThreadID(IMG_VOID);
	IMG_VOID OSMemSet(IMG_VOID * pvDest, IMG_UINT8 ui8Value,
			  IMG_SIZE_T ui32Size);

	PVRSRV_ERROR OSMMUPxAlloc(PVRSRV_DEVICE_NODE * psDevNode,
				  IMG_SIZE_T uiSize, Px_HANDLE * psMemHandle,
				  IMG_DEV_PHYADDR * psDevPAddr);

	IMG_VOID OSMMUPxFree(PVRSRV_DEVICE_NODE * psDevNode,
			     Px_HANDLE * psMemHandle);

	PVRSRV_ERROR OSMMUPxMap(PVRSRV_DEVICE_NODE * psDevNode,
				Px_HANDLE * psMemHandle, IMG_SIZE_T uiSize,
				IMG_DEV_PHYADDR * psDevPAddr,
				IMG_VOID ** pvPtr);

	IMG_VOID OSMMUPxUnmap(PVRSRV_DEVICE_NODE * psDevNode,
			      Px_HANDLE * psMemHandle, IMG_VOID * pvPtr);

	PVRSRV_ERROR OSInitEnvData(IMG_VOID);
	IMG_VOID OSDeInitEnvData(IMG_VOID);

	IMG_CHAR *OSStringCopy(IMG_CHAR * pszDest, const IMG_CHAR * pszSrc);
	IMG_INT32 OSSNPrintf(IMG_CHAR * pStr, IMG_SIZE_T ui32Size,
			     const IMG_CHAR * pszFormat,
			     ...) IMG_FORMAT_PRINTF(3, 4);
	IMG_SIZE_T OSStringLength(const IMG_CHAR * pStr);
	IMG_INT32 OSStringCompare(const IMG_CHAR * pStr1,
				  const IMG_CHAR * pStr2);

	PVRSRV_ERROR OSEventObjectCreate(const IMG_CHAR * pszName,
					 IMG_HANDLE * EventObject);
	PVRSRV_ERROR OSEventObjectDestroy(IMG_HANDLE hEventObject);
	PVRSRV_ERROR OSEventObjectSignal(IMG_HANDLE hOSEventKM);
	PVRSRV_ERROR OSEventObjectWait(IMG_HANDLE hOSEventKM);
	PVRSRV_ERROR OSEventObjectOpen(IMG_HANDLE hEventObject,
				       IMG_HANDLE * phOSEvent);
	PVRSRV_ERROR OSEventObjectClose(IMG_HANDLE hOSEventKM);

	PVRSRV_ERROR OSLockResource(PVRSRV_RESOURCE * psResource,
				    IMG_UINT32 ui32ID);
	PVRSRV_ERROR OSUnlockResource(PVRSRV_RESOURCE * psResource,
				      IMG_UINT32 ui32ID);
	IMG_BOOL OSIsResourceLocked(PVRSRV_RESOURCE * psResource,
				    IMG_UINT32 ui32ID);
	PVRSRV_ERROR OSCreateResource(PVRSRV_RESOURCE * psResource);
	PVRSRV_ERROR OSDestroyResource(PVRSRV_RESOURCE * psResource);
	IMG_VOID OSBreakResourceLock(PVRSRV_RESOURCE * psResource,
				     IMG_UINT32 ui32ID);

/*!
******************************************************************************

 @Function OSWaitus
 
 @Description 
    This function implements a busy wait of the specified microseconds
    This function does NOT release thread quanta
 
 @Input ui32Timeus - (us)

 @Return IMG_VOID

******************************************************************************/
	IMG_VOID OSWaitus(IMG_UINT32 ui32Timeus);

/*!
******************************************************************************

 @Function OSSleepms
 
 @Description 
    This function implements a sleep of the specified milliseconds
    This function may allow pre-emption if implemented
 
 @Input ui32Timems - (ms)

 @Return IMG_VOID

******************************************************************************/
	IMG_VOID OSSleepms(IMG_UINT32 ui32Timems);

	IMG_VOID OSReleaseThreadQuanta(IMG_VOID);

	IMG_UINT8 OSReadHWReg8(IMG_PVOID pvLinRegBaseAddr,
			       IMG_UINT32 ui32Offset);
	IMG_UINT16 OSReadHWReg16(IMG_PVOID pvLinRegBaseAddr,
				 IMG_UINT32 ui32Offset);
	IMG_UINT32 OSReadHWReg32(IMG_PVOID pvLinRegBaseAddr,
				 IMG_UINT32 ui32Offset);
	IMG_UINT64 OSReadHWReg64(IMG_PVOID pvLinRegBaseAddr,
				 IMG_UINT32 ui32Offset);
	IMG_UINT64 OSReadHWRegBank(IMG_PVOID pvLinRegBaseAddr,
				   IMG_UINT32 ui32Offset,
				   IMG_UINT8 * pui8DstBuf,
				   IMG_UINT64 ui64DstBufLen);

	IMG_VOID OSWriteHWReg8(IMG_PVOID pvLinRegBaseAddr,
			       IMG_UINT32 ui32Offset, IMG_UINT8 ui32Value);
	IMG_VOID OSWriteHWReg16(IMG_PVOID pvLinRegBaseAddr,
				IMG_UINT32 ui32Offset, IMG_UINT16 ui32Value);
	IMG_VOID OSWriteHWReg32(IMG_PVOID pvLinRegBaseAddr,
				IMG_UINT32 ui32Offset, IMG_UINT32 ui32Value);
	IMG_VOID OSWriteHWReg64(IMG_PVOID pvLinRegBaseAddr,
				IMG_UINT32 ui32Offset, IMG_UINT64 ui64Value);
	IMG_UINT64 OSWriteHWRegBank(IMG_PVOID pvLinRegBaseAddr,
				    IMG_UINT32 ui32Offset,
				    IMG_UINT8 * pui8SrcBuf,
				    IMG_UINT64 ui64rcBufLen);

	typedef IMG_VOID(*PFN_TIMER_FUNC) (IMG_VOID *);
	IMG_HANDLE OSAddTimer(PFN_TIMER_FUNC pfnTimerFunc, IMG_VOID * pvData,
			      IMG_UINT32 ui32MsTimeout);
	PVRSRV_ERROR OSRemoveTimer(IMG_HANDLE hTimer);
	PVRSRV_ERROR OSEnableTimer(IMG_HANDLE hTimer);
	PVRSRV_ERROR OSDisableTimer(IMG_HANDLE hTimer);

/******************************************************************************

 @Function		OSPanic

 @Description	Take action in response to an unrecoverable driver error

 @Input    IMG_VOID

 @Return   IMG_VOID

******************************************************************************/
	IMG_VOID OSPanic(IMG_VOID);

	IMG_BOOL OSProcHasPrivSrvInit(IMG_VOID);

	typedef enum _img_verify_test {
		PVR_VERIFY_WRITE = 0,
		PVR_VERIFY_READ
	} IMG_VERIFY_TEST;

	IMG_BOOL OSAccessOK(IMG_VERIFY_TEST eVerification, IMG_VOID * pvUserPtr,
			    IMG_SIZE_T ui32Bytes);

	PVRSRV_ERROR OSCopyToUser(IMG_PVOID pvProcess, IMG_VOID * pvDest,
				  IMG_VOID * pvSrc, IMG_SIZE_T ui32Bytes);
	PVRSRV_ERROR OSCopyFromUser(IMG_PVOID pvProcess, IMG_VOID * pvDest,
				    IMG_VOID * pvSrc, IMG_SIZE_T ui32Bytes);

	IMG_VOID OSWriteMemoryBarrier(IMG_VOID);
	IMG_VOID OSMemoryBarrier(IMG_VOID);

	struct _img_osspinlock;
	typedef struct _img_osspinlock *IMG_SPINLOCK;

	IMG_SPINLOCK OSSpinLockAlloc(IMG_VOID);
	IMG_VOID OSSpinLockFree(IMG_SPINLOCK psLock);
	IMG_VOID OSSpinLockClaim(IMG_SPINLOCK psLock);
	IMG_VOID OSSpinLockRelease(IMG_SPINLOCK psLock);

/* FIXME: need to re-think pvrlock for server syncs */
	IMG_VOID OSSetReleasePVRLock(IMG_VOID);
	IMG_VOID OSSetKeepPVRLock(IMG_VOID);
	IMG_BOOL OSGetReleasePVRLock(IMG_VOID);
/* end FIXME */

#if defined (__cplusplus)
}
#endif
#endif				/* __OSFUNC_H__ */
/******************************************************************************
 End of file (osfunc.h)
******************************************************************************/
