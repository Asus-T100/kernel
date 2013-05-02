/*************************************************************************/ /*!
@File
@Title          RGX CCb routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX CCB routines
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

#include "pvr_debug.h"
#include "rgxdevice.h"
#include "pdump_km.h"
#include "allocmem.h"
#include "devicemem.h"
#include "rgxfwutils.h"
#include "osfunc.h"
#include "rgxccb.h"
#include "devicemem_pdump.h"


static
PVRSRV_ERROR RGXDestroyCCB(DEVMEM_MEMDESC 		*psClientCCBMemDesc,
						   DEVMEM_MEMDESC 		*psClientCCBCtlMemDesc,
						   DEVMEM_EXPORTCOOKIE 	*psClientCCBExportCookie,
						   DEVMEM_EXPORTCOOKIE 	*psClientCCBCtlExportCookie);

IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateCCBKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
								  IMG_UINT32			ui32AllocSize,
								  IMG_UINT32			ui32AllocAlignment,
								  RGX_CCB_CLEANUP_DATA	**ppsCleanupData,
								  DEVMEM_MEMDESC 		**ppsClientCCBMemDesc,
								  DEVMEM_MEMDESC 		**ppsClientCCBCtlMemDesc,
								  DEVMEM_EXPORTCOOKIE 	**psClientCCBExportCookie,
								  DEVMEM_EXPORTCOOKIE 	**psClientCCBCtlExportCookie)
{
	PVRSRV_ERROR			eError;
	DEVMEM_FLAGS_T			uiClientCCBMemAllocFlags, uiClientCCBCtlMemAllocFlags;
	RGX_CCB_CLEANUP_DATA	*psTmpCleanup;

	psTmpCleanup = OSAllocMem(sizeof(RGX_CCB_CLEANUP_DATA));
	if (psTmpCleanup == IMG_NULL)
		return PVRSRV_ERROR_OUT_OF_MEMORY;

	OSMemSet(psTmpCleanup, 0, sizeof(RGX_CCB_CLEANUP_DATA));

	uiClientCCBMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
								PVRSRV_MEMALLOCFLAG_GPU_READABLE |
								PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
								PVRSRV_MEMALLOCFLAG_CPU_READABLE |
								PVRSRV_MEMALLOCFLAG_UNCACHED |
								PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
								PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE;

	uiClientCCBCtlMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
								PVRSRV_MEMALLOCFLAG_GPU_READABLE |
								PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
								PVRSRV_MEMALLOCFLAG_CPU_READABLE |
								/* FIXME: Client CCB Ctl should be read-only for the CPU 
									(it is not because for now we initialize it from the host) */
								PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE | 
								PVRSRV_MEMALLOCFLAG_UNCACHED |
								PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
								PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE;

	PDUMPCOMMENT("Allocate RGXFW cCCB");
	eError = DevmemFwAllocateExportable(psDeviceNode,
										ui32AllocSize,
										uiClientCCBMemAllocFlags,
										ppsClientCCBMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateCCBKM: Failed to allocate RGX client CCB (%u)",
				eError));
		goto e0;
	}
	psTmpCleanup->psClientCCBMemDesc = *ppsClientCCBMemDesc;

	/*
	 * Export the CCB allocation so it can be mapped client-side.
	 */
	eError = DevmemExport(*ppsClientCCBMemDesc,
                          &psTmpCleanup->sClientCCBExportCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateCCBKM: Failed to export RGX client CCB (%u)",
				eError));
		goto e1;
	}
	*psClientCCBExportCookie = &psTmpCleanup->sClientCCBExportCookie;

	PDUMPCOMMENT("Allocate RGXFW cCCB control");
	eError = DevmemFwAllocateExportable(psDeviceNode,
										sizeof(RGXFWIF_CCCB_CTL),
										uiClientCCBCtlMemAllocFlags,
										ppsClientCCBCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateCCBKM: Failed to allocate RGX client CCB control (%u)",
				eError));
		goto e2;
	}
	psTmpCleanup->psClientCCBCtlMemDesc = *ppsClientCCBCtlMemDesc;

	/*
	 * Export the CCB control allocation so it can be mapped client-side.
	 */
	eError = DevmemExport(*ppsClientCCBCtlMemDesc,
                          &psTmpCleanup->sClientCCBCtlExportCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateCCBKM: Failed to export RGX client CCB control (%u)",
				eError));
		goto e3;
	}
	*psClientCCBCtlExportCookie = &psTmpCleanup->sClientCCBCtlExportCookie;

	*ppsCleanupData = psTmpCleanup;

	return PVRSRV_OK;

e3:
	DevmemFwFree(*ppsClientCCBCtlMemDesc);
e2:
	DevmemUnexport(*ppsClientCCBMemDesc,
					&psTmpCleanup->sClientCCBExportCookie);
e1:
	DevmemFwFree(*ppsClientCCBMemDesc);
e0:
	OSFreeMem(psTmpCleanup);
	return eError;
}

IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyCCBKM(RGX_CCB_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR eError;

	PDUMPCOMMENT("Free RGXFW cCCB");
	eError = RGXDestroyCCB(psCleanupData->psClientCCBMemDesc,
							psCleanupData->psClientCCBCtlMemDesc,
							&psCleanupData->sClientCCBExportCookie,
							&psCleanupData->sClientCCBCtlExportCookie);
	OSFreeMem(psCleanupData);
	return eError;
}

static
PVRSRV_ERROR RGXDestroyCCB(DEVMEM_MEMDESC 		*psClientCCBMemDesc,
						   DEVMEM_MEMDESC 		*psClientCCBCtlMemDesc,
						   DEVMEM_EXPORTCOOKIE 	*psClientCCBExportCookie,
						   DEVMEM_EXPORTCOOKIE 	*psClientCCBCtlExportCookie)
{
	if (psClientCCBCtlMemDesc != IMG_NULL)
	{
		DevmemUnexport(psClientCCBCtlMemDesc, psClientCCBCtlExportCookie);
		DevmemFwFree(psClientCCBCtlMemDesc);
	}

	if (psClientCCBMemDesc != IMG_NULL)
	{
		DevmemUnexport(psClientCCBMemDesc, psClientCCBExportCookie);
		DevmemFwFree(psClientCCBMemDesc);
	}
	
	return PVRSRV_OK;
}


static PVRSRV_ERROR _RGXAcquireCCB(volatile RGXFWIF_CCCB_CTL	*psCCBCtl,
								   IMG_UINT32		ui32Woff,
								   IMG_PVOID		pvCCB,
								   IMG_UINT32		ui32CmdSize,
								   IMG_PVOID			*ppvBufferSpace)
{
	/* For now the client does the wait for space so we don't need to */
#if 0
	IMG_UINT32 ui32FreeSpace;
	IMG_UINT32	uiStart;
	IMG_HANDLE hOSEvent = IMG_NULL;

	PVRSRV_ERROR eError;
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();


	uiStart = OSClockus();

	eError = OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWaitForValueKM: Failed to setup EventObject with error (%d)", eError));
		return eError;
	}

	for (;;)
	{
		ui32FreeSpace = GET_CCB_SPACE(ui32Woff,
									  psCCBCtl->ui32ReadOffset,
									  (1 << RGX_CCB_SIZE_LOG2));
		if (ui32FreeSpace >= ui32CmdSize)
		{
			*ppvBufferSpace = (pvCCB + ui32Woff);
			if (hOSEvent)
			{
				OSEventObjectClose(hOSEvent);
			}
			return PVRSRV_OK;
		}

		if ((OSClockus() - uiStart) >= MAX_HW_TIME_US)
		{
			/* FIXME: We should decide if Services or Client Drivers should retry to get CCB space.
			 * For the moment it is Services that does the retry
			 */
			PVR_DPF((PVR_DBG_WARNING, "_RGXAcquireCCB: Timeout, insufficient space in CCB: Required=0x%x, available=0x%x. (woff=%d, roff=%d) Keep trying...",
					ui32CmdSize,
					ui32FreeSpace,
					ui32Woff,
					psCCBCtl->ui32ReadOffset));
			uiStart = OSClockus();
/*			break;*/

		}
		
		if (hOSEvent)
		{
			if(OSEventObjectWait(hOSEvent) != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_MESSAGE, "RGXAcquireCCB: PVRSRVEventObjectWait failed"));
			}
		}
		else
		{
			OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
		}
	}
#else
	IMG_UINT32 ui32FreeSpace;

	ui32FreeSpace = GET_CCB_SPACE(ui32Woff,
								  psCCBCtl->ui32ReadOffset,
								  (1 << RGX_CCB_SIZE_LOG2));
	if (ui32FreeSpace >= ui32CmdSize)
	{
		*ppvBufferSpace = (pvCCB + ui32Woff);
		return PVRSRV_OK;
	}

	/* If we get here there is nothing we can do to recover */
	PVR_ASSERT(0);
	return PVRSRV_ERROR_OUT_OF_MEMORY;
#endif
}


static IMG_VOID _WaitForFWtoDrainCCB(volatile RGXFWIF_CCCB_CTL		*psCCBCtl, 
									 IMG_UINT32			ui32Woff)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hOSEvent;
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	/*
		Note:
		This means that we block all other calls across the bridge which
		can cause performance issues, but as this code only runs when
		PDump is active then this isn't an issue
	*/
	OSSetKeepPVRLock();
	eError = OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWaitForValueKM: Failed to setup EventObject with error (%d)", eError));
	}


	while (psCCBCtl->ui32ReadOffset != ui32Woff)
	{
		PVR_DPF((PVR_DBG_WARNING, "_WaitForCCBToDrain: Waiting on CCB(%p) for readoff (%d) == wroff (%d)", 
					psCCBCtl,
					psCCBCtl->ui32ReadOffset,
					ui32Woff));

		/* wait */
		if (hOSEvent)
		{
			if(OSEventObjectWait(hOSEvent) != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_MESSAGE, "RGXAcquireCCB: PVRSRVEventObjectWait failed"));
			}
		}
		else
		{
			OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
		}

	}

	if (hOSEvent)
	{
		OSEventObjectClose(hOSEvent);
	}
}

/******************************************************************************
 FUNCTION	: RGXAcquireCCB

 PURPOSE	: Obtains access to write some commands to a CCB

 PARAMETERS	: psDevData			- device data
  			  psCCB				- the CCB
			  ui32CmdSize		- How much space is required
			  hOSEvent			- Event object handle
			  ppvBufferSpace	- Pointer to space in the buffer

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
IMG_INTERNAL PVRSRV_ERROR RGXAcquireCCB(volatile RGXFWIF_CCCB_CTL	*psCCBCtl,
										IMG_UINT32		*pui32Woff,
										DEVMEM_MEMDESC 		*pscCCBMemDesc,
										IMG_UINT32		ui32CmdSize,
										IMG_PVOID		*ppvBufferSpace,
										IMG_BOOL		bDumpedCCBCtlAlready,
										IMG_BOOL		bPDumpContinuous)
{
	PVRSRV_ERROR eError;
	IMG_UINT32	ui32PDumpFlags	= bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS : 0;
	IMG_BOOL	bInCaptureRange;
	IMG_BOOL	bPdumpEnabled;
	IMG_PVOID	pvCCB;
	
	PDumpIsCaptureFrameKM(&bInCaptureRange);
	bPdumpEnabled = (bInCaptureRange || bPDumpContinuous);
		
	/* need to dump initial state */
	if (bPdumpEnabled && !bDumpedCCBCtlAlready)
	{
		/* wait for the firmware to catch up (in nohw this is a noop) */
		_WaitForFWtoDrainCCB(psCCBCtl, *pui32Woff);

	}

	/* FIXME: Once the PR-3Ds are removed, the 2*ui32CmdSize has to change to ui32CmdSize */
	/* Check that the CCB can hold this command + padding (the CCBSize/2 is to allow
	   previous commands to be swallowed for the roff to advance */
	if ((2*ui32CmdSize + PADDING_COMMAND_SIZE + 1) > ((1 << RGX_CCB_SIZE_LOG2)/2))
	{
		PVR_DPF((PVR_DBG_ERROR, "Command size (%d bytes) too big for CCB (%d bytes)\n", ui32CmdSize, (1 << RGX_CCB_SIZE_LOG2)));
		return PVRSRV_ERROR_CMD_TOO_BIG;
	}
	
	eError = DevmemAcquireCpuVirtAddr(pscCCBMemDesc,
									  &pvCCB);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"CreateCCB: Failed to map client CCB (0x%x)", eError));
		return eError;
	}
	

	/*
		Check we don't overflow the end of the buffer and make sure we have
		enough for the padding command.
	*/
	if ((*pui32Woff + ui32CmdSize + PADDING_COMMAND_SIZE) > (1 << RGX_CCB_SIZE_LOG2))
	{
		RGXFWIF_CCB_CMD_HEADER *psHeader;
		IMG_VOID *pvHeader;
		PVRSRV_ERROR eError;
		IMG_UINT32 ui32Remain = (1 << RGX_CCB_SIZE_LOG2) - *pui32Woff;

		/* We're at the end of the buffer without enough contiguous space */
		eError = _RGXAcquireCCB(psCCBCtl, *pui32Woff, pvCCB, ui32Remain,
								&pvHeader);
		if (eError != PVRSRV_OK)
		{

			/* We should never get here */
			PVR_DPF((PVR_DBG_ERROR, "Failed to get space for padding command\n"));
			PVR_ASSERT(0);
			return PVRSRV_ERROR_CMD_TOO_BIG;
		}
		psHeader = pvHeader;
		psHeader->eCmdType = RGXFWIF_CCB_CMD_TYPE_PADDING;
		psHeader->ui32CmdSize = ui32Remain - sizeof(RGXFWIF_CCB_CMD_HEADER);

		//PDUMPCOMMENT("cCCB(%p): Padding cmd %d", psCCB, psHeader->ui32CmdSize);
		if (bPdumpEnabled)
		{
			DevmemPDumpLoadMem(pscCCBMemDesc,
						   *pui32Woff,
						   ui32Remain,
						   ui32PDumpFlags);
		}
				
		UPDATE_CCB_OFFSET(*pui32Woff, ui32Remain, (1 << RGX_CCB_SIZE_LOG2));
	}

	return _RGXAcquireCCB(psCCBCtl, *pui32Woff, pvCCB, ui32CmdSize,
						  ppvBufferSpace);
}

/******************************************************************************
 FUNCTION	: RGXReleaseCCB

 PURPOSE	: Release a CCB that we have been writing to.

 PARAMETERS	: psDevData			- device data
  			  psCCB				- the CCB

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
IMG_INTERNAL PVRSRV_ERROR RGXReleaseCCB(PVRSRV_DEVICE_NODE	*psDevNode,
										volatile RGXFWIF_CCCB_CTL	*psCCBCtl,
										DEVMEM_MEMDESC 		*pscCCBMemDesc,
										DEVMEM_MEMDESC 		*pscCCBCtlMemDesc,
										IMG_BOOL		*pbDumpedCCBCtlAlready,
										IMG_UINT32		*pui32Woff,
										IMG_UINT32		ui32CmdSize,
										IMG_BOOL		bPDumpContinuous,
										SYNC_CONNECTION_DATA 	*psSyncConnectionData)
{
	PVRSRV_ERROR				eError		= PVRSRV_OK;

	IMG_UINT32	ui32PDumpFlags	= bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS : 0;
	IMG_BOOL	bInCaptureRange;
	IMG_BOOL	bPdumpEnabled;
	
	PDumpIsCaptureFrameKM(&bInCaptureRange);
	bPdumpEnabled = (bInCaptureRange || bPDumpContinuous);
		
	/* Entering a pdumped perido: CCB ctl state at the start of the day */
	if (bPdumpEnabled)
	{
		/* FIXME: For this to work properly, the CCB flush should happen for all the CCBs */

		/* This function will only PDump the syncs is required */
		SyncConnectionPDumpSyncBlocks(psSyncConnectionData);

		/* check if we need to pdump ccb state */
		if (!(*pbDumpedCCBCtlAlready))
		{

			/* fw should have reach this point, therefore it's safe to modify this value */
			psCCBCtl->ui32WriteOffset = *pui32Woff;

			PDUMPCOMMENT("cCCB: Dump initial state woff: 0x%x roff: 0x%x",
								   psCCBCtl->ui32WriteOffset, psCCBCtl->ui32ReadOffset);

			DevmemPDumpLoadMem(pscCCBCtlMemDesc,
							   0,
							   sizeof(RGXFWIF_CCCB_CTL),
							   ui32PDumpFlags);
							   
			*pbDumpedCCBCtlAlready = IMG_TRUE;
		}
	}

	/* Exiting a pdumped period: need to sync to this point */
	if (!bPdumpEnabled)
	{
		/* SyncPrim needs to be re-pdump next time we enter in the pdump */
		SyncConnectionPDumpExit(psSyncConnectionData);

		if (*pbDumpedCCBCtlAlready)
		{
			/* CCB needs to be re-pdump next time we enter in the pdump */
			*pbDumpedCCBCtlAlready = IMG_FALSE;

			/* make sure previous cmds are drained in pdump in case we will 'jump' over some future cmds */
			PDUMPCOMMENT("cCCB: Draining rgxfw_roff (0x%x) == woff (0x%x)",
							psCCBCtl->ui32ReadOffset,
							*pui32Woff);

			eError = DevmemPDumpDevmemPol32(pscCCBMemDesc,
											offsetof(RGXFWIF_CCCB_CTL, ui32ReadOffset),
											*pui32Woff,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_WARNING, "RGXReleaseCCB: problem pdumping POL for cCCBCtl (%d)", eError));
				return eError;
			}
		}
	}

	/* Dump the CCB data */
	if (bPdumpEnabled)
	{
		DevmemPDumpLoadMem(pscCCBMemDesc,
						   *pui32Woff,
						   ui32CmdSize,
						   ui32PDumpFlags);
	}
	/*
	 * Update the CCB write offset.
	 */
	UPDATE_CCB_OFFSET(*pui32Woff,
					  ui32CmdSize,
					  (1 << RGX_CCB_SIZE_LOG2));

#if defined(NO_HARDWARE)
	/*
		The firmware is not running, it cannot update these; we do here instead.
	*/
	psCCBCtl->ui32ReadOffset = *pui32Woff;
	psCCBCtl->ui32DepOffset = *pui32Woff;
#endif

	DevmemReleaseCpuVirtAddr(pscCCBMemDesc);
	
	return eError;
}
/******************************************************************************
 End of file (rgxccb.c)
******************************************************************************/
