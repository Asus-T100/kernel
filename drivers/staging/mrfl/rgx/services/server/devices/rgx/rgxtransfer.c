/*************************************************************************/ /*!
@File
@Title          Device specific transfer queue routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device specific functions
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

#include "pdump_km.h"
#include "rgxdevice.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgxtransfer.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "pvr_debug.h"
#include "pvrsrv.h"
#include "rgx_fwif_resetframework.h"
#include "rgxccb.h"

#include "sync_server.h"
#include "sync_internal.h"

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
#include "pvr_sync.h"
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateTQ2DContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
										  DEVMEM_MEMDESC 			*psTQ2DCCBMemDesc,
										  DEVMEM_MEMDESC 			*psTQ2DCCBCtlMemDesc,
										  RGX_TQ2D_CLEANUP_DATA		**ppsCleanupData,
										  DEVMEM_MEMDESC 			**ppsFWTQ2DContextMemDesc,
										  IMG_UINT32				ui32Priority,
										  IMG_UINT32				ui32FrameworkRegisterSize,
										  IMG_PBYTE					pbyFrameworkRegisters,
										  IMG_HANDLE				hMemCtxPrivData)
{
	PVRSRV_ERROR			eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO 		*psDevInfo = psDeviceNode->pvDevice;	
	RGXFWIF_FWCOMMONCONTEXT	*psFWTQ2DContext;
	RGX_TQ2D_CLEANUP_DATA	*psTmpCleanup;
	DEVMEM_MEMDESC 			*psFWFrameworkMemDesc;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/* Allocate cleanup sync */
	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psTmpCleanup->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateComputeContextKM: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto fail_syncalloc;
	}

	/*
		Allocate device memory for the firmware TQ 2D context.
	*/
	PDUMPCOMMENT("Allocate RGX firmware TQ 2D context");
	
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psFWTQ2DContext),
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWTQ2DContextMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ2DContextKM: Failed to allocate firmware TQ 2D context (%u)",
				eError));
		goto fail_contextalloc;
	}
	psTmpCleanup->psFWTQ2DContextMemDesc = *ppsFWTQ2DContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;
	psTmpCleanup->bDumpedCCBCtlAlready = IMG_FALSE;

	/*
		Temporarily map the firmware TQ 2D context to the kernel.
	*/
	eError = DevmemAcquireCpuVirtAddr(*ppsFWTQ2DContextMemDesc,
                                      (IMG_VOID **)&psFWTQ2DContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ2DContextKM: Failed to map firmware TQ 2D context (%u)",
				eError));
		goto fail_cpuvirtacquire;
	}

	/* 
	 * Create the FW framework buffer
	 */
	eError = PVRSRVRGXFrameworkCreateKM(psDeviceNode, & psFWFrameworkMemDesc, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ2DContextKM: Failed to allocate firmware GPU framework state (%u)",
				eError));
		goto fail_frameworkcreate;
	}
	
	psTmpCleanup->psFWFrameworkMemDesc = psFWFrameworkMemDesc;

	/* Copy the Framework client data into the framework buffer */
	eError = PVRSRVRGXFrameworkCopyRegisters(psFWFrameworkMemDesc, pbyFrameworkRegisters, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ2DContextKM: Failed to populate the framework buffer (%u)",
				eError));
		goto fail_frameworkcopy;
	}

	eError = RGXInitFWCommonContext(psFWTQ2DContext,
									psTQ2DCCBMemDesc,
									psTQ2DCCBCtlMemDesc,
									hMemCtxPrivData,
									psFWFrameworkMemDesc,
									ui32Priority,
									IMG_NULL,
									& psTmpCleanup->sFWComContextCleanup);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ2DContextKM: Failed to init firmware common context (%u)",
				eError));
		goto fail_contextinit;
	}

	/*
	 * Dump the TQ2D and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWTQ2DContext");
	DevmemPDumpLoadMem(*ppsFWTQ2DContextMemDesc, 0, sizeof(*psFWTQ2DContext), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWTQ2DContextMemDesc);

	return PVRSRV_OK;
fail_contextinit:
fail_frameworkcopy:
	DevmemFwFree(psFWFrameworkMemDesc);
fail_frameworkcreate:
	DevmemReleaseCpuVirtAddr(*ppsFWTQ2DContextMemDesc);
fail_cpuvirtacquire:
	DevmemFwFree(*ppsFWTQ2DContextMemDesc);
fail_contextalloc:
	SyncPrimFree(psTmpCleanup->psCleanupSync);
fail_syncalloc:
	OSFreeMem(psTmpCleanup);
	return eError;
}


/*
 * PVRSRVRGXDestroyTQ2DContextKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyTQ2DContextKM(RGX_TQ2D_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR				eError = PVRSRV_OK;
	PRGXFWIF_FWCOMMONCONTEXT	psFWComContextFWAddr;

	RGXSetFirmwareAddress(&psFWComContextFWAddr,
							psCleanupData->psFWTQ2DContextMemDesc,
							0,
							RFW_FWADDR_NOREF_FLAG);

	eError = RGXFWRequestCommonContextCleanUp(psCleanupData->psDeviceNode,
											  psFWComContextFWAddr,
											  psCleanupData->psCleanupSync,
											  RGXFWIF_DM_2D);

	/*
		If we get retry error then we can't free this resource
		as it's still in use and we will be called again
	*/
	if (eError != PVRSRV_ERROR_RETRY)
	{
		eError = RGXDeinitFWCommonContext(&psCleanupData->sFWComContextCleanup);
	
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXDestroyTQ2DContextKM : failed to deinit fw common ctx. Error:%u", eError));
			goto e0;
		}
	
		/* Free the framework buffer */
		DevmemFwFree(psCleanupData->psFWFrameworkMemDesc);
		
		/*
		 * Free the firmware common context.
		 */
		DevmemFwFree(psCleanupData->psFWTQ2DContextMemDesc);

		/* Free the cleanup sync */
		SyncPrimFree(psCleanupData->psCleanupSync);

		OSFreeMem(psCleanupData);
	}

e0:
	return eError;
}

/*
 * PVRSRVRGXCreateTQ3DContextKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateTQ3DContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
										  DEVMEM_MEMDESC 			*psTQ3DCCBMemDesc,
										  DEVMEM_MEMDESC 			*psTQ3DCCBCtlMemDesc,
										  RGX_TQ3D_CLEANUP_DATA		**ppsCleanupData,
										  DEVMEM_MEMDESC 			**ppsFWTQ3DContextMemDesc,
										  DEVMEM_MEMDESC 			**ppsFWTQ3DContextStateMemDesc,
										  IMG_UINT32				ui32Priority,
										  IMG_DEV_VIRTADDR			sMCUFenceAddr,
										  IMG_UINT32				ui32FrameworkRegisterSize,
										  IMG_PBYTE					pbyFrameworkRegisters,
										  IMG_HANDLE				hMemCtxPrivData)
{
	PVRSRV_ERROR			eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO 		*psDevInfo = psDeviceNode->pvDevice;	
	RGXFWIF_FWCOMMONCONTEXT	*psFWTQ3DContext;
	RGX_TQ3D_CLEANUP_DATA	*psTmpCleanup;
	DEVMEM_MEMDESC 			*psFWFrameworkMemDesc;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}


	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/* Allocate cleanup sync */
	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psTmpCleanup->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateComputeContextKM: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto fail_syncalloc;
	}

	/*
		Allocate device memory for the firmware TQ 3D context.
	*/
	PDUMPCOMMENT("Allocate RGX firmware TQ 3D context");
	
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psFWTQ3DContext),
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWTQ3DContextMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to allocate firmware TQ 3D context (%u)",
				eError));
		goto fail_contextalloc;
	}
	psTmpCleanup->psFWTQ3DContextMemDesc = *ppsFWTQ3DContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;
	psTmpCleanup->bDumpedCCBCtlAlready = IMG_FALSE;

	/*
		Temporarily map the firmware TQ 3D context to the kernel.
	*/
	eError = DevmemAcquireCpuVirtAddr(*ppsFWTQ3DContextMemDesc,
                                      (IMG_VOID **)&psFWTQ3DContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to map firmware TQ 3D context (%u)",
				eError));
		goto fail_cpuvirtacquire;
	}

	/*
		Allocate device memory for the firmware GPU context suspend state.
		Note: the FW reads/writes the state to memory by accessing the GPU register interface.
	*/
	PDUMPCOMMENT("Allocate RGX firmware TQ/3D context suspend state");

	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_3DCTX_STATE),
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWTQ3DContextStateMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to allocate firmware GPU context suspend state (%u)",
				eError));
		goto fail_contextsuspendalloc;
	}
	psTmpCleanup->psFWTQ3DContextStateMemDesc = *ppsFWTQ3DContextStateMemDesc;

	/* 
	 * Create the FW framework buffer
	 */
	eError = PVRSRVRGXFrameworkCreateKM(psDeviceNode, & psFWFrameworkMemDesc, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to allocate firmware GPU framework state (%u)",
				eError));
		goto fail_frameworkcreate;
	}
	
	psTmpCleanup->psFWFrameworkMemDesc = psFWFrameworkMemDesc;

	/* Copy the Framework client data into the framework buffer */
	eError = PVRSRVRGXFrameworkCopyRegisters(psFWFrameworkMemDesc, pbyFrameworkRegisters, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to populate the framework buffer (%u)",
				eError));
		goto fail_frameworkcopy;
	}

	/* Init TQ/3D FW common context */
	eError = RGXInitFWCommonContext(psFWTQ3DContext,
									psTQ3DCCBMemDesc,
									psTQ3DCCBCtlMemDesc,
									hMemCtxPrivData,
									psFWFrameworkMemDesc,
									ui32Priority,
									&sMCUFenceAddr,
									&psTmpCleanup->sFWComContextCleanup);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateTQ3DContextKM: Failed to init firmware common context (%u)",
				eError));
		goto fail_contextinit;
	}

	/*
	 * Set the firmware GPU context state buffer.
	 * 
	 * The common context stores a dword pointer (FW) so we can cast the generic buffer to
	 * the correct 3D (3D/TQ = normal 3D) state structure type in the FW.
	 */
	RGXSetFirmwareAddress(&psFWTQ3DContext->psContextState,
								   *ppsFWTQ3DContextStateMemDesc,
								   0,
								   RFW_FWADDR_FLAG_NONE);

	/*
	 * Dump the TQ3D and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWTQ3DContext");
	DevmemPDumpLoadMem(*ppsFWTQ3DContextMemDesc, 0, sizeof(*psFWTQ3DContext), PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Dump the FW TQ/3D context suspend state buffer
	 */
	PDUMPCOMMENT("Dump FWTQ3DContextState");
	DevmemPDumpLoadMem(*ppsFWTQ3DContextStateMemDesc, 0, sizeof(RGXFWIF_3DCTX_STATE), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWTQ3DContextMemDesc);

	return PVRSRV_OK;

fail_contextinit:
fail_frameworkcopy:
	DevmemFwFree(psFWFrameworkMemDesc);
fail_frameworkcreate:
	DevmemFwFree(*ppsFWTQ3DContextStateMemDesc);
fail_contextsuspendalloc:
	DevmemReleaseCpuVirtAddr(*ppsFWTQ3DContextMemDesc);
fail_cpuvirtacquire:
	DevmemFwFree(*ppsFWTQ3DContextMemDesc);
fail_contextalloc:
	SyncPrimFree(psTmpCleanup->psCleanupSync);
fail_syncalloc:
	OSFreeMem(psTmpCleanup);
	return eError;
}


/*
 * PVRSRVRGXDestroyTQ3DContextKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyTQ3DContextKM(RGX_TQ3D_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR				eError = PVRSRV_OK;
	PRGXFWIF_FWCOMMONCONTEXT	psFWComContextFWAddr;

	RGXSetFirmwareAddress(&psFWComContextFWAddr,
							psCleanupData->psFWTQ3DContextMemDesc,
							0,
							RFW_FWADDR_NOREF_FLAG);

	eError = RGXFWRequestCommonContextCleanUp(psCleanupData->psDeviceNode,
											  psFWComContextFWAddr,
											  psCleanupData->psCleanupSync,
											  RGXFWIF_DM_3D);

	/*
		If we get retry error then we can't free this resource
		as it's still in use and we will be called again
	*/
	if (eError != PVRSRV_ERROR_RETRY)
	{
		eError = RGXDeinitFWCommonContext(&psCleanupData->sFWComContextCleanup);
	
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXDestroyTQ3DContextKM : failed to deinit fw common ctx. Error:%u", eError));
			goto e0;
		}
	
	#if defined(DEBUG)
		/* Log the number of TQ3D context stores which occurred */
		{
			RGXFWIF_3DCTX_STATE	*psFWState;

			eError = DevmemAcquireCpuVirtAddr(psCleanupData->psFWTQ3DContextStateMemDesc,
											  (IMG_VOID**)&psFWState);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
						eError));
			}
			else
			{
	
				PVR_DPF((PVR_DBG_WARNING,"Number of context stores on FW TQ3D context 0x%010x: %u",
						psFWComContextFWAddr.ui32Addr,
						psFWState->ui32NumStores));

				/* Release the CPU virt addr */
				DevmemReleaseCpuVirtAddr(psCleanupData->psFWTQ3DContextStateMemDesc);
			}
		}
	#endif
	
		/*
		 * Unmap the TA/3D context state buffer pointers
		 */
		RGXUnsetFirmwareAddress(psCleanupData->psFWTQ3DContextStateMemDesc);

		/*
		 * Free the firmware TQ/3D context state buffer
		 */
		DevmemFwFree(psCleanupData->psFWTQ3DContextStateMemDesc);

		/* Free the framework buffer */
		DevmemFwFree(psCleanupData->psFWFrameworkMemDesc);

		/*
		 * Free the firmware common context.
		 */
		DevmemFwFree(psCleanupData->psFWTQ3DContextMemDesc);

		/* Free the cleanup sync */
		SyncPrimFree(psCleanupData->psCleanupSync);

		OSFreeMem(psCleanupData);
	}
e0:
	return eError;
}

/*
 * PVRSRVSubmit2DKickKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVSubmitTQ2DKickKM(CONNECTION_DATA		*psConnection,
									PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ2DContextMemDesc,
									IMG_UINT32			*pui32cCCBWoffUpdate,
									DEVMEM_MEMDESC 		*pscCCBMemDesc,
									DEVMEM_MEMDESC 		*psCCBCtlMemDesc,
									IMG_UINT32			ui32ServerSyncPrims,
									PVRSRV_CLIENT_SYNC_PRIM_OP**	pasSyncOp,
									SERVER_SYNC_PRIMITIVE **pasServerSyncs,
									IMG_UINT32			ui32CmdSize,
									IMG_PBYTE			pui8Cmd,
									IMG_UINT32			ui32FenceEnd,
									IMG_UINT32			ui32UpdateEnd,
									IMG_UINT32          ui32NumFenceFds,
									IMG_INT32           *ai32FenceFds,
									IMG_BOOL			bPDumpContinuous,
									RGX_TQ2D_CLEANUP_DATA *psCleanupData)
{
	RGXFWIF_KCCB_CMD		s2DCCBCmd;
	PVRSRV_ERROR			eError;
	volatile RGXFWIF_CCCB_CTL	*psCCBCtl;
	IMG_UINT8					*pui8CmdPtr;
	IMG_UINT32 i = 0;
	RGXFWIF_UFO					*psUFOPtr;
	IMG_UINT8 *pui8FencePtr = pui8Cmd + ui32FenceEnd; 
	IMG_UINT8 *pui8UpdatePtr = pui8Cmd + ui32UpdateEnd;
	IMG_UINT32                  ui32FDFenceCmdSize = 0;
	IMG_UINT32                  ui32FDUpdateCmdSize = 0;
#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	IMG_UINT32 ui32NumFenceSyncs = 0;
	IMG_UINT32 *pui32FenceFWAddrs;
	IMG_UINT32 *pui32FenceValues;
	IMG_UINT32 ui32NumUpdateSyncs = 0;
	IMG_UINT32 *pui32UpdateFWAddrs;
	IMG_UINT32 *pui32UpdateValues;
	RGXFWIF_CCB_CMD_HEADER *psFDFenceHdr  = IMG_NULL;
	RGXFWIF_CCB_CMD_HEADER *psFDUpdateHdr = IMG_NULL;
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	for (i = 0; i < ui32ServerSyncPrims; i++)
	{
		PVRSRV_CLIENT_SYNC_PRIM_OP *psSyncOp = pasSyncOp[i];

			IMG_BOOL bUpdate;
			
			PVR_ASSERT(psSyncOp->ui32Flags != 0);
			if (psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				bUpdate = IMG_TRUE;
			}
			else
			{
				bUpdate = IMG_FALSE;
			}

			eError = PVRSRVServerSyncQueueHWOpKM(pasServerSyncs[i],
												  bUpdate,
												  &psSyncOp->ui32FenceValue,
												  &psSyncOp->ui32UpdateValue);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVServerSyncQueueHWOpKM: Failed (0x%x)", eError));
				return eError;
			}
			
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32FenceValue;
				PDUMPCOMMENT("TQ client server fence - 0x%x <- 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8FencePtr += sizeof(*psUFOPtr);
			}
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32UpdateValue;
				PDUMPCOMMENT("TQ client server update - 0x%x -> 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8UpdatePtr += sizeof(*psUFOPtr);
			}
		
	}
	
	eError = DevmemAcquireCpuVirtAddr(psCCBCtlMemDesc,
									  (IMG_VOID **)&psCCBCtl);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"CreateCCB: Failed to map client CCB control (0x%x)", eError));
		return eError;
	}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	eError = PVRFDSyncQueryFencesKM(ui32NumFenceFds,
									ai32FenceFds,
									IMG_TRUE,
									&ui32NumFenceSyncs,
									&pui32FenceFWAddrs,
									&pui32FenceValues,
									&ui32NumUpdateSyncs,
									&pui32UpdateFWAddrs,
									&pui32UpdateValues);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRFDSyncQueryFencesKM: Failed (0x%x)", eError));
		return eError;
	}			
	if (ui32NumFenceSyncs)
	{
		ui32FDFenceCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumFenceSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
	}
	if (ui32NumUpdateSyncs)
	{
		ui32FDUpdateCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumUpdateSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
	}

	ui32CmdSize += ui32FDFenceCmdSize + ui32FDUpdateCmdSize;
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	/*
	 * Acquire space in the CCB for the command.
	 */
	eError = RGXAcquireCCB(psCCBCtl,
						   pui32cCCBWoffUpdate,
						   pscCCBMemDesc,
						   ui32CmdSize,
						   (IMG_PVOID *)&pui8CmdPtr,
						   psCleanupData->bDumpedCCBCtlAlready,
						   bPDumpContinuous);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVSubmitTQ2DKickKM: Failed to acquire %d bytes in client CCB", ui32CmdSize));
		PVR_ASSERT(0);
		return eError;
	}
	
	OSMemCopy(&pui8CmdPtr[ui32FDFenceCmdSize], pui8Cmd, ui32CmdSize - ui32FDFenceCmdSize - ui32FDUpdateCmdSize);
	
#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	if (ui32FDFenceCmdSize)
	{
		/* Fill the fence header */
		psFDFenceHdr = (RGXFWIF_CCB_CMD_HEADER *) pui8CmdPtr;
		psFDFenceHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_FENCE;
		psFDFenceHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumFenceSyncs);;
		/* Fill in the actual fence commands */
		pui8FencePtr = (IMG_UINT8 *) &pui8CmdPtr[sizeof(RGXFWIF_CCB_CMD_HEADER)];
		for (i = 0; i < ui32NumFenceSyncs; i++)
		{
			psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
			psUFOPtr->puiAddrUFO.ui32Addr = pui32FenceFWAddrs[i];
			psUFOPtr->ui32Value = pui32FenceValues[i];
			PDUMPCOMMENT("TQ client server fence - 0x%x <- 0x%x (Android native fence)",
						 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
			pui8FencePtr += sizeof(*psUFOPtr);
		}
		OSFreeMem(pui32FenceFWAddrs);
		OSFreeMem(pui32FenceValues);
	}
	if (ui32FDUpdateCmdSize)
	{
		/* Fill the update header */
		psFDUpdateHdr = (RGXFWIF_CCB_CMD_HEADER *) &pui8CmdPtr[ui32CmdSize - ui32FDUpdateCmdSize];
		psFDUpdateHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_UPDATE;
		psFDUpdateHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumUpdateSyncs);
		/* Fill in the actual update commands */
		pui8UpdatePtr = (IMG_UINT8 *) &pui8CmdPtr[ui32CmdSize - ui32FDUpdateCmdSize + sizeof(RGXFWIF_CCB_CMD_HEADER)];
		for (i = 0; i < ui32NumUpdateSyncs; i++)
		{
			psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
			psUFOPtr->puiAddrUFO.ui32Addr = pui32UpdateFWAddrs[i];
			psUFOPtr->ui32Value = pui32UpdateValues[i];
			PDUMPCOMMENT("TQ client server update - 0x%x -> 0x%x (Android native fence)",
						 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
			pui8UpdatePtr += sizeof(*psUFOPtr);
		}
		OSFreeMem(pui32UpdateFWAddrs);
		OSFreeMem(pui32UpdateValues);
	}
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	eError = RGXReleaseCCB(psDeviceNode, 
						   psCCBCtl,
						   pscCCBMemDesc, 
						   psCCBCtlMemDesc,
						   &psCleanupData->bDumpedCCBCtlAlready,
						   pui32cCCBWoffUpdate,
						   ui32CmdSize, 
						   bPDumpContinuous,
						   psConnection->psSyncConnectionData);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVSubmitTQ2DKickKM: Failed to release space in TQ CCB"));
		return eError;
	}
	

	/*
	 * Construct the kernel 2D CCB command.
	 */
	s2DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s2DCCBCmd.uCmdData.sCmdKickData.psContext, psFWTQ2DContextMemDesc,
						  0,
						  RFW_FWADDR_NOREF_FLAG);
	s2DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = *pui32cCCBWoffUpdate;

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
								RGXFWIF_DM_2D,
								&s2DCCBCmd,
								sizeof(s2DCCBCmd),
								bPDumpContinuous);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVSubmitTQ2DKickKM : failed to schedule kernel 2D command. Error:%u", eError));
		return eError;
	}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) && defined(NO_HARDWARE)
	for (i = 0; i < ui32NumFenceFds; i++)
	{
		if (PVRFDSyncNoHwUpdateFenceKM(ai32FenceFds[i]) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed nohw update on fence fd=%d",
					 __func__, ai32FenceFds[i]));
		}
	}
#endif

	DevmemReleaseCpuVirtAddr(psCCBCtlMemDesc);
	return PVRSRV_OK;
}


/*
 * PVRSRVSubmitTQ3DKickKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVSubmitTQ3DKickKM(CONNECTION_DATA		*psConnection,
									PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ3DContextMemDesc,
									IMG_UINT32			*pui32cCCBWoffUpdate,
									DEVMEM_MEMDESC 		*pscCCBMemDesc,
									DEVMEM_MEMDESC 		*psCCBCtlMemDesc,
									IMG_UINT32			ui32ServerSyncPrims,
									PVRSRV_CLIENT_SYNC_PRIM_OP**	pasSyncOp,
									SERVER_SYNC_PRIMITIVE **pasServerSyncs,
									IMG_UINT32			ui32CmdSize,
									IMG_PBYTE			pui8Cmd,
									IMG_UINT32			ui32FenceEnd,
									IMG_UINT32			ui32UpdateEnd,
									IMG_UINT32          ui32NumFenceFds,
									IMG_INT32           *ai32FenceFds,
									IMG_BOOL			bPDumpContinuous,
									RGX_TQ3D_CLEANUP_DATA *psCleanupData)
{
	RGXFWIF_KCCB_CMD		s3DCCBCmd;
	PVRSRV_ERROR			eError;
	volatile RGXFWIF_CCCB_CTL	*psCCBCtl;
	IMG_UINT8					*pui8CmdPtr;
	IMG_UINT32 i = 0;
	RGXFWIF_UFO					*psUFOPtr;
	IMG_UINT8 *pui8FencePtr = pui8Cmd + ui32FenceEnd; 
	IMG_UINT8 *pui8UpdatePtr = pui8Cmd + ui32UpdateEnd;
	IMG_UINT32                  ui32FDFenceCmdSize = 0;
	IMG_UINT32                  ui32FDUpdateCmdSize = 0;
#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	IMG_UINT32 ui32NumFenceSyncs = 0;
	IMG_UINT32 *pui32FenceFWAddrs;
	IMG_UINT32 *pui32FenceValues;
	IMG_UINT32 ui32NumUpdateSyncs = 0;
	IMG_UINT32 *pui32UpdateFWAddrs;
	IMG_UINT32 *pui32UpdateValues;
	RGXFWIF_CCB_CMD_HEADER *psFDFenceHdr  = IMG_NULL;
	RGXFWIF_CCB_CMD_HEADER *psFDUpdateHdr = IMG_NULL;
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	for (i = 0; i < ui32ServerSyncPrims; i++)
	{
		PVRSRV_CLIENT_SYNC_PRIM_OP *psSyncOp = pasSyncOp[i];

			IMG_BOOL bUpdate;
			
			PVR_ASSERT(psSyncOp->ui32Flags != 0);
			if (psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				bUpdate = IMG_TRUE;
			}
			else
			{
				bUpdate = IMG_FALSE;
			}

			eError = PVRSRVServerSyncQueueHWOpKM(pasServerSyncs[i],
												  bUpdate,
												  &psSyncOp->ui32FenceValue,
												  &psSyncOp->ui32UpdateValue);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVServerSyncQueueHWOpKM: Failed (0x%x)", eError));
				return eError;
			}
			
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32FenceValue;
				PDUMPCOMMENT("TQ3D client server fence - 0x%x <- 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8FencePtr += sizeof(*psUFOPtr);
			}
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32UpdateValue;
				PDUMPCOMMENT("TQ3D client server update - 0x%x -> 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8UpdatePtr += sizeof(*psUFOPtr);
			}
	}
	
	eError = DevmemAcquireCpuVirtAddr(psCCBCtlMemDesc,
									  (IMG_VOID **)&psCCBCtl);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"CreateCCB: Failed to map client CCB control (0x%x)", eError));
		return eError;
	}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	eError = PVRFDSyncQueryFencesKM(ui32NumFenceFds,
									ai32FenceFds,
									IMG_TRUE,
									&ui32NumFenceSyncs,
									&pui32FenceFWAddrs,
									&pui32FenceValues,
									&ui32NumUpdateSyncs,
									&pui32UpdateFWAddrs,
									&pui32UpdateValues);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRFDSyncQueryFencesKM: Failed (0x%x)", eError));
		return eError;
	}			
	if (ui32NumFenceSyncs)
	{
		ui32FDFenceCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumFenceSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
	}
	if (ui32NumUpdateSyncs)
	{
		ui32FDUpdateCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumUpdateSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
	}

	ui32CmdSize += ui32FDFenceCmdSize + ui32FDUpdateCmdSize;

	if (ui32NumFenceSyncs || ui32NumUpdateSyncs)
	{
		PDUMPCOMMENT("(TQ) Android native fences in use: %u fence syncs, %u update syncs, %u total cmd size",
					 ui32NumFenceSyncs, ui32NumUpdateSyncs, ui32FDFenceCmdSize + ui32FDUpdateCmdSize);
	}
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	/*
	 * Acquire space in the TQ CCB for the command.
	 */
	eError = RGXAcquireCCB(psCCBCtl,
						   pui32cCCBWoffUpdate,
						   pscCCBMemDesc,
						   ui32CmdSize,
						   (IMG_PVOID *)&pui8CmdPtr,
						   psCleanupData->bDumpedCCBCtlAlready,
						   bPDumpContinuous);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVSubmitTQ2DKickKM: Failed to acquire %d bytes in client CCB", ui32CmdSize));
		PVR_ASSERT(0);
		return eError;
	}
	
	OSMemCopy(&pui8CmdPtr[ui32FDFenceCmdSize], pui8Cmd, ui32CmdSize - ui32FDFenceCmdSize - ui32FDUpdateCmdSize);
	
#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	if (ui32FDFenceCmdSize)
	{
		/* Fill the fence header */
		psFDFenceHdr = (RGXFWIF_CCB_CMD_HEADER *) pui8CmdPtr;
		psFDFenceHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_FENCE;
		psFDFenceHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumFenceSyncs);;
		/* Fill in the actual fence commands */
		pui8FencePtr = (IMG_UINT8 *) &pui8CmdPtr[sizeof(RGXFWIF_CCB_CMD_HEADER)];
		for (i = 0; i < ui32NumFenceSyncs; i++)
		{
			psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
			psUFOPtr->puiAddrUFO.ui32Addr = pui32FenceFWAddrs[i];
			psUFOPtr->ui32Value = pui32FenceValues[i];
			PDUMPCOMMENT("TQ3D client server fence - 0x%x <- 0x%x (Android native fence)",
						 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
			pui8FencePtr += sizeof(*psUFOPtr);
		}
		OSFreeMem(pui32FenceFWAddrs);
		OSFreeMem(pui32FenceValues);
	}
	if (ui32FDUpdateCmdSize)
	{
		/* Fill the update header */
		psFDUpdateHdr = (RGXFWIF_CCB_CMD_HEADER *) &pui8CmdPtr[ui32CmdSize - ui32FDUpdateCmdSize];
		psFDUpdateHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_UPDATE;
		psFDUpdateHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumUpdateSyncs);
		/* Fill in the actual update commands */
		pui8UpdatePtr = (IMG_UINT8 *) &pui8CmdPtr[ui32CmdSize - ui32FDUpdateCmdSize + sizeof(RGXFWIF_CCB_CMD_HEADER)];
		for (i = 0; i < ui32NumUpdateSyncs; i++)
		{
			psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
			psUFOPtr->puiAddrUFO.ui32Addr = pui32UpdateFWAddrs[i];
			psUFOPtr->ui32Value = pui32UpdateValues[i];
			PDUMPCOMMENT("TQ3D client server update - 0x%x -> 0x%x (Android native fence)",
						 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
			pui8UpdatePtr += sizeof(*psUFOPtr);
		}
		OSFreeMem(pui32UpdateFWAddrs);
		OSFreeMem(pui32UpdateValues);
	}
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	/*
	 * Release the TQ CCB for the command.
	 */
	PDUMPCOMMENT("TQ 3D command");

	eError = RGXReleaseCCB(psDeviceNode, 
						   psCCBCtl,
						   pscCCBMemDesc, 
						   psCCBCtlMemDesc,
						   &psCleanupData->bDumpedCCBCtlAlready,
						   pui32cCCBWoffUpdate,
						   ui32CmdSize, 
						   bPDumpContinuous,
						   psConnection->psSyncConnectionData);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXKickCDM: Failed to release space in TQ CCB"));
		return eError;
	}
	
	/*
	 * Construct the kernel TQ3D CCB command.
	 */
	s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s3DCCBCmd.uCmdData.sCmdKickData.psContext,
						  psFWTQ3DContextMemDesc,
						  0,
						  RFW_FWADDR_NOREF_FLAG);
	s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = *pui32cCCBWoffUpdate;

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
								RGXFWIF_DM_3D,
								&s3DCCBCmd, 
								sizeof(s3DCCBCmd),
								bPDumpContinuous);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVSubmitTQ3DKickKM : failed to schedule kernel TQ command. Error:%u", eError));
		return eError;
	}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) && defined(NO_HARDWARE)
	for (i = 0; i < ui32NumFenceFds; i++)
	{
		if (PVRFDSyncNoHwUpdateFenceKM(ai32FenceFds[i]) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed nohw update on fence fd=%d",
					 __func__, ai32FenceFds[i]));
		}
	}
#endif

	DevmemReleaseCpuVirtAddr(psCCBCtlMemDesc);
	return PVRSRV_OK;
}

/**************************************************************************//**
 End of file (rgxtransfer.c)
******************************************************************************/
