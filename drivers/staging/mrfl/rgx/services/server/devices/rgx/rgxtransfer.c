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
PVRSRV_ERROR PVRSRVSubmitTQ2DKickKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ2DContextMemDesc,
									IMG_UINT32			ui32TQ2DcCCBWoffUpdate,
									IMG_BOOL			bPDumpContinuous)
{
	RGXFWIF_KCCB_CMD		s2DCCBCmd;
	PVRSRV_ERROR			eError;

	/*
	 * Construct the kernel 2D CCB command.
	 */
	s2DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s2DCCBCmd.uCmdData.sCmdKickData.psContext, psFWTQ2DContextMemDesc,
						  0,
						  RFW_FWADDR_NOREF_FLAG);
	s2DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = ui32TQ2DcCCBWoffUpdate;

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
	return PVRSRV_OK;
}


/*
 * PVRSRVSubmitTQ3DKickKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVSubmitTQ3DKickKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ3DContextMemDesc,
									IMG_UINT32			ui32TQ3DcCCBWoffUpdate,
									IMG_BOOL			bPDumpContinuous)
{
	RGXFWIF_KCCB_CMD		s3DCCBCmd;
	PVRSRV_ERROR			eError;

	/*
	 * Construct the kernel 3D CCB command.
	 */
	s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s3DCCBCmd.uCmdData.sCmdKickData.psContext,
						  psFWTQ3DContextMemDesc,
						  0,
						  RFW_FWADDR_NOREF_FLAG);
	s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = ui32TQ3DcCCBWoffUpdate;

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
	return PVRSRV_OK;
}

/**************************************************************************//**
 End of file (rgxtransfer.c)
******************************************************************************/
