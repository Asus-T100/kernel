									    /*************************************************************************//*!
									       @File
									       @Title          RGX memory context management
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX memory context management
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pvr_debug.h"
#include "rgxmem.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "rgxdevice.h"
#include "rgx_fwif_km.h"
#include "rgxfwutils.h"
#include "pdump_km.h"
#include "pvrsrv.h"
#include "sync_internal.h"

/*
	FIXME:
	For now just get global state, but what we really want is to do
	this per memory context
*/
static IMG_UINT32 ui32CacheOpps = 0;
/* FIXME: End */

IMG_VOID RGXMMUCacheInvalidate(PVRSRV_DEVICE_NODE * psDeviceNode,
			       IMG_HANDLE hDeviceData,
			       MMU_LEVEL eMMULevel, IMG_BOOL bUnmap)
{
	PVR_UNREFERENCED_PARAMETER(bUnmap);

	switch (eMMULevel) {
	case MMU_LEVEL_3:
		ui32CacheOpps |= RGXFWIF_MMUCACHEDATA_FLAGS_PC;
		break;
	case MMU_LEVEL_2:
		ui32CacheOpps |= RGXFWIF_MMUCACHEDATA_FLAGS_PD;
		break;
	case MMU_LEVEL_1:
		ui32CacheOpps |= RGXFWIF_MMUCACHEDATA_FLAGS_PT;
		ui32CacheOpps |= RGXFWIF_MMUCACHEDATA_FLAGS_TLB;
		break;
	default:
		PVR_ASSERT(0);
		break;
	}
}

PVRSRV_ERROR RGXSLCCacheInvalidateRequest(PVRSRV_DEVICE_NODE * psDeviceNode,
					  PMR * psPmr)
{
	RGXFWIF_KCCB_CMD sFlushInvalCmd;
	IMG_UINT32 ulPMRFlags;
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_ASSERT(psDeviceNode);

	/* In DEINIT state, we stop scheduling SLC flush commands, because we don't know in what state the firmware is.
	 * Anyway, if we are in DEINIT state, we don't care anymore about FW memory consistency
	 */
	if (psDeviceNode->eDevState != PVRSVR_DEVICE_STATE_DEINIT) {

		/* get the PMR's caching flags */
		eError = PMR_Flags(psPmr, &ulPMRFlags);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_WARNING,
				 "RGXSLCCacheInvalidateRequest: Unable to get the caching attributes of PMR %p",
				 psPmr));
		}

		/* Schedule a SLC flush and invalidate if
		 * - the memory is not UNCACHED.
		 * - we can't get the caching attributes (by precaution).
		 */
		if (((ulPMRFlags & PVRSRV_MEMALLOCFLAG_GPU_CACHE_MODE_MASK) !=
		     PVRSRV_MEMALLOCFLAG_GPU_UNCACHED)
		    || (eError != PVRSRV_OK)) {
			/* Schedule the SLC flush command ... */
#if defined(PDUMP)
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS,
					      "Submit SLC flush and invalidate");
#endif
			sFlushInvalCmd.eCmdType =
			    RGXFWIF_KCCB_CMD_SLCFLUSHINVAL;
			sFlushInvalCmd.uCmdData.sSLCFlushInvalData.bInval =
			    IMG_TRUE;
			sFlushInvalCmd.uCmdData.sSLCFlushInvalData.eDM = RGXFWIF_DM_2D;	//Covers all of Sidekick
			sFlushInvalCmd.uCmdData.sSLCFlushInvalData.psContext.
			    ui32Addr = IMG_NULL;

			eError = _RGXScheduleCommand(psDeviceNode->pvDevice,
						     RGXFWIF_DM_GP,
						     &sFlushInvalCmd,
						     sizeof(sFlushInvalCmd),
						     IMG_TRUE);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXSLCCacheInvalidateRequest: Failed to schedule SLC flush command with error (%u)",
					 eError));
			} else {
				/* Wait for the SLC flush to complete */
				eError =
				    RGXWaitForFWOp(psDeviceNode->pvDevice,
						   RGXFWIF_DM_GP, IMG_TRUE);
				if (eError != PVRSRV_OK) {
					PVR_DPF((PVR_DBG_ERROR,
						 "RGXSLCCacheInvalidateRequest: SLC flush and invalidate aborted with error (%u)",
						 eError));
				}
			}
		}
	}

	return eError;
}

PVRSRV_ERROR RGXPreKickCacheCommand(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	RGXFWIF_KCCB_CMD sFlushCmd;
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (ui32CacheOpps) {
		/* Schedule MMU cache command */
#if defined(PDUMP)
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS,
				      "Submit MMU flush and invalidate (flags = 0x%08x)",
				      ui32CacheOpps);
#endif

		sFlushCmd.eCmdType = RGXFWIF_KCCB_CMD_MMUCACHE;
#if 0				//defined(FIXME)
		/* Set which memory context this command is for */
		sFlushCmd.uCmdData.sMMUCacheData.psMemoryContext = ? ? ?
#endif
		    sFlushCmd.uCmdData.sMMUCacheData.ui32Flags = ui32CacheOpps;
		ui32CacheOpps = 0;

		eError = _RGXScheduleCommand(psDevInfo,
					     RGXFWIF_DM_GP,
					     &sFlushCmd,
					     sizeof(RGXFWIF_KCCB_CMD),
					     IMG_TRUE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXPreKickCacheCommand: Failed to schedule MMU cache command with error (%u)",
				 eError));
		} else {
			/* Wait for the MMU cache to complete */
			eError =
			    RGXWaitForFWOp(psDevInfo, RGXFWIF_DM_GP, IMG_TRUE);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXPreKickCacheCommand: MMU cache command aborted with error (%u)",
					 eError));
			}
		}
	}

	return eError;
}

IMG_VOID RGXUnregisterMemoryContext(IMG_HANDLE hPrivData)
{
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hPrivData;

	/*
	 * Release the page catalogue address acquired in RGXRegisterMemoryContext().
	 */
	MMU_ReleaseBaseAddr(IMG_NULL /* FIXME */ );

	/*
	 * Free the firmware memory context.
	 */
	DevmemFwFree(psFWMemContextMemDesc);
}

/*
 * RGXRegisterMemoryContext
 */
PVRSRV_ERROR RGXRegisterMemoryContext(PVRSRV_DEVICE_NODE * psDeviceNode,
				      MMU_CONTEXT * psMMUContext,
				      IMG_HANDLE * hPrivData)
{
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	DEVMEM_FLAGS_T uiFWMemContextMemAllocFlags;
	RGXFWIF_FWMEMCONTEXT *psFWMemContext;
	DEVMEM_MEMDESC *psFWMemContextMemDesc;

	if (psDevInfo->psKernelMMUCtx == IMG_NULL) {
		/*
		 * This must be the creation of the Kernel memory context. Take a copy
		 * of the MMU context for use when programming the BIF.
		 */
		psDevInfo->psKernelMMUCtx = psMMUContext;
	} else {
		/*
		 * This FW MemContext is only mapped into kernel for initialisation purposes.
		 * Otherwise this allocation is only used by the FW.
		 * Therefore the GPU cache doesn't need coherency,
		 * and write-combine is suffice on the CPU side (WC buffer will be flushed at any kick)
		 */
		uiFWMemContextMemAllocFlags =
		    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
		    PVRSRV_MEMALLOCFLAG_GPU_READABLE |
		    PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
		    PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
		    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
		    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
		    PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
		    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE;

		/*
		   Allocate device memory for the firmware memory context for the new
		   application.
		 */
		PDUMPCOMMENT("Allocate RGX firmware memory context");
		/* FIXME: why cache-consistent? */
		eError = DevmemFwAllocate(psDevInfo,
					  sizeof(*psFWMemContext),
					  uiFWMemContextMemAllocFlags,
					  &psFWMemContextMemDesc);

		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXRegisterMemoryContext: Failed to allocate firmware memory context (%u)",
				 eError));
			goto RGXRegisterMemoryContext_error;
		}

		/*
		   Temporarily map the firmware memory context to the kernel.
		 */
		eError = DevmemAcquireCpuVirtAddr(psFWMemContextMemDesc,
						  (IMG_VOID **) &
						  psFWMemContext);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXRegisterMemoryContext: Failed to map firmware memory context (%u)",
				 eError));
			goto RGXRegisterMemoryContext_error;
		}

		/*
		 * Write the new memory context's page catalogue into the firmware memory
		 * context for the client.
		 */
		eError =
		    MMU_AcquireBaseAddr(psMMUContext,
					&psFWMemContext->sPCDevPAddr);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXRegisterMemoryContext: Failed to acquire Page Catalogue address (%u)",
				 eError));
			goto RGXRegisterMemoryContext_error;
		}

		/*
		 * Set default values for the rest of the structure.
		 */
		psFWMemContext->uiPageCatBaseRegID = -1;
		psFWMemContext->bEnableTilingRegs = IMG_TRUE;
		psFWMemContext->uiBreakpointAddr = 0;
		psFWMemContext->uiBPHandlerAddr = 0;
		psFWMemContext->uiBreakpointCtl = 0;

#if defined(PDUMP)
		{
			IMG_CHAR
			    aszName[PMR_MAX_MEMSPNAME_SYMB_ADDR_LENGTH_DEFAULT];
			IMG_DEVMEM_OFFSET_T uiOffset = 0;

			/*
			 * Dump the Mem context allocation
			 */
			DevmemPDumpLoadMem(psFWMemContextMemDesc, 0,
					   sizeof(*psFWMemContext),
					   PDUMP_FLAGS_CONTINUOUS);

			/*
			 * Obtain a symbolic addr of the mem context structure
			 */
			eError =
			    DevmemPDumpPageCatBaseToSAddr(psFWMemContextMemDesc,
							  &uiOffset, aszName,
							  PMR_MAX_MEMSPNAME_SYMB_ADDR_LENGTH_DEFAULT);

			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXRegisterMemoryContext: Failed to generate a Dump Page Catalogue address (%u)",
					 eError));
				goto RGXRegisterMemoryContext_error;
			}

			/*
			 * Dump the Page Cat tag in the mem context (symbolic addresss)
			 */
			eError =
			    MMU_PDumpWritePageCatBase(psMMUContext, aszName,
						      uiOffset);

			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXRegisterMemoryContext: Failed to acquire Page Catalogue address (%u)",
					 eError));
				goto RGXRegisterMemoryContext_error;
			}
		}
#endif
		/*
		 * Release kernel address acquired above.
		 */
		DevmemReleaseCpuVirtAddr(psFWMemContextMemDesc);

		MMU_SetDeviceData(psMMUContext, psFWMemContextMemDesc);
		*hPrivData = psFWMemContextMemDesc;
	}

	return PVRSRV_OK;

 RGXRegisterMemoryContext_error:
	return eError;
}

/******************************************************************************
 End of file (rgxmem.c)
******************************************************************************/
