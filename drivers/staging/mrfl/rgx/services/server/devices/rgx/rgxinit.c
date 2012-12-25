/*************************************************************************/ /*!
@File
@Title          Device specific initialisation routines
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

#include <stddef.h>

#include "rgxheapconfig.h"
#include "rgxpower.h"

#include "rgxinit.h"

#include "pdump_km.h"
#include "handle.h"
#include "allocmem.h"
#include "devicemem_pdump.h"
#include "rgxmem.h"
#include "sync_internal.h"

#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgx_fwif_km.h"

#include "rgxmmuinit.h"
#include "devicemem_utils.h"
#include "devicemem_server.h"

#include "rgxdebug.h"

#include "rgx_options_km.h"
#include "pvrversion.h"

#include "rgx_compat_bvnc.h"

#include "rgx_heaps.h"

#include "rgxta3d.h"

static PVRSRV_ERROR RGXDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT32 ui32ClientBuildOptions);

#define RGX_MMU_LOG2_PAGE_SIZE_4KB   (12)
#define RGX_MMU_LOG2_PAGE_SIZE_16KB  (14)
#define RGX_MMU_LOG2_PAGE_SIZE_64KB  (16)
#define RGX_MMU_LOG2_PAGE_SIZE_256KB (18)
#define RGX_MMU_LOG2_PAGE_SIZE_1MB   (20)
#define RGX_MMU_LOG2_PAGE_SIZE_2MB   (21)

#define RGX_MMU_PAGE_SIZE_4KB   (   4 * 1024)
#define RGX_MMU_PAGE_SIZE_16KB  (  16 * 1024)
#define RGX_MMU_PAGE_SIZE_64KB  (  64 * 1024)
#define RGX_MMU_PAGE_SIZE_256KB ( 256 * 1024)
#define RGX_MMU_PAGE_SIZE_1MB   (1024 * 1024)
#define RGX_MMU_PAGE_SIZE_2MB   (2048 * 1024)
#define RGX_MMU_PAGE_SIZE_MIN RGX_MMU_PAGE_SIZE_4KB
#define RGX_MMU_PAGE_SIZE_MAX RGX_MMU_PAGE_SIZE_2MB

#define VAR(x) #x


typedef struct _DEVMEM_REF_LOOKUP_
{
	IMG_UINT32 ui32ZSBufferID;
	RGX_ZSBUFFER_DATA *psZSBuffer;
} DEVMEM_REF_LOOKUP;

typedef struct _DEVMEM_FREELIST_LOOKUP_
{
	IMG_UINT32 ui32FreeListID;
	RGX_FREELIST *psFreeList;
} DEVMEM_FREELIST_LOOKUP;

/* FIXME: This is a workaround due to having 2 inits but only 1 deinit */
static IMG_BOOL g_bDevInit2Done = IMG_FALSE;


static IMG_VOID RGX_DeInitHeaps(DEVICE_MEMORY_INFO *psDevMemoryInfo);



#if !defined(NO_HARDWARE)
/*
	RGX LISR Handler
*/
static IMG_BOOL RGX_LISRHandler (IMG_VOID *pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	PVRSRV_DEVICE_CONFIG *psDevConfig;
	PVRSRV_RGXDEV_INFO *psDevInfo;
	IMG_UINT32 ui32IRQStatus;
	IMG_BOOL bInterruptProcessed = IMG_FALSE;

	psDeviceNode = pvData;
	psDevConfig = psDeviceNode->psDevConfig;
	psDevInfo = psDeviceNode->pvDevice;

	ui32IRQStatus = OSReadHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVIRQSTATUS);

	if (ui32IRQStatus & RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_EN)
	{
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVIRQSTATUS, RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_CLRMSK);
		
#if defined(RGX_FEATURE_OCPBUS)
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_OCP_IRQSTATUS_2, RGX_CR_OCP_IRQSTATUS_2_RGX_IRQ_STATUS_EN);
#endif

		if (psDevConfig->pfnInterruptHandled)
		{
			psDevConfig->pfnInterruptHandled(psDevConfig);
		}

		bInterruptProcessed = IMG_TRUE;
		OSScheduleMISR(psDevInfo->pvMISRData);
	}
	return bInterruptProcessed;
}

static IMG_VOID RGXCheckFWActivePowerState(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_TRACEBUF *psFWTraceBuf = psDevInfo->psRGXFWIfTraceBuf;
	PVRSRV_ERROR eError = PVRSRV_OK;
	
	if (psFWTraceBuf->ePowState == RGXFWIF_APM_IDLE)
	{
		/* The FW is IDLE and therefore could be shut down */
		eError = RGXActivePowerRequest(psDeviceNode);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXCheckFWActivePowerState: Failed RGXActivePowerRequest call (device index: %d) with %s", 
						psDeviceNode->sDevId.ui32DeviceIndex,
						PVRSRVGetErrorStringKM(eError)));
		}
	}

}

static IMG_BOOL _FindZSBuffer(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	DEVMEM_REF_LOOKUP *psRefLookUp = (DEVMEM_REF_LOOKUP *)pvCallbackData;
	RGX_ZSBUFFER_DATA *psZSBuffer;

	psZSBuffer = IMG_CONTAINER_OF(psNode, RGX_ZSBUFFER_DATA, sNode);

	if (psZSBuffer->ui32DeferredAllocID == psRefLookUp->ui32ZSBufferID)
	{
		psRefLookUp->psZSBuffer = psZSBuffer;
		return IMG_FALSE;
	}
	else
	{
		return IMG_TRUE;
	}
}

static IMG_BOOL _FindFreeList(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	DEVMEM_FREELIST_LOOKUP *psRefLookUp = (DEVMEM_FREELIST_LOOKUP *)pvCallbackData;
	RGX_FREELIST *psFreeList;

	psFreeList = IMG_CONTAINER_OF(psNode, RGX_FREELIST, sNode);

	if (psFreeList->ui32FreelistID == psRefLookUp->ui32FreeListID)
	{
		psRefLookUp->psFreeList = psFreeList;
		return IMG_FALSE;
	}
	else
	{
		return IMG_TRUE;
	}
}

static IMG_VOID RGXCheckZSBuffer(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	volatile IMG_UINT32 *psAddress;
	IMG_UINT32	ui32Value;
	PVRSRV_ERROR eError;
	DEVMEM_REF_LOOKUP sLookUp;

	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(psDeviceNode->psZSBufferPopulateSyncPrim);
	PVR_ASSERT(psDeviceNode->psZSBufferUnPopulateSyncPrim);

	/* Get populate sync prim value  */
	psAddress = psDeviceNode->psZSBufferPopulateSyncPrim->pui32LinAddr;
	ui32Value = *(IMG_UINT32 *)psAddress;

	if (ui32Value)
	{
		/* scan all deferred allocations */
		sLookUp.ui32ZSBufferID = ui32Value;
		sLookUp.psZSBuffer = IMG_NULL;

		OSLockAcquire(psDeviceNode->hLockZSBuffer);
		dllist_foreach_node(&psDeviceNode->sDeferredAllocHead, _FindZSBuffer, (IMG_PVOID)&sLookUp);
		OSLockRelease(psDeviceNode->hLockZSBuffer);

		if (sLookUp.psZSBuffer)
		{
			/* Populate ZLS */
			eError = RGXBackingZSBuffer(sLookUp.psZSBuffer);
			if (eError != PVRSRV_OK)
			{
				/* Fixme: What to do if population fails? (mi) */
				PVR_DPF((PVR_DBG_ERROR,"Populating ZS-Buffer failed failed with error %u (ID = 0x%08x)", eError, ui32Value));
				PVR_ASSERT(IMG_FALSE);
			}

			/* Population of deferred memory completed. Clear down population request */
			*psAddress = 0;

			sLookUp.psZSBuffer->ui32NumReqByFW++;
		}
		else
		{
			PVR_DPF((PVR_DBG_ERROR,"ZS Buffer Lookup for ZS Buffer ID 0x%08x failed (Populate)", sLookUp.ui32ZSBufferID));
		}
	}


	/* Get unpopulate sync prim value */
	psAddress = psDeviceNode->psZSBufferUnPopulateSyncPrim->pui32LinAddr;
	ui32Value = *(IMG_UINT32 *)psAddress;

	if (ui32Value)
	{
		/* scan all deferred allocations */
		sLookUp.ui32ZSBufferID = ui32Value;
		sLookUp.psZSBuffer = IMG_NULL;

		OSLockAcquire(psDeviceNode->hLockZSBuffer);
		dllist_foreach_node(&psDeviceNode->sDeferredAllocHead, _FindZSBuffer, (IMG_PVOID)&sLookUp);
		OSLockRelease(psDeviceNode->hLockZSBuffer);

		if (sLookUp.psZSBuffer)
		{
			/* Populate ZLS */
			eError = RGXUnbackingZSBuffer(sLookUp.psZSBuffer);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"UnPopulating ZS-Buffer failed failed with error %u (ID = 0x%08x)", eError, ui32Value));
				PVR_ASSERT(IMG_FALSE);
			}

			/* Population of deferred memory completed. Clear down population request */
			*psAddress = 0;
		}
		else
		{
			PVR_DPF((PVR_DBG_ERROR,"ZS Buffer Lookup for ZS Buffer ID 0x%08x failed (UnPopulate)", sLookUp.ui32ZSBufferID));
		}
	}
}

static IMG_VOID RGXCheckFreeList(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	static IMG_UINT32 ui32LastValue = 0;
	volatile IMG_UINT32 *psAddress;
	IMG_UINT32	ui32Value;
	PVRSRV_ERROR eError;
	DEVMEM_FREELIST_LOOKUP sLookUp;
	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(psDeviceNode->psGrowSyncPrim);
	PVR_ASSERT(psDeviceNode->psShrinkSyncPrim);

	/* Get populate sync prim value  */
	psAddress = psDeviceNode->psGrowSyncPrim->pui32LinAddr;
	ui32Value = *(IMG_UINT32 *)psAddress;

	if ((ui32Value != 0) && (ui32Value != ui32LastValue))
	{
		/* find the freelist with the corresponding ID */
		sLookUp.ui32FreeListID = ui32Value & 0x7FFFFFFF;
		sLookUp.psFreeList = IMG_NULL;

		OSLockAcquire(psDeviceNode->hLockFreeList);
		dllist_foreach_node(&psDeviceNode->sFreeListHead, _FindFreeList, (IMG_PVOID)&sLookUp);
		OSLockRelease(psDeviceNode->hLockFreeList);

		if (sLookUp.psFreeList)
		{
			RGX_FREELIST *psFreeList = sLookUp.psFreeList;

			/* Try to grow the freelist */
			eError = RGXGrowFreeList(psFreeList,
									psFreeList->ui32GrowFLPages,
									&psFreeList->sMemoryBlockHead);
			if (eError == PVRSRV_OK)
			{
				/* Grow successful, return size of grow size */
				SyncPrimSet(psDeviceNode->psGrowSyncPrim, psFreeList->ui32GrowFLPages);
				ui32LastValue = psFreeList->ui32GrowFLPages;
				psFreeList->ui32NumGrowReqByFW++;
			}
			else
			{
				/* Grow failed */
				SyncPrimSet(psDeviceNode->psGrowSyncPrim, 0);
				ui32LastValue = 0;
				PVR_DPF((PVR_DBG_ERROR,"Grow for FreeList %p failed (error %u)",
										psFreeList,
										eError));
			}
		}
		else
		{
			/* Should never happen */
			PVR_DPF((PVR_DBG_ERROR,"FreeList Lookup for FreeList ID 0x%08x failed (Populate)", sLookUp.ui32FreeListID));
			PVR_ASSERT(IMG_FALSE);
		}
	}

	/* TODO: On-demand shrink (mi) */
}


/*
	RGX MISR Handler
*/
static IMG_VOID RGX_MISRHandler (IMG_VOID *pvData)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode = pvData;
	PVRSRV_RGXDEV_INFO	*psDevInfo	  = psDeviceNode->pvDevice;

	/* Inform other services devices that we have finished an operation */
	PVRSRVCheckStatus(psDeviceNode);

	/* Check that the server has HWPerf enabled i.e. the stream is created
	 * and extract the HWPerf data from the FW L1 buffer into the Server 
	 * L2 stream buffer.
	 */
	if (psDevInfo->hHWPerfStream != 0)
	{
		RGXHWPerfDataStore(psDevInfo);
	}

	/* React to ZLS population/unpopulation requests */
	RGXCheckZSBuffer(psDeviceNode);

	/* React to grow/shrink requests */
	RGXCheckFreeList(psDeviceNode);

	/* Check APM state */
	if (psDevInfo->pfnActivePowerCheck)
	{
		psDevInfo->pfnActivePowerCheck(psDeviceNode);
	}
}
#endif


/*
 * PVRSRVRGXInitDevPart2KM
 */ 
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXInitDevPart2KM (PVRSRV_DEVICE_NODE	*psDeviceNode,
									  RGX_INIT_COMMAND		*psInitScript,
									  RGX_INIT_COMMAND		*psDbgScript,
									  RGX_INIT_COMMAND		*psDeinitScript,
									  IMG_UINT32			ui32KernelCatBase, 
									  RGX_ACTIVEPM_CONF		eActivePMConf)
{
	PVRSRV_ERROR			eError;
	PVRSRV_RGXDEV_INFO		*psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_DEV_POWER_STATE	eDefaultPowerState;
	PVRSRV_DEVICE_CONFIG	*psDevConfig = psDeviceNode->psDevConfig;

	PDUMPCOMMENT("RGX Initialisation Part 2");

	psDevInfo->ui32KernelCatBase = ui32KernelCatBase;

	/*
	 * Map RGX Registers
	 */
#if !defined(NO_HARDWARE)
	psDevInfo->pvRegsBaseKM = OSMapPhysToLin(psDevConfig->sRegsCpuPBase,
										     psDevConfig->ui32RegsSize,
										     0);

	if (psDevInfo->pvRegsBaseKM == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXInitDevPart2KM: Failed to create RGX register mapping\n"));
		return PVRSRV_ERROR_BAD_MAPPING;
	}
#else
	psDevInfo->pvRegsBaseKM = IMG_NULL;
#endif /* !NO_HARDWARE */

	/* free the export cookies provided to srvinit */
	DevmemUnexport(psDevInfo->psRGXFWMemDesc, &psDevInfo->sRGXFWExportCookie);

	/*
	 * Copy scripts
	 */
	OSMemCopy(psDevInfo->sScripts.asInitCommands, psInitScript,
			  RGX_MAX_INIT_COMMANDS * sizeof(*psInitScript));

	OSMemCopy(psDevInfo->sScripts.asDbgCommands, psDbgScript,
			  RGX_MAX_INIT_COMMANDS * sizeof(*psDbgScript));

	OSMemCopy(psDevInfo->sScripts.asDeinitCommands, psDeinitScript,
			  RGX_MAX_DEINIT_COMMANDS * sizeof(*psDeinitScript));

#if defined(PDUMP)
	/* Run the deinit script to feed the last-frame deinit buffer */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_LASTFRAME, "RGX deinitialisation script");
	RGXRunScript(psDevInfo, psDevInfo->sScripts.asDeinitCommands, RGX_MAX_DEINIT_COMMANDS, PDUMP_FLAGS_LASTFRAME);
#endif


	psDevInfo->ui32RegSize = psDevConfig->ui32RegsSize;
	psDevInfo->sRegsPhysBase = psDevConfig->sRegsCpuPBase;


	eDefaultPowerState = PVRSRV_DEV_POWER_STATE_ON;

	/* set-up the Active Power Mgmt callback */
#if !defined(NO_HARDWARE)
	{
		RGX_DATA *psRGXData = (RGX_DATA*) psDeviceNode->psDevConfig->hDevData;
		IMG_BOOL bSysEnableAPM = psRGXData->psRGXTimingInfo->bEnableActivePM;
		IMG_BOOL bEnableAPM = ((eActivePMConf == RGX_ACTIVEPM_DEFAULT) && bSysEnableAPM) ||
							   (eActivePMConf == RGX_ACTIVEPM_FORCE_ON);

		if (bEnableAPM)
		{
			psDevInfo->pfnActivePowerCheck = RGXCheckFWActivePowerState;
			/* Prevent the device being woken up before there is something to do. */
			eDefaultPowerState = PVRSRV_DEV_POWER_STATE_OFF;
		}
	}
#endif

	/* Register the device with the power manager. */
	eError = PVRSRVRegisterPowerDevice (psDeviceNode->sDevId.ui32DeviceIndex,
										&RGXPrePowerState, &RGXPostPowerState,
										psDevConfig->pfnPrePowerState, psDevConfig->pfnPostPowerState,
										&RGXPreClockSpeedChange, &RGXPostClockSpeedChange,
										(IMG_HANDLE)psDeviceNode,
										PVRSRV_DEV_POWER_STATE_OFF,
										eDefaultPowerState);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXInitDevPart2KM: failed to register device with power manager"));
		return eError;
	}

#if !defined(NO_HARDWARE)
	/* Register the interrupt handlers */
	eError = OSInstallMISR(&psDevInfo->pvMISRData,
									RGX_MISRHandler, psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = OSInstallDeviceLISR(psDevConfig, &psDevInfo->pvLISRData,
								 RGX_LISRHandler, psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		(IMG_VOID) OSUninstallMISR(psDevInfo->pvLISRData);
		return eError;
	}

#endif
	g_bDevInit2Done = IMG_TRUE;

	return PVRSRV_OK;
}


/*
 * PVRSRVRGXInitFirmwareKM
 */ 
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXInitFirmwareKM(PVRSRV_DEVICE_NODE			*psDeviceNode, 
									 IMG_DEVMEM_SIZE_T			ui32FWMemAllocSize,
									 DEVMEM_EXPORTCOOKIE		**ppsFWMemAllocServerExportCookie,
									 IMG_DEV_VIRTADDR			*psFWMemDevVAddrBase,
									 IMG_UINT64					*pui64FWHeapBase,
									 RGXFWIF_DEV_VIRTADDR		*psRGXFwInit,
									 IMG_BOOL					bEnableSignatureChecks,
									 IMG_UINT32					ui32SignatureChecksBufSize,
									 IMG_UINT32					ui32RGXFWAlignChecksSize,
									 IMG_UINT32					*pui32RGXFWAlignChecks,
									 IMG_UINT32					ui32ConfigFlags,
									 IMG_UINT32					ui32LogType,
									 RGXFWIF_COMPCHECKS_BVNC	*psClientBVNC)
{
	PVRSRV_RGXDEV_INFO			*psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_ERROR				eError = PVRSRV_OK;
	RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(sBVNC);
	IMG_BOOL bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV;

	/* Check if BVNC numbers of client and driver are compatible */
	rgx_bvnc_packed(&sBVNC.ui32BNC, sBVNC.aszV, sBVNC.ui32VLenMax, RGX_BVNC_B, RGX_BVNC_V_ST, RGX_BVNC_N, RGX_BVNC_C);

	RGX_BVNC_EQUAL(sBVNC, *psClientBVNC, bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV);
	
	if (!bCompatibleAll)
	{
		if (!bCompatibleVersion)
		{
			PVR_LOG(("(FAIL) %s: Incompatible compatibility struct version of driver (%d) and client (%d).",
					__FUNCTION__, 
					sBVNC.ui32LayoutVersion, 
					psClientBVNC->ui32LayoutVersion));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			PVR_DBG_BREAK;
			goto failed_to_pass_compatibility_check;
		}

		if (!bCompatibleLenMax)
		{
			PVR_LOG(("(FAIL) %s: Incompatible V maxlen of driver (%d) and client (%d).",
					__FUNCTION__, 
					sBVNC.ui32VLenMax, 
					psClientBVNC->ui32VLenMax));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			PVR_DBG_BREAK;
			goto failed_to_pass_compatibility_check;
		}

		if (!bCompatibleBNC)
		{
			PVR_LOG(("(FAIL) %s: Incompatible driver BNC (%d._.%d.%d) / client BNC (%d._.%d.%d).",
					__FUNCTION__, 
					RGX_BVNC_PACKED_EXTR_B(sBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sBVNC), 
					RGX_BVNC_PACKED_EXTR_B(*psClientBVNC), 
					RGX_BVNC_PACKED_EXTR_N(*psClientBVNC), 
					RGX_BVNC_PACKED_EXTR_C(*psClientBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			PVR_DBG_BREAK;
			goto failed_to_pass_compatibility_check;
		}
		
		if (!bCompatibleV)
		{
			PVR_LOG(("(FAIL) %s: Incompatible driver BVNC (%d.%s.%d.%d) / client BVNC (%d.%s.%d.%d).",
					__FUNCTION__, 
					RGX_BVNC_PACKED_EXTR_B(sBVNC), 
					RGX_BVNC_PACKED_EXTR_V(sBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sBVNC), 
					RGX_BVNC_PACKED_EXTR_B(*psClientBVNC), 
					RGX_BVNC_PACKED_EXTR_V(*psClientBVNC), 
					RGX_BVNC_PACKED_EXTR_N(*psClientBVNC), 
					RGX_BVNC_PACKED_EXTR_C(*psClientBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			PVR_DBG_BREAK;
			goto failed_to_pass_compatibility_check;
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "%s: COMPAT_TEST: driver BVNC (%d.%s.%d.%d) and client BVNC (%d.%s.%d.%d) match. [ OK ]",
				__FUNCTION__, 
				RGX_BVNC_PACKED_EXTR_B(sBVNC), 
				RGX_BVNC_PACKED_EXTR_V(sBVNC), 
				RGX_BVNC_PACKED_EXTR_N(sBVNC), 
				RGX_BVNC_PACKED_EXTR_C(sBVNC), 
				RGX_BVNC_PACKED_EXTR_B(*psClientBVNC), 
				RGX_BVNC_PACKED_EXTR_V(*psClientBVNC), 
				RGX_BVNC_PACKED_EXTR_N(*psClientBVNC), 
				RGX_BVNC_PACKED_EXTR_C(*psClientBVNC)));
	}

	*pui64FWHeapBase = RGX_FIRMWARE_HEAP_BASE;

	/* Register callbacks for creation of device memory contexts */
	psDeviceNode->pfnRegisterMemoryContext = RGXRegisterMemoryContext;
	psDeviceNode->pfnUnregisterMemoryContext = RGXUnregisterMemoryContext;

	/* Create the memory context for the firmware. */
	eError = DevmemCreateContext(IMG_NULL, psDeviceNode,
								 DEVMEM_HEAPCFG_META,
								 &psDevInfo->psKernelDevmemCtx);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitRGXPart1: Failed DevmemCreateContext (%u)", eError));
		goto failed_to_create_ctx;
	}
	
	eError = DevmemFindHeapByName(psDevInfo->psKernelDevmemCtx,
								  "Firmware", /* FIXME: We need to create an IDENT macro for this string.
								                 Make sure the IDENT macro is not accessible to userland */
								  &psDevInfo->psFirmwareHeap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitRGXPart1: Failed DevmemFindHeapByName (%u)", eError));
		goto failed_to_find_heap;
	}

	eError = RGXSetupFirmware(psDeviceNode, 
							  ui32FWMemAllocSize,
							  ppsFWMemAllocServerExportCookie,
							  psFWMemDevVAddrBase,
							  bEnableSignatureChecks, 
							  ui32SignatureChecksBufSize,
							  ui32RGXFWAlignChecksSize,
							  pui32RGXFWAlignChecks,
							  ui32ConfigFlags,
							  ui32LogType,
							  psRGXFwInit);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXInitFirmwareKM: RGXSetupFirmware failed (%u)", eError));
		goto failed_init_firmware;
	}

	return eError;

failed_init_firmware:
failed_to_find_heap:
	DevmemDestroyContext(psDevInfo->psKernelDevmemCtx);
failed_to_create_ctx:

failed_to_pass_compatibility_check:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/* See device.h for function declaration */
static PVRSRV_ERROR RGXAllocUFOBlock(PVRSRV_DEVICE_NODE *psDeviceNode,
									 DEVMEM_MEMDESC **psMemDesc,
									 IMG_UINT32 *puiSyncPrimVAddr,
									 IMG_UINT32 *puiSyncPrimBlockSize)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;
	PVRSRV_ERROR eError;
	RGXFWIF_DEV_VIRTADDR pFirmwareAddr;
	IMG_DEVMEM_SIZE_T ui32UFOBlockSize = sizeof(IMG_UINT32);
	IMG_DEVMEM_ALIGN_T ui32UFOBlockAlign = sizeof(IMG_UINT32);

	psDevInfo = psDeviceNode->pvDevice;

	/* Size and align are 'expanded' because we request an Exportalign allocation */
	DevmemExportalignAdjustSizeAndAlign(psDevInfo->psFirmwareHeap,
										&ui32UFOBlockSize,
										&ui32UFOBlockAlign);

	eError = DevmemFwAllocateExportable(psDeviceNode,
										ui32UFOBlockSize,
										PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
										PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
										PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
										PVRSRV_MEMALLOCFLAG_CACHE_COHERENT | 
										PVRSRV_MEMALLOCFLAG_GPU_READABLE |
										PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
										PVRSRV_MEMALLOCFLAG_CPU_READABLE |
										PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE,
										psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto e0;
	}

	DevmemPDumpLoadMem(*psMemDesc,
					   0,
					   ui32UFOBlockSize,
					   PDUMP_FLAGS_CONTINUOUS);

	RGXSetFirmwareAddress(&pFirmwareAddr, *psMemDesc, 0, RFW_FWADDR_FLAG_NONE);
	*puiSyncPrimVAddr = pFirmwareAddr.ui32Addr;
	*puiSyncPrimBlockSize = ui32UFOBlockSize;

	return PVRSRV_OK;

	DevmemFwFree(*psMemDesc);
e0:
	return eError;
}

/* See device.h for function declaration */
static IMG_VOID RGXFreeUFOBlock(PVRSRV_DEVICE_NODE *psDeviceNode,
								DEVMEM_MEMDESC *psMemDesc)
{
	/*
		We know that only if the system has cache snooping then
		the UFO data might be cached
	*/
	if (PVRSRVSystemHasCacheSnooping())
	{
		RGXFWIF_KCCB_CMD sFlushInvalCmd;
		PVRSRV_ERROR eError;

		/* Schedule the SLC flush command ... */
#if defined(PDUMP)
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Submit SLC flush and invalidate");
#endif
		sFlushInvalCmd.eCmdType = RGXFWIF_KCCB_CMD_SLCFLUSHINVAL;
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.bInval = IMG_TRUE;
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.eDM = RGXFWIF_DM_2D; //Covers all of Sidekick
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.psContext.ui32Addr = 0;
		
		eError = RGXSendCommandWithPowLock(psDeviceNode->pvDevice,
											RGXFWIF_DM_GP,
											&sFlushInvalCmd,
											sizeof(sFlushInvalCmd),
											IMG_TRUE);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXFreeUFOBlock: Failed to schedule SLC flush command with error (%u)", eError));
		}
		else
		{
			/* Wait for the SLC flush to complete */
			eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"RGXFreeUFOBlock: SLC flush and invalidate aborted with error (%u)", eError));
			}
		}
	}

	RGXUnsetFirmwareAddress(psMemDesc);
	DevmemFwFree(psMemDesc);
}

/*
	DevDeInitRGX
*/
PVRSRV_ERROR DevDeInitRGX (PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_RGXDEV_INFO			*psDevInfo = (PVRSRV_RGXDEV_INFO*)psDeviceNode->pvDevice;
	PVRSRV_ERROR				eError;
	DEVICE_MEMORY_INFO		    *psDevMemoryInfo;

	if (!psDevInfo)
	{
		/* Can happen if DevInitRGX failed */
		PVR_DPF((PVR_DBG_ERROR,"DevDeInitRGX: Null DevInfo"));
		return PVRSRV_OK;
	}

#if 0 // FIXME
	if (psDevInfo->hTimer)
	{
		eError = OSRemoveTimer(psDevInfo->hTimer);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"DevDeInitRGX: Failed to remove timer"));
			return 	eError;
		}
		psDevInfo->hTimer = IMG_NULL;
	}
#endif /* FIXME */

	/* Cancel notifications to this device */
	PVRSRVUnregisterCmdCompleteNotify(psDeviceNode->hCmdCompNotify);
	psDeviceNode->hCmdCompNotify = IMG_NULL;

    psDevMemoryInfo = &psDeviceNode->sDevMemoryInfo;

	RGX_DeInitHeaps(psDevMemoryInfo);

	if (DevmemIsValidExportCookie(&psDevInfo->sRGXFWExportCookie))
	{
		/* if the export cookie is valid, the init sequence failed */
		PVR_DPF((PVR_DBG_ERROR,"DevDeInitRGX: FW Export cookie still valid (should have been unexported at init time)"));
		DevmemUnexport(psDevInfo->psRGXFWMemDesc, &psDevInfo->sRGXFWExportCookie);
	}

	/*
	   Free the firmware allocations.
	 */
	RGXFreeFirmware(psDevInfo);

	/*
	 * Clear the mem context create callbacks before destroying the RGX firmware
	 * context to avoid a spurious callback.
	 */
	psDeviceNode->pfnRegisterMemoryContext = IMG_NULL;
	psDeviceNode->pfnUnregisterMemoryContext = IMG_NULL;

	if (psDevInfo->psKernelDevmemCtx)
	{
		eError = DevmemDestroyContext(psDevInfo->psKernelDevmemCtx);
		/* oops - this should return void -- FIXME */
		PVR_ASSERT(eError == PVRSRV_OK);
	}

	if (g_bDevInit2Done)
	{
#if !defined(NO_HARDWARE)
		(IMG_VOID) OSUninstallDeviceLISR(psDevInfo->pvLISRData);
		(IMG_VOID) OSUninstallMISR(psDevInfo->pvMISRData);
#endif /* !NO_HARDWARE */

		/* unregister MMU related stuff */
		eError = RGXMMUInit_Unregister(psDeviceNode);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"DevDeInitRGX: Failed RGXMMUInit_Unregister (0x%x)", eError));
			return eError;
		}

		/* remove the device from the power manager */
		eError = PVRSRVRemovePowerDevice(psDeviceNode->sDevId.ui32DeviceIndex);
		if (eError != PVRSRV_OK)
		{
			return eError;
		}

		/* UnMap Regs */
		if (psDevInfo->pvRegsBaseKM != IMG_NULL)
		{
#if !defined(NO_HARDWARE)
			OSUnMapPhysToLin(psDevInfo->pvRegsBaseKM,
							 psDevInfo->ui32RegSize,
							 0);
#endif /* !NO_HARDWARE */
		}
	}

	/* DeAllocate devinfo */
	OSFreeMem(psDevInfo);

	psDeviceNode->pvDevice = IMG_NULL;

	return PVRSRV_OK;
}

/*!
******************************************************************************
 
 @Function	RGXDumpDebugInfoWrapper

 @Description Callback function for psDeviceNode->pfnDumpDebugInfo
  
******************************************************************************/
static IMG_VOID RGXDumpDebugInfoWrapper (PVRSRV_DEVICE_NODE *psDeviceNode)
{
	RGXDumpDebugInfo(psDeviceNode->pvDevice, IMG_TRUE);
}

static PVRSRV_ERROR RGXZSBufferSyncPrimUpdateWrapper(PVRSRV_DEVICE_NODE *psDeviceNode,
												PVRSRV_CLIENT_SYNC_PRIM *psZSBufferPopulateSyncPrim,
												PVRSRV_CLIENT_SYNC_PRIM *psZSBufferUnPopulateSyncPrim,
												PVRSRV_CLIENT_SYNC_PRIM *psGrowSyncPrim,
												PVRSRV_CLIENT_SYNC_PRIM *psShrinkSyncPrim)
{
	RGXFWIF_KCCB_CMD			sZSBufferUpdate;

	sZSBufferUpdate.eCmdType = RGXFWIF_KCCB_CMD_ZSBUFFER_SYNCPRIM_UDPATE;
	sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psZSBufferPopulateSyncPrimAddr = SyncPrimGetFirmwareAddr(psZSBufferPopulateSyncPrim);
	sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psZSBufferUnPopulateSyncPrimAddr = SyncPrimGetFirmwareAddr(psZSBufferUnPopulateSyncPrim);
	sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psGrowSyncPrimAddr = SyncPrimGetFirmwareAddr(psGrowSyncPrim);
	sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psShrinkSyncPrimAddr = SyncPrimGetFirmwareAddr(psShrinkSyncPrim);

	PDUMPCOMMENT("ZS Buffer Sync Prim update Request [FW Populate = 0x%08x,  UnPopulate = 0x%08x]",
					sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psZSBufferPopulateSyncPrimAddr,
					sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.psZSBufferUnPopulateSyncPrimAddr);

	return RGXScheduleCommandAndWait(psDeviceNode->pvDevice,
									RGXFWIF_DM_GP,
									&sZSBufferUpdate,
									sizeof(RGXFWIF_KCCB_CMD),
									&sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.uiSyncObjDevVAddr,
									&sZSBufferUpdate.uCmdData.sZSBufferSyncPrimUpdateData.uiUpdateVal,
									psDeviceNode->psSyncPrim,
									IMG_TRUE);
}


#if defined(PDUMP)
static
PVRSRV_ERROR RGXResetPDump(PVRSRV_DEVICE_NODE *psDeviceNode)
{
 	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *)(psDeviceNode->pvDevice);
	IMG_UINT32			ui32Idx;

	for (ui32Idx = 0; ui32Idx < RGXFWIF_DM_MAX; ui32Idx++)
	{
		psDevInfo->abDumpedKCCBCtlAlready[ui32Idx] = IMG_FALSE;
	}


	return PVRSRV_OK;
}
#endif /* PDUMP */


static PVRSRV_ERROR RGX_InitHeaps(DEVICE_MEMORY_INFO *psNewMemoryInfo)
{
    DEVMEM_HEAP_BLUEPRINT *psDeviceMemoryHeapCursor;

    /* actually - this ought not to be on the device node itself, I think.  Hmmm.  FIXME */
	psNewMemoryInfo->psDeviceMemoryHeap = OSAllocMem(sizeof(DEVMEM_HEAP_BLUEPRINT) * RGX_MAX_HEAP_ID);
    if(psNewMemoryInfo->psDeviceMemoryHeap == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXRegisterDevice : Failed to alloc memory for DEVMEM_HEAP_BLUEPRINT"));
		goto e0;
	}

	psDeviceMemoryHeapCursor = psNewMemoryInfo->psDeviceMemoryHeap;

	/************* general ***************/
    psDeviceMemoryHeapCursor->pszName = RGX_GENERAL_HEAP_IDENT;
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_GENERAL_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_GENERAL_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */

	/************* PDS code and data ***************/
    psDeviceMemoryHeapCursor->pszName = RGX_PDSCODEDATA_HEAP_IDENT;
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_PDSCODEDATA_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_PDSCODEDATA_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */
	
	/************* 3D Parameters ***************/
    psDeviceMemoryHeapCursor->pszName = RGX_3DPARAMETERS_HEAP_IDENT;
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_3DPARAMETERS_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_3DPARAMETERS_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */

	/************* USC code ***************/
    psDeviceMemoryHeapCursor->pszName = RGX_USCCODE_HEAP_IDENT;
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_USCCODE_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_USCCODE_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */

	/************* TQ 3D Parameters ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_TQ3DPARAMETERS_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_TQ3DPARAMETERS_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_TQ3DPARAMETERS_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */

	/************ Tiling Heaps ************/
	#define INIT_TILING_HEAP(N) \
	do { \
   		psDeviceMemoryHeapCursor->pszName = RGX_TILING_XSTRIDE ## N ## _HEAP_IDENT; \
   		psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_TILING_XSTRIDE ## N ## _HEAP_BASE; \
		psDeviceMemoryHeapCursor->uiHeapLength = RGX_TILING_XSTRIDE ## N ## _HEAP_SIZE; \
		psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB; \
		psDeviceMemoryHeapCursor++; \
	} while (0)

	INIT_TILING_HEAP(0);
	INIT_TILING_HEAP(1);
	INIT_TILING_HEAP(2);
	INIT_TILING_HEAP(3);
	INIT_TILING_HEAP(4);
	INIT_TILING_HEAP(5);
	INIT_TILING_HEAP(6);
	INIT_TILING_HEAP(7);

	#undef INIT_TILING_HEAP

	/************* HWBRN37200 ***************/
#if defined(FIX_HW_BRN_37200)
    psDeviceMemoryHeapCursor->pszName = "HWBRN37200";
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_HWBRN37200_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_HWBRN37200_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */
#endif

	/************* Firmware ***************/
    psDeviceMemoryHeapCursor->pszName = "Firmware";
    psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_FIRMWARE_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_FIRMWARE_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize = RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;/* advance to the next heap */

	/* set the heap count */
	psNewMemoryInfo->ui32HeapCount = (IMG_UINT32)(psDeviceMemoryHeapCursor - psNewMemoryInfo->psDeviceMemoryHeap);

	PVR_ASSERT(psNewMemoryInfo->ui32HeapCount <= RGX_MAX_HEAP_ID);

    /* the new way: we'll set up 2 heap configs: one will be for Meta
       only, and has only the firmware heap in it. 
       The remaining one shall be for clients only, and shall have all
       the other heaps in it */

    psNewMemoryInfo->uiNumHeapConfigs = 2;
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray = OSAllocMem(sizeof(DEVMEM_HEAP_CONFIG) * psNewMemoryInfo->uiNumHeapConfigs);
    if (psNewMemoryInfo->psDeviceMemoryHeapConfigArray == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXRegisterDevice : Failed to alloc memory for DEVMEM_HEAP_CONFIG"));
		goto e1;
	}
    
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].pszName = "Default Heap Configuration";
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].uiNumHeaps = psNewMemoryInfo->ui32HeapCount-1;
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].psHeapBlueprintArray = psNewMemoryInfo->psDeviceMemoryHeap;

    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].pszName = "Firmware Heap Configuration";
#if defined(FIX_HW_BRN_37200)
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].uiNumHeaps = 2;
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].psHeapBlueprintArray = psDeviceMemoryHeapCursor-2;
#else
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].uiNumHeaps = 1;
    psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].psHeapBlueprintArray = psDeviceMemoryHeapCursor-1;
#endif

	return PVRSRV_OK;
e1:
	OSFreeMem(psNewMemoryInfo->psDeviceMemoryHeap);
e0:
	return PVRSRV_ERROR_OUT_OF_MEMORY;
}

static IMG_VOID RGX_DeInitHeaps(DEVICE_MEMORY_INFO *psDevMemoryInfo)
{
	OSFreeMem(psDevMemoryInfo->psDeviceMemoryHeapConfigArray);
	OSFreeMem(psDevMemoryInfo->psDeviceMemoryHeap);
}


/*
	RGXRegisterDevice
*/
PVRSRV_ERROR RGXRegisterDevice (PVRSRV_DEVICE_NODE *psDeviceNode)
{
    PVRSRV_ERROR eError;
	DEVICE_MEMORY_INFO *psDevMemoryInfo;
	PVRSRV_RGXDEV_INFO	*psDevInfo;

	/* pdump info about the core */
	PDUMPCOMMENT("RGX Version Information: %s", RGX_BVNC);
	
	#if defined(RGX_FEATURE_SYSTEM_CACHE)
	PDUMPCOMMENT("RGX System Level Cache is present");
	#endif /* RGX_FEATURE_SYSTEM_CACHE */

	PDUMPCOMMENT("RGX Initialisation (Part 1)");

	/*********************
	 * Device node setup *
	 *********************/
	/* Setup static data and callbacks on the device agnostic device node */
	psDeviceNode->sDevId.eDeviceType		= DEV_DEVICE_TYPE;
	psDeviceNode->sDevId.eDeviceClass		= DEV_DEVICE_CLASS;
#if defined(PDUMP)
	psDeviceNode->sDevId.pszPDumpRegName	= RGX_PDUMPREG_NAME;
	/*
		FIXME: This should not be required as PMR's should give the memspace
		name. However, due to limitations within PDump we need a memspace name
		when dpumping with MMU context with virtual address in which case we
		don't have a PMR to get the name from.
		
		There is also the issue obtaining a namespace name for the catbase which
		is required when we PDump the write of the physical catbase into the FW
		structure
	*/
	psDeviceNode->sDevId.pszPDumpDevName	= PhysHeapPDumpMemspaceName(psDeviceNode->psPhysHeap);
	psDeviceNode->pfnPDumpInitDevice = &RGXResetPDump;
#endif /* PDUMP */


#if defined(SUPPORT_MEMORY_TILING)
	psDeviceNode->pfnAllocMemTilingRange = RGX_AllocMemTilingRange;
	psDeviceNode->pfnFreeMemTilingRange = RGX_FreeMemTilingRange;
#endif

	/* Configure MMU specific stuff */
	RGXMMUInit_Register(psDeviceNode);

	psDeviceNode->pfnMMUCacheInvalidate = RGXMMUCacheInvalidate;

	psDeviceNode->pfnSLCCacheInvalidateRequest = RGXSLCCacheInvalidateRequest;

	/* Register RGX to receive notifies when other devices complete some work */
	PVRSRVRegisterCmdCompleteNotify(&psDeviceNode->hCmdCompNotify, &RGXScheduleProcessQueuesKM, psDeviceNode);

	psDeviceNode->pfnInitDeviceCompatCheck	= &RGXDevInitCompatCheck;

	/* Register callbacks for creation of device memory contexts */
	psDeviceNode->pfnRegisterMemoryContext = RGXRegisterMemoryContext;
	psDeviceNode->pfnUnregisterMemoryContext = RGXUnregisterMemoryContext;

	/* Register callbacks for Unified Fence Objects */
	psDeviceNode->pfnAllocUFOBlock = RGXAllocUFOBlock;
	psDeviceNode->pfnFreeUFOBlock = RGXFreeUFOBlock;

	/* Register callback for dumping debug info */
	psDeviceNode->pfnDumpDebugInfo = RGXDumpDebugInfoWrapper;

	/* Register callback for updating ZLS Sync Prim */
	psDeviceNode->pfnZSBufferSyncPrimUpdate = RGXZSBufferSyncPrimUpdateWrapper;


	/*********************
	 * Device info setup *
	 *********************/
	/* Allocate device control block */
	psDevInfo = OSAllocMem(sizeof(*psDevInfo));
	if (psDevInfo == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitRGXPart1 : Failed to alloc memory for DevInfo"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}
	OSMemSet (psDevInfo, 0, sizeof(*psDevInfo));
	psDeviceNode->pvDevice = psDevInfo;

	/* Setup static data and callbacks on the device specific device info */
	psDevInfo->eDeviceType 		= DEV_DEVICE_TYPE;
	psDevInfo->eDeviceClass 	= DEV_DEVICE_CLASS;
	psDevInfo->psDeviceNode		= psDeviceNode;

	psDevMemoryInfo = &psDeviceNode->sDevMemoryInfo;
	psDevMemoryInfo->ui32AddressSpaceSizeLog2 = RGX_FEATURE_VIRTUAL_ADDRESS_SPACE_BITS;
	psDevInfo->pvDeviceMemoryHeap = psDevMemoryInfo->psDeviceMemoryHeap;

	/* flags, backing store details to be specified by system */
	psDevMemoryInfo->ui32Flags = 0;

	eError = RGX_InitHeaps(psDevMemoryInfo);
	if (eError != PVRSRV_OK)
	{
		goto e0;
	}

	return PVRSRV_OK;
e0:
    PVR_ASSERT(eError != PVRSRV_OK);
    return eError;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_KMBuildOptions_FWAgainstDriver

 @Description

 Validate the FW build options against KM driver build options (KM build options only)

 Following check is reduntant, because next check checks the same bits.
 Redundancy occurs because if client-server are build-compatible and client-firmware are 
 build-compatible then server-firmware are build-compatible as well.
 
 This check is left for clarity in error messages if any incompatibility occurs.

 @Input psRGXFWInit - FW init data

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck_KMBuildOptions_FWAgainstDriver(RGXFWIF_INIT *psRGXFWInit)
{
#if !defined(NO_HARDWARE)
	IMG_UINT32			ui32BuildOptions, ui32BuildOptionsFWKMPart, ui32BuildOptionsMismatch;

	if (psRGXFWInit == IMG_NULL)
		return PVRSRV_ERROR_INVALID_PARAMS;

	ui32BuildOptions = (RGX_BUILD_OPTIONS_KM);

	ui32BuildOptionsFWKMPart = psRGXFWInit->sRGXCompChecks.ui32BuildOptions & RGX_BUILD_OPTIONS_MASK_KM;
	
	if (ui32BuildOptions != ui32BuildOptionsFWKMPart)
	{
		ui32BuildOptionsMismatch = ui32BuildOptions ^ ui32BuildOptionsFWKMPart;
		if ( (ui32BuildOptions & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware and KM driver build options; "
				"extra options present in the KM driver: (0x%x). Please check rgx_options_km.h",
				ui32BuildOptions & ui32BuildOptionsMismatch ));
		}

		if ( (ui32BuildOptionsFWKMPart & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware-side and KM driver build options; "
				"extra options present in Firmware: (0x%x). Please check rgx_options_km.h",
				ui32BuildOptionsFWKMPart & ui32BuildOptionsMismatch ));
		}
		return PVRSRV_ERROR_BUILD_OPTIONS_MISMATCH;
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: Firmware and KM driver build options match. [ OK ]"));
	}
#endif

	return PVRSRV_OK;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_BuildOptions_FWAgainstClient

 @Description

 Validate the FW build options against client build options (KM and non-KM)

 @Input psDevInfo - device info
 @Input psRGXFWInit - FW init data
 @Input ui32ClientBuildOptions - client build options flags

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck_BuildOptions_FWAgainstClient(PVRSRV_RGXDEV_INFO 	*psDevInfo,
																			RGXFWIF_INIT *psRGXFWInit,
																			IMG_UINT32 ui32ClientBuildOptions)
{
#if !defined(NO_HARDWARE)
	IMG_UINT32			ui32BuildOptionsMismatch;
	IMG_UINT32			ui32BuildOptionsFW;
#endif
#if defined(PDUMP)
	PVRSRV_ERROR		eError;
#endif

#if defined(PDUMP)
	PDUMPCOMMENT("Compatibility check: client and FW build options");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, ui32BuildOptions),
												ui32ClientBuildOptions,
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		return eError;
	}
#endif

#if !defined(NO_HARDWARE)
	if (psRGXFWInit == IMG_NULL)
		return PVRSRV_ERROR_INVALID_PARAMS;
	
	ui32BuildOptionsFW = psRGXFWInit->sRGXCompChecks.ui32BuildOptions;
	
	if (ui32ClientBuildOptions != ui32BuildOptionsFW)
	{
		ui32BuildOptionsMismatch = ui32ClientBuildOptions ^ ui32BuildOptionsFW;
		if ( (ui32ClientBuildOptions & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware and client build options; "
				"extra options present in client: (0x%x). Please check rgx_options.h",
				ui32ClientBuildOptions & ui32BuildOptionsMismatch ));
		}

		if ( (ui32BuildOptionsFW & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware and client build options; "
				"extra options present in Firmware: (0x%x). Please check rgx_options.h",
				ui32BuildOptionsFW & ui32BuildOptionsMismatch ));
		}
		return PVRSRV_ERROR_BUILD_OPTIONS_MISMATCH;
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: Firmware and client build options match. [ OK ]"));
	}
#endif

	return PVRSRV_OK;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_DDKVersion_FWAgainstDriver

 @Description

 Validate FW DDK version against driver DDK version

 @Input psDevInfo - device info
 @Input psRGXFWInit - FW init data

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck_DDKVersion_FWAgainstDriver(PVRSRV_RGXDEV_INFO *psDevInfo,
																			RGXFWIF_INIT *psRGXFWInit)
{
#if defined(PDUMP)||(!defined(NO_HARDWARE))
	IMG_UINT32			ui32DDKVersion;
	PVRSRV_ERROR		eError;
	
	ui32DDKVersion = PVRVERSION_PACK(PVRVERSION_MAJ, PVRVERSION_MIN);
#endif

#if defined(PDUMP)
	PDUMPCOMMENT("Compatibility check: KM driver and FW DDK version");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, ui32DDKVersion),
												ui32DDKVersion,
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		return eError;
	}
#endif

#if !defined(NO_HARDWARE)
	if (psRGXFWInit == IMG_NULL)
		return PVRSRV_ERROR_INVALID_PARAMS;

	if (psRGXFWInit->sRGXCompChecks.ui32DDKVersion != ui32DDKVersion)
	{
		PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Incompatible driver DDK revision (%u.%u) / Firmware DDK revision (%u.%u).",
				PVRVERSION_MAJ, PVRVERSION_MIN, 
				PVRVERSION_UNPACK_MAJ(psRGXFWInit->sRGXCompChecks.ui32DDKVersion),
				PVRVERSION_UNPACK_MIN(psRGXFWInit->sRGXCompChecks.ui32DDKVersion)));
		eError = PVRSRV_ERROR_DDK_VERSION_MISMATCH;
		PVR_DBG_BREAK;
		return eError;
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: driver DDK revision (%u.%u) and Firmware DDK revision (%u.%u) match. [ OK ]",
				PVRVERSION_MAJ, PVRVERSION_MIN, 
				PVRVERSION_MAJ, PVRVERSION_MIN));
	}
#endif	

	return PVRSRV_OK;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_DDKBuild_FWAgainstDriver

 @Description

 Validate FW DDK build against driver DDK build

 @Input psDevInfo - device info
 @Input psRGXFWInit - FW init data

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck_DDKBuild_FWAgainstDriver(PVRSRV_RGXDEV_INFO *psDevInfo,
																			RGXFWIF_INIT *psRGXFWInit)
{
#if defined(PDUMP)||(!defined(NO_HARDWARE))
	IMG_UINT32			ui32DDKBuild;
	PVRSRV_ERROR		eError;
	
	ui32DDKBuild = PVRVERSION_BUILD;
#endif

#if defined(PDUMP)
	PDUMPCOMMENT("Compatibility check: KM driver and FW DDK build");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, ui32DDKBuild),
												ui32DDKBuild,
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		return eError;
	}
#endif

#if !defined(NO_HARDWARE)
	if (psRGXFWInit == IMG_NULL)
		return PVRSRV_ERROR_INVALID_PARAMS;

	if (psRGXFWInit->sRGXCompChecks.ui32DDKBuild != ui32DDKBuild)
	{
		PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Incompatible driver DDK build (%d) / Firmware DDK build (%d).",
				ui32DDKBuild, psRGXFWInit->sRGXCompChecks.ui32DDKBuild));
		eError = PVRSRV_ERROR_DDK_BUILD_MISMATCH;
		PVR_DBG_BREAK;
		return eError;
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: driver DDK build (%d) and Firmware DDK build (%d) match. [ OK ]",
				ui32DDKBuild, psRGXFWInit->sRGXCompChecks.ui32DDKBuild));
	}
#endif
	return PVRSRV_OK;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_BVNC_FWAgainstDriver

 @Description

 Validate FW BVNC against driver BVNC

 @Input psDevInfo - device info
 @Input psRGXFWInit - FW init data

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck_BVNC_FWAgainstDriver(PVRSRV_RGXDEV_INFO *psDevInfo,
																			RGXFWIF_INIT *psRGXFWInit)
{
#if defined(PDUMP)
	IMG_UINT32					i;
#endif
#if !defined(NO_HARDWARE)
	IMG_BOOL bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV;
#endif
#if defined(PDUMP)||(!defined(NO_HARDWARE))
	RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(sBVNC);
	PVRSRV_ERROR				eError;
	
	rgx_bvnc_packed(&sBVNC.ui32BNC, sBVNC.aszV, sBVNC.ui32VLenMax, RGX_BVNC_B, RGX_BVNC_V_ST, RGX_BVNC_N, RGX_BVNC_C);
#endif

#if defined(PDUMP)
	PDUMPCOMMENT("Compatibility check: KM driver and FW BVNC (struct version)");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
											offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
											offsetof(RGXFWIF_COMPCHECKS, sFWBVNC) +
											offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32LayoutVersion),
											sBVNC.ui32LayoutVersion,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
	}

	PDUMPCOMMENT("Compatibility check: KM driver and FW BVNC (maxlen)");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
											offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
											offsetof(RGXFWIF_COMPCHECKS, sFWBVNC) +
											offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32VLenMax),
											sBVNC.ui32VLenMax,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
	}

	PDUMPCOMMENT("Compatibility check: KM driver and FW BVNC (BNC part)");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
											offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
											offsetof(RGXFWIF_COMPCHECKS, sFWBVNC) +
											offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32BNC),
											sBVNC.ui32BNC,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
	}

	for (i = 0; i < sBVNC.ui32VLenMax; i += sizeof(IMG_UINT32))
	{
		PDUMPCOMMENT("Compatibility check: KM driver and FW BVNC (V part)");
		eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, sFWBVNC) +
												offsetof(RGXFWIF_COMPCHECKS_BVNC, aszV) + 
												i,
												*((IMG_UINT32 *)(sBVNC.aszV + i)),
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		}
	}
#endif

#if !defined(NO_HARDWARE)
	if (psRGXFWInit == IMG_NULL)
		return PVRSRV_ERROR_INVALID_PARAMS;

	RGX_BVNC_EQUAL(sBVNC, psRGXFWInit->sRGXCompChecks.sFWBVNC, bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV);
	
	if (!bCompatibleAll)
	{
		if (!bCompatibleVersion)
		{
			PVR_LOG(("(FAIL) %s: Incompatible compatibility struct version of driver (%d) and firmware (%d).",
					__FUNCTION__, 
					sBVNC.ui32LayoutVersion, 
					psRGXFWInit->sRGXCompChecks.sFWBVNC.ui32LayoutVersion));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}

		if (!bCompatibleLenMax)
		{
			PVR_LOG(("(FAIL) %s: Incompatible V maxlen of driver (%d) and firmware (%d).",
					__FUNCTION__, 
					sBVNC.ui32VLenMax, 
					psRGXFWInit->sRGXCompChecks.sFWBVNC.ui32VLenMax));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}

		if (!bCompatibleBNC)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in KM driver BNC (%d._.%d.%d) and Firmware BNC (%d._.%d.%d)",
					RGX_BVNC_PACKED_EXTR_B(sBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sBVNC), 
					RGX_BVNC_PACKED_EXTR_B(psRGXFWInit->sRGXCompChecks.sFWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(psRGXFWInit->sRGXCompChecks.sFWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(psRGXFWInit->sRGXCompChecks.sFWBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}
		
		if (!bCompatibleV)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in KM driver BVNC (%d.%s.%d.%d) and Firmware BVNC (%d.%s.%d.%d)",
					RGX_BVNC_PACKED_EXTR_B(sBVNC), 
					RGX_BVNC_PACKED_EXTR_V(sBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sBVNC), 
					RGX_BVNC_PACKED_EXTR_B(psRGXFWInit->sRGXCompChecks.sFWBVNC), 
					RGX_BVNC_PACKED_EXTR_V(psRGXFWInit->sRGXCompChecks.sFWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(psRGXFWInit->sRGXCompChecks.sFWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(psRGXFWInit->sRGXCompChecks.sFWBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: Firmware BVNC and KM driver BNVC match. [ OK ]"));
	}
#endif
	return PVRSRV_OK;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck_BVNC_HWAgainstDriver

 @Description

 Validate HW BVNC against driver BVNC

 @Input psDevInfo - device info
 @Input psRGXFWInit - FW init data

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
#define COMPAT_BVNC_MASK_B
#define COMPAT_BVNC_MASK_V
#define COMPAT_BVNC_MASK_C
#if !defined(EMULATOR)
static PVRSRV_ERROR RGXDevInitCompatCheck_BVNC_HWAgainstDriver(PVRSRV_RGXDEV_INFO *psDevInfo,
																	RGXFWIF_INIT *psRGXFWInit)
{
#if defined(PDUMP) || (!defined(NO_HARDWARE))
	IMG_UINT32 ui32MaskBNC = RGX_BVNC_PACK_MASK_B |
								RGX_BVNC_PACK_MASK_N |
								RGX_BVNC_PACK_MASK_C;

	IMG_UINT32 bMaskV = IMG_FALSE;

	PVRSRV_ERROR				eError;
	RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(sSWBVNC);
#endif

#if !defined(NO_HARDWARE)
	RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(sHWBVNC);
	IMG_BOOL bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV;
#endif

#if defined(PDUMP) || (!defined(NO_HARDWARE))

#if defined(COMPAT_BVNC_MASK_B)
	ui32MaskBNC &= ~RGX_BVNC_PACK_MASK_B;
#endif
#if defined(COMPAT_BVNC_MASK_V)
	bMaskV = IMG_TRUE;
#endif
#if defined(COMPAT_BVNC_MASK_N)
	ui32MaskBNC &= ~RGX_BVNC_PACK_MASK_N;
#endif
#if defined(COMPAT_BVNC_MASK_C)
	ui32MaskBNC &= ~RGX_BVNC_PACK_MASK_C;
#endif
	
	rgx_bvnc_packed(&sSWBVNC.ui32BNC, sSWBVNC.aszV, sSWBVNC.ui32VLenMax, RGX_BVNC_B, RGX_BVNC_V_ST, RGX_BVNC_N, RGX_BVNC_C);

#endif

#if defined(PDUMP)
	PDUMPCOMMENT("Compatibility check: Layout version of compchecks struct");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
											offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
											offsetof(RGXFWIF_COMPCHECKS, sHWBVNC) +
											offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32LayoutVersion),
											sSWBVNC.ui32LayoutVersion,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		return eError;
	}

	PDUMPCOMMENT("Compatibility check: HW V max len and FW V max len");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
											offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
											offsetof(RGXFWIF_COMPCHECKS, sHWBVNC) +
											offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32VLenMax),
											sSWBVNC.ui32VLenMax,
											0xffffffff,
											PDUMP_POLL_OPERATOR_EQUAL,
											PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
		return eError;
	}

	if (ui32MaskBNC != 0)
	{
		PDUMPCOMMENT("Compatibility check: HW BNC and FW BNC");
		eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, sHWBVNC) +
												offsetof(RGXFWIF_COMPCHECKS_BVNC, ui32BNC),
												sSWBVNC.ui32BNC,
												ui32MaskBNC,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
			return eError;
		}
	}
	if (!bMaskV)
	{
		IMG_UINT32 i;
		for (i = 0; i < sSWBVNC.ui32VLenMax; i += sizeof(IMG_UINT32))
		{
			PDUMPCOMMENT("Compatibility check: HW V and FW V");
			eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
												offsetof(RGXFWIF_INIT, sRGXCompChecks) + 
												offsetof(RGXFWIF_COMPCHECKS, sHWBVNC) +
												offsetof(RGXFWIF_COMPCHECKS_BVNC, aszV) + 
												i,
												*((IMG_UINT32 *)(sSWBVNC.aszV + i)),
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: problem pdumping POL for psRGXFWIfInitMemDesc (%d)", eError));
				return eError;
			}
		}
	}
#endif

#if !defined(NO_HARDWARE)
	if (psRGXFWInit == IMG_NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	sHWBVNC = psRGXFWInit->sRGXCompChecks.sHWBVNC;

	sHWBVNC.ui32BNC &= ui32MaskBNC;
	sSWBVNC.ui32BNC &= ui32MaskBNC;

	if (bMaskV)
	{
		sHWBVNC.aszV[0] = '\0';
		sSWBVNC.aszV[0] = '\0';
	}

	RGX_BVNC_EQUAL(sSWBVNC, sHWBVNC, bCompatibleAll, bCompatibleVersion, bCompatibleLenMax, bCompatibleBNC, bCompatibleV);
	
	if (!bCompatibleAll)
	{
		if (!bCompatibleVersion)
		{
			PVR_LOG(("(FAIL) %s: Incompatible compatibility struct version of HW (%d) and FW (%d).",
					__FUNCTION__, 
					sHWBVNC.ui32LayoutVersion, 
					sSWBVNC.ui32LayoutVersion));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}

		if (!bCompatibleLenMax)
		{
			PVR_LOG(("(FAIL) %s: Incompatible V maxlen of HW (%d) and FW (%d).",
					__FUNCTION__, 
					sHWBVNC.ui32VLenMax, 
					sSWBVNC.ui32VLenMax));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}

		if (!bCompatibleBNC)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Incompatible HW BNC (%d._.%d.%d) and FW BNC (%d._.%d.%d).",
					RGX_BVNC_PACKED_EXTR_B(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_B(sSWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sSWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sSWBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}
		
		if (!bCompatibleV)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Incompatible HW BVNC (%d.%s.%d.%d) and FW BVNC (%d.%s.%d.%d).",
					RGX_BVNC_PACKED_EXTR_B(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_V(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sHWBVNC), 
					RGX_BVNC_PACKED_EXTR_B(sSWBVNC), 
					RGX_BVNC_PACKED_EXTR_V(sSWBVNC), 
					RGX_BVNC_PACKED_EXTR_N(sSWBVNC), 
					RGX_BVNC_PACKED_EXTR_C(sSWBVNC)));
			eError = PVRSRV_ERROR_BVNC_MISMATCH;
			return eError;
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: HW BVNC (%d.%s.%d.%d) and FW BVNC (%d.%s.%d.%d) match. [ OK ]", 
				RGX_BVNC_PACKED_EXTR_B(sHWBVNC), 
				RGX_BVNC_PACKED_EXTR_V(sHWBVNC), 
				RGX_BVNC_PACKED_EXTR_N(sHWBVNC), 
				RGX_BVNC_PACKED_EXTR_C(sHWBVNC), 
				RGX_BVNC_PACKED_EXTR_B(sSWBVNC), 
				RGX_BVNC_PACKED_EXTR_V(sSWBVNC), 
				RGX_BVNC_PACKED_EXTR_N(sSWBVNC), 
				RGX_BVNC_PACKED_EXTR_C(sSWBVNC)));
	}
#endif

	return PVRSRV_OK;
}
#endif

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck

 @Description

 Check compatibility of host driver and firmware (DDK and build options)
 for RGX devices at services/device initialisation

 @Input psDeviceNode - device node

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT32 ui32ClientBuildOptions)
{
	PVRSRV_ERROR		eError;
	PVRSRV_RGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_INIT		*psRGXFWInit = IMG_NULL;

	/* Ensure it's a RGX device */
	if(psDeviceNode->sDevId.eDeviceType != PVRSRV_DEVICE_TYPE_RGX)
	{
		PVR_LOG(("(FAIL) %s: Device not of type RGX", __FUNCTION__));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto chk_exit;
	}

	/* 
	 * Retrieve the FW information
	 */
	
#if !defined(NO_HARDWARE)
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc,
												(IMG_VOID **)&psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire kernel fw compatibility check info (%u)",
				__FUNCTION__, eError));
		return eError;
	}

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		if(psRGXFWInit->sRGXCompChecks.bUpdated)
		{
			/* No need to wait if the FW has already updated the values */
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	if (!psRGXFWInit->sRGXCompChecks.bUpdated)
	{
		eError = PVRSRV_ERROR_TIMEOUT;
		PVR_DPF((PVR_DBG_ERROR,"%s: Missing compatibility info from FW (%u)",
				__FUNCTION__, eError));
		RGXDumpDebugInfo(psDevInfo, IMG_FALSE);
		goto chk_exit;
	}
#endif

	eError = RGXDevInitCompatCheck_KMBuildOptions_FWAgainstDriver(psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}

	eError = RGXDevInitCompatCheck_BuildOptions_FWAgainstClient(psDevInfo, psRGXFWInit, ui32ClientBuildOptions);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}
	
	eError = RGXDevInitCompatCheck_DDKVersion_FWAgainstDriver(psDevInfo, psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}

	eError = RGXDevInitCompatCheck_DDKBuild_FWAgainstDriver(psDevInfo, psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}

	eError = RGXDevInitCompatCheck_BVNC_FWAgainstDriver(psDevInfo, psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}

#if !defined(EMULATOR)
	eError = RGXDevInitCompatCheck_BVNC_HWAgainstDriver(psDevInfo, psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		goto chk_exit;
	}
#endif

	eError = PVRSRV_OK;
chk_exit:
#if !defined(NO_HARDWARE)
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
#endif
	return eError;
}


/******************************************************************************
 End of file (rgxinit.c)
******************************************************************************/
