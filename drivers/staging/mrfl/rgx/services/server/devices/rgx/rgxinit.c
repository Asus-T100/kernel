									    /*************************************************************************//*!
									       @File
									       @Title          Device specific initialisation routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>

#include "rgx_cr_defs_km.h"

#include "rgx_heaps.h"
#include "rgx_fwif_km.h"
#include "rgxheapconfig.h"
#include "rgxdefs.h"
#include "rgxdevice.h"
#include "rgxpower.h"

#include "rgxinit.h"
#include "rgxdebug.h"

#include "pdump_km.h"
#include "ra.h"
#include "mmu_common.h"
#include "handle.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "rgxmem.h"
#include "pvrsrv.h"

#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgx_options.h"

#include "lists.h"
#include "srvkm.h"

/* TODO: sort out the above includes */

/* services5/srvkm/devices/rgx/ */
#include "rgxmmuinit.h"
/* services5/srvkm/include/ */
#include "device.h"
#include "devicemem_heapcfg.h"
#include "devicemem_server.h"
#include "cache_external.h"
/* include5/ */
#include "pvrsrv_error.h"

static PVRSRV_ERROR RGXDevInitCompatCheck(PVRSRV_DEVICE_NODE * psDeviceNode);

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

		     /* PRQA S 0881 11 *//* ignore 'order of evaluation' warning */
#define CHECK_SIZE(NAME) \
{	\
	if (psRGXStructSizes->ui32Sizeof_##NAME != psDevInfo->sRGXStructSizes.ui32Sizeof_##NAME) \
	{	\
		PVR_DPF((PVR_DBG_ERROR, "RGXDevInitCompatCheck: Size check failed for RGXFWIF_%s (client) = %d bytes, (firmware) = %d bytes\n", \
			VAR(NAME), \
			psDevInfo->sRGXStructSizes.ui32Sizeof_##NAME, \
			psRGXStructSizes->ui32Sizeof_##NAME )); \
		bStructSizesFailed = IMG_TRUE; \
	}	\
}

/* FIXME: This is a workaround due to having 2 inits but only 1 deinit */
static IMG_BOOL g_bDevInit2Done = IMG_FALSE;

static IMG_VOID RGX_DeInitHeaps(DEVICE_MEMORY_INFO * psDevMemoryInfo);

#if !defined(NO_HARDWARE)
/*
	RGX LISR Handler
*/
static IMG_BOOL RGX_LISRHandler(IMG_VOID * pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	PVRSRV_RGXDEV_INFO *psDevInfo;
	IMG_UINT32 ui32IRQStatus;
	IMG_BOOL bInterruptProcessed = IMG_FALSE;

	psDeviceNode = pvData;
	psDevInfo = psDeviceNode->pvDevice;

	ui32IRQStatus =
	    OSReadHWReg32(psDevInfo->pvRegsBaseKM,
			  RGX_CR_META_SP_MSLVIRQSTATUS);

	if (ui32IRQStatus & RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_EN) {
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM,
			       RGX_CR_META_SP_MSLVIRQSTATUS,
			       RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_CLRMSK);
		bInterruptProcessed = IMG_TRUE;
		OSScheduleMISR(psDevInfo->pvMISRData);
#if defined(EMULATOR)
		/* Flush register writes */
		(void)OSReadHWReg32(psDevInfo->pvRegsBaseKM,
				    RGX_CR_META_SP_MSLVIRQSTATUS);
#endif
	}
	return bInterruptProcessed;
}

/*
	RGX MISR Handler
*/
static IMG_VOID RGX_MISRHandler(IMG_VOID * pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = pvData;

	/* Inform other services devices that we have finished an operation */
	PVRSRVCheckStatus(psDeviceNode);
}
#endif

/*
 * PVRSRVRGXInitDevPart2KM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXInitDevPart2KM(PVRSRV_DEVICE_NODE * psDeviceNode,
					 RGX_INIT_COMMAND * psInitScript,
					 RGX_INIT_COMMAND * psDbgScript,
					 RGX_INIT_COMMAND * psDeinitScript,
					 IMG_UINT32 ui32KernelCatBase)
{
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_DEV_POWER_STATE eDefaultPowerState;
	PVRSRV_DEVICE_CONFIG *psDevConfig = psDeviceNode->psDevConfig;

	PDUMPCOMMENT("RGX Initialisation Part 2");

	psDevInfo->ui32KernelCatBase = ui32KernelCatBase;

	/*
	 * Map RGX Registers
	 */
#if !defined(NO_HARDWARE)
	psDevInfo->pvRegsBaseKM = OSMapPhysToLin(psDevConfig->sRegsCpuPBase,
						 psDevConfig->ui32RegsSize,
						 IMG_NULL);

	if (psDevInfo->pvRegsBaseKM == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXInitDevPart2KM: Failed to create RGX register mapping\n"));
		return PVRSRV_ERROR_BAD_MAPPING;
	}
#else
	psDevInfo->pvRegsBaseKM = IMG_NULL;
#endif				/* !NO_HARDWARE */

	/* free the export cookies provided to srvinit */
	DevmemUnexport(psDevInfo->psRGXFWMemDesc,
		       &psDevInfo->sRGXFWExportCookie);

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
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_LASTFRAME,
			      "RGX deinitialisation script");
	RGXRunScript(psDevInfo, psDevInfo->sScripts.asDeinitCommands,
		     RGX_MAX_DEINIT_COMMANDS, PDUMP_FLAGS_LASTFRAME);
#endif

#if defined(FIXME)
	/*
	 * Assign client-side build options and firmware IF structure sizes for
	 * later verification (RGXDevInitCompatCheck)
	 */
	psDevInfo->ui32ClientBuildOptions = psInitInfo->ui32ClientBuildOptions;
	psDevInfo->sRGXStructSizes = psInitInfo->sRGXStructSizes;

	/*
	 * Initial state of the cache control.
	 */
	psDevInfo->ui32CacheControl = psInitInfo->ui32CacheControl;

	/*
	 * Initialise Dev Data
	 */
	OSMemCopy(&psDevInfo->asRGXDevData, &psInitInfo->asInitDevData,
		  sizeof(psDevInfo->asRGXDevData));
#endif

	psDevInfo->ui32RegSize = psDevConfig->ui32RegsSize;
	psDevInfo->sRegsPhysBase = psDevConfig->sRegsCpuPBase;

	/* Prevent the device being woken up before there is something to do. */
	eDefaultPowerState = PVRSRV_DEV_POWER_STATE_OFF;
	/* Register the device with the power manager. */
	eError = PVRSRVRegisterPowerDevice(psDeviceNode->sDevId.ui32DeviceIndex,
					   &RGXPrePowerState,
					   &RGXPostPowerState,
					   psDevConfig->pfnPrePowerState,
					   psDevConfig->pfnPostPowerState,
					   &RGXPreClockSpeedChange,
					   &RGXPostClockSpeedChange,
					   (IMG_HANDLE) psDeviceNode,
					   PVRSRV_DEV_POWER_STATE_OFF,
					   eDefaultPowerState);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXInitDevPart2KM: failed to register device with power manager"));
		return eError;
	}
#if !defined(NO_HARDWARE)
	/* Register the interrupt handlers */
	eError = OSInstallMISR(&psDevInfo->pvMISRData,
			       RGX_MISRHandler, psDeviceNode);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = OSInstallDeviceLISR(psDevConfig, &psDevInfo->pvLISRData,
				     RGX_LISRHandler, psDeviceNode);
	if (eError != PVRSRV_OK) {
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
    PVRSRV_ERROR PVRSRVRGXInitFirmwareKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					 IMG_DEVMEM_SIZE_T ui32FWMemAllocSize,
					 DEVMEM_EXPORTCOOKIE **
					 ppsFWMemAllocServerExportCookie,
					 IMG_DEV_VIRTADDR * psFWMemDevVAddrBase,
					 IMG_UINT64 * pui64FWHeapBase,
					 RGXFWIF_DEV_VIRTADDR * psRGXFwInit,
					 IMG_BOOL bEnableSignatureChecks,
					 IMG_UINT32 ui32SignatureChecksBufSize,
					 IMG_UINT32 ui32RGXFWAlignChecksSize,
					 IMG_UINT32 * pui32RGXFWAlignChecks,
					 IMG_UINT32 ui32ConfigFlags)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_ERROR eError = PVRSRV_OK;

	*pui64FWHeapBase = RGX_FIRMWARE_HEAP_BASE;

	/* Register callbacks for creation of device memory contexts */
	psDeviceNode->pfnRegisterMemoryContext = RGXRegisterMemoryContext;
	psDeviceNode->pfnUnregisterMemoryContext = RGXUnregisterMemoryContext;

	/* Create the memory context for the firmware. */
	eError = DevmemCreateContext(IMG_NULL, psDeviceNode,
				     DEVMEM_HEAPCFG_META,
				     &psDevInfo->psKernelDevmemCtx);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "DevInitRGXPart1: Failed DevmemCreateContext (%u)",
			 eError));
		goto failed_to_create_ctx;
	}

	eError = DevmemFindHeapByName(psDevInfo->psKernelDevmemCtx, "Firmware",	/* FIXME: Dave needs to create an IDENT macro for this string.
										   Make sure the IDENT macro is not accessible to userland */
				      &psDevInfo->psFirmwareHeap);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "DevInitRGXPart1: Failed DevmemFindHeapByName (%u)",
			 eError));
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
				  ui32ConfigFlags, psRGXFwInit);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXInitFirmwareKM: RGXSetupFirmware failed (%u)",
			 eError));
		goto failed_init_firmware;
	}

	return eError;

 failed_init_firmware:
 failed_to_find_heap:
	DevmemDestroyContext(psDevInfo->psKernelDevmemCtx);
 failed_to_create_ctx:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/* See device.h for function declaration */
static PVRSRV_ERROR RGXAllocUFOBlock(PVRSRV_DEVICE_NODE * psDeviceNode,
				     DEVMEM_MEMDESC ** psMemDesc,
				     IMG_UINT32 * puiSyncPrimVAddr,
				     IMG_UINT32 * puiSyncPrimBlockSize)
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
					    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
					    (PMMETA_PROTECT) |
					    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE
					    | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC
					    | PVRSRV_MEMALLOCFLAG_CACHE_COHERENT
					    | PVRSRV_MEMALLOCFLAG_GPU_READABLE |
					    PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
					    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
					    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE,
					    psMemDesc);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	DevmemPDumpLoadMem(*psMemDesc,
			   0, ui32UFOBlockSize, PDUMP_FLAGS_CONTINUOUS);

	RGXSetFirmwareAddress(&pFirmwareAddr, *psMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE);
	*puiSyncPrimVAddr = pFirmwareAddr.ui32Addr;
	*puiSyncPrimBlockSize = ui32UFOBlockSize;

	return PVRSRV_OK;

	DevmemFwFree(*psMemDesc);
 e0:
	return eError;
}

/* See device.h for function declaration */
static IMG_VOID RGXFreeUFOBlock(DEVMEM_MEMDESC * psMemDesc)
{
	RGXUnsetFirmwareAddress(psMemDesc);
	DevmemFwFree(psMemDesc);
}

/* See device.h for function declaration */
static IMG_VOID RGXCPUCacheOperation(PVRSRV_DEVICE_NODE * psDeviceNode,
				     PVRSRV_CACHE_OP uiCacheOp)
{
	PVRSRV_RGXDEV_INFO *psDevInfo;

	psDevInfo = psDeviceNode->pvDevice;
	psDevInfo->uiCacheOp = SetCacheOp(psDevInfo->uiCacheOp, uiCacheOp);
}

/*
	DevDeInitRGX
*/
PVRSRV_ERROR DevDeInitRGX(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo =
	    (PVRSRV_RGXDEV_INFO *) psDeviceNode->pvDevice;
	PVRSRV_ERROR eError;
	DEVICE_MEMORY_INFO *psDevMemoryInfo;

	if (!psDevInfo) {
		/* Can happen if DevInitRGX failed */
		PVR_DPF((PVR_DBG_ERROR, "DevDeInitRGX: Null DevInfo"));
		return PVRSRV_OK;
	}
#if defined(FIXME)
	if (psDevInfo->hTimer) {
		eError = OSRemoveTimer(psDevInfo->hTimer);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "DevDeInitRGX: Failed to remove timer"));
			return eError;
		}
		psDevInfo->hTimer = IMG_NULL;
	}
#endif				/* FIXME */

	/* Cancel notifications to this device */
	PVRSRVUnregisterCmdCompleteNotify(&RGXScheduleProcessQueuesKM);

	psDevMemoryInfo = &psDeviceNode->sDevMemoryInfo;

	RGX_DeInitHeaps(psDevMemoryInfo);

	if (DevmemIsValidExportCookie(&psDevInfo->sRGXFWExportCookie)) {
		/* if the export cookie is valid, the init sequence failed */
		PVR_DPF((PVR_DBG_ERROR,
			 "DevDeInitRGX: FW Export cookie still valid (should have been unexported at init time)"));
		DevmemUnexport(psDevInfo->psRGXFWMemDesc,
			       &psDevInfo->sRGXFWExportCookie);
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

	if (psDevInfo->psKernelDevmemCtx) {
		eError = DevmemDestroyContext(psDevInfo->psKernelDevmemCtx);
		/* oops - this should return void -- TODO */
		PVR_ASSERT(eError == PVRSRV_OK);
	}

	if (g_bDevInit2Done) {
#if !defined(NO_HARDWARE)
		(IMG_VOID) OSUninstallDeviceLISR(psDevInfo->pvLISRData);
		(IMG_VOID) OSUninstallMISR(psDevInfo->pvMISRData);
#endif				/* !NO_HARDWARE */

		/* unregister MMU related stuff */
		eError = RGXMMUInit_Unregister(psDeviceNode);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "DevDeInitRGX: Failed RGXMMUInit_Unregister (0x%x)",
				 eError));
			return eError;
		}

		/* remove the device from the power manager */
		eError =
		    PVRSRVRemovePowerDevice(psDeviceNode->sDevId.
					    ui32DeviceIndex);
		if (eError != PVRSRV_OK) {
			return eError;
		}

		/* UnMap Regs */
		if (psDevInfo->pvRegsBaseKM != IMG_NULL) {
#if !defined(NO_HARDWARE)
			OSUnMapPhysToLin(psDevInfo->pvRegsBaseKM,
					 psDevInfo->ui32RegSize, IMG_NULL);
#endif				/* !NO_HARDWARE */
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
static IMG_VOID RGXDumpDebugInfoWrapper(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	RGXDumpDebugInfo(psDeviceNode->pvDevice, IMG_TRUE);
}

#if defined(PDUMP)
static
PVRSRV_ERROR RGXResetPDump(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo =
	    (PVRSRV_RGXDEV_INFO *) (psDeviceNode->pvDevice);
	IMG_UINT32 ui32Idx;

	for (ui32Idx = 0; ui32Idx < RGXFWIF_DM_MAX; ui32Idx++) {
		psDevInfo->abDumpedKCCBCtlAlready[ui32Idx] = IMG_FALSE;
	}

	return PVRSRV_OK;
}
#endif				/* PDUMP */

static PVRSRV_ERROR RGX_InitHeaps(DEVICE_MEMORY_INFO * psNewMemoryInfo)
{
	DEVMEM_HEAP_BLUEPRINT *psDeviceMemoryHeapCursor;

	/* actually - this ought not to be on the device node itself, I think.  Hmmm.  TODO FIXME */
	psNewMemoryInfo->psDeviceMemoryHeap =
	    OSAllocMem(sizeof(DEVMEM_HEAP_BLUEPRINT) * RGX_MAX_HEAP_ID);
	if (psNewMemoryInfo->psDeviceMemoryHeap == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXRegisterDevice : Failed to alloc memory for DEVMEM_HEAP_BLUEPRINT"));
		goto e0;
	}

	psDeviceMemoryHeapCursor = psNewMemoryInfo->psDeviceMemoryHeap;

	/************* general ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_GENERAL_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_GENERAL_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_GENERAL_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

	/************* PDS code and data ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_PDSCODEDATA_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr =
	    RGX_PDSCODEDATA_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_PDSCODEDATA_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

	/************* 3D Parameters ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_3DPARAMETERS_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr =
	    RGX_3DPARAMETERS_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_3DPARAMETERS_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

	/************* USC code ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_USCCODE_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_USCCODE_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_USCCODE_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

	/************* TQ 3D Parameters ***************/
	psDeviceMemoryHeapCursor->pszName = RGX_TQ3DPARAMETERS_HEAP_IDENT;
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr =
	    RGX_TQ3DPARAMETERS_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_TQ3DPARAMETERS_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

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

	/************* Firmware ***************/
	psDeviceMemoryHeapCursor->pszName = "Firmware";
	psDeviceMemoryHeapCursor->sHeapBaseAddr.uiAddr = RGX_FIRMWARE_HEAP_BASE;
	psDeviceMemoryHeapCursor->uiHeapLength = RGX_FIRMWARE_HEAP_SIZE;
	psDeviceMemoryHeapCursor->uiLog2DataPageSize =
	    RGX_MMU_LOG2_PAGE_SIZE_4KB;

	psDeviceMemoryHeapCursor++;	/* advance to the next heap */

	/* set the heap count */
	psNewMemoryInfo->ui32HeapCount =
	    (IMG_UINT32) (psDeviceMemoryHeapCursor -
			  psNewMemoryInfo->psDeviceMemoryHeap);

	/* the new way: we'll set up 2 heap configs: one will be for Meta
	   only, and has only the firmware heap in it. 
	   The remaining one shall be for clients only, and shall have all
	   the other heaps in it */

	psNewMemoryInfo->uiNumHeapConfigs = 2;
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray =
	    OSAllocMem(sizeof(DEVMEM_HEAP_CONFIG) *
		       psNewMemoryInfo->uiNumHeapConfigs);
	if (psNewMemoryInfo->psDeviceMemoryHeapConfigArray == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXRegisterDevice : Failed to alloc memory for DEVMEM_HEAP_CONFIG"));
		goto e1;
	}

	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].pszName =
	    "Default Heap Configuration";
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].uiNumHeaps =
	    psNewMemoryInfo->ui32HeapCount - 1;
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[0].psHeapBlueprintArray =
	    psNewMemoryInfo->psDeviceMemoryHeap;

	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].pszName =
	    "Firmware Heap Configuration";
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].uiNumHeaps = 1;
	psNewMemoryInfo->psDeviceMemoryHeapConfigArray[1].psHeapBlueprintArray =
	    psDeviceMemoryHeapCursor - 1;

	return PVRSRV_OK;
 e1:
	OSFreeMem(psNewMemoryInfo->psDeviceMemoryHeap);
 e0:
	return PVRSRV_ERROR_OUT_OF_MEMORY;
}

static IMG_VOID RGX_DeInitHeaps(DEVICE_MEMORY_INFO * psDevMemoryInfo)
{
	OSFreeMem(psDevMemoryInfo->psDeviceMemoryHeapConfigArray);
	OSFreeMem(psDevMemoryInfo->psDeviceMemoryHeap);
}

/*
	RGXRegisterDevice
*/
PVRSRV_ERROR RGXRegisterDevice(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	PVRSRV_ERROR eError;
	DEVICE_MEMORY_INFO *psDevMemoryInfo;
	PVRSRV_RGXDEV_INFO *psDevInfo;

	/* pdump info about the core */
	PDUMPCOMMENT("RGX Version Information: %s", RGX_BVNC);

#if defined(RGX_FEATURE_SYSTEM_CACHE)
	PDUMPCOMMENT("RGX System Level Cache is present");
#endif				/* RGX_FEATURE_SYSTEM_CACHE */

	PDUMPCOMMENT("RGX Initialisation (Part 1)");

	/*********************
	 * Device node setup *
	 *********************/
	/* Setup static data and callbacks on the device agnostic device node */
	psDeviceNode->sDevId.eDeviceType = DEV_DEVICE_TYPE;
	psDeviceNode->sDevId.eDeviceClass = DEV_DEVICE_CLASS;
#if defined(PDUMP)
	psDeviceNode->sDevId.pszPDumpRegName = RGX_PDUMPREG_NAME;
	/*
	   FIXME: This should not be required as PMR's should give the memspace
	   name. However, due to limitations within PDump we need a memspace name
	   when dpumping with MMU context with virtual address in which case we
	   don't have a PMR to get the name from.
	 */
	psDeviceNode->sDevId.pszPDumpDevName = "SYSMEM";
	psDeviceNode->pfnPDumpInitDevice = &RGXResetPDump;
#endif				/* PDUMP */

#if defined(SUPPORT_MEMORY_TILING)
	psDeviceNode->pfnAllocMemTilingRange = RGX_AllocMemTilingRange;
	psDeviceNode->pfnFreeMemTilingRange = RGX_FreeMemTilingRange;
#endif

	/* Configure MMU specific stuff */
	RGXMMUInit_Register(psDeviceNode);

	psDeviceNode->pfnMMUCacheInvalidate = RGXMMUCacheInvalidate;

	psDeviceNode->pfnSLCCacheInvalidateRequest =
	    RGXSLCCacheInvalidateRequest;

	/* Register RGX to recieve notifies when other devices complete some work */
	PVRSRVRegisterCmdCompleteNotify(&RGXScheduleProcessQueuesKM,
					psDeviceNode);

	psDeviceNode->pfnInitDeviceCompatCheck = &RGXDevInitCompatCheck;

	/* Register callbacks for creation of device memory contexts */
	psDeviceNode->pfnRegisterMemoryContext = RGXRegisterMemoryContext;
	psDeviceNode->pfnUnregisterMemoryContext = RGXUnregisterMemoryContext;

	/* Register callbacks for Unified Fence Objects */
	psDeviceNode->pfnAllocUFOBlock = RGXAllocUFOBlock;
	psDeviceNode->pfnFreeUFOBlock = RGXFreeUFOBlock;

	psDeviceNode->pfnCPUCacheOperation = RGXCPUCacheOperation;

	/* Register callback for dumping debug info */
	psDeviceNode->pfnDumpDebugInfo = RGXDumpDebugInfoWrapper;

	/*********************
	 * Device info setup *
	 *********************/
	/* Allocate device control block */
	psDevInfo = OSAllocMem(sizeof(*psDevInfo));
	if (psDevInfo == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "DevInitRGXPart1 : Failed to alloc memory for DevInfo"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}
	OSMemSet(psDevInfo, 0, sizeof(*psDevInfo));
	psDeviceNode->pvDevice = psDevInfo;

	/* Setup static data and callbacks on the device specific device info */
	psDevInfo->eDeviceType = DEV_DEVICE_TYPE;
	psDevInfo->eDeviceClass = DEV_DEVICE_CLASS;
	psDevInfo->psDeviceNode = psDeviceNode;

	psDevMemoryInfo = &psDeviceNode->sDevMemoryInfo;
	psDevMemoryInfo->ui32AddressSpaceSizeLog2 =
	    RGX_FEATURE_ADDRESS_SPACE_SIZE;
	psDevInfo->pvDeviceMemoryHeap = psDevMemoryInfo->psDeviceMemoryHeap;

	/* flags, backing store details to be specified by system */
	psDevMemoryInfo->ui32Flags = 0;

	eError = RGX_InitHeaps(psDevMemoryInfo);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*!
*******************************************************************************

 @Function	RGXDevInitCompatCheck

 @Description

 Check compatibility of host driver and firmware (DDK and build options)
 for RGX devices at services/device initialisation

 @Input psDeviceNode - device node

 @Return   PVRSRV_ERROR - depending on mismatch found

******************************************************************************/
static PVRSRV_ERROR RGXDevInitCompatCheck(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	PVRSRV_ERROR eError;

	/* Ensure it's a RGX device */
	if (psDeviceNode->sDevId.eDeviceType != PVRSRV_DEVICE_TYPE_RGX) {
		PVR_LOG(("(FAIL) RGXInit: Device not of type RGX"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto chk_exit;
	}

	/*
	 *  1. Check kernel-side and client-side build options
	 *      2. Ensure firmware DDK version and driver DDK version are compatible
	 *      3. Check firmware build options against kernel-side build options
	 */

	/*
	 * Check KM build options against client-side host driver
	 */

#if defined(FIXME)
	/* FIXME
	 * The build options for prvsrvinit should not be a special case. There is
	 * already code checking client flags against server flags in PVRSRVConnect,
	 * but pvrsrvctl is bypassing this.
	 */
	ui32BuildOptions = (RGX_BUILD_OPTIONS);
	if (ui32BuildOptions != psDevInfo->ui32ClientBuildOptions) {
		ui32BuildOptionsMismatch =
		    ui32BuildOptions ^ psDevInfo->ui32ClientBuildOptions;
		if ((psDevInfo->
		     ui32ClientBuildOptions & ui32BuildOptionsMismatch) != 0) {
			PVR_LOG(("(FAIL) RGXInit: Mismatch in client-side and KM driver build options; " "extra options present in client-side driver: (0x%x). Please check rgx_options.h", psDevInfo->ui32ClientBuildOptions & ui32BuildOptionsMismatch));
		}

		if ((ui32BuildOptions & ui32BuildOptionsMismatch) != 0) {
			PVR_LOG(("(FAIL) RGXInit: Mismatch in client-side and KM driver build options; " "extra options present in KM: (0x%x). Please check rgx_options.h", ui32BuildOptions & ui32BuildOptionsMismatch));
		}
		eError = PVRSRV_ERROR_BUILD_MISMATCH;
		goto chk_exit;
	} else {
		PVR_DPF((PVR_DBG_MESSAGE,
			 "RGXInit: Client-side and KM driver build options match. [ OK ]"));
	}
#endif

#if defined (FIXME)		/* All this compat stuff need reworking */
	psMemInfo = psDevInfo->psKernelRGXMiscMemInfo;

	/* Clear state (not strictly necessary since this is the first call) */
	psRGXMiscInfoInt = psMemInfo->pvLinAddrKM;
	psRGXMiscInfoInt->ui32MiscInfoFlags = 0;
/* 	psRGXMiscInfoInt->ui32MiscInfoFlags |= PVRSRV_USSE_MISCINFO_GET_STRUCT_SIZES; */
	eError = RGXGetFirmwareMiscInfo(psDevInfo, psDeviceNode);

	/*
	 * Validate DDK version
	 */
	if (eError != PVRSRV_OK) {
		PVR_LOG(("(FAIL) RGXInit: Unable to validate device DDK version"));
		goto chk_exit;
	}
	psRGXFeatures =
	    &((PVRSRV_RGX_MISCINFO_INFO *) (psMemInfo->pvLinAddrKM))->
	    sRGXFeatures;
	if ((psRGXFeatures->ui32DDKVersion !=
	     ((PVRVERSION_MAJ << 16) | (PVRVERSION_MIN << 8))
	     || (psRGXFeatures->ui32DDKBuild != PVRVERSION_BUILD)) {
	    PVR_LOG(("(FAIL) RGXInit: Incompatible driver DDK revision (%d)/device DDK revision (%d).", PVRVERSION_BUILD, psRGXFeatures->ui32DDKBuild)); eError = PVRSRV_ERROR_DDK_VERSION_MISMATCH; PVR_DBG_BREAK; goto chk_exit;}
	    else
	    {
	    PVR_DPF((PVR_DBG_MESSAGE,
		     "RGXInit: driver DDK (%d) and device DDK (%d) match. [ OK ]",
		     PVRVERSION_BUILD, psRGXFeatures->ui32DDKBuild));}

	    /*
	     *      Check hardware core revision is compatible with the one in software
	     */
	    if (psRGXFeatures->ui32CoreRevSW == 0) {
	    /*
	       Head core revision cannot be checked.
	     */
	    PVR_LOG(("RGXInit: HW core rev (%x) check skipped.",
		     psRGXFeatures->ui32CoreRev));}
	    else
	    {
	    /* For some cores the hw/sw core revisions are expected not to match. For these
	     * exceptional cases the core rev compatibility check should be skipped.
	     */
	    bCheckCoreRev = IMG_TRUE;
	    for (i = 0; i < ui32NumCoreExceptions; i += 2) {
	    if ((psRGXFeatures->ui32CoreRev == aui32CoreRevExceptions[i]) &&
		(psRGXFeatures->ui32CoreRevSW == aui32CoreRevExceptions[i + 1]))
	    {
	    PVR_LOG(("RGXInit: HW core rev (%x), SW core rev (%x) check skipped.", psRGXFeatures->ui32CoreRev, psRGXFeatures->ui32CoreRevSW)); bCheckCoreRev = IMG_FALSE;}
	    }

	    if (bCheckCoreRev) {
	    if (psRGXFeatures->ui32CoreRev != psRGXFeatures->ui32CoreRevSW) {
	    PVR_LOG(("(FAIL) RGXInit: Incompatible HW core rev (%x) and SW core rev (%x).", psRGXFeatures->ui32CoreRev, psRGXFeatures->ui32CoreRevSW)); eError = PVRSRV_ERROR_BUILD_MISMATCH; goto chk_exit;}
	    else
	    {
	    PVR_DPF((PVR_DBG_MESSAGE,
		     "RGXInit: HW core rev (%x) and SW core rev (%x) match. [ OK ]",
		     psRGXFeatures->ui32CoreRev,
		     psRGXFeatures->ui32CoreRevSW));}
	    }
	    }

	    /*
	     * Check firmware build options against KM host driver
	     */

	    ui32BuildOptions = psRGXFeatures->ui32BuildOptions;
	    if (ui32BuildOptions != (RGX_BUILD_OPTIONS)) {
	    ui32BuildOptionsMismatch = ui32BuildOptions ^ (RGX_BUILD_OPTIONS);
	    if (((RGX_BUILD_OPTIONS) & ui32BuildOptionsMismatch) != 0) {
	    PVR_LOG(("(FAIL) RGXInit: Mismatch in driver and firmware build options; " "extra options present in driver: (0x%x). Please check rgx_options.h", (RGX_BUILD_OPTIONS) & ui32BuildOptionsMismatch));}

	    if ((ui32BuildOptions & ui32BuildOptionsMismatch) != 0) {
	    PVR_LOG(("(FAIL) RGXInit: Mismatch in driver and firmware build options; " "extra options present in firmware: (0x%x). Please check rgx_options.h", ui32BuildOptions & ui32BuildOptionsMismatch));}
	    eError = PVRSRV_ERROR_BUILD_MISMATCH; goto chk_exit;}
	    else
	    {
	    PVR_DPF((PVR_DBG_MESSAGE,
		     "RGXInit: Driver and firmware build options match. [ OK ]"));}
#endif				// NO_HARDWARE

 eError = PVRSRV_OK; chk_exit:
	    return eError;}

/******************************************************************************
 End of file (rgxinit.c)
******************************************************************************/
