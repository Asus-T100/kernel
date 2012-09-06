									    /*************************************************************************//*!
									       @File
									       @Title          RGX CCb routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX CCB routines
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pvr_debug.h"
#include "rgxdevice.h"
#include "pdump_km.h"
#include "allocmem.h"
#include "devicemem.h"
#include "rgxfwutils.h"
#include "osfunc.h"
#include "rgxccb.h"

static
PVRSRV_ERROR RGXDestroyCCB(DEVMEM_MEMDESC * psClientCCBMemDesc,
			   DEVMEM_MEMDESC * psClientCCBCtlMemDesc,
			   DEVMEM_EXPORTCOOKIE * psClientCCBExportCookie,
			   DEVMEM_EXPORTCOOKIE * psClientCCBCtlExportCookie);

IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXCreateCCBKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				      IMG_UINT32 ui32AllocSize,
				      IMG_UINT32 ui32AllocAlignment,
				      RGX_CCB_CLEANUP_DATA ** ppsCleanupData,
				      DEVMEM_MEMDESC ** ppsClientCCBMemDesc,
				      DEVMEM_MEMDESC ** ppsClientCCBCtlMemDesc,
				      DEVMEM_EXPORTCOOKIE **
				      psClientCCBExportCookie,
				      DEVMEM_EXPORTCOOKIE **
				      psClientCCBCtlExportCookie)
{
	PVRSRV_ERROR eError;
	DEVMEM_FLAGS_T uiClientCCBMemAllocFlags, uiClientCCBCtlMemAllocFlags;
	RGX_CCB_CLEANUP_DATA *psTmpCleanup;

	psTmpCleanup = OSAllocMem(sizeof(RGX_CCB_CLEANUP_DATA));
	if (psTmpCleanup == IMG_NULL)
		return PVRSRV_ERROR_OUT_OF_MEMORY;

	OSMemSet(psTmpCleanup, 0, sizeof(RGX_CCB_CLEANUP_DATA));

	uiClientCCBMemAllocFlags =
	    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE | PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
	    | PVRSRV_MEMALLOCFLAG_CPU_READABLE | PVRSRV_MEMALLOCFLAG_UNCACHED |
	    PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	uiClientCCBCtlMemAllocFlags =
	    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE | PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
	    | PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    /* FIXME: Client CCB Ctl should be read-only for the CPU 
	       (it is not because for now we initialize it from the host) */
	    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate RGXFW cCCB");
	eError = DevmemFwAllocateExportable(psDeviceNode,
					    ui32AllocSize,
					    uiClientCCBMemAllocFlags,
					    ppsClientCCBMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateCCBKM: Failed to allocate RGX client CCB (%u)",
			 eError));
		goto e0;
	}
	psTmpCleanup->psClientCCBMemDesc = *ppsClientCCBMemDesc;

	/*
	 * Export the CCB allocation so it can be mapped client-side.
	 */
	eError = DevmemExport(*ppsClientCCBMemDesc,
			      &psTmpCleanup->sClientCCBExportCookie);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateCCBKM: Failed to export RGX client CCB (%u)",
			 eError));
		goto e1;
	}
	*psClientCCBExportCookie = &psTmpCleanup->sClientCCBExportCookie;

	PDUMPCOMMENT("Allocate RGXFW cCCB control");
	eError = DevmemFwAllocateExportable(psDeviceNode,
					    sizeof(RGXFWIF_CCCB_CTL),
					    uiClientCCBCtlMemAllocFlags,
					    ppsClientCCBCtlMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateCCBKM: Failed to allocate RGX client CCB control (%u)",
			 eError));
		goto e2;
	}
	psTmpCleanup->psClientCCBCtlMemDesc = *ppsClientCCBCtlMemDesc;

	/*
	 * Export the CCB control allocation so it can be mapped client-side.
	 */
	eError = DevmemExport(*ppsClientCCBCtlMemDesc,
			      &psTmpCleanup->sClientCCBCtlExportCookie);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateCCBKM: Failed to export RGX client CCB control (%u)",
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
    PVRSRV_ERROR PVRSRVRGXDestroyCCBKM(RGX_CCB_CLEANUP_DATA * psCleanupData)
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
PVRSRV_ERROR RGXDestroyCCB(DEVMEM_MEMDESC * psClientCCBMemDesc,
			   DEVMEM_MEMDESC * psClientCCBCtlMemDesc,
			   DEVMEM_EXPORTCOOKIE * psClientCCBExportCookie,
			   DEVMEM_EXPORTCOOKIE * psClientCCBCtlExportCookie)
{
	if (psClientCCBCtlMemDesc != IMG_NULL) {
		DevmemUnexport(psClientCCBCtlMemDesc,
			       psClientCCBCtlExportCookie);
		DevmemFwFree(psClientCCBCtlMemDesc);
	}

	if (psClientCCBMemDesc != IMG_NULL) {
		DevmemUnexport(psClientCCBMemDesc, psClientCCBExportCookie);
		DevmemFwFree(psClientCCBMemDesc);
	}

	return PVRSRV_OK;
}

/******************************************************************************
 End of file (rgxccb.c)
******************************************************************************/
