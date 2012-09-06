									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for mm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for mm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "devicemem_server.h"
#include "pmr.h"
#include "devicemem_heapcfg.h"
#include "physmem.h"

#include "common_mm_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR PMRUnexportPMRResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR
PMRUnmakeServerExportClientExportResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DevmemIntCtxDestroyResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DevmemIntHeapDestroyResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DevmemIntUnmapPMRResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DevmemIntUnreserveRangeResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR PMRUnrefPMRResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgePMRExportPMR(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_PMREXPORTPMR * psPMRExportPMRIN,
			 PVRSRV_BRIDGE_OUT_PMREXPORTPMR * psPMRExportPMROUT,
			 CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;
	PMR_EXPORT *psPMRExportInt;
	IMG_HANDLE hPMRExportInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_MM_PMREXPORTPMR);

	/* Look up the address from the handle */
	psPMRExportPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRExportPMRIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRExportPMROUT->eError != PVRSRV_OK) {
		goto PMRExportPMR_exit;
	}

	/* Look up the data from the resman address */
	psPMRExportPMROUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRExportPMROUT->eError != PVRSRV_OK) {
		goto PMRExportPMR_exit;
	}

	psPMRExportPMROUT->eError =
	    PMRExportPMR(psPMRInt,
			 &psPMRExportInt,
			 &psPMRExportPMROUT->ui32Size,
			 &psPMRExportPMROUT->ui32Log2Contig,
			 &psPMRExportPMROUT->ui32Password);
	/* Exit early if bridged call fails */
	if (psPMRExportPMROUT->eError != PVRSRV_OK) {
		goto PMRExportPMR_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPMRExportInt2 = ResManRegisterRes(psConnection->hResManContext,
					   RESMAN_TYPE_PMR_EXPORT,
					   psPMRExportInt, 0,
					   /* FIXME: how can we avoid this cast? */
					   (RESMAN_FREE_FN) & PMRUnexportPMR);
	if (hPMRExportInt2 == IMG_NULL) {
		psPMRExportPMROUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PMRExportPMR_exit;
	}
	/* see if it's already exported */
	/* FIXME: This code should go when we implement serverImport, serverUnimport */
	psPMRExportPMROUT->eError =
	    PVRSRVFindHandle(KERNEL_HANDLE_BASE,
			     &psPMRExportPMROUT->hPMRExport,
			     (IMG_HANDLE) hPMRExportInt2,
			     PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);
	if (psPMRExportPMROUT->eError == PVRSRV_OK) {
		/* it's already exported */
		return 0;
	}

	psPMRExportPMROUT->eError =
	    PVRSRVAllocHandle(KERNEL_HANDLE_BASE,
			      &psPMRExportPMROUT->hPMRExport,
			      (IMG_HANDLE) hPMRExportInt2,
			      PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psPMRExportPMROUT->eError != PVRSRV_OK) {
		goto PMRExportPMR_exit;
	}

 PMRExportPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRUnexportPMR(IMG_UINT32 ui32BridgeID,
			   PVRSRV_BRIDGE_IN_PMRUNEXPORTPMR * psPMRUnexportPMRIN,
			   PVRSRV_BRIDGE_OUT_PMRUNEXPORTPMR *
			   psPMRUnexportPMROUT, CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hPMRExportInt2;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_MM_PMRUNEXPORTPMR);

	/* Look up the address from the handle */
	psPMRUnexportPMROUT->eError =
	    PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
			       (IMG_HANDLE *) & hPMRExportInt2,
			       psPMRUnexportPMRIN->hPMRExport,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);
	if (psPMRUnexportPMROUT->eError != PVRSRV_OK) {
		goto PMRUnexportPMR_exit;
	}

	psPMRUnexportPMROUT->eError = PMRUnexportPMRResManProxy(hPMRExportInt2);
	/* Exit early if bridged call fails */
	if (psPMRUnexportPMROUT->eError != PVRSRV_OK) {
		goto PMRUnexportPMR_exit;
	}

	psPMRUnexportPMROUT->eError =
	    PVRSRVReleaseHandle(KERNEL_HANDLE_BASE,
				(IMG_HANDLE) psPMRUnexportPMRIN->hPMRExport,
				PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);

 PMRUnexportPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRGetUID(IMG_UINT32 ui32BridgeID,
		      PVRSRV_BRIDGE_IN_PMRGETUID * psPMRGetUIDIN,
		      PVRSRV_BRIDGE_OUT_PMRGETUID * psPMRGetUIDOUT,
		      CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_MM_PMRGETUID);

	/* Look up the address from the handle */
	psPMRGetUIDOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRGetUIDIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRGetUIDOUT->eError != PVRSRV_OK) {
		goto PMRGetUID_exit;
	}

	/* Look up the data from the resman address */
	psPMRGetUIDOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRGetUIDOUT->eError != PVRSRV_OK) {
		goto PMRGetUID_exit;
	}

	psPMRGetUIDOUT->eError = PMRGetUID(psPMRInt, &psPMRGetUIDOUT->ui32UID);

 PMRGetUID_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRMakeServerExportClientExport(IMG_UINT32 ui32BridgeID,
					    PVRSRV_BRIDGE_IN_PMRMAKESERVEREXPORTCLIENTEXPORT
					    *
					    psPMRMakeServerExportClientExportIN,
					    PVRSRV_BRIDGE_OUT_PMRMAKESERVEREXPORTCLIENTEXPORT
					    *
					    psPMRMakeServerExportClientExportOUT,
					    CONNECTION_DATA * psConnection)
{
	DEVMEM_EXPORTCOOKIE *psPMRServerExportInt;
	PMR_EXPORT *psPMRExportOutInt;
	IMG_HANDLE hPMRExportOutInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_PMRMAKESERVEREXPORTCLIENTEXPORT);

	/* Look up the address from the handle */
	psPMRMakeServerExportClientExportOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & psPMRServerExportInt,
			       psPMRMakeServerExportClientExportIN->
			       hPMRServerExport,
			       PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE);
	if (psPMRMakeServerExportClientExportOUT->eError != PVRSRV_OK) {
		goto PMRMakeServerExportClientExport_exit;
	}

	psPMRMakeServerExportClientExportOUT->eError =
	    PMRMakeServerExportClientExport(psPMRServerExportInt,
					    &psPMRExportOutInt,
					    &psPMRMakeServerExportClientExportOUT->
					    ui32Size,
					    &psPMRMakeServerExportClientExportOUT->
					    ui32Log2Contig,
					    &psPMRMakeServerExportClientExportOUT->
					    ui32Password);
	/* Exit early if bridged call fails */
	if (psPMRMakeServerExportClientExportOUT->eError != PVRSRV_OK) {
		goto PMRMakeServerExportClientExport_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPMRExportOutInt2 = ResManRegisterRes(psConnection->hResManContext,
					      RESMAN_TYPE_PMR_EXPORT,
					      psPMRExportOutInt, 0,
					      /* FIXME: how can we avoid this cast? */
					      (RESMAN_FREE_FN) &
					      PMRUnmakeServerExportClientExport);
	if (hPMRExportOutInt2 == IMG_NULL) {
		psPMRMakeServerExportClientExportOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PMRMakeServerExportClientExport_exit;
	}
	/* see if it's already exported */
	/* FIXME: This code should go when we implement serverImport, serverUnimport */
	psPMRMakeServerExportClientExportOUT->eError =
	    PVRSRVFindHandle(KERNEL_HANDLE_BASE,
			     &psPMRMakeServerExportClientExportOUT->
			     hPMRExportOut, (IMG_HANDLE) hPMRExportOutInt2,
			     PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);
	if (psPMRMakeServerExportClientExportOUT->eError == PVRSRV_OK) {
		/* it's already exported */
		return 0;
	}

	psPMRMakeServerExportClientExportOUT->eError =
	    PVRSRVAllocHandle(KERNEL_HANDLE_BASE,
			      &psPMRMakeServerExportClientExportOUT->
			      hPMRExportOut, (IMG_HANDLE) hPMRExportOutInt2,
			      PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psPMRMakeServerExportClientExportOUT->eError != PVRSRV_OK) {
		goto PMRMakeServerExportClientExport_exit;
	}

 PMRMakeServerExportClientExport_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRUnmakeServerExportClientExport(IMG_UINT32 ui32BridgeID,
					      PVRSRV_BRIDGE_IN_PMRUNMAKESERVEREXPORTCLIENTEXPORT
					      *
					      psPMRUnmakeServerExportClientExportIN,
					      PVRSRV_BRIDGE_OUT_PMRUNMAKESERVEREXPORTCLIENTEXPORT
					      *
					      psPMRUnmakeServerExportClientExportOUT,
					      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hPMRExportInt2;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_PMRUNMAKESERVEREXPORTCLIENTEXPORT);

	/* Look up the address from the handle */
	psPMRUnmakeServerExportClientExportOUT->eError =
	    PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
			       (IMG_HANDLE *) & hPMRExportInt2,
			       psPMRUnmakeServerExportClientExportIN->
			       hPMRExport,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);
	if (psPMRUnmakeServerExportClientExportOUT->eError != PVRSRV_OK) {
		goto PMRUnmakeServerExportClientExport_exit;
	}

	psPMRUnmakeServerExportClientExportOUT->eError =
	    PMRUnmakeServerExportClientExportResManProxy(hPMRExportInt2);
	/* Exit early if bridged call fails */
	if (psPMRUnmakeServerExportClientExportOUT->eError != PVRSRV_OK) {
		goto PMRUnmakeServerExportClientExport_exit;
	}

	psPMRUnmakeServerExportClientExportOUT->eError =
	    PVRSRVReleaseHandle(KERNEL_HANDLE_BASE,
				(IMG_HANDLE)
				psPMRUnmakeServerExportClientExportIN->
				hPMRExport,
				PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);

 PMRUnmakeServerExportClientExport_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRImportPMR(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_PMRIMPORTPMR * psPMRImportPMRIN,
			 PVRSRV_BRIDGE_OUT_PMRIMPORTPMR * psPMRImportPMROUT,
			 CONNECTION_DATA * psConnection)
{
	PMR_EXPORT *psPMRExportInt;
	IMG_HANDLE hPMRExportInt2;
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_MM_PMRIMPORTPMR);

	/* Look up the address from the handle */
	psPMRImportPMROUT->eError =
	    PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
			       (IMG_HANDLE *) & hPMRExportInt2,
			       psPMRImportPMRIN->hPMRExport,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT);
	if (psPMRImportPMROUT->eError != PVRSRV_OK) {
		goto PMRImportPMR_exit;
	}

	/* Look up the data from the resman address */
	psPMRImportPMROUT->eError =
	    ResManFindPrivateDataByPtr(hPMRExportInt2,
				       (IMG_VOID **) & psPMRExportInt,
				       IMG_NULL);
	if (psPMRImportPMROUT->eError != PVRSRV_OK) {
		goto PMRImportPMR_exit;
	}

	psPMRImportPMROUT->eError =
	    PMRImportPMR(psPMRExportInt,
			 psPMRImportPMRIN->ui32uiPassword,
			 psPMRImportPMRIN->ui32uiSize,
			 psPMRImportPMRIN->ui32uiLog2Contig, &psPMRInt);
	/* Exit early if bridged call fails */
	if (psPMRImportPMROUT->eError != PVRSRV_OK) {
		goto PMRImportPMR_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPMRInt2 = ResManRegisterRes(psConnection->hResManContext,
				     RESMAN_TYPE_PMR, psPMRInt, 0,
				     /* FIXME: how can we avoid this cast? */
				     (RESMAN_FREE_FN) & PMRUnrefPMR);
	if (hPMRInt2 == IMG_NULL) {
		psPMRImportPMROUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PMRImportPMR_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psPMRImportPMROUT->hPMR,
			    (IMG_HANDLE) hPMRInt2,
			    PVRSRV_HANDLE_TYPE_PHYSMEM_PMR,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);

 PMRImportPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntCtxCreate(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_DEVMEMINTCTXCREATE *
			       psDevmemIntCtxCreateIN,
			       PVRSRV_BRIDGE_OUT_DEVMEMINTCTXCREATE *
			       psDevmemIntCtxCreateOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	DEVMEMINT_CTX *psDevMemServerContextInt;
	IMG_HANDLE hDevMemServerContextInt2;
	IMG_HANDLE hPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTCTXCREATE);

	NEW_HANDLE_BATCH_OR_ERROR(psDevmemIntCtxCreateOUT->eError, psConnection,
				  2)

	    /* Look up the address from the handle */
	    psDevmemIntCtxCreateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psDevmemIntCtxCreateIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psDevmemIntCtxCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntCtxCreate_exit;
	}

	psDevmemIntCtxCreateOUT->eError =
	    DevmemIntCtxCreate(hDeviceNodeInt,
			       &psDevMemServerContextInt, &hPrivDataInt);
	/* Exit early if bridged call fails */
	if (psDevmemIntCtxCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntCtxCreate_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hDevMemServerContextInt2 =
	    ResManRegisterRes(psConnection->hResManContext,
			      RESMAN_TYPE_DEVICEMEM2_CONTEXT,
			      psDevMemServerContextInt, 0,
			      /* FIXME: how can we avoid this cast? */
			      (RESMAN_FREE_FN) & DevmemIntCtxDestroy);
	if (hDevMemServerContextInt2 == IMG_NULL) {
		psDevmemIntCtxCreateOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DevmemIntCtxCreate_exit;
	}
	psDevmemIntCtxCreateOUT->eError =
	    PVRSRVAllocHandle(psConnection->psHandleBase,
			      &psDevmemIntCtxCreateOUT->hDevMemServerContext,
			      (IMG_HANDLE) hDevMemServerContextInt2,
			      PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psDevmemIntCtxCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntCtxCreate_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psDevmemIntCtxCreateOUT->hPrivData,
			       (IMG_HANDLE) hPrivDataInt,
			       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psDevmemIntCtxCreateOUT->hDevMemServerContext);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDevmemIntCtxCreateOUT->eError,
				     psConnection);

 DevmemIntCtxCreate_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntCtxDestroy(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_DEVMEMINTCTXDESTROY *
				psDevmemIntCtxDestroyIN,
				PVRSRV_BRIDGE_OUT_DEVMEMINTCTXDESTROY *
				psDevmemIntCtxDestroyOUT,
				CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevmemServerContextInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTCTXDESTROY);

	/* Look up the address from the handle */
	psDevmemIntCtxDestroyOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemServerContextInt2,
			       psDevmemIntCtxDestroyIN->hDevmemServerContext,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX);
	if (psDevmemIntCtxDestroyOUT->eError != PVRSRV_OK) {
		goto DevmemIntCtxDestroy_exit;
	}

	psDevmemIntCtxDestroyOUT->eError =
	    DevmemIntCtxDestroyResManProxy(hDevmemServerContextInt2);
	/* Exit early if bridged call fails */
	if (psDevmemIntCtxDestroyOUT->eError != PVRSRV_OK) {
		goto DevmemIntCtxDestroy_exit;
	}

	psDevmemIntCtxDestroyOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDevmemIntCtxDestroyIN->
				hDevmemServerContext,
				PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX);

 DevmemIntCtxDestroy_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntHeapCreate(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_DEVMEMINTHEAPCREATE *
				psDevmemIntHeapCreateIN,
				PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPCREATE *
				psDevmemIntHeapCreateOUT,
				CONNECTION_DATA * psConnection)
{
	DEVMEMINT_CTX *psDevmemCtxInt;
	IMG_HANDLE hDevmemCtxInt2;
	DEVMEMINT_HEAP *psDevmemHeapPtrInt;
	IMG_HANDLE hDevmemHeapPtrInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTHEAPCREATE);

	NEW_HANDLE_BATCH_OR_ERROR(psDevmemIntHeapCreateOUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psDevmemIntHeapCreateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemCtxInt2,
			       psDevmemIntHeapCreateIN->hDevmemCtx,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX);
	if (psDevmemIntHeapCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntHeapCreate_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntHeapCreateOUT->eError =
	    ResManFindPrivateDataByPtr(hDevmemCtxInt2,
				       (IMG_VOID **) & psDevmemCtxInt,
				       IMG_NULL);
	if (psDevmemIntHeapCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntHeapCreate_exit;
	}

	psDevmemIntHeapCreateOUT->eError =
	    DevmemIntHeapCreate(psDevmemCtxInt,
				psDevmemIntHeapCreateIN->sHeapBaseAddr,
				psDevmemIntHeapCreateIN->uiHeapLength,
				psDevmemIntHeapCreateIN->ui32Log2DataPageSize,
				&psDevmemHeapPtrInt);
	/* Exit early if bridged call fails */
	if (psDevmemIntHeapCreateOUT->eError != PVRSRV_OK) {
		goto DevmemIntHeapCreate_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hDevmemHeapPtrInt2 = ResManRegisterRes(psConnection->hResManContext,
					       RESMAN_TYPE_DEVICEMEM2_HEAP,
					       psDevmemHeapPtrInt, 0,
					       /* FIXME: how can we avoid this cast? */
					       (RESMAN_FREE_FN) &
					       DevmemIntHeapDestroy);
	if (hDevmemHeapPtrInt2 == IMG_NULL) {
		psDevmemIntHeapCreateOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DevmemIntHeapCreate_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDevmemIntHeapCreateOUT->hDevmemHeapPtr,
			    (IMG_HANDLE) hDevmemHeapPtrInt2,
			    PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDevmemIntHeapCreateOUT->eError,
				     psConnection);

 DevmemIntHeapCreate_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntHeapDestroy(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_DEVMEMINTHEAPDESTROY *
				 psDevmemIntHeapDestroyIN,
				 PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPDESTROY *
				 psDevmemIntHeapDestroyOUT,
				 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevmemHeapInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTHEAPDESTROY);

	/* Look up the address from the handle */
	psDevmemIntHeapDestroyOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemHeapInt2,
			       psDevmemIntHeapDestroyIN->hDevmemHeap,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP);
	if (psDevmemIntHeapDestroyOUT->eError != PVRSRV_OK) {
		goto DevmemIntHeapDestroy_exit;
	}

	psDevmemIntHeapDestroyOUT->eError =
	    DevmemIntHeapDestroyResManProxy(hDevmemHeapInt2);
	/* Exit early if bridged call fails */
	if (psDevmemIntHeapDestroyOUT->eError != PVRSRV_OK) {
		goto DevmemIntHeapDestroy_exit;
	}

	psDevmemIntHeapDestroyOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDevmemIntHeapDestroyIN->
				hDevmemHeap, PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP);

 DevmemIntHeapDestroy_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntMapPMR(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_DEVMEMINTMAPPMR *
			    psDevmemIntMapPMRIN,
			    PVRSRV_BRIDGE_OUT_DEVMEMINTMAPPMR *
			    psDevmemIntMapPMROUT,
			    CONNECTION_DATA * psConnection)
{
	DEVMEMINT_HEAP *psDevmemServerHeapInt;
	IMG_HANDLE hDevmemServerHeapInt2;
	DEVMEMINT_RESERVATION *psReservationInt;
	IMG_HANDLE hReservationInt2;
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;
	DEVMEMINT_MAPPING *psMappingInt;
	IMG_HANDLE hMappingInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTMAPPMR);

	NEW_HANDLE_BATCH_OR_ERROR(psDevmemIntMapPMROUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psDevmemIntMapPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemServerHeapInt2,
			       psDevmemIntMapPMRIN->hDevmemServerHeap,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntMapPMROUT->eError =
	    ResManFindPrivateDataByPtr(hDevmemServerHeapInt2,
				       (IMG_VOID **) & psDevmemServerHeapInt,
				       IMG_NULL);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}
	/* Look up the address from the handle */
	psDevmemIntMapPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hReservationInt2,
			       psDevmemIntMapPMRIN->hReservation,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_RESERVATION);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntMapPMROUT->eError =
	    ResManFindPrivateDataByPtr(hReservationInt2,
				       (IMG_VOID **) & psReservationInt,
				       IMG_NULL);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}
	/* Look up the address from the handle */
	psDevmemIntMapPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psDevmemIntMapPMRIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntMapPMROUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}

	psDevmemIntMapPMROUT->eError =
	    DevmemIntMapPMR(psDevmemServerHeapInt,
			    psReservationInt,
			    psPMRInt,
			    psDevmemIntMapPMRIN->uiMapFlags, &psMappingInt);
	/* Exit early if bridged call fails */
	if (psDevmemIntMapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntMapPMR_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hMappingInt2 = ResManRegisterRes(psConnection->hResManContext,
					 RESMAN_TYPE_DEVICEMEM2_MAPPING,
					 psMappingInt, 0,
					 /* FIXME: how can we avoid this cast? */
					 (RESMAN_FREE_FN) & DevmemIntUnmapPMR);
	if (hMappingInt2 == IMG_NULL) {
		psDevmemIntMapPMROUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DevmemIntMapPMR_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDevmemIntMapPMROUT->hMapping,
			    (IMG_HANDLE) hMappingInt2,
			    PVRSRV_HANDLE_TYPE_DEVMEMINT_MAPPING,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDevmemIntMapPMROUT->eError,
				     psConnection);

 DevmemIntMapPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntUnmapPMR(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_DEVMEMINTUNMAPPMR *
			      psDevmemIntUnmapPMRIN,
			      PVRSRV_BRIDGE_OUT_DEVMEMINTUNMAPPMR *
			      psDevmemIntUnmapPMROUT,
			      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hMappingInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTUNMAPPMR);

	/* Look up the address from the handle */
	psDevmemIntUnmapPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hMappingInt2,
			       psDevmemIntUnmapPMRIN->hMapping,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_MAPPING);
	if (psDevmemIntUnmapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntUnmapPMR_exit;
	}

	psDevmemIntUnmapPMROUT->eError =
	    DevmemIntUnmapPMRResManProxy(hMappingInt2);
	/* Exit early if bridged call fails */
	if (psDevmemIntUnmapPMROUT->eError != PVRSRV_OK) {
		goto DevmemIntUnmapPMR_exit;
	}

	psDevmemIntUnmapPMROUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDevmemIntUnmapPMRIN->hMapping,
				PVRSRV_HANDLE_TYPE_DEVMEMINT_MAPPING);

 DevmemIntUnmapPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntReserveRange(IMG_UINT32 ui32BridgeID,
				  PVRSRV_BRIDGE_IN_DEVMEMINTRESERVERANGE *
				  psDevmemIntReserveRangeIN,
				  PVRSRV_BRIDGE_OUT_DEVMEMINTRESERVERANGE *
				  psDevmemIntReserveRangeOUT,
				  CONNECTION_DATA * psConnection)
{
	DEVMEMINT_HEAP *psDevmemServerHeapInt;
	IMG_HANDLE hDevmemServerHeapInt2;
	DEVMEMINT_RESERVATION *psReservationInt;
	IMG_HANDLE hReservationInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTRESERVERANGE);

	NEW_HANDLE_BATCH_OR_ERROR(psDevmemIntReserveRangeOUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psDevmemIntReserveRangeOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemServerHeapInt2,
			       psDevmemIntReserveRangeIN->hDevmemServerHeap,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP);
	if (psDevmemIntReserveRangeOUT->eError != PVRSRV_OK) {
		goto DevmemIntReserveRange_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntReserveRangeOUT->eError =
	    ResManFindPrivateDataByPtr(hDevmemServerHeapInt2,
				       (IMG_VOID **) & psDevmemServerHeapInt,
				       IMG_NULL);
	if (psDevmemIntReserveRangeOUT->eError != PVRSRV_OK) {
		goto DevmemIntReserveRange_exit;
	}

	psDevmemIntReserveRangeOUT->eError =
	    DevmemIntReserveRange(psDevmemServerHeapInt,
				  psDevmemIntReserveRangeIN->sAddress,
				  psDevmemIntReserveRangeIN->uiLength,
				  &psReservationInt);
	/* Exit early if bridged call fails */
	if (psDevmemIntReserveRangeOUT->eError != PVRSRV_OK) {
		goto DevmemIntReserveRange_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hReservationInt2 = ResManRegisterRes(psConnection->hResManContext,
					     RESMAN_TYPE_DEVICEMEM2_RESERVATION,
					     psReservationInt, 0,
					     /* FIXME: how can we avoid this cast? */
					     (RESMAN_FREE_FN) &
					     DevmemIntUnreserveRange);
	if (hReservationInt2 == IMG_NULL) {
		psDevmemIntReserveRangeOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DevmemIntReserveRange_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDevmemIntReserveRangeOUT->hReservation,
			    (IMG_HANDLE) hReservationInt2,
			    PVRSRV_HANDLE_TYPE_DEVMEMINT_RESERVATION,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDevmemIntReserveRangeOUT->eError,
				     psConnection);

 DevmemIntReserveRange_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntUnreserveRange(IMG_UINT32 ui32BridgeID,
				    PVRSRV_BRIDGE_IN_DEVMEMINTUNRESERVERANGE *
				    psDevmemIntUnreserveRangeIN,
				    PVRSRV_BRIDGE_OUT_DEVMEMINTUNRESERVERANGE *
				    psDevmemIntUnreserveRangeOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hReservatioInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMINTUNRESERVERANGE);

	/* Look up the address from the handle */
	psDevmemIntUnreserveRangeOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hReservatioInt2,
			       psDevmemIntUnreserveRangeIN->hReservatio,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_RESERVATION);
	if (psDevmemIntUnreserveRangeOUT->eError != PVRSRV_OK) {
		goto DevmemIntUnreserveRange_exit;
	}

	psDevmemIntUnreserveRangeOUT->eError =
	    DevmemIntUnreserveRangeResManProxy(hReservatioInt2);
	/* Exit early if bridged call fails */
	if (psDevmemIntUnreserveRangeOUT->eError != PVRSRV_OK) {
		goto DevmemIntUnreserveRange_exit;
	}

	psDevmemIntUnreserveRangeOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDevmemIntUnreserveRangeIN->
				hReservatio,
				PVRSRV_HANDLE_TYPE_DEVMEMINT_RESERVATION);

 DevmemIntUnreserveRange_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePhysmemNewRamBackedPMR(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_PHYSMEMNEWRAMBACKEDPMR *
				   psPhysmemNewRamBackedPMRIN,
				   PVRSRV_BRIDGE_OUT_PHYSMEMNEWRAMBACKEDPMR *
				   psPhysmemNewRamBackedPMROUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	PMR *psPMRPtrInt;
	IMG_HANDLE hPMRPtrInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_PHYSMEMNEWRAMBACKEDPMR);

	NEW_HANDLE_BATCH_OR_ERROR(psPhysmemNewRamBackedPMROUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psPhysmemNewRamBackedPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psPhysmemNewRamBackedPMRIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psPhysmemNewRamBackedPMROUT->eError != PVRSRV_OK) {
		goto PhysmemNewRamBackedPMR_exit;
	}

	psPhysmemNewRamBackedPMROUT->eError =
	    PhysmemNewRamBackedPMR(hDeviceNodeInt,
				   psPhysmemNewRamBackedPMRIN->uiSize,
				   psPhysmemNewRamBackedPMRIN->ui32Log2PageSize,
				   psPhysmemNewRamBackedPMRIN->uiFlags,
				   &psPMRPtrInt);
	/* Exit early if bridged call fails */
	if (psPhysmemNewRamBackedPMROUT->eError != PVRSRV_OK) {
		goto PhysmemNewRamBackedPMR_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPMRPtrInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_PMR, psPMRPtrInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) & PMRUnrefPMR);
	if (hPMRPtrInt2 == IMG_NULL) {
		psPhysmemNewRamBackedPMROUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PhysmemNewRamBackedPMR_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psPhysmemNewRamBackedPMROUT->hPMRPtr,
			    (IMG_HANDLE) hPMRPtrInt2,
			    PVRSRV_HANDLE_TYPE_PHYSMEM_PMR,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psPhysmemNewRamBackedPMROUT->eError,
				     psConnection);

 PhysmemNewRamBackedPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRUnrefPMR(IMG_UINT32 ui32BridgeID,
			PVRSRV_BRIDGE_IN_PMRUNREFPMR * psPMRUnrefPMRIN,
			PVRSRV_BRIDGE_OUT_PMRUNREFPMR * psPMRUnrefPMROUT,
			CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_MM_PMRUNREFPMR);

	/* Look up the address from the handle */
	psPMRUnrefPMROUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRUnrefPMRIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRUnrefPMROUT->eError != PVRSRV_OK) {
		goto PMRUnrefPMR_exit;
	}

	psPMRUnrefPMROUT->eError = PMRUnrefPMRResManProxy(hPMRInt2);
	/* Exit early if bridged call fails */
	if (psPMRUnrefPMROUT->eError != PVRSRV_OK) {
		goto PMRUnrefPMR_exit;
	}

	psPMRUnrefPMROUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psPMRUnrefPMRIN->hPMR,
				PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);

 PMRUnrefPMR_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemSLCFlushInvalRequest(IMG_UINT32 ui32BridgeID,
				       PVRSRV_BRIDGE_IN_DEVMEMSLCFLUSHINVALREQUEST
				       * psDevmemSLCFlushInvalRequestIN,
				       PVRSRV_BRIDGE_OUT_DEVMEMSLCFLUSHINVALREQUEST
				       * psDevmemSLCFlushInvalRequestOUT,
				       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	PMR *psPmrInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_DEVMEMSLCFLUSHINVALREQUEST);

	/* Look up the address from the handle */
	psDevmemSLCFlushInvalRequestOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psDevmemSLCFlushInvalRequestIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psDevmemSLCFlushInvalRequestOUT->eError != PVRSRV_OK) {
		goto DevmemSLCFlushInvalRequest_exit;
	}
	/* Look up the address from the handle */
	psDevmemSLCFlushInvalRequestOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & psPmrInt,
			       psDevmemSLCFlushInvalRequestIN->hPmr,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psDevmemSLCFlushInvalRequestOUT->eError != PVRSRV_OK) {
		goto DevmemSLCFlushInvalRequest_exit;
	}

	psDevmemSLCFlushInvalRequestOUT->eError =
	    DevmemSLCFlushInvalRequest(hDeviceNodeInt, psPmrInt);

 DevmemSLCFlushInvalRequest_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeHeapCfgHeapConfigCount(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGCOUNT *
				   psHeapCfgHeapConfigCountIN,
				   PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGCOUNT *
				   psHeapCfgHeapConfigCountOUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGCOUNT);

	/* Look up the address from the handle */
	psHeapCfgHeapConfigCountOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psHeapCfgHeapConfigCountIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHeapCfgHeapConfigCountOUT->eError != PVRSRV_OK) {
		goto HeapCfgHeapConfigCount_exit;
	}

	psHeapCfgHeapConfigCountOUT->eError =
	    HeapCfgHeapConfigCount(hDeviceNodeInt,
				   &psHeapCfgHeapConfigCountOUT->
				   ui32NumHeapConfigs);

 HeapCfgHeapConfigCount_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeHeapCfgHeapCount(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_HEAPCFGHEAPCOUNT *
			     psHeapCfgHeapCountIN,
			     PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCOUNT *
			     psHeapCfgHeapCountOUT,
			     CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_HEAPCFGHEAPCOUNT);

	/* Look up the address from the handle */
	psHeapCfgHeapCountOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psHeapCfgHeapCountIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHeapCfgHeapCountOUT->eError != PVRSRV_OK) {
		goto HeapCfgHeapCount_exit;
	}

	psHeapCfgHeapCountOUT->eError =
	    HeapCfgHeapCount(hDeviceNodeInt,
			     psHeapCfgHeapCountIN->ui32HeapConfigIndex,
			     &psHeapCfgHeapCountOUT->ui32NumHeaps);

 HeapCfgHeapCount_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeHeapCfgHeapConfigName(IMG_UINT32 ui32BridgeID,
				  PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGNAME *
				  psHeapCfgHeapConfigNameIN,
				  PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGNAME *
				  psHeapCfgHeapConfigNameOUT,
				  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	IMG_CHAR *puiHeapConfigNameInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGNAME);

	puiHeapConfigNameInt =
	    kmalloc(psHeapCfgHeapConfigNameIN->ui32HeapConfigNameBufSz *
		    sizeof(IMG_CHAR), GFP_KERNEL);
	if (!puiHeapConfigNameInt) {
		psHeapCfgHeapConfigNameOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto HeapCfgHeapConfigName_exit;
	}

	/* Look up the address from the handle */
	psHeapCfgHeapConfigNameOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psHeapCfgHeapConfigNameIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHeapCfgHeapConfigNameOUT->eError != PVRSRV_OK) {
		goto HeapCfgHeapConfigName_exit;
	}

	psHeapCfgHeapConfigNameOUT->eError =
	    HeapCfgHeapConfigName(hDeviceNodeInt,
				  psHeapCfgHeapConfigNameIN->
				  ui32HeapConfigIndex,
				  psHeapCfgHeapConfigNameIN->
				  ui32HeapConfigNameBufSz,
				  puiHeapConfigNameInt);

	if (copy_to_user
	    (psHeapCfgHeapConfigNameOUT->puiHeapConfigName,
	     puiHeapConfigNameInt,
	     (psHeapCfgHeapConfigNameIN->ui32HeapConfigNameBufSz *
	      sizeof(IMG_CHAR))) != 0) {
		psHeapCfgHeapConfigNameOUT->eError =
		    PVRSRV_ERROR_INVALID_PARAMS;

		goto HeapCfgHeapConfigName_exit;
	}

 HeapCfgHeapConfigName_exit:
	if (puiHeapConfigNameInt)
		kfree(puiHeapConfigNameInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeHeapCfgHeapDetails(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_HEAPCFGHEAPDETAILS *
			       psHeapCfgHeapDetailsIN,
			       PVRSRV_BRIDGE_OUT_HEAPCFGHEAPDETAILS *
			       psHeapCfgHeapDetailsOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	IMG_CHAR *puiHeapNameOutInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_MM_HEAPCFGHEAPDETAILS);

	puiHeapNameOutInt =
	    kmalloc(psHeapCfgHeapDetailsIN->ui32HeapNameBufSz *
		    sizeof(IMG_CHAR), GFP_KERNEL);
	if (!puiHeapNameOutInt) {
		psHeapCfgHeapDetailsOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto HeapCfgHeapDetails_exit;
	}

	/* Look up the address from the handle */
	psHeapCfgHeapDetailsOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psHeapCfgHeapDetailsIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHeapCfgHeapDetailsOUT->eError != PVRSRV_OK) {
		goto HeapCfgHeapDetails_exit;
	}

	psHeapCfgHeapDetailsOUT->eError =
	    HeapCfgHeapDetails(hDeviceNodeInt,
			       psHeapCfgHeapDetailsIN->ui32HeapConfigIndex,
			       psHeapCfgHeapDetailsIN->ui32HeapIndex,
			       psHeapCfgHeapDetailsIN->ui32HeapNameBufSz,
			       puiHeapNameOutInt,
			       &psHeapCfgHeapDetailsOUT->sDevVAddrBase,
			       &psHeapCfgHeapDetailsOUT->uiHeapLength,
			       &psHeapCfgHeapDetailsOUT->
			       ui32Log2DataPageSizeOut);

	if (copy_to_user
	    (psHeapCfgHeapDetailsOUT->puiHeapNameOut, puiHeapNameOutInt,
	     (psHeapCfgHeapDetailsIN->ui32HeapNameBufSz * sizeof(IMG_CHAR))) !=
	    0) {
		psHeapCfgHeapDetailsOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto HeapCfgHeapDetails_exit;
	}

 HeapCfgHeapDetails_exit:
	if (puiHeapNameOutInt)
		kfree(puiHeapNameOutInt);

	return 0;
}

PVRSRV_ERROR RegisterMMFunctions(IMG_VOID);
IMG_VOID UnregisterMMFunctions(IMG_VOID);

/*
 * Register all MM functions with services
 */
PVRSRV_ERROR RegisterMMFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMREXPORTPMR,
			      PVRSRVBridgePMRExportPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMRUNEXPORTPMR,
			      PVRSRVBridgePMRUnexportPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMRGETUID,
			      PVRSRVBridgePMRGetUID);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMRMAKESERVEREXPORTCLIENTEXPORT,
			      PVRSRVBridgePMRMakeServerExportClientExport);
	SetDispatchTableEntry
	    (PVRSRV_BRIDGE_MM_PMRUNMAKESERVEREXPORTCLIENTEXPORT,
	     PVRSRVBridgePMRUnmakeServerExportClientExport);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMRIMPORTPMR,
			      PVRSRVBridgePMRImportPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTCTXCREATE,
			      PVRSRVBridgeDevmemIntCtxCreate);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTCTXDESTROY,
			      PVRSRVBridgeDevmemIntCtxDestroy);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTHEAPCREATE,
			      PVRSRVBridgeDevmemIntHeapCreate);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTHEAPDESTROY,
			      PVRSRVBridgeDevmemIntHeapDestroy);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTMAPPMR,
			      PVRSRVBridgeDevmemIntMapPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTUNMAPPMR,
			      PVRSRVBridgeDevmemIntUnmapPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTRESERVERANGE,
			      PVRSRVBridgeDevmemIntReserveRange);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMINTUNRESERVERANGE,
			      PVRSRVBridgeDevmemIntUnreserveRange);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PHYSMEMNEWRAMBACKEDPMR,
			      PVRSRVBridgePhysmemNewRamBackedPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_PMRUNREFPMR,
			      PVRSRVBridgePMRUnrefPMR);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_DEVMEMSLCFLUSHINVALREQUEST,
			      PVRSRVBridgeDevmemSLCFlushInvalRequest);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGCOUNT,
			      PVRSRVBridgeHeapCfgHeapConfigCount);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_HEAPCFGHEAPCOUNT,
			      PVRSRVBridgeHeapCfgHeapCount);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGNAME,
			      PVRSRVBridgeHeapCfgHeapConfigName);
	SetDispatchTableEntry(PVRSRV_BRIDGE_MM_HEAPCFGHEAPDETAILS,
			      PVRSRVBridgeHeapCfgHeapDetails);

	return PVRSRV_OK;
}

/*
 * Unregister all mm functions with services
 */
IMG_VOID UnregisterMMFunctions(IMG_VOID)
{
}
