									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for pdumpmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for pdumpmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "devicemem_server.h"
#include "pmr.h"
#include "physmem.h"

#include "common_pdumpmm_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgePMRPDumpLoadMem(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEM *
			    psPMRPDumpLoadMemIN,
			    PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEM *
			    psPMRPDumpLoadMemOUT,
			    CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEM);

	/* Look up the address from the handle */
	psPMRPDumpLoadMemOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpLoadMemIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpLoadMemOUT->eError != PVRSRV_OK) {
		goto PMRPDumpLoadMem_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpLoadMemOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpLoadMemOUT->eError != PVRSRV_OK) {
		goto PMRPDumpLoadMem_exit;
	}

	psPMRPDumpLoadMemOUT->eError =
	    PMRPDumpLoadMem(psPMRInt,
			    psPMRPDumpLoadMemIN->uiOffset,
			    psPMRPDumpLoadMemIN->uiSize,
			    psPMRPDumpLoadMemIN->ui32PDumpFlags);

 PMRPDumpLoadMem_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRPDumpLoadMemValue(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEMVALUE *
				 psPMRPDumpLoadMemValueIN,
				 PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEMVALUE *
				 psPMRPDumpLoadMemValueOUT,
				 CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEMVALUE);

	/* Look up the address from the handle */
	psPMRPDumpLoadMemValueOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpLoadMemValueIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpLoadMemValueOUT->eError != PVRSRV_OK) {
		goto PMRPDumpLoadMemValue_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpLoadMemValueOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpLoadMemValueOUT->eError != PVRSRV_OK) {
		goto PMRPDumpLoadMemValue_exit;
	}

	psPMRPDumpLoadMemValueOUT->eError =
	    PMRPDumpLoadMemValue(psPMRInt,
				 psPMRPDumpLoadMemValueIN->uiOffset,
				 psPMRPDumpLoadMemValueIN->ui32Value,
				 psPMRPDumpLoadMemValueIN->ui32PDumpFlags);

 PMRPDumpLoadMemValue_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRPDumpSaveToFile(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_PMRPDUMPSAVETOFILE *
			       psPMRPDumpSaveToFileIN,
			       PVRSRV_BRIDGE_OUT_PMRPDUMPSAVETOFILE *
			       psPMRPDumpSaveToFileOUT,
			       CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;
	IMG_CHAR *uiFileNameInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSAVETOFILE);

	uiFileNameInt =
	    kmalloc(psPMRPDumpSaveToFileIN->ui32ArraySize * sizeof(IMG_CHAR),
		    GFP_KERNEL);
	if (!uiFileNameInt) {
		psPMRPDumpSaveToFileOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto PMRPDumpSaveToFile_exit;
	}

	if (copy_from_user(uiFileNameInt, psPMRPDumpSaveToFileIN->puiFileName,
			   psPMRPDumpSaveToFileIN->ui32ArraySize *
			   sizeof(IMG_CHAR)) != 0) {
		psPMRPDumpSaveToFileOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto PMRPDumpSaveToFile_exit;
	}

	/* Look up the address from the handle */
	psPMRPDumpSaveToFileOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpSaveToFileIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpSaveToFileOUT->eError != PVRSRV_OK) {
		goto PMRPDumpSaveToFile_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpSaveToFileOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpSaveToFileOUT->eError != PVRSRV_OK) {
		goto PMRPDumpSaveToFile_exit;
	}

	psPMRPDumpSaveToFileOUT->eError =
	    PMRPDumpSaveToFile(psPMRInt,
			       psPMRPDumpSaveToFileIN->uiOffset,
			       psPMRPDumpSaveToFileIN->uiSize,
			       psPMRPDumpSaveToFileIN->ui32ArraySize,
			       uiFileNameInt);

 PMRPDumpSaveToFile_exit:
	if (uiFileNameInt)
		kfree(uiFileNameInt);

	return 0;
}

static IMG_INT
PVRSRVBridgePMRPDumpSymbolicAddr(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_PMRPDUMPSYMBOLICADDR *
				 psPMRPDumpSymbolicAddrIN,
				 PVRSRV_BRIDGE_OUT_PMRPDUMPSYMBOLICADDR *
				 psPMRPDumpSymbolicAddrOUT,
				 CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;
	IMG_CHAR *puiMemspaceNameInt = IMG_NULL;
	IMG_CHAR *puiSymbolicAddrInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSYMBOLICADDR);

	puiMemspaceNameInt =
	    kmalloc(psPMRPDumpSymbolicAddrIN->ui32MemspaceNameLen *
		    sizeof(IMG_CHAR), GFP_KERNEL);
	if (!puiMemspaceNameInt) {
		psPMRPDumpSymbolicAddrOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto PMRPDumpSymbolicAddr_exit;
	}

	puiSymbolicAddrInt =
	    kmalloc(psPMRPDumpSymbolicAddrIN->ui32SymbolicAddrLen *
		    sizeof(IMG_CHAR), GFP_KERNEL);
	if (!puiSymbolicAddrInt) {
		psPMRPDumpSymbolicAddrOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto PMRPDumpSymbolicAddr_exit;
	}

	/* Look up the address from the handle */
	psPMRPDumpSymbolicAddrOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpSymbolicAddrIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpSymbolicAddrOUT->eError != PVRSRV_OK) {
		goto PMRPDumpSymbolicAddr_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpSymbolicAddrOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpSymbolicAddrOUT->eError != PVRSRV_OK) {
		goto PMRPDumpSymbolicAddr_exit;
	}

	psPMRPDumpSymbolicAddrOUT->eError =
	    PMR_PDumpSymbolicAddr(psPMRInt,
				  psPMRPDumpSymbolicAddrIN->uiOffset,
				  psPMRPDumpSymbolicAddrIN->ui32MemspaceNameLen,
				  puiMemspaceNameInt,
				  psPMRPDumpSymbolicAddrIN->ui32SymbolicAddrLen,
				  puiSymbolicAddrInt,
				  &psPMRPDumpSymbolicAddrOUT->uiNewOffset,
				  &psPMRPDumpSymbolicAddrOUT->uiNextSymName);

	if (copy_to_user
	    (psPMRPDumpSymbolicAddrOUT->puiMemspaceName, puiMemspaceNameInt,
	     (psPMRPDumpSymbolicAddrIN->ui32MemspaceNameLen *
	      sizeof(IMG_CHAR))) != 0) {
		psPMRPDumpSymbolicAddrOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto PMRPDumpSymbolicAddr_exit;
	}

	if (copy_to_user
	    (psPMRPDumpSymbolicAddrOUT->puiSymbolicAddr, puiSymbolicAddrInt,
	     (psPMRPDumpSymbolicAddrIN->ui32SymbolicAddrLen *
	      sizeof(IMG_CHAR))) != 0) {
		psPMRPDumpSymbolicAddrOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto PMRPDumpSymbolicAddr_exit;
	}

 PMRPDumpSymbolicAddr_exit:
	if (puiMemspaceNameInt)
		kfree(puiMemspaceNameInt);
	if (puiSymbolicAddrInt)
		kfree(puiSymbolicAddrInt);

	return 0;
}

static IMG_INT
PVRSRVBridgePMRPDumpPol32(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_PMRPDUMPPOL32 * psPMRPDumpPol32IN,
			  PVRSRV_BRIDGE_OUT_PMRPDUMPPOL32 * psPMRPDumpPol32OUT,
			  CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPPOL32);

	/* Look up the address from the handle */
	psPMRPDumpPol32OUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpPol32IN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpPol32OUT->eError != PVRSRV_OK) {
		goto PMRPDumpPol32_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpPol32OUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpPol32OUT->eError != PVRSRV_OK) {
		goto PMRPDumpPol32_exit;
	}

	psPMRPDumpPol32OUT->eError =
	    PMRPDumpPol32(psPMRInt,
			  psPMRPDumpPol32IN->uiOffset,
			  psPMRPDumpPol32IN->ui32Value,
			  psPMRPDumpPol32IN->ui32Mask,
			  psPMRPDumpPol32IN->eOperator,
			  psPMRPDumpPol32IN->ui32PDumpFlags);

 PMRPDumpPol32_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRPDumpCBP(IMG_UINT32 ui32BridgeID,
			PVRSRV_BRIDGE_IN_PMRPDUMPCBP * psPMRPDumpCBPIN,
			PVRSRV_BRIDGE_OUT_PMRPDUMPCBP * psPMRPDumpCBPOUT,
			CONNECTION_DATA * psConnection)
{
	PMR *psPMRInt;
	IMG_HANDLE hPMRInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPCBP);

	/* Look up the address from the handle */
	psPMRPDumpCBPOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPMRInt2,
			       psPMRPDumpCBPIN->hPMR,
			       PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (psPMRPDumpCBPOUT->eError != PVRSRV_OK) {
		goto PMRPDumpCBP_exit;
	}

	/* Look up the data from the resman address */
	psPMRPDumpCBPOUT->eError =
	    ResManFindPrivateDataByPtr(hPMRInt2, (IMG_VOID **) & psPMRInt,
				       IMG_NULL);
	if (psPMRPDumpCBPOUT->eError != PVRSRV_OK) {
		goto PMRPDumpCBP_exit;
	}

	psPMRPDumpCBPOUT->eError =
	    PMRPDumpCBP(psPMRInt,
			psPMRPDumpCBPIN->uiReadOffset,
			psPMRPDumpCBPIN->uiWriteOffset,
			psPMRPDumpCBPIN->uiPacketSize,
			psPMRPDumpCBPIN->uiBufferSize);

 PMRPDumpCBP_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDevmemIntPDumpSaveToFileVirtual(IMG_UINT32 ui32BridgeID,
					    PVRSRV_BRIDGE_IN_DEVMEMINTPDUMPSAVETOFILEVIRTUAL
					    *
					    psDevmemIntPDumpSaveToFileVirtualIN,
					    PVRSRV_BRIDGE_OUT_DEVMEMINTPDUMPSAVETOFILEVIRTUAL
					    *
					    psDevmemIntPDumpSaveToFileVirtualOUT,
					    CONNECTION_DATA * psConnection)
{
	DEVMEMINT_CTX *psDevmemServerContextInt;
	IMG_HANDLE hDevmemServerContextInt2;
	IMG_CHAR *uiFileNameInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPMM_DEVMEMINTPDUMPSAVETOFILEVIRTUAL);

	uiFileNameInt =
	    kmalloc(psDevmemIntPDumpSaveToFileVirtualIN->ui32ArraySize *
		    sizeof(IMG_CHAR), GFP_KERNEL);
	if (!uiFileNameInt) {
		psDevmemIntPDumpSaveToFileVirtualOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DevmemIntPDumpSaveToFileVirtual_exit;
	}

	if (copy_from_user
	    (uiFileNameInt, psDevmemIntPDumpSaveToFileVirtualIN->puiFileName,
	     psDevmemIntPDumpSaveToFileVirtualIN->ui32ArraySize *
	     sizeof(IMG_CHAR)) != 0) {
		psDevmemIntPDumpSaveToFileVirtualOUT->eError =
		    PVRSRV_ERROR_INVALID_PARAMS;

		goto DevmemIntPDumpSaveToFileVirtual_exit;
	}

	/* Look up the address from the handle */
	psDevmemIntPDumpSaveToFileVirtualOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemServerContextInt2,
			       psDevmemIntPDumpSaveToFileVirtualIN->
			       hDevmemServerContext,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX);
	if (psDevmemIntPDumpSaveToFileVirtualOUT->eError != PVRSRV_OK) {
		goto DevmemIntPDumpSaveToFileVirtual_exit;
	}

	/* Look up the data from the resman address */
	psDevmemIntPDumpSaveToFileVirtualOUT->eError =
	    ResManFindPrivateDataByPtr(hDevmemServerContextInt2,
				       (IMG_VOID **) & psDevmemServerContextInt,
				       IMG_NULL);
	if (psDevmemIntPDumpSaveToFileVirtualOUT->eError != PVRSRV_OK) {
		goto DevmemIntPDumpSaveToFileVirtual_exit;
	}

	psDevmemIntPDumpSaveToFileVirtualOUT->eError =
	    DevmemIntPDumpSaveToFileVirtual(psDevmemServerContextInt,
					    psDevmemIntPDumpSaveToFileVirtualIN->
					    sAddress,
					    psDevmemIntPDumpSaveToFileVirtualIN->
					    uiSize,
					    psDevmemIntPDumpSaveToFileVirtualIN->
					    ui32ArraySize, uiFileNameInt,
					    psDevmemIntPDumpSaveToFileVirtualIN->
					    ui32FileOffset,
					    psDevmemIntPDumpSaveToFileVirtualIN->
					    ui32PDumpFlags);

 DevmemIntPDumpSaveToFileVirtual_exit:
	if (uiFileNameInt)
		kfree(uiFileNameInt);

	return 0;
}

PVRSRV_ERROR RegisterPDUMPMMFunctions(IMG_VOID);
IMG_VOID UnregisterPDUMPMMFunctions(IMG_VOID);

/*
 * Register all PDUMPMM functions with services
 */
PVRSRV_ERROR RegisterPDUMPMMFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEM,
			      PVRSRVBridgePMRPDumpLoadMem);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEMVALUE,
			      PVRSRVBridgePMRPDumpLoadMemValue);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSAVETOFILE,
			      PVRSRVBridgePMRPDumpSaveToFile);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSYMBOLICADDR,
			      PVRSRVBridgePMRPDumpSymbolicAddr);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPPOL32,
			      PVRSRVBridgePMRPDumpPol32);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPCBP,
			      PVRSRVBridgePMRPDumpCBP);
	SetDispatchTableEntry
	    (PVRSRV_BRIDGE_PDUMPMM_DEVMEMINTPDUMPSAVETOFILEVIRTUAL,
	     PVRSRVBridgeDevmemIntPDumpSaveToFileVirtual);

	return PVRSRV_OK;
}

/*
 * Unregister all pdumpmm functions with services
 */
IMG_VOID UnregisterPDUMPMMFunctions(IMG_VOID)
{
}
