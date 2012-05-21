									     /**************************************************************************//*!
									        @File           devicemem_pdump.c
									        @Title          Shared device memory management PDump functions
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Implements common (client & server) PDump functions for the
									        memory management code
    *//***************************************************************************/

#include "allocmem.h"
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pdump.h"
#include "devicemem_utils.h"
#include "devicemem_pdump.h"
#include "client_pdumpmm_bridge.h"

IMG_INTERNAL IMG_VOID
DevmemPDumpLoadMem(DEVMEM_MEMDESC * psMemDesc,
		   IMG_DEVMEM_OFFSET_T uiOffset,
		   IMG_DEVMEM_SIZE_T uiSize, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	eError = BridgePMRPDumpLoadMem(psMemDesc->psImport->hBridge,
				       psMemDesc->psImport->hPMR,
				       psMemDesc->uiOffset + uiOffset,
				       uiSize, uiPDumpFlags);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL IMG_VOID
DevmemPDumpLoadMemValue(DEVMEM_MEMDESC * psMemDesc,
			IMG_DEVMEM_OFFSET_T uiOffset,
			IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	eError = BridgePMRPDumpLoadMemValue(psMemDesc->psImport->hBridge,
					    psMemDesc->psImport->hPMR,
					    psMemDesc->uiOffset + uiOffset,
					    ui32Value, uiPDumpFlags);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

/* FIXME: This should be server side only */
IMG_INTERNAL PVRSRV_ERROR
DevmemPDumpPageCatBaseToSAddr(DEVMEM_MEMDESC * psMemDesc,
			      IMG_DEVMEM_OFFSET_T * puiMemOffset,
			      IMG_CHAR * pszName, IMG_UINT32 ui32Size)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[100];
	IMG_CHAR aszSymbolicName[100];
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	*puiMemOffset += psMemDesc->uiOffset;

	eError = BridgePMRPDumpSymbolicAddr(psMemDesc->psImport->hBridge,
					    psMemDesc->psImport->hPMR,
					    *puiMemOffset,
					    sizeof(aszMemspaceName),
					    &aszMemspaceName[0],
					    sizeof(aszSymbolicName),
					    &aszSymbolicName[0],
					    puiMemOffset, &uiNextSymName);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);

	OSSNPrintf(pszName, ui32Size, "%s:%s", &aszMemspaceName[0],
		   &aszSymbolicName[0]);
	return eError;
}

IMG_INTERNAL IMG_VOID
DevmemPDumpSaveToFile(DEVMEM_MEMDESC * psMemDesc,
		      IMG_DEVMEM_OFFSET_T uiOffset,
		      IMG_DEVMEM_SIZE_T uiSize, const IMG_CHAR * pszFilename)
{
	PVRSRV_ERROR eError;

	eError = BridgePMRPDumpSaveToFile(psMemDesc->psImport->hBridge,
					  psMemDesc->psImport->hPMR,
					  psMemDesc->uiOffset + uiOffset,
					  uiSize,
					  OSStringLength(pszFilename) + 1,
					  pszFilename);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

/* FIXME: Remove? */
IMG_INTERNAL IMG_VOID
DevmemPDumpSaveToFileVirtual(DEVMEM_MEMDESC * psMemDesc,
			     IMG_DEVMEM_OFFSET_T uiOffset,
			     IMG_DEVMEM_SIZE_T uiSize,
			     const IMG_CHAR * pszFilename,
			     IMG_UINT32 ui32FileOffset,
			     IMG_UINT32 ui32PdumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_DEV_VIRTADDR sDevAddrStart;

	sDevAddrStart = psMemDesc->psImport->sDeviceImport.sDevVAddr;
	sDevAddrStart.uiAddr += psMemDesc->uiOffset;
	sDevAddrStart.uiAddr += uiOffset;

	eError =
	    BridgeDevmemIntPDumpSaveToFileVirtual(psMemDesc->psImport->hBridge,
						  psMemDesc->psImport->
						  sDeviceImport.psHeap->psCtx->
						  hDevMemServerContext,
						  sDevAddrStart, uiSize,
						  OSStringLength(pszFilename) +
						  1, pszFilename,
						  ui32FileOffset,
						  ui32PdumpFlags);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL PVRSRV_ERROR
DevmemPDumpDevmemPol32(const DEVMEM_MEMDESC * psMemDesc,
		       IMG_DEVMEM_OFFSET_T uiOffset,
		       IMG_UINT32 ui32Value,
		       IMG_UINT32 ui32Mask,
		       PDUMP_POLL_OPERATOR eOperator,
		       PDUMP_FLAGS_T ui32PDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_DEVMEM_SIZE_T uiNumBytes;

	uiNumBytes = 4;

	if (psMemDesc->uiOffset + uiOffset + uiNumBytes >=
	    psMemDesc->psImport->uiSize) {
		eError = PVRSRV_ERROR_DEVICEMEM_OUT_OF_RANGE;
		goto e0;
	}

	eError = BridgePMRPDumpPol32(psMemDesc->psImport->hBridge,
				     psMemDesc->psImport->hPMR,
				     psMemDesc->uiOffset + uiOffset,
				     ui32Value,
				     ui32Mask, eOperator, ui32PDumpFlags);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemPDumpCBP(const DEVMEM_MEMDESC * psMemDesc,
	       IMG_DEVMEM_OFFSET_T uiReadOffset,
	       IMG_DEVMEM_OFFSET_T uiWriteOffset,
	       IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize)
{
	PVRSRV_ERROR eError;

	if ((psMemDesc->uiOffset + uiReadOffset) > psMemDesc->psImport->uiSize) {
		eError = PVRSRV_ERROR_DEVICEMEM_OUT_OF_RANGE;
		goto e0;
	}

	eError = BridgePMRPDumpCBP(psMemDesc->psImport->hBridge,
				   psMemDesc->psImport->hPMR,
				   psMemDesc->uiOffset + uiReadOffset,
				   uiWriteOffset, uiPacketSize, uiBufferSize);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}
