									    /*************************************************************************//*!
									       @Title          Direct client bridge for mm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "client_mm_bridge.h"
#include "img_defs.h"
#include "pvr_debug.h"

/* Module specific includes */
#include "pvrsrv_memallocflags.h"
#include "devicemem_typedefs.h"

#include "devicemem_server.h"
#include "pmr.h"
#include "devicemem_heapcfg.h"
#include "physmem.h"

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRExportPMR(IMG_HANDLE hBridge,
							  IMG_HANDLE hPMR,
							  IMG_HANDLE *
							  phPMRExport,
							  IMG_UINT64 *
							  pui32Size,
							  IMG_UINT32 *
							  pui32Log2Contig,
							  IMG_UINT64 *
							  pui32Password)
{
	PVRSRV_ERROR eError;
	PMR *psPMRInt;
	PMR_EXPORT *psPMRExportInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRInt = (PMR *) hPMR;

	eError =
	    PMRExportPMR(psPMRInt,
			 &psPMRExportInt,
			 pui32Size, pui32Log2Contig, pui32Password);

	*phPMRExport = psPMRExportInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRUnexportPMR(IMG_HANDLE hBridge,
							    IMG_HANDLE
							    hPMRExport)
{
	PVRSRV_ERROR eError;
	PMR_EXPORT *psPMRExportInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRExportInt = (PMR_EXPORT *) hPMRExport;

	eError = PMRUnexportPMR(psPMRExportInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRGetUID(IMG_HANDLE hBridge,
						       IMG_HANDLE hPMR,
						       IMG_UINT64 * pui32UID)
{
	PVRSRV_ERROR eError;
	PMR *psPMRInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRInt = (PMR *) hPMR;

	eError = PMRGetUID(psPMRInt, pui32UID);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgePMRMakeServerExportClientExport(IMG_HANDLE hBridge,
				      DEVMEM_SERVER_EXPORTCOOKIE
				      hPMRServerExport,
				      IMG_HANDLE * phPMRExportOut,
				      IMG_UINT64 * pui32Size,
				      IMG_UINT32 * pui32Log2Contig,
				      IMG_UINT64 * pui32Password)
{
	PVRSRV_ERROR eError;
	DEVMEM_EXPORTCOOKIE *psPMRServerExportInt;
	PMR_EXPORT *psPMRExportOutInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRServerExportInt = (DEVMEM_EXPORTCOOKIE *) hPMRServerExport;

	eError =
	    PMRMakeServerExportClientExport(psPMRServerExportInt,
					    &psPMRExportOutInt,
					    pui32Size,
					    pui32Log2Contig, pui32Password);

	*phPMRExportOut = psPMRExportOutInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgePMRUnmakeServerExportClientExport(IMG_HANDLE hBridge,
					IMG_HANDLE hPMRExport)
{
	PVRSRV_ERROR eError;
	PMR_EXPORT *psPMRExportInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRExportInt = (PMR_EXPORT *) hPMRExport;

	eError = PMRUnmakeServerExportClientExport(psPMRExportInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRImportPMR(IMG_HANDLE hBridge,
							  IMG_HANDLE hPMRExport,
							  IMG_UINT64
							  ui32uiPassword,
							  IMG_UINT64 ui32uiSize,
							  IMG_UINT32
							  ui32uiLog2Contig,
							  IMG_HANDLE * phPMR)
{
	PVRSRV_ERROR eError;
	PMR_EXPORT *psPMRExportInt;
	PMR *psPMRInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRExportInt = (PMR_EXPORT *) hPMRExport;

	eError =
	    PMRImportPMR(psPMRExportInt,
			 ui32uiPassword,
			 ui32uiSize, ui32uiLog2Contig, &psPMRInt);

	*phPMR = psPMRInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntCtxCreate(IMG_HANDLE
								hBridge,
								IMG_HANDLE
								hDeviceNode,
								IMG_HANDLE *
								phDevMemServerContext,
								IMG_HANDLE *
								phPrivData)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;
	DEVMEMINT_CTX *psDevMemServerContextInt;
	IMG_HANDLE hPrivDataInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError =
	    DevmemIntCtxCreate(hDeviceNodeInt,
			       &psDevMemServerContextInt, &hPrivDataInt);

	*phDevMemServerContext = psDevMemServerContextInt;
	*phPrivData = hPrivDataInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntCtxDestroy(IMG_HANDLE
								 hBridge,
								 IMG_HANDLE
								 hDevmemServerContext)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_CTX *psDevmemServerContextInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psDevmemServerContextInt = (DEVMEMINT_CTX *) hDevmemServerContext;

	eError = DevmemIntCtxDestroy(psDevmemServerContextInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntHeapCreate(IMG_HANDLE
								 hBridge,
								 IMG_HANDLE
								 hDevmemCtx,
								 IMG_DEV_VIRTADDR
								 sHeapBaseAddr,
								 IMG_DEVMEM_SIZE_T
								 uiHeapLength,
								 IMG_UINT32
								 ui32Log2DataPageSize,
								 IMG_HANDLE *
								 phDevmemHeapPtr)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_CTX *psDevmemCtxInt;
	DEVMEMINT_HEAP *psDevmemHeapPtrInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psDevmemCtxInt = (DEVMEMINT_CTX *) hDevmemCtx;

	eError =
	    DevmemIntHeapCreate(psDevmemCtxInt,
				sHeapBaseAddr,
				uiHeapLength,
				ui32Log2DataPageSize, &psDevmemHeapPtrInt);

	*phDevmemHeapPtr = psDevmemHeapPtrInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntHeapDestroy(IMG_HANDLE
								  hBridge,
								  IMG_HANDLE
								  hDevmemHeap)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_HEAP *psDevmemHeapInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psDevmemHeapInt = (DEVMEMINT_HEAP *) hDevmemHeap;

	eError = DevmemIntHeapDestroy(psDevmemHeapInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntMapPMR(IMG_HANDLE hBridge,
							     IMG_HANDLE
							     hDevmemServerHeap,
							     IMG_HANDLE
							     hReservation,
							     IMG_HANDLE hPMR,
							     PVRSRV_MEMALLOCFLAGS_T
							     uiMapFlags,
							     IMG_HANDLE *
							     phMapping)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_HEAP *psDevmemServerHeapInt;
	DEVMEMINT_RESERVATION *psReservationInt;
	PMR *psPMRInt;
	DEVMEMINT_MAPPING *psMappingInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psDevmemServerHeapInt = (DEVMEMINT_HEAP *) hDevmemServerHeap;
	psReservationInt = (DEVMEMINT_RESERVATION *) hReservation;
	psPMRInt = (PMR *) hPMR;

	eError =
	    DevmemIntMapPMR(psDevmemServerHeapInt,
			    psReservationInt,
			    psPMRInt, uiMapFlags, &psMappingInt);

	*phMapping = psMappingInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntUnmapPMR(IMG_HANDLE
							       hBridge,
							       IMG_HANDLE
							       hMapping)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_MAPPING *psMappingInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psMappingInt = (DEVMEMINT_MAPPING *) hMapping;

	eError = DevmemIntUnmapPMR(psMappingInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntReserveRange(IMG_HANDLE
								   hBridge,
								   IMG_HANDLE
								   hDevmemServerHeap,
								   IMG_DEV_VIRTADDR
								   sAddress,
								   IMG_DEVMEM_SIZE_T
								   uiLength,
								   IMG_HANDLE *
								   phReservation)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_HEAP *psDevmemServerHeapInt;
	DEVMEMINT_RESERVATION *psReservationInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psDevmemServerHeapInt = (DEVMEMINT_HEAP *) hDevmemServerHeap;

	eError =
	    DevmemIntReserveRange(psDevmemServerHeapInt,
				  sAddress, uiLength, &psReservationInt);

	*phReservation = psReservationInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntUnreserveRange(IMG_HANDLE
								     hBridge,
								     IMG_HANDLE
								     hReservatio)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_RESERVATION *psReservatioInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psReservatioInt = (DEVMEMINT_RESERVATION *) hReservatio;

	eError = DevmemIntUnreserveRange(psReservatioInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePhysmemNewRamBackedPMR(IMG_HANDLE
								    hBridge,
								    IMG_HANDLE
								    hDeviceNode,
								    IMG_DEVMEM_SIZE_T
								    uiSize,
								    IMG_UINT32
								    ui32Log2PageSize,
								    PVRSRV_MEMALLOCFLAGS_T
								    uiFlags,
								    IMG_HANDLE *
								    phPMRPtr)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;
	PMR *psPMRPtrInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError =
	    PhysmemNewRamBackedPMR(hDeviceNodeInt,
				   uiSize,
				   ui32Log2PageSize, uiFlags, &psPMRPtrInt);

	*phPMRPtr = psPMRPtrInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRUnrefPMR(IMG_HANDLE hBridge,
							 IMG_HANDLE hPMR)
{
	PVRSRV_ERROR eError;
	PMR *psPMRInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psPMRInt = (PMR *) hPMR;

	eError = PMRUnrefPMR(psPMRInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgeDevmemSLCFlushInvalRequest(IMG_HANDLE hBridge, IMG_HANDLE hDeviceNode,
				 IMG_HANDLE hPmr)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;
	PMR *psPmrInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;
	psPmrInt = (PMR *) hPmr;

	eError = DevmemSLCFlushInvalRequest(hDeviceNodeInt, psPmrInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapConfigCount(IMG_HANDLE
								    hBridge,
								    IMG_HANDLE
								    hDeviceNode,
								    IMG_UINT32 *
								    pui32NumHeapConfigs)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError = HeapCfgHeapConfigCount(hDeviceNodeInt, pui32NumHeapConfigs);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapCount(IMG_HANDLE
							      hBridge,
							      IMG_HANDLE
							      hDeviceNode,
							      IMG_UINT32
							      ui32HeapConfigIndex,
							      IMG_UINT32 *
							      pui32NumHeaps)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError =
	    HeapCfgHeapCount(hDeviceNodeInt,
			     ui32HeapConfigIndex, pui32NumHeaps);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapConfigName(IMG_HANDLE
								   hBridge,
								   IMG_HANDLE
								   hDeviceNode,
								   IMG_UINT32
								   ui32HeapConfigIndex,
								   IMG_UINT32
								   ui32HeapConfigNameBufSz,
								   IMG_CHAR *
								   puiHeapConfigName)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError =
	    HeapCfgHeapConfigName(hDeviceNodeInt,
				  ui32HeapConfigIndex,
				  ui32HeapConfigNameBufSz, puiHeapConfigName);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapDetails(IMG_HANDLE
								hBridge,
								IMG_HANDLE
								hDeviceNode,
								IMG_UINT32
								ui32HeapConfigIndex,
								IMG_UINT32
								ui32HeapIndex,
								IMG_UINT32
								ui32HeapNameBufSz,
								IMG_CHAR *
								puiHeapNameOut,
								IMG_DEV_VIRTADDR
								*
								psDevVAddrBase,
								IMG_DEVMEM_SIZE_T
								* puiHeapLength,
								IMG_UINT32 *
								pui32Log2DataPageSizeOut)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDeviceNodeInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDeviceNodeInt = (IMG_HANDLE) hDeviceNode;

	eError =
	    HeapCfgHeapDetails(hDeviceNodeInt,
			       ui32HeapConfigIndex,
			       ui32HeapIndex,
			       ui32HeapNameBufSz,
			       puiHeapNameOut,
			       psDevVAddrBase,
			       puiHeapLength, pui32Log2DataPageSizeOut);

	return eError;
}
