									    /*************************************************************************//*!
									       @Title          Direct client bridge header for mm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef CLIENT_MM_BRIDGE_H
#define CLIENT_MM_BRIDGE_H

#include "common_mm_bridge.h"

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRExportPMR(IMG_HANDLE hBridge,
							  IMG_HANDLE hPMR,
							  IMG_HANDLE *
							  phPMRExport,
							  IMG_UINT64 *
							  pui32Size,
							  IMG_UINT32 *
							  pui32Log2Contig,
							  IMG_UINT64 *
							  pui32Password);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRUnexportPMR(IMG_HANDLE hBridge,
							    IMG_HANDLE
							    hPMRExport);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRGetUID(IMG_HANDLE hBridge,
						       IMG_HANDLE hPMR,
						       IMG_UINT64 * pui32UID);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgePMRMakeServerExportClientExport(IMG_HANDLE hBridge,
				      DEVMEM_SERVER_EXPORTCOOKIE
				      hPMRServerExport,
				      IMG_HANDLE * phPMRExportOut,
				      IMG_UINT64 * pui32Size,
				      IMG_UINT32 * pui32Log2Contig,
				      IMG_UINT64 * pui32Password);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgePMRUnmakeServerExportClientExport(IMG_HANDLE hBridge,
					IMG_HANDLE hPMRExport);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRImportPMR(IMG_HANDLE hBridge,
							  IMG_HANDLE hPMRExport,
							  IMG_UINT64
							  ui32uiPassword,
							  IMG_UINT64 ui32uiSize,
							  IMG_UINT32
							  ui32uiLog2Contig,
							  IMG_HANDLE * phPMR);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntCtxCreate(IMG_HANDLE
								hBridge,
								IMG_HANDLE
								hDeviceNode,
								IMG_HANDLE *
								phDevMemServerContext,
								IMG_HANDLE *
								phPrivData);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntCtxDestroy(IMG_HANDLE
								 hBridge,
								 IMG_HANDLE
								 hDevmemServerContext);

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
								 phDevmemHeapPtr);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntHeapDestroy(IMG_HANDLE
								  hBridge,
								  IMG_HANDLE
								  hDevmemHeap);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntMapPMR(IMG_HANDLE hBridge,
							     IMG_HANDLE
							     hDevmemServerHeap,
							     IMG_HANDLE
							     hReservation,
							     IMG_HANDLE hPMR,
							     PVRSRV_MEMALLOCFLAGS_T
							     uiMapFlags,
							     IMG_HANDLE *
							     phMapping);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntUnmapPMR(IMG_HANDLE
							       hBridge,
							       IMG_HANDLE
							       hMapping);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntReserveRange(IMG_HANDLE
								   hBridge,
								   IMG_HANDLE
								   hDevmemServerHeap,
								   IMG_DEV_VIRTADDR
								   sAddress,
								   IMG_DEVMEM_SIZE_T
								   uiLength,
								   IMG_HANDLE *
								   phReservation);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeDevmemIntUnreserveRange(IMG_HANDLE
								     hBridge,
								     IMG_HANDLE
								     hReservatio);

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
								    phPMRPtr);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRUnrefPMR(IMG_HANDLE hBridge,
							 IMG_HANDLE hPMR);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgeDevmemSLCFlushInvalRequest(IMG_HANDLE hBridge, IMG_HANDLE hDeviceNode,
				 IMG_HANDLE hPmr);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapConfigCount(IMG_HANDLE
								    hBridge,
								    IMG_HANDLE
								    hDeviceNode,
								    IMG_UINT32 *
								    pui32NumHeapConfigs);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapCount(IMG_HANDLE
							      hBridge,
							      IMG_HANDLE
							      hDeviceNode,
							      IMG_UINT32
							      ui32HeapConfigIndex,
							      IMG_UINT32 *
							      pui32NumHeaps);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeHeapCfgHeapConfigName(IMG_HANDLE
								   hBridge,
								   IMG_HANDLE
								   hDeviceNode,
								   IMG_UINT32
								   ui32HeapConfigIndex,
								   IMG_UINT32
								   ui32HeapConfigNameBufSz,
								   IMG_CHAR *
								   puiHeapConfigName);

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
								pui32Log2DataPageSizeOut);

#endif				/* CLIENT_MM_BRIDGE_H */
