									    /*************************************************************************//*!
									       @File
									       @Title          Client bridge header for pdumpmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Exports the client bridge functions for pdumpmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef CLIENT_PDUMPMM_BRIDGE_H
#define CLIENT_PDUMPMM_BRIDGE_H

#include "pvr_bridge_client.h"

#include "common_pdumpmm_bridge.h"

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpLoadMem(IMG_HANDLE hBridge,
							     IMG_HANDLE hPMR,
							     IMG_DEVMEM_OFFSET_T
							     uiOffset,
							     IMG_DEVMEM_SIZE_T
							     uiSize,
							     IMG_UINT32
							     ui32PDumpFlags);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpLoadMemValue(IMG_HANDLE
								  hBridge,
								  IMG_HANDLE
								  hPMR,
								  IMG_DEVMEM_OFFSET_T
								  uiOffset,
								  IMG_UINT32
								  ui32Value,
								  IMG_UINT32
								  ui32PDumpFlags);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpSaveToFile(IMG_HANDLE
								hBridge,
								IMG_HANDLE hPMR,
								IMG_DEVMEM_OFFSET_T
								uiOffset,
								IMG_DEVMEM_SIZE_T
								uiSize,
								IMG_UINT32
								ui32ArraySize,
								const IMG_CHAR *
								puiFileName);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpSymbolicAddr(IMG_HANDLE
								  hBridge,
								  IMG_HANDLE
								  hPMR,
								  IMG_DEVMEM_OFFSET_T
								  uiOffset,
								  IMG_UINT32
								  ui32MemspaceNameLen,
								  IMG_CHAR *
								  puiMemspaceName,
								  IMG_UINT32
								  ui32SymbolicAddrLen,
								  IMG_CHAR *
								  puiSymbolicAddr,
								  IMG_DEVMEM_OFFSET_T
								  *
								  puiNewOffset,
								  IMG_DEVMEM_OFFSET_T
								  *
								  puiNextSymName);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpPol32(IMG_HANDLE hBridge,
							   IMG_HANDLE hPMR,
							   IMG_DEVMEM_OFFSET_T
							   uiOffset,
							   IMG_UINT32 ui32Value,
							   IMG_UINT32 ui32Mask,
							   PDUMP_POLL_OPERATOR
							   eOperator,
							   IMG_UINT32
							   ui32PDumpFlags);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgePMRPDumpCBP(IMG_HANDLE hBridge,
							 IMG_HANDLE hPMR,
							 IMG_DEVMEM_OFFSET_T
							 uiReadOffset,
							 IMG_DEVMEM_OFFSET_T
							 uiWriteOffset,
							 IMG_DEVMEM_SIZE_T
							 uiPacketSize,
							 IMG_DEVMEM_SIZE_T
							 uiBufferSize);

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV
BridgeDevmemIntPDumpSaveToFileVirtual(IMG_HANDLE hBridge,
				      IMG_HANDLE hDevmemServerContext,
				      IMG_DEV_VIRTADDR sAddress,
				      IMG_DEVMEM_SIZE_T uiSize,
				      IMG_UINT32 ui32ArraySize,
				      const IMG_CHAR * puiFileName,
				      IMG_UINT32 ui32FileOffset,
				      IMG_UINT32 ui32PDumpFlags);

#endif				/* CLIENT_PDUMPMM_BRIDGE_H */
