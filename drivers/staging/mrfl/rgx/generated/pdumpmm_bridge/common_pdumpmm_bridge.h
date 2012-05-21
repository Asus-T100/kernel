									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for pdumpmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for pdumpmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_PDUMPMM_BRIDGE_H
#define COMMON_PDUMPMM_BRIDGE_H

#include "pdump.h"
#include "pdumpdefs.h"
#include "pvrsrv_memallocflags.h"
#include "devicemem_typedefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST			(PVRSRV_BRIDGE_PDUMPMM_START)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEM			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+0)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPLOADMEMVALUE			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+1)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSAVETOFILE			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+2)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPSYMBOLICADDR			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+3)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPPOL32			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+4)
#define PVRSRV_BRIDGE_PDUMPMM_PMRPDUMPCBP			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+5)
#define PVRSRV_BRIDGE_PDUMPMM_DEVMEMINTPDUMPSAVETOFILEVIRTUAL			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+6)
#define PVRSRV_BRIDGE_PDUMPMM_CMD_LAST			(PVRSRV_BRIDGE_PDUMPMM_CMD_FIRST+6)

/*******************************************
            PMRPDumpLoadMem          
 *******************************************/

/* Bridge in structure for PMRPDumpLoadMem */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEM_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEM;

/* Bridge out structure for PMRPDumpLoadMem */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEM_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEM;

/*******************************************
            PMRPDumpLoadMemValue          
 *******************************************/

/* Bridge in structure for PMRPDumpLoadMemValue */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEMVALUE_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_UINT32 ui32Value;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_PMRPDUMPLOADMEMVALUE;

/* Bridge out structure for PMRPDumpLoadMemValue */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEMVALUE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPLOADMEMVALUE;

/*******************************************
            PMRPDumpSaveToFile          
 *******************************************/

/* Bridge in structure for PMRPDumpSaveToFile */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPSAVETOFILE_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_UINT32 ui32ArraySize;
	const IMG_CHAR *puiFileName;
} PVRSRV_BRIDGE_IN_PMRPDUMPSAVETOFILE;

/* Bridge out structure for PMRPDumpSaveToFile */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPSAVETOFILE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPSAVETOFILE;

/*******************************************
            PMRPDumpSymbolicAddr          
 *******************************************/

/* Bridge in structure for PMRPDumpSymbolicAddr */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPSYMBOLICADDR_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_UINT32 ui32MemspaceNameLen;
	IMG_UINT32 ui32SymbolicAddrLen;
} PVRSRV_BRIDGE_IN_PMRPDUMPSYMBOLICADDR;

/* Bridge out structure for PMRPDumpSymbolicAddr */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPSYMBOLICADDR_TAG {
	IMG_CHAR *puiMemspaceName;
	IMG_CHAR *puiSymbolicAddr;
	IMG_DEVMEM_OFFSET_T uiNewOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPSYMBOLICADDR;

/*******************************************
            PMRPDumpPol32          
 *******************************************/

/* Bridge in structure for PMRPDumpPol32 */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPPOL32_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_UINT32 ui32Value;
	IMG_UINT32 ui32Mask;
	PDUMP_POLL_OPERATOR eOperator;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_PMRPDUMPPOL32;

/* Bridge out structure for PMRPDumpPol32 */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPPOL32_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPPOL32;

/*******************************************
            PMRPDumpCBP          
 *******************************************/

/* Bridge in structure for PMRPDumpCBP */
typedef struct PVRSRV_BRIDGE_IN_PMRPDUMPCBP_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_OFFSET_T uiReadOffset;
	IMG_DEVMEM_OFFSET_T uiWriteOffset;
	IMG_DEVMEM_SIZE_T uiPacketSize;
	IMG_DEVMEM_SIZE_T uiBufferSize;
} PVRSRV_BRIDGE_IN_PMRPDUMPCBP;

/* Bridge out structure for PMRPDumpCBP */
typedef struct PVRSRV_BRIDGE_OUT_PMRPDUMPCBP_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRPDUMPCBP;

/*******************************************
            DevmemIntPDumpSaveToFileVirtual          
 *******************************************/

/* Bridge in structure for DevmemIntPDumpSaveToFileVirtual */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTPDUMPSAVETOFILEVIRTUAL_TAG {
	IMG_HANDLE hDevmemServerContext;
	IMG_DEV_VIRTADDR sAddress;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_UINT32 ui32ArraySize;
	const IMG_CHAR *puiFileName;
	IMG_UINT32 ui32FileOffset;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_DEVMEMINTPDUMPSAVETOFILEVIRTUAL;

/* Bridge out structure for DevmemIntPDumpSaveToFileVirtual */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTPDUMPSAVETOFILEVIRTUAL_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTPDUMPSAVETOFILEVIRTUAL;

#endif				/* COMMON_PDUMPMM_BRIDGE_H */
