									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxinit
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxinit
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXINIT_BRIDGE_H
#define COMMON_RGXINIT_BRIDGE_H

#include "rgx_bridge.h"
#include "rgxscript.h"
#include "rgx_fwif_shared.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXINIT_CMD_FIRST			(PVRSRV_BRIDGE_RGXINIT_START)
#define PVRSRV_BRIDGE_RGXINIT_RGXINITFIRMWARE			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXINIT_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXINIT_RGXINITDEVPART2			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXINIT_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXINIT_CMD_LAST			(PVRSRV_BRIDGE_RGXINIT_CMD_FIRST+1)

/*******************************************
            RGXInitFirmware          
 *******************************************/

/* Bridge in structure for RGXInitFirmware */
typedef struct PVRSRV_BRIDGE_IN_RGXINITFIRMWARE_TAG {
	IMG_HANDLE hDevNode;
	IMG_DEVMEM_SIZE_T uiFWMemAllocSize;
	IMG_BOOL bEnableSignatureChecks;
	IMG_UINT32 ui32SignatureChecksBufSize;
	IMG_UINT32 ui32RGXFWAlignChecksSize;
	IMG_UINT32 *pui32RGXFWAlignChecks;
	IMG_UINT32 ui32ui32ConfigFlags;
} PVRSRV_BRIDGE_IN_RGXINITFIRMWARE;

/* Bridge out structure for RGXInitFirmware */
typedef struct PVRSRV_BRIDGE_OUT_RGXINITFIRMWARE_TAG {
	DEVMEM_SERVER_EXPORTCOOKIE hFWMemAllocServerExportCookie;
	IMG_DEV_VIRTADDR sFWMemDevVAddrBase;
	IMG_UINT64 ui32FWHeapBase;
	RGXFWIF_DEV_VIRTADDR spsRGXFwInit;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXINITFIRMWARE;

/*******************************************
            RGXInitDevPart2          
 *******************************************/

/* Bridge in structure for RGXInitDevPart2 */
typedef struct PVRSRV_BRIDGE_IN_RGXINITDEVPART2_TAG {
	IMG_HANDLE hDevNode;
	RGX_INIT_COMMAND *psInitScript;
	RGX_INIT_COMMAND *psDbgScript;
	RGX_INIT_COMMAND *psDeinitScript;
	IMG_UINT32 ui32KernelCatBase;
} PVRSRV_BRIDGE_IN_RGXINITDEVPART2;

/* Bridge out structure for RGXInitDevPart2 */
typedef struct PVRSRV_BRIDGE_OUT_RGXINITDEVPART2_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXINITDEVPART2;

#endif				/* COMMON_RGXINIT_BRIDGE_H */
