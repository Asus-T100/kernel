									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for cmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for cmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_CMM_BRIDGE_H
#define COMMON_CMM_BRIDGE_H

#include "devicemem_typedefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_CMM_CMD_FIRST			(PVRSRV_BRIDGE_CMM_START)
#define PVRSRV_BRIDGE_CMM_PMRLOCALIMPORTPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_CMM_CMD_FIRST+0)
#define PVRSRV_BRIDGE_CMM_PMRWRITEPMPAGELIST			PVRSRV_IOWR(PVRSRV_BRIDGE_CMM_CMD_FIRST+1)
#define PVRSRV_BRIDGE_CMM_PMRUNWRITEPMPAGELIST			PVRSRV_IOWR(PVRSRV_BRIDGE_CMM_CMD_FIRST+2)
#define PVRSRV_BRIDGE_CMM_CMD_LAST			(PVRSRV_BRIDGE_CMM_CMD_FIRST+2)

/*******************************************
            PMRLocalImportPMR          
 *******************************************/

/* Bridge in structure for PMRLocalImportPMR */
typedef struct PVRSRV_BRIDGE_IN_PMRLOCALIMPORTPMR_TAG {
	IMG_HANDLE hExtHandle;
} PVRSRV_BRIDGE_IN_PMRLOCALIMPORTPMR;

/* Bridge out structure for PMRLocalImportPMR */
typedef struct PVRSRV_BRIDGE_OUT_PMRLOCALIMPORTPMR_TAG {
	IMG_HANDLE hPMR;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_DEVMEM_ALIGN_T sAlign;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRLOCALIMPORTPMR;

/*******************************************
            PMRWritePMPageList          
 *******************************************/

/* Bridge in structure for PMRWritePMPageList */
typedef struct PVRSRV_BRIDGE_IN_PMRWRITEPMPAGELIST_TAG {
	IMG_HANDLE hPageListPMR;
	IMG_DEVMEM_OFFSET_T uiTableOffset;
	IMG_DEVMEM_SIZE_T uiTableLength;
	IMG_HANDLE hReferencePMR;
	IMG_UINT32 ui32Log2PageSize;
} PVRSRV_BRIDGE_IN_PMRWRITEPMPAGELIST;

/* Bridge out structure for PMRWritePMPageList */
typedef struct PVRSRV_BRIDGE_OUT_PMRWRITEPMPAGELIST_TAG {
	IMG_HANDLE hPageList;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRWRITEPMPAGELIST;

/*******************************************
            PMRUnwritePMPageList          
 *******************************************/

/* Bridge in structure for PMRUnwritePMPageList */
typedef struct PVRSRV_BRIDGE_IN_PMRUNWRITEPMPAGELIST_TAG {
	IMG_HANDLE hPageList;
} PVRSRV_BRIDGE_IN_PMRUNWRITEPMPAGELIST;

/* Bridge out structure for PMRUnwritePMPageList */
typedef struct PVRSRV_BRIDGE_OUT_PMRUNWRITEPMPAGELIST_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRUNWRITEPMPAGELIST;

#endif				/* COMMON_CMM_BRIDGE_H */
