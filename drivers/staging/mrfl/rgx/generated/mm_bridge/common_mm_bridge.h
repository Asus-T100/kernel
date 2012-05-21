									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for mm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for mm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_MM_BRIDGE_H
#define COMMON_MM_BRIDGE_H

#include "pvrsrv_memallocflags.h"
#include "devicemem_typedefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_MM_CMD_FIRST			(PVRSRV_BRIDGE_MM_START)
#define PVRSRV_BRIDGE_MM_PMREXPORTPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+0)
#define PVRSRV_BRIDGE_MM_PMRUNEXPORTPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+1)
#define PVRSRV_BRIDGE_MM_PMRGETUID			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+2)
#define PVRSRV_BRIDGE_MM_PMRMAKESERVEREXPORTCLIENTEXPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+3)
#define PVRSRV_BRIDGE_MM_PMRUNMAKESERVEREXPORTCLIENTEXPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+4)
#define PVRSRV_BRIDGE_MM_PMRIMPORTPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+5)
#define PVRSRV_BRIDGE_MM_DEVMEMINTCTXCREATE			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+6)
#define PVRSRV_BRIDGE_MM_DEVMEMINTCTXDESTROY			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+7)
#define PVRSRV_BRIDGE_MM_DEVMEMINTHEAPCREATE			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+8)
#define PVRSRV_BRIDGE_MM_DEVMEMINTHEAPDESTROY			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+9)
#define PVRSRV_BRIDGE_MM_DEVMEMINTMAPPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+10)
#define PVRSRV_BRIDGE_MM_DEVMEMINTUNMAPPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+11)
#define PVRSRV_BRIDGE_MM_DEVMEMINTRESERVERANGE			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+12)
#define PVRSRV_BRIDGE_MM_DEVMEMINTUNRESERVERANGE			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+13)
#define PVRSRV_BRIDGE_MM_PHYSMEMNEWRAMBACKEDPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+14)
#define PVRSRV_BRIDGE_MM_PMRUNREFPMR			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+15)
#define PVRSRV_BRIDGE_MM_DEVMEMSLCFLUSHINVALREQUEST			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+16)
#define PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGCOUNT			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+17)
#define PVRSRV_BRIDGE_MM_HEAPCFGHEAPCOUNT			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+18)
#define PVRSRV_BRIDGE_MM_HEAPCFGHEAPCONFIGNAME			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+19)
#define PVRSRV_BRIDGE_MM_HEAPCFGHEAPDETAILS			PVRSRV_IOWR(PVRSRV_BRIDGE_MM_CMD_FIRST+20)
#define PVRSRV_BRIDGE_MM_CMD_LAST			(PVRSRV_BRIDGE_MM_CMD_FIRST+20)

/*******************************************
            PMRExportPMR          
 *******************************************/

/* Bridge in structure for PMRExportPMR */
typedef struct PVRSRV_BRIDGE_IN_PMREXPORTPMR_TAG {
	IMG_HANDLE hPMR;
} PVRSRV_BRIDGE_IN_PMREXPORTPMR;

/* Bridge out structure for PMRExportPMR */
typedef struct PVRSRV_BRIDGE_OUT_PMREXPORTPMR_TAG {
	IMG_HANDLE hPMRExport;
	IMG_UINT64 ui32Size;
	IMG_UINT32 ui32Log2Contig;
	IMG_UINT64 ui32Password;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMREXPORTPMR;

/*******************************************
            PMRUnexportPMR          
 *******************************************/

/* Bridge in structure for PMRUnexportPMR */
typedef struct PVRSRV_BRIDGE_IN_PMRUNEXPORTPMR_TAG {
	IMG_HANDLE hPMRExport;
} PVRSRV_BRIDGE_IN_PMRUNEXPORTPMR;

/* Bridge out structure for PMRUnexportPMR */
typedef struct PVRSRV_BRIDGE_OUT_PMRUNEXPORTPMR_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRUNEXPORTPMR;

/*******************************************
            PMRGetUID          
 *******************************************/

/* Bridge in structure for PMRGetUID */
typedef struct PVRSRV_BRIDGE_IN_PMRGETUID_TAG {
	IMG_HANDLE hPMR;
} PVRSRV_BRIDGE_IN_PMRGETUID;

/* Bridge out structure for PMRGetUID */
typedef struct PVRSRV_BRIDGE_OUT_PMRGETUID_TAG {
	IMG_UINT64 ui32UID;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRGETUID;

/*******************************************
            PMRMakeServerExportClientExport          
 *******************************************/

/* Bridge in structure for PMRMakeServerExportClientExport */
typedef struct PVRSRV_BRIDGE_IN_PMRMAKESERVEREXPORTCLIENTEXPORT_TAG {
	DEVMEM_SERVER_EXPORTCOOKIE hPMRServerExport;
} PVRSRV_BRIDGE_IN_PMRMAKESERVEREXPORTCLIENTEXPORT;

/* Bridge out structure for PMRMakeServerExportClientExport */
typedef struct PVRSRV_BRIDGE_OUT_PMRMAKESERVEREXPORTCLIENTEXPORT_TAG {
	IMG_HANDLE hPMRExportOut;
	IMG_UINT64 ui32Size;
	IMG_UINT32 ui32Log2Contig;
	IMG_UINT64 ui32Password;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRMAKESERVEREXPORTCLIENTEXPORT;

/*******************************************
            PMRUnmakeServerExportClientExport          
 *******************************************/

/* Bridge in structure for PMRUnmakeServerExportClientExport */
typedef struct PVRSRV_BRIDGE_IN_PMRUNMAKESERVEREXPORTCLIENTEXPORT_TAG {
	IMG_HANDLE hPMRExport;
} PVRSRV_BRIDGE_IN_PMRUNMAKESERVEREXPORTCLIENTEXPORT;

/* Bridge out structure for PMRUnmakeServerExportClientExport */
typedef struct PVRSRV_BRIDGE_OUT_PMRUNMAKESERVEREXPORTCLIENTEXPORT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRUNMAKESERVEREXPORTCLIENTEXPORT;

/*******************************************
            PMRImportPMR          
 *******************************************/

/* Bridge in structure for PMRImportPMR */
typedef struct PVRSRV_BRIDGE_IN_PMRIMPORTPMR_TAG {
	IMG_HANDLE hPMRExport;
	IMG_UINT64 ui32uiPassword;
	IMG_UINT64 ui32uiSize;
	IMG_UINT32 ui32uiLog2Contig;
} PVRSRV_BRIDGE_IN_PMRIMPORTPMR;

/* Bridge out structure for PMRImportPMR */
typedef struct PVRSRV_BRIDGE_OUT_PMRIMPORTPMR_TAG {
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRIMPORTPMR;

/*******************************************
            DevmemIntCtxCreate          
 *******************************************/

/* Bridge in structure for DevmemIntCtxCreate */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTCTXCREATE_TAG {
	IMG_HANDLE hDeviceNode;
} PVRSRV_BRIDGE_IN_DEVMEMINTCTXCREATE;

/* Bridge out structure for DevmemIntCtxCreate */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTCTXCREATE_TAG {
	IMG_HANDLE hDevMemServerContext;
	IMG_HANDLE hPrivData;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTCTXCREATE;

/*******************************************
            DevmemIntCtxDestroy          
 *******************************************/

/* Bridge in structure for DevmemIntCtxDestroy */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTCTXDESTROY_TAG {
	IMG_HANDLE hDevmemServerContext;
} PVRSRV_BRIDGE_IN_DEVMEMINTCTXDESTROY;

/* Bridge out structure for DevmemIntCtxDestroy */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTCTXDESTROY_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTCTXDESTROY;

/*******************************************
            DevmemIntHeapCreate          
 *******************************************/

/* Bridge in structure for DevmemIntHeapCreate */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTHEAPCREATE_TAG {
	IMG_HANDLE hDevmemCtx;
	IMG_DEV_VIRTADDR sHeapBaseAddr;
	IMG_DEVMEM_SIZE_T uiHeapLength;
	IMG_UINT32 ui32Log2DataPageSize;
} PVRSRV_BRIDGE_IN_DEVMEMINTHEAPCREATE;

/* Bridge out structure for DevmemIntHeapCreate */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPCREATE_TAG {
	IMG_HANDLE hDevmemHeapPtr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPCREATE;

/*******************************************
            DevmemIntHeapDestroy          
 *******************************************/

/* Bridge in structure for DevmemIntHeapDestroy */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTHEAPDESTROY_TAG {
	IMG_HANDLE hDevmemHeap;
} PVRSRV_BRIDGE_IN_DEVMEMINTHEAPDESTROY;

/* Bridge out structure for DevmemIntHeapDestroy */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPDESTROY_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTHEAPDESTROY;

/*******************************************
            DevmemIntMapPMR          
 *******************************************/

/* Bridge in structure for DevmemIntMapPMR */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTMAPPMR_TAG {
	IMG_HANDLE hDevmemServerHeap;
	IMG_HANDLE hReservation;
	IMG_HANDLE hPMR;
	PVRSRV_MEMALLOCFLAGS_T uiMapFlags;
} PVRSRV_BRIDGE_IN_DEVMEMINTMAPPMR;

/* Bridge out structure for DevmemIntMapPMR */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTMAPPMR_TAG {
	IMG_HANDLE hMapping;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTMAPPMR;

/*******************************************
            DevmemIntUnmapPMR          
 *******************************************/

/* Bridge in structure for DevmemIntUnmapPMR */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTUNMAPPMR_TAG {
	IMG_HANDLE hMapping;
} PVRSRV_BRIDGE_IN_DEVMEMINTUNMAPPMR;

/* Bridge out structure for DevmemIntUnmapPMR */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTUNMAPPMR_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTUNMAPPMR;

/*******************************************
            DevmemIntReserveRange          
 *******************************************/

/* Bridge in structure for DevmemIntReserveRange */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTRESERVERANGE_TAG {
	IMG_HANDLE hDevmemServerHeap;
	IMG_DEV_VIRTADDR sAddress;
	IMG_DEVMEM_SIZE_T uiLength;
} PVRSRV_BRIDGE_IN_DEVMEMINTRESERVERANGE;

/* Bridge out structure for DevmemIntReserveRange */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTRESERVERANGE_TAG {
	IMG_HANDLE hReservation;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTRESERVERANGE;

/*******************************************
            DevmemIntUnreserveRange          
 *******************************************/

/* Bridge in structure for DevmemIntUnreserveRange */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMINTUNRESERVERANGE_TAG {
	IMG_HANDLE hReservatio;
} PVRSRV_BRIDGE_IN_DEVMEMINTUNRESERVERANGE;

/* Bridge out structure for DevmemIntUnreserveRange */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMINTUNRESERVERANGE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMINTUNRESERVERANGE;

/*******************************************
            PhysmemNewRamBackedPMR          
 *******************************************/

/* Bridge in structure for PhysmemNewRamBackedPMR */
typedef struct PVRSRV_BRIDGE_IN_PHYSMEMNEWRAMBACKEDPMR_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_UINT32 ui32Log2PageSize;
	PVRSRV_MEMALLOCFLAGS_T uiFlags;
} PVRSRV_BRIDGE_IN_PHYSMEMNEWRAMBACKEDPMR;

/* Bridge out structure for PhysmemNewRamBackedPMR */
typedef struct PVRSRV_BRIDGE_OUT_PHYSMEMNEWRAMBACKEDPMR_TAG {
	IMG_HANDLE hPMRPtr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PHYSMEMNEWRAMBACKEDPMR;

/*******************************************
            PMRUnrefPMR          
 *******************************************/

/* Bridge in structure for PMRUnrefPMR */
typedef struct PVRSRV_BRIDGE_IN_PMRUNREFPMR_TAG {
	IMG_HANDLE hPMR;
} PVRSRV_BRIDGE_IN_PMRUNREFPMR;

/* Bridge out structure for PMRUnrefPMR */
typedef struct PVRSRV_BRIDGE_OUT_PMRUNREFPMR_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PMRUNREFPMR;

/*******************************************
            DevmemSLCFlushInvalRequest          
 *******************************************/

/* Bridge in structure for DevmemSLCFlushInvalRequest */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMSLCFLUSHINVALREQUEST_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_HANDLE hPmr;
} PVRSRV_BRIDGE_IN_DEVMEMSLCFLUSHINVALREQUEST;

/* Bridge out structure for DevmemSLCFlushInvalRequest */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMSLCFLUSHINVALREQUEST_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMSLCFLUSHINVALREQUEST;

/*******************************************
            HeapCfgHeapConfigCount          
 *******************************************/

/* Bridge in structure for HeapCfgHeapConfigCount */
typedef struct PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGCOUNT_TAG {
	IMG_HANDLE hDeviceNode;
} PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGCOUNT;

/* Bridge out structure for HeapCfgHeapConfigCount */
typedef struct PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGCOUNT_TAG {
	IMG_UINT32 ui32NumHeapConfigs;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGCOUNT;

/*******************************************
            HeapCfgHeapCount          
 *******************************************/

/* Bridge in structure for HeapCfgHeapCount */
typedef struct PVRSRV_BRIDGE_IN_HEAPCFGHEAPCOUNT_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_UINT32 ui32HeapConfigIndex;
} PVRSRV_BRIDGE_IN_HEAPCFGHEAPCOUNT;

/* Bridge out structure for HeapCfgHeapCount */
typedef struct PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCOUNT_TAG {
	IMG_UINT32 ui32NumHeaps;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCOUNT;

/*******************************************
            HeapCfgHeapConfigName          
 *******************************************/

/* Bridge in structure for HeapCfgHeapConfigName */
typedef struct PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGNAME_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_UINT32 ui32HeapConfigIndex;
	IMG_UINT32 ui32HeapConfigNameBufSz;
} PVRSRV_BRIDGE_IN_HEAPCFGHEAPCONFIGNAME;

/* Bridge out structure for HeapCfgHeapConfigName */
typedef struct PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGNAME_TAG {
	IMG_CHAR *puiHeapConfigName;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HEAPCFGHEAPCONFIGNAME;

/*******************************************
            HeapCfgHeapDetails          
 *******************************************/

/* Bridge in structure for HeapCfgHeapDetails */
typedef struct PVRSRV_BRIDGE_IN_HEAPCFGHEAPDETAILS_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_UINT32 ui32HeapConfigIndex;
	IMG_UINT32 ui32HeapIndex;
	IMG_UINT32 ui32HeapNameBufSz;
} PVRSRV_BRIDGE_IN_HEAPCFGHEAPDETAILS;

/* Bridge out structure for HeapCfgHeapDetails */
typedef struct PVRSRV_BRIDGE_OUT_HEAPCFGHEAPDETAILS_TAG {
	IMG_CHAR *puiHeapNameOut;
	IMG_DEV_VIRTADDR sDevVAddrBase;
	IMG_DEVMEM_SIZE_T uiHeapLength;
	IMG_UINT32 ui32Log2DataPageSizeOut;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HEAPCFGHEAPDETAILS;

#endif				/* COMMON_MM_BRIDGE_H */
