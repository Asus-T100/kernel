									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for dc
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for dc
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_DC_BRIDGE_H
#define COMMON_DC_BRIDGE_H

#include "pvrsrv_surface.h"
#include "dc_external.h"
#include "dc_common.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_DC_CMD_FIRST			(PVRSRV_BRIDGE_DC_START)
#define PVRSRV_BRIDGE_DC_DCDEVICESQUERYCOUNT			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+0)
#define PVRSRV_BRIDGE_DC_DCDEVICESENUMERATE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+1)
#define PVRSRV_BRIDGE_DC_DCDEVICEACQUIRE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+2)
#define PVRSRV_BRIDGE_DC_DCDEVICERELEASE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+3)
#define PVRSRV_BRIDGE_DC_DCGETINFO			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+4)
#define PVRSRV_BRIDGE_DC_DCPANELQUERYCOUNT			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+5)
#define PVRSRV_BRIDGE_DC_DCPANELQUERY			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+6)
#define PVRSRV_BRIDGE_DC_DCFORMATQUERY			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+7)
#define PVRSRV_BRIDGE_DC_DCDIMQUERY			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+8)
#define PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERACQUIRE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+9)
#define PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERRELEASE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+10)
#define PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCREATE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+11)
#define PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCONFIGURE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+12)
#define PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTDESTROY			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+13)
#define PVRSRV_BRIDGE_DC_DCBUFFERALLOC			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+14)
#define PVRSRV_BRIDGE_DC_DCBUFFERIMPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+15)
#define PVRSRV_BRIDGE_DC_DCBUFFERFREE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+16)
#define PVRSRV_BRIDGE_DC_DCBUFFERUNIMPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+17)
#define PVRSRV_BRIDGE_DC_DCBUFFERPIN			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+18)
#define PVRSRV_BRIDGE_DC_DCBUFFERUNPIN			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+19)
#define PVRSRV_BRIDGE_DC_DCBUFFERACQUIRE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+20)
#define PVRSRV_BRIDGE_DC_DCBUFFERRELEASE			PVRSRV_IOWR(PVRSRV_BRIDGE_DC_CMD_FIRST+21)
#define PVRSRV_BRIDGE_DC_CMD_LAST			(PVRSRV_BRIDGE_DC_CMD_FIRST+21)

/*******************************************
            DCDevicesQueryCount          
 *******************************************/

/* Bridge in structure for DCDevicesQueryCount */
typedef struct PVRSRV_BRIDGE_IN_DCDEVICESQUERYCOUNT_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_DCDEVICESQUERYCOUNT;

/* Bridge out structure for DCDevicesQueryCount */
typedef struct PVRSRV_BRIDGE_OUT_DCDEVICESQUERYCOUNT_TAG {
	IMG_UINT32 ui32DeviceCount;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDEVICESQUERYCOUNT;

/*******************************************
            DCDevicesEnumerate          
 *******************************************/

/* Bridge in structure for DCDevicesEnumerate */
typedef struct PVRSRV_BRIDGE_IN_DCDEVICESENUMERATE_TAG {
	IMG_UINT32 ui32DeviceArraySize;
} PVRSRV_BRIDGE_IN_DCDEVICESENUMERATE;

/* Bridge out structure for DCDevicesEnumerate */
typedef struct PVRSRV_BRIDGE_OUT_DCDEVICESENUMERATE_TAG {
	IMG_UINT32 ui32DeviceCount;
	IMG_UINT32 *pui32DeviceIndex;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDEVICESENUMERATE;

/*******************************************
            DCDeviceAcquire          
 *******************************************/

/* Bridge in structure for DCDeviceAcquire */
typedef struct PVRSRV_BRIDGE_IN_DCDEVICEACQUIRE_TAG {
	IMG_UINT32 ui32DeviceIndex;
} PVRSRV_BRIDGE_IN_DCDEVICEACQUIRE;

/* Bridge out structure for DCDeviceAcquire */
typedef struct PVRSRV_BRIDGE_OUT_DCDEVICEACQUIRE_TAG {
	IMG_HANDLE hDevice;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDEVICEACQUIRE;

/*******************************************
            DCDeviceRelease          
 *******************************************/

/* Bridge in structure for DCDeviceRelease */
typedef struct PVRSRV_BRIDGE_IN_DCDEVICERELEASE_TAG {
	IMG_HANDLE hDevice;
} PVRSRV_BRIDGE_IN_DCDEVICERELEASE;

/* Bridge out structure for DCDeviceRelease */
typedef struct PVRSRV_BRIDGE_OUT_DCDEVICERELEASE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDEVICERELEASE;

/*******************************************
            DCGetInfo          
 *******************************************/

/* Bridge in structure for DCGetInfo */
typedef struct PVRSRV_BRIDGE_IN_DCGETINFO_TAG {
	IMG_HANDLE hDevice;
} PVRSRV_BRIDGE_IN_DCGETINFO;

/* Bridge out structure for DCGetInfo */
typedef struct PVRSRV_BRIDGE_OUT_DCGETINFO_TAG {
	DC_DISPLAY_INFO sDisplayInfo;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCGETINFO;

/*******************************************
            DCPanelQueryCount          
 *******************************************/

/* Bridge in structure for DCPanelQueryCount */
typedef struct PVRSRV_BRIDGE_IN_DCPANELQUERYCOUNT_TAG {
	IMG_HANDLE hDevice;
} PVRSRV_BRIDGE_IN_DCPANELQUERYCOUNT;

/* Bridge out structure for DCPanelQueryCount */
typedef struct PVRSRV_BRIDGE_OUT_DCPANELQUERYCOUNT_TAG {
	IMG_UINT32 ui32NumPanels;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCPANELQUERYCOUNT;

/*******************************************
            DCPanelQuery          
 *******************************************/

/* Bridge in structure for DCPanelQuery */
typedef struct PVRSRV_BRIDGE_IN_DCPANELQUERY_TAG {
	IMG_HANDLE hDevice;
	IMG_UINT32 ui32PanelsArraySize;
} PVRSRV_BRIDGE_IN_DCPANELQUERY;

/* Bridge out structure for DCPanelQuery */
typedef struct PVRSRV_BRIDGE_OUT_DCPANELQUERY_TAG {
	IMG_UINT32 ui32NumPanels;
	PVRSRV_SURFACE_INFO *psSurfInfo;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCPANELQUERY;

/*******************************************
            DCFormatQuery          
 *******************************************/

/* Bridge in structure for DCFormatQuery */
typedef struct PVRSRV_BRIDGE_IN_DCFORMATQUERY_TAG {
	IMG_HANDLE hDevice;
	IMG_UINT32 ui32NumFormats;
	PVRSRV_SURFACE_FORMAT *psFormat;
} PVRSRV_BRIDGE_IN_DCFORMATQUERY;

/* Bridge out structure for DCFormatQuery */
typedef struct PVRSRV_BRIDGE_OUT_DCFORMATQUERY_TAG {
	IMG_UINT32 *pui32Supported;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCFORMATQUERY;

/*******************************************
            DCDimQuery          
 *******************************************/

/* Bridge in structure for DCDimQuery */
typedef struct PVRSRV_BRIDGE_IN_DCDIMQUERY_TAG {
	IMG_HANDLE hDevice;
	IMG_UINT32 ui32NumDims;
	PVRSRV_SURFACE_DIMS *psDim;
} PVRSRV_BRIDGE_IN_DCDIMQUERY;

/* Bridge out structure for DCDimQuery */
typedef struct PVRSRV_BRIDGE_OUT_DCDIMQUERY_TAG {
	IMG_UINT32 *pui32Supported;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDIMQUERY;

/*******************************************
            DCSystemBufferAcquire          
 *******************************************/

/* Bridge in structure for DCSystemBufferAcquire */
typedef struct PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERACQUIRE_TAG {
	IMG_HANDLE hDevice;
} PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERACQUIRE;

/* Bridge out structure for DCSystemBufferAcquire */
typedef struct PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERACQUIRE_TAG {
	IMG_UINT32 ui32Stride;
	IMG_HANDLE hBuffer;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERACQUIRE;

/*******************************************
            DCSystemBufferRelease          
 *******************************************/

/* Bridge in structure for DCSystemBufferRelease */
typedef struct PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERRELEASE_TAG {
	IMG_HANDLE hBuffer;
} PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERRELEASE;

/* Bridge out structure for DCSystemBufferRelease */
typedef struct PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERRELEASE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERRELEASE;

/*******************************************
            DCDisplayContextCreate          
 *******************************************/

/* Bridge in structure for DCDisplayContextCreate */
typedef struct PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCREATE_TAG {
	IMG_HANDLE hDevice;
} PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCREATE;

/* Bridge out structure for DCDisplayContextCreate */
typedef struct PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCREATE_TAG {
	IMG_HANDLE hDisplayContext;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCREATE;

/*******************************************
            DCDisplayContextConfigure          
 *******************************************/

/* Bridge in structure for DCDisplayContextConfigure */
typedef struct PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCONFIGURE_TAG {
	IMG_HANDLE hDisplayContext;
	IMG_UINT32 ui32PipeCount;
	PVRSRV_SURFACE_CONFIG_INFO *psSurfInfo;
	IMG_HANDLE *phBuffers;
	IMG_UINT32 ui32SyncCount;
	IMG_HANDLE *phSync;
	IMG_UINT32 ui32DisplayPeriod;
} PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCONFIGURE;

/* Bridge out structure for DCDisplayContextConfigure */
typedef struct PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCONFIGURE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCONFIGURE;

/*******************************************
            DCDisplayContextDestroy          
 *******************************************/

/* Bridge in structure for DCDisplayContextDestroy */
typedef struct PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTDESTROY_TAG {
	IMG_HANDLE hDisplayContext;
} PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTDESTROY;

/* Bridge out structure for DCDisplayContextDestroy */
typedef struct PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTDESTROY_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTDESTROY;

/*******************************************
            DCBufferAlloc          
 *******************************************/

/* Bridge in structure for DCBufferAlloc */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERALLOC_TAG {
	IMG_HANDLE hDisplayContext;
	DC_BUFFER_CREATE_INFO sSurfInfo;
} PVRSRV_BRIDGE_IN_DCBUFFERALLOC;

/* Bridge out structure for DCBufferAlloc */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERALLOC_TAG {
	IMG_UINT32 ui32Stride;
	IMG_HANDLE hBuffer;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERALLOC;

/*******************************************
            DCBufferImport          
 *******************************************/

/* Bridge in structure for DCBufferImport */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERIMPORT_TAG {
	IMG_HANDLE hDisplayContext;
	IMG_UINT32 ui32NumPlanes;
	IMG_HANDLE *phImport;
	DC_BUFFER_IMPORT_INFO sSurfAttrib;
} PVRSRV_BRIDGE_IN_DCBUFFERIMPORT;

/* Bridge out structure for DCBufferImport */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERIMPORT_TAG {
	IMG_HANDLE hBuffer;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERIMPORT;

/*******************************************
            DCBufferFree          
 *******************************************/

/* Bridge in structure for DCBufferFree */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERFREE_TAG {
	IMG_HANDLE hBuffer;
} PVRSRV_BRIDGE_IN_DCBUFFERFREE;

/* Bridge out structure for DCBufferFree */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERFREE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERFREE;

/*******************************************
            DCBufferUnimport          
 *******************************************/

/* Bridge in structure for DCBufferUnimport */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERUNIMPORT_TAG {
	IMG_HANDLE hBuffer;
} PVRSRV_BRIDGE_IN_DCBUFFERUNIMPORT;

/* Bridge out structure for DCBufferUnimport */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERUNIMPORT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERUNIMPORT;

/*******************************************
            DCBufferPin          
 *******************************************/

/* Bridge in structure for DCBufferPin */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERPIN_TAG {
	IMG_HANDLE hBuffer;
} PVRSRV_BRIDGE_IN_DCBUFFERPIN;

/* Bridge out structure for DCBufferPin */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERPIN_TAG {
	IMG_HANDLE hPinHandle;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERPIN;

/*******************************************
            DCBufferUnpin          
 *******************************************/

/* Bridge in structure for DCBufferUnpin */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERUNPIN_TAG {
	IMG_HANDLE hPinHandle;
} PVRSRV_BRIDGE_IN_DCBUFFERUNPIN;

/* Bridge out structure for DCBufferUnpin */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERUNPIN_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERUNPIN;

/*******************************************
            DCBufferAcquire          
 *******************************************/

/* Bridge in structure for DCBufferAcquire */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERACQUIRE_TAG {
	IMG_HANDLE hBuffer;
} PVRSRV_BRIDGE_IN_DCBUFFERACQUIRE;

/* Bridge out structure for DCBufferAcquire */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERACQUIRE_TAG {
	IMG_HANDLE hExtMem;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERACQUIRE;

/*******************************************
            DCBufferRelease          
 *******************************************/

/* Bridge in structure for DCBufferRelease */
typedef struct PVRSRV_BRIDGE_IN_DCBUFFERRELEASE_TAG {
	IMG_HANDLE hExtMem;
} PVRSRV_BRIDGE_IN_DCBUFFERRELEASE;

/* Bridge out structure for DCBufferRelease */
typedef struct PVRSRV_BRIDGE_OUT_DCBUFFERRELEASE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DCBUFFERRELEASE;

#endif				/* COMMON_DC_BRIDGE_H */
