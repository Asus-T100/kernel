									    /*************************************************************************//*!
									       @File                    dc_server.h

									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved

									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _DC_SERVER_H_
#define _DC_SERVER_H_

#include "img_types.h"
#include "pvrsrv_error.h"
#include "sync_external.h"
#include "pvrsrv_surface.h"
#include "pmr.h"
#include "kerneldisplay.h"
#include "sync_server.h"

typedef struct _DC_DEVICE_ DC_DEVICE;
typedef struct _DC_DISPLAY_CONTEXT_ DC_DISPLAY_CONTEXT;
typedef struct _DC_BUFFER_ DC_BUFFER;
typedef DC_BUFFER *DC_PIN_HANDLE;

PVRSRV_ERROR DCDevicesQueryCount(IMG_UINT32 * pui32DeviceCount);

PVRSRV_ERROR DCDevicesEnumerate(IMG_UINT32 ui32DeviceArraySize,
				IMG_UINT32 * pui32DeviceCount,
				IMG_UINT32 * paui32DeviceIndex);

PVRSRV_ERROR DCDeviceAcquire(IMG_UINT32 ui32DeviceIndex,
			     DC_DEVICE ** ppsDevice);

PVRSRV_ERROR DCDeviceRelease(DC_DEVICE * psDevice);

PVRSRV_ERROR DCGetInfo(DC_DEVICE * psDevice, DC_DISPLAY_INFO * psDisplayInfo);

PVRSRV_ERROR DCPanelQueryCount(DC_DEVICE * psDevice,
			       IMG_UINT32 * pui32NumPanels);

PVRSRV_ERROR DCPanelQuery(DC_DEVICE * psDevice,
			  IMG_UINT32 ui32PanelsArraySize,
			  IMG_UINT32 * pui32NumPanels,
			  PVRSRV_SURFACE_INFO * pasSurfInfo);

PVRSRV_ERROR DCFormatQuery(DC_DEVICE * psDevice,
			   IMG_UINT32 ui32FormatArraySize,
			   PVRSRV_SURFACE_FORMAT * pasFormat,
			   IMG_UINT32 * pui32Supported);

PVRSRV_ERROR DCDimQuery(DC_DEVICE * psDevice,
			IMG_UINT32 ui32DimSize,
			PVRSRV_SURFACE_DIMS * pasDim,
			IMG_UINT32 * pui32Supported);

PVRSRV_ERROR DCSystemBufferAcquire(DC_DEVICE * psDevice,
				   IMG_UINT32 * pui32ByteStride,
				   DC_BUFFER ** ppsBuffer);

PVRSRV_ERROR DCSystemBufferRelease(DC_BUFFER * psBuffer);

PVRSRV_ERROR DCDisplayContextCreate(DC_DEVICE * psDevice,
				    DC_DISPLAY_CONTEXT ** ppsDisplayContext);

PVRSRV_ERROR DCDisplayContextConfigure(DC_DISPLAY_CONTEXT * psDisplayContext,
				       IMG_UINT32 ui32PipeCount,
				       PVRSRV_SURFACE_CONFIG_INFO *
				       pasSurfAttrib, DC_BUFFER ** papsBuffers,
				       IMG_UINT32 ui32SyncOpCount,
				       SERVER_SYNC_PRIMITIVE ** papsSync,
				       IMG_UINT32 ui32DisplayPeriod);

PVRSRV_ERROR DCDisplayContextDestroy(DC_DISPLAY_CONTEXT * psDisplayContext);

PVRSRV_ERROR DCBufferAlloc(DC_DISPLAY_CONTEXT * psDisplayContext,
			   DC_BUFFER_CREATE_INFO * psSurfInfo,
			   IMG_UINT32 * pui32ByteStride,
			   DC_BUFFER ** ppsBuffer);

PVRSRV_ERROR DCBufferFree(DC_BUFFER * psBuffer);

PVRSRV_ERROR DCBufferImport(DC_DISPLAY_CONTEXT * psDisplayContext,
			    IMG_UINT32 ui32NumPlanes,
			    PMR ** papsImport,
			    DC_BUFFER_IMPORT_INFO * psSurfAttrib,
			    DC_BUFFER ** ppsBuffer);

PVRSRV_ERROR DCBufferUnimport(DC_BUFFER * psBuffer);

PVRSRV_ERROR DCBufferAcquire(DC_BUFFER * psBuffer, PMR ** psPMR);

PVRSRV_ERROR DCBufferRelease(PMR * psPMR);

PVRSRV_ERROR DCBufferPin(DC_BUFFER * psBuffer, DC_PIN_HANDLE * phPin);

PVRSRV_ERROR DCBufferUnpin(DC_PIN_HANDLE hPin);

PVRSRV_ERROR DCInit(IMG_VOID);
PVRSRV_ERROR DCDeInit(IMG_VOID);

#endif /*_DC_SERVER_H_  */
