									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for srvcore
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for srvcore
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_SRVCORE_BRIDGE_H
#define COMMON_SRVCORE_BRIDGE_H

#include "pvrsrv_device_types.h"
#include "cache_external.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_SRVCORE_CMD_FIRST			(PVRSRV_BRIDGE_SRVCORE_START)
#define PVRSRV_BRIDGE_SRVCORE_CONNECT			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+0)
#define PVRSRV_BRIDGE_SRVCORE_DISCONNECT			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+1)
#define PVRSRV_BRIDGE_SRVCORE_ENUMERATEDEVICES			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+2)
#define PVRSRV_BRIDGE_SRVCORE_ACQUIREDEVICEDATA			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+3)
#define PVRSRV_BRIDGE_SRVCORE_GETMISCINFO			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+4)
#define PVRSRV_BRIDGE_SRVCORE_INITSRVCONNECT			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+5)
#define PVRSRV_BRIDGE_SRVCORE_INITSRVDISCONNECT			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+6)
#define PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTOPEN			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+7)
#define PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTWAIT			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+8)
#define PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTCLOSE			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+9)
#define PVRSRV_BRIDGE_SRVCORE_DUMPDEBUGINFO			PVRSRV_IOWR(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+10)
#define PVRSRV_BRIDGE_SRVCORE_CMD_LAST			(PVRSRV_BRIDGE_SRVCORE_CMD_FIRST+10)

/*******************************************
            Connect          
 *******************************************/

/* Bridge in structure for Connect */
typedef struct PVRSRV_BRIDGE_IN_CONNECT_TAG {
	IMG_UINT32 ui32Flags;
} PVRSRV_BRIDGE_IN_CONNECT;

/* Bridge out structure for Connect */
typedef struct PVRSRV_BRIDGE_OUT_CONNECT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_CONNECT;

/*******************************************
            Disconnect          
 *******************************************/

/* Bridge in structure for Disconnect */
typedef struct PVRSRV_BRIDGE_IN_DISCONNECT_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_DISCONNECT;

/* Bridge out structure for Disconnect */
typedef struct PVRSRV_BRIDGE_OUT_DISCONNECT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DISCONNECT;

/*******************************************
            EnumerateDevices          
 *******************************************/

/* Bridge in structure for EnumerateDevices */
typedef struct PVRSRV_BRIDGE_IN_ENUMERATEDEVICES_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_ENUMERATEDEVICES;

/* Bridge out structure for EnumerateDevices */
typedef struct PVRSRV_BRIDGE_OUT_ENUMERATEDEVICES_TAG {
	IMG_UINT32 ui32NumDevices;
	PVRSRV_DEVICE_IDENTIFIER *psDeviceIdentifier;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_ENUMERATEDEVICES;

/*******************************************
            AcquireDeviceData          
 *******************************************/

/* Bridge in structure for AcquireDeviceData */
typedef struct PVRSRV_BRIDGE_IN_ACQUIREDEVICEDATA_TAG {
	IMG_UINT32 ui32DevIndex;
	PVRSRV_DEVICE_TYPE eDeviceType;
} PVRSRV_BRIDGE_IN_ACQUIREDEVICEDATA;

/* Bridge out structure for AcquireDeviceData */
typedef struct PVRSRV_BRIDGE_OUT_ACQUIREDEVICEDATA_TAG {
	IMG_HANDLE hDevCookie;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_ACQUIREDEVICEDATA;

/*******************************************
            GetMiscInfo          
 *******************************************/

/* Bridge in structure for GetMiscInfo */
typedef struct PVRSRV_BRIDGE_IN_GETMISCINFO_TAG {
	IMG_UINT32 ui32StateRequest;
	IMG_UINT32 ui32MemoryStrLen;
} PVRSRV_BRIDGE_IN_GETMISCINFO;

/* Bridge out structure for GetMiscInfo */
typedef struct PVRSRV_BRIDGE_OUT_GETMISCINFO_TAG {
	IMG_UINT32 ui32ui32StatePresent;
	IMG_CHAR *puiMemoryStr;
	IMG_HANDLE hGlobalEventObject;
	IMG_UINT32 *pui32DDKVersion;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_GETMISCINFO;

/*******************************************
            InitSrvConnect          
 *******************************************/

/* Bridge in structure for InitSrvConnect */
typedef struct PVRSRV_BRIDGE_IN_INITSRVCONNECT_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_INITSRVCONNECT;

/* Bridge out structure for InitSrvConnect */
typedef struct PVRSRV_BRIDGE_OUT_INITSRVCONNECT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_INITSRVCONNECT;

/*******************************************
            InitSrvDisconnect          
 *******************************************/

/* Bridge in structure for InitSrvDisconnect */
typedef struct PVRSRV_BRIDGE_IN_INITSRVDISCONNECT_TAG {
	IMG_BOOL bInitSuccesful;
} PVRSRV_BRIDGE_IN_INITSRVDISCONNECT;

/* Bridge out structure for InitSrvDisconnect */
typedef struct PVRSRV_BRIDGE_OUT_INITSRVDISCONNECT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_INITSRVDISCONNECT;

/*******************************************
            EventObjectOpen          
 *******************************************/

/* Bridge in structure for EventObjectOpen */
typedef struct PVRSRV_BRIDGE_IN_EVENTOBJECTOPEN_TAG {
	IMG_HANDLE hEventObject;
} PVRSRV_BRIDGE_IN_EVENTOBJECTOPEN;

/* Bridge out structure for EventObjectOpen */
typedef struct PVRSRV_BRIDGE_OUT_EVENTOBJECTOPEN_TAG {
	IMG_HANDLE hOSEvent;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_EVENTOBJECTOPEN;

/*******************************************
            EventObjectWait          
 *******************************************/

/* Bridge in structure for EventObjectWait */
typedef struct PVRSRV_BRIDGE_IN_EVENTOBJECTWAIT_TAG {
	IMG_HANDLE hOSEventKM;
} PVRSRV_BRIDGE_IN_EVENTOBJECTWAIT;

/* Bridge out structure for EventObjectWait */
typedef struct PVRSRV_BRIDGE_OUT_EVENTOBJECTWAIT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_EVENTOBJECTWAIT;

/*******************************************
            EventObjectClose          
 *******************************************/

/* Bridge in structure for EventObjectClose */
typedef struct PVRSRV_BRIDGE_IN_EVENTOBJECTCLOSE_TAG {
	IMG_HANDLE hOSEventKM;
} PVRSRV_BRIDGE_IN_EVENTOBJECTCLOSE;

/* Bridge out structure for EventObjectClose */
typedef struct PVRSRV_BRIDGE_OUT_EVENTOBJECTCLOSE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_EVENTOBJECTCLOSE;

/*******************************************
            DumpDebugInfo          
 *******************************************/

/* Bridge in structure for DumpDebugInfo */
typedef struct PVRSRV_BRIDGE_IN_DUMPDEBUGINFO_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_DUMPDEBUGINFO;

/* Bridge out structure for DumpDebugInfo */
typedef struct PVRSRV_BRIDGE_OUT_DUMPDEBUGINFO_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DUMPDEBUGINFO;

#endif				/* COMMON_SRVCORE_BRIDGE_H */
