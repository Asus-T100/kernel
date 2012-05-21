									    /*************************************************************************//*!
									       @File
									       @Title          RGX Bridge Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the rgx Bridge code
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGX_BRIDGE_H__)
#define __RGX_BRIDGE_H__

#include "rgxapi_miscinfo.h"
#include "pvr_bridge.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "common_rgxinit_bridge.h"
#include "common_rgxta3d_bridge.h"
#include "common_rgxcmp_bridge.h"
#include "common_rgxccb_bridge.h"
#include "common_rgxtq_bridge.h"
#include "common_hostportio_bridge.h"
#include "common_breakpoint_bridge.h"
#include "common_debugmisc_bridge.h"
#include "common_rgxpdump_bridge.h"
/* 
 * Bridge Cmd Ids
 */

/* *REMEMBER* to update PVRSRV_BRIDGE_LAST_RGX_CMD if you add/remove a command! 
 * Also you need to ensure all PVRSRV_BRIDGE_RGX_CMD_BASE+ offsets are sequential!
 */

#define PVRSRV_BRIDGE_RGX_CMD_BASE (PVRSRV_BRIDGE_LAST_NON_DEVICE_CMD+1)

/* FIXME*/
#define PVRSRV_BRIDGE_RGXTQ_START	(PVRSRV_BRIDGE_RGX_CMD_BASE + 0)
#define PVRSRV_BRIDGE_RGXCCB_START	(PVRSRV_BRIDGE_RGXTQ_CMD_LAST + 1)
#define PVRSRV_BRIDGE_RGXCMP_START	(PVRSRV_BRIDGE_RGXCCB_CMD_LAST + 1)
#define PVRSRV_BRIDGE_RGXINIT_START	(PVRSRV_BRIDGE_RGXCMP_CMD_LAST + 1)
#define PVRSRV_BRIDGE_RGXTA3D_START	(PVRSRV_BRIDGE_RGXINIT_CMD_LAST + 1)
#define PVRSRV_BRIDGE_HOSTPORTIO_START (PVRSRV_BRIDGE_RGXTA3D_CMD_LAST + 1)
#define PVRSRV_BRIDGE_BREAKPOINT_START (PVRSRV_BRIDGE_HOSTPORTIO_CMD_LAST +1)
#define PVRSRV_BRIDGE_DEBUGMISC_START (PVRSRV_BRIDGE_BREAKPOINT_CMD_LAST + 1)
#define PVRSRV_BRIDGE_RGXPDUMP_START (PVRSRV_BRIDGE_DEBUGMISC_CMD_LAST +1)
/* "last" below actually means last, not plus 1 as elsewhere */
#define PVRSRV_BRIDGE_LAST_RGX_CMD	(PVRSRV_BRIDGE_RGXPDUMP_CMD_LAST)

#if defined (__cplusplus)
}
#endif
#endif				/* __RGX_BRIDGE_H__ */
