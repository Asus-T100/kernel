/*************************************************************************/ /*!
@File
@Title          Common bridge header for rgxinit
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Declares common defines and structures that are used by both
                the client and sever side of the bridge for rgxinit
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#ifndef COMMON_RGXINIT_BRIDGE_H
#define COMMON_RGXINIT_BRIDGE_H

#include "rgx_bridge.h"
#include "rgxscript.h"
#include "rgx_fwif_shared.h"
#include "rgx_fwif.h"


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
typedef struct PVRSRV_BRIDGE_IN_RGXINITFIRMWARE_TAG
{
	IMG_HANDLE hDevNode;
	IMG_DEVMEM_SIZE_T uiFWMemAllocSize;
	IMG_BOOL bEnableSignatureChecks;
	IMG_UINT32 ui32SignatureChecksBufSize;
	IMG_UINT32 ui32RGXFWAlignChecksSize;
	IMG_UINT32 * pui32RGXFWAlignChecks;
	IMG_UINT32 ui32ConfigFlags;
	IMG_UINT32 ui32LogType;
	RGXFWIF_COMPCHECKS_BVNC sClientBVNC;
} PVRSRV_BRIDGE_IN_RGXINITFIRMWARE;


/* Bridge out structure for RGXInitFirmware */
typedef struct PVRSRV_BRIDGE_OUT_RGXINITFIRMWARE_TAG
{
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
typedef struct PVRSRV_BRIDGE_IN_RGXINITDEVPART2_TAG
{
	IMG_HANDLE hDevNode;
	RGX_INIT_COMMAND * psInitScript;
	RGX_INIT_COMMAND * psDbgScript;
	RGX_INIT_COMMAND * psDeinitScript;
	IMG_UINT32 ui32KernelCatBase;
	IMG_UINT32 ui32RGXActivePMConf;
} PVRSRV_BRIDGE_IN_RGXINITDEVPART2;


/* Bridge out structure for RGXInitDevPart2 */
typedef struct PVRSRV_BRIDGE_OUT_RGXINITDEVPART2_TAG
{
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXINITDEVPART2;

#endif /* COMMON_RGXINIT_BRIDGE_H */
