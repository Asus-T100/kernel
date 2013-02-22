/*************************************************************************/ /*!
@File
@Title          Server bridge for ion
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for ion
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

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "physmem_ion.h"
#include "pmr.h"


#include "common_ion_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>


static IMG_INT
PVRSRVBridgePhysmemImportIon(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_PHYSMEMIMPORTION *psPhysmemImportIonIN,
					 PVRSRV_BRIDGE_OUT_PHYSMEMIMPORTION *psPhysmemImportIonOUT,
					 CONNECTION_DATA *psConnection)
{
	PMR * psPMRPtrInt;
	IMG_HANDLE hPMRPtrInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_ION_PHYSMEMIMPORTION);


	NEW_HANDLE_BATCH_OR_ERROR(psPhysmemImportIonOUT->eError, psConnection, 1);


	psPhysmemImportIonOUT->eError =
		PhysmemImportIon(psConnection,
					psPhysmemImportIonIN->ifd,
					psPhysmemImportIonIN->uiFlags,
					&psPMRPtrInt,
					&psPhysmemImportIonOUT->uiSize,
					&psPhysmemImportIonOUT->sAlign);
	/* Exit early if bridged call fails */
	if(psPhysmemImportIonOUT->eError != PVRSRV_OK)
	{
		goto PhysmemImportIon_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPMRPtrInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_PMR,
												psPMRPtrInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&PMRUnrefPMR);
	if (hPMRPtrInt2 == IMG_NULL)
	{
		psPhysmemImportIonOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PhysmemImportIon_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
					  &psPhysmemImportIonOUT->hPMRPtr,
					  (IMG_HANDLE) hPMRPtrInt2,
					  PVRSRV_HANDLE_TYPE_PHYSMEM_PMR,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  );
	COMMIT_HANDLE_BATCH_OR_ERROR(psPhysmemImportIonOUT->eError, psConnection);



PhysmemImportIon_exit:

	return 0;
}


PVRSRV_ERROR RegisterIONFunctions(IMG_VOID);
IMG_VOID UnregisterIONFunctions(IMG_VOID);

/*
 * Register all ION functions with services
 */
PVRSRV_ERROR RegisterIONFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_ION_PHYSMEMIMPORTION, PVRSRVBridgePhysmemImportIon);

	return PVRSRV_OK;
}

/*
 * Unregister all ion functions with services
 */
IMG_VOID UnregisterIONFunctions(IMG_VOID)
{
}
