/*************************************************************************/ /*!
@File
@Title          KM server Transport Layer implementation
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Main bridge APIs for Transport Layer client functions
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

#include "img_defs.h"

//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON
#include "pvr_debug.h"

#include "connection_server.h"
#include "allocmem.h"

#include "tlintern.h"
#include "tlserver.h"

#include "tltestdefs.h"

#define NO_DATA_WAIT_PERIOD	5000
#define NO_ACQUIRE 			0xffffffffU


/*
 * Transport Layer Client API Kernel-Mode bridge implementation
 */

PVRSRV_ERROR
TLServerConnectKM(CONNECTION_DATA *psConnection)
{
	PVR_DPF_ENTERED;

	PVR_ASSERT(psConnection);
	if (TLGGD()->psTlClient != 0)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_SRV_CONNECT_FAILED);

	TLGGD()->psTlClient = psConnection;

	// TODO: Create event object to give daemon something to wait on until the
	// stream is created

	PVR_DPF_RETURN_OK;
}

PVRSRV_ERROR
TLServerDisconnectKM(CONNECTION_DATA *psConnection)
{
	PVR_DPF_ENTERED;

	PVR_ASSERT(psConnection);

	if (TLGGD()->psTlClient != psConnection)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_SRV_DISCONNECT_FAILED);

	TLGGD()->psTlClient = 0;

	// TODO: Add resource clean up to mirror the above connect call

	PVR_DPF_RETURN_OK;
}

PVRSRV_ERROR
TLServerOpenStreamKM(IMG_PCHAR  	 	   pszName,
			   	     IMG_UINT32 		   ui32Mode,
			   	     PTL_STREAM_DESC* 	   ppsSD,
			   	     DEVMEM_EXPORTCOOKIE** ppsBufCookie)
{
	PVRSRV_ERROR 	eError = PVRSRV_OK;
	PTL_SNODE		psNode = 0;
	TL_STREAM_DESC* psNewSD = 0;
	IMG_HANDLE 		hEventObj;
	PTL_GLOBAL_DATA psGD = TLGGD();

	PVR_DPF_ENTERED;

	// Sanity check, quick exit if there are no streams
	if (psGD->psHead == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_NOT_FOUND);

	// There are one or more streams, find the correct one in the list
	psNode = TLFindStreamNodeByName(pszName);
	if (psNode == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_NOT_FOUND);

	// Only one client/descriptor per stream supported
	if (psNode->psDesc != NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_ALREADY_OPEN)

	// Create an event handle for this client to wait on when no data in stream
	// buffer.
	eError = OSEventObjectOpen(psNode->hDataEventObj, &hEventObj);
	if (eError != PVRSRV_OK)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_CREATE_EVENT);

	psNewSD = TLMakeStreamDesc(psNode, ui32Mode, hEventObj);
	if (!psNewSD)
		goto e1;

	// Copy the export cookie back to the user mode API to enable access to
	// the stream buffer from user-mode process.
	*ppsBufCookie = TLStreamGetBufferCookie(psNode->psStream);

	psNode->psDesc = psNewSD;
	*ppsSD = psNewSD;

	PVR_DPF((PVR_DBG_VERBOSE, "TLServerOpenStreamKM evList=%p, evObj=%p", psNode->hDataEventObj, psNode->psDesc->hDataEvent));

	PVR_DPF_RETURN_OK;


// Cleanup jump for event object
e1:
	OSEventObjectClose(hEventObj);
	eError = PVRSRV_ERROR_OUT_OF_MEMORY;
//e0:
	PVR_DPF_RETURN_RC(eError);
}

PVRSRV_ERROR
TLServerCloseStreamKM(PTL_STREAM_DESC psSD)
{
	PVRSRV_ERROR    eError = PVRSRV_OK;
	PTL_GLOBAL_DATA psGD = TLGGD();
	PTL_SNODE		psNode = 0;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psSD);

	// Sanity check, quick exit if there are no streams
	if (psGD->psHead == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_HANDLE_NOT_FOUND)

	// Check stream still valid
	psNode = TLFindStreamNodeByDesc(psSD);
	if ((psNode == NULL) || (psNode != psSD->psNode))
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_HANDLE_NOT_FOUND)

	// Close and free the event handle resource used by this descriptor
	eError = OSEventObjectClose(psSD->hDataEvent);
	if (eError != PVRSRV_OK)
	{
		// Log error but continue as it seems best
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectClose() failed error %d", eError));
		eError = PVRSRV_ERROR_UNABLE_TO_DESTROY_EVENT;
	}

	// Remove from stream object/list
	psSD->psNode->psDesc = NULL;
	TLTryToRemoveAndFreeStreamNode(psSD->psNode);

	// Free the stream descriptor object
	OSFreeMem(psSD);

	PVR_DPF_RETURN_RC(eError);
}

PVRSRV_ERROR
TLServerAcquireDataKM(PTL_STREAM_DESC psSD,
		   	   		  IMG_UINT32*	  puiReadOffset,
		   	   		  IMG_UINT32* 	  puiReadLen)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	TL_GLOBAL_GDATA*	psGD = TLGGD();
	IMG_UINT32		    uiTmpOffset = NO_ACQUIRE;
	IMG_UINT32  		uiTmpLen;
	PTL_SNODE			psNode = 0;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psSD);

	// Sanity check, quick exit if there are no streams
	if (psGD->psHead == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_STREAM_ERROR)

	// Check stream still valid
	psNode = TLFindStreamNodeByDesc(psSD);
	if ((psNode == NULL) || (psNode != psSD->psNode))
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_HANDLE_NOT_FOUND)

	// Does stream still exist?
	if (psNode->psStream == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_RESOURCE_UNAVAILABLE);

	//PVR_DPF((PVR_DBG_VERBOSE, "TLServerAcquireDataKM evList=%p, evObj=%p", psSD->psNode->hDataEventObj, psSD->hDataEvent));

	// Check for data in the associated stream buffer, sleep/wait if none
	while ((uiTmpLen =TLStreamAcquireReadPos(psSD->psNode->psStream, &uiTmpOffset)) == 0 && (!(psSD->ui32Flags&PVRSRV_STREAM_FLAG_NONBLOCKING)))
	{
		PVR_DPF((PVR_DBG_VERBOSE, "TLAcquireDataKM sleeping..."));

		// Loop around if EndOfStream (nothing to read) and wait times out,
		// exit loop if not time out but data is ready for client
		while (TLStreamEOS(psSD->psNode->psStream))
		{
			eError = OSEventObjectWaitTimeout(psSD->hDataEvent, NO_DATA_WAIT_PERIOD);
			if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_TIMEOUT))
				PVR_DPF_RETURN_RC(eError);

			// Check we have been woken up because of data OR because
			// the stream has been destroyed?
			//
			if (psSD->psNode->psStream == NULL)
			{
				PVR_DPF((PVR_DBG_VERBOSE, "TLAcquireDataKM awake, but stream now NULL"));
				PVR_DPF_RETURN_RC(PVRSRV_ERROR_RESOURCE_UNAVAILABLE);
			}
		}
	}

	PVR_DPF((PVR_DBG_VERBOSE, "TLAcquireDataKM awake, buffer start =%d, count=%d ", uiTmpOffset, uiTmpLen));

	// If non blocking then default tmp values get returned, client code must
	// handle it. if blocking data should now be available, return offset and
	// len to user mode caller
	*puiReadOffset = uiTmpOffset;
	*puiReadLen = uiTmpLen;

	PVR_DPF((PVR_DBG_VERBOSE, "TLAcquireDataKM return offset=%d, len=%d bytes", *puiReadOffset, *puiReadLen));

	PVR_DPF_RETURN_OK;
}

PVRSRV_ERROR
TLServerReleaseDataKM(PTL_STREAM_DESC psSD,
		 	 		  IMG_UINT32  	  uiReadOffset,
		 	 		  IMG_UINT32  	  uiReadLen)
{
	TL_GLOBAL_GDATA*	psGD = TLGGD();
	PTL_SNODE			psNode = 0;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psSD);

	// Sanity check, quick exit if there are no streams
	if (psGD->psHead == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_STREAM_ERROR)

	// Check stream still valid
	psNode = TLFindStreamNodeByDesc(psSD);
	if ((psNode == NULL) || (psNode != psSD->psNode))
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_HANDLE_NOT_FOUND)

	// Does stream still exist?
	if (psNode->psStream == NULL)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_RESOURCE_UNAVAILABLE);

	PVR_DPF((PVR_DBG_VERBOSE, "TLReleaseDataKM uiReadOffset=%d, uiReadLen=%d", uiReadOffset, uiReadLen));

	// Move read position on to free up space in stream buffer
	TLStreamAdvanceReadPos(psNode->psStream, uiReadLen);

	PVR_DPF_RETURN_OK;
}

/*****************************************************************************
 End of file (tlserver.c)
*****************************************************************************/

