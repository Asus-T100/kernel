/*************************************************************************/ /*!
@File
@Title          Transport Layer kernel side API implementation.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Transport Layer API implementation.
                These functions are provided to driver components.
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

//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON
#include "pvr_debug.h"

#include "allocmem.h"
#include "pvrsrv_error.h"
#include "osfunc.h"

#include "pvr_tlcommon.h"
#include "tlintern.h"
#include "tlstream.h"

/*
 * stream helper functions: word/byte conversion and array position.
 * IMPORTANT: these two functions are duplicated in tl_stream_test.c
 */
/* Rounds input value to next multiple of sizeof(img_uint32), ie roundToUI32(3) = 4 */
static IMG_UINT32 roundToUI32(IMG_UINT32 x)
{
	if ( !(x%sizeof(IMG_UINT32)))
		return x;
	else
		return (((x+sizeof(IMG_UINT32)))& ~(sizeof(IMG_UINT32)-1));
}
/* Convert given argument from bytes to array position */
static IMG_UINT32 elementize(IMG_UINT32 b)
{
	return ( (b+sizeof(IMG_UINT32)-1)/sizeof(IMG_UINT32) );
}

/******************************************************************************* 
 * circular buffer specific routines
 ******************************************************************************/
static IMG_UINT32
cbHasNSpace(IMG_TLBUF *psTmp, IMG_UINT32 size) 
{
	return ((psTmp->ui32Count + psTmp->ui32CountPending + size) < psTmp->ui32Size);
}   

/******************************************************************************* 
 * TL Server public API implementation.
 ******************************************************************************/
PVRSRV_ERROR
TLStreamCreate(IMG_HANDLE *phStream,
			   IMG_CHAR *szStreamName,
			   IMG_UINT32 ui32Size,
			   IMG_UINT32 ui32StreamFlags)
{
	IMG_TLBUF*     psTmp;
	PVRSRV_ERROR   eError;
	IMG_HANDLE     hEventList;
	PTL_SNODE      psn = 0;

	DEVMEM_FLAGS_T uiMemFlags =  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
								 PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE | 
								 PVRSRV_MEMALLOCFLAG_GPU_READABLE |
								 PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
								 PVRSRV_MEMALLOCFLAG_UNCACHED |
								 PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
								 PVRSRV_MEMALLOCFLAG_CPU_UNCACHED |
								 PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE;

	PVR_DPF_ENTERED;
	/* Sanity checks:  */
	/* non NULL handler required */
	if ( NULL == phStream ) 
	{ 
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_INVALID_PARAMS);
	}
	if (OSStringLength(szStreamName) >= PRVSRVTL_MAX_STREAM_NAME_SIZE) 
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_INVALID_PARAMS);
	}
	
	/* Allocate stream structure container (stream struct) for the new stream */
	psTmp = OSAllocZMem(sizeof(IMG_TLBUF)) ;
	if ( NULL == psTmp ) 
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	OSStringCopy(psTmp->szName, szStreamName);

	if ( ui32StreamFlags & TL_FLAG_DROP_DATA ) 
	{
		psTmp->bDrop = IMG_TRUE;
	}

	/* Round the requested bytes to a multiple of array elements' size, eg round 3 to 4 */
	psTmp->ui32Size = roundToUI32(ui32Size);
	psTmp->ui32Start = 0;
	psTmp->ui32Count = 0;
	psTmp->ui32CountPending = 0;

	/* Allocate memory for the circular buffer and export it to user space. */
	eError = DevmemAllocateExportable( IMG_NULL,
									   (IMG_HANDLE) TLGetGlobalRgxDevice(),
									   (IMG_DEVMEM_SIZE_T)psTmp->ui32Size,
									   4096,
									   uiMemFlags | PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
									   &psTmp->psStreamMemDesc);
	PVR_LOGG_IF_ERROR(eError, "DevmemAllocateExportable", e0);

	eError = DevmemAcquireCpuVirtAddr( psTmp->psStreamMemDesc, (IMG_VOID**) &psTmp->ui32Buffer );
	PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", e1);

	eError = DevmemExport(psTmp->psStreamMemDesc, &(psTmp->sExportCookie));
	PVR_LOGG_IF_ERROR(eError, "DevmemExport", e2);

	/* Synchronisation object to synchronise with user side data transfers. */
	eError = OSEventObjectCreate(psTmp->szName, &hEventList);
	if (eError != PVRSRV_OK)
	{
		goto e3;
	}

/* 	PVR_DPF((PVR_DBG_WARNING,
			 "== TLStreamCreating \" %s \": stream->size: %d, requested size: %d \n",
			 psTmp->szName,
			 psTmp->ui32Size,
			 ui32Size));
*/
	/* Now remember the stream in the global TL structures */
	psn = TLMakeSNode(hEventList, (TL_STREAM *)psTmp, 0);
	if (psn == NULL)
	{
		eError=PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e3;
	}
	TLAddStreamNode(psn);

	/* Pass the new stream back to caller */
	*phStream = (IMG_HANDLE)psTmp;
	PVR_DPF_RETURN_OK;

e3:
	DevmemUnexport(psTmp->psStreamMemDesc, &(psTmp->sExportCookie));
e2:
	DevmemReleaseCpuVirtAddr( psTmp->psStreamMemDesc );
e1:
	DevmemFree(psTmp->psStreamMemDesc);
e0:
	OSFreeMem((IMG_VOID*)psTmp);
	PVR_DPF_RETURN_RC(eError);
}

IMG_VOID 
TLStreamDestroy(IMG_HANDLE hStream)
{
	IMG_TLBUF	*psTmp;

/* 	PVR_DPF((PVR_DBG_WARNING, "== TLStreamDestroying (freeing stream structure)."));*/

	PVR_DPF_ENTERED;

	if ( IMG_NULL == hStream )
	{
		PVR_DPF((PVR_DBG_WARNING, "TLStreamDestroy failed as NULL stream handler passed, nothing done.\n"));
		PVR_DPF_RETURN;
	}

	psTmp = (IMG_TLBUF*)hStream;

	/* First remove it from the global structures to prevent access
	 * while it is being free'd. Lock it?
	 */
	psTmp->psNode->psStream = IMG_NULL;
	TLTryToRemoveAndFreeStreamNode(psTmp->psNode);

	DevmemUnexport(psTmp->psStreamMemDesc, &psTmp->sExportCookie);
	DevmemReleaseCpuVirtAddr(psTmp->psStreamMemDesc);
	DevmemFree(psTmp->psStreamMemDesc);

	OSFreeMem((IMG_VOID*)psTmp);
	PVR_DPF_RETURN;
}

PVRSRV_ERROR 
TLStreamReserve(IMG_HANDLE hStream, 
				IMG_UINT8 **ppui8Data, 
				IMG_UINT32 ui32ReqSize)
{
	IMG_TLBUF *psTmp;
	IMG_UINT32 end, localReqSize;
	int pad;

/* 	PVR_DPF((PVR_DBG_WARNING, "== TLStreamReserving. requested:%d, rounded:%d.", ui32ReqSize, roundUI32(ui32ReqSize))); */

	PVR_DPF_ENTERED;

	if ( IMG_NULL == hStream )
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_INVALID_PARAMS);
	}
	psTmp = (IMG_TLBUF*)hStream;

	if ( 0 != psTmp->ui32CountPending ) 
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_NOT_READY);
	}

	/* round requested size to a multiple of buffer (array) elements. */
	localReqSize = roundToUI32(ui32ReqSize);

	/* Find the end of the existing data in the array */
	end = (psTmp->ui32Start + psTmp->ui32Count) % psTmp->ui32Size;
	/* How much padding might be required? */
	if ( end + localReqSize > psTmp->ui32Size - sizeof(PVRSRVTL_PACKETHDR) ) // minus packet header size.
	{
		pad = psTmp->ui32Size - end;
	}
	else
	{
		pad = 0 ;
	}
	/* The easy case: buffer has enough space to hold the requested packet (data + header) */
	if ( cbHasNSpace(psTmp, localReqSize + sizeof(PVRSRVTL_PACKETHDR) + pad ) ) 
	{
		if ( pad ) 
		{ 
			psTmp->ui32Buffer[elementize(end)] = PVRSRVTL_SET_PACKET_PADDING(pad-sizeof(PVRSRVTL_PACKETHDR)) ;
			end = end + pad;
			psTmp->ui32Count = psTmp->ui32Count + pad;
		}
		/* size-stamp new data (save data size in packet header) */
		end = ( end ) % psTmp->ui32Size;
		psTmp->ui32Buffer[elementize(end)] = PVRSRVTL_SET_PACKET_DATA(ui32ReqSize);

		/* write the new data to the next position in the buffer */
		end = ( end + sizeof(PVRSRVTL_PACKETHDR) ) % psTmp->ui32Size;
		*ppui8Data =  (IMG_UINT8*)&(psTmp->ui32Buffer[elementize(end)]) ;

		/* update pending offset: size stamp + data  */
		psTmp->ui32CountPending += localReqSize + sizeof(PVRSRVTL_PACKETHDR) ;
	}
	/* The not so easy case: not enough space, need to decide what to do */
	else 	
	{
		/* Sanity check that the user is not trying to add more data than the buffer size */
		if ( localReqSize+sizeof(PVRSRVTL_PACKETHDR) > psTmp->ui32Size )
		{
			PVR_DPF_RETURN_RC(PVRSRV_ERROR_STREAM_MISUSE);
		}

		/* No data overwriting, insert data_lost flag and return */
		if (psTmp->bDrop) 
		{
			/* Caller should not try to use ppui8Data,
			 * NULLify to give user a chance of avoiding memory corruption */
			ppui8Data = IMG_NULL;

			/* minus one ui32 to account for array arithmetic. 
			 * This flag should not be inserted two consecutive times. */
			end = ((psTmp->ui32Start + psTmp->ui32Count ) - sizeof(IMG_UINT32)) % psTmp->ui32Size ;
			if ( PVRSRVTL_PACKETTYPE_MOST_RECENT_DATA_LOST 
				 != 
				 GET_PACKET_TYPE( (PVRSRVTL_PACKETHDR*)&(psTmp->ui32Buffer[elementize(end)]) ) )
			{
				end = ( end + sizeof(PVRSRVTL_PACKETHDR) ) % psTmp->ui32Size ;
				psTmp->ui32Buffer[elementize(end)] = PVRSRVTL_SET_PACKET_DATA_LOST ;
				psTmp->ui32Count = psTmp->ui32Count + sizeof(PVRSRVTL_PACKETHDR) ;
			}

			PVR_DPF_RETURN_RC(PVRSRV_ERROR_DATA_DROPPED);
		} 
/* This is where circular buffer variations WILL be implemented in the FUTURE.
		else 
		{
			// Safely overwriting data is impossible
			// if there are data pending to be inserted in the buffer
		}
*/	}
	PVR_DPF_RETURN_OK;
}

PVRSRV_ERROR
TLStreamCommit(IMG_HANDLE hStream, IMG_UINT32 ui32ReqSize)
{
	IMG_TLBUF *psTmp;
	IMG_UINT32 uiOldCount, localReqSize;
	PVRSRV_ERROR eError;

	PVR_DPF_ENTERED;

	if ( IMG_NULL == hStream )
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_INVALID_PARAMS);
	}
	psTmp = (IMG_TLBUF*)hStream;

	// Space in buffer is 4b aligned
	localReqSize = roundToUI32(ui32ReqSize);

	/* Sanity check. Should never happen. ReqSize + the packet header. */
	if ( psTmp->ui32Count + localReqSize + sizeof(PVRSRVTL_PACKETHDR) > psTmp->ui32Size )
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_STREAM_MISUSE);
	}

	uiOldCount = psTmp->ui32Count;

	/* Update number of committed bytes: packet (data + header). */
	psTmp->ui32Count += localReqSize+sizeof(PVRSRVTL_PACKETHDR);

	/* and update CountPending by as much as the Count update.  */
	psTmp->ui32CountPending -= (localReqSize+sizeof(PVRSRVTL_PACKETHDR));

	if ((uiOldCount == 0) && (psTmp->ui32Count))
	{
		/* Need to cope with testing streams at init stage i.e. no descriptor */
		eError = OSEventObjectSignal(psTmp->psNode->hDataEventObj);
		if ( eError != PVRSRV_OK)
		{
			PVR_DPF_RETURN_RC(eError);
		}
	}
/* 	PVR_DPF((PVR_DBG_WARNING, "== TLStreamCommited:%d. Start: %u, Count:%u .", ui32ReqSize, psTmp->ui32Start, psTmp->ui32Count));*/

	PVR_DPF_RETURN_OK;
}

PVRSRV_ERROR
TLStreamWrite(IMG_HANDLE hStream, IMG_UINT8 *pui8Src, IMG_UINT32 ui32Size)
{
	IMG_TLBUF *psTmp;
	IMG_UINT32 *pui32Dest = IMG_NULL;
	PVRSRV_ERROR eError;

	PVR_DPF_ENTERED;

	//PVR_DPF((PVR_DBG_WARNING, "== TLStreamWrite."));

	if ( IMG_NULL == hStream )
	{
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_INVALID_PARAMS);
	}
	psTmp = (IMG_TLBUF*)hStream;
	PVR_ASSERT(psTmp);

	eError = TLStreamReserve(hStream, (IMG_UINT8**)&pui32Dest, ui32Size); 
	if ( PVRSRV_OK != eError ) 
	{	
		PVR_DPF_RETURN_RC(eError);
	}
	else
	{
		PVR_ASSERT ( pui32Dest != NULL );
		OSMemCopy((IMG_VOID*)pui32Dest, (IMG_VOID*)pui8Src, roundToUI32(ui32Size) * sizeof(IMG_UINT8));
		eError = TLStreamCommit(hStream, ui32Size);
		if ( PVRSRV_OK != eError ) 
		{	
			PVR_DPF_RETURN_RC(eError);
		}
	}
	PVR_DPF_RETURN_OK;
}

IMG_VOID TLStreamInfo(PTL_STREAM_INFO psInfo)
{
 	IMG_DEVMEM_SIZE_T actual_req_size;
	IMG_DEVMEM_ALIGN_T align = 4;

 	actual_req_size = 2; 
	DevmemExportalignAdjustSizeAndAlign(IMG_NULL, &actual_req_size, &align);

	psInfo->headerSize = sizeof(PVRSRVTL_PACKETHDR);
	psInfo->minReservationSize = sizeof(IMG_UINT32);
	psInfo->pageSize = (IMG_UINT32)(actual_req_size);
	psInfo->pageAlign = (IMG_UINT32)(align);
}

/*
 * Internal stream APIs to server part of Transport Layer, declared in
 * header tlintern.h. Direct pointers to stream objects are used here as
 * these functions are internal.
 */
IMG_UINT32
TLStreamAcquireReadPos(PTL_STREAM psStream, IMG_UINT32* puiReadOffset)
{
	IMG_UINT32 uiReadLen = 0;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psStream);
	PVR_ASSERT(puiReadOffset);

	// No data available...
	if (psStream->ui32Count == 0)
		PVR_DPF_RETURN_VAL(0);

	// Data available to read...
	*puiReadOffset = psStream->ui32Start;

	PVR_DPF((PVR_DBG_VERBOSE,
			"TLStreamAcquireReadPos Start before: count:%d, start:%d, size:%d",
			psStream->ui32Count, psStream->ui32Start, psStream->ui32Size));

	if (psStream->ui32Start+psStream->ui32Count > psStream->ui32Size)
	{	// CB has wrapped around
		PVR_DPF((PVR_DBG_VERBOSE, "TLStreamAcquireReadPos buffer has wrapped"));
		uiReadLen = psStream->ui32Size - psStream->ui32Start;
	}
	else
	{	// CB has not wrapped
		uiReadLen = psStream->ui32Count;
	}

	PVR_DPF_RETURN_VAL(uiReadLen);
}

IMG_VOID
TLStreamAdvanceReadPos(PTL_STREAM psStream, IMG_UINT32 uiReadLen)
{
	PVR_ASSERT(psStream);

	PVR_DPF_ENTERED;

	PVR_DPF((PVR_DBG_VERBOSE,
			"TLStreamAdvanceReadPos Count before: uiReadLen:%d, count:%d",
			uiReadLen, psStream->ui32Count));

	psStream->ui32Count -= uiReadLen;

	PVR_DPF((PVR_DBG_VERBOSE,
			"TLStreamAdvanceReadPos Start before: count:%d, start:%d, size:%d",
			psStream->ui32Count, psStream->ui32Start, psStream->ui32Size));

	psStream->ui32Start = (psStream->ui32Start + uiReadLen) %psStream->ui32Size;

	PVR_DPF((PVR_DBG_VERBOSE, "TLStreamAdvanceReadPos Start after: start:%d",
			psStream->ui32Start));

	PVR_DPF_RETURN;
}

DEVMEM_EXPORTCOOKIE*
TLStreamGetBufferCookie(PTL_STREAM psStream)
{
	PVR_DPF_ENTERED;

	PVR_ASSERT(psStream);

	PVR_DPF_RETURN_VAL(&psStream->sExportCookie);
}

IMG_BOOL
TLStreamEOS(PTL_STREAM psStream)
{
	PVR_DPF_ENTERED;

	PVR_DPF_RETURN_VAL(psStream->ui32Count == 0);
}

