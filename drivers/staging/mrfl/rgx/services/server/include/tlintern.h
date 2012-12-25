/*************************************************************************/ /*!
@File
@Title          Transport Layer internals
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Transport Layer header used by TL internally
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
#ifndef __TLINTERN_H__
#define __TLINTERN_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "devicemem_typedefs.h"
#include "pvr_tlcommon.h"
#include "device.h"

/* Forward declarations */
typedef struct _TL_SNODE_* PTL_SNODE;

/*! TL stream structure container.
 *    ui32Buffer holds the circular buffer.
 *    ui32Start points to the beginning of the buffer and takes values from 
 *      0 to ui32Size.
 *    ui32Count counts how many data have been committed.
 *    ui32CountPending counts space that has been reserved but may have not 
 *      yet been filled with data.
 */
typedef struct _IMG_TLBUF 
{
	IMG_CHAR szName[PRVSRVTL_MAX_STREAM_NAME_SIZE];	/*!< String name identifier */
	IMG_BOOL bDrop; 							/*!< Flag: When buffer is full drop new data instead of overwriting older data */
// Currently not implemented 	IMG_BOOL bOWriteLast;						/*!< Flag: Overwrite most recent data when buffer is full */

	volatile IMG_UINT32 ui32Start; 				/*!< Pointer to the beginning of available data */
	volatile IMG_UINT32 ui32Count;				/*!< Pointer to already committed data which are ready to be copied to user space*/
	IMG_UINT32 ui32CountPending;				/*!< Pointer to next unallocated position in buffer */
	IMG_UINT32 ui32Size; 						/*!< Buffer size */
	IMG_UINT32 *ui32Buffer;			 			/*!< Actual data buffer */

	PTL_SNODE psNode;                           /*!< Ptr to parent stream node */
	DEVMEM_MEMDESC *psStreamMemDesc;	 		/*!< MemDescriptor used to allocate buffer space through PMR */
	DEVMEM_EXPORTCOOKIE sExportCookie; 			/*!< Export cookie for stream DEVMEM */
}IMG_TLBUF;
typedef IMG_TLBUF TL_STREAM, *PTL_STREAM;

/*
 * Transport Layer Stream Descriptor types/defs
 */
typedef struct _TL_STREAM_DESC_
{
	PTL_SNODE	psNode;			/*!< Ptr to parent stream node */
	IMG_UINT32	ui32Flags;
	IMG_HANDLE	hDataEvent; 	/* For wait call */
} TL_STREAM_DESC, *PTL_STREAM_DESC;

PTL_STREAM_DESC TLMakeStreamDesc(PTL_SNODE f1, IMG_UINT32 f2, IMG_HANDLE f3);

#define TL_STREAM_KM_FLAG_MASK	0xFFFF0000
#define TL_STREAM_FLAG_TEST 	0x10000000
#define TL_STREAM_FLAG_WRAPREAD	0x00010000

#define TL_STREAM_UM_FLAG_MASK	0x0000FFFF

/*
 * Transport Layer stream list node
 */
typedef struct _TL_SNODE_
{
	struct _TL_SNODE_*  psNext;
	IMG_HANDLE			hDataEventObj;
	PTL_STREAM 			psStream;
	PTL_STREAM_DESC 	psDesc;
} TL_SNODE;

PTL_SNODE TLMakeSNode(IMG_HANDLE f2, TL_STREAM *f3, TL_STREAM_DESC *f4);

/*
 * Transport Layer global top types and variables
 * Use access function to obtain pointer.
 */
typedef struct _TL_GDATA_
{
	/* Only allow one client for now... */
	IMG_PVOID psTlClient;

	/* List of Streams, only 1 node supported at presnet */
	PTL_SNODE psHead;

	// TODO: Add Lock in case Destroy stream clash with client APIs?

	/* Void pointer to */
	IMG_PVOID psRgxDevNode;					/* PVRSRV_DEVICE_NODE*    */

} TL_GLOBAL_GDATA, *PTL_GLOBAL_DATA;

/*
 * Transport Layer Internal Kernel-Mode Server API
 */
TL_GLOBAL_GDATA* TLGGD(void);		/* TLGetGlobalData() */

IMG_VOID TLSetGlobalRgxDevice(PVRSRV_DEVICE_NODE *psDevNode);

PVRSRV_DEVICE_NODE* TLGetGlobalRgxDevice(void);

IMG_VOID  TLAddStreamNode(PTL_SNODE psAdd);
PTL_SNODE TLFindStreamNodeByName(IMG_PCHAR pszName);
PTL_SNODE TLFindStreamNodeByDesc(PTL_STREAM_DESC psDesc);
IMG_VOID  TLTryToRemoveAndFreeStreamNode(PTL_SNODE psRemove);

/*
 * Transport Layer stream interface to server part declared here to avoid
 * circular dependency.
 */
IMG_UINT32 TLStreamAcquireReadPos(PTL_STREAM psStream, IMG_UINT32* puiReadOffset);
IMG_VOID TLStreamAdvanceReadPos(PTL_STREAM psStream, IMG_UINT32 uiReadLen);

DEVMEM_EXPORTCOOKIE* TLStreamGetBufferCookie(PTL_STREAM psStream);
IMG_BOOL TLStreamEOS(PTL_STREAM psStream);

#if defined (__cplusplus)
}
#endif

#endif /* __TLINTERN_H__ */
/******************************************************************************
 End of file (tlintern.h)
******************************************************************************/

