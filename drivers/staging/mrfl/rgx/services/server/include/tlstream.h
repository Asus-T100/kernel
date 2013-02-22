/*************************************************************************/ /*!
@File
@Title          Transport Layer kernel side API.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    TL provides driver components with a way to copy data from kernel
                space to user space (eg screen/file). 

                Data can be passed to the Transport Layer through the 
                TL Stream (kernel space) API interface.

                The buffer provided to every stream is a modified version of a 
                circular buffer. Which CB version is created is specified by
                relevant flags when creating a stream. Currently only one type
                of buffer is available:
                - TL_FLAG_DROP_DATA:
                When the buffer is full, incoming data are dropped 
                (instead of overwriting older data) and a marker is set 
                to let the user know that data have been lost.

                All size/space requests are in bytes. However, the actual
                implementation uses native word sizes (ie 4 byte aligned).

                The user does not need to provide space for the stream buffer 
                as the TL handles memory allocations and usage.

                Inserting data to a stream's buffer can be done either:
                - by using TLReserve/TLCommit: User is provided with a buffer
                  to write data to.
                - or by using TLWrite:		 User provides a buffer with 
                                                 data to be committed. The TL 
                                                 copies the data from the 
                                                 buffer into the stream buffer 
                                                 and returns.
                Users should be aware that there are implementation overheads 
                associated with every stream buffer. If you find that less 
                data are captured than expected then try increasing the
                stream buffer size or use TLInfo to obtain buffer parameters
                and calculate optimum required values at run time.
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
#ifndef __TLSTREAM_H__
#define __TLSTREAM_H__

#if defined (__cplusplus)
extern "C" {
#endif
#include "img_types.h"

/*! Flags specifying circular buffer behaviour */
/*! Discard new data if the buffer is full */
#define TL_FLAG_DROP_DATA (1U<<0)		
/*! Overwrite most recent data instead of oldest data*/
/* NOT YET IMPLEMENTED #define TL_FLAG_OVERWRITE_LAST (1U<<1) */

/*! Struct used to pass internal TL stream sizes information to users.*/
typedef struct _TL_STREAM_INFO_
{
	IMG_UINT32 headerSize;			/*!< Packet header size in bytes */
	IMG_UINT32 minReservationSize;	/*!< Minimum data size reserved in bytes */
	IMG_UINT32 pageSize;			/*!< Page size in bytes */
	IMG_UINT32 pageAlign;			/*!< Page alignment in bytes */
} TL_STREAM_INFO, *PTL_STREAM_INFO;

/*************************************************************************/ /*!
 @Function		TLStreamCreate
 @Description	Request the creation of a new stream.
 @Output		phStream		Pointer to handle to store the new stream.
 @Input			szStreamName	Name of stream, maximum length:
 								  PRVSRVTL_MAX_STREAM_NAME_SIZE.
								  If a longer string is provided,creation fails.
 @Input			ui32Size		Desired buffer size in bytes.
 @Input			ui32StreamFlags	Flags that configure buffer behaviour.See above.
 @Return		PVRSRV_ERROR_INVALID_PARAMS	NULL stream handle or string name 
                                              exceeded MAX_STREAM_NAME_SIZE
 @Return 		PVRSRV_ERROR_OUT_OF_MEMORY  Failed to allocate space for stream
                                              handle.
 @Return		eError						Internal services call returned
                                              eError error number.
 @Return		PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR 
TLStreamCreate(IMG_HANDLE *phStream, 
			   IMG_CHAR *szStreamName,
			   IMG_UINT32 ui32Size,
			   IMG_UINT32 ui32StreamFlags);

/*************************************************************************/ /*!
 @Function		TLStreamDestroy
 @Description	Delete the stream associated with the given handle. After this
 				  call returns the handle is no longer valid.
 @Input			hStream		Handle to stream that will be released.
 @Return		None.
*/ /**************************************************************************/
IMG_VOID
TLStreamDestroy(IMG_HANDLE hStream);

/*************************************************************************/ /*!
 @Function		TLStreamReserve
 @Description	Reserve space in stream buffer. Every TLStreamReserve call must
 				  be followed by a matching TLStreamCommit call. While a 
				  TLStreamCommit call is pending for a stream, subsequent 
				  TLStreamReserve calls for this stream will fail.
 @Input			hStream			Stream handle.
 @Output		ppui8Data		Pointer to a pointer to a location in the 
 								  buffer. The caller can then use this address
								  in writing data into the stream. 
 @Input			ui32Size		Number of bytes to reserve in buffer.
 @Return        PVRSRV_INVALID_PARAMS       NULL stream handler.
 @Return        PVRSRV_ERROR_NOT_READY		There are data previously reserved
											  that are pending to be committed.
 @Return        PVRSRV_ERROR_STREAM_MISUSE  Misusing the stream by trying to 
                                              reserve more space than the 
											  buffer size.
 				PVRSRV_ERROR_DATA_DROPPED	Data are dropped as buffer is full.
 @Return		PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR 
TLStreamReserve(IMG_HANDLE hStream, 
				IMG_UINT8 **ppui8Data,
				IMG_UINT32 ui32Size);

/*************************************************************************/ /*!
 @Function		TLStreamCommit
 @Description	Notify TL that data have been written in the stream buffer.
 				  Should always follow and match TLStreamReserve call.
 @Input			hStream			Stream handle.
 @Input			ui32Size		Number of bytes that have been added to the
 								  stream.
 @Return		PVRSRV_ERROR_INVALID_PARAMS  NULL stream hundler.
 @Return        PVRSRV_ERROR_STREAM_MISUSE   Commit results in more data 
                                               committed than the buffer size,
											   the stream is misused.
 @Return        eError                        Commit was successful but 
                                                internal services call returned
                                                eError error number.
 @Return		PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR 
TLStreamCommit(IMG_HANDLE hStream,
			   IMG_UINT32 ui32Size);

/*************************************************************************/ /*!
 @Function		TLStreamWrite
 @Description	Combined Reserve/Commit call. This function Reserves space in 
 				  the specified stream buffer, copies ui32Size bytes of data
				  from the array pui8Src points to and Commits in an "atomic"
				  style operation.
 @Input			hStream			Stream handle.
 @Input			pui8Src			Source to read data from.
 @Input			ui32Size		Number of elements to copy and commit.
 @Return		PVRSRV_ERROR_INVALID_PARAMS  NULL stream handler.
 @Return		eError                       Error codes returned by either 
                                               Reserve or Commit.
 @Return		PVRSRV_OK
 */ /**************************************************************************/
PVRSRV_ERROR 
TLStreamWrite(IMG_HANDLE hStream, 
			  IMG_UINT8 *pui8Src,
			  IMG_UINT32 ui32Size);

/*************************************************************************/ /*!
 @Function		TLStreamInfo
 @Description	Run time information about buffer elemental sizes.
 				It sets psInfo members accordingly. Users can use those values
				to calculate the parameters they use in TLStreamCreate and 
				TLStreamReserve.
 @Output		psInfo			pointer to stream info struct.
 @Return		None.
*/ /**************************************************************************/
IMG_VOID 
TLStreamInfo(PTL_STREAM_INFO psInfo);

#if defined (__cplusplus)
}
#endif

#endif /* __TLSTREAM_H__ */
/*****************************************************************************
 End of file (tlstream.h)
*****************************************************************************/

