/*************************************************************************/ /*!
@File			tltestdefs.h
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
#ifndef __TLTESTDEFS_H__
#define __TLTESTDEFS_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "pvr_tlcommon.h"

/******************************************************************************
 *
 * TEST Related definitions and constants
 */
#define PVR_TL_TEST_STREAM 	"TLBRIDGE_TEST"
#define PVR_TL_TEST_UMBASE	0x00202000
#define PVR_TL_TEST_OFFSET	0x0008
#define PVR_TL_TEST_LEN		0x0010

#define PVR_TL_TEST_STREAM2_NAME "TLSTREAM2_TEST"
#define PVR_TL_TEST_STREAM2_SIZE 2

#define PVR_TL_TEST_CMD_SOURCE_START	10
typedef struct _PVR_TL_TEST_CMD_SOURCE_START_IN_
{
	IMG_CHAR 	pszStreamName[PRVSRVTL_MAX_STREAM_NAME_SIZE];
	IMG_UINT16	uiStreamSizeInPages;
	IMG_UINT16	uiInterval;
	IMG_UINT16  uiCallbackKicks;			// 0 for no limit of timer callbacks
	IMG_UINT16	uiPacketSize;				// 0 for random size between 1..255
											// size in bytes
	IMG_UINT32  uiStreamCreateFlags;
	IMG_UINT16	uiStartDelay;               // 0 for normal uiInterval delay
} PVR_TL_TEST_CMD_SOURCE_START_IN;


#define PVR_TL_TEST_CMD_SOURCE_STOP		11
typedef struct _PVR_TL_TEST_CMD_SOURCE_STOP_IN_
{
	IMG_CHAR 	pszStreamName[PRVSRVTL_MAX_STREAM_NAME_SIZE];
} PVR_TL_TEST_CMD_SOURCE_STOP_IN;

#define PVR_TL_TEST_CMD_SOURCE_START2	12	// Uses two stage data submit
typedef PVR_TL_TEST_CMD_SOURCE_START_IN PVR_TL_TEST_CMD_SOURCE_START2_IN;

#define PVR_TL_TEST_CMD_DEBUG_LEVEL		13
// No typedef, uses integer uiIn1 in union

#define PVR_TL_TEST_CMD_DUMP_TL_STATE	14
// No typedef, uses integer uiIn1 in union

#define PVR_TL_TEST_CMD_HWPERF_STATE	15
// No typedef, uses integer uiIn1 in union


typedef union _PVR_TL_TEST_CMD_IN_
{
	PVR_TL_TEST_CMD_SOURCE_START_IN sStart;
	PVR_TL_TEST_CMD_SOURCE_STOP_IN  sStop;
//	PVR_TL_TEST_CMD_SOURCE_START_IN sStart2; // Used by #12, use sStart instead
	IMG_UINT32	uiIn1;						 // Used by #13
} PVR_TL_TEST_CMD_IN;

// Has to be the largest test IN structure
#define PVR_TL_TEST_PARAM_MAX_SIZE  (sizeof(PVR_TL_TEST_CMD_IN)+4)

#if defined (__cplusplus)
}
#endif
#endif /* __TLINTERN_H__ */
/******************************************************************************
 End of file (tltestdefs.h)
******************************************************************************/

