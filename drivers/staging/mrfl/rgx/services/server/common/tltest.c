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

#if defined(LINUX)
#include "linux/random.h"
#elif defined(_WIN32)
#include "wdm.h"
#else
#error TLTEST.C needs random number API
#endif

#if defined(DEBUG) && !defined(PVRSRV_NEED_PVR_DPF)
#define PVRSRV_NEED_PVR_DPF
#endif

#include "img_defs.h"

#include "pvrsrv_error.h"

//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON
#include "pvr_debug.h"

#include "allocmem.h"
#include "osfunc.h"

#include "tlintern.h"
#include "tlstream.h"
#include "tltestdefs.h"

/******************************************************************************
 * Do not include tlserver.h, as this is testing the kernelAPI, some
 * duplicate definitions
 */
PVRSRV_ERROR
TLServerTestIoctlKM(IMG_UINT32  	uiCmd,
 			IMG_PBYTE	  	uiIn1,
	 		IMG_UINT32  	uiIn2,
	   	  	IMG_UINT32*		puiOut1,
	   	  	IMG_UINT32* 	puiOut2);


/*****************************************************************************/

#if defined(PVR_TRANSPORTLAYER_TESTING)

/******************************************************************************
 *
 * TL KM Test helper global variables, prototypes
 */

typedef struct _TLT_SRCNODE_
{
	struct _TLT_SRCNODE_* psNext;

	IMG_HANDLE  gTLStream;
	IMG_HANDLE  gStartTimer;
	IMG_HANDLE  gStartCleanupMISR;
	IMG_HANDLE  gSourceTimer;
	IMG_HANDLE  gSourceCleanupMISR;

	IMG_UINT32  gSourceCount;
	IMG_UINT32* gpuiDataPacket;

	PVR_TL_TEST_CMD_SOURCE_START_IN gsStartIn;
	IMG_VOID (*gSourceWriteCB)(IMG_VOID *, IMG_UINT16);

} TLT_SRCNODE;

TLT_SRCNODE* gpsSourceStack = 0;


#define RANDOM_MOD 256
IMG_BOOL	gRandomReady = IMG_FALSE;
IMG_BYTE	gRandomStore[RANDOM_MOD];

static IMG_VOID StartTimerFuncCB(IMG_VOID* p);
static IMG_VOID SourceTimerFuncCB(IMG_VOID* p);
static IMG_VOID SourceWriteFunc(IMG_VOID* p, IMG_UINT16 uiPacketSize);
static IMG_VOID SourceWriteFunc2(IMG_VOID* p, IMG_UINT16 uiPacketSize);

/******************************************************************************
 *
 * TL KM Test helper routines
 */
static PVRSRV_ERROR  TLTestCMD_SourceStart (PVR_TL_TEST_CMD_SOURCE_START_IN* psIn1,
							IMG_VOID (*pfCB)(IMG_VOID* , IMG_UINT16))
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	IMG_UINT32 		i;
	IMG_UINT16		uiPacketBufferSize;
	TLT_SRCNODE*	srcn =0;


	PVR_DPF_ENTERED;

	srcn = OSAllocZMem(sizeof(TLT_SRCNODE));
	if (!srcn)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_OUT_OF_MEMORY);

	// Remember the start parameters, initialise buffers
	srcn->gsStartIn = *psIn1;
	PVR_DPF((PVR_DBG_MESSAGE, "--- SourceParameters (n:%s, p:%d, i:%d, k:%d, s:%d, f:%x d:%d)",
			psIn1->pszStreamName, psIn1->uiStreamSizeInPages,
			psIn1->uiInterval, psIn1->uiCallbackKicks, psIn1->uiPacketSize,
			psIn1->uiStreamCreateFlags, psIn1->uiStartDelay));


	uiPacketBufferSize = (psIn1->uiPacketSize == 0) ? 256 : psIn1->uiPacketSize;
	srcn->gpuiDataPacket = OSAllocMem(uiPacketBufferSize * sizeof(IMG_UINT32));
	if (!srcn->gpuiDataPacket)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_OUT_OF_MEMORY);
	for (i=0; i<uiPacketBufferSize; i++) srcn->gpuiDataPacket[i] = 10000+i;

	if (!gRandomReady)
	{
#if defined(LINUX)
		get_random_bytes(gRandomStore, sizeof(gRandomStore));
#elif defined(_WIN32)
		LARGE_INTEGER seed = KeQueryPerformanceCounter(0);

		for (i=0; i< RANDOM_MOD/sizeof(ULONG); i+=sizeof(ULONG))
		{
			gRandomStore[i] =  RtlRandomEx((ULONG)seed);
		}
#endif
		gRandomReady = IMG_TRUE;
	}

	// Setup the stream for use in CB function. Use 32 bytes less to ensure we
	// don't go over a page boundary.
	eError = TLStreamCreate (&srcn->gTLStream, psIn1->pszStreamName,
			((OSGetPageSize()*psIn1->uiStreamSizeInPages)) - 32, psIn1->uiStreamCreateFlags);
	if (eError != PVRSRV_OK)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_FAILED_TO_CREATE);
	PVR_ASSERT(srcn->gTLStream);

	// Setup timer and start it
	i = (psIn1->uiStartDelay==0) ? psIn1->uiInterval : psIn1->uiStartDelay;
	srcn->gStartTimer = OSAddTimer(StartTimerFuncCB, (IMG_VOID *)srcn, i);
	if (!srcn->gStartTimer)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_ADD_TIMER);
	srcn->gStartCleanupMISR = 0;

	// Reset global count and enable timer
	srcn->gSourceTimer = 0;
	srcn->gSourceCount = 0;
	srcn->gSourceWriteCB = pfCB;
	eError = OSEnableTimer(srcn->gStartTimer);

	srcn->psNext = gpsSourceStack;
	gpsSourceStack = srcn;

	PVR_DPF_RETURN_RC(eError);
}

static PVRSRV_ERROR  TLTestCMD_SourceStop (PVR_TL_TEST_CMD_SOURCE_STOP_IN* psIn1)
{
	PVRSRV_ERROR 	eError = PVRSRV_OK;
	TLT_SRCNODE*	srcn = gpsSourceStack;
	TLT_SRCNODE**	srcn_prev = &gpsSourceStack;


	PVR_DPF_ENTERED;

	if (psIn1->pszStreamName[0] != '\0')
	{
		// Find the data source in the stack of sources
		for (; srcn != 0; srcn = srcn->psNext)
		{
			if (OSStringCompare(srcn->gsStartIn.pszStreamName, psIn1->pszStreamName) == 0) break;
			srcn_prev = &srcn->psNext;
		}
	}
	//else
	//    Select the data source at the top of the stack.

	if (!srcn)
		PVR_DPF_RETURN_RC(PVRSRV_ERROR_NOT_FOUND);

	// See if we have been asked to stop before we have started
	if (srcn->gStartTimer)
	{
		if (srcn->gStartCleanupMISR)
		{
			// Cleanup scheduled but has not started yet
			//eError = OSUninstallMISR (srcn->gStartCleanupMISR);
			// commented out as this leads to a crash, is OSUinstallMISR() broken?
			srcn->gStartCleanupMISR = 0;
			if (eError != PVRSRV_OK)
				PVR_DPF_RETURN_RC(eError);
		}

		// Stop and clean up timer
		eError = OSDisableTimer(srcn->gStartTimer);
		if (eError != PVRSRV_OK)
			PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_DISABLE_TIMER);

		eError = OSRemoveTimer(srcn->gStartTimer);
		if (eError != PVRSRV_OK)
			PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_REMOVE_TIMER);
		srcn->gStartTimer = 0;
	}

	if (srcn->gSourceTimer)
	{
		if (srcn->gSourceCleanupMISR)
		{
			// Cleanup scheduled but has not started yet
			//eError = OSUninstallMISR (srcn->gSourceCleanupMISR);
			// commented out as this leads to a crash, is OSUinstallMISR() broken?
			srcn->gSourceCleanupMISR = 0;
			if (eError != PVRSRV_OK)
				PVR_DPF_RETURN_RC(eError);
		}

		eError = OSDisableTimer(srcn->gSourceTimer);
		if (eError != PVRSRV_OK)
			PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_DISABLE_TIMER);

		eError = OSRemoveTimer(srcn->gSourceTimer);
		if (eError != PVRSRV_OK)
			PVR_DPF_RETURN_RC(PVRSRV_ERROR_UNABLE_TO_REMOVE_TIMER);
		srcn->gSourceTimer = 0;
	}

	if (srcn->gpuiDataPacket)
		OSFreeMem(srcn->gpuiDataPacket);

	// Cleanup transport stream
	TLStreamDestroy(srcn->gTLStream);
	srcn->gTLStream = 0;

	*srcn_prev = srcn->psNext;
	OSFreeMem(srcn);
	srcn = 0;

	PVR_DPF_RETURN_OK;
}

static IMG_VOID StartCB_Cleanup_MISRHandler (IMG_VOID *pvData)
{
	PVRSRV_ERROR 	eError = PVRSRV_OK;
	TLT_SRCNODE*	srcn = (TLT_SRCNODE*)pvData;

	PVR_DPF_ENTERED;

	PVR_ASSERT(srcn);
	PVR_ASSERT(srcn->gStartTimer);

	// Our job is to clean up the start timer, should only be called once
	eError = OSDisableTimer(srcn->gStartTimer);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SourceCB_Cleanup_MISRHandler() error %d", eError));
	}

	eError = OSRemoveTimer(srcn->gStartTimer);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SourceCB_Cleanup_MISRHandler() error %d", eError));
	}

	srcn->gStartTimer = 0;

	// TODO: Uninstall inside handler leads to crash.
	// Work out the best way to clean up the MISR, for now leave it
	// known to the system i.e. installed, make sure we only install it once.
	// (void) OSUninstallMISR(srcn->gStartCleanupMISR);
	// srcn->gStartCleanupMISR=0;

	PVR_DPF_RETURN;
}

static IMG_VOID SourceCB_Cleanup_MISRHandler (IMG_VOID *pvData)
{
	PVRSRV_ERROR 					eError = PVRSRV_OK;
	PVR_TL_TEST_CMD_SOURCE_STOP_IN  sStopParams;
	TLT_SRCNODE*					srcn = (TLT_SRCNODE*)pvData;

	PVR_DPF_ENTERED;

	PVR_ASSERT(srcn);

	OSStringCopy(sStopParams.pszStreamName, srcn->gsStartIn.pszStreamName);
	eError = TLTestCMD_SourceStop (&sStopParams);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "TLTestCMD_SourceStop() error %d", eError));
	}

	// TODO: Uninstall inside handler leads to crash.
	// Work out the best way to clean up the MISR, for now leave it
	// known to the system i.e. installed, make sure we only install it once.
	// (void) OSUninstallMISR(srcn->gSourceCleanupMISR);
	// srcn->gSourceCleanupMISR=0;

	PVR_DPF_RETURN;
}


// Return TRUE if the source should generate a packet, FALSE if not.
static IMG_BOOL SourceCBCommon(TLT_SRCNODE* srcn, IMG_UINT16* puiPacketSize)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;

	srcn->gSourceCount++;
	PVR_DPF((PVR_DBG_MESSAGE, "--- Data Source CB: %s #%d", srcn->gsStartIn.pszStreamName, srcn->gSourceCount));

	// Work out packet size to use...
	if (srcn->gsStartIn.uiPacketSize == 0)
	{
		// Random packet size
		*puiPacketSize = (IMG_UINT16)gRandomStore[srcn->gSourceCount%RANDOM_MOD];
	}
	else
	{
		// Fixed packet size
		*puiPacketSize = srcn->gsStartIn.uiPacketSize;
	}
	if (*puiPacketSize==0) *puiPacketSize=4;
	PVR_ASSERT(srcn->gTLStream);

	if (srcn->gsStartIn.uiCallbackKicks && (srcn->gSourceCount >= srcn->gsStartIn.uiCallbackKicks+2))
	{ // If there is a kick limit and we have supassed it, clean up

		// Kick sequence should mean that this clean is only triggered
		// on a periodic (not start) kick.
		PVR_ASSERT((srcn->gStartTimer == 0) && (srcn->gSourceTimer != 0));

		if (srcn->gSourceCleanupMISR == 0)
		{ // Only install it once as we reuse it because we can't uninstall it
		  // safely at the moment.

			eError = OSInstallMISR(&srcn->gSourceCleanupMISR, SourceCB_Cleanup_MISRHandler, srcn);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "OSInstallMISR() error %d", eError));
				return IMG_TRUE;
			}
		}
		eError = OSScheduleMISR(srcn->gSourceCleanupMISR);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "OSScheduleMISR() error %d", eError));
			return IMG_TRUE;
		}

		return IMG_FALSE;
	}
	else if (srcn->gsStartIn.uiCallbackKicks &&
			(srcn->gSourceCount > srcn->gsStartIn.uiCallbackKicks) &&
			(srcn->gSourceCount <= srcn->gsStartIn.uiCallbackKicks+2))
	{ // We have reached the callback limit, do not add data
      // Delay the stream destructions for two kicks
		return IMG_FALSE;
	}
	else
	{ //
		return IMG_TRUE;
	}
}


static IMG_VOID StartTimerFuncCB(IMG_VOID* p)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	IMG_UINT16		uiPacketSize;
	TLT_SRCNODE*	srcn = (TLT_SRCNODE*)p;

	PVR_ASSERT(srcn);

	 // We should only get CB once but if the period is short we migth get
	// invoked multiple times so silently return
	if (srcn->gStartCleanupMISR)
		return;

	// Clean up start timer, we will use the source timer for our next invoke
	// Only install cleanup MISR once as we reuse it because we can't uninstall
	// it safely at the moment.
	eError = OSInstallMISR(&srcn->gStartCleanupMISR, StartCB_Cleanup_MISRHandler, srcn);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSInstallMISR() error %d", eError));
		// No clean up not fatal so cont...
		// return;
	}
	eError = OSScheduleMISR(srcn->gStartCleanupMISR);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSScheduleMISR() error %d", eError));
		// No clean up not fatal so cont...
		// return;
	}

	// Attempt to perform our first kick, submit 1st packet
	// Has the kicks limit not been exceeded?
	if (SourceCBCommon(srcn, &uiPacketSize) == IMG_TRUE)
	{
		srcn->gSourceWriteCB(srcn, uiPacketSize);
	}

	// Always start the periodic timer as clean up happens two kicks after
	// the last data packet. This allows clients to drain the stream buffer.

	// Setup timer and start it
	srcn->gSourceTimer = OSAddTimer(SourceTimerFuncCB, (IMG_VOID *)srcn,  srcn->gsStartIn.uiInterval);
	if (!srcn->gSourceTimer)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSAddTimer() unable to add timer"));
		return;
	}

	eError = OSEnableTimer(srcn->gSourceTimer);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEnableTimer() error %d", eError));
	}

	return;
}


static IMG_VOID SourceTimerFuncCB(IMG_VOID* p)
{
	IMG_UINT16		uiPacketSize;
	TLT_SRCNODE*	srcn = (TLT_SRCNODE*)p;

	PVR_ASSERT(srcn);

	// Has the kicks limit not been exceeded?
	if (SourceCBCommon(srcn, &uiPacketSize) == IMG_TRUE)
	{
		srcn->gSourceWriteCB(srcn, uiPacketSize);
	}
}


static IMG_VOID SourceWriteFunc(IMG_VOID* p, IMG_UINT16 uiPacketSize)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	TLT_SRCNODE*    srcn = (TLT_SRCNODE*)p;

	PVR_ASSERT(srcn);

	// Commit packet into transport layer
	// Special case if it is one word in a packet, use global counter as data
	if (uiPacketSize == 4)
		eError = TLStreamWrite(srcn->gTLStream, (IMG_UINT8*)&srcn->gSourceCount, 4);
	else
		eError = TLStreamWrite(srcn->gTLStream, (IMG_UINT8*)srcn->gpuiDataPacket, uiPacketSize);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "TLStreamWrite() error %d", eError));
	}
}

static IMG_VOID SourceWriteFunc2(IMG_VOID* p, IMG_UINT16 uiPacketSize)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	TLT_SRCNODE*    srcn = (TLT_SRCNODE*)p;
	IMG_UINT32*		pBuffer;

	PVR_ASSERT(srcn);

	// Commit packet into transport layer via 2-stage API...
	eError = TLStreamReserve(srcn->gTLStream, (IMG_UINT8**) &pBuffer, uiPacketSize);
	if (eError == PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_MESSAGE, "TLStreamReserve() pBuffer %p, uiPacketSize %d bytes", pBuffer, uiPacketSize));

		OSMemCopy(pBuffer, srcn->gpuiDataPacket, uiPacketSize);

		TLStreamCommit(srcn->gTLStream, uiPacketSize);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "TLStreamReserve() error %d", eError));
	}
}

#if defined(PVRSRV_NEED_PVR_DPF)
// From services/server/env/linux/pvr_debug.c
// Externed here to allow us to update it to log "message" class trace
extern IMG_UINT32 gPVRDebugLevel;
#endif

static PVRSRV_ERROR  TLTestCMD_DebugLevel (IMG_UINT32 uiIn1, IMG_UINT32 *puiOut1)
{
	PVR_DPF_ENTERED;

#if defined(PVRSRV_NEED_PVR_DPF)
//	gPVRDebugLevel |= DBGPRIV_MESSAGE;

	*puiOut1 = gPVRDebugLevel;

	gPVRDebugLevel = uiIn1;

#endif

	PVR_DPF((PVR_DBG_WARNING, "TLTestCMD_DebugLevel: gPVRDebugLevel set to 0x%x", gPVRDebugLevel));

	PVR_DPF_RETURN_OK;
}


static PVRSRV_ERROR  TLTestCMD_DumpState (void)
{
	TL_GLOBAL_GDATA* psd = TLGGD();
	PTL_SNODE		 psn = 0;
	IMG_UINT		 count = 0;

	PVR_DPF_ENTERED;
	PVR_ASSERT(psd);

	PVR_DPF((PVR_DBG_MESSAGE, "--- TL_GLOBAL_GDATA: %p - psTlClient(%p) psHead(%p) psRgxDevNode(%p)",
			psd, psd->psTlClient, psd->psHead, psd->psRgxDevNode));

	for (psn = psd->psHead; psn; psn = psn->psNext)
	{
		count++;
		PVR_DPF((PVR_DBG_MESSAGE, "----- TL_SNODE[%d]: %p - psNext(%p) hDataEventObj(%p) psStream(%p) psDesc(%p)",
				count, psn, psn->psNext, psn->hDataEventObj, psn->psStream, psn->psDesc));

		if (psn->psStream)
		{
			PVR_DPF((PVR_DBG_MESSAGE, "------- TL_STREAM[%d]: %p - psNode(%p) szName(%s) bDrop(%d) ",
				count, psn->psStream, psn->psStream->psNode, psn->psStream->szName, psn->psStream->bDrop));
			PVR_DPF((PVR_DBG_MESSAGE, "------- TL_STREAM[%d]: %p - ui32Start(%d) ui32Count(%d) ui32CountPending(%d) ui32Size(%d) ",
				count, psn->psStream, psn->psStream->ui32Start, psn->psStream->ui32Count, psn->psStream->ui32CountPending, psn->psStream->ui32Size));
			PVR_DPF((PVR_DBG_MESSAGE, "------- TL_STREAM[%d]: %p - ui32Buffer(%p) psStreamMemDesc(%p) sExportCookie.hPMRExportHandle(%x)",
				count, psn->psStream, psn->psStream->ui32Buffer, psn->psStream->psStreamMemDesc, (IMG_UINT) psn->psStream->sExportCookie.hPMRExportHandle));
		}

		if (psn->psDesc)
		{
			PVR_DPF((PVR_DBG_MESSAGE, "------ TL_STREAM_DESC[%d]: %p - psNode(%p) ui32Flags(%x) hDataEvent(%x)",
				count, psn->psDesc, psn->psDesc->psNode, psn->psDesc->ui32Flags, (IMG_UINT) psn->psDesc->hDataEvent));
		}

	}
	PVR_DPF_RETURN_OK;
}


static PVRSRV_ERROR  TLTestCMD_DumpHWPerfState (void)
{
	PVR_DPF_RETURN_OK;
}


PVRSRV_ERROR
TLServerTestIoctlKM(IMG_UINT32  	uiCmd,
	 		IMG_PBYTE	  	uiIn1,
	 		IMG_UINT32  	uiIn2,
	   	  	IMG_UINT32*		puiOut1,
	   	  	IMG_UINT32* 	puiOut2)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_DPF_ENTERED;

	PVR_UNREFERENCED_PARAMETER(puiOut1);
	PVR_UNREFERENCED_PARAMETER(puiOut2);

	PVR_DPF((PVR_DBG_MESSAGE, "--- Processing Test IOCTL command %d", uiCmd));
	PVR_DPF((PVR_DBG_MESSAGE, "--- In Arguments: %p, %d", uiIn1, uiIn2));

	switch (uiCmd)
	{
	case PVR_TL_TEST_CMD_SOURCE_START:

		eError = TLTestCMD_SourceStart((PVR_TL_TEST_CMD_SOURCE_START_IN*)uiIn1, SourceWriteFunc);
		break;

	case PVR_TL_TEST_CMD_SOURCE_STOP:

		eError = TLTestCMD_SourceStop((PVR_TL_TEST_CMD_SOURCE_STOP_IN*)uiIn1);
		break;

	case PVR_TL_TEST_CMD_SOURCE_START2:

		eError = TLTestCMD_SourceStart((PVR_TL_TEST_CMD_SOURCE_START_IN*)uiIn1, SourceWriteFunc2);
		break;


	case PVR_TL_TEST_CMD_DEBUG_LEVEL:

		eError = TLTestCMD_DebugLevel(*(IMG_UINT32*)uiIn1, puiOut1);
		break;

	case PVR_TL_TEST_CMD_DUMP_TL_STATE:

		eError = TLTestCMD_DumpState();
		break;

	case PVR_TL_TEST_CMD_HWPERF_STATE:

		eError = TLTestCMD_DumpHWPerfState();
		break;

	default:
		// Do nothing...
		break;
	}

	PVR_DPF_RETURN_RC(eError);
}

#else

PVRSRV_ERROR
TLServerTestIoctlKM(IMG_UINT32  	uiCmd,
 			IMG_PBYTE	  	uiIn1,
	 		IMG_UINT32  	uiIn2,
	   	  	IMG_UINT32*		puiOut1,
	   	  	IMG_UINT32* 	puiOut2)
{
	PVR_DPF_ENTERED;

	PVR_UNREFERENCED_PARAMETER(uiCmd);
	PVR_UNREFERENCED_PARAMETER(uiIn1);
	PVR_UNREFERENCED_PARAMETER(uiIn2);
	PVR_UNREFERENCED_PARAMETER(puiOut1);
	PVR_UNREFERENCED_PARAMETER(puiOut2);

	PVR_DPF_RETURN_RC(PVRSRV_ERROR_NOT_SUPPORTED);
}
#endif

/******************************************************************************
 End of file (tltest.c)
******************************************************************************/

