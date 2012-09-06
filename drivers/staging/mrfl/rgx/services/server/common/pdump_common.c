									    /*************************************************************************//*!
									       @File
									       @Title           Common PDump functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

/* 
    This file has a mixture of top-level exported API functions, and
    low level functions used by services.  This makes enforcement of a
    strict hierarchy impossible, and nasty circular references are
    inevitable.

    TODO:

    separate out these functions into two levels.
*/

#if defined(PDUMP)
#include <stdarg.h>

#include "allocmem.h"
#include "osfunc.h"
#include "pvrsrv.h"
#include "pvr_debug.h"
#include "pdump_physmem.h"
#include "hash.h"

/* pdump headers */
#include "pdump_km.h"
#include "pdump_int.h"

/* Allow temporary buffer size override */
#if !defined(PDUMP_TEMP_BUFFER_SIZE)
#define PDUMP_TEMP_BUFFER_SIZE (64 * 1024U)
#endif

/* DEBUG */
#if 1
#define PDUMP_DBG(a)   PDumpOSDebugPrintf (a)
#else
#define PDUMP_DBG(a)
#endif

#define	PTR_PLUS(t, p, x) ((t)(((IMG_CHAR *)(p)) + (x)))
#define	VPTR_PLUS(p, x) PTR_PLUS(IMG_VOID *, p, x)
#define	VPTR_INC(p, x) ((p) = VPTR_PLUS(p, x))
#define MAX_PDUMP_MMU_CONTEXTS	(32)
static IMG_VOID *gpvTempBuffer = IMG_NULL;

#define PERSISTANT_MAGIC ((IMG_UINTPTR_T) 0xe33ee33e)

#define PDUMP_PERSISTENT_HASH_SIZE 10
static HASH_TABLE *g_psPersistentHash = IMG_NULL;

#if defined(PDUMP_DEBUG_OUTFILES)
/* counter increments each time debug write is called */
IMG_UINT32 g_ui32EveryLineCounter = 1U;
#endif

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpIsPersistent)
#endif

IMG_BOOL PDumpIsPersistent(IMG_VOID)
{
	IMG_PID uiPID = OSGetCurrentProcessIDKM();
	IMG_UINTPTR_T puiRetrieve;

	puiRetrieve = HASH_Retrieve(g_psPersistentHash, uiPID);
	if (puiRetrieve != 0) {
		PVR_ASSERT(puiRetrieve == PERSISTANT_MAGIC);
		return IMG_TRUE;
	}
	return IMG_FALSE;
}

/**************************************************************************
 * Function Name  : GetTempBuffer
 * Inputs         : None
 * Outputs        : None
 * Returns        : Temporary buffer address, or IMG_NULL
 * Description    : Get temporary buffer address.
**************************************************************************/
static IMG_VOID *GetTempBuffer(IMG_VOID)
{
	/*
	 * Allocate the temporary buffer, it it hasn't been allocated already.
	 * Return the address of the temporary buffer, or IMG_NULL if it
	 * couldn't be allocated.
	 * It is expected that the buffer will be allocated once, at driver
	 * load time, and left in place until the driver unloads.
	 */

	if (gpvTempBuffer == IMG_NULL) {
		gpvTempBuffer = OSAllocMem(PDUMP_TEMP_BUFFER_SIZE);
		if (gpvTempBuffer == IMG_NULL) {
			PVR_DPF((PVR_DBG_ERROR,
				 "GetTempBuffer: OSAllocMem failed"));
		}
	}

	return gpvTempBuffer;
}

static IMG_VOID FreeTempBuffer(IMG_VOID)
{

	if (gpvTempBuffer != IMG_NULL) {
		OSFreeMem(gpvTempBuffer);
		gpvTempBuffer = IMG_NULL;
	}
}

PVRSRV_ERROR PDumpInitCommon(IMG_VOID)
{
	PVRSRV_ERROR eError;

	/* Allocate temporary buffer for copying from user space */
	(IMG_VOID) GetTempBuffer();

	g_psPersistentHash = HASH_Create(PDUMP_PERSISTENT_HASH_SIZE);
	if (g_psPersistentHash == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* create the global PDump lock */
	eError = PDumpCreateLockKM();

	if (eError != PVRSRV_OK) {
		return eError;
	}

	/* Call environment specific PDump initialisation */
	PDumpInit();

	return PVRSRV_OK;
}

IMG_VOID PDumpDeInitCommon(IMG_VOID)
{
	/* Free temporary buffer */
	FreeTempBuffer();

	/* Call environment specific PDump Deinitialisation */
	PDumpDeInit();

	/* take down the global PDump lock */
	PDumpDestroyLockKM();
}

PVRSRV_ERROR PDumpAddPersistantProcess(IMG_VOID)
{
	IMG_PID uiPID = OSGetCurrentProcessIDKM();
	IMG_UINTPTR_T puiRetrieve;
	PVRSRV_ERROR eError = PVRSRV_OK;

	puiRetrieve = HASH_Retrieve(g_psPersistentHash, uiPID);
	if (puiRetrieve == 0) {
		if (!HASH_Insert(g_psPersistentHash, uiPID, PERSISTANT_MAGIC)) {
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		}
	} else {
		PVR_ASSERT(puiRetrieve == PERSISTANT_MAGIC);
	}

	return eError;
}

IMG_BOOL PDumpIsSuspended(IMG_VOID)
{
	return PDumpOSIsSuspended();
}

PVRSRV_ERROR PDumpIsCaptureFrameKM(IMG_BOOL * bIsCapturing)
{
#if defined(SUPPORT_PDUMP_MULTI_PROCESS)
	if (_PDumpIsProcessActive()) {
		*bIsCapturing = PDumpOSIsCaptureFrameKM();
	}
#else
	*bIsCapturing = PDumpOSIsCaptureFrameKM();
#endif

	return PVRSRV_OK;
}

PVRSRV_ERROR PDumpSetFrameKM(IMG_UINT32 ui32Frame)
{
#if defined(SUPPORT_PDUMP_MULTI_PROCESS)
	if (_PDumpIsProcessActive()) {
		return PDumpOSSetFrameKM(ui32Frame);
	}
	return PVRSRV_OK;
#else
	return PDumpOSSetFrameKM(ui32Frame);
#endif
}

/**************************************************************************
 * Function Name  : PDumpReg32
 * Inputs         : pszPDumpDevName, Register offset, and value to write
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Create a PDUMP string, which represents a register write
**************************************************************************/
PVRSRV_ERROR PDumpReg32(IMG_CHAR * pszPDumpRegName,
			IMG_UINT32 ui32Reg,
			IMG_UINT32 ui32Data, IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING()
	    PDUMP_DBG(("PDumpReg32"));

	eErr =
	    PDumpOSBufprintf(hScript, ui32MaxLen, "WRW :%s:0x%08X 0x%08X",
			     pszPDumpRegName, ui32Reg, ui32Data);

	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpReg64
 * Inputs         : pszPDumpDevName, Register offset, and value to write
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Create a PDUMP string, which represents a register write
**************************************************************************/
PVRSRV_ERROR PDumpReg64(IMG_CHAR * pszPDumpRegName,
			IMG_UINT32 ui32Reg,
			IMG_UINT64 ui64Data, IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING()
	    PDUMP_DBG(("PDumpRegKM"));

	eErr =
	    PDumpOSBufprintf(hScript, ui32MaxLen, "WRW64 :%s:0x%08X 0x%010llX",
			     pszPDumpRegName, ui32Reg, ui64Data);

	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpLDW
 * Inputs         : pcBuffer -- buffer to send to register bank
 *                  ui32NumLoadBytes -- number of bytes in pcBuffer
 *                  pszDevSpaceName -- devspace for register bank
 *                  ui32Offset -- value of offset control register
 *                  ui32PDumpFlags -- flags to pass to PDumpOSWriteString
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Dumps the contents of pcBuffer to a .prm file and
 *                  writes an LDW directive to the pdump output.
 *                  NB: ui32NumLoadBytes must be divisible by 4
**************************************************************************/
PVRSRV_ERROR PDumpLDW(IMG_CHAR * pcBuffer,
		      IMG_CHAR * pszDevSpaceName,
		      IMG_UINT32 ui32OffsetBytes,
		      IMG_UINT32 ui32NumLoadBytes, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_CHAR
	    aszParamStreamFilename[PMR_MAX_PARAMSTREAM_FILENAME_LENGTH_DEFAULT];
	IMG_UINT32 ui32ParamStreamFileOffset;

	PDUMP_GET_SCRIPT_STRING()

	    eError = PDumpWriteBuffer(pcBuffer,
				      ui32NumLoadBytes,
				      uiPDumpFlags,
				      &aszParamStreamFilename[0],
				      sizeof(aszParamStreamFilename),
				      &ui32ParamStreamFileOffset);
	PVR_ASSERT(eError == PVRSRV_OK);

	uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "LDW :%s:0x%x 0x%x 0x%x %s\n",
				  pszDevSpaceName,
				  ui32OffsetBytes,
				  ui32NumLoadBytes / sizeof(IMG_UINT32),
				  ui32ParamStreamFileOffset,
				  aszParamStreamFilename);

	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpSAW
 * Inputs         : pszDevSpaceName -- device space from which to output
 *                  ui32Offset -- offset value from register base
 *                  ui32NumSaveBytes -- number of bytes to output
 *                  pszOutfileName -- name of file to output to
 *                  ui32OutfileOffsetByte -- offset into output file to write
 *                  uiPDumpFlags -- flags to pass to PDumpOSWriteString
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Dumps the contents of a register bank into a file
 *                  NB: ui32NumSaveBytes must be divisible by 4
**************************************************************************/
PVRSRV_ERROR PDumpSAW(IMG_CHAR * pszDevSpaceName,
		      IMG_UINT32 ui32HPOffsetBytes,
		      IMG_UINT32 ui32NumSaveBytes,
		      IMG_CHAR * pszOutfileName,
		      IMG_UINT32 ui32OutfileOffsetByte,
		      PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	PDUMP_GET_SCRIPT_STRING()

	    PVR_DPF((PVR_DBG_ERROR, "PDumpSAW\n"));

	uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "SAW :%s:0x%x 0x%x 0x%x %s\n",
				  pszDevSpaceName,
				  ui32HPOffsetBytes,
				  ui32NumSaveBytes / sizeof(IMG_UINT32),
				  ui32OutfileOffsetByte, pszOutfileName);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpSAW PDumpOSBufprintf failed: eError=%u\n",
			 eError));
		return eError;
	}

	PDUMP_LOCK();
	if (!PDumpOSWriteString2(hScript, uiPDumpFlags)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpSAW PDumpOSWriteString2 failed!\n"));
	}
	PDUMP_UNLOCK();

	return PVRSRV_OK;

}

/**************************************************************************
 * Function Name  : PDumpRegPolKM
 * Inputs         : Description of what this register read is trying to do
 *					pszPDumpDevName
 *					Register offset
 *					expected value
 *					mask for that value
 * Outputs        : None
 * Returns        : None
 * Description    : Create a PDUMP string which represents a register read
 *					with the expected value
**************************************************************************/
PVRSRV_ERROR PDumpRegPolKM(IMG_CHAR * pszPDumpRegName,
			   IMG_UINT32 ui32RegAddr,
			   IMG_UINT32 ui32RegValue,
			   IMG_UINT32 ui32Mask,
			   IMG_UINT32 ui32Flags, PDUMP_POLL_OPERATOR eOperator)
{
	/* Timings correct for linux and XP */
	/* FIXME: Timings should be passed in */
#define POLL_DELAY			1000U
#define POLL_COUNT_LONG		(2000000000U / POLL_DELAY)
#define POLL_COUNT_SHORT	(1000000U / POLL_DELAY)

	PVRSRV_ERROR eErr;
	IMG_UINT32 ui32PollCount;

	PDUMP_GET_SCRIPT_STRING();
	PDUMP_DBG(("PDumpRegPolKM"));
	if (PDumpIsPersistent()) {
		/* Don't pdump-poll if the process is persistent */
		return PVRSRV_OK;
	}

	ui32PollCount = POLL_COUNT_LONG;

	eErr =
	    PDumpOSBufprintf(hScript, ui32MaxLen,
			     "POL :%s:0x%08X 0x%08X 0x%08X %d %u %d",
			     pszPDumpRegName, ui32RegAddr, ui32RegValue,
			     ui32Mask, eOperator, ui32PollCount, POLL_DELAY);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpCommentKM
 * Inputs         : pszComment, ui32Flags
 * Outputs        : None
 * Returns        : None
 * Description    : Dumps a comment
**************************************************************************/
PVRSRV_ERROR PDumpCommentKM(IMG_CHAR * pszComment, IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	IMG_CHAR pszCommentPrefix[] = "-- ";	/* prefix for comments */
#if defined(PDUMP_DEBUG_OUTFILES)
	IMG_CHAR pszTemp[256];
#endif
	IMG_UINT32 ui32LenCommentPrefix;
	PDUMP_GET_SCRIPT_STRING();
	PDUMP_DBG(("PDumpCommentKM"));
#if defined(PDUMP_DEBUG_OUTFILES)
	/* include comments in the "extended" init phase.
	 * default is to ignore them.
	 */
	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;
#endif
	/* Put line ending sequence at the end if it isn't already there */
	PDumpOSVerifyLineEnding(pszComment, ui32MaxLen);

	/* Length of string excluding terminating NULL character */
	ui32LenCommentPrefix =
	    PDumpOSBuflen(pszCommentPrefix, sizeof(pszCommentPrefix));

	PDUMP_LOCK();

	/* Ensure output file is available for writing */
	if (!PDumpOSWriteString(PDumpOSGetStream(PDUMP_STREAM_SCRIPT2),
				(IMG_UINT8 *) pszCommentPrefix,
				ui32LenCommentPrefix, ui32Flags)) {
		if (ui32Flags & PDUMP_FLAGS_CONTINUOUS) {
			eErr = PVRSRV_ERROR_PDUMP_BUFFER_FULL;
			goto ErrUnlock;
		} else {
			eErr = PVRSRV_ERROR_CMD_NOT_PROCESSED;
			goto ErrUnlock;
		}
	}
#if defined(PDUMP_DEBUG_OUTFILES)
	/* Prefix comment with PID and line number */
	eErr = PDumpOSSprintf(pszTemp, 256, "%d-%d %s",
			      _PDumpGetPID(),
			      g_ui32EveryLineCounter, pszComment);

	/* Append the comment to the script stream */
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "%s", pszTemp);
#else
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "%s", pszComment);
#endif
	if ((eErr != PVRSRV_OK) && (eErr != PVRSRV_ERROR_PDUMP_BUF_OVERFLOW)) {
		goto ErrUnlock;
	}
	PDumpOSWriteString2(hScript, ui32Flags);

 ErrUnlock:
	PDUMP_UNLOCK();
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpCommentWithFlags
 * Inputs         : psPDev - PDev for PDump device
 *				  : pszFormat - format string for comment
 *				  : ... - args for format string
 * Outputs        : None
 * Returns        : None
 * Description    : PDumps a comments
**************************************************************************/
PVRSRV_ERROR PDumpCommentWithFlags(IMG_UINT32 ui32Flags, IMG_CHAR * pszFormat,
				   ...)
{
	PVRSRV_ERROR eErr;
	PDUMP_va_list ap;
	PDUMP_GET_MSG_STRING();

	/* Construct the string */
	PDUMP_va_start(ap, pszFormat);
	eErr = PDumpOSVSprintf(pszMsg, ui32MaxLen, pszFormat, ap);
	PDUMP_va_end(ap);

	if (eErr != PVRSRV_OK) {
		return eErr;
	}
	return PDumpCommentKM(pszMsg, ui32Flags);
}

/**************************************************************************
 * Function Name  : PDumpComment
 * Inputs         : psPDev - PDev for PDump device
 *				  : pszFormat - format string for comment
 *				  : ... - args for format string
 * Outputs        : None
 * Returns        : None
 * Description    : PDumps a comments
**************************************************************************/
PVRSRV_ERROR PDumpComment(IMG_CHAR * pszFormat, ...)
{
	PVRSRV_ERROR eErr;
	PDUMP_va_list ap;
	PDUMP_GET_MSG_STRING();

	/* Construct the string */
	PDUMP_va_start(ap, pszFormat);
	eErr = PDumpOSVSprintf(pszMsg, ui32MaxLen, pszFormat, ap);
	PDUMP_va_end(ap);

	if (eErr != PVRSRV_OK) {
		return eErr;
	}
	return PDumpCommentKM(pszMsg, PDUMP_FLAGS_CONTINUOUS);
}

/*!
******************************************************************************

 @Function	PDumpBitmapKM

 @Description

 Dumps a bitmap from device memory to a file

 @Input    psDevId
 @Input    pszFileName
 @Input    ui32FileOffset
 @Input    ui32Width
 @Input    ui32Height
 @Input    ui32StrideInBytes
 @Input    sDevBaseAddr
 @Input    ui32Size
 @Input    ePixelFormat
 @Input    eMemFormat
 @Input    ui32PDumpFlags

 @Return   PVRSRV_ERROR			:

******************************************************************************/
PVRSRV_ERROR PDumpBitmapKM(PVRSRV_DEVICE_NODE * psDeviceNode,
			   IMG_CHAR * pszFileName,
			   IMG_UINT32 ui32FileOffset,
			   IMG_UINT32 ui32Width,
			   IMG_UINT32 ui32Height,
			   IMG_UINT32 ui32StrideInBytes,
			   IMG_DEV_VIRTADDR sDevBaseAddr,
			   IMG_UINT32 ui32MMUContextID,
			   IMG_UINT32 ui32Size,
			   PDUMP_PIXEL_FORMAT ePixelFormat,
			   PDUMP_MEM_FORMAT eMemFormat,
			   IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_DEVICE_IDENTIFIER *psDevId = &psDeviceNode->sDevId;
	PVRSRV_ERROR eErr = 0;
	PDUMP_GET_SCRIPT_STRING();

	if (PDumpIsPersistent()) {
		return PVRSRV_OK;
	}

	PDumpCommentWithFlags(ui32PDumpFlags, "Dump bitmap of render.");

	switch (ePixelFormat) {
	case PVRSRV_PDUMP_PIXEL_FORMAT_420PL12YUV8:	// YUV420 2 planes
		{
			const IMG_UINT32 ui32Plane0Size =
			    ui32StrideInBytes * ui32Height;
			const IMG_UINT32 ui32Plane1Size = ui32Plane0Size >> 1;	// YUV420
			const IMG_UINT32 ui32Plane1FileOffset =
			    ui32FileOffset + ui32Plane0Size;
			const IMG_UINT32 ui32Plane1MemOffset = ui32Plane0Size;

#if 1				// Remove this when the c-sim is fixed
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "YUV420 2-plane. Width=0x%08X Height=0x%08X Stride=0x%08X",
					      ui32Width, ui32Height,
					      ui32StrideInBytes);

			PDumpCommentWithFlags(ui32PDumpFlags,
					      "SII <imageset> <filename>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    :<memsp1>:v<id1>:<virtaddr1> <size1> <fileoffset1>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    :<memsp2>:v<id2>:<virtaddr2> <size2> <fileoffset2>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    <pixfmt> <width> <height> <stride> <addrmode>");
#endif

			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"SII %s %s.bin :%s:v%x:0x%010llX 0x%08X 0x%08X :%s:v%x:0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X",
						pszFileName, pszFileName,
						// Plane 0 (Y)
						psDevId->pszPDumpDevName,	// memsp
						ui32MMUContextID,	// Context id
						sDevBaseAddr.uiAddr,	// virtaddr
						ui32Plane0Size,	// size
						ui32FileOffset,	// fileoffset
						// Plane 1 (UV)
						psDevId->pszPDumpDevName,	// memsp
						ui32MMUContextID,	// Context id
						sDevBaseAddr.uiAddr + ui32Plane1MemOffset,	// virtaddr
						ui32Plane1Size,	// size
						ui32Plane1FileOffset,	// fileoffset
						ePixelFormat,
						ui32Width,
						ui32Height,
						ui32StrideInBytes, eMemFormat);

			if (eErr != PVRSRV_OK) {
				return eErr;
			}

			PDUMP_LOCK();
			PDumpOSWriteString2(hScript, ui32PDumpFlags);
			PDUMP_UNLOCK();
			break;
		}

	case PVRSRV_PDUMP_PIXEL_FORMAT_YUV_YV12:	// YUV420 3 planes
		{
			const IMG_UINT32 ui32Plane0Size =
			    ui32StrideInBytes * ui32Height;
			const IMG_UINT32 ui32Plane1Size = ui32Plane0Size >> 2;	// YUV420
			const IMG_UINT32 ui32Plane2Size = ui32Plane1Size;
			const IMG_UINT32 ui32Plane1FileOffset =
			    ui32FileOffset + ui32Plane0Size;
			const IMG_UINT32 ui32Plane2FileOffset =
			    ui32Plane1FileOffset + ui32Plane1Size;
			const IMG_UINT32 ui32Plane1MemOffset = ui32Plane0Size;
			const IMG_UINT32 ui32Plane2MemOffset =
			    ui32Plane0Size + ui32Plane1Size;

#if 1				// Remove this when the c-sim is fixed
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "YUV420 3-plane. Width=0x%08X Height=0x%08X Stride=0x%08X",
					      ui32Width, ui32Height,
					      ui32StrideInBytes);

			PDumpCommentWithFlags(ui32PDumpFlags,
					      "SII <imageset> <filename>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    :<memsp1>:v<id1>:<virtaddr1> <size1> <fileoffset1>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    :<memsp2>:v<id2>:<virtaddr2> <size2> <fileoffset2>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    :<memsp3>:v<id2>:<virtaddr3> <size3> <fileoffset3>");
			PDumpCommentWithFlags(ui32PDumpFlags,
					      "    <pixfmt> <width> <height> <stride> <addrmode>");
#endif

			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"SII %s %s.bin :%s:v%x:0x%010llX 0x%08X 0x%08X :%s:v%x:0x%010llX 0x%08X 0x%08X :%s:v%x:0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X",
						pszFileName, pszFileName,
						// Plane 0 (Y)
						psDevId->pszPDumpDevName,	// memsp
						ui32MMUContextID,	// MMU context id
						sDevBaseAddr.uiAddr,	// virtaddr
						ui32Plane0Size,	// size
						ui32FileOffset,	// fileoffset
						// Plane 1 (U)
						psDevId->pszPDumpDevName,	// memsp
						ui32MMUContextID,	// MMU context id
						sDevBaseAddr.uiAddr + ui32Plane1MemOffset,	// virtaddr
						ui32Plane1Size,	// size
						ui32Plane1FileOffset,	// fileoffset
						// Plane 2 (V)
						psDevId->pszPDumpDevName,	// memsp
						ui32MMUContextID,	// MMU context id
						sDevBaseAddr.uiAddr + ui32Plane2MemOffset,	// virtaddr
						ui32Plane2Size,	// size
						ui32Plane2FileOffset,	// fileoffset
						ePixelFormat,
						ui32Width,
						ui32Height,
						ui32StrideInBytes, eMemFormat);

			if (eErr != PVRSRV_OK) {
				return eErr;
			}

			PDUMP_LOCK();
			PDumpOSWriteString2(hScript, ui32PDumpFlags);
			PDUMP_UNLOCK();
			break;
		}

	default:		// Single plane formats
		{
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"SII %s %s.bin :%s:v%x:0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X",
						pszFileName,
						pszFileName,
						psDevId->pszPDumpDevName,
						ui32MMUContextID,
						sDevBaseAddr.uiAddr,
						ui32Size,
						ui32FileOffset,
						ePixelFormat,
						ui32Width,
						ui32Height,
						ui32StrideInBytes, eMemFormat);

			if (eErr != PVRSRV_OK) {
				return eErr;
			}

			PDUMP_LOCK();
			PDumpOSWriteString2(hScript, ui32PDumpFlags);
			PDUMP_UNLOCK();
			break;
		}
	}

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PDumpReadRegKM

 @Description

 Dumps a read from a device register to a file

 @Input    psConnection 		: connection info
 @Input    pszFileName
 @Input    ui32FileOffset
 @Input    ui32Address
 @Input    ui32Size
 @Input    ui32PDumpFlags

 @Return   PVRSRV_ERROR			:

******************************************************************************/
PVRSRV_ERROR PDumpReadRegKM(IMG_CHAR * pszPDumpRegName,
			    IMG_CHAR * pszFileName,
			    IMG_UINT32 ui32FileOffset,
			    IMG_UINT32 ui32Address,
			    IMG_UINT32 ui32Size, IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();

	PVR_UNREFERENCED_PARAMETER(ui32Size);

	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"SAB :%s:0x%08X 0x%08X %s",
				pszPDumpRegName,
				ui32Address, ui32FileOffset, pszFileName);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32PDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/*****************************************************************************
 @name		PDumpTestNextFrame
 @brief		Tests whether the next frame will be pdumped
 @param		ui32CurrentFrame
 @return	bFrameDumped
*****************************************************************************/
IMG_BOOL PDumpTestNextFrame(IMG_UINT32 ui32CurrentFrame)
{
	IMG_BOOL bFrameDumped;

	/*
	   Try dumping a string
	 */
	(IMG_VOID) PDumpSetFrameKM(ui32CurrentFrame + 1);
	PDumpIsCaptureFrameKM(&bFrameDumped);
	(IMG_VOID) PDumpSetFrameKM(ui32CurrentFrame);

	return bFrameDumped;
}

/*****************************************************************************
 @name		PDumpSignatureRegister
 @brief		Dumps a single signature register
 @param 	psDevId - device ID
 @param 	ui32Address	- The register address
 @param		ui32Size - The amount of data to be dumped in bytes
 @param		pui32FileOffset - Offset of dump in output file
 @param		ui32Flags - Flags
 @return	none
*****************************************************************************/
static PVRSRV_ERROR PDumpSignatureRegister(PVRSRV_DEVICE_IDENTIFIER * psDevId,
					   IMG_CHAR * pszFileName,
					   IMG_UINT32 ui32Address,
					   IMG_UINT32 ui32Size,
					   IMG_UINT32 * pui32FileOffset,
					   IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();

	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"SAB :%s:0x%08X 0x%08X %s",
				psDevId->pszPDumpRegName,
				ui32Address, *pui32FileOffset, pszFileName);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();
	*pui32FileOffset += ui32Size;
	return PVRSRV_OK;
}

/*****************************************************************************
 @name		PDumpRegisterRange
 @brief		Dumps a list of signature registers to a file
 @param		psDevId - device ID
 @param		pszFileName - target filename for dump
 @param		pui32Registers - register list
 @param		ui32NumRegisters - number of regs to dump
 @param		pui32FileOffset - file offset
 @param		ui32Size - size of write in bytes
 @param		ui32Flags - pdump flags
 @return	none
 *****************************************************************************/
static IMG_VOID PDumpRegisterRange(PVRSRV_DEVICE_IDENTIFIER * psDevId,
				   IMG_CHAR * pszFileName,
				   IMG_UINT32 * pui32Registers,
				   IMG_UINT32 ui32NumRegisters,
				   IMG_UINT32 * pui32FileOffset,
				   IMG_UINT32 ui32Size, IMG_UINT32 ui32Flags)
{
	IMG_UINT32 i;
	for (i = 0; i < ui32NumRegisters; i++) {
		PDumpSignatureRegister(psDevId, pszFileName, pui32Registers[i],
				       ui32Size, pui32FileOffset, ui32Flags);
	}
}

/*****************************************************************************
 @name		PDumpCounterRegisters
 @brief		Dumps the performance counters
 @param		psDevId - device id info
 @param		ui32DumpFrameNum - frame number
 @param		bLastFrame
 @param		pui32Registers - register list
 @param		ui32NumRegisters - number of regs to dump
 @return	Error
*****************************************************************************/
PVRSRV_ERROR PDumpCounterRegisters(PVRSRV_DEVICE_IDENTIFIER * psDevId,
				   IMG_UINT32 ui32DumpFrameNum,
				   IMG_BOOL bLastFrame,
				   IMG_UINT32 * pui32Registers,
				   IMG_UINT32 ui32NumRegisters)
{
	PVRSRV_ERROR eErr;
	IMG_UINT32 ui32FileOffset, ui32Flags;

	PDUMP_GET_FILE_STRING();

	ui32Flags = bLastFrame ? PDUMP_FLAGS_LASTFRAME : 0UL;
	ui32FileOffset = 0UL;

	PDumpCommentWithFlags(ui32Flags, "-- Dump counter registers");
	eErr =
	    PDumpOSSprintf(pszFileName, ui32MaxLen, "out%u.perf",
			   ui32DumpFrameNum);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}

	PDumpRegisterRange(psDevId,
			   pszFileName,
			   pui32Registers,
			   ui32NumRegisters,
			   &ui32FileOffset, sizeof(IMG_UINT32), ui32Flags);

	return PVRSRV_OK;
}

/*****************************************************************************
 @name		PDumpRegRead32
 @brief		Dump 32-bit register read to script
 @param		pszPDumpDevName - pdump device name
 @param		ui32RegOffset - register offset
 @param		ui32Flags - pdump flags
 @return	Error
*****************************************************************************/
PVRSRV_ERROR PDumpRegRead32(IMG_CHAR * pszPDumpRegName,
			    const IMG_UINT32 ui32RegOffset,
			    IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();

	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "RDW :%s:0x%X",
				pszPDumpRegName, ui32RegOffset);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();
	return PVRSRV_OK;
}

/*****************************************************************************
 @name		PDumpRegRead64
 @brief		Dump 64-bit register read to script
 @param		pszPDumpDevName - pdump device name
 @param		ui32RegOffset - register offset
 @param		ui32Flags - pdump flags
 @return	Error
*****************************************************************************/
PVRSRV_ERROR PDumpRegRead64(IMG_CHAR * pszPDumpRegName,
			    const IMG_UINT32 ui32RegOffset,
			    IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();

	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "RDW64 :%s:0x%X",
				pszPDumpRegName, ui32RegOffset);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();
	return PVRSRV_OK;
}

/*****************************************************************************
 FUNCTION	: PDumpWriteShiftedMaskedValue

 PURPOSE	: Emits the PDump commands for writing a masked shifted address
              into another location

 PARAMETERS	: PDump symbolic name and offset of target word
              PDump symbolic name and offset of source address
              right shift amount
              left shift amount
              mask

 RETURNS	: None
*****************************************************************************/
PVRSRV_ERROR
PDumpWriteShiftedMaskedValue(const IMG_CHAR * pszDestRegspaceName,
			     const IMG_CHAR * pszDestSymbolicName,
			     IMG_DEVMEM_OFFSET_T uiDestOffset,
			     const IMG_CHAR * pszRefRegspaceName,
			     const IMG_CHAR * pszRefSymbolicName,
			     IMG_DEVMEM_OFFSET_T uiRefOffset,
			     IMG_UINT32 uiSHRAmount,
			     IMG_UINT32 uiSHLAmount,
			     IMG_UINT32 uiMask,
			     IMG_DEVMEM_SIZE_T uiWordSize,
			     IMG_UINT32 uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	/* Suffix of WRW command in PDump (i.e. WRW or WRW64) */
	const IMG_CHAR *pszWrwSuffix;

	/* Internal PDump register used for interim calculation */
	const IMG_CHAR *pszPDumpIntRegSpace;
	IMG_UINT32 uiPDumpIntRegNum;

	PDUMP_GET_SCRIPT_STRING();

	if ((uiWordSize != 4) && (uiWordSize != 8)) {
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	pszWrwSuffix = (uiWordSize == 8) ? "64" : "";

	/* FIXME: "Acquire" a pdump register */
	pszPDumpIntRegSpace = pszDestRegspaceName;	/* FIXME! FIXME */
	uiPDumpIntRegNum = 1;

	eError = PDumpOSBufprintf(hScript, ui32MaxLen,
				  /* NB: should be "MOV" -- FIXME! */
				  "WRW :%s:$%d :%s:%s:"
				  IMG_DEVMEM_OFFSET_FMTSPEC "\n",
				  /* dest */
				  pszPDumpIntRegSpace, uiPDumpIntRegNum,
				  /* src */
				  pszRefRegspaceName,
				  pszRefSymbolicName, uiRefOffset);
	if (eError != PVRSRV_OK) {
		goto ErrOut;	/* FIXME: proper error handling */
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);

	if (uiSHRAmount > 0) {
		eError = PDumpOSBufprintf(hScript,
					  ui32MaxLen,
					  "SHR :%s:$%d :%s:$%d 0x%X\n",
					  /* dest */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src A */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src B */
					  uiSHRAmount);
		if (eError != PVRSRV_OK) {
			goto ErrUnlock;	/* FIXME: proper error handling */
		}
		PDumpOSWriteString2(hScript, uiPDumpFlags);
	}

	if (uiSHLAmount > 0) {
		eError = PDumpOSBufprintf(hScript,
					  ui32MaxLen,
					  "SHL :%s:$%d :%s:$%d 0x%X\n",
					  /* dest */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src A */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src B */
					  uiSHLAmount);
		if (eError != PVRSRV_OK) {
			goto ErrUnlock;	/* FIXME: proper error handling */
		}
		PDumpOSWriteString2(hScript, uiPDumpFlags);
	}

	if (uiMask != (1ULL << (8 * uiWordSize)) - 1) {
		eError = PDumpOSBufprintf(hScript,
					  ui32MaxLen,
					  "AND :%s:$%d :%s:$%d 0x%X\n",
					  /* dest */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src A */
					  pszPDumpIntRegSpace, uiPDumpIntRegNum,
					  /* src B */
					  uiMask);
		if (eError != PVRSRV_OK) {
			goto ErrUnlock;	/* FIXME: proper error handling */
		}
		PDumpOSWriteString2(hScript, uiPDumpFlags);
	}

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "WRW%s :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC
				  " :%s:$%d\n", pszWrwSuffix,
				  /* dest */
				  pszDestRegspaceName,
				  pszDestSymbolicName, uiDestOffset,
				  /* src */
				  pszPDumpIntRegSpace, uiPDumpIntRegNum);
	if (eError != PVRSRV_OK) {
		goto ErrUnlock;	/* FIXME: proper error handling */
	}
	PDumpOSWriteString2(hScript, uiPDumpFlags);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eError;
}

PVRSRV_ERROR
PDumpWriteSymbAddress(const IMG_CHAR * pszDestSpaceName,
		      IMG_DEVMEM_OFFSET_T uiDestOffset,
		      const IMG_CHAR * pszRefSymbolicName,
		      IMG_DEVMEM_OFFSET_T uiRefOffset,
		      IMG_DEVMEM_SIZE_T uiWordSize, IMG_UINT32 uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	/* Suffix of WRW command in PDump (i.e. WRW or WRW64) */
	const IMG_CHAR *pszWrwSuffix;

	PDUMP_GET_SCRIPT_STRING();

	if (uiWordSize == 4) {
		pszWrwSuffix = "";
	}

	if (uiWordSize == 8) {
		pszWrwSuffix = "64";
	}

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "WRW%s :%s:" IMG_DEVMEM_OFFSET_FMTSPEC " %s:"
				  IMG_DEVMEM_OFFSET_FMTSPEC "\n", pszWrwSuffix,
				  /* dest */
				  pszDestSpaceName, uiDestOffset,
				  /* src */
				  pszRefSymbolicName, uiRefOffset);
	if (eError != PVRSRV_OK) {
		return eError;	/* FIXME: proper error handling */
	}
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpIDLWithFlags
 * Inputs         : Idle time in clocks
 * Outputs        : None
 * Returns        : Error
 * Description    : Dump IDL command to script
**************************************************************************/
PVRSRV_ERROR PDumpIDLWithFlags(IMG_UINT32 ui32Clocks, IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();
	PDUMP_DBG(("PDumpIDLWithFlags"));

	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "IDL %u", ui32Clocks);
	if (eErr != PVRSRV_OK) {
		return eErr;
	}
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();
	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpIDL
 * Inputs         : Idle time in clocks
 * Outputs        : None
 * Returns        : Error
 * Description    : Dump IDL command to script
**************************************************************************/
PVRSRV_ERROR PDumpIDL(IMG_UINT32 ui32Clocks)
{
	return PDumpIDLWithFlags(ui32Clocks, PDUMP_FLAGS_CONTINUOUS);
}

/*****************************************************************************
 FUNCTION	: PDumpRegBasedCBP
    
 PURPOSE	: Dump CBP command to script

 PARAMETERS	:
			  
 RETURNS	: None
*****************************************************************************/
PVRSRV_ERROR PDumpRegBasedCBP(IMG_CHAR * pszPDumpRegName,
			      IMG_UINT32 ui32RegOffset,
			      IMG_UINT32 ui32WPosVal,
			      IMG_UINT32 ui32PacketSize,
			      IMG_UINT32 ui32BufferSize, IMG_UINT32 ui32Flags)
{
	PDUMP_GET_SCRIPT_STRING();

	PDumpOSBufprintf(hScript,
			 ui32MaxLen,
			 "CBP :%s:0x%08X 0x%08X 0x%08X 0x%08X",
			 pszPDumpRegName,
			 ui32RegOffset,
			 ui32WPosVal, ui32PacketSize, ui32BufferSize);
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpConnectionNotify
 * Description    : Called by the debugdrv to tell Services that pdump has
 * 					connected
 **************************************************************************/
IMG_EXPORT IMG_VOID PDumpConnectionNotify(IMG_VOID)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_DEVICE_NODE *psThis;
	PVR_DPF((PVR_DBG_WARNING, "PDump has connected."));

	/* Loop over all known devices */
	psThis = psPVRSRVData->psDeviceNodeList;
	while (psThis) {
		if (psThis->pfnPDumpInitDevice) {
			/* Reset pdump according to connected device */
			psThis->pfnPDumpInitDevice(psThis);
		}
		psThis = psThis->psNext;
	}
}

/*****************************************************************************
 * Function Name  : DbgWrite
 * Inputs         : psStream - debug stream to write to
 					pui8Data - buffer
 					ui32BCount - buffer length
 					ui32Flags - flags, e.g. continuous, LF
 * Outputs        : None
 * Returns        : Error
 * Description    : Write a block of data to a debug stream
 *****************************************************************************/
IMG_UINT32 DbgWrite(PDBG_STREAM psStream, IMG_UINT8 * pui8Data,
		    IMG_UINT32 ui32BCount, IMG_UINT32 ui32Flags)
{
	IMG_UINT32 ui32BytesWritten = 0;
	IMG_UINT32 ui32Off = 0;
	PDBG_STREAM_CONTROL psCtrl = psStream->psCtrl;

	/* Return immediately if marked as "never" */
	if ((ui32Flags & PDUMP_FLAGS_NEVER) != 0) {
		return ui32BCount;
	}
#if defined(SUPPORT_PDUMP_MULTI_PROCESS)
	/* Return if process is not marked for pdumping, unless it's persistent.
	 */
	if ((_PDumpIsProcessActive() == IMG_FALSE) &&
	    ((ui32Flags & PDUMP_FLAGS_PERSISTENT) == 0)) {
		return ui32BCount;
	}
#endif

	/* Send persistent data first ...
	 * If we're still initialising the params will be captured to the
	 * init stream in the call to pfnDBGDrivWrite2 below.
	 */
	if (((ui32Flags & PDUMP_FLAGS_PERSISTENT) != 0)
	    && (psCtrl->bInitPhaseComplete)) {
		while (ui32BCount > 0) {
			/*
			   Params marked as persistent should be appended to the init phase.
			   For example window system mem mapping of the primary surface.
			 */
			ui32BytesWritten = PDumpOSDebugDriverWrite(psStream,
								   PDUMP_WRITE_MODE_PERSISTENT,
								   &pui8Data
								   [ui32Off],
								   ui32BCount,
								   1, 0);

			if (ui32BytesWritten == 0) {
				PDumpOSReleaseExecution();
			}

			if (ui32BytesWritten != 0xFFFFFFFFU) {
				ui32Off += ui32BytesWritten;
				ui32BCount -= ui32BytesWritten;
			} else {
				PVR_DPF((PVR_DBG_ERROR,
					 "DbgWrite: Failed to send persistent data"));
				if ((psCtrl->
				     ui32Flags & DEBUG_FLAGS_READONLY) != 0) {
					/* suspend pdump to prevent flooding kernel log buffer */
					PDumpSuspendKM();
				}
				return 0xFFFFFFFFU;
			}
		}

		/* reset buffer counters */
		ui32BCount = ui32Off;
		ui32Off = 0;
		ui32BytesWritten = 0;
	}

	while (((IMG_UINT32) ui32BCount > 0)
	       && (ui32BytesWritten != 0xFFFFFFFFU)) {
		if ((ui32Flags & PDUMP_FLAGS_CONTINUOUS) != 0) {
			/*
			   If pdump client (or its equivalent) isn't running then throw continuous data away.
			 */
			if (((psCtrl->ui32CapMode & DEBUG_CAPMODE_FRAMED) != 0)
			    && (psCtrl->ui32Start == 0xFFFFFFFFU)
			    && (psCtrl->ui32End == 0xFFFFFFFFU)
			    && psCtrl->bInitPhaseComplete) {
				ui32BytesWritten = ui32BCount;
			} else {
				ui32BytesWritten =
				    PDumpOSDebugDriverWrite(psStream,
							    PDUMP_WRITE_MODE_CONTINUOUS,
							    &pui8Data[ui32Off],
							    ui32BCount, 1, 0);
			}
		} else {
			if (ui32Flags & PDUMP_FLAGS_LASTFRAME) {
				IMG_UINT32 ui32DbgFlags;

				ui32DbgFlags = 0;
				if (ui32Flags & PDUMP_FLAGS_RESETLFBUFFER) {
					ui32DbgFlags |= WRITELF_FLAGS_RESETBUF;
				}

				ui32BytesWritten =
				    PDumpOSDebugDriverWrite(psStream,
							    PDUMP_WRITE_MODE_LASTFRAME,
							    &pui8Data[ui32Off],
							    ui32BCount, 1,
							    ui32DbgFlags);
			} else {
				ui32BytesWritten =
				    PDumpOSDebugDriverWrite(psStream,
							    PDUMP_WRITE_MODE_BINCM,
							    &pui8Data[ui32Off],
							    ui32BCount, 1, 0);
			}
		}

		/*
		   If the debug driver's buffers are full so no data could be written then yield
		   execution so pdump can run and empty them.
		 */
		if (ui32BytesWritten == 0) {
			PDumpOSReleaseExecution();
		}

		if (ui32BytesWritten != 0xFFFFFFFFU) {
			ui32Off += ui32BytesWritten;
			ui32BCount -= ui32BytesWritten;
		}

		/* loop exits when i) all data is written, or ii) an unrecoverable error occurs */
	}

	/* FIXME: translate ui32BytesWritten to error here */
	return ui32BytesWritten;
}

PVRSRV_ERROR PDumpCreateLockKM(IMG_VOID)
{
	return PDumpOSCreateLock();
}

IMG_VOID PDumpDestroyLockKM(IMG_VOID)
{
	PDumpOSDestroyLock();
}

IMG_VOID PDumpLockKM(IMG_VOID)
{
	PDumpOSLock();
}

IMG_VOID PDumpUnlockKM(IMG_VOID)
{
	PDumpOSUnlock();
}

#else				/* defined(PDUMP) */
/* disable warning about empty module */
#ifdef	_WIN32
#pragma warning (disable:4206)
#endif
#endif				/* defined(PDUMP) */
/*****************************************************************************
 End of file (pdump_common.c)
*****************************************************************************/
