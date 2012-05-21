									    /*************************************************************************//*!
									       @File
									       @Title           Physmem PDump functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description     Common PDump (PMR specific) functions
									       @License        Strictly Confidential.
    *//***************************************************************************/
#include "pdump_physmem.h"

#include "img_types.h"
#include "pdumpdefs.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#include "pdump.h"

#include "pdump_km.h"
#include "pdump_osfunc.h"	// << ugly circular dependency! FIXME FIXME

#include "allocmem.h"
#include "osfunc.h"

/* #define MAX_PDUMP_MMU_CONTEXTS	(10) */
/* static IMG_UINT32 guiPDumpMMUContextAvailabilityMask = (1<<MAX_PDUMP_MMU_CONTEXTS)-1; */

/* arbitrary buffer length here.  FIXME: store this data in the MD
   rather than construct it on the fly on everyone's stacks */
#define MAX_SYMBOLIC_ADDRESS_LENGTH 40

struct _PDUMP_PHYSMEM_INFO_T_ {
	IMG_CHAR aszSymbolicAddress[MAX_SYMBOLIC_ADDRESS_LENGTH];
	IMG_UINT64 ui64Size;
	IMG_UINT32 ui32Align;
	IMG_UINT32 ui32SerialNum;
};

/**************************************************************************
 * Function Name  : PDumpPMRMalloc
 * Inputs         :
 * Outputs        :
 * Returns        : PVRSRV_ERROR
 * Description    :
**************************************************************************/
PVRSRV_ERROR PDumpPMRMalloc(const IMG_CHAR * pszDevSpace,
			    const IMG_CHAR * pszSymbolicAddress,
			    IMG_UINT64 ui64Size,
			    /* alignment is alignment of start
			       of buffer _and_ minimum
			       contiguity - i.e. smallest
			       allowable page-size.  FIXME:
			       review this decision. */
			    IMG_UINT32 ui32Align,	/* FIXME: should this be IMG_DEVMEM_ALIGN_T? */
			    IMG_BOOL bForcePersistent, IMG_HANDLE * phHandlePtr)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;

	PDUMP_PHYSMEM_INFO_T *psPDumpAllocationInfo;

	PDUMP_GET_SCRIPT_STRING()

	    psPDumpAllocationInfo = OSAllocMem(sizeof *psPDumpAllocationInfo);
	PVR_ASSERT(psPDumpAllocationInfo != IMG_NULL);	/* FIXME: handle error */

	if (bForcePersistent) {
		ui32Flags |= PDUMP_FLAGS_PERSISTENT;
	} else {
		ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;
	}

	/*
	   construct the symbolic address
	 */

	OSSNPrintf(psPDumpAllocationInfo->aszSymbolicAddress,
		   sizeof(psPDumpAllocationInfo->aszSymbolicAddress),
		   ":%s:%s", pszDevSpace, pszSymbolicAddress);

	/*
	   Write to the MMU script stream indicating the memory allocation
	 */
	eError =
	    PDumpOSBufprintf(hScript, ui32MaxLen, "MALLOC %s 0x%llX 0x%X\r\n",
			     psPDumpAllocationInfo->aszSymbolicAddress,
			     ui64Size, ui32Align);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	psPDumpAllocationInfo->ui64Size = ui64Size;
	psPDumpAllocationInfo->ui32Align = ui32Align;

	*phHandlePtr = (IMG_HANDLE) psPDumpAllocationInfo;

	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : PDumpPMRFree
 * Inputs         :
 * Outputs        :
 * Returns        : PVRSRV_ERROR
 * Description    :
**************************************************************************/
PVRSRV_ERROR PDumpPMRFree(IMG_HANDLE hPDumpAllocationInfoHandle)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;

	PDUMP_PHYSMEM_INFO_T *psPDumpAllocationInfo;

	PDUMP_GET_SCRIPT_STRING()

	    psPDumpAllocationInfo =
	    (PDUMP_PHYSMEM_INFO_T *) hPDumpAllocationInfoHandle;

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	/*
	   Write to the MMU script stream indicating the memory free
	 */
	eError = PDumpOSBufprintf(hScript, ui32MaxLen, "FREE %s\n",
				  psPDumpAllocationInfo->aszSymbolicAddress);
	if (eError != PVRSRV_OK) {
		return eError;
	}
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);
	PDUMP_UNLOCK();

	OSFreeMem(psPDumpAllocationInfo);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PDumpPMRWRW(const IMG_CHAR * pszDevSpace,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	PDUMP_GET_SCRIPT_STRING()

	    uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "WRW :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC " "
				  PMR_VALUE32_FMTSPEC " ",
				  pszDevSpace,
				  pszSymbolicName, uiOffset, ui32Value);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

PVRSRV_ERROR
PDumpPMRLDB(const IMG_CHAR * pszDevSpace,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_DEVMEM_SIZE_T uiSize,
	    const IMG_CHAR * pszFilename,
	    IMG_UINT32 uiFileOffset, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	PDUMP_GET_SCRIPT_STRING()

	    uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "LDB :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC " "
				  IMG_DEVMEM_SIZE_FMTSPEC " "
				  PDUMP_FILEOFFSET_FMTSPEC " %s\n",
				  pszDevSpace,
				  pszSymbolicName,
				  uiOffset, uiSize, uiFileOffset, pszFilename);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

PVRSRV_ERROR PDumpPMRSAB(const IMG_CHAR * pszDevSpace,
			 const IMG_CHAR * pszSymbolicName,
			 IMG_DEVMEM_OFFSET_T uiOffset,
			 IMG_DEVMEM_SIZE_T uiSize,
			 const IMG_CHAR * pszFileName, IMG_UINT32 uiFileOffset)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPDumpFlags;

	PDUMP_GET_SCRIPT_STRING()

	    uiPDumpFlags = 0;	//PDUMP_FLAGS_CONTINUOUS;
	uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "SAB :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC " "
				  IMG_DEVMEM_SIZE_FMTSPEC " "
				  "0x%08X %s.bin\n",
				  pszDevSpace,
				  pszSymbolicName,
				  uiOffset, uiSize, uiFileOffset, pszFileName);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

PVRSRV_ERROR
PDumpPMRPOL(const IMG_CHAR * pszMemspaceName,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_UINT32 ui32Value,
	    IMG_UINT32 ui32Mask,
	    PDUMP_POLL_OPERATOR eOperator,
	    IMG_UINT32 uiCount, IMG_UINT32 uiDelay, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	PDUMP_GET_SCRIPT_STRING()

	    uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "POL :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC " "
				  "0x%08X 0x%08X %d %d %d\n",
				  pszMemspaceName,
				  pszSymbolicName,
				  uiOffset,
				  ui32Value,
				  ui32Mask, eOperator, uiCount, uiDelay);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

PVRSRV_ERROR
PDumpPMRCBP(const IMG_CHAR * pszMemspaceName,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiReadOffset,
	    IMG_DEVMEM_OFFSET_T uiWriteOffset,
	    IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize)
{
	PVRSRV_ERROR eError;
	PDUMP_FLAGS_T uiPDumpFlags = 0;

	PDUMP_GET_SCRIPT_STRING()

	    uiPDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "CBP :%s:%s:" IMG_DEVMEM_OFFSET_FMTSPEC " "
				  IMG_DEVMEM_OFFSET_FMTSPEC " "
				  IMG_DEVMEM_SIZE_FMTSPEC " "
				  IMG_DEVMEM_SIZE_FMTSPEC "\n", pszMemspaceName,
				  pszSymbolicName, uiReadOffset, uiWriteOffset,
				  uiPacketSize, uiBufferSize);

	if (eError != PVRSRV_OK) {
		return eError;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, uiPDumpFlags);
	PDUMP_UNLOCK();

	return PVRSRV_OK;
}

PVRSRV_ERROR PDumpWriteBuffer( /* const */ IMG_UINT8 * pcBuffer,
			      /* FIXME:
			         pcBuffer above ought to be :
			         const IMG_UINT8 *pcBuffer
			         but, PDumpOSWriteString takes pointer to non-const data.
			       */
			      IMG_SIZE_T uiNumBytes,
			      PDUMP_FLAGS_T uiPDumpFlags,
			      IMG_CHAR * pszFilenameOut,
			      IMG_SIZE_T uiFilenameBufSz,
			      PDUMP_FILEOFFSET_T * puiOffsetOut)
{
	PVRSRV_ERROR eError;
	PDUMP_FILEOFFSET_T uiParamStreamOffset;
	IMG_BOOL bStatus;

	if (!PDumpOSJTInitialised()) {
		eError = PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
		goto e0;
	}

	PVR_ASSERT(uiNumBytes > 0);

	/* PRQA S 3415 1 *//* side effects desired */
	if (PDumpOSIsSuspended()) {
		return PVRSRV_OK;
	}

	PDUMP_LOCK();

	PDumpOSCheckForSplitting(PDumpOSGetStream(PDUMP_STREAM_PARAM2),
				 uiNumBytes, uiPDumpFlags);

	uiParamStreamOffset = PDumpOSGetStreamOffset(PDUMP_STREAM_PARAM2);

	bStatus = PDumpOSWriteString(PDumpOSGetStream(PDUMP_STREAM_PARAM2),
				     pcBuffer, uiNumBytes, uiPDumpFlags);

	PDUMP_UNLOCK();

	if (!bStatus) {
		eError = PVRSRV_ERROR_PDUMP_BUFFER_FULL;
		goto e0;
	}

	if (PDumpOSGetParamFileNum() == 0) {
		eError =
		    PDumpOSSprintf(pszFilenameOut, uiFilenameBufSz,
				   "%%0%%.prm");
		if (eError != PVRSRV_OK) {
			goto e0;
		}
	} else {
		eError =
		    PDumpOSSprintf(pszFilenameOut, uiFilenameBufSz,
				   "%%0%%_%u.prm", PDumpOSGetParamFileNum());
		if (eError != PVRSRV_OK) {
			goto e0;
		}
	}

	*puiOffsetOut = uiParamStreamOffset;

	return PVRSRV_OK;

	/*
	   error exit paths follow:
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}
