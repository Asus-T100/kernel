									    /*************************************************************************//*!
									       @File
									       @Title           MMU PDump functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description     Common PDump (MMU specific) functions
									       @License        Strictly Confidential.
    *//***************************************************************************/

#include "pdump_mmu.h"

#if defined (PDUMP)

#include "img_types.h"
#include "pdump_km.h"
#include "pdump_osfunc.h"	/* << ugly circular dependency! FIXME FIXME */
#include "osfunc.h"
#include "pdump.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#define MAX_PDUMP_MMU_CONTEXTS	(10)
static IMG_UINT32 guiPDumpMMUContextAvailabilityMask =
    (1 << MAX_PDUMP_MMU_CONTEXTS) - 1;

/* arbitrary buffer length here.  FIXME: store this data in the MD
   rather than construct it on the fly on everyone's stacks */
#define MAX_SYMBOLIC_ADDRESS_LENGTH 40

#define MMUPT_FMT  "MMUPT_%016llX"

static PVRSRV_ERROR		/* FIXME: I wanted this to be void,
				   but I can't because of the
				   PDUMP_GET_SCRIPT_AND_FILE_STRING macro */
_ContiguousPDumpBytes(const IMG_CHAR * pszSymbolicName,
		      IMG_UINT32 ui32SymAddrOffset,
		      IMG_BOOL bFlush,
		      IMG_UINT32 uiNumBytes,
		      IMG_VOID * pvBytes, IMG_UINT32 ui32Flags)
{
	/* FIXME:  ought to have a notion of a "context" for this, perhaps */
	static const IMG_CHAR *pvBeyondLastPointer;
	static const IMG_CHAR *pvBasePointer;
	static IMG_UINT32 ui32BeyondLastOffset;
	static IMG_UINT32 ui32BaseOffset;
	//FIXME//    static const IMG_CHAR *pszLastSymbolicName;
	static IMG_UINT32 uiAccumulatedBytes = 0;
	IMG_UINT32 ui32ParamOutPos;
	PVRSRV_ERROR eErr = PVRSRV_OK;

	PDUMP_GET_SCRIPT_AND_FILE_STRING();

	if (!bFlush && uiAccumulatedBytes > 0) {
		/* do some tests for contiguity.  If it fails, we flush anyway */

		if (pvBeyondLastPointer != pvBytes ||
		    ui32SymAddrOffset != ui32BeyondLastOffset
		    /* NB: ought to check that symbolic name agrees too, but
		       we know this always to be the case in the current use-case */
		    ) {
			bFlush = IMG_TRUE;
		}
	}

	/* Flush if necessary */
	if (bFlush && uiAccumulatedBytes > 0) {
		/* This stuff hoiked from a high-level func pdump_common,
		   FIXME: put the common code in a more common place, where we
		   can use it here */

		PDumpOSCheckForSplitting(PDumpOSGetStream(PDUMP_STREAM_PARAM2),
					 uiAccumulatedBytes, ui32Flags);
		ui32ParamOutPos = PDumpOSGetStreamOffset(PDUMP_STREAM_PARAM2);

		if (PDumpOSGetParamFileNum() == 0) {
			eErr =
			    PDumpOSSprintf(pszFileName, ui32MaxLenFileName,
					   "%%0%%.prm");
		} else {
			eErr =
			    PDumpOSSprintf(pszFileName, ui32MaxLenFileName,
					   "%%0%%_%u.prm",
					   PDumpOSGetParamFileNum());
		}
		if (eErr != PVRSRV_OK) {
			goto ErrOut;
		}

		if (!PDumpOSWriteString(PDumpOSGetStream(PDUMP_STREAM_PARAM2),
					(IMG_UINT8 *) (IMG_UINTPTR_T)
					pvBasePointer, uiAccumulatedBytes,
					ui32Flags)) {
			eErr = PVRSRV_ERROR_PDUMP_BUFFER_FULL;
			goto ErrOut;
		}

		eErr = PDumpOSBufprintf(hScript, ui32MaxLenScript,
					"LDB %s:0x%X 0x%X 0x%X %s",
					/* dest */
					pszSymbolicName, ui32BaseOffset,
					/* size */
					uiAccumulatedBytes,
					/* file offset */
					ui32ParamOutPos,
					/* filename */
					pszFileName);
		if (eErr != PVRSRV_OK) {
			goto ErrOut;
		}
		PDumpOSWriteString2(hScript, ui32Flags);

		uiAccumulatedBytes = 0;
	}

	/* Initialise offsets and pointers if necessary */
	if (uiAccumulatedBytes == 0) {
		ui32BaseOffset = ui32BeyondLastOffset = ui32SymAddrOffset;
		pvBeyondLastPointer = pvBasePointer = (const IMG_CHAR *)pvBytes;
	}

	/* Accumulate some bytes */
	ui32BeyondLastOffset += uiNumBytes;
	pvBeyondLastPointer += uiNumBytes;
	uiAccumulatedBytes += uiNumBytes;

 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUMalloc
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUMalloc(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
			    IMG_DEV_PHYADDR * psDevPAddr,
			    IMG_UINT32 ui32Size, IMG_UINT32 ui32Align)
{
	PVRSRV_ERROR eErr;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;
	IMG_UINT64 ui64SymbolicAddr;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	/*
	   Write a comment to the PDump2 script streams indicating the memory allocation
	 */
	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"-- MALLOC :%s:%s Size=0x%08X Alignment=0x%08X DevPAddr=0x%08llX",
				pszPDumpDevName,
				pszTableType,
				ui32Size, ui32Align, psDevPAddr->uiAddr);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);

	/*
	   construct the symbolic address
	 */
	ui64SymbolicAddr = (IMG_UINT64) psDevPAddr->uiAddr;

	/*
	   Write to the MMU script stream indicating the memory allocation
	 */
	eErr =
	    PDumpOSBufprintf(hScript, ui32MaxLen,
			     "MALLOC :%s:" MMUPT_FMT " 0x%X 0x%X",
			     pszPDumpDevName, ui64SymbolicAddr, ui32Size,
			     ui32Align
			     /* don't need this sDevPAddr.uiAddr */ );
	if (eErr != PVRSRV_OK) {
		goto ErrUnlock;
	}
	PDumpOSWriteString2(hScript, ui32Flags);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUFree
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUFree(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
			  IMG_DEV_PHYADDR * psDevPAddr)
{
	PVRSRV_ERROR eErr;
	IMG_UINT64 ui64SymbolicAddr;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	/*
	   Write a comment to the PDUMP2 script streams indicating the memory free
	 */
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "-- FREE :%s:%s",
				pszPDumpDevName, pszTableType);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);

	/*
	   construct the symbolic address
	 */
	ui64SymbolicAddr = (IMG_UINT64) psDevPAddr->uiAddr;

	/*
	   Write to the MMU script stream indicating the memory free
	 */
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "FREE :%s:" MMUPT_FMT,
				pszPDumpDevName, ui64SymbolicAddr);
	if (eErr != PVRSRV_OK) {
		goto ErrUnlock;
	}
	PDumpOSWriteString2(hScript, ui32Flags);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUMalloc
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUMalloc2(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
			     const IMG_CHAR * pszSymbolicAddr,
			     IMG_UINT32 ui32Size, IMG_UINT32 ui32Align)
{
	PVRSRV_ERROR eErr;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	/*
	   Write a comment to the PDump2 script streams indicating the memory allocation
	 */
	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"-- MALLOC :%s:%s Size=0x%08X Alignment=0x%08X\n",
				pszPDumpDevName,
				pszTableType, ui32Size, ui32Align);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);

	/*
	   Write to the MMU script stream indicating the memory allocation
	 */
	eErr =
	    PDumpOSBufprintf(hScript, ui32MaxLen, "MALLOC :%s:%s 0x%X 0x%X\n",
			     pszPDumpDevName, pszSymbolicAddr, ui32Size,
			     ui32Align
			     /* don't need this sDevPAddr.uiAddr */ );
	if (eErr != PVRSRV_OK) {
		goto ErrUnlock;
	}
	PDumpOSWriteString2(hScript, ui32Flags);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUFree
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUFree2(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
			   const IMG_CHAR * pszSymbolicAddr)
{
	PVRSRV_ERROR eErr;
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	/*
	   Write a comment to the PDUMP2 script streams indicating the memory free
	 */
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "-- FREE :%s:%s\n",
				pszPDumpDevName, pszTableType);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);

	/*
	   Write to the MMU script stream indicating the memory free
	 */
	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "FREE :%s:%s\n",
				pszPDumpDevName, pszSymbolicAddr);
	if (eErr != PVRSRV_OK) {
		goto ErrUnlock;
	}
	PDumpOSWriteString2(hScript, ui32Flags);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUDumpPCEntries
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUDumpPCEntries(const IMG_CHAR * pszPDumpDevName,
				   IMG_VOID * pvPCMem,
				   IMG_DEV_PHYADDR sPCDevPAddr,
				   IMG_UINT32 uiFirstEntry,
				   IMG_UINT32 uiNumEntries,
				   IMG_UINT32 uiBytesPerEntry,
				   IMG_UINT32 uiPDLog2Align,
				   IMG_UINT32 uiPDAddrShift,
				   IMG_UINT64 uiPDAddrMask,
				   IMG_UINT64 uiPCEProtMask,
				   IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	IMG_UINT64 ui64PCSymAddr;
	IMG_UINT64 ui64PCEValueSymAddr;
	IMG_UINT32 ui32SymAddrOffset = 0;
	IMG_UINT32 *pui32PCMem;
	IMG_UINT64 *pui64PCMem;
	IMG_BOOL bPCEValid;
	IMG_UINT32 uiPCEIdx;
	IMG_INT32 iShiftAmount;
	IMG_CHAR *pszWrwSuffix = 0;
	IMG_VOID *pvRawBytes = 0;
	IMG_CHAR aszPCSymbolicAddr[MAX_SYMBOLIC_ADDRESS_LENGTH];
	IMG_UINT64 ui64PCE64;
	IMG_UINT64 ui64Protflags64;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	if (!PDumpOSJTInitialised()) {
		eErr = PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
		goto ErrOut;
	}

	if (PDumpOSIsSuspended()) {
		eErr = PVRSRV_OK;
		goto ErrOut;
	}

	if (pvPCMem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDUMPMMUDUMPPCENTRIES: PCMem is Null"));
		eErr = PVRSRV_ERROR_INVALID_PARAMS;
		goto ErrOut;
	}

	/*
	   create the symbolic address of the PC
	 */
	ui64PCSymAddr = sPCDevPAddr.uiAddr;

	/* FIXME: should this be in the PC MD, perhaps? */
	OSSNPrintf(aszPCSymbolicAddr,
		   MAX_SYMBOLIC_ADDRESS_LENGTH,
		   ":%s:" MMUPT_FMT, pszPDumpDevName, ui64PCSymAddr);

	PDUMP_LOCK();

	/*
	   traverse PCEs, dumping entries
	 */
	for (uiPCEIdx = uiFirstEntry;
	     uiPCEIdx < uiFirstEntry + uiNumEntries; uiPCEIdx++) {
		/* Calc the symbolic address offset of the PCE */
		ui32SymAddrOffset = (uiPCEIdx * uiBytesPerEntry);

		/* Calc the symbolic address of the PCE value and HW protflags */
#if 0				//FIXME - use callback from mmu_common.c
		sPCEValueDevPAddr = pfnGetPCEAddr(pvPCMem, uiPCEIdx);
		uiPCEProtFlags = pfnGetPCEProtFlags(pvPCMem, uiPCEIdx);
		bPCEValid = ?
#else
		/* just read it here */
		switch (uiBytesPerEntry) {
			case 4 : {
				pui32PCMem = pvPCMem;
				ui64PCE64 = pui32PCMem[uiPCEIdx];
				pszWrwSuffix = "";
				pvRawBytes = &pui32PCMem[uiPCEIdx];
				break;
			}
		case 8:
			{
				pui64PCMem = pvPCMem;
				ui64PCE64 = pui64PCMem[uiPCEIdx];
				pszWrwSuffix = "64";
				pvRawBytes = &pui64PCMem[uiPCEIdx];
				break;
			}
		default:
			{
				PVR_DPF((PVR_DBG_ERROR,
					 "PDumpMMUPCEntries: error"));
				ui64PCE64 = 0;
				//!!error
				break;
			}
		}

		ui64PCEValueSymAddr =
		    (ui64PCE64 & uiPDAddrMask) >> uiPDAddrShift <<
		    uiPDLog2Align;
		ui64Protflags64 = ui64PCE64 & uiPCEProtMask;

		bPCEValid = (ui64Protflags64 & 1) ? IMG_TRUE : IMG_FALSE;
		/* FIXME: ought to pass _actual_ value in the case where it is invalid */
#endif

		if (bPCEValid) {
			_ContiguousPDumpBytes(aszPCSymbolicAddr,
					      ui32SymAddrOffset, IMG_TRUE, 0, 0,
					      ui32Flags |
					      PDUMP_FLAGS_CONTINUOUS);

			iShiftAmount =
			    (IMG_INT32) (uiPDLog2Align - uiPDAddrShift);

			/* First put the symbolic representation of the actual
			   address of the Page Directory into a pdump internal register */
			/* MOV seemed cleaner here, since (a) it's 64-bit; (b) the
			   target is not memory.  However, MOV cannot do the
			   "reference" of the symbolic address.  Apparently WRW is
			   correct. */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"WRW :%s:$1 :%s:" MMUPT_FMT
						":0x0",
						/* dest */
						pszPDumpDevName,
						/* src */
						pszPDumpDevName,
						ui64PCEValueSymAddr);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);

			/* Now shift it to the right place, if necessary: */
			/* Now shift that value down, by the "Align shift"
			   amount, to get it into units (ought to assert that
			   we get an integer - i.e. we don't shift any bits
			   off the bottom, don't know how to do PDUMP
			   assertions yet) and then back up by the right
			   amount to get it into the position of the field.
			   This is optimised into a single shift right by the
			   difference between the two. */
			/* FIXME: find out whether SHR by 0 or -ve
			   amount is legal.  If so, we can get rid of this
			   shift */
			if (iShiftAmount > 0) {
				/* Page Directory Address is specified in units larger
				   than the position in the PCE would suggest.  */
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"SHR :%s:$1 :%s:$1 0x%X",
							/* dest */
							pszPDumpDevName,
							/* src A */
							pszPDumpDevName,
							/* src B */
							iShiftAmount);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			} else if (iShiftAmount < 0) {
				/* Page Directory Address is specified in units smaller
				   than the position in the PCE would suggest.  */
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"SHL :%s:$1 :%s:$1 0x%X",
							/* dest */
							pszPDumpDevName,
							/* src A */
							pszPDumpDevName,
							/* src B */
							-iShiftAmount);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			}

			/* Now we can "or" in the protection flags */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"OR :%s:$1 :%s:$1 0x%llX",
						/* dest */
						pszPDumpDevName,
						/* src A */
						pszPDumpDevName,
						/* src B */
						ui64Protflags64);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);

			/* Finally, we write the register into the actual PCE */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"WRW%s :%s:" MMUPT_FMT
						":0x%08X :%s:$1", pszWrwSuffix,
						/* dest */
						pszPDumpDevName,
						ui64PCSymAddr,
						ui32SymAddrOffset,
						/* src */
						pszPDumpDevName);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);
		} else {
			/* If the entry was "invalid", simply write the actual
			   value found to the memory location */
			eErr =
			    _ContiguousPDumpBytes(aszPCSymbolicAddr,
						  ui32SymAddrOffset, IMG_FALSE,
						  uiBytesPerEntry, pvRawBytes,
						  ui32Flags |
						  PDUMP_FLAGS_CONTINUOUS);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
		}
	}

	/* Flush out any partly accumulated data for the LDB */
	_ContiguousPDumpBytes(aszPCSymbolicAddr, ui32SymAddrOffset, IMG_TRUE,
			      0, 0, ui32Flags | PDUMP_FLAGS_CONTINUOUS);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUDumpPDEntries
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUDumpPDEntries(const IMG_CHAR * pszPDumpDevName,
				   IMG_VOID * pvPDMem,
				   IMG_DEV_PHYADDR sPDDevPAddr,
				   IMG_UINT32 uiFirstEntry,
				   IMG_UINT32 uiNumEntries,
				   IMG_UINT32 uiBytesPerEntry,
				   IMG_UINT32 uiPTLog2Align,
				   IMG_UINT32 uiPTAddrShift,
				   IMG_UINT64 uiPTAddrMask,
				   IMG_UINT64 uiPDEProtMask,
				   IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	IMG_UINT64 ui64PDSymAddr;
	IMG_UINT64 ui64PDEValueSymAddr;
	IMG_UINT32 ui32SymAddrOffset = 0;
	IMG_UINT32 *pui32PDMem;
	IMG_UINT64 *pui64PDMem;
	IMG_BOOL bPDEValid;
	IMG_UINT32 uiPDEIdx;
	IMG_INT32 iShiftAmount;
	IMG_CHAR *pszWrwSuffix = 0;
	IMG_VOID *pvRawBytes = 0;
	IMG_CHAR aszPDSymbolicAddr[MAX_SYMBOLIC_ADDRESS_LENGTH];
	IMG_UINT64 ui64PDE64;
	IMG_UINT64 ui64Protflags64;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	if (!PDumpOSJTInitialised()) {
		eErr = PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
		goto ErrOut;
	}

	if (PDumpOSIsSuspended()) {
		eErr = PVRSRV_OK;
		goto ErrOut;
	}

	if (pvPDMem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDUMPMMUDUMPPDENTRIES: PDMem is Null"));
		eErr = PVRSRV_ERROR_INVALID_PARAMS;
		goto ErrOut;
	}

	/*
	   create the symbolic address of the PD
	 */
	ui64PDSymAddr = sPDDevPAddr.uiAddr;

	/* FIXME: should this be in the PD MD, perhaps? */
	OSSNPrintf(aszPDSymbolicAddr,
		   MAX_SYMBOLIC_ADDRESS_LENGTH,
		   ":%s:" MMUPT_FMT, pszPDumpDevName, ui64PDSymAddr);

	PDUMP_LOCK();

	/*
	   traverse PDEs, dumping entries
	 */
	for (uiPDEIdx = uiFirstEntry;
	     uiPDEIdx < uiFirstEntry + uiNumEntries; uiPDEIdx++) {
		/* Calc the symbolic address offset of the PDE */
		ui32SymAddrOffset = (uiPDEIdx * uiBytesPerEntry);

		/* Calc the symbolic address of the PDE value and HW protflags */
#if 0				//FIXME - use callback from mmu_common.c
		sPDEValueDevPAddr = pfnGetPDEAddr(pvPDMem, uiPDEIdx);
		uiPDEProtFlags = pfnGetPDEProtFlags(pvPDMem, uiPDEIdx);
		bPDEValid = pfn ? ? ?
#else
		/* just read it here */
		switch (uiBytesPerEntry) {
			case 4 : {
				pui32PDMem = pvPDMem;
				ui64PDE64 = pui32PDMem[uiPDEIdx];
				pszWrwSuffix = "";
				pvRawBytes = &pui32PDMem[uiPDEIdx];
				break;
			}
		case 8:
			{
				pui64PDMem = pvPDMem;
				ui64PDE64 = pui64PDMem[uiPDEIdx];
				pszWrwSuffix = "64";
				pvRawBytes = &pui64PDMem[uiPDEIdx];
				break;
			}
		default:
			{
				PVR_DPF((PVR_DBG_ERROR,
					 "PDumpMMUPDEntries: error"));
				ui64PDE64 = 0;
				//!!error
				break;
			}
		}

		ui64PDEValueSymAddr =
		    (ui64PDE64 & uiPTAddrMask) >> uiPTAddrShift <<
		    uiPTLog2Align;
		ui64Protflags64 = ui64PDE64 & uiPDEProtMask;

		bPDEValid = (ui64Protflags64 & 1) ? IMG_TRUE : IMG_FALSE;
#endif

		if (bPDEValid) {
			_ContiguousPDumpBytes(aszPDSymbolicAddr,
					      ui32SymAddrOffset, IMG_TRUE, 0, 0,
					      ui32Flags |
					      PDUMP_FLAGS_CONTINUOUS);

			iShiftAmount =
			    (IMG_INT32) (uiPTLog2Align - uiPTAddrShift);

			/* First put the symbolic representation of the actual
			   address of the Page Table into a pdump internal register */
			/* MOV seemed cleaner here, since (a) it's 64-bit; (b) the
			   target is not memory.  However, MOV cannot do the
			   "reference" of the symbolic address.  Apparently WRW is
			   correct. */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"WRW :%s:$1 :%s:" MMUPT_FMT
						":0x0",
						/* dest */
						pszPDumpDevName,
						/* src */
						pszPDumpDevName,
						ui64PDEValueSymAddr);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);

			/* Now shift it to the right place, if necessary: */
			/* Now shift that value down, by the "Align shift"
			   amount, to get it into units (ought to assert that
			   we get an integer - i.e. we don't shift any bits
			   off the bottom, don't know how to do PDUMP
			   assertions yet) and then back up by the right
			   amount to get it into the position of the field.
			   This is optimised into a single shift right by the
			   difference between the two. */
			/* FIXME: find out whether SHR by 0 or -ve
			   amount is legal.  If so, we can get rid of this
			   shift */
			if (iShiftAmount > 0) {
				/* Page Table Address is specified in units larger
				   than the position in the PDE would suggest.  */
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"SHR :%s:$1 :%s:$1 0x%X",
							/* dest */
							pszPDumpDevName,
							/* src A */
							pszPDumpDevName,
							/* src B */
							iShiftAmount);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			} else if (iShiftAmount < 0) {
				/* Page Table Address is specified in units smaller
				   than the position in the PDE would suggest.  */
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"SHL :%s:$1 :%s:$1 0x%X",
							/* dest */
							pszPDumpDevName,
							/* src A */
							pszPDumpDevName,
							/* src B */
							-iShiftAmount);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			}

			/* Now we can "or" in the protection flags */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"OR :%s:$1 :%s:$1 0x%llX",
						/* dest */
						pszPDumpDevName,
						/* src A */
						pszPDumpDevName,
						/* src B */
						ui64Protflags64);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);

			/* Finally, we write the register into the actual PDE */
			eErr = PDumpOSBufprintf(hScript,
						ui32MaxLen,
						"WRW%s :%s:" MMUPT_FMT
						":0x%08X :%s:$1", pszWrwSuffix,
						/* dest */
						pszPDumpDevName,
						ui64PDSymAddr,
						ui32SymAddrOffset,
						/* src */
						pszPDumpDevName);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
			PDumpOSWriteString2(hScript,
					    ui32Flags | PDUMP_FLAGS_CONTINUOUS);
		} else {
			/* If the entry was "invalid", simply write the actual
			   value found to the memory location */
			eErr =
			    _ContiguousPDumpBytes(aszPDSymbolicAddr,
						  ui32SymAddrOffset, IMG_FALSE,
						  uiBytesPerEntry, pvRawBytes,
						  ui32Flags |
						  PDUMP_FLAGS_CONTINUOUS);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
		}
	}

	/* flush out any partly accumulated stuff for LDB */
	_ContiguousPDumpBytes(aszPDSymbolicAddr, ui32SymAddrOffset, IMG_TRUE,
			      0, 0, ui32Flags | PDUMP_FLAGS_CONTINUOUS);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUDumpPTEntries
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUDumpPTEntries(const IMG_CHAR * pszPDumpDevName, IMG_VOID * pvPTMem, IMG_DEV_PHYADDR sPTDevPAddr, IMG_UINT32 uiFirstEntry, IMG_UINT32 uiNumEntries, const IMG_CHAR * pszDPMemspaceName,	// mustn't change - hmm... perhaps valid for 1 entry only...
				   const IMG_CHAR * pszDPSymbolicAddr,	// mustn't change - hmm... perhaps valid for 1 entry only...
				   IMG_UINT64 uiDPSymbolicAddrOffset,
				   IMG_UINT32 uiBytesPerEntry,
				   IMG_UINT32 uiDPLog2Align,
				   IMG_UINT32 uiDPAddrShift,
				   IMG_UINT64 uiDPAddrMask,
				   IMG_UINT64 uiPTEProtMask,
				   IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eErr;
	IMG_UINT64 ui64PTSymAddr;
	IMG_UINT32 ui32PTESymAddrOffset = 0;	// FIXME:  PT itself ought to be a PMR.  This ought to go away!
	IMG_UINT32 *pui32PTMem;
	IMG_UINT64 *pui64PTMem;
	IMG_BOOL bPTEValid;
	IMG_UINT32 uiPTEIdx;
	IMG_INT32 iShiftAmount;
	IMG_CHAR *pszWrwSuffix = 0;
	IMG_VOID *pvRawBytes = 0;
	IMG_CHAR aszPTSymbolicAddr[MAX_SYMBOLIC_ADDRESS_LENGTH];
	IMG_UINT64 ui64PTE64;
	IMG_UINT64 ui64Protflags64;

	PDUMP_GET_SCRIPT_STRING();

	/* FIXME: we ought to change the API really.  These values are not
	   relevant for this new way of doing stuff - i.e. we should be
	   reading in the page table entry itself.  We should know it! */
	PVR_UNREFERENCED_PARAMETER(uiDPAddrShift);
	PVR_UNREFERENCED_PARAMETER(uiDPAddrMask);

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	if (!PDumpOSJTInitialised()) {
		eErr = PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
		goto ErrOut;
	}

	if (PDumpOSIsSuspended()) {
		eErr = PVRSRV_OK;
		goto ErrOut;
	}

	if (pvPTMem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDUMPMMUDUMPPTENTRIES: PTMem is Null"));
		eErr = PVRSRV_ERROR_INVALID_PARAMS;
		goto ErrOut;
	}

	/*
	   create the symbolic address of the PT
	 */
	ui64PTSymAddr = sPTDevPAddr.uiAddr;

	/* FIXME: should this be in the PT MD, perhaps? */
	OSSNPrintf(aszPTSymbolicAddr,
		   MAX_SYMBOLIC_ADDRESS_LENGTH,
		   ":%s:" MMUPT_FMT, pszPDumpDevName, ui64PTSymAddr);

	PDUMP_LOCK();

	/*
	   traverse PTEs, dumping entries
	 */
	for (uiPTEIdx = uiFirstEntry;
	     uiPTEIdx < uiFirstEntry + uiNumEntries; uiPTEIdx++) {
		/* Calc the symbolic address offset of the PTE location */
		ui32PTESymAddrOffset = (uiPTEIdx * uiBytesPerEntry);

		/* Calc the symbolic address of the PTE value and HW protflags */
#if 0				//FIXME - use callback from mmu_common.c
		sPTEValueDevPAddr = pfnGetPTEAddr(pvPTMem, uiPTEIdx);
		uiPTEProtFlags = pfnGetPTEProtFlags(pvPTMem, uiPTEIdx);
		bPTEValid = ?
#else
		/* just read it here */
		switch (uiBytesPerEntry) {
			case 4 : {
				pui32PTMem = pvPTMem;
				ui64PTE64 = pui32PTMem[uiPTEIdx];
				pszWrwSuffix = "";
				pvRawBytes = &pui32PTMem[uiPTEIdx];
				break;
			}
		case 8:
			{
				pui64PTMem = pvPTMem;
				ui64PTE64 = pui64PTMem[uiPTEIdx];
				pszWrwSuffix = "64";
				pvRawBytes = &pui64PTMem[uiPTEIdx];
				break;
			}
		default:
			{
				PVR_DPF((PVR_DBG_ERROR,
					 "PDumpMMUPTEntries: error"));
				ui64PTE64 = 0;
				//!!error
				break;
			}
		}

		ui64Protflags64 = ui64PTE64 & uiPTEProtMask;

		bPTEValid = (ui64Protflags64 & 1) ? IMG_TRUE : IMG_FALSE;
#endif

		if (bPTEValid) {
			_ContiguousPDumpBytes(aszPTSymbolicAddr,
					      ui32PTESymAddrOffset, IMG_TRUE, 0,
					      0,
					      ui32Flags |
					      PDUMP_FLAGS_CONTINUOUS);

			iShiftAmount =
			    (IMG_INT32) (uiDPLog2Align - uiDPAddrShift);

			/* First put the symbolic representation of the actual
			   address of the Data Page into a pdump internal register */
			/* MOV seemed cleaner here, since (a) it's 64-bit; (b) the
			   target is not memory.  However, MOV cannot do the
			   "reference" of the symbolic address.  Apparently WRW is
			   correct. */
			if (pszDPSymbolicAddr == IMG_NULL) {
				pszDPSymbolicAddr = "wibble";
			}

			if (iShiftAmount == 0) {
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"WRW%s :%s:" MMUPT_FMT
							":0x%08X :%s:%s:0x%llx | 0x%llX\n",
							pszWrwSuffix,
							/* dest */
							pszPDumpDevName,
							ui64PTSymAddr,
							ui32PTESymAddrOffset,
							/* src */
							pszDPMemspaceName,
							pszDPSymbolicAddr,
							uiDPSymbolicAddrOffset,
							/* ORing prot flags */
							ui64Protflags64);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			} else {

				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"WRW :%s:$1 :%s:%s:0x%llx\n",
							/* dest */
							pszPDumpDevName,
							/* src */
							pszDPMemspaceName,
							pszDPSymbolicAddr,
							uiDPSymbolicAddrOffset);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);

				/* Now shift it to the right place, if necessary: */
				/* Now shift that value down, by the "Align shift"
				   amount, to get it into units (ought to assert that
				   we get an integer - i.e. we don't shift any bits
				   off the bottom, don't know how to do PDUMP
				   assertions yet) and then back up by the right
				   amount to get it into the position of the field.
				   This is optimised into a single shift right by the
				   difference between the two. */
				/* FIXME: find out whether SHR by 0 or -ve
				   amount is legal.  If so, we can get rid of this
				   shift */
				if (iShiftAmount > 0) {
					/* Data Page Address is specified in units larger
					   than the position in the PTE would suggest.  */
					eErr = PDumpOSBufprintf(hScript,
								ui32MaxLen,
								"SHR :%s:$1 :%s:$1 0x%X",
								/* dest */
								pszPDumpDevName,
								/* src A */
								pszPDumpDevName,
								/* src B */
								iShiftAmount);
					if (eErr != PVRSRV_OK) {
						goto ErrUnlock;	/* FIXME: proper error handling */
					}
					PDumpOSWriteString2(hScript,
							    ui32Flags |
							    PDUMP_FLAGS_CONTINUOUS);
				} else if (iShiftAmount < 0) {
					/* DataPage Address is specified in units smaller
					   than the position in the PTE would suggest.  */
					eErr = PDumpOSBufprintf(hScript,
								ui32MaxLen,
								"SHL :%s:$1 :%s:$1 0x%X",
								/* dest */
								pszPDumpDevName,
								/* src A */
								pszPDumpDevName,
								/* src B */
								-iShiftAmount);
					if (eErr != PVRSRV_OK) {
						goto ErrUnlock;	/* FIXME: proper error handling */
					}
					PDumpOSWriteString2(hScript,
							    ui32Flags |
							    PDUMP_FLAGS_CONTINUOUS);
				}

				/* Finally, we write the register into the actual PTE (or'ing the prot bits) */
				eErr = PDumpOSBufprintf(hScript,
							ui32MaxLen,
							"WRW%s :%s:" MMUPT_FMT
							":0x%08X :%s:$1  | 0x%llX",
							pszWrwSuffix,
							/* dest */
							pszPDumpDevName,
							ui64PTSymAddr,
							ui32PTESymAddrOffset,
							/* src */
							pszPDumpDevName,
							/* ORing prot flags */
							ui64Protflags64);
				if (eErr != PVRSRV_OK) {
					goto ErrUnlock;	/* FIXME: proper error handling */
				}
				PDumpOSWriteString2(hScript,
						    ui32Flags |
						    PDUMP_FLAGS_CONTINUOUS);
			}
		} else {
			/* If the entry was "invalid", simply write the actual
			   value found to the memory location */
			eErr =
			    _ContiguousPDumpBytes(aszPTSymbolicAddr,
						  ui32PTESymAddrOffset,
						  IMG_FALSE, uiBytesPerEntry,
						  pvRawBytes,
						  ui32Flags |
						  PDUMP_FLAGS_CONTINUOUS);
			if (eErr != PVRSRV_OK) {
				goto ErrUnlock;	/* FIXME: proper error handling */
			}
		}
	}

	/* flush out any partly accumulated data for LDB */
	_ContiguousPDumpBytes(aszPTSymbolicAddr, ui32PTESymAddrOffset, IMG_TRUE,
			      0, 0, ui32Flags | PDUMP_FLAGS_CONTINUOUS);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return PVRSRV_OK;
}

/**************************************************************************
 * Function Name  : _PdumpAllocMMUContext
 * Inputs         : pui32MMUContextID
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : pdump util to allocate MMU contexts
**************************************************************************/
static PVRSRV_ERROR _PdumpAllocMMUContext(IMG_UINT32 * pui32MMUContextID)
{
	IMG_UINT32 i;

	/* there are MAX_PDUMP_MMU_CONTEXTS contexts available, find one */
	for (i = 0; i < MAX_PDUMP_MMU_CONTEXTS; i++) {
		if ((guiPDumpMMUContextAvailabilityMask & (1U << i))) {
			/* mark in use */
			guiPDumpMMUContextAvailabilityMask &= ~(1U << i);
			*pui32MMUContextID = i;
			return PVRSRV_OK;
		}
	}

	PVR_DPF((PVR_DBG_ERROR,
		 "_PdumpAllocMMUContext: no free MMU context ids"));

	return PVRSRV_ERROR_MMU_CONTEXT_NOT_FOUND;
}

/**************************************************************************
 * Function Name  : _PdumpFreeMMUContext
 * Inputs         : ui32MMUContextID
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : pdump util to free MMU contexts
**************************************************************************/
static PVRSRV_ERROR _PdumpFreeMMUContext(IMG_UINT32 ui32MMUContextID)
{
	if (ui32MMUContextID < MAX_PDUMP_MMU_CONTEXTS) {
		/* free the id */
		PVR_ASSERT(!
			   (guiPDumpMMUContextAvailabilityMask &
			    (1U << ui32MMUContextID)));
		guiPDumpMMUContextAvailabilityMask |= (1U << ui32MMUContextID);
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_ERROR,
		 "_PdumpFreeMMUContext: MMU context ids invalid"));

	return PVRSRV_ERROR_MMU_CONTEXT_NOT_FOUND;
}

/**************************************************************************
 * Function Name  : PDumpSetMMUContext
 * Inputs         :
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Set MMU Context
**************************************************************************/
/* FIXME: some device specific stuff here */
PVRSRV_ERROR PDumpMMUAllocMMUContext(const IMG_CHAR * pszPDumpMemSpaceName,
				     IMG_DEV_PHYADDR sPCDevPAddr,
				     IMG_UINT32 * pui32MMUContextID)
{
	IMG_UINT64 ui64PCSymAddr;

	IMG_UINT32 ui32MMUContextID;
	PVRSRV_ERROR eErr;	/* FIXME: PDump print function shouldn't return error */
	PDUMP_GET_SCRIPT_STRING();

	eErr = _PdumpAllocMMUContext(&ui32MMUContextID);
	if (eErr != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpSetMMUContext: _PdumpAllocMMUContext failed: %d",
			 eErr));
		PVR_DBG_BREAK;	// FIXME: removeme when caller can handle errors
		goto ErrOut;
	}

	/*
	   create the symbolic address of the PC
	 */
	/*
	   FIXME:  this should be part of the PC's MD, surely?
	 */
	ui64PCSymAddr = sPCDevPAddr.uiAddr;

	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen, "MMU :%s:v%d %d :%s:" MMUPT_FMT,
				/* mmu context */
				pszPDumpMemSpaceName, ui32MMUContextID,
				/* mmu type */
				6,	/* FIXME: Need to get this from devicenode somehow */
				/* PC base address */
				pszPDumpMemSpaceName, ui64PCSymAddr);
	if (eErr != PVRSRV_OK) {
		PVR_DBG_BREAK;	// FIXME: removeme when caller can handle errors
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, PDUMP_FLAGS_CONTINUOUS);
	PDUMP_UNLOCK();

	/* return the MMU Context ID */
	*pui32MMUContextID = ui32MMUContextID;

 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpClearMMUContext
 * Inputs         :
 * Outputs        : None
 * Returns        : PVRSRV_ERROR
 * Description    : Clear MMU Context
**************************************************************************/
PVRSRV_ERROR PDumpMMUFreeMMUContext(const IMG_CHAR * pszPDumpMemSpaceName,
				    IMG_UINT32 ui32MMUContextID)
{
	PVRSRV_ERROR eErr;
	PDUMP_GET_SCRIPT_STRING();

	/* FIXME: Propagate error from PDumpComment once it's supported on
	 * all OSes and platforms
	 */
	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"-- Clear MMU Context for memory space %s",
				pszPDumpMemSpaceName);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}
	PDumpOSWriteString2(hScript, PDUMP_FLAGS_CONTINUOUS);

	eErr = PDumpOSBufprintf(hScript,
				ui32MaxLen,
				"MMU :%s:v%d",
				pszPDumpMemSpaceName, ui32MMUContextID);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, PDUMP_FLAGS_CONTINUOUS);

	eErr = _PdumpFreeMMUContext(ui32MMUContextID);
	if (eErr != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpClearMMUContext: _PdumpFreeMMUContext failed: %d",
			 eErr));
		goto ErrUnlock;
	}

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/**************************************************************************
 * Function Name  : PDumpMMUActivateCatalog
 * Inputs         :
 * Outputs        : 
 * Returns        : PVRSRV_ERROR
 * Description    : 
**************************************************************************/
PVRSRV_ERROR PDumpMMUActivateCatalog(const IMG_CHAR * pszPDumpRegSpaceName,
				     const IMG_CHAR * pszPDumpRegName,
				     IMG_UINT32 uiRegAddr,
				     const IMG_CHAR * pszPDumpPCSymbolicName)
{
	IMG_UINT32 ui32Flags = PDUMP_FLAGS_CONTINUOUS;
	PVRSRV_ERROR eErr;

	PDUMP_GET_SCRIPT_STRING();

	ui32Flags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	if (!PDumpOSJTInitialised()) {
		return PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
	}

	if (PDumpOSIsSuspended()) {
		return PVRSRV_OK;
	}

	eErr = PDumpOSBufprintf(hScript, ui32MaxLen,
				"-- Write Page Catalogue Address to %s",
				pszPDumpRegName);
	if (eErr != PVRSRV_OK) {
		goto ErrOut;
	}

	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32Flags);

	eErr = PDumpOSBufprintf(hScript, ui32MaxLen, "WRW :%s:0x%04X %s:0",
				/* dest */
				pszPDumpRegSpaceName, uiRegAddr,
				/* src */
				pszPDumpPCSymbolicName);
	if (eErr != PVRSRV_OK) {
		goto ErrUnlock;	/* FIXME: proper error handling */
	}
	PDumpOSWriteString2(hScript, ui32Flags | PDUMP_FLAGS_CONTINUOUS);

 ErrUnlock:
	PDUMP_UNLOCK();
 ErrOut:
	return eErr;
}

/* FIXME: split to separate file... (not really "MMU" specific... or... is it? */
PVRSRV_ERROR
PDumpMMUSAB(const IMG_CHAR * pszPDumpMemNamespace,
	    IMG_UINT32 uiPDumpMMUCtx,
	    IMG_DEV_VIRTADDR sDevAddrStart,
	    IMG_DEVMEM_SIZE_T uiSize,
	    const IMG_CHAR * pszFilename,
	    IMG_UINT32 uiFileOffset, IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_ERROR eError;

	//                                                  "SAB :%s:v%x:0x%010llX 0x%08X 0x%08X %s.bin",

	PDUMP_GET_SCRIPT_STRING();

	ui32PDumpFlags |= (PDumpIsPersistent())? PDUMP_FLAGS_PERSISTENT : 0;

	if (!PDumpOSJTInitialised()) {
		eError = PVRSRV_ERROR_PDUMP_NOT_AVAILABLE;
		goto ErrOut;
	}

	if (PDumpOSIsSuspended()) {
		eError = PVRSRV_OK;
		goto ErrOut;
	}

	eError = PDumpOSBufprintf(hScript,
				  ui32MaxLen,
				  "SAB :%s:v%x:" IMG_DEV_VIRTADDR_FMTSPEC " "
				  IMG_DEVMEM_SIZE_FMTSPEC " "
				  "0x%x %s.bin\n",
				  pszPDumpMemNamespace,
				  uiPDumpMMUCtx,
				  sDevAddrStart.uiAddr,
				  uiSize, uiFileOffset, pszFilename);
	PVR_ASSERT(eError == PVRSRV_OK);
	PDUMP_LOCK();
	PDumpOSWriteString2(hScript, ui32PDumpFlags);
	PDUMP_UNLOCK();

 ErrOut:
	return eError;
}

#endif				/* #if defined (PDUMP) */
