									    /*************************************************************************//*!
									       @File
									       @Title          pdump functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Main APIs for pdump functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _PDUMP_KM_H_
#define _PDUMP_KM_H_

/*
 * Include the OS abstraction APIs
 */
#include "pdump_osfunc.h"

/* services/srvkm/include/ */
#include "device.h"

/* include/ */
#include "pvrsrv_error.h"
#include "services.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*
 *	Pull in pdump flags from services include
 */
#include "pdump.h"

#define PDUMP_PD_UNIQUETAG			(IMG_HANDLE)0
#define PDUMP_PT_UNIQUETAG			(IMG_HANDLE)0

/*
 * PDump streams (common to all OSes)
 */
#define PDUMP_STREAM_PARAM2			0
#define PDUMP_STREAM_SCRIPT2		1
#define PDUMP_STREAM_DRIVERINFO		2
#define PDUMP_NUM_STREAMS			3

#if defined(PDUMP_DEBUG_OUTFILES)
/* counter increments each time debug write is called */
	extern IMG_UINT32 g_ui32EveryLineCounter;
#endif

#ifdef PDUMP
	/* Shared across pdump_x files */
	IMG_BOOL PDumpIsPersistent(IMG_VOID);
	PVRSRV_ERROR PDumpAddPersistantProcess(IMG_VOID);

	PVRSRV_ERROR PDumpInitCommon(IMG_VOID);
	IMG_VOID PDumpDeInitCommon(IMG_VOID);
	IMG_VOID PDumpInit(IMG_VOID);
	IMG_VOID PDumpDeInit(IMG_VOID);
	IMG_BOOL PDumpIsSuspended(IMG_VOID);
	PVRSRV_ERROR PDumpStartInitPhaseKM(IMG_VOID);
	PVRSRV_ERROR PDumpStopInitPhaseKM(IMG_MODULE_ID eModuleID);
	IMG_IMPORT PVRSRV_ERROR PDumpSetFrameKM(IMG_UINT32 ui32Frame);
	IMG_IMPORT PVRSRV_ERROR PDumpCommentKM(IMG_CHAR * pszComment,
					       IMG_UINT32 ui32Flags);

	PVRSRV_ERROR PDumpReg32(IMG_CHAR * pszPDumpRegName,
				IMG_UINT32 ui32RegAddr,
				IMG_UINT32 ui32RegValue, IMG_UINT32 ui32Flags);

	PVRSRV_ERROR PDumpReg64(IMG_CHAR * pszPDumpRegName,
				IMG_UINT32 ui32RegAddr,
				IMG_UINT64 ui64RegValue, IMG_UINT32 ui32Flags);

	PVRSRV_ERROR PDumpLDW(IMG_CHAR * pcBuffer,
			      IMG_CHAR * pszDevSpaceName,
			      IMG_UINT32 ui32OffsetBytes,
			      IMG_UINT32 ui32NumLoadBytes,
			      PDUMP_FLAGS_T uiPDumpFlags);

	PVRSRV_ERROR PDumpSAW(IMG_CHAR * pszDevSpaceName,
			      IMG_UINT32 ui32HPOffsetBytes,
			      IMG_UINT32 ui32NumSaveBytes,
			      IMG_CHAR * pszOutfileName,
			      IMG_UINT32 ui32OutfileOffsetByte,
			      PDUMP_FLAGS_T uiPDumpFlags);

	PVRSRV_ERROR PDumpRegPolKM(IMG_CHAR * pszPDumpRegName,
				   IMG_UINT32 ui32RegAddr,
				   IMG_UINT32 ui32RegValue,
				   IMG_UINT32 ui32Mask,
				   IMG_UINT32 ui32Flags,
				   PDUMP_POLL_OPERATOR eOperator);

	IMG_IMPORT PVRSRV_ERROR PDumpBitmapKM(PVRSRV_DEVICE_NODE * psDeviceNode,
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
					      IMG_UINT32 ui32PDumpFlags);

	IMG_IMPORT PVRSRV_ERROR PDumpReadRegKM(IMG_CHAR * pszPDumpRegName,
					       IMG_CHAR * pszFileName,
					       IMG_UINT32 ui32FileOffset,
					       IMG_UINT32 ui32Address,
					       IMG_UINT32 ui32Size,
					       IMG_UINT32 ui32PDumpFlags);

	PVRSRV_ERROR PDumpComment(IMG_CHAR * pszFormat,
				  ...) IMG_FORMAT_PRINTF(1, 2);
	PVRSRV_ERROR PDumpCommentWithFlags(IMG_UINT32 ui32Flags,
					   IMG_CHAR * pszFormat,
					   ...) IMG_FORMAT_PRINTF(2, 3);

	PVRSRV_ERROR PDumpPDReg(PDUMP_MMU_ATTRIB * psMMUAttrib,
				IMG_UINT32 ui32Reg,
				IMG_UINT32 ui32dwData, IMG_HANDLE hUniqueTag);
	PVRSRV_ERROR PDumpPDRegWithFlags(PDUMP_MMU_ATTRIB * psMMUAttrib,
					 IMG_UINT32 ui32Reg,
					 IMG_UINT32 ui32Data,
					 IMG_UINT32 ui32Flags,
					 IMG_HANDLE hUniqueTag);

	IMG_BOOL PDumpIsLastCaptureFrameKM(IMG_VOID);

	PVRSRV_ERROR PDumpIsCaptureFrameKM(IMG_BOOL * bIsCapturing);

	IMG_VOID PDumpMallocPagesPhys(PVRSRV_DEVICE_IDENTIFIER * psDevID,
				      IMG_UINT64 ui64DevVAddr,
				      IMG_PUINT32 pui32PhysPages,
				      IMG_UINT32 ui32NumPages,
				      IMG_HANDLE hUniqueTag);
	PVRSRV_ERROR PDumpSetMMUContext(PVRSRV_DEVICE_TYPE eDeviceType,
					IMG_CHAR * pszMemSpace,
					IMG_UINT32 * pui32MMUContextID,
					IMG_UINT32 ui32MMUType,
					IMG_HANDLE hUniqueTag1,
					IMG_HANDLE hOSMemHandle,
					IMG_VOID * pvPDCPUAddr);
	PVRSRV_ERROR PDumpClearMMUContext(PVRSRV_DEVICE_TYPE eDeviceType,
					  IMG_CHAR * pszMemSpace,
					  IMG_UINT32 ui32MMUContextID,
					  IMG_UINT32 ui32MMUType);

	IMG_BOOL PDumpTestNextFrame(IMG_UINT32 ui32CurrentFrame);

	PVRSRV_ERROR PDumpCounterRegisters(PVRSRV_DEVICE_IDENTIFIER * psDevId,
					   IMG_UINT32 ui32DumpFrameNum,
					   IMG_BOOL bLastFrame,
					   IMG_UINT32 * pui32Registers,
					   IMG_UINT32 ui32NumRegisters);

	PVRSRV_ERROR PDumpRegRead32(IMG_CHAR * pszPDumpRegName,
				    const IMG_UINT32 dwRegOffset,
				    IMG_UINT32 ui32Flags);
	PVRSRV_ERROR PDumpRegRead64(IMG_CHAR * pszPDumpRegName,
				    const IMG_UINT32 dwRegOffset,
				    IMG_UINT32 ui32Flags);

	PVRSRV_ERROR PDumpIDLWithFlags(IMG_UINT32 ui32Clocks,
				       IMG_UINT32 ui32Flags);
	PVRSRV_ERROR PDumpIDL(IMG_UINT32 ui32Clocks);

	IMG_IMPORT PVRSRV_ERROR PDumpHWPerfCBKM(PVRSRV_DEVICE_IDENTIFIER *
						psDevId, IMG_CHAR * pszFileName,
						IMG_UINT32 ui32FileOffset,
						IMG_DEV_VIRTADDR sDevBaseAddr,
						IMG_UINT32 ui32Size,
						IMG_UINT32 ui32MMUContextID,
						IMG_UINT32 ui32PDumpFlags);

	PVRSRV_ERROR PDumpRegBasedCBP(IMG_CHAR * pszPDumpRegName,
				      IMG_UINT32 ui32RegOffset,
				      IMG_UINT32 ui32WPosVal,
				      IMG_UINT32 ui32PacketSize,
				      IMG_UINT32 ui32BufferSize,
				      IMG_UINT32 ui32Flags);

	IMG_VOID PDumpSuspendKM(IMG_VOID);
	IMG_VOID PDumpResumeKM(IMG_VOID);

	PVRSRV_ERROR PDumpCreateLockKM(IMG_VOID);
	IMG_VOID PDumpDestroyLockKM(IMG_VOID);
	IMG_VOID PDumpLockKM(IMG_VOID);
	IMG_VOID PDumpUnlockKM(IMG_VOID);

	/*
	   PDumpWriteShiftedMaskedValue():

	   loads the "reference" address into an internal PDump register,
	   optionally shifts it right,
	   optionally shifts it left,
	   optionally masks it
	   then finally writes the computed value to the given destination address

	   i.e. it emits pdump language equivalent to this expression:

	   dest = ((&ref) >> SHRamount << SHLamount) & MASK
	 */
	extern PVRSRV_ERROR
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
					 IMG_UINT32 uiPDumpFlags);

	/*
	   PDumpWriteSymbAddress():

	   writes the address of the "reference" to the offset given
	 */
	extern PVRSRV_ERROR
	    PDumpWriteSymbAddress(const IMG_CHAR * pszDestSpaceName,
				  IMG_DEVMEM_OFFSET_T uiDestOffset,
				  const IMG_CHAR * pszRefSymbolicName,
				  IMG_DEVMEM_OFFSET_T uiRefOffset,
				  IMG_DEVMEM_SIZE_T uiWordSize,
				  IMG_UINT32 uiPDumpFlags);

#define PDUMP_LOCK				PDumpLockKM
#define PDUMP_UNLOCK			PDumpUnlockKM

#define PDUMPINIT				PDumpInitCommon
#define PDUMPDEINIT				PDumpDeInitCommon
#define PDUMPISLASTFRAME		PDumpIsLastCaptureFrameKM
#define PDUMPTESTNEXTFRAME		PDumpTestNextFrame
#define PDUMPREG32				PDumpReg32
#define PDUMPREG64				PDumpReg64
#define PDUMPREGREAD32			PDumpRegRead32
#define PDUMPREGREAD64			PDumpRegRead64
#define PDUMPCOMMENT			PDumpComment
#define PDUMPCOMMENTWITHFLAGS	PDumpCommentWithFlags
#define PDUMPREGPOL				PDumpRegPolKM
#define PDUMPSETMMUCONTEXT		PDumpSetMMUContext
#define PDUMPCLEARMMUCONTEXT	PDumpClearMMUContext
#define PDUMPPDREG				PDumpPDReg
#define PDUMPPDREGWITHFLAGS		PDumpPDRegWithFlags
#define PDUMPREGBASEDCBP		PDumpRegBasedCBP
#define PDUMPENDINITPHASE		PDumpStopInitPhaseKM
#define PDUMPIDLWITHFLAGS		PDumpIDLWithFlags
#define PDUMPIDL				PDumpIDL
#define PDUMPSUSPEND			PDumpSuspendKM
#define PDUMPRESUME				PDumpResumeKM
#else
	/*
	   We should be clearer about which functions can be called
	   across the bridge as this looks rather unblanced
	 */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpInitCommon)
#endif
	static INLINE PVRSRV_ERROR PDumpInitCommon(IMG_VOID) {
		return PVRSRV_OK;
	}
#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpCreateLockKM)
#endif
	static INLINE PVRSRV_ERROR PDumpCreateLockKM(IMG_VOID) {
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpDestroyLockKM)
#endif
	static INLINE IMG_VOID PDumpDestroyLockKM(IMG_VOID) {
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpLockKM)
#endif
	static INLINE IMG_VOID PDumpLockKM(IMG_VOID) {
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpUnlockKM)
#endif
	static INLINE IMG_VOID PDumpUnlockKM(IMG_VOID) {
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpAddPersistantProcess)
#endif
	static INLINE PVRSRV_ERROR PDumpAddPersistantProcess(IMG_VOID) {
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpStartInitPhaseKM)
#endif
	static INLINE PVRSRV_ERROR PDumpStartInitPhaseKM(IMG_VOID) {
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpStopInitPhaseKM)
#endif
	static INLINE PVRSRV_ERROR PDumpStopInitPhaseKM(IMG_MODULE_ID eModuleID) {
		PVR_UNREFERENCED_PARAMETER(eModuleID);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpSetFrameKM)
#endif
	static INLINE PVRSRV_ERROR PDumpSetFrameKM(IMG_UINT32 ui32Frame) {
		PVR_UNREFERENCED_PARAMETER(ui32Frame);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpCommentKM)
#endif
	static INLINE PVRSRV_ERROR
	    PDumpCommentKM(IMG_CHAR * pszComment, IMG_UINT32 ui32Flags) {
		PVR_UNREFERENCED_PARAMETER(pszComment);
		PVR_UNREFERENCED_PARAMETER(ui32Flags);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpIsLastCaptureFrameKM)
#endif
	static INLINE IMG_BOOL PDumpIsLastCaptureFrameKM(IMG_VOID) {
		return IMG_FALSE;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpIsCaptureFrameKM)
#endif
	static INLINE PVRSRV_ERROR
	    PDumpIsCaptureFrameKM(IMG_BOOL * bIsCapturing) {
		*bIsCapturing = IMG_FALSE;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PDumpBitmapKM)
#endif
	static INLINE PVRSRV_ERROR
	    PDumpBitmapKM(PVRSRV_DEVICE_NODE * psDeviceNode,
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
			  IMG_UINT32 ui32PDumpFlags) {
		PVR_UNREFERENCED_PARAMETER(psDeviceNode);
		PVR_UNREFERENCED_PARAMETER(pszFileName);
		PVR_UNREFERENCED_PARAMETER(ui32FileOffset);
		PVR_UNREFERENCED_PARAMETER(ui32Width);
		PVR_UNREFERENCED_PARAMETER(ui32Height);
		PVR_UNREFERENCED_PARAMETER(ui32StrideInBytes);
		PVR_UNREFERENCED_PARAMETER(sDevBaseAddr);
		PVR_UNREFERENCED_PARAMETER(ui32MMUContextID);
		PVR_UNREFERENCED_PARAMETER(ui32Size);
		PVR_UNREFERENCED_PARAMETER(ePixelFormat);
		PVR_UNREFERENCED_PARAMETER(eMemFormat);
		PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
		return PVRSRV_OK;
	}

#if defined WIN32 || defined UNDER_WDDM
#define PDUMPINIT			PDumpInitCommon
#define PDUMPDEINIT(...)		/ ## * PDUMPDEINIT(__VA_ARGS__) * ## /
#define PDUMPISLASTFRAME(...)		/ ## * PDUMPISLASTFRAME(__VA_ARGS__) * ## /
#define PDUMPTESTNEXTFRAME(...)		/ ## * PDUMPTESTNEXTFRAME(__VA_ARGS__) * ## /
#define PDUMPREG32(...)			/ ## * PDUMPREG32(__VA_ARGS__) * ## /
#define PDUMPREG64(...)			/ ## * PDUMPREG64(__VA_ARGS__) * ## /
#define PDUMPREGREAD32(...)			/ ## * PDUMPREGREAD32(__VA_ARGS__) * ## /
#define PDUMPREGREAD64(...)			/ ## * PDUMPREGREAD64(__VA_ARGS__) * ## /
#define PDUMPCOMMENT(...)		/ ## * PDUMPCOMMENT(__VA_ARGS__) * ## /
#define PDUMPREGPOL(...)		/ ## * PDUMPREGPOL(__VA_ARGS__) * ## /
#define PDUMPSETMMUCONTEXT(...)		/ ## * PDUMPSETMMUCONTEXT(__VA_ARGS__) * ## /
#define PDUMPCLEARMMUCONTEXT(...)	/ ## * PDUMPCLEARMMUCONTEXT(__VA_ARGS__) * ## /
#define PDUMPPDREG(...)			/ ## * PDUMPPDREG(__VA_ARGS__) * ## /
#define PDUMPPDREGWITHFLAGS(...)	/ ## * PDUMPPDREGWITHFLAGS(__VA_ARGS__) * ## /
#define PDUMPSYNC(...)			/ ## * PDUMPSYNC(__VA_ARGS__) * ## /
#define PDUMPCOPYTOMEM(...)		/ ## * PDUMPCOPYTOMEM(__VA_ARGS__) * ## /
#define PDUMPWRITE(...)			/ ## * PDUMPWRITE(__VA_ARGS__) * ## /
#define PDUMPCBP(...)			/ ## * PDUMPCBP(__VA_ARGS__) * ## /
#define	PDUMPREGBASEDCBP(...)		/ ## * PDUMPREGBASEDCBP(__VA_ARGS__) * ## /
#define PDUMPCOMMENTWITHFLAGS(...)	/ ## * PDUMPCOMMENTWITHFLAGS(__VA_ARGS__) * ## /
#define PDUMPMALLOCPAGESPHYS(...)	/ ## * PDUMPMALLOCPAGESPHYS(__VA_ARGS__) * ## /
#define PDUMPENDINITPHASE(...)		/ ## * PDUMPENDINITPHASE(__VA_ARGS__) * ## /
#define PDUMPMSVDXREG(...)		/ ## * PDUMPMSVDXREG(__VA_ARGS__) * ## /
#define PDUMPMSVDXREGWRITE(...)		/ ## * PDUMPMSVDXREGWRITE(__VA_ARGS__) * ## /
#define PDUMPMSVDXREGREAD(...)		/ ## * PDUMPMSVDXREGREAD(__VA_ARGS__) * ## /
#define PDUMPMSVDXPOLEQ(...)		/ ## * PDUMPMSVDXPOLEQ(__VA_ARGS__) * ## /
#define PDUMPMSVDXPOL(...)		/ ## * PDUMPMSVDXPOL(__VA_ARGS__) * ## /
#define PDUMPIDLWITHFLAGS(...)		/ ## * PDUMPIDLWITHFLAGS(__VA_ARGS__) * ## /
#define PDUMPIDL(...)			/ ## * PDUMPIDL(__VA_ARGS__) * ## /
#define PDUMPSUSPEND(...)		/ ## * PDUMPSUSPEND(__VA_ARGS__) * ## /
#define PDUMPRESUME(...)		/ ## * PDUMPRESUME(__VA_ARGS__) * ## /
#define PDUMP_LOCK			/ ## * PDUMP_LOCK(__VA_ARGS__) * ## /
#define PDUMP_UNLOCK			/ ## * PDUMP_UNLOCK(__VA_ARGS__) * ## /
#else
#if defined LINUX || defined GCC_IA32 || defined GCC_ARM
#define PDUMPINIT	PDumpInitCommon
#define PDUMPDEINIT(args...)
#define PDUMPISLASTFRAME(args...)
#define PDUMPTESTNEXTFRAME(args...)
#define PDUMPREG32(args...)
#define PDUMPREG64(args...)
#define PDUMPREGREAD32(args...)
#define PDUMPREGREAD64(args...)
#define PDUMPCOMMENT(args...)
#define PDUMPREGPOL(args...)
#define PDUMPSETMMUCONTEXT(args...)
#define PDUMPCLEARMMUCONTEXT(args...)
#define PDUMPPDREG(args...)
#define PDUMPPDREGWITHFLAGS(args...)
#define PDUMPSYNC(args...)
#define PDUMPCOPYTOMEM(args...)
#define PDUMPWRITE(args...)
#define PDUMPREGBASEDCBP(args...)
#define PDUMPCOMMENTWITHFLAGS(args...)
#define PDUMPENDINITPHASE(args...)
#define PDUMPIDLWITHFLAGS(args...)
#define PDUMPIDL(args...)
#define PDUMPSUSPEND(args...)
#define PDUMPRESUME(args...)
#define PDUMP_LOCK(args...)
#define PDUMP_UNLOCK(args...)

#else
#error Compiler not specified
#endif
#endif
#endif

#if defined (__cplusplus)
}
#endif

#endif				/* _PDUMP_KM_H_ */

/******************************************************************************
 End of file (pdump_km.h)
******************************************************************************/
