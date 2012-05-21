									    /*************************************************************************//*!
									       @File
									       @Title          Resource Allocator API
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _RA_H_
#define _RA_H_

#include "img_types.h"
#include "pvrsrv_error.h"

/** Resource arena.
 *  struct _RA_ARENA_ deliberately opaque
 */
typedef struct _RA_ARENA_ RA_ARENA;	//PRQA S 3313

/*
 * Per-Arena handle - this is private data for the caller of the RA.
 * The RA knows nothing about this data.  It is given it upon
 * RA_Create, and promises to pass it to calls to the ImportAlloc and
 * ImportFree callbacks
 */
typedef IMG_HANDLE RA_PERARENA_HANDLE;
/*
 * Per-Import handle - this is private data for the caller of the RA.
 * The RA knows nothing about this data.  It is given it on a
 * per-import basis, either the "initial" import at RA_Create time, or
 * further imports via the ImportAlloc callback.  It sends it back via
 * the ImportFree callback, and also provides it in answer to any
 * RA_Alloc request to signify from which "import" the allocation came
 */
typedef IMG_HANDLE RA_PERISPAN_HANDLE;

typedef IMG_UINT64 RA_BASE_T;
typedef IMG_UINT32 RA_LOG2QUANTUM_T;
typedef IMG_UINT64 RA_LENGTH_T;

#define RA_BASE_FMTSPEC "0x%010llx"
#define RA_ALIGN_FMTSPEC "0x%llx"
#define RA_LENGTH_FMTSPEC "0x%llx"

/*
 * Flags in an "import" must much the flags for an allocation
 */
typedef IMG_UINT32 RA_FLAGS_T;

/** Enable support for arena statistics. */
#define RA_STATS

/** Resource arena statistics. */
struct _RA_STATISTICS_ {
    /** total number of segments add to the arena */
	IMG_SIZE_T uSpanCount;

    /** number of current live segments within the arena */
	IMG_SIZE_T uLiveSegmentCount;

    /** number of current free segments within the arena */
	IMG_SIZE_T uFreeSegmentCount;

    /** total number of resource within the arena */
	IMG_SIZE_T uTotalResourceCount;

    /** number of free resource within the arena */
	IMG_SIZE_T uFreeResourceCount;

    /** total number of resources allocated from the arena */
	IMG_SIZE_T uCumulativeAllocs;

    /** total number of resources returned to the arena */
	IMG_SIZE_T uCumulativeFrees;

    /** total number of spans allocated by the callback mechanism */
	IMG_SIZE_T uImportCount;

    /** total number of spans deallocated by the callback mechanism */
	IMG_SIZE_T uExportCount;
};
typedef struct _RA_STATISTICS_ RA_STATISTICS;

struct _RA_SEGMENT_DETAILS_ {
	RA_LENGTH_T uiSize;
	IMG_CPU_PHYADDR sCpuPhyAddr;
	IMG_HANDLE hSegment;
};
typedef struct _RA_SEGMENT_DETAILS_ RA_SEGMENT_DETAILS;

/**
 *  @Function   RA_Create
 *
 *  @Description
 *
 *  To create a resource arena.
 *
 *  @Input name - the name of the arena for diagnostic purposes.
 *  @Input base - the base of an initial resource span or 0.
 *  @Input uSize - the size of an initial resource span or 0.
 *  @Input pRef - the reference to return for the initial resource or 0.
 *  @Input uQuantum - the arena allocation quantum.
 *  @Input alloc - a resource allocation callback or 0.
 *  @Input free - a resource de-allocation callback or 0.
 *  @Input import_handle - handle passed to alloc and free or 0.
 *  @Return arena handle, or IMG_NULL.
 */
RA_ARENA *RA_Create(IMG_CHAR * name,
		    /* "initial" import */
		    RA_BASE_T base,
		    RA_LENGTH_T uSize, RA_FLAGS_T uFlags, IMG_HANDLE hPriv,
		    /* subsequent imports: */
		    RA_LOG2QUANTUM_T uLog2Quantum,
		    IMG_BOOL(*imp_alloc) (RA_PERARENA_HANDLE _h,
					  RA_LENGTH_T uSize,
					  RA_FLAGS_T uFlags,
					  RA_BASE_T * pBase,
					  RA_LENGTH_T * pActualSize,
					  RA_PERISPAN_HANDLE * phPriv),
		    IMG_VOID(*imp_free) (RA_PERARENA_HANDLE,
					 RA_BASE_T,
					 RA_PERISPAN_HANDLE),
		    RA_PERARENA_HANDLE import_handle);

/**
 *  @Function   RA_Delete
 *
 *  @Description
 *
 *  To delete a resource arena. All resources allocated from the arena
 *  must be freed before deleting the arena.
 *                  
 *  @Input  pArena - the arena to delete.
 *  @Return None
 */
IMG_VOID RA_Delete(RA_ARENA * pArena);

/**
 *  @Function   RA_Add
 *
 *  @Description
 *
 *  To add a resource span to an arena. The span must not overlap with
 *  any span previously added to the arena.
 *
 *  @Input pArena - the arena to add a span into.
 *  @Input base - the base of the span.
 *  @Input uSize - the extent of the span.
 *  @Return IMG_TRUE - success, IMG_FALSE - failure
 */
IMG_BOOL
RA_Add(RA_ARENA * pArena, RA_BASE_T base, RA_LENGTH_T uSize, RA_FLAGS_T uFlags);

/**
 *  @Function   RA_Alloc
 *
 *  @Description
 *
 *  To allocate resource from an arena.
 *
 *  @Input  pArena - the arena
 *  @Input  uRequestSize - the size of resource segment requested.
 *  @Output pActualSize - the actual_size of resource segment allocated,
 *          typcially rounded up by quantum.
 *  @Input  uFlags - flags influencing allocation policy.
 *  @Input  uAlignment - the alignment constraint required for the
 *          allocated segment, use 0 if alignment not required.
 *  @Output pBase - allocated base resource
 *  @Output phPriv - the user reference associated with allocated
 *          resource span.
 *  @Return IMG_TRUE - success, IMG_FALSE - failure
 */
IMG_BOOL
RA_Alloc(RA_ARENA * pArena,
	 RA_LENGTH_T uSize,
	 RA_FLAGS_T uFlags,
	 RA_LENGTH_T uAlignment,
	 RA_BASE_T * pBase,
	 RA_LENGTH_T * pActualSize, RA_PERISPAN_HANDLE * phPriv);

/**
 *  @Function   RA_Free
 *
 *  @Description    To free a resource segment.
 *  
 *  @Input  pArena - the arena the segment was originally allocated from.
 *  @Input  base - the base of the resource span to free.
 *	@Input	bFreeBackingStore - Should backing store memory be freed?
 *
 *  @Return None
 */
IMG_VOID RA_Free(RA_ARENA * pArena, RA_BASE_T base);

#ifdef RA_STATS

#define CHECK_SPACE(total)					\
{											\
	if((total)<100) 							\
		return PVRSRV_ERROR_INVALID_PARAMS;	\
}

#define UPDATE_SPACE(str, count, total)		\
{											\
	/* FIXME sign casts should not be required - fix the type */ \
	if((count) == (IMG_INT32)-1)			\
		return PVRSRV_ERROR_INVALID_PARAMS;	\
	else									\
	{										\
		(str) += (count);					\
		(total) -= (IMG_UINT32)(count);		\
	}										\
}

/**
 *  @Function   RA_GetStats
 *
 *  @Description    gets stats on a given arena
 *  
 *  @Input  pArena - the arena the segment was originally allocated from.
 *  @Input  ppszStr - string to write stats to 
 *	@Input	pui32StrLen - length of string
 *
 *  @Return PVRSRV_ERROR
 */
PVRSRV_ERROR RA_GetStats(RA_ARENA * pArena,
			 IMG_CHAR ** ppszStr, IMG_UINT32 * pui32StrLen);

PVRSRV_ERROR RA_GetStatsFreeMem(RA_ARENA * pArena,
				IMG_CHAR ** ppszStr, IMG_UINT32 * pui32StrLen);

#endif				/* #ifdef RA_STATS */

#endif
