									    /*************************************************************************//*!
									       @File
									       @Title          RGX firmware interface structures
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX firmware interface structures shared by both host client
									       and host server
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__RGX_FWIF_SHARED_H__)
#define __RGX_FWIF_SHARED_H__

#include "img_types.h"
#include "rgxdefs.h"

/*!
 ******************************************************************************
 * RGXFW Compiler alignment definitions
 *****************************************************************************/
#if defined(__GNUC__)
#define RGXFW_ALIGN			__attribute__ ((aligned (8)))
#elif defined(__MECC__)
#define RGXFW_ALIGN			_Pragma("align 8")
#elif defined(_MSC_VER)
#define RGXFW_ALIGN			__declspec(align(8))
#else
#error "Align MACROS need to be defined for this compiler"
#endif

/* Required memory alignment for 64-bit variables accessible by Meta 
  (the gcc meta aligns 64-bit vars to 64-bit; therefore, mem shared between
   the host and meta that contains 64-bit vars has to maintain this aligment)*/
#define RGXFWIF_FWALLOC_ALIGN	sizeof(IMG_UINT64)

typedef struct _RGXFWIF_DEV_VIRTADDR_ {
	IMG_UINT32 ui32Addr;
} RGXFWIF_DEV_VIRTADDR;

typedef IMG_UINT8 RGXFWIF_CCCB;

#if defined(RGX_FIRMWARE)
/* Compiling the actual firmware - use a fully typed pointer */
typedef RGXFWIF_CCCB *PRGXFWIF_CCCB;
typedef struct _RGXFWIF_CCCB_CTL_ *PRGXFWIF_CCCB_CTL;
typedef struct _RGXFWIF_RENDER_TARGET_ *PRGXFWIF_RENDER_TARGET;
typedef struct _RGXFWIF_HWRTDATA_ *PRGXFWIF_HWRTDATA;
typedef struct _RGXFWIF_FREELIST_ *PRGXFWIF_FREELIST;
#else
/* Compiling the host driver - use a firmware device virtual pointer */
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_CCCB;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_CCCB_CTL;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_RENDER_TARGET;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_HWRTDATA;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FREELIST;
#endif				/* RGX_FIRMWARE */

/* FIXME these should move back into rgx_fwif_client.h */
typedef IMG_UINT32 *PRGXFWIF_UFO_ADDR;

typedef struct _RGXFWIF_UFO_ {
	PRGXFWIF_UFO_ADDR puiAddrUFO;
	IMG_UINT32 ui32Value;
} RGXFWIF_UFO;

/*!
	Cleanup state: states that a FWComCtxt goes through before cleanup
*/
typedef enum _RGXFWIF_CLEANUP_STATE_ {
	RGXFWIF_CLEANUP_NONE = 0,
	RGXFWIF_CLEANUP_BG_NEEDED,
	RGXFWIF_CLEANUP_BG_DONE,
	RGXFWIF_CLEANUP_IRQ_NEEDED,
	RGXFWIF_CLEANUP_FINISHED
} RGXFWIF_CLEANUP_STATE;

typedef struct _RGXFWIF_CLEANUP_CTL_ {
	RGXFWIF_CLEANUP_STATE eCleanupState;
	IMG_UINT32 ui32SubmittedCommands;	/*!< Number of commands submitted to the FW */
	IMG_UINT32 ui32ExecutedCommands;	/*!< Number of commands executed by the FW */
	IMG_UINT32 ui32SyncObjDevVAddr;	/*!< SyncPrimitive to update after cleanup completion */
	IMG_UINT32 ui32UpdateVal;	/*!< SyncPrimitive to update after cleanup completion */

} RGXFWIF_CLEANUP_CTL;

/*!
 ******************************************************************************
 * Client CCB control for RGX
 *****************************************************************************/
typedef struct _RGXFWIF_CCCB_CTL_ {
	IMG_UINT32 ui32WriteOffset;	/*!< write offset into array of commands (MUST be aligned to 16 bytes!) */
	IMG_UINT32 ui32ReadOffset;	/*!< read offset into array of commands */
	IMG_UINT32 ui32DepOffset;	/*!< Dependency offset */
	IMG_UINT32 ui32WrapMask;	/*!< Offset wrapping mask (Total capacity of the CCB - 1) */
} RGXFWIF_CCCB_CTL;

/* TODO: Use enums */
#define RGX_NUM_FREELIST_TYPES		(5)
#define RGX_PAGE_TABLE_FREELIST		(0)
#define RGX_PER_RENDER_FREELIST		(1)
#define RGX_PER_RT_FREELIST			(2)
#define RGX_PER_CONTEXT_FREELIST	(3)
#define RGX_SHARED_FREELIST			(4)

typedef struct _RGXFWIF_FREELIST_ {
	IMG_DEV_VIRTADDR RGXFW_ALIGN psFreeListDevVAddr;
	IMG_UINT64 RGXFW_ALIGN ui64CurrentStackTop;
	IMG_UINT32 ui32TotalPMPages;
	IMG_UINT32 ui32AllocatedPageCount;
	IMG_UINT32 ui32AllocatedMMUPageCount;
	IMG_UINT32 ui32State;
	IMG_UINT32 ui32HWRCounter;
} RGXFWIF_FREELIST;

typedef struct _RGXFWIF_RENDER_TARGET_ {
	IMG_DEV_VIRTADDR RGXFW_ALIGN psVHeapTableDevVAddr;	/*!< VHeap Data Store */

} RGXFWIF_RENDER_TARGET;

typedef struct _RGXFWIF_HWRTDATA_ {
	IMG_DEV_VIRTADDR RGXFW_ALIGN psPMMListDevVAddr;	/*!< MList Data Store */
	PRGXFWIF_FREELIST RGXFW_ALIGN apsFreeLists[RGX_NUM_FREELIST_TYPES];
	IMG_UINT64 RGXFW_ALIGN ui64VCECatBase;
	IMG_UINT64 RGXFW_ALIGN ui64TECatBase;
	IMG_UINT64 RGXFW_ALIGN ui64AlistCatBase;

	IMG_UINT64 RGXFW_ALIGN ui64PMAListStackPointer;

	IMG_UINT32 ui32PMMListStackPointer;

	IMG_UINT32 aui32FreeListHWRSnapshot[RGX_NUM_FREELIST_TYPES];

	PRGXFWIF_RENDER_TARGET psParentRenderTarget;
	IMG_UINT32 ui32State;

	IMG_UINT32 ui32NumPartialRenders;	/*!< Number of partial renders. Used to setup ZLS bits correctly */

	RGXFWIF_CLEANUP_CTL sTACleanupState;
	RGXFWIF_CLEANUP_CTL s3DCleanupState;

} RGXFWIF_HWRTDATA;

/*!
    Data master selection
*/
typedef enum _RGXFWIF_DM_ {
	RGXFWIF_DM_TA = 0,
	RGXFWIF_DM_3D = 1,
	RGXFWIF_DM_CDM = 2,
	RGXFWIF_DM_2D = 3,
	RGXFWIF_DM_GP = 4,
	RGXFWIF_DM_UNUSED = 5,
	RGXFWIF_DM_MAX_MTS = 6,

	RGXFWIF_DM_FORCE_I32 = -1,

} RGXFWIF_DM;
/* Maximum number of DM in use: TA, 3D, 2D, CDM, GP */
#define RGXFWIF_DM_MAX			(5)

#endif				/*  __RGX_FWIF_SHARED_H__ */

/******************************************************************************
 End of file (rgx_fwif_shared.h)
******************************************************************************/
