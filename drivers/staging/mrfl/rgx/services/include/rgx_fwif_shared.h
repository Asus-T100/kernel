/*************************************************************************/ /*!
@File
@Title          RGX firmware interface structures
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface structures shared by both host client
                and host server
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

#if !defined (__RGX_FWIF_SHARED_H__)
#define __RGX_FWIF_SHARED_H__

#include "img_types.h"
#include "rgx_hwperf.h"
#include "devicemem_typedefs.h"


/*!
 ******************************************************************************
 * Device state flags
 *****************************************************************************/
#define RGXKMIF_DEVICE_STATE_ZERO_FREELIST		(0x1 << 0)		/*!< Zeroing the physical pages of reconstructed freelists */


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

typedef struct _RGXFWIF_DEV_VIRTADDR_
{
	IMG_UINT32	ui32Addr;
} RGXFWIF_DEV_VIRTADDR;


typedef IMG_UINT8	RGXFWIF_CCCB;

#if defined(RGX_FIRMWARE)
/* Compiling the actual firmware - use a fully typed pointer */
typedef RGXFWIF_CCCB						*PRGXFWIF_CCCB;
typedef struct _RGXFWIF_CCCB_CTL_			*PRGXFWIF_CCCB_CTL;
typedef struct _RGXFWIF_RENDER_TARGET_		*PRGXFWIF_RENDER_TARGET;
typedef struct _RGXFWIF_HWRTDATA_			*PRGXFWIF_HWRTDATA;
typedef struct _RGXFWIF_FREELIST_			*PRGXFWIF_FREELIST;
typedef struct _RGXFWIF_RTA_CTL_			*PRGXFWIF_RTA_CTL;
typedef IMG_UINT32						*PRGXFWIF_UFO_ADDR;
#else
/* Compiling the host driver - use a firmware device virtual pointer */
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_CCCB;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_CCCB_CTL;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_RENDER_TARGET;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_HWRTDATA;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_FREELIST;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_RTA_CTL;
typedef RGXFWIF_DEV_VIRTADDR	PRGXFWIF_UFO_ADDR;
#endif /* RGX_FIRMWARE */

typedef struct _RGXFWIF_UFO_
{
	PRGXFWIF_UFO_ADDR	puiAddrUFO;
	IMG_UINT32			ui32Value;
} RGXFWIF_UFO;

/*!
	HWRTData state the render is in
*/
typedef enum
{
	RGXFWIF_RTDATA_STATE_NONE = 0,
	RGXFWIF_RTDATA_STATE_KICKTA,
	RGXFWIF_RTDATA_STATE_KICKTAFIRST,
	RGXFWIF_RTDATA_STATE_TAFINISHED,
	RGXFWIF_RTDATA_STATE_KICK3D,
	RGXFWIF_RTDATA_STATE_3DFINISHED,
	RGXFWIF_RTDATA_STATE_TAOUTOFMEM,
	RGXFWIF_RTDATA_STATE_PARTIALRENDERFINISHED,
	RGXFWIF_RTDATA_STATE_HWR					/*!< In case of HWR, we can't set the RTDATA state to NONE,
													 as this will cause any TA to become a first TA.
													 To ensure all related TA's are skipped, we use the HWR state */
} RGXFWIF_RTDATA_STATE;

typedef struct _RGXFWIF_CLEANUP_CTL_
{
	IMG_UINT32				ui32SubmittedCommands;	/*!< Number of commands submitted to the FW */
	IMG_UINT32				ui32ExecutedCommands;	/*!< Number of commands executed by the FW */
	IMG_UINT32 				ui32SyncObjDevVAddr;	/*!< SyncPrimitive to update after cleanup completion */
}RGXFWIF_CLEANUP_CTL;


/*!
 ******************************************************************************
 * Client CCB control for RGX
 *****************************************************************************/
typedef struct _RGXFWIF_CCCB_CTL_
{
	IMG_UINT32				ui32WriteOffset;	/*!< write offset into array of commands (MUST be aligned to 16 bytes!) */
	IMG_UINT32				ui32ReadOffset;		/*!< read offset into array of commands */
	IMG_UINT32				ui32DepOffset;		/*!< Dependency offset */
	IMG_UINT32				ui32WrapMask;		/*!< Offset wrapping mask (Total capacity of the CCB - 1) */
} RGXFWIF_CCCB_CTL;

typedef enum 
{
	RGXFW_LOCAL_FREELIST = 0,
	RGXFW_GLOBAL_FREELIST = 1,
	RGXFW_MMU_FREELIST = 2,
} RGXFW_FREELIST_TYPE;

#if defined(SUPPORT_MMU_FREELIST)
#define RGXFW_MAX_FREELISTS		(3)
#else
#define RGXFW_MAX_FREELISTS		(2)
#endif

typedef struct _RGXFWIF_RTA_CTL_
{
	IMG_UINT32				ui32RenderTargetIndex;		//Render number
	IMG_UINT32				ui32CurrentRenderTarget;	//index in RTA
	IMG_UINT32				ui32ActiveRenderTargets;	//total active RTs
#if defined(RGX_FIRMWARE)
	IMG_UINT32				*paui32ValidRenderTargets;	//Array of valid RT indices
#else
	RGXFWIF_DEV_VIRTADDR	paui32ValidRenderTargets;
#endif
} RGXFWIF_RTA_CTL;

typedef struct _RGXFWIF_FREELIST_
{
	IMG_DEV_VIRTADDR	RGXFW_ALIGN psFreeListDevVAddr;
	IMG_UINT64			RGXFW_ALIGN ui64CurrentDevVAddr;
	IMG_UINT64			RGXFW_ALIGN ui64CurrentStackTop;
	IMG_UINT32			ui32MaxPages;
	IMG_UINT32			ui32GrowPages;
	IMG_UINT32			ui32CurrentPages;
	IMG_UINT32			ui32AllocatedPageCount;
	IMG_UINT32			ui32AllocatedMMUPageCount;
	IMG_UINT32			ui32HWRCounter;
	IMG_UINT32			ui32FreeListID;
} RGXFWIF_FREELIST;


typedef struct _RGXFWIF_RENDER_TARGET_
{
	IMG_DEV_VIRTADDR	RGXFW_ALIGN psVHeapTableDevVAddr; /*!< VHeap Data Store */
	IMG_BOOL			bZeroTACaches;					  /*!< Whether RTC and TPC caches (on mem) need to be zeroed on next TA kick */

} RGXFWIF_RENDER_TARGET;


typedef struct _RGXFWIF_HWRTDATA_ 
{
	IMG_DEV_VIRTADDR		RGXFW_ALIGN psPMMListDevVAddr; /*!< MList Data Store */
	PRGXFWIF_FREELIST 		RGXFW_ALIGN apsFreeLists[RGXFW_MAX_FREELISTS]; 
	IMG_UINT64				RGXFW_ALIGN ui64VCECatBase;
	IMG_UINT64				RGXFW_ALIGN ui64VCELastCatBase;
	IMG_UINT64				RGXFW_ALIGN ui64TECatBase;
	IMG_UINT64				RGXFW_ALIGN ui64TELastCatBase;
	IMG_UINT64				RGXFW_ALIGN ui64AlistCatBase;
	IMG_UINT64				RGXFW_ALIGN ui64AlistLastCatBase;
#if defined(SUPPORT_VFP)
	IMG_DEV_VIRTADDR		RGXFW_ALIGN sVFPPageTableAddr;
#endif

	IMG_UINT64				RGXFW_ALIGN ui64PMAListStackPointer;

	IMG_UINT32				ui32PMMListStackPointer;

	IMG_UINT32				aui32FreeListHWRSnapshot[RGXFW_MAX_FREELISTS];
	
	PRGXFWIF_RENDER_TARGET	psParentRenderTarget;
	RGXFWIF_RTDATA_STATE	eState;

	IMG_UINT32				ui32NumPartialRenders; /*!< Number of partial renders. Used to setup ZLS bits correctly */

	RGXFWIF_CLEANUP_CTL		sTACleanupState;
	RGXFWIF_CLEANUP_CTL		s3DCleanupState;
	IMG_UINT32				ui32CleanupStatus;
#define HWRTDATA_TA_CLEAN	(1 << 0)
#define HWRTDATA_3D_CLEAN	(1 << 1)

	PRGXFWIF_RTA_CTL		psRTACtl;

	IMG_UINT32				ui32PagesTE;		/*!< Number of pages allocated for TE (only setup/used by MLIST checker */
	IMG_UINT32				ui32PagesVCE;		/*!< Number of pages allocated for VCE (only setup/used by MLIST checker */
	IMG_UINT32				ui32PagesALIST;		/*!< Number of pages allocated for ALIST (only setup/used by MLIST checker */
	IMG_UINT32				bHasLastTA;

} RGXFWIF_HWRTDATA;

typedef enum
{
	RGXFWIF_ZSBUFFER_UNBACKED = 0,
	RGXFWIF_ZSBUFFER_BACKED,
	RGXFWIF_ZSBUFFER_BACKING_PENDING,
	RGXFWIF_ZSBUFFER_UNBACKING_PENDING,
}RGXFWIF_ZSBUFFER_STATE;

typedef struct _RGXFWIF_ZSBUFFER_
{
	IMG_UINT32				ui32ZSBufferID;				/*!< Buffer ID*/
	IMG_BOOL				bOnDemand;					/*!< Needs On-demand ZS Buffer allocation */
	RGXFWIF_ZSBUFFER_STATE	eState;						/*!< Z/S-Buffer state */
	RGXFWIF_CLEANUP_CTL		sCleanupState;				/*!< Cleanup state */
} RGXFWIF_FWZSBUFFER;

/*!
 *****************************************************************************
 * RGX Compatibility checks
 *****************************************************************************/
/* WARNING: RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX can be increased only and
		always equal to (N * sizeof(IMG_UINT32) - 1) */
#define RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX 3 /* WARNING: Do not change this macro without changing 
			accesses from dword to byte in function rgx_bvnc_packed() */

/* WARNING: Whenever the layout of RGXFWIF_COMPCHECKS_BVNC is a subject of change,
	following define should be increased by 1 to indicate to compatibility logic, 
	that layout has changed */
#define RGXFWIF_COMPCHECKS_LAYOUT_VERSION 1

typedef struct _RGXFWIF_COMPCHECKS_BVNC_
{
	IMG_UINT32	ui32LayoutVersion; /* WARNING: This field must be defined as first one in this structure */
	IMG_UINT32  ui32VLenMax;
	IMG_UINT32	ui32BNC;
	IMG_CHAR	aszV[RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX + 1];
} RGXFWIF_COMPCHECKS_BVNC;

#define RGXFWIF_COMPCHECKS_BVNC_DECLARE_AND_INIT(name) RGXFWIF_COMPCHECKS_BVNC name = { RGXFWIF_COMPCHECKS_LAYOUT_VERSION, RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX }
#define RGXFWIF_COMPCHECKS_BVNC_INIT(name) do { (name).ui32LayoutVersion = RGXFWIF_COMPCHECKS_LAYOUT_VERSION; \
												(name).ui32VLenMax = RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX; } while (0)

typedef struct _RGXFWIF_COMPCHECKS_
{
	RGXFWIF_COMPCHECKS_BVNC		sHWBVNC;			/*!< hardware BNC (from the RGX registers) */
	RGXFWIF_COMPCHECKS_BVNC		sFWBVNC;			/*!< firmware BNC */
	IMG_UINT32					ui32DDKVersion;		/*!< software DDK version */
	IMG_UINT32					ui32DDKBuild;		/*!< software DDK build no. */
	IMG_UINT32					ui32BuildOptions;	/*!< build options bit-field */
	IMG_BOOL					bUpdated;			/*!< Information is valid */
} RGXFWIF_COMPCHECKS;


#define GET_CCB_SPACE(WOff, ROff, CCBSize) \
	((((ROff) - (WOff)) + ((CCBSize) - 1)) & ((CCBSize) - 1))

#define UPDATE_CCB_OFFSET(Off, PacketSize, CCBSize) \
	(Off) = (((Off) + (PacketSize)) & ((CCBSize) - 1))

#define RESERVED_CCB_SPACE 		(sizeof(IMG_UINT32))


/* Defines relating to the per-context CCBs */
#define RGX_CCB_SIZE_LOG2			(16) /* 64kB */
#define RGX_CCB_ALLOCGRAN			(64)
#define RGX_CCB_TYPE_TASK			(1 << 31)
#define RGX_CCB_FWALLOC_ALIGN(size)	(((size) + (RGXFWIF_FWALLOC_ALIGN-1)) & ~(RGXFWIF_FWALLOC_ALIGN - 1))

/*!
 ******************************************************************************
 * Client CCB commands for RGX
 *****************************************************************************/
typedef enum _RGXFWIF_CCB_CMD_TYPE_
{
	RGXFWIF_CCB_CMD_TYPE_TA			= 201 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_3D			= 202 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_CDM		= 203 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_TQ_3D		= 204 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_TQ_2D		= 205 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_3D_PR		= 206 | RGX_CCB_TYPE_TASK,
	RGXFWIF_CCB_CMD_TYPE_NULL		= 207 | RGX_CCB_TYPE_TASK,

/* Leave a gap between CCB specific commands and generic commands */
	RGXFWIF_CCB_CMD_TYPE_FENCE		= 211,
	RGXFWIF_CCB_CMD_TYPE_UPDATE		= 212,
	RGXFWIF_CCB_CMD_TYPE_FENCE_PR		= 213,
	RGXFWIF_CCB_CMD_TYPE_PRIORITY		= 214,
	
	RGXFWIF_CCB_CMD_TYPE_PADDING	= 220,
} RGXFWIF_CCB_CMD_TYPE;

/*!
 ******************************************************************************
 * per context CCB control structure for RGX
 *****************************************************************************/
typedef struct _RGX_CLIENT_CCB_
{
	IMG_HANDLE					hClientCCB;					/*!< kernel handle for CCB */
	DEVMEM_EXPORTCOOKIE			sClientCCBExportCookie;		/*!< export cookie for CCB */
	DEVMEM_MEMDESC				*psClientCCBMemDesc;		/*!< MemDesc for CCB */
	IMG_VOID					*pui32CCBLinAddr;			/*!< linear address of the buffer */

	IMG_HANDLE					hClientCCBCtl;				/*!< kernel handle for CCB control */
	DEVMEM_EXPORTCOOKIE			sClientCCBCtlExportCookie;	/*!< export cookie for CCB control */
	DEVMEM_MEMDESC				*psClientCCBCtlMemDesc;		/*!< MemDesc for CCB control */
	volatile RGXFWIF_CCCB_CTL	*psCCBCtl;					/*!< CCB control structure used by the fw */

	IMG_UINT32					ui32CCBWriteOffset;			/*!< CCB write offset from the driver side */
	
	IMG_UINT32					ui32Size;					/*!< ((Size of the buffer) - (overrun size)) */

	IMG_HANDLE					hCleanupCookie;				/*!< Cookie to pass back to the server for cleanup */
	IMG_BOOL					bDumpedCCBCtlAlready;		/*! <To track if we have already dumped CCBCtl */
} RGX_CLIENT_CCB;


typedef struct _RGXFWIF_CCB_CMD_HEADER_
{
	RGXFWIF_CCB_CMD_TYPE	eCmdType;
	IMG_UINT32				ui32CmdSize;
} RGXFWIF_CCB_CMD_HEADER;

/*!
 *****************************************************************************
 * Data master selection
 *****************************************************************************/

/* RGXFWIF_DM moved to the client header rgx_hwperf.h for broader use as
 * the master definition.
 */

/* Maximum number of DM in use: TA, 3D, 2D, CDM, GP */
#define RGXFWIF_DM_MAX			(5)

/* Maximum number of HW DMs (all but GP) : TA, 3D, 2D, CDM */
#define RGXFWIF_HWDM_MAX		(RGXFWIF_DM_2D+1)


#endif /*  __RGX_FWIF_SHARED_H__ */

/******************************************************************************
 End of file (rgx_fwif_shared.h)
******************************************************************************/


