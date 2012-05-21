									    /*************************************************************************//*!
									       @File
									       @Title          RGX firmware interface structures used by pvrsrvkm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX firmware interface structures used by pvrsrvkm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__RGX_FWIF_KM_H__)
#define __RGX_FWIF_KM_H__

#include "img_types.h"
#include "rgxapi_miscinfo.h"
#include "rgx_fwif_shared.h"
#include "dllist.h"

#if defined(RGX_FIRMWARE)
/* Compiling the actual firmware - use a fully typed pointer */
typedef struct _RGXFWIF_HOST_CTL_ *PRGXFWIF_HOST_CTL;
typedef struct _RGXFWIF_KCCB_CTL_ *PRGXFWIF_KCCB_CTL;
typedef IMG_UINT8 *PRGXFWIF_KCCB;
typedef struct _RGXFWIF_FWMEMCONTEXT_ *PRGXFWIF_FWMEMCONTEXT;
typedef struct _RGXFWIF_FWRENDERCONTEXT_ *PRGXFWIF_FWRENDERCONTEXT;
typedef struct _RGXFWIF_FWTQ2DCONTEXT_ *PRGXFWIF_FWTQ2DCONTEXT;
typedef struct _RGXFWIF_FWTQ3DCONTEXT_ *PRGXFWIF_FWTQ3DCONTEXT;
typedef struct _RGXFWIF_FWCOMPUTECONTEXT_ *PRGXFWIF_FWCOMPUTECONTEXT;
typedef struct _RGXFWIF_FWCOMMONCONTEXT_ *PRGXFWIF_FWCOMMONCONTEXT;
typedef IMG_UINT32 *PRGXFWIF_SIGBUFFER;
typedef struct _RGXFWIF_INIT_ *PRGXFWIF_INIT;
typedef struct _RGXFW_UNITTESTS_ *PRGXFW_UNITTESTS;
typedef struct _RGXFWIF_TRACEBUF_ *PRGXFWIF_TRACEBUF;
typedef IMG_UINT8 *PRGXFWIF_COMMONCTX_STATE;
typedef struct _RGXFWIF_TACTX_STATE_ *PRGXFWIF_TACTX_STATE;
typedef struct _RGXFWIF_3DCTX_STATE_ *PRGXFWIF_3DCTX_STATE;
#else
/* Compiling the host driver - use a firmware device virtual pointer */
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_HOST_CTL;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_KCCB_CTL;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_KCCB;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWMEMCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWRENDERCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWTQ2DCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWTQ3DCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWCOMPUTECONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_FWCOMMONCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_SIGBUFFER;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_INIT;
typedef RGXFWIF_DEV_VIRTADDR PRGXFW_UNITTESTS;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_TRACEBUF;
typedef RGXFWIF_DEV_VIRTADDR PRGXFWIF_COMMONCTX_STATE;
#endif				/* RGX_FIRMWARE */

/*!
	Firmware memory context.
*/
typedef struct _RGXFWIF_FWMEMCONTEXT_ {
	IMG_DEV_PHYADDR RGXFW_ALIGN sPCDevPAddr;	/*!< device physical address of context's page catalogue */
	IMG_INT32 uiPageCatBaseRegID;	/*!< associated page catalog base register (-1 == unallocated) */
	IMG_BOOL bEnableTilingRegs;	/*!< should tiling range registers should be enabled? */
	IMG_UINT32 uiBreakpointAddr;	/*!< breakpoint address */
	IMG_UINT32 uiBPHandlerAddr;	/*!< breakpoint handler address */
	IMG_UINT32 uiBreakpointCtl;	/*!< DM and enable control for BP */
} RGXFWIF_FWMEMCONTEXT;

/*!
 * 	FW context state flags
 */
#define	RGXFWIF_CONTEXT_TAFLAGS_NEED_RESUME			0x00000001
#define	RGXFWIF_CONTEXT_RENDERFLAGS_NEED_RESUME		0x00000002
#define RGXFWIF_CONTEXT_CDMFLAGS_NEED_RESUME		0x00000004

typedef struct _RGXFWIF_TACTX_STATE_ {
	/* FW-accessible TA state which must be written out to memory on context store */
	IMG_UINT64 RGXFW_ALIGN uTAReg_VDM_CALL_STACK_POINTER;
	IMG_UINT64 RGXFW_ALIGN uTAReg_VDM_BATCH;
	IMG_UINT64 RGXFW_ALIGN uTAReg_TE_PSG_RTC;
} RGXFWIF_TACTX_STATE;

typedef struct _RGXFWIF_3DCTX_STATE_ {
	/* FW-accessible ISP state which must be written out to memory on context store */
	IMG_UINT64 RGXFW_ALIGN au3DReg_ISP_STORE[RGX_MAX_TILES_IN_FLIGHT];	/* FIXME: runtime control for tiles in flight */
	IMG_UINT64 RGXFW_ALIGN u3DReg_PM_DEALLOCATED_MASK_STATUS;
	IMG_UINT64 RGXFW_ALIGN u3DReg_PM_PDS_MTILEFREE_STATUS;
} RGXFWIF_3DCTX_STATE;

typedef struct _RGXFWIF_CTX_STATE_ {
	RGXFWIF_TACTX_STATE sTAContextState;
	RGXFWIF_3DCTX_STATE s3DContextState;
} RGXFWIF_CTX_STATE;

typedef struct _RGXFWIF_FWCOMMONCONTEXT_ {
	/*
	   Used by bg and irq context
	 */
	/* CCB details for this firmware context */
	PRGXFWIF_CCCB_CTL psCCBCtl;	/*!< CCB control */
	PRGXFWIF_CCCB psCCB;	/*!< CCB base */

	/* Memory Context cleanup state */
	RGXFWIF_CLEANUP_CTL sCleanupState;	/*!< Cleanup information */

	/*
	   Used by the bg context only
	 */
	DLLIST_NODE sWaitingNode;	/*!< List entry for the waiting list */

	/*
	   Used by the irq context only
	 */
	DLLIST_NODE sRunNode;	/*!< List entry for the run list */

	PRGXFWIF_FWMEMCONTEXT psFWMemContext;	/*!< Memory context */

	/* Context suspend state */
	PRGXFWIF_COMMONCTX_STATE RGXFW_ALIGN psContextState;	/*!< TA/3D context suspend state, read/written by FW */

	/*
	 *      Flags e.g. for context switching
	 */
	IMG_UINT32 ui32Flags;
	IMG_UINT32 ui32Priority;
	IMG_UINT64 RGXFW_ALIGN ui64MCUFenceAddr;

} RGXFWIF_FWCOMMONCONTEXT;

/*!
	Firmware render context.
*/
typedef struct _RGXFWIF_FWRENDERCONTEXT_ {
	RGXFWIF_FWCOMMONCONTEXT sTAContext;	/*!< Firmware context for the TA */
	RGXFWIF_FWCOMMONCONTEXT s3DContext;	/*!< Firmware context for the 3D */

} RGXFWIF_FWRENDERCONTEXT;

/*!
    MMU bus interface selection
*/
typedef enum _RGXFWIF_BIFID_ {
	RGXFWIF_BIFID_TA = 0,
	RGXFWIF_BIFID_3D = 1,
	RGXFWIF_BIFID_2D = 2,
	RGXFWIF_BIFID_CDM = 3,
	RGXFWIF_BIFID_HOST = 4,
	RGXFWIF_BIFID_MAX = 5,

	RGXFWIF_BFID_FORCE_I32 = -1,
} RGXFWIF_BIFID;

/*!
 ******************************************************************************
 * Kernel CCB control for RGX
 *****************************************************************************/
typedef struct _RGXFWIF_KCCB_CTL_ {
	IMG_UINT32 ui32WriteOffset;	/*!< write offset into array of commands (MUST be aligned to 16 bytes!) */
	IMG_UINT32 ui32ReadOffset;	/*!< read offset into array of commands */
	IMG_UINT32 ui32WrapMask;	/*!< Offset wrapping mask (Total capacity of the CCB - 1) */
	IMG_UINT32 ui32CmdSize;	/*!< size of each command in bytes */
} RGXFWIF_KCCB_CTL;

/*!
 ******************************************************************************
 * Kernel CCB command structure for RGX
 *****************************************************************************/
#define RGXFWIF_MMUCACHEDATA_FLAGS_TLB	(1 << 0)
#define RGXFWIF_MMUCACHEDATA_FLAGS_PT	(1 << 1)
#define RGXFWIF_MMUCACHEDATA_FLAGS_PD	(1 << 2)
#define RGXFWIF_MMUCACHEDATA_FLAGS_PC	(1 << 3)
#define RGXFWIF_MMUCACHEDATA_FLAGS_PMTLB	(1 << 4)

typedef struct _RGXFWIF_MMUCACHEDATA_ {
	PRGXFWIF_FWMEMCONTEXT psMemoryContext;
	IMG_UINT32 ui32Flags;
} RGXFWIF_MMUCACHEDATA;

typedef struct _RGXFWIF_HOSTPORTDATA_ {
	PRGXFWIF_FWMEMCONTEXT psMemContext;	/*!< Memory context to link the aperture to */
	IMG_UINT32 ui32RgxCRHostIFVal;	/*!< Value to set the CR_HOSTIF register to */
	IMG_BOOL bAcquire;	/*!< Acquire (otherwise release) the hostport */
} RGXFWIF_HOSTPORTDATA;

typedef struct _RGXFWIF_TILINGDATA_ {
	PRGXFWIF_FWMEMCONTEXT psMemContext;	/*!< Memory context to link the aperture to */
	IMG_BOOL bEnabled;	/*!< Should tiling be enabled for psMemContext? */
} RGXFWIF_TILINGDATA;

typedef struct _RGXFWIF_SLCBPCTLDATA_ {
	IMG_BOOL bSetBypassed;	/*!< Should SLC be/not be bypassed for indicated units? */
	IMG_UINT32 uiFlags;	/*!< Units to enable/disable */
} RGXFWIF_SLCBPCTLDATA;

#define RGXFWIF_BPDATA_FLAGS_WRITE	(1 << 0)
#define RGXFWIF_BPDATA_FLAGS_CTL	(1 << 1)
#define RGXFWIF_BPDATA_FLAGS_REGS	(1 << 2)

typedef struct _RGXFWIF_FWBPDATA_ {
	PRGXFWIF_FWMEMCONTEXT psFWMemContext;	/*!< Memory context */
	IMG_UINT32 ui32BPAddr;	/*!< Breakpoint address */
	IMG_UINT32 ui32HandlerAddr;	/*!< Breakpoint handler */
	IMG_UINT32 ui32BPDM;	/*!< Breakpoint control */
	IMG_BOOL bEnable;
	IMG_UINT32 ui32Flags;
	IMG_UINT32 ui32TempRegs;	/*!< Number of temporary registers to overallocate */
	IMG_UINT32 ui32SharedRegs;	/*!< Number of shared registers to overallocate */
} RGXFWIF_BPDATA;

typedef struct _RGXFWIF_KCCB_CMD_KICK_DATA_ {
	PRGXFWIF_FWCOMMONCONTEXT psContext;	/*!< address of the firmware context */
	IMG_UINT32 ui32CWoffUpdate;	/*!< Client CCB woff update */
} RGXFWIF_KCCB_CMD_KICK_DATA;

typedef struct _RGXFWIF_KCCB_CMD_FENCE_DATA_ {
	IMG_UINT32 uiSyncObjDevVAddr;
	IMG_UINT32 uiUpdateVal;
} RGXFWIF_KCCB_CMD_SYNC_DATA;

typedef struct _RGXFWIF_MC_CLEANUPDATA_ {
	PRGXFWIF_FWCOMMONCONTEXT psContext;	/*!< address of the firmware context */
	IMG_UINT32 uiSyncObjDevVAddr;	/*!< sync primitive */
	IMG_UINT32 uiUpdateVal;	/*!< update value */
} RGXFWIF_MC_CLEANUPDATA;

typedef struct _RGXFWIF_HWRTDATA_CLEANUPDATA_ {
	PRGXFWIF_HWRTDATA psHWRTData;	/*!< address of the firmware HWRTData */
	IMG_UINT32 uiSyncObjDevVAddr;	/*!< sync primitive */
	IMG_UINT32 uiUpdateVal;	/*!< update value */
} RGXFWIF_HWRTDATA_CLEANUPDATA;

typedef struct _RGXFWIF_SLCFLUSHINVALDATA_ {
	PRGXFWIF_FWCOMMONCONTEXT psContext;	/*!< Context to fence on */
	IMG_BOOL bInval;	/*!< invalidate the cache as well as flushing */
	IMG_UINT32 eDM;		/*!< DM to flush entries for */
} RGXFWIF_SLCFLUSHINVALDATA;

typedef enum _RGXFWIF_KCCB_CMD_TYPE_ {
	RGXFWIF_KCCB_CMD_KICK = 101,
	RGXFWIF_KCCB_CMD_MMUCACHE = 102,
	RGXFWIF_KCCB_CMD_HPCTL = 103,	/*!< Acquire/release hostport, requires sHostPortData */
	RGXFWIF_KCCB_CMD_BP = 104,
	RGXFWIF_KCCB_CMD_TILINGCTL = 105,	/*!< tiling registers control. Requires sTileData. For validation. */
	RGXFWIF_KCCB_CMD_SLCBPCTL = 106,	/*!< slc bypass control. Requires sSLCBPCtlData. For validation */
	RGXFWIF_KCCB_CMD_SYNC = 107,	/*!< host sync command. Requires sSyncData. */
	RGXFWIF_KCCB_CMD_SLCFLUSHINVAL = 108,	/*!< slc flush and invalidation request */
	RGXFWIF_KCCB_CMD_MC_CLEANUP = 109,	/*!< Requests a Render Context cleanup */
	RGXFWIF_KCCB_CMD_HWRTDATA_CLEANUP = 110,	/*!< Requests a HWRTData cleanup */
} RGXFWIF_KCCB_CMD_TYPE;

typedef struct _RGXFWIF_KCCB_CMD_ {
	RGXFWIF_KCCB_CMD_TYPE eCmdType;	/*!< Command type */
	union {
		RGXFWIF_KCCB_CMD_KICK_DATA sCmdKickData;	/*!< Data for Kick command */
		RGXFWIF_MMUCACHEDATA sMMUCacheData;	/*!< Data for MMUCACHE command */
		RGXFWIF_HOSTPORTDATA sHostPortData;	/*!< Data for HPACQUIRE/RELEASE Commands */
		RGXFWIF_BPDATA sBPData;	/*!< Data for Breakpoint Commands */
		RGXFWIF_TILINGDATA sTileData;	/*!< Data for tiling commands */
		RGXFWIF_SLCBPCTLDATA sSLCBPCtlData;	/*!< Data for SLC Bypass Control */
		RGXFWIF_KCCB_CMD_SYNC_DATA sSyncData;	/*!< Data for host sync commands */
		RGXFWIF_SLCFLUSHINVALDATA sSLCFlushInvalData;	/*!< Data for SLC Flush/Inval commands */
		RGXFWIF_MC_CLEANUPDATA sMCCleanupData;	/*!< Data for firmware memory context cleanup commands */
		RGXFWIF_HWRTDATA_CLEANUPDATA sHWRTDataCleanupData;	/*!< Data for firmware hwrtdata cleanup commands */
		IMG_UINT32 ui32Padding[9];	/*!< Force structure to be 10 words */
	} uCmdData;
} RGXFWIF_KCCB_CMD;

/*!
 ******************************************************************************
 * Signature and Checksums Buffer
 *****************************************************************************/
typedef struct _RGXFWIF_SIGBUF_CTL_ {
	PRGXFWIF_SIGBUFFER psBuffer;	/*!< Ptr to Signature Buffer memory */
	IMG_UINT32 ui32LeftSizeInRegs;	/*!< Amount of space left for storing regs in the buffer */
} RGXFWIF_SIGBUF_CTL;

/*!
 *****************************************************************************
 * Control data for RGX
 *****************************************************************************/

#define MAX_HW_TA3DCONTEXTS	2

typedef struct _RGXFWIF_INIT_ {
	IMG_DEV_VIRTADDR RGXFW_ALIGN sPDSExecBase;
	IMG_DEV_VIRTADDR RGXFW_ALIGN sUSCExecBase;

	IMG_BOOL bEnable2Thrds;

	/* Kernel CCBs */
	PRGXFWIF_KCCB_CTL psKernelCCBCtl[RGXFWIF_DM_MAX];
	PRGXFWIF_KCCB psKernelCCB[RGXFWIF_DM_MAX];
	RGXFWIF_DM eDM[RGXFWIF_DM_MAX];

	RGXFWIF_SIGBUF_CTL asSigBufCtl[RGXFWIF_DM_MAX];

	IMG_BOOL bEnableLogging;
	IMG_UINT32 ui32ConfigFlags;	/*!< Configuration flags from host */

	PRGXFWIF_TRACEBUF psTraceBufCtl;

#if defined(RGXFW_ALIGNCHECKS)
#if defined(RGX_FIRMWARE)
	IMG_UINT32 *paui32AlignChecks;
#else
	RGXFWIF_DEV_VIRTADDR paui32AlignChecks;
#endif
#endif

#if defined(SUPPORT_RGXFW_UNITTESTS)
	PRGXFW_UNITTESTS psFWUnitTests;
#endif

} RGXFWIF_INIT;

/*!
 ******************************************************************************
 * RGXFW Unittests declarations
 *****************************************************************************/
typedef struct _RGXFW_UNITTEST2_ {
	/* Irq events */
	IMG_UINT32 ui32IrqKicksDM[RGXFWIF_DM_MAX_MTS];
	IMG_UINT32 ui32IrqKicksBg;
	IMG_UINT32 ui32IrqKicksTimer;

	/* Bg events */
	IMG_UINT32 ui32BgKicksDM[RGXFWIF_DM_MAX_MTS];
	IMG_UINT32 ui32BgKicksCounted;

} RGXFW_UNITTEST2;

/*!
 ******************************************************************************
 * RGXFW_UNITTESTS declaration
 *****************************************************************************/
#define RGXFW_UNITTEST_FWPING		(0x1)
#define RGXFW_UNITTEST_FWPONG		(0x2)

#define RGXFW_UNITTEST_IS_BGKICK(DM)	((DM) & 0x1)

typedef struct _RGXFW_UNITTESTS_ {
	IMG_UINT32 ui32Status;

	RGXFW_UNITTEST2 sUnitTest2;

} RGXFW_UNITTESTS;

/* Cleanup command control word */
#define	PVRSRV_CLEANUPCMD_RT		0x1
#define	PVRSRV_CLEANUPCMD_RC		0x2
#define	PVRSRV_CLEANUPCMD_TC		0x3
#define	PVRSRV_CLEANUPCMD_2DC		0x4
#define	PVRSRV_CLEANUPCMD_PB		0x5

/* Power command control word */
#define PVRSRV_POWERCMD_POWEROFF	0x1
#define PVRSRV_POWERCMD_IDLE		0x2
#define PVRSRV_POWERCMD_RESUME		0x3

#endif				/*  __RGX_FWIF_KM_H__ */

/******************************************************************************
 End of file (rgx_fwif_km.h)
******************************************************************************/
