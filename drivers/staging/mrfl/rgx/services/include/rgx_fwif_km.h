/*************************************************************************/ /*!
@File
@Title          RGX firmware interface structures used by pvrsrvkm
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface structures used by pvrsrvkm
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

#if !defined (__RGX_FWIF_KM_H__)
#define __RGX_FWIF_KM_H__

#include "img_types.h"
#include "rgx_fwif_shared.h"
#include "rgxdefs_km.h"
#include "pvr_debug.h"
#include "dllist.h"
#include "rgx_hwperf.h"


#if defined(RGX_FIRMWARE)
/* Compiling the actual firmware - use a fully typed pointer */
typedef struct _RGXFWIF_HOST_CTL_			*PRGXFWIF_HOST_CTL;
typedef struct _RGXFWIF_KCCB_CTL_			*PRGXFWIF_KCCB_CTL;
typedef IMG_UINT8							*PRGXFWIF_KCCB;
typedef struct _RGXFWIF_FWMEMCONTEXT_		*PRGXFWIF_FWMEMCONTEXT;
typedef struct _RGXFWIF_FWRENDERCONTEXT_	*PRGXFWIF_FWRENDERCONTEXT;
typedef struct _RGXFWIF_FWTQ2DCONTEXT_		*PRGXFWIF_FWTQ2DCONTEXT;
typedef struct _RGXFWIF_FWTQ3DCONTEXT_		*PRGXFWIF_FWTQ3DCONTEXT;
typedef struct _RGXFWIF_FWCOMPUTECONTEXT_	*PRGXFWIF_FWCOMPUTECONTEXT;
typedef struct _RGXFWIF_FWCOMMONCONTEXT_	*PRGXFWIF_FWCOMMONCONTEXT;
typedef struct _RGXFWIF_ZSBUFFER_			*PRGXFWIF_ZSBUFFER;
typedef IMG_UINT32							*PRGXFWIF_SIGBUFFER;
typedef struct _RGXFWIF_INIT_				*PRGXFWIF_INIT;
typedef struct _RGXFW_UNITTESTS_			*PRGXFW_UNITTESTS;
typedef struct _RGXFWIF_TRACEBUF_			*PRGXFWIF_TRACEBUF;
typedef IMG_UINT8							*PRGXFWIF_COMMONCTX_STATE;
typedef struct _RGXFWIF_TACTX_STATE_		*PRGXFWIF_TACTX_STATE;
typedef struct _RGXFWIF_3DCTX_STATE_		*PRGXFWIF_3DCTX_STATE;
typedef IMG_UINT8							*PRGXFWIF_RF_CMD;
typedef struct _RGXFWIF_COMPCHECKS_			*PRGXFWIF_COMPCHECKS;
typedef struct _RGX_HWPERF_CONFIG_CNTBLK_	*PRGX_HWPERF_CONFIG_CNTBLK;
#else
/* Compiling the host driver - use a firmware device virtual pointer */
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_HOST_CTL;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_KCCB_CTL;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_KCCB;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWMEMCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWRENDERCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWTQ2DCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWTQ3DCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWCOMPUTECONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_FWCOMMONCONTEXT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_ZSBUFFER;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_SIGBUFFER;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_INIT;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFW_UNITTESTS;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_TRACEBUF;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_COMMONCTX_STATE;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_RF_CMD;
typedef RGXFWIF_DEV_VIRTADDR				PRGXFWIF_COMPCHECKS;
typedef RGXFWIF_DEV_VIRTADDR				PRGX_HWPERF_CONFIG_CNTBLK;
#endif /* RGX_FIRMWARE */

/*!
	Firmware memory context.
*/
typedef struct _RGXFWIF_FWMEMCONTEXT_
{
	IMG_DEV_PHYADDR			RGXFW_ALIGN sPCDevPAddr;	/*!< device physical address of context's page catalogue */
	IMG_INT32				uiPageCatBaseRegID;	/*!< associated page catalog base register (-1 == unallocated) */
	IMG_UINT32				uiBreakpointAddr; /*!< breakpoint address */
	IMG_UINT32				uiBPHandlerAddr;  /*!< breakpoint handler address */
	IMG_UINT32				uiBreakpointCtl; /*!< DM and enable control for BP */
} RGXFWIF_FWMEMCONTEXT;

/*!
 * 	FW context state flags
 */
#define	RGXFWIF_CONTEXT_TAFLAGS_NEED_RESUME			0x00000001
#define	RGXFWIF_CONTEXT_RENDERFLAGS_NEED_RESUME		0x00000002
#define RGXFWIF_CONTEXT_CDMFLAGS_NEED_RESUME		0x00000004

typedef struct _RGXFWIF_TACTX_STATE_
{
	/* FW-accessible TA state which must be written out to memory on context store */
	IMG_UINT64	RGXFW_ALIGN uTAReg_VDM_CALL_STACK_POINTER;
	IMG_UINT64	RGXFW_ALIGN uTAReg_VDM_BATCH;
	IMG_UINT64	RGXFW_ALIGN uTAReg_TE_PSG_RTC;
	
#if defined(DEBUG)
	/* Number of stores on this context */
	IMG_UINT32  ui32NumStores;
#endif
} RGXFWIF_TACTX_STATE;

typedef struct _RGXFWIF_3DCTX_STATE_
{
	/* FW-accessible ISP state which must be written out to memory on context store */
	IMG_UINT64	RGXFW_ALIGN au3DReg_ISP_STORE[RGX_MAX_NUM_PIPES]; /* FIXME: runtime control for tiles in flight */
	IMG_UINT64	RGXFW_ALIGN u3DReg_PM_DEALLOCATED_MASK_STATUS;
	IMG_UINT64	RGXFW_ALIGN u3DReg_PM_PDS_MTILEFREE_STATUS;
	
#if defined(DEBUG)
	/* Number of stores on this context */
	IMG_UINT32  ui32NumStores;
#endif
} RGXFWIF_3DCTX_STATE;

typedef struct _RGXFWIF_CTX_STATE_
{
	RGXFWIF_TACTX_STATE		sTAContextState;
	RGXFWIF_3DCTX_STATE		s3DContextState;
} RGXFWIF_CTX_STATE;

typedef struct _RGXFWIF_FWCOMMONCONTEXT_
{
	/*
		Used by bg and irq context
	*/
	/* CCB details for this firmware context */
	PRGXFWIF_CCCB_CTL		psCCBCtl;				/*!< CCB control */
	PRGXFWIF_CCCB			psCCB;					/*!< CCB base */

	/* Memory Context cleanup state */
	RGXFWIF_CLEANUP_CTL		sCleanupState;			/*!< Cleanup information */

	/*
		Used by the bg context only
	*/
	DLLIST_NODE				sWaitingNode;			/*!< List entry for the waiting list */

	/*
		Used by the irq context only
	*/
	DLLIST_NODE				sRunNode;				/*!< List entry for the run list */
	
	PRGXFWIF_FWMEMCONTEXT	psFWMemContext;			/*!< Memory context */

	/* Context suspend state */
	PRGXFWIF_COMMONCTX_STATE	RGXFW_ALIGN psContextState;		/*!< TA/3D context suspend state, read/written by FW */
	
	/* Framework state
	 */
	PRGXFWIF_RF_CMD		RGXFW_ALIGN psRFCmd;		/*!< Register updates for Framework */
	
	/*
	 * 	Flags e.g. for context switching
	 */
	IMG_UINT32				ui32Flags;
	IMG_UINT32				ui32Priority;
	IMG_UINT64		RGXFW_ALIGN 	ui64MCUFenceAddr;

} RGXFWIF_FWCOMMONCONTEXT;


/*!
	Firmware render context.
*/
typedef struct _RGXFWIF_FWRENDERCONTEXT_
{
	RGXFWIF_FWCOMMONCONTEXT	sTAContext;				/*!< Firmware context for the TA */
	RGXFWIF_FWCOMMONCONTEXT	s3DContext;				/*!< Firmware context for the 3D */

	IMG_UINT32			ui32TotalNumPartialRenders; /*!< Total number of partial renders */
	IMG_UINT32			ui32TotalNumOutOfMemory;	/*!< Total number of OOMs */

} RGXFWIF_FWRENDERCONTEXT;

/*!
    BIF requester selection
*/
typedef enum _RGXFWIF_BIFREQ_
{
	RGXFWIF_BIFREQ_TA		= 0,
	RGXFWIF_BIFREQ_3D		= 1,
	RGXFWIF_BIFREQ_2D		= 2,
	RGXFWIF_BIFREQ_CDM		= 3,
	RGXFWIF_BIFREQ_MAX		= 4,

	RGXFWIF_BIFREQ_FORCE_I32  = -1,
} RGXFWIF_BIFREQ;

static INLINE RGXFWIF_BIFREQ rgxfwif_BIFREQ_from_DM(RGXFWIF_DM eDM)
{
	switch(eDM)
	{
	case RGXFWIF_DM_TA:  return RGXFWIF_BIFREQ_TA;
	case RGXFWIF_DM_3D:  return RGXFWIF_BIFREQ_3D;
	case RGXFWIF_DM_CDM: return RGXFWIF_BIFREQ_CDM;
	case RGXFWIF_DM_2D:  return RGXFWIF_BIFREQ_2D;
	default: break;
	}
	/* It is an error to translate any other DM to BIFREQ */
	return RGXFWIF_BIFREQ_MAX;
}

/*!
 ******************************************************************************
 * Kernel CCB control for RGX
 *****************************************************************************/
typedef struct _RGXFWIF_KCCB_CTL_
{
	IMG_UINT32				ui32WriteOffset;		/*!< write offset into array of commands (MUST be aligned to 16 bytes!) */
	IMG_UINT32				ui32ReadOffset;			/*!< read offset into array of commands */
	IMG_UINT32				ui32WrapMask;			/*!< Offset wrapping mask (Total capacity of the CCB - 1) */
	IMG_UINT32				ui32CmdSize;			/*!< size of each command in bytes */
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

typedef struct _RGXFWIF_MMUCACHEDATA_
{
	PRGXFWIF_FWMEMCONTEXT		psMemoryContext;
	IMG_UINT32					ui32Flags;
} RGXFWIF_MMUCACHEDATA;

typedef struct _RGXFWIF_SLCBPCTLDATA_
{
	IMG_BOOL               bSetBypassed;        /*!< Should SLC be/not be bypassed for indicated units? */
	IMG_UINT32             uiFlags;             /*!< Units to enable/disable */
} RGXFWIF_SLCBPCTLDATA;

#define RGXFWIF_BPDATA_FLAGS_WRITE	(1 << 0)
#define RGXFWIF_BPDATA_FLAGS_CTL	(1 << 1)
#define RGXFWIF_BPDATA_FLAGS_REGS	(1 << 2)

typedef struct _RGXFWIF_FWBPDATA_
{
	PRGXFWIF_FWMEMCONTEXT	psFWMemContext;			/*!< Memory context */
	IMG_UINT32		ui32BPAddr;			/*!< Breakpoint address */
	IMG_UINT32		ui32HandlerAddr;		/*!< Breakpoint handler */
	IMG_UINT32		ui32BPDM;			/*!< Breakpoint control */
	IMG_BOOL		bEnable;
	IMG_UINT32		ui32Flags;
	IMG_UINT32		ui32TempRegs;		/*!< Number of temporary registers to overallocate */
	IMG_UINT32		ui32SharedRegs;		/*!< Number of shared registers to overallocate */
} RGXFWIF_BPDATA;

typedef struct _RGXFWIF_KCCB_CMD_KICK_DATA_
{
	PRGXFWIF_FWCOMMONCONTEXT	psContext;			/*!< address of the firmware context */
	IMG_UINT32					ui32CWoffUpdate;	/*!< Client CCB woff update */
} RGXFWIF_KCCB_CMD_KICK_DATA;

typedef struct _RGXFWIF_KCCB_CMD_FENCE_DATA_
{
	IMG_UINT32 uiSyncObjDevVAddr;
	IMG_UINT32 uiUpdateVal;
} RGXFWIF_KCCB_CMD_SYNC_DATA;

/*
	If you add/remove entries here then please
	update rgxfw_logdata_kernel_ccb and 
	rgxfw_logdata_ready_list accordingly
*/
typedef enum _RGXFWIF_CLEANUP_TYPE_
{
	RGXFWIF_CLEANUP_FWCOMMONCONTEXT,		/*!< FW common context cleanup */
	RGXFWIF_CLEANUP_HWRTDATA,				/*!< FW HW RT data cleanup */
	RGXFWIF_CLEANUP_FREELIST,				/*!< FW freelist cleanup */
	RGXFWIF_CLEANUP_ZSBUFFER,				/*!< FW ZS Buffer cleanup */
} RGXFWIF_CLEANUP_TYPE;

#define RGXFWIF_CLEANUP_RUN		(1 << 0)	/*!< The requested cleanup command has run on the FW */
#define RGXFWIF_CLEANUP_BUSY	(1 << 1)	/*!< The requested resource is busy */

typedef struct _RGXFWIF_CLEANUP_REQUEST_
{
	RGXFWIF_CLEANUP_TYPE			eCleanupType;			/*!< Cleanup type */
	union {
		PRGXFWIF_FWCOMMONCONTEXT 	psContext;				/*!< FW common context to cleanup */
		PRGXFWIF_HWRTDATA 			psHWRTData;				/*!< HW RT to cleanup */
		PRGXFWIF_FREELIST 			psFreelist;				/*!< Freelist to cleanup */
		PRGXFWIF_ZSBUFFER 			psZSBuffer;				/*!< ZS Buffer to cleanup */
	} uCleanupData;
	IMG_UINT32						uiSyncObjDevVAddr;		/*!< sync primitive used to indicate state of the request */
} RGXFWIF_CLEANUP_REQUEST;

typedef enum _RGXFWIF_POWER_TYPE_
{
	RGXFWIF_POW_OFF_REQ = 1,
	RGXFWIF_POW_NUMDUST_CHANGE
} RGXFWIF_POWER_TYPE;

typedef struct _RGXFWIF_POWER_REQUEST_
{
	RGXFWIF_POWER_TYPE				ePowType;				/*!< Type of power request */
	union
	{
		IMG_UINT32					ui32NumOfDusts;			/*!< Number of active Dusts */
		IMG_BOOL					bForced;				/*!< If the operation is mandatory */
	} uPoweReqData;
} RGXFWIF_POWER_REQUEST;

typedef struct _RGXFWIF_SLCFLUSHINVALDATA_
{
    PRGXFWIF_FWCOMMONCONTEXT psContext;	/*!< Context to fence on */
    IMG_BOOL	bInval;			/*!< invalidate the cache as well as flushing */
    IMG_UINT32	eDM;			/*!< DM to flush entries for */
} RGXFWIF_SLCFLUSHINVALDATA;

typedef struct _RGXFWIF_ZSBUFFERSYNCPRIMDATA_
{
    IMG_UINT32	psZSBufferPopulateSyncPrimAddr;		/*!< FW ZS Buffer populate Sync Prim address */
    IMG_UINT32	psZSBufferUnPopulateSyncPrimAddr;	/*!< FW ZS buffer unpopulate Sync Prim address */
    IMG_UINT32	psGrowSyncPrimAddr;		/*!< FW Grow Sync Prim address */
    IMG_UINT32	psShrinkSyncPrimAddr;	/*!< FW Shrink Sync Prim address */
	IMG_UINT32 uiSyncObjDevVAddr;		/*!< sync primitive */
	IMG_UINT32 uiUpdateVal;				/*!< update value */
} RGXFWIF_ZSBUFFERSYNCPRIMDATA;

typedef struct _RGXFWIF_FREELIST_SIZE_DATA_
{
	IMG_UINT32 ui32FwFreeListAddr;		/*!< address of the fw freelist */
	IMG_UINT32 ui32NewNumPages;			/*!< Number of available pages */
} RGXFWIF_FREELIST_SIZE_DATA;

typedef struct _RGXFWIF_HWPERF_CTRL_
{
	IMG_BOOL	 bEnable; 		/*!< Enable/disable the generation of all HWPerf events */
	IMG_UINT32	 ui32Mask;		/*!< Unused at present */
} RGXFWIF_HWPERF_CTRL;

typedef struct _RGXFWIF_HWPERF_CONFIG_ENABLE_BLKS_
{
	IMG_UINT32				ui32NumBlocks; 	/*!< Number of RGX_HWPERF_CONFIG_CNTBLK in the array */
	PRGX_HWPERF_CONFIG_CNTBLK pasBlockConfigs;	/*!< Address of the RGX_HWPERF_CONFIG_CNTBLK array */
} RGXFWIF_HWPERF_CONFIG_ENABLE_BLKS;

typedef struct _RGXFWIF_HWPERF_CTRL_BLKS_
{
	IMG_BOOL				bEnable;
	IMG_UINT32				ui32NumBlocks; 						/*!< Number of block IDs in the array */
	IMG_UINT8				aeBlockIDs[RGX_HWPERF_MAX_BLKS];	/*!< Array of RGX_HWPERF_CNTBLK_ID values */
} RGXFWIF_HWPERF_CTRL_BLKS;


/*
	If you add/remove entries here then please
	update rgxfw_logdata_kernel_ccb accordingly
*/
typedef enum _RGXFWIF_KCCB_CMD_TYPE_
{
	RGXFWIF_KCCB_CMD_KICK						= 101,
	RGXFWIF_KCCB_CMD_MMUCACHE					= 102,
	RGXFWIF_KCCB_CMD_BP							= 104,
	RGXFWIF_KCCB_CMD_SLCBPCTL   				= 106, /*!< slc bypass control. Requires sSLCBPCtlData. For validation */
	RGXFWIF_KCCB_CMD_SYNC       				= 107, /*!< host sync command. Requires sSyncData. */
	RGXFWIF_KCCB_CMD_SLCFLUSHINVAL				= 108, /*!< slc flush and invalidation request */
	RGXFWIF_KCCB_CMD_CLEANUP					= 109, /*!< Requests cleanup of a FW resource (type specified in the command data) */
	RGXFWIF_KCCB_CMD_POW						= 110, /*!< Power request */
	RGXFWIF_KCCB_CMD_ZSBUFFER_SYNCPRIM_UDPATE	= 111, /*!< Update ZLS SyncPrim FW address */
	RGXFWIF_KCCB_CMD_FREELIST_SIZE_UPDATE		= 112, /*!< Requests an update of available pages on the Freelist */
	RGXFWIF_KCCB_CMD_HWPERF_CTRL_EVENTS			= 113, /*!< Control the HWPerf event generation behaviour */
	RGXFWIF_KCCB_CMD_HWPERF_CONFIG_ENABLE_BLKS	= 114, /*!< Configure, clear and enable multiple HWPerf blocks */
	RGXFWIF_KCCB_CMD_HWPERF_CTRL_BLKS			= 115 /*!< Enable or disable multiple HWPerf blocks (reusing existing configuration) */
} RGXFWIF_KCCB_CMD_TYPE;

typedef struct _RGXFWIF_KCCB_CMD_
{
	RGXFWIF_KCCB_CMD_TYPE					eCmdType;			/*!< Command type */
	union
	{
		RGXFWIF_KCCB_CMD_KICK_DATA			sCmdKickData;			/*!< Data for Kick command */
		RGXFWIF_MMUCACHEDATA				sMMUCacheData;			/*!< Data for MMUCACHE command */
		RGXFWIF_BPDATA						sBPData;				/*!< Data for Breakpoint Commands */
		RGXFWIF_SLCBPCTLDATA       			sSLCBPCtlData;  		/*!< Data for SLC Bypass Control */
		RGXFWIF_KCCB_CMD_SYNC_DATA 			sSyncData;          	/*!< Data for host sync commands */
		RGXFWIF_SLCFLUSHINVALDATA			sSLCFlushInvalData;		/*!< Data for SLC Flush/Inval commands */
		RGXFWIF_CLEANUP_REQUEST				sCleanupData; 			/*!< Data for cleanup commands */
		RGXFWIF_POWER_REQUEST				sPowData;				/*!< Data for power request commands */
		RGXFWIF_ZSBUFFERSYNCPRIMDATA		sZSBufferSyncPrimUpdateData;	/*!< Data for ZS Buffer Sync Prim address update command > */
		RGXFWIF_FREELIST_SIZE_DATA			sFreelistSizeData;		/*!< Data to update freelist size > */
		RGXFWIF_HWPERF_CTRL					sHWPerfCtrl;			/*!< Data for HWPerf control command */
		RGXFWIF_HWPERF_CONFIG_ENABLE_BLKS	sHWPerfCfgEnableBlks;	/*!< Data for HWPerf configure, clear and enable performance counter block command */
		RGXFWIF_HWPERF_CTRL_BLKS			sHWPerfCtrlBlks;		/*!< Data for HWPerf enable or disable performance counter block commands */
		IMG_UINT32							ui32Padding[9];			/*!< Force structure to be 10 words */
	} uCmdData;
} RGXFWIF_KCCB_CMD;

BLD_ASSERT(sizeof(RGXFWIF_KCCB_CMD)==40, rgx_fwif_km_h);

/*!
 ******************************************************************************
 * Signature and Checksums Buffer
 *****************************************************************************/
typedef struct _RGXFWIF_SIGBUF_CTL_
{
	PRGXFWIF_SIGBUFFER		psBuffer;			/*!< Ptr to Signature Buffer memory */
	IMG_UINT32				ui32LeftSizeInRegs;	/*!< Amount of space left for storing regs in the buffer */
} RGXFWIF_SIGBUF_CTL;

/*!
 *****************************************************************************
 * Control data for RGX
 *****************************************************************************/

#define MAX_HW_TA3DCONTEXTS	2


typedef struct _RGXFWIF_INIT_
{
	IMG_DEV_PHYADDR 		RGXFW_ALIGN sBifFaultPhysAddr;

	IMG_DEV_VIRTADDR		RGXFW_ALIGN sPDSExecBase;
	IMG_DEV_VIRTADDR		RGXFW_ALIGN sUSCExecBase;

	IMG_BOOL				bFirstRender;
	IMG_BOOL				bFrameworkAfterInit;
	IMG_BOOL				bEnableHWPerf;
	IMG_UINT32				uiPowerSync;

	/* Kernel CCBs */
	PRGXFWIF_KCCB_CTL		psKernelCCBCtl[RGXFWIF_DM_MAX];
	PRGXFWIF_KCCB			psKernelCCB[RGXFWIF_DM_MAX];
	RGXFWIF_DM				eDM[RGXFWIF_DM_MAX];

	RGXFWIF_SIGBUF_CTL		asSigBufCtl[RGXFWIF_DM_MAX];

	IMG_BOOL				bEnableLogging;
	IMG_UINT32				ui32ConfigFlags;	/*!< Configuration flags from host */
	IMG_UINT32				ui32BreakpointTemps;
	IMG_UINT32				ui32BreakpointShareds;

	PRGXFWIF_TRACEBUF		psTraceBufCtl;

#if defined(RGXFW_ALIGNCHECKS)
#if defined(RGX_FIRMWARE)
	IMG_UINT32*				paui32AlignChecks;
#else
	RGXFWIF_DEV_VIRTADDR	paui32AlignChecks;
#endif
#endif

	/* Compatibility checks to be populated by the Firmware */
	RGXFWIF_COMPCHECKS		sRGXCompChecks;

} RGXFWIF_INIT;

/*!
 ******************************************************************************
 * RGXFW Unittests declarations
 *****************************************************************************/
typedef struct _RGXFW_UNITTEST2_
{
	/* Irq events */
	IMG_UINT32	ui32IrqKicksDM[RGXFWIF_DM_MAX_MTS];
	IMG_UINT32	ui32IrqKicksBg;
	IMG_UINT32	ui32IrqKicksTimer;

	/* Bg events */
	IMG_UINT32	ui32BgKicksDM[RGXFWIF_DM_MAX_MTS];
	IMG_UINT32	ui32BgKicksCounted;

} RGXFW_UNITTEST2;

/*!
 ******************************************************************************
 * RGXFW_UNITTESTS declaration
 *****************************************************************************/
#define RGXFW_UNITTEST_FWPING		(0x1)
#define RGXFW_UNITTEST_FWPONG		(0x2)

#define RGXFW_UNITTEST_IS_BGKICK(DM)	((DM) & 0x1)

typedef struct _RGXFW_UNITTESTS_
{
	IMG_UINT32	ui32Status;

	RGXFW_UNITTEST2 sUnitTest2;

} RGXFW_UNITTESTS;

#endif /*  __RGX_FWIF_KM_H__ */

/******************************************************************************
 End of file (rgx_fwif_km.h)
******************************************************************************/


