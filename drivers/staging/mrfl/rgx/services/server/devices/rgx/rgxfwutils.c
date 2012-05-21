									    /*************************************************************************//*!
									       @File
									       @Title          RGX firmware utility routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX firmware utility routines
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>

#include "rgxdefs.h"
#include "rgx_cr_defs_km.h"
#include "rgx_fwif_km.h"
#include "pdump_km.h"
#include "osfunc.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "pvr_debug.h"
#include "rgxfwutils.h"
#include "rgx_fwif.h"
#include "rgx_fwif_alignchecks_km.h"
#include "rgxheapconfig.h"
#include "pvrsrv.h"
#include "rgxinit.h"
#include "rgxmem.h"
#include "rgxutils.h"
#include "sync_internal.h"

#ifdef __linux__
#include <linux/kernel.h>	// sprintf
#include <linux/string.h>	// strncpy, strlen
#else
#include <stdio.h>
#endif

#define RGXFWIF_KCCB_TA_NUMCMDS_LOG2	(8)
#define RGXFWIF_KCCB_3D_NUMCMDS_LOG2	(8)
#define RGXFWIF_KCCB_2D_NUMCMDS_LOG2	(8)
#define RGXFWIF_KCCB_CDM_NUMCMDS_LOG2	(8)
#define RGXFWIF_KCCB_GP_NUMCMDS_LOG2	(8)

static void __MTSScheduleWrite(PVRSRV_RGXDEV_INFO * psDevInfo,
			       IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPdumpFlags)
{
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_MTS_SCHEDULE, ui32Value);
#if defined(EMULATOR)
	/* Flush register writes */
	(void)OSReadHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_MTS_SCHEDULE);
#endif
	PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_MTS_SCHEDULE, ui32Value,
		   uiPdumpFlags);
}

#if defined(SUPPORT_RGXFW_UNITTESTS)
/*!
 ******************************************************************************
 * RGXFW Unittests
 *****************************************************************************/

/* 
 * RGXFirmwareUnittestsInit: Allocate the Unittest structure for the fw
 */
static PVRSRV_ERROR RGXFirmwareUnittestsInit(PVRSRV_RGXDEV_INFO * psDevInfo,
					     RGXFWIF_INIT * psRGXFWInit)
{
	PVRSRV_ERROR eError;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for unittests");
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFW_UNITTESTS),
				  PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
				  (PMMETA_PROTECT) |
				  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
				  PVRSRV_MEMALLOCFLAG_UNCACHED,
				  &psDevInfo->psRGXFWUnittestsMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsInit: Failed to allocate %d bytes (%u)",
			 sizeof(RGXFW_UNITTESTS), eError));
		return eError;
	}
#if defined(PDUMP)
	eError = DevmemPDumpLoadMem(psDevInfo->psRGXFWUnittestsMemDesc,
				    0,
				    sizeof(RGXFW_UNITTESTS),
				    PDUMP_FLAGS_CONTINUOUS);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsInit: Failed to dump Unittest memory (%u)",
			 eError));

		DevmemFwFree(psDevInfo->psRGXFWUnittestsMemDesc);
		return eError;
	}
#endif

	/* Prepare the pointer for the fw */
	RGXSetFirmwareAddress(&psRGXFWInit->psFWUnitTests,
			      psDevInfo->psRGXFWUnittestsMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	return PVRSRV_OK;
}

/* 
 * RGXFirmwareUnittestsFWWait: wait for the fw
 */
static PVRSRV_ERROR RGXFirmwareUnittestsFWWait(PVRSRV_RGXDEV_INFO * psDevInfo,
					       RGXFW_UNITTESTS * psFWUnitTest)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

#if defined(PDUMP)
	PDUMPCOMMENTWITHFLAGS(0, "** RGXFW Unittests: Wait for the fw");
	eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWUnittestsMemDesc,
					offsetof(RGXFW_UNITTESTS, ui32Status),
					RGXFW_UNITTEST_FWPONG,
					0xFFFFFFFF,
					PDUMP_POLL_OPERATOR_EQUAL,
					PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsFWWait: DevmemPDumpDevmemPol32 failed (%u)",
			 eError));
		goto RGXFirmwareUnittestsFWWait_exit;
	}
#endif

	eError = PVRSRVPollForValueKM(&psFWUnitTest->ui32Status,
				      RGXFW_UNITTEST_FWPONG, 0xFFFFFFFF);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsFWWait: PVRSRVPollForValueKM failed (%u)",
			 eError));
		goto RGXFirmwareUnittestsFWWait_exit;
	}

 RGXFirmwareUnittestsFWWait_exit:
	return eError;
}

/* 
 * RGXFirmwareUnittestsFWRelease: signal the fw that is waiting on our ping
 */
static IMG_VOID RGXFirmwareUnittestsFWSync(PVRSRV_RGXDEV_INFO * psDevInfo,
					   RGXFW_UNITTESTS * psFWUnitTest)
{
	psFWUnitTest->ui32Status = RGXFW_UNITTEST_FWPING;

	OSWriteMemoryBarrier();

#if defined(PDUMP)
	PDUMPCOMMENTWITHFLAGS(0, "** RGXFW Unittests: Let the fw run");
	DevmemPDumpLoadMem(psDevInfo->psRGXFWUnittestsMemDesc,
			   offsetof(RGXFW_UNITTESTS, ui32Status),
			   sizeof(IMG_UINT32), PDUMP_FLAGS_CONTINUOUS);
#endif
}

/* 
 * RGXFirmwareUnittestsTest1: Test uncounted kicks
 */
static PVRSRV_ERROR RGXFirmwareUnittestsTest1(PVRSRV_RGXDEV_INFO * psDevInfo,
					      RGXFW_UNITTESTS * psFWUnitTest)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PDUMPCOMMENTWITHFLAGS(0, "** RGXFW Unittests: Test1");

	/* Let the fw run from this point */
	RGXFirmwareUnittestsFWSync(psDevInfo, psFWUnitTest);

	/* MTS-kick */
	__MTSScheduleWrite(psDevInfo,
			   RGX_CR_MTS_SCHEDULE_CONTEXT_BGCTX |
			   RGX_CR_MTS_SCHEDULE_TASK_NON_COUNTED,
			   PDUMP_FLAGS_CONTINUOUS);

	eError = RGXFirmwareUnittestsFWWait(psDevInfo, psFWUnitTest);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsTest1: Failed (%u)", eError));
	}

	return eError;
}

/* 
 * RGXFirmwareUnittestsTest2: Test counted kicks, 1 threads
 */
static PVRSRV_ERROR RGXFirmwareUnittestsTest2(PVRSRV_RGXDEV_INFO * psDevInfo,
					      RGXFW_UNITTESTS * psFWUnitTest)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFW_UNITTEST2 sUnitTest2;
	IMG_UINT32 ui32Finished = 0;
	IMG_UINT32 ui32Idx;

	PDUMPCOMMENTWITHFLAGS(0, "** RGXFW Unittests: Test2");

	OSMemSet(&sUnitTest2, 0, sizeof(RGXFW_UNITTEST2));

	for (ui32Idx = 0; ui32Idx < RGXFWIF_DM_MAX_MTS; ui32Idx++) {
		IMG_UINT32 ui32Count =
		    RGXFW_UNITTEST_IS_BGKICK(ui32Idx) ? 20 : 30;

		/* Init the bg counters */
		sUnitTest2.ui32BgKicksDM[ui32Idx] = ui32Count;
		sUnitTest2.ui32BgKicksCounted += ui32Count;

		/* irq kicks are spwan from bg kicks from inside the fw */
		sUnitTest2.ui32IrqKicksDM[ui32Idx] = ui32Count;

		if (RGXFW_UNITTEST_IS_BGKICK(ui32Idx)) {
			sUnitTest2.ui32IrqKicksBg += ui32Count;
		} else {
			sUnitTest2.ui32IrqKicksTimer += ui32Count;
		}
	}

	/* copy the results to the FW */
	OSMemCopy(&psFWUnitTest->sUnitTest2, &sUnitTest2,
		  sizeof(RGXFW_UNITTEST2));

	DevmemPDumpLoadMem(psDevInfo->psRGXFWUnittestsMemDesc,
			   0, sizeof(RGXFW_UNITTESTS), PDUMP_FLAGS_CONTINUOUS);

	/* Let the fw run from this point */
	RGXFirmwareUnittestsFWSync(psDevInfo, psFWUnitTest);

	/* kick MTS for several counted kicks (those spawn irq kicks from inside the fw) */
	while (ui32Finished <
	       sizeof(sUnitTest2.ui32BgKicksDM) / sizeof(IMG_UINT32)) {
		for (ui32Idx = 0;
		     ui32Idx <
		     sizeof(sUnitTest2.ui32BgKicksDM) / sizeof(IMG_UINT32);
		     ui32Idx++) {
			IMG_UINT32 ui32RegValue =
			    RGX_CR_MTS_SCHEDULE_CONTEXT_BGCTX |
			    RGX_CR_MTS_SCHEDULE_TASK_COUNTED;

			if (sUnitTest2.ui32BgKicksDM[ui32Idx] > 0) {
				ui32RegValue |=
				    ((ui32Idx) << RGX_CR_MTS_SCHEDULE_DM_SHIFT)
				    & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK;

				/* MTS-kick */
				__MTSScheduleWrite(psDevInfo, ui32RegValue,
						   PDUMP_FLAGS_CONTINUOUS);

				sUnitTest2.ui32BgKicksDM[ui32Idx]--;
				if (sUnitTest2.ui32BgKicksDM[ui32Idx] == 0)
					ui32Finished++;
			}
		}
	}

	eError = RGXFirmwareUnittestsFWWait(psDevInfo, psFWUnitTest);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittestsTest2: Failed (%u)", eError));
	}

	return eError;
}

/* 
 * RGXFirmwareUnittests: Initialize the different unittests
 */
PVRSRV_ERROR RGXFirmwareUnittests(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	RGXFW_UNITTESTS *psFWUnitTests;
	PVRSRV_ERROR eError = PVRSRV_OK;

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWUnittestsMemDesc,
					  (IMG_VOID **) & psFWUnitTests);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittests: Failed to acquire kernel addr for fw unittest structure (%u)",
			 eError));
		goto RGXFirmwareUnittests_exit;
	}

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS,
			      "** RGXFW Unittests start");

	eError = RGXFirmwareUnittestsTest1(psDevInfo, psFWUnitTests);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittests: fw unittest 1 failed (%u)",
			 eError));
		goto RGXFirmwareUnittests_exit;
	}

	eError = RGXFirmwareUnittestsTest2(psDevInfo, psFWUnitTests);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXFirmwareUnittests: fw unittest 2 failed (%u)",
			 eError));
		goto RGXFirmwareUnittests_exit;
	}

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "** RGXFW Unittests end");

 RGXFirmwareUnittests_exit:

	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWUnittestsMemDesc);
	return eError;

}
#endif

/*!
*******************************************************************************
 @Function		RGXFWSetupSignatureChecks
 @Description	
 @Input			psDevInfo
 
 @Return		PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXFWSetupSignatureChecks(PVRSRV_RGXDEV_INFO * psDevInfo,
					      DEVMEM_MEMDESC **
					      ppsSigChecksMemDesc,
					      IMG_UINT32 ui32SigChecksBufSize,
					      RGXFWIF_SIGBUF_CTL * psSigBufCtl)
{
	PVRSRV_ERROR eError;
	DEVMEM_FLAGS_T uiMemAllocFlags =
	    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE | PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
	    | PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for signature checks");
	eError = DevmemFwAllocate(psDevInfo,
				  ui32SigChecksBufSize,
				  uiMemAllocFlags, ppsSigChecksMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate %d bytes for signature checks (%u)",
			 ui32SigChecksBufSize, eError));
		return eError;
	}

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(&psSigBufCtl->psBuffer,
			      *ppsSigChecksMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	DevmemPDumpLoadMem(*ppsSigChecksMemDesc,
			   0, ui32SigChecksBufSize, PDUMP_FLAGS_CONTINUOUS);

	psSigBufCtl->ui32LeftSizeInRegs =
	    ui32SigChecksBufSize / sizeof(IMG_UINT32);

	return PVRSRV_OK;
}

#if defined(RGXFW_ALIGNCHECKS)
/*!
*******************************************************************************
 @Function		RGXFWSetupAlignChecks
 @Description	
 @Input			psDevInfo
 
 @Return		PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXFWSetupAlignChecks(PVRSRV_RGXDEV_INFO * psDevInfo,
					  RGXFWIF_DEV_VIRTADDR *
					  psAlignChecksDevFW,
					  IMG_UINT32 * pui32RGXFWAlignChecks,
					  IMG_UINT32 ui32RGXFWAlignChecksSize)
{
	IMG_UINT32 aui32RGXFWAlignChecksKM[] = { RGXFW_ALIGN_CHECKS_INIT_KM };
	IMG_UINT32 ui32RGXFWAlingChecksTotal =
	    sizeof(aui32RGXFWAlignChecksKM) + ui32RGXFWAlignChecksSize;
	IMG_UINT32 *paui32AlignChecks;
	PVRSRV_ERROR eError;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for alignment checks");
	eError = DevmemFwAllocate(psDevInfo,
				  ui32RGXFWAlingChecksTotal,
				  PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
				  (PMMETA_PROTECT) |
				  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
				  PVRSRV_MEMALLOCFLAG_UNCACHED,
				  &psDevInfo->psRGXFWAlignChecksMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate %d bytes for alignment checks (%u)",
			 ui32RGXFWAlingChecksTotal, eError));
		goto failAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc,
					  (IMG_VOID **) & paui32AlignChecks);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to acquire kernel addr for alignment checks (%u)",
			 eError));
		goto failAqCpuAddr;
	}

	/* Copy the values */
	OSMemCopy(paui32AlignChecks, &aui32RGXFWAlignChecksKM[0],
		  sizeof(aui32RGXFWAlignChecksKM));
	paui32AlignChecks +=
	    sizeof(aui32RGXFWAlignChecksKM) / sizeof(IMG_UINT32);

	OSMemCopy(paui32AlignChecks, pui32RGXFWAlignChecks,
		  ui32RGXFWAlignChecksSize);

	DevmemPDumpLoadMem(psDevInfo->psRGXFWAlignChecksMemDesc,
			   0,
			   ui32RGXFWAlingChecksTotal, PDUMP_FLAGS_CONTINUOUS);

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(psAlignChecksDevFW,
			      psDevInfo->psRGXFWAlignChecksMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	return PVRSRV_OK;

 failAqCpuAddr:
	DevmemFwFree(psDevInfo->psRGXFWAlignChecksMemDesc);
 failAlloc:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static IMG_VOID RGXFWFreeAlignChecks(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc);
	DevmemFwFree(psDevInfo->psRGXFWAlignChecksMemDesc);
}
#endif

IMG_VOID RGXSetFirmwareAddress(RGXFWIF_DEV_VIRTADDR * ppDest,
			       DEVMEM_MEMDESC * psSrc,
			       IMG_UINT32 uiExtraOffset, IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eError;
	IMG_DEV_VIRTADDR psDevVirtAddr;
	IMG_UINT64 ui64Offset;

	eError = DevmemAcquireDevVirtAddr(psSrc, &psDevVirtAddr);
	PVR_ASSERT(eError == PVRSRV_OK);

	/* Convert to an address in META memmap */
	ui64Offset =
	    psDevVirtAddr.uiAddr + uiExtraOffset - RGX_FIRMWARE_HEAP_BASE;

	/* The biggest offset for the Shared region that can be addressed */
	PVR_ASSERT(ui64Offset <= 4 * RGXFW_SEGMMU_DMAP_SIZE);

	if (ui32Flags & RFW_FWADDR_METACACHED_FLAG) {
		ppDest->ui32Addr =
		    ((IMG_UINT32) ui64Offset) | RGXFW_BOOTLDR_META_ADDR;
	} else {
		ppDest->ui32Addr =
		    ((IMG_UINT32) ui64Offset) | RGXFW_SEGMMU_DMAP_ADDR_START;
	}

	if (ui32Flags & RFW_FWADDR_NOREF_FLAG) {
		DevmemReleaseDevVirtAddr(psSrc);
	}
}

IMG_VOID RGXUnsetFirmwareAddress(DEVMEM_MEMDESC * psSrc)
{
	DevmemReleaseDevVirtAddr(psSrc);
}

/*!
*******************************************************************************
 @Function		RGXSetupKernelCCB
 @Description	Allocate and initialise a kernel CCB
 @Input			psDevInfo
 
 @Return		PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXSetupKernelCCB(PVRSRV_RGXDEV_INFO * psDevInfo,
				      RGXFWIF_INIT * psRGXFWInit,
				      RGXFWIF_DM eKCCBType,
				      IMG_UINT32 ui32NumCmdsLog2,
				      IMG_UINT32 ui32CmdSize)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CTL *psKCCBCtl;
	DEVMEM_FLAGS_T uiCCBCtlMemAllocFlags, uiCCBMemAllocFlags;
	IMG_UINT32 ui32kCCBSize = (1U << ui32NumCmdsLog2);

	/*
	 * FIXME: the write offset need not be writeable by the firmware, indeed may
	 * not even be needed for reading. Consider moving it to its own data
	 * structure.
	 */
	uiCCBCtlMemAllocFlags =
	    PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE | PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
	    | PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	uiCCBMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/*
	   Allocate memory for the kernel CCB control.
	 */
	PDUMPCOMMENT("Allocate memory for kernel CCB control %u", eKCCBType);
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_KCCB_CTL),
				  uiCCBCtlMemAllocFlags,
				  &psDevInfo->
				  apsKernelCCBCtlMemDesc[eKCCBType]);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupKernelCCB: Failed to allocate kernel CCB ctl %u (%u)",
			 eKCCBType, eError));
		goto failCCBCtlMemDescAlloc;
	}

	/*
	   Allocate memory for the kernel CCB.
	   (this will reference further command data in non-shared CCBs)
	 */
	PDUMPCOMMENT("Allocate memory for kernel CCB %u", eKCCBType);
	eError = DevmemFwAllocate(psDevInfo,
				  ui32kCCBSize * ui32CmdSize,
				  uiCCBMemAllocFlags,
				  &psDevInfo->apsKernelCCBMemDesc[eKCCBType]);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupKernelCCB: Failed to allocate kernel CCB %u (%u)",
			 eKCCBType, eError));
		goto failCCBMemDescAlloc;
	}

	/*
	   Map the kernel CCB control to the kernel.
	 */
	eError =
	    DevmemAcquireCpuVirtAddr(psDevInfo->
				     apsKernelCCBCtlMemDesc[eKCCBType],
				     (IMG_VOID **) & psDevInfo->
				     apsKernelCCBCtl[eKCCBType]);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupKernelCCB: Failed to acquire cpu kernel CCB Ctl %u (%u)",
			 eKCCBType, eError));
		goto failCCBCtlMemDescAqCpuVirt;
	}

	/*
	   Map the kernel CCB to the kernel.
	 */
	eError =
	    DevmemAcquireCpuVirtAddr(psDevInfo->apsKernelCCBMemDesc[eKCCBType],
				     (IMG_VOID **) & psDevInfo->
				     apsKernelCCB[eKCCBType]);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupKernelCCB: Failed to acquire cpu kernel CCB %u (%u)",
			 eKCCBType, eError));
		goto failCCBMemDescAqCpuVirt;
	}

	/*
	 * Initialise the kernel CCB control.
	 */
	psKCCBCtl = psDevInfo->apsKernelCCBCtl[eKCCBType];
	psKCCBCtl->ui32WriteOffset = 0;
	psKCCBCtl->ui32ReadOffset = 0;
	psKCCBCtl->ui32WrapMask = ui32kCCBSize - 1;
	psKCCBCtl->ui32CmdSize = ui32CmdSize;

	/*
	 * Set-up RGXFWIfCtl pointers to access the kCCBs
	 */
	RGXSetFirmwareAddress(&psRGXFWInit->psKernelCCBCtl[eKCCBType],
			      psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
			      0, RFW_FWADDR_NOREF_FLAG);

	RGXSetFirmwareAddress(&psRGXFWInit->psKernelCCB[eKCCBType],
			      psDevInfo->apsKernelCCBMemDesc[eKCCBType],
			      0, RFW_FWADDR_NOREF_FLAG);

	psRGXFWInit->eDM[eKCCBType] = eKCCBType;

	/*
	 * Pdump the kernel CCB control.
	 */
	PDUMPCOMMENT("Initialise kernel CCB ctl %d", eKCCBType);
	DevmemPDumpLoadMem(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
			   0, sizeof(RGXFWIF_KCCB_CTL), 0);

	return PVRSRV_OK;

 failCCBMemDescAqCpuVirt:
	DevmemReleaseCpuVirtAddr(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
 failCCBCtlMemDescAqCpuVirt:
	DevmemFwFree(psDevInfo->apsKernelCCBMemDesc[eKCCBType]);
 failCCBMemDescAlloc:
	DevmemFwFree(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
 failCCBCtlMemDescAlloc:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*!
*******************************************************************************
 @Function		RGXFreeKernelCCB
 @Description	Free a kernel CCB
 @Input			psDevInfo
 @Input			eKCCBType
 
 @Return		PVRSRV_ERROR
******************************************************************************/
static IMG_VOID RGXFreeKernelCCB(PVRSRV_RGXDEV_INFO * psDevInfo,
				 RGXFWIF_DM eKCCBType)
{
	DevmemReleaseCpuVirtAddr(psDevInfo->apsKernelCCBMemDesc[eKCCBType]);
	DevmemReleaseCpuVirtAddr(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
	DevmemFwFree(psDevInfo->apsKernelCCBMemDesc[eKCCBType]);
	DevmemFwFree(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
}

/*!
*******************************************************************************

 @Function	RGXSetupFirmware

 @Description

 Setups all the firmware related data

 @Input psDevInfo

 @Return PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR RGXSetupFirmware(PVRSRV_DEVICE_NODE * psDeviceNode,
			      IMG_DEVMEM_SIZE_T ui32FWMemAllocSize,
			      DEVMEM_EXPORTCOOKIE **
			      ppsFWMemAllocServerExportCookie,
			      IMG_DEV_VIRTADDR * psFWMemDevVAddrBase,
			      IMG_BOOL bEnableSignatureChecks,
			      IMG_UINT32 ui32SignatureChecksBufSize,
			      IMG_UINT32 ui32RGXFWAlignChecksSize,
			      IMG_UINT32 * pui32RGXFWAlignChecks,
			      IMG_UINT32 ui32ConfigFlags,
			      RGXFWIF_DEV_VIRTADDR * psRGXFWInitFWAddr)
{
	PVRSRV_ERROR eError;
	DEVMEM_FLAGS_T uiMemAllocFlags;
	RGXFWIF_INIT *psRGXFWInit;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/* 
	 * Set up Allocations for FW code
	 */
	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_CACHE_INCOHERENT |
	    PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate and export memory for fw");

	eError = DevmemFwAllocateExportable(psDeviceNode,
					    ui32FWMemAllocSize,
					    uiMemAllocFlags,
					    &psDevInfo->psRGXFWMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "Failed to allocate fw mem (%u)",
			 eError));
		goto failFWMemDescAlloc;
	}

	eError =
	    DevmemExport(psDevInfo->psRGXFWMemDesc,
			 &psDevInfo->sRGXFWExportCookie);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "Failed to export fw mem (%u)",
			 eError));
		goto failFWMemDescExport;
	}

	eError =
	    DevmemAcquireDevVirtAddr(psDevInfo->psRGXFWMemDesc,
				     psFWMemDevVAddrBase);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "Failed to acquire devVAddr for fw mem (%u)", eError));
		goto failFWMemDescAqDevVirt;
	}

	/*
	 * The FW code must be the first allocation in the firmware heap, otherwise
	 * the bootloader will not work (META will not be able to find the bootloader).
	 */
	PVR_ASSERT(psFWMemDevVAddrBase->uiAddr == RGX_FIRMWARE_HEAP_BASE);

	*ppsFWMemAllocServerExportCookie = &psDevInfo->sRGXFWExportCookie;

	/* Fw init data */
	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;
	/* TODO: Change to Cached */

	PDUMPCOMMENT("Allocate RGXFWIF_INIT structure");
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_INIT),
				  uiMemAllocFlags,
				  &psDevInfo->psRGXFWIfInitMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate %d bytes for fw if ctl (%u)",
			 sizeof(RGXFWIF_INIT), eError));
		goto failFWIfInitMemDescAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc,
					  (IMG_VOID **) & psRGXFWInit);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to acquire kernel fw if ctl (%u)",
			 eError));
		goto failFWIfInitMemDescAqCpuVirt;
	}

	RGXSetFirmwareAddress(psRGXFWInitFWAddr,
			      psDevInfo->psRGXFWIfInitMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	/* FW Trace buffer */
	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
	    PVRSRV_MEMALLOCFLAG_GPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
	    PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	    PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
	    PVRSRV_MEMALLOCFLAG_UNCACHED | PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw trace structure");
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_TRACEBUF),
				  uiMemAllocFlags,
				  &psDevInfo->psRGXFWIfTraceBufCtlMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate %d bytes for fw trace (%u)",
			 sizeof(RGXFWIF_TRACEBUF), eError));
		goto failFWIfTraceBufCtlMemDescAlloc;
	}

	RGXSetFirmwareAddress(&psRGXFWInit->psTraceBufCtl,
			      psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	eError =
	    DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
				     (IMG_VOID **) & psDevInfo->
				     psRGXFWIfTraceBuf);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to acquire kernel tracebuf ctl (%u)",
			 eError));
		goto failFWIfTraceBufCtlMemDescAqCpuVirt;
	}
	/*
	 * Set up kernel TA CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
				   psRGXFWInit,
				   RGXFWIF_DM_TA, RGXFWIF_KCCB_TA_NUMCMDS_LOG2,
				   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate kernel TA CCB"));
		goto failTACCB;
	}

	/*
	 * Set up kernel 3D CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
				   psRGXFWInit,
				   RGXFWIF_DM_3D, RGXFWIF_KCCB_3D_NUMCMDS_LOG2,
				   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate kernel 3D CCB"));
		goto fail3DCCB;
	}

	/*
	 * Set up kernel 2D CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
				   psRGXFWInit,
				   RGXFWIF_DM_2D, RGXFWIF_KCCB_2D_NUMCMDS_LOG2,
				   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate kernel 2D CCB"));
		goto fail2DCCB;
	}

	/*
	 * Set up kernel compute CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
				   psRGXFWInit,
				   RGXFWIF_DM_CDM,
				   RGXFWIF_KCCB_CDM_NUMCMDS_LOG2,
				   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate kernel Compute CCB"));
		goto failCompCCB;
	}

	/*
	 * Set up kernel general purpose CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
				   psRGXFWInit,
				   RGXFWIF_DM_GP, RGXFWIF_KCCB_GP_NUMCMDS_LOG2,
				   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to allocate kernel General Purpose CCB"));
		goto failGPCCB;
	}

	/* Require a minimum amount of memory for the signature buffers */
	if (ui32SignatureChecksBufSize < RGXFW_SIG_BUFFER_SIZE_DEFAULT) {
		ui32SignatureChecksBufSize = RGXFW_SIG_BUFFER_SIZE_DEFAULT;
	}

	/* Setup Signature and Checksum Buffers for TA and 3D */
	eError = RGXFWSetupSignatureChecks(psDevInfo,
					   &psDevInfo->
					   psRGXFWSigTAChecksMemDesc,
					   ui32SignatureChecksBufSize,
					   &psRGXFWInit->
					   asSigBufCtl[RGXFWIF_DM_TA]);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to setup TA signature checks"));
		goto failTASigCheck;
	}
	psDevInfo->ui32SigTAChecksSize = ui32SignatureChecksBufSize;

	eError = RGXFWSetupSignatureChecks(psDevInfo,
					   &psDevInfo->
					   psRGXFWSig3DChecksMemDesc,
					   ui32SignatureChecksBufSize,
					   &psRGXFWInit->
					   asSigBufCtl[RGXFWIF_DM_3D]);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to setup 3D signature checks"));
		goto fail3DSigCheck;
	}
	psDevInfo->ui32Sig3DChecksSize = ui32SignatureChecksBufSize;

#if defined(RGXFW_ALIGNCHECKS)
	eError = RGXFWSetupAlignChecks(psDevInfo,
				       &psRGXFWInit->paui32AlignChecks,
				       pui32RGXFWAlignChecks,
				       ui32RGXFWAlignChecksSize);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXSetupFirmware: Failed to setup alignment checks"));
		goto failAlignCheck;
	}
#endif

#if defined(SUPPORT_RGXFW_UNITTESTS)
	RGXFirmwareUnittestsInit(psDevInfo, psRGXFWInit);
#endif

	/* Fill the remaining bits of fw the init data */
	psRGXFWInit->sPDSExecBase.uiAddr = RGX_PDSCODEDATA_HEAP_BASE;
	psRGXFWInit->sUSCExecBase.uiAddr = RGX_USCCODE_HEAP_BASE;

	psRGXFWInit->ui32ConfigFlags = ui32ConfigFlags;

	PDUMPCOMMENT("Dump RGXFW Init data");
	if (!bEnableSignatureChecks) {
#if defined(PDUMP)
		PDUMPCOMMENT
		    ("(to enable rgxfw signatures place the following line after the RTCONF line)");
		DevmemPDumpLoadMem(psDevInfo->psRGXFWIfInitMemDesc,
				   offsetof(RGXFWIF_INIT, asSigBufCtl),
				   sizeof(RGXFWIF_SIGBUF_CTL) * RGXFWIF_DM_MAX,
				   PDUMP_FLAGS_CONTINUOUS);
#endif
		psRGXFWInit->asSigBufCtl[RGXFWIF_DM_3D].psBuffer.ui32Addr = 0x0;
		psRGXFWInit->asSigBufCtl[RGXFWIF_DM_TA].psBuffer.ui32Addr = 0x0;
	}
#if defined(PDUMP)
#if defined(SUPPORT_RGXFW_LOG)
	PDUMPCOMMENT
	    ("(to enable rgxfw tbi logging place the following line after the RTCONF line)");
	psDevInfo->psRGXFWIfTraceBuf->eLogType = RGXFWIF_LOG_TYPE_TBI;
	DevmemPDumpLoadMem(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			   offsetof(RGXFWIF_TRACEBUF, eLogType),
			   sizeof(RGXFWIF_LOG_TYPE), PDUMP_FLAGS_CONTINUOUS);
	PDUMPCOMMENT
	    ("(to enable rgxfw trace logging place the following line after the RTCONF line)");
	psDevInfo->psRGXFWIfTraceBuf->eLogType = RGXFWIF_LOG_TYPE_TRACE;
	DevmemPDumpLoadMem(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			   offsetof(RGXFWIF_TRACEBUF, eLogType),
			   sizeof(RGXFWIF_LOG_TYPE), PDUMP_FLAGS_CONTINUOUS);
	PDUMPCOMMENT
	    ("(to disable rgxfw logging place the following line after the RTCONF line)");
#endif
	psDevInfo->psRGXFWIfTraceBuf->eLogType = RGXFWIF_LOG_TYPE_NONE;

	PDUMPCOMMENT("Dump rgxfw trace structure");
	DevmemPDumpLoadMem(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			   0, sizeof(RGXFWIF_TRACEBUF), PDUMP_FLAGS_CONTINUOUS);

	DevmemPDumpLoadMem(psDevInfo->psRGXFWIfInitMemDesc,
			   0, sizeof(RGXFWIF_INIT), PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("RTCONF: run-time configuration");

	/* Dump the config options so they can be edited.
	 * 
	 * TODO: Need new DevmemPDumpWRW API which writes a WRW to load ui32ConfigFlags
	 */
	PDUMPCOMMENT("(Set the FW config options here)");
	PDUMPCOMMENT("( bit 0: Ctx Switch TA Enable)");
	PDUMPCOMMENT("( bit 1: Ctx Switch 3D Enable)");
	PDUMPCOMMENT("( bit 2: Ctx Switch CDM Enable)");
	PDUMPCOMMENT("( bit 3: Ctx Switch Rand mode)");
	PDUMPCOMMENT("( bit 4: Ctx Switch Soft Reset Enable)");
	PDUMPCOMMENT("( bit 5: Enable 2nd thread)");
	PDUMPCOMMENT("( bit 6: Rascal+Dust Power Island)");
	DevmemPDumpLoadMemValue(psDevInfo->psRGXFWIfInitMemDesc,
				offsetof(RGXFWIF_INIT, ui32ConfigFlags),
				psRGXFWInit->ui32ConfigFlags,
				PDUMP_FLAGS_CONTINUOUS);
#endif

	/* We don't need access to the fw init data structure anymore */
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
	psRGXFWInit = IMG_NULL;

	return PVRSRV_OK;

#if defined(RGXFW_ALIGNCHECKS)
 failAlignCheck:
	DevmemFwFree(psDevInfo->psRGXFWSig3DChecksMemDesc);
#endif
 fail3DSigCheck:
	DevmemFwFree(psDevInfo->psRGXFWSigTAChecksMemDesc);
 failTASigCheck:
	RGXFreeKernelCCB(psDevInfo, RGXFWIF_DM_GP);

 failGPCCB:
	RGXFreeKernelCCB(psDevInfo, RGXFWIF_DM_CDM);
 failCompCCB:
	RGXFreeKernelCCB(psDevInfo, RGXFWIF_DM_2D);
 fail2DCCB:
	RGXFreeKernelCCB(psDevInfo, RGXFWIF_DM_3D);
 fail3DCCB:
	RGXFreeKernelCCB(psDevInfo, RGXFWIF_DM_TA);

 failTACCB:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
 failFWIfTraceBufCtlMemDescAqCpuVirt:
	DevmemFwFree(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);

 failFWIfTraceBufCtlMemDescAlloc:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
 failFWIfInitMemDescAqCpuVirt:
	DevmemFwFree(psDevInfo->psRGXFWIfInitMemDesc);

 failFWIfInitMemDescAlloc:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWMemDesc);
 failFWMemDescAqDevVirt:
	DevmemUnexport(psDevInfo->psRGXFWMemDesc,
		       &psDevInfo->sRGXFWExportCookie);
 failFWMemDescExport:
	DevmemFwFree(psDevInfo->psRGXFWMemDesc);
 failFWMemDescAlloc:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*!
*******************************************************************************

 @Function	RGXFreeFirmware

 @Description

 Frees all the firmware-related allocations

 @Input psDevInfo

 @Return PVRSRV_ERROR

******************************************************************************/
IMG_VOID RGXFreeFirmware(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	RGXFWIF_DM eKCCBType;

	for (eKCCBType = 0; eKCCBType < RGXFWIF_DM_MAX; eKCCBType++) {
		if (psDevInfo->apsKernelCCBMemDesc[eKCCBType]) {
			RGXFreeKernelCCB(psDevInfo, eKCCBType);
		}
	}

	if (psDevInfo->psRGXFWMemDesc) {
		/* Free fw code */
		PDUMPCOMMENT("Freeing FW memory");
		DevmemReleaseDevVirtAddr(psDevInfo->psRGXFWMemDesc);
		DevmemFwFree(psDevInfo->psRGXFWMemDesc);
	}
#if defined(RGXFW_ALIGNCHECKS)
	if (psDevInfo->psRGXFWAlignChecksMemDesc) {
		RGXFWFreeAlignChecks(psDevInfo);
	}
#endif

	if (psDevInfo->psRGXFWSigTAChecksMemDesc) {
		DevmemFwFree(psDevInfo->psRGXFWSigTAChecksMemDesc);
	}

	if (psDevInfo->psRGXFWSig3DChecksMemDesc) {
		DevmemFwFree(psDevInfo->psRGXFWSig3DChecksMemDesc);
	}
#if defined(SUPPORT_RGXFW_UNITTESTS)
	if (psDevInfo->psRGXFWUnittestsMemDesc) {
		DevmemFwFree(psDevInfo->psRGXFWUnittestsMemDesc);
	}
#endif

	if (psDevInfo->psRGXFWIfTraceBufCtlMemDesc) {
		DevmemReleaseCpuVirtAddr(psDevInfo->
					 psRGXFWIfTraceBufCtlMemDesc);
		psDevInfo->psRGXFWIfTraceBuf = IMG_NULL;
		DevmemFwFree(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
	}

	if (psDevInfo->psRGXFWIfInitMemDesc) {
		DevmemFwFree(psDevInfo->psRGXFWIfInitMemDesc);
	}
}

/******************************************************************************
 FUNCTION	: RGXStartFirmware

 PURPOSE	: Attempts to obtain a slot in the Kernel CCB

 PARAMETERS	: psDevInfo

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR RGXStartFirmware(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Start the firmware\n");

#if defined(SUPPORT_META_SLAVE_BOOT)
	/*
	 * Run init script.
	 */
	PDUMPCOMMENT("Start of RGX initialisation script (Slave boot)");
	eError =
	    RGXRunScript(psDevInfo, psDevInfo->sScripts.asInitCommands,
			 RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "RGXStart: RGXRunScript failed (%d)",
			 eError));
		return eError;
	}
	PDUMPCOMMENT("End of RGX initialisation script");
#else
	PDUMPCOMMENT("RGX firmware Master boot");
#endif

#if defined(SUPPORT_RGXFW_UNITTESTS)
	RGXFirmwareUnittests(psDevInfo);
#endif

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS,
			      "Firmware startup complete\n");

	return eError;
}

/******************************************************************************
 FUNCTION	: RGXAcquireKernelCCBSlot

 PURPOSE	: Attempts to obtain a slot in the Kernel CCB

 PARAMETERS	: psCCB - the CCB
			: Address of space if available, IMG_NULL otherwise

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXAcquireKernelCCBSlot(RGXFWIF_KCCB_CTL * psKCCBCtl,
					    IMG_UINT32 * pui32Offset)
{
	IMG_UINT32 ui32OldWriteOffset, ui32NewWriteOffset;

	ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
	ui32NewWriteOffset = (ui32OldWriteOffset + 1) & psKCCBCtl->ui32WrapMask;

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US) {
		if (ui32NewWriteOffset != psKCCBCtl->ui32ReadOffset) {
			*pui32Offset = ui32NewWriteOffset;
			return PVRSRV_OK;
		}

		OSWaitus(MAX_HW_TIME_US / WAIT_TRY_COUNT);
	}
	END_LOOP_UNTIL_TIMEOUT();

	/* Time out on waiting for CCB space */
	return PVRSRV_ERROR_KERNEL_CCB_FULL;
}

/*!
******************************************************************************

 @Function	_RGXScheduleCommand

 @Description - Submits a CCB command and kicks the firmware

 @Input psDevInfo - pointer to device info
 @Input eKCCBType - see RGXFWIF_CMD_*
 @Input pvKCCBCmd - kernel CCB command
 @Input ui32CallerID - KERNEL_ID or ISR_ID
 @Input ui32PDumpFlags

 @Return ui32Error - success or failure

******************************************************************************/
PVRSRV_ERROR _RGXScheduleCommand(PVRSRV_RGXDEV_INFO * psDevInfo,
				 RGXFWIF_DM eKCCBType,
				 RGXFWIF_KCCB_CMD * psKCCBCmd,
				 IMG_UINT32 ui32CmdSize,
				 IMG_BOOL bPDumpContinuous)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CTL *psKCCBCtl = psDevInfo->apsKernelCCBCtl[eKCCBType];
	IMG_UINT8 *pui8KCCB = psDevInfo->apsKernelCCB[eKCCBType];
	IMG_UINT32 ui32WriteOffset;
	PDUMP_FLAGS_T uiPdumpFlags =
	    bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS : 0;
#if defined(PDUMP)
	IMG_BOOL bIsInCaptureRange = IMG_FALSE;
#endif

	PVR_ASSERT(eKCCBType != RGXFWIF_DM_GP || bPDumpContinuous);

	/* FIXME: this function should consider power/locking */

	PVR_ASSERT(ui32CmdSize == psKCCBCtl->ui32CmdSize);

	/*
	 * Acquire a slot in the CCB.
	 */
	eError = RGXAcquireKernelCCBSlot(psKCCBCtl, &ui32WriteOffset);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_RGXScheduleCommand failed to acquire CCB slot. Type:%u Error:%u",
			 eKCCBType, eError));
		RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
		goto _RGXScheduleCommand_Exit;
	}

	/*
	 * Copy the command into the CCB.
	 */
	OSMemCopy(&pui8KCCB
		  [psKCCBCtl->ui32WriteOffset * psKCCBCtl->ui32CmdSize],
		  psKCCBCmd, psKCCBCtl->ui32CmdSize);

#if defined(PDUMP)
	{
		IMG_BOOL bDumpInitialState = IMG_FALSE;

		PDumpIsCaptureFrameKM(&bIsInCaptureRange);

		if ((bIsInCaptureRange
		     && !psDevInfo->abDumpedKCCBCtlAlready[eKCCBType])
		    || (!bIsInCaptureRange && bPDumpContinuous)) {
			RGXFWIF_KCCB_CTL *psKCCBCtl =
			    psDevInfo->apsKernelCCBCtl[eKCCBType];

			if (psKCCBCtl->ui32ReadOffset !=
			    psKCCBCtl->ui32WriteOffset) {
				/* Wait until the fw catches-up */
				PVR_DPF((PVR_DBG_WARNING,
					 "_RGXScheduleCommand: waiting on fw to catch-up. DM: %d, roff: %d, woff: %d",
					 eKCCBType, psKCCBCtl->ui32ReadOffset,
					 psKCCBCtl->ui32WriteOffset));

				PVRSRVPollForValueKM(&psKCCBCtl->ui32ReadOffset,
						     psKCCBCtl->ui32WriteOffset,
						     0xFFFFFFFF);
			}

			bDumpInitialState =
			    (IMG_BOOL) (eKCCBType != RGXFWIF_DM_GP);
		}

		bDumpInitialState = bDumpInitialState
		    || (eKCCBType == RGXFWIF_DM_GP
			&& !psDevInfo->abDumpedKCCBCtlAlready[RGXFWIF_DM_GP]);

		if (bDumpInitialState) {
			PDUMPCOMMENTWITHFLAGS(uiPdumpFlags,
					      "Initial state of kernel CCB Control(%d), roff: %d",
					      eKCCBType,
					      psKCCBCtl->ui32ReadOffset);
			DevmemPDumpLoadMem(psDevInfo->
					   apsKernelCCBCtlMemDesc[eKCCBType], 0,
					   sizeof(RGXFWIF_KCCB_CTL),
					   uiPdumpFlags);
			if (bIsInCaptureRange || eKCCBType == RGXFWIF_DM_GP) {
				psDevInfo->abDumpedKCCBCtlAlready[eKCCBType] =
				    IMG_TRUE;
			}
		}

		PDUMPCOMMENTWITHFLAGS(uiPdumpFlags,
				      "Dump kCCB(%d) cmd, woff = %d", eKCCBType,
				      psKCCBCtl->ui32WriteOffset);
		DevmemPDumpLoadMem(psDevInfo->apsKernelCCBMemDesc[eKCCBType],
				   psKCCBCtl->ui32WriteOffset *
				   psKCCBCtl->ui32CmdSize,
				   psKCCBCtl->ui32CmdSize, uiPdumpFlags);
	}
#endif

	/* Move past the current command */
	psKCCBCtl->ui32WriteOffset = ui32WriteOffset;

#if defined(PDUMP)
	PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "Dump kCCBCtl(%d) woff: %d",
			      eKCCBType, psKCCBCtl->ui32WriteOffset);
	DevmemPDumpLoadMem(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
			   offsetof(RGXFWIF_KCCB_CTL, ui32WriteOffset),
			   sizeof(IMG_UINT32), uiPdumpFlags);

#endif

	OSWriteMemoryBarrier();

	PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "MTS kick for kernel CCB %d",
			      eKCCBType);
	OSMemoryBarrier();

	/*
	 * Kick the MTS to schedule the firmware.
	 */
	{
		IMG_UINT32 ui32MTSRegVal =
		    (eKCCBType & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) |
		    RGX_CR_MTS_SCHEDULE_TASK_COUNTED;

		__MTSScheduleWrite(psDevInfo, ui32MTSRegVal, uiPdumpFlags);
	}

	/* Memory barrier may be required to flush the write-combine buffer */
	OSMemoryBarrier();

#if defined (NO_HARDWARE)
	/* keep the roff updated because fw isn't there to update it */
	psKCCBCtl->ui32ReadOffset = psKCCBCtl->ui32WriteOffset;
#endif

 _RGXScheduleCommand_Exit:
	return eError;
}

/*!
******************************************************************************

 @Function	RGXScheduleCommand

 @Description - Submits a CCB command and kicks the firmware but first schedules
                any commands which have to happen before handle  

 @Input psDevInfo - pointer to device info
 @Input eKCCBType - see RGXFWIF_CMD_*
 @Input pvKCCBCmd - kernel CCB command
 @Input ui32CmdSize -
 @Input bPDumpContinuous - TRUE if the pdump flags should be continuous

 @Return ui32Error - success or failure

******************************************************************************/
PVRSRV_ERROR RGXScheduleCommand(PVRSRV_RGXDEV_INFO * psDevInfo,
				RGXFWIF_DM eKCCBType,
				RGXFWIF_KCCB_CMD * psKCCBCmd,
				IMG_UINT32 ui32CmdSize,
				IMG_BOOL bPDumpContinuous)
{
	PVRSRV_ERROR eError;

	if ((eKCCBType == RGXFWIF_DM_3D) || (eKCCBType == RGXFWIF_DM_2D)) {
		/* This handles the no operation case */
		OSCPUOperation(psDevInfo->uiCacheOp);
		psDevInfo->uiCacheOp = PVRSRV_CACHE_OP_NONE;
	}

	eError = RGXPreKickCacheCommand(psDevInfo);
	if (eError != PVRSRV_OK)
		goto RGXScheduleCommand_exit;

	eError =
	    _RGXScheduleCommand(psDevInfo, eKCCBType, psKCCBCmd, ui32CmdSize,
				bPDumpContinuous);
	if (eError != PVRSRV_OK)
		goto RGXScheduleCommand_exit;

 RGXScheduleCommand_exit:
	return eError;
}

PVRSRV_ERROR RGXInitFWCommonContext(RGXFWIF_FWCOMMONCONTEXT * psFWComContext,
				    DEVMEM_MEMDESC * psCCBMemDesc,
				    DEVMEM_MEMDESC * psCCBCtlMemDesc,
				    DEVMEM_MEMDESC * psFWMemContextMemDesc,
				    IMG_UINT32 ui32Priority,
				    IMG_DEV_VIRTADDR * psMCUFenceAddr,
				    RGX_FWCOMCTX_CLEANUP * psCleanupData)
{
	/* Save what we need for clean up */
	psCleanupData->psCCBMemDesc = psCCBMemDesc;
	psCleanupData->psCCBCtlMemDesc = psCCBCtlMemDesc;
	psCleanupData->psFWMemContextMemDesc = psFWMemContextMemDesc;

	/*
	 * Set the firmware CCB device addresses in the firmware common context.
	 */
	RGXSetFirmwareAddress(&psFWComContext->psCCB, psCCBMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE |
			      RFW_FWADDR_METACACHED_FLAG);
	RGXSetFirmwareAddress(&psFWComContext->psCCBCtl, psCCBCtlMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE);

	/*
	 * And the memory context device address.
	 */
	RGXSetFirmwareAddress(&psFWComContext->psFWMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	/* Setup the cleanup state */
	psFWComContext->sCleanupState.eCleanupState = RGXFWIF_CLEANUP_NONE;
	psFWComContext->ui32Priority = ui32Priority;
	if (psMCUFenceAddr != IMG_NULL) {
		psFWComContext->ui64MCUFenceAddr = psMCUFenceAddr->uiAddr;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXDeinitFWCommonContext(RGX_FWCOMCTX_CLEANUP * psCleanupData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	/*
	 * Unset the firmware CCB device addr to release the virtual addresses.
	 */
	RGXUnsetFirmwareAddress(psCleanupData->psCCBMemDesc);
	RGXUnsetFirmwareAddress(psCleanupData->psCCBCtlMemDesc);

	/*
	 * And the memory context device address.
	 */
	RGXUnsetFirmwareAddress(psCleanupData->psFWMemContextMemDesc);

	return eError;
}

PVRSRV_ERROR RGXScheduleCommandAndWait(PVRSRV_RGXDEV_INFO * psDevInfo,
				       RGXFWIF_DM eDM,
				       RGXFWIF_KCCB_CMD * psKCCBCmd,
				       IMG_UINT32 ui32CmdSize,
				       IMG_UINT32 * puiSyncObjDevVAddr,
				       IMG_UINT32 * puiUpdateValue,
				       IMG_BOOL bPDumpContinuous)
{
	PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim;
	PVRSRV_ERROR eError;

	/* Allocate a sync primitive */
	eError =
	    SyncPrimAlloc(psDevInfo->psDeviceNode->hSyncPrimContext,
			  &psSyncPrim);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXScheduleCommandAndWait: Failed to allocate sync primitive with error (%u)",
			 eError));
		goto SyncPrimAllocError;
	}

	SyncPrimSet(psSyncPrim, 0);
	*puiSyncObjDevVAddr = SyncPrimGetFirmwareAddr(psSyncPrim);
	*puiUpdateValue = 1;

	PDUMPCOMMENT
	    ("RGXScheduleCommandAndWait: Submit Kernel SyncPrim [0x%08x] to DM %d ",
	     *puiSyncObjDevVAddr, eDM);

	/* submit the sync primitive to the kernel CCB */
	eError = RGXScheduleCommand(psDevInfo,
				    eDM,
				    psKCCBCmd,
				    sizeof(RGXFWIF_KCCB_CMD), bPDumpContinuous);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXScheduleCommandAndWait: Failed to schedule Kernel SyncPrim with error (%u)",
			 eError));
		goto ScheduleError;
	}

	/* Wait for sync primitive to be updated */
#if defined(PDUMP)
	SyncPrimPDumpPol(psSyncPrim,
			 1,
			 0xffffffff,
			 PDUMP_POLL_OPERATOR_EQUAL,
			 bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS : 0);
#endif

	for (;;) {
		/* FIXME: Need to re-think PVRSRVLocking */
		OSSetKeepPVRLock();
		eError =
		    PVRSRVWaitForValueKM(psSyncPrim->pui32LinAddr, 1,
					 0xffffffff, 5);
		OSSetReleasePVRLock();

		if (eError == PVRSRV_ERROR_TIMEOUT) {
			PVR_DPF((PVR_DBG_WARNING,
				 "RGXScheduleCommandAndWait: PVRSRVWaitForValueKM timed out. Dump debug information."));

			RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
			continue;
		} else {
			break;
		}

	}

 ScheduleError:
	SyncPrimFree(psSyncPrim);

 SyncPrimAllocError:
	return eError;
}

PVRSRV_ERROR RGXWaitForFWOp(PVRSRV_RGXDEV_INFO * psDevInfo,
			    RGXFWIF_DM eDM, IMG_BOOL bPDumpContinuous)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sCmdSyncPrim;

	/* prepare a sync command */
	sCmdSyncPrim.eCmdType = RGXFWIF_KCCB_CMD_SYNC;

	eError = RGXScheduleCommandAndWait(psDevInfo,
					   eDM,
					   &sCmdSyncPrim,
					   sizeof(RGXFWIF_KCCB_CMD),
					   &sCmdSyncPrim.uCmdData.sSyncData.
					   uiSyncObjDevVAddr,
					   &sCmdSyncPrim.uCmdData.sSyncData.
					   uiUpdateVal, bPDumpContinuous);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXWaitForFWOp: Failed to schedule Kernel SyncPrim with error (%u)",
			 eError));
	}

	return eError;
}

/******************************************************************************
 End of file (rgxfwutils.c)
******************************************************************************/
