/*************************************************************************/ /*!
@File
@Title          RGX firmware utility routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware utility routines
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

#include <stddef.h>

#include "rgxdefs_km.h"
#include "rgx_fwif_km.h"
#include "pdump_km.h"
#include "osfunc.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "devicemem_server.h"
#include "pvr_debug.h"
#include "rgxfwutils.h"
#include "rgx_fwif.h"
#include "rgx_fwif_alignchecks_km.h"
#include "rgx_fwif_resetframework.h"
#include "rgxheapconfig.h"
#include "pvrsrv.h"
#include "rgxinit.h"
#include "rgxmem.h"
#include "rgxutils.h"
#include "sync_internal.h"
#include "tlstream.h"

#ifdef __linux__
#include <linux/kernel.h>	// sprintf
#include <linux/string.h>	// strncpy, strlen
#else
#include <stdio.h>
#endif


#define RGXFWIF_KCCB_TA_NUMCMDS_LOG2	(6)
#define RGXFWIF_KCCB_3D_NUMCMDS_LOG2	(6)
#define RGXFWIF_KCCB_2D_NUMCMDS_LOG2	(6)
#define RGXFWIF_KCCB_CDM_NUMCMDS_LOG2	(6)
#define RGXFWIF_KCCB_GP_NUMCMDS_LOG2	(6)

static void __MTSScheduleWrite(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT32 ui32Value)
{
	/* ensure memory is flushed before kicking MTS */
	OSWriteMemoryBarrier();

	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_MTS_SCHEDULE, ui32Value);

	/* ensure the MTS kick goes through before continuing */
	OSMemoryBarrier();
}


/*!
*******************************************************************************
 @Function		RGXFWSetupSignatureChecks
 @Description	
 @Input			psDevInfo
 
 @Return		PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXFWSetupSignatureChecks(PVRSRV_RGXDEV_INFO* psDevInfo,
											  DEVMEM_MEMDESC** ppsSigChecksMemDesc, 
											  IMG_UINT32 ui32SigChecksBufSize,
											  RGXFWIF_SIGBUF_CTL *psSigBufCtl)
{
	PVRSRV_ERROR	eError;
	DEVMEM_FLAGS_T	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
									  PVRSRV_MEMALLOCFLAG_GPU_READABLE | 
					                  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
									  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
									  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | 
									  PVRSRV_MEMALLOCFLAG_UNCACHED |
									  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for signature checks");
	eError = DevmemFwAllocate(psDevInfo,
							ui32SigChecksBufSize,
							uiMemAllocFlags,
							ppsSigChecksMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for signature checks (%u)",
					ui32SigChecksBufSize,
					eError));
		return eError;
	}

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(&psSigBufCtl->psBuffer,
						  *ppsSigChecksMemDesc,
						  0, RFW_FWADDR_NOREF_FLAG);

	DevmemPDumpLoadMem(	*ppsSigChecksMemDesc,
						0,
						ui32SigChecksBufSize,
						PDUMP_FLAGS_CONTINUOUS);

	psSigBufCtl->ui32LeftSizeInRegs = ui32SigChecksBufSize / sizeof(IMG_UINT32);

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
static PVRSRV_ERROR RGXFWSetupAlignChecks(PVRSRV_RGXDEV_INFO* psDevInfo, 
								RGXFWIF_DEV_VIRTADDR	*psAlignChecksDevFW,
								IMG_UINT32				*pui32RGXFWAlignChecks,
								IMG_UINT32				ui32RGXFWAlignChecksSize)
{
	IMG_UINT32		aui32RGXFWAlignChecksKM[] = { RGXFW_ALIGN_CHECKS_INIT_KM };
	IMG_UINT32		ui32RGXFWAlingChecksTotal = sizeof(aui32RGXFWAlignChecksKM) + ui32RGXFWAlignChecksSize;
	IMG_UINT32*		paui32AlignChecks;
	PVRSRV_ERROR	eError;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for alignment checks");
	eError = DevmemFwAllocate(psDevInfo,
							ui32RGXFWAlingChecksTotal,
							PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | PVRSRV_MEMALLOCFLAG_UNCACHED, 
							&psDevInfo->psRGXFWAlignChecksMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for alignment checks (%u)",
					ui32RGXFWAlingChecksTotal,
					eError));
		goto failAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc,
									(IMG_VOID **)&paui32AlignChecks);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire kernel addr for alignment checks (%u)",
					eError));
		goto failAqCpuAddr;
	}

	/* Copy the values */
	OSMemCopy(paui32AlignChecks, &aui32RGXFWAlignChecksKM[0], sizeof(aui32RGXFWAlignChecksKM));
	paui32AlignChecks += sizeof(aui32RGXFWAlignChecksKM)/sizeof(IMG_UINT32);

	OSMemCopy(paui32AlignChecks, pui32RGXFWAlignChecks, ui32RGXFWAlignChecksSize);

	DevmemPDumpLoadMem(	psDevInfo->psRGXFWAlignChecksMemDesc,
						0,
						ui32RGXFWAlingChecksTotal,
						PDUMP_FLAGS_CONTINUOUS);

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

static IMG_VOID RGXFWFreeAlignChecks(PVRSRV_RGXDEV_INFO* psDevInfo)
{
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc);
	DevmemFwFree(psDevInfo->psRGXFWAlignChecksMemDesc);
}
#endif


IMG_VOID RGXSetFirmwareAddress(RGXFWIF_DEV_VIRTADDR	*ppDest,
							   DEVMEM_MEMDESC		*psSrc,
							   IMG_UINT32			uiExtraOffset,
							   IMG_UINT32			ui32Flags)
{
	PVRSRV_ERROR		eError;
	IMG_DEV_VIRTADDR	psDevVirtAddr;
	IMG_UINT64			ui64Offset;

	eError = DevmemAcquireDevVirtAddr(psSrc, &psDevVirtAddr);
	PVR_ASSERT(eError == PVRSRV_OK);

	/* Convert to an address in META memmap */
	ui64Offset = psDevVirtAddr.uiAddr + uiExtraOffset - RGX_FIRMWARE_HEAP_BASE;

	/* The biggest offset for the Shared region that can be addressed */
	PVR_ASSERT(ui64Offset <= 3*RGXFW_SEGMMU_DMAP_SIZE);

	if (ui32Flags & RFW_FWADDR_METACACHED_FLAG)
	{
		ppDest->ui32Addr = ((IMG_UINT32) ui64Offset) | RGXFW_BOOTLDR_META_ADDR;
	}
	else
	{
		ppDest->ui32Addr = ((IMG_UINT32) ui64Offset) | RGXFW_SEGMMU_DMAP_ADDR_START;
	}

	if (ui32Flags & RFW_FWADDR_NOREF_FLAG)
	{
		DevmemReleaseDevVirtAddr(psSrc);
	}
}


IMG_VOID RGXUnsetFirmwareAddress(DEVMEM_MEMDESC *psSrc)
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
static PVRSRV_ERROR RGXSetupKernelCCB(PVRSRV_RGXDEV_INFO 	*psDevInfo, 
									  RGXFWIF_INIT			*psRGXFWInit,
									  RGXFWIF_DM			eKCCBType,
									  IMG_UINT32			ui32NumCmdsLog2,
									  IMG_UINT32			ui32CmdSize)
{
	PVRSRV_ERROR		eError;
	RGXFWIF_KCCB_CTL	*psKCCBCtl;
	DEVMEM_FLAGS_T		uiCCBCtlMemAllocFlags, uiCCBMemAllocFlags;
	IMG_UINT32			ui32kCCBSize = (1U << ui32NumCmdsLog2);


	/*
	 * FIXME: the write offset need not be writeable by the firmware, indeed may
	 * not even be needed for reading. Consider moving it to its own data
	 * structure.
	 */
	uiCCBCtlMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
							PVRSRV_MEMALLOCFLAG_UNCACHED | 
							 PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	uiCCBMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
						 PVRSRV_MEMALLOCFLAG_GPU_READABLE |
						 PVRSRV_MEMALLOCFLAG_CPU_READABLE |
						 PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
						 PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
						 PVRSRV_MEMALLOCFLAG_UNCACHED | 
						 PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/*
		Allocate memory for the kernel CCB control.
	*/
	PDUMPCOMMENT("Allocate memory for kernel CCB control %u", eKCCBType);
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_KCCB_CTL),
							uiCCBCtlMemAllocFlags,
                            &psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupKernelCCB: Failed to allocate kernel CCB ctl %u (%u)",
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

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupKernelCCB: Failed to allocate kernel CCB %u (%u)",
				eKCCBType, eError));
		goto failCCBMemDescAlloc;
	}

	/*
		Map the kernel CCB control to the kernel.
	*/
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
                                      (IMG_VOID **)&psDevInfo->apsKernelCCBCtl[eKCCBType]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupKernelCCB: Failed to acquire cpu kernel CCB Ctl %u (%u)",
				eKCCBType, eError));
		goto failCCBCtlMemDescAqCpuVirt;
	}

	/*
		Map the kernel CCB to the kernel.
	*/
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->apsKernelCCBMemDesc[eKCCBType],
                                      (IMG_VOID **)&psDevInfo->apsKernelCCB[eKCCBType]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupKernelCCB: Failed to acquire cpu kernel CCB %u (%u)",
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
					   0,
					   sizeof(RGXFWIF_KCCB_CTL),
					   0);

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
static IMG_VOID RGXFreeKernelCCB(PVRSRV_RGXDEV_INFO 	*psDevInfo,
								 RGXFWIF_DM				eKCCBType)
{
	DevmemReleaseCpuVirtAddr(psDevInfo->apsKernelCCBMemDesc[eKCCBType]);
	DevmemReleaseCpuVirtAddr(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
	DevmemFwFree(psDevInfo->apsKernelCCBMemDesc[eKCCBType]);
	DevmemFwFree(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType]);
}

static IMG_VOID RGXSetupBIFFaultReadRegisterRollback(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PMR *psPMR;
	
	if (psDevInfo->psRGXBIFFaultAddressMemDesc)
	{
		if (DevmemServerGetImportHandle(psDevInfo->psRGXBIFFaultAddressMemDesc,(IMG_VOID **)&psPMR) == PVRSRV_OK)
		{
			PMRUnlockSysPhysAddresses(psPMR);
		}
		DevmemFwFree(psDevInfo->psRGXBIFFaultAddressMemDesc);
	}
}

static PVRSRV_ERROR RGXSetupBIFFaultReadRegister(PVRSRV_DEVICE_NODE	*psDeviceNode, RGXFWIF_INIT *psRGXFWInit)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	IMG_UINT32			*pui32MemoryVirtAddr;
	IMG_UINT32			i;
	IMG_SIZE_T			ui32PageSize;
	DEVMEM_FLAGS_T		uiMemAllocFlags;
	PVRSRV_RGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice;
	PMR					*psPMR;

	ui32PageSize = OSGetPageSize();

	/* Allocate page of memory for use by RGX_CR_BIF_FAULT_READ */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
						PVRSRV_MEMALLOCFLAG_GPU_READABLE |
						PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_CPU_READABLE |
						PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
						PVRSRV_MEMALLOCFLAG_UNCACHED;
	
	psDevInfo->psRGXBIFFaultAddressMemDesc = IMG_NULL;
	eError = DevmemFwAllocateExportable(psDeviceNode,
										ui32PageSize,
										uiMemAllocFlags,
										&psDevInfo->psRGXBIFFaultAddressMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Failed to allocate mem for RGX_CR_BIF_FAULT_READ (%u)",
				eError));
		goto failBIFFaultAddressDescAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXBIFFaultAddressMemDesc,
									  (IMG_VOID **)&pui32MemoryVirtAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire mem for RGX_CR_BIF_FAULT_READ (%u)",
				eError));
		goto failBIFFaultAddressDescAqCpuVirt;
	}

	for (i=0; i<(RGX_CR_BIF_FAULT_READ_ADDRESS_ALIGNSIZE/sizeof(IMG_UINT32)); i++)
	{
		*(pui32MemoryVirtAddr + i) = 0xDEADBEEF;
	}

	eError = DevmemServerGetImportHandle(psDevInfo->psRGXBIFFaultAddressMemDesc,(IMG_VOID **)&psPMR);
		
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error getting PMR for RGX_CR_BIF_FAULT_READ MemDesc (%u)",
				eError));
		
		goto failBIFFaultAddressDescGetPMR;
	}
	else
	{
		IMG_BOOL bValid;
		
		eError = PMRLockSysPhysAddresses(psPMR,OSGetPageShift());
			
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error locking physical address for RGX_CR_BIF_FAULT_READ MemDesc (%u)",
					eError));
			
			goto failBIFFaultAddressDescLockPhys;
		}
			
		eError = PMR_DevPhysAddr(psPMR,0,&(psRGXFWInit->sBifFaultPhysAddr),&bValid);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error getting physical address for RGX_CR_BIF_FAULT_READ MemDesc (%u)",
					eError));
			
			goto failBIFFaultAddressDescGetPhys;
		}

		if (bValid)
		{
			//PVR_DPF((PVR_DBG_MESSAGE,"RGXSetupFirmware: Got physical address for RGX_CR_BIF_FAULT_READ MemDesc (0x%llX)",
			//		psRGXFWInit->sBifFaultPhysAddr.uiAddr));
		}
		else
		{
			psRGXFWInit->sBifFaultPhysAddr.uiAddr = 0;
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed getting physical address for RGX_CR_BIF_FAULT_READ MemDesc - invalid page (0x%llX)",
					psRGXFWInit->sBifFaultPhysAddr.uiAddr));

			goto failBIFFaultAddressDescGetPhys;
		}
	}

	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXBIFFaultAddressMemDesc);
	
	return PVRSRV_OK;

failBIFFaultAddressDescGetPhys:
	PMRUnlockSysPhysAddresses(psPMR);

failBIFFaultAddressDescLockPhys:

failBIFFaultAddressDescGetPMR:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXBIFFaultAddressMemDesc);

failBIFFaultAddressDescAqCpuVirt:
	DevmemFwFree(psDevInfo->psRGXBIFFaultAddressMemDesc);

failBIFFaultAddressDescAlloc:

	return eError;
}

static IMG_VOID RGXHwBrn37200Rollback(PVRSRV_RGXDEV_INFO *psDevInfo)
{
#if defined(FIX_HW_BRN_37200)
	DevmemFwFree(psDevInfo->psRGXFWHWBRN37200MemDesc);
#endif
}

static PVRSRV_ERROR RGXHwBrn37200(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PVRSRV_ERROR			eError = PVRSRV_OK;

#if defined(FIX_HW_BRN_37200)
	struct _DEVMEM_HEAP_	*psBRNHeap;
	DEVMEM_FLAGS_T			uiFlags;
	IMG_DEV_VIRTADDR		sTmpDevVAddr;
	IMG_SIZE_T				ui32PageSize;

	ui32PageSize = OSGetPageSize();
	
	uiFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
				PVRSRV_MEMALLOCFLAG_GPU_READABLE | 
				PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | 
				PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
				PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
				PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	eError = DevmemFindHeapByName(psDevInfo->psKernelDevmemCtx,
							  "HWBRN37200", /* FIXME: We need to create an IDENT macro for this string.
							                 Make sure the IDENT macro is not accessible to userland */
							  &psBRNHeap);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXHwBrn37200: HWBRN37200 Failed DevmemFindHeapByName (%u)", eError));
		goto failFWHWBRN37200FindHeapByName;
	}

	psDevInfo->psRGXFWHWBRN37200MemDesc = IMG_NULL;
	eError = DevmemAllocate(psBRNHeap,
						ui32PageSize,
						ROGUE_CACHE_LINE_SIZE,
						uiFlags,
						&psDevInfo->psRGXFWHWBRN37200MemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXHwBrn37200: Failed to allocate %d bytes for HWBRN37200 (%u)",
				ui32PageSize,
				eError));
		goto failFWHWBRN37200MemDescAlloc;
	}
		
	/*
		We need to map it so the heap for this allocation
		is set
	*/
	eError = DevmemMapToDevice(psDevInfo->psRGXFWHWBRN37200MemDesc,
						   psBRNHeap,
						   &sTmpDevVAddr);
		
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXHwBrn37200: Failed to allocate %d bytes for HWBRN37200 (%u)",
				ui32PageSize,
				eError));
		goto failFWHWBRN37200DevmemMapToDevice;
	}

	return PVRSRV_OK;

failFWHWBRN37200DevmemMapToDevice:

failFWHWBRN37200MemDescAlloc:
	RGXHwBrn37200Rollback(psDevInfo);

failFWHWBRN37200FindHeapByName:
#endif

	return eError;
}

/*!
*******************************************************************************

 @Function	RGXSetupFirmware

 @Description

 Setups all the firmware related data

 @Input psDevInfo

 @Return PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR RGXSetupFirmware(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							  IMG_DEVMEM_SIZE_T		ui32FWMemAllocSize,
							  DEVMEM_EXPORTCOOKIE	**ppsFWMemAllocServerExportCookie,
							  IMG_DEV_VIRTADDR		*psFWMemDevVAddrBase,
							  IMG_BOOL				bEnableSignatureChecks,
							  IMG_UINT32			ui32SignatureChecksBufSize,
							  IMG_UINT32			ui32RGXFWAlignChecksSize,
							  IMG_UINT32			*pui32RGXFWAlignChecks,
							  IMG_UINT32			ui32ConfigFlags,
							  IMG_UINT32			ui32LogType,
							  RGXFWIF_DEV_VIRTADDR	*psRGXFWInitFWAddr)
{
 	PVRSRV_ERROR		eError;
	DEVMEM_FLAGS_T		uiMemAllocFlags;
	RGXFWIF_INIT		*psRGXFWInit;
	PVRSRV_RGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice;
#if defined(PDUMP)
	IMG_UINT32 dm;
#endif

	/* 
	 * Set up Allocations for FW code
	 */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
						PVRSRV_MEMALLOCFLAG_GPU_READABLE | 
						PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_CPU_READABLE |
						PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | 
						PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
						PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
						PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate and export memory for fw");

	eError = DevmemFwAllocateExportable(psDeviceNode,
										ui32FWMemAllocSize,
										uiMemAllocFlags,
										&psDevInfo->psRGXFWMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Failed to allocate fw mem (%u)",
				eError));
		goto failFWMemDescAlloc;
	}

	eError = DevmemExport(psDevInfo->psRGXFWMemDesc, &psDevInfo->sRGXFWExportCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Failed to export fw mem (%u)",
				eError));
		goto failFWMemDescExport;
	}

	eError = DevmemAcquireDevVirtAddr(psDevInfo->psRGXFWMemDesc, psFWMemDevVAddrBase);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Failed to acquire devVAddr for fw mem (%u)",
				eError));
		goto failFWMemDescAqDevVirt;
	}

	/*
	* The FW code must be the first allocation in the firmware heap, otherwise
	* the bootloader will not work (META will not be able to find the bootloader).
	*/
	PVR_ASSERT(psFWMemDevVAddrBase->uiAddr == RGX_FIRMWARE_HEAP_BASE);

	*ppsFWMemAllocServerExportCookie = &psDevInfo->sRGXFWExportCookie;

	/* Fw init data */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
						PVRSRV_MEMALLOCFLAG_GPU_READABLE |
						PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_CPU_READABLE |
						PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
						PVRSRV_MEMALLOCFLAG_UNCACHED |
						PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;
						/* FIXME: Change to Cached */

	PDUMPCOMMENT("Allocate RGXFWIF_INIT structure");
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_INIT),
							uiMemAllocFlags,
							&psDevInfo->psRGXFWIfInitMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for fw if ctl (%u)",
				sizeof(RGXFWIF_INIT),
				eError));
		goto failFWIfInitMemDescAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc,
									  (IMG_VOID **)&psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire kernel fw if ctl (%u)",
				eError));
		goto failFWIfInitMemDescAqCpuVirt;
	}

	RGXSetFirmwareAddress(psRGXFWInitFWAddr,
						psDevInfo->psRGXFWIfInitMemDesc,
						0, RFW_FWADDR_NOREF_FLAG);

	/* FW Trace buffer */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
						PVRSRV_MEMALLOCFLAG_GPU_READABLE |
						PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
						PVRSRV_MEMALLOCFLAG_CPU_READABLE |
						PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
						PVRSRV_MEMALLOCFLAG_UNCACHED |
						PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw trace structure");
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_TRACEBUF),
							uiMemAllocFlags,
							&psDevInfo->psRGXFWIfTraceBufCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for fw trace (%u)",
				sizeof(RGXFWIF_TRACEBUF),
				eError));
		goto failFWIfTraceBufCtlMemDescAlloc;
	}

	RGXSetFirmwareAddress(&psRGXFWInit->psTraceBufCtl,
						psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
						0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
									  (IMG_VOID **)&psDevInfo->psRGXFWIfTraceBuf);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire kernel tracebuf ctl (%u)",
				eError));
		goto failFWIfTraceBufCtlMemDescAqCpuVirt;
	}

	/* init HWPERF data */
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfRIdx = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfWIdx = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfOrdinal = IMG_UINT32_MAX;
	
	/* Allocate HWPERF buffer (user side) if HWPerf enabled, usually a developer
	 * or test system would enable this. Should not be enabled on a production
	 * device due to the RAM foot print costs.
	 */
	if (ui32ConfigFlags & RGXFWIF_INICFG_HWPERF_EN)
	{
		eError = TLStreamCreate(&psDevInfo->hHWPerfStream, "hwperf", RGXFW_HWPERF_SERVER_COUNT*(sizeof(RGX_HWPERF_PACKET)+sizeof(PVRSRVTL_PACKETHDR)), TL_FLAG_DROP_DATA);
		if (eError != PVRSRV_OK)
			PVR_LOGG_IF_ERROR(eError, "TLStreamCreate", failFWIfTraceBufCtlMemDescAqCpuVirt);
	}
	/* Assume zero on allocate: else psDevInfo->hHWPerfStream = 0; */

	/* Set initial log type */
	if (ui32LogType & ~RGXFWIF_LOG_TYPE_MASK)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Invalid initial log type (0x%X)",ui32LogType));
		goto failFWIfTraceBufCtlMemDescAqCpuVirt;
	}
	psDevInfo->psRGXFWIfTraceBuf->ui32LogType = ui32LogType;

	/* Allocate a sync for power management */
	eError = SyncPrimContextCreate(IMG_NULL,
									psDevInfo->psDeviceNode,
						  			IMG_NULL,
						  			&psDevInfo->hSyncPrimContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXInitDevPart2KM: Failed to allocate sync primitive context with error (%u)", eError));
		goto failed_sync_ctx_alloc;
	}

	eError = SyncPrimAlloc(psDevInfo->hSyncPrimContext, &psDevInfo->psPowSyncPrim);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXInitDevPart2KM: Failed to allocate sync primitive with error (%u)", eError));
		goto failed_sync_alloc;
	}

	psRGXFWInit->uiPowerSync = SyncPrimGetFirmwareAddr(psDevInfo->psPowSyncPrim);


	/* Setup BIF Fault read register */
	eError = RGXSetupBIFFaultReadRegister(psDeviceNode, psRGXFWInit);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to setup BIF fault read register"));
		goto failFWSetupBIFFaultReadRegister;
	}

	/* Apply FIX_HW_BRN_37200 */
	eError = RGXHwBrn37200(psDevInfo);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to apply HWBRN37200"));
		goto failFWHWBRN37200;
	}

	/*
	 * Set up kernel TA CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
							   psRGXFWInit,
							   RGXFWIF_DM_TA, RGXFWIF_KCCB_TA_NUMCMDS_LOG2,
							   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate kernel TA CCB"));
		goto failTACCB;
	}

	/*
	 * Set up kernel 3D CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
							   psRGXFWInit,
							   RGXFWIF_DM_3D, RGXFWIF_KCCB_3D_NUMCMDS_LOG2,
							   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate kernel 3D CCB"));
		goto fail3DCCB;
	}

	/*
	 * Set up kernel 2D CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
							   psRGXFWInit,
							   RGXFWIF_DM_2D, RGXFWIF_KCCB_2D_NUMCMDS_LOG2,
							   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate kernel 2D CCB"));
		goto fail2DCCB;
	}

	/*
	 * Set up kernel compute CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
							   psRGXFWInit,
							   RGXFWIF_DM_CDM, RGXFWIF_KCCB_CDM_NUMCMDS_LOG2,
							   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate kernel Compute CCB"));
		goto failCompCCB;
	}


	/*
	 * Set up kernel general purpose CCB.
	 */
	eError = RGXSetupKernelCCB(psDevInfo,
							   psRGXFWInit,
							   RGXFWIF_DM_GP, RGXFWIF_KCCB_GP_NUMCMDS_LOG2,
							   sizeof(RGXFWIF_KCCB_CMD));
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate kernel General Purpose CCB"));
		goto failGPCCB;
	}

	/* Setup Signature and Checksum Buffers for TA and 3D */
	eError = RGXFWSetupSignatureChecks(psDevInfo,
									   &psDevInfo->psRGXFWSigTAChecksMemDesc, 
									   ui32SignatureChecksBufSize,
									   &psRGXFWInit->asSigBufCtl[RGXFWIF_DM_TA]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to setup TA signature checks"));
		goto failTASigCheck;
	}
	psDevInfo->ui32SigTAChecksSize = ui32SignatureChecksBufSize;

	eError = RGXFWSetupSignatureChecks(psDevInfo,
									   &psDevInfo->psRGXFWSig3DChecksMemDesc, 
									   ui32SignatureChecksBufSize,
									   &psRGXFWInit->asSigBufCtl[RGXFWIF_DM_3D]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to setup 3D signature checks"));
		goto fail3DSigCheck;
	}
	psDevInfo->ui32Sig3DChecksSize = ui32SignatureChecksBufSize;

#if defined(RGXFW_ALIGNCHECKS)
	eError = RGXFWSetupAlignChecks(psDevInfo, 
								&psRGXFWInit->paui32AlignChecks, 
								pui32RGXFWAlignChecks, 
								ui32RGXFWAlignChecksSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to setup alignment checks"));
		goto failAlignCheck;
	}
#endif

	/* Fill the remaining bits of fw the init data */
	psRGXFWInit->sPDSExecBase.uiAddr = RGX_PDSCODEDATA_HEAP_BASE;
	psRGXFWInit->sUSCExecBase.uiAddr = RGX_USCCODE_HEAP_BASE;

	/* RD Power Island */
	{
		RGX_DATA *psRGXData = (RGX_DATA*) psDeviceNode->psDevConfig->hDevData;
		IMG_BOOL bEnableRDPowIsland = psRGXData->psRGXTimingInfo->bEnableRDPowIsland;

		ui32ConfigFlags |= bEnableRDPowIsland? RGXFWIF_INICFG_POW_RASCALDUST : 0;

	}

	psRGXFWInit->ui32ConfigFlags = ui32ConfigFlags;
	
	/* Initialise the compatibility check data */
	RGXFWIF_COMPCHECKS_BVNC_INIT(psRGXFWInit->sRGXCompChecks.sFWBVNC);
	RGXFWIF_COMPCHECKS_BVNC_INIT(psRGXFWInit->sRGXCompChecks.sHWBVNC);

	{
		/* Below line is to make sure (compilation time check) that 
				RGX_BVNC_V_ST fits into RGXFWIF_COMPCHECKS_BVNC structure */
		IMG_CHAR _tmp_[RGXFWIF_COMPCHECKS_BVNC_V_LEN_MAX] = RGX_BVNC_V_ST;
		_tmp_[0] = '\0';
	}

	PDUMPCOMMENT("Dump RGXFW Init data");
	if (!bEnableSignatureChecks)
	{
#if defined(PDUMP)
		PDUMPCOMMENT("(to enable rgxfw signatures place the following line after the RTCONF line)");
		DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfInitMemDesc,
							offsetof(RGXFWIF_INIT, asSigBufCtl),
							sizeof(RGXFWIF_SIGBUF_CTL)*RGXFWIF_DM_MAX,
							PDUMP_FLAGS_CONTINUOUS);
#endif
		psRGXFWInit->asSigBufCtl[RGXFWIF_DM_3D].psBuffer.ui32Addr = 0x0;
		psRGXFWInit->asSigBufCtl[RGXFWIF_DM_TA].psBuffer.ui32Addr = 0x0;
	}

#if defined(PDUMP)
	for (dm = 0; dm < RGXFWIF_HWDM_MAX; dm++)
	{
		psDevInfo->psRGXFWIfTraceBuf->aui16HwrDmLockedUpCount[dm] = 0;
		psDevInfo->psRGXFWIfTraceBuf->aui16HwrDmRecoveredCount[dm] = 0;
		psDevInfo->psRGXFWIfTraceBuf->apsHwrDmFWCommonContext[dm] = IMG_NULL;
	}

	PDUMPCOMMENT("Dump rgxfw trace structure");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
						0,
						sizeof(RGXFWIF_TRACEBUF),
						PDUMP_FLAGS_CONTINUOUS);


	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfInitMemDesc,
						0,
						sizeof(RGXFWIF_INIT),
						PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("RTCONF: run-time configuration");

	
	/* Dump the config options so they can be edited.
	 * 
	 * FIXME: Need new DevmemPDumpWRW API which writes a WRW to load ui32ConfigFlags
	 */
	PDUMPCOMMENT("(Set the FW config options here)");
	PDUMPCOMMENT("( bit 0: Ctx Switch TA Enable)");
	PDUMPCOMMENT("( bit 1: Ctx Switch 3D Enable)");
	PDUMPCOMMENT("( bit 2: Ctx Switch CDM Enable)");
	PDUMPCOMMENT("( bit 3: Ctx Switch Rand mode)");
	PDUMPCOMMENT("( bit 4: Ctx Switch Soft Reset Enable)");
	PDUMPCOMMENT("( bit 5: Enable 2nd thread)");
	PDUMPCOMMENT("( bit 6: Rascal+Dust Power Island)");
	PDUMPCOMMENT("( bit 7: Enable HWPerf)");
	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfInitMemDesc,
							offsetof(RGXFWIF_INIT, ui32ConfigFlags),
							psRGXFWInit->ui32ConfigFlags,
							PDUMP_FLAGS_CONTINUOUS);

	/* 
	 * Dump the log config so it can be edited.
	 */
	PDUMPCOMMENT("(Set the log config here)");
	PDUMPCOMMENT("( bit 0: Log Type: set for TRACE, reset for TBI)");
	PDUMPCOMMENT("( bit 1: MAIN Group Enable)");
	PDUMPCOMMENT("( bit 2: MTS Group Enable)");
	PDUMPCOMMENT("( bit 3: CLEANUP Group Enable)");
	PDUMPCOMMENT("( bit 4: CSW Group Enable)");
	PDUMPCOMMENT("( bit 5: BIF Group Enable)");
	PDUMPCOMMENT("( bit 6: PM Group Enable)");
	PDUMPCOMMENT("( bit 7: RTD Group Enable)");
	PDUMPCOMMENT("( bit 8: SPM Group Enable)");
	PDUMPCOMMENT("( bit 9: POW Group Enable)");
	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
							offsetof(RGXFWIF_TRACEBUF, ui32LogType),
							psDevInfo->psRGXFWIfTraceBuf->ui32LogType,
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

	RGXHwBrn37200Rollback(psDevInfo);
failFWHWBRN37200:

	RGXSetupBIFFaultReadRegisterRollback(psDevInfo);
failFWSetupBIFFaultReadRegister:

	SyncPrimFree(psDevInfo->psPowSyncPrim);
failed_sync_alloc:

	SyncPrimContextDestroy(psDevInfo->hSyncPrimContext);
failed_sync_ctx_alloc:

failFWIfTraceBufCtlMemDescAqCpuVirt:
	DevmemFwFree(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);

failFWIfTraceBufCtlMemDescAlloc:	
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
failFWIfInitMemDescAqCpuVirt:
	DevmemFwFree(psDevInfo->psRGXFWIfInitMemDesc);

failFWIfInitMemDescAlloc:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWMemDesc);
failFWMemDescAqDevVirt:
	DevmemUnexport(psDevInfo->psRGXFWMemDesc, &psDevInfo->sRGXFWExportCookie);
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
IMG_VOID RGXFreeFirmware(PVRSRV_RGXDEV_INFO 	*psDevInfo)
{
	RGXFWIF_DM	eKCCBType;
	
	for (eKCCBType = 0; eKCCBType < RGXFWIF_DM_MAX; eKCCBType++)
	{
		if (psDevInfo->apsKernelCCBMemDesc[eKCCBType])
		{
			RGXFreeKernelCCB(psDevInfo, eKCCBType);
		}
	}

	if (psDevInfo->psRGXFWMemDesc)
	{
		/* Free fw code */
		PDUMPCOMMENT("Freeing FW memory");
		DevmemReleaseDevVirtAddr(psDevInfo->psRGXFWMemDesc);
		DevmemFwFree(psDevInfo->psRGXFWMemDesc);
	}

#if defined(RGXFW_ALIGNCHECKS)
	if (psDevInfo->psRGXFWAlignChecksMemDesc)
	{
		RGXFWFreeAlignChecks(psDevInfo);
	}
#endif

	if (psDevInfo->psRGXFWSigTAChecksMemDesc)
	{
		DevmemFwFree(psDevInfo->psRGXFWSigTAChecksMemDesc);
	}

	if (psDevInfo->psRGXFWSig3DChecksMemDesc)
	{
		DevmemFwFree(psDevInfo->psRGXFWSig3DChecksMemDesc);
	}

#if defined(FIX_HW_BRN_37200)
	if (psDevInfo->psRGXFWHWBRN37200MemDesc)
	{
		DevmemReleaseDevVirtAddr(psDevInfo->psRGXFWHWBRN37200MemDesc);
		DevmemFree(psDevInfo->psRGXFWHWBRN37200MemDesc);
	}
#endif

	RGXSetupBIFFaultReadRegisterRollback(psDevInfo);


	SyncPrimFree(psDevInfo->psPowSyncPrim);
	SyncPrimContextDestroy(psDevInfo->hSyncPrimContext);

	if (psDevInfo->psRGXFWIfTraceBufCtlMemDesc)
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
		psDevInfo->psRGXFWIfTraceBuf = IMG_NULL;
		DevmemFwFree(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
	}

	if (psDevInfo->hHWPerfStream != 0)
	{
		TLStreamDestroy(psDevInfo->hHWPerfStream);
		psDevInfo->hHWPerfStream = 0;
	}

	if (psDevInfo->psRGXFWIfInitMemDesc)
	{
		DevmemFwFree(psDevInfo->psRGXFWIfInitMemDesc);
	}
}


/******************************************************************************
 FUNCTION	: RGXStartFirmware

 PURPOSE	: Attempts to obtain a slot in the Kernel CCB

 PARAMETERS	: psDevInfo

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR RGXStartFirmware(PVRSRV_RGXDEV_INFO 	*psDevInfo)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS, "Start the firmware\n");

#if defined(SUPPORT_META_SLAVE_BOOT)
	/*
	 * Run init script.
	 */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS, "Start of RGX initialisation script (Slave boot)");
	eError = RGXRunScript(psDevInfo, psDevInfo->sScripts.asInitCommands, RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXStart: RGXRunScript failed (%d)", eError));
		return eError;
	}
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS, "End of RGX initialisation script");
#else
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS, "RGX firmware Master boot");
#endif


	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS|PDUMP_FLAGS_CONTINUOUS, "Firmware startup complete\n");
	
	return eError;
}


/******************************************************************************
 FUNCTION	: RGXAcquireKernelCCBSlot

 PURPOSE	: Attempts to obtain a slot in the Kernel CCB

 PARAMETERS	: psCCB - the CCB
			: Address of space if available, IMG_NULL otherwise

 RETURNS	: PVRSRV_ERROR
******************************************************************************/
static PVRSRV_ERROR RGXAcquireKernelCCBSlot(RGXFWIF_KCCB_CTL	*psKCCBCtl,
											IMG_UINT32			*pui32Offset)
{
	IMG_UINT32	ui32OldWriteOffset, ui32NextWriteOffset;

	ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
	ui32NextWriteOffset = (ui32OldWriteOffset + 1) & psKCCBCtl->ui32WrapMask;

	/* Note: The MTS can queue up to 255 kicks (254 pending kicks and 1 executing kick)
	 * Hence the kernel CCB should not queue more 254 commands
	 */
	PVR_ASSERT(psKCCBCtl->ui32WrapMask < 255);

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{

		if (ui32NextWriteOffset != psKCCBCtl->ui32ReadOffset)
		{
			*pui32Offset = ui32NextWriteOffset;
			return PVRSRV_OK;
		}
		{
			/* 
			 * The following sanity check doesn't impact performance,
			 * since the CPU has to wait for the GPU anyway (full kernel CCB).
			 */
			if (PVRSRVGetPVRSRVData()->eServicesState != PVRSRV_SERVICES_STATE_OK)
			{
				return PVRSRV_ERROR_KERNEL_CCB_FULL;
			}
		}

		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	/* Time out on waiting for CCB space */
	return PVRSRV_ERROR_KERNEL_CCB_FULL;
}


PVRSRV_ERROR RGXSendCommandWithPowLock(PVRSRV_RGXDEV_INFO 	*psDevInfo,
										 RGXFWIF_DM			eKCCBType,
										 RGXFWIF_KCCB_CMD	*psKCCBCmd,
										 IMG_UINT32			ui32CmdSize,
										 IMG_BOOL			bPDumpContinuous)
{
	PVRSRV_ERROR		eError;
	PVRSRV_DEVICE_NODE *psDeviceNode = psDevInfo->psDeviceNode;

	/* Ensure RGX is powered up before kicking MTS */
	PVRSRVForcedPowerLock()

	PDUMPPOWCMDSTART();

	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
										 PVRSRV_DEV_POWER_STATE_ON,
										 KERNEL_ID,
										 IMG_FALSE,
										 IMG_FALSE);
	PDUMPPOWCMDEND();

	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_WARNING, "RGXSendCommandWithPowLock: failed to transition RGX to ON (%s)",
					PVRSRVGetErrorStringKM(eError)));

		goto _RGXSendCommandWithPowLock_Exit;
	}

	RGXSendCommandRaw(psDevInfo, eKCCBType,  psKCCBCmd, ui32CmdSize, bPDumpContinuous?PDUMP_FLAGS_CONTINUOUS:0);

_RGXSendCommandWithPowLock_Exit:
	PVRSRVPowerUnlock();
	return eError;
}

PVRSRV_ERROR RGXSendCommandRaw(PVRSRV_RGXDEV_INFO 	*psDevInfo,
								 RGXFWIF_DM			eKCCBType,
								 RGXFWIF_KCCB_CMD	*psKCCBCmd,
								 IMG_UINT32			ui32CmdSize,
								 PDUMP_FLAGS_T		uiPdumpFlags)
{
	PVRSRV_ERROR		eError;
	RGXFWIF_KCCB_CTL	*psKCCBCtl = psDevInfo->apsKernelCCBCtl[eKCCBType];
	IMG_UINT8			*pui8KCCB = psDevInfo->apsKernelCCB[eKCCBType];
	IMG_UINT32			ui32NewWriteOffset;
	IMG_UINT32			ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
#if !defined(PDUMP)
	PVR_UNREFERENCED_PARAMETER(uiPdumpFlags);
#endif
	
	PVR_ASSERT(ui32CmdSize == psKCCBCtl->ui32CmdSize);

	/*
	 * Acquire a slot in the CCB.
	 */ 
	eError = RGXAcquireKernelCCBSlot(psKCCBCtl, &ui32NewWriteOffset);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXSendCommandRaw failed to acquire CCB slot. Type:%u Error:%u",
				eKCCBType, eError));
		RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
		goto _RGXSendCommandRaw_Exit;
	}
	
	/*
	 * Copy the command into the CCB.
	 */
	OSMemCopy(&pui8KCCB[ui32OldWriteOffset * psKCCBCtl->ui32CmdSize],
			  psKCCBCmd, psKCCBCtl->ui32CmdSize);

	/* ensure kCCB data is written before the offsets */
	OSWriteMemoryBarrier();

	/* Move past the current command */
	psKCCBCtl->ui32WriteOffset = ui32NewWriteOffset;


#if defined(PDUMP)
	{
		IMG_BOOL bIsInCaptureRange;
		IMG_BOOL bPdumpEnabled;
		IMG_BOOL bPDumpContinuous = (uiPdumpFlags & PDUMP_FLAGS_CONTINUOUS) != 0;
		IMG_BOOL bPDumpPowTrans = (uiPdumpFlags & PDUMP_FLAGS_POWERTRANS) != 0;

		PDumpIsCaptureFrameKM(&bIsInCaptureRange);
		bPdumpEnabled = (bIsInCaptureRange || bPDumpContinuous) && !bPDumpPowTrans;

		/* in capture range */
		if (bPdumpEnabled)
		{
			/* Dump new Kernel CCB content */
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump kCCB(%d) cmd, woff = %d", eKCCBType, ui32OldWriteOffset);
			DevmemPDumpLoadMem(psDevInfo->apsKernelCCBMemDesc[eKCCBType],
					ui32OldWriteOffset * psKCCBCtl->ui32CmdSize,
					psKCCBCtl->ui32CmdSize,
					PDUMP_FLAGS_CONTINUOUS);

			if (!psDevInfo->abDumpedKCCBCtlAlready[eKCCBType])
			{
				/* entering capture range */
				psDevInfo->abDumpedKCCBCtlAlready[eKCCBType] = IMG_TRUE;

				/* wait for firmware to catch up */
				PVR_DPF((PVR_DBG_WARNING, "RGXSendCommandRaw: waiting on fw to catch-up. DM: %d, roff: %d, woff: %d",
							eKCCBType, psKCCBCtl->ui32ReadOffset, ui32OldWriteOffset));
				PVRSRVPollForValueKM(&psKCCBCtl->ui32ReadOffset, ui32OldWriteOffset, 0xFFFFFFFF);

				/* Dump Init state of Kernel CCB control (read and write offset) */
				PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Initial state of kernel CCB Control(%d), roff: %d, woff: %d", eKCCBType, psKCCBCtl->ui32ReadOffset, psKCCBCtl->ui32WriteOffset);
				DevmemPDumpLoadMem(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
						0,
						sizeof(RGXFWIF_KCCB_CTL),
						PDUMP_FLAGS_CONTINUOUS);
			}
			else
			{
				/* already in capture range */

				/* Dump new kernel CCB write offset */
				PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "Dump kCCBCtl(%d) woff: %d", eKCCBType, ui32NewWriteOffset);
				DevmemPDumpLoadMem(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
								   offsetof(RGXFWIF_KCCB_CTL, ui32WriteOffset),
								   sizeof(IMG_UINT32),
								   uiPdumpFlags);
			}
		}

		/* out of capture range */
		if (!bPdumpEnabled)
		{
			if (psDevInfo->abDumpedKCCBCtlAlready[eKCCBType])
			{
				/* exiting capture range */
				psDevInfo->abDumpedKCCBCtlAlready[eKCCBType] = IMG_FALSE;

				/* make sure previous cmds are drained in pdump in case we will 'jump' over some future cmds */
				PDUMPCOMMENTWITHFLAGS(uiPdumpFlags,
							"kCCB(%p): Draining rgxfw_roff (0x%x) == woff (0x%x)",
								psKCCBCtl,
								ui32OldWriteOffset,
								ui32OldWriteOffset);
				eError = DevmemPDumpDevmemPol32(psDevInfo->apsKernelCCBCtlMemDesc[eKCCBType],
												offsetof(RGXFWIF_KCCB_CTL, ui32ReadOffset),
												ui32OldWriteOffset,
												0xffffffff,
												PDUMP_POLL_OPERATOR_EQUAL,
												PDUMP_FLAGS_CONTINUOUS);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_WARNING, "RGXSendCommandRaw: problem pdumping POL for cCCBCtl (%d)", eError));
					goto _RGXSendCommandRaw_Exit;
				}
			}
		}
	}
#endif


	PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "MTS kick for kernel CCB %d", eKCCBType);
	/*
	 * Kick the MTS to schedule the firmware.
	 */
	{
		IMG_UINT32	ui32MTSRegVal = (eKCCBType & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_COUNTED;
		
		__MTSScheduleWrite(psDevInfo, ui32MTSRegVal);

		PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_MTS_SCHEDULE, ui32MTSRegVal, uiPdumpFlags);
	}
	
#if defined (NO_HARDWARE)
	/* keep the roff updated because fw isn't there to update it */
	psKCCBCtl->ui32ReadOffset = psKCCBCtl->ui32WriteOffset;
#endif

_RGXSendCommandRaw_Exit:
	return eError;
}

/*!
******************************************************************************

 @Function	RGXScheduleProcessQueuesKM

 @Description - Sends uncounted kick to all the DMs (the FW will process all
				the queue for all the DMs)

 @Note - This function is called from irq context, avoid any mutex (therefore,
          no pdump calls).
******************************************************************************/
IMG_VOID RGXScheduleProcessQueuesKM(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = (PVRSRV_DEVICE_NODE*) hCmdCompHandle;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_DM			eDM;
	PVRSRV_ERROR		eError;

	/* Ensure RGX is powered up before kicking MTS */
	eError = PVRSRVPowerLock();
	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_WARNING, "RGXScheduleProcessQueuesKM: failed to acquire powerlock (%s)",
					PVRSRVGetErrorStringKM(eError)));

		return;
	}

	/* We don't need to acquire the BridgeLock as this power transition won't
	   send a command to the FW */
	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
										 PVRSRV_DEV_POWER_STATE_ON,
										 KERNEL_ID,
										 IMG_FALSE,
										 IMG_FALSE);
	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_WARNING, "RGXScheduleProcessQueuesKM: failed to transition RGX to ON (%s)",
					PVRSRVGetErrorStringKM(eError)));

		PVRSRVPowerUnlock();
		return;
	}

	/* uncounted kick for all DMs */
	for (eDM = RGXFWIF_DM_TA; eDM < RGXFWIF_HWDM_MAX; eDM++)
	{
		IMG_UINT32	ui32MTSRegVal = (eDM & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_NON_COUNTED;

		__MTSScheduleWrite(psDevInfo, ui32MTSRegVal);
	}

	PVRSRVPowerUnlock();
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
PVRSRV_ERROR RGXScheduleCommand(PVRSRV_RGXDEV_INFO 	*psDevInfo,
								RGXFWIF_DM			eKCCBType,
								RGXFWIF_KCCB_CMD	*psKCCBCmd,
								IMG_UINT32			ui32CmdSize,
								IMG_BOOL			bPDumpContinuous)
{
	PVRSRV_DATA *psData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR eError;

	if ((eKCCBType == RGXFWIF_DM_3D) || (eKCCBType == RGXFWIF_DM_2D))
	{
		/* This handles the no operation case */
		OSCPUOperation(psData->uiCacheOp);
		psData->uiCacheOp = PVRSRV_CACHE_OP_NONE;
	}

	eError = RGXPreKickCacheCommand(psDevInfo);
	if (eError != PVRSRV_OK) goto RGXScheduleCommand_exit;

	eError = RGXSendCommandWithPowLock(psDevInfo, eKCCBType, psKCCBCmd, ui32CmdSize, bPDumpContinuous);
	if (eError != PVRSRV_OK) goto RGXScheduleCommand_exit;


RGXScheduleCommand_exit:
	return eError;
}


/*
 * PVRSRVRGXFrameworkCopyRegisters
 */ 
PVRSRV_ERROR PVRSRVRGXFrameworkCopyRegisters(DEVMEM_MEMDESC * psFWFrameworkMemDesc,
		IMG_PBYTE                                           pbyGPUFRegisterList,
		IMG_UINT32                                          ui32FrameworkRegisterSize)
{
	PVRSRV_ERROR			eError;
	RGXFWIF_RF_REGISTERS	*psRegUpdates;

	eError = DevmemAcquireCpuVirtAddr(psFWFrameworkMemDesc,
                                      (IMG_VOID **)&psRegUpdates);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
				eError));
		return eError;
	}

	OSMemCopy(psRegUpdates, pbyGPUFRegisterList, ui32FrameworkRegisterSize);
	
	/* Release the CPU mapping */
	DevmemReleaseCpuVirtAddr(psFWFrameworkMemDesc);

	/*
	 * Dump the FW framework buffer
	 */
	PDUMPCOMMENT("Dump FWFramework buffer");
	DevmemPDumpLoadMem(psFWFrameworkMemDesc, 0, ui32FrameworkRegisterSize, PDUMP_FLAGS_CONTINUOUS);

	return PVRSRV_OK;
}

/*
 * PVRSRVRGXFrameworkCreateKM
 */
PVRSRV_ERROR PVRSRVRGXFrameworkCreateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
		DEVMEM_MEMDESC                                     ** ppsFWFrameworkMemDesc,
		IMG_UINT32                                         ui32FrameworkRegisterSize)
{
	PVRSRV_ERROR			eError;
	PVRSRV_RGXDEV_INFO 		*psDevInfo = psDeviceNode->pvDevice;
	
	/*
		Allocate device memory for the firmware GPU framework state.
		Sufficient info to kick one or more DMs should be contained in this buffer
	*/
	PDUMPCOMMENT("Allocate RGX firmware framework state");

	eError = DevmemFwAllocate(psDevInfo,
							ui32FrameworkRegisterSize,
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWFrameworkMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXFrameworkContextKM: Failed to allocate firmware framework state (%u)",
				eError));
		return eError;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXInitFWCommonContext(RGXFWIF_FWCOMMONCONTEXT		*psFWComContext,
									  DEVMEM_MEMDESC 			*psCCBMemDesc,
									  DEVMEM_MEMDESC 			*psCCBCtlMemDesc,
									  DEVMEM_MEMDESC			*psFWMemContextMemDesc,
									  DEVMEM_MEMDESC			*psFWFrameworkMemDesc,
									  IMG_UINT32				ui32Priority,
									  IMG_DEV_VIRTADDR			*psMCUFenceAddr,
									  RGX_FWCOMCTX_CLEANUP		*psCleanupData)
{
	/* Save what we need for clean up */
	psCleanupData->psCCBMemDesc = psCCBMemDesc;
	psCleanupData->psCCBCtlMemDesc = psCCBCtlMemDesc;
	psCleanupData->psFWMemContextMemDesc = psFWMemContextMemDesc;
	psCleanupData->psFWFrameworkMemDesc = psFWFrameworkMemDesc;

	/*
	 * Set the firmware CCB device addresses in the firmware common context.
	 */
	RGXSetFirmwareAddress(&psFWComContext->psCCB, psCCBMemDesc, 0, RFW_FWADDR_FLAG_NONE | RFW_FWADDR_METACACHED_FLAG);
	RGXSetFirmwareAddress(&psFWComContext->psCCBCtl, psCCBCtlMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	/*
	 * And the memory context device address.
	 */ 
	RGXSetFirmwareAddress(&psFWComContext->psFWMemContext, psFWMemContextMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	/*
	 * Set the framework register updates address 
	 */
	RGXSetFirmwareAddress(&psFWComContext->psRFCmd, psFWFrameworkMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	psFWComContext->ui32Priority = ui32Priority;
	if(psMCUFenceAddr != IMG_NULL)
	{
		psFWComContext->ui64MCUFenceAddr = psMCUFenceAddr->uiAddr;
	}
	
	return PVRSRV_OK;
}

PVRSRV_ERROR RGXDeinitFWCommonContext(RGX_FWCOMCTX_CLEANUP *psCleanupData)
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

	/*
	 * And the FW framework buffer address.
	 */ 
	RGXUnsetFirmwareAddress(psCleanupData->psFWFrameworkMemDesc);

	return eError;
}


PVRSRV_ERROR RGXScheduleCommandAndWait(PVRSRV_RGXDEV_INFO 	*psDevInfo,
									   RGXFWIF_DM			eDM,
									   RGXFWIF_KCCB_CMD		*psKCCBCmd,
									   IMG_UINT32			ui32CmdSize,
									   IMG_UINT32			*puiSyncObjDevVAddr,
									   IMG_UINT32			*puiUpdateValue,
									   PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
									   IMG_BOOL				bPDumpContinuous)
{
	PVRSRV_ERROR eError;

	/* sanity check. Concurrent use of the syncPrim should be prevented by the bridge-lock */
	if (psSyncPrim == psDevInfo->psDeviceNode->psSyncPrim)
	{
		/* general command sync prim */
		PVR_ASSERT(psDevInfo->psDeviceNode->ui32SyncPrimRefCount == 0);
		psDevInfo->psDeviceNode->ui32SyncPrimRefCount++;
	}
	else
	{
		/* prekick sync prim */
		PVR_ASSERT(psSyncPrim == psDevInfo->psDeviceNode->psSyncPrimPreKick);
		PVR_ASSERT(psDevInfo->psDeviceNode->ui32SyncPrimPreKickRefCount == 0);
		psDevInfo->psDeviceNode->ui32SyncPrimPreKickRefCount++;
	}

	/* Setup sync primitive */
	SyncPrimSet(psSyncPrim, 0);
	*puiSyncObjDevVAddr = SyncPrimGetFirmwareAddr(psSyncPrim);
	*puiUpdateValue = 1;

	PDUMPCOMMENT("RGXScheduleCommandAndWait: Submit Kernel SyncPrim [0x%08x] to DM %d ", *puiSyncObjDevVAddr, eDM);

	/* submit the sync primitive to the kernel CCB */
	eError = RGXScheduleCommand(psDevInfo,
								eDM,
								psKCCBCmd,
								sizeof(RGXFWIF_KCCB_CMD),
								bPDumpContinuous);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXScheduleCommandAndWait: Failed to schedule Kernel SyncPrim with error (%u)", eError));
		goto ScheduleError;
	}

	/* Wait for sync primitive to be updated */
#if defined(PDUMP)
	SyncPrimPDumpPol(psSyncPrim,
					1,
					0xffffffff,
					PDUMP_POLL_OPERATOR_EQUAL,
					bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS:0);
#endif

	for(;;)
	{
		RGXFWIF_KCCB_CTL  *psKCCBCtl = psDevInfo->apsKernelCCBCtl[eDM];
		IMG_UINT32        ui32CurrentQueueLength = (psKCCBCtl->ui32WrapMask+1 +
		                                            psKCCBCtl->ui32WriteOffset -
		                                            psKCCBCtl->ui32ReadOffset) & psKCCBCtl->ui32WrapMask;
		IMG_UINT32        ui32MaxRetries;

		for (ui32MaxRetries = ui32CurrentQueueLength + 1;
			 ui32MaxRetries > 0;
			 ui32MaxRetries--)
		{
			/* FIXME: Need to re-think PVRSRVLocking */
			OSSetKeepPVRLock();
			eError = PVRSRVWaitForValueKM(psSyncPrim->pui32LinAddr, 1, 0xffffffff);
			OSSetReleasePVRLock();
			
			if (eError != PVRSRV_ERROR_TIMEOUT)
			{
				break;
			}
		}

		if (eError == PVRSRV_ERROR_TIMEOUT)
		{
			PVR_DPF((PVR_DBG_WARNING,"RGXScheduleCommandAndWait: PVRSRVWaitForValueKM timed out. Dump debug information."));

			RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
			continue;
		}
		else
		{
			break;
		}

	}

ScheduleError:
if (psSyncPrim == psDevInfo->psDeviceNode->psSyncPrim)
{
	psDevInfo->psDeviceNode->ui32SyncPrimRefCount--;
}
else
{
	psDevInfo->psDeviceNode->ui32SyncPrimPreKickRefCount--;
}

	return eError;
}


PVRSRV_ERROR RGXWaitForFWOp(PVRSRV_RGXDEV_INFO	*psDevInfo,
							RGXFWIF_DM eDM,
							PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
							IMG_BOOL bPDumpContinuous)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD	sCmdSyncPrim;

	/* prepare a sync command */
	sCmdSyncPrim.eCmdType = RGXFWIF_KCCB_CMD_SYNC;

	eError = RGXScheduleCommandAndWait(psDevInfo,
									   eDM,
									   &sCmdSyncPrim,
									   sizeof(RGXFWIF_KCCB_CMD),
									   &sCmdSyncPrim.uCmdData.sSyncData.uiSyncObjDevVAddr,
									   &sCmdSyncPrim.uCmdData.sSyncData.uiUpdateVal,
									   psSyncPrim,
									   bPDumpContinuous);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXWaitForFWOp: Failed to schedule Kernel SyncPrim with error (%u)", eError));
	}

	return eError;
}

static
PVRSRV_ERROR RGXScheduleCleanupCommand(PVRSRV_RGXDEV_INFO 	*psDevInfo,
									   RGXFWIF_DM			eDM,
									   RGXFWIF_KCCB_CMD		*psKCCBCmd,
									   IMG_UINT32			ui32CmdSize,
									   RGXFWIF_CLEANUP_TYPE	eCleanupType,
									   PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
									   IMG_BOOL				bPDumpContinuous)
{
	PVRSRV_ERROR eError;

	psKCCBCmd->eCmdType = RGXFWIF_KCCB_CMD_CLEANUP;

	psKCCBCmd->uCmdData.sCleanupData.eCleanupType = eCleanupType;
	psKCCBCmd->uCmdData.sCleanupData.uiSyncObjDevVAddr = SyncPrimGetFirmwareAddr(psSyncPrim);

	SyncPrimSet(psSyncPrim, 0);

	/*
		Send the cleanup request to the firmware. If the resource is still busy
		the firmware will tell us and we'll drop out with a retry.
	*/
	eError = RGXScheduleCommand(psDevInfo,
								eDM,
								psKCCBCmd,
								ui32CmdSize,
								bPDumpContinuous);
	if (eError != PVRSRV_OK)
	{
		goto fail_command;
	}

	/* Wait for sync primitive to be updated */
#if defined(PDUMP)
	SyncPrimPDumpPol(psSyncPrim,
					RGXFWIF_CLEANUP_RUN,
					RGXFWIF_CLEANUP_RUN,
					PDUMP_POLL_OPERATOR_EQUAL,
					bPDumpContinuous ? PDUMP_FLAGS_CONTINUOUS:0);
#endif

	for(;;)
	{
		RGXFWIF_KCCB_CTL  *psKCCBCtl = psDevInfo->apsKernelCCBCtl[eDM];
		IMG_UINT32        ui32CurrentQueueLength = (psKCCBCtl->ui32WrapMask+1 +
		                                            psKCCBCtl->ui32WriteOffset -
		                                            psKCCBCtl->ui32ReadOffset) & psKCCBCtl->ui32WrapMask;
		IMG_UINT32        ui32MaxRetries;

		for (ui32MaxRetries = ui32CurrentQueueLength + 1;
			 ui32MaxRetries > 0;
			 ui32MaxRetries--)
		{
			/* FIXME: Need to re-think PVRSRVLocking */
			OSSetKeepPVRLock();
			eError = PVRSRVWaitForValueKM(psSyncPrim->pui32LinAddr, RGXFWIF_CLEANUP_RUN, RGXFWIF_CLEANUP_RUN);
			OSSetReleasePVRLock();
			
			if (eError != PVRSRV_ERROR_TIMEOUT)
			{
				break;
			}
		}
		
		/*
			If the firmware hasn't got back to us in a timely manner
			then bail with retry and we'll try next time around the loop
		*/
		if (eError == PVRSRV_ERROR_TIMEOUT)
		{
			PVR_DPF((PVR_DBG_WARNING,"RGXScheduleCleanupCommand: PVRSRVWaitForValueKM timed out. Dump debug information."));

			eError = PVRSRV_ERROR_RETRY;
			RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
			goto fail_poll;
		}
		else
		{
			break;
		}
	}

	/*
		If the command has was run but a resource was busy, then the request
		will need to be retried.
	*/
	if (*psSyncPrim->pui32LinAddr & RGXFWIF_CLEANUP_BUSY)
	{
		eError = PVRSRV_ERROR_RETRY;
		goto fail_requestbusy;
	}

	return PVRSRV_OK;

fail_requestbusy:
fail_poll:
fail_command:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

/*
	RGXRequestCommonContextCleanUp
*/
PVRSRV_ERROR RGXFWRequestCommonContextCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
											  PRGXFWIF_FWCOMMONCONTEXT psFWComContextFWAddr,
											  PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
											  RGXFWIF_DM eDM)
{
	RGXFWIF_KCCB_CMD			sRCCleanUpCmd = {0};
	PVRSRV_ERROR				eError;

	PDUMPCOMMENT("Common ctx cleanup Request DM%d [context = 0x%08x]", eDM, psFWComContextFWAddr.ui32Addr);

	/* Setup our command data, the cleanup call will fill in the rest */
	sRCCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psContext = psFWComContextFWAddr;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
									   eDM,
									   &sRCCleanUpCmd,
									   sizeof(RGXFWIF_KCCB_CMD),
									   RGXFWIF_CLEANUP_FWCOMMONCONTEXT,
									   psSyncPrim,
									   IMG_FALSE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXRequestCommonContextCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

/*
 * RGXRequestHWRTDataCleanUp
 */

PVRSRV_ERROR RGXFWRequestHWRTDataCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_HWRTDATA psHWRTData,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync,
										 RGXFWIF_DM eDM)
{
	RGXFWIF_KCCB_CMD			sHWRTDataCleanUpCmd = {0};
	PVRSRV_ERROR				eError;

	PDUMPCOMMENT("HW RTData cleanup Request DM%d [HWRTData = 0x%08x]", eDM, psHWRTData.ui32Addr);

	sHWRTDataCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psHWRTData = psHWRTData;

	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
									   eDM,
									   &sHWRTDataCleanUpCmd,
									   sizeof(sHWRTDataCleanUpCmd),
									   RGXFWIF_CLEANUP_HWRTDATA,
									   psSync,
									   IMG_FALSE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXRequestHWRTDataCleanUp: Failed to schedule a HWRTData cleanup with error (%u)", eError));
	}

	return eError;
}

/*
	RGXFWRequestFreeListCleanUp
*/
PVRSRV_ERROR RGXFWRequestFreeListCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_FREELIST psFWFreeList,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync)
{
	RGXFWIF_KCCB_CMD			sFLCleanUpCmd = {0};
	PVRSRV_ERROR 				eError;

	PDUMPCOMMENT("Free list cleanup Request [FreeList = 0x%08x]", psFWFreeList.ui32Addr);

	/* Setup our command data, the cleanup call will fill in the rest */
	sFLCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psFreelist = psFWFreeList;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
									   RGXFWIF_DM_GP,
									   &sFLCleanUpCmd,
									   sizeof(RGXFWIF_KCCB_CMD),
									   RGXFWIF_CLEANUP_FREELIST,
									   psSync,
									   IMG_FALSE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXFWRequestFreeListCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

/*
	RGXFWRequestZSBufferCleanUp
*/
PVRSRV_ERROR RGXFWRequestZSBufferCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_ZSBUFFER psFWZSBuffer,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync)
{
	RGXFWIF_KCCB_CMD			sZSBufferCleanUpCmd = {0};
	PVRSRV_ERROR 				eError;

	PDUMPCOMMENT("ZS Buffer cleanup Request [ZS Buffer = 0x%08x]", psFWZSBuffer.ui32Addr);

	/* Setup our command data, the cleanup call will fill in the rest */
	sZSBufferCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psZSBuffer = psFWZSBuffer;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
									   RGXFWIF_DM_GP,
									   &sZSBufferCleanUpCmd,
									   sizeof(RGXFWIF_KCCB_CMD),
									   RGXFWIF_CLEANUP_ZSBUFFER,
									   psSync,
									   IMG_FALSE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXFWRequestZSBufferCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

/******************************************************************************
 End of file (rgxfwutils.c)
******************************************************************************/
