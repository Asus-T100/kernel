									    /*************************************************************************//*!
									       @File
									       @Title          Rgx debug information
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX debugging functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "rgx_cr_defs_km.h"
#include "rgxdefs.h"
#include "rgxdevice.h"
#include "allocmem.h"
#include "osfunc.h"

#include "rgxdebug.h"
#include "pvrversion.h"
#include "pvr_debug.h"
#include "rgxutils.h"

#define RGX_DEBUG_STR_SIZE	(50)

/*!
*******************************************************************************

 @Function	_RGXDecodePMPC

 @Description

 Return the name for the PM managed Page Catalogues

 @Input ui32PC	 - Page Catalogue number

 @Return   IMG_VOID

******************************************************************************/
static IMG_CHAR *_RGXDecodePMPC(IMG_UINT32 ui32PC)
{
	IMG_CHAR *pszPMPC = " (-)";

	switch (ui32PC) {
	case 0x8:
		pszPMPC = " (PM-VCE0)";
		break;
	case 0x9:
		pszPMPC = " (PM-TE0)";
		break;
	case 0xA:
		pszPMPC = " (PM-ZLS0)";
		break;
	case 0xB:
		pszPMPC = " (PM-ALIST0)";
		break;
	case 0xC:
		pszPMPC = " (PM-VCE1)";
		break;
	case 0xD:
		pszPMPC = " (PM-TE1)";
		break;
	case 0xE:
		pszPMPC = " (PM-ZLS1)";
		break;
	case 0xF:
		pszPMPC = " (PM-ALIST1)";
		break;
	}

	return pszPMPC;
}

/*!
*******************************************************************************

 @Function	_RGXDecodeBIFReqTags

 @Description

 Decode the BIF Tag ID and SideBand data fields from BIF_FAULT_BANK_REQ_STATUS regs

 @Input ui32TagID           - Tag ID value
 @Input ui32TagSB           - Tag Sideband data
 @Output ppszTagID          - Decoded string from the Tag ID
 @Output ppszTagSB          - Decoded string from the Tag SB
 @Output pszScratchBuf      - Buffer provided to the function to generate the debug strings
 @Input ui32ScratchBufSize  - Size of the provided buffer

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDecodeBIFReqTags(IMG_UINT32 ui32TagID,
				     IMG_UINT32 ui32TagSB,
				     IMG_CHAR ** ppszTagID,
				     IMG_CHAR ** ppszTagSB,
				     IMG_CHAR * pszScratchBuf,
				     IMG_UINT32 ui32ScratchBufSize)
{
	/* default to unknown */
	IMG_CHAR *pszTagID = "-";
	IMG_CHAR *pszTagSB = "-";

	switch (ui32TagID) {
	case 0x0:
		{
			pszTagID = "MMU";
			break;
		}
	case 0x1:
		{
			pszTagID = "TLA";
			break;
		}
	case 0x2:
		{
			pszTagID = "HOST";
			break;
		}
	case 0x3:
		{
			pszTagID = "META";
			switch (ui32TagSB) {
			case 0x0:
				pszTagSB = "dcache";
				break;
			case 0x1:
				pszTagSB = "icache";
				break;
			case 0x2:
				pszTagSB = "jtag";
				break;
			case 0x3:
				pszTagSB = "slave bus";
				break;
			}
			break;
		}
	case 0x4:
		{
			pszTagID = "USC";
			break;
		}
	case 0x5:
		{
			pszTagID = "PBE";
			break;
		}
	case 0x6:
		{
			pszTagID = "ISP";
			break;
		}
	case 0x7:
		{
			pszTagID = "IPF";
			switch (ui32TagSB) {
			case 0x0:
				pszTagSB = "Macrotile Header";
				break;
			case 0x1:
				pszTagSB = "Region Header";
				break;
			case 0x2:
				pszTagSB = "DBSC";
				break;
			case 0x3:
				pszTagSB = "CPF";
				break;
			case 0x4:
			case 0x6:
			case 0x8:
				pszTagSB = "Control Stream";
				break;
			case 0x5:
			case 0x7:
			case 0x9:
				pszTagSB = "Primitive Block";
				break;
			}
			break;
		}
	case 0x8:
		{
			pszTagID = "CDM";
			switch (ui32TagSB) {
			case 0x0:
				pszTagSB = "Control Stream";
				break;
			case 0x1:
				pszTagSB = "Indirect Data";
				break;
			case 0x2:
				pszTagSB = "Event Write";
				break;
			case 0x3:
				pszTagSB = "Context State";
				break;
			}
			break;
		}
	case 0x9:
		{
			pszTagID = "VDM";
			switch (ui32TagSB) {
			case 0x0:
				pszTagSB = "Control Stream";
				break;
			case 0x1:
				pszTagSB = "PPP State";
				break;
			case 0x2:
				pszTagSB = "Index Data";
				break;
			case 0x4:
				pszTagSB = "Call Stack";
				break;
			case 0x8:
				pszTagSB = "Context State";
				break;
			}
			break;
		}
	case 0xA:
		{
			pszTagID = "PM";
			switch (ui32TagSB) {
			case 0x0:
				pszTagSB = "PMA_TAFSTACK";
				break;
			case 0x1:
				pszTagSB = "PMA_TAMLIST";
				break;
			case 0x2:
				pszTagSB = "PMA_3DFSTACK";
				break;
			case 0x3:
				pszTagSB = "PMA_3DMLIST";
				break;
			case 0x4:
				pszTagSB = "PMA_PMCTX0";
				break;
			case 0x5:
				pszTagSB = "PMA_PMCTX1";
				break;
			case 0x6:
				pszTagSB = "PMA_MAVP";
				break;
			case 0x7:
				pszTagSB = "PMA_UFSTACK";
				break;
			case 0x8:
				pszTagSB = "PMD_TAFSTACK";
				break;
			case 0x9:
				pszTagSB = "PMD_TAMLIST";
				break;
			case 0xA:
				pszTagSB = "PMD_3DFSTACK";
				break;
			case 0xB:
				pszTagSB = "PMD_3DMLIST";
				break;
			case 0xC:
				pszTagSB = "PMD_PMCTX0";
				break;
			case 0xD:
				pszTagSB = "PMD_PMCTX1";
				break;
			case 0xF:
				pszTagSB = "PMD_UFSTACK";
				break;
			case 0x10:
				pszTagSB = "PMA_TAMMUSTACK";
				break;
			case 0x11:
				pszTagSB = "PMA_3DMMUSTACK";
				break;
			case 0x12:
				pszTagSB = "PMD_TAMMUSTACK";
				break;
			case 0x13:
				pszTagSB = "PMD_3DMMUSTACK";
				break;
			case 0x14:
				pszTagSB = "PMA_TAUFSTACK";
				break;
			case 0x15:
				pszTagSB = "PMA_3DUFSTACK";
				break;
			case 0x16:
				pszTagSB = "PMD_TAUFSTACK";
				break;
			case 0x17:
				pszTagSB = "PMD_3DUFSTACK";
				break;
			case 0x18:
				pszTagSB = "PMA_TAVFP";
				break;
			case 0x19:
				pszTagSB = "PMD_3DVFP";
				break;
			case 0x1A:
				pszTagSB = "PMD_TAVFP";
				break;
			}
			break;
		}
	case 0xB:
		{
			pszTagID = "TA";
			switch (ui32TagSB) {
			case 0x1:
				pszTagSB = "VCE";
				break;
			case 0x2:
				pszTagSB = "TPC";
				break;
			case 0x3:
				pszTagSB = "TE Control Stream";
				break;
			case 0x4:
				pszTagSB = "TE Region Header";
				break;
			case 0x5:
				pszTagSB = "TE Render Target Cache";
				break;
			case 0x6:
				pszTagSB = "TEAC Render Target Cache";
				break;
			case 0x7:
				pszTagSB = "VCE Render Target Cache";
				break;
			}
			break;
		}
	case 0xC:
		{
			pszTagID = "TPF";
			break;
		}
	case 0xD:
		{
			pszTagID = "PDS";
			break;
		}
	case 0xE:
		{
			pszTagID = "MCU";
			{
				IMG_UINT32 ui32Burst = (ui32TagSB >> 5) & 0x7;
				IMG_UINT32 ui32GroupEnc =
				    (ui32TagSB >> 2) & 0x7;
				IMG_UINT32 ui32Group = ui32TagSB & 0x2;

				IMG_CHAR *pszBurst = "";
				IMG_CHAR *pszGroupEnc = "";
				IMG_CHAR *pszGroup = "";

				switch (ui32Burst) {
				case 0x4:
					pszBurst = "Lower 256bits";
					break;
				case 0x5:
					pszBurst = "Upper 256bits";
					break;
				case 0x6:
					pszBurst = "512 bits";
					break;
				default:
					pszBurst = (ui32Burst & 0x2) ?
					    "128bit word within the Lower 256bits"
					    :
					    "128bit word within the Upper 256bits";
					break;
				}
				switch (ui32GroupEnc) {
				case 0x0:
					pszGroupEnc = "TPUA_USC";
					break;
				case 0x1:
					pszGroupEnc = "TPUB_USC";
					break;
				case 0x2:
					pszGroupEnc = "USCA_USC";
					break;
				case 0x3:
					pszGroupEnc = "USCB_USC";
					break;
				case 0x4:
					pszGroupEnc = "PDS_USC";
					break;
#if (RGX_BVNC_N == 2) || (RGX_BVNC_N == 4)
				case 0x5:
					pszGroupEnc = "PDSRW";
					break;
#elif (RGX_BVNC_N == 6)
				case 0x5:
					pszGroupEnc = "UPUC_USC";
					break;
				case 0x6:
					pszGroupEnc = "TPUC_USC";
					break;
				case 0x7:
					pszGroupEnc = "PDSRW";
					break;
#endif
				}
				switch (ui32Group) {
				case 0x0:
					pszGroup = "Banks 0-3";
					break;
				case 0x1:
					pszGroup = "Banks 4-7";
					break;
				case 0x2:
					pszGroup = "Banks 8-11";
					break;
				case 0x3:
					pszGroup = "Banks 12-15";
					break;
				}

				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
					   "%s, %s, %s", pszBurst, pszGroupEnc,
					   pszGroup);
				pszTagSB = pszScratchBuf;
			}
			break;
		}
	case 0xF:
		{
			pszTagID = "FB_CDC";
			{
				IMG_UINT32 ui32Req = (ui32TagSB >> 2) & 0x3;
				IMG_UINT32 ui32MCUSB = ui32TagSB & 0x3;

				IMG_CHAR *pszReqId =
				    (ui32TagSB & 0x10) ? "FBDC" : "FBC";
				IMG_CHAR *pszOrig = "";

				switch (ui32Req) {
				case 0x0:
					pszOrig = "ZLS";
					break;
				case 0x1:
					pszOrig =
					    (ui32TagSB & 0x10) ? "MCU" : "PBE";
					break;
				case 0x2:
					pszOrig = "Host";
					break;
				case 0x3:
					pszOrig = "TLA";
					break;
				}
				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
					   "%s Request, originator %s, MCU sideband 0x%X",
					   pszReqId, pszOrig, ui32MCUSB);
				pszTagSB = pszScratchBuf;
			}
			break;
		}
	}			/* switch(TagID) */

	*ppszTagID = pszTagID;
	*ppszTagSB = pszTagSB;
}

/*!
*******************************************************************************

 @Function	_RGXDumpRGXBIFBank

 @Description

 Dump BIF Bank state in human readable form.

 @Input psDevInfo				- RGX device info
 @Input ui32BankID	 			- BIF Bank identification number
 @Input ui32MMUStatusRegAddr	- MMU Status register address
 @Input ui32ReqStatusRegAddr	- BIF request Status register address

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpRGXBIFBank(PVRSRV_RGXDEV_INFO * psDevInfo,
				   IMG_UINT32 ui32BankID,
				   IMG_UINT32 ui32MMUStatusRegAddr,
				   IMG_UINT32 ui32ReqStatusRegAddr)
{
	IMG_UINT64 ui64RegVal;

	ui64RegVal =
	    OSReadHWReg64(psDevInfo->pvRegsBaseKM, ui32MMUStatusRegAddr);

	if (ui64RegVal == 0x0) {
		PVR_LOG(("BIF%d - OK", ui32BankID));
	} else {
		/* Bank 0 & 1 share the same fields */
		PVR_LOG(("BIF%d - FAULT:", ui32BankID));

		/* MMU Status */
		{
			IMG_UINT32 ui32PC =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_CLRMSK)
			    >> RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_SHIFT;

			IMG_UINT32 ui32PageSize =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_CLRMSK)
			    >>
			    RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_SHIFT;

			IMG_UINT32 ui32MMUDataType =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_CLRMSK)
			    >>
			    RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_SHIFT;

			IMG_BOOL bROFault =
			    (ui64RegVal &
			     RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_RO_EN) !=
			    0;
			IMG_BOOL bProtFault =
			    (ui64RegVal &
			     RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_PM_META_RO_EN)
			    != 0;

			PVR_LOG(("  * MMU status (0x%016llX): PC = %d%s, Page Size = %d, MMU data type = %d%s%s.", ui64RegVal, ui32PC, (ui32PC < 0x8) ? "" : _RGXDecodePMPC(ui32PC), ui32PageSize, ui32MMUDataType, (bROFault) ? ", Read Only fault" : "", (bProtFault) ? ", PM/META protection fault" : ""));
		}

		/* Req Status */
		ui64RegVal =
		    OSReadHWReg64(psDevInfo->pvRegsBaseKM,
				  ui32ReqStatusRegAddr);
		{
			IMG_CHAR *pszTagID;
			IMG_CHAR *pszTagSB;
			IMG_CHAR aszScratch[RGX_DEBUG_STR_SIZE];

			IMG_BOOL bRead =
			    (ui64RegVal &
			     RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_RNW_EN) != 0;
			IMG_UINT32 ui32TagSB =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_CLRMSK)
			    >> RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_SHIFT;
			IMG_UINT32 ui32TagID =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_CLRMSK)
			    >> RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_SHIFT;
			IMG_UINT64 ui32Addr =
			    (ui64RegVal &
			     ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_ADDRESS_CLRMSK);

			_RGXDecodeBIFReqTags(ui32TagID, ui32TagSB, &pszTagID,
					     &pszTagSB, &aszScratch[0],
					     RGX_DEBUG_STR_SIZE);

			PVR_LOG(("  * Request (0x%016llX): %s (%s), %s 0x%10llX.", ui64RegVal, pszTagID, pszTagSB, (bRead) ? "Reading" : "Writing", ui32Addr));
		}
	}

}

/*!
*******************************************************************************

 @Function	_RGXDumpRGXDebugSummary

 @Description

 Dump a summary in human readable form with the RGX state

 @Input psDevInfo	 - RGX device info

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpRGXDebugSummary(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	PVR_LOG(("------[ RGX debug (%s) ]------", PVRVERSION_STRING));

	_RGXDumpRGXBIFBank(psDevInfo, 0,
			   RGX_CR_BIF_FAULT_BANK0_MMU_STATUS,
			   RGX_CR_BIF_FAULT_BANK0_REQ_STATUS);

	_RGXDumpRGXBIFBank(psDevInfo, 1,
			   RGX_CR_BIF_FAULT_BANK1_MMU_STATUS,
			   RGX_CR_BIF_FAULT_BANK1_REQ_STATUS);

}

/*
	RGXDumpDebugInfo
*/
IMG_VOID RGXDumpDebugInfo(PVRSRV_RGXDEV_INFO * psDevInfo, IMG_BOOL bDumpRGXRegs)
{
	PVRSRV_ERROR eError;

	_RGXDumpRGXDebugSummary(psDevInfo);

	if (bDumpRGXRegs) {
		PVR_LOG(("------[ RGX registers ]------"));
		PVR_LOG(("RGX Register Base Address (Linear):   0x%p",
			 psDevInfo->pvRegsBaseKM));
		PVR_LOG(("RGX Register Base Address (Physical): 0x%08lX",
			 (unsigned long)psDevInfo->sRegsPhysBase.uiAddr));

		eError =
		    RGXRunScript(psDevInfo, psDevInfo->sScripts.asDbgCommands,
				 RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXDumpDebugInfo: RGXRunScript failed (%d)",
				 eError));
			return;
		}

	}

	{
		RGXFWIF_DM eKCCBType;

		/*
		   Dump out the kernel CCBs.
		 */
		for (eKCCBType = 0; eKCCBType < RGXFWIF_DM_MAX; eKCCBType++) {
			RGXFWIF_KCCB_CTL *psKCCBCtl =
			    psDevInfo->apsKernelCCBCtl[eKCCBType];

			PVR_LOG(("RGX Kernel CCB %u WO:0x%X RO:0x%X",
				 eKCCBType, psKCCBCtl->ui32WriteOffset,
				 psKCCBCtl->ui32ReadOffset));

		}
	}

	/* Dump FW trace information */
	{
		IMG_UINT32 i;
		IMG_BOOL bPrevLineWasZero = IMG_FALSE;
		IMG_BOOL bLineIsAllZeros = IMG_FALSE;
		IMG_UINT32 ui32CountLines = 0;
		IMG_UINT32 *pui32TraceBuffer;
		RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl;
		IMG_CHAR *pszLine;

		eError =
		    DevmemAcquireCpuVirtAddr(psDevInfo->
					     psRGXFWIfTraceBufCtlMemDesc,
					     (IMG_VOID **) &
					     psRGXFWIfTraceBufCtl);

		pui32TraceBuffer = &psRGXFWIfTraceBufCtl->aui32TraceBuffer[0];

		/* each element in the line is 8 characters plus a space */
		pszLine = OSAllocMem(9 * RGXFW_TRACE_BUFFER_LINESIZE);
		if (pszLine == IMG_NULL) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXDumpDebugInfo: Out of mem allocating line string (size: %d)",
				 9 * RGXFW_TRACE_BUFFER_LINESIZE));
			return;
		}

		/* Print the tracepointer */
		PVR_LOG(("Debug log type: %d", psRGXFWIfTraceBufCtl->eLogType));

		PVR_LOG(("------[ RGX Kernel FW trace START ]------"));
		PVR_LOG(("FWT[traceptr]: %X",
			 psRGXFWIfTraceBufCtl->ui32TracePointer));

		for (i = 0; i < RGXFW_TRACE_BUFFER_SIZE;
		     i += RGXFW_TRACE_BUFFER_LINESIZE) {
			IMG_UINT32 k = 0;
			IMG_UINT32 ui32Line = 0x0;
			IMG_UINT32 ui32LineOffset = i * sizeof(IMG_UINT32);
			IMG_CHAR *pszBuf = pszLine;

			for (k = 0; k < RGXFW_TRACE_BUFFER_LINESIZE; k++) {
				ui32Line |= pui32TraceBuffer[i + k];

				/* prepare the line to print it. The '+1' is because of the trailing '\0' added */
				OSSNPrintf(pszBuf, 9 + 1, " %08x",
					   pui32TraceBuffer[i + k]);
				pszBuf += 9;	/* write over the '\0' */
			}

			bLineIsAllZeros = (ui32Line == 0x0);

			if (bLineIsAllZeros && bPrevLineWasZero) {
				ui32CountLines++;
			} else if (bLineIsAllZeros && !bPrevLineWasZero) {
				bPrevLineWasZero = IMG_TRUE;
				ui32CountLines = 0;
				PVR_LOG(("FWT[%08x]: 00000000 ... 00000000",
					 ui32LineOffset))
			} else {
				if (bPrevLineWasZero) {
					PVR_LOG(("FWT[%08x]: %d lines were all zero", ui32LineOffset, ui32CountLines));
				} else {

					PVR_LOG(("FWT[%08x]:%s", ui32LineOffset,
						 pszLine));
				}
				bPrevLineWasZero = IMG_FALSE;
			}

		}
		if (bPrevLineWasZero) {
			PVR_LOG(("FWT[END]: %d lines were all zero",
				 ui32CountLines));
		}

		PVR_LOG(("------[ RGX Kernel FW trace END ]------"));

		OSFreeMem(pszLine);

		DevmemReleaseCpuVirtAddr(psDevInfo->
					 psRGXFWIfTraceBufCtlMemDesc);
	}
}

/*
	RGXPanic
*/
IMG_VOID RGXPanic(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	PVR_LOG(("RGX panic"));
	RGXDumpDebugInfo(psDevInfo, IMG_FALSE);
	OSPanic();
}

/******************************************************************************
 End of file (rgxdebug.c)
******************************************************************************/
