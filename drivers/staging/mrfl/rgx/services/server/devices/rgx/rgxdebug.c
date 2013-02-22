/*************************************************************************/ /*!
@File
@Title          Rgx debug information
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX debugging functions
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


//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON

#include "rgxdefs_km.h"
#include "rgxdevice.h"
#include "allocmem.h"
#include "osfunc.h"

#include "rgxdebug.h"
#include "pvrversion.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "tlstream.h"
#include "rgxfwutils.h"

#include "devicemem_pdump.h"

#include "rgx_fwif.h"

#define RGX_DEBUG_STR_SIZE	(50)

IMG_CHAR* pszPowStateName [] = {
#define X(NAME)	#NAME,
	RGXFWIF_POW_STATES
#undef X
};

/*!
*******************************************************************************

 @Function	_RGXGetPowerStateString

 @Description

 Return a string with the name for the APM FW state.

 @Input ePowState	 - The APM FW state

 @Return   The string name of the APM state

******************************************************************************/
static IMG_CHAR* _RGXGetPowerStateString(RGXFWIF_POW_STATE ePowState)
{
	return pszPowStateName[ePowState];
}

/*!
*******************************************************************************

 @Function	_RGXDecodePMPC

 @Description

 Return the name for the PM managed Page Catalogues

 @Input ui32PC	 - Page Catalogue number

 @Return   IMG_VOID

******************************************************************************/
static IMG_CHAR* _RGXDecodePMPC(IMG_UINT32 ui32PC)
{
	IMG_CHAR* pszPMPC = " (-)";

	switch (ui32PC)
	{
		case 0x8: pszPMPC = " (PM-VCE0)"; break;
		case 0x9: pszPMPC = " (PM-TE0)"; break;
		case 0xA: pszPMPC = " (PM-ZLS0)"; break;
		case 0xB: pszPMPC = " (PM-ALIST0)"; break;
		case 0xC: pszPMPC = " (PM-VCE1)"; break;
		case 0xD: pszPMPC = " (PM-TE1)"; break;
		case 0xE: pszPMPC = " (PM-ZLS1)"; break;
		case 0xF: pszPMPC = " (PM-ALIST1)"; break;
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
static IMG_VOID _RGXDecodeBIFReqTags(IMG_UINT32		ui32TagID, 
									 IMG_UINT32		ui32TagSB, 
									 IMG_CHAR		**ppszTagID, 
									 IMG_CHAR		**ppszTagSB,
									 IMG_CHAR		*pszScratchBuf,
									 IMG_UINT32		ui32ScratchBufSize)
{
	/* default to unknown */
	IMG_CHAR *pszTagID = "-";
	IMG_CHAR *pszTagSB = "-";

	switch (ui32TagID)
	{
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
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "dcache"; break;
				case 0x1: pszTagSB = "icache"; break;
				case 0x2: pszTagSB = "jtag"; break;
				case 0x3: pszTagSB = "slave bus"; break;
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
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Macrotile Header"; break;
				case 0x1: pszTagSB = "Region Header"; break;
				case 0x2: pszTagSB = "DBSC"; break;
				case 0x3: pszTagSB = "CPF"; break;
				case 0x4: 
				case 0x6:
				case 0x8: pszTagSB = "Control Stream"; break;
				case 0x5: 
				case 0x7:
				case 0x9: pszTagSB = "Primitive Block"; break;
			}
			break;
		}
		case 0x8:
		{
			pszTagID = "CDM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Control Stream"; break;
				case 0x1: pszTagSB = "Indirect Data"; break;
				case 0x2: pszTagSB = "Event Write"; break;
				case 0x3: pszTagSB = "Context State"; break;
			}
			break;
		}
		case 0x9:
		{
			pszTagID = "VDM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Control Stream"; break;
				case 0x1: pszTagSB = "PPP State"; break;
				case 0x2: pszTagSB = "Index Data"; break;
				case 0x4: pszTagSB = "Call Stack"; break;
				case 0x8: pszTagSB = "Context State"; break;
			}
			break;
		}
		case 0xA:
		{
			pszTagID = "PM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "PMA_TAFSTACK"; break;
				case 0x1: pszTagSB = "PMA_TAMLIST"; break;
				case 0x2: pszTagSB = "PMA_3DFSTACK"; break;
				case 0x3: pszTagSB = "PMA_3DMLIST"; break;
				case 0x4: pszTagSB = "PMA_PMCTX0"; break;
				case 0x5: pszTagSB = "PMA_PMCTX1"; break;
				case 0x6: pszTagSB = "PMA_MAVP"; break;
				case 0x7: pszTagSB = "PMA_UFSTACK"; break;
				case 0x8: pszTagSB = "PMD_TAFSTACK"; break;
				case 0x9: pszTagSB = "PMD_TAMLIST"; break;
				case 0xA: pszTagSB = "PMD_3DFSTACK"; break;
				case 0xB: pszTagSB = "PMD_3DMLIST"; break;
				case 0xC: pszTagSB = "PMD_PMCTX0"; break;
				case 0xD: pszTagSB = "PMD_PMCTX1"; break;
				case 0xF: pszTagSB = "PMD_UFSTACK"; break;
				case 0x10: pszTagSB = "PMA_TAMMUSTACK"; break;
				case 0x11: pszTagSB = "PMA_3DMMUSTACK"; break;
				case 0x12: pszTagSB = "PMD_TAMMUSTACK"; break;
				case 0x13: pszTagSB = "PMD_3DMMUSTACK"; break;
				case 0x14: pszTagSB = "PMA_TAUFSTACK"; break;
				case 0x15: pszTagSB = "PMA_3DUFSTACK"; break;
				case 0x16: pszTagSB = "PMD_TAUFSTACK"; break;
				case 0x17: pszTagSB = "PMD_3DUFSTACK"; break;
				case 0x18: pszTagSB = "PMA_TAVFP"; break;
				case 0x19: pszTagSB = "PMD_3DVFP"; break;
				case 0x1A: pszTagSB = "PMD_TAVFP"; break;
			}
			break;
		}
		case 0xB:
		{
			pszTagID = "TA";
			switch (ui32TagSB)
			{
				case 0x1: pszTagSB = "VCE"; break;
				case 0x2: pszTagSB = "TPC"; break;
				case 0x3: pszTagSB = "TE Control Stream"; break;
				case 0x4: pszTagSB = "TE Region Header"; break;
				case 0x5: pszTagSB = "TE Render Target Cache"; break;
				case 0x6: pszTagSB = "TEAC Render Target Cache"; break;
				case 0x7: pszTagSB = "VCE Render Target Cache"; break;
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
				IMG_UINT32 ui32GroupEnc = (ui32TagSB >> 2) & 0x7;
				IMG_UINT32 ui32Group = ui32TagSB & 0x2;

				IMG_CHAR* pszBurst = "";
				IMG_CHAR* pszGroupEnc = "";
				IMG_CHAR* pszGroup = "";

				switch (ui32Burst)
				{
					case 0x4: pszBurst = "Lower 256bits"; break;
					case 0x5: pszBurst = "Upper 256bits"; break;
					case 0x6: pszBurst = "512 bits"; break;
					default:  pszBurst = (ui32Burst & 0x2)?
								"128bit word within the Lower 256bits":
								"128bit word within the Upper 256bits"; break;
				}
				switch (ui32GroupEnc)
				{
					case 0x0: pszGroupEnc = "TPUA_USC"; break;
					case 0x1: pszGroupEnc = "TPUB_USC"; break;
					case 0x2: pszGroupEnc = "USCA_USC"; break;
					case 0x3: pszGroupEnc = "USCB_USC"; break;
					case 0x4: pszGroupEnc = "PDS_USC"; break;
#if (RGX_BVNC_N < 6)
					case 0x5: pszGroupEnc = "PDSRW"; break;
#elif (RGX_BVNC_N == 6)
					case 0x5: pszGroupEnc = "UPUC_USC"; break;
					case 0x6: pszGroupEnc = "TPUC_USC"; break;
					case 0x7: pszGroupEnc = "PDSRW"; break;
#endif
				}
				switch (ui32Group)
				{
					case 0x0: pszGroup = "Banks 0-3"; break;
					case 0x1: pszGroup = "Banks 4-7"; break;
					case 0x2: pszGroup = "Banks 8-11"; break;
					case 0x3: pszGroup = "Banks 12-15"; break;
				}

				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
								"%s, %s, %s", pszBurst, pszGroupEnc, pszGroup);
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

				IMG_CHAR* pszReqId = (ui32TagSB & 0x10)?"FBDC":"FBC";
				IMG_CHAR* pszOrig = "";

				switch (ui32Req)
				{
					case 0x0: pszOrig = "ZLS"; break;
					case 0x1: pszOrig = (ui32TagSB & 0x10)?"MCU":"PBE"; break;
					case 0x2: pszOrig = "Host"; break;
					case 0x3: pszOrig = "TLA"; break;
				}
				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
							"%s Request, originator %s, MCU sideband 0x%X",
							pszReqId, pszOrig, ui32MCUSB);
				pszTagSB = pszScratchBuf;
			}
			break;
		}
	} /* switch(TagID) */

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
static IMG_VOID _RGXDumpRGXBIFBank(	PVRSRV_RGXDEV_INFO	*psDevInfo, 
									IMG_UINT32			ui32BankID, 
									IMG_UINT32			ui32MMUStatusRegAddr,
									IMG_UINT32			ui32ReqStatusRegAddr)
{
	IMG_UINT64	ui64RegVal;

	ui64RegVal = OSReadHWReg64(psDevInfo->pvRegsBaseKM, ui32MMUStatusRegAddr);

	if (ui64RegVal == 0x0)
	{
		PVR_LOG(("BIF%d - OK", ui32BankID));
	}
	else
	{
		/* Bank 0 & 1 share the same fields */
		PVR_LOG(("BIF%d - FAULT:", ui32BankID));

		/* MMU Status */
		{
			IMG_UINT32 ui32PC = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_SHIFT;

			IMG_UINT32 ui32PageSize = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_SHIFT;

			IMG_UINT32 ui32MMUDataType = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_SHIFT;

			IMG_BOOL bROFault = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_RO_EN) != 0;
			IMG_BOOL bProtFault = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_PM_META_RO_EN) != 0;

			PVR_LOG(("  * MMU status (0x%016llX): PC = %d%s, Page Size = %d, MMU data type = %d%s%s.",
						ui64RegVal,
						ui32PC,
						(ui32PC < 0x8)?"":_RGXDecodePMPC(ui32PC),
						ui32PageSize,
						ui32MMUDataType,
						(bROFault)?", Read Only fault":"",
						(bProtFault)?", PM/META protection fault":""));
		}

		/* Req Status */
		ui64RegVal = OSReadHWReg64(psDevInfo->pvRegsBaseKM, ui32ReqStatusRegAddr);
		{
			IMG_CHAR *pszTagID;
			IMG_CHAR *pszTagSB;
			IMG_CHAR aszScratch[RGX_DEBUG_STR_SIZE];

			IMG_BOOL bRead = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_RNW_EN) != 0;
			IMG_UINT32 ui32TagSB = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_SHIFT;
			IMG_UINT32 ui32TagID = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_CLRMSK) >>
							RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_SHIFT;
			IMG_UINT64 ui64Addr = (ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_ADDRESS_CLRMSK);

			_RGXDecodeBIFReqTags(ui32TagID, ui32TagSB, &pszTagID, &pszTagSB, &aszScratch[0], RGX_DEBUG_STR_SIZE);

			PVR_LOG(("  * Request (0x%016llX): %s (%s), %s 0x%010llX.",
						ui64RegVal,
						pszTagID,
						pszTagSB,
						(bRead)?"Reading":"Writing",
						ui64Addr));
		}
	}

}


/*!
*******************************************************************************

 @Function	_RGXDumpFWAssert

 @Description

 Dump FW assert strings when a thread asserts.

 @Input psDevInfo				- RGX device info

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpFWAssert(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	RGXFWIF_TRACEBUF	*psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
	IMG_CHAR			*pszTraceAssertPath;
	IMG_CHAR			*pszTraceAssertInfo;
	IMG_INT32			ui32TraceAssertLine;
	IMG_UINT32			i;
	
	for (i = 0; i < RGXFW_THREAD_NUM; i++)
	{
		pszTraceAssertPath = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.szPath;
		pszTraceAssertInfo = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.szInfo;
		ui32TraceAssertLine = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.ui32LineNum;

		/* print non null assert strings */
		if (*pszTraceAssertInfo)
		{
			PVR_LOG(("FW-T%d Assert: %s (%s:%d)", 
					 i, pszTraceAssertInfo, pszTraceAssertPath, ui32TraceAssertLine));
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
static IMG_VOID _RGXDumpRGXDebugSummary(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PVR_LOG(("------[ RGX debug (%s) ]------", PVRVERSION_STRING));

	OSDumpStack();

	_RGXDumpRGXBIFBank(psDevInfo, 0, 
						RGX_CR_BIF_FAULT_BANK0_MMU_STATUS, 
						RGX_CR_BIF_FAULT_BANK0_REQ_STATUS);
	
	_RGXDumpRGXBIFBank(psDevInfo, 1, 
						RGX_CR_BIF_FAULT_BANK1_MMU_STATUS, 
						RGX_CR_BIF_FAULT_BANK1_REQ_STATUS);

	_RGXDumpFWAssert(psDevInfo);

	
	/* Dump DM lock-up information and FW cr poll state */
	{
		RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
		IMG_BOOL bAnyLocked = IMG_FALSE;
		IMG_UINT32 dm, i;

		for (i = 0; i < RGXFW_THREAD_NUM; i++)
		{
			if (psRGXFWIfTraceBufCtl->aui32CrPollAddr[i])
			{
				PVR_LOG(("T%u polling %s (reg:0x%08X val:0x%08X)",
						i,
						((psRGXFWIfTraceBufCtl->aui32CrPollAddr[i] & RGXFW_POLL_TYPE_SET)?("set"):("unset")), 
						psRGXFWIfTraceBufCtl->aui32CrPollAddr[i] & ~RGXFW_POLL_TYPE_SET, 
						psRGXFWIfTraceBufCtl->aui32CrPollValue[i]));
			}
		}
			
		for (dm = 0; dm < RGXFWIF_HWDM_MAX; dm++)
		{
			if (psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[dm])
			{
				bAnyLocked = IMG_TRUE;
				break;					
			}
		}

		if (bAnyLocked)
		{
			IMG_CHAR	*pszLine, *pszTemp;
			IMG_UINT32  ui32LineSize;
			const IMG_CHAR * apszDmNames[RGXFWIF_HWDM_MAX + 1] = { "TA(", "3D(", "CDM(", "2D(", NULL };
			const IMG_CHAR * pszMsgHeader = "Number of HWR: ";
			IMG_UINT32 ui32MsgHeaderSize = OSStringLength(pszMsgHeader);

			ui32LineSize = sizeof(IMG_CHAR) * (	ui32MsgHeaderSize + 
												(RGXFWIF_HWDM_MAX*(	4/*DM name + left parenthesis*/ + 
																	5/*UINT16 max num of digits*/ + 
																	1/*slash*/ + 
																	5/*UINT16 max num of digits*/ + 
																	3/*right parenthesis + comma + space*/)) + 
																	1/* \0 */);
	
			pszLine = OSAllocMem(ui32LineSize);
			if (pszLine == IMG_NULL)
			{
				PVR_DPF((PVR_DBG_ERROR,"_RGXDumpRGXDebugSummary: Out of mem allocating line string (size: %d)", ui32LineSize));
				return;
			}

			OSStringCopy(pszLine,pszMsgHeader);
			pszTemp = pszLine + ui32MsgHeaderSize;

			for (dm = 0; (dm < RGXFWIF_HWDM_MAX) && (apszDmNames[dm] != IMG_NULL); dm++)
			{
				OSStringCopy(pszTemp,apszDmNames[dm]);
				pszTemp += OSStringLength(apszDmNames[dm]);
				pszTemp += OSSNPrintf(pszTemp, 
					5 + 1 + 5 + 1 + 1 + 1 + 1 /* UINT16 + slash + UINT16 + right parenthesis + comma + space + \0 */,
					"%u/%u), ",
					psRGXFWIfTraceBufCtl->aui16HwrDmRecoveredCount[dm],
					psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[dm]);
			}

			PVR_LOG((pszLine));
			
			OSFreeMem(pszLine);
		}
	}

	PVR_LOG(("RGXFW APM State: %s", _RGXGetPowerStateString(psDevInfo->psRGXFWIfTraceBuf->ePowState)));

}

/*
	RGXDumpDebugInfo
*/
IMG_VOID RGXDumpDebugInfo (PVRSRV_RGXDEV_INFO	*psDevInfo,
						   IMG_BOOL				bDumpRGXRegs)
{
	PVRSRV_ERROR	eError;
	int tid;

	_RGXDumpRGXDebugSummary(psDevInfo);

	if (bDumpRGXRegs)
	{
		PVR_LOG(("------[ RGX registers ]------"));
		PVR_LOG(("RGX Register Base Address (Linear):   0x%p", psDevInfo->pvRegsBaseKM));
		PVR_LOG(("RGX Register Base Address (Physical): 0x%08lX", (unsigned long)psDevInfo->sRegsPhysBase.uiAddr));

		/* Forcing bit 6 of MslvCtrl1 to 0 to avoid internal reg read going though the core */
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVCTRL1, 0x0);

		eError = RGXRunScript(psDevInfo, psDevInfo->sScripts.asDbgCommands, RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXDumpDebugInfo: RGXRunScript failed (%d)", eError));
			return;
		}

	}

	{
		RGXFWIF_DM	eKCCBType;
		
		/*
 			Dump out the kernel CCBs.
 		*/
		for (eKCCBType = 0; eKCCBType < RGXFWIF_DM_MAX; eKCCBType++)
		{
			RGXFWIF_KCCB_CTL	*psKCCBCtl = psDevInfo->apsKernelCCBCtl[eKCCBType];

			PVR_LOG(("RGX Kernel CCB %u WO:0x%X RO:0x%X",
					 eKCCBType, psKCCBCtl->ui32WriteOffset, psKCCBCtl->ui32ReadOffset));

		}
 	}
 	
	/* Dump FW trace information */
	for ( tid = 0 ; tid < RGXFW_THREAD_NUM ; tid++) 
	{
		IMG_UINT32	i;
		IMG_BOOL	bPrevLineWasZero = IMG_FALSE;
		IMG_BOOL	bLineIsAllZeros = IMG_FALSE;
		IMG_UINT32	ui32CountLines = 0;
		IMG_UINT32	*pui32TraceBuffer;
		RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
		IMG_CHAR	*pszLine;

		pui32TraceBuffer = &psRGXFWIfTraceBufCtl->sTraceBuf[tid].aui32TraceBuffer[0];

		/* each element in the line is 8 characters plus a space */
		pszLine = OSAllocMem(9*RGXFW_TRACE_BUFFER_LINESIZE);
		if (pszLine == IMG_NULL)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXDumpDebugInfo: Out of mem allocating line string (size: %d)", 9*RGXFW_TRACE_BUFFER_LINESIZE));
			return;
		}

		/* Print the tracepointer */
		if (psRGXFWIfTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_GROUP_MASK)
		{
			PVR_LOG(("Debug log type: %s ( %s%s%s%s%s%s%s%s%s)", 
				((psRGXFWIfTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_TRACE)?("trace"):("tbi")),
				RGXFWIF_LOG_ENABLED_GROUPS_LIST(psRGXFWIfTraceBufCtl->ui32LogType)
				));
		}
		else
		{
			PVR_LOG(("Debug log type: none"));
		}
		
		PVR_LOG(("------[ RGX Kernel FW thread %d trace START ]------", tid));
		PVR_LOG(("FWT[traceptr]: %X", psRGXFWIfTraceBufCtl->sTraceBuf[tid].ui32TracePointer));

		for (i = 0; i < RGXFW_TRACE_BUFFER_SIZE; i += RGXFW_TRACE_BUFFER_LINESIZE)
		{
			IMG_UINT32 k = 0;
			IMG_UINT32 ui32Line = 0x0;
			IMG_UINT32 ui32LineOffset = i*sizeof(IMG_UINT32);
			IMG_CHAR   *pszBuf = pszLine;

			for (k = 0; k < RGXFW_TRACE_BUFFER_LINESIZE; k++)
			{
				ui32Line |= pui32TraceBuffer[i + k];

				/* prepare the line to print it. The '+1' is because of the trailing '\0' added */
				OSSNPrintf(pszBuf, 9 + 1, " %08x", pui32TraceBuffer[i + k]);
				pszBuf += 9; /* write over the '\0' */
			}

			bLineIsAllZeros = (ui32Line == 0x0);

			if (bLineIsAllZeros && bPrevLineWasZero)
			{
				ui32CountLines++;
			}
			else if (bLineIsAllZeros && !bPrevLineWasZero)
			{
				bPrevLineWasZero = IMG_TRUE;
				ui32CountLines = 0;
				PVR_LOG(("FWT[%08x]: 00000000 ... 00000000", ui32LineOffset))
			}
			else
			{
				if (bPrevLineWasZero)
				{
					PVR_LOG(("FWT[%08x]: %d lines were all zero", ui32LineOffset, ui32CountLines));
				}
				else
				{

					PVR_LOG(("FWT[%08x]:%s", ui32LineOffset, pszLine));
				}
				bPrevLineWasZero = IMG_FALSE;
			}

		}
		if (bPrevLineWasZero)
		{
			PVR_LOG(("FWT[END]: %d lines were all zero", ui32CountLines));
		}

		PVR_LOG(("------[ RGX Kernel FW thread %d trace END ]------", tid));

		OSFreeMem(pszLine);

	}
}

/*
	RGXPanic
*/
IMG_VOID RGXPanic(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	PVR_LOG(("RGX panic"));
	RGXDumpDebugInfo(psDevInfo, IMG_FALSE);
	OSPanic();
}

/*
	RGXQueryDMState
*/
PVRSRV_ERROR RGXQueryDMState(PVRSRV_RGXDEV_INFO *psDevInfo, RGXFWIF_DM eDM, RGXFWIF_DM_STATE *peState, IMG_VOID **ppCommonContext)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	if (eDM >= RGXFWIF_DM_MAX)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: eDM parameter is out of range (%u)",eError));
		return eError;
	}

	if (peState == IMG_NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: peState is NULL (%u)",eError));
		return eError;
	}

	if (ppCommonContext == IMG_NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: ppCommonContext is NULL (%u)",eError));
		return eError;
	}

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: Failed (%d) to acquire adress for trace buffer", eError));
		return eError;
	}

	if (psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[eDM] != psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[eDM])
	{
		*peState = RGXFWIF_DM_STATE_LOCKEDUP;
	}
	else
	{
		*peState = RGXFWIF_DM_STATE_NORMAL;
	}
	
	*ppCommonContext = psRGXFWIfTraceBufCtl->apsHwrDmFWCommonContext[eDM];

	return eError;
}

/*
	RGXHWPerfDataStoreFillBuffer
*/
static IMG_UINT32 RGXHWPerfDataStoreFillBuffer(IMG_HANDLE hHWPerfStream, RGX_HWPERF_PACKET* psFwBuffer, IMG_UINT32 ui32SrcRIdxMasked, IMG_UINT32 ui32PktCount)
{
	PVRSRV_ERROR	eError;


	PVR_DPF_ENTERED;

	//PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfDataStoreFillBuffer srcBuf:0x%p srcRIdx:%05d pktCount:%05d", psFwBuffer, ui32SrcRIdxMasked, ui32PktCount));

	/* Write one TL packet for each HWPerf packet. Stop when we run out of
	 * HWPerf packets to write or when TL steam gets full.
	 */
	while (ui32PktCount)
	{

		eError = TLStreamWrite(hHWPerfStream,
			(IMG_UINT8 *)(psFwBuffer+ui32SrcRIdxMasked), sizeof(RGX_HWPERF_PACKET));

		/* Break when error or TL stream full */
		if (eError != PVRSRV_OK)
		{
			if (eError == PVRSRV_ERROR_DATA_DROPPED)
			{
				PVR_DPF((PVR_DBG_MESSAGE, "HWPerf TL L2 buffer full, may cause packet loss if L1 overflows"));
			}
			else
			{
				PVR_DPF((PVR_DBG_ERROR, "TLStreamWrite() failed (%d) in %s(), unable to copy packet from L1 to L2 buffer", eError, __func__));
			}
			break;
		}

		//PVR_DPF((PVR_DBG_MESSAGE, "TLStreamWrite() output HWPerf packet (%d)", psFwBuffer[ui32SrcRIdxMasked].ui32Ordinal));
		ui32SrcRIdxMasked++;
		ui32PktCount--;
	}

	/* Return the remaining packets left to be transported.
	 */
	PVR_DPF_RETURN_VAL(ui32PktCount);
}


/*
	RGXHWPerfDataStore
*/
IMG_VOID RGXHWPerfDataStore(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	RGXFWIF_TRACEBUF	*psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
	IMG_UINT32			ui32SrcRIdx;
	IMG_UINT32			ui32SrcWIdx;
	IMG_UINT32			ui32SrcRIdxMasked;
	IMG_UINT32			ui32SrcWIdxMasked;
	IMG_UINT32			ui32PktCount;
	IMG_UINT32			ui32PktLeft;


	PVR_DPF_ENTERED;

	/* Caller should check this member is valid before calling */
	PVR_ASSERT(psDevInfo->hHWPerfStream);
	
	/* Obtain the current read and write indexes into FW buffer */
	ui32SrcRIdx = psRGXFWIfTraceBufCtl->ui32HWPerfRIdx;
	ui32SrcWIdx = psRGXFWIfTraceBufCtl->ui32HWPerfWIdx;

	/* Is there any data in the buffer not yet retrieved? */
	if (ui32SrcRIdx != ui32SrcWIdx)
	{
		/* Obtain the true and safe array indexes */
		ui32SrcRIdxMasked = ui32SrcRIdx & RGXFW_HWPERF_FIRMWARE_COUNT_MASK;
		ui32SrcWIdxMasked = ui32SrcWIdx & RGXFW_HWPERF_FIRMWARE_COUNT_MASK;

		PVR_DPF((PVR_DBG_MESSAGE, "RGXHWPerfDataStore EVENTS found srcRIdx:%d (%d) srcWIdx: %d (%d)", ui32SrcRIdxMasked, ui32SrcRIdx, ui32SrcWIdxMasked, ui32SrcWIdx));

		/* Is the write position higher than the read position */
		if (ui32SrcWIdxMasked > ui32SrcRIdxMasked)
		{
			/* Yes, therefore buffer wrap around has not happened */
			ui32PktCount = ui32SrcWIdxMasked - ui32SrcRIdxMasked;
			ui32PktLeft = RGXHWPerfDataStoreFillBuffer(psDevInfo->hHWPerfStream,
					psRGXFWIfTraceBufCtl->asHWPerfPackets, ui32SrcRIdxMasked, ui32PktCount);

			/* Advance the read index by the number of packets we have been able
			 * to transport. Items will be left in buffer if not all packets
			 * could be transported. */
			psRGXFWIfTraceBufCtl->ui32HWPerfRIdx = ui32SrcWIdx-ui32PktLeft;
		}
		else
		{
			/* No, therefore buffer write position has wrapped and is behind read position */

			/* 1st count equals number of packets to the end of the buffer. */
			ui32PktCount = RGXFW_HWPERF_FIRMWARE_COUNT - ui32SrcRIdxMasked;

			/* Attempt to transfer the packets to the TL stream buffer */
			ui32PktLeft = RGXHWPerfDataStoreFillBuffer(psDevInfo->hHWPerfStream,
					psRGXFWIfTraceBufCtl->asHWPerfPackets, ui32SrcRIdxMasked, ui32PktCount);

			/* Any packets not transfered? */
			if (ui32PktLeft == 0)
			{
				/* All packets copied on first fill, perform second fill */
				ui32PktLeft = RGXHWPerfDataStoreFillBuffer(psDevInfo->hHWPerfStream,
						psRGXFWIfTraceBufCtl->asHWPerfPackets, 0, ui32SrcWIdxMasked);

				/* Advance the FW buffer read position to those packets
				 * not sent. */
				psRGXFWIfTraceBufCtl->ui32HWPerfRIdx = ui32SrcWIdx-ui32PktLeft;
			}
			else
			{
				/* Yes, packets left over from first fill, destination TL
				 * stream buffer unable to take any more so skip second fill
				 * and adjust read position accordingly.  */
				psRGXFWIfTraceBufCtl->ui32HWPerfRIdx+= ui32PktCount-ui32PktLeft;
			}

		}
	}
	else
	{
		PVR_DPF((PVR_DBG_VERBOSE, "RGXHWPerfDataStore NO EVENTS to transport"));
	}

	PVR_DPF_RETURN;
}


/******************************************************************************
 * RGX HW Performance Profiling Server API(s)
 *****************************************************************************/

/*
	PVRSRVRGXCtrlHWPerfKM
*/
PVRSRV_ERROR PVRSRVRGXCtrlHWPerfKM(
		PVRSRV_DEVICE_NODE*	psDeviceNode,
		IMG_BOOL			bEnable,
		IMG_UINT32 			ui32Mask)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO* psDevice;
	RGXFWIF_KCCB_CMD 	sKccbCmd;

	PVR_DPF_ENTERED;
	PVR_ASSERT(psDeviceNode);
	psDevice = psDeviceNode->pvDevice;


	/* If it is being asked to enable, does the stream exist, create it not
	 * Stream created on demand on first use to reduce RAM foot print on
	 * systems not needing HWPerf resources.
	 */
	if (bEnable && (psDevice->hHWPerfStream == 0))
	{
		eError = TLStreamCreate(&psDevice->hHWPerfStream, "hwperf", RGXFW_HWPERF_SERVER_COUNT*sizeof(RGX_HWPERF_PACKET), TL_FLAG_DROP_DATA);
		if (eError != PVRSRV_OK)
			PVR_LOGR_IF_ERROR(eError, "TLStreamCreate");
	}

	/* Prepare command parameters ...
	 */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_CTRL_EVENTS;
	sKccbCmd.uCmdData.sHWPerfCtrl.bEnable = bEnable;
	sKccbCmd.uCmdData.sHWPerfCtrl.ui32Mask = ui32Mask;

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfKM parameters set, calling FW"));

	/* Ask the FW to carry out the HWPerf configuration command
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXScheduleCommand");

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfKM command scheduled for FW"));

	/* Wait for FW to complete
	 */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXWaitForFWOp");

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfKM firmware completed"));

	/* If it was being asked to disable then don't delete the stream as the FW
	 * will continue to generate events during the disabling phase. Clean up
	 * will be done when the driver is unloaded.
	 * The increase in extra memory used by the stream would only occur on a
	 * developer system and not a production device as a user would never
	 * enable HWPerf. If this is not the case then a deferred clean system will
	 * need to be implemented.
	 */
	/*if ((!bEnable) && (psDevice->hHWPerfStream))
	{
		TLStreamDestroy(psDevice->hHWPerfStream);
		psDevice->hHWPerfStream = 0;
	}*/

#if defined(DEBUG)
	if (bEnable)
		PVR_DPF((PVR_DBG_WARNING, "HWPerf events have been ENABLED"));
	else
		PVR_DPF((PVR_DBG_WARNING, "HWPerf events have been DISABLED"));
#endif

	PVR_DPF_RETURN_OK;
}


/*
	PVRSRVRGXEnableHWPerfCountersKM
*/
PVRSRV_ERROR PVRSRVRGXConfigEnableHWPerfCountersKM(
		PVRSRV_DEVICE_NODE* 		psDeviceNode,
		IMG_UINT32 					ui32ArrayLen,
		RGX_HWPERF_CONFIG_CNTBLK* 	psBlockConfigs)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD 	sKccbCmd;
	DEVMEM_MEMDESC*		psFwBlkConfigsMemDesc;
	RGX_HWPERF_CONFIG_CNTBLK* psFwArray;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(ui32ArrayLen>0);
	PVR_ASSERT(psBlockConfigs);

	/* Fill in the command structure with the parameters needed
	 */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_CONFIG_ENABLE_BLKS;
	sKccbCmd.uCmdData.sHWPerfCfgEnableBlks.ui32NumBlocks = ui32ArrayLen;

	eError = DevmemFwAllocate(psDeviceNode->pvDevice,
			sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen, 
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
									  PVRSRV_MEMALLOCFLAG_GPU_READABLE | 
					                  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
									  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
									  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | 
									  PVRSRV_MEMALLOCFLAG_UNCACHED |
									  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC,
			&psFwBlkConfigsMemDesc);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXScheduleCommand");

	RGXSetFirmwareAddress(&sKccbCmd.uCmdData.sHWPerfCfgEnableBlks.pasBlockConfigs,
			psFwBlkConfigsMemDesc, 0, 0);

	eError = DevmemAcquireCpuVirtAddr(psFwBlkConfigsMemDesc, (IMG_VOID **)&psFwArray);
	if (eError != PVRSRV_OK)
		PVR_LOGG_IF_ERROR(eError, "DevmemAcquireCpuVirtAddr", fail1);

	OSMemCopy(psFwArray, psBlockConfigs, sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen);
	DevmemPDumpLoadMem(psFwBlkConfigsMemDesc,
						0,
						sizeof(RGX_HWPERF_CONFIG_CNTBLK)*ui32ArrayLen,
						0);

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM parameters set, calling FW"));

	/* Ask the FW to carry out the HWPerf configuration command
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGG_IF_ERROR(eError, "RGXScheduleCommand", fail2);

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM command scheduled for FW"));

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGG_IF_ERROR(eError, "RGXWaitForFWOp", fail2);

	/* Release temporary memory used for block configuration
	 */
	RGXUnsetFirmwareAddress(psFwBlkConfigsMemDesc);
	DevmemReleaseCpuVirtAddr(psFwBlkConfigsMemDesc);
	DevmemFwFree(psFwBlkConfigsMemDesc);

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXConfigEnableHWPerfCountersKM firmware completed"));

	PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks configured and ENABLED",  ui32ArrayLen));

	PVR_DPF_RETURN_OK;

fail2:
	DevmemReleaseCpuVirtAddr(psFwBlkConfigsMemDesc);
fail1:
	RGXUnsetFirmwareAddress(psFwBlkConfigsMemDesc);
	DevmemFwFree(psFwBlkConfigsMemDesc);

	PVR_DPF_RETURN_RC(eError);
}


/*
	PVRSRVRGXDisableHWPerfcountersKM
*/
PVRSRV_ERROR PVRSRVRGXCtrlHWPerfCountersKM(
		PVRSRV_DEVICE_NODE*		psDeviceNode,
		IMG_BOOL				bEnable,
	    IMG_UINT32 				ui32ArrayLen,
	    IMG_UINT8*				psBlockIDs)
{
	PVRSRV_ERROR 		eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD 	sKccbCmd;

	PVR_DPF_ENTERED;

	PVR_ASSERT(psDeviceNode);
	PVR_ASSERT(ui32ArrayLen>0);
	PVR_ASSERT(psBlockIDs);

	/* Fill in the command structure with the parameters needed
	 */
	sKccbCmd.eCmdType = RGXFWIF_KCCB_CMD_HWPERF_CTRL_BLKS;
	sKccbCmd.uCmdData.sHWPerfCtrlBlks.bEnable = bEnable;
	sKccbCmd.uCmdData.sHWPerfCtrlBlks.ui32NumBlocks = ui32ArrayLen;
	OSMemCopy(sKccbCmd.uCmdData.sHWPerfCtrlBlks.aeBlockIDs, psBlockIDs, sizeof(IMG_UINT8)*ui32ArrayLen);

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM parameters set, calling FW"));

	/* Ask the FW to carry out the HWPerf configuration command
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
			RGXFWIF_DM_GP, &sKccbCmd, sizeof(sKccbCmd), IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXScheduleCommand");

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM command scheduled for FW"));

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
	if (eError != PVRSRV_OK)
		PVR_LOGR_IF_ERROR(eError, "RGXWaitForFWOp");

	//PVR_DPF((PVR_DBG_VERBOSE, "PVRSRVRGXCtrlHWPerfCountersKM firmware completed"));

#if defined(DEBUG)
	if (bEnable)
		PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks have been ENABLED",  ui32ArrayLen));
	else
		PVR_DPF((PVR_DBG_WARNING, "HWPerf %d counter blocks have been DISBALED",  ui32ArrayLen));
#endif

	PVR_DPF_RETURN_OK;
}

/******************************************************************************
 End of file (rgxdebug.c)
******************************************************************************/
