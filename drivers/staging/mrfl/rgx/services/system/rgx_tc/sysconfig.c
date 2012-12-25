/*************************************************************************/ /*!
@File
@Title          System Configuration
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    System Configuration functions
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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include "pvr_debug.h"
#include "allocmem.h"
#include "pvrsrv_device.h"
#include "syscommon.h"
#include "sysinfo.h"
#include "sysconfig.h"
#include "physheap.h"
#include "pci_support.h"
#include "tcf_clk_ctrl.h"
#include "tcf_pll.h"

#if defined(LDM_PCI) || defined(SUPPORT_DRM)
/* The following is exported by the Linux module code */
extern struct pci_dev *gpsPVRLDMDev;
#endif

typedef struct _SYS_DATA_
{
	IMG_HANDLE	hRGXPCI;

	IMG_CHAR	*pszSystemInfoString;

	IMG_CPU_PHYADDR	sSystemRegCpuPBase;
	IMG_VOID	*pvSystemRegCpuVBase;
	IMG_UINT32	uiSystemRegSize;
	PVRSRV_SYS_POWER_STATE	ePowerState;
} SYS_DATA;

#define SYSTEM_INFO_FORMAT_STRING	"%s\tFPGA Revision: %s.%s.%s\tTCF Core Revision: %s.%s.%s\tTCF Core Target Build ID: %s\tPCI Version: %s\tMacro Version: %s.%s"
#define SYSTEM_INFO_REV_NUM_LEN		3

/* The version information is encoded so that each hex value is limited to between 0-9. So 15 would 
   appear as 0x15 instead of 0xF. This macro converts it from the first form to the second. */
#define UINT8_HEX_TO_DEC(hexIntVal)	((((hexIntVal) >> 4) * 10) + ((hexIntVal) & 0x0F))

static IMG_CHAR *GetSystemInfoString(SYS_DATA *psSysData)
{
	IMG_CHAR apszFPGARev[3][SYSTEM_INFO_REV_NUM_LEN];
	IMG_CHAR apszCoreRev[3][SYSTEM_INFO_REV_NUM_LEN];
	IMG_CHAR apszConfigRev[SYSTEM_INFO_REV_NUM_LEN];
	IMG_CHAR apszPCIVer[SYSTEM_INFO_REV_NUM_LEN];
	IMG_CHAR apszMacroVer[2][SYSTEM_INFO_REV_NUM_LEN];
	IMG_CHAR *pszSystemInfoString;
	IMG_UINT32 ui32StringLength;
	IMG_UINT32 ui32Value;
	IMG_CPU_PHYADDR	sHostFPGARegCpuPBase;
	IMG_VOID *pvHostFPGARegCpuVBase;

	/* To get some of the version information we need to read from a register that we don't normally have 
	   mapped. Map it temporarily (without trying to reserve it) to get the information we need. */
	sHostFPGARegCpuPBase.uiAddr	= OSPCIAddrRangeStart(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM) + 0x40F0;
	pvHostFPGARegCpuVBase		= OSMapPhysToLin(sHostFPGARegCpuPBase, 0x04, 0);
	if (pvHostFPGARegCpuVBase == NULL)
	{
		return NULL;
	}

	/* Create the components of the PCI and macro versions */
	ui32Value = OSReadHWReg32(pvHostFPGARegCpuVBase, 0);
	snprintf(&apszPCIVer[0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & 0x00FF0000) >> 16));
	snprintf(&apszMacroVer[0][0], SYSTEM_INFO_REV_NUM_LEN, "%d", ((ui32Value & 0x00000F00) >> 8));
	snprintf(&apszMacroVer[1][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & 0x000000FF) >> 0));

	/* Unmap the register now that we no longer need it */
	OSUnMapPhysToLin(pvHostFPGARegCpuVBase, 0x04, 0);

	/* Create the components of the FPGA revision number */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_FPGA_REV_REG);
	snprintf(&apszFPGARev[0][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & FPGA_REV_REG_MAJOR_MASK) >> FPGA_REV_REG_MAJOR_SHIFT));
	snprintf(&apszFPGARev[1][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & FPGA_REV_REG_MINOR_MASK) >> FPGA_REV_REG_MINOR_SHIFT));
	snprintf(&apszFPGARev[2][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & FPGA_REV_REG_MAINT_MASK) >> FPGA_REV_REG_MAINT_SHIFT));

	/* Create the components of the TCF core revision number */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TCF_CORE_REV_REG);
	snprintf(&apszCoreRev[0][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & TCF_CORE_REV_REG_MAJOR_MASK) >> TCF_CORE_REV_REG_MAJOR_SHIFT));
	snprintf(&apszCoreRev[1][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & TCF_CORE_REV_REG_MINOR_MASK) >> TCF_CORE_REV_REG_MINOR_SHIFT));
	snprintf(&apszCoreRev[2][0], SYSTEM_INFO_REV_NUM_LEN, "%d", UINT8_HEX_TO_DEC((ui32Value & TCF_CORE_REV_REG_MAINT_MASK) >> TCF_CORE_REV_REG_MAINT_SHIFT));

	/* Create the component of the TCF core target build ID */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TCF_CORE_TARGET_BUILD_CFG);
	snprintf(&apszConfigRev[0], SYSTEM_INFO_REV_NUM_LEN, "%d", (ui32Value & TCF_CORE_TARGET_BUILD_ID_MASK) >> TCF_CORE_TARGET_BUILD_ID_SHIFT);

	/* Calculate how much space we need to allocate for the string */
	ui32StringLength = strlen(SYSTEM_INFO_FORMAT_STRING);
	ui32StringLength += strlen(TC_SYSTEM_NAME);
	ui32StringLength += strlen(&apszFPGARev[0][0]) + strlen(&apszFPGARev[1][0]) + strlen(&apszFPGARev[2][0]);
	ui32StringLength += strlen(&apszCoreRev[0][0]) + strlen(&apszCoreRev[1][0]) + strlen(&apszCoreRev[2][0]);
	ui32StringLength += strlen(&apszConfigRev[0]);
	ui32StringLength += strlen(&apszPCIVer[0]);
	ui32StringLength += strlen(&apszMacroVer[0][0]) + strlen(&apszMacroVer[1][0]);

	/* Create the system info string */
	pszSystemInfoString = OSAllocZMem(ui32StringLength * sizeof(IMG_CHAR));
	if (pszSystemInfoString)
	{
		snprintf(&pszSystemInfoString[0], ui32StringLength, SYSTEM_INFO_FORMAT_STRING, TC_SYSTEM_NAME, 
			 &apszFPGARev[0][0], &apszFPGARev[1][0], &apszFPGARev[2][0],
			 &apszCoreRev[0][0], &apszCoreRev[1][0], &apszCoreRev[2][0],
			 &apszConfigRev[0], &apszPCIVer[0],
			 &apszMacroVer[0][0], &apszMacroVer[1][0]);
	}

	return pszSystemInfoString;
}

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL) || (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
static PVRSRV_ERROR AcquireLocalMemory(SYS_DATA *psSysData, IMG_UINT32 *puiMemCpuPAddr, IMG_UINT32 *puiMemSize)
{
	IMG_UINT32 uiMemCpuPAddr;
	IMG_UINT32 uiExpectedMemSize;
	IMG_UINT32 uiMemSize;
	IMG_UINT16 uiDevID;
	PVRSRV_ERROR eError;

	uiMemCpuPAddr	= OSPCIAddrRangeStart(psSysData->hRGXPCI, SYS_RGX_MEM_PCI_BASENUM);
	uiMemSize	= OSPCIAddrRangeLen(psSysData->hRGXPCI, SYS_RGX_MEM_PCI_BASENUM);

	OSPCIDevID(psSysData->hRGXPCI, &uiDevID);
	switch (uiDevID)
	{
		default:
			PVR_DPF((PVR_DBG_WARNING, 
				 "%s: Unexpected device ID. Checking device mem region size against 0x%08X", 
				 __FUNCTION__, SYS_RGX_DEV1_MEM_REGION_SIZE));
		case SYS_RGX_DEV_DEVICE_ID:
			uiExpectedMemSize = SYS_RGX_DEV1_MEM_REGION_SIZE;
			break;
		case SYS_RGX_DEV1_DEVICE_ID:
			uiExpectedMemSize = SYS_RGX_DEV2_MEM_REGION_SIZE;
			break;
	}

	/* Check the address range is large enough. */
	if (uiMemSize < uiExpectedMemSize)
	{
		PVR_DPF((PVR_DBG_ERROR, 
			 "%s: Device memory region isn't big enough (was 0x%08x, required 0x%08x)",
			 __FUNCTION__, uiMemSize, SYS_RGX_DEV1_MEM_REGION_SIZE));

		return PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
	}

	/* Reserve the address range */
	eError = OSPCIRequestAddrRange(psSysData->hRGXPCI, SYS_RGX_MEM_PCI_BASENUM);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Device memory region not available", __FUNCTION__));

		return eError;
	}

	/* Clear any BIOS-configured MTRRs */
	eError = OSPCIClearResourceMTRRs(psSysData->hRGXPCI, SYS_RGX_MEM_PCI_BASENUM);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: Failed to clear BIOS MTRRs", __FUNCTION__));
		/* Soft-fail, the driver can limp along. */
	}

	*puiMemCpuPAddr = uiMemCpuPAddr;
	*puiMemSize = uiExpectedMemSize;

	return PVRSRV_OK;
}

static inline void ReleaseLocalMemory(SYS_DATA *psSysData)
{
	OSPCIReleaseAddrRange(psSysData->hRGXPCI, SYS_RGX_MEM_PCI_BASENUM);
}
#endif /* (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL) || (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID) */

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
static PVRSRV_ERROR InitLocalMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	IMG_UINT32 uiMemCpuPAddr;
	IMG_UINT32 uiMemSize;
	IMG_UINT32 ui32Value;
	PVRSRV_ERROR eError;

	eError = AcquireLocalMemory(psSysData, &uiMemCpuPAddr, &uiMemSize);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/* Setup the RGX heap */
	psSysConfig->pasPhysHeaps[0].sStartAddr.uiAddr	= uiMemCpuPAddr;
	psSysConfig->pasPhysHeaps[0].uiSize		= uiMemSize - RGX_TC_RESERVE_DC_MEM_SIZE;

	/* Setup the DC heap */
	psSysConfig->pasPhysHeaps[1].sStartAddr.uiAddr	= uiMemCpuPAddr + psSysConfig->pasPhysHeaps[0].uiSize;
	psSysConfig->pasPhysHeaps[1].uiSize		= RGX_TC_RESERVE_DC_MEM_SIZE;

	/* Configure Apollo for regression compatibility (i.e. local memory) mode */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	ui32Value &= ~(ADDRESS_FORCE_MASK | PCI_TEST_MODE_MASK | HOST_ONLY_MODE_MASK | HOST_PHY_MODE_MASK);
	ui32Value |= (0x1 << ADDRESS_FORCE_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, ui32Value);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);

	return PVRSRV_OK;
}

static void DeInitLocalMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	ReleaseLocalMemory(psSysData);

	psSysConfig->pasPhysHeaps[0].sStartAddr.uiAddr	= 0;
	psSysConfig->pasPhysHeaps[0].uiSize		= 0;
	psSysConfig->pasPhysHeaps[1].sStartAddr.uiAddr	= 0;
	psSysConfig->pasPhysHeaps[1].uiSize		= 0;

	/* Set the register back to the default value */
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, 0x1 << ADDRESS_FORCE_SHIFT);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);
}
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)
static PVRSRV_ERROR InitHostMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	IMG_UINT32 ui32Value;

	PVR_UNREFERENCED_PARAMETER(psSysConfig);

	/* Configure Apollo for host only mode */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	ui32Value &= ~(ADDRESS_FORCE_MASK | PCI_TEST_MODE_MASK | HOST_ONLY_MODE_MASK | HOST_PHY_MODE_MASK);
	ui32Value |= (0x1 << HOST_ONLY_MODE_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, ui32Value);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);

	return PVRSRV_OK;
}

static void DeInitHostMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	PVR_UNREFERENCED_PARAMETER(psSysConfig);

	/* Set the register back to the default value */
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, 0x1 << ADDRESS_FORCE_SHIFT);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);
}
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
static PVRSRV_ERROR InitHybridMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	IMG_UINT32 uiMemCpuPAddr;
	IMG_UINT32 uiMemSize;
	IMG_UINT32 ui32Value;
	PVRSRV_ERROR eError;

	eError = AcquireLocalMemory(psSysData, &uiMemCpuPAddr, &uiMemSize);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/* Rogue is using system memory so there is no additional heap setup needed */

	/* Setup the DC heap */
	psSysConfig->pasPhysHeaps[1].sStartAddr.uiAddr	= uiMemCpuPAddr;
	psSysConfig->pasPhysHeaps[1].uiSize		= uiMemSize;

	/* Configure Apollo for host physical (i.e. hybrid) mode */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	ui32Value &= ~(ADDRESS_FORCE_MASK | PCI_TEST_MODE_MASK | HOST_ONLY_MODE_MASK | HOST_PHY_MODE_MASK);
	ui32Value |= ((0x1 << HOST_ONLY_MODE_SHIFT) | (0x1 << HOST_PHY_MODE_SHIFT));
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, ui32Value);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);

	/* Setup the start address of the 1GB window which is redirected to local memory */
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_HOST_PHY_OFFSET, uiMemCpuPAddr);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_HOST_PHY_OFFSET);
	OSWaitus(10);

	return PVRSRV_OK;
}

static void DeInitHybridMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
	ReleaseLocalMemory(psSysData);

	psSysConfig->pasPhysHeaps[1].sStartAddr.uiAddr	= 0;
	psSysConfig->pasPhysHeaps[1].uiSize		= 0;

	/* Set the register back to the default value */
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL, 0x1 << ADDRESS_FORCE_SHIFT);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_TEST_CTRL);
	OSWaitus(10);
}
#endif

static PVRSRV_ERROR InitMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
	return InitLocalMemory(psSysConfig, psSysData);
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)
	return InitHostMemory(psSysConfig, psSysData);
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
	return InitHybridMemory(psSysConfig, psSysData);
#endif
}

static inline void DeInitMemory(PVRSRV_SYSTEM_CONFIG *psSysConfig, SYS_DATA *psSysData)
{
#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
	return DeInitLocalMemory(psSysConfig, psSysData);
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)
	return DeInitHostMemory(psSysConfig, psSysData);
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
	return DeInitHybridMemory(psSysConfig, psSysData);
#endif
}

static IMG_VOID EnableInterrupts(SYS_DATA *psSysData)
{
	IMG_UINT32 ui32Value;

	/* Set sense to active high */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_OP_CFG);
	ui32Value &= ~(INT_SENSE_MASK);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_OP_CFG, ui32Value);
	OSWaitus(1000);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_OP_CFG);

	/* Enable Rogue and PDP1 interrupts */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE);
	ui32Value |= (0x1 << EXT_INT_SHIFT) | (0x1 << PDP1_INT_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE, ui32Value);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE);
	OSWaitus(10);
}

static IMG_VOID DisableInterrupts(SYS_DATA *psSysData)
{
	IMG_UINT32 ui32Value;

	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE);
	ui32Value &= ~(EXT_INT_MASK | PDP1_INT_MASK);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE, ui32Value);

	/* Flush register write */
	(void)OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_ENABLE);
	OSWaitus(10);
}

#define pol(base,reg,val,msk) \
	do { \
		int polnum; \
		for (polnum = 0; polnum < 500; polnum++) \
		{ \
			if ((OSReadHWReg32(base, reg) & msk) == val) \
			{ \
				break; \
			} \
			udelay(1000); \
		} \
		if (polnum == 500) \
		{ \
			PVR_DPF((PVR_DBG_WARNING, "Pol failed for register: 0x%08X", (unsigned int)reg)); \
		} \
	} while (0)

#define polrgx(reg,val,msk) pol(pvRegsBaseKM, reg, val, msk)

static IMG_VOID RGXInitBistery(IMG_CPU_PHYADDR sRegisters, IMG_UINT32 ui32Size)
{
	IMG_VOID *pvRegsBaseKM = OSMapPhysToLin(sRegisters, ui32Size, 0);
	IMG_UINT instance;
	IMG_UINT i;

	/* Force clocks on */
	OSWriteHWReg32(pvRegsBaseKM, 0, 0x55555555);
	OSWriteHWReg32(pvRegsBaseKM, 4, 0x55555555);

	polrgx(0xa18, 0x05000000, 0x05000000);
	OSWriteHWReg32(pvRegsBaseKM, 0xa10, 0x048000b0);
	OSWriteHWReg32(pvRegsBaseKM, 0xa08, 0x55111111);
	polrgx(0xa18, 0x05000000, 0x05000000);

	/* Clear PDS CSRM and USRM to prevent ERRORs at end of test */
	OSWriteHWReg32(pvRegsBaseKM, 0x630, 0x1);
	OSWriteHWReg32(pvRegsBaseKM, 0x648, 0x1);
	OSWriteHWReg32(pvRegsBaseKM, 0x608, 0x1);

	/* Run BIST for SLC (43) */
	/* Reset BIST */
	OSWriteHWReg32(pvRegsBaseKM, 0x7000, 0x8);
	udelay(100);

	/* Clear BIST controller */
	OSWriteHWReg32(pvRegsBaseKM, 0x7000, 0x10);
	OSWriteHWReg32(pvRegsBaseKM, 0x7000, 0);
	udelay(100);

	for (i = 0; i < 3; i++)
	{
		IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

		/* Start BIST */
		OSWriteHWReg32(pvRegsBaseKM, 0x7000, 0x4);

		udelay(100);

		/* Wait for pause */
		polrgx(0x7000, ui32Pol, ui32Pol);
	}
	udelay(100);

	/* Check results for 43 RAMs */
	polrgx(0x7010, 0xffffffff, 0xffffffff);
	polrgx(0x7014, 0x7, 0x7);

	OSWriteHWReg32(pvRegsBaseKM, 0x7000, 8);
	OSWriteHWReg32(pvRegsBaseKM, 0x7008, 0);
	OSWriteHWReg32(pvRegsBaseKM, 0x7000, 6);
	polrgx(0x7000, 0x00010000, 0x00010000);
	udelay(100);

	polrgx(0x75B0, 0, ~0U);
	polrgx(0x75B4, 0, ~0U);
	polrgx(0x75B8, 0, ~0U);
	polrgx(0x75BC, 0, ~0U);
	polrgx(0x75C0, 0, ~0U);
	polrgx(0x75C4, 0, ~0U);
	polrgx(0x75C8, 0, ~0U);
	polrgx(0x75CC, 0, ~0U);

	/* Sidekick */
	OSWriteHWReg32(pvRegsBaseKM, 0x7040, 8);
	udelay(100);

	OSWriteHWReg32(pvRegsBaseKM, 0x7040, 0x10);
	//OSWriteHWReg32(pvRegsBaseKM, 0x7000, 0);
	udelay(100);

	for (i = 0; i < 3; i++)
	{
		IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

		OSWriteHWReg32(pvRegsBaseKM, 0x7040, 4);
		udelay(100);
		polrgx(0x7040, ui32Pol, ui32Pol);
	}

	udelay(100);
	polrgx(0x7050, 0xffffffff, 0xffffffff);
	polrgx(0x7054, 0xffffffff, 0xffffffff);
	polrgx(0x7058, 0x1, 0x1);

	/* USC */
	for (instance = 0; instance < 4; instance++)
	{
		OSWriteHWReg32(pvRegsBaseKM, 0x8010, instance);

		OSWriteHWReg32(pvRegsBaseKM, 0x7088, 8);
		udelay(100);

		OSWriteHWReg32(pvRegsBaseKM, 0x7088, 0x10);
		udelay(100);

		for (i = 0; i < 3; i++)
		{
			IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

			OSWriteHWReg32(pvRegsBaseKM, 0x7088, 4);
			udelay(100);
			polrgx(0x7088, ui32Pol, ui32Pol);
		}

		udelay(100);
		polrgx(0x7098, 0xffffffff, 0xffffffff);
		polrgx(0x709c, 0xffffffff, 0xffffffff);
		polrgx(0x70a0, 0x3f, 0x3f);
	}

	/* tpumcul0 DustA and DustB */
	for (instance = 0; instance < 2; instance++)
	{
		OSWriteHWReg32(pvRegsBaseKM, 0x8018, instance);

		OSWriteHWReg32(pvRegsBaseKM, 0x7380, 8);
		udelay(100);

		OSWriteHWReg32(pvRegsBaseKM, 0x7380, 0x10);
		udelay(100);

		for (i = 0; i < 3; i++)
		{
			IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

			OSWriteHWReg32(pvRegsBaseKM, 0x7380, 4);
			udelay(100);
			polrgx(0x7380, ui32Pol, ui32Pol);
		}

		udelay(100);
		polrgx(0x7390, 0x1fff, 0x1fff);
	}

	/* TA */
	OSWriteHWReg32(pvRegsBaseKM, 0x7500, 8);
	udelay(100);

	OSWriteHWReg32(pvRegsBaseKM, 0x7500, 0x10);
	udelay(100);

	for (i = 0; i < 3; i++)
	{
		IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

		OSWriteHWReg32(pvRegsBaseKM, 0x7500, 4);
		udelay(100);
		polrgx(0x7500, ui32Pol, ui32Pol);
	}

	udelay(100);
	polrgx(0x7510, 0x1fffffff, 0x1fffffff);

	/* Rasterisation */
	OSWriteHWReg32(pvRegsBaseKM, 0x7540, 8);
	udelay(100);

	OSWriteHWReg32(pvRegsBaseKM, 0x7540, 0x10);
	udelay(100);

	for (i = 0; i < 3; i++)
	{
		IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

		OSWriteHWReg32(pvRegsBaseKM, 0x7540, 4);
		udelay(100);
		polrgx(0x7540, ui32Pol, ui32Pol);
	}

	udelay(100);
	polrgx(0x7550, 0xffffffff, 0xffffffff);
	polrgx(0x7554, 0xffffffff, 0xffffffff);
	polrgx(0x7558, 0xf, 0xf);

	/* hub_bifpmache */
	OSWriteHWReg32(pvRegsBaseKM, 0x7588, 8);
	udelay(100);

	OSWriteHWReg32(pvRegsBaseKM, 0x7588, 0x10);
	udelay(100);

	for (i = 0; i < 3; i++)
	{
		IMG_UINT32 ui32Pol = i == 2 ? 0x10000 : 0x20000;

		OSWriteHWReg32(pvRegsBaseKM, 0x7588, 4);
		udelay(100);
		polrgx(0x7588, ui32Pol, ui32Pol);
	}

	udelay(100);
	polrgx(0x7598, 0xffffffff, 0xffffffff);
	polrgx(0x759c, 0xffffffff, 0xffffffff);
	polrgx(0x75a0, 0x1111111f, 0x1111111f);

	OSUnMapPhysToLin(pvRegsBaseKM, ui32Size, 0);
}

static IMG_VOID ApolloHardReset(SYS_DATA *psSysData)
{
	IMG_UINT32 ui32Value;

	ui32Value = (0x1 << GLB_CLKG_EN_SHIFT);
	ui32Value |= (0x1 << SCB_RESETN_SHIFT);
	ui32Value |= (0x1 << PDP2_RESETN_SHIFT);
	ui32Value |= (0x1 << PDP1_RESETN_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_CLK_AND_RST_CTRL, ui32Value);

	ui32Value |= (0x1 << DDR_RESETN_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_CLK_AND_RST_CTRL, ui32Value);

	ui32Value |= (0x1 << DUT_RESETN_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_CLK_AND_RST_CTRL, ui32Value);

	ui32Value |= (0x1 << DUT_DCM_RESETN_SHIFT);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_CLK_AND_RST_CTRL, ui32Value);

	udelay(1000);
	udelay(1000);
	udelay(1000);
	udelay(1000);
	pol(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_DCM_LOCK_STATUS, 0x7, DCM_LOCK_STATUS_MASK);
}

#undef pol
#undef polrgx


static PVRSRV_ERROR SetClocks(SYS_DATA *psSysData, IMG_UINT32 ui32CoreClock, IMG_UINT32 ui32MemClock)
{
	IMG_CPU_PHYADDR	sPLLRegCpuPBase;
	IMG_VOID *pvPLLRegCpuVBase;
	IMG_UINT32 ui32Value;
	IMG_UINT32 i;
	PVRSRV_ERROR eError;

	/* Reserve the PLL register region and map the registers in */
	eError = OSPCIRequestAddrRegion(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM, SYS_APOLLO_REG_PLL_OFFSET, SYS_APOLLO_REG_PLL_SIZE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to request the PLL register region (%d)", __FUNCTION__, eError));

		return eError;
	}
	sPLLRegCpuPBase.uiAddr = OSPCIAddrRangeStart(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM) + SYS_APOLLO_REG_PLL_OFFSET;

	pvPLLRegCpuVBase = OSMapPhysToLin(sPLLRegCpuPBase, SYS_APOLLO_REG_PLL_SIZE, 0);
	if (pvPLLRegCpuVBase == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to map PLL registers", __FUNCTION__));

		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* Modify the core clock */
	OSWriteHWReg32(pvPLLRegCpuVBase, TCF_PLL_PLL_CORE_CLK0, (ui32CoreClock / 1000000));

	ui32Value = 0x1 << PLL_CORE_DRP_GO_SHIFT;
	OSWriteHWReg32(pvPLLRegCpuVBase, TCF_PLL_PLL_CORE_DRP_GO, ui32Value);

	for (i = 0; i < 1000; i++)
	{
		udelay(600);
	}

	/* Modify the memory clock */
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_PLL_PLL_MEMIF_CLK0, (ui32MemClock / 1000000));

	ui32Value = 0x1 << PLL_MEM_DRP_GO_SHIFT;
	OSWriteHWReg32(pvPLLRegCpuVBase, TCF_PLL_PLL_MEM_DRP_GO, ui32Value);

	for (i = 0; i < 1000; i++)
	{
		udelay(600);
	}

	/* Unmap and release the PLL registers since we no longer need access to them */
	OSUnMapPhysToLin(pvPLLRegCpuVBase, SYS_APOLLO_REG_PLL_SIZE, 0);
	OSPCIReleaseAddrRegion(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM, SYS_APOLLO_REG_PLL_OFFSET, SYS_APOLLO_REG_PLL_SIZE);

	return PVRSRV_OK;
}

static irqreturn_t SystemISRHandler(int irq, void *dev_id)
{
	SYS_DATA *psSysData = (SYS_DATA *)dev_id;
	IMG_UINT32 ui32InterruptClear = 0;
	IMG_UINT32 ui32InterruptStatus;

	ui32InterruptStatus = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_STATUS);

	if (ui32InterruptStatus & PDP1_INT_MASK)
	{
		ui32InterruptClear |= (0x1 << PDP1_INT_SHIFT);
	}

	if (ui32InterruptStatus & EXT_INT_MASK)
	{
		ui32InterruptClear = (0x1 << EXT_INT_SHIFT);
	}

	if (ui32InterruptClear)
	{
		OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_INTERRUPT_CLEAR, ui32InterruptClear);
	}

	/* Always return IRQ_NONE because we don't actually handle the interrupts here. 
	   The expectation is that Rogue and the PDP will clear their own interrupts and 
	   return IRQ_HANDLED. However, if we don't clear the system control interrupt 
	   register then the interrupt line will stay asserted. */
	return IRQ_NONE;
}

static PVRSRV_ERROR PCIInitDev(SYS_DATA *psSysData)
{
	PVRSRV_DEVICE_CONFIG *psDevice = &gsSysConfig.pasDevices[0];
	IMG_CPU_PHYADDR	sApolloRegCpuPBase;
	IMG_UINT32 uiApolloRegSize;
	IMG_UINT32 ui32Value;
	IMG_UINT32 i;
	PVRSRV_ERROR eError;

#if defined(LDM_PCI) || defined(SUPPORT_DRM)
	/* Use the pci_dev structure pointer from module.c */
	PVR_ASSERT(gpsPVRLDMDev != IMG_NULL);

	psSysData->hRGXPCI = OSPCISetDev((IMG_VOID *)gpsPVRLDMDev, HOST_PCI_INIT_FLAG_BUS_MASTER);
#else
	psSysData->hRGXPCI = OSPCIAcquireDev(SYS_RGX_DEV_VENDOR_ID, SYS_RGX_DEV_DEVICE_ID, HOST_PCI_INIT_FLAG_BUS_MASTER);

#if defined(SYS_RGX_DEV1_DEVICE_ID)
	if (!psSysData->hRGXPCI)
	{
		PVR_DPF((PVR_DBG_MESSAGE, "%s: Trying alternative PCI device ID", __FUNCTION__));

		psSysData->hRGXPCI = OSPCIAcquireDev(SYS_RGX_DEV_VENDOR_ID, SYS_RGX_DEV1_DEVICE_ID, HOST_PCI_INIT_FLAG_BUS_MASTER);
	}
#endif /* defined(SYS_RGX_DEV1_DEVICE_ID) */
#endif /* defined(LDM_PCI) || defined(SUPPORT_DRM) */
	if (!psSysData->hRGXPCI)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire PCI device", __FUNCTION__));
		return PVRSRV_ERROR_PCI_DEVICE_NOT_FOUND;
	}

	/* Get Apollo register information */
	sApolloRegCpuPBase.uiAddr	= OSPCIAddrRangeStart(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM);
	uiApolloRegSize			= OSPCIAddrRangeLen(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM);

	/* Check the address range is large enough. */
	if (uiApolloRegSize < SYS_APOLLO_REG_REGION_SIZE)
	{
		PVR_DPF((PVR_DBG_ERROR, 
			 "%s: Apollo register region isn't big enough (was 0x%08X, required 0x%08X)",
			 __FUNCTION__, uiApolloRegSize, SYS_APOLLO_REG_REGION_SIZE));

		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto ErrorPCIReleaseDevice;
	}

	/* The Apollo register range contains several register regions. Request only the system control register region */
	eError = OSPCIRequestAddrRegion(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM, SYS_APOLLO_REG_SYS_OFFSET, SYS_APOLLO_REG_SYS_SIZE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to request the system register region", __FUNCTION__));

		goto ErrorPCIReleaseDevice;
	}
	psSysData->sSystemRegCpuPBase.uiAddr	= sApolloRegCpuPBase.uiAddr + SYS_APOLLO_REG_SYS_OFFSET;
	psSysData->uiSystemRegSize		= SYS_APOLLO_REG_SYS_SIZE;

	/* Setup Rogue register information */
	psDevice->sRegsCpuPBase.uiAddr	= OSPCIAddrRangeStart(psSysData->hRGXPCI, SYS_RGX_REG_PCI_BASENUM);
	psDevice->ui32RegsSize		= OSPCIAddrRangeLen(psSysData->hRGXPCI, SYS_RGX_REG_PCI_BASENUM);

	/* Check the address range is large enough. */
	if (psDevice->ui32RegsSize < SYS_RGX_REG_REGION_SIZE)
	{
		PVR_DPF((PVR_DBG_ERROR, 
			 "%s: Rogue register region isn't big enough (was 0x%08x, required 0x%08x)",
			 __FUNCTION__, psDevice->ui32RegsSize, SYS_RGX_REG_REGION_SIZE));

		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto ErrorSystemRegReleaseAddrRegion;
	}

	/* Reserve the address range */
	eError = OSPCIRequestAddrRange(psSysData->hRGXPCI, SYS_RGX_REG_PCI_BASENUM);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Rogue register memory region not available", __FUNCTION__));

		goto ErrorSystemRegReleaseAddrRegion;
	}

	/* Map in system registers so we can:
	   - Configure the memory mode (LMA, UMA or LMA/UMA hybrid)
	   - Hard reset Apollo
	   - Run BIST
	   - Clear interrupts
	*/
	psSysData->pvSystemRegCpuVBase = OSMapPhysToLin(psSysData->sSystemRegCpuPBase, psSysData->uiSystemRegSize, 0);
	if (psSysData->pvSystemRegCpuVBase == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to map system registers", __FUNCTION__));

		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorRGXRegReleaseAddrRange;
	}

	ApolloHardReset(psSysData);
	RGXInitBistery(psDevice->sRegsCpuPBase, psDevice->ui32RegsSize);
	ApolloHardReset(psSysData);

	eError = InitMemory(&gsSysConfig, psSysData);
	if (eError != PVRSRV_OK)
	{
		goto ErrorSystemRegUnMapPhysToLin;
	}

	if (SetClocks(psSysData, RGX_TC_CORE_CLOCK_SPEED, RGX_TC_MEM_CLOCK_SPEED) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to set the core and memory clocks", __FUNCTION__));
	}

	/* Enable the rogue PLL (defaults to 3x), giving a Rogue clock of 3 x RGX_TC_CORE_CLOCK_SPEED */
	ui32Value = OSReadHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_DUT_CONTROL_1);
	OSWriteHWReg32(psSysData->pvSystemRegCpuVBase, TCF_CLK_CTRL_DUT_CONTROL_1, ui32Value & 0xFFFFFFFB);
	for (i = 0; i < 1000; i++)
	{
		udelay(600);
	}

	/* Override the system name if we can get the system info string */
	psSysData->pszSystemInfoString = GetSystemInfoString(psSysData);
	if (psSysData->pszSystemInfoString)
	{
		gsSysConfig.pszSystemName = psSysData->pszSystemInfoString;
	}

	eError = OSPCIIRQ(psSysData->hRGXPCI, &psDevice->ui32IRQ);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Couldn't get IRQ", __FUNCTION__));

		goto ErrorDeInitMemory;
	}

	/* Register our handler */
	if (request_irq(psDevice->ui32IRQ, SystemISRHandler, IRQF_SHARED, PVRSRV_MODNAME, psSysData))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to install the system device interrupt handler", __FUNCTION__));

		eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_ISR_HANDLER;
		goto ErrorDeInitMemory;
	}

	EnableInterrupts(psSysData);

	return PVRSRV_OK;

ErrorDeInitMemory:
	DeInitMemory(&gsSysConfig, psSysData);

ErrorSystemRegUnMapPhysToLin:
	OSUnMapPhysToLin(psSysData->pvSystemRegCpuVBase, psSysData->uiSystemRegSize, 0);
	psSysData->pvSystemRegCpuVBase = NULL;

ErrorRGXRegReleaseAddrRange:
	OSPCIReleaseAddrRange(psSysData->hRGXPCI, SYS_RGX_REG_PCI_BASENUM);
	psDevice->sRegsCpuPBase.uiAddr	= 0;
	psDevice->ui32RegsSize		= 0;

ErrorSystemRegReleaseAddrRegion:
	OSPCIReleaseAddrRegion(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM, SYS_APOLLO_REG_SYS_OFFSET, SYS_APOLLO_REG_SYS_SIZE);
	psSysData->sSystemRegCpuPBase.uiAddr	= 0;
	psSysData->uiSystemRegSize		= 0;

ErrorPCIReleaseDevice:
	OSPCIReleaseDev(psSysData->hRGXPCI);
	psSysData->hRGXPCI = IMG_NULL;

	return eError;
}

static IMG_VOID PCIDeInitDev(SYS_DATA *psSysData)
{
	PVRSRV_DEVICE_CONFIG *psDevice = &gsSysConfig.pasDevices[0];

	DisableInterrupts(psSysData);

	free_irq(psDevice->ui32IRQ, psSysData);

	if (psSysData->pszSystemInfoString)
	{
		OSFreeMem(psSysData->pszSystemInfoString);
		psSysData->pszSystemInfoString = NULL;
	}

	DeInitMemory(&gsSysConfig, psSysData);

	OSUnMapPhysToLin(psSysData->pvSystemRegCpuVBase, psSysData->uiSystemRegSize, 0);
	psSysData->pvSystemRegCpuVBase = NULL;

	OSPCIReleaseAddrRange(psSysData->hRGXPCI, SYS_RGX_REG_PCI_BASENUM);
	psDevice->sRegsCpuPBase.uiAddr	= 0;
	psDevice->ui32RegsSize		= 0;

	OSPCIReleaseAddrRegion(psSysData->hRGXPCI, SYS_APOLLO_REG_PCI_BASENUM, SYS_APOLLO_REG_SYS_OFFSET, SYS_APOLLO_REG_SYS_SIZE);
	psSysData->sSystemRegCpuPBase.uiAddr	= 0;
	psSysData->uiSystemRegSize		= 0;

	OSPCIReleaseDev(psSysData->hRGXPCI);
	psSysData->hRGXPCI = IMG_NULL;
}

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
static IMG_VOID TCLocalCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
					  IMG_DEV_PHYADDR *psDevPAddr,
					  IMG_CPU_PHYADDR *psCpuPAddr)
{
	PVRSRV_SYSTEM_CONFIG *psSysConfig = (PVRSRV_SYSTEM_CONFIG *)hPrivData;

	psDevPAddr->uiAddr = psCpuPAddr->uiAddr - psSysConfig->pasPhysHeaps[0].sStartAddr.uiAddr;
}

static IMG_VOID TCLocalDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
					  IMG_CPU_PHYADDR *psCpuPAddr,
					  IMG_DEV_PHYADDR *psDevPAddr)
{
	PVRSRV_SYSTEM_CONFIG *psSysConfig = (PVRSRV_SYSTEM_CONFIG *)hPrivData;

	psCpuPAddr->uiAddr = psDevPAddr->uiAddr + psSysConfig->pasPhysHeaps[0].sStartAddr.uiAddr;
}
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)  || (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
static IMG_VOID TCSystemCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
					   IMG_DEV_PHYADDR *psDevPAddr,
					   IMG_CPU_PHYADDR *psCpuPAddr)
{
	PVR_UNREFERENCED_PARAMETER(hPrivData);

	psDevPAddr->uiAddr = psCpuPAddr->uiAddr;
}

static IMG_VOID TCSystemDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
					   IMG_CPU_PHYADDR *psCpuPAddr,
					   IMG_DEV_PHYADDR *psDevPAddr)
{
	PVR_UNREFERENCED_PARAMETER(hPrivData);

	psCpuPAddr->uiAddr = psDevPAddr->uiAddr;
}
#endif /* (TC_MEMORY_CONFIG == TC_MEMORY_HOST) || (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID) */

PVRSRV_ERROR SysCreateConfigData(PVRSRV_SYSTEM_CONFIG **ppsSysConfig)
{
	SYS_DATA *psSysData;
	PVRSRV_ERROR eError;

	psSysData = OSAllocMem(sizeof *psSysData);
	if (psSysData == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	OSMemSet(psSysData, 0, sizeof *psSysData);

	eError = PCIInitDev(psSysData);
	if (eError != PVRSRV_OK)
	{
		goto ErrorFreeSysData;
	}

	/* Save data for this device */
	gsSysConfig.pasDevices[0].hSysData = (IMG_HANDLE)psSysData;

	/* Save private data for the physical memory heaps */
	gsPhysHeapConfig[0].hPrivData = (IMG_HANDLE)&gsSysConfig;

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL) || (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
	gsPhysHeapConfig[1].hPrivData = (IMG_HANDLE)&gsSysConfig;
#endif

	*ppsSysConfig = &gsSysConfig;

	return PVRSRV_OK;

ErrorFreeSysData:
	OSFreeMem(psSysData);

	return eError;
}

IMG_VOID SysDestroyConfigData(PVRSRV_SYSTEM_CONFIG *psSysConfig)
{
	SYS_DATA *psSysData = (SYS_DATA *)psSysConfig->pasDevices[0].hSysData;

	PCIDeInitDev(psSysData);
	OSFreeMem(psSysData);
}

PVRSRV_ERROR SysDebugInfo(PVRSRV_SYSTEM_CONFIG *psSysConfig)
{
	PVR_UNREFERENCED_PARAMETER(psSysConfig);

	return PVRSRV_OK;
}
