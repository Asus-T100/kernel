/*************************************************************************/ /*!
@File
@Title          Device specific power routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device specific functions
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

#include "rgxpower.h"
#include "rgx_fwif_km.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "pdump_km.h"
#include "rgxdefs_km.h"
#include "pvrsrv.h"
#include "pvr_debug.h"
#include "osfunc.h"
#include "rgxdebug.h"


/*!
*******************************************************************************

 @Function	RGXEnableClocks

 @Description Enable RGX Clocks

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXEnableClocks(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGX clock: use default (automatic clock gating)");
}


/*!
*******************************************************************************

 @Function	RGXInitSLC

 @Description Initialise RGX SLC

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXInitSLC(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	IMG_UINT32	ui32Reg;
	IMG_UINT32	ui32RegVal;

	/*
	 * SLC Bypass control
	 */
	ui32Reg = RGX_CR_SLC_CTRL_BYPASS;
	ui32RegVal = 0x0;

	if (PVRSRVSystemHasCacheSnooping())
	{
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "System has cache snooping");
	}
	else
	{
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "System has not cache snooping: Bypass SLC when cc bit is set");
		ui32RegVal |= RGX_CR_SLC_CTRL_BYPASS_BYP_CC_EN; 
	}

	/* Bypass SLC for textures if the SLC size is less than 128kB */
#if (RGX_FEATURE_SLC_SIZE_IN_BYTES < (128*1024))
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "Bypass SLC for TPU");
	ui32RegVal |= RGX_CR_SLC_CTRL_BYPASS_REQ_TPU_EN;
#endif

	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32Reg, ui32RegVal);
	PDUMPREG32(RGX_PDUMPREG_NAME, ui32Reg, ui32RegVal, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	/*
	 * SLC Bypass control
	 */
	ui32Reg = RGX_CR_SLC_CTRL_MISC;
	ui32RegVal = RGX_CR_SLC_CTRL_MISC_ADDR_DECODE_MODE_PVR_HASH1;

	/* Bypass burst combiner if SLC line size is smaller than 1024 bits */
#if (RGX_FEATURE_SLC_CACHE_LINE_SIZE_BITS < 1024)
	ui32RegVal |= RGX_CR_SLC_CTRL_MISC_BYPASS_BURST_COMBINER_EN;
#endif
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32Reg, ui32RegVal);
	PDUMPREG32(RGX_PDUMPREG_NAME, ui32Reg, ui32RegVal, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);
}


/*!
*******************************************************************************

 @Function	RGXInitBIF

 @Description Initialise RGX BIF

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXInitBIF(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	PVRSRV_ERROR	eError;
	IMG_DEV_PHYADDR sPCAddr;

	/*
		Acquire the address of the Kernel Page Catalogue.
	*/
	eError = MMU_AcquireBaseAddr(psDevInfo->psKernelMMUCtx, &sPCAddr);
	PVR_ASSERT(eError == PVRSRV_OK);

	/*
		Write the kernel catalogue base.
	*/
	/* FIXME: There should be a RGX_CR macro for getting the address of the cat base */
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, psDevInfo->ui32KernelCatBase, sPCAddr.uiAddr);

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGX firmware MMU Page Catalogue");
	MMU_PDumpWritePageCatBase(psDevInfo->psKernelMMUCtx,
							  RGX_PDUMPREG_NAME,
							  psDevInfo->ui32KernelCatBase,
							  PDUMP_FLAGS_POWERTRANS);
}

#if defined(RGX_FEATURE_AXI_ACELITE)
/*!
*******************************************************************************

 @Function	RGXAXIACELiteInit

 @Description Initialise AXI-ACE Lite interface

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXAXIACELiteInit(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	IMG_UINT32 ui32RegAddr;
	IMG_UINT64 ui64RegVal;

/*
	FIXME: These need to be unrestricted
*/

/*
	Register RGX_CR_AXI_ACE_LITE_CONFIGURATION
*/
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION                 (0x38C0U)


#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_CACHE_MAINTENANCE_SHIFT (30U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_COHERENT_SHIFT (26U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWCACHE_COHERENT_SHIFT (22U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_CACHE_MAINTENANCE_SHIFT (16U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_COHERENT_SHIFT (14U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_COHERENT_SHIFT (12U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_NON_SNOOPING_SHIFT (10U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_NON_SNOOPING_SHIFT (8U)

	ui32RegAddr = RGX_CR_AXI_ACE_LITE_CONFIGURATION;

	/* Setup AXI-ACE config. Set everything to outer cache */
	ui64RegVal =   (3U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_NON_SNOOPING_SHIFT) |
				   (3U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_NON_SNOOPING_SHIFT) |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_CACHE_MAINTENANCE_SHIFT)  |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_COHERENT_SHIFT) |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_COHERENT_SHIFT) |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWCACHE_COHERENT_SHIFT) |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_COHERENT_SHIFT) |
				   (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_CACHE_MAINTENANCE_SHIFT);

	OSWriteHWReg64(psDevInfo->pvRegsBaseKM,
				   ui32RegAddr,
				   ui64RegVal);
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "Init AXI-ACE interface");
	PDUMPREG64(RGX_PDUMPREG_NAME, ui32RegAddr, ui64RegVal, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);
}
#endif

/*!
*******************************************************************************

 @Function	RGXStart

 @Description

 (client invoked) chip-reset and initialisation

 @Input psDevInfo - device info structure

 @Return   PVRSRV_ERROR

******************************************************************************/
static PVRSRV_ERROR RGXStart(PVRSRV_RGXDEV_INFO	*psDevInfo, PVRSRV_DEVICE_CONFIG *psDevConfig)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;

	/* Set RGX in soft-reset */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: soft reset everything");
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_MASKFULL);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_MASKFULL, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	/* Take Rascal and Dust out of reset */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: Rascal and Dust out of reset");
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_MASKFULL ^ RGX_CR_SOFT_RESET_RASCALDUSTS_EN);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_MASKFULL ^ RGX_CR_SOFT_RESET_RASCALDUSTS_EN, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	/* Read soft-reset to fence previos write in order to clear the SOCIF pipeline */
	(void) OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	/* Take everything out of reset but META */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: Take everything out of reset but META");
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_GARTEN_EN);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_GARTEN_EN, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	/*
	 * Enable clocks.
	 */
	RGXEnableClocks(psDevInfo);

	/*
	 * Initialise SLC.
	 */
	RGXInitSLC(psDevInfo);

	/*
	 * Trusted META boot
	 */
#if defined(SUPPORT_TRUSTED_ZONE)
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: Support Trusted Zone");
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_BIF_TRUST, RGX_CR_BIF_TRUST_ENABLE_EN);
	PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_BIF_TRUST, RGX_CR_BIF_TRUST_ENABLE_EN, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);
#endif

#if !defined(SUPPORT_META_SLAVE_BOOT)
	/* Configure META to Master boot */
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: META Master boot");
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_BOOT, RGX_CR_META_BOOT_MODE_EN);
	PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_META_BOOT, RGX_CR_META_BOOT_MODE_EN, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);
#endif

#if defined(RGX_FEATURE_AXI_ACELITE)
	/*
		We must init the AXI-ACE interface before 1st BIF transaction
	*/
	RGXAXIACELiteInit(psDevInfo);
#endif

	/*
	 * Initialise BIF.
	 */
	RGXInitBIF(psDevInfo);

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXStart: Take META out of reset");
	/* need to wait for at least 16 cycles before taking meta out of reset ... */
	PVRSRVSystemWaitCycles(psDevConfig, 32);
	PDUMPIDLWITHFLAGS(32, PDUMP_FLAGS_POWERTRANS);
	
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET, 0x0);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, 0x0, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);

	(void) OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWERTRANS);
	
	/* ... and afterwards */
	PVRSRVSystemWaitCycles(psDevConfig, 32);
	PDUMPIDLWITHFLAGS(32, PDUMP_FLAGS_POWERTRANS);

	/*
	 * Start the firmware.
	 */
	RGXStartFirmware(psDevInfo);
	
	OSMemoryBarrier();

	return eError;
}


/*!
*******************************************************************************

 @Function	RGXStop

 @Description Stop RGX in preparation for power down

 @Input psDevInfo - RGX device info

 @Return   PVRSRV_ERROR

******************************************************************************/
static PVRSRV_ERROR RGXStop(PVRSRV_RGXDEV_INFO	*psDevInfo)

{
	PVRSRV_ERROR		eError; 


	eError = RGXRunScript(psDevInfo, psDevInfo->sScripts.asDeinitCommands, RGX_MAX_DEINIT_COMMANDS, PDUMP_FLAGS_LASTFRAME | PDUMP_FLAGS_POWERTRANS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXStop: RGXRunScript failed (%d)", eError));
		return eError;
	}


	return PVRSRV_OK;
}

/*
	RGXPrePowerState
*/
PVRSRV_ERROR RGXPrePowerState (IMG_HANDLE				hDevHandle,
							   PVRSRV_DEV_POWER_STATE	eNewPowerState,
							   PVRSRV_DEV_POWER_STATE	eCurrentPowerState,
							   IMG_BOOL					bForced)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if ((eNewPowerState != eCurrentPowerState) &&
		(eNewPowerState != PVRSRV_DEV_POWER_STATE_ON))
	{
		PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;
		PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
		RGXFWIF_KCCB_CMD	sPowCmd;
		RGXFWIF_TRACEBUF	*psFWTraceBuf = psDevInfo->psRGXFWIfTraceBuf;
		IMG_UINT32			ui32DM;
		IMG_BOOL			bRetry = IMG_FALSE;

		/* Send the Power off request to the FW */
		sPowCmd.eCmdType = RGXFWIF_KCCB_CMD_POW;
		sPowCmd.uCmdData.sPowData.ePowType = RGXFWIF_POW_OFF_REQ;
		sPowCmd.uCmdData.sPowData.uPoweReqData.bForced = bForced;

		do
		{
			SyncPrimSet(psDevInfo->psPowSyncPrim, 0);

			/* Send one pow command to each DM to make sure we flush all the DMs pipelines */
			for (ui32DM = RGXFWIF_DM_TA; ui32DM <= RGXFWIF_DM_GP; ui32DM++)
			{
				eError = RGXSendCommandRaw(psDevInfo,
											ui32DM,
											&sPowCmd,
											sizeof(sPowCmd),
											PDUMP_FLAGS_POWERTRANS);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR,"RGXPrePowerState: Failed to send Power off request for DM%d", ui32DM));
					return eError;
				}
			}

			/* Wait for the firmware to complete processing. */
			for (;;)
			{

				eError = PVRSRVWaitForValueKM(psDevInfo->psPowSyncPrim->pui32LinAddr, 0x1, 0x1);

				/* The Firmware has answered */
				if (eError == PVRSRV_OK)
				{
					break;
				}
				else if (eError == PVRSRV_ERROR_TIMEOUT)
				{
					/* The firmware hasn't answered yet */
					PVR_DPF((PVR_DBG_ERROR,"RGXPrePowerState: Timeout waiting for powoff ack from the FW"));
					RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
					break;
				}
				else
				{
					PVR_DPF((PVR_DBG_ERROR,"RGXPrePowerState: Error waiting for powoff ack from the FW (%s)", PVRSRVGetErrorStringKM(eError)));
					return eError;
				}
			}

			/* Check the Power state after the answer */
			if (psFWTraceBuf->ePowState != RGXFWIF_APM_OFF)
			{
				if (bForced)
				{
					PVR_DPF((PVR_DBG_ERROR,"RGXPrePowerState: Forced powoff request not honoured by the FW. Retry."));
					RGXDumpDebugInfo(psDevInfo, IMG_TRUE);
					bRetry = IMG_TRUE;
				}
				else
				{
					eError = PVRSRV_ERROR_DEVICE_POWER_CHANGE_FAILURE;
				}
			}
		} while (bRetry);

		/* Check the Power state after the answer */
		if (psFWTraceBuf->ePowState == RGXFWIF_APM_OFF)
		{
			/* Finally, de-initialise some registers. */
			eError = RGXStop(psDevInfo);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"RGXPrePowerState: RGXStop failed (%s)", PVRSRVGetErrorStringKM(eError)));
			}
		}
	}

	return eError;
}


/*
	RGXPostPowerState
*/
PVRSRV_ERROR RGXPostPowerState (IMG_HANDLE				hDevHandle,
								PVRSRV_DEV_POWER_STATE	eNewPowerState,
								PVRSRV_DEV_POWER_STATE	eCurrentPowerState,
								IMG_BOOL				bForced)
{
	if ((eNewPowerState != eCurrentPowerState) &&
		(eCurrentPowerState != PVRSRV_DEV_POWER_STATE_ON))
	{
		PVRSRV_ERROR		 eError;
		PVRSRV_DEVICE_NODE	 *psDeviceNode = hDevHandle;
		PVRSRV_RGXDEV_INFO	 *psDevInfo = psDeviceNode->pvDevice;
		PVRSRV_DEVICE_CONFIG *psDevConfig = psDeviceNode->psDevConfig;

		if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_OFF)
		{
			/*
				Coming up from off, re-initialise RGX.
			*/


			/*
				Run the RGX init script.
			*/
			eError = RGXStart(psDevInfo, psDevConfig);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"RGXPostPowerState: RGXStart failed"));
				return eError;
			}
		}
	}

	PDUMPCOMMENT("RGXPostPowerState: Current state: %d, New state: %d", eCurrentPowerState, eNewPowerState);

	return PVRSRV_OK;
}


/*
	RGXPreClockSpeedChange
*/
PVRSRV_ERROR RGXPreClockSpeedChange (IMG_HANDLE				hDevHandle,
									 IMG_BOOL				bIdleDevice,
									 PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{

	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;

    PVR_UNREFERENCED_PARAMETER(psDevInfo);
    PVR_UNREFERENCED_PARAMETER(bIdleDevice);
    PVR_UNREFERENCED_PARAMETER(eCurrentPowerState);

	PVR_DPF((PVR_DBG_MESSAGE,"RGXPreClockSpeedChange: RGX clock speed was %uHz",
			psDevInfo->ui32CoreClockSpeed));

	return PVRSRV_OK;
}


/*
	RGXPostClockSpeedChange
*/
PVRSRV_ERROR RGXPostClockSpeedChange (IMG_HANDLE				hDevHandle,
									  IMG_BOOL					bIdleDevice,
									  PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	IMG_UINT32			ui32OldClockSpeed = psDevInfo->ui32CoreClockSpeed;
	PVRSRV_ERROR		eError = PVRSRV_OK;

	PVR_UNREFERENCED_PARAMETER(ui32OldClockSpeed);

	if (bIdleDevice)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR, "RGXPostClockSpeedChange: Scheduling KCCB command failed. Error:%u", eError));
		return eError;
	}

	PVR_DPF((PVR_DBG_MESSAGE,"RGXPostClockSpeedChange: RGX clock speed changed from %uHz to %uHz",
			ui32OldClockSpeed, psDevInfo->ui32CoreClockSpeed));

	return eError;
}

/*
	RGXActivePowerRequest
*/
PVRSRV_ERROR RGXActivePowerRequest(IMG_HANDLE hDevHandle)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;

	PDUMPPOWCMDSTART();

	OSAcquireBridgeLock();
	OSSetKeepPVRLock();

	/* Powerlock to avoid further requests from racing with the FW hand-shake from now on
	   (previous kicks to this point are detected by the FW) */
	eError = PVRSRVPowerLock();
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXActivePowerRequest: Failed to acquire PowerLock (device index: %d, error: %s)", 
					psDeviceNode->sDevId.ui32DeviceIndex,
					PVRSRVGetErrorStringKM(eError)));
		goto _RGXActivePowerRequest_failed;
	}

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_POWERTRANS, "RGXActivePowerRequest: FW set APM, handshake to power off");

	eError = 
	  PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
								  PVRSRV_DEV_POWER_STATE_OFF,
								  KERNEL_ID,
								  IMG_FALSE, /* retain mutex */
								  IMG_FALSE); /* forced */

	if (eError == PVRSRV_ERROR_DEVICE_POWER_CHANGE_FAILURE)
	{
		PVR_DPF((PVR_DBG_MESSAGE,"  RGXActivePowerRequest end: Active Power request (device index: %d) denied.", 
					psDeviceNode->sDevId.ui32DeviceIndex));
	} 
	else if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"  RGXActivePowerRequest end: Failed PVRSRVSetDevicePowerStateKM call (device index: %d) with %s", 
					psDeviceNode->sDevId.ui32DeviceIndex,
					PVRSRVGetErrorStringKM(eError)));
	}
	
	PVRSRVPowerUnlock();

_RGXActivePowerRequest_failed:
	OSReleaseBridgeLock();
	
	PDUMPPOWCMDEND();

	return eError;

}
/******************************************************************************
 End of file (rgxpower.c)
******************************************************************************/
