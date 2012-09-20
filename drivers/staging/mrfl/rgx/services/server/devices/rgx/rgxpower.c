									    /*************************************************************************//*!
									       @File
									       @Title          Device specific power routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>

#include "rgxdefs.h"
#include "rgxpower.h"
#include "rgx_fwif_km.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "pdump_km.h"
#include "rgx_cr_defs_km.h"
#include "pvrsrv.h"
#include "pvr_debug.h"
#include "osfunc.h"

/*!
*******************************************************************************

 @Function	RGXEnableClocks

 @Description Enable RGX Clocks

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXEnableClocks(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	PDUMPCOMMENT("RGX clock: use default (automatic clock gating)");
}

/*!
*******************************************************************************

 @Function	RGXInitSLC

 @Description Initialise RGX SLC

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXInitSLC(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	IMG_UINT32 ui32Reg;
	IMG_UINT32 ui32RegVal;

	/*
	 * SLC Bypass control
	 */
	ui32Reg = RGX_CR_SLC_CTRL_BYPASS;
	ui32RegVal = 0x0;

	if (PVRSRVSystemHasCacheSnooping()) {
		PDUMPCOMMENT("System has cache snooping");
	} else {
		PDUMPCOMMENT
		    ("System has not cache snooping: Bypass SLC when cc bit is set");
		ui32RegVal |= RGX_CR_SLC_CTRL_BYPASS_BYP_CC_EN;
	}

#if defined(RGX_FEATURE_TEXTURES_BYPASS_SLC)
	PDUMPCOMMENT("Bypass SLC for TPU");
	ui32RegVal |= RGX_CR_SLC_CTRL_BYPASS_REQ_TPU_EN;
#endif

	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32Reg, ui32RegVal);
	PDUMPREG32(RGX_PDUMPREG_NAME, ui32Reg, ui32RegVal,
		   PDUMP_FLAGS_CONTINUOUS);

	/*
	 * SLC Bypass control
	 */
	ui32Reg = RGX_CR_SLC_CTRL_MISC;
	ui32RegVal = RGX_CR_SLC_CTRL_MISC_ADDR_DECODE_MODE_PVR_HASH1;
#if defined(RGX_FEATURE_BYPASS_SLC_COMBINER)
	ui32RegVal |= RGX_CR_SLC_CTRL_MISC_BYPASS_BURST_COMBINER_EN;
#endif
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32Reg, ui32RegVal);
	PDUMPREG32(RGX_PDUMPREG_NAME, ui32Reg, ui32RegVal,
		   PDUMP_FLAGS_CONTINUOUS);
}

/*!
*******************************************************************************

 @Function	RGXInitBIF

 @Description Initialise RGX BIF

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXInitBIF(PVRSRV_RGXDEV_INFO * psDevInfo)
{
	PVRSRV_ERROR eError;
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
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, psDevInfo->ui32KernelCatBase,
		       sPCAddr.uiAddr);

	PDUMPCOMMENT("RGX firmware MMU Page Catalogue");
	MMU_PDumpWritePageCatBase(psDevInfo->psKernelMMUCtx,
				  RGX_PDUMPREG_NAME,
				  psDevInfo->ui32KernelCatBase);
}

#if defined(RGX_FEATURE_AXI_ACELITE)
/*!
*******************************************************************************

 @Function	RGXAXIACELiteInit

 @Description Initialise AXI-ACE Lite interface

 @Input psDevInfo - device info structure

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID RGXAXIACELiteInit(PVRSRV_RGXDEV_INFO * psDevInfo)
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

	/* Setup AXI-ACE config. Set everthing to outer cache */
	ui64RegVal =
	    (3U <<
	     RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_NON_SNOOPING_SHIFT) |
	    (3U <<
	     RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_NON_SNOOPING_SHIFT) |
	    (2U <<
	     RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_CACHE_MAINTENANCE_SHIFT)
	    | (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWDOMAIN_COHERENT_SHIFT)
	    | (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARDOMAIN_COHERENT_SHIFT)
	    | (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_AWCACHE_COHERENT_SHIFT) |
	    (2U << RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_COHERENT_SHIFT) |
	    (2U <<
	     RGX_CR_AXI_ACE_LITE_CONFIGURATION_ARCACHE_CACHE_MAINTENANCE_SHIFT);

	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, ui32RegAddr, ui64RegVal);
	PDUMPCOMMENT("Init AXI-ACE interface");
	PDUMPREG64(RGX_PDUMPREG_NAME, ui32RegAddr, ui64RegVal,
		   PDUMP_FLAGS_CONTINUOUS);
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
static PVRSRV_ERROR RGXStart(PVRSRV_RGXDEV_INFO * psDevInfo,
			     PVRSRV_DEVICE_CONFIG * psDevConfig)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Set RGX in soft-reset */
	PDUMPCOMMENT("RGXStart: soft reset everything");
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET,
		       RGX_CR_SOFT_RESET_MASKFULL);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET,
		   RGX_CR_SOFT_RESET_MASKFULL, PDUMP_FLAGS_CONTINUOUS);

	(void)OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET,
		       PDUMP_FLAGS_CONTINUOUS);

	/* Take everything out of reset but META */
	PDUMPCOMMENT("RGXStart: Take everything out of reset but META");
	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET,
		       RGX_CR_SOFT_RESET_GARTEN_EN);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET,
		   RGX_CR_SOFT_RESET_GARTEN_EN, PDUMP_FLAGS_CONTINUOUS);

	(void)OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET,
		       PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Enable clocks.
	 */
	RGXEnableClocks(psDevInfo);

	/*
	 * Initialise SLC.
	 */
	RGXInitSLC(psDevInfo);

#if !defined(SUPPORT_META_SLAVE_BOOT)
	/* Configure META to Master boot */
	PDUMPCOMMENT("RGXStart: META Master boot");
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_BOOT,
		       RGX_CR_META_BOOT_MODE_EN);
	PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_META_BOOT,
		   RGX_CR_META_BOOT_MODE_EN, PDUMP_FLAGS_CONTINUOUS);
#endif

#if defined(RGX_FEATURE_AXI_ACELITE)
	/*
	   We must init the AXI-ACE interface before 1st BIF transation
	 */
	RGXAXIACELiteInit(psDevInfo);
#endif

	/*
	 * Initialise BIF.
	 */
	RGXInitBIF(psDevInfo);

	PDUMPCOMMENT("RGXStart: Take META out of reset");
	/* need to wait for at least 16 cycles before taking meta out of reset ... */
	PVRSRVSystemWaitCycles(psDevConfig, 32);

	OSWriteHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET, 0x0);
	PDUMPREG64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET, 0x0,
		   PDUMP_FLAGS_CONTINUOUS);

	(void)OSReadHWReg64(psDevInfo->pvRegsBaseKM, RGX_CR_SOFT_RESET);
	PDUMPREGREAD64(RGX_PDUMPREG_NAME, RGX_CR_SOFT_RESET,
		       PDUMP_FLAGS_CONTINUOUS);

	/* ... and afterwards */
	PVRSRVSystemWaitCycles(psDevConfig, 32);

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
static PVRSRV_ERROR RGXStop(PVRSRV_RGXDEV_INFO * psDevInfo)
{
/*	PVRSRV_ERROR		eError; */

/*
	eError = RGXRunScript(psDevInfo, psDevInfo->sScripts.asDeinitCommands, RGX_MAX_DEINIT_COMMANDS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXStop: RGXRunScript failed (%d)", eError));
		return eError;
	}
*/

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	RGXStartTimer

 @Description

 Start the host timer

 @Input	   psDevInfo : RGX Device Info

 @Return   IMG_VOID :

******************************************************************************/
static IMG_VOID RGXStartTimer(PVRSRV_RGXDEV_INFO * psDevInfo)
{
#if defined(FIXME)
	PVRSRV_ERROR eError;

	eError = OSEnableTimer(psDevInfo->hTimer);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXStartTimer : Failed to enable host timer"));
	}
#endif
}

/*
	RGXPrePowerState
*/
PVRSRV_ERROR RGXPrePowerState(IMG_HANDLE hDevHandle,
			      PVRSRV_DEV_POWER_STATE eNewPowerState,
			      PVRSRV_DEV_POWER_STATE eCurrentPowerState)
{
	/* TODO -- write some code here! */

	if ((eNewPowerState != eCurrentPowerState) &&
	    (eNewPowerState != PVRSRV_DEV_POWER_STATE_ON)) {
		PVRSRV_ERROR eError;
		PVRSRV_DEVICE_NODE *psDeviceNode = hDevHandle;
		PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
#if 0
		IMG_UINT32 ui32PowerCmd, ui32CompleteStatus;
		RGXMKIF_COMMAND sCommand = { 0 };

		/* Disable timer callback for HW recovery */
		eError = OSDisableTimer(psDevInfo->hTimer);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXPrePowerState: Failed to disable timer"));
			return eError;
		}

		if (eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
			/* Request the firmware to idle RGX and save its state. */
			ui32PowerCmd = PVRSRV_POWERCMD_POWEROFF;
			ui32CompleteStatus =
			    PVRSRV_USSE_EDM_POWMAN_POWEROFF_COMPLETE;
			PDUMPCOMMENT("RGX power off request");
		} else {
			/* Request the firmware to idle RGX. */
			ui32PowerCmd = PVRSRV_POWERCMD_IDLE;
			ui32CompleteStatus =
			    PVRSRV_USSE_EDM_POWMAN_IDLE_COMPLETE;
			PDUMPCOMMENT("RGX idle request");
		}

		sCommand.ui32Data[1] = ui32PowerCmd;

		eError =
		    RGXScheduleCCBCommand(psDevInfo, RGXMKIF_CMD_POWER,
					  &sCommand, KERNEL_ID, 0);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXPrePowerState: Failed to submit power down command"));
			return eError;
		}

		/* Wait for the firmware to complete processing. */
#if !defined(NO_HARDWARE)
		if (PVRSRVPollForValueKM
		    (&psDevInfo->psRGXHostCtl->ui32PowerStatus,
		     ui32CompleteStatus, ui32CompleteStatus) != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXPrePowerState: Wait for RGX firmware power transition failed."));
			PVR_DBG_BREAK;
		}
#endif				/* NO_HARDWARE */

#if defined(PDUMP)
		PDUMPCOMMENT
		    ("TA/3D CCB Control - Wait for power event on firmware.");
		PDUMPMEMPOL(psDevInfo->psKernelRGXHostCtlMemInfo,
			    offsetof(RGXMKIF_HOST_CTL, ui32PowerStatus),
			    ui32CompleteStatus, ui32CompleteStatus,
			    PDUMP_POLL_OPERATOR_EQUAL, 0,
			    MAKEUNIQUETAG(psDevInfo->
					  psKernelRGXHostCtlMemInfo));
#endif				/* PDUMP */

		/* Wait for RGX clock gating. */
		RGXPollForClockGating(psDevInfo,
				      psDevInfo->ui32ClkGateStatusReg,
				      psDevInfo->ui32ClkGateStatusMask,
				      "Wait for RGX clock gating");
#else
		if (eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
			/* Finally, de-initialise some registers. */
			eError = RGXStop(psDevInfo);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXPrePowerState: RGXStop failed: %u",
					 eError));
				return eError;
			}
		}
		PVR_UNREFERENCED_PARAMETER(hDevHandle);
#endif
	}

	return PVRSRV_OK;
}

/*
	RGXPostPowerState
*/
PVRSRV_ERROR RGXPostPowerState(IMG_HANDLE hDevHandle,
			       PVRSRV_DEV_POWER_STATE eNewPowerState,
			       PVRSRV_DEV_POWER_STATE eCurrentPowerState)
{
	/* TODO -- write some code here! */

	if ((eNewPowerState != eCurrentPowerState) &&
	    (eCurrentPowerState != PVRSRV_DEV_POWER_STATE_ON)) {
		PVRSRV_ERROR eError;
		PVRSRV_DEVICE_NODE *psDeviceNode = hDevHandle;
		PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
		PVRSRV_DEVICE_CONFIG *psDevConfig = psDeviceNode->psDevConfig;
		//RGXMKIF_HOST_CTL *psRGXHostCtl = psDevInfo->psRGXHostCtl;

#if 0
		/* Reset the power manager flags. */
		psRGXHostCtl->ui32PowerStatus = 0;
#if defined(PDUMP)
		PDUMPCOMMENT("Host Control - Reset power status");
		PDUMPMEM(IMG_NULL, psDevInfo->psKernelRGXHostCtlMemInfo,
			 offsetof(RGXMKIF_HOST_CTL, ui32PowerStatus),
			 sizeof(IMG_UINT32), PDUMP_FLAGS_CONTINUOUS,
			 MAKEUNIQUETAG(psDevInfo->psKernelRGXHostCtlMemInfo));
#endif				/* PDUMP */
#endif

		if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
			/*
			   Coming up from off, re-initialise RGX.
			 */

#if 0
			/*
			   Re-generate the timing data required by RGX.
			 */
			eError = RGXUpdateTimingInfo(psDeviceNode);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXPostPowerState: RGXUpdateTimingInfo failed"));
				return eError;
			}
#endif

			/*
			   Run the RGX init script.
			 */
			eError = RGXStart(psDevInfo, psDevConfig);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXPostPowerState: RGXStart failed"));
				return eError;
			}
		} else {
#if 0
			/*
			   Coming up from idle, restart the firmware.
			 */
			RGXMKIF_COMMAND sCommand = { 0 };

			sCommand.ui32Data[1] = PVRSRV_POWERCMD_RESUME;
			eError =
			    RGXScheduleCCBCommand(psDevInfo, RGXMKIF_CMD_POWER,
						  &sCommand, ISR_ID, 0);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXPostPowerState failed to schedule CCB command: %u",
					 eError));
				return eError;
			}
#endif
		}

		RGXStartTimer(psDevInfo);
	}

	return PVRSRV_OK;
}

/*
	RGXPreClockSpeedChange
*/
PVRSRV_ERROR RGXPreClockSpeedChange(IMG_HANDLE hDevHandle,
				    IMG_BOOL bIdleDevice,
				    PVRSRV_DEV_POWER_STATE eCurrentPowerState)
{
	/* TODO: write some code here */

	//      PVRSRV_ERROR            eError;
	PVRSRV_DEVICE_NODE *psDeviceNode = hDevHandle;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	PVR_UNREFERENCED_PARAMETER(psDevInfo);

	if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_ON) {
#if 0
		if (bIdleDevice) {
			/*
			 * Idle RGX.
			 */
			PDUMPSUSPEND();

			eError =
			    RGXPrePowerState(hDevHandle,
					     PVRSRV_DEV_POWER_STATE_IDLE,
					     PVRSRV_DEV_POWER_STATE_ON);

			if (eError != PVRSRV_OK) {
				PDUMPRESUME();
				return eError;
			}
		}
#else
		PVR_UNREFERENCED_PARAMETER(bIdleDevice);
#endif
	}

	PVR_DPF((PVR_DBG_MESSAGE,
		 "RGXPreClockSpeedChange: RGX clock speed was %uHz",
		 psDevInfo->ui32CoreClockSpeed));

	return PVRSRV_OK;
}

/*
	RGXPostClockSpeedChange
*/
PVRSRV_ERROR RGXPostClockSpeedChange(IMG_HANDLE hDevHandle,
				     IMG_BOOL bIdleDevice,
				     PVRSRV_DEV_POWER_STATE eCurrentPowerState)
{
	/* TODO: write some code here */

	PVRSRV_DEVICE_NODE *psDeviceNode = hDevHandle;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	IMG_UINT32 ui32OldClockSpeed = psDevInfo->ui32CoreClockSpeed;

	PVR_UNREFERENCED_PARAMETER(ui32OldClockSpeed);

	if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_ON) {
		PVRSRV_ERROR eError;

#if 0
		/*
		   Re-generate the timing data required by RGX.
		 */
		eError = RGXUpdateTimingInfo(psDeviceNode);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "RGXPostPowerState: RGXUpdateTimingInfo failed"));
			return eError;
		}
#endif

		if (bIdleDevice) {
			/*
			 * Resume RGX.
			 */
			eError =
			    RGXPostPowerState(hDevHandle,
					      PVRSRV_DEV_POWER_STATE_ON,
					      PVRSRV_DEV_POWER_STATE_IDLE);

			PDUMPRESUME();

			if (eError != PVRSRV_OK) {
				return eError;
			}
		} else {
			RGXStartTimer(psDevInfo);
		}
	}

	PVR_DPF((PVR_DBG_MESSAGE,
		 "RGXPostClockSpeedChange: RGX clock speed changed from %uHz to %uHz",
		 ui32OldClockSpeed, psDevInfo->ui32CoreClockSpeed));

	return PVRSRV_OK;
}

/******************************************************************************
 End of file (rgxpower.c)
******************************************************************************/
