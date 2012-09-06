									    /*************************************************************************//*!
									       @File
									       @Title          32 Bit Highlander kernel manager VxD Services
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if defined(_MSC_VER)
#pragma  warning(disable:4201)
#pragma  warning(disable:4214)
#pragma  warning(disable:4115)
#pragma  warning(disable:4514)
#endif				/* defined(_MSC_VER) */

#if defined(UNDER_CE)
#include <windows.h>
#include <ceddk.h>
#else
#if defined(WIN9X)
#define VXDBUILD
#define WANTVXDWRAPS
#include <basedef.h>
#include <vmm.h>
#include <vwin32.h>
#include <vmmreg.h>
#include <regstr.h>
#include <vxdwraps.h>
//#include "vxdutil.h"
#else
#if !defined(LINUX) && !defined(__SYMBIAN32__)
#include <ntddk.h>
#include <windef.h>
#endif
#endif
#endif

#include "img_types.h"
#include "pvr_debug.h"
#include "dbgdrvif.h"
#include "dbgdriv.h"
#include "hotkey.h"
#include "hostfunc.h"

#if defined(_MSC_VER)
#pragma  warning(default:4214)
#pragma  warning(default:4115)
#endif				/* defined(_MSC_VER) */

/*****************************************************************************
 Global vars
*****************************************************************************/
#ifdef WIN9X
#pragma VxD_LOCKED_DATA_SEG
#endif

IMG_UINT32 g_ui32HotKeyFrame = 0xFFFFFFFF;
IMG_BOOL g_bHotKeyPressed = IMG_FALSE;
IMG_BOOL g_bHotKeyRegistered = IMG_FALSE;

/* Hotkey stuff */
PRIVATEHOTKEYDATA g_PrivateHotKeyData;

/*****************************************************************************
 Code
*****************************************************************************/

/******************************************************************************
 * Function Name: ReadInHotKeys
 *
 * Inputs       : none
 * Outputs      : -
 * Returns      : nothing
 * Globals Used : -
 *
 * Description  : Gets Hot key entries from system.ini
 *****************************************************************************/
IMG_VOID ReadInHotKeys(IMG_VOID)
{
//      PSTR    pszRegPath = "SOFTWARE\\VideoLogic\\DEBUG\\Streams";
//      VMMHKEY hKey;

	g_PrivateHotKeyData.ui32ScanCode = 0x58;	/* F12  */
	g_PrivateHotKeyData.ui32ShiftState = 0x0;

	/*
	   Find buffer names etc..
	 */
#if 0
	if (_RegOpenKey(HKEY_LOCAL_MACHINE, pszRegPath, &hKey) == ERROR_SUCCESS) {
		/*
		   Read scan code and shift state.
		 */
		QueryReg(hKey, "ui32ScanCode",
			 &g_PrivateHotKeyData.ui32ScanCode);
		QueryReg(hKey, "ui32ShiftState",
			 &g_PrivateHotKeyData.ui32ShiftState);
	}
#else
	HostReadRegistryDWORDFromString("DEBUG\\Streams", "ui32ScanCode",
					&g_PrivateHotKeyData.ui32ScanCode);
	HostReadRegistryDWORDFromString("DEBUG\\Streams", "ui32ShiftState",
					&g_PrivateHotKeyData.ui32ShiftState);
#endif
}

/******************************************************************************
 * Function Name: RegisterKeyPressed
 *
 * Inputs       : IMG_UINT32 dwui32ScanCode, PHOTKEYINFO pInfo
 * Outputs      : -
 * Returns      : nothing
 * Globals Used : -
 *
 * Description  : Called when hotkey pressed.
 *****************************************************************************/
IMG_VOID RegisterKeyPressed(IMG_UINT32 dwui32ScanCode, PHOTKEYINFO pInfo)
{
	PDBG_STREAM psStream;

	PVR_UNREFERENCED_PARAMETER(pInfo);

	if (dwui32ScanCode == g_PrivateHotKeyData.ui32ScanCode) {
		PVR_DPF((PVR_DBG_MESSAGE, "PDUMP Hotkey pressed !\n"));

		psStream =
		    (PDBG_STREAM) g_PrivateHotKeyData.sHotKeyInfo.pvStream;

		if (!g_bHotKeyPressed) {
			/*
			   Capture the next frame.
			 */
			g_ui32HotKeyFrame = psStream->psCtrl->ui32Current + 2;

			/*
			   Do the flag.
			 */
			g_bHotKeyPressed = IMG_TRUE;
		}
	}
}

/******************************************************************************
 * Function Name: ActivateHotKeys
 *
 * Inputs       : -
 * Outputs      : -
 * Returns      : -
 * Globals Used : -
 *
 * Description  : Installs HotKey callbacks
 *****************************************************************************/
IMG_VOID ActivateHotKeys(PDBG_STREAM psStream)
{
	/*
	   Setup hotkeys.
	 */
	ReadInHotKeys();

	/*
	   Has it already been allocated.
	 */
	if (!g_PrivateHotKeyData.sHotKeyInfo.hHotKey) {
		if (g_PrivateHotKeyData.ui32ScanCode != 0) {
			PVR_DPF((PVR_DBG_MESSAGE,
				 "Activate HotKey for PDUMP.\n"));

			/*
			   Add in stream data.
			 */
			g_PrivateHotKeyData.sHotKeyInfo.pvStream = psStream;

			DefineHotKey(g_PrivateHotKeyData.ui32ScanCode,
				     g_PrivateHotKeyData.ui32ShiftState,
				     &g_PrivateHotKeyData.sHotKeyInfo);
		} else {
			g_PrivateHotKeyData.sHotKeyInfo.hHotKey = 0;
		}
	}
}

/******************************************************************************
 * Function Name: DeactivateHotKeys
 *
 * Inputs       : -
 * Outputs      : -
 * Returns      : -
 * Globals Used : -
 *
 * Description  : Removes HotKey callbacks
 *****************************************************************************/
IMG_VOID DeactivateHotKeys(IMG_VOID)
{
	if (g_PrivateHotKeyData.sHotKeyInfo.hHotKey != 0) {
		PVR_DPF((PVR_DBG_MESSAGE, "Deactivate HotKey.\n"));

		RemoveHotKey(g_PrivateHotKeyData.sHotKeyInfo.hHotKey);
		g_PrivateHotKeyData.sHotKeyInfo.hHotKey = 0;
	}
}

/*****************************************************************************
 End of file (HOTKEY.C)
*****************************************************************************/
