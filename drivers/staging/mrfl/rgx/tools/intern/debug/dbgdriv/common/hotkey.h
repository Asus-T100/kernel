									    /*************************************************************************//*!
									       @File
									       @Title          VxD utilities header file.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Hotkey stuff
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _HOTKEY_
#define _HOTKEY_

typedef struct _hotkeyinfo {
	IMG_UINT8 ui8ScanCode;
	IMG_UINT8 ui8Type;
	IMG_UINT8 ui8Flag;
	IMG_UINT8 ui8Filler1;
	IMG_UINT32 ui32ShiftState;
	IMG_UINT32 ui32HotKeyProc;
	IMG_VOID *pvStream;
	IMG_UINT32 hHotKey;	/* handle.      */
} HOTKEYINFO, *PHOTKEYINFO;

typedef struct _privatehotkeydata {
	IMG_UINT32 ui32ScanCode;
	IMG_UINT32 ui32ShiftState;
	HOTKEYINFO sHotKeyInfo;
} PRIVATEHOTKEYDATA, *PPRIVATEHOTKEYDATA;

/*****************************************************************************
 Hotkey stuff
*****************************************************************************/
IMG_VOID ReadInHotKeys(IMG_VOID);
IMG_VOID ActivateHotKeys(PDBG_STREAM psStream);
IMG_VOID DeactivateHotKeys(IMG_VOID);

IMG_VOID RemoveHotKey(IMG_UINT32 hHotKey);
IMG_VOID DefineHotKey(IMG_UINT32 ui32ScanCode, IMG_UINT32 ui32ShiftState,
		      PHOTKEYINFO psInfo);
IMG_VOID RegisterKeyPressed(IMG_UINT32 ui32ScanCode, PHOTKEYINFO psInfo);

#endif

/*****************************************************************************
 End of file (HOTKEY.H)
*****************************************************************************/
