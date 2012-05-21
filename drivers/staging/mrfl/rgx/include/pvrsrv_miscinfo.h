									    /*************************************************************************//*!
									       @File
									       @Title          PVRSRV Misc Info API
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__PVRSRV_MISCINFO_H__)
#define __PVRSRV_MISCINFO_H__

/*
 * Misc Info. present flags
 */
#define PVRSRV_MISC_INFO_MEMSTATS_PRESENT			(1U<<0)	/*!< Misc Info Request is for Memroy Stats */
#define PVRSRV_MISC_INFO_GLOBALEVENTOBJECT_PRESENT	(1U<<1)	/*!< Misc Info Request is for Global Event Object */
#define PVRSRV_MISC_INFO_DDKVERSION_PRESENT			(1U<<2)	/*!< Misc Info Request is for DDK Version */
#define PVRSRV_MISC_INFO_FREEMEM_PRESENT			(1U<<3)	/*!< Misc Info Request is for amount of Free Memory */
#define PVRSRV_MISC_INFO_RESET_PRESENT				(1U<<4)	/*!< Misc Info Request is for a Reset */

/*!
 ******************************************************************************
 * Structure to retrieve misc. information from services
 *****************************************************************************/
typedef struct _PVRSRV_MISC_INFO_ {
	IMG_UINT32 ui32StateRequest;	/*!< requested State Flags */
	IMG_UINT32 ui32StatePresent;	/*!< Present/Valid State Flags */

	/*! Memory Stats/DDK version string depending on ui32StateRequest flags */
	IMG_CHAR *pszMemoryStr;

	/*! Length of returned string in pszMemoryStr */
	IMG_UINT32 ui32MemoryStrLen;

	/*! global event object */
	IMG_HANDLE hGlobalEventObject;	//FIXME: should be private to services
	IMG_HANDLE hOSGlobalEvent;	/* DOXYGEN_FIXME */

	/* Note: add misc. items as required */

	/*! DDK Version in binary format: [0]=MAJ [1]=MIN [2]=Branch [3]=Build */
	IMG_UINT32 aui32DDKVersion[4];

} PVRSRV_MISC_INFO;

#endif				/* __PVRSRV_MISCINFO_H__ */
