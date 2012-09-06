									    /*************************************************************************//*!
									       @File
									       @Title          PVR Debug Declarations
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Provides debug functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __PVR_DEBUG_H__
#define __PVR_DEBUG_H__

#include "img_types.h"

#if defined (WIN32)
extern IMG_VOID __debugbreak(IMG_VOID);
#endif

#if defined (__cplusplus)
extern "C" {
#endif

#define PVR_MAX_DEBUG_MESSAGE_LEN	(512)	/*!< Max length of a Debug Message */

/* These are privately used by pvr_debug, use the PVR_DBG_ defines instead */
#define DBGPRIV_FATAL			0x01UL	/*!< Debug-Fatal. Privately used by pvr_debug. */
#define DBGPRIV_ERROR			0x02UL	/*!< Debug-Error. Privately used by pvr_debug. */
#define DBGPRIV_WARNING			0x04UL	/*!< Debug-Warning. Privately used by pvr_debug. */
#define DBGPRIV_MESSAGE			0x08UL	/*!< Debug-Message. Privately used by pvr_debug. */
#define DBGPRIV_VERBOSE			0x10UL	/*!< Debug-Verbose. Privately used by pvr_debug. */
#define DBGPRIV_CALLTRACE		0x20UL	/*!< Debug-CallTrace. Privately used by pvr_debug. */
#define DBGPRIV_ALLOC			0x40UL	/*!< Debug-Alloc. Privately used by pvr_debug. */
#define DBGPRIV_DBGDRV_MESSAGE	0x80UL	/*!< Debug-DbgDrivMessage. Privately used by pvr_debug. */

#if !defined(PVRSRV_NEED_PVR_ASSERT) && defined(DEBUG)
#define PVRSRV_NEED_PVR_ASSERT
#endif

#if defined(PVRSRV_NEED_PVR_ASSERT) && !defined(PVRSRV_NEED_PVR_DPF)
#define PVRSRV_NEED_PVR_DPF
#endif

#if !defined(PVRSRV_NEED_PVR_TRACE) && (defined(DEBUG) || defined(TIMING))
#define PVRSRV_NEED_PVR_TRACE
#endif

/* PVR_ASSERT() and PVR_DBG_BREAK handling */

#if defined(PVRSRV_NEED_PVR_ASSERT)

#if defined(_WIN32)
#define PVR_ASSERT(Con) if (!(Con))						\
	{								\
		__debugbreak();						\
	}

#else

#if defined(LINUX) && defined(__KERNEL__)
#include <linux/kernel.h>
/* In Linux kernel mode, use BUG() directly. This produces the correct
   filename and line number in the panic message. */
#define PVR_ASSERT(EXPR) do											\
	{																\
		if (!(EXPR))												\
		{															\
			PVRSRVDebugPrintf(DBGPRIV_FATAL, __FILE__, __LINE__,	\
							  "Debug assertion failed!");			\
			BUG();													\
		}															\
	} while (0)

#else				/* defined(LINUX) && defined(__KERNEL__) */

	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVDebugAssertFail(const IMG_CHAR *
							       pszFile,
							       IMG_UINT32
							       ui32Line,
							       const IMG_CHAR *
							       pszAssertion);

#if defined(_MSC_VER)
/* This alternate definition is for MSVC, which warns about do {} while (0) */
#define PVR_ASSERT(EXPR) if (!(EXPR)) PVRSRVDebugAssertFail(__FILE__, __LINE__)
#else
#define PVR_ASSERT(EXPR) do										\
	{															\
		if (!(EXPR))											\
			PVRSRVDebugAssertFail(__FILE__, __LINE__, #EXPR);	\
	} while (0);
#endif

#endif				/* defined(LINUX) && defined(__KERNEL__) */

#endif				/* defined(PVRSRV_NEED_PVR_ASSERT) */

#if defined (WIN32)
#define PVR_DBG_BREAK __debugbreak();	/*!< Implementation of PVR_DBG_BREAK for (non-WinCE) Win32 */
#else
#if defined(PVR_DBG_BREAK_ASSERT_FAIL)
	/*!< Implementation of PVR_DBG_BREAK that maps onto PVRSRVDebugAssertFail */
#if defined(_WIN32)
#define PVR_DBG_BREAK	DBG_BREAK
#else
#if defined(LINUX) && defined(__KERNEL__)
#define PVR_DBG_BREAK BUG()
#else
#define PVR_DBG_BREAK	PVRSRVDebugAssertFail(__FILE__, __LINE__, "PVR_DBG_BREAK")
#endif
#endif
#else
	/*!< Null Implementation of PVR_DBG_BREAK (does nothing) */
#define PVR_DBG_BREAK
#endif
#endif

#else				/* defined(PVRSRV_NEED_PVR_ASSERT) */

#define PVR_ASSERT(EXPR) (IMG_VOID)(EXPR)	/*!< Null Implementation of PVR_ASSERT (does nothing) */
#define PVR_DBG_BREAK		/*!< Null Implementation of PVR_DBG_BREAK (does nothing) */

#endif				/* defined(PVRSRV_NEED_PVR_ASSERT) */

/* PVR_DPF() handling */

#if defined(PVRSRV_NEED_PVR_DPF)

#if defined(PVRSRV_NEW_PVR_DPF)

	/* New logging mechanism */
#define PVR_DBG_FATAL		DBGPRIV_FATAL
#define PVR_DBG_ERROR		DBGPRIV_ERROR
#define PVR_DBG_WARNING		DBGPRIV_WARNING
#define PVR_DBG_MESSAGE		DBGPRIV_MESSAGE
#define PVR_DBG_VERBOSE		DBGPRIV_VERBOSE
#define PVR_DBG_CALLTRACE	DBGPRIV_CALLTRACE
#define PVR_DBG_ALLOC		DBGPRIV_ALLOC
#define PVR_DBGDRIV_MESSAGE	DBGPRIV_DBGDRV_MESSAGE

	/* These levels are always on with PVRSRV_NEED_PVR_DPF */
#define __PVR_DPF_0x01UL(...) PVRSRVDebugPrintf(DBGPRIV_FATAL, __VA_ARGS__)
#define __PVR_DPF_0x02UL(...) PVRSRVDebugPrintf(DBGPRIV_ERROR, __VA_ARGS__)

	/* Some are compiled out completely in release builds */
#if defined(DEBUG)
#define __PVR_DPF_0x04UL(...) PVRSRVDebugPrintf(DBGPRIV_WARNING, __VA_ARGS__)
#define __PVR_DPF_0x08UL(...) PVRSRVDebugPrintf(DBGPRIV_MESSAGE, __VA_ARGS__)
#define __PVR_DPF_0x10UL(...) PVRSRVDebugPrintf(DBGPRIV_VERBOSE, __VA_ARGS__)
#define __PVR_DPF_0x20UL(...) PVRSRVDebugPrintf(DBGPRIV_CALLTRACE, __VA_ARGS__)
#define __PVR_DPF_0x40UL(...) PVRSRVDebugPrintf(DBGPRIV_ALLOC, __VA_ARGS__)
#define __PVR_DPF_0x80UL(...) PVRSRVDebugPrintf(DBGPRIV_DBGDRV_MESSAGE, __VA_ARGS__)
#else
#define __PVR_DPF_0x04UL(...)
#define __PVR_DPF_0x08UL(...)
#define __PVR_DPF_0x10UL(...)
#define __PVR_DPF_0x20UL(...)
#define __PVR_DPF_0x40UL(...)
#define __PVR_DPF_0x80UL(...)
#endif

	/* Translate the different log levels to separate macros
	 * so they can each be compiled out.
	 */
#if defined(DEBUG)
#define __PVR_DPF(lvl, ...) __PVR_DPF_ ## lvl (__FILE__, __LINE__, __VA_ARGS__)
#else
#define __PVR_DPF(lvl, ...) __PVR_DPF_ ## lvl ("", 0, __VA_ARGS__)
#endif

	/* Get rid of the double bracketing */
#define PVR_DPF(x) __PVR_DPF x

#else				/* defined(PVRSRV_NEW_PVR_DPF) */

	/* Old logging mechanism */
#define PVR_DBG_FATAL		DBGPRIV_FATAL, __FILE__, __LINE__
#define PVR_DBG_ERROR		DBGPRIV_ERROR, __FILE__, __LINE__
#define PVR_DBG_WARNING		DBGPRIV_WARNING, __FILE__, __LINE__
#define PVR_DBG_MESSAGE		DBGPRIV_MESSAGE, __FILE__, __LINE__
#define PVR_DBG_VERBOSE		DBGPRIV_VERBOSE, __FILE__, __LINE__
#define PVR_DBG_CALLTRACE	DBGPRIV_CALLTRACE, __FILE__, __LINE__
#define PVR_DBG_ALLOC		DBGPRIV_ALLOC, __FILE__, __LINE__
#define PVR_DBGDRIV_MESSAGE	DBGPRIV_DBGDRV_MESSAGE, "", 0

#define PVR_DPF(X)			PVRSRVDebugPrintf X

#endif				/* defined(PVRSRV_NEW_PVR_DPF) */

	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVDebugPrintf(IMG_UINT32
							   ui32DebugLevel,
							   const IMG_CHAR *
							   pszFileName,
							   IMG_UINT32 ui32Line,
							   const IMG_CHAR *
							   pszFormat,
							   ...)
	    IMG_FORMAT_PRINTF(4, 5);

#else				/* defined(PVRSRV_NEED_PVR_DPF) */

#define PVR_DPF(X)		/*!< Null Implementation of PowerVR Debug Printf (does nothing) */

#endif				/* defined(PVRSRV_NEED_PVR_DPF) */

/* PVR_TRACE() handling */

#if defined(PVRSRV_NEED_PVR_TRACE)

#define PVR_TRACE(X)	PVRSRVTrace X	/*!< PowerVR Debug Trace Macro */

	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVTrace(const IMG_CHAR * pszFormat,
						     ...) IMG_FORMAT_PRINTF(1,
									    2);

#else				/* defined(PVRSRV_NEED_PVR_TRACE) */
	/*! Null Implementation of PowerVR Debug Trace Macro (does nothing) */
#define PVR_TRACE(X)

#endif				/* defined(PVRSRV_NEED_PVR_TRACE) */

#if defined (__cplusplus)
}
#endif
#endif				/* __PVR_DEBUG_H__ */
/******************************************************************************
 End of file (pvr_debug.h)
******************************************************************************/
