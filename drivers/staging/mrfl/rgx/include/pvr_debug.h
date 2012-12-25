/*************************************************************************/ /*!
@File
@Title          PVR Debug Declarations
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Provides debug functionality
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

#ifndef __PVR_DEBUG_H__
#define __PVR_DEBUG_H__

#include "img_types.h"

#if defined (WIN32)
extern IMG_VOID __debugbreak(IMG_VOID);
#endif

#if defined (__cplusplus)
extern "C" {
#endif

#define PVR_MAX_DEBUG_MESSAGE_LEN	(512)   /*!< Max length of a Debug Message */

/* These are privately used by pvr_debug, use the PVR_DBG_ defines instead */
#define DBGPRIV_FATAL			0x001UL  /*!< Debug-Fatal. Privately used by pvr_debug. */
#define DBGPRIV_ERROR			0x002UL  /*!< Debug-Error. Privately used by pvr_debug. */
#define DBGPRIV_WARNING			0x004UL  /*!< Debug-Warning. Privately used by pvr_debug. */
#define DBGPRIV_MESSAGE			0x008UL  /*!< Debug-Message. Privately used by pvr_debug. */
#define DBGPRIV_VERBOSE			0x010UL  /*!< Debug-Verbose. Privately used by pvr_debug. */
#define DBGPRIV_CALLTRACE		0x020UL  /*!< Debug-CallTrace. Privately used by pvr_debug. */
#define DBGPRIV_ALLOC			0x040UL  /*!< Debug-Alloc. Privately used by pvr_debug. */
#define DBGPRIV_BUFFERED		0x080UL  /*!< Debug-Buffered. Privately used by pvr_debug. */
#define DBGPRIV_DBGDRV_MESSAGE	0x100UL  /*!< Debug-DbgDrivMessage. Privately used by pvr_debug. */

#define DBGPRIV_DBGLEVEL_COUNT	8

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
#include <linux/bug.h>
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

#else /* defined(LINUX) && defined(__KERNEL__) */

IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVDebugAssertFail(const IMG_CHAR *pszFile,
													   IMG_UINT32 ui32Line,
													   const IMG_CHAR *pszAssertion);

#if defined(_MSC_VER)
/* This alternate definition is for MSVC, which warns about do {} while (0) */
#define PVR_ASSERT(EXPR) if (!(EXPR)) PVRSRVDebugAssertFail(__FILE__, __LINE__)
#else
#define PVR_ASSERT(EXPR) do										\
	{															\
		if (!(EXPR))											\
			PVRSRVDebugAssertFail(__FILE__, __LINE__, #EXPR);	\
	} while (0)
#endif

#endif /* defined(LINUX) && defined(__KERNEL__) */

#endif /* defined(PVRSRV_NEED_PVR_ASSERT)*/

#if defined (WIN32)
	#define PVR_DBG_BREAK __debugbreak();   /*!< Implementation of PVR_DBG_BREAK for (non-WinCE) Win32 */
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

#else  /* defined(PVRSRV_NEED_PVR_ASSERT) */

	#define PVR_ASSERT(EXPR) (IMG_VOID)(EXPR) /*!< Null Implementation of PVR_ASSERT (does nothing) */
	#define PVR_DBG_BREAK    /*!< Null Implementation of PVR_DBG_BREAK (does nothing) */

#endif /* defined(PVRSRV_NEED_PVR_ASSERT) */


/* PVR_DPF() handling */

#if defined(PVRSRV_NEED_PVR_DPF)

	/* New logging mechanism */
	#define PVR_DBG_FATAL		DBGPRIV_FATAL
	#define PVR_DBG_ERROR		DBGPRIV_ERROR
	#define PVR_DBG_WARNING		DBGPRIV_WARNING
	#define PVR_DBG_MESSAGE		DBGPRIV_MESSAGE
	#define PVR_DBG_VERBOSE		DBGPRIV_VERBOSE
	#define PVR_DBG_CALLTRACE	DBGPRIV_CALLTRACE
	#define PVR_DBG_ALLOC		DBGPRIV_ALLOC
	#define PVR_DBG_BUFFERED	DBGPRIV_BUFFERED
	#define PVR_DBGDRIV_MESSAGE	DBGPRIV_DBGDRV_MESSAGE

	/* These levels are always on with PVRSRV_NEED_PVR_DPF */
	#define __PVR_DPF_0x001UL(...) PVRSRVDebugPrintf(DBGPRIV_FATAL, __VA_ARGS__)
	#define __PVR_DPF_0x002UL(...) PVRSRVDebugPrintf(DBGPRIV_ERROR, __VA_ARGS__)
	#define __PVR_DPF_0x080UL(...) PVRSRVDebugPrintf(DBGPRIV_BUFFERED, __VA_ARGS__)

	/* Some are compiled out completely in release builds */
#if defined(DEBUG)
	#define __PVR_DPF_0x004UL(...) PVRSRVDebugPrintf(DBGPRIV_WARNING, __VA_ARGS__)
	#define __PVR_DPF_0x008UL(...) PVRSRVDebugPrintf(DBGPRIV_MESSAGE, __VA_ARGS__)
	#define __PVR_DPF_0x010UL(...) PVRSRVDebugPrintf(DBGPRIV_VERBOSE, __VA_ARGS__)
	#define __PVR_DPF_0x020UL(...) PVRSRVDebugPrintf(DBGPRIV_CALLTRACE, __VA_ARGS__)
	#define __PVR_DPF_0x040UL(...) PVRSRVDebugPrintf(DBGPRIV_ALLOC, __VA_ARGS__)
	#define __PVR_DPF_0x100UL(...) PVRSRVDebugPrintf(DBGPRIV_DBGDRV_MESSAGE, __VA_ARGS__)
#else
	#define __PVR_DPF_0x004UL(...)
	#define __PVR_DPF_0x008UL(...)
	#define __PVR_DPF_0x010UL(...)
	#define __PVR_DPF_0x020UL(...)
	#define __PVR_DPF_0x040UL(...)
	#define __PVR_DPF_0x100UL(...)
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

	#define PVR_LOG_IF_ERROR(e, c) do \
		{ int _r = (e); \
		  if (_r != PVRSRV_OK) \
			PVR_DPF((PVR_DBG_ERROR, "%s() failed (%d) in %s()", c, e, __func__)); \
		} while (0)


	#define PVR_LOGR_IF_ERROR(e, c) do \
		{ int _r = (e); \
		  if (_r != PVRSRV_OK) { \
			PVR_DPF((PVR_DBG_ERROR, "%s() failed (%d) in %s()", c, _r, __func__)); \
			return (_r); }\
		} while (0)

	#define PVR_LOGRN_IF_ERROR(e, c) do \
		{ int _r = (e); \
		  if (_r != PVRSRV_OK) { \
			PVR_DPF((PVR_DBG_ERROR, "%s() failed (%d) in %s()", c, _r, __func__)); \
			return; }\
		} while (0)

	#define PVR_LOGG_IF_ERROR(e, c, g) do \
		{ int _r = (e); \
		if (_r != PVRSRV_OK) { \
			PVR_DPF((PVR_DBG_ERROR, "%s() failed (%d) in %s()", c, _r, __func__)); \
			goto g; }\
		} while (0)

IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVDebugPrintf(IMG_UINT32 ui32DebugLevel,
												   const IMG_CHAR *pszFileName,
												   IMG_UINT32 ui32Line,
												   const IMG_CHAR *pszFormat,
												   ...) IMG_FORMAT_PRINTF(4, 5);

IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVDebugPrintfDumpCCB(void);

#else  /* defined(PVRSRV_NEED_PVR_DPF) */

	#define PVR_DPF(X)  /*!< Null Implementation of PowerVR Debug Printf (does nothing) */

	#define PVR_LOG_IF_ERROR(e, c)
	#define PVR_LOGR_IF_ERROR(e, c) do { if (e != PVRSRV_OK) { return (e); } } while(0)
	#define PVR_LOGRN_IF_ERROR(e, c) do { if (e != PVRSRV_OK) { return; } } while(0)
	#define PVR_LOGG_IF_ERROR(e, c, g) do { if (e != PVRSRV_OK) { goto g; } } while(0)

	#undef PVR_DPF_FUNCTION_TRACE_ON

#endif /* defined(PVRSRV_NEED_PVR_DPF) */


#if defined(PVR_DPF_FUNCTION_TRACE_ON)

	#define PVR_DPF_ENTERED \
        PVR_DPF((PVR_DBG_CALLTRACE, "--> %s:%d entered", __func__, __LINE__))

	#define PVR_DPF_RETURN_RC(a) \
        do { int _r = (a); PVR_DPF((PVR_DBG_CALLTRACE, "-< %s:%d returned %d", __func__, __LINE__, (_r))); return (_r); } while (0)

	#define PVR_DPF_RETURN_VAL(a) \
		do { PVR_DPF((PVR_DBG_CALLTRACE, "-< %s:%d returned with value", __func__, __LINE__ )); return (a); } while (0)

	#define PVR_DPF_RETURN_OK \
		do { PVR_DPF((PVR_DBG_CALLTRACE, "-< %s:%d returned ok", __func__, __LINE__)); return PVRSRV_OK; } while (0)

	#define PVR_DPF_RETURN \
		do { PVR_DPF((PVR_DBG_CALLTRACE, "-< %s:%d returned", __func__, __LINE__)); return; } while (0)

	#if !defined(DEBUG)
	#error PVR DPF Function trace enabled in release build, rectify
	#endif

#else /* defined(PVR_DPF_FUNCTION_TRACE_ON) */

	#define PVR_DPF_ENTERED
	#define PVR_DPF_RETURN_RC(a) 	return (a);
	#define PVR_DPF_RETURN_VAL(a) 	return (a);
	#define PVR_DPF_RETURN_OK 		return PVRSRV_OK;
	#define PVR_DPF_RETURN	 		return;

#endif /* defined(PVR_DPF_FUNCTION_TRACE_ON) */


/* PVR_TRACE() handling */

#if defined(PVRSRV_NEED_PVR_TRACE)

	#define PVR_TRACE(X)	PVRSRVTrace X    /*!< PowerVR Debug Trace Macro */

IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVTrace(const IMG_CHAR* pszFormat, ... )
	IMG_FORMAT_PRINTF(1, 2);

#else /* defined(PVRSRV_NEED_PVR_TRACE) */
    /*! Null Implementation of PowerVR Debug Trace Macro (does nothing) */
	#define PVR_TRACE(X)    

#endif /* defined(PVRSRV_NEED_PVR_TRACE) */


#if defined (__cplusplus)
}
#endif

#endif	/* __PVR_DEBUG_H__ */

/******************************************************************************
 End of file (pvr_debug.h)
******************************************************************************/

