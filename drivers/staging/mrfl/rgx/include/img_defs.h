/*************************************************************************/ /*!
@File
@Title          Common header containing type definitions for portability
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Contains variable and structure definitions. Any platform
                specific types should be defined in this file.
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

#if !defined (__IMG_DEFS_H__)
#define __IMG_DEFS_H__

#include <stddef.h>

#include "img_types.h"

#if defined (NO_INLINE_FUNCS)
	#define	INLINE
	#define	FORCE_INLINE
#else
#if defined (__cplusplus)
	#define INLINE					inline
	#define	FORCE_INLINE			inline
#else
#if	!defined(INLINE)
	#define	INLINE					__inline
#endif
#if defined(UNDER_WDDM)
	#define	FORCE_INLINE			__forceinline
#else
	#define	FORCE_INLINE			static __inline
#endif
#endif
#endif


/* Use this in any file, or use attributes under GCC - see below */
#ifndef PVR_UNREFERENCED_PARAMETER
#define	PVR_UNREFERENCED_PARAMETER(param) ((void)(param))
#endif

/* The best way to supress unused parameter warnings using GCC is to use a
 * variable attribute.  Place the unref__ between the type and name of an
 * unused parameter in a function parameter list, eg `int unref__ var'. This
 * should only be used in GCC build environments, for example, in files that
 * compile only on Linux. Other files should use UNREFERENCED_PARAMETER */
#ifdef __GNUC__
#define unref__ __attribute__ ((unused))
#else
#define unref__
#endif

#if defined(_WIN32)

	#define IMG_CALLCONV __stdcall
	#define IMG_INTERNAL
	#define	IMG_EXPORT	__declspec(dllexport)
	#define IMG_RESTRICT __restrict


	/* IMG_IMPORT is defined as IMG_EXPORT so that headers and implementations match.
	 * Some compilers require the header to be declared IMPORT, while the implementation is declared EXPORT 
	 */
	#define	IMG_IMPORT	IMG_EXPORT
	#if defined(UNDER_WDDM)
		#ifndef	_INC_STDLIB
			#if defined (UNDER_WIN8)
				_CRTIMP __declspec(noreturn) void __cdecl abort(void);
			#else
				_CRTIMP void __cdecl abort(void);
			#endif
		#endif
		#if defined(EXIT_ON_ABORT)
			#define IMG_ABORT()	exit(1);
		#else
			#define IMG_ABORT()	abort();
		#endif
//		#define IMG_ABORT()	img_abort()
	#endif
#else
	#if defined(LINUX) || defined(__METAG)

		#define IMG_CALLCONV
		#if defined(__linux__)
			#define IMG_INTERNAL	__attribute__((visibility("hidden")))
		#else
			#define IMG_INTERNAL
		#endif
		#define IMG_EXPORT		__attribute__((visibility("default")))
		#define IMG_IMPORT
		#define IMG_RESTRICT	__restrict__

	#else
		#error("define an OS")
	#endif
#endif

// Use default definition if not overridden
#ifndef IMG_ABORT
	#if defined(EXIT_ON_ABORT)
		#define IMG_ABORT()	exit(1)
	#else
		#define IMG_ABORT()	abort()
	#endif
#endif

#if defined(__GNUC__)
#define IMG_FORMAT_PRINTF(x,y)		__attribute__((format(printf,x,y)))
#else
#define IMG_FORMAT_PRINTF(x,y)
#endif

#if defined(__GNUC__)
#define IMG_WARN_UNUSED_RESULT		__attribute__((warn_unused_result))
#else
#define IMG_WARN_UNUSED_RESULT
#endif

#if defined(_MSC_VER) || defined(CC_ARM)
	#define IMG_NORETURN __declspec(noreturn)
#else
	#if defined(__GNUC__)
		#define IMG_NORETURN __attribute__((noreturn))
	#else
		#define IMG_NORETURN
	#endif
#endif

#if defined (_WIN64)
#define IMG_UNDEF	(~0ULL)
#else
#define IMG_UNDEF	(~0U)
#endif

#define MAX(a,b) 					(((a) > (b)) ? (a) : (b))
#define MIN(a,b) 					(((a) < (b)) ? (a) : (b))

/* Get a structures address from the address of a member */
#define IMG_CONTAINER_OF(ptr, type, member) \
	(type *) ((IMG_UINT8 *) (ptr) - offsetof(type, member))

/* To guarantee that __func__ can be used, define it as a macro here if it
   isn't already provided by the compiler. */
#if defined(_MSC_VER)
#define __func__ __FUNCTION__
#endif

#endif /* #if !defined (__IMG_DEFS_H__) */
/*****************************************************************************
 End of file (IMG_DEFS.H)
*****************************************************************************/

