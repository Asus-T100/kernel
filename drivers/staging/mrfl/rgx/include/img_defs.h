									    /*************************************************************************//*!
									       @File
									       @Title          Common header containing type definitions for portability
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Contains variable and structure definitions. Any platform
									       specific types should be defined in this file.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__IMG_DEFS_H__)
#define __IMG_DEFS_H__

#include <stddef.h>

#include "img_types.h"

typedef enum img_tag_TriStateSwitch {
	IMG_ON = 0x00,
	IMG_OFF,
	IMG_IGNORE
} img_TriStateSwitch, *img_pTriStateSwitch;

#define		IMG_SUCCESS				0

#define		IMG_NO_REG				1

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
#if defined(UNDER_VISTA)
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

/*
	Wide character definitions
*/
#ifndef _TCHAR_DEFINED
#if defined(UNICODE)
typedef unsigned short TCHAR, *PTCHAR, *PTSTR;
#else				/* #if defined(UNICODE) */
typedef char TCHAR, *PTCHAR, *PTSTR;
#endif				/* #if defined(UNICODE) */
#define _TCHAR_DEFINED
#endif				/* #ifndef _TCHAR_DEFINED */

#if defined(_WIN32)

#define IMG_CALLCONV __stdcall
#define IMG_INTERNAL
#define	IMG_EXPORT	__declspec(dllexport)
#define IMG_RESTRICT __restrict

	/* IMG_IMPORT is defined as IMG_EXPORT so that headers and implementations match.
	 * Some compilers require the header to be declared IMPORT, while the implementation is declared EXPORT 
	 */
#define	IMG_IMPORT	IMG_EXPORT
#if defined( UNDER_VISTA )
#ifndef	_INC_STDLIB
void __cdecl abort(void);	/*PRQA S 5115 *//* Make sure abort is defined. */
#endif
#if defined(EXIT_ON_ABORT)
#define IMG_ABORT()	exit(1);
#else
#define IMG_ABORT()	abort();
#endif
//              #define IMG_ABORT()     img_abort()
#endif
#else
#if defined(__linux__) || defined(__METAG)

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

#ifndef IMG_MALLOC
#define IMG_MALLOC(A)		malloc	(A)
#endif

#ifndef IMG_FREE
#define IMG_FREE(A)			free	(A)
#endif

#define IMG_CONST const

#if defined(__GNUC__)
#define IMG_FORMAT_PRINTF(x,y)		__attribute__((format(printf,x,y)))
#else
#define IMG_FORMAT_PRINTF(x,y)
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

#endif				/* #if !defined (__IMG_DEFS_H__) */
/*****************************************************************************
 End of file (IMG_DEFS.H)
*****************************************************************************/
