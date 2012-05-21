									    /*************************************************************************//*!
									       @File
									       @Title          Global types for use by IMG APIs
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines type aliases for use by IMG APIs.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __IMG_TYPES_H__
#define __IMG_TYPES_H__

/* define all address space bit depths: */
/* CPU virtual address space defaults to 32bits */
#if !defined(IMG_ADDRSPACE_CPUVADDR_BITS)
#define IMG_ADDRSPACE_CPUVADDR_BITS		32
#endif

/* Physical address space defaults to 32bits */
#if !defined(IMG_ADDRSPACE_PHYSADDR_BITS)
#define IMG_ADDRSPACE_PHYSADDR_BITS		32
#endif

typedef unsigned int IMG_UINT, *IMG_PUINT;
typedef signed int IMG_INT, *IMG_PINT;

typedef unsigned char IMG_UINT8, *IMG_PUINT8;
typedef unsigned char IMG_BYTE, *IMG_PBYTE;
typedef signed char IMG_INT8, *IMG_PINT8;
typedef char IMG_CHAR, *IMG_PCHAR;
typedef IMG_CHAR const *IMG_PCCHAR;

typedef unsigned short IMG_UINT16, *IMG_PUINT16;
typedef signed short IMG_INT16, *IMG_PINT16;
#if !defined(IMG_UINT32_IS_ULONG)
typedef unsigned int IMG_UINT32, *IMG_PUINT32;
typedef signed int IMG_INT32, *IMG_PINT32;
#else
typedef unsigned long IMG_UINT32, *IMG_PUINT32;
typedef signed long IMG_INT32, *IMG_PINT32;
#endif
#if !defined(IMG_UINT32_MAX)
#define IMG_UINT32_MAX 0xFFFFFFFFUL
#endif

typedef IMG_UINT16 const *IMG_PCUINT16;
typedef IMG_INT16 const *IMG_PCINT16;
typedef IMG_UINT32 const *IMG_PCUINT32;
typedef IMG_INT32 const *IMG_PCINT32;

#if defined(_WIN32)

typedef unsigned __int64 IMG_UINT64, *IMG_PUINT64;
typedef __int64 IMG_INT64, *IMG_PINT64;
#define IMG_INT64_C(c)	c ## LL
#define IMG_UINT64_C(c)	c ## ULL

#else
#if defined(LINUX) || defined(__METAG)
typedef unsigned long long IMG_UINT64, *IMG_PUINT64;
typedef long long IMG_INT64, *IMG_PINT64;
#define IMG_INT64_C(c)	c ## LL
#define IMG_UINT64_C(c)	c ## ULL
#else
#error("define an OS")
#endif
#endif

#if !(defined(LINUX) && defined (__KERNEL__))
/* Linux kernel mode does not use floating point */
typedef float IMG_FLOAT, *IMG_PFLOAT;
typedef double IMG_DOUBLE, *IMG_PDOUBLE;
#endif

#if defined(LINUX)
typedef int IMG_SECURE_TYPE;
#endif

typedef enum tag_img_bool {
	IMG_FALSE = 0,
	IMG_TRUE = 1,
	IMG_FORCE_ALIGN = 0x7FFFFFFF
} IMG_BOOL, *IMG_PBOOL;

typedef void IMG_VOID, *IMG_PVOID;

typedef IMG_INT32 IMG_RESULT;

/* Figure out which headers to include to get uintptr_t. */
#if !defined(_MSC_VER) && !defined(__KERNEL__)
/* MSVC before VS2010 doesn't have <stdint.h>, but it has uintptr_t in
   <stddef.h>. */
#include <stdint.h>
#endif
#if defined(__KERNEL__)
#include <linux/types.h>
#endif
#include <stddef.h>

typedef uintptr_t IMG_UINTPTR_T;
typedef size_t IMG_SIZE_T;

#if defined(_MSC_VER)
#define IMG_SIZE_FMTSPEC "%Iu"
#else
#define IMG_SIZE_FMTSPEC "%zu"
#endif

typedef IMG_PVOID IMG_HANDLE;

typedef void **IMG_HVOID, *IMG_PHVOID;

#define IMG_NULL        0

/* services/stream ID */
typedef IMG_UINT32 IMG_SID;

/* Process IDs */
typedef IMG_UINT32 IMG_PID;

/*
 * Address types.
 * All types used to refer to a block of memory are wrapped in structures
 * to enforce some degree of type safety, i.e. a IMG_DEV_VIRTADDR cannot
 * be assigned to a variable of type IMG_DEV_PHYADDR because they are not the
 * same thing.
 *
 * There is an assumption that the system contains at most one non-cpu mmu,
 * and a memory block is only mapped by the MMU once.
 *
 * Different devices could have offset views of the physical address space.
 * 
 */

/*
 *
 * +------------+    +------------+      +------------+        +------------+
 * |    CPU     |    |    DEV     |      |    DEV     |        |    DEV     |
 * +------------+    +------------+      +------------+        +------------+
 *       |                 |                   |                     |
 *       | PVOID           |IMG_DEV_VIRTADDR   |IMG_DEV_VIRTADDR     |
 *       |                 \-------------------/                     |
 *       |                          |                                |
 * +------------+             +------------+                         |     
 * |    MMU     |             |    MMU     |                         |
 * +------------+             +------------+                         | 
 *       |                          |                                | 
 *       |                          |                                |
 *       |                          |                                |
 *   +--------+                +---------+                      +--------+
 *   | Offset |                | (Offset)|                      | Offset |
 *   +--------+                +---------+                      +--------+    
 *       |                          |                IMG_DEV_PHYADDR | 
 *       |                          |                                |
 *       |                          | IMG_DEV_PHYADDR                |
 * +---------------------------------------------------------------------+ 
 * |                         System Address bus                          |
 * +---------------------------------------------------------------------+
 *
 */

typedef IMG_PVOID IMG_CPU_VIRTADDR;

/* device virtual address */
typedef struct _IMG_DEV_VIRTADDR {
	IMG_UINT64 uiAddr;
#define IMG_CAST_TO_DEVVADDR_UINT(var)		(IMG_UINT64)(var)

} IMG_DEV_VIRTADDR;

typedef IMG_UINT64 IMG_DEVMEM_SIZE_T;
typedef IMG_UINT64 IMG_DEVMEM_ALIGN_T;
typedef IMG_UINT64 IMG_DEVMEM_OFFSET_T;
typedef IMG_UINT32 IMG_DEVMEM_LOG2ALIGN_T;

#define IMG_DEV_VIRTADDR_FMTSPEC "0x%010llX"
#define IMG_DEVMEM_SIZE_FMTSPEC "0x%010llX"
#define IMG_DEVMEM_ALIGN_FMTSPEC "0x%010llX"
#define IMG_DEVMEM_OFFSET_FMTSPEC "0x%010llX"

/* cpu physical address */
typedef struct _IMG_CPU_PHYADDR {
	/* variable sized type (32,64) */
	IMG_UINTPTR_T uiAddr;
#define IMG_CAST_TO_CPUPHYADDR_UINT(var)		(IMG_UINTPTR_T)(var)
} IMG_CPU_PHYADDR;

/* device physical address */
typedef struct _IMG_DEV_PHYADDR {
	IMG_UINT64 uiAddr;
} IMG_DEV_PHYADDR;

/* system physical address */
typedef struct _IMG_SYS_PHYADDR {
	/* variable sized type (32,64) */
	IMG_UINTPTR_T uiAddr;
} IMG_SYS_PHYADDR;

/*
	rectangle structure
*/
typedef struct _IMG_RECT_ {
	IMG_INT32 x0;
	IMG_INT32 y0;
	IMG_INT32 x1;
	IMG_INT32 y1;
} IMG_RECT;

typedef struct _IMG_RECT_16_ {
	IMG_INT16 x0;
	IMG_INT16 y0;
	IMG_INT16 x1;
	IMG_INT16 y1;
} IMG_RECT_16;

#include "img_defs.h"

#endif				/* __IMG_TYPES_H__ */
/******************************************************************************
 End of file (img_types.h)
******************************************************************************/
