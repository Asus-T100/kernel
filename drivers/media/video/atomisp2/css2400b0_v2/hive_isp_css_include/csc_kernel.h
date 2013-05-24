#ifndef __CSC_KERNEL_H_INCLUDED__
#define __CSC_KERNEL_H_INCLUDED__

/*
 * This file is included on every cell {SP,ISP,host} and on every system
 * that uses the CSC kernel.
 *
 * System and cell specific interfaces and inline code are included
 * conditionally through Makefile path settings.
 *
 *  - .        system and cell agnostic interfaces, constants and identifiers
 *	- public:  system agnostic, cell specific interfaces
 *	- private: system dependent, cell specific interfaces & inline implementations
 *	- global:  system specific constants and identifiers
 *	- local:   system and cell specific constants and identifiers
 */

#include "storage_class.h"

#include "csc_kernel_local.h"

#ifdef __ON__

#ifndef __INLINE_CSC_KERNEL__
#define STORAGE_CLASS_CSC_KERNEL_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_CSC_KERNEL_C 
#include "csc_kernel_public.h"
#else  /* __INLINE_CSC_KERNEL__ */
#define STORAGE_CLASS_CSC_KERNEL_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_CSC_KERNEL_C STORAGE_CLASS_INLINE
#include "csc_kernel_private.h"
#endif /* __INLINE_CSC_KERNEL__ */

#endif /* __ON__ */

#endif /* __CSC_KERNEL_H_INCLUDED__ */
