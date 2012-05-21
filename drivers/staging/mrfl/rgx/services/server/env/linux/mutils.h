									    /*************************************************************************//*!
									       @File
									       @Title          Declares various memory management support functions for Linux.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef __IMG_LINUX_MUTILS_H__
#define __IMG_LINUX_MUTILS_H__

#include <linux/version.h>

#if !(defined(__i386__) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)))
#if defined(SUPPORT_LINUX_X86_PAT)
#undef SUPPORT_LINUX_X86_PAT
#endif
#endif

#if defined(SUPPORT_LINUX_X86_PAT)
pgprot_t pvr_pgprot_writecombine(pgprot_t prot);
#define	PGPROT_WC(pv)	pvr_pgprot_writecombine(pv)
#else
#if defined(__arm__) || defined(__sh__)
#define	PGPROT_WC(pv)	pgprot_writecombine(pv)
#else
#if defined(__i386__)
#define	PGPROT_WC(pv)	pgprot_noncached(pv)
#else
#define PGPROT_WC(pv)	pgprot_noncached(pv)
#error  Unsupported architecture!
#endif
#endif
#endif

#define	PGPROT_UC(pv)	pgprot_noncached(pv)

#if defined(__i386__) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#define	IOREMAP(pa, bytes)	ioremap_cache(pa, bytes)
#else
#if defined(__arm__) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#define	IOREMAP(pa, bytes)	ioremap_cached(pa, bytes)
#else
#define IOREMAP(pa, bytes)	ioremap(pa, bytes)
#endif
#endif

#if defined(SUPPORT_LINUX_X86_PAT)
#if defined(SUPPORT_LINUX_X86_WRITECOMBINE)
#define IOREMAP_WC(pa, bytes) ioremap_wc(pa, bytes)
#else
#define IOREMAP_WC(pa, bytes) ioremap_nocache(pa, bytes)
#endif
#else
#if defined(__arm__)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
#define IOREMAP_WC(pa, bytes) ioremap_wc(pa, bytes)
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22))
#define	IOREMAP_WC(pa, bytes)	ioremap_nocache(pa, bytes)
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)) || (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
#define	IOREMAP_WC(pa, bytes)	__ioremap(pa, bytes, L_PTE_BUFFERABLE)
#else
#define IOREMAP_WC(pa, bytes)	__ioremap(pa, bytes, , L_PTE_BUFFERABLE, 1)
#endif
#endif
#endif
#else
#define IOREMAP_WC(pa, bytes)	ioremap_nocache(pa, bytes)
#endif
#endif

#define	IOREMAP_UC(pa, bytes)	ioremap_nocache(pa, bytes)

IMG_VOID PVRLinuxMUtilsInit(IMG_VOID);

#endif				/* __IMG_LINUX_MUTILS_H__ */
