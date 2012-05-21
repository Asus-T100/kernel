
									    /*************************************************************************//*!
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

/* FIXME_XU: This header file was copied and revised from dc_example.c */

/* Linux headers */
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/spinlock_types.h>	/* Required for pgtable.h */
#include <linux/module.h>

#if defined(SUPPORT_DRI_DRM)
#include <drm/drmP.h>
#endif

#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <asm/page.h>

/* for MODULE_LICENSE */
#include "pvrmodule.h"

/* Services headers */
#include "kerneldisplay.h"
#include "imgpixfmts_km.h"

#if defined (LMA)
#include "physheap.h"
#define DC_PHYS_HEAP_ID		1
#else
#define DC_PHYS_HEAP_ID		0
#endif

#if defined(SUPPORT_DRI_DRM)
#include "pvr_drm.h"
#endif

#if defined (LMA)
#define DRVNAME	"dc_example_LMA"
#else
#define DRVNAME	"dc_example_UMA"
#endif

#define ASSERT(n) \
do \
{ \
	if (!(n)) \
	{ \
		printk(KERN_ERR DRVNAME "Assertion faild @ %s:%d\n", __FUNCTION__, __LINE__); \
		BUG(); \
	} \
} while(0)

/*
	Enable to track contexts and buffers
*/
/* #define DCEX_DEBUG 1 */

/*
	Enable to get more debug. Only supported on UMA
*/
/* #define DCEX_VERBOSE 1*/

#if defined(DCEX_DEBUG)
#define DCEX_DEBUG_PRINT(fmt, ...) \
	printk(KERN_WARNING DRVNAME " " fmt, __VA_ARGS__)
#else
#define DCEX_DEBUG_PRINT(fmt, ...)
#endif

/*
	The number of inflight commands this display driver can handle
*/
#define MAX_COMMANDS_INFLIGHT 2

extern PVRSRV_ERROR MerrifieldDCInit(struct drm_device unref__ * dev);
extern PVRSRV_ERROR MerrifieldDCDeinit(void);
