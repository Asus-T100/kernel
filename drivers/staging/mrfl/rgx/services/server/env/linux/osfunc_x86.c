									    /*************************************************************************//*!
									       @Title          x86 specific OS functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    OS functions who's implementation are processor specific
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include <linux/smp.h>
#include <asm/system.h>

#include "pvrsrv_error.h"
#include "img_types.h"
#include "osfunc.h"
#include "pvr_debug.h"

static void per_cpu_cache_flush(void *arg)
{
	PVR_UNREFERENCED_PARAMETER(arg);
	wbinvd();
}

IMG_VOID OSCPUOperation(PVRSRV_CACHE_OP uiCacheOp)
{
	switch (uiCacheOp) {
		/* Fall-through */
	case PVRSRV_CACHE_OP_CLEAN:
	case PVRSRV_CACHE_OP_FLUSH:
	case PVRSRV_CACHE_OP_INVALIDATE:
		on_each_cpu(per_cpu_cache_flush, NULL, 1);
		break;

	case PVRSRV_CACHE_OP_NONE:
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Invalid cache operation type %d",
			 __FUNCTION__, uiCacheOp));
		PVR_ASSERT(0);
		break;
	}
}
