/**************************************************************************
 *
 * Name         : allocmem.c
 *
 * Copyright    : 2007 by Imagination Technologies Limited. 
 *                All rights reserved.
 *                No part of this software, either material or conceptual 
 *                may be copied or distributed, transmitted, transcribed,
 *                stored in a retrieval system or translated into any 
 *                human or computer language in any form by any means,
 *                electronic, mechanical, manual or other-wise, or 
 *                disclosed to third parties without the express written
 *                permission of:
 *                Imagination Technologies Limited, 
 *                HomePark Industrial Estate, 
 *                Kings Langley, 
 *                Hertfordshire,
 *                WD4 8LZ, 
 *                UK
 *
 * Description  : Host memory management implementation for Linux
 *
 **************************************************************************/

#include <linux/slab.h>

#include "img_defs.h"
#include "allocmem.h"

IMG_INTERNAL IMG_PVOID OSAllocMem(IMG_UINT32 ui32Size)
{
	IMG_PVOID pvRet = kmalloc(ui32Size, GFP_KERNEL);
	return pvRet;
}

IMG_INTERNAL IMG_VOID OSFreeMem(IMG_PVOID pvMem)
{
	kfree(pvMem);
}
