/******************************************************************************
 * Name         : pdump_symbolicaddr.h
 * Title        : Abstraction of PDUMP symbolic address derivation
 * Author(s)    : Imagination Technologies
 * Created      : 
 *
 * Copyright    : 2010 by Imagination Technologies Limited.
 *                All rights reserved. No part of this software, either
 *                material or conceptual may be copied or distributed,
 *                transmitted, transcribed, stored in a retrieval system or
 *                translated into any human or computer language in any form
 *                by any means, electronic, mechanical, manual or otherwise,
 *                or disclosed to third parties without the express written
 *                permission of Imagination Technologies Limited,
 *                Home Park Estate, Kings Langley, Hertfordshire,
 *                WD4 8LZ, U.K.
 *
 * Description  : Allows pdump functions to derive symbolic addresses on-the-fly
 *
 * Platform     : ALL
 *
 *****************************************************************************/

#ifndef SRVKM_PDUMP_SYMBOLICADDR_H
#define SRVKM_PDUMP_SYMBOLICADDR_H

#include "img_types.h"

#include "pvrsrv_error.h"

/* pdump symbolic addresses are generated on-the-fly with a callback */

typedef PVRSRV_ERROR(*PVRSRV_SYMADDRFUNCPTR) (IMG_HANDLE hPriv,
					      IMG_UINT32 uiOffset,
					      IMG_CHAR * pszSymbolicAddr,
					      IMG_UINT32 ui32SymbolicAddrLen,
					      IMG_UINT32 * pui32NewOffset);

#endif				/* #ifndef SRVKM_PDUMP_SYMBOLICADDR_H */
