/*************************************************************************/ /*!
@File
@Title          Misc memory management utility functions for Linux
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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

#include <asm/io.h>

#include "img_defs.h"
#include "mutils.h"
#include "pvr_debug.h"
#include "mm.h"
#include "pvrsrv_memallocflags.h"



IMG_VOID *
_IORemapWrapper(IMG_CPU_PHYADDR BasePAddr,
               IMG_UINT32 ui32Bytes,
               IMG_UINT32 ui32MappingFlags,
               IMG_CHAR *pszFileName,
               IMG_UINT32 ui32Line)
{
    IMG_VOID *pvIORemapCookie;

#if 1    
	/* FIXME: We should not be using PVRSRV_HAP_*, heap flags should have no meaning here */
	switch (ui32MappingFlags & PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK)
	{
		/* FIXME: What do we do for cache coherent? For now make uncached*/
		case PVRSRV_MEMALLOCFLAG_CPU_CACHE_COHERENT:
		case PVRSRV_MEMALLOCFLAG_CPU_UNCACHED:
				pvIORemapCookie = (IMG_VOID *)IOREMAP_UC(BasePAddr.uiAddr, ui32Bytes);
				break;

		case PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE:
				pvIORemapCookie = (IMG_VOID *)IOREMAP_WC(BasePAddr.uiAddr, ui32Bytes);
				break;

		case PVRSRV_MEMALLOCFLAG_CPU_CACHE_INCOHERENT:
				pvIORemapCookie = (IMG_VOID *)IOREMAP(BasePAddr.uiAddr, ui32Bytes);
				break;

		default:
				return IMG_NULL;
				break;
	}
#else
    PVR_UNREFERENCED_PARAMETER(ui32MappingFlags);

    pvIORemapCookie = (IMG_VOID *)IOREMAP_UC(BasePAddr.uiAddr, ui32Bytes);
#endif

    PVR_UNREFERENCED_PARAMETER(pszFileName);
    PVR_UNREFERENCED_PARAMETER(ui32Line);

    return pvIORemapCookie;
}


IMG_VOID
_IOUnmapWrapper(IMG_VOID *pvIORemapCookie, IMG_CHAR *pszFileName, IMG_UINT32 ui32Line)
{
    PVR_UNREFERENCED_PARAMETER(pszFileName);
    PVR_UNREFERENCED_PARAMETER(ui32Line);
    iounmap(pvIORemapCookie);
}
