/*************************************************************************/ /*!
@File
@Title          RGX heap definitions
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

#if !defined(__RGX_HEAPS_H__)
#define __RGX_HEAPS_H__

#include "km/rgxdefs_km.h"

/* RGX Heap IDs, note: not all heaps are available to clients */
/* N.B.  Old heap identifiers are deprecated now that the old memory
   management is. New heap identifiers should be suitably renamed */
#define RGX_UNDEFINED_HEAP_ID					(~0LU)          /*!< RGX Undefined Heap ID */
#define RGX_GENERAL_HEAP_ID						0               /*!< RGX General Heap ID */
#define RGX_PDSCODEDATA_HEAP_ID					1               /*!< RGX PDS Code/Data Heap ID */
#define RGX_3DPARAMETERS_HEAP_ID				2               /*!< RGX 3D Parameters Heap ID */
#define RGX_USCCODE_HEAP_ID						3               /*!< RGX USC Code Heap ID */
#define RGX_FIRMWARE_HEAP_ID					4               /*!< RGX Firmware Heap ID */
#define RGX_TQ3DPARAMETERS_HEAP_ID				5               /*!< RGX Firmware Heap ID */
#define RGX_TILING_XSTRIDE0_HEAP_ID				6 				/*!< RGX Tiling Heap (x-stride=0) ID */
#define RGX_TILING_XSTRIDE1_HEAP_ID				7 				/*!< RGX Tiling Heap (x-stride=1) ID */
#define RGX_TILING_XSTRIDE2_HEAP_ID				8 				/*!< RGX Tiling Heap (x-stride=2) ID */
#define RGX_TILING_XSTRIDE3_HEAP_ID				9 				/*!< RGX Tiling Heap (x-stride=3) ID */
#define RGX_TILING_XSTRIDE4_HEAP_ID				10				/*!< RGX Tiling Heap (x-stride=4) ID */
#define RGX_TILING_XSTRIDE5_HEAP_ID				11				/*!< RGX Tiling Heap (x-stride=5) ID */
#define RGX_TILING_XSTRIDE6_HEAP_ID				12				/*!< RGX Tiling Heap (x-stride=6) ID */
#define RGX_TILING_XSTRIDE7_HEAP_ID				13				/*!< RGX Tiling Heap (x-stride=7) ID */
#define RGX_HWBRN37200_HEAP_ID					14				/*!< RGX HWBRN37200 */

/* FIXME: work out what this ought to be.  In the old days it was
   typically bigger than it needed to be.  Is the correct thing
   "max + 1" ?? */
#if defined(FIX_HW_BRN_37200)
#define RGX_MAX_HEAP_ID     	(RGX_HWBRN37200_HEAP_ID + 1)		/*!< Max Valid Heap ID */
#else
#define RGX_MAX_HEAP_ID     	(RGX_TILING_XSTRIDE7_HEAP_ID + 1)		/*!< Max Valid Heap ID */
#endif

/*
  Identify heaps by their names
*/
#define RGX_GENERAL_HEAP_IDENT 			"General"               /*!< RGX General Heap Identifier */
#define RGX_PDSCODEDATA_HEAP_IDENT 		"PDS Code and Data"     /*!< RGX PDS Code/Data Heap Identifier */
#define RGX_3DPARAMETERS_HEAP_IDENT		"3DParameters"          /*!< RGX 3D Parameters Heap Identifier */
#define RGX_USCCODE_HEAP_IDENT			"USC Code"              /*!< RGX USC Code Heap Identifier */
#define RGX_TQ3DPARAMETERS_HEAP_IDENT	"TQ3DParameters"        /*!< RGX TQ 3D Parameters Heap Identifier */
#define RGX_TILING_XSTRIDE0_HEAP_IDENT	"Tiling Heap Stride 0"	/*!< RGX Tiling Heap (x-stride=0) ID */
#define RGX_TILING_XSTRIDE1_HEAP_IDENT	"Tiling Heap Stride 1"	/*!< RGX Tiling Heap (x-stride=1) ID */
#define RGX_TILING_XSTRIDE2_HEAP_IDENT	"Tiling Heap Stride 2"	/*!< RGX Tiling Heap (x-stride=2) ID */
#define RGX_TILING_XSTRIDE3_HEAP_IDENT	"Tiling Heap Stride 3"	/*!< RGX Tiling Heap (x-stride=3) ID */
#define RGX_TILING_XSTRIDE4_HEAP_IDENT	"Tiling Heap Stride 4"	/*!< RGX Tiling Heap (x-stride=4) ID */
#define RGX_TILING_XSTRIDE5_HEAP_IDENT	"Tiling Heap Stride 5"	/*!< RGX Tiling Heap (x-stride=5) ID */
#define RGX_TILING_XSTRIDE6_HEAP_IDENT	"Tiling Heap Stride 6"	/*!< RGX Tiling Heap (x-stride=6) ID */
#define RGX_TILING_XSTRIDE7_HEAP_IDENT	"Tiling Heap Stride 7"	/*!< RGX Tiling Heap (x-stride=7) ID */

#endif /* __RGX_HEAPS_H__ */

