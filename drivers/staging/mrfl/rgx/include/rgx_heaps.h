									    /*************************************************************************//*!
									       @File
									       @Title          RGX heap definitions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGX_HEAPS_H__)
#define __RGX_HEAPS_H__

/* RGX Heap IDs, note: not all heaps are available to clients */
/* N.B.  Old heap identifiers are deprecated now that the old memory
   management is. New heap identifiers should be suitably renamed */
#define RGX_UNDEFINED_HEAP_ID					(~0LU)	/*!< RGX Undefined Heap ID */
#define RGX_GENERAL_HEAP_ID						0	/*!< RGX General Heap ID */
#define RGX_PDSCODEDATA_HEAP_ID					1	/*!< RGX PDS Code/Data Heap ID */
#define RGX_3DPARAMETERS_HEAP_ID				2	/*!< RGX 3D Parameters Heap ID */
#define RGX_USCCODE_HEAP_ID						3	/*!< RGX USC Code Heap ID */
#define RGX_FIRMWARE_HEAP_ID					4	/*!< RGX Firmware Heap ID */
#define RGX_TQ3DPARAMETERS_HEAP_ID				5	/*!< RGX Firmware Heap ID */
#define RGX_TILING_XSTRIDE0_HEAP_ID				6	/*!< RGX Tiling Heap (x-stride=0) ID */
#define RGX_TILING_XSTRIDE1_HEAP_ID				7	/*!< RGX Tiling Heap (x-stride=1) ID */
#define RGX_TILING_XSTRIDE2_HEAP_ID				8	/*!< RGX Tiling Heap (x-stride=2) ID */
#define RGX_TILING_XSTRIDE3_HEAP_ID				9	/*!< RGX Tiling Heap (x-stride=3) ID */
#define RGX_TILING_XSTRIDE4_HEAP_ID				10	/*!< RGX Tiling Heap (x-stride=4) ID */
#define RGX_TILING_XSTRIDE5_HEAP_ID				11	/*!< RGX Tiling Heap (x-stride=5) ID */
#define RGX_TILING_XSTRIDE6_HEAP_ID				12	/*!< RGX Tiling Heap (x-stride=6) ID */
#define RGX_TILING_XSTRIDE7_HEAP_ID				13	/*!< RGX Tiling Heap (x-stride=7) ID */

/* FIXME: work out what this ought to be.  In the old days it was
   typically bigger than it needed to be.  Is the correct thing
   "max + 1" ?? */
#define RGX_MAX_HEAP_ID     	(RGX_TILING_XSTRIDE7_HEAP_ID + 1)	/*!< Max Valid Heap ID */

/*
  Identify heaps by their names
*/
#define RGX_GENERAL_HEAP_IDENT 			"General"	/*!< RGX General Heap Identifier */
#define RGX_PDSCODEDATA_HEAP_IDENT 		"PDS Code and Data"	/*!< RGX PDS Code/Data Heap Identifier */
#define RGX_3DPARAMETERS_HEAP_IDENT		"3DParameters"	/*!< RGX 3D Parameters Heap Identifier */
#define RGX_USCCODE_HEAP_IDENT			"USC Code"	/*!< RGX USC Code Heap Identifier */
#define RGX_TQ3DPARAMETERS_HEAP_IDENT	"TQ3DParameters"	/*!< RGX TQ 3D Parameters Heap Identifier */
#define RGX_TILING_XSTRIDE0_HEAP_IDENT	"Tiling Heap Stride 0"	/*!< RGX Tiling Heap (x-stride=0) ID */
#define RGX_TILING_XSTRIDE1_HEAP_IDENT	"Tiling Heap Stride 1"	/*!< RGX Tiling Heap (x-stride=1) ID */
#define RGX_TILING_XSTRIDE2_HEAP_IDENT	"Tiling Heap Stride 2"	/*!< RGX Tiling Heap (x-stride=2) ID */
#define RGX_TILING_XSTRIDE3_HEAP_IDENT	"Tiling Heap Stride 3"	/*!< RGX Tiling Heap (x-stride=3) ID */
#define RGX_TILING_XSTRIDE4_HEAP_IDENT	"Tiling Heap Stride 4"	/*!< RGX Tiling Heap (x-stride=4) ID */
#define RGX_TILING_XSTRIDE5_HEAP_IDENT	"Tiling Heap Stride 5"	/*!< RGX Tiling Heap (x-stride=5) ID */
#define RGX_TILING_XSTRIDE6_HEAP_IDENT	"Tiling Heap Stride 6"	/*!< RGX Tiling Heap (x-stride=6) ID */
#define RGX_TILING_XSTRIDE7_HEAP_IDENT	"Tiling Heap Stride 7"	/*!< RGX Tiling Heap (x-stride=7) ID */

#endif				/* __RGX_HEAPS_H__ */
