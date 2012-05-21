									    /*************************************************************************//*!
									       @File
									       @Title          device configuration
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Memory heaps device specific configuration
									       @License        Strictly Confidential.
    *//**************************************************************************/

//#warning TODO:  add the MMU specialisation defines here (or in hwdefs, perhaps?)

#ifndef __RGXHEAPCONFIG_H__
#define __RGXHEAPCONFIG_H__

#include "rgxdefs.h"

#define DEV_DEVICE_TYPE			PVRSRV_DEVICE_TYPE_RGX
#define DEV_DEVICE_CLASS		PVRSRV_DEVICE_CLASS_3D

#define DEV_MAJOR_VERSION		1
#define DEV_MINOR_VERSION		0

/*      
	RGX Device Virtual Address Space Definitions:

	Notes:
	Base addresses have to be a multiple of 4MiB
	
	RGX_PDSCODEDATA_HEAP_BASE and RGX_USCCODE_HEAP_BASE will be programmed, on a
	global basis, into RGX_CR_PDS_EXEC_BASE and RGX_CR_USC_CODE_BASE_*
	respectively.
	Therefore if clients use multiple configs they must still be consistent with
	their definitions for these heaps.
*/

#if RGX_FEATURE_ADDRESS_SPACE_SIZE == 40

	/* Start at 128 Kb. Size of 256 Mb */
#define RGX_3DPARAMETERS_HEAP_BASE			IMG_UINT64_C(0x0000020000)
#define RGX_3DPARAMETERS_HEAP_SIZE			IMG_UINT64_C(0x0010000000)

	/* Start at 4GiB. Size of 512 GiB */
#define RGX_GENERAL_HEAP_BASE				IMG_UINT64_C(0x0100000000)
#define RGX_GENERAL_HEAP_SIZE				IMG_UINT64_C(0x8000000000)

	/* start at 516 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE0_HEAP_BASE		IMG_UINT64_C(0x8100000000)
#define RGX_TILING_XSTRIDE0_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 520 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE1_HEAP_BASE		IMG_UINT64_C(0x8200000000)
#define RGX_TILING_XSTRIDE1_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 524 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE2_HEAP_BASE		IMG_UINT64_C(0x8300000000)
#define RGX_TILING_XSTRIDE2_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 528 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE3_HEAP_BASE		IMG_UINT64_C(0x8400000000)
#define RGX_TILING_XSTRIDE3_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 532 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE4_HEAP_BASE		IMG_UINT64_C(0x8500000000)
#define RGX_TILING_XSTRIDE4_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 536 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE5_HEAP_BASE		IMG_UINT64_C(0x8600000000)
#define RGX_TILING_XSTRIDE5_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 540 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE6_HEAP_BASE		IMG_UINT64_C(0x8700000000)
#define RGX_TILING_XSTRIDE6_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* start at 544 GiB. Size of 4 GiB */
#define RGX_TILING_XSTRIDE7_HEAP_BASE		IMG_UINT64_C(0x8800000000)
#define RGX_TILING_XSTRIDE7_HEAP_SIZE		IMG_UINT64_C(0x0100000000)

	/* Start at 600GiB. Size of 4 GiB */
#define RGX_PDSCODEDATA_HEAP_BASE			IMG_UINT64_C(0x9600000000)
#define RGX_PDSCODEDATA_HEAP_SIZE			IMG_UINT64_C(0x0100000000)

	/* Start at 800GiB. Size of 4 GiB */
#define RGX_USCCODE_HEAP_BASE				IMG_UINT64_C(0xC800000000)
#define RGX_USCCODE_HEAP_SIZE				IMG_UINT64_C(0x0100000000)

	/* Start at 903GiB. Size of 4 GiB */
#define RGX_FIRMWARE_HEAP_BASE				IMG_UINT64_C(0xE1C0000000)
#define RGX_FIRMWARE_HEAP_SIZE				IMG_UINT64_C(0x0100000000)

	/* start at 907GiB. Size of 16 GiB */
#define RGX_TQ3DPARAMETERS_HEAP_BASE		IMG_UINT64_C(0xE400000000)
#define RGX_TQ3DPARAMETERS_HEAP_SIZE		IMG_UINT64_C(0x0400000000)

	/* signal we've identified the core by the build */
#define RGX_CORE_IDENTIFIED
#endif				/* RGX_FEATURE_ADDRESS_SPACE_SIZE == 40 */

#if !defined(RGX_CORE_IDENTIFIED)
#error "rgxheapconfig.h: ERROR: unspecified RGX Core version"
#endif

/* /\********************************************************************************* */
/*  * */
/*  * Heap overlap check */
/*  * */
/*  ********************************************************************************\/ */
/* #if defined(SUPPORT_RGX_GENERAL_MAPPING_HEAP) */
/* 	#if ((RGX_GENERAL_MAPPING_HEAP_BASE + RGX_GENERAL_MAPPING_HEAP_SIZE) >= RGX_GENERAL_HEAP_BASE) */
/* 		#error "rgxheapconfig.h: ERROR: RGX_GENERAL_MAPPING_HEAP overlaps RGX_GENERAL_HEAP" */
/* 	#endif */
/* #endif */

/* #if ((RGX_GENERAL_HEAP_BASE + RGX_GENERAL_HEAP_SIZE) >= RGX_3DPARAMETERS_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_GENERAL_HEAP overlaps RGX_3DPARAMETERS_HEAP" */
/* #endif */

/* #if ((RGX_3DPARAMETERS_HEAP_BASE + RGX_3DPARAMETERS_HEAP_SIZE) >= RGX_TADATA_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_3DPARAMETERS_HEAP overlaps RGX_TADATA_HEAP" */
/* #endif */

/* #if ((RGX_TADATA_HEAP_BASE + RGX_TADATA_HEAP_SIZE) >= RGX_SYNCINFO_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_TADATA_HEAP overlaps RGX_SYNCINFO_HEAP" */
/* #endif */

/* #if ((RGX_SYNCINFO_HEAP_BASE + RGX_SYNCINFO_HEAP_SIZE) >= RGX_PDSPIXEL_CODEDATA_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_SYNCINFO_HEAP overlaps RGX_PDSPIXEL_CODEDATA_HEAP" */
/* #endif */

/* #if ((RGX_PDSPIXEL_CODEDATA_HEAP_BASE + RGX_PDSPIXEL_CODEDATA_HEAP_SIZE) >= RGX_KERNEL_CODE_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_PDSPIXEL_CODEDATA_HEAP overlaps RGX_KERNEL_CODE_HEAP" */
/* #endif */

/* #if ((RGX_KERNEL_CODE_HEAP_BASE + RGX_KERNEL_CODE_HEAP_SIZE) >= RGX_PDSVERTEX_CODEDATA_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_KERNEL_CODE_HEAP overlaps RGX_PDSVERTEX_CODEDATA_HEAP" */
/* #endif */

/* #if ((RGX_PDSVERTEX_CODEDATA_HEAP_BASE + RGX_PDSVERTEX_CODEDATA_HEAP_SIZE) >= RGX_KERNEL_DATA_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_PDSVERTEX_CODEDATA_HEAP overlaps RGX_KERNEL_DATA_HEAP" */
/* #endif */

/* #if ((RGX_KERNEL_DATA_HEAP_BASE + RGX_KERNEL_DATA_HEAP_SIZE) >= RGX_PIXELSHADER_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_KERNEL_DATA_HEAP overlaps RGX_PIXELSHADER_HEAP" */
/* #endif */

/* #if ((RGX_PIXELSHADER_HEAP_BASE + RGX_PIXELSHADER_HEAP_SIZE) >= RGX_VERTEXSHADER_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_PIXELSHADER_HEAP overlaps RGX_VERTEXSHADER_HEAP" */
/* #endif */

/* #if ((RGX_VERTEXSHADER_HEAP_BASE + RGX_VERTEXSHADER_HEAP_SIZE) < RGX_VERTEXSHADER_HEAP_BASE) */
/* 	#error "rgxheapconfig.h: ERROR: RGX_VERTEXSHADER_HEAP_BASE size cause wraparound" */
/* #endif */

#endif				/* __RGXHEAPCONFIG_H__ */

/*****************************************************************************
 End of file (rgxheapconfig.h)
*****************************************************************************/
