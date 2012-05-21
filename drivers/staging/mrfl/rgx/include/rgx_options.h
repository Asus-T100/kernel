									    /*************************************************************************//*!
									       @File
									       @Title          RGX feature options
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

/* Each build option listed here is packed into a dword which
 * provides up to 32 flags (or up to 28 flags plus a numeric
 * value in the range 0-15 which corresponds to the number of
 * cores minus one if RGX_FEATURE_MP is defined). The corresponding
 * bit is set if the build option was enabled at compile time.
 *
 * In order to extract the enabled build flags the INTERNAL_TEST
 * switch should be enabled in a client program which includes this
 * header. Then the client can test specific build flags by reading
 * the bit value at ##OPTIONNAME##_SET_OFFSET in RGX_BUILD_OPTIONS.
 *
 * IMPORTANT: add new options to unused bits or define a new dword
 * (e.g. RGX_BUILD_OPTIONS2) so that the bitfield remains backwards
 * compatible.
 */

#if defined(DEBUG) || defined (INTERNAL_TEST)
#define DEBUG_SET_OFFSET	OPTIONS_BIT0
#define OPTIONS_BIT0		0x1
#else
#define OPTIONS_BIT0		0x0
#endif				/* DEBUG */

#if defined(PDUMP) || defined (INTERNAL_TEST)
#define PDUMP_SET_OFFSET	OPTIONS_BIT1
#define OPTIONS_BIT1		(0x1 << 1)
#else
#define OPTIONS_BIT1		0x0
#endif				/* PDUMP */

#if defined(PVRSRV_USSE_EDM_STATUS_DEBUG) || defined (INTERNAL_TEST)
#define PVRSRV_USSE_EDM_STATUS_DEBUG_SET_OFFSET		OPTIONS_BIT2
#define OPTIONS_BIT2		(0x1 << 2)
#else
#define OPTIONS_BIT2		0x0
#endif				/* PVRSRV_USSE_EDM_STATUS_DEBUG */

#if defined(SUPPORT_HW_RECOVERY) || defined (INTERNAL_TEST)
#define SUPPORT_HW_RECOVERY_SET_OFFSET	OPTIONS_BIT3
#define OPTIONS_BIT3		(0x1 << 3)
#else
#define OPTIONS_BIT3		0x0
#endif				/* SUPPORT_HW_RECOVERY */

#if defined(PVR_SECURE_HANDLES) || defined (INTERNAL_TEST)
#define PVR_SECURE_HANDLES_SET_OFFSET	OPTIONS_BIT4
#define OPTIONS_BIT4		(0x1 << 4)
#else
#define OPTIONS_BIT4		0x0
#endif				/* PVR_SECURE_HANDLES */

#if defined(RGX_DMS_AGE_ENABLE) || defined (INTERNAL_TEST)
#define RGX_DMS_AGE_ENABLE_SET_OFFSET	OPTIONS_BIT6
#define OPTIONS_BIT6		(0x1 << 6)
#else
#define OPTIONS_BIT6		0x0
#endif				/* RGX_DMS_AGE_ENABLE */

#if defined(RGX_FAST_DPM_INIT) || defined (INTERNAL_TEST)
#define RGX_FAST_DPM_INIT_SET_OFFSET	OPTIONS_BIT8
#define OPTIONS_BIT8		(0x1 << 8)
#else
#define OPTIONS_BIT8		0x0
#endif				/* RGX_FAST_DPM_INIT */

#if defined(RGX_FEATURE_MP) || defined (INTERNAL_TEST)
#define RGX_FEATURE_MP_SET_OFFSET	OPTIONS_BIT10
#define OPTIONS_BIT10		(0x1 << 10)
#else
#define OPTIONS_BIT10		0x0
#endif				/* RGX_FEATURE_MP */

#if defined(RGX_FEATURE_SYSTEM_CACHE) || defined (INTERNAL_TEST)
#define RGX_FEATURE_SYSTEM_CACHE_SET_OFFSET	OPTIONS_BIT13
#define OPTIONS_BIT13		(0x1 << 13)
#else
#define OPTIONS_BIT13		0x0
#endif				/* RGX_FEATURE_SYSTEM_CACHE */

#if defined(SUPPORT_DISPLAYCONTROLLER_TILING) || defined (INTERNAL_TEST)
#define SUPPORT_DISPLAYCONTROLLER_TILING_SET_OFFSET	OPTIONS_BIT16
#define OPTIONS_BIT16		(0x1 << 16)
#else
#define OPTIONS_BIT16		0x0
#endif				/* SUPPORT_DISPLAYCONTROLLER_TILING */

#if defined(SUPPORT_PERCONTEXT_PB) || defined (INTERNAL_TEST)
#define SUPPORT_PERCONTEXT_PB_SET_OFFSET	OPTIONS_BIT17
#define OPTIONS_BIT17		(0x1 << 17)
#else
#define OPTIONS_BIT17		0x0
#endif				/* SUPPORT_PERCONTEXT_PB */

/* GAP OPTIONS_BIT18 */

#if defined(SUPPORT_RGX_MMU_DUMMY_PAGE) || defined (INTERNAL_TEST)
#define SUPPORT_RGX_MMU_DUMMY_PAGE_SET_OFFSET	OPTIONS_BIT19
#define OPTIONS_BIT19		(0x1 << 19)
#else
#define OPTIONS_BIT19		0x0
#endif				/* SUPPORT_RGX_MMU_DUMMY_PAGE */

#if defined(SUPPORT_RGX_LOW_LATENCY_SCHEDULING) || defined (INTERNAL_TEST)
#define SUPPORT_RGX_LOW_LATENCY_SCHEDULING_SET_OFFSET	OPTIONS_BIT21
#define OPTIONS_BIT21		(0x1 << 21)
#else
#define OPTIONS_BIT21		0x0
#endif				/* SUPPORT_RGX_LOW_LATENCY_SCHEDULING */

#if defined(USE_SUPPORT_NO_TA3D_OVERLAP) || defined (INTERNAL_TEST)
#define USE_SUPPORT_NO_TA3D_OVERLAP_SET_OFFSET	OPTIONS_BIT22
#define OPTIONS_BIT22		(0x1 << 22)
#else
#define OPTIONS_BIT22		0x0
#endif				/* USE_SUPPORT_NO_TA3D_OVERLAP */

#if defined(RGX_FEATURE_MP) || defined (INTERNAL_TEST)
#define OPTIONS_HIGHBYTE ((RGX_FEATURE_MP_CORE_COUNT-1) << RGX_FEATURE_MP_CORE_COUNT_SET_OFFSET)
#define RGX_FEATURE_MP_CORE_COUNT_SET_OFFSET	28UL
#define RGX_FEATURE_MP_CORE_COUNT_SET_MASK		0xFF
#else
#define OPTIONS_HIGHBYTE	0x0
#endif				/* RGX_FEATURE_MP */

#define RGX_BUILD_OPTIONS	\
	OPTIONS_BIT0 |\
	OPTIONS_BIT1 |\
	OPTIONS_BIT2 |\
	OPTIONS_BIT3 |\
	OPTIONS_BIT4 |\
	OPTIONS_BIT6 |\
	OPTIONS_BIT8 |\
	OPTIONS_BIT10 |\
	OPTIONS_BIT13 |\
	OPTIONS_BIT16 |\
	OPTIONS_BIT17 |\
	OPTIONS_BIT19 |\
	OPTIONS_BIT21 |\
	OPTIONS_HIGHBYTE
