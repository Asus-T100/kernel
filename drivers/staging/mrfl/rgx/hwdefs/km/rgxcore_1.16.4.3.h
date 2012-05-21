									    /*************************************************************************//*!
									       @Title          RGX Config B 1 V 16 N 4 C 3
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _RGXCONFIG_1_16_4_3_H_
#define _RGXCONFIG_1_16_4_3_H_

/***** Automatically generated file: Do not edit manually *****/

/******************************************************************************
 * BVNC = 1.16.4.3 
 *****************************************************************************/
#define RGX_BVNC_B 1
#define RGX_BVNC_V 16
#define RGX_BVNC_N 4
#define RGX_BVNC_C 3

/******************************************************************************
 * Errata 
 *****************************************************************************/
/* workraround in hwresourcecalc.c */
#define FIX_HW_BRN_35772

/* workaround in usc2 */
#define FIX_HW_BRN_36486

/* workaround in usc2 */
#define FIX_HW_BRN_36419

#define FIX_HW_BRN_36411

/******************************************************************************
 * HW defines 
 *****************************************************************************/
#define RGX_FEATURE_USC_NUM_INTERNAL_REGISTERS (4)
#define RGX_FEATURE_CEMFACE_4K_ALIGN
#define RGX_FEATURE_USC_F16_SOP
#define RGX_SLC_CACHE_LINE_SIZE_BITS (512)
#define RGX_MAX_TILES_IN_FLIGHT (3)

/******************************************************************************
 * SW defines 
 *****************************************************************************/
#define RGX_FEATURE_TEXTURES_BYPASS_SLC
#define RGX_FEATURE_BYPASS_SLC_COMBINER
#define RGX_MAX_TILES_IN_FLIGHT (3)
#define RGX_FEATURE_ADDRESS_SPACE_SIZE (40)

#endif				/* _RGXCONFIG_1_16_4_3_H_ */
