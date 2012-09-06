									    /*************************************************************************//*!
									       @Title          Rogue hw definitions (kernel mode)
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _RGXDEFS_KM_H_
#define _RGXDEFS_KM_H_

#include "rgxcore_1.14.4.4.h"

#define RGX_BVNC_B 1
#define RGX_BVNC_V 14
#define RGX_BVNC_N 4
#define RGX_BVNC_C 4

/******************************************************************************
 * Check for valid B.X.N.C
 *****************************************************************************/
#if !defined(RGX_BVNC_B) || !defined(RGX_BVNC_N) || !defined(RGX_BVNC_C)
#error "Need to specify BVNC (RGX_BVNC_B, RGX_BVNC_N and RGX_BVNC_C). Omitting RGX_BVNC_V means V=Head"
#endif

/******************************************************************************
 * RGX Version name
 *****************************************************************************/
#define _RGX_BVNC_ST2(S)	#S
#define _RGX_BVNC_ST(S)	_RGX_BVNC_ST2(S)

#if !defined(RGX_BVNC_V)
#define RGX_BVNC	_RGX_BVNC_ST(RGX_BVNC_B) ".X." _RGX_BVNC_ST(RGX_BVNC_N) "." _RGX_BVNC_ST(RGX_BVNC_C)
#else
#define RGX_BVNC	_RGX_BVNC_ST(RGX_BVNC_B) "." _RGX_BVNC_ST(RGX_BVNC_V) "." _RGX_BVNC_ST(RGX_BVNC_N) "." _RGX_BVNC_ST(RGX_BVNC_C)
#endif

#endif
