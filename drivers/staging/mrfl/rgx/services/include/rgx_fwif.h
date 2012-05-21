									    /*************************************************************************//*!
									       @File
									       @Title          RGX firmware interface structures
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX firmware interface structures used by srvinit and server
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__RGX_FWIF_H__)
#define __RGX_FWIF_H__

#include "rgx_meta.h"

/************************************************************************
* RGX FW signature checks
************************************************************************/
#define RGXFW_SIG_BUFFER_SIZE_DEFAULT		(1024)
#if defined(DEBUG) || defined(PDUMP)
#define RGXFW_SIG_CHECKS_ENABLED_DEFAULT	(IMG_TRUE)
#else
#define RGXFW_SIG_CHECKS_ENABLED_DEFAULT	(IMG_FALSE)
#endif				/* DEBUG */

#define RGXFW_SIG_TA_CHECKS						\
				RGX_CR_USC_UVS0_CHECKSUM,		\
				RGX_CR_USC_UVS1_CHECKSUM,		\
				RGX_CR_USC_UVS2_CHECKSUM,		\
				RGX_CR_USC_UVS3_CHECKSUM,		\
				RGX_CR_PPP_SIGNATURE,			\
				RGX_CR_TE_SIGNATURE,			\
				RGX_CR_VCE_CHECKSUM

#define RGXFW_SIG_3D_CHECKS						\
				RGX_CR_ISP_PDS_CHECKSUM,		\
				RGX_CR_ISP_TPF_CHECKSUM,		\
				RGX_CR_TFPU_PLANE0_CHECKSUM,	\
				RGX_CR_TFPU_PLANE1_CHECKSUM,	\
				RGX_CR_PBE_CHECKSUM,			\
				RGX_CR_IFPU_ISP_CHECKSUM

/*!
 ******************************************************************************
 * Logging type
 *****************************************************************************/
typedef enum _RGXFWIF_LOG_TYPE_ {
	RGXFWIF_LOG_TYPE_NONE = 0,
	RGXFWIF_LOG_TYPE_TBI = 1,
	RGXFWIF_LOG_TYPE_TRACE = 2,
	RGXFWIF_LOG_TYPE_NUM
} RGXFWIF_LOG_TYPE;

/*! Logging function */
typedef IMG_VOID(*PFN_RGXFW_LOG) (IMG_CONST IMG_CHAR * pszFmt, ...);

/*!
 ******************************************************************************
 * Trace Buffer
 *****************************************************************************/

/*! Number of elements on each line when dumping the trace buffer */
#define RGXFW_TRACE_BUFFER_LINESIZE	(20)

/*! Total size of RGXFWIF_TRACEBUF dword (needs to be a multiple of RGXFW_TRACE_BUFFER_LINESIZE) */
#define RGXFW_TRACE_BUFFER_SIZE		(100*RGXFW_TRACE_BUFFER_LINESIZE)

typedef struct _RGXFWIF_TRACEBUF_ {
	RGXFWIF_LOG_TYPE eLogType;

	IMG_UINT32 ui32TracePointer;
	IMG_UINT32 aui32TraceBuffer[RGXFW_TRACE_BUFFER_SIZE];
} RGXFWIF_TRACEBUF;

/*! RGX firmware Init Config Data */
#define RGXFWIF_INICFG_CTXSWITCH_TA_EN		(0x1 << 0)
#define RGXFWIF_INICFG_CTXSWITCH_3D_EN		(0x1 << 1)
#define RGXFWIF_INICFG_CTXSWITCH_CDM_EN		(0x1 << 2)
#define RGXFWIF_INICFG_CTXSWITCH_MODE_RAND	(0x1 << 3)
#define RGXFWIF_INICFG_CTXSWITCH_SRESET_EN	(0x1 << 4)
#define RGXFWIF_INICFG_2ND_THREAD_EN		(0x1 << 5)
#define RGXFWIF_INICFG_POW_RASCALDUST		(0x1 << 6)
#define RGXFWIF_INICFG_ALL					(0x0000007FU)

#define RGXFWIF_INICFG_CTXSWITCH_DM_ALL		(RGXFWIF_INICFG_CTXSWITCH_TA_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_3D_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_CDM_EN)

#define RGXFWIF_INICFG_CTXSWITCH_CLRMSK		~(RGXFWIF_INICFG_CTXSWITCH_DM_ALL | \
											 RGXFWIF_INICFG_CTXSWITCH_MODE_RAND | \
											 RGXFWIF_INICFG_CTXSWITCH_SRESET_EN)

#endif				/*  __RGX_FWIF_H__ */

/******************************************************************************
 End of file (rgx_fwif.h)
******************************************************************************/
