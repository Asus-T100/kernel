/*************************************************************************/ /*!
@File			rgx_fwif.h
@Title          RGX firmware interface structures
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface structures used by srvinit and server
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

#if !defined (__RGX_FWIF_H__)
#define __RGX_FWIF_H__

#include "rgx_meta.h"
#include "rgx_fwif_shared.h"

#include "pvr_tlcommon.h"
#include "rgx_hwperf.h"

/*************************************************************************/ /*!
 Logging type
*/ /**************************************************************************/
#define RGXFWIF_LOG_TYPE_NONE			 0x00000000
#define RGXFWIF_LOG_TYPE_TRACE			 0x00000001
#define RGXFWIF_LOG_TYPE_GROUP_MAIN		 0x00000002
#define RGXFWIF_LOG_TYPE_GROUP_MTS		 0x00000004
#define RGXFWIF_LOG_TYPE_GROUP_CLEANUP	 0x00000008
#define RGXFWIF_LOG_TYPE_GROUP_CSW		 0x00000010
#define RGXFWIF_LOG_TYPE_GROUP_BIF		 0x00000020
#define RGXFWIF_LOG_TYPE_GROUP_PM		 0x00000040
#define RGXFWIF_LOG_TYPE_GROUP_RTD		 0x00000080
#define RGXFWIF_LOG_TYPE_GROUP_SPM		 0x00000100
#define RGXFWIF_LOG_TYPE_GROUP_POW		 0x00000200
#define RGXFWIF_LOG_TYPE_GROUP_MASK		 0x000003FE
#define RGXFWIF_LOG_TYPE_MASK			 0x000003FF

#define RGXFWIF_LOG_ENABLED_GROUPS_LIST(types)	(((types) & RGXFWIF_LOG_TYPE_GROUP_MAIN)	?("main ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_MTS)		?("mts ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_CLEANUP)	?("cleanup ")	:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_CSW)		?("csw ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_BIF)		?("bif ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_PM)		?("pm ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_RTD)		?("rtd ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_SPM)		?("spm ")		:("")),		\
												(((types) & RGXFWIF_LOG_TYPE_GROUP_POW)		?("pow ")		:(""))

/*! Logging function */
typedef IMG_VOID (*PFN_RGXFW_LOG) (const IMG_CHAR* pszFmt, ...);

/*!
 ******************************************************************************
 * HWPERF
 *****************************************************************************/
#define RGXFW_HWPERF_FIRMWARE_COUNT (2048) /* Must be 2^N */
#define RGXFW_HWPERF_FIRMWARE_COUNT_MASK (RGXFW_HWPERF_FIRMWARE_COUNT - 1)

// Packet size has increased to ~224 bytes with HW perf counters, which is
// 18 packets per 4Kb page, so 256 is ~ 15 pages ~ 62Kb and 1024 is ~ 250Kb
#define RGXFW_HWPERF_SERVER_COUNT (1024)


/*!
 ******************************************************************************
 * Trace Buffer
 *****************************************************************************/

/*! Number of elements on each line when dumping the trace buffer */
#define RGXFW_TRACE_BUFFER_LINESIZE	(30)

/*! Total size of RGXFWIF_TRACEBUF dword (needs to be a multiple of RGXFW_TRACE_BUFFER_LINESIZE) */
#define RGXFW_TRACE_BUFFER_SIZE		(400*RGXFW_TRACE_BUFFER_LINESIZE)
#define RGXFW_TRACE_BUFFER_ASSERT_SIZE 200
#define RGXFW_THREAD_NUM 2

#define RGXFW_POLL_TYPE_SET 0x80000000

typedef struct _RGXFWIF_ASSERTBUF_
{
	IMG_CHAR	szPath[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
	IMG_CHAR	szInfo[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
	IMG_UINT32	ui32LineNum;
}RGXFWIF_ASSERTBUF;

typedef struct _RGXFWIF_TRACEBUF_SPACE_
{
	IMG_UINT32			ui32TracePointer;
	IMG_UINT32			aui32TraceBuffer[RGXFW_TRACE_BUFFER_SIZE];
	RGXFWIF_ASSERTBUF	sAssertBuf;
} RGXFWIF_TRACEBUF_SPACE;

#define RGXFWIF_POW_STATES \
  X(RGXFWIF_APM_OFF)			/* idle and handshaked with the host (ready to full power down) */ \
  X(RGXFWIF_APM_ON)				/* running HW mds */ \
  X(RGXFWIF_APM_IDLE)			/* idle waiting for host handshake */

typedef enum _RGXFWIF_POW_STATE_
{
#define X(NAME) NAME,
	RGXFWIF_POW_STATES
#undef X
} RGXFWIF_POW_STATE;

typedef struct _RGXFWIF_TRACEBUF_
{
	IMG_UINT32				ui32LogType;
	RGXFWIF_POW_STATE		ePowState;
	RGXFWIF_TRACEBUF_SPACE	sTraceBuf[RGXFW_THREAD_NUM];

	IMG_UINT16			aui16HwrDmLockedUpCount[RGXFWIF_HWDM_MAX];
	IMG_UINT16			aui16HwrDmRecoveredCount[RGXFWIF_HWDM_MAX];

	IMG_UINT32			aui32CrPollAddr[RGXFW_THREAD_NUM];
	IMG_UINT32			aui32CrPollValue[RGXFW_THREAD_NUM];

	IMG_VOID			*apsHwrDmFWCommonContext[RGXFWIF_HWDM_MAX];

	IMG_UINT32			ui32HWPerfRIdx;
	IMG_UINT32			ui32HWPerfWIdx;
	IMG_UINT32			ui32HWPerfOrdinal;
	RGX_HWPERF_PACKET	RGXFW_ALIGN asHWPerfPackets[RGXFW_HWPERF_FIRMWARE_COUNT];
} RGXFWIF_TRACEBUF;

/*! RGX firmware Init Config Data */
#define RGXFWIF_INICFG_CTXSWITCH_TA_EN		(0x1 << 0)
#define RGXFWIF_INICFG_CTXSWITCH_3D_EN		(0x1 << 1)
#define RGXFWIF_INICFG_CTXSWITCH_CDM_EN		(0x1 << 2)
#define RGXFWIF_INICFG_CTXSWITCH_MODE_RAND	(0x1 << 3)
#define RGXFWIF_INICFG_CTXSWITCH_SRESET_EN	(0x1 << 4)
#define RGXFWIF_INICFG_2ND_THREAD_EN		(0x1 << 5)
#define RGXFWIF_INICFG_POW_RASCALDUST		(0x1 << 6)
#define RGXFWIF_INICFG_HWPERF_EN			(0x1 << 7)
#define RGXFWIF_INICFG_ALL					(0x000000FFU)

#define RGXFWIF_INICFG_CTXSWITCH_DM_ALL		(RGXFWIF_INICFG_CTXSWITCH_TA_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_3D_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_CDM_EN)

#define RGXFWIF_INICFG_CTXSWITCH_CLRMSK		~(RGXFWIF_INICFG_CTXSWITCH_DM_ALL | \
											 RGXFWIF_INICFG_CTXSWITCH_MODE_RAND | \
											 RGXFWIF_INICFG_CTXSWITCH_SRESET_EN)

typedef enum
{
	RGX_ACTIVEPM_FORCE_OFF = 0,
	RGX_ACTIVEPM_FORCE_ON = 1,
	RGX_ACTIVEPM_DEFAULT = 3
} RGX_ACTIVEPM_CONF;

/*!
 ******************************************************************************
 * Querying DM state
 *****************************************************************************/

typedef enum _RGXFWIF_DM_STATE_
{
	RGXFWIF_DM_STATE_NORMAL			= 0,
	RGXFWIF_DM_STATE_LOCKEDUP		= 1,

} RGXFWIF_DM_STATE;

#endif /*  __RGX_FWIF_H__ */

/******************************************************************************
 End of file (rgx_fwif.h)
******************************************************************************/

