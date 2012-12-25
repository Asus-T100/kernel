/*************************************************************************/ /*!
@File
@Title          RGX HWPerf Types and Defines Header (Performance profiling)
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description	Common data types definitions for hardware performance
				profiling used in user-side, kernel	and firmware builds
				environments.
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

#ifndef __RGX_HWPERF_H__
#define __RGX_HWPERF_H__

#if defined (__cplusplus)
extern "C" {
#endif


/******************************************************************************
 * 	Includes and Defines
 *****************************************************************************/

#include "img_types.h"
#include "img_defs.h"

/* Included to get the BVNC_N defined always */
#include "km/rgxdefs_km.h"


/*! Macro used to check structure size and alignment at compile time.
 */
#define BLD_ASSERT(expr, file) _impl_ASSERT_LINE(expr,__LINE__,file)
#define _impl_JOIN(a,b) a##b
#define _impl_ASSERT_LINE(expr, line, file) \
	typedef char _impl_JOIN(build_assertion_failed_##file##_,line)[2*!!(expr)-1];


/*! Defines the number of performance counter blocks that are directly
 * addressable in the RGX register map. Should always be even to aid
 * structure alignment.
 */
#define RGX_HWPERF_DIRECT_ADDR_BLKS 4

/* HWPerf interface assumption checks */
BLD_ASSERT((RGX_BVNC_N<=8), rgx_hwperf_h)

/*! The number of indirectly addressable TPU_MSC blocks in the GPU */
#define RGX_HWPERF_INDIRECT_TPU_BLKS MAX((RGX_BVNC_N>>1),1)

/*! The number of indirectly addressable USC blocks in the GPU */
#define RGX_HWPERF_INDIRECT_USC_BLKS (RGX_BVNC_N)

/*! The number of layout blocks in the GPU with configurable performance counters */
#define RGX_HWPERF_MAX_BLKS 	(RGX_HWPERF_DIRECT_ADDR_BLKS+\
								RGX_HWPERF_INDIRECT_TPU_BLKS+\
								RGX_HWPERF_INDIRECT_USC_BLKS)

/*! The number of performance counters in each layout block */
#define RGX_HWPERF_CNTRS_IN_BLK	4

/*! The total number of counters in the GPU */
#define RGX_HWPERF_MAX_CNTRS 	(RGX_HWPERF_MAX_BLKS*\
								 RGX_HWPERF_CNTRS_IN_BLK)

/*! Defines the maximum stream size which is # header + # counters for all blocks */
#define RGX_HWPERF_MAX_STREAM 	(RGX_HWPERF_MAX_BLKS+\
										 RGX_HWPERF_MAX_CNTRS)


/******************************************************************************
 * 	Data Stream Types
 *****************************************************************************/

/*! Type used to encode the event that generated the HW performance packet.
 */
typedef enum
{
	RGX_HWPERF_INVALID				= 0x00,
	/* fw types */
	RGX_HWPERF_FW_BGSTART			= 0x01,
	RGX_HWPERF_FW_BGEND				= 0x02,

	RGX_HWPERF_FW_IRQSTART			= 0x03,
	RGX_HWPERF_FW_IRQEND			= 0x04,

	/* hw types */
	RGX_HWPERF_HW_TAKICK			= 0x08,
	RGX_HWPERF_HW_TAFINISHED		= 0x09,

	RGX_HWPERF_HW_3DTQKICK			= 0x0A,
	RGX_HWPERF_HW_3DKICK			= 0x0B,
	RGX_HWPERF_HW_3DFINISHED		= 0x0C,

	RGX_HWPERF_HW_CDMKICK			= 0x0D,
	RGX_HWPERF_HW_CDMFINISHED		= 0x0E,

	RGX_HWPERF_HW_TLAKICK			= 0x0F,
	RGX_HWPERF_HW_TLAFINISHED		= 0x10,

	/* Future other packet event types */
/*	RGX_HWPERF_PWR_ON				= 0x18, */
/*	RGX_HWPERF_PWR_OFF				= 0x19, */
/*  RGX_HWPERF_CLKS_CHG				= 0x1A, */

/*	RGX_HWPERF_PERIODIC				= 0x1B, */
/*	RGX_HWPERF_VSYNC				= 0x1C, */

	/* last */
	RGX_HWPERF_LAST_TYPE

} RGX_HWPERF_EVENT_TYPE;

#define RGX_HWPERF_EVENT_ALL	0xFFFFFFFFU


/* The event type values are incrementing integers for use as a shift ordinal
 * in the event filtering process at the point events are generated.
 * This scheme thus implies a limit of 31 event types at present but could be
 * expanded to 64 if needed.
 */
BLD_ASSERT((RGX_HWPERF_LAST_TYPE<=32), rgx_hwperf_h)


/*! The master definition for data masters known to the firmware of RGX */
typedef enum _RGXFWIF_DM_
{
	RGXFWIF_DM_TA			= 0,
	RGXFWIF_DM_3D			= 1,
	RGXFWIF_DM_CDM			= 2,
	RGXFWIF_DM_2D			= 3,
	RGXFWIF_DM_GP			= 4,
	RGXFWIF_DM_UNUSED		= 5,
	RGXFWIF_DM_MAX_MTS		= 6,

	RGXFWIF_DM_LAST,

	RGXFWIF_DM_FORCE_I32  = -1,

} RGXFWIF_DM;

/*! This type records in the packet which RGX data master the HW event relates to. */
typedef RGXFWIF_DM RGX_HWPERF_DM;


/*! This structure holds the data of a FirmWare packet. */
typedef struct
{
	RGX_HWPERF_DM	eDM;					/*!< DataMaster identifier, see RGX_HWPERF_DM */
	IMG_UINT32		ui32TxtActCyc;          /*!< Meta TXTACTCYC register value */
	IMG_UINT32		ui32METAPerfCount0;     /*!< Meta PERF_COUNT0 register */
	IMG_UINT32		ui32METAPerfCount1;     /*!< Meta PERF_COUNT1 register */
} RGX_HWPERF_FW_DATA;


/*! This structure holds the data of a HardWare packet, including counters */
typedef struct
{
	IMG_UINT32	ui32DMCyc;		/*!< DataMaster cycle count register, 0 if none */
	IMG_UINT32  ui32NumBlocks;	/*!< Number of counter blocks in the
	                                 aui32CountBlksStream member, there may be
	                                 0 or more blocks present */

	/*! Counter stream data containing just the counters that are enabled.
     *
	 * The counter stream contains a counter block for each block that is
	 * enabled in the hardware. Each counter block has an ID to identify the
	 * HW block and counter bits which describe whether there are 1, 2, 3 or 4
	 * counters in the stream. The format of the stream is:
	 * 	[<counter_block> ...]
	 *
	 * The format of the counter block encoding is (maintaining dword alignment):
	 * 	<16bit> :		 The block ID, see RGX_HWPERF_CNTBLK_ID
	 *	<16bit> :    	 The 4 LSBs used to indicate which counters 0..3 follow
	 *              	 in the stream
	 * 	<32bit> x 1..4 : The enabled counter values in ascending counter order
	 *
	 * Examples of a counter blocks:
	 *		| RGX_CNTBLK_ID_TPU_MCU0, 0x5, <counter0>, <counter2> |
	 *		| RGX_CNTBLK_ID_TA, 0x8, <counter3> |
	 *		| RGX_CNTBLK_ID_USC1, 0xf, <counter0>, <counter1>, <counter2>, <counter3>  |
	 *
	 * The array is large enough for all counter blocks that are present
	 * for a given BVNC_N value the driver is built for to maintain a fixed
	 * packet size. Size handles odd stream lengths to get correct alignment.
	 */
	IMG_UINT32	aui32CountBlksStream[RGX_HWPERF_MAX_STREAM+(RGX_HWPERF_MAX_STREAM%2)];

} RGX_HWPERF_HW_DATA;

/* RGX_HWPERF_HW_DATA must be a size that is a multiple of 64bits, 8bytes
 * for use on Meta cores in the GPU as well as CPU
 */
BLD_ASSERT((sizeof(RGX_HWPERF_HW_DATA)&0x07)==0, rgx_hwperf_h)


/*! The structure for a HWPerf packet with header info and data pay load. */
typedef struct
{
	IMG_UINT16	eType;			/*!< See RGX_HWPERF_EVENT_TYPE */
	IMG_UINT16	ui16MetaThread;	/*!< META Core thread number */
	IMG_UINT32	ui32Ordinal;    /*!< Sequential number of the packet */
	IMG_UINT64	ui64RGXTimer; 	/*!< Value of RGX_CR_TIMER at event */
	union
	{
		RGX_HWPERF_FW_DATA sFW; /*!< FirmWare event packet data */
		RGX_HWPERF_HW_DATA sHW; /*!< HardWare event packet data */
	} data;						/*!< Packet pay load */
} RGX_HWPERF_PACKET;

/* ui64RGXTimer must be on an 8 byte boundary for use of Meta cores in GPU
 */
BLD_ASSERT((offsetof(RGX_HWPERF_PACKET, ui64RGXTimer)&0x7)==0, rgx_hwperf_h)

/* RGX_HWPERF_PACKET must be a size that is a multiple of 64bits, 8bytes
 * for use on Meta cores in the GPU as well as CPU
 */
BLD_ASSERT((sizeof(RGX_HWPERF_PACKET)&0x7)==0, rgx_hwperf_h)


/******************************************************************************
 * 	API Types
 *****************************************************************************/

/*! Current count block IDs for all the hardware blocks with a performance
 * counting module in RGX.
 */
typedef enum
{
	/* Directly addressable counter blocks */
	RGX_CNTBLK_ID_TA			= 0x00,
	RGX_CNTBLK_ID_RASTER		= 0x01,
	RGX_CNTBLK_ID_HUB			= 0x02,

	/* Indirectly addressable counter blocks */
	RGX_CNTBLK_ID_TPU_MCU0		= 0x10,
	RGX_CNTBLK_ID_TPU_MCU1		= 0x11,
	RGX_CNTBLK_ID_TPU_MCU2		= 0x12,
	RGX_CNTBLK_ID_TPU_MCU3		= 0x13,
	RGX_CNTBLK_ID_TPU_MCU_ALL	= 0x1F,

	RGX_CNTBLK_ID_USC0			= 0x20,
	RGX_CNTBLK_ID_USC1			= 0x21,
	RGX_CNTBLK_ID_USC2			= 0x22,
	RGX_CNTBLK_ID_USC3			= 0x23,
	RGX_CNTBLK_ID_USC4			= 0x24,
	RGX_CNTBLK_ID_USC5			= 0x25,
	RGX_CNTBLK_ID_USC6			= 0x26,
	RGX_CNTBLK_ID_USC7			= 0x27,
	RGX_CNTBLK_ID_USC_ALL		= 0x2F,

	RGX_CNTBLK_ID_LAST

} RGX_HWPERF_CNTBLK_ID;

/*! Identifier for each counter in a performance counting module */
typedef enum
{
	RGX_CNTBLK_COUNTER0_ID		= 0,
	RGX_CNTBLK_COUNTER1_ID		= 1,
	RGX_CNTBLK_COUNTER2_ID		= 2,
	RGX_CNTBLK_COUNTER3_ID		= 3,

	RGX_CNTBLK_COUNTER_LAST

} RGX_HWPERF_CNTBLK_COUNTER_ID;

/*! Parameter 2 type for use in the array of the RGX_HWPERF_COP_ENABLE_CNTBLKS
 * configuration operation. It is used to configure the performance counter
 * module in a layout block and allows 1, 2, 3 or all the counters in the block
 * to be configured in one operation based on the counter select mask. The bit
 * shifts for this are the values in RGX_HWPERF_CNTBLK_COUNTER_ID. This mask
 * also encodes which values in the arrays are valid, for example, if bit 1 set
 * then aui8Mode[1], aui16GroupSelect[1] and aui16BitSelect[1] must be valid.
 *
 * Each layout block has 4 counters that can be programmed independently to
 * profile the performance of a HW block. Each counter can be configured to
 * accumulate statistics from 1 of 32 counter groups defined for that block.
 * Each counter group can have up to 16	signals/bits defined that can be
 * selected. Each counter may accumulate in one of two modes.
 *
 * See hwdefs/regapiperf.h for block/group/signal definitions.
 */
typedef struct _RGX_HWPERF_CONFIG_CNTBLK_
{
	/*! Counter block ID, see RGX_HWPERF_CNTBLK_ID */
	IMG_UINT8	ui8BlockID;

	/*! 4 LSBs are a mask of which counters to configure. Bit 0 is counter 0,
	 * bit 1 is counter 1 on so on. */
	IMG_UINT8   ui8CounterSelect;

	/*! 4 LSBs 0 for counting 1's in the group, 1 for treating the group
	 * signals as a number for unsigned addition. Bit 0 is counter 0, bit 1 is
	 * counter 1 on so on. This member relates to the MODE field
	 * in the RGX_CR_<n>_PERF_SELECTm register for each counter */
	IMG_UINT8	ui8Mode;

	/*! 5 LSBs used as the GROUP_SELECT field in the RGX_CR_<n>_PERF_SELECTm
	 * register. Array index 0 is counter 0, index 1 is counter 1 and so on. */
	IMG_UINT8	aui8GroupSelect[RGX_HWPERF_CNTRS_IN_BLK];

	/*! 16 LSBs used as the BIT_SELECT field in the RGX_CR_<n>_PERF_SELECTm
	 * register. Array indexes relate to counters as above. */
	IMG_UINT16	aui16BitSelect[RGX_HWPERF_CNTRS_IN_BLK];

} RGX_HWPERF_CONFIG_CNTBLK;


/* RGX_HWPERF_CONFIG_PARAM2 must be a size that is a multiple of 64bits, 8bytes
 * for use on Meta cores in the GPU as well as CPU
 */
BLD_ASSERT(sizeof(RGX_HWPERF_CONFIG_CNTBLK)==16, rgx_hwperf_h)

BLD_ASSERT((sizeof(RGX_HWPERF_CONFIG_CNTBLK)&0x7)==0, rgx_hwperf_h)



#if defined (__cplusplus)
}
#endif

#endif /* __RGX_HWPERF_H__ */

/******************************************************************************
 End of file
******************************************************************************/

