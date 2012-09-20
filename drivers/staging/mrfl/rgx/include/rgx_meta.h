									    /*************************************************************************//*!
									       @File
									       @Title          RGX META definitions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX META helper definitions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__RGX_META_H__)
#define __RGX_META_H__

#include "img_defs.h"

/************************************************************************
* META registers and MACROS 
************************************************************************/
#define	META_CR_CTRLREG_BASE(T)					(0x04800000 + 0x1000*(T))

#define META_CR_TXPRIVEXT						(0x048000E8)
#define META_CR_TXPRIVEXT_MINIM_EN				(0x1<<7)

#define META_CR_SYSC_JTAG_THREAD				(0x04830030)
#define META_CR_SYSC_JTAG_THREAD_PRIV_EN		(0x00000004)

#define	META_CR_TXUXXRXDT_OFFSET				(META_CR_CTRLREG_BASE(0) + 0x0000FFF0)
#define	META_CR_TXUXXRXRQ_OFFSET				(META_CR_CTRLREG_BASE(0) + 0x0000FFF8)

#define META_CR_TXUXXRXRQ_DREADY_BIT			(0x80000000)	/* Poll for done */
#define META_CR_TXUXXRXRQ_RDnWR_BIT  			(0x00010000)	/* Set for read  */
#define META_CR_TXUXXRXRQ_TX_S       			(12)
#define META_CR_TXUXXRXRQ_RX_S       			(4)
#define META_CR_TXUXXRXRQ_UXX_S      			(0)

#define META_CR_TXUA0_ID						(0x3)	/* Address unit regs */
#define META_CR_TXUPC_ID						(0x5)	/* PC registers */

/* Macros to calculate register access values */
#define META_CR_CORE_REG(Thr, RegNum, Unit)	(((Thr)			<< META_CR_TXUXXRXRQ_TX_S ) | \
											 ((RegNum)		<< META_CR_TXUXXRXRQ_RX_S ) | \
											 ((Unit)		<< META_CR_TXUXXRXRQ_UXX_S))

#define META_CR_THR0_PC		META_CR_CORE_REG(0, 0, META_CR_TXUPC_ID)
#define META_CR_THR0_PCX	META_CR_CORE_REG(0, 1, META_CR_TXUPC_ID)
#define META_CR_THR0_SP		META_CR_CORE_REG(0, 0, META_CR_TXUA0_ID)

#define META_CR_THR1_PC		META_CR_CORE_REG(1, 0, META_CR_TXUPC_ID)
#define META_CR_THR1_PCX	META_CR_CORE_REG(1, 1, META_CR_TXUPC_ID)
#define META_CR_THR1_SP		META_CR_CORE_REG(1, 0, META_CR_TXUA0_ID)

#define SP_ACCESS(Thread)	META_CR_CORE_REG(Thread, 0, META_CR_TXUA0_ID)
#define PC_ACCESS(Thread)	META_CR_CORE_REG(Thread, 0, META_CR_TXUPC_ID)

#define	META_CR_COREREG_ENABLE			(0x0000000)
#define	META_CR_COREREG_STATUS			(0x0000010)
#define	META_CR_COREREG_DEFR			(0x00000A0)

#define	META_CR_T0ENABLE_OFFSET			(META_CR_CTRLREG_BASE(0) + META_CR_COREREG_ENABLE)
#define	META_CR_T0STATUS_OFFSET			(META_CR_CTRLREG_BASE(0) + META_CR_COREREG_STATUS)
#define	META_CR_T0DEFR_OFFSET			(META_CR_CTRLREG_BASE(0) + META_CR_COREREG_DEFR)

#define	META_CR_T1ENABLE_OFFSET			(META_CR_CTRLREG_BASE(1) + META_CR_COREREG_ENABLE)
#define	META_CR_T1STATUS_OFFSET			(META_CR_CTRLREG_BASE(1) + META_CR_COREREG_STATUS)
#define	META_CR_T1DEFR_OFFSET			(META_CR_CTRLREG_BASE(1) + META_CR_COREREG_DEFR)

#define META_CR_TXENABLE_ENABLE_BIT		(0x00000001)	/* Set if running */

#define META_MEM_GLOBAL_RANGE_BIT				(0x80000000)

/************************************************************************
* META LDR Format
************************************************************************/
/* Block header structure */
typedef struct {
	IMG_UINT32 ui32DevID;
	IMG_UINT32 ui32SLCode;
	IMG_UINT32 ui32SLData;
	IMG_UINT16 ui16PLCtrl;
	IMG_UINT16 ui16CRC;

} RGX_META_LDR_BLOCK_HDR;

/* High level data stream block  structure */
typedef struct {
	IMG_UINT16 ui16Cmd;
	IMG_UINT16 ui16Length;
	IMG_UINT32 ui32Next;
	IMG_UINT32 aui32CmdData[4];

} RGX_META_LDR_L1_DATA_BLK;

/* High level data stream block  structure */
typedef struct {
	IMG_UINT16 ui16Tag;
	IMG_UINT16 ui16Length;
	IMG_UINT32 aui32BlockData[4];

} RGX_META_LDR_L2_DATA_BLK;

/* Config command structure */
typedef struct {
	IMG_UINT32 ui32Type;
	IMG_UINT32 aui32BlockData[4];

} RGX_META_LDR_CFG_BLK;

/* Block type definitions */
#define RGX_META_LDR_COMMENT_TYPE_MASK			(0x0010)
#define RGX_META_LDR_BLK_IS_COMMENT(X)			((X & RGX_META_LDR_COMMENT_TYPE_MASK) != 0)

/* Command definitions
	Value	Name			Description
	0		LoadMem			Load memory with binary data.
	1		LoadCore		Load a set of core registers.
	2		LoadMMReg		Load a set of memory mapped registers.
	3		StartThreads	Set each thread PC and SP, then enable	threads.
	4		ZeroMem			Zeros a memory region.
	5		Config			Perform	a configuration command. */
#define RGX_META_LDR_CMD_MASK				(0x000F)

#define RGX_META_LDR_CMD_LOADMEM			(0x0000)
#define RGX_META_LDR_CMD_LOADCORE			(0x0001)
#define RGX_META_LDR_CMD_LOADMMREG			(0x0002)
#define RGX_META_LDR_CMD_START_THREADS		(0x0003)
#define RGX_META_LDR_CMD_ZEROMEM			(0x0004)
#define RGX_META_LDR_CMD_CONFIG			(0x0005)

/* Config Command definitions
	Value	Name		Description
	0		Pause		Pause for x times 100 instructions
	1		Read		Read a value from register - No value return needed.
						Utilises effects of issuing reads to certain registers
	2		Write		Write to mem location
	3		MemSet		Set mem to value
	4		MemCheck	check mem for specific value.*/
#define RGX_META_LDR_CFG_PAUSE			(0x0000)
#define RGX_META_LDR_CFG_READ			(0x0001)
#define RGX_META_LDR_CFG_WRITE			(0x0002)
#define RGX_META_LDR_CFG_MEMSET			(0x0003)
#define RGX_META_LDR_CFG_MEMCHECK		(0x0004)

/************************************************************************
* RGX FW segmented MMU definitions
************************************************************************/
/* All threads can access the segment */
#define RGXFW_SEGMMU_ALLTHRS	(0xf << 8)
/* Writeable */
#define RGXFW_SEGMMU_WRITEABLE	(0x1 << 1)
/* All threads can access and writeable */
#define RGXFW_SEGMMU_ALLTHRS_WRITEABLE	(RGXFW_SEGMMU_ALLTHRS | RGXFW_SEGMMU_WRITEABLE)

/* Direct map regions mapping */
#define RGXFW_SEGMMU_DMAP_ID_START			(8)
#define RGXFW_SEGMMU_DMAP_ADDR_START		(0x06000000U)
#define RGXFW_SEGMMU_DMAP_ADDR_META			(0x86000000U)
#define RGXFW_SEGMMU_DMAP_SIZE				(8*1024*1024)	/* 8 MB */
#define RGXFW_SEGMMU_DMAP_NUM				(4)

/* Segment IDs */
#define RGXFW_SEGMMU_THR0_ID			(0)
#define RGXFW_SEGMMU_SHARED_ID			(1)
#define RGXFW_SEGMMU_BOOTLDR_ID			(2)
#define RGXFW_SEGMMU_THR1_ID			(3)

/* To configure the Page Catalog used */
#define RGXFW_SEGMMU_META_DM_PC(pc)			((((IMG_UINT64) ((pc) & 0xF)) << 44) | IMG_UINT64_C(0x70000000000))

/* META segments have 4kB minimum size */
#define RGXFW_SEGMMU_ALIGN			(0x1000)

/* Segmented MMU registers (n = segment id) */
#define META_CR_MMCU_SEGMENTn_BASE(n)			(0x04850000 + (n)*0x10)
#define META_CR_MMCU_SEGMENTn_LIMIT(n)			(0x04850004 + (n)*0x10)
#define META_CR_MMCU_SEGMENTn_OUTA0(n)			(0x04850008 + (n)*0x10)
#define META_CR_MMCU_SEGMENTn_OUTA1(n)			(0x0485000C + (n)*0x10)

/************************************************************************
* RGX FW Bootloader defaults
************************************************************************/
#define RGXFW_BOOTLDR_META_ADDR					(0x40000000)
#define RGXFW_BOOTLDR_DEVV_ADDR_0				(0xC0000000)
#define RGXFW_BOOTLDR_DEVV_ADDR_1				(0x000007E1)
#define RGXFW_BOOTLDR_LIMIT						(0x1FFFF000)

#endif				/*  __RGX_META_H__ */

/******************************************************************************
 End of file (rgx_meta.h)
******************************************************************************/
