/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
 * Copyright (c) Imagination Technologies Limited, UK
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#ifndef _LNC_TOPAZ_HW_REG_H_
#define _LNC_TOPAZ_HW_REG_H_

#ifdef _PNW_TOPAZ_HW_REG_H_
#error "pnw_topaz_hw_reg.h shouldn't be included"
#endif

#include "psb_drv.h"
#include "lnc_topaz.h"

#define LNC_TOPAZ_NO_IRQ 0
#define TOPAZ_MTX_REG_SIZE (34 * 4 + 183 * 4)

extern int drm_topaz_pmpolicy;

/*
 * MACROS to insert values into fields within a word. The basename of the
 * field must have MASK_BASENAME and SHIFT_BASENAME constants.
 */
#define MM_WRITE32(base, offset, value)  \
do {				       \
	*((unsigned long *)((unsigned char *)(dev_priv->topaz_reg)	\
				+ base + offset)) = value;		\
} while (0)

#define MM_READ32(base, offset, pointer) \
do {                                   \
	*(pointer) = *((unsigned long *)((unsigned char *)(dev_priv->topaz_reg)\
						+ base + offset));	\
} while (0)

#define F_MASK(basename)  (MASK_##basename)
#define F_SHIFT(basename) (SHIFT_##basename)

#define F_ENCODE(val, basename)  \
	(((val) << (F_SHIFT(basename))) & (F_MASK(basename)))

/* MVEA macro */
#define MVEA_START 0x03000

#define MVEA_WRITE32(offset, value) MM_WRITE32(MVEA_START, offset, value)
#define MVEA_READ32(offset, pointer) MM_READ32(MVEA_START, offset, pointer);

#define F_MASK_MVEA(basename)  (MASK_MVEA_##basename)	/*     MVEA    */
#define F_SHIFT_MVEA(basename) (SHIFT_MVEA_##basename)	/*     MVEA    */
#define F_ENCODE_MVEA(val, basename)  \
	(((val)<<(F_SHIFT_MVEA(basename)))&(F_MASK_MVEA(basename)))

/* VLC macro */
#define TOPAZ_VLC_START 0x05000

/* TOPAZ macro */
#define TOPAZ_START 0x02000

#define TOPAZ_WRITE32(offset, value) MM_WRITE32(TOPAZ_START, offset, value)
#define TOPAZ_READ32(offset, pointer) MM_READ32(TOPAZ_START, offset, pointer)

#define F_MASK_TOPAZ(basename)  (MASK_TOPAZ_##basename)
#define F_SHIFT_TOPAZ(basename) (SHIFT_TOPAZ_##basename)
#define F_ENCODE_TOPAZ(val, basename) \
	(((val)<<(F_SHIFT_TOPAZ(basename)))&(F_MASK_TOPAZ(basename)))

/* MTX macro */
#define MTX_START 0x0

#define MTX_WRITE32(offset, value) MM_WRITE32(MTX_START, offset, value)
#define MTX_READ32(offset, pointer) MM_READ32(MTX_START, offset, pointer)

/* DMAC macro */
#define DMAC_START 0x0f000

#define DMAC_WRITE32(offset, value) MM_WRITE32(DMAC_START, offset, value)
#define DMAC_READ32(offset, pointer) MM_READ32(DMAC_START, offset, pointer)

#define F_MASK_DMAC(basename)  (MASK_DMAC_##basename)
#define F_SHIFT_DMAC(basename) (SHIFT_DMAC_##basename)
#define F_ENCODE_DMAC(val, basename)  \
	(((val)<<(F_SHIFT_DMAC(basename)))&(F_MASK_DMAC(basename)))

/* Register CR_IMG_TOPAZ_INTENAB */
#define TOPAZ_CR_IMG_TOPAZ_INTENAB  0x0008
#define MASK_TOPAZ_CR_IMG_TOPAZ_INTEN_MVEA 0x00000001
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTEN_MVEA 0
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTEN_MVEA 0x0008

#define MASK_TOPAZ_CR_IMG_TOPAZ_MAS_INTEN 0x80000000
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_MAS_INTEN 31
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_MAS_INTEN 0x0008

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTEN_MMU_FAULT 0x00000008
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTEN_MMU_FAULT 3
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTEN_MMU_FAULT 0x0008

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX 0x00000002
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX 1
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX 0x0008

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX_HALT 0x00000004
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX_HALT 2
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTEN_MTX_HALT 0x0008

#define TOPAZ_CR_IMG_TOPAZ_INTCLEAR 0x000C
#define MASK_TOPAZ_CR_IMG_TOPAZ_INTCLR_MVEA 0x00000001
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTCLR_MVEA 0
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTCLR_MVEA 0x000C

#define TOPAZ_CR_IMG_TOPAZ_INTSTAT  0x0004
#define MASK_TOPAZ_CR_IMG_TOPAZ_INTS_MVEA 0x00000001
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTS_MVEA 0
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTS_MVEA 0x0004

#define MTX_CCBCTRL_ROFF		0
#define MTX_CCBCTRL_COMPLETE		4
#define MTX_CCBCTRL_CCBSIZE		8
#define MTX_CCBCTRL_QP			12
#define MTX_CCBCTRL_FRAMESKIP		20
#define MTX_CCBCTRL_INITQP		24

#define TOPAZ_CR_MMU_STATUS         0x001C
#define MASK_TOPAZ_CR_MMU_PF_N_RW   0x00000001
#define SHIFT_TOPAZ_CR_MMU_PF_N_RW  0
#define REGNUM_TOPAZ_CR_MMU_PF_N_RW 0x001C

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTCLR_MMU_FAULT 0x00000008
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTCLR_MMU_FAULT 3
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTCLR_MMU_FAULT 0x000C

#define TOPAZ_CR_MMU_MEM_REQ        0x0020
#define MASK_TOPAZ_CR_MEM_REQ_STAT_READS 0x000000FF
#define SHIFT_TOPAZ_CR_MEM_REQ_STAT_READS 0
#define REGNUM_TOPAZ_CR_MEM_REQ_STAT_READS 0x0020

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX 0x00000002
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX 1
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX 0x000C

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX_HALT 0x00000004
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX_HALT 2
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX_HALT 0x000C

#define MTX_CR_MTX_KICK             0x0080
#define MASK_MTX_MTX_KICK           0x0000FFFF
#define SHIFT_MTX_MTX_KICK          0
#define REGNUM_MTX_MTX_KICK         0x0080

#define MTX_DATA_MEM_BASE		0x82880000

#define MTX_CR_MTX_RAM_ACCESS_CONTROL 0x0108
#define MASK_MTX_MTX_MCMR           0x00000001
#define SHIFT_MTX_MTX_MCMR          0
#define REGNUM_MTX_MTX_MCMR         0x0108

#define MASK_MTX_MTX_MCMID          0x0FF00000
#define SHIFT_MTX_MTX_MCMID         20
#define REGNUM_MTX_MTX_MCMID        0x0108

#define MASK_MTX_MTX_MCM_ADDR       0x000FFFFC
#define SHIFT_MTX_MTX_MCM_ADDR      2
#define REGNUM_MTX_MTX_MCM_ADDR     0x0108

#define MTX_CR_MTX_RAM_ACCESS_STATUS 0x010C
#define MASK_MTX_MTX_MTX_MCM_STAT   0x00000001
#define SHIFT_MTX_MTX_MTX_MCM_STAT  0
#define REGNUM_MTX_MTX_MTX_MCM_STAT 0x010C

#define MASK_MTX_MTX_MCMAI          0x00000002
#define SHIFT_MTX_MTX_MCMAI         1
#define REGNUM_MTX_MTX_MCMAI        0x0108

#define MTX_CR_MTX_RAM_ACCESS_DATA_TRANSFER 0x0104

#define MVEA_CR_MVEA_BUSY           0x0018
#define MVEA_CR_MVEA_DMACMDFIFO_WAIT 0x001C
#define MVEA_CR_MVEA_DMACMDFIFO_STATUS 0x0020

#define MVEA_CR_IMG_MVEA_SRST       0x0000
#define MASK_MVEA_CR_IMG_MVEA_SPE_SOFT_RESET 0x00000001
#define SHIFT_MVEA_CR_IMG_MVEA_SPE_SOFT_RESET 0
#define REGNUM_MVEA_CR_IMG_MVEA_SPE_SOFT_RESET 0x0000

#define MASK_MVEA_CR_IMG_MVEA_IPE_SOFT_RESET 0x00000002
#define SHIFT_MVEA_CR_IMG_MVEA_IPE_SOFT_RESET 1
#define REGNUM_MVEA_CR_IMG_MVEA_IPE_SOFT_RESET 0x0000

#define MASK_MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET 0x00000004
#define SHIFT_MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET 2
#define REGNUM_MVEA_CR_IMG_MVEA_CMPRS_SOFT_RESET 0x0000

#define MASK_MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET 0x00000008
#define SHIFT_MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET 3
#define REGNUM_MVEA_CR_IMG_MVEA_JMCOMP_SOFT_RESET 0x0000

#define MASK_MVEA_CR_IMG_MVEA_CMC_SOFT_RESET 0x00000010
#define SHIFT_MVEA_CR_IMG_MVEA_CMC_SOFT_RESET 4
#define REGNUM_MVEA_CR_IMG_MVEA_CMC_SOFT_RESET 0x0000

#define MASK_MVEA_CR_IMG_MVEA_DCF_SOFT_RESET 0x00000020
#define SHIFT_MVEA_CR_IMG_MVEA_DCF_SOFT_RESET 5
#define REGNUM_MVEA_CR_IMG_MVEA_DCF_SOFT_RESET 0x0000

#define TOPAZ_CR_IMG_TOPAZ_CORE_ID  0x03C0
#define TOPAZ_CR_IMG_TOPAZ_CORE_REV 0x03D0

#define TOPAZ_MTX_PC		(0x00000005)
#define PC_START_ADDRESS	(0x80900000)

#define TOPAZ_CR_TOPAZ_AUTO_CLK_GATE 0x0014
#define MASK_TOPAZ_CR_TOPAZ_VLC_AUTO_CLK_GATE 0x00000001
#define SHIFT_TOPAZ_CR_TOPAZ_VLC_AUTO_CLK_GATE 0
#define REGNUM_TOPAZ_CR_TOPAZ_VLC_AUTO_CLK_GATE 0x0014

#define MASK_TOPAZ_CR_TOPAZ_DB_AUTO_CLK_GATE 0x00000002
#define SHIFT_TOPAZ_CR_TOPAZ_DB_AUTO_CLK_GATE 1
#define REGNUM_TOPAZ_CR_TOPAZ_DB_AUTO_CLK_GATE 0x0014

#define MASK_TOPAZ_CR_TOPAZ_MTX_MAN_CLK_GATE 0x00000002
#define SHIFT_TOPAZ_CR_TOPAZ_MTX_MAN_CLK_GATE 1
#define REGNUM_TOPAZ_CR_TOPAZ_MTX_MAN_CLK_GATE 0x0010

#define	MTX_CORE_CR_MTX_REGISTER_READ_WRITE_DATA_OFFSET 0x000000F8
#define	MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_OFFSET 0x000000FC
#define	MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_RNW_MASK 0x00010000
#define	MTX_CORE_CR_MTX_REGISTER_READ_WRITE_REQUEST_MTX_DREADY_MASK 0x80000000

#define	TOPAZ_CORE_CR_MTX_DEBUG_OFFSET	0x0000003C

#define MASK_TOPAZ_CR_MTX_DBG_IS_SLAVE 0x00000004
#define SHIFT_TOPAZ_CR_MTX_DBG_IS_SLAVE 2
#define REGNUM_TOPAZ_CR_MTX_DBG_IS_SLAVE 0x003C

#define MASK_TOPAZ_CR_MTX_DBG_GPIO_OUT 0x00000018
#define SHIFT_TOPAZ_CR_MTX_DBG_GPIO_OUT 3
#define REGNUM_TOPAZ_CR_MTX_DBG_GPIO_OUT 0x003C

#define	MTX_CORE_CR_MTX_RAM_ACCESS_CONTROL_OFFSET 0x00000108

#define TOPAZ_CR_MMU_CONTROL0       0x0024
#define MASK_TOPAZ_CR_MMU_BYPASS    0x00000800
#define SHIFT_TOPAZ_CR_MMU_BYPASS   11
#define REGNUM_TOPAZ_CR_MMU_BYPASS  0x0024

#define TOPAZ_CR_MMU_DIR_LIST_BASE(X) (0x0030 + (4 * (X)))
#define MASK_TOPAZ_CR_MMU_DIR_LIST_BASE_ADDR 0xFFFFF000
#define SHIFT_TOPAZ_CR_MMU_DIR_LIST_BASE_ADDR 12
#define REGNUM_TOPAZ_CR_MMU_DIR_LIST_BASE_ADDR 0x0030

#define MASK_TOPAZ_CR_MMU_INVALDC   0x00000008
#define SHIFT_TOPAZ_CR_MMU_INVALDC  3
#define REGNUM_TOPAZ_CR_MMU_INVALDC 0x0024

#define MASK_TOPAZ_CR_MMU_FLUSH     0x00000004
#define SHIFT_TOPAZ_CR_MMU_FLUSH    2
#define REGNUM_TOPAZ_CR_MMU_FLUSH   0x0024

#define TOPAZ_CR_MMU_BANK_INDEX     0x0038
#define MASK_TOPAZ_CR_MMU_BANK_N_INDEX_M(i) (0x00000003 << (8 + ((i) * 2)))
#define SHIFT_TOPAZ_CR_MMU_BANK_N_INDEX_M(i) (8 + ((i) * 2))
#define REGNUM_TOPAZ_CR_MMU_BANK_N_INDEX_M(i) 0x0038

#define TOPAZ_CR_TOPAZ_MAN_CLK_GATE 0x0010
#define MASK_TOPAZ_CR_TOPAZ_MVEA_MAN_CLK_GATE 0x00000001
#define SHIFT_TOPAZ_CR_TOPAZ_MVEA_MAN_CLK_GATE 0
#define REGNUM_TOPAZ_CR_TOPAZ_MVEA_MAN_CLK_GATE 0x0010

#define MTX_CORE_CR_MTX_TXRPT_OFFSET 0x0000000c
#define TXRPT_WAITONKICK_VALUE 0x8ade0000

#define MTX_CORE_CR_MTX_ENABLE_MTX_TOFF_MASK 0x00000002

#define MTX_CORE_CR_MTX_ENABLE_OFFSET 0x00000000
#define	MTX_CORE_CR_MTX_ENABLE_MTX_ENABLE_MASK 0x00000001

#define MASK_TOPAZ_CR_IMG_TOPAZ_INTS_MTX 0x00000002
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_INTS_MTX 1
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_INTS_MTX 0x0004

#define	MTX_CORE_CR_MTX_SOFT_RESET_OFFSET 0x00000200
#define	MTX_CORE_CR_MTX_SOFT_RESET_MTX_RESET_MASK 0x00000001

#define MTX_CR_MTX_SYSC_CDMAA       0x0344
#define MASK_MTX_CDMAA_ADDRESS      0x03FFFFFC
#define SHIFT_MTX_CDMAA_ADDRESS     2
#define REGNUM_MTX_CDMAA_ADDRESS    0x0344

#define MTX_CR_MTX_SYSC_CDMAC       0x0340
#define MASK_MTX_LENGTH             0x0000FFFF
#define SHIFT_MTX_LENGTH            0
#define REGNUM_MTX_LENGTH           0x0340

#define MASK_MTX_BURSTSIZE          0x07000000
#define SHIFT_MTX_BURSTSIZE         24
#define REGNUM_MTX_BURSTSIZE        0x0340

#define MASK_MTX_RNW                0x00020000
#define SHIFT_MTX_RNW               17
#define REGNUM_MTX_RNW              0x0340

#define MASK_MTX_ENABLE             0x00010000
#define SHIFT_MTX_ENABLE            16
#define REGNUM_MTX_ENABLE           0x0340

#define MASK_MTX_LENGTH             0x0000FFFF
#define SHIFT_MTX_LENGTH            0
#define REGNUM_MTX_LENGTH           0x0340

#define TOPAZ_CR_IMG_TOPAZ_SRST     0x0000
#define MASK_TOPAZ_CR_IMG_TOPAZ_MVEA_SOFT_RESET 0x00000001
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_MVEA_SOFT_RESET 0
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_MVEA_SOFT_RESET 0x0000

#define MASK_TOPAZ_CR_IMG_TOPAZ_VLC_SOFT_RESET 0x00000008
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_VLC_SOFT_RESET 3
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_VLC_SOFT_RESET 0x0000

#define MASK_TOPAZ_CR_IMG_TOPAZ_MTX_SOFT_RESET 0x00000002
#define SHIFT_TOPAZ_CR_IMG_TOPAZ_MTX_SOFT_RESET 1
#define REGNUM_TOPAZ_CR_IMG_TOPAZ_MTX_SOFT_RESET 0x0000

#define MVEA_CR_MVEA_AUTO_CLOCK_GATING 0x0024
#define MASK_MVEA_CR_MVEA_SPE_AUTO_CLK_GATE 0x00000001
#define SHIFT_MVEA_CR_MVEA_SPE_AUTO_CLK_GATE 0
#define REGNUM_MVEA_CR_MVEA_SPE_AUTO_CLK_GATE 0x0024

#define MASK_MVEA_CR_MVEA_IPE_AUTO_CLK_GATE 0x00000002
#define SHIFT_MVEA_CR_MVEA_IPE_AUTO_CLK_GATE 1
#define REGNUM_MVEA_CR_MVEA_IPE_AUTO_CLK_GATE 0x0024

#define MASK_MVEA_CR_MVEA_CMPRS_AUTO_CLK_GATE 0x00000004
#define SHIFT_MVEA_CR_MVEA_CMPRS_AUTO_CLK_GATE 2
#define REGNUM_MVEA_CR_MVEA_CMPRS_AUTO_CLK_GATE 0x0024

#define MASK_MVEA_CR_MVEA_JMCOMP_AUTO_CLK_GATE 0x00000008
#define SHIFT_MVEA_CR_MVEA_JMCOMP_AUTO_CLK_GATE 3
#define REGNUM_MVEA_CR_MVEA_JMCOMP_AUTO_CLK_GATE 0x0024

#define TOPAZ_CR_IMG_TOPAZ_DMAC_MODE 0x0040
#define MASK_TOPAZ_CR_DMAC_MASTER_MODE 0x00000001
#define SHIFT_TOPAZ_CR_DMAC_MASTER_MODE 0
#define REGNUM_TOPAZ_CR_DMAC_MASTER_MODE 0x0040

#define MTX_CR_MTX_SYSC_CDMAT       0x0350
#define MASK_MTX_TRANSFERDATA       0xFFFFFFFF
#define SHIFT_MTX_TRANSFERDATA      0
#define REGNUM_MTX_TRANSFERDATA     0x0350

#define IMG_SOC_DMAC_IRQ_STAT(X)    (0x000C + (32 * (X)))
#define MASK_IMG_SOC_TRANSFER_FIN   0x00020000
#define SHIFT_IMG_SOC_TRANSFER_FIN  17
#define REGNUM_IMG_SOC_TRANSFER_FIN 0x000C

#define IMG_SOC_DMAC_COUNT(X)       (0x0004 + (32 * (X)))
#define MASK_IMG_SOC_CNT            0x0000FFFF
#define SHIFT_IMG_SOC_CNT           0
#define REGNUM_IMG_SOC_CNT          0x0004

#define MASK_IMG_SOC_EN             0x00010000
#define SHIFT_IMG_SOC_EN            16
#define REGNUM_IMG_SOC_EN           0x0004

#define MASK_IMG_SOC_LIST_EN        0x00040000
#define SHIFT_IMG_SOC_LIST_EN       18
#define REGNUM_IMG_SOC_LIST_EN      0x0004

#define IMG_SOC_DMAC_PER_HOLD(X)    (0x0018 + (32 * (X)))
#define MASK_IMG_SOC_PER_HOLD       0x0000007F
#define SHIFT_IMG_SOC_PER_HOLD      0
#define REGNUM_IMG_SOC_PER_HOLD     0x0018

#define IMG_SOC_DMAC_SETUP(X)       (0x0000 + (32 * (X)))
#define MASK_IMG_SOC_START_ADDRESS  0xFFFFFFF
#define SHIFT_IMG_SOC_START_ADDRESS 0
#define REGNUM_IMG_SOC_START_ADDRESS 0x0000

#define MASK_IMG_SOC_BSWAP          0x40000000
#define SHIFT_IMG_SOC_BSWAP         30
#define REGNUM_IMG_SOC_BSWAP        0x0004

#define MASK_IMG_SOC_PW             0x18000000
#define SHIFT_IMG_SOC_PW            27
#define REGNUM_IMG_SOC_PW           0x0004

#define MASK_IMG_SOC_DIR            0x04000000
#define SHIFT_IMG_SOC_DIR           26
#define REGNUM_IMG_SOC_DIR          0x0004

#define MASK_IMG_SOC_PI             0x03000000
#define SHIFT_IMG_SOC_PI            24
#define REGNUM_IMG_SOC_PI           0x0004
#define IMG_SOC_PI_1		0x00000002
#define IMG_SOC_PI_2		0x00000001
#define IMG_SOC_PI_4		0x00000000

#define MASK_IMG_SOC_TRANSFER_IEN   0x20000000
#define SHIFT_IMG_SOC_TRANSFER_IEN  29
#define REGNUM_IMG_SOC_TRANSFER_IEN 0x0004

#define DMAC_VALUE_COUNT(BSWAP, PW, DIR, PERIPH_INCR, COUNT)        \
	((((BSWAP) << SHIFT_IMG_SOC_BSWAP) & MASK_IMG_SOC_BSWAP)|	\
		(((PW) << SHIFT_IMG_SOC_PW) & MASK_IMG_SOC_PW)|		\
		(((DIR) << SHIFT_IMG_SOC_DIR) & MASK_IMG_SOC_DIR)|	\
		(((PERIPH_INCR) << SHIFT_IMG_SOC_PI) & MASK_IMG_SOC_PI)| \
		(((COUNT) << SHIFT_IMG_SOC_CNT) & MASK_IMG_SOC_CNT))

#define IMG_SOC_DMAC_PERIPH(X)      (0x0008 + (32 * (X)))
#define MASK_IMG_SOC_EXT_SA         0x0000000F
#define SHIFT_IMG_SOC_EXT_SA        0
#define REGNUM_IMG_SOC_EXT_SA       0x0008

#define MASK_IMG_SOC_ACC_DEL        0xE0000000
#define SHIFT_IMG_SOC_ACC_DEL       29
#define REGNUM_IMG_SOC_ACC_DEL      0x0008

#define MASK_IMG_SOC_INCR           0x08000000
#define SHIFT_IMG_SOC_INCR          27
#define REGNUM_IMG_SOC_INCR         0x0008

#define MASK_IMG_SOC_BURST          0x07000000
#define SHIFT_IMG_SOC_BURST         24
#define REGNUM_IMG_SOC_BURST        0x0008

#define DMAC_VALUE_PERIPH_PARAM(ACC_DEL, INCR, BURST)             \
((((ACC_DEL) << SHIFT_IMG_SOC_ACC_DEL) & MASK_IMG_SOC_ACC_DEL)|	\
(((INCR) << SHIFT_IMG_SOC_INCR) & MASK_IMG_SOC_INCR)|             \
(((BURST) << SHIFT_IMG_SOC_BURST) & MASK_IMG_SOC_BURST))

#define IMG_SOC_DMAC_PERIPHERAL_ADDR(X) (0x0014 + (32 * (X)))
#define MASK_IMG_SOC_ADDR           0x007FFFFF
#define SHIFT_IMG_SOC_ADDR          0
#define REGNUM_IMG_SOC_ADDR         0x0014

#define SHIFT_TOPAZ_VEC_BUSY        11
#define MASK_TOPAZ_VEC_BUSY         (0x1<<SHIFT_TOPAZ_VEC_BUSY)

#define TOPAZ_MTX_TXRPT_OFFSET         0xc
#define TOPAZ_GUNIT_GVD_PSMI_GFX_OFFSET 0x20D0

#define TOPAZ_GUNIT_READ32(offset)  ioread32(dev_priv->vdc_reg + offset)
#define TOPAZ_READ_BITS(val, basename) \
		(((val)&MASK_TOPAZ_##basename)>>SHIFT_TOPAZ_##basename)

#define TOPAZ_WAIT_UNTIL_IDLE \
do { \
	uint8_t tmp_poll_number = 0;\
	uint32_t tmp_reg; \
	if (topaz_priv->topaz_cmd_windex == WB_CCB_CTRL_RINDEX(dev_priv)) { \
		tmp_reg = TOPAZ_GUNIT_READ32(TOPAZ_GUNIT_GVD_PSMI_GFX_OFFSET);\
		if (0 != TOPAZ_READ_BITS(tmp_reg, VEC_BUSY)) { \
			MTX_READ32(TOPAZ_MTX_TXRPT_OFFSET, &tmp_reg);\
			while ((tmp_reg != 0x8ade0000) && \
				(tmp_poll_number++ < 10)) \
				MTX_READ32(0xc, &tmp_reg); \
			PSB_DEBUG_GENERAL(	\
			"TOPAZ: TXRPT reg remain: %x,poll %d times.\n",\
			tmp_reg, tmp_poll_number);\
		} \
	} \
} while (0)

/* **************** DMAC define **************** */
enum DMAC_eBSwap {
	/* !< No byte swapping will be performed. */
	DMAC_BSWAP_NO_SWAP = 0x0,
	DMAC_BSWAP_REVERSE = 0x1,	/* !< Byte order will be reversed. */
};

enum DMAC_ePW {
	DMAC_PWIDTH_32_BIT = 0x0,	/* !< Peripheral width 32-bit. */
	DMAC_PWIDTH_16_BIT = 0x1,	/* !< Peripheral width 16-bit. */
	DMAC_PWIDTH_8_BIT = 0x2,	/* !< Peripheral width 8-bit. */
};

enum DMAC_eAccDel {
	DMAC_ACC_DEL_0 = 0x0,	/* !< Access delay zero clock cycles */
	DMAC_ACC_DEL_256 = 0x1,	/* !< Access delay 256 clock cycles */
	DMAC_ACC_DEL_512 = 0x2,	/* !< Access delay 512 clock cycles */
	DMAC_ACC_DEL_768 = 0x3,	/* !< Access delay 768 clock cycles */
	DMAC_ACC_DEL_1024 = 0x4,	/* !< Access delay 1024 clock cycles */
	DMAC_ACC_DEL_1280 = 0x5,	/* !< Access delay 1280 clock cycles */
	DMAC_ACC_DEL_1536 = 0x6,	/* !< Access delay 1536 clock cycles */
	DMAC_ACC_DEL_1792 = 0x7,	/* !< Access delay 1792 clock cycles */
};

enum DMAC_eBurst {
	DMAC_BURST_0 = 0x0,	/* !< burst size of 0 */
	DMAC_BURST_1 = 0x1,	/* !< burst size of 1 */
	DMAC_BURST_2 = 0x2,	/* !< burst size of 2 */
	DMAC_BURST_3 = 0x3,	/* !< burst size of 3 */
	DMAC_BURST_4 = 0x4,	/* !< burst size of 4 */
	DMAC_BURST_5 = 0x5,	/* !< burst size of 5 */
	DMAC_BURST_6 = 0x6,	/* !< burst size of 6 */
	DMAC_BURST_7 = 0x7,	/* !< burst size of 7 */
};

/* codecs topaz supports,shared with user space driver */
enum drm_lnc_topaz_codec {
	IMG_CODEC_JPEG = 0,
	IMG_CODEC_H264_NO_RC,
	IMG_CODEC_H264_VBR,
	IMG_CODEC_H264_CBR,
	IMG_CODEC_H264_VCM,
	IMG_CODEC_H263_NO_RC,
	IMG_CODEC_H263_VBR,
	IMG_CODEC_H263_CBR,
	IMG_CODEC_MPEG4_NO_RC,
	IMG_CODEC_MPEG4_VBR,
	IMG_CODEC_MPEG4_CBR,
	IMG_CODEC_NUM
};

/* commands for topaz,shared with user space driver */
enum drm_lnc_topaz_cmd {
	MTX_CMDID_NULL = 0,
	MTX_CMDID_DO_HEADER = 1,
	MTX_CMDID_ENCODE_SLICE = 2,
	MTX_CMDID_WRITEREG = 3,
	MTX_CMDID_START_PIC = 4,
	MTX_CMDID_END_PIC = 5,
	MTX_CMDID_SYNC = 6,
	MTX_CMDID_ENCODE_ONE_ROW = 7,
	MTX_CMDID_FLUSH = 8,
	MTX_CMDID_SW_LEAVE_LOWPOWER = 0x7c,
	MTX_CMDID_SW_ENTER_LOWPOWER = 0x7e,
	MTX_CMDID_SW_NEW_CODEC = 0x7f
};

struct topaz_cmd_header {
	union {
		struct {
			unsigned long enable_interrupt:1;
			unsigned long id:7;
			unsigned long size:8;
			unsigned long seq:16;
		};
		uint32_t val;
	};
};

/* lnc_topazinit.c */
int lnc_topaz_reset(struct drm_psb_private *dev_priv);
int topaz_init_fw(struct drm_device *dev);
int topaz_setup_fw(struct drm_device *dev, enum drm_lnc_topaz_codec codec);
int topaz_wait_for_register(struct drm_psb_private *dev_priv,
			    uint32_t addr, uint32_t value, uint32_t enable);
void topaz_write_mtx_mem(struct drm_psb_private *dev_priv,
			 uint32_t byte_addr, uint32_t val);
uint32_t topaz_read_mtx_mem(struct drm_psb_private *dev_priv,
			    uint32_t byte_addr);
void topaz_write_mtx_mem_multiple_setup(struct drm_psb_private *dev_priv,
					uint32_t addr);
void topaz_write_mtx_mem_multiple(struct drm_psb_private *dev_priv,
				  uint32_t val);
void topaz_mmu_flushcache(struct drm_psb_private *dev_priv);

void topaz_mtx_kick(struct drm_psb_private *dev_priv, uint32_t kick_cout);

uint32_t psb_get_default_pd_addr(struct psb_mmu_driver *driver);

/* macros to get/set CCB control data */
#define WB_CCB_CTRL_RINDEX(dev_priv) \
(*((uint32_t *)((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_wb))

#define WB_CCB_CTRL_SEQ(dev_priv) \
(*((uint32_t *)((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_wb\
	+ 1))

#define POLL_WB_RINDEX(dev_priv, value)				\
do {								\
	int i;							\
	for (i = 0; i < 10000; i++) {				\
		if (WB_CCB_CTRL_RINDEX(dev_priv) == value)	\
			break;					\
		else						\
			DRM_UDELAY(100);			\
	}							\
	if (WB_CCB_CTRL_RINDEX(dev_priv) != value) {		\
		DRM_ERROR("TOPAZ: poll rindex timeout\n");	\
		ret = -EBUSY;					\
	}							\
} while (0)

#define POLL_WB_SEQ(dev_priv, value)				\
do {								\
	int i;							\
	for (i = 0; i < 10000; i++) {				\
		if (CCB_CTRL_SEQ(dev_priv) == value)	\
			break;					\
		else						\
			DRM_UDELAY(1000);			\
	}							\
	if (CCB_CTRL_SEQ(dev_priv) != value) {		\
		DRM_ERROR("TOPAZ:poll mtxseq timeout,0x%08x(mtx) vs 0x%08x\n",\
			WB_CCB_CTRL_SEQ(dev_priv), value);		\
		ret = -EBUSY;					\
	}							\
} while (0)

#define CCB_CTRL_RINDEX(dev_priv)			\
	topaz_read_mtx_mem(dev_priv,			\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_ROFF)

#define CCB_CTRL_RINDEX(dev_priv)			\
	topaz_read_mtx_mem(dev_priv,			\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_ROFF)

#define CCB_CTRL_QP(dev_priv)						\
	topaz_read_mtx_mem(dev_priv,					\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_QP)

#define CCB_CTRL_SEQ(dev_priv)						\
	topaz_read_mtx_mem(dev_priv,					\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_COMPLETE)

#define CCB_CTRL_FRAMESKIP(dev_priv)				   \
	topaz_read_mtx_mem(dev_priv,				   \
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_FRAMESKIP)

#define CCB_CTRL_SET_QP(dev_priv, qp)					\
	topaz_write_mtx_mem(dev_priv,					\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_QP, qp)

#define CCB_CTRL_SET_INITIALQP(dev_priv, qp)			    \
	topaz_write_mtx_mem(dev_priv,				    \
	((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_ctrl_addr \
			+ MTX_CCBCTRL_INITQP, qp)

#define TOPAZ_BEGIN_CCB(dev_priv)					\
topaz_write_mtx_mem_multiple_setup(dev_priv,			\
((struct topaz_private *)dev_priv->topaz_private)->topaz_ccb_buffer_addr + \
((struct topaz_private *)dev_priv->topaz_private)->topaz_cmd_windex * 4)

#define TOPAZ_OUT_CCB(dev_priv, cmd)					\
do {									\
	topaz_write_mtx_mem_multiple(dev_priv, cmd);			\
	((struct topaz_private *)dev_priv->topaz_private)->topaz_cmd_windex++; \
} while (0)

#define TOPAZ_END_CCB(dev_priv, kick_count)	\
	topaz_mtx_kick(dev_priv, 1);

static inline char *cmd_to_string(int cmd_id)
{
	switch (cmd_id) {
	case MTX_CMDID_START_PIC:
		return "MTX_CMDID_START_PIC";
	case MTX_CMDID_END_PIC:
		return "MTX_CMDID_END_PIC";
	case MTX_CMDID_DO_HEADER:
		return "MTX_CMDID_DO_HEADER";
	case MTX_CMDID_ENCODE_SLICE:
		return "MTX_CMDID_ENCODE_SLICE";
	case MTX_CMDID_SYNC:
		return "MTX_CMDID_SYNC";

	default:
		return "Undefined command";

	}
}

static inline char *codec_to_string(int codec)
{
	switch (codec) {
	case IMG_CODEC_H264_NO_RC:
		return "H264_NO_RC";
	case IMG_CODEC_H264_VBR:
		return "H264_VBR";
	case IMG_CODEC_H264_CBR:
		return "H264_CBR";
	case IMG_CODEC_H263_NO_RC:
		return "H263_NO_RC";
	case IMG_CODEC_H263_VBR:
		return "H263_VBR";
	case IMG_CODEC_H263_CBR:
		return "H263_CBR";
	case IMG_CODEC_MPEG4_NO_RC:
		return "MPEG4_NO_RC";
	case IMG_CODEC_MPEG4_VBR:
		return "MPEG4_VBR";
	case IMG_CODEC_MPEG4_CBR:
		return "MPEG4_CBR";
	case IMG_CODEC_H264_VCM:
		return "H264_VCM";
	default:
		return "Undefined codec";
	}
}

static inline void lnc_topaz_clearirq(struct drm_device *dev,
				      uint32_t clear_topaz)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_INIT("TOPAZ: clear IRQ\n");
	if (clear_topaz != 0)
		TOPAZ_WRITE32(TOPAZ_CR_IMG_TOPAZ_INTCLEAR, clear_topaz);

	/* PSB_WVDC32(_LNC_IRQ_TOPAZ_FLAG, PSB_INT_IDENTITY_R); */
}

static inline uint32_t lnc_topaz_queryirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	uint32_t val, /* iir, */ clear = 0;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;

	TOPAZ_READ32(TOPAZ_CR_IMG_TOPAZ_INTSTAT, &val);
	/* iir = PSB_RVDC32(PSB_INT_IDENTITY_R); */

	(void)topaz_priv;

	if ((val == 0) /* && (iir == 0) */) {	/* no interrupt */
		PSB_DEBUG_GENERAL("TOPAZ: no interrupt,IIR=TOPAZ_INTSTAT=0\n");
		return 0;
	}

	PSB_DEBUG_IRQ("TOPAZ:TOPAZ_INTSTAT=0x%08x\n", val);

	if (val & (1 << 31))
		PSB_DEBUG_IRQ("TOPAZ:IRQ pin activated,cmd seq=0x%04x,"
			      "sync seq: 0x%08x vs 0x%08x (MTX)\n",
			      CCB_CTRL_SEQ(dev_priv),
			      dev_priv->sequence[LNC_ENGINE_ENCODE],
			      *(uint32_t *) topaz_priv->topaz_sync_addr);
	else
		PSB_DEBUG_IRQ("TOPAZ:IRQ pin not activated,cmd seq=0x%04x,"
			      "sync seq: 0x%08x vs 0x%08x (MTX)\n",
			      CCB_CTRL_SEQ(dev_priv),
			      dev_priv->sequence[LNC_ENGINE_ENCODE],
			      *(uint32_t *) topaz_priv->topaz_sync_addr);

	if (val & 0x8) {
		uint32_t mmu_status, mmu_req;

		TOPAZ_READ32(TOPAZ_CR_MMU_STATUS, &mmu_status);
		TOPAZ_READ32(TOPAZ_CR_MMU_MEM_REQ, &mmu_req);

		PSB_DEBUG_IRQ("TOPAZ: detect a page fault interrupt, "
			      "address=0x%08x,mem req=0x%08x\n",
			      mmu_status, mmu_req);
		clear |= F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MMU_FAULT);
	}

	if (val & 0x4) {
		PSB_DEBUG_IRQ("TOPAZ: detect a MTX_HALT interrupt\n");
		clear |= F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX_HALT);
	}

	if (val & 0x2) {
		PSB_DEBUG_IRQ("TOPAZ: detect a MTX interrupt\n");
		clear |= F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MTX);
	}

	if (val & 0x1) {
		PSB_DEBUG_IRQ("TOPAZ: detect a MVEA interrupt\n");
		clear |= F_ENCODE(1, TOPAZ_CR_IMG_TOPAZ_INTCLR_MVEA);
	}

	return clear;
}

#endif				/* _LNC_TOPAZ_H_ */
