/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
*
* Copyright (c) 2010 Intel Corporation. All Rights Reserved.
*
* Copyright (c) 2010 Silicon Hive www.siliconhive.com.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License version
* 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
* 02110-1301, USA.
*
*/

#ifndef _SH_CSS_HW_H_
#define _SH_CSS_HW_H_

#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#ifdef HRT_ISP_CSS_CUSTOM_HOST
#ifndef HRT_USE_VIR_ADDRS
#define HRT_USE_VIR_ADDRS
#endif
#include <hive_isp_css_custom_host_hrt.h>
#endif

#include <hrt/master_port.h>
#include <hrt/bits.h>

#include <sp_hrt.h>
#include <irq_controller_defs.h>
#include <if_defs.h>
#if defined(SYSTEM_hive_isp_css_2400_system)
#include <dma_v2_defs.h>
#else
#include <dma_v1_defs.h>
#endif
#include <gp_regs_defs.h>
#include <mmu_defs.h>
#if defined(SYSTEM_hive_isp_css_2400_system)
#include <css_receiver_2400_defs.h>
#include <input_system_defs.h>
#include <input_formatter_subsystem_defs.h>
#else
#include <css_receiver_ahb_defs.h>
#endif
#include <hive_isp_css_streaming_monitors_types_hrt.h>
#include <hive_isp_css_if_hrt.h>
#include <hive_isp_css_streaming_to_mipi_types_hrt.h>

#if defined(SYSTEM_hive_isp_css_2400_system)
#define MMU_BASE       ((void *)0x00070000)
#define GDC_LUT_BASE   ((void *)0x00050000)
#define GDC2_LUT_BASE  ((void *)0x00060000)
#define DMA_BASE       ((void *)0x00040000)
#define IF_PRIM_A_BASE ((void *)0x00030000)
#define IF_PRIM_B_BASE ((void *)0x00030200)
#define IF_SEC_BASE    ((void *)0x00030400)
#define SP_DMEM_BASE   ((void *)0x00300000)
#define SP_SC_BASE     ((void *)0x00010000)
#define ISP_SC_BASE    ((void *)0x00020000)
#define STR_MON_BASE   ((void *)0x00000000)
#define IRQ_CTRL_BASE  ((void *)0x00000500)
//#define GP_FIFO_BASE   ((void *)0x10200304)     /* different set-up */
#define GP_FIFO_BASE   ((void *)0x00090104)
#define CSS_RX_BASE    ((void *)0x00080000)
#else
#define MMU_BASE       ((void *)0x10250000)
#define GDC_LUT_BASE   ((void *)0x10180000)
#define DMA_BASE       ((void *)0x10240000)
#define IF_PRIM_A_BASE ((void *)0x10210000)
#define IF_PRIM_B_BASE ((void *)0x10270000)
#define SP_DMEM_BASE   ((void *)0x10100000)
#define SP_SC_BASE     ((void *)0x10104000)
#define ISP_SC_BASE    ((void *)0x10020000)
#define STR_MON_BASE   ((void *)0x10200000)
#define IRQ_CTRL_BASE  ((void *)0x10200500)
#define GP_FIFO_BASE   ((void *)0x10200304)
#define CSS_RX_BASE    ((void *)0x10260000)

#endif

#define SP_DMEM_SIZE        (0x4000)
/* SP Registers */
#define SP_PC_REG        0x9
#define SP_SC_REG        0x0
#define SP_START_ADDR_REG  0x1
#define SP_ICACHE_ADDR_REG 0x5
#define SP_IRQ_READY_REG   0x0
#define SP_IRQ_CLEAR_REG   0x0
#define SP_ICACHE_INV_REG  0x0
#define SP_CTRL_SINK_REG   0xA

/* SP Register bits */
#if defined(SYSTEM_hive_isp_css_2400_system)
#define SP_RST_BIT          0x0
#define SP_START_BIT        0x1
#define SP_BREAK_BIT        0x2
#define SP_RUN_BIT          0x3
#define SP_BROKEN_BIT       0x4
#define SP_IDLE_BIT         0x5     /* READY */
#define SP_STALLING_BIT     0x7
#define SP_IRQ_CLEAR_BIT    0x8
#define SP_IRQ_READY_BIT    0xA
#define SP_SLEEPING_BIT     0xB     /* SLEEPING_IRQ_MASK */

#define SP_ICACHE_INV_BIT   0xC
#define SP_IPREFETCH_EN_BIT 0xD
#else
#define SP_START_BIT       0x1
#define SP_RUN_BIT         0x3
#define SP_BROKEN_BIT      0x4
#define SP_IDLE_BIT        0x5
#define SP_STALLING_BIT    0x7
#define SP_IRQ_CLEAR_BIT   0x8
#define SP_IRQ_READY_BIT   0xA
#define SP_SLEEPING_BIT    0xB
#define SP_ICACHE_INV_BIT  0xC
#endif

#if defined(SYSTEM_hive_isp_css_2400_system)
/* ISP SC Registers */
#define ISP_PC_REG          0x7
#define ISP_SC_REG          0x0
/* ISP SC Register bits */
#define ISP_RST_BIT         0x0
#define ISP_START_BIT       0x1
#define ISP_BREAK_BIT       0x2
#define ISP_RUN_BIT         0x3
#define ISP_BROKEN_BIT      0x4
#define ISP_IDLE_BIT        0x5     /* READY */
#define ISP_STALLING_BIT    0x7
#define ISP_IRQ_CLEAR_BIT   0x8
#define ISP_IRQ_READY_BIT   0xA
#define ISP_SLEEPING_BIT    0xB     /* SLEEPING_IRQ_MASK */

/* ISP Register bits */
#define ISP_CTRL_SINK_BIT   0x0
#define ISP_PMEM_SINK_BIT   0x1
#define ISP_DMEM_SINK_BIT   0x2
#define ISP_FIFO0_SINK_BIT  0x3
#define ISP_FIFO1_SINK_BIT  0x4
#define ISP_FIFO2_SINK_BIT  0x5
#define ISP_FIFO3_SINK_BIT  0x6
#define ISP_FIFO4_SINK_BIT  0x7
#define ISP_FIFO5_SINK_BIT  0x8
#define ISP_FIFO6_SINK_BIT  0x9
#define ISP_VMEM_SINK_BIT   0xA
#define ISP_VAMEM1_SINK_BIT 0xB
#define ISP_VAMEM2_SINK_BIT 0xC
#define ISP_VAMEM3_SINK_BIT 0xD
#define ISP_HMEM_SINK_BIT   0xE

#define ISP_CTRL_SINK_REG   0x8
#define ISP_PMEM_SINK_REG   0x8
#define ISP_DMEM_SINK_REG   0x8
#define ISP_FIFO0_SINK_REG  0x8
#define ISP_FIFO1_SINK_REG  0x8
#define ISP_FIFO2_SINK_REG  0x8
#define ISP_FIFO3_SINK_REG  0x8
#define ISP_FIFO4_SINK_REG  0x8
#define ISP_FIFO5_SINK_REG  0x8
#define ISP_FIFO6_SINK_REG  0x8
#define ISP_VMEM_SINK_REG   0x8
#define ISP_VAMEM1_SINK_REG 0x8
#define ISP_VAMEM2_SINK_REG 0x8
#define ISP_VAMEM3_SINK_REG 0x8
#define ISP_HMEM_SINK_REG   0x8
#else
/* ISP Registers */
#define ISP_PC_REG       0x5
#define ISP_SC_REG       0x0
#define ISP_BROKEN_BIT   0x4
#define ISP_IDLE_BIT     0x5
#define ISP_STALLING_BIT 0x7
#define ISP_SLEEPING_BIT 0xB

/* ISP Register bits */
#define ISP_CTRL_SINK_BIT   0x0
#define ISP_DMEM_SINK_BIT   0x1
#define ISP_VMEM_SINK_BIT   0x2
#define ISP_FIFO0_SINK_BIT  0x3
#define ISP_FIFO1_SINK_BIT  0x4
#define ISP_FIFO2_SINK_BIT  0x5
#define ISP_FIFO3_SINK_BIT  0x6
#define ISP_FIFO4_SINK_BIT  0x7
#define ISP_FIFO5_SINK_BIT  0x8
#define ISP_VAMEM1_SINK_BIT 0x9
#define ISP_VAMEM2_SINK_BIT 0xA

#define ISP_CTRL_SINK_REG   0x6
#define ISP_DMEM_SINK_REG   0x6
#define ISP_VMEM_SINK_REG   0x6
#define ISP_FIFO0_SINK_REG  0x6
#define ISP_FIFO1_SINK_REG  0x6
#define ISP_FIFO2_SINK_REG  0x6
#define ISP_FIFO3_SINK_REG  0x6
#define ISP_FIFO4_SINK_REG  0x6
#define ISP_FIFO5_SINK_REG  0x6
#define ISP_VAMEM1_SINK_REG 0x6
#define ISP_VAMEM2_SINK_REG 0x6
#endif

#define SP_FIFO0_SINK_BIT     0x0
#define SP_FIFO1_SINK_BIT     0x1
#define SP_FIFO2_SINK_BIT     0x2
#define SP_FIFO3_SINK_BIT     0x3
#define SP_FIFO4_SINK_BIT     0x4
#define SP_FIFO5_SINK_BIT     0x5
#define SP_FIFO6_SINK_BIT     0x6
#define SP_FIFO7_SINK_BIT     0x7
#if defined(SYSTEM_hive_isp_css_2400_system)
#define SP_FIFO8_SINK_BIT     0x8
#define SP_FIFO9_SINK_BIT     0x9
#define SP_FIFOA_SINK_BIT     0xA
#define SP_DMEM_SINK_BIT      0xB
#define SP_CTRL_MT_SINK_BIT   0xC
#define SP_ICACHE_MT_SINK_BIT 0xD
#else
#define SP_DMEM_SINK_BIT      0x8
#define SP_CTRL_MT_SINK_BIT   0x9
#define SP_ICACHE_MT_SINK_BIT 0xA
#endif

#define SP_FIFO0_SINK_REG     0xA
#define SP_FIFO1_SINK_REG     0xA
#define SP_FIFO2_SINK_REG     0xA
#define SP_FIFO3_SINK_REG     0xA
#define SP_FIFO4_SINK_REG     0xA
#define SP_FIFO5_SINK_REG     0xA
#define SP_FIFO6_SINK_REG     0xA
#define SP_FIFO7_SINK_REG     0xA
#if defined(SYSTEM_hive_isp_css_2400_system)
#define SP_FIFO8_SINK_REG     0xA
#define SP_FIFO9_SINK_REG     0xA
#define SP_FIFOA_SINK_REG     0xA
#endif
#define SP_DMEM_SINK_REG      0xA
#define SP_CTRL_MT_SINK_REG   0xA
#define SP_ICACHE_MT_SINK_REG 0xA

#endif /* _SH_CSS_HW_H_ */
