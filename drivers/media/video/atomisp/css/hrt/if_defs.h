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

#ifndef _IF_DEFS_H
#define _IF_DEFS_H

/* Hardware registers */
#define HIVE_IF_RESET_ADDRESS                   0x000
#define HIVE_IF_START_LINE_ADDRESS              0x004
#define HIVE_IF_START_COLUMN_ADDRESS            0x008
#define HIVE_IF_CROPPED_HEIGHT_ADDRESS          0x00C
#define HIVE_IF_CROPPED_WIDTH_ADDRESS           0x010
#define HIVE_IF_VERTICAL_DECIMATION_ADDRESS     0x014
#define HIVE_IF_HORIZONTAL_DECIMATION_ADDRESS   0x018
#define HIVE_IF_H_DEINTERLEAVING_ADDRESS        0x01C
#define HIVE_IF_LEFTPADDING_WIDTH_ADDRESS       0x020
#define HIVE_IF_END_OF_LINE_OFFSET_ADDRESS      0x024
#define HIVE_IF_VMEM_START_ADDRESS_ADDRESS      0x028
#define HIVE_IF_VMEM_END_ADDRESS_ADDRESS        0x02C
#define HIVE_IF_VMEM_INCREMENT_ADDRESS          0x030
#define HIVE_IF_YUV_420_FORMAT_ADDRESS          0x034
#define HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS       0x038
#define HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS       0x03C
#define HIVE_IF_ALLOW_FIFO_OVERFLOW_ADDRESS     0x040
#define HIVE_IF_BLOCK_FIFO_NO_REQ_ADDRESS       0x044
#define HIVE_IF_V_DEINTERLEAVING_ADDRESS        0x048
#define HIVE_IF_FSM_SYNC_STATUS                 0x100
#define HIVE_IF_FSM_SYNC_COUNTER                0x104
#define HIVE_IF_FSM_CROP_STATUS                 0x108
#define HIVE_IF_FSM_CROP_LINE_COUNTER           0x10C
#define HIVE_IF_FSM_CROP_PIXEL_COUNTER          0x110
#define HIVE_IF_FSM_DEINTERLEAVING_IDX          0x114
#define HIVE_IF_FSM_DECIMATION_H_COUNTER        0x118
#define HIVE_IF_FSM_DECIMATION_V_COUNTER        0x11C
#define HIVE_IF_FSM_DECIMATION_BLOCK_V_COUNTER  0x120
#define HIVE_IF_FSM_PADDING_STATUS              0x124
#define HIVE_IF_FSM_PADDING_ELEMENT_COUNTER     0x128
#define HIVE_IF_FSM_VECTOR_SUPPORT_ERROR        0x12C
#define HIVE_IF_FSM_VECTOR_SUPPORT_BUFF_FULL    0x130
#define HIVE_IF_FSM_VECTOR_SUPPORT              0x134
#define HIVE_IF_FIFO_SENSOR_DATA_LOST           0x138

/* Registers only for simulation */
#define HIVE_IF_CRUN_MODE_ADDRESS               0x04C
#define HIVE_IF_DUMP_OUTPUT_ADDRESS             0x050

#define HIVE_IF_FRAME_REQUEST        0xA000
#define HIVE_IF_LINES_REQUEST        0xB000
#define HIVE_IF_VECTORS_REQUEST      0xC000

#define _HRT_IF_VEC_ALIGN(if_id) HRTCAT(if_id, _vector_alignment)

#endif /* _IF_DEFS_H */
