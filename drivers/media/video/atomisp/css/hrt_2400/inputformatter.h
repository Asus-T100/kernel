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

#ifndef _HRT_INPUTFORMATTER_H
#define _HRT_INPUTFORMATTER_H

//#include <hrt/api.h>

#define _HRT_IF_VEC_ALIGN(if_id) HRTCAT(if_id, _vector_alignment)

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
/* Registers only for simulation */
#define HIVE_IF_CRUN_MODE_ADDRESS               0x04C
#define HIVE_IF_DUMP_OUTPUT_ADDRESS             0x050

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
#define HIVE_IF_FIFO_SENSOR_STATUS              0x138


#define hrt_if_slave_port(if_id) HRTCAT(if_id, _sl_in)

#define _hrt_if_set_register(if_id, addr, val) \
  _hrt_slave_port_store_32_volatile( hrt_if_slave_port(if_id), (addr), (val))

#define _hrt_if_get_register(if_id, addr) \
  _hrt_slave_port_load_32_volatile(hrt_if_slave_port(if_id), (addr))

#define hrt_if_set_start_line(if_id, line) \
  _hrt_if_set_register(if_id, HIVE_IF_START_LINE_ADDRESS, line)

#define hrt_if_set_start_column(if_id, column) \
  _hrt_if_set_register(if_id, HIVE_IF_START_COLUMN_ADDRESS, column)

#define hrt_if_set_cropped_height(if_id, height) \
  _hrt_if_set_register(if_id, HIVE_IF_CROPPED_HEIGHT_ADDRESS, height)

#define hrt_if_set_cropped_width(if_id, width) \
  _hrt_if_set_register(if_id, HIVE_IF_CROPPED_WIDTH_ADDRESS, width)

#define hrt_if_set_vertical_decimation(if_id, v_dec) \
  _hrt_if_set_register(if_id, HIVE_IF_VERTICAL_DECIMATION_ADDRESS, v_dec)

#define hrt_if_set_horizontal_decimation(if_id, h_dec) \
  _hrt_if_set_register(if_id, HIVE_IF_HORIZONTAL_DECIMATION_ADDRESS, h_dec)

#define hrt_if_set_v_deinterleaving(if_id, inter) \
  _hrt_if_set_register(if_id, HIVE_IF_V_DEINTERLEAVING_ADDRESS, inter)

#define hrt_if_set_h_deinterleaving(if_id, inter) \
  _hrt_if_set_register(if_id, HIVE_IF_H_DEINTERLEAVING_ADDRESS, inter)

#define hrt_if_set_leftpadding_width(if_id, width) \
  _hrt_if_set_register(if_id, HIVE_IF_LEFTPADDING_WIDTH_ADDRESS, width)

#define hrt_if_set_deinterleaving(if_id, inter) \
{ \
  hrt_if_set_v_deinterleaving(if_id, inter); \
  hrt_if_set_h_deinterleaving(if_id, inter); \
}

#define hrt_if_set_vmem_start_address(if_id, addr) \
  _hrt_if_set_register(if_id, HIVE_IF_VMEM_START_ADDRESS_ADDRESS, addr)

#define hrt_if_get_start_line(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_START_LINE_ADDRESS)

#define hrt_if_get_start_column(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_START_COLUMN_ADDRESS)

#define hrt_if_get_cropped_height(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_CROPPED_HEIGHT_ADDRESS)

#define hrt_if_get_cropped_width(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_CROPPED_WIDTH_ADDRESS)

#define hrt_if_get_vertical_decimation(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_VERTICAL_DECIMATION_ADDRESS)

#define hrt_if_get_horizontal_decimation(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_HORIZONTAL_DECIMATION_ADDRESS)

// this one will became obsolete
#define hrt_if_get_deinterleaving(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_H_DEINTERLEAVING_ADDRESS)

#define hrt_if_get_v_deinterleaving(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_V_DEINTERLEAVING_ADDRESS)

#define hrt_if_get_h_deinterleaving(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_H_DEINTERLEAVING_ADDRESS)

#define hrt_if_get_leftpadding_width(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_LEFTPADDING_WIDTH_ADDRESS)

#define hrt_if_get_vmem_start_address(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_VMEM_START_ADDRESS_ADDRESS)

#define hrt_if_get_end_of_line_offset(ift) \
  _hrt_if_get_register(ift, HIVE_IF_END_OF_LINE_OFFSET_ADDRESS)

#define hrt_if_get_vmem_end_address(ift) \
  _hrt_if_get_register(ift, HIVE_IF_VMEM_END_ADDRESS_ADDRESS)
  
#define hrt_if_get_allow_fifo_overflow(ift) \
  _hrt_if_get_register(ift, HIVE_IF_ALLOW_FIFO_OVERFLOW_ADDRESS)

#define hrt_if_get_vmem_increment(ift) \
  _hrt_if_get_register(ift, HIVE_IF_VMEM_INCREMENT_ADDRESS)
  
#define hrt_if_get_fsm_sync_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_SYNC_COUNTER)

#define hrt_if_get_fsm_crop_status(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_CROP_STATUS)
  
#define hrt_if_get_fsm_crop_line_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_CROP_LINE_COUNTER)

#define hrt_if_get_fsm_crop_pixel_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_CROP_PIXEL_COUNTER)
  
#define hrt_if_get_fsm_deinterleaving_index(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_DEINTERLEAVING_IDX)

#define hrt_if_get_fsm_dec_h_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_DECIMATION_H_COUNTER)

#define hrt_if_get_fsm_dec_v_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_DECIMATION_V_COUNTER)

#define hrt_if_get_fsm_dec_block_v_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_DECIMATION_BLOCK_V_COUNTER)

#define hrt_if_get_fsm_padding_status(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_PADDING_STATUS)

#define hrt_if_get_fsm_padding_elem_counter(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_PADDING_ELEMENT_COUNTER)

#define hrt_if_get_fsm_vector_support_error(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_VECTOR_SUPPORT_ERROR)

#define hrt_if_get_fsm_vector_buffer_full(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_VECTOR_SUPPORT_BUFF_FULL)

#define hrt_if_get_vector_support(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FSM_VECTOR_SUPPORT)

#define hrt_if_get_sensor_data_lost(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FIFO_SENSOR_STATUS)

#define hrt_if_get_sensor_status(ift) \
  _hrt_if_get_register(ift, HIVE_IF_FIFO_SENSOR_STATUS)

#define hrt_if_get_reset(ift) \
  _hrt_if_get_register(ift, HIVE_IF_RESET_ADDRESS)

/* adding 3 dummy writes after the reset to return control to the caller after the IF has finish resetting */ 
#define hrt_if_reset(if_id) \
{ \
  _hrt_if_set_register(if_id, HIVE_IF_RESET_ADDRESS, 1); \
  hrt_if_set_start_line(if_id, 0); \
  hrt_if_set_start_column(if_id, 0); \
  hrt_if_set_vmem_start_address(if_id, 0); \
}

#define hrt_if_en_stat_update(if_id) \
{ \
  _hrt_if_set_register(if_id, HIVE_IF_EN_STAT_UPDATE, 0); \
  _hrt_if_set_register(if_id, HIVE_IF_EN_STAT_UPDATE, 1); \
}

#define hrt_if_get_fsm_sync_status(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_FSM_SYNC_STATUS)

#ifdef _HRT_LEGACY_INPUTFORMATTER
#define hrt_if_set_vmem_end_address(if_id, addr) \
  _hrt_if_set_register(if_id, HIVE_IF_VMEM_END_ADDRESS_ADDRESS, (addr)+1)
#else
#define hrt_if_set_vmem_end_address(if_id, addr) \
  _hrt_if_set_register(if_id, HIVE_IF_VMEM_END_ADDRESS_ADDRESS, addr)
#endif

#define hrt_if_set_vmem_increment(if_id, incr) \
  _hrt_if_set_register(if_id, HIVE_IF_VMEM_INCREMENT_ADDRESS, incr)

#define hrt_if_allow_sensor_stall(if_id, val) \
  _hrt_if_set_register(if_id, HIVE_IF_ALLOW_FIFO_OVERFLOW_ADDRESS, val)

#define hrt_if_block_fifo_no_reqs(if_id, val) \
  _hrt_if_set_register(if_id, HIVE_IF_BLOCK_FIFO_NO_REQ_ADDRESS, val)

#define hrt_if_set_yuv_420_format(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS, 1)
#define hrt_if_unset_yuv_420_format(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS, 0)
#define hrt_if_get_yuv_420_format(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS)

#define hrt_if_set_vsynck_active_low(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS, 1)
#define hrt_if_set_vsynck_active_high(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS, 0)
#define hrt_if_get_vsynck_active_low(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS)

#define hrt_if_set_hsynck_active_low(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS, 1)
#define hrt_if_set_hsynck_active_high(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS, 0)
#define hrt_if_get_hsynck_active_low(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS)

#define hrt_if_set_end_of_line_offset(if_id, offset) \
  _hrt_if_set_register(if_id, HIVE_IF_END_OF_LINE_OFFSET_ADDRESS, offset)

/* this one only applies to the simulation model, it returns an address of
   the output buffer used internally in the model. do not use if you don't know
   what it's for. */
#define hrt_if_dump_output(if_id) \
  ((unsigned short*)_hrt_if_get_register(if_id, HIVE_IF_DUMP_OUTPUT_ADDRESS))

#define hrt_if_set_yuv_420_format(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS, 1)
#define hrt_if_unset_yuv_420_format(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS, 0)
#define hrt_if_get_yuv_420_format(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_YUV_420_FORMAT_ADDRESS)

#define hrt_if_set_vsynck_active_low(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS, 1)
#define hrt_if_set_vsynck_active_high(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS, 0)
#define hrt_if_get_vsynck_active_low(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS)

#define hrt_if_set_hsynck_active_low(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS, 1)
#define hrt_if_set_hsynck_active_high(if_id) \
  _hrt_if_set_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS, 0)
#define hrt_if_get_hsynck_active_low(if_id) \
  _hrt_if_get_register(if_id, HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS)

/* this hrt_if_set_vector_buffer_addresses function does not differ between crun and other
   runs. It should be used when the destination of the vectors is a block that doesn't do
   crun. An example of this is when the IF writes directly to external memory, not to a
   hive processor.
   Please use with care. */
#define hrt_if_set_vector_buffer_addresses(if_id, start_addr, num_vectors, start_index, incr, eol_offset) \
{ \
  hrt_if_set_vmem_start_address(if_id, (unsigned)(start_addr) + (start_index) * _HRT_IF_VEC_ALIGN(if_id)); \
  hrt_if_set_vmem_end_address  (if_id, (unsigned)(start_addr) + (num_vectors) * _HRT_IF_VEC_ALIGN(if_id)); \
  hrt_if_set_end_of_line_offset(if_id, (incr)*_HRT_IF_VEC_ALIGN(if_id) + (eol_offset)); \
  hrt_if_set_vmem_increment    (if_id, incr); \
}

#ifdef C_RUN
#define hrt_if_set_crun_mode(if_id) _hrt_if_set_register(if_id, HIVE_IF_CRUN_MODE_ADDRESS, 1)
#define _hrt_if_buf_address(vector_array, byte_offset) (_hrt_cell_get_crun_indexed_symbol(ISP,vector_array))
#else
#define hrt_if_set_crun_mode(if_id)
#define _hrt_if_buf_address(vector_array, byte_offset) (HRTCAT(HIVE_ADDR_,vector_array) + (byte_offset))
#endif

/* set vector buffer vector_array of num_vectors vectors, start at index start_index and increment
   by incr vectors after every write of 1 vector.
   Basic example where the IF writes the entire input_buf, one vector at a time:
     configure_vector_buffer(IF, input_buf, 8, 0, 1)
   More complicated example where the IF writes vectors at indices 1, 3, 5, 7:
     configure_vector_buffer(IF, input_buf, 8, 1, 2)
   to write indices 0, 1, 2, 6:
     configure_vector_buffer(IF, input_buf, 8, 0, 2)

   The eol_offset specifies an extra offset to be added at the end of every line. This offset
   is in bytes. The default value should be 0.
 */
#define hrt_if_configure_vector_buffer(if_id, vector_array, num_vectors, start_index, incr, eol_offset) \
{ \
  hrt_if_set_vector_buffer_addresses(if_id, _hrt_if_buf_address(vector_array, 0), num_vectors, start_index, incr, eol_offset); \
  hrt_if_set_crun_mode(if_id); \
}

/* This function has a byte offset which is used if the master port needs to go onto
   a bus to get to a certain memory, the byte offset is then the start address of the
   memory. In crun, we do not add this offset since we are writing directly into the
   buffer there. */
#define hrt_if_set_vector_buffer_with_offset(if_id, buf, num_vectors, byte_offset) \
{ \
  void *dummy = (void*)(byte_offset); \
  dummy = dummy; /* not used in crun */\
  hrt_if_set_vmem_start_address(if_id, (unsigned)_hrt_if_buf_address(buf, byte_offset)); \
  hrt_if_set_vmem_end_address  (if_id, (unsigned)_hrt_if_buf_address(buf, byte_offset) + (num_vectors) * _HRT_IF_VEC_ALIGN(if_id)); \
  hrt_if_set_end_of_line_offset(if_id, _HRT_IF_VEC_ALIGN(if_id)); \
  hrt_if_set_vmem_increment    (if_id, 1); \
  hrt_if_set_crun_mode         (if_id); \
}

#define hrt_if_set_vector_buffer(if_id, buf, vecs) \
  hrt_if_configure_vector_buffer(if_id, buf, vecs, 0, 1, 0)

#endif /* _HRT_INPUTFORMATTER_H */
