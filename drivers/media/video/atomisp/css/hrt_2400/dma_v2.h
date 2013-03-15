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

#ifndef _hrt_dma_v2_h
#define _hrt_dma_v2_h

//#include <hrt/api.h>
#include "dma_v2_defs.h"

/* Access a dma register at an address */
#define _hrt_dma_v2_slave_port(dma_id) HRTCAT(dma_id,_ctrl)

#define _hrt_dma_v2_set_register(dma_id, addr, val) \
  _hrt_slave_port_store_32_volatile(_hrt_dma_v2_slave_port(dma_id), addr, (val))

#define _hrt_dma_v2_get_register(dma_id, addr) \
  _hrt_slave_port_load_32_volatile(_hrt_dma_v2_slave_port(dma_id), addr)

/* constants */
#define hrt_dma_v2_zero_extension     _DMA_V2_ZERO_EXTEND
#define hrt_dma_v2_sign_extension     _DMA_V2_SIGN_EXTEND

/* Construct address from field type */
#define _hrt_dma_v2_sel_comp(comp)     (((comp)  & _hrt_ones(_DMA_V2_ADDR_SEL_COMP_BITS))            << _DMA_V2_ADDR_SEL_COMP_IDX)
#define _hrt_dma_v2_sel_ch(ch)         (((ch)    & _hrt_ones(_DMA_V2_ADDR_SEL_CH_REG_BITS))          << _DMA_V2_ADDR_SEL_CH_REG_IDX)
#define _hrt_dma_v2_sel_param(param)   (((param) & _hrt_ones(_DMA_V2_ADDR_SEL_PARAM_BITS))           << _DMA_V2_ADDR_SEL_PARAM_IDX)
#define _hrt_dma_v2_sel_cg_info(info)  (((info)  & _hrt_ones(_DMA_V2_ADDR_SEL_GROUP_COMP_INFO_BITS)) << _DMA_V2_ADDR_SEL_GROUP_COMP_INFO_IDX)
#define _hrt_dma_v2_sel_cg_comp(comp)  (((comp)  & _hrt_ones(_DMA_V2_ADDR_SEL_GROUP_COMP_BITS))      << _DMA_V2_ADDR_SEL_GROUP_COMP_IDX)
#define _hrt_dma_v2_sel_dev_info(info) (((info)  & _hrt_ones(_DMA_V2_ADDR_SEL_DEV_INTERF_INFO_BITS)) << _DMA_V2_ADDR_SEL_DEV_INTERF_INFO_IDX)
#define _hrt_dma_v2_sel_dev_id(dev)    (((dev)   & _hrt_ones(_DMA_V2_ADDR_SEL_DEV_INTERF_IDX_BITS))  << _DMA_V2_ADDR_SEL_DEV_INTERF_IDX_IDX)

/* Retrieve return values from packed fields */
#define _hrt_dma_v2_get_connection(val)    _hrt_get_bits(val, _DMA_V2_CONNECTION_IDX,    _DMA_V2_CONNECTION_BITS)
#define _hrt_dma_v2_get_extension(val)     _hrt_get_bits(val, _DMA_V2_EXTENSION_IDX,     _DMA_V2_EXTENSION_BITS)
#define _hrt_dma_v2_get_elements(val)      _hrt_get_bits(val, _DMA_V2_ELEMENTS_IDX,      _DMA_V2_ELEMENTS_BITS)
#define _hrt_dma_v2_get_cropping(val)      _hrt_get_bits(val, _DMA_V2_LEFT_CROPPING_IDX, _DMA_V2_LEFT_CROPPING_BITS)
#define _hrt_dma_v2_get_cmd_ctrl(val)      _hrt_get_bits(val, _DMA_V2_CMD_CTRL_IDX, _DMA_V2_CMD_CTRL_BITS)

#define hrt_dma_v2_command_fsm_register_address \
  _hrt_dma_v2_sel_comp(_DMA_V2_SEL_FSM_CMD)
#define hrt_dma_v2_channel_parameter_register_address(ch, param) \
  (_hrt_dma_v2_sel_comp(_DMA_V2_SEL_CH_REG) | _hrt_dma_v2_sel_ch(ch) | _hrt_dma_v2_sel_param(param))
#define hrt_dma_v2_conn_group_info_register_address(info_id, comp_id) \
  (_hrt_dma_v2_sel_comp(_DMA_V2_SEL_CONN_GROUP) | _hrt_dma_v2_sel_cg_info(info_id) | _hrt_dma_v2_sel_cg_comp(comp_id) )
#define hrt_dma_v2_device_interface_info_register_address(info_id, dev_id) \
  (_hrt_dma_v2_sel_comp(_DMA_V2_SEL_DEV_INTERF) | _hrt_dma_v2_sel_dev_info(info_id) | _hrt_dma_v2_sel_dev_id(dev_id))
#define hrt_dma_v2_reset_register_address \
  _hrt_dma_v2_sel_comp(_DMA_V2_SEL_RESET)

/* Command FSM state */
#define hrt_dma_v2_get_command_fsm_state(dma_id) \
  _hrt_dma_v2_get_register(dma_id, hrt_dma_v2_command_fsm_register_address)

/* Get channel parameters */
#define _hrt_dma_v2_get_channel_parameter(dma_id, ch, param) \
  _hrt_dma_v2_get_register(dma_id, hrt_dma_v2_channel_parameter_register_address(ch, param))
     
#define hrt_dma_v2_get_channel_connection(dma_id, channel) \
  _hrt_dma_v2_get_connection(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_PACKING_SETUP_PARAM))

#define hrt_dma_v2_get_channel_extension(dma_id, channel) \
  _hrt_dma_v2_get_extension(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_PACKING_SETUP_PARAM))

#define hrt_dma_v2_get_channel_stride_A(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_STRIDE_A_PARAM)

#define hrt_dma_v2_get_channel_elements_A(dma_id, channel) \
  _hrt_dma_v2_get_elements(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_A_PARAM))
      
#define hrt_dma_v2_get_channel_cropping_A(dma_id, channel) \
  _hrt_dma_v2_get_cropping(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_A_PARAM))
      
#define hrt_dma_v2_get_channel_width_A(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_WIDTH_A_PARAM)
      
#define hrt_dma_v2_get_channel_stride_B(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_STRIDE_B_PARAM)

#define hrt_dma_v2_get_channel_elements_B(dma_id, channel) \
  _hrt_dma_v2_get_elements(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_B_PARAM))

#define hrt_dma_v2_get_channel_cropping_B(dma_id, channel) \
  _hrt_dma_v2_get_cropping(_hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_B_PARAM))

#define hrt_dma_v2_get_channel_width_B(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_WIDTH_B_PARAM)
      
#define hrt_dma_v2_get_channel_height(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_HEIGHT_PARAM)

#define hrt_dma_v2_get_channel_queued_commands(dma_id, channel) \
  _hrt_dma_v2_get_channel_parameter(dma_id, channel, _DMA_V2_QUEUED_CMDS)

/* Connection group */
#define _hrt_dma_v2_get_conn_group_info(dma_id, info_id, comp_id) \
  _hrt_dma_v2_get_register(dma_id, hrt_dma_v2_conn_group_info_register_address(info_id, comp_id))

#define hrt_dma_v2_get_conn_group_command(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, 0, _DMA_V2_FSM_GROUP_CMD_IDX)

#define hrt_dma_v2_get_conn_group_address_src(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, 0, _DMA_V2_FSM_GROUP_ADDR_SRC_IDX)

#define hrt_dma_v2_get_conn_group_address_dest(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, 0, _DMA_V2_FSM_GROUP_ADDR_DEST_IDX)

#define hrt_dma_v2_get_conn_group_cmd_ctrl(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, 0, _DMA_V2_FSM_GROUP_CMD_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_state(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_request_device(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_REQ_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_request_address(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_REQ_ADDR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_request_stride(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_REQ_STRIDE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_request_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_REQ_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_request_height(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_REQ_YB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_request_device(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_REQ_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_write_device(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_write_address(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_WR_ADDR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_write_stride(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_WR_STRIDE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_request_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_REQ_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_write_height(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_YB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_write_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_request_elems(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_ELEM_REQ_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_pack_write_elems(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_ELEM_WR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX)

#define hrt_dma_v2_get_conn_group_fsm_control_cmd_controller(dma_id) \
  _hrt_dma_v2_get_cmd_ctrl(_hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_CMD_CTRL_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX))

#define hrt_dma_v2_get_conn_group_fsm_control_pack_extension(dma_id) \
  (_hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_CTRL_PACK_S_Z_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX) & 1)\

#define hrt_dma_v2_get_conn_group_fsm_pack_state(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_PACK_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX)

#define hrt_dma_v2_get_conn_group_fsm_pack_counter_height(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_PACK_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX)

#define hrt_dma_v2_get_conn_group_fsm_pack_request_counter_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_PACK_CNT_XB_REQ_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX)

#define hrt_dma_v2_get_conn_group_fsm_pack_write_counter_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_PACK_CNT_XB_WR_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX)

#define hrt_dma_v2_get_conn_group_fsm_request_state(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_REQ_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX)

#define hrt_dma_v2_get_conn_group_fsm_request_counter_height(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_REQ_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX)

#define hrt_dma_v2_get_conn_group_fsm_request_counter_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_REQ_CNT_XB_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX)

#define hrt_dma_v2_get_conn_group_fsm_request_xb_remaining(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_REQ_XB_REMAINING_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_request_burst_cnt(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_REQ_CNT_BURST_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_write_state(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_WR_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_write_height(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_WR_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_write_width(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_WR_CNT_XB_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_write_xb_remaining(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_WR_XB_REMAINING_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

#define hrt_dma_v2_get_conn_group_fsm_write_burst_cnt(dma_id) \
  _hrt_dma_v2_get_conn_group_info(dma_id, _DMA_V2_FSM_GROUP_FSM_WR_CNT_BURST_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX)

/* Device Interface */
#define _hrt_dma_v2_get_device_interface_info(dma_id, info_id, dev_id) \
  _hrt_dma_v2_get_register(dma_id, hrt_dma_v2_device_interface_info_register_address(info_id, dev_id))

#define hrt_dma_v2_get_device_interface_request_side_state(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_REQ_SIDE_STATUS_IDX, dev_id)

#define hrt_dma_v2_get_device_interface_send_side_state(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_SEND_SIDE_STATUS_IDX, dev_id)

#define hrt_dma_v2_get_device_interface_fifo_state(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_FIFO_STATUS_IDX, dev_id)

#define hrt_dma_v2_get_device_interface_req_only_complete_burst(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_REQ_ONLY_COMPLETE_BURST_IDX, dev_id)

#define _hrt_dma_v2_set_device_interface_info(dma_id, info_id, dev_id, value) \
  _hrt_dma_v2_set_register(dma_id, hrt_dma_v2_device_interface_info_register_address(info_id, dev_id), value)

#define hrt_dma_v2_set_device_interface_req_only_complete_burst_on(dma_id, dev_id) \
  _hrt_dma_v2_set_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_REQ_ONLY_COMPLETE_BURST_IDX, dev_id, 1)

#define hrt_dma_v2_set_device_interface_req_only_complete_burst_off(dma_id, dev_id) \
  _hrt_dma_v2_set_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_REQ_ONLY_COMPLETE_BURST_IDX, dev_id, 0)

#define hrt_dma_v2_set_device_interface_max_burst_size(dma_id, dev_id, value) \
  _hrt_dma_v2_set_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_MAX_BURST_IDX, dev_id, value)

#define hrt_dma_v2_get_device_interface_max_burst_size(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_MAX_BURST_IDX, dev_id)

#ifdef HRT_CSIM

#define hrt_dma_v2_set_device_interface_check_addr_alignment_on(dma_id, dev_id) \
  _hrt_dma_v2_set_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_CHK_ADDR_ALIGN, dev_id, 1)

#define hrt_dma_v2_set_device_interface_check_addr_alignment_off(dma_id, dev_id) \
  _hrt_dma_v2_set_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_CHK_ADDR_ALIGN, dev_id, 0)

#define hrt_dma_v2_get_device_interface_check_addr_alignment(dma_id, dev_id) \
  _hrt_dma_v2_get_device_interface_info(dma_id, _DMA_V2_DEV_INTERF_CHK_ADDR_ALIGN, dev_id)

#else

#define hrt_dma_v2_set_device_interface_check_addr_alignment_on(dma_id, dev_id) \
  fprintf(stderr, "Warning DMA RTL does not have check addr aligment feature. Run compile simulation instead\n");

#define hrt_dma_v2_set_device_interface_check_addr_alignment_off(dma_id, dev_id) \
  fprintf(stderr, "Warning DMA RTL does not have check addr aligment feature.\n");

#define hrt_dma_v2_get_device_interface_check_addr_alignment(dma_id, dev_id) 0

#endif
//  fprintf(stderr, "%c", dma_id); 

#define hrt_dma_v2_dump_registers(dma_id, dev_ports, chs) \
{ \
  fprintf(stderr, "DMA Status dump\n\t"); \
{ \
  int dev_id = 0; \
  int chnl_id = 0; \
  int tmp =  hrt_dma_v2_get_command_fsm_state(dma_id); \
  fprintf(stderr, "FSM Command status:\n\t\t"); \
  if (tmp & 0x1) fprintf(stderr, "IDLE\n\t\t"); \
  if (tmp & 0x2) fprintf(stderr, "RUN\n\t\t"); \
  if (tmp & 0x4) fprintf(stderr, "STALL\n\t\t"); \
  if (tmp & 0x8) fprintf(stderr, "ERROR\n\t\t"); \
  fprintf(stderr, "last command received : 0x%x\n\t", (tmp>> 4) & 0xFFFF);\
  fprintf(stderr, "DMA registers, connection group\n\t"); \
  fprintf(stderr, "Cmd Fifo Command             : 0x%x\n\t", hrt_dma_v2_get_conn_group_command(dma_id)); \
  fprintf(stderr, "Cmd Fifo Command from Ctrl   : 0x%x\n\t", hrt_dma_v2_get_conn_group_cmd_ctrl(dma_id)); \
  fprintf(stderr, "Cmd Fifo Address source      : 0x%x\n\t", hrt_dma_v2_get_conn_group_address_src(dma_id)); \
  fprintf(stderr, "Cmd Fifo Address dest        : 0x%x\n\t", hrt_dma_v2_get_conn_group_address_dest(dma_id)); \
  fprintf(stderr, "FSM Ctrl state\n\t\t"); \
  tmp = hrt_dma_v2_get_conn_group_fsm_control_state(dma_id); \
  if (tmp & 0x1) fprintf(stderr, "IDLE -> "); \
  if (tmp & 0x2) fprintf(stderr, "RUN -> "); \
  if (tmp & 0x4) fprintf(stderr, " STALL -> "); \
  if (tmp & 0x8) fprintf(stderr, " ERROR -> "); \
  tmp = tmp >> 4; \
  if (tmp == 0) fprintf(stderr, "Idle state\n\t"); \
  if (tmp == 1) fprintf(stderr, "Req Rcv state\n\t"); \
  if (tmp == 2) fprintf(stderr, "Rcv state\n\t"); \
  if (tmp == 3) fprintf(stderr, "Rcv Req state\n\t"); \
  if (tmp == 4) fprintf(stderr, "Init state\n\t"); \
  fprintf(stderr, "FSM Ctrl command from ctrl   : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_cmd_controller(dma_id)); \
  fprintf(stderr, "FSM Ctrl source dev          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_request_device(dma_id)); \
  fprintf(stderr, "FSM Ctrl source addr         : 0x%x\n\t", hrt_dma_v2_get_conn_group_fsm_control_request_address(dma_id)); \
  fprintf(stderr, "FSM Ctrl source stride       : 0x%x\n\t", hrt_dma_v2_get_conn_group_fsm_control_request_stride(dma_id)); \
  fprintf(stderr, "FSM Ctrl source width        : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_request_width(dma_id)); \
  fprintf(stderr, "FSM Ctrl source height       : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_request_height(dma_id)); \
  fprintf(stderr, "FSM Ctrl pack source dev     : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_request_device(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest device         : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_write_device(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest addr           : 0x%x\n\t", hrt_dma_v2_get_conn_group_fsm_control_write_address(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest stride         : 0x%x\n\t", hrt_dma_v2_get_conn_group_fsm_control_write_stride(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest width          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_request_width(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest height         : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_write_height(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest width          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_write_width(dma_id)); \
  fprintf(stderr, "FSM Ctrl source elems        : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_request_elems(dma_id)); \
  fprintf(stderr, "FSM Ctrl dest elems          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_control_pack_write_elems(dma_id)); \
  fprintf(stderr, "FSM Ctrl pack extension "); \
  fprintf(stderr, (hrt_dma_v2_get_conn_group_fsm_control_pack_extension(dma_id))? "Enabled\n\t":"Disabled\n\t"); \
  fprintf(stderr, "FSM Pack state\n\t\t"); \
  tmp = hrt_dma_v2_get_conn_group_fsm_pack_state(dma_id); \
  if (tmp & 0x1) fprintf(stderr, "IDLE \n\t"); \
  if (tmp & 0x2) fprintf(stderr, "RUN \n\t"); \
  if (tmp & 0x4) fprintf(stderr, " STALL \n\t"); \
  if (tmp & 0x8) fprintf(stderr, " ERROR \n\t"); \
  fprintf(stderr, "FSM Pack cnt height          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_pack_counter_height(dma_id)); \
  fprintf(stderr, "FSM Pack src cnt width       : %d\n\t", hrt_dma_v2_get_conn_group_fsm_pack_request_counter_width(dma_id)); \
  fprintf(stderr, "FSM Pack dest cnt width      : %d\n\t", hrt_dma_v2_get_conn_group_fsm_pack_write_counter_width(dma_id)); \
  fprintf(stderr, "FSM Read state\n\t"); \
  tmp = hrt_dma_v2_get_conn_group_fsm_request_state(dma_id); \
  if ((tmp & 0x3) == 0) fprintf(stderr, "\tIdle state\n\t"); \
  if ((tmp & 0x3) == 1) fprintf(stderr, "\tReq state\n\t"); \
  if ((tmp & 0x3) == 2) fprintf(stderr, "\tNext line\n\t"); \
  fprintf(stderr, "FSM Read Split burst state\n\t\t"); \
  tmp = tmp >> 4; \
  if (tmp == 0) fprintf(stderr, "Idle state\n\t"); \
  if (tmp == 1) fprintf(stderr, "Split burst state\n\t"); \
  if (tmp == 2) fprintf(stderr, "Last burst state\n\t"); \
  fprintf(stderr, "FSM Read cnt height          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_request_counter_height(dma_id)); \
  fprintf(stderr, "FSM Read cnt width           : %d\n\t", hrt_dma_v2_get_conn_group_fsm_request_counter_width(dma_id)); \
  fprintf(stderr, "FSM Read cnt word remaining  : %d\n\t", hrt_dma_v2_get_conn_group_fsm_request_xb_remaining(dma_id)); \
  fprintf(stderr, "FSM Read cnt burst           : %d\n\t", hrt_dma_v2_get_conn_group_fsm_request_burst_cnt(dma_id)); \
  fprintf(stderr, "FSM Write state\n\t\t"); \
  tmp = hrt_dma_v2_get_conn_group_fsm_write_state(dma_id); \
  if ((tmp & 0x3) == 0) fprintf(stderr, "Idle state\n\t"); \
  if ((tmp & 0x3) == 1) fprintf(stderr, "Req state\n\t"); \
  if ((tmp & 0x3) == 2) fprintf(stderr, "Next line\n\t"); \
  fprintf(stderr, "FSM Write Split burst state\n\t\t"); \
  tmp = tmp >> 4; \
  if (tmp == 0) fprintf(stderr, "Idle state\n\t"); \
  if (tmp == 1) fprintf(stderr, "Split burst state\n\t"); \
  if (tmp == 2) fprintf(stderr, "Last burst state\n\t"); \
  fprintf(stderr, "FSM Write height             : %d\n\t", hrt_dma_v2_get_conn_group_fsm_write_height(dma_id)); \
  fprintf(stderr, "FSM Write width              : %d\n\t", hrt_dma_v2_get_conn_group_fsm_write_width(dma_id)); \
  fprintf(stderr, "FSM Write cnt word remaining : %d\n\t", hrt_dma_v2_get_conn_group_fsm_write_xb_remaining(dma_id)); \
  fprintf(stderr, "FSM Write cnt burst          : %d\n\t", hrt_dma_v2_get_conn_group_fsm_write_burst_cnt(dma_id)); \
  fprintf(stderr, "\n\t"); \
  for (dev_id = 0; dev_id < dev_ports; dev_id++) { \
    fprintf(stderr, "DMA device interface %d\n", dev_id); \
    fprintf(stderr, "\t\tRequest side state\n\t\t\t"); \
    tmp = hrt_dma_v2_get_device_interface_request_side_state(dma_id, dev_id); \
    fprintf(stderr, "CS: %d - We_n: %d - Run: %d - Ack: %d\n", (tmp & 0x1)!=0, (tmp & 0x2)!=0, (tmp & 0x4)!=0, (tmp & 0x8)!=0 ); \
    fprintf(stderr, "\t\tOutput side state\n\t\t\t"); \
    tmp = hrt_dma_v2_get_device_interface_send_side_state(dma_id, dev_id); \
    fprintf(stderr, "CS: %d - We_n: %d - Run: %d - Ack: %d\n", (tmp & 0x1)!=0, (tmp & 0x2)!=0, (tmp & 0x4)!=0, (tmp & 0x8)!=0 ); \
    fprintf(stderr, "\t\tFifo state\n\t\t\t"); \
    tmp = hrt_dma_v2_get_device_interface_fifo_state(dma_id, dev_id); \
    if (tmp & 0x1) fprintf(stderr, "FiFo will be full\n"); \
    if (tmp & 0x2) fprintf(stderr, "Fifo Full\n"); \
    if (tmp & 0x4) fprintf(stderr, "Fifo Empty\n"); \
    fprintf(stderr, "\t\tFifo counter %d \n\t\t", tmp >> 3); \
    fprintf(stderr, hrt_dma_v2_get_device_interface_req_only_complete_burst(dma_id, dev_id)? "Do " : "Don`t "); \
    fprintf(stderr, "request only complete burst\n\t"); \
    tmp = hrt_dma_v2_get_device_interface_max_burst_size(dma_id, dev_id); \
    fprintf(stderr, "\t\tMax burst: %d \n", tmp); \
  } \
  fprintf(stderr, "\n\t"); \
  for (chnl_id = 0; chnl_id < chs; chnl_id++) { \
    fprintf(stderr, "DMA channel register %d\n\t\t", chnl_id); \
    fprintf(stderr, "Queued commands : %d\n\t\t", hrt_dma_v2_get_channel_queued_commands(dma_id, chnl_id)); \
    fprintf(stderr, "Connection      : %d\n\t\t", hrt_dma_v2_get_channel_connection(dma_id, chnl_id)); \
    fprintf(stderr, (hrt_dma_v2_get_channel_extension(dma_id, chnl_id))? "Sign Extension Enabled\n\t\t":"Sign Extension Disabled\n\t\t"); \
    fprintf(stderr, "Stride Dev A    : 0x%x\n\t\t", hrt_dma_v2_get_channel_stride_A(dma_id, chnl_id)); \
    fprintf(stderr, "Elems Dev A     : %d\n\t\t", hrt_dma_v2_get_channel_elements_A(dma_id, chnl_id)); \
    fprintf(stderr, "Cropping Dev A  : %d\n\t\t", hrt_dma_v2_get_channel_cropping_A(dma_id, chnl_id)); \
    fprintf(stderr, "Width Dev A     : %d\n\t\t", hrt_dma_v2_get_channel_width_A(dma_id, chnl_id)); \
    fprintf(stderr, "Stride Dev B    : 0x%x\n\t\t", hrt_dma_v2_get_channel_stride_B(dma_id, chnl_id)); \
    fprintf(stderr, "Elems Dev B     : %d\n\t\t", hrt_dma_v2_get_channel_elements_B(dma_id, chnl_id)); \
    fprintf(stderr, "Cropping Dev B  : %d\n\t\t", hrt_dma_v2_get_channel_cropping_B(dma_id, chnl_id)); \
    fprintf(stderr, "Width Dev B     : %d\n\t\t", hrt_dma_v2_get_channel_width_B(dma_id, chnl_id)); \
    fprintf(stderr, "Height          : %d\n\t", hrt_dma_v2_get_channel_height(dma_id, chnl_id)); \
  } \
  fprintf(stderr, "\n"); \
  } \
}


/* Reset */
#define hrt_dma_v2_slave_reset(dma_id) \
  _hrt_dma_v2_set_register(dma_id, hrt_dma_v2_reset_register_address, _DMA_V2_RESET_TOKEN)

#define hrt_dma_v2_slave_get_reset(dma_id) \
  _hrt_dma_v2_get_register(dma_id, hrt_dma_v2_reset_register_address)

#endif /* _hrt_dma_v2_h */
