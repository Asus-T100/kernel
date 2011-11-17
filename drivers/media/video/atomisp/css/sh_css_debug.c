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

#include "sh_css_debug.h"
#include "sh_css.h"
#include "sh_css_hrt.h"
#include "sh_css_internal.h"
#include "sh_css_rx.h"

static void
print_cell_state(struct sh_css_cell_state *state, const char *cell)
{
	sh_css_print("%s state:\n", cell);
	sh_css_print("  PC: 0x%x\n", state->pc);
	sh_css_print(" Status register : 0x%x\n", state->status_register);
	sh_css_print(" is broken:   %d\n", state->is_broken);
	sh_css_print(" is idle:    %d\n", state->is_idle);
	sh_css_print(" is sleeping: %d\n", state->is_sleeping);
	sh_css_print(" is stalling: %d\n", state->is_stalling);
}

void
sh_css_dump_isp_state(void)
{
	struct sh_css_cell_state state;
	struct sh_css_isp_stall_state stall_state;
	sh_css_hrt_isp_get_state(&state, &stall_state);
	print_cell_state(&state, "ISP");
	if (state.is_stalling) {
		sh_css_print("  FIFO 0 stalled: %d\n", stall_state.fifo0);
		sh_css_print("  FIFO 1 stalled: %d\n", stall_state.fifo1);
		sh_css_print("  FIFO 2 stalled: %d\n", stall_state.fifo2);
		sh_css_print("  FIFO 3 stalled: %d\n", stall_state.fifo3);
		sh_css_print("  FIFO 4 stalled: %d\n", stall_state.fifo4);
		sh_css_print("  FIFO 5 stalled: %d\n", stall_state.fifo5);
		sh_css_print("  status & control stalled: %d\n",
				stall_state.stat_ctrl);
		sh_css_print("  dmem stalled: %d\n", stall_state.dmem);
		sh_css_print("  vmem stalled: %d\n", stall_state.vmem);
		sh_css_print("  vamem1 stalled: %d\n", stall_state.vamem1);
		sh_css_print("  vamem2 stalled: %d\n", stall_state.vamem2);
	}
}

void
sh_css_dump_sp_state(void)
{
	struct sh_css_cell_state state;
	struct sh_css_sp_stall_state stall_state;
	sh_css_hrt_sp_get_state(&state, &stall_state);
	print_cell_state(&state, "SP");
	if (state.is_stalling) {
		sh_css_print("  if_prim_FIFO    stalled: %d\n",
				stall_state.fifo0);
		sh_css_print("  if_sec_FIFO     stalled: %d\n",
				stall_state.fifo1);
		sh_css_print("  str_to_mem_FIFO stalled: %d\n",
				stall_state.fifo2);
		sh_css_print("  dma_FIFO       stalled: %d\n",
				stall_state.fifo3);
		sh_css_print("  gdc_FIFO       stalled: %d\n",
				stall_state.fifo4);
		sh_css_print("  isp_FIFO       stalled: %d\n",
				stall_state.fifo5);
		sh_css_print("  gp_FIFO        stalled: %d\n",
				stall_state.fifo6);
		sh_css_print("  if_prim_b_FIFO stalled: %d\n",
				stall_state.fifo7);
		sh_css_print("  dmem stalled: %d\n", stall_state.dmem);
		sh_css_print("  control master stalled: %d\n",
				stall_state.control_master);
		sh_css_print("  i-cache master stalled: %d\n",
				stall_state.icache_master);
	}
}

static void
print_if_state(struct sh_css_if_state *state)
{
	unsigned int val;
	sh_css_print("InputFormatter State:\n");
	sh_css_print("  Configuration:\n");
	sh_css_print("    Software reset:             %s\n",
		     state->reset ? "Active" : "Not active");
	sh_css_print("    Start line:                 %d\n", state->start_line);
	sh_css_print("    Start column:               %d\n",
		     state->start_column);
	sh_css_print("    Cropped height:             %d\n",
		     state->cropped_height);
	sh_css_print("    Cropped width:              %d\n",
		     state->cropped_width);
	sh_css_print("    Vertical decimation:        %d\n",
		     state->ver_decimation);
	sh_css_print("    Horizontal decimation:      %d\n",
		     state->hor_decimation);
	sh_css_print("    Deinterleaving:             %d\n",
		     state->deinterleaving);
	sh_css_print("    Left padding:               %d\n",
		     state->left_padding);
	sh_css_print("    End-of-line offset (bytes): %d\n",
		     state->eol_offset);
	sh_css_print("    VMEM start address:         0x%06X\n",
		     state->vmem_start_address);
	sh_css_print("    VMEM end address:           0x%06X\n",
		     state->vmem_end_address);
	sh_css_print("    VMEM increment:             0x%06X\n",
		     state->vmem_increment);
	sh_css_print("    YUV 420 format:             %d\n", state->yuv420);
	sh_css_print("    Vsync:                      Active %s\n",
		     state->vsync_active_low ? "low" : "high");
	sh_css_print("    Hsync:                      Active %s\n",
		     state->hsync_active_low ? "low" : "high");
#if 0
	/* not supported on  css_dev */
	sh_css_print("    Sensor stalling:            %s\n",
		     state->allow_fifo_overflow ? "Allowed" : "Not allowed");
#endif
	sh_css_print("  FSM Status\n");
	val = state->fsm_sync_status;
	sh_css_print("    FSM Synchronization Status 0x%X: ", val);
	if (val > 7)
		sh_css_print("ERROR : ");

	switch (val & 0x7) {
	case 0:
		sh_css_print("idle\n");
		break;
	case 1:
		sh_css_print("request frame\n");
		break;
	case 2:
		sh_css_print("request lines\n");
		break;
	case 3:
		sh_css_print("request vectors\n");
		break;
	case 4:
		sh_css_print("send acknowledge\n");
		break;
	default:
		sh_css_print("unknown\n");
		break;
	}
	sh_css_print("    FSM Synchronization Counter               %d\n",
		     state->fsm_sync_counter);
	val = state->fsm_crop_status;
	sh_css_print("    FSM Crop Status                           0x%X: ",
		     val);
	if (val > 7)
		sh_css_print("ERROR : ");
	switch (val & 0x7) {
	case 0:
		sh_css_print("idle\n");
		break;
	case 1:
		sh_css_print("wait line\n");
		break;
	case 2:
		sh_css_print("crop line\n");
		break;
	case 3:
		sh_css_print("crop pixel\n");
		break;
	case 4:
		sh_css_print("pass pixel\n");
		break;
	case 5:
		sh_css_print("pass line\n");
		break;
	case 6:
		sh_css_print("lost line\n");
		break;
	default:
		sh_css_print("unknown\n");
		break;
	}

	sh_css_print("    FSM Crop Line Counter                     %d\n",
		     state->fsm_crop_line_counter);
	sh_css_print("    FSM Crop Pixel Counter                    %d\n",
		     state->fsm_crop_pixel_counter);
	sh_css_print("    FSM Deinterleaving idx buffer             %d\n",
		     state->fsm_deinterleaving_index);
	sh_css_print("    FSM Decimation H decimation counter       %d\n",
		     state->fsm_dec_h_counter);
	sh_css_print("    FSM Decimation V decimation counter       %d\n",
		     state->fsm_dec_v_counter);
	sh_css_print("    FSM Decimation block V decimation counter %d\n",
		     state->fsm_dec_block_v_counter);

	val = state->fsm_padding_status;
	sh_css_print("    FSM Padding Status 0x%X: ", val);
	if (val > 7)
		sh_css_print("ERROR : ");

	switch (val & 0x7) {
	case 0:
		sh_css_print("idle\n");
		break;
	case 1:
		sh_css_print("left pad\n");
		break;
	case 2:
		sh_css_print("write\n");
		break;
	case 3:
		sh_css_print("right pad\n");
		break;
	case 4:
		sh_css_print("send end of line\n");
		break;
	default:
		sh_css_print("unknown\n");
		break;
	}
	sh_css_print("FSM Padding element index counter             %d\n",
		     state->fsm_padding_elem_counter);
	sh_css_print("FSM Vector support error                      %d\n",
		     state->fsm_vector_support_error);
	sh_css_print("FSM Vector support buf full                   %d\n",
		     state->fsm_vector_buffer_full);
	sh_css_print("FSM Vector support                            %d\n",
		     state->vector_support);
	sh_css_print("Fifo sensor data lost                         %d\n",
		     state->sensor_data_lost);
}

void
sh_css_dump_if_state(void)
{
	struct sh_css_if_state state;
	sh_css_hrt_if_prim_a_get_state(&state);
	print_if_state(&state);
	sh_css_dump_pif_isp_fifo_state();
}

void
sh_css_dump_dma_state(void)
{
	struct sh_css_dma_state state;
	int i, ch_id, num_ports = 3, num_channels = 8;

	sh_css_hrt_dma_get_state(&state);
	sh_css_print("DMA dump status:\n\t");
	sh_css_print("FSM Command flag state:\n\t\t");
	if (state.fsm_command_idle)
		sh_css_print("IDLE\n\t\t");
	if (state.fsm_command_run)
		sh_css_print("RUN\n\t\t");
	if (state.fsm_command_stalling)
		sh_css_print("STALL\n\t\t");
	if (state.fsm_command_error)
		sh_css_print("ERROR\n\t\t");
	ch_id = state.last_command_channel;
	sh_css_print("last command received (0x%x) : ", state.last_command);
	if (state.last_command == sh_css_dma_command_read)
		sh_css_print(" Read  2D Block with settings from ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_write)
		sh_css_print(" Write 2D Block with settings from ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_set_channel)
		sh_css_print(" Set Channel:%d", ch_id);
	if (state.last_command == sh_css_dma_command_set_param)
		sh_css_print(" Set Param:%d on Channel:%d",
				state.last_command_param, ch_id);
	if (state.last_command == sh_css_dma_command_read_spec)
		sh_css_print(" Read Specific 2D Block on ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_write_spec)
		sh_css_print(" Write Specific 2D Block on ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_init)
		sh_css_print(" Init 2D Block on Device A on ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_init_spec)
		sh_css_print(" Init Specific 2D Block on ch:%d", ch_id);
	if (state.last_command == sh_css_dma_command_reset)
		sh_css_print(" DMA SW Reset");
	sh_css_print("\n\t");
	sh_css_print("DMA registers,  connection group 0\n\t");
	sh_css_print("Cmd Fifo Command    0x%x\n\t\t", state.current_command);
	sh_css_print("Cmd Fifo Address A  0x%x\n\t\t", state.current_addr_a);
	sh_css_print("Cmd Fifo Address B  0x%x\n\t\t", state.current_addr_b);
	sh_css_print("FSM Ctrl flag and state:\n\t\t\t");
	if (state.fsm_ctrl_idle)
		sh_css_print("IDLE -> ");
	if (state.fsm_ctrl_run)
		sh_css_print("RUN -> ");
	if (state.fsm_ctrl_stalling)
		sh_css_print("STALL -> ");
	if (state.fsm_ctrl_error)
		sh_css_print("ERROR -> ");
	if (state.fsm_ctrl_state == sh_css_dma_ctrl_state_idle)
		sh_css_print("Idle state\n\t\t");
	if (state.fsm_ctrl_state == sh_css_dma_ctrl_state_req_rcv)
		sh_css_print("Req Rcv state\n\t\t");
	if (state.fsm_ctrl_state == sh_css_dma_ctrl_state_rcv)
		sh_css_print("Rcv state\n\t\t");
	if (state.fsm_ctrl_state == sh_css_dma_ctrl_state_rcv_req)
		sh_css_print("Rcv Req state\n\t\t");
	if (state.fsm_ctrl_state == sh_css_dma_ctrl_state_init)
		sh_css_print("Init state\n\t\t");
	sh_css_print("FSM Ctrl source dev          : %d\n\t\t",
			state.fsm_ctrl_source_dev);
	sh_css_print("FSM Ctrl source addr         : 0x%x\n\t\t",
			state.fsm_ctrl_source_addr);
	sh_css_print("FSM Ctrl source stride       : 0x%x\n\t\t",
			state.fsm_ctrl_source_stride);
	sh_css_print("FSM Ctrl source width        : %d\n\t\t",
			state.fsm_ctrl_source_width);
	sh_css_print("FSM Ctrl source height       : %d\n\t\t",
			state.fsm_ctrl_source_height);
	sh_css_print("FSM Ctrl pack source dev     : %d\n\t\t",
			state.fsm_ctrl_pack_source_dev);
	sh_css_print("FSM Ctrl pack dest dev       : %d\n\t\t",
			state.fsm_ctrl_pack_dest_dev);
	sh_css_print("FSM Ctrl dest addr           : 0x%x\n\t\t",
			state.fsm_ctrl_dest_addr);
	sh_css_print("FSM Ctrl dest stride         : 0x%x\n\t\t",
			state.fsm_ctrl_dest_stride);
	sh_css_print("FSM Ctrl pack source width   : %d\n\t\t",
			state.fsm_ctrl_pack_source_width);
	sh_css_print("FSM Ctrl pack dest height    : %d\n\t\t",
			state.fsm_ctrl_pack_dest_height);
	sh_css_print("FSM Ctrl pack dest width     : %d\n\t\t",
			state.fsm_ctrl_pack_dest_width);
	sh_css_print("FSM Ctrl pack source elems   : %d\n\t\t",
			state.fsm_ctrl_pack_source_elems);
	sh_css_print("FSM Ctrl pack dest elems     : %d\n\t\t",
			state.fsm_ctrl_pack_dest_elems);
	sh_css_print("FSM Ctrl pack extension      : %d\n\t\t",
			state.fsm_ctrl_pack_extension);
	sh_css_print("FSM Pack flag state\t\t\t");
	if (state.pack_idle)
		sh_css_print("IDLE\t\t");
	if (state.pack_run)
		sh_css_print("RUN\t\t");
	if (state.pack_stalling)
		sh_css_print("STALL\t\t");
	if (state.pack_error)
		sh_css_print("ERROR\t\t");
	sh_css_print("FSM Pack cnt height          : %d\n\t\t",
			state.pack_cnt_height);
	sh_css_print("FSM Pack src cnt width       : %d\n\t\t",
			state.pack_src_cnt_width);
	sh_css_print("FSM Pack dest cnt width      : %d\n\t\t",
			state.pack_dest_cnt_width);
	sh_css_print("FSM Read state\n\t\t");
	if (state.read_state == sh_css_dma_rw_state_idle)
		sh_css_print("\tIdle state\n\t\t");
	if (state.read_state == sh_css_dma_rw_state_req)
		sh_css_print("\tReq state\n\t\t");
	if (state.read_state == sh_css_dma_rw_state_next_line)
		sh_css_print("\tNext line\n\t\t");
	if (state.read_state == sh_css_dma_rw_state_unlock_channel)
		sh_css_print("\tUnlock channel\n\t\t");
	sh_css_print("FSM Read cnt height          : %d\n\t\t",
			state.read_cnt_height);
	sh_css_print("FSM Read cnt width           : %d\n\t\t",
			state.read_cnt_width);
	sh_css_print("FSM Write state\n\t\t\t");
	if (state.write_state == sh_css_dma_rw_state_idle)
		sh_css_print("\tIdle state\n\t\t");
	if (state.write_state == sh_css_dma_rw_state_req)
		sh_css_print("\tReq state\n\t\t");
	if (state.write_state == sh_css_dma_rw_state_next_line)
		sh_css_print("\tNext line\n\t\t");
	if (state.write_state == sh_css_dma_rw_state_unlock_channel)
		sh_css_print("\tUnlock channel\n\t\t");
	sh_css_print("FSM Write height             : %d\n\t\t",
			state.write_height);
	sh_css_print("FSM Write width              : %d\n\t\t",
			state.write_width);
	sh_css_print("\n\t");
	for (i = 0; i < num_ports; i++) {
		sh_css_print("DMA device interface %d\n", i);
		sh_css_print("\t\tDMA internal side state\n\t\t\t");
		sh_css_print("CS: %d - We_n: %d - Run: %d - Ack: %d\n",
				state.port_states[i].req_cs,
				state.port_states[i].req_we_n,
				state.port_states[i].req_run,
				state.port_states[i].req_ack);
		sh_css_print("\t\tMaster Output side state\n\t\t\t");
		sh_css_print("CS: %d - We_n: %d - Run: s%d - Ack: %d\n",
				state.port_states[i].send_cs,
				state.port_states[i].send_we_n,
				state.port_states[i].send_run,
				state.port_states[i].send_ack);
		sh_css_print("\t\tFifo state\n\t\t\t");
		if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_will_be_full) {
			sh_css_print("FiFo will be full\n");
		}
		if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_full) {
			sh_css_print("Fifo Full\n");
		}
		if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_empty) {
			sh_css_print("Fifo Empty\n");
		}
		sh_css_print("\t\tFifo counter %d\n\t",
				state.port_states[i].fifo_counter);
	}
	sh_css_print("\n\t");
	for (i = 0; i < num_channels; i++) {
		struct sh_css_dma_channel_state *ch;
		ch = &(state.channel_states[i]);
		sh_css_print("DMA channel register %d\n\t\t", i);
		sh_css_print("Connection      : %d\n\t\t", ch->connection);
		sh_css_print("Sign extend:    : %d\n\t\t", ch->sign_extend);
		sh_css_print("Reverse elems  : %d\n\t\t",
				ch->reverse_elem_order);
		sh_css_print("Stride Dev A    : 0x%x\n\t\t", ch->stride_a);
		sh_css_print("Elems Dev A     : %d\n\t\t", ch->elems_a);
		sh_css_print("Cropping Dev A  : %d\n\t\t", ch->cropping_a);
		sh_css_print("Width Dev A     : %d\n\t\t", ch->width_a);
		sh_css_print("Stride Dev B    : 0x%x\n\t\t", ch->stride_b);
		sh_css_print("Elems Dev B     : %d\n\t\t", ch->elems_b);
		sh_css_print("Cropping Dev B  : %d\n\t\t", ch->cropping_b);
		sh_css_print("Width Dev B     : %d\n\t\t", ch->width_b);
		sh_css_print("Height          : %d\n\t", ch->height);
	}
	sh_css_print("\n");
}

static void
print_fifo_channel_state(struct sh_css_fifo_channel_state *state,
		const char *descr)
{
	sh_css_print("FIFO channel: %s\n", descr);
	sh_css_print("  source valid: %d\n", state->src_valid);
	sh_css_print("  fifo accept:  %d\n", state->fifo_accept);
	sh_css_print("  fifo valid:   %d\n", state->fifo_valid);
	sh_css_print("  sink accept:  %d\n", state->sink_accept);
}

void
sh_css_dump_pif_isp_fifo_state(void)
{
	struct sh_css_fifo_channel_state pif_to_isp,
					 isp_to_pif;
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_isp_to_if_prim_a,
					  &isp_to_pif);
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_if_prim_a_to_isp,
					  &pif_to_isp);
	print_fifo_channel_state(&pif_to_isp, "Primary IF A to ISP");
	print_fifo_channel_state(&isp_to_pif, "ISP to Primary IF A)");
}

void
sh_css_dump_dma_sp_fifo_state(void)
{
	struct sh_css_fifo_channel_state dma_to_sp,
					 sp_to_dma;
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_dma_to_sp,
					  &dma_to_sp);
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_sp_to_dma,
					  &sp_to_dma);
	print_fifo_channel_state(&dma_to_sp, "DMA to SP");
	print_fifo_channel_state(&sp_to_dma, "SP to DMA");
}

void
sh_css_dump_dma_isp_fifo_state(void)
{
	struct sh_css_fifo_channel_state dma_to_isp,
					 isp_to_dma;
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_dma_to_isp,
					  &dma_to_isp);
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_isp_to_dma,
					  &isp_to_dma);
	print_fifo_channel_state(&dma_to_isp, "DMA to ISP");
	print_fifo_channel_state(&isp_to_dma, "ISP to DMA");
}

void
sh_css_dump_isp_sp_fifo_state(void)
{
	struct sh_css_fifo_channel_state sp_to_isp,
					 isp_to_sp;
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_sp_to_isp,
					  &sp_to_isp);
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_isp_to_sp,
					  &isp_to_sp);
	print_fifo_channel_state(&sp_to_isp, "SP to ISP");
	print_fifo_channel_state(&isp_to_sp, "ISP to SP");
}

void
sh_css_dump_debug_info(void)
{
	sh_css_dump_rx_state();
	sh_css_dump_if_state();
	/* some of these are commented out because they usually don't offer
	   anything meaningful. */
	if (0) {
		sh_css_dump_isp_state();
		sh_css_dump_isp_sp_fifo_state();
	}
	sh_css_dump_sp_state();
	if (0) {
		sh_css_dump_dma_isp_fifo_state();
		sh_css_dump_dma_sp_fifo_state();
		sh_css_dump_dma_state();
	}
}

static void
sh_css_binary_info_print(const struct sh_css_binary_info *info)
{
	sh_css_print("id = %d\n", info->id);
	sh_css_print("mode = %d\n", info->mode);
	sh_css_print("max_input_width = %d\n", info->max_input_width);
	sh_css_print("min_output_width = %d\n", info->min_output_width);
	sh_css_print("max_output_width = %d\n", info->max_output_width);
	sh_css_print("top_cropping = %d\n", info->top_cropping);
	sh_css_print("left_cropping = %d\n", info->left_cropping);
	sh_css_print("xmem_addr = %p\n", info->xmem_addr);
	sh_css_print("enable_vf_veceven = %d\n", info->enable_vf_veceven);
	sh_css_print("enable_dis = %d\n", info->enable_dis);
	sh_css_print("enable_uds = %d\n", info->enable_uds);
	sh_css_print("enable ds = %d\n", info->enable_ds);
	sh_css_print("s3atbl_use_dmem = %d\n", info->s3atbl_use_dmem);
}

void
sh_css_binary_print(const struct sh_css_binary *bi)
{
	sh_css_binary_info_print(bi->info);
	sh_css_print("input:  %dx%d, format = %d, padded width = %d\n",
		     bi->in_frame_info.width, bi->in_frame_info.height,
		     bi->in_frame_info.format, bi->in_frame_info.padded_width);
	sh_css_print("internal :%dx%d, format = %d, padded width = %d\n",
		     bi->internal_frame_info.width,
		     bi->internal_frame_info.height,
		     bi->internal_frame_info.format,
		     bi->internal_frame_info.padded_width);
	sh_css_print("out:    %dx%d, format = %d, padded width = %d\n",
		     bi->out_frame_info.width, bi->out_frame_info.height,
		     bi->out_frame_info.format,
		     bi->out_frame_info.padded_width);
	sh_css_print("vf out: %dx%d, format = %d, padded width = %d\n",
		     bi->vf_frame_info.width, bi->vf_frame_info.height,
		     bi->vf_frame_info.format, bi->vf_frame_info.padded_width);
	sh_css_print("online = %d\n", bi->online);
	sh_css_print("input_buf_vectors = %d\n", bi->input_buf_vectors);
	sh_css_print("deci_factor_log2 = %d\n", bi->deci_factor_log2);
	sh_css_print("vf_downscale_log2 = %d\n", bi->vf_downscale_log2);
	sh_css_print("dis_deci_factor_log2 = %d\n", bi->dis_deci_factor_log2);
	sh_css_print("dis hor coef num = %d\n", bi->dis_hor_coef_num_isp);
	sh_css_print("dis ver coef num = %d\n", bi->dis_ver_coef_num_isp);
	sh_css_print("dis hor proj num = %d\n", bi->dis_ver_proj_num_isp);
	sh_css_print("sctbl_width_per_color = %d\n", bi->sctbl_width_per_color);
	sh_css_print("s3atbl_width = %d\n", bi->s3atbl_width);
	sh_css_print("s3atbl_height = %d\n", bi->s3atbl_height);
}

void
sh_css_frame_print(const struct sh_css_frame *frame, const char *descr)
{
	sh_css_print("frame %s (%p):\n", descr, frame);
	sh_css_print("  resolution    = %dx%d\n",
		     frame->info.width, frame->info.height);
	sh_css_print("  padded width  = %d\n", frame->info.padded_width);
	sh_css_print("  format        = %d\n", frame->info.format);
	sh_css_print("  is contiguous = %s\n",
		     frame->contiguous ? "yes" : "no");
	switch (frame->info.format) {
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV21:
	case SH_CSS_FRAME_FORMAT_NV61:
		sh_css_print("  Y = %p\n", frame->planes.nv.y.data);
		sh_css_print("  UV = %p\n", frame->planes.nv.uv.data);
		break;
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_UYVY:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
		sh_css_print("  YUYV = %p\n", frame->planes.yuyv.data);
		break;
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV422:
	case SH_CSS_FRAME_FORMAT_YUV444:
	case SH_CSS_FRAME_FORMAT_YV12:
	case SH_CSS_FRAME_FORMAT_YV16:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_YUV422_16:
		sh_css_print("  Y = %p\n", frame->planes.yuv.y.data);
		sh_css_print("  U = %p\n", frame->planes.yuv.u.data);
		sh_css_print("  V = %p\n", frame->planes.yuv.v.data);
		break;
	case SH_CSS_FRAME_FORMAT_RAW:
		sh_css_print("  RAW = %p\n", frame->planes.raw.data);
		break;
	case SH_CSS_FRAME_FORMAT_RGBA888:
	case SH_CSS_FRAME_FORMAT_RGB565:
		sh_css_print("  RGB = %p\n", frame->planes.rgb.data);
		break;
	case SH_CSS_FRAME_FORMAT_QPLANE6:
		sh_css_print("  R    = %p\n", frame->planes.plane6.r.data);
		sh_css_print("  RatB = %p\n", frame->planes.plane6.r_at_b.data);
		sh_css_print("  Gr   = %p\n", frame->planes.plane6.gr.data);
		sh_css_print("  Gb   = %p\n", frame->planes.plane6.gb.data);
		sh_css_print("  B    = %p\n", frame->planes.plane6.b.data);
		sh_css_print("  BatR = %p\n", frame->planes.plane6.b_at_r.data);
		break;
	case SH_CSS_FRAME_FORMAT_BINARY_8:
		sh_css_print("  Binary data = %p\n",
				frame->planes.binary.data.data);
		break;
	default:
		sh_css_print("  unknown frame type\n");
		break;
	}
}

void
sh_css_print_sp_debug_state(const struct sh_css_sp_debug_state *state)
{
	int i;

	sh_css_print("sp_error = 0x%x\n", state->error);
	for (i = 0; i < 16; i++)
		sh_css_print("sp_debug[%d] = %d\n", i, state->debug[i]);
}

void
sh_css_dump_rx_state(void)
{
	unsigned int infos = 0, bits;
	bits = sh_css_rx_get_interrupt_reg();
	sh_css_rx_get_interrupt_info(&infos);

	sh_css_print("CSI Receiver errors: (irq reg = 0x%X\n", bits);
	if (infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		sh_css_print("  buffer overrun");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT)
		sh_css_print("  start-of-transmission error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		sh_css_print("  start-of-transmission sync error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CONTROL)
		sh_css_print("  control error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		sh_css_print("  2 or more ECC errors");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CRC)
		sh_css_print("  CRC mismatch");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		sh_css_print("  unknown error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		sh_css_print("  frame sync error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		sh_css_print("  frame data error");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		sh_css_print("  data timeout");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		sh_css_print("  unknown escape command entry");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		sh_css_print("  line sync error");
}
