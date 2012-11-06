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

/* Global variable to store the dtrace verbosity level */
unsigned int sh_css_trace_level;

void sh_css_set_dtrace_level(unsigned int trace_level)
{
	sh_css_trace_level = trace_level;
}

static void
print_cell_state(struct sh_css_cell_state *state, const char *cell)
{
	sh_css_dtrace(2, "%s state:\n", cell);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "PC", state->pc);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "Status register",
			state->status_register);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is broken", state->is_broken);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is idle", state->is_idle);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is sleeping", state->is_sleeping);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is stalling", state->is_stalling);
}

void
sh_css_dump_isp_state(void)
{
	struct sh_css_cell_state      state;
	struct sh_css_isp_stall_state stall_state;

	sh_css_hrt_isp_get_state(&state, &stall_state);

	print_cell_state(&state, "ISP");

	if (state.is_stalling) {
		sh_css_dtrace(2, "\t%-32s: %d\n", "[0] if_prim_A_FIFO stalled",
				stall_state.fifo0);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[1] if_prim_b_FIFO stalled",
				stall_state.fifo1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[2] dma_FIFO stalled",
				stall_state.fifo2);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[3] gdc_FIFO stalled",
				stall_state.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[4] gpio_FIFO stalled",
				stall_state.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[5] sp_FIFO stalled",
				stall_state.fifo5);
		sh_css_dtrace(2, "\t%-32s: %d\n", "status & control stalled",
				stall_state.stat_ctrl);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dmem stalled",
				stall_state.dmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vmem stalled",
				stall_state.vmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vamem1 stalled",
				stall_state.vamem1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vamem2 stalled",
				stall_state.vamem2);
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
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_FIFO stalled",
				stall_state.fifo0);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_sec_FIFO stalled",
				stall_state.fifo1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "str_to_mem_FIFO stalled",
				stall_state.fifo2);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dma_FIFO stalled",
				stall_state.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gdc_FIFO stalled",
				stall_state.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "isp_FIFO stalled",
				stall_state.fifo5);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gp_FIFO stalled",
				stall_state.fifo6);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_b_FIFO stalled",
				stall_state.fifo7);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dmem stalled",
				stall_state.dmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "control master stalled",
				stall_state.control_master);
		sh_css_dtrace(2, "\t%-32s: %d\n", "i-cache master stalled",
				stall_state.icache_master);
	}
}

static void
print_if_state(struct sh_css_if_state *state)
{
	unsigned int val;

	const char *st_reset  = (state->reset ? "Active" : "Not active");
	const char *st_vsalow = (state->vsync_active_low ? "low" : "high");
	const char *st_hsalow = (state->hsync_active_low ? "low" : "high");

	const char *fsm_sync_status_str    = "unknown";
	const char *fsm_crop_status_str    = "unknown";
	const char *fsm_padding_status_str = "unknown";

	int         st_stline = state->start_line;
	int         st_stcol  = state->start_column;
	int         st_crpht  = state->cropped_height;
	int         st_crpwd  = state->cropped_width;
	int         st_verdcm = state->ver_decimation;
	int         st_hordcm = state->hor_decimation;
	int         st_deintr = state->deinterleaving;
	int         st_leftpd = state->left_padding;
	int         st_eoloff = state->eol_offset;
	int         st_vmstad = state->vmem_start_address;
	int         st_vmenad = state->vmem_end_address;
	int         st_vmincr = state->vmem_increment;
	int         st_yuv420 = state->yuv420;

	sh_css_dtrace(2, "InputFormatter State:\n");

	sh_css_dtrace(2, "\tConfiguration:\n");

	sh_css_dtrace(2, "\t\t%-32s: %s\n"       ,
			"Software reset"         , st_reset);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Start line"             , st_stline);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Start column"           , st_stcol);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Cropped height"         , st_crpht);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Cropped width"          , st_crpwd);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Ver decimation"         , st_verdcm);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Hor decimation"         , st_hordcm);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Deinterleaving"         , st_deintr);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Left padding"           , st_leftpd);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"EOL offset (bytes)"     , st_eoloff);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM start address"     , st_vmstad);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM end address"       , st_vmenad);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM increment"         , st_vmincr);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"YUV 420 format"         , st_yuv420);
	sh_css_dtrace(2, "\t\t%-32s: Active %s\n",
			"Vsync"                  , st_vsalow);
	sh_css_dtrace(2, "\t\t%-32s: Active %s\n",
			"Hsync"                  , st_hsalow);

	sh_css_dtrace(2, "\tFSM Status:\n");

	val = state->fsm_sync_status;

	if (val > 7)
		fsm_sync_status_str = "ERROR";

	switch (val & 0x7) {
	case 0:
		fsm_sync_status_str = "idle";
		break;
	case 1:
		fsm_sync_status_str = "request frame";
		break;
	case 2:
		fsm_sync_status_str = "request lines";
		break;
	case 3:
		fsm_sync_status_str = "request vectors";
		break;
	case 4:
		fsm_sync_status_str = "send acknowledge";
		break;
	default:
		fsm_sync_status_str = "unknown";
		break;
	}

	sh_css_dtrace(2, "\t\t%-32s: (0x%X: %s)\n",
		     "FSM Synchronization Status", val, fsm_sync_status_str);

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM Synchronization Counter", state->fsm_sync_counter);

	val = state->fsm_crop_status;

	if (val > 7)
		fsm_crop_status_str = "ERROR";

	switch (val & 0x7) {
	case 0:
		fsm_crop_status_str = "idle";
		break;
	case 1:
		fsm_crop_status_str = "wait line";
		break;
	case 2:
		fsm_crop_status_str = "crop line";
		break;
	case 3:
		fsm_crop_status_str = "crop pixel";
		break;
	case 4:
		fsm_crop_status_str = "pass pixel";
		break;
	case 5:
		fsm_crop_status_str = "pass line";
		break;
	case 6:
		fsm_crop_status_str = "lost line";
		break;
	default:
		fsm_crop_status_str = "unknown";
		break;
	}
	sh_css_dtrace(2, "\t\t%-32s: (0x%X: %s)\n",
		     "FSM Crop Status", val, fsm_crop_status_str);

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM Crop Line Counter",
		     state->fsm_crop_line_counter);
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM Crop Pixel Counter",
		     state->fsm_crop_pixel_counter);
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM Deinterleaving idx buffer",
		     state->fsm_deinterleaving_index);
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM H decimation counter",
		     state->fsm_dec_h_counter);
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM V decimation counter",
		     state->fsm_dec_v_counter);
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		     "FSM block V decimation counter",
		     state->fsm_dec_block_v_counter);

	val = state->fsm_padding_status;

	if (val > 7)
		fsm_padding_status_str = "ERROR";

	switch (val & 0x7) {
	case 0:
		fsm_padding_status_str = "idle";
		break;
	case 1:
		fsm_padding_status_str = "left pad";
		break;
	case 2:
		fsm_padding_status_str = "write";
		break;
	case 3:
		fsm_padding_status_str = "right pad";
		break;
	case 4:
		fsm_padding_status_str = "send end of line";
		break;
	default:
		fsm_padding_status_str = "unknown";
		break;
	}

	sh_css_dtrace(2, "\t\t%-32s: (0x%X: %s)\n", "FSM Padding Status",
		     val, fsm_padding_status_str);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Padding element idx counter",
		     state->fsm_padding_elem_counter);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Vector support error",
		     state->fsm_vector_support_error);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Vector support buf full",
		     state->fsm_vector_buffer_full);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Vector support",
		     state->vector_support);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "Fifo sensor data lost",
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

	const char *fsm_cmd_st_lbl = "FSM Command flag state";
	const char *fsm_ctl_st_lbl = "FSM Control flag state";
	const char *fsm_ctl_state  = NULL;
	const char *fsm_ctl_flag   = NULL;
	const char *fsm_pack_st    = NULL;
	const char *fsm_read_st    = NULL;
	const char *fsm_write_st   = NULL;
	char last_cmd_str[64];

	sh_css_hrt_dma_get_state(&state);
	/* Print header for DMA dump status */
	sh_css_dtrace(2, "DMA dump status:\n");

	/* Print FSM command flag state */
	if (state.fsm_command_idle)
		sh_css_dtrace(2, "\t%-32s: %s\n", fsm_cmd_st_lbl, "IDLE");
	if (state.fsm_command_run)
		sh_css_dtrace(2, "\t%-32s: %s\n", fsm_cmd_st_lbl, "RUN");
	if (state.fsm_command_stalling)
		sh_css_dtrace(2, "\t%-32s: %s\n", fsm_cmd_st_lbl, "STALL");
	if (state.fsm_command_error)
		sh_css_dtrace(2, "\t%-32s: %s\n", fsm_cmd_st_lbl, "ERROR");

	/* Print last command along with the channel */
	ch_id = state.last_command_channel;

	switch (state.last_command) {
	case sh_css_dma_command_read:
		snprintf(last_cmd_str, 64,
		  "Read 2D Block [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_write:
		snprintf(last_cmd_str, 64,
		  "Write 2D Block [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_set_channel:
		snprintf(last_cmd_str, 64,
		  "Set Channel [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_set_param:
		snprintf(last_cmd_str, 64,
		  "Set Param: %d [Channel: %d]",
		  state.last_command_param, ch_id);
		break;
	case sh_css_dma_command_read_spec:
		snprintf(last_cmd_str, 64,
		  "Read Specific 2D Block [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_write_spec:
		snprintf(last_cmd_str, 64,
		  "Write Specific 2D Block [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_init:
		snprintf(last_cmd_str, 64,
		  "Init 2D Block on Device A [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_init_spec:
		snprintf(last_cmd_str, 64,
		  "Init Specific 2D Block [Channel: %d]", ch_id);
		break;
	case sh_css_dma_command_reset:
		snprintf(last_cmd_str, 64,
		  "DMA SW Reset");
		break;
	}
	sh_css_dtrace(2, "\t%-32s: (0x%X : %s)\n", "last command received",
		     state.last_command, last_cmd_str);

	/* Print DMA registers */
	sh_css_dtrace(2, "\t%-32s\n", "DMA registers, connection group 0");
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "Cmd Fifo Command",
			state.current_command);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "Cmd Fifo Address A",
			state.current_addr_a);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "Cmd Fifo Address B",
			state.current_addr_b);

	if (state.fsm_ctrl_idle)
		fsm_ctl_flag = "IDLE";
	else if (state.fsm_ctrl_run)
		fsm_ctl_flag = "RUN";
	else if (state.fsm_ctrl_stalling)
		fsm_ctl_flag = "STAL";
	else if (state.fsm_ctrl_error)
		fsm_ctl_flag = "ERROR";
	else
		fsm_ctl_flag = "UNKNOWN";

	switch (state.fsm_ctrl_state) {
	case sh_css_dma_ctrl_state_idle:
		fsm_ctl_state = "Idle state";
		break;
	case sh_css_dma_ctrl_state_req_rcv:
		fsm_ctl_state = "Req Rcv state";
		break;
	case sh_css_dma_ctrl_state_rcv:
		fsm_ctl_state = "Rcv state";
		break;
	case sh_css_dma_ctrl_state_rcv_req:
		fsm_ctl_state = "Rcv Req state";
		break;
	case sh_css_dma_ctrl_state_init:
		fsm_ctl_state = "Init state";
		break;
	}

	sh_css_dtrace(2, "\t\t%-32s: %s -> %s\n", fsm_ctl_st_lbl,
		     fsm_ctl_flag, fsm_ctl_state);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl source dev",
			state.fsm_ctrl_source_dev);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "FSM Ctrl source addr",
			state.fsm_ctrl_source_addr);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "FSM Ctrl source stride",
			state.fsm_ctrl_source_stride);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl source width",
			state.fsm_ctrl_source_width);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl source height",
			state.fsm_ctrl_source_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack source dev",
			state.fsm_ctrl_pack_source_dev);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack dest dev",
			state.fsm_ctrl_pack_dest_dev);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "FSM Ctrl dest addr",
			state.fsm_ctrl_dest_addr);
	sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "FSM Ctrl dest stride",
			state.fsm_ctrl_dest_stride);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack source width",
			state.fsm_ctrl_pack_source_width);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack dest height",
			state.fsm_ctrl_pack_dest_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack dest width",
			state.fsm_ctrl_pack_dest_width);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack source elems",
			state.fsm_ctrl_pack_source_elems);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack dest elems",
			state.fsm_ctrl_pack_dest_elems);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Ctrl pack extension",
			state.fsm_ctrl_pack_extension);

	if (state.pack_idle)
		fsm_pack_st = "IDLE";
	if (state.pack_run)
		fsm_pack_st = "RUN";
	if (state.pack_stalling)
		fsm_pack_st = "STALL";
	if (state.pack_error)
		fsm_pack_st = "ERROR";

	sh_css_dtrace(2, "\t\t%-32s: %s\n", "FSM Pack flag state", fsm_pack_st);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Pack cnt height",
			state.pack_cnt_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Pack src cnt width",
			state.pack_src_cnt_width);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Pack dest cnt width",
			state.pack_dest_cnt_width);

	if (state.read_state == sh_css_dma_rw_state_idle)
		fsm_read_st = "Idle state";
	if (state.read_state == sh_css_dma_rw_state_req)
		fsm_read_st = "Req state";
	if (state.read_state == sh_css_dma_rw_state_next_line)
		fsm_read_st = "Next line";
	if (state.read_state == sh_css_dma_rw_state_unlock_channel)
		fsm_read_st = "Unlock channel";

	sh_css_dtrace(2, "\t\t%-32s: %s\n", "FSM Read state", fsm_read_st);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Read cnt height",
			state.read_cnt_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Read cnt width",
			state.read_cnt_width);

	if (state.write_state == sh_css_dma_rw_state_idle)
		fsm_write_st = "Idle state";
	if (state.write_state == sh_css_dma_rw_state_req)
		fsm_write_st = "Req state";
	if (state.write_state == sh_css_dma_rw_state_next_line)
		fsm_write_st = "Next line";
	if (state.write_state == sh_css_dma_rw_state_unlock_channel)
		fsm_write_st = "Unlock channel";

	sh_css_dtrace(2, "\t\t%-32s: %s\n", "FSM Write state", fsm_write_st);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Write height",
			state.write_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Write width",
			state.write_width);

	for (i = 0; i < num_ports; i++) {
		sh_css_dtrace(2, "\tDMA device interface %d\n", i);
		sh_css_dtrace(2, "\t\tDMA internal side state\n");
		sh_css_dtrace(2, "\t\t\tCS:%d - We_n:%d - Run:%d - Ack:%d\n",
				state.port_states[i].req_cs,
				state.port_states[i].req_we_n,
				state.port_states[i].req_run,
				state.port_states[i].req_ack);
		sh_css_dtrace(2, "\t\tMaster Output side state\n");
		sh_css_dtrace(2, "\t\t\tCS:%d - We_n:%d - Run:%d - Ack:%d\n",
				state.port_states[i].send_cs,
				state.port_states[i].send_we_n,
				state.port_states[i].send_run,
				state.port_states[i].send_ack);
		sh_css_dtrace(2, "\t\tFifo state\n");
		if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_will_be_full) {
			sh_css_dtrace(2, "\t\t\tFiFo will be full\n");
		} else if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_full) {
			sh_css_dtrace(2, "\t\t\tFifo Full\n");
		} else if (state.port_states[i].fifo_state ==
				sh_css_dma_fifo_state_empty) {
			sh_css_dtrace(2, "\t\t\tFifo Empty\n");
		} else {
			sh_css_dtrace(2, "\t\t\tFifo state unknown\n");
		}
		sh_css_dtrace(2, "\t\tFifo counter %d\n\n",
				state.port_states[i].fifo_counter);
	}

	for (i = 0; i < num_channels; i++) {
		struct sh_css_dma_channel_state *ch;
		ch = &(state.channel_states[i]);
		sh_css_dtrace(2, "\t%-32s: %d\n", "DMA channel register",
				i);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Connection",
				ch->connection);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Sign extend",
				ch->sign_extend);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Reverse elems",
				ch->reverse_elem_order);
		sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "Stride Dev A",
				ch->stride_a);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Elems Dev A",
				ch->elems_a);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Cropping Dev A",
				ch->cropping_a);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Width Dev A",
				ch->width_a);
		sh_css_dtrace(2, "\t\t%-32s: 0x%X\n", "Stride Dev B",
				ch->stride_b);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Elems Dev B",
				ch->elems_b);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Cropping Dev B",
				ch->cropping_b);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Width Dev B",
				ch->width_b);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Height",
				ch->height);
	}
	sh_css_dtrace(2, "\n");
}

static void
print_fifo_channel_state(struct sh_css_fifo_channel_state *state,
		const char *descr)
{
	sh_css_dtrace(2, "FIFO channel: %s\n", descr);
	sh_css_dtrace(2, "\t%-32s: %d\n", "source valid", state->src_valid);
	sh_css_dtrace(2, "\t%-32s: %d\n", "fifo accept" , state->fifo_accept);
	sh_css_dtrace(2, "\t%-32s: %d\n", "fifo valid"  , state->fifo_valid);
	sh_css_dtrace(2, "\t%-32s: %d\n", "sink accept" , state->sink_accept);
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
sh_css_dump_isp_gdc_fifo_state(void)
{
	struct sh_css_fifo_channel_state gdc_to_isp,
					 isp_to_gdc;
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_gdc_to_isp,
					  &gdc_to_isp);
	sh_css_hrt_fifo_channel_get_state(sh_css_hrt_fifo_isp_to_gdc,
					  &isp_to_gdc);
	print_fifo_channel_state(&gdc_to_isp, "GDC to ISP");
	print_fifo_channel_state(&isp_to_gdc, "ISP to GDC");
}

static void
sh_css_binary_info_print(const struct sh_css_binary_info *info)
{
	sh_css_dtrace(2, "id = %d\n", info->id);
	sh_css_dtrace(2, "mode = %d\n", info->mode);
	sh_css_dtrace(2, "max_input_width = %d\n", info->max_input_width);
	sh_css_dtrace(2, "min_output_width = %d\n", info->min_output_width);
	sh_css_dtrace(2, "max_output_width = %d\n", info->max_output_width);
	sh_css_dtrace(2, "top_cropping = %d\n", info->top_cropping);
	sh_css_dtrace(2, "left_cropping = %d\n", info->left_cropping);
	sh_css_dtrace(2, "xmem_addr = %p\n", info->xmem_addr);
	sh_css_dtrace(2, "enable_vf_veceven = %d\n", info->enable_vf_veceven);
	sh_css_dtrace(2, "enable_dis = %d\n", info->enable_dis);
	sh_css_dtrace(2, "enable_uds = %d\n", info->enable_uds);
	sh_css_dtrace(2, "enable ds = %d\n", info->enable_ds);
	sh_css_dtrace(2, "s3atbl_use_dmem = %d\n", info->s3atbl_use_dmem);
}

void
sh_css_binary_print(const struct sh_css_binary *bi)
{
	sh_css_binary_info_print(bi->info);
	sh_css_dtrace(2, "input:  %dx%d, format = %d, padded width = %d\n",
		     bi->in_frame_info.width, bi->in_frame_info.height,
		     bi->in_frame_info.format, bi->in_frame_info.padded_width);
	sh_css_dtrace(2, "internal :%dx%d, format = %d, padded width = %d\n",
		     bi->internal_frame_info.width,
		     bi->internal_frame_info.height,
		     bi->internal_frame_info.format,
		     bi->internal_frame_info.padded_width);
	sh_css_dtrace(2, "out:    %dx%d, format = %d, padded width = %d\n",
		     bi->out_frame_info.width, bi->out_frame_info.height,
		     bi->out_frame_info.format,
		     bi->out_frame_info.padded_width);
	sh_css_dtrace(2, "vf out: %dx%d, format = %d, padded width = %d\n",
		     bi->vf_frame_info.width, bi->vf_frame_info.height,
		     bi->vf_frame_info.format, bi->vf_frame_info.padded_width);
	sh_css_dtrace(2, "online = %d\n", bi->online);
	sh_css_dtrace(2, "input_buf_vectors = %d\n", bi->input_buf_vectors);
	sh_css_dtrace(2, "deci_factor_log2 = %d\n", bi->deci_factor_log2);
	sh_css_dtrace(2, "vf_downscale_log2 = %d\n", bi->vf_downscale_log2);
	sh_css_dtrace(2, "dis_deci_factor_log2 = %d\n",
			bi->dis_deci_factor_log2);
	sh_css_dtrace(2, "dis hor coef num = %d\n", bi->dis_hor_coef_num_isp);
	sh_css_dtrace(2, "dis ver coef num = %d\n", bi->dis_ver_coef_num_isp);
	sh_css_dtrace(2, "dis hor proj num = %d\n", bi->dis_ver_proj_num_isp);
	sh_css_dtrace(2, "sctbl_width_per_color = %d\n",
			bi->sctbl_width_per_color);
	sh_css_dtrace(2, "s3atbl_width = %d\n", bi->s3atbl_width);
	sh_css_dtrace(2, "s3atbl_height = %d\n", bi->s3atbl_height);
}

void
sh_css_frame_print(const struct sh_css_frame *frame, const char *descr)
{
	sh_css_dtrace(2, "frame %s (%p):\n", descr, frame);
	sh_css_dtrace(2, "  resolution    = %dx%d\n",
		     frame->info.width, frame->info.height);
	sh_css_dtrace(2, "  padded width  = %d\n", frame->info.padded_width);
	sh_css_dtrace(2, "  format        = %d\n", frame->info.format);
	sh_css_dtrace(2, "  is contiguous = %s\n",
		     frame->contiguous ? "yes" : "no");
	switch (frame->info.format) {
	case SH_CSS_FRAME_FORMAT_NV12:
	case SH_CSS_FRAME_FORMAT_NV16:
	case SH_CSS_FRAME_FORMAT_NV21:
	case SH_CSS_FRAME_FORMAT_NV61:
		sh_css_dtrace(2, "  Y = %p\n", frame->planes.nv.y.data);
		sh_css_dtrace(2, "  UV = %p\n", frame->planes.nv.uv.data);
		break;
	case SH_CSS_FRAME_FORMAT_YUYV:
	case SH_CSS_FRAME_FORMAT_UYVY:
	case SH_CSS_FRAME_FORMAT_YUV_LINE:
		sh_css_dtrace(2, "  YUYV = %p\n", frame->planes.yuyv.data);
		break;
	case SH_CSS_FRAME_FORMAT_YUV420:
	case SH_CSS_FRAME_FORMAT_YUV422:
	case SH_CSS_FRAME_FORMAT_YUV444:
	case SH_CSS_FRAME_FORMAT_YV12:
	case SH_CSS_FRAME_FORMAT_YV16:
	case SH_CSS_FRAME_FORMAT_YUV420_16:
	case SH_CSS_FRAME_FORMAT_YUV422_16:
		sh_css_dtrace(2, "  Y = %p\n", frame->planes.yuv.y.data);
		sh_css_dtrace(2, "  U = %p\n", frame->planes.yuv.u.data);
		sh_css_dtrace(2, "  V = %p\n", frame->planes.yuv.v.data);
		break;
	case SH_CSS_FRAME_FORMAT_RAW:
		sh_css_dtrace(2, "  RAW = %p\n", frame->planes.raw.data);
		break;
	case SH_CSS_FRAME_FORMAT_RGBA888:
	case SH_CSS_FRAME_FORMAT_RGB565:
		sh_css_dtrace(2, "  RGB = %p\n", frame->planes.rgb.data);
		break;
	case SH_CSS_FRAME_FORMAT_QPLANE6:
		sh_css_dtrace(2, "  R    = %p\n", frame->planes.plane6.r.data);
		sh_css_dtrace(2, "  RatB = %p\n",
				frame->planes.plane6.r_at_b.data);
		sh_css_dtrace(2, "  Gr   = %p\n", frame->planes.plane6.gr.data);
		sh_css_dtrace(2, "  Gb   = %p\n", frame->planes.plane6.gb.data);
		sh_css_dtrace(2, "  B    = %p\n", frame->planes.plane6.b.data);
		sh_css_dtrace(2, "  BatR = %p\n",
				frame->planes.plane6.b_at_r.data);
		break;
	case SH_CSS_FRAME_FORMAT_BINARY_8:
		sh_css_dtrace(2, "  Binary data = %p\n",
				frame->planes.binary.data.data);
		break;
	default:
		sh_css_dtrace(2, "  unknown frame type\n");
		break;
	}
}

void
sh_css_print_sp_debug_state(const struct sh_css_sp_debug_state *state)
{
	int i;

	sh_css_dtrace(2, "sp_error = 0x%x\n", state->error);
	for (i = 0; i < 16; i++)
		sh_css_dtrace(2, "sp_debug[%d] = %d\n", i, state->debug[i]);
}

void
sh_css_dump_rx_state(void)
{
	unsigned int infos = 0, bits;
	bits = sh_css_rx_get_interrupt_reg();
	sh_css_rx_get_interrupt_info(&infos);

	sh_css_dtrace(2, "CSI Receiver errors: (irq reg = 0x%X)\n", bits);

	if (infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		sh_css_dtrace(2, "\tbuffer overrun\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT)
		sh_css_dtrace(2, "\tstart-of-transmission error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		sh_css_dtrace(2, "\tstart-of-transmission sync error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CONTROL)
		sh_css_dtrace(2, "\tcontrol error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		sh_css_dtrace(2, "\t2 or more ECC errors\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_CRC)
		sh_css_dtrace(2, "\tCRC mismatch\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		sh_css_dtrace(2, "\tunknown error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		sh_css_dtrace(2, "\tframe sync error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		sh_css_dtrace(2, "\tframe data error\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		sh_css_dtrace(2, "\tdata timeout\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		sh_css_dtrace(2, "\tunknown escape command entry\n");
	if (infos & SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		sh_css_dtrace(2, "\tline sync error\n");
}

void
sh_css_dump_debug_info(const char *context)
{
	if (!context)
		context = "No Context provided";

	sh_css_dtrace(2, "CSS Debug Info dump [Context = %s]\n", context);
	sh_css_dump_rx_state();
	sh_css_dump_if_state();
	sh_css_dump_isp_state();
	sh_css_dump_isp_sp_fifo_state();
	sh_css_dump_isp_gdc_fifo_state();
	sh_css_dump_sp_state();
	sh_css_dump_dma_isp_fifo_state();
	sh_css_dump_dma_sp_fifo_state();
	sh_css_dump_dma_state();
}

