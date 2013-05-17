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
#include "debug.h"
#include "memory_access.h"

#include "sh_css_debug.h"
#include "sh_css_debug_internal.h"
#include "ia_css_stream.h"

#include "assert_support.h"
#include "print_support.h"
#if !defined(__KERNEL__) && !defined(_MSC_VER)
#include <stdio.h>	/* snprintf() */
#endif

#include "fifo_monitor.h"
#include "input_formatter.h"
#include "dma.h"
#include "irq.h"
#include "gp_device.h"
#include "sp.h"
#include "isp.h"
#include "mmu_device.h"
#if defined(HAS_INPUT_FORMATTER_VERSION_2)
#include "input_system.h"	/* input_formatter_reg_load */
#include "gp_device.h"		/* gp_device_reg_load */
#endif

#include "ia_css.h"
#include "sh_css_internal.h"
#include "sh_css_rx.h"
#include "sh_css_sp.h"	/* sh_css_sp_get_debug_state() */

/* snprintf is a C99 feature, MS visual studio defines _snprintf */
#if defined(_MSC_VER)
#define snprintf _snprintf
#endif

/* Global variable to store the dtrace verbosity level */
unsigned int sh_css_trace_level;

void sh_css_set_dtrace_level(
	const unsigned int	trace_level)
{
	sh_css_trace_level = trace_level;
return;
}

static void print_sp_state(
	const sp_state_t	*state,
	const char			*cell)
{
	sh_css_dtrace(2, "%s state:\n", cell);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "PC", state->pc);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "Status register",
			state->status_register);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is broken", state->is_broken);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is idle", state->is_idle);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is sleeping", state->is_sleeping);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is stalling", state->is_stalling);
return;
}

static void print_isp_state(
	const isp_state_t	*state,
	const char			*cell)
{
	sh_css_dtrace(2, "%s state:\n", cell);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "PC", state->pc);
	sh_css_dtrace(2, "\t%-32s: 0x%X\n", "Status register",
			state->status_register);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is broken", state->is_broken);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is idle", state->is_idle);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is sleeping", state->is_sleeping);
	sh_css_dtrace(2, "\t%-32s: %d\n", "Is stalling", state->is_stalling);
return;
}

void sh_css_dump_isp_state(void)
{
	isp_state_t		state;
	isp_stall_t		stall;

	isp_get_state(ISP0_ID, &state, &stall);

	print_isp_state(&state, "ISP");

	if (state.is_stalling) {
		sh_css_dtrace(2, "\t%-32s: %d\n", "[0] if_prim_a_FIFO stalled",
				stall.fifo0);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[1] if_prim_b_FIFO stalled",
				stall.fifo1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[2] dma_FIFO stalled",
				stall.fifo2);
#if defined(HAS_ISP_2400_MAMOIADA) || defined(HAS_ISP_2400A0_MAMOIADA) || defined(HAS_ISP_2401_MAMOIADA)
		sh_css_dtrace(2, "\t%-32s: %d\n", "[3] gdc0_FIFO stalled",
				stall.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[4] gdc1_FIFO stalled",
				stall.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[5] gpio_FIFO stalled",
				stall.fifo5);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[6] sp_FIFO stalled",
				stall.fifo6);
#elif defined(HAS_ISP_2300_MEDFIELD) || defined(HAS_ISP_2300_MEDFIELD_DEMO)
		sh_css_dtrace(2, "\t%-32s: %d\n", "[3] gdc_FIFO stalled",
				stall.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[4] gpio_FIFO stalled",
				stall.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "[5] sp_FIFO stalled",
				stall.fifo5);
#else
#error "sh_css_debug: ISP cell must be \
	one of {2300_MEDFIELD, 2300_MEDFIELD_DEMO, 2400_MAMOIADA,, 2400A0_MAMOIADA, 2401_MAMOIADA}"
#endif
		sh_css_dtrace(2, "\t%-32s: %d\n", "status & control stalled",
				stall.stat_ctrl);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dmem stalled",
				stall.dmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vmem stalled",
				stall.vmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vamem1 stalled",
				stall.vamem1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "vamem2 stalled",
				stall.vamem2);
#if defined(HAS_ISP_2400_MAMOIADA) || defined(HAS_ISP_2400A0_MAMOIADA) || defined(HAS_ISP_2401_MAMOIADA)
		sh_css_dtrace(2, "\t%-32s: %d\n", "vamem3 stalled",
				stall.vamem3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "hmem stalled",
				stall.hmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "pmem stalled",
				stall.pmem);
#endif
	}
return;
}

void sh_css_dump_sp_state(void)
{
	sp_state_t		state;
	sp_stall_t		stall;
	sp_get_state(SP0_ID, &state, &stall);
	print_sp_state(&state, "SP");
	if (state.is_stalling) {
#if defined(HAS_SP_2400) || defined(HAS_SP_2400A0)
		sh_css_dtrace(2, "\t%-32s: %d\n", "isys_FIFO stalled",
				stall.fifo0);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_sec_FIFO stalled",
				stall.fifo1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "str_to_mem_FIFO stalled",
				stall.fifo2);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dma_FIFO stalled",
				stall.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_a_FIFO stalled",
				stall.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "isp_FIFO stalled",
				stall.fifo5);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gp_FIFO stalled",
				stall.fifo6);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_b_FIFO stalled",
				stall.fifo7);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gdc0_FIFO stalled",
				stall.fifo8);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gdc1_FIFO stalled",
				stall.fifo9);
		sh_css_dtrace(2, "\t%-32s: %d\n", "irq FIFO stalled",
				stall.fifoa);
#elif defined(HAS_SP_2300) || defined(HAS_SP_2300_DEMO)
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_a_FIFO stalled",
				stall.fifo0);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_sec_FIFO stalled",
				stall.fifo1);
		sh_css_dtrace(2, "\t%-32s: %d\n", "str_to_mem_FIFO stalled",
				stall.fifo2);
		sh_css_dtrace(2, "\t%-32s: %d\n", "dma_FIFO stalled",
				stall.fifo3);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gdc_FIFO stalled",
				stall.fifo4);
		sh_css_dtrace(2, "\t%-32s: %d\n", "isp_FIFO stalled",
				stall.fifo5);
		sh_css_dtrace(2, "\t%-32s: %d\n", "gp_FIFO stalled",
				stall.fifo6);
		sh_css_dtrace(2, "\t%-32s: %d\n", "if_prim_b_FIFO stalled",
				stall.fifo7);
#else
#error "sh_css_debug: SP cell must be \
	one of {SP_2300, SP_2300_DEMO, SP2400, SP2400A0}"
#endif
		sh_css_dtrace(2, "\t%-32s: %d\n", "dmem stalled",
				stall.dmem);
		sh_css_dtrace(2, "\t%-32s: %d\n", "control master stalled",
				stall.control_master);
		sh_css_dtrace(2, "\t%-32s: %d\n", "i-cache master stalled",
				stall.icache_master);
	}
return;
}

static void print_if_state(
	input_formatter_state_t		*state)
{
	unsigned int val;

#if defined(HAS_INPUT_FORMATTER_VERSION_1)
	const char *st_reset  = (state->reset ? "Active" : "Not active");
#endif
	const char *st_vsync_active_low =
		(state->vsync_active_low ? "low" : "high");
	const char *st_hsync_active_low = 
		(state->hsync_active_low ? "low" : "high");

	const char *fsm_sync_status_str    = "unknown";
	const char *fsm_crop_status_str    = "unknown";
	const char *fsm_padding_status_str = "unknown";

	int st_stline = state->start_line;
	int st_stcol  = state->start_column;
	int st_crpht  = state->cropped_height;
	int st_crpwd  = state->cropped_width;
	int st_verdcm = state->ver_decimation;
	int st_hordcm = state->hor_decimation;
	int st_ver_deinterleaving = state->ver_deinterleaving;
	int st_hor_deinterleaving = state->hor_deinterleaving;
	int st_leftpd = state->left_padding;
	int st_eoloff = state->eol_offset;
	int st_vmstartaddr = state->vmem_start_address;
	int st_vmendaddr = state->vmem_end_address;
	int st_vmincr = state->vmem_increment;
	int st_yuv420 = state->is_yuv420;
	int st_allow_fifo_overflow = state->allow_fifo_overflow;
	int st_block_fifo_when_no_req = state->block_fifo_when_no_req;

	sh_css_dtrace(2, "InputFormatter State:\n");

	sh_css_dtrace(2, "\tConfiguration:\n");

#if defined(HAS_INPUT_FORMATTER_VERSION_1)
	sh_css_dtrace(2, "\t\t%-32s: %s\n"       ,
			"Software reset"         , st_reset);
#endif
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
			"Ver deinterleaving"         , st_ver_deinterleaving);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Hor deinterleaving"         , st_hor_deinterleaving);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Left padding"           , st_leftpd);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"EOL offset (bytes)"     , st_eoloff);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM start address"     , st_vmstartaddr);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM end address"       , st_vmendaddr);
	sh_css_dtrace(2, "\t\t%-32s: 0x%06X\n"   ,
			"VMEM increment"         , st_vmincr);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"YUV 420 format"         , st_yuv420);
	sh_css_dtrace(2, "\t\t%-32s: Active %s\n",
			"Vsync"                  , st_vsync_active_low);
	sh_css_dtrace(2, "\t\t%-32s: Active %s\n",
			"Hsync"                  , st_hsync_active_low);
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Allow FIFO overflow", st_allow_fifo_overflow);
/* Flag that tells whether the IF gives backpressure on frames */
/*
 * FYI, this is only on the frame request (indicate), when the IF has
 * synch'd on a frame it will always give back pressure
 */
	sh_css_dtrace(2, "\t\t%-32s: %d\n"       ,
			"Block when no request", st_block_fifo_when_no_req);


#if defined(HAS_INPUT_FORMATTER_VERSION_2)
	/***********************************************************
	 * Hack for WHQL.
	 *
	 * AUTHOR: zhengjie.lu@intel.com
	 * DATE: 2013-01-16
	 * TIME: 12:35
	 * LOCATION: Santa Clara
	 * COMMENT: Print out the value of the register that tells
	 * whehter the IF is set to the blocked mode (i.e. "1") or
	 * not (i.e. "0").
	 ***********************************************************/	
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
			"IF_BLOCKED_FIFO_NO_REQ_ADDRESS",
			input_formatter_reg_load(INPUT_FORMATTER0_ID, 							HIVE_IF_BLOCK_FIFO_NO_REQ_ADDRESS)
			);
	/** End of hack for WHQL ***********************************/

	/***********************************************************
	 * Hack for WHQL.
	 *
	 * AUTHOR: zhengjie.lu@intel.com
	 * DATE: 2013-01-23
	 * TIME: 20:14
	 * LOCATION: Santa Clara
	 * COMMENT: Print out the input switch states.
	 * not (i.e. "0").
	 ***********************************************************/	
	sh_css_dtrace(2, "\t%-32s:\n",
		"InputSwitch State");

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg0",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg0));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg1",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg1));

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg2",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg2));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg3",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg3));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg4",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg4));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg5",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg5));

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg6",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg6));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_lut_reg7",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_lut_reg7));

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_input_switch_fsync_lut",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_input_switch_fsync_lut));

	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_srst",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_srst));
		
	sh_css_dtrace(2, "\t\t%-32s: %d\n",
		"_REG_GP_IFMT_slv_reg_srst",
		gp_device_reg_load(GP_DEVICE0_ID, _REG_GP_IFMT_slv_reg_srst));
	/** End of Hack for WHQL **********************************/

#endif

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
return;
}

void sh_css_dump_if_state(void)
{
	input_formatter_state_t		state;
	input_formatter_get_state(INPUT_FORMATTER0_ID, &state);
	print_if_state(&state);
	sh_css_dump_pif_isp_fifo_state();
return;
}

void sh_css_dump_dma_state(void)
{
	dma_state_t		state;
	int i, ch_id;

	const char *fsm_cmd_st_lbl = "FSM Command flag state";
	const char *fsm_ctl_st_lbl = "FSM Control flag state";
	const char *fsm_ctl_state  = NULL;
	const char *fsm_ctl_flag   = NULL;
	const char *fsm_pack_st    = NULL;
	const char *fsm_read_st    = NULL;
	const char *fsm_write_st   = NULL;
	char last_cmd_str[64];

	dma_get_state(DMA0_ID, &state);
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
	case DMA_COMMAND_READ:
		snprintf(last_cmd_str, 64,
		  "Read 2D Block [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_WRITE:
		snprintf(last_cmd_str, 64,
		  "Write 2D Block [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_SET_CHANNEL:
		snprintf(last_cmd_str, 64,
		  "Set Channel [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_SET_PARAM:
		snprintf(last_cmd_str, 64,
		  "Set Param: %d [Channel: %d]",
		  state.last_command_param, ch_id);
		break;
	case DMA_COMMAND_READ_SPECIFIC:
		snprintf(last_cmd_str, 64,
		  "Read Specific 2D Block [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_WRITE_SPECIFIC:
		snprintf(last_cmd_str, 64,
		  "Write Specific 2D Block [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_INIT:
		snprintf(last_cmd_str, 64,
		  "Init 2D Block on Device A [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_INIT_SPECIFIC:
		snprintf(last_cmd_str, 64,
		  "Init Specific 2D Block [Channel: %d]", ch_id);
		break;
	case DMA_COMMAND_RST:
		snprintf(last_cmd_str, 64,
		  "DMA SW Reset");
		break;
	case N_DMA_COMMANDS:
		snprintf(last_cmd_str, 64,
		  "UNKNOWN");
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
	case DMA_CTRL_STATE_IDLE:
		fsm_ctl_state = "Idle state";
		break;
	case DMA_CTRL_STATE_REQ_RCV:
		fsm_ctl_state = "Req Rcv state";
		break;
	case DMA_CTRL_STATE_RCV:
		fsm_ctl_state = "Rcv state";
		break;
	case DMA_CTRL_STATE_RCV_REQ:
		fsm_ctl_state = "Rcv Req state";
		break;
	case DMA_CTRL_STATE_INIT:
		fsm_ctl_state = "Init state";
		break;
	case N_DMA_CTRL_STATES:
		fsm_ctl_state = "Unknown";
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

	if (state.read_state == DMA_RW_STATE_IDLE)
		fsm_read_st = "Idle state";
	if (state.read_state == DMA_RW_STATE_REQ)
		fsm_read_st = "Req state";
	if (state.read_state == DMA_RW_STATE_NEXT_LINE)
		fsm_read_st = "Next line";
	if (state.read_state == DMA_RW_STATE_UNLOCK_CHANNEL)
		fsm_read_st = "Unlock channel";

	sh_css_dtrace(2, "\t\t%-32s: %s\n", "FSM Read state", fsm_read_st);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Read cnt height",
			state.read_cnt_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Read cnt width",
			state.read_cnt_width);

	if (state.write_state == DMA_RW_STATE_IDLE)
		fsm_write_st = "Idle state";
	if (state.write_state == DMA_RW_STATE_REQ)
		fsm_write_st = "Req state";
	if (state.write_state == DMA_RW_STATE_NEXT_LINE)
		fsm_write_st = "Next line";
	if (state.write_state == DMA_RW_STATE_UNLOCK_CHANNEL)
		fsm_write_st = "Unlock channel";

	sh_css_dtrace(2, "\t\t%-32s: %s\n", "FSM Write state", fsm_write_st);

	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Write height",
			state.write_height);
	sh_css_dtrace(2, "\t\t%-32s: %d\n", "FSM Write width",
			state.write_width);

	for (i = 0; i < HIVE_ISP_NUM_DMA_CONNS; i++) {
		dma_port_state_t	*port = &(state.port_states[i]);
		sh_css_dtrace(2, "\tDMA device interface %d\n", i);
		sh_css_dtrace(2, "\t\tDMA internal side state\n");
		sh_css_dtrace(2, "\t\t\tCS:%d - We_n:%d - Run:%d - Ack:%d\n",
				port->req_cs,
				port->req_we_n,
				port->req_run,
				port->req_ack);
		sh_css_dtrace(2, "\t\tMaster Output side state\n");
		sh_css_dtrace(2, "\t\t\tCS:%d - We_n:%d - Run:%d - Ack:%d\n",
				port->send_cs,
				port->send_we_n,
				port->send_run,
				port->send_ack);
		sh_css_dtrace(2, "\t\tFifo state\n");
		if (port->fifo_state == DMA_FIFO_STATE_WILL_BE_FULL)
			sh_css_dtrace(2, "\t\t\tFiFo will be full\n");
		else if (port->fifo_state == DMA_FIFO_STATE_FULL)
			sh_css_dtrace(2, "\t\t\tFifo Full\n");
		else if (port->fifo_state == DMA_FIFO_STATE_EMPTY)
			sh_css_dtrace(2, "\t\t\tFifo Empty\n");
		else
			sh_css_dtrace(2, "\t\t\tFifo state unknown\n");

		sh_css_dtrace(2, "\t\tFifo counter %d\n\n",
				port->fifo_counter);
	}

	for (i = 0; i < HIVE_DMA_NUM_CHANNELS; i++) {
		dma_channel_state_t	*ch = &(state.channel_states[i]);
		sh_css_dtrace(2, "\t%-32s: %d\n", "DMA channel register",
				i);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Connection",
				ch->connection);
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Sign extend",
				ch->sign_extend);
#if defined(IS_DMA_VERSION_1)
		sh_css_dtrace(2, "\t\t%-32s: %d\n", "Reverse elems",
				ch->reverse_elem_order);
#endif
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
return;
}

static void print_fifo_channel_state(
	const fifo_channel_state_t	*state,
	const char					*descr)
{
	sh_css_dtrace(2, "FIFO channel: %s\n", descr);
	sh_css_dtrace(2, "\t%-32s: %d\n", "source valid", state->src_valid);
	sh_css_dtrace(2, "\t%-32s: %d\n", "fifo accept" , state->fifo_accept);
	sh_css_dtrace(2, "\t%-32s: %d\n", "fifo valid"  , state->fifo_valid);
	sh_css_dtrace(2, "\t%-32s: %d\n", "sink accept" , state->sink_accept);
return;
}

void sh_css_dump_pif_isp_fifo_state(void)
{
	fifo_channel_state_t pif_to_isp, isp_to_pif;
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_IF0_TO_ISP0, &pif_to_isp);
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_ISP0_TO_IF0, &isp_to_pif);
	print_fifo_channel_state(&pif_to_isp, "Primary IF A to ISP");
	print_fifo_channel_state(&isp_to_pif, "ISP to Primary IF A)");
return;
}

void sh_css_dump_dma_sp_fifo_state(void)
{
	fifo_channel_state_t dma_to_sp, sp_to_dma;
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_DMA0_TO_SP0, &dma_to_sp);
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_SP0_TO_DMA0, &sp_to_dma);
	print_fifo_channel_state(&dma_to_sp, "DMA to SP");
	print_fifo_channel_state(&sp_to_dma, "SP to DMA");
return;
}

void sh_css_dump_dma_isp_fifo_state(void)
{
	fifo_channel_state_t dma_to_isp, isp_to_dma;
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_DMA0_TO_ISP0, &dma_to_isp);
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_ISP0_TO_DMA0, &isp_to_dma);
	print_fifo_channel_state(&dma_to_isp, "DMA to ISP");
	print_fifo_channel_state(&isp_to_dma, "ISP to DMA");
return;
}

void sh_css_dump_isp_sp_fifo_state(void)
{
	fifo_channel_state_t sp_to_isp, isp_to_sp;
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_SP0_TO_ISP0, &sp_to_isp);
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_ISP0_TO_SP0, &isp_to_sp);
	print_fifo_channel_state(&sp_to_isp, "SP to ISP");
	print_fifo_channel_state(&isp_to_sp, "ISP to SP");
return;
}

void sh_css_dump_isp_gdc_fifo_state(void)
{
	fifo_channel_state_t gdc_to_isp, isp_to_gdc;

	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_GDC0_TO_ISP0, &gdc_to_isp);
	fifo_channel_get_state(FIFO_MONITOR0_ID,
		FIFO_CHANNEL_ISP0_TO_GDC0, &isp_to_gdc);
	print_fifo_channel_state(&gdc_to_isp, "GDC to ISP");
	print_fifo_channel_state(&isp_to_gdc, "ISP to GDC");
return;
}

void sh_css_dump_all_fifo_state(void)
{
	int	i;
	fifo_monitor_state_t	state;
	fifo_monitor_get_state(FIFO_MONITOR0_ID, &state);

	for (i = 0; i < N_FIFO_CHANNEL; i++)
		print_fifo_channel_state(&(state.fifo_channels[i]),
			"squepfstqkt");
return;
}

static void sh_css_binary_info_print(
	const struct ia_css_binary_info *info)
{
	sh_css_dtrace(2, "id = %d\n", info->id);
	sh_css_dtrace(2, "mode = %d\n", info->mode);
	sh_css_dtrace(2, "max_input_width = %d\n", info->max_input_width);
	sh_css_dtrace(2, "min_output_width = %d\n", info->min_output_width);
	sh_css_dtrace(2, "max_output_width = %d\n", info->max_output_width);
	sh_css_dtrace(2, "top_cropping = %d\n", info->top_cropping);
	sh_css_dtrace(2, "left_cropping = %d\n", info->left_cropping);
	sh_css_dtrace(2, "xmem_addr = %d\n", info->xmem_addr);
	sh_css_dtrace(2, "enable_vf_veceven = %d\n", info->enable.vf_veceven);
	sh_css_dtrace(2, "enable_dis = %d\n", info->enable.dis);
	sh_css_dtrace(2, "enable_uds = %d\n", info->enable.uds);
	sh_css_dtrace(2, "enable ds = %d\n", info->enable.ds);
	sh_css_dtrace(2, "s3atbl_use_dmem = %d\n", info->s3atbl_use_dmem);
return;
}

void sh_css_binary_print(
	const struct sh_css_binary *bi)
{
	sh_css_binary_info_print(bi->info);
	sh_css_dtrace(2, "input:  %dx%d, format = %d, padded width = %d\n",
		     bi->in_frame_info.res.width, bi->in_frame_info.res.height,
		     bi->in_frame_info.format, bi->in_frame_info.padded_width);
	sh_css_dtrace(2, "internal :%dx%d, format = %d, padded width = %d\n",
		     bi->internal_frame_info.res.width,
		     bi->internal_frame_info.res.height,
		     bi->internal_frame_info.format,
		     bi->internal_frame_info.padded_width);
	sh_css_dtrace(2, "out:    %dx%d, format = %d, padded width = %d\n",
		     bi->out_frame_info.res.width, bi->out_frame_info.res.height,
		     bi->out_frame_info.format,
		     bi->out_frame_info.padded_width);
	sh_css_dtrace(2, "vf out: %dx%d, format = %d, padded width = %d\n",
		     bi->vf_frame_info.res.width, bi->vf_frame_info.res.height,
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
return;
}

void sh_css_frame_print(
	const struct ia_css_frame	*frame,
	const char					*descr)
{
	char *data = (char *)HOST_ADDRESS(frame->data);
	sh_css_dtrace(2, "frame %s (%p):\n", descr, frame);
	sh_css_dtrace(2, "  resolution    = %dx%d\n",
		     frame->info.res.width, frame->info.res.height);
	sh_css_dtrace(2, "  padded width  = %d\n", frame->info.padded_width);
	sh_css_dtrace(2, "  format        = %d\n", frame->info.format);
	sh_css_dtrace(2, "  is contiguous = %s\n",
		     frame->contiguous ? "yes" : "no");
	switch (frame->info.format) {
	case IA_CSS_FRAME_FORMAT_NV12:
	case IA_CSS_FRAME_FORMAT_NV16:
	case IA_CSS_FRAME_FORMAT_NV21:
	case IA_CSS_FRAME_FORMAT_NV61:
		sh_css_dtrace(2, "  Y = %p\n",
				data + frame->planes.nv.y.offset);
		sh_css_dtrace(2, "  UV = %p\n",
				data + frame->planes.nv.uv.offset);
		break;
	case IA_CSS_FRAME_FORMAT_YUYV:
	case IA_CSS_FRAME_FORMAT_UYVY:
	case IA_CSS_FRAME_FORMAT_YUV_LINE:
		sh_css_dtrace(2, "  YUYV = %p\n",
				data + frame->planes.yuyv.offset);
		break;
	case IA_CSS_FRAME_FORMAT_YUV420:
	case IA_CSS_FRAME_FORMAT_YUV422:
	case IA_CSS_FRAME_FORMAT_YUV444:
	case IA_CSS_FRAME_FORMAT_YV12:
	case IA_CSS_FRAME_FORMAT_YV16:
	case IA_CSS_FRAME_FORMAT_YUV420_16:
	case IA_CSS_FRAME_FORMAT_YUV422_16:
		sh_css_dtrace(2, "  Y = %p\n",
				data + frame->planes.yuv.y.offset);
		sh_css_dtrace(2, "  U = %p\n",
				data + frame->planes.yuv.u.offset);
		sh_css_dtrace(2, "  V = %p\n",
				data + frame->planes.yuv.v.offset);
		break;
	case IA_CSS_FRAME_FORMAT_RAW:
		sh_css_dtrace(2, "  RAW = %p\n",
				data + frame->planes.raw.offset);
		break;
	case IA_CSS_FRAME_FORMAT_RGBA888:
	case IA_CSS_FRAME_FORMAT_RGB565:
		sh_css_dtrace(2, "  RGB = %p\n",
				data + frame->planes.rgb.offset);
		break;
	case IA_CSS_FRAME_FORMAT_QPLANE6:
		sh_css_dtrace(2, "  R    = %p\n",
				data + frame->planes.plane6.r.offset);
		sh_css_dtrace(2, "  RatB = %p\n",
				data + frame->planes.plane6.r_at_b.offset);
		sh_css_dtrace(2, "  Gr   = %p\n",
				data + frame->planes.plane6.gr.offset);
		sh_css_dtrace(2, "  Gb   = %p\n",
				data + frame->planes.plane6.gb.offset);
		sh_css_dtrace(2, "  B    = %p\n",
				data + frame->planes.plane6.b.offset);
		sh_css_dtrace(2, "  BatR = %p\n",
				data + frame->planes.plane6.b_at_r.offset);
		break;
	case IA_CSS_FRAME_FORMAT_BINARY_8:
		sh_css_dtrace(2, "  Binary data = %p\n",
				data + frame->planes.binary.data.offset);
		break;
	default:
		sh_css_dtrace(2, "  unknown frame type\n");
		break;
	}
return;
}

#if SP_DEBUG !=SP_DEBUG_NONE

void sh_css_print_sp_debug_state(
	const struct sh_css_sp_debug_state	*state)
{

#endif

#if SP_DEBUG == SP_DEBUG_DUMP

	sh_css_dtrace(SH_DBG_DEBUG, "current SP software counter: %d\n",
				state->debug[0]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty output buffer queue head: 0x%x\n",
				state->debug[1]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty output buffer queue tail: 0x%x\n",
				state->debug[2]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a buffer queue head: 0x%x\n",
				state->debug[3]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a buffer queue tail: 0x%x\n",
				state->debug[4]);
	sh_css_dtrace(SH_DBG_DEBUG, "full output buffer queue head: 0x%x\n",
				state->debug[5]);
	sh_css_dtrace(SH_DBG_DEBUG, "full output buffer queue tail: 0x%x\n",
				state->debug[6]);
	sh_css_dtrace(SH_DBG_DEBUG, "full s3a buffer queue head: 0x%x\n",
				state->debug[7]);
	sh_css_dtrace(SH_DBG_DEBUG, "full s3a buffer queue tail: 0x%x\n",
				state->debug[8]);
	sh_css_dtrace(SH_DBG_DEBUG, "event queue head: 0x%x\n",
				state->debug[9]);
	sh_css_dtrace(SH_DBG_DEBUG, "event queue tail: 0x%x\n",
				state->debug[10]);
	sh_css_dtrace(SH_DBG_DEBUG, "num of stages of current pipeline: 0x%x\n",
				state->debug[11]);
	sh_css_dtrace(SH_DBG_DEBUG, "DDR address of stage 1: 0x%x\n",
				state->debug[12]);
	sh_css_dtrace(SH_DBG_DEBUG, "DDR address of stage 2: 0x%x\n",
				state->debug[13]);
	sh_css_dtrace(SH_DBG_DEBUG, "current stage out_vf buffer idx: 0x%x\n",
				state->debug[14]);
	sh_css_dtrace(SH_DBG_DEBUG, "current stage output buffer idx: 0x%x\n",
				state->debug[15]);
	sh_css_dtrace(SH_DBG_DEBUG, "current stage s3a buffer idx: 0x%x\n",
				state->debug[16]);
	sh_css_dtrace(SH_DBG_DEBUG, "first char of current stage name: 0x%x\n",
				state->debug[17]);
	sh_css_dtrace(SH_DBG_DEBUG, "current SP thread id: 0x%x\n",
				state->debug[18]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty output buffer address 1: 0x%x\n",
				state->debug[19]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty output buffer address 2: 0x%x\n",
				state->debug[20]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty out_vf buffer address 1: 0x%x\n",
				state->debug[21]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty out_vf buffer address 2: 0x%x\n",
				state->debug[22]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a_hi buffer address 1: 0x%x\n",
				state->debug[23]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a_hi buffer address 2: 0x%x\n",
				state->debug[24]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a_lo buffer address 1: 0x%x\n",
				state->debug[25]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty s3a_lo buffer address 2: 0x%x\n",
				state->debug[26]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty dis_hor buffer address 1: 0x%x\n",
				state->debug[27]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty dis_hor buffer address 2: 0x%x\n",
				state->debug[28]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty dis_ver buffer address 1: 0x%x\n",
				state->debug[29]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty dis_ver buffer address 2: 0x%x\n",
				state->debug[30]);
	sh_css_dtrace(SH_DBG_DEBUG, "empty param buffer address: 0x%x\n",
				state->debug[31]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect frame address: 0x%x\n",
				state->debug[32]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect frame container address: 0x%x\n",
				state->debug[33]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect frame container payload: 0x%x\n",
				state->debug[34]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_hi address: 0x%x\n",
				state->debug[35]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_hi container address: 0x%x\n",
				state->debug[36]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_hi container payload: 0x%x\n",
				state->debug[37]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_lo address: 0x%x\n",
				state->debug[38]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_lo container address: 0x%x\n",
				state->debug[39]);
	sh_css_dtrace(SH_DBG_DEBUG, "first incorrect s3a_lo container payload: 0x%x\n",
				state->debug[40]);
	sh_css_dtrace(SH_DBG_DEBUG, "number of calling flash start function: 0x%x\n",
				state->debug[41]);
	sh_css_dtrace(SH_DBG_DEBUG, "number of calling flash close function: 0x%x\n",
				state->debug[42]);
	sh_css_dtrace(SH_DBG_DEBUG, "number of flashed frame: 0x%x\n",
				state->debug[43]);
	sh_css_dtrace(SH_DBG_DEBUG, "flash in use flag: 0x%x\n",
				state->debug[44]);
	sh_css_dtrace(SH_DBG_DEBUG, "number of update frame flashed flag: 0x%x\n",
				state->debug[46]);
	sh_css_dtrace(SH_DBG_DEBUG, "number of active threads: 0x%x\n",
				state->debug[45]);


#elif SP_DEBUG == SP_DEBUG_COPY

	/* Remember last_index because we only want to print new entries */
	static int last_index = 0;
	int sp_index = state->index;
	int n;

	if (sp_index < last_index) {
		/* SP has been reset */
		last_index = 0;
	}

	if (last_index == 0) {
		sh_css_dtrace(SH_DBG_DEBUG, 
		  "copy-trace init: sp_dbg_if_start_line=%d, sp_dbg_if_start_column=%d, "
		  "sp_dbg_if_cropped_height=%d, sp_debg_if_cropped_width=%d\n",
		  state->if_start_line,
		  state->if_start_column,
		  state->if_cropped_height,
		  state->if_cropped_width
		  );
	}

	if ((last_index + SH_CSS_SP_DBG_TRACE_DEPTH) < sp_index) {
		/* last index can be multiple rounds behind */
		/* while trace size is only SH_CSS_SP_DBG_TRACE_DEPTH */
		last_index = sp_index - SH_CSS_SP_DBG_TRACE_DEPTH;
	}

	for (n = last_index; n < sp_index; n++) {
		int i = n % SH_CSS_SP_DBG_TRACE_DEPTH;
		if (state->trace[i].frame != 0) {
			sh_css_dtrace(SH_DBG_DEBUG, 
			  "copy-trace: frame=%d, line=%d, "
			  "pixel_distance=%d, mipi_used_dword=%d, sp_index=%d\n",
			  state->trace[i].frame,
			  state->trace[i].line,
			  state->trace[i].pixel_distance,
			  state->trace[i].mipi_used_dword,
			  state->trace[i].sp_index
			  );
		}
	}

	last_index = sp_index;

#elif SP_DEBUG == SP_DEBUG_TRACE

#if 1

	static char const *id2filename[8] = {
		"N.A.", "sp.hive.c", "event.sp.c", "sp_raw_copy.hive.c",
		"buffer_queue.sp.c", "event_proxy_sp.hive.c", "ERROR", "ERROR"
		};

#if 0
	static char const *trace_name[SH_CSS_SP_DBG_NR_OF_TRACES] = {
				"copy", "preview/video", "capture",
				"acceleration", "control", "TBD" };
#else
	static char const *trace_name[SH_CSS_SP_DBG_NR_OF_TRACES] = {
				"copy", "preview/video", "event_ext_io",
				"event_PIF_A" };
#endif
	int t, d;

	for (t = 0; t < SH_CSS_SP_DBG_NR_OF_TRACES; t++) {
		/* base contains the "oldest" index */
		int base = state->index[t];
		for (d = 0; d < SH_CSS_SP_DBG_TRACE_DEPTH; d++) {
			int i = (base + d) % SH_CSS_SP_DBG_TRACE_DEPTH;
			int l = state->trace[t][i].location &
				((1<<SH_CSS_SP_DBG_TRACE_FILE_ID_BIT_POS)-1);
			int fid = state->trace[t][i].location >>
				SH_CSS_SP_DBG_TRACE_FILE_ID_BIT_POS;
			int ts = state->trace[t][i].time_stamp;

			if (ts) {
				sh_css_dtrace(SH_DBG_DEBUG, 
				  "%05d trace=%s, file=%s:%d, data=0x%08x\n",
				  ts,
				  trace_name[t],
				  id2filename[fid], l,
				  state->trace[t][i].data);
			}
		}
	}


#endif

#elif SP_DEBUG == SP_DEBUG_MINIMAL
	int i;
	int base = 0;
	int limit = SH_CSS_NUM_SP_DEBUG;
	int step = 1;
	
	for (i = base; i < limit; i += step) {
		sh_css_dtrace(SH_DBG_DEBUG,
			"sp_dbg_trace[%d] = %d\n",
			i,
			state->debug[i]);
	}
#endif

#if SP_DEBUG !=SP_DEBUG_NONE

	return;
}
#endif

#if defined(HAS_INPUT_FORMATTER_VERSION_2)
static void print_rx_mipi_port_state(
		mipi_port_state_t *state)
{
	int i;

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"device_ready"		, state->device_ready);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"irq_status"		, state->irq_status);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"irq_enable"		, state->irq_enable);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"timeout_count"		, state->timeout_count);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"init_count"		, state->init_count);


	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"raw16_18"		, state->raw16_18);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"sync_count"		, state->sync_count);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"rx_count"		, state->rx_count);


	for (i = 0; i < MIPI_4LANE_CFG; i++) {
		sh_css_dtrace(2, "\t\t%-32s%d%-32s: %d\n"	,
				"lane_sync_count[", i, "]", state->lane_sync_count[i]);
	}

	for (i = 0; i < MIPI_4LANE_CFG; i++) {
		sh_css_dtrace(2, "\t\t%-32s%d%-32s: %d\n"	,
				"lane_rx_count[", i, "]", state->lane_rx_count[i]);		
	}

	return;
}

static void print_rx_channel_state(
		rx_channel_state_t *state)
{
	int i;

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"compression_scheme0"	, state->comp_scheme0);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"compression_scheme1"	, state->comp_scheme1);

	for (i = 0; i < N_MIPI_FORMAT_CUSTOM; i++) {
		sh_css_dtrace(2, "\t\t%-32s%d: %d\n"	,
				"MIPI Predictor ", i, state->pred[i]);
	}

	for (i = 0; i < N_MIPI_FORMAT_CUSTOM; i++) {
		sh_css_dtrace(2, "\t\t%-32s%d: %d\n"	,
				"MIPI Compressor ", i, state->comp[i]);
	}	

	return;
}

static void print_rx_state(
		receiver_state_t *state)
{
	int i;

	sh_css_dtrace(2, "CSI Receiver State:\n");

	sh_css_dtrace(2, "\tConfiguration:\n");

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"fs_to_ls_delay"	, state->fs_to_ls_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"ls_to_data_delay"	, state->ls_to_data_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"data_to_le_delay"	, state->data_to_le_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"le_to_fe_delay"	, state->le_to_fe_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"fe_to_fs_delay"	, state->fe_to_fs_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"le_to_fs_delay"	, state->le_to_fs_delay);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"is_two_ppc"		, state->is_two_ppc);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"backend_rst"		, state->backend_rst);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"raw18"			, state->raw18);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"force_raw8"		, state->force_raw8);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"raw16"	, state->raw16);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_gsp_acc_ovl"	, state->be_gsp_acc_ovl);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_srst"		, state->be_srst);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_is_two_ppc"		, state->be_is_two_ppc);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_comp_format0"	, state->be_comp_format0);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_comp_format1"	, state->be_comp_format1);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_comp_format2"	, state->be_comp_format2);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_comp_format3"	, state->be_comp_format3);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_sel"		, state->be_sel);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_raw16_config"	, state->be_raw16_config);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_raw18_config"	, state->be_raw18_config);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_force_raw8"		, state->be_force_raw8);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_irq_status"		, state->be_irq_status);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"be_irq_clear"		, state->be_irq_clear);

	/* mipi port state */
	for (i = 0; i < N_MIPI_PORT_ID; i++) {
		sh_css_dtrace(2, "\tMIPI Port %d State:\n",
				i);

		print_rx_mipi_port_state(&state->mipi_port_state[i]);
	}
	/* end of mipi port state */

	/* rx channel state */
	for (i = 0; i < N_RX_CHANNEL_ID; i++) {
		sh_css_dtrace(2, "\tRX Channel %d State:\n",
				i);
		
		print_rx_channel_state(&state->rx_channel_state[i]);
	}
	/* end of rx channel state */

	return;
}
#endif

void sh_css_dump_rx_state(void)
{
#if defined(HAS_INPUT_FORMATTER_VERSION_2)
	receiver_state_t state;
#endif
	unsigned int infos = 0, bits;
	bits = sh_css_rx_get_interrupt_reg();
	ia_css_rx_get_irq_info(&infos);

	sh_css_dtrace(2, "CSI Receiver errors: (irq reg = 0x%X)\n", bits);

	if (infos & IA_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		sh_css_dtrace(2, "\tbuffer overrun\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_SOT)
		sh_css_dtrace(2, "\tstart-of-transmission error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		sh_css_dtrace(2, "\tstart-of-transmission sync error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_CONTROL)
		sh_css_dtrace(2, "\tcontrol error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		sh_css_dtrace(2, "\t2 or more ECC errors\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_CRC)
		sh_css_dtrace(2, "\tCRC mismatch\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		sh_css_dtrace(2, "\tunknown error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		sh_css_dtrace(2, "\tframe sync error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		sh_css_dtrace(2, "\tframe data error\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		sh_css_dtrace(2, "\tdata timeout\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		sh_css_dtrace(2, "\tunknown escape command entry\n");
	if (infos & IA_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		sh_css_dtrace(2, "\tline sync error\n");

#if defined(HAS_INPUT_FORMATTER_VERSION_2)
	receiver_get_state(RX0_ID, &state);
	print_rx_state(&state);
#endif
return;
}

void
sh_css_dump_sp_sw_debug_info(void)
{
#if SP_DEBUG !=SP_DEBUG_NONE
	struct sh_css_sp_debug_state state;

	sh_css_sp_get_debug_state(&state);
	sh_css_print_sp_debug_state(&state);
#endif
return;
}

#if defined(HAS_INPUT_FORMATTER_VERSION_2)
static void print_isys_capture_unit_state(
	capture_unit_state_t *state)
{
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Packet_Length"		, state->Packet_Length);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Received_Length"	, state->Received_Length);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"		,
			"Received_Short_Packets"	, state->Received_Short_Packets);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Received_Long_Packets"	, state->Received_Long_Packets);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Last_Command"		, state->Last_Command);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Next_Command"		, state->Next_Command);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Last_Acknowledge"	, state->Last_Acknowledge);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Next_Acknowledge"	, state->Next_Acknowledge);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"FSM_State_Info"	, state->FSM_State_Info);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"StartMode"		, state->StartMode);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Start_Addr"		, state->Start_Addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Mem_Region_Size"	, state->Mem_Region_Size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Num_Mem_Regions"	, state->Num_Mem_Regions);
#if 0	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Init"			, state->Init);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Start"			, state->Start);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Stop"			, state->Stop);
#endif
	return;
}

static void print_isys_acquisition_unit_state(
	acquisition_unit_state_t *state)
{
#if 0
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Init"			, state->Init);
#endif
	sh_css_dtrace(2, "\t\t%-32s: %d\n"		,
			"Received_Short_Packets"	, state->Received_Short_Packets);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Received_Long_Packets"	, state->Received_Long_Packets);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Last_Command"		, state->Last_Command);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Next_Command"		, state->Next_Command);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Last_Acknowledge"	, state->Last_Acknowledge);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Next_Acknowledge"	, state->Next_Acknowledge);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"FSM_State_Info"	, state->FSM_State_Info);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Int_Cntr_Info"		, state->Int_Cntr_Info);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Start_Addr"		, state->Start_Addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Mem_Region_Size"	, state->Mem_Region_Size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"Num_Mem_Regions"	, state->Num_Mem_Regions);
}

static void print_isys_ctrl_unit_state(
	ctrl_unit_state_t *state)
{
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"last_cmd"		, state->last_cmd);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"next_cmd"		, state->next_cmd);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"last_ack"		, state->last_ack);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"next_ack"		, state->next_ack);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"top_fsm_state"		, state->top_fsm_state);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captA_fsm_state"	, state->captA_fsm_state);	

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captB_fsm_state"	, state->captB_fsm_state);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captC_fsm_state"	, state->captC_fsm_state);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"acq_fsm_state"		, state->acq_fsm_state);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captA_start_addr"	, state->captA_start_addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captB_start_addr"	, state->captB_start_addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captC_start_addr"	, state->captC_start_addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captA_mem_region_size"	, state->captA_mem_region_size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captB_mem_region_size"	, state->captB_mem_region_size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captC_mem_region_size"	, state->captC_mem_region_size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captA_num_mem_regions"	, state->captA_num_mem_regions);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captB_num_mem_regions"	, state->captB_num_mem_regions);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"captC_num_mem_regions"	, state->captC_num_mem_regions);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"acq_start_addr"	, state->acq_start_addr);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"acq_mem_region_size"	, state->acq_mem_region_size);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"acq_num_mem_regions"	, state->acq_num_mem_regions);
#if 0
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"ctrl_init"		, state->ctrl_init);
#endif
	sh_css_dtrace(2, "\t\t%-32s: %d\n"		,
			"capt_reserve_one_mem_region"	, state->capt_reserve_one_mem_region);

	return;
}

static void print_isys_state(
	input_system_state_t *state)
{
	int i;

	sh_css_dtrace(2, "InputSystem State:\n");

	/* configuration */
	sh_css_dtrace(2, "\tConfiguration:\n");

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_multiCastA_sel"	, state->str_multicastA_sel);
	
	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_multicastB_sel"	, state->str_multicastB_sel);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_multicastC_sel"	, state->str_multicastC_sel);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_mux_sel"		, state->str_mux_sel);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_mon_status"	, state->str_mon_status);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_mon_irq_cond"	, state->str_mon_irq_cond);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_mon_irq_en"	, state->str_mon_irq_en);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"isys_srst"		, state->isys_srst);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"isys_slv_reg_srst"	, state->isys_slv_reg_srst);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_deint_portA_cnt"	, state->str_deint_portA_cnt);

	sh_css_dtrace(2, "\t\t%-32s: %d\n"	,
			"str_deint_portB_cnd"	, state->str_deint_portB_cnt);
	/* end of configuration */

	/* capture unit state */
	for (i = 0; i < N_CAPTURE_UNIT_ID; i++) {
		capture_unit_state_t *capture_unit_state;

		sh_css_dtrace(2, "\tCaptureUnit %d State:\n",
				i);

		capture_unit_state = &state->capture_unit[i];
		print_isys_capture_unit_state(capture_unit_state);
	}
	/* end of capture unit state */

	/* acquisition unit state */
	for (i = 0; i < N_ACQUISITION_UNIT_ID; i++) {
		acquisition_unit_state_t *acquisition_unit_state;

		sh_css_dtrace(2, "\tAcquisitionUnit %d State:\n",
				i);

		acquisition_unit_state = &state->acquisition_unit[i];
		print_isys_acquisition_unit_state(acquisition_unit_state);
	}
	/* end of acquisition unit state */

	/* control unit state */
	for (i = 0; i < N_CTRL_UNIT_ID; i++) {
		sh_css_dtrace(2, "\tControlUnit %d State:\n",
				i);

		print_isys_ctrl_unit_state(&state->ctrl_unit_state[i]);
	}
	/* end of control unit state */
}

void sh_css_dump_isys_state(void)
{
	input_system_state_t state;	
	
	input_system_get_state(INPUT_SYSTEM0_ID, &state);
	print_isys_state(&state);

	return;
}
#endif

void sh_css_dump_debug_info(
	const char	*context)
{
	if (context == NULL)
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
#if defined(HAS_INPUT_FORMATTER_VERSION_2)
	sh_css_dump_isys_state();
	
    {
        irq_controller_state_t state;
        irq_controller_get_state(IRQ2_ID, &state);
    
        sh_css_dtrace(2, "\t%-32s:\n" ,
         "Input System IRQ Controller State");

        sh_css_dtrace(2, "\t\t%-32s: %d\n"  ,
                                "irq_edge"  , state.irq_edge);

        sh_css_dtrace(2, "\t\t%-32s: %d\n"  ,
                                "irq_mask"  , state.irq_mask);
        
        sh_css_dtrace(2, "\t\t%-32s: %d\n"  ,
                               "irq_status" , state.irq_status);
        
        sh_css_dtrace(2, "\t\t%-32s: %d\n"  ,
                               "irq_enable" , state.irq_enable);

        sh_css_dtrace(2, "\t\t%-32s: %d\n"  ,
                               "irq_level_not_pulse", state.irq_level_not_pulse);
    }
#endif
return;
}

#if SP_DEBUG == SP_DEBUG_TRACE

void
sh_css_sp_debug_dump_mipi_fifo_high_water(void)
{
	const struct ia_css_fw_info *fw = &sh_css_sp_fw;
	unsigned int mfhw;
	unsigned int HIVE_ADDR_sp_output = fw->info.sp.output;
	unsigned offset = offsetof(struct sh_css_sp_output, debug.mipi_fifo_high_water);

	(void)HIVE_ADDR_sp_output; /* To get rid of warning in CRUN */

	load_sp_var_with_offset(sp_output, offset, &mfhw, sizeof(mfhw));

	sh_css_dtrace(SH_DBG_DEBUG, "Mipi fifo high water mark: %d, line %d\n",mfhw&0xFFFF, mfhw>>16);
}

#endif


/* this function is for debug use, it can make SP go to sleep
  state after each frame, then user can dump the stable SP dmem.
  this function can be called after sh_css_start()
  and before sh_css_init_buffer_queues() */
void sh_css_enable_sp_sleep_mode(enum ia_css_sp_sleep_mode mode)
{
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_sp_sleep_mode;

	fw = &sh_css_sp_fw;
	HIVE_ADDR_sp_sleep_mode = fw->info.sp.sleep_mode;

	(void)HIVE_ADDR_sp_sleep_mode; /* Suppres warnings in CRUN */

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(sp_sleep_mode),
		(uint32_t)mode);
}

void sh_css_wake_up_sp(void)
{
	/*hrt_ctl_start(SP);*/
	sp_ctrl_setbit(SP0_ID, SP_SC_REG, SP_START_BIT);
}

void sh_css_dump_isp_params(struct ia_css_stream *stream, unsigned int enable)
{
	const struct sh_css_isp_params *isp_params = ia_css_get_isp_params(stream);

	sh_css_dtrace(SH_DBG_DEBUG, "ISP PARAMETERS:\n");
	if ((enable & SH_CSS_DEBUG_DUMP_FPN)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Fixed Pattern Noise Reduction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"fpn_shift", isp_params->fpn_shift);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"fpn_enabled", isp_params->fpn_enabled);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_OB)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Optical Black:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"ob_blacklevel_gr", isp_params->ob_blacklevel_gr);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"ob_blacklevel_r", isp_params->ob_blacklevel_r);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"ob_blacklevel_b", isp_params->ob_blacklevel_b);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"ob_blacklevel_gb", isp_params->ob_blacklevel_gb);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"obarea_start_bq", isp_params->obarea_start_bq);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"obarea_length_bq", isp_params->obarea_length_bq);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"obarea_length_bq_inverse",
			isp_params->obarea_length_bq_inverse);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_SC)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Shading Correction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"sc_gain_shift", isp_params->sc_gain_shift);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_WB)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "White Balance:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"wb_gain_shift", isp_params->wb_gain_shift);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"wb_gain_gr", isp_params->wb_gain_gr);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"wb_gain_r", isp_params->wb_gain_r);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"wb_gain_b", isp_params->wb_gain_b);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"wb_gain_gb", isp_params->wb_gain_gb);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_DP)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Defect Pixel Correction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dp_threshold_single_w_2adj_on",
			isp_params->dp_threshold_single_when_2adjacent_on);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dp_threshold_2adj_w_2adj_on",
			isp_params->dp_threshold_2adjacent_when_2adjacent_on);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dp_threshold_single_w_2adj_off",
			isp_params->dp_threshold_single_when_2adjacent_off);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dp_threshold_2adj_w_2adj_off",
			isp_params->dp_threshold_2adjacent_when_2adjacent_off);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dp_gain", isp_params->dp_gain);
#if SH_CSS_ISP_SUPPORT_DPC_BEFORE_WB
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_rr_gr", isp_params->dpc_coef_rr_gr);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_rr_gb", isp_params->dpc_coef_rr_gb);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_bb_gb", isp_params->dpc_coef_bb_gb);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_bb_gr", isp_params->dpc_coef_bb_gr);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_gr_rr", isp_params->dpc_coef_gr_rr);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_gr_bb", isp_params->dpc_coef_gr_bb);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_gb_bb", isp_params->dpc_coef_gb_bb);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
			"dpc_coef_gb_rr", isp_params->dpc_coef_gb_rr);
#endif
	}
	if ((enable & SH_CSS_DEBUG_DUMP_BNR)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Bayer Noise Reduction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_gain_all", isp_params->bnr_gain_all);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_gain_dir", isp_params->bnr_gain_dir);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_threshold_low",
				isp_params->bnr_threshold_low);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_threshold_width_log2",
				isp_params->bnr_threshold_width_log2);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_threshold_width",
				isp_params->bnr_threshold_width);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"bnr_clip", isp_params->bnr_clip);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_S3A)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "S3A Support:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ae_y_coef_r", isp_params->ae_y_coef_r);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ae_y_coef_g", isp_params->ae_y_coef_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ae_y_coef_b", isp_params->ae_y_coef_b);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"awb_lg_high_raw", isp_params->awb_lg_high_raw);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"awb_lg_low", isp_params->awb_lg_low);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"awb_lg_high", isp_params->awb_lg_high);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[0]", isp_params->af_fir1[0]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[1]", isp_params->af_fir1[1]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[2]", isp_params->af_fir1[2]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[3]", isp_params->af_fir1[3]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[4]", isp_params->af_fir1[4]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[5]", isp_params->af_fir1[5]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir1[6]", isp_params->af_fir1[6]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[0]", isp_params->af_fir2[0]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[1]", isp_params->af_fir2[1]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[2]", isp_params->af_fir2[2]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[3]", isp_params->af_fir2[3]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[4]", isp_params->af_fir2[4]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[5]", isp_params->af_fir2[5]);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"af_fir2[6]", isp_params->af_fir2[6]);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_DE)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Demosaic:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"de_pixelnoise", isp_params->de_pixelnoise);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"de_c1_coring_threshold",
				isp_params->de_c1_coring_threshold);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"de_c2_coring_threshold",
				isp_params->de_c2_coring_threshold);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_YNR)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG,
			"Y Noise Reduction and Edge Enhancement:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynr_threshold", isp_params->ynr_threshold);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynr_gain_all", isp_params->ynr_gain_all);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynr_gain_dir", isp_params->ynr_gain_dir);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynryee_dirthreshold_s",
				isp_params->ynryee_dirthreshold_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynryee_dirthreshold_g",
				isp_params->ynryee_dirthreshold_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynryee_dirthreshold_width_log2",
				isp_params->ynryee_dirthreshold_width_log2);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynryee_dirthreshold_width",
				isp_params->ynryee_dirthreshold_width);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_detailgain",
				isp_params->yee_detailgain);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_coring_s",
				isp_params->yee_coring_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_coring_g",
				isp_params->yee_coring_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_scale_plus_s",
				isp_params->yee_scale_plus_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_scale_plus_g",
				isp_params->yee_scale_plus_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_scale_minus_s",
				isp_params->yee_scale_minus_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_scale_minus_g",
				isp_params->yee_scale_minus_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_clip_plus_s",
				isp_params->yee_clip_plus_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_clip_plus_g",
				isp_params->yee_clip_plus_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_clip_minus_s",
				isp_params->yee_clip_minus_s);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"yee_clip_minus_g",
				isp_params->yee_clip_minus_g);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ynryee_Yclip",
				isp_params->ynryee_Yclip);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_CSC)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		int	i;
		for (i=0;i<N_CSC_KERNEL_PARAM_SET;i++) {
			sh_css_dtrace(SH_DBG_DEBUG, "Color Space Conversion [%d]:\n",i);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m_shift",
				isp_params->csc_kernel_param[i].m_shift);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m00",
				isp_params->csc_kernel_param[i].m00);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m01",
				isp_params->csc_kernel_param[i].m01);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m02",
				isp_params->csc_kernel_param[i].m02);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m10",
				isp_params->csc_kernel_param[i].m10);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m11",
				isp_params->csc_kernel_param[i].m11);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m12",
				isp_params->csc_kernel_param[i].m12);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m20",
				isp_params->csc_kernel_param[i].m20);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m21",
				isp_params->csc_kernel_param[i].m21);
			sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"m22",
				isp_params->csc_kernel_param[i].m22);
		}
	}
	if ((enable & SH_CSS_DEBUG_DUMP_GC)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Gamma Correction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"gamma_gain_k1", isp_params->gamma_gain_k1);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"gamma_gain_k2", isp_params->gamma_gain_k2);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_TNR)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Temporal Noise Reduction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"tnr_coef", isp_params->tnr_coef);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"tnr_threshold_Y", isp_params->tnr_threshold_Y);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"tnr_threshold_C", isp_params->tnr_threshold_C);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_ANR)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Advance Noise Reduction:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"anr_threshold", isp_params->anr_threshold);
	}
	if ((enable & SH_CSS_DEBUG_DUMP_CE)
			|| (enable & SH_CSS_DEBUG_DUMP_ALL)) {
		sh_css_dtrace(SH_DBG_DEBUG, "Chroma Enhancement:\n");
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ce_uv_level_min", isp_params->ce_uv_level_min);
		sh_css_dtrace(SH_DBG_DEBUG, "\t%-32s = %d\n",
				"ce_uv_level_max", isp_params->ce_uv_level_max);
	}
}



/*

void sh_css_init_ddr_debug_queue(void)
{
	hrt_vaddress ddr_debug_queue_addr =
			mmgr_malloc(sizeof(debug_data_ddr_t));
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_debug_buffer_ddr_address;

	fw = &sh_css_sp_fw;
	HIVE_ADDR_debug_buffer_ddr_address =
			fw->info.sp.debug_buffer_ddr_address;

	(void)HIVE_ADDR_debug_buffer_ddr_address;

	debug_buffer_ddr_init(ddr_debug_queue_addr);

	sp_dmem_store_uint32(SP0_ID,
		(unsigned int)sp_address_of(debug_buffer_ddr_address),
		(uint32_t)(ddr_debug_queue_addr));
}

void sh_css_load_ddr_debug_queue(void)
{
	debug_synch_queue_ddr();
}

void sh_css_dump_ddr_debug_queue(void)
{
	int i;
	sh_css_load_ddr_debug_queue();
#ifdef __KERNEL__
	for (i = 0; i < DEBUG_BUF_SIZE; i++)
		printk(KERN_DEBUG, "ddr_debug_queue[%d] = 0x%x\n",
				i, debug_data_ptr->buf[i]);
#else
	for (i = 0; i < DEBUG_BUF_SIZE; i++)
		printf("ddr_debug_queue[%d] = 0x%x\n",
				i, debug_data_ptr->buf[i]);
#endif
}
*/

/**
 * @brief Initialize the debug mode.
 * Refer to "sh_css_debug.h" for more details.
 */
bool
sh_css_debug_mode_init(void)
{
	bool rc;
	rc = sh_css_sp_init_dma_sw_reg(0);
	return	rc;
}

/**
 * @brief Disable the DMA channel.
 * Refer to "sh_css_debug.h" for more details.
 */
bool
sh_css_debug_mode_disable_dma_channel(int dma_id,
		int channel_id,
		int request_type)
{
	bool rc;

	rc = sh_css_sp_set_dma_sw_reg(dma_id,
			channel_id,
			request_type,
			false);

	return rc;
}

/**
 * @brief Enable the DMA channel.
 * Refer to "sh_css_debug.h" for more details.
 */
bool
sh_css_debug_mode_enable_dma_channel(int dma_id,
		int channel_id,
		int request_type)
{
	bool rc;

	rc = sh_css_sp_set_dma_sw_reg(dma_id,
			channel_id,
			request_type,
			true);

	return rc;
}

static bool debug_pipe_graph_do_init = true;

static const char *format2str[] = {
	/*[IA_CSS_FRAME_FORMAT_NV11]          =*/ "NV11",
	/*[IA_CSS_FRAME_FORMAT_NV12]          =*/ "NV12",
	/*[IA_CSS_FRAME_FORMAT_NV16]          =*/ "NV16",
	/*[IA_CSS_FRAME_FORMAT_NV21]          =*/ "NV21",
	/*[IA_CSS_FRAME_FORMAT_NV61]          =*/ "NV61",
	/*[IA_CSS_FRAME_FORMAT_YV12]          =*/ "YV12",
	/*[IA_CSS_FRAME_FORMAT_YV16]          =*/ "YV16",
	/*[IA_CSS_FRAME_FORMAT_YUV420]        =*/ "YUV420",
	/*[IA_CSS_FRAME_FORMAT_YUV420_16]     =*/ "YUV420_16",
	/*[IA_CSS_FRAME_FORMAT_YUV422]        =*/ "YUV422",
	/*[IA_CSS_FRAME_FORMAT_YUV422_16]     =*/ "YUV422_16",
	/*[IA_CSS_FRAME_FORMAT_UYVY]          =*/ "UYVY",
	/*[IA_CSS_FRAME_FORMAT_YUYV]          =*/ "YUYV",
	/*[IA_CSS_FRAME_FORMAT_YUV444]        =*/ "YUV444",
	/*[IA_CSS_FRAME_FORMAT_YUV_LINE]      =*/ "YUV_LINE",
	/*[IA_CSS_FRAME_FORMAT_RAW]           =*/ "RAW",
	/*[IA_CSS_FRAME_FORMAT_RGB565]        =*/ "RGB565",
	/*[IA_CSS_FRAME_FORMAT_PLANAR_RGB888] =*/ "PLANAR_RGB888",
	/*[IA_CSS_FRAME_FORMAT_RGBA888]       =*/ "RGBA888",
	/*[IA_CSS_FRAME_FORMAT_QPLANE6]       =*/ "QPLANE6",
	/*[IA_CSS_FRAME_FORMAT_BINARY_8]      =*/ "BINARY_8"
};

#define DPG_START "sh_css_pipe_graph_dump_start "
#define DPG_END   " sh_css_pipe_graph_dump_end\n"

STORAGE_CLASS_INLINE void
dtrace_dot(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);

	sh_css_dtrace(SH_DBG_INFO, "%s", DPG_START);
	sh_css_vdtrace(SH_DBG_INFO, fmt, ap);
	sh_css_dtrace(SH_DBG_INFO, "%s", DPG_END);
#ifdef HRT_CSIM
	/* For CSIM we print double because HSS log can mess up this output
	 * As post processing, we remove incomplete lines and make lines uniq.
	 * */
	sh_css_dtrace(SH_DBG_INFO, "%s", DPG_START);
	sh_css_vdtrace(SH_DBG_INFO, fmt, ap);
	sh_css_dtrace(SH_DBG_INFO, "%s", DPG_END);
#endif
	va_end(ap);
}

void
sh_css_debug_pipe_graph_dump_prologue(void)
{

	dtrace_dot("digraph sh_css_pipe_graph {" );
	dtrace_dot("rankdir=LR;" );
}

void
sh_css_debug_pipe_graph_dump_epilogue(void)
{

	dtrace_dot("}");
	debug_pipe_graph_do_init = true;
}

void
sh_css_debug_pipe_graph_dump_stage(
	struct sh_css_pipeline_stage *stage,
	enum ia_css_pipe_id id)
{

	char const *blob_name = "<unknow name>";
	char const *bin_type = "<unknow type>";

	if (debug_pipe_graph_do_init) {
		sh_css_debug_pipe_graph_dump_prologue();
		debug_pipe_graph_do_init = false;
	}

	if (stage->binary) {
		bin_type= "binary";
		if (stage->binary_info->blob)
			blob_name = stage->binary_info->blob->name;
	} else if (stage->firmware){
		bin_type= "firmware";
		blob_name =
			(char const *)IA_CSS_EXT_ISP_PROG_NAME(stage->firmware);
	}

	dtrace_dot("node [shape = circle, fixedsize=true, width=2, "
		"label=\"%s\\n%s\\np:%d, s:%d\"]; \"%s_%d\"",
		bin_type, blob_name, id, stage->stage_num, blob_name, id);

	if (stage->args.cc_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.cc_frame),
			format2str[stage->args.cc_frame->info.format],
			stage->args.cc_frame->info.res.width,
			stage->args.cc_frame->info.padded_width,
			stage->args.cc_frame->info.res.height,
			stage->args.cc_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"0x%08lx\"->\"%s_%d\" "
			"[label = in_frame];",
			HOST_ADDRESS(stage->args.cc_frame), blob_name, id);


	} else if (stage->args.in_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.in_frame),
			format2str[stage->args.in_frame->info.format],
			stage->args.in_frame->info.res.width,
			stage->args.in_frame->info.padded_width,
			stage->args.in_frame->info.res.height,
			stage->args.in_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"0x%08lx\"->\"%s_%d\" "
			"[label = in_frame];",
			HOST_ADDRESS(stage->args.in_frame), blob_name, id);
	}

	if (stage->args.in_ref_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.in_ref_frame),
			format2str[stage->args.in_ref_frame->info.format],
			stage->args.in_ref_frame->info.res.width,
			stage->args.in_ref_frame->info.padded_width,
			stage->args.in_ref_frame->info.res.height,
			stage->args.in_ref_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"0x%08lx\"->\"%s_%d\" "
			"[label = in_ref_frame];",
			HOST_ADDRESS(stage->args.in_ref_frame), blob_name, id);
	}

	if (stage->args.in_tnr_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.in_tnr_frame),
			format2str[stage->args.in_tnr_frame->info.format],
			stage->args.in_tnr_frame->info.res.width,
			stage->args.in_tnr_frame->info.padded_width,
			stage->args.in_tnr_frame->info.res.height,
			stage->args.in_tnr_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"0x%08lx\"->\"%s_%d\" "
			"[label = in_tnr_frame];",
			HOST_ADDRESS(stage->args.in_tnr_frame), blob_name, id);
	}

	if (stage->args.out_ref_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.out_ref_frame),
			format2str[stage->args.out_ref_frame->info.format],
			stage->args.out_ref_frame->info.res.width,
			stage->args.out_ref_frame->info.padded_width,
			stage->args.out_ref_frame->info.res.height,
			stage->args.out_ref_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"%s_%d\"->\"0x%08lx\" "
			"[label = out_ref_frame];",
			blob_name, id, HOST_ADDRESS(stage->args.out_ref_frame));
	}

	if (stage->args.out_tnr_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.out_tnr_frame),
			format2str[stage->args.out_tnr_frame->info.format],
			stage->args.out_tnr_frame->info.res.width,
			stage->args.out_tnr_frame->info.padded_width,
			stage->args.out_tnr_frame->info.res.height,
			stage->args.out_tnr_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"%s_%d\"->\"0x%08lx\" "
			"[label = out_tnr_frame];",
			blob_name, id, HOST_ADDRESS(stage->args.out_tnr_frame));
	}

	if (stage->args.out_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.out_frame),
			format2str[stage->args.out_frame->info.format],
			stage->args.out_frame->info.res.width,
			stage->args.out_frame->info.padded_width,
			stage->args.out_frame->info.res.height,
			stage->args.out_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"%s_%d\"->\"0x%08lx\" "
			"[label = out_frame];",
			blob_name, id, HOST_ADDRESS(stage->args.out_frame));
	}

	if (stage->args.out_vf_frame) {
		dtrace_dot(
			"node [shape = box, "
			"fixedsize=true, width=2]; \"0x%08lx\" "
			"[label = \"%s\\n%d(%d) x %d x %d\"];",
			HOST_ADDRESS(stage->args.out_vf_frame),
			format2str[stage->args.out_vf_frame->info.format],
			stage->args.out_vf_frame->info.res.width,
			stage->args.out_vf_frame->info.padded_width,
			stage->args.out_vf_frame->info.res.height,
			stage->args.out_vf_frame->info.raw_bit_depth);
		dtrace_dot(
			"\"%s_%d\"->\"0x%08lx\" "
			"[label = out_vf_frame];",
			blob_name, id, HOST_ADDRESS(stage->args.out_vf_frame));
	}

}

void
sh_css_debug_pipe_graph_dump_sp_raw_copy(
	struct ia_css_frame *cc_frame)
{
	dtrace_dot(
		"node [shape = circle, "
		"fixedsize=true, width=2]; \"%s\"",
		"sp_raw_copy_1");

	dtrace_dot("node [shape = circle, fixedsize=true, width=2, "
		"label=\"%s\\n%s\\np:%d, s:%d\"]; \"%s_%d\"",
		"sp-binary", "sp_raw_copy", 1, 0, "sp_raw_copy", 1);


	dtrace_dot(
		"node [shape = box, "
		"fixedsize=true, width=2]; \"0x%08lx\" "
		"[label = \"%s\\n%d(%d) x %d x %d\"];",
		HOST_ADDRESS(cc_frame),
		format2str[cc_frame->info.format],
		cc_frame->info.res.width,
		cc_frame->info.padded_width,
		cc_frame->info.res.height,
		cc_frame->info.raw_bit_depth);


	dtrace_dot(
		"\"%s_%d\"->\"0x%08lx\" "
		"[label = cc_frame];",
		"sp_raw_copy", 1, HOST_ADDRESS(cc_frame));

}



#if defined(HRT_SCHED) || defined(SH_CSS_DEBUG_SPMEM_DUMP_SUPPORT)
#include "spmem_dump.c"
#endif

