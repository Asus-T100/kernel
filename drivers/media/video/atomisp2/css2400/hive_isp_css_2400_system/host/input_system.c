
#include "stddef.h"		/* NULL */

#include "input_system.h"

#include "assert_support.h"

#ifndef __INLINE_INPUT_SYSTEM__
#include "input_system_private.h"
#endif /* __INLINE_INPUT_SYSTEM__ */

STORAGE_CLASS_INLINE void capture_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	capture_unit_state_t			*state);

STORAGE_CLASS_INLINE void acquisition_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	acquisition_unit_state_t		*state);

STORAGE_CLASS_INLINE void ctrl_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	ctrl_unit_state_t				*state);

STORAGE_CLASS_INLINE void mipi_port_get_state(
	const rx_ID_t					ID,
	const mipi_port_ID_t			port_ID,
	mipi_port_state_t				*state);

STORAGE_CLASS_INLINE void rx_channel_get_state(
	const rx_ID_t					ID,
	const unsigned int				ch_id,
	rx_channel_state_t				*state);

void input_system_get_state(
	const input_system_ID_t			ID,
	input_system_state_t			*state)
{
	sub_system_ID_t	sub_id;

assert(ID < N_INPUT_SYSTEM_ID);
assert(state != NULL);

	state->str_multicastA_sel = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_MULTICAST_A_IDX);
	state->str_multicastB_sel = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_MULTICAST_B_IDX);
	state->str_multicastC_sel = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_MULTICAST_C_IDX);
	state->str_mux_sel = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_MUX_IDX);
	state->str_mon_status = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_STRMON_STAT_IDX);
	state->str_mon_irq_cond = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_STRMON_COND_IDX);
	state->str_mon_irq_en = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_STRMON_IRQ_EN_IDX);
	state->isys_srst = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_SRST_IDX);
	state->isys_slv_reg_srst = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_SLV_REG_SRST_IDX);
	state->str_deint_portA_cnt = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_REG_PORT_A_IDX);
	state->str_deint_portB_cnt = input_system_sub_system_reg_load(ID,
		GPREGS_UNIT0_ID,
		HIVE_ISYS_GPREG_REG_PORT_B_IDX);

	for (sub_id = CAPTURE_UNIT0_ID; sub_id < CAPTURE_UNIT0_ID + N_CAPTURE_UNIT_ID; sub_id++) {
		capture_unit_get_state(ID, sub_id,
			&(state->capture_unit[sub_id - CAPTURE_UNIT0_ID]));
	}
	for (sub_id = ACQUISITION_UNIT0_ID; sub_id < ACQUISITION_UNIT0_ID + N_ACQUISITION_UNIT_ID; sub_id++) {
		acquisition_unit_get_state(ID, sub_id,
			&(state->acquisition_unit[sub_id - ACQUISITION_UNIT0_ID]));
	}
	for (sub_id = CTRL_UNIT0_ID; sub_id < CTRL_UNIT0_ID + N_CTRL_UNIT_ID; sub_id++) {
		ctrl_unit_get_state(ID, sub_id,
			&(state->ctrl_unit_state[sub_id - CTRL_UNIT0_ID]));
	}

return;
}

void receiver_get_state(
	const rx_ID_t				ID,
	receiver_state_t			*state)
{
	mipi_port_ID_t	port_id;
	unsigned int	ch_id;

assert(ID < N_RX_ID);
assert(state != NULL);

	state->fs_to_ls_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_FS_TO_LS_DELAY_REG_IDX);
	state->ls_to_data_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_LS_TO_DATA_DELAY_REG_IDX);
	state->data_to_le_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_DATA_TO_LE_DELAY_REG_IDX);
	state->le_to_fe_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_LE_TO_FE_DELAY_REG_IDX);
	state->fe_to_fs_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_FE_TO_FS_DELAY_REG_IDX);
	state->le_to_fs_delay = (uint8_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_LE_TO_LS_DELAY_REG_IDX);
	state->is_two_ppc = (bool)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_TWO_PIXEL_EN_REG_IDX);
	state->backend_rst = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BACKEND_RST_REG_IDX);
	state->raw18 = (uint16_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_RAW18_REG_IDX);
	state->force_raw8 = (bool)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_FORCE_RAW8_REG_IDX);
	state->raw16 = (uint16_t)receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_RAW16_REG_IDX);

	for (port_id = (mipi_port_ID_t)0; port_id < N_MIPI_PORT_ID; port_id++) {
		mipi_port_get_state(ID, port_id,
			&(state->mipi_port_state[port_id]));
	}
	for (ch_id = (unsigned int)0; ch_id < N_RX_CHANNEL_ID; ch_id++) {
		rx_channel_get_state(ID, ch_id,
			&(state->rx_channel_state[ch_id]));
	}

	state->be_gsp_acc_ovl = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_GSP_ACC_OVL_REG_IDX);
	state->be_srst = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_SRST_REG_IDX);
	state->be_is_two_ppc = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_TWO_PPC_REG_IDX);
	state->be_comp_format0 = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_COMP_FORMAT_REG0_IDX);
	state->be_comp_format1 = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_COMP_FORMAT_REG1_IDX);
	state->be_comp_format2 = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_COMP_FORMAT_REG2_IDX);
	state->be_comp_format3 = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_COMP_FORMAT_REG3_IDX);
	state->be_sel = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_SEL_REG_IDX);
	state->be_raw16_config = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_RAW16_CONFIG_REG_IDX);
	state->be_raw18_config = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_RAW18_CONFIG_REG_IDX);
	state->be_force_raw8 = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_FORCE_RAW8_REG_IDX);
	state->be_irq_status = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_IRQ_STATUS_REG_IDX);
	state->be_irq_clear = receiver_reg_load(ID,
		_HRT_CSS_RECEIVER_BE_IRQ_CLEAR_REG_IDX);

return;
}

bool is_mipi_format_yuv420(
	const mipi_format_t			mipi_format)
{
	bool	is_yuv420 = (
		(mipi_format == MIPI_FORMAT_YUV420_8) ||
		(mipi_format == MIPI_FORMAT_YUV420_10) ||
		(mipi_format == MIPI_FORMAT_YUV420_8_SHIFT) ||
		(mipi_format == MIPI_FORMAT_YUV420_10_SHIFT));
/* MIPI_FORMAT_YUV420_8_LEGACY is not YUV420 */

return is_yuv420;
}

void receiver_set_compression(
	const rx_ID_t				ID,
	const unsigned int			cfg_ID,
	const mipi_compressor_t		comp,
	const mipi_predictor_t		pred)
{
	const unsigned int	field_id = cfg_ID % N_MIPI_FORMAT_CUSTOM;
	const unsigned int	ch_id = cfg_ID / N_MIPI_FORMAT_CUSTOM;
	hrt_data			val;
	hrt_address			addr;
	hrt_data			reg;

assert(ID < N_RX_ID);
assert(cfg_ID < N_MIPI_COMPRESSOR_CONTEXT);
assert(field_id < N_MIPI_FORMAT_CUSTOM);
assert(ch_id < N_RX_CHANNEL_ID);
assert(comp < N_MIPI_COMPRESSOR_METHODS);
assert(pred < N_MIPI_PREDICTOR_TYPES);

	val = (((uint8_t)pred) << 3) | comp;

	switch (ch_id) {
	case 0: addr = ((field_id<6)?_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG0_IDX:_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG1_IDX);
		break;
	case 1: addr = ((field_id<6)?_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG0_IDX:_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG1_IDX);
		break;
	case 2: addr = ((field_id<6)?_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG0_IDX:_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG1_IDX);
		break;
	case 3: addr = ((field_id<6)?_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG0_IDX:_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG1_IDX);
		break;
	}

	reg = ((field_id < 6)?(val << (field_id * 5)):(val << ((field_id - 6) * 5)));
	receiver_reg_store(ID, addr, reg);

return;
}

void receiver_port_enable(
	const rx_ID_t				ID,
	const mipi_port_ID_t		port_ID,
	const bool					cnd)
{
	hrt_data	reg = receiver_port_reg_load(ID, port_ID,
		_HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX);

	if (cnd) {
		reg |= 0x01;
	} else {
		reg &= ~0x01;
	}

	receiver_port_reg_store(ID, port_ID,
		_HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX, reg);
return;
}

bool is_receiver_port_enabled(
	const rx_ID_t				ID,
	const mipi_port_ID_t		port_ID)
{
	hrt_data	reg = receiver_port_reg_load(ID, port_ID,
		_HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX);
return ((reg & 0x01) != 0);
}

void receiver_irq_enable(
	const rx_ID_t				ID,
	const mipi_port_ID_t		port_ID,
	const rx_irq_info_t			irq_info)
{
	receiver_port_reg_store(ID,
		port_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX, irq_info);
return;
}

rx_irq_info_t receiver_get_irq_info(
	const rx_ID_t				ID,
	const mipi_port_ID_t		port_ID)
{
return receiver_port_reg_load(ID,
	port_ID, _HRT_CSS_RECEIVER_IRQ_STATUS_REG_IDX);
}

void receiver_irq_clear(
	const rx_ID_t				ID,
	const mipi_port_ID_t		port_ID,
	const rx_irq_info_t			irq_info)
{
	receiver_port_reg_store(ID,
		port_ID, _HRT_CSS_RECEIVER_IRQ_STATUS_REG_IDX, irq_info);
return;
}

STORAGE_CLASS_INLINE void capture_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	capture_unit_state_t			*state)
{
assert(state != NULL);
assert(/*(sub_id >= CAPTURE_UNIT0_ID) &&*/ (sub_id <= CAPTURE_UNIT2_ID));

	state->StartMode = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_START_MODE_REG_ID);
	state->Start_Addr = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_START_ADDR_REG_ID);
	state->Mem_Region_Size = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_MEM_REGION_SIZE_REG_ID);
	state->Num_Mem_Regions = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_NUM_MEM_REGIONS_REG_ID);
	state->Init = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_INIT_REG_ID);
	state->Start = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_START_REG_ID);
	state->Stop = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_STOP_REG_ID);
	state->Packet_Length = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_PACKET_LENGTH_REG_ID);
	state->Received_Length = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_RECEIVED_LENGTH_REG_ID);
	state->Received_Short_Packets = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_RECEIVED_SHORT_PACKETS_REG_ID);
	state->Received_Long_Packets = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_RECEIVED_LONG_PACKETS_REG_ID);
	state->Last_Command = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_LAST_COMMAND_REG_ID);
	state->Next_Command = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_NEXT_COMMAND_REG_ID);
	state->Last_Acknowledge = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_LAST_ACKNOWLEDGE_REG_ID);
	state->Next_Acknowledge = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_NEXT_ACKNOWLEDGE_REG_ID);
	state->FSM_State_Info = input_system_sub_system_reg_load(ID,
		sub_id,
		CAPT_FSM_STATE_INFO_REG_ID);

return;
}

STORAGE_CLASS_INLINE void acquisition_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	acquisition_unit_state_t		*state)
{
assert(state != NULL);
assert(sub_id == ACQUISITION_UNIT0_ID);

	state->Start_Addr = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_START_ADDR_REG_ID);
	state->Mem_Region_Size = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_MEM_REGION_SIZE_REG_ID);
	state->Num_Mem_Regions = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_NUM_MEM_REGIONS_REG_ID);
	state->Init = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_INIT_REG_ID);
	state->Received_Short_Packets = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_RECEIVED_SHORT_PACKETS_REG_ID);
	state->Received_Long_Packets = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_RECEIVED_LONG_PACKETS_REG_ID);
	state->Last_Command = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_LAST_COMMAND_REG_ID);
	state->Next_Command = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_NEXT_COMMAND_REG_ID);
	state->Last_Acknowledge = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_LAST_ACKNOWLEDGE_REG_ID);
	state->Next_Acknowledge = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_NEXT_ACKNOWLEDGE_REG_ID);
	state->FSM_State_Info = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_FSM_STATE_INFO_REG_ID);
	state->Int_Cntr_Info = input_system_sub_system_reg_load(ID,
		sub_id,
		ACQ_INT_CNTR_INFO_REG_ID);

return;
}

STORAGE_CLASS_INLINE void ctrl_unit_get_state(
	const input_system_ID_t			ID,
	const sub_system_ID_t			sub_id,
	ctrl_unit_state_t				*state)
{
assert(state != NULL);
assert(sub_id == CTRL_UNIT0_ID);

	state->captA_start_addr = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_START_ADDR_A_REG_ID);
	state->captB_start_addr = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_START_ADDR_B_REG_ID);
	state->captC_start_addr = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_START_ADDR_C_REG_ID);
	state->captB_mem_region_size = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_MEM_REGION_SIZE_A_REG_ID);
	state->captA_mem_region_size = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_MEM_REGION_SIZE_B_REG_ID);
	state->captC_mem_region_size = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_MEM_REGION_SIZE_C_REG_ID);
	state->captA_num_mem_regions = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_NUM_MEM_REGIONS_A_REG_ID);
	state->captB_num_mem_regions = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_NUM_MEM_REGIONS_B_REG_ID);
	state->captC_num_mem_regions = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_NUM_MEM_REGIONS_C_REG_ID);
	state->acq_start_addr = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_ACQ_START_ADDR_REG_ID);
	state->acq_mem_region_size = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_ACQ_MEM_REGION_SIZE_REG_ID);
	state->acq_num_mem_regions = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_ACQ_NUM_MEM_REGIONS_REG_ID);
	state->ctrl_init = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_INIT_REG_ID);
	state->last_cmd = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_LAST_COMMAND_REG_ID);
	state->next_cmd = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_NEXT_COMMAND_REG_ID);
	state->last_ack = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_LAST_ACKNOWLEDGE_REG_ID);
	state->next_ack = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_NEXT_ACKNOWLEDGE_REG_ID);
	state->top_fsm_state = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_FSM_STATE_INFO_REG_ID);
	state->captA_fsm_state = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_A_FSM_STATE_INFO_REG_ID);
	state->captB_fsm_state = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_B_FSM_STATE_INFO_REG_ID);
	state->captC_fsm_state = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_C_FSM_STATE_INFO_REG_ID);
	state->acq_fsm_state = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_ACQ_FSM_STATE_INFO_REG_ID);
	state->capt_reserve_one_mem_region = input_system_sub_system_reg_load(ID,
		sub_id,
		ISYS_CTRL_CAPT_RESERVE_ONE_MEM_REGION_REG_ID);

return;
}

STORAGE_CLASS_INLINE void mipi_port_get_state(
	const rx_ID_t					ID,
	const mipi_port_ID_t			port_ID,
	mipi_port_state_t				*state)
{
	int	i;

assert(ID < N_RX_ID);
assert(port_ID < N_MIPI_PORT_ID);
assert(state != NULL);

	state->device_ready = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX);
	state->irq_status = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX);
	state->irq_enable = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_IRQ_STATUS_REG_IDX);
	state->timeout_count = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_TIMEOUT_COUNT_REG_IDX);
	state->init_count = (uint16_t)receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_INIT_COUNT_REG_IDX);
	state->raw16_18 = (uint16_t)receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_RAW16_18_DATAID_REG_IDX);
	state->sync_count = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_SYNC_COUNT_REG_IDX);
	state->rx_count = receiver_port_reg_load(ID,
		port_ID, _HRT_CSS_RECEIVER_RX_COUNT_REG_IDX);

	for (i = 0; i < MIPI_4LANE_CFG ; i++) {
		state->lane_sync_count[i] = (uint8_t)((state->sync_count)>>(i*8));
		state->lane_rx_count[i] = (uint8_t)((state->rx_count)>>(i*8));
	}

return;
}

STORAGE_CLASS_INLINE void rx_channel_get_state(
	const rx_ID_t					ID,
	const unsigned int				ch_id,
	rx_channel_state_t				*state)
{
	int	i;

assert(ID < N_RX_ID);
assert(ch_id < N_RX_CHANNEL_ID);
assert(state != NULL);

	switch (ch_id) {
		case 0:
			state->comp_scheme0 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG0_IDX);
			state->comp_scheme1 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG1_IDX);
	break;
		case 1:
			state->comp_scheme0 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG0_IDX);
			state->comp_scheme1 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG1_IDX);
	break;
		case 2:
			state->comp_scheme0 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG0_IDX);
			state->comp_scheme1 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG1_IDX);
	break;
		case 3:
			state->comp_scheme0 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG0_IDX);
			state->comp_scheme1 = receiver_reg_load(ID,
				_HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC3_REG1_IDX);
	break;
	}

/* See Table 7.1.17,..., 7.1.24 */
	for (i = 0; i < 6; i++) {
		uint8_t	val = (uint8_t)((state->comp_scheme0)>>(i*5)) & 0x1f;
		state->comp[i] = (mipi_compressor_t)(val & 0x07);
		state->pred[i] = (mipi_predictor_t)((val & 0x18) >> 3);
	}
	for (i = 6; i < N_MIPI_FORMAT_CUSTOM; i++) {
		uint8_t	val = (uint8_t)((state->comp_scheme0)>>((i-6)*5)) & 0x1f;
		state->comp[i] = (mipi_compressor_t)(val & 0x07);
		state->pred[i] = (mipi_predictor_t)((val & 0x18) >> 3);
	}

return;
}
