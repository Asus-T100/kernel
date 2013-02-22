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
#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#include <linux/bitops.h>

#include "sh_css_hrt.h"
#include "sh_css_internal.h"
#define HRT_NO_BLOB_sp
#include "sh_css.h"
#include "sh_css_hw.h"
#include "sh_css_debug.h"

#define HBLANK_CYCLES (187)
#define MARKER_CYCLES (6)
#if defined(SYSTEM_hive_isp_css_2400_system)
#include <dma_v2.h>
#include <inputformatter.h>
#include "hrt_2400/sp.map.h"
#else
#include "hrt/sp.map.h"
#endif

/* The data type is used to send special cases:
 * yuv420: odd lines (1, 3 etc) are twice as wide as even
 *         lines (0, 2, 4 etc).
 * rgb: for two pixels per clock, the R and B values are sent
 *      to output_0 while only G is sent to output_1. This means
 *      that output_1 only gets half the number of values of output_0.
 *      WARNING: This type should also be used for Legacy YUV420.
 * regular: used for all other data types (RAW, YUV422, etc)
 */
#if defined(SYSTEM_hive_isp_css_2400_system)
enum sh_css_mipi_data_type {
	sh_css_mipi_data_type_regular,
	sh_css_mipi_data_type_yuv420,
	sh_css_mipi_data_type_rgb,
};
#else  /* defined(SYSTEM_hive_isp_css_2400_system) */
enum sh_css_mipi_data_type {
	sh_css_mipi_data_type_regular,
	sh_css_mipi_data_type_yuv420,
	sh_css_mipi_data_type_yuv420_legacy,
	sh_css_mipi_data_type_rgb,
};
#endif /* !defined(SYSTEM_hive_isp_css_2400_system) */

/* these are unsigned long pointers such that we can simply
 * add the register index to get the right address.
 */
static unsigned long *mmu_base_address         = MMU_BASE,
		     *gdc_lut_base_address     = GDC_LUT_BASE,
		     *dma_base_address         = DMA_BASE,
		     *if_prim_a_base_address   = IF_PRIM_A_BASE,
		     *if_prim_b_base_address   = IF_PRIM_B_BASE,
		     *sp_sc_base_address       = SP_SC_BASE,
		     *isp_sc_base_address      = ISP_SC_BASE,
		     *str_monitor_base_address = STR_MON_BASE,
		     *irq_ctrl_base_address    = IRQ_CTRL_BASE,
		     *gp_fifo_base_address     = GP_FIFO_BASE;

static unsigned int curr_ch_id, curr_fmt_type;


struct streamining_to_mipi_instance {
	unsigned int			ch_id;
	enum sh_css_input_format	input_format;
	bool				two_ppc;
	bool				streaming;
	unsigned int			hblank_cycles;
	unsigned int			marker_cycles;
	unsigned int			fmt_type;
	enum sh_css_mipi_data_type	type;
};

/*
 * Maintain a basic streaming to Mipi administration with ch_id as index
 * ch_id maps on the "Mipi virtual channel ID" and can have value 0..3
 */
#define NR_OF_S2M_CHANNELS	(4)
static struct streamining_to_mipi_instance s2m_inst_admin[NR_OF_S2M_CHANNELS];

void
sh_css_sp_ctrl_store(unsigned int reg, unsigned int value)
{
	unsigned int addr = (unsigned int)(sp_sc_base_address + reg);
	hrt_master_port_store_32(addr, value);
}

static unsigned long
sh_css_sp_ctrl_load(unsigned int reg)
{
	unsigned int addr = (unsigned int)(sp_sc_base_address + reg);
	return hrt_master_port_uload_32(addr);
}

static unsigned long
sh_css_isp_ctrl_load(unsigned int reg)
{
	unsigned int addr = (unsigned int)(isp_sc_base_address + reg);
	return hrt_master_port_uload_32(addr);
}

static unsigned int
isp_ctrl_get_bit(unsigned int reg, unsigned int bit)
{
	unsigned long val;
	val = sh_css_isp_ctrl_load(reg);
	return (val & (1U << bit)) != 0;
}

void
sh_css_sp_ctrl_set_bits(unsigned int reg, unsigned long bits)
{
	unsigned long val;
	val = sh_css_sp_ctrl_load(reg);
	val |= bits;
	sh_css_sp_ctrl_store(reg, val);
}

static unsigned int
sp_ctrl_get_bit(unsigned int reg, unsigned int bit)
{
	unsigned long val;
	val = sh_css_sp_ctrl_load(reg);
	return (val & (1U << bit)) != 0;
}

static void
sh_css_sp_ctrl_reset_bits(unsigned int reg, unsigned long bits)
{
	unsigned long val;
	val = sh_css_sp_ctrl_load(reg);
	val &= ~bits;
	sh_css_sp_ctrl_store(reg, val);
}

void
sh_css_hrt_irq_enable_sp(bool enable)
{
	if (enable)
		sh_css_sp_ctrl_set_bits(SP_IRQ_READY_REG,
					1UL<<SP_IRQ_READY_BIT);
	else
		sh_css_sp_ctrl_reset_bits(SP_IRQ_READY_REG,
					  1UL<<SP_IRQ_READY_BIT);
}

void
sh_css_hrt_irq_clear_sp(void)
{
	sh_css_sp_ctrl_set_bits(SP_IRQ_CLEAR_REG, 1UL<<SP_IRQ_CLEAR_BIT);
}

static int
sh_css_cell_is_ready(unsigned long *status_reg, unsigned int ready_bit)
{
	unsigned long ready_reg;
	ready_reg = hrt_master_port_uload_32((unsigned long)status_reg);
	return (ready_reg & (1U << ready_bit)) != 0;
}

static enum sh_css_err
sh_css_cell_wait(unsigned long *status_reg, unsigned int ready_bit)
{
	while (!sh_css_cell_is_ready(status_reg, ready_bit))
		hrt_sleep();
	return sh_css_success;
}

enum sh_css_err
sh_css_hrt_sp_wait(void)
{
	return sh_css_cell_wait(sp_sc_base_address + SP_SC_REG, SP_IDLE_BIT);
}

bool
sh_css_hrt_sp_is_idle(void)
{
	return sh_css_cell_is_ready(sp_sc_base_address + SP_SC_REG,
				    SP_IDLE_BIT);
}

bool
sh_css_hrt_sp_is_stalling(void)
{
	return sh_css_cell_is_ready(sp_sc_base_address + SP_SC_REG,
				    SP_STALLING_BIT);
}

unsigned int
sh_css_hrt_sp_current_pc(void)
{
	return sh_css_sp_ctrl_load(SP_PC_REG);
}

unsigned int
sh_css_hrt_sp_current_msink(void)
{
	return sh_css_sp_ctrl_load(SP_CTRL_SINK_REG);
}

void
sh_css_hrt_sp_start_histogram(void)
{
	sh_css_sp_start_function(sp_gen_histogram);
}

void
sh_css_hrt_sp_start_copy_binary_data(void)
{
	sh_css_sp_start_function(sp_bin_copy);
}

void
sh_css_hrt_sp_start_copy_raw_data(void)
{
	sh_css_sp_start_function(sp_raw_copy);
}

void
sh_css_hrt_sp_start_isp(void)
{
	sh_css_sp_start_function(sp_start_isp);
}

bool
sh_css_hrt_isp_is_idle(void)
{
	return sh_css_cell_is_ready(isp_sc_base_address + ISP_SC_REG,
				    ISP_IDLE_BIT);
}

bool
sh_css_hrt_isp_is_stalling(void)
{
	return sh_css_cell_is_ready(isp_sc_base_address + ISP_SC_REG,
				    ISP_STALLING_BIT);
}

unsigned int
sh_css_hrt_isp_current_pc(void)
{
	return sh_css_isp_ctrl_load(ISP_PC_REG);
}

unsigned int
sh_css_hrt_isp_current_msink(void)
{
	return sh_css_isp_ctrl_load(ISP_CTRL_SINK_REG);
}

unsigned int
sh_css_hrt_isp_current_sc(void)
{
	return sh_css_isp_ctrl_load(ISP_SC_REG);
}

void
sh_css_isp_ctrl_store(unsigned int reg, unsigned int value)
{
	unsigned int addr = (unsigned int)(isp_sc_base_address + reg);
	hrt_master_port_store_32(addr, value);
}

void
sh_css_hrt_sp_get_state(struct sh_css_cell_state *state,
			struct sh_css_sp_stall_state *stall_state)
{
	unsigned int sc;

	sc = sh_css_sp_ctrl_load(SP_SC_REG);
	state->pc = sh_css_sp_ctrl_load(SP_PC_REG);
	state->status_register = sc;
	state->is_broken   = (sc & (1U << SP_BROKEN_BIT)) != 0;
	state->is_idle     = (sc & (1U << SP_IDLE_BIT)) != 0;
	state->is_sleeping = (sc & (1U << SP_SLEEPING_BIT)) != 0;
	state->is_stalling = (sc & (1U << SP_STALLING_BIT)) != 0;
	stall_state->fifo0 =
		!sp_ctrl_get_bit(SP_FIFO0_SINK_REG, SP_FIFO0_SINK_BIT);
	stall_state->fifo1 =
		!sp_ctrl_get_bit(SP_FIFO1_SINK_REG, SP_FIFO1_SINK_BIT);
	stall_state->fifo2 =
		!sp_ctrl_get_bit(SP_FIFO2_SINK_REG, SP_FIFO2_SINK_BIT);
	stall_state->fifo3 =
		!sp_ctrl_get_bit(SP_FIFO3_SINK_REG, SP_FIFO3_SINK_BIT);
	stall_state->fifo4 =
		!sp_ctrl_get_bit(SP_FIFO4_SINK_REG, SP_FIFO4_SINK_BIT);
	stall_state->fifo5 =
		!sp_ctrl_get_bit(SP_FIFO5_SINK_REG, SP_FIFO5_SINK_BIT);
	stall_state->fifo6 =
		!sp_ctrl_get_bit(SP_FIFO6_SINK_REG, SP_FIFO6_SINK_BIT);
	stall_state->fifo7 =
		!sp_ctrl_get_bit(SP_FIFO7_SINK_REG, SP_FIFO7_SINK_BIT);
#if defined(SYSTEM_hive_isp_css_2400_system)
	stall_state->fifo8 =
		sp_ctrl_get_bit(SP_FIFO8_SINK_BIT, SP_FIFO8_SINK_REG);
	stall_state->fifo9 =
		sp_ctrl_get_bit(SP_FIFO9_SINK_BIT, SP_FIFO9_SINK_REG);
	stall_state->fifoa =
		sp_ctrl_get_bit(SP_FIFOA_SINK_BIT, SP_FIFOA_SINK_REG);
#endif
	stall_state->dmem =
		!sp_ctrl_get_bit(SP_DMEM_SINK_REG, SP_DMEM_SINK_BIT);
	stall_state->control_master =
		!sp_ctrl_get_bit(SP_CTRL_MT_SINK_REG, SP_CTRL_MT_SINK_BIT);
	stall_state->icache_master =
		!sp_ctrl_get_bit(SP_ICACHE_MT_SINK_REG, SP_ICACHE_MT_SINK_BIT);
}

void
sh_css_hrt_isp_get_state(struct sh_css_cell_state *state,
			 struct sh_css_isp_stall_state *stall_state)
{
	unsigned int sc;
	sc = sh_css_isp_ctrl_load(ISP_SC_REG);
	state->pc = sh_css_isp_ctrl_load(ISP_PC_REG);
	state->status_register = sc;
	state->is_broken = isp_ctrl_get_bit(ISP_SC_REG, ISP_BROKEN_BIT);
	state->is_idle = isp_ctrl_get_bit(ISP_SC_REG, ISP_IDLE_BIT);
	state->is_sleeping = isp_ctrl_get_bit(ISP_SC_REG, ISP_SLEEPING_BIT);
	state->is_stalling = isp_ctrl_get_bit(ISP_SC_REG, ISP_STALLING_BIT);
	stall_state->stat_ctrl =
		!isp_ctrl_get_bit(ISP_CTRL_SINK_REG  , ISP_CTRL_SINK_BIT);
#if defined(SYSTEM_hive_isp_css_2400_system)
	stall_state->pmem =
		isp_ctrl_get_bit(ISP_PMEM_SINK_BIT, ISP_PMEM_SINK_REG);
#endif
	stall_state->dmem =
		!isp_ctrl_get_bit(ISP_DMEM_SINK_REG  , ISP_DMEM_SINK_BIT);
	stall_state->vmem =
		!isp_ctrl_get_bit(ISP_VMEM_SINK_REG  , ISP_VMEM_SINK_BIT);
	stall_state->fifo0 =
		!isp_ctrl_get_bit(ISP_FIFO0_SINK_REG , ISP_FIFO0_SINK_BIT);
	stall_state->fifo1 =
		!isp_ctrl_get_bit(ISP_FIFO1_SINK_REG , ISP_FIFO1_SINK_BIT);
	stall_state->fifo2 =
		!isp_ctrl_get_bit(ISP_FIFO2_SINK_REG , ISP_FIFO2_SINK_BIT);
	stall_state->fifo3 =
		!isp_ctrl_get_bit(ISP_FIFO3_SINK_REG , ISP_FIFO3_SINK_BIT);
	stall_state->fifo4 =
		!isp_ctrl_get_bit(ISP_FIFO4_SINK_REG , ISP_FIFO4_SINK_BIT);
	stall_state->fifo5 =
		!isp_ctrl_get_bit(ISP_FIFO5_SINK_REG , ISP_FIFO5_SINK_BIT);
#if defined(SYSTEM_hive_isp_css_2400_system)
	stall_state->fifo6 =
		isp_ctrl_get_bit(ISP_FIFO6_SINK_BIT, ISP_FIFO6_SINK_REG);
#endif
	stall_state->vamem1 =
		!isp_ctrl_get_bit(ISP_VAMEM1_SINK_REG, ISP_VAMEM1_SINK_BIT);
	stall_state->vamem2 =
		!isp_ctrl_get_bit(ISP_VAMEM2_SINK_REG, ISP_VAMEM2_SINK_BIT);
#if defined(SYSTEM_hive_isp_css_2400_system)
	stall_state->vamem3 =
		isp_ctrl_get_bit(ISP_VAMEM3_SINK_BIT, ISP_VAMEM3_SINK_REG);
	stall_state->hmem =
		isp_ctrl_get_bit(ISP_HMEM_SINK_BIT, ISP_HMEM_SINK_REG);
#endif
}

static inline unsigned long
_sh_css_dma_get_register(unsigned int addr)
{
	unsigned char *dma_base = (unsigned char *)dma_base_address;
	unsigned int bus_addr = (unsigned int)(dma_base + addr);
	return hrt_master_port_load_32(bus_addr);
}

/* Command FSM state */
static inline unsigned
sh_css_dma_get_command_fsm_state(void)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_register(_hrt_dma_v2_sel_comp(0));
#else
	return _sh_css_dma_get_register(_hrt_dma_v1_sel_comp(0));
#endif
}

/* Get channel parameters */
static inline unsigned
_sh_css_dma_get_channel_parameter(int ch, int param)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_register(hrt_dma_v2_channel_parameter_register_address(ch, param));
#else
	return _sh_css_dma_get_register(hrt_dma_v1_channel_parameter_register_address(ch, param));
#endif
}

static inline unsigned
sh_css_dma_get_channel_connection(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _hrt_dma_v2_get_connection(_sh_css_dma_get_channel_parameter(channel, _DMA_V2_PACKING_SETUP_PARAM));
#else
	return _hrt_dma_v1_get_connection(_sh_css_dma_get_channel_parameter(channel, 0));
#endif
}

static inline unsigned
sh_css_dma_get_channel_extension(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _hrt_dma_v2_get_extension(_sh_css_dma_get_channel_parameter(channel, _DMA_V2_PACKING_SETUP_PARAM));
#else
	return _hrt_dma_v1_get_extension(_sh_css_dma_get_channel_parameter(channel, 0));
#endif
}

static inline unsigned
sh_css_dma_get_channel_element_order(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
(void)channel;
	return 0;
#else
	return _hrt_dma_v1_get_element_order(_sh_css_dma_get_channel_parameter(channel, 0));
#endif
}

static inline unsigned
sh_css_dma_get_channel_stride_A(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_channel_parameter(channel, _DMA_V2_STRIDE_A_PARAM);
#else
	return _sh_css_dma_get_channel_parameter(channel, 1);
#endif
}

static inline unsigned
sh_css_dma_get_channel_elements_A(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _hrt_dma_v2_get_elements(_sh_css_dma_get_channel_parameter(channel, _DMA_V2_ELEM_CROPPING_A_PARAM));
#else
	return _hrt_dma_v1_get_elements(_sh_css_dma_get_channel_parameter(channel, 2));
#endif
}

static inline unsigned
sh_css_dma_get_channel_cropping_A(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _hrt_dma_v2_get_cropping(
			_sh_css_dma_get_channel_parameter(channel, _DMA_V2_ELEM_CROPPING_A_PARAM));
#else
	return _hrt_dma_v1_get_cropping(
			_sh_css_dma_get_channel_parameter(channel, 2));
#endif
}

static inline unsigned
sh_css_dma_get_channel_width_A(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_channel_parameter(channel, _DMA_V2_WIDTH_A_PARAM);
#else
	return _sh_css_dma_get_channel_parameter(channel, 3);
#endif
}

static inline unsigned
sh_css_dma_get_channel_stride_B(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_channel_parameter(channel, _DMA_V2_STRIDE_B_PARAM);
#else
	return _sh_css_dma_get_channel_parameter(channel, 4);
#endif
}

static inline unsigned
sh_css_dma_get_channel_elements_B(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return	_hrt_dma_v2_get_elements(_sh_css_dma_get_channel_parameter(channel, _DMA_V2_ELEM_CROPPING_B_PARAM));
#else
	return	_hrt_dma_v1_get_elements(_sh_css_dma_get_channel_parameter(channel, 5));
#endif
}

static inline unsigned
sh_css_dma_get_channel_cropping_B(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _hrt_dma_v2_get_cropping(_sh_css_dma_get_channel_parameter(channel, _DMA_V2_ELEM_CROPPING_B_PARAM));
#else
	return _hrt_dma_v1_get_cropping(_sh_css_dma_get_channel_parameter(channel, 5));
#endif
}

static inline unsigned
sh_css_dma_get_channel_width_B(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_channel_parameter(channel, _DMA_V2_WIDTH_B_PARAM);
#else
	return _sh_css_dma_get_channel_parameter(channel, 6);
#endif
}

static inline unsigned
sh_css_dma_get_channel_height(int channel)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_channel_parameter(channel, _DMA_V2_HEIGHT_PARAM);
#else
	return _sh_css_dma_get_channel_parameter(channel, 7);
#endif
}

/* Connection group */

static inline unsigned
_sh_css_dma_get_conn_group_info(int info_id, int comp_id, int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
(void)gr_id;
	return _sh_css_dma_get_register(
		  hrt_dma_v2_conn_group_info_register_address(info_id,comp_id));
#else
	return _sh_css_dma_get_register(
		  hrt_dma_v1_conn_group_info_register_address(info_id,comp_id, gr_id));
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_command(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(0, _DMA_V2_FSM_GROUP_CMD_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(0, _DMA_SEL_CONN_CMD, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_address_A(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(0, _DMA_V2_FSM_GROUP_ADDR_SRC_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(0, _DMA_SEL_CONN_ADDRESS_A, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_address_B(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(0, _DMA_V2_FSM_GROUP_ADDR_DEST_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(0, _DMA_SEL_CONN_ADDRESS_B, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_state(int	gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_STATE_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_request_device(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_REQ_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_REQ_DEV_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_request_address(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_REQ_ADDR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_REQ_ADDR_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_request_stride(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_REQ_STRIDE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_REQ_STRIDE_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_request_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_REQ_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_REQ_XB_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_request_height(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_REQ_YB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_REQ_YB_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_request_device(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_REQ_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_REQ_DEV_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_write_device(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_DEV_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_WR_DEV_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_write_address(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_WR_ADDR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_WR_ADDR_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_write_stride(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_WR_STRIDE_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_WR_STRIDE_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_request_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_REQ_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_REQ_XB_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_write_height(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_YB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_WR_YB_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_write_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_WR_XB_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_WR_XB_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_request_elems(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_ELEM_REQ_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_ELEM_REQ_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_write_elems(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_ELEM_WR_IDX, _DMA_V2_FSM_GROUP_FSM_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_ELEM_WR_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_control_pack_extension_and_elem_order(int
								    gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_CTRL_PACK_S_Z_IDX, _DMA_V2_FSM_GROUP_CMD_CTRL_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_CTRL_PACK_S_Z_REV_IDX, _DMA_SEL_FSM_CONN_CTRL, gr_id);
#endif
}

static inline unsigned sh_css_dma_get_conn_group_fsm_pack_state(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_PACK_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_PACK_STATE_IDX, _DMA_SEL_FSM_PACK, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_pack_counter_height(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_PACK_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_PACK_CNT_YB_IDX, _DMA_SEL_FSM_PACK, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_pack_request_counter_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_PACK_CNT_XB_REQ_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_PACK_CNT_XB_REQ_IDX, _DMA_SEL_FSM_PACK, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_pack_write_counter_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_PACK_CNT_XB_WR_IDX, _DMA_V2_FSM_GROUP_FSM_PACK_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_PACK_CNT_XB_WR_IDX, _DMA_SEL_FSM_PACK, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_request_state(int	gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_REQ_STATE_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_REQ_STATE_IDX, _DMA_SEL_FSM_REQ, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_request_counter_height(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_REQ_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_REQ_CNT_YB_IDX, _DMA_SEL_FSM_REQ, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_request_counter_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_REQ_CNT_XB_IDX, _DMA_V2_FSM_GROUP_FSM_REQ_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_REQ_CNT_XB_IDX, _DMA_SEL_FSM_REQ, gr_id);
#endif
}

static inline unsigned
sh_css_dma_get_conn_group_fsm_write_height(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_WR_CNT_YB_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_WR_CNT_YB_IDX, _DMA_SEL_FSM_WR, gr_id);
#endif
}

static inline unsigned sh_css_dma_get_conn_group_fsm_write_width(int gr_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_conn_group_info(_DMA_V2_FSM_GROUP_FSM_WR_CNT_XB_IDX, _DMA_V2_FSM_GROUP_FSM_WR_IDX, gr_id);
#else
	return _sh_css_dma_get_conn_group_info(_DMA_FSM_GROUP_FSM_WR_CNT_XB_IDX, _DMA_SEL_FSM_WR, gr_id);
#endif
}

/* Device Interface */
static inline unsigned
_sh_css_dma_get_device_interface_info(int info_id, int dev_id)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_dma_get_register(
		  hrt_dma_v2_device_interface_info_register_address(info_id,dev_id));
#else
	return _sh_css_dma_get_register(
		  hrt_dma_v1_device_interface_info_register_address(info_id,dev_id));
#endif
}

static inline unsigned
sh_css_dma_get_device_interface_request_side_state(int dev_id)
{
	return _sh_css_dma_get_device_interface_info(0, dev_id);
}

static inline unsigned
sh_css_dma_get_device_interface_send_side_state(int dev_id)
{
	return _sh_css_dma_get_device_interface_info(1, dev_id);
}

static inline unsigned
sh_css_dma_get_device_interface_fifo_state(int dev_id)
{
	return _sh_css_dma_get_device_interface_info(2, dev_id);
}

void
sh_css_hrt_dma_get_state(struct sh_css_dma_state *state)
{
	int tmp, i, num_ports = 3, num_channels = 8;
	tmp = sh_css_dma_get_command_fsm_state();
	state->fsm_command_idle = tmp & 0x1;
	state->fsm_command_run = tmp & 0x2;
	state->fsm_command_stalling = tmp & 0x4;
	state->fsm_command_error    = tmp & 0x8;
	state->last_command_channel = (tmp>>8 & 0xFF);
	state->last_command_param =  (tmp>>16 & 0xF);
	tmp = (tmp>>4) & 0xF;
	if (tmp == 0)
		state->last_command = sh_css_dma_command_read;
	if (tmp == 1)
		state->last_command = sh_css_dma_command_write;
	if (tmp == 2)
		state->last_command = sh_css_dma_command_set_channel;
	if (tmp == 3)
		state->last_command = sh_css_dma_command_set_param;
	if (tmp == 4)
		state->last_command = sh_css_dma_command_read_spec;
	if (tmp == 5)
		state->last_command = sh_css_dma_command_write_spec;
	if (tmp == 8)
		state->last_command = sh_css_dma_command_init;
	if (tmp == 12)
		state->last_command = sh_css_dma_command_init_spec;
	if (tmp == 15)
		state->last_command = sh_css_dma_command_reset;
	state->current_command = sh_css_dma_get_conn_group_command(0);
	state->current_addr_a = sh_css_dma_get_conn_group_address_A(0);
	state->current_addr_b = sh_css_dma_get_conn_group_address_B(0);
	tmp = sh_css_dma_get_conn_group_fsm_control_state(0);
	state->fsm_ctrl_idle = tmp & 0x1;
	state->fsm_ctrl_run = tmp & 0x2;
	state->fsm_ctrl_stalling = tmp & 0x4;
	state->fsm_ctrl_error = tmp & 0x8;
	tmp = tmp >> 4;
	if (tmp == 0)
		state->fsm_ctrl_state = sh_css_dma_ctrl_state_idle;
	if (tmp == 1)
		state->fsm_ctrl_state = sh_css_dma_ctrl_state_req_rcv;
	if (tmp == 2)
		state->fsm_ctrl_state = sh_css_dma_ctrl_state_rcv;
	if (tmp == 3)
		state->fsm_ctrl_state = sh_css_dma_ctrl_state_rcv_req;
	if (tmp == 4)
		state->fsm_ctrl_state = sh_css_dma_ctrl_state_init;
	state->fsm_ctrl_source_dev  =
		sh_css_dma_get_conn_group_fsm_control_request_device(0);
	state->fsm_ctrl_source_addr =
		sh_css_dma_get_conn_group_fsm_control_request_address(0);
	state->fsm_ctrl_source_stride =
		sh_css_dma_get_conn_group_fsm_control_request_stride(0);
	state->fsm_ctrl_source_width =
		sh_css_dma_get_conn_group_fsm_control_request_width(0);
	state->fsm_ctrl_source_height =
		sh_css_dma_get_conn_group_fsm_control_request_height(0);
	state->fsm_ctrl_pack_source_dev =
	    sh_css_dma_get_conn_group_fsm_control_pack_request_device(0);
	state->fsm_ctrl_pack_dest_dev =
		sh_css_dma_get_conn_group_fsm_control_pack_write_device(0);
	state->fsm_ctrl_dest_addr =
		sh_css_dma_get_conn_group_fsm_control_write_address(0);
	state->fsm_ctrl_dest_stride =
		sh_css_dma_get_conn_group_fsm_control_write_stride(0);
	state->fsm_ctrl_pack_source_width =
	    sh_css_dma_get_conn_group_fsm_control_pack_request_width(0);
	state->fsm_ctrl_pack_dest_height =
		sh_css_dma_get_conn_group_fsm_control_pack_write_height(0);
	state->fsm_ctrl_pack_dest_width =
		sh_css_dma_get_conn_group_fsm_control_pack_write_width(0);
	state->fsm_ctrl_pack_source_elems =
		sh_css_dma_get_conn_group_fsm_control_pack_request_elems(0);
	state->fsm_ctrl_pack_dest_elems =
		sh_css_dma_get_conn_group_fsm_control_pack_write_elems(0);
	state->fsm_ctrl_pack_extension =
	sh_css_dma_get_conn_group_fsm_control_pack_extension_and_elem_order(0);
	tmp = sh_css_dma_get_conn_group_fsm_pack_state(0);
	state->pack_idle     = tmp & 0x1;
	state->pack_run      = tmp & 0x2;
	state->pack_stalling = tmp & 0x4;
	state->pack_error    = tmp & 0x8;
	state->pack_cnt_height =
		sh_css_dma_get_conn_group_fsm_pack_counter_height(0);
	state->pack_src_cnt_width =
	    sh_css_dma_get_conn_group_fsm_pack_request_counter_width(0);
	state->pack_dest_cnt_width =
		sh_css_dma_get_conn_group_fsm_pack_write_counter_width(0);
	tmp = sh_css_dma_get_conn_group_fsm_request_state(0) & 0x3;
	if (tmp == 0)
		state->read_state = sh_css_dma_rw_state_idle;
	if (tmp == 1)
		state->read_state = sh_css_dma_rw_state_req;
	if (tmp == 2)
		state->read_state = sh_css_dma_rw_state_next_line;
	if (tmp == 3)
		state->read_state = sh_css_dma_rw_state_unlock_channel;
	state->read_cnt_height =
		sh_css_dma_get_conn_group_fsm_request_counter_height(0);
	state->read_cnt_width =
		sh_css_dma_get_conn_group_fsm_request_counter_width(0);
	tmp = sh_css_dma_get_conn_group_fsm_request_state(0) & 0x3;
	if (tmp == 0)
		state->write_state = sh_css_dma_rw_state_idle;
	if (tmp == 1)
		state->write_state = sh_css_dma_rw_state_req;
	if (tmp == 2)
		state->write_state = sh_css_dma_rw_state_next_line;
	if (tmp == 3)
		state->write_state = sh_css_dma_rw_state_unlock_channel;
	state->write_height =
		sh_css_dma_get_conn_group_fsm_write_height(0);
	state->write_width = sh_css_dma_get_conn_group_fsm_write_width(0);
	for (i = 0; i < num_ports; i++) {
		tmp =
		    sh_css_dma_get_device_interface_request_side_state(i);
		state->port_states[i].req_cs   = (tmp & 0x1) != 0;
		state->port_states[i].req_we_n = (tmp & 0x2) != 0;
		state->port_states[i].req_run  = (tmp & 0x4) != 0;
		state->port_states[i].req_ack  = (tmp & 0x8) != 0;
		tmp = sh_css_dma_get_device_interface_send_side_state(i);
		state->port_states[i].send_cs   = (tmp & 0x1) != 0;
		state->port_states[i].send_we_n = (tmp & 0x2) != 0;
		state->port_states[i].send_run  = (tmp & 0x4) != 0;
		state->port_states[i].send_ack  = (tmp & 0x8) != 0;
		tmp = sh_css_dma_get_device_interface_fifo_state(i);
		if (tmp & 0x1) {
			state->port_states[i].fifo_state =
				sh_css_dma_fifo_state_will_be_full;
		}
		if (tmp & 0x2) {
			state->port_states[i].fifo_state =
				sh_css_dma_fifo_state_full;
		}
		if (tmp & 0x4) {
			state->port_states[i].fifo_state =
				sh_css_dma_fifo_state_empty;
		}
		state->port_states[i].fifo_counter = tmp >> 3;
	}
	for (i = 0; i < num_channels; i++) {
		state->channel_states[i].connection =
			sh_css_dma_get_channel_connection(i);
		state->channel_states[i].sign_extend =
			sh_css_dma_get_channel_extension(i);
		state->channel_states[i].reverse_elem_order =
			sh_css_dma_get_channel_element_order(i);
		state->channel_states[i].height =
			sh_css_dma_get_channel_height(i);
		state->channel_states[i].stride_a =
			sh_css_dma_get_channel_stride_A(i);
		state->channel_states[i].elems_a =
			sh_css_dma_get_channel_elements_A(i);
		state->channel_states[i].cropping_a =
			sh_css_dma_get_channel_cropping_A(i);
		state->channel_states[i].width_a =
			sh_css_dma_get_channel_width_A(i);
		state->channel_states[i].stride_b =
			sh_css_dma_get_channel_stride_B(i);
		state->channel_states[i].elems_b =
			sh_css_dma_get_channel_elements_B(i);
		state->channel_states[i].cropping_b =
			sh_css_dma_get_channel_cropping_B(i);
		state->channel_states[i].width_b =
			sh_css_dma_get_channel_width_B(i);
	}
}

static inline void
_sh_css_gdc_set_lut_entry(unsigned int index, unsigned int val)
{
	hrt_master_port_store_32(gdc_lut_base_address + index, val);
}

void
sh_css_hrt_gdc_set_lut(const int lut[4][HRT_GDC_N])
{
	unsigned int i, entry_0, entry_1, entry_2, entry_3,
		     word_0, word_1, lut_offset = HRT_GDC_LUT_IDX;

	for (i = 0; i < HRT_GDC_N; i++) {
		entry_0 = lut[0][i] & HRT_GDC_BCI_COEF_MASK;
		entry_1 = lut[1][i] & HRT_GDC_BCI_COEF_MASK;
		entry_2 = lut[2][i] & HRT_GDC_BCI_COEF_MASK;
		entry_3 = lut[3][i] & HRT_GDC_BCI_COEF_MASK;
		word_0  = entry_0 | (entry_1 << HRT_GDC_BCI_COEF_BITS);
		word_1  = entry_2 | (entry_3 << HRT_GDC_BCI_COEF_BITS);
		_sh_css_gdc_set_lut_entry(lut_offset++, word_0);
		_sh_css_gdc_set_lut_entry(lut_offset++, word_1);
	}
}

static inline void
_sh_css_mmu_set_register(unsigned int index, unsigned int value)
{
	hrt_master_port_store_32(mmu_base_address + index, value);
}

static inline unsigned int
_sh_css_mmu_get_register(unsigned int index)
{
	return hrt_master_port_uload_32(mmu_base_address + index);
}

void
sh_css_hrt_mmu_set_page_table_base_address(void *base_address)
{
	_sh_css_mmu_set_register(_HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX,
		  (unsigned long)base_address);
}

void *
sh_css_hrt_mmu_get_page_table_base_address(void)
{
	return (void *)_sh_css_mmu_get_register(
		_HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX);
}

void
sh_css_hrt_mmu_invalidate_cache(void)
{
	_sh_css_mmu_set_register(_HRT_MMU_INVALIDATE_TLB_REG_IDX, 1);
}

static inline unsigned long
_sh_css_if_get_register(unsigned long *base, unsigned addr)
{
	/* offsets are in bytes, so we cast the base to char*
	   to get the right address for each register */
	unsigned char *if_base = (unsigned char *)base;
	unsigned int bus_addr = (unsigned int)(if_base + addr);
	return hrt_master_port_load_32(bus_addr);
}

static inline void
_sh_css_if_set_register(unsigned long *base, unsigned addr, unsigned long value)
{
	/* offsets are in bytes, so we cast the base to char*
	   to get the right address for each register */
	unsigned char *if_base = (unsigned char *)base;
	unsigned int bus_addr = (unsigned int)(if_base + addr);
	hrt_master_port_store_32(bus_addr, value);
	/* hack */
	(void)hrt_master_port_load_32(bus_addr);
}

static inline unsigned
sh_css_if_get_start_line(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_START_LINE_ADDRESS);
}

static inline unsigned
sh_css_if_get_start_column(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_START_COLUMN_ADDRESS);
}

static inline unsigned
sh_css_if_get_cropped_height(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_CROPPED_HEIGHT_ADDRESS);
}

static inline unsigned
sh_css_if_get_cropped_width(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_CROPPED_WIDTH_ADDRESS);
}

static inline unsigned
sh_css_if_get_vertical_decimation(unsigned long	*ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_VERTICAL_DECIMATION_ADDRESS);
}

static inline unsigned
sh_css_if_get_horizontal_decimation(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_HORIZONTAL_DECIMATION_ADDRESS);
}

static inline unsigned
sh_css_if_get_v_deinterleaving(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_V_DEINTERLEAVING_ADDRESS);
}

static inline unsigned
sh_css_if_get_h_deinterleaving(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_H_DEINTERLEAVING_ADDRESS);
}

static inline unsigned
sh_css_if_get_leftpadding_width(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_LEFTPADDING_WIDTH_ADDRESS);
}

static inline unsigned
sh_css_if_get_vmem_start_address(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_VMEM_START_ADDRESS_ADDRESS);
}

static inline unsigned
sh_css_if_get_end_of_line_offset(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_END_OF_LINE_OFFSET_ADDRESS);
}

static inline unsigned
sh_css_if_get_vmem_end_address(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_VMEM_END_ADDRESS_ADDRESS);
}

static inline unsigned
sh_css_if_get_allow_fifo_overflow(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_ALLOW_FIFO_OVERFLOW_ADDRESS);
}

static inline unsigned
sh_css_if_get_vmem_increment(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_VMEM_INCREMENT_ADDRESS);
}

static inline unsigned
sh_css_if_get_fsm_sync_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase, HIVE_IF_FSM_SYNC_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_crop_status(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase, HIVE_IF_FSM_CROP_STATUS);
}

static inline unsigned
sh_css_if_get_fsm_crop_line_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_CROP_LINE_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_crop_pixel_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_CROP_PIXEL_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_deinterleaving_index(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_DEINTERLEAVING_IDX);
}

static inline unsigned
sh_css_if_get_fsm_dec_h_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_DECIMATION_H_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_dec_v_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_DECIMATION_V_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_dec_block_v_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_DECIMATION_BLOCK_V_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_padding_status(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_PADDING_STATUS);
}

static inline unsigned
sh_css_if_get_fsm_padding_elem_counter(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
				       HIVE_IF_FSM_PADDING_ELEMENT_COUNTER);
}

static inline unsigned
sh_css_if_get_fsm_vector_support_error(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_VECTOR_SUPPORT_ERROR);
}

static inline unsigned
sh_css_if_get_fsm_vector_buffer_full(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_VECTOR_SUPPORT_BUFF_FULL);
}

static inline unsigned
sh_css_if_get_vector_support(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FSM_VECTOR_SUPPORT);
}

static inline unsigned
sh_css_if_get_sensor_data_lost(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
			HIVE_IF_FIFO_SENSOR_DATA_LOST);
}

static inline unsigned
sh_css_if_get_reset(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase, HIVE_IF_RESET_ADDRESS);
}

static inline unsigned
sh_css_if_get_fsm_sync_status(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase, HIVE_IF_FSM_SYNC_STATUS);
}

static inline unsigned
sh_css_if_get_yuv_420_format(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
				       HIVE_IF_YUV_420_FORMAT_ADDRESS);
}

static inline unsigned
sh_css_if_get_vsynck_active_low(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
				       HIVE_IF_VSYNCK_ACTIVE_LOW_ADDRESS);
}

static inline unsigned
sh_css_if_get_hsynck_active_low(unsigned long *ifbase)
{
	return _sh_css_if_get_register(ifbase,
				       HIVE_IF_HSYNCK_ACTIVE_LOW_ADDRESS);
}

static inline void
sh_css_if_set_block_fifo_no_reqs(unsigned long *ifbase, bool enable)
{
	_sh_css_if_set_register(ifbase,
				HIVE_IF_BLOCK_FIFO_NO_REQ_ADDRESS,
				enable);
}

static void
sh_css_if_get_state(unsigned long *ifbase, struct sh_css_if_state *state)
{
	state->reset = sh_css_if_get_reset(ifbase);
	state->start_line = sh_css_if_get_start_line(ifbase);
	state->start_column = sh_css_if_get_start_column(ifbase);
	state->cropped_height = sh_css_if_get_cropped_height(ifbase);
	state->cropped_width = sh_css_if_get_cropped_width(ifbase);
	state->ver_decimation = sh_css_if_get_vertical_decimation(ifbase);
	state->hor_decimation = sh_css_if_get_horizontal_decimation(ifbase);
	state->deinterleaving = sh_css_if_get_h_deinterleaving(ifbase);
	state->left_padding = sh_css_if_get_leftpadding_width(ifbase);
	state->eol_offset = sh_css_if_get_end_of_line_offset(ifbase);
	state->vmem_start_address = sh_css_if_get_vmem_start_address(ifbase);
	state->vmem_end_address = sh_css_if_get_vmem_end_address(ifbase);
	state->vmem_increment = sh_css_if_get_vmem_increment(ifbase);
	state->yuv420 = sh_css_if_get_yuv_420_format(ifbase);
	state->vsync_active_low = sh_css_if_get_vsynck_active_low(ifbase);
	state->hsync_active_low = sh_css_if_get_hsynck_active_low(ifbase);
	state->allow_fifo_overflow = sh_css_if_get_allow_fifo_overflow(ifbase);
	state->fsm_sync_status = sh_css_if_get_fsm_sync_status(ifbase);
	state->fsm_sync_counter = sh_css_if_get_fsm_sync_counter(ifbase);
	state->fsm_crop_status = sh_css_if_get_fsm_crop_status(ifbase);
	state->fsm_crop_line_counter =
		sh_css_if_get_fsm_crop_line_counter(ifbase);
	state->fsm_crop_pixel_counter =
		sh_css_if_get_fsm_crop_pixel_counter(ifbase);
	state->fsm_deinterleaving_index =
		sh_css_if_get_fsm_deinterleaving_index(ifbase);
	state->fsm_dec_h_counter = sh_css_if_get_fsm_dec_h_counter(ifbase);
	state->fsm_dec_v_counter = sh_css_if_get_fsm_dec_v_counter(ifbase);
	state->fsm_dec_block_v_counter =
		sh_css_if_get_fsm_dec_block_v_counter(ifbase);
	state->fsm_padding_status = sh_css_if_get_fsm_padding_status(ifbase);
	state->fsm_padding_elem_counter =
		sh_css_if_get_fsm_padding_elem_counter(ifbase);
	state->fsm_vector_support_error =
		sh_css_if_get_fsm_vector_support_error(ifbase);
	state->fsm_vector_buffer_full =
		sh_css_if_get_fsm_vector_buffer_full(ifbase);
	state->vector_support = sh_css_if_get_vector_support(ifbase);
	state->sensor_data_lost = sh_css_if_get_sensor_data_lost(ifbase);
}

unsigned int
sh_css_hrt_if_prim_vec_align(void)
{
	return _HRT_IF_VEC_ALIGN(IF_PRIM);
}

void
sh_css_hrt_if_prim_a_get_state(struct sh_css_if_state *state)
{
	sh_css_if_get_state(if_prim_a_base_address, state);
}

void
sh_css_hrt_if_prim_b_get_state(struct sh_css_if_state *state)
{
	sh_css_if_get_state(if_prim_b_base_address, state);
}

void
sh_css_hrt_if_reset(void)
{
	_sh_css_if_set_register(if_prim_a_base_address,
				HIVE_IF_RESET_ADDRESS, 1);
	_sh_css_if_set_register(if_prim_b_base_address,
				HIVE_IF_RESET_ADDRESS, 1);
}

void
sh_css_hrt_if_set_block_fifo_no_reqs(bool enable_prim, bool enable_prim_b)
{
	sh_css_if_set_block_fifo_no_reqs(if_prim_a_base_address, enable_prim);
	sh_css_if_set_block_fifo_no_reqs(if_prim_b_base_address, enable_prim_b);
}

/* Streaming monitors */
static inline unsigned long
sh_css_stream_monitor_get_register(unsigned addr)
{
	unsigned int bus_addr = (unsigned int)(str_monitor_base_address + addr);
	return hrt_master_port_load_32(bus_addr);
}

static inline unsigned int
sh_css_sp_stream_monitor_get_status(void)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_SP_STREAM_STAT_IDX);
#else
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_SP_STREAM_STAT);
#endif
}

static inline unsigned int
sh_css_isp_stream_monitor_get_status(void)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_ISP_STREAM_STAT_IDX);
#else
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_ISP_STREAM_STAT);
#endif
}

static inline unsigned int
sh_css_mod_stream_monitor_get_status(void)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_MOD_STREAM_STAT_IDX);
#else
	return sh_css_stream_monitor_get_register(HIVE_GP_REGS_MOD_STREAM_STAT);
#endif
}

/* use the following function if the stream stat word is already available.  */
static inline unsigned int
sh_css_get_stream_stat_valid(unsigned int stream_stat_word,
			     unsigned int port_id)
{
	return ((stream_stat_word) >>
		(((port_id * 2) + _hive_str_mon_valid_offset))) & 0x1;
}

static inline unsigned int
sh_css_get_stream_stat_accept(unsigned int stream_stat_word,
			      unsigned int port_id)
{
	return ((stream_stat_word) >>
		(((port_id * 2) + _hive_str_mon_accept_offset))) & 0x1;
}

/* Following functions are intended for single bit lookup.
 * If multiple valid/accept bits are required from the same words
 * use the hrt_get_*_stream_stat_word in combination with
 * hrt_gets_stream_valid/accept functions.
 */
static inline unsigned int
sh_css_get_sp_stream_stat_valid(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_sp_stream_monitor_get_status();
	return sh_css_get_stream_stat_valid(status, port_id);
}

static inline unsigned int
sh_css_get_sp_stream_stat_accept(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_sp_stream_monitor_get_status();
	return sh_css_get_stream_stat_accept(status, port_id);
}

static inline unsigned int
sh_css_get_isp_stream_stat_valid(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_isp_stream_monitor_get_status();
	return sh_css_get_stream_stat_valid(status, port_id);
}

static inline unsigned int
sh_css_get_isp_stream_stat_accept(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_isp_stream_monitor_get_status();
	return sh_css_get_stream_stat_accept(status, port_id);
}

static inline unsigned int
sh_css_get_mod_stream_stat_valid(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_mod_stream_monitor_get_status();
	return sh_css_get_stream_stat_valid(status, port_id);
}

static inline unsigned int
sh_css_get_mod_stream_stat_accept(unsigned int port_id)
{
	unsigned int status;
	status = sh_css_mod_stream_monitor_get_status();
	return sh_css_get_stream_stat_accept(status, port_id);
}

bool
sh_css_hrt_system_is_idle(void)
{
	bool not_idle = false;

	not_idle |= !sh_css_cell_is_ready(isp_sc_base_address + ISP_SC_REG,
					  ISP_IDLE_BIT);
	/* ISP to SP */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_SND_SP);
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_RCV_ISP);
	/* SP to ISP */
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_SND_ISP);
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_RCV_SP);
	/* IF_PRIM_A to ISP */
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_SND_PIF_A);
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_RCV_PIF_A);
	/* ISP to IF_PRIM_A */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_SND_PIF_A);
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_RCV_PIF_A);
	/* IF_PRIM_B to ISP */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_RCV_PIF_B);
	/* ISP to IF_PRIM_B */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_SND_PIF_B);
	/* DMA to SP */
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_SND_DMA);
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_RCV_DMA);
	/* SP to DMA */
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_SND_DMA);
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_RCV_DMA);
	/* DMA to ISP */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_RCV_DMA);
	/* ISP to DMA */
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_SND_DMA);
	/* GDC to SP/ISP */
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_SND_GDC);
#ifndef SYSTEM_hive_isp_css_2400_system
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_RCV_GDC);
#endif
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_RCV_GDC);
	/* SP/ISP to GDC */
#ifndef SYSTEM_hive_isp_css_2400_system
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_SND_GDC);
#endif
	not_idle |=
	    sh_css_get_isp_stream_stat_valid(ISP_STR_MON_PORT_SND_GDC);
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_RCV_GDC);
	/* Stream2Mem to SP */
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_SND_MC);
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_RCV_MC);
	/* SP to Stream2Mem */
	not_idle |=
	    sh_css_get_sp_stream_stat_valid(SP_STR_MON_PORT_SND_MC);
	not_idle |=
	    sh_css_get_mod_stream_stat_valid(MOD_STR_MON_PORT_RCV_MC);
	return !not_idle;
}

void
sh_css_hrt_fifo_channel_get_state(enum sh_css_fifo_channel channel,
				  struct sh_css_fifo_channel_state *state)
{
	switch (channel) {
	case sh_css_hrt_fifo_isp_to_sp:
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_SND_SP);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_SND_SP);
		state->fifo_valid  = sh_css_get_sp_stream_stat_valid(
					SP_STR_MON_PORT_RCV_ISP);
		state->sink_accept = sh_css_get_sp_stream_stat_accept(
					SP_STR_MON_PORT_RCV_ISP);
		break;
	case sh_css_hrt_fifo_sp_to_isp:
		state->src_valid   = sh_css_get_sp_stream_stat_valid(
					SP_STR_MON_PORT_SND_ISP);
		state->fifo_accept = sh_css_get_sp_stream_stat_accept(
					SP_STR_MON_PORT_SND_ISP);
		state->fifo_valid  = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_RCV_SP);
		state->sink_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_RCV_SP);
		break;
	case sh_css_hrt_fifo_isp_to_if_prim_a:
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_SND_PIF_A);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_SND_PIF_A);
		state->fifo_valid  = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_RCV_PIF_A);
		state->sink_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_RCV_PIF_A);
		break;
	case sh_css_hrt_fifo_if_prim_a_to_isp:
		state->src_valid   = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_SND_PIF_A);
		state->fifo_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_SND_PIF_A);
		state->fifo_valid  = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_RCV_PIF_A);
		state->sink_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_RCV_PIF_A);
		break;
	case sh_css_hrt_fifo_isp_to_if_prim_b:
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_SND_PIF_B);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_SND_PIF_B);
		state->fifo_valid  = false; /* no monitor connected */
		state->sink_accept = false; /* no monitor connected */
		break;
	case sh_css_hrt_fifo_if_prim_b_to_isp:
		state->src_valid   = false; /* no monitor connected */
		state->fifo_accept = false; /* no monitor connected */
		state->fifo_valid  = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_RCV_PIF_B);
		state->sink_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_RCV_PIF_B);
		break;
	case sh_css_hrt_fifo_isp_to_dma:
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_SND_DMA);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_SND_DMA);
		state->fifo_valid  = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_RCV_DMA);
		state->sink_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_RCV_DMA);
		break;
	case sh_css_hrt_fifo_dma_to_isp:
		state->src_valid   = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_SND_DMA);
		state->fifo_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_SND_DMA);
		state->fifo_valid  = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_RCV_DMA);
		state->sink_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_RCV_DMA);
		break;
	case sh_css_hrt_fifo_sp_to_dma:
		state->src_valid   = sh_css_get_sp_stream_stat_valid(
					SP_STR_MON_PORT_SND_DMA);
		state->fifo_accept = sh_css_get_sp_stream_stat_accept(
					SP_STR_MON_PORT_SND_DMA);
		state->fifo_valid  = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_RCV_DMA);
		state->sink_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_RCV_DMA);
		break;
	case sh_css_hrt_fifo_dma_to_sp:
		state->src_valid   = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_SND_DMA);
		state->fifo_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_SND_DMA);
		state->fifo_valid  = sh_css_get_sp_stream_stat_valid(
					SP_STR_MON_PORT_RCV_DMA);
		state->sink_accept = sh_css_get_sp_stream_stat_accept(
					SP_STR_MON_PORT_RCV_DMA);
		break;
	case sh_css_hrt_fifo_isp_to_gdc:
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_SND_GDC);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_SND_GDC);
		state->fifo_valid  = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_RCV_GDC);
		state->sink_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_RCV_GDC);
		break;
	case sh_css_hrt_fifo_gdc_to_isp:
		state->fifo_valid  = sh_css_get_mod_stream_stat_valid(
					MOD_STR_MON_PORT_SND_GDC);
		state->sink_accept = sh_css_get_mod_stream_stat_accept(
					MOD_STR_MON_PORT_SND_GDC);
		state->src_valid   = sh_css_get_isp_stream_stat_valid(
					ISP_STR_MON_PORT_RCV_GDC);
		state->fifo_accept = sh_css_get_isp_stream_stat_accept(
					ISP_STR_MON_PORT_RCV_GDC);
		break;
	}
}

static inline unsigned long
_sh_css_irq_get_reg(unsigned addr)
{
	unsigned int bus_addr = (unsigned int)(irq_ctrl_base_address + addr);
	return hrt_master_port_load_32(bus_addr);
}

static inline void
_sh_css_irq_set_reg(unsigned addr, unsigned long value)
{
	hrt_master_port_store_32(irq_ctrl_base_address + addr, value);
	/* workaround: without this read, the write sometimes does not
	   get propagated into the register. This being investigated by
	   Jozef. */
#ifndef HRT_CSIM
	/* todo: make this register readable in hss model */
	(void)hrt_master_port_load_32(irq_ctrl_base_address + addr);
#else
	/* Lex: write needs to be processed by IRQ controller.
	   Therefore, we need a context switch to let it do this.
	   The asic system has a bus with delay, so that will do this
	   context switch. The fpga system, however, does not have a bus delay.
	   Jozef to find a clean sync mechanism.
	 */
	hrt_sleep();
#endif
}

/* set register */
static inline void
sh_css_irq_set_edge_reg(unsigned long val)
{
	_sh_css_irq_set_reg(_HRT_IRQ_CONTROLLER_EDGE_REG_IDX, val);
}

static inline void
sh_css_irq_set_mask_reg(unsigned long val)
{
	_sh_css_irq_set_reg(_HRT_IRQ_CONTROLLER_MASK_REG_IDX, val);
}

static inline void
sh_css_irq_clear_status_reg(unsigned long val)
{
	_sh_css_irq_set_reg(_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX, val);
}

static inline void
sh_css_irq_set_enable_reg(unsigned long val)
{
	_sh_css_irq_set_reg(_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX, val);
}

static inline void
sh_css_irq_set_edge_not_pulse_reg(unsigned long val)
{
	_sh_css_irq_set_reg(_HRT_IRQ_CONTROLLER_EDGE_NOT_PULSE_REG_IDX, val);
}

static inline unsigned
sh_css_irq_get_edge_reg(void)
{
	return _sh_css_irq_get_reg(_HRT_IRQ_CONTROLLER_EDGE_REG_IDX);
}

static inline unsigned
sh_css_irq_get_mask_reg(void)
{
	return _sh_css_irq_get_reg(_HRT_IRQ_CONTROLLER_MASK_REG_IDX);
}

static inline unsigned
sh_css_irq_get_status_reg(void)
{
	return _sh_css_irq_get_reg(_HRT_IRQ_CONTROLLER_STATUS_REG_IDX);
}

static inline unsigned
sh_css_irq_get_enable_reg(void)
{
	return _sh_css_irq_get_reg(_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX);
}

static inline unsigned
sh_css_irq_get_edge_not_pulse_reg(void)
{
	return _sh_css_irq_get_reg(_HRT_IRQ_CONTROLLER_EDGE_NOT_PULSE_REG_IDX);
}

void
sh_css_hrt_irq_clear_all(void)
{
	sh_css_irq_clear_status_reg(0xFFFFFFFF);
}

void
sh_css_hrt_irq_enable(enum hrt_isp_css_irq irq_id,
		      bool rising_edge_in,
		      bool edge_not_pulse_out)
{
	unsigned int mask = sh_css_irq_get_mask_reg();
	unsigned int enable = sh_css_irq_get_enable_reg();
	unsigned int edge_in = sh_css_irq_get_edge_reg();
	unsigned int edge_out = sh_css_irq_get_edge_not_pulse_reg();
	unsigned int me = 1u << irq_id;

	mask |= me;
	enable |= me;
	if (rising_edge_in)
		edge_in |= me;
	if (edge_not_pulse_out)
		edge_out |= me;

	/* to avoid mishaps configuration must follow the following order */

	/* mask this interrupt */
	sh_css_irq_set_mask_reg(mask & ~me);
	/* rising edge at input */
	sh_css_irq_set_edge_reg(edge_in);
	/* enable interrupt to output */
	sh_css_irq_set_enable_reg(enable);
	/* output is given as edge, not pulse */
	sh_css_irq_set_edge_not_pulse_reg(edge_out);
	/* clear current irq only */
	sh_css_irq_clear_status_reg(me);
	/* unmask interrupt from input */
	sh_css_irq_set_mask_reg(mask);
}

void
sh_css_hrt_irq_disable(enum hrt_isp_css_irq irq_id)
{
	unsigned int mask = sh_css_irq_get_mask_reg();
	unsigned int enable = sh_css_irq_get_enable_reg();
	unsigned int me = 1u << irq_id;

	mask &= ~me;
	enable &= ~me;

	/* enable interrupt to output */
	sh_css_irq_set_enable_reg(enable);
	/* unmask interrupt from input */
	sh_css_irq_set_mask_reg(mask);
	/* clear current irq only */
	sh_css_irq_clear_status_reg(me);
}

enum hrt_isp_css_irq_status
sh_css_hrt_irq_get_id(enum hrt_isp_css_irq *irq_id)
{
	unsigned int irq_status = sh_css_irq_get_status_reg();
	unsigned int irq_mask = sh_css_irq_get_mask_reg();
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_success;
	int id1, id2;

	/* find the first irq bit */
	id1 = ffs(irq_status) - 1;

	if (id1 == -1 || id1 >= hrt_isp_css_irq_num_irqs)
		return hrt_isp_css_irq_status_error;

	irq_status &= ~(1u << id1);

	/* now check whether there are more bits set */
	id2 = ffs(irq_status) - 1;
	if (id2 > id1 && id2 < hrt_isp_css_irq_num_irqs)
		status = hrt_isp_css_irq_status_more_irqs;

	/* now we clear the irq status bit, to avoid generating a
	 * new IRQ, we must set the mask temporarily to zero.
	 */
	sh_css_irq_set_mask_reg(0);
	sh_css_irq_clear_status_reg(1u << id1);
	sh_css_irq_set_mask_reg(irq_mask);
	if (irq_id)
		*irq_id = (enum hrt_isp_css_irq) id1;
	return status;
}

/* Streaming to MIPI */
static inline unsigned
_sh_css_wrap_marker(unsigned marker)
{
	return marker |
	       (curr_ch_id << HIVE_STR_TO_MIPI_CH_ID_LSB) |
	       (curr_fmt_type << _HIVE_STR_TO_MIPI_FMT_TYPE_LSB);
}

static inline void
_sh_css_fifo_snd(unsigned token)
{
	hrt_master_port_store_32(gp_fifo_base_address, token);
}

static inline void
sh_css_streaming_to_mipi_send_data_a(unsigned int data)
{
	unsigned int token = (1 << HIVE_STR_TO_MIPI_VALID_A_BIT) |
			     (data << HIVE_STR_TO_MIPI_DATA_A_LSB);
	_sh_css_fifo_snd(token);
}

static inline void
sh_css_streaming_to_mipi_send_data_b(unsigned int data)
{
	unsigned int token = (1 << HIVE_STR_TO_MIPI_VALID_B_BIT) |
			     (data << _HIVE_STR_TO_MIPI_DATA_B_LSB);
	_sh_css_fifo_snd(token);
}

static inline void
sh_css_streaming_to_mipi_send_data(unsigned int a, unsigned int b)
{
	unsigned int token = ((1 << HIVE_STR_TO_MIPI_VALID_A_BIT) |
			      (1 << HIVE_STR_TO_MIPI_VALID_B_BIT) |
			      (a << HIVE_STR_TO_MIPI_DATA_A_LSB) |
			      (b << _HIVE_STR_TO_MIPI_DATA_B_LSB));
	_sh_css_fifo_snd(token);
}

static inline void
sh_css_streaming_to_mipi_send_sol(void)
{
	_sh_css_fifo_snd(_sh_css_wrap_marker(1 << HIVE_STR_TO_MIPI_SOL_BIT));
}

static inline void
sh_css_streaming_to_mipi_send_eol(void)
{
	_sh_css_fifo_snd(_sh_css_wrap_marker(1 << HIVE_STR_TO_MIPI_EOL_BIT));
}

static inline void
sh_css_streaming_to_mipi_send_sof(void)
{
	_sh_css_fifo_snd(_sh_css_wrap_marker(1 << HIVE_STR_TO_MIPI_SOF_BIT));
}

static inline void
sh_css_streaming_to_mipi_send_eof(void)
{
	_sh_css_fifo_snd(_sh_css_wrap_marker(1 << HIVE_STR_TO_MIPI_EOF_BIT));
}

static inline void
sh_css_streaming_to_mipi_send_ch_id(unsigned int ch_id)
{
	curr_ch_id = ch_id & _HIVE_ISP_CH_ID_MASK;
	/* we send an zero marker, this will wrap the ch_id and
	 * fmt_type automatically.
	 */
	_sh_css_fifo_snd(_sh_css_wrap_marker(0));
}

static inline void
sh_css_streaming_to_mipi_send_fmt_type(unsigned int fmt_type)
{
	curr_fmt_type = fmt_type & _HIVE_ISP_FMT_TYPE_MASK;
	/* we send an zero marker, this will wrap the ch_id and
	 * fmt_type automatically.
	 */
	_sh_css_fifo_snd(_sh_css_wrap_marker(0));
}

static inline void
sh_css_streaming_to_mipi_send_ch_id_and_fmt_type(unsigned int ch_id,
						 unsigned int fmt_type)
{
	curr_ch_id = ch_id & _HIVE_ISP_CH_ID_MASK;
	curr_fmt_type = fmt_type & _HIVE_ISP_FMT_TYPE_MASK;
	/* we send an zero marker, this will wrap the ch_id and
	 * fmt_type automatically.
	 */
	_sh_css_fifo_snd(_sh_css_wrap_marker(0));
}

static inline void
sh_css_streaming_to_mipi_send_empty_token(void)
{
	_sh_css_fifo_snd(_sh_css_wrap_marker(0));
}

static inline void
sh_css_hrt_s2m_start_frame(unsigned int ch_id,
				     unsigned int fmt_type)
{
	sh_css_streaming_to_mipi_send_ch_id_and_fmt_type(ch_id, fmt_type);
	sh_css_streaming_to_mipi_send_sof();
}

static void
sh_css_hrt_s2m_end_frame(unsigned int marker_cycles)
{
	unsigned int i;
	for (i = 0; i < marker_cycles; i++)
		sh_css_streaming_to_mipi_send_empty_token();
	sh_css_streaming_to_mipi_send_eof();
}

static void
sh_css_hrt_s2m_send_line2(unsigned short *data,
				unsigned int width,
				unsigned short *data2,
				unsigned int width2,
				unsigned int hblank_cycles,
				unsigned int marker_cycles,
				unsigned int two_ppc,
				enum sh_css_mipi_data_type type)
{
	unsigned int i, is_rgb = 0, is_legacy = 0;

	if (type == sh_css_mipi_data_type_rgb)
		is_rgb = 1;

#ifndef SYSTEM_hive_isp_css_2400_system
	if (type == sh_css_mipi_data_type_yuv420_legacy)
		is_legacy = 1;
#endif

	for (i = 0; i < hblank_cycles; i++)
		sh_css_streaming_to_mipi_send_empty_token();
	sh_css_streaming_to_mipi_send_sol();
	for (i = 0; i < marker_cycles; i++)
		sh_css_streaming_to_mipi_send_empty_token();

	for (i = 0; i < width; i++, data++) {
		/* for RGB in two_ppc, we only actually send 2 pixels per
		 * clock in the even pixels (0, 2 etc). In the other cycles,
		 * we only send 1 pixel, to data[0].
		 */
		unsigned int send_two_pixels = two_ppc;
		if ((is_rgb || is_legacy) && (i % 3 == 2))
			send_two_pixels = 0;
		if (send_two_pixels) {
			if (i + 1 == width) {
				/* for jpg (binary) copy, this can occur
				 * if the file contains an odd number of bytes.
				 */
				sh_css_streaming_to_mipi_send_data(
							data[0], 0);
			} else {
				sh_css_streaming_to_mipi_send_data(
							data[0], data[1]);
			}
			/* Additional increment because we send 2 pixels */
			data++;
			i++;
		} else if (two_ppc && is_legacy) {
			sh_css_streaming_to_mipi_send_data_b(data[0]);
		} else {
			sh_css_streaming_to_mipi_send_data_a(data[0]);
		}
#ifdef SYSTEM_hive_isp_css_2400_system
		hrt_sleep();
#endif
	}

#ifndef SYSTEM_hive_isp_css_2400_system
	for (i = 0; i < width2; i++, data2++) {
		/* for RGB in two_ppc, we only actually send 2 pixels per
		 * clock in the even pixels (0, 2 etc). In the other cycles,
		 * we only send 1 pixel, to data2[0].
		 */
		unsigned int send_two_pixels = two_ppc;
		if ((is_rgb || is_legacy) && (i % 3 == 2))
			send_two_pixels = 0;
		if (send_two_pixels) {
			if (i + 1 == width2) {
				/* for jpg (binary) copy, this can occur
				 * if the file contains an odd number of bytes.
				 */
				sh_css_streaming_to_mipi_send_data(
							data2[0], 0);
			} else {
				sh_css_streaming_to_mipi_send_data(
							data2[0], data2[1]);
			}
			/* Additional increment because we send 2 pixels */
			data2++;
			i++;
		} else if (two_ppc && is_legacy) {
			sh_css_streaming_to_mipi_send_data_b(data2[0]);
		} else {
			sh_css_streaming_to_mipi_send_data_a(data2[0]);
		}
	}
#endif
	for (i = 0; i < hblank_cycles; i++)
		sh_css_streaming_to_mipi_send_empty_token();
	sh_css_streaming_to_mipi_send_eol();
}

static void
sh_css_hrt_s2m_send_line(unsigned short *data,
				unsigned int width,
				unsigned int hblank_cycles,
				unsigned int marker_cycles,
				unsigned int two_ppc,
				enum sh_css_mipi_data_type type)
{
	sh_css_hrt_s2m_send_line2(data, width, NULL, 0,
					hblank_cycles,
					marker_cycles,
					two_ppc,
					type);
}

/* Send a frame of data into the input network via the GP FIFO.
 *  Parameters:
 *   - data: array of 16 bit values that contains all data for the frame.
 *   - width: width of a line in number of subpixels, for yuv420 it is the
 *            number of Y components per line.
 *   - height: height of the frame in number of lines.
 *   - ch_id: channel ID.
 *   - fmt_type: format type.
 *   - hblank_cycles: length of horizontal blanking in cycles.
 *   - marker_cycles: number of empty cycles after start-of-line and before
 *                    end-of-frame.
 *   - two_ppc: boolean, describes whether to send one or two pixels per clock
 *              cycle. In this mode, we sent pixels N and N+1 in the same cycle,
 *              to IF_PRIM_A and IF_PRIM_B respectively. The caller must make
 *              sure the input data has been formatted correctly for this.
 *              For example, for RGB formats this means that unused values
 *              must be inserted.
 *   - yuv420: boolean, describes whether (non-legacy) yuv420 data is used. In
 *             this mode, the odd lines (1,3,5 etc) are half as long as the
 *             even lines (2,4,6 etc).
 *             Note that the first line is odd (1) and the second line is even
 *             (2).
 *
 * This function does not do any reordering of pixels, the caller must make
 * sure the data is in the righ format. Please refer to the CSS receiver
 * documentation for details on the data formats.
 */
static void
sh_css_hrt_s2m_send_frame(unsigned short *data,
				    unsigned int width,
				    unsigned int height,
				    unsigned int ch_id,
				    unsigned int fmt_type,
				    unsigned int hblank_cycles,
				    unsigned int marker_cycles,
				    unsigned int two_ppc,
				    enum sh_css_mipi_data_type type)
{
	unsigned int i;

	sh_css_hrt_s2m_start_frame(ch_id, fmt_type);
	for (i = 0; i < height; i++) {
		if ((type == sh_css_mipi_data_type_yuv420) &&
		    (i & 1) == 1) {
			sh_css_hrt_s2m_send_line(data, 2 * width,
							   hblank_cycles,
							   marker_cycles,
							   two_ppc, type);
			data += 2 * width;
		} else {
			sh_css_hrt_s2m_send_line(data, width,
							   hblank_cycles,
							   marker_cycles,
							   two_ppc, type);
			data += width;
		}
	}
	sh_css_hrt_s2m_end_frame(marker_cycles);
}

#if defined(SYSTEM_hive_isp_css_2400_system)
#include <hive_isp_css_2400_system.h>
#endif

static enum sh_css_mipi_data_type
sh_css_hrt_s2m_determine_type(enum sh_css_input_format input_format)
{
	enum sh_css_mipi_data_type type;

	type = sh_css_mipi_data_type_regular;
	if (input_format == SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY) {
#if defined(SYSTEM_hive_isp_css_2400_system)
/* MW_R1MRFLD : yuv420_legacy and rgb share the same protocol */
		type = sh_css_mipi_data_type_rgb;
#else
		type =
			sh_css_mipi_data_type_yuv420_legacy;
#endif
	} else if (input_format == SH_CSS_INPUT_FORMAT_YUV420_8 ||
		   input_format == SH_CSS_INPUT_FORMAT_YUV420_10) {
		type =
			sh_css_mipi_data_type_yuv420;
	} else if (input_format >= SH_CSS_INPUT_FORMAT_RGB_444 &&
		   input_format <= SH_CSS_INPUT_FORMAT_RGB_888) {
		type =
			sh_css_mipi_data_type_rgb;
	}
	return type;
}

static struct streamining_to_mipi_instance *
sh_css_hrt_s2m_get_inst(unsigned int ch_id)
{
	return &s2m_inst_admin[ch_id];
}

void
sh_css_hrt_send_input_frame(unsigned short *data,
			    unsigned int width,
			    unsigned int height,
			    unsigned int ch_id,
			    enum sh_css_input_format input_format,
			    bool two_ppc)
{
	unsigned int fmt_type, hblank_cycles, marker_cycles;
	enum sh_css_mipi_data_type type;

#if defined(SYSTEM_hive_isp_css_2400_system)
	hblank_cycles = 187;
	marker_cycles = 6;

	sh_css_input_format_type(input_format, SH_CSS_MIPI_COMPRESSION_NONE, &fmt_type);
	type = sh_css_mipi_data_type_regular;
	if (input_format == SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY) {
/* MW_R1MRFLD : yuv420_legacy and rgb share the same protocol */
		type = sh_css_mipi_data_type_rgb;
	} else if (input_format == SH_CSS_INPUT_FORMAT_YUV420_8 ||
		   input_format == SH_CSS_INPUT_FORMAT_YUV420_10) {
		type = sh_css_mipi_data_type_yuv420;
	} else if (input_format >= SH_CSS_INPUT_FORMAT_RGB_444 &&
		   input_format <= SH_CSS_INPUT_FORMAT_RGB_888) {
		type = sh_css_mipi_data_type_rgb;
	}

	sh_css_hrt_s2m_send_frame(data, width, height,
			ch_id, fmt_type, hblank_cycles, marker_cycles,
			two_ppc, type);

#else
	hblank_cycles = HBLANK_CYCLES;
	marker_cycles = MARKER_CYCLES;
	sh_css_input_format_type(input_format,
				 SH_CSS_MIPI_COMPRESSION_NONE,
				 &fmt_type);

	type = sh_css_hrt_s2m_determine_type(input_format);

	sh_css_hrt_s2m_send_frame(data, width, height,
			ch_id, fmt_type, hblank_cycles, marker_cycles,
			two_ppc, type);
#endif
}

void
sh_css_hrt_streaming_to_mipi_start_frame(unsigned int ch_id,
				enum sh_css_input_format input_format,
				bool two_ppc)
{
	struct streamining_to_mipi_instance *s2mi;
	s2mi = sh_css_hrt_s2m_get_inst(ch_id);

	s2mi->ch_id = ch_id;
	sh_css_input_format_type(input_format, SH_CSS_MIPI_COMPRESSION_NONE,
				&s2mi->fmt_type);
	s2mi->two_ppc = two_ppc;
	s2mi->type = sh_css_hrt_s2m_determine_type(input_format);
	s2mi->hblank_cycles = HBLANK_CYCLES;
	s2mi->marker_cycles = MARKER_CYCLES;
	s2mi->streaming = true;

	sh_css_hrt_s2m_start_frame(ch_id, s2mi->fmt_type);
}

void
sh_css_hrt_streaming_to_mipi_send_line(unsigned int ch_id,
						unsigned short *data,
						unsigned int width,
						unsigned short *data2,
						unsigned int width2)

{
	struct streamining_to_mipi_instance *s2mi;
	s2mi = sh_css_hrt_s2m_get_inst(ch_id);

	/* Set global variables that indicate channel_id and format_type */
	curr_ch_id = (s2mi->ch_id) & _HIVE_ISP_CH_ID_MASK;
	curr_fmt_type = (s2mi->fmt_type) & _HIVE_ISP_FMT_TYPE_MASK;

	sh_css_hrt_s2m_send_line2(data, width, data2, width2,
					s2mi->hblank_cycles,
					s2mi->marker_cycles,
					s2mi->two_ppc,
					s2mi->type);

}

void
sh_css_hrt_streaming_to_mipi_end_frame(unsigned int ch_id)
{
	struct streamining_to_mipi_instance *s2mi;
	s2mi = sh_css_hrt_s2m_get_inst(ch_id);

	/* Set global variables that indicate channel_id and format_type */
	curr_ch_id = (s2mi->ch_id) & _HIVE_ISP_CH_ID_MASK;
	curr_fmt_type = (s2mi->fmt_type) & _HIVE_ISP_FMT_TYPE_MASK;

	/* Call existing HRT function */
	sh_css_hrt_s2m_end_frame(s2mi->marker_cycles);

	s2mi->streaming = false;
}

