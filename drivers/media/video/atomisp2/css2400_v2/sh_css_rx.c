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

#define __INLINE_INPUT_SYSTEM__
#include "input_system.h"

#include "ia_css.h"
#include "sh_css_rx.h"
#include "sh_css_internal.h"

void
sh_css_rx_enable_all_interrupts(void)
{
	hrt_data	bits = receiver_port_reg_load(RX0_ID,
		MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX);

	bits |= (1U << _HRT_CSS_RECEIVER_IRQ_OVERRUN_BIT) |
#if defined(HAS_RX_VERSION_2)
		(1U << _HRT_CSS_RECEIVER_IRQ_INIT_TIMEOUT_BIT) |
#endif
		(1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_ENTRY_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_EXIT_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_HS_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_SYNC_HS_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_CONTROL_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_DOUBLE_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_CORRECTED_BIT) |
/*		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_NO_CORRECTION_BIT) | */
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_CRC_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_ID_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_SYNC_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_DATA_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_DATA_TIMEOUT_BIT) |
		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_ESCAPE_BIT);
/*		(1U << _HRT_CSS_RECEIVER_IRQ_ERR_LINE_SYNC_BIT); */

	receiver_port_reg_store(RX0_ID,
		MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX, bits);
return;
}

unsigned int sh_css_rx_get_interrupt_reg(void)
{
return receiver_port_reg_load(RX0_ID,
	MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_STATUS_REG_IDX);
}

void
ia_css_rx_get_irq_info(unsigned int *irq_infos)
{
	unsigned long	infos = 0;

	hrt_data	bits = receiver_port_reg_load(RX0_ID,
		MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_STATUS_REG_IDX);

	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_OVERRUN_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_BUFFER_OVERRUN;
#if defined(HAS_RX_VERSION_2)
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_INIT_TIMEOUT_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_INIT_TIMEOUT;
#endif
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_ENTRY_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_EXIT_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_CORRECTED_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ECC_CORRECTED;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_HS_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_SOT;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_SYNC_HS_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_SOT_SYNC;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_CONTROL_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_CONTROL;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_DOUBLE_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_CRC_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_CRC;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_ID_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_SYNC_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_DATA_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_FRAME_DATA;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_DATA_TIMEOUT_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_ESCAPE_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC;
	if (bits & (1U << _HRT_CSS_RECEIVER_IRQ_ERR_LINE_SYNC_BIT))
		infos |= IA_CSS_RX_IRQ_INFO_ERR_LINE_SYNC;

	*irq_infos = infos;
}

void
ia_css_rx_clear_irq_info(unsigned int irq_infos)
{
	hrt_data	bits = receiver_port_reg_load(RX0_ID,
		MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX);

/* MW: Why do we remap the receiver bitmap */
	if (irq_infos & IA_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_OVERRUN_BIT;
#if defined(HAS_RX_VERSION_2)
	if (irq_infos & IA_CSS_RX_IRQ_INFO_INIT_TIMEOUT)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_INIT_TIMEOUT_BIT;
#endif
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_ENTRY_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_SLEEP_MODE_EXIT_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ECC_CORRECTED)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_CORRECTED_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_SOT)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_HS_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_SOT_SYNC_HS_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_CONTROL)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_CONTROL_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_ECC_DOUBLE_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_CRC)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_CRC_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_ID_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_SYNC_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_FRAME_DATA_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_DATA_TIMEOUT_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_ESCAPE_BIT;
	if (irq_infos & IA_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		bits |= 1U << _HRT_CSS_RECEIVER_IRQ_ERR_LINE_SYNC_BIT;

	receiver_port_reg_store(RX0_ID,
		MIPI_PORT1_ID, _HRT_CSS_RECEIVER_IRQ_ENABLE_REG_IDX, bits);
return;
}

enum ia_css_err sh_css_input_format_type(
	enum ia_css_stream_format input_format,
	mipi_predictor_t compression,
	unsigned int *fmt_type)
{
/*
 * Custom (user defined) modes. Used for compressed
 * MIPI transfers
 *
 * Checkpatch thinks the indent before "if" is suspect
 * I think the only suspect part is the missing "else"
 * because of the return.
 */
	if (compression != MIPI_PREDICTOR_NONE) {
		switch (input_format) {
		case IA_CSS_STREAM_FORMAT_RAW_6:
			*fmt_type = 6;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_7:
			*fmt_type = 7;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_8:
			*fmt_type = 8;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_10:
			*fmt_type = 10;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_12:
			*fmt_type = 12;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_14:
			*fmt_type = 14;
			break;
		case IA_CSS_STREAM_FORMAT_RAW_16:
			*fmt_type = 16;
			break;
		default:
			return IA_CSS_ERR_INTERNAL_ERROR;
		}
		return IA_CSS_SUCCESS;
	}
/*
 * This mapping comes from the Arasan CSS function spec
 * (CSS_func_spec1.08_ahb_sep29_08.pdf).
 *
 * MW: For some reason the mapping is not 1-to-1
 */
	switch (input_format) {
	case IA_CSS_STREAM_FORMAT_RGB_888:
		*fmt_type = MIPI_FORMAT_RGB888;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_555:
		*fmt_type = MIPI_FORMAT_RGB555;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_444:
		*fmt_type = MIPI_FORMAT_RGB444;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_565:
		*fmt_type = MIPI_FORMAT_RGB565;
		break;
	case IA_CSS_STREAM_FORMAT_RGB_666:
		*fmt_type = MIPI_FORMAT_RGB666;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_8:
		*fmt_type = MIPI_FORMAT_RAW8;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_10:
		*fmt_type = MIPI_FORMAT_RAW10;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_6:
		*fmt_type = MIPI_FORMAT_RAW6;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_7:
		*fmt_type = MIPI_FORMAT_RAW7;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_12:
		*fmt_type = MIPI_FORMAT_RAW12;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_14:
		*fmt_type = MIPI_FORMAT_RAW14;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_8:
		*fmt_type = MIPI_FORMAT_YUV420_8;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_10:
		*fmt_type = MIPI_FORMAT_YUV420_10;
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_8:
		*fmt_type = MIPI_FORMAT_YUV422_8;
		break;
	case IA_CSS_STREAM_FORMAT_YUV422_10:
		*fmt_type = MIPI_FORMAT_YUV422_10;
		break;
	case IA_CSS_STREAM_FORMAT_BINARY_8:
		*fmt_type = MIPI_FORMAT_BINARY_8;
		break;
	case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:
		*fmt_type = MIPI_FORMAT_YUV420_8_LEGACY;
		break;
	case IA_CSS_STREAM_FORMAT_RAW_16:
		/* This is not specified by Arasan, so we use
		 * 17 for now.
		 */
		*fmt_type = MIPI_FORMAT_RAW16;
		break;
#if defined(HAS_RX_VERSION_2)
	default:
		if (input_format > (enum ia_css_stream_format)N_MIPI_FORMAT)
			return IA_CSS_ERR_INTERNAL_ERROR;
		*fmt_type = input_format;
		break;
#else
	default:
		return IA_CSS_ERR_INTERNAL_ERROR;
#endif
	}
return IA_CSS_SUCCESS;
}

#if defined(HAS_RX_VERSION_1)

/* This is a device function, shouldn't be here */
static void sh_css_rx_set_bits(
	const mipi_port_ID_t	port,
	const unsigned int		reg,
	const unsigned int		lsb,
	const unsigned int		bits,
	const unsigned int		val)
{
	hrt_data	data = receiver_port_reg_load(RX0_ID, port, reg);
/* prevent writing out of range */
	hrt_data	tmp = val & ((1U << bits) - 1);
/* shift into place */
	data |= (tmp << lsb);
	receiver_port_reg_store(RX0_ID, port, reg, data);
return;
}

static void sh_css_rx_set_num_lanes(
	const mipi_port_ID_t	port,
	const unsigned int		lanes)
{
	sh_css_rx_set_bits(port,
		_HRT_CSS_RECEIVER_FUNC_PROG_REG_IDX,
		_HRT_CSS_RECEIVER_AHB_CSI2_NUM_DATA_LANES_IDX,
		_HRT_CSS_RECEIVER_AHB_CSI2_NUM_DATA_LANES_BITS,
		lanes);
return;
}

static void sh_css_rx_set_timeout(
	const mipi_port_ID_t	port,
	const unsigned int		timeout)
{
	sh_css_rx_set_bits(port,
		_HRT_CSS_RECEIVER_FUNC_PROG_REG_IDX,
		_HRT_CSS_RECEIVER_DATA_TIMEOUT_IDX,
		_HRT_CSS_RECEIVER_DATA_TIMEOUT_BITS,
		timeout);
return;
}

static void sh_css_rx_set_compression(
	const mipi_port_ID_t				port,
	const mipi_predictor_t				comp)
{
	unsigned int reg = _HRT_CSS_RECEIVER_COMP_PREDICT_REG_IDX;

assert(comp < N_MIPI_PREDICTOR_TYPES);

	receiver_port_reg_store(RX0_ID, port, reg, comp);
return;
}

static void sh_css_rx_set_uncomp_size(
	const mipi_port_ID_t	port,
	const unsigned int		size)
{
	sh_css_rx_set_bits(port,
		_HRT_CSS_RECEIVER_AHB_COMP_FORMAT_REG_IDX,
		_HRT_CSS_RECEIVER_AHB_COMP_NUM_BITS_IDX,
		_HRT_CSS_RECEIVER_AHB_COMP_NUM_BITS_BITS,
		size);
return;
}

static void sh_css_rx_set_comp_size(
	const mipi_port_ID_t	port,
	const unsigned int		size)
{
	sh_css_rx_set_bits(port,
		_HRT_CSS_RECEIVER_AHB_COMP_FORMAT_REG_IDX,
		_HRT_CSS_RECEIVER_AHB_COMP_RAW_BITS_IDX,
		_HRT_CSS_RECEIVER_AHB_COMP_RAW_BITS_BITS,
		size);
return;
}
#endif /* defined(HAS_RX_VERSION_1) */

void sh_css_rx_configure(
	const rx_cfg_t		*config,
	const enum ia_css_input_mode input_mode)
{
#if defined(HAS_RX_VERSION_2)
	bool	port_enabled[N_MIPI_PORT_ID];
	mipi_port_ID_t	port;

/* AM: Check whether this is a problem with multiple streams. MS: This is the case.*/

/* Must turn off all ports because of the 2ppc setting */
#ifdef THIS_CODE_IS_NO_LONGER_NEEDED_FOR_DUAL_STREAM
    for (port = (mipi_port_ID_t)0; port < N_MIPI_PORT_ID; port++) {
		port_enabled[port] = is_receiver_port_enabled(RX0_ID, port);
		receiver_port_enable(RX0_ID, port, false);
	}
#else
	port = config->port;
	receiver_port_enable(RX0_ID, port, false);
#endif

	port = config->port;

	/* AM: Check whether this is a problem with multiple streams. */
	if (MIPI_PORT_LANES[config->mode][port] != MIPI_0LANE_CFG) {
		receiver_port_reg_store(RX0_ID, port,
			_HRT_CSS_RECEIVER_FUNC_PROG_REG_IDX,
			config->timeout);
		receiver_port_reg_store(RX0_ID, port,
			_HRT_CSS_RECEIVER_2400_INIT_COUNT_REG_IDX,
			config->initcount);
		receiver_port_reg_store(RX0_ID, port,
			_HRT_CSS_RECEIVER_2400_SYNC_COUNT_REG_IDX,
			config->synccount);
		receiver_port_reg_store(RX0_ID, port,
			_HRT_CSS_RECEIVER_2400_RX_COUNT_REG_IDX,
			config->rxcount);
		
		port_enabled[port] = true;
		
		if (input_mode != IA_CSS_INPUT_MODE_BUFFERED_SENSOR) {

		/* MW: A bit of a hack, straight wiring of the capture units,assuming they are linearly enumerated. */
		input_system_sub_system_reg_store(INPUT_SYSTEM0_ID,
			GPREGS_UNIT0_ID, HIVE_ISYS_GPREG_MULTICAST_A_IDX +
			(unsigned int)port, INPUT_SYSTEM_CSI_BACKEND);
		/* MW: Like the integration test example we overwite, the GPREG_MUX register */	
		input_system_sub_system_reg_store(INPUT_SYSTEM0_ID,
			GPREGS_UNIT0_ID, HIVE_ISYS_GPREG_MUX_IDX,
			(input_system_multiplex_t)port);
		} else {
/*
 * AM: A bit of a hack, wiring the input system.
 */
		input_system_sub_system_reg_store(INPUT_SYSTEM0_ID,
			GPREGS_UNIT0_ID, HIVE_ISYS_GPREG_MULTICAST_A_IDX +
			(unsigned int)port, INPUT_SYSTEM_INPUT_BUFFER);
		input_system_sub_system_reg_store(INPUT_SYSTEM0_ID,
			GPREGS_UNIT0_ID, HIVE_ISYS_GPREG_MUX_IDX,
			INPUT_SYSTEM_ACQUISITION_UNIT);
		}
	}
/*
 * The 2ppc is shared for all ports, so we cannot disable->configure->enable individual ports
 */
/* AM: Check whether this is a problem with multiple streams. */
#ifdef THIS_CODE_IS_NO_LONGER_NEEDED_FOR_DUAL_STREAM
	receiver_reg_store(RX0_ID,
		_HRT_CSS_RECEIVER_TWO_PIXEL_EN_REG_IDX, config->is_two_ppc);
	receiver_reg_store(RX0_ID,
		_HRT_CSS_RECEIVER_BE_TWO_PPC_REG_IDX, config->is_two_ppc);
/* enable the selected port(s) */
	for (port = (mipi_port_ID_t)0; port < N_MIPI_PORT_ID; port++) {
		receiver_port_enable(RX0_ID, port, port_enabled[port]);
	}
#else
     receiver_port_enable(RX0_ID, port, true);
#endif
// TODO: JB: need to add the beneath used define to mizuchi
// sh_css_sw_hive_isp_css_2400A0_system_20121224_0125\css\hrt\input_system_defs.h
// #define INPUT_SYSTEM_CSI_RECEIVER_SELECT_BACKENG 0X207
// TODO: need better name for define
//input_system_reg_store(INPUT_SYSTEM0_ID, INPUT_SYSTEM_CSI_RECEIVER_SELECT_BACKENG, 1);
input_system_reg_store(INPUT_SYSTEM0_ID, 0x207, 1);

#elif defined(HAS_RX_VERSION_1)
	mipi_port_ID_t	port = config->port;
    
	(void) input_mode;  //AM: just to satisfy the compiler.

/* turn off all ports just in case */
	sh_css_rx_disable();
	
/* All settings are per port */
	sh_css_rx_set_timeout(port, config->timeout);
/* configure the selected port */
	sh_css_rx_set_num_lanes(port, config->num_lanes);
	sh_css_rx_set_compression(port, config->comp);
	sh_css_rx_set_uncomp_size(port, config->uncomp_bpp);
	sh_css_rx_set_comp_size(port, config->comp_bpp);

	receiver_port_reg_store(RX0_ID, port,
		_HRT_CSS_RECEIVER_TWO_PIXEL_EN_REG_IDX, config->is_two_ppc);

/* enable the selected port */
	receiver_port_reg_store(RX0_ID, port,
		_HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX, true);
#else
#error "sh_css_rx.c: RX version must be one of {RX_VERSION_1, RX_VERSION_2}"
#endif

return;
}

void sh_css_rx_disable(void)
{
	mipi_port_ID_t	port;
	for (port = (mipi_port_ID_t)0; port < N_MIPI_PORT_ID; port++) {
		receiver_port_reg_store(RX0_ID, port,
			_HRT_CSS_RECEIVER_DEVICE_READY_REG_IDX, false);
	}
return;
}

