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

#include "sh_css.h"
#include "sh_css_hw.h"
#include "sh_css_rx.h"
#include "sh_css_internal.h"

static unsigned long *css_rx_base_address = CSS_RX_BASE;

static unsigned long
_sh_css_rx_get_register(enum sh_css_mipi_port port, unsigned int reg)
{
	unsigned int bus_addr = (unsigned int)(css_rx_base_address + reg);
#if defined(SYSTEM_hive_isp_css_2400_system)
	if (port == SH_CSS_MIPI_PORT_1LANE)
		bus_addr += hrt_css_receiver_2400_1_lane_port_offset;
	else
		bus_addr += hrt_css_receiver_2400_4_lane_port_offset;
#else
	if (port == SH_CSS_MIPI_PORT_1LANE)
		bus_addr += hrt_css_receiver_ahb_1_lane_port_offset;
	else
		bus_addr += hrt_css_receiver_ahb_4_lane_port_offset;
#endif
	return hrt_master_port_load_32(bus_addr);
}

static void
_sh_css_rx_set_register(enum sh_css_mipi_port port,
			unsigned int reg,
			unsigned long val)
{
	unsigned int bus_addr = (unsigned int)(css_rx_base_address + reg);
#if defined(SYSTEM_hive_isp_css_2400_system)
	if (port == SH_CSS_MIPI_PORT_1LANE)
		bus_addr += hrt_css_receiver_2400_1_lane_port_offset;
	else
		bus_addr += hrt_css_receiver_2400_4_lane_port_offset;
#else
	if (port == SH_CSS_MIPI_PORT_1LANE)
		bus_addr += hrt_css_receiver_ahb_1_lane_port_offset;
	else
		bus_addr += hrt_css_receiver_ahb_4_lane_port_offset;
#endif
	hrt_master_port_store_32(bus_addr, val);
}

static void
_sh_css_rx_set_bits(enum sh_css_mipi_port port,
		    unsigned int reg,
		    unsigned long bits)
{
	unsigned long val;
	val = _sh_css_rx_get_register(port, reg);
	val |= bits;
	_sh_css_rx_set_register(port, reg, val);
}

void
sh_css_rx_enable_all_interrupts(void)
{
	unsigned long bits;

/* MW_R1MRFLD : Disable "no_correction", do not handle */
/* MW_R1MRFLD : Not enable "line_sync, but handled handle" */
#if defined(SYSTEM_hive_isp_css_2400_system)
	bits = _HRT_CSS_RECEIVER_2400_IRQ_OVERRUN_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_ENTRY_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_EXIT_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_HS_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_SYNC_HS_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_CONTROL_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_DOUBLE_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_CORRECTED_BIT |
/*	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_NO_CORRECTION_BIT | */
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_CRC_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_ID_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_SYNC_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_DATA_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_DATA_TIMEOUT_BIT |
	       _HRT_CSS_RECEIVER_2400_IRQ_ERR_ESCAPE_BIT;
/*         _HRT_CSS_RECEIVER_2400_IRQ_ERR_LINE_SYNC_BIT; */

	_sh_css_rx_set_bits(SH_CSS_MIPI_PORT_1LANE,
			    _HRT_CSS_RECEIVER_2400_IRQ_ENABLE_REG_IDX,
			    bits);
#else
	bits = _HRT_CSS_RECEIVER_AHB_IRQ_OVERRUN_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_ENTRY_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_EXIT_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_HS_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_SYNC_HS_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CONTROL_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_DOUBLE_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_CORRECTED_BIT |
/*	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_NO_CORRECTION_BIT | */
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CRC_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ID_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_SYNC_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_DATA_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_DATA_TIMEOUT_BIT |
	       _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ESCAPE_BIT;
/*         _HRT_CSS_RECEIVER_AHB_IRQ_ERR_LINE_SYNC_BIT; */

	_sh_css_rx_set_bits(SH_CSS_MIPI_PORT_1LANE,
			    _HRT_CSS_RECEIVER_AHB_IRQ_ENABLE_REG_IDX, bits);
#endif
}

unsigned int
sh_css_rx_get_interrupt_reg(void)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	return _sh_css_rx_get_register(SH_CSS_MIPI_PORT_1LANE,
			_HRT_CSS_RECEIVER_2400_IRQ_STATUS_REG_IDX);
#else
	return _sh_css_rx_get_register(SH_CSS_MIPI_PORT_1LANE,
			_HRT_CSS_RECEIVER_AHB_IRQ_STATUS_REG_IDX);
#endif
}

void
sh_css_rx_get_interrupt_info(unsigned int *irq_infos)
{
	unsigned long bits, infos = 0;

#if defined(SYSTEM_hive_isp_css_2400_system)
	bits = _sh_css_rx_get_register(SH_CSS_MIPI_PORT_1LANE,
			_HRT_CSS_RECEIVER_2400_IRQ_STATUS_REG_IDX);

/* MW_R1MRFLD : Enable "no_correction, do not handle" */
/* MW_R1MRFLD : Not enable "line_sync, but handled handle" */
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_OVERRUN_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_ENTRY_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_EXIT_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_CORRECTED_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ECC_CORRECTED;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_HS_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_SOT;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_SYNC_HS_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_CONTROL_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_CONTROL;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_DOUBLE_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_CRC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_CRC;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ID_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_SYNC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_DATA_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_DATA_TIMEOUT_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ESCAPE_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC;
	if (bits & (1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_LINE_SYNC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC;
#else
	bits = _sh_css_rx_get_register(SH_CSS_MIPI_PORT_1LANE,
			_HRT_CSS_RECEIVER_AHB_IRQ_STATUS_REG_IDX);

	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_OVERRUN_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_ENTRY_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_EXIT_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_CORRECTED_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ECC_CORRECTED;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_HS_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_SOT;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_SYNC_HS_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CONTROL_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_CONTROL;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_DOUBLE_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CRC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_CRC;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ID_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_SYNC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_DATA_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_DATA_TIMEOUT_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ESCAPE_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC;
	if (bits & (1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_LINE_SYNC_BIT))
		infos |= SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC;
#endif
	*irq_infos = infos;
}

void
sh_css_rx_clear_interrupt_info(unsigned int irq_infos)
{
	unsigned int bits = 0;

#if defined(SYSTEM_hive_isp_css_2400_system)
	if (irq_infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_OVERRUN_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_ENTRY_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_SLEEP_MODE_EXIT_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ECC_CORRECTED)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_CORRECTED_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_SOT)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_HS_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_SOT_SYNC_HS_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_CONTROL)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_CONTROL_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ECC_DOUBLE_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_CRC)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_CRC_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ID_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_SYNC_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_FRAME_DATA_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_DATA_TIMEOUT_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_ESCAPE_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_2400_IRQ_ERR_LINE_SYNC_BIT;

/* MW_R1MRFLD : HW has a "clear by 1" protocol */
	_sh_css_rx_set_bits(SH_CSS_MIPI_PORT_1LANE,
			    _HRT_CSS_RECEIVER_2400_IRQ_ENABLE_REG_IDX, bits);
#else
	if (irq_infos & SH_CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_OVERRUN_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_ENTRY_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_SLEEP_MODE_EXIT_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ECC_CORRECTED)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_CORRECTED_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_SOT)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_HS_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_SOT_SYNC_HS_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_CONTROL)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CONTROL_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ECC_DOUBLE_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_CRC)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_CRC_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ID_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_SYNC_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_FRAME_DATA_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_DATA_TIMEOUT_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_ESCAPE_BIT;
	if (irq_infos & SH_CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
		bits |= 1 << _HRT_CSS_RECEIVER_AHB_IRQ_ERR_LINE_SYNC_BIT;

/* MW_R1MRFLD : HW has a "clear by 1" protocol */
	_sh_css_rx_set_bits(SH_CSS_MIPI_PORT_1LANE,
			    _HRT_CSS_RECEIVER_AHB_IRQ_ENABLE_REG_IDX, bits);
#endif
return;
}

enum sh_css_err
sh_css_input_format_type(enum sh_css_input_format input_format,
			 enum sh_css_mipi_compression compression,
			 unsigned int *fmt_type)
{
	if (compression != SH_CSS_MIPI_COMPRESSION_NONE) {
		switch (input_format) {
		case SH_CSS_INPUT_FORMAT_RAW_6:
			*fmt_type = 6;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_7:
			*fmt_type = 7;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_8:
			*fmt_type = 8;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_10:
			*fmt_type = 10;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_12:
			*fmt_type = 12;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_14:
			*fmt_type = 14;
			break;
		case SH_CSS_INPUT_FORMAT_RAW_16:
			*fmt_type = 16;
			break;
		default:
			return sh_css_err_internal_error;
		}
		return sh_css_success;
	}
	/* This mapping comes from the Arasan CSS function spec
	 * (CSS_func_spec1.08_ahb_sep29_08.pdf).
	 */
	switch (input_format) {
	case SH_CSS_INPUT_FORMAT_RGB_888:
		*fmt_type = 0;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_555:
		*fmt_type = 1;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_444:
		*fmt_type = 2;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_565:
		*fmt_type = 3;
		break;
	case SH_CSS_INPUT_FORMAT_RGB_666:
		*fmt_type = 4;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_8:
		*fmt_type = 5;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_10:
		*fmt_type = 6;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_6:
		*fmt_type = 7;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_7:
		*fmt_type = 8;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_12:
		*fmt_type = 9;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_14:
		*fmt_type = 10;
		break;
	case SH_CSS_INPUT_FORMAT_YUV420_8:
		*fmt_type = 11;
		break;
	case SH_CSS_INPUT_FORMAT_YUV420_10:
		*fmt_type = 12;
		break;
	case SH_CSS_INPUT_FORMAT_YUV422_8:
		*fmt_type = 13;
		break;
	case SH_CSS_INPUT_FORMAT_YUV422_10:
		*fmt_type = 14;
		break;
	case SH_CSS_INPUT_FORMAT_BINARY_8:
		*fmt_type = 15;
		break;
	case SH_CSS_INPUT_FORMAT_YUV420_8_LEGACY:
		*fmt_type = 16;
		break;
	case SH_CSS_INPUT_FORMAT_RAW_16:
		/* This is not specified by Arasan, so we use
		 * 17 for now.
		 */
		*fmt_type = 17;
		break;
	default:
		return sh_css_err_internal_error;
	}
	return sh_css_success;
}

static void
sh_css_rx_set_bits(enum sh_css_mipi_port port, unsigned int reg,
		   unsigned int lsb, unsigned int bits, unsigned int val)
{
	/* prevent writing out of range */
	val &= (1U << bits)-1;
	/* shift into place */
	val <<= lsb;
	_sh_css_rx_set_bits(port, reg, val);
}

static void
sh_css_rx_set_num_lanes(enum sh_css_mipi_port port, unsigned int lanes)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
/*
 * This part of the interface does not exist in 2400
 *
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_2400_CSI2_FUNC_PROG_REG_IDX,
			   _HRT_CSS_RECEIVER_2400_CSI2_NUM_DATA_LANES_IDX,
			   _HRT_CSS_RECEIVER_2400_CSI2_NUM_DATA_LANES_BITS,
			   lanes);
 */
(void)port;
(void)lanes;
#else
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_AHB_CSI2_FUNC_PROG_REG_IDX,
			   _HRT_CSS_RECEIVER_AHB_CSI2_NUM_DATA_LANES_IDX,
			   _HRT_CSS_RECEIVER_AHB_CSI2_NUM_DATA_LANES_BITS,
			   lanes);
#endif
}

static void
sh_css_rx_port_enable(enum sh_css_mipi_port port, bool enable)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	_sh_css_rx_set_register(port,
				_HRT_CSS_RECEIVER_2400_DEVICE_READY_REG_IDX,
				enable);
#else
	_sh_css_rx_set_register(port,
				_HRT_CSS_RECEIVER_AHB_DEVICE_READY_REG_IDX,
				enable);
#endif
}

static void
sh_css_rx_set_timeout(enum sh_css_mipi_port port, unsigned int timeout)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_2400_CSI2_FUNC_PROG_REG_IDX,
			   _HRT_CSS_RECEIVER_2400_CSI2_DATA_TIMEOUT_IDX,
			   _HRT_CSS_RECEIVER_2400_CSI2_DATA_TIMEOUT_BITS,
			   timeout);
#else
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_AHB_CSI2_FUNC_PROG_REG_IDX,
			   _HRT_CSS_RECEIVER_AHB_CSI2_DATA_TIMEOUT_IDX,
			   _HRT_CSS_RECEIVER_AHB_CSI2_DATA_TIMEOUT_BITS,
			   timeout);
#endif
}

static void
sh_css_rx_set_compression(enum sh_css_mipi_port port,
			  enum sh_css_mipi_compression comp)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
/* There are multiple registers, so this is certainly the wrong one */
	unsigned int reg = _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC0_REG0_IDX,
		     comp_val = _HRT_CSS_RECEIVER_2400_PREDICT_NO_COMP;

	if (comp == SH_CSS_MIPI_COMPRESSION_1)
		comp_val = _HRT_CSS_RECEIVER_2400_PREDICT_1;
	if (comp == SH_CSS_MIPI_COMPRESSION_2)
		comp_val = _HRT_CSS_RECEIVER_2400_PREDICT_2;
#else
	unsigned int reg = _HRT_CSS_RECEIVER_AHB_COMP_PREDICT_REG_IDX,
		     comp_val = _HRT_CSS_RECEIVER_AHB_PREDICT_NO_COMP;

	if (comp == SH_CSS_MIPI_COMPRESSION_1)
		comp_val = _HRT_CSS_RECEIVER_AHB_PREDICT_1;
	if (comp == SH_CSS_MIPI_COMPRESSION_2)
		comp_val = _HRT_CSS_RECEIVER_AHB_PREDICT_2;
#endif
	_sh_css_rx_set_register(port, reg, comp_val);
}

static void
sh_css_rx_set_uncomp_size(enum sh_css_mipi_port port, unsigned int size)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
/* There are multiple registers, so this is certainly the wrong one */
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC1_REG0_IDX,
			   _HRT_CSS_RECEIVER_2400_COMP_NUM_BITS_IDX,
			   _HRT_CSS_RECEIVER_2400_COMP_NUM_BITS_BITS,
			   size);
#else
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_AHB_COMP_FORMAT_REG_IDX,
			   _HRT_CSS_RECEIVER_AHB_COMP_NUM_BITS_IDX,
			   _HRT_CSS_RECEIVER_AHB_COMP_NUM_BITS_BITS,
			   size);
#endif
}

static void
sh_css_rx_set_comp_size(enum sh_css_mipi_port port, unsigned int size)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
/* There are multiple registers, so this is certainly the wrong one */
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_2400_COMP_SCHEME_VC2_REG0_IDX,
			   _HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_IDX,
			   _HRT_CSS_RECEIVER_2400_COMP_RAW_BITS_BITS,
			   size);
#else
	sh_css_rx_set_bits(port,
			   _HRT_CSS_RECEIVER_AHB_COMP_FORMAT_REG_IDX,
			   _HRT_CSS_RECEIVER_AHB_COMP_RAW_BITS_IDX,
			   _HRT_CSS_RECEIVER_AHB_COMP_RAW_BITS_BITS,
			   size);
#endif
}

static void
sh_css_rx_set_two_ppc(enum sh_css_mipi_port port, bool enable)
{
#if defined(SYSTEM_hive_isp_css_2400_system)
	_sh_css_rx_set_register(port,
				_HRT_CSS_RECEIVER_2400_TWO_PIXEL_EN_REG_IDX,
				enable);
#else
	_sh_css_rx_set_register(port,
				_HRT_CSS_RECEIVER_AHB_TWO_PIXEL_EN_REG_IDX,
				enable);
#endif
}

void
sh_css_rx_configure(const struct sh_css_mipi_config *config)
{
	/* turn off both ports just in case */
	sh_css_rx_disable();

	/* configure the selected port */
	sh_css_rx_set_num_lanes(config->port, config->num_lanes);
	sh_css_rx_set_timeout(config->port, config->timeout);
	sh_css_rx_set_compression(config->port, config->comp);
	sh_css_rx_set_uncomp_size(config->port, config->uncomp_bpp);
	sh_css_rx_set_comp_size(config->port, config->comp_bpp);
	sh_css_rx_set_two_ppc(config->port, config->two_ppc);

	/* enable the selected port */
	sh_css_rx_port_enable(config->port, true);
}

void
sh_css_rx_disable(void)
{
	sh_css_rx_port_enable(SH_CSS_MIPI_PORT_1LANE, false);
	sh_css_rx_port_enable(SH_CSS_MIPI_PORT_4LANE, false);
}

