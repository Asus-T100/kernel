/*
 * intel_mid_hsi.h
 *
 * Header for the Intel HSI controller driver.
 *
 * Copyright (C) 2010, 2011 Intel Corporation. All rights reserved.
 *
 * Contact: Jim Stanley <jim.stanley@intel.com>
 * Contact: Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
 * Modified from OMAP SSI driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#ifndef __INTEL_MID_HSI_H__
#define __INTEL_MID_HSI_H__

#define HSI_MID_MAX_CHANNELS	8

/**
 * struct hsi_mid_platform_data - HSI platform specific data for clients
 * @rx_dma_channels: HSI-channel indexed list of RX DMA channel (-1 if no DMA)
 * @rx_sg_entries: HSI-channel indexed list of RX scatter gather entries
 * @rx_fifo_size: HSI-channel indexed list of RX FIFO size in HSI frames
 * @tx_dma_channels: HSI-channel indexed list of TX DMA channel (-1 if no DMA)
 * @tx_sg_entries: HSI-channel indexed list of TX scatter gather entries
 * @tx_fifo_size: HSI-channel indexed list of TX FIFO size in HSI frames
 * @gpio_mdm_rst_out: GPIO index for modem reset input
 * @gpio_mdm_pwr_on: GPIO index for modem power on
 * @gpio_mdm_rst_bbn: GPIO index for modem reset request
 * @gpio_fcdp_rb: GPIO index for modem core dump
 */
struct hsi_mid_platform_data {
	int	rx_dma_channels[HSI_MID_MAX_CHANNELS];
	int	rx_sg_entries[HSI_MID_MAX_CHANNELS];
	int	rx_fifo_sizes[HSI_MID_MAX_CHANNELS];
	int	tx_dma_channels[HSI_MID_MAX_CHANNELS];
	int	tx_sg_entries[HSI_MID_MAX_CHANNELS];
	int	tx_fifo_sizes[HSI_MID_MAX_CHANNELS];

	/* FIXME: the next four entries need to go in a separate client specific
	 *        section */
	int	gpio_mdm_rst_out;
	int	gpio_mdm_pwr_on;
	int	gpio_mdm_rst_bbn;
	int	gpio_fcdp_rb;
};

/* FIXME::Flag declration to avoid register of HSI Driver
 * to enable Standby on CVT Platform
 */
#ifdef CONFIG_BOARD_CTP
/*
 * FIXME:: enable_standby has been define in mfld-pmu file so
 * compilation fails if OSPM is Disabled.
 */
#ifdef CONFIG_ATOM_SOC_POWER
extern unsigned int enable_standby;
#endif
#endif

#endif /* __INTEL_MID_HSI_H__ */
