/*
 * platform_hsi.c: hsi platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <asm/intel-mid.h>
#include "platform_hsi_modem.h"

void *hsi_modem_platform_data(void *data)
{
	static const char hsi_char_name[]	= "hsi_char";
#if defined(CONFIG_HSI_FFL_TTY)
	static const char hsi_ffl_name[]	= "hsi-ffl";
#elif defined(CONFIG_HSI_DLP)
	static const char hsi_dlp_name[]	= "hsi-dlp";
#endif

#if ((defined CONFIG_HSI_FFL_TTY) && (defined CONFIG_HSI_DLP))
#error "Only define one of HSI_FFL_TTY or HSI_DLP"
#elif (defined(CONFIG_HSI_FFL_TTY) || defined(CONFIG_HSI_DLP))
#define HSI_CLIENT_CNT	2
#else
#define HSI_CLIENT_CNT	1
#endif

	static struct hsi_board_info hsi_info[HSI_CLIENT_CNT] = {
		[0] = {
			.name = hsi_char_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 200000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		},
#if defined(CONFIG_HSI_FFL_TTY)
		[1] = {
			.name = hsi_ffl_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 100000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		}
#elif defined(CONFIG_HSI_DLP)
		[1] = {
			.name = hsi_dlp_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 100000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		}
#endif
	};

#if defined(CONFIG_HSI_FFL_TTY)
	static struct hsi_mid_platform_data mid_info = {
		.tx_dma_channels[0] = -1,
		.tx_dma_channels[1] = 5,
		.tx_dma_channels[2] = -1,
		.tx_dma_channels[3] = -1,
		.tx_dma_channels[4] = -1,
		.tx_dma_channels[5] = -1,
		.tx_dma_channels[6] = -1,
		.tx_dma_channels[7] = -1,
		.tx_sg_entries[0] = 1,
		.tx_sg_entries[1] = 1,
		.tx_sg_entries[2] = 1,
		.tx_sg_entries[3] = 1,
		.tx_sg_entries[4] = 1,
		.tx_sg_entries[5] = 1,
		.tx_sg_entries[6] = 1,
		.tx_sg_entries[7] = 1,
		.tx_fifo_sizes[0] = -1,
		.tx_fifo_sizes[1] = 1024,
		.tx_fifo_sizes[2] = -1,
		.tx_fifo_sizes[3] = -1,
		.tx_fifo_sizes[4] = -1,
		.tx_fifo_sizes[5] = -1,
		.tx_fifo_sizes[6] = -1,
		.tx_fifo_sizes[7] = -1,
		.rx_dma_channels[0] = -1,
		.rx_dma_channels[1] = 1,
		.rx_dma_channels[2] = -1,
		.rx_dma_channels[3] = -1,
		.rx_dma_channels[4] = -1,
		.rx_dma_channels[5] = -1,
		.rx_dma_channels[6] = -1,
		.rx_dma_channels[7] = -1,
		.rx_sg_entries[0] = 1,
		.rx_sg_entries[1] = 1,
		.rx_sg_entries[2] = 1,
		.rx_sg_entries[3] = 1,
		.rx_sg_entries[4] = 1,
		.rx_sg_entries[5] = 1,
		.rx_sg_entries[6] = 1,
		.rx_sg_entries[7] = 1,
		.rx_fifo_sizes[0] = -1,
		.rx_fifo_sizes[1] = 1024,
		.rx_fifo_sizes[2] = -1,
		.rx_fifo_sizes[3] = -1,
		.rx_fifo_sizes[4] = -1,
		.rx_fifo_sizes[5] = -1,
		.rx_fifo_sizes[6] = -1,
		.rx_fifo_sizes[7] = -1,
	};

#elif defined(CONFIG_HSI_DLP)
	static struct hsi_mid_platform_data mid_info = {
		.tx_dma_channels[0] = -1,
		.tx_dma_channels[1] = 0,
		.tx_dma_channels[2] = 1,
		.tx_dma_channels[3] = 2,
		.tx_dma_channels[4] = 3,
		.tx_dma_channels[5] = -1,
		.tx_dma_channels[6] = -1,
		.tx_dma_channels[7] = -1,
		.tx_sg_entries[0] = 1,
		.tx_sg_entries[1] = 1,
		.tx_sg_entries[2] = 64,
		.tx_sg_entries[3] = 64,
		.tx_sg_entries[4] = 64,
		.tx_sg_entries[5] = 1,
		.tx_sg_entries[6] = 1,
		.tx_sg_entries[7] = 1,
		.tx_fifo_sizes[0] = 128,
		.tx_fifo_sizes[1] = 128,
		.tx_fifo_sizes[2] = 256,
		.tx_fifo_sizes[3] = 256,
		.tx_fifo_sizes[4] = 256,
		.tx_fifo_sizes[5] = -1,
		.tx_fifo_sizes[6] = -1,
		.tx_fifo_sizes[7] = -1,
		.rx_dma_channels[0] = -1,
		.rx_dma_channels[1] = 4,
		.rx_dma_channels[2] = 5,
		.rx_dma_channels[3] = 6,
		.rx_dma_channels[4] = 7,
		.rx_dma_channels[5] = -1,
		.rx_dma_channels[6] = -1,
		.rx_dma_channels[7] = -1,
		.rx_sg_entries[0] = 1,
		.rx_sg_entries[1] = 1,
		.rx_sg_entries[2] = 1,
		.rx_sg_entries[3] = 1,
		.rx_sg_entries[4] = 1,
		.rx_sg_entries[5] = 1,
		.rx_sg_entries[6] = 1,
		.rx_sg_entries[7] = 1,
		.rx_fifo_sizes[0] = 128,
		.rx_fifo_sizes[1] = 128,
		.rx_fifo_sizes[2] = 256,
		.rx_fifo_sizes[3] = 256,
		.rx_fifo_sizes[4] = 256,
		.rx_fifo_sizes[5] = -1,
		.rx_fifo_sizes[6] = -1,
		.rx_fifo_sizes[7] = -1,
	};
#else
	static struct hsi_mid_platform_data mid_info = {};
#endif

#if (defined(CONFIG_HSI_FFL_TTY) || defined(CONFIG_HSI_DLP))
	hsi_info[0].platform_data = (void *)&mid_info;
	hsi_info[1].platform_data = (void *)&mid_info;
#endif

	return &hsi_info[0];
}
