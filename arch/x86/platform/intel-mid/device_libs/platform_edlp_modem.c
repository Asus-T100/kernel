/*
 * platform_edlp_modem.c: hsi EDLP platform data initilization file
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
#include "platform_edlp_modem.h"

#define HSI_CLIENT_CNT	2

void *edlp_modem_platform_data(void *data)
{
	int is_v2;
	static const char hsi_char_name[]	= "hsi_char";
	static const char hsi_dlp_name[]	= "hsi-dlp";

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
	};

	/* HSI IP v2 ? */
	is_v2 = (SPID_PLATFORM_ID(INTEL, MRFL, PHONE) ||
			 SPID_PLATFORM_ID(INTEL, MRFL, TABLET));

	if (is_v2) {
		static struct hsi_mid_platform_data mid_info_v2 = {
		  /* TX/RX DMA channels mapping */
		  .tx_dma_channels = {  0,  1,  2,  3,  4,  -1, -1,  -1},
		  .rx_dma_channels = {  5,  6,  7,  8,  9,  -1, -1,  -1},

		  /* TX/RX FIFOs sizes */
		  .tx_fifo_sizes = { 256, 256, 256, 256, 256,  -1,  -1,  -1},
		  .rx_fifo_sizes = { 256, 256, 256, 256, 256,  -1,  -1,  -1},

		  /* RX/TX Threshold */
		  .tx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},
		  .rx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},

		  /* TX/RX SG entries count */
		  .tx_sg_entries = {  1,  1,  64,  64,  64,  1,  1,  1},
		  .rx_sg_entries = {  1,  1,   1,   1,   1,  1,  1,  1}
		};

		hsi_info[0].platform_data = (void *)&mid_info_v2;
		hsi_info[1].platform_data = (void *)&mid_info_v2;
	} else {
		static struct hsi_mid_platform_data mid_info_v1 = {
		  /* TX/RX DMA channels mapping */
		  .tx_dma_channels = {  -1,  0,  1,  2,  3,  -1, -1,  -1},
		  .rx_dma_channels = {  -1,  4,  5,  6,  7,  -1, -1,  -1},

		  /* TX/RX FIFOs sizes */
		  .tx_fifo_sizes = { 128, 128, 256, 256, 256,  -1,  -1,  -1},
		  .rx_fifo_sizes = { 128, 128, 256, 256, 256,  -1,  -1,  -1},

		  /* RX/TX Threshold */
		  .tx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},
		  .rx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},

		  /* TX/RX SG entries count */
		  .tx_sg_entries = {  1,  1,  64,  64,  64,  1,  1,  1},
		  .rx_sg_entries = {  1,  1,   1,   1,   1,  1,  1,  1}
		};

		hsi_info[0].platform_data = (void *)&mid_info_v1;
		hsi_info[1].platform_data = (void *)&mid_info_v1;
	}

	pr_info("HSI EDLP platform data setup\n");
	return &hsi_info[0];
}
