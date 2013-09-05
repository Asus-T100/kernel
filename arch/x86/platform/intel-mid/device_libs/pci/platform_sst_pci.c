/*
 * platform_sst_pci.c: SST platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:  Dharageswari R <dharageswari.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <linux/intel_mid_dma.h>
#include <asm/intel-mid.h>
#include <asm/platform_sst.h>

#define CTP_SSP_BASE 0xffa23000
#define CTP_DMA_BASE 0xffaf8000
#define CTP_MAX_CONFIG_SIZE 500

#define SST_V1_MAILBOX_RECV	0x800
#define SST_V2_MAILBOX_RECV	0x400

struct sst_platform_info sst_data;

static struct sst_ssp_info ssp_inf = {
	.gpio = {
		.alt_function = LNW_ALT_2,
	},
	.in_use = true,
};

static const struct sst_platform_config_data sst_ctp_pdata = {
	.sst_sram_buff_base = 0xfffc0000,
	.sst_dma_base[0] = CTP_DMA_BASE,
	.sst_dma_base[1] = 0x0,
};

static const struct sst_board_config_data sst_ctp_bdata = {
	.active_ssp_ports = 4,
	.platform_id = 2,/*FIXME: Once the firmware fix is available*/
	.board_id = 1,/*FIXME: Once the firmware fix is available*/
	.ihf_num_chan = 2,
	.osc_clk_freq = 19200000,
	.ssp_platform_data = {
		[SST_SSP_AUDIO] = {
				.ssp_cfg_sst = 1,
				.port_number = 3,
				.is_master = 1,
				.pack_mode = 1,
				.num_slots_per_frame = 2,
				.num_bits_per_slot = 25,
				.active_tx_map = 3,
				.active_rx_map = 3,
				.ssp_frame_format = 3,
				.frame_polarity = 0,
				.serial_bitrate_clk_mode = 0,
				.frame_sync_width = 24,
				.dma_handshake_interface_tx = 5,
				.dma_handshake_interface_rx = 4,
				.ssp_base_add = 0xFFA23000,
		},
		[SST_SSP_MODEM] = {0},
		[SST_SSP_BT] = {0},
		[SST_SSP_FM] = {0},
	},
};

struct sst_info ctp_sst_info = {
	.iram_start = 0,
	.iram_end = 0,
	.iram_use = false,
	.dram_start = 0,
	.dram_end = 0,
	.dram_use = false,
	.imr_start = 0,
	.imr_end = 0,
	.imr_use = false,
	.use_elf = false,
	.max_streams = 5,
	.dma_max_len = (SST_MAX_DMA_LEN * 4),
	.num_probes = 1,
};

static const struct sst_ipc_info ctp_ipc_info = {
	.use_32bit_ops = true,
	.ipc_offset = 0,
	.mbox_recv_off = SST_V1_MAILBOX_RECV,
};

static const struct sst_info mrfld_sst_info = {
	.iram_start = 0,
	.iram_end = 0,
	.iram_use = false,
	.dram_start = 0,
	.dram_end = 0,
	.dram_use = false,
	.imr_start = 0,
	.imr_end = 0,
	.imr_use = false,
	.use_elf = true,
	.max_streams = 23,
	.dma_max_len = SST_MAX_DMA_LEN_MRFLD,
	.num_probes = 16,
};

static const struct sst_ipc_info mrfld_ipc_info = {
	.use_32bit_ops = false,
	.ipc_offset = 0,
	.mbox_recv_off = SST_V2_MAILBOX_RECV,
};

static int set_ctp_sst_config(struct sst_platform_info *sst_info)
{
	unsigned int conf_len;

	ssp_inf.base_add = CTP_SSP_BASE;
	ssp_inf.gpio.i2s_rx_alt = get_gpio_by_name("gpio_i2s3_rx");
	ssp_inf.gpio.i2s_tx_alt = get_gpio_by_name("gpio_i2s3_rx");
	ssp_inf.gpio.i2s_frame = get_gpio_by_name("gpio_i2s3_fs");
	ssp_inf.gpio.i2s_clock = get_gpio_by_name("gpio_i2s3_clk");

	sst_info->ssp_data = &ssp_inf;
	conf_len = sizeof(sst_ctp_pdata) + sizeof(sst_ctp_bdata);
	if (conf_len > CTP_MAX_CONFIG_SIZE)
		return -EINVAL;
	sst_info->pdata = &sst_ctp_pdata;
	sst_info->bdata = &sst_ctp_bdata;
	sst_info->probe_data = &ctp_sst_info;
	sst_info->ipc_info = &ctp_ipc_info;

	return 0;
}

static struct sst_platform_info *get_sst_platform_data(struct pci_dev *pdev)
{
	int ret;
	struct sst_platform_info *sst_pinfo = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_SST_CLV:
		ret = set_ctp_sst_config(&sst_data);
		if (ret < 0)
			return NULL;
		sst_pinfo = &sst_data;
		break;
	case PCI_DEVICE_ID_INTEL_SST_MRFLD:
		sst_data.ssp_data = NULL;
		sst_data.probe_data = &mrfld_sst_info;
		sst_data.pdata = NULL;
		sst_data.bdata = NULL;
		sst_data.ipc_info = &mrfld_ipc_info;
		sst_pinfo = &sst_data;
		break;
	default:
		return NULL;
	}
	return sst_pinfo;
}

static void __devinit sst_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_sst_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_SST_CLV,
							sst_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_SST_MRFLD,
							sst_pci_early_quirks);
