/*
 * platform_sst_libs.c: SST platform  data initilization file
 *
 * Copyright (C) 2012 Intel Corporation
 * Author: Jeeja KP <jeeja.kp@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <asm/platform_sst_audio.h>
#include <asm/intel-mid.h>
#include <asm/intel_sst_ctp.h>
#include <asm/platform_byt_audio.h>
#include <sound/asound.h>

static struct sst_platform_data sst_platform_pdata;

static struct sst_dev_stream_map mfld_strm_map[MAX_DEVICES_MFLD] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_HEADSET_MAX_SLOTS, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{1, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_IHF_MAX_SLOTS, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_MAX_SLOTS, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map ctp_rhb_strm_map[] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{CTP_RHB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT0, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_RHB_AUD_ASP_DEV, 1, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT1, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_RHB_AUD_COMP_ASP_DEV, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_COMPRESSED_OUT, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_RHB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_RHB_AUD_PROBE_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_PROBE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map ctp_vb_strm_map[] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{CTP_VB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT0, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_VB_AUD_ASP_DEV, 1, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT1, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_VB_AUD_COMP_ASP_DEV, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_COMPRESSED_OUT, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_VB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{CTP_VB_AUD_PROBE_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_PROBE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map merr_bb_strm_map[] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{MERR_BB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT0, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{MERR_BB_AUD_ASP_DEV, 1, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT1, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{MERR_BB_AUD_ASP_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
	{MERR_BB_AUD_PROBE_DEV, 0, SNDRV_PCM_STREAM_CAPTURE, SST_PROBE_IN, SST_TASK_ID_NONE, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map byt_bl_strm_map[] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{BYT_AUD_AIF1, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT0, SST_DEV_MAP_IN_USE},
	{BYT_AUD_AIF1, 1, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT1, SST_DEV_MAP_IN_USE},
	{BYT_AUD_AIF1, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_IN, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map mrfld_strm_map[MAX_DEVICES_MRFLD] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA1_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{1, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA0_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{2, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_VOIP_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{3, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 1, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 2, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 3, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 4, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 5, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 6, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 7, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{0, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_PCM1_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{2, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_VOIP_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{3, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 1, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 2, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 3, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 4, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 5, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 6, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{3, 7, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_TASK_ID_MEDIA, SST_DEV_MAP_FREE},
	{4, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_AWARE_OUT, SST_TASK_ID_AWARE, SST_DEV_MAP_IN_USE},
	{5, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_VAD_OUT, SST_TASK_ID_SBA, SST_DEV_MAP_IN_USE},
};

static struct sst_platform_config_data sst_pdata[] = {
	[PDATA_CTP] = {
		.lpe_sram_buff_base = 0xfffc0000,
		.lpe_dma_base[0] = 0x0,
		.lpe_dma_base[1] = 0x0,
	},
};
static struct sst_ssp_platform_cfg ssp_ctp_data[] = {
	[SST_SSP_AUDIO] = {
		.ssp_cfg_lpe = 1,
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
		.reserved[0] = 0xff,
		.reserved[1] = 0xff,
		.lpe_ssp_base_add = 0xFFA23000,
	},
	[SST_SSP_MODEM] = {0},
	[SST_SSP_BT] = {0},
	[SST_SSP_FM] = {0},
};

static struct sst_board_config_data sst_bdata[] = {
	[BDATA_CTP] = {
		.active_ssp_ports = 4,
		.platform_id = 2,
		.board_id = 1,
		.ihf_num_chan = 2,
		.ssp_clk_freq = 19200000,
	}
};
static void set_mfld_platform_config(void)
{
	sst_platform_pdata.pdata = NULL;
	sst_platform_pdata.bdata = NULL;
	sst_platform_pdata.use_strm_map = false;
	sst_platform_pdata.pdev_strm_map = mfld_strm_map;
	sst_platform_pdata.strm_map_size = MAX_DEVICES_MFLD;
}

static void set_ctp_platform_config(void)
{
	sst_platform_pdata.pdata = &sst_pdata[PDATA_CTP];
	sst_platform_pdata.bdata = &sst_bdata[BDATA_CTP];
	memcpy(sst_platform_pdata.bdata->ssp_platform_data,
					&ssp_ctp_data, sizeof(ssp_ctp_data));
	sst_platform_pdata.use_strm_map = true;

	/* FIX ME: SPID for PRh is not yet available */
#ifdef CONFIG_PRH_TEMP_WA_FOR_SPID
	sst_platform_pdata.pdev_strm_map = &merr_bb_strm_map;
	sst_platform_pdata.strm_map_size = ARRAY_SIZE(merr_bb_strm_map);
#else
	if ((INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO)) ||
	    (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG))) {

		sst_platform_pdata.pdev_strm_map = ctp_vb_strm_map;
		sst_platform_pdata.strm_map_size = ARRAY_SIZE(ctp_vb_strm_map);

	} else if ((INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO)) ||
		   (INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG)) ||
		   (INTEL_MID_BOARD(2, TABLET, CLVT, TBD, PRO)) ||
		   (INTEL_MID_BOARD(2, TABLET, CLVT, TBD, ENG))) {

		sst_platform_pdata.pdev_strm_map = ctp_rhb_strm_map;
		sst_platform_pdata.strm_map_size = ARRAY_SIZE(ctp_rhb_strm_map);
	}
#endif
	pr_debug("audio:ctp:strm_map_size %d\n", sst_platform_pdata.strm_map_size);
}

static void set_byt_platform_config(void)
{
	sst_platform_pdata.pdata = NULL;
	sst_platform_pdata.bdata = NULL;
	sst_platform_pdata.use_strm_map = true;
	sst_platform_pdata.pdev_strm_map = byt_bl_strm_map;
	sst_platform_pdata.strm_map_size =  ARRAY_SIZE(byt_bl_strm_map);
}

static void set_mrfld_platform_config(void)
{
	sst_platform_pdata.pdata = NULL;
	sst_platform_pdata.bdata = NULL;
	sst_platform_pdata.use_strm_map = true;
	sst_platform_pdata.pdev_strm_map = mrfld_strm_map;
	sst_platform_pdata.strm_map_size = MAX_DEVICES_MRFLD;
}

static void  populate_platform_data(void)
{
	sst_platform_pdata.spid = &spid;
	/* FIX ME: SPID for PRh is not yet available */
#ifdef CONFIG_PRH_TEMP_WA_FOR_SPID
	set_ctp_platform_config();
#else
	if ((INTEL_MID_BOARD(1, PHONE, MFLD)) ||
	    (INTEL_MID_BOARD(1, TABLET, MFLD))) {
		set_mfld_platform_config();
	} else if ((INTEL_MID_BOARD(1, PHONE, CLVTP)) ||
		   (INTEL_MID_BOARD(1, TABLET, CLVT))) {
		set_ctp_platform_config();
	} else if ((INTEL_MID_BOARD(1, TABLET, BYT))) {
		set_byt_platform_config();
	} else if ((INTEL_MID_BOARD(1, PHONE, MRFL)) ||
		   (INTEL_MID_BOARD(1, TABLET, MRFL))) {
		set_mrfld_platform_config();
	} else {
		pr_warn("Board not Supported\n");
	}
#endif
}

int add_sst_platform_device(void)
{
	struct platform_device *pdev = NULL;
	int ret;

	populate_platform_data();

	pdev = platform_device_alloc("sst-platform", -1);
	if (!pdev) {
		pr_err("failed to allocate audio platform device\n");
		return -EINVAL;
	}

	ret = platform_device_add_data(pdev, &sst_platform_pdata,
					sizeof(sst_platform_pdata));
	if (ret) {
		pr_err("failed to add sst platform data\n");
		platform_device_put(pdev);
		return  -EINVAL;
	}
	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add audio platform device\n");
		platform_device_put(pdev);
		return  -EINVAL;
	}
	return ret;
}
