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
#include <asm/platform_sst_audio.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <sound/asound.h>

static struct sst_platform_data sst_platform_pdata;

static struct sst_dev_stream_map mfld_strm_map[MAX_DEVICES_MFLD] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_HEADSET_MAX_SLOTS, SST_DEV_MAP_IN_USE},
	{1, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_IHF_MAX_SLOTS, SST_DEV_MAP_IN_USE},
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_MAX_SLOTS, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map ctp_strm_map[MAX_DEVICES_CTP] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_PCM_OUT, SST_DEV_MAP_IN_USE},
	{0, 0, SNDRV_PCM_STREAM_CAPTURE, SST_CAPTURE_IN, SST_DEV_MAP_IN_USE},
	{2, 0, SNDRV_PCM_STREAM_PLAYBACK, SST_COMPRESSED_OUT, SST_DEV_MAP_IN_USE},
};

static struct sst_dev_stream_map mrfld_strm_map[MAX_DEVICES_MRFLD] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{0, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA1_IN, SST_DEV_MAP_IN_USE},
	{1, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA0_IN, SST_DEV_MAP_IN_USE},
	{2, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_VOIP_IN, SST_DEV_MAP_IN_USE},
	{3, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 1, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 2, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 3, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 4, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 5, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 6, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 7, SNDRV_PCM_STREAM_PLAYBACK, PIPE_RSVD, SST_DEV_MAP_FREE},
	{0, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_PCM1_OUT, SST_DEV_MAP_IN_USE},
	{2, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_VOIP_OUT, SST_DEV_MAP_IN_USE},
	{3, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 1, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 2, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 3, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 4, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 5, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 6, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{3, 7, SNDRV_PCM_STREAM_CAPTURE, PIPE_RSVD, SST_DEV_MAP_FREE},
	{4, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_AWARE_OUT, SST_DEV_MAP_IN_USE},
	{5, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_VAD_OUT, SST_DEV_MAP_IN_USE},
};

static void set_mfld_platform_config()
{
	sst_platform_pdata.use_strm_map = false;
	sst_platform_pdata.pdev_strm_map = &mfld_strm_map;
	sst_platform_pdata.strm_map_size = MAX_DEVICES_MFLD;
}

static void set_ctp_platform_config()
{
	sst_platform_pdata.use_strm_map = false;
	sst_platform_pdata.pdev_strm_map = &ctp_strm_map;
	sst_platform_pdata.strm_map_size = MAX_DEVICES_CTP;
}

static void set_mrfld_platform_config()
{
	sst_platform_pdata.use_strm_map = true;
	sst_platform_pdata.pdev_strm_map = &mrfld_strm_map;
	sst_platform_pdata.strm_map_size = MAX_DEVICES_MRFLD;
}

static void populate_platform_data()
{
	sst_platform_pdata.spid = &spid;

	if ((INTEL_MID_BOARD(1, PHONE, MFLD)) ||
			(INTEL_MID_BOARD(1, TABLET, MFLD))) {
		set_mfld_platform_config();
	} else if ((INTEL_MID_BOARD(1, PHONE, CLVTP)) ||
			(INTEL_MID_BOARD(1, TABLET, CLVT))) {
		set_ctp_platform_config();
	} else if ((INTEL_MID_BOARD(1, PHONE, MRFL)) ||
			(INTEL_MID_BOARD(1, TABLET, MRFL))) {
		set_mrfld_platform_config();
	}
}

int add_sst_platform_device()
{
	struct platform_device *pdev = NULL;
	int ret;

	pdev = platform_device_alloc("sst-platform", -1);
	if (!pdev) {
		pr_err("failed to allocate audio platform device\n");
		return -EINVAL;
	}

	populate_platform_data();

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
