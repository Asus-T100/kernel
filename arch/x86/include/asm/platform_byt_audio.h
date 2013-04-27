/*
 * platform_byt_audio.h: Baytrail audio platform data header file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Md Abdullah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_BYT_AUDIO_H_
#define _PLATFORM_BYT_AUDIO_H_

#include <linux/sfi.h>

struct byt_audio_platform_data {
	const struct soft_platform_id *spid;
	int codec_gpio;
	int hsdet_gpio;
	int dock_hs_gpio;
};

enum {
	BYT_AUD_AIF1 = 0,
	BYT_AUD_AIF2,
	BYT_AUD_PROBE_DEV,
};

extern void __init *byt_audio_platform_data(void *info) __attribute__((weak));
#endif
