/*
 * platform_sst_audio.h:  sst audio platform data header file
 *
 * Copyright (C) 2012 Intel Corporation
 * Author: Jeeja KP <jeeja.kp@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_SST_AUDIO_H_
#define _PLATFORM_SST_AUDIO_H_

#include <linux/sfi.h>
struct sst_platform_data {
	/* Intel software platform id*/
	const struct sfi_soft_platform_id *spid;
};

int add_sst_platform_device();
#endif

