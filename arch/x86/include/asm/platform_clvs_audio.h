/*
 * platform_clvs_audio.h: CLVS audio platform data header file
 *
 * Copyright (C) 2012 Intel Corporation
 * Author: Jeeja KP <jeeja.kp@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CLVS_AUDIO_H_
#define _PLATFORM_CLVS_AUDIO_H_

#include <linux/sfi.h>
struct clvcs_audio_platform_data {
	/* Intel software platform id*/
	const struct sfi_soft_platform_id *spid;
};

extern void __init *clvs_audio_platform_data(void *info) __attribute__((weak));
#endif
