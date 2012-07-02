/*
 * sound/sn95031_platform.h -- Platform data for SN95031
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __SND_SN95031_PLATFORM_H
#define __SND_SN95031_PLATFORM_H


#include <linux/sfi.h>
struct sn95031_platform_data {
	/* Intel software platform id*/
	const struct sfi_soft_platform_id *spid;
};

#endif
