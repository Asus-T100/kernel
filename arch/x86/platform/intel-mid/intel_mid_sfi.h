/*
 * intel_mid_sfi.h: intel_mid SFI header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _INTEL_MID_SFI_H_
#define _INTEL_MID_SFI_H_

#include <linux/sfi.h>

int handle_sfi_table(char *signature, char *oem_id, char *oem_table_id,
			sfi_table_handler handler);
#endif
