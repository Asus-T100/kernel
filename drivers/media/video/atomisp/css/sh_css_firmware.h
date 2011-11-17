/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
*
* Copyright (c) 2010 Intel Corporation. All Rights Reserved.
*
* Copyright (c) 2010 Silicon Hive www.siliconhive.com.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License version
* 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
* 02110-1301, USA.
*
*/

#ifndef _SH_CSS_FIRMWARE_H_
#define _SH_CSS_FIRMWARE_H_

#include "sh_css_binary.h"

/* This is for the firmware loaded from user space */
struct  sh_css_fw_bi_file_h {
	int version;			/* Date in the form YYYYMMDD */
	int binary_nr;			/* Number of binaries */
	unsigned int h_size;		/* sizeof(struct sh_css_fw_bi_file_h) */
};

struct sh_css_fw_bi_h {
	int          id;                /* Binary id */
	unsigned int offset;		/* Offset in firmware file */
	unsigned int size;		/* Size of blob */
	unsigned int text_source;
	unsigned int text_size;
	unsigned int data_source;
	unsigned int data_target;
	unsigned int data_size;
	unsigned int bss_target;
	unsigned int bss_size;
};

struct sh_css_blob_info {
	const unsigned char *blob;
	unsigned int size;
};

extern struct sh_css_sp_fw sh_css_sp_fw;
extern struct sh_css_blob_info sh_css_blob_info[SH_CSS_BINARY_NUM_IDS];

enum sh_css_err
sh_css_load_firmware(const char *fw_data,
		     unsigned int fw_size);

void *sh_css_load_blob(const unsigned char *blob, unsigned size);

#endif /* _SH_CSS_FIRMWARE_H_ */
