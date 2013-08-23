/* Release Version: ci_master_byt_20130820_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#ifndef _SH_CSS_ACCELERATE_H_
#define _SH_CSS_ACCELERATE_H_

#include "ia_css.h"
#include "ia_css_acc_types.h"


/* Unload the firmware*/
void
sh_css_acc_unload(const struct ia_css_acc_fw *firmware);

/* Set acceleration parameter to value <val> for isp dmem */
enum ia_css_err
sh_css_acc_set_parameter(struct ia_css_acc_fw *firmware,
			 struct ia_css_data parameters);

/* Set acceleration parameters to value <parameters> for memory <mem> */
enum ia_css_err
sh_css_acc_set_firmware_parameters(struct ia_css_fw_info *firmware,
			 enum ia_css_isp_memories mem,
			 struct ia_css_data parameters);


/* Start the sp, which will start the isp.
   Load the firmware if not yet loaded.
*/
enum ia_css_err
sh_css_acc_start(struct ia_css_acc_fw *firmware);

/* To be called when acceleration has terminated.
*/
void
sh_css_acc_done(struct ia_css_acc_fw *firmware);

/* Wait for the firmware to terminate */
void
sh_css_acc_wait(void);

/* Flag abortion of acceleration */
void sh_css_acc_abort(struct ia_css_acc_fw *firmware);

/* Load the firmware into xmem */
enum ia_css_err
sh_css_acc_load_extension(struct ia_css_fw_info *firmware);

/* Unload the firmware*/
void
sh_css_acc_unload_extension(struct ia_css_fw_info *firmware);


#if 0
/* Get type for argument <num> */
enum sh_css_acc_arg_type
sh_css_argument_type(struct ia_css_acc_fw *firmware, unsigned int num);

/* Set host private data for argument <num> */
enum ia_css_err
sh_css_argument_set_host(struct ia_css_acc_fw *firmware,
			 unsigned num, void *host);

/* Get host private data for argument <num> */
void *
sh_css_argument_get_host(struct ia_css_acc_fw *firmware, unsigned num);

/* Get size for argument <num> */
size_t
sh_css_argument_get_size(struct ia_css_acc_fw *firmware, unsigned num);

/* Get the number of accelerator arguments */
unsigned
sh_css_num_accelerator_args(struct ia_css_acc_fw *firmware);

/* Destabilize argument <num>, i.e. flush it from the cache */
void
sh_css_acc_stabilize(struct ia_css_acc_fw *firmware, unsigned num, bool stable);

/* Check stability of argument <num> */
bool
sh_css_acc_is_stable(struct ia_css_acc_fw *firmware, unsigned num);

#endif

#endif /* _SH_CSS_ACCELERATE_H_ */
