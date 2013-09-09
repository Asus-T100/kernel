/* Release Version: ci_master_byt_20130905_2200 */
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

#ifndef _IA_CSS_ACC_V1_H_
#define _IA_CSS_ACC_V1_H_

#include "ia_css.h"
#include "ia_css_acc_types.h"

/* Acceleration API.
 * This file is still in CSS 1.0 style, it will be converted to the
 * 2.0 API and naming conventions soon.
 * Do not start any new code on this API until the 2.0 version is
 * available.
 */


/** @brief Unload firmware for acceleration.
 *
 * @param   firmware    Firmware to be unloaded.
 *
 * Unload firmware for acceleration.
 */
void
sh_css_unload_acceleration(struct ia_css_acc_fw *firmware);


/** @brief Set parameter for acceleration.
 *
 * @param   firmware    Firmware of acceleration.
 * @param   val     Parameter value.
 * @return          IA_CSS_SUCCESS or error code upon error.
 *
 * Set acceleration parameter to value <val>.
 * The parameter value is an isp pointer, i.e. allocated in DDR and mapped
 * to the CSS virtual address space.
 */
enum ia_css_err
sh_css_set_acceleration_parameter(struct ia_css_acc_fw *firmware,
                  ia_css_ptr val, size_t size);


/** @brief Start acceleration.
 *
 * @param   firmware    Firmware of acceleration.
 * @return          IA_CSS_SUCCESS or error code upon error.
 *
 * Start acceleration of firmware.
 * Load the firmware if not yet loaded.
 */
enum ia_css_err
sh_css_start_acceleration(struct ia_css_acc_fw *firmware);

/** @brief Signal termination of acceleration.
 *
 * @param   firmware    Firmware of acceleration.
 *
 * To be called when acceleration has terminated.
 */
void
sh_css_acceleration_done(struct ia_css_acc_fw *firmware);

/** @brief Abort current acceleration.
 *
 * @param   firmware    Firmware of acceleration.
 * @param   deadline    Deadline in microseconds.
 *
 * Abort acceleration within <deadline> microseconds
 */
void
sh_css_abort_acceleration(struct ia_css_acc_fw *firmware, unsigned deadline);

#endif
