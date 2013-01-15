/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#include "sh_css.h"
#include "sh_css_sp.h"

#ifdef CONFIG_X86_MRFLD
#define sh_css_sp_has_booted() sh_css_sp_has_initialized()
#define sh_css_enable_cont_capt(enable, stop_copy_preview) \
	sh_css_enable_cont_capt(enable);
#define sh_css_enable_raw_binning(enable) \
	sh_css_enable_raw_reordered(enable)
#define sh_css_update_continuous_frames()

static inline enum sh_css_err sh_css_allocate_continuous_frames(bool enable)
{
	return sh_css_err_unsupported_configuration;
}

#endif
