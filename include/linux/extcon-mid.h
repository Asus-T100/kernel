/*
 *  Extcon Switch Mid
 *
 * Copyright (C) 2013 Intel Corp.
 * Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef __EXTCON_MID_H__
#define __EXTCON_MID_H__

#if IS_ENABLED(CONFIG_EXTCON_MID)
void mid_extcon_headset_report(u32 state);
#else
void mid_extcon_headset_report(u32 state)
{
}
#endif

enum {
	HEADSET_PULL_OUT = 0,
	HEADSET_WITH_MIC = 1,
	HEADSET_NO_MIC = 2,
};

#endif

