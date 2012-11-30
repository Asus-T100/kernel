/*
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 * Copyright (C) 2012 Intel Corporation
 * Author: Jason Chen <jason.cj.chen@intel.com>
 */

#ifndef _INTEL_MID_THERMAL_PROBE_H_
#define _INTEL_MID_THERMAL_PROBE_H_

#define DRIVER_NAME "mid_thermal_probe"

struct thermal_probe_platform_data {
	int adc_ch;
};

#define EVENT_THERM_PULL_OUT	0
#define EVENT_THERM_PLUG_IN	1

int mid_thermal_probe_notifier_call_chain(unsigned long val, void *v);

#endif
