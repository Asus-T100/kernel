/*
 * board-vp.c: Intel Merrifield based board (Virtual Platform)
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}
