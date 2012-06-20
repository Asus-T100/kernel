/*
 * basincove_charger.h: Basincove charger header file
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __BASINCOVE_CHARGER_H_
#define __BASINCOVE_CHARGER_H_

int bc_check_battery_health(void);
int bc_check_battery_status(void);
int bc_get_battery_pack_temp(int *val);

#endif
