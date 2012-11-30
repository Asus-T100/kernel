/*
 * Copyright (c) 2009, Kionix, Inc. All Rights Reserved.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MS5607_H__
#define __MS5607_H__

#ifdef __KERNEL__
struct ms5607_platform_data {
	int poll_interval;
	int min_interval;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};
#endif /* __KERNEL__ */

#endif  /* __MS5607_H__ */
