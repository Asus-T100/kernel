/*
 * lite-on ltr301 digital light sensor driver
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
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef _LTR301ALS_H
#define _LTR301ALS_H

/**
 * struct ltr301als_platform_data - ltr301 light sensor platform data
 * @window_opacity: opacity of window covering sensor, 100 is totally clear
 */

struct ltr301als_platform_data {
	int window_opacity;
};

#endif /* _LTR301ALS_H */
