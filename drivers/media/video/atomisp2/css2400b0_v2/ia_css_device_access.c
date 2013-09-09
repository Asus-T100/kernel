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

#include "ia_css_device_access.h"
#include "device_access.h"
#include "assert_support.h"

static struct ia_css_hw_access_env my_env;

void
ia_css_device_access_init(const struct ia_css_hw_access_env *env)
{
	assert(env != NULL);

	my_env = *env;
}

uint8_t
device_load_uint8(const hrt_address addr)
{
	return my_env.load_8(addr);
}

uint16_t
device_load_uint16(const hrt_address addr)
{
	return my_env.load_16(addr);
}

uint32_t
device_load_uint32(const hrt_address addr)
{
	return my_env.load_32(addr);
}

uint64_t
device_load_uint64(const hrt_address addr)
{
	assert(0);

	(void)addr;
	return 0;
}

void
device_store_uint8(const hrt_address addr, const uint8_t data)
{
	my_env.store_8(addr, data);
}

void
device_store_uint16(const hrt_address addr, const uint16_t data)
{
	my_env.store_16(addr, data);
}

void
device_store_uint32(const hrt_address addr, const uint32_t data)
{
	my_env.store_32(addr, data);
}

void
device_store_uint64(const hrt_address addr, const uint64_t data)
{
	assert(0);

	(void)addr;
	(void)data;
}

void
device_load(const hrt_address addr, void *data, const size_t size)
{
	my_env.load(addr, data, size);
}

void
device_store(const hrt_address addr, const void *data, const size_t size)
{
	my_env.store(addr, data, size);
}
