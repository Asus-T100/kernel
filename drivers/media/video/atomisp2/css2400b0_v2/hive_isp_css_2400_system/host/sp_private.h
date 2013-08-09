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

#ifndef __SP_PRIVATE_H_INCLUDED__
#define __SP_PRIVATE_H_INCLUDED__

#include "sp_public.h"

#include "device_access.h"
#ifdef C_RUN
#include <string.h>	/* memcpy() */
#endif

#include "assert_support.h"

STORAGE_CLASS_SP_C void sp_ctrl_store(
	const sp_ID_t		ID,
	const hrt_address	reg,
	const hrt_data		value)
{
assert(ID < N_SP_ID);
assert(SP_CTRL_BASE[ID] != (hrt_address)-1);
	device_store_uint32(SP_CTRL_BASE[ID] + reg*sizeof(hrt_data), value);
return;
}

STORAGE_CLASS_SP_C hrt_data sp_ctrl_load(
	const sp_ID_t		ID,
	const hrt_address	reg)
{
assert(ID < N_SP_ID);
assert(SP_CTRL_BASE[ID] != (hrt_address)-1);
return device_load_uint32(SP_CTRL_BASE[ID] + reg*sizeof(hrt_data));
}

STORAGE_CLASS_SP_C bool sp_ctrl_getbit(
	const sp_ID_t		ID,
	const hrt_address	reg,
	const unsigned int	bit)
{
	hrt_data val = sp_ctrl_load(ID, reg);
return (val & (1UL << bit)) != 0;
}

STORAGE_CLASS_SP_C void sp_ctrl_setbit(
	const sp_ID_t		ID,
	const hrt_address	reg,
	const unsigned int	bit)
{
	hrt_data	data = sp_ctrl_load(ID, reg);
	sp_ctrl_store(ID, reg, (data | (1UL << bit)));
return;
}

STORAGE_CLASS_SP_C void sp_ctrl_clearbit(
	const sp_ID_t		ID,
	const hrt_address	reg,
	const unsigned int	bit)
{
	hrt_data	data = sp_ctrl_load(ID, reg);
	sp_ctrl_store(ID, reg, (data & ~(1UL << bit)));
return;
}

STORAGE_CLASS_SP_C void sp_dmem_store(
	const sp_ID_t		ID,
	hrt_address		addr,
	const void			*data,
	const size_t		size)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
#ifndef C_RUN
	device_store(SP_DMEM_BASE[ID] + addr, data, size);
#else
	memcpy((void *)addr, data, size);
#endif
return;
}

STORAGE_CLASS_SP_C void sp_dmem_load(
	const sp_ID_t		ID,
	const hrt_address	addr,
	void				*data,
	const size_t		size)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
#ifndef C_RUN
	device_load(SP_DMEM_BASE[ID] + addr, data, size);
#else
	memcpy(data, (void *)addr, size);
#endif
return;
}

STORAGE_CLASS_SP_C void sp_dmem_store_uint8(
	const sp_ID_t		ID,
	hrt_address		addr,
	const uint8_t		data)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	device_store_uint8(SP_DMEM_BASE[SP0_ID] + addr, data);
#else
	*(uint8_t *)addr = data;
#endif
return;
}

STORAGE_CLASS_SP_C void sp_dmem_store_uint16(
	const sp_ID_t		ID,
	hrt_address		addr,
	const uint16_t		data)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	device_store_uint16(SP_DMEM_BASE[SP0_ID] + addr, data);
#else
	*(uint16_t *)addr = data;
#endif
return;
}

STORAGE_CLASS_SP_C void sp_dmem_store_uint32(
	const sp_ID_t		ID,
	hrt_address		addr,
	const uint32_t		data)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	device_store_uint32(SP_DMEM_BASE[SP0_ID] + addr, data);
#else
	*(uint32_t *)addr = data;
#endif
return;
}

STORAGE_CLASS_SP_C uint8_t sp_dmem_load_uint8(
	const sp_ID_t		ID,
	const hrt_address	addr)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	return device_load_uint8(SP_DMEM_BASE[SP0_ID] + addr);
#else
	return *(uint8_t *)addr;
#endif
}

STORAGE_CLASS_SP_C uint16_t sp_dmem_load_uint16(
	const sp_ID_t		ID,
	const hrt_address	addr)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	return device_load_uint16(SP_DMEM_BASE[SP0_ID] + addr);
#else
	return *(uint16_t *)addr;
#endif
}

STORAGE_CLASS_SP_C uint32_t sp_dmem_load_uint32(
	const sp_ID_t		ID,
	const hrt_address	addr)
{
assert(ID < N_SP_ID);
assert(SP_DMEM_BASE[ID] != (hrt_address)-1);
	(void)ID;
#ifndef C_RUN
	return device_load_uint32(SP_DMEM_BASE[SP0_ID] + addr);
#else
	return *(uint32_t *)addr;
#endif
}

#endif /* __SP_PRIVATE_H_INCLUDED__ */
