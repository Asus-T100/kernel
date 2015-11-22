/*
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include "asustek_boardinfo.h"

typedef int (*idfuntion_handler) (void);

typedef struct{
	char *describe;
	idfuntion_handler func;
	void *meta;
	int value;
}func_handle;

extern int set_boardinfo_tab(func_handle *handler_table);

#define ADD_FUNC(a,b,c,d) a[b].describe = c; a[b].func=&d;

