/*
 * (C) Copyright 2011 Intel Corporation
 * Author: Anand Bodas <anand.v.bodas@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifdef CONFIG_MATOM
#define SHARED_CACHE_SIZE       (512*1024)         /* Atom L2 Cache */
#define DATA_CACHE_SIZE         (24*1024)          /* Atom L1 Data Cache */
#define SHARED_CACHE_SIZE_HALF  (SHARED_CACHE_SIZE / 2)
#define DATA_CACHE_SIZE_HALF    (DATA_CACHE_SIZE / 2)
#endif

# undef __i686
# define MEMCPY	_memcpy_ssse3
# define USE_AS_MEMMOVE
# include "ssse3-memcpy6.S"
