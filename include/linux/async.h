/*
 * async.h: Asynchronous function calls for boot performance
 *
 * (C) Copyright 2009 Intel Corporation
 * Author: Arjan van de Ven <arjan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/types.h>
#include <linux/list.h>

typedef u64 async_cookie_t;
typedef void (async_func_ptr) (void *data, async_cookie_t cookie);

extern async_cookie_t async_schedule(async_func_ptr *ptr, void *data);
extern async_cookie_t async_schedule_domain(async_func_ptr *ptr, void *data,
					    struct list_head *list);
extern void async_synchronize_full(void);
extern void async_synchronize_full_domain(struct list_head *list);
extern void async_synchronize_cookie(async_cookie_t cookie);
extern void async_synchronize_cookie_domain(async_cookie_t cookie,
					    struct list_head *list);

#define __module_init_async(callback, init_level)			\
	static void   __init __##callback##_async(void *unused,		\
						  async_cookie_t cookie) \
	{								\
		callback();						\
	}								\
	static int __init __##callback(void)				\
	{								\
		async_schedule(__##callback##_async, NULL);		\
		return 0;						\
	}								\
	init_level(__##callback);

/* use these macro only if your module does not have other module that
   depends on it. if you have, you will need to use the more complex
   cookie mechanism */
#define module_init_async(callback) __module_init_async(callback, module_init)
#define late_initcall_async(callback) __module_init_async(callback,	\
							  late_initcall)
