/*
 * logger_pti.h - logger messages redirection to PTI
 *
 *  Copyright (C) Intel 2010
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * To active logger to PTI messages, configure 'out' parameter
 * of 'logger_pti' module in sysfs with one or more values
 *  # echo "main,system" > /sys/module/logger_pti/parameters/out
 *
 * To active logger to PTI messages from boot, add this
 * commandline parameter to the boot commandline
 *  logger_pti.out=main,system
 *
 * Possible log buffers are : main, system, radio, events
 * See logger.h if others.
 */

#ifndef _LINUX_LOGGER_PTI_H
#define _LINUX_LOGGER_PTI_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pti.h>
#include <linux/string.h>
#include "logger.h"

#ifdef CONFIG_ANDROID_LOGGER_PTI
/*
 * struct queued_entry - a logger entry with maximum space allocated
 *
 * This structure is necessary to retrieve a full logger entry
 */
struct queued_entry {
	union {
		unsigned char buf[LOGGER_ENTRY_MAX_LEN + 1]
			__attribute__((aligned(4)));
		struct logger_entry entry __attribute__((aligned(4)));
	};
};

/*
 * struct pti_reader - a specific reader associate to a pti master/channel
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting.
 */
struct pti_reader {
	struct	logger_reader *reader;
	struct	pti_masterchannel *mc;
	struct	queued_entry *entry;
};

extern int init_pti(struct logger_log *log);
extern void log_write_to_pti(struct logger_log *log);
extern void log_kernel_write_to_pti(struct logger_log *log,
					const char *buf, size_t count);
#else
static inline int init_pti(struct logger_log *log) { return 0; }
static inline int log_kernel_write_to_pti(struct logger_log *log,
				const char *buf, size_t count) { return 0; }
static inline int log_write_to_pti(struct logger_log *log) { return 0; }
#endif

#endif /* _LINUX_LOGGER_PTI_H */
