/*
 * logger_pti.c - logger messages redirection to PTI
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
 * Possible log buffers are : main, system, radio, events, kernel
 * See logger.h if others.
 */

#include <linux/slab.h>
#include "logger_pti.h"

/*
 * init_pti - initialize the pti members with the 'log' passed in
 * argument
 */
int init_pti(struct logger_log *log)
{
	struct pti_reader *pti_reader;
	struct logger_reader *reader;
	int ret = 0;

	pti_reader = kmalloc(sizeof(struct pti_reader), GFP_KERNEL);

	if (!pti_reader) {
		ret = -ENOMEM;
		goto out;
	}

	reader = kmalloc(sizeof(struct logger_reader), GFP_KERNEL);

	if (!reader) {
		kfree(pti_reader);
		ret = -ENOMEM;
		goto out;
	}

	/* entry init */
	pti_reader->entry = kmalloc(sizeof(struct queued_entry), GFP_KERNEL);
	if (!pti_reader->entry) {
		kfree(pti_reader);
		kfree(reader);
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * masterchannel init
	 * request OS channel type : 1, see drivers/misc/pti.c
	 */
	pti_reader->mc = pti_request_masterchannel(1, log->misc.name);
	if (!pti_reader->mc) {
		kfree(pti_reader);
		kfree(reader);
		ret = -ENODEV;
		goto out;
	}

	pti_reader->reader = reader;
	reader->r_off = log->w_off;
	mutex_lock(&log->mutex);
	log->pti_reader = pti_reader;
	mutex_unlock(&log->mutex);

	pr_info("logger_pti: %s, mc : %d %d\n", log->misc.name,
			pti_reader->mc->master, pti_reader->mc->channel);

out:
	if (unlikely(ret)) {
		log->pti_reader = NULL;
		pr_err("logger_pti: failed to init %s\n",
		       log->misc.name);
	}
	return ret;
}

/*
 * log_kernel_write_to_pti - write a kernel log to the pti master/channel
 * Can be called in any context
 *
 */
void log_kernel_write_to_pti(struct logger_log *log,
				const char *buf, size_t count)
{
	struct pti_reader *pti_reader = log->pti_reader;

	/* kernel logs are not buffered */
	if (log->ptienable && pti_reader && pti_reader->mc)
		pti_writedata(pti_reader->mc, (unsigned char *) buf, count);
}

/*
 * log_write_to_pti - write a buffer to the pti master/channel
 * Can be called in any context
 *
 */
void log_write_to_pti(struct logger_log *log)
{
	struct pti_reader *pti_reader;
	struct logger_reader *reader;
	size_t count;

	/* other logs are buffered to add the tag from the header*/
	pti_reader = log->pti_reader;
	if (unlikely(!pti_reader))
		return;

	reader = pti_reader->reader;

	/* get the size of the next entry */
	count = sizeof(struct logger_entry) +
			get_entry_msg_len(log, reader->r_off);

	if (log->ptienable) {
		/* copy exactly one entry from the log */
		do_read_log(log, reader, pti_reader->entry->buf, count);

		pti_writedata(pti_reader->mc,
				pti_reader->entry->entry.msg,
				pti_reader->entry->entry.len);
	} else {
		reader->r_off = logger_offset(reader->r_off + count);
	}
}

/*
 * set_out - 'out' parameter set function from 'logger_pti' module
 *
 * called when writing to 'out' parameter from 'logger_pti' module in sysfs
 */
static int set_out(const char *val, struct kernel_param *kp)
{
	int i;
	const char *log_name;
	struct logger_log *log;
	struct logger_log **log_list;

	log_list = get_log_list();
	for (i = 0; i < LOGGER_LIST_SIZE; i++) {
		log = log_list[i];
		log_name = log->misc.name;
		/* remove "log_" in the log_name string */
		log_name += 4;
		if (strstr(val, log_name))
			log->ptienable = true;
		else
			log->ptienable = false;
	}

	return 0;
}

/*
 * get_out - 'out' parameter get function from 'logger_pti' module
 *
 * called when reading 'out' parameter from 'logger_pti' module in sysfs
 */
static int get_out(char *buffer, struct kernel_param *kp)
{
	int i;
	const char *log_name;
	const char *k = ",";
	struct logger_log *log;
	struct logger_log **log_list;

	log_list = get_log_list();
	for (i = 0; i < LOGGER_LIST_SIZE; i++) {
		if (log_list[i]->ptienable) {
			log = log_list[i];
			log_name = log->misc.name;
			/* remove "log_" in the log_name string */
			log_name += 4;
			strcat(buffer, log_name);
			strcat(buffer, k);
		}
	}
	buffer[strlen(buffer)-1] = '\0';

	return strlen(buffer);
}

module_param_call(out, set_out, get_out, NULL, 0644);
MODULE_PARM_DESC(out, "configure logger to pti [main|events|radio|system|kernel]");

