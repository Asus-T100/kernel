/* include/linux/logger.h
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Author: Robert Love <rlove@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_LOGGER_H
#define _LINUX_LOGGER_H

#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>

/*
 * The userspace structure for version 1 of the logger_entry ABI.
 * This structure is returned to userspace unless the caller requests
 * an upgrade to a newer ABI version.
 */
struct user_logger_entry_compat {
        __u16           len;    /* length of the payload */
        __u16           __pad;  /* no matter what, we get 2 bytes of padding */
        __s32           pid;    /* generating process's pid */
        __s32           tid;    /* generating process's tid */
        __s32           sec;    /* seconds since Epoch */
        __s32           nsec;   /* nanoseconds */
        char            msg[0]; /* the entry's payload */
};

/*
 * struct logger_log - represents a specific log, such as 'main' or 'radio'
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * mutex 'mutex'.
 */
struct logger_log {
	unsigned char 		*buffer;/* the ring buffer itself */
	struct miscdevice	misc;	/* misc device representing the log */
	wait_queue_head_t	wq;	/* wait queue for readers */
	struct list_head	readers; /* this log's readers */
	struct mutex		mutex;	/* mutex protecting buffer */
	size_t			w_off;	/* current write head offset */
	size_t			head;	/* new readers start here */
	size_t			size;	/* size of the log */
#ifdef CONFIG_ANDROID_LOGGER_PTI
	bool			ptienable;
	struct pti_reader	*pti_reader;
#endif
};

/*
 * struct logger_reader - a logging device open for reading
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->mutex.
 */
struct logger_reader {
	struct logger_log	*log;	/* associated log */
	struct list_head	list;	/* entry in logger_log's list */
	size_t			r_off;	/* current read head offset */
	bool			r_all;	/* reader can read all entries */
	int			r_ver;	/* reader ABI version */
};


/*
 * The structure for version 2 of the logger_entry ABI.
 * This structure is returned to userspace if ioctl(LOGGER_SET_VERSION)
 * is called with version >= 2
 */
struct logger_entry {
	__u16		len;		/* length of the payload */
	__u16		hdr_size;	/* sizeof(struct logger_entry_v2) */
	__s32		pid;		/* generating process's pid */
	__s32		tid;		/* generating process's tid */
	__s32		sec;		/* seconds since Epoch */
	__s32		nsec;		/* nanoseconds */
	uid_t		euid;		/* effective UID of logger */
	char		msg[0];		/* the entry's payload */
};

void do_read_log(struct logger_log *log,
			struct logger_reader *reader,
			char *buf,
			size_t count);
__u32 get_entry_msg_len(struct logger_log *log, size_t off);
struct logger_log *get_log_from_minor(int minor);
struct logger_log **get_log_list(void);

/* logger_offset - returns index 'n' into the log via (optimized) modulus */
#define logger_offset(n)	((n) & (log->size - 1))

#define LOGGER_LOG_RADIO	"log_radio"	/* radio-related messages */
#define LOGGER_LOG_EVENTS	"log_events"	/* system/hardware events */
#define LOGGER_LOG_SYSTEM	"log_system"	/* system/framework messages */
#define LOGGER_LOG_KERNEL	"log_kernel"	/* kernel */
#define LOGGER_LOG_KERNEL_BOT	"log_kernel_bottom" /* kernel bottom */
#define LOGGER_LOG_MAIN		"log_main"	/* everything else */

#define LOGGER_LIST_SIZE 5

#define LOGGER_ENTRY_MAX_LEN		(5*1024)
#define LOGGER_ENTRY_MAX_PAYLOAD	4076

#define __LOGGERIO	0xAE

#define LOGGER_GET_LOG_BUF_SIZE		_IO(__LOGGERIO, 1) /* size of log */
#define LOGGER_GET_LOG_LEN		_IO(__LOGGERIO, 2) /* used log len */
#define LOGGER_GET_NEXT_ENTRY_LEN	_IO(__LOGGERIO, 3) /* next entry len */
#define LOGGER_FLUSH_LOG		_IO(__LOGGERIO, 4) /* flush log */
#define LOGGER_GET_VERSION		_IO(__LOGGERIO, 5) /* abi version */
#define LOGGER_SET_VERSION		_IO(__LOGGERIO, 6) /* abi version */

#endif /* _LINUX_LOGGER_H */
