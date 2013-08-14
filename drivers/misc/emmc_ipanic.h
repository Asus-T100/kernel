/*
 * drivers/misc/emmc_ipanic.h
 *
 * Copyright (C) 2011 Intel Corp
 * Author: dongxing.zhang@intel.com
 * Author: jun.zhang@intel.com
 * Author: chuansheng.liu@intel.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef _LINUX_EMMC_IPANIC_H
#define _LINUX_EMMC_IPANIC_H

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>

extern int log_buf_copy(char *dest, int idx, int len);
extern void log_buf_clear(void);

#define EMMC_PANIC_PART_NUM 2
#define EMMC_PANIC_BLOCK_NAME "mmcblk0"

#define SECTOR_SIZE_SHIFT (9)

#define IPANIC_LOG_CONSOLE       0
#define IPANIC_LOG_THREADS       1
#define IPANIC_LOG_GBUFFER       2
#ifdef CONFIG_ANDROID_LOGGER
#define IPANIC_LOG_LOGCAT_MAIN   3
#define IPANIC_LOG_LOGCAT_EVENTS 4
#define IPANIC_LOG_LOGCAT_RADIO  5
#define IPANIC_LOG_LOGCAT_SYSTEM 6
#define LOGCAT_BUFF_COUNT        5
#define IPANIC_LOG_MAX           3
#else
#define IPANIC_LOG_MAX           3
#endif
#define IPANIC_LOG_PROC_ENTRY    3

struct mmc_emergency_info {
#define DISK_NAME_LENGTH 20
	/* Default emmc disk name */
	char emmc_disk_name[DISK_NAME_LENGTH];
	/* panic partition number */
	int part_number;
	/* emmc_panic_label */
	char name[DISK_NAME_LENGTH];

	struct block_device *bdev;
	struct device *disk_device;
	struct gendisk *disk;
	struct hd_struct *part;

	/*panic partition start block */
	sector_t start_block;
	/*panic partition block count */
	sector_t block_count;

	int (*init) (void);
	int (*write) (char *, unsigned int);
	int (*read) (char *, unsigned int);
};

struct panic_header {
	u32 magic;
#define PANIC_MAGIC 0xdeadf00d

	u32 version;
#define PHDR_VERSION   0x01

	u32 log_offset[IPANIC_LOG_MAX];
	u32 log_length[IPANIC_LOG_MAX];

	/* For logcat and generic buffer log status */
	size_t log_head[IPANIC_LOG_MAX];
	size_t log_woff[IPANIC_LOG_MAX];
};

struct emmc_ipanic_data {
	struct mmc_emergency_info *emmc;
	struct panic_header curr;
	void *bounce;
	struct proc_dir_entry *ipanic_proc_entry[IPANIC_LOG_PROC_ENTRY];
	unsigned char **ipanic_proc_entry_name;
};

#endif /* _LINUX_EMMC_IPANIC_H */
