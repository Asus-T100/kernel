/*
 * drivers/misc/emmc_ipanic.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/mmc/host.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/preempt.h>
#include <linux/pci.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include "emmc_ipanic.h"

static unsigned int ipanic_part_number;
module_param(ipanic_part_number, uint, 0);
MODULE_PARM_DESC(ipanic_part_number, "IPanic dump partition on mmcblk0");

#ifdef CONFIG_ANDROID_LOGGER
#include "../staging/android/logger.h"
static unsigned char *logcat_name[LOGCAT_BUFF_COUNT] = {
	LOGGER_LOG_MAIN,
	LOGGER_LOG_EVENTS,
	LOGGER_LOG_RADIO,
	LOGGER_LOG_SYSTEM
};
#endif

/*
 * The part_number will be filled in driver init.
 */

static struct mmc_emergency_info emmc_info = {
	.init = mmc_emergency_init,
	.write = mmc_emergency_write,
	.emmc_disk_name = "mmcblk0",
	.part_number = 0,
	.name = "emmc_ipanic",
	.disk_device = NULL
};

static unsigned char *ipanic_proc_entry_name[IPANIC_LOG_PROC_ENTRY] = {
	"emmc_ipanic_console",
	"emmc_ipanic_threads"
};

static int in_panic;
static struct emmc_ipanic_data drv_ctx;
static struct work_struct proc_removal_work;
static int is_found_panic_par;
static u32 disable_emmc_ipanic;
static int log_offset[IPANIC_LOG_MAX];
static int log_len[IPANIC_LOG_MAX];	/* sector count */
static int log_size[IPANIC_LOG_MAX];	/* byte count */
static size_t log_head[IPANIC_LOG_MAX];
static size_t log_woff[IPANIC_LOG_MAX];
static unsigned char last_chunk_buf[SECTOR_SIZE];
static int last_chunk_buf_len;
static DEFINE_MUTEX(drv_mutex);
static void (*func_stream_emmc) (void);

#ifdef CONFIG_ANDROID_LOGGER
static int emmc_ipanic_writeflashpage(struct mmc_emergency_info *emmc,
				      loff_t to, const u_char *buf);
static void emmc_ipanic_flush_lastchunk_emmc(loff_t to,
					     int *size_written,
					     int *sector_written);
static struct logger_log *get_logcat_log(unsigned char *buf_name)
{
	struct logger_log *log = NULL;
	int i;
	struct logger_log **log_list = get_log_list();

	/* Find the log thought the buffer name */
	for (i = 0; i < LOGGER_LIST_SIZE; i++)
		if (!strcmp(log_list[i]->misc.name, buf_name)) {
			log = log_list[i];
			break;
		}
	if (!log) {
		printk(KERN_EMERG "Invalid logcat buffer name: %s\n", buf_name);
		return NULL;
	}
	return log;
}

static size_t get_logcat_head(struct logger_log *log)
{
	if (log)
		return log->head;
	return 0;
}

static size_t get_logcat_woff(struct logger_log *log)
{
	if (log)
		return log->w_off;
	return 0;
}

static void *get_logcat_buffer(struct logger_log *log)
{
	if (log)
		return log->buffer;
	return NULL;
}

static size_t get_logcat_size(struct logger_log *log)
{
	if (log)
		return log->size;
	return 0;
}

static void lock_logcat_mutex(struct logger_log *log)
{
	if (log)
		mutex_lock(&log->mutex);
}

static void unlock_logcat_mutex(struct logger_log *log)
{
	if (log)
		mutex_unlock(&log->mutex);
}

static void set_logcat_head(struct logger_log *log, size_t head)
{
	if (log)
		log->head = head;
}

static void set_logcat_woff(struct logger_log *log, size_t woff)
{
	if (log)
		log->w_off = woff;
}

static int emmc_ipanic_write_logcat(struct mmc_emergency_info *emmc,
				    void *logcat, unsigned int off,
				    int *actual_size)
{
	int rc, block_shift = 0;
	unsigned char *buf;
	size_t index = 0, log_size;

	buf = get_logcat_buffer(logcat);
	if (!buf) {
		printk(KERN_EMERG "Invalid logcat buffer pointer(%u)\n",
					(unsigned int)logcat);
		return 0;
	}

	log_size = get_logcat_size(logcat);
	while (index < log_size) {
		size_t size_copy = log_size - index;
		if (size_copy < SECTOR_SIZE) {
			memcpy(last_chunk_buf, buf + index, size_copy);
			last_chunk_buf_len = size_copy;
			break;
		}
		rc = emmc_ipanic_writeflashpage(emmc, off + block_shift,
						buf + index);
		if (rc <= 0) {
			printk(KERN_EMERG
			       "%s: Flash write failed (%d)\n", __func__, rc);
			return 0;
		}
		index += rc;
		block_shift++;
	}
	*actual_size = index;

	return block_shift;
}

static void emmc_ipanic_write_logcatbuf(struct mmc_emergency_info *emmc,
					int log, unsigned char *logcat_buf_name)
{
	void *logcat = get_logcat_log(logcat_buf_name);
	if (!logcat) {
			printk(KERN_EMERG "Invalid log buffer name(%s)\n",
				logcat_buf_name);
			return;
	}
	log_offset[log] = log_offset[log - 1] + log_len[log - 1];
	log_len[log] = emmc_ipanic_write_logcat(emmc, logcat,
						log_offset[log],
						&log_size[log]);
	if (log_size[log] < 0) {
		printk(KERN_EMERG
		       "Error writing console to panic log! (%d)\n",
		       log_len[log]);
		log_size[log] = 0;
		log_len[log] = 0;
	}
	/* flush last chunk buffer */
	emmc_ipanic_flush_lastchunk_emmc(log_offset[log] +
					 log_len[log],
					 &log_size[log], &log_len[log]);
	log_head[log] = get_logcat_head(logcat);
	log_woff[log] = get_logcat_woff(logcat);
}
#endif

static void emmc_panic_erase(unsigned char *buffer, Sector *sect)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc = ctx->emmc;
	unsigned char *read_buf_ptr = buffer;
	Sector new_sect;
	int rc;

	if (!read_buf_ptr || !sect) {
		sect = &new_sect;
		if (!emmc->bdev) {
			printk(KERN_ERR "%s:invalid emmc block device\n",
			       __func__);
			goto out;
		}
		/* make sure the block device is open rw */
		rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE, emmc_panic_erase);
		if (rc < 0) {
			printk(KERN_ERR "%s: blk_dev_get failed!\n", __func__);
			goto out;
		}

		/*read panic header */
		read_buf_ptr =
		    read_dev_sector(emmc->bdev, emmc->start_block, sect);
		if (!read_buf_ptr) {
			printk(KERN_ERR "%s: read sector error(%llu)!\n",
			       __func__, emmc->start_block);
			goto out;
		}
	}

	/*write all zero to panic header */
	lock_page(sect->v);
	memset(read_buf_ptr, 0, sizeof(struct panic_header));
	set_page_dirty(sect->v);
	unlock_page(sect->v);
	sync_blockdev(emmc->bdev);

	if (!read_buf_ptr)
		put_dev_sector(*sect);
out:
	memset(&ctx->curr, 0, sizeof(struct panic_header));
	return;
}

static int emmc_ipanic_proc_read(char *buffer, char **start, off_t offset,
				 int count, int *peof, void *dat)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;
	unsigned char *read_buf_ptr;
	Sector sect;
	size_t file_length;
	off_t file_offset;
	unsigned int sector_no;
	off_t sector_offset;
	int rc;
	int log;

	if (!ctx) {
		printk(KERN_ERR "%s:invalid panic handler\n", __func__);
		return 0;
	}
	emmc = ctx->emmc;
	if (!emmc) {
		printk(KERN_ERR "%s:invalid emmc infomation\n", __func__);
		return 0;
	}
	if (!emmc->bdev) {
		printk(KERN_ERR "%s:invalid emmc block device\n", __func__);
		return 0;
	}

	if (!count)
		return 0;

	mutex_lock(&drv_mutex);

	log = (int)dat;
	if (log < 0 || log >= IPANIC_LOG_MAX) {
		pr_err("Bad dat (%d)\n", (int)dat);
		mutex_unlock(&drv_mutex);
		return -EINVAL;
	}

	file_length = ctx->curr.log_length[log];
	file_offset = ctx->curr.log_offset[log];

	if ((offset + count) > file_length) {
		mutex_unlock(&drv_mutex);
		return 0;
	}

	/* We only support reading a maximum of a flash page */
	if (count > SECTOR_SIZE)
		count = SECTOR_SIZE;

	sector_no = (file_offset + offset) >> SECTOR_SIZE_SHIFT;
	sector_offset = (file_offset + offset) & (SECTOR_SIZE - 1);
	if (sector_no >= emmc->block_count) {
		printk(KERN_EMERG "%s: reading an invalid address\n", __func__);
		mutex_unlock(&drv_mutex);
		return -EINVAL;
	}

	/* make sure the block device is open rw */
	rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE, emmc_ipanic_proc_read);
	if (rc < 0) {
		printk(KERN_ERR "%s: blk_dev_get failed!\n", __func__);
		mutex_unlock(&drv_mutex);
		return 0;
	}

	read_buf_ptr =
	    read_dev_sector(emmc->bdev, sector_no + emmc->start_block, &sect);
	if (!read_buf_ptr) {
		count = -EINVAL;
		mutex_unlock(&drv_mutex);
		return count;
	}

	if (sector_offset)
		count -= sector_offset;
	memcpy(buffer, read_buf_ptr + sector_offset, count);
	*start = (char *)count;

	if ((offset + count) == file_length)
		*peof = 1;

	put_dev_sector(sect);
	mutex_unlock(&drv_mutex);

	return count;
}

static void emmc_ipanic_remove_proc_work(struct work_struct *work)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	int log;

	mutex_lock(&drv_mutex);
	emmc_panic_erase(NULL, NULL);

	for (log = 0; log < IPANIC_LOG_PROC_ENTRY; log++) {
		if (ctx->ipanic_proc_entry[log]) {
			remove_proc_entry(ctx->ipanic_proc_entry_name
					  [log], NULL);
			ctx->ipanic_proc_entry[log] = NULL;
		}
	}
	mutex_unlock(&drv_mutex);
}

static int emmc_ipanic_proc_write(struct file *file, const char __user *buffer,
				  unsigned long count, void *data)
{
	schedule_work(&proc_removal_work);
	return count;
}

static void emmc_panic_notify_add(void)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;
	unsigned char *read_buf_ptr;
	Sector sect;
	int rc, log;
	int proc_entry_created = 0;

	if (!ctx) {
		printk(KERN_ERR "%s:invalid panic handler\n", __func__);
		return;
	}

	emmc = ctx->emmc;
	if (!emmc) {
		printk(KERN_ERR "%s:invalid emmc infomation\n", __func__);
		goto out_err;
	}
#ifdef CONFIG_EMMC_IPANIC_PLABEL
	if (strcmp(emmc->name, CONFIG_EMMC_IPANIC_PLABEL))
		goto out_err;
#endif

	if (!emmc->bdev) {
		printk(KERN_ERR "%s:invalid emmc block device\n", __func__);
		goto out_err;
	}

	/* make sure the block device is open rw */
	rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE, emmc_panic_notify_add);
	if (rc < 0) {
		printk(KERN_ERR "%s: blk_dev_get failed!\n", __func__);
		goto out_err;
	}

	/*read panic header */
	read_buf_ptr = read_dev_sector(emmc->bdev, emmc->start_block, &sect);
	if (!read_buf_ptr) {
		printk(KERN_ERR "%s: read sector error(%llu)!\n", __func__,
		       emmc->start_block);
		return;
	}

	memcpy(&ctx->curr, read_buf_ptr, sizeof(struct panic_header));

	printk(KERN_INFO "%s: Bound to emmc partition '%s'\n",
	       __func__, emmc->name);

	if (ctx->curr.magic != PANIC_MAGIC) {
		emmc_panic_erase(read_buf_ptr, &sect);
		goto put_sector;
	}

	printk(KERN_INFO "%s: Data available in panic partition\n", __func__);

	if (ctx->curr.version != PHDR_VERSION) {
		printk(KERN_ERR "%s: Version mismatch (%d != %d)\n",
		       __func__, ctx->curr.version, PHDR_VERSION);
		emmc_panic_erase(read_buf_ptr, &sect);
		goto put_sector;
	}

	for (log = 0; log < IPANIC_LOG_MAX; log++) {
		unsigned char *ptr;
		void *logcat, *buf;
		int size, offset;
		Sector sect2;
		printk(KERN_INFO "%s: log file %u(%u, %u)\n", __func__,
		       log + 1, ctx->curr.log_offset[log],
		       ctx->curr.log_length[log]);

		/* Skip empty file. */
		if (ctx->curr.log_length[log] == 0)
			continue;

		/* Only create proc entry for console and threads log. */
		if (log < IPANIC_LOG_PROC_ENTRY) {
			ctx->ipanic_proc_entry[log] =
			    create_proc_entry(ctx->ipanic_proc_entry_name
					      [log], S_IFREG | S_IRUGO, NULL);

			if (!ctx->ipanic_proc_entry[log])
				printk(KERN_ERR
				       "%s: failed creating proc file\n",
				       __func__);
			else {
				ctx->ipanic_proc_entry[log]->read_proc =
				    emmc_ipanic_proc_read;
				ctx->ipanic_proc_entry[log]->write_proc =
				    emmc_ipanic_proc_write;
				ctx->ipanic_proc_entry[log]->size =
				    ctx->curr.log_length[log];
				ctx->ipanic_proc_entry[log]->data = (void *)log;
				proc_entry_created = 1;
			}
			continue;
		}

#ifdef CONFIG_ANDROID_LOGGER
		/* For logcat log, copy back to logcat buffer. */
		logcat =
		    get_logcat_log(logcat_name[log - IPANIC_LOG_PROC_ENTRY]);
		if (!logcat) {
			printk(KERN_EMERG "Invalid log buffer(%d)\n", log);
			continue;
		}
		buf = get_logcat_buffer(logcat);
		if (!buf) {
			printk(KERN_EMERG "Invalid logcat pointer(%u)\n",
						(unsigned int)logcat);
			continue;
		}

		offset = emmc->start_block +
			(ctx->curr.log_offset[log] >> SECTOR_SIZE_SHIFT);
		lock_logcat_mutex(logcat);
		for (size = 0; size < ctx->curr.log_length[log];
		     size += SECTOR_SIZE) {
			int write_size = SECTOR_SIZE;
			int sec_count = size >> SECTOR_SIZE_SHIFT;

			ptr = read_dev_sector(emmc->bdev, offset + sec_count,
					      &sect2);
			if (!ptr) {
				printk(KERN_ERR
				       "%s: read sector error(%u)!\n",
				       __func__, offset + sec_count);
				unlock_logcat_mutex(logcat);
				goto put_sector;
			}
			if (ctx->curr.log_length[log] - size < SECTOR_SIZE)
				write_size = ctx->curr.log_length[log] - size;
			memcpy(buf + size, ptr, write_size);
		}
		set_logcat_head(logcat, ctx->curr.log_head[log]);
		set_logcat_woff(logcat, ctx->curr.log_woff[log]);
		unlock_logcat_mutex(logcat);
#endif
	}

	if (!proc_entry_created)
		emmc_panic_erase(read_buf_ptr, &sect);

put_sector:
	put_dev_sector(sect);
	return;
out_err:
	ctx->emmc = NULL;
}

static void emmc_panic_notify_remove(void)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;

	ctx->emmc = NULL;
}

static int emmc_ipanic_writeflashpage(struct mmc_emergency_info *emmc,
				      loff_t to, const u_char *buf)
{
	int rc;
	size_t wlen = SECTOR_SIZE;

	if (to >= emmc->start_block + emmc->block_count) {
		printk(KERN_EMERG "%s: panic partition is full.\n", __func__);
		return 0;
	}

	rc = emmc->write((char *)buf, (unsigned int)to);
	if (rc) {
		printk(KERN_EMERG
		       "%s: Error writing data to flash (%d)\n", __func__, rc);
		return rc;
	}

	return wlen;
}

/*
 * Writes the contents of the console to the specified offset in flash.
 * Returns number of bytes written
 */
static int emmc_ipanic_write_console(struct mmc_emergency_info *emmc,
				     unsigned int off, int *actual_size)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	int saved_oip;
	int idx = 0;
	int rc, rc1, rc2;
	int block_shift = 0;

	*actual_size = 0;
	while (1) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;

		if (last_chunk_buf_len) {
			memcpy(ctx->bounce, last_chunk_buf, last_chunk_buf_len);
			rc1 =
			    log_buf_copy(ctx->bounce + last_chunk_buf_len, idx,
					 SECTOR_SIZE - last_chunk_buf_len);
		} else
			rc1 = log_buf_copy(ctx->bounce, idx, SECTOR_SIZE);

		if (rc1 < 0)	/* nothing copied */
			break;

		if (last_chunk_buf_len)
			rc = rc1 + last_chunk_buf_len;
		else
			rc = rc1;

		/* If it is the last chunk, just copy it to
		   last chunk buffer and exit loop. */
		if (rc != SECTOR_SIZE) {
			/*Leave the last chunk for next writting */
			memcpy(last_chunk_buf, ctx->bounce, rc);
			last_chunk_buf_len = rc;
			break;
		}
		oops_in_progress = saved_oip;

		rc2 = emmc_ipanic_writeflashpage(emmc, off + block_shift,
						 ctx->bounce);
		if (rc2 <= 0) {
			printk(KERN_EMERG
			       "%s: Flash write failed (%d)\n", __func__, rc2);
			return idx;
		}

		idx += rc1;
		block_shift++;

		if (last_chunk_buf_len) {
			*actual_size += last_chunk_buf_len;
			last_chunk_buf_len = 0;
		}
	}
	*actual_size += idx;
	return block_shift;
}

static void emmc_ipanic_flush_lastchunk_emmc(loff_t to,
					     int *size_written,
					     int *sector_written)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc = ctx->emmc;
	int rc = 0;

	if (last_chunk_buf_len) {
		memset(last_chunk_buf + last_chunk_buf_len, 0,
		       SECTOR_SIZE - last_chunk_buf_len);

		rc = emmc_ipanic_writeflashpage(emmc, to, last_chunk_buf);
		if (rc <= 0) {
			printk(KERN_EMERG
			       "emmc_ipanic: write last chunk failed (%d)\n",
			       rc);
			return;
		}

		*size_written += last_chunk_buf_len;
		(*sector_written)++;
		last_chunk_buf_len = 0;
	}
	return;
}

static void emmc_ipanic_write_thread_func(void)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc = ctx->emmc;
	int size_written;
	int thread_sector_count;

	thread_sector_count =
	    emmc_ipanic_write_console(emmc,
				      log_offset[IPANIC_LOG_THREADS] +
				      log_len[IPANIC_LOG_THREADS],
				      &size_written);
	if (thread_sector_count < 0) {
		printk(KERN_EMERG "Error writing threads to panic log! (%d)\n",
		       log_len[IPANIC_LOG_THREADS]);
		return;
	}
	log_size[IPANIC_LOG_THREADS] += size_written;
	log_len[IPANIC_LOG_THREADS] += thread_sector_count;

	/*reset the log buffer */
	log_buf_clear();
}

static void emmc_ipanic_write_logbuf(struct mmc_emergency_info *emmc, int log)
{
	/*
	 * Write the log data from second, the first block is reserved for
	 * panic header
	 */
	log_offset[log] = emmc->start_block + 1;
	log_len[log] =
	    emmc_ipanic_write_console(emmc, log_offset[log], &log_size[log]);
	if (log_size[log] < 0) {
		printk(KERN_EMERG
		       "Error writing console to panic log! (%d)\n",
		       log_len[log]);
		log_size[log] = 0;
		log_len[log] = 0;
	}
	/* flush last chunk buffer for console */
	emmc_ipanic_flush_lastchunk_emmc(log_offset[log] +
					 log_len[log],
					 &log_size[log], &log_len[log]);
}

static void emmc_ipanic_write_calltrace(struct mmc_emergency_info *emmc,
					int log)
{
	log_offset[log] = log_offset[log - 1] + log_len[log - 1];
	/*
	 * config func_stream_emmc to emmc_ipanic_write_thread_func to
	 * stream thread call trace.
	 */
	log_buf_clear();
	func_stream_emmc = emmc_ipanic_write_thread_func;
	show_state_filter(0);

	/* flush last chunk buffer */
	emmc_ipanic_flush_lastchunk_emmc(log_offset[log] +
					 log_len[log],
					 &log_size[log], &log_len[log]);
}

static void emmc_ipanic_write_pageheader(struct mmc_emergency_info *emmc)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct panic_header *hdr = (struct panic_header *)ctx->bounce;
	int log = IPANIC_LOG_CONSOLE;
	int rc;

	memset(ctx->bounce, 0, SECTOR_SIZE);
	hdr->magic = PANIC_MAGIC;
	hdr->version = PHDR_VERSION;
	/*Fill up log offset and size */
	while (log < IPANIC_LOG_MAX) {
		/*Configurate log offset and log size */
		hdr->log_offset[log] = (log_offset[log] - emmc->start_block)
		    << SECTOR_SIZE_SHIFT;
		hdr->log_length[log] = log_size[log];
		hdr->log_head[log] = log_head[log];
		hdr->log_woff[log] = log_woff[log];
		log++;
	}
	rc = emmc_ipanic_writeflashpage(emmc, emmc->start_block, ctx->bounce);
	if (rc <= 0) {
		printk(KERN_EMERG "emmc_ipanic: Header write failed (%d)\n",
		       rc);
		return;
	}
}

static int emmc_ipanic(struct notifier_block *this, unsigned long event,
		       void *ptr)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;
	int rc, log;

	if (!is_found_panic_par) {
		printk(KERN_EMERG "Not found the emergency partition!\n");
		return NOTIFY_DONE;
	}

	if (in_panic || disable_emmc_ipanic)
		return NOTIFY_DONE;

	in_panic = 1;

#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	add_preempt_count(PREEMPT_ACTIVE);
#endif
	touch_softlockup_watchdog();

	if (!ctx)
		goto out;
	emmc = ctx->emmc;
	if (!emmc)
		goto out;
	if (ctx->curr.magic) {
		printk(KERN_EMERG "Crash partition in use!\n");
		goto out;
	}

	rc = emmc->init();
	if (rc) {
		printk(KERN_EMERG
		       "Emmc emergency driver is not initialized successfully!, rc=%d\n",
		       rc);
		goto out;
	}

	memset(log_offset, 0, IPANIC_LOG_MAX * sizeof(int));
	memset(log_len, 0, IPANIC_LOG_MAX * sizeof(int));
	memset(log_size, 0, IPANIC_LOG_MAX * sizeof(int));

	/*Write all buffer into emmc */
	log = IPANIC_LOG_CONSOLE;
	while (log < IPANIC_LOG_MAX) {
		/* Clear temporary buffer */
		memset(ctx->bounce, 0, SECTOR_SIZE);
		/* Log every buffer into emmc */
		switch (log) {
		case IPANIC_LOG_CONSOLE:
			emmc_ipanic_write_logbuf(emmc, log);
			break;
		case IPANIC_LOG_THREADS:
			emmc_ipanic_write_calltrace(emmc, log);
			break;
#ifdef CONFIG_ANDROID_LOGGER
		case IPANIC_LOG_LOGCAT_MAIN:
		case IPANIC_LOG_LOGCAT_EVENTS:
		case IPANIC_LOG_LOGCAT_RADIO:
		case IPANIC_LOG_LOGCAT_SYSTEM:
			emmc_ipanic_write_logcatbuf(emmc, log,
				logcat_name[log - IPANIC_LOG_PROC_ENTRY]);
			break;
#endif
		default:
			break;
		}
		log++;
	}
	/* Write emmc ipanic partition header */
	emmc_ipanic_write_pageheader(emmc);
	printk(KERN_INFO "Panic log data written done!\n");
out:
#ifdef CONFIG_PREEMPT
	sub_preempt_count(PREEMPT_ACTIVE);
#endif
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call = emmc_ipanic,
};

static int panic_dbg_get(void *data, u64 *val)
{
	emmc_ipanic(NULL, 0, NULL);
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	BUG();
	return -1;
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set, "%llu\n");

static int match_panic_par(struct device *dev, void *data)
{
	struct mmc_emergency_info *emmc = drv_ctx.emmc;

	if (strcmp(dev_name(dev), emmc->emmc_disk_name) == 0) {
		emmc->disk_device = dev;
		printk(KERN_INFO "%s:emmc found\n", __func__);
		return 1;
	}
	dev = device_find_child(dev, NULL, match_panic_par);
	return dev != NULL;
}

static int emmc_panic_partition_notify(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct device *dev = data;
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;

	if (!ctx) {
		printk(KERN_ERR "%s:invalid panic handler\n", __func__);
		return 0;
	}

	emmc = ctx->emmc;
	if (!emmc) {
		printk(KERN_ERR "%s:invalid emmc infomation\n", __func__);
		return 0;
	}

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
	case BUS_NOTIFY_BOUND_DRIVER:
		/* if emmc already found, exit the function */
		if (emmc->disk_device)
			return 0;
		bus_find_device(&pci_bus_type, NULL, NULL, match_panic_par);
		if (emmc->disk_device) {
			emmc->disk = dev_to_disk(emmc->disk_device);
			if (emmc->disk == NULL) {
				printk(KERN_ERR "unable to get emmc disk\n");
				return 0;
			}

			/*get whole disk */
			emmc->bdev = bdget_disk(emmc->disk, 0);
			if (!emmc->bdev) {
				printk(KERN_ERR
				       "unable to get emmc block device\n");
				return 0;
			}
			emmc->part =
			    disk_get_part(emmc->disk, emmc->part_number);
			if (emmc->part == NULL) {
				printk(KERN_ERR "unable to get partition\n");
				return 0;
			}
			printk(KERN_INFO "panic partition found\n");
			emmc->start_block = emmc->part->start_sect;
			emmc->block_count = emmc->part->nr_sects;

			is_found_panic_par = 1;

			/*notify to add the panic device */
			emmc_panic_notify_add();
		}
		break;
	case BUS_NOTIFY_DEL_DEVICE:
	case BUS_NOTIFY_UNBIND_DRIVER:
		/*notify to add the panic device */
		emmc_panic_notify_remove();
		break;
	case BUS_NOTIFY_BIND_DRIVER:
	case BUS_NOTIFY_UNBOUND_DRIVER:
		/* Nothing to do here, but we don't want
		 * these actions to generate error messages,
		 * so we need to catch them
		 */
		break;
	default:
		printk(KERN_ERR "Unknown action (%lu) on %s\n",
		       action, dev_name(dev));
		return 0;
	}
	return 1;
}

static struct notifier_block panic_partition_notifier = {
	.notifier_call = emmc_panic_partition_notify,
};

void emmc_ipanic_stream_emmc(void)
{
	if (func_stream_emmc)
		(*func_stream_emmc) ();
}

EXPORT_SYMBOL(emmc_ipanic_stream_emmc);

int __init emmc_ipanic_init(void)
{
	is_found_panic_par = 0;
	bus_register_notifier(&pci_bus_type, &panic_partition_notifier);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	debugfs_create_file("emmc_ipanic", 0644, NULL, NULL, &panic_dbg_fops);
	debugfs_create_u32("disable_emmc_ipanic", 0644, NULL,
			   &disable_emmc_ipanic);

	/*initialization of drv_ctx */
	memset(&drv_ctx, 0, sizeof(drv_ctx));
	drv_ctx.emmc = &emmc_info;
	if (!ipanic_part_number)
		emmc_info.part_number = EMMC_PANIC_PART_NUM;
	else
		emmc_info.part_number = ipanic_part_number;

	drv_ctx.ipanic_proc_entry_name = ipanic_proc_entry_name;
	drv_ctx.bounce = (void *)__get_free_page(GFP_KERNEL);

	INIT_WORK(&proc_removal_work, emmc_ipanic_remove_proc_work);
	printk(KERN_INFO "Android kernel panic handler initialized on partition %sp%d!\n",
		emmc_info.emmc_disk_name, emmc_info.part_number);
	return 0;
}

core_param(disable_emmc_ipanic, disable_emmc_ipanic, uint, 0644);
module_init(emmc_ipanic_init);
