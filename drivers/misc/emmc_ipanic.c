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

#define SECTOR_SIZE_SHIFT (9)

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

	u32 console_offset;
	u32 console_length;

	u32 threads_offset;
	u32 threads_length;
};

struct emmc_ipanic_data {
	struct mmc_emergency_info *emmc;
	struct panic_header curr;
	void *bounce;
	struct proc_dir_entry *emmc_ipanic_console;
	struct proc_dir_entry *emmc_ipanic_threads;
};

static struct mmc_emergency_info emmc_info = {
	.init = mmc_emergency_init,
	.write = mmc_emergency_write,
	.emmc_disk_name = "mmcblk0",
	.part_number = 9,
	.name = "emmc_ipanic",
	.disk_device = NULL
};

static struct emmc_ipanic_data drv_ctx;
static struct work_struct proc_removal_work;
static int is_found_panic_par;

static DEFINE_MUTEX(drv_mutex);

static void emmc_panic_erase(unsigned char *buffer, Sector * sect)
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
		rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE);
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

	switch ((int)dat) {
	case 1:		/* emmc_ipanic_console */
		file_length = ctx->curr.console_length;
		file_offset = ctx->curr.console_offset;
		break;
	case 2:		/* emmc_ipanic_threads */
		file_length = ctx->curr.threads_length;
		file_offset = ctx->curr.threads_offset;
		break;
	default:
		pr_err("Bad dat (%d)\n", (int)dat);
		mutex_unlock(&drv_mutex);
		return -EINVAL;
	}

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
	rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE);
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
	*start = count;

	if ((offset + count) == file_length)
		*peof = 1;

	put_dev_sector(sect);
	mutex_unlock(&drv_mutex);

	return count;
}

static void emmc_ipanic_remove_proc_work(struct work_struct *work)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;

	mutex_lock(&drv_mutex);
	emmc_panic_erase(NULL, NULL);
	if (ctx->emmc_ipanic_console) {
		remove_proc_entry("emmc_ipanic_console", NULL);
		ctx->emmc_ipanic_console = NULL;
	}
	if (ctx->emmc_ipanic_threads) {
		remove_proc_entry("emmc_ipanic_threads", NULL);
		ctx->emmc_ipanic_threads = NULL;
	}
	mutex_unlock(&drv_mutex);
}

static int emmc_ipanic_proc_write(struct file *file, const char __user * buffer,
				  unsigned long count, void *data)
{
	schedule_work(&proc_removal_work);
	return count;
}

static void emmc_panic_notify_add()
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;
	unsigned char *read_buf_ptr;
	Sector sect;
	int rc;
	int proc_entry_created = 0;

	if (!ctx) {
		printk(KERN_ERR "%s:invalid panic handler\n", __func__);
		goto out_err;
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
	rc = blkdev_get(emmc->bdev, FMODE_READ | FMODE_WRITE);
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
		printk(KERN_ERR "%s: No panic data available\n", __func__);
		emmc_panic_erase(read_buf_ptr, &sect);
		goto put_sector;
	}

	if (ctx->curr.version != PHDR_VERSION) {
		printk(KERN_ERR "%s: Version mismatch (%d != %d)\n",
		       __func__, ctx->curr.version, PHDR_VERSION);
		emmc_panic_erase(read_buf_ptr, &sect);
		goto put_sector;
	}

	printk(KERN_INFO "%s: c(%u, %u) t(%u, %u)\n", __func__,
	       ctx->curr.console_offset, ctx->curr.console_length,
	       ctx->curr.threads_offset, ctx->curr.threads_length);

	if (ctx->curr.console_length) {
		ctx->emmc_ipanic_console =
		    create_proc_entry("emmc_ipanic_console", S_IFREG | S_IRUGO,
				      NULL);
		if (!ctx->emmc_ipanic_console)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->emmc_ipanic_console->read_proc =
			    emmc_ipanic_proc_read;
			ctx->emmc_ipanic_console->write_proc =
			    emmc_ipanic_proc_write;
			ctx->emmc_ipanic_console->size =
			    ctx->curr.console_length;
			ctx->emmc_ipanic_console->data = (void *)1;
			proc_entry_created = 1;
		}
	}

	if (ctx->curr.threads_length) {
		ctx->emmc_ipanic_threads =
		    create_proc_entry("emmc_ipanic_threads", S_IFREG | S_IRUGO,
				      NULL);
		if (!ctx->emmc_ipanic_threads)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->emmc_ipanic_threads->read_proc =
			    emmc_ipanic_proc_read;
			ctx->emmc_ipanic_threads->write_proc =
			    emmc_ipanic_proc_write;
			ctx->emmc_ipanic_threads->size =
			    ctx->curr.threads_length;
			ctx->emmc_ipanic_threads->data = (void *)2;
			proc_entry_created = 1;
		}
	}

	if (!proc_entry_created)
		emmc_panic_erase(read_buf_ptr, &sect);

put_sector:
	put_dev_sector(sect);
	return;
out_err:
	ctx->emmc = NULL;
}

static void emmc_panic_notify_remove()
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	if (!ctx)
		ctx->emmc = NULL;
}

static int in_panic;

static int emmc_ipanic_writeflashpage(struct mmc_emergency_info *emmc,
				      loff_t to, const u_char * buf)
{
	int rc;
	size_t wlen = SECTOR_SIZE;

	rc = emmc->write(buf, to);
	if (rc) {
		printk(KERN_EMERG
		       "%s: Error writing data to flash (%d)\n", __func__, rc);
		return rc;
	}

	return wlen;
}

extern int log_buf_copy(char *dest, int idx, int len);
extern void log_buf_clear(void);

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
	int rc, rc2;
	unsigned int last_chunk = 0;
	int block_shift = 0;

	while (!last_chunk) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;
		rc = log_buf_copy(ctx->bounce, idx, SECTOR_SIZE);
		if (rc < 0)
			break;

		if (rc != SECTOR_SIZE)
			last_chunk = rc;

		oops_in_progress = saved_oip;
		if (rc <= 0)
			break;
		if (rc != SECTOR_SIZE)
			memset(ctx->bounce + rc, 0, SECTOR_SIZE - rc);

		/* Check if there is spare block available */
		if (block_shift >= emmc->block_count) {
			printk(KERN_EMERG
			       "%s: No spare block for panic log dump (%d/%d)\n",
			       __func__, block_shift, emmc->block_count);
			break;
		}

		rc2 = emmc_ipanic_writeflashpage(emmc, off + block_shift,
						 ctx->bounce);
		if (rc2 <= 0) {
			printk(KERN_EMERG
			       "%s: Flash write failed (%d)\n", __func__, rc2);
			return idx;
		}
		if (!last_chunk)
			idx += rc2;
		else
			idx += last_chunk;
		block_shift++;
	}
	*actual_size = idx;
	return block_shift;
}

static int emmc_ipanic(struct notifier_block *this, unsigned long event,
		       void *ptr)
{
	struct emmc_ipanic_data *ctx = &drv_ctx;
	struct mmc_emergency_info *emmc;
	struct panic_header *hdr = (struct panic_header *)ctx->bounce;
	int console_offset = 0;
	int console_len = 0;
	int threads_offset = 0;
	int threads_len = 0;
	int rc;
	int actual_size_console;
	int actual_size_thread;

	if (!is_found_panic_par) {
		printk(KERN_EMERG "Not found the emergency partition!\n");
		return NOTIFY_DONE;
	}

	if (in_panic)
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
	/*
	 * Write the log data from second, the first block is reserved for
	 * panic header
	 */
	console_offset = emmc->start_block + 1;
	/*
	 * Write out the console
	 */
	console_len = emmc_ipanic_write_console(emmc, console_offset,
						&actual_size_console);
	if (actual_size_console < 0) {
		printk(KERN_EMERG "Error writing console to panic log! (%d)\n",
		       console_len);
		actual_size_console = 0;
		console_len = 0;
	}
	/*
	 * Write out all threads
	 */
	threads_offset = console_offset + console_len;

	log_buf_clear();
	show_state_filter(0);
	threads_len = emmc_ipanic_write_console(emmc, threads_offset,
						&actual_size_thread);
	if (actual_size_thread < 0) {
		printk(KERN_EMERG "Error writing threads to panic log! (%d)\n",
		       threads_len);
		actual_size_thread = 0;
		threads_len = 0;
	}
	/*
	 * Finally write the panic header
	 */
	memset(ctx->bounce, 0, SECTOR_SIZE);
	hdr->magic = PANIC_MAGIC;
	hdr->version = PHDR_VERSION;

	hdr->console_offset = (console_offset - emmc->start_block)
	    << SECTOR_SIZE_SHIFT;
	hdr->console_length = actual_size_console;

	hdr->threads_offset = (threads_offset - emmc->start_block)
	    << SECTOR_SIZE_SHIFT;
	hdr->threads_length = actual_size_thread;

	rc = emmc_ipanic_writeflashpage(emmc, emmc->start_block, ctx->bounce);
	if (rc <= 0) {
		printk(KERN_EMERG "emmc_ipanic: Header write failed (%d)\n",
		       rc);
		goto out;
	}

	printk(KERN_INFO "Panic log data wirttien done!\n");

out:
#ifdef CONFIG_PREEMPT
	sub_preempt_count(PREEMPT_ACTIVE);
#endif
	in_panic = 0;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call = emmc_ipanic,
};

static int panic_dbg_get(void *data, u64 * val)
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
	default:
		printk(KERN_EMERG "Incorrect action on %s\n", dev_name(dev));
		return 0;
	}
	return 1;
}

static struct notifier_block panic_partition_notifier = {
	.notifier_call = emmc_panic_partition_notify,
};

int __init emmc_ipanic_init(void)
{
	is_found_panic_par = 0;
	bus_register_notifier(&pci_bus_type, &panic_partition_notifier);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	debugfs_create_file("emmc_ipanic", 0644, NULL, NULL, &panic_dbg_fops);

	/*initialization of drv_ctx */
	memset(&drv_ctx, 0, sizeof(drv_ctx));
	drv_ctx.emmc = &emmc_info;
	drv_ctx.bounce = (void *)__get_free_page(GFP_KERNEL);

	INIT_WORK(&proc_removal_work, emmc_ipanic_remove_proc_work);
	printk(KERN_INFO "Android kernel panic handler initialized!\n");
	return 0;
}

module_init(emmc_ipanic_init);
