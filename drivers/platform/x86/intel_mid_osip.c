/*
 * OSIP driver for Medfield.
 *
 * Copyright (C) 2011 Intel Corp
 *
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/reboot.h>
#include <linux/pagemap.h>
#include <linux/blkdev.h>
#include <linux/debugfs.h>
#include <linux/genhd.h>
#include <linux/seq_file.h>
#include <linux/rpmsg.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>

/* change to "loop0" and use losetup for safe testing */
#define OSIP_BLKDEVICE "mmcblk0"
#include <asm/intel_scu_ipc.h>
#include <linux/power_supply.h>

#ifdef CONFIG_INTEL_MID_OSNIB_ILB
#include <linux/reboot.h>
#include <linux/intel_mid_osnib_ilb.h>
#endif

/* OSIP backup will be stored with this offset in the first sector */
#define OSIP_BACKUP_OFFSET 0xE0
#define MAX_OSII (7)
struct OSII {                   /* os image identifier */
	uint16_t os_rev_minor;
	uint16_t os_rev_major;
	uint32_t logical_start_block;

	uint32_t ddr_load_address;
	uint32_t entry_point;
	uint32_t size_of_os_image;

	uint8_t attribute;
	uint8_t reserved[3];
};

struct OSIP_header {            /* os image profile */
	uint32_t sig;
	uint8_t intel_reserved;
	uint8_t header_rev_minor;
	uint8_t header_rev_major;
	uint8_t header_checksum;
	uint8_t num_pointers;
	uint8_t num_images;
	uint16_t header_size;
	uint32_t reserved[5];

	struct OSII desc[MAX_OSII];
};

/* A boolean variable, that is set, when wants to make the platform
    force shuts down */
static int force_shutdown_occured;

int mmcblk0_match(struct device *dev, void *data)
{
	if (strcmp(dev_name(dev), OSIP_BLKDEVICE) == 0)
		return 1;
	return 0;
}
static struct block_device *get_emmc_bdev(void)
{
	struct block_device *bdev;
	struct device *emmc_disk;

	emmc_disk = class_find_device(&block_class, NULL, NULL, mmcblk0_match);
	if (emmc_disk == 0) {
		pr_err("emmc not found!\n");
		return NULL;
	}
	/* partition 0 means raw disk */
	bdev = bdget_disk(dev_to_disk(emmc_disk), 0);
	if (bdev == NULL) {
		dev_err(emmc_disk, "unable to get disk\n");
		return NULL;
	}
	/* Note: this bdev ref will be freed after first
	   bdev_get/bdev_put cycle */
	return bdev;
}
static uint8_t calc_checksum(void *_buf, int size)
{
	int i;
	uint8_t checksum = 0, *buf = (uint8_t *)_buf;
	for (i = 0; i < size; i++)
		checksum = checksum ^ (buf[i]);
	return checksum;
}
/*
  Allows to access the osip image. Callback is passed for user to
  implement actual usage.
  This function takes care of the blkdev housekeeping

  how to do proper block access is got from:
  fs/partitions/check.c
  mtd/devices/block2mtd.c
*/
/* callbacks returns whether the OSIP was modified */
typedef int (*osip_callback_t)(struct OSIP_header *osip, void *data);

static int access_osip_record(osip_callback_t callback, void *cb_data)
{
	Sector sect;
	struct block_device *bdev;
	char *buffer;
	struct OSIP_header *osip;
	struct OSIP_header *osip_backup;
	int ret = 0;
	int dirty = 0;

	bdev = get_emmc_bdev();
	if (bdev == NULL) {
		pr_err("%s: get_emmc failed!\n", __func__);
		return -ENODEV;
	}
	/* make sure the block device is open rw */
	ret = blkdev_get(bdev, FMODE_READ|FMODE_WRITE, NULL);
	if (ret < 0) {
		pr_err("%s: blk_dev_get failed!\n", __func__);
		return -ret;
	}
	/* get memmap of the OSIP header */
	buffer = read_dev_sector(bdev, 0, &sect);

	if (buffer == NULL) {
		ret = -ENODEV;
		goto bd_put;
	}
	osip = (struct OSIP_header *) buffer;
	/* some sanity checks */
	if (osip->header_size <= 0 || osip->header_size > PAGE_SIZE) {
		pr_err("%s: corrupted osip!\n", __func__);
		ret = -EINVAL;
		goto put_sector;
	}
	if (calc_checksum(osip, osip->header_size) != 0) {
		pr_err("%s: corrupted osip!\n", __func__);
		ret = -EINVAL;
		goto put_sector;
	}
	/* store the OSIP backup which will be used to recover in PrOS */
	osip_backup = kmalloc(sizeof(struct OSIP_header), GFP_KERNEL);
	if (osip_backup == NULL)
		goto put_sector;
	memcpy(osip_backup, osip, sizeof(struct OSIP_header));

	lock_page(sect.v);
	dirty = callback(osip, cb_data);
	if (dirty) {
		memcpy(buffer + OSIP_BACKUP_OFFSET, osip_backup,
		       sizeof(struct OSIP_header));
		osip->header_checksum = 0;
		osip->header_checksum = calc_checksum(osip, osip->header_size);
		set_page_dirty(sect.v);
	}
	unlock_page(sect.v);
	sync_blockdev(bdev);
	kfree(osip_backup);
put_sector:
	put_dev_sector(sect);
bd_put:
	blkdev_put(bdev, FMODE_READ|FMODE_WRITE);
	return 0;
}

/*
   OSHOB - OS Handoff Buffer
   OSNIB - OS No Init Buffer
   This buffer contains the 32-byte value that is persists across cold and warm
   resets only, but loses context on a cold boot.
   More info about OSHOB, OSNIB could be found in FAS Section 2.8.
   We use the first byte in OSNIB to store and pass the Reboot/boot Reason.
   The attribute of OS image is selected for Reboot/boot reason.
*/

#define SIGNED_MOS_ATTR		0x0
#define SIGNED_COS_ATTR		0x0A
#define SIGNED_POS_ATTR		0x0E
#define SIGNED_RECOVERY_ATTR	0x0C
#define SIGNED_POSCOS_ATTR	0x10

static int osip_reboot_notifier_call(struct notifier_block *notifier,
				     unsigned long what, void *data)
{
	int ret = NOTIFY_DONE;
	int ret_ipc;
	int ret_cmos;
	char *cmd = (char *)data;
#ifdef DEBUG
	u8 rbt_reason;
#endif

	/* If system power off with charger connected, set the Reboot
	   Reason to COS */
	if (what != SYS_RESTART) {
		if (what == SYS_HALT || what == SYS_POWER_OFF) {
			pr_info("%s(): sys power off ...\n", __func__);
			if (power_supply_is_system_supplied()) {
				pr_warn("[SHTDWN] %s, Shutdown overload, "
					"switching to COS because a charger is "
					"plugged in\n", __func__);
				pr_info("charger connected ...\n");
#ifdef DEBUG
				intel_scu_ipc_read_osnib_rr(&rbt_reason);
#endif
				ret_ipc = intel_scu_ipc_write_osnib_rr(
							SIGNED_COS_ATTR);
				if (ret_ipc < 0)
					pr_err("%s cannot write reboot reason in OSNIB\n",
					       __func__);
#ifdef DEBUG
				intel_scu_ipc_read_osnib_rr(&rbt_reason);
#endif
			} else {
				pr_warn("[SHTDWN] %s, Shutdown without charger plugged in\n",
					__func__);
								/*
				 * PNW and CLVP depend on watchdog driver to
				 * send COLD OFF message to SCU.
				 * SCU watchdog is not available from TNG A0,
				 * so SCU FW provides a new IPC message to shut
				 * down the system.
				 */
				if (intel_mid_identify_cpu() ==
					INTEL_MID_CPU_CHIP_TANGIER) {
					ret = rpmsg_send_generic_simple_command(
						IPCMSG_COLD_OFF, 0);
					if (ret)
						pr_err("%s(): COLD_OFF ipc failed\n",
								 __func__);
				}
			}
		} else {
			pr_warn("[SHTDWN] %s, invalid value\n", __func__);
		}

		return NOTIFY_DONE;
	}

#ifdef CONFIG_INTEL_MID_OSNIB_ILB
	if (data && 0 == strncmp(cmd, "recovery", 9)) {
		pr_warn("[SHTDWN] %s, rebooting into Recovery\n", __func__);
#ifdef DEBUG
		intel_mid_ilb_read_osnib_rr(&rbt_reason);
#endif
		ret_cmos = intel_mid_ilb_write_osnib_rr(SIGNED_RECOVERY_ATTR);
		if (ret_cmos < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				__func__);
		ret = NOTIFY_OK;
	} else if (data && 0 == strncmp(cmd, "bootloader", 11)) {
		pr_warn("[SHTDWN] %s, rebooting into Fastboot\n", __func__);
		ret_cmos = intel_mid_ilb_write_osnib_rr(SIGNED_POS_ATTR);
		if (ret_cmos < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				__func__);
		ret = NOTIFY_OK;
	} else {
		pr_warn("[SHTDWN] %s, rebooting into Android\n", __func__);
		ret_cmos = intel_mid_ilb_write_osnib_rr(SIGNED_MOS_ATTR);
		if (ret_cmos < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				 __func__);
		ret = NOTIFY_OK;
	}
	emergency_restart();
	return ret;
#else
	if (data && 0 == strncmp(cmd, "recovery", 9)) {
		pr_warn("[SHTDWN] %s, rebooting into Recovery\n", __func__);
#ifdef DEBUG
		intel_scu_ipc_read_osnib_rr(&rbt_reason);
#endif
		ret_ipc = intel_scu_ipc_write_osnib_rr(SIGNED_RECOVERY_ATTR);
		if (ret_ipc < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				__func__);
		ret = NOTIFY_OK;
	} else if (data && 0 == strncmp(cmd, "bootloader", 11)) {
		pr_warn("[SHTDWN] %s, rebooting into Fastboot\n", __func__);
		ret_ipc = intel_scu_ipc_write_osnib_rr(SIGNED_POS_ATTR);
		if (ret_ipc < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				__func__);
		ret = NOTIFY_OK;
	} else {
		pr_warn("[SHTDWN] %s, rebooting into Android\n", __func__);
		ret_ipc = intel_scu_ipc_write_osnib_rr(SIGNED_MOS_ATTR);
		if (ret_ipc < 0)
			pr_err("%s cannot write reboot reason in OSNIB\n",
				 __func__);
		ret = NOTIFY_OK;
	}
	return ret;

#endif
}

#ifdef DEBUG
/* to test without reboot...
echo -n fastboot >/sys/module/intel_mid_osip/parameters/test
*/
static int osip_test_write(const char *val, struct kernel_param *kp)
{
	osip_reboot_notifier_call(NULL, SYS_RESTART, (void *)val);
	return 0;
}

static int osip_test_read(char *buffer, struct kernel_param *kp)
{
	return 0;
}

module_param_call(test, osip_test_write,
		osip_test_read, NULL, 0644);

#endif

module_param(force_shutdown_occured, int, 0644);
MODULE_PARM_DESC(force_shutdown_occured,
		"Variable to be set by user space"
		" when a force shudown condition occurs, to allow"
		" system shut down even with charger connected");

int get_force_shutdown_occured()
{
	return force_shutdown_occured;
}
EXPORT_SYMBOL(get_force_shutdown_occured);

static struct notifier_block osip_reboot_notifier = {
	.notifier_call = osip_reboot_notifier_call,
};

/* useful for engineering, not for product */
#ifdef CONFIG_INTEL_MID_OSIP_DEBUG_FS
/* show and edit boot.bin's cmdline */
#define OSIP_MAX_CMDLINE_SIZE 0x400
/* number N of sectors (512bytes) needed for the cmdline, N+1 needed */
#define OSIP_MAX_CMDLINE_SECTOR ((OSIP_MAX_CMDLINE_SIZE >> 9) + 1)

struct cmdline_priv {
	Sector sect[OSIP_MAX_CMDLINE_SECTOR];
	struct block_device *bdev;
	int lba;
	char *cmdline;
	int osip_id;
	uint8_t attribute;
};

static int osip_find_cmdline(struct OSIP_header *osip, void *data)
{
	struct cmdline_priv *p = (struct cmdline_priv *) data;
	if (p->osip_id < MAX_OSII) {
		p->attribute = osip->desc[p->osip_id].attribute;
		p->lba = osip->desc[p->osip_id].logical_start_block;
	}
	return 0;
}
int open_cmdline(struct inode *i, struct file *f)
{
	struct cmdline_priv *p;
	int ret, j;
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("%s: unable to allocate p!\n", __func__);
		ret = -ENOMEM;
		goto end;
	}
	if (i->i_private)
		p->osip_id = (int) i->i_private;
	f->private_data = 0;
	access_osip_record(osip_find_cmdline, (void *)p);
	if (!p->lba) {
		pr_err("%s: osip_find_cmdline failed!\n", __func__);
		ret = -ENODEV;
		goto free;
	}
	/* need to open it again */
	p->bdev = get_emmc_bdev();
	if (!p->bdev) {
		pr_err("%s: access_osip_record failed!\n", __func__);
		ret = -ENODEV;
		goto free;
	}
	ret = blkdev_get(p->bdev, f->f_mode, NULL);
	if (ret < 0) {
		pr_err("%s: blk_dev_get failed!\n", __func__);
		ret = -ENODEV;
		goto free;
	}
	if (p->lba >= get_capacity(p->bdev->bd_disk)) {
		pr_err("%s: %d out of disk bound!\n", __func__, p->lba);
		ret = -EINVAL;
		goto put;
	}
	for (j = (OSIP_MAX_CMDLINE_SECTOR - 1); j >= 0; j--) {
		p->cmdline = read_dev_sector(p->bdev,
				     p->lba + j,
				     &p->sect[j]);
	}
	if (!p->cmdline) {
		pr_err("%s: read_dev_sector failed!\n", __func__);
		ret = -ENODEV;
		goto put;
	}
	if (!(p->attribute & 1))
		/* even number: signed plus 480 bytes for VRL header. */
		p->cmdline += 0x1E0;

	f->private_data = p;
	return 0;
put:
	blkdev_put(p->bdev, f->f_mode);
free:
	kfree(p);
end:
	return ret;
}

static ssize_t read_cmdline(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct cmdline_priv *p =
		(struct cmdline_priv *)file->private_data;
	if (!p)
		return -ENODEV;
	return simple_read_from_buffer(buf, count, ppos,
			p->cmdline, strnlen(p->cmdline, OSIP_MAX_CMDLINE_SIZE));
}

static ssize_t write_cmdline(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct cmdline_priv *p =
		(struct cmdline_priv *)file->private_data;
	int ret, i;
	if (!p)
		return -ENODEV;
	/* @todo detect if image is signed, and prevent write */
	lock_page(p->sect[0].v);
	for (i = 1; i < OSIP_MAX_CMDLINE_SECTOR; i++) {
		if (p->sect[i-1].v != p->sect[i].v)
			lock_page(p->sect[i].v);
	}
	ret = simple_write_to_buffer(p->cmdline, OSIP_MAX_CMDLINE_SIZE-1,
				     ppos,
				     buf, count);
	if (ret < 0)
		goto unlock;
	/* make sure we zero terminate the cmdline */
	if (file->f_pos + count < OSIP_MAX_CMDLINE_SIZE)
		p->cmdline[file->f_pos + count] = '\0';

unlock:
	set_page_dirty(p->sect[0].v);
	unlock_page(p->sect[0].v);
	for (i = 1; i < OSIP_MAX_CMDLINE_SECTOR; i++) {
		if (p->sect[i-1].v != p->sect[i].v) {
			set_page_dirty(p->sect[i].v);
			unlock_page(p->sect[i].v);
		}
	}
	return ret;
}
int release_cmdline(struct inode *i, struct file *f)
{
	int j;
	struct cmdline_priv *p =
		(struct cmdline_priv *)f->private_data;
	if (!p)
		return -ENOMEM;
	put_dev_sector(p->sect[0]);
	for (j = 1; j < OSIP_MAX_CMDLINE_SECTOR; j++)
		if (p->sect[j-1].v != p->sect[j].v)
			put_dev_sector(p->sect[j]);
	blkdev_put(p->bdev, f->f_mode);
	kfree(p);
	return 0;
}
int fsync_cmdline(struct file *f, int datasync)
{
	struct cmdline_priv *p =
		(struct cmdline_priv *)f->private_data;
	if (!p)
		return -ENOMEM;
	sync_blockdev(p->bdev);
	return 0;
}
static const struct file_operations osip_cmdline_fops = {
	.open =         open_cmdline,
	.read =         read_cmdline,
	.write =        write_cmdline,
	.release =      release_cmdline,
	.fsync =        fsync_cmdline
};

/* decode the osip */

static int decode_show_cb(struct OSIP_header *osip, void *data)
{
	struct seq_file *s = (struct seq_file *) data;
	int i;

	seq_printf(s, "HEADER:\n"
		   "\tsig              = 0x%x\n"
		   "\theader_size      = 0x%hx\n"
		   "\theader_rev_minor = 0x%hhx\n"
		   "\theader_rev_major = 0x%hhx\n"
		   "\theader_checksum  = 0x%hhx\n"
		   "\tnum_pointers     = 0x%hhx\n"
		   "\tnum_images       = 0x%hhx\n",
		   osip->sig,
		   osip->header_size,
		   osip->header_rev_minor,
		   osip->header_rev_major,
		   osip->header_checksum,
		   osip->num_pointers,
		   osip->num_images);

	for (i = 0; i < osip->num_pointers; i++)
		seq_printf(s, "image%d\n"
			   "\tos_rev              =  0x%0hx\n"
			   "\tos_rev              = 0x%hx\n"
			   "\tlogical_start_block = 0x%x\n"
			   "\tddr_load_address    = 0x%0x\n"
			   "\tentry_point         = 0x%0x\n"
			   "\tsize_of_os_image    = 0x%x\n"
			   "\tattribute           = 0x%02x\n"
			   "\treserved            = %02x%02x%02x\n",
			   i,
			   osip->desc[i].os_rev_minor,
			   osip->desc[i].os_rev_major,
			   osip->desc[i].logical_start_block,
			   osip->desc[i].ddr_load_address,
			   osip->desc[i].entry_point,
			   osip->desc[i].size_of_os_image,
			   osip->desc[i].attribute,
			   osip->desc[i].reserved[0],
			   osip->desc[i].reserved[1],
			   osip->desc[i].reserved[2]);
	return 0;
}
static int decode_show(struct seq_file *s, void *unused)
{
	access_osip_record(decode_show_cb, (void *)s);
	return 0;
}
static int decode_open(struct inode *inode, struct file *file)
{
	return single_open(file, decode_show, NULL);
}

static const struct file_operations osip_decode_fops = {
	.open           = decode_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};
static struct dentry *osip_dir;
static void create_debugfs_files(void)
{
	/* /sys/kernel/debug/osip */
	osip_dir = debugfs_create_dir("osip", NULL);
	/* /sys/kernel/debug/osip/cmdline */
	(void) debugfs_create_file("cmdline",
				S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				osip_dir, (void *)0, &osip_cmdline_fops);
	/* /sys/kernel/debug/osip/cmdline_ros */
	(void) debugfs_create_file("cmdline_ros",
				S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				osip_dir, (void *)1, &osip_cmdline_fops);
	/* /sys/kernel/debug/osip/cmdline_pos */
	(void) debugfs_create_file("cmdline_pos",
				S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				osip_dir, (void *)2, &osip_cmdline_fops);
	/* /sys/kernel/debug/osip/decode */
	(void) debugfs_create_file("decode",
				S_IFREG | S_IRUGO,
				osip_dir, NULL, &osip_decode_fops);
}
static void remove_debugfs_files(void)
{
	debugfs_remove_recursive(osip_dir);
}
#else /* defined(CONFIG_INTEL_MID_OSIP_DEBUG_FS) */
static void create_debugfs_files(void)
{
}
static void remove_debugfs_files(void)
{
}
#endif

static int osip_init(void)
{
	if (intel_mid_identify_cpu() == 0)
		return -ENODEV;

	if (register_reboot_notifier(&osip_reboot_notifier))
		pr_warning("osip: unable to register reboot notifier");

	create_debugfs_files();
	return 0;
}

static void osip_exit(void)
{
	unregister_reboot_notifier(&osip_reboot_notifier);
	remove_debugfs_files();
}

static int osip_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		return -ENODEV;
	}

	dev_info(&rpdev->dev, "Probed OSIP rpmsg device\n");

	return osip_init();
}

static void osip_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static void osip_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	osip_exit();
	dev_info(&rpdev->dev, "Removed OSIP rpmsg device\n");
}

static struct rpmsg_device_id osip_rpmsg_id_table[] = {
	{ .name = "rpmsg_osip" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, osip_rpmsg_id_table);

static struct rpmsg_driver osip_rpmsg_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= osip_rpmsg_id_table,
	.probe		= osip_rpmsg_probe,
	.callback	= osip_rpmsg_cb,
	.remove		= osip_rpmsg_remove,
};

static int __init osip_rpmsg_init(void)
{
	return register_rpmsg_driver(&osip_rpmsg_driver);
}
module_init(osip_rpmsg_init);

static void __exit osip_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&osip_rpmsg_driver);
}
module_exit(osip_rpmsg_exit);

MODULE_AUTHOR("Pierre Tardy <pierre.tardy@intel.com>");
MODULE_AUTHOR("Xiaokang Qin <xiaokang.qin@intel.com>");
MODULE_DESCRIPTION("Intel Medfield OSIP Driver");
MODULE_LICENSE("GPL v2");
