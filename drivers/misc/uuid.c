/*
 * drivers/misc/uuid.c
 *
 * Copyright (C) 2011 Intel Corp
 * Author: jun.zhang@intel.com
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/pti.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/genhd.h>
#include <linux/jhashv2.h>
#include <linux/io.h>
#include <asm/intel_scu_ipc.h>

#define EMMC0_ID_LENGTH            17
static char emmc0_id[EMMC0_ID_LENGTH];
struct proc_dir_entry *emmc0_id_entry;

static int emmc0_id_read(char *buffer, char **start, off_t offset,
					int count, int *peof, void *data)
{
	if (offset > 0) {
		/* We have finished to read, return 0 */
		return 0;
	} else {
		/* Fill the buffer, return the buffer size */
		memcpy(buffer, emmc0_id, sizeof(emmc0_id)-1);
		return sizeof(emmc0_id)-1;
	}
}

static int mmcblk0_match(struct device *dev, void *data)
{
	if (strcmp(dev_name(dev), "mmcblk0") == 0)
		return 1;
	return 0;
}

static int get_emmc0_cid(void)
{
	struct device *emmc_disk;
	/*
	 * Automation people are needing proper serial number for ADB
	 * lets derivate from the serial number of the emmc card.
	 */
	emmc_disk = class_find_device(&block_class, NULL, NULL, mmcblk0_match);
	if (emmc_disk) {
		struct gendisk *disk = dev_to_disk(emmc_disk);
		struct mmc_card *card = mmc_dev_to_card(disk->driverfs_dev);
		if (card) {
			snprintf(emmc0_id, sizeof(emmc0_id),
				 "Medfield%08X",
				 jhash(&card->cid, sizeof(card->cid), 0));
			return 1;
		}
	}
	return 0;
}

#define SERIALNO_CMDLINE "androidboot.serialno="

static void set_cmdline_serialno(void)
{
	char *start;
	char *serialno;
	char *end_of_field;
	int serialno_len;
	int value_length;

	if (intel_mid_ssn[0] != '\0') {
		serialno = intel_mid_ssn;
	} else {
		if (strlen(emmc0_id)) {
			serialno = emmc0_id;
		} else {
			pr_err("Failed to get SSN or emmc0 ID\n");
			goto error;
		}
	}

	start = strstr(saved_command_line, SERIALNO_CMDLINE);
	if (!start) {
		pr_err("Could not find %s in cmdline\n" SERIALNO_CMDLINE);
		goto error;
	}

	serialno_len = strlen(serialno);

	start += sizeof(SERIALNO_CMDLINE) - 1;

	end_of_field = strstr(start, " ");
	if (end_of_field)
		value_length = end_of_field - start;
	else
		value_length = strlen(start);

	if (value_length < serialno_len) {
		pr_err("Pre-filled serialno cmdline value is too small\n");
		goto error;
	}

	memcpy(start, serialno, serialno_len);
	memset(start + serialno_len, ' ', value_length - serialno_len);

	return;
error:
	pr_err("serialno will not be updated in cmdline!\n");
	return;
}

static int __init uuid_init(void)
{

	memset(emmc0_id, 0x00, sizeof(emmc0_id));
	if (get_emmc0_cid()) {
		emmc0_id_entry = create_proc_entry("emmc0_id_entry",
						S_IFREG | S_IRUGO, NULL);
		if (emmc0_id_entry == 0) {
			pr_err("Fail creating procfile emmc0_id_entry\n");
			return -ENOMEM;
		}
		emmc0_id_entry->read_proc = emmc0_id_read;
		emmc0_id_entry->write_proc = NULL;
		emmc0_id_entry->size = sizeof(emmc0_id)-1;
	}

	set_cmdline_serialno();

	return 0;
}

static void __exit uuid_exit(void)
{

	if (emmc0_id_entry)
		remove_proc_entry("emmc0_id_entry", NULL);

}

late_initcall(uuid_init);
module_exit(uuid_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("get uuid");
MODULE_AUTHOR("Zhang Jun<jun.zhang@intel.com>");

