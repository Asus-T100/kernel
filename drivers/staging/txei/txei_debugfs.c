/*
 *
 * Intel Management Engine Interface (Intel TXEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#define DEBUG
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/pci.h>
#include <linux/export.h>

#include "txei.h"

#include "txei_dev.h"
#include "hw.h"
#include "hw-txe.h"



#define DEBUGFS_ADD_FILE(_txei, name, parent, mode)	\
	debugfs_create_file(#name, mode, parent,	\
			_txei, &txei_dbgfs_fops_##name)


static int txei_dbgfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t txei_dbgfs_read_meclients(struct file *fp, char __user *ubuf,
					size_t cnt, loff_t *ppos)
{
	struct txei_device *dev = fp->private_data;
	struct txei_me_client *cl;
	const size_t bufsz = 1024;
	char *buf = kzalloc(bufsz, GFP_KERNEL);
	int i;
	int pos = 0;
	int ret;

	if  (!buf)
		return -ENOMEM;

	pos += scnprintf(buf + pos, bufsz - pos,
			"  |id|addr|         UUID                       |con|msg len|\n");
	mutex_lock(&dev->device_lock);
	for (i = 0; i < dev->me_clients_num; i++) {
		cl = &dev->me_clients[i];
		pos += scnprintf(buf + pos, bufsz - pos,
			"%2d|%2d|%4d|%pUl|%3d|%7d|\n",
			i, cl->client_id,
			cl->props.fixed_address,
			&cl->props.protocol_name,
			cl->props.max_number_of_connections,
			cl->props.max_msg_length);
	}
	mutex_unlock(&dev->device_lock);
	ret = simple_read_from_buffer(ubuf, cnt, ppos, buf, pos);
	if (buf != NULL)
		kfree(buf);
	return ret;
}

static const struct file_operations txei_dbgfs_fops_meclients = {
	.open = txei_dbgfs_open,
	.read = txei_dbgfs_read_meclients,
	.llseek = generic_file_llseek,
};

static const char * const me_state[] = {
	"INITIALIZING",
	"INIT_CLIENTS",
	"ENABLED",
	"RESETING",
	"DISABLED",
	"RECOVERING_FROM_RESET",
	"POWER_DOWN",
	"POWER_UP"
};

static ssize_t txei_dbgfs_read_mestate(struct file *fp, char __user *ubuf,
					size_t cnt, loff_t *ppos)
{
	struct txei_device *dev = fp->private_data;
	const size_t bufsz = 1024;
	char *buf = kzalloc(bufsz, GFP_KERNEL);
	int pos = 0;
	int ret;

	if  (!buf)
		return -ENOMEM;

	mutex_lock(&dev->device_lock);
	/* FIXME check txei_state boundaries */
	pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
			me_state[dev->dev_state]);
	mutex_unlock(&dev->device_lock);
	ret = simple_read_from_buffer(ubuf, cnt, ppos, buf, pos);
	if (buf != NULL)
		kfree(buf);
	return ret;
}
static const struct file_operations txei_dbgfs_fops_mestate = {
	.open = txei_dbgfs_open,
	.read = txei_dbgfs_read_mestate,
	.llseek = generic_file_llseek,
};

static ssize_t txei_dbgfs_read_aliveness(struct file *fp, char __user *ubuf,
					size_t cnt, loff_t *ppos)
{
	struct txei_device *dev = fp->private_data;
	const size_t bufsz = 1024;
	char *buf = kzalloc(bufsz, GFP_KERNEL);
	int pos = 0;
	int ret;

	if  (!buf)
		return -ENOMEM;

	/* FIMXE: move to atime to msec:sec */
	pos += scnprintf(buf + pos, bufsz - pos,
			"ALIVENESS |ATIME  |         |\n");
	mutex_lock(&dev->device_lock);
	pos += scnprintf(buf + pos, bufsz - pos, "%s|%ld|%ld\n",
		dev->aliveness ? "alive     " : "dormant   ",
		dev->aliveness_atime, dev->aliveness_timeout);
	/* FIXME check txei_state boundaries */
	mutex_unlock(&dev->device_lock);
	ret = simple_read_from_buffer(ubuf, cnt, ppos, buf, pos);
	kfree(buf);
	return ret;
}
static const struct file_operations txei_dbgfs_fops_aliveness = {
	.open = txei_dbgfs_open,
	.read = txei_dbgfs_read_aliveness,
	.llseek = generic_file_llseek,
};
/**
 * txei_dbgfs_unregister - Remove the debugfs files and directories
 * @txei - pointer to txei device private dat
 */
void txei_dbgfs_unregister(struct txei_device *dev)
{
	if (!dev->dbgfs_dir)
		return;
	debugfs_remove_recursive(dev->dbgfs_dir);
	dev->dbgfs_dir = NULL;
}
EXPORT_SYMBOL_GPL(txei_dbgfs_unregister);

/**
 * Add the debugfs files
 *
 */
int txei_dbgfs_register(struct txei_device *dev, const char *name)
{
	struct dentry *dir;
	dir = debugfs_create_dir(name, NULL);
	if (!dir)
		return -ENOMEM;
	if (!DEBUGFS_ADD_FILE(dev, meclients, dir, S_IRUSR)) {
		txei_err(dev, "meclients: registration failed\n");
		goto err;
	}
	if (!DEBUGFS_ADD_FILE(dev, mestate, dir, S_IRUSR)) {
		txei_err(dev, "mestate: registration failed\n");
		goto err;
	}
	if (!DEBUGFS_ADD_FILE(dev, aliveness, dir, S_IRUSR)) {
		txei_err(dev, "aliveness: registration failed\n");
		goto err;
	}
	dev->dbgfs_dir = dir;
	return 0;
err:
	txei_dbgfs_unregister(dev);
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(txei_dbgfs_register);
