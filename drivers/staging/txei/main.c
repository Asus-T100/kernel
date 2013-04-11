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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/uuid.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>

#include "txei.h"


#include "txei_dev.h"
#include "hw-txe.h"
#include "txei_mm.h"

#define TXEI_READ_TIMEOUT 45

static DEFINE_MUTEX(txei_mutex);


/**
 * find_read_list_entry - find read list entry
 *
 * @dev: device structure
 * @file: pointer to file structure
 *
 * returns cb on success, NULL on error
 */
static struct txei_cl_cb *find_read_list_entry(
		struct txei_device *dev,
		struct txei_cl *cl)
{
	struct txei_cl_cb *cb_pos = NULL;
	struct txei_cl_cb *cb_next = NULL;

	if (!dev->read_list.status &&
			!list_empty(&dev->read_list.txei_cb.cb_list)) {

		txei_dbg(dev, "remove read_list CB\n");
		list_for_each_entry_safe(cb_pos, cb_next,
				&dev->read_list.txei_cb.cb_list, cb_list) {
			struct txei_cl *cl_temp;
			cl_temp = (struct txei_cl *)cb_pos->file_private;

			if (txei_cl_cmp_id(cl, cl_temp))
				return cb_pos;
		}
	}
	return NULL;
}

/**
 * txei_open - the open function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * returns 0 on success, <0 on error
 */
static int txei_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct txei_cl *cl;
	struct pci_dev *pdev;
	struct txei_device *dev;
	unsigned long cl_id;

	int err;

	err = -ENODEV;
	if (!misc->parent)
		goto out;

	pdev = container_of(misc->parent, struct pci_dev, dev);

	dev = pci_get_drvdata(pdev);
	if (!dev)
		goto out;

	mutex_lock(&dev->device_lock);
	err = -ENOMEM;
	cl = txei_cl_allocate(dev);
	if (!cl)
		goto out_unlock;

	err = -ENODEV;
	if (dev->dev_state != TXEI_DEV_ENABLED) {
		txei_dbg(dev, "txei_state != TXEI_DEV_ENABLED  txei_state = %d\n",
			dev->dev_state);
		goto out_unlock;
	}
	err = -EMFILE;
	if (dev->open_handle_count >= TXEI_MAX_OPEN_HANDLE_COUNT)
		goto out_unlock;

	cl_id = find_first_zero_bit(dev->host_clients_map, TXEI_CLIENTS_MAX);
	if (cl_id > TXEI_CLIENTS_MAX)
		goto out_unlock;

	cl->host_client_id = cl_id;

	txei_dbg(dev, "client_id = %d\n", cl->host_client_id);

	dev->open_handle_count++;
	list_add_tail(&cl->link, &dev->file_list);

	set_bit(cl->host_client_id, dev->host_clients_map);
	cl->state = TXEI_FILE_INITIALIZING;
	cl->sm_state = 0;

	file->private_data = cl;
	mutex_unlock(&dev->device_lock);

	return 0;

out_unlock:
	mutex_unlock(&dev->device_lock);
	if (cl != NULL)
		kfree(cl);
out:
	return err;
}

/**
 * txei_release - the release function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * returns 0 on success, <0 on error
 */
static int txei_release(struct inode *inode, struct file *file)
{
	struct txei_cl *cl = file->private_data;
	struct txei_cl_cb *cb;
	struct txei_device *dev;
	int rets = 0;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);
	if (cl->state == TXEI_FILE_CONNECTED) {
		cl->state = TXEI_FILE_DISCONNECTING;
		txei_dbg(dev, "disconnecting client host client = %d ME client = %d\n",
			cl->host_client_id, cl->me_client_id);
		rets = txei_disconnect_host_client(dev, cl);
	}
	txei_cl_flush_queues(cl);
	txei_dbg(dev, "remove client host client = %d, ME client = %d\n",
			cl->host_client_id,
			cl->me_client_id);

	if (dev->open_handle_count > 0) {
		clear_bit(cl->host_client_id,
				dev->host_clients_map);
		dev->open_handle_count--;
	}
	txei_remove_client_from_file_list(dev, cl->host_client_id);

	/* free read cb */
	cb = NULL;
	if (cl->read_cb) {
		cb = find_read_list_entry(dev, cl);
		/* Remove entry from read list */
		if (cb)
			list_del(&cb->cb_list);

		cb = cl->read_cb;
		cl->read_cb = NULL;
	}

	file->private_data = NULL;

	if (cb) {
		txei_free_cb_private(cb);
		cb = NULL;
	}

	kfree(cl);
	mutex_unlock(&dev->device_lock);
	return rets;
}


/**
 * txei_read - the read function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * returns >=0 data length on success , <0 on error
 */
static ssize_t txei_read(struct file *file, char __user *ubuf,
		size_t length, loff_t *offset)
{
	struct txei_cl *cl = file->private_data;
	struct txei_cl_cb *cb_pos = NULL;
	struct txei_cl_cb *cb = NULL;
	struct txei_device *dev;
	int rets;
	int err;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);
	if (dev->dev_state != TXEI_DEV_ENABLED) {
		rets = -ENODEV;
		goto out;
	}

	if (cl->read_cb && cl->read_cb->buf_idx > *offset) {
		cb = cl->read_cb;
		goto copy_buffer;
	} else if (cl->read_cb && cl->read_cb->buf_idx > 0 &&
			cl->read_cb->buf_idx <= *offset) {
		cb = cl->read_cb;
		rets = 0;
		goto free;
	} else if ((!cl->read_cb || !cl->read_cb->buf_idx) &&
			*offset > 0) {
		/*Offset needs to be cleaned for contingous reads*/
		*offset = 0;
		rets = 0;
		goto out;
	}

	err = txei_start_read(dev, cl);
	if (err && err != -EBUSY) {
		txei_err(dev, "txei start read failure with status = %d\n",
			err);
		rets = err;
		goto out;
	}

	if (TXEI_READ_COMPLETE != cl->reading_state &&
	    !waitqueue_active(&cl->rx_wait)) {
		if (file->f_flags & O_NONBLOCK) {
			rets = -EAGAIN;
			goto out;
		}

		mutex_unlock(&dev->device_lock);

		if (wait_event_interruptible(cl->rx_wait,
			(TXEI_READ_COMPLETE == cl->reading_state ||
			TXEI_FILE_INITIALIZING == cl->state ||
			TXEI_FILE_DISCONNECTED == cl->state ||
			TXEI_FILE_DISCONNECTING == cl->state))) {

			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}

		mutex_lock(&dev->device_lock);
		if (TXEI_FILE_INITIALIZING == cl->state ||
			TXEI_FILE_DISCONNECTED == cl->state ||
			TXEI_FILE_DISCONNECTING == cl->state) {

			rets = -EBUSY;
			goto out;
		}
	}

	cb = cl->read_cb;

	if (!cb) {
		rets = -ENODEV;
		goto out;
	}
	if (cl->reading_state != TXEI_READ_COMPLETE) {
		rets = 0;
		goto out;
	}
	/* now copy the data to user space */
copy_buffer:
	txei_dbg(dev, "cb->response_buffer size - %d\n",
		cb->response_buffer.size);
	txei_dbg(dev, "cb->buf_idx - %lu\n", cb->buf_idx);
	if (length == 0 || ubuf == NULL || *offset > cb->buf_idx) {
		rets = -EMSGSIZE;
		goto free;
	}

	/* length is being turncated to PAGE_SIZE, however, */
	/* buf_idx size may be longer */
	length = min_t(size_t, length, (cb->buf_idx - *offset));

	if (copy_to_user(ubuf, cb->response_buffer.data + *offset, length)) {
		rets = -EFAULT;
		goto free;
	}

	rets = length;
	*offset += length;
	if ((unsigned long)*offset < cb->buf_idx)
		goto out;

free:
	cb_pos = find_read_list_entry(dev, cl);
	/* Remove entry from read list */
	if (cb_pos)
		list_del(&cb_pos->cb_list);
	txei_free_cb_private(cb);
	cl->reading_state = TXEI_IDLE;
	cl->read_cb = NULL;
out:
	txei_dbg(dev, "end txei read rets = %d\n", rets);
	mutex_unlock(&dev->device_lock);
	return rets;
}

/**
 * txei_write - the write function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * returns >=0 data length on success , <0 on error
 */
static ssize_t txei_write(struct file *file, const char __user *ubuf,
		size_t length, loff_t *offset)
{
	struct txei_cl *cl = file->private_data;
	struct txei_cl_cb *write_cb = NULL;
	struct txei_msg_hdr txei_hdr;
	struct txei_device *dev;
	int rets;
	int i;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (dev->dev_state != TXEI_DEV_ENABLED) {
		mutex_unlock(&dev->device_lock);
		return -ENODEV;
	}

	/* free entry used in read */
	if (cl->reading_state == TXEI_READ_COMPLETE) {
		*offset = 0;
		write_cb = find_read_list_entry(dev, cl);
		if (write_cb) {
			list_del(&write_cb->cb_list);
			txei_free_cb_private(write_cb);
			write_cb = NULL;
			cl->reading_state = TXEI_IDLE;
			cl->read_cb = NULL;
		}
	} else if (cl->reading_state == TXEI_IDLE)
		*offset = 0;


	write_cb = kzalloc(sizeof(struct txei_cl_cb), GFP_KERNEL);
	if (!write_cb) {
		mutex_unlock(&dev->device_lock);
		return -ENOMEM;
	}

	write_cb->file_object = file;
	write_cb->file_private = cl;
	write_cb->request_buffer.data = kmalloc(length, GFP_KERNEL);
	rets = -ENOMEM;
	if (!write_cb->request_buffer.data)
		goto unlock_dev;

	txei_dbg(dev, "writing = %zd bytes\n", length);

	rets = -EFAULT;
	if (copy_from_user(write_cb->request_buffer.data, ubuf, length))
		goto unlock_dev;

	cl->sm_state = 0;

	INIT_LIST_HEAD(&write_cb->cb_list);

	write_cb->fop_type = TXEI_FOP_WRITE;
	/* make sure buf_idx is zero before we start */

	write_cb->buf_idx = 0;
	write_cb->request_buffer.size = length;

	txei_dbg(dev, "host client = %d, ME client = %d\n",
			cl->host_client_id, cl->me_client_id);
	if (cl->state != TXEI_FILE_CONNECTED) {
		rets = -ENODEV;
		txei_dbg(dev, "host client = %d,  is not connected to ME client = %d",
			cl->host_client_id,
			cl->me_client_id);

		goto unlock_dev;
	}
	for (i = 0; i < dev->me_clients_num; i++) {
		if (dev->me_clients[i].client_id ==
				cl->me_client_id)
			break;
	}
	if (WARN_ON(dev->me_clients[i].client_id != cl->me_client_id)) {
		rets = -ENODEV;
		goto unlock_dev;
	}
	if (i == dev->me_clients_num) {
		rets = -ENODEV;
		goto unlock_dev;
	}
	if (length > dev->me_clients[i].props.max_msg_length || length <= 0) {
		rets = -EINVAL;
		goto unlock_dev;
	}
	write_cb->file_private = cl;

	TXEI_TIME(dev, "txei_flow_ctrl_creds",
		rets = txei_flow_ctrl_creds(dev, cl));

	if (rets < 0)
		goto unlock_dev;

	if (rets && dev->hbuf_is_ready) {
		rets = 0;
		dev->hbuf_is_ready = false;

		if (length >  txei_hbuf_max_len(dev)) {
			txei_hdr.length = txei_hbuf_max_len(dev);
			txei_hdr.msg_complete = 0;
			txei_dbg(dev, "truncate the payload from %zd to %d\n",
				length, txei_hdr.length);
		} else {
			txei_hdr.length = length;
			txei_hdr.msg_complete = 1;
		}
		txei_hdr.host_addr = cl->host_client_id;
		txei_hdr.me_addr = cl->me_client_id;
		txei_hdr.reserved = 0;
		txei_dbg(dev, "call txei_write_message header = %08x.\n",
				*((u32 *) &txei_hdr));
		if (!txei_write_message(dev, &txei_hdr,
			(unsigned char *) write_cb->request_buffer.data,
			txei_hdr.length)) {

			rets = -ENODEV;
			goto unlock_dev;
		}
		cl->writing_state = TXEI_WRITING;
		write_cb->buf_idx = txei_hdr.length;
		if (txei_hdr.msg_complete) {
			if (txei_flow_ctrl_reduce(dev, cl)) {
				rets = -ENODEV;
				goto unlock_dev;
			}
			list_add_tail(&write_cb->cb_list,
				&dev->write_waiting_list.txei_cb.cb_list);
		} else {
			list_add_tail(&write_cb->cb_list,
				&dev->write_list.txei_cb.cb_list);
		}

	} else {
		txei_dbg(dev, "input is not ready queuing write request");
		write_cb->buf_idx = 0;
		cl->writing_state = TXEI_WRITING;
		list_add_tail(&write_cb->cb_list,
			&dev->write_list.txei_cb.cb_list);
	}
	mutex_unlock(&dev->device_lock);
	return length;

unlock_dev:
	txei_free_cb_private(write_cb);
	mutex_unlock(&dev->device_lock);
	return rets;
}


/**
 * txei_ioctl - the IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to txei message structure
 *
 * returns 0 on success , <0 on error
 */
static long txei_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	struct txei_device *dev;
	struct txei_cl *cl = file->private_data;
	struct txei_connect_client_data *connect_data = NULL;
	int rets;


	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	if (cmd != IOCTL_TXEI_CONNECT_CLIENT)
		return -EINVAL;

	dev = cl->dev;

	txei_dbg(dev, "IOCTL cmd = 0x%x", cmd);

	mutex_lock(&dev->device_lock);
	if (dev->dev_state != TXEI_DEV_ENABLED) {
		rets = -ENODEV;
		goto out;
	}

	txei_dbg(dev, ": IOCTL_TXEI_CONNECT_CLIENT.\n");

	connect_data = kzalloc(sizeof(struct txei_connect_client_data),
		GFP_KERNEL);

	if (!connect_data) {
		rets = -ENOMEM;
		goto out;
	}
	txei_dbg(dev, "copy connect data from user\n");
	if (copy_from_user(connect_data, (char __user *)data,
			sizeof(struct txei_connect_client_data))) {
		txei_dbg(dev, "failed to copy data from userland\n");
		rets = -EFAULT;
		goto out;
	}
	/* Note that this call requires device lock to be locked */
	rets = txei_ioctl_connect_client(file, connect_data);

	/* if all is ok, copying the data back to user. */
	if (rets)
		goto out;

	txei_dbg(dev, "copy connect data to user\n");
	if (copy_to_user((char __user *)data, connect_data,
			sizeof(struct txei_connect_client_data))) {
		txei_dbg(dev, "failed to copy data to userland\n");
		rets = -EFAULT;
		goto out;
	}

out:
	if (connect_data != NULL)
		kfree(connect_data);
	mutex_unlock(&dev->device_lock);
	return rets;
}

/**
 * txei_compat_ioctl - the compat IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to txei message structure
 *
 * returns 0 on success , <0 on error
 */
#ifdef CONFIG_COMPAT
static long txei_compat_ioctl(struct file *file,
		unsigned int cmd, unsigned long data)
{
	return txei_ioctl(file, cmd, (unsigned long)compat_ptr(data));
}
#endif


/**
 * txei_poll - the poll function
 *
 * @file: pointer to file structure
 * @wait: pointer to poll_table structure
 *
 * returns poll mask
 */
static unsigned int txei_poll(struct file *file, poll_table *wait)
{
	struct txei_cl *cl = file->private_data;
	struct txei_device *dev;
	unsigned int mask = 0;

	if (WARN_ON(!cl || !cl->dev))
		return mask;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (dev->dev_state != TXEI_DEV_ENABLED)
		goto out;

	mutex_unlock(&dev->device_lock);
	poll_wait(file, &cl->tx_wait, wait);
	mutex_lock(&dev->device_lock);
	if (TXEI_WRITE_COMPLETE == cl->writing_state)
		mask |= (POLLIN | POLLRDNORM);

out:
	mutex_unlock(&dev->device_lock);
	return mask;
}

/*
 *  PCI driver structure
 */
static const struct file_operations txei_fops = {
	.owner = THIS_MODULE,
	.read = txei_read,
	.unlocked_ioctl = txei_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = txei_compat_ioctl,
#endif
	.open = txei_open,
	.release = txei_release,
	.write = txei_write,
	.poll = txei_poll,
};

static struct miscdevice  txei_misc_device = {
		.name = "txei",
		.fops = &txei_fops,
		.minor = MISC_DYNAMIC_MINOR,
};

int txei_register(struct device *dev)
{
	txei_misc_device.parent = dev;
	return misc_register(&txei_misc_device);
}

void txei_deregister(void)
{
	misc_deregister(&txei_misc_device);
	txei_misc_device.parent = NULL;
}

