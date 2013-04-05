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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/uuid.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>

#include "txei.h"

#include "txei_dev.h"
#include "hw.h"
#include "hw-txe.h"

/**
 * txei_ioctl_connect_client - the connect to fw client IOCTL function
 *
 * @dev: the device structure
 * @data: IOCTL connect data, input and output parameters
 * @file: private data of the file object
 *
 * Note that this function requires device_lock to be locked
 * at invocation and then leaves it locked upon exit. It
 * unlocks device_lock for the wait event timeout call.
 *
 * Locking: called under "dev->device_lock" lock
 *
 * returns 0 on success, <0 on failure.
 */
int txei_ioctl_connect_client(struct file *file,
			struct txei_connect_client_data *data)
{
	struct txei_device *dev;
	struct txei_cl_cb *cb;
	struct txei_client *client;
	struct txei_cl *cl;
	long timeout = TXEI_CONNECT_TIMEOUT;
	int i;
	int err;
	int rets;

	cl = file->private_data;
	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	txei_dbg(dev, "txei_ioctl_connect_client() Entry\n");


	/* buffered ioctl cb */
	cb = kzalloc(sizeof(struct txei_cl_cb), GFP_KERNEL);
	if (!cb) {
		rets = -ENOMEM;
		goto end;
	}
	INIT_LIST_HEAD(&cb->cb_list);

	cb->fop_type = TXEI_FOP_IOCTL;

	if (dev->dev_state != TXEI_DEV_ENABLED) {
		rets = -ENODEV;
		goto end;
	}
	if (cl->state != TXEI_FILE_INITIALIZING &&
	    cl->state != TXEI_FILE_DISCONNECTED) {
		rets = -EBUSY;
		goto end;
	}

	/* find ME client we're trying to connect to */
	i = txei_find_me_client_index(dev, data->in_client_uuid);
	if (i >= 0 && !dev->me_clients[i].props.fixed_address) {
		cl->me_client_id = dev->me_clients[i].client_id;
		cl->state = TXEI_FILE_CONNECTING;
	}

	txei_dbg(dev, "Connect to FW Client ID = %d\n", cl->me_client_id);
	txei_dbg(dev, "FW Client - Protocol Version = %d\n",
			dev->me_clients[i].props.protocol_version);
	txei_dbg(dev, "FW Client - Max Msg Len = %d\n",
			dev->me_clients[i].props.max_msg_length);

	if (cl->state != TXEI_FILE_CONNECTING) {
		rets = -ENODEV;
		goto end;
	}


	/* prepare the output buffer */
	client = &data->out_client_properties;
	client->max_msg_length = dev->me_clients[i].props.max_msg_length;
	client->protocol_version = dev->me_clients[i].props.protocol_version;
	txei_dbg(dev, "Can connect?\n");
	if (dev->hbuf_is_ready && !txei_other_client_is_connecting(dev, cl)) {
		txei_dbg(dev, "Sending Connect Message\n");
		dev->hbuf_is_ready = false;
		if (!txei_connect(dev, cl)) {
			txei_dbg(dev, "Sending connect message - failed\n");
			rets = -ENODEV;
			goto end;
		}
		txei_dbg(dev, "Sending connect message - succeeded\n");
		cl->timer_count = TXEI_CONNECT_TIMEOUT;
		cb->file_private = cl;
		list_add_tail(&cb->cb_list, &dev->ctrl_rd_list.txei_cb.cb_list);

	} else {
		txei_dbg(dev, "Queuing the connect request due to device busy\n");
		cb->file_private = cl;
		txei_dbg(dev, "add connect cb to control write list.\n");
		list_add_tail(&cb->cb_list, &dev->ctrl_wr_list.txei_cb.cb_list);
	}
	mutex_unlock(&dev->device_lock);
	err = wait_event_timeout(dev->wait_recvd_msg,
			(TXEI_FILE_CONNECTED == cl->state ||
			 TXEI_FILE_DISCONNECTED == cl->state),
			timeout * HZ);

	mutex_lock(&dev->device_lock);
	if (TXEI_FILE_CONNECTED == cl->state) {
		txei_dbg(dev, "successfully connected to FW client.\n");
		rets = cl->status;
		goto end;
	} else {
		txei_dbg(dev, "failed to connect to FW client.cl->state = %d.\n",
			cl->state);

		if (!err)
			txei_dbg(dev, "wait_event_interruptible_timeout failed on client connect message fw response message.\n");

		rets = -EFAULT;

		txei_io_list_flush(&dev->ctrl_rd_list, cl);
		txei_io_list_flush(&dev->ctrl_wr_list, cl);
		goto end;
	}
	rets = 0;
end:
	txei_dbg(dev, "free connect cb memory.");
	if (cb != NULL)
		kfree(cb);
	return rets;
}

/**
 * txei_start_read - the start read client message function.
 *
 * @dev: the device structure
 * @if_num:  minor number
 * @cl: private data of the file object
 * This will need to have device lock locked upon entry
 *
 * returns 0 on success, <0 on failure.
 */
int txei_start_read(struct txei_device *dev, struct txei_cl *cl)
{
	struct txei_cl_cb *cb;
	int rets = 0;
	int i;

	if (cl->state != TXEI_FILE_CONNECTED)
		return -ENODEV;

	if (dev->dev_state != TXEI_DEV_ENABLED)
		return -ENODEV;

	txei_dbg(dev, "check if read is pending.\n");
	if (cl->read_cb) {
		txei_dbg(dev, "read is pending.\n");
		return -EBUSY;
	}

	cb = kzalloc(sizeof(struct txei_cl_cb), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	txei_dbg(dev, "allocation call back successful. host client = %d, ME client = %d\n",
		cl->host_client_id, cl->me_client_id);

	for (i = 0; i < dev->me_clients_num; i++) {
		if (dev->me_clients[i].client_id == cl->me_client_id)
			break;

	}

	if (WARN_ON(dev->me_clients[i].client_id != cl->me_client_id)) {
		rets = -ENODEV;
		goto unlock;
	}

	if (i == dev->me_clients_num) {
		rets = -ENODEV;
		goto unlock;
	}

	cb->response_buffer.size = dev->me_clients[i].props.max_msg_length;
	cb->response_buffer.data =
		kmalloc(cb->response_buffer.size, GFP_KERNEL);
	if (!cb->response_buffer.data) {
		rets = -ENOMEM;
		goto unlock;
	}
	txei_dbg(dev, "allocation call back data success.\n");
	cb->fop_type = TXEI_FOP_READ;
	/* make sure buf_idx is zero before we start */
	cb->buf_idx = 0;
	cb->file_private = cl;
	cl->read_cb = cb;
	if (dev->hbuf_is_ready) {
		dev->hbuf_is_ready = false;
		if (!txei_send_flow_control(dev, cl)) {
			rets = -ENODEV;
			goto unlock;
		} else {
			list_add_tail(&cb->cb_list,
				      &dev->read_list.txei_cb.cb_list);
		}
	} else {
		list_add_tail(&cb->cb_list,
			      &dev->ctrl_wr_list.txei_cb.cb_list);
	}
	return rets;
unlock:
	txei_free_cb_private(cb);
	return rets;
}

/**
 * txei_free_cb_private - free txei_cb_private related memory
 *
 * @cb: txei callback struct
 */
void txei_free_cb_private(struct txei_cl_cb *cb)
{
	if (cb == NULL)
		return;

	if (cb->request_buffer.data != NULL)
		kfree(cb->request_buffer.data);
	if (cb->response_buffer.data != NULL)
		kfree(cb->response_buffer.data);
	kfree(cb);
}
