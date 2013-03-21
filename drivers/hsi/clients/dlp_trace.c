/*
 * dlp_trace.c
 *
 * Intel Mobile Communication protocol driver for modem tracing
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/hsi/hsi.h>
#include <linux/uaccess.h>

#include "dlp_main.h"

#define TRACE_DEVNAME	CONFIG_HSI_TRACE_DEV_NAME
#define HSI_TRACE_TEMP_BUFFERS	4

/*
 * struct trace_driver - HSI Modem trace driver protocol
 *
 * @lock: spinlock to serialise access to the driver information
 * @major: char device major number
 * @tdev: char device type dev
 * @dev: char device
 * @cdev: char device
 * @class: char device class
 * @opened: This flasg is used to allow only ONE instance of this driver
 * @read_wq: Read/Poll/Select wait event
 * @rx_msgs: RX messages queue
 * @ch_ctx: Trace Channel context
*/
struct dlp_trace_ctx {
	/* Char device registration */
	int major;
	dev_t tdev;
	struct device *dev;
	struct cdev cdev;
	struct class *class;

	/* Used to prevent multiple access to device */
	unsigned int opened;
	unsigned int hangup;

	/* A waitqueue for poll/read operations */
	wait_queue_head_t read_wq;

	/* RX msg queue */
	struct list_head rx_msgs;
	int rx_msgs_count;

	struct dlp_channel *ch_ctx;

#ifdef DEBUG
	unsigned long dropped_data_size;
#endif

};


#ifdef DEBUG
/* Used to activate the dump of dropped packets */
static unsigned int log_dropped_data;
module_param_named(log_dropped_data, log_dropped_data, int, S_IRUGO | S_IWUSR);
#endif


/*
 *
 */
static void dlp_trace_complete_rx(struct hsi_msg *msg);


/*
* @brief Called to destroy the allocated msg
*
* @param msg
*/
static inline void dlp_trace_msg_destruct(struct hsi_msg *msg)
{
	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);
}

/*
 * Push RX pdu on channel 0
 *
 */
static int dlp_trace_push_rx_pdu(struct dlp_channel *ch_ctx)
{
	int ret;
	struct hsi_msg *rx_msg;

	/* Allocate a new RX msg */
	rx_msg = dlp_pdu_alloc(ch_ctx->hsi_channel,
				HSI_MSG_READ,
				DLP_TRACE_RX_PDU_SIZE,
				1,
				ch_ctx,
				dlp_trace_complete_rx,
				dlp_trace_msg_destruct);

	if (!rx_msg) {
		pr_err(DRVNAME": dlp_pdu_alloc(RX) failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Send the RX HSI msg */
	ret = hsi_async(rx_msg->cl, rx_msg);
	if (ret) {
		pr_err(DRVNAME": hsi_async() failed, ret:%d\n", ret);
		ret = -EIO;
		goto free_msg;
	}

	return 0;

free_msg:
	/* Free the msg */
	dlp_pdu_free(rx_msg, rx_msg->channel);

out:
	return ret;
}

/*
* @brief Remove & return the first list item (return NULL if empty)
*
* @param ch_ctx
* @param msg
*/
static struct hsi_msg *dlp_trace_peek_msg(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	struct hsi_msg *msg = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);

	if (!list_empty(&trace_ctx->rx_msgs)) {
		/* Get the fist list item (head) */
		msg = list_entry(trace_ctx->rx_msgs.next, struct hsi_msg, link);

		/* Remove the item from the list */
		list_del_init(&msg->link);

		/* Update the counter */
		trace_ctx->rx_msgs_count--;
	}

	spin_unlock_irqrestore(&ch_ctx->lock, flags);
	return msg;
}


/*
* @brief
*
* @param msg
*/
static void dlp_trace_complete_rx(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx = msg->context;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	u32 *header = sg_virt(msg->sgt.sgl);
	unsigned long flags;
	int ret;

	/* Check the link readiness (TTY still opened) */
	if (!dlp_tty_is_link_valid()) {
		pr_debug(DRVNAME ": TRACE: CH%d PDU ignored (close:%d, Time out: %d)\n",
				ch_ctx->ch_id,
				dlp_drv.tty_closed, dlp_drv.tx_timeout);
		return;
	}

	/* Check the PDU status & signature */
	if (msg->status != HSI_STATUS_COMPLETED) {
		pr_err(DRVNAME": Invalid PDU status: %d (ignored)\n",
				msg->status);
		goto push_again;
	} else if (!DLP_HEADER_VALID_SIGNATURE(header[0])) {
		pr_err("\n" DRVNAME ": Invalid PDU signature 0x%x\n",
				header[0]);

		/* Dump the first 64 bytes */
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_LOG", DUMP_PREFIX_OFFSET,
				16, 4,
				header, 64, 1);

		goto push_again;
	}


	/* Still have space in the rx queue ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->rx_msgs_count >= DLP_HSI_RX_WAIT_FIFO) {
		/* Just drop the msg */
		spin_unlock_irqrestore(&ch_ctx->lock, flags);

#ifdef DEBUG
		trace_ctx->dropped_data_size += msg->actual_len;
		if (log_dropped_data)
			pr_debug(DRVNAME ": Packet dropped (dropped data size: %lu Bytes)\n",
					trace_ctx->dropped_data_size);
#endif
		goto push_again;
	}

	/* Update the counter */
	trace_ctx->rx_msgs_count++;

	/* Queue the msg to the read queue */
	list_add_tail(&msg->link, &trace_ctx->rx_msgs);
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Wakeup any waiting clients for read/poll */
	wake_up_interruptible(&trace_ctx->read_wq);
	return;

push_again:
	/* Push again the RX msg */
	ret = hsi_async(msg->cl, msg);
	if (ret) {
		pr_err(DRVNAME": hsi_async failed (%d), FIFO will be empty\n",
				ret);

		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);
	}
}


/*
 * Called when a process tries to open the device file
 */
static int dlp_trace_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0, state, count;
	unsigned long flags;
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_TRACE);
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	/* Check if the the channel is not already opened by the NET IF */
	state = dlp_ctrl_get_channel_state(ch_ctx->hsi_channel);
	if (state != DLP_CH_STATE_CLOSED) {
		pr_err(DRVNAME ": Can't open CH%d (HSI CH%d) => invalid state: %d\n",
				ch_ctx->ch_id, ch_ctx->hsi_channel, state);
		ret = -EBUSY;
		goto out;
	}

	/* Update/Set the eDLP channel id */
	dlp_drv.channels_hsi[ch_ctx->hsi_channel].edlp_channel = ch_ctx->ch_id;

	/* Only ONE instance of this device can be opened */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->opened) {
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		pr_err(DRVNAME": Trace channel ALREADY opened");
		ret = -EBUSY;
		goto out;
	}
	/* Set the open flag */
	trace_ctx->opened = 1;
	/* reset hangup flag */
	trace_ctx->hangup = 0 ;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Save private data for futur use */
	filp->private_data = ch_ctx;


	/* Disable the flow control */
	ch_ctx->use_flow_ctrl = 1;

	/* Reply to any waiting OPEN_CONN command */
	ret = dlp_ctrl_send_ack_nack(ch_ctx);
	if (ret) {
		pr_err(DRVNAME ": ch%d open failed !\n", ch_ctx->ch_id);
		ret = -EIO;
		goto out;
	}

	/* device opened => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_OPENED);

	/* Push RX PDUs */
	count = DLP_HSI_RX_WAIT_FIFO + HSI_TRACE_TEMP_BUFFERS;
	for (ret = count; ret; ret--)
		dlp_trace_push_rx_pdu(ch_ctx);

	pr_debug(DRVNAME": Trace Channel opened");

out:
	return ret;
}

/*
 * Called when a process closes the device file.
 */
static int dlp_trace_dev_close(struct inode *inode, struct file *filp)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	unsigned long flags;

	pr_debug(DRVNAME": Close Trace channel");

	spin_lock_irqsave(&ch_ctx->lock, flags);
	/* set the hangup flag to make the poll function return an error*/
	trace_ctx->hangup = 1;
	/* Set the open flag */
	trace_ctx->opened = 0;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* device closed => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_CLOSED);

	/*
	 * Wake up the read waitqueue to unblock the poll_wait
	 */
	wake_up_interruptible(&trace_ctx->read_wq);

	return 0;
}

/*
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 */
static ssize_t dlp_trace_dev_read(struct file *filp,
			   char __user *data,
			   size_t count,
			   loff_t *ppos)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	struct hsi_msg *msg;
	int ret, to_copy, copied, available, more_packets;
	unsigned int data_size, offset;
	unsigned char *data_addr, *start_addr;
	unsigned int *ptr;
	unsigned long flags;

	/* Check the user buffer size */
	if (count < DLP_TRACE_RX_PDU_SIZE) {
		pr_err(DRVNAME": Too small read buffer size %d, should be >= %d\n",
				count, DLP_TRACE_RX_PDU_SIZE);
		return -EINVAL;
	}

	/* Have some data to read ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	ret = list_empty(&trace_ctx->rx_msgs);
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* List empty ? */
	if (ret) {
		/* Descriptor opened in Non-Blocking mode ? */
		if (filp->f_flags & O_NONBLOCK) {
			copied = -EWOULDBLOCK;
			goto out;
		} else {
			ret = wait_event_interruptible(trace_ctx->read_wq, 0);
			if (ret) {
				copied = -EINTR;
				goto out;
			}
		}
	}

	copied = 0;
	available = count;

	/* Parse RX msgs queue */
	while (available) {
		msg = dlp_trace_peek_msg(ch_ctx);
		if (!msg)
			break;

		ptr = sg_virt(msg->sgt.sgl);
		start_addr = (unsigned char *)ptr;

		do {
			/* Get the start offset */
			ptr++;
			offset = (*ptr);

			/* Get the size & address */
			ptr++;
			more_packets = (*ptr) & DLP_HDR_MORE_DESC;
			data_size  = DLP_HDR_DATA_SIZE((*ptr));
			data_size -= DLP_HDR_SPACE_AP;
			data_addr = start_addr + offset + DLP_HDR_SPACE_AP;

			/* Calculate the data size */
			to_copy = MIN(data_size, available);

			/* Copy data to the user buffer */
			ret = copy_to_user(data+copied, data_addr, to_copy);
			if (ret) {
				/* Stop copying */
				pr_err(DRVNAME": Unable to copy data to the user buffer\n");
				break;
			}

			copied += to_copy;
			available -= to_copy;
		} while ((more_packets) && (available));

		/* Read done => Queue the RX msg again */
		ret = hsi_async(msg->cl, msg);
		if (ret) {
			pr_err(DRVNAME": hsi_async failed (%d) FIFO will be empty\n",
					ret);

			/* Delete the received msg */
			dlp_pdu_free(msg, msg->channel);
		}
	}

	/* Update the position */
	(*ppos) += copied;

out:
	return copied;
}

/*
 * Called when a process writes to dev file
 */
static ssize_t dlp_trace_dev_write(struct file *filp,
		const char __user *data,
		size_t count,
		loff_t *ppos)
{
	pr_err(DRVNAME": Modem trace TX path is not allowed !\n");
	return 0;
}


/*
* @brief
*
* @param filp
* @param wait
*
* @return
*/
static unsigned int dlp_trace_dev_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	unsigned long flags;
	unsigned int ret = 0;

	/* if the channel is hang-up/closed no reason to wait*/
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->hangup) {
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		ret = POLLHUP;
		goto out;
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	poll_wait(filp, &trace_ctx->read_wq, pt);

	/* Have some data to read ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->hangup) {
		/* The close function has been executed <=> hangup */
		ret = POLLHUP;
	} else if (!list_empty(&trace_ctx->rx_msgs)) {
		ret = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

out:
	return ret;
}

/**
 * dlp_net_hsi_tx_timeout_cb - Called when we have an HSI TX timeout
 * @ch_ctx : Channel context ref
 */
static void dlp_trace_dev_tx_timeout_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	/* Set hangup flag */
	trace_ctx->hangup = 1 ;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
	wake_up_interruptible(&trace_ctx->read_wq);

}


/*
* Device driver file operations
*/
static const struct file_operations dlp_trace_ops = {
	.open	= dlp_trace_dev_open,
	.read	= dlp_trace_dev_read,
	.write	= dlp_trace_dev_write,
	.poll	= dlp_trace_dev_poll,
	.release = dlp_trace_dev_close
};

static int dlp_trace_ctx_cleanup(struct dlp_channel *ch_ctx);

/*
* @brief
*
* @param ch_id
* @param hsi_channel
* @param dev
*
* @return
*/
struct dlp_channel *dlp_trace_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
{
	int ret;
	struct hsi_client *client = to_hsi_client(dev);
	struct dlp_channel *ch_ctx;
	struct dlp_trace_ctx *trace_ctx;

	/* Allocate channel struct data */
	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		pr_err(DRVNAME": Out of memory (ch%d)\n", ch_id);
		return NULL;
	}

	/* Allocate the context private data */
	trace_ctx = kzalloc(sizeof(struct dlp_trace_ctx), GFP_KERNEL);
	if (!trace_ctx) {
		pr_err(DRVNAME": Out of memory (trace_ctx)\n");
		goto free_ch;
	}

	/* Save params */
	ch_ctx->ch_data = trace_ctx;
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&trace_ctx->read_wq);
	INIT_LIST_HEAD(&trace_ctx->rx_msgs);

	/* Register debug, cleanup CBs */
	ch_ctx->dump_state = dlp_dump_channel_state;
	ch_ctx->cleanup = dlp_trace_ctx_cleanup;

	/* Hangup context */
	dlp_ctrl_hangup_ctx_init(ch_ctx, dlp_trace_dev_tx_timeout_cb);

	/* Init the RX/TX contexts */
	dlp_xfer_ctx_init(ch_ctx,
			DLP_TRACE_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			DLP_TRACE_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Register the device */
	ret = alloc_chrdev_region(&trace_ctx->tdev, 0, 1, TRACE_DEVNAME);
	if (ret) {
		pr_err(DRVNAME": Unable to allocate the device (err: %d)\n",
				ret);
		goto free_ctx;
	}

	trace_ctx->major = MAJOR(trace_ctx->tdev);
	cdev_init(&trace_ctx->cdev, &dlp_trace_ops);
	trace_ctx->cdev.owner = THIS_MODULE;

	ret = cdev_add(&trace_ctx->cdev, trace_ctx->tdev, 1);
	if (ret) {
		pr_err(DRVNAME": Unable to register the device (err: %d)\n",
				ret);
		goto unreg_reg;
	}

	trace_ctx->class = class_create(THIS_MODULE, DRVNAME"-trace");
	if (IS_ERR(trace_ctx->class))
		goto del_cdev;

	trace_ctx->dev = device_create(trace_ctx->class,
			NULL,
			trace_ctx->tdev,
			NULL, TRACE_DEVNAME);
	if (IS_ERR(trace_ctx->dev)) {
		pr_err(DRVNAME": Unable to create the device (err: %ld)\n",
				PTR_ERR(trace_ctx->dev));
		goto del_class;
	}

	return ch_ctx;

del_class:
	class_destroy(trace_ctx->class);

del_cdev:
	cdev_del(&trace_ctx->cdev);

unreg_reg:
	unregister_chrdev_region(trace_ctx->tdev, 1);

free_ctx:
	kfree(trace_ctx);

free_ch:
	kfree(ch_ctx);
	return NULL;
}

/*
* @brief This function will delete/unregister
*	the char device and class
*
* @param ch_ctx: Trace channel context
*
* @return 0 when sucess, error code otherwise
*/
static int dlp_trace_ctx_cleanup(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	int ret = 0;

	/* Unregister/Delete char device & class */
	device_destroy(trace_ctx->class, trace_ctx->tdev);
	cdev_del(&trace_ctx->cdev);
	unregister_chrdev_region(trace_ctx->tdev, 1);
	class_destroy(trace_ctx->class);

	return ret;
}

/*
 * This function will release the allocated memory
 * done in the _ctx_create function
 */
int dlp_trace_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	/* Free the Trace context */
	kfree(trace_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);
	return 0;
}

