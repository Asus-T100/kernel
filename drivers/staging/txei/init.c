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
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include "txei_dev.h"
#include "hw.h"
#include "hw-txe.h"

const char *txei_dev_state_str(int state)
{
	switch (state) {
	case TXEI_DEV_INITIALIZING:
		return "TXEI_DEV_INITIALIZING";
	case TXEI_DEV_INIT_CLIENTS:
		return "TXEI_DEV_INIT_CLIENTS";
	case TXEI_DEV_ENABLED:
		return "TXEI_DEV_ENABLED";
	case TXEI_DEV_RESETING:
		return "TXEI_DEV_RESETING";
	case TXEI_DEV_DISABLED:
		return "TXEI_DEV_DISABLED";
	case TXEI_DEV_RECOVERING_FROM_RESET:
		return "TXEI_DEV_RECOVERING_FROM_RESET";
	case TXEI_DEV_POWER_DOWN:
		return "TXEI_DEV_POWER_DOWN";
	case TXEI_DEV_POWER_UP:
		return "TXEI_DEV_POWER_UP";
	default:
		return "unkown";
	}
}


/**
 * txei_io_list_init - Sets up a queue list.
 *
 * @list: An instance io list structure
 * @dev: the device structure
 */
void txei_io_list_init(struct txei_io_list *list)
{
	/* initialize our queue list */
	INIT_LIST_HEAD(&list->txei_cb.cb_list);
	list->status = 0;
}

/**
 * txei_io_list_flush - removes list entry belonging to cl.
 *
 * @list:  An instance of our list structure
 * @cl: private data of the file object
 */
void txei_io_list_flush(struct txei_io_list *list, struct txei_cl *cl)
{
	struct txei_cl_cb *cb_pos = NULL;
	struct txei_cl_cb *cb_next = NULL;

	if (list->status != 0)
		return;

	if (list_empty(&list->txei_cb.cb_list))
		return;

	list_for_each_entry_safe(cb_pos, cb_next,
				 &list->txei_cb.cb_list, cb_list) {
		struct txei_cl *cl_tmp = cb_pos->file_private;
		if (txei_cl_cmp_id(cl, cl_tmp))
			list_del(&cb_pos->cb_list);
	}
}
/**
 * txei_cl_flush_queues - flushes queue lists belonging to cl.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 */
int txei_cl_flush_queues(struct txei_cl *cl)
{
	if (!cl || !cl->dev)
		return -EINVAL;

	txei_dbg(cl->dev, "remove list entry belonging to cl\n");
	txei_io_list_flush(&cl->dev->read_list, cl);
	txei_io_list_flush(&cl->dev->write_list, cl);
	txei_io_list_flush(&cl->dev->write_waiting_list, cl);
	txei_io_list_flush(&cl->dev->ctrl_wr_list, cl);
	txei_io_list_flush(&cl->dev->ctrl_rd_list, cl);
	return 0;
}


/**
 * init_txei_device - allocates and initializes the txei device structure
 *
 * @pdev: The pci device structure
 *
 * returns The txei_device_device pointer on success, NULL on failure.
 */
struct txei_device *txei_device_init(struct pci_dev *pdev)
{
	struct txei_device *dev;

	dev = kzalloc(sizeof(struct txei_device), GFP_KERNEL);
	if (!dev)
		return NULL;

	/* setup our list array */
	INIT_LIST_HEAD(&dev->file_list);
	mutex_init(&dev->device_lock);
	init_waitqueue_head(&dev->wait_recvd_msg);
	init_waitqueue_head(&dev->wait_aliveness_resp);
	init_waitqueue_head(&dev->wait_readiness_int);
	init_waitqueue_head(&dev->wait_polling_thread_terminated);

	dev->dev_state = TXEI_DEV_INITIALIZING;
	dev->aliveness_timeout = SEC_ALIVENESS_TIMER_TIMEOUT;

	txei_io_list_init(&dev->read_list);
	txei_io_list_init(&dev->write_list);
	txei_io_list_init(&dev->write_waiting_list);
	txei_io_list_init(&dev->ctrl_wr_list);
	txei_io_list_init(&dev->ctrl_rd_list);
	dev->pdev = pdev;

	return dev;
}

/**
 * txei_hw_init - initializes host and fw to start work.
 *
 * @dev: the device structure
 *
 * returns 0 on success, <0 on failure.
 */
int txei_hw_init(struct txei_device *dev)
{
	int err = 0;
	int ret;

	/**
	 * We are locking device_lock because the txei_reset
	 * function has a place where the device lock is unlocked
	 * and re-locked around a sleep. Since txei_reset is used by
	 * other functions that have a need for a lock, we have to
	 * do the same here
	 */
	mutex_lock(&dev->device_lock);

	dev->aliveness = ipc_aliveness_get(dev);
	dev->readiness_state = ipc_readiness_get(dev);

	txei_dbg(dev, "aliveness_resp = 0x%08x, readiness = 0x%08x.\n",
		dev->aliveness, dev->readiness_state);

	/* acknowledge interrupt and stop interupts */
	ipc_interrupts_clear(dev);

	dev->recvd_msg = false;
	dev->recvd_aliv_resp = false;
	dev->recvd_readiness_int = false;

	INIT_DELAYED_WORK(&dev->aliveness_timer, ipc_aliveness_timer);

	txei_dbg(dev, "reset in start the txei device.\n");
	txei_reset(dev, true);

	txei_dbg(dev, "aliveness_resp = 0x%08x, readiness = 0x%08x.\n",
	    dev->aliveness, dev->readiness_state);

	/* wait for ME to turn on ME_RDY */
	if (!dev->recvd_msg) {
		mutex_unlock(&dev->device_lock);
		err = wait_event_interruptible_timeout(dev->wait_recvd_msg,
			dev->recvd_msg,
			msecs_to_jiffies(MSEC_PER_SEC * TXEI_INTEROP_TIMEOUT));
		mutex_lock(&dev->device_lock);
	}

	if (err <= 0 && !dev->recvd_msg) {
		dev->dev_state = TXEI_DEV_DISABLED;
		txei_dbg(dev, "wait_event_interruptible_timeout failed on wait for ME to turn on ME_RDY. err:0x%08x\n",
			err);
		ret = -ENODEV;
		goto out;
	}
	if (!ipc_readiness_is_host_rdy(dev)) {
		dev->dev_state = TXEI_DEV_DISABLED;
		txei_dbg(dev, "readiness = x%08x.\n", dev->readiness_state);

		txei_err(dev, "link layer initialization failed.\n");
		ret = -ENODEV;
		goto out;
	}

	if (dev->version.major_version != HBM_MAJOR_VERSION ||
	    dev->version.minor_version != HBM_MINOR_VERSION) {
		txei_dbg(dev, "TXEI start failed.\n");
		ret = -ENODEV;
		goto out;
	}

	dev->recvd_msg = false;
	txei_dbg(dev, "aliveness_resp = 0x%08x, readiness = 0x%08x.\n",
	    dev->aliveness, dev->readiness_state);
	txei_dbg(dev, "ME turn on ME_RDY and host turn on H_RDY.\n");
	txei_dbg(dev, "link layer has been established.\n");
	txei_dbg(dev, "TXEI  start success.\n");
	ret = 0;

out:
	mutex_unlock(&dev->device_lock);

	return ret;
}

/**
 * txei_reset - resets host and fw.
 *
 * @dev: the device structure
 * Please note that the dev->device_lock needs to be
 * set when this function is invoked. There is a place
 * where device_lock is unlocked to sleep and then relocked
 * upon awakening
 * Also, the ipc_readiness_wait and ipc_aliveness_poll require
 * device lock to be locked
 * For example, the txei_interrupt_thread_handler sets the
 * device_lock prior to calling the parent functions for
 * this function
 * @interrupts_enabled: if interrupt should be enabled after reset.
 */
void txei_reset(struct txei_device *dev, bool intr_en)
{
	struct txei_cl *cl_pos = NULL;
	struct txei_cl *cl_next = NULL;
	struct txei_cl_cb *cb_pos = NULL;
	struct txei_cl_cb *cb_next = NULL;
	u32 aliveness_req;
	u32 interruptStatus;
	bool unexpected;

	unexpected = (dev->dev_state != TXEI_DEV_INITIALIZING &&
			dev->dev_state != TXEI_DEV_DISABLED &&
			dev->dev_state != TXEI_DEV_POWER_DOWN &&
			dev->dev_state != TXEI_DEV_POWER_UP);

	/*
	 * HPS: 3.1.1.1-2
	 * read input doorbell to ensure consistency between  Bridge and SeC
	 * return value might be garbage return
	 */
	(void)ipc_sec_reg_read_silent(dev, SEC_IPC_INPUT_DOORBELL_OFFSET);

	/* Cancle aliveness machine */
	cancel_delayed_work_sync(&dev->aliveness_timer);

	aliveness_req = ipc_aliveness_req_get(dev);
	dev->aliveness = ipc_aliveness_get(dev);

	/* Disable interrupts in this stage we will poll */
	ipc_host_interrupts_disable(dev);

	/*
	 * HPS: 3.1.1.1-3
	 * If Aliveness Request and Aliveness Response are not equal then
	 * wait for them to be equal
	 * Since we might have interrupts disabled - poll for it
	 * Note that this call requires device lock to be locked
	 */
	if (aliveness_req != dev->aliveness)
		if (ipc_aliveness_poll(dev, aliveness_req) < 0) {
			txei_dbg(dev, "wait for aliveness failed ... bailing out\n");
			return;
		}


	/*
	 * HPS: 3.1.1.1-4
	 * If Aliveness Request and Aliveness Response are set then clear them
	 * Note that this call requires device lock to be locked
	 */
	if (aliveness_req) {
		ipc_aliveness_set(dev, 0);
		if (ipc_aliveness_poll(dev, 0) < 0) {
			txei_dbg(dev, "wait for aliveness failed ... bailing out\n");
			return;
		}
	}

	/*
	 * HPS: 3.1.1.1-5
	 * Set rediness RDY_CLR bit
	 */
	ipc_readiness_clear(dev);

	/*
	 * HPS: 3.1.1.1-6
	 * Host removes any open HECI connections
	 */
	if (dev->dev_state != TXEI_DEV_INITIALIZING) {

		if (dev->dev_state != TXEI_DEV_DISABLED &&
		    dev->dev_state != TXEI_DEV_POWER_DOWN)
			dev->dev_state = TXEI_DEV_RESETING;

		list_for_each_entry_safe(cl_pos, cl_next,
			&dev->file_list, link) {

			cl_pos->state = TXEI_FILE_DISCONNECTED;
			cl_pos->txei_flow_ctrl_creds = 0;
			cl_pos->read_cb = NULL;
			cl_pos->timer_count = 0;
		}
		dev->extra_write_index = 0;
	}

	dev->me_clients_num = 0;
	dev->rd_msg_hdr = 0;
	dev->stop = false;

	/*
	 * HPS: 3.1.1.1-7
	 * SATT2_CTRL.ENTRY_VLD clear - not set here
	 */

	/* In case host initiated the reset flow due
	 * to a case of the driver being disabled,
	 * or entry into D3 state, or OS unload,
	 * the flow ends here for host side */
	if (!intr_en) {
		txei_dbg(dev, "irq not enabled end of reset\n");
		return;
	}

	/* bring back interrupts */

	ipc_host_interrupts_enable(dev);

	/* Note that this call requires device lock to be locked */
	if (ipc_readiness_wait(dev) < 0) {
		txei_err(dev, "polling for readiness failed\n");
		return;
	}

	/* HPS: 3.1.1.1-10
	 * If HISR.INT2_STS interrupt status bit is set then clear it. */
	interruptStatus = ipc_bridge_reg_read(dev, HISR_OFFSET);
	if (interruptStatus  & TXEI_HISR_INT_2_STS)
		ipc_bridge_reg_write(dev, HISR_OFFSET, TXEI_HISR_INT_2_STS);

	/* Clear the interrupt cause of OutputDoorbell */
	dev->ipc_interrupt_cause.OutputDoorbellInt = 0;

	/* HPS: 3.1.1.1-11
	 * set SATT2 - not here */

	/* HPS: 3.1.1.1-12 */
	/* enable input ready interrupts:
	 * SEC_IPC_HOST_INT_MASK.IPC_INPUT_READY_INT_MASK
	 */
	ipc_input_ready_interrupt_enable(dev);

	/* HPS: 3.1.1.1-13 */
	/*  Set the SICR_SEC_IPC_OUTPUT_STATUS.IPC_OUTPUT_READY bit */
	ipc_output_ready_set(dev);

	ipc_aliveness_set(dev, 1);
	/* Note that this call requires device lock to be locked */
	if (ipc_aliveness_wait(dev, 1) < 0) {
		txei_dbg(dev, "wait for aliveness failed ... bailing out\n");
		return;
	}

	/* HPS: 3.1.1.1-14 */
	/* Set bit SICR_HOST_IPC_READINESS.HOST_RDY
	 */
	ipc_readiness_set_host_rdy(dev);

	dev->dev_state = TXEI_DEV_INIT_CLIENTS;
	txei_host_start_message(dev);

	if (unexpected) {
		dev_warn(&dev->pdev->dev, "unexpected reset: dev_state = %s\n",
			txei_dev_state_str(dev->dev_state));
	}

	/* Wake up all readings so they can be interrupted */
	list_for_each_entry_safe(cl_pos, cl_next, &dev->file_list, link) {
		if (waitqueue_active(&cl_pos->rx_wait)) {
			txei_dbg(dev, "Waking up client!\n");
			wake_up_interruptible(&cl_pos->rx_wait);
		}
	}
	/* remove all waiting requests */
	if (dev->write_list.status == 0) {
		list_for_each_entry_safe(cb_pos, cb_next,
				&dev->write_list.txei_cb.cb_list, cb_list) {
			list_del(&cb_pos->cb_list);
			txei_free_cb_private(cb_pos);
		}
	}
}

/**
 * txei_setup_satt2 - SATT2 configuration for DMA support.
 *
 * @dev:  the device structure
 * @hi:   high physical DMA address bits (4 bits significant).
 * @lo:   low  physical DMA address bits
 * @size: DMA range size
 */
void txei_setup_satt2(struct txei_device *dev, dma_addr_t addr, u32 dma_range)
{
	u32 dma_lo32 = addr & 0xFFFFFFFF;
	u32 dma_hi = (sizeof(addr) > sizeof(u32) ? (addr >> 16) >> 16 : 0)
		& 0xF;
	u32 ctrl = SATT2_CTRL_VALID_MSK |
		dma_hi  << SATT2_CTRL_BRIDGE_BASE_ADDR_OFFSET;

	ipc_bridge_reg_write(dev, SATT2_SAP_SIZE_OFFSET, dma_range);
	ipc_bridge_reg_write(dev, SATT2_BRG_BA_LSB_OFFSET, dma_lo32);
	ipc_bridge_reg_write(dev, SATT2_CTRL_OFFSET, ctrl);
	txei_dbg(dev, "SATT2: SAP_SIZE_OFFSET=0x%08X, BRG_BA_LSB_OFFSET=0x%08X, CTRL_OFFSET=0x%08X\n",
		dma_range, dma_lo32, ctrl);
}

/**
 * host_start_message - txei host sends start message.
 *
 * @dev: the device structure
 *
 * returns none.
 */
void txei_host_start_message(struct txei_device *dev)
{
	struct txei_msg_hdr *txei_hdr;
	struct hbm_host_version_request *host_start_req;

	/* host start message */
	txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
	txei_hdr->host_addr = 0;
	txei_hdr->me_addr = 0;
	txei_hdr->length = sizeof(struct hbm_host_version_request);
	txei_hdr->msg_complete = 1;
	txei_hdr->reserved = 0;

	host_start_req =
	    (struct hbm_host_version_request *) &dev->wr_msg_buf[1];
	memset(host_start_req, 0, sizeof(struct hbm_host_version_request));
	host_start_req->hbm_cmd = HOST_START_REQ_CMD;
	host_start_req->host_version.major_version = HBM_MAJOR_VERSION;
	host_start_req->host_version.minor_version = HBM_MINOR_VERSION;
	dev->recvd_msg = false;
	if (!txei_write_message(dev, txei_hdr,
				       (unsigned char *) (host_start_req),
				       txei_hdr->length)) {
		txei_dbg(dev, "write send version message to FW fail.\n");
		dev->dev_state = TXEI_DEV_RESETING;
		txei_reset(dev, true);
	}
	dev->init_clients_state = TXEI_START_MESSAGE;
	dev->init_clients_timer = TXEI_CLIENTS_INIT_TIMEOUT;

}

/**
 * host_enum_clients_message - host sends enumeration client request message.
 *
 * @dev: the device structure
 *
 * returns none.
 */
void txei_host_enum_clients_message(struct txei_device *dev)
{
	struct txei_msg_hdr *txei_hdr;
	struct hbm_host_enum_request *host_enum_req;

	txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
	/* enumerate clients */
	txei_hdr->host_addr = 0;
	txei_hdr->me_addr = 0;
	txei_hdr->length = sizeof(struct hbm_host_enum_request);
	txei_hdr->msg_complete = 1;
	txei_hdr->reserved = 0;

	host_enum_req = (struct hbm_host_enum_request *) &dev->wr_msg_buf[1];
	memset(host_enum_req, 0, sizeof(struct hbm_host_enum_request));
	host_enum_req->hbm_cmd = HOST_ENUM_REQ_CMD;
	if (!txei_write_message(dev, txei_hdr,
			       (unsigned char *) (host_enum_req),
				txei_hdr->length)) {
		dev->dev_state = TXEI_DEV_RESETING;
		txei_dbg(dev, "write send enumeration request message to FW fail.\n");
		txei_reset(dev, true);
	}
	dev->init_clients_state = TXEI_ENUM_CLIENTS_MESSAGE;
	dev->init_clients_timer = TXEI_CLIENTS_INIT_TIMEOUT;

}


/**
 * allocate_me_clients_storage - allocates storage for me clients
 *
 * @dev: the device structure
 *
 * returns none.
 */
/* FIXME: return something */
void txei_allocate_me_clients_storage(struct txei_device *dev)
{
	struct txei_me_client *clients;
	int b;

	/* count how many ME clients we have */
	for_each_set_bit(b, dev->me_clients_map, TXEI_CLIENTS_MAX)
		dev->me_clients_num++;

	if (dev->me_clients_num <= 0)
		return;

	if (dev->me_clients != NULL) {
		kfree(dev->me_clients);
		dev->me_clients = NULL;
	}

	txei_dbg(dev, "memory allocation for ME %d clients size = %zd.\n",
		dev->me_clients_num,
		dev->me_clients_num * sizeof(struct txei_me_client));
	/* allocate storage for ME clients representation */
	clients = kcalloc(dev->me_clients_num,
			sizeof(struct txei_me_client), GFP_KERNEL);
	if (!clients) {
		txei_dbg(dev, "memory allocation for ME clients failed.\n");
		dev->dev_state = TXEI_DEV_RESETING;
		txei_reset(dev, true);
		return ;
	}
	dev->me_clients = clients;
	return;
}
/**
 * host_client_properties - reads properties for client
 *
 * @dev: the device structure
 *
 * returns none.
 */
void txei_host_client_properties(struct txei_device *dev)
{
	struct txei_msg_hdr *txei_header;
	struct hbm_props_request *host_cli_req;
	int b;
	u8 client_num = dev->me_client_presentation_num;

	b = dev->me_client_index;
	b = find_next_bit(dev->me_clients_map, TXEI_CLIENTS_MAX, b);
	if (b < TXEI_CLIENTS_MAX) {
		dev->me_clients[client_num].client_id = b;
		dev->me_clients[client_num].txei_flow_ctrl_creds = 0;
		txei_header = (struct txei_msg_hdr *)&dev->wr_msg_buf[0];
		txei_header->host_addr = 0;
		txei_header->me_addr = 0;
		txei_header->length = sizeof(struct hbm_props_request);
		txei_header->msg_complete = 1;
		txei_header->reserved = 0;

		host_cli_req = (struct hbm_props_request *)&dev->wr_msg_buf[1];

		memset(host_cli_req, 0, sizeof(struct hbm_props_request));

		host_cli_req->hbm_cmd = HOST_CLIENT_PROPERTIES_REQ_CMD;
		host_cli_req->address = b;

		if (!txei_write_message(dev, txei_header,
				(unsigned char *)host_cli_req,
				txei_header->length)) {
			dev->dev_state = TXEI_DEV_RESETING;
			txei_dbg(dev, "write send enumeration request message to FW fail.\n");
			txei_reset(dev, true);
			return;
		}

		dev->init_clients_timer = TXEI_CLIENTS_INIT_TIMEOUT;
		dev->me_client_index = b;

		return;
	}


	/*
	 * Clear Map for indicating now ME clients
	 * with associated host client
	 */
	bitmap_zero(dev->host_clients_map, TXEI_CLIENTS_MAX);
	dev->open_handle_count = 0;
	bitmap_set(dev->host_clients_map, 0, 3);
	/* There is no AMTHI or WD in IPC */

	dev->dev_state = TXEI_DEV_ENABLED;
	/* we are done we can go sleep */
	if (nopg == false && dev->aliveness) {
		txei_dbg(dev, "sechudle aliveness timer\n");
		schedule_delayed_work(&dev->aliveness_timer,
			dev->aliveness_timeout);
	}

}

/**
 * txei_init_file_private - initializes private file structure.
 *
 * @priv: private file structure to be initialized
 * @file: the file structure
*/
void txei_cl_init(struct txei_cl *priv, struct txei_device *dev)
{
	memset(priv, 0, sizeof(struct txei_cl));
	init_waitqueue_head(&priv->wait);
	init_waitqueue_head(&priv->rx_wait);
	init_waitqueue_head(&priv->tx_wait);
	INIT_LIST_HEAD(&priv->link);
	priv->reading_state = TXEI_IDLE;
	priv->writing_state = TXEI_IDLE;
	priv->dev = dev;
}

int txei_find_me_client_index(const struct txei_device *dev, uuid_le cuuid)
{
	int i, res = -1;

	for (i = 0; i < dev->me_clients_num; ++i)
		if (uuid_le_cmp(cuuid,
				dev->me_clients[i].props.protocol_name) == 0) {
			res = i;
			break;
		}

	return res;
}


/**
 * txei_find_me_client_update_filext - searches for ME client guid
 *                       sets client_id in txei_file_private if found
 * @dev: the device structure
 * @priv: private file structure to set client_id in
 * @cguid: searched guid of ME client
 * @client_id: id of host client to be set in file private structure
 *
 * returns ME client index
 */
u8 txei_find_me_client_update_filext(struct txei_device *dev,
	struct txei_cl *priv,
	const uuid_le *cguid, u8 client_id)
{
	int i;

	if (!dev || !priv || !cguid)
		return 0;

	/* check for valid client id */
	i = txei_find_me_client_index(dev, *cguid);
	if (i >= 0) {
		priv->me_client_id = dev->me_clients[i].client_id;
		priv->state = TXEI_FILE_CONNECTING;
		priv->host_client_id = client_id;

		list_add_tail(&priv->link, &dev->file_list);
		return (u8)i;
	}

	return 0;
}

/**
 * txei_alloc_file_private - allocates a private file structure and sets it up.
 * @file: the file structure
 *
 * returns  The allocated file or NULL on failure
 */
struct txei_cl *txei_cl_allocate(struct txei_device *dev)
{
	struct txei_cl *cl;

	cl = kmalloc(sizeof(struct txei_cl), GFP_KERNEL);
	if (!cl)
		return NULL;

	txei_cl_init(cl, dev);

	return cl;
}



/**
 * txei_disconnect_host_client - sends disconnect message to fw from host client.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * Locking: called under "dev->device_lock" lock
 *
 * Please note that the dev->device_lock needs to be
 * set when this function is invoked. There is a place
 * where device_lock is unlocked to sleep and then relocked
 * upon awakening
 * returns 0 on success, <0 on failure.
 */
int txei_disconnect_host_client(struct txei_device *dev, struct txei_cl *cl)
{
	int rets, err;
	long timeout = 15;	/* 15 seconds */
	struct txei_cl_cb *cb;

	if (!dev || !cl)
		return -ENODEV;

	if (cl->state != TXEI_FILE_DISCONNECTING)
		return 0;

	cb = kzalloc(sizeof(struct txei_cl_cb), GFP_KERNEL);
	if (!cb)
		return -ENOMEM;

	INIT_LIST_HEAD(&cb->cb_list);
	cb->file_private = cl;
	cb->fop_type = TXEI_FOP_CLOSE;
	if (dev->hbuf_is_ready) {
		dev->hbuf_is_ready = false;
		if (txei_disconnect(dev, cl)) {
			mdelay(10); /* Wait for hardware disconnection ready */
			list_add_tail(&cb->cb_list,
				&dev->ctrl_rd_list.txei_cb.cb_list);
		} else {
			rets = -ENODEV;
			txei_dbg(dev, "failed to call txei_disconnect.\n");
			goto free;
		}
	} else {
		txei_dbg(dev, "add disconnect cb to control write list\n");
		list_add_tail(&cb->cb_list,
				&dev->ctrl_wr_list.txei_cb.cb_list);
	}
	mutex_unlock(&dev->device_lock);

	err = wait_event_timeout(dev->wait_recvd_msg,
		 (TXEI_FILE_DISCONNECTED == cl->state),
		 timeout * HZ);

	mutex_lock(&dev->device_lock);
	if (TXEI_FILE_DISCONNECTED == cl->state) {
		rets = 0;
		txei_dbg(dev, "successfully disconnected from FW client.\n");
	} else {
		rets = -ENODEV;
		if (TXEI_FILE_DISCONNECTED != cl->state)
			txei_dbg(dev, "wrong status client disconnect.\n");

		if (err)
			txei_dbg(dev,
					"wait failed disconnect err = %08x\n",
					err);

		txei_dbg(dev, "failed to disconnect from FW client.\n");
	}

	txei_io_list_flush(&dev->ctrl_rd_list, cl);
	txei_io_list_flush(&dev->ctrl_wr_list, cl);
free:
	txei_free_cb_private(cb);

	return rets;
}

/**
 * txei_remove_client_from_file_list -
 *	removes file private data from device file list
 *
 * @dev: the device structure
 * @host_client_id: host client id to be removed
 */
void txei_remove_client_from_file_list(struct txei_device *dev,
				       u8 host_client_id)
{
	struct txei_cl *cl_pos = NULL;
	struct txei_cl *cl_next = NULL;
	list_for_each_entry_safe(cl_pos, cl_next, &dev->file_list, link) {
		if (host_client_id == cl_pos->host_client_id) {
			txei_dbg(dev, "remove host client = %d, ME client = %d\n",
					cl_pos->host_client_id,
					cl_pos->me_client_id);
			list_del_init(&cl_pos->link);
			break;
		}
	}
}
