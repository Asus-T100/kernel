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
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/jiffies.h>

#include "txei.h"

#include "txei_dev.h"
#include "hw-txe.h"
#include "hw.h"

static const char *hcmd_str(u8 cmd)
{
	switch (cmd) {
	case HOST_START_REQ_CMD:
		return "HOST_START_REQ_CMD";
	case HOST_START_RES_CMD:
		return "HOST_START_RES_CMD";
	case HOST_STOP_REQ_CMD:
		return "HOST_STOP_REQ_CMD";
	case HOST_STOP_RES_CMD:
		return "HOST_STOP_RES_CMD";
	case ME_STOP_REQ_CMD:
		return "ME_STOP_REQ_CMD";
	case HOST_ENUM_REQ_CMD:
		return "HOST_ENUM_REQ_CMD";
	case HOST_ENUM_RES_CMD:
		return "HOST_ENUM_RES_CMD";
	case HOST_CLIENT_PROPERTIES_REQ_CMD:
		return "HOST_CLIENT_PROPERTIES_REQ_CMD";
	case HOST_CLIENT_PROPERTIES_RES_CMD:
		return "HOST_CLIENT_PROPERTIES_RES_CMD";
	case CLIENT_CONNECT_REQ_CMD:
		return "CLIENT_CONNECT_REQ_CMD";
	case CLIENT_CONNECT_RES_CMD:
		return "CLIENT_CONNECT_RES_CMD";
	case CLIENT_DISCONNECT_REQ_CMD:
		return "CLIENT_DISCONNECT_REQ_CMD";
	case CLIENT_DISCONNECT_RES_CMD:
		return "CLIENT_DISCONNECT_RES_CMD";
	case TXEI_FLOW_CONTROL_CMD:
		return "TXEI_FLOW_CONTROL_CMD";
	default:
		return "unkown";
	}
}

static bool ipc_check_and_ack_interrupts(struct txei_device *dev, bool do_ack)
{
	u32 hisr;
	u32 hhisr;
	u32 ipc_isr;
	bool generated;

	/* read interrupt registers */
	hhisr = ipc_bridge_reg_read(dev, HHISR_OFFSET);
	generated = (hhisr & TXEI_IPC_HHIER_MSK);
	if (!generated)
		goto out;

	hisr = ipc_bridge_reg_read(dev, HISR_OFFSET);

	if (hhisr & TXEI_IPC_HHIER_SEC)
		ipc_isr = ipc_sec_reg_read_silent(dev,
				SEC_IPC_HOST_INT_STATUS_OFFSET);
	else
		ipc_isr = 0;

	generated = generated ||
		(hisr & TXEI_HISR_INT_STS_MSK) ||
		(ipc_isr & TXEI_SEC_IPC_HOST_INT_STATUS_PENDING);

	if (generated && do_ack) {
		/* Save the interrupt causes */
		dev->ipc_interrupt_cause.IPCInputReadyInt |=
			!!(ipc_isr & TXEI_SEC_IPC_HOST_INT_STATUS_IN_RDY);
		dev->ipc_interrupt_cause.ReadinessInt |=
			!!(hisr & TXEI_HISR_INT_0_STS);
		dev->ipc_interrupt_cause.AlivenessInt |=
			!!(hisr & TXEI_HISR_INT_1_STS);

		if (hisr & TXEI_HISR_INT_2_STS)
			dev->ipc_interrupt_cause.OutputDoorbellInt++;

#ifdef POLLING_MODE
		/* Cannot print in IRQ Level */
		txei_dbg(dev, "Interrupt was generated (%02X|%04X|%02X) InReady=%d, Readiness=%d, Aliveness=%d, OutputDoorbell=%d\n",
			hhisr, hisr, ipc_isr,
			dev->ipc_interrupt_cause.IPCInputReadyInt,
			dev->ipc_interrupt_cause.ReadinessInt,
			dev->ipc_interrupt_cause.AlivenessInt,
			dev->ipc_interrupt_cause.OutputDoorbellInt);
#endif /* POLLING_MODE */

		ipc_host_interrupts_disable(dev);
		/**
		 * Clear the interrupts in hierarchy:
		 * PPC and Bridge, than the High Level
		 */
		ipc_sec_reg_write_silent(dev,
			SEC_IPC_HOST_INT_STATUS_OFFSET, ipc_isr);
		ipc_bridge_reg_write(dev, HISR_OFFSET, hisr);
		ipc_bridge_reg_write(dev, HHISR_OFFSET, hhisr);
	}

out:
	return generated;
}

/**
 * txei_interrupt_quick_handler - The ISR of the TXEI device
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * returns irqreturn_t
 */
irqreturn_t txei_interrupt_quick_handler(int irq, void *dev_id)
{
	struct txei_device *dev = dev_id;

	if (ipc_check_and_ack_interrupts(dev, true))
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}

/**
 * txei_complete_callback - processes completed callback
 *
 * @cl: private data of the file object.
 * @cb: callback block.
 */
static void txei_complete_callback(struct txei_cl *cl, struct txei_cl_cb *cb)
{

	if (cb->fop_type == TXEI_FOP_WRITE) {
		txei_free_cb_private(cb);
		cl->writing_state = TXEI_WRITE_COMPLETE;
		if (waitqueue_active(&cl->tx_wait))
			wake_up_interruptible(&cl->tx_wait);

	} else if (cb->fop_type == TXEI_FOP_READ &&
		   cl->reading_state == TXEI_READING) {
		cl->reading_state = TXEI_READ_COMPLETE;
		if (waitqueue_active(&cl->rx_wait))
			wake_up_interruptible(&cl->rx_wait);
	}
}

/**
 * _txei_irq_thread_state_ok - checks if txei header matches file private data
 *
 * @cl: private data of the file object
 * @txei_hdr: header of txei client message
 *
 * returns !=0 if matches, 0 if no match.
 */
static int _txei_irq_thread_state_ok(struct txei_cl *cl,
				struct txei_msg_hdr *txei_hdr)
{
	return (cl->host_client_id == txei_hdr->host_addr &&
		cl->me_client_id == txei_hdr->me_addr &&
		cl->state == TXEI_FILE_CONNECTED &&
		TXEI_READ_COMPLETE != cl->reading_state);
}

/**
 * txei_irq_thread_read_client_message - bottom half read routine after ISR to
 * handle the read txei client message data processing.
 *
 * @complete_list: An instance of our list structure
 * @dev: the device structure
 * @txei_hdr: header of txei client message
 *
 * returns 0 on success, <0 on failure.
 */
static int txei_irq_thread_read_client_message(
	struct txei_io_list *complete_list,
	struct txei_device *dev, struct txei_msg_hdr *txei_hdr) {

	struct txei_cl *cl;
	struct txei_cl_cb *cb_pos = NULL, *cb_next = NULL;
	unsigned char *buffer = NULL;

	txei_dbg(dev, "start client msg\n");
	if (!(dev->read_list.status == 0 &&
	      !list_empty(&dev->read_list.txei_cb.cb_list)))
		goto quit;

	list_for_each_entry_safe(cb_pos, cb_next,
		&dev->read_list.txei_cb.cb_list, cb_list) {

		cl = (struct txei_cl *) cb_pos->file_private;
		if (cl && _txei_irq_thread_state_ok(cl, txei_hdr)) {
			cl->reading_state = TXEI_READING;
			buffer = cb_pos->response_buffer.data +
				cb_pos->buf_idx;

			if (cb_pos->response_buffer.size <
				txei_hdr->length + cb_pos->buf_idx) {
				txei_err(dev, "msg overflow: buf[%d] < hdr[%d] + info[%lu]\n",
					cb_pos->response_buffer.size,
					txei_hdr->length, cb_pos->buf_idx);
				list_del(&cb_pos->cb_list);
				return -ENOMEM;
			}
			if (buffer)
				txei_read_data(dev, buffer, txei_hdr->length);

			cb_pos->buf_idx += txei_hdr->length;
			if (txei_hdr->msg_complete) {
				cl->status = 0;
				list_del(&cb_pos->cb_list);
				txei_dbg(dev, "completed read host client = %d, ME client = %d, data length = %lu\n",
					cl->host_client_id,
					cl->me_client_id, cb_pos->buf_idx);

				txei_print_hex_dump(dev, "read: ",
					cb_pos->response_buffer.data,
					cb_pos->buf_idx);
				list_add_tail(&cb_pos->cb_list,
					&complete_list->txei_cb.cb_list);
			}

			break;
		}

	}

quit:
	txei_dbg(dev, "message read\n");
	if (!buffer) {
		txei_read_data(dev, (unsigned char *) dev->rd_msg_buf,
			txei_hdr->length);
		txei_dbg(dev, "discarding message, header = %08x.\n",
			*(u32 *) dev->rd_msg_buf);
	}

	return 0;
}

/**
 * _txei_irq_thread_close - processes close related operation.
 *
 * @dev: the device structure.
 * @cb_pos: callback block.
 * @cl: private data of the file object.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int _txei_irq_thread_close(struct txei_device *dev,
		struct txei_cl_cb *cb_pos, struct txei_cl *cl,
		struct txei_io_list *cmpl_list)
{
	if (txei_hbuf_max_len(dev) >=
		sizeof(struct hbm_client_connect_request)) {

		if (!txei_disconnect(dev, cl)) {
			cl->status = 0;
			cb_pos->buf_idx = 0;
			list_move_tail(&cb_pos->cb_list,
				&cmpl_list->txei_cb.cb_list);
			return -EMSGSIZE;
		} else {
			cl->state = TXEI_FILE_DISCONNECTING;
			cl->status = 0;
			cb_pos->buf_idx = 0;
			list_move_tail(&cb_pos->cb_list,
				&dev->ctrl_rd_list.txei_cb.cb_list);
			cl->timer_count = TXEI_CONNECT_TIMEOUT;
		}
	} else {
		/* return the cancel routine */
		return -EBADMSG;
	}

	return 0;
}

/**
 * is_treat_specially_client - checks if the message belongs
 * to the file private data.
 *
 * @cl: private data of the file object
 * @rs: connect response bus message
 *
 */
static bool is_treat_specially_client(struct txei_cl *cl,
		struct hbm_client_connect_response *rs) {

	if (cl->host_client_id == rs->host_addr &&
	    cl->me_client_id == rs->me_addr) {
		if (!rs->status) {
			cl->state = TXEI_FILE_CONNECTED;
			cl->status = 0;

		} else {
			cl->state = TXEI_FILE_DISCONNECTED;
			cl->status = -ENODEV;
		}
		cl->timer_count = 0;

		return true;
	}
	return false;
}

/**
 * txei_client_connect_response - connects to response irq routine
 *
 * @dev: the device structure
 * @rs: connect response bus message
 */
static void txei_client_connect_response(struct txei_device *dev,
		struct hbm_client_connect_response *rs) {

	struct txei_cl *cl;
	struct txei_cl_cb *cb_pos = NULL, *cb_next = NULL;

	txei_dbg(dev, "connect_response:\n"
	"ME Client = %d\n"
	"Host Client = %d\n"
	"Status = %d\n", rs->me_addr, rs->host_addr, rs->status);

	if (!dev->ctrl_rd_list.status &&
		!list_empty(&dev->ctrl_rd_list.txei_cb.cb_list)) {

		list_for_each_entry_safe(cb_pos, cb_next,
				&dev->ctrl_rd_list.txei_cb.cb_list, cb_list) {
			cl = (struct txei_cl *) cb_pos->file_private;
			if (!cl) {
				list_del(&cb_pos->cb_list);
				return;
			}
			if (TXEI_FOP_IOCTL == cb_pos->fop_type) {
				if (is_treat_specially_client(cl, rs)) {
					list_del(&cb_pos->cb_list);
					cl->status = 0;
					cl->timer_count = 0;
					break;
				}
			}
		}
	}
}

/**
 * txei_client_disconnect_response - disconnects from response irq routine
 *
 * @dev: the device structure
 * @rs: disconnect response bus message
 */
static void txei_client_disconnect_response(struct txei_device *dev,
		struct hbm_client_connect_response *rs) {
	struct txei_cl *cl;
	struct txei_cl_cb *cb_pos = NULL, *cb_next = NULL;

	txei_dbg(dev, "disconnect_response:\n"
	"ME Client = %d\n"
	"Host Client = %d\n"
	"Status = %d\n", rs->me_addr, rs->host_addr, rs->status);

	if (!dev->ctrl_rd_list.status
			&& !list_empty(&dev->ctrl_rd_list.txei_cb.cb_list)) {
		list_for_each_entry_safe(cb_pos, cb_next,
				&dev->ctrl_rd_list.txei_cb.cb_list, cb_list) {
			cl = (struct txei_cl *) cb_pos->file_private;

			if (!cl) {
				list_del(&cb_pos->cb_list);
				return;
			}

			txei_dbg(dev, "list_for_each_entry_safe in ctrl_rd_list.\n");
			if (cl->host_client_id == rs->host_addr
					&& cl->me_client_id == rs->me_addr) {

				list_del(&cb_pos->cb_list);
				if (!rs->status)
					cl->state = TXEI_FILE_DISCONNECTED;

				cl->status = 0;
				cl->timer_count = 0;
				break;
			}
		}
	}
}

/**
 * same_flow_addr - tells if they have the same address.
 *
 * @file: private data of the file object.
 * @flow: flow control.
 *
 * returns  !=0, same; 0,not.
 */
static int same_flow_addr(struct txei_cl *cl, struct hbm_flow_control *flow)
{
	return (cl->host_client_id == flow->host_addr
			&& cl->me_client_id == flow->me_addr);
}

/**
 * add_single_flow_creds - adds single buffer credentials.
 *
 * @file: private data of the file object.
 * @flow: flow control.
 */
static void add_single_flow_creds(struct txei_device *dev,
		struct hbm_flow_control *flow) {
	struct txei_me_client *client;
	int i;

	for (i = 0; i < dev->me_clients_num; i++) {
		client = &dev->me_clients[i];
		if (client && flow->me_addr == client->client_id) {
			if (client->props.single_recv_buf) {
				client->txei_flow_ctrl_creds++;
				txei_dbg(dev, "recv flow ctrl msg ME %d (single).\n",
					flow->me_addr);
				txei_dbg(dev, "flow control credentials = %d.\n",
					client->txei_flow_ctrl_creds);
			} else {
				BUG(); /* error in flow control */
			}
		}
	}
}

/**
 * txei_client_flow_control_response - flow control response irq routine
 *
 * @dev: the device structure
 * @flow_control: flow control response bus message
 */
static void txei_client_flow_control_response(struct txei_device *dev,
		struct hbm_flow_control *flow_control) {
	struct txei_cl *cl_pos = NULL;
	struct txei_cl *cl_next = NULL;

	if (!flow_control->host_addr) {
		/* single receive buffer */
		add_single_flow_creds(dev, flow_control);
	} else {
		/* normal connection */
		list_for_each_entry_safe(cl_pos, cl_next,
				&dev->file_list, link) {
			txei_dbg(dev, "list_for_each_entry_safe in file_list\n");

			txei_dbg(dev, "cl of host client %d ME client %d.\n",
				cl_pos->host_client_id, cl_pos->me_client_id);

			txei_dbg(dev, "flow ctrl msg for host %d ME %d.\n",
				flow_control->host_addr, flow_control->me_addr);

			if (same_flow_addr(cl_pos, flow_control)) {
				txei_dbg(dev, "recv ctrl msg for host  %d ME %d.\n",
					flow_control->host_addr,
					flow_control->me_addr);

				cl_pos->txei_flow_ctrl_creds++;
				txei_dbg(dev, "flow control credentials = %d.\n",
					cl_pos->txei_flow_ctrl_creds);
				break;
			}
		}
	}
}

/**
 * same_disconn_addr - tells if they have the same address
 *
 * @file: private data of the file object.
 * @disconn: disconnection request.
 *
 * returns !=0, same; 0,not.
 */
static int same_disconn_addr(struct txei_cl *cl,
		struct hbm_client_connect_request *disconn) {
	return (cl->host_client_id == disconn->host_addr
			&& cl->me_client_id == disconn->me_addr);
}

/**
 * txei_client_disconnect_request - disconnects from request irq routine
 *
 * @dev: the device structure.
 * @disconnect_req: disconnect request bus message.
 */
static void txei_client_disconnect_request(struct txei_device *dev,
		struct hbm_client_connect_request *disconnect_req) {
	struct txei_msg_hdr *txei_hdr;
	struct hbm_client_connect_response *disconnect_res;
	struct txei_cl *cl_pos = NULL;
	struct txei_cl *cl_next = NULL;

	list_for_each_entry_safe(cl_pos, cl_next, &dev->file_list, link) {
		if (same_disconn_addr(cl_pos, disconnect_req)) {
			txei_dbg(dev, "disconnect request host client %d ME client %d.\n",
				disconnect_req->host_addr,
				disconnect_req->me_addr);

			cl_pos->state = TXEI_FILE_DISCONNECTED;
			cl_pos->timer_count = 0;

			/* prepare disconnect response */
			txei_hdr = (struct txei_msg_hdr *) &dev->ext_msg_buf[0];
			txei_hdr->host_addr = 0;
			txei_hdr->me_addr = 0;
			txei_hdr->length =
				sizeof(struct hbm_client_connect_response);
			txei_hdr->msg_complete = 1;
			txei_hdr->reserved = 0;

			disconnect_res =
				(struct hbm_client_connect_response *)
				&dev->ext_msg_buf[1];

			disconnect_res->host_addr = cl_pos->host_client_id;
			disconnect_res->me_addr = cl_pos->me_client_id;
			disconnect_res->hbm_cmd = CLIENT_DISCONNECT_RES_CMD;
			disconnect_res->status = 0;
			dev->extra_write_index = 2;
			break;
		}
	}
}

/**
 * txei_irq_thread_read_bus_message - bottom half read routine after ISR to
 * handle the read bus message cmd processing.
 *
 * @dev: the device structure
 * @txei_hdr: header of bus message
 * Please note that the dev->device_lock needs to be
 * set when this function is invoked. There is a place
 * where device_lock is unlocked to sleep and then relocked
 * upon awakening
 * For example, the txei_interrupt_thread_handler sets the
 * device_lock prior to calling the parent functions for
 * this function
 */
static int txei_irq_thread_read_bus_message(struct txei_device *dev,
	struct txei_msg_hdr *txei_hdr)
{
	struct txei_bus_message *txei_msg;
	struct hbm_host_version_response *version_res;
	struct hbm_client_connect_response *connect_res;
	struct hbm_client_connect_response *disconnect_res;
	struct hbm_client_connect_request *disconnect_req;
	struct hbm_flow_control *flow_control;
	struct hbm_props_response *props_res;
	struct hbm_host_enum_response *enum_res;
	struct hbm_host_stop_request *host_stop_req;

	unsigned char *buffer;
	u8 cmd;

	/* read the message to our buffer */
	buffer = (unsigned char *) dev->rd_msg_buf;
	BUG_ON(txei_hdr->length >= sizeof(dev->rd_msg_buf));
	txei_read_data(dev, buffer, txei_hdr->length);
	txei_msg = (struct txei_bus_message *)buffer;
	cmd = buffer[0];


	txei_dbg(dev, "txei trace: bus message opcode : 0x%02X:%s\n",
		cmd, hcmd_str(cmd));

	switch (cmd) {
	case HOST_START_RES_CMD:
		version_res = (struct hbm_host_version_response *) txei_msg;
		if (version_res->host_version_supported) {
			dev->version.major_version = HBM_MAJOR_VERSION;
			dev->version.minor_version = HBM_MINOR_VERSION;
			if (dev->dev_state == TXEI_DEV_INIT_CLIENTS &&
			    dev->init_clients_state == TXEI_START_MESSAGE) {
				dev->init_clients_timer = 0;
				txei_host_enum_clients_message(dev);
			} else {
				dev->recvd_msg = false;
				txei_dbg(dev, "ITXEI reset due to received host start response bus message.\n");
				goto reset;
			}
		} else {
			dev->version = version_res->me_max_version;
			/* send stop message */
			txei_hdr->host_addr = 0;
			txei_hdr->me_addr = 0;
			txei_hdr->length = sizeof(struct hbm_host_stop_request);
			txei_hdr->msg_complete = 1;
			txei_hdr->reserved = 0;

			host_stop_req = (struct hbm_host_stop_request *)
				&dev->wr_msg_buf[1];

			memset(host_stop_req, 0,
				sizeof(struct hbm_host_stop_request));

			host_stop_req->hbm_cmd = HOST_STOP_REQ_CMD;
			host_stop_req->reason = DRIVER_STOP_REQUEST;
			txei_write_message(dev, txei_hdr,
				(unsigned char *) host_stop_req,
				txei_hdr->length);

			txei_dbg(dev, "version mismatch.\n");
			/* FIXME warp this out */
			return 0;
		}

		dev->recvd_msg = true;
		txei_dbg(dev, "host start response message received.\n");
		break;

	case CLIENT_CONNECT_RES_CMD:
		connect_res = (struct hbm_client_connect_response *) txei_msg;
		txei_client_connect_response(dev, connect_res);
		txei_dbg(dev, "client connect response message received.\n");
		wake_up(&dev->wait_recvd_msg);
		break;

	case CLIENT_DISCONNECT_RES_CMD:
		disconnect_res = (struct hbm_client_connect_response *)
			txei_msg;

		txei_client_disconnect_response(dev, disconnect_res);
		txei_dbg(dev, "client disconnect response message received.\n");
		wake_up(&dev->wait_recvd_msg);
		break;

	case TXEI_FLOW_CONTROL_CMD:
		flow_control = (struct hbm_flow_control *) txei_msg;
		txei_client_flow_control_response(dev, flow_control);
		txei_dbg(dev, "client flow control response message received.\n");
		break;

	case HOST_CLIENT_PROPERTIES_RES_CMD:
		props_res = (struct hbm_props_response *) txei_msg;
		if (props_res->status || !dev->me_clients) {
			txei_dbg(dev, "reset due to received host client properties response bus message wrong status.\n");
			goto reset;
		}
		if (dev->me_clients[dev->me_client_presentation_num].
			client_id == props_res->address) {

			dev->me_clients[dev->me_client_presentation_num].
				props = props_res->client_properties;

			txei_dbg(dev, "porpeties idx=%d presen=%d uuid=%pU\n",
				dev->me_client_index,
				dev->me_client_presentation_num,
				&props_res->client_properties.protocol_name);

			if (dev->dev_state == TXEI_DEV_INIT_CLIENTS &&
				dev->init_clients_state ==
				TXEI_CLIENT_PROPERTIES_MESSAGE) {

				dev->me_client_index++;
				dev->me_client_presentation_num++;
				txei_host_client_properties(dev);
			} else {
				txei_dbg(dev, "reset due to received host client properties response bus message");
				goto reset;
			}
		} else {
			txei_dbg(dev, "reset due to received host client properties response bus message for wrong client ID\n");
			goto reset;
		}
		break;

	case HOST_ENUM_RES_CMD:
		enum_res = (struct hbm_host_enum_response *) txei_msg;
		memcpy(dev->me_clients_map, enum_res->valid_addresses, 32);
		if (dev->dev_state == TXEI_DEV_INIT_CLIENTS &&
		    dev->init_clients_state == TXEI_ENUM_CLIENTS_MESSAGE) {
			dev->init_clients_timer = 0;
			dev->me_client_presentation_num = 0;
			dev->me_client_index = 0;
			txei_allocate_me_clients_storage(dev);
			dev->init_clients_state =
				TXEI_CLIENT_PROPERTIES_MESSAGE;
			txei_host_client_properties(dev);
		} else {
			txei_dbg(dev, "reset due to received host enumeration clients response bus message.\n");
			goto reset;
		}
		break;

	case HOST_STOP_RES_CMD:
		dev->dev_state = TXEI_DEV_DISABLED;
		txei_dbg(dev, "resetting because of FW stop response.\n");
		txei_reset(dev, true);
		break;

	case CLIENT_DISCONNECT_REQ_CMD:
		/* search for client */
		disconnect_req = (struct hbm_client_connect_request *)
			txei_msg;

		txei_client_disconnect_request(dev, disconnect_req);
		break;

	case ME_STOP_REQ_CMD:
		/* prepare stop request */
		txei_hdr = (struct txei_msg_hdr *) &dev->ext_msg_buf[0];
		txei_hdr->host_addr = 0;
		txei_hdr->me_addr = 0;
		txei_hdr->length = sizeof(struct hbm_host_stop_request);
		txei_hdr->msg_complete = 1;
		txei_hdr->reserved = 0;
		host_stop_req = (struct hbm_host_stop_request *)
			&dev->ext_msg_buf[1];
		memset(host_stop_req, 0, sizeof(struct hbm_host_stop_request));
		host_stop_req->hbm_cmd = HOST_STOP_REQ_CMD;
		host_stop_req->reason = DRIVER_STOP_REQUEST;
		host_stop_req->reserved[0] = 0;
		host_stop_req->reserved[1] = 0;
		dev->extra_write_index = 2;
		break;

	default:
		BUG();
		break;

	}
	return 0;
reset:
	txei_reset(dev, true);
	return 0;


}

/**
 * _txei_hb_read - processes read related operation.
 *
 * @dev: the device structure.
 * @cb_pos: callback block.
 * @cl: private data of the file object.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int _txei_irq_thread_read(struct txei_device *dev,
		struct txei_cl_cb *cb_pos, struct txei_cl *cl,
		struct txei_io_list *cmpl_list) {

	if (txei_hbuf_max_len(dev) >=  sizeof(struct hbm_flow_control)) {
		if (!txei_send_flow_control(dev, cl)) {
			cl->status = -ENODEV;
			cb_pos->buf_idx = 0;
			list_move_tail(&cb_pos->cb_list,
				&cmpl_list->txei_cb.cb_list);
			return -ENODEV;
		} else {
			list_move_tail(&cb_pos->cb_list,
				&dev->read_list.txei_cb.cb_list);
		}
	} else {
		/* return the cancel routine */
		list_del(&cb_pos->cb_list);
		return -EBADMSG;
	}

	return 0;
}

/**
 * _txei_irq_thread_ioctl - processes ioctl related operation.
 *
 * @dev: the device structure.
 * @cb_pos: callback block.
 * @cl: private data of the file object.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int _txei_irq_thread_ioctl(struct txei_device *dev,
		struct txei_cl_cb *cb_pos, struct txei_cl *cl,
		struct txei_io_list *cmpl_list) {

	if (txei_hbuf_max_len(dev) >=
		sizeof(struct hbm_client_connect_request)) {

		cl->state = TXEI_FILE_CONNECTING;
		if (!txei_connect(dev, cl)) {
			cl->status = -ENODEV;
			cb_pos->buf_idx = 0;
			list_del(&cb_pos->cb_list);
			return -ENODEV;
		} else {
			list_move_tail(&cb_pos->cb_list,
				&dev->ctrl_rd_list.txei_cb.cb_list);
			cl->timer_count = TXEI_CONNECT_TIMEOUT;
		}
	} else {
		/* return the cancel routine */
		list_del(&cb_pos->cb_list);
		return -EBADMSG;
	}

	return 0;
}

/**
 * _txei_irq_thread_cmpl - processes completed operation.
 *
 * @dev: the device structure.
 * @cb_pos: callback block.
 * @cl: private data of the file object.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int _txei_irq_thread_cmpl(struct txei_device *dev,
	struct txei_cl_cb *cb_pos, struct txei_cl *cl,
	struct txei_io_list *cmpl_list) {

	struct txei_msg_hdr *txei_hdr;

	if (txei_hbuf_max_len(dev) >= (cb_pos->request_buffer.size -
		cb_pos->buf_idx)) {

		txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
		txei_hdr->host_addr = cl->host_client_id;
		txei_hdr->me_addr = cl->me_client_id;
		txei_hdr->length = cb_pos->request_buffer.size -
			cb_pos->buf_idx;
		txei_hdr->msg_complete = 1;
		txei_hdr->reserved = 0;
		txei_dbg(dev, "cb_pos->request_buffer.size = %d txei_hdr->msg_complete = %d\n",
			cb_pos->request_buffer.size, txei_hdr->msg_complete);

		txei_dbg(dev, "cb_pos->buf_idx  = %lu\n", cb_pos->buf_idx);
		txei_dbg(dev, "txei_hdr->length  = %d\n", txei_hdr->length);

		if (!txei_write_message(
			dev, txei_hdr,
			(unsigned char *) (cb_pos->request_buffer.data
			+ cb_pos->buf_idx), txei_hdr->length)) {

			cl->status = -ENODEV;
			list_move_tail(&cb_pos->cb_list,
				&cmpl_list->txei_cb.cb_list);
			return -ENODEV;
		} else {
			if (txei_flow_ctrl_reduce(dev, cl))
				return -ENODEV;
			cl->status = 0;
			cb_pos->buf_idx += txei_hdr->length;
			list_move_tail(&cb_pos->cb_list,
				&dev->write_waiting_list.txei_cb.cb_list);
		}
	} else {
		/* buffer is still empty */
		txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
		txei_hdr->host_addr = cl->host_client_id;
		txei_hdr->me_addr = cl->me_client_id;
		txei_hdr->length = txei_hbuf_max_len(dev);
		txei_hdr->msg_complete = 0;
		txei_hdr->reserved = 0;

		if (!txei_write_message(dev, txei_hdr,
			(unsigned char *) (cb_pos->request_buffer.data +
			cb_pos->buf_idx), txei_hdr->length)) {

			cl->status = -ENODEV;
			list_move_tail(&cb_pos->cb_list,
				&cmpl_list->txei_cb.cb_list);
			return -ENODEV;
		} else {
			cb_pos->buf_idx += txei_hdr->length;
			txei_dbg(dev, "cb_pos->request_buffer.size = %d txei_hdr->msg_complete = %d\n",
				cb_pos->request_buffer.size,
				txei_hdr->msg_complete);

			txei_dbg(dev, "cb_pos->buf_idx  = %lu\n",
				cb_pos->buf_idx);

			txei_dbg(dev, "txei_hdr->length  = %d\n",
				txei_hdr->length);
		}
		return -EMSGSIZE;
	}

	return 0;
}

/**
 * txei_irq_thread_read_handler - bottom half read routine after ISR to
 * handle the read processing.
 *
 * @cmpl_list: An instance of our list structure
 * @dev: the device structure
 *
 * Please note that the dev->device_lock needs to be
 * set when this function is invoked. There is a place
 * where device_lock is unlocked to sleep and then relocked
 * upon awakening
 * For example, the txei_interrupt_thread_handler sets the
 * device_lock prior to calling the parent functions for
 * this function
 * returns 0 on success, <0 on failure.
 */
static int txei_irq_thread_read_handler(struct txei_io_list *cmpl_list,
		struct txei_device *dev) {
	struct txei_msg_hdr *txei_hdr;
	struct txei_cl *pos = NULL;
	struct txei_cl *next = NULL;
	int ret;

	if (!dev->rd_msg_hdr) {
		dev->rd_msg_hdr = ipc_output_payload_read(dev, 0);
		txei_dbg(dev, "update read msg header 0x%08x\n",
			dev->rd_msg_hdr);
	}
	txei_hdr = (struct txei_msg_hdr *) &dev->rd_msg_hdr;
	txei_dbg(dev, "txei_hdr->length = %d\n", txei_hdr->length);

	if (txei_hdr->reserved || !dev->rd_msg_hdr) {
		txei_err(dev, "corrupted message header: reserved=%d, hdr=0x%08X\n",
			txei_hdr->reserved, dev->rd_msg_hdr);

		ret = -EBADMSG;
		goto end;
	}

	if (txei_hdr->host_addr || txei_hdr->me_addr) {
		list_for_each_entry_safe(pos, next, &dev->file_list, link) {
			txei_dbg(dev, "read host client = %d, ME client = %d\n",
				pos->host_client_id, pos->me_client_id);
			if (pos->host_client_id == txei_hdr->host_addr &&
				pos->me_client_id == txei_hdr->me_addr)

				break;
		}

		if (&pos->link == &dev->file_list) {
			txei_err(dev, "corrupted message header: address doesn't match haddr=%d meaddr=%d\n",
				txei_hdr->host_addr, txei_hdr->me_addr);
			ret = -EBADMSG;
			goto end;
		}
	}
	if (PAYLOAD_SIZE < txei_hdr->length) {
		/* we can't read the message */
		txei_dbg(dev, "we can't read the message data (data is greater than payload-buffer size)");
		ret = -ERANGE;
		goto end;
	}

	/* decide where to read the message too */
	if (!txei_hdr->host_addr) {
		txei_dbg(dev, "call txei_irq_thread_read_bus_message.\n");
		ret = txei_irq_thread_read_bus_message(dev, txei_hdr);
		txei_dbg(dev, "txei_irq_thread_read_bus_message ret=%d.\n",
			ret);
	} else {
		txei_dbg(dev, "call txei_irq_thread_read_client_message.\n");
		ret = txei_irq_thread_read_client_message(cmpl_list,
			dev, txei_hdr);
		txei_dbg(dev, "txei_irq_thread_read_client_message ret=%d.\n",
			ret);
		if (ret)
			goto end;

	}

	/* reset the header */
	dev->rd_msg_hdr = 0;

end:
	return ret;
}

/**
 * txei_irq_thread_write_handler - bottom half write routine after
 * ISR to handle the write processing.
 *
 * @cmpl_list: An instance of our list structure
 * @dev: the device structure
 *
 * returns 0 on success, <0 on failure.
 */
static int txei_irq_thread_write_handler(struct txei_io_list *cmpl_list,
		struct txei_device *dev) {

	struct txei_cl *cl;
	struct txei_cl_cb *cb_pos = NULL, *cb_next = NULL;
	struct txei_io_list *list;
	int ret;

	if (!dev->hbuf_is_ready) {
		txei_dbg(dev, "ipc payload is not writable.\n");
		return 0;
	}
	/* complete all waiting for write CB */
	txei_dbg(dev, "complete all waiting for write cb.\n");

	list = &dev->write_waiting_list;
	if (!list->status && !list_empty(&list->txei_cb.cb_list)) {
		list_for_each_entry_safe(cb_pos, cb_next,
			&list->txei_cb.cb_list, cb_list) {

			cl = (struct txei_cl *) cb_pos->file_private;
			if (cl) {
				cl->status = 0;
				list_del(&cb_pos->cb_list);
				if (TXEI_WRITING == cl->writing_state &&
					cb_pos->fop_type == TXEI_FOP_WRITE) {
					txei_dbg(dev, "TXEI WRITE COMPLETE\n");
					cl->writing_state =
						TXEI_WRITE_COMPLETE;
					list_add_tail(&cb_pos->cb_list,
						&cmpl_list->txei_cb.cb_list);
				}
			}

		}
	}

	if (dev->extra_write_index) {
		txei_dbg(dev, "extra_write_index = %d.\n",
			dev->extra_write_index);

		txei_write_message(dev, (struct txei_msg_hdr *)
			&dev->ext_msg_buf[0],
			(unsigned char *) &dev->ext_msg_buf[1],
			(dev->extra_write_index - 1) * sizeof(u32));

		dev->extra_write_index = 0;
	}
	if (dev->stop)
		return ~ENODEV;

	/* complete control write list CB */
	if (!dev->ctrl_wr_list.status) {
		/* complete control write list CB */
		txei_dbg(dev, "complete control write list cb.\n");
		list_for_each_entry_safe(cb_pos, cb_next,
			&dev->ctrl_wr_list.txei_cb.cb_list, cb_list) {

			cl = (struct txei_cl *) cb_pos->file_private;
			if (!cl) {
				list_del(&cb_pos->cb_list);
				return -ENODEV;
			}
			switch (cb_pos->fop_type) {
			case TXEI_FOP_CLOSE:
				/* send disconnect message */
				ret = _txei_irq_thread_close(dev, cb_pos,
					cl, cmpl_list);
				if (ret)
					return ret;

				break;
			case TXEI_FOP_READ:
				/* send flow control message */
				ret = _txei_irq_thread_read(dev, cb_pos, cl,
					cmpl_list);
				if (ret)
					return ret;

				break;
			case TXEI_FOP_IOCTL:
				/* connect message */
				if (txei_other_client_is_connecting(dev, cl))
					continue;
				ret = _txei_irq_thread_ioctl(dev, cb_pos,
					cl, cmpl_list);
				if (ret)
					return ret;

				break;

			default:
				BUG();
				break;
			}

		}
	}
	/* complete  write list CB */
	if (!dev->write_list.status &&
		!list_empty(&dev->write_list.txei_cb.cb_list)) {

		txei_dbg(dev, "complete write list cb.\n");
		list_for_each_entry_safe(cb_pos, cb_next,
			&dev->write_list.txei_cb.cb_list, cb_list) {

			cl = (struct txei_cl *)cb_pos->file_private;

			if (cl) {
				if (!txei_flow_ctrl_creds(dev, cl)) {
					txei_dbg(dev, "No flow control credentials for client %d, not sending.\n",
						cl->host_client_id);
					continue;
				}
				ret = _txei_irq_thread_cmpl(dev, cb_pos, cl,
					cmpl_list);
				if (ret)
					return ret;
			}

		}
	}
	return 0;
}

/**
 * txei_interrupt_thread_handler -
 * function called after ISR to handle the interrupt
 * processing.
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * Please note that all functions this calls will require the
 * device_lock mutex to be locked.
 * This includes the following:
 * ipc_aliveness_poll
 * ipc_aliveness_wait
 * txei_write_message
 * txei_irq_thread_read_bus_message
 * txei_irq_thread_read_handler
 * _txei_irq_thread_cmpl
 * txei_irq_thread_write_handler
 *
 *
 * returns irqreturn_t
 *
 */
irqreturn_t txei_interrupt_thread_handler(int irq, void *dev_id)
{

	struct txei_device *dev = (struct txei_device *) dev_id;
	struct txei_io_list complete_list;
	struct txei_cl_cb *cb_pos = NULL, *cb_next = NULL;
	struct txei_cl *cl;
	int rets;
	bool bus_message_received;

	/* Cannot print in IRQ Level */

	txei_dbg(dev, "Interrupt was generated (%02X|%04X|%02X) InReady=%d, Readiness=%d, Aliveness=%d, OutputDoorbell=%d\n",
		ipc_bridge_reg_read(dev, HHISR_OFFSET),
		ipc_bridge_reg_read(dev, HISR_OFFSET),
		ipc_sec_reg_read_silent(dev, SEC_IPC_HOST_INT_STATUS_OFFSET),
		dev->ipc_interrupt_cause.IPCInputReadyInt,
		dev->ipc_interrupt_cause.ReadinessInt,
		dev->ipc_interrupt_cause.AlivenessInt,
		dev->ipc_interrupt_cause.OutputDoorbellInt);

again:
	txei_dbg(dev, "function called after ISR to handle the interrupt processing.\n");
	/* initialize our complete list */
	mutex_lock(&dev->device_lock);
	txei_io_list_init(&complete_list);
	dev->aliveness = ipc_aliveness_get(dev);
	dev->readiness_state = ipc_readiness_get(dev);


	/* Readiness:
	 * Detection of SeC driver going through reset
	 * or SeC driver resetting HECI interface.
	 */
	if (dev->ipc_interrupt_cause.ReadinessInt) {
		txei_dbg(dev, "Readiness Interrupt was received...\n");
		dev->ipc_interrupt_cause.ReadinessInt = false;

		/* Check if SeC is going through reset */
		if (!ipc_readiness_is_sec_rdy(dev)) {
			txei_dbg(dev, "**** SeC turned off, SeCReady =0.\n");
			dev->recvd_readiness_int = false;
			if (dev->dev_state != TXEI_DEV_RESETING &&
				dev->dev_state != TXEI_DEV_POWER_DOWN &&
				dev->dev_state != TXEI_DEV_INITIALIZING) {

				txei_dbg(dev, "Readiness: SeC driver going through reset or SeC driver resetting HECI interface.\n");
				txei_reset(dev, true);
				goto end;
			}
		} else {
			txei_dbg(dev, "**** SeC turned on, SeCReady =1.\n");
			dev->recvd_readiness_int = true;
		}
		wake_up_interruptible(&dev->wait_readiness_int);
	}


	/* Check interrupt cause:
	 * Aliveness: Detection of SeC acknowledge of host request that
	 * it remain alive or host cancellation of that request.
	 */
	if (dev->ipc_interrupt_cause.AlivenessInt) {
		/* Clear the interrupt cause */
		txei_dbg(dev, "Aliveness Interrrupt: Status: %d\n",
			dev->aliveness);
		dev->ipc_interrupt_cause.AlivenessInt = false;
		dev->recvd_aliv_resp = true;
		wake_up_interruptible(&dev->wait_aliveness_resp);

		if (nopg == false &&
			dev->dev_state == TXEI_DEV_ENABLED && dev->aliveness &&
			!delayed_work_pending(&dev->aliveness_timer)) {

			txei_dbg(dev, "sechudle aliveness timer\n");
			schedule_delayed_work(&dev->aliveness_timer,
				dev->aliveness_timeout);
		}

	}

	if (nopg == false && !dev->aliveness) {
		txei_dbg(dev, "Bailing Out: HW Is dromant %d\n",
			dev->aliveness);
		mutex_unlock(&dev->device_lock);
		/* Note that out skips the mutex_unlock at end */
		goto out;
	}
	/* Output Doorbell:
	 * Detection of SeC having sent output to host
	 */
	if (dev->ipc_interrupt_cause.OutputDoorbellInt > 0) {
		/**
		 * NOTE: Clearing the interrupt cause is done in the read
		 * handler
		 * since it should be cleared only after reading the
		 * available data
		 * of course it is being cleared in Reset flow as well.
		 *
		 * TODO: Check if FW is doing Reset (then cancel
		 * this interrupt handler)
		 */

		txei_dbg(dev, "call txei_irq_thread_read_handler.\n");
		rets = txei_irq_thread_read_handler(&complete_list, dev);
		txei_dbg(dev, "from txei_irq_thread_read_handler ret=%d.\n",
			rets);
		if (rets)
			goto end;
		/**
		 * clear the pending interrupt only if read hanlder suceeded
		 * FIXME: check if this is correct and whether if this interfer
		 * with the reset flow
		 */
		dev->ipc_interrupt_cause.OutputDoorbellInt--;
	}
	/* Input Ready: Detection if host can write to SeC */
	else if (dev->ipc_interrupt_cause.IPCInputReadyInt == true) {
		dev->hbuf_is_ready = true;
		dev->ipc_interrupt_cause.IPCInputReadyInt = false;
	}
	if (dev->hbuf_is_ready) {
		if (!ipc_input_ready_get(dev)) {
			txei_dbg(dev, "HECI reset due to receiving Input Ready interrupt, but SEC_IPC_INPUT_STATUS_RDY is 0.\n");
			txei_reset(dev, true);
			goto end;
		}

		txei_dbg(dev, "call txei_irq_thread_write_handler.\n");
		rets = txei_irq_thread_write_handler(&complete_list, dev);
		txei_dbg(dev, "txei_irq_thread_write_handler ret=%d.\n", rets);
	}

end:
	txei_dbg(dev, "end of bottom half function.\n");
	dev->hbuf_is_ready = ipc_input_ready_get(dev);
	/* TODO: Check if this is correct */

	bus_message_received = false;
	if (dev->recvd_msg && waitqueue_active(&dev->wait_recvd_msg)) {
		txei_dbg(dev, "received waiting bus message\n");
		bus_message_received = true;
	}
	mutex_unlock(&dev->device_lock);
	if (bus_message_received) {
		txei_dbg(dev, "wake up dev->wait_recvd_msg\n");
		wake_up_interruptible(&dev->wait_recvd_msg);
		bus_message_received = false;
	}
	if (complete_list.status || list_empty(&complete_list.txei_cb.cb_list))
		goto out;

	list_for_each_entry_safe(cb_pos, cb_next,
		&complete_list.txei_cb.cb_list, cb_list) {

		cl = (struct txei_cl *) cb_pos->file_private;
		list_del(&cb_pos->cb_list);
		if (cl) {
			txei_dbg(dev, "completing call back.\n");
			txei_complete_callback(cl, cb_pos);
			cb_pos = NULL;
		}
	}

out:
	/**
	 * We cannot handle write and read at one pass because it will
	 * interlock due to the hardware design.
	 */
	if (ipc_pending_interrupts(dev)) {
		txei_dbg(dev, "Pending: InReady=%d, Readiness=%d, Aliveness=%d, OutputDoorbell=%d\n",
			dev->ipc_interrupt_cause.IPCInputReadyInt,
			dev->ipc_interrupt_cause.ReadinessInt,
			dev->ipc_interrupt_cause.AlivenessInt,
			dev->ipc_interrupt_cause.OutputDoorbellInt);
		goto again;
	}


	ipc_host_interrupts_enable(dev);
	return IRQ_HANDLED;
}

#ifdef POLLING_MODE

int polling_thread(void *_dev)
{
	struct txei_device *dev = _dev;
	struct sched_param param = { .sched_priority = 1 };
	unsigned long period /* , idle_period */;

	sched_setscheduler(current, SCHED_FIFO, &param);

	do {
		period = msecs_to_jiffies(100);
		if (dev->stop_polling_thread)
			break;

		if (ipc_interrupts_enabled(dev)) {
			if (ipc_check_and_ack_interrupts(dev, true) ||
				ipc_pending_interrupts(dev)) {
				txei_interrupt_thread_handler(0, dev);
			}
		}
		set_current_state(TASK_INTERRUPTIBLE);
		if (!kthread_should_stop())
			schedule_timeout(period);

		set_current_state(TASK_RUNNING);
	} while (!kthread_should_stop());

	pr_debug("polling thread is being stopped\n");
	dev->stop_polling_thread = false;
	wake_up_interruptible_all(&dev->wait_polling_thread_terminated);
	return 0;
}
#endif  /* POLLING_MODE */
