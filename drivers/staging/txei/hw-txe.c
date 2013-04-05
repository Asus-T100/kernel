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
#include <linux/jiffies.h>

#include "txei.h"

#include "txei_dev.h"
#include "hw-txe.h"

/**
 * Sets the Aliveness bit.
 * @dev: the device structure
 * @req: the value of aliveness bit to be written
 */
void ipc_aliveness_set(struct txei_device *dev, u32 req)
{
	txei_dbg(dev, "Aliveness current=%d request=%d\n",
		dev->aliveness, req);
	/* Clear requested */
	if (req)
		dev->aliveness_atime = jiffies;

	if (dev->aliveness != req) {
		dev->recvd_aliv_resp = false;
		ipc_bridge_reg_write(dev, SICR_HOST_ALIVENESS_REQ_OFFSET, req);
	}
}


/*
 * ipc_aliveness_req_get - reads SICR_HOST_ALIVENESS_REQ_OFFSET
 * @dev: the device structure
 */
u32 ipc_aliveness_req_get(struct txei_device *dev)
{
	u32 reg;
	reg = ipc_bridge_reg_read(dev, SICR_HOST_ALIVENESS_REQ_OFFSET);
	return reg & TXEI_SICR_HOST_ALIVENESS_REQ_REQUESTED;
}
/*
 * Reads nad returns the HICR_HOST_ALIVENESS_RESP register value
 * @dev: the device structure
 */
u32 ipc_aliveness_get(struct txei_device *dev)
{
	u32 reg;
	reg = ipc_bridge_reg_read(dev, HICR_HOST_ALIVENESS_RESP_OFFSET);
	return reg & TXEI_HICR_HOST_ALIVENESS_RESP_ACK;
}

/**
 * ipc_input_ready_interrupt_enable - sets the Input Ready
 *	Interrupt and check its enablement.
 * @dev: the device structure
 */
void ipc_input_ready_interrupt_enable(struct txei_device *dev)
{
	u32 hintmsk;
	/* Enable the  SEC_IPC_HOST_INT_MASK_IN_RDY interrupt */
	hintmsk = ipc_sec_reg_read(dev, SEC_IPC_HOST_INT_MASK_OFFSET);
	hintmsk |= SEC_IPC_HOST_INT_MASK_IN_RDY;
	ipc_sec_reg_write(dev, SEC_IPC_HOST_INT_MASK_OFFSET, hintmsk);
}

/**
* Sets bit 0 in SEC_IPC_INPUT_DOORBELL.IPC_INPUT_DOORBELL.
* @dev: the device structure
*/
void
ipc_input_doorbell_set(struct txei_device *dev)
{
	/* Clear the interrupt cause */
	dev->ipc_interrupt_cause.IPCInputReadyInt = false;
	ipc_sec_reg_write(dev, SEC_IPC_INPUT_DOORBELL_OFFSET, 1);
}

/*
 * ipc_host_ready_set
 * Sets the HICR_SEC_IPC_READINESS.HostReady bit to 1
 * @dev: the device structure
 */

/**
 * ipc_output_ready_set - Sets the SICR_SEC_IPC_OUTPUT_STATUS bit to 1
 * @dev: the device structure
 */
void ipc_output_ready_set(struct txei_device *dev)
{
	ipc_bridge_reg_write(dev,
			SICR_SEC_IPC_OUTPUT_STATUS_OFFSET,
			SEC_IPC_OUTPUT_STATUS_RDY);
}

/**
* Checks and returns if Input-Ready (i.e. SeC is ready for receiving data)
* @dev: the device structure
*/
bool ipc_input_ready_get(struct txei_device *dev)
{
	u32 status;
	status = ipc_sec_reg_read(dev, SEC_IPC_INPUT_STATUS_OFFSET);
	return !!(SEC_IPC_INPUT_STATUS_RDY & status);
}

/**
 * ipc_interrupts_enabled -
 *	Checks and returns if all interrupts are enabled
 * @dev: the device structure
 */
bool ipc_interrupts_enabled(struct txei_device *dev)
{
	u32 host_hier;
	u32 host_ier;

	host_hier = ipc_bridge_reg_read(dev, HHIER_OFFSET);
	host_ier = ipc_bridge_reg_read(dev, HIER_OFFSET);
	host_ier &= TXEI_HIER_INT_EN_MSK;

	return ((host_hier == TXEI_IPC_HHIER_MSK) &&
		(host_ier == TXEI_HIER_INT_EN_MSK));
}

/**
* Sets all the host interrupts to the given value
* @dev: the device structure
* @enablementBit: the enablement bit to be set to all host interrupts
*/
void ipc_host_interrupts_disable(struct txei_device *dev)
{
	ipc_bridge_reg_write(dev, HHIER_OFFSET, 0);
	ipc_bridge_reg_write(dev, HIER_OFFSET,  0);
}

void ipc_host_interrupts_enable(struct txei_device *dev)
{
	ipc_bridge_reg_write(dev, HHIER_OFFSET, TXEI_IPC_HHIER_MSK);
	ipc_bridge_reg_write(dev, HIER_OFFSET, TXEI_HIER_INT_EN_MSK);
}

/**
* Checks and returns if an interrupt was generated, and if so it will clear it.
* @dev: the device structure
* @do_ack: determines if to acknowledge the generated interrupt(s) or not
*/
bool ipc_pending_interrupts(struct txei_device *dev)
{

	bool ret = (dev->ipc_interrupt_cause.AlivenessInt ||
		dev->ipc_interrupt_cause.IPCInputReadyInt ||
		dev->ipc_interrupt_cause.OutputDoorbellInt > 0);

	if (ret) {
		txei_dbg(dev, "Pending Interrupts Alivnes=%d, Readiness=%d, IPCInReady=%d, OutDoor=%d\n",
			dev->ipc_interrupt_cause.AlivenessInt,
			dev->ipc_interrupt_cause.ReadinessInt,
			dev->ipc_interrupt_cause.IPCInputReadyInt,
			dev->ipc_interrupt_cause.OutputDoorbellInt);
	}
	return ret;
}


void ipc_input_payload_write(struct txei_device *dev,
	unsigned long index, u32 value)
{
	ipc_sec_reg_write(dev, SEC_IPC_INPUT_PAYLOAD_OFFSET +
			(index * sizeof(u32)), value);
}

u32 ipc_output_payload_read(struct txei_device *dev, unsigned long index)
{

	return ipc_bridge_reg_read(dev, BRIDGE_IPC_OUTPUT_PAYLOAD_OFFSET +
				(index * sizeof(u32)));
}

void ipc_readiness_set_host_rdy(struct txei_device *dev)
{
	ipc_bridge_reg_write(dev,
		SICR_HOST_IPC_READINESS_REQ_OFFSET,
		TXEI_SICR_HOST_IPC_READINESS_HOST_RDY);
}

void ipc_readiness_clear(struct txei_device *dev)
{
	ipc_bridge_reg_write(dev, SICR_HOST_IPC_READINESS_REQ_OFFSET,
				TXEI_SICR_HOST_IPC_READINESS_RDY_CLR);
}
/**
* Reads nad returns the HICR_SEC_IPC_READINESS register value
*/
u32 ipc_readiness_get(struct txei_device *dev)
{
	return ipc_bridge_reg_read(dev, HICR_SEC_IPC_READINESS_OFFSET);
}

/**
* Wait for HICR_HOST_IPC_READINESS.SEC_RDY to be set
* Return: >= 0 if the SEC_RDY bit was set after wait and -ETIMEDOUT otherwise
* This function expects the device_lock to be lucked at invocation and it will
* return with device_lock locked.
*/
int ipc_readiness_poll(struct txei_device *dev)
{
	int t = 0;
	u32 readiness;
	/*
	 * Wait for an interrupt indicating that
	 * HICR_HOST_IPC_READINESS.SEC_RDY bit is set
	 */
	do {
		readiness = ipc_readiness_get(dev);
		if (readiness & TXEI_HICR_SEC_IPC_READINESS_SEC_RDY) {
			txei_dbg(dev, "readiness settled after %d msec\n", t);
			return t;
		}
		mutex_unlock(&dev->device_lock);
		msleep(MSEC_PER_SEC / 100);
		mutex_lock(&dev->device_lock);
		t += MSEC_PER_SEC / 100;
	} while (t < SEC_READY_WAIT_TIMEOUT);

	return -ETIMEDOUT;
}

/**
 * This function expects the device_lock to be locked
 * at invocation and will leave device_locked upon exit
 * It will unlock device_lock upon wait event interruptable
 */
int ipc_readiness_wait(struct txei_device *dev)
{
	u32 readiness;
	int err;

	readiness =  ipc_readiness_get(dev);
	if (readiness & TXEI_HICR_SEC_IPC_READINESS_SEC_RDY)
		return 0;

	mutex_unlock(&dev->device_lock);
	err = wait_event_interruptible_timeout(dev->wait_readiness_int,
			dev->recvd_readiness_int, SEC_READY_WAIT_TIMEOUT);
	mutex_lock(&dev->device_lock);
	if (!err && !dev->recvd_readiness_int) {
		txei_dbg(dev, "wait_event_interruptible_timeout failed. status =0x%x\n",
			err);
		return -ETIMEDOUT;
	}

	dev->recvd_readiness_int = false;
	dev->readiness_state = ipc_readiness_get(dev);
	return 0;
}

/**
* Wait for HICR_HOST_ALIVENESS_RESP.ALIVENESS_RESP to be set
* Return: TRUE if the AlivenessACK bit was set after wait and FALSE otherwise
* This function expects the device_lock to be locked upon invocation and
* will leave device_lock locked. It will  unlock for the msleep call
*/
int ipc_aliveness_poll(struct txei_device *dev, u32 expected)
{
	int t = 0;

	do {
		dev->aliveness = ipc_aliveness_get(dev);
		if (dev->aliveness == expected) {
			txei_dbg(dev, "alivness settled after %d msec\n", t);
			return t;
		}
		mutex_unlock(&dev->device_lock);
		msleep(MSEC_PER_SEC / 5);
		mutex_lock(&dev->device_lock);
		t += MSEC_PER_SEC / 5;
	} while (t < SEC_ALIVENESS_WAIT_TIMEOUT);

	txei_err(dev, "timed out\n");
	return -ETIMEDOUT;
}

/**
 * This function expects the device_lock to be locked upon invocation and
 * will leave device_lock locked. It will  unlock for the wait event call
 */
long ipc_aliveness_wait(struct txei_device *dev, u32 expected)
{
	long err = 0;
	const unsigned long timeout =
			msecs_to_jiffies(SEC_ALIVENESS_WAIT_TIMEOUT);

	dev->aliveness = ipc_aliveness_get(dev);
	if (dev->aliveness != expected) {
		mutex_unlock(&dev->device_lock);
		err = wait_event_interruptible_timeout(dev->wait_aliveness_resp,
			dev->recvd_aliv_resp, timeout);
		mutex_lock(&dev->device_lock);
		if (!err && !dev->recvd_aliv_resp) {
			txei_err(dev, "aliveness timed out = 0x%ld\n", err);
			return -ETIMEDOUT;
		}
	}
	txei_dbg(dev, "alivness settled after %d msec\n",
		jiffies_to_msecs(timeout - err));
	dev->recvd_aliv_resp = false;
	return err;
}

void ipc_aliveness_timer(struct work_struct *work)
{
	struct txei_device *dev =
		container_of(work, struct txei_device,  aliveness_timer.work);
	/*
	 * if we passed the time dassert the alivness
	 * else schedule next attempt after last access
	 */
	txei_dbg(dev, "aliveness timer jiffies=%ld timeout=%ld atime=%ld\n",
		jiffies, dev->aliveness_timeout, dev->aliveness_atime);
	if (dev->dev_state != TXEI_DEV_ENABLED)
		return;

	mutex_lock(&dev->device_lock);

	if (time_after(jiffies, dev->aliveness_atime + dev->aliveness_timeout))
			ipc_aliveness_set(dev, 0);
	else
		schedule_delayed_work(&dev->aliveness_timer,
			dev->aliveness_timeout);

	mutex_unlock(&dev->device_lock);
}




/**
 * txei_write_message - writes a message to txei device.
 *
 * @dev: the device structure
 * @header: header of message
 * @write_buffer: message buffer will be written
 * @write_length: message size will be written
 *
 * Note that this requires the device lock to be locked
 * upon entry
 *
 * returns 1 if success, 0 - otherwise.
 */
int txei_write_message(struct txei_device *dev,
	struct txei_msg_hdr *header,
	unsigned char *write_buffer,
	unsigned long write_length)
{
	u32 temp_msg = 0;
	unsigned long bytes_written = 0;
	unsigned long dw_index = 0;

	txei_dbg(dev, "txei_write_message header = %08x.\n", *((u32 *) header));

	if ((write_length + sizeof(struct txei_msg_hdr)) > PAYLOAD_SIZE) {
		txei_dbg(dev, "write lenght exceeded = %ld\n", write_length);
		return 0;
	}

	if (!nopg && !dev->aliveness) {
		txei_dbg(dev, "Aliveness not set .... requesting\n");
		ipc_aliveness_set(dev, 1);
		/* Note that this call requires device lock to be locked */
		if (ipc_aliveness_wait(dev, 1) < 0)
			return 0;
	}

	/* Enable Input Ready Interrupt. */
	ipc_input_ready_interrupt_enable(dev);

	if (!ipc_input_ready_get(dev)) {
		txei_dbg(dev, "Input is not ready");
		return 0;
	}

	/* Write header */
	ipc_sec_reg_write(dev, SEC_IPC_INPUT_PAYLOAD_OFFSET, *((u32 *) header));
	dw_index++;

	while (write_length >= 4) {
		ipc_input_payload_write(dev, dw_index,
			*(u32 *) (write_buffer + bytes_written));
		bytes_written += 4;
		write_length -= 4;
		dw_index++;
	}

	if (write_length > 0) {
		memcpy(&temp_msg, &write_buffer[bytes_written], write_length);
		ipc_input_payload_write(dev, dw_index, temp_msg);
	}

	dev->hbuf_is_ready = false;
	/* Set Input-Doorbell */
	ipc_input_doorbell_set(dev);

	return 1;
}

size_t txei_hbuf_max_len(const struct txei_device *dev)
{
	return PAYLOAD_SIZE - sizeof(struct txei_msg_hdr);
}

/**
 * txei_read_slots - reads a message from txei device.
 *
 * @dev: the device structure
 * @buffer: message buffer will be written
 * @buffer_length: message size will be read
 */
void txei_read_data(struct txei_device *dev,
		     unsigned char *buffer, unsigned long buffer_length)
{
	u32 i;
	u32 *reg_buf = (u32 *)buffer;
	u32 rem = buffer_length & 0x3;

	txei_dbg(dev, "buffer-length = %lu buf[0]0x%08X\n",
		buffer_length, ipc_output_payload_read(dev, 0));
	for (i = 0; i < buffer_length / 4; i++) {
		/* skip header: index starts from 1 */
		u32 reg = ipc_output_payload_read(dev, i + 1);
		txei_dbg(dev, "buf[%d] = 0x%08X\n", i, reg);
		*reg_buf++ = reg;
	}

	if (rem) {
		u32 reg = ipc_output_payload_read(dev, i + 1);
		memcpy(reg_buf, &reg, rem);
	}

	ipc_output_ready_set(dev);
}

/**
 * txei_flow_ctrl_creds - checks flow_control credentials.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if txei_flow_ctrl_creds >0, 0 - otherwise.
 *	-ENOENT if txei_cl is not present
 *	-EINVAL if single_recv_buf == 0
 */
int txei_flow_ctrl_creds(struct txei_device *dev, struct txei_cl *cl)
{
	int i;

	if (!dev->me_clients_num)
		return 0;

	if (cl->txei_flow_ctrl_creds > 0)
		return 1;

	for (i = 0; i < dev->me_clients_num; i++) {
		struct txei_me_client  *me_cl = &dev->me_clients[i];
		if (me_cl->client_id == cl->me_client_id) {
			if (me_cl->txei_flow_ctrl_creds) {
				if (WARN_ON(me_cl->props.single_recv_buf == 0))
					return -EINVAL;
				return 1;
			} else {
				return 0;
			}
		}
	}
	return -ENOENT;
}

/**
 * txei_flow_ctrl_reduce - reduces flow_control.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 * @returns
 *	0 on success
 *	-ENOENT when me client is not found
 *	-EINVAL wehn ctrl credits are <= 0
 */
int txei_flow_ctrl_reduce(struct txei_device *dev, struct txei_cl *cl)
{
	int i;

	if (!dev->me_clients_num)
		return -ENOENT;

	for (i = 0; i < dev->me_clients_num; i++) {
		struct txei_me_client  *me_cl = &dev->me_clients[i];
		if (me_cl->client_id == cl->me_client_id) {
			if (me_cl->props.single_recv_buf != 0) {
				if (WARN_ON(me_cl->txei_flow_ctrl_creds <= 0))
					return -EINVAL;
				dev->me_clients[i].txei_flow_ctrl_creds--;
			} else {
				if (WARN_ON(cl->txei_flow_ctrl_creds <= 0))
					return -EINVAL;
				cl->txei_flow_ctrl_creds--;
			}
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * txei_send_flow_control - sends flow control to fw.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if success, 0 - otherwise.
 */
int txei_send_flow_control(struct txei_device *dev, struct txei_cl *cl)
{
	struct txei_msg_hdr *txei_hdr;
	struct hbm_flow_control *txei_flow_control;

	txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
	txei_hdr->host_addr = 0;
	txei_hdr->me_addr = 0;
	txei_hdr->length = sizeof(struct hbm_flow_control);
	txei_hdr->msg_complete = 1;
	txei_hdr->reserved = 0;

	txei_flow_control = (struct hbm_flow_control *) &dev->wr_msg_buf[1];
	memset(txei_flow_control, 0, sizeof(txei_flow_control));
	txei_flow_control->host_addr = cl->host_client_id;
	txei_flow_control->me_addr = cl->me_client_id;
	txei_flow_control->hbm_cmd = TXEI_FLOW_CONTROL_CMD;
	memset(txei_flow_control->reserved, 0,
			sizeof(txei_flow_control->reserved));
	txei_dbg(dev, "sending flow control host client = %d, ME client = %d\n",
		cl->host_client_id, cl->me_client_id);
	if (!txei_write_message(dev, txei_hdr,
		(unsigned char *) txei_flow_control,
		sizeof(struct hbm_flow_control)))

		return 0;

	return 1;

}

/**
 * txei_other_client_is_connecting - checks if other
 *    client with the same client id is connected.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if other client is connected, 0 - otherwise.
 */
int txei_other_client_is_connecting(struct txei_device *dev,
				struct txei_cl *cl)
{
	struct txei_cl *cl_pos = NULL;
	struct txei_cl *cl_next = NULL;

	list_for_each_entry_safe(cl_pos, cl_next, &dev->file_list, link) {
		if ((cl_pos->state == TXEI_FILE_CONNECTING) &&
			(cl_pos != cl) &&
			cl->me_client_id == cl_pos->me_client_id)
			return 1;

	}
	return 0;
}

/**
 * txei_disconnect - sends disconnect message to fw.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if success, 0 - otherwise.
 */
int txei_disconnect(struct txei_device *dev, struct txei_cl *cl)
{
	struct txei_msg_hdr *txei_hdr;
	struct hbm_client_connect_request *txei_cli_disconnect;

	txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
	txei_hdr->host_addr = 0;
	txei_hdr->me_addr = 0;
	txei_hdr->length = sizeof(struct hbm_client_connect_request);
	txei_hdr->msg_complete = 1;
	txei_hdr->reserved = 0;

	txei_cli_disconnect =
	    (struct hbm_client_connect_request *) &dev->wr_msg_buf[1];
	memset(txei_cli_disconnect, 0, sizeof(txei_cli_disconnect));
	txei_cli_disconnect->host_addr = cl->host_client_id;
	txei_cli_disconnect->me_addr = cl->me_client_id;
	txei_cli_disconnect->hbm_cmd = CLIENT_DISCONNECT_REQ_CMD;
	txei_cli_disconnect->reserved = 0;

	if (!txei_write_message(dev, txei_hdr,
				(unsigned char *) txei_cli_disconnect,
				sizeof(struct hbm_client_connect_request))) {
		return 0;
	}

	return 1;
}

/**
 * txei_connect - sends connect message to fw.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if success, 0 - otherwise.
 */
int txei_connect(struct txei_device *dev, struct txei_cl *cl)
{
	struct txei_msg_hdr *txei_hdr;
	struct hbm_client_connect_request *txei_cli_connect;

	txei_hdr = (struct txei_msg_hdr *) &dev->wr_msg_buf[0];
	txei_hdr->host_addr = 0;
	txei_hdr->me_addr = 0;
	txei_hdr->length = sizeof(struct hbm_client_connect_request);
	txei_hdr->msg_complete = 1;
	txei_hdr->reserved = 0;

	txei_cli_connect =
	    (struct hbm_client_connect_request *) &dev->wr_msg_buf[1];
	txei_cli_connect->host_addr = cl->host_client_id;
	txei_cli_connect->me_addr = cl->me_client_id;
	txei_cli_connect->hbm_cmd = CLIENT_CONNECT_REQ_CMD;
	txei_cli_connect->reserved = 0;
	txei_dbg(dev, "connect host=%d me=%d\n",
		cl->host_client_id, cl->me_client_id);

	if (!txei_write_message(dev, txei_hdr,
		(unsigned char *) txei_cli_connect,
		sizeof(struct hbm_client_connect_request)))

		return 0;

	return 1;
}
