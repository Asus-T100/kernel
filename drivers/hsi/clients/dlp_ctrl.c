/*
 * dlp_ctrl.c
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)). This driver is implementing a 5-channel HSI
 * protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/jiffies.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/hsi/hsi_dlp.h>
#include <linux/hsi/hsi.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "dlp_main.h"

#define DEBUG_TAG 0x2
#define DEBUG_VAR dlp_drv.debug

/*
 *   23            16 15            8  7              0
 *  |                |                |                |
 *   ................ ................ ................
 *
 *        PARAM3            PARAM2           PARAM1
 */

#define PARAM1(w)  ((unsigned char) (w))
#define PARAM2(w)  ((unsigned char)((w) >> 8))
#define PARAM3(w)  ((unsigned char)((w) >> 16))

#define LSB(b)    ((unsigned char) ((b) &  0xF))
#define MSB(b)    ((unsigned char) ((b) >> 4))

#define DLP_CMD_TX_TIMOUT		 1000  /* 1 sec */
#define DLP_CMD_RX_TIMOUT	     5000  /* 5 sec */

#define DLP_ECHO_CMD_CHECKSUM	0xACAFFE

#define DLP_CTRL_CMD_TO_STR(id)								\
	(id == DLP_CMD_BREAK		? "BREAK":					\
	(id == DLP_CMD_ECHO			? "ECHO":					\
	(id == DLP_CMD_NOP)			? "NOP":					\
	(id == DLP_CMD_CONF_CH)		? "CONF_CH":				\
	(id == DLP_CMD_OPEN_CONN)	? "OPEN_CONN":				\
	(id == DLP_CMD_CANCEL_CONN) ? "CANCEL_CONN":			\
	(id == DLP_CMD_ACK)			? "ACK":					\
	(id == DLP_CMD_NACK)		? "NACK":					\
	(id == DLP_CMD_OPEN_CONN_OCTET)  ? "OPEN_CONN_OCTET":	\
	(id == DLP_CMD_CREDITS)		? "CREDITS" : "UNKNOWN"))

#define DLP_CTRL_CTX	dlp_drv.channels[DLP_CHANNEL_CTRL]->ch_data

/* DLP commands list */
#define DLP_CMD_BREAK				0x0
#define DLP_CMD_ECHO				0x1
#define DLP_CMD_NOP					0x4
#define DLP_CMD_CONF_CH				0x5
#define DLP_CMD_OPEN_CONN			0x7
#define DLP_CMD_CANCEL_CONN			0xA
#define DLP_CMD_ACK					0xB
#define DLP_CMD_NACK				0xC
#define DLP_CMD_OPEN_CONN_OCTET		0xE
#define DLP_CMD_CREDITS				0xF

/* Flow control */
enum {
	DLP_FLOW_CTRL_NONE,
	DLP_FLOW_CTRL_CREDITS
};

/* Data format */
enum {
	DLP_DATA_FORMAT_RAW,
	DLP_DATA_FORMAT_PACKET
};

/* Direction */
enum {
	DLP_DIR_TRANSMIT,
	DLP_DIR_RECEIVE,
	DLP_DIR_TRANSMIT_AND_RECEIVE
};

/**
 * struct dlp_ctrl_cmd_params - DLP modem comamnd/response
 * @data1: Command data (byte1)
 * @data2: Command data (byte2)
 * @data3: Command data (byte3)
 * @channel: the HSI channel number
 * @id: The command id
 */
struct dlp_command_params {
	unsigned char data1:8;
	unsigned char data2:8;
	unsigned char data3:8;

	unsigned char channel:4;
	unsigned char id:4;
};

/**
 * struct dlp_ctrl_cmd - DLP comamnd
 * @params: DLP modem comamnd/response
 * @channel: the DLP channel context
 * @status: Status of the transfer when completed
 */
struct dlp_command {
	struct dlp_command_params params;

	struct dlp_channel *channel;
	int status;
};

/*
 * struct dlp_ctrl_context - CTRL channel private data
 *
 * @response: Modem response
 * @readiness_wq: Modem readiness worqueue
 * @tx_done: Wait for the command TX (command request) to be sent
 * @rx_done: Wait for the command RX (command response) to be received
 * @start_rx_cb:
 * @stop_rx_cb:
 */
struct dlp_ctrl_context {
	/* Modem readiness work & worqueue */
	struct workqueue_struct *readiness_wq;
	struct work_struct	readiness_work;
	struct completion reset_done;

	/* Modem response */
	struct dlp_command response;

	/* Command RX/TX completion */
	struct completion tx_done;
	struct completion rx_done;

	/* RX start/stop callbacks */
	hsi_client_cb start_rx_cb;
	hsi_client_cb stop_rx_cb;

	/* GPIO */
	unsigned int gpio_mdm_rst_out;
	unsigned int gpio_mdm_pwr_on;
	unsigned int gpio_mdm_rst_bbn;
	unsigned int gpio_fcdp_rb;
};

/*
 * dlp_ctrl_coredump_it	-	Modem has signaled a core dump
 *
 */
static irqreturn_t dlp_ctrl_coredump_it(int irq, void *data)
{
	int i;
	struct dlp_channel *ch_ctx = data;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	CRITICAL("Modem CORE_DUMP 0x%x", gpio_get_value(ctrl_ctx->gpio_fcdp_rb));

	/* Call registered channels */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = dlp_drv.channels[i];
		if (ch_ctx && ch_ctx->modem_coredump_cb) {
			ch_ctx->modem_coredump_cb(ch_ctx);
		}
	}

	EPILOG();
	return IRQ_HANDLED;
}

/*
 * dlp_ctrl_reset_it -	Modem has changed reset state
 *
 */
static irqreturn_t dlp_ctrl_reset_it(int irq, void *data)
{
	int i, value;
	struct dlp_channel *ch_ctx = data;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	value = gpio_get_value(ctrl_ctx->gpio_mdm_rst_out);

	CRITICAL("Modem RESET_OUT 0x%x, reset_ignore: %d",
			value, dlp_drv.reset_ignore);
	if (dlp_drv.reset_ignore) {
		/* Rising EDGE (Reset done) ? */
		if (value)
			complete(&ctrl_ctx->reset_done);

		goto out;
	}

	/* Unexpected reset received */
	dlp_drv.reset_ignore = 1;

	/* Call registered channels */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = dlp_drv.channels[i];
		if (ch_ctx) {
			/* Call any register callback */
			if (ch_ctx->modem_reset_cb) {
				ch_ctx->modem_reset_cb(ch_ctx);
			}

			/* Reset the credits value */
			ch_ctx->credits = 0;
		}
	}

out:
	EPILOG();
	return IRQ_HANDLED;
}

/*
 *
 *
 */
static int dlp_ctrl_free_gpios(struct dlp_channel *ch_ctx,
								struct device *dev)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	gpio_free(ctrl_ctx->gpio_fcdp_rb);
	gpio_free(ctrl_ctx->gpio_mdm_rst_out);
	gpio_free(ctrl_ctx->gpio_mdm_pwr_on);
	gpio_free(ctrl_ctx->gpio_mdm_rst_bbn);

	if (dlp_drv.core_dump_irq)
		free_irq(dlp_drv.core_dump_irq, dev);

	if (dlp_drv.reset_irq)
		free_irq(dlp_drv.reset_irq, dev);

	EPILOG();
	return 0;
}

static int dlp_ctrl_configure_gpios(struct dlp_channel *ch_ctx,
								    struct device *dev)
{
	int ret;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	ret = gpio_request(ctrl_ctx->gpio_mdm_rst_bbn, "ifxHSIModem");
	ret += gpio_direction_output(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	ret += gpio_export(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	if (ret) {
		CRITICAL("Unable to configure GPIO%d (RESET)",
			 ctrl_ctx->gpio_mdm_rst_bbn);
		ret = -ENODEV;
		goto free_ctx4;
	}
	pr_info("dlp: gpio rst_bbn %d\n", ctrl_ctx->gpio_mdm_rst_bbn);

	ret = gpio_request(ctrl_ctx->gpio_mdm_pwr_on, "ifxHSIModem");
	ret += gpio_direction_output(ctrl_ctx->gpio_mdm_pwr_on, 1);
	ret += gpio_export(ctrl_ctx->gpio_mdm_pwr_on, 1);
	if (ret) {
		CRITICAL("Unable to configure GPIO%d (ON)",
			 ctrl_ctx->gpio_mdm_pwr_on);
		ret = -ENODEV;
		goto free_ctx3;
	}

	pr_info("dlp: gpio pwr_on %d\n", ctrl_ctx->gpio_mdm_pwr_on);

	/* set up irq for modem reset line */
	ret = gpio_request(ctrl_ctx->gpio_mdm_rst_out, "ifxHSIModem");
	ret += gpio_direction_input(ctrl_ctx->gpio_mdm_rst_out);
	ret += gpio_export(ctrl_ctx->gpio_mdm_rst_out, 0);
	if (ret) {
		CRITICAL("Unable to configure GPIO%d (RST_OUT)",
			 ctrl_ctx->gpio_mdm_rst_out);
		ret = -ENODEV;
		goto free_ctx2;
	}

	dlp_drv.reset_irq = gpio_to_irq(ctrl_ctx->gpio_mdm_rst_out);
	if (dlp_drv.reset_irq < 0) {
		ret = -ENODEV;
		goto free_ctx2;
	}

	pr_info("dlp: gpio rst_out %d\n", ctrl_ctx->gpio_mdm_rst_out);

	ret = request_irq(dlp_drv.reset_irq,
			  dlp_ctrl_reset_it,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DRVNAME,
			  ch_ctx);
	if (ret) {
		CRITICAL("IRQ request failed for GPIO%d (RST_OUT)",
			 dlp_drv.reset_irq);
		ret = -ENODEV;
		goto free_ctx2;
	}

	/* set up input for core dump interrupt line */
	ret = gpio_request(ctrl_ctx->gpio_fcdp_rb, "ifxHSIModem");
	ret += gpio_direction_input(ctrl_ctx->gpio_fcdp_rb);
	ret += gpio_export(ctrl_ctx->gpio_fcdp_rb, 0);
	if (ret) {
		CRITICAL("Unable to configure GPIO%d (CORE DUMP)",
			 ctrl_ctx->gpio_fcdp_rb);
		ret = -ENODEV;
		goto free_all;
	}

	dlp_drv.core_dump_irq = gpio_to_irq(ctrl_ctx->gpio_fcdp_rb);
	if (dlp_drv.core_dump_irq < 0) {
		ret = -ENODEV;
		goto free_all;
	}

	ret = request_irq(dlp_drv.core_dump_irq,
			  dlp_ctrl_coredump_it,
			  IRQF_TRIGGER_RISING, DRVNAME,
			  ch_ctx);
	if (ret) {
		CRITICAL("IRQ request failed for GPIO%d (CORE DUMP)",
			 ctrl_ctx->gpio_fcdp_rb);
		ret = -ENODEV;
		goto free_all;
	}

	pr_info("dlp: gpio fcdp_rb %d\n", ctrl_ctx->gpio_fcdp_rb);
	EPILOG();
	return ret;

free_all:
	dlp_ctrl_free_gpios(ch_ctx, dev);
	return ret;

free_ctx2:
	gpio_free(ctrl_ctx->gpio_mdm_rst_out);
free_ctx3:
	gpio_free(ctrl_ctx->gpio_mdm_pwr_on);
free_ctx4:
	gpio_free(ctrl_ctx->gpio_mdm_rst_bbn);

	if (dlp_drv.core_dump_irq)
		free_irq(dlp_drv.core_dump_irq, dev);

	if (dlp_drv.reset_irq)
		free_irq(dlp_drv.reset_irq, dev);

	EPILOG();
	return ret;
}

/****************************************************************************
 *
 * Control flow
 *
 *
 ***************************************************************************/

/*
 *
 */
static void dlp_ctrl_save_rx_callbacks(struct dlp_ctrl_context *ctrl_ctx)
{
	ctrl_ctx->start_rx_cb = dlp_drv.client->hsi_start_rx;
	ctrl_ctx->stop_rx_cb = dlp_drv.client->hsi_stop_rx;

	dlp_drv.client->hsi_start_rx = NULL;
	dlp_drv.client->hsi_stop_rx = NULL;
}

/*
 *
 */
static void dlp_ctrl_restore_rx_callbacks(struct dlp_ctrl_context *ctrl_ctx)
{
	dlp_drv.client->hsi_start_rx = ctrl_ctx->start_rx_cb;
	dlp_drv.client->hsi_stop_rx = ctrl_ctx->stop_rx_cb;

	ctrl_ctx->start_rx_cb = NULL;
	ctrl_ctx->stop_rx_cb = NULL;
}

/*
 *
 */
static struct dlp_command *dlp_ctrl_cmd_alloc(struct dlp_channel *ch_ctx,
					      unsigned char id,
					      unsigned char param1,
					      unsigned char param2,
					      unsigned char param3)
{
	int flags = in_interrupt()? GFP_ATOMIC : GFP_KERNEL;
	struct dlp_command *dlp_cmd;

	/* Allocate DLP command */
	dlp_cmd = kmalloc(sizeof(struct dlp_command), flags);
	if (!dlp_cmd) {
		CRITICAL("Out of memory (dlp_cmd)");
		goto out;
	}

	/* Set command params */
	dlp_cmd->params.id = id;
	dlp_cmd->params.channel = ch_ctx->hsi_channel;
	dlp_cmd->params.data1 = param1;
	dlp_cmd->params.data2 = param2;
	dlp_cmd->params.data3 = param3;

	dlp_cmd->channel = ch_ctx;

out:
	return dlp_cmd;
}

/*
 *
 */
static inline void dlp_ctrl_cmd_free(struct dlp_command *dlp_cmd)
{
	if (dlp_cmd)
		kfree(dlp_cmd);
}

/*
 *
 */
static void dlp_ctrl_msg_destruct(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;

	PROLOG("hsi_ch:%d, cmd:%s, msg:0x%p",
	       dlp_cmd->params.channel,
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), msg);

	/* Delete the received msg */
	dlp_pdu_free(msg, DLP_CTRL_PDU_SIZE);

	/* Delete the command */
	kfree(dlp_cmd);

	EPILOG();
}

/**
 * Synchronous TX message callback
 *
 * @msg: a reference to the HSI msg
 *
 * This function:
 *		- Set the xfer status
 *		- wakeup the caller
 *		- delete the hsi message
 */
static void dlp_ctrl_complete_tx(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

	PROLOG("hsi_ch:%d, cmd:%s, msg:0x%p",
	       dlp_cmd->params.channel,
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), msg);

	dlp_cmd->status = (msg->status == HSI_STATUS_COMPLETED) ? 0 : -EIO;

	/* Command done, notify the sender */
	complete(&ctrl_ctx->tx_done);

	/* Delete the received msg */
	dlp_pdu_free(msg, DLP_CTRL_PDU_SIZE);

	EPILOG();
}

/**
 * Asynchronous TX message callback
 *
 * @msg: a reference to the HSI msg
 *
 * This function:
 *		- delete the hsi message
 */
static void dlp_ctrl_complete_tx_async(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;

	PROLOG("cmd:%s, hsi_ch:%d, msg:0x%p",
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id),
	       dlp_cmd->params.channel, msg);

	/* Delete the received msg */
	dlp_pdu_free(msg, DLP_CTRL_PDU_SIZE);

	/* Delete the command */
	kfree(dlp_cmd);

	EPILOG();
}

/*
 *
 */
static void dlp_ctrl_complete_rx(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	struct dlp_command_params params;
	int hsi_channel, ret, response, msg_complete;

	/* Copy the reponse */
	memcpy(&params,
	       sg_virt(msg->sgt.sgl), sizeof(struct dlp_command_params));

	response = -1;
	msg_complete = (msg->status == HSI_STATUS_COMPLETED);

	PROLOG("cmd:%s, hsi_ch:%d, params: 0x%02X%02X%02X, msg_status: %d",
	       DLP_CTRL_CMD_TO_STR(params.id),
	       params.channel,
	       params.data1, params.data2, params.data3, msg->status);

	hsi_channel = params.channel;
	if ((hsi_channel < 0) || (hsi_channel >= DLP_CHANNEL_COUNT)) {
		CRITICAL("CREDITS: Invalid channel id (%d)", hsi_channel);
		BUG_ON(1);	/* FIXME: We have BIGGGGGG pb (To be removed ?) */
		goto out;
	}

	ch_ctx = dlp_drv.channels[hsi_channel];

	switch (params.id) {
	case DLP_CMD_CREDITS:
		if (msg_complete) {
			unsigned long flags;

			/* Increase the CREDITS counter */
			spin_lock_irqsave(&ch_ctx->lock, flags);
			ch_ctx->credits += params.data3;
			ret = ch_ctx->credits;
			spin_unlock_irqrestore(&ch_ctx->lock, flags);

			/* Credits available ==> Notify the channel */
			if (ch_ctx->credits_available_cb) {
				ch_ctx->credits_available_cb(ch_ctx);
			}

			PTRACE_NO_FUNC("New CREDITS value: %d (+%d)\n", ret,
				       params.data3);
			response = DLP_CMD_ACK;
		}
		break;

	case DLP_CMD_OPEN_CONN:
		if (msg_complete) {
			ret = ((params.data2 << 8) | params.data1);
			response = DLP_CMD_ACK;

			/* Check the requested PDU size */
			if (ch_ctx->pdu_size != ret) {
				CRITICAL
				    ("Unexpected PDU size: %d => Expected: %d (ch: %d)",
				     ret, ch_ctx->pdu_size,
				     ch_ctx->hsi_channel);

				response = DLP_CMD_NACK;
			}
		}
		break;

	default:
		/* Save the modem response */
		ctrl_ctx->response.status = (msg_complete ? 0 : -EIO);

		memcpy(&ctrl_ctx->response.params,
		       &params, sizeof(struct dlp_command_params));

		/* Command done, notify the sender */
		complete(&ctrl_ctx->rx_done);
		break;
	}

	/* Send command response */
	if (response != -1) {
		struct dlp_command *dlp_cmd;
		struct hsi_msg *tx_msg = NULL;

		/* Allocate the DLP command */
		dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx,
					     response,
					     params.data1,
					     params.data2, params.data3);
		if (!dlp_cmd) {
			CRITICAL("Out of memory (dlp_cmd)");
			goto push_rx;
		}

		/* Allocate a new TX msg */
		tx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
				       HSI_MSG_WRITE,
				       DLP_CTRL_PDU_SIZE,
				       1,
				       dlp_cmd,
				       dlp_ctrl_complete_tx_async,
				       dlp_ctrl_msg_destruct);

		if (!tx_msg) {
			CRITICAL("dlp_pdu_alloc(TX) failed");

			/* Delete the command */
			kfree(dlp_cmd);

			goto push_rx;
		}

		/* Copy the command data */
		memcpy(sg_virt(tx_msg->sgt.sgl),
		       &dlp_cmd->params, sizeof(struct dlp_command_params));

		/* Send the TX HSI msg */
		ret = hsi_async(tx_msg->cl, tx_msg);
		if (ret) {
			CRITICAL("hsi_async(TX) failed ! (%s, ret:%d)",
				 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), ret);

			/* Free the TX msg */
			dlp_pdu_free(tx_msg, DLP_CTRL_PDU_SIZE);

			/* Delete the command */
			kfree(dlp_cmd);
		}
	}

push_rx:
	/* Push the RX msg again for futur response */
	ret = hsi_async(msg->cl, msg);
	if (ret) {
		CRITICAL("hsi_async() failed, ret:%d", ret);

		/* We have A BIG PROBLEM if the RX msg cant be  */
		/* pushed again in the controller ==>                   */
		/* No response could be received (FIFO empty)   */

		/* Delete the received msg */
		dlp_pdu_free(msg, DLP_CTRL_PDU_SIZE);
	}

out:
	EPILOG();
}

/**
 * Send an HSI msg AND wait for the status
 *
 * @ch_ctx: a reference to the channel context to use
 * @id: the DLP command id
 * @response_id: the expected response id
 * @param1: the DLP command params
 * @param2: the DLP command params
 * @param3: the DLP command params
 *
 * This function blocks the caller until a response is received
 * or the timeout expires
 */
static int dlp_ctrl_cmd_send(struct dlp_channel *ch_ctx,
			     unsigned char id,
			     unsigned char response_id,
			     unsigned char param1,
			     unsigned char param2, unsigned char param3)
{
	int ret = 0;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	struct dlp_command *dlp_cmd;
	struct hsi_msg *tx_msg = NULL;

	PROLOG("cmd:%s, hsi_ch:%d",
	       DLP_CTRL_CMD_TO_STR(id), ch_ctx->hsi_channel);

	/* Backup RX callback */
	dlp_ctrl_save_rx_callbacks(ctrl_ctx);

	/* Allocate the DLP command */
	dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx, id, param1, param2, param3);
	if (!dlp_cmd) {
		CRITICAL("Out of memory (dlp_cmd)");
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate a new TX msg */
	tx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
			       HSI_MSG_WRITE,
			       DLP_CTRL_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_tx, dlp_ctrl_msg_destruct);

	if (!tx_msg) {
		CRITICAL("dlp_pdu_alloc(TX) failed");
		ret = -ENOMEM;
		goto free_cmd;
	}

	/* Copy the command data */
	memcpy(sg_virt(tx_msg->sgt.sgl),
	       &dlp_cmd->params, sizeof(struct dlp_command_params));

	/* Send the TX HSI msg */
	ret = hsi_async(tx_msg->cl, tx_msg);
	if (ret) {
		CRITICAL("hsi_async(TX) failed ! (%s, ret:%d)",
			 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), ret);

		goto free_tx;
	}

	/* Wait for TX msg to be sent */
	ret = wait_for_completion_timeout(&ctrl_ctx->tx_done,
					  msecs_to_jiffies(DLP_CMD_TX_TIMOUT));
	if (ret == 0) {
		CRITICAL("TX Timeout => %s, hsi_ch: %d",
			 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id),
			 dlp_cmd->params.channel);

		ret = -EIO;
		goto out;
	}

	/* TX msg sent, check the status */
	if (dlp_cmd->status) {
		CRITICAL("Failed to send  %s",
			 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id));

		ret = -EIO;
		goto out;
	}

	/* TX OK, Wait for the response */
	ret = wait_for_completion_timeout(&ctrl_ctx->rx_done,
					  msecs_to_jiffies(DLP_CMD_RX_TIMOUT));
	if (ret == 0) {
		CRITICAL("RX Timeout => %s, hsi_ch: %d",
			 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id),
			 dlp_cmd->params.channel);

		ret = -EIO;
		goto out;
	}

	/* Check the response */
	ret = 0;
	if ((ctrl_ctx->response.params.id != response_id) ||
	    (ctrl_ctx->response.params.data1 != param1) ||
	    (ctrl_ctx->response.params.data2 != param2) ||
	    (ctrl_ctx->response.params.data3 != param3)) {

		CRITICAL("Unexpected response received: 0x%x%x [%02X%02X%02X]"
			 " => expected:  0x%x%x [%02X%02X%02X]",
			 ctrl_ctx->response.params.id,
			 ctrl_ctx->response.params.channel,
			 ctrl_ctx->response.params.data1,
			 ctrl_ctx->response.params.data2,
			 ctrl_ctx->response.params.data3,
			 response_id,
			 ch_ctx->hsi_channel, param1, param2, param3);

		ret = -EIO;
	}

	/* Restore RX callback */
	dlp_ctrl_restore_rx_callbacks(ctrl_ctx);

	/* Everything is OK */
	EPILOG("%d", ret);
	return ret;

free_tx:
	/* Free the TX msg */
	dlp_pdu_free(tx_msg, DLP_CTRL_PDU_SIZE);

free_cmd:
	/* Free the DLP command */
	dlp_ctrl_cmd_free(dlp_cmd);

out:
	/* Restore RX callback */
	dlp_ctrl_restore_rx_callbacks(ctrl_ctx);

	EPILOG("%d", ret);
	return ret;
}

/**
 * Push RX pdu for any modem command
 *
 */
static int dlp_ctrl_push_rx_pdu(struct dlp_channel *ch_ctx)
{
	int ret;
	struct hsi_msg *rx_msg;
	struct dlp_command *dlp_cmd;

	PROLOG();

	/* Allocate the DLP command */
	dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx, DLP_CMD_NOP, 0, 0, 0);
	if (!dlp_cmd) {
		CRITICAL("Out of memory (dlp_cmd)");
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate a new RX msg */
	rx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
			       HSI_MSG_READ,
			       DLP_CTRL_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_rx, dlp_ctrl_msg_destruct);

	if (!rx_msg) {
		CRITICAL("dlp_pdu_alloc() failed");
		ret = -ENOMEM;
		goto free_cmd;
	}

	/* Send the RX HSI msg */
	ret = hsi_async(rx_msg->cl, rx_msg);
	if (ret) {
		CRITICAL("hsi_async() failed, ret:%d", ret);
		ret = -EIO;
		goto free_msg;
	}

	EPILOG();
	return 0;

free_msg:
	/* Free the msg */
	dlp_pdu_free(rx_msg, DLP_CTRL_PDU_SIZE);

free_cmd:
	/* Delete the command */
	kfree(dlp_cmd);

out:
	EPILOG();
	return ret;
}

static inline void dlp_ctrl_set_modem_readiness(unsigned int value)
{
	unsigned long flags;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	dlp_drv.modem_ready = value;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);
}


/*
* @brief
*
* @param work
*/
static void dlp_ctrl_ipc_readiness(struct work_struct *work)
{
	struct dlp_channel *ch_ctx = dlp_drv.channels[DLP_CHANNEL_CTRL];
	struct dlp_ctrl_context	*ctrl_ctx = ch_ctx->ch_data;
	int ret;

	PROLOG();

	/* Set the modem readiness state */
	dlp_ctrl_set_modem_readiness(0);

	/* Wait for RESET_OUT */
	wait_for_completion(&ctrl_ctx->reset_done);

	/* Send ECHO continuously until the modem become ready */
	do {
		ret = 0;
		// ret = dlp_ctrl_send_echo_cmd(ctrl_ctx);
		if (ret == 0) {
			/* Set the modem state */
			dlp_ctrl_set_modem_readiness(1);
		}
	}
	while (ret);

	EPILOG();
}


/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

/**
 * dlp_ctrl_modem_reset - activity required to bring up modem
 * @ch_ctx: a reference to related channel context
 *
 * Toggle gpios required to bring up modem power and start modem.
 * This can be called after the modem has been started to reset it.
 */
int dlp_ctrl_modem_reset(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;
	int ret = 0;

	PROLOG();

	/* AP requested reset => just ignore */
	dlp_drv.reset_ignore = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	mdelay(DLP_POWER_ON_INTERLINE_DELAY);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	msleep(DLP_POWER_ON_POST_DELAY);

	EPILOG();
	return ret;
}

/*
* @brief
*
* @param ch_ctx
*
* @return
*/
static int dlp_ctrl_modem_power(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	/* AP requested reset => just ignore */
	dlp_drv.reset_ignore = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 1);
	msleep(DLP_POWER_ON_INTERLINE_DELAY);

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 0);
	msleep(DLP_POWER_ON_POST_DELAY);

	EPILOG();
	return ret;
}


/*
* @brief
*
* @return
*/
inline unsigned int dlp_ctrl_modem_is_ready(void)
{
	unsigned long flags;
	unsigned int value;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	value = dlp_drv.modem_ready;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	return value;
}

/*
* @brief
*
* @param index
* @param dev
*
* @return
*/
struct dlp_channel *dlp_ctrl_ctx_create(unsigned int index, struct device *dev)
{
	int ret, i;
	struct hsi_client *client = to_hsi_client(dev);
	struct hsi_mid_platform_data *pd = client->device.platform_data;
	struct dlp_channel *ch_ctx;
	struct dlp_ctrl_context *ctrl_ctx;

	PROLOG();

	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		CRITICAL("Unable to allocate memory (ch_ctx)");
		goto out;
	}

	/* Allocate the context private data */
	ctrl_ctx = kzalloc(sizeof(struct dlp_ctrl_context), GFP_KERNEL);
	if (!ctrl_ctx) {
		CRITICAL("Unable to allocate memory (ctrl_ctx)");
		goto free_ch;
	}

	/* Create a workqueue to check the modem readiness */
	ctrl_ctx->readiness_wq = create_singlethread_workqueue(DRVNAME "-mdmreadiness");
	if (!ctrl_ctx->readiness_wq) {
		CRITICAL("Unable to create modem readiness workqueue");
		goto free_ctx;
	}

	/* Save params */
	ch_ctx->ch_data = ctrl_ctx;
	ch_ctx->hsi_channel = index;
	ch_ctx->pdu_size = DLP_CTRL_PDU_SIZE;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_completion(&ctrl_ctx->reset_done);
	INIT_WORK(&ctrl_ctx->readiness_work, dlp_ctrl_ipc_readiness);

	/* Configure GPIOs */
	ctrl_ctx->gpio_mdm_rst_out = pd->gpio_mdm_rst_out;
	ctrl_ctx->gpio_mdm_pwr_on  = pd->gpio_mdm_pwr_on;
	ctrl_ctx->gpio_mdm_rst_bbn = pd->gpio_mdm_rst_bbn;
	ctrl_ctx->gpio_fcdp_rb	   = pd->gpio_fcdp_rb;

	ret = dlp_ctrl_configure_gpios(ch_ctx, dev);
	if (ret) {
		goto free_ctx;
	}

	/* Power on the modem */
	ret = dlp_ctrl_modem_power(ch_ctx);
	if (ret) {
		ret = -ENODEV;
		dlp_ctrl_free_gpios(ch_ctx, dev);
		goto free_ctx;
	}

	/* Reset the modem */
	ret = dlp_ctrl_modem_reset(ch_ctx);
	if (ret) {
		ret = -ENODEV;
		dlp_ctrl_free_gpios(ch_ctx, dev);
		goto free_ctx;
	}

	/* Init the RX/TX completion */
	init_completion(&ctrl_ctx->rx_done);
	init_completion(&ctrl_ctx->tx_done);

	/* Set ch_ctx, not yet done in the probe */
	dlp_drv.channels[DLP_CHANNEL_CTRL] = ch_ctx;

	/* Push RX pdus for CREDTIS/OPEN_CONN commands */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++)
		dlp_ctrl_push_rx_pdu(ch_ctx);

	/* Start the modem readiness worqeue */
	queue_work(ctrl_ctx->readiness_wq, &ctrl_ctx->readiness_work);

	EPILOG();
	return ch_ctx;

free_ctx:
	kfree(ctrl_ctx);

free_ch:
	kfree(ch_ctx);

out:
	EPILOG("Failed");
	return NULL;
}

/*
* @brief
*
* @param ch_ctx
*
* @return
*/
int dlp_ctrl_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;
	int ret = 0;

	PROLOG();

	/* Delete the modem readiness worqueue */
	destroy_workqueue(ctrl_ctx->readiness_wq);

	/* Free GPIOs */

	/* Free the CTRL context */
	kfree(ctrl_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);

	EPILOG();
	return ret;
}

/*
 *
 */
int dlp_ctrl_send_echo_cmd(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned char param1 = PARAM1(DLP_ECHO_CMD_CHECKSUM);
	unsigned char param2 = PARAM2(DLP_ECHO_CMD_CHECKSUM);
	unsigned char param3 = PARAM3(DLP_ECHO_CMD_CHECKSUM);

	PROLOG();

	/* Send the command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_ECHO,
				DLP_CMD_ECHO, param1, param2, param3);
	EPILOG();
	return ret;
}

/*
 *
 */
int dlp_ctrl_send_open_conn_cmd(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned char param1 = PARAM1(ch_ctx->pdu_size);
	unsigned char param2 = PARAM2(ch_ctx->pdu_size);

	PROLOG();

	/* Send the command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_OPEN_CONN,
				DLP_CMD_ACK, param1, param2, 0);

	EPILOG();
	return ret;
}

/*
 *
 */
int dlp_ctrl_send_cancel_conn_cmd(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	// FIXME: both or just xmit/receive ?
	//unsigned char param3 = PARAM1(DLP_DIR_TRANSMIT_AND_RECEIVE);
	unsigned char param3;

	PROLOG();

	// FIXME: TX & RX dont work ==> the modem response is TX only
	param3 = PARAM1(DLP_DIR_TRANSMIT);

	/* Send the command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_CANCEL_CONN, DLP_CMD_ACK, 0, 0, param3);
	EPILOG();
	return ret;
}


/*
 * Modem reset sysfs entries
 */
static int set_modem_reset(const char *val, struct kernel_param *kp)
{
	unsigned long reset_request;

	if (strict_strtoul(val, 16, &reset_request) < 0)
		return -EINVAL;

	if (reset_request) {
		struct dlp_channel *ch_ctx = dlp_drv.channels[DLP_CHANNEL_CTRL];

		dlp_ctrl_modem_reset(ch_ctx);
	}

	return 0;
}

static int get_modem_reset(const char *val, struct kernel_param *kp)
{
	unsigned long reset_ongoing = 0;

	return sprintf(val, "%d", reset_ongoing);
}

/*
 * Modem hangup sysfs entries
 */
static int set_hangup_reasons(const char *val, struct kernel_param *kp)
{
	unsigned long reset_request;

	if (strict_strtoul(val, 16, &reset_request) < 0)
		return -EINVAL;

	return 0;
}

static int get_hangup_reasons(const char *val, struct kernel_param *kp)
{
	unsigned long hangup_reasons = 0;

	return sprintf(val, "%d", hangup_reasons);
}


module_param_call(reset_modem, set_modem_reset, get_modem_reset, NULL, 0644);
module_param_call(hangup_reasons, set_hangup_reasons, get_hangup_reasons,
		  NULL, 0644);

