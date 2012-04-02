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
#include <asm/intel_scu_ipc.h>

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

#define DLP_CTRL_CMD_TO_STR(id) \
	(id == DLP_CMD_BREAK		? "BREAK" :	\
	(id == DLP_CMD_ECHO			? "ECHO" : \
	(id == DLP_CMD_NOP)			? "NOP" : \
	(id == DLP_CMD_CONF_CH)		? "CONF_CH" : \
	(id == DLP_CMD_OPEN_CONN)	? "OPEN_CONN" : \
	(id == DLP_CMD_CANCEL_CONN) ? "CANCEL_CONN" : \
	(id == DLP_CMD_ACK)			? "ACK" : \
	(id == DLP_CMD_NACK)		? "NACK" : \
	(id == DLP_CMD_OPEN_CONN_OCTET)  ? "OPEN_CONN_OCTET" : \
	(id == DLP_CMD_CREDITS)		? "CREDITS" : "UNKNOWN"))

#define DLP_CTRL_CTX	(dlp_drv.channels[DLP_CHANNEL_CTRL]->ch_data)

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

/* Modem cold boot management */
#define V1P35CNT_W	0x0E0		/* PMIC reg to power off the modem */
#define V1P35_OFF	4
#define V1P35_ON	6

#define COLD_BOOT_DELAY_OFF	20000	/* 20 ms (use of usleep_range) */
#define COLD_BOOT_DELAY_ON	10000	/* 10 ms (use of usleep_range) */

/* Delays for powering up/resetting the modem */
#define DLP_DURATION_ON1	60	/* ON1 pulse delay: 60 us */
#define DLP_DURATION_RST	1	/* RST_BBN pulse delay: 1 ms */
#define DLP_POST_DELAY		200 /* Post seq. sleep duration: 200 ms */


/**
 * struct dlp_command_params - DLP modem comamnd/response
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
 * struct dlp_command - DLP comamnd
 * @params: DLP modem comamnd/response
 * @channel: the DLP channel context
 * @status: Status of the transfer when completed
 */
struct dlp_command {
	struct dlp_command_params params;

	struct dlp_channel *channel;
	int status;
};

/**
 * struct dlp_ctrl_reset_ctx - reset context
 * @cd_irq: the modem core dump interrupt line
 * @rst_irq: the modem reset interrupt line
 * @ongoing: Stating that a reset is ongoing
 */
struct dlp_ctrl_reset_ctx {
	int	cd_irq;
	int	rst_irq;
	int	ongoing;
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

	/* Reset & Hangup contexts */
	struct dlp_ctrl_reset_ctx reset;
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

	CRITICAL("Modem CORE_DUMP 0x%x",
			gpio_get_value(ctrl_ctx->gpio_fcdp_rb));

	/* Call registered channels */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = dlp_drv.channels[i];
		if (ch_ctx && ch_ctx->modem_coredump_cb)
			ch_ctx->modem_coredump_cb(ch_ctx);
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

	CRITICAL("Modem RESET_OUT 0x%x, reset_ongoing: %d",
			value, ctrl_ctx->reset.ongoing);
	if (ctrl_ctx->reset.ongoing) {
		/* Rising EDGE (Reset done) ? */
		if (value)
			complete(&ctrl_ctx->reset_done);

		goto out;
	}

	/* Unexpected reset received */
	ctrl_ctx->reset.ongoing = 1;

	/* Call registered channels */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = dlp_drv.channels[i];
		if (ch_ctx) {
			/* Call any register callback */
			if (ch_ctx->modem_reset_cb)
				ch_ctx->modem_reset_cb(ch_ctx);

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
static int
dlp_ctrl_free_gpios(struct dlp_channel *ch_ctx,
					struct device *dev)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	gpio_free(ctrl_ctx->gpio_fcdp_rb);
	gpio_free(ctrl_ctx->gpio_mdm_rst_out);
	gpio_free(ctrl_ctx->gpio_mdm_pwr_on);
	gpio_free(ctrl_ctx->gpio_mdm_rst_bbn);

	if (ctrl_ctx->reset.cd_irq)
		free_irq(ctrl_ctx->reset.cd_irq, dev);

	if (ctrl_ctx->reset.rst_irq)
		free_irq(ctrl_ctx->reset.rst_irq, dev);

	EPILOG();
	return 0;
}

/*
* @brief Configure IRQs & GPIOs
*
* @param ch_ctx
* @param dev
*
* @return
*/
static inline int
dlp_ctrl_configure_gpio(int gpio,
						int direction,
						int value,
						const char *desc)
{
	int ret;

	ret = gpio_request(gpio, "ifxHSIModem");

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		CRITICAL("Unable to configure GPIO%d (%s)",
			 gpio,
			 desc);
		ret = -ENODEV;
	}

	return ret;
}

/*
* @brief This function is:
*	- requesting all needed gpios
*	- requesting all needed irqs
*	- registering irqs callbacks
*
* @param ch_ctx : Channel context
* @param dev : Device driver info
*/
static int
dlp_ctrl_setup_irq_gpio(struct dlp_channel *ch_ctx,
						struct device *dev)
{
	int ret;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	/* Configure the RESET_BB gpio */
	ret = dlp_ctrl_configure_gpio(ctrl_ctx->gpio_mdm_rst_bbn,
			1, 1, "RST_BB");
	if (ret)
		goto free_ctx4;

	/* Configure the ON gpio */
	ret = dlp_ctrl_configure_gpio(ctrl_ctx->gpio_mdm_pwr_on,
			1, 1, "ON");
	if (ret)
		goto free_ctx3;

	/* Configure the RESET_OUT gpio & irq */
	ret = dlp_ctrl_configure_gpio(ctrl_ctx->gpio_mdm_rst_out,
			0, 0, "RST_OUT");
	if (ret)
		goto free_ctx2;

	ctrl_ctx->reset.rst_irq = gpio_to_irq(ctrl_ctx->gpio_mdm_rst_out);
	if (ctrl_ctx->reset.rst_irq < 0) {
		ret = -ENODEV;
		goto free_ctx2;
	}

	ret = request_irq(ctrl_ctx->reset.rst_irq,
			  dlp_ctrl_reset_it,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  DRVNAME,
			  ch_ctx);
	if (ret) {
		CRITICAL("IRQ request failed for GPIO%d (RST_OUT)",
			 ctrl_ctx->reset.rst_irq);
		ret = -ENODEV;
		goto free_ctx2;
	}

	/* Configure the CORE_DUMP gpio & irq */
	ret = dlp_ctrl_configure_gpio(ctrl_ctx->gpio_fcdp_rb,
			0, 0, "CORE_DUMP");
	if (ret)
		goto free_all;

	ctrl_ctx->reset.cd_irq = gpio_to_irq(ctrl_ctx->gpio_fcdp_rb);
	if (ctrl_ctx->reset.cd_irq < 0) {
		ret = -ENODEV;
		goto free_all;
	}

	ret = request_irq(ctrl_ctx->reset.cd_irq,
			  dlp_ctrl_coredump_it,
			  IRQF_TRIGGER_RISING, DRVNAME,
			  ch_ctx);
	if (ret) {
		CRITICAL("IRQ request failed for GPIO%d (CORE DUMP)",
			 ctrl_ctx->gpio_fcdp_rb);
		ret = -ENODEV;
		goto free_all;
	}

	pr_info("dlp: GPIO (rst_bbn: %d, pwr_on: %d, rst_out: %d, fcdp_rb: %d)\n",
			ctrl_ctx->gpio_mdm_rst_bbn,
			ctrl_ctx->gpio_mdm_pwr_on,
			ctrl_ctx->gpio_mdm_rst_out,
			ctrl_ctx->gpio_fcdp_rb);

	pr_info("dlp: IRQ  (rst_out: %d, fcdp_rb: %d)\n",
			ctrl_ctx->reset.rst_irq, ctrl_ctx->reset.cd_irq);

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

	if (ctrl_ctx->reset.cd_irq)
		free_irq(ctrl_ctx->reset.cd_irq, dev);

	if (ctrl_ctx->reset.rst_irq)
		free_irq(ctrl_ctx->reset.rst_irq, dev);

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
static struct dlp_command *dlp_ctrl_cmd_alloc(struct dlp_channel *ch_ctx,
					      unsigned char id,
					      unsigned char param1,
					      unsigned char param2,
					      unsigned char param3)
{
	struct dlp_command *dlp_cmd;
	int flags = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;

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
		BUG_ON(1); /* FIXME: We have BIGGGGGG issue */
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
			if (ch_ctx->credits_available_cb)
				ch_ctx->credits_available_cb(ch_ctx);

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
				CRITICAL("Unexpected PDU size: %d => "
						"Expected: %d (ch: %d)",
						ret,
						ch_ctx->pdu_size,
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
		/* pushed again in the controller ==>           */
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
	dlp_save_rx_callbacks(&ctrl_ctx->start_rx_cb, &ctrl_ctx->stop_rx_cb);

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
	dlp_restore_rx_callbacks(&ctrl_ctx->start_rx_cb, &ctrl_ctx->stop_rx_cb);

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
	dlp_restore_rx_callbacks(&ctrl_ctx->start_rx_cb, &ctrl_ctx->stop_rx_cb);

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

/*
* @brief Set the modem readiness (READY/NOT READY) flag
*
* @param value
*/
static inline void dlp_ctrl_set_modem_readiness(unsigned int value)
{
	unsigned long flags;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	dlp_drv.modem_ready = value;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);
}


/*
* @brief A deferred work that :
*	 - Send ECHO & Wait for the modem respose
*	 - Set the modem readiness flag when the modem respond
*
* @param work
*/
static void dlp_ctrl_ipc_readiness(struct work_struct *work)
{
	struct dlp_channel *ch_ctx = dlp_drv.channels[DLP_CHANNEL_CTRL];
	struct dlp_ctrl_context	*ctrl_ctx = ch_ctx->ch_data;
	unsigned char param1, param2, param3;
	int ret = 0;

	PROLOG();

	/* Set the modem readiness state */
	dlp_ctrl_set_modem_readiness(0);

	/* Wait for RESET_OUT */
	wait_for_completion(&ctrl_ctx->reset_done);

	/* Send ECHO continuously until the modem become ready */
	param1 = PARAM1(DLP_ECHO_CMD_CHECKSUM);
	param2 = PARAM2(DLP_ECHO_CMD_CHECKSUM);
	param3 = PARAM3(DLP_ECHO_CMD_CHECKSUM);

	do {
		ret = 0;
		/* Send the ECHO command */
#if 0	/* Not sent because even the response is received the
		   modem is not ready !
		*/
		ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_ECHO,
				DLP_CMD_ECHO, param1, param2, param3);
#endif
		if (ret == 0) {
			/* Set the modem state */
			dlp_ctrl_set_modem_readiness(1);
		}
	} while (ret);

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
void dlp_ctrl_modem_reset(struct dlp_channel *ch_ctx)
{
	struct dlp_channel *ctrl_ch = dlp_drv.channels[DLP_CHANNEL_CTRL];
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;

	PROLOG();

	WARNING("Modem reset requested !");

	/* AP requested reset => just ignore */
	ctrl_ctx->reset.ongoing = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	mdelay(DLP_DURATION_RST);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	msleep(DLP_POST_DELAY);

	EPILOG();
}

/*
* @brief Toggle gpios required to bring up modem power and start modem
*
* @param ch_ctx: IPC CTRL channel
*/
void dlp_ctrl_modem_power(struct dlp_channel *ch_ctx)
{
	struct dlp_channel *ctrl_ch = dlp_drv.channels[DLP_CHANNEL_CTRL];
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;

	PROLOG();

	WARNING("Modem power requested !");

	/* AP requested reset => just ignore */
	ctrl_ctx->reset.ongoing = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 1);
	udelay(DLP_DURATION_ON1);

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 0);
	msleep(DLP_POST_DELAY);

	EPILOG();
}


/*
* @brief Return TRUE when the modem is ready:
*	- The modem has toggled the RESET_OUT pin
*	- The modem has respond to the echo command
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
* @brief Set the RESET ongoing flag value
*
*/
inline void dlp_ctrl_set_reset_ongoing(int ongoing)
{
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	ctrl_ctx->reset.ongoing = ongoing;
}

/*
* @brief Get the RESET ongoing flag value
*
*/
inline int dlp_ctrl_get_reset_ongoing(void)
{
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	return ctrl_ctx->reset.ongoing;
}

/*
* @brief Get the Modem hangup reasons value
*
*/
inline int dlp_ctrl_get_hangup_reasons(void)
{
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	int hangup_reasons;
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	hangup_reasons = ch_ctx->hangup.last_cause | ch_ctx->hangup.cause;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	return hangup_reasons;
}

/*
* @brief
*
* @param hsi_channel
* @param hangup_reasons
*/
inline void dlp_ctrl_set_hangup_reasons(unsigned int hsi_channel,
		int hangup_reasons)
{
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	ch_ctx->hangup.cause |= ((hsi_channel << 4) | hangup_reasons);
	ch_ctx->hangup.last_cause |= ((hsi_channel << 4) | hangup_reasons);
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
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
	ctrl_ctx->readiness_wq = create_singlethread_workqueue(
			DRVNAME "-mdmreadiness");
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

	/* Init the RX/TX contexts */
	init_completion(&ctrl_ctx->rx_done);
	init_completion(&ctrl_ctx->tx_done);

	dlp_xfer_ctx_init(ch_ctx, &ch_ctx->tx,
			  0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx, &ch_ctx->rx,
			  0, 0, 0, NULL, HSI_MSG_READ);

	/* Configure GPIOs */
	ctrl_ctx->gpio_mdm_rst_out = pd->gpio_mdm_rst_out;
	ctrl_ctx->gpio_mdm_pwr_on  = pd->gpio_mdm_pwr_on;
	ctrl_ctx->gpio_mdm_rst_bbn = pd->gpio_mdm_rst_bbn;
	ctrl_ctx->gpio_fcdp_rb	   = pd->gpio_fcdp_rb;

	ret = dlp_ctrl_setup_irq_gpio(ch_ctx, dev);
	if (ret)
		goto free_ctx;

	/* Set ch_ctx, not yet done in the probe */
	dlp_drv.channels[DLP_CHANNEL_CTRL] = ch_ctx;

	/* Push RX pdus for CREDTIS/OPEN_CONN commands */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++)
		dlp_ctrl_push_rx_pdu(ch_ctx);

	/* Start the modem readiness worqeue */
	queue_work(ctrl_ctx->readiness_wq, &ctrl_ctx->readiness_work);

	/* Power on & Reset the modem */
	dlp_ctrl_modem_power(ch_ctx);
	dlp_ctrl_modem_reset(ch_ctx);

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
* @brief Delete any resources allocated by dlp_ctrl_ctx_create
*
* @param ch_ctx : The IPC CTRL channel
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

	/* Free IRQs & GPIOs */
	free_irq(ctrl_ctx->reset.cd_irq, (void *)ch_ctx);
	free_irq(ctrl_ctx->reset.rst_irq, (void *)ch_ctx);

	gpio_free(ctrl_ctx->gpio_fcdp_rb);
	gpio_free(ctrl_ctx->gpio_mdm_rst_out);
	gpio_free(ctrl_ctx->gpio_mdm_pwr_on);
	gpio_free(ctrl_ctx->gpio_mdm_rst_bbn);

	/* Free the CTRL context */
	kfree(ctrl_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);

	EPILOG();
	return ret;
}

/*
* @brief Open the specified IPC channel for communication
*	- Send the OPEN_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The IPC Channel to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_open_channel(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned char param1 = PARAM1(ch_ctx->pdu_size);
	unsigned char param2 = PARAM2(ch_ctx->pdu_size);

	PROLOG();

	/* Send the OPEN_CONN command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_OPEN_CONN,
				DLP_CMD_ACK, param1, param2, 0);

	EPILOG();
	return ret;
}

/*
* @brief Close the specified IPC channel
*	- Send the CANCEL_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The IPC Channel to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_close_channel(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned long flags;
	unsigned char param3 = PARAM1(DLP_DIR_TRANSMIT_AND_RECEIVE);

	PROLOG();

	/* Reset the credits value */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	ch_ctx->credits = 0;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Send the command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
			DLP_CMD_CANCEL_CONN, DLP_CMD_ACK, 0, 0, param3);

	EPILOG();
	return ret;
}


/*
* @brief Modem reset module_param set function
*	- This function is performing a modem reset
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_reset(const char *val, struct kernel_param *kp)
{
	long do_reset;

	PROLOG();

	if (strict_strtol(val, 16, &do_reset) < 0) {
		EPILOG();
		return -EINVAL;
	}

	if (do_reset)
		dlp_ctrl_modem_reset(dlp_drv.channels[DLP_CHANNEL_CTRL]);

	EPILOG();
	return 0;
}

/*
* @brief Return the reset ongoing flag value
*
* @param val
* @param kp
*
* @return 0
*/
static int get_modem_reset(char *val, struct kernel_param *kp)
{
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	return sprintf(val, "%d", ctrl_ctx->reset.ongoing);
}

/*
* @brief Modem reset module_param set function
*	- This function is performing a modem reset
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_power(const char *val, struct kernel_param *kp)
{
	long do_power;

	PROLOG();

	if (strict_strtol(val, 16, &do_power) < 0) {
		EPILOG();
		return -EINVAL;
	}

	if (do_power)
		dlp_ctrl_modem_power(dlp_drv.channels[DLP_CHANNEL_CTRL]);

	EPILOG();
	return 0;
}

/*
 * Modem cold reset sysfs entries
 *
 * To do a modem cold reset we have to:
 * - Set the EXT1P35VREN field to low  during 20ms (V1P35CNT_W PMIC register)
 * - set the EXT1P35VREN field to high during 10ms (V1P35CNT_W PMIC register)
 */
static int dp_modem_cold_reset(const char *val, struct kernel_param *kp)
{
	long do_reset;
	int ret = 0;
	u16 addr = V1P35CNT_W;
	u8 data, def_value;

	PROLOG();

	if (strict_strtol(val, 10, &do_reset) < 0) {
		ret = -EINVAL;
		goto out;
	}

	WARNING("Modem cold reset requested !");

	/* Need to do something ? */
	if (!do_reset)
		goto out;

	/* Read the current registre value */
	ret = intel_scu_ipc_readv(&addr, &def_value, 2);
	if (ret) {
		CRITICAL("intel_scu_ipc_readv() failed (ret: %d)", ret);
		ret = -EINVAL;
		goto exit_modem_cold_reset;
	}

	/* Write the new registre value (V1P35_OFF) */
	data = (def_value & 0xf8) | V1P35_OFF;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev(OFF)  failed (ret: %d)", ret);
		ret = -EINVAL;
		goto exit_modem_cold_reset;
	}
	usleep_range(COLD_BOOT_DELAY_OFF, COLD_BOOT_DELAY_OFF);

	/* Write the new registre value (V1P35_ON) */
	data = (def_value & 0xf8) | V1P35_ON;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev(ON) failed (ret: %d)", ret);
		ret = -EINVAL;
		goto exit_modem_cold_reset;
	}
	usleep_range(COLD_BOOT_DELAY_ON, COLD_BOOT_DELAY_ON);

	/* FIXME : Do we really need this read operation ??? */
	ret = intel_scu_ipc_readv(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_readv() failed (ret: %d)", ret);
		ret = -EINVAL;
		goto exit_modem_cold_reset;
	}

	/* Write back the old registre value */
	data = def_value;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev() failed (ret: %d)", ret);
		ret = -EINVAL;
	}

 exit_modem_cold_reset:
	/* Cold reset failed => Do a modem reset */
	if (ret) {
		CRITICAL("Cold reset failed => Doing a modem reset");
		ret = do_modem_reset(val, kp);
	}

 out:
	EPILOG();
	return ret;
}

/*
* @brief Modem Hangup reasons module_param set function
*	- This function is reseting the provided HUP reasons
*
* @param val
* @param kp
*
* @return 0
*/
static int clear_hangup_reasons(const char *val, struct kernel_param *kp)
{
	long reasons_to_clear;

	PROLOG();

	if (strict_strtol(val, 16, &reasons_to_clear) < 0)
		return -EINVAL;

	if (reasons_to_clear) {
		unsigned long flags;
		struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);

		spin_lock_irqsave(&ch_ctx->lock, flags);
		ch_ctx->hangup.last_cause &= ~reasons_to_clear;
		ch_ctx->hangup.cause &= ~reasons_to_clear;
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
	}

	EPILOG();
	return 0;
}

/*
* @brief Modem Hangup reasons module_param get function
*	- This function return the modem HUP reasons
*
* @param val
* @param kp
*
* @return 0
*/
static int get_hangup_reasons(char *val, struct kernel_param *kp)
{
	unsigned long hangup_reasons;

	PROLOG();
	hangup_reasons = dlp_ctrl_get_hangup_reasons();

	EPILOG();
	return sprintf(val, "%lud", hangup_reasons);
}

module_param_call(cold_reset_modem, dp_modem_cold_reset, NULL, NULL, 0644);
module_param_call(reset_modem, do_modem_reset, get_modem_reset, NULL, 0644);
module_param_call(power_modem, do_modem_power, NULL, NULL, 0644);
module_param_call(hangup_reasons, clear_hangup_reasons, get_hangup_reasons,
		  NULL, 0644);

