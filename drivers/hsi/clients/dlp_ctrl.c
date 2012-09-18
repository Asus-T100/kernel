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

#define DEBUG_TAG 0x1
#define DEBUG_VAR (dlp_drv.debug)

/*
 * When enabled, all the communication done on the channel0
 * is logged.
 *
 */
/* #define DLP_CTRL_IO_DEBUG */

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
#define DLP_CMD_RX_TIMOUT	     1000  /* 1 sec */

#define DLP_ECHO_CMD_CHECKSUM	0xACAFFE
#define DLP_NOP_CMD_CHECKSUM	0xE7E7EC

#define DLP_CTRL_CTX	(dlp_drv.channels[DLP_CHANNEL_CTRL]->ch_data)

/* DLP commands list */
#define DLP_CMD_BREAK				0x00
#define DLP_CMD_ECHO				0x01
#define DLP_CMD_NOP					0x04
#define DLP_CMD_CONF_CH				0x05
#define DLP_CMD_OPEN_CONN			0x07
#define DLP_CMD_CLOSE_CONN			0x0A
#define DLP_CMD_ACK					0x0B
#define DLP_CMD_NACK				0x0C
#define DLP_CMD_OPEN_CONN_OCTET		0x0E
#define DLP_CMD_CREDITS				0x0F
#define DLP_CMD_NONE				0xFF

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
#define CHIPCNTRL          0x100
#define CHIPCNTRL_MODEMRST 0x10
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
 * @wq: Modem readiness/TX timeout worqueue
 * @hangup_work: Modem Reset/Coredump work
 * @tx_timeout_work: Modem TX timeout work
 * @readiness_work: Modem readiness work
 * @reset_done: Modem reset wait completion
 * @ready_event: Modem readiness wait event
 * @response: Received response from the modem
 * @start_rx_cb: HSI client start RX CB
 * @stop_rx_cb: HSI client stop RX CB
 * @gpio_mdm_rst_out: Modem RESET_OUT GPIO
 * @gpio_mdm_pwr_on: Modem Power on GPIO
 * @gpio_mdm_rst_bbn: Modem RESET_BBN GPIO
 * @gpio_fcdp_rb: Modem coredump GPIO
 * @reset: The modem reset context parameters
 * @xfers_list: The list of ALL the RX/TX msgs exchanged on channel 0
 */
struct dlp_ctrl_context {
	/* Modem readiness/TX timeout work & worqueue */
	struct workqueue_struct *wq;
	struct work_struct hangup_work;
	struct work_struct tx_timeout_work;
	struct work_struct readiness_work;
	struct completion reset_done;
	wait_queue_head_t ready_event;

	/* Modem response */
	struct dlp_command response;

	/* RX start/stop callbacks */
	hsi_client_cb start_rx_cb;
	hsi_client_cb stop_rx_cb;

	/* GPIO */
	unsigned int gpio_mdm_rst_out;
	unsigned int gpio_mdm_pwr_on;
	unsigned int gpio_mdm_rst_bbn;
	unsigned int gpio_fcdp_rb;

	/* Reset context */
	struct dlp_ctrl_reset_ctx reset;

#ifdef DLP_CTRL_IO_DEBUG
	struct list_head xfers_list;
	int xfers_list_incomplete;
	spinlock_t debug_lock;
#endif
};

#ifdef DLP_CTRL_IO_DEBUG
/*
*
*
*/
struct dlp_ctrl_hsi_xfer {
	struct list_head	link;

	struct dlp_command_params cmd_params;
	unsigned long long timestamp;
	int status;
	int ttype;
};

#define DLP_CTRL_XFERS_LIST_ADD(ctrl_ctx, msg) \
	dlp_ctrl_xfers_list_add(ctrl_ctx, msg)
#define DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx) \
	dlp_ctrl_xfers_list_clear(ctrl_ctx)


/*
 *
 *
 */
void dlp_ctrl_xfers_list_add(struct dlp_ctrl_context *ctrl_ctx,
		struct hsi_msg *msg)
{
	int malloc_flags = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;
	unsigned long flags;
	struct dlp_ctrl_hsi_xfer *xfer;

	xfer = kmalloc(sizeof(struct dlp_ctrl_hsi_xfer), malloc_flags);
	if (!xfer) {
		ctrl_ctx->xfers_list_incomplete = 1;
		return;
	}

	memcpy(&xfer->cmd_params,
	       sg_virt(msg->sgt.sgl),
		   sizeof(struct dlp_command_params));

	xfer->timestamp = sched_clock();
	xfer->status = msg->status;
	xfer->ttype = msg->ttype;

	spin_lock_irqsave(&ctrl_ctx->debug_lock, flags);
	list_add_tail(&xfer->link, &ctrl_ctx->xfers_list);
	spin_unlock_irqrestore(&ctrl_ctx->debug_lock, flags);
}

/*
* @brief Clear the list of all the RX/TX transactions
*
* @return 0
*/
static int dlp_ctrl_xfers_list_clear(struct dlp_ctrl_context *ctrl_ctx)
{
	struct dlp_ctrl_hsi_xfer *xfer;
	struct list_head *pos;
	unsigned long flags;

	PROLOG();

	spin_lock_irqsave(&ctrl_ctx->debug_lock, flags);

	/* Delete all the list entries */
	list_for_each(pos, &ctrl_ctx->xfers_list) {
		xfer = list_entry(pos, struct dlp_ctrl_hsi_xfer, link);

		/* Remove frm the list */
		list_del(&xfer->link);

		/* Free */
		kfree(xfer);
	}

	ctrl_ctx->xfers_list_incomplete = 0;

	spin_unlock_irqrestore(&ctrl_ctx->debug_lock, flags);

	EPILOG();
	return 0;
}

/*
* @brief Display the list of all the RX/TX transactions
*
* @return 0
*/
static void dlp_ctrl_xfers_logs_dump(struct dlp_channel *ch_ctx,
		struct seq_file *m)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;
	struct dlp_ctrl_hsi_xfer *xfer;
	struct list_head *pos;
	unsigned long nanosec_rem, flags;
	unsigned long long t;
	unsigned int i = 0;
	char buf[128], *cmd_id;

	PROLOG();

	seq_printf(m, "\nChannel: %d\n", ch_ctx->hsi_channel);
	seq_printf(m, "-------------\n");
	seq_printf(m, " TX pdu_size  : %d\n", ch_ctx->tx.pdu_size);
	seq_printf(m, " RX xdu_size  : %d\n", ch_ctx->rx.pdu_size);

	spin_lock_irqsave(&ctrl_ctx->debug_lock, flags);

	/* Delete all the list entries */
	list_for_each(pos, &ctrl_ctx->xfers_list) {
		xfer = list_entry(pos, struct dlp_ctrl_hsi_xfer, link);

		/* Dump the xfer params */
		switch (xfer->cmd_params.id) {
		case DLP_CMD_BREAK:
			cmd_id = "BREAK          ";
			break;
		case DLP_CMD_ECHO:
			cmd_id = "ECHO           ";
			break;
		case DLP_CMD_NOP:
			cmd_id = "NOP            ";
			break;
		case DLP_CMD_CONF_CH:
			cmd_id = "CONF_CH        ";
			break;
		case DLP_CMD_OPEN_CONN:
			cmd_id = "OPEN_CONN      ";
			break;
		case DLP_CMD_CLOSE_CONN:
			cmd_id = "CLOSE_CONN    ";
			break;
		case DLP_CMD_ACK:
			cmd_id = "ACK            ";
			break;
		case DLP_CMD_NACK:
			cmd_id = "NACK           ";
			break;
		case DLP_CMD_OPEN_CONN_OCTET:
			cmd_id = "OPEN_CONN_OCTET";
			break;
		case DLP_CMD_CREDITS:
			cmd_id = "CREDITS        ";
			break;
		default:
			cmd_id = "UNKNOWN";
		}

		t = xfer->timestamp;
		nanosec_rem = do_div(t, 1000000000);
		sprintf(buf, "\t%06d => CH_%d: %s [%s] params[0x%02X%02X%02X]"
				" @[%5lu.%06lu] status: %d\n",
				++i,
				xfer->cmd_params.channel,
				(xfer->ttype == HSI_MSG_WRITE) ? "TX" : "RX",
				cmd_id,
				xfer->cmd_params.data1,
				xfer->cmd_params.data2, xfer->cmd_params.data3,
				(unsigned long)t,
				nanosec_rem / 1000,
				xfer->status);

		seq_printf(m, buf);
	}

	if (ctrl_ctx->xfers_list_incomplete) {
		seq_printf(m, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1!!!!!!!!!!!!!!");
		seq_printf(m, "! INCOMPLETE LIST (memory allocation issue) !");
		seq_printf(m, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}

	spin_unlock_irqrestore(&ctrl_ctx->debug_lock, flags);

	EPILOG();
}

#else /* #ifdef DLP_CTRL_IO_DEBUG */

#define DLP_CTRL_XFERS_LIST_ADD(ctrl_ctx, msg)	do {} while (0)
#define DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx)		do {} while (0)
#endif


/*
 * dlp_ctrl_coredump_it	-	Modem has signaled a core dump
 *
 */
static irqreturn_t dlp_ctrl_coredump_it(int irq, void *data)
{
	struct dlp_channel *ch_ctx = data;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	CRITICAL("Modem CORE_DUMP 0x%x",
			gpio_get_value(ctrl_ctx->gpio_fcdp_rb));

	/* Set the reason & launch the work to handle the hangup */
	ch_ctx->hangup.cause |= DLP_MODEM_HU_COREDUMP;
	queue_work(ctrl_ctx->wq, &ctrl_ctx->hangup_work);

	EPILOG();
	return IRQ_HANDLED;
}

/*
 * dlp_ctrl_reset_it -	Modem has changed reset state
 *
 */
static irqreturn_t dlp_ctrl_reset_it(int irq, void *data)
{
	int value, reset_ongoing;
	struct dlp_channel *ch_ctx = data;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	value = gpio_get_value(ctrl_ctx->gpio_mdm_rst_out);
	reset_ongoing = dlp_ctrl_get_reset_ongoing();
	if (reset_ongoing) {
		CRITICAL("Modem RESET_OUT 0x%x", value);

		/* Rising EDGE (Reset done) ? */
		if (value)
			complete(&ctrl_ctx->reset_done);

		goto out;
	}

	CRITICAL("Unexpected modem RESET_OUT 0x%x", value);

	/* Unexpected reset received */
	dlp_ctrl_set_reset_ongoing(1);

	/* Set the reason & launch the work to handle the hangup */
	ch_ctx->hangup.cause |= DLP_MODEM_HU_RESET;
	queue_work(ctrl_ctx->wq, &ctrl_ctx->hangup_work);

out:
	EPILOG();
	return IRQ_HANDLED;
}

/**
 * dlp_ctrl_hsi_tx_timout_cb - timer function for tx timeout
 * @param: a reference to the channel to consider
 *
 */
static void dlp_ctrl_hsi_tx_timout_cb(unsigned long int param)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)param;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

	PROLOG();

	/* Set the reason & launch the work to handle the hangup */
	ch_ctx->hangup.cause |= DLP_MODEM_HU_TIMEOUT;
	queue_work(ctrl_ctx->wq, &ctrl_ctx->tx_timeout_work);

	EPILOG();
}

/**
 * dlp_ctrl_handle_tx_timeout - Manage the HSI TX timeout
 * @work: a reference to work queue element
 *
 * Required since hsi_port->flush calls might sleep
 */
static void dlp_ctrl_handle_tx_timeout(struct work_struct *work)
{
	struct dlp_channel *ch_ctx;
	int i;

	PROLOG();

	/* Call any register TX timeout CB */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = DLP_CHANNEL_CTX(i);
		if ((ch_ctx) && (ch_ctx->modem_tx_timeout_cb))
			ch_ctx->modem_tx_timeout_cb(ch_ctx);
	}

	EPILOG();
}

/**
 * This function handle the modem reset/coredump:
 *   - Flush the current controller FIFOs
 *   - Call each channel CB (to notify the upperlayer about the crash)
 *   - Release the HSI port
 *
 * @work: a reference to work queue element
 *
 * Required since hsi_port->flush calls might sleep
 */
static void dlp_ctrl_handle_hangup(struct work_struct *work)
{
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	int i, modem_rst = ch_ctx->hangup.cause;

	PROLOG();

	/* Check the hangup reason */
	modem_rst = ((modem_rst & DLP_MODEM_HU_RESET) == DLP_MODEM_HU_RESET);

	/* Call registered channels to notify
	 * the upper layer about the modem reset */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = DLP_CHANNEL_CTX(i);
		if (!ch_ctx)
			continue;

		/* call any register callback */
		if (modem_rst) {
			if (ch_ctx->modem_reset_cb)
				ch_ctx->modem_reset_cb(ch_ctx);
		} else if (ch_ctx->modem_coredump_cb)
				ch_ctx->modem_coredump_cb(ch_ctx);

		/* Reset the credits value */
		ch_ctx->credits = 0;
	}

	EPILOG();
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
			1, 0, "RST_BB");
	if (ret)
		goto free_ctx4;

	/* Configure the ON gpio */
	ret = dlp_ctrl_configure_gpio(ctrl_ctx->gpio_mdm_pwr_on,
			1, 0, "ON");
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

	pr_info(DRVNAME ": GPIO (rst_bbn: %d, pwr_on: %d, rst_out: %d, fcdp_rb: %d)\n",
			ctrl_ctx->gpio_mdm_rst_bbn,
			ctrl_ctx->gpio_mdm_pwr_on,
			ctrl_ctx->gpio_mdm_rst_out,
			ctrl_ctx->gpio_fcdp_rb);

	pr_info(DRVNAME ": IRQ  (rst_out: %d, fcdp_rb: %d)\n",
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

/*
 * This function is:
 *  - Calling the HSI hsi_async function
 *  - Adding the HSI msg to the xfers list for debug purpose
 *
 */
static inline int dlp_ctrl_send_hsi_msg(struct dlp_ctrl_context *ctrl_ctx,
		struct hsi_msg *msg,
		int add_to_log)
{
	int ret;

	/* Push the msg to the controller */
	ret = hsi_async(msg->cl, msg);

	/* Need to keep a trace of the msg */
	if (add_to_log)
		DLP_CTRL_XFERS_LIST_ADD(ctrl_ctx, msg);

	return ret;
}


/*
 * This function will check if the CTRL channel ctx is created
 *	(In some special cases (PnP tests for exp), the eDLP protocol will
 *  not be registered and the CTRL channel is not initialized at all,
 *  but the module_param are created
 */
static inline int dlp_ctrl_have_control_context(void)
{
	int have_ctrl = 0;
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);

	if ((ctrl_ch) && (ctrl_ch->ch_data))
		have_ctrl = 1;

	return have_ctrl;
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

	PROLOG("hsi_ch:%d, cmd:0x%X, msg:0x%p",
	       dlp_cmd->params.channel,
	       dlp_cmd->params.id, msg);

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);

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
	struct dlp_channel *ch_ctx = dlp_cmd->channel;

	PROLOG("hsi_ch:%d, cmd:0x%X, msg:0x%p",
	       dlp_cmd->params.channel,
	       dlp_cmd->params.id, msg);

	dlp_cmd->status = (msg->status == HSI_STATUS_COMPLETED) ? 0 : -EIO;

	/* Command done, notify the sender */
	complete(&ch_ctx->tx.cmd_xfer_done);

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);

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

	PROLOG("hsi_ch:%d, cmd:0x%X, msg:0x%p",
			dlp_cmd->params.channel,
			dlp_cmd->params.id, msg);

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);

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

	PROLOG("hsi_ch:%d, cmd:0x%X, params: 0x%02X%02X%02X, msg_status: %d",
			params.channel, params.id,
			params.data1, params.data2, params.data3, msg->status);

	/* Add the received msg to the xfers list */
	DLP_CTRL_XFERS_LIST_ADD(ctrl_ctx, msg);

	hsi_channel = params.channel;
	if ((hsi_channel < 0) || (hsi_channel >= DLP_CHANNEL_COUNT)) {
		CRITICAL("CREDITS: Invalid channel id (%d)", hsi_channel);
		BUG_ON(1); /* FIXME: We have BIGGGGGG issue */
		goto out;
	}

	ch_ctx = DLP_CHANNEL_CTX(hsi_channel);

	switch (params.id) {
	case DLP_CMD_NOP:
		PTRACE_NO_FUNC("NOP received => hsi_ch:%d, "
				"params: 0x%02X%02X%02X\n",
				params.channel,
				params.data1, params.data2, params.data3);
		break;

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
			pr_info(DRVNAME ": ch%d open_conn received\n",
					params.channel);

			/* Check the requested PDU size */
			if (ch_ctx->tx.pdu_size != ret) {
				CRITICAL("Unexpected PDU size: %d => "
						"Expected: %d (ch: %d)",
						ret,
						ch_ctx->tx.pdu_size,
						ch_ctx->hsi_channel);

				response = DLP_CMD_NACK;
			}
		}
		break;

	case DLP_CMD_CLOSE_CONN:
		response = DLP_CMD_ACK;
		pr_info(DRVNAME ": ch%d close_conn received\n", params.channel);
		break;

	default:
		/* Save the modem response */
		ctrl_ctx->response.status = (msg_complete ? 0 : -EIO);

		memcpy(&ctrl_ctx->response.params,
		       &params, sizeof(struct dlp_command_params));

		/* Command done, notify the sender */
		complete(&ch_ctx->rx.cmd_xfer_done);
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
				       DLP_CTRL_TX_PDU_SIZE,
				       1,
				       dlp_cmd,
				       dlp_ctrl_complete_tx_async,
				       dlp_ctrl_msg_destruct);

		if (!tx_msg) {
			CRITICAL("TX alloc failed");

			/* Delete the command */
			kfree(dlp_cmd);

			goto push_rx;
		}

		/* Copy the command data */
		memcpy(sg_virt(tx_msg->sgt.sgl),
		       &dlp_cmd->params, sizeof(struct dlp_command_params));

		/* Send the TX HSI msg */
		ret = dlp_ctrl_send_hsi_msg(ctrl_ctx, tx_msg, 1);
		if (ret) {
			CRITICAL("TX xfer failed ! (cmd:0x%X, ret:%d)",
				dlp_cmd->params.id, ret);

			/* Free the TX msg */
			dlp_pdu_free(tx_msg, msg->channel);

			/* Delete the command */
			kfree(dlp_cmd);
		}
	}

push_rx:
	/* Push the RX msg again for futur response */
	ret = dlp_ctrl_send_hsi_msg(ctrl_ctx, msg, 0);
	if (ret) {
		CRITICAL("RX push failed, ret:%d", ret);

		/* We have A BIG PROBLEM if the RX msg cant be  */
		/* pushed again in the controller ==>           */
		/* No response could be received (FIFO empty)   */

		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);
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
 * @interm_state: the intermidiate state (to be set when TX is OK)
 * @final_state: the final state (to be set when RX (response) is OK)
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
				unsigned char interm_state,
				unsigned char final_state,
				unsigned char param1,
				unsigned char param2, unsigned char param3)
{
	int ret = 0;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	struct dlp_command *dlp_cmd;
	struct hsi_msg *tx_msg = NULL;

	PROLOG("hsi_ch:%d, cmd:0x%X, resp:0x%X",
				ch_ctx->hsi_channel, id, response_id);

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
			       DLP_CTRL_TX_PDU_SIZE,
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
	ret = dlp_ctrl_send_hsi_msg(ctrl_ctx, tx_msg, 1);
	if (ret) {
		CRITICAL("Unable to send 0x%X cmd (ret:%d)",
			dlp_cmd->params.id, ret);

		goto free_tx;
	}

	/* Wait for TX msg to be sent */
	ret = wait_for_completion_timeout(&ch_ctx->tx.cmd_xfer_done,
					  msecs_to_jiffies(DLP_CMD_TX_TIMOUT));
	if (ret == 0) {
		CRITICAL("hsi_ch:%d, cmd:0x%X => TX timeout",
			dlp_cmd->params.channel, dlp_cmd->params.id);

		ret = -EIO;
		goto out;
	}

	/* TX msg sent, check the status */
	if (dlp_cmd->status) {
		CRITICAL("Failed to send cmd:0x%X", dlp_cmd->params.id);

		ret = -EIO;
		goto out;
	}

	/* TX OK */
   /* 1. Set the intermidiate channel state */
	if (interm_state != -1)
		dlp_ctrl_set_channel_state(ch_ctx, interm_state);

	/* Wait for response ? */
	if (response_id == DLP_CMD_NONE) {
		ret = 0;
		goto no_resp;
	}

	/* 2. Wait for the response */
	ret = wait_for_completion_timeout(&ch_ctx->rx.cmd_xfer_done,
					  msecs_to_jiffies(DLP_CMD_RX_TIMOUT));
	if (ret == 0) {
		CRITICAL("hsi_ch:%d, cmd:0x%X => RX timeout",
			dlp_cmd->params.channel, dlp_cmd->params.id);

		ret = -EIO;
		goto out;
	}

	/* Check the response */
	ret = 0;
	if ((ctrl_ctx->response.params.id != response_id) ||
	    (ctrl_ctx->response.params.data1 != param1) ||
	    (ctrl_ctx->response.params.data2 != param2) ||
	    (ctrl_ctx->response.params.data3 != param3)) {

		CRITICAL("cmd:0x%X unexpected response [0x%X%X%02X%02X%02X]"
			 " => expected [0x%X%X%02X%02X%02X]",
			 id,
			 ctrl_ctx->response.params.id,
			 ctrl_ctx->response.params.channel,
			 ctrl_ctx->response.params.data1,
			 ctrl_ctx->response.params.data2,
			 ctrl_ctx->response.params.data3,
			 response_id,
			 ch_ctx->hsi_channel, param1, param2, param3);

		ret = -EIO;
	}

no_resp:
	/* Response received & OK => set the new channel state */
	if (final_state != -1)
		dlp_ctrl_set_channel_state(ch_ctx, final_state);

	/* Restore RX callback */
	dlp_restore_rx_callbacks(&ctrl_ctx->start_rx_cb, &ctrl_ctx->stop_rx_cb);

	/* Everything is OK */
	EPILOG("%d", ret);
	return ret;

free_tx:
	/* Free the TX msg */
	dlp_pdu_free(tx_msg, tx_msg->channel);

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
			       DLP_CTRL_RX_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_rx, dlp_ctrl_msg_destruct);

	if (!rx_msg) {
		CRITICAL("dlp_pdu_alloc() failed");
		ret = -ENOMEM;
		goto free_cmd;
	}

	/* Send the RX HSI msg */
	ret = dlp_ctrl_send_hsi_msg(ch_ctx->ch_data, rx_msg, 0);
	if (ret) {
		CRITICAL("RX push failed, ret:%d", ret);
		ret = -EIO;
		goto free_msg;
	}

	EPILOG();
	return 0;

free_msg:
	/* Free the msg */
	dlp_pdu_free(rx_msg, rx_msg->channel);

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
* @param ready: Ready/Not ready
*/
static inline void
dlp_ctrl_set_modem_readiness(struct dlp_ctrl_context *ctrl_ctx,
							unsigned int ready)
{
	unsigned long flags;

	/* Update the flag */
	spin_lock_irqsave(&dlp_drv.lock, flags);
	dlp_drv.modem_ready = ready;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	/* Wakeup any waiting client for modem readiness */
	if (ready)
		wake_up(&ctrl_ctx->ready_event);
}

/*
* @brief Get the modem readiness (READY/NOT READY) flag
*
* @param ctrl_ctx: the control channel context
*
* @return 0 or 1 for modem ready/not ready
*/
static inline int
dlp_ctrl_get_modem_readiness(struct dlp_ctrl_context *ctrl_ctx)
{
	unsigned long flags;
	int ready;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	ready = dlp_drv.modem_ready;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	return ready;
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
	struct dlp_ctrl_context	*ctrl_ctx = DLP_CTRL_CTX;

	PROLOG();

	/* Reset the modem readiness state */
	dlp_ctrl_set_modem_readiness(ctrl_ctx, 0);

	/* Wait for RESET_OUT */
	wait_for_completion(&ctrl_ctx->reset_done);

	/* RESET_OUT received => Set the modem readiness flag */
	dlp_ctrl_set_modem_readiness(ctrl_ctx, 1);

	EPILOG();
}


/**
*  Push RX PDUs in the controller FIFO for modem requests
*
 * @ch_ctx: a reference to related channel context
*
* @return 0 when OK, an error otherwise
*/
static int dlp_ctrl_push_rx_pdus(struct dlp_channel *ch_ctx)
{
	int i, ret = 0;

	/* Push RX pdus for CREDTIS/OPEN_CONN/NOP commands */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++)
		dlp_ctrl_push_rx_pdu(ch_ctx);

	EPILOG();
	return ret;
}

/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

/**
 * Perform a modem cold boot/reset sequence:
 *
 *  - Set Reset_BB_N to 0
 *  - Delay 200us
 *  - Program CHIPCNTRL register 4th bit (MODEMRST)
 *  - Delay 20ms
 *  - Set Reset_BB_N to 1
 *  - Delay 60us
 *  - Do a pulse on ON1
 *
 * @ch_ctx: a reference to related channel context
 */
int dlp_ctrl_cold_boot(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx;
	int ret = 0;
	u16 addr = CHIPCNTRL;
	u8 data;

	PROLOG();

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		return 0;
	}

	pr_warn(DRVNAME ": Cold boot/reset request\n");

	/* AP request => just ignore the modem reset */
	ctrl_ctx = DLP_CTRL_CTX;
	dlp_ctrl_set_reset_ongoing(1);

	/* Set the RESET_BB_N to 0 */
	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	udelay(DLP_COLD_RST_DELAY);

	/* Write the 4th bit in the CHIPCTRL reg (MODEMRST) */
	data = CHIPCNTRL_MODEMRST;
	ret = intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("scu_ipc_write(MODEMRST) failed (ret: %d)", ret);
		goto out;
	}

	/* Wait before RESET_PWRDN_N to be 1 */
	msleep(DLP_COLD_REG_DELAY);

	/* Set the RESET_BB_N to 1 */
	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	udelay(DLP_ON1_DELAY);

	/* Do a pulse on ON1 */
	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 1);
	udelay(DLP_ON1_DURATION);
	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 0);

out:
	return ret;
	EPILOG();
}

/**
 * Perform a modem cold reset:
 *
 * Same sequence as cold boot
 *
 */
int dlp_ctrl_cold_reset(struct dlp_channel *ch_ctx)
{
	return dlp_ctrl_cold_boot(ch_ctx);
}

/**
 *  Perform a normal modem warm reset sequence:
 *
 *  - Do a pulse on the RESET_BB_N
 *
 * @ch_ctx: a reference to related channel context
 */
int dlp_ctrl_normal_warm_reset(struct dlp_channel *ch_ctx)
{
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;
	int ret = 0;

	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Warm reset ignored (protocol not registered)");
		return -ENODEV;
	}

	pr_info(DRVNAME ": Normal warm reset requested by ch%d",
			ch_ctx->hsi_channel);

	/* Clear the xfers debug list */
	DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx);

	/* AP requested reset => just ignore */
	dlp_ctrl_set_reset_ongoing(1);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	udelay(DLP_WARM_RST_DURATION);
	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);

	return ret;
}

/**
 *  Perform a normal modem warm reset sequence:
 *
 *  - Do a pulse on the RESET_BB_N
 *
 * @ch_ctx: a reference to related channel context
 */
int dlp_ctrl_flashing_warm_reset(struct dlp_channel *ch_ctx)
{
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;
	int ret = 0;

	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Flashing reset ignored (protocol not registered)");
		return -ENODEV;
	}

	pr_info(DRVNAME ": Flashing warm reset requested by ch%d",
			ch_ctx->hsi_channel);

	/* Clear the xfers debug list */
	DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx);

	/* AP requested reset => just ignore */
	dlp_ctrl_set_reset_ongoing(1);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	udelay(DLP_WARM_RST_DURATION);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	msleep(DLP_WARM_RST_FLASHING_DELAY);

	return ret;
}

/**
 *  Perform the modem switch OFF sequence:
 *
 *  - Set Reset_BB_N to 0
 *  - Program CHIPCNTRL register 4th bit (MODEMRST)
 *
 * @ch_ctx: a reference to related channel context
 */
static int dlp_ctrl_power_off(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx;
	u16 addr = CHIPCNTRL;
	u8 data;
	int ret = 0;

	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Power off ignored (protocol not registered)");
		return -ENODEV;
	}

	pr_info(DRVNAME ": Power OFF requested by ch%d", ch_ctx->hsi_channel);

	/* AP requested reset => just ignore */
	ctrl_ctx = DLP_CTRL_CTX;
	dlp_ctrl_set_reset_ongoing(1);

	/* Set the RESET_BB_N to 0 */
	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);

	/* Write the 4th bit in the CHIPCTRL reg (MODEMRST) */
	data = CHIPCNTRL_MODEMRST;
	ret = intel_scu_ipc_writev(&addr, &data, 1);
	if (ret)
		CRITICAL("scu_ipc_writev(MODEMRST) failed (ret: %d)", ret);

	return ret;
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
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	int ret, modem_ready = 1;

	/* Wait for modem_ready flag */
	ret = wait_event_timeout(ctrl_ctx->ready_event,
					dlp_ctrl_get_modem_readiness(ctrl_ctx),
					DLP_MODEM_READY_DELAY * HZ);
	if (ret == 0) {
		CRITICAL("Modem still not ready after %d sec !",
				DLP_MODEM_READY_DELAY);
		modem_ready = 0;
	}

	return modem_ready;
}

/*
* @brief Set the RESET ongoing flag value
*
*/
inline void dlp_ctrl_set_reset_ongoing(int ongoing)
{
	unsigned long flags;
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	ctrl_ctx->reset.ongoing = ongoing;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}

/*
* @brief Get the RESET ongoing flag value
*
*/
inline int dlp_ctrl_get_reset_ongoing(void)
{
	int reset_ongoing;
	unsigned long flags;
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	reset_ongoing = ctrl_ctx->reset.ongoing;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	return reset_ongoing;
}

/*
* @brief Get the Modem hangup reasons value
*
* Returns if the TTY interface has hang up and why it has hangup.
* For instance, a returned value of 5 is meaning that tty interface hang
* up because of both a TX timeout and a modem core dump.
*/
inline int dlp_ctrl_get_hangup_reasons(void)
{
	struct dlp_channel *ch_ctx;
	int reset, timeout, coredump, i, cause, hup_reasons;
	unsigned long flags;

	reset = 0;
	timeout = 0;
	coredump = 0;
	hup_reasons = 0;

	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = DLP_CHANNEL_CTX(i);
		if (!ch_ctx)
			continue;

		/* Check the hangup reasons for each channel */
		read_lock_irqsave(&ch_ctx->tx.lock, flags);
		cause = ch_ctx->hangup.cause;
		read_unlock_irqrestore(&ch_ctx->tx.lock, flags);
		if ((cause & DLP_MODEM_HU_TIMEOUT) == DLP_MODEM_HU_TIMEOUT)
			timeout++;
		if ((cause & DLP_MODEM_HU_RESET) == DLP_MODEM_HU_RESET)
			reset++;
		if ((cause & DLP_MODEM_HU_COREDUMP) == DLP_MODEM_HU_COREDUMP)
			coredump++;
	}

	if (reset)
		hup_reasons |= DLP_MODEM_HU_RESET;

	if (timeout)
		hup_reasons |= DLP_MODEM_HU_TIMEOUT;

	if (coredump)
		hup_reasons |= DLP_MODEM_HU_COREDUMP;

	return hup_reasons;
}

/*
* @brief  Clear the hangup reason the hangup reason if any
*
* @param hsi_channel: HSI channel to consider
* @param reason : hangup reason to set
*/
inline void dlp_ctrl_set_hangup_reasons(unsigned int hsi_channel, int reason)
{
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(hsi_channel);
	unsigned long flags;

	if (ch_ctx) {
		write_lock_irqsave(&ch_ctx->tx.lock, flags);
		/* Save the old hangup reason */
		ch_ctx->hangup.last_cause = ch_ctx->hangup.cause;

		/* Set the new reason */
		ch_ctx->hangup.cause = reason;
		write_unlock_irqrestore(&ch_ctx->tx.lock, flags);
	}
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

	/* Create a workqueue to to manage:
	 *	- Modem readiness
	 *	- HSI TX timeout
	 */
	ctrl_ctx->wq = create_singlethread_workqueue(DRVNAME "-ctrl_wq");
	if (!ctrl_ctx->wq) {
		CRITICAL("Unable to create control workqueue");
		goto free_ctx;
	}

	/* Save params */
	ch_ctx->ch_data = ctrl_ctx;
	ch_ctx->hsi_channel = index;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_completion(&ctrl_ctx->reset_done);
	init_waitqueue_head(&ctrl_ctx->ready_event);
	INIT_WORK(&ctrl_ctx->readiness_work, dlp_ctrl_ipc_readiness);
	INIT_WORK(&ctrl_ctx->hangup_work, dlp_ctrl_handle_hangup);
	INIT_WORK(&ctrl_ctx->tx_timeout_work, dlp_ctrl_handle_tx_timeout);

#ifdef DLP_CTRL_IO_DEBUG
	ch_ctx->dump_state = dlp_ctrl_xfers_logs_dump;

	INIT_LIST_HEAD(&ctrl_ctx->xfers_list);
	spin_lock_init(&ctrl_ctx->debug_lock);
#endif

	/* Register PDUs push CB */
	ch_ctx->push_rx_pdus = dlp_ctrl_push_rx_pdus;

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Configure GPIOs */
	ctrl_ctx->gpio_mdm_rst_out = pd->gpio_mdm_rst_out;
	ctrl_ctx->gpio_mdm_pwr_on  = pd->gpio_mdm_pwr_on;
	ctrl_ctx->gpio_mdm_rst_bbn = pd->gpio_mdm_rst_bbn;
	ctrl_ctx->gpio_fcdp_rb	   = pd->gpio_fcdp_rb;

	if (dlp_ctrl_setup_irq_gpio(ch_ctx, dev))
		goto free_ctx;

	/* Set ch_ctx, not yet done in the probe */
	DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL) = ch_ctx;

	dlp_ctrl_push_rx_pdus(ch_ctx);

	/* Reset & Wait for the modem readiness flag */
	queue_work(ctrl_ctx->wq, &ctrl_ctx->readiness_work);

	/* Modem cold boot sequence */
	dlp_ctrl_cold_boot(ch_ctx);

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
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;
	int ret = 0;

	PROLOG();

	/* Delete the modem readiness/tx timeout worqueue */
	destroy_workqueue(ctrl_ctx->wq);

	/* Free IRQs & GPIOs */
	free_irq(ctrl_ctx->reset.cd_irq, (void *)ch_ctx);
	free_irq(ctrl_ctx->reset.rst_irq, (void *)ch_ctx);

	gpio_free(ctrl_ctx->gpio_fcdp_rb);
	gpio_free(ctrl_ctx->gpio_mdm_rst_out);
	gpio_free(ctrl_ctx->gpio_mdm_pwr_on);
	gpio_free(ctrl_ctx->gpio_mdm_rst_bbn);

	/* Clear the xfers debug list */
	DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx);

	/* Free the CTRL context */
	kfree(ctrl_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);

	EPILOG();
	return ret;
}

/*
* @brief Open the specified channel for communication
*	- Send the OPEN_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_open_channel(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned char param1 = PARAM1(ch_ctx->tx.pdu_size);
	unsigned char param2 = PARAM2(ch_ctx->tx.pdu_size);

	PROLOG("hsi_ch:%d", ch_ctx->hsi_channel);

	/* Send the OPEN_CONN command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_OPEN_CONN, DLP_CMD_ACK,
				DLP_CH_STATE_OPENING, DLP_CH_STATE_OPENED,
				param1, param2, 0);

	EPILOG();
	return ret;
}

/*
* @brief Close the specified channel
*	- Send the CLOSE_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_close_channel(struct dlp_channel *ch_ctx)
{
	int state, ret = 0;
	unsigned long flags;
	unsigned char param3 = PARAM1(DLP_DIR_TRANSMIT_AND_RECEIVE);

	PROLOG("hsi_ch:%d", ch_ctx->hsi_channel);

	/* Reset the credits value */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	ch_ctx->credits = 0;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Check if the channel was correctly opened */
	state = dlp_ctrl_get_channel_state(ch_ctx);
	if (state == DLP_CH_STATE_OPENED) {
		/* Send the command */
		ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_CLOSE_CONN, DLP_CMD_ACK,
				DLP_CH_STATE_CLOSING, DLP_CH_STATE_CLOSED,
				0, 0, param3);
	} else {
		WARNING("Invalid channel%d state (%d)",
				ch_ctx->hsi_channel, state);
	}

	EPILOG();
	return ret;
}

/*
* @brief Send the NOP command
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_send_nop(struct dlp_channel *ch_ctx)
{
#if 0
	int ret;
	unsigned char param1, param2, param3;

	PROLOG("hsi_ch:%d", ch_ctx->hsi_channel);

	param1 = PARAM1(DLP_NOP_CMD_CHECKSUM);
	param2 = PARAM2(DLP_NOP_CMD_CHECKSUM);
	param3 = PARAM3(DLP_NOP_CMD_CHECKSUM);

	/* Send the NOP command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
			DLP_CMD_NOP, DLP_CMD_NONE,
			-1, -1,
			param1, param2, param3);

	EPILOG();
	return ret;
#endif
	return 0;
}

/*
* @brief Get the current channel state
*
* @param ch_ctx : The channel context to consider
*
* @return the current channel state
*/
inline unsigned char dlp_ctrl_get_channel_state(struct dlp_channel *ch_ctx)
{
	unsigned long flags;
	unsigned char state;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	state = ch_ctx->state;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	return state;
}

/*
* @brief Set the given channel state
*
* @param ch_ctx : The channel context to consider
* @param state : The new channel state to set
*
*/
inline void dlp_ctrl_set_channel_state(struct dlp_channel *ch_ctx,
				unsigned char state)
{
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	ch_ctx->state = state;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}

/****************************************************************************
 *
 * Hangup/Reset management
 *
 ***************************************************************************/
/*
* dlp_ctrl_hangup_ctx_init - Initialises the given hangup context
*
* @param ch_ctx : Channel context to consider
*/
void dlp_ctrl_hangup_ctx_init(struct dlp_channel *ch_ctx,
		void (*timeout_func)(struct dlp_channel *ch_ctx))
{
	PROLOG();

	/* Init values */
	ch_ctx->hangup.cause = 0;
	ch_ctx->hangup.last_cause = 0;
	ch_ctx->modem_tx_timeout_cb = timeout_func;

	/* Register the timer CB (Use always the CTRL context) */
	init_timer(&ch_ctx->hangup.timer);
	ch_ctx->hangup.timer.function = dlp_ctrl_hsi_tx_timout_cb;
	ch_ctx->hangup.timer.data = (unsigned long int)ch_ctx;

	EPILOG();
}

/**
 * dlp_ctrl_hangup_ctx_deinit - Clears a hangup context
 * @hangup_ctx: a reference to the considered hangup context
 */
void dlp_ctrl_hangup_ctx_deinit(struct dlp_channel *ch_ctx)
{
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->tx;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
	unsigned long flags;
	int is_hunging_up;

	PROLOG();

	write_lock_irqsave(&xfer_ctx->lock, flags);
	is_hunging_up = (ch_ctx->hangup.cause);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* No need to wait for the end of the calling work! */
	if (!is_hunging_up) {
		if (del_timer_sync(&ch_ctx->hangup.timer))
			cancel_work_sync(&ctrl_ctx->tx_timeout_work);
		else
			flush_work(&ctrl_ctx->tx_timeout_work);
	}

	EPILOG();
}

/*
* @brief Modem reset module_param set function
*	- This function is performing a modem normal warm reset
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_normal_reset(const char *val, struct kernel_param *kp)
{
	int do_reset;

	if (kstrtoint(val, 10, &do_reset) < 0)
		return -EINVAL;

	/* Need to do something ? */
	if (!do_reset)
		return -EINVAL;

	return dlp_ctrl_normal_warm_reset(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL));
}

/*
* @brief Modem reset module_param set function
*	- This function is performing a modem flashing warm reset
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_flashing_reset(const char *val, struct kernel_param *kp)
{
	int do_reset;

	if (kstrtoint(val, 10, &do_reset) < 0)
		return -EINVAL;

	/* Need to do something ? */
	if (!do_reset)
		return -EINVAL;

	return dlp_ctrl_flashing_warm_reset(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL));
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
	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Cant read reset value (protocol not registered)");
		return -ENODEV;
	}

	return sprintf(val, "%d", dlp_ctrl_get_reset_ongoing());
}

/*
* @brief Modem power OFF
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_power_off(const char *val, struct kernel_param *kp)
{
	int switch_off;

	if (kstrtoint(val, 10, &switch_off) < 0)
		return -EINVAL;

	/* Need to do something ? */
	if (!switch_off)
		return -EINVAL;

	return dlp_ctrl_power_off(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL));
}

/*
* @brief Modem cold reset module_param set function
*	- This function is performing a modem cold reset
*
* @param val
* @param kp
*
* @return 0
*/
static int do_modem_cold_reset(const char *val, struct kernel_param *kp)
{
	int do_reset;

	if (kstrtoint(val, 10, &do_reset) < 0)
		return -EINVAL;

	/* Need to do something ? */
	if (!do_reset)
		return -EINVAL;

	return dlp_ctrl_cold_reset(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL));
}

/*
* @brief This function will clear the hangup reasons for the
*  specified HSI channels (channel num is mapped on 4bits nibble)
*
*  for example to reset the HUP reasons for channel 0,1 and 3:
*     echo 0x1011 > /sys/module/../parameters/hangup_reasons
*
* @param val
* @param kp
*
* @return 0
*/
static int clear_hangup_reasons(const char *val, struct kernel_param *kp)
{
	long channels_list;

	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Cant clear hangup reason (protocol not registered)");
		return -ENODEV;
	}

	if (kstrtol(val, 16, &channels_list) < 0)
		return -EINVAL;

	if (channels_list) {
		int channel, i;

		for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
			channel = channels_list & 0xF;
			dlp_ctrl_set_hangup_reasons(i, 0);
			channels_list >>= 4;
		}
	}

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
	if (!dlp_ctrl_have_control_context()) {
		pr_warn(DRVNAME ": Cant read hangup reason (protocol not registered)");
		return -ENODEV;
	}

	return sprintf(val, "%d", dlp_ctrl_get_hangup_reasons());
}

module_param_call(power_off_modem, do_modem_power_off, NULL, NULL, 0644);
module_param_call(cold_reset_modem, do_modem_cold_reset, NULL, NULL, 0644);
module_param_call(reset_modem, do_modem_normal_reset, get_modem_reset,
		NULL, 0644);
module_param_call(flashing_reset_modem, do_modem_flashing_reset, NULL,
		NULL, 0644);
module_param_call(hangup_reasons, clear_hangup_reasons, get_hangup_reasons,
		NULL, 0644);

