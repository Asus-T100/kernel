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
#define DEBUG_VAR dlp_drv.debug

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
#define DLP_CMD_RX_TIMOUT	     5000  /* 5 sec */

#define DLP_ECHO_CMD_CHECKSUM	0xACAFFE
#define DLP_NOP_CMD_CHECKSUM	0xE7E7EC

#define DLP_CTRL_CMD_TO_STR(id) \
	(id == DLP_CMD_BREAK		? "BREAK" :	\
	(id == DLP_CMD_ECHO			? "ECHO" : \
	(id == DLP_CMD_NOP)			? "NOP" : \
	(id == DLP_CMD_CONF_CH)		? "CONF_CH" : \
	(id == DLP_CMD_OPEN_CONN)	? "OPEN_CONN" : \
	(id == DLP_CMD_CLOSE_CONN)  ? "CLOSE_CONN" : \
	(id == DLP_CMD_ACK)			? "ACK" : \
	(id == DLP_CMD_NACK)		? "NACK" : \
	(id == DLP_CMD_OPEN_CONN_OCTET)  ? "OPEN_CONN_OCTET" : \
	(id == DLP_CMD_CREDITS)		? "CREDITS" : \
	(id == DLP_CMD_NONE) ?		"NONE" : "UNKNOWN"))

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
#define V1P35CNT_W	0x0E0		/* PMIC reg to power off the modem */
#define V1P35_OFF	4
#define V1P35_ON	6

#define COLD_BOOT_DELAY_OFF	20000	/* 20 ms (use of usleep_range) */
#define COLD_BOOT_DELAY_ON	10000	/* 10 ms (use of usleep_range) */

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
 * @hangup_work: Modem TX timeout work
 * @readiness_work: Modem readiness work
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
	struct work_struct readiness_work;
	struct completion reset_done;

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
	int i;
	struct dlp_channel *ch_ctx = data;
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	PROLOG();

	CRITICAL("Modem CORE_DUMP 0x%x",
			gpio_get_value(ctrl_ctx->gpio_fcdp_rb));

	/* Call registered channels */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = DLP_CHANNEL_CTX(i);
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
		ch_ctx = DLP_CHANNEL_CTX(i);
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

/**
 * dlp_net_hangup_timer_cb - timer function for tx timeout
 * @param: a reference to the channel to consider
 *
 */
static void dlp_ctrl_hsi_tx_timout_cb(unsigned long int param)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)param;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

	PROLOG();

	/* Set the hangup reason for this channel */
	ch_ctx->hangup.cause |= DLP_MODEM_HU_TIMEOUT;

	/* Launch the work to handle the hangup */
	queue_work(ctrl_ctx->wq, &ctrl_ctx->hangup_work);

	EPILOG();
}

/**
 * dlp_ctrl_handle_tx_timeout - Manage the HSI TX timeout
 * @work: a reference to work queue element
 *
 * Required since port shutdown calls a mutex that might sleep
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

	PROLOG("hsi_ch:%d, cmd:%s, msg:0x%p",
	       dlp_cmd->params.channel,
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), msg);

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

	PROLOG("hsi_ch:%d, cmd:%s, msg:0x%p",
	       dlp_cmd->params.channel,
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), msg);

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

	PROLOG("cmd:%s, hsi_ch:%d, msg:0x%p",
	       DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id),
	       dlp_cmd->params.channel, msg);

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

	PROLOG("cmd:%s, hsi_ch:%d, params: 0x%02X%02X%02X, msg_status: %d",
	       DLP_CTRL_CMD_TO_STR(params.id),
	       params.channel,
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
			CRITICAL("TX xfer failed ! (%s, ret:%d)",
				DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id),
				ret);

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

	PROLOG("hsi_ch:%d, cmd:%s, resp:%s",
				ch_ctx->hsi_channel,
				DLP_CTRL_CMD_TO_STR(id),
				DLP_CTRL_CMD_TO_STR(response_id));

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
		CRITICAL("dlp_ctrl_send_hsi_msg(TX) failed ! (%s, ret:%d)",
			 DLP_CTRL_CMD_TO_STR(dlp_cmd->params.id), ret);

		goto free_tx;
	}

	/* Wait for TX msg to be sent */
	ret = wait_for_completion_timeout(&ch_ctx->tx.cmd_xfer_done,
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

		CRITICAL("%s unexpected response [0x%X%X%02X%02X%02X]"
			 " => expected [0x%X%X%02X%02X%02X]",
			 DLP_CTRL_CMD_TO_STR(id),
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
		CRITICAL("dlp_ctrl_send_hsi_msg() failed, ret:%d", ret);
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
	struct dlp_ctrl_context	*ctrl_ctx = DLP_CTRL_CTX;

	PROLOG();

	/* Set the modem readiness state */
	dlp_ctrl_set_modem_readiness(0);

	/* Wait for RESET_OUT */
	wait_for_completion(&ctrl_ctx->reset_done);

#if 0
	/* Even the ECHO cmd response is received the modem is not ready ! */
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	unsigned char param1, param2, param3;
	int ret = 0;

	/* Send ECHO continuously until the modem become ready */
	param1 = PARAM1(DLP_ECHO_CMD_CHECKSUM);
	param2 = PARAM2(DLP_ECHO_CMD_CHECKSUM);
	param3 = PARAM3(DLP_ECHO_CMD_CHECKSUM);

	do {
		ret = 0;
		/* Send the ECHO command */
		ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_ECHO, DLP_CMD_ECHO,
				-1, -1,
				param1, param2, param3);
		if (ret == 0) {
			/* Set the modem state */
			dlp_ctrl_set_modem_readiness(1);
		}
	} while (ret);
#else
	/* RESET_OUT received => The modem is ready */
	dlp_ctrl_set_modem_readiness(1);
#endif

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
 * @guard_time: Guard time after resetting the modem (IPC vs Flashing)
 *
 * Toggle gpios required to bring up modem power and start modem.
 * This can be called after the modem has been started to reset it.
 */
void dlp_ctrl_modem_reset(struct dlp_channel *ch_ctx, int guard_time)
{
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;

	PROLOG();

	WARNING("Modem reset requested !");

	/* Clear the xfers debug list */
	DLP_CTRL_XFERS_LIST_CLEAR(ctrl_ctx);

	/* AP requested reset => just ignore */
	ctrl_ctx->reset.ongoing = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);
	mdelay(DLP_DURATION_RST);

	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 1);
	msleep(guard_time);

	EPILOG();
}

/*
* @brief Toggle gpios required to bring up modem power and start modem
*
* @param ch_ctx : The channel context to consider
*/
void dlp_ctrl_modem_power(struct dlp_channel *ch_ctx)
{
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);
	struct dlp_ctrl_context	*ctrl_ctx = ctrl_ch->ch_data;

	PROLOG();

	WARNING("Modem power requested !");

	/* AP requested reset => just ignore */
	ctrl_ctx->reset.ongoing = 1;

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 1);
	udelay(DLP_DURATION_ON1);

	gpio_set_value(ctrl_ctx->gpio_mdm_pwr_on, 0);
	msleep(DLP_NORMAL_POST_DELAY);

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
	INIT_WORK(&ctrl_ctx->readiness_work, dlp_ctrl_ipc_readiness);
	INIT_WORK(&ctrl_ctx->hangup_work, dlp_ctrl_handle_tx_timeout);

#ifdef DLP_CTRL_IO_DEBUG
	ch_ctx->dump_state = dlp_ctrl_xfers_logs_dump;

	INIT_LIST_HEAD(&ctrl_ctx->xfers_list);
	spin_lock_init(&ctrl_ctx->debug_lock);
#endif

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Configure GPIOs */
	ctrl_ctx->gpio_mdm_rst_out = pd->gpio_mdm_rst_out;
	ctrl_ctx->gpio_mdm_pwr_on  = pd->gpio_mdm_pwr_on;
	ctrl_ctx->gpio_mdm_rst_bbn = pd->gpio_mdm_rst_bbn;
	ctrl_ctx->gpio_fcdp_rb	   = pd->gpio_fcdp_rb;

	ret = dlp_ctrl_setup_irq_gpio(ch_ctx, dev);
	if (ret)
		goto free_ctx;

	/* Set ch_ctx, not yet done in the probe */
	DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL) = ch_ctx;

	/* Push RX pdus for CREDTIS/OPEN_CONN commands */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++)
		dlp_ctrl_push_rx_pdu(ch_ctx);

	/* Start the modem readiness work */
	queue_work(ctrl_ctx->wq, &ctrl_ctx->readiness_work);

	/* Power on & Reset the modem */
	dlp_ctrl_modem_power(ch_ctx);
	dlp_ctrl_modem_reset(ch_ctx, DLP_NORMAL_POST_DELAY);

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

	/* Send the ECHO command */
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
			cancel_work_sync(&ctrl_ctx->hangup_work);
		else
			flush_work(&ctrl_ctx->hangup_work);
	}

	EPILOG();
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

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		EPILOG();
		return 0;
	}

	if (strict_strtol(val, 16, &do_reset) < 0) {
		EPILOG();
		return -EINVAL;
	}

	if (do_reset)
		dlp_ctrl_modem_reset(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL),
				DLP_NORMAL_POST_DELAY);

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
	int ret = 0;

	PROLOG();

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		EPILOG();
	} else {
		struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;
		ret = sprintf(val, "%d", ctrl_ctx->reset.ongoing);

		EPILOG("%d", ctrl_ctx->reset.ongoing);
	}

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
static int do_modem_power(const char *val, struct kernel_param *kp)
{
	long do_power;

	PROLOG();

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		EPILOG();
		return 0;
	}

	if (strict_strtol(val, 16, &do_power) < 0) {
		EPILOG();
		return -EINVAL;
	}

	if (do_power)
		dlp_ctrl_modem_power(DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL));

	EPILOG();
	return 0;
}

/*
 * Modem cold reset sysfs entries
 *
 * To do a modem cold reset we have to:
 * - Set the RESET_BB_N to low (better SIM protection)
 * - Set the EXT1P35VREN field to low  during 20ms (V1P35CNT_W PMIC register)
 * - set the EXT1P35VREN field to high during 10ms (V1P35CNT_W PMIC register)
 */
static int do_modem_cold_reset(const char *val, struct kernel_param *kp)
{
	struct dlp_ctrl_context *ctrl_ctx;
	long do_reset;
	int ret = 0;
	u16 addr = V1P35CNT_W;
	u8 data, def_value;

	PROLOG();

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		return 0;
	}

	if (strict_strtol(val, 10, &do_reset) < 0) {
		ret = -EINVAL;
		goto out;
	}

	WARNING("Modem cold reset requested !");

	/* Need to do something ? */
	if (!do_reset) {
		ret = -EINVAL;
		goto out;
	}

	/* AP requested reset => just ignore */
	ctrl_ctx = DLP_CTRL_CTX;
	ctrl_ctx->reset.ongoing = 1;

	/* Set the reset_bb to low */
	gpio_set_value(ctrl_ctx->gpio_mdm_rst_bbn, 0);

	/* Save the current registre value */
	ret = intel_scu_ipc_readv(&addr, &def_value, 2);
	if (ret) {
		CRITICAL("intel_scu_ipc_readv() failed (ret: %d)", ret);
		goto out;
	}

	/* Write the new registre value (V1P35_OFF) */
	data = (def_value & 0xf8) | V1P35_OFF;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev(OFF)  failed (ret: %d)", ret);
		goto out;
	}
	usleep_range(COLD_BOOT_DELAY_OFF, COLD_BOOT_DELAY_OFF);

	/* Write the new registre value (V1P35_ON) */
	data = (def_value & 0xf8) | V1P35_ON;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev(ON) failed (ret: %d)", ret);
		goto out;
	}
	usleep_range(COLD_BOOT_DELAY_ON, COLD_BOOT_DELAY_ON);

	/* Write back the saved registre value */
	ret =  intel_scu_ipc_writev(&addr, &def_value, 1);
	if (ret) {
		CRITICAL("intel_scu_ipc_writev() failed (ret: %d)", ret);
	} else {
		/* Perform a warm reset to finish the reset process */
		do_modem_reset(val, kp);
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

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		return 0;
	}

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

	if (!dlp_ctrl_have_control_context()) {
		WARNING("Nothing to do (dlp protocol not registered)");
		return 0;
	}

	hangup_reasons = dlp_ctrl_get_hangup_reasons();

	EPILOG();
	return sprintf(val, "%lud", hangup_reasons);
}

module_param_call(cold_reset_modem, do_modem_cold_reset, NULL, NULL, 0644);
module_param_call(reset_modem, do_modem_reset, get_modem_reset, NULL, 0644);
module_param_call(power_modem, do_modem_power, NULL, NULL, 0644);
module_param_call(hangup_reasons, clear_hangup_reasons, get_hangup_reasons,
		  NULL, 0644);

