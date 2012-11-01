/*
 * drivers/hid/ntrig_spi.c
 *
 * Copyright (c) 2011, N-Trig
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/spi/ntrig_spi.h>

#include "typedef-ntrig.h"
#include "ntrig-common.h"
#include "ntrig-dispatcher.h"
#include "ntrig_low_msg.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DRIVER_NAME	"ntrig_spi"


/** define this macro to put driver in "debug" mode - it will
 *  not	be loaded at startup, and need to be loaded by:
 *  echo 6 > /sys/spi_test/test
 */
/* #define NTRIG_SPI_DEBUG_MODE			1 */

/** the maximum time to	wait for a response when sending
 *  command to sensor. If sensor does not respond within this
 *  time host assumes it is either in boot loader or not working */
/* HZ/2 = 500ms */
#define MAX_SPI_COMMAND_REPLY_TIMEOUT		(HZ/2)

/** the maximum time to wait for a reply from raw(ncp/dfu) command.*/
/* 10 seconds */
#define MAX_SPI_RAW_COMMAND_REPLY_TIMEOUT	(HZ*10)

/** maximum reply size to a feature command (hid ncp) */
#define MAX_HID_NCP_REPLY_SIZE			16
/** size (bytes) of the circular buffer we keep for raw/ncp commands */
#define RAW_RECEIVE_BUFFER_INITIAL_SIZE		4096
#define RAW_RECEIVE_BUFFER_MAX_SIZE		(1024*120)
/** maximum size of SPI buffer for send/receive. The buffers
 *  will be allocated for this size.
 * 264 bytes for G4 (256 + 8 bytes preamble
 **/
#define MAX_SPI_TRANSFER_BUFFER_SIZE		264
/** maximum size of an aggregated logical SPI message,
 *  16 (max fragments) * 122 (max message size) */
#define MAX_LOGICAL_MESSAGE_SIZE		1952
/** spi watchdog timer interval in msec */
#define SPI_WATCHDOG_INTERVAL_MSEC		1000
/** spi watchdog timer max number of retries before power reset */
#define SPI_WATCHDOG_MAX_RETRIES		3

/** SPI message size for G3.x sensor (128 bytes + 8 bytes preamble) */
#define SPI_MESSAGE_SIZE_G3			136
/** SPI message size for G4 sensor (256 bytes + 8 bytes preamble */
#define SPI_MESSAGE_SIZE_G4			264
#define MAX_NUMBER_OF_FAIL			20

/** counters names **/
/* device to host */
#define CNTR_NAME_MULTITOUCH			"channel multitouch"
#define CNTR_NAME_SINGLETOUCH			"channel singletouch"
#define CNTR_NAME_PEN				"channel pen"
#define CNTR_NAME_CONTROL_REPLY			"channel control reply"
#define CNTR_NAME_MAINT_REPLY			"channel maint reply"
#define CNTR_NAME_DEBUG_REPLY			"channel debug reply"

/* host to device */
#define CNTR_NAME_CONTROL			"channel control"
#define CNTR_NAME_MAINT				"channel maint"
#define CNTR_NAME_DEBUG				"channel debug"

enum spi_cntrs_names {
	CNTR_MULTITOUCH = 0,
	CNTR_SINGLETOUCH,
	CNTR_PEN,
	CNTR_CONTROLREPLY,
	CNTR_MAINTREPLAY,
	CNTR_DEBUGREPLAY,
	CNTR_CONTROL,
	CNTR_MAINT,
	CNTR_DEBUG,
	CNTR_NUMBER_OF_SPI_CNTRS
};

/** driver private data */
struct ntrig_spi_privdata {
	/** pointer back to the spi device */
	struct spi_device	*spi;
	/** the sensor id assigned to us by dispatcher */
	int			sensor_id;
	/** for debugging: sysfs file for sending test commands */
	struct kobject		*test_kobj;
	/** gpio index for output_enable, copied from spi platform data */
	int			oe_gpio;
	/** true if output_enable line is connected to inverter,
	 *  copied from spi platform data */
	int			oe_inverted;
	/** gpio index for the irq line */
	unsigned		irq_gpio;
	/** flags to use for requesting interrupt handler */
	int			irq_flags;
	/** gpio index for power */
	int			pwr_gpio;
	/** for spi transfer */
	struct spi_message	msg;
	struct spi_transfer	xfer;
	uint8_t			*tx_buf;
	uint8_t			*rx_buf;
	/** the current configured SPI message size (136 bytes or 264
	 *  bytes). initialized to 136 bytes but we will switch to
	 *  264 bytes if we detect a G4 sensor */
	int			spi_msg_size;
	/** state machine for processing incoming data from SPI link */
	struct _ntrig_low_sm_info sm;
	/** semaphore (mutex) for protecting the spi transfer/state
	 *  machine access */
	struct semaphore	spi_lock;
	/** shared structure between this driver and dispatcher TODO
	 *  to be removed in new architecture */
	struct ntrig_bus_device	*ntrig_dispatcher;
	/** message for sending hid reports to dispatcher */
	struct mr_message_types_t report;
	/** last pen button code, for implementing pen event logic
	 *  (usbhid compatibility) */
	int			pen_last_btn;
	/** counter for single touch report, in order to emulate
	 * "frame index" */
	int			stReportCount;
	/** --------- WRITE/READ HID NCP(FEATURE) COMMAND ----------- */
	/* wait queue for waiting for command result after sending command */
	wait_queue_head_t	hid_ncp_wait_queue;
	/** true if we got result from the last ncp hid command */
	int			hid_ncp_got_result;
	/** amount of bytes received for ncp hid reply */
	int			hid_ncp_reply_size;
	/** buffer for storing the hid ncp reply */
	u8			hid_ncp_reply_buf[MAX_HID_NCP_REPLY_SIZE];
	/** --------- WRITE/READ BULK/RAW NCP COMMANDS -------- */
	/** circular buffer for storing incoming raw replies */
	struct kfifo		raw_fifo;
	/* wait queue for implementing read blocking (when raw_fifo is empty)*/
	wait_queue_head_t	raw_wait_queue;
	/** flag used when waiting for buffer to be filled */
	int			raw_data_arrived;
	/** scratch buffer for discarding packets (when circular
	 *  buffer is full) and for copying the old fifo to the
	 *  new one during fifo expansion */
	u8			scratch_buf[MAX_SPI_TRANSFER_BUFFER_SIZE];
	/** buffer for aggregating fragmented messages */
	u8			aggregation_buf[MAX_LOGICAL_MESSAGE_SIZE];
	/** size of aggregated message */
	int			aggregation_size;
	/** number of logical message fragments left for complete message */
	int			fragments_left;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

struct watchdog_timer_t {
	struct workqueue_struct *work_q;
	unsigned long interval;
	/* when true - watchdog is enabled
	 * (config by /sys/spi_watchdog/watchdog) */
	bool is_enabled;
	/* true when driver is suspended */
	bool is_suspended;
	/* set this to true to exit the watchdog timer*/
	bool is_exit;
	/* true if watchdog msg sent to the device */
	bool is_msg_sent;
	/* number of consecutive watchdog msgs sent */
	int msg_count;
	struct ntrig_spi_privdata *pdata;
	struct kobject *kobj;
	spinlock_t lock;
};

static struct watchdog_timer_t watchdog_timer = {
	.work_q = NULL,
	.interval = 0,
	.is_enabled = false,
	.is_suspended = false,
	.is_exit = false,
	.is_msg_sent = false,
	.msg_count = 0,
	.pdata = NULL,
	.kobj = NULL
};

static struct ntrig_counter spi_cntrs_list[CNTR_NUMBER_OF_SPI_CNTRS] = {
	{.name = CNTR_NAME_MULTITOUCH,		.count = 0},
	{.name = CNTR_NAME_SINGLETOUCH,		.count = 0},
	{.name = CNTR_NAME_PEN,			.count = 0},
	{.name = CNTR_NAME_CONTROL_REPLY,	.count = 0},
	{.name = CNTR_NAME_MAINT_REPLY,		.count = 0},
	{.name = CNTR_NAME_DEBUG_REPLY,		.count = 0},
	{.name = CNTR_NAME_CONTROL,		.count = 0},
	{.name = CNTR_NAME_MAINT,		.count = 0},
	{.name = CNTR_NAME_DEBUG,		.count = 0},
};

void spi_reset_counters(void)
{
	int i;
	for (i = 0; i < CNTR_NUMBER_OF_SPI_CNTRS; i++)
		spi_cntrs_list[i].count = 0;
}


static void watchdog_task_handler(struct work_struct *w);
DECLARE_DELAYED_WORK(watchdog_task, watchdog_task_handler);

static int spi_write_hid_ncp(void *dev, uint8_t cmd, const char *buf,
				short msg_len);
static void ntrig_spi_send_driver_alive(struct spi_device *spi);
static void spi_set_pwr(struct ntrig_spi_privdata *pdata, bool enable);

static void watchdog_task_handler(struct work_struct *w)
{
	if (watchdog_timer.is_enabled && !watchdog_timer.is_suspended) {
		/* sensor communication is OK */
		if (watchdog_timer.msg_count < SPI_WATCHDOG_MAX_RETRIES) {
			/* not enough communication with the sensor.
			 * Send a message to verify communication. */
			if (watchdog_timer.msg_count > 0) {
				spin_lock(&watchdog_timer.lock);
				watchdog_timer.is_msg_sent = true;
				spin_unlock(&watchdog_timer.lock);
				/* send the "Get FW Version" command */
				spi_write_hid_ncp(watchdog_timer.pdata->spi,
						LOWMSG_FUNCTION_GET_FW_VERSION,
						NULL, 0);
			}
			++(watchdog_timer.msg_count);
		} else {
			/* sensor is not OK, power off sensor */
			spi_set_pwr(watchdog_timer.pdata, false);
			/* power on sensor */
			spi_set_pwr(watchdog_timer.pdata, true);
		}
	}

	if (!watchdog_timer.is_exit) {
		unsigned long interval = watchdog_timer.interval;

		switch (watchdog_timer.msg_count) {
		case (SPI_WATCHDOG_MAX_RETRIES - 1):
			interval = watchdog_timer.interval / 4;
			break;
		case SPI_WATCHDOG_MAX_RETRIES:
			interval = watchdog_timer.interval / 8;
			break;
		default:
			break;
		}
		queue_delayed_work(watchdog_timer.work_q,
					&watchdog_task, interval);
	}
}

/** for debugging */
static struct spi_device *tmp_spi_dev;

/**
 * create the spi_transfer structure, that will allow us to
 * send/receive data over SPI
 */
static int setup_transfer(struct ntrig_spi_privdata *data)
{
	struct spi_message *m;
	struct spi_transfer *x;
	int len;

	len = MAX_SPI_TRANSFER_BUFFER_SIZE;
	ntrig_dbg("%s: enter. message size is %d\n", __func__, len);
	data->tx_buf = kmalloc(len, GFP_KERNEL);
	if (!data->tx_buf) {
		ntrig_dbg("%s: fail to allocate tx_buf\n", __func__);
		return -ENOMEM;
	}

	data->rx_buf = kmalloc(len, GFP_KERNEL);
	if (!data->rx_buf) {
		ntrig_dbg("%s: fail to allocate rx_buf\n", __func__);
		kfree(data->tx_buf);
		return -ENOMEM;
	}

	m = &data->msg;
	x = &data->xfer;

	spi_message_init(m);

	x->tx_buf = &data->tx_buf[0];
	x->rx_buf = &data->rx_buf[0];
	/* make sure you fill the length correctly before doing an SPI
	 * transfer, up to MAX_SPI_TRANSFER_BUFFER_SIZE
	 * x->len = sizeof(struct _ntrig_low_bus_msg); */
	spi_message_add_tail(x, m);

	/** initial message size - 136 bytes for G3.5 sensor. We will
	 * switch to 264 if detected a G4 sensor */
	data->spi_msg_size = SPI_MESSAGE_SIZE_G3;

	return 0;
}

/**
 * DEBUGGING
 * functions prototypes for sysfs debug file
 */
static ssize_t ntrig_spi_test_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t ntrig_spi_test_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static void ntrig_spi_test_release(struct kobject *kobj);

/**
 * enable or disable the output_enable line connected to our sensor.
 * When enabled, sensor can access the SPI bus
 * When disabled, sensor cannot access the SPI bus.
 * Note that if the output_enable line is connected to an
 * inverter, we reverse the values for enable/disable (high=disable, low=enable)
 * Assumes gpio line was prepared before (request, set direction)
 */
static void spi_set_output_enable(struct ntrig_spi_privdata *pdata, bool enable)
{
	if (pdata->oe_gpio >= 0) {
		int val;

		if (pdata->oe_inverted)
			val = enable ? 0 : 1;
		else
			val = enable ? 1 : 0;

		gpio_set_value(pdata->oe_gpio, val);
	}
}

/**
 * helper for managing the raw receive buffer.
 * Discards a message from the buffer
 * Must be called with spi_lock held
 */
static void spi_discard_raw_message(struct ntrig_spi_privdata *privdata)
{
	u16 size;
	u32 res;

	res = kfifo_out(&privdata->raw_fifo,
			(unsigned char *)&size, sizeof(u16));
	while (size > 0) {
		int getsize = size;
		if (getsize > MAX_SPI_TRANSFER_BUFFER_SIZE)
			getsize = MAX_SPI_TRANSFER_BUFFER_SIZE;
		res = kfifo_out(&privdata->raw_fifo,
				&privdata->scratch_buf[0], getsize);
		size -= getsize;
	}
}

/**
 * called to extend the raw fifo size when the first debug agent reply is
 * received. The debug agent requires a significantly larger fifo.
 */
static void spi_expand_raw_fifo(struct ntrig_spi_privdata *privdata)
{
	struct kfifo new_fifo;
	int buf_size, get_size;
	u32 res;
	int err = kfifo_alloc(&new_fifo,
				RAW_RECEIVE_BUFFER_MAX_SIZE, GFP_KERNEL);
	if (err) {
		ntrig_dbg("%s: fail to allocate a larger raw_fifo, err = %d\n",
				__func__, err);
		return;
	}
	/* copy the contents of the old fifo to the new one */
	buf_size = sizeof(privdata->scratch_buf);
	while ((get_size = kfifo_len(&privdata->raw_fifo)) > 0) {
		if (get_size > buf_size)
			get_size = buf_size;
		res = kfifo_out(&privdata->raw_fifo,
				&privdata->scratch_buf[0], get_size);
		kfifo_in(&new_fifo, &privdata->scratch_buf[0], get_size);
	}
	kfifo_free(&privdata->raw_fifo);
	privdata->raw_fifo = new_fifo;
}

/**
 * called when we have a complete message received from the spi layer
 */
static void spi_process_message(struct ntrig_spi_privdata *privdata)
{
	struct _ntrig_low_msg *msg = &privdata->sm.lmsg;
	struct mr_message_types_t *mr = &privdata->report;
	bool is_watchdog_reply = false;
	ntrig_dbg("%s: message type %d\n", __func__, msg->type);
	ntrig_dbg("%s: channel=%d function=%d\n", __func__, msg->channel,
			msg->function);
	watchdog_timer.msg_count = 0; /* reset the watchdog timer */
	if (msg->flags & 0x80) {
		/* bit 7 set in flags means a 256 byte packet arrived,
		 * this is a G4 sensor. Switch our SPI message size so
		 * next transfers will be more efficient. */
		privdata->spi_msg_size = SPI_MESSAGE_SIZE_G4;
	}
	switch (msg->channel) {
	case LOWMSG_CHANNEL_MULTITOUCH:
		if (msg->function == LOWMSG_FUNCTION_MT_REPORT) {
			/* fill in multi-touch report and send to dispatcher */
			u8 contactCount;
			struct _ntrig_low_mt_report *mtr =
				(struct _ntrig_low_mt_report *)&msg->data[0];
			struct _ntrig_low_mt_finger *fingers;
			int i;

			if (msg->flags & 0x80) {
				/* 256 byte report always sends 10 fingers */
				contactCount = mtr->g4fingers.contactCount;
				fingers = &mtr->g4fingers.fingers[0];
				if (contactCount > MAX_MT_FINGERS_G4) {
					ntrig_dbg("%s: invalid g4 mt report, too many fingers: %d\n",
						  __func__, contactCount);
					return;
				}
			} else {
				/* 128 byte report, 6 fingers (G3.x) */
				contactCount = mtr->g3fingers.contactCount;
				fingers = &mtr->g3fingers.fingers[0];
				if (contactCount > MAX_MT_FINGERS_G3) {
					ntrig_dbg("%s: invalid g3 mt report, too many fingers: %d\n",
						  __func__, contactCount);
					return;
				}
			}
			ntrig_dbg("%s: finger count=%d vendor defined = %u\n",
					__func__, contactCount,
					fingers[0].vendorDefined);
			mr->type = MSG_FINGER_PARSE;
			mr->msg.fingers_event.sensor_id = privdata->sensor_id;
			mr->msg.fingers_event.frame_index = mtr->reportCount;
			mr->msg.fingers_event.num_of_fingers = contactCount;
			for (i = 0; i < contactCount; i++) {
				u32 vendor = fingers[i].vendorDefined;
				mr->msg.fingers_event.finger_array[i].x_coord =
							fingers[i].x;
				mr->msg.fingers_event.finger_array[i].y_coord =
							fingers[i].y;
				mr->msg.fingers_event.finger_array[i].dx =
							fingers[i].dx;
				mr->msg.fingers_event.finger_array[i].dy =
							fingers[i].dy;
				mr->msg.fingers_event.finger_array[i].track_id =
							fingers[i].fingerIndex;
				/* vendor defined: first byte is "removed"
				 * (valid first occurance) third byte is
				 * "isPalm" */
				mr->msg.fingers_event.finger_array[i].removed =
							vendor & 0xFF;
				mr->msg.fingers_event.finger_array[i].generic =
							(vendor >> 8) & 0xFF;
				mr->msg.fingers_event.finger_array[i].palm =
							(vendor >> 16) & 0xFF;
			}
			spi_cntrs_list[CNTR_MULTITOUCH].count++;
			/* call the dispatcher to deliver the message */
			WriteHIDNTRIG(mr);
		} else {
			ntrig_dbg("%s: invalid mt report, function=%d\n",
						__func__, msg->function);
		}
		break;
	case LOWMSG_CHANNEL_PEN:
		if (msg->function == LOWMSG_FUNCTION_PEN_REPORT) {
			/* fill in pen report and send to dispatcher */
			int btn = 0;
			struct _ntrig_low_pen_report *pr =
				(struct _ntrig_low_pen_report *)&msg->data[0];
			mr->type = MSG_PEN_EVENTS;
			mr->msg.pen_event.sensor_id = privdata->sensor_id;
			mr->msg.pen_event.x_coord = pr->x;
			mr->msg.pen_event.y_coord = pr->y;
			mr->msg.pen_event.pressure = pr->pressure;
			mr->msg.pen_event.btn_code = pr->flags;
			mr->msg.pen_event.btn_removed = privdata->pen_last_btn;
			mr->msg.pen_event.battery_status = pr->battery_status;
			privdata->pen_last_btn = btn;
			spi_cntrs_list[CNTR_PEN].count++;
			/* call the dispatcher to deliver the message */
			WriteHIDNTRIG(mr);
		} else {
			ntrig_dbg("%s: invalid pen report, function=%d\n",
					__func__, msg->function);
		}
		break;
	case LOWMSG_CHANNEL_SINGLETOUCH:
		if (msg->function == LOWMSG_FUNCTION_ST_REPORT) {
			/* single touch report is actually sent to
			 * multi-touch channel. We feed it to tracklib as it
			 * involves fingers (the upper level may optimize
			 * and bypass tracklib) */
			struct _ntrig_low_st_report *str =
				(struct _ntrig_low_st_report *)&msg->data[0];
			/* use in_range bit for "touch confidence" */
			uint8_t valid1st = str->flags & 0x01;
			ntrig_dbg("%s: ST flags = %u\n",
					__func__, str->flags);
			mr->type = MSG_FINGER_PARSE;
			mr->msg.fingers_event.sensor_id = privdata->sensor_id;
			mr->msg.fingers_event.frame_index =
						privdata->stReportCount++;
			mr->msg.fingers_event.num_of_fingers = 1;
			mr->msg.fingers_event.finger_array[0].x_coord = str->x;
			mr->msg.fingers_event.finger_array[0].y_coord = str->y;
			mr->msg.fingers_event.finger_array[0].dx = str->dx;
			mr->msg.fingers_event.finger_array[0].dy = str->dy;
			mr->msg.fingers_event.finger_array[0].track_id = 0;
			mr->msg.fingers_event.finger_array[0].removed =
								     valid1st;
			/* emulate "blob_id" */
			mr->msg.fingers_event.finger_array[0].generic = 1;
			mr->msg.fingers_event.finger_array[0].palm = 0;
			spi_cntrs_list[CNTR_SINGLETOUCH].count++;
			/* call the dispatcher to deliver the message */
			WriteHIDNTRIG(mr);
		} else {
			ntrig_dbg("%s: invalid st report, function=%d\n",
						__func__, msg->function);
		}
		break;
	case LOWMSG_CHANNEL_CONTROL_REPLY:
		/* reply to a hid feature message ("hid ncp") */
		/* copy the payload (after function) to the hid_ncp_reply_buf
		 * and wake up anyone waiting for reply
		 * (usually spi_write_ncp_hid).
		 * Note that we assume we are inside the spi_lock */
		/*pr_err("Reply recieved. ID = 0x%x\n", msg->data[0]); */
		spin_lock(&watchdog_timer.lock);
		if (watchdog_timer.is_msg_sent && msg->data[0] ==
			LOWMSG_FUNCTION_GET_FW_VERSION) {
			/*pr_err("Get FW Version reply recieved."
					"Watchdog deactivated.\n");*/
			watchdog_timer.is_msg_sent = false;
			is_watchdog_reply = true;
		}
		spin_unlock(&watchdog_timer.lock);
		if (is_watchdog_reply)
			return;

		privdata->hid_ncp_reply_size = msg->length -
					offsetof(struct _ntrig_low_msg, data);
		ntrig_dbg("%s: got control reply, size=%d\n",
				__func__, privdata->hid_ncp_reply_size);
		if (privdata->hid_ncp_reply_size < 0 ||
		   privdata->hid_ncp_reply_size > MAX_HID_NCP_REPLY_SIZE) {
			ntrig_dbg("%s: got control reply, but it is too large(%d) and will be truncated\n",
				__func__, privdata->hid_ncp_reply_size);
			privdata->hid_ncp_reply_size = MAX_HID_NCP_REPLY_SIZE;
		}
		memcpy(&privdata->hid_ncp_reply_buf[0], &msg->data[0],
						privdata->hid_ncp_reply_size);
		privdata->hid_ncp_got_result = 1;
		wake_up(&privdata->hid_ncp_wait_queue);
		spi_cntrs_list[CNTR_CONTROLREPLY].count++;
		break;
	case LOWMSG_CHANNEL_DEBUG_REPLY:
	{
		/* reply to the debug agent - extend raw fifo if not already
		 * extended. we assume we are inside spi_lock */
		if (kfifo_size(&privdata->raw_fifo) <
				RAW_RECEIVE_BUFFER_MAX_SIZE) {
			spi_cntrs_list[CNTR_DEBUGREPLAY].count++;
			spi_expand_raw_fifo(privdata);
		}
	}
	/* fall through */
	case LOWMSG_CHANNEL_MAINT_REPLY:
	{
		/* reply to a raw/ncp message (mostly used for dfu) */
		/* copy the payload (after function) to a circular buffer where
		 * it can be retrieved later as a fifo. we assume we are inside
		 * spi_lock */
		u16 size = msg->length - offsetof(struct _ntrig_low_msg, data);
		u8 *data = &msg->data[0];
		int avail;
		ntrig_dbg("%s: received ncp reply, size=%d\n",
				__func__, size);
		if ((size + sizeof(u16)) > kfifo_size(&privdata->raw_fifo)) {
			ntrig_dbg("%s: packet too large to put in buffer (size=%d, max=%d), discarding\n",
					__func__,
					size, kfifo_size(&privdata->raw_fifo));
			break;
		}
		/* handle fragmented logical messages
		 * flags=number of fragments left */
		if (msg->channel == LOWMSG_CHANNEL_DEBUG_REPLY) {
			/* Ignore MSB, which indicates packet size */
			u8 flags = (msg->flags & ~0x80);
			/* logical message fragment */
			if ((flags > 0) || (privdata->fragments_left > 0)) {
				int message_error = 0;
				if ((privdata->fragments_left > 0) &&
				    (privdata->fragments_left != flags + 1)) {
					ntrig_err("%s: logical message corruption - fragments=%d, current=%d, discarding\n",
						__func__,
						privdata->fragments_left,
						flags);
					message_error = 1;
				} else if (privdata->aggregation_size +
					size > MAX_LOGICAL_MESSAGE_SIZE) {
					ntrig_err("%s: logical message too large to put in aggregation buffer (size=%d, max=%d), discarding\n",
							__func__,
						privdata->aggregation_size +
									size,
						MAX_LOGICAL_MESSAGE_SIZE);
					message_error = 1;
				}
				if (message_error) {
					/* discard logical message */
					privdata->aggregation_size = 0;
					privdata->fragments_left = 0;
					break;
				}
				memcpy(&privdata->aggregation_buf[
						privdata->aggregation_size],
							&msg->data[0], size);
				privdata->aggregation_size += size;
				privdata->fragments_left = flags;
				if (flags > 0)	{/* more fragments to come */
					ntrig_dbg("%s: fragmented logical message, waiting for complete message\n",
							__func__);
					break;
				}
				/* last fragment received */
				data = privdata->aggregation_buf;
				size = privdata->aggregation_size;
				privdata->aggregation_size = 0;
			}
		}
		/* if fifo can't hold our messages, discard messages
		 * from it until it has room */
		avail = kfifo_avail(&privdata->raw_fifo);
		if (avail < size) {
			ntrig_err("%s: raw receive buffer is full, discarding messages\n",
								__func__);
			do {
				spi_discard_raw_message(privdata);
				avail = kfifo_avail(&privdata->raw_fifo);
			} while (avail < size);
		}
		/* we got here, there is enough room in the buffer,
		 * insert the message */
		kfifo_in(&privdata->raw_fifo,
				(unsigned char *)&size, sizeof(u16));
		kfifo_in(&privdata->raw_fifo, data, size);

		/* wake up any threads blocked on the buffer */
		privdata->raw_data_arrived = 1;
		spi_cntrs_list[CNTR_MAINTREPLAY].count++;
		wake_up(&privdata->raw_wait_queue);
		break;
	}
	}
}

/**
 * execute a transfer over the SPI bus. Data will be transmitted
 * and received. Received data will be fed to the state machine.
 * Call spi_process_message for complete data packets
 * !!!MUST be called with spi_lock held!!!
 */
static int
execute_spi_bus_transfer(struct ntrig_spi_privdata *privdata, int len)
{
	int res, err;

	privdata->xfer.len = len;
	err = spi_sync(privdata->spi, &privdata->msg);

	if (err) {
		pr_err("%s: spi_sync failure, bailing out\n", __func__);
		return err;
	}

	set_low_msg_sm_data_packet(&privdata->sm,
			privdata->rx_buf, privdata->xfer.len);
	do {
		res = process_low_sm_data_packet(&privdata->sm);

		if (has_complete_low_message(&privdata->sm))
			spi_process_message(privdata);
	} while (res);

	return 0;
}


static int ntrig_get_irq_value(void)
{
	void __iomem *gafr;
	u32 gafr_value = 0;

	gafr = ioremap_nocache(0xff12c004, 0x4);
	if (gafr)
		gafr_value = readl(gafr);
	else
		pr_err("ntrig_get_irq_value(): ERROR, ioremap FAILED!\n");
	iounmap(gafr);

	return (gafr_value >> 30) & 0x01;
}

static int read_touchbuffer_to_empty(struct ntrig_spi_privdata *privdata)
{
	/** repeat until there is no more data */

	while (1) {
		int err, sm_idle, irq_high;

		/* critical section: spi transfer + state machine */
		down(&privdata->spi_lock);
		err = execute_spi_bus_transfer(privdata,
					privdata->spi_msg_size);
		if (err) {
			pr_err("%s: spi_transfer failure %d, bailing out\n",
							__func__, err);
			up(&privdata->spi_lock);
			break;
		}
		/* critial section end */
		up(&privdata->spi_lock);
		/* another transfer is needed if we're in the middle of a
		 * message (state machine not idle) or the irq is high */
		sm_idle = is_state_machine_idle(&privdata->sm);
		irq_high = ntrig_get_irq_value();

		/*pr_err("%s: state machine %s idle, gpio is %s\n",
		 * __func__, (sm_idle ? "is" : "not"),
		 * (irq_high ? "high" : "low"));*/
		if (sm_idle && (!irq_high))
			break;
	}

	return 0;
}

/**
 * interrupt handler, invoked when we have data waiting from
 * sensor
 * Note this function is registered as a threaded irq, so
 * executed in a separate thread and we can sleep here (the spi
 * transfers, for example, can sleep)
 */

static irqreturn_t spi_irq(int irq, void *dev_id)
{
	struct ntrig_spi_privdata *privdata = dev_id;
	int failCounter = 0;
#if 0
	if (!spi_get_output_enabled(privdata)) {
		/* output_enable is low, meaning the sensor will not be able
		 * to access the SPI bus, no point in trying any SPI transfer,
		 * so end here to avoid extra noise on the SPI bus.
		 * Wait a bit to avoid loading the CPU too much */
		msleep(100);
		return IRQ_HANDLED;
	}
#endif


	/** repeat until there is no more data */
	while (1) {
		int err, sm_idle, irq_high;

		/* critical section: spi transfer + state machine */
		down(&privdata->spi_lock);
		err = execute_spi_bus_transfer(privdata,
						privdata->spi_msg_size);
		if (err) {
			pr_err("%s: spi_transfer failure %d, bailing out\n",
							__func__, err);

			if (ntrig_get_irq_value() == 0) {
				pr_err("ntrig irq = %d\n",
						ntrig_get_irq_value());
				up(&privdata->spi_lock);
				break;
			} else {
				pr_err("ntrig irq = %d\n",
						ntrig_get_irq_value());
				failCounter++;

				if (failCounter > MAX_NUMBER_OF_FAIL) {
					int err;
					failCounter = 0;
					pr_err("reset to ntrig\n");

					err = gpio_direction_output(176, 0);
					if (err)
						pr_err("Setting gpio 176 to 0 failed\n");
					msleep(500);
					err = gpio_direction_output(176, 1);
					if (err)
						pr_err("Setting gpio 176 to 1 failed\n");
				}
			}
		}
		/* critial section end */
		up(&privdata->spi_lock);
		/* another transfer is needed if we're in the middle of a
		 * message (state machine not idle) or the irq is high */
		sm_idle = is_state_machine_idle(&privdata->sm);
#if 0
		irq_high = gpio_get_value(privdata->irq_gpio);
#else
		irq_high = ntrig_get_irq_value();
#endif
		/*pr_info("%s: state machine %s idle, gpio is %s\n",
		 *	__func__,(sm_idle ? "is" : "not"),
		 *	(irq_high ? "high" : "low"));*/
		if (sm_idle && (!irq_high))
			break;
	}

	return IRQ_HANDLED;
}

/**
 * Write NCP msg to HID device.
 * buffer contains at least one byte - the command code. msg_len
 * is the length including the command (currently 1 because
 * payout is always empty) if success return number of bytes
 * written >0 if failed returns <0 We emulate usbhid driver
 * behavior, which waits to response after writing. Response is
 * always 32 bit. We do not wait more than 500ms - according to
 * spec if it takes longer to answer, it means firmware is in
 * boot loader or damaged
 */
static int
spi_write_hid_ncp(void *dev, uint8_t cmd, const char *buf, short msg_len)
{
	struct spi_device *spi = dev;
	struct _ntrig_low_bus_msg *txbuf;
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *)spi_get_drvdata(spi);
	int err;

	/* ntrig_dbg("%s: msg_len=%d\n", __func__, msg_len); */
	if (msg_len != 0) {
		ntrig_dbg("%s: non-empty payload not supported\n", __func__);
		return msg_len;
	}

	/* critical section: spi bus and state machine */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}

	switch (cmd) {
	case LOWMSG_FUNCTION_DRIVER_ALIVE:
	case LOWMSG_FUNCTION_START_CALIBRATION:
	case LOWMSG_FUNCTION_GET_MODE:
	case LOWMSG_FUNCTION_SET_MODE_PEN:
	case LOWMSG_FUNCTION_SET_MODE_TOUCH:
	case LOWMSG_FUNCTION_SET_MODE_DUAL:
	case LOWMSG_FUNCTION_GET_CALIBRATION_STATUS:
	case LOWMSG_FUNCTION_GET_FW_VERSION:
	case LOWMSG_FUNCTION_SET_FEATURE_SET_MODE_DISABLE:
	case LOWMSG_FUNCTION_SET_FEATURE_SET_MODE_AUTO:
	case LOWMSG_FUNCTION_GET_FEATURE_GET_PEN_BITS:
		txbuf = (struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		build_low_bus_msg_short_cmd(cmd, txbuf);
		/** clear the "got reply" flag before sending the command. We
		 *  do it inside the spi lock so it	is done	before any reply
		 *  arrives */
		privdata->hid_ncp_got_result = 0;
		switch (txbuf->msg.channel) {
		case LOWMSG_CHANNEL_CONTROL:
			spi_cntrs_list[CNTR_CONTROL].count++;
			break;
		case LOWMSG_CHANNEL_DEBUG:
			spi_cntrs_list[CNTR_DEBUG].count++;
			break;
		case LOWMSG_CHANNEL_MAINT:
			spi_cntrs_list[CNTR_MAINT].count++;
			break;
		}
		err = execute_spi_bus_transfer(privdata,
						privdata->spi_msg_size);
		if (err) {
			ntrig_dbg("%s: spi transfer failure %d\n",
							__func__, err);
			goto exit_err;
		}
		/* clear the txbuf so we don't send this command again*/
		memset(txbuf, 0xFF, sizeof(struct _ntrig_low_bus_msg));
		break;
	default:
		ntrig_dbg("%s: unsupported command %d\n", __func__, cmd);
		err = -1;
		goto exit_err;
	}

	/* normal finish */
	up(&privdata->spi_lock);

	/* wait for reply. This is done without a lock since we can only have a
	 * single command/reply at any given time and we don't accumulate
	 * the reply results */
	err = wait_event_interruptible_timeout(privdata->hid_ncp_wait_queue,
					(privdata->hid_ncp_got_result != 0),
					MAX_SPI_COMMAND_REPLY_TIMEOUT);
	if (err < 0) {
		/* we were interrupted */
		return -ERESTARTSYS;
	}
	/* done */
	return msg_len;
exit_err:
	up(&privdata->spi_lock);
	return err;
}

/**
 * Read NCP msg from HID device
 * if success return number of bytes written and fill buffer.
 * If failed return <0
 * Allocate at least MAX_SPI_RESPONSE_SIZE bytes in the buffer
 * buf is kernel memory
 */
static int spi_read_hid_ncp(void *dev, char *buf, size_t count)
{
	struct spi_device *spi = dev;
	struct ntrig_spi_privdata *privdata =
			(struct ntrig_spi_privdata *)spi_get_drvdata(spi);
	int ret, err;

	/* use spi_lock to protect the hid_ncp variables */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}

	if (privdata->hid_ncp_got_result == 0) {
		/* no reply was received */
		up(&privdata->spi_lock);
		return 0;
	}

	ret = privdata->hid_ncp_reply_size;
	if (ret > MAX_HID_NCP_REPLY_SIZE) {
		ntrig_dbg("%s: reply received is too large (%d)\n",
							__func__, ret);
		ret = MAX_HID_NCP_REPLY_SIZE;
	}
	if (count < ret) {
		ntrig_dbg("%s: supplied buffer is too small (%d, need %d)\n",
						__func__, count, ret);
		ret = count;
	}
	memcpy(buf, &privdata->hid_ncp_reply_buf, ret);
	up(&privdata->spi_lock);
	return ret;
}

/**
 * Write NCP msg to RAW device (on SPI it is the same as HID
 * device though we may use a different channel in the SPI
 * message). if success return number of bytes written >0 if
 * failed returns <0
 * the function returns immediately and does not wait for a
 * reply. Replies are buffered and obtained using spi_read_raw,
 * which blocks if there are no replies waiting.
 */
static int spi_write_raw(void *dev, const char *buf, short msg_len)
{
	struct spi_device *spi = dev;
	struct _ntrig_low_bus_msg *txbuf;
	struct ntrig_spi_privdata *privdata =
			(struct ntrig_spi_privdata *)spi_get_drvdata(spi);
	u8 request;
	int err, len;

	if (msg_len <= 0) {
		ntrig_dbg("%s: empty message\n", __func__);
		return 0;
	}

	request = *buf;
	ntrig_dbg("%s: request=%d, msg_len=%d\n",
					__func__, request, msg_len);

	/* critical section: spi bus and state machine */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}

	switch (request) {
	case SPI_ENABLED_COMMAND:
	case LOWMSG_REQUEST_DEBUG_AGENT:
	case LOWMSG_REQUEST_GO_TO_BOOTLOADER:
	case LOWMSG_REQUEST_NCP_DFU:
		txbuf = (struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		build_ncp_dfu_cmd(request, buf, msg_len, (char *)txbuf);
		switch (txbuf->msg.channel) {
		case LOWMSG_CHANNEL_CONTROL:
			spi_cntrs_list[CNTR_CONTROL].count++;
			break;
		case LOWMSG_CHANNEL_DEBUG:
			spi_cntrs_list[CNTR_DEBUG].count++;
			break;
		case LOWMSG_CHANNEL_MAINT:
			spi_cntrs_list[CNTR_MAINT].count++;
			break;
		}
		len = MAX_SPI_TRANSFER_BUFFER_SIZE;
		err = execute_spi_bus_transfer(privdata, len);
		if (err) {
			ntrig_dbg("%s: spi transfer failure %d\n",
							__func__, err);
			goto exit_err;
		}
		if (request == LOWMSG_REQUEST_GO_TO_BOOTLOADER) {
			/* when executing go to bootloader, sensor will be
			 * reset. We must keep the output_enable low
			 * for a while to prevent the sensor going crazy */
			ntrig_dbg("%s: go to bootloader, lowering output_enable for a while...\n",
								__func__);
			spi_set_output_enable(privdata, 0);
			msleep(500);
			spi_set_output_enable(privdata, 1);
			ntrig_dbg("%s: go to bootloader, output_enable is back up\n",
								__func__);
		}
		/* clear the txbuf so we don't send this command again*/
		memset(txbuf, 0xFF, len);
		break;
	default:
		ntrig_dbg("%s: unsupported command %d\n", __func__, request);
		err = -1;
		goto exit_err;
	}

	/* normal finish */
	up(&privdata->spi_lock);

	/* done */
	return msg_len;
exit_err:
	up(&privdata->spi_lock);
	return err;
}

/**
 * helper for spi_read_raw (below)
 * reads packet from fifo and store it in the buffer.
 * Packets are stored as 2 bytes length, followed by <length>
 * bytes for the message
 * Return the number of bytes copied to the buffer, or 0 if the
 * fifo was empty
 * No locking is performed in this function, it is caller
 * responsibility to protect the fifo if needed
 */
static int
spi_get_raw_data_from_fifo(struct kfifo *fifo, char *buf, size_t count)
{
	u16 data_len;
	u32 res;

	if (kfifo_len(fifo)) {
		/* we have data, can return immediately */
		res = kfifo_out(fifo, (unsigned char *)&data_len, sizeof(u16));
		ntrig_dbg("%s: raw message size %d, count = %d\n",
				__func__, data_len, count);
		if (data_len > count)
			data_len = count;
		res = kfifo_out(fifo, buf, data_len);
	} else {
		data_len = 0;
	}
	return data_len;
}

/**
 * Read NCP msg from RAW device.
 * On SPI, all responses come from the same device. We separate
 * them into HID and RAW based on some fields in the message
 * if success return number of bytes read and fill buffer. If
 * failed return <0 Allocate at least MAX_SPI_RESPONSE_SIZE
 * bytes in the buffer
 * If there is no data in the buffer, it will block until data
 * is received, or until timeout is reached (500ms). Return -1
 * if no data arrived
 * Note: buf is kernel memory
 */
static int spi_read_raw(void *dev, char *buf, size_t count)
{
	struct spi_device *spi = dev;
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *)spi_get_drvdata(spi);
	int err, data_len;

	/* use spi_lock to protect the circular buffer */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}


	data_len = spi_get_raw_data_from_fifo(&privdata->raw_fifo, buf, count);
	if (data_len > 0) {
		/* we got data, return immediately */
		up(&privdata->spi_lock);
		return data_len;
	}

	/* buffer is empty, we will need to wait.
	 * Release the lock before waiting */
	privdata->raw_data_arrived = 0;
	up(&privdata->spi_lock);
	err = wait_event_interruptible_timeout(privdata->raw_wait_queue,
					(privdata->raw_data_arrived != 0),
					MAX_SPI_RAW_COMMAND_REPLY_TIMEOUT);
	if (err < 0) {
		/* we were interrupted */
		return -ERESTARTSYS;
	}

	/* get the lock again */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}

	/* read from fifo again, this time return 0
	 * if there is no data (timeout) */
	data_len = spi_get_raw_data_from_fifo(&privdata->raw_fifo, buf, count);
	up(&privdata->spi_lock);
	return data_len;
}

/* return the array of struct ntrig_counter and it's length */
int get_counters(struct ntrig_counter **counters_list_local, int *length)
{
	*counters_list_local = spi_cntrs_list;
	*length = CNTR_NUMBER_OF_SPI_CNTRS;
	return 0;
}

/* registers the device to the dispatcher driver */
static int register_to_dispatcher(struct spi_device *spi)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *)spi_get_drvdata(tmp_spi_dev);
	struct ntrig_bus_device *nd;
	struct ntrig_dev_ncp_func ncp_func;
	struct ntrig_dev_hid_func hid_func;
	int ret, flags;

	if (DTRG_NO_ERROR != allocate_device(&privdata->ntrig_dispatcher)) {
		dev_err(&spi->dev, "cannot allocate N-Trig dispatcher\n");
		return DTRG_FAILED;
	}

	/* register device in the dispatcher. We register twice
	 * once for HID and once for RAW (ncp/bulk)
	 * TODO we use a hard-coded bus name of "spi", need to change
	 * if we want to support multiple sensors connected over SPI */
	ncp_func.dev = spi;
	ncp_func.read = spi_read_raw;
	ncp_func.write = spi_write_raw;
	ncp_func.read_counters = get_counters;
	ncp_func.reset_counters = spi_reset_counters;

	hid_func.dev = spi;
	hid_func.read = spi_read_hid_ncp;
	hid_func.write = spi_write_hid_ncp;

	privdata->sensor_id = RegNtrigDispatcher(TYPE_BUS_SPI_HID, "spi",
			&ncp_func, &hid_func);
	if (privdata->sensor_id == DTRG_FAILED) {
		ntrig_dbg("%s: Cannot register device to dispatcher\n",
				__func__);
		return DTRG_FAILED;
	}
	ret = RegNtrigDispatcher(TYPE_BUS_SPI_RAW, "spi", &ncp_func, NULL);
	if (ret == DTRG_FAILED) {
		ntrig_dbg("%s: cannot register raw device to dispatcher\n",
				__func__);
		return DTRG_FAILED;
	}

	/** fill some default values for sensor area
	 *  TODO should be retrieved from sensor, currently not
	 *  supported in SPI */
	nd = privdata->ntrig_dispatcher;
	nd->logical_min_x = 0;
	nd->logical_max_x = 9600;
	nd->logical_min_y = 0;
	nd->logical_max_y = 7200;
	nd->pressure_min = 1;
	nd->pressure_max = 255;
	nd->is_touch_set = 1;
	nd->touch_width = 2;
	create_single_touch(nd, privdata->sensor_id);
	create_multi_touch(nd, privdata->sensor_id);

	/** register to	receive	interrupts when	sensor has data */
	flags = privdata->irq_flags;
	if (flags == 0) {
		/* default flags */
		flags = IRQF_TRIGGER_RISING | IRQF_SHARED;
	}
	/* get the irq */
	ntrig_dbg("%s: requesting irq %d\n", __func__, spi->irq);
	ret = request_threaded_irq(spi->irq, NULL, spi_irq,
			flags, DRIVER_NAME, privdata);

	if (ret) {
		dev_err(&spi->dev, "%s: request_irq(%d) failed\n",
				__func__, privdata->irq_gpio);
		return ret;
	}
	pr_info("ntrig pin:%d state:%d\n",
			privdata->irq_gpio, ntrig_get_irq_value());

	ntrig_dbg("End of %s\n", __func__);
	return DTRG_NO_ERROR;
}

/**
 * enable or disable the power line connected to our
 * sensor.
 * When enabled, sensor power is up .
 * When disabled, sensor power is down.
 * Assumes gpio line was prepared before (request, set
 * direction)
 */
static void spi_set_pwr(struct ntrig_spi_privdata *pdata, bool enable)
{
	if (pdata->pwr_gpio >= 0) {/* power gpio is present */
		disable_irq(pdata->spi->irq);
		if (enable) {
			gpio_set_value(pdata->pwr_gpio, 1);
			msleep(500);
			enable_irq(pdata->spi->irq);
			spi_set_output_enable(pdata, enable);
			ntrig_spi_send_driver_alive(pdata->spi);
		} else {
			gpio_set_value(pdata->pwr_gpio, 0);
			spi_set_output_enable(pdata, enable);
			enable_irq(pdata->spi->irq);
		}
	}
}

static int init_spi_pwr_gpio(struct ntrig_spi_privdata *pdata)
{
	if (pdata->pwr_gpio >= 0) {/* power gpio is present */
		/* set the pwr gpio line to turn on the sensor */
		int pwr_gpio = pdata->pwr_gpio;
		int err = gpio_request(pwr_gpio, "ntrig_spi_pwr");
		if (err) {
			ntrig_dbg("%s: fail to request gpio for pwr(%d), err=%d\n",
					__func__, pwr_gpio, err);
			/* continue anyway... */
		}
		err = gpio_direction_output(pwr_gpio, 0); /* low */
		if (err) {
			ntrig_dbg("%s: fail to change pwr\n", __func__);
			return err;
		}
		msleep(50);
		gpio_set_value(pwr_gpio, 1); /* high */
		msleep(50);
	}
	return 0;
}

/**
 * /sys/spi_watchdog/watchdog file definitions
 * Used to control the watchdog timer.
 */

/* callbacks for the /sys/spi_watchdog/watchdog file */
static ssize_t ntrig_spi_watchdog_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t ntrig_spi_watchdog_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static void ntrig_spi_watchdog_release(struct kobject *kobj);

/*sysfs data structures for the /sys/spi_watchdog/watchdog file */

/*define the kobj attributes: name, mode, show_function, store_function */
static struct kobj_attribute ntrig_spi_watchdog_attr = {
	.attr = {
		.name = __stringify(""),
		.mode = S_IRUGO | S_IWUGO
	},
	.show	= ntrig_spi_watchdog_show,
	.store	= ntrig_spi_watchdog_store,
};

const struct attribute *ntrig_spi_watchdog_attrs = {
	&ntrig_spi_watchdog_attr.attr
};

/*sysfs functions for the /sys/spi_watchdog/watchdog file */

/* writes (stores) data in the sysfs file (user triggered) */
static ssize_t ntrig_spi_watchdog_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	/* data is not written into actual file on file system,
	 * but rather saves it in a memory buffer */
	ntrig_dbg("inside %s\n", __func__);

	if (count > 0)
		watchdog_timer.is_enabled = buf[0] - '0'; /* cast to int */

	return count;
}

/* reads (shows) data from the sysfs file (user triggered) */
static ssize_t ntrig_spi_watchdog_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	ntrig_dbg("inside %s\n", __func__);
	buf[0] = watchdog_timer.is_enabled + '0'; /* cast to char */
	buf[1] = 0; /* end of string */

	return 1; /* number of written chars */
	/*return snprintf(buf, 2, "%d" , watchdog_timer.is_enabled); */
}

/*
 * The release function for our object. This is REQUIRED by the kernel to
 * have. We free the memory held in our object here.
 */
static void ntrig_spi_watchdog_release(struct kobject *kobj)
{
	ntrig_dbg("inside %s\n", __func__);

	if (kobj == NULL) {
		ntrig_dbg("inside %s , kobj==NULL\n", __func__);
		return;
	}

	if (ntrig_spi_watchdog_attr.attr.name != NULL) {
		sysfs_remove_file(kobj, ntrig_spi_watchdog_attrs);
		ntrig_dbg("inside %s After call to sysfs_remove_file\n",
				__func__);
	} else
		ntrig_dbg("inside %s Skip call to sysfs_remove_file\n",
				__func__);
	kobject_put(kobj);
	ntrig_dbg("inside %s After call to kobject_put\n", __func__);

	/* kobj is pointer to properties_kobj which was allocated in
	 * ntrig_create_virtualkeys_file */
	ntrig_dbg("inside %s After call to kfree(kobj)\n", __func__);
}

/*
 * This function creates folder "spi_test" under sysfs,
 * and creates file "test" under this folder
 */
#define NTRIG_SPI_WATCHDOG_DIRECTORY	"spi_watchdog"
#define NTRIG_SPI_WATCHDOG_FILE_NAME	"watchdog"
static struct kobject *ntrig_create_spi_watchdog_file(void)
{
	int retval;
	int len;
	char *attr_name = NULL;
	struct kobject *properties_kobj;

	ntrig_dbg("inside %s\n", __func__);

	/* 0. Preparations - to deal with case of multiple calls to
	 * this function, with same ntrig dev name generate the file (attribute)
	 * name, from user data */
	len = strlen(NTRIG_SPI_WATCHDOG_FILE_NAME);
	attr_name = kmalloc((len)+1, GFP_KERNEL);
	sprintf(attr_name, NTRIG_SPI_WATCHDOG_FILE_NAME);
	ntrig_spi_watchdog_attr.attr.name = attr_name;
	ntrig_dbg("inside %s creating new attr_name: %s\n",
			__func__, ntrig_spi_watchdog_attr.attr.name);


	/* 1. Create folder "spi_watchdog" under "sys" */
	/* allocate the memory for the whole object */
	properties_kobj = kobject_create_and_add(NTRIG_SPI_WATCHDOG_DIRECTORY,
			NULL);
	ntrig_dbg("inside %s\n", __func__);

	if (!properties_kobj) {
		kobject_put(properties_kobj);
		kfree(properties_kobj);
		kfree(attr_name);
		pr_err("failed to create spi_test\n");
		ntrig_dbg("inside %s - kobject_create_and_add FAILED\n",
				__func__);
		return NULL;
	}

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.  */
	/*kobject_uevent(properties_kobj, KOBJ_ADD); */

	/* 2. create file "watchdog" under folder "spi_watchdog"  */
	retval = sysfs_create_file(properties_kobj, ntrig_spi_watchdog_attrs);

	if (retval != 0) {
		pr_err("failed to create spi_watchdog/watchdog\n");
		/* TODO: remove file ?? */
		ntrig_dbg("inside %s - sysfs_create_file FAILED\n",
				__func__);

		/* remove properties_kobj */
		kobject_put(properties_kobj);
		kfree(properties_kobj);
		kfree(attr_name);
		return NULL;
	}

	return properties_kobj;
}



/**
 * sysfs data structures
 * USED FOR DEBUGGING ONLY. We create a /sys/spi_test/test file,
 * and use it to control the driver.
 */

/* define the kobj attributes: name, mode, show_function, store_function */
static struct kobj_attribute ntrig_spi_test_attr =
		__ATTR(NULL, S_IRUGO | S_IWUGO,
				ntrig_spi_test_show,
				ntrig_spi_test_store);

const struct attribute *ntrig_spi_test_attrs = {
	&ntrig_spi_test_attr.attr
};

/* sysfs functions */
static void debug_print_msg(struct _ntrig_low_bus_msg *msg)
{
	const char *buf = (const char *)msg;
	int i, offset = 0;

	for (i = 0; i < 17; i++) {
		ntrig_dbg("msg part %d: %d %d %d %d %d %d %d %d\n", i,
				buf[offset], buf[offset+1], buf[offset+2],
				buf[offset+3], buf[offset+4], buf[offset+5],
				buf[offset+6], buf[offset+7]);
		offset += 8;
	}
}

/*
 * This function writes (stores) data in the sysfs file
 */
static ssize_t ntrig_spi_test_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *)spi_get_drvdata(tmp_spi_dev);
	char ch;
	int err;

	/* data is not written into actual file on file system,
	 * but rather saves it in a memory buffer */
	ntrig_dbg("inside %s\n", __func__);

	if (count < 1)
		return count;

	ch = buf[0];
	privdata->xfer.len = MAX_SPI_TRANSFER_BUFFER_SIZE;
	if (ch == '0') {
		/* print the state of the IRQ line */
		int err;
		int irq = tmp_spi_dev->irq;
		int irq_gpio = irq_to_gpio(irq);
		int val;
		ntrig_dbg("%s: gpio line for irq is: %d\n",
						__func__, irq_gpio);
		err = gpio_request(irq_gpio, "ntrig_spi_irq");
		if (err) {
			ntrig_dbg("%s: fail to request gpio for irq(%d), err=%d\n",
						__func__, irq_gpio, err);
			/* continue anyway... */
		}
		if (!err)
			err = gpio_direction_input(irq_gpio);
#if 0
		val = gpio_get_value(irq_gpio);
#else
		val = ntrig_get_irq_value();
#endif
		ntrig_dbg("%s: gpio_irq value is %d\n", __func__, val);

	} else if (ch == '1' || ch == '2') {
		/** test: turn on or off output_enable
		 * (1=turn off 2=turn on) note, if an inverter is
		 * connected, the actual result will be the opposite */
		int gpio_index;
		int val = ch - '1';
		gpio_index = privdata->oe_gpio;

		err = gpio_request(gpio_index, "ntrig_spi_output_enable");
		if (err) {
			ntrig_dbg("%s: fail to request gpio for output_enable(%d), err=%d\n",
					__func__, gpio_index, err);
			/* continue anyway... */
		}
		err = gpio_direction_output(gpio_index, val);
		if (err) {
			ntrig_dbg("%s: fail to change output_enable\n",
					__func__);
			return count;
		}
		ntrig_dbg("%s: success, output_enable(%d) set to %d\n",
				__func__, gpio_index, val);
		return count;
	} else if (ch == '3') {
		/** test: send a sample command (GET_FIRMWARE_VERSION) and
		 *  display its response. WARNING: do not use this command
		 *  after activating the driver (ch=='6') because it may
		 *  infefere with operation (we don't have locking yet) */
		struct _ntrig_low_bus_msg *msg =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *)(privdata->rx_buf);
		int err;
		int i;

		msg->preamble = 0xFFFFFFFF;
		msg->pattern[0] = 0xa5;
		msg->pattern[1] = 0x5a;
		msg->pattern[2] = 0xe7;
		msg->pattern[3] = 0x7e;
		msg->msg.type = 0x02; /* COMMAND */
		msg->msg.length = 14;
		msg->msg.flags = 0;
		msg->msg.channel = 0; /* CONTROL */
		msg->msg.function = 0x0C; /* GET_FW_VERSION */
		msg->msg.data[0] = 0xa1;
		msg->msg.data[1] = 0x01;
		msg->msg.data[2] = 0x03;
		msg->msg.data[3] = 0x0c;
		msg->msg.data[4] = 0x01;
		msg->msg.data[5] = 0x00;
		msg->msg.data[6] = 0x08;
		msg->msg.data[7] = 0;
		for (i = 8; i < 122; i++)
			msg->msg.data[i] = 0xff;
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		gpio_set_value(privdata->oe_gpio, 1);

		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		debug_print_msg(rmsg);
		/* once again to see more received data... */
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		gpio_set_value(privdata->oe_gpio, 1);

		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		debug_print_msg(rmsg);
	} else if (ch == '4') {
		/** same as '3' but sends the command many times. Mostly for
		 *  SPI "alive" tests */
		struct _ntrig_low_bus_msg *txbuf =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		int err;
		int i;

		txbuf->preamble = 0xFFFFFFFF;
		txbuf->pattern[0] = 0xa5;
		txbuf->pattern[1] = 0x5a;
		txbuf->pattern[2] = 0xe7;
		txbuf->pattern[3] = 0x7e;
		txbuf->msg.type = 0x02; /* COMMAND */
		txbuf->msg.length = 14;
		txbuf->msg.flags = 0;
		txbuf->msg.channel = 0; /* CONTROL */
		txbuf->msg.function = 0x0C; /* GET_FW_VERSION */
		txbuf->msg.data[0] = 0xa1;
		txbuf->msg.data[1] = 0x01;
		txbuf->msg.data[2] = 0x03;
		txbuf->msg.data[3] = 0x0c;
		txbuf->msg.data[4] = 0x01;
		txbuf->msg.data[5] = 0x00;
		txbuf->msg.data[6] = 0x08;
		txbuf->msg.data[7] = 0;
		for (i = 8; i < 122; i++)
			txbuf->msg.data[i] = 0xff;

		for (i = 0; i < 10000; i++) {
			gpio_set_value(privdata->oe_gpio, 1);
			msleep(20);
			err = spi_sync(tmp_spi_dev, &privdata->msg);
			gpio_set_value(privdata->oe_gpio, 1);
			msleep(20);
		}
		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
	} else if (ch == '5') {
		/* SPI data lines test by sending AA pattern */
		struct _ntrig_low_bus_msg *txbuf =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		int err;
		int i;

		txbuf->preamble = 0xaaaaaaaa;
		txbuf->pattern[0] = 0xaa;
		txbuf->pattern[1] = 0xaa;
		txbuf->pattern[2] = 0xaa;
		txbuf->pattern[3] = 0xaa;
		txbuf->msg.type = 0xaa; /* COMMAND */
		txbuf->msg.length = 0xaaaa;
		txbuf->msg.flags = 0xaa;
		txbuf->msg.channel = 0xaa; /* CONTROL */
		txbuf->msg.function = 0xaa; /* GET_FW_VERSION */
		txbuf->msg.data[0] = 0xaa;
		txbuf->msg.data[1] = 0xaa;
		txbuf->msg.data[2] = 0xaa;
		txbuf->msg.data[3] = 0xaa;
		txbuf->msg.data[4] = 0xaa;
		txbuf->msg.data[5] = 0xaa;
		txbuf->msg.data[6] = 0xaa;
		txbuf->msg.data[7] = 0xaa;
		for (i = 8; i < 122; i++)
			txbuf->msg.data[i] = 0xaa;

		for (i = 0; i < 10000; i++) {
			gpio_set_value(privdata->oe_gpio, 1);
			msleep(20);
			err = spi_sync(tmp_spi_dev, &privdata->msg);
			gpio_set_value(privdata->oe_gpio, 1);
			msleep(20);
		}
		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
	} else if (ch == '6') {
		/* PROBE: activate the driver, register to dispatcher.
		 * This emulates the part that should be executed
		 * as part of device startup */
		int err;
		err = register_to_dispatcher(tmp_spi_dev);
		if (err) {
			ntrig_dbg("%s: register_to_dispatcher failed. result=%d\n",
							__func__, err);
		}
	} else if (ch == '7') {
		/* send START_CALIBRATE command */
		struct _ntrig_low_bus_msg *msg =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *)(privdata->rx_buf);
		int err;
		int i;

		msg->preamble = 0xFFFFFFFF;
		msg->pattern[0] = 0xa5;
		msg->pattern[1] = 0x5a;
		msg->pattern[2] = 0xe7;
		msg->pattern[3] = 0x7e;
		msg->msg.type = 0x02; /* COMMAND */
		msg->msg.length = 14;
		msg->msg.flags = 0;
		msg->msg.channel = 0; /* CONTROL */
		msg->msg.function = 0x0B; /* START_CALIBRATION */
		msg->msg.data[0] = 0xa1;
		msg->msg.data[1] = 0x01;
		msg->msg.data[2] = 0x0b;
		msg->msg.data[3] = 0x03;
		msg->msg.data[4] = 0x00;
		msg->msg.data[5] = 0x01;
		msg->msg.data[6] = 0;
		msg->msg.data[7] = 0x04;

		for (i = 8; i < 122; i++)
			msg->msg.data[i] = 0xff;
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		/* clear the tx_buf so we don't send unexpected commands
		 * to the sensor */
		for (i = 0; i < privdata->xfer.len; i++)
			privdata->tx_buf[i] = 0xFF;
		debug_print_msg(rmsg);
		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		/* do another cycle to get response */
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		debug_print_msg(rmsg);
		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
	} else if (ch == '8') {
		/* send GET_CALIBRATION_STATUS */
		struct _ntrig_low_bus_msg *msg =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *)(privdata->rx_buf);
		int err;
		int i;

		msg->preamble = 0xFFFFFFFF;
		msg->pattern[0] = 0xa5;
		msg->pattern[1] = 0x5a;
		msg->pattern[2] = 0xe7;
		msg->pattern[3] = 0x7e;
		msg->msg.type = 0x02; /* COMMAND */
		msg->msg.length = 14;
		msg->msg.flags = 0;
		msg->msg.channel = 0; /* CONTROL */
		msg->msg.function = 0x11; /* GET_CALIBRATION_STATUS */
		msg->msg.data[0] = 0xa1;
		msg->msg.data[1] = 0x01;
		msg->msg.data[2] = 0x11;
		msg->msg.data[3] = 0x03;
		msg->msg.data[4] = 0x00;
		msg->msg.data[5] = 0x01;
		msg->msg.data[6] = 0;
		msg->msg.data[7] = 0x04;

		for (i = 8; i < 122; i++)
			msg->msg.data[i] = 0xff;
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		/* clear the tx_buf so we don't send unexpected commands
		 * to the sensor */
		for (i = 0; i < privdata->xfer.len; i++)
			privdata->tx_buf[i] = 0xFF;

		debug_print_msg(rmsg);
		/* do another cycle to get response */
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		/*gpio_set_value(privdata->oe_gpio, 1); */

		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		debug_print_msg(rmsg);
	} else if (ch == '9') {
		/* print state machine statistics */
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *)(privdata->rx_buf);
		ntrig_dbg("%s: state=%d substate=%d\n",
				__func__, privdata->sm.state,
				privdata->sm.substate);
		/* last received data */
		debug_print_msg(rmsg);
	} else if (ch == 'a') {
		/* send message which comes completely from sysfs */
		struct _ntrig_low_bus_msg *msg =
			(struct _ntrig_low_bus_msg *)(privdata->tx_buf);
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *)(privdata->rx_buf);
		int err;
		int i;

		memcpy((char *)msg, buf+1, count-1);
		for (i = count - 1; i < 136; i++)
			((char *)msg)[i] = 0xff;
		pr_info("spi_test_store: writing ");
		for (i = 0; i < 136; i++)
			pr_info("%x ", (int)((unsigned char *)msg)[i]);
		pr_info("\n");
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		/*gpio_set_value(privdata->oe_gpio, 1); */

		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		/* clear the tx_buf so we don't send unexpected commands
		 * to the sensor */
		for (i = 0; i < privdata->xfer.len; i++)
			privdata->tx_buf[i] = 0xFF;
		debug_print_msg(rmsg);
		/* do another cycle to get response */
		gpio_set_value(privdata->oe_gpio, 1);
		msleep(20);
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		/*gpio_set_value(privdata->oe_gpio, 1); */

		if (err) {
			ntrig_dbg("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
		debug_print_msg(rmsg);
	} else if (ch == 'b') {
		pr_info("ntrig sakont test interface");
		read_touchbuffer_to_empty(privdata);
	}

	return count;
}

/*
 * This function reads (shows) data from the sysfs file
 */
static ssize_t ntrig_spi_test_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	ntrig_dbg("inside %s\n", __func__);
	return snprintf(buf, PAGE_SIZE, "hello from spi_test\n");
}

/*
 * The release function for our object.  This is REQUIRED by the kernel to
 * have.  We free the memory held in our object here.
 */
static void ntrig_spi_test_release(struct kobject *kobj)
{
	ntrig_dbg("inside %s\n", __func__);

	if (kobj == NULL) {
		ntrig_dbg("inside %s , kobj==NULL\n", __func__);
		return;
	}

	if (ntrig_spi_test_attr.attr.name != NULL) {
		sysfs_remove_file(kobj, ntrig_spi_test_attrs);
		ntrig_dbg("inside %s After call to sysfs_remove_file\n",
				__func__);
	} else
		ntrig_dbg("inside %s Skip call to sysfs_remove_file\n",
				__func__);

	kobject_put(kobj);
	ntrig_dbg("inside %s After call to kobject_put\n", __func__);

	/* kobj is pointer to properties_kobj which was allocated in
	 * ntrig_create_virtualkeys_file */
	ntrig_dbg("inside %s After call to kfree(kobj)\n", __func__);
#if 0
	/* ntrig_virtual_keys_attr.attr.name was allocated in
	 * ntrig_create_virtualkeys_file */
	if (tmpBuf != NULL) {
		kfree(tmpBuf);
		ntrig_dbg("inside %s After call to kfree(tmpBuf)\n",
				__func__);
	} else
		ntrig_dbg("inside %s Skip call to kfree(tmpBuf)\n",
				__func__);
#endif
}

/**
 * send the DRIVER_ALIVE message to the sensor, to ensure
 * it switches to multi-touch mode
 */
static void ntrig_spi_send_driver_alive(struct spi_device *spi)
{
	ntrig_dbg("in %s\n", __func__);
	spi_write_hid_ncp(spi, LOWMSG_FUNCTION_DRIVER_ALIVE, NULL, 0);
}

static int __devinit ntrig_spi_probe(struct spi_device *spi);
static int ntrig_spi_remove(struct spi_device *spi);

#ifdef CONFIG_PM
/**
 * called when suspending the device (sleep with preserve
 * memory)
 */
static int ntrig_spi_suspend(struct spi_device *dev, pm_message_t mesg)
{
	struct ntrig_spi_privdata *privdata;

	privdata = (struct ntrig_spi_privdata *)dev_get_drvdata(
					(const struct device *)dev);
	watchdog_timer.is_suspended = true;
	ntrig_dbg("in %s\n", __func__);
	spi_set_pwr(privdata, false);

	return 0;
}

/**
 * called when resuming after sleep (suspend)
 */
static int ntrig_spi_resume(struct spi_device *dev)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *)dev_get_drvdata(
					(const struct device *)dev);
	ntrig_dbg("in %s\n", __func__);
	spi_set_pwr(privdata, true);
	watchdog_timer.is_suspended = false;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ntrig_spi_early_suspend(struct early_suspend *h)
{
	struct ntrig_spi_privdata *privdata;
	pm_message_t msg = PMSG_SUSPEND;

	privdata = container_of(h, struct ntrig_spi_privdata, early_suspend);
	ntrig_spi_suspend(privdata->spi, msg);
}

static void ntrig_spi_early_resume(struct early_suspend *h)
{
	struct ntrig_spi_privdata *privdata;
	privdata = container_of(h, struct ntrig_spi_privdata, early_suspend);
	ntrig_spi_resume(privdata->spi);
}
#endif

#endif

static struct spi_driver ntrig_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		/*.pm = &spi_pm_ops, */
#endif
		},
	.suspend =	ntrig_spi_suspend,
	.resume =	ntrig_spi_resume,
	.probe =	ntrig_spi_probe,
	.remove =	__devexit_p(ntrig_spi_remove),
};

/**
 * initialize the SPI driver. Called by board init code
 */
static int __devinit ntrig_spi_probe(struct spi_device *spi)
{
	struct ntrig_spi_privdata *pdata;
	int err, low, high, gpio_index;
	int irq_gpio;
	struct ntrig_spi_platform_data *platdata = spi->dev.platform_data;

	pr_err("Ntrig_spi_probe N-trig SPI driver\n");
	if (platdata == NULL) {
		pr_err("ntrig_spi_probe() ERROR platform data is NULL!!!\n");
		return -ENOMEM;
	}
	ntrig_dbg("%s: output_enable gpio is %d\n",
						__func__, platdata->oe_gpio);
	pdata = kzalloc(sizeof(struct ntrig_spi_privdata), GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(&spi->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	err = setup_transfer(pdata);
	if (err) {
		ntrig_dbg("%s: setup_transfer failure\n", __func__);
		return err;
	}

	pdata->spi = spi;
	pdata->irq_gpio = spi->irq;
	spi->irq = gpio_to_irq(spi->irq);
	pdata->oe_gpio = platdata->oe_gpio;
	pdata->oe_inverted = platdata->oe_inverted;
	pdata->irq_flags = platdata->irq_flags;
	pdata->aggregation_size = 0;
	pdata->fragments_left = 0;
	dev_set_drvdata(&spi->dev, pdata);

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0) {
		ntrig_dbg("%s: spi_setup failure\n", __func__);
		return err;
	}
	sema_init(&pdata->spi_lock, 1);
	spin_lock_init(&watchdog_timer.lock);
	watchdog_timer.pdata = pdata;

	/* init variables related to ncp hid read/write */
	pdata->hid_ncp_got_result = 0;
	init_waitqueue_head(&pdata->hid_ncp_wait_queue);

	/* init variables relatedto raw read/write we use the spi_lock to
	 * protect the fifo, no additional lock is needed */
	err = kfifo_alloc(&pdata->raw_fifo,
			RAW_RECEIVE_BUFFER_INITIAL_SIZE, GFP_KERNEL);
	if (err) {
		ntrig_dbg("%s: fail to alloc raw_fifo, err = %d\n",
							__func__, err);
		return err;
	}
	init_waitqueue_head(&pdata->raw_wait_queue);

	tmp_spi_dev = spi; /* for debugging */

#ifndef NTRIG_SPI_DEBUG_MODE
	/* set the output_enable gpio line to allow sensor to work */
	gpio_index = pdata->oe_gpio;
	if (gpio_index >= 0) {
		err = gpio_request(gpio_index, "ntrig_spi_output_enable");
		if (err) {
			ntrig_dbg("%s: fail to request gpio for output_enable(%d), err=%d\n",
					__func__, gpio_index, err);
			/* continue anyway...*/
		}
		low = pdata->oe_inverted ? 1 : 0;
		high = pdata->oe_inverted ? 0 : 1;
		err = gpio_direction_output(gpio_index, low);
		if (err) {
			ntrig_dbg("%s: fail to change output_enable\n",
					__func__);
			return err;
		}
		msleep(50);
	}

	/* register the IRQ GPIO line. The actual interrupt is requested
	 * in register_to_dispatcher */
	/*irq_gpio = irq_to_gpio(spi->irq); */
	irq_gpio = pdata->irq_gpio;
	err = gpio_request(irq_gpio, "ntrig_spi_irq");

	if (err) {
		pr_err("%s: fail to request gpio for irq(%d), err=%d\n",
				__func__, irq_gpio, err);
		/* continue anyway... */
	}

	err = gpio_direction_input(irq_gpio);
	if (err)
		pr_err("%s: fail to set irq input, err = %d\n", __func__, err);

	/* register with the dispatcher,
	 * this will also create input event files */
	err = register_to_dispatcher(spi);
	if (err) {
		pr_err("%s: fail to register to dispatcher, err = %d\n",
								__func__, err);
		return err;
	}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	pdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	pdata->early_suspend.suspend = ntrig_spi_early_suspend;
	pdata->early_suspend.resume = ntrig_spi_early_resume;
	register_early_suspend(&pdata->early_suspend);
#endif

	pdata->pwr_gpio = platdata->pwr_gpio;

	disable_irq(pdata->spi->irq);
	init_spi_pwr_gpio(pdata);

	/* set the output_enable gpio line to allow sensor to work */
	gpio_index = pdata->oe_gpio;
	if (gpio_index >= 0) {
		high = pdata->oe_inverted ? 0 : 1;
		msleep(500);
		gpio_set_value(gpio_index, high);
	}

	enable_irq(pdata->spi->irq);

	watchdog_timer.kobj = ntrig_create_spi_watchdog_file();
	/* 1 sec */
	watchdog_timer.interval = msecs_to_jiffies(SPI_WATCHDOG_INTERVAL_MSEC);

	if (!watchdog_timer.work_q) {
		watchdog_timer.work_q =
				create_singlethread_workqueue("watchdog");
	}
	if (watchdog_timer.work_q) {
		queue_delayed_work(watchdog_timer.work_q, &watchdog_task,
						watchdog_timer.interval);
	}

	spi_reset_counters();

	/* send DRIVER_ALIVE to put sensor*/
	ntrig_spi_send_driver_alive(spi);

	/* success */
	return 0;
}

/**
 * release the SPI driver
 */
static int ntrig_spi_remove(struct spi_device *spi)
{
	struct ntrig_spi_privdata *pdata = dev_get_drvdata(&spi->dev);

	if (watchdog_timer.work_q) {
		/* signal the task function to stop queueing new tasks */
		watchdog_timer.is_exit = true;
		/* cancel existing queued work */
		cancel_delayed_work(&watchdog_task);
		/* wait for ongoing work to finish */
		flush_workqueue(watchdog_timer.work_q);
		destroy_workqueue(watchdog_timer.work_q);
	}

	if (!pdata)
		return -EINVAL;

	/* TODO we use a hard-coded bus id - need to change it in order to
	 * support multiple sensors connected over SPI bus */
	UnregNtrigDispatcher(pdata->ntrig_dispatcher,
			pdata->sensor_id, TYPE_BUS_SPI_HID, "spi");
	UnregNtrigDispatcher(pdata->ntrig_dispatcher,
			pdata->sensor_id, TYPE_BUS_SPI_RAW, "spi");

	if (pdata->test_kobj)
		ntrig_spi_test_release(pdata->test_kobj);

	if (watchdog_timer.kobj)
		ntrig_spi_watchdog_release(watchdog_timer.kobj);

	kfree(pdata);
	return 0;
}


static int __init ntrig_spi_init(void)
{
	int ret;
	ret = spi_register_driver(&ntrig_spi_driver);

	if (ret != 0)
		pr_err("Failed to register N-trig SPI driver: %d\n", ret);

	return ret;
}

static void __exit ntrig_spi_exit(void)
{
	spi_unregister_driver(&ntrig_spi_driver);
}

module_init(ntrig_spi_init);
module_exit(ntrig_spi_exit);

MODULE_ALIAS("ntrig_spi");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("N-Trig SPI driver");
