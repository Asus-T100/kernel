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


#include <linux/spi/ntrig_spi.h>

#include "typedef-ntrig.h"
#include "ntrig-common.h"
#include "ntrig-dispatcher.h"
#include "ntrig_low_msg.h"

/* The following file has been added in kernel version 2.6.39 */
#include <linux/sched.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DRIVER_NAME "ntrig_g4_spi"

/** define this macro to put driver in "debug" mode - it will
 * not be loaded at startup, and need to be loaded by echo 6 >
 * /sys/spi_test/test
 */
/* #define NTRIG_SPI_DEBUG_MODE 1 */

/** the maximum time to wait for a reply from raw(ncp/dfu)
 * command.
 */
/* 1 seconds */
#define MAX_SPI_RAW_COMMAND_REPLY_TIMEOUT (HZ * 1)

/** size (bytes) of the circular buffer we keep for raw/ncp
 * commands */
#define RAW_RECEIVE_BUFFER_INITIAL_SIZE 4096
#define RAW_RECEIVE_BUFFER_MAX_SIZE (1024 * 120)
/** maximum size of SPI buffer for send/receive. The buffers
 * will be allocated for this size. 264 bytes for G4 (256 +
 * 8 bytes preamble */
#define MAX_SPI_TRANSFER_BUFFER_SIZE 264
/** maximum size of an aggregated logical SPI message,
 * G3.5: 16 (max fragments) * 122 (max message size)
 * G4: 4K */
#define MAX_LOGICAL_MESSAGE_SIZE (1024 * 4)

/** SPI message size for G3.x sensor (128 bytes + 8 bytes
 * preamble) */
#define SPI_MESSAGE_SIZE_G3 136
/** SPI message size for G4 sensor (256 bytes + 8 bytes
 * preamble */
#define SPI_MESSAGE_SIZE_G4 264

/** counters names **/
/* device to host */
#define CNTR_NAME_MULTITOUCH "channel multitouch"
#define CNTR_NAME_PEN "channel pen"
#define CNTR_NAME_MAINT_REPLY "channel maint reply"
#define CNTR_NAME_DEBUG_REPLY "channel debug reply"

/* host to device */
#define CNTR_NAME_MAINT "channel maint"
#define CNTR_NAME_DEBUG "channel debug"

#define CNTR_NAME_ERROR_FULL_RECEIVE_QUEUE "full queue error"
#define CNTR_NAME_ERROR_FRAGMENTATION "fragmentation error"
#define CNTR_NAME_ERROR_MAINT_PACKET_SIZE "maint. packet size error"
#define CNTR_NAME_ERROR_NCP_BAD_FIRST_BYTE "ncp bad first byte error"
#define CNTR_NAME_NUM_MT_PACKET_LOST "number of multi touch lost packets"
#define CNTR_NAME_NUM_PEN_PACKET_LOST "number of pen lost packets"

enum _spi_cntrs_names {
	CNTR_MULTITOUCH = 0,
	CNTR_PEN,
	CNTR_MAINTREPLY,
	CNTR_DEBUGREPLY,
	CNTR_MAINT,
	CNTR_DEBUG,
	CNTR_ERROR_FULL_RECEIVE_QUEUE,
	CNTR_ERROR_FRAGMENTATION,
	CNTR_ERROR_MAINT_PACKET_SIZE,
	CNTR_ERROR_NCP_BAD_FIRST_BYTE,
	CNTR_NUM_MT_PACKET_LOST,
	CNTR_NUM_PEN_PACKET_LOST,
	CNTR_NUMBER_OF_SPI_CNTRS
};

static struct _ntrig_counter spi_cntrs_list[CNTR_NUMBER_OF_SPI_CNTRS] = {
	{.name = CNTR_NAME_MULTITOUCH, .count = 0},
	{.name = CNTR_NAME_PEN, .count = 0},
	{.name = CNTR_NAME_MAINT_REPLY, .count = 0},
	{.name = CNTR_NAME_DEBUG_REPLY, .count = 0},
	{.name = CNTR_NAME_MAINT, .count = 0},
	{.name = CNTR_NAME_DEBUG, .count = 0},
	{.name = CNTR_NAME_ERROR_FULL_RECEIVE_QUEUE, .count = 0},
	{.name = CNTR_NAME_ERROR_FRAGMENTATION, .count = 0},
	{.name = CNTR_NAME_ERROR_MAINT_PACKET_SIZE, .count = 0},
	{.name = CNTR_NAME_ERROR_NCP_BAD_FIRST_BYTE, .count = 0},
	{.name = CNTR_NAME_NUM_MT_PACKET_LOST, .count = 0},
	{.name = CNTR_NAME_NUM_PEN_PACKET_LOST, .count = 0},
};

/* NOTE: Static variables and global variables are automatically initialized to
 * 0 by the compiler. The kernel style checker tool (checkpatch.pl) complains
 * if they are explicitly initialized to 0 (or NULL) in their definition.
 */

static int check_hid_checksum = 1;
static unsigned long bad_hid_checksum_counter;
static unsigned char packet_dump[SPI_MESSAGE_SIZE_G4 * 3];
	/* buffer for packet dump */

/** driver private data */
struct ntrig_spi_privdata {
	/** pointer back to the spi device */
	struct spi_device *spi;
	/** the sensor id assigned to us by dispatcher */
	int sensor_id;
	/** for debugging: sysfs file for sending test commands */
	struct kobject *test_kobj;
	/** gpio index for output_enable, copied from spi platform data */
	int oe_gpio;
	/** true if output_enable line is connected to inverter,
	 *  copied from spi platform data */
	int oe_inverted;
	/** gpio index for the irq line */
	unsigned irq_gpio;
	/** flags to use for requesting interrupt handler */
	int irq_flags;
	/** gpio index for power */
	int pwr_gpio;
	/** for spi transfer */
	struct spi_message msg;
	struct spi_transfer xfer;
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	/** the current configured SPI message size (136 bytes or 264
	 *  bytes). initialized to 136 bytes but we will switch to
	 *  264 bytes if we detect a G4 sensor */
	int spi_msg_size;
	/** state machine for processing incoming data from SPI link */
	struct _ntrig_low_sm_info sm;
	/** semaphore (mutex) for protecting the spi transfer/state
	 *  machine access */
	struct semaphore spi_lock;
	/** shared structure between this driver and dispatcher TODO
	 *  to be removed in new architecture */
	struct _ntrig_bus_device *ntrig_dispatcher;
	/** message for sending hid reports to dispatcher */
	struct mr_message_types_s report;
	/** --------- WRITE/READ BULK/RAW NCP COMMANDS -------- */
	/** circular buffer for storing incoming raw replies */
	struct kfifo raw_fifo;
	/** wait queue for implementing read blocking (when raw_fifo
	 *  is empty) */
	wait_queue_head_t raw_wait_queue;
	/** flag used when waiting for buffer to be filled */
	int raw_data_arrived;
	/** scratch buffer for discarding packets (when circular
	 *  buffer is full) and for copying the old fifo to the
	 *  new one during fifo expansion */
	u8 scratch_buf[MAX_SPI_TRANSFER_BUFFER_SIZE];
	/** buffer for aggregating fragmented messages */
	u8 aggregation_buf[MAX_LOGICAL_MESSAGE_SIZE];
	/** size of aggregated message */
	int aggregation_size;
	/** number of logical message fragments left for complete message */
	int fragments_left;
	/** counters for multi touch reports **/
	unsigned int expected_mt_counter;
	unsigned int cur_mt_counter;

	/** counters for pen reports **/
	unsigned int expected_pen_counter;
	unsigned int cur_pen_counter;

	/** boolean to determine if drop or not heart bit report **/
	bool filter_HB;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
};

void g4_spi_reset_counters(void)
{
	int i;
	for (i = 0; i < CNTR_NUMBER_OF_SPI_CNTRS; ++i)
		spi_cntrs_list[i].count = 0;
}

static void spi_set_pwr(struct ntrig_spi_privdata *pdata, bool enable);

/** for	debugging */
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
		ntrig_err("%s: fail to allocate tx_buf\n", __func__);
		return -ENOMEM;
	}

	data->rx_buf = kmalloc(len, GFP_KERNEL);
	if (!data->rx_buf) {
		ntrig_err("%s: fail to allocate rx_buf\n", __func__);
		kfree(data->tx_buf);
		return -ENOMEM;
	}

	m = &data->msg;
	x = &data->xfer;

	spi_message_init(m);

	x->tx_buf = &data->tx_buf[0];
	x->rx_buf = &data->rx_buf[0];
	/* make sure you fill the length correctly before doing an SPI transfer,
	 * up to MAX_SPI_TRANSFER_BUFFER_SIZE */
	spi_message_add_tail(x, m);

	/** initial message size - 136 bytes for G3.5 sensor. We will switch to
	 * 264 if detected a G4 sensor */
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
 * enable or disable the output_enable line connected to our
 * sensor.
 * When enabled, sensor can access the SPI bus
 * When disabled, sensor cannot access the SPI bus.
 * Note that if the output_enable line is connected to an
 * inverter, we reverse the values for enable/disable
 * (high=disable, low=enable)
 * Assumes gpio line was prepared before (request, set
 * direction)
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

		pr_info("ntrig spi_set_output_enable pin:%d val:%d\n",
				pdata->oe_gpio, val);
	}
}

/**
 * return 1 if output_enable line is in enabled state, 0 if
 * it is in disabled state.
 * If the output_enable line is connected through an inverter,
 * reverse the gpio returned values to take it into account
 */
static bool spi_get_output_enabled(struct ntrig_spi_privdata *pdata)
{
	if (pdata->oe_gpio >= 0) {
		int val = gpio_get_value(pdata->oe_gpio);

		if (pdata->oe_inverted)
			return val ? 0 : 1;
		else
			return val ? 1 : 0;
	} else /* no gpio available, assume as if it is always set */
		return 1;
}

/**
 * helper for managing the raw receive buffer. Discards a
 * message from the buffer
 * Must be called with spi_lock held
 */
static void spi_discard_raw_message(struct ntrig_spi_privdata *privdata)
{
	u16 size;
	int res;
	res = kfifo_out(&privdata->raw_fifo,
		(unsigned char *)&size, sizeof(u16));
	while (size > 0) {
		int getsize = size;
		if (getsize > MAX_SPI_TRANSFER_BUFFER_SIZE)
			getsize = MAX_SPI_TRANSFER_BUFFER_SIZE;
		res = kfifo_out(&privdata->raw_fifo, &privdata->scratch_buf[0],
			getsize);
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
	int buf_size, get_size, res;
	int err = kfifo_alloc(&new_fifo, RAW_RECEIVE_BUFFER_MAX_SIZE,
			GFP_KERNEL);
	if (err) {
		ntrig_err("%s: fail to allocate a larger raw_fifo, err = %d\n",
			__func__, err);
		return;
	}
	/* copy the contents of the old fifo to the new one */
	buf_size = sizeof(privdata->scratch_buf);
	while ((get_size = kfifo_len(&privdata->raw_fifo)) > 0) {
		if (get_size > buf_size)
			get_size = buf_size;
		res = kfifo_out(&privdata->raw_fifo, &privdata->scratch_buf[0],
			get_size);
		kfifo_in(&new_fifo, &privdata->scratch_buf[0], get_size);
	}
	kfifo_free(&privdata->raw_fifo);
	privdata->raw_fifo = new_fifo;
}

static int spi_check_hid_checksum(struct _ntrig_low_msg *msg)
{
	u16 expected_checksum = 0, real_checksum;
	int i;
	for (i = 0; i < msg->length - 2; i++)
		expected_checksum += ((u8 *)msg)[i];
	memcpy(&real_checksum, &(msg->data[msg->length - 8]), 2);
	if (expected_checksum != real_checksum) {
		ntrig_err("%s: Found incorrect checksum %d, expected %d\n",
			__func__, (int) real_checksum, (int) expected_checksum);
		bad_hid_checksum_counter++;
		/* Print the entire packet every 10 bad checksums */
		if (bad_hid_checksum_counter % 10 == 0) {
			unsigned char *write_ptr = packet_dump;
			unsigned char *write_limit = packet_dump +
				sizeof(packet_dump) - 4;
			for (i = 0; i < msg->length; i++) {
				int n;
				if (write_ptr > write_limit) {
					/* No more space in buffer - error */
					break;
				}
				n = snprintf(write_ptr, 4, " %x",
					(unsigned int) ((u8 *)msg)[i]);
				write_ptr += n;
			}
			ntrig_err("%s: packet bytes: %s%s\n", __func__,
				packet_dump, (write_ptr > write_limit) ?
				" ..." : "");
		}
		return 1;
	}
	return 0;
}

/*
 * verify that current counter is equal to he expected
 * if not, print number of packets that been missed
 * and update expected counter
 * return the nuber of missed packets
 */
int checkForLostPackets(u32 cur, u32 *expected)
{
	ntrig_dbg_lvl(5, "%s: current counter: %u, expected counter: %u\n",
			__func__, cur, *expected);
	if ((*expected) != cur) {
		u32 nLost = cur - (*expected);
		ntrig_err("we lost %u packet(s), current: %u, expected: %u\n",
				nLost, cur, (*expected));
		(*expected) = cur+1;
		return nLost;
	} else
		(*expected)++;

	return 0;
}

static int check_fragmentation_error(struct ntrig_spi_privdata *privdata,
	u8 flags, u16 size)
{
	int message_error = 0;
	if ((privdata->fragments_left > 0) &&
		(privdata->fragments_left != flags + 1)) {
		spi_cntrs_list[CNTR_ERROR_FRAGMENTATION].count++;
		ntrig_err("%s: logical message fragmentation corruption - previous =%d, current=%d, discarding\n",
				__func__, privdata->fragments_left,
			flags);
		message_error = 1;
	} else if (privdata->aggregation_size + size >
		MAX_LOGICAL_MESSAGE_SIZE) {
		ntrig_err("%s: logical message too large buffer (size=%d, max=%d), discarding\n",
				__func__, privdata->aggregation_size + size,
				MAX_LOGICAL_MESSAGE_SIZE);
		message_error = 1;
	}
	return message_error;
}

static void add_message_fragment(struct ntrig_spi_privdata *privdata, u8 *data,
	u16 size, u8 flags)
{
	memcpy(&privdata->aggregation_buf[privdata->aggregation_size], data,
		size);
	privdata->aggregation_size += size;
	privdata->fragments_left = flags;
}

/**
 * called when we have a complete message received from the spi layer
 */
static void spi_process_message(struct ntrig_spi_privdata *privdata)
{
	struct _ntrig_low_msg *msg = &privdata->sm.lmsg;
	struct mr_message_types_s *mr = &privdata->report;
	int num_pct_lost, events;
	ntrig_dbg("%s: message type %d\n", __func__, msg->type);
	ntrig_dbg("%s: channel=%d function=%d\n", __func__, msg->channel,
		msg->function);

	if (msg->flags & 0x80) {
		/* bit 7 set in flags means a 256 byte packet arrived, this is
		 * a G4 sensor. Switch our SPI message size so next transfers
		 * will be more efficient. */
		privdata->spi_msg_size = SPI_MESSAGE_SIZE_G4;
	}
	switch (msg->channel) {
	case LOWMSG_CHANNEL_MULTITOUCH_TRACKED:
	case LOWMSG_CHANNEL_MULTITOUCH:
		if (msg->function != LOWMSG_FUNCTION_MT_REPORT) {
			ntrig_err("%s: invalid mt report, function=%d\n",
				__func__, msg->function);
			break;
		}
		/* fill in multi-touch report and send to dispatcher */
		int i;
		u8 contactCount;
		struct _ntrig_low_mt_report *mtr =
			(struct _ntrig_low_mt_report *)&msg->data[0];

		if (check_hid_checksum) {
			if (spi_check_hid_checksum(msg))
				/*break*/
				;
		}
		/***** check for lost reports *****/
		/*copy the report counter to cur_mt_counter*/
		memcpy(&privdata->cur_mt_counter,
			&(msg->data[msg->length-12]), 4);
		num_pct_lost = checkForLostPackets(
			privdata->cur_mt_counter,
			&privdata->expected_mt_counter);
		spi_cntrs_list[CNTR_NUM_MT_PACKET_LOST].count +=
			num_pct_lost;

		if (msg->flags & 0x80) {
			struct _ntrig_low_mt_finger_g4 *fingers;
			/*256 byte report always sends 10 fingers (G4)*/
			contactCount = mtr->g4fingers.contactCount;
			fingers = &mtr->g4fingers.fingers[0];
			if (contactCount > MAX_MT_FINGERS_G4) {
				ntrig_err("invalid g4 mt report, too many contactCount: %d\n",
						contactCount);
				return;
			}
			ntrig_dbg("%s: finger count=%d vendor defined = 0x%X\n",
					__func__, contactCount,
					fingers[0].vendorDefined);
			mr->type = MSG_FINGER_PARSE;
			mr->msg.fingers_event.sensor_id = privdata->sensor_id;
			mr->msg.fingers_event.frame_index = mtr->reportCount;
			mr->msg.fingers_event.num_of_fingers = contactCount;
			for (i = 0; i < contactCount; i++) {
				struct device_finger_s *finger =
					&mr->msg.fingers_event.finger_array[i];
				finger->x_coord = fingers[i].x;
				finger->y_coord = fingers[i].y;
				finger->dx = fingers[i].dx;
				finger->dy = fingers[i].dy;
				finger->track_id = fingers[i].fingerIndex;
				ntrig_dbg("%s: finger flags = 0x%X\n", __func__,
						(int)fingers[i].flags);
				/*tip switch*/
				finger->removed = !(fingers[i].flags & 0x01);
				/*in range is same as removed == tip switch*/
				finger->generic = !(fingers[i].flags & 0x01);
				finger->palm = !((fingers[i].flags & 0x04) >>
						2);
				/*1=touch valid, 0=palm detected*/
			}
			spi_cntrs_list[CNTR_MULTITOUCH].count++;
			/* call the dispatcher to deliver the message */
			g4_WriteHIDNTRIG(mr);
		} else {
			struct _ntrig_low_mt_finger_g3 *fingers;
			/* 128 byte report, 6 fingers (G3.x) */
			contactCount = mtr->g3fingers.contactCount;
			fingers = &mtr->g3fingers.fingers[0];
			if (contactCount > MAX_MT_FINGERS_G3) {
				ntrig_err("%s: invalid g3 mt report, too many fingers: %d\n",
					__func__, contactCount);
				return;
			}
			ntrig_dbg("%s: finger count=%d vendor defined = 0x%X\n",
					__func__, contactCount,
					fingers[0].vendorDefined);

			mr->type = MSG_FINGER_PARSE;
			mr->msg.fingers_event.sensor_id = privdata->sensor_id;
			mr->msg.fingers_event.frame_index = mtr->reportCount;
			mr->msg.fingers_event.num_of_fingers = contactCount;
			for (i = 0; i < contactCount; i++) {
				struct device_finger_s *finger =
					&mr->msg.fingers_event.finger_array[i];
				finger->x_coord = fingers[i].x;
				finger->y_coord = fingers[i].y;
				finger->dx = fingers[i].dx;
				finger->dy = fingers[i].dy;
				finger->track_id = fingers[i].fingerIndex;
				ntrig_dbg("%s: finger flags = 0x%X\n", __func__,
						(int)fingers[i].flags);
				/*tip switch*/
				finger->removed = !(fingers[i].flags & 0x01);
				/*in range is same as removed == tip switch*/
				finger->generic = !(fingers[i].flags & 0x01);
				/*1=touch valid, 0=palm detected*/
				finger->palm = !((fingers[i].flags & 0x04) >>
						2);
			}
			spi_cntrs_list[CNTR_MULTITOUCH].count++;
			/* call the dispatcher to deliver the message */
			g4_WriteHIDNTRIG(mr);
		}
		break;
	case LOWMSG_CHANNEL_PEN:
		if (msg->function != LOWMSG_FUNCTION_PEN_REPORT) {
			ntrig_err("%s: invalid pen report, function=%d\n",
				__func__, msg->function);
			break;
		}
		/* fill in pen report and send to dispatcher */
		struct _ntrig_low_pen_report *pr =
			(struct _ntrig_low_pen_report *)&msg->data[0];
		if (check_hid_checksum) {
			if (spi_check_hid_checksum(msg))
				/*break*/
				;
		}
		/***** check for lost reports *****/
		/*copy the report counter to cur_pen_counter*/
		memcpy(&privdata->cur_pen_counter,
				&(msg->data[msg->length-12]), 4);
		num_pct_lost = checkForLostPackets(privdata->cur_pen_counter,
				&privdata->expected_pen_counter);
		spi_cntrs_list[CNTR_NUM_PEN_PACKET_LOST].count += num_pct_lost;
		/***** check for lost reports *****/
		mr->type = MSG_PEN_EVENTS;
		mr->msg.pen_event.sensor_id = privdata->sensor_id;
		mr->msg.pen_event.x_coord = pr->x;
		mr->msg.pen_event.y_coord = pr->y;
		mr->msg.pen_event.pressure = pr->pressure;
		mr->msg.pen_event.btn_code = pr->flags;
		mr->msg.pen_event.battery_status = pr->battery_status;
		spi_cntrs_list[CNTR_PEN].count++;
		/* call the dispatcher to deliver the message */
		g4_WriteHIDNTRIG(mr);
		break;
	case LOWMSG_CHANNEL_DEBUG_REPLY:
	{
		/* reply to the debug agent - extend raw fifo if not already
		 * extended.
		 * we assume we are inside spi_lock */
		if (kfifo_size(&privdata->raw_fifo) <
			RAW_RECEIVE_BUFFER_MAX_SIZE) {
			spi_cntrs_list[CNTR_DEBUGREPLY].count++;
			spi_expand_raw_fifo(privdata);
		}
	}
	/* fall through */
	case LOWMSG_CHANNEL_MAINT_REPLY:
	{
		/* reply to a raw/ncp message (mostly used for dfu) */
		/* copy the payload (after function) to a circular buffer where
		 * it can be retrieved later as a fifo.
		 * we assume we are inside spi_lock */
		u16 size = msg->length - offsetof(struct _ntrig_low_msg, data);
		u8 *data = &msg->data[0];
		int avail;
		ntrig_dbg("%s: received ncp reply, size=%d\n", __func__, size);
		if ((size + sizeof(u16)) > kfifo_size(&privdata->raw_fifo)) {
			spi_cntrs_list[CNTR_ERROR_MAINT_PACKET_SIZE].count++;
			ntrig_err("%s: packet too large to put in buffer (size=%d, max=%d), discarding\n",
					__func__, size,
					kfifo_size(&privdata->raw_fifo));
			break;
		}
		/* handle fragmented logical messages - flags=number of
		 * fragments left */
		u8 flags = (msg->flags & ~0x80);
			/* Ignore MSB, which indicates packet size */
		if ((flags > 0) || (privdata->fragments_left > 0)) {
			/* logical message fragment */
			if (check_fragmentation_error(privdata, flags, size)) {
				/* discard logical message */
				privdata->aggregation_size = 0;
				privdata->fragments_left = 0;
				break;
			}
			add_message_fragment(privdata, data, size, flags);
			if (flags > 0) { /* more fragments to come */
				ntrig_dbg("%s: fragmented logical message, waiting for complete message\n",
					__func__);
				break;
			}
			/* last fragment received */
			data = privdata->aggregation_buf;
			size = privdata->aggregation_size;
			privdata->aggregation_size = 0;
		}
#define FIRST_HB_MESSAGE 0x01
		/* check heartbeat message - group=0x20, code=0x1 */
		/* If this is heart bit message, check:
		 * 1. If filter_HB enable (it enable / disable via sysfs file,
		 * /sys/n-trig_spi/filter_HB) - discard message
		 * 2. If this is the first HB and enableHB in FW==0 (data 68) -
		 * discard message.
		 * If enableHB in FW==1 or 2, there should be application in
		 * the user space that should hold this message and ack it.
		 */
		if (data[6] == 0x20 && data[7] == 0x1)
			pr_err("heartBit, data[5]=0x%X\n", (int)data[5]);
		if (data[5] == 0x82 && data[6] == 0x20 && data[7] == 0x1 &&
				(privdata->filter_HB ||
				 (data[67] == FIRST_HB_MESSAGE &&
				  data[68] == 0))) {
			pr_debug("%s: Discarding heartbeat\n", __func__);
			break;
		}
		/* Count the NCP messages with start bytes which is not 0x7e
		 * (data error) */
		if ((msg->function == LOWMSG_FUNCTION_NCP) &&
			(data[0] != LOWMSG_REQUEST_NCP_DFU))
			spi_cntrs_list[CNTR_ERROR_NCP_BAD_FIRST_BYTE].count++;
		/* if fifo can't hold our messages, discard messages from it
		 * until it has room */
		avail = kfifo_avail(&privdata->raw_fifo);
		if (avail < size) {
			spi_cntrs_list[CNTR_ERROR_FULL_RECEIVE_QUEUE].count++;
			ntrig_err("%s: raw receive buffer is full, discarding messages\n",
					__func__);
			do {
				spi_discard_raw_message(privdata);
				avail = kfifo_avail(&privdata->raw_fifo);
			} while (avail < size);
		}
		/* we got here, there is enough room in the buffer, insert the
		 * message */
		kfifo_in(&privdata->raw_fifo, (unsigned char *)&size,
			sizeof(u16));
		kfifo_in(&privdata->raw_fifo, data, size);

		/* wake up any threads blocked on the buffer */
		privdata->raw_data_arrived = 1;
		spi_cntrs_list[CNTR_MAINTREPLY].count++;
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
static int execute_spi_bus_transfer(struct ntrig_spi_privdata *privdata,
	int len)
{
	int res, err;

	privdata->xfer.len = len;

	/*memset(privdata->rx_buf,0,MAX_SPI_TRANSFER_BUFFER_SIZE);*/

	err = spi_sync(privdata->spi, &privdata->msg);
	if (err) {
		ntrig_err("%s: spi_sync failure, bailing out\n", __func__);
		return err;
	}

	g4_set_low_msg_sm_data_packet(&privdata->sm, privdata->rx_buf,
		privdata->xfer.len);
	do {
		res = g4_process_low_sm_data_packet(&privdata->sm);
		ntrig_dbg("%s: process packet returned %d\n", __func__, res);
		if (g4_has_complete_low_message(&privdata->sm))
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

	pr_debug("ntrig_get_irq_value:%d\n", ((gafr_value >> 30) & 0x01));
	return (gafr_value >> 30) & 0x01;
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

	pr_debug("ntrig spi_irq start irq:%d\n", ntrig_get_irq_value());
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
			pr_info("%s: spi_transfer failure %d, bailing out\n",
					__func__, err);

			if (ntrig_get_irq_value() == 0) {
				pr_info("ntrig irq = %d\n",
						ntrig_get_irq_value());
				up(&privdata->spi_lock);
				break;
			} else {
				pr_info("ntrig irq = %d\n",
						ntrig_get_irq_value());
				failCounter++;

				if (failCounter > 5) {
					int err;
					failCounter = 0;
					pr_info("reset to ntrig\n");

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
		sm_idle = g4_is_state_machine_idle(&privdata->sm);
#if 0
		irq_high = gpio_get_value(privdata->irq_gpio);
#else
		irq_high = ntrig_get_irq_value();
#endif
		/*pr_info("%s: state machine %s idle, gpio is %s\n",
		 *	__FUNCTION__,(sm_idle ? "is" : "not"),
		 *	(irq_high ? "high" : "low"));*/
		if (sm_idle && (!irq_high))
			break;
	}
	return IRQ_HANDLED;
}

static void update_spi_counters(struct _ntrig_low_bus_msg *txbuf)
{
	switch (txbuf->msg.channel) {
	case LOWMSG_CHANNEL_DEBUG:
		spi_cntrs_list[CNTR_DEBUG].count++;
		break;
	case LOWMSG_CHANNEL_MAINT:
		spi_cntrs_list[CNTR_MAINT].count++;
		break;
	}
}

/**
 * Write NCP msg to RAW device (on SPI it is the same as HID device though we
 * may use a different channel in the SPI message). If success return number of
 * bytes written (>0). If failed returns <0. The function returns immediately
 * and does not wait for a reply. Replies are buffered and obtained using
 * spi_read_raw, which blocks if there are no replies waiting.
 */
static int spi_write_raw(void *dev, const char *buf, short msg_len)
{
	struct spi_device *spi = dev;
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(spi);
	u8 request;
	int err, len;

	if (msg_len <= 0) {
		ntrig_err("%s: empty message\n", __func__);
		return 0;
	}

	request = *buf;
	ntrig_dbg("%s: request=%d, msg_len=%d\n", __func__, request, msg_len);

	/* critical section: spi bus and state machine */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) {
		/* we were interrupted, cancel the request */
		return -ERESTARTSYS;
	}

	switch (request) {
	case SPI_ENABLED_COMMAND:
	case LOWMSG_REQUEST_DEBUG_AGENT:
	case LOWMSG_REQUEST_NCP_DFU:
	{
		struct _ntrig_low_bus_msg *txbuf =
			(struct _ntrig_low_bus_msg *) (privdata->tx_buf);
		g4_build_ncp_dfu_cmd(request, buf, msg_len, (char *)txbuf);
		update_spi_counters(txbuf);
		len = MAX_SPI_TRANSFER_BUFFER_SIZE;
		err = execute_spi_bus_transfer(privdata, len);
		if (err) {
			ntrig_err("%s: spi transfer failure %d\n", __func__,
				err);
			goto exit_err;
		}
		if (request == LOWMSG_REQUEST_NCP_DFU && buf[6] == 0x07 &&
			buf[7] == 0x01) {
			/* When executing go to bootloader, sensor will be
			 * reset. We must keep the output_enable low for a while
			 * to prevent the sensor from going crazy.
			 */
			ntrig_dbg("%s: go to bootloader, lowering output_enable for a while...\n",
					__func__);
			spi_set_output_enable(privdata, 0);
			msleep(500);
			spi_set_output_enable(privdata, 1);
			ntrig_dbg("%s: go to bootloader, output_enable is back up\n",
					__func__);
		}
		/* clear the txbuf so we don't send this command again by
		 * mistake */
		memset(txbuf, 0xFF, len);
		break;
	}
	default:
		ntrig_err("%s: unsupported command %d\n", __func__, request);
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
static int spi_get_raw_data_from_fifo(struct kfifo *fifo, char *buf,
	size_t count)
{
	u16 data_len;
	int res;
	if (kfifo_len(fifo)) {
		/* we have data, can return immediately */
		res = kfifo_out(fifo, (unsigned char *)&data_len, sizeof(u16));
		ntrig_dbg("%s: raw message size %d, count = %d\n", __func__,
			data_len, count);
		if (data_len > count)
			data_len = count;
		res = kfifo_out(fifo, buf, data_len);
	} else
		data_len = 0;
	return data_len;
}

/**
 * Read NCP msg from RAW device.
 * On SPI, all responses come from the same device. We separate them into HID
 * and RAW based on some fields in the message. On success, return number of
 * bytes read and fill buffer. If failed return <0. Allocate at least
 * MAX_SPI_RESPONSE_SIZE bytes in the buffer.
 * If there is no data in the buffer, it will block until data is received, or
 * until timeout is reached (500ms). Return -1 if no data arrived.
 * Note: buf is kernel memory
 */
static int spi_read_raw(void *dev, char *buf, size_t count)
{
	struct spi_device *spi = dev;
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(spi);
	int err, data_len;

	/* use spi_lock to protect the circular buffer */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) /* we were interrupted, cancel the request */
		return -ERESTARTSYS;

	data_len = spi_get_raw_data_from_fifo(&privdata->raw_fifo, buf, count);
	if (data_len > 0) {
		/* we got data, return immediately */
		up(&privdata->spi_lock);
		return data_len;
	}

	/* buffer is empty, we will need to wait. Release the lock before
	 * waiting */
	privdata->raw_data_arrived = 0;
	up(&privdata->spi_lock);
	err = wait_event_interruptible_timeout(privdata->raw_wait_queue,
		(privdata->raw_data_arrived != 0),
		MAX_SPI_RAW_COMMAND_REPLY_TIMEOUT);
	if (err < 0) /* we were interrupted */
		return -ERESTARTSYS;

	/* get the lock again */
	err = down_interruptible(&privdata->spi_lock);
	if (err != 0) /* we were interrupted, cancel the request */
		return -ERESTARTSYS;

	/* read from fifo again, this time return 0 if there is no data
	 * (timeout) */
	data_len = spi_get_raw_data_from_fifo(&privdata->raw_fifo, buf, count);
	up(&privdata->spi_lock);
	return data_len;
}

/*
 * return the array of struct _ntrig_counter and it's length
 */

int g4_get_counters(struct _ntrig_counter **counters_list_local,  int *length)
{
	*counters_list_local = spi_cntrs_list;
	*length = CNTR_NUMBER_OF_SPI_CNTRS;
	return 0;
}



/**
 * registers the device to the dispatcher driver
 */
static int register_to_dispatcher(struct spi_device *spi)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(tmp_spi_dev);
	struct _ntrig_bus_device *nd;
	struct _ntrig_dev_ncp_func ncp_func;
	int ret, flags;

	if (DTRG_NO_ERROR != g4_allocate_device(&privdata->ntrig_dispatcher)) {
		dev_err(&spi->dev, "cannot allocate N-Trig dispatcher\n");
		return DTRG_FAILED;
	}

	/* Register device in the dispatcher. We register twice - once for HID
	 * and once for RAW (ncp/bulk).
	 * TODO we use a hard-coded bus name of "spi", need to change if we want
	 * to support multiple sensors connected over SPI
	 */
	ncp_func.dev = spi;
	ncp_func.read = spi_read_raw;
	ncp_func.write = spi_write_raw;
	ncp_func.read_counters = g4_get_counters;
	ncp_func.reset_counters = g4_spi_reset_counters;

	privdata->sensor_id = g4_RegNtrigDispatcher(TYPE_BUS_SPI_HID, "spi",
		&ncp_func);
	if (privdata->sensor_id == DTRG_FAILED) {
		ntrig_err("%s: Cannot register device to dispatcher\n",
			__func__);
		return DTRG_FAILED;
	}
	ret = g4_RegNtrigDispatcher(TYPE_BUS_SPI_RAW, "spi", &ncp_func);
	if (ret == DTRG_FAILED) {
		ntrig_err("%s: cannot register raw device to dispatcher\n",
			__func__);
		return DTRG_FAILED;
	}

	/** fill some default values for sensor area
	 *  TODO should be retrieved from sensor, currently	not
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
	g4_create_single_touch(nd, privdata->sensor_id);
	g4_create_multi_touch(nd, privdata->sensor_id);

	/** register to	receive	interrupts when	sensor has data */
	flags = privdata->irq_flags;
	if (flags == 0) {
		/* default flags */
		flags = IRQF_TRIGGER_RISING | IRQF_SHARED;
	}
	/* get the irq */
	ntrig_dbg("%s: requesting irq %d\n", __func__, spi->irq);
	ret = request_threaded_irq(spi->irq, NULL, spi_irq, flags, DRIVER_NAME,
		privdata);
	if (ret) {
		dev_err(&spi->dev, "%s: request_irq(%d) failed\n",
			__func__, privdata->irq_gpio);
		return ret;
	}
	pr_info("ntrig irq_pin:%d state:%d\n",
			privdata->irq_gpio, ntrig_get_irq_value());
	pr_info("ntrig oe_gpio:%d\n", privdata->oe_gpio);
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
	if (pdata->pwr_gpio >= 0) { /* power gpio is present */
		if (enable) {
			pr_debug("ntrig power pin:%d 1\n", pdata->pwr_gpio);
			gpio_set_value(pdata->pwr_gpio, 1);
			msleep(500);
			spi_set_output_enable(pdata, enable);
			enable_irq(pdata->spi->irq);

		} else {
			disable_irq(pdata->spi->irq);
			gpio_set_value(pdata->pwr_gpio, 0);
			pr_debug("ntrig power pin:%d 0\n", pdata->pwr_gpio);
			spi_set_output_enable(pdata, enable);

		}
	}
}

static int init_spi_pwr_gpio(struct ntrig_spi_privdata *pdata)
{
	if (pdata->pwr_gpio >= 0) { /* power gpio is present */
		/* set the pwr gpio line to turn on the sensor */
		int pwr_gpio = pdata->pwr_gpio;
		int err = gpio_request(pwr_gpio, "ntrig_spi_pwr");
		if (err) {
			ntrig_err("%s: fail to request gpio for pwr(%d), err=%d\n",
					__func__, pwr_gpio, err);
			/* continue anyway... */
		}
		err = gpio_direction_output(pwr_gpio, 0); /* low */
		if (err) {
			ntrig_err("%s: fail to change pwr\n", __func__);
			return err;
		}
		msleep(50);
		gpio_set_value(pwr_gpio, 1); /* high */
		pr_info("ntrig set power gpio to high\n");
		msleep(50);
	}
	return 0;
}

/**
 * sysfs data structures
 * USED FOR DEBUGGING ONLY. We create a /sys/spi_test/test file,
 * and use it to control the driver.
 */

/* define the kobj attributes: name, mode, show_function, store_function */
static struct kobj_attribute ntrig_spi_test_attr =
	__ATTR(NULL, S_IRUGO | S_IWUSR, ntrig_spi_test_show,
		ntrig_spi_test_store);

const struct attribute *g4_ntrig_spi_test_attrs = {
	&ntrig_spi_test_attr.attr
};

/***************filter****************/
/*kobj for filter / not filter HB messages from sensor.*/

static ssize_t filter_HB_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t filter_HB_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

/*kobj for showing the status of the interrupt line */
static ssize_t spi_irq_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);
/*************** hid checksum ************/
static ssize_t spi_enable_hid_checksum_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t spi_enable_hid_checksum_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);
static ssize_t spi_bad_hid_checksum_counter_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);
/* attributes */
static struct kobj_attribute ntrig_filter_hb =
	__ATTR(filter_HB, S_IRUGO | S_IWUSR, filter_HB_show, filter_HB_store);

static struct kobj_attribute ntrig_spi_irq =
	__ATTR(spi_irq, S_IRUGO , spi_irq_show, NULL);

static struct kobj_attribute ntrig_enable_hid_checksum =
	__ATTR(enable_hid_checksum, S_IRUGO | S_IWUSR,
		spi_enable_hid_checksum_show, spi_enable_hid_checksum_store);

static struct kobj_attribute ntrig_bad_hid_checksum_counter =
	__ATTR(bad_hid_checksum_counter, S_IRUGO,
		spi_bad_hid_checksum_counter_show, NULL);


/*show / store functions implementation*/
static ssize_t filter_HB_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(tmp_spi_dev);
	/* data is not written into actual file on file system, but rather
	 * saves it in a memory buffer */
	ntrig_dbg("inside %s\n", __func__);
	if (count > 0)
		privdata->filter_HB = buf[0] - '0'; /* cast to int */
	return count;
}

/* reads (shows) data from the sysfs file (user triggered) */
static ssize_t filter_HB_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(tmp_spi_dev);
	ntrig_dbg("inside %s\n", __func__);
	buf[0] = privdata->filter_HB + '0'; /* cast to char */
	buf[1] = 0; /* end of string */
	return 2; /* number of written chars */
}


static ssize_t spi_irq_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(tmp_spi_dev);
	buf[0] =  gpio_get_value(privdata->irq_gpio) + '0'; /* cast to char */
	buf[1] = 0; /* end of string */
	return 2; /* number of written chars */
}

static ssize_t spi_enable_hid_checksum_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (count > 0) {
		check_hid_checksum = (buf[0] == '0') ? 0 : 1;
		ntrig_dbg("%s: %s hid checksum\n", __func__,
			check_hid_checksum ? "enabling" : "disabling");
	} else
		ntrig_err("%s: called with empty buffer\n", __func__);
	return count;
}

static ssize_t spi_enable_hid_checksum_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ntrig_dbg("inside %s: hid checksum is %s\n", __func__,
			check_hid_checksum ? "enabled" : "disabled");
	buf[0] = check_hid_checksum ? '1' : '0';
	buf[1] = 0; /* end of string */
	return 2; /* number of bytes written */
}

static ssize_t spi_bad_hid_checksum_counter_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int res;
	ntrig_dbg("inside %s: bad hid checksum counter = %lu\n",
			__func__, bad_hid_checksum_counter);
	res = snprintf(buf, PAGE_SIZE, "%lu", bad_hid_checksum_counter);
	return res + 1; /* number of bytes written */
}
/****************** end of hid checksum ****************/
static struct attribute *ntrig_spi_attrs[] = {
	&ntrig_filter_hb.attr,
	&ntrig_spi_irq.attr,
	&ntrig_enable_hid_checksum.attr,
	&ntrig_bad_hid_checksum_counter.attr,
	NULL,
};
static struct attribute_group attr_group_ntrig_spi = {
	.attrs = ntrig_spi_attrs
};

/**
 * sysfs functions
 */

static void debug_print_msg(struct _ntrig_low_bus_msg *msg)
{
	const char *buf = (const char *) msg;
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
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) spi_get_drvdata(tmp_spi_dev);
	char ch;
	int err;
	int val;
	/* data is not written into actual file on file system, but rather
	 * saves it in a memory buffer */
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
		ntrig_dbg("%s: gpio line for irq is: %d\n", __func__, irq_gpio);
		err = gpio_request(irq_gpio, "ntrig_spi_irq");
		if (err) {
			ntrig_err("%s: fail to request gpio for irq(%d), err=%d\n",
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
			/** test: turn on or off output_enable (1=turn off
			 *  2=turn on)
			 *  note, if an	inverter is connected, the actual result
			 *  will be the opposite */
			int gpio_index;
			int val = ch - '1';
			gpio_index = privdata->oe_gpio;

			err = gpio_request(gpio_index,
					"ntrig_spi_output_enable");
			if (err) {
				ntrig_err("%s: fail to request gpio for output_enable(%d), err=%d\n",
					__func__, gpio_index, err);
				/* continue anyway... */
			}
			err = gpio_direction_output(gpio_index, val);
			if (err) {
				ntrig_err("%s: fail to change output_enable\n",
					__func__);
				return count;
			}
			ntrig_dbg("%s: success, output_enable(%d) set to %d\n",
					__func__, gpio_index, val);
			return count;
	/* '3' dropped - old "get FW version" command using hid_ncp */
	/* '4' dropped - old "get FW version" command (running many times)
	 *               using hid_ncp */
	} else if (ch == '5') {
		/* SPI data lines test by sending AA pattern */
		struct _ntrig_low_bus_msg *txbuf =
			(struct _ntrig_low_bus_msg *) (privdata->tx_buf);
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
		for (i = 0; i < 10000; i++)
			err = spi_sync(tmp_spi_dev, &privdata->msg);
		if (err) {
			ntrig_err("%s: fail in spi_sync, err=%d\n",
					__func__, err);
			return err;
		}
	} else if (ch == '6') {
		/* PROBE: activate the driver, register to dispatcher. This
		 * emulates the part that should be executed as part of device
		 * startup
		 */
		int err;
		err = register_to_dispatcher(tmp_spi_dev);
		if (err) {
			ntrig_err("%s: register_to_dispatcher failed. result=%d\n",
					__func__, err);
		}
	/* '7' dropped - old "start calibration" command using hid_ncp */
	/* '8' dropped - old "get calibration status" command using hid_ncp */
	} else if (ch == '9') {
		/* print state machine statistics */
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *) (privdata->rx_buf);
		ntrig_dbg("%s: state=%d substate=%d\n", __func__,
			privdata->sm.state, privdata->sm.substate);
		/* last received data */
		debug_print_msg(rmsg);
	} else if (ch == 'a') {
		/* send message which comes completely from sysfs */
		struct _ntrig_low_bus_msg *msg =
			(struct _ntrig_low_bus_msg *) (privdata->tx_buf);
		struct _ntrig_low_bus_msg *rmsg =
			(struct _ntrig_low_bus_msg *) (privdata->rx_buf);
		int err;
		int i;

		memcpy((char *)msg, buf+1, count-1);
		for (i = count - 1; i < 136; i++)
			((char *)msg)[i] = 0xff;
		pr_debug("spi_test_store: writing ");
		for (i = 0; i < 136; i++)
			pr_info("%x ", (int)((unsigned char *) msg)[i]);
		pr_info("\n");
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		if (err) {
			ntrig_err("%s: fail in spi_sync, err=%d\n",
				__func__, err);
			return err;
		}
		/* clear the tx_buf so we don't send unexpected commands to
		 * the sensor */
		for (i = 0; i < privdata->xfer.len; i++)
			privdata->tx_buf[i] = 0xFF;
		debug_print_msg(rmsg);
		/* do another cycle to get response */
		err = spi_sync(tmp_spi_dev, &privdata->msg);
		if (err) {
			ntrig_err("%s: fail in spi_sync, err=%d\n",
				__func__, err);
			return err;
		}
		debug_print_msg(rmsg);
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
		sysfs_remove_file(kobj, g4_ntrig_spi_test_attrs);
		ntrig_dbg("inside %s After call to sysfs_remove_file\n",
			__func__);
	} else {
		ntrig_dbg("inside %s Skip call to sysfs_remove_file\n",
			__func__);
	}

	kobject_put(kobj);
	ntrig_dbg("inside %s After call to kobject_put\n", __func__);

	/* kobj is pointer to properties_kobj which was allocated in
	 * ntrig_create_virtualkeys_file */
	/* kfree(kobj); this might be redundant if we call kobject_put(kobj),
	 * and cause a crahs later */
	ntrig_dbg("inside %s After call to kfree(kobj)\n", __func__);
}

/*
 * This function creates folder "spi_test" under sysfs,
 * and creates file "test" under this folder
 */
#define NTRIG_SPI_TEST_DIRECTORY "spi_test"
#define NTRIG_SPI_TEST_FILE_NAME "test"

static struct kobject *ntrig_create_spi_test_file(void)
{
	int retval;
	int len;
	char *attr_name = NULL;
	struct kobject *properties_kobj;

	ntrig_dbg("inside %s\n", __func__);

	/* 0. Preparations - to deal with case of multiple calls to this
	 * function, with same ntrig dev name */
	/* generate the file (attribute) name, from user data */
	len = sizeof(NTRIG_SPI_TEST_FILE_NAME) / sizeof(char);
	attr_name = kmalloc(len, GFP_KERNEL);
	snprintf(attr_name, len, NTRIG_SPI_TEST_FILE_NAME);
	ntrig_spi_test_attr.attr.name = attr_name;
	ntrig_dbg("inside %s creating new attr_name: %s\n", __func__,
		ntrig_spi_test_attr.attr.name);

	/* 1. Create folder "spi_test" under "sys" */
	/* allocate the memory for the whole object */
	properties_kobj = kobject_create_and_add(NTRIG_SPI_TEST_DIRECTORY,
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

	/* 2. create file "test" under folder "spi_test"  */
	retval = sysfs_create_file(properties_kobj, g4_ntrig_spi_test_attrs);

	if (retval != 0) {
		pr_err("failed to create spi_test/test\n");
		/* TODO: remove file ?? */
		ntrig_dbg("inside %s - sysfs_create_file FAILED\n", __func__);

		/* remove properties_kobj */
		kobject_put(properties_kobj);
		kfree(properties_kobj);
		kfree(attr_name);
		return NULL;
	}

	return properties_kobj;
}

static int __devinit g4_ntrig_spi_probe(struct spi_device *spi);
static int ntrig_spi_remove(struct spi_device *spi);

#ifdef CONFIG_PM
/**
 * called when suspending the device (sleep with preserve
 * memory)
 */
static int ntrig_spi_suspend(struct spi_device *dev, pm_message_t mesg)
{
	struct ntrig_spi_privdata *privdata =
		(struct ntrig_spi_privdata *) dev_get_drvdata(&dev->dev);
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
		(struct ntrig_spi_privdata *) dev_get_drvdata(&dev->dev);
	ntrig_dbg("in %s\n", __func__);
	spi_set_pwr(privdata, true);

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

#endif	/* CONFIG_PM */

static struct spi_driver ntrig_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = ntrig_spi_suspend,
	.resume = ntrig_spi_resume,
	.probe		= g4_ntrig_spi_probe,
	.remove		= __devexit_p(ntrig_spi_remove),
};

/**
 * initialize the SPI driver. Called by board init code
 */
static int __devinit g4_ntrig_spi_probe(struct spi_device *spi)
{
	struct ntrig_spi_privdata *pdata;
	int err, low, high, gpio_index;
	int irq_gpio;
	struct ntrig_spi_platform_data *platdata = spi->dev.platform_data;

	ntrig_dbg("%s: output_enable gpio is %d\n", __func__,
		platdata->oe_gpio);

	pdata = kzalloc(sizeof(struct ntrig_spi_privdata), GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(&spi->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	err = setup_transfer(pdata);
	if (err) {
		ntrig_err("%s: setup_transfer failure\n", __func__);
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

	/* init variables relatedto raw read/write
	 * we use the spi_lock to protect the fifo, no additional lock needed */
	err = kfifo_alloc(&pdata->raw_fifo, RAW_RECEIVE_BUFFER_INITIAL_SIZE,
		GFP_KERNEL);
	if (err) {
		ntrig_err("%s: fail to alloc raw_fifo, err = %d\n",
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
			ntrig_err("%s: fail to request gpio for output_enable(%d), err=%d\n",
				__func__, gpio_index, err);
			/* continue anyway...*/
		}
		low = pdata->oe_inverted ? 1 : 0;
		high = pdata->oe_inverted ? 0 : 1;
		err = gpio_direction_output(gpio_index, low);
		if (err) {
			ntrig_err("%s: fail to change output_enable\n",
				__func__);
			return err;
		}

	}


	/* register the IRQ GPIO line. The actual interrupt is requested
	 * in register_to_dispatcher */
	/*irq_gpio = irq_to_gpio(spi->irq);*/
	irq_gpio = pdata->irq_gpio;
	err = gpio_request(irq_gpio, "ntrig_spi_irq");
	if (err) {
		ntrig_err("%s: fail to request gpio for irq(%d), err=%d\n",
			__func__, irq_gpio, err);
		/* continue anyway... */
	}

	err = gpio_direction_input(irq_gpio);
	if (err) {
		pr_err("%s: fail to set irq input, err = %d\n",
				__func__, err);
	}
	/* register with the dispatcher,
	 * this will also create input event files */
	err = register_to_dispatcher(spi);
	if (err) {
		ntrig_err("%s: fail to register to dispatcher, err = %d\n",
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
	init_spi_pwr_gpio(pdata);

	msleep(50);
	pr_info("ntrig oe_gpio:%d to:%d\n", gpio_index, high);
	gpio_set_value(gpio_index, high);
	msleep(50);

	g4_spi_reset_counters();

	pdata->expected_mt_counter = 0;
	pdata->expected_pen_counter = 0;
	pdata->cur_mt_counter = 0;
	pdata->cur_pen_counter = 0;
	pdata->filter_HB = 0;
	/* success */
	return 0;
}

/**
 * release the SPI driver
 */
static int ntrig_spi_remove(struct spi_device *spi)
{
	struct ntrig_spi_privdata *pdata = dev_get_drvdata(&spi->dev);
	if (!pdata)
		return -EINVAL;

	/* TODO we use a hard-coded bus id - need to change it in order to
	 * support multiple sensors connected over SPI bus */
	g4_UnregNtrigDispatcher(pdata->ntrig_dispatcher, pdata->sensor_id,
		TYPE_BUS_SPI_HID, "spi");
	g4_UnregNtrigDispatcher(pdata->ntrig_dispatcher, pdata->sensor_id,
		TYPE_BUS_SPI_RAW, "spi");

	if (pdata->test_kobj)
		ntrig_spi_test_release(pdata->test_kobj);

	kfree(pdata);
	return 0;
}

static struct kobject *filter_dispatcher_kobj;

static int __init g4_ntrig_spi_init(void)
{
	int ret;

	pr_info("Enter %s\n", __func__);

	/* create n-trig_spi folder under /sys */
	filter_dispatcher_kobj = kobject_create_and_add("n-trig_spi", NULL);
	if (!filter_dispatcher_kobj) {
		pr_err("%s: failed to create dispatcher_kobj", __func__);
		return -ENOMEM;
	}

	/* create sys files under /sys/n-trig_spi */
	ret = sysfs_create_group(filter_dispatcher_kobj, &attr_group_ntrig_spi);
	if (ret) {
		pr_err("%s: failed to create sysfs_group for hid checksum",
				__func__);
		kobject_put(filter_dispatcher_kobj);
	}
	/* hid checksum */
	ret = spi_register_driver(&ntrig_spi_driver);
	pr_info("sakont test 0 %x\n", ret);
	if (ret != 0)
		pr_err("Failed to register N-trig SPI driver: %d\n", ret);
	pr_info("sakont test 1\n");
	return ret;
}
module_init(g4_ntrig_spi_init);

static void __exit g4_ntrig_spi_exit(void)
{
	spi_unregister_driver(&ntrig_spi_driver);
}
module_exit(g4_ntrig_spi_exit);

MODULE_ALIAS("ntrig__g4_spi");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("N-Trig SPI driver");

