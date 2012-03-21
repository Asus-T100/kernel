/*
 * dlp_main.h
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

#ifndef _DLP_MAIN_H_
#define _DLP_MAIN_H_

#include <linux/hsi/hsi.h>
#include <linux/hsi/hsi_dlp.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/wait.h>

#include "../dlp_debug.h"

#define DRVNAME				"hsi-dlp"

/* Defaut TX timeout delay (in microseconds) */
#define DLP_HANGUP_DELAY	1000000

/* Defaut HSI TX delay (in microseconds) */
#define DLP_HSI_TX_DELAY		10000

/* Defaut HSI RX delay (in microseconds) */
#define DLP_HSI_RX_DELAY		10000

/* Maximal number of pdu allocation failure prior firing an error message */
#define DLP_PDU_ALLOC_RETRY_MAX_CNT	10

/* Round-up the pdu and header length to a multiple of 4-bytes to align
 * on the HSI 4-byte granularity*/
#define DLP_PDU_LENGTH	(((CONFIG_HSI_DLP_PDU_LENGTH+3)/4)*4)
#define DLP_HEADER_LENGTH	(((CONFIG_HSI_DLP_HEADER_LENGTH+3)/4)*4)
#define DLP_PAYLOAD_LENGTH	(DLP_PDU_LENGTH-DLP_HEADER_LENGTH)
#define DLP_LENGTH_MASK		(roundup_pow_of_two(DLP_PAYLOAD_LENGTH)-1)

/* Initial minimal buffering size (in bytes) */
#define DLP_MIN_TX_BUFFERING	65536
#define DLP_MIN_RX_BUFFERING	65536

/* Compute the TX and RX, FIFO depth from the buffering requirements */
/* For optimal performances the DLP_HSI_TX_CTRL_FIFO size shall be set to 2 at
 * least to allow back-to-back transfers. */
#define DLP_TX_MAX_LEN	((DLP_MIN_TX_BUFFERING+DLP_PAYLOAD_LENGTH-1)/DLP_PAYLOAD_LENGTH)
#define DLP_RX_MAX_LEN	((DLP_MIN_RX_BUFFERING+DLP_PAYLOAD_LENGTH-1)/DLP_PAYLOAD_LENGTH)

#define DLP_HSI_TX_CTRL_FIFO	2
#define DLP_HSI_TX_WAIT_FIFO	max(DLP_TX_MAX_LEN-DLP_HSI_TX_CTRL_FIFO, 1)

#define DLP_HSI_RX_WAIT_FIFO	max(DLP_RX_MAX_LEN/2, 1)
#define DLP_HSI_RX_CTRL_FIFO	max(DLP_RX_MAX_LEN-DLP_HSI_RX_WAIT_FIFO, 1)

/* Tag for detecting buggy pdu sizes (must be greater than the maximum pdu
 * size */
#define DLP_BUGGY_PDU_SIZE	0xFFFFFFFFUL

/* PDU size for TTY channel */
#define DLP_TTY_PDU_LENGTH		4096	/* 1500 Bytes */
#define DLP_TTY_HEADER_LENGTH	16
#define DLP_TTY_PAYLOAD_LENGTH	(DLP_TTY_PDU_LENGTH - DLP_TTY_HEADER_LENGTH)

/* PDU size for NET channels */
#define DLP_NET_PDU_SIZE	4096	/* 15360: 15 KBytes */

/* PDU size for CTRL channel */
#define DLP_CTRL_PDU_SIZE	4	/* 4 Bytes */

/* Alignment params */
#define DLP_PACKET_ALIGN_AP		16
#define DLP_PACKET_ALIGN_CP		16

/* Header space params */
#define DLP_HDR_SPACE_AP		16
#define DLP_HDR_SPACE_CP		16

/* Header signature */
#define DLP_HEADER_SIGNATURE	0xF9A80000

/* header fields */
#define DLP_HDR_DATA_SIZE(sz)	((sz) & 0x3FFFF)
#define DLP_HDR_MORE_DESC		(0x1 << 31)
#define DLP_HDR_NO_MORE_DESC	(0x0 << 31)

#define DLP_HDR_MIDDLE_OF_PACKET (0x0 << 29)
#define DLP_HDR_END_OF_PACKET	 (0x1 << 29)
#define DLP_HDR_START_OF_PACKET	 (0x2 << 29)
#define DLP_HDR_COMPLETE_PACKET	 (0x3 << 29)

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

/*
 * Get a ref to the given channel context
 */
#define DLP_CHANNEL_CTX(hsi_ch) dlp_drv.channels[hsi_ch]

/* RX and TX state machine definitions */
enum {
	IDLE,
	ACTIVE,
	TTY,
};

/* DLP contexts */
enum {
	DLP_CHANNEL_CTRL,	/* HSI Channel 0 */
	DLP_CHANNEL_TTY,	/* HSI Channel 1 */
	DLP_CHANNEL_NET1,	/* HSI Channel 2 */
	DLP_CHANNEL_NET2,	/* HSI Channel 3 */
	DLP_CHANNEL_NET3,	/* HSI Channel 4 */

	DLP_CHANNEL_COUNT
};

/*  */
#define DLP_GLOBAL_STATE_SZ		2
#define DLP_GLOBAL_STATE_MASK	((1<<DLP_GLOBAL_STATE_SZ)-1)

/*
 * HSI transfer complete callback prototype
 */
typedef void (*xfer_complete_cb) (struct hsi_msg * pdu);

/*
 * HSI client start/stop RX callback prototype
 */
typedef void (*hsi_client_cb) (struct hsi_client * cl);

/**
 * struct dlp_xfer_ctx - TX/RX transfer context
 * @wait_pdus: head of the FIFO of TX/RX waiting pdus
 * @wait_len: current length of the TX/RX waiting pdus FIFO
 * @wait_max: maximal length of the TX/RX waiting pdus FIFO
 * @recycled_pdus: head of the FIFO of TX/RX recycled pdus
 * @ctrl_max: maximal count of outstanding TX/RX pdus in the controller
 * @ctrl_len: current count of TX/RX outstanding pdus in the controller
 * @buffered: number of bytes currently buffered in the wait FIFO
 * @room: room available in the wait FIFO with a byte granularity
 * @timer: context of the TX active timeout/RX TTY insert retry timer
 * @lock: spinlock to serialise access to the TX/RX context information
 * @delay: nb of jiffies for the TX active timeout/RX TTY insert retry timer
 * @state: current TX/RX state (global and internal one)
 * @channel: reference to the channel context
 * @ch_num: HSI channel number
 * @payload_len: the fixed (maximal) size of a pdu payload in bytes
 * @all_len: total count of TX/RX pdus (including recycled ones)
 * @increase_pool: context of the increase pool work queue
 * @config: current updated HSI configuration
 * @complete_cb: xfer complete callback
 * @ttype: xfer type (RX/TX)
 * @tty_stats: TTY stats
 */
struct dlp_xfer_ctx {
	struct list_head wait_pdus;
	unsigned int wait_len;
	unsigned int wait_max;

	struct list_head recycled_pdus;

	unsigned int ctrl_max;
	unsigned int ctrl_len;

	int buffered;
	int room;

	struct timer_list timer;
	rwlock_t lock;
	unsigned long delay;
	unsigned int state;

	struct dlp_channel *channel;
	unsigned int payload_len;
	unsigned int all_len;

	struct work_struct increase_pool;
	struct hsi_config config;

	xfer_complete_cb complete_cb;
	unsigned int ttype;

	unsigned int seq_num;

#ifdef CONFIG_HSI_DLP_TTY_STATS
	struct hsi_dlp_stats tty_stats;
#endif
};

/**
 * struct dlp_ctrl_hangup_ctx - Hangup management context
 * @cause: Current cause of the hangup
 * @last_cause: Previous cause of the hangup
 * @timer: TX timeout timner
 * @work: TX timeout deferred work
 */
struct dlp_hangup_ctx {
	unsigned int cause;
	unsigned int last_cause;
	struct timer_list timer;
	struct work_struct work;
};

/**
 * struct dlp_channel - HSI channel context
 * @client: reference to this HSI client
 * @credits: credits value (nb of pdus that can be sent to the modem)
 * @credits_lock: credits lock
 * @pdu_size: the fixed pdu size
 * @ready: the channel is ready (TTY opened, NET IF configure)
 * @controller: reference to the controller bound to this context
 * @hsi_channel: the HSI channel number
 * @credits: the credits value
 * @tx_empty_event: TX empty check event
 * @tx: current TX context
 * @rx: current RX context
 */
struct dlp_channel {
	unsigned int hsi_channel;
	unsigned int credits;
	spinlock_t	 lock;
	unsigned int pdu_size;
	unsigned int ready;

	/* TX/RX contexts */
	wait_queue_head_t tx_empty_event;
	struct dlp_xfer_ctx tx;
	struct dlp_xfer_ctx rx;

	/* Hangup management */
	struct dlp_hangup_ctx hangup;

	/* Reset & Coredump callbacks */
	void (*modem_coredump_cb) (struct dlp_channel * ch_ctx);
	void (*modem_reset_cb) (struct dlp_channel * ch_ctx);

	/* Credits callback */
	void (*credits_available_cb) (struct dlp_channel * ch_ctx);

	/* Channel sepecific data */
	void *ch_data;
};

/**
 * struct dlp_driver - fixed pdu length protocol on HSI driver data
 * @channels: array of DLP Channel contex references
 * @is_dma_capable: a flag to check if the ctrl supports the DMA
 * @controller: a reference to the HSI controller
 * @channels: a reference to the HSI client
 * @recycle_wq: Workqueue for submitting pdu-recycling background tasks
 * @tx_hangup_wq: Workqueue for submitting tx timeout hangup background tasks
 * @modem_ready: The modem is up & running
 * @lock: Used for modem ready flag lock
 * @ipc_tx_cfg: HSI client configuration (Used for IPC TX)
 * @ipc_xx_cfg: HSI client configuration (Used for IPC RX)
 * @flash_tx_cfg: HSI client configuration (Used for Boot/Flashing TX)
 * @flash_rx_cfg: HSI client configuration (Used for Boot/Flashing RX)
 *
 * @debug: Debug variable
 */
struct dlp_driver {
	struct dlp_channel *channels[DLP_CHANNEL_COUNT];

	unsigned int is_dma_capable;
	struct hsi_client *client;
	struct device *controller;

	/* Workqueue for tty buffer forwarding */
	struct workqueue_struct *forwarding_wq;

	struct workqueue_struct *recycle_wq;
	struct workqueue_struct *tx_hangup_wq;

	/* Modem readiness */
	int modem_ready;
	spinlock_t lock;

	/* Modem boot/flashing */
	struct hsi_config ipc_tx_cfg;
	struct hsi_config ipc_rx_cfg;

	struct hsi_config flash_tx_cfg;
	struct hsi_config flash_rx_cfg;

	/* Debug variables */
	int debug;
};

/*
 * Context alloc/free function proptype
 */
typedef struct dlp_channel *(*dlp_context_create) (unsigned int index,
						   struct device * dev);

typedef int (*dlp_context_delete) (struct dlp_channel * ch_ctx);

/****************************************************************************
 *
 * PDU handling
 *
 ***************************************************************************/
void *dlp_buffer_alloc(unsigned int buff_size, dma_addr_t * dma_addr);

void dlp_buffer_free(void *buff, dma_addr_t dma_addr, unsigned int buff_size);

void dlp_pdu_dump(struct hsi_msg *pdu, int as_string);

struct hsi_msg *dlp_pdu_alloc(unsigned int hsi_channel,
			      int ttype,
			      int buffer_size,
			      int nb_entries,
			      void *user_data,
			      xfer_complete_cb complete_cb,
			      xfer_complete_cb destruct_cb);

void dlp_pdu_free(struct hsi_msg *pdu, unsigned int pdu_size);

void dlp_pdu_delete(struct dlp_xfer_ctx *xfer_ctx, struct hsi_msg *pdu);

void dlp_pdu_recycle(struct dlp_xfer_ctx *xfer_ctx, struct hsi_msg *pdu);

void dlp_pdu_update(struct dlp_channel *ch_ctx, struct hsi_msg *pdu);

inline void dlp_pdu_reset(struct dlp_xfer_ctx *xfer_ctx,
			  struct hsi_msg *pdu, unsigned int length);

int dlp_pdu_header_valid(struct hsi_msg *pdu);

inline void dlp_pdu_set_length(struct hsi_msg *pdu, u32 sz);

unsigned int dlp_pdu_get_offset(struct hsi_msg *pdu);

inline unsigned int dlp_pdu_room_in(struct hsi_msg *pdu);

inline __attribute_const__
    unsigned char *dlp_pdu_data_ptr(struct hsi_msg *pdu, unsigned int offset);

/****************************************************************************
 *
 * State handling
 *
 ***************************************************************************/
inline __must_check
    unsigned int dlp_ctx_get_state(struct dlp_xfer_ctx *xfer_ctx);

inline void dlp_ctx_set_state(struct dlp_xfer_ctx *xfer_ctx,
			      unsigned int state);

inline __must_check int dlp_ctx_has_flag(struct dlp_xfer_ctx *xfer_ctx,
					 unsigned int flag);

inline void dlp_ctx_set_flag(struct dlp_xfer_ctx *xfer_ctx, unsigned int flag);

inline void dlp_ctx_clear_flag(struct dlp_xfer_ctx *xfer_ctx,
			       unsigned int flag);

inline int dlp_ctx_is_empty(struct dlp_xfer_ctx *xfer_ctx);

int dlp_ctx_have_credits(struct dlp_xfer_ctx *xfer_ctx,
			 struct dlp_channel *ch_ctx);

int dlp_ctx_is_empty_safe(struct dlp_xfer_ctx *xfer_ctx);

void dlp_ctx_update_status(struct dlp_xfer_ctx *xfer_ctx);

/****************************************************************************
 *
 * Generic FIFO handling
 *
 ***************************************************************************/
inline __must_check
    struct hsi_msg *dlp_fifo_tail(struct dlp_xfer_ctx *xfer_ctx,
				  struct list_head *fifo);

inline void _dlp_fifo_pdu_push(struct hsi_msg *pdu, struct list_head *fifo);

inline void _dlp_fifo_pdu_push_back(struct hsi_msg *pdu,
				    struct list_head *fifo);

/****************************************************************************
 *
 * Wait FIFO handling
 *
 ***************************************************************************/
struct hsi_msg *dlp_fifo_wait_pop(struct dlp_xfer_ctx *xfer_ctx);

inline void dlp_fifo_wait_push(struct dlp_xfer_ctx *xfer_ctx,
			       struct hsi_msg *pdu);

inline void dlp_fifo_wait_push_back(struct dlp_xfer_ctx *xfer_ctx,
				    struct hsi_msg *pdu);

void dlp_pop_wait_push_ctrl(struct dlp_xfer_ctx *xfer_ctx,
			    unsigned int check_pdu);

/****************************************************************************
 *
 * Frame recycling handling
 *
 ***************************************************************************/
inline struct hsi_msg *dlp_fifo_recycled_pop(struct dlp_xfer_ctx *xfer_ctx);

__must_check int dlp_pop_recycled_push_ctrl(struct dlp_xfer_ctx *xfer_ctx);

/****************************************************************************
 *
 * HSI Controller
 *
 ***************************************************************************/

inline void dlp_hsi_controller_pop(struct dlp_xfer_ctx *xfer_ctx);

int dlp_hsi_controller_push(struct dlp_xfer_ctx *xfer_ctx,
		struct hsi_msg *pdu);

void dlp_hsi_start_tx(struct dlp_xfer_ctx *xfer_ctx);

void dlp_stop_tx(struct dlp_xfer_ctx *xfer_ctx);

inline void dlp_stop_rx(struct dlp_xfer_ctx *xfer_ctx,
			struct dlp_channel *ch_ctx);

int dlp_hsi_port_claim(void);

inline void dlp_hsi_port_unclaim(void);

void dlp_save_rx_callbacks(hsi_client_cb *start_rx_cb,
		hsi_client_cb *stop_rx_cb);

void dlp_restore_rx_callbacks(hsi_client_cb *start_rx_cb,
		hsi_client_cb *stop_rx_cb);

/****************************************************************************
 *
 * Hangup/Reset management
 *
 ***************************************************************************/

void dlp_hangup_ctx_init(struct dlp_channel *ch_ctx,
		void (* work_func)(struct work_struct *work),
		void (* timeout_func)(unsigned long int param),
		void *data);

void dlp_hangup_ctx_deinit(struct dlp_channel *ch_ctx);

/****************************************************************************
 *
 * RX/TX xfer contexts
 *
 ***************************************************************************/
void dlp_xfer_ctx_init(struct dlp_channel *ch_ctx,
		       struct dlp_xfer_ctx *xfer_ctx,
		       unsigned int delay,
		       unsigned int wait_max,
		       unsigned int ctrl_max,
		       xfer_complete_cb complete_cb, unsigned int ttype);

void dlp_xfer_ctx_clear(struct dlp_xfer_ctx *xfer_ctx);

/****************************************************************************
 *
 * Time handling
 *
 ***************************************************************************/
inline unsigned long from_usecs(const unsigned long delay);

/****************************************************************************
 *
 * CONTROL channel exported functions
 *
 ***************************************************************************/
struct dlp_channel *dlp_ctrl_ctx_create(unsigned int index,
		struct device *dev);

int dlp_ctrl_ctx_delete(struct dlp_channel *ch_ctx);

void dlp_ctrl_modem_reset(struct dlp_channel *ch_ctx);

inline int dlp_ctrl_get_reset_ongoing(void);

inline void dlp_ctrl_set_reset_ongoing(int ongoing);

inline int dlp_ctrl_get_hangup_reasons(void);

inline void dlp_ctrl_set_hangup_reasons(unsigned int hsi_channel,
		int hangup_reasons);

inline unsigned int dlp_ctrl_modem_is_ready(void);

int dlp_ctrl_open_channel(struct dlp_channel *ch_ctx);

int dlp_ctrl_close_channel(struct dlp_channel *ch_ctx);

/****************************************************************************
 *
 * TTY channel exported functions
 *
 ***************************************************************************/
struct dlp_channel *dlp_tty_ctx_create(unsigned int index, struct device *dev);

int dlp_tty_ctx_delete(struct dlp_channel *ch_ctx);

/****************************************************************************
 *
 * NETWORK channels exported functions
 *
 ***************************************************************************/
struct dlp_channel *dlp_net_ctx_create(unsigned int index, struct device *dev);

int dlp_net_ctx_delete(struct dlp_channel *ch_ctx);

/****************************************************************************
 *
 * Global variables
 *
 ***************************************************************************/
extern struct dlp_driver dlp_drv;

#endif /* _DLP_MAIN_H_ */
