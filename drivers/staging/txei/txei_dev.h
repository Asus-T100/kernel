/*
 * Intel Management Engine Interface (Intel TXEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 */

#ifndef _TXEI_DEV_H_
#define _TXEI_DEV_H_

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "txei.h"

#include "hw.h"

/*
 * Number of Maximum TXEI Clients
 */
#define TXEI_CLIENTS_MAX 256

/*
 * Number of File descriptors/handles
 * that can be opened to the driver.
 *
 * Limit to 255: 256 Total Clients
 * minus internal client for TXEI Bus Messags
 */
#define TXEI_MAX_OPEN_HANDLE_COUNT (TXEI_CLIENTS_MAX - 1)


/* File state */
enum file_state {
	TXEI_FILE_INITIALIZING = 0,
	TXEI_FILE_CONNECTING,
	TXEI_FILE_CONNECTED,
	TXEI_FILE_DISCONNECTING,
	TXEI_FILE_DISCONNECTED
};

/* TXEI device states */
enum txei_dev_state {
	TXEI_DEV_INITIALIZING = 0,
	TXEI_DEV_INIT_CLIENTS,
	TXEI_DEV_ENABLED,
	TXEI_DEV_RESETING,
	TXEI_DEV_DISABLED,
	TXEI_DEV_RECOVERING_FROM_RESET,
	TXEI_DEV_POWER_DOWN,
	TXEI_DEV_POWER_UP
};

/* init clients  states*/
enum txei_init_clients_states {
	TXEI_START_MESSAGE = 0,
	TXEI_ENUM_CLIENTS_MESSAGE,
	TXEI_CLIENT_PROPERTIES_MESSAGE
};

enum txei_file_transaction_states {
	TXEI_IDLE,
	TXEI_WRITING,
	TXEI_WRITE_COMPLETE,
	TXEI_FLOW_CONTROL,
	TXEI_READING,
	TXEI_READ_COMPLETE
};

/**
 * enum txei_cb_file_ops  - file operation associated with the callback
 * @TXEI_FOP_READ   - read
 * @TXEI_FOP_WRITE  - write
 * @TXEI_FOP_IOCTL  - ioctl
 * @TXEI_FOP_OPEN   - open
 * @TXEI_FOP_CLOSE  - close
 */
enum txei_cb_file_ops {
	TXEI_FOP_READ = 0,
	TXEI_FOP_WRITE,
	TXEI_FOP_IOCTL,
	TXEI_FOP_OPEN,
	TXEI_FOP_CLOSE
};

/*
 * Intel TXEI message data struct
 */
struct txei_message_data {
	u32 size;
	unsigned char *data;
};

/**
 * struct txei_me_client - representation of me (fw) client
 *
 * @props  - client properties
 * @client_id - me client id
 * @txei_flow_ctrl_creds - flow control credits
 */
struct txei_me_client {
	struct txei_client_properties props;
	u8 client_id;
	u8 txei_flow_ctrl_creds;
};

struct txei_cl_cb {
	struct list_head cb_list;
	enum txei_cb_file_ops fop_type;
	void *file_private;
	struct txei_message_data request_buffer;
	struct txei_message_data response_buffer;
	unsigned long buf_idx;
	unsigned long read_time;
	struct file *file_object;
};

/* TXEI client instance carried as file->pirvate_data*/
struct txei_cl {
	struct list_head link;
	struct txei_device *dev;
	enum file_state state;
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	wait_queue_head_t wait;
	int status;
	/* ID of client connected */
	u8 host_client_id;
	u8 me_client_id;
	u8 txei_flow_ctrl_creds;
	u8 timer_count;
	enum txei_file_transaction_states reading_state;
	enum txei_file_transaction_states writing_state;
	int sm_state;
	struct txei_cl_cb *read_cb;
};

struct txei_io_list {
	struct txei_cl_cb txei_cb;
	int status;
};

struct txei_interrupt_cause {
	bool	IPCOuputDoorbellInt;
	bool	IPCInputReadyInt;
	bool	ReadinessInt;
	bool	AlivenessInt;
	int	OutputDoorbellInt;
};

struct txei_mm_device;

/* TXEI private device struct */
struct txei_device {
	struct pci_dev *pdev;	/* pointer to pci device struct */
	/*
	 * lists of queues
	 */
	 /* array of pointers to aio lists */
	struct txei_io_list read_list;		/* driver read queue */
	struct txei_io_list write_list;		/* driver write queue */
	struct txei_io_list write_waiting_list;	/* write waiting queue */
	struct txei_io_list ctrl_wr_list;	/* managed write IOCTL list */
	struct txei_io_list ctrl_rd_list;	/* managed read IOCTL list */

	/*
	 * list of files
	 */
	struct list_head file_list;
	long open_handle_count;
	/*
	 * memory of device
	 */
	/* FIXME:
	void __iomem *mem_addr[NUM_OF_MEM_BARS];
	*/
	void __iomem *mem_addr[3];
	/*
	 * lock for the device
	 */
	struct mutex device_lock; /* device lock */
	bool recvd_msg;
	/*
	 * the polling thread task
	 */
	struct task_struct *polling_thread;
	bool stop_polling_thread;
	wait_queue_head_t wait_polling_thread_terminated;
	/*
	 * Aliveness state of FW(SeC)
	 */
	u32 aliveness;		    /* Aliveness state of FW(SeC) */
	u32 readiness_state;        /* Readiness state of the FW(SeC) */
	/*
	 * Generated interrupt cause (to be saved for ISR later handling)
	 */
	struct txei_interrupt_cause ipc_interrupt_cause;
	/*
	 * waiting queue for receive aliveness-response from FW
	 */
	wait_queue_head_t wait_aliveness_resp;
	bool recvd_aliv_resp;
	struct delayed_work aliveness_timer;
	unsigned long aliveness_timeout;
	unsigned long aliveness_atime;

	/*
	 * waiting queue for receive readiness-interrupt from FW
	 */
	wait_queue_head_t wait_readiness_int;
	bool recvd_readiness_int;
	/*
	 * waiting queue for receive message from FW
	 */
	wait_queue_head_t wait_recvd_msg;

	/*
	 * txei device  states
	 */
	enum txei_dev_state dev_state;
	enum txei_init_clients_states init_clients_state;
	u16 init_clients_timer;
	bool stop;

	u32 extra_write_index;
	u32 rd_msg_buf[128];	/* used for control messages */
	u32 wr_msg_buf[128];	/* used for control messages */
	u32 ext_msg_buf[8];	/* for control responses */
	u32 rd_msg_hdr;

	struct hbm_version version;

	struct txei_me_client *me_clients;
	/* Note: memory has to be allocated */

	DECLARE_BITMAP(me_clients_map, TXEI_CLIENTS_MAX);
	DECLARE_BITMAP(host_clients_map, TXEI_CLIENTS_MAX);
	u8 me_clients_num;
	u8 me_client_presentation_num;
	u8 me_client_index;
	bool hbuf_is_ready;

	struct txei_mm_device *mdev;
	void *pool_vaddr;
	dma_addr_t pool_paddr;
	size_t pool_size;


	struct dentry *dbgfs_dir;
};

/*
 * txei init function prototypes
 */
struct txei_device *txei_device_init(struct pci_dev *pdev);
void txei_reset(struct txei_device *dev, bool intr_en);
int txei_hw_init(struct txei_device *dev);
int txei_task_initialize_clients(void *data);
int txei_initialize_clients(struct txei_device *dev);
int txei_disconnect_host_client(struct txei_device *dev, struct txei_cl *cl);
void txei_remove_client_from_file_list(struct txei_device *dev,
	u8 host_client_id);
void txei_allocate_me_clients_storage(struct txei_device *dev);


u8 txei_find_me_client_update_filext(struct txei_device *dev,
				struct txei_cl *priv,
				const uuid_le *cguid, u8 client_id);

/*
 * TXEI IO List Functions
 */
void txei_io_list_init(struct txei_io_list *list);
void txei_io_list_flush(struct txei_io_list *list, struct txei_cl *cl);

/*
 * TXEI ME Client Functions
 */

struct txei_cl *txei_cl_allocate(struct txei_device *dev);
void txei_cl_init(struct txei_cl *cl, struct txei_device *dev);
int txei_cl_flush_queues(struct txei_cl *cl);
/**
 * txei_cl_cmp_id - tells if file private data have same id
 *
 * @fe1: private data of 1. file object
 * @fe2: private data of 2. file object
 *
 * returns true  - if ids are the same and not NULL
 */
static inline bool txei_cl_cmp_id(const struct txei_cl *cl1,
				const struct txei_cl *cl2)
{
	return cl1 && cl2 &&
		(cl1->host_client_id == cl2->host_client_id) &&
		(cl1->me_client_id == cl2->me_client_id);
}



/*
 * TXEI Host Client Functions
 */
void txei_host_start_message(struct txei_device *dev);
void txei_host_enum_clients_message(struct txei_device *dev);
void txei_host_client_properties(struct txei_device *dev);

/*
 *  TXEI interrupt functions prototype
 */
irqreturn_t txei_interrupt_quick_handler(int irq, void *dev_id);
irqreturn_t txei_interrupt_thread_handler(int irq, void *dev_id);

/*
 *  TXEI input output function prototype
 */
int txei_ioctl_connect_client(struct file *file,
			struct txei_connect_client_data *data);

int txei_start_read(struct txei_device *dev, struct txei_cl *cl);

void txei_free_cb_private(struct txei_cl_cb *priv_cb);

int txei_find_me_client_index(const struct txei_device *dev, uuid_le cuuid);

/*
 * Register Access Function
 */
size_t txei_hbuf_max_len(const struct txei_device *dev);

#define DEBUG_TRACE

#ifdef DEBUG_TRACE
# define txei_print_hex_dump(__dev, msg, data, len)	\
	print_hex_dump(KERN_DEBUG, msg,			\
		DUMP_PREFIX_OFFSET, 16, 1,		\
		data, len, false)
# define txei_dbg(__dev, fmt, ...) \
	dev_dbg(&__dev->pdev->dev, "%s[%d]: " pr_fmt(fmt), \
		__func__, __LINE__, ##__VA_ARGS__)
#else
# define txei_dbg(__dev, fmt, ...) \
	dev_dbg(&__dev->pdev->dev, "%s[%d]: " pr_fmt(fmt), \
		__func__, __LINE__, ##__VA_ARGS__)
# define txei_print_hex_dump(__dev, msg, data, len)
#endif /* DEBUG_TRACE */

#define txei_warn(__dev, fmt, ...) \
	dev_warn(&__dev->pdev->dev, "Warn: %s[%d]: " pr_fmt(fmt),  \
		__func__, __LINE__, ##__VA_ARGS__)
#define txei_info(__dev, fmt, ...) \
	dev_info(&__dev->pdev->dev, "Info: " pr_fmt(fmt), ##__VA_ARGS__)

#define txei_err(__dev, fmt, ...) \
	dev_err(&__dev->pdev->dev, "Error: " pr_fmt(fmt), ##__VA_ARGS__)

#define TXEI_TIME(dev, name, call) do { \
	unsigned long _x, _y; \
	_x = get_cycles(); \
	call; \
	_y = get_cycles();			\
	txei_dbg(dev, "%s: in %lu cycles ==\n", \
		name,  _y - _x); \
} while (0)

int txei_register(struct device *dev);
void txei_deregister(void);

#endif /* _TXEI_DEV_H_ */
