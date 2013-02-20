#ifndef _PSH_IA_COMMON_H_
#define _PSH_IA_COMMON_H_

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>


#ifndef _CMD_ENGINE_H_
enum cmd_id {
	CMD_RESET,
	CMD_SETUP_DDR,
	CMD_GET_SINGLE,
	CMD_CFG_STREAM,
	CMD_STOP_STREAM,
	CMD_ADD_EVENT,
	CMD_CLEAR_EVENT,
	CMD_SELF_TEST,
	CMD_DEBUG,
	CMD_CALIBRATION,
	CMD_UPDATE_DDR,
	CMD_GET_STATUS,
	CMD_SET_PROPERTY,
	CMD_ID_MAX,
};

enum resp_type {
	RESP_CMD_ACK,
	RESP_GET_TIME,
	RESP_GET_SINGLE,
	RESP_STREAMING,
	RESP_DEBUG_MSG,
	RESP_DEBUG_GET_MASK = 5,
	RESP_GYRO_CAL_RESULT,
	RESP_BIST_RESULT,
	RESP_ADD_EVENT,
	RESP_CLEAR_EVENT,
	RESP_EVENT = 10,
	RESP_GET_STATUS,
	RESP_COMP_CAL_RESULT,
};

#define CMD_PARAM_MAX_SIZE ((u16)60)
struct ia_cmd {
	u8 tran_id;
	u8 cmd_id;
	u8 sensor_id;
	char param[CMD_PARAM_MAX_SIZE];
} __packed;

struct cmd_resp {
	u8 tran_id;
	u8 type;
	u8 sensor_id;
	u16 data_len;
	char buf[0];
} __packed;

#define SCMD_DEBUG_SET_MASK ((u16)0x1)
#define SCMD_DEBUG_GET_MASK ((u16)0x2)
struct cmd_debug_param {
	u16 sub_cmd;
	u16 mask_out;
	u16 mask_level;
} __packed;

struct get_status_param {
	u32 snr_bitmask;
} __packed;

struct resp_debug_get_mask {
	u16 mask_out;
	u16 mask_level;
} __packed;

#define LINK_AS_CLIENT		(0)
#define LINK_AS_MONITOR		(1)
#define LINK_AS_REPORTER	(2)
struct link_info {
	u8 id;
	u8 ltype;
	u16 slide;
} __packed;

#define SNR_NAME_MAX_LEN 6
struct snr_info {
	u8 id;
	u8 status;
	u16 freq;
	u16 data_cnt;
	u16 slide;
	u16 priv;
	u16 attri;

	u16 freq_max;
	char name[SNR_NAME_MAX_LEN];

	u8 health;
	u8 link_num;
	struct link_info linfo[0];
} __packed;
#define SNR_INFO_SIZE(sinfo) (sizeof(struct snr_info) \
		+ sinfo->link_num * sizeof(struct link_info))
#define SNR_INFO_MAX_SIZE 256

#define BUF_IA_DDR_SIZE 8192

#endif


#ifndef _SENSOR_DEF_H
struct sensor_cfg {
	u16 sample_freq; /* HZ */
	u16 buff_delay; /* max time(ms) for data bufferring */
	char extra[0];
} __packed;

#define SNR_RUNONLY_BITMASK ((u32)0x1 << 0)

#endif


#ifndef _LOOP_BUFFER_H_
typedef int (*update_finished_f)(u16 offset);

struct loop_buffer {
	int in_reading;
	u8 *addr;
	u16 length;

	u16 off_head;
	u16 off_tail;

	update_finished_f update_finished;
};

#define LBUF_CELL_SIGN ((u16)0x4853)
#define LBUF_DISCARD_SIGN ((u16)0x4944)

struct frame_head {
	u16 sign;
	u16 length;
};

#define LBUF_MAX_CELL_SIZE ((u16)4096)
#define LBUF_MAX_DATA_SIZE (LBUF_MAX_CELL_SIZE \
	- 4 - 2 * sizeof(struct frame_head)\
	- sizeof(struct cmd_resp))

#define size_align(size) ((size % 4) ? (size + 4 - (size % 4)) : size)
#define frame_size(size) (size_align(size) + \
		sizeof(struct frame_head))
#endif

#define PSH2IA_CHANNEL0	0
#define PSH2IA_CHANNEL1	1
#define PSH2IA_CHANNEL2	2
#define PSH2IA_CHANNEL3	3

#define CIRC_SIZE (1024 * 64)

struct psh_ia_priv {
	struct loop_buffer lbuf;	/* loop bufer */
	struct page *pg;
	struct circ_buf circ, circ_dbg;	/* circ buf for sysfs data node */
	struct resp_debug_get_mask dbg_mask;
	struct mutex cmd_mutex;
	struct completion cmpl;
	struct completion get_status_comp;
	struct completion cmd_reset_comp;
	u32 reset_in_progress;
	u32 status_bitmask;
};

/* exports */
void ia_process_lbuf(struct device *dev);
int ia_send_cmd(int ch, struct ia_cmd *cmd, int len);
int psh_ia_common_init(struct device *dev, struct psh_ia_priv **data);
void psh_ia_common_deinit(struct device *dev);


/* imports */
/* need implemented by user */
int do_setup_ddr(struct device *dev);
int process_send_cmd(int ch, struct ia_cmd *cmd, int len);
#endif
