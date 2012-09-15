#ifndef _INTEL_MID_RPMSG_H_
#define _INTEL_MID_RPMSG_H_

#include <asm/scu_ipc_rpmsg.h>
#include <linux/wakelock.h>

#define RPMSG_TX_TIMEOUT   (5 * HZ)

struct rpmsg_instance {
	struct rpmsg_channel *rpdev;
	struct mutex instance_lock;
	struct tx_ipc_msg *tx_msg;
	struct rx_ipc_msg *rx_msg;
	struct mutex rx_lock;
	struct completion reply_arrived;
	struct rpmsg_endpoint *endpoint;
	struct wake_lock wake_lock;
};

extern int rpmsg_send_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub, u8 *in,
						u32 *out, u32 inlen,
						u32 outlen);

extern int rpmsg_send_raw_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub, u8 *in,
						u32 *out, u32 inlen,
						u32 outlen, u32 sptr,
						u32 dptr);

extern int rpmsg_send_simple_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub);

extern int alloc_rpmsg_instance(struct rpmsg_channel *rpdev,
				struct rpmsg_instance **pInstance);

extern void free_rpmsg_instance(struct rpmsg_channel *rpdev,
				struct rpmsg_instance **pInstance);

extern void init_rpmsg_instance(struct rpmsg_instance *instance);

#endif
