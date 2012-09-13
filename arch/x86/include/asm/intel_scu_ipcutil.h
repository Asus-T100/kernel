#ifndef _ASM_X86_INTEL_SCU_IPCUTIL_H_
#define _ASM_X86_INTEL_SCU_IPCUTIL_H_

#include <linux/types.h>

/* ioctl commnds */
#define INTEL_SCU_IPC_REGISTER_READ	0
#define INTEL_SCU_IPC_REGISTER_WRITE	1
#define INTEL_SCU_IPC_REGISTER_UPDATE	2
#define INTEL_SCU_IPC_FW_UPDATE		0xA2
#define INTEL_SCU_IPC_MEDFIELD_FW_UPDATE	0xA3
#define INTEL_SCU_IPC_FW_REVISION_GET	0xB0
#define INTEL_SCU_IPC_READ_RR_FROM_OSNIB	0xC1
#define INTEL_SCU_IPC_WRITE_RR_TO_OSNIB	0xC2
#define INTEL_SCU_IPC_READ_VBATTCRIT	0xC4
#define INTEL_SCU_IPC_WRITE_ALARM_FLAG_TO_OSNIB	0xC5
#define INTEL_SCU_IPC_OSC_CLK_CNTL	0xC6
#define INTEL_SCU_IPC_S0IX_RESIDENCY	0xD1

struct scu_ipc_data {
	__u32	count;  /* No. of registers */
	__u16	addr[5]; /* Register addresses */
	__u8	data[5]; /* Register data */
	__u8	mask; /* Valid for read-modify-write */
};

struct scu_ipc_version {
	__u32	count;  /* length of version info */
	__u8	data[16]; /* version data */
};

struct osc_clk_t {
	__u32	id; /* clock id */
	__u32	khz; /* clock frequency */
};


/* Penwell has 4 osc clocks */
#define OSC_CLK_AUDIO	0	/* Audio */
#define OSC_CLK_CAM0	1	/* Primary camera */
#define OSC_CLK_CAM1	2	/* Secondary camera */
#define OSC_CLK_DISP	3	/* Display buffer */

#ifdef __KERNEL__

int intel_scu_ipc_osc_clk(u8 clk, unsigned int khz);

enum clk0_mode {
	CLK0_AUDIENCE = 0x4,
	CLK0_VIBRA1 = 0x8,
	CLK0_VIBRA2 = 0x10,
	CLK0_MSIC = 0x20,
	CLK0_DEBUG = 0x100,
	CLK0_QUERY = 0x1000,
};

int intel_scu_ipc_set_osc_clk0(unsigned int enable, enum clk0_mode mode);

/* Helpers to turn on/off msic vprog1 and vprog2 */
int intel_scu_ipc_msic_vprog1(int on);
int intel_scu_ipc_msic_vprog2(int on);

/* OSHOB-OS Handoff Buffer read */
int intel_scu_ipc_read_oshob(u8 *data, int len, int offset);
/* OSNIB-OS No Init Buffer write */
#define OSNIB_OFFSET           0x0C
int intel_scu_ipc_write_osnib(u8 *data, int len, int offset);
int intel_scu_ipc_read_osnib(u8 *data, int len, int offset);
int intel_scu_ipc_write_osnib_rr(u8 rr);
int intel_scu_ipc_read_osnib_rr(u8 *rr);

#endif

#endif
