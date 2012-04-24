#ifndef _ASM_X86_INTEL_SCU_IPCUTIL_H_
#define _ASM_X86_INTEL_SCU_IPCUTIL_H_

/* Penwell has 4 osc clocks */
#define OSC_CLK_AUDIO	0	/* Audio */
#define OSC_CLK_CAM0	1	/* Primary camera */
#define OSC_CLK_CAM1	2	/* Secondary camera */
#define OSC_CLK_DISP	3	/* Display buffer */

int intel_scu_ipc_osc_clk(u8 clk, unsigned int khz);

enum clk0_mode {
	CLK0_AUDIENCE = 0x4,
	CLK0_VIBRA1 = 0x8,
	CLK0_VIBRA2 = 0x10,
	CLK0_MSIC = 0x20,
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
int intel_scu_ipc_write_osnib(u8 *data, int len, int offset, u32 mask);
int intel_scu_ipc_write_osnib_rr(u8 rr);
int intel_scu_ipc_read_osnib_rr(u8 *rr);

#endif
