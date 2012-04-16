#ifndef _ASM_X86_INTEL_SCU_IPCUTIL_H_
#define  _ASM_X86_INTEL_SCU_IPCUTIL_H_

/* OSHOB-OS Handoff Buffer read */
int intel_scu_ipc_read_oshob(u8 *data, int len, int offset);
/* OSNIB-OS No Init Buffer write */
#define OSNIB_OFFSET           0x0C
int intel_scu_ipc_write_osnib(u8 *data, int len, int offset, u32 mask);
int intel_scu_ipc_write_osnib_rr(u8 rr);
int intel_scu_ipc_read_osnib_rr(u8 *rr);

#endif
