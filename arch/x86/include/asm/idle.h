#ifndef _ASM_X86_IDLE_H
#define _ASM_X86_IDLE_H

void enter_idle(void);
void exit_idle(void);

void amd_e400_remove_cpu(int cpu);

#endif /* _ASM_X86_IDLE_H */
