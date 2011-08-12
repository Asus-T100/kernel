/*
 * mrst.h: Intel Moorestown platform specific setup code
 *
 * (C) Copyright 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_MRST_H
#define _ASM_X86_MRST_H

#include <linux/sfi.h>
#include <linux/pci.h>

extern int pci_mrst_init(void);
extern int __init sfi_parse_mrtc(struct sfi_table_header *table);
extern int sfi_mrtc_num;
extern struct sfi_rtc_table_entry sfi_mrtc_array[];

/*
 * Medfield is the follow-up of Moorestown, it combines two chip solution into
 * one. Other than that it also added always-on and constant tsc and lapic
 * timers. Medfield is the platform name, and the chip name is called Penwell
 * we treat Medfield/Penwell as a variant of Moorestown. Penwell can be
 * identified via MSRs.
 */
enum mrst_cpu_type {
	MRST_CPU_CHIP_LINCROFT = 1,
	MRST_CPU_CHIP_PENWELL,
};

extern enum mrst_cpu_type __mrst_cpu_chip;

#ifdef CONFIG_X86_INTEL_MID

static inline enum mrst_cpu_type mrst_identify_cpu(void)
{
	return __mrst_cpu_chip;
}

#else /* !CONFIG_X86_INTEL_MID */

#define mrst_identify_cpu()    (0)

#endif /* !CONFIG_X86_INTEL_MID */

enum mrst_timer_options {
	MRST_TIMER_DEFAULT,
	MRST_TIMER_APBT_ONLY,
	MRST_TIMER_LAPIC_APBT,
};

extern enum mrst_timer_options mrst_timer_options;

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 */
#define PENWELL_FSB_FREQ_83SKU         83200
#define PENWELL_FSB_FREQ_100SKU        99840

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

extern struct console early_mrst_console;
extern void mrst_early_console_init(void);

extern struct console early_hsu_console;
extern void hsu_early_console_init(void);

extern void intel_scu_devices_create(void);
extern void intel_scu_devices_destroy(void);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void mrst_rtc_init(void);

extern int pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state);

#ifdef CONFIG_X86_MRST
extern int mrst_pmu_s0i3_entry(void);
extern void mrst_pmu_disable_msi(void);
extern u32 mrst_pmu_msi_is_disabled(void);
extern void mrst_pmu_enable_msi(void);
extern void mrst_reserve_memory(void);
#else
static inline void mrst_reserve_memory(void) { }
#endif	/* !CONFIG_X86_MRST */

#include <linux/cpuidle.h>
extern int mrst_s0i3(struct cpuidle_device *dev, struct cpuidle_state *state);
extern void mrst_s0i3_resume(void);
extern int mrst_pmu_invalid_cstates(void);
extern const char s0i3_trampoline_data[], s0i3_trampoline_data_end[];

#endif /* _ASM_X86_MRST_H */
