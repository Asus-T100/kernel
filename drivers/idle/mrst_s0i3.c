/*
 * mrst_s0i3.c - super-deep sleep state for the Moorestown MID platform
 *
 * Copyright (c) 2011, Intel Corporation.
 * H. Peter Anvin <hpa@linux.intel.com>
 * Len Brown <len.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/clockchips.h>
#include <linux/hrtimer.h>	/* ktime_get_real() */
#include <linux/pci.h>
#include <linux/cpu.h>
#include <trace/events/power.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/sfi.h>
#include <linux/memblock.h>
#include <asm/apic.h>
#include <asm/i387.h>
#include <asm/msr.h>
#include <asm/mtrr.h>
#include <asm/mwait.h>
#include <asm/mrst.h>

static void do_s0i3(void);
static u64 *wakeup_ptr;
static phys_addr_t s0i3_trampoline_phys;
static void *s0i3_trampoline_base;

/**
 * mrst_idle
 * @dev: cpuidle_device
 * @state: cpuidle state
 *
 * This enters S0i3, C6 or C4 depending on what is currently permitted.
 * C1-C4 are handled via the normal intel_idle entry.
 */
int mrst_idle(struct cpuidle_device *dev, struct cpuidle_state *state)
{
	unsigned long ecx = 1; /* break on interrupt flag */
	unsigned long eax = (unsigned long)cpuidle_get_statedata(state);
	ktime_t kt_before, kt_after;
	s64 usec_delta;
	int cpu = smp_processor_id();

	local_irq_disable();

	/*
	 * leave_mm() to avoid costly and often unnecessary wakeups
	 * for flushing the user TLB's associated with the active mm.
	 */
#ifdef CPUIDLE_FLAG_TLB_FLUSHED	 
	if (state->flags & CPUIDLE_FLAG_TLB_FLUSHED)
		leave_mm(cpu);
#endif /* FIXME */
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	kt_before = ktime_get_real();

	stop_critical_timings();

	if (!need_resched()) {
		if (eax == -1UL) {
			do_s0i3();
		} else {
			/* Conventional MWAIT */

			__monitor((void *)&current_thread_info()->flags, 0, 0);
			smp_mb();
			if (!need_resched())
				__mwait(eax, ecx);
		}
	}

	start_critical_timings();

	kt_after = ktime_get_real();
	usec_delta = ktime_to_us(ktime_sub(kt_after, kt_before));

	local_irq_enable();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);

	return usec_delta;
}

/*
 * List of MSRs to be saved/restored, *other* than what is handled by
 * * save_processor_state/restore_processor_state.  * This is
 * specific to Langwell, and any future processors would need a new
 * list.
 *
 * XXX: check how much on this list is actually necessary.
 */
static const u32 s0i3_msr_list[] = {
	/* These are used by index in s0i3_adjust_msrs() */
	MSR_NHM_SNB_PKG_CST_CFG_CTL,

	/* These are not... */
	MSR_IA32_EBL_CR_POWERON,
	MSR_IA32_FEATURE_CONTROL,
	MSR_IA32_PERFCTR0,
	MSR_IA32_PERFCTR1,
	MSR_IA32_MPERF,
	MSR_IA32_THERM_INTERRUPT,
	MSR_CORE_PERF_FIXED_CTR0,
	MSR_CORE_PERF_FIXED_CTR1,
	MSR_CORE_PERF_FIXED_CTR2,
	MSR_IA32_DS_AREA,
	MSR_IA32_CR_PAT,
};

static struct msr s0i3_msr_data[ARRAY_SIZE(s0i3_msr_list)];

static void s0i3_save_msrs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s0i3_msr_list); i++)
		s0i3_msr_data[i].q = native_read_msr(s0i3_msr_list[i]);
}

static void s0i3_adjust_msrs(void)
{
	native_write_msr(MSR_NHM_SNB_PKG_CST_CFG_CTL,
			 s0i3_msr_data[0].l & ~(1 << 25),
			 s0i3_msr_data[0].h);
}

static void s0i3_restore_msrs(void)
{
	int i;

	for (i = ARRAY_SIZE(s0i3_msr_list) - 1; i >= 0; i--)
		native_write_msr(s0i3_msr_list[i],
				 s0i3_msr_data[i].l, s0i3_msr_data[i].h);
}

/*
 * List of APIC registers to be saved/restored.
 * XXX: Verify that this list is actually complete.
 * XXX: Try to figure out a better way to do this using kernel facilities.
 *
 * Note: these are open-coded to minimize delay and therefore improve the
 * power consumption.
 */
static const u32 s0i3_lapic_list[] = {
	APIC_ID,
	APIC_TASKPRI,
	APIC_LDR,
	APIC_DFR,
	APIC_SPIV,
	APIC_LVTT,
	APIC_LVTTHMR,
	APIC_LVTPC,
	APIC_LVT0,
	APIC_LVT1,
	APIC_LVTERR,
	APIC_TMICT,
	APIC_TDCR
};

static u32 s0i3_lapic_data[ARRAY_SIZE(s0i3_lapic_list)];

static void s0i3_save_lapic(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s0i3_lapic_list); i++) {
		volatile u32 *addr = (volatile u32 *)
			(APIC_BASE + s0i3_lapic_list[i]);

		s0i3_lapic_data[i] = readl(addr);
	}
}

static void s0i3_restore_lapic(void)
{
	int i;

	for (i = ARRAY_SIZE(s0i3_lapic_list) - 1; i >= 0; i--) {
		volatile u32 *addr = (volatile u32 *)
			(APIC_BASE + s0i3_lapic_list[i]);

		writel(s0i3_lapic_data[i], addr);
	}
}

/*
 * Leaving S0i3 will have put the other CPU thread into wait for SIPI;
 * we need to put it back into C6 in order to be able to use S0i3
 * again.
 *
 * XXX: this should probably be turned into a
 * mrst_wakeup_secondary_cpu function.
 */
static void s0i3_poke_other_cpu(void)
{
	const struct init_wakeup_delays delays = {
		.assert_init	= 0,
		.icr_accept	= 30,
		.cpu_accept	= 20,
	};

	wakeup_secondary_cpu_via_init_delays(1, s0i3_trampoline_phys, &delays);
}

static inline void s0i3_update_wake_pointer(void)
{
	*wakeup_ptr = virt_to_phys(mrst_s0i3_resume);
}

static noinline void do_s0i3(void)
{
	s0i3_update_wake_pointer();
	mrst_pmu_disable_msi();	/* disable MSIs before save LAPIC */
	s0i3_save_lapic();
	s0i3_save_msrs();
	save_processor_state();
	s0i3_adjust_msrs();
	mrst_pmu_s0i3_prepare();
	if (mrst_pmu_s0i3_entry()) {
		s0i3_restore_msrs();
		restore_processor_state();
		s0i3_restore_lapic();

		/* The PMU command executed correctly, so no longer pending */
		mrst_pmu_pending_set(false);


		s0i3_poke_other_cpu();
	} else {
		/* save_processor_state() did execute kernel_fpu_begin() */
		kernel_fpu_end();
	}
	mrst_pmu_enable_msi();
}

static int s0i3_sfi_parse_wake(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_wake_table_entry *pentry;
	int num;

	sb = (struct sfi_table_simple *)table;
	pentry = (struct sfi_wake_table_entry *)sb->pentry;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_wake_table_entry);

	if (num < 1)		/* num == 1? */
		return -EINVAL;

	wakeup_ptr = ioremap_cache(pentry->phys_addr, 8);

	printk(KERN_DEBUG "s0i3: wakeup pointer at 0x%llx mapped to %p\n",
	       pentry->phys_addr, wakeup_ptr);

	return wakeup_ptr ? 0 : -ENOMEM;
}

/*
 * Reserve memory for the return-to-C6 trampoline.  This is called
 * extremely early in initialization in order to allocate low memory.
 *
 * XXX: Replace this with unified trampoline code.
 */

void __init mrst_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;

	size = s0i3_trampoline_data_end - s0i3_trampoline_data;
	size = ALIGN(size, PAGE_SIZE);

	/* Has to be in very low memory so we can execute real-mode AP code. */
	mem = memblock_find_in_range(0, 1<<20, size, PAGE_SIZE);
	if (mem == MEMBLOCK_ERROR)
		panic("Cannot allocate S0i3 trampoline\n");

	s0i3_trampoline_phys = mem;
	s0i3_trampoline_base = __va(mem);
	memblock_x86_reserve_range(mem, mem + size, "S0I3");
}

static int __init s0i3_prepare(void)
{
	int err;

	wakeup_ptr = NULL;
	err = sfi_table_parse(SFI_SIG_WAKE, NULL, NULL, s0i3_sfi_parse_wake);
	if (err)
		return err;

	/* Set up the return-to-C6 code trampoline in low memory */
	memcpy(s0i3_trampoline_base, s0i3_trampoline_data,
	       s0i3_trampoline_data_end - s0i3_trampoline_data);

	return 0;
}

device_initcall(s0i3_prepare);
