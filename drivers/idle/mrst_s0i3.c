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
#include <linux/debugfs.h>
#include <linux/seq_file.h>
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
#include <asm/intel-mid.h>

void do_s0i3(void);
static u64 *wakeup_ptr;
static phys_addr_t s0i3_trampoline_phys;
static void *s0i3_trampoline_base;

static u64 mrst_s0i3_entry_count;
static u64 mrst_s0i3_exit_count;
static u64 mrst_s0i3_fail_count;

static ktime_t kt_s0i3_enter;
static u64 s0i3_ts_end(ktime_t kt_start,
	u64 *us_delta_minp, u64 *us_delta_maxp, u64 *us_delta_totalp);

static u64 s0i3_entry_us_min;	// debug
static u64 s0i3_entry_us_max;	// debug
static u64 s0i3_entry_us_total;	// debug

static u64 s0i3_poll_msi_disabled_us_min;	// debug
static u64 s0i3_poll_msi_disabled_us_max;	// debug
static u64 s0i3_poll_msi_disabled_us_total;	// debug

static u32 s0i3_poll_msi_disabled_calls;
static u32 s0i3_poll_msi_disabled_cnt;
static u32 s0i3_poll_msi_disabled_max;

static ktime_t s0i3_exit_end_kt;	// debug

static ktime_t s0i3_exit_start_kt; // debug
static ktime_t s0i3_exit_restore_msrs_kt;	// debug
static ktime_t s0i3_exit_restore_processor_state_kt;	// debug
static ktime_t s0i3_exit_restore_lapic_kt;	// debug
static ktime_t s0i3_exit_poke_kt;	// debug


static u64 s0i3_us_min;
static u64 s0i3_us_max;
static u64 s0i3_us_total;

/*
 * s0i3_ts_end()
 * simplify maintaining min, max, average us timestamps
 *
 * call at end of timestamp range
 * take end timestamp, calculate us_delta
 * update min, max, total
 * later: average = total/count
 * return: us_delta
 */
static u64 s0i3_ts_end(ktime_t kt_start,
	u64 *us_delta_minp, u64 *us_delta_maxp, u64 *us_delta_totalp)
{
	u64 us_delta;
	ktime_t kt_now = ktime_get_real();

	us_delta = ktime_to_us(ktime_sub(kt_now, kt_start));

	if (us_delta > *us_delta_maxp)
		*us_delta_maxp = us_delta;

	if (us_delta < *us_delta_minp || *us_delta_minp == 0)
		*us_delta_minp = us_delta;

	*us_delta_totalp += us_delta;

	return us_delta;
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
		.icr_accept	= 0,
		.cpu_accept	= 0,
	};

	wakeup_secondary_cpu_via_init_delays(1, s0i3_trampoline_phys, &delays);
}

static inline void s0i3_update_wake_pointer(void)
{
	*wakeup_ptr = virt_to_phys(mrst_s0i3_resume);
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

/* n.b. match intel_idle.c */
#define CPUIDLE_FLAG_TLB_FLUSHED        0x10000

/**
 * mrst_s0i3
 * @dev: cpuidle_device
 * @state: cpuidle state
 *
 * Enter S0i3.
 */
int mrst_s0i3(struct cpuidle_device *dev, struct cpuidle_state *state)
{
	s64 us_delta;
	int cpu = smp_processor_id();

	local_irq_disable();

	/*
	 * leave_mm() to avoid costly and often unnecessary wakeups
	 * for flushing the user TLB's associated with the active mm.
	 */
	if (state->flags & CPUIDLE_FLAG_TLB_FLUSHED)
		leave_mm(cpu);

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	kt_s0i3_enter = ktime_get_real();

	stop_critical_timings();
	trace_power_start(POWER_CSTATE, 7, 0);

	mrst_s0i3_entry_count++;
	s0i3_update_wake_pointer();
	s0i3_save_lapic();
	s0i3_save_msrs();
	save_processor_state();
	if (mrst_pmu_s0i3_entry()) {

		s0i3_exit_start_kt = ktime_get_real(); // debug

		s0i3_restore_msrs();
		s0i3_exit_restore_msrs_kt = ktime_get_real(); // debug

		restore_processor_state();
		s0i3_exit_restore_processor_state_kt = ktime_get_real(); // debug

		s0i3_restore_lapic();
		s0i3_exit_restore_lapic_kt = ktime_get_real(); // debug

		mrst_s0i3_exit_count++;

		s0i3_poke_other_cpu();	 /* 12 uS */
		s0i3_exit_poke_kt = ktime_get_real();

		mrst_pmu_enable_msi();

		/* exit latency */
		s0i3_exit_end_kt = ktime_get_real();

		/* S0i3 Residency */
		us_delta = s0i3_ts_end(kt_s0i3_enter, &s0i3_us_min,
			&s0i3_us_max, &s0i3_us_total);
	} else {
		mrst_s0i3_fail_count++;
		/* save_processor_state() did execute kernel_fpu_begin() */
		kernel_fpu_end();
		us_delta = 0;	 /* tell cpudile not to add any S0i3-time */
	}

	start_critical_timings();

	local_irq_enable();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);

	return us_delta;
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
#ifdef CONFIG_DEBUG_FS
static u64 *s0i3_us_addr;

/*
 * After we send an S0i3-enter command to the SCU,
 * we poll the SCU's MSI-DISABLED flag to be sure
 * the SCU is ready to receive our MWAIT status.
 *
 * Without this check, it is possible for the SCU to process our MWAIT
 * (on ARC IRQ level2) before our S0i3 command (on ARC IRQ level1),
 * which would cause the SCU to wait for 1ms for our (lost) MWAIT results
 * and bail out with an E501.
 */

/*
 * Since WBINVD preceeds checking for MSI-DISABLED,
 * we typically see MSI-DISABLEd on the 1st read.
 * If do not see MWAIT-DISABLED in 200 reads, we will never see it.
 * For then the SCU has dropped our command, the SCU busy bit is stuck set,
 * and the system will likely die due to inability to send LSS commands
 * to the SCU to un-clock-gate devices.
 */
#define MAX_MSI_DISABLED_POLLS  200

asmlinkage void mrst_s0i3_wait_for_msi_disabled(void)
{
	int reads;
	static ktime_t kt_before; // debug

	s0i3_ts_end(kt_s0i3_enter, &s0i3_entry_us_min,
			&s0i3_entry_us_max, &s0i3_entry_us_total); // debug

	kt_before = ktime_get_real();	// debug

	s0i3_poll_msi_disabled_calls++;	// todo: redundant with entry_count?
	for (reads = 1;  reads <= MAX_MSI_DISABLED_POLLS; ++reads) {
		s0i3_poll_msi_disabled_cnt++;

		if (mrst_pmu_msi_is_disabled()) {
			s0i3_ts_end(kt_before, &s0i3_poll_msi_disabled_us_min,
				&s0i3_poll_msi_disabled_us_max, &s0i3_poll_msi_disabled_us_total); // debug
			return;
		}

		if (reads > s0i3_poll_msi_disabled_max)
			s0i3_poll_msi_disabled_max = reads;
	}
	printk(KERN_EMERG FW_BUG "SCU dropped S0i3 command\n");
}

static int mrst_s0i3_debugfs_show(struct seq_file *s, void *unused)
{
	u64 us_avg;

	seq_printf(s, "entry_count\t%8lld\n", mrst_s0i3_entry_count);
	if (mrst_s0i3_entry_count) {
		us_avg = s0i3_entry_us_total;
		do_div(us_avg, mrst_s0i3_entry_count);
	} else
		us_avg = 0;
	seq_printf(s, "entry_us_max %2lld\n", s0i3_entry_us_max);
	seq_printf(s, "entry_us_avg %2lld\n", us_avg);
	seq_printf(s, "entry_us_min %2lld\n", s0i3_entry_us_min);

	if (s0i3_poll_msi_disabled_calls) {
		us_avg = s0i3_poll_msi_disabled_cnt / s0i3_poll_msi_disabled_calls;
	} else
		us_avg = 0;

	seq_printf(s, "poll_msi_disabled_calls  %8d\n", s0i3_poll_msi_disabled_calls);
	seq_printf(s, "poll_msi_disabled_avg    %8lld\n", us_avg);
	seq_printf(s, "poll_msi_disabled_max    %8d\n", s0i3_poll_msi_disabled_max);

	if (mrst_s0i3_entry_count) {
		us_avg = s0i3_poll_msi_disabled_us_total;
		do_div(us_avg, mrst_s0i3_entry_count);
	} else
		us_avg = 0;
	seq_printf(s, "poll_msi_disabled_us_max %2lld\n", s0i3_poll_msi_disabled_us_max);
	seq_printf(s, "poll_msi_disabled_us_avg %2lld\n", us_avg);
	seq_printf(s, "poll_msi_disabled_us_min %2lld\n", s0i3_poll_msi_disabled_us_min);

	seq_printf(s, "exit_count\t%8lld\n", mrst_s0i3_exit_count);

	seq_printf(s, "exit_us %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_end_kt, s0i3_exit_start_kt)));

	seq_printf(s, " exit_restore_msrs %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_restore_msrs_kt, s0i3_exit_start_kt)));
	seq_printf(s, " exit_processor_restore_state %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_restore_processor_state_kt, s0i3_exit_restore_msrs_kt)));

	seq_printf(s, " exit_restore_lapic %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_restore_lapic_kt, s0i3_exit_restore_processor_state_kt)));
	seq_printf(s, " exit_poke_cpu %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_poke_kt, s0i3_exit_restore_lapic_kt)));
	seq_printf(s, " exit_enable_msi %3lld\n",
		ktime_to_us(ktime_sub(s0i3_exit_end_kt, s0i3_exit_poke_kt)));

	if (mrst_s0i3_exit_count) {
		us_avg = s0i3_us_total;
		do_div(us_avg, mrst_s0i3_exit_count);
	} else
		us_avg = 0;
	seq_printf(s, "Residency us_max %6lld\n", s0i3_us_max);
	seq_printf(s, "Residency us_avg %6lld\n", us_avg);
	seq_printf(s, "Residency us_min %6lld\n", s0i3_us_min);


	seq_printf(s, "fail_count\t%8lld\n", mrst_s0i3_fail_count);

	return 0;
}

static int mrst_s0i3_debugfs_open(struct inode *inode, struct file *file)
{
        return single_open(file, mrst_s0i3_debugfs_show, NULL);
}

static const struct file_operations s0i3_debugfs_fops = {
        .open           = mrst_s0i3_debugfs_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};
#endif

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

#ifdef CONFIG_DEBUG_FS
	/* /sys/kernel/debug/mrst_s0i3_residency */
	s0i3_us_addr = ioremap_nocache((resource_size_t)0xffffeef0, sizeof(u64));
	debugfs_create_u64("mrst_s0i3_residency", S_IFREG | S_IWUSR | S_IRUGO,
		NULL, (u64 *) s0i3_us_addr);

	/* /sys/kernel/debug/mrst_s0i3 */
	debugfs_create_file("mrst_s0i3", S_IFREG | S_IRUGO,
		NULL, NULL, &s0i3_debugfs_fops);
#endif
	return 0;
}

device_initcall(s0i3_prepare);
