/*
 * intel_idle.c - native hardware idle loop for modern Intel processors
 *
 * Copyright (c) 2010, Intel Corporation.
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

/*
 * intel_idle is a cpuidle driver that loads on specific Intel processors
 * in lieu of the legacy ACPI processor_idle driver.  The intent is to
 * make Linux more efficient on these processors, as intel_idle knows
 * more than ACPI, as well as make Linux more immune to ACPI BIOS bugs.
 */

/*
 * Design Assumptions
 *
 * All CPUs have same idle states as boot CPU
 *
 * Chipset BM_STS (bus master status) bit is a NOP
 *	for preventing entry into deep C-stats
 */

/*
 * Known limitations
 *
 * The driver currently initializes for_each_online_cpu() upon modprobe.
 * It it unaware of subsequent processors hot-added to the system.
 * This means that if you boot with maxcpus=n and later online
 * processors above n, those processors will use C1 only.
 *
 * ACPI has a .suspend hack to turn off deep c-statees during suspend
 * to avoid complications with the lapic timer workaround.
 * Have not seen issues with suspend, but may need same workaround here.
 *
 * There is currently no kernel-based automatic probing/loading mechanism
 * if the driver is built as a module.
 */

/* un-comment DEBUG to enable pr_debug() statements */
#define DEBUG

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/clockchips.h>
#include <trace/events/power.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/intel_mid_pm.h>
#include <asm/mwait.h>
#include <asm/msr.h>
#include <asm/intel-mid.h>

#define INTEL_IDLE_VERSION "0.4"
#define PREFIX "intel_idle: "

static struct cpuidle_driver intel_idle_driver = {
	.name = "intel_idle",
	.owner = THIS_MODULE,
	.en_core_tk_irqen = 1,
};
/* intel_idle.max_cstate=0 disables driver */
static int max_cstate = MWAIT_MAX_NUM_CSTATES - 1;

static unsigned int mwait_substates;

#define LAPIC_TIMER_ALWAYS_RELIABLE 0xFFFFFFFF
/* Reliable LAPIC Timer States, bit 1 for C1 etc.  */
static unsigned int lapic_timer_reliable_states = (1 << 1);	 /* Default to only C1 */

static struct cpuidle_device __percpu *intel_idle_cpuidle_devices;
static int intel_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index);

#if defined(CONFIG_INTEL_ATOM_MDFLD_POWER) || \
	defined(CONFIG_INTEL_ATOM_CLV_POWER)
static int soc_s0ix_idle(struct cpuidle_device *dev,
			 struct cpuidle_driver *drv, int index);

static atomic_t nr_cpus_in_c6;
#endif

static struct cpuidle_state *cpuidle_state_table;

/*
 * Hardware C-state auto-demotion may not always be optimal.
 * Indicate which enable bits to clear here.
 */
static unsigned long long auto_demotion_disable_flags;

/*
 * Set this flag for states where the HW flushes the TLB for us
 * and so we don't need cross-calls to keep it consistent.
 * If this flag is set, SW flushes the TLB, so even if the
 * HW doesn't do the flushing, this flag is safe to use.
 */
#define CPUIDLE_FLAG_TLB_FLUSHED	0x10000

/*
 * States are indexed by the cstate number,
 * which is also the index into the MWAIT hint array.
 * Thus C0 is a dummy.
 */
static struct cpuidle_state nehalem_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */ },
	{ /* MWAIT C1 */
		.name = "C1-NHM",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 3,
		.target_residency = 6,
		.enter = &intel_idle },
	{ /* MWAIT C2 */
		.name = "C3-NHM",
		.desc = "MWAIT 0x10",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 20,
		.target_residency = 80,
		.enter = &intel_idle },
	{ /* MWAIT C3 */
		.name = "C6-NHM",
		.desc = "MWAIT 0x20",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 200,
		.target_residency = 800,
		.enter = &intel_idle },
};

static struct cpuidle_state snb_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */ },
	{ /* MWAIT C1 */
		.name = "C1-SNB",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 1,
		.enter = &intel_idle },
	{ /* MWAIT C2 */
		.name = "C3-SNB",
		.desc = "MWAIT 0x10",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 80,
		.target_residency = 211,
		.enter = &intel_idle },
	{ /* MWAIT C3 */
		.name = "C6-SNB",
		.desc = "MWAIT 0x20",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 104,
		.target_residency = 345,
		.enter = &intel_idle },
	{ /* MWAIT C4 */
		.name = "C7-SNB",
		.desc = "MWAIT 0x30",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 109,
		.target_residency = 345,
		.enter = &intel_idle },
};

#ifdef CONFIG_X86_MRFLD
static struct cpuidle_state mrfld_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */ },
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C2 */},
	{ /* MWAIT C3 */ },
	{ /* MWAIT C4 */
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C5 */ },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{ /* MWAIT C7-S0i1 */
		.name = "S0i1-ATM",
		.desc = "MWAIT 0x60",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 1200,
		.target_residency = 4000,
		.enter = &intel_idle },
	{ /* MWAIT C8-S0i2 */
		.name = "S0i2-ATM",
		.desc = "MWAIT 0x62",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 2000,
		.target_residency = 8000,
		.enter = &intel_idle },
	{ /* MWAIT C9-S0i3 */
		.name = "S0i3-ATM",
		.desc = "MWAIT 0x64",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 10000,
		.target_residency = 20000,
		.enter = &intel_idle },
};
#endif

static struct cpuidle_state atom_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */ },
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C2 */
		.name = "C2-ATM",
		.desc = "MWAIT 0x10",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 20,
		.target_residency = 80,
		.enter = &intel_idle },
	{ /* MWAIT C3 */ },
	{ /* MWAIT C4 */
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C5 */ },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
};

/* FIXME: Please populate below with proper values */
static struct cpuidle_state vlv2_atom_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */ },
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C2 */ },
	{ /* MWAIT C3 */ },
	{ /* MWAIT C4 */ },
	{ /* MWAIT C5 */ },
	{ /* MWAIT C6 */ },
};

#ifdef CONFIG_ATOM_SOC_POWER

#ifdef CONFIG_X86_MDFLD
static struct cpuidle_state mfld_cstates[MWAIT_MAX_NUM_CSTATES] = {
	{ /* MWAIT C0 */
		.power_usage = C0_POWER_USAGE },
	{ /* MWAIT C1 */
		.name = "ATM-C1",
		.desc = "MWAIT 0x00",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = CSTATE_EXIT_LATENCY_C1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C2 */
		.name = "ATM-C2",
		.desc = "MWAIT 0x10",
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = CSTATE_EXIT_LATENCY_C2,
		.target_residency = 80,
		.enter = &intel_idle },
	{ /* MWAIT C3 */ },
	{ /* MWAIT C4 */
		.name = "ATM-C4",
		.desc = "MWAIT 0x30",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_C4,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C5 */ },
	{ /* MWAIT C6 */
		.name = "ATM-C6",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_C6,
		.power_usage  = C6_POWER_USAGE,
		.target_residency = 560,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-S0i1",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_S0i1,
		.power_usage  = S0I1_POWER_USAGE,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-LpAudio",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_LPMP3,
		.power_usage  = LPMP3_POWER_USAGE,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-S0i3",
		.desc = "MWAIT 0x52",
		.flags = CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_S0i3,
		.power_usage  = S0I3_POWER_USAGE,
		.enter = &soc_s0ix_idle }
};
#endif

#else /*if !CONFIG_ATOM_SOC_POWER*/

#ifdef CONFIG_X86_MDFLD
#define mfld_cstates atom_cstates
#elif defined(CONFIG_X86_MRFLD)
#define mrfld_cstates atom_cstates
#endif

#endif /*#ifdef CONFIG_ATOM_SOC_POWER*/

static long get_driver_data(int cstate)
{
	int driver_data;
	switch (cstate) {

	case 1:	/* MWAIT C1 */
		driver_data = 0x00;
		break;
	case 2:	/* MWAIT C2 */
		driver_data = 0x10;
		break;
	case 3:	/* MWAIT C3 */
		driver_data = 0x20;
		break;
	case 4:	/* MWAIT C4 */
		driver_data = 0x30;
		break;
	case 5:	/* MWAIT C5 */
		driver_data = 0x40;
		break;
	case 6:	/* MWAIT C6 */
		driver_data = 0x52;
		break;
#ifdef CONFIG_ATOM_SOC_POWER
	case 7: /* S0i1 state */
		driver_data = MID_S0I1_STATE;
		break;
	case 8: /* LPMP3 state */
		driver_data = MID_LPMP3_STATE;
		break;
	case 9: /* S0i1 state */
		driver_data = MID_S0I3_STATE;
		break;
#endif
	default:
		driver_data = 0x00;
	}
	return driver_data;
}

#if defined(CONFIG_INTEL_ATOM_MDFLD_POWER) || \
	defined(CONFIG_INTEL_ATOM_CLV_POWER)
static int enter_s0ix_state(u32 eax, int gov_req_state, int s0ix_state,
					struct cpuidle_device *dev, int index)
{
	int s0ix_entered = 0;
	int selected_state = C6_STATE_IDX;
	int sleep_state;

	if (atomic_add_return(1, &nr_cpus_in_c6) == num_online_cpus() &&
		 s0ix_state) {
		s0ix_entered = mid_s0ix_enter(s0ix_state);
		if (!s0ix_entered) {
			if (pmu_is_s0ix_in_progress()) {
				atomic_dec(&nr_cpus_in_c6);
				eax = C4_HINT;
			}
			pmu_set_s0ix_complete();
		}
	}

	sleep_state = mid_state_to_sys_state(s0ix_entered);

	switch (s0ix_state) {
	case MID_S0I1_STATE:
		trace_cpu_idle(S0I1_STATE_IDX, dev->cpu);
		break;
	case MID_LPMP3_STATE:
		trace_cpu_idle(LPMP3_STATE_IDX, dev->cpu);
		break;
	case MID_S0I3_STATE:
		trace_cpu_idle(S0I3_STATE_IDX, dev->cpu);
		break;
	case MID_S3_STATE:
		trace_cpu_idle(S0I3_STATE_IDX, dev->cpu);
		break;
	default:
		trace_cpu_idle((eax >> 4) + 1, dev->cpu);
	}

	if (sleep_state)
		/* time stamp for end of s0ix entry */
		time_stamp_for_sleep_state_latency(sleep_state, false, true);

	__monitor((void *)&current_thread_info()->flags, 0, 0);
	smp_mb();
	if (!need_resched())
		__mwait(eax, 1);

	if (sleep_state)
		/* time stamp for start of s0ix exit */
		time_stamp_for_sleep_state_latency(sleep_state, true, false);

	if (likely(eax == C6_HINT))
		atomic_dec(&nr_cpus_in_c6);

	/* During s0ix exit inform scu that OS
	 * has exited. In case scu is still waiting
	 * for ack c6 trigger, it would exit out
	 * of the ack-c6 timeout loop
	 */
	pmu_set_s0ix_complete();

	/* In case of demotion to S0i1/lpmp3 update last_state */
	if (s0ix_entered) {
		selected_state = S0I3_STATE_IDX;

		if (s0ix_state == MID_S0I1_STATE) {
			index = S0I1_STATE_IDX;
			selected_state = S0I1_STATE_IDX;
		} else if (s0ix_state == MID_LPMP3_STATE) {
			index = LPMP3_STATE_IDX;
			selected_state = LPMP3_STATE_IDX;
		}
	} else if (eax == C4_HINT) {
		index = C4_STATE_IDX;
		selected_state = C4_STATE_IDX;
	} else
		index = C6_STATE_IDX;

	pmu_s0ix_demotion_stat(gov_req_state, selected_state);

	return index;
}

static int soc_s0ix_idle(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	struct cpuidle_state *state = &drv->states[index];
	struct cpuidle_state_usage *state_usage = &dev->states_usage[index];
	unsigned long eax = (unsigned long)cpuidle_get_statedata(state_usage);
	int cpu = smp_processor_id();
	int s0ix_state   = 0;
	unsigned int cstate;
	int gov_req_state = (int) eax;
	int sleep_state;

	/* Check if s0ix is already in progress,
	 * This is required to demote C6 while S0ix
	 * is in progress
	 */
	if (unlikely(pmu_is_s0ix_in_progress()))
		return intel_idle(dev, drv, C4_STATE_IDX);

	/* check if we need/possible to do s0ix */
	if (eax != C6_HINT)
		s0ix_state = get_target_platform_state(&eax);

	sleep_state = mid_state_to_sys_state(s0ix_state);

	/* time stamp for start of s0ix entry */
	time_stamp_for_sleep_state_latency(sleep_state, true, true);
	/*
	 * leave_mm() to avoid costly and often unnecessary wakeups
	 * for flushing the user TLB's associated with the active mm.
	 */
	if (state->flags & CPUIDLE_FLAG_TLB_FLUSHED)
		leave_mm(cpu);

	cstate = (((eax) >> MWAIT_SUBSTATE_SIZE) & MWAIT_CSTATE_MASK) + 1;

	if (!(lapic_timer_reliable_states & (1 << (cstate))))
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	stop_critical_timings();

	if (!need_resched())
		index = enter_s0ix_state(eax, gov_req_state,
					s0ix_state, dev, index);

	start_critical_timings();

	if (!(lapic_timer_reliable_states & (1 << (cstate))))
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);

	return index;
}
#endif

/**
 * intel_idle
 * @dev: cpuidle_device
 * @drv: cpuidle driver
 * @index: index of cpuidle state
 *
 * Must be called under local_irq_disable().
 */
static int intel_idle(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	unsigned long ecx = 1; /* break on interrupt flag */
	struct cpuidle_state *state = &drv->states[index];
	struct cpuidle_state_usage *state_usage = &dev->states_usage[index];
	unsigned long eax = (unsigned long)cpuidle_get_statedata(state_usage);
	unsigned int cstate;
	int cpu = smp_processor_id();

#if (defined(CONFIG_X86_MRFLD) && defined(CONFIG_PM_DEBUG))
	{
		/* Get Cstate based on ignore table from PMU driver */
		unsigned int ncstate;
		cstate =
		(((eax) >> MWAIT_SUBSTATE_SIZE) & MWAIT_CSTATE_MASK) + 1;
		ncstate = pmu_get_new_cstate(cstate, &index);
		eax	= (unsigned long)get_driver_data((int)ncstate);
	}
#endif
	cstate = (((eax) >> MWAIT_SUBSTATE_SIZE) & MWAIT_CSTATE_MASK) + 1;

	/*
	 * leave_mm() to avoid costly and often unnecessary wakeups
	 * for flushing the user TLB's associated with the active mm.
	 */
	if (state->flags & CPUIDLE_FLAG_TLB_FLUSHED)
		leave_mm(cpu);

	if (!(lapic_timer_reliable_states & (1 << (cstate))))
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	stop_critical_timings();
	if (!need_resched()) {
		__monitor((void *)&current_thread_info()->flags, 0, 0);
		smp_mb();
		if (!need_resched())
			__mwait(eax, ecx);
	}

	start_critical_timings();

	if (!(lapic_timer_reliable_states & (1 << (cstate))))
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);

	return index;
}

static void __setup_broadcast_timer(void *arg)
{
	unsigned long reason = (unsigned long)arg;
	int cpu = smp_processor_id();

	reason = reason ?
		CLOCK_EVT_NOTIFY_BROADCAST_ON : CLOCK_EVT_NOTIFY_BROADCAST_OFF;

	clockevents_notify(reason, &cpu);
}

static int setup_broadcast_cpuhp_notify(struct notifier_block *n,
		unsigned long action, void *hcpu)
{
	int hotcpu = (unsigned long)hcpu;

	switch (action & 0xf) {
	case CPU_ONLINE:
		smp_call_function_single(hotcpu, __setup_broadcast_timer,
			(void *)true, 1);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block setup_broadcast_notifier = {
	.notifier_call = setup_broadcast_cpuhp_notify,
};

static void auto_demotion_disable(void *dummy)
{
	unsigned long long msr_bits;

	rdmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
	msr_bits &= ~auto_demotion_disable_flags;
	wrmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
}

/*
 * intel_idle_probe()
 */
static int intel_idle_probe(void)
{
	unsigned int eax, ebx, ecx;

	if (max_cstate == 0) {
		pr_debug(PREFIX "disabled\n");
		return -EPERM;
	}

	if (boot_cpu_data.x86_vendor != X86_VENDOR_INTEL)
		return -ENODEV;

	if (!boot_cpu_has(X86_FEATURE_MWAIT))
		return -ENODEV;

	if (boot_cpu_data.cpuid_level < CPUID_MWAIT_LEAF)
		return -ENODEV;

	cpuid(CPUID_MWAIT_LEAF, &eax, &ebx, &ecx, &mwait_substates);

	if (!(ecx & CPUID5_ECX_EXTENSIONS_SUPPORTED) ||
	    !(ecx & CPUID5_ECX_INTERRUPT_BREAK) ||
	    !mwait_substates)
			return -ENODEV;

	pr_debug(PREFIX "MWAIT substates: 0x%x\n", mwait_substates);


	if (boot_cpu_data.x86 != 6)	/* family 6 */
		return -ENODEV;

	switch (boot_cpu_data.x86_model) {

	case 0x1A:	/* Core i7, Xeon 5500 series */
	case 0x1E:	/* Core i7 and i5 Processor - Lynnfield Jasper Forest */
	case 0x1F:	/* Core i7 and i5 Processor - Nehalem */
	case 0x2E:	/* Nehalem-EX Xeon */
	case 0x2F:	/* Westmere-EX Xeon */
	case 0x25:	/* Westmere */
	case 0x2C:	/* Westmere */
		cpuidle_state_table = nehalem_cstates;
		auto_demotion_disable_flags =
			(NHM_C1_AUTO_DEMOTE | NHM_C3_AUTO_DEMOTE);
		break;

	case 0x1C:	/* 28 - Atom Processor */
		cpuidle_state_table = atom_cstates;
		break;

	case 0x26:	/* 38 - Lincroft Atom Processor */
		cpuidle_state_table = atom_cstates;
		auto_demotion_disable_flags = ATM_LNC_C6_AUTO_DEMOTE;
		break;
#ifdef CONFIG_X86_MDFLD
	case 0x27:	/* 39 - Penwell Atom Processor */
	case 0x35:	/* 53 - Cloverview Atom Processor */
		cpuidle_state_table = mfld_cstates;
		auto_demotion_disable_flags = ATM_LNC_C6_AUTO_DEMOTE;
		break;
#endif
	 case 0x37:	/* 55 - Valleyview Atom Processor */
		cpuidle_state_table = vlv2_atom_cstates;
		break;
#ifdef CONFIG_X86_MRFLD
	case 0x4a:	/*74 - Tangier Atom Processor */
		cpuidle_state_table = mrfld_cstates;
		break;
#endif
	case 0x2A:	/* SNB */
	case 0x2D:	/* SNB Xeon */
		cpuidle_state_table = snb_cstates;
		break;

	default:
		pr_debug(PREFIX "does not run on family %d model %d\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		return -ENODEV;
	}

	if (boot_cpu_has(X86_FEATURE_ARAT))	/* Always Reliable APIC Timer */
		lapic_timer_reliable_states = LAPIC_TIMER_ALWAYS_RELIABLE;
	else {
		on_each_cpu(__setup_broadcast_timer, (void *)true, 1);
		register_cpu_notifier(&setup_broadcast_notifier);
	}

	pr_debug(PREFIX "v" INTEL_IDLE_VERSION
		" model 0x%X\n", boot_cpu_data.x86_model);

	pr_debug(PREFIX "lapic_timer_reliable_states 0x%x\n",
		lapic_timer_reliable_states);
	return 0;
}

/*
 * intel_idle_cpuidle_devices_uninit()
 * unregister, free cpuidle_devices
 */
static void intel_idle_cpuidle_devices_uninit(void)
{
	int i;
	struct cpuidle_device *dev;

	for_each_online_cpu(i) {
		dev = per_cpu_ptr(intel_idle_cpuidle_devices, i);
		cpuidle_unregister_device(dev);
	}

	free_percpu(intel_idle_cpuidle_devices);
	return;
}

#ifdef CONFIG_ATOM_SOC_POWER
static unsigned int get_target_residency(unsigned int cstate)
{
	unsigned int t_sleep = cpuidle_state_table[cstate].target_residency;
	unsigned int prev_idx;

	/* get the previous lower sleep state */
	if ((cstate == 8) || (cstate == 9))
		prev_idx = cstate - 2;
	else
		prev_idx = cstate - 1;

	/* calculate target_residency only if not defined already */
	if (!t_sleep) {
		unsigned int p_active = cpuidle_state_table[0].power_usage;
		unsigned int prev_state_power = cpuidle_state_table
							[prev_idx].power_usage;
		unsigned int curr_state_power = cpuidle_state_table
							[cstate].power_usage;
		unsigned int prev_state_lat = cpuidle_state_table
							[prev_idx].exit_latency;
		unsigned int curr_state_lat = cpuidle_state_table
							[cstate].exit_latency;

		if (curr_state_power && prev_state_power && p_active &&
		    prev_state_lat && curr_state_lat &&
		    (curr_state_lat > prev_state_lat) &&
		    (prev_state_power > curr_state_power)) {
			t_sleep = (p_active * (curr_state_lat - prev_state_lat)
					+ (prev_state_lat * prev_state_power)
					- (curr_state_lat * curr_state_power)) /
				  (prev_state_power - curr_state_power);

			/* round-up target_residency */
			t_sleep++;
		}
	}

	WARN_ON(!t_sleep);

	pr_debug(PREFIX "cpuidle: target_residency[%d]= %d\n", cstate, t_sleep);

	return t_sleep;
}
#endif

/*
 * intel_idle_cpuidle_driver_init()
 * allocate, initialize cpuidle_states
 */
static int intel_idle_cpuidle_driver_init(void)
{
	int cstate;
	struct cpuidle_driver *drv = &intel_idle_driver;

	drv->state_count = 1;

	for (cstate = 1; cstate < MWAIT_MAX_NUM_CSTATES; ++cstate) {
		int num_substates;

		if (cstate > max_cstate) {
			printk(PREFIX "max_cstate %d reached\n",
				max_cstate);
			break;
		}

		/* does the state exist in CPUID.MWAIT? */
		/* Do not check number of substates for any states above C6
		 * as these are not real C states supported by the CPU, they
		 * are emulated c states for s0ix support. */
		if (cstate < 6) {
			num_substates = (mwait_substates >> ((cstate) * 4))
						& MWAIT_SUBSTATE_MASK;
			if (num_substates == 0)
				continue;
		}

		/* is the state not enabled? */
		if (cpuidle_state_table[cstate].enter == NULL) {
			/* does the driver not know about the state? */
			if (*cpuidle_state_table[cstate].name == '\0')
				pr_debug(PREFIX "unaware of model 0x%x"
					" MWAIT %d please"
					" contact lenb@kernel.org",
				boot_cpu_data.x86_model, cstate);
			continue;
		}

		if ((cstate > 2) &&
		    !boot_cpu_has(X86_FEATURE_NONSTOP_TSC))
			mark_tsc_unstable(
				"TSC halts in idle states deeper than C2");

#ifdef CONFIG_ATOM_SOC_POWER
		/* Calculate target_residency if power_usage is given */
		cpuidle_state_table[cstate].target_residency =
			get_target_residency(cstate);
#endif

		drv->states[drv->state_count] =	/* structure copy */
			cpuidle_state_table[cstate];

		drv->state_count += 1;
	}

	if (auto_demotion_disable_flags)
		on_each_cpu(auto_demotion_disable, NULL, 1);

	return 0;
}

/*
 * intel_idle_cpu_init()
 * allocate, initialize, register cpuidle_devices
 * @cpu: cpu/core to initialize
 */
int intel_idle_cpu_init(int cpu)
{
	int cstate;
	struct cpuidle_device *dev;

	dev = per_cpu_ptr(intel_idle_cpuidle_devices, cpu);

	dev->state_count = 1;

	for (cstate = 1; cstate < MWAIT_MAX_NUM_CSTATES; ++cstate) {
		int num_substates;

		if (cstate > max_cstate) {
			printk(PREFIX "max_cstate %d reached\n",
			       max_cstate);
			break;
		}

		/* does the state exist in CPUID.MWAIT? */
		/* Do not check number of substates for any states above C6
		 * as these are not real C states supported by the CPU, they
		 * are emulated c states for s0ix support. */
		if (cstate < 6) {
			num_substates = (mwait_substates >> ((cstate) * 4))
						& MWAIT_SUBSTATE_MASK;
			if (num_substates == 0)
				continue;
		}

		/* is the state not enabled? */
		if (cpuidle_state_table[cstate].enter == NULL)
			continue;

		dev->states_usage[dev->state_count].driver_data =
			(void *)get_driver_data(cstate);

		dev->state_count += 1;
	}
	dev->cpu = cpu;

	if (cpuidle_register_device(dev)) {
		pr_debug(PREFIX "cpuidle_register_device %d failed!\n", cpu);
		intel_idle_cpuidle_devices_uninit();
		return -EIO;
	}

	if (auto_demotion_disable_flags)
		smp_call_function_single(cpu, auto_demotion_disable, NULL, 1);

	return 0;
}
EXPORT_SYMBOL_GPL(intel_idle_cpu_init);

static int __init intel_idle_init(void)
{
	int retval, i;

	/* Do not load intel_idle at all for now if idle= is passed */
	if (boot_option_idle_override != IDLE_NO_OVERRIDE)
		return -ENODEV;

	retval = intel_idle_probe();
	if (retval)
		return retval;

	intel_idle_cpuidle_driver_init();
	retval = cpuidle_register_driver(&intel_idle_driver);
	if (retval) {
		printk(KERN_DEBUG PREFIX "intel_idle yielding to %s",
			cpuidle_get_driver()->name);
		return retval;
	}

	intel_idle_cpuidle_devices = alloc_percpu(struct cpuidle_device);
	if (intel_idle_cpuidle_devices == NULL)
		return -ENOMEM;

	for_each_online_cpu(i) {
		retval = intel_idle_cpu_init(i);
		if (retval) {
			cpuidle_unregister_driver(&intel_idle_driver);
			return retval;
		}
	}

	return 0;
}

static void __exit intel_idle_exit(void)
{
	intel_idle_cpuidle_devices_uninit();
	cpuidle_unregister_driver(&intel_idle_driver);

	if (lapic_timer_reliable_states != LAPIC_TIMER_ALWAYS_RELIABLE) {
		on_each_cpu(__setup_broadcast_timer, (void *)false, 1);
		unregister_cpu_notifier(&setup_broadcast_notifier);
	}

	return;
}

module_init(intel_idle_init);
module_exit(intel_idle_exit);

module_param(max_cstate, int, 0444);

MODULE_AUTHOR("Len Brown <len.brown@intel.com>");
MODULE_DESCRIPTION("Cpuidle driver for Intel Hardware v" INTEL_IDLE_VERSION);
MODULE_LICENSE("GPL");
