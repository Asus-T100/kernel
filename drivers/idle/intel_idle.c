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
#include <linux/intel_mid_pm.h>
#include <linux/pm_qos.h>
#include <asm/cpu_device_id.h>
#include <asm/mwait.h>
#include <asm/msr.h>
#include <asm/io_apic.h>
#include <asm/hypervisor.h>
#include <asm/xen/hypercall.h>


#define INTEL_IDLE_VERSION "0.4"
#define PREFIX "intel_idle: "

#define CLPU_CR_C6_POLICY_CONFIG	0x668
#define CLPU_MD_C6_POLICY_CONFIG	0x669
#define DISABLE_CORE_C6_DEMOTION	0x0
#define DISABLE_MODULE_C6_DEMOTION	0x0

#ifdef CONFIG_MOOREFIELD
#define S0I1_DISPLAY_MODE		(1 << 8)
#define PUNIT_PORT			0x04
#define DSP_SS_PM			0x36
#define S0i1_LATENCY			1200
#define LOW_LATENCY_S0I1		1000
#define S0I1_STATE			0x60
#endif

static struct cpuidle_driver intel_idle_driver = {
	.name = "intel_idle",
	.owner = THIS_MODULE,
};
/* intel_idle.max_cstate=0 disables driver */
static int max_cstate = CPUIDLE_STATE_MAX - 1;

static unsigned int mwait_substates;

#define LAPIC_TIMER_ALWAYS_RELIABLE 0xFFFFFFFF
/* Reliable LAPIC Timer States, bit 1 for C1 etc.  */
static unsigned int lapic_timer_reliable_states = (1 << 1);	 /* Default to only C1 */

#if defined(CONFIG_REMOVEME_INTEL_ATOM_MDFLD_POWER) || \
	defined(CONFIG_REMOVEME_INTEL_ATOM_CLV_POWER)
static int soc_s0ix_idle(struct cpuidle_device *dev,
			 struct cpuidle_driver *drv, int index);
static atomic_t nr_cpus_in_c6;
#endif

struct idle_cpu {
	struct cpuidle_state *state_table;

	/*
	 * Hardware C-state auto-demotion may not always be optimal.
	 * Indicate which enable bits to clear here.
	 */
	unsigned long auto_demotion_disable_flags;
	bool disable_promotion_to_c1e;
};

static const struct idle_cpu *icpu;
static struct cpuidle_device __percpu *intel_idle_cpuidle_devices;
static int intel_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index);
static int intel_idle_cpu_init(int cpu);

static struct cpuidle_state *cpuidle_state_table;

/*
 * Set this flag for states where the HW flushes the TLB for us
 * and so we don't need cross-calls to keep it consistent.
 * If this flag is set, SW flushes the TLB, so even if the
 * HW doesn't do the flushing, this flag is safe to use.
 */
#define CPUIDLE_FLAG_TLB_FLUSHED	0x10000

/*
 * MWAIT takes an 8-bit "hint" in EAX "suggesting"
 * the C-state (top nibble) and sub-state (bottom nibble)
 * 0x00 means "MWAIT(C1)", 0x10 means "MWAIT(C2)" etc.
 *
 * We store the hint at the top of our "flags" for each state.
 */
#define flg2MWAIT(flags) (((flags) >> 24) & 0xFF)
#define MWAIT2flg(eax) ((eax & 0xFF) << 24)

/*
 * States are indexed by the cstate number,
 * which is also the index into the MWAIT hint array.
 * Thus C0 is a dummy.
 */
static struct cpuidle_state nehalem_cstates[CPUIDLE_STATE_MAX] = {
	{
		.name = "C1-NHM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 3,
		.target_residency = 6,
		.enter = &intel_idle },
	{
		.name = "C1E-NHM",
		.desc = "MWAIT 0x01",
		.flags = MWAIT2flg(0x01) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 10,
		.target_residency = 20,
		.enter = &intel_idle },
	{
		.name = "C3-NHM",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 20,
		.target_residency = 80,
		.enter = &intel_idle },
	{
		.name = "C6-NHM",
		.desc = "MWAIT 0x20",
		.flags = MWAIT2flg(0x20) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 200,
		.target_residency = 800,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state snb_cstates[CPUIDLE_STATE_MAX] = {
	{
		.name = "C1-SNB",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 2,
		.target_residency = 2,
		.enter = &intel_idle },
	{
		.name = "C1E-SNB",
		.desc = "MWAIT 0x01",
		.flags = MWAIT2flg(0x01) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 10,
		.target_residency = 20,
		.enter = &intel_idle },
	{
		.name = "C3-SNB",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 80,
		.target_residency = 211,
		.enter = &intel_idle },
	{
		.name = "C6-SNB",
		.desc = "MWAIT 0x20",
		.flags = MWAIT2flg(0x20) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 104,
		.target_residency = 345,
		.enter = &intel_idle },
	{
		.name = "C7-SNB",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 109,
		.target_residency = 345,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state ivb_cstates[CPUIDLE_STATE_MAX] = {
	{
		.name = "C1-IVB",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 1,
		.enter = &intel_idle },
	{
		.name = "C1E-IVB",
		.desc = "MWAIT 0x01",
		.flags = MWAIT2flg(0x01) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 10,
		.target_residency = 20,
		.enter = &intel_idle },
	{
		.name = "C3-IVB",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 59,
		.target_residency = 156,
		.enter = &intel_idle },
	{
		.name = "C6-IVB",
		.desc = "MWAIT 0x20",
		.flags = MWAIT2flg(0x20) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 80,
		.target_residency = 300,
		.enter = &intel_idle },
	{
		.name = "C7-IVB",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 87,
		.target_residency = 300,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state hsw_cstates[CPUIDLE_STATE_MAX] = {
	{
		.name = "C1-HSW",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 2,
		.target_residency = 2,
		.enter = &intel_idle },
	{
		.name = "C1E-HSW",
		.desc = "MWAIT 0x01",
		.flags = MWAIT2flg(0x01) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 10,
		.target_residency = 20,
		.enter = &intel_idle },
	{
		.name = "C3-HSW",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 33,
		.target_residency = 100,
		.enter = &intel_idle },
	{
		.name = "C6-HSW",
		.desc = "MWAIT 0x20",
		.flags = MWAIT2flg(0x20) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 133,
		.target_residency = 400,
		.enter = &intel_idle },
	{
		.name = "C7s-HSW",
		.desc = "MWAIT 0x32",
		.flags = MWAIT2flg(0x32) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 166,
		.target_residency = 500,
		.enter = &intel_idle },
	{
		.name = "C8-HSW",
		.desc = "MWAIT 0x40",
		.flags = MWAIT2flg(0x40) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 300,
		.target_residency = 900,
		.enter = &intel_idle },
	{
		.name = "C9-HSW",
		.desc = "MWAIT 0x50",
		.flags = MWAIT2flg(0x50) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 600,
		.target_residency = 1800,
		.enter = &intel_idle },
	{
		.name = "C10-HSW",
		.desc = "MWAIT 0x60",
		.flags = MWAIT2flg(0x60) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 2600,
		.target_residency = 7700,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state atom_cstates[CPUIDLE_STATE_MAX] = {
	{
		.name = "C1E-ATM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 10,
		.target_residency = 20,
		.enter = &intel_idle },
	{
		.name = "C2-ATM",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 20,
		.target_residency = 80,
		.enter = &intel_idle },
	{
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state vlv_cstates[CPUIDLE_STATE_MAX] = {
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C4 */
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{ /* MWAIT C7-S0i1 */
		.name = "S0i1-ATM",
		.desc = "MWAIT 0x60",
		.flags = MWAIT2flg(0x60) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 1200,
		.target_residency = 4000,
		.enter = &intel_idle },
	{ /* MWAIT C9-S0i3 */
		.name = "S0i3-ATM",
		.desc = "MWAIT 0x64",
		.flags = MWAIT2flg(0x64) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 10000,
		.target_residency = 20000,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

static struct cpuidle_state chv_cstates[CPUIDLE_STATE_MAX] = {
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C4 */
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID
						| CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID
						| CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{ /* MWAIT C7-S0i1 */
		.name = "S0i1-ATM",
		.desc = "MWAIT 0x60",
		.flags = MWAIT2flg(0x60) | CPUIDLE_FLAG_TIME_VALID
						| CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 1200,
		.target_residency = 4000,
		.enter = &intel_idle },
	{ /* MWAIT C8-S0i2 */
		.name = "S0i2-ATM",
		.desc = "MWAIT 0x62",
		.flags = MWAIT2flg(0x62) | CPUIDLE_FLAG_TIME_VALID
						| CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 2000,
		.target_residency = 8000,
		.enter = &intel_idle },
	{ /* MWAIT C9-S0i3 */
		.name = "S0i3-ATM",
		.desc = "MWAIT 0x64",
		.flags = MWAIT2flg(0x64) | CPUIDLE_FLAG_TIME_VALID
						| CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 10000,
		.target_residency = 20000,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

#if defined(CONFIG_REMOVEME_INTEL_ATOM_MRFLD_POWER)
static struct cpuidle_state mrfld_cstates[CPUIDLE_STATE_MAX] = {
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C4 */
		.name = "C4-ATM",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 100,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{ /* MWAIT C7-S0i1 */
		.name = "S0i1-ATM",
		.desc = "MWAIT 0x60",
		.flags = MWAIT2flg(0x60) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 1200,
		.target_residency = 4000,
		.enter = &intel_idle },
	{ /* MWAIT C9-S0i3 */
		.name = "S0i3-ATM",
		.desc = "MWAIT 0x64",
		.flags = MWAIT2flg(0x64) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 10000,
		.target_residency = 20000,
		.enter = &intel_idle },
	{
		.enter = NULL }
};
#else
#define mrfld_cstates atom_cstates
#endif

static struct cpuidle_state moorfld_cstates[CPUIDLE_STATE_MAX] = {
	{ /* MWAIT C1 */
		.name = "C1-ATM",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = 1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C6 */
		.name = "C6-ATM",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID
					 | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 140,
		.target_residency = 560,
		.enter = &intel_idle },
	{ /* MWAIT C7-S0i1 */
		.name = "S0i1-ATM",
		.desc = "MWAIT 0x60",
		.flags = MWAIT2flg(0x60) | CPUIDLE_FLAG_TIME_VALID
					 | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 1200,
		.target_residency = 4000,
		.enter = &intel_idle },
	{ /* MWAIT C9-S0i3 */
		.name = "S0i3-ATM",
		.desc = "MWAIT 0x64",
		.flags = MWAIT2flg(0x64) | CPUIDLE_FLAG_TIME_VALID
					 | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = 10000,
		.target_residency = 20000,
		.enter = &intel_idle },
	{
		.enter = NULL }
};

#if defined(CONFIG_REMOVEME_INTEL_ATOM_MDFLD_POWER) || \
	defined(CONFIG_REMOVEME_INTEL_ATOM_CLV_POWER)

static struct cpuidle_state mfld_cstates[CPUIDLE_STATE_MAX] = {
	{ /* MWAIT C1 */
		.name = "ATM-C1",
		.desc = "MWAIT 0x00",
		.flags = MWAIT2flg(0x00) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = CSTATE_EXIT_LATENCY_C1,
		.target_residency = 4,
		.enter = &intel_idle },
	{ /* MWAIT C2 */
		.name = "ATM-C2",
		.desc = "MWAIT 0x10",
		.flags = MWAIT2flg(0x10) | CPUIDLE_FLAG_TIME_VALID,
		.exit_latency = CSTATE_EXIT_LATENCY_C2,
		.target_residency = 80,
		.enter = &intel_idle },
	{ /* MWAIT C4 */
		.name = "ATM-C4",
		.desc = "MWAIT 0x30",
		.flags = MWAIT2flg(0x30) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_C4,
		.target_residency = 400,
		.enter = &intel_idle },
	{ /* MWAIT C6 */
		.name = "ATM-C6",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x52) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_C6,
		.power_usage  = C6_POWER_USAGE,
		.target_residency = 560,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-S0i1",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x1) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_S0i1,
		.power_usage  = S0I1_POWER_USAGE,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-LpAudio",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x3) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_LPMP3,
		.power_usage  = LPMP3_POWER_USAGE,
		.enter = &soc_s0ix_idle },
	{
		.name = "ATM-S0i3",
		.desc = "MWAIT 0x52",
		.flags = MWAIT2flg(0x7) | CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_TLB_FLUSHED,
		.exit_latency = CSTATE_EXIT_LATENCY_S0i3,
		.power_usage  = S0I3_POWER_USAGE,
		.enter = &soc_s0ix_idle },
	{
		.enter = NULL }
};

static inline bool is_irq_pending(void)
{
	int i, base = APIC_IRR;

	for (i = 0; i < 8; i++)
		if (apic_read(base + i*0x10) != 0)
			return true;

	return false;
}
static int enter_s0ix_state(u32 eax, int gov_req_state, int s0ix_state,
		  struct cpuidle_device *dev, int index)
{
	int s0ix_entered = 0;
	int selected_state = C6_STATE_IDX;

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

#ifdef CONFIG_XEN
	HYPERVISOR_monitor_op((void *)&current_thread_info()->flags, 0, 0);
#else
	__monitor((void *)&current_thread_info()->flags, 0, 0);
#endif
	smp_mb();
	if (!need_resched()) {
#ifdef CONFIG_XEN
		HYPERVISOR_mwait_op(eax, 1,
					(void *)&current_thread_info()->flags,
					0);
#else
		__mwait(eax, 1);
#endif
	}

	if (!need_resched() && is_irq_pending() == 0)
		__get_cpu_var(update_buckets) = 0;

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
	unsigned long eax = flg2MWAIT(state->flags);
	int cpu = smp_processor_id();
	int s0ix_state   = 0;
	unsigned int cstate;
	int gov_req_state = (int) eax;

	/* Check if s0ix is already in progress,
	 * This is required to demote C6 while S0ix
	 * is in progress
	 */
	if (unlikely(pmu_is_s0ix_in_progress()))
		return intel_idle(dev, drv, C4_STATE_IDX);

	/* check if we need/possible to do s0ix */
	if (eax != C6_HINT)
		s0ix_state = get_target_platform_state(&eax);

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
#else
#define mfld_cstates atom_cstates
#endif

#ifdef CONFIG_ATOM_SOC_POWER
static unsigned int get_target_residency(unsigned int cstate)
{
	unsigned int t_sleep = cpuidle_state_table[cstate].target_residency;
	unsigned int prev_idx;

	/* get the previous lower sleep state */
	if ((cstate == 5) || (cstate == 6))
		prev_idx = cstate - 2;
	else
		prev_idx = cstate - 1;

	/* calculate target_residency only if not defined already */
	if (!t_sleep) {
		/* Use C0 power usage to calculate the target residency */
		unsigned int p_active = C0_POWER_USAGE;
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

			t_sleep = ((p_active * (curr_state_lat - prev_state_lat))
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

#ifdef CONFIG_MOOREFIELD
/* MOFD: Optimize special variants of S0i1 where low residency is sufficient */
int low_latency_s0ix_state(int eax)
{
	u32 dsp_ss_pm_val;

	dsp_ss_pm_val = intel_mid_msgbus_read32(PUNIT_PORT, DSP_SS_PM);
	if (dsp_ss_pm_val & S0I1_DISPLAY_MODE)
		eax = S0I1_STATE;

	return eax;
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
	unsigned long eax = flg2MWAIT(state->flags);
	unsigned int cstate;
	int cpu = smp_processor_id();
#ifdef CONFIG_MOOREFIELD
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
#endif

#if (defined(CONFIG_REMOVEME_INTEL_ATOM_MRFLD_POWER) && \
	defined(CONFIG_PM_DEBUG))
	{
		/* Get Cstate based on ignore table from PMU driver */
		unsigned int ncstate;
		cstate =
		(((eax) >> MWAIT_SUBSTATE_SIZE) & MWAIT_CSTATE_MASK) + 1;
		ncstate = pmu_get_new_cstate(cstate, &index);
		eax	= flg2MWAIT(drv->states[index].flags);
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

	if (!need_resched()) {
#ifdef CONFIG_XEN
		HYPERVISOR_mwait_op(eax, ecx,
					(void *)&current_thread_info()->flags,
					0);
#else

#ifdef CONFIG_MOOREFIELD
		if (eax >= C6_HINT && latency_req > S0i1_LATENCY
			&& per_cpu(predicted_time, cpu) > LOW_LATENCY_S0I1)
			eax = low_latency_s0ix_state(eax);
#endif
		__monitor((void *)&current_thread_info()->flags, 0, 0);
		smp_mb();
		if (!need_resched())
			__mwait(eax, ecx);
#if defined(CONFIG_REMOVEME_INTEL_ATOM_MDFLD_POWER) || \
	defined(CONFIG_REMOVEME_INTEL_ATOM_CLV_POWER)
		if (!need_resched() && is_irq_pending() == 0)
			__get_cpu_var(update_buckets) = 0;
#endif
#endif /* CONFIG_XEN */
	}

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

static int cpu_hotplug_notify(struct notifier_block *n,
			      unsigned long action, void *hcpu)
{
	int hotcpu = (unsigned long)hcpu;
	struct cpuidle_device *dev;

	switch (action & 0xf) {
	case CPU_ONLINE:

		if (lapic_timer_reliable_states != LAPIC_TIMER_ALWAYS_RELIABLE)
			smp_call_function_single(hotcpu, __setup_broadcast_timer,
						 (void *)true, 1);

		/*
		 * Some systems can hotplug a cpu at runtime after
		 * the kernel has booted, we have to initialize the
		 * driver in this case
		 */
		dev = per_cpu_ptr(intel_idle_cpuidle_devices, hotcpu);
		if (!dev->registered)
			intel_idle_cpu_init(hotcpu);

		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_notify,
};

static void auto_demotion_disable(void *dummy)
{
	unsigned long long msr_bits;

	rdmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
	msr_bits &= ~(icpu->auto_demotion_disable_flags);
	wrmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
}
static void c1e_promotion_disable(void *dummy)
{
	unsigned long long msr_bits;

	rdmsrl(MSR_IA32_POWER_CTL, msr_bits);
	msr_bits &= ~0x2;
	wrmsrl(MSR_IA32_POWER_CTL, msr_bits);
}

static const struct idle_cpu idle_cpu_nehalem = {
	.state_table = nehalem_cstates,
	.auto_demotion_disable_flags = NHM_C1_AUTO_DEMOTE | NHM_C3_AUTO_DEMOTE,
	.disable_promotion_to_c1e = true,
};

static const struct idle_cpu idle_cpu_atom = {
	.state_table = atom_cstates,
};

static const struct idle_cpu idle_cpu_lincroft = {
	.state_table = atom_cstates,
	.auto_demotion_disable_flags = ATM_LNC_C6_AUTO_DEMOTE,
};

static const struct idle_cpu idle_cpu_snb = {
	.state_table = snb_cstates,
	.disable_promotion_to_c1e = true,
};

static const struct idle_cpu idle_cpu_ivb = {
	.state_table = ivb_cstates,
	.disable_promotion_to_c1e = true,
};

static const struct idle_cpu idle_cpu_hsw = {
	.state_table = hsw_cstates,
	.disable_promotion_to_c1e = true,
};

static const struct idle_cpu idle_cpu_mfld = {
	.state_table = mfld_cstates,
	.auto_demotion_disable_flags = ATM_LNC_C6_AUTO_DEMOTE,
};

static const struct idle_cpu idle_cpu_mrfld = {
	.state_table = mrfld_cstates,
};

static const struct idle_cpu idle_cpu_vlv = {
	.state_table = vlv_cstates,
};

static const struct idle_cpu idle_cpu_moorfld = {
	.state_table = moorfld_cstates,
};

static const struct idle_cpu idle_cpu_chv = {
	.state_table = chv_cstates,
};

#define ICPU(model, cpu) \
	{ X86_VENDOR_INTEL, 6, model, X86_FEATURE_MWAIT, (unsigned long)&cpu }

static const struct x86_cpu_id intel_idle_ids[] = {
	ICPU(0x1a, idle_cpu_nehalem),
	ICPU(0x1e, idle_cpu_nehalem),
	ICPU(0x1f, idle_cpu_nehalem),
	ICPU(0x25, idle_cpu_nehalem),
	ICPU(0x2c, idle_cpu_nehalem),
	ICPU(0x2e, idle_cpu_nehalem),
	ICPU(0x1c, idle_cpu_atom),
	ICPU(0x26, idle_cpu_lincroft),
	ICPU(0x2f, idle_cpu_nehalem),
	ICPU(0x2a, idle_cpu_snb),
	ICPU(0x2d, idle_cpu_snb),
	ICPU(0x4c, idle_cpu_chv),
	ICPU(0x37, idle_cpu_vlv),
	ICPU(0x3a, idle_cpu_ivb),
	ICPU(0x3e, idle_cpu_ivb),
	ICPU(0x3c, idle_cpu_hsw),
	ICPU(0x3f, idle_cpu_hsw),
	ICPU(0x45, idle_cpu_hsw),
	ICPU(0x46, idle_cpu_hsw),
	ICPU(0x27, idle_cpu_mfld),
	ICPU(0x35, idle_cpu_mfld),
	ICPU(0x4a, idle_cpu_mrfld),	/* Tangier SoC */
	ICPU(0x5a, idle_cpu_moorfld),	/* Anniedale SoC */
	{}
};
MODULE_DEVICE_TABLE(x86cpu, intel_idle_ids);

/*
 * intel_idle_probe()
 */
static int intel_idle_probe(void)
{
	unsigned int eax, ebx, ecx;
	const struct x86_cpu_id *id;

	if (max_cstate == 0) {
		pr_debug(PREFIX "disabled\n");
		return -EPERM;
	}

	id = x86_match_cpu(intel_idle_ids);
	if (!id) {
		if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
		    boot_cpu_data.x86 == 6)
			pr_debug(PREFIX "does not run on family %d model %d\n",
				boot_cpu_data.x86, boot_cpu_data.x86_model);
		return -ENODEV;
	}

	if (boot_cpu_data.cpuid_level < CPUID_MWAIT_LEAF)
		return -ENODEV;

	cpuid(CPUID_MWAIT_LEAF, &eax, &ebx, &ecx, &mwait_substates);

	if (!(ecx & CPUID5_ECX_EXTENSIONS_SUPPORTED) ||
	    !(ecx & CPUID5_ECX_INTERRUPT_BREAK) ||
	    !mwait_substates)
			return -ENODEV;

	pr_debug(PREFIX "MWAIT substates: 0x%x\n", mwait_substates);

	icpu = (const struct idle_cpu *)id->driver_data;
	cpuidle_state_table = icpu->state_table;

	if (boot_cpu_has(X86_FEATURE_ARAT))	/* Always Reliable APIC Timer */
		lapic_timer_reliable_states = LAPIC_TIMER_ALWAYS_RELIABLE;
	else
		on_each_cpu(__setup_broadcast_timer, (void *)true, 1);

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
/*
 * intel_idle_cpuidle_driver_init()
 * allocate, initialize cpuidle_states
 */
static int intel_idle_cpuidle_driver_init(void)
{
	int cstate;
	struct cpuidle_driver *drv = &intel_idle_driver;

	drv->state_count = 1;

	for (cstate = 0; cstate < CPUIDLE_STATE_MAX; ++cstate) {
		int num_substates = 0, mwait_hint, mwait_cstate, mwait_substate;

		if (cpuidle_state_table[cstate].enter == NULL)
			break;

		if (cstate + 1 > max_cstate) {
			printk(PREFIX "max_cstate %d reached\n",
				max_cstate);
			break;
		}

		mwait_hint = flg2MWAIT(cpuidle_state_table[cstate].flags);
		mwait_cstate = MWAIT_HINT2CSTATE(mwait_hint);
		mwait_substate = MWAIT_HINT2SUBSTATE(mwait_hint);

		/* does the state exist in CPUID.MWAIT? */

		/* FIXME: Do not check number of substates for any states above C6
		 * as these are not real C states supported by the CPU, they
		 * are emulated c states for s0ix support.
		*/
		if ((mwait_cstate + 1) <= 6) {
			num_substates = (mwait_substates >> ((mwait_cstate + 1) * 4))
					& MWAIT_SUBSTATE_MASK;
			if (num_substates == 0)
				continue;
		}

#if !defined(CONFIG_ATOM_SOC_POWER)
		if ((boot_cpu_data.x86_model != 0x37) && (boot_cpu_data.x86_model != 0x4c)) {
			/* if sub-state in table is not enumerated by CPUID */
			if ((mwait_substate + 1) > num_substates)
				continue;
		}
#endif
		if (((mwait_cstate + 1) > 2) &&
			!boot_cpu_has(X86_FEATURE_NONSTOP_TSC))
			mark_tsc_unstable("TSC halts in idle"
					" states deeper than C2");

#ifdef CONFIG_ATOM_SOC_POWER
		/* Calculate target_residency if power_usage is given */
		cpuidle_state_table[cstate].target_residency =
			get_target_residency(cstate);
#endif
		drv->states[drv->state_count] =	/* structure copy */
			cpuidle_state_table[cstate];

		drv->state_count += 1;
	}

	if (icpu->auto_demotion_disable_flags)
		on_each_cpu(auto_demotion_disable, NULL, 1);

	if (icpu->disable_promotion_to_c1e)	/* each-cpu is redundant */
		on_each_cpu(c1e_promotion_disable, NULL, 1);

	return 0;
}


/*
 * intel_idle_cpu_init()
 * allocate, initialize, register cpuidle_devices
 * @cpu: cpu/core to initialize
 */
static int intel_idle_cpu_init(int cpu)
{
	int cstate;
	struct cpuidle_device *dev;

	dev = per_cpu_ptr(intel_idle_cpuidle_devices, cpu);

	dev->state_count = 1;

	for (cstate = 0; cstate < CPUIDLE_STATE_MAX; ++cstate) {
		int num_substates = 0, mwait_hint, mwait_cstate, mwait_substate;

		if (cpuidle_state_table[cstate].enter == NULL)
			continue;

		if (cstate + 1 > max_cstate) {
			printk(PREFIX "max_cstate %d reached\n", max_cstate);
			break;
		}

		mwait_hint = flg2MWAIT(cpuidle_state_table[cstate].flags);
		mwait_cstate = MWAIT_HINT2CSTATE(mwait_hint);
		mwait_substate = MWAIT_HINT2SUBSTATE(mwait_hint);

		/* does the state exist in CPUID.MWAIT? */

		/* FIXME: Do not check number of substates for any states above C6
		 * as these are not real C states supported by the CPU, they
		 * are emulated c states for s0ix support.
		 */
		if ((mwait_cstate + 1) <= 6) {
			num_substates = (mwait_substates >> ((mwait_cstate + 1) * 4))
					& MWAIT_SUBSTATE_MASK;
			if (num_substates == 0)
				continue;
		}

#if !defined(CONFIG_ATOM_SOC_POWER)
		if ((boot_cpu_data.x86_model != 0x37) && (boot_cpu_data.x86_model != 0x4c)) {
			/* if sub-state in table is not enumerated by CPUID */
			if ((mwait_substate + 1) > num_substates)
				continue;
		}
#endif
		dev->state_count += 1;
	}

	dev->cpu = cpu;

	if (cpuidle_register_device(dev)) {
		pr_debug(PREFIX "cpuidle_register_device %d failed!\n", cpu);
		intel_idle_cpuidle_devices_uninit();
		return -EIO;
	}

	if (icpu->auto_demotion_disable_flags)
		smp_call_function_single(cpu, auto_demotion_disable, NULL, 1);

	per_cpu(update_buckets, cpu) = 1;

	return 0;
}

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
		struct cpuidle_driver *drv = cpuidle_get_driver();
		printk(KERN_DEBUG PREFIX "intel_idle yielding to %s",
			drv ? drv->name : "none");
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

		if (platform_is(INTEL_ATOM_BYT) || platform_is(INTEL_ATOM_CHT)) {
			/* Disable automatic core C6 demotion by PUNIT */
			if (wrmsr_on_cpu(i, CLPU_CR_C6_POLICY_CONFIG,
					DISABLE_CORE_C6_DEMOTION, 0x0))
				pr_err("Error to disable core C6 demotion");

			/* Disable automatic module C6 demotion by PUNIT */
			if (wrmsr_on_cpu(i, CLPU_MD_C6_POLICY_CONFIG,
					DISABLE_MODULE_C6_DEMOTION, 0x0))
				pr_err("Error to disable module C6 demotion");
		}
	}
	register_cpu_notifier(&cpu_hotplug_notifier);

	return 0;
}

static void __exit intel_idle_exit(void)
{
	intel_idle_cpuidle_devices_uninit();
	cpuidle_unregister_driver(&intel_idle_driver);


	if (lapic_timer_reliable_states != LAPIC_TIMER_ALWAYS_RELIABLE)
		on_each_cpu(__setup_broadcast_timer, (void *)false, 1);
	unregister_cpu_notifier(&cpu_hotplug_notifier);

	return;
}

module_init(intel_idle_init);
module_exit(intel_idle_exit);

module_param(max_cstate, int, 0444);

MODULE_AUTHOR("Len Brown <len.brown@intel.com>");
MODULE_DESCRIPTION("Cpuidle driver for Intel Hardware v" INTEL_IDLE_VERSION);
MODULE_LICENSE("GPL");
