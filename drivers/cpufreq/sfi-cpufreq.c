/*
 * sfi_cpufreq.c - sfi Processor P-States Driver
 *
 * (C) 2010-2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Vishwesh M Rudramuni
 * Contact information: Vishwesh Rudramuni <vishwesh.m.rudramuni@intel.com>
 */

/*
 * This sfi Processor P-States Driver re-uses most part of the code available
 * in acpi cpufreq driver.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/compiler.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sfi.h>
#include <linux/io.h>

#include <asm/msr.h>
#include <asm/processor.h>
#include <asm/cpufeature.h>

#include "sfi-cpufreq.h"
#include "mperf.h"

MODULE_AUTHOR("Vishwesh Rudramuni");
MODULE_DESCRIPTION("SFI Processor P-States Driver");
MODULE_LICENSE("GPL");

DEFINE_PER_CPU(struct sfi_processor *, sfi_processors);

static DEFINE_MUTEX(performance_mutex);
static int sfi_cpufreq_num;
static u32 sfi_cpu_num;

#define		SFI_FREQ_MAX            32
#define		INTEL_MSR_RANGE		(0xffff)
#define		SFI_CPU_MAX		8


struct sfi_cpufreq_data {
	struct sfi_processor_performance *sfi_data;
	struct cpufreq_frequency_table *freq_table;
	unsigned int max_freq;
	unsigned int resume;
};

static DEFINE_PER_CPU(struct sfi_cpufreq_data *, drv_data);
struct sfi_freq_table_entry sfi_cpufreq_array[SFI_FREQ_MAX];
static struct sfi_cpu_table_entry sfi_cpu_array[SFI_CPU_MAX];

/* sfi_perf_data is a pointer to percpu data. */
static struct sfi_processor_performance *sfi_perf_data;

static struct cpufreq_driver sfi_cpufreq_driver;

static int parse_freq(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_freq_table_entry *pentry;
	int totallen;

	sb = (struct sfi_table_simple *)table;
	if (!sb) {
		printk(KERN_WARNING "SFI: Unable to map FREQ\n");
		return -ENODEV;
	}

	if (!sfi_cpufreq_num) {
		sfi_cpufreq_num = SFI_GET_NUM_ENTRIES(sb,
			 struct sfi_freq_table_entry);
		pentry = (struct sfi_freq_table_entry *)sb->pentry;
		totallen = sfi_cpufreq_num * sizeof(*pentry);
		memcpy(sfi_cpufreq_array, pentry, totallen);
	}

	return 0;
}

static int sfi_processor_get_performance_states(struct sfi_processor *pr)
{
	int result = 0;
	int i;

	pr->performance->state_count = sfi_cpufreq_num;
	pr->performance->states =
	    kmalloc(sizeof(struct sfi_processor_px) * sfi_cpufreq_num,
		    GFP_KERNEL);
	if (!pr->performance->states)
		result = -ENOMEM;

	printk(KERN_INFO "Num p-states %d\n", sfi_cpufreq_num);

	/* Populate the P-states info from the SFI table here */
	for (i = 0; i < sfi_cpufreq_num; i++) {
		pr->performance->states[i].core_frequency = \
			sfi_cpufreq_array[i].freq_mhz;
		pr->performance->states[i].transition_latency = \
			sfi_cpufreq_array[i].latency;
		pr->performance->states[i].control = \
			sfi_cpufreq_array[i].ctrl_val;
		printk(KERN_INFO "State [%d]: core_frequency[%d] transition_latency[%d] control[0x%x]\n",
			i,
			(u32) pr->performance->states[i].core_frequency,
			(u32) pr->performance->states[i].transition_latency,
			(u32) pr->performance->states[i].control);
	}

	return result;
}

static int sfi_processor_register_performance(struct sfi_processor_performance
				    *performance, unsigned int cpu)
{
	struct sfi_processor *pr;

	mutex_lock(&performance_mutex);

	pr = per_cpu(sfi_processors, cpu);
	if (!pr) {
		mutex_unlock(&performance_mutex);
		return -ENODEV;
	}

	if (pr->performance) {
		mutex_unlock(&performance_mutex);
		return -EBUSY;
	}

	WARN_ON(!performance);

	pr->performance = performance;

	/* parse the freq table from sfi */
	sfi_cpufreq_num = 0;
	sfi_table_parse(SFI_SIG_FREQ, NULL, NULL, parse_freq);

	sfi_processor_get_performance_states(pr);

	mutex_unlock(&performance_mutex);
	return 0;
}

void sfi_processor_unregister_performance(struct sfi_processor_performance
				      *performance, unsigned int cpu)
{
	struct sfi_processor *pr;


	mutex_lock(&performance_mutex);

	pr = per_cpu(sfi_processors, cpu);
	if (!pr) {
		mutex_unlock(&performance_mutex);
		return;
	}

	if (pr->performance)
		kfree(pr->performance->states);
	pr->performance = NULL;

	mutex_unlock(&performance_mutex);

	return;
}

static unsigned int get_cur_freq_on_cpu(unsigned int cpu)
{
	struct sfi_cpufreq_data *data = per_cpu(drv_data, cpu);
	unsigned int cached_freq;

	pr_debug("get_cur_freq_on_cpu (%d)\n", cpu);

	if (unlikely(data == NULL ||
		data->sfi_data == NULL || data->freq_table == NULL)) {
		return 0;
	}

	cached_freq = data->freq_table[data->sfi_data->state].frequency;

	return cached_freq;
}

/*
 * Since MSRs record the residency with 1MHz clock,
 * then to compute the actual residency at maximum speed,
 * the value should multiply (maximum clock / 1M ).
 * For MFLD, maximum speed is 1597MHz, so the multiplier=1597
 */
#define MFLD_RESIDENCY_COUNT_MULTIPLIER (1597UL)
/*C-states related data structures and fuctions for cpu load calculation*/

/*  MSR counter stuff */
enum {
	MPERF = 0, /* C0 */
	APERF,     /* C1 */
	C2,
	C3,
	C4,
	C5,
	C6,
	C7,
	C8,
	C9,
	/* C10, */
	/* C11, */
	MAX_MSR_ADDRESSES
};

/*
 * The core MSR addresses are hard coded for Intel ATOM MFLD
 * The addresses need to be re-checked for other Intel devices.
 *
 */
static unsigned int CoreResidencyMSRAddresses[MAX_MSR_ADDRESSES] = {
	0xE7,                   /*MPERF*/
	0xFFFFFFFF,             /*C1*/
	0x3F8,                  /*C2*/
	0xFFFFFFFF,             /*C3*/
	0x3F9,                  /*C4*/
	0xFFFFFFFF,             /*C5*/
	0x3FA,                  /*C6*/
	0xFFFFFFFF,             /*C7*/
	0xFFFFFFFF,             /*C8*/
	0xFFFFFFFF              /*C9*/
};

/*
 * Per-cpu structure holding MSR residency counts,
 * timer-TSC values etc.
 */
struct per_cpu_t {
	u64 tsc; /* 8 bytes */
	u64 residencies[MAX_MSR_ADDRESSES]; /* 96 bytes */
	u64 prev_msr_vals[MAX_MSR_ADDRESSES]; /* 96 bytes */
};

/*
 * Convenience macros for accessing per-cpu residencies
 */
#define RESIDENCY(p, i) ((p)->residencies[(i)])
#define PREV_MSR_VAL(p, i) ((p)->prev_msr_vals[(i)])

static DEFINE_PER_CPU(struct per_cpu_t, per_cpu_counts);

/*
 * Do we read the TSC MSR directly to determine
 * TSC (as opposed to using a kernel
 * function call -- e.g. rdtscll)?
 */
#define READ_MSR_FOR_TSC 1

/* Helper function to get TSC */
static inline void tscval(u64 *v)
{
#if READ_MSR_FOR_TSC
	u64 res;
	rdmsrl(0x10, res);
	*v = res;
#else
	unsigned int aux;
	rdtscpll(*v, aux);
#endif
};

#define C1 APERF

/*
 * Get the delta residency for MSRs
 */
static u64 read_one_residency(int cpu, int msr_addr, u64 *prev)
{
	u64 curr = 0, delta = 0;

	rdmsrl(msr_addr, curr);

	if (unlikely(curr < *prev))
		delta = ((u64)(~0) - *prev) + (curr + 1);
	else
		delta = curr - *prev;

	*prev = curr;

	return delta;
};

static unsigned int calc_cpu_load(struct per_cpu_t *pcpu, int cpu)
{
	int i = 0;
	u64 prev;
	int msr_addr;
	u64 tsc;
	u64 delta_tsc, c0;
	u64 m_delta, c_delta;
	bool is_first = false;
	u64 cx_total = 0;
	u32 clock_multiplier = MFLD_RESIDENCY_COUNT_MULTIPLIER;
	u64 cpu_load = 0;
	/*
	 * Ensure updates are propagated.
	 */
	smp_mb();

	is_first = false;

	if (unlikely(PREV_MSR_VAL(pcpu, MPERF) == 0))
		is_first = true;

	msr_addr = CoreResidencyMSRAddresses[MPERF];
	prev = PREV_MSR_VAL(pcpu, MPERF);
	/*
	 * Read MPERF, compute DELTA(MPERF)
	 */
	m_delta = read_one_residency(cpu, msr_addr, &prev);

	PREV_MSR_VAL(pcpu, MPERF) = prev;
	/*
	 * 'C1' is a DERIVED residency -- we
	 * don't read MSRs for it. Instead, we
	 * compute its value from the values of
	 * OTHER Cx/MPERF/TSC. Reset to zero here.
	 * Currently, we combine C1 with C0 together
	 * as C0 for load calculation. And expriments
	 * show that make sense.
	 */
	RESIDENCY(pcpu, C1) = 0;
	/*
	 * Calculate (non-C1) C-state residency
	 */
	for (i = C2; i <= C6; ++i) {
		RESIDENCY(pcpu, i) = 0;
		msr_addr = CoreResidencyMSRAddresses[i];
		if (msr_addr <= 0)
			continue;

		prev = PREV_MSR_VAL(pcpu, i);
		c_delta = read_one_residency(cpu, msr_addr, &prev);
		PREV_MSR_VAL(pcpu, i) = prev;

		if (!is_first && c_delta) {
			c_delta *= clock_multiplier;
			RESIDENCY(pcpu, i) = c_delta;
			cx_total += c_delta;
		}
	}

	/* compute time interval between two measurements */
	tscval(&tsc);
	delta_tsc = tsc - pcpu->tsc; /* TSC delta */
	pcpu->tsc = tsc;

	/*Actually, it is c0+c1 residency*/
	RESIDENCY(pcpu, MPERF) = c0 = delta_tsc - cx_total;
	/* cpu_load = 100*c0 / delta_tsc */
	cpu_load = c0 * 100;
	do_div(cpu_load, delta_tsc);

	return (unsigned int)cpu_load;

};

unsigned int cpufreq_get_load(struct cpufreq_policy *policy, unsigned int cpu)
{
	struct per_cpu_t *pcpu = NULL;

	pcpu = &__get_cpu_var(per_cpu_counts);

	return calc_cpu_load(pcpu, cpu);
}

static int sfi_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int target_freq, unsigned int relation)
{
	struct sfi_cpufreq_data *data = per_cpu(drv_data, policy->cpu);
	struct sfi_processor_performance *perf;
	struct cpufreq_freqs freqs;
	unsigned int next_state = 0; /* Index into freq_table */
	unsigned int next_perf_state = 0; /* Index into perf table */
	int result = 0;
	u32 lo, hi;

	pr_debug("sfi_cpufreq_target %d (%d)\n", target_freq, policy->cpu);

	if (unlikely(data == NULL ||
	     data->sfi_data == NULL || data->freq_table == NULL)) {
		return -ENODEV;
	}

	perf = data->sfi_data;
	result = cpufreq_frequency_table_target(policy,
						data->freq_table,
						target_freq,
						relation, &next_state);
	if (unlikely(result))
		return -ENODEV;

	next_perf_state = data->freq_table[next_state].index;
	if (perf->state == next_perf_state) {
		if (unlikely(data->resume)) {
			pr_debug("Called after resume, resetting to P%d\n",
				next_perf_state);
			data->resume = 0;
		} else {
			pr_debug("Already at target state (P%d)\n",
				next_perf_state);
			return 0;
		}
	}

	freqs.old = perf->states[perf->state].core_frequency * 1000;
	freqs.new = data->freq_table[next_state].frequency;
	freqs.cpu = policy->cpu;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	rdmsr_on_cpu(policy->cpu, MSR_IA32_PERF_CTL, &lo, &hi);
	lo = (lo & ~INTEL_MSR_RANGE) |
		((u32) perf->states[next_perf_state].control & INTEL_MSR_RANGE);
	wrmsr_on_cpu(policy->cpu, MSR_IA32_PERF_CTL, lo, hi);


	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	perf->state = next_perf_state;

	return result;
}

static int sfi_cpufreq_verify(struct cpufreq_policy *policy)
{
	struct sfi_cpufreq_data *data = per_cpu(drv_data, policy->cpu);

	pr_debug("sfi_cpufreq_verify\n");

	return cpufreq_frequency_table_verify(policy, data->freq_table);
}

/*
 * sfi_cpufreq_early_init - initialize SFI P-States library
 *
 * Initialize the SFI P-States library (drivers/sfi/processor_perflib.c)
 * in order to cope with the correct frequency and voltage pairings.
 */
static int __init sfi_cpufreq_early_init(void)
{
	sfi_perf_data = alloc_percpu(struct sfi_processor_performance);
	if (!sfi_perf_data) {
		pr_debug("Memory allocation error for sfi_perf_data.\n");
		return -ENOMEM;
	}

	return 0;
}


static int sfi_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	unsigned int i;
	unsigned int valid_states = 0;
	unsigned int cpu = policy->cpu;
	struct sfi_cpufreq_data *data;
	unsigned int result = 0;
	struct cpuinfo_x86 *c = &cpu_data(policy->cpu);
	struct sfi_processor_performance *perf;

	pr_debug("sfi_cpufreq_cpu_init CPU:%d\n", policy->cpu);

	data = kzalloc(sizeof(struct sfi_cpufreq_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->sfi_data = per_cpu_ptr(sfi_perf_data, cpu);
	per_cpu(drv_data, cpu) = data;

	sfi_cpufreq_driver.flags |= CPUFREQ_CONST_LOOPS;


	result = sfi_processor_register_performance(data->sfi_data, cpu);
	if (result)
		goto err_free;

	perf = data->sfi_data;
	policy->shared_type = CPUFREQ_SHARED_TYPE_HW;

	cpumask_set_cpu(policy->cpu, policy->cpus);
	cpumask_set_cpu(policy->cpu, policy->related_cpus);

	/* capability check */
	if (perf->state_count <= 1) {
		pr_debug("No P-States\n");
		result = -ENODEV;
		goto err_unreg;
	}

	data->freq_table = kzalloc(sizeof(struct cpufreq_frequency_table) *
		    (perf->state_count+1), GFP_KERNEL);
	if (!data->freq_table) {
		result = -ENOMEM;
		goto err_unreg;
	}

	/* detect transition latency */
	policy->cpuinfo.transition_latency = 0;
	for (i = 0; i < perf->state_count; i++) {
		if ((perf->states[i].transition_latency * 1000) >
		    policy->cpuinfo.transition_latency)
			policy->cpuinfo.transition_latency =
			    perf->states[i].transition_latency * 1000;
	}

	data->max_freq = perf->states[0].core_frequency * 1000;
	/* table init */
	for (i = 0; i < perf->state_count; i++) {
		if (i > 0 && perf->states[i].core_frequency >=
		    data->freq_table[valid_states-1].frequency / 1000)
			continue;

		data->freq_table[valid_states].index = i;
		data->freq_table[valid_states].frequency =
		    perf->states[i].core_frequency * 1000;
		valid_states++;
	}
	data->freq_table[valid_states].frequency = CPUFREQ_TABLE_END;
	perf->state = 0;

	result = cpufreq_frequency_table_cpuinfo(policy, data->freq_table);
	if (result)
		goto err_freqfree;

	policy->cur = get_cur_freq_on_cpu(cpu);


	/* Check for APERF/MPERF support in hardware */
	if (cpu_has(c, X86_FEATURE_APERFMPERF)) {
		sfi_cpufreq_driver.getavg = cpufreq_get_measured_perf;
		sfi_cpufreq_driver.getload = cpufreq_get_load;
	}

	pr_debug("CPU%u - SFI performance management activated.\n", cpu);
	for (i = 0; i < perf->state_count; i++)
		pr_debug("     %cP%d: %d MHz, %d uS\n",
			(i == perf->state ? '*' : ' '), i,
			(u32) perf->states[i].core_frequency,
			(u32) perf->states[i].transition_latency);

	cpufreq_frequency_table_get_attr(data->freq_table, policy->cpu);

	/*
	 * the first call to ->target() should result in us actually
	 * writing something to the appropriate registers.
	 */
	data->resume = 1;

	return result;

err_freqfree:
	kfree(data->freq_table);
err_unreg:
	sfi_processor_unregister_performance(perf, cpu);
err_free:
	kfree(data);
	per_cpu(drv_data, cpu) = NULL;

	return result;
}

static int sfi_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	struct sfi_cpufreq_data *data = per_cpu(drv_data, policy->cpu);

	pr_debug("sfi_cpufreq_cpu_exit\n");

	if (data) {
		cpufreq_frequency_table_put_attr(policy->cpu);
		per_cpu(drv_data, policy->cpu) = NULL;
		sfi_processor_unregister_performance(data->sfi_data,
							policy->cpu);
		kfree(data);
	}

	return 0;
}

static int sfi_cpufreq_resume(struct cpufreq_policy *policy)
{
	struct sfi_cpufreq_data *data = per_cpu(drv_data, policy->cpu);

	pr_debug("sfi_cpufreq_resume\n");

	data->resume = 1;

	return 0;
}

static struct freq_attr *sfi_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sfi_cpufreq_driver = {
	.get = get_cur_freq_on_cpu,
	.verify = sfi_cpufreq_verify,
	.target = sfi_cpufreq_target,
	.init = sfi_cpufreq_cpu_init,
	.exit = sfi_cpufreq_cpu_exit,
	.resume = sfi_cpufreq_resume,
	.name = "sfi-cpufreq",
	.owner = THIS_MODULE,
	.attr = sfi_cpufreq_attr,
};

static int __init parse_cpus(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_cpu_table_entry *pentry;
	int i;

	sb = (struct sfi_table_simple *)table;

	sfi_cpu_num = SFI_GET_NUM_ENTRIES(sb, struct sfi_cpu_table_entry);

	pentry = (struct sfi_cpu_table_entry *) sb->pentry;
	for (i = 0; i < sfi_cpu_num; i++) {
		sfi_cpu_array[i].apic_id = pentry->apic_id;
		printk(KERN_INFO "APIC ID: %d\n", pentry->apic_id);
		pentry++;
	}

	return 0;

}


static int __init init_sfi_processor_list(void)
{
	struct sfi_processor *pr;
	int i;
	int result;

	/* parse the cpus from the sfi table */
	result = sfi_table_parse(SFI_SIG_CPUS, NULL, NULL, parse_cpus);

	if (result < 0)
		return result;

	pr = kzalloc(sfi_cpu_num * sizeof(struct sfi_processor), GFP_KERNEL);
	if (!pr)
		return -ENOMEM;

	for (i = 0; i < sfi_cpu_num; i++) {
		pr->id = sfi_cpu_array[i].apic_id;
		per_cpu(sfi_processors, pr->id) = pr;
		pr++;
	}

	return 0;
}

static int __init sfi_cpufreq_init(void)
{
	int ret;

	pr_debug("sfi_cpufreq_init\n");

	ret = init_sfi_processor_list();
	if (ret)
		return ret;

	ret = sfi_cpufreq_early_init();
	if (ret)
		return ret;

	return cpufreq_register_driver(&sfi_cpufreq_driver);
}

static void __exit sfi_cpufreq_exit(void)
{

	struct sfi_processor *pr;

	pr_debug("sfi_cpufreq_exit\n");

	pr = per_cpu(sfi_processors, 0);
	kfree(pr);

	cpufreq_unregister_driver(&sfi_cpufreq_driver);

	free_percpu(sfi_perf_data);

	return;
}
late_initcall(sfi_cpufreq_init);
module_exit(sfi_cpufreq_exit);

MODULE_ALIAS("sfi");
