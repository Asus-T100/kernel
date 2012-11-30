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

#ifdef CONFIG_COMPUTE_PHYSICAL_CORE_LOAD
#define phy_core_id(cpu) (topology_core_id(cpu))
#define phy_core_idle(mask) (mask == 0)

/* percpu struct to record the per cpu active info */
struct per_cpu_t {
	u64 tsc;		  /* for recording the sampling time */
	u64 active_tsc;		  /* active tsc recorded by this cpu */
	u64 total_active_tsc;	  /* total core active tsc */
#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
	u64 prev_gpu_block_time;
#endif
};

DEFINE_PER_CPU(struct per_cpu_t, pcpu_counts);
#pragma pack(8)
/* per physical core struct to record physical core info */
struct per_physical_core_t {
	/*
	 * Variable to record all cpu status (idle or busy) together.
	 * Aligned 64-bit access in 64-bit processor is atomic, hence SMP
	 * safe without needing a lock. One byte for one core, so it can
	 * support up to eight cores. It is enough up to near future.
	 */
	u64 busy_mask;		/*show the status of siblings in physical core*/
	u64 active_start_tsc;	/*the active start time of the physical core*/
	u64 accum_flag;		/*indicate the active period has been counted*/
#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
	u64 gpu_block_time;
	u64 gpu_block_start_tsc;
	atomic_t wait_for_gpu_count;
#endif
};

DEFINE_PER_CPU(struct per_physical_core_t, pphycore_counts);
#pragma pack()

/*
 * Set mask to indicate the cpu is busy or idle. One byte for one cpu.
 * So the 64-bit mask can support up to eight cpus.
 * Byte operations are atomic
 */
static inline void set_cpu_idle(int cpu, u64 *mask)
{
	*((u8 *)mask + cpu) = 0;
}
static inline void set_cpu_busy(int cpu, u64 *mask)
{
	*((u8 *)mask + cpu) = 1;
}

/*
 * Update the active time when CPU enter/exit idle. Physical core is active
 * when at least one logical core is active, physical core is idle when all
 * the logical cores are idle. So the physical core active tsc is the time
 * period between the first logical core exit idle and the time all the
 * logical cores enter idle. The active period is updated at the time all
 * the logical cores enter idle. Physical core active time is used for
 * computing load of physical core.
 */
void update_cpu_active_tsc(int cpu, int enter_idle)
{
	u64 tsc;
	u64 old;
	struct per_cpu_t *pcpu = NULL;
	struct per_physical_core_t *pphycore = NULL;
	int phycore_id;
	u64 *phycore_start;

	phycore_id = phy_core_id(cpu);
	pphycore = &per_cpu(pphycore_counts, phycore_id);
	phycore_start = &(pphycore->active_start_tsc);

	if (enter_idle) {
		/* get the TSC at the time */
		rdtscll(tsc);
		old = *phycore_start;
		/* set the cpu's byte in its physical mask */
		set_cpu_idle(cpu, &(pphycore->busy_mask));
		/*
		 * update the active time only when all the siblings in the
		 * physical core are idle. To avoid simulataneous access by
		 * siblings, we use cmpxchg here.
		 */
		if (phy_core_idle(pphycore->busy_mask)) {
			if (old == *phycore_start &&
			    old == cmpxchg64(phycore_start, old, tsc)) {
				pcpu = &per_cpu(pcpu_counts, cpu);
				if (tsc > old)
					pcpu->active_tsc += tsc - old;
#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
				/*
				 * If current physical core is blocked by GPU,
				 * record entering idle tsc as the start of a
				 * time slice blocked on GPU.
				 */
				if (atomic_read(&pphycore->wait_for_gpu_count)
				    > 0)
					pphycore->gpu_block_start_tsc = tsc;
#endif
				pphycore->accum_flag = 1;
			}
		}
	} else { /* exit idle */
		/*
		 * the active period is counted at the time the first core
		 * exits idle. We use cmpxchg to avoid confliction.
		 */
		rdtscll(tsc);
		if (phy_core_idle(pphycore->busy_mask)) {
			if (1 == cmpxchg64(&(pphycore->accum_flag), 1, 0)) {
				*phycore_start = tsc;
#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
				if (atomic_read(&pphycore->wait_for_gpu_count)
				    > 0 && pphycore->gpu_block_start_tsc > 0)
					pphycore->gpu_block_time += tsc -
						pphycore->gpu_block_start_tsc;
				pphycore->gpu_block_start_tsc = 0;
#endif
			}
		}
		/* since the cpu exits idle, set its byte in mask as active */
		set_cpu_busy(cpu, &(pphycore->busy_mask));
	}
}

/*
 * Return the cpu load value used in CPUFreq governor.
 * The function can be used in CPUFreq governor to show the load of physical
 * core. Governor uses sampling to get the cpu load in each sampling
 * interval. This function is called in every sampling, and return the load
 * of that sampling interval.
 */
#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
static unsigned int cpufreq_get_load(struct cpufreq_policy *policy,
			unsigned int cpu, unsigned int *gpu_block_load)
{
	u64 delta_gpu_block_time;
	u64 tmp_block_start;
#else
static unsigned int cpufreq_get_load(struct cpufreq_policy *policy,
				     unsigned int cpu)
{
#endif
	u64 tsc;
	u64 total_active_tsc = 0;
	u64 delta_tsc, delta_active_tsc;
	u64 load;
	u64 tmp;
	unsigned int j;
	struct per_cpu_t *this_cpu;
	struct per_cpu_t *pcpu;
	struct per_physical_core_t *pphycore = NULL;
	int phycore_id;
	u64 *phycore_start;

	phycore_id = phy_core_id(cpu);
	pphycore = &per_cpu(pphycore_counts, phycore_id);
	phycore_start = &(pphycore->active_start_tsc);
	this_cpu = &per_cpu(pcpu_counts, cpu);
	rdtscll(tsc);
	delta_tsc = tsc - this_cpu->tsc;

	/*
	 * if this sampling occurs at the same time when all logical cores
	 * enter idle, they may compete to access the active tsc and shared
	 * active_start_tsc. To solve the issue, we use cmpxchg to make sure
	 * only one can do it.
	 */
	tmp = *phycore_start;
	if (!phy_core_idle(pphycore->busy_mask)) {
		if (tmp == *phycore_start && tmp ==
		    cmpxchg64(phycore_start, tmp, tsc)) {
			if (tsc > tmp)
				this_cpu->active_tsc += tsc - tmp;
			pphycore->accum_flag = 1;
		}
	}

	/*
	 * To compute the load of physical core, we need to sum all the
	 * active time accumulated by siblings in the physical core.
	 */
	for_each_cpu(j, cpu_sibling_mask(cpu)) {
		pcpu = &per_cpu(pcpu_counts, j);
		total_active_tsc += pcpu->active_tsc;
	}
	/* active time between two calls */
	delta_active_tsc = total_active_tsc - this_cpu->total_active_tsc;

	this_cpu->tsc = tsc;
	this_cpu->total_active_tsc = total_active_tsc;

	if (delta_tsc > 0) {
		load = div64_u64(delta_active_tsc * 100, delta_tsc);
		if (unlikely(load > 100))
			load = 100;
	} else
		load = 0;

#ifdef CONFIG_COUNT_GPU_BLOCKING_TIME
	/*
	 * if this sampling occurs at the same time when all logical cores
	 * are in idle, accumulate GPU blocking timer from last block start
	 * time.
	 */
	tmp_block_start = pphycore->gpu_block_start_tsc;
	if (tmp_block_start > 0 && tmp_block_start < tsc &&
	    tmp_block_start == pphycore->gpu_block_start_tsc &&
	    tmp_block_start == cmpxchg64(&pphycore->gpu_block_start_tsc,
					 tmp_block_start, tsc))
		pphycore->gpu_block_time += tsc - tmp_block_start;
	/*
	 * compute GPU blocking time since last sampling,
	 * and add the blocking time to CPU active time for
	 * load computing
	 */
	delta_gpu_block_time = pphycore->gpu_block_time -
		this_cpu->prev_gpu_block_time;
	this_cpu->prev_gpu_block_time = pphycore->gpu_block_time;
	*gpu_block_load = div64_u64(delta_gpu_block_time * 100, delta_tsc);
#endif

	return (unsigned int)load;
}
#endif

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
	if (cpu_has(c, X86_FEATURE_APERFMPERF))
		sfi_cpufreq_driver.getavg = cpufreq_get_measured_perf;

#ifdef CONFIG_COMPUTE_PHYSICAL_CORE_LOAD
	if (smt_capable())
		sfi_cpufreq_driver.getload = cpufreq_get_load;
#endif

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
		kfree(data->freq_table);
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
		per_cpu(sfi_processors, i) = pr;
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
