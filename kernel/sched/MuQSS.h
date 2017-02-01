#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/interrupt.h>
#include <linux/skip_list.h>
#include <linux/stop_machine.h>
#include <linux/u64_stats_sync.h>
#include "cpuacct.h"

#ifndef MUQSS_SCHED_H
#define MUQSS_SCHED_H

#ifdef CONFIG_SCHED_DEBUG
#define SCHED_WARN_ON(x)	WARN_ONCE(x, #x)
#else
#define SCHED_WARN_ON(x)	((void)(x))
#endif

/* task_struct::on_rq states: */
#define TASK_ON_RQ_QUEUED	1
#define TASK_ON_RQ_MIGRATING	2

/*
 * This is the main, per-CPU runqueue data structure.
 * This data should only be modified by the local cpu.
 */
struct rq {
	struct task_struct *curr, *idle, *stop;
	struct mm_struct *prev_mm;

	raw_spinlock_t lock;

	/* Stored data about rq->curr to work outside rq lock */
	u64 rq_deadline;
	int rq_prio;

	/* Best queued id for use outside lock */
	u64 best_key;

	unsigned long last_scheduler_tick; /* Last jiffy this RQ ticked */
	unsigned long last_jiffy; /* Last jiffy this RQ updated rq clock */
	u64 niffies; /* Last time this RQ updated rq clock */
	u64 last_niffy; /* Last niffies as updated by local clock */
	u64 last_jiffy_niffies; /* Niffies @ last_jiffy */

	u64 load_update; /* When we last updated load */
	unsigned long load_avg; /* Rolling load average */
#ifdef CONFIG_SMT_NICE
	struct mm_struct *rq_mm;
	int rq_smt_bias; /* Policy/nice level bias across smt siblings */
#endif
	/* Accurate timekeeping data */
	unsigned long user_ns, nice_ns, irq_ns, softirq_ns, system_ns,
		iowait_ns, idle_ns;
	atomic_t nr_iowait;

	skiplist_node node;
	skiplist *sl;
#ifdef CONFIG_SMP
	struct task_struct *preempt; /* Preempt triggered on this task */

	int cpu;		/* cpu of this runqueue */
	bool online;

	struct root_domain *rd;
	struct sched_domain *sd;
	int *cpu_locality; /* CPU relative cache distance */
	struct rq **rq_order; /* RQs ordered by relative cache distance */

#ifdef CONFIG_SCHED_SMT
	cpumask_t thread_mask;
	bool (*siblings_idle)(struct rq *rq);
	/* See if all smt siblings are idle */
#endif /* CONFIG_SCHED_SMT */
#ifdef CONFIG_SCHED_MC
	cpumask_t core_mask;
	bool (*cache_idle)(struct rq *rq);
	/* See if all cache siblings are idle */
#endif /* CONFIG_SCHED_MC */
#endif /* CONFIG_SMP */
#ifdef CONFIG_IRQ_TIME_ACCOUNTING
	u64 prev_irq_time;
#endif /* CONFIG_IRQ_TIME_ACCOUNTING */
#ifdef CONFIG_PARAVIRT
	u64 prev_steal_time;
#endif /* CONFIG_PARAVIRT */
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
	u64 prev_steal_time_rq;
#endif /* CONFIG_PARAVIRT_TIME_ACCOUNTING */

	u64 clock, old_clock, last_tick;
	u64 clock_task;
	int dither;

	int iso_ticks;
	bool iso_refractory;

#ifdef CONFIG_HIGH_RES_TIMERS
	struct hrtimer hrexpiry_timer;
#endif

#ifdef CONFIG_SCHEDSTATS

	/* latency stats */
	struct sched_info rq_sched_info;
	unsigned long long rq_cpu_time;
	/* could above be rq->cfs_rq.exec_clock + rq->rt_rq.rt_runtime ? */

	/* sys_sched_yield() stats */
	unsigned int yld_count;

	/* schedule() stats */
	unsigned int sched_switch;
	unsigned int sched_count;
	unsigned int sched_goidle;

	/* try_to_wake_up() stats */
	unsigned int ttwu_count;
	unsigned int ttwu_local;
#endif /* CONFIG_SCHEDSTATS */

#ifdef CONFIG_SMP
	struct llist_head wake_list;
#endif

#ifdef CONFIG_CPU_IDLE
	/* Must be inspected within a rcu lock section */
	struct cpuidle_state *idle_state;
#endif
};

#ifdef CONFIG_SMP
struct rq *cpu_rq(int cpu);
#endif

#ifndef CONFIG_SMP
extern struct rq *uprq;
#define cpu_rq(cpu)	(uprq)
#define this_rq()	(uprq)
#define raw_rq()	(uprq)
#define task_rq(p)	(uprq)
#define cpu_curr(cpu)	((uprq)->curr)
#else /* CONFIG_SMP */
DECLARE_PER_CPU_SHARED_ALIGNED(struct rq, runqueues);
#define this_rq()		this_cpu_ptr(&runqueues)
#define raw_rq()		raw_cpu_ptr(&runqueues)
#endif /* CONFIG_SMP */

/*
 * {de,en}queue flags:
 *
 * DEQUEUE_SLEEP  - task is no longer runnable
 * ENQUEUE_WAKEUP - task just became runnable
 *
 * SAVE/RESTORE - an otherwise spurious dequeue/enqueue, done to ensure tasks
 *                are in a known state which allows modification. Such pairs
 *                should preserve as much state as possible.
 *
 * MOVE - paired with SAVE/RESTORE, explicitly does not preserve the location
 *        in the runqueue.
 *
 * ENQUEUE_HEAD      - place at front of runqueue (tail if not specified)
 * ENQUEUE_REPLENISH - CBS (replenish runtime and postpone deadline)
 * ENQUEUE_MIGRATED  - the task was migrated during wakeup
 *
 */

#define DEQUEUE_SLEEP		0x01
#define DEQUEUE_SAVE		0x02 /* matches ENQUEUE_RESTORE */
#define DEQUEUE_MOVE		0x04 /* matches ENQUEUE_MOVE */

#define ENQUEUE_WAKEUP		0x01
#define ENQUEUE_RESTORE		0x02
#define ENQUEUE_MOVE		0x04

#define ENQUEUE_HEAD		0x08
#define ENQUEUE_REPLENISH	0x10
#ifdef CONFIG_SMP
#define ENQUEUE_MIGRATED	0x20
#else
#define ENQUEUE_MIGRATED	0x00
#endif

static inline u64 __rq_clock_broken(struct rq *rq)
{
	return READ_ONCE(rq->clock);
}

static inline u64 rq_clock(struct rq *rq)
{
	lockdep_assert_held(&rq->lock);
	return rq->clock;
}

static inline u64 rq_clock_task(struct rq *rq)
{
	lockdep_assert_held(&rq->lock);
	return rq->clock_task;
}

extern struct mutex sched_domains_mutex;
extern struct static_key_false sched_schedstats;

#define rcu_dereference_check_sched_domain(p) \
	rcu_dereference_check((p), \
			      lockdep_is_held(&sched_domains_mutex))

#define for_each_lower_domain(sd) for (; sd; sd = sd->child)

/*
 * The domain tree (rq->sd) is protected by RCU's quiescent state transition.
 * See detach_destroy_domains: synchronize_sched for details.
 *
 * The domain tree of any CPU may only be accessed from within
 * preempt-disabled sections.
 */
#define for_each_domain(cpu, __sd) \
	for (__sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd); __sd; __sd = __sd->parent)

#if defined(CONFIG_SCHED_DEBUG) && defined(CONFIG_SYSCTL)
void register_sched_domain_sysctl(void);
void unregister_sched_domain_sysctl(void);
#else
static inline void register_sched_domain_sysctl(void)
{
}
static inline void unregister_sched_domain_sysctl(void)
{
}
#endif

#ifdef CONFIG_SMP
extern void sched_ttwu_pending(void);
extern void set_cpus_allowed_common(struct task_struct *p, const struct cpumask *new_mask);
#else
static inline void sched_ttwu_pending(void) { }
#endif

#ifdef CONFIG_CPU_IDLE
static inline void idle_set_state(struct rq *rq,
				  struct cpuidle_state *idle_state)
{
	rq->idle_state = idle_state;
}

static inline struct cpuidle_state *idle_get_state(struct rq *rq)
{
	SCHED_WARN_ON(!rcu_read_lock_held());
	return rq->idle_state;
}
#else
static inline void idle_set_state(struct rq *rq,
				  struct cpuidle_state *idle_state)
{
}

static inline struct cpuidle_state *idle_get_state(struct rq *rq)
{
	return NULL;
}
#endif

#ifdef CONFIG_IRQ_TIME_ACCOUNTING
struct irqtime {
	u64			hardirq_time;
	u64			softirq_time;
	u64			irq_start_time;
	struct u64_stats_sync	sync;
};

DECLARE_PER_CPU(struct irqtime, cpu_irqtime);

static inline u64 irq_time_read(int cpu)
{
	struct irqtime *irqtime = &per_cpu(cpu_irqtime, cpu);
	unsigned int seq;
	u64 total;

	do {
		seq = __u64_stats_fetch_begin(&irqtime->sync);
		total = irqtime->softirq_time + irqtime->hardirq_time;
	} while (__u64_stats_fetch_retry(&irqtime->sync, seq));

	return total;
}
#endif /* CONFIG_IRQ_TIME_ACCOUNTING */

#ifdef CONFIG_CPU_FREQ
DECLARE_PER_CPU(struct update_util_data *, cpufreq_update_util_data);

static inline void cpufreq_trigger(u64 time, unsigned int flags)
{
       struct update_util_data *data = rcu_dereference_sched(*this_cpu_ptr(&cpufreq_update_util_data));

       if (data)
               data->func(data, time, flags);
}
#else
static inline void cpufreq_trigger(u64 time, unsigned int flag)
{
}
#endif /* CONFIG_CPU_FREQ */

#ifdef arch_scale_freq_capacity
#ifndef arch_scale_freq_invariant
#define arch_scale_freq_invariant()	(true)
#endif
#else /* arch_scale_freq_capacity */
#define arch_scale_freq_invariant()	(false)
#endif

/*
 * This should only be called when current == rq->idle. Dodgy workaround for
 * when softirqs are pending and we are in the idle loop. Setting current to
 * resched will kick us out of the idle loop and the softirqs will be serviced
 * on our next pass through schedule().
 */
static inline bool softirq_pending(int cpu)
{
	if (likely(!local_softirq_pending()))
		return false;
	set_tsk_need_resched(current);
	return true;
}

#ifdef CONFIG_64BIT
static inline u64 read_sum_exec_runtime(struct task_struct *t)
{
	return tsk_seruntime(t);
}
#else
struct rq *task_rq_lock(struct task_struct *p, unsigned long *flags);
void task_rq_unlock(struct rq *rq, struct task_struct *p, unsigned long *flags);

static inline u64 read_sum_exec_runtime(struct task_struct *t)
{
	unsigned long flags;
	u64 ns;
	struct rq *rq;

	rq = task_rq_lock(t, &flags);
	ns = tsk_seruntime(t);
	task_rq_unlock(rq, t, &flags);

	return ns;
}
#endif

#endif /* MUQSS_SCHED_H */
