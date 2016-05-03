/*
 *  kernel/sched/bfs.c, was kernel/sched.c
 *
 *  Kernel scheduler and related syscalls
 *
 *  Copyright (C) 1991-2002  Linus Torvalds
 *
 *  1996-12-23  Modified by Dave Grothe to fix bugs in semaphores and
 *		make semaphores SMP safe
 *  1998-11-19	Implemented schedule_timeout() and related stuff
 *		by Andrea Arcangeli
 *  2002-01-04	New ultra-scalable O(1) scheduler by Ingo Molnar:
 *		hybrid priority-list and round-robin design with
 *		an array-switch method of distributing timeslices
 *		and per-CPU runqueues.  Cleanups and useful suggestions
 *		by Davide Libenzi, preemptible kernel bits by Robert Love.
 *  2003-09-03	Interactivity tuning by Con Kolivas.
 *  2004-04-02	Scheduler domains code by Nick Piggin
 *  2007-04-15  Work begun on replacing all interactivity tuning with a
 *              fair scheduling design by Con Kolivas.
 *  2007-05-05  Load balancing (smp-nice) and other improvements
 *              by Peter Williams
 *  2007-05-06  Interactivity improvements to CFS by Mike Galbraith
 *  2007-07-01  Group scheduling enhancements by Srivatsa Vaddagiri
 *  2007-11-29  RT balancing improvements by Steven Rostedt, Gregory Haskins,
 *              Thomas Gleixner, Mike Kravetz
 *  now		Brainfuck deadline scheduling policy by Con Kolivas deletes
 *              a whole lot of those previous things.
 */

#include <linux/kasan.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/highmem.h>
#include <asm/mmu_context.h>
#include <linux/interrupt.h>
#include <linux/capability.h>
#include <linux/completion.h>
#include <linux/kernel_stat.h>
#include <linux/debug_locks.h>
#include <linux/perf_event.h>
#include <linux/security.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/freezer.h>
#include <linux/vmalloc.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/timer.h>
#include <linux/rcupdate.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/syscalls.h>
#include <linux/times.h>
#include <linux/tsacct_kern.h>
#include <linux/kprobes.h>
#include <linux/delayacct.h>
#include <linux/tick.h>
#include <linux/ftrace.h>
#include <linux/slab.h>
#include <linux/init_task.h>
#include <linux/binfmts.h>
#include <linux/context_tracking.h>
#include <linux/sched/prio.h>

#include <asm/switch_to.h>
#include <asm/tlb.h>
#include <asm/irq_regs.h>
#include <asm/mutex.h>
#ifdef CONFIG_PARAVIRT
#include <asm/paravirt.h>
#endif

#include "bfs_sched.h"
#include "../workqueue_internal.h"
#include "../smpboot.h"

#define CREATE_TRACE_POINTS
#include <trace/events/sched.h>

#include "cpupri.h"

#define rt_prio(prio)		unlikely((prio) < MAX_RT_PRIO)
#define rt_task(p)		rt_prio((p)->prio)
#define rt_queue(rq)		rt_prio((rq)->rq_prio)
#define batch_task(p)		(unlikely((p)->policy == SCHED_BATCH))
#define is_rt_policy(policy)	((policy) == SCHED_FIFO || \
					(policy) == SCHED_RR)
#define has_rt_policy(p)	unlikely(is_rt_policy((p)->policy))

#define is_idle_policy(policy)	((policy) == SCHED_IDLEPRIO)
#define idleprio_task(p)	unlikely(is_idle_policy((p)->policy))
#define task_running_idle(p)	unlikely((p)->prio == IDLE_PRIO)
#define idle_queue(rq)		(unlikely(is_idle_policy((rq)->rq_policy)))

/* is_iso_policy() and iso_task() are moved to include/linux/sched.h */
#define iso_queue(rq)		unlikely(is_iso_policy((rq)->rq_policy))
#define task_running_iso(p)	unlikely((p)->prio == ISO_PRIO)
#define rq_running_iso(rq)	((rq)->rq_prio == ISO_PRIO)

#define rq_idle(rq)		((rq)->rq_prio == PRIO_LIMIT)

#define ISO_PERIOD		((5 * HZ * grq.noc) + 1)

#define SCHED_PRIO(p)		((p) + MAX_RT_PRIO)
#define STOP_PRIO		(MAX_RT_PRIO - 1)

/*
 * Some helpers for converting to/from various scales. Use shifts to get
 * approximate multiples of ten for less overhead.
 */
#define JIFFIES_TO_NS(TIME)	((TIME) * (1000000000 / HZ))
#define JIFFY_NS		(1000000000 / HZ)
#define HALF_JIFFY_NS		(1000000000 / HZ / 2)
#define HALF_JIFFY_US		(1000000 / HZ / 2)
#define MS_TO_NS(TIME)		((TIME) << 20)
#define MS_TO_US(TIME)		((TIME) << 10)
#define NS_TO_MS(TIME)		((TIME) >> 20)
#define NS_TO_US(TIME)		((TIME) >> 10)

#define RESCHED_US	(100) /* Reschedule if less than this many μs left */

/*
 * rq->on_cpu states
 */
#define NOT_ON_CPU	0
#define ON_CPU		1
#define ON_CPU_RQ	2

void print_scheduler_version(void)
{
	printk(KERN_INFO "BFS CPU scheduler v0.469 by Con Kolivas.\n");
	printk(KERN_INFO "BFS enhancement patchset v4.5_0469_1 by Alfred Chen.\n");
}

/* BFS default rr interval in ms */
#define DEFAULT_RR_INTERVAL (6)

/*
 * This is the time all tasks within the same priority round robin.
 * Value is in ms and set to a minimum of 6ms. Scales with number of cpus.
 * Tunable via /proc interface.
 */
int rr_interval __read_mostly = DEFAULT_RR_INTERVAL;

/* Unlimited cached task wait time in ms */
#define UNLIMITED_CACHED_WAITTIME (1000)

/*
 * Normal policy task cached wait time, based on Preemption Model Kernel config
 */
#ifdef CONFIG_PREEMPT_NONE
#define NORMAL_POLICY_CACHED_WAITTIME UNLIMITED_CACHED_WAITTIME
#else
#define NORMAL_POLICY_CACHED_WAITTIME DEFAULT_RR_INTERVAL
#endif

/*
 * task policy cached timeout (in ns)
 */
static unsigned long policy_cached_timeout[] = {
	MS_TO_NS(NORMAL_POLICY_CACHED_WAITTIME),	/* NORMAL */
	MS_TO_NS(DEFAULT_RR_INTERVAL),			/* FIFO */
	MS_TO_NS(DEFAULT_RR_INTERVAL),			/* RR */
	MS_TO_NS(UNLIMITED_CACHED_WAITTIME),		/* BATCH */
	MS_TO_NS(DEFAULT_RR_INTERVAL),			/* ISO */
	MS_TO_NS(UNLIMITED_CACHED_WAITTIME)		/* IDLE */
};

/*
 * sched_iso_cpu - sysctl which determines the cpu percentage SCHED_ISO tasks
 * are allowed to run five seconds as real time tasks. This is the total over
 * all online cpus.
 */
int sched_iso_cpu __read_mostly = 70;

/*
 * The relative length of deadline for each priority(nice) level.
 */
static int prio_ratios[NICE_WIDTH] __read_mostly;

/*
 * The quota handed out to tasks of all priority levels when refilling their
 * time_slice.
 */
static inline int timeslice(void)
{
	return MS_TO_US(rr_interval);
}

/*
 * The global runqueue data that all CPUs work off. Data is protected either
 * by the global grq lock, or the discrete lock that precedes the data in this
 * struct.
 */
struct global_rq {
	raw_spinlock_t lock;
	unsigned long nr_running;
	unsigned long nr_uninterruptible;
	struct list_head queue[PRIO_LIMIT];
	DECLARE_BITMAP(prio_bitmap, PRIO_LIMIT + 1);
	unsigned long qnr; /* queued not running */
#ifdef CONFIG_SMP
	cpumask_t cpu_idle_map;
	cpumask_t non_scaled_cpumask;
#ifndef CONFIG_64BIT
	raw_spinlock_t priodl_lock;
#endif
#endif
	u64 rq_priodls[NR_CPUS];
	int noc; /* num_online_cpus stored and updated when it changes */

	raw_spinlock_t iso_lock;
	int iso_ticks;
	bool iso_refractory;
};

#ifdef CONFIG_SMP
/*
 * We add the notion of a root-domain which will be used to define per-domain
 * variables. Each exclusive cpuset essentially defines an island domain by
 * fully partitioning the member cpus from any other cpuset. Whenever a new
 * exclusive cpuset is created, we also create and attach a new root-domain
 * object.
 *
 */
struct root_domain {
	atomic_t refcount;
	atomic_t rto_count;
	struct rcu_head rcu;
	cpumask_var_t span;
	cpumask_var_t online;

	/*
	 * The "RT overload" flag: it gets set if a CPU has more than
	 * one runnable RT task.
	 */
	cpumask_var_t rto_mask;
	struct cpupri cpupri;
};

/*
 * By default the system creates a single root-domain with all cpus as
 * members (mimicking the global state we have today).
 */
static struct root_domain def_root_domain;

#endif /* CONFIG_SMP */

/* There can be only one */
#ifdef CONFIG_SMP
static struct global_rq grq ____cacheline_aligned_in_smp;
#else
static struct global_rq grq ____cacheline_aligned;
#endif

static DEFINE_MUTEX(sched_hotcpu_mutex);

/* cpus with isolated domains */
cpumask_var_t cpu_isolated_map;

DEFINE_PER_CPU_SHARED_ALIGNED(struct rq, runqueues);
#ifdef CONFIG_SMP
/*
 * sched_domains_mutex serialises calls to init_sched_domains,
 * detach_destroy_domains and partition_sched_domains.
 */
DEFINE_MUTEX(sched_domains_mutex);

/*
 * By default the system creates a single root-domain with all cpus as
 * members (mimicking the global state we have today).
 */
static struct root_domain def_root_domain;

int __weak arch_sd_sibling_asym_packing(void)
{
       return 0*SD_ASYM_PACKING;
}
#else
struct rq *uprq;
#endif /* CONFIG_SMP */

#ifdef CONFIG_SMP
static inline int cpu_of(struct rq *rq)
{
	return rq->cpu;
}

#else /* CONFIG_SMP */
static inline int cpu_of(struct rq *rq)
{
	return 0;
}
#endif

#include "stats.h"

#ifndef prepare_arch_switch
# define prepare_arch_switch(next)	do { } while (0)
#endif
#ifndef finish_arch_switch
# define finish_arch_switch(prev)	do { } while (0)
#endif
#ifndef finish_arch_post_lock_switch
# define finish_arch_post_lock_switch()	do { } while (0)
#endif

/*
 * All common locking functions performed on grq.lock. rq->clock is local to
 * the CPU accessing it so it can be modified just with interrupts disabled
 * when we're not updating niffies.
 * Looking up task_rq must be done under grq.lock to be safe.
 */
static void update_rq_clock_task(struct rq *rq, s64 delta);

static inline void update_rq_clock(struct rq *rq)
{
	s64 delta = sched_clock_cpu(cpu_of(rq)) - rq->clock;

	if (unlikely(delta <= 0))
		return;
	rq->clock += delta;
	update_rq_clock_task(rq, delta);
}

static inline bool task_running(struct task_struct *p)
{
	return (ON_CPU == p->on_cpu);
}

/*
 * double_rq_lock - safely lock two runqueues
 *
 * Note this does not disable interrupts like task_rq_lock,
 * you need to do so manually before calling.
 */
static inline void double_rq_lock(struct rq *rq1, struct rq *rq2)
	__acquires(rq1->lock)
	__acquires(rq2->lock)
{
	BUG_ON(!irqs_disabled());
	if (rq1 == rq2) {
		raw_spin_lock(&rq1->lock);
		__acquire(rq2->lock);	/* Fake it out ;) */
	} else {
		if (rq1 < rq2) {
			raw_spin_lock(&rq1->lock);
			raw_spin_lock_nested(&rq2->lock, SINGLE_DEPTH_NESTING);
		} else {
			raw_spin_lock(&rq2->lock);
			raw_spin_lock_nested(&rq1->lock, SINGLE_DEPTH_NESTING);
		}
	}
}

/*
 * double_rq_unlock - safely unlock two runqueues
 *
 * Note this does not restore interrupts like task_rq_unlock,
 * you need to do so manually after calling.
 */
static inline void double_rq_unlock(struct rq *rq1, struct rq *rq2)
	__releases(rq1->lock)
	__releases(rq2->lock)
{
	raw_spin_unlock(&rq1->lock);
	if (rq1 != rq2)
		raw_spin_unlock(&rq2->lock);
	else
		__release(rq2->lock);
}

static inline void _grq_lock(void)
	__acquires(grq.lock)
{
	raw_spinlock_t *lock = &grq.lock;
	spin_acquire(&lock->dep_map, 0, 0, _RET_IP_);
	LOCK_CONTENDED(lock, do_raw_spin_trylock, do_raw_spin_lock);
}

static inline void _grq_unlock(void)
	__releases(grq.lock)
{
	raw_spinlock_t *lock = &grq.lock;
	spin_release(&lock->dep_map, 1, _RET_IP_);
	do_raw_spin_unlock(lock);
}

static inline void grq_lock(void)
	__acquires(grq.lock)
{
	raw_spin_lock(&grq.lock);
}

static inline void grq_unlock(void)
	__releases(grq.lock)
{
	raw_spin_unlock(&grq.lock);
}

static inline void
rq_grq_lock_irqsave(struct rq *rq, unsigned long *flags)
	__acquires(rq->lock)
	__acquires(grq.lock)
{
	raw_spin_lock_irqsave(&rq->lock, *flags);
	raw_spin_lock(&grq.lock);
}

static inline void
rq_grq_unlock_irqrestore(struct rq *rq, unsigned long *flags)
	__releases(grq.lock)
	__releases(rq->lock)
{
	raw_spin_unlock(&grq.lock);
	raw_spin_unlock_irqrestore(&rq->lock, *flags);
}

/*
 * A task that is queued but not running will be on the grq run list.
 * A task that is not running or queued will not be on the grq run list.
 * A task that is currently running will have ->on_cpu set but not on the
 * grq run list.
 */
static inline bool task_queued(struct task_struct *p)
{
	return (!list_empty(&p->run_list));
}

/*
 * Context: p->pi_lock
 */
static inline struct rq
*__task_access_lock(struct task_struct *p, raw_spinlock_t **plock)
{
	struct rq *rq;
	for (;;) {
		rq = task_rq(p);
		if (p->on_cpu) {
			raw_spin_lock(&rq->lock);
			if (likely(p->on_cpu && rq == task_rq(p))) {
				*plock = &rq->lock;
				return rq;
			}
			raw_spin_unlock(&rq->lock);
		} else if (task_queued(p)) {
			raw_spin_lock(&grq.lock);
			if (likely(!p->on_cpu && task_queued(p) && rq == task_rq(p))) {
				*plock = &grq.lock;
				return rq;
			}
			raw_spin_unlock(&grq.lock);
		} else {
			*plock = NULL;
			return rq;
		}
	}
}

static inline void
__task_access_unlock(raw_spinlock_t *lock)
{
	if (NULL != lock)
		raw_spin_unlock(lock);
}

static inline struct rq
*task_access_lock_irqsave(struct task_struct *p, raw_spinlock_t **plock, unsigned long *flags)
{
	struct rq *rq;
	for (;;) {
		rq = task_rq(p);
		if (p->on_cpu) {
			raw_spin_lock_irqsave(&rq->lock, *flags);
			if (likely(p->on_cpu && rq == task_rq(p))) {
				*plock = &rq->lock;
				return rq;
			}
			raw_spin_unlock_irqrestore(&rq->lock, *flags);
		} else if (task_queued(p)) {
			raw_spin_lock_irqsave(&grq.lock, *flags);
			if (likely(!p->on_cpu && task_queued(p) && rq == task_rq(p))) {
				*plock = &grq.lock;
				return rq;
			}
			raw_spin_unlock_irqrestore(&grq.lock, *flags);
		} else {
			raw_spin_lock_irqsave(&p->pi_lock, *flags);
			if (likely(!p->on_cpu && !task_queued(p) && rq == task_rq(p))) {
				*plock = &p->pi_lock;
				return rq;
			}
			raw_spin_unlock_irqrestore(&p->pi_lock, *flags);
		}
	}
}

static inline void
task_access_unlock_irqrestore(raw_spinlock_t *lock, unsigned long *flags)
{
	raw_spin_unlock_irqrestore(lock, *flags);
}


static inline void prepare_lock_switch(struct rq *rq, struct task_struct *next)
{
}

static inline void finish_lock_switch(struct rq *rq, struct task_struct *prev)
{
	/*
	 * After ->on_cpu is cleared, the task would be locked on grq
	 * lock or pi_lock, We must ensure this doesn't happen until the
	 * switch is completely finished.
	 */
	smp_wmb();
	prev->on_cpu = NOT_ON_CPU;
#ifdef CONFIG_DEBUG_SPINLOCK
	/* this is a valid case when another task releases the spinlock */
	grq.lock.owner = current;
	rq->lock.owner = current;
#endif
	/*
	 * If we are tracking spinlock dependencies then we have to
	 * fix up the runqueue lock - which gets 'carried over' from
	 * prev into current:
	 */
	spin_acquire(&rq->lock.dep_map, 0, 0, _THIS_IP_);
	spin_acquire(&grq.lock.dep_map, 0, 0, _THIS_IP_);

	_grq_unlock();
	raw_spin_unlock_irq(&rq->lock);
}

static inline bool deadline_before(u64 deadline, u64 time)
{
	return (deadline < time);
}

static inline bool deadline_after(u64 deadline, u64 time)
{
	return (deadline > time);
}

static inline void update_task_priodl(struct task_struct *p)
{
	p->priodl = (((u64) (p->prio))<<56) | ((p->deadline)>>8);
}

#if defined(CONFIG_SMP) && !defined(CONFIG_64BIT)
static inline void grq_priodl_lock(void)
{
	raw_spin_lock(&grq.priodl_lock);
}

static inline void grq_priodl_unlock(void)
{
	raw_spin_unlock(&grq.priodl_lock);
}
#else
static inline void grq_priodl_lock(void)
{
}

static inline void grq_priodl_unlock(void)
{
}
#endif

/*
 * Removing from the global runqueue. Enter with grq locked.
 */
static inline void __dequeue_task(struct task_struct *p, int prio)
{
	list_del_init(&p->run_list);
	if (list_empty(grq.queue + prio))
		__clear_bit(prio, grq.prio_bitmap);
}

static void dequeue_task(struct task_struct *p)
{
	__dequeue_task(p, p->prio);
}

/*
 * To determine if it's safe for a task of SCHED_IDLEPRIO to actually run as
 * an idle task, we ensure none of the following conditions are met.
 */
static bool idleprio_suitable(struct task_struct *p)
{
	return (!freezing(p) && !signal_pending(p) &&
		!(task_contributes_to_load(p)) && !(p->flags & (PF_EXITING)));
}

/*
 * To determine if a task of SCHED_ISO can run in pseudo-realtime, we check
 * that the iso_refractory flag is not set.
 */
static bool isoprio_suitable(void)
{
	return !grq.iso_refractory;
}

/*
 * Adding to the global runqueue. Enter with grq locked.
 */
static void enqueue_task(struct task_struct *p, struct rq *rq)
{
	if (!rt_task(p)) {
		/* Check it hasn't gotten rt from PI */
		if ((idleprio_task(p) && idleprio_suitable(p)) ||
		   (iso_task(p) && isoprio_suitable()))
			p->prio = p->normal_prio;
		else
			p->prio = NORMAL_PRIO;
		update_task_priodl(p);
	}
	__set_bit(p->prio, grq.prio_bitmap);
	list_add_tail(&p->run_list, grq.queue + p->prio);
}

static inline void requeue_task(struct task_struct *p)
{
	sched_info_queued(task_rq(p), p);
}

/*
 * Returns the relative length of deadline all compared to the shortest
 * deadline which is that of nice -20.
 */
static inline int task_prio_ratio(struct task_struct *p)
{
	return prio_ratios[TASK_USER_PRIO(p)];
}

/*
 * task_timeslice - all tasks of all priorities get the exact same timeslice
 * length. CPU distribution is handled by giving different deadlines to
 * tasks of different priorities. Use 128 as the base value for fast shifts.
 */
static inline int task_timeslice(struct task_struct *p)
{
	return (rr_interval * task_prio_ratio(p) / 128);
}

/*
 * cmpxchg based fetch_or, macro so it works for different integer types
 */
#define fetch_or(ptr, val)						\
({	typeof(*(ptr)) __old, __val = *(ptr);				\
 	for (;;) {							\
 		__old = cmpxchg((ptr), __val, __val | (val));		\
 		if (__old == __val)					\
 			break;						\
 		__val = __old;						\
 	}								\
 	__old;								\
})

#if defined(CONFIG_SMP) && defined(TIF_POLLING_NRFLAG)
/*
 * Atomically set TIF_NEED_RESCHED and test for TIF_POLLING_NRFLAG,
 * this avoids any races wrt polling state changes and thereby avoids
 * spurious IPIs.
 */
static bool set_nr_and_not_polling(struct task_struct *p)
{
	struct thread_info *ti = task_thread_info(p);
	return !(fetch_or(&ti->flags, _TIF_NEED_RESCHED) & _TIF_POLLING_NRFLAG);
}

/*
 * Atomically set TIF_NEED_RESCHED if TIF_POLLING_NRFLAG is set.
 *
 * If this returns true, then the idle task promises to call
 * sched_ttwu_pending() and reschedule soon.
 */
static bool set_nr_if_polling(struct task_struct *p)
{
	struct thread_info *ti = task_thread_info(p);
	typeof(ti->flags) old, val = READ_ONCE(ti->flags);

	for (;;) {
		if (!(val & _TIF_POLLING_NRFLAG))
			return false;
		if (val & _TIF_NEED_RESCHED)
			return true;
		old = cmpxchg(&ti->flags, val, val | _TIF_NEED_RESCHED);
		if (old == val)
			break;
		val = old;
	}
	return true;
}

#else
static bool set_nr_and_not_polling(struct task_struct *p)
{
	set_tsk_need_resched(p);
	return true;
}

#ifdef CONFIG_SMP
static bool set_nr_if_polling(struct task_struct *p)
{
	return false;
}
#endif
#endif

static void resched_curr(struct rq *rq);

static inline void preempt_rq(struct rq * rq)
{
	unsigned long flags;

	if (rq) {
		raw_spin_lock_irqsave(&rq->lock, flags);
		resched_curr(rq);
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}
}

/*
 * qnr is the "queued but not running" count which is the total number of
 * tasks on the global runqueue list waiting for cpu time but not actually
 * currently running on a cpu.
 */
static inline void inc_qnr(void)
{
	grq.qnr++;
}

static inline void dec_qnr(void)
{
	grq.qnr--;
}

static inline int queued_notrunning(void)
{
	return grq.qnr;
}

#ifdef CONFIG_SMP
/*
 * The cpu_idle_map stores a bitmap of all the CPUs currently idle to
 * allow easy lookup of whether any suitable idle CPUs are available.
 * It's cheaper to maintain a binary yes/no if there are any idle CPUs on the
 * idle_cpus variable than to do a full bitmask check when we are busy.
 */
static inline void set_cpuidle_map(int cpu)
{
	if (likely(cpu_online(cpu)))
		cpumask_set_cpu(cpu, &grq.cpu_idle_map);
}

static inline void clear_cpuidle_map(int cpu)
{
	cpumask_clear_cpu(cpu, &grq.cpu_idle_map);
}

static inline bool suitable_idle_cpus(struct task_struct *p)
{
	return (cpumask_intersects(tsk_cpus_allowed(p), &grq.cpu_idle_map));
}

static inline bool scaling_rq(struct rq *rq);

/*
 * The best idle cpu is first-matched by the following cpumask list
 *
 * Non scaled same cpu as task originally runs on
 * Non scaled SMT of the cup
 * Non scaled cores/threads shares last level cache
 * Scaled same cpu as task originally runs on
 * Scaled SMT of the cup
 * Scaled cores/threads shares last level cache
 * Non scaled cores within the same physical cpu
 * Non scaled cpus/Cores within the local NODE
 * Scaled cores within the same physical cpu
 * Scaled cpus/Cores within the local NODE
 * All cpus avariable
 */

static inline int llc_cpu_check(int cpu, cpumask_t *cpumask, cpumask_t *res_mask)
{
	return (
#ifdef CONFIG_SCHED_SMT
	/* SMT of the cpu */
	cpumask_and(res_mask, cpumask, topology_sibling_cpumask(cpu)) ||
#endif

#ifdef CONFIG_SCHED_MC
	/* Cores shares last level cache */
	cpumask_and(res_mask, cpumask, cpu_coregroup_mask(cpu))
#else
	0
#endif
	);
}

static inline int nonllc_cpu_check(int cpu, cpumask_t *cpumask, cpumask_t *res_mask)
{
	return (
#ifdef CONFIG_SCHED_MC
	/* Cores within the same physical cpu */
	cpumask_and(res_mask, cpumask, topology_core_cpumask(cpu)) ||
#endif

	/* Cpus/Cores within the local NODE */
	cpumask_and(res_mask, cpumask, cpu_cpu_mask(cpu))
	);
}

static inline int best_mask_cpu(const int cpu, cpumask_t *cpumask)
{
	cpumask_t tmpmask, non_scaled_mask;
	cpumask_t *res_mask = &tmpmask;

	if (cpumask_and(&non_scaled_mask, cpumask, &grq.non_scaled_cpumask)) {
		/*
		 * non_scaled llc cpus checking
		 */
		if (llc_cpu_check(cpu, &non_scaled_mask, res_mask)) {
			if (cpumask_test_cpu(cpu, res_mask))
				return cpu;
			return cpumask_first(res_mask);
		}
		/*
		 * scaling llc cpus checking
		 */
		if (llc_cpu_check(cpu, cpumask, res_mask)) {
			if (cpumask_test_cpu(cpu, res_mask))
				return cpu;
			return cpumask_first(res_mask);
		}

		/*
		 * non_scaled non_llc cpus checking
		 */
		if (nonllc_cpu_check(cpu, &non_scaled_mask, res_mask))
			return cpumask_first(res_mask);
		/*
		 * scaling non_llc cpus checking
		 */
		if (nonllc_cpu_check(cpu, cpumask, res_mask))
			return cpumask_first(res_mask);

		/* All cpus avariable */

		return cpumask_first(cpumask);
	}

	/*
	 * scaling llc cpus checking
	 */
	if (llc_cpu_check(cpu, cpumask, res_mask)) {
		if (cpumask_test_cpu(cpu, res_mask))
			return cpu;
		return cpumask_first(res_mask);
	}

	/*
	 * scaling non_llc cpus checking
	 */
	if (nonllc_cpu_check(cpu, cpumask, res_mask))
		return cpumask_first(res_mask);

	/* All cpus avariable */

	return cpumask_first(cpumask);
}

void wake_up_if_idle(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	rcu_read_lock();

	if (!is_idle_task(rcu_dereference(rq->curr)))
		goto out;

	if (set_nr_if_polling(rq->idle)) {
		trace_sched_wake_idle_without_ipi(cpu);
	} else {
		raw_spin_lock_irqsave(&rq->lock, flags);
		if (is_idle_task(rq->curr))
			smp_send_reschedule(cpu);
		/* Else cpu is not in idle, do nothing here */
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

out:
	rcu_read_unlock();
}

bool cpus_share_cache(int this_cpu, int that_cpu)
{
	struct rq *this_rq = cpu_rq(this_cpu);

	return (this_rq->cpu_locality[that_cpu] < 3);
}

#ifdef CONFIG_SCHED_SMT
#ifdef CONFIG_SMT_NICE
static const cpumask_t *thread_cpumask(int cpu);

/* Find the best real time priority running on any SMT siblings of cpu and if
 * none are running, the static priority of the best deadline task running.
 * The lookups to the other runqueues is done lockless as the occasional wrong
 * value would be harmless. */
static int best_smt_bias(int cpu)
{
	int other_cpu, best_bias = 0;

	for_each_cpu(other_cpu, thread_cpumask(cpu)) {
		struct rq *rq;

		if (other_cpu == cpu)
			continue;
		rq = cpu_rq(other_cpu);
		if (rq_idle(rq))
			continue;
		if (!rq->online)
			continue;
		if (!rq->rq_mm)
			continue;
		if (likely(rq->rq_smt_bias > best_bias))
			best_bias = rq->rq_smt_bias;
	}
	return best_bias;
}

static int task_prio_bias(struct task_struct *p)
{
	if (rt_task(p))
		return 1 << 30;
	else if (task_running_iso(p))
		return 1 << 29;
	else if (task_running_idle(p))
		return 0;
	return MAX_PRIO - p->static_prio;
}

/* We've already decided p can run on CPU, now test if it shouldn't for SMT
 * nice reasons. */
static bool smt_should_schedule(struct task_struct *p, int cpu)
{
	int best_bias, task_bias;

	/* Kernel threads always run */
	if (unlikely(!p->mm))
		return true;
	if (rt_task(p))
		return true;
	if (!idleprio_suitable(p))
		return true;
	best_bias = best_smt_bias(cpu);
	/* The smt siblings are all idle or running IDLEPRIO */
	if (best_bias < 1)
		return true;
	task_bias = task_prio_bias(p);
	if (task_bias < 1)
		return false;
	if (task_bias >= best_bias)
		return true;
	/* Dither 25% cpu of normal tasks regardless of nice difference */
	if (best_bias % 4 == 1)
		return true;
	/* Sorry, you lose */
	return false;
}
#endif
#endif

static inline struct rq *task_best_idle_rq(struct task_struct *p)
{
        cpumask_t check_cpumask;

        if (cpumask_and(&check_cpumask, &p->cpus_allowed, &grq.cpu_idle_map)) {
		int best_cpu;

                best_cpu = best_mask_cpu(task_cpu(p), &check_cpumask);
#ifdef CONFIG_SMT_NICE
		if (!smt_should_schedule(p, best_cpu))
			return NULL;
#endif
		return cpu_rq(best_cpu);
	}

	return NULL;
}

/*
 * Flags to tell us whether this CPU is running a CPU frequency governor that
 * has slowed its speed or not. No locking required as the very rare wrongly
 * read value would be harmless.
 */
void cpu_scaling(int cpu)
{
	cpu_rq(cpu)->scaling = true;
	cpumask_clear_cpu(cpu, &grq.non_scaled_cpumask);
}

void cpu_nonscaling(int cpu)
{
	cpu_rq(cpu)->scaling = false;
	cpumask_set_cpu(cpu, &grq.non_scaled_cpumask);
}

static inline bool scaling_rq(struct rq *rq)
{
	return rq->scaling;
}

static inline int locality_diff(int cpu, struct rq *rq)
{
	return rq->cpu_locality[cpu];
}
#else /* CONFIG_SMP */
static inline void set_cpuidle_map(int cpu)
{
}

static inline void clear_cpuidle_map(int cpu)
{
}

static inline struct rq *task_best_idle_rq(struct task_struct *p)
{
	return NULL;
}

static inline bool suitable_idle_cpus(struct task_struct *p)
{
	return uprq->curr == uprq->idle;
}

void cpu_scaling(int __unused)
{
}

void cpu_nonscaling(int __unused)
{
}

/*
 * Although CPUs can scale in UP, there is nowhere else for tasks to go so this
 * always returns 0.
 */
static inline bool scaling_rq(struct rq *rq)
{
	return false;
}

static inline int locality_diff(int cpu, struct rq *rq)
{
	return 0;
}
#endif /* CONFIG_SMP */
EXPORT_SYMBOL_GPL(cpu_scaling);
EXPORT_SYMBOL_GPL(cpu_nonscaling);

static inline int normal_prio(struct task_struct *p)
{
	if (has_rt_policy(p))
		return MAX_RT_PRIO - 1 - p->rt_priority;
	if (idleprio_task(p))
		return IDLE_PRIO;
	if (iso_task(p))
		return ISO_PRIO;
	return NORMAL_PRIO;
}

/*
 * Calculate the current priority, i.e. the priority
 * taken into account by the scheduler. This value might
 * be boosted by RT tasks as it will be RT if the task got
 * RT-boosted. If not then it returns p->normal_prio.
 */
static int effective_prio(struct task_struct *p)
{
	p->normal_prio = normal_prio(p);
	/*
	 * If we are RT tasks or we were boosted to RT priority,
	 * keep the priority unchanged. Otherwise, update priority
	 * to the normal priority:
	 */
	if (!rt_prio(p->prio))
		return p->normal_prio;
	return p->prio;
}

/*
 * activate_task - move a task to the runqueue. Enter with grq locked.
 */
static void activate_task(struct task_struct *p, struct rq *rq)
{
	update_rq_clock(rq);

	/*
	 * Sleep time is in units of nanosecs, so shift by 20 to get a
	 * milliseconds-range estimation of the amount of time that the task
	 * spent sleeping:
	 */
	if (unlikely(prof_on == SLEEP_PROFILING)) {
		if (p->state == TASK_UNINTERRUPTIBLE)
			profile_hits(SLEEP_PROFILING, (void *)get_wchan(p),
				     (rq->clock_task - p->last_ran) >> 20);
	}

	p->prio = effective_prio(p);
	if (task_contributes_to_load(p))
		grq.nr_uninterruptible--;
	enqueue_task(p, rq);
	p->on_rq = 1;
	grq.nr_running++;
	inc_qnr();
}

/*
 * deactivate_task - If it's running, it's not on the grq and we can just
 * decrement the nr_running. Enter with grq locked.
 */
static inline void deactivate_task(struct task_struct *p, struct rq *rq)
{
	if (task_contributes_to_load(p))
		grq.nr_uninterruptible++;
	p->on_rq = 0;
	grq.nr_running--;
}

#ifdef CONFIG_SMP
void set_task_cpu(struct task_struct *p, unsigned int cpu)
{
#ifdef CONFIG_LOCKDEP
	/*
	 * The caller should hold grq lock or rq lock, release this checking
	 * atm.
	 */
	/*WARN_ON_ONCE(debug_locks && !lockdep_is_held(&grq.lock));*/
#endif
	if (task_cpu(p) == cpu)
		return;
	trace_sched_migrate_task(p, cpu);
	perf_event_task_migrate(p);

	/*
	 * After ->cpu is set up to a new value, task_access_lock(p, ...) can be
	 * successfully executed on another CPU. We must ensure that updates of
	 * per-task data have been completed by this moment.
	 */
	smp_wmb();

	task_thread_info(p)->cpu = cpu;
}
#endif

static inline void
set_cpus_allowed_common(struct task_struct *p, const struct cpumask *new_mask)
{
	cpumask_copy(&p->cpus_allowed_master, new_mask);
	if (likely(cpumask_and(&p->cpus_allowed,
			       &p->cpus_allowed_master, cpu_active_mask))) {
		p->nr_cpus_allowed = cpumask_weight(new_mask);
		return;
	}

	cpumask_set_cpu(0, &p->cpus_allowed);
	p->nr_cpus_allowed = 1;
}

void do_set_cpus_allowed(struct task_struct *p, const struct cpumask *new_mask)
{
	set_cpus_allowed_common(p, new_mask);
}

/*
 * We set the task cached when it is descheduled involuntarily meaning it is
 * awaiting further CPU time. Before caching time out, task will stick to
 * non_scaling cpu or its original cpu.
 * Realtime tasks don't use cache count to minimise their latency at all times.
 */
static inline void cache_task(struct task_struct *p, struct rq *rq)
{
	if(!rt_task(p) && p->mm) {
		p->cached = 1ULL;
		p->policy_cached_timeout = rq->clock_task +
			policy_cached_timeout[p->policy];
	}
}

static inline bool
is_task_policy_cached_timeout(struct task_struct *p, struct rq *rq)
{
	return (rq->clock_task > p->policy_cached_timeout);
}

static inline bool
is_task_should_cached_off(struct task_struct *p, struct rq *rq)
{
	return is_task_policy_cached_timeout(p, rq);
}

/*
 * Move a task off the global queue and take it to a cpu for it will
 * become the running task.
 */
static inline void take_task(int cpu, struct task_struct *p)
{
	set_task_cpu(p, cpu);
	/*
	 * We can optimise this out completely for !SMP, because the
	 * SMP rebalancing from interrupt is the only thing that cares
	 * here.
	 */
	p->on_cpu = ON_CPU;
	dequeue_task(p);
	dec_qnr();
}

/* Enter with rq lock held. We know p is on the local cpu */
static inline void __set_tsk_resched(struct task_struct *p)
{
	set_tsk_need_resched(p);
	set_preempt_need_resched();
}

/*
 * resched_curr - mark rq's current task 'to be rescheduled now'.
 *
 * On UP this means the setting of the need_resched flag, on SMP it
 * might also involve a cross-CPU call to trigger the scheduler on
 * the target CPU.
 */
void resched_curr(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	int cpu;

	lockdep_assert_held(&rq->lock);

	if (test_tsk_need_resched(curr))
		return;

	cpu = cpu_of(rq);
	if (cpu == smp_processor_id()) {
		set_tsk_need_resched(curr);
		set_preempt_need_resched();
		return;
	}

	if (set_nr_and_not_polling(curr))
		smp_send_reschedule(cpu);
	else
		trace_sched_wake_idle_without_ipi(cpu);
}

/**
 * task_curr - is this task currently executing on a CPU?
 * @p: the task in question.
 *
 * Return: 1 if the task is currently executing. 0 otherwise.
 */
inline int task_curr(const struct task_struct *p)
{
	return cpu_curr(task_cpu(p)) == p;
}

#ifdef CONFIG_SMP
struct migration_req {
	struct task_struct *task;
	int dest_cpu;
};

/*
 * wait_task_inactive - wait for a thread to unschedule.
 *
 * If @match_state is nonzero, it's the @p->state value just checked and
 * not expected to change.  If it changes, i.e. @p might have woken up,
 * then return zero.  When we succeed in waiting for @p to be off its CPU,
 * we return a positive number (its total switch count).  If a second call
 * a short while later returns the same number, the caller can be sure that
 * @p has remained unscheduled the whole time.
 *
 * The caller must ensure that the task *will* unschedule sometime soon,
 * else this function might spin for a *long* time. This function can't
 * be called with interrupts off, or it may introduce deadlock with
 * smp_call_function() if an IPI is sent by the same process we are
 * waiting to become inactive.
 */
unsigned long wait_task_inactive(struct task_struct *p, long match_state)
{
	unsigned long flags;
	bool running, on_rq;
	unsigned long ncsw;
	struct rq *rq;
	raw_spinlock_t *lock;

	for (;;) {
		rq = task_rq(p);

		/*
		 * If the task is actively running on another CPU
		 * still, just relax and busy-wait without holding
		 * any locks.
		 *
		 * NOTE! Since we don't hold any locks, it's not
		 * even sure that "rq" stays as the right runqueue!
		 * But we don't care, since this will return false
		 * if the runqueue has changed and p is actually now
		 * running somewhere else!
		 */
		while (task_running(p) && p == rq->curr) {
			if (match_state && unlikely(p->state != match_state))
				return 0;
			cpu_relax();
		}

		/*
		 * Ok, time to look more closely! We need the grq
		 * lock now, to be *sure*. If we're wrong, we'll
		 * just go back and repeat.
		 */
		/* Only access p, task_access_lock...() is good enough,
		 * most likely p already stops running and will lock
		 * on grq; if not, it will lock on it's rq and running
		 * will be false*/
		task_access_lock_irqsave(p, &lock, &flags);
		trace_sched_wait_task(p);
		running = task_running(p);
		on_rq = p->on_rq;
		ncsw = 0;
		if (!match_state || p->state == match_state)
			ncsw = p->nvcsw | LONG_MIN; /* sets MSB */
		task_access_unlock_irqrestore(lock, &flags);

		/*
		 * If it changed from the expected state, bail out now.
		 */
		if (unlikely(!ncsw))
			break;

		/*
		 * Was it really running after all now that we
		 * checked with the proper locks actually held?
		 *
		 * Oops. Go back and try again..
		 */
		if (unlikely(running)) {
			cpu_relax();
			continue;
		}

		/*
		 * It's not enough that it's not actively running,
		 * it must be off the runqueue _entirely_, and not
		 * preempted!
		 *
		 * So if it was still runnable (but just not actively
		 * running right now), it's preempted, and we should
		 * yield - it could be a while.
		 */
		if (unlikely(on_rq)) {
			ktime_t to = ktime_set(0, NSEC_PER_SEC / HZ);

			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_hrtimeout(&to, HRTIMER_MODE_REL);
			continue;
		}

		/*
		 * Ahh, all good. It wasn't running, and it wasn't
		 * runnable, which means that it will never become
		 * running in the future either. We're all done!
		 */
		break;
	}

	return ncsw;
}

/***
 * kick_process - kick a running thread to enter/exit the kernel
 * @p: the to-be-kicked thread
 *
 * Cause a process which is running on another CPU to enter
 * kernel-mode, without any delay. (to get signals handled.)
 *
 * NOTE: this function doesn't have to take the runqueue lock,
 * because all it wants to ensure is that the remote task enters
 * the kernel. If the IPI races and the task has been migrated
 * to another CPU then no harm is done and the purpose has been
 * achieved as well.
 */
void kick_process(struct task_struct *p)
{
	int cpu;

	preempt_disable();
	cpu = task_cpu(p);
	if ((cpu != smp_processor_id()) && task_curr(p))
		smp_send_reschedule(cpu);
	preempt_enable();
}
EXPORT_SYMBOL_GPL(kick_process);
#endif

/*
 * RT tasks preempt purely on priority. SCHED_NORMAL tasks preempt on the
 * basis of earlier deadlines. SCHED_IDLEPRIO don't preempt anything else or
 * between themselves, they cooperatively multitask. An idle rq scores as
 * prio PRIO_LIMIT so it is always preempted.
 */
static inline bool
can_preempt(struct task_struct *p, u64 priodl)
{
	return (p->priodl < priodl);
}

#ifdef CONFIG_SMP
/*
 * Check to see if p can run on cpu, and if not, whether there are any online
 * CPUs it can run on instead.
 */
static inline bool needs_other_cpu(struct task_struct *p, int cpu)
{
	return !cpumask_test_cpu(cpu, &p->cpus_allowed);
}

static struct rq* task_preemptable_rq(struct task_struct *p)
{
	int cpu, target_cpu;
	struct rq *target_rq;
	u64 highest_priodl;
	cpumask_t tmp;

	target_rq = task_best_idle_rq(p);
	if (target_rq)
		return target_rq;

	/* IDLEPRIO tasks never preempt anything but idle */
	if (p->policy == SCHED_IDLEPRIO)
		return NULL;

	if (unlikely(!cpumask_and(&tmp, cpu_online_mask, &p->cpus_allowed)))
		return NULL;

	target_cpu = cpu = cpumask_first(&tmp);

	grq_priodl_lock();
	highest_priodl = grq.rq_priodls[cpu];

	for(;cpu = cpumask_next(cpu, &tmp), cpu < nr_cpu_ids;) {
		u64 rq_priodl;

		rq_priodl = grq.rq_priodls[cpu];
		if (rq_priodl > highest_priodl ) {
			target_cpu = cpu;
			highest_priodl = rq_priodl;
		}
	}
	grq_priodl_unlock();

#ifdef CONFIG_SMT_NICE
	if (!smt_should_schedule(p, target_cpu))
		return NULL;
#endif
	if (can_preempt(p, highest_priodl))
		return cpu_rq(target_cpu);

	return NULL;
}
#else /* CONFIG_SMP */
static inline bool needs_other_cpu(struct task_struct *p, int cpu)
{
	return false;
}

static struct rq* task_preemptable_rq(struct task_struct *p)
{
	if (p->policy == SCHED_IDLEPRIO)
		return NULL;
	if (can_preempt(p, grq.rq_priodls[0]))
		return uprq;
	return NULL;
}
#endif /* CONFIG_SMP */

static void
ttwu_stat(struct task_struct *p, int cpu, int wake_flags)
{
#ifdef CONFIG_SCHEDSTATS
	struct rq *rq = this_rq();

#ifdef CONFIG_SMP
	int this_cpu = smp_processor_id();

	if (cpu == this_cpu)
		schedstat_inc(rq, ttwu_local);
	else {
		struct sched_domain *sd;

		rcu_read_lock();
		for_each_domain(this_cpu, sd) {
			if (cpumask_test_cpu(cpu, sched_domain_span(sd))) {
				schedstat_inc(sd, ttwu_wake_remote);
				break;
			}
		}
		rcu_read_unlock();
	}

#endif /* CONFIG_SMP */

	schedstat_inc(rq, ttwu_count);
#endif /* CONFIG_SCHEDSTATS */
}

#ifdef CONFIG_SMP
void scheduler_ipi(void)
{
	/*
	 * Fold TIF_NEED_RESCHED into the preempt_count; anybody setting
	 * TIF_NEED_RESCHED remotely (for the first time) will also send
	 * this IPI.
	 */
	preempt_fold_need_resched();
}
#endif

static inline void ttwu_activate(struct task_struct *p, struct rq *rq)
{
	activate_task(p, rq);

	/*
	 * if a worker is waking up, notify workqueue. Note that on BFS, we
	 * don't really know what cpu it will be, so we fake it for
	 * wq_worker_waking_up :/
	 */
	if (p->flags & PF_WQ_WORKER)
		wq_worker_waking_up(p, cpu_of(rq));
}

/*
 * Mark the task runnable and perform wakeup-preemption.
 */
static inline void
ttwu_do_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	p->state = TASK_RUNNING;
	trace_sched_wakeup(p);
}

/*
 * wake flags
 */
#define WF_SYNC		0x01		/* waker goes to sleep after wakeup */
#define WF_FORK		0x02		/* child wakeup after fork */
#define WF_MIGRATED	0x4		/* internal use, task got migrated */

/***
 * try_to_wake_up - wake up a thread
 * @p: the thread to be awakened
 * @state: the mask of task states that can be woken
 * @wake_flags: wake modifier flags (WF_*)
 *
 * Put it on the run-queue if it's not already there. The "current"
 * thread is always on the run-queue (except when the actual
 * re-schedule is in progress), and as such you're allowed to do
 * the simpler "current->state = TASK_RUNNING" to mark yourself
 * runnable without the overhead of this.
 *
 * Return: %true if @p was woken up, %false if it was already running.
 * or @state didn't match @p's state.
 */
static int try_to_wake_up(struct task_struct *p, unsigned int state,
			  int wake_flags)
{
	unsigned long flags;
	struct rq *rq, *prq;
	raw_spinlock_t *lock;
	int cpu, success = 0;

	/*
	 * If we are going to wake up a thread waiting for CONDITION we
	 * need to ensure that CONDITION=1 done by the caller can not be
	 * reordered with p->state check below. This pairs with mb() in
	 * set_current_state() the waiting thread does.
	 */
	smp_mb__before_spinlock();

	/*
	 * We only need to update the rq clock if we activate the task
	 */
	rq = task_access_lock_irqsave(p, &lock, &flags);

	/* state is a volatile long, どうして、分からない */
	if (!(p->state & state)) {
		task_access_unlock_irqrestore(lock, &flags);
		return 0;
	}

	trace_sched_waking(p);

	success = 1;
	if (unlikely(p->on_cpu || task_queued(p))) {
		ttwu_do_wakeup(rq, p, 0);
		cpu = task_cpu(p);
		ttwu_stat(p, cpu, wake_flags);
		task_access_unlock_irqrestore(lock, &flags);
		return success;
	}

	_grq_lock();

	ttwu_activate(p, rq);
	ttwu_do_wakeup(rq, p, 0);

	/*
	 * Sync wakeups (i.e. those types of wakeups where the waker
	 * has indicated that it will leave the CPU in short order)
	 * don't trigger a preemption if there are no idle cpus,
	 * instead waiting for current to deschedule.
	 */
	if (!(wake_flags & WF_SYNC) || suitable_idle_cpus(p))
		prq = task_preemptable_rq(p);
	else
		prq = NULL;

	_grq_unlock();

	cpu = task_cpu(p);
	ttwu_stat(p, cpu, wake_flags);
	task_access_unlock_irqrestore(lock, &flags);

	preempt_rq(prq);

	return success;
}

/**
 * try_to_wake_up_local - try to wake up a local task with rq lock held
 * @p: the thread to be awakened
 *
 * Put @p on the run-queue if it's not already there. The caller must
 * ensure that grq is locked and, @p is not the current task.
 * grq stays locked over invocation.
 */
static void try_to_wake_up_local(struct task_struct *p)
{
	struct rq *rq = task_rq(p);

	lockdep_assert_held(&grq.lock);

	if (!(p->state & TASK_NORMAL))
		return;

	trace_sched_waking(p);

	if (!task_queued(p)) {
		ttwu_activate(p, rq);
	}
	ttwu_do_wakeup(rq, p, 0);
	ttwu_stat(p, smp_processor_id(), 0);
}

/**
 * wake_up_process - Wake up a specific process
 * @p: The process to be woken up.
 *
 * Attempt to wake up the nominated process and move it to the set of runnable
 * processes.
 *
 * Return: 1 if the process was woken up, 0 if it was already running.
 *
 * It may be assumed that this function implies a write memory barrier before
 * changing the task state if and only if any tasks are woken up.
 */
int wake_up_process(struct task_struct *p)
{
	return try_to_wake_up(p, TASK_NORMAL, 0);
}
EXPORT_SYMBOL(wake_up_process);

int wake_up_state(struct task_struct *p, unsigned int state)
{
	return try_to_wake_up(p, state, 0);
}

static void time_slice_expired(struct task_struct *p, struct rq *rq);

/*
 * Perform scheduler related setup for a newly forked process p.
 * p is forked by current.
 */
int sched_fork(unsigned long __maybe_unused clone_flags, struct task_struct *p)
{
#ifdef CONFIG_PREEMPT_NOTIFIERS
	INIT_HLIST_HEAD(&p->preempt_notifiers);
#endif
	/*
	 * The process state is set to the same value of the process executing
	 * do_fork() code. That is running. This guarantees that nobody will
	 * actually run it, and a signal or other external event cannot wake
	 * it up and insert it on the runqueue either.
	 */

	/* Should be reset in fork.c but done here for ease of bfs patching */
	p->on_rq =
	p->utime =
	p->stime =
	p->utimescaled =
	p->stimescaled =
	p->sched_time =
	p->stime_pc =
	p->utime_pc = 0;

	/*
	 * Revert to default priority/policy on fork if requested.
	 */
	if (unlikely(p->sched_reset_on_fork)) {
		if (p->policy == SCHED_FIFO || p->policy == SCHED_RR) {
			p->policy = SCHED_NORMAL;
			p->normal_prio = normal_prio(p);
		}

		if (PRIO_TO_NICE(p->static_prio) < 0) {
			p->static_prio = NICE_TO_PRIO(0);
			p->normal_prio = p->static_prio;
		}

		/*
		 * We don't need the reset flag anymore after the fork. It has
		 * fulfilled its duty:
		 */
		p->sched_reset_on_fork = 0;
	}

	INIT_LIST_HEAD(&p->run_list);
#ifdef CONFIG_SCHED_INFO
	if (likely(sched_info_on()))
		memset(&p->sched_info, 0, sizeof(p->sched_info));
#endif
	p->on_cpu = NOT_ON_CPU;
	p->cached = 0ULL;
	init_task_preempt_count(p);
	return 0;
}

/*
 * wake_up_new_task - wake up a newly created task for the first time.
 *
 * This function will do some initial scheduler statistics housekeeping
 * that must be done for every newly created context, then puts the task
 * on the runqueue and wakes it.
 */
void wake_up_new_task(struct task_struct *p)
{
	struct task_struct *parent;
	unsigned long flags;
	struct rq *rq, *prq = NULL;

retry:
	rq = task_rq(p);
	raw_spin_lock_irqsave(&rq->lock, flags);
	if (unlikely(rq != task_rq(p))) {
		raw_spin_unlock_irqrestore(&rq->lock, flags);
		goto retry;
	}

	parent = p->parent;

#ifdef CONFIG_SMP
	set_task_cpu(p, rq->cpu);
#endif

	/*
	 * Reinit new task deadline as its creator deadline could have changed
	 * since call to dup_task_struct().
	 */
	p->deadline = rq->rq_deadline;

	/*
	 * Make sure we do not leak PI boosting priority to the child.
	 */
	p->prio = rq->curr->normal_prio;

	update_task_priodl(p);

	trace_sched_wakeup_new(p);
	if (unlikely(p->policy == SCHED_FIFO))
		goto after_ts_init;

	/*
	 * Share the timeslice between parent and child, thus the
	 * total amount of pending timeslices in the system doesn't change,
	 * resulting in more scheduling fairness. If it's negative, it won't
	 * matter since that's the same as being 0. current's time_slice is
	 * actually in rq_time_slice when it's running, as is its last_ran
	 * value. rq->rq_deadline is only modified within schedule() so it
	 * is always equal to current->deadline.
	 */
	p->last_ran = rq->rq_last_ran;
	if (likely(rq->rq_time_slice >= RESCHED_US * 2)) {
		rq->rq_time_slice /= 2;
		p->time_slice = rq->rq_time_slice;
after_ts_init:
		if (rq->curr == parent && !suitable_idle_cpus(p)) {
			/*
			 * The VM isn't cloned, so we're in a good position to
			 * do child-runs-first in anticipation of an exec. This
			 * usually avoids a lot of COW overhead.
			 */
			__set_tsk_resched(parent);
		} else
			prq = task_preemptable_rq(p);
	} else {
		if (rq->curr == parent) {
			/*
		 	* Forking task has run out of timeslice. Reschedule it and
		 	* start its child with a new time slice and deadline. The
		 	* child will end up running first because its deadline will
		 	* be slightly earlier.
		 	*/
			rq->rq_time_slice = 0;
			__set_tsk_resched(parent);
		}
		time_slice_expired(p, rq);
	}

	_grq_lock();
	activate_task(p, rq);
	_grq_unlock();

	raw_spin_unlock_irqrestore(&rq->lock, flags);

	preempt_rq(prq);
}

#ifdef CONFIG_PREEMPT_NOTIFIERS

static struct static_key preempt_notifier_key = STATIC_KEY_INIT_FALSE;

void preempt_notifier_inc(void)
{
	static_key_slow_inc(&preempt_notifier_key);
}
EXPORT_SYMBOL_GPL(preempt_notifier_inc);

void preempt_notifier_dec(void)
{
	static_key_slow_dec(&preempt_notifier_key);
}
EXPORT_SYMBOL_GPL(preempt_notifier_dec);

/**
 * preempt_notifier_register - tell me when current is being preempted & rescheduled
 * @notifier: notifier struct to register
 */
void preempt_notifier_register(struct preempt_notifier *notifier)
{
	if (!static_key_false(&preempt_notifier_key))
		WARN(1, "registering preempt_notifier while notifiers disabled\n");

	hlist_add_head(&notifier->link, &current->preempt_notifiers);
}
EXPORT_SYMBOL_GPL(preempt_notifier_register);

/**
 * preempt_notifier_unregister - no longer interested in preemption notifications
 * @notifier: notifier struct to unregister
 *
 * This is *not* safe to call from within a preemption notifier.
 */
void preempt_notifier_unregister(struct preempt_notifier *notifier)
{
	hlist_del(&notifier->link);
}
EXPORT_SYMBOL_GPL(preempt_notifier_unregister);

static void __fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
	struct preempt_notifier *notifier;

	hlist_for_each_entry(notifier, &curr->preempt_notifiers, link)
		notifier->ops->sched_in(notifier, raw_smp_processor_id());
}

static __always_inline void fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
	if (static_key_false(&preempt_notifier_key))
		__fire_sched_in_preempt_notifiers(curr);
}

static void
__fire_sched_out_preempt_notifiers(struct task_struct *curr,
				   struct task_struct *next)
{
	struct preempt_notifier *notifier;

	hlist_for_each_entry(notifier, &curr->preempt_notifiers, link)
		notifier->ops->sched_out(notifier, next);
}

static __always_inline void
fire_sched_out_preempt_notifiers(struct task_struct *curr,
				 struct task_struct *next)
{
	if (static_key_false(&preempt_notifier_key))
		__fire_sched_out_preempt_notifiers(curr, next);
}

#else /* !CONFIG_PREEMPT_NOTIFIERS */

static inline void fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
}

static inline void
fire_sched_out_preempt_notifiers(struct task_struct *curr,
				 struct task_struct *next)
{
}

#endif /* CONFIG_PREEMPT_NOTIFIERS */

/**
 * prepare_task_switch - prepare to switch tasks
 * @rq: the runqueue preparing to switch
 * @next: the task we are going to switch to.
 *
 * This is called with the rq lock held and interrupts off. It must
 * be paired with a subsequent finish_task_switch after the context
 * switch.
 *
 * prepare_task_switch sets up locking and calls architecture specific
 * hooks.
 */
static inline void
prepare_task_switch(struct rq *rq, struct task_struct *prev,
		    struct task_struct *next)
{
	sched_info_switch(rq, prev, next);
	perf_event_task_sched_out(prev, next);
	fire_sched_out_preempt_notifiers(prev, next);
	prepare_lock_switch(rq, next);
	prepare_arch_switch(next);
}

/**
 * finish_task_switch - clean up after a task-switch
 * @rq: runqueue associated with task-switch
 * @prev: the thread we just switched away from.
 *
 * finish_task_switch must be called after the context switch, paired
 * with a prepare_task_switch call before the context switch.
 * finish_task_switch will reconcile locking set up by prepare_task_switch,
 * and do any other architecture-specific cleanup actions.
 *
 * Note that we may have delayed dropping an mm in context_switch(). If
 * so, we finish that here outside of the runqueue lock.  (Doing it
 * with the lock held can cause deadlocks; see schedule() for
 * details.)
 *
 * The context switch have flipped the stack from under us and restored the
 * local variables which were saved when this task called schedule() in the
 * past. prev == current is still correct but we need to recalculate this_rq
 * because prev may have moved to another CPU.
 */
static struct rq *finish_task_switch(struct task_struct *prev)
	__releases(grq.lock)
	__releases(rq->lock)
{
	struct rq *rq = this_rq();
	struct mm_struct *mm = rq->prev_mm;
	long prev_state;

	/*
	 * The previous task will have left us with a preempt_count of 2
	 * because it left us after:
	 *
	 *	schedule()
	 *	  preempt_disable();			// 1
	 *	  __schedule()
	 *	    raw_spin_lock_irq(&rq->lock)	// 2
	 *
	 * Also, see FORK_PREEMPT_COUNT.
	 */
	if (WARN_ONCE(preempt_count() != 2*PREEMPT_DISABLE_OFFSET,
		      "corrupted preempt_count: %s/%d/0x%x\n",
		      current->comm, current->pid, preempt_count()))
		preempt_count_set(FORK_PREEMPT_COUNT);

	rq->prev_mm = NULL;

	/*
	 * A task struct has one reference for the use as "current".
	 * If a task dies, then it sets TASK_DEAD in tsk->state and calls
	 * schedule one last time. The schedule call will never return, and
	 * the scheduled task must drop that reference.
	 *
	 * We must observe prev->state before clearing prev->on_cpu (in
	 * finish_lock_switch), otherwise a concurrent wakeup can get prev
	 * running on another CPU and we could rave with its RUNNING -> DEAD
	 * transition, resulting in a double drop.
	 */
	prev_state = prev->state;
	vtime_task_switch(prev);
	perf_event_task_sched_in(prev, current);

	finish_lock_switch(rq, prev);
	finish_arch_post_lock_switch();

	fire_sched_in_preempt_notifiers(current);
	if (mm)
		mmdrop(mm);
	if (unlikely(prev_state == TASK_DEAD)) {
		/*
		 * Remove function-return probe instances associated with this
		 * task and put them back on the free list.
		 */
		kprobe_flush_task(prev);
		put_task_struct(prev);
	}

	return rq;
}

/**
 * schedule_tail - first thing a freshly forked thread must call.
 * @prev: the thread we just switched away from.
 */
asmlinkage __visible void schedule_tail(struct task_struct *prev)
	__releases(rq->lock)
{
	struct rq *rq;

	/*
	 * New tasks start with FORK_PREEMPT_COUNT, see there and
	 * finish_task_switch() for details.
	 *
	 * finish_task_switch() will drop rq->lock() and lower preempt_count
	 * and the preempt_enable() will end up enabling preemption (on
	 * PREEMPT_COUNT kernels).
	 */

	rq = finish_task_switch(prev);
	preempt_enable();

	if (current->set_child_tid)
		put_user(task_pid_vnr(current), current->set_child_tid);
}

/*
 * context_switch - switch to the new MM and the new thread's register state.
 */
static inline struct rq *
context_switch(struct rq *rq, struct task_struct *prev,
	       struct task_struct *next)
{
	struct mm_struct *mm, *oldmm;
	struct rq *prq;

	prepare_task_switch(rq, prev, next);

	mm = next->mm;
	oldmm = prev->active_mm;
	/*
	 * For paravirt, this is coupled with an exit in switch_to to
	 * combine the page table reload and the switch backend into
	 * one hypercall.
	 */
	arch_start_context_switch(prev);

	if (!mm) {
		next->active_mm = oldmm;
		atomic_inc(&oldmm->mm_count);
		enter_lazy_tlb(oldmm, next);
	} else
		switch_mm(oldmm, mm, next);

	if (!prev->mm) {
		prev->active_mm = NULL;
		rq->prev_mm = oldmm;
	}
	/*
	 * Since the runqueue lock will be released by the next
	 * task (which is an invalid locking op but in the case
	 * of the scheduler it's an obvious special-case), so we
	 * do an early lockdep release here:
	 */
	spin_release(&grq.lock.dep_map, 1, _THIS_IP_);
	lockdep_unpin_lock(&rq->lock);
	spin_release(&rq->lock.dep_map, 1, _THIS_IP_);

	/* Here we just switch the register state and the stack. */
	switch_to(prev, next, prev);
	barrier();

	/*
	 * Before unlock rq, record rq which need to be rescheduled in the stack
	 */
	rq = this_rq();
	if (rq->try_preempt_tsk) {
		prq = (current == rq->try_preempt_tsk)?
			NULL:
			task_best_idle_rq(rq->try_preempt_tsk);
		rq->try_preempt_tsk = NULL;
	} else
		prq = NULL;

	rq = finish_task_switch(prev);

	preempt_rq(prq);

	return rq;
}

/*
 * nr_running, nr_uninterruptible and nr_context_switches:
 *
 * externally visible scheduler statistics: current number of runnable
 * threads, total number of context switches performed since bootup. All are
 * measured without grabbing the grq lock but the occasional inaccurate result
 * doesn't matter so long as it's positive.
 */
unsigned long nr_running(void)
{
	long nr = grq.nr_running;

	if (unlikely(nr < 0))
		nr = 0;
	return (unsigned long)nr;
}

static unsigned long nr_uninterruptible(void)
{
	long nu = grq.nr_uninterruptible;

	if (unlikely(nu < 0))
		nu = 0;
	return nu;
}

/*
 * Check if only the current task is running on the cpu.
 *
 * Caution: this function does not check that the caller has disabled
 * preemption, thus the result might have a time-of-check-to-time-of-use
 * race.  The caller is responsible to use it correctly, for example:
 *
 * - from a non-preemptable section (of course)
 *
 * - from a thread that is bound to a single CPU
 *
 * - in a loop with very short iterations (e.g. a polling loop)
 */
bool single_task_running(void)
{
	return (raw_rq()->rq_running && (0 == queued_notrunning()));
}
EXPORT_SYMBOL(single_task_running);

unsigned long long nr_context_switches(void)
{
	int i;
	unsigned long long sum = 0;

	for_each_possible_cpu(i)
		sum += cpu_rq(i)->nr_switches;

	return sum;
}

unsigned long nr_iowait(void)
{
	unsigned long i, sum = 0;

	for_each_possible_cpu(i)
		sum += atomic_read(&cpu_rq(i)->nr_iowait);

	return sum;
}

unsigned long nr_iowait_cpu(int cpu)
{
	struct rq *this = cpu_rq(cpu);
	return atomic_read(&this->nr_iowait);
}

unsigned long nr_active(void)
{
	return nr_running() + nr_uninterruptible();
}

/* Beyond a task running on this CPU, load is equal everywhere on BFS, so we
 * base it on the number of running or queued tasks with their ->rq pointer
 * set to this cpu as being the CPU they're more likely to run on. */
void get_iowait_load(unsigned long *nr_waiters, unsigned long *load)
{
	struct rq *rq = this_rq();

	*nr_waiters = atomic_read(&rq->nr_iowait);
	/* Beyond a task running on this CPU, load is equal everywhere on BFS */
	*load = rq->rq_running +
		((queued_notrunning() + nr_uninterruptible()) / grq.noc);
}

/* Variables and functions for calc_load */
static unsigned long calc_load_update;
unsigned long avenrun[3];
EXPORT_SYMBOL(avenrun);

/**
 * get_avenrun - get the load average array
 * @loads:	pointer to dest load array
 * @offset:	offset to add
 * @shift:	shift count to shift the result left
 *
 * These values are estimates at best, so no need for locking.
 */
void get_avenrun(unsigned long *loads, unsigned long offset, int shift)
{
	loads[0] = (avenrun[0] + offset) << shift;
	loads[1] = (avenrun[1] + offset) << shift;
	loads[2] = (avenrun[2] + offset) << shift;
}

static unsigned long
calc_load(unsigned long load, unsigned long exp, unsigned long active)
{
	load *= exp;
	load += active * (FIXED_1 - exp);
	return load >> FSHIFT;
}

/*
 * calc_load - update the avenrun load estimates every LOAD_FREQ seconds.
 */
void calc_global_load(unsigned long ticks)
{
	long active;

	if (time_before(jiffies, calc_load_update))
		return;
	active = nr_active() * FIXED_1;

	avenrun[0] = calc_load(avenrun[0], EXP_1, active);
	avenrun[1] = calc_load(avenrun[1], EXP_5, active);
	avenrun[2] = calc_load(avenrun[2], EXP_15, active);

	calc_load_update = jiffies + LOAD_FREQ;
}

DEFINE_PER_CPU(struct kernel_stat, kstat);
DEFINE_PER_CPU(struct kernel_cpustat, kernel_cpustat);

EXPORT_PER_CPU_SYMBOL(kstat);
EXPORT_PER_CPU_SYMBOL(kernel_cpustat);

#ifdef CONFIG_IRQ_TIME_ACCOUNTING

/*
 * There are no locks covering percpu hardirq/softirq time.
 * They are only modified in account_system_vtime, on corresponding CPU
 * with interrupts disabled. So, writes are safe.
 * They are read and saved off onto struct rq in update_rq_clock().
 * This may result in other CPU reading this CPU's irq time and can
 * race with irq/account_system_vtime on this CPU. We would either get old
 * or new value with a side effect of accounting a slice of irq time to wrong
 * task when irq is in progress while we read rq->clock. That is a worthy
 * compromise in place of having locks on each irq in account_system_time.
 */
static DEFINE_PER_CPU(u64, cpu_hardirq_time);
static DEFINE_PER_CPU(u64, cpu_softirq_time);

static DEFINE_PER_CPU(u64, irq_start_time);
static int sched_clock_irqtime;

void enable_sched_clock_irqtime(void)
{
	sched_clock_irqtime = 1;
}

void disable_sched_clock_irqtime(void)
{
	sched_clock_irqtime = 0;
}

#ifndef CONFIG_64BIT
static DEFINE_PER_CPU(seqcount_t, irq_time_seq);

static inline void irq_time_write_begin(void)
{
	__this_cpu_inc(irq_time_seq.sequence);
	smp_wmb();
}

static inline void irq_time_write_end(void)
{
	smp_wmb();
	__this_cpu_inc(irq_time_seq.sequence);
}

static inline u64 irq_time_read(int cpu)
{
	u64 irq_time;
	unsigned seq;

	do {
		seq = read_seqcount_begin(&per_cpu(irq_time_seq, cpu));
		irq_time = per_cpu(cpu_softirq_time, cpu) +
			   per_cpu(cpu_hardirq_time, cpu);
	} while (read_seqcount_retry(&per_cpu(irq_time_seq, cpu), seq));

	return irq_time;
}
#else /* CONFIG_64BIT */
static inline void irq_time_write_begin(void)
{
}

static inline void irq_time_write_end(void)
{
}

static inline u64 irq_time_read(int cpu)
{
	return per_cpu(cpu_softirq_time, cpu) + per_cpu(cpu_hardirq_time, cpu);
}
#endif /* CONFIG_64BIT */

/*
 * Called before incrementing preempt_count on {soft,}irq_enter
 * and before decrementing preempt_count on {soft,}irq_exit.
 */
void irqtime_account_irq(struct task_struct *curr)
{
	unsigned long flags;
	s64 delta;
	int cpu;

	if (!sched_clock_irqtime)
		return;

	local_irq_save(flags);

	cpu = smp_processor_id();
	delta = sched_clock_cpu(cpu) - __this_cpu_read(irq_start_time);
	__this_cpu_add(irq_start_time, delta);

	irq_time_write_begin();
	/*
	 * We do not account for softirq time from ksoftirqd here.
	 * We want to continue accounting softirq time to ksoftirqd thread
	 * in that case, so as not to confuse scheduler with a special task
	 * that do not consume any time, but still wants to run.
	 */
	if (hardirq_count())
		__this_cpu_add(cpu_hardirq_time, delta);
	else if (in_serving_softirq() && curr != this_cpu_ksoftirqd())
		__this_cpu_add(cpu_softirq_time, delta);

	irq_time_write_end();
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(irqtime_account_irq);

#endif /* CONFIG_IRQ_TIME_ACCOUNTING */

#ifdef CONFIG_PARAVIRT
static inline u64 steal_ticks(u64 steal)
{
	if (unlikely(steal > NSEC_PER_SEC))
		return div_u64(steal, TICK_NSEC);

	return __iter_div_u64_rem(steal, TICK_NSEC, &steal);
}
#endif

static void update_rq_clock_task(struct rq *rq, s64 delta)
{
/*
 * In theory, the compile should just see 0 here, and optimize out the call
 * to sched_rt_avg_update. But I don't trust it...
 */
#ifdef CONFIG_IRQ_TIME_ACCOUNTING
	s64 irq_delta = irq_time_read(cpu_of(rq)) - rq->prev_irq_time;

	/*
	 * Since irq_time is only updated on {soft,}irq_exit, we might run into
	 * this case when a previous update_rq_clock() happened inside a
	 * {soft,}irq region.
	 *
	 * When this happens, we stop ->clock_task and only update the
	 * prev_irq_time stamp to account for the part that fit, so that a next
	 * update will consume the rest. This ensures ->clock_task is
	 * monotonic.
	 *
	 * It does however cause some slight miss-attribution of {soft,}irq
	 * time, a more accurate solution would be to update the irq_time using
	 * the current rq->clock timestamp, except that would require using
	 * atomic ops.
	 */
	if (irq_delta > delta)
		irq_delta = delta;

	rq->prev_irq_time += irq_delta;
	delta -= irq_delta;
#endif
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
	if (static_key_false((&paravirt_steal_rq_enabled))) {
		s64 steal = paravirt_steal_clock(cpu_of(rq));

		steal -= rq->prev_steal_time_rq;

		if (unlikely(steal > delta))
			steal = delta;

		rq->prev_steal_time_rq += steal;

		delta -= steal;
	}
#endif

	rq->clock_task += delta;
}

#ifndef nsecs_to_cputime
# define nsecs_to_cputime(__nsecs)	nsecs_to_jiffies(__nsecs)
#endif

#ifdef CONFIG_IRQ_TIME_ACCOUNTING
static void irqtime_account_hi_si(void)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;
	u64 latest_ns;

	latest_ns = nsecs_to_cputime64(this_cpu_read(cpu_hardirq_time));
	if (latest_ns > cpustat[CPUTIME_IRQ])
		cpustat[CPUTIME_IRQ] += (__force u64)cputime_one_jiffy;

	latest_ns = nsecs_to_cputime64(this_cpu_read(cpu_softirq_time));
	if (latest_ns > cpustat[CPUTIME_SOFTIRQ])
		cpustat[CPUTIME_SOFTIRQ] += (__force u64)cputime_one_jiffy;
}
#else /* CONFIG_IRQ_TIME_ACCOUNTING */

#define sched_clock_irqtime	(0)

static inline void irqtime_account_hi_si(void)
{
}
#endif /* CONFIG_IRQ_TIME_ACCOUNTING */

static __always_inline bool steal_account_process_tick(void)
{
#ifdef CONFIG_PARAVIRT
	if (static_key_false(&paravirt_steal_enabled)) {
		u64 steal;
		cputime_t steal_ct;

		steal = paravirt_steal_clock(smp_processor_id());
		steal -= this_rq()->prev_steal_time;

		/*
		 * cputime_t may be less precise than nsecs (eg: if it's
		 * based on jiffies). Lets cast the result to cputime
		 * granularity and account the rest on the next rounds.
		 */
		steal_ct = nsecs_to_cputime(steal);
		this_rq()->prev_steal_time += cputime_to_nsecs(steal_ct);

		account_steal_time(steal_ct);
		return steal_ct;
	}
#endif
	return false;
}

/*
 * Accumulate raw cputime values of dead tasks (sig->[us]time) and live
 * tasks (sum on group iteration) belonging to @tsk's group.
 */
void thread_group_cputime(struct task_struct *tsk, struct task_cputime *times)
{
	struct signal_struct *sig = tsk->signal;
	cputime_t utime, stime;
	struct task_struct *t;
	unsigned int seq, nextseq;
	unsigned long flags;

	rcu_read_lock();
	/* Attempt a lockless read on the first round. */
	nextseq = 0;
	do {
		seq = nextseq;
		flags = read_seqbegin_or_lock_irqsave(&sig->stats_lock, &seq);
		times->utime = sig->utime;
		times->stime = sig->stime;
		times->sum_exec_runtime = sig->sum_sched_runtime;

		for_each_thread(tsk, t) {
			task_cputime(t, &utime, &stime);
			times->utime += utime;
			times->stime += stime;
			times->sum_exec_runtime += task_sched_runtime(t);
		}
		/* If lockless access failed, take the lock. */
		nextseq = 1;
	} while (need_seqretry(&sig->stats_lock, seq));
	done_seqretry_irqrestore(&sig->stats_lock, seq, flags);
	rcu_read_unlock();
}

/*
 * On each tick, see what percentage of that tick was attributed to each
 * component and add the percentage to the _pc values. Once a _pc value has
 * accumulated one tick's worth, account for that. This means the total
 * percentage of load components will always be 128 (pseudo 100) per tick.
 */
static void pc_idle_time(struct rq *rq, struct task_struct *idle, unsigned long pc)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;

	if (atomic_read(&rq->nr_iowait) > 0) {
		rq->iowait_pc += pc;
		if (rq->iowait_pc >= 128) {
			cpustat[CPUTIME_IOWAIT] += (__force u64)cputime_one_jiffy * rq->iowait_pc / 128;
			rq->iowait_pc %= 128;
		}
	} else {
		rq->idle_pc += pc;
		if (rq->idle_pc >= 128) {
			cpustat[CPUTIME_IDLE] += (__force u64)cputime_one_jiffy * rq->idle_pc / 128;
			rq->idle_pc %= 128;
		}
	}
	acct_update_integrals(idle);
}

static void
pc_system_time(struct rq *rq, struct task_struct *p, int hardirq_offset,
	       unsigned long pc, unsigned long ns)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;
	cputime_t one_jiffy_scaled = cputime_to_scaled(cputime_one_jiffy);

	p->stime_pc += pc;
	if (p->stime_pc >= 128) {
		int jiffs = p->stime_pc / 128;

		p->stime_pc %= 128;
		p->stime += (__force u64)cputime_one_jiffy * jiffs;
		p->stimescaled += one_jiffy_scaled * jiffs;
		account_group_system_time(p, cputime_one_jiffy * jiffs);
	}
	p->sched_time += ns;
	account_group_exec_runtime(p, ns);

	if (hardirq_count() - hardirq_offset) {
		rq->irq_pc += pc;
		if (rq->irq_pc >= 128) {
			cpustat[CPUTIME_IRQ] += (__force u64)cputime_one_jiffy * rq->irq_pc / 128;
			rq->irq_pc %= 128;
		}
	} else if (in_serving_softirq()) {
		rq->softirq_pc += pc;
		if (rq->softirq_pc >= 128) {
			cpustat[CPUTIME_SOFTIRQ] += (__force u64)cputime_one_jiffy * rq->softirq_pc / 128;
			rq->softirq_pc %= 128;
		}
	} else {
		rq->system_pc += pc;
		if (rq->system_pc >= 128) {
			cpustat[CPUTIME_SYSTEM] += (__force u64)cputime_one_jiffy * rq->system_pc / 128;
			rq->system_pc %= 128;
		}
	}
	acct_update_integrals(p);
}

static void pc_user_time(struct rq *rq, struct task_struct *p,
			 unsigned long pc, unsigned long ns)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;
	cputime_t one_jiffy_scaled = cputime_to_scaled(cputime_one_jiffy);

	p->utime_pc += pc;
	if (p->utime_pc >= 128) {
		int jiffs = p->utime_pc / 128;

		p->utime_pc %= 128;
		p->utime += (__force u64)cputime_one_jiffy * jiffs;
		p->utimescaled += one_jiffy_scaled * jiffs;
		account_group_user_time(p, cputime_one_jiffy * jiffs);
	}
	p->sched_time += ns;
	account_group_exec_runtime(p, ns);

	if (this_cpu_ksoftirqd() == p) {
		/*
		 * ksoftirqd time do not get accounted in cpu_softirq_time.
		 * So, we have to handle it separately here.
		 */
		rq->softirq_pc += pc;
		if (rq->softirq_pc >= 128) {
			cpustat[CPUTIME_SOFTIRQ] += (__force u64)cputime_one_jiffy * rq->softirq_pc / 128;
			rq->softirq_pc %= 128;
		}
	}

	if (task_nice(p) > 0 || idleprio_task(p)) {
		rq->nice_pc += pc;
		if (rq->nice_pc >= 128) {
			cpustat[CPUTIME_NICE] += (__force u64)cputime_one_jiffy * rq->nice_pc / 128;
			rq->nice_pc %= 128;
		}
	} else {
		rq->user_pc += pc;
		if (rq->user_pc >= 128) {
			cpustat[CPUTIME_USER] += (__force u64)cputime_one_jiffy * rq->user_pc / 128;
			rq->user_pc %= 128;
		}
	}
	acct_update_integrals(p);
}

/*
 * Convert nanoseconds to pseudo percentage of one tick. Use 128 for fast
 * shifts instead of 100
 */
#define NS_TO_PC(NS)	(NS * 128 / JIFFY_NS)

/*
 * This is called on clock ticks.
 * Bank in p->sched_time the ns elapsed since the last tick or switch.
 * CPU scheduler quota accounting is also performed here in microseconds.
 */
static void
update_cpu_clock_tick(struct rq *rq, struct task_struct *p)
{
	long account_ns = rq->clock_task - rq->rq_last_ran;
	struct task_struct *idle = rq->idle;
	unsigned long account_pc;

	if (unlikely(account_ns < 0) || steal_account_process_tick())
		goto ts_account;

	account_pc = NS_TO_PC(account_ns);

	/* Accurate tick timekeeping */
	if (user_mode(get_irq_regs()))
		pc_user_time(rq, p, account_pc, account_ns);
	else if (p != idle || (irq_count() != HARDIRQ_OFFSET))
		pc_system_time(rq, p, HARDIRQ_OFFSET,
			       account_pc, account_ns);
	else
		pc_idle_time(rq, idle, account_pc);

	if (sched_clock_irqtime)
		irqtime_account_hi_si();

ts_account:
	/* time_slice accounting is done in usecs to avoid overflow on 32bit */
	if (rq->rq_policy != SCHED_FIFO && p != idle) {
		s64 time_diff = rq->clock - rq->timekeep_clock;

		rq->rq_time_slice -= NS_TO_US(time_diff);
	}

	rq->rq_last_ran = rq->clock_task;
	rq->timekeep_clock = rq->clock;
}

/*
 * This is called on context switches.
 * Bank in p->sched_time the ns elapsed since the last tick or switch.
 * CPU scheduler quota accounting is also performed here in microseconds.
 */
static inline void
update_cpu_clock_switch_nonidle(struct rq *rq, struct task_struct *p)
{
	long account_ns = rq->clock_task - rq->rq_last_ran;
	unsigned long account_pc;

	if (unlikely(account_ns < 0))
		goto ts_account;

	account_pc = NS_TO_PC(account_ns);

	/* Accurate subtick timekeeping */
	pc_user_time(rq, p, account_pc, account_ns);

ts_account:
	/* time_slice accounting is done in usecs to avoid overflow on 32bit */
	if (rq->rq_policy != SCHED_FIFO) {
		s64 time_diff = rq->clock - rq->timekeep_clock;

		rq->rq_time_slice -= NS_TO_US(time_diff);
	}

	rq->rq_last_ran = rq->clock_task;
	rq->timekeep_clock = rq->clock;
}

static inline void
update_cpu_clock_switch_idle(struct rq *rq, struct task_struct *idle)
{
	long account_ns = rq->clock_task - rq->rq_last_ran;
	unsigned long account_pc;

	if (unlikely(account_ns < 0))
		goto ts_account;

	account_pc = NS_TO_PC(account_ns);
	/* Accurate subtick timekeeping */
	pc_idle_time(rq, idle, account_pc);
ts_account:
	rq->rq_last_ran = rq->clock_task;
	rq->timekeep_clock = rq->clock;
}

/*
 * Return accounted runtime for the task.
 * Return separately the current's pending runtime that have not been
 * accounted yet.
 */
unsigned long long task_sched_runtime(struct task_struct *p)
{
	unsigned long flags;
	struct rq *rq;
	raw_spinlock_t *lock;
	u64 ns;

#if defined(CONFIG_64BIT) && defined(CONFIG_SMP)
	/*
	 * 64-bit doesn't need locks to atomically read a 64bit value.
	 * So we have a optimization chance when the task's delta_exec is 0.
	 * Reading ->on_cpu is racy, but this is ok.
	 *
	 * If we race with it leaving cpu, we'll take a lock. So we're correct.
	 * If we race with it entering cpu, unaccounted time is 0. This is
	 * indistinguishable from the read occurring a few cycles earlier.
	 * If we see ->on_cpu without ->on_rq, the task is leaving, and has
	 * been accounted, so we're correct here as well.
	 */
	if (!p->on_cpu || !p->on_rq)
		return tsk_seruntime(p);
#endif

	rq = task_access_lock_irqsave(p, &lock, &flags);
	/*
	 * Must be ->curr _and_ ->on_rq.  If dequeued, we would
	 * project cycles that may never be accounted to this
	 * thread, breaking clock_gettime().
	 */
	if (p == rq->curr && p->on_rq) {
		update_rq_clock(rq);
		ns = rq->clock_task - rq->rq_last_ran;
		if (unlikely((s64)ns < 0))
			ns = 0;
		p->sched_time += ns;
	}
	ns = tsk_seruntime(p);
	task_access_unlock_irqrestore(lock, &flags);

	return ns;
}

/* Compatibility crap */
void account_user_time(struct task_struct *p, cputime_t cputime,
		       cputime_t cputime_scaled)
{
}

void account_idle_time(cputime_t cputime)
{
}

/*
 * Account guest cpu time to a process.
 * @p: the process that the cpu time gets accounted to
 * @cputime: the cpu time spent in virtual machine since the last update
 * @cputime_scaled: cputime scaled by cpu frequency
 */
static void account_guest_time(struct task_struct *p, cputime_t cputime,
			       cputime_t cputime_scaled)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;

	/* Add guest time to process. */
	p->utime += (__force u64)cputime;
	p->utimescaled += (__force u64)cputime_scaled;
	account_group_user_time(p, cputime);
	p->gtime += (__force u64)cputime;

	/* Add guest time to cpustat. */
	if (task_nice(p) > 0) {
		cpustat[CPUTIME_NICE] += (__force u64)cputime;
		cpustat[CPUTIME_GUEST_NICE] += (__force u64)cputime;
	} else {
		cpustat[CPUTIME_USER] += (__force u64)cputime;
		cpustat[CPUTIME_GUEST] += (__force u64)cputime;
	}
}

/*
 * Account system cpu time to a process and desired cpustat field
 * @p: the process that the cpu time gets accounted to
 * @cputime: the cpu time spent in kernel space since the last update
 * @cputime_scaled: cputime scaled by cpu frequency
 * @target_cputime64: pointer to cpustat field that has to be updated
 */
static inline
void __account_system_time(struct task_struct *p, cputime_t cputime,
			cputime_t cputime_scaled, cputime64_t *target_cputime64)
{
	/* Add system time to process. */
	p->stime += (__force u64)cputime;
	p->stimescaled += (__force u64)cputime_scaled;
	account_group_system_time(p, cputime);

	/* Add system time to cpustat. */
	*target_cputime64 += (__force u64)cputime;

	/* Account for system time used */
	acct_update_integrals(p);
}

/*
 * Account system cpu time to a process.
 * @p: the process that the cpu time gets accounted to
 * @hardirq_offset: the offset to subtract from hardirq_count()
 * @cputime: the cpu time spent in kernel space since the last update
 * @cputime_scaled: cputime scaled by cpu frequency
 * This is for guest only now.
 */
void account_system_time(struct task_struct *p, int hardirq_offset,
			 cputime_t cputime, cputime_t cputime_scaled)
{

	if ((p->flags & PF_VCPU) && (irq_count() - hardirq_offset == 0))
		account_guest_time(p, cputime, cputime_scaled);
}

/*
 * Account for involuntary wait time.
 * @steal: the cpu time spent in involuntary wait
 */
void account_steal_time(cputime_t cputime)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;

	cpustat[CPUTIME_STEAL] += (__force u64)cputime;
}

/*
 * Account for idle time.
 * @cputime: the cpu time spent in idle wait
 */
static void account_idle_times(cputime_t cputime)
{
	u64 *cpustat = kcpustat_this_cpu->cpustat;
	struct rq *rq = this_rq();

	if (atomic_read(&rq->nr_iowait) > 0)
		cpustat[CPUTIME_IOWAIT] += (__force u64)cputime;
	else
		cpustat[CPUTIME_IDLE] += (__force u64)cputime;
}

#ifndef CONFIG_VIRT_CPU_ACCOUNTING_NATIVE

void account_process_tick(struct task_struct *p, int user_tick)
{
}

/*
 * Account multiple ticks of steal time.
 * @p: the process from which the cpu time has been stolen
 * @ticks: number of stolen ticks
 */
void account_steal_ticks(unsigned long ticks)
{
	account_steal_time(jiffies_to_cputime(ticks));
}

/*
 * Account multiple ticks of idle time.
 * @ticks: number of stolen ticks
 */
void account_idle_ticks(unsigned long ticks)
{
	account_idle_times(jiffies_to_cputime(ticks));
}
#endif

static inline void grq_iso_lock(void)
	__acquires(grq.iso_lock)
{
	raw_spin_lock(&grq.iso_lock);
}

static inline void grq_iso_unlock(void)
	__releases(grq.iso_lock)
{
	raw_spin_unlock(&grq.iso_lock);
}

/*
 * Functions to test for when SCHED_ISO tasks have used their allocated
 * quota as real time scheduling and convert them back to SCHED_NORMAL.
 * Where possible, the data is tested lockless, to avoid grabbing iso_lock
 * because the occasional inaccurate result won't matter. However the
 * tick data is only ever modified under lock. iso_refractory is only simply
 * set to 0 or 1 so it's not worth grabbing the lock yet again for that.
 */
static bool set_iso_refractory(void)
{
	grq.iso_refractory = true;
	return grq.iso_refractory;
}

static bool clear_iso_refractory(void)
{
	grq.iso_refractory = false;
	return grq.iso_refractory;
}

/*
 * Test if SCHED_ISO tasks have run longer than their alloted period as RT
 * tasks and set the refractory flag if necessary. There is 10% hysteresis
 * for unsetting the flag. 115/128 is ~90/100 as a fast shift instead of a
 * slow division.
 */
static bool test_ret_isorefractory(struct rq *rq)
{
	if (likely(!grq.iso_refractory)) {
		if (grq.iso_ticks > ISO_PERIOD * sched_iso_cpu)
			return set_iso_refractory();
	} else {
		if (grq.iso_ticks < ISO_PERIOD * (sched_iso_cpu * 115 / 128))
			return clear_iso_refractory();
	}
	return grq.iso_refractory;
}

static void iso_tick(void)
{
	grq_iso_lock();
	grq.iso_ticks += 100;
	grq_iso_unlock();
}

/* No SCHED_ISO task was running so decrease rq->iso_ticks */
static inline void no_iso_tick(void)
{
	if (grq.iso_ticks) {
		grq_iso_lock();
		grq.iso_ticks -= grq.iso_ticks / ISO_PERIOD + 1;
		if (unlikely(grq.iso_refractory && grq.iso_ticks <
		    ISO_PERIOD * (sched_iso_cpu * 115 / 128)))
			clear_iso_refractory();
		grq_iso_unlock();
	}
}

/* This manages tasks that have run out of timeslice during a scheduler_tick */
static void task_running_tick(struct rq *rq)
{
	struct task_struct *p;

	/*
	 * If a SCHED_ISO task is running we increment the iso_ticks. In
	 * order to prevent SCHED_ISO tasks from causing starvation in the
	 * presence of true RT tasks we account those as iso_ticks as well.
	 */
	if ((rt_queue(rq) || (iso_queue(rq) && !grq.iso_refractory))) {
		if (grq.iso_ticks <= (ISO_PERIOD * 128) - 128)
			iso_tick();
	} else
		no_iso_tick();

	if (iso_queue(rq)) {
		if (unlikely(test_ret_isorefractory(rq))) {
			if (rq_running_iso(rq)) {
				/*
				 * SCHED_ISO task is running as RT and limit
				 * has been hit. Force it to reschedule as
				 * SCHED_NORMAL by zeroing its time_slice
				 */
				rq->rq_time_slice = 0;
			}
		}
	}

	/* SCHED_FIFO tasks never run out of timeslice. */
	if (rq->rq_policy == SCHED_FIFO)
		return;
	/*
	 * Tasks that were scheduled in the first half of a tick are not
	 * allowed to run into the 2nd half of the next tick if they will
	 * run out of time slice in the interim. Otherwise, if they have
	 * less than RESCHED_US μs of time slice left they will be rescheduled.
	 */
	if (rq->dither) {
		if (rq->rq_time_slice > HALF_JIFFY_US)
			return;
		else
			rq->rq_time_slice = 0;
	} else if (rq->rq_time_slice >= RESCHED_US)
			return;

	/* p->time_slice < RESCHED_US. We will modify task_struct under 
	 * rq lock as p is rq->curr
	 */
	p = rq->curr;

	requeue_task(p);
	__set_tsk_resched(p);
}

/*
 * This function gets called by the timer code, with HZ frequency.
 * We call it with interrupts disabled. The data modified is all
 * local to struct rq so we don't need to grab grq lock.
 */
void scheduler_tick(void)
{
	int cpu __maybe_unused = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);

	sched_clock_tick();
	/* update rq clock */
	raw_spin_lock(&rq->lock);

	update_rq_clock(rq);
	update_cpu_clock_tick(rq, rq->curr);
	if (!rq_idle(rq))
		task_running_tick(rq);
	else
		no_iso_tick();
	rq->last_tick = rq->clock;
	raw_spin_unlock(&rq->lock);

	perf_event_task_tick();
}

notrace unsigned long get_parent_ip(unsigned long addr)
{
	if (in_lock_functions(addr)) {
		addr = CALLER_ADDR2;
		if (in_lock_functions(addr))
			addr = CALLER_ADDR3;
	}
	return addr;
}

#if defined(CONFIG_PREEMPT) && (defined(CONFIG_DEBUG_PREEMPT) || \
				defined(CONFIG_PREEMPT_TRACER))
void preempt_count_add(int val)
{
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	if (DEBUG_LOCKS_WARN_ON((preempt_count() < 0)))
		return;
#endif
	__preempt_count_add(val);
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Spinlock count overflowing soon?
	 */
	DEBUG_LOCKS_WARN_ON((preempt_count() & PREEMPT_MASK) >=
				PREEMPT_MASK - 10);
#endif
	if (preempt_count() == val) {
		unsigned long ip = get_parent_ip(CALLER_ADDR1);
#ifdef CONFIG_DEBUG_PREEMPT
		current->preempt_disable_ip = ip;
#endif
		trace_preempt_off(CALLER_ADDR0, ip);
	}
}
EXPORT_SYMBOL(preempt_count_add);
NOKPROBE_SYMBOL(preempt_count_add);

void preempt_count_sub(int val)
{
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	if (DEBUG_LOCKS_WARN_ON(val > preempt_count()))
		return;
	/*
	 * Is the spinlock portion underflowing?
	 */
	if (DEBUG_LOCKS_WARN_ON((val < PREEMPT_MASK) &&
			!(preempt_count() & PREEMPT_MASK)))
		return;
#endif

	if (preempt_count() == val)
		trace_preempt_on(CALLER_ADDR0, get_parent_ip(CALLER_ADDR1));
	__preempt_count_sub(val);
}
EXPORT_SYMBOL(preempt_count_sub);
NOKPROBE_SYMBOL(preempt_count_sub);
#endif

/*
 * Deadline is "now" in niffies + (offset by priority). Setting the deadline
 * is the key to everything. It distributes cpu fairly amongst tasks of the
 * same nice value, it proportions cpu according to nice level, it means the
 * task that last woke up the longest ago has the earliest deadline, thus
 * ensuring that interactive tasks get low latency on wake up. The CPU
 * proportion works out to the square of the virtual deadline difference, so
 * this equation will give nice 19 3% CPU compared to nice 0.
 */
static inline u64 prio_deadline_diff(int user_prio)
{
	return (prio_ratios[user_prio] * rr_interval * (MS_TO_NS(1) / 128));
}

static inline u64 task_deadline_diff(struct task_struct *p)
{
	return prio_deadline_diff(TASK_USER_PRIO(p));
}

static inline u64 static_deadline_diff(int static_prio)
{
	return prio_deadline_diff(USER_PRIO(static_prio));
}

static inline int longest_deadline_diff(void)
{
	return prio_deadline_diff(39);
}

static inline int ms_longest_deadline_diff(void)
{
	return NS_TO_MS(longest_deadline_diff());
}

/*
 * The time_slice is only refilled when it is empty and that is when we set a
 * new deadline.
 */
static void time_slice_expired(struct task_struct *p, struct rq *rq)
{
	p->time_slice = timeslice();
	p->deadline = rq->clock + task_deadline_diff(p);
	update_task_priodl(p);
#ifdef CONFIG_SMT_NICE
	if (!p->mm)
		p->smt_bias = 0;
	else if (rt_task(p))
		p->smt_bias = 1 << 30;
	else if (task_running_iso(p))
		p->smt_bias = 1 << 29;
	else if (idleprio_task(p)) {
		if (task_running_idle(p))
			p->smt_bias = 0;
		else
			p->smt_bias = 1;
	} else if (--p->smt_bias < 1)
		p->smt_bias = MAX_PRIO - p->static_prio;
#endif
}

/*
 * Timeslices below RESCHED_US are considered as good as expired as there's no
 * point rescheduling when there's so little time left. SCHED_BATCH tasks
 * have been flagged be not latency sensitive and likely to be fully CPU
 * bound so every time they're rescheduled they have their time_slice
 * refilled, but get a new later deadline to have little effect on
 * SCHED_NORMAL tasks.

 */
static inline void check_deadline(struct task_struct *p, struct rq *rq)
{
	if (p->time_slice < RESCHED_US || batch_task(p))
		time_slice_expired(p, rq);
}

#define BITOP_WORD(nr)		((nr) / BITS_PER_LONG)

/*
 * Scheduler queue bitmap specific find next bit.
 */
static inline unsigned long
next_sched_bit(const unsigned long *addr, unsigned long offset)
{
	const unsigned long *p;
	unsigned long result;
	unsigned long size;
	unsigned long tmp;

	size = PRIO_LIMIT;
	if (offset >= size)
		return size;

	p = addr + BITOP_WORD(offset);
	result = offset & ~(BITS_PER_LONG-1);
	size -= result;
	offset %= BITS_PER_LONG;
	if (offset) {
		tmp = *(p++);
		tmp &= (~0UL << offset);
		if (size < BITS_PER_LONG)
			goto found_first;
		if (tmp)
			goto found_middle;
		size -= BITS_PER_LONG;
		result += BITS_PER_LONG;
	}
	while (size & ~(BITS_PER_LONG-1)) {
		if ((tmp = *(p++)))
			goto found_middle;
		result += BITS_PER_LONG;
		size -= BITS_PER_LONG;
	}
	if (!size)
		return result;
	tmp = *p;

found_first:
	tmp &= (~0UL >> (BITS_PER_LONG - size));
	if (tmp == 0UL)		/* Are any bits set? */
		return result + size;	/* Nope. */
found_middle:
	return result + __ffs(tmp);
}

/*
 * O(n) lookup of all tasks in the global runqueue. The real brainfuck
 * of lock contention and O(n). It's not really O(n) as only the queued,
 * but not running tasks are scanned, and is O(n) queued in the worst case
 * scenario only because the right task can be found before scanning all of
 * them.
 * Tasks are selected in this order:
 * Real time tasks are selected purely by their static priority and in the
 * order they were queued, so the lowest value idx, and the first queued task
 * of that priority value is chosen.
 * If no real time tasks are found, the SCHED_ISO priority is checked, and
 * all SCHED_ISO tasks have the same priority value, so they're selected by
 * the earliest deadline value.
 * If no SCHED_ISO tasks are found, SCHED_NORMAL tasks are selected by the
 * earliest deadline.
 * Finally if no SCHED_NORMAL tasks are found, SCHED_IDLEPRIO tasks are
 * selected by the earliest deadline.
 */
static inline struct
task_struct *earliest_deadline_task(struct rq *rq, int cpu, struct task_struct *idle)
{
	struct task_struct *edt = NULL;
	unsigned long idx = -1;

	do {
		struct list_head *queue;
		struct task_struct *p;
		u64 earliest_deadline;

		idx = next_sched_bit(grq.prio_bitmap, ++idx);
		if (idx >= PRIO_LIMIT)
			return idle;
		queue = grq.queue + idx;

		if (idx < MAX_RT_PRIO) {
			/* We found an rt task */
			list_for_each_entry(p, queue, run_list) {
				/* Make sure cpu affinity is ok */
				if (needs_other_cpu(p, cpu))
					continue;
				edt = p;
				goto out_take;
			}
			/*
			 * None of the RT tasks at this priority can run on
			 * this cpu
			 */
			continue;
		}

		/*
		 * No rt tasks. Find the earliest deadline task. Now we're in
		 * O(n) territory.
		 */
		earliest_deadline = ~0ULL;
		list_for_each_entry(p, queue, run_list) {
			u64 dl;
			int tcpu;

			/* Make sure cpu affinity is ok */
			if (needs_other_cpu(p, cpu))
				continue;

#ifdef CONFIG_SMT_NICE
			if (!smt_should_schedule(p, cpu))
				continue;
#endif
			/*
			 * Soft affinity happens here by not scheduling a cache
			 * task to a different CPU that the task is last ran on,
			 * or by greatly biasing against its deadline based on
			 * cpu cache locality.
			 */
			tcpu = task_cpu(p);

			if (p->cached) {
				if (is_task_should_cached_off(p, task_rq(p)))
					p->cached = 0ULL;
				else if (tcpu != cpu && scaling_rq(rq))
					continue;
			}
			dl = p->deadline << locality_diff(tcpu, rq);

			if (deadline_before(dl, earliest_deadline)) {
				earliest_deadline = dl;
				edt = p;
			}
		}
	} while (!edt);

out_take:
	take_task(cpu, edt);
	return edt;
}


/*
 * Print scheduling while atomic bug:
 */
static noinline void __schedule_bug(struct task_struct *prev)
{
	if (oops_in_progress)
		return;

	printk(KERN_ERR "BUG: scheduling while atomic: %s/%d/0x%08x\n",
		prev->comm, prev->pid, preempt_count());

	debug_show_held_locks(prev);
	print_modules();
	if (irqs_disabled())
		print_irqtrace_events(prev);
#ifdef CONFIG_DEBUG_PREEMPT
	if (in_atomic_preempt_off()) {
		pr_err("Preemption disabled at:");
		print_ip_sym(current->preempt_disable_ip);
		pr_cont("\n");
	}
#endif
	dump_stack();
	add_taint(TAINT_WARN, LOCKDEP_STILL_OK);
}

/*
 * Various schedule()-time debugging checks and statistics:
 */
static inline void schedule_debug(struct task_struct *prev)
{
#ifdef CONFIG_SCHED_STACK_END_CHECK
	BUG_ON(task_stack_end_corrupted(prev));
#endif

	if (unlikely(in_atomic_preempt_off())) {
		__schedule_bug(prev);
		preempt_count_set(PREEMPT_DISABLED);
	}
	rcu_sleep_check();

	profile_hit(SCHED_PROFILING, __builtin_return_address(0));

	schedstat_inc(this_rq(), sched_count);
}

/*
 * The currently running task's information is all stored in rq local data
 * which is only modified by the local CPU, thereby allowing the data to be
 * changed without grabbing the grq lock.
 */
static inline void set_rq_task(struct rq *rq, struct task_struct *p)
{
	rq->rq_time_slice = p->time_slice;
	rq->rq_deadline = p->deadline;
	rq->rq_last_ran = p->last_ran = rq->clock_task;
	rq->rq_policy = p->policy;
	rq->rq_prio = p->prio;

	grq_priodl_lock();
	grq.rq_priodls[cpu_of(rq)] = p->priodl;
	grq_priodl_unlock();

#ifdef CONFIG_SMT_NICE
	rq->rq_mm = p->mm;
	rq->rq_smt_bias = p->smt_bias;
#endif
	rq->rq_running = (p != rq->idle);
}

static inline void reset_rq_task(struct rq *rq, struct task_struct *p)
{
	rq->rq_policy = p->policy;
	rq->rq_prio = p->prio;
	rq->rq_deadline = p->deadline;

	grq_priodl_lock();
	grq.rq_priodls[cpu_of(rq)] = p->priodl;
	grq_priodl_unlock();

#ifdef CONFIG_SMT_NICE
	rq->rq_smt_bias = p->smt_bias;
#endif
}

#ifdef CONFIG_SMT_NICE
/* Iterate over smt siblings when we've scheduled a process on cpu and decide
 * whether they should continue running or be descheduled. */
static void check_smt_siblings(int cpu)
{
	int other_cpu;

	for_each_cpu(other_cpu, thread_cpumask(cpu)) {
		struct task_struct *p;
		struct rq *rq;

		if (other_cpu == cpu)
			continue;
		rq = cpu_rq(other_cpu);
		if (rq_idle(rq))
			continue;
		if (!rq->online)
			continue;
		p = rq->curr;
		if (!smt_should_schedule(p, cpu)) {
			set_tsk_need_resched(p);
			smp_send_reschedule(other_cpu);
		}
	}
}

static void wake_smt_siblings(int cpu)
{
	int other_cpu;

	if (!queued_notrunning())
		return;

	for_each_cpu(other_cpu, thread_cpumask(cpu)) {
		struct rq *rq;

		if (other_cpu == cpu)
			continue;
		rq = cpu_rq(other_cpu);
		if (rq_idle(rq)) {
			struct task_struct *p = rq->curr;

			set_tsk_need_resched(p);
			smp_send_reschedule(other_cpu);
		}
	}
}
#else
static void check_smt_siblings(int __maybe_unused cpu) {}
static void wake_smt_siblings(int __maybe_unused cpu) {}
#endif

/*
 * schedule() is the main scheduler function.
 *
 * The main means of driving the scheduler and thus entering this function are:
 *
 *   1. Explicit blocking: mutex, semaphore, waitqueue, etc.
 *
 *   2. TIF_NEED_RESCHED flag is checked on interrupt and userspace return
 *      paths. For example, see arch/x86/entry_64.S.
 *
 *      To drive preemption between tasks, the scheduler sets the flag in timer
 *      interrupt handler scheduler_tick().
 *
 *   3. Wakeups don't really cause entry into schedule(). They add a
 *      task to the run-queue and that's it.
 *
 *      Now, if the new task added to the run-queue preempts the current
 *      task, then the wakeup sets TIF_NEED_RESCHED and schedule() gets
 *      called on the nearest possible occasion:
 *
 *       - If the kernel is preemptible (CONFIG_PREEMPT=y):
 *
 *         - in syscall or exception context, at the next outmost
 *           preempt_enable(). (this might be as soon as the wake_up()'s
 *           spin_unlock()!)
 *
 *         - in IRQ context, return from interrupt-handler to
 *           preemptible context
 *
 *       - If the kernel is not preemptible (CONFIG_PREEMPT is not set)
 *         then at the next:
 *
 *          - cond_resched() call
 *          - explicit schedule() call
 *          - return from syscall or exception to user-space
 *          - return from interrupt-handler to user-space
 *
 * WARNING: must be called with preemption disabled!
 */
static void __sched notrace __schedule(bool preempt)
{
	struct task_struct *prev, *next, *idle;
	unsigned long *switch_count;
	bool deactivate = false;
	struct rq *rq;
	int cpu;

	cpu = smp_processor_id();
	rq = cpu_rq(cpu);
	prev = rq->curr;

	/*
	 * do_exit() calls schedule() with preemption disabled as an exception;
	 * however we must fix that up, otherwise the next task will see an
	 * inconsistent (higher) preempt count.
	 *
	 * It also avoids the below schedule_debug() test from complaining
	 * about this.
	 */
	if (unlikely(prev->state == TASK_DEAD))
		preempt_enable_no_resched_notrace();

	schedule_debug(prev);

	local_irq_disable();
	rcu_note_context_switch();

	/*
	 * Make sure that signal_pending_state()->signal_pending() below
	 * can't be reordered with __set_current_state(TASK_INTERRUPTIBLE)
	 * done by the caller to avoid the race with signal_wake_up().
	 */
	smp_mb__before_spinlock();
	raw_spin_lock(&rq->lock);
	lockdep_pin_lock(&rq->lock);

	switch_count = &prev->nivcsw;
	if (!preempt && prev->state) {
		if (unlikely(signal_pending_state(prev->state, prev))) {
			prev->state = TASK_RUNNING;
		} else {
			deactivate = true;
			prev->on_rq = 0;

			/*
			 * If a worker is going to sleep, notify and
			 * ask workqueue whether it wants to wake up a
			 * task to maintain concurrency.  If so, wake
			 * up the task.
			 */
			if (prev->flags & PF_WQ_WORKER) {
				struct task_struct *to_wakeup;

				to_wakeup = wq_worker_sleeping(prev, cpu);
				if (to_wakeup) {
					/* This shouldn't happen, but does */
					if (unlikely(to_wakeup == prev))
						deactivate = false;
					else {
						_grq_lock();
						try_to_wake_up_local(to_wakeup);
						_grq_unlock();
						rq->try_preempt_tsk = to_wakeup;
					}
				}
			}
		}
		switch_count = &prev->nvcsw;
	}

	clear_tsk_need_resched(prev);
	clear_preempt_need_resched();

	idle = rq->idle;

	if (idle != prev) {
		update_rq_clock(rq);

		update_cpu_clock_switch_nonidle(rq, prev);
		rq->dither = (rq->clock - rq->last_tick < HALF_JIFFY_NS);
		/* Update all the information stored on struct rq */
		prev->time_slice = rq->rq_time_slice;
		check_deadline(prev, rq);
		prev->last_ran = rq->clock_task;

		_grq_lock();

		if (deactivate)
			deactivate_task(prev, rq);
		else {
			/* Task changed affinity off this CPU */
			if (unlikely(needs_other_cpu(prev, cpu))) {
				enqueue_task(prev, rq);
				inc_qnr();
				rq->try_preempt_tsk = prev;
				goto earliest_deadline_next;
			} else {
				if (queued_notrunning()) {
					enqueue_task(prev, rq);
					inc_qnr();
					next = earliest_deadline_task(rq, cpu, idle);
					if (likely(prev != next)) {
						/*
						 * Don't stick tasks when a real time task is going
						 * to run as they may literally get stuck.
						 */
						if (!rt_task(next))
							cache_task(prev, rq);
						else
							rq->try_preempt_tsk = prev;
						goto do_switch;
					}
					goto unlock_out;
				} else {
					/*
					* We now know prev is the only thing that is
					* awaiting CPU so we can bypass rechecking for
					* the earliest deadline task and just run it
					* again.
					*/
					set_rq_task(rq, prev);
					goto unlock_out;
				}
			}
		}
	} else {
		update_rq_clock(rq);
		update_cpu_clock_switch_idle(rq, prev);
		rq->dither = (rq->clock - rq->last_tick < HALF_JIFFY_NS);
		_grq_lock();
	}

	if (likely(queued_notrunning())) {
earliest_deadline_next:
		next = earliest_deadline_task(rq, cpu, idle);
	} else {
		/*
		 * This CPU is now truly idle as opposed to when idle is
		 * scheduled as a high priority task in its own right.
		 */
		next = idle;
		schedstat_inc(rq, sched_goidle);
	}

	if (likely(prev != next)) {
do_switch:
		if (likely(next->prio != PRIO_LIMIT))
			clear_cpuidle_map(cpu);
		else
			set_cpuidle_map(cpu);

		set_rq_task(rq, next);

		if (next != idle)
			check_smt_siblings(cpu);
		else
			wake_smt_siblings(cpu);

		/* Once next->on_cpu is set, task_access_lock...() can be locked on
		 * task's runqueue, so set it before release grq.lock 
		 */
		next->on_cpu = ON_CPU;
		next->cached = 0ULL;
		rq->curr = next;
		++*switch_count;

		trace_sched_switch(preempt, prev, next);
		rq = context_switch(rq, prev, next); /* unlocks the grq */
		cpu = cpu_of(rq);
		idle = rq->idle;
	} else {
unlock_out:
		check_smt_siblings(cpu);
		_grq_unlock();
		lockdep_unpin_lock(&rq->lock);
		raw_spin_unlock_irq(&rq->lock);
	}
}

static inline void sched_submit_work(struct task_struct *tsk)
{
	if (!tsk->state || tsk_is_pi_blocked(tsk) ||
	    signal_pending_state(tsk->state, tsk))
		return;

	/*
	 * If we are going to sleep and we have plugged IO queued,
	 * make sure to submit it to avoid deadlocks.
	 */
	if (blk_needs_flush_plug(tsk))
		blk_schedule_flush_plug(tsk);
}

asmlinkage __visible void __sched schedule(void)
{
	struct task_struct *tsk = current;

	sched_submit_work(tsk);
	do {
		preempt_disable();
		__schedule(false);
		sched_preempt_enable_no_resched();
	} while (need_resched());
}
EXPORT_SYMBOL(schedule);

#ifdef CONFIG_CONTEXT_TRACKING
asmlinkage __visible void __sched schedule_user(void)
{
	/*
	 * If we come here after a random call to set_need_resched(),
	 * or we have been woken up remotely but the IPI has not yet arrived,
	 * we haven't yet exited the RCU idle mode. Do it here manually until
	 * we find a better solution.
	 *
	 * NB: There are buggy callers of this function.  Ideally we
	 * should warn if prev_state != CONTEXT_USER, but that will trigger
	 * too frequently to make sense yet.
	 */
	enum ctx_state prev_state = exception_enter();
	schedule();
	exception_exit(prev_state);
}
#endif

/**
 * schedule_preempt_disabled - called with preemption disabled
 *
 * Returns with preemption disabled. Note: preempt_count must be 1
 */
void __sched schedule_preempt_disabled(void)
{
	sched_preempt_enable_no_resched();
	schedule();
	preempt_disable();
}

static void __sched notrace preempt_schedule_common(void)
{
	do {
		preempt_disable_notrace();
		__schedule(true);
		preempt_enable_no_resched_notrace();

		/*
		 * Check again in case we missed a preemption opportunity
		 * between schedule and now.
		 */
	} while (need_resched());
}

#ifdef CONFIG_PREEMPT
/*
 * this is the entry point to schedule() from in-kernel preemption
 * off of preempt_enable. Kernel preemptions off return from interrupt
 * occur there and call schedule directly.
 */
asmlinkage __visible void __sched notrace preempt_schedule(void)
{
	/*
	 * If there is a non-zero preempt_count or interrupts are disabled,
	 * we do not want to preempt the current task. Just return..
	 */
	if (likely(!preemptible()))
		return;

	preempt_schedule_common();
}
NOKPROBE_SYMBOL(preempt_schedule);
EXPORT_SYMBOL(preempt_schedule);

/**
 * preempt_schedule_notrace - preempt_schedule called by tracing
 *
 * The tracing infrastructure uses preempt_enable_notrace to prevent
 * recursion and tracing preempt enabling caused by the tracing
 * infrastructure itself. But as tracing can happen in areas coming
 * from userspace or just about to enter userspace, a preempt enable
 * can occur before user_exit() is called. This will cause the scheduler
 * to be called when the system is still in usermode.
 *
 * To prevent this, the preempt_enable_notrace will use this function
 * instead of preempt_schedule() to exit user context if needed before
 * calling the scheduler.
 */
asmlinkage __visible void __sched notrace preempt_schedule_notrace(void)
{
	enum ctx_state prev_ctx;

	if (likely(!preemptible()))
		return;

	do {
		preempt_disable_notrace();
		/*
		 * Needs preempt disabled in case user_exit() is traced
		 * and the tracer calls preempt_enable_notrace() causing
		 * an infinite recursion.
		 */
		prev_ctx = exception_enter();
		__schedule(true);
		exception_exit(prev_ctx);

		preempt_enable_no_resched_notrace();
	} while (need_resched());
}
EXPORT_SYMBOL_GPL(preempt_schedule_notrace);

#endif /* CONFIG_PREEMPT */

/*
 * this is the entry point to schedule() from kernel preemption
 * off of irq context.
 * Note, that this is called and return with irqs disabled. This will
 * protect us against recursive calling from irq.
 */
asmlinkage __visible void __sched preempt_schedule_irq(void)
{
	enum ctx_state prev_state;

	/* Catch callers which need to be fixed */
	BUG_ON(preempt_count() || !irqs_disabled());

	prev_state = exception_enter();

	do {
		preempt_disable();
		local_irq_enable();
		__schedule(true);
		local_irq_disable();
		sched_preempt_enable_no_resched();
	} while (need_resched());

	exception_exit(prev_state);
}

int default_wake_function(wait_queue_t *curr, unsigned mode, int wake_flags,
			  void *key)
{
	return try_to_wake_up(curr->private, mode, wake_flags);
}
EXPORT_SYMBOL(default_wake_function);

static inline struct rq *
check_task_changed(struct rq *rq, struct task_struct *p, int oldprio)
{
	/*
	 * Reschedule if we are currently running on this runqueue and
	 * our priority decreased, or if we are not currently running on
	 * this runqueue and our priority is higher than the current's
	 */
	if (task_running(p)) {
		reset_rq_task(rq, p);
		/* Resched only if we might now be preempted */
		if (p->prio > oldprio)
			resched_curr(rq);
	} else if (task_queued(p)) {
		__dequeue_task(p, oldprio);
		enqueue_task(p, rq);
		return task_preemptable_rq(p);
	}

	return NULL;
}

#ifdef CONFIG_RT_MUTEXES

/*
 * rt_mutex_setprio - set the current priority of a task
 * @p: task
 * @prio: prio value (kernel-internal form)
 *
 * This function changes the 'effective' priority of a task. It does
 * not touch ->normal_prio like __setscheduler().
 *
 * Used by the rt_mutex code to implement priority inheritance
 * logic. Call site only calls if the priority of the task changed.
 */
void rt_mutex_setprio(struct task_struct *p, int prio)
{
	int oldprio;
	struct rq *rq, *prq = NULL;
	raw_spinlock_t *lock;

	BUG_ON(prio < 0 || prio > MAX_PRIO);

	rq = __task_access_lock(p, &lock);

	/*
	 * Idle task boosting is a nono in general. There is one
	 * exception, when PREEMPT_RT and NOHZ is active:
	 *
	 * The idle task calls get_next_timer_interrupt() and holds
	 * the timer wheel base->lock on the CPU and another CPU wants
	 * to access the timer (probably to cancel it). We can safely
	 * ignore the boosting request, as the idle CPU runs this code
	 * with interrupts disabled and will complete the lock
	 * protected section without being interrupted. So there is no
	 * real need to boost.
	 */
	if (unlikely(p == rq->idle)) {
		WARN_ON(p != rq->curr);
		WARN_ON(p->pi_blocked_on);
		goto out_unlock;
	}

	trace_sched_pi_setprio(p, prio);
	oldprio = p->prio;
	p->prio = prio;
	update_task_priodl(p);

	prq = check_task_changed(rq, p, oldprio);

out_unlock:
	__task_access_unlock(lock);

	preempt_rq(prq);
}

#endif

/*
 * Adjust the deadline for when the priority is to change, before it's
 * changed.
 */
static inline void adjust_deadline(struct task_struct *p, int new_prio)
{
	p->deadline += static_deadline_diff(new_prio) - task_deadline_diff(p);
}

void set_user_nice(struct task_struct *p, long nice)
{
	int queued, new_static, old_static;
	unsigned long flags;
	struct rq *rq, *prq = NULL;
	raw_spinlock_t *lock;

	if (task_nice(p) == nice || nice < MIN_NICE || nice > MAX_NICE)
		return;
	new_static = NICE_TO_PRIO(nice);
	/*
	 * We have to be careful, if called from sys_setpriority(),
	 * the task might be in the middle of scheduling on another CPU.
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);
	rq = __task_access_lock(p, &lock);

	/* rq lock may not held!! */
	update_rq_clock(rq);
	/*
	 * The RT priorities are set via sched_setscheduler(), but we still
	 * allow the 'normal' nice value to be set - but as expected
	 * it wont have any effect on scheduling until the task is
	 * not SCHED_NORMAL/SCHED_BATCH:
	 */
	if (has_rt_policy(p)) {
		p->static_prio = new_static;
		goto out_unlock;
	}
	queued = task_queued(p);
	if (queued) {
		dequeue_task(p);
	}

	adjust_deadline(p, new_static);
	old_static = p->static_prio;
	p->static_prio = new_static;
	p->prio = effective_prio(p);
	update_task_priodl(p);

	if (queued) {
		enqueue_task(p, rq);
		if (new_static < old_static)
			prq = task_preemptable_rq(p);
	} else if (task_running(p)) {
		reset_rq_task(rq, p);
		if (old_static < new_static)
			resched_curr(rq);
	}
out_unlock:
	__task_access_unlock(lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

	preempt_rq(prq);
}
EXPORT_SYMBOL(set_user_nice);

/*
 * can_nice - check if a task can reduce its nice value
 * @p: task
 * @nice: nice value
 */
int can_nice(const struct task_struct *p, const int nice)
{
	/* convert nice value [19,-20] to rlimit style value [1,40] */
	int nice_rlim = nice_to_rlimit(nice);

	return (nice_rlim <= task_rlimit(p, RLIMIT_NICE) ||
		capable(CAP_SYS_NICE));
}

#ifdef __ARCH_WANT_SYS_NICE

/*
 * sys_nice - change the priority of the current process.
 * @increment: priority increment
 *
 * sys_setpriority is a more generic, but much slower function that
 * does similar things.
 */
SYSCALL_DEFINE1(nice, int, increment)
{
	long nice, retval;

	/*
	 * Setpriority might change our priority at the same moment.
	 * We don't have to worry. Conceptually one call occurs first
	 * and we have a single winner.
	 */

	increment = clamp(increment, -NICE_WIDTH, NICE_WIDTH);
	nice = task_nice(current) + increment;

	nice = clamp_val(nice, MIN_NICE, MAX_NICE);
	if (increment < 0 && !can_nice(current, nice))
		return -EPERM;

	retval = security_task_setnice(current, nice);
	if (retval)
		return retval;

	set_user_nice(current, nice);
	return 0;
}

#endif

/**
 * task_prio - return the priority value of a given task.
 * @p: the task in question.
 *
 * Return: The priority value as seen by users in /proc.
 * RT tasks are offset by -100. Normal tasks are centered around 1, value goes
 * from 0 (SCHED_ISO) up to 82 (nice +19 SCHED_IDLEPRIO).
 */
int task_prio(const struct task_struct *p)
{
	int delta, prio = p->prio - MAX_RT_PRIO;

	/* rt tasks and iso tasks */
	if (prio <= 0)
		goto out;

	/* Convert to ms to avoid overflows */
	preempt_disable();
	delta = NS_TO_MS(p->deadline - this_rq()->clock);
	preempt_enable();
	delta = delta * 40 / ms_longest_deadline_diff();
	if (delta > 0 && delta <= 80)
		prio += delta;
	if (idleprio_task(p))
		prio += 40;
out:
	return prio;
}

/**
 * idle_cpu - is a given cpu idle currently?
 * @cpu: the processor in question.
 *
 * Return: 1 if the CPU is currently idle. 0 otherwise.
 */
int idle_cpu(int cpu)
{
	return cpu_curr(cpu) == cpu_rq(cpu)->idle;
}

/**
 * idle_task - return the idle task for a given cpu.
 * @cpu: the processor in question.
 *
 * Return: The idle task for the cpu @cpu.
 */
struct task_struct *idle_task(int cpu)
{
	return cpu_rq(cpu)->idle;
}

/**
 * find_process_by_pid - find a process with a matching PID value.
 * @pid: the pid in question.
 *
 * The task of @pid, if found. %NULL otherwise.
 */
static inline struct task_struct *find_process_by_pid(pid_t pid)
{
	return pid ? find_task_by_vpid(pid) : current;
}

#ifdef CONFIG_SMP
/*
 * Change a given task's CPU affinity. Migrate the thread to a
 * proper CPU and schedule it away if the CPU it's executing on
 * is removed from the allowed bitmask.
 *
 * NOTE: the caller must have a valid reference to the task, the
 * task must not exit() & deallocate itself prematurely. The
 * call is not atomic; no spinlocks may be held.
 */
static int __set_cpus_allowed_ptr(struct task_struct *p,
				  const struct cpumask *new_mask, bool check)
{
	bool running_wrong = false;
	bool queued = false;
	unsigned long flags;
	struct rq *rq, *prq = NULL;
	raw_spinlock_t *lock;
	int ret = 0;

	rq = task_access_lock_irqsave(p, &lock, &flags);

	/*
	 * Must re-check here, to close a race against __kthread_bind(),
	 * sched_setaffinity() is not guaranteed to observe the flag.
	 */
	if (check && (p->flags & PF_NO_SETAFFINITY)) {
		ret = -EINVAL;
		goto out;
	}

	if (cpumask_equal(tsk_cpus_allowed(p), new_mask))
		goto out;

	if (!cpumask_intersects(new_mask, cpu_active_mask)) {
		ret = -EINVAL;
		goto out;
	}

	queued = task_queued(p);

	do_set_cpus_allowed(p, new_mask);

	/* Can the task run on the task's current CPU? If so, we're done */
	if (cpumask_test_cpu(task_cpu(p), new_mask))
		goto out;

	if (task_running(p)) {
		/* Task is running on the wrong cpu now, reschedule it. */
		if (rq == this_rq()) {
			set_tsk_need_resched(p);
			running_wrong = true;
		} else
			resched_curr(rq);
	} else
		set_task_cpu(p, cpumask_any_and(cpu_active_mask, new_mask));

out:
	if (queued)
		prq = task_preemptable_rq(p);
	task_access_unlock_irqrestore(lock, &flags);

	preempt_rq(prq);

	if (running_wrong)
		preempt_schedule_common();

	return ret;
}

int set_cpus_allowed_ptr(struct task_struct *p, const struct cpumask *new_mask)
{
	return __set_cpus_allowed_ptr(p, new_mask, false);
}
EXPORT_SYMBOL_GPL(set_cpus_allowed_ptr);

#else
static inline int
__set_cpus_allowed_ptr(struct task_struct *p,
		       const struct cpumask *new_mask, bool check)
{
	return set_cpus_allowed_ptr(p, new_mask);
}
#endif

/*
 * sched_setparam() passes in -1 for its policy, to let the functions
 * it calls know not to change it.
 */
#define SETPARAM_POLICY -1

static void __setscheduler_params(struct task_struct *p,
		const struct sched_attr *attr)
{
	int policy = attr->sched_policy;

	if (policy == SETPARAM_POLICY)
		policy = p->policy;

	p->policy = policy;

	/*
	 * allow normal nice value to be set, but will not have any
	 * effect on scheduling until the task not SCHED_NORMAL/
	 * SCHED_BATCH
	 */
	p->static_prio = NICE_TO_PRIO(attr->sched_nice);

	/*
	 * __sched_setscheduler() ensures attr->sched_priority == 0 when
	 * !rt_policy. Always setting this ensures that things like
	 * getparam()/getattr() don't report silly values for !rt tasks.
	 */
	p->rt_priority = attr->sched_priority;
	p->normal_prio = normal_prio(p);
}

/* Actually do priority change: must hold rq lock. */
static void __setscheduler(struct rq *rq, struct task_struct *p,
			   const struct sched_attr *attr, bool keep_boost)
{
	int oldrtprio = p->rt_priority;
	int oldprio = p->prio;

	__setscheduler_params(p, attr);

	/*
	 * Keep a potential priority boosting if called from
	 * sched_setscheduler().
	 */
	if (keep_boost)
		p->prio = rt_mutex_get_effective_prio(p, p->normal_prio);
	else
		p->prio = p->normal_prio;
	update_task_priodl(p);

	if (task_running(p)) {
		reset_rq_task(rq, p);
		/* Resched only if we might now be preempted */
		if (p->prio > oldprio || p->rt_priority > oldrtprio)
			resched_curr(rq);
	}
}

/*
 * check the target process has a UID that matches the current process's
 */
static bool check_same_owner(struct task_struct *p)
{
	const struct cred *cred = current_cred(), *pcred;
	bool match;

	rcu_read_lock();
	pcred = __task_cred(p);
	match = (uid_eq(cred->euid, pcred->euid) ||
		 uid_eq(cred->euid, pcred->uid));
	rcu_read_unlock();
	return match;
}

static int
__sched_setscheduler(struct task_struct *p,
		     const struct sched_attr *attr, bool user, bool pi)
{
	int newprio = MAX_RT_PRIO - 1 - attr->sched_priority;
	int retval, oldprio, oldpolicy = -1;
	int policy = attr->sched_policy;
	unsigned long flags;
	struct rq *rq, *prq;
	int reset_on_fork;
	raw_spinlock_t *lock;

	/* may grab non-irq protected spin_locks */
	BUG_ON(in_interrupt());
recheck:
	/* double check policy once rq lock held */
	if (policy < 0) {
		reset_on_fork = p->sched_reset_on_fork;
		policy = oldpolicy = p->policy;
	} else {
		reset_on_fork = !!(attr->sched_flags & SCHED_RESET_ON_FORK);

		if (!SCHED_RANGE(policy))
			return -EINVAL;
	}

	if (attr->sched_flags & ~(SCHED_FLAG_RESET_ON_FORK))
		return -EINVAL;

	/*
	 * Valid priorities for SCHED_FIFO and SCHED_RR are
	 * 1..MAX_USER_RT_PRIO-1, valid priority for SCHED_NORMAL and
	 * SCHED_BATCH and SCHED_IDLE is 0.
	 */
	if (attr->sched_priority < 0 ||
	    (p->mm && attr->sched_priority > MAX_USER_RT_PRIO - 1) ||
	    (!p->mm && attr->sched_priority > MAX_RT_PRIO - 1))
		return -EINVAL;
	if (is_rt_policy(policy) != (attr->sched_priority != 0))
		return -EINVAL;

	/*
	 * Allow unprivileged RT tasks to decrease priority:
	 */
	if (user && !capable(CAP_SYS_NICE)) {
		if (is_rt_policy(policy)) {
			unsigned long rlim_rtprio =
					task_rlimit(p, RLIMIT_RTPRIO);

			/* can't set/change the rt policy */
			if (policy != p->policy && !rlim_rtprio)
				return -EPERM;

			/* can't increase priority */
			if (attr->sched_priority > p->rt_priority &&
			    attr->sched_priority > rlim_rtprio)
				return -EPERM;
		} else {
			switch (p->policy) {
				/*
				 * Can only downgrade policies but not back to
				 * SCHED_NORMAL
				 */
				case SCHED_ISO:
					if (policy == SCHED_ISO)
						goto out;
					if (policy == SCHED_NORMAL)
						return -EPERM;
					break;
				case SCHED_BATCH:
					if (policy == SCHED_BATCH)
						goto out;
					if (policy != SCHED_IDLEPRIO)
						return -EPERM;
					break;
				case SCHED_IDLEPRIO:
					if (policy == SCHED_IDLEPRIO)
						goto out;
					return -EPERM;
				default:
					break;
			}
		}

		/* can't change other user's priorities */
		if (!check_same_owner(p))
			return -EPERM;

		/* Normal users shall not reset the sched_reset_on_fork flag */
		if (p->sched_reset_on_fork && !reset_on_fork)
			return -EPERM;
	}

	if (user) {
		retval = security_task_setscheduler(p);
		if (retval)
			return retval;
	}

	/*
	 * make sure no PI-waiters arrive (or leave) while we are
	 * changing the priority of the task:
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);

	/*
	 * To be able to change p->policy safely, task_vrq_lock()
	 * must be called.
	 * IF use task_access_lock() here:
	 * For the task p which is not running, reading rq->stop is
	 * racy but acceptable as ->stop doesn't change much.
	 * An enhancemnet can be made to read rq->stop saftly.
	 */
	rq = __task_access_lock(p, &lock);

	/*
	 * Changing the policy of the stop threads its a very bad idea
	 */
	if (p == rq->stop) {
		__task_access_unlock(lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, flags);
		return -EINVAL;
	}

	/*
	 * If not changing anything there's no need to proceed further:
	 */
	if (unlikely(policy == p->policy && (!is_rt_policy(policy) ||
		attr->sched_priority == p->rt_priority))) {
		p->sched_reset_on_fork = reset_on_fork;
		__task_access_unlock(lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, flags);
		return 0;
	}

	/* recheck policy now with rq lock held */
	if (unlikely(oldpolicy != -1 && oldpolicy != p->policy)) {
		policy = oldpolicy = -1;
		__task_access_unlock(lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, flags);
		goto recheck;
	}

	p->sched_reset_on_fork = reset_on_fork;
	oldprio = p->prio;

	if (pi) {
		/*
		 * Take priority boosted tasks into account. If the new
		 * effective priority is unchanged, we just store the new
		 * normal parameters and do not touch the scheduler class and
		 * the runqueue. This will be done when the task deboost
		 * itself.
		 */
		if (rt_mutex_get_effective_prio(p, newprio) == oldprio) {
			__setscheduler_params(p, attr);
			__task_access_unlock(lock);
			raw_spin_unlock_irqrestore(&p->pi_lock, flags);
			return 0;
		}
	}

	__setscheduler(rq, p, attr, pi);

	prq = check_task_changed(rq, p, oldprio);

	__task_access_unlock(lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

	if (pi)
		rt_mutex_adjust_pi(p);

	preempt_rq(prq);
out:
	return 0;
}

static int _sched_setscheduler(struct task_struct *p, int policy,
			       const struct sched_param *param, bool check)
{
	struct sched_attr attr = {
		.sched_policy   = policy,
		.sched_priority = param->sched_priority,
		.sched_nice     = PRIO_TO_NICE(p->static_prio),
	};
	unsigned long rlim_rtprio = 0;

	/* Fixup the legacy SCHED_RESET_ON_FORK hack. */
	if ((policy != SETPARAM_POLICY) && (policy & SCHED_RESET_ON_FORK)) {
		attr.sched_flags |= SCHED_FLAG_RESET_ON_FORK;
		policy &= ~SCHED_RESET_ON_FORK;
		attr.sched_policy = policy;
	}

	if (is_rt_policy(policy) && !capable(CAP_SYS_NICE)) {
		unsigned long lflags;

		if (!lock_task_sighand(p, &lflags))
			return -ESRCH;
		rlim_rtprio = task_rlimit(p, RLIMIT_RTPRIO);
		unlock_task_sighand(p, &lflags);
		if (!rlim_rtprio) {
			/*
			 * If the caller requested an RT policy without having the
			 * necessary rights, we downgrade the policy to SCHED_ISO.
			 * We also set the attr to zero to pass the checks.
			 */
			attr.sched_policy = SCHED_ISO;
			attr.sched_priority = 0;
			attr.sched_nice = 0;
		}
	}

	return __sched_setscheduler(p, &attr, check, true);
}

/**
 * sched_setscheduler - change the scheduling policy and/or RT priority of a thread.
 * @p: the task in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 *
 * Return: 0 on success. An error code otherwise.
 *
 * NOTE that the task may be already dead.
 */
int sched_setscheduler(struct task_struct *p, int policy,
		       const struct sched_param *param)
{
	return _sched_setscheduler(p, policy, param, true);
}

EXPORT_SYMBOL_GPL(sched_setscheduler);

int sched_setattr(struct task_struct *p, const struct sched_attr *attr)
{
	return __sched_setscheduler(p, attr, true, true);
}
EXPORT_SYMBOL_GPL(sched_setattr);

/**
 * sched_setscheduler_nocheck - change the scheduling policy and/or RT priority of a thread from kernelspace.
 * @p: the task in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 *
 * Just like sched_setscheduler, only don't bother checking if the
 * current context has permission.  For example, this is needed in
 * stop_machine(): we create temporary high priority worker threads,
 * but our caller might not have that capability.
 *
 * Return: 0 on success. An error code otherwise.
 */
int sched_setscheduler_nocheck(struct task_struct *p, int policy,
			       const struct sched_param *param)
{
	return _sched_setscheduler(p, policy, param, false);
}
EXPORT_SYMBOL_GPL(sched_setscheduler_nocheck);

static int
do_sched_setscheduler(pid_t pid, int policy, struct sched_param __user *param)
{
	struct sched_param lparam;
	struct task_struct *p;
	int retval;

	if (!param || pid < 0)
		return -EINVAL;
	if (copy_from_user(&lparam, param, sizeof(struct sched_param)))
		return -EFAULT;

	rcu_read_lock();
	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (p != NULL)
		retval = sched_setscheduler(p, policy, &lparam);
	rcu_read_unlock();

	return retval;
}

/*
 * Mimics kernel/events/core.c perf_copy_attr().
 */
static int sched_copy_attr(struct sched_attr __user *uattr,
			   struct sched_attr *attr)
{
	u32 size;
	int ret;

	if (!access_ok(VERIFY_WRITE, uattr, SCHED_ATTR_SIZE_VER0))
		return -EFAULT;

	/*
	 * zero the full structure, so that a short copy will be nice.
	 */
	memset(attr, 0, sizeof(*attr));

	ret = get_user(size, &uattr->size);
	if (ret)
		return ret;

	if (size > PAGE_SIZE)	/* silly large */
		goto err_size;

	if (!size)		/* abi compat */
		size = SCHED_ATTR_SIZE_VER0;

	if (size < SCHED_ATTR_SIZE_VER0)
		goto err_size;

	/*
	 * If we're handed a bigger struct than we know of,
	 * ensure all the unknown bits are 0 - i.e. new
	 * user-space does not rely on any kernel feature
	 * extensions we dont know about yet.
	 */
	if (size > sizeof(*attr)) {
		unsigned char __user *addr;
		unsigned char __user *end;
		unsigned char val;

		addr = (void __user *)uattr + sizeof(*attr);
		end  = (void __user *)uattr + size;

		for (; addr < end; addr++) {
			ret = get_user(val, addr);
			if (ret)
				return ret;
			if (val)
				goto err_size;
		}
		size = sizeof(*attr);
	}

	ret = copy_from_user(attr, uattr, size);
	if (ret)
		return -EFAULT;

	/*
	 * XXX: do we want to be lenient like existing syscalls; or do we want
	 * to be strict and return an error on out-of-bounds values?
	 */
	attr->sched_nice = clamp(attr->sched_nice, -20, 19);

	/* sched/core.c uses zero here but we already know ret is zero */
	return 0;

err_size:
	put_user(sizeof(*attr), &uattr->size);
	return -E2BIG;
}

/**
 * sys_sched_setscheduler - set/change the scheduler policy and RT priority
 * @pid: the pid in question.
 * @policy: new policy.
 *
 * Return: 0 on success. An error code otherwise.
 * @param: structure containing the new RT priority.
 */
asmlinkage long sys_sched_setscheduler(pid_t pid, int policy,
				       struct sched_param __user *param)
{
	/* negative values for policy are not valid */
	if (policy < 0)
		return -EINVAL;

	return do_sched_setscheduler(pid, policy, param);
}

/**
 * sys_sched_setparam - set/change the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the new RT priority.
 *
 * Return: 0 on success. An error code otherwise.
 */
SYSCALL_DEFINE2(sched_setparam, pid_t, pid, struct sched_param __user *, param)
{
	return do_sched_setscheduler(pid, SETPARAM_POLICY, param);
}

/**
 * sys_sched_setattr - same as above, but with extended sched_attr
 * @pid: the pid in question.
 * @uattr: structure containing the extended parameters.
 */
SYSCALL_DEFINE3(sched_setattr, pid_t, pid, struct sched_attr __user *, uattr,
			       unsigned int, flags)
{
	struct sched_attr attr;
	struct task_struct *p;
	int retval;

	if (!uattr || pid < 0 || flags)
		return -EINVAL;

	retval = sched_copy_attr(uattr, &attr);
	if (retval)
		return retval;

	if ((int)attr.sched_policy < 0)
		return -EINVAL;

	rcu_read_lock();
	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (p != NULL)
		retval = sched_setattr(p, &attr);
	rcu_read_unlock();

	return retval;
}

/**
 * sys_sched_getscheduler - get the policy (scheduling class) of a thread
 * @pid: the pid in question.
 *
 * Return: On success, the policy of the thread. Otherwise, a negative error
 * code.
 */
SYSCALL_DEFINE1(sched_getscheduler, pid_t, pid)
{
	struct task_struct *p;
	int retval = -EINVAL;

	if (pid < 0)
		goto out_nounlock;

	retval = -ESRCH;
	rcu_read_lock();
	p = find_process_by_pid(pid);
	if (p) {
		retval = security_task_getscheduler(p);
		if (!retval)
			retval = p->policy;
	}
	rcu_read_unlock();

out_nounlock:
	return retval;
}

/**
 * sys_sched_getscheduler - get the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the RT priority.
 *
 * Return: On success, 0 and the RT priority is in @param. Otherwise, an error
 * code.
 */
SYSCALL_DEFINE2(sched_getparam, pid_t, pid, struct sched_param __user *, param)
{
	struct sched_param lp = { .sched_priority = 0 };
	struct task_struct *p;
	int retval = -EINVAL;

	if (!param || pid < 0)
		goto out_nounlock;

	rcu_read_lock();
	p = find_process_by_pid(pid);
	retval = -ESRCH;
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	if (has_rt_policy(p))
		lp.sched_priority = p->rt_priority;
	rcu_read_unlock();

	/*
	 * This one might sleep, we cannot do it with a spinlock held ...
	 */
	retval = copy_to_user(param, &lp, sizeof(*param)) ? -EFAULT : 0;

out_nounlock:
	return retval;

out_unlock:
	rcu_read_unlock();
	return retval;
}

static int sched_read_attr(struct sched_attr __user *uattr,
			   struct sched_attr *attr,
			   unsigned int usize)
{
	int ret;

	if (!access_ok(VERIFY_WRITE, uattr, usize))
		return -EFAULT;

	/*
	 * If we're handed a smaller struct than we know of,
	 * ensure all the unknown bits are 0 - i.e. old
	 * user-space does not get uncomplete information.
	 */
	if (usize < sizeof(*attr)) {
		unsigned char *addr;
		unsigned char *end;

		addr = (void *)attr + usize;
		end  = (void *)attr + sizeof(*attr);

		for (; addr < end; addr++) {
			if (*addr)
				return -EFBIG;
		}

		attr->size = usize;
	}

	ret = copy_to_user(uattr, attr, attr->size);
	if (ret)
		return -EFAULT;

	/* sched/core.c uses zero here but we already know ret is zero */
	return ret;
}

/**
 * sys_sched_getattr - similar to sched_getparam, but with sched_attr
 * @pid: the pid in question.
 * @uattr: structure containing the extended parameters.
 * @size: sizeof(attr) for fwd/bwd comp.
 * @flags: for future extension.
 */
SYSCALL_DEFINE4(sched_getattr, pid_t, pid, struct sched_attr __user *, uattr,
		unsigned int, size, unsigned int, flags)
{
	struct sched_attr attr = {
		.size = sizeof(struct sched_attr),
	};
	struct task_struct *p;
	int retval;

	if (!uattr || pid < 0 || size > PAGE_SIZE ||
	    size < SCHED_ATTR_SIZE_VER0 || flags)
		return -EINVAL;

	rcu_read_lock();
	p = find_process_by_pid(pid);
	retval = -ESRCH;
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	attr.sched_policy = p->policy;
	if (rt_task(p))
		attr.sched_priority = p->rt_priority;
	else
		attr.sched_nice = task_nice(p);

	rcu_read_unlock();

	retval = sched_read_attr(uattr, &attr, size);
	return retval;

out_unlock:
	rcu_read_unlock();
	return retval;
}

long sched_setaffinity(pid_t pid, const struct cpumask *in_mask)
{
	cpumask_var_t cpus_allowed, new_mask;
	struct task_struct *p;
	int retval;

	get_online_cpus();
	rcu_read_lock();

	p = find_process_by_pid(pid);
	if (!p) {
		rcu_read_unlock();
		put_online_cpus();
		return -ESRCH;
	}

	/* Prevent p going away */
	get_task_struct(p);
	rcu_read_unlock();

	if (p->flags & PF_NO_SETAFFINITY) {
		retval = -EINVAL;
		goto out_put_task;
	}
	if (!alloc_cpumask_var(&cpus_allowed, GFP_KERNEL)) {
		retval = -ENOMEM;
		goto out_put_task;
	}
	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL)) {
		retval = -ENOMEM;
		goto out_free_cpus_allowed;
	}
	retval = -EPERM;
	if (!check_same_owner(p)) {
		rcu_read_lock();
		if (!ns_capable(__task_cred(p)->user_ns, CAP_SYS_NICE)) {
			rcu_read_unlock();
			goto out_unlock;
		}
		rcu_read_unlock();
	}

	retval = security_task_setscheduler(p);
	if (retval)
		goto out_unlock;

	cpuset_cpus_allowed(p, cpus_allowed);
	cpumask_and(new_mask, in_mask, cpus_allowed);
again:
	retval = __set_cpus_allowed_ptr(p, new_mask, true);

	if (!retval) {
		cpuset_cpus_allowed(p, cpus_allowed);
		if (!cpumask_subset(new_mask, cpus_allowed)) {
			/*
			 * We must have raced with a concurrent cpuset
			 * update. Just reset the cpus_allowed to the
			 * cpuset's cpus_allowed
			 */
			cpumask_copy(new_mask, cpus_allowed);
			goto again;
		}
	}
out_unlock:
	free_cpumask_var(new_mask);
out_free_cpus_allowed:
	free_cpumask_var(cpus_allowed);
out_put_task:
	put_task_struct(p);
	put_online_cpus();
	return retval;
}

static int get_user_cpu_mask(unsigned long __user *user_mask_ptr, unsigned len,
			     cpumask_t *new_mask)
{
	if (len < sizeof(cpumask_t)) {
		memset(new_mask, 0, sizeof(cpumask_t));
	} else if (len > sizeof(cpumask_t)) {
		len = sizeof(cpumask_t);
	}
	return copy_from_user(new_mask, user_mask_ptr, len) ? -EFAULT : 0;
}


/**
 * sys_sched_setaffinity - set the cpu affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to the new cpu mask
 *
 * Return: 0 on success. An error code otherwise.
 */
SYSCALL_DEFINE3(sched_setaffinity, pid_t, pid, unsigned int, len,
		unsigned long __user *, user_mask_ptr)
{
	cpumask_var_t new_mask;
	int retval;

	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL))
		return -ENOMEM;

	retval = get_user_cpu_mask(user_mask_ptr, len, new_mask);
	if (retval == 0)
		retval = sched_setaffinity(pid, new_mask);
	free_cpumask_var(new_mask);
	return retval;
}

long sched_getaffinity(pid_t pid, cpumask_t *mask)
{
	struct task_struct *p;
	raw_spinlock_t *lock;
	unsigned long flags;
	int retval;

	rcu_read_lock();

	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	task_access_lock_irqsave(p, &lock, &flags);
	cpumask_and(mask, tsk_cpus_allowed(p), cpu_active_mask);
	task_access_unlock_irqrestore(lock, &flags);

out_unlock:
	rcu_read_unlock();

	return retval;
}

/**
 * sys_sched_getaffinity - get the cpu affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to hold the current cpu mask
 *
 * Return: 0 on success. An error code otherwise.
 */
SYSCALL_DEFINE3(sched_getaffinity, pid_t, pid, unsigned int, len,
		unsigned long __user *, user_mask_ptr)
{
	int ret;
	cpumask_var_t mask;

	if ((len * BITS_PER_BYTE) < nr_cpu_ids)
		return -EINVAL;
	if (len & (sizeof(unsigned long)-1))
		return -EINVAL;

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	ret = sched_getaffinity(pid, mask);
	if (ret == 0) {
		size_t retlen = min_t(size_t, len, cpumask_size());

		if (copy_to_user(user_mask_ptr, mask, retlen))
			ret = -EFAULT;
		else
			ret = retlen;
	}
	free_cpumask_var(mask);

	return ret;
}

/*
 * this_rq_lock - lock this runqueue and disable interrupts.
 */
static struct rq *this_rq_lock(void)
	__acquires(rq->lock)
{
	struct rq *rq;

	local_irq_disable();
	rq = this_rq();
	raw_spin_lock(&rq->lock);

	return rq;
}

/**
 * sys_sched_yield - yield the current processor to other threads.
 *
 * This function yields the current CPU to other tasks. It does this by
 * scheduling away the current task. If it still has the earliest deadline
 * it will be scheduled again as the next task.
 *
 * Return: 0.
 */
SYSCALL_DEFINE0(sched_yield)
{
	struct rq *rq = this_rq_lock();

	schedstat_inc(rq, yld_count);
	requeue_task(rq->curr);

	/*
	 * Since we are going to call schedule() anyway, there's
	 * no need to preempt or enable interrupts:
	 */
	__release(&rq->lock);
	spin_release(&rq->lock.dep_map, 1, _THIS_IP_);
	do_raw_spin_unlock(&rq->lock);
	sched_preempt_enable_no_resched();

	schedule();

	return 0;
}

int __sched _cond_resched(void)
{
	if (should_resched(0)) {
		preempt_schedule_common();
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(_cond_resched);

/*
 * __cond_resched_lock() - if a reschedule is pending, drop the given lock,
 * call schedule, and on return reacquire the lock.
 *
 * This works OK both with and without CONFIG_PREEMPT.  We do strange low-level
 * operations here to prevent schedule() from being called twice (once via
 * spin_unlock(), once by hand).
 */
int __cond_resched_lock(spinlock_t *lock)
{
	int resched = should_resched(PREEMPT_LOCK_OFFSET);
	int ret = 0;

	lockdep_assert_held(lock);

	if (spin_needbreak(lock) || resched) {
		spin_unlock(lock);
		if (resched)
			preempt_schedule_common();
		else
			cpu_relax();
		ret = 1;
		spin_lock(lock);
	}
	return ret;
}
EXPORT_SYMBOL(__cond_resched_lock);

int __sched __cond_resched_softirq(void)
{
	BUG_ON(!in_softirq());

	if (should_resched(SOFTIRQ_DISABLE_OFFSET)) {
		local_bh_enable();
		preempt_schedule_common();
		local_bh_disable();
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(__cond_resched_softirq);

/**
 * yield - yield the current processor to other threads.
 *
 * Do not ever use this function, there's a 99% chance you're doing it wrong.
 *
 * The scheduler is at all times free to pick the calling task as the most
 * eligible task to run, if removing the yield() call from your code breaks
 * it, its already broken.
 *
 * Typical broken usage is:
 *
 * while (!event)
 * 	yield();
 *
 * where one assumes that yield() will let 'the other' process run that will
 * make event true. If the current task is a SCHED_FIFO task that will never
 * happen. Never use yield() as a progress guarantee!!
 *
 * If you want to use yield() to wait for something, use wait_event().
 * If you want to use yield() to be 'nice' for others, use cond_resched().
 * If you still want to use yield(), do not!
 */
void __sched yield(void)
{
	set_current_state(TASK_RUNNING);
	sys_sched_yield();
}
EXPORT_SYMBOL(yield);

/**
 * yield_to - yield the current processor to another thread in
 * your thread group, or accelerate that thread toward the
 * processor it's on.
 * @p: target task
 * @preempt: whether task preemption is allowed or not
 *
 * It's the caller's job to ensure that the target task struct
 * can't go away on us before we can do any checks.
 *
 * Return:
 *	true (>0) if we indeed boosted the target task.
 *	false (0) if we failed to boost the target.
 *	-ESRCH if there's no task to yield to.
 */
int __sched yield_to(struct task_struct *p, bool preempt)
{
	struct rq *rq, *p_rq;
	unsigned long flags;
	int yielded = 0;

	rq = this_rq();
	raw_spin_lock_irqsave(&rq->lock, flags);
	grq_lock();
	if (task_running(p) || p->state) {
		yielded = -ESRCH;
		goto out_unlock;
	}

	p_rq = task_rq(p);
	yielded = 1;
	if (p->deadline > rq->rq_deadline) {
		p->deadline = rq->rq_deadline;
		update_task_priodl(p);
	}
	p->time_slice += rq->rq_time_slice;
	rq->rq_time_slice = 0;
	if (p->time_slice > timeslice())
		p->time_slice = timeslice();
	if (preempt && rq != p_rq)
		resched_curr(p_rq);
out_unlock:
	grq_unlock();
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	if (yielded > 0)
		schedule();
	return yielded;
}
EXPORT_SYMBOL_GPL(yield_to);

/*
 * This task is about to go to sleep on IO.  Increment rq->nr_iowait so
 * that process accounting knows that this is a task in IO wait state.
 *
 * But don't do that if it is a deliberate, throttling IO wait (this task
 * has set its backing_dev_info: the queue against which it should throttle)
 */

long __sched io_schedule_timeout(long timeout)
{
	int old_iowait = current->in_iowait;
	struct rq *rq;
	long ret;

	current->in_iowait = 1;
	blk_schedule_flush_plug(current);

	delayacct_blkio_start();
	rq = raw_rq();
	atomic_inc(&rq->nr_iowait);
	ret = schedule_timeout(timeout);
	current->in_iowait = old_iowait;
	atomic_dec(&rq->nr_iowait);
	delayacct_blkio_end();

	return ret;
}
EXPORT_SYMBOL(io_schedule_timeout);

/**
 * sys_sched_get_priority_max - return maximum RT priority.
 * @policy: scheduling class.
 *
 * Return: On success, this syscall returns the maximum
 * rt_priority that can be used by a given scheduling class.
 * On failure, a negative error code is returned.
 */
SYSCALL_DEFINE1(sched_get_priority_max, int, policy)
{
	int ret = -EINVAL;

	switch (policy) {
	case SCHED_FIFO:
	case SCHED_RR:
		ret = MAX_USER_RT_PRIO-1;
		break;
	case SCHED_NORMAL:
	case SCHED_BATCH:
	case SCHED_ISO:
	case SCHED_IDLEPRIO:
		ret = 0;
		break;
	}
	return ret;
}

/**
 * sys_sched_get_priority_min - return minimum RT priority.
 * @policy: scheduling class.
 *
 * Return: On success, this syscall returns the minimum
 * rt_priority that can be used by a given scheduling class.
 * On failure, a negative error code is returned.
 */
SYSCALL_DEFINE1(sched_get_priority_min, int, policy)
{
	int ret = -EINVAL;

	switch (policy) {
	case SCHED_FIFO:
	case SCHED_RR:
		ret = 1;
		break;
	case SCHED_NORMAL:
	case SCHED_BATCH:
	case SCHED_ISO:
	case SCHED_IDLEPRIO:
		ret = 0;
		break;
	}
	return ret;
}

/**
 * sys_sched_rr_get_interval - return the default timeslice of a process.
 * @pid: pid of the process.
 * @interval: userspace pointer to the timeslice value.
 *
 *
 * Return: On success, 0 and the timeslice is in @interval. Otherwise,
 * an error code.
 */
SYSCALL_DEFINE2(sched_rr_get_interval, pid_t, pid,
		struct timespec __user *, interval)
{
	struct task_struct *p;
	unsigned int time_slice;
	unsigned long flags;
	int retval;
	struct timespec t;
	raw_spinlock_t *lock;

	if (pid < 0)
		return -EINVAL;

	retval = -ESRCH;
	rcu_read_lock();
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	task_access_lock_irqsave(p, &lock, &flags);
	time_slice = p->policy == SCHED_FIFO ? 0 : MS_TO_NS(task_timeslice(p));
	task_access_unlock_irqrestore(lock, &flags);

	rcu_read_unlock();
	t = ns_to_timespec(time_slice);
	retval = copy_to_user(interval, &t, sizeof(t)) ? -EFAULT : 0;
	return retval;

out_unlock:
	rcu_read_unlock();
	return retval;
}

static const char stat_nam[] = TASK_STATE_TO_CHAR_STR;

void sched_show_task(struct task_struct *p)
{
	unsigned long free = 0;
	int ppid;
	unsigned long state = p->state;

	if (state)
		state = __ffs(state) + 1;
	printk(KERN_INFO "%-15.15s %c", p->comm,
		state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
#if BITS_PER_LONG == 32
	if (state == TASK_RUNNING)
		printk(KERN_CONT " running  ");
	else
		printk(KERN_CONT " %08lx ", thread_saved_pc(p));
#else
	if (state == TASK_RUNNING)
		printk(KERN_CONT "  running task    ");
	else
		printk(KERN_CONT " %016lx ", thread_saved_pc(p));
#endif
#ifdef CONFIG_DEBUG_STACK_USAGE
	free = stack_not_used(p);
#endif
	ppid = 0;
	rcu_read_lock();
	if (pid_alive(p))
		ppid = task_pid_nr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	printk(KERN_CONT "%5lu %5d %6d 0x%08lx\n", free,
		task_pid_nr(p), ppid,
		(unsigned long)task_thread_info(p)->flags);

	print_worker_info(KERN_INFO, p);
	show_stack(p, NULL);
}

void show_state_filter(unsigned long state_filter)
{
	struct task_struct *g, *p;

#if BITS_PER_LONG == 32
	printk(KERN_INFO
		"  task                PC stack   pid father\n");
#else
	printk(KERN_INFO
		"  task                        PC stack   pid father\n");
#endif
	rcu_read_lock();
	for_each_process_thread(g, p) {
		/*
		 * reset the NMI-timeout, listing all files on a slow
		 * console might take a lot of time:
		 */
		touch_nmi_watchdog();
		if (!state_filter || (p->state & state_filter))
			sched_show_task(p);
	}

	touch_all_softlockup_watchdogs();

	rcu_read_unlock();
	/*
	 * Only show locks if all tasks are dumped:
	 */
	if (!state_filter)
		debug_show_all_locks();
}

void dump_cpu_task(int cpu)
{
	pr_info("Task dump for CPU %d:\n", cpu);
	sched_show_task(cpu_curr(cpu));
}

/**
 * init_idle - set up an idle thread for a given CPU
 * @idle: task in question
 * @cpu: cpu the idle task belongs to
 *
 * NOTE: this function does not set the idle thread's NEED_RESCHED
 * flag, to make booting more robust.
 */
void init_idle(struct task_struct *idle, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	raw_spin_lock_irqsave(&idle->pi_lock, flags);
	raw_spin_lock(&rq->lock);
	_grq_lock();
	update_rq_clock(rq);

	idle->last_ran = rq->clock_task;
	idle->state = TASK_RUNNING;
	/* Setting prio to illegal value shouldn't matter when never queued */
	idle->prio = PRIO_LIMIT;
	idle->deadline = 0ULL;
	update_task_priodl(idle);
	idle->cached = 0ULL;

	kasan_unpoison_task_stack(idle);

#ifdef CONFIG_SMP
	/*
	 * It's possible that init_idle() gets called multiple times on a task,
	 * in that case do_set_cpus_allowed() will not do the right thing.
	 *
	 * And since this is boot we can forgo the serialisation.
	 */
	set_cpus_allowed_common(idle, cpumask_of(cpu));
#ifdef CONFIG_SMT_NICE
	idle->smt_bias = 0;
#endif
#endif
	set_rq_task(rq, idle);

	/* Silence PROVE_RCU */
	rcu_read_lock();
	set_task_cpu(idle, cpu);
	rcu_read_unlock();

	rq->curr = rq->idle = idle;
	idle->on_cpu = ON_CPU;

	_grq_unlock();
	raw_spin_unlock(&rq->lock);
	raw_spin_unlock_irqrestore(&idle->pi_lock, flags);

	/* Set the preempt count _outside_ the spinlocks! */
	init_idle_preempt_count(idle, cpu);

	ftrace_graph_init_idle_task(idle, cpu);
#ifdef CONFIG_SMP
	sprintf(idle->comm, "%s/%d", INIT_TASK_COMM, cpu);
#endif
}

void resched_cpu(int cpu)
{
	unsigned long flags;
	struct rq *rq;

	rq = cpu_rq(cpu);
	raw_spin_lock_irqsave(&rq->lock, flags);
	resched_curr(cpu_rq(cpu));
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

int cpuset_cpumask_can_shrink(const struct cpumask __maybe_unused *cur,
			      const struct cpumask __maybe_unused *trial)
{
	return 1;
}

int task_can_attach(struct task_struct *p,
		    const struct cpumask *cs_cpus_allowed)
{
	int ret = 0;

	/*
	 * Kthreads which disallow setaffinity shouldn't be moved
	 * to a new cpuset; we don't want to change their cpu
	 * affinity and isolating such threads by their set of
	 * allowed nodes is unnecessary.  Thus, cpusets are not
	 * applicable for such threads.  This prevents checking for
	 * success of set_cpus_allowed_ptr() on all attached tasks
	 * before cpus_allowed may be changed.
	 */
	if (p->flags & PF_NO_SETAFFINITY)
		ret = -EINVAL;

	return ret;
}

void wake_q_add(struct wake_q_head *head, struct task_struct *task)
{
	struct wake_q_node *node = &task->wake_q;

	/*
	 * Atomically grab the task, if ->wake_q is !nil already it means
	 * its already queued (either by us or someone else) and will get the
	 * wakeup due to that.
	 *
	 * This cmpxchg() implies a full barrier, which pairs with the write
	 * barrier implied by the wakeup in wake_up_list().
	 */
	if (cmpxchg(&node->next, NULL, WAKE_Q_TAIL))
		return;

	get_task_struct(task);

	/*
	 * The head is context local, there can be no concurrency.
	 */
	*head->lastp = node;
	head->lastp = &node->next;
}

void wake_up_q(struct wake_q_head *head)
{
	struct wake_q_node *node = head->first;

	while (node != WAKE_Q_TAIL) {
		struct task_struct *task;

		task = container_of(node, struct task_struct, wake_q);
		BUG_ON(!task);
		/* task can safely be re-inserted now */
		node = node->next;
		task->wake_q.next = NULL;

		/*
		 * wake_up_process() implies a wmb() to pair with the queueing
		 * in wake_q_add() so as not to miss wakeups.
		 */
		wake_up_process(task);
		put_task_struct(task);
	}
}

#ifdef CONFIG_SMP
#ifdef CONFIG_NO_HZ_COMMON
void nohz_balance_enter_idle(int cpu)
{
}

void select_nohz_load_balancer(int stop_tick)
{
}

void set_cpu_sd_state_idle(void) {}
#if defined(CONFIG_SCHED_MC) || defined(CONFIG_SCHED_SMT)
/**
 * lowest_flag_domain - Return lowest sched_domain containing flag.
 * @cpu:	The cpu whose lowest level of sched domain is to
 *		be returned.
 * @flag:	The flag to check for the lowest sched_domain
 *		for the given cpu.
 *
 * Returns the lowest sched_domain of a cpu which contains the given flag.
 */
static inline struct sched_domain *lowest_flag_domain(int cpu, int flag)
{
	struct sched_domain *sd;

	for_each_domain(cpu, sd)
		if (sd && (sd->flags & flag))
			break;

	return sd;
}

/**
 * for_each_flag_domain - Iterates over sched_domains containing the flag.
 * @cpu:	The cpu whose domains we're iterating over.
 * @sd:		variable holding the value of the power_savings_sd
 *		for cpu.
 * @flag:	The flag to filter the sched_domains to be iterated.
 *
 * Iterates over all the scheduler domains for a given cpu that has the 'flag'
 * set, starting from the lowest sched_domain to the highest.
 */
#define for_each_flag_domain(cpu, sd, flag) \
	for (sd = lowest_flag_domain(cpu, flag); \
		(sd && (sd->flags & flag)); sd = sd->parent)

#endif /*  (CONFIG_SCHED_MC || CONFIG_SCHED_SMT) */

/*
 * In the semi idle case, use the nearest busy cpu for migrating timers
 * from an idle cpu.  This is good for power-savings.
 *
 * We don't do similar optimization for completely idle system, as
 * selecting an idle cpu will add more delays to the timers than intended
 * (as that cpu's timer base may not be uptodate wrt jiffies etc).
 */
int get_nohz_timer_target(void)
{
	int i, cpu = smp_processor_id();
	struct sched_domain *sd;

	if (!idle_cpu(cpu) && is_housekeeping_cpu(cpu))
		return cpu;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		for_each_cpu(i, sched_domain_span(sd)) {
			if (!idle_cpu(i) && is_housekeeping_cpu(cpu)) {
				cpu = i;
				goto unlock;
			}
		}
	}

	if (!is_housekeeping_cpu(cpu))
		cpu = housekeeping_any_cpu();
unlock:
	rcu_read_unlock();
	return cpu;
}

/*
 * When add_timer_on() enqueues a timer into the timer wheel of an
 * idle CPU then this timer might expire before the next timer event
 * which is scheduled to wake up that CPU. In case of a completely
 * idle system the next event might even be infinite time into the
 * future. wake_up_idle_cpu() ensures that the CPU is woken up and
 * leaves the inner idle loop so the newly added timer is taken into
 * account when the CPU goes back to idle and evaluates the timer
 * wheel for the next timer event.
 */
void wake_up_idle_cpu(int cpu)
{
	if (cpu == smp_processor_id())
		return;

	set_tsk_need_resched(cpu_rq(cpu)->idle);
	smp_send_reschedule(cpu);
}

void wake_up_nohz_cpu(int cpu)
{
	wake_up_idle_cpu(cpu);
}
#endif /* CONFIG_NO_HZ_COMMON */

#ifdef CONFIG_HOTPLUG_CPU
/*
 * Ensures that the idle task is using init_mm right before its cpu goes
 * offline.
 */
void idle_task_exit(void)
{
	struct mm_struct *mm = current->active_mm;

	BUG_ON(cpu_online(smp_processor_id()));

	if (mm != &init_mm) {
		switch_mm(mm, &init_mm, current);
		finish_arch_post_lock_switch();
	}
	mmdrop(mm);
}
#endif /* CONFIG_HOTPLUG_CPU */

void sched_set_stop_task(int cpu, struct task_struct *stop)
{
	struct sched_param stop_param = { .sched_priority = STOP_PRIO };
	struct sched_param start_param = { .sched_priority = 0 };
	struct task_struct *old_stop = cpu_rq(cpu)->stop;

	if (stop) {
		/*
		 * Make it appear like a SCHED_FIFO task, its something
		 * userspace knows about and won't get confused about.
		 *
		 * Also, it will make PI more or less work without too
		 * much confusion -- but then, stop work should not
		 * rely on PI working anyway.
		 */
		sched_setscheduler_nocheck(stop, SCHED_FIFO, &stop_param);
	}

	cpu_rq(cpu)->stop = stop;

	if (old_stop) {
		/*
		 * Reset it back to a normal scheduling policy so that
		 * it can die in pieces.
		 */
		sched_setscheduler_nocheck(old_stop, SCHED_NORMAL, &start_param);
	}
}


#if defined(CONFIG_SCHED_DEBUG) && defined(CONFIG_SYSCTL)

static struct ctl_table sd_ctl_dir[] = {
	{
		.procname	= "sched_domain",
		.mode		= 0555,
	},
	{}
};

static struct ctl_table sd_ctl_root[] = {
	{
		.procname	= "kernel",
		.mode		= 0555,
		.child		= sd_ctl_dir,
	},
	{}
};

static struct ctl_table *sd_alloc_ctl_entry(int n)
{
	struct ctl_table *entry =
		kcalloc(n, sizeof(struct ctl_table), GFP_KERNEL);

	return entry;
}

static void sd_free_ctl_entry(struct ctl_table **tablep)
{
	struct ctl_table *entry;

	/*
	 * In the intermediate directories, both the child directory and
	 * procname are dynamically allocated and could fail but the mode
	 * will always be set. In the lowest directory the names are
	 * static strings and all have proc handlers.
	 */
	for (entry = *tablep; entry->mode; entry++) {
		if (entry->child)
			sd_free_ctl_entry(&entry->child);
		if (entry->proc_handler == NULL)
			kfree(entry->procname);
	}

	kfree(*tablep);
	*tablep = NULL;
}

static void
set_table_entry(struct ctl_table *entry,
		const char *procname, void *data, int maxlen,
		mode_t mode, proc_handler *proc_handler)
{
	entry->procname = procname;
	entry->data = data;
	entry->maxlen = maxlen;
	entry->mode = mode;
	entry->proc_handler = proc_handler;
}

static struct ctl_table *
sd_alloc_ctl_domain_table(struct sched_domain *sd)
{
	struct ctl_table *table = sd_alloc_ctl_entry(14);

	if (table == NULL)
		return NULL;

	set_table_entry(&table[0], "min_interval", &sd->min_interval,
		sizeof(long), 0644, proc_doulongvec_minmax);
	set_table_entry(&table[1], "max_interval", &sd->max_interval,
		sizeof(long), 0644, proc_doulongvec_minmax);
	set_table_entry(&table[2], "busy_idx", &sd->busy_idx,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[3], "idle_idx", &sd->idle_idx,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[4], "newidle_idx", &sd->newidle_idx,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[5], "wake_idx", &sd->wake_idx,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[6], "forkexec_idx", &sd->forkexec_idx,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[7], "busy_factor", &sd->busy_factor,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[8], "imbalance_pct", &sd->imbalance_pct,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[9], "cache_nice_tries",
		&sd->cache_nice_tries,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[10], "flags", &sd->flags,
		sizeof(int), 0644, proc_dointvec_minmax);
	set_table_entry(&table[11], "max_newidle_lb_cost",
		&sd->max_newidle_lb_cost,
		sizeof(long), 0644, proc_doulongvec_minmax);
	set_table_entry(&table[12], "name", sd->name,
		CORENAME_MAX_SIZE, 0444, proc_dostring);
	/* &table[13] is terminator */

	return table;
}

static struct ctl_table *sd_alloc_ctl_cpu_table(int cpu)
{
	struct ctl_table *entry, *table;
	struct sched_domain *sd;
	int domain_num = 0, i;
	char buf[32];

	for_each_domain(cpu, sd)
		domain_num++;
	entry = table = sd_alloc_ctl_entry(domain_num + 1);
	if (table == NULL)
		return NULL;

	i = 0;
	for_each_domain(cpu, sd) {
		snprintf(buf, 32, "domain%d", i);
		entry->procname = kstrdup(buf, GFP_KERNEL);
		entry->mode = 0555;
		entry->child = sd_alloc_ctl_domain_table(sd);
		entry++;
		i++;
	}
	return table;
}

static struct ctl_table_header *sd_sysctl_header;
static void register_sched_domain_sysctl(void)
{
	int i, cpu_num = num_possible_cpus();
	struct ctl_table *entry = sd_alloc_ctl_entry(cpu_num + 1);
	char buf[32];

	WARN_ON(sd_ctl_dir[0].child);
	sd_ctl_dir[0].child = entry;

	if (entry == NULL)
		return;

	for_each_possible_cpu(i) {
		snprintf(buf, 32, "cpu%d", i);
		entry->procname = kstrdup(buf, GFP_KERNEL);
		entry->mode = 0555;
		entry->child = sd_alloc_ctl_cpu_table(i);
		entry++;
	}

	WARN_ON(sd_sysctl_header);
	sd_sysctl_header = register_sysctl_table(sd_ctl_root);
}

/* may be called multiple times per register */
static void unregister_sched_domain_sysctl(void)
{
	unregister_sysctl_table(sd_sysctl_header);
	sd_sysctl_header = NULL;
	if (sd_ctl_dir[0].child)
		sd_free_ctl_entry(&sd_ctl_dir[0].child);
}
#else /* CONFIG_SCHED_DEBUG && CONFIG_SYSCTL */
static void register_sched_domain_sysctl(void)
{
}
static void unregister_sched_domain_sysctl(void)
{
}
#endif /* CONFIG_SCHED_DEBUG && CONFIG_SYSCTL */

static void set_rq_online(struct rq *rq)
{
	if (!rq->online) {
		cpumask_set_cpu(cpu_of(rq), rq->rd->online);
		rq->online = true;
	}
}

static void set_rq_offline(struct rq *rq)
{
	if (rq->online) {
		cpumask_clear_cpu(cpu_of(rq), rq->rd->online);
		rq->online = false;
	}
}

/* Run through task list and find tasks affined to the dead cpu, then remove
 * that cpu from the list, enable cpu0 and set the zerobound flag. */
static void tasks_cpu_hotplug(int cpu)
{
	struct task_struct *p, *t;
	int count = 0;

	if (cpu == 0)
		return;

	do_each_thread(t, p) {
		if (cpumask_test_cpu(cpu, &p->cpus_allowed_master)) {
			count++;
			if (unlikely(!cpumask_and(tsk_cpus_allowed(p),
						  &p->cpus_allowed_master,
						  cpu_active_mask)))
				cpumask_set_cpu(0, tsk_cpus_allowed(p));
			if (p->cached == 1ULL && task_cpu(p) == cpu)
				p->cached = 0ULL;
		}
	} while_each_thread(t, p);

	if (count) {
		printk(KERN_INFO "Renew affinity for %d processes to cpu %d\n",
		       count, cpu);
	}
}
/*
 * migration_call - callback that gets triggered when a CPU is added.
 */
static int
migration_call(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	int cpu = (long)hcpu;
	unsigned long flags;
	struct rq *rq = cpu_rq(cpu);
#ifdef CONFIG_HOTPLUG_CPU
	struct task_struct *idle = rq->idle;
#endif

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		return NOTIFY_OK;
	case CPU_UP_PREPARE:
		break;

	case CPU_ONLINE:
		/* Update our root-domain */
		read_lock(&tasklist_lock);
		rq_grq_lock_irqsave(rq, &flags);
		if (rq->rd) {
			BUG_ON(!cpumask_test_cpu(cpu, rq->rd->span));

			set_rq_online(rq);
		}
		tasks_cpu_hotplug(cpu);
		grq.noc = num_online_cpus();
		cpumask_set_cpu(cpu, &grq.cpu_idle_map);
		rq_grq_unlock_irqrestore(rq, &flags);
		read_unlock(&tasklist_lock);
		break;

#ifdef CONFIG_HOTPLUG_CPU
	case CPU_DEAD:
		rq_grq_lock_irqsave(rq, &flags);
		set_rq_task(rq, idle);
		update_rq_clock(rq);
		rq_grq_unlock_irqrestore(rq, &flags);
		break;

	case CPU_DYING:
		/* Update our root-domain */
		read_lock(&tasklist_lock);
		rq_grq_lock_irqsave(rq, &flags);
		if (rq->rd) {
			BUG_ON(!cpumask_test_cpu(cpu, rq->rd->span));
			set_rq_offline(rq);
		}
		tasks_cpu_hotplug(cpu);
		grq.noc = num_online_cpus();
		cpumask_clear_cpu(cpu, &grq.cpu_idle_map);
		rq_grq_unlock_irqrestore(rq, &flags);
		read_unlock(&tasklist_lock);
		break;
#endif
	}
	return NOTIFY_OK;
}

/*
 * Register at high priority so that task migration (migrate_all_tasks)
 * happens before everything else.  This has to be lower priority than
 * the notifier in the perf_counter subsystem, though.
 */
static struct notifier_block  migration_notifier = {
	.notifier_call = migration_call,
	.priority = CPU_PRI_MIGRATION,
};

static int sched_cpu_active(struct notifier_block *nfb,
				      unsigned long action, void *hcpu)
{
	int cpu = (long)hcpu;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		return NOTIFY_OK;
	case CPU_ONLINE:
		/*
		 * At this point a starting CPU has marked itself as online via
		 * set_cpu_online(). But it might not yet have marked itself
		 * as active, which is essential from here on.
		 */
		set_cpu_active(cpu, true);
		stop_machine_unpark(cpu);
		return NOTIFY_OK;

	case CPU_DOWN_FAILED:
		set_cpu_active(cpu, true);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static int sched_cpu_inactive(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_PREPARE:
		set_cpu_active((long)hcpu, false);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

int __init migration_init(void)
{
	void *cpu = (void *)(long)smp_processor_id();
	int err;

	/* Initialise migration for the boot CPU */
	err = migration_call(&migration_notifier, CPU_UP_PREPARE, cpu);
	BUG_ON(err == NOTIFY_BAD);
	migration_call(&migration_notifier, CPU_ONLINE, cpu);
	register_cpu_notifier(&migration_notifier);

	/* Register cpu active notifiers */
	cpu_notifier(sched_cpu_active, CPU_PRI_SCHED_ACTIVE);
	cpu_notifier(sched_cpu_inactive, CPU_PRI_SCHED_INACTIVE);

	return 0;
}
early_initcall(migration_init);

static cpumask_var_t sched_domains_tmpmask; /* sched_domains_mutex */

#ifdef CONFIG_SCHED_DEBUG

static __read_mostly int sched_debug_enabled;

static int __init sched_debug_setup(char *str)
{
	sched_debug_enabled = 1;

	return 0;
}
early_param("sched_debug", sched_debug_setup);

static inline bool sched_debug(void)
{
	return sched_debug_enabled;
}

static int sched_domain_debug_one(struct sched_domain *sd, int cpu, int level,
				  struct cpumask *groupmask)
{
	cpumask_clear(groupmask);

	printk(KERN_DEBUG "%*s domain %d: ", level, "", level);

	if (!(sd->flags & SD_LOAD_BALANCE)) {
		printk("does not load-balance\n");
		if (sd->parent)
			printk(KERN_ERR "ERROR: !SD_LOAD_BALANCE domain"
					" has parent");
		return -1;
	}

	printk(KERN_CONT "span %*pbl level %s\n",
	       cpumask_pr_args(sched_domain_span(sd)), sd->name);

	if (!cpumask_test_cpu(cpu, sched_domain_span(sd))) {
		printk(KERN_ERR "ERROR: domain->span does not contain "
				"CPU%d\n", cpu);
	}

	printk(KERN_CONT "\n");

	if (!cpumask_equal(sched_domain_span(sd), groupmask))
		printk(KERN_ERR "ERROR: groups don't span domain->span\n");

	if (sd->parent &&
	    !cpumask_subset(groupmask, sched_domain_span(sd->parent)))
		printk(KERN_ERR "ERROR: parent span is not a superset "
			"of domain->span\n");
	return 0;
}

static void sched_domain_debug(struct sched_domain *sd, int cpu)
{
	int level = 0;

	if (!sched_debug_enabled)
		return;

	if (!sd) {
		printk(KERN_DEBUG "CPU%d attaching NULL sched-domain.\n", cpu);
		return;
	}

	printk(KERN_DEBUG "CPU%d attaching sched-domain:\n", cpu);

	for (;;) {
		if (sched_domain_debug_one(sd, cpu, level, sched_domains_tmpmask))
			break;
		level++;
		sd = sd->parent;
		if (!sd)
			break;
	}
}
#else /* !CONFIG_SCHED_DEBUG */
# define sched_domain_debug(sd, cpu) do { } while (0)
static inline bool sched_debug(void)
{
	return false;
}
#endif /* CONFIG_SCHED_DEBUG */

static int sd_degenerate(struct sched_domain *sd)
{
	if (cpumask_weight(sched_domain_span(sd)) == 1)
		return 1;

	/* Following flags don't use groups */
	if (sd->flags & (SD_WAKE_AFFINE))
		return 0;

	return 1;
}

static int
sd_parent_degenerate(struct sched_domain *sd, struct sched_domain *parent)
{
	unsigned long cflags = sd->flags, pflags = parent->flags;

	if (sd_degenerate(parent))
		return 1;

	if (!cpumask_equal(sched_domain_span(sd), sched_domain_span(parent)))
		return 0;

	if (~cflags & pflags)
		return 0;

	return 1;
}

static void free_rootdomain(struct rcu_head *rcu)
{
	struct root_domain *rd = container_of(rcu, struct root_domain, rcu);

	cpupri_cleanup(&rd->cpupri);
	free_cpumask_var(rd->rto_mask);
	free_cpumask_var(rd->online);
	free_cpumask_var(rd->span);
	kfree(rd);
}

static void rq_attach_root(struct rq *rq, struct root_domain *rd)
{
	struct root_domain *old_rd = NULL;
	unsigned long flags;

	rq_grq_lock_irqsave(rq, &flags);

	if (rq->rd) {
		old_rd = rq->rd;

		if (cpumask_test_cpu(rq->cpu, old_rd->online))
			set_rq_offline(rq);

		cpumask_clear_cpu(rq->cpu, old_rd->span);

		/*
		 * If we dont want to free the old_rd yet then
		 * set old_rd to NULL to skip the freeing later
		 * in this function:
		 */
		if (!atomic_dec_and_test(&old_rd->refcount))
			old_rd = NULL;
	}

	atomic_inc(&rd->refcount);
	rq->rd = rd;

	cpumask_set_cpu(rq->cpu, rd->span);
	if (cpumask_test_cpu(rq->cpu, cpu_active_mask))
		set_rq_online(rq);

	rq_grq_unlock_irqrestore(rq, &flags);

	if (old_rd)
		call_rcu_sched(&old_rd->rcu, free_rootdomain);
}

static int init_rootdomain(struct root_domain *rd)
{
	memset(rd, 0, sizeof(*rd));

	if (!zalloc_cpumask_var(&rd->span, GFP_KERNEL))
		goto out;
	if (!zalloc_cpumask_var(&rd->online, GFP_KERNEL))
		goto free_span;
	if (!zalloc_cpumask_var(&rd->rto_mask, GFP_KERNEL))
		goto free_online;

	if (cpupri_init(&rd->cpupri) != 0)
		goto free_rto_mask;
	return 0;

free_rto_mask:
	free_cpumask_var(rd->rto_mask);
free_online:
	free_cpumask_var(rd->online);
free_span:
	free_cpumask_var(rd->span);
out:
	return -ENOMEM;
}

static void init_defrootdomain(void)
{
	init_rootdomain(&def_root_domain);

	atomic_set(&def_root_domain.refcount, 1);
}

static struct root_domain *alloc_rootdomain(void)
{
	struct root_domain *rd;

	rd = kmalloc(sizeof(*rd), GFP_KERNEL);
	if (!rd)
		return NULL;

	if (init_rootdomain(rd) != 0) {
		kfree(rd);
		return NULL;
	}

	return rd;
}

static void free_sched_domain(struct rcu_head *rcu)
{
	struct sched_domain *sd = container_of(rcu, struct sched_domain, rcu);

	kfree(sd);
}

static void destroy_sched_domain(struct sched_domain *sd, int cpu)
{
	call_rcu(&sd->rcu, free_sched_domain);
}

static void destroy_sched_domains(struct sched_domain *sd, int cpu)
{
	for (; sd; sd = sd->parent)
		destroy_sched_domain(sd, cpu);
}

/*
 * Attach the domain 'sd' to 'cpu' as its base domain. Callers must
 * hold the hotplug lock.
 */
static void
cpu_attach_domain(struct sched_domain *sd, struct root_domain *rd, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	struct sched_domain *tmp;

	/* Remove the sched domains which do not contribute to scheduling. */
	for (tmp = sd; tmp; ) {
		struct sched_domain *parent = tmp->parent;
		if (!parent)
			break;

		if (sd_parent_degenerate(tmp, parent)) {
			tmp->parent = parent->parent;
			if (parent->parent)
				parent->parent->child = tmp;
			/*
			 * Transfer SD_PREFER_SIBLING down in case of a
			 * degenerate parent; the spans match for this
			 * so the property transfers.
			 */
			if (parent->flags & SD_PREFER_SIBLING)
				tmp->flags |= SD_PREFER_SIBLING;
			destroy_sched_domain(parent, cpu);
		} else
			tmp = tmp->parent;
	}

	if (sd && sd_degenerate(sd)) {
		tmp = sd;
		sd = sd->parent;
		destroy_sched_domain(tmp, cpu);
		if (sd)
			sd->child = NULL;
	}

	sched_domain_debug(sd, cpu);

	rq_attach_root(rq, rd);
	tmp = rq->sd;
	rcu_assign_pointer(rq->sd, sd);
	destroy_sched_domains(tmp, cpu);
}

/* Setup the mask of cpus configured for isolated domains */
static int __init isolated_cpu_setup(char *str)
{
	alloc_bootmem_cpumask_var(&cpu_isolated_map);
	cpulist_parse(str, cpu_isolated_map);
	return 1;
}

__setup("isolcpus=", isolated_cpu_setup);

struct s_data {
	struct sched_domain ** __percpu sd;
	struct root_domain	*rd;
};

enum s_alloc {
	sa_rootdomain,
	sa_sd,
	sa_sd_storage,
	sa_none,
};

/*
 * Initializers for schedule domains
 * Non-inlined to reduce accumulated stack pressure in build_sched_domains()
 */

static int default_relax_domain_level = -1;
int sched_domain_level_max;

static int __init setup_relax_domain_level(char *str)
{
	if (kstrtoint(str, 0, &default_relax_domain_level))
		pr_warn("Unable to set relax_domain_level\n");

	return 1;
}
__setup("relax_domain_level=", setup_relax_domain_level);

static void set_domain_attribute(struct sched_domain *sd,
				 struct sched_domain_attr *attr)
{
	int request;

	if (!attr || attr->relax_domain_level < 0) {
		if (default_relax_domain_level < 0)
			return;
		else
			request = default_relax_domain_level;
	} else
		request = attr->relax_domain_level;
	if (request < sd->level) {
		/* turn off idle balance on this domain */
		sd->flags &= ~(SD_BALANCE_WAKE|SD_BALANCE_NEWIDLE);
	} else {
		/* turn on idle balance on this domain */
		sd->flags |= (SD_BALANCE_WAKE|SD_BALANCE_NEWIDLE);
	}
}

static void __sdt_free(const struct cpumask *cpu_map);
static int __sdt_alloc(const struct cpumask *cpu_map);

static void __free_domain_allocs(struct s_data *d, enum s_alloc what,
				 const struct cpumask *cpu_map)
{
	switch (what) {
	case sa_rootdomain:
		if (!atomic_read(&d->rd->refcount))
			free_rootdomain(&d->rd->rcu); /* fall through */
	case sa_sd:
		free_percpu(d->sd); /* fall through */
	case sa_sd_storage:
		__sdt_free(cpu_map); /* fall through */
	case sa_none:
		break;
	}
}

static enum s_alloc __visit_domain_allocation_hell(struct s_data *d,
						   const struct cpumask *cpu_map)
{
	memset(d, 0, sizeof(*d));

	if (__sdt_alloc(cpu_map))
		return sa_sd_storage;
	d->sd = alloc_percpu(struct sched_domain *);
	if (!d->sd)
		return sa_sd_storage;
	d->rd = alloc_rootdomain();
	if (!d->rd)
		return sa_sd;
	return sa_rootdomain;
}

/*
 * NULL the sd_data elements we've used to build the sched_domain
 * structure so that the subsequent __free_domain_allocs()
 * will not free the data we're using.
 */
static void claim_allocations(int cpu, struct sched_domain *sd)
{
	struct sd_data *sdd = sd->private;

	WARN_ON_ONCE(*per_cpu_ptr(sdd->sd, cpu) != sd);
	*per_cpu_ptr(sdd->sd, cpu) = NULL;
}

#ifdef CONFIG_NUMA
static int sched_domains_numa_levels;
static int *sched_domains_numa_distance;
static struct cpumask ***sched_domains_numa_masks;
static int sched_domains_curr_level;
#endif

/*
 * SD_flags allowed in topology descriptions.
 *
 * SD_SHARE_CPUCAPACITY      - describes SMT topologies
 * SD_SHARE_PKG_RESOURCES - describes shared caches
 * SD_NUMA                - describes NUMA topologies
 * SD_SHARE_POWERDOMAIN   - describes shared power domain
 *
 * Odd one out:
 * SD_ASYM_PACKING        - describes SMT quirks
 */
#define TOPOLOGY_SD_FLAGS		\
	(SD_SHARE_CPUCAPACITY |		\
	 SD_SHARE_PKG_RESOURCES |	\
	 SD_NUMA |			\
	 SD_ASYM_PACKING |		\
	 SD_SHARE_POWERDOMAIN)

static struct sched_domain *
sd_init(struct sched_domain_topology_level *tl, int cpu)
{
	struct sched_domain *sd = *per_cpu_ptr(tl->data.sd, cpu);
	int sd_weight, sd_flags = 0;

#ifdef CONFIG_NUMA
	/*
	 * Ugly hack to pass state to sd_numa_mask()...
	 */
	sched_domains_curr_level = tl->numa_level;
#endif

	sd_weight = cpumask_weight(tl->mask(cpu));

	if (tl->sd_flags)
		sd_flags = (*tl->sd_flags)();
	if (WARN_ONCE(sd_flags & ~TOPOLOGY_SD_FLAGS,
			"wrong sd_flags in topology description\n"))
		sd_flags &= ~TOPOLOGY_SD_FLAGS;

	*sd = (struct sched_domain){
		.min_interval		= sd_weight,
		.max_interval		= 2*sd_weight,
		.busy_factor		= 32,
		.imbalance_pct		= 125,

		.cache_nice_tries	= 0,
		.busy_idx		= 0,
		.idle_idx		= 0,
		.newidle_idx		= 0,
		.wake_idx		= 0,
		.forkexec_idx		= 0,

		.flags			= 1*SD_LOAD_BALANCE
					| 1*SD_BALANCE_NEWIDLE
					| 1*SD_BALANCE_EXEC
					| 1*SD_BALANCE_FORK
					| 0*SD_BALANCE_WAKE
					| 1*SD_WAKE_AFFINE
					| 0*SD_SHARE_CPUCAPACITY
					| 0*SD_SHARE_PKG_RESOURCES
					| 0*SD_SERIALIZE
					| 0*SD_PREFER_SIBLING
					| 0*SD_NUMA
					| sd_flags
					,

		.last_balance		= jiffies,
		.balance_interval	= sd_weight,
		.smt_gain		= 0,
		.max_newidle_lb_cost	= 0,
		.next_decay_max_lb_cost	= jiffies,
#ifdef CONFIG_SCHED_DEBUG
		.name			= tl->name,
#endif
	};

	/*
	 * Convert topological properties into behaviour.
	 */

	if (sd->flags & SD_SHARE_CPUCAPACITY) {
		sd->flags |= SD_PREFER_SIBLING;
		sd->imbalance_pct = 110;
		sd->smt_gain = 1178; /* ~15% */

	} else if (sd->flags & SD_SHARE_PKG_RESOURCES) {
		sd->imbalance_pct = 117;
		sd->cache_nice_tries = 1;
		sd->busy_idx = 2;

#ifdef CONFIG_NUMA
	} else if (sd->flags & SD_NUMA) {
		sd->cache_nice_tries = 2;
		sd->busy_idx = 3;
		sd->idle_idx = 2;

		sd->flags |= SD_SERIALIZE;
		if (sched_domains_numa_distance[tl->numa_level] > RECLAIM_DISTANCE) {
			sd->flags &= ~(SD_BALANCE_EXEC |
				       SD_BALANCE_FORK |
				       SD_WAKE_AFFINE);
		}

#endif
	} else {
		sd->flags |= SD_PREFER_SIBLING;
		sd->cache_nice_tries = 1;
		sd->busy_idx = 2;
		sd->idle_idx = 1;
	}

	sd->private = &tl->data;

	return sd;
}

/*
 * Topology list, bottom-up.
 */
static struct sched_domain_topology_level default_topology[] = {
#ifdef CONFIG_SCHED_SMT
	{ cpu_smt_mask, cpu_smt_flags, SD_INIT_NAME(SMT) },
#endif
#ifdef CONFIG_SCHED_MC
	{ cpu_coregroup_mask, cpu_core_flags, SD_INIT_NAME(MC) },
#endif
	{ cpu_cpu_mask, SD_INIT_NAME(DIE) },
	{ NULL, },
};

static struct sched_domain_topology_level *sched_domain_topology =
	default_topology;

#define for_each_sd_topology(tl)			\
	for (tl = sched_domain_topology; tl->mask; tl++)

void set_sched_topology(struct sched_domain_topology_level *tl)
{
	sched_domain_topology = tl;
}

#ifdef CONFIG_NUMA

static const struct cpumask *sd_numa_mask(int cpu)
{
	return sched_domains_numa_masks[sched_domains_curr_level][cpu_to_node(cpu)];
}

static void sched_numa_warn(const char *str)
{
	static int done = false;
	int i,j;

	if (done)
		return;

	done = true;

	printk(KERN_WARNING "ERROR: %s\n\n", str);

	for (i = 0; i < nr_node_ids; i++) {
		printk(KERN_WARNING "  ");
		for (j = 0; j < nr_node_ids; j++)
			printk(KERN_CONT "%02d ", node_distance(i,j));
		printk(KERN_CONT "\n");
	}
	printk(KERN_WARNING "\n");
}

static bool find_numa_distance(int distance)
{
	int i;

	if (distance == node_distance(0, 0))
		return true;

	for (i = 0; i < sched_domains_numa_levels; i++) {
		if (sched_domains_numa_distance[i] == distance)
			return true;
	}

	return false;
}

static void sched_init_numa(void)
{
	int next_distance, curr_distance = node_distance(0, 0);
	struct sched_domain_topology_level *tl;
	int level = 0;
	int i, j, k;

	sched_domains_numa_distance = kzalloc(sizeof(int) * nr_node_ids, GFP_KERNEL);
	if (!sched_domains_numa_distance)
		return;

	/*
	 * O(nr_nodes^2) deduplicating selection sort -- in order to find the
	 * unique distances in the node_distance() table.
	 *
	 * Assumes node_distance(0,j) includes all distances in
	 * node_distance(i,j) in order to avoid cubic time.
	 */
	next_distance = curr_distance;
	for (i = 0; i < nr_node_ids; i++) {
		for (j = 0; j < nr_node_ids; j++) {
			for (k = 0; k < nr_node_ids; k++) {
				int distance = node_distance(i, k);

				if (distance > curr_distance &&
				    (distance < next_distance ||
				     next_distance == curr_distance))
					next_distance = distance;

				/*
				 * While not a strong assumption it would be nice to know
				 * about cases where if node A is connected to B, B is not
				 * equally connected to A.
				 */
				if (sched_debug() && node_distance(k, i) != distance)
					sched_numa_warn("Node-distance not symmetric");

				if (sched_debug() && i && !find_numa_distance(distance))
					sched_numa_warn("Node-0 not representative");
			}
			if (next_distance != curr_distance) {
				sched_domains_numa_distance[level++] = next_distance;
				sched_domains_numa_levels = level;
				curr_distance = next_distance;
			} else break;
		}

		/*
		 * In case of sched_debug() we verify the above assumption.
		 */
		if (!sched_debug())
			break;
	}

	if (!level)
		return;

	/*
	 * 'level' contains the number of unique distances, excluding the
	 * identity distance node_distance(i,i).
	 *
	 * The sched_domains_numa_distance[] array includes the actual distance
	 * numbers.
	 */

	/*
	 * Here, we should temporarily reset sched_domains_numa_levels to 0.
	 * If it fails to allocate memory for array sched_domains_numa_masks[][],
	 * the array will contain less then 'level' members. This could be
	 * dangerous when we use it to iterate array sched_domains_numa_masks[][]
	 * in other functions.
	 *
	 * We reset it to 'level' at the end of this function.
	 */
	sched_domains_numa_levels = 0;

	sched_domains_numa_masks = kzalloc(sizeof(void *) * level, GFP_KERNEL);
	if (!sched_domains_numa_masks)
		return;

	/*
	 * Now for each level, construct a mask per node which contains all
	 * cpus of nodes that are that many hops away from us.
	 */
	for (i = 0; i < level; i++) {
		sched_domains_numa_masks[i] =
			kzalloc(nr_node_ids * sizeof(void *), GFP_KERNEL);
		if (!sched_domains_numa_masks[i])
			return;

		for (j = 0; j < nr_node_ids; j++) {
			struct cpumask *mask = kzalloc(cpumask_size(), GFP_KERNEL);
			if (!mask)
				return;

			sched_domains_numa_masks[i][j] = mask;

			for_each_node(k) {
				if (node_distance(j, k) > sched_domains_numa_distance[i])
					continue;

				cpumask_or(mask, mask, cpumask_of_node(k));
			}
		}
	}

	/* Compute default topology size */
	for (i = 0; sched_domain_topology[i].mask; i++);

	tl = kzalloc((i + level + 1) *
			sizeof(struct sched_domain_topology_level), GFP_KERNEL);
	if (!tl)
		return;

	/*
	 * Copy the default topology bits..
	 */
	for (i = 0; sched_domain_topology[i].mask; i++)
		tl[i] = sched_domain_topology[i];

	/*
	 * .. and append 'j' levels of NUMA goodness.
	 */
	for (j = 0; j < level; i++, j++) {
		tl[i] = (struct sched_domain_topology_level){
			.mask = sd_numa_mask,
			.sd_flags = cpu_numa_flags,
			.flags = SDTL_OVERLAP,
			.numa_level = j,
			SD_INIT_NAME(NUMA)
		};
	}

	sched_domain_topology = tl;

	sched_domains_numa_levels = level;
}

static void sched_domains_numa_masks_set(int cpu)
{
	int i, j;
	int node = cpu_to_node(cpu);

	for (i = 0; i < sched_domains_numa_levels; i++) {
		for (j = 0; j < nr_node_ids; j++) {
			if (node_distance(j, node) <= sched_domains_numa_distance[i])
				cpumask_set_cpu(cpu, sched_domains_numa_masks[i][j]);
		}
	}
}

static void sched_domains_numa_masks_clear(int cpu)
{
	int i, j;
	for (i = 0; i < sched_domains_numa_levels; i++) {
		for (j = 0; j < nr_node_ids; j++)
			cpumask_clear_cpu(cpu, sched_domains_numa_masks[i][j]);
	}
}

/*
 * Update sched_domains_numa_masks[level][node] array when new cpus
 * are onlined.
 */
static int sched_domains_numa_masks_update(struct notifier_block *nfb,
					   unsigned long action,
					   void *hcpu)
{
	int cpu = (long)hcpu;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		sched_domains_numa_masks_set(cpu);
		break;

	case CPU_DEAD:
		sched_domains_numa_masks_clear(cpu);
		break;

	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}
#else
static inline void sched_init_numa(void)
{
}

static int sched_domains_numa_masks_update(struct notifier_block *nfb,
					   unsigned long action,
					   void *hcpu)
{
	return 0;
}
#endif /* CONFIG_NUMA */

static int __sdt_alloc(const struct cpumask *cpu_map)
{
	struct sched_domain_topology_level *tl;
	int j;

	for_each_sd_topology(tl) {
		struct sd_data *sdd = &tl->data;

		sdd->sd = alloc_percpu(struct sched_domain *);
		if (!sdd->sd)
			return -ENOMEM;

		for_each_cpu(j, cpu_map) {
			struct sched_domain *sd;

			sd = kzalloc_node(sizeof(struct sched_domain) + cpumask_size(),
					GFP_KERNEL, cpu_to_node(j));
			if (!sd)
				return -ENOMEM;

			*per_cpu_ptr(sdd->sd, j) = sd;
		}
	}

	return 0;
}

static void __sdt_free(const struct cpumask *cpu_map)
{
	struct sched_domain_topology_level *tl;
	int j;

	for_each_sd_topology(tl) {
		struct sd_data *sdd = &tl->data;

		for_each_cpu(j, cpu_map) {
			struct sched_domain *sd;

			if (sdd->sd) {
				sd = *per_cpu_ptr(sdd->sd, j);
				kfree(*per_cpu_ptr(sdd->sd, j));
			}
		}
		free_percpu(sdd->sd);
		sdd->sd = NULL;
	}
}

struct sched_domain *build_sched_domain(struct sched_domain_topology_level *tl,
		const struct cpumask *cpu_map, struct sched_domain_attr *attr,
		struct sched_domain *child, int cpu)
{
	struct sched_domain *sd = sd_init(tl, cpu);
	if (!sd)
		return child;

	cpumask_and(sched_domain_span(sd), cpu_map, tl->mask(cpu));
	if (child) {
		sd->level = child->level + 1;
		sched_domain_level_max = max(sched_domain_level_max, sd->level);
		child->parent = sd;
		sd->child = child;

		if (!cpumask_subset(sched_domain_span(child),
				    sched_domain_span(sd))) {
			pr_err("BUG: arch topology borken\n");
#ifdef CONFIG_SCHED_DEBUG
			pr_err("     the %s domain not a subset of the %s domain\n",
					child->name, sd->name);
#endif
			/* Fixup, ensure @sd has at least @child cpus. */
			cpumask_or(sched_domain_span(sd),
				   sched_domain_span(sd),
				   sched_domain_span(child));
		}

	}
	set_domain_attribute(sd, attr);

	return sd;
}

/*
 * Build sched domains for a given set of cpus and attach the sched domains
 * to the individual cpus
 */
static int build_sched_domains(const struct cpumask *cpu_map,
			       struct sched_domain_attr *attr)
{
	enum s_alloc alloc_state;
	struct sched_domain *sd;
	struct s_data d;
	int i, ret = -ENOMEM;

	alloc_state = __visit_domain_allocation_hell(&d, cpu_map);
	if (alloc_state != sa_rootdomain)
		goto error;

	/* Set up domains for cpus specified by the cpu_map. */
	for_each_cpu(i, cpu_map) {
		struct sched_domain_topology_level *tl;

		sd = NULL;
		for_each_sd_topology(tl) {
			sd = build_sched_domain(tl, cpu_map, attr, sd, i);
			if (tl == sched_domain_topology)
				*per_cpu_ptr(d.sd, i) = sd;
			if (tl->flags & SDTL_OVERLAP)
				sd->flags |= SD_OVERLAP;
			if (cpumask_equal(cpu_map, sched_domain_span(sd)))
				break;
		}
	}

	/* Calculate CPU capacity for physical packages and nodes */
	for (i = nr_cpumask_bits-1; i >= 0; i--) {
		if (!cpumask_test_cpu(i, cpu_map))
			continue;

		for (sd = *per_cpu_ptr(d.sd, i); sd; sd = sd->parent) {
			claim_allocations(i, sd);
		}
	}

	/* Attach the domains */
	rcu_read_lock();
	for_each_cpu(i, cpu_map) {
		sd = *per_cpu_ptr(d.sd, i);
		cpu_attach_domain(sd, d.rd, i);
	}
	rcu_read_unlock();

	ret = 0;
error:
	__free_domain_allocs(&d, alloc_state, cpu_map);
	return ret;
}

static cpumask_var_t *doms_cur;	/* current sched domains */
static int ndoms_cur;		/* number of sched domains in 'doms_cur' */
static struct sched_domain_attr *dattr_cur;
				/* attribues of custom domains in 'doms_cur' */

/*
 * Special case: If a kmalloc of a doms_cur partition (array of
 * cpumask) fails, then fallback to a single sched domain,
 * as determined by the single cpumask fallback_doms.
 */
static cpumask_var_t fallback_doms;

/*
 * arch_update_cpu_topology lets virtualized architectures update the
 * cpu core maps. It is supposed to return 1 if the topology changed
 * or 0 if it stayed the same.
 */
int __weak arch_update_cpu_topology(void)
{
	return 0;
}

cpumask_var_t *alloc_sched_domains(unsigned int ndoms)
{
	int i;
	cpumask_var_t *doms;

	doms = kmalloc(sizeof(*doms) * ndoms, GFP_KERNEL);
	if (!doms)
		return NULL;
	for (i = 0; i < ndoms; i++) {
		if (!alloc_cpumask_var(&doms[i], GFP_KERNEL)) {
			free_sched_domains(doms, i);
			return NULL;
		}
	}
	return doms;
}

void free_sched_domains(cpumask_var_t doms[], unsigned int ndoms)
{
	unsigned int i;
	for (i = 0; i < ndoms; i++)
		free_cpumask_var(doms[i]);
	kfree(doms);
}

/*
 * Set up scheduler domains and groups. Callers must hold the hotplug lock.
 * For now this just excludes isolated cpus, but could be used to
 * exclude other special cases in the future.
 */
static int init_sched_domains(const struct cpumask *cpu_map)
{
	int err;

	arch_update_cpu_topology();
	ndoms_cur = 1;
	doms_cur = alloc_sched_domains(ndoms_cur);
	if (!doms_cur)
		doms_cur = &fallback_doms;
	cpumask_andnot(doms_cur[0], cpu_map, cpu_isolated_map);
	err = build_sched_domains(doms_cur[0], NULL);
	register_sched_domain_sysctl();

	return err;
}

/*
 * Detach sched domains from a group of cpus specified in cpu_map
 * These cpus will now be attached to the NULL domain
 */
static void detach_destroy_domains(const struct cpumask *cpu_map)
{
	int i;

	rcu_read_lock();
	for_each_cpu(i, cpu_map)
		cpu_attach_domain(NULL, &def_root_domain, i);
	rcu_read_unlock();
}

/* handle null as "default" */
static int dattrs_equal(struct sched_domain_attr *cur, int idx_cur,
			struct sched_domain_attr *new, int idx_new)
{
	struct sched_domain_attr tmp;

	/* fast path */
	if (!new && !cur)
		return 1;

	tmp = SD_ATTR_INIT;
	return !memcmp(cur ? (cur + idx_cur) : &tmp,
			new ? (new + idx_new) : &tmp,
			sizeof(struct sched_domain_attr));
}

/*
 * Partition sched domains as specified by the 'ndoms_new'
 * cpumasks in the array doms_new[] of cpumasks. This compares
 * doms_new[] to the current sched domain partitioning, doms_cur[].
 * It destroys each deleted domain and builds each new domain.
 *
 * 'doms_new' is an array of cpumask_var_t's of length 'ndoms_new'.
 * The masks don't intersect (don't overlap.) We should setup one
 * sched domain for each mask. CPUs not in any of the cpumasks will
 * not be load balanced. If the same cpumask appears both in the
 * current 'doms_cur' domains and in the new 'doms_new', we can leave
 * it as it is.
 *
 * The passed in 'doms_new' should be allocated using
 * alloc_sched_domains.  This routine takes ownership of it and will
 * free_sched_domains it when done with it. If the caller failed the
 * alloc call, then it can pass in doms_new == NULL && ndoms_new == 1,
 * and partition_sched_domains() will fallback to the single partition
 * 'fallback_doms', it also forces the domains to be rebuilt.
 *
 * If doms_new == NULL it will be replaced with cpu_online_mask.
 * ndoms_new == 0 is a special case for destroying existing domains,
 * and it will not create the default domain.
 *
 * Call with hotplug lock held
 */
void partition_sched_domains(int ndoms_new, cpumask_var_t doms_new[],
			     struct sched_domain_attr *dattr_new)
{
	int i, j, n;
	int new_topology;

	mutex_lock(&sched_domains_mutex);

	/* always unregister in case we don't destroy any domains */
	unregister_sched_domain_sysctl();

	/* Let architecture update cpu core mappings. */
	new_topology = arch_update_cpu_topology();

	n = doms_new ? ndoms_new : 0;

	/* Destroy deleted domains */
	for (i = 0; i < ndoms_cur; i++) {
		for (j = 0; j < n && !new_topology; j++) {
			if (cpumask_equal(doms_cur[i], doms_new[j])
			    && dattrs_equal(dattr_cur, i, dattr_new, j))
				goto match1;
		}
		/* no match - a current sched domain not in new doms_new[] */
		detach_destroy_domains(doms_cur[i]);
match1:
		;
	}

	n = ndoms_cur;
	if (doms_new == NULL) {
		n = 0;
		doms_new = &fallback_doms;
		cpumask_andnot(doms_new[0], cpu_active_mask, cpu_isolated_map);
		WARN_ON_ONCE(dattr_new);
	}

	/* Build new domains */
	for (i = 0; i < ndoms_new; i++) {
		for (j = 0; j < n && !new_topology; j++) {
			if (cpumask_equal(doms_new[i], doms_cur[j])
			    && dattrs_equal(dattr_new, i, dattr_cur, j))
				goto match2;
		}
		/* no match - add a new doms_new */
		build_sched_domains(doms_new[i], dattr_new ? dattr_new + i : NULL);
match2:
		;
	}

	/* Remember the new sched domains */
	if (doms_cur != &fallback_doms)
		free_sched_domains(doms_cur, ndoms_cur);
	kfree(dattr_cur);	/* kfree(NULL) is safe */
	doms_cur = doms_new;
	dattr_cur = dattr_new;
	ndoms_cur = ndoms_new;

	register_sched_domain_sysctl();

	mutex_unlock(&sched_domains_mutex);
}

static int num_cpus_frozen;	/* used to mark begin/end of suspend/resume */

/*
 * Update cpusets according to cpu_active mask.  If cpusets are
 * disabled, cpuset_update_active_cpus() becomes a simple wrapper
 * around partition_sched_domains().
 *
 * If we come here as part of a suspend/resume, don't touch cpusets because we
 * want to restore it back to its original state upon resume anyway.
 */
static int cpuset_cpu_active(struct notifier_block *nfb, unsigned long action,
			     void *hcpu)
{
	switch (action) {
	case CPU_ONLINE_FROZEN:
	case CPU_DOWN_FAILED_FROZEN:

		/*
		 * num_cpus_frozen tracks how many CPUs are involved in suspend
		 * resume sequence. As long as this is not the last online
		 * operation in the resume sequence, just build a single sched
		 * domain, ignoring cpusets.
		 */
		num_cpus_frozen--;
		if (likely(num_cpus_frozen)) {
			partition_sched_domains(1, NULL, NULL);
			break;
		}

		/*
		 * This is the last CPU online operation. So fall through and
		 * restore the original sched domains by considering the
		 * cpuset configurations.
		 */

	case CPU_ONLINE:
		cpuset_update_active_cpus(true);
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static int cpuset_cpu_inactive(struct notifier_block *nfb, unsigned long action,
			       void *hcpu)
{
	switch (action) {
	case CPU_DOWN_PREPARE:
		cpuset_update_active_cpus(false);
		break;
	case CPU_DOWN_PREPARE_FROZEN:
		num_cpus_frozen++;
		partition_sched_domains(1, NULL, NULL);
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

#ifdef CONFIG_SCHED_SMT
static const cpumask_t *thread_cpumask(int cpu)
{
	return topology_sibling_cpumask(cpu);
}
#endif

enum sched_domain_level {
	SD_LV_NONE = 0,
	SD_LV_SIBLING,
	SD_LV_MC,
	SD_LV_BOOK,
	SD_LV_CPU,
	SD_LV_NODE,
	SD_LV_ALLNODES,
	SD_LV_MAX
};

void __init sched_init_smp(void)
{
	struct sched_domain *sd;
	int cpu, other_cpu;

	cpumask_var_t non_isolated_cpus;

	alloc_cpumask_var(&non_isolated_cpus, GFP_KERNEL);
	alloc_cpumask_var(&fallback_doms, GFP_KERNEL);

	sched_init_numa();

	/*
	 * There's no userspace yet to cause hotplug operations; hence all the
	 * cpu masks are stable and all blatant races in the below code cannot
	 * happen.
	 */
	mutex_lock(&sched_domains_mutex);
	init_sched_domains(cpu_active_mask);
	cpumask_andnot(non_isolated_cpus, cpu_possible_mask, cpu_isolated_map);
	if (cpumask_empty(non_isolated_cpus))
		cpumask_set_cpu(smp_processor_id(), non_isolated_cpus);
	mutex_unlock(&sched_domains_mutex);

	hotcpu_notifier(sched_domains_numa_masks_update, CPU_PRI_SCHED_ACTIVE);
	hotcpu_notifier(cpuset_cpu_active, CPU_PRI_CPUSET_ACTIVE);
	hotcpu_notifier(cpuset_cpu_inactive, CPU_PRI_CPUSET_INACTIVE);

	/* Move init over to a non-isolated CPU */
	if (set_cpus_allowed_ptr(current, non_isolated_cpus) < 0)
		BUG();
	free_cpumask_var(non_isolated_cpus);

	mutex_lock(&sched_domains_mutex);
	/*
	 * Set up the relative cache distance of each online cpu from each
	 * other in a simple array for quick lookup. Locality is determined
	 * by the closest sched_domain that CPUs are separated by. CPUs with
	 * shared cache in SMT and MC are treated as local. Separate CPUs
	 * (within the same package or physically) within the same node are
	 * treated as not local. CPUs not even in the same domain (different
	 * nodes) are treated as very distant.
	 */
	for_each_online_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		raw_spin_lock_irq(&rq->lock);
		_grq_lock();
		/* First check if this cpu is in the same node */
		for_each_domain(cpu, sd) {
			if (sd->level > SD_LV_NODE)
				continue;
			/* Set locality to local node if not already found lower */
			for_each_cpu(other_cpu, sched_domain_span(sd)) {
				if (rq->cpu_locality[other_cpu] > 3)
					rq->cpu_locality[other_cpu] = 3;
			}
		}

#ifdef CONFIG_SCHED_MC
		for_each_cpu(other_cpu, cpu_coregroup_mask(cpu)) {
			if (rq->cpu_locality[other_cpu] > 2)
				rq->cpu_locality[other_cpu] = 2;
		}
#endif
#ifdef CONFIG_SCHED_SMT
		for_each_cpu(other_cpu, thread_cpumask(cpu))
			if (rq->cpu_locality[other_cpu] > 1)
				rq->cpu_locality[other_cpu] = 1;
#endif
		_grq_unlock();
		raw_spin_unlock_irq(&rq->lock);
	}
	mutex_unlock(&sched_domains_mutex);

	for_each_online_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);
		for_each_online_cpu(other_cpu) {
			if (other_cpu <= cpu)
				continue;
			printk(KERN_DEBUG "BFS LOCALITY CPU %d to %d: %d\n", cpu, other_cpu, rq->cpu_locality[other_cpu]);
		}
	}
}
#else
void __init sched_init_smp(void)
{
}
#endif /* CONFIG_SMP */

int in_sched_functions(unsigned long addr)
{
	return in_lock_functions(addr) ||
		(addr >= (unsigned long)__sched_text_start
		&& addr < (unsigned long)__sched_text_end);
}

void __init sched_init(void)
{
#ifdef CONFIG_SMP
	int cpu_ids;
#endif
	int i;
	struct rq *rq;

	prio_ratios[0] = 128;
	for (i = 1 ; i < NICE_WIDTH ; i++)
		prio_ratios[i] = prio_ratios[i - 1] * 11 / 10;

	raw_spin_lock_init(&grq.lock);
	grq.nr_running = grq.nr_uninterruptible = 0;
	raw_spin_lock_init(&grq.iso_lock);
	grq.iso_ticks = 0;
	grq.iso_refractory = false;
	grq.noc = 1;
#ifdef CONFIG_SMP
	init_defrootdomain();
	cpumask_clear(&grq.cpu_idle_map);
#ifndef CONFIG_64BIT
	raw_spin_lock_init(&grq.priodl_lock);
#endif
#else
	uprq = &per_cpu(runqueues, 0);
#endif
	for_each_possible_cpu(i) {
		rq = cpu_rq(i);
		rq->try_preempt_tsk = NULL;
		raw_spin_lock_init(&rq->lock);
		rq->user_pc = rq->nice_pc = rq->softirq_pc = rq->system_pc =
			      rq->iowait_pc = rq->idle_pc = 0;
		rq->dither = false;
#ifdef CONFIG_SMP
		rq->sd = NULL;
		rq->rd = NULL;
		rq->online = false;
		rq->cpu = i;
		rq_attach_root(rq, &def_root_domain);
#endif
		rq->nr_switches = 0ULL;
		atomic_set(&rq->nr_iowait, 0);
	}

#ifdef CONFIG_SMP
	cpu_ids = i;
	/*
	 * Set the base locality for cpu cache distance calculation to
	 * "distant" (3). Make sure the distance from a CPU to itself is 0.
	 */
	for_each_possible_cpu(i) {
		int j;

		rq = cpu_rq(i);
		rq->cpu_locality = kmalloc(cpu_ids * sizeof(int *), GFP_ATOMIC);
		for_each_possible_cpu(j) {
			if (i == j)
				rq->cpu_locality[j] = 0;
			else
				rq->cpu_locality[j] = 4;
		}
	}
#endif

	for (i = 0; i < PRIO_LIMIT; i++)
		INIT_LIST_HEAD(grq.queue + i);
	/* delimiter for bitsearch */
	__set_bit(PRIO_LIMIT, grq.prio_bitmap);

#ifdef CONFIG_PREEMPT_NOTIFIERS
	INIT_HLIST_HEAD(&init_task.preempt_notifiers);
#endif

	/*
	 * The boot idle thread does lazy MMU switching as well:
	 */
	atomic_inc(&init_mm.mm_count);
	enter_lazy_tlb(&init_mm, current);

	/*
	 * Make us the idle thread. Technically, schedule() should not be
	 * called from this thread, however somewhere below it might be,
	 * but because we are the idle thread, we just pick up running again
	 * when this runqueue becomes "idle".
	 */
	init_idle(current, smp_processor_id());

#ifdef CONFIG_SMP
	zalloc_cpumask_var(&sched_domains_tmpmask, GFP_NOWAIT);
	/* May be allocated at isolcpus cmdline parse time */
	if (cpu_isolated_map == NULL)
		zalloc_cpumask_var(&cpu_isolated_map, GFP_NOWAIT);
	idle_thread_set_boot_cpu();
#endif /* SMP */
}

#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
static inline int preempt_count_equals(int preempt_offset)
{
	int nested = preempt_count() + rcu_preempt_depth();

	return (nested == preempt_offset);
}

void __might_sleep(const char *file, int line, int preempt_offset)
{
	/*
	 * Blocking primitives will set (and therefore destroy) current->state,
	 * since we will exit with TASK_RUNNING make sure we enter with it,
	 * otherwise we will destroy state.
	 */
	WARN_ONCE(current->state != TASK_RUNNING && current->task_state_change,
			"do not call blocking ops when !TASK_RUNNING; "
			"state=%lx set at [<%p>] %pS\n",
			current->state,
			(void *)current->task_state_change,
			(void *)current->task_state_change);

	___might_sleep(file, line, preempt_offset);
}
EXPORT_SYMBOL(__might_sleep);

void ___might_sleep(const char *file, int line, int preempt_offset)
{
	static unsigned long prev_jiffy;	/* ratelimiting */

	rcu_sleep_check(); /* WARN_ON_ONCE() by default, no rate limit reqd. */
	if ((preempt_count_equals(preempt_offset) && !irqs_disabled() &&
	     !is_idle_task(current)) ||
	    system_state != SYSTEM_RUNNING || oops_in_progress)
		return;
	if (time_before(jiffies, prev_jiffy + HZ) && prev_jiffy)
		return;
	prev_jiffy = jiffies;

	printk(KERN_ERR
		"BUG: sleeping function called from invalid context at %s:%d\n",
			file, line);
	printk(KERN_ERR
		"in_atomic(): %d, irqs_disabled(): %d, pid: %d, name: %s\n",
			in_atomic(), irqs_disabled(),
			current->pid, current->comm);

	if (task_stack_end_corrupted(current))
		printk(KERN_EMERG "Thread overran stack, or stack corrupted\n");

	debug_show_held_locks(current);
	if (irqs_disabled())
		print_irqtrace_events(current);
#ifdef CONFIG_DEBUG_PREEMPT
	if (!preempt_count_equals(preempt_offset)) {
		pr_err("Preemption disabled at:");
		print_ip_sym(current->preempt_disable_ip);
		pr_cont("\n");
	}
#endif
	dump_stack();
}
EXPORT_SYMBOL(___might_sleep);
#endif

#ifdef CONFIG_MAGIC_SYSRQ
void normalize_rt_tasks(void)
{
	struct task_struct *g, *p;
	struct sched_attr attr = {
		.sched_policy = SCHED_NORMAL,
	};

	read_lock(&tasklist_lock);
	for_each_process_thread(g, p) {
		/*
		 * Only normalize user tasks:
		 */
		if (p->flags & PF_KTHREAD)
			continue;

		if (!rt_task(p) && !iso_task(p)) {
			/*
			 * Renice negative nice level userspace
			 * tasks back to 0:
			 */
			if (task_nice(p) < 0 && p->mm)
				set_user_nice(p, 0);
			continue;
		}

		/*
		 * Only normalize user tasks:
		 */
		if (p->flags & PF_KTHREAD)
			continue;

		__sched_setscheduler(p, &attr, false, false);
	}
	read_unlock(&tasklist_lock);
}
#endif /* CONFIG_MAGIC_SYSRQ */

#if defined(CONFIG_IA64) || defined(CONFIG_KGDB_KDB)
/*
 * These functions are only useful for the IA64 MCA handling, or kdb.
 *
 * They can only be called when the whole system has been
 * stopped - every CPU needs to be quiescent, and no scheduling
 * activity can take place. Using them for anything else would
 * be a serious bug, and as a result, they aren't even visible
 * under any other configuration.
 */

/**
 * curr_task - return the current task for a given cpu.
 * @cpu: the processor in question.
 *
 * ONLY VALID WHEN THE WHOLE SYSTEM IS STOPPED!
 *
 * Return: The current task for @cpu.
 */
struct task_struct *curr_task(int cpu)
{
	return cpu_curr(cpu);
}

#endif /* defined(CONFIG_IA64) || defined(CONFIG_KGDB_KDB) */

#ifdef CONFIG_IA64
/**
 * set_curr_task - set the current task for a given cpu.
 * @cpu: the processor in question.
 * @p: the task pointer to set.
 *
 * Description: This function must only be used when non-maskable interrupts
 * are serviced on a separate stack.  It allows the architecture to switch the
 * notion of the current task on a cpu in a non-blocking manner.  This function
 * must be called with all CPU's synchronised, and interrupts disabled, the
 * and caller must save the original value of the current task (see
 * curr_task() above) and restore that value before reenabling interrupts and
 * re-starting the system.
 *
 * ONLY VALID WHEN THE WHOLE SYSTEM IS STOPPED!
 */
void set_curr_task(int cpu, struct task_struct *p)
{
	cpu_curr(cpu) = p;
}

#endif

/*
 * Use precise platform statistics if available:
 */
#ifdef CONFIG_VIRT_CPU_ACCOUNTING_NATIVE
void task_cputime_adjusted(struct task_struct *p, cputime_t *ut, cputime_t *st)
{
	*ut = p->utime;
	*st = p->stime;
}
EXPORT_SYMBOL_GPL(task_cputime_adjusted);

void thread_group_cputime_adjusted(struct task_struct *p, cputime_t *ut, cputime_t *st)
{
	struct task_cputime cputime;

	thread_group_cputime(p, &cputime);

	*ut = cputime.utime;
	*st = cputime.stime;
}

void vtime_account_system_irqsafe(struct task_struct *tsk)
{
	unsigned long flags;

	local_irq_save(flags);
	vtime_account_system(tsk);
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(vtime_account_system_irqsafe);

#ifndef __ARCH_HAS_VTIME_TASK_SWITCH
void vtime_task_switch(struct task_struct *prev)
{
	if (is_idle_task(prev))
		vtime_account_idle(prev);
	else
		vtime_account_system(prev);

	vtime_account_user(prev);
	arch_vtime_task_switch(prev);
}
#endif

#else
/*
 * Perform (stime * rtime) / total, but avoid multiplication overflow by
 * losing precision when the numbers are big.
 */
static cputime_t scale_stime(u64 stime, u64 rtime, u64 total)
{
	u64 scaled;

	for (;;) {
		/* Make sure "rtime" is the bigger of stime/rtime */
		if (stime > rtime) {
			u64 tmp = rtime; rtime = stime; stime = tmp;
		}

		/* Make sure 'total' fits in 32 bits */
		if (total >> 32)
			goto drop_precision;

		/* Does rtime (and thus stime) fit in 32 bits? */
		if (!(rtime >> 32))
			break;

		/* Can we just balance rtime/stime rather than dropping bits? */
		if (stime >> 31)
			goto drop_precision;

		/* We can grow stime and shrink rtime and try to make them both fit */
		stime <<= 1;
		rtime >>= 1;
		continue;

drop_precision:
		/* We drop from rtime, it has more bits than stime */
		rtime >>= 1;
		total >>= 1;
	}

	/*
	 * Make sure gcc understands that this is a 32x32->64 multiply,
	 * followed by a 64/32->64 divide.
	 */
	scaled = div_u64((u64) (u32) stime * (u64) (u32) rtime, (u32)total);
	return (__force cputime_t) scaled;
}

/*
 * Tick based cputime accounting depend on random scheduling timeslices of a
 * task to be interrupted or not by the timer.  Depending on these
 * circumstances, the number of these interrupts may be over or
 * under-optimistic, matching the real user and system cputime with a variable
 * precision.
 *
 * Fix this by scaling these tick based values against the total runtime
 * accounted by the CFS scheduler.
 *
 * This code provides the following guarantees:
 *
 *   stime + utime == rtime
 *   stime_i+1 >= stime_i, utime_i+1 >= utime_i
 *
 * Assuming that rtime_i+1 >= rtime_i.
 */
static void cputime_adjust(struct task_cputime *curr,
			   struct prev_cputime *prev,
			   cputime_t *ut, cputime_t *st)
{
	cputime_t rtime, stime, utime;
	unsigned long flags;

	/* Serialize concurrent callers such that we can honour our guarantees */
	raw_spin_lock_irqsave(&prev->lock, flags);
	rtime = nsecs_to_cputime(curr->sum_exec_runtime);

	/*
	 * This is possible under two circumstances:
	 *  - rtime isn't monotonic after all (a bug);
	 *  - we got reordered by the lock.
	 *
	 * In both cases this acts as a filter such that the rest of the code
	 * can assume it is monotonic regardless of anything else.
	 */
	if (prev->stime + prev->utime >= rtime)
		goto out;

	stime = curr->stime;
	utime = curr->utime;

	if (utime == 0) {
		stime = rtime;
		goto update;
	}

	if (stime == 0) {
		utime = rtime;
		goto update;
	}

	stime = scale_stime((__force u64)stime, (__force u64)rtime,
			    (__force u64)(stime + utime));

	/*
	 * Make sure stime doesn't go backwards; this preserves monotonicity
	 * for utime because rtime is monotonic.
	 *
	 *  utime_i+1 = rtime_i+1 - stime_i
	 *            = rtime_i+1 - (rtime_i - utime_i)
	 *            = (rtime_i+1 - rtime_i) + utime_i
	 *            >= utime_i
	 */
	if (stime < prev->stime)
		stime = prev->stime;
	utime = rtime - stime;

	/*
	 * Make sure utime doesn't go backwards; this still preserves
	 * monotonicity for stime, analogous argument to above.
	 */
	if (utime < prev->utime) {
		utime = prev->utime;
		stime = rtime - utime;
	}

update:
	prev->stime = stime;
	prev->utime = utime;
out:
	*ut = prev->utime;
	*st = prev->stime;
	raw_spin_unlock_irqrestore(&prev->lock, flags);
}

void task_cputime_adjusted(struct task_struct *p, cputime_t *ut, cputime_t *st)
{
	struct task_cputime cputime = {
		.sum_exec_runtime = tsk_seruntime(p),
	};

	task_cputime(p, &cputime.utime, &cputime.stime);
	cputime_adjust(&cputime, &p->prev_cputime, ut, st);
}
EXPORT_SYMBOL_GPL(task_cputime_adjusted);

/*
 * Must be called with siglock held.
 */
void thread_group_cputime_adjusted(struct task_struct *p, cputime_t *ut, cputime_t *st)
{
	struct task_cputime cputime;

	thread_group_cputime(p, &cputime);
	cputime_adjust(&cputime, &p->signal->prev_cputime, ut, st);
}
#endif

void init_idle_bootup_task(struct task_struct *idle)
{}

#ifdef CONFIG_SCHED_DEBUG
void proc_sched_show_task(struct task_struct *p, struct seq_file *m)
{}

void proc_sched_set_task(struct task_struct *p)
{}
#endif

#ifdef CONFIG_SMP
#define SCHED_LOAD_SHIFT	(10)
#define SCHED_LOAD_SCALE	(1L << SCHED_LOAD_SHIFT)

unsigned long default_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return SCHED_LOAD_SCALE;
}

unsigned long default_scale_smt_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = cpumask_weight(sched_domain_span(sd));
	unsigned long smt_gain = sd->smt_gain;

	smt_gain /= weight;

	return smt_gain;
}
#endif
