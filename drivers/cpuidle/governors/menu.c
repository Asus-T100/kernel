/*
 * menu.c - the menu idle governor
 *
 * Copyright (C) 2006-2007 Adam Belay <abelay@novell.com>
 * Copyright (C) 2009 Intel Corporation
 * Author:
 *        Arjan van de Ven <arjan@linux.intel.com>
 *
 * This code is licenced under the GPL version 2 as described
 * in the COPYING file that acompanies the Linux Kernel.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define BUCKETS 12
#define INTERVALS 8
#define RESOLUTION 1024
#define DECAY 8
#define MAX_INTERESTING 50000
#define STDDEV_THRESH 400

/* 60 * 60 > STDDEV_THRESH * INTERVALS = 400 * 8 */
#define MAX_DEVIATION 60

#ifdef CONFIG_PM_DEBUG
#define IDLE_HIST_BUCKET_WIDTH 10

struct idle_hist {
	u32 no_of_buckets;
	u32 max_range;
	u32 bucket_width;
	u32 *buckets;
	bool active;
};
static DEFINE_PER_CPU(struct idle_hist *, idle_hists);
#endif

static DEFINE_PER_CPU(struct hrtimer, menu_hrtimer);
static DEFINE_PER_CPU(int, hrtimer_status);
/* menu hrtimer mode */
enum {MENU_HRTIMER_STOP, MENU_HRTIMER_REPEAT};

/*
 * Concepts and ideas behind the menu governor
 *
 * For the menu governor, there are 3 decision factors for picking a C
 * state:
 * 1) Energy break even point
 * 2) Performance impact
 * 3) Latency tolerance (from pmqos infrastructure)
 * These these three factors are treated independently.
 *
 * Energy break even point
 * -----------------------
 * C state entry and exit have an energy cost, and a certain amount of time in
 * the  C state is required to actually break even on this cost. CPUIDLE
 * provides us this duration in the "target_residency" field. So all that we
 * need is a good prediction of how long we'll be idle. Like the traditional
 * menu governor, we start with the actual known "next timer event" time.
 *
 * Since there are other source of wakeups (interrupts for example) than
 * the next timer event, this estimation is rather optimistic. To get a
 * more realistic estimate, a correction factor is applied to the estimate,
 * that is based on historic behavior. For example, if in the past the actual
 * duration always was 50% of the next timer tick, the correction factor will
 * be 0.5.
 *
 * menu uses a running average for this correction factor, however it uses a
 * set of factors, not just a single factor. This stems from the realization
 * that the ratio is dependent on the order of magnitude of the expected
 * duration; if we expect 500 milliseconds of idle time the likelihood of
 * getting an interrupt very early is much higher than if we expect 50 micro
 * seconds of idle time. A second independent factor that has big impact on
 * the actual factor is if there is (disk) IO outstanding or not.
 * (as a special twist, we consider every sleep longer than 50 milliseconds
 * as perfect; there are no power gains for sleeping longer than this)
 *
 * For these two reasons we keep an array of 12 independent factors, that gets
 * indexed based on the magnitude of the expected duration as well as the
 * "is IO outstanding" property.
 *
 * Repeatable-interval-detector
 * ----------------------------
 * There are some cases where "next timer" is a completely unusable predictor:
 * Those cases where the interval is fixed, for example due to hardware
 * interrupt mitigation, but also due to fixed transfer rate devices such as
 * mice.
 * For this, we use a different predictor: We track the duration of the last 8
 * intervals and if the stand deviation of these 8 intervals is below a
 * threshold value, we use the average of these intervals as prediction.
 *
 * Limiting Performance Impact
 * ---------------------------
 * C states, especially those with large exit latencies, can have a real
 * noticeable impact on workloads, which is not acceptable for most sysadmins,
 * and in addition, less performance has a power price of its own.
 *
 * As a general rule of thumb, menu assumes that the following heuristic
 * holds:
 *     The busier the system, the less impact of C states is acceptable
 *
 * This rule-of-thumb is implemented using a performance-multiplier:
 * If the exit latency times the performance multiplier is longer than
 * the predicted duration, the C state is not considered a candidate
 * for selection due to a too high performance impact. So the higher
 * this multiplier is, the longer we need to be idle to pick a deep C
 * state, and thus the less likely a busy CPU will hit such a deep
 * C state.
 *
 * Two factors are used in determing this multiplier:
 * a value of 10 is added for each point of "per cpu load average" we have.
 * a value of 5 points is added for each process that is waiting for
 * IO on this CPU.
 * (these values are experimentally determined)
 *
 * The load average factor gives a longer term (few seconds) input to the
 * decision, while the iowait value gives a cpu local instantanious input.
 * The iowait factor may look low, but realize that this is also already
 * represented in the system load average.
 *
 */

struct menu_device {
	int		last_state_idx;
	int             needs_update;

	unsigned int	expected_us;
	u64		predicted_us;
	unsigned int	exit_us;
	unsigned int	bucket;
	u64		correction_factor[BUCKETS];
	u32		intervals[INTERVALS];
	int		interval_ptr;
};


#define LOAD_INT(x) ((x) >> FSHIFT)
#define LOAD_FRAC(x) LOAD_INT(((x) & (FIXED_1-1)) * 100)

/*
 * Define a variable per CPU in order to indicate when to
 * update the buckets or not. The buckets need to be updated
 * only when the wakeup is destinated to the CPU otherwise
 * consider a perfect prediction for the buckets.
 */
DEFINE_PER_CPU(int, update_buckets);

static int get_loadavg(void)
{
	unsigned long this = this_cpu_load();


	return LOAD_INT(this) * 10 + LOAD_FRAC(this) / 10;
}

static inline int which_bucket(unsigned int duration)
{
	int bucket = 0;

	/*
	 * We keep two groups of stats; one with no
	 * IO pending, one without.
	 * This allows us to calculate
	 * E(duration)|iowait
	 */
	if (nr_iowait_cpu(smp_processor_id()))
		bucket = BUCKETS/2;

	if (duration < 10)
		return bucket;
	if (duration < 100)
		return bucket + 1;
	if (duration < 1000)
		return bucket + 2;
	if (duration < 10000)
		return bucket + 3;
	if (duration < 100000)
		return bucket + 4;
	return bucket + 5;
}

/*
 * Return a multiplier for the exit latency that is intended
 * to take performance requirements into account.
 * The more performance critical we estimate the system
 * to be, the higher this multiplier, and thus the higher
 * the barrier to go to an expensive C state.
 */
static inline int performance_multiplier(void)
{
	int mult = 1;

	/* for higher loadavg, we are more reluctant */

	/*
	 * this doesn't work as intended - it is almost always 0, but can
	 * sometimes, depending on workload, spike very high into the hundreds
	 * even when the average cpu load is under 10%.
	 */
	/* mult += 2 * get_loadavg(); */

	/* for IO wait tasks (per cpu!) we add 5x each */
	mult += 10 * nr_iowait_cpu(smp_processor_id());

	return mult;
}

static DEFINE_PER_CPU(struct menu_device, menu_devices);

static void menu_update(struct cpuidle_driver *drv, struct cpuidle_device *dev);

/* This implements DIV_ROUND_CLOSEST but avoids 64 bit division */
static u64 div_round64(u64 dividend, u32 divisor)
{
	return div_u64(dividend + (divisor / 2), divisor);
}

/* Cancel the hrtimer if it is not triggered yet */
void menu_hrtimer_cancel(void)
{
	int cpu = smp_processor_id();
	struct hrtimer *hrtmr = &per_cpu(menu_hrtimer, cpu);

	/* The timer is still not time out*/
	if (per_cpu(hrtimer_status, cpu)) {
		hrtimer_cancel(hrtmr);
		per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_STOP;
	}
}
EXPORT_SYMBOL_GPL(menu_hrtimer_cancel);

/* Call back for hrtimer is triggered */
static enum hrtimer_restart menu_hrtimer_notify(struct hrtimer *hrtimer)
{
	int cpu = smp_processor_id();

	per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_STOP;

	return HRTIMER_NORESTART;
}

/*
 * Try detecting repeating patterns by keeping track of the last 8
 * intervals, and checking if the standard deviation of that set
 * of points is below a threshold. If it is... then use the
 * average of these 8 points as the estimated value.
 */
static u32 get_typical_interval(struct menu_device *data)
{
	int i = 0, divisor = 0;
	uint64_t max = 0, avg = 0, stddev = 0;
	int64_t thresh = LLONG_MAX; /* Discard outliers above this value. */
	unsigned int ret = 0;

again:

	/* first calculate average and standard deviation of the past */
	max = avg = divisor = stddev = 0;
	for (i = 0; i < INTERVALS; i++) {
		int64_t value = data->intervals[i];
		if (value <= thresh) {
			avg += value;
			divisor++;
			if (value > max)
				max = value;
		}
	}
	do_div(avg, divisor);

	for (i = 0; i < INTERVALS; i++) {
		int64_t value = data->intervals[i];
		if (value <= thresh) {
			int64_t diff = value - avg;
			stddev += diff * diff;
		}
	}
	do_div(stddev, divisor);
	stddev = int_sqrt(stddev);
	/*
	 * If we have outliers to the upside in our distribution, discard
	 * those by setting the threshold to exclude these outliers, then
	 * calculate the average and standard deviation again. Once we get
	 * down to the bottom 3/4 of our samples, stop excluding samples.
	 *
	 * This can deal with workloads that have long pauses interspersed
	 * with sporadic activity with a bunch of short pauses.
	 *
	 * The typical interval is obtained when standard deviation is small
	 * or standard deviation is small compared to the average interval.
	 */
	if (((avg > stddev * 6) && (divisor * 4 >= INTERVALS * 3))
							|| stddev <= 20) {
		data->predicted_us = avg;
		ret = 1;
		return ret;

	} else if ((divisor * 4) > INTERVALS * 3) {
		/* Exclude the max interval */
		thresh = max - 1;
		goto again;
	}

	return ret;
}

DEFINE_PER_CPU(u64, predicted_time);

/**
 * menu_select - selects the next idle state to enter
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static int menu_select(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	int i;
	int multiplier;
	struct timespec t;
	int repeat = 0, low_predicted = 0;
	int cpu = smp_processor_id();
	struct hrtimer *hrtmr = &per_cpu(menu_hrtimer, cpu);
#ifdef CONFIG_PM_DEBUG
	struct idle_hist *idle_hist = per_cpu(idle_hists, cpu);
#endif

	if (data->needs_update) {
		menu_update(drv, dev);
		data->needs_update = 0;
	}

	data->last_state_idx = 0;
	data->exit_us = 0;

	/* Special case when user has set very strict latency requirement */
	if (unlikely(latency_req == 0))
		return 0;

	/* determine the expected residency time, round up */
	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	data->expected_us =
		t.tv_sec * USEC_PER_SEC + t.tv_nsec / NSEC_PER_USEC;


	data->bucket = which_bucket(data->expected_us);

	multiplier = performance_multiplier();

	/*
	 * if the correction factor is 0 (eg first time init or cpu hotplug
	 * etc), we actually want to start out with a unity factor.
	 */
	if (data->correction_factor[data->bucket] == 0)
		data->correction_factor[data->bucket] = RESOLUTION * DECAY;

	/* Make sure to round up for half microseconds */
	data->predicted_us = div_round64(data->expected_us * data->correction_factor[data->bucket],
					 RESOLUTION * DECAY);

	per_cpu(predicted_time, cpu) = data->predicted_us;

#ifdef CONFIG_PM_DEBUG
	/* Collect the idleness histogram data if it is activated */
	if (idle_hist && idle_hist->active && idle_hist->buckets &&
						idle_hist->bucket_width) {
		u32 bucket = (u32)div_round64(data->predicted_us,
					idle_hist->bucket_width);

		/* Last bucket is used to collect the frequency of idleness
		 * longer than target_residency of deepest C-state.
		 */
		if (bucket > idle_hist->no_of_buckets)
			bucket = idle_hist->no_of_buckets;
		idle_hist->buckets[bucket] += 1;
	}
#endif

	repeat = get_typical_interval(data);

	/*
	 * We want to default to C1 (hlt), not to busy polling
	 * unless the timer is happening really really soon.
	 */
	if (data->expected_us > 5 &&
	    !drv->states[CPUIDLE_DRIVER_STATE_START].disabled &&
		dev->states_usage[CPUIDLE_DRIVER_STATE_START].disable == 0)
		data->last_state_idx = CPUIDLE_DRIVER_STATE_START;

	/*
	 * Find the idle state with the lowest power while satisfying
	 * our constraints.
	 */
	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		struct cpuidle_state *s = &drv->states[i];
		struct cpuidle_state_usage *su = &dev->states_usage[i];

		if (s->disabled || su->disable)
			continue;
		if (s->target_residency > data->predicted_us) {
			low_predicted = 1;
			continue;
		}
		if (s->exit_latency > latency_req)
			continue;
		if (s->exit_latency * multiplier > data->predicted_us)
			continue;

		data->last_state_idx = i;
		data->exit_us = s->exit_latency;
	}

	/* not deepest C-state chosen for low predicted residency */
	if (low_predicted) {
		unsigned int timer_us = 0;

		/*
		 * Set a timer to detect whether this sleep is much
		 * longer than repeat mode predicted.  If the timer
		 * triggers, the code will evaluate whether to put
		 * the CPU into a deeper C-state.
		 * The timer is cancelled on CPU wakeup.
		 */
		timer_us = 2 * (data->predicted_us + MAX_DEVIATION);

		if (repeat && (4 * timer_us < data->expected_us)) {
			RCU_NONIDLE(hrtimer_start(hrtmr,
				ns_to_ktime(1000 * timer_us),
				HRTIMER_MODE_REL_PINNED));
			/* In repeat case, menu hrtimer is started */
			per_cpu(hrtimer_status, cpu) = MENU_HRTIMER_REPEAT;
		}
	}

	return data->last_state_idx;
}

/**
 * menu_reflect - records that data structures need update
 * @dev: the CPU
 * @index: the index of actual entered state
 *
 * NOTE: it's important to be fast here because this operation will add to
 *       the overall exit latency.
 */
static void menu_reflect(struct cpuidle_device *dev, int index)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	data->last_state_idx = index;
	if (index >= 0)
		data->needs_update = 1;
}

/**
 * menu_update - attempts to guess what happened after entry
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static void menu_update(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct menu_device *data = &__get_cpu_var(menu_devices);
	int last_idx = data->last_state_idx;
	unsigned int last_idle_us = cpuidle_get_last_residency(dev);
	struct cpuidle_state *target = &drv->states[last_idx];
	unsigned int measured_us;
	u64 new_factor;

	/*
	 * Ugh, this idle state doesn't support residency measurements, so we
	 * are basically lost in the dark.  As a compromise, assume we slept
	 * for the whole expected time.
	 */
	if (unlikely(!(target->flags & CPUIDLE_FLAG_TIME_VALID)))
		last_idle_us = data->expected_us;


	measured_us = last_idle_us;

	/*
	 * We correct for the exit latency; we are assuming here that the
	 * exit latency happens after the event that we're interested in.
	 */
	if (measured_us > data->exit_us)
		measured_us -= data->exit_us;


	/* update our correction ratio */

	new_factor = data->correction_factor[data->bucket]
			* (DECAY - 1) / DECAY;

	/* if its a fake wakeup just consider it has perfect wakeup */
	if ((__get_cpu_var(update_buckets)) &&
		(data->expected_us > 0 && measured_us < MAX_INTERESTING))
		new_factor += RESOLUTION * measured_us / data->expected_us;
	else
		/*
		 * we were idle so long that we count it as a perfect
		 * prediction
		 */
		new_factor += RESOLUTION;

	/*
	 * We don't want 0 as factor; we always want at least
	 * a tiny bit of estimated time.
	 */
	if (new_factor == 0)
		new_factor = 1;

	data->correction_factor[data->bucket] = new_factor;

	/* update the repeating-pattern data */
	if (__get_cpu_var(update_buckets))
		data->intervals[data->interval_ptr++] = last_idle_us;
	else
		data->intervals[data->interval_ptr++] = data->expected_us;

	if (data->interval_ptr >= INTERVALS)
		data->interval_ptr = 0;

	__get_cpu_var(update_buckets) = 1;
}

/**
 * menu_enable_device - scans a CPU's states and does setup
 * @drv: cpuidle driver
 * @dev: the CPU
 */
static int menu_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	struct menu_device *data = &per_cpu(menu_devices, dev->cpu);
	struct hrtimer *t = &per_cpu(menu_hrtimer, dev->cpu);
#ifdef CONFIG_PM_DEBUG
	struct idle_hist *idle_hist = kzalloc(sizeof(struct idle_hist),
							GFP_KERNEL);
#endif

	hrtimer_init(t, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	t->function = menu_hrtimer_notify;

	memset(data, 0, sizeof(struct menu_device));

#ifdef CONFIG_PM_DEBUG
	if (!idle_hist) {
		pr_warn("Failed to allocate memory for idle_hist\n");
		return 0;
	}

	/* Max range is the target_residency of deepest C-state */
	idle_hist->max_range =
		drv->states[drv->state_count - 1].target_residency;
	idle_hist->bucket_width = IDLE_HIST_BUCKET_WIDTH;
	idle_hist->no_of_buckets =
		idle_hist->max_range / idle_hist->bucket_width;
	idle_hist->buckets = kzalloc(
		(idle_hist->no_of_buckets + 1) * sizeof(u32), GFP_KERNEL);
	if (!idle_hist->buckets)
		pr_warn("Failed to allocate memory for idle_hist buckets\n");
	pr_info("cpu %d: max_range = %u, #buckets = %d, width = %d\n",
		dev->cpu, idle_hist->max_range, idle_hist->no_of_buckets,
		idle_hist->bucket_width);

	per_cpu(idle_hists, dev->cpu) = idle_hist;
#endif
	return 0;
}

static struct cpuidle_governor menu_governor = {
	.name =		"menu",
	.rating =	20,
	.enable =	menu_enable_device,
	.select =	menu_select,
	.reflect =	menu_reflect,
	.owner =	THIS_MODULE,
};

#ifdef CONFIG_PM_DEBUG
static int idle_hist_show(struct seq_file *s, void *unused)
{
	int cpu, i, max_no_of_buckets = 0;
	struct idle_hist *idle_hist;

	for_each_online_cpu(cpu) {
		idle_hist = per_cpu(idle_hists, cpu);
		if (idle_hist && idle_hist->no_of_buckets > max_no_of_buckets)
			max_no_of_buckets = idle_hist->no_of_buckets;
		seq_printf(s, "\tCPU%d", cpu);
	}

	if (unlikely(idle_hist == NULL))
		return 0;

	seq_puts(s, "\n");
	for (i = 0; i <= max_no_of_buckets; i++) {
		seq_printf(s, "%d", i * idle_hist->bucket_width);
		for_each_online_cpu(cpu) {
			u32 freq = -1; /* -1 indicates invalid frquency */
			idle_hist = per_cpu(idle_hists, cpu);
			if (idle_hist && idle_hist->buckets &&
				i <= idle_hist->no_of_buckets)
				freq = idle_hist->buckets[i];
			seq_printf(s, "\t%d", freq);
		}
		seq_puts(s, "\n");
	}
	return 0;
}

static ssize_t idle_hist_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	struct idle_hist *idle_hist;
	int buf_size = min(count, sizeof(buf)-1);
	int bucket_width = 0, no_of_buckets;
	char *start_msg = "start";
	char *stop_msg = "stop";
	u32 *temp;
	int cpu;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (buf[0] != 's') {	/* If it is not 'start' or 'stop' most probably
				 * it is a update of bucket_width
				 */
		if (sscanf(buf, "%u", &bucket_width) != 1)
			return -EFAULT;

		if (bucket_width <= 0)
			return -EFAULT;

		/* update bucket width */
		for_each_online_cpu(cpu) {
			idle_hist = per_cpu(idle_hists, cpu);
			if (bucket_width == idle_hist->bucket_width)
					/* No need to update */
				continue;

			idle_hist->active = false;
			no_of_buckets =	idle_hist->max_range / bucket_width;
			temp = krealloc(idle_hist->buckets,
				(no_of_buckets + 1) * sizeof(u32), GFP_KERNEL);
			if (!temp) {
				pr_warn("Failed to update bucket_width\n");
				continue;
			}
			memset(temp, 0, (no_of_buckets + 1) * sizeof(u32));
			idle_hist->buckets = temp;
			idle_hist->bucket_width = bucket_width;
			idle_hist->no_of_buckets = no_of_buckets;
		}
	} else if (!strncmp(buf, start_msg, strlen(start_msg))) {
		/* start data collecting */
		for_each_online_cpu(cpu) {
			idle_hist = per_cpu(idle_hists, cpu);
			memset(idle_hist->buckets, 0,
				(idle_hist->no_of_buckets + 1) * sizeof(u32));
			idle_hist->active = true;
		}
	} else if (!strncmp(buf, stop_msg, strlen(stop_msg))) {
		/* stop data collecting */
		for_each_online_cpu(cpu) {
			idle_hist = per_cpu(idle_hists, cpu);
			idle_hist->active = false;
		}
	}

	return count;
}

static int idle_hist_open(struct inode *inode, struct file *file)
{
	return single_open(file, idle_hist_show, NULL);
}

static const struct file_operations idle_hist_ops = {
	.open           = idle_hist_open,
	.read           = seq_read,
	.write		= idle_hist_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

/**
 * init_menu - initializes the governor
 */
static int __init init_menu(void)
{
#ifdef CONFIG_PM_DEBUG
	struct dentry *d3 = debugfs_create_file("idle_hist", S_IFREG | S_IRUGO,
				NULL, NULL, &idle_hist_ops);
	if (!d3)
		pr_warn("idle_hist: Failed to create debugfs for idle_hist\n");
#endif
	return cpuidle_register_governor(&menu_governor);
}

/**
 * exit_menu - exits the governor
 */
static void __exit exit_menu(void)
{
	cpuidle_unregister_governor(&menu_governor);
}

MODULE_LICENSE("GPL");
module_init(init_menu);
module_exit(exit_menu);
