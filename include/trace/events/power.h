#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#if !defined(_TRACE_POWER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_POWER_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>
#include <linux/device.h>

DECLARE_EVENT_CLASS(cpu,

	TP_PROTO(unsigned int state, unsigned int cpu_id),

	TP_ARGS(state, cpu_id),

	TP_STRUCT__entry(
		__field(	u32,		state		)
		__field(	u32,		cpu_id		)
	),

	TP_fast_assign(
		__entry->state = state;
		__entry->cpu_id = cpu_id;
	),

	TP_printk("state=%lu cpu_id=%lu", (unsigned long)__entry->state,
		  (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(cpu, cpu_idle,

	TP_PROTO(unsigned int state, unsigned int cpu_id),

	TP_ARGS(state, cpu_id)
);

/* This file can get included multiple times, TRACE_HEADER_MULTI_READ at top */
#ifndef _PWR_EVENT_AVOID_DOUBLE_DEFINING
#define _PWR_EVENT_AVOID_DOUBLE_DEFINING

#define PWR_EVENT_EXIT -1
#endif

DEFINE_EVENT(cpu, cpu_frequency,

	TP_PROTO(unsigned int frequency, unsigned int cpu_id),

	TP_ARGS(frequency, cpu_id)
);

TRACE_EVENT(machine_suspend,

	TP_PROTO(unsigned int state),

	TP_ARGS(state),

	TP_STRUCT__entry(
		__field(	u32,		state		)
	),

	TP_fast_assign(
		__entry->state = state;
	),

	TP_printk("state=%lu", (unsigned long)__entry->state)
);

/* This code will be removed after deprecation time exceeded (2.6.41) */
#ifdef CONFIG_EVENT_POWER_TRACING_DEPRECATED

/*
 * The power events are used for cpuidle & suspend (power_start, power_end)
 *  and for cpufreq (power_frequency)
 */
DECLARE_EVENT_CLASS(power,

	TP_PROTO(unsigned int type, unsigned int state, unsigned int cpu_id),

	TP_ARGS(type, state, cpu_id),

	TP_STRUCT__entry(
		__field(	u64,		type		)
		__field(	u64,		state		)
		__field(	u64,		cpu_id		)
	),

	TP_fast_assign(
		__entry->type = type;
		__entry->state = state;
		__entry->cpu_id = cpu_id;
	),

	TP_printk("type=%lu state=%lu cpu_id=%lu", (unsigned long)__entry->type,
		(unsigned long)__entry->state, (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(power, power_start,

	TP_PROTO(unsigned int type, unsigned int state, unsigned int cpu_id),

	TP_ARGS(type, state, cpu_id)
);

DEFINE_EVENT(power, power_frequency,

	TP_PROTO(unsigned int type, unsigned int state, unsigned int cpu_id),

	TP_ARGS(type, state, cpu_id)
);

TRACE_EVENT(power_end,

	TP_PROTO(unsigned int cpu_id),

	TP_ARGS(cpu_id),

	TP_STRUCT__entry(
		__field(	u64,		cpu_id		)
	),

	TP_fast_assign(
		__entry->cpu_id = cpu_id;
	),

	TP_printk("cpu_id=%lu", (unsigned long)__entry->cpu_id)

);

/* Deprecated dummy functions must be protected against multi-declartion */
#ifndef _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED
#define _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED

enum {
	POWER_NONE = 0,
	POWER_CSTATE = 1,
	POWER_PSTATE = 2,
};
#endif /* _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED */

#else /* CONFIG_EVENT_POWER_TRACING_DEPRECATED */

#ifndef _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED
#define _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED
enum {
       POWER_NONE = 0,
       POWER_CSTATE = 1,
       POWER_PSTATE = 2,
};

/* These dummy declaration have to be ripped out when the deprecated
   events get removed */
static inline void trace_power_start(u64 type, u64 state, u64 cpuid) {};
static inline void trace_power_end(u64 cpuid) {};
static inline void trace_power_frequency(u64 type, u64 state, u64 cpuid) {};
#endif /* _PWR_EVENT_AVOID_DOUBLE_DEFINING_DEPRECATED */

#endif /* CONFIG_EVENT_POWER_TRACING_DEPRECATED */

/*
 * The clock events are used for clock enable/disable and for
 *  clock rate change
 */
DECLARE_EVENT_CLASS(clock,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__field(        u64,            state           )
		__field(        u64,            cpu_id          )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->state = state;
		__entry->cpu_id = cpu_id;
	),

	TP_printk("%s state=%lu cpu_id=%lu", __get_str(name),
		(unsigned long)__entry->state, (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(clock, clock_enable,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

DEFINE_EVENT(clock, clock_disable,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

DEFINE_EVENT(clock, clock_set_rate,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

/*
 * The power domain events are used for power domains transitions
 */
DECLARE_EVENT_CLASS(power_domain,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__field(        u64,            state           )
		__field(        u64,            cpu_id          )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->state = state;
		__entry->cpu_id = cpu_id;
),

	TP_printk("%s state=%lu cpu_id=%lu", __get_str(name),
		(unsigned long)__entry->state, (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(power_domain, power_domain_target,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);
#ifdef CONFIG_PM_RUNTIME
#define rpm_status_name(status) { RPM_##status, #status }
#define show_rpm_status_name(val)				\
	__print_symbolic(val,					\
		rpm_status_name(SUSPENDED),			\
		rpm_status_name(SUSPENDING),			\
		rpm_status_name(RESUMING),			\
		rpm_status_name(ACTIVE)		                \
		)
TRACE_EVENT(runtime_pm_status,

	TP_PROTO(struct device *dev, int status),

	TP_ARGS(dev, status),

	TP_STRUCT__entry(
		__string(devname, dev_name(dev))
		__string(drivername, dev_driver_string(dev))
		__field(u32, status)
	),

	TP_fast_assign(
		__assign_str(devname, dev_name(dev));
		__assign_str(drivername, dev_driver_string(dev));
		__entry->status = status;
	),

	TP_printk("driver=%s dev=%s status=%s", __get_str(drivername),
		  __get_str(devname), show_rpm_status_name(__entry->status))
);
TRACE_EVENT(runtime_pm_usage,

	TP_PROTO(struct device *dev, int usage),

	TP_ARGS(dev, usage),

	TP_STRUCT__entry(
		__string(devname, dev_name(dev))
		__string(drivername, dev_driver_string(dev))
		__field(u32, usage)
	),

	TP_fast_assign(
		__assign_str(devname, dev_name(dev));
		__assign_str(drivername, dev_driver_string(dev));
		__entry->usage = (u32)usage;
	),

	TP_printk("driver=%s dev=%s usage=%d", __get_str(drivername),
		  __get_str(devname), __entry->usage)
);
#endif /* CONFIG_PM_RUNTIME */

#endif /* _TRACE_POWER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
