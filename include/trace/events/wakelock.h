#undef TRACE_SYSTEM
#define TRACE_SYSTEM wakelock

#if !defined(_TRACE_WAKELOCK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_WAKELOCK_H

#include <linux/tracepoint.h>
#include <linux/wakelock.h>

DECLARE_EVENT_CLASS(wake_lock_template,

	TP_PROTO(struct wake_lock *lock),

	TP_ARGS(lock),

	TP_STRUCT__entry(
		__string(name,		lock->name)
		__field(unsigned long,	expires)
		__field(int,		flags)
	),

	TP_fast_assign(
		__assign_str(name, lock->name);
		__entry->expires = lock->expires;
		__entry->flags = lock->flags;
	),

	TP_printk("name=%s, flags=%d, expires=%lu",
		__get_str(name),
		__entry->flags,
		__entry->expires)
);

DEFINE_EVENT(wake_lock_template, wake_lock,
		TP_PROTO(struct wake_lock *lock),
		TP_ARGS(lock));
DEFINE_EVENT(wake_lock_template, wake_unlock,
		TP_PROTO(struct wake_lock *lock),
		TP_ARGS(lock));

#endif /* _TRACE_WAKELOCK_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
