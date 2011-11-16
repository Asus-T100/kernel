#undef TRACE_SYSTEM
#define TRACE_SYSTEM wakelock

#if !defined(_TRACE_WAKELOCK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_WAKELOCK_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>
#include <linux/wakelock.h>

TRACE_EVENT(wake_lock,

	TP_PROTO(struct wake_lock *lock),

	TP_ARGS(lock),

	TP_STRUCT__entry(
	         __field( void *,    lock   )
	),

	TP_fast_assign(
	    __entry->lock  = lock;
	),

	TP_printk("timer=%p", __entry->lock)
);

TRACE_EVENT(wake_unlock,

	TP_PROTO(struct wake_lock *lock),

	TP_ARGS(lock),

	TP_STRUCT__entry(
	         __field( void *,    lock   )
	),

	TP_fast_assign(
	    __entry->lock  = lock;
	),

	TP_printk("timer=%p", __entry->lock)
);

#endif /* _TRACE_WAKELOCK_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
