#ifndef KCT_H_
#  define KCT_H_

#  include <linux/netlink.h>

#  ifndef MAX_SB_N
#    define MAX_SB_N 32
#  endif

#  ifndef MAX_EV_N
#    define MAX_EV_N 32
#  endif

#  define NETLINK_CRASHTOOL 27
#  define ATTCHMT_ALIGN 4U

/* Type of events supported by crashtool */
enum ct_ev_type {
	CT_EV_STAT,
	CT_EV_INFO,
	CT_EV_ERROR,
	CT_EV_CRASH,
	CT_EV_LAST
};

enum ct_attchmt_type {
	CT_ATTCHMT_DATA0,
	CT_ATTCHMT_DATA1,
	CT_ATTCHMT_DATA2,
	/* Always add new types after DATA2 */
	CT_ATTCHMT_BINARY,
};

struct ct_attchmt {
	__u32 size; /* sizeof(data) */
	enum ct_attchmt_type type;
	char data[];
} __aligned(4);

struct ct_event {
	char submitter_name[MAX_SB_N];
	char ev_name[MAX_EV_N];
	enum ct_ev_type type;
	__u64 timestamp;
	__u32 attchmt_size; /* sizeof(all_attachments inc. padding) */
	struct ct_attchmt attachments[];
}  __aligned(4);

enum kct_nlmsg_type {
	/* kernel -> userland */
	KCT_EVENT,
	/* userland -> kernel */
	KCT_SET_PID = 4200,
};

struct kct_packet {
	struct nlmsghdr nlh;
	struct ct_event event;
};

#  define ATTCHMT_ALIGNMENT	4

#  ifndef ALIGN
#    define __ALIGN_MASK(x, mask)    (((x) + (mask)) & ~(mask))
#    define __ALIGN(x, a)            __ALIGN_MASK(x, (typeof(x))(a) - 1)
#    define ALIGN(x, a)		     __ALIGN((x), (a))
#  endif /* !ALIGN */

#  define foreach_attchmt(Event, Attchmt)				\
	if ((Event)->attchmt_size)					\
		for ((Attchmt) = (Event)->attachments;			\
		     (Attchmt) < (typeof(Attchmt))(((char *)		\
				  (Event)->attachments) +               \
			(Event)->attchmt_size);                         \
		     (Attchmt) = (typeof(Attchmt))ALIGN(((size_t)(Attchmt)) \
						     + sizeof(*(Attchmt)) + \
			      (Attchmt)->size, ATTCHMT_ALIGNMENT))

#  ifdef __KERNEL__

/* Helper functions */
extern int kct_log_stat(const char *submitter_name,
			const char *ev_name,
			gfp_t flags) __weak;

/* Raw API */
extern struct ct_event *kct_alloc_event(const char *submitter_name,
					const char *ev_name,
					enum ct_ev_type ev_type,
					gfp_t flags) __weak;
extern int kct_add_attchmt(struct ct_event **ev,
			   enum ct_attchmt_type at_type,
			   unsigned int size,
			   char *data, gfp_t flags)  __weak;
extern void kct_free_event(struct ct_event *ev) __weak;
extern int kct_log_event(struct ct_event *ev, gfp_t flags) __weak;

#  endif /* !__KERNEL__ */

#endif /* !KCT_H_ */
