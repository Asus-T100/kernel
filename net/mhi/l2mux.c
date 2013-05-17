/*
 * File: l2mux.c
 *
 * Modem-Host Interface (MHI) L2MUX layer
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 * Author: Petri Mattila <petri.to.mattila@renesasmobile.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/if_mhi.h>
#include <linux/mhi.h>
#include <linux/l2mux.h>

#ifdef ACTIVATE_L2MUX_STAT
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#endif /* ACTIVATE_L2MUX_STAT */

#include <net/af_mhi.h>

#ifdef CONFIG_MHI_DEBUG
# define DPRINTK(...)    printk(KERN_DEBUG "MHI/L2MUX: " __VA_ARGS__)
#else
# define DPRINTK(...)
#endif


#ifdef ACTIVATE_L2MUX_STAT
#define MAX_COOKIE_LENGTH       PAGE_SIZE

/* MAX_COOKIE_LENGTH/sizeof(struct l2muxstat) */
#define MAX_DEBUG_MESSAGES      5000

#define list_l2mux_first_entry_safe(head, type, member) \
					(list_empty(head) ? NULL : \
					list_first_entry(head, type, member))
static DEFINE_RWLOCK(l2mux_stat_lock);

static struct l2mux_stat_info l2mux_sinf;

#endif

/* Handle ONLY Non DIX types 0x00-0xff */
#define ETH_NON_DIX_NPROTO   0x0100


/* L2MUX master lock */
static DEFINE_SPINLOCK(l2mux_lock);

/* L3 ID -> RX function table */
static l2mux_skb_fn *l2mux_id2rx_tab[MHI_L3_NPROTO] __read_mostly;

/* Packet Type -> TX function table */
static l2mux_skb_fn *l2mux_pt2tx_tab[ETH_NON_DIX_NPROTO] __read_mostly;


#ifdef ACTIVATE_L2MUX_STAT

static void l2mux_write_stat(unsigned l3pid, unsigned l3len,
				enum l2mux_direction dir,
				struct net_device *dev);
/*static int l2mux_read_stat(char *page,
						char **start,
						off_t off,
						int count,
						int *eof,
						void *data); */
static ssize_t store_l2mux_traces_state(struct device *dev,
					struct device_attribute *attr,
						const char *buf,
						size_t count);
static ssize_t show_l2mux_traces_state(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static struct device_attribute l2mux_dev_attrs[] = {
	__ATTR(l2mux_trace_status,
			S_IRUGO | S_IWUSR,
			show_l2mux_traces_state,
			store_l2mux_traces_state),
	__ATTR_NULL,
};

/**
 * l2mux_stat_dowork - work scheduled for L2mux stats node creation
 * @work: work scheduled
 */
void
l2mux_stat_dowork(struct work_struct *work)
{
	int err;
	struct l2mux_stat_info *info =
	container_of(work, struct l2mux_stat_info, l2mux_stat_work);

	struct net_device *dev = info->dev;

	if (l2mux_sinf.l2mux_traces_activation_done != 1) {

		err = device_create_file(&dev->dev, &l2mux_dev_attrs[0]);

		if (err == 0)
			l2mux_sinf.l2mux_traces_activation_done = 1;
		else
			printk(KERN_ERR "l2mux cannot create device file");
	}
}


/**
 * l2mux_write_stat - update the l2mux write statistic
 * @l3pid: protocol id to be stored
 * @length: length of the transfer
 * @dir: direction of the transfer
 * @dev: Device L2mux relies on
 */
static void
l2mux_write_stat(unsigned l3pid,
		unsigned l3len,
		enum l2mux_direction dir,
		struct net_device *dev)
{

	struct l2muxstat *tmp_stat;
	struct l2muxstat *old_stat;

	l2mux_sinf.l2mux_total_stat_counter++;

	if ((dev != NULL) && (l2mux_sinf.l2mux_traces_activation_done == 0)) {
		l2mux_sinf.dev = dev;
		schedule_work(&l2mux_sinf.l2mux_stat_work);
		return;

	} else {

		if ((ON == l2mux_sinf.l2mux_traces_state) ||
			(KERNEL == l2mux_sinf.l2mux_traces_state)) {

			if (write_trylock(&l2mux_stat_lock)) {

				tmp_stat = kmalloc(sizeof(struct l2muxstat),
							GFP_ATOMIC);
				if (NULL == tmp_stat) {
					write_unlock(&l2mux_stat_lock);
					return;
				}

				tmp_stat->l3pid = l3pid;
				tmp_stat->l3len = l3len;
				tmp_stat->dir = dir;
				do_gettimeofday(&(tmp_stat->time_val));
				tmp_stat->stat_counter =
					l2mux_sinf.l2mux_total_stat_counter;

				if (l2mux_sinf.l2mux_stat_id < 0)
					l2mux_sinf.l2mux_stat_id = 0;

				l2mux_sinf.l2mux_stat_id++;

				if (l2mux_sinf.l2mux_stat_id >=
						MAX_DEBUG_MESSAGES) {

					old_stat =
					list_l2mux_first_entry_safe(
						&l2mux_sinf.l2muxstat_tab.list,
						struct l2muxstat, list);
					if (old_stat != NULL) {
						list_del(&old_stat->list);
						kfree(old_stat);
						l2mux_sinf.l2mux_stat_id =
							MAX_DEBUG_MESSAGES;
					}
				}

				list_add_tail(&(tmp_stat->list),
					&(l2mux_sinf.l2muxstat_tab.list));

				write_unlock(&l2mux_stat_lock);
			}
		}
	}
	/*in the case lock is taken, information is missed*/
}


/**
 * l2mux_seq_start - start() standard method of L2mux stats node
 */
static void *
l2mux_seq_start(struct seq_file *seq, loff_t *pos)
{
	void *ret = NULL;

	if (l2mux_sinf.l2mux_traces_state == OFF) {
		printk(KERN_ERR "L2mux traces are off." \
			"activation -echo on > " \
			"/sys/class/net/my_modem_net_device/l2mux_trace_status" \
			" -sizeof(l2muxstat) = %d\n",
					sizeof(struct l2muxstat));
	} else {
		if (write_trylock(&l2mux_stat_lock)) {
			ret = list_l2mux_first_entry_safe(
					&l2mux_sinf.l2muxstat_tab.list,
					struct l2muxstat, list);
			write_unlock(&l2mux_stat_lock);
		}
	}

	return ret;
}

/**
 * l2mux_seq_next - next() standard method of L2mux stats node
 */
static void *
l2mux_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	return list_l2mux_first_entry_safe(&l2mux_sinf.l2muxstat_tab.list,
				struct l2muxstat, list);
}

/**
 * l2mux_seq_show - show() standard method of L2mux stats node
 */
static int
l2mux_seq_show(struct seq_file *seq, void *v)
{
	struct l2muxstat *tmp_stat = v;
	char temp_string[100];

	if (write_trylock(&l2mux_stat_lock)) {

		while (l2mux_sinf.previous_stat_counter !=
				(tmp_stat->stat_counter-1)) {

			sprintf(temp_string,
				"L2MHI_%d : missed : NA : NA : NA : NA\n",
				l2mux_sinf.previous_stat_counter+1);

			/* Interpret the iterator, 'v' */
			seq_printf(seq, temp_string);

			l2mux_sinf.previous_stat_counter++;
		}

		l2mux_sinf.previous_stat_counter = tmp_stat->stat_counter;

		sprintf(temp_string, "L2MHI_%d : %d : %d : %x : %d : %d\n",
					tmp_stat->stat_counter, tmp_stat->dir,
					tmp_stat->l3pid, tmp_stat->l3len,
				(unsigned int)tmp_stat->time_val.tv_sec,
				(unsigned int)tmp_stat->time_val.tv_usec);

		/* Interpret the iterator, 'v' */
		seq_printf(seq, temp_string);

		if (l2mux_sinf.l2mux_traces_state == KERNEL)
			printk(KERN_ERR "%s", temp_string);

		list_del(&tmp_stat->list);
		kfree(tmp_stat);
		tmp_stat = NULL;
		l2mux_sinf.l2mux_stat_id--;

		write_unlock(&l2mux_stat_lock);
	}

	return 0;
}

/**
 * l2mux_seq_stop - stop() standard method of L2mux stats node
 */
static void
l2mux_seq_stop(struct seq_file *seq, void *v)
{
  /* No cleanup needed */
}

/* Define iterator operations */
static const struct seq_operations l2mux_seq_ops = {
	.start = l2mux_seq_start,
	.next  = l2mux_seq_next,
	.stop  = l2mux_seq_stop,
	.show  = l2mux_seq_show,
};

/**
 * l2mux_seq_open - open() standard method of L2mux stats node
 */
static int
l2mux_seq_open(struct inode *inode, struct file *file)
{
	/* Register the operators */
	return seq_open(file, &l2mux_seq_ops);
}

static const struct file_operations l2mux_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = l2mux_seq_open, /* User supplied */
	.read    = seq_read,       /* Built-in helper function */
	.llseek  = seq_lseek,      /* Built-in helper function */
	.release = seq_release,    /* Built-in helper funciton */
};


/**
 * init_l2mux_stat - init the l2mux write statistic
 */
void init_l2mux_stat(void)
{
	l2mux_sinf.proc_entry = create_proc_entry("l2mux_mhi", 0644, NULL);

	if (l2mux_sinf.proc_entry == NULL)
		DPRINTK("cannot create proc file l2mux_mhi\n");
	else {

		l2mux_sinf.proc_entry->proc_fops = &l2mux_proc_fops;
		l2mux_sinf.l2mux_stat_id = 0;
		l2mux_sinf.previous_stat_counter = 0;
		l2mux_sinf.l2mux_total_stat_counter = 0;
		l2mux_sinf.l2mux_traces_state = OFF;
		l2mux_sinf.l2mux_traces_activation_done = 0;
		INIT_LIST_HEAD(&l2mux_sinf.l2muxstat_tab.list);
		INIT_WORK(&l2mux_sinf.l2mux_stat_work, l2mux_stat_dowork);
	}
}

/**
 * exit_l2mux_stat - exit the l2mux write statistic
 */
void exit_l2mux_stat(void)
{
	remove_proc_entry("l2mux_mhi", l2mux_sinf.proc_entry);
}

/**
 * store_l2mux_traces_state - store the l2mux traces status
 * @dev: Device to be created
 * @attr: attribute of sysfs
 * @buf: output stringwait
 */
static ssize_t
store_l2mux_traces_state(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int retval = count;

	if (sysfs_streq(buf, "on")) {
		l2mux_sinf.l2mux_traces_state = ON;
		printk(KERN_ERR "l2mux traces activated and available in proc fs\n");
	} else if (sysfs_streq(buf, "off")) {
		l2mux_sinf.l2mux_traces_state = OFF;
	} else if (sysfs_streq(buf, "kernel")) {
		l2mux_sinf.l2mux_traces_state = KERNEL;
	} else {
		retval = -EINVAL;
	}
	return retval;
}


/**
 * show_l2mux_traces_state - show l2mux traces state
 * @dev: Funnel device
 * @attr: attribute of sysfs
 * @buf: string written to sysfs file
 */
static ssize_t
show_l2mux_traces_state(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int retval = 0;
	char *temp_buf = buf;

	switch (l2mux_sinf.l2mux_traces_state) {
	case ON:
		return sprintf(temp_buf, "on\n");
	case OFF:
		return sprintf(temp_buf, "off\n");
	case KERNEL:
		return sprintf(temp_buf, "kernel\n");
	default:
		return -ENODEV;
	}

	return retval;
}
#endif /* ACTIVATE_L2MUX_STAT */


/**
 * l2mux_netif_rx_register - register RX function of L3 MHI protocol
 * @l3: L3 MHI protocol
 * @fn: registered RX function
 */
int
l2mux_netif_rx_register(int l3, l2mux_skb_fn *fn)
{
	int err = 0;

	DPRINTK("l2mux_netif_rx_register(l3:%d, fn:%p)\n", l3, fn);

	if (l3 < 0 || l3 >= MHI_L3_NPROTO)
		return -EINVAL;

	if (!fn)
		return -EINVAL;

	spin_lock(&l2mux_lock);
	{
		if (l2mux_id2rx_tab[l3] == NULL)
			l2mux_id2rx_tab[l3] = fn;
		else
			err = -EBUSY;
	}
	spin_unlock(&l2mux_lock);

	return err;
}
EXPORT_SYMBOL(l2mux_netif_rx_register);

/**
 * l2mux_netif_rx_unregister - Unregister RX function of L3 MHI protocol
 * @l3: L3 MHI protocol
 */
int
l2mux_netif_rx_unregister(int l3)
{
	int err = 0;

	DPRINTK("l2mux_netif_rx_unregister(l3:%d)\n", l3);

	if (l3 < 0 || l3 >= MHI_L3_NPROTO)
		return -EINVAL;

	spin_lock(&l2mux_lock);
	{
		if (l2mux_id2rx_tab[l3])
			l2mux_id2rx_tab[l3] = NULL;
		else
			err = -EPROTONOSUPPORT;
	}
	spin_unlock(&l2mux_lock);

	return err;
}
EXPORT_SYMBOL(l2mux_netif_rx_unregister);

/**
 * l2mux_netif_tx_register - register TX function of L3 MHI protocol
 * @pt: protocol id (= MAPID)
 * @fn: registered TX function
 */
int
l2mux_netif_tx_register(int pt, l2mux_skb_fn *fn)
{
	int err = 0;

	DPRINTK("l2mux_netif_tx_register(pt:%d, fn:%p)\n", pt, fn);

	if (pt <= 0 || pt >= ETH_NON_DIX_NPROTO)
		return -EINVAL;

	if (!fn)
		return -EINVAL;

	spin_lock(&l2mux_lock);
	{
		if (l2mux_pt2tx_tab[pt] == NULL)
			l2mux_pt2tx_tab[pt] = fn;
		else
			err = -EBUSY;
	}
	spin_unlock(&l2mux_lock);

	return err;
}
EXPORT_SYMBOL(l2mux_netif_tx_register);

/**
 * l2mux_netif_tx_unregister - Unregister TX function of L3 MHI protocol
 * @pt: protocol id (= MAPID)
 */
int
l2mux_netif_tx_unregister(int pt)
{
	int err = 0;

	DPRINTK("l2mux_netif_tx_unregister(pt:%d)\n", pt);

	if (pt <= 0 || pt >= ETH_NON_DIX_NPROTO)
		return -EINVAL;

	spin_lock(&l2mux_lock);
	{
		if (l2mux_pt2tx_tab[pt])
			l2mux_pt2tx_tab[pt] = NULL;
		else
			err = -EPROTONOSUPPORT;
	}
	spin_unlock(&l2mux_lock);

	return err;
}
EXPORT_SYMBOL(l2mux_netif_tx_unregister);

/**
 * l2mux_skb_rx - Format received skb
 * @skb: received skb
 * @dev: device on which skb is received
 */
int
l2mux_skb_rx(struct sk_buff *skb, struct net_device *dev)
{
	struct l2muxhdr	 *l2hdr;
	unsigned          l3pid;
	unsigned	  l3len;
	l2mux_skb_fn     *rxfn;

	/* Set the device in the skb */
	skb->dev = dev;

	/* Set MAC header here */
	skb_reset_mac_header(skb);

	/* L2MUX header */
	l2hdr = l2mux_hdr(skb);

	/* proto id and length in L2 header */
	l3pid = l2mux_get_proto(l2hdr);
	l3len = l2mux_get_length(l2hdr);

#ifdef ACTIVATE_L2MUX_STAT
	l2mux_write_stat(l3pid, l3len, DOWNLINK_DIR, dev);
#endif /* ACTIVATE_L2MUX_STAT */

#ifdef CONFIG_MHI_DUMP_FRAMES
	{
		u8 *ptr = skb->data;
		int len = skb_headlen(skb);
		int i;

		printk(KERN_DEBUG "L2MUX: RX dev:%d skb_len:%d l3_len:%d l3_pid:%d\n",
		       dev->ifindex, skb->len, l3len, l3pid);

		for (i = 0; i < len; i++) {
			if (i%8 == 0)
				printk(KERN_DEBUG "L2MUX: RX [%04X] ", i);
			printk(" 0x%02X", ptr[i]);
			if (i%8 == 7 || i == len-1)
				printk("\n");
		}
	}
#endif
	/* check that the advertised length is correct */
	if (l3len != skb->len - L2MUX_HDR_SIZE) {
		printk(KERN_WARNING "L2MUX: l2mux_skb_rx: L3_id:%d - skb length mismatch L3:%d (+4) <> SKB:%d",
		       l3pid, l3len, skb->len);
		goto drop;
	}

	/* get RX function */
	rxfn = l2mux_id2rx_tab[l3pid];

	/* Not registered */
	if (!rxfn)
		goto drop;


	/* Update RX statistics */
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb->len;


	/* Call the receiver function */
	return rxfn(skb, dev);

drop:
	dev->stats.rx_dropped++;
	kfree_skb(skb);
	return NET_RX_DROP;
}
EXPORT_SYMBOL(l2mux_skb_rx);

/**
 * l2mux_skb_tx - Format skb to be send
 * @skb: skb to be send
 * @dev: device on which skb is to be send
 */
int
l2mux_skb_tx(struct sk_buff *skb, struct net_device *dev)
{
	l2mux_skb_fn *txfn;
	unsigned      type;
	int err = 0;
#ifdef ACTIVATE_L2MUX_STAT
	struct l2muxhdr	 *l2hdr;
	unsigned          l3pid;
	unsigned	  l3len;
#endif /* ACTIVATE_L2MUX_STAT */

	/* Packet type ETH_P_XXX */
	if (!skb) {
		pr_err("invalud skb (NULL)!\n");
		return -EINVAL;
	}

	type = ntohs(skb->protocol);

#ifdef CONFIG_MHI_DUMP_FRAMES
	{
		u8 *ptr = skb->data;
		int len = skb_headlen(skb);
		int i;

		printk(KERN_DEBUG "L2MUX: TX dev:%d skb_len:%d ETH_P:%d\n",
		       dev->ifindex, skb->len, type);

		for (i = 0; i < len; i++) {
			if (i%8 == 0)
				printk(KERN_DEBUG "L2MUX: TX [%04X] ", i);
			printk(" 0x%02X", ptr[i]);
			if (i%8 == 7 || i == len-1)
				printk("\n");
		}
	}
#endif
	/* Only handling non DIX types */
	if (type <= 0 || type >= ETH_NON_DIX_NPROTO)
		return -EINVAL;

	/* TX function for this packet type */
	txfn = l2mux_pt2tx_tab[type];

	if (txfn)
		err = txfn(skb, dev);

#ifdef ACTIVATE_L2MUX_STAT

	if ((skb) && (0 == err)) {
		/* L2MUX header */
		l2hdr = l2mux_hdr(skb);
		/* proto id and length in L2 header */
		l3pid = l2mux_get_proto(l2hdr);
		l3len = l2mux_get_length(l2hdr);

		l2mux_write_stat(l3pid, l3len, UPLINK_DIR, dev);
	} else {
		printk(KERN_ERR "L2MUX TX skb invalid\n");
	}

#endif /* ACTIVATE_L2MUX_STAT */

	return err;
}
EXPORT_SYMBOL(l2mux_skb_tx);

/**
 * l2mux_init - L2MUX initialization
 */
static int __init l2mux_init(void)
{
	int i;

	DPRINTK("l2mux_init\n");

	for (i = 0; i < MHI_L3_NPROTO; i++)
		l2mux_id2rx_tab[i] = NULL;

	for (i = 0; i < ETH_NON_DIX_NPROTO; i++)
		l2mux_pt2tx_tab[i] = NULL;

#ifdef ACTIVATE_L2MUX_STAT
	init_l2mux_stat();

#endif /* ACTIVATE_L2MUX_STAT */


	return 0;
}

/**
 * l2mux_exit - L2MUX exit
 */
static void __exit l2mux_exit(void)
{
#ifdef ACTIVATE_L2MUX_STAT
	exit_l2mux_stat();
#endif /* ACTIVATE_L2MUX_STAT */
	DPRINTK("l2mux_exit\n");
}

module_init(l2mux_init);
module_exit(l2mux_exit);

MODULE_AUTHOR("RMC");
MODULE_DESCRIPTION("L2MUX for MHI Protocol Stack");
MODULE_LICENSE("GPL");

