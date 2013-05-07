/*
 * hsi_net_device.c
 *
 * Copyright (C) 2011 Renesas. All rights reserved.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/if_phonet.h>
#include <linux/if_mhi.h>
#include <linux/mhi.h>
#include <linux/l2mux.h>
#include <linux/phonet.h>
#include <linux/platform_device.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/hsi/hsi_logical.h>
#include <linux/hsi/hsi.h>
#include <net/phonet/pn_dev.h>

#ifndef PN_DEV_HOST
#define PN_DEV_HOST     0x00
#endif

#define SIOMODEMONOFF (SIOCDEVPRIVATE + 1)
#define SIOMODEMRSTOUTID (SIOCDEVPRIVATE + 2)

struct hsi_protocol *hsi_protocol_context;

static int hsi_net_device_xmit(struct sk_buff *skb,
					struct net_device *dev);
static int hsi_net_device_ioctl(struct net_device *dev,
				struct ifreq *ifr, int cmd);
static int hsi_net_device_set_mtu(struct net_device *dev, int new_mtu);
static int hsi_net_device_open(struct net_device *dev);
static int hsi_net_device_close(struct net_device *dev);
static u16 hsi_net_device_select_queue(struct net_device *dev,
					struct sk_buff *skb);

extern struct header_ops phonet_header_ops;

/* traces */
static ssize_t store_hsi_logical_traces_state(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t show_hsi_logical_traces_state(
	struct device *dev,
	struct device_attribute *attr,
	char *buf);

static struct device_attribute hsi_logical_dev_attrs[] = {
	__ATTR(traces,
		S_IRUGO|S_IWUSR,
		show_hsi_logical_traces_state,
		store_hsi_logical_traces_state),
	__ATTR_NULL,
};

enum hsi_logical_trace_states hsi_logical_trace_state;
static int traces_activation_done;
static int hsi_gpio_configured;

#ifdef HSI_LOGICAL_USE_DEBUG
#define DPRINTK(...)		{if (hsi_logical_trace_state == HSI_ALL) \
					printk(KERN_DEBUG __VA_ARGS__); }

#define DPRINTK_HIGH(...)	{if ((hsi_logical_trace_state == HSI_HIGH)\
				|| (hsi_logical_trace_state == HSI_ALL)) \
					printk(KERN_DEBUG __VA_ARGS__); }
#else
# define DPRINTK(...)
#endif

static int configure_gpios(struct hsi_protocol_client *hsi)
{
	int ret;

	if (1 == hsi_gpio_configured)
		return 0;

	ret = gpio_request(hsi->gpio_rst_out, "resetOUT");
	if (ret < 0) {
		printk(KERN_ERR "gpio_request failed for rst_out");
		return -1;
	}
	ret = gpio_direction_input(hsi->gpio_rst_out);
	if (ret < 0) {
		printk(KERN_ERR "gpio_direction_input failed for rst_out");
		goto err_rst_out;
	}
	ret = gpio_request(hsi->gpio_pwr_on, "powerON");
	if (ret < 0) {
		printk(KERN_ERR "gpio_request failed for pwr_on");
		goto err_rst_out;
	}
	ret = gpio_direction_output(hsi->gpio_pwr_on, 0);
	if (ret < 0) {
		printk(KERN_ERR "gpio_direction_output failed for pwr_on");
		goto err_pwr_on;
	}
	ret = gpio_export(hsi->gpio_rst_out, 0);
	if (ret < 0) {
		printk(KERN_ERR "gpio_export failed for rst_out");
		goto err_pwr_on;
	}
	pr_info("GPIOs configuration - pwr_on:%d, rst_out: %d\n",
		hsi->gpio_pwr_on,
		hsi->gpio_rst_out);
	hsi_gpio_configured = 1;

	return 0;

err_pwr_on:
	gpio_free(hsi->gpio_pwr_on);

err_rst_out:
	gpio_free(hsi->gpio_rst_out);
	return ret;
}

/**
 * show_hsi_logical_traces_state - show HSI logical traces state
 * @dev: Funnel device
 * @attr: attribute of sysfs
 * @buf: string written to sysfs file
 */
static ssize_t
show_hsi_logical_traces_state(
		struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int retval = 0;
	char *temp_buf = buf;

	switch (hsi_logical_trace_state) {

	case HSI_ALL:
		return sprintf(temp_buf, "all\n");
	case HSI_HIGH:
		return sprintf(temp_buf, "high\n");
	case HSI_HIGH_RX:
		return sprintf(temp_buf, "high_rx\n");
	case HSI_HIGH_TX:
		return sprintf(temp_buf, "high_tx\n");
	case HSI_OFF:
		return sprintf(temp_buf, "off\n");
	default:
		return -ENODEV;
	}

	return retval;
}

/**
 * store_hsi_logical_traces_state - store the HSI logical traces status
 * @dev: Device to be created
 * @attr: attribute of sysfs
 * @buf: output string
 */
static ssize_t
store_hsi_logical_traces_state(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int retval = count;

	if (sysfs_streq(buf, "all"))
		hsi_logical_trace_state = HSI_ALL;
	else if (sysfs_streq(buf, "high"))
		hsi_logical_trace_state = HSI_HIGH;
	else if (sysfs_streq(buf, "high_rx"))
		hsi_logical_trace_state = HSI_HIGH_RX;
	else if (sysfs_streq(buf, "high_tx"))
		hsi_logical_trace_state = HSI_HIGH_TX;
	else if (sysfs_streq(buf, "off"))
		hsi_logical_trace_state = HSI_OFF;
	else
		retval = -EINVAL;
	return retval;
}

static const struct net_device_ops hsi_net_device_ops = {
	.ndo_open           = hsi_net_device_open,
	.ndo_stop           = hsi_net_device_close,
	.ndo_select_queue   = hsi_net_device_select_queue,
	.ndo_start_xmit     = hsi_net_device_xmit,
	.ndo_do_ioctl       = hsi_net_device_ioctl,
	.ndo_change_mtu     = hsi_net_device_set_mtu,
};

static void hsi_net_device_setup(struct net_device *dev)
{
	dev->features        = NETIF_F_SG;
	dev->netdev_ops      = &hsi_net_device_ops;
	dev->destructor      = free_netdev;
	dev->type            = ARPHRD_MHI;
	dev->flags           = IFF_POINTOPOINT | IFF_NOARP;
	dev->mtu             = MHI_MAX_MTU;
	dev->hard_header_len = 4;
	dev->dev_addr[0]     = PN_MEDIA_MODEM_HOST_IF;
	dev->addr_len        = 1;
	dev->tx_queue_len    = 500;
}


u16 hsi_net_device_select_queue(struct net_device *dev, struct sk_buff *skb)
{
	u16 subqueue = 0;

	DPRINTK("hsi_net_device_select_queue proto: 0x%04X\n", skb->protocol);
	/* Subqueue 0 is used for medium priority HSI bank (HSI BANK id 1)
	Subqueue 1 is used for low priority HSI bank (HSI BANK id 2)

	To be noticed: - HSI BANK id 0 is used for control
			(L2 header, ACK/NACK exchange)
		- no high priority HSI Bank
		(reserved for audio, managed in hw) */

	if (skb->protocol == htons(ETH_P_PHONET)) {
		/* In phonet case, L2 header is not yet added.
			Anyway MAPID is mandatory MHI_L3_PHONET
				=> prio medium */
		subqueue = HSI_LOGICAL_QUEUE_PRIO_MEDIUM ;

	} else if (skb->protocol == htons(ETH_P_MHI)) {
		if ((MAPID_FROM_HEADER(*(u32 *)skb->data) ==
				MHI_L3_AUDIO)
		|| (MAPID_FROM_HEADER(*(u32 *)skb->data) ==
				MHI_L3_TEST_PRIO)
		|| (MAPID_FROM_HEADER(*(u32 *)skb->data) ==
				MHI_L3_HIGH_PRIO_TEST)) {

			/* High prio */
			subqueue = HSI_LOGICAL_QUEUE_PRIO_HIGH;

		} else if ((MAPID_FROM_HEADER(*(u32 *)skb->data) ==
				MHI_L3_LOW_PRIO_TEST)) {

			subqueue = HSI_LOGICAL_QUEUE_PRIO_LOW;

		} else	{

			/* Medim prio */
			subqueue = HSI_LOGICAL_QUEUE_PRIO_MEDIUM;
		}
	} else if ((skb->protocol == htons(ETH_P_MHDP))
		|| (MAPID_FROM_HEADER(*(u32 *)skb->data) ==
			MHI_L3_LOW_PRIO_TEST)) {
		/* Low prio */
		subqueue = HSI_LOGICAL_QUEUE_PRIO_LOW;
	} else {
		BUG();
	}

	return subqueue;
}

static int hsi_net_device_ioctl(struct net_device *dev,
				struct ifreq *ifr, int cmd)
{
	struct if_phonet_req *req = (struct if_phonet_req *)ifr;
	int value;

	switch (cmd) {
	case SIOCPNGAUTOCONF:

		req->ifr_phonet_autoconf.device = PN_DEV_HOST;

		phonet_route_add(dev, 0x60);
		phonet_route_add(dev, 0x44);
		phonet_route_add(dev, 0x64);

		break;

	case SIOMODEMONOFF:

		if (copy_from_user(&value, (int *)ifr->ifr_data,
			sizeof(unsigned int)))
			return -EFAULT;

		if (value > 0) {
			gpio_set_value(hsi_protocol_context->cl[0]->gpio_pwr_on,
					1);
		} else {
			gpio_set_value(hsi_protocol_context->cl[0]->gpio_pwr_on,
					0);
	}

		break;

	case SIOMODEMRSTOUTID:
		value = hsi_protocol_context->cl[0]->gpio_rst_out;
		if (copy_to_user((int *)ifr->ifr_data,
				&value,
				sizeof(int))) {
			return -EINVAL;

		}

		break;
	}

	return 0;
}

static int hsi_net_device_set_mtu(struct net_device *dev, int new_mtu)
{
	DPRINTK("hsi_net_device_set_mtu\n");

	if ((new_mtu < MHI_MIN_MTU) || (new_mtu > MHI_MAX_MTU))
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}


static int hsi_net_device_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct hsi_protocol *context = netdev_priv(dev);
	int err;

	DPRINTK("hsi_net_device_xmit proto 0x%04X on queue %d\n",
			skb->protocol,
			skb->queue_mapping);

	if (skb->len < PHONET_MIN_MTU) {
		DPRINTK("proto: %d, len; %d\n", skb->protocol, skb->len);
		goto drop;
	 }

	/* Pad to 32-bits */
	if ((skb->len & 3) && skb_pad(skb, 4 - (skb->len & 3)))
		goto drop;

	/* Stop this subqueue (to allow queuing by upper layer) */
	netif_stop_subqueue(dev, skb->queue_mapping);

	DPRINTK("HSI netdevice Subqueue %d is now stopped\n",
			skb->queue_mapping);

	/* Call l2mux registered function, according to skb->protocol */
	err = l2mux_skb_tx(skb, dev);
	if (unlikely(err))
		goto drop;

#ifdef HSI_LOGICAL_USE_DEBUG__
	{
		u32 *buf = (u32 *)skb->data;
		int i, j = 0;

		DPRINTK("TX Buffer content: ");
		for (i = 0; i < skb->len/4; i++) {
			DPRINTK(" 0x%08X", *buf++);

			/* just for display */
			if (++j%4 == 0)
				DPRINTK("\n                   ");
		}
		DPRINTK("\n");
	}
#endif

	/* Update TX statistics */
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	/* Send to HSI LOGICAL.*/
	err =  hsi_logical_send(context,
			skb,
			HSI_LOG_QUEUE_ID_TO_CL_ID(skb->queue_mapping));

	if (err < 0)
		goto drop;

	 return 0;

drop:
	dev->stats.tx_dropped++;
	dev_kfree_skb(skb);

	DPRINTK("\n hsi_net_device_xmit: dropped");

	return 0;
}


void hsi_net_device_receive(struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;

	DPRINTK("Entering hsi_net_device callback\n");

	BUG_ON(skb == NULL);

	if (unlikely(!netif_running(dev))) {
		DPRINTK("Drop RX packet (device not up)\n");
		dev->stats.rx_dropped++;
		dev_kfree_skb(skb);
		return;
	}
	if (unlikely(!pskb_may_pull(skb, PHONET_MIN_MTU))) {
		DPRINTK("Error drop RX packet (too short)\n");
		dev->stats.rx_errors++;
		dev->stats.rx_length_errors++;
		dev_kfree_skb(skb);
		return;
	}
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb->len;

#ifdef HSI_LOGICAL_USE_DEBUG__
	{
		int i = 0, j = 0;
		u32 *buf = (u32 *) skb->data;

	DPRINTK("RX Buffer content: ");
		for (i = 0; i < skb->len/4; i++) {
			DPRINTK(" 0x%08X", *buf++);

			/* just for display */
			if (++j%4 == 0)
				DPRINTK("\n                   ");
		}
		DPRINTK("\n");
	 }
#endif

	l2mux_skb_rx(skb, dev);

}

static int hsi_net_device_open(struct net_device *dev)
{
	struct hsi_protocol *context = netdev_priv(dev);
	int err;

	DPRINTK("hsi_net_device_open\n");

	if ((dev != NULL) && (traces_activation_done == 0)) {
		err = device_create_file(&dev->dev, &hsi_logical_dev_attrs[0]);

		if (err == 0)
			traces_activation_done = 1;
		else
			EPRINTK("hsi_net_device_open can't create dev file");
	}

	/* Set receive callback function */
	context->receive_fn = &hsi_net_device_receive;

	/* Configure HSI logical */
	return hsi_logical_open(context);
}

static int hsi_net_device_close(struct net_device *dev)
{
	struct hsi_protocol *context = netdev_priv(dev);

	DPRINTK("hsi_net_device_close\n");

	return hsi_logical_close(context);
}


static int hsi_client_probe(struct device *dev)
{
	struct hsi_protocol_client *hsi;
	struct hsi_client *cl = to_hsi_client(dev);
	struct hsi_mid_platform_data *pd;
	int err;

	DPRINTK("hsi_client_probe\n");

	if (!hsi_protocol_context) {
		DPRINTK("hsi_protocol_context" \
			"is NULL => net device has not be probed\n");
		return -EFAULT;
	}

	hsi = kzalloc(sizeof(*hsi), GFP_KERNEL);

	if (!hsi)
		return -ENOMEM;

	hsi_protocol_context->cl[hsi_protocol_context->nb_client] = hsi;
#ifdef HSI_USE_SEND_SCHEDULED
	INIT_WORK(&hsi->send_work, hsi_logical_send_work);

	hsi->send_wq = create_singlethread_workqueue("hsi-send_wq");
	if (!hsi->send_wq) {
		EPRINTK("%s, Unable to create send workqueue\n", __func__);
		return -ENOMEM;
	}

#endif /*#ifdef HSI_USE_SEND_SCHEDULED*/

#ifdef HSI_USE_RCV_SCHEDULED
	INIT_LIST_HEAD(&hsi->rcv_msgs);
	INIT_WORK(&hsi->rcv_work, hsi_logical_rcv_work);
	spin_lock_init(&hsi->rcv_msgs_lock);

	hsi->rcv_wq = create_singlethread_workqueue("hsi-rcv_wq");
	if (!hsi->rcv_wq) {
		EPRINTK("%s, Unable to create receive workqueue\n", __func__);
		return -ENOMEM;
	}
#endif /*#ifdef HSI_USE_RCV_SCHEDULED*/

	hsi->hsi_cl = cl;

	hsi_client_set_drvdata(cl, hsi);

	hsi_protocol_context->nb_client++;

	pd = dev->platform_data;
	hsi->gpio_pwr_on = pd->gpio_mdm_pwr_on;
	hsi->gpio_rst_out = pd->gpio_mdm_rst_out;
	err = configure_gpios(hsi);
	if (err < 0)
		return err;

	return 0;
}

static int  hsi_client_remove(struct device *dev)
{
	struct hsi_client *cl = to_hsi_client(dev);

	struct hsi_protocol_client *hsi_control_client = hsi_client_drvdata(cl);
	gpio_free(hsi_control_client->gpio_pwr_on);
	gpio_free(hsi_control_client->gpio_rst_out);

	kfree(hsi_client_drvdata(cl));
	hsi_client_set_drvdata(cl, NULL);

	return 0;
}

static struct hsi_client_driver hsi_client0 = {
	.driver = {
		.name = "client0",
			.owner = THIS_MODULE,
			.probe = hsi_client_probe,
			.remove = hsi_client_remove,
	},
};

static struct hsi_client_driver hsi_client1 = {
	.driver = {
		.name = "client1",
			.owner = THIS_MODULE,
			.probe = hsi_client_probe,
			.remove = hsi_client_remove,
	},
};

static struct hsi_client_driver hsi_client2 = {
	.driver = {
		.name = "client2",
			.owner = THIS_MODULE,
			.probe = hsi_client_probe,
			.remove = hsi_client_remove,
	},
};

static struct hsi_client_driver hsi_client3 = {
	.driver = {
		.name = "client3",
			.owner = THIS_MODULE,
			.probe = hsi_client_probe,
			.remove = hsi_client_remove,
	},
};


static int hsi_net_device_probe(struct platform_device *dev)
{
	struct hsi_protocol *context;

	struct net_device *ndev;
	int err;

	DPRINTK(" hsi_net_device_probe\n");

	/* Alloc multiqueues net device.
		There is one queue less than HSI client number
		due to HSI control client that has no associated queue */
	ndev = alloc_netdev_mq(sizeof(struct hsi_protocol),
			"mhi%d", hsi_net_device_setup, HSI_NB_CLIENT-1);
	if (!ndev) {
		pr_err("alloc_netdev_mq failed\n");
		return -ENOMEM;
	}

	/* Stop all queues.
		They will be waked up after configuration exchange */
	netif_tx_stop_all_queues(ndev);

	context = netdev_priv(ndev);

	context->netdev = ndev;

	hsi_protocol_context = context;


	if (!context->netdev) {
		dev_err(&dev->dev, "No memory for netdev\n");
		err = -ENOMEM;
		goto out1;
	}
	SET_NETDEV_DEV(context->netdev, &dev->dev);
	err = register_netdev(context->netdev);
	if (err < 0) {
		dev_err(&dev->dev, "Register netdev failed (%d)\n", err);
		free_netdev(context->netdev);
		goto out1;
	}

	hsi_register_client_driver(&hsi_client0);

	hsi_register_client_driver(&hsi_client1);

	hsi_register_client_driver(&hsi_client2);

	hsi_register_client_driver(&hsi_client3);

	return 0;

out1:
	unregister_netdev(context->netdev);

	return err;
}

static int hsi_net_device_remove(struct platform_device *dev)
{
	struct hsi_protocol *priv = netdev_priv(hsi_protocol_context->netdev);

	DPRINTK(" hsi_net_device_remove\n");

	hsi_unregister_client_driver(&hsi_client0);
	hsi_unregister_client_driver(&hsi_client1);
	hsi_unregister_client_driver(&hsi_client2);

	unregister_netdev(hsi_protocol_context->netdev);

	kfree(priv);
	hsi_protocol_context = NULL;

	return 0;
}


static struct platform_driver hsi_net_device_driver = {
	.driver = {
		.name = "hsi_net_device"
		},
	.probe    = hsi_net_device_probe,
	.remove    = hsi_net_device_remove,
};


static int __init hsi_net_device_init(void)
{
	DPRINTK("hsi_net_device_init\n");

	hsi_protocol_context = NULL;
	traces_activation_done = 0;
	hsi_gpio_configured = 0;
	hsi_logical_trace_state = HSI_OFF;
	platform_driver_register(&hsi_net_device_driver);

	return 0;
}

static void __exit hsi_net_device_exit(void)
{
	platform_driver_unregister(&hsi_net_device_driver);
}

module_init(hsi_net_device_init);
module_exit(hsi_net_device_exit);

MODULE_AUTHOR("RMC");
MODULE_LICENSE("GPL");
