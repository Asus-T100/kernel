/*
 * dlp_net.c
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)). This driver is implementing a 5-channel HSI
 * protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/dma-mapping.h>
#include <net/arp.h>

#include "dlp_main.h"

/* Defaut NET stack TX timeout delay (in milliseconds) */
#define DLP_NET_TX_DELAY		20000	/* 20 sec */

/*
 * struct dlp_net_context - NET channel private data
 *
 * @ndev: Registred network device
 * @net_padd: Padding buffer
 * @net_padd_dma: Padding buffer dma address
 */
struct dlp_net_context {
	struct net_device *ndev;

	/* Padding buffer */
	void *net_padd;
	dma_addr_t net_padd_dma;
};

/*
 *
 *
 */
struct dlp_net_tx_params {
	struct dlp_channel *ch_ctx;
	struct sk_buff *skb;
};

/*
 *
 * LOCAL functions
 *
 **/

static inline int dlp_net_is_trace_channel(struct dlp_channel *ch_ctx)
{
	/* The channel 4 can be used for modem Trace or NET IF */
	return (ch_ctx->hsi_channel == DLP_CHANNEL_NET3);
}

/**
 * Push as many RX PDUs  as possible to the controller FIFO
 *
 * @param ch_ctx : The channel context to consider
 *
 * @return 0 when OK, error value otherwise
 */
static int dlp_net_push_rx_pdus(struct dlp_channel *ch_ctx)
{
	int ret;
	struct dlp_xfer_ctx *rx_ctx = &ch_ctx->rx;

	ret = dlp_pop_recycled_push_ctrl(rx_ctx);
	if (ret == -EAGAIN) {
		mod_timer(&rx_ctx->timer, jiffies + rx_ctx->delay);
		ret = 0;
	}

	return ret;
}

/*
 *
 *
 */
static void dlp_net_modem_hangup(struct dlp_channel *ch_ctx, int reason)
{
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	ch_ctx->hangup.cause |= reason;

	/* Stop the NET IF */
	if (!netif_queue_stopped(net_ctx->ndev))
		netif_stop_queue(net_ctx->ndev);
}

/**
 *	dlp_net_mdm_coredump_cb	-	Modem has signaled a core dump
 *	@irq: interrupt number
 *	@dev: our device pointer
 *
 *	The modem has indicated a core dump.
 */
static void dlp_net_mdm_coredump_cb(struct dlp_channel *ch_ctx)
{
	pr_err(DRVNAME ": %s (Modem coredump)\n", __func__);
	dlp_net_modem_hangup(ch_ctx, DLP_MODEM_HU_COREDUMP);
}

/**
 *	dlp_net_mdm_reset_cb	-	Modem has changed reset state
 *	@data: channel pointer
 *
 *	The modem has either entered or left reset state. Check the GPIO
 *	line to see which.
 */
static void dlp_net_mdm_reset_cb(struct dlp_channel *ch_ctx)
{
	pr_err(DRVNAME ": %s (Modem reset)\n", __func__);
	dlp_net_modem_hangup(ch_ctx, DLP_MODEM_HU_RESET);
}

/**
 *	dlp_net_credits_available_cb -	TX credits are available
 *	@data: channel pointer
 */
static void dlp_net_credits_available_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	/* Restart the NET stack if it was stopped */
	if (netif_queue_stopped(net_ctx->ndev))
		netif_wake_queue(net_ctx->ndev);
}

/**
 *	dlp_net_type_trans
 *
 */
static __be16 dlp_net_type_trans(const char *buffer)
{
	if (!buffer)
		return htons(0);

	/* Look at IP version field */
	switch ((*buffer) >> 4) {
	case 4:
		return htons(ETH_P_IP);
	case 6:
		return htons(ETH_P_IPV6);
	default:
		pr_err(DRVNAME ": Invalid IP frame header (0x%x)\n",
				(*buffer) >> 4);

		/* Dump the invalid PDU data */
		print_hex_dump(KERN_DEBUG,
				DRVNAME": NET", DUMP_PREFIX_OFFSET,
				16, 4,
				buffer, 64, 0);
	}

	return htons(0);
}

/**
 * dlp_net_complete_tx - bottom-up flow for the TX side
 * @pdu: a reference to the completed pdu
 *
 * A TX transfer has completed: recycle the completed pdu and kick a new
 * delayed request or enter the IDLE state if nothing else is expected.
 */
static void dlp_net_complete_tx(struct hsi_msg *pdu)
{
	unsigned long flags;
	struct dlp_net_tx_params *msg_param = pdu->context;
	struct dlp_channel *ch_ctx = msg_param->ch_ctx;
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->tx;

	/* TX xfer done => Reset the "ongoing" flag */
	dlp_ctrl_set_reset_ongoing(0);

	/* TX done, free the skb */
	dev_kfree_skb(msg_param->skb);

	/* Update statistics */
	net_ctx->ndev->stats.tx_bytes += pdu->actual_len;
	net_ctx->ndev->stats.tx_packets++;

	/* Free the pdu */
	dlp_pdu_free(pdu, -1);

	/* Decrease the CTRL fifo size */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_hsi_controller_pop(xfer_ctx);

	/* Still have queued TX pdu ? */
	if (xfer_ctx->ctrl_len) {
		mod_timer(&ch_ctx->hangup.timer,
			  jiffies + usecs_to_jiffies(DLP_HANGUP_DELAY));
	} else {
		del_timer(&ch_ctx->hangup.timer);
	}

	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 */
static void dlp_net_complete_rx(struct hsi_msg *pdu)
{
	struct sk_buff *skb;
	struct dlp_xfer_ctx *xfer_ctx = pdu->context;
	struct dlp_channel *ch_ctx = xfer_ctx->channel;
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;
	unsigned int more_packets, data_size, offset, ret = 0;
	unsigned char *skb_data, *data_addr, *start_addr;
	unsigned int *ptr;
	unsigned long flags;

	/* Pop the CTRL queue */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_hsi_controller_pop(xfer_ctx);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* Check the received PDU header & seq_num */
	ret = dlp_pdu_header_check(xfer_ctx, pdu);
	if (ret == -EINVAL) {
		/* Dump the first 64 bytes */
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_NET", DUMP_PREFIX_OFFSET,
				16, 4,
				sg_virt(pdu->sgt.sgl), 64, 0);
		goto recycle;
	}

	/* Dump the RX data/length */
	if (EDLP_NET_RX_DATA_REPORT)
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_NET_RX", DUMP_PREFIX_OFFSET,
				16, 4,
				sg_virt(pdu->sgt.sgl), 64, 0);
	else if (EDLP_NET_RX_DATA_LEN_REPORT)
		pr_debug(DRVNAME ": NET_RX %d bytes\n", pdu->actual_len);

	/* Read packets desc  */
	/*---------------------*/
	ptr = sg_virt(pdu->sgt.sgl);
	start_addr = (unsigned char *)ptr;	/* Skip the header */

	do {
		/* Get the start offset */
		ptr++;
		offset = (*ptr);

		/* Get the size & address */
		ptr++;
		more_packets = (*ptr) & DLP_HDR_MORE_DESC;
		data_size = DLP_HDR_DATA_SIZE((*ptr)) - DLP_HDR_SPACE_AP;
		data_addr = start_addr + offset + DLP_HDR_SPACE_AP;

		/*
		 * The packet has been retrieved from the transmission
		 * medium. Build an skb around it, so upper layers can handle it
		 */
		skb = netdev_alloc_skb_ip_align(net_ctx->ndev, data_size);
		if (!skb) {
			pr_err(DRVNAME ": Out of memory (size: %d) => packet dropped\n",
					data_size);

			net_ctx->ndev->stats.rx_dropped++;
			goto recycle;
		}

		skb_data = skb_put(skb, data_size);
		memcpy(skb_data, data_addr, data_size);

		skb->dev = net_ctx->ndev;
		skb_reset_mac_header(skb);
		skb->protocol = dlp_net_type_trans(skb_data);
		skb->ip_summed = CHECKSUM_UNNECESSARY;	/* don't check it */

		/* Push received packet up to the IP networking stack */
		ret = netif_rx(skb);

		/* Update statistics */
		if (ret) {
			pr_warn(DRVNAME ": packet dropped\n");
			net_ctx->ndev->stats.rx_dropped++;

			/* Free the allocated skb */
			/* FIXME : to be freed ???? */
		} else {
			net_ctx->ndev->stats.rx_bytes += data_size;
			net_ctx->ndev->stats.rx_packets++;
		}
	} while (more_packets);

recycle:
	/* Recycle or free the pdu */
	dlp_pdu_recycle(xfer_ctx, pdu);
}

/**
 * dlp_net_tx_stop - update the TX state machine after expiration of the TX active
 *		 timeout further to a no outstanding TX transaction status
 * @param: a hidden reference to the TX context to consider
 *
 * This helper function updates the TX state if it is currently active and
 * inform the HSI pduwork and attached controller.
 */
void dlp_net_tx_stop(unsigned long param)
{
	struct dlp_xfer_ctx *xfer_ctx = (struct dlp_xfer_ctx *)param;

	dlp_stop_tx(xfer_ctx);
}

/**
 * dlp_net_hsi_tx_timeout_cb - Called when we have an HSI TX timeout
 * @ch_ctx : Channel context ref
 */
static void dlp_net_hsi_tx_timeout_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	/* Stop the NET IF */
	if (!netif_queue_stopped(net_ctx->ndev))
		netif_stop_queue(net_ctx->ndev);

	/* Need to reset the net link or not ? */
}

/*
 *
 * NETWORK INTERFACE functions
 *
 **/
int dlp_net_open(struct net_device *dev)
{
	int ret;
	struct dlp_channel *ch_ctx = netdev_priv(dev);

	/* Check & wait for modem readiness */
	if (!dlp_ctrl_modem_is_ready()) {
		pr_err(DRVNAME ": Unale to open NETIF%d (Modem NOT ready) !\n",
				ch_ctx->hsi_channel);
		ret = -EBUSY;
		goto out;
	}

	/* Is is a shared channel */
	if (dlp_net_is_trace_channel(ch_ctx)) {
		int state;

		/* Check if the the channel is not already opened for Trace */
		state = dlp_ctrl_get_channel_state(ch_ctx);
		if (state != DLP_CH_STATE_CLOSED) {
			pr_err(DRVNAME": Invalid channel state (%d)\n", state);
			ret = -EBUSY;
			goto out;
		}

		/* Allocate TX FIFO */
		ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->tx);
		if (ret) {
			pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
					ch_ctx->hsi_channel);
			goto out;
		}

		/* Allocate RX FIFO */
		ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->rx);
		if (ret) {
			pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
					ch_ctx->hsi_channel);
			goto out;
		}
	}

	/* Open the channel */
	ret = dlp_ctrl_open_channel(ch_ctx);
	if (ret) {
		pr_err(DRVNAME ": ch%d open failed !\n", ch_ctx->hsi_channel);
		ret = -EIO;
		goto out;
	}

	/* Push all RX pdus */
	ret = dlp_pop_recycled_push_ctrl(&ch_ctx->rx);

	/* Start the netif */
	netif_wake_queue(dev);

out:
	return ret;
}

int dlp_net_stop(struct net_device *dev)
{
	struct dlp_channel *ch_ctx = netdev_priv(dev);
	struct dlp_xfer_ctx *tx_ctx;
	struct dlp_xfer_ctx *rx_ctx;
	int ret;

	tx_ctx = &ch_ctx->tx;
	rx_ctx = &ch_ctx->rx;

	del_timer_sync(&ch_ctx->hangup.timer);

	/* Stop the NET IF */
	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);

	ret = dlp_ctrl_close_channel(ch_ctx);
	if (ret)
		pr_err(DRVNAME ": ch%d close failed !\n", ch_ctx->hsi_channel);

	/* RX */
	del_timer_sync(&rx_ctx->timer);
	dlp_stop_rx(rx_ctx, ch_ctx);

	/* TX */
	del_timer_sync(&tx_ctx->timer);
	dlp_stop_tx(tx_ctx);

	dlp_ctx_set_state(tx_ctx, IDLE);

	/* Flush the ACWAKE works */
	flush_work_sync(&ch_ctx->start_tx_w);
	flush_work_sync(&ch_ctx->stop_tx_w);

	return 0;
}

static void dlp_net_pdu_destructor(struct hsi_msg *pdu)
{
	if (pdu->ttype == HSI_MSG_WRITE)
		dlp_pdu_free(pdu, -1);
	else
		dlp_pdu_free(pdu, pdu->channel);
}

/*
 * Transmit a packet
 */
static int dlp_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dlp_channel *ch_ctx = netdev_priv(dev);
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;
	struct dlp_net_tx_params *msg_param;
	int i, ret, nb_padding, nb_entries, nb_packets;
	unsigned char *skb_data;
	unsigned int *ptr;
	unsigned int skb_len, padding_len, offset, desc_size, align_size;
	struct hsi_msg *new;
	struct scatterlist *sg;
	skb_frag_t *frag;

	if (!dlp_ctx_have_credits(&ch_ctx->tx, ch_ctx)) {
		/* Stop the NET if */
		netif_stop_queue(net_ctx->ndev);

		pr_warn(DRVNAME ": ch%d out of credits (%d)",
				ch_ctx->hsi_channel, ch_ctx->tx.seq_num);
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	if (skb->len < ETH_ZLEN) {
		/* WARNING("Padding received packet (size: %d)", skb->len); */
		if (skb_padto(skb, ETH_ZLEN))
			return NETDEV_TX_OK;
	}

	/* Workaround for the 4Bytes alignment issue */
	if ((unsigned int)(skb->data) & 3) {
		struct sk_buff *new_skb;
		new_skb = alloc_skb(NET_IP_ALIGN + skb->len + 4, GFP_ATOMIC);
		if (new_skb == NULL) {
			kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		skb_reserve(new_skb, 4);
		skb_reset_mac_header(new_skb);
		skb_set_network_header(new_skb,
				skb_network_header(skb) - skb->head);
		skb_set_transport_header(new_skb,
				skb_transport_header(skb) - skb->head);
		skb_copy_and_csum_dev(skb,
				skb_put(new_skb, skb->len));
		kfree_skb(skb);

		/* Update to the new skb */
		skb = new_skb;
	}

	/* Dump the TX data/length */
	if (EDLP_NET_TX_DATA_REPORT)
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_NET_TX", DUMP_PREFIX_OFFSET,
				16, 4,
				skb->data, skb->len, 0);
	else if (EDLP_NET_TX_DATA_LEN_REPORT)
		pr_debug(DRVNAME ": NET_TX %d bytes\n", skb->len);

	/* Set msg params */
	msg_param = (struct dlp_net_tx_params *)skb->cb;
	msg_param->ch_ctx = ch_ctx;
	msg_param->skb = skb;

	/* Save the timestamp */
	dev->trans_start = jiffies;

	/* Compute the number of needed padding entries */
	nb_padding = 1;
	if (skb_has_frag_list(skb)) {
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			align_size = IS_ALIGNED(skb_shinfo(skb)->frags[i].size,
						DLP_PACKET_ALIGN_CP);
			/* Size aligned ? */
			nb_padding += (align_size ? 0 : 1);
		}
	}

	/* Compute the number of needed SGT entries */
	nb_packets = skb_shinfo(skb)->nr_frags + 1;
	nb_entries = 1 +	/* Header */
	    nb_packets +	/* Packets */
	    nb_padding;		/* Padding */

	/* Allocate the HSI msg */
	new = hsi_alloc_msg(nb_entries, GFP_ATOMIC);
	if (!new) {
		pr_err(DRVNAME ": Out of memory (hsi_msg)\n");
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	new->cl = dlp_drv.client;
	new->channel = ch_ctx->hsi_channel;
	new->ttype = HSI_MSG_WRITE;
	new->context = msg_param;
	new->complete = dlp_net_complete_tx;
	new->destructor = dlp_net_pdu_destructor;

	/* Allocate the header buffer */
	sg = new->sgt.sgl;

	desc_size = 1 * 4 +	/* Signature + Seq_num */
	    nb_packets * 8;	/* Packets Offset+Size */

	desc_size = ALIGN(desc_size, DLP_PACKET_ALIGN_CP);
	desc_size += DLP_HDR_SPACE_CP;

	ptr = dlp_buffer_alloc(desc_size, &sg_dma_address(sg));
	if (!ptr) {
		pr_err(DRVNAME ": Out of memory (msg_desc)\n");
		goto free_msg;
	}

	/* Set the header buffer */
	/*-----------------------*/
	sg_set_buf(sg, ptr, desc_size);

	/* Write packets desc  */
	/*---------------------*/
	i = 0;
	offset = desc_size - DLP_HDR_SPACE_CP;

	do {
		if (nb_packets == 1) {
			skb_data = skb->data;
			skb_len = skb->len;
		} else {
			frag = &skb_shinfo(skb)->frags[i];

			skb_len = frag->size;
			skb_data = (void *)frag->page;
		}

		/* Set the start offset */
		ptr++;
		(*ptr) = offset;

		/* Set the size (skb_len + header_space) */
		ptr++;
		(*ptr) = (DLP_HDR_NO_MORE_DESC|DLP_HDR_COMPLETE_PACKET|skb_len);
		(*ptr) += DLP_HDR_SPACE_CP;

		/* Set the packet SG entry */
		sg = sg_next(sg);
		sg_set_buf(sg, skb_data, skb_len);
		sg->dma_address = dma_map_single(dlp_drv.controller,
						 skb_data,
						 skb_len, DMA_TO_DEVICE);

		/* Still have packets ? */
		i++;
		if (i < nb_packets) {
			/* Need padding ? */
			align_size = ALIGN(skb_len, DLP_PACKET_ALIGN_CP);
			if (align_size != skb_len) {
				skb_data = net_ctx->net_padd;
				skb_len = align_size - skb_len;

				sg = sg_next(sg);
				sg_set_buf(sg, skb_data, skb_len);

				sg->dma_address =
				    dma_map_single(dlp_drv.controller, skb_data,
						   skb_len, DMA_TO_DEVICE);
			}

			/* Update the offset value */
			offset += align_size + DLP_HDR_SPACE_CP;
		} else {
			/* Update the offset value */
			offset += skb_len + DLP_HDR_SPACE_CP;
		}
	} while (i < nb_packets);

	/* Write the padding entry (Check 4 bytes alignment) */
	/*---------------------------------------------------*/
	padding_len = ch_ctx->tx.pdu_size - offset;
	padding_len = (padding_len / 4) * 4;
	if (padding_len) {
		sg = sg_next(sg);
		sg_set_buf(sg, net_ctx->net_padd, padding_len);
		sg->dma_address = net_ctx->net_padd_dma;
	}

	ret = dlp_hsi_controller_push(&ch_ctx->tx, new);
	if (ret) {
		ret = NETDEV_TX_BUSY;
		goto free_msg;
	}

	ret = NETDEV_TX_OK;
	return ret;

free_msg:
	hsi_free_msg(new);

out:
	return ret;
}

/*
 * Deal with a transmit timeout.
 */
void dlp_net_tx_timeout(struct net_device *dev)
{
	struct dlp_channel *ch_ctx = netdev_priv(dev);
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	pr_warn(DRVNAME ": NET TX timeout at %d ms (latency: %d ms)\n",
	       jiffies_to_msecs(jiffies),
	       jiffies_to_msecs(jiffies - dev->trans_start));

	/* Update statistics */
	net_ctx->ndev->stats.tx_errors++;
}

/*
 *
 */
int dlp_net_change_mtu(struct net_device *dev, int new_mtu)
{
	int ret = -EPERM;
	return ret;
}

static const struct net_device_ops dlp_net_netdev_ops = {
	.ndo_open = dlp_net_open,
	.ndo_stop = dlp_net_stop,
	.ndo_start_xmit = dlp_net_start_xmit,
	.ndo_change_mtu = dlp_net_change_mtu,
	.ndo_tx_timeout = dlp_net_tx_timeout,
};

/*
 *
 * INIT function
 *
 **/
void dlp_net_dev_setup(struct net_device *dev)
{
	dev->netdev_ops = &dlp_net_netdev_ops;
	dev->watchdog_timeo = DLP_NET_TX_DELAY;

	/* fill in the other fields */
	/*  dev->features = NETIF_F_SG | NETIF_F_NO_CSUM; FIXME: wget is KO */

	dev->type = ARPHRD_NONE;
	dev->mtu = DLP_NET_TX_PDU_SIZE;	/* FIXME: check wget crash */
	dev->tx_queue_len = 10;
	dev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
}

/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

struct dlp_channel *dlp_net_ctx_create(unsigned int index, struct device *dev)
{
	struct hsi_client *client = to_hsi_client(dev);
	struct dlp_channel *ch_ctx;
	struct net_device *ndev;
	struct dlp_net_context *net_ctx;
	int ret;

	/* Allocate the net device */
	ndev = alloc_netdev(sizeof(struct dlp_channel),
			    CONFIG_HSI_DLP_NET_NAME "%d", dlp_net_dev_setup);

	if (!ndev) {
		pr_err(DRVNAME ": alloc_netdev() failed !\n");
		return NULL;
	}

	/* Allocate the context private data */
	net_ctx = kzalloc(sizeof(struct dlp_net_context), GFP_KERNEL);
	if (!net_ctx) {
		pr_err(DRVNAME ": Out of memory (net_ctx)\n");
		goto free_dev;
	}

	/* Allocate the padding buffer */
	net_ctx->net_padd = dlp_buffer_alloc(DLP_NET_TX_PDU_SIZE,
					     &net_ctx->net_padd_dma);

	if (!net_ctx->net_padd) {
		pr_err(DRVNAME ": Out of memory (padding)\n");
		ret = -ENOMEM;
		goto free_ctx;
	}

	/* Register the net device */
	ret = register_netdev(ndev);
	if (ret) {
		pr_err(DRVNAME ": register_netdev(%s) failed (%d)\n",
			 ndev->name, ret);
		goto free_buff;
	}

	net_ctx->ndev = ndev;
	ch_ctx = netdev_priv(ndev);

	ch_ctx->ch_data = net_ctx;
	ch_ctx->hsi_channel = index;
	ch_ctx->use_flow_ctrl = 1;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&ch_ctx->tx_empty_event);

	/* Hangup context */
	dlp_ctrl_hangup_ctx_init(ch_ctx, dlp_net_hsi_tx_timeout_cb);

	/* Register the Credits, Reset & Coredump CB */
	ch_ctx->modem_coredump_cb = dlp_net_mdm_coredump_cb;
	ch_ctx->modem_reset_cb = dlp_net_mdm_reset_cb;
	ch_ctx->credits_available_cb = dlp_net_credits_available_cb;
	ch_ctx->push_rx_pdus = dlp_net_push_rx_pdus;
	ch_ctx->dump_state = dlp_dump_channel_state;

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_NET_TX_PDU_SIZE, DLP_HSI_TX_DELAY,
			  DLP_HSI_TX_WAIT_FIFO, DLP_HSI_TX_CTRL_FIFO,
			  dlp_net_complete_tx, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_NET_RX_PDU_SIZE, DLP_HSI_RX_DELAY,
			  DLP_HSI_RX_WAIT_FIFO, DLP_HSI_RX_CTRL_FIFO,
			  dlp_net_complete_rx, HSI_MSG_READ);

	INIT_WORK(&ch_ctx->start_tx_w, dlp_do_start_tx);
	INIT_WORK(&ch_ctx->stop_tx_w, dlp_do_stop_tx);

	ch_ctx->tx.timer.function = dlp_net_tx_stop;

	if (!dlp_net_is_trace_channel(ch_ctx)) {
		/* Allocate TX FIFO */
		ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->tx);
		if (ret) {
			pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
					index);
			goto cleanup;
		}

		/* Allocate RX FIFO */
		ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->rx);
		if (ret) {
			pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
					index);
			goto cleanup;
		}
	}

	return ch_ctx;

free_buff:
	dlp_buffer_free(net_ctx->net_padd,
			net_ctx->net_padd_dma, DLP_NET_TX_PDU_SIZE);

free_ctx:
	kfree(net_ctx);

free_dev:
	free_netdev(ndev);
	pr_err(DRVNAME": Failed to create context for ch%d\n", index);
	return NULL;

cleanup:
	dlp_net_ctx_delete(ch_ctx);

	pr_err(DRVNAME": Failed to create context for ch%d\n", index);
	return NULL;
}

int dlp_net_ctx_delete(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	/* Clear the hangup context */
	dlp_ctrl_hangup_ctx_deinit(ch_ctx);

	/* Unregister the net device */
	unregister_netdev(net_ctx->ndev);

	/* Delete the xfers context */
	dlp_xfer_ctx_clear(&ch_ctx->rx);
	dlp_xfer_ctx_clear(&ch_ctx->tx);

	/* Free the padding buffer */
	dlp_buffer_free(net_ctx->net_padd,
			net_ctx->net_padd_dma, DLP_NET_TX_PDU_SIZE);

	/* Free the ch_ctx */
	free_netdev(net_ctx->ndev);
	return ret;
}
