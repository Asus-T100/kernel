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
 * LOCAL functions
 *
 **/

static inline int dlp_net_is_trace_channel(struct dlp_channel *ch_ctx)
{
	/* The channel 4 can be used for modem Trace or NET IF */
	return (ch_ctx->hsi_channel == DLP_CHANNEL_NET3);
}

static int dlp_net_alloc_xfer_pdus(struct dlp_channel *ch_ctx)
{
	int ret;

	/* Allocate TX FIFO */
	ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->tx);
	if (ret) {
		pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
				ch_ctx->ch_id);
		return ret;
	}

	/* Allocate RX FIFO */
	ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->rx);
	if (ret) {
		pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
				ch_ctx->ch_id);

		/* Release the allocated TX context PDUs */
		dlp_xfer_ctx_clear(&ch_ctx->tx);
		return ret;
	}

	return 0;
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
	struct dlp_xfer_ctx *xfer_ctx = pdu->context;
	struct dlp_channel *ch_ctx = xfer_ctx->channel;
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;
	int wakeup, avail, pending;

	/* Recycle or Free the pdu */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_pdu_delete(xfer_ctx, pdu, flags);

	/* Decrease the CTRL fifo size */
	dlp_hsi_controller_pop(xfer_ctx);

	/* Check the wait FIFO size */
	avail = (xfer_ctx->wait_len <= xfer_ctx->wait_max / 2);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* Start new waiting pdus (if any) */
	dlp_pop_wait_push_ctrl(xfer_ctx);

	/* Wake-up the NET if whenever the TX wait FIFO is half empty, and
	 * not before, to prevent too many wakeups */
	pending = netif_queue_stopped(net_ctx->ndev);
	wakeup = (pending && avail);
	if (wakeup)
		netif_wake_queue(net_ctx->ndev);
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

	/* Check the link readiness (TTY still opened) */
	if (!dlp_tty_is_link_valid()) {
		if ((EDLP_NET_RX_DATA_REPORT) ||
			(EDLP_NET_RX_DATA_LEN_REPORT))
			pr_debug(DRVNAME ": NET: CH%d RX PDU ignored (close:%d, Time out: %d)\n",
				ch_ctx->ch_id,
				dlp_drv.tty_closed, dlp_drv.tx_timeout);
		return;
	}

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
			pr_warn(DRVNAME ": IP Packet dropped\n");
			net_ctx->ndev->stats.rx_dropped++;
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
 * dlp_net_rx_stop - update the RX state machine after expiration of the RX active
 *		 timeout further to a no outstanding RX transaction status
 * @param: a hidden reference to the RX context to consider
 *
 * This helper function updates the RX state if it is currently active and
 * inform the HSI pduwork and attached controller.
 */
void dlp_net_rx_stop(unsigned long param)
{
	struct dlp_xfer_ctx *xfer_ctx = (struct dlp_xfer_ctx *)param;
	struct dlp_channel *ch_ctx = xfer_ctx->channel;

	dlp_stop_rx(xfer_ctx, ch_ctx);
}

/**
 * dlp_net_hsi_tx_timeout_cb - Called when we have an HSI TX timeout
 * @ch_ctx : Channel context ref
 */
static void dlp_net_hsi_tx_timeout_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	/* Stop the NET IF */
	net_ctx->ndev->trans_start = jiffies;
	netif_tx_disable(net_ctx->ndev);
}

/*
 *
 * NETWORK INTERFACE functions
 *
 **/
int dlp_net_open(struct net_device *dev)
{
	int ret, state;
	struct dlp_channel *ch_ctx = netdev_priv(dev);

	pr_debug(DRVNAME ": %s (CH%d) open requested (%s, %d)\n",
			dev->name, ch_ctx->ch_id,
			current->comm, current->tgid);

	/* Check if the the channel is not already opened for Trace */
	state = dlp_ctrl_get_channel_state(ch_ctx->hsi_channel);
	if (state != DLP_CH_STATE_CLOSED) {
		pr_err(DRVNAME ": Can't open CH%d (HSI CH%d) => invalid state: %d\n",
				ch_ctx->ch_id, ch_ctx->hsi_channel, state);
		return -EBUSY;
	}

	/* Update/Set the eDLP channel id */
	dlp_drv.channels_hsi[ch_ctx->hsi_channel].edlp_channel = ch_ctx->ch_id;

	if (dlp_net_is_trace_channel(ch_ctx)) {
		if (dlp_net_alloc_xfer_pdus(ch_ctx))
			return -ENOMEM;
	}

	/* Open the channel */
	ret = dlp_ctrl_open_channel(ch_ctx);
	if (ret) {
		pr_err(DRVNAME ": ch%d open failed !\n", ch_ctx->ch_id);
		return -EIO;
	}

	/* Push all RX pdus */
	ret = dlp_pop_recycled_push_ctrl(&ch_ctx->rx);

	/* Start the netif */
	netif_wake_queue(dev);
	return ret;
}

int dlp_net_stop(struct net_device *dev)
{
	struct dlp_channel *ch_ctx = netdev_priv(dev);
	struct dlp_xfer_ctx *tx_ctx;
	struct dlp_xfer_ctx *rx_ctx;
	int ret;

	pr_debug(DRVNAME ": %s (CH%d) close requested (%s, %d)\n",
			dev->name, ch_ctx->ch_id,
			current->comm, current->tgid);

	tx_ctx = &ch_ctx->tx;
	rx_ctx = &ch_ctx->rx;

	del_timer_sync(&dlp_drv.timer[ch_ctx->ch_id]);

	/* Stop the NET IF */
	dev->trans_start = jiffies;
	netif_tx_disable(dev);

	/* RX */
	del_timer_sync(&rx_ctx->timer);
	dlp_stop_rx(rx_ctx, ch_ctx);

	/* TX */
	del_timer_sync(&tx_ctx->timer);
	dlp_stop_tx(tx_ctx);

	ret = dlp_ctrl_close_channel(ch_ctx);
	if (ret)
		pr_err(DRVNAME ": Can't close CH%d (HSI CH%d) => err: %d\n",
				ch_ctx->ch_id, ch_ctx->hsi_channel, ret);

	dlp_ctx_set_state(tx_ctx, IDLE);

	/* Flush the ACWAKE works */
	cancel_work_sync(&ch_ctx->start_tx_w);
	cancel_work_sync(&ch_ctx->stop_tx_w);

	/* device closed => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_CLOSED);

	return 0;
}

/*
* @brief Check if the skb is 4B aligned
*
* @param skb : skb reference
*
* @return: if skb data is:
*	=> 4B aligned, return the same provided skb
*	=> NOT 4B aligned, retrun a new allocated & 4B data aligned skb
*/
static struct sk_buff *dlp_net_aligne_skb(struct sk_buff *skb)
{
	struct sk_buff *new_skb = skb;

	/* Workaround for the 4Bytes alignment issue */
	if (IS_ALIGNED((u32)(skb->data), 4))
		return new_skb;

	new_skb = alloc_skb(NET_IP_ALIGN + skb->len + 4, GFP_ATOMIC);
	if (!new_skb)
		goto out;

	skb_reserve(new_skb, 4);
	skb_reset_mac_header(new_skb);
	skb_set_network_header(new_skb, skb_network_header(skb) - skb->head);
	skb_set_transport_header(new_skb,
			skb_transport_header(skb) - skb->head);
	skb_copy_and_csum_dev(skb, skb_put(new_skb, skb->len));

out:
	kfree_skb(skb);
	return new_skb;
}

/*
* @brief Get an available TX PDU
*
* @param ch_ctx : Channel context reference
* @param xfer_ctx : TX xfer context reference
* @param data_len : data to transfer size
*
* @return PDU reference or NULL if no PDU available
*/
struct hsi_msg *dlp_net_get_xmit_pdu(struct dlp_channel *ch_ctx,
					struct dlp_xfer_ctx *xfer_ctx,
					int data_len)
{
	struct hsi_msg *pdu;
	unsigned long flags;
	unsigned int offset, size, pdu_size;
	u32 *ptr;

	offset = DLP_DEFAULT_DESC_OFFSET;

	read_lock_irqsave(&xfer_ctx->lock, flags);
	pdu = dlp_fifo_tail(&xfer_ctx->wait_pdus);
	if (!pdu) {
		read_unlock_irqrestore(&xfer_ctx->lock, flags);
		goto recycled;
	}

	pdu_size = xfer_ctx->pdu_size;

	/* Start Address & Size */
	ptr = sg_virt(pdu->sgt.sgl);
	ptr += 2;

	for (offset += DLP_HDR_SPACE_CP;
		(*ptr) & DLP_HDR_COMPLETE_PACKET && (offset < pdu_size);
		ptr += 2) {
		/* Read the size (mask the N/P fields) */
		size = DLP_HDR_DATA_SIZE(*ptr) - DLP_HDR_SPACE_AP;

		/* Update the offset */
		offset += ALIGN(size, DLP_PACKET_ALIGN_CP);
		offset += DLP_HDR_SPACE_AP;
	}
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* Still have enough space ? */
	size = ALIGN(data_len, DLP_PACKET_ALIGN_CP);
	if ((offset + size) <= xfer_ctx->pdu_size)
		return pdu;

recycled:
	pdu = dlp_fifo_recycled_pop(xfer_ctx);
	if (pdu) {
		dlp_fifo_wait_push(xfer_ctx, pdu);
		pdu->status = HSI_STATUS_PENDING;

		/* Need to be done only when new PDU */
		dlp_start_tx(xfer_ctx);
	}

	return pdu;
}

/*
* @brief Calculate the eDLP header desc size
*
* @param pdu: PDU to consider
*
* @return : Header desc size
*/
void dlp_net_edlp_header_size(struct hsi_msg *pdu,
			u32 **start_addr,
			u32 **data_addr,
			u32 *offset)
{
	int size, nb_items;
	struct scatterlist *sg = pdu->sgt.sgl;
	u32 *ptr = sg_virt(sg);

	(*offset)  = DLP_DEFAULT_DESC_OFFSET;

	/* Start Address */
	ptr++;
	(*start_addr) = ptr;

	/* Size */
	ptr++;

	for (nb_items = 0; (*ptr) & DLP_HDR_COMPLETE_PACKET; nb_items++) {
		/* Read the size (mask the N/P fields) */
		size = DLP_HDR_DATA_SIZE((*ptr)) - DLP_HDR_SPACE_AP;
		(*offset) += ALIGN(size, DLP_PACKET_ALIGN_CP);
		(*offset) += DLP_HDR_SPACE_AP;

		/* Go to the next packet desc */
		ptr += 2;
		(*start_addr) += 2;
	}

	if (nb_items) {
		/* Goto the last packet */
		ptr -= 2;

		/* Set the packet flag */
		(*ptr) |= DLP_HDR_MORE_DESC;
	}

	ptr = sg_virt(sg);
	(*data_addr) = (u32 *)(((char *)ptr) + (*offset) + DLP_HDR_SPACE_CP);
}

/*
 * Transmit a packet
 */
static int dlp_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dlp_channel *ch_ctx = netdev_priv(dev);
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->tx;
	struct hsi_msg *pdu;
	unsigned long flags;
	u32 *start_addr, *size_ptr, *data_ptr, offset;

	if (!dlp_ctx_have_credits(xfer_ctx, ch_ctx)) {
		/* Stop the NET if */
		netif_stop_queue(net_ctx->ndev);

		pr_warn(DRVNAME ": CH%d TX ignored (credits:%d, seq_num:%d, closed:%d, timeout:%d)",
			xfer_ctx->channel->ch_id,
			xfer_ctx->channel->credits,
			xfer_ctx->seq_num,
			dlp_drv.tty_closed, dlp_drv.tx_timeout);
		return NETDEV_TX_BUSY;
	}

	if (skb_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;

	skb = dlp_net_aligne_skb(skb);
	if (!skb)
		return NETDEV_TX_OK;

	pdu = dlp_net_get_xmit_pdu(ch_ctx, xfer_ctx, skb->len);
	if (!pdu) {
		/* Stop the NET if */
		netif_stop_queue(net_ctx->ndev);

		return NETDEV_TX_BUSY;
	}

	/* Dump the TX data/length */
	if (EDLP_NET_TX_DATA_REPORT)
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_NET_TX", DUMP_PREFIX_OFFSET,
				16, 4,
				skb->data, skb->len, 0);
	else if (EDLP_NET_TX_DATA_LEN_REPORT)
		pr_debug(DRVNAME ": NET_TX %d bytes\n", skb->len);

	/* Set the eDLP header  */
	/*-----------------------*/
	dlp_net_edlp_header_size(pdu, &start_addr, &data_ptr, &offset);
	if (!data_ptr) {
		/* Stop the NET if */
		netif_stop_queue(net_ctx->ndev);

		return NETDEV_TX_BUSY;
	}

	/* Set the start offset */
	(*start_addr) = offset;

	/* Set the size (skb->len + header_space) */
	size_ptr = ++start_addr;
	(*size_ptr) = (DLP_HDR_NO_MORE_DESC|DLP_HDR_COMPLETE_PACKET|skb->len);
	(*size_ptr) += DLP_HDR_SPACE_CP;

	/* Copy the data */
	memcpy(data_ptr, skb->data, skb->len);

	/* Update statistics */
	dev->stats.tx_bytes += skb->len;
	dev->stats.tx_packets++;

	/* Free the skb */
	dev_kfree_skb(skb);

	/* Save the timestamp */
	dev->trans_start = jiffies;

	if (pdu->status != HSI_STATUS_ERROR) {  /* still valid ? */
		pdu->status = HSI_STATUS_COMPLETED;
		if (dlp_ctx_get_state(xfer_ctx) != IDLE)
			dlp_pop_wait_push_ctrl(xfer_ctx);
	} else {
		/* ERROR frame have already been popped from the wait FIFO */
		pr_debug(DRVNAME ": ERROR: PDU alrady popped\n");
		write_lock_irqsave(&xfer_ctx->lock, flags);
		dlp_pdu_delete(xfer_ctx, pdu, flags);
		write_unlock_irqrestore(&xfer_ctx->lock, flags);
		return NETDEV_TX_BUSY;
	}

	/* Everythin is OK */
	return NETDEV_TX_OK;
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
	/* Should not exceed the PDU size */
	if (new_mtu > DLP_NET_TX_PDU_SIZE) {
		pr_err(DRVNAME ": Invalid MTU size (%d)\n", new_mtu);
		return -EINVAL;
	}

	dev->mtu = new_mtu;
	return 0;
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
	dev->type = ARPHRD_NONE;
	dev->mtu = ETH_DATA_LEN;
	dev->tx_queue_len = 10;
	dev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
}

/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

static int dlp_net_ctx_cleanup(struct dlp_channel *ch_ctx);

struct dlp_channel *dlp_net_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
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
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	ch_ctx->use_flow_ctrl = 1;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&ch_ctx->tx_empty_event);

	/* Hangup context */
	dlp_ctrl_hangup_ctx_init(ch_ctx, dlp_net_hsi_tx_timeout_cb);

	/* Register the Credits, Reset/Coredump, cleanup CBs */
	ch_ctx->credits_available_cb = dlp_net_credits_available_cb;
	ch_ctx->push_rx_pdus = dlp_net_push_rx_pdus;
	ch_ctx->dump_state = dlp_dump_channel_state;
	ch_ctx->cleanup = dlp_net_ctx_cleanup;

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
	ch_ctx->rx.timer.function = dlp_net_rx_stop;

	if (!dlp_net_is_trace_channel(ch_ctx)) {
		if (dlp_net_alloc_xfer_pdus(ch_ctx))
			goto cleanup;
	}

	return ch_ctx;

free_buff:
	dlp_buffer_free(net_ctx->net_padd,
			net_ctx->net_padd_dma, DLP_NET_TX_PDU_SIZE);

free_ctx:
	kfree(net_ctx);

free_dev:
	free_netdev(ndev);
	pr_err(DRVNAME": Failed to create context for ch%d\n", ch_id);
	return NULL;

cleanup:
	dlp_net_ctx_cleanup(ch_ctx);
	dlp_net_ctx_delete(ch_ctx);

	pr_err(DRVNAME": Failed to create context for ch%d\n", ch_id);
	return NULL;
}

/*
* @brief This function will:
*	- Delete TX/RX timer
*	- Flush RX/TX queues
*	- Free the allocated padding buffer
*
* @param ch_ctx: NET channel context
*
* @return 0 when sucess, error code otherwise
*/
static int dlp_net_ctx_cleanup(struct dlp_channel *ch_ctx)
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
	return ret;
}

/*
 * This function will release the allocated memory
 * done in the _ctx_create function
 */
int dlp_net_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_net_context *net_ctx = ch_ctx->ch_data;

	/* Free the ch_ctx */
	free_netdev(net_ctx->ndev);
	return 0;
}
