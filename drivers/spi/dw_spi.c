/*
 *
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-dw.h>

#include "dw_spi.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif


#define QUEUE_RUNNING	0
#define QUEUE_STOPPED	1

/* Slave spi_dev related */
struct chip_data {
	struct spi_device *spi_dev;
	u32 cr0;
	u32 cs;			/* chip select pin */
	u32 n_bytes;		/* current is a 1/2/4 byte op */
	u32 type;		/* SPI/SSP/MicroWire */

	u32 dma_width;
	u32 enable_dma;
	u32 bits_per_word;
	u32 clk_div;		/* baud rate divider */
	u32 speed_hz;		/* baud rate */
	void (*cs_control)(u32 command);
};

#ifdef CONFIG_DEBUG_FS
static int spi_show_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define SPI_REGS_BUFSIZE	1024
static ssize_t  spi_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct spi_dw *dws;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	dws = file->private_data;

	buf = kzalloc(SPI_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DW SPI registers:\n");
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"CTRL0: \t\t0x%08x\n", dw_readl(dws, ctrl0));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"CTRL1: \t\t0x%08x\n", dw_readl(dws, ctrl1));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SSIENR: \t0x%08x\n", dw_readl(dws, ssienr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SER: \t\t0x%08x\n", dw_readl(dws, ser));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"BAUDR: \t\t0x%08x\n", dw_readl(dws, baudr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"TXFTLR: \t0x%08x\n", dw_readl(dws, txfltr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"RXFTLR: \t0x%08x\n", dw_readl(dws, rxfltr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"TXFLR: \t\t0x%08x\n", dw_readl(dws, txflr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"RXFLR: \t\t0x%08x\n", dw_readl(dws, rxflr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"SR: \t\t0x%08x\n", dw_readl(dws, sr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"IMR: \t\t0x%08x\n", dw_readl(dws, imr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"ISR: \t\t0x%08x\n", dw_readl(dws, isr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMACR: \t\t0x%08x\n", dw_readl(dws, dmacr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMATDLR: \t0x%08x\n", dw_readl(dws, dmatdlr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"DMARDLR: \t0x%08x\n", dw_readl(dws, dmardlr));
	len += snprintf(buf + len, SPI_REGS_BUFSIZE - len,
			"=================================\n");

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations spi_dw_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= spi_show_regs_open,
	.read		= spi_show_regs,
	.llseek		= default_llseek,
};

static int spi_dw_debugfs_init(struct spi_dw *dws)
{
	dws->debugfs = debugfs_create_dir("spi_dw", NULL);
	if (!dws->debugfs)
		return -ENOMEM;

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
		dws->debugfs, (void *)dws, &spi_dw_regs_ops);
	return 0;
}

static void spi_dw_debugfs_remove(struct spi_dw *dws)
{
	if (dws->debugfs)
		debugfs_remove_recursive(dws->debugfs);
}

#else
static inline int spi_dw_debugfs_init(struct spi_dw *dws)
{
	return 0;
}

static inline void spi_dw_debugfs_remove(struct spi_dw *dws)
{
}
#endif /* CONFIG_DEBUG_FS */

static irqreturn_t spi_dw_irq(int irq, void *dev_id)
{
	struct spi_dw *dws = dev_id;
	u16  irq_mask = 0x3f;

	dws->xfer.irq_status = dw_readw(dws, isr) & irq_mask;

	if (!dws->xfer.irq_status)
		return IRQ_NONE;
	if (dws->xfer.irq_status &
			(SPI_INT_TXOI | SPI_INT_RXOI | SPI_INT_RXUI)) {
		dw_readw(dws, txoicr);
		dw_readw(dws, rxoicr);
		dw_readw(dws, rxuicr);
		dws->xfer.err = -EIO;
		spi_dw_disable(dws);
		complete(&dws->xfer.complete);
		return IRQ_HANDLED;
	}

	/* disable interrupts */
	spi_dw_mask_intr(dws, irq_mask);
	return IRQ_WAKE_THREAD;
}
struct spi_message *get_message(struct spi_dw *dws)
{
	struct spi_message *message = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dws->lock, flags);
	if (!list_empty(&dws->queue)) {
		message = list_entry(dws->queue.next,
				struct spi_message, queue);
		list_del_init(&message->queue);
	}
	spin_unlock_irqrestore(&dws->lock, flags);
	return message;
}
static inline u32 tx_max(struct spi_dw *dws)
{
	u32 tx_left, tx_room;

	tx_left = (dws->xfer.len - dws->xfer.sent) / dws->xfer.n_bytes;
	tx_room = (dws->fifo_len - dw_readw(dws, txflr));

	return min(tx_left, tx_room);
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct spi_dw *dws)
{
	u32 rx_left = (dws->xfer.len - dws->xfer.rcvd) / dws->xfer.n_bytes;
	return min(rx_left, (u32)dw_readw(dws, rxflr));
}

static int transfer_setup(struct spi_dw *dws, struct chip_data *controller,
			struct spi_transfer *transfer, struct spi_message *msg)
{
	int err = 0;
	u32 cr0 = controller->cr0;
	u32 clk_div;

	dws->xfer.tx_buf = transfer->tx_buf;
	dws->xfer.tx_dma = transfer->tx_dma;
	dws->xfer.rx_buf = transfer->rx_buf;
	dws->xfer.rx_dma = transfer->rx_dma;

	dws->xfer.len = transfer->len;
	dws->xfer.n_bytes = controller->n_bytes;
	dws->xfer.sent = 0;
	dws->xfer.rcvd = 0;
	dws->xfer.msg = msg;
	dws->xfer.err = 0;
	dws->xfer.irq_status = 0;
	INIT_COMPLETION(dws->xfer.complete);

	/* {tx, rx}_threshold should probably be a module param with
	 *  some reasonable default but these work for now.
	 */
	dws->xfer.tx_threshold = 10;
	dws->xfer.rx_threshold = dws->xfer.tx_threshold - 1;

	/* we need to make the decsion about the type of transfer more
	 *  inteligently but this works for now
	 */
	if (transfer->len > dws->fifo_len)
		dws->xfer.type = INT_XFER;
	else
		dws->xfer.type = PIO_XFER;

	if (msg->is_dma_mapped &&
		dws->dma_inited &&
		dws->dma_ops)
		dws->xfer.type = DMA_XFER;

	/* Setup the controller based on parameters in the transfer
	 * each transfer can set the bit_per_word and the speed_hz to
	 * change these values in the controller the controller MUST
	 * be disabled
	 */
	if (unlikely(!controller->clk_div)) {
		controller->clk_div = dws->max_freq / controller->speed_hz;
		controller->clk_div = (controller->clk_div + 1) & 0xfffe;
		spi_dw_set_clk(dws, controller->clk_div);
		err = 1;
	}

	if (transfer->speed_hz) {
		if (transfer->speed_hz != controller->speed_hz) {
			if (transfer->speed_hz > dws->max_freq) {
				err = -EIO;
				goto out;
			}

			clk_div = dws->max_freq / transfer->speed_hz;
			clk_div = (clk_div + 1) & 0xfffe;
			controller->clk_div = clk_div;
			controller->speed_hz = transfer->speed_hz;
			err = 1;
		}
	}

	if (transfer->bits_per_word) {
		if (transfer->bits_per_word != 8 &&
			transfer->bits_per_word != 16) {
			err = -EIO;
			goto out;
		}
		cr0 &= ~SPI_DFS_MASK;
		cr0 |= transfer->bits_per_word - 1;
		err = 1;
	} else {
		cr0 &= ~SPI_DFS_MASK;
		cr0 |= controller->spi_dev->bits_per_word - 1;
	}

	cr0 &= ~SPI_MODE_MASK;
	cr0 |= (controller->spi_dev->mode << SPI_MODE_OFFSET);
	controller->cr0 = cr0;

	if (err || dw_readw(dws, ctrl0) != cr0) {
		spi_dw_disable(dws);
		spi_dw_chip_sel(dws, controller->spi_dev->chip_select);
		dw_writew(dws, ctrl0, cr0);
		spi_dw_set_clk(dws, controller->clk_div);
		spi_dw_enable(dws);
		err = 0;
	}
out:
	return err;
}

static void tx_fifo_fill(struct spi_dw *dws)
{
	int room;
	u16 txw = 0;
	if (dws->xfer.sent < dws->xfer.len) {
		room = tx_max(dws);
		while (room--) {
			if (dws->xfer.tx_buf) {
				if (dws->xfer.n_bytes == 2)
					txw = *(u16 *)dws->xfer.tx_buf;
				else
					txw = *(u8 *)dws->xfer.tx_buf;
				dws->xfer.tx_buf += dws->xfer.n_bytes;
			}
			dw_writew(dws, dr, txw);
			dws->xfer.sent += dws->xfer.n_bytes;
		}
	}
}


static void rx_fifo_drain(struct spi_dw *dws)
{
	u16 rx_val;
	int avail;

	if (dws->xfer.rcvd < dws->xfer.len) {
		avail = rx_max(dws);
		while (avail--) {
			rx_val = dw_readw(dws, dr);
			if (dws->xfer.rx_buf) {
				if (dws->xfer.n_bytes == 2)
					*(u16 *)(dws->xfer.rx_buf) =
						(u16)rx_val;
				else
					*dws->xfer.rx_buf = (u8)rx_val;
				dws->xfer.rx_buf += dws->xfer.n_bytes;
			}
			dws->xfer.rcvd += dws->xfer.n_bytes;
		}
	}
}

static inline void do_pio(struct spi_dw *dws)
{
	while (dws->xfer.sent < dws->xfer.len ||
					dws->xfer.rcvd < dws->xfer.len) {
		tx_fifo_fill(dws);
		rx_fifo_drain(dws);
		cpu_relax();
	}
	complete(&dws->xfer.complete);
}

static irqreturn_t spi_dw_irq_thread_handler(int irq, void *dev_id)
{
	struct spi_dw *dws = dev_id;

	if (dws->xfer.irq_status & SPI_INT_TXEI) {
		rx_fifo_drain(dws);
		tx_fifo_fill(dws);
	}

	if (dws->xfer.irq_status & SPI_INT_RXFI) {
		tx_fifo_fill(dws);
		rx_fifo_drain(dws);
	}

	if (dws->xfer.len == dws->xfer.rcvd &&
		dws->xfer.len == dws->xfer.sent) {
		complete(&dws->xfer.complete);
		goto out;
	}

	spi_dw_umask_intr(dws, SPI_INT_ALL);
out:
	return IRQ_HANDLED;
}

static inline void do_int_xfer(struct spi_dw *dws)
{
	spi_dw_disable(dws);
	dw_writew(dws, txfltr, dws->xfer.tx_threshold);
	dw_writew(dws, rxfltr, dws->xfer.rx_threshold);
	spi_dw_enable(dws);
	dw_readw(dws, icr);
	tx_fifo_fill(dws);
	dw_writew(dws, imr, SPI_INT_ALL);
}

static inline int do_transfer(struct spi_dw *dws)
{
	switch (dws->xfer.type) {
	case PIO_XFER:
		do_pio(dws);
		break;
	case INT_XFER:
		do_int_xfer(dws);
		break;
	case DMA_XFER:
		dws->dma_ops->dma_transfer(dws);
		break;
	default:
		BUG();
	}

	wait_for_completion(&dws->xfer.complete);


	return dws->xfer.err;
}

static void drain_message_queue(struct spi_dw *dws)
{
	struct spi_message *message;

	message = get_message(dws);
	while (message) {
		message->status = -ESHUTDOWN;
		message->complete(message->context);
		message = get_message(dws);
	}
}

static void pump_messages(struct work_struct *work)
{
	struct spi_dw *dws =
		container_of(work, struct spi_dw, pump_messages);
	struct spi_transfer *transfer;
	struct spi_message *message;
	struct chip_data *controller;
	int err = 0;

	pm_runtime_get_sync(dws->parent_dev);
	message = get_message(dws);

	while (message && dws->run != QUEUE_STOPPED) {
		controller = spi_get_ctldata(message->spi);
		list_for_each_entry(transfer, &message->transfers,
				transfer_list){

			err = transfer_setup(dws, controller,
						transfer, message);
			if (err < 0) {
				dev_err(&dws->master->dev,
					"transfer_setup failed");
				dws->xfer.err = -EIO;
				break;
			}

			if (controller->cs_control)
				controller->cs_control(SPI_DW_ASSERT);

			err = do_transfer(dws);
			if (err < 0) {
				dev_err(&dws->master->dev,
					"do_transfer failed");
				break;
			}
			message->actual_length += dws->xfer.len;

			if (transfer->delay_usecs)
				udelay(transfer->delay_usecs);

			if (transfer->cs_change &&
				controller->cs_control)
				controller->cs_control(SPI_DW_DEASSERT);
		}

		if (controller->cs_control)
			controller->cs_control(SPI_DW_ASSERT);

		message->status = dws->xfer.err;
		message->complete(message->context);
		message =  get_message(dws);
	}
	if (dws->run == QUEUE_STOPPED)
		drain_message_queue(dws);
	pm_runtime_put_sync(dws->parent_dev);
}

/* spi_device use this to queue in their spi_msg */
static int spi_dw_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_dw *dws = spi_master_get_devdata(spi->master);
	unsigned long flags;


	spin_lock_irqsave(&dws->lock, flags);

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;

	list_add_tail(&msg->queue, &dws->queue);

	queue_work(dws->workqueue,
		&dws->pump_messages);

	spin_unlock_irqrestore(&dws->lock, flags);

	return 0;
}

/* This may be called twice for each spi dev */
static int spi_dw_setup(struct spi_device *spi)
{
	struct spi_dw_chip *chip_info = NULL;
	struct chip_data *chip;

	if (spi->bits_per_word != 8 && spi->bits_per_word != 16)
		return -EINVAL;

	if (!spi->max_speed_hz) {
		dev_err(&spi->dev, "No max speed HZ parameter\n");
		return -EINVAL;
	}

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
	}
	chip->spi_dev = spi;

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info)
		chip->cs_control = chip_info->cs_control;

	chip->bits_per_word = spi->bits_per_word;
	chip->n_bytes = chip->bits_per_word / 8;
	chip->dma_width = chip->bits_per_word / 8;

	chip->speed_hz = spi->max_speed_hz;

	/* Default SPI mode is SCPOL = 0, SCPH = 0 */
	chip->cr0 = (chip->bits_per_word - 1)
		| (spi->mode  << SPI_MODE_OFFSET);

	spi_set_ctldata(spi, chip);
	return 0;
}

static void spi_dw_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);
	kfree(chip);
}

static int __devinit init_queue(struct spi_dw *dws)
{
	INIT_LIST_HEAD(&dws->queue);
	spin_lock_init(&dws->lock);

	dws->run = QUEUE_STOPPED;

	INIT_WORK(&dws->pump_messages, pump_messages);
	dws->workqueue = create_singlethread_workqueue(
					dev_name(dws->master->dev.parent));
	if (dws->workqueue == NULL)
		return -EBUSY;
	dws->run = QUEUE_RUNNING;
	return 0;
}


int spi_dw_stop_queue(struct spi_dw *dws)
{
	unsigned long flags;
	int status = 0;

	spin_lock_irqsave(&dws->lock, flags);
	dws->run = QUEUE_STOPPED;

	if (!list_empty(&dws->queue))
		status = -EBUSY;
	spin_unlock_irqrestore(&dws->lock, flags);

	return status;
}
EXPORT_SYMBOL_GPL(spi_dw_stop_queue);

static int destroy_queue(struct spi_dw *dws)
{
	int status;

	status = spi_dw_stop_queue(dws);
	if (status != 0)
		return status;
	destroy_workqueue(dws->workqueue);
	return 0;
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init(struct spi_dw *dws)
{
	spi_dw_disable(dws);
	spi_dw_mask_intr(dws, SPI_INT_ALL);
	dw_readw(dws, icr);
	spi_dw_enable(dws);

	if (!dws->fifo_len) {
		WARN(!dws->fifo_len, "spi-dw fifo_len not set using default");
		dws->fifo_len = 16;
	}
}

int __devinit spi_dw_add_host(struct spi_dw *dws)
{
	struct spi_master *master;
	int ret;

	BUG_ON(dws == NULL);

	master = spi_alloc_master(dws->parent_dev, 0);
	if (!master) {
		ret = -ENOMEM;
		goto exit;
	}

	dws->master = master;

	init_completion(&dws->xfer.complete);
	dws->dma_inited = 0;
	/* Change to address of FIFO */
	dws->dma_addr = (dma_addr_t)(dws->paddr + 0x60);

	ret = request_threaded_irq(dws->irq, spi_dw_irq,
				spi_dw_irq_thread_handler,
				IRQF_SHARED, "spi_dw", dws);
	if (ret < 0) {
		dev_err(&master->dev, "can not get IRQ\n");
		goto err_free_master;
	}

	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bus_num = dws->bus_num;
	master->num_chipselect = dws->num_cs;
	master->cleanup = spi_dw_cleanup;
	master->setup = spi_dw_setup;
	master->transfer = spi_dw_transfer;

	/* Basic HW init */
	spi_hw_init(dws);

	if (dws->dma_ops && dws->dma_ops->dma_init) {
		ret = dws->dma_ops->dma_init(dws);
		if (ret) {
			dev_warn(&master->dev, "DMA init failed\n");
			dws->dma_inited = 0;
		}
	}

	/* Initial and start queue */
	ret = init_queue(dws);
	if (ret) {
		dev_err(&master->dev, "problem initializing queue\n");
		goto err_diable_hw;
	}

	spi_master_set_devdata(master, dws);
	ret = spi_register_master(master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_queue_alloc;
	}

	spi_dw_debugfs_init(dws);
	return 0;

err_queue_alloc:
	destroy_queue(dws);
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
err_diable_hw:
	spi_dw_disable(dws);
	free_irq(dws->irq, dws);
err_free_master:
	spi_master_put(master);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(spi_dw_add_host);

void __devexit spi_dw_remove_host(struct spi_dw *dws)
{
	int status = 0;

	if (!dws)
		return;
	spi_dw_debugfs_remove(dws);

	/* Remove the queue */
	status = destroy_queue(dws);
	if (status != 0)
		dev_err(&dws->master->dev, "spi_dw_remove: workqueue will not "
			"complete, message memory not freed\n");

	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	spi_dw_disable(dws);
	dw_readw(dws, icr);
	free_irq(dws->irq, dws);

	/* Disconnect from the SPI framework */
	spi_unregister_master(dws->master);
}
EXPORT_SYMBOL_GPL(spi_dw_remove_host);

int spi_dw_suspend_host(struct spi_dw *dws)
{
	int ret = 0;

	ret = spi_dw_stop_queue(dws);
	if (ret)
		return ret;
	spi_dw_disable(dws);
	return ret;
}
EXPORT_SYMBOL_GPL(spi_dw_suspend_host);

int spi_dw_resume_host(struct spi_dw *dws)
{
	spi_hw_init(dws);
	dws->run = QUEUE_RUNNING;
	return 0;
}
EXPORT_SYMBOL_GPL(spi_dw_resume_host);

MODULE_AUTHOR("Feng Tang <feng.tang@intel.com>");
MODULE_DESCRIPTION("Driver for DesignWare SPI controller core");
MODULE_LICENSE("GPL v2");
