/*
 * mfd.c: driver for High Speed UART device of Intel Medfield platform
 *
 * Refer pxa.c, 8250.c and some other drivers in drivers/serial/
 *
 * (C) Copyright 2010 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

/* Notes:
 * 1. DMA channel allocation: 0/1 channel are assigned to port 0,
 *    2/3 chan to port 1, 4/5 chan to port 3. Even number chans
 *    are used for RX, odd chans for TX
 *
 * 2. In A0 stepping, UART will not support TX half empty flag
 *
 * 3. The RI/DSR/DCD/DTR are not pinned out, DCD & DSR are always
 *    asserted, only when the HW is reset the DDCD and DDSR will
 *    be triggered
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/slab.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial_mfd.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>
#include <linux/pm_runtime.h>
#include <asm/intel_mid_hsu.h>

#define  MFD_HSU_A0_STEPPING	1

#define chan_readl(chan, offset)	readl(chan->reg + offset)
#define chan_writel(chan, offset, val)	writel(val, chan->reg + offset)

#define mfd_readl(obj, offset)		readl(obj->reg + offset)
#define mfd_writel(obj, offset, val)	writel(val, obj->reg + offset)

#define HSU_DMA_TIMEOUT_CHECK_FREQ	(HZ/100)

struct hsu_dma_buffer {
	u8		*buf;
	dma_addr_t	dma_addr;
	u32		dma_size;
	u32		ofs;
};

struct hsu_dma_chan {
	u32	id;
	enum dma_data_direction	dirt;
	struct uart_hsu_port	*uport;
	void __iomem		*reg;
};

/* max queue before HSU pm active */
#define MAXQ	2048
#define CMD_WL	1
#define CMD_WB	2
#define CMD_TX	3
#define CMD_RX_STOP	4
#define CMD_TX_STOP	5

/* to record the pin wakeup states */
#define PM_WAKEUP	1

struct uart_hsu_port {
	struct uart_port        port;
	unsigned char           ier;
	unsigned char           lcr;
	unsigned char           mcr;
	unsigned int            lsr_break_flag;
	char			name[12];
	int			index;
	struct device		*dev;

	unsigned int		tx_addr;
	struct hsu_dma_chan	*txc;
	struct hsu_dma_chan	*rxc;
	struct hsu_dma_buffer	txbuf;
	struct hsu_dma_buffer	rxbuf;
	int			use_dma;	/* flag for DMA/PIO */
	int			running;
	int			dma_tx_on;
	int			dma_rx_on;
	char			reg_shadow[HSU_PORT_REG_LENGTH];
	struct circ_buf		qcirc;
	int			qbuf[MAXQ * 3]; /* cmd + offset + value */
	struct work_struct	qwork;
	spinlock_t		qlock;
	unsigned long		pm_flags;
	/* hsu_dma_rx_tasklet is used to displace specific treatment
	 * from dma_chan_irq IRQ handler into the tasklet callback,
	 * hence enabling low latency mode for hsu dma (forbidden
	 * in IRQ context) */
	struct tasklet_struct	hsu_dma_rx_tasklet;
	int			suspended;
};

/* Top level data structure of HSU */
struct hsu_port {
	void __iomem	*reg;
	unsigned long	paddr;
	unsigned long	iolen;
	u32		irq;
	struct uart_hsu_port	port[4];
	struct hsu_dma_chan	chans[10];

	struct dentry *debugfs;
};
/* Mainly for uart console use */
static struct uart_hsu_port *serial_hsu_ports[4];
static struct uart_driver serial_hsu_reg;
static int dma_dscr_size = 2048;
static int logic_idx = -1, share_idx = -1;

inline bool hsu_port_is_active(struct uart_hsu_port *up)
{
#ifdef CONFIG_PM_SLEEP
#ifdef CONFIG_PM_RUNTIME
	return (up->dev->power.runtime_status == RPM_ACTIVE);
#endif
#endif
	return true;
}

/* the caller must hold the up->qlock */
static void insert_q(struct uart_hsu_port *up, int cmd, int offset, int value)
{
	struct circ_buf *circ = &up->qcirc;
	int *buf;

	/* we can't call printk for error case, otherwise the printk flood
	 * will happen
	 */
	if (CIRC_SPACE(circ->head, circ->tail, MAXQ) < 1)
		return;
	buf = (int *)circ->buf + circ->head * 3;
	*buf++ = cmd;
	*buf++ = offset;
	*buf = value;
	circ->head++;
	circ->head &= (MAXQ - 1);
}

static int query_q(struct uart_hsu_port *up)
{
	struct circ_buf *circ = &up->qcirc;

	return CIRC_CNT(circ->head, circ->tail, MAXQ);
}

/* the caller must hold the up->qlock */
static int get_q(struct uart_hsu_port *up, int *cmd, int *offset, int *value)
{
	struct circ_buf *circ = &up->qcirc;
	int *buf;

	if (!CIRC_CNT(circ->head, circ->tail, MAXQ))
		return 0; /* no more data */
	buf = (int *)circ->buf + circ->tail * 3;
	*cmd = *buf++;
	*offset = *buf++;
	*value = *buf;
	circ->tail++;
	circ->tail &= (MAXQ - 1);
	return 1;
}
/* hsu_low_latency module parameter controls the activation
 * of low_latency setting for the high speed uarts 0, 1 and 2
 * it is a 3 bits bit-map
 * uart 0 low latency activation mask: 0x00000001
 * uart 1 low latency activation mask: 0x00000002
 * uart 2 low latency activation mask: 0x00000004
 * default is 0x00000001 (low latency for wl1283 device hsu
 * is required for BZ#583 resolution) */
static unsigned int hsu_low_latency = 1;
module_param(hsu_low_latency, uint, S_IRUGO);

/*
 * The runtime check is used to check whether the current chip
 * stepping is Penwell A0, if yes, enable the DMA RX timeout timer
 */

static inline void runtime_suspend_delay(struct uart_hsu_port *up)
{
	pm_runtime_get(up->dev);
	pm_runtime_put(up->dev);
}

static inline unsigned int serial_in_irq(struct uart_hsu_port *up, int offset)
{
	if (offset > UART_MSR) {
		offset <<= 2;
		return readl(up->port.membase + offset);
	} else
		return (unsigned int)readb(up->port.membase + offset);
}

static unsigned int serial_in(struct uart_hsu_port *up, int offset)
{
	unsigned long flags;
	unsigned int val;
	void *reg;

	pm_runtime_get(up->dev);
	spin_lock_irqsave(&up->qlock, flags);
	if (!hsu_port_is_active(up))
		reg = up->reg_shadow;
	else
		reg = up->port.membase;

	if (offset > UART_MSR) {
		offset <<= 2;
		val = readl(reg + offset);
	} else
		val = (unsigned int)readb(reg + offset);
	spin_unlock_irqrestore(&up->qlock, flags);
	pm_runtime_put(up->dev);
	return val;
}

static inline void serial_out_irq(struct uart_hsu_port *up,
		int offset, int value)
{
	if (offset > UART_MSR) {
		offset <<= 2;
		writel(value, up->port.membase + offset);
	} else
		writeb((unsigned char)(value & 0xff),
				up->port.membase + offset);
}

static void serial_out(struct uart_hsu_port *up, int offset, int value)
{
	unsigned long flags;
	int qflag = 0;

	pm_runtime_get(up->dev);
	spin_lock_irqsave(&up->qlock, flags);
	if (!hsu_port_is_active(up))
		qflag = 1;
	/* existing queue not finished yet, continue queue */
	if (CIRC_CNT(up->qcirc.head, up->qcirc.tail, MAXQ))
		qflag = 1;
	if (offset > UART_MSR) {
		offset <<= 2;
		if (qflag)
			insert_q(up, CMD_WL, offset, value);
		else
			writel(value, up->port.membase + offset);
	} else {
		unsigned char val = value & 0xff;
		if (qflag)
			insert_q(up, CMD_WB, offset, val);
		else
			writeb(val, up->port.membase + offset);
	}
	spin_unlock_irqrestore(&up->qlock, flags);
	if (qflag)
		schedule_work(&up->qwork);
	pm_runtime_put(up->dev);
}

#ifdef CONFIG_DEBUG_FS

#define HSU_REGS_BUFSIZE	1024

static int hsu_show_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t port_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct uart_hsu_port *up = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(HSU_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	pm_runtime_get_sync(up->dev);

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MFD HSU port[%d] regs:\n", up->index);

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"IER: \t\t0x%08x\n", serial_in(up, UART_IER));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"IIR: \t\t0x%08x\n", serial_in(up, UART_IIR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"LCR: \t\t0x%08x\n", serial_in(up, UART_LCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MCR: \t\t0x%08x\n", serial_in(up, UART_MCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"LSR: \t\t0x%08x\n", serial_in(up, UART_LSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MSR: \t\t0x%08x\n", serial_in(up, UART_MSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"FOR: \t\t0x%08x\n", serial_in(up, UART_FOR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"PS: \t\t0x%08x\n", serial_in(up, UART_PS));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MUL: \t\t0x%08x\n", serial_in(up, UART_MUL));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"DIV: \t\t0x%08x\n", serial_in(up, UART_DIV));

	pm_runtime_put(up->dev);

	if (len > HSU_REGS_BUFSIZE)
			len = HSU_REGS_BUFSIZE;

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static ssize_t dma_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct hsu_dma_chan *chan = file->private_data;
	char *buf;
	u32 len = 0;
	ssize_t ret;

	buf = kzalloc(HSU_REGS_BUFSIZE, GFP_KERNEL);
	if (!buf)
		return 0;

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MFD HSU DMA channel [%d] regs:\n", chan->id);

	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"=================================\n");
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"CR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_CR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"DCR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_DCR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"BSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_BSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"MOTSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_MOTSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D0SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D0TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D1SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D1TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D2SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D2TSR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0SAR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D3SAR));
	len += snprintf(buf + len, HSU_REGS_BUFSIZE - len,
			"D0TSR: \t\t0x%08x\n", chan_readl(chan, HSU_CH_D3TSR));

	if (len > HSU_REGS_BUFSIZE)
			len = HSU_REGS_BUFSIZE;

	ret =  simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations port_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= hsu_show_regs_open,
	.read		= port_show_regs,
	.llseek		= default_llseek,
};

static const struct file_operations dma_regs_ops = {
	.owner		= THIS_MODULE,
	.open		= hsu_show_regs_open,
	.read		= dma_show_regs,
	.llseek		= default_llseek,
};

static int hsu_debugfs_init(struct hsu_port *hsu)
{
	int i;
	char name[32];

	hsu->debugfs = debugfs_create_dir("hsu", NULL);
	if (!hsu->debugfs)
		return -ENOMEM;

	for (i = 0; i < 3; i++) {
		snprintf(name, sizeof(name), "port_%d_regs", i);
		debugfs_create_file(name, S_IFREG | S_IRUGO,
			hsu->debugfs, (void *)(&hsu->port[i]), &port_regs_ops);
	}

	for (i = 0; i < 6; i++) {
		snprintf(name, sizeof(name), "dma_chan_%d_regs", i);
		debugfs_create_file(name, S_IFREG | S_IRUGO,
			hsu->debugfs, (void *)&hsu->chans[i], &dma_regs_ops);
	}

	return 0;
}

static void hsu_debugfs_remove(struct hsu_port *hsu)
{
	if (hsu->debugfs)
		debugfs_remove_recursive(hsu->debugfs);
}

#else
static inline int hsu_debugfs_init(struct hsu_port *hsu)
{
	return 0;
}

static inline void hsu_debugfs_remove(struct hsu_port *hsu)
{
}
#endif /* CONFIG_DEBUG_FS */

static void serial_hsu_enable_ms(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

void hsu_dma_tx(struct uart_hsu_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	struct hsu_dma_buffer *dbuf = &up->txbuf;
	int count;

	/* test_and_set_bit may be better, but anyway it's in lock protected mode */
	if (up->dma_tx_on)
		return;

	if (dbuf->ofs) {
		u32 real = chan_readl(up->txc, HSU_CH_D0SAR) - up->tx_addr;

		/* we found in flow control case, TX irq came without sending
		 * all TX buffer
		 */
		if (real < dbuf->ofs)
			dbuf->ofs = real; /* adjust to real chars sent */

		/* Update the circ buf info */
		xmit->tail += dbuf->ofs;
		xmit->tail &= UART_XMIT_SIZE - 1;

		up->port.icount.tx += dbuf->ofs;
		dbuf->ofs = 0;
	}
	/* Disable the channel */
	chan_writel(up->txc, HSU_CH_CR, 0x0);

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&up->port)) {
		dma_sync_single_for_device(up->port.dev,
					   dbuf->dma_addr,
					   dbuf->dma_size,
					   DMA_TO_DEVICE);

		count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		dbuf->ofs = count;

		/* Reprogram the channel */
		up->tx_addr = dbuf->dma_addr + xmit->tail;
		chan_writel(up->txc, HSU_CH_D0SAR, up->tx_addr);
		chan_writel(up->txc, HSU_CH_D0TSR, count);

		/* Reenable the channel */
		chan_writel(up->txc, HSU_CH_DCR, 0x1
						 | (0x1 << 8)
						 | (0x1 << 16));
		up->dma_tx_on = 1;
		chan_writel(up->txc, HSU_CH_CR, 0x1);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
}

/* The buffer is already cache coherent */
void hsu_dma_start_rx_chan(struct uart_hsu_port *up,
			struct hsu_dma_buffer *dbuf)
{
	struct hsu_dma_chan *rxc = up->rxc;

	dbuf->ofs = 0;

	chan_writel(rxc, HSU_CH_BSR, 32);
	chan_writel(rxc, HSU_CH_MOTSR, 4);

	chan_writel(rxc, HSU_CH_D0SAR, dbuf->dma_addr);
	chan_writel(rxc, HSU_CH_D0TSR, dbuf->dma_size);
	chan_writel(rxc, HSU_CH_DCR, 0x1 | (0x1 << 8)
					 | (0x1 << 16)
					 | (0x1 << 24)	/* timeout bit, see HSU Errata 1 */
					 );
	chan_writel(rxc, HSU_CH_CR, 0x3);
	up->dma_rx_on = 1;
}

/* Protected by spin_lock_irqsave(port->lock) */
static void serial_hsu_start_tx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;

	if (up->use_dma) {
		pm_runtime_get(up->dev);
		spin_lock_irqsave(&up->qlock, flags);
		if (!hsu_port_is_active(up)) {
			insert_q(up, CMD_TX, 0, 0);
			spin_unlock_irqrestore(&up->qlock, flags);
			schedule_work(&up->qwork);
		} else {
			spin_unlock_irqrestore(&up->qlock, flags);
			hsu_dma_tx(up);
		}
		pm_runtime_put(up->dev);
	} else if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serial_hsu_stop_tx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	struct hsu_dma_chan *txc = up->txc;
	unsigned long flags;

	pm_runtime_get(up->dev);
	if (up->use_dma) {
		if (!hsu_port_is_active(up)) {
			spin_lock_irqsave(&up->qlock, flags);
			insert_q(up, CMD_TX_STOP, 0, 0);
			spin_unlock_irqrestore(&up->qlock, flags);
			schedule_work(&up->qwork);
		} else {
			chan_writel(txc, HSU_CH_CR, 0x0);
		}
		up->dma_tx_on = 0;
	} else if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
	pm_runtime_put(up->dev);
}

/* hsu_dma_rx_tasklet tasklet function callback,
 * contains code previously located in hsu_dma_rx
 * function */
static void hsu_dma_rx_tasklet(unsigned long data)
{
	struct uart_hsu_port *up = (struct uart_hsu_port *)data;
	struct hsu_dma_chan *chan = up->rxc;
	struct tty_struct *tty;
	unsigned char low_latency;

	/* Get a reference to tty to prevent its closing
	 * from elsewhere during our treatment
	 */
	tty = tty_port_tty_get(&up->port.state->port);

	if (tty) {
		low_latency = tty->low_latency;

		/* If function is called from tasklet context, pm_runtime
		 * needs to be notified. It is not necessary from IRQ
		 * context as it is already done by IRQ handler */
		if (low_latency)
			pm_runtime_get(up->dev);

		tty_flip_buffer_push(tty);

		/* Release reference */
		tty_kref_put(tty);

		chan_writel(chan, HSU_CH_CR, 0x3);

		/* If function is called from tasklet context, pm_runtime
		 * needs to be notified */
		if (low_latency)
			pm_runtime_put(up->dev);
	}
}
/* This is always called in spinlock protected mode, so
 * modify timeout timer is safe here */
void hsu_dma_rx(struct uart_hsu_port *up, u32 int_sts)
{
	struct hsu_dma_buffer *dbuf = &up->rxbuf;
	struct hsu_dma_chan *chan = up->rxc;
	struct uart_port *port = &up->port;
	struct tty_struct *tty;
	int count;

	tty = tty_port_tty_get(&up->port.state->port);
	if (!tty)
		return;

	/*
	 * First need to know how many is already transferred,
	 * then check if its a timeout DMA irq, and return
	 * the trail bytes out, push them up and reenable the
	 * channel
	 */

	/* Timeout IRQ, need wait some time, see Errata 2 */
	if (int_sts & 0xf00)
		udelay(2);

	/* Stop the channel */
	chan_writel(chan, HSU_CH_CR, 0x0);

	count = chan_readl(chan, HSU_CH_D0SAR) - dbuf->dma_addr;
	if (!count) {
		/* Restart the channel before we leave */
		chan_writel(chan, HSU_CH_CR, 0x3);
		tty_kref_put(tty);
		return;
	}

	dma_sync_single_for_cpu(port->dev, dbuf->dma_addr,
			dbuf->dma_size, DMA_FROM_DEVICE);

	/*
	 * Head will only wrap around when we recycle
	 * the DMA buffer, and when that happens, we
	 * explicitly set tail to 0. So head will
	 * always be greater than tail.
	 */
	tty_insert_flip_string(tty, dbuf->buf, count);
	port->icount.rx += count;

	dma_sync_single_for_device(up->port.dev, dbuf->dma_addr,
			dbuf->dma_size, DMA_FROM_DEVICE);

	/* Reprogram the channel */
	chan_writel(chan, HSU_CH_D0SAR, dbuf->dma_addr);
	chan_writel(chan, HSU_CH_D0TSR, dbuf->dma_size);
	chan_writel(chan, HSU_CH_DCR, 0x1
					 | (0x1 << 8)
					 | (0x1 << 16)
					 | (0x1 << 24)	/* timeout bit, see HSU Errata 1 */
					 );
	if (tty->low_latency)
		/* Schedule the remainder of hsu_dma_rx function (now located
		 * in hsu_dma_rx_tasklet function) to be executed via a
		 * tasklet execution outside IRQ context */
		tasklet_schedule(&up->hsu_dma_rx_tasklet);
	else
		/* Directly call the remainder of hsu_dma_rx function
		 * form IRQ context */
		hsu_dma_rx_tasklet((unsigned long)up);
	tty_kref_put(tty);
}


static void serial_hsu_stop_rx(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;

	if (up->use_dma) {
		if (!hsu_port_is_active(up)) {
			spin_lock_irqsave(&up->qlock, flags);
			insert_q(up, CMD_RX_STOP, 0, 0);
			spin_unlock_irqrestore(&up->qlock, flags);
			schedule_work(&up->qwork);
		} else {
			chan_writel(up->rxc, HSU_CH_CR, 0x2);
		}
		up->dma_rx_on = 0;
	} else {
		up->ier &= ~UART_IER_RLSI;
		up->port.read_status_mask &= ~UART_LSR_DR;
		serial_out(up, UART_IER, up->ier);
	}
}

static inline void receive_chars(struct uart_hsu_port *up, int *status)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned int ch, flag;
	unsigned int max_count = 256;

	if (!tty)
		return;

	do {
		ch = serial_in_irq(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {

			/* For statistics only */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/* Mask off conditions which should be ignored. */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE
			if (up->port.cons &&
				up->port.cons->index == up->port.line) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI) {
				flag = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);
	ignore_char:
		*status = serial_in_irq(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && max_count--);
	tty_flip_buffer_push(tty);
}

static void transmit_chars(struct uart_hsu_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_out_irq(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_hsu_stop_tx(&up->port);
		return;
	}

#ifndef MFD_HSU_A0_STEPPING
	count = up->port.fifosize / 2;
#else
	/*
	 * A0 only supports fully empty IRQ, and the first char written
	 * into it won't clear the EMPT bit, so we may need be cautious
	 * by useing a shorter buffer
	 */
	count = up->port.fifosize - 4;
#endif
	do {
		serial_out_irq(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_hsu_stop_tx(&up->port);
}

static inline void check_modem_status(struct uart_hsu_port *up)
{
	int status;

	status = serial_in_irq(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	/* We may only get DDCD when HW init and reset */
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	/* Will start/stop_tx accordingly */
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
}

/* triggered by RX/CTS, just do a get and put, the idle timeout
 * will give us a change work
 */
static irqreturn_t wakeup_irq(int irq, void *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);

	dev_dbg(dev, "HSU wake up\n");

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT
	if (up->index == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT)
		set_bit(PM_WAKEUP, &up->pm_flags);
	else if (up->index == logic_idx &&
		share_idx == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		if (up3->running)
			set_bit(PM_WAKEUP, &up3->pm_flags);
	}
#endif

	pm_runtime_get(dev);
	pm_runtime_put(dev);
	return IRQ_HANDLED;
}

/*
 * This handles the interrupt from one port.
 */
static irqreturn_t port_irq(int irq, void *dev_id)
{
	struct uart_hsu_port *up = dev_id;
	unsigned int iir, lsr;

	if (up->index == logic_idx && !up->running)
		up = serial_hsu_ports[share_idx];

	if (unlikely(!up->running))
		return IRQ_NONE;

	pm_runtime_get(up->dev);
	spin_lock(&up->port.lock);
	if (up->use_dma) {
		lsr = serial_in_irq(up, UART_LSR);
		check_modem_status(up);
		spin_unlock(&up->port.lock);

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
					UART_LSR_FE | UART_LSR_OE)))
			dev_warn(up->dev,
				"Got lsr irq while using DMA, lsr = 0x%2x\n",
				lsr);

		pm_runtime_put(up->dev);
		return IRQ_HANDLED;
	}

	iir = serial_in_irq(up, UART_IIR);
	if (iir & UART_IIR_NO_INT) {
		spin_unlock(&up->port.lock);
		pm_runtime_put(up->dev);
		return IRQ_NONE;
	}

	lsr = serial_in_irq(up, UART_LSR);
	if (lsr & UART_LSR_DR)
		receive_chars(up, &lsr);
	check_modem_status(up);

	/* lsr will be renewed during the receive_chars */
	if (lsr & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock(&up->port.lock);
	pm_runtime_put(up->dev);
	return IRQ_HANDLED;
}

static inline void dma_chan_irq(struct hsu_dma_chan *chan)
{
	struct uart_hsu_port *up = chan->uport;
	unsigned long flags;
	u32 int_sts;

	if (up->index == logic_idx && !up->running)
		up = serial_hsu_ports[share_idx];

	pm_runtime_get(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * No matter what situation, need read clear the IRQ status
	 * There is a bug, see Errata 5, HSD 2900918
	 */
	int_sts = chan_readl(chan, HSU_CH_SR);

	if (!up->use_dma || !up->running)
		goto exit;

	/* Rx channel */
	if (up->dma_rx_on && chan->dirt == DMA_FROM_DEVICE) {
		if (up->dev->power.runtime_status == RPM_ACTIVE ||
		up->dev->power.runtime_status == RPM_RESUMING)
			hsu_dma_rx(up, int_sts);
	}
	/* Tx channel */
	if (chan->dirt == DMA_TO_DEVICE) {
		if (up->dev->power.runtime_status == RPM_ACTIVE ||
		up->dev->power.runtime_status == RPM_RESUMING) {
			chan_writel(chan, HSU_CH_CR, 0x0);
			up->dma_tx_on = 0;
			hsu_dma_tx(up);
		}
	}

exit:
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_put(up->dev);
	return;
}

static irqreturn_t dma_irq(int irq, void *dev_id)
{
	struct hsu_port *hsu = dev_id;
	u32 int_sts, i;

	int_sts = mfd_readl(hsu, HSU_GBL_DMAISR);

	/* Currently we only have 6 channels may be used */
	for (i = 0; i < 6; i++) {
		if (int_sts & 0x1)
			dma_chan_irq(&hsu->chans[i]);
		int_sts >>= 1;
	}

	return IRQ_HANDLED;
}

static unsigned int serial_hsu_tx_empty(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serial_hsu_get_mctrl(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned char status;
	unsigned int ret;

	status = serial_in(up, UART_MSR);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_hsu_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr |= up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serial_hsu_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * What special to do:
 * 1. chose the 64B fifo mode
 * 2. start dma or pio depends on configuration
 * 3. we only allocate dma memory when needed
 */
static int serial_hsu_startup(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;
	static DEFINE_MUTEX(hsu_lock);

	mutex_lock(&hsu_lock);
	if (up->index == share_idx) {
		struct uart_hsu_port *up1 = serial_hsu_ports[logic_idx];
		/* wait for mux port to close */
		while (up1->running)
			msleep(1000);
		intel_mid_hsu_switch(share_idx);
	}
	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		if (up3->running) {
			uart_suspend_port(&serial_hsu_reg, &up3->port);
			/* after suspend port, up3->running will be 0
			 * while we need keep it, so we can resume later
			 */
			up3->running = 1;
		}
		intel_mid_hsu_switch(logic_idx);
	}
	/* startup function is not under atomic context for sure */
	pm_runtime_get_sync(up->dev);
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);

	/* Clear the interrupt registers. */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/* Now, initialize the UART, default is 8n1 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);

	up->port.mctrl |= TIOCM_OUT2;
	serial_hsu_set_mctrl(&up->port, up->port.mctrl);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	if (!up->use_dma)
		up->ier = UART_IER_RLSI | UART_IER_RDI | UART_IER_RTOIE;
	else
		up->ier = 0;
	serial_out(up, UART_IER, up->ier);

	spin_unlock_irqrestore(&up->port.lock, flags);

	/* DMA init */
	if (up->use_dma) {
		struct hsu_dma_buffer *dbuf;
		struct circ_buf *xmit = &port->state->xmit;

		up->dma_tx_on = 0;

		/* First allocate the RX buffer */
		dbuf = &up->rxbuf;
		dbuf->buf = kzalloc(dma_dscr_size, GFP_KERNEL);
		if (!dbuf->buf) {
			up->use_dma = 0;
			goto exit;
		}
		dbuf->dma_addr = dma_map_single(port->dev,
						dbuf->buf,
						dma_dscr_size,
						DMA_FROM_DEVICE);
		dbuf->dma_size = dma_dscr_size;

		/* Start the RX channel right now */
		hsu_dma_start_rx_chan(up, dbuf);

		/* Next init the TX DMA */
		dbuf = &up->txbuf;
		dbuf->buf = xmit->buf;
		dbuf->dma_addr = dma_map_single(port->dev,
					       dbuf->buf,
					       UART_XMIT_SIZE,
					       DMA_TO_DEVICE);
		dbuf->dma_size = UART_XMIT_SIZE;

		/* This should not be changed all around */
		chan_writel(up->txc, HSU_CH_BSR, 32);
		chan_writel(up->txc, HSU_CH_MOTSR, 4);
		dbuf->ofs = 0;
	}

exit:
	 /* And clear the interrupt registers again for luck. */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	up->running = 1;
	mutex_unlock(&hsu_lock);
	pm_runtime_put(up->dev);
	return 0;
}

static void qwork(struct work_struct *work);
static void serial_hsu_shutdown(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;
	static DEFINE_MUTEX(hsu_lock);

	pm_runtime_get_sync(up->dev);

	/* clear the queue */
	cancel_work_sync(&up->qwork);
	qwork(&up->qwork);
	spin_lock_irqsave(&up->qlock, flags);
	up->qcirc.head = up->qcirc.tail = 0;
	spin_unlock_irqrestore(&up->qlock, flags);
	mutex_lock(&hsu_lock);
	up->running = 0;
	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		intel_mid_hsu_switch(share_idx);
		if (up3->running) {
			mutex_unlock(&hsu_lock);
			uart_resume_port(&serial_hsu_reg, &up3->port);
			goto f_out;
		}
	} else if (up->index == share_idx) {
		struct uart_hsu_port *up1 = serial_hsu_ports[logic_idx];
		if (up1->running) {
			mutex_unlock(&hsu_lock);
			goto f_out;
		}

	}

	/* Disable interrupts from this port */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	/* Free allocated dma buffer */
	if (up->use_dma) {
		struct hsu_dma_buffer *dbuf;

		/* stop dma */
		chan_writel(up->txc, HSU_CH_CR, 0x0);
		up->dma_tx_on = 0;

		chan_writel(up->rxc, HSU_CH_CR, 0x2);
		up->dma_rx_on = 0;

		/* Free and unmap rx dma buffer */
		dbuf = &up->rxbuf;
		dma_unmap_single(port->dev,
				dbuf->dma_addr,
				dbuf->dma_size,
				DMA_FROM_DEVICE);

		kfree(dbuf->buf);

		/* Next unmap tx dma buffer*/
		dbuf = &up->txbuf;
		dma_unmap_single(port->dev,
				dbuf->dma_addr,
				dbuf->dma_size,
				DMA_TO_DEVICE);
	}

	mutex_unlock(&hsu_lock);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;
	serial_hsu_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/* Disable break condition and FIFOs */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
				  UART_FCR_CLEAR_RCVR |
				  UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);

f_out:
	pm_runtime_put(up->dev);
}

static void
serial_hsu_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_hsu_port *up =
			container_of(port, struct uart_hsu_port, port);
	struct tty_struct *tty = port->state->port.tty;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	u32 ps, mul;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	/* CMSPAR isn't supported by this driver */
	if (tty)
		tty->termios->c_cflag &= ~CMSPAR;

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * The base clk is 50Mhz, and the baud rate come from:
	 *	baud = 50M * MUL / (DIV * PS * DLAB)
	 *
	 * For those basic low baud rate we can get the direct
	 * scalar from 2746800, like 115200 = 2746800/24. For those
	 * higher baud rate, we handle them case by case, mainly by
	 * adjusting the MUL/PS registers, and DIV register is kept
	 * as default value 0x3d09 to make things simple
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);

	quot = 1;
	ps = 0x10;
	mul = 0x3600;
	switch (baud) {
	case 3500000:
		mul = 0x3345;
		ps = 0xC;
		break;
	case 1843200:
		mul = 0x2400;
		break;
	case 3000000:
	case 2500000:
	case 2000000:
	case 1500000:
	case 1000000:
	case 500000:
		/* mul/ps/quot = 0x9C4/0x10/0x1 will make a 500000 bps */
		mul = baud / 500000 * 0x9C4;
		break;
	default:
		/* Use uart_get_divisor to get quot for other baud rates */
		quot = 0;
	}

	if (!quot)
		quot = uart_get_divisor(port, baud);

	if ((up->port.uartclk / quot) < (2400 * 16))
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_1B;
	else if ((up->port.uartclk / quot) < (230400 * 16))
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_16B;
	else
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_HSU_64_32B;

	fcr |= UART_FCR_HSU_64B_FIFO;
#ifdef MFD_HSU_A0_STEPPING
	/* A0 doesn't support half empty IRQ */
	fcr |= UART_FCR_FULL_EMPT_TXI;
#endif

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/* Characters to ignore */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts, disable
	 * MSI by default
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE | UART_MCR_RTS;
	else
		up->mcr &= ~UART_MCR_AFE;

	serial_out(up, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
	serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_out(up, UART_LCR, cval);			/* reset DLAB */
	serial_out(up, UART_MUL, mul);			/* set MUL */
	serial_out(up, UART_PS, ps);			/* set PS */
	up->lcr = cval;					/* Save LCR */
	serial_hsu_set_mctrl(&up->port, up->port.mctrl);
	serial_out(up, UART_FCR, fcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
serial_hsu_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
}

static void serial_hsu_release_port(struct uart_port *port)
{
}

static int serial_hsu_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_hsu_config_port(struct uart_port *port, int flags)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	up->port.type = PORT_MFD;
}

static int
serial_hsu_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* We don't want the core code to modify any port params */
	return -EINVAL;
}

static const char *
serial_hsu_type(struct uart_port *port)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	return up->name;
}

#ifdef CONFIG_CONSOLE_POLL
static int serial_hsu_get_poll_char(struct uart_port *port)
{
	u8 lsr;
	struct uart_hsu_port *up;
	void __iomem	*base;

	up = container_of(port, struct uart_hsu_port, port);
	base = up->port.membase;

	lsr = readb(base + UART_LSR);

	if (!(lsr & UART_LSR_DR))
		return NO_POLL_CHAR;

	return readb(base + UART_RX);
}

#define HSU_POLL_TX_TIMEOUT	10000 /* 10 ms */
static void hsu_poll_wait_for_xmit(void *base)
{
	u8 status;
	unsigned int timeout = HSU_POLL_TX_TIMEOUT;

	while (--timeout) {
		status = readb(base + UART_LSR);
		if (status & (UART_LSR_TEMT | UART_LSR_THRE))
			break;
		udelay(1);
	}
}

static void serial_hsu_put_poll_char(struct uart_port *port,
			unsigned char c)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	void __iomem	*base = up->port.membase;

	/* save IER then disable interrupts */
	u8 ier = readb(base + UART_IER);
	writeb(0, base + UART_IER);

	/* send the char */
	hsu_poll_wait_for_xmit(base);
	writeb(c, base + UART_TX);

	/* wait for transmitter to be empty and restore IER */
	hsu_poll_wait_for_xmit(base);
	writeb(ier, base + UART_IER);
}
#endif

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/* Wait for transmitter & holding register to empty */
static inline void wait_for_xmitr(struct uart_hsu_port *up)
{
	unsigned int status, tmout = 1000;

	if (!hsu_port_is_active(up))
		return;

	/* Wait up to 1ms for the character to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while (!(status & BOTH_EMPTY));

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
		       ((serial_in(up, UART_MSR) & UART_MSR_CTS) == 0))
			udelay(1);
	}
}

static inline int try_xmitr(struct uart_hsu_port *up)
{
	unsigned int status, tmout = 10000;

	while (--tmout) {
		status = serial_in_irq(up, UART_LSR);
		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;
		udelay(1);
		if (status & BOTH_EMPTY)
			break;
	}
	if (tmout == 0)
		return 0;

	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 10000;
		while (--tmout &&
		       ((serial_in_irq(up, UART_MSR) & UART_MSR_CTS) == 0))
			udelay(1);
		if (tmout == 0)
			return 0;
	}
	return 1;
}

static void serial_hsu_console_putchar(struct uart_port *port, int ch)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);

#ifdef CONFIG_EMMC_IPANIC
	static int oops_char_len;

	if (oops_in_progress && oops_char_len++ > 2048)
		return;
	if (!oops_in_progress)
		oops_char_len = 0;
#endif
	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

static void serial_hsu_console_qchar(struct uart_port *port, int ch)
{
	struct uart_hsu_port *up =
		container_of(port, struct uart_hsu_port, port);
	unsigned long flags;

	spin_lock_irqsave(&up->qlock, flags);
	insert_q(up, CMD_WB, UART_TX, ch & 0xff);
	spin_unlock_irqrestore(&up->qlock, flags);
	schedule_work(&up->qwork);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_hsu_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_hsu_port *up = serial_hsu_ports[co->index];
	unsigned long flags;
	int locked = 1;

	/* if this console write was due to console dev resuming/suspending
	 * we must stop it, otherwise the power->lock dead lock ....
	 */
#ifdef CONFIG_PM_RUNTIME
	if (up->dev->power.runtime_status == RPM_RESUMING ||
			up->dev->power.runtime_status == RPM_SUSPENDING) {
		spin_lock(&up->port.lock);
		uart_console_write(&up->port, s, count,
				serial_hsu_console_qchar);
		spin_unlock(&up->port.lock);
		return;
	}
#endif
	pm_runtime_get(up->dev);
	local_irq_save(flags);
	if (up->port.sysrq)
		locked = 0;
	else if (oops_in_progress) {
		locked = spin_trylock(&up->port.lock);
	} else
		spin_lock(&up->port.lock);

	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_hsu_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, up->ier);

	if (locked)
		spin_unlock(&up->port.lock);
	local_irq_restore(flags);
	pm_runtime_put(up->dev);
}

static struct console serial_hsu_console;

static int __init
serial_hsu_console_setup(struct console *co, char *options)
{
	struct uart_hsu_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	if (co->index == -1 || co->index >= serial_hsu_reg.nr)
		co->index = 0;
	up = serial_hsu_ports[co->index];
	if (!up)
		return -ENODEV;

	pm_runtime_get_sync(up->dev);
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(&up->port, co, baud, parity, bits, flow);
	pm_runtime_put(up->dev);

	return ret;
}

static struct console serial_hsu_console = {
	.name		= "ttyMFD",
	.write		= serial_hsu_console_write,
	.device		= uart_console_device,
	.setup		= serial_hsu_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT,
	.data		= &serial_hsu_reg,
};
#endif

struct uart_ops serial_hsu_pops = {
	.tx_empty	= serial_hsu_tx_empty,
	.set_mctrl	= serial_hsu_set_mctrl,
	.get_mctrl	= serial_hsu_get_mctrl,
	.stop_tx	= serial_hsu_stop_tx,
	.start_tx	= serial_hsu_start_tx,
	.stop_rx	= serial_hsu_stop_rx,
	.enable_ms	= serial_hsu_enable_ms,
	.break_ctl	= serial_hsu_break_ctl,
	.startup	= serial_hsu_startup,
	.shutdown	= serial_hsu_shutdown,
	.set_termios	= serial_hsu_set_termios,
	.pm		= serial_hsu_pm,
	.type		= serial_hsu_type,
	.release_port	= serial_hsu_release_port,
	.request_port	= serial_hsu_request_port,
	.config_port	= serial_hsu_config_port,
	.verify_port	= serial_hsu_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = serial_hsu_get_poll_char,
	.poll_put_char = serial_hsu_put_poll_char,
#endif
};

static struct uart_driver serial_hsu_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "MFD serial",
	.dev_name	= "ttyMFD",
	.major		= TTY_MAJOR,
	.minor		= 128,
	.nr		= 4,
};

/* temp global pointer before we settle down on using one or four PCI dev */
static struct hsu_port *phsu;

static void hsu_init_low_latency_tasklet(struct uart_hsu_port *uport,
				int hsu_index)
{
	/* Unconditionally initialize tasklet.
	 * depending if low latency setting is
	 * active or not it is enabled
	 * or disabled */
	tasklet_init(&uport->hsu_dma_rx_tasklet,
		hsu_dma_rx_tasklet,
		(unsigned long)uport);
	/* Depending on hsu_low_latency module parameter
	 * activate low latency mode, tasklet is enabled
	 * by default after tasklet_init*/
	if (hsu_low_latency & (1<<hsu_index)) {
		uport->port.flags |= UPF_LOW_LATENCY;
	} else {
		uport->port.flags &= ~UPF_LOW_LATENCY;
		/* Tasklet can be disabled as it will never be scheduled */
		tasklet_disable(&uport->hsu_dma_rx_tasklet);
	}
}

static int serial_hsu_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	struct uart_hsu_port *uport;
	int index, ret;

	printk(KERN_INFO "HSU: found PCI Serial controller(ID: %04x:%04x)\n",
		pdev->vendor, pdev->device);

	switch (pdev->device) {
	case 0x081B:
	case 0x08FC:
		index = 0;
		break;
	case 0x081C:
	case 0x08FD:
		index = 1;
		break;
	case 0x081D:
	case 0x08FE:
		index = 2;
		break;
	case 0x081E:
	case 0x08FF:
		/* internal DMA controller */
		index = 3;
		break;
	default:
		dev_err(&pdev->dev, "HSU: out of index!");
		return -ENODEV;
	}

	/*
	 * There is a hsu rx timeout interrup lost silicon bug, this workaround
	 * is to shrink dma descriptor xfer size to 16 Bytes.
	 * PNW A0 and CLVP A0 need this workaround.
	 */
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW &&
		pdev->revision < 0xC) {
		dev_warn(&pdev->dev, "CLVP A0 detected, dma_dscr_size=16\n");
		dma_dscr_size = 16;
	} else if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL &&
		pdev->revision < 0x8) {
		dev_warn(&pdev->dev, "PNW A0 detected, dma_dscr_size=16\n");
		dma_dscr_size = 16;
	}

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	if (index == 3) {
		/* DMA controller */
		ret = request_irq(pdev->irq, dma_irq, 0, "hsu_dma", phsu);
		if (ret) {
			dev_err(&pdev->dev, "can not get IRQ\n");
			goto err_disable;
		}
		pci_set_drvdata(pdev, phsu);
	} else
		do {
			/* UART port 0~2 is physical, port 3 is mirror */
			uport = &phsu->port[index];
			uport->port.irq = pdev->irq;
			uport->port.dev = &pdev->dev;
			uport->dev = &pdev->dev;
			intel_mid_hsu_init(index, &pdev->dev, wakeup_irq);

			if (index != 3) {
				ret = request_irq(pdev->irq, port_irq, 0, uport->name, uport);
				if (ret) {
					dev_err(&pdev->dev, "can not get IRQ\n");
					goto err_disable;
				}

				pci_set_drvdata(pdev, uport);
				pm_runtime_put_noidle(&pdev->dev);
				pm_runtime_allow(&pdev->dev);
			}
			hsu_init_low_latency_tasklet(uport, index);
			uart_add_one_port(&serial_hsu_reg, &uport->port);

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE
			if (index == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT) {
				register_console(&serial_hsu_console);
				uport->port.cons = &serial_hsu_console;
			}
#endif
			if ( index == 1 ) {
				index = 3;
			} else
				break;
		}while(1);

	intel_mid_hsu_port_map(&logic_idx, &share_idx);
	return 0;

err_disable:
	pci_disable_device(pdev);
	return ret;
}

static void qwork(struct work_struct *work)
{
	struct uart_hsu_port *up =
		container_of(work, struct uart_hsu_port, qwork);
	unsigned long flags;
	int cmd, offset, value, count = 0;

	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->qlock, flags);
	while (get_q(up, &cmd, &offset, &value)){
		dev_dbg(up->dev, "qwork get cmd %d %d %d\n", cmd, offset, value);
		switch (cmd) {
		case CMD_WL:
			writel(value, up->port.membase + offset);
			break;
		case CMD_WB:
			if (UART_TX == offset) {
				/* for the console polling mode will take a long
				   time to send out characters that queued
				   during pm non-active statues. 140bytes on
				   115200bps about 10ms to schedule.
				*/
				while (!try_xmitr(up) || count > 140) {
					count = 0;

					spin_unlock_irqrestore(&up->qlock,
								flags);
					schedule();
					spin_lock_irqsave(&up->qlock, flags);
				}
			}
			writeb(value, up->port.membase + offset);
			count++;
			break;
		case CMD_TX:
			hsu_dma_tx(up);
			break;
		case CMD_RX_STOP:
			chan_writel(up->rxc, HSU_CH_CR, 0x2);
			break;
		case CMD_TX_STOP:
			chan_writel(up->txc, HSU_CH_CR, 0x0);
			break;
		default:
			dev_err(up->dev, "wrong queue cmd type!\n");
		}
	}
	spin_unlock_irqrestore(&up->qlock, flags);
	pm_runtime_put(up->dev);
}

static void hsu_global_init(void)
{
	struct hsu_port *hsu;
	struct uart_hsu_port *uport;
	struct hsu_dma_chan *dchan;
	int i, ret;

	hsu = kzalloc(sizeof(struct hsu_port), GFP_KERNEL);
	if (!hsu)
		return;

	/* Get basic io resource and map it */
	hsu->paddr = 0xffa28000;
	hsu->iolen = 0x1000;

	if (!(request_mem_region(hsu->paddr, hsu->iolen, "HSU global")))
		pr_warning("HSU: error in request mem region\n");

	hsu->reg = ioremap_nocache((unsigned long)hsu->paddr, hsu->iolen);
	if (!hsu->reg) {
		pr_err("HSU: error in ioremap\n");
		ret = -ENOMEM;
		goto err_free_region;
	}

	/* Initialise the 3 UART ports */
	uport = hsu->port;
	for (i = 0; i < 4; i++) {
		int offset = i;

		if (offset == 3) /* port 3 is mux of port 1*/
			offset = 1;
		uport->port.type = PORT_MFD;
		uport->port.iotype = UPIO_MEM;
		uport->port.mapbase = (resource_size_t)hsu->paddr
					+ HSU_PORT_REG_OFFSET
					+ offset * HSU_PORT_REG_LENGTH;
		uport->port.membase = hsu->reg + HSU_PORT_REG_OFFSET
					+ offset * HSU_PORT_REG_LENGTH;

		snprintf(uport->name, sizeof(uport->name), "hsu_port%d", i);
		uport->port.fifosize = 64;
		uport->port.ops = &serial_hsu_pops;
		uport->port.line = i;
		uport->port.flags = UPF_IOREMAP;
		/* set the scalable maxim support rate to 2746800 bps */
		uport->port.uartclk = 115200 * 24 * 16;

		uport->running = 0;
		uport->txc = &hsu->chans[offset * 2];
		uport->rxc = &hsu->chans[offset * 2 + 1];

		serial_hsu_ports[i] = uport;
		uport->index = i;

		if (hsu_dma_enable & (1<<i))
			uport->use_dma = 1;
		else
			uport->use_dma = 0;

		uport->qcirc.buf = (char *)uport->qbuf;
		uport->qcirc.head = uport->qcirc.tail = 0;
		INIT_WORK(&uport->qwork, qwork);
		spin_lock_init(&uport->qlock);

		uport++;
	}

	/* Initialise 6 dma channels */
	dchan = hsu->chans;
	for (i = 0; i < 6; i++) {
		dchan->id = i;
		dchan->dirt = (i & 0x1) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
		/* for port1 mux case, we will fix it in dma call backs */
		dchan->uport = &hsu->port[i/2];
		dchan->reg = hsu->reg + HSU_DMA_CHANS_REG_OFFSET +
				i * HSU_DMA_CHANS_REG_LENGTH;
		dchan++;
	}

	phsu = hsu;

	hsu_debugfs_init(hsu);

	return;

err_free_region:
	release_mem_region(hsu->paddr, hsu->iolen);
	kfree(hsu);
	return;
}

static void serial_hsu_remove(struct pci_dev *pdev)
{
	void *priv = pci_get_drvdata(pdev);
	struct uart_hsu_port *up;

	if (!priv)
		return;

	/* For port 0/1/2, priv is the address of uart_hsu_port */
	if ((pdev->device != 0x081E) && (pdev->device != 0x08FF)) {
		up = priv;
		/* Moved cancel_work_sync call after up initialization */
		cancel_work_sync(&up->qwork);
		/* Free low latency tasklet ressources */
		tasklet_kill(&up->hsu_dma_rx_tasklet);
		uart_remove_one_port(&serial_hsu_reg, &up->port);
		if (up->index == logic_idx) {
			up = serial_hsu_ports[share_idx];
			uart_remove_one_port(&serial_hsu_reg, &up->port);
		}
	}

	pci_set_drvdata(pdev, NULL);
	free_irq(pdev->irq, priv);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pci_disable_device(pdev);
}

/* First 3 are UART ports, and the 4th is the DMA */
static const struct pci_device_id pci_ids[] __devinitdata = {
	/* Penwell support */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081B) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081C) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081D) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081E) },

	/* Cloverview support */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08FC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08FD) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08FE) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08FF) },
	{},
};

#ifdef CONFIG_PM
static bool allow_for_suspend(struct uart_hsu_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	struct hsu_dma_chan *chan = up->rxc;
	struct hsu_dma_buffer *dbuf = &up->rxbuf;
	int rx_count;

	if (!uart_circ_empty(xmit)) {
		dev_dbg(up->dev, "%s: circ_not_empty\n", __func__);
		return false;
	}

	if (up->use_dma) {
		if (up->dma_rx_on) {
			rx_count = chan_readl(chan, HSU_CH_D0SAR) -
				dbuf->dma_addr;
			if (rx_count) {
				dev_dbg(up->dev, "%s: rx cnt=%d\n",
					__func__, rx_count);
				return false;
			}
		}
		if (up->dma_tx_on) {
			dev_dbg(up->dev, "%s: dma_tx_on\n", __func__);
			return false;
		}
	}

	return true;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int hsu_runtime_idle(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);
	struct uart_hsu_port *up_pair = NULL;
	unsigned int delay = 0;

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT
	if (up->index == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT &&
		test_bit(PM_WAKEUP, &up->pm_flags))
		up_pair = up;
#endif

	if (up->index == logic_idx)
		up_pair = serial_hsu_ports[share_idx];
	if (system_state == SYSTEM_BOOTING) {
		/* if HSU is set as default console, but earlyprintk is not hsu,
		 * then it will enter suspend and can not get back since system
		 * is on boot up, no contex switch to let it resume, here just
		 * postpone the suspend retry 30 seconds, then system should
		 * have finished booting
		 */
		delay = 30000;
	} else if ((up_pair && up_pair->running)) {
		/* console timeout need longer, because human input slower */
		if (test_bit(PM_WAKEUP, &up_pair->pm_flags))
			delay = 2000;
		else
			delay = 100;
	} else if (up->running) {
		if (up->suspended)
			/*need to set longer for S3 resuming*/
			delay = 500;
		else if (up->index == 0)
			/* idle detection handled by wl12xx combo chip, the chip
			 * is now asleep so no need for any additional delay.
			 */
			delay = 20;
		else
			delay = 100;
	}

	pm_schedule_suspend(dev, delay);
	return -EBUSY;
}

static int hsu_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);

	if (up->running)
		intel_mid_hsu_suspend(up->index);

	if (!allow_for_suspend(up)) {
		if (up->use_dma && up->dma_rx_on)
			hsu_dma_rx(up, 0);

		if (up->running)
			intel_mid_hsu_resume(up->index);

		pm_schedule_suspend(dev, 100);
		return -EBUSY;
	}

	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		if (up3->running) {
			intel_mid_hsu_suspend(up3->index);
		}
	}

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT
	if (up->index == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT)
		clear_bit(PM_WAKEUP, &up->pm_flags);
	else if (up->index == logic_idx &&
		share_idx == CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		if (up3->running)
			clear_bit(PM_WAKEUP, &up3->pm_flags);
	}
#endif

	chan_writel(up->rxc, HSU_CH_CR, 0x2);
	disable_irq(up->port.irq);

	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		memcpy(up3->reg_shadow + 1, up3->port.membase + 1,
			HSU_PORT_REG_LENGTH - 1);
	}
	memcpy(up->reg_shadow + 1, up->port.membase + 1, HSU_PORT_REG_LENGTH - 1);

	return 0;
}

static int hsu_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct uart_hsu_port *up = pci_get_drvdata(pdev);
	int dma_rx_on = 0;

	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];

		if (up3->dma_rx_on) {
			dma_rx_on = 1;
		}
	}

	enable_irq(up->port.irq);
	if (up->dma_rx_on || dma_rx_on)
		chan_writel(up->rxc, HSU_CH_CR, 0x3);

	if (up->running)
		intel_mid_hsu_resume(up->index);

	if (up->index == logic_idx) {
		struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
		if (up3->running)
			intel_mid_hsu_resume(up3->index);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM

static int hsu_suspend_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	void *priv = pci_get_drvdata(pdev);
	struct uart_hsu_port *up;

	/* Make sure this is not the internal dma controller */
	if (priv && (pdev->device != 0x081E) && (pdev->device != 0x08FF)) {
		up = priv;
		if (!allow_for_suspend(up))
			return -EBUSY;

#ifdef CONFIG_PM_RUNTIME
		/* check if RPM suspend has been unlocked */
		if (atomic_read(&up->dev->power.usage_count) > 1
		    || up->dev->power.disable_depth > 0) {
			dev_dbg(up->dev, "%s: rmp is active\n", __func__);
			return -EBUSY;
		}
#endif
	}

	return 0;
}

static int hsu_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	void *priv = pci_get_drvdata(pdev);
	struct uart_hsu_port *up;

	/* Make sure this is not the internal dma controller */
	if (priv && (pdev->device != 0x081E) && (pdev->device != 0x08FF)) {
		up = priv;

		if (query_q(up))
			return -EBUSY;

		if (!allow_for_suspend(up))
			return -EBUSY;

#ifdef CONFIG_PM_RUNTIME
		/* check if RPM suspend has been unlocked */
		if (atomic_read(&up->dev->power.usage_count) > 1
		    || up->dev->power.disable_depth > 0) {
			dev_dbg(up->dev, "%s: rmp is active\n", __func__);
			return -EBUSY;
		}
#endif

		disable_irq(up->port.irq);
		if (up->index == logic_idx) {
			struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
			if (up3->running) {
				uart_suspend_port(&serial_hsu_reg, &up3->port);
				/* after suspend port, up3->running will be 0
				 * while we need keep it, so we can resume later
				 */
				up3->running = 1;
			}
			up3->suspended = 1;
		}
		if (up->running) {
			uart_suspend_port(&serial_hsu_reg, &up->port);
			up->running = 1;
			intel_mid_hsu_suspend(up->index);
		}
		up->suspended = 1;
	}
	return 0;
}

static int hsu_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	void *priv = pci_get_drvdata(pdev);
	struct uart_hsu_port *up;

	if (priv && (pdev->device != 0x081E) && (pdev->device != 0x08FF)) {
		up = priv;

		if (up->suspended)
			enable_irq(up->port.irq);

		if (up->index == logic_idx) {
			struct uart_hsu_port *up3 = serial_hsu_ports[share_idx];
			if (up3->suspended && up3->running) {
				uart_resume_port(&serial_hsu_reg, &up3->port);
				schedule_work(&up3->qwork);
			}
			up3->suspended = 0;
		}

		if (up->suspended && up->running) {
			uart_resume_port(&serial_hsu_reg, &up->port);
			intel_mid_hsu_resume(up->index);
			schedule_work(&up->qwork);
		}
		up->suspended = 0;
	}
	return 0;
}

static const struct dev_pm_ops hsu_pm_ops = {
	.suspend_noirq = hsu_suspend_noirq,
	.suspend = hsu_suspend,
	.resume = hsu_resume,
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = hsu_runtime_suspend,
	.runtime_resume = hsu_runtime_resume,
	.runtime_idle = hsu_runtime_idle,
#endif
};

#endif

static struct pci_driver hsu_pci_driver = {
	.name =		"HSU serial",
	.id_table =	pci_ids,
	.probe =	serial_hsu_probe,
	.remove =	__devexit_p(serial_hsu_remove),

/* Disable PM only when kgdb(poll mode uart) is enabled */
#if defined(CONFIG_PM) && !defined(CONFIG_CONSOLE_POLL)
	.driver =	{
		.pm =	&hsu_pm_ops
	},
#endif
};

static int __init hsu_pci_init(void)
{
	int ret;

	hsu_global_init();

	ret = uart_register_driver(&serial_hsu_reg);
	if (ret)
		return ret;

	return pci_register_driver(&hsu_pci_driver);
}

static void __exit hsu_pci_exit(void)
{
	int i;
	/* Free low latency tasklet ressources for each hsu */
	for (i = 0; i < 3; i++)
		tasklet_kill(&phsu->port->hsu_dma_rx_tasklet);

	pci_unregister_driver(&hsu_pci_driver);
	uart_unregister_driver(&serial_hsu_reg);

	hsu_debugfs_remove(phsu);

	kfree(phsu);
}

module_init(hsu_pci_init);
module_exit(hsu_pci_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:medfield-hsu");
