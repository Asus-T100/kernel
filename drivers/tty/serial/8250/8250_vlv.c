/*
 * 8250_vlv.c: driver for High Speed UART device of Intel ValleyView2
 *
 * Refer 8250.c and some other drivers in drivers/serial/
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/debugfs.h>
#include <asm/intel-mid.h>
#include <linux/pm_runtime.h>

#include "8250.h"

MODULE_AUTHOR("Yang Bin <bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel ValleyView HSU Runtime PM friendly Driver");
MODULE_LICENSE("GPL");

enum {
	baylake_0 = 0,
	baylake_1,
};

struct vlv_hsu_port {
	char			*name;
	int			use_dma;
	int			irq;
	int			*dev;
	unsigned char __iomem	*membase;
	int			idle_delay;
	int			wake_gpio;
	int			last_lcr;
	int			line;

	/* For reqclk generation */
	u32			m;
	u32			n;

	struct dentry		*debugfs;
};

struct vlv_hsu_config {
	char *name;
	int use_dma;
	int uartclk;
	int wake_gpio;
	int idle_delay;
	void(*setup)(struct vlv_hsu_port *);
};

static void vlv_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
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

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_port_out(port, UART_MCR, mcr);
}

static inline u32 set_clk_param(struct vlv_hsu_port *vp, u32 m, u32 n)
{
	u32 param, update_bit;

	update_bit = 1 << 31;
	param = (m << 1) | (n << 16) | 0x1;

	writel(param, (vp->membase + 0x800));
	writel((param | update_bit), (vp->membase + 0x800));
	writel(param, (vp->membase + 0x800));
}


void vlv_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot, m, n;
	struct vlv_hsu_port *vp = port->private_data;

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

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, 4000000);

	/* need calc quot here */
	switch (baud) {
	case 3000000:
	case 1500000:
	case 1000000:
	case 500000:
		m = 48;
		n = 100;
		quot = 3000000 / baud;
		break;
	default:
		m = 9216;
		n = 15625;
		quot = 0;
	}
	if (!quot)
		quot = uart_get_divisor(port, baud);

	fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 |
		UART_FCR_T_TRIG_11;
	if (baud < 2400) {
		fcr &= ~UART_FCR_TRIGGER_MASK;
		fcr |= UART_FCR_TRIGGER_1;
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	up->mcr &= ~UART_MCR_AFE;
	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (!(up->bugs & UART_BUG_NOMSR) &&
			UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	if (up->capabilities & UART_CAP_UUE)
		up->ier |= UART_IER_UUE;
	if (up->capabilities & UART_CAP_RTOIE)
		up->ier |= UART_IER_RTOIE;

	serial_port_out(port, UART_IER, up->ier);

	if (m != vp->m || n != vp->n) {
		set_clk_param(vp, m, n);
		vp->m = m;
		vp->n = n;
	}
	serial_port_out(port, UART_LCR, cval | UART_LCR_DLAB);
	serial_out(up, UART_DLL, quot & 0xff);
	serial_out(up, UART_DLM, quot >> 8 & 0xff);
	serial_port_out(port, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	serial_port_out(port, UART_FCR, fcr);		/* set fcr */

	vlv_set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

static void baylake_setup(struct vlv_hsu_port *vp)
{
	writel(0, (vp->membase + 0x804));
	writel(3, (vp->membase + 0x804));

	vp->m = 9216;
	vp->n = 15625;
	set_clk_param(vp, vp->m, vp->n);
}

static struct vlv_hsu_config port_configs[] = {
	[baylake_0] = {
		.name = "bt_hsu",
		.uartclk = 58982400,
		.idle_delay = 100,
		.setup = baylake_setup,
	},
	[baylake_1] = {
		.name = "gps_hsu",
		.uartclk = 58982400,
		.idle_delay = 100,
		.setup = baylake_setup,
	},
};

static void vlv_hsu_serial_out(struct uart_port *p, int offset, int value)
{
	struct vlv_hsu_port *vp = p->private_data;

	if (offset == UART_LCR)
		vp->last_lcr = value;

	offset <<= p->regshift;
	writeb(value, p->membase + offset);
}

static unsigned int vlv_hsu_serial_in(struct uart_port *p, int offset)
{
	offset <<= p->regshift;

	return readb(p->membase + offset);
}

static void vlv_hsu_serial_out32(struct uart_port *p, int offset, int value)
{
	struct vlv_hsu_port *vp = p->private_data;

	if (offset == UART_LCR)
		vp->last_lcr = value;

	offset <<= p->regshift;
	writel(value, p->membase + offset);
}

static unsigned int vlv_hsu_serial_in32(struct uart_port *p, int offset)
{
	offset <<= p->regshift;

	return readl(p->membase + offset);
}

/* Offset for the DesignWare's UART Status Register. */
#define UART_USR	0x1f

static int vlv_hsu_handle_irq(struct uart_port *p)
{
	struct vlv_hsu_port *vp = p->private_data;
	unsigned int iir = p->serial_in(p, UART_IIR);

	if (serial8250_handle_irq(p, iir)) {
		return 1;
	} else if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
		/* Clear the USR and write the LCR again. */
		(void)p->serial_in(p, UART_USR);
		p->serial_out(p, vp->last_lcr, UART_LCR);

		return 1;
	}

	return 0;
}

static int vlv_hsu_do_suspend(struct pci_dev *pdev)
{
	struct vlv_hsu_port *vp = pci_get_drvdata(pdev);

	serial8250_suspend_port(vp->line);
	return 0;
}

static int vlv_hsu_do_resume(struct pci_dev *pdev)
{
	struct vlv_hsu_port *vp = pci_get_drvdata(pdev);

	serial8250_resume_port(vp->line);
	return 0;
}

static int vlv_hsu_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return vlv_hsu_do_suspend(pdev);
}

static int vlv_hsu_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return vlv_hsu_do_resume(pdev);
}

static int vlv_hsu_runtime_idle(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct vlv_hsu_port *vp = pci_get_drvdata(pdev);

	pm_schedule_suspend(dev, vp->idle_delay);
	return -EBUSY;
}

static int vlv_hsu_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return vlv_hsu_do_suspend(pdev);
}

static int vlv_hsu_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	return vlv_hsu_do_resume(pdev);
}

static const struct dev_pm_ops vlv_hsu_pm_ops = {

	SET_SYSTEM_SLEEP_PM_OPS(vlv_hsu_suspend,
				vlv_hsu_resume)
	SET_RUNTIME_PM_OPS(vlv_hsu_runtime_suspend,
				vlv_hsu_runtime_resume,
				vlv_hsu_runtime_idle)
};

#ifdef CONFIG_DEBUG_FS
static int vlv_hsu_show(struct seq_file *s, void *data)
{
	struct vlv_hsu_port	*vp = data;

	seq_printf(s, "debugfs not implemented yet\n");
	return 0;
}

static int vlv_hsu_open(struct inode *inode, struct file *file)
{
	return single_open(file, vlv_hsu_show, inode->i_private);
}

static const struct file_operations vlv_hsu_operations = {
	.open		= vlv_hsu_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void vlv_hsu_debugfs_init(struct vlv_hsu_port *vp)
{
	vp->debugfs = debugfs_create_dir("hsu", NULL);
	debugfs_create_file(vp->name, S_IFREG | S_IRUGO,
				vp->debugfs, vp, &vlv_hsu_operations);
}

static void vlv_hsu_debugfs_exit(struct vlv_hsu_port *vp)
{
	if (vp->debugfs)
		debugfs_remove_recursive(vp->debugfs);
}
#else
static void vlv_hsu_debugfs_init(struct vlv_hsu_port *vp) { return; }
static void vlv_hsu_debugfs_exit(struct vlv_hsu_port *vp) { return; }
#endif	/* DEBUG_FS */

DEFINE_PCI_DEVICE_TABLE(hsu_port_pci_ids) = {
	{},
};

static int vlv_hsu_port_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct vlv_hsu_port *vp = NULL;
	struct uart_port port = {};
	struct vlv_hsu_config *cfg = &port_configs[id->driver_data];
	int ret;

	dev_info(&pdev->dev,
			"ValleyView HSU serial controller (ID: %04x:%04x)\n",
			pdev->vendor, pdev->device);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	vp = devm_kzalloc(&pdev->dev, sizeof(*vp), GFP_KERNEL);
	if (!vp)
		goto err;

	spin_lock_init(&port.lock);
	memset(&port, 0, sizeof(port));
	port.private_data = vp;
	port.mapbase = pci_resource_start(pdev, 0);
	port.membase =
		ioremap_nocache(port.mapbase, pci_resource_len(pdev, 0));
	port.irq = pdev->irq;
	port.handle_irq = vlv_hsu_handle_irq;
	port.type = PORT_8250;
	port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF | UPF_IOREMAP |
		UPF_FIXED_PORT | UPF_FIXED_TYPE;
	port.dev = &pdev->dev;
	port.iotype = UPIO_MEM32;
	port.serial_in = vlv_hsu_serial_in32;
	port.serial_out = vlv_hsu_serial_out32;
	port.set_termios = vlv_set_termios;
	port.regshift = 2;
	port.uartclk = cfg->uartclk;
	port.fifosize = 64;

	vp->name = cfg->name;
	vp->use_dma = cfg->use_dma;
	vp->wake_gpio = cfg->wake_gpio;
	vp->idle_delay = cfg->idle_delay;
	vp->membase = port.membase;
	vp->irq = port.irq;
	vp->dev = port.dev;
	pci_set_drvdata(pdev, vp);
	if (cfg->setup)
		cfg->setup(vp);

	vp->line = serial8250_register_port(&port);
	if (vp->line < 0)
		goto err;

	vlv_hsu_debugfs_init(vp);
	pm_runtime_put_noidle(&pdev->dev);
	/* pm_runtime_allow(&pdev->dev); */
	return 0;

err:
	devm_kfree(&pdev->dev, vp);
	pci_disable_device(pdev);
	return ret;
}

static void vlv_hsu_port_remove(struct pci_dev *pdev)
{
	struct vlv_hsu_port *vp = pci_get_drvdata(pdev);

	vlv_hsu_debugfs_exit(vp);
	serial8250_unregister_port(vp->line);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pci_disable_device(pdev);
	devm_kfree(&pdev->dev, vp);
}

static struct pci_driver hsu_port_pci_driver = {
	.name =		"VLV HSU serial",
	.id_table =	hsu_port_pci_ids,
	.probe =	vlv_hsu_port_probe,
	.remove =	__devexit_p(vlv_hsu_port_remove),
	.driver = {
		.pm = &vlv_hsu_pm_ops,
	},
};

static int __init vlv_hsu_init(void)
{
	return pci_register_driver(&hsu_port_pci_driver);
}

static void __exit vlv_hsu_exit(void)
{
	pci_unregister_driver(&hsu_port_pci_driver);
}

module_init(vlv_hsu_init);
module_exit(vlv_hsu_exit);

