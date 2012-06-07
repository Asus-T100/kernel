/*
 * Intel Penwell USB OTG transceiver driver
 * Copyright (C) 2009 - 2010, Intel Corporation.
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
 *
 */
/* This driver helps to switch Penwell OTG controller function between host
 * and peripheral. It works with EHCI driver and Penwell client controller
 * driver together.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <asm/intel_scu_ipc.h>
#include "../core/usb.h"

#include <linux/usb/penwell_otg.h>

#define	DRIVER_DESC		"Intel Penwell USB OTG transceiver driver"
#define	DRIVER_VERSION		"July 4, 2010"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Henry Yuan <hang.yuan@intel.com>, Hao Wu <hao.wu@intel.com>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

static const char driver_name[] = "penwell_otg";

static int penwell_otg_probe(struct pci_dev *pdev,
			const struct pci_device_id *id);
static void penwell_otg_remove(struct pci_dev *pdev);
static int penwell_otg_suspend(struct pci_dev *pdev, pm_message_t message);
static int penwell_otg_resume(struct pci_dev *pdev);

static int penwell_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host);
static int penwell_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget);
static int penwell_otg_start_srp(struct otg_transceiver *otg);
static void penwell_otg_mon_bus(void);

static int penwell_otg_msic_write(u16 addr, u8 data);

static const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:
		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:
		return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:
		return "a_wait_bcon";
	case OTG_STATE_A_HOST:
		return "a_host";
	case OTG_STATE_A_SUSPEND:
		return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:
		return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:
		return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:
		return "a_vbus_err";
	case OTG_STATE_B_IDLE:
		return "b_idle";
	case OTG_STATE_B_PERIPHERAL:
		return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:
		return "b_wait_acon";
	case OTG_STATE_B_HOST:
		return "b_host";
	default:
		return "UNDEFINED";
	}
}

static const char *charger_string(enum usb_charger_type charger)
{
	switch (charger) {
	case CHRG_SDP:
		return "Standard Downstream Port";
	case CHRG_CDP:
		return "Charging Downstream Port";
	case CHRG_DCP:
		return "Dedicated Charging Port";
	case CHRG_UNKNOWN:
		return "Unknown";
	default:
		return "Undefined";
	}
}

static struct penwell_otg *the_transceiver;

void penwell_update_transceiver(void)
{
	struct penwell_otg	*pnw = the_transceiver;

	dev_dbg(pnw->dev, "transceiver is updated\n");

	if (!pnw->qwork)
		return ;

	queue_work(pnw->qwork, &pnw->work);
}

static int penwell_otg_set_host(struct otg_transceiver *otg,
					struct usb_bus *host)
{
	otg->host = host;

	return 0;
}

static int penwell_otg_set_peripheral(struct otg_transceiver *otg,
					struct usb_gadget *gadget)
{
	otg->gadget = gadget;

	return 0;
}

static void penwell_otg_set_charger(enum usb_charger_type charger)
{
	struct penwell_otg	*pnw = the_transceiver;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__,
			charger_string(charger));

	switch (charger) {
	case CHRG_SDP:
	case CHRG_DCP:
	case CHRG_CDP:
	case CHRG_ACA:
	case CHRG_UNKNOWN:
		pnw->charging_cap.chrg_type = charger;
		break;
	default:
		dev_warn(pnw->dev, "undefined charger type\n");
		break;
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

static void _penwell_otg_update_chrg_cap(enum usb_charger_type charger,
				unsigned mA)
{
	struct penwell_otg	*pnw = the_transceiver;
	int			flag = 0;
	int			event, retval;

	dev_dbg(pnw->dev, "%s = %s, %d --->\n", __func__,
			charger_string(charger), mA);

	/* Check charger type information */
	if (pnw->charging_cap.chrg_type != charger) {
		if (pnw->charging_cap.chrg_type == CHRG_UNKNOWN ||
			charger == CHRG_UNKNOWN) {
			penwell_otg_set_charger(charger);
		} else
			return;
	} else {
		/* Do nothing if no update for current */
		if (pnw->charging_cap.mA == mA)
			return;
	}

	/* set current */
	switch (pnw->charging_cap.chrg_type) {
	case CHRG_SDP:
		if ((pnw->charging_cap.mA == CHRG_CURR_DISCONN
			|| pnw->charging_cap.mA == CHRG_CURR_SDP_LOW
			|| pnw->charging_cap.mA == CHRG_CURR_SDP_HIGH)
				&& mA == CHRG_CURR_SDP_SUSP) {
			/* SDP event: enter suspend state */
			event = USBCHRG_EVENT_SUSPEND;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_DISCONN
				&& (mA == CHRG_CURR_SDP_LOW
				|| mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: charger connect */
			event = USBCHRG_EVENT_CONNECT;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_SDP_SUSP
				&& (mA == CHRG_CURR_SDP_LOW
				|| mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: resume from suspend state */
			event = USBCHRG_EVENT_RESUME;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_SDP_LOW
				&& mA == CHRG_CURR_SDP_HIGH) {
			/* SDP event: configuration update */
			event = USBCHRG_EVENT_UPDATE;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_SDP_HIGH
				&& mA == CHRG_CURR_SDP_LOW) {
			/* SDP event: configuration update */
			event = USBCHRG_EVENT_UPDATE;
			flag = 1;
		} else
			dev_dbg(pnw->dev, "SDP: no need to update EM\n");
		break;
	case CHRG_DCP:
		if (mA == CHRG_CURR_DCP) {
			/* DCP event: charger connect */
			event = USBCHRG_EVENT_CONNECT;
			flag = 1;
		} else
			dev_dbg(pnw->dev, "DCP: no need to update EM\n");
		break;
	case CHRG_CDP:
		if (pnw->charging_cap.mA == CHRG_CURR_DISCONN
				&& mA == CHRG_CURR_CDP) {
			/* CDP event: charger connect */
			event = USBCHRG_EVENT_CONNECT;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_CDP
				&& mA == CHRG_CURR_CDP_HS) {
			/* CDP event: mode update */
			event = USBCHRG_EVENT_UPDATE;
			flag = 1;
		} else if (pnw->charging_cap.mA == CHRG_CURR_CDP_HS
				&& mA == CHRG_CURR_CDP) {
			/* CDP event: mode update */
			event = USBCHRG_EVENT_UPDATE;
			flag = 1;
		} else
			dev_dbg(pnw->dev, "CDP: no need to update EM\n");
		break;
	case CHRG_UNKNOWN:
		if (mA == CHRG_CURR_DISCONN) {
			/* event: chargers disconnect */
			event = USBCHRG_EVENT_DISCONN;
			flag = 1;
		} else
			dev_dbg(pnw->dev, "UNKNOWN: no need to update EM\n");
		break;
	default:
		break;
	}

	if (flag) {
		pnw->charging_cap.mA = mA;
		pnw->charging_cap.current_event = event;

		/* Notify EM the charging current update */
		dev_dbg(pnw->dev, "Notify EM charging capability change\n");
		dev_dbg(pnw->dev, "%s event = %d mA = %d\n",
			charger_string(pnw->charging_cap.chrg_type), event, mA);

		if (pnw->bc_callback) {
			retval = pnw->bc_callback(pnw->bc_arg, event,
					&pnw->charging_cap);
			if (retval)
				dev_dbg(pnw->dev,
					"bc callback return %d\n", retval);
		}
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

static void penwell_otg_update_chrg_cap(enum usb_charger_type charger,
				unsigned mA)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	spin_lock_irqsave(&pnw->charger_lock, flags);
	_penwell_otg_update_chrg_cap(charger, mA);
	spin_unlock_irqrestore(&pnw->charger_lock, flags);
}

static int penwell_otg_set_power(struct otg_transceiver *otg,
				unsigned mA)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	spin_lock_irqsave(&pnw->charger_lock, flags);

	if (pnw->charging_cap.chrg_type != CHRG_SDP) {
		spin_unlock(&pnw->charger_lock);
		return 0;
	}

	_penwell_otg_update_chrg_cap(CHRG_SDP, mA);

	spin_unlock_irqrestore(&pnw->charger_lock, flags);

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return 0;
}

int penwell_otg_query_charging_cap(struct otg_bc_cap *cap)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (pnw == NULL)
		return -ENODEV;

	if (cap == NULL)
		return -EINVAL;

	spin_lock_irqsave(&pnw->charger_lock, flags);
	cap->chrg_type = pnw->charging_cap.chrg_type;
	cap->mA = pnw->charging_cap.mA;
	cap->current_event = pnw->charging_cap.current_event;
	spin_unlock_irqrestore(&pnw->charger_lock, flags);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(penwell_otg_query_charging_cap);

/* Register/unregister battery driver callback */
void *penwell_otg_register_bc_callback(
	int (*cb)(void *, int, struct otg_bc_cap *), void *arg)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	if (pnw == NULL)
		return pnw;

	spin_lock_irqsave(&pnw->charger_lock, flags);

	if (pnw->bc_callback != NULL)
		dev_dbg(pnw->dev, "callback has already registered\n");

	pnw->bc_callback = cb;
	pnw->bc_arg = arg;
	spin_unlock_irqrestore(&pnw->charger_lock, flags);

	return pnw;
}
EXPORT_SYMBOL_GPL(penwell_otg_register_bc_callback);

int penwell_otg_unregister_bc_callback(void *handler)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	if (pnw == NULL)
		return -ENODEV;

	if (pnw != handler)
		return -EINVAL;

	spin_lock_irqsave(&pnw->charger_lock, flags);
	pnw->bc_callback = NULL;
	pnw->bc_arg = NULL;
	spin_unlock_irqrestore(&pnw->charger_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(penwell_otg_unregister_bc_callback);

/* After probe, it should enable the power of USB PHY */
static void penwell_otg_phy_enable(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			data;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	data = on ? 0x37 : 0x24;

	mutex_lock(&pnw->msic_mutex);

	if (penwell_otg_msic_write(MSIC_VUSB330CNT, data)) {
		mutex_unlock(&pnw->msic_mutex);
		dev_err(pnw->dev, "Fail to enable PHY power\n");
		return;
	}

	mutex_unlock(&pnw->msic_mutex);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

/* A-device drives vbus, controlled through MSIC register */
static int penwell_otg_set_vbus(struct otg_transceiver *otg, bool enabled)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			data;
	int			retval;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, enabled ? "on" : "off");

	data = enabled ? VOTGEN : 0;

	mutex_lock(&pnw->msic_mutex);

	retval = intel_scu_ipc_update_register(MSIC_VOTGCNT, data, VOTGEN);

	if (retval)
		dev_err(pnw->dev, "Fail to set power on OTG Port\n");

	mutex_unlock(&pnw->msic_mutex);

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return retval;
}

static int penwell_otg_ulpi_run(void)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;

	val = readl(pnw->iotg.base + CI_ULPIVP);

	if (val & ULPI_RUN) {
		dev_dbg(pnw->dev, "%s: ULPI command wip\n", __func__);
		return 1;
	}

	dev_dbg(pnw->dev, "%s: ULPI command done\n", __func__);
	return 0;
}

/* io_ops to access ulpi registers */
static int
penwell_otg_ulpi_read(struct intel_mid_otg_xceiv *iotg, u8 reg, u8 *val)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val32 = 0;
	int			count;

	dev_dbg(pnw->dev, "%s - addr 0x%x\n", __func__, reg);

	/* Port = 0 */
	val32 = ULPI_RUN | reg << 16;
	writel(val32, pnw->iotg.base + CI_ULPIVP);

	/* Polling for write operation to complete*/
	count = 10;

	while (count) {
		val32 = readl(pnw->iotg.base + CI_ULPIVP);
		if (val32 & ULPI_RUN) {
			count--;
			udelay(20);
		} else {
			*val = (u8)((val32 & ULPI_DATRD) >> 8);
			dev_dbg(pnw->dev,
				"%s - done data 0x%x\n", __func__, *val);
			return 0;
		}
	}

	dev_dbg(pnw->dev, "%s - timeout\n", __func__);

	return -ETIMEDOUT;

}

static int
penwell_otg_ulpi_write(struct intel_mid_otg_xceiv *iotg, u8 reg, u8 val)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val32 = 0;
	int			count;

	dev_dbg(pnw->dev,
		"%s - addr 0x%x - data 0x%x\n", __func__, reg, val);

	/* Port = 0 */
	val32 = ULPI_RUN | ULPI_RW | reg << 16 | val;
	writel(val32, pnw->iotg.base + CI_ULPIVP);

	/* Polling for write operation to complete*/
	count = 10;

	while (count && penwell_otg_ulpi_run()) {
		count--;
		udelay(20);
	}

	dev_dbg(pnw->dev,
		"%s - %s\n", __func__, count ? "complete" : "timeout");

	return count ? 0 : -ETIMEDOUT;
}

static enum msic_vendor penwell_otg_check_msic(void)
{
	/* Return MSIC_VD_TI directly */
	return MSIC_VD_TI;
}

/* Monitor function check if SRP initial conditions. Use polling on current
 * status for b_ssend_srp, b_se0_srp */
static void penwell_otg_mon_bus(void)
{
	struct penwell_otg *pnw = the_transceiver;
	int count = 5;
	int interval = 300; /* ms */
	u32 val = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);
	pnw->iotg.hsm.b_ssend_srp = 0;
	pnw->iotg.hsm.b_se0_srp = 0;

	while (count) {
		msleep(interval);

		/* Check VBus status */
		val = readl(pnw->iotg.base + CI_OTGSC);
		if (!(val & OTGSC_BSE))
			return;

		val = readl(pnw->iotg.base + CI_PORTSC1);
		if (val & PORTSC_LS)
			return;

		count--;
	}

	pnw->iotg.hsm.b_ssend_srp = 1;
	pnw->iotg.hsm.b_se0_srp = 1;
	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

/* Start SRP function */
static int penwell_otg_start_srp(struct otg_transceiver *otg)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	val = readl(pnw->iotg.base + CI_OTGSC);

	writel((val & ~OTGSC_INTSTS_MASK) | OTGSC_HADP,
				pnw->iotg.base + CI_OTGSC);

	/* Check if the data plus is finished or not */
	msleep(8);
	val = readl(pnw->iotg.base + CI_OTGSC);
	if (val & (OTGSC_HADP | OTGSC_DP))
		dev_dbg(pnw->dev, "DataLine SRP Error\n");

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return 0;
}

/* The timeout callback function to poll the host request flag */
static void penwell_otg_hnp_poll_fn(unsigned long indicator)
{
	struct penwell_otg *pnw = the_transceiver;

	queue_work(pnw->qwork, &pnw->hnp_poll_work);
}

/* stop SOF via bus_suspend */
static void penwell_otg_loc_sof(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct usb_hcd		*hcd;
	int			err;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "suspend" : "resume");

	hcd = bus_to_hcd(pnw->iotg.otg.host);
	if (on)
		err = hcd->driver->bus_resume(hcd);
	else
		err = hcd->driver->bus_suspend(hcd);

	if (err)
		dev_dbg(pnw->dev, "Fail to resume/suspend USB bus - %d\n", err);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

static void penwell_otg_phy_low_power(int on)
{
	struct	penwell_otg	*pnw = the_transceiver;
	u32			val;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	val = readl(pnw->iotg.base + CI_HOSTPC1);
	dev_dbg(pnw->dev, "---> Register CI_HOSTPC1 = %x\n", val);

	if (on) {
		if (val & HOSTPC1_PHCD) {
			dev_dbg(pnw->dev, "already in Low power mode\n");
			return;
		}
		writel(val | HOSTPC1_PHCD, pnw->iotg.base + CI_HOSTPC1);
	} else {
		if (!(val & HOSTPC1_PHCD)) {
			dev_dbg(pnw->dev, "already in Normal mode\n");
			return;
		}
		writel(val & ~HOSTPC1_PHCD, pnw->iotg.base + CI_HOSTPC1);
	}

	val = readl(pnw->iotg.base + CI_HOSTPC1);

	dev_dbg(pnw->dev, "<--- Register CI_HOSTPC1 = %x\n", val);
	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

/* Enable/Disable OTG interrupt */
static void penwell_otg_intr(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	val = readl(pnw->iotg.base + CI_OTGSC);
	if (on) {
		val = val | (OTGSC_INTEN_MASK);
		writel(val, pnw->iotg.base + CI_OTGSC);
	} else {
		val = val & ~(OTGSC_INTEN_MASK);
		writel(val, pnw->iotg.base + CI_OTGSC);
	}
}

/* set HAAR: Hardware Assist Auto-Reset */
static void penwell_otg_HAAR(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	val = readl(pnw->iotg.base + CI_OTGSC);
	if (on)
		writel((val & ~OTGSC_INTSTS_MASK) | OTGSC_HAAR,
				pnw->iotg.base + CI_OTGSC);
	else
		writel((val & ~OTGSC_INTSTS_MASK) & ~OTGSC_HAAR,
				pnw->iotg.base + CI_OTGSC);
}

/* set HABA: Hardware Assist B-Disconnect to A-Connect */
static void penwell_otg_HABA(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32	val;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	val = readl(pnw->iotg.base + CI_OTGSC);
	if (on)
		writel((val & ~OTGSC_INTSTS_MASK) | OTGSC_HABA,
					pnw->iotg.base + CI_OTGSC);
	else
		writel((val & ~OTGSC_INTSTS_MASK) & ~OTGSC_HABA,
					pnw->iotg.base + CI_OTGSC);
}

/* write 8bit msic register */
static int penwell_otg_msic_write(u16 addr, u8 data)
{
	struct penwell_otg	*pnw = the_transceiver;
	int			retval = 0;

	retval = intel_scu_ipc_iowrite8(addr, data);
	if (retval) {
		dev_warn(pnw->dev, "Failed to write MSIC register %x\n", addr);
		return retval;
	}

	return retval;
}

/* USB related register in MSIC can be access via SPI address and ulpi address
 * Access the control register to switch */
static void penwell_otg_msic_spi_access(bool enabled)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			data;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	/* Set ULPI ACCESS MODE */
	data = enabled ? SPIMODE : 0;

	penwell_otg_msic_write(MSIC_ULPIACCESSMODE, data);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

/* USB Battery Charger detection related functions */
/* Data contact detection is the first step for charger detection */
static int penwell_otg_data_contact_detect(void)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			data;
	int			count = 10;
	int			retval = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	/* Enable SPI access */
	penwell_otg_msic_spi_access(true);

	/* Set POWER_CTRL_CLR */
	retval = penwell_otg_msic_write(MSIC_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	/* Set FUNC_CTRL_SET */
	retval = penwell_otg_msic_write(MSIC_FUNCTRLSET, OPMODE0);
	if (retval)
		return retval;

	/* Set FUNC_CTRL_CLR */
	retval = penwell_otg_msic_write(MSIC_FUNCTRLCLR, OPMODE1);
	if (retval)
		return retval;

	/* Set OTG_CTRL_CLR */
	retval = penwell_otg_msic_write(MSIC_OTGCTRLCLR,
					DMPULLDOWN | DPPULLDOWN);
	if (retval)
		return retval;

	/* Set POWER_CTRL_CLR */
	retval = penwell_otg_msic_write(MSIC_PWRCTRLCLR, SWCNTRL);
	if (retval)
		return retval;

	retval = penwell_otg_msic_write(MSIC_VS3SET, DATACONEN | SWUSBDET);
	if (retval)
		return retval;

	dev_dbg(pnw->dev, "Start Polling for Data contact detection!\n");

	while (count) {
		retval = intel_scu_ipc_ioread8(MSIC_PWRCTRL, &data);
		if (retval) {
			dev_warn(pnw->dev, "Failed to read MSIC register\n");
			return retval;
		}

		if (data & DPVSRCEN) {
			dev_dbg(pnw->dev, "Data contact detected!\n");
			return 0;
		}
		count--;
		/* Interval is 50ms */
		msleep(50);
	}

	dev_dbg(pnw->dev, "Data contact Timeout\n");

	retval = penwell_otg_msic_write(MSIC_VS3CLR, DATACONEN | SWUSBDET);
	if (retval)
		return retval;

	udelay(100);

	retval = penwell_otg_msic_write(MSIC_VS3SET, SWUSBDET);
	if (retval)
		return retval;

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return 0;
}

static int penwell_otg_charger_detect(void)
{
	struct penwell_otg		*pnw = the_transceiver;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	msleep(125);

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return 0;
}

static int penwell_otg_charger_type_detect(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	enum usb_charger_type		charger;
	u8				data;
	int				retval;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	retval = penwell_otg_msic_write(MSIC_VS3CLR, DATACONEN);
	if (retval)
		return retval;

	retval = penwell_otg_msic_write(MSIC_PWRCTRLSET, DPWKPUEN | SWCNTRL);
	if (retval)
		return retval;

	retval = penwell_otg_msic_write(MSIC_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	retval = penwell_otg_msic_write(MSIC_OTGCTRLCLR,
					DMPULLDOWN | DPPULLDOWN);
	if (retval)
		return retval;

	msleep(55);

	retval = penwell_otg_msic_write(MSIC_PWRCTRLCLR,
					SWCNTRL | DPWKPUEN | HWDET);
	if (retval)
		return retval;

	msleep(1);

	/* Enable ULPI mode */
	penwell_otg_msic_spi_access(false);

	retval = intel_scu_ipc_ioread8(MSIC_SPWRSRINT1, &data);
	if (retval) {
		dev_warn(pnw->dev, "Failed to read MSIC register\n");
		return retval;
	}

	switch (data & MSIC_SPWRSRINT1_MASK) {
	case SPWRSRINT1_SDP:
		charger = CHRG_SDP;
		break;
	case SPWRSRINT1_DCP:
		charger = CHRG_DCP;
		break;
	case SPWRSRINT1_CDP:
		charger = CHRG_CDP;
		break;
	default:
		charger = CHRG_UNKNOWN;
		break;
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return charger;
}

void penwell_otg_nsf_msg(unsigned long indicator)
{
	switch (indicator) {
	case 2:
	case 4:
	case 6:
	case 7:
		dev_warn(the_transceiver->dev,
			"NSF-%lu - deivce not responding\n", indicator);
		break;
	case 3:
		dev_warn(the_transceiver->dev,
			"NSF-%lu - deivce not supported\n", indicator);
		break;
	default:
		dev_warn(the_transceiver->dev,
			"Do not have this kind of NSF\n");
		break;
	}
}

/* The timeout callback function to set time out bit */
static void penwell_otg_timer_fn(unsigned long indicator)
{
	struct penwell_otg *pnw = the_transceiver;

	*(int *)indicator = 1;

	dev_dbg(pnw->dev, "kernel timer - timeout\n");

	queue_work(pnw->qwork, &pnw->work);
}

/* kernel timer used for OTG timing */
static void penwell_otg_add_timer(enum penwell_otg_timer_type timers)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	unsigned long			j = jiffies;
	unsigned long			data, time;

	switch (timers) {
	case TA_WAIT_VRISE_TMR:
		iotg->hsm.a_wait_vrise_tmout = 0;
		data = (unsigned long)&iotg->hsm.a_wait_vrise_tmout;
		time = TA_WAIT_VRISE;
		dev_dbg(pnw->dev,
			"Add timer TA_WAIT_VRISE = %d\n", TA_WAIT_VRISE);
		break;
	case TA_WAIT_BCON_TMR:
		iotg->hsm.a_wait_bcon_tmout = 0;
		data = (unsigned long)&iotg->hsm.a_wait_bcon_tmout;
		time = TA_WAIT_BCON;
		dev_dbg(pnw->dev,
			"Add timer TA_WAIT_BCON = %d\n", TA_WAIT_BCON);
		break;
	case TA_AIDL_BDIS_TMR:
		iotg->hsm.a_aidl_bdis_tmout = 0;
		data = (unsigned long)&iotg->hsm.a_aidl_bdis_tmout;
		time = TA_AIDL_BDIS;
		dev_dbg(pnw->dev,
			"Add timer TA_AIDL_BDIS = %d\n", TA_AIDL_BDIS);
		break;
	case TA_BIDL_ADIS_TMR:
		iotg->hsm.a_bidl_adis_tmout = 0;
		iotg->hsm.a_bidl_adis_tmr = 1;
		data = (unsigned long)&iotg->hsm.a_bidl_adis_tmout;
		time = TA_BIDL_ADIS;
		dev_dbg(pnw->dev,
			"Add timer TA_BIDL_ADIS = %d\n", TA_BIDL_ADIS);
		break;
	case TA_WAIT_VFALL_TMR:
		iotg->hsm.a_wait_vfall_tmout = 0;
		data = (unsigned long)&iotg->hsm.a_wait_vfall_tmout;
		time = TA_WAIT_VFALL;
		dev_dbg(pnw->dev,
			"Add timer TA_WAIT_VFALL = %d\n", TA_WAIT_VFALL);
		break;
	case TB_ASE0_BRST_TMR:
		iotg->hsm.b_ase0_brst_tmout = 0;
		data = (unsigned long)&iotg->hsm.b_ase0_brst_tmout;
		time = TB_ASE0_BRST;
		dev_dbg(pnw->dev,
			"Add timer TB_ASE0_BRST = %d\n", TB_ASE0_BRST);
		break;
	case TB_SRP_FAIL_TMR:
		iotg->hsm.b_srp_fail_tmout = 0;
		iotg->hsm.b_srp_fail_tmr = 1;
		data = (unsigned long)&iotg->hsm.b_srp_fail_tmout;
		time = TB_SRP_FAIL;
		dev_dbg(pnw->dev,
			"Add timer TB_SRP_FAIL = %d\n", TB_SRP_FAIL);
		break;
	default:
		dev_dbg(pnw->dev,
			"unkown timer, can not enable such timer\n");
		return;
	}

	init_timer(&pnw->hsm_timer);

	pnw->hsm_timer.data = data;
	pnw->hsm_timer.function = penwell_otg_timer_fn;
	pnw->hsm_timer.expires = j + time * HZ / 1000; /* milliseconds */

	add_timer(&pnw->hsm_timer);
}

static inline void penwell_otg_del_timer(enum penwell_otg_timer_type timers)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	switch (timers) {
	case TA_BIDL_ADIS_TMR:
		iotg->hsm.a_bidl_adis_tmr = 0;
		break;
	case TB_SRP_FAIL_TMR:
		iotg->hsm.b_srp_fail_tmr = 0;
		break;
	default:
		break;
	}

	dev_dbg(pnw->dev, "state machine timer deleted\n");
	del_timer_sync(&pnw->hsm_timer);
}

static void reset_otg(void)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;
	int			delay_time = 1000;

	dev_dbg(pnw->dev, "reseting OTG controller ...\n");
	val = readl(pnw->iotg.base + CI_USBCMD);
	writel(val | USBCMD_RST, pnw->iotg.base + CI_USBCMD);
	do {
		udelay(100);
		if (!delay_time--)
			dev_dbg(pnw->dev, "reset timeout\n");
		val = readl(pnw->iotg.base + CI_USBCMD);
		val &= USBCMD_RST;
	} while (val != 0);
	dev_dbg(pnw->dev, "reset done.\n");
}

static void set_host_mode(void)
{
	u32	val;

	reset_otg();
	val = readl(the_transceiver->iotg.base + CI_USBMODE);
	val = (val & (~USBMODE_CM)) | USBMODE_HOST;
	writel(val, the_transceiver->iotg.base + CI_USBMODE);
}

static void set_client_mode(void)
{
	u32	val;

	reset_otg();
	val = readl(the_transceiver->iotg.base + CI_USBMODE);
	val = (val & (~USBMODE_CM)) | USBMODE_DEVICE;
	writel(val, the_transceiver->iotg.base + CI_USBMODE);
}

static void init_hsm(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	u32				val32;

	/* read OTGSC after reset */
	val32 = readl(iotg->base + CI_OTGSC);
	dev_dbg(pnw->dev,
		"%s: OTGSC init value = 0x%x\n", __func__, val32);

	/* set init state */
	if (val32 & OTGSC_ID) {
		iotg->hsm.id = ID_B;
		iotg->otg.default_a = 0;
		set_client_mode();
		iotg->otg.state = OTG_STATE_B_IDLE;
	} else {
		iotg->hsm.id = ID_A;
		iotg->otg.default_a = 1;
		set_host_mode();
		iotg->otg.state = OTG_STATE_A_IDLE;
	}

	/* set session indicator */
	if (val32 & OTGSC_BSE)
		iotg->hsm.b_sess_end = 1;
	if (val32 & OTGSC_BSV)
		iotg->hsm.b_sess_vld = 1;
	if (val32 & OTGSC_ASV)
		iotg->hsm.a_sess_vld = 1;
	if (val32 & OTGSC_AVV)
		iotg->hsm.a_vbus_vld = 1;

	/* default user is not request the bus */
	iotg->hsm.a_bus_req = 1;
	iotg->hsm.a_bus_drop = 0;
	/* init hsm means power_up case */
	iotg->hsm.power_up = 1;
	/* defautly don't request bus as B device */
	iotg->hsm.b_bus_req = 0;
	/* no system error */
	iotg->hsm.a_clr_err = 0;

	penwell_otg_phy_low_power(1);
}

static void update_hsm(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	u32				val32;

	/* read OTGSC */
	val32 = readl(iotg->base + CI_OTGSC);
	dev_dbg(pnw->dev,
		"%s OTGSC current value = 0x%x\n", __func__, val32);

	iotg->hsm.id = !!(val32 & OTGSC_ID) ? ID_B : ID_A;
	iotg->hsm.b_sess_end = !!(val32 & OTGSC_BSE);
	iotg->hsm.b_sess_vld = !!(val32 & OTGSC_BSV);
	iotg->hsm.a_sess_vld = !!(val32 & OTGSC_ASV);
	iotg->hsm.a_vbus_vld = !!(val32 & OTGSC_AVV);
}

static irqreturn_t otg_irq(int irq, void *_dev)
{
	struct penwell_otg		*pnw = _dev;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				flag = 0;
	u32				int_sts, int_en, int_mask = 0;

	/* Check VBUS/SRP interrup */
	int_sts = readl(pnw->iotg.base + CI_OTGSC);
	int_en = (int_sts & OTGSC_INTEN_MASK) >> 8;
	int_mask = int_sts & int_en;

	if (int_mask == 0)
		return IRQ_NONE;

	if (int_mask) {
		dev_dbg(pnw->dev,
			"OTGSC = 0x%x, mask =0x%x\n", int_sts, int_mask);

		/* FIXME: if ACA/ID interrupt is enabled, */
		if (int_mask & OTGSC_IDIS) {
			iotg->hsm.id = (int_sts & OTGSC_ID) ? ID_B : ID_A;
			flag = 1;
			dev_dbg(pnw->dev, "%s: id change int = %d\n",
						__func__, iotg->hsm.id);
		}
		if (int_mask & OTGSC_DPIS) {
			iotg->hsm.a_srp_det = (int_sts & OTGSC_DPS) ? 1 : 0;
			flag = 1;
			dev_dbg(pnw->dev, "%s: data pulse int = %d\n",
						__func__, iotg->hsm.a_srp_det);
		}
		if (int_mask & OTGSC_BSEIS) {
			iotg->hsm.b_sess_end = (int_sts & OTGSC_BSE) ? 1 : 0;
			flag = 1;
			dev_dbg(pnw->dev, "%s: b sess end int = %d\n",
						__func__, iotg->hsm.b_sess_end);
		}
		if (int_mask & OTGSC_BSVIS) {
			iotg->hsm.b_sess_vld = (int_sts & OTGSC_BSV) ? 1 : 0;
			flag = 1;
			dev_dbg(pnw->dev, "%s: b sess valid int = %d\n",
						__func__, iotg->hsm.b_sess_vld);
		}
		if (int_mask & OTGSC_ASVIS) {
			iotg->hsm.a_sess_vld = (int_sts & OTGSC_ASV) ? 1 : 0;
			flag = 1;
			dev_dbg(pnw->dev, "%s: a sess valid int = %d\n",
						__func__, iotg->hsm.a_sess_vld);
		}
		if (int_mask & OTGSC_AVVIS) {
			iotg->hsm.a_vbus_vld = (int_sts & OTGSC_AVV) ? 1 : 0;
			flag = 1;
			dev_dbg(pnw->dev, "%s: a vbus valid int = %d\n",
						__func__, iotg->hsm.a_vbus_vld);
		}

		writel((int_sts & ~OTGSC_INTSTS_MASK) | int_mask,
				pnw->iotg.base + CI_OTGSC);
	}

	if (flag)
		penwell_update_transceiver();

	return IRQ_HANDLED;
}

static int penwell_otg_iotg_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = data;
	int				flag = 0;

	if (iotg == NULL)
		return NOTIFY_BAD;

	if (pnw == NULL)
		return NOTIFY_BAD;

	switch (action) {
	case MID_OTG_NOTIFY_CONNECT:
		dev_dbg(pnw->dev, "PNW OTG Notify Connect Event\n");
		if (iotg->otg.default_a == 1)
			iotg->hsm.b_conn = 1;
		else
			iotg->hsm.a_conn = 1;
		flag = 1;
		break;
	case MID_OTG_NOTIFY_DISCONN:
		dev_dbg(pnw->dev, "PNW OTG Notify Disconnect Event\n");
		if (iotg->otg.default_a == 1)
			iotg->hsm.b_conn = 0;
		else
			iotg->hsm.a_conn = 0;
		flag = 1;
		break;
	case MID_OTG_NOTIFY_HSUSPEND:
		dev_dbg(pnw->dev, "PNW OTG Notify Host Bus suspend Event\n");
		if (iotg->otg.default_a == 1)
			iotg->hsm.a_bus_req = 0;
		else
			iotg->hsm.b_bus_req = 0;
		flag = 1;
		break;
	case MID_OTG_NOTIFY_HRESUME:
		dev_dbg(pnw->dev, "PNW OTG Notify Host Bus resume Event\n");
		if (iotg->otg.default_a == 1)
			iotg->hsm.a_bus_req = 1;
		flag = 1;
		break;
	case MID_OTG_NOTIFY_CSUSPEND:
		dev_dbg(pnw->dev, "PNW OTG Notify Client Bus suspend Event\n");
		flag = 0;
		break;
	case MID_OTG_NOTIFY_CRESUME:
		dev_dbg(pnw->dev, "PNW OTG Notify Client Bus resume Event\n");
		flag = 0;
		break;
	case MID_OTG_NOTIFY_HOSTADD:
		dev_dbg(pnw->dev, "PNW OTG Nofity Host Driver Add\n");
		flag = 1;
		break;
	case MID_OTG_NOTIFY_HOSTREMOVE:
		dev_dbg(pnw->dev, "PNW OTG Nofity Host Driver remove\n");
		flag = 1;
		break;
	case MID_OTG_NOTIFY_CLIENTADD:
		dev_dbg(pnw->dev, "PNW OTG Nofity Client Driver Add\n");
		flag = 1;
		break;
	case MID_OTG_NOTIFY_CLIENTREMOVE:
		dev_dbg(pnw->dev, "PNW OTG Nofity Client Driver remove\n");
		flag = 1;
		break;
	case MID_OTG_NOTIFY_CLIENTFS:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Client FullSpeed\n");
		penwell_otg_update_chrg_cap(CHRG_CDP, CHRG_CURR_CDP);
		flag = 0;
		break;
	case MID_OTG_NOTIFY_CLIENTHS:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Client HighSpeed\n");
		penwell_otg_update_chrg_cap(CHRG_CDP, CHRG_CURR_CDP_HS);
		flag = 0;
		break;
	default:
		dev_dbg(pnw->dev, "PNW OTG Nofity unknown notify message\n");
		return NOTIFY_DONE;
	}

	if (flag)
		penwell_update_transceiver();

	return NOTIFY_OK;
}


static void penwell_otg_hnp_poll_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	struct usb_device		*udev;
	unsigned long			j = jiffies;
	int				err = 0;
	u8				data;

	if (iotg->otg.host && iotg->otg.host->root_hub) {
		udev = iotg->otg.host->root_hub->children[0];
	} else {
		dev_dbg(pnw->dev, "no host or root_hub registered\n");
		return;
	}

	if (iotg->otg.state != OTG_STATE_A_HOST
		&& iotg->otg.state != OTG_STATE_B_HOST)
		return;

	if (!udev) {
		dev_dbg(pnw->dev,
			"no usb dev connected, stop HNP polling\n");
		return;
	}

	/* get host request flag from connected USB device */
	err = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		USB_REQ_GET_STATUS, USB_DIR_IN, 0, 0xF000, &data, 1, 5000);

	if (err < 0) {
		dev_warn(pnw->dev,
			"ERR in HNP polling = %d, stop HNP polling\n", err);
		return ;
	}

	if (data & HOST_REQUEST_FLAG) {
		/* set a_bus_req = 0 */
		if (iotg->hsm.id == ID_B) {
			dev_dbg(pnw->dev,
				"Device B host - start HNP - b_bus_req = 0\n");
			iotg->hsm.b_bus_req = 0;
		} else if (iotg->hsm.id == ID_A) {
			dev_dbg(pnw->dev,
				"Device A host - start HNP - a_bus_req = 0\n");
			iotg->hsm.a_bus_req = 0;
		}
		penwell_update_transceiver();
	} else {
		pnw->hnp_poll_timer.data = 1;
		pnw->hnp_poll_timer.function = penwell_otg_hnp_poll_fn;
		pnw->hnp_poll_timer.expires = j + THOS_REQ_POL * HZ / 1000;
		add_timer(&pnw->hnp_poll_timer);

		dev_dbg(pnw->dev, "HNP Polling - continue\n");
	}
}

static void penwell_otg_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = container_of(work,
					struct penwell_otg, work);
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	struct otg_hsm			*hsm = &iotg->hsm;
	enum usb_charger_type		charger_type;
	int				retval;

	dev_dbg(pnw->dev,
		"old state = %s\n", state_string(iotg->otg.state));

	pm_runtime_get_sync(pnw->dev);

	switch (iotg->otg.state) {
	case OTG_STATE_UNDEFINED:
	case OTG_STATE_B_IDLE:
		if (hsm->id == ID_A || hsm->id == ID_ACA_A) {
			/* Move to A_IDLE state, ID changes */

			/* Delete current timer */
			penwell_otg_del_timer(TB_SRP_FAIL_TMR);

			iotg->otg.default_a = 1;
			hsm->a_srp_det = 0;
			set_host_mode();
			penwell_otg_phy_low_power(0);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (hsm->b_adp_sense_tmout) {
			hsm->b_adp_sense_tmout = 0;
		} else if (hsm->b_srp_fail_tmout) {
			hsm->b_srp_fail_tmr = 0;
			hsm->b_srp_fail_tmout = 0;
			hsm->b_bus_req = 0;
			penwell_otg_nsf_msg(6);

			penwell_update_transceiver();
		} else if (hsm->b_sess_vld) {
			/* Check it is caused by ACA attachment */
			if (hsm->id == ID_ACA_B) {
				/* in this case, update current limit*/
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

				/* make sure PHY low power state */
				penwell_otg_phy_low_power(1);
				break;
			} else if (hsm->id == ID_ACA_C) {
				/* in this case, update current limit*/
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			}

			/* Clear power_up */
			hsm->power_up = 0;

			/* Move to B_PERIPHERAL state, Session Valid */

			/* Delete current timer */
			penwell_otg_del_timer(TB_SRP_FAIL_TMR);

			hsm->b_sess_end = 0;
			hsm->a_bus_suspend = 0;

			/* Start USB Battery charger detection flow */

			mutex_lock(&pnw->msic_mutex);
			/* Enable data contact detection */
			penwell_otg_data_contact_detect();
			/* Enable charger detection functionality */
			penwell_otg_charger_detect();
			retval = penwell_otg_charger_type_detect();
			mutex_unlock(&pnw->msic_mutex);
			if (retval < 0) {
				dev_warn(pnw->dev, "Charger detect failure\n");
				break;
			} else
				charger_type = retval;

			if (charger_type == CHRG_DCP) {
				dev_info(pnw->dev, "DCP detected\n");

				/* DCP: set charger type, current, notify EM */
				penwell_otg_update_chrg_cap(CHRG_DCP,
							CHRG_CURR_DCP);
				penwell_otg_phy_low_power(1);
				break;

			} else if (charger_type == CHRG_CDP) {
				dev_info(pnw->dev, "CDP detected\n");

				/* CDP: set charger type, current, notify EM */
				penwell_otg_update_chrg_cap(CHRG_CDP,
							CHRG_CURR_CDP);

				if (iotg->start_peripheral) {
					iotg->start_peripheral(iotg);
				} else {
					dev_dbg(pnw->dev,
						"client driver not support\n");
					break;
				}
			} else if (charger_type == CHRG_SDP) {
				dev_info(pnw->dev, "SDP detected\n");

				/* SDP: set charger type */
				penwell_otg_update_chrg_cap(CHRG_SDP,
							pnw->charging_cap.mA);

				if (iotg->start_peripheral) {
					iotg->start_peripheral(iotg);
				} else {
					dev_dbg(pnw->dev,
						"client driver not support\n");
					break;
				}
			} else if (charger_type == CHRG_UNKNOWN) {
				dev_info(pnw->dev, "Unknown Charger Found\n");

				/* Unknown: set charger type */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
							pnw->charging_cap.mA);
				penwell_otg_phy_low_power(1);
			}

			iotg->otg.state = OTG_STATE_B_PERIPHERAL;

		} else if ((hsm->b_bus_req || hsm->power_up ||
				hsm->adp_change) && !hsm->b_srp_fail_tmr) {

			penwell_otg_mon_bus();

			if (hsm->b_ssend_srp && hsm->b_se0_srp) {

				hsm->power_up = 0;
				hsm->adp_change = 0;

				/* clear the PHCD before start srp */
				penwell_otg_phy_low_power(0);

				/* Start SRP */
				if (pnw->iotg.otg.start_srp)
					pnw->iotg.otg.start_srp(&pnw->iotg.otg);
				penwell_otg_add_timer(TB_SRP_FAIL_TMR);

				/* reset PHY low power mode here */
				penwell_otg_phy_low_power(1);
			} else {
				hsm->b_bus_req = 0;
				dev_info(pnw->dev,
					"BUS is active, try SRP later\n");
			}
		} else if (!hsm->b_sess_vld && hsm->id == ID_B) {
			/* Notify EM charger remove event */
			penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
						CHRG_CURR_DISCONN);
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		/* FIXME: Check if ID_ACA_A event will happened in this state */
		if (hsm->id == ID_A) {
			iotg->otg.default_a = 1;
			hsm->a_srp_det = 0;

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			set_host_mode();
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (!hsm->b_sess_vld || hsm->id == ID_ACA_B) {
			/* Move to B_IDLE state, VBUS off/ACA */

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			else if (hsm->id == ID_B) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
							CHRG_CURR_DISCONN);
			}

			hsm->b_hnp_enable = 0;

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_B_IDLE;
		} else if (hsm->b_bus_req && hsm->b_hnp_enable
				&& hsm->a_bus_suspend) {

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			penwell_otg_HAAR(1);
			hsm->a_conn = 0;
			hsm->a_bus_resume = 0;

			if (iotg->start_host) {
				iotg->start_host(iotg);
				hsm->test_device = 0;
				iotg->otg.state = OTG_STATE_B_WAIT_ACON;
			} else
				dev_dbg(pnw->dev, "host driver not loaded.\n");

			penwell_otg_add_timer(TB_ASE0_BRST_TMR);
		} else if (hsm->id == ID_ACA_C) {
			/* Make sure current limit updated */
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);
#if 0
		} else if (hsm->id == ID_B) {
			if (iotg->otg.set_power)
				iotg->otg.set_power(&iotg->otg, 100);
#endif
		}
		break;

	case OTG_STATE_B_WAIT_ACON:
		if (hsm->id == ID_A) {
			/* Move to A_IDLE state, ID changes */

			/* Delete current timer */
			penwell_otg_del_timer(TB_ASE0_BRST_TMR);

			iotg->otg.default_a = 1;
			hsm->a_srp_det = 0;

			penwell_otg_HAAR(0);
			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			set_host_mode();
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			iotg->hsm.a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (!hsm->b_sess_vld || hsm->id == ID_ACA_B) {
			/* Move to B_IDLE state, VBUS off/ACA */

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			else if (hsm->id == ID_B) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
							CHRG_CURR_DISCONN);
			}

			/* Delete current timer */
			penwell_otg_del_timer(TB_ASE0_BRST_TMR);

			hsm->b_hnp_enable = 0;
			hsm->b_bus_req = 0;
			penwell_otg_HAAR(0);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			set_client_mode();
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_B_IDLE;
		} else if (hsm->a_conn) {
			/* Move to B_HOST state, A connected */

			/* Delete current timer */
			penwell_otg_del_timer(TB_ASE0_BRST_TMR);

			penwell_otg_HAAR(0);
			iotg->otg.state = OTG_STATE_B_HOST;
			penwell_update_transceiver();
		} else if (hsm->a_bus_resume || hsm->b_ase0_brst_tmout) {
			/* Move to B_HOST state, A connected */

			/* Delete current timer */
			penwell_otg_del_timer(TB_ASE0_BRST_TMR);

			penwell_otg_HAAR(0);
			penwell_otg_nsf_msg(7);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			hsm->a_bus_suspend = 0;
			hsm->b_bus_req = 0;

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev, "client driver not loaded\n");

			iotg->otg.state = OTG_STATE_B_PERIPHERAL;
		} else if (hsm->id == ID_ACA_C) {
			/* Make sure current limit updated */
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);
		} else if (hsm->id == ID_B) {
#if 0
			/* only set 2mA due to client function stopped */
			if (iotg->otg.set_power)
				iotg->otg.set_power(&iotg->otg, 2);
#endif
		}
		break;

	case OTG_STATE_B_HOST:
		if (hsm->id == ID_A) {
			iotg->otg.default_a = 1;
			hsm->a_srp_det = 0;

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			set_host_mode();
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (!hsm->b_sess_vld || hsm->id == ID_ACA_B) {
			/* Move to B_IDLE state, VBUS off/ACA */

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			else if (hsm->id == ID_B) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
							CHRG_CURR_DISCONN);
			}

			hsm->b_hnp_enable = 0;
			hsm->b_bus_req = 0;

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			set_client_mode();
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_B_IDLE;
		} else if (!hsm->b_bus_req || !hsm->a_conn
					|| hsm->test_device) {
			hsm->b_bus_req = 0;

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			hsm->a_bus_suspend = 0;

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
						"client driver not loaded.\n");

			iotg->otg.state = OTG_STATE_B_PERIPHERAL;
		} else if (hsm->id == ID_ACA_C) {
			/* Make sure current limit updated */
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);
		}
		break;

	case OTG_STATE_A_IDLE:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B) {
			pnw->iotg.otg.default_a = 0;
			hsm->b_bus_req = 0;

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

			set_client_mode();
			msleep(5);
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_B_IDLE;
			penwell_update_transceiver();
		} else if (hsm->id == ID_ACA_A) {

			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			if (hsm->power_up)
				hsm->power_up = 0;

			if (hsm->adp_change)
				hsm->adp_change = 0;

			if (hsm->a_srp_det)
				hsm->a_srp_det = 0;

			hsm->b_conn = 0;
			hsm->hnp_poll_enable = 0;

			if (iotg->start_host)
				iotg->start_host(iotg);
			else {
				dev_dbg(pnw->dev, "host driver not loaded.\n");
				break;
			}
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (!hsm->a_bus_drop && (hsm->power_up || hsm->a_bus_req
				|| hsm->a_srp_det || hsm->adp_change)) {
			/* power up / adp changes / srp detection should be
			 * cleared at once after handled. */
			if (hsm->power_up)
				hsm->power_up = 0;

			if (hsm->adp_change)
				hsm->adp_change = 0;

			if (hsm->a_srp_det)
				hsm->a_srp_det = 0;

			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, true);

			penwell_otg_add_timer(TA_WAIT_VRISE_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VRISE;

			penwell_update_transceiver();
		} else if (hsm->b_sess_end || hsm->a_sess_vld ||
				hsm->a_srp_det || !hsm->b_sess_vld) {
			hsm->a_srp_det = 0;
			dev_dbg(pnw->dev,
				"reconfig...PHCD bit for PHY low power mode\n");
			penwell_otg_phy_low_power(1);
		}
		break;

	case OTG_STATE_A_WAIT_VRISE:
		if (hsm->a_bus_drop ||
				hsm->id == ID_B || hsm->id == ID_ACA_B) {
			/* Move to A_WAIT_VFALL, over current/user request */

			/* Delete current timer */
			penwell_otg_del_timer(TA_WAIT_VRISE_TMR);

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (hsm->a_vbus_vld || hsm->a_wait_vrise_tmout
						|| hsm->id == ID_ACA_A) {
			/* Move to A_WAIT_BCON state, a vbus vld */
			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_WAIT_VRISE_TMR);

			if (!hsm->a_vbus_vld) {
				/* Turn off VBUS */
				if (iotg->otg.set_vbus)
					iotg->otg.set_vbus(&iotg->otg, false);
				penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
				iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
				break;
			}

			if (hsm->id == ID_ACA_A) {
				if (iotg->otg.set_vbus)
					iotg->otg.set_vbus(&iotg->otg, false);

				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			}

			hsm->a_bus_req = 1;
			hsm->b_conn = 0;
			hsm->hnp_poll_enable = 0;

			if (iotg->start_host) {
				dev_dbg(pnw->dev, "host_ops registered!\n");
				iotg->start_host(iotg);
			} else {
				dev_dbg(pnw->dev, "host driver not loaded.\n");
				break;
			}

			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B || hsm->a_bus_drop) {
			/* Move to A_WAIT_VFALL state, user request */

			/* Delete current timer and clear flags for B-Device */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			hsm->b_bus_req = 0;

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state, over-current detected */

			/* Delete current timer and disable host function */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS and enter PHY low power mode */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_phy_low_power(1);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (hsm->b_conn) {
			/* Move to A_HOST state, device connected */

			/* Delete current timer and disable host function */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			iotg->otg.state = OTG_STATE_A_HOST;

			if (!hsm->a_bus_req)
				hsm->a_bus_req = 1;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
		}
		break;

	case OTG_STATE_A_HOST:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B || hsm->a_bus_drop) {
			/* Move to A_WAIT_VFALL state, timeout/user request */
			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state */

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_phy_low_power(1);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (!hsm->a_bus_req) {
			/* Move to A_SUSPEND state */

			penwell_otg_loc_sof(0);

			if (iotg->otg.host->b_hnp_enable) {
				/* According to Spec 7.1.5 */
				penwell_otg_add_timer(TA_AIDL_BDIS_TMR);
			}

			iotg->otg.state = OTG_STATE_A_SUSPEND;
		} else if (!hsm->b_conn) {
			hsm->hnp_poll_enable = 0;
			/* add kernel timer */
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
		}
		break;

	case OTG_STATE_A_SUSPEND:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B ||
				hsm->a_bus_drop || hsm->a_aidl_bdis_tmout) {
			/* Move to A_WAIT_VFALL state, timeout/user request */

			/* Delete current timer and clear HW assist */
			if (hsm->a_aidl_bdis_tmout)
				hsm->a_aidl_bdis_tmout = 0;
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state, Over-current */

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
			penwell_otg_phy_low_power(1);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (!hsm->b_conn && !pnw->iotg.otg.host->b_hnp_enable) {
			/* Move to A_WAIT_BCON */

			/* delete current timer */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			/* add kernel timer */
			penwell_otg_add_timer(TA_WAIT_BCON_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (!hsm->b_conn && pnw->iotg.otg.host->b_hnp_enable) {
			/* Move to A_PERIPHERAL state, HNP */

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			if (iotg->stop_host)
				iotg->stop_host(iotg);
			else
				dev_dbg(pnw->dev,
					"host driver has been removed.\n");

			hsm->b_bus_suspend = 0;

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
						"client driver not loaded.\n");

			penwell_otg_add_timer(TA_BIDL_ADIS_TMR);
			iotg->otg.state = OTG_STATE_A_PERIPHERAL;
		} else if (hsm->a_bus_req) {
			/* Move to A_HOST state, user request */

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			penwell_otg_loc_sof(1);
			iotg->otg.state = OTG_STATE_A_HOST;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
		}
		break;
	case OTG_STATE_A_PERIPHERAL:
		if (hsm->id == ID_B || hsm->a_bus_drop) {
			/* Move to A_WAIT_VFALL state */

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_BIDL_ADIS_TMR);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
			set_host_mode();

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state, over-current detected */

			/* Delete current timer and disable client function */
			penwell_otg_del_timer(TA_BIDL_ADIS_TMR);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			/* Turn off the VBUS and enter PHY low power mode */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (hsm->a_bidl_adis_tmout) {
			/* Move to A_WAIT_BCON state */
			hsm->a_bidl_adis_tmr = 0;

			/* Disable client function and switch to host mode */
			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			hsm->hnp_poll_enable = 0;
			hsm->b_conn = 0;

			if (iotg->start_host)
				iotg->start_host(iotg);
			else
				dev_dbg(pnw->dev,
						"host driver not loaded.\n");

			penwell_otg_add_timer(TA_WAIT_BCON_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (!hsm->b_bus_suspend && hsm->a_bidl_adis_tmr) {
			/* Client report suspend state end, delete timer */
			penwell_otg_del_timer(TA_BIDL_ADIS_TMR);
		} else if (hsm->b_bus_suspend && !hsm->a_bidl_adis_tmr) {
			/* Client report suspend state start, start timer */
			if (!timer_pending(&pnw->hsm_timer))
				penwell_otg_add_timer(TA_BIDL_ADIS_TMR);
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			if (iotg->otg.set_vbus)
				iotg->otg.set_vbus(&iotg->otg, false);
		}
		break;
	case OTG_STATE_A_VBUS_ERR:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B ||
			hsm->id == ID_ACA_A || hsm->a_bus_drop ||
						hsm->a_clr_err) {
			if (hsm->a_clr_err)
				hsm->a_clr_err = 0;

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		if (hsm->a_wait_vfall_tmout) {
			hsm->a_srp_det = 0;

			/* Move to A_IDLE state, vbus falls */
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		}
		break;
	default:
		;
	}

	pm_runtime_put_sync(pnw->dev);

	dev_dbg(pnw->dev,
			"new state = %s\n", state_string(iotg->otg.state));
}

static ssize_t
show_registers(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg	*pnw = the_transceiver;
	char			*next;
	unsigned		size;
	unsigned		t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size,
		"\n"
		"USBCMD = 0x%08x\n"
		"USBSTS = 0x%08x\n"
		"USBINTR = 0x%08x\n"
		"ASYNCLISTADDR = 0x%08x\n"
		"PORTSC1 = 0x%08x\n"
		"HOSTPC1 = 0x%08x\n"
		"OTGSC = 0x%08x\n"
		"USBMODE = 0x%08x\n",
		readl(pnw->iotg.base + 0x30),
		readl(pnw->iotg.base + 0x34),
		readl(pnw->iotg.base + 0x38),
		readl(pnw->iotg.base + 0x48),
		readl(pnw->iotg.base + 0x74),
		readl(pnw->iotg.base + 0xb4),
		readl(pnw->iotg.base + 0xf4),
		readl(pnw->iotg.base + 0xf8)
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(registers, S_IRUGO, show_registers, NULL);

static ssize_t
show_hsm(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	char				*next;
	unsigned			size, t;

	next = buf;
	size = PAGE_SIZE;

	if (iotg->otg.host)
		iotg->hsm.a_set_b_hnp_en = iotg->otg.host->b_hnp_enable;

	if (iotg->otg.gadget)
		iotg->hsm.b_hnp_enable = iotg->otg.gadget->b_hnp_enable;

	t = scnprintf(next, size,
		"\n"
		"current state = %s\n"
		"a_bus_resume = \t%d\n"
		"a_bus_suspend = \t%d\n"
		"a_conn = \t%d\n"
		"a_sess_vld = \t%d\n"
		"a_srp_det = \t%d\n"
		"a_vbus_vld = \t%d\n"
		"b_bus_suspend = \t%d\n"
		"b_conn = \t%d\n"
		"b_se0_srp = \t%d\n"
		"b_ssend_srp = \t%d\n"
		"b_sess_end = \t%d\n"
		"b_sess_vld = \t%d\n"
		"id = \t%d\n"
		"power_up = \t%d\n"
		"adp_change = \t%d\n"
		"test_device = \t%d\n"
		"a_set_b_hnp_en = \t%d\n"
		"b_srp_done = \t%d\n"
		"b_hnp_enable = \t%d\n"
		"hnp_poll_enable = \t%d\n"
		"a_wait_vrise_tmout = \t%d\n"
		"a_wait_bcon_tmout = \t%d\n"
		"a_aidl_bdis_tmout = \t%d\n"
		"a_bidl_adis_tmout = \t%d\n"
		"a_bidl_adis_tmr = \t%d\n"
		"a_wait_vfall_tmout = \t%d\n"
		"b_ase0_brst_tmout = \t%d\n"
		"b_srp_fail_tmout = \t%d\n"
		"b_srp_fail_tmr = \t%d\n"
		"b_adp_sense_tmout = \t%d\n"
		"a_bus_drop = \t%d\n"
		"a_bus_req = \t%d\n"
		"a_clr_err = \t%d\n"
		"b_bus_req = \t%d\n",
		state_string(iotg->otg.state),
		iotg->hsm.a_bus_resume,
		iotg->hsm.a_bus_suspend,
		iotg->hsm.a_conn,
		iotg->hsm.a_sess_vld,
		iotg->hsm.a_srp_det,
		iotg->hsm.a_vbus_vld,
		iotg->hsm.b_bus_suspend,
		iotg->hsm.b_conn,
		iotg->hsm.b_se0_srp,
		iotg->hsm.b_ssend_srp,
		iotg->hsm.b_sess_end,
		iotg->hsm.b_sess_vld,
		iotg->hsm.id,
		iotg->hsm.power_up,
		iotg->hsm.adp_change,
		iotg->hsm.test_device,
		iotg->hsm.a_set_b_hnp_en,
		iotg->hsm.b_srp_done,
		iotg->hsm.b_hnp_enable,
		iotg->hsm.hnp_poll_enable,
		iotg->hsm.a_wait_vrise_tmout,
		iotg->hsm.a_wait_bcon_tmout,
		iotg->hsm.a_aidl_bdis_tmout,
		iotg->hsm.a_bidl_adis_tmout,
		iotg->hsm.a_bidl_adis_tmr,
		iotg->hsm.a_wait_vfall_tmout,
		iotg->hsm.b_ase0_brst_tmout,
		iotg->hsm.b_srp_fail_tmout,
		iotg->hsm.b_srp_fail_tmr,
		iotg->hsm.b_adp_sense_tmout,
		iotg->hsm.a_bus_drop,
		iotg->hsm.a_bus_req,
		iotg->hsm.a_clr_err,
		iotg->hsm.b_bus_req
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(hsm, S_IRUGO, show_hsm, NULL);

static ssize_t
show_chargers(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg		*pnw = the_transceiver;
	char				*next;
	unsigned			size, t;
	enum usb_charger_type		type;
	unsigned int			mA;
	unsigned long			flags;

	next = buf;
	size = PAGE_SIZE;

	spin_lock_irqsave(&pnw->charger_lock, flags);
	type = pnw->charging_cap.chrg_type;
	mA = pnw->charging_cap.mA;
	spin_unlock_irqrestore(&pnw->charger_lock, flags);

	t = scnprintf(next, size,
		"USB Battery Charging Capability\n"
		"\tUSB Charger Type:  %s\n"
		"\tMax Charging Current:  %u\n",
		charger_string(type),
		mA
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(chargers, S_IRUGO, show_chargers, NULL);

static ssize_t
get_a_bus_req(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg	*pnw = the_transceiver;
	char			*next;
	unsigned		size, t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size, "%d", pnw->iotg.hsm.a_bus_req);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_a_bus_req(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	if (!iotg->otg.default_a)
		return -1;
	if (count > 2)
		return -1;

	if (buf[0] == '0') {
		iotg->hsm.a_bus_req = 0;
		dev_dbg(pnw->dev, "a_bus_req = 0\n");
	} else if (buf[0] == '1') {
		/* If a_bus_drop is TRUE, a_bus_req can't be set */
		if (iotg->hsm.a_bus_drop)
			return -1;
		iotg->hsm.a_bus_req = 1;
		dev_dbg(pnw->dev, "a_bus_req = 1\n");
		if (iotg->otg.state == OTG_STATE_A_PERIPHERAL) {
			dev_warn(pnw->dev, "Role switch will be "
				"performed soon, if connected OTG device "
				"supports role switch request.\n");
			dev_warn(pnw->dev, "It may cause data"
				"corruption during data transfer\n");
		}
	}

	penwell_update_transceiver();

	return count;
}
static DEVICE_ATTR(a_bus_req, S_IRUGO | S_IWUGO, get_a_bus_req, set_a_bus_req);

static ssize_t
get_a_bus_drop(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg	*pnw = the_transceiver;
	char			*next;
	unsigned		size;
	unsigned		t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size, "%d", pnw->iotg.hsm.a_bus_drop);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_a_bus_drop(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	if (!iotg->otg.default_a)
		return -1;
	if (count > 2)
		return -1;

	if (buf[0] == '0') {
		iotg->hsm.a_bus_drop = 0;
		dev_dbg(pnw->dev, "a_bus_drop = 0\n");
	} else if (buf[0] == '1') {
		iotg->hsm.a_bus_drop = 1;
		iotg->hsm.a_bus_req = 0;
		dev_dbg(pnw->dev, "a_bus_drop = 1, so a_bus_req = 0\n");
	}

	penwell_update_transceiver();

	return count;
}
static DEVICE_ATTR(a_bus_drop, S_IRUGO | S_IWUGO,
	get_a_bus_drop, set_a_bus_drop);

static ssize_t
get_b_bus_req(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg	*pnw = the_transceiver;
	char			*next;
	unsigned		size;
	unsigned		t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size, "%d", pnw->iotg.hsm.b_bus_req);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static ssize_t
set_b_bus_req(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	if (iotg->otg.default_a)
		return -1;

	if (count > 2)
		return -1;

	if (buf[0] == '0') {
		iotg->hsm.b_bus_req = 0;
		dev_dbg(pnw->dev, "b_bus_req = 0\n");
	} else if (buf[0] == '1') {
		iotg->hsm.b_bus_req = 1;
		dev_dbg(pnw->dev, "b_bus_req = 1\n");
		if (iotg->otg.state == OTG_STATE_B_PERIPHERAL) {
			dev_warn(pnw->dev, "Role switch will be "
				"performed soon, if connected OTG device "
				"supports role switch request.\n");
			dev_warn(pnw->dev, "It may cause data "
				"corruption during data transfer\n");
		}
	}

	penwell_update_transceiver();

	return count;
}
static DEVICE_ATTR(b_bus_req, S_IRUGO | S_IWUGO, get_b_bus_req, set_b_bus_req);

static ssize_t
set_a_clr_err(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	if (!iotg->otg.default_a)
		return -1;
	if (iotg->otg.state != OTG_STATE_A_VBUS_ERR)
		return -1;
	if (count > 2)
		return -1;

	if (buf[0] == '1') {
		iotg->hsm.a_clr_err = 1;
		dev_dbg(pnw->dev, "a_clr_err = 1\n");
	}

	penwell_update_transceiver();

	return count;
}
static DEVICE_ATTR(a_clr_err, S_IWUGO, NULL, set_a_clr_err);

static struct attribute *inputs_attrs[] = {
	&dev_attr_a_bus_req.attr,
	&dev_attr_a_bus_drop.attr,
	&dev_attr_b_bus_req.attr,
	&dev_attr_a_clr_err.attr,
	NULL,
};

static struct attribute_group debug_dev_attr_group = {
	.name = "inputs",
	.attrs = inputs_attrs,
};

static int penwell_otg_probe(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	unsigned long		resource, len;
	void __iomem		*base = NULL;
	int			retval;
	u32			val32;
	struct penwell_otg	*pnw;
	char			qname[] = "penwell_otg_queue";

	retval = 0;

	dev_dbg(&pdev->dev, "\notg controller is detected.\n");

	if (pci_enable_device(pdev) < 0) {
		retval = -ENODEV;
		goto done;
	}

	pnw = kzalloc(sizeof *pnw, GFP_KERNEL);
	if (pnw == NULL) {
		retval = -ENOMEM;
		goto done;
	}
	the_transceiver = pnw;

	/* control register: BAR 0 */
	resource = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	if (!request_mem_region(resource, len, driver_name)) {
		retval = -EBUSY;
		goto err;
	}
	pnw->region = 1;

	base = ioremap_nocache(resource, len);
	if (base == NULL) {
		retval = -EFAULT;
		goto err;
	}
	pnw->iotg.base = base;

	if (!request_mem_region(USBCFG_ADDR, USBCFG_LEN, driver_name)) {
		retval = -EBUSY;
		goto err;
	}
	pnw->cfg_region = 1;

	if (!pdev->irq) {
		dev_dbg(&pdev->dev, "No IRQ.\n");
		retval = -ENODEV;
		goto err;
	}

	pnw->qwork = create_singlethread_workqueue(qname);
	if (!pnw->qwork) {
		dev_dbg(&pdev->dev, "cannot create workqueue %s\n", qname);
		retval = -ENOMEM;
		goto err;
	}
	INIT_WORK(&pnw->work, penwell_otg_work);
	INIT_WORK(&pnw->hnp_poll_work, penwell_otg_hnp_poll_work);

	/* OTG common part */
	pnw->dev = &pdev->dev;
	pnw->iotg.otg.dev = &pdev->dev;
	pnw->iotg.otg.label = driver_name;
	pnw->iotg.otg.set_host = penwell_otg_set_host;
	pnw->iotg.otg.set_peripheral = penwell_otg_set_peripheral;
	pnw->iotg.otg.set_power = penwell_otg_set_power;
	pnw->iotg.otg.set_vbus =  penwell_otg_set_vbus;
	pnw->iotg.otg.start_srp = penwell_otg_start_srp;
	pnw->iotg.set_adp_probe = NULL;
	pnw->iotg.set_adp_sense = NULL;
	pnw->iotg.otg.state = OTG_STATE_UNDEFINED;
	if (otg_set_transceiver(&pnw->iotg.otg)) {
		dev_dbg(pnw->dev, "can't set transceiver\n");
		retval = -EBUSY;
		goto err;
	}

	pnw->iotg.ulpi_ops.read = penwell_otg_ulpi_read;
	pnw->iotg.ulpi_ops.write = penwell_otg_ulpi_write;

	init_timer(&pnw->hsm_timer);
	init_timer(&pnw->bus_mon_timer);
	init_timer(&pnw->hnp_poll_timer);
	init_completion(&pnw->adp.adp_comp);

	/* Battery Charging part */
	spin_lock_init(&pnw->charger_lock);
	pnw->charging_cap.mA = CHRG_CURR_DISCONN;
	pnw->charging_cap.chrg_type = CHRG_UNKNOWN;
	pnw->charging_cap.current_event = USBCHRG_EVENT_DISCONN;

	ATOMIC_INIT_NOTIFIER_HEAD(&pnw->iotg.iotg_notifier);

	pnw->iotg_notifier.notifier_call = penwell_otg_iotg_notify;
	if (intel_mid_otg_register_notifier(&pnw->iotg, &pnw->iotg_notifier)) {
		dev_dbg(pnw->dev, "Failed to register notifier\n");
		retval = -EBUSY;
		goto err;
	}

	mutex_init(&pnw->msic_mutex);
	pnw->msic = penwell_otg_check_msic();

	penwell_otg_phy_low_power(0);
	penwell_otg_phy_enable(1);

	reset_otg();
	init_hsm();

	if (request_irq(pdev->irq, otg_irq, IRQF_SHARED,
				driver_name, pnw) != 0) {
		dev_dbg(pnw->dev,
			"request interrupt %d failed\n", pdev->irq);
		retval = -EBUSY;
		goto err;
	}

	/* enable OTGSC int */
	val32 = OTGSC_DPIE | OTGSC_BSEIE | OTGSC_BSVIE |
		OTGSC_ASVIE | OTGSC_AVVIE | OTGSC_IDIE | OTGSC_IDPU;
	writel(val32, pnw->iotg.base + CI_OTGSC);

	retval = device_create_file(&pdev->dev, &dev_attr_registers);
	if (retval < 0) {
		dev_dbg(pnw->dev,
			"Can't register sysfs attribute: %d\n", retval);
		goto err;
	}

	retval = device_create_file(&pdev->dev, &dev_attr_hsm);
	if (retval < 0) {
		dev_dbg(pnw->dev,
			"Can't hsm sysfs attribute: %d\n", retval);
		goto err;
	}

	retval = device_create_file(&pdev->dev, &dev_attr_chargers);
	if (retval < 0) {
		dev_dbg(pnw->dev,
			"Can't chargers sysfs attribute: %d\n", retval);
		goto err;
	}

	retval = sysfs_create_group(&pdev->dev.kobj, &debug_dev_attr_group);
	if (retval < 0) {
		dev_dbg(pnw->dev,
			"Can't register sysfs attr group: %d\n", retval);
		goto err;
	}

	if (pnw->iotg.otg.state == OTG_STATE_A_IDLE)
		queue_work(pnw->qwork, &pnw->work);

	return 0;

err:
	if (the_transceiver)
		penwell_otg_remove(pdev);
done:
	return retval;
}

static void penwell_otg_remove(struct pci_dev *pdev)
{
	struct penwell_otg *pnw = the_transceiver;

	if (pnw->qwork) {
		flush_workqueue(pnw->qwork);
		destroy_workqueue(pnw->qwork);
	}

	/* disable OTGSC interrupt as OTGSC doesn't change in reset */
	writel(0, pnw->iotg.base + CI_OTGSC);

	if (pdev->irq)
		free_irq(pdev->irq, pnw);
	if (pnw->cfg_region)
		release_mem_region(USBCFG_ADDR, USBCFG_LEN);
	if (pnw->iotg.base)
		iounmap(pnw->iotg.base);
	if (pnw->region)
		release_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	otg_set_transceiver(NULL);
	pci_disable_device(pdev);
	sysfs_remove_group(&pdev->dev.kobj, &debug_dev_attr_group);
	device_remove_file(&pdev->dev, &dev_attr_chargers);
	device_remove_file(&pdev->dev, &dev_attr_hsm);
	device_remove_file(&pdev->dev, &dev_attr_registers);
	kfree(pnw);
	pnw = NULL;
}

static void transceiver_suspend(struct pci_dev *pdev)
{
	penwell_otg_phy_low_power(1);
	pci_save_state(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
}

static int penwell_otg_suspend(struct pci_dev *pdev, pm_message_t message)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				ret = 0;

	/* Disbale OTG interrupts */
	penwell_otg_intr(0);

	if (pdev->irq)
		free_irq(pdev->irq, pnw);

	/* Prevent more otg_work */
	flush_workqueue(pnw->qwork);
	destroy_workqueue(pnw->qwork);
	pnw->qwork = NULL;

	/* start actions */
	switch (iotg->otg.state) {
	case OTG_STATE_A_WAIT_VFALL:
		iotg->otg.state = OTG_STATE_A_IDLE;
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_VBUS_ERR:
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_B_IDLE:
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_A_WAIT_VRISE:
		penwell_otg_del_timer(TA_WAIT_VRISE_TMR);
		iotg->hsm.a_srp_det = 0;

		/* Turn off VBus */
		iotg->otg.set_vbus(&iotg->otg, false);
		iotg->otg.state = OTG_STATE_A_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_A_WAIT_BCON:
		penwell_otg_del_timer(TA_WAIT_BCON_TMR);
		if (pnw->iotg.stop_host)
			pnw->iotg.stop_host(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "host driver has been stopped.\n");

		iotg->hsm.a_srp_det = 0;

		/* Turn off VBus */
		iotg->otg.set_vbus(&iotg->otg, false);
		iotg->otg.state = OTG_STATE_A_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_A_HOST:
		if (pnw->iotg.stop_host)
			pnw->iotg.stop_host(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "host driver has been stopped.\n");

		iotg->hsm.a_srp_det = 0;

		/* Turn off VBus */
		iotg->otg.set_vbus(&iotg->otg, false);

		iotg->otg.state = OTG_STATE_A_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_A_SUSPEND:
		penwell_otg_del_timer(TA_AIDL_BDIS_TMR);
		penwell_otg_HABA(0);
		if (pnw->iotg.stop_host)
			pnw->iotg.stop_host(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "host driver has been removed.\n");
		iotg->hsm.a_srp_det = 0;

		/* Turn off VBus */
		iotg->otg.set_vbus(&iotg->otg, false);
		iotg->otg.state = OTG_STATE_A_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_A_PERIPHERAL:
		penwell_otg_del_timer(TA_BIDL_ADIS_TMR);

		if (pnw->iotg.stop_peripheral)
			pnw->iotg.stop_peripheral(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "client driver has been stopped.\n");

		/* Turn off VBus */
		iotg->otg.set_vbus(&iotg->otg, false);
		iotg->hsm.a_srp_det = 0;
		iotg->otg.state = OTG_STATE_A_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_B_HOST:
		if (pnw->iotg.stop_host)
			pnw->iotg.stop_host(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "host driver has been stopped.\n");
		iotg->hsm.b_bus_req = 0;
		iotg->otg.state = OTG_STATE_B_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (pnw->iotg.stop_peripheral)
			pnw->iotg.stop_peripheral(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "client driver has been stopped.\n");
		iotg->otg.state = OTG_STATE_B_IDLE;
		transceiver_suspend(pdev);
		break;
	case OTG_STATE_B_WAIT_ACON:
		penwell_otg_del_timer(TB_ASE0_BRST_TMR);

		penwell_otg_HAAR(0);

		if (pnw->iotg.stop_host)
			pnw->iotg.stop_host(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "host driver has been stopped.\n");
		iotg->hsm.b_bus_req = 0;
		iotg->otg.state = OTG_STATE_B_IDLE;
		transceiver_suspend(pdev);
		break;
	default:
		dev_dbg(pnw->dev, "error state before suspend\n");
		break;
	}

	return ret;
}

static void transceiver_resume(struct pci_dev *pdev)
{
	pci_restore_state(pdev);
	pci_set_power_state(pdev, PCI_D0);
}

static int penwell_otg_resume(struct pci_dev *pdev)
{
	struct penwell_otg	*pnw = the_transceiver;
	int			ret = 0;

	transceiver_resume(pdev);

	pnw->qwork = create_singlethread_workqueue("penwell_otg_queue");
	if (!pnw->qwork) {
		dev_dbg(pnw->dev, "cannot create penwell otg workqueue\n");
		ret = -ENOMEM;
		goto error;
	}

	if (request_irq(pdev->irq, otg_irq, IRQF_SHARED,
				driver_name, pnw) != 0) {
		dev_dbg(pnw->dev, "request irq %d failed\n", pdev->irq);
		ret = -EBUSY;
		goto error;
	}

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_B_IDLE:
		break;
	default:
		break;
	}

	/* enable OTG interrupts */
	penwell_otg_intr(1);

	update_hsm();

	penwell_update_transceiver();

	return ret;
error:
	penwell_otg_intr(0);
	transceiver_suspend(pdev);
	return ret;
}

#ifdef CONFIG_PM_RUNTIME
/* Runtime PM */
static int penwell_otg_runtime_suspend(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct pci_dev		*pdev;
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	pdev = to_pci_dev(dev);

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
		/* Transceiver handle it itself */
		penwell_otg_phy_low_power(1);
		pci_save_state(pdev);
		pci_disable_device(pdev);
		pci_set_power_state(pdev, PCI_D3hot);
		break;
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_HOST:
	case OTG_STATE_A_SUSPEND:
		if (pnw->iotg.runtime_suspend_host)
			ret = pnw->iotg.runtime_suspend_host(&pnw->iotg);
		break;
	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_B_PERIPHERAL:
		if (pnw->iotg.runtime_suspend_peripheral)
			ret = pnw->iotg.runtime_suspend_peripheral(&pnw->iotg);
		break;
	default:
		break;
	}

	dev_dbg(dev, "%s <---\n", __func__);
	return ret;
}

static int penwell_otg_runtime_resume(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct pci_dev		*pdev;
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	pdev = to_pci_dev(dev);

	penwell_otg_intr(1);
	penwell_otg_phy_low_power(0);

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
		/* Transceiver handle it itself */
		pci_set_power_state(pdev, PCI_D0);
		pci_restore_state(pdev);
		ret = pci_enable_device(pdev);
		if (ret)
			dev_err(&pdev->dev, "device cant be enabled\n");
		penwell_otg_phy_low_power(0);
		break;
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_HOST:
	case OTG_STATE_A_SUSPEND:
		if (pnw->iotg.runtime_resume_host)
			ret = pnw->iotg.runtime_resume_host(&pnw->iotg);
		break;
	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_B_PERIPHERAL:
		if (pnw->iotg.runtime_resume_peripheral)
			ret = pnw->iotg.runtime_resume_peripheral(&pnw->iotg);
		break;
	default:
		break;
	}

	dev_dbg(dev, "%s <---\n", __func__);
	return 0;
}

static int penwell_otg_runtime_idle(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;

	dev_dbg(dev, "%s --->\n", __func__);

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_A_WAIT_VRISE:
	case OTG_STATE_A_WAIT_VFALL:
	case OTG_STATE_A_VBUS_ERR:
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_HOST:
		dev_dbg(dev, "Keep in active\n");
		return -EBUSY;
	default:
		break;
	}

	dev_dbg(dev, "%s <---\n", __func__);

	return 0;
}

#else

#define penwell_otg_runtime_suspend NULL
#define penwell_otg_runtime_resume NULL
#define penwell_otg_runtime_idle NULL

#endif

/*----------------------------------------------------------*/

static const struct pci_device_id pci_ids[] = {{
	.class =        ((PCI_CLASS_SERIAL_USB << 8) | 0x20),
	.class_mask =   ~0,
	.vendor =	0x8086,
	.device =	0x0829,
	.subvendor =	PCI_ANY_ID,
	.subdevice =	PCI_ANY_ID,
}, { /* end: all zeroes */ }
};

static const struct dev_pm_ops penwell_otg_pm_ops = {
	.runtime_suspend = penwell_otg_runtime_suspend,
	.runtime_resume = penwell_otg_runtime_resume,
	.runtime_idle = penwell_otg_runtime_idle,
};

static struct pci_driver otg_pci_driver = {
	.name =		(char *) driver_name,
	.id_table =	pci_ids,

	.probe =	penwell_otg_probe,
	.remove =	penwell_otg_remove,

	.suspend =	penwell_otg_suspend,
	.resume =	penwell_otg_resume,

	.driver = {
		.pm =	&penwell_otg_pm_ops
	},
};

static int __init penwell_otg_init(void)
{
	return pci_register_driver(&otg_pci_driver);
}
module_init(penwell_otg_init);

static void __exit penwell_otg_cleanup(void)
{
	pci_unregister_driver(&otg_pci_driver);
}
module_exit(penwell_otg_cleanup);
