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
#include <linux/gpio.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#include "../core/usb.h"
#include <linux/intel_mid_pm.h>

#include <linux/usb/penwell_otg.h>

#define	DRIVER_DESC		"Intel Penwell USB OTG transceiver driver"
#define	DRIVER_VERSION		"0.8"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Henry Yuan <hang.yuan@intel.com>, Hao Wu <hao.wu@intel.com>");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

static const char driver_name[] = "penwell_otg";

static void penwell_otg_remove(struct pci_dev *pdev);

static int penwell_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host);
static int penwell_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget);
static int penwell_otg_start_srp(struct otg_transceiver *otg);
static void penwell_otg_mon_bus(void);

static int penwell_otg_msic_write(u16 addr, u8 data);

static void penwell_otg_phy_low_power(int on);
static int penwell_otg_ulpi_read(struct intel_mid_otg_xceiv *iotg,
				u8 reg, u8 *val);
static int penwell_otg_ulpi_write(struct intel_mid_otg_xceiv *iotg,
				u8 reg, u8 val);
static void penwell_spi_reset_phy(void);
static int penwell_otg_charger_hwdet(bool enable);

#ifdef CONFIG_PM_SLEEP
#include <linux/suspend.h>
DECLARE_WAIT_QUEUE_HEAD(stop_host_wait);
atomic_t pnw_sys_suspended;

static int pnw_sleep_pm_callback(struct notifier_block *nfb,
			unsigned long action, void *ignored)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		atomic_set(&pnw_sys_suspended, 1);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		atomic_set(&pnw_sys_suspended, 0);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block pnw_sleep_pm_notifier = {
	.notifier_call = pnw_sleep_pm_callback,
	.priority = 0
};

#define PNW_PM_RESUME_WAIT(a) \
		while (atomic_read(&pnw_sys_suspended)) { \
			wait_event_timeout(a, false, HZ/100); \
		}
#else

#define PNW_PM_RESUME_WAIT(a)

#endif

#define PNW_STOP_HOST(pnw) do { \
	if ((pnw)->iotg.stop_host) { \
		PNW_PM_RESUME_WAIT(stop_host_wait); \
		(pnw)->iotg.stop_host(&(pnw)->iotg); \
	} \
	} while (0)

inline int is_clovertrail(struct pci_dev *pdev)
{
	return (pdev->vendor == 0x8086 && pdev->device == 0xE006);
}
EXPORT_SYMBOL_GPL(is_clovertrail);

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
	case CHRG_SDP_INVAL:
		return "Invalid Standard Downstream Port";
	case CHRG_CDP:
		return "Charging Downstream Port";
	case CHRG_DCP:
		return "Dedicated Charging Port";
	case CHRG_ACA:
		return "Accessory Charger Adaptor";
	case CHRG_UNKNOWN:
		return "Unknown";
	default:
		return "Undefined";
	}
}

static const char *psc_string(enum power_supply_type charger)
{
	switch (charger) {
	case POWER_SUPPLY_TYPE_USB:
		return "Standard Downstream Port";
	case POWER_SUPPLY_TYPE_USB_CDP:
		return "Charging Downstream Port";
	case POWER_SUPPLY_TYPE_USB_DCP:
		return "Dedicated Charging Port";
	case POWER_SUPPLY_TYPE_USB_ACA:
		return "Accessory Charger Adaptor";
	case POWER_SUPPLY_TYPE_USB_HOST:
		return "USB Host";
	case POWER_SUPPLY_TYPE_BATTERY:
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

	if (!pnw->qwork) {
		dev_warn(pnw->dev, "no workqueue for state machine\n");
		return ;
	}

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
	case CHRG_SDP_INVAL:
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
		} else if (pnw->charging_cap.chrg_type == CHRG_SDP &&
			charger == CHRG_SDP_INVAL) {
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
		if (pnw->charging_cap.mA == CHRG_CURR_DISCONN
				&& (mA == CHRG_CURR_SDP_LOW
				|| mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: charger connect */
			event = USBCHRG_EVENT_CONNECT;
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
		} else if (pnw->charging_cap.mA == CHRG_CURR_SDP_SUSP
				&& (mA == CHRG_CURR_SDP_LOW
				|| mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: resume from suspend state */
			event = USBCHRG_EVENT_RESUME;
			flag = 1;
		} else if ((pnw->charging_cap.mA == CHRG_CURR_SDP_LOW
			|| pnw->charging_cap.mA == CHRG_CURR_SDP_HIGH)
				&& mA == CHRG_CURR_SDP_SUSP) {
			/* SDP event: enter suspend state */
			event = USBCHRG_EVENT_SUSPEND;
			flag = 1;
		} else if (mA == 0) {
			event = USBCHRG_EVENT_DISCONN;
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
	case CHRG_SDP_INVAL:
		if (mA == CHRG_CURR_SDP_INVAL) {
			event = USBCHRG_EVENT_UPDATE;
			flag = 1;
		} else
			dev_dbg(pnw->dev, "SDP_INVAL: no need to update EM\n");
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

static enum power_supply_type usb_chrg_to_power_supply_chrg(
				enum usb_charger_type chrg_type)
{
	switch (chrg_type) {
	case CHRG_SDP: return POWER_SUPPLY_TYPE_USB;
	case CHRG_CDP: return POWER_SUPPLY_TYPE_USB_CDP;
	case CHRG_DCP: return POWER_SUPPLY_TYPE_USB_DCP;
	case CHRG_ACA: return POWER_SUPPLY_TYPE_USB_ACA;
	default: return POWER_SUPPLY_TYPE_BATTERY;
	}
}

static enum power_supply_charger_event check_psc_event(
			struct power_supply_charger_cap old,
			struct power_supply_charger_cap new)
{
	struct penwell_otg		*pnw = the_transceiver;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	/* Check charger type information */
	if (old.chrg_type != new.chrg_type) {
		if (old.chrg_type == POWER_SUPPLY_TYPE_BATTERY && new.mA != 0)
			return POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		else if (new.chrg_type == POWER_SUPPLY_TYPE_BATTERY)
			return POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		else {
			dev_dbg(pnw->dev, "not a valid event\n");
			return -1;
		}
	}

	/* Check the charging current limit */
	if (old.mA == new.mA) {
		dev_dbg(pnw->dev, "not a valid event\n");
		return -1;
	}

	switch (new.chrg_type) {
	case POWER_SUPPLY_TYPE_USB:
		if (old.mA == CHRG_CURR_DISCONN
				&& (new.mA == CHRG_CURR_SDP_LOW
				|| new.mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: charger connect */
			return POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		} else if (old.mA == CHRG_CURR_SDP_LOW
				&& new.mA == CHRG_CURR_SDP_HIGH) {
			/* SDP event: configuration update */
			return POWER_SUPPLY_CHARGER_EVENT_UPDATE;
		} else if (old.mA == CHRG_CURR_SDP_HIGH
				&& new.mA == CHRG_CURR_SDP_LOW) {
			/* SDP event: configuration update */
			return POWER_SUPPLY_CHARGER_EVENT_UPDATE;
		} else if (old.mA == CHRG_CURR_SDP_SUSP
				&& (new.mA == CHRG_CURR_SDP_LOW
				|| new.mA == CHRG_CURR_SDP_HIGH)) {
			/* SDP event: resume from suspend state */
			return POWER_SUPPLY_CHARGER_EVENT_RESUME;
		} else if ((old.mA == CHRG_CURR_SDP_LOW
			|| old.mA == CHRG_CURR_SDP_HIGH)
				&& new.mA == CHRG_CURR_SDP_SUSP) {
			/* SDP event: enter suspend state */
			return POWER_SUPPLY_CHARGER_EVENT_SUSPEND;
		} else
			dev_dbg(pnw->dev, "SDP: no need to update EM\n");
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		if (new.mA == CHRG_CURR_DCP) {
			/* DCP event: charger connect */
			return POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		} else
			dev_dbg(pnw->dev, "DCP: no need to update EM\n");
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (pnw->charging_cap.mA == CHRG_CURR_DISCONN
				&& new.mA == CHRG_CURR_CDP) {
			/* CDP event: charger connect */
			return POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		} else
			dev_dbg(pnw->dev, "CDP: no need to update EM\n");
		break;
	case POWER_SUPPLY_TYPE_BATTERY:
		if (new.mA == CHRG_CURR_DISCONN) {
			/* event: chargers disconnect */
			return POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		} else
			dev_dbg(pnw->dev, "UNKNOWN: no need to update EM\n");
		break;
	default:
		break;
	}

	return -1;

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

static void penwell_otg_update_chrg_cap(enum usb_charger_type charger,
				unsigned mA)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct pci_dev			*pdev;
	unsigned long			flags;
	struct otg_bc_event		*event;

	pdev = to_pci_dev(pnw->dev);

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (!is_clovertrail(pdev)) {
		spin_lock_irqsave(&pnw->charger_lock, flags);
		_penwell_otg_update_chrg_cap(charger, mA);
		spin_unlock_irqrestore(&pnw->charger_lock, flags);
	} else {
		dev_dbg(pnw->dev, "clv charger_cap_update\n");

		event = kzalloc(sizeof(*event), GFP_ATOMIC);
		if (!event) {
			dev_err(pnw->dev, "no memory for charging event");
			return ;
		}

		event->cap.chrg_type = usb_chrg_to_power_supply_chrg(charger);
		event->cap.mA = mA;
		INIT_LIST_HEAD(&event->node);

		spin_lock_irqsave(&pnw->charger_lock, flags);
		list_add_tail(&event->node, &pnw->chrg_evt_queue);
		spin_unlock_irqrestore(&pnw->charger_lock, flags);

		queue_work(pnw->chrg_qwork, &pnw->psc_notify);
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}

static int penwell_otg_set_power(struct otg_transceiver *otg,
				unsigned mA)
{
	struct penwell_otg		*pnw = the_transceiver;
	unsigned long			flags;
	struct pci_dev			*pdev;
	struct otg_bc_event		*event;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	pdev = to_pci_dev(pnw->dev);

	if (!is_clovertrail(pdev)) {
		spin_lock_irqsave(&pnw->charger_lock, flags);

		if (pnw->charging_cap.chrg_type != CHRG_SDP) {
			spin_unlock_irqrestore(&pnw->charger_lock, flags);
			return 0;
		}

		_penwell_otg_update_chrg_cap(CHRG_SDP, mA);

		spin_unlock_irqrestore(&pnw->charger_lock, flags);
	} else {
		dev_dbg(pnw->dev, "clv charger_set_power\n");

		if (pnw->psc_cap.chrg_type != POWER_SUPPLY_TYPE_USB)
			return 0;

		event = kzalloc(sizeof(*event), GFP_ATOMIC);
		if (!event) {
			dev_err(pnw->dev, "no memory for charging event");
			return -ENOMEM;
		}

		event->cap.chrg_type = POWER_SUPPLY_TYPE_USB;
		event->cap.mA = mA;
		INIT_LIST_HEAD(&event->node);

		spin_lock_irqsave(&pnw->charger_lock, flags);
		list_add_tail(&event->node, &pnw->chrg_evt_queue);
		spin_unlock_irqrestore(&pnw->charger_lock, flags);

		queue_work(pnw->chrg_qwork, &pnw->psc_notify);
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return 0;
}

/* for Clovertrail only */
int penwell_otg_query_power_supply_cap(struct power_supply_charger_cap *cap)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (pnw == NULL)
		return -ENODEV;

	if (cap == NULL)
		return -EINVAL;

	if (!is_clovertrail(to_pci_dev(pnw->dev)))
		return -ENODEV;

	spin_lock_irqsave(&pnw->cap_lock, flags);
	cap->chrg_evt = pnw->psc_cap.chrg_evt;
	cap->chrg_type = pnw->psc_cap.chrg_type;
	cap->mA = pnw->psc_cap.mA;
	spin_unlock_irqrestore(&pnw->cap_lock, flags);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(penwell_otg_query_power_supply_cap);


int penwell_otg_query_charging_cap(struct otg_bc_cap *cap)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (pnw == NULL)
		return -ENODEV;

	if (cap == NULL)
		return -EINVAL;

	if (is_clovertrail(to_pci_dev(pnw->dev)))
		return -ENODEV;

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

	if (is_clovertrail(to_pci_dev(pnw->dev)))
		return NULL;

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

	if (is_clovertrail(to_pci_dev(pnw->dev)))
		return -ENODEV;

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
	struct penwell_otg		*pnw = the_transceiver;
	u8				data;
	unsigned long			flags;
	int				retval = 0;
	struct power_supply_charger_cap psc_cap;
	struct otg_bc_event		*evt;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, enabled ? "on" : "off");

	/*
	 * For Clovertrail, VBUS is driven by TPS2052 power switch chip.
	 * But TPS2052 is controlled by ULPI PHY.
	 */
	if (is_clovertrail(to_pci_dev(pnw->dev))) {
		penwell_otg_phy_low_power(0);
		if (enabled)
			penwell_otg_ulpi_write(&pnw->iotg,
				ULPI_OTGCTRLSET, DRVVBUS | DRVVBUS_EXTERNAL);
		else
			penwell_otg_ulpi_write(&pnw->iotg,
				ULPI_OTGCTRLCLR, DRVVBUS | DRVVBUS_EXTERNAL);

		evt = kzalloc(sizeof(*evt), GFP_KERNEL);
		if (!evt) {
			dev_err(pnw->dev, "no memory for charging event");
			return -ENOMEM;
		}

		evt->cap.chrg_type = POWER_SUPPLY_TYPE_USB;
		INIT_LIST_HEAD(&evt->node);

		if (enabled) {
			evt->cap.chrg_type = POWER_SUPPLY_TYPE_USB_HOST;
			evt->cap.mA = 500;
			evt->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		} else if (pnw->iotg.hsm.id == ID_ACA_A
				|| pnw->iotg.hsm.id == ID_ACA_B
				|| pnw->iotg.hsm.id == ID_ACA_C) {
			evt->cap.chrg_type = POWER_SUPPLY_TYPE_USB_ACA;
			evt->cap.mA = CHRG_CURR_ACA;
			evt->cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		} else {
			evt->cap.chrg_type = POWER_SUPPLY_TYPE_BATTERY;
			evt->cap.mA = 0;
			evt->cap.chrg_evt =
				POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		}

		dev_dbg(pnw->dev, "notify power_supply_charger_event\n");
		dev_dbg(pnw->dev, "mA = %d\n", evt->cap.mA);
		dev_dbg(pnw->dev, "event = %d\n", evt->cap.chrg_evt);
		dev_dbg(pnw->dev, "type = %s\n",
				psc_string(evt->cap.chrg_type));

		spin_lock_irqsave(&pnw->charger_lock, flags);
		list_add_tail(&evt->node, &pnw->chrg_evt_queue);
		spin_unlock_irqrestore(&pnw->charger_lock, flags);

		queue_work(pnw->chrg_qwork, &pnw->psc_notify);

		goto done;
	}

	data = enabled ? VOTGEN : 0;

	mutex_lock(&pnw->msic_mutex);

	retval = intel_scu_ipc_update_register(MSIC_VOTGCNT, data, VOTGEN);

	if (retval)
		dev_err(pnw->dev, "Fail to set power on OTG Port\n");

	mutex_unlock(&pnw->msic_mutex);

done:
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
	unsigned long		flags;

	dev_dbg(pnw->dev, "%s - addr 0x%x\n", __func__, reg);

	spin_lock_irqsave(&pnw->lock, flags);

	/* Port = 0 */
	val32 = ULPI_RUN | reg << 16;
	writel(val32, pnw->iotg.base + CI_ULPIVP);

	/* Polling at least 2ms for read operation to complete*/
	count = 400;

	while (count) {
		val32 = readl(pnw->iotg.base + CI_ULPIVP);
		if (val32 & ULPI_RUN) {
			count--;
			udelay(5);
		} else {
			*val = (u8)((val32 & ULPI_DATRD) >> 8);
			dev_dbg(pnw->dev,
				"%s - done data 0x%x\n", __func__, *val);
			spin_unlock_irqrestore(&pnw->lock, flags);
			return 0;
		}
	}

	dev_warn(pnw->dev, "%s - timeout\n", __func__);

	spin_unlock_irqrestore(&pnw->lock, flags);
	return -ETIMEDOUT;

}

static int
penwell_otg_ulpi_write(struct intel_mid_otg_xceiv *iotg, u8 reg, u8 val)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val32 = 0;
	int			count;
	unsigned long		flags;

	dev_dbg(pnw->dev,
		"%s - addr 0x%x - data 0x%x\n", __func__, reg, val);

	spin_lock_irqsave(&pnw->lock, flags);

	/* Port = 0 */
	val32 = ULPI_RUN | ULPI_RW | reg << 16 | val;
	writel(val32, pnw->iotg.base + CI_ULPIVP);

	/* Polling at least 2ms for write operation to complete*/
	count = 400;

	while (count && penwell_otg_ulpi_run()) {
		count--;
		udelay(5);
	}

	dev_dbg(pnw->dev,
		"%s - %s\n", __func__, count ? "complete" : "timeout");

	spin_unlock_irqrestore(&pnw->lock, flags);
	return count ? 0 : -ETIMEDOUT;
}

int pnw_otg_ulpi_write(u8 reg, u8 val)
{
	return penwell_otg_ulpi_write(&the_transceiver->iotg,
					reg, val);
}
EXPORT_SYMBOL_GPL(pnw_otg_ulpi_write);

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

/* HNP polling function */
/* The timeout callback function which polls the host request flag for HNP */
static void penwell_otg_hnp_poll_fn(unsigned long indicator)
{
	struct penwell_otg	*pnw = the_transceiver;

	queue_work(pnw->qwork, &pnw->hnp_poll_work);
}

/* Start HNP polling */
/* Call this function with iotg->hnp_poll_lock held */
static int penwell_otg_add_hnp_poll_timer(struct intel_mid_otg_xceiv *iotg,
					unsigned long delay)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		j = jiffies;

	pnw->hnp_poll_timer.data = 1;
	pnw->hnp_poll_timer.function = penwell_otg_hnp_poll_fn;
	pnw->hnp_poll_timer.expires = j + msecs_to_jiffies(delay);

	add_timer(&pnw->hnp_poll_timer);

	return 0;
}

static int penwell_otg_start_hnp_poll(struct intel_mid_otg_xceiv *iotg)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	spin_lock_irqsave(&pnw->iotg.hnp_poll_lock, flags);

	if (pnw->iotg.hsm.hnp_poll_enable) {
		spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);
		dev_dbg(pnw->dev, "HNP polling is already enabled\n");
		return 0;
	}

	/* mark HNP polling enabled and start HNP polling in 50ms */
	pnw->iotg.hsm.hnp_poll_enable = 1;
	penwell_otg_add_hnp_poll_timer(&pnw->iotg, 50);

	spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);

	return 0;
}

static int penwell_otg_continue_hnp_poll(struct intel_mid_otg_xceiv *iotg)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	spin_lock_irqsave(&pnw->iotg.hnp_poll_lock, flags);

	if (!pnw->iotg.hsm.hnp_poll_enable) {
		spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);
		dev_dbg(pnw->dev, "HNP polling is disabled, stop polling\n");
		return 0;
	}

	penwell_otg_add_hnp_poll_timer(&pnw->iotg, THOS_REQ_POL);

	spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);

	return 0;
}

/* Stop HNP polling */
static int penwell_otg_stop_hnp_poll(struct intel_mid_otg_xceiv *iotg)
{
	struct penwell_otg	*pnw = the_transceiver;
	unsigned long		flags;

	spin_lock_irqsave(&pnw->iotg.hnp_poll_lock, flags);

	if (!pnw->iotg.hsm.hnp_poll_enable) {
		spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);
		dev_dbg(pnw->dev, "HNP polling is already disabled\n");
		return 0;
	}

	pnw->iotg.hsm.hnp_poll_enable = 0;
	del_timer_sync(&pnw->hnp_poll_timer);

	spin_unlock_irqrestore(&pnw->iotg.hnp_poll_lock, flags);

	return 0;
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
		dev_warn(pnw->dev, "DataLine SRP Error\n");

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return 0;
}

/* stop SOF via bus_suspend */
static void penwell_otg_loc_sof(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct usb_hcd		*hcd;
	int			err;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "resume" : "suspend");

	hcd = bus_to_hcd(pnw->iotg.otg.host);
	if (on)
		err = hcd->driver->bus_resume(hcd);
	else
		err = hcd->driver->bus_suspend(hcd);

	if (err)
		dev_warn(pnw->dev, "Fail to resume/suspend USB bus -%d\n", err);

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

/*
 * For Penwell, VBUS330 is the power rail to otg PHY inside MSIC, set it
 * into low power mode or normal mode according to pm state.
 * Call this function when spi access to MSIC registers is enabled.
 *
 * For Clovertrail, we don't have a controllable power rail to the PHY -
 * it's always on.
 */
static int penwell_otg_vusb330_low_power(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			data;
	int			retval;

	dev_dbg(pnw->dev, "%s ---> %s\n", __func__, on ? "on" : "off");

	if (!is_clovertrail(to_pci_dev(pnw->dev))) {
		if (on)
			data = 0x5; /* Low power mode */
		else
			data = 0x7; /* Normal mode */
		retval = penwell_otg_msic_write(MSIC_VUSB330CNT, data);
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return retval;
}

/* penwell_otg_id enables/disables id interrupt */
static void penwell_otg_id(int on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u32			val;

	val = readl(pnw->iotg.base + CI_OTGSC);
	if (on) {
		val = val | OTGSC_IDIE;
		writel(val, pnw->iotg.base + CI_OTGSC);
	} else {
		val = val & ~OTGSC_IDIE;
		writel(val, pnw->iotg.base + CI_OTGSC);
	}
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
	if (retval)
		dev_warn(pnw->dev, "Failed to write MSIC register %x\n", addr);

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
	int			count = 50;
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
		retval = intel_scu_ipc_ioread8(MSIC_USB_MISC, &data);
		if (retval) {
			dev_warn(pnw->dev, "Failed to read MSIC register\n");
			return retval;
		}

		if (data & MISC_CHGDSERXDPINV) {
			dev_dbg(pnw->dev, "Data contact detected!\n");
			return 0;
		}
		count--;
		/* Interval is 10 - 11ms */
		usleep_range(10000, 11000);
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

/* manual charger detection by ULPI access */
static int penwell_otg_manual_chrg_det(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg;
	int				retval;
	u8				data, data1, data2;
	unsigned long			timeout, interval;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	iotg = &pnw->iotg;

	dev_info(pnw->dev, "USB charger detection start...\n");

	/* config PHY for DCD
	 * - OPMODE/TERMSEL/XCVRSEL=01/0/01
	 * - enable DP/DM pulldowns (ensure RX's don't float) */

	/* ulpi_write(0x0b, 0x06) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_OTGCTRLSET,
				DMPULLDOWN | DPPULLDOWN);
	if (retval)
		return retval;

	/* ulpi_write(0x05, 0x09) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_FUNCTRLSET,
				OPMODE0 | XCVRSELECT0);
	if (retval)
		return retval;

	/* ulpi_write(0x06, 0x16) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_FUNCTRLCLR,
				OPMODE1 | TERMSELECT | XCVRSELECT1);
	if (retval)
		return retval;

	/* ulpi_write(0x0c, 0x02) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_OTGCTRLCLR, DPPULLDOWN);
	if (retval)
		return retval;

	/* ulpi_write(0x3e, 0x01) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, SWCNTRL);
	if (retval)
		return retval;

	/* ulpi_write(0x86, 0x40) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_VS3SET, CHGD_IDP_SRC);
	if (retval)
		return retval;

	dev_info(pnw->dev, "charger detection DCD start...\n");

	/* Check DCD result, use same polling parameter */
	timeout = jiffies + msecs_to_jiffies(DATACON_TIMEOUT);
	interval = DATACON_INTERVAL * 1000; /* us */

	while (!time_after(jiffies, timeout)) {
		retval = penwell_otg_ulpi_read(iotg, ULPI_VS4, &data);
		if (retval) {
			dev_warn(pnw->dev, "Failed to read ULPI register\n");
			return retval;
		}

		if (!(data & CHRG_SERX_DP)) {
			dev_info(pnw->dev, "Data contact detected!\n");
			break;
		}

		/* Polling interval */
		usleep_range(interval, interval + 2000);
	}

	/* ulpi_write(0x87, 0x40)*/
	retval = penwell_otg_ulpi_write(iotg, ULPI_VS3CLR, CHGD_IDP_SRC);
	if (retval)
		return retval;

	dev_info(pnw->dev, "DCD complete\n");
	dev_info(pnw->dev, "Primary Detection start...\n");

	/* Primary Dection config */
	/* ulpi_write(0x0b, 0x06) */
	retval = penwell_otg_ulpi_write(iotg, ULPI_OTGCTRLSET,
				DMPULLDOWN | DPPULLDOWN);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, DPVSRCEN);
	if (retval)
		return retval;

	msleep(125);

	/* Check result SDP vs CDP/DCP */
	retval = penwell_otg_ulpi_read(iotg, ULPI_PWRCTRL, &data1);
	if (retval)
		return retval;

	data1 = data1 & VDATDET;

	retval = penwell_otg_ulpi_read(iotg, ULPI_VS4, &data2);
	if (retval)
		return retval;

	data2 = data2 & CHRG_SERX_DM;

	if (!data1 || data2) {
		retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR,
						DPVSRCEN);
		if (retval)
			return retval;

		dev_info(pnw->dev, "USB Charger Detection done\n");
		return CHRG_SDP;
	}

	/* start detection on CDP vs DCP */
	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	/* sleep 1ms between Primary and Secondary detection */
	usleep_range(1000, 1200);

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS1CLR, DATAPOLARITY);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, DPVSRCEN);
	if (retval)
		return retval;

	msleep(85);

	/* read result on CDP vs DCP */
	retval = penwell_otg_ulpi_read(iotg, ULPI_PWRCTRL, &data);
	if (retval)
		return retval;

	data = data & VDATDET;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS1SET, DATAPOLARITY);
	if (retval)
		return retval;

	dev_info(pnw->dev, "USB Charger Detection done\n");

	if (data) {
		retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET,
							DPVSRCEN);
		if (retval)
			return retval;

		return CHRG_DCP;
	} else
		return CHRG_CDP;

	dev_dbg(pnw->dev, "%s <---\n", __func__);
};

static int penwell_otg_charger_det_dcd_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;
	unsigned long			timeout, interval;
	u8				data;

	/* Change OPMODE to '01' Non-driving */
	retval = penwell_otg_ulpi_write(iotg, ULPI_FUNCTRLSET,
						OPMODE0 | XCVRSELECT0);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_FUNCTRLCLR,
					OPMODE1 | XCVRSELECT1 | TERMSELECT);
	if (retval)
		return retval;

	/* Disable DP pulldown to allow weak Idp_src for DCD */
	retval = penwell_otg_ulpi_write(iotg, ULPI_OTGCTRLCLR, DPPULLDOWN);
	if (retval)
		return retval;

	/* Enable SW control */
	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, SWCNTRL);
	if (retval)
		return retval;

	/* Clear HWDETECT result for safety */
	penwell_otg_charger_hwdet(false);

	/* Enable IDPSRC */
	retval = penwell_otg_ulpi_write(iotg, ULPI_VS3SET, CHGD_IDP_SRC);
	if (retval)
		return retval;

	/* Check DCD result, use same polling parameter */
	timeout = jiffies + msecs_to_jiffies(DATACON_TIMEOUT);
	interval = DATACON_INTERVAL * 1000; /* us */

	while (!time_after(jiffies, timeout)) {
		retval = penwell_otg_ulpi_read(iotg, ULPI_VS4, &data);
		if (retval) {
			dev_warn(pnw->dev, "Failed to read ULPI register\n");
			return retval;
		}

		if (!(data & CHRG_SERX_DP)) {
			dev_info(pnw->dev, "Data contact detected!\n");
			break;
		}

		/* Polling interval */
		usleep_range(interval, interval + 2000);
	}

	/* ulpi_write(0x87, 0x40)*/
	retval = penwell_otg_ulpi_write(iotg, ULPI_VS3CLR, CHGD_IDP_SRC);
	if (retval)
		return retval;

	dev_info(pnw->dev, "DCD complete\n");

	return 0;
}

static int penwell_otg_charger_det_aca_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;
	u8				data;
	u8				usb_vs2_sts = 0;
	u8				usb_vs2_latch = 0;
	u8				usb_vdat_det = 0;
	u8				usb_vdm = 0;

	retval = penwell_otg_ulpi_read(iotg, ULPI_VS2LATCH, &usb_vs2_latch);
	dev_dbg(pnw->dev, "%s: usb_vs2_latch = 0x%x\n",
			__func__, usb_vs2_latch);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return retval;
	}

	retval = penwell_otg_ulpi_read(iotg, ULPI_VS2STS, &usb_vs2_sts);
	dev_dbg(pnw->dev, "%s: usb_vs2_sts = 0x%x\n",
			__func__, usb_vs2_sts);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return retval;
	}

	if (usb_vs2_latch & IDRARBRC_MSK) {
		switch (IDRARBRC_STS(usb_vs2_sts)) {
		case IDRARBRC_A:
			iotg->hsm.id = ID_ACA_A;
			break;
		case IDRARBRC_B:
			iotg->hsm.id = ID_ACA_B;
			dev_info(pnw->dev, "ACA-B detected\n");
			break;
		case IDRARBRC_C:
			iotg->hsm.id = ID_ACA_C;
			dev_info(pnw->dev, "ACA-C detected\n");
			break;
		default:
			break;
		}
	}

	if (iotg->hsm.id == ID_ACA_A) {
		retval = penwell_otg_ulpi_write(iotg,
					ULPI_PWRCTRLSET, DPVSRCEN);
		if (retval)
			return retval;

		msleep(70);

		retval = penwell_otg_ulpi_read(iotg, ULPI_PWRCTRL, &data);
		usb_vdat_det = data & VDATDET;
		if (retval) {
			dev_warn(pnw->dev, "ULPI read failed, exit\n");
			return retval;
		}

		retval = penwell_otg_ulpi_read(iotg, ULPI_VS4, &data);
		usb_vdm = data & CHRG_SERX_DM;
		if (retval) {
			dev_warn(pnw->dev, "ULPI read failed, exit\n");
			return retval;
		}

		retval = penwell_otg_ulpi_write(iotg,
					 ULPI_PWRCTRLCLR, DPVSRCEN);
		if (retval)
			return retval;

		if (usb_vdat_det && !usb_vdm)
			dev_info(pnw->dev, "ACA-Dock detected\n");
		else if (!usb_vdat_det && usb_vdm)
			dev_info(pnw->dev, "ACA-A detected\n");
	}

	if (iotg->hsm.id == ID_ACA_A || iotg->hsm.id == ID_ACA_B
			|| iotg->hsm.id == ID_ACA_C) {
		return CHRG_ACA;
	}

	return 0;
}

static int penwell_otg_charger_det_se1_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;
	u8				data;

	retval = penwell_otg_ulpi_write(iotg, ULPI_OTGCTRLSET,
						DMPULLDOWN | DPPULLDOWN);
	if (retval)
		return retval;

	msleep(50);
	retval = penwell_otg_ulpi_read(iotg, ULPI_DEBUG, &data);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return -EBUSY;
	}

	if ((data & CHRG_SERX_DP) && (data & CHRG_SERX_DM))
		return CHRG_SE1;

	return 0;
}

static int penwell_otg_charger_det_pri_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;
	u8				vdat_det, serx_dm;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, DPVSRCEN);
	if (retval)
		return retval;

	msleep(110);

	retval = penwell_otg_ulpi_read(iotg, ULPI_PWRCTRL, &vdat_det);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return -EBUSY;
	}

	retval = penwell_otg_ulpi_read(iotg, ULPI_VS4, &serx_dm);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return -EBUSY;
	}

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	vdat_det &= VDATDET;
	serx_dm &= CHRG_SERX_DM;

	if ((!vdat_det) || serx_dm)
		return CHRG_SDP;

	return 0;
}

static int penwell_otg_charger_det_sec_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;
	u8				vdat_det;

	usleep_range(1000, 1500);

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS1CLR, DATAPOLARITY);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, DPVSRCEN);
	if (retval)
		return retval;

	msleep(80);

	retval = penwell_otg_ulpi_read(iotg, ULPI_PWRCTRL, &vdat_det);
	if (retval) {
		dev_warn(pnw->dev, "ULPI read failed, exit\n");
		return retval;
	}

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR, DPVSRCEN);
	if (retval)
		return retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS1SET, DATAPOLARITY);
	if (retval)
		return retval;

	vdat_det &= VDATDET;

	if (vdat_det)
		return CHRG_DCP;
	else
		return CHRG_CDP;

	return 0;
}

static int penwell_otg_charger_det_clean_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;

	retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET,
					DPVSRCEN | SWCNTRL);
	if (retval)
		return retval;

	return 0;
}

/* As we use SW mode to do charger detection, need to notify HW
 * the result SW get, charging port or not */
static int penwell_otg_charger_hwdet(bool enable)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;

	/* This is for CLV only */
	if (!is_clovertrail(to_pci_dev(pnw->dev)))
		return 0;

	if (enable) {
		retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLSET, HWDET);
		if (retval)
			return retval;
		dev_dbg(pnw->dev, "set HWDETECT\n");
	} else {
		retval = penwell_otg_ulpi_write(iotg, ULPI_PWRCTRLCLR, HWDET);
		if (retval)
			return retval;
		dev_dbg(pnw->dev, "clear HWDETECT\n");
	}

	return 0;
}

static int penwell_otg_charger_det_clt(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	int				retval;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	/* DCD */
	retval = penwell_otg_charger_det_dcd_clt();
	if (retval) {
		dev_warn(pnw->dev, "DCD failed, exit\n");
		return CHRG_UNKNOWN;
	}

	/* ACA Detection */
	retval = penwell_otg_charger_det_aca_clt();
	if (retval < 0) {
		dev_warn(pnw->dev, "ACA Det failed, exit\n");
		return CHRG_UNKNOWN;
	} else if (retval == CHRG_ACA) {
		dev_info(pnw->dev, "ACA detected\n");
		penwell_otg_charger_hwdet(true);
		return CHRG_ACA;
	}

	/* SE1 Detection */
	retval = penwell_otg_charger_det_se1_clt();
	if (retval < 0) {
		dev_warn(pnw->dev, "SE1 Det failed, exit\n");
		return CHRG_UNKNOWN;
	} else if (retval == CHRG_SE1) {
		dev_info(pnw->dev, "SE1 detected\n");
		penwell_otg_charger_hwdet(true);
		return CHRG_SE1;
	}

	/* Pri Det */
	retval = penwell_otg_charger_det_pri_clt();
	if (retval < 0) {
		dev_warn(pnw->dev, "Pri Det failed, exit\n");
		return CHRG_UNKNOWN;
	} else if (retval == CHRG_SDP) {
		dev_info(pnw->dev, "SDP detected\n");
		return CHRG_SDP;
	}

	/* Sec Det */
	retval = penwell_otg_charger_det_sec_clt();
	if (retval < 0) {
		dev_warn(pnw->dev, "Sec Det failed, exit\n");
		return CHRG_UNKNOWN;
	} else if (retval == CHRG_CDP) {
		dev_info(pnw->dev, "CDP detected\n");
		penwell_otg_charger_hwdet(true);
		return CHRG_CDP;
	} else  if (retval == CHRG_DCP) {
		dev_info(pnw->dev, "DCP detected\n");
		penwell_otg_charger_det_clean_clt();
		penwell_otg_charger_hwdet(true);
		return CHRG_DCP;
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return 0;
}

void penwell_otg_phy_vbus_wakeup(bool on)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct intel_mid_otg_xceiv  *iotg = &pnw->iotg;
	u8			flag = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	penwell_otg_msic_spi_access(true);

	flag = VBUSVLD | SESSVLD | SESSEND;

	if (is_clovertrail(to_pci_dev(pnw->dev))) {
		if (on) {
			penwell_otg_ulpi_write(iotg, ULPI_USBINTEN_RISINGSET,
						flag);
			penwell_otg_ulpi_write(iotg, ULPI_USBINTEN_FALLINGSET,
						flag);
		} else {
			penwell_otg_ulpi_write(iotg, ULPI_USBINTEN_RISINGCLR,
						flag);
			penwell_otg_ulpi_write(iotg, ULPI_USBINTEN_FALLINGCLR,
						flag);
		}
	} else {
		if (on) {
			penwell_otg_msic_write(MSIC_USBINTEN_RISESET, flag);
			penwell_otg_msic_write(MSIC_USBINTEN_FALLSET, flag);
		} else {
			penwell_otg_msic_write(MSIC_USBINTEN_RISECLR, flag);
			penwell_otg_msic_write(MSIC_USBINTEN_FALLCLR, flag);
		}
	}

	penwell_otg_msic_spi_access(false);

	dev_dbg(pnw->dev, "%s --->\n", __func__);
}

void penwell_otg_phy_intr(bool on)
{
	struct penwell_otg	*pnw = the_transceiver;
	u8			flag = 0;
	int			retval;
	u8			data;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	penwell_otg_msic_spi_access(true);

	flag = VBUSVLD | IDGND;

	if (on) {
		dev_info(pnw->dev, "enable VBUSVLD & IDGND\n");
		penwell_otg_msic_write(MSIC_USBINTEN_RISESET, flag);
		penwell_otg_msic_write(MSIC_USBINTEN_FALLSET, flag);
	} else {
		dev_info(pnw->dev, "disable VBUSVLD & IDGND\n");
		penwell_otg_msic_write(MSIC_USBINTEN_RISECLR, flag);
		penwell_otg_msic_write(MSIC_USBINTEN_FALLCLR, flag);
	}

	retval = intel_scu_ipc_ioread8(MSIC_USBINTEN_RISE, &data);
	if (retval)
		dev_warn(pnw->dev, "Failed to read MSIC register\n");
	else
		dev_info(pnw->dev, "MSIC_USBINTEN_RISE = 0x%x", data);

	retval = intel_scu_ipc_ioread8(MSIC_USBINTEN_FALL, &data);
	if (retval)
		dev_warn(pnw->dev, "Failed to read MSIC register\n");
	else
		dev_info(pnw->dev, "MSIC_USBINTEN_FALL = 0x%x", data);

	penwell_otg_msic_spi_access(false);
}

static penwell_otg_notify_warning(int warning_code)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct usb_hcd		*hcd;
	int			err;

	dev_dbg(pnw->dev, "%s ---> %d\n", __func__, warning_code);

	if (pnw && pnw->iotg.otg.host && pnw->iotg.otg.host->root_hub)
		usb_notify_warning(pnw->iotg.otg.host->root_hub, warning_code);
	else
		dev_dbg(pnw->dev, "no valid device for notification\n");

	dev_dbg(pnw->dev, "%s <--- %d\n", __func__);
}

void penwell_otg_nsf_msg(unsigned long indicator)
{
	switch (indicator) {
	case 2:
	case 4:
	case 6:
	case 7:
		dev_warn(the_transceiver->dev,
			"NSF-%lu - device not responding\n", indicator);
		break;
	case 3:
		dev_warn(the_transceiver->dev,
			"NSF-%lu - device not supported\n", indicator);
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

	if (timer_pending(&pnw->hsm_timer))
		return ;

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
	/* support OTG test mode */
	case TTST_MAINT_TMR:
		iotg->hsm.tst_maint_tmout = 0;
		data = (unsigned long)&iotg->hsm.tst_maint_tmout;
		time = TTST_MAINT;
		dev_dbg(pnw->dev,
			"Add timer TTST_MAINT = %d\n", TTST_MAINT);
		break;
	case TTST_NOADP_TMR:
		iotg->hsm.tst_noadp_tmout = 0;
		data = (unsigned long)&iotg->hsm.tst_noadp_tmout;
		time = TTST_NOADP;
		dev_dbg(pnw->dev,
			"Add timer TTST_NOADP = %d\n", TTST_NOADP);
		break;
	default:
		dev_dbg(pnw->dev,
			"unkown timer, can not enable such timer\n");
		return;
	}

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
	case TA_WAIT_BCON_TMR:
		iotg->hsm.a_wait_bcon_tmout = 0;
		break;
	case TTST_MAINT_TMR:
		iotg->hsm.tst_maint_tmout = 0;
		break;
	case TTST_NOADP_TMR:
		iotg->hsm.tst_noadp_tmout = 0;
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

static void pnw_phy_ctrl_rst(void)
{
	struct penwell_otg *pnw = the_transceiver;
	unsigned long	flags;

	spin_lock_irqsave(&pnw->lock, flags);

	/* mask id intr before reset and delay for 4 ms
	* before unmasking id intr to avoid wrongly
	* detecting ID_A which is a side-effect of reset
	* PHY
	*/
	penwell_otg_id(0);
	gpio_direction_output(pnw->otg_pdata->gpio_reset, 0);
	udelay(500);
	gpio_set_value(pnw->otg_pdata->gpio_reset, 1);

	reset_otg();

	udelay(4000);
	penwell_otg_id(1);

	spin_unlock_irqrestore(&pnw->lock, flags);
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
	iotg->hsm.power_up = 0;
	/* defautly don't request bus as B device */
	iotg->hsm.b_bus_req = 0;
	/* no system error */
	iotg->hsm.a_clr_err = 0;

	if (iotg->otg.state == OTG_STATE_A_IDLE) {
		wake_lock(&pnw->wake_lock);
		pm_runtime_get(pnw->dev);
	}

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

static void penwell_otg_eye_diagram_optimize(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval;

	/* Set 0x77 for better quality in eye diagram
	 * It means ZHSDRV = 0b11 and IHSTX = 0b0111 */

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS1SET, 0x77);
	if (retval)
		dev_warn(pnw->dev, "eye diagram optimize failed with ulpi failure\n");
}

static irqreturn_t otg_dummy_irq(int irq, void *_dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	void __iomem		*reg_base = _dev;
	u32			val;
	u32			int_mask = 0;

	val = readl(reg_base + CI_USBMODE);
	if ((val & USBMODE_CM) != USBMODE_DEVICE)
		return IRQ_NONE;

	val = readl(reg_base + CI_USBSTS);
	int_mask = val & INTR_DUMMY_MASK;

	if (int_mask == 0)
		return IRQ_NONE;

	/* clear hsm.b_conn here since host driver can't detect it
	*  otg_dummy_irq called means B-disconnect happened.
	*/
	if (pnw->iotg.hsm.b_conn) {
		pnw->iotg.hsm.b_conn = 0;
		penwell_update_transceiver();
	}

	/* Clear interrupts */
	writel(int_mask, reg_base + CI_USBSTS);
	return IRQ_HANDLED;
}

static irqreturn_t otg_irq_handle(struct penwell_otg *pnw,
					struct intel_mid_otg_xceiv *iotg)
{
	int				id = 0;
	int				flag = 0;
	u32				int_sts, int_en, int_mask = 0;
	u8				usb_vs2_latch = 0;
	u8				usb_vs2_sts = 0;
	struct iotg_ulpi_access_ops	*ops;

	/* Check VBUS/SRP interrup */
	int_sts = readl(pnw->iotg.base + CI_OTGSC);
	int_en = (int_sts & OTGSC_INTEN_MASK) >> 8;
	int_mask = int_sts & int_en;

	if (int_mask == 0)
		return IRQ_NONE;

	ops = &iotg->ulpi_ops;
	ops->read(iotg, ULPI_VS2LATCH, &usb_vs2_latch);
	dev_dbg(pnw->dev, "usb_vs2_latch = 0x%x\n", usb_vs2_latch);
	if (usb_vs2_latch & IDRARBRC_MSK) {
		ops->read(iotg, ULPI_VS2STS, &usb_vs2_sts);
		dev_dbg(pnw->dev, "%s: usb_vs2_sts = 0x%x\n",
				__func__, usb_vs2_sts);

		switch (IDRARBRC_STS(usb_vs2_sts)) {
		case IDRARBRC_A:
			id = ID_ACA_A;
			dev_dbg(pnw->dev, "ACA-A interrupt detected\n");
			break;
		case IDRARBRC_B:
			id = ID_ACA_B;
			dev_dbg(pnw->dev, "ACA-B interrupt detected\n");
			break;
		case IDRARBRC_C:
			id = ID_ACA_C;
			dev_dbg(pnw->dev, "ACA-C interrupt detected\n");
			break;
		default:
			break;
		}

		if (id) {
			iotg->hsm.id = id;
			flag = 1;
			dev_dbg(pnw->dev, "%s: id change int = %d\n",
					__func__, iotg->hsm.id);
		} else {
			iotg->hsm.id = (int_sts & OTGSC_ID) ?
				ID_B : ID_A;
			flag = 1;
			dev_dbg(pnw->dev, "%s: id change int = %d\n",
					__func__, iotg->hsm.id);
		}
	}

	if (int_mask) {
		dev_dbg(pnw->dev,
			"OTGSC = 0x%x, mask =0x%x\n", int_sts, int_mask);

		if (int_mask & OTGSC_IDIS) {
			if (!id)
				iotg->hsm.id = (int_sts & OTGSC_ID) ?
					ID_B : ID_A;
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

static irqreturn_t otg_irq(int irq, void *_dev)
{
	struct penwell_otg		*pnw = _dev;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

#ifdef CONFIG_PM_RUNTIME
	if (pnw->rt_resuming)
		return IRQ_HANDLED;
	else if (pnw->dev->power.runtime_status == RPM_RESUMING)
		return IRQ_HANDLED;

	/* If it's not active, resume device first before access regs */
	if (pnw->dev->power.runtime_status != RPM_ACTIVE) {
		dev_dbg(pnw->dev, "Wake up? Interrupt detected in suspended\n");
		pnw->rt_resuming = 1;

		pm_runtime_get(pnw->dev);

		return IRQ_HANDLED;
	}
#endif /* CONFIG_PM_RUNTIME */

	return otg_irq_handle(pnw, iotg);
}

static void penwell_otg_check_wakeup_event(struct penwell_otg *pnw)
{
	u32		int_sts, int_en, int_mask = 0;
	int		flag = 0;
	int		id, a_vbus_vld, b_sess_end;
	int		b_sess_vld, a_sess_vld;

	pnw = the_transceiver;

	/* OTGSC needs 1ms debounce time to get synced with MSIC
	 * after back from clock gated state to D0, sleep 2ms
	 * will be enough */
	usleep_range(1800, 2000);

	/* Check VBUS/SRP interrup */
	int_sts = readl(pnw->iotg.base + CI_OTGSC);
	int_en = (int_sts & OTGSC_INTEN_MASK) >> 8;
	int_mask = int_sts & int_en;

	/* won't use OTGSC INTSTS bit, because controller is
	 * reset during runtime suspend, just check current status */

	/* get current id/a_vbus_vld/b_sess_end/b_sess_vld/a_sess_vld */
	id = !!(int_sts & OTGSC_ID) ? ID_B : ID_A;
	b_sess_end = !!(int_sts & OTGSC_BSE);
	b_sess_vld = !!(int_sts & OTGSC_BSV);
	a_sess_vld = !!(int_sts & OTGSC_ASV);
	a_vbus_vld = !!(int_sts & OTGSC_AVV);

	if (id != pnw->iotg.hsm.id) {
		pnw->iotg.hsm.id = id;
		dev_dbg(pnw->dev, "ID Wake up id = %d\n", id);
		flag = 1;
	}

	if (b_sess_end != pnw->iotg.hsm.b_sess_end) {
		pnw->iotg.hsm.b_sess_end = b_sess_end;
		dev_dbg(pnw->dev, "B_sess_end Wake up = %d\n", b_sess_end);
		flag = 1;
	}

	if (b_sess_vld != pnw->iotg.hsm.b_sess_vld) {
		pnw->iotg.hsm.b_sess_vld = b_sess_vld;
		dev_dbg(pnw->dev, "B_sess_vld Wake up = %d\n", b_sess_vld);
		flag = 1;
	}

	if (a_sess_vld != pnw->iotg.hsm.a_sess_vld) {
		pnw->iotg.hsm.a_sess_vld = a_sess_vld;
		dev_dbg(pnw->dev, "A_sess_vld Wake up = %d\n", a_sess_vld);
		flag = 1;
	}

	if (a_vbus_vld != pnw->iotg.hsm.a_vbus_vld) {
		pnw->iotg.hsm.a_vbus_vld = a_vbus_vld;
		dev_dbg(pnw->dev, "A_vbus_vld Wake up = %d\n", a_vbus_vld);
		flag = 1;
	}

	if (int_mask)
		writel((int_sts & ~OTGSC_INTSTS_MASK) | int_mask,
				pnw->iotg.base + CI_OTGSC);
	if (flag)
		penwell_update_transceiver();
}

static void penwell_otg_start_ulpi_poll(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	int				retval = 0;

	retval = penwell_otg_ulpi_write(&pnw->iotg, 0x16, 0x5a);
	if (retval)
		dev_err(pnw->dev, "ulpi write in init failed\n");

	schedule_delayed_work(&pnw->ulpi_poll_work, HZ);
}

static void penwell_otg_continue_ulpi_poll(void)
{
	struct penwell_otg		*pnw = the_transceiver;

	schedule_delayed_work(&pnw->ulpi_poll_work, HZ);
}

static void penwell_otg_stop_ulpi_poll(void)
{
	struct penwell_otg		*pnw = the_transceiver;

	/* we have to use this version because this function can
	 * be called in irq handler */
	__cancel_delayed_work(&pnw->ulpi_poll_work);
}


static int penwell_otg_iotg_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = data;
	unsigned long			flags;
	int				flag = 0;
	struct pci_dev			*pdev;

	if (iotg == NULL)
		return NOTIFY_BAD;

	if (pnw == NULL)
		return NOTIFY_BAD;

	pdev = to_pci_dev(pnw->dev);

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
		break;
	case MID_OTG_NOTIFY_HRESUME:
		dev_dbg(pnw->dev, "PNW OTG Notify Host Bus resume Event\n");
		if (iotg->otg.default_a == 1 && iotg->hsm.a_bus_req == 0) {
			iotg->hsm.a_bus_req = 1;
			flag = 1;
		}
		break;
	case MID_OTG_NOTIFY_CSUSPEND:
		dev_dbg(pnw->dev, "PNW OTG Notify Client Bus suspend Event\n");
		if (iotg->otg.default_a == 1) {
			iotg->hsm.b_bus_suspend = 1;
			flag = 1;
		} else {
			penwell_otg_stop_ulpi_poll();
			if (iotg->hsm.a_bus_suspend == 0) {
				iotg->hsm.a_bus_suspend = 1;
				flag = 1;
			} else
				flag = 0;
		}
		break;
	case MID_OTG_NOTIFY_CRESUME:
		dev_dbg(pnw->dev, "PNW OTG Notify Client Bus resume Event\n");
		if (iotg->otg.default_a == 1) {
			/* in A_PERIPHERAL state */
			iotg->hsm.b_bus_suspend = 0;
			flag = 1;
		} else {
			/* in B_PERIPHERAL state */
			if (!is_clovertrail(pdev))
				penwell_otg_start_ulpi_poll();
			iotg->hsm.a_bus_suspend = 0;
			flag = 0;
		}
		break;
	case MID_OTG_NOTIFY_CRESET:
		dev_dbg(pnw->dev, "PNW OTG Notify Client Bus reset Event\n");
		penwell_otg_set_power(&pnw->iotg.otg, CHRG_CURR_SDP_SUSP);
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
	/* Test mode support */
	case MID_OTG_NOTIFY_TEST_MODE_START:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Client Testmode Start\n");
		iotg->hsm.in_test_mode = 1;
		flag = 0;
		break;
	case MID_OTG_NOTIFY_TEST_MODE_STOP:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Client Testmode Stop\n");
		iotg->hsm.in_test_mode = 0;
		flag = 0;
		break;
	case MID_OTG_NOTIFY_TEST_SRP_REQD:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Client SRP REQD\n");
		iotg->hsm.otg_srp_reqd = 1;
		flag = 1;
		break;
	case MID_OTG_NOTIFY_TEST:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Test device detected\n");
		iotg->hsm.test_device = 1;
		flag = 0;
		break;
	case MID_OTG_NOTIFY_TEST_VBUS_OFF:
		dev_dbg(pnw->dev, "PNW OTG Notfiy Test device Vbus off mode\n");
		iotg->hsm.test_device = 1;
		iotg->hsm.otg_vbus_off = 1;
		flag = 1;
		break;
	default:
		dev_dbg(pnw->dev, "PNW OTG Nofity unknown notify message\n");
		return NOTIFY_DONE;
	}

	spin_lock_irqsave(&pnw->notify_lock, flags);
	if (flag && pnw->queue_stop == 0)
		penwell_update_transceiver();
	spin_unlock_irqrestore(&pnw->notify_lock, flags);

	return NOTIFY_OK;
}

static void penwell_otg_hnp_poll_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	struct usb_device		*udev;
	int				err = 0;
	u8				data;

	if (iotg->otg.host && iotg->otg.host->root_hub) {
		udev = iotg->otg.host->root_hub->children[0];
	} else {
		dev_warn(pnw->dev, "no host or root_hub registered\n");
		return;
	}

	if (iotg->otg.state != OTG_STATE_A_HOST
		&& iotg->otg.state != OTG_STATE_B_HOST)
		return;

	if (!udev) {
		dev_warn(pnw->dev,
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
		/* start HNP sequence to switch role */
		dev_dbg(pnw->dev, "host_request_flag = 1\n");

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
		dev_dbg(pnw->dev, "host_request_flag = 0\n");
		penwell_otg_continue_hnp_poll(&pnw->iotg);
	}
}

static void penwell_otg_ulpi_check_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	u8				data;
	int				status;

	status = pm_runtime_get_sync(pnw->dev);
	if (status < 0) {
		dev_err(pnw->dev, "%s: pm_runtime_get_sync FAILED err = %d\n",
				__func__, status);
		pm_runtime_put_sync(pnw->dev);
		return;
	}

	if (penwell_otg_ulpi_read(iotg, 0x16, &data)) {
		dev_err(pnw->dev,
				"%s: [ ULPI hang ] detected\n"
				"reset PHY & ctrl to recover\n",
				 __func__);
		pnw_phy_ctrl_rst();
	}

	pm_runtime_put(pnw->dev);
}

static void penwell_otg_uevent_work(struct work_struct *work)
{
	struct penwell_otg	*pnw = the_transceiver;
	char *uevent_envp[2] = { "USB_INTR=BOGUS", NULL };

	printk(KERN_INFO"%s: send uevent USB_INTR=BOGUS\n", __func__);
	kobject_uevent_env(&pnw->dev->kobj, KOBJ_CHANGE, uevent_envp);
}

static void penwell_otg_ulpi_poll_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	u8				data;

	if (iotg->otg.state != OTG_STATE_B_PERIPHERAL)
		return;

	if (iotg->hsm.in_test_mode)
		return;

	if (penwell_otg_ulpi_read(iotg, 0x16, &data)) {
		dev_err(pnw->dev, "ulpi read time out by polling\n");
		iotg->hsm.ulpi_error = 1;
		iotg->hsm.ulpi_polling = 0;
		penwell_update_transceiver();
	} else if (data != 0x5A) {
		dev_err(pnw->dev, "ulpi read value incorrect by polling\n");
		iotg->hsm.ulpi_error = 1;
		iotg->hsm.ulpi_polling = 0;
		penwell_update_transceiver();
	} else {
		dev_dbg(pnw->dev, "ulpi fine by polling\n");
		iotg->hsm.ulpi_error = 0;
		iotg->hsm.ulpi_polling = 1;
		penwell_otg_continue_ulpi_poll();
	}
}

static void penwell_otg_psc_notify_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct power_supply_charger_cap	psc_cap;
	enum power_supply_charger_event chrg_event;
	unsigned long			flags;
	struct otg_bc_event		*event, *temp;

	spin_lock_irqsave(&pnw->charger_lock, flags);
	list_for_each_entry_safe(event, temp, &pnw->chrg_evt_queue, node) {
		list_del(&event->node);
		spin_unlock_irqrestore(&pnw->charger_lock, flags);

		spin_lock_irqsave(&pnw->cap_lock, flags);
		chrg_event = check_psc_event(pnw->psc_cap, event->cap);
		if (chrg_event == -1)
			dev_dbg(pnw->dev, "no need to notify\n");
		else {
			pnw->psc_cap = event->cap;
			pnw->psc_cap.chrg_evt = chrg_event;
			psc_cap = pnw->psc_cap;
		}
		spin_unlock_irqrestore(&pnw->cap_lock, flags);

		if (chrg_event != -1) {
			dev_dbg(pnw->dev, "notify power_supply event\n");
			dev_dbg(pnw->dev, "mA = %d\n", psc_cap.mA);
			dev_dbg(pnw->dev, "evt = %d\n", psc_cap.chrg_evt);
			dev_dbg(pnw->dev, "type = %s\n",
					psc_string(psc_cap.chrg_type));

			power_supply_charger_event(psc_cap);
		}

		kfree(event);
		spin_lock_irqsave(&pnw->charger_lock, flags);
	}
	spin_unlock_irqrestore(&pnw->charger_lock, flags);
}

static void penwell_otg_sdp_check_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct otg_bc_cap		cap;

	/* Need to handle MFLD/CLV differently per different interface */
	if (!is_clovertrail(to_pci_dev(pnw->dev))) {
		if (penwell_otg_query_charging_cap(&cap)) {
			dev_warn(pnw->dev, "SDP checking failed\n");
			return;
		}

		/* If current charging cap is still 100mA SDP,
		 * assume this is a invalid charger and do 500mA
		 * charging */
		if (cap.mA != 100 || cap.chrg_type != CHRG_SDP)
			return;
	} else
		return;

	dev_info(pnw->dev, "Notify invalid SDP at %dmA\n", CHRG_CURR_SDP_INVAL);
	penwell_otg_update_chrg_cap(CHRG_SDP_INVAL, CHRG_CURR_SDP_INVAL);
}

static void penwell_otg_work(struct work_struct *work)
{
	struct penwell_otg		*pnw = container_of(work,
					struct penwell_otg, work);
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	struct otg_hsm			*hsm = &iotg->hsm;
	enum usb_charger_type		charger_type;
	enum power_supply_type		ps_type;
	int				retval;
	struct pci_dev			*pdev;
	unsigned long			flags;

	dev_dbg(pnw->dev,
		"old state = %s\n", state_string(iotg->otg.state));

	pm_runtime_get_sync(pnw->dev);

	pdev = to_pci_dev(pnw->dev);

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

			/* Prevent device enter D0i1 or S3*/
			wake_lock(&pnw->wake_lock);
			pm_runtime_get(pnw->dev);

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
			/* Check if DCP is detected */
			spin_lock_irqsave(&pnw->charger_lock, flags);
			charger_type = pnw->charging_cap.chrg_type;
			ps_type = pnw->psc_cap.chrg_type;
			if (charger_type == CHRG_DCP ||
				ps_type == POWER_SUPPLY_TYPE_USB_DCP) {
				spin_unlock_irqrestore(&pnw->charger_lock,
						flags);
				break;
			}
			spin_unlock_irqrestore(&pnw->charger_lock, flags);

			penwell_otg_phy_low_power(0);

			/* Clear power_up */
			hsm->power_up = 0;

			/* Move to B_PERIPHERAL state, Session Valid */

			/* Delete current timer */
			penwell_otg_del_timer(TB_SRP_FAIL_TMR);

			hsm->b_sess_end = 0;
			hsm->a_bus_suspend = 0;

			/* Start USB Battery charger detection flow */

			/* We need new charger detection flow for Clovertrail.
			 * But for now(power-on), we just skip it.
			 * Later on we'll figure it out.
			 */
			if (!is_clovertrail(pdev)) {
				mutex_lock(&pnw->msic_mutex);
				if (pdev->revision >= 0x8) {
					retval = penwell_otg_manual_chrg_det();
					if (retval < 0) {
						/* if failed, reset controller
						 * and try charger detection
						 * flow again */
						dev_warn(pnw->dev,
								"detection failed, retry");
						set_client_mode();
						msleep(100);
						penwell_otg_phy_low_power(0);
						penwell_spi_reset_phy();
					retval = penwell_otg_manual_chrg_det();
					}
				} else {
					/* Enable data contact detection */
					penwell_otg_data_contact_detect();
					/* Enable charger detection */
					penwell_otg_charger_detect();
					retval =
					penwell_otg_charger_type_detect();
				}
				mutex_unlock(&pnw->msic_mutex);
				if (retval < 0) {
					dev_warn(pnw->dev, "Charger detect failure\n");
					break;
				} else {
					charger_type = retval;
				}
			} else {
				/* Clovertrail charger detection flow */
				retval = penwell_otg_charger_det_clt();
				if (retval < 0) {
					dev_warn(pnw->dev, "Charger detect failure\n");
					break;
				} else
					charger_type = retval;
			}

			/* This is a workaround for self-powered hub case,
			 * vbus valid event comes several ms before id change */
			if (hsm->id == ID_A) {
				dev_warn(pnw->dev, "ID changed\n");
				break;
			}

			if (charger_type == CHRG_DCP) {
				dev_info(pnw->dev, "DCP detected\n");

				/* DCP: set charger type, current, notify EM */
				penwell_otg_update_chrg_cap(CHRG_DCP,
							CHRG_CURR_DCP);
				set_client_mode();
				break;

			} else if (charger_type == CHRG_ACA) {
				dev_info(pnw->dev, "ACA detected\n");
				if (hsm->id == ID_ACA_A) {
					/* Move to A_IDLE state, ID changes */
					penwell_otg_update_chrg_cap(CHRG_ACA,
								CHRG_CURR_ACA);

					/* Delete current timer */
					penwell_otg_del_timer(TB_SRP_FAIL_TMR);

					iotg->otg.default_a = 1;
					hsm->a_srp_det = 0;
					set_host_mode();
					penwell_otg_phy_low_power(0);

					/* Always set a_bus_req to 1,
					 * in case no ADP */
					hsm->a_bus_req = 1;

					/* Prevent device enter D0i1 or S3*/
					wake_lock(&pnw->wake_lock);
					pm_runtime_get(pnw->dev);

					iotg->otg.state = OTG_STATE_A_IDLE;
					penwell_update_transceiver();
					break;
				} else if (hsm->id == ID_ACA_B) {
					penwell_otg_update_chrg_cap(CHRG_ACA,
								CHRG_CURR_ACA);

					/* make sure PHY low power state */
					penwell_otg_phy_low_power(1);
					break;
				} else if (hsm->id == ID_ACA_C) {
					penwell_otg_update_chrg_cap(CHRG_ACA,
								CHRG_CURR_ACA);
					/* Clear HNP polling flag */
					if (iotg->otg.gadget)
						iotg->otg.gadget->
							host_request_flag = 0;

					penwell_otg_phy_low_power(0);
					set_client_mode();

					if (iotg->start_peripheral) {
						iotg->start_peripheral(iotg);
					} else {
						dev_dbg(pnw->dev,
							"client driver not support\n");
						break;
					}
				}
			} else if (charger_type == CHRG_CDP) {
				dev_info(pnw->dev, "CDP detected\n");

				/* MFLD WA: MSIC issue need disable phy intr */
				if (!is_clovertrail(pdev)) {
					dev_dbg(pnw->dev,
						"MFLD WA: enable PHY int\n");
					penwell_otg_phy_intr(0);
				}

				/* CDP: set charger type, current, notify EM */
				penwell_otg_update_chrg_cap(CHRG_CDP,
							CHRG_CURR_CDP);

				/* Clear HNP polling flag */
				if (iotg->otg.gadget)
					iotg->otg.gadget->host_request_flag = 0;

				if (iotg->start_peripheral) {
					iotg->start_peripheral(iotg);
				} else {
					dev_dbg(pnw->dev,
						"client driver not support\n");
					break;
				}
			} else if (charger_type == CHRG_SDP) {
				dev_info(pnw->dev, "SDP detected\n");

				/* MFLD WA: MSIC issue need disable phy intr */
				if (!is_clovertrail(pdev)) {
					dev_dbg(pnw->dev,
						"MFLD WA: enable PHY int\n");
					penwell_otg_phy_intr(0);
				}

				/* SDP: set charger type and 100mA by default */
				penwell_otg_update_chrg_cap(CHRG_SDP, 100);

				/* Clear HNP polling flag */
				if (iotg->otg.gadget)
					iotg->otg.gadget->host_request_flag = 0;

				penwell_otg_phy_low_power(0);
				set_client_mode();

				if (iotg->start_peripheral) {
					iotg->start_peripheral(iotg);
				} else {
					dev_dbg(pnw->dev,
						"client driver not support\n");
					break;
				}

				/* Schedule the SDP checking after TIMEOUT */
				queue_delayed_work(pnw->qwork,
						&pnw->sdp_check_work,
						INVALID_SDP_TIMEOUT);
			} else if (charger_type == CHRG_UNKNOWN) {
				dev_info(pnw->dev, "Unknown Charger Found\n");

				/* Unknown: set charger type */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN, 0);
			}

			penwell_otg_eye_diagram_optimize();

			/* MFLD WA for PHY issue */
			iotg->hsm.in_test_mode = 0;
			iotg->hsm.ulpi_error = 0;

			if (!is_clovertrail(pdev))
				penwell_otg_start_ulpi_poll();

			iotg->otg.state = OTG_STATE_B_PERIPHERAL;

		} else if ((hsm->b_bus_req || hsm->power_up || hsm->adp_change
				|| hsm->otg_srp_reqd) && !hsm->b_srp_fail_tmr) {

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

			/* clear after SRP attemp */
			if (hsm->otg_srp_reqd) {
				dev_dbg(pnw->dev, "Test mode: SRP done\n");
				hsm->otg_srp_reqd = 0;
			}
		} else if (!hsm->b_sess_vld && hsm->id == ID_B) {
			spin_lock_irqsave(&pnw->charger_lock, flags);
			charger_type = pnw->charging_cap.chrg_type;
			ps_type = pnw->psc_cap.chrg_type;
			spin_unlock_irqrestore(&pnw->charger_lock, flags);

			if (charger_type == CHRG_DCP) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
						CHRG_CURR_DISCONN);

				retval = penwell_otg_ulpi_write(iotg,
						ULPI_PWRCTRLCLR, DPVSRCEN);
				if (retval)
					dev_warn(pnw->dev, "ulpi failed\n");
				penwell_otg_charger_hwdet(false);
			} else if (ps_type == POWER_SUPPLY_TYPE_USB_ACA) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
						CHRG_CURR_DISCONN);
				penwell_otg_charger_hwdet(false);
			} else if (ps_type == POWER_SUPPLY_TYPE_USB_DCP) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
						CHRG_CURR_DISCONN);
				penwell_otg_charger_hwdet(false);
			}
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		/* FIXME: Check if ID_ACA_A event will happened in this state */
		if (hsm->id == ID_A) {
			iotg->otg.default_a = 1;
			hsm->a_srp_det = 0;

			cancel_delayed_work_sync(&pnw->ulpi_poll_work);
			cancel_delayed_work_sync(&pnw->sdp_check_work);
			penwell_otg_charger_hwdet(false);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			set_host_mode();
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			/* Notify EM charger remove event */
			penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
						CHRG_CURR_DISCONN);

			/* Prevent device enter D0i1 or S3*/
			wake_lock(&pnw->wake_lock);
			pm_runtime_get(pnw->dev);

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (hsm->ulpi_error && !hsm->in_test_mode) {
			/* WA: try to recover once detected PHY issue */
			hsm->ulpi_error = 0;

			cancel_delayed_work_sync(&pnw->ulpi_poll_work);
			cancel_delayed_work_sync(&pnw->sdp_check_work);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);

			msleep(2000);

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);

			if (!is_clovertrail(pdev))
				penwell_otg_start_ulpi_poll();

		} else if (!hsm->b_sess_vld || hsm->id == ID_ACA_B) {
			/* Move to B_IDLE state, VBUS off/ACA */

			cancel_delayed_work(&pnw->ulpi_poll_work);
			cancel_delayed_work_sync(&pnw->sdp_check_work);

			hsm->b_bus_req = 0;

			if (is_clovertrail(pdev))
				schedule_delayed_work(&pnw->ulpi_check_work,
									HZ);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			/* MFLD WA: reenable it for unplug event */
			if (!is_clovertrail(pdev)) {
				dev_dbg(pnw->dev, "MFLD WA: disable PHY int\n");
				penwell_otg_phy_intr(1);
			}

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			else if (hsm->id == ID_B) {
				/* Notify EM charger remove event */
				penwell_otg_update_chrg_cap(CHRG_UNKNOWN,
							CHRG_CURR_DISCONN);
				penwell_otg_charger_hwdet(false);
			}

			iotg->otg.state = OTG_STATE_B_IDLE;
		} else if (hsm->b_bus_req && hsm->a_bus_suspend
				&& iotg->otg.gadget
				&& iotg->otg.gadget->b_hnp_enable) {

			penwell_otg_phy_low_power(0);
			msleep(10);

			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			penwell_otg_phy_low_power(0);

			hsm->a_conn = 0;
			hsm->a_bus_resume = 0;

			if (iotg->start_host) {
				iotg->start_host(iotg);
				hsm->test_device = 0;

				/* FIXME: can we allow D3 and D0i3
				* in B_WAIT_ACON?
				* Now just disallow it
				*/
				/* disallow D3 or D0i3 */
				pm_runtime_get(pnw->dev);
				wake_lock(&pnw->wake_lock);
				iotg->otg.state = OTG_STATE_B_WAIT_ACON;
				penwell_otg_add_timer(TB_ASE0_BRST_TMR);
			} else
				dev_dbg(pnw->dev, "host driver not loaded.\n");

		} else if (hsm->id == ID_ACA_C) {
			cancel_delayed_work_sync(&pnw->sdp_check_work);

			/* Make sure current limit updated */
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);
		} else if (hsm->id == ID_B) {
			if (iotg->otg.set_power)
				iotg->otg.set_power(&iotg->otg, 500);
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

			PNW_STOP_HOST(pnw);

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

			PNW_STOP_HOST(pnw);

			set_client_mode();
			penwell_otg_phy_low_power(1);

			/* allow D3 and D0i3 */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
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

			PNW_STOP_HOST(pnw);

			hsm->a_bus_suspend = 0;
			hsm->b_bus_req = 0;

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev, "client driver not loaded\n");

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
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

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			PNW_STOP_HOST(pnw);

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

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			PNW_STOP_HOST(pnw);

			set_client_mode();
			penwell_otg_phy_low_power(1);

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_B_IDLE;
		} else if (!hsm->b_bus_req || !hsm->a_conn
					|| hsm->test_device) {
			hsm->b_bus_req = 0;

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			PNW_STOP_HOST(pnw);
			hsm->a_bus_suspend = 0;

			/* Clear HNP polling flag */
			if (iotg->otg.gadget)
				iotg->otg.gadget->host_request_flag = 0;

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
						"client driver not loaded.\n");

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
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

			hsm->b_bus_req = 0;

			set_client_mode();
			msleep(5);
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_B_IDLE;
			penwell_update_transceiver();

			/* Decrement the device usage counter */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
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

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);

			iotg->otg.state = OTG_STATE_A_WAIT_BCON;

		} else if (!hsm->a_bus_drop && (hsm->power_up || hsm->a_bus_req
				|| hsm->a_srp_det || hsm->adp_change)) {
			/* power up / adp changes / srp detection should be
			 * cleared at once after handled. */
			if (hsm->power_up)
				hsm->power_up = 0;

			if (hsm->adp_change)
				hsm->adp_change = 0;

			if (hsm->a_srp_det) {
				hsm->a_srp_det = 0;
				/* wait SRP done, then enable VBUS */
				usleep_range(10000, 11000);
			}

			otg_set_vbus(&iotg->otg, true);

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
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (hsm->a_vbus_vld || hsm->a_wait_vrise_tmout
						|| hsm->id == ID_ACA_A) {
			/* Move to A_WAIT_BCON state, a vbus vld */
			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_WAIT_VRISE_TMR);

			if (!hsm->a_vbus_vld) {
				/* Turn off VBUS */
				otg_set_vbus(&iotg->otg, false);

				penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
				iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
				break;
			}

			if (hsm->id == ID_ACA_A) {
				otg_set_vbus(&iotg->otg, false);

				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);
			}

			hsm->a_bus_req = 1;
			hsm->b_conn = 0;
			hsm->hnp_poll_enable = 0;

			penwell_otg_eye_diagram_optimize();

			if (iotg->start_host) {
				dev_dbg(pnw->dev, "host_ops registered!\n");
				iotg->start_host(iotg);
			} else {
				dev_dbg(pnw->dev, "host driver not loaded.\n");
				break;
			}

			penwell_otg_add_timer(TA_WAIT_BCON_TMR);

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
			/* at least give some time to USB HOST to enumerate
			* devices before trying to suspend the system*/
			wake_lock_timeout(&pnw->wake_lock, 5 * HZ);

			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B || hsm->a_bus_drop ||
			hsm->a_wait_bcon_tmout) {
			/* Move to A_WAIT_VFALL state, user request */

			/* Delete current timer and clear flags for B-Device */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			hsm->b_bus_req = 0;

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state, over-current detected */

			/* Notify user space for vbus invalid event */
			penwell_otg_notify_warning(USB_WARNING_VBUS_INVALID);

			/* Delete current timer and disable host function */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS and enter PHY low power mode */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_phy_low_power(1);
			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (hsm->b_conn) {
			/* Move to A_HOST state, device connected */

			/* Delete current timer and disable host function */
			penwell_otg_del_timer(TA_WAIT_BCON_TMR);

			/* Start HNP polling */
			iotg->start_hnp_poll(iotg);

			if (!hsm->a_bus_req)
				hsm->a_bus_req = 1;

			if (hsm->test_device)
				penwell_otg_add_timer(TTST_MAINT_TMR);

			iotg->otg.state = OTG_STATE_A_HOST;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);
		}
		break;

	case OTG_STATE_A_HOST:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B || hsm->a_bus_drop) {
			/* Move to A_WAIT_VFALL state, timeout/user request */

			/* Delete current timer and clear flags */
			if (hsm->test_device) {
				hsm->test_device = 0;
				penwell_otg_del_timer(TTST_MAINT_TMR);
			}

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			penwell_otg_phy_low_power(0);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (hsm->test_device && hsm->tst_maint_tmout) {

			hsm->test_device = 0;

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			penwell_otg_phy_low_power(0);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			/* Clear states and wait for SRP */
			hsm->a_srp_det = 0;
			hsm->a_bus_req = 0;

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_IDLE;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state */

			/* Notify user space for vbus invalid event */
			penwell_otg_notify_warning(USB_WARNING_VBUS_INVALID);

			/* Delete current timer and clear flags */
			if (hsm->test_device) {
				hsm->test_device = 0;
				penwell_otg_del_timer(TTST_MAINT_TMR);
			}

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_phy_low_power(1);

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (!hsm->a_bus_req && iotg->otg.host->b_hnp_enable) {
			/* Move to A_SUSPEND state */

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			/* According to Spec 7.1.5 */
			penwell_otg_add_timer(TA_AIDL_BDIS_TMR);

			/* Set HABA to enable hardware assistance to
			 * signal A-connect after receiver B-disconnect
			 * Hardware will then set client mode and
			 * enable URE, SLE and PCE after the assistance
			 * otg_dummy_irq is used to clean these ints
			 * when client driver is not resumed.
			 */
			if (request_irq(pdev->irq, otg_dummy_irq,
					IRQF_SHARED, driver_name,
					iotg->base) != 0) {
					dev_dbg(pnw->dev,
					"request interrupt %d failed\n",
					pdev->irq);
			}
			penwell_otg_HABA(1);

			penwell_otg_loc_sof(0);
			penwell_otg_phy_low_power(0);

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_SUSPEND;
		} else if (!hsm->b_conn && hsm->test_device
						&& hsm->otg_vbus_off) {
			/* If it is a test device with otg_vbus_off bit set,
			 * turn off VBUS on disconnect event and stay for
			 * TTST_NOADP without ADP */

			penwell_otg_del_timer(TTST_MAINT_TMR);

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			penwell_otg_phy_low_power(0);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TTST_NOADP_TMR);

			/* disallow D3 or D0i3 */
			pm_runtime_get(pnw->dev);
			wake_lock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;

		} else if (!hsm->b_conn) {

			/* Delete current timer and clear flags */
			if (hsm->test_device) {
				hsm->test_device = 0;
				penwell_otg_del_timer(TTST_MAINT_TMR);
			}

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			/* add kernel timer */
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);
		} else if (hsm->id == ID_A) {
			/* Turn on VBUS */
			otg_set_vbus(&iotg->otg, true);
		}
		break;

	case OTG_STATE_A_SUSPEND:
		if (hsm->id == ID_B || hsm->id == ID_ACA_B ||
				hsm->a_bus_drop || hsm->a_aidl_bdis_tmout) {
			/* Move to A_WAIT_VFALL state, timeout/user request */
			penwell_otg_HABA(0);
			free_irq(pdev->irq, iotg->base);

			/* Delete current timer and clear HW assist */
			if (hsm->a_aidl_bdis_tmout)
				hsm->a_aidl_bdis_tmout = 0;
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			if (hsm->id == ID_ACA_B)
				penwell_otg_update_chrg_cap(CHRG_ACA,
							CHRG_CURR_ACA);

			/* Stop HNP polling */
			iotg->stop_hnp_poll(iotg);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);

			penwell_otg_add_timer(TA_WAIT_VFALL_TMR);
			iotg->otg.state = OTG_STATE_A_WAIT_VFALL;
		} else if (!hsm->a_vbus_vld) {
			/* Move to A_VBUS_ERR state, Over-current */
			penwell_otg_HABA(0);
			free_irq(pdev->irq, iotg->base);

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			PNW_STOP_HOST(pnw);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);
			penwell_otg_phy_low_power(1);
			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (!hsm->b_conn && !pnw->iotg.otg.host->b_hnp_enable) {
			/* Move to A_WAIT_BCON */

			/* delete current timer */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			/* add kernel timer */
			penwell_otg_add_timer(TA_WAIT_BCON_TMR);

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (!hsm->b_conn && pnw->iotg.otg.host->b_hnp_enable) {
			/* Move to A_PERIPHERAL state, HNP */
			penwell_otg_HABA(0);
			free_irq(pdev->irq, iotg->base);

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);
			penwell_otg_phy_low_power(0);

			PNW_STOP_HOST(pnw);

			penwell_otg_phy_low_power(0);
			hsm->b_bus_suspend = 0;

			/* Clear HNP polling flag */
			if (iotg->otg.gadget)
				iotg->otg.gadget->host_request_flag = 0;

			penwell_otg_phy_low_power(0);

			if (iotg->start_peripheral)
				iotg->start_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
						"client driver not loaded.\n");

			penwell_otg_add_timer(TA_BIDL_ADIS_TMR);
			iotg->otg.state = OTG_STATE_A_PERIPHERAL;
		} else if (hsm->a_bus_req) {
			/* Move to A_HOST state, user request */
			penwell_otg_HABA(0);
			free_irq(pdev->irq, iotg->base);

			/* Delete current timer and clear flags */
			penwell_otg_del_timer(TA_AIDL_BDIS_TMR);

			penwell_otg_loc_sof(1);

			/* Start HNP polling */
			iotg->start_hnp_poll(iotg);

			/* allow D3 and D0i3 in A_HOST */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_HOST;
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);
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
			otg_set_vbus(&iotg->otg, false);
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
			otg_set_vbus(&iotg->otg, false);
			penwell_otg_phy_low_power(1);

			iotg->otg.state = OTG_STATE_A_VBUS_ERR;
		} else if (hsm->a_bidl_adis_tmout) {
			/* Move to A_WAIT_BCON state */
			hsm->a_bidl_adis_tmr = 0;

			msleep(10);
			penwell_otg_phy_low_power(0);

			/* Disable client function and switch to host mode */
			if (iotg->stop_peripheral)
				iotg->stop_peripheral(iotg);
			else
				dev_dbg(pnw->dev,
					"client driver has been removed.\n");

			hsm->hnp_poll_enable = 0;
			hsm->b_conn = 0;

			penwell_otg_phy_low_power(0);

			if (iotg->start_host)
				iotg->start_host(iotg);
			else
				dev_dbg(pnw->dev,
						"host driver not loaded.\n");

			penwell_otg_add_timer(TA_WAIT_BCON_TMR);

			/* allow D3 and D0i3 in A_WAIT_BCON */
			pm_runtime_put(pnw->dev);
			wake_unlock(&pnw->wake_lock);
			iotg->otg.state = OTG_STATE_A_WAIT_BCON;
		} else if (hsm->id == ID_A && hsm->b_bus_suspend) {
			if (!timer_pending(&pnw->hsm_timer))
				penwell_otg_add_timer(TA_BIDL_ADIS_TMR);
		} else if (hsm->id == ID_A && !hsm->b_bus_suspend) {
			penwell_otg_del_timer(TA_BIDL_ADIS_TMR);
		} else if (hsm->id == ID_ACA_A) {
			penwell_otg_update_chrg_cap(CHRG_ACA, CHRG_CURR_ACA);

			/* Turn off VBUS */
			otg_set_vbus(&iotg->otg, false);
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
			hsm->a_wait_vfall_tmout = 0;

			/* Move to A_IDLE state, vbus falls */
			penwell_otg_phy_low_power(1);

			/* Always set a_bus_req to 1, in case no ADP */
			hsm->a_bus_req = 1;

			iotg->otg.state = OTG_STATE_A_IDLE;
			penwell_update_transceiver();
		} else if (hsm->test_device && hsm->otg_vbus_off
					&& hsm->tst_noadp_tmout) {
			/* After noadp timeout, switch back to normal mode */
			hsm->test_device = 0;
			hsm->otg_vbus_off = 0;
			hsm->tst_noadp_tmout = 0;

			penwell_otg_phy_low_power(1);

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

	pm_runtime_get_sync(pnw->dev);

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

	pm_runtime_put_sync(pnw->dev);

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
		"tst_maint_tmout = \t%d\n"
		"tst_noadp_tmout = \t%d\n"
		"a_bus_drop = \t%d\n"
		"a_bus_req = \t%d\n"
		"a_clr_err = \t%d\n"
		"b_bus_req = \t%d\n"
		"ulpi_error = \t%d\n"
		"ulpi_polling = \t%d\n",
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
		iotg->hsm.tst_maint_tmout,
		iotg->hsm.tst_noadp_tmout,
		iotg->hsm.a_bus_drop,
		iotg->hsm.a_bus_req,
		iotg->hsm.a_clr_err,
		iotg->hsm.b_bus_req,
		iotg->hsm.ulpi_error,
		iotg->hsm.ulpi_polling
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(hsm, S_IRUGO | S_IWUSR | S_IWGRP, show_hsm, NULL);

static ssize_t
show_chargers(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct penwell_otg		*pnw = the_transceiver;
	char				*next;
	unsigned			size, t;
	enum usb_charger_type		type;
	unsigned int			mA;
	unsigned long			flags;
	struct pci_dev			*pdev;
	struct power_supply_charger_cap	psc_cap;

	pdev = to_pci_dev(pnw->dev);

	next = buf;
	size = PAGE_SIZE;

	if (!is_clovertrail(pdev)) {
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
	} else {
		spin_lock_irqsave(&pnw->charger_lock, flags);
		psc_cap = pnw->psc_cap;
		spin_unlock_irqrestore(&pnw->charger_lock, flags);

		t = scnprintf(next, size,
			"USB Battery Charging Capability(CLV)\n"
			"\tUSB Charger Type:  %s\n"
			"\tMax Charging Current:  %u\n",
			psc_string(psc_cap.chrg_type),
			psc_cap.mA
		);

	}
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
static DEVICE_ATTR(a_bus_req, S_IRUGO | S_IWUSR | S_IWGRP,
				   get_a_bus_req, set_a_bus_req);

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
static DEVICE_ATTR(a_bus_drop, S_IRUGO | S_IWUSR | S_IWGRP,
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

		if (iotg->otg.gadget)
			iotg->otg.gadget->host_request_flag = 0;
	} else if (buf[0] == '1') {
		iotg->hsm.b_bus_req = 1;
		dev_dbg(pnw->dev, "b_bus_req = 1\n");

		if (iotg->otg.state == OTG_STATE_B_PERIPHERAL) {
			if (iotg->otg.gadget)
				iotg->otg.gadget->host_request_flag = 1;

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
static DEVICE_ATTR(b_bus_req, S_IRUGO | S_IWUSR | S_IWGRP,
				   get_b_bus_req, set_b_bus_req);

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
static DEVICE_ATTR(a_clr_err, S_IRUGO | S_IWUSR | S_IWGRP, NULL, set_a_clr_err);

static ssize_t
set_ulpi_err(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;

	dev_dbg(pnw->dev, "trigger ulpi error manually\n");

	iotg->hsm.ulpi_error = 1;

	penwell_update_transceiver();

	return count;
}
static DEVICE_ATTR(ulpi_err, S_IRUGO | S_IWUSR | S_IWGRP, NULL, set_ulpi_err);

static struct attribute *inputs_attrs[] = {
	&dev_attr_a_bus_req.attr,
	&dev_attr_a_bus_drop.attr,
	&dev_attr_b_bus_req.attr,
	&dev_attr_a_clr_err.attr,
	&dev_attr_ulpi_err.attr,
	NULL,
};

static struct attribute_group debug_dev_attr_group = {
	.name = "inputs",
	.attrs = inputs_attrs,
};

static int penwell_otg_aca_enable(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval = 0;
	struct pci_dev			*pdev;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	pdev = to_pci_dev(pnw->dev);

	if (!is_clovertrail(pdev)) {
		penwell_otg_msic_spi_access(true);

		retval = intel_scu_ipc_update_register(SPI_TI_VS4,
				TI_ACA_DET_EN, TI_ACA_DET_EN);
		if (retval)
			goto done;

		retval = intel_scu_ipc_update_register(SPI_TI_VS5,
				TI_ID_FLOAT_EN | TI_ID_RES_EN,
				TI_ID_FLOAT_EN | TI_ID_RES_EN);
		if (retval)
			goto done;
	} else {
		retval = penwell_otg_ulpi_write(iotg, ULPI_VS4SET,
				ACADET);
		if (retval)
			goto done;

		retval = penwell_otg_ulpi_write(iotg, ULPI_VS5SET,
				IDFLOAT_EN | IDRES_EN);
	}
done:
	if (!is_clovertrail(pdev))
		penwell_otg_msic_spi_access(false);

	if (retval)
		dev_warn(pnw->dev, "Failed to enable ACA device detection\n");

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return retval;
}

static int penwell_otg_aca_disable(void)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				retval = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS5CLR,
			IDFLOAT_EN | IDRES_EN);
	if (retval)
		goto done;

	retval = penwell_otg_ulpi_write(iotg, ULPI_VS4CLR,
			ACADET);

done:
	if (retval)
		dev_warn(pnw->dev, "Failed to disable ACA device detection\n");

	dev_dbg(pnw->dev, "%s <---\n", __func__);

	return retval;
}

static void penwell_spi_reset_phy(void)
{
	struct penwell_otg	*pnw = the_transceiver;

	dev_dbg(pnw->dev, "Reset Phy over SPI\n");
	penwell_otg_msic_spi_access(true);
	penwell_otg_msic_write(MSIC_FUNCTRLSET, PHYRESET);
	penwell_otg_msic_spi_access(false);
	dev_dbg(pnw->dev, "Reset Phy over SPI Done\n");
}

static int penwell_otg_probe(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	unsigned long		resource, len;
	void __iomem		*base = NULL;
	int			retval;
	u32			val32;
	struct penwell_otg	*pnw;
	char			qname[] = "penwell_otg_queue";
	char			chrg_qname[] = "penwell_otg_chrg_queue";

	retval = 0;

	dev_info(&pdev->dev, "Intel Penwell USB OTG controller is detected.\n");
	dev_info(&pdev->dev, "Driver version: " DRIVER_VERSION "\n");

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

	pnw->chrg_qwork = create_singlethread_workqueue(chrg_qname);
	if (!pnw->chrg_qwork) {
		dev_dbg(&pdev->dev, "cannot create workqueue %s\n", chrg_qname);
		retval = -ENOMEM;
		goto err;
	}

	INIT_LIST_HEAD(&pnw->chrg_evt_queue);
	INIT_WORK(&pnw->work, penwell_otg_work);
	INIT_WORK(&pnw->psc_notify, penwell_otg_psc_notify_work);
	INIT_WORK(&pnw->hnp_poll_work, penwell_otg_hnp_poll_work);
	INIT_WORK(&pnw->uevent_work, penwell_otg_uevent_work);
	INIT_DELAYED_WORK(&pnw->ulpi_poll_work, penwell_otg_ulpi_poll_work);
	INIT_DELAYED_WORK(&pnw->ulpi_check_work, penwell_otg_ulpi_check_work);
	INIT_DELAYED_WORK(&pnw->sdp_check_work, penwell_otg_sdp_check_work);

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
	pnw->iotg.start_hnp_poll = penwell_otg_start_hnp_poll;
	pnw->iotg.stop_hnp_poll = penwell_otg_stop_hnp_poll;
	pnw->iotg.otg.state = OTG_STATE_UNDEFINED;
	pnw->rt_resuming = 0;
	pnw->queue_stop = 0;
	if (otg_set_transceiver(&pnw->iotg.otg)) {
		dev_dbg(pnw->dev, "can't set transceiver\n");
		retval = -EBUSY;
		goto err;
	}

	pnw->iotg.ulpi_ops.read = penwell_otg_ulpi_read;
	pnw->iotg.ulpi_ops.write = penwell_otg_ulpi_write;

	spin_lock_init(&pnw->iotg.hnp_poll_lock);
	spin_lock_init(&pnw->notify_lock);
	spin_lock_init(&pnw->lock);

	wake_lock_init(&pnw->wake_lock, WAKE_LOCK_SUSPEND, "pnw_wake_lock");

	init_timer(&pnw->hsm_timer);
	init_timer(&pnw->bus_mon_timer);
	init_timer(&pnw->hnp_poll_timer);
	init_completion(&pnw->adp.adp_comp);

	/* Battery Charging part */
	spin_lock_init(&pnw->charger_lock);
	spin_lock_init(&pnw->cap_lock);
	pnw->charging_cap.mA = CHRG_CURR_DISCONN;
	pnw->charging_cap.chrg_type = CHRG_UNKNOWN;
	pnw->charging_cap.current_event = USBCHRG_EVENT_DISCONN;
	pnw->psc_cap.mA = CHRG_CURR_DISCONN;
	pnw->psc_cap.chrg_type = POWER_SUPPLY_TYPE_BATTERY;
	pnw->psc_cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;

	ATOMIC_INIT_NOTIFIER_HEAD(&pnw->iotg.iotg_notifier);
	/* For generic otg notifications */
	ATOMIC_INIT_NOTIFIER_HEAD(&pnw->iotg.otg.notifier);

	pnw->iotg_notifier.notifier_call = penwell_otg_iotg_notify;
	if (intel_mid_otg_register_notifier(&pnw->iotg, &pnw->iotg_notifier)) {
		dev_dbg(pnw->dev, "Failed to register notifier\n");
		retval = -EBUSY;
		goto err;
	}
	if (register_pm_notifier(&pnw_sleep_pm_notifier)) {
		dev_dbg(pnw->dev, "Fail to register PM notifier\n");
		retval = -EBUSY;
		goto err;
	}

#ifdef CONFIG_BOARD_CTP
	/* Set up gpio for Clovertrail */
	pnw->otg_pdata = (struct cloverview_usb_otg_pdata *)
			cloverview_usb_otg_get_pdata();
	if (pnw->otg_pdata == NULL) {
		dev_err(pnw->dev, "Failed to get OTG platform data.\n");
		retval = -ENODEV;
		goto err;
	}
	retval = gpio_request(pnw->otg_pdata->gpio_reset,
				"usb_otg_phy_reset");
	if (retval < 0) {
		dev_err(pnw->dev, "request gpio(%d) for usb otg phy reset "
			"failed\n", pnw->otg_pdata->gpio_reset);
		retval = -ENODEV;
		goto err;
	}
	retval = gpio_request(pnw->otg_pdata->gpio_cs,
				"usb_otg_phy_cs");
	if (retval < 0) {
		dev_err(pnw->dev, "request gpio(%d) for usb otg phy cs "
			"failed\n", pnw->otg_pdata->gpio_cs);
		gpio_free(pnw->otg_pdata->gpio_reset);
		retval = -ENODEV;
		goto err;
	}
	/* Drive CS pin to high */
	gpio_direction_output(pnw->otg_pdata->gpio_cs, 1);

	/* Reset the PHY (minimal reset width: 100us) */
	gpio_direction_output(pnw->otg_pdata->gpio_reset, 0);
	usleep_range(200, 500);
	gpio_set_value(pnw->otg_pdata->gpio_reset, 1);
#endif

	mutex_init(&pnw->msic_mutex);
	pnw->msic = penwell_otg_check_msic();

	penwell_otg_phy_low_power(0);

	if (!is_clovertrail(pdev)) {
		/* Workaround for ULPI lockup issue, need turn off PHY 4ms */
		penwell_otg_phy_enable(0);
		usleep_range(4000, 4500);
		penwell_otg_phy_enable(1);
		/* reset phy */
		dev_dbg(pnw->dev, "Reset Phy over SPI\n");
		penwell_otg_msic_spi_access(true);
		penwell_otg_msic_write(MSIC_FUNCTRLSET, PHYRESET);
		penwell_otg_msic_spi_access(false);
		dev_dbg(pnw->dev, "Reset Phy over SPI Done\n");
	}

	/* Enable ID pullup immediately after reeable PHY */
	val32 = readl(pnw->iotg.base + CI_OTGSC);
	writel(val32 | OTGSC_IDPU, pnw->iotg.base + CI_OTGSC);

	/* Wait correct value to be synced */
	set_host_mode();
	usleep_range(2000, 3000);
	penwell_otg_phy_low_power(1);
	msleep(100);

	/* enable ACA device detection for CTP */
	if (is_clovertrail(pdev))
		penwell_otg_aca_enable();

	reset_otg();
	init_hsm();

	/* we need to set active early or the first irqs will be ignored */
	pm_runtime_set_active(&pdev->dev);

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

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	penwell_update_transceiver();

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
	struct otg_bc_event *evt, *tmp;

	/* ACA device detection disable */
	penwell_otg_aca_disable();

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	if (pnw->qwork) {
		flush_workqueue(pnw->qwork);
		destroy_workqueue(pnw->qwork);
	}

	if (pnw->chrg_qwork) {
		flush_workqueue(pnw->chrg_qwork);
		destroy_workqueue(pnw->chrg_qwork);
		list_for_each_entry_safe(evt, tmp, &pnw->chrg_evt_queue, node) {
			list_del(&evt->node);
			kfree(evt);
		}
	}

	/* disable OTGSC interrupt as OTGSC doesn't change in reset */
	writel(0, pnw->iotg.base + CI_OTGSC);

	wake_lock_destroy(&pnw->wake_lock);

	if (pdev->irq)
		free_irq(pdev->irq, pnw);
	if (pnw->cfg_region)
		release_mem_region(USBCFG_ADDR, USBCFG_LEN);
	if (pnw->iotg.base)
		iounmap(pnw->iotg.base);
	if (pnw->region)
		release_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));
	if (is_clovertrail(pdev) && pnw->otg_pdata != NULL)
		kfree(pnw->otg_pdata);

	otg_set_transceiver(NULL);
	pci_disable_device(pdev);
	sysfs_remove_group(&pdev->dev.kobj, &debug_dev_attr_group);
	device_remove_file(&pdev->dev, &dev_attr_chargers);
	device_remove_file(&pdev->dev, &dev_attr_hsm);
	device_remove_file(&pdev->dev, &dev_attr_registers);
	kfree(pnw);
	pnw = NULL;
}

void penwell_otg_shutdown(struct pci_dev *pdev)
{
	struct penwell_otg *pnw = the_transceiver;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (!is_clovertrail(pdev)) {
		/* Disable MSIC Interrupt Notifications */
		penwell_otg_msic_spi_access(true);

		penwell_otg_msic_write(MSIC_INT_EN_RISE_CLR, 0x1F);
		penwell_otg_msic_write(MSIC_INT_EN_FALL_CLR, 0x1F);

		penwell_otg_msic_spi_access(false);
	}

	dev_dbg(pnw->dev, "%s <---\n", __func__);
}


static int penwell_otg_suspend_noirq(struct device *dev)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				ret = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	switch (iotg->otg.state) {
	case OTG_STATE_A_VBUS_ERR:
		set_host_mode();
		iotg->otg.state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_A_WAIT_VFALL:
		penwell_otg_del_timer(TA_WAIT_VFALL_TMR);
		iotg->otg.state = OTG_STATE_A_IDLE;
	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
		break;
	case OTG_STATE_A_WAIT_VRISE:
		penwell_otg_del_timer(TA_WAIT_VRISE_TMR);
		iotg->hsm.a_srp_det = 0;

		/* Turn off VBus */
		otg_set_vbus(&iotg->otg, false);
		iotg->otg.state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_HOST:
		if (pnw->iotg.suspend_noirq_host)
			ret = pnw->iotg.suspend_noirq_host(&pnw->iotg);
		goto done;
		break;
	case OTG_STATE_A_SUSPEND:
		penwell_otg_del_timer(TA_AIDL_BDIS_TMR);
		penwell_otg_HABA(0);
		PNW_STOP_HOST(pnw);
		iotg->hsm.a_srp_det = 0;

		penwell_otg_phy_vbus_wakeup(false);

		/* Turn off VBus */
		otg_set_vbus(&iotg->otg, false);
		iotg->otg.state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_A_PERIPHERAL:
		penwell_otg_del_timer(TA_BIDL_ADIS_TMR);

		if (pnw->iotg.stop_peripheral)
			pnw->iotg.stop_peripheral(&pnw->iotg);
		else
			dev_dbg(pnw->dev, "client driver has been stopped.\n");

		/* Turn off VBus */
		otg_set_vbus(&iotg->otg, false);
		iotg->hsm.a_srp_det = 0;
		iotg->otg.state = OTG_STATE_A_IDLE;
		break;
	case OTG_STATE_B_HOST:
		/* Stop HNP polling */
		iotg->stop_hnp_poll(iotg);

		PNW_STOP_HOST(pnw);
		iotg->hsm.b_bus_req = 0;
		iotg->otg.state = OTG_STATE_B_IDLE;
		break;
	case OTG_STATE_B_PERIPHERAL:
		dev_dbg(pnw->dev, "don't suspend, client still alive\n");
		ret = -EBUSY;
		break;
	case OTG_STATE_B_WAIT_ACON:
		penwell_otg_del_timer(TB_ASE0_BRST_TMR);

		penwell_otg_HAAR(0);

		PNW_STOP_HOST(pnw);
		iotg->hsm.b_bus_req = 0;
		iotg->otg.state = OTG_STATE_B_IDLE;
		break;
	default:
		dev_dbg(pnw->dev, "error state before suspend\n");
		break;
	}


	if (ret) {
		/* allow queue work from notifier */
		spin_lock(&pnw->notify_lock);
		pnw->queue_stop = 0;
		spin_unlock(&pnw->notify_lock);

		penwell_update_transceiver();
	} else {
		penwell_otg_phy_low_power(1);
		penwell_otg_vusb330_low_power(1);
	}

done:
	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return ret;
}

static int penwell_otg_suspend(struct device *dev)
{
	struct penwell_otg		*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int				ret = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (iotg->otg.state == OTG_STATE_B_PERIPHERAL) {
		dev_dbg(pnw->dev, "still alive, don't suspend\n");
		ret = -EBUSY;
		goto done;
	}

	/* Stop queue work from notifier */
	spin_lock(&pnw->notify_lock);
	pnw->queue_stop = 1;
	spin_unlock(&pnw->notify_lock);
	flush_workqueue(pnw->qwork);
	if (delayed_work_pending(&pnw->ulpi_check_work)) {
		spin_lock(&pnw->notify_lock);
		pnw->queue_stop = 0;
		spin_unlock(&pnw->notify_lock);
		ret = -EBUSY;
		goto done;
	} else
		flush_delayed_work_sync(&pnw->ulpi_check_work);

	switch (iotg->otg.state) {
	case OTG_STATE_A_WAIT_BCON:
		penwell_otg_del_timer(TA_WAIT_BCON_TMR);
		iotg->hsm.a_srp_det = 0;
		if (iotg->suspend_host)
			ret = iotg->suspend_host(iotg);
		break;
	case OTG_STATE_A_HOST:
		iotg->stop_hnp_poll(iotg);
		if (iotg->suspend_host)
			ret = iotg->suspend_host(iotg);
		break;
	default:
		break;
	}

done:
	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return ret;
}


static int penwell_otg_resume_noirq(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int			ret = 0;
	u32			val;

	dev_dbg(pnw->dev, "%s --->\n", __func__);

	if (mid_pmu_is_wake_source(PMU_OTG_WAKE_SOURCE)) {
		/* dump OTGSC register for wakeup event */
		val = readl(pnw->iotg.base + CI_OTGSC);
		dev_info(pnw->dev, "%s: CI_OTGSC=0x%x\n", __func__, val);
		if (val & OTGSC_IDIS)
			dev_info(pnw->dev, "%s: id change\n", __func__);
		if (val & OTGSC_DPIS)
			dev_info(pnw->dev, "%s: data pulse\n", __func__);
		if (val & OTGSC_BSEIS)
			dev_info(pnw->dev, "%s: b sess end\n", __func__);
		if (val & OTGSC_BSVIS)
			dev_info(pnw->dev, "%s: b sess valid\n", __func__);
		if (val & OTGSC_ASVIS)
			dev_info(pnw->dev, "%s: a sess valid\n", __func__);
		if (val & OTGSC_AVVIS)
			dev_info(pnw->dev, "%s: a vbus valid\n", __func__);

		if (!(val & OTGSC_INTSTS_MASK)) {

			static bool uevent_reported;
			dev_info(pnw->dev,
				 "%s: waking up from USB source, but not a OTG wakeup event\n",
				 __func__);
			if (!uevent_reported) {
				queue_work(pnw->qwork, &pnw->uevent_work);
				uevent_reported = true;
			}
		}
	}

	if (iotg->otg.state != OTG_STATE_A_WAIT_BCON &&
		iotg->otg.state != OTG_STATE_A_HOST) {
		penwell_otg_vusb330_low_power(0);
		penwell_otg_phy_low_power(0);
	}

	/* D3->D0 controller will be reset, so reset work mode and PHY state
	 * which is cleared by the reset */

	switch (iotg->otg.state) {
	case OTG_STATE_B_IDLE:
		break;
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_HOST:
		if (iotg->resume_noirq_host)
			ret = iotg->resume_noirq_host(iotg);
		break;
	default:
		break;
	}


	/* We didn't disable otgsc interrupt, to prevent intr from happening
	* before penwell_otg_resume, intr is disabled here, and can be enabled
	* by penwell_otg_resume
	*/
	penwell_otg_intr(0);

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return ret;
}

static int penwell_otg_resume(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct intel_mid_otg_xceiv	*iotg = &pnw->iotg;
	int			ret = 0;

	dev_dbg(pnw->dev, "%s --->\n", __func__);
	switch (iotg->otg.state) {
	case OTG_STATE_A_WAIT_BCON:
		if (iotg->resume_host)
			ret = iotg->resume_host(iotg);
		penwell_otg_add_timer(TA_WAIT_BCON_TMR);
		break;
	case OTG_STATE_A_HOST:
		if (iotg->resume_host)
			ret = iotg->resume_host(iotg);

		/* FIXME: Ideally here should re-start HNP polling,
		 * no start HNP here, because it blocks the resume
		*/
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	/* allow queue work from notifier */
	spin_lock(&pnw->notify_lock);
	pnw->queue_stop = 0;
	spin_unlock(&pnw->notify_lock);

	penwell_otg_intr(1);

	/* If a plugging in or pluggout event happens during D3,
	* we will miss the interrupt, so check OTGSC here to check
	* if any ID change and update hsm correspondingly
	*/
	update_hsm();
	penwell_update_transceiver();

	dev_dbg(pnw->dev, "%s <---\n", __func__);
	return ret;
}


#ifdef CONFIG_PM_RUNTIME
/* Runtime PM */
static int penwell_otg_runtime_suspend(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	struct pci_dev		*pdev = to_pci_dev(dev);
	int			ret = 0;
	u32			val;

	dev_dbg(dev, "%s --->\n", __func__);

	/* Flush any pending otg irq on local or any other CPUs.
	*
	* Host mode or Device mode irq should be synchronized by itself in
	* their runtime_suspend handler. In fact, Host mode does so. For
	* device mode, we don't care as its runtime PM is disabled.
	*
	* As device's runtime_status is already RPM_SUSPENDING, after flushing,
	* any new irq handling will be rejected (otg irq handler only continues
	* if runtime_status is RPM_ACTIVE).
	* Thus, now it's safe to put PHY into low power mode and gate the
	* fabric later in pci_set_power_state().
	*/
	synchronize_irq(pdev->irq);

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_A_IDLE:
		break;
	case OTG_STATE_B_IDLE:
		val = readl(pnw->iotg.base + CI_USBMODE);
		if (!(val & USBMODE_CM)) {
			/* Controller needs to reset & set mode */
			dev_dbg(dev, "reset to client mode\n");
			set_client_mode();
		}
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

	if (ret)
		goto DONE;

	penwell_otg_phy_low_power(1);

	msleep(2);

	penwell_otg_vusb330_low_power(1);

DONE:
	dev_dbg(dev, "%s <---: ret = %d\n", __func__, ret);
	return ret;
}

static int penwell_otg_runtime_resume(struct device *dev)
{
	struct penwell_otg	*pnw = the_transceiver;
	int			ret = 0;
	u32			val;

	dev_dbg(dev, "%s --->\n", __func__);

	if (pnw->rt_resuming) {
		dev_dbg(pnw->dev, "%s  rt_resuming--->\n", __func__);
		penwell_otg_check_wakeup_event(pnw);
	}

	penwell_otg_phy_low_power(0);
	penwell_otg_vusb330_low_power(0);
	penwell_otg_intr(1);

	switch (pnw->iotg.otg.state) {
	case OTG_STATE_A_IDLE:
		break;
	case OTG_STATE_B_IDLE:
		val = readl(pnw->iotg.base + CI_USBMODE);
		if (!(val & USBMODE_CM)) {
			/* Controller needs to reset & set mode */
			dev_dbg(dev, "reset to client mode\n");
			set_client_mode();
		}
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

	if (pnw->rt_resuming) {
		dev_dbg(pnw->dev, "irq num: %d\n", pnw->rt_resuming);
		pnw->rt_resuming = 0;

		pm_runtime_put(pnw->dev);
	}

	dev_dbg(dev, "%s <---\n", __func__);

	return ret;
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
		dev_dbg(dev, "%s <---\n", __func__);
		return -EBUSY;
	default:
		break;
	}

	/* some delay for stability */
	pm_schedule_suspend(dev, 500);

	dev_dbg(dev, "%s <---\n", __func__);

	return -EBUSY;
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
	},
	{ /* Cloverview */
		.class =        ((PCI_CLASS_SERIAL_USB << 8) | 0x20),
		.class_mask =   ~0,
		.vendor =	0x8086,
		.device =	0xE006,
		.subvendor =	PCI_ANY_ID,
		.subdevice =	PCI_ANY_ID,
	},
	{ /* end: all zeroes */ }
};

static const struct dev_pm_ops penwell_otg_pm_ops = {
	.runtime_suspend = penwell_otg_runtime_suspend,
	.runtime_resume = penwell_otg_runtime_resume,
	.runtime_idle = penwell_otg_runtime_idle,
	.suspend = penwell_otg_suspend,
	.suspend_noirq = penwell_otg_suspend_noirq,
	.resume = penwell_otg_resume,
	.resume_noirq = penwell_otg_resume_noirq,
};

static struct pci_driver otg_pci_driver = {
	.name =		(char *) driver_name,
	.id_table =	pci_ids,

	.probe =	penwell_otg_probe,
	.remove =	penwell_otg_remove,
	.shutdown =	penwell_otg_shutdown,
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
