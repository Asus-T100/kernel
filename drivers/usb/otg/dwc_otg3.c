#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/dwc_otg3.h>


static struct mutex lock;
static const char driver_name[] = "dwc_otg3";
static void __devexit dwc_otg_remove(struct pci_dev *pdev);
static struct dwc_device_par *platform_par;
static struct dwc_otg2 *the_transceiver;
static irqreturn_t dwc_otg_irq(int irq, void *_dev);

static void print_debug_regs(struct dwc_otg2 *otg)
{
	u32 usbcmd = otg_read(otg, 0x0000);
	u32 usbsts = otg_read(otg, 0x0004);
	u32 portsc = otg_read(otg, 0x400);
	u32 gctl = otg_read(otg, 0xc110);
	u32 gsts = otg_read(otg, 0xc118);
	u32 gdbgltssm = otg_read(otg, 0xc164);
	u32 gusb2phycfg0 = otg_read(otg, 0xc200);
	u32 gusb3pipectl0 = otg_read(otg, 0xc2c0);
	u32 dcfg = otg_read(otg, 0xc700);
	u32 dctl = otg_read(otg, 0xc704);
	u32 dsts = otg_read(otg, 0xc70c);
	u32 ocfg = otg_read(otg, OCFG);
	u32 octl = otg_read(otg, OCTL);
	u32 oevt = otg_read(otg, OEVT);
	u32 oevten = otg_read(otg, OEVTEN);
	u32 osts = otg_read(otg, OSTS);

	otg_dbg(otg, "usbcmd = %08x\n", usbcmd);
	otg_dbg(otg, "usbsts = %08x\n", usbsts);
	otg_dbg(otg, "portsc = %08x\n", portsc);
	otg_dbg(otg, "gctl = %08x\n", gctl);
	otg_dbg(otg, "gsts = %08x\n", gsts);
	otg_dbg(otg, "gdbgltssm = %08x\n", gdbgltssm);
	otg_dbg(otg, "gusb2phycfg0 = %08x\n", gusb2phycfg0);
	otg_dbg(otg, "gusb3pipectl0 = %08x\n", gusb3pipectl0);
	otg_dbg(otg, "dcfg = %08x\n", dcfg);
	otg_dbg(otg, "dctl = %08x\n", dctl);
	otg_dbg(otg, "dsts = %08x\n", dsts);
	otg_dbg(otg, "ocfg = %08x\n", ocfg);
	otg_dbg(otg, "octl = %08x\n", octl);
	otg_dbg(otg, "oevt = %08x\n", oevt);
	otg_dbg(otg, "oevten = %08x\n", oevten);
	otg_dbg(otg, "osts = %08x\n", osts);
}

/* Caller must hold otg->lock */
static void wakeup_main_thread(struct dwc_otg2 *otg)
{
	otg_dbg(otg, "\n");
	/* Tell the command thread that something has happened */
	otg->main_wakeup_needed = 1;
	if (otg->main_thread)
		wake_up_interruptible(&otg->main_wq);
}

static int sleep_main_thread_timeout(struct dwc_otg2 *otg, int msecs)
{
	signed long jiffies;
	int rc = msecs;

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exited\n");
		rc = -EINTR;
		goto done;
	}

	if (signal_pending(current)) {
		otg_dbg(otg, "Main thread signal pending\n");
		rc = -EINTR;
		goto done;
	}
	if (otg->main_wakeup_needed) {
		otg_dbg(otg, "Main thread wakeup needed\n");
		rc = msecs;
		goto done;
	}

	jiffies = msecs_to_jiffies(msecs);
	rc = wait_event_interruptible_timeout(otg->main_wq,
					otg->main_wakeup_needed,
					jiffies);

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exited\n");
		rc = -EINTR;
		goto done;
	}

	if (rc > 0)
		rc = jiffies_to_msecs(rc);

done:
	otg->main_wakeup_needed = 0;
	return rc;
}

static int sleep_main_thread(struct dwc_otg2 *otg)
{
	int rc = 0;

	do {
		rc = sleep_main_thread_timeout(otg, 5000);
	} while (rc == 0);

	return rc;
}

static void get_events(struct dwc_otg2 *otg,
		u32 *otg_events,
		u32 *adp_events,
		u32 *user_events)
{
	unsigned long flags;

	spin_lock_irqsave(&otg->lock, flags);

	if (otg_events)
		*otg_events = otg->otg_events;

	if (adp_events)
		*adp_events = otg->adp_events;

	if (user_events)
		*user_events = otg->user_events;
	spin_unlock_irqrestore(&otg->lock, flags);
}

static void get_and_clear_events(struct dwc_otg2 *otg,
				u32 *otg_events,
				u32 *adp_events,
				u32 *user_events)
{
	unsigned long flags;

	spin_lock_irqsave(&otg->lock, flags);

	if (otg_events)
		*otg_events = otg->otg_events;

	if (adp_events)
		*adp_events = otg->adp_events;

	if (user_events)
		*user_events = otg->user_events;

	otg->otg_events = 0;
	otg->adp_events = 0;
	otg->user_events = 0;

	spin_unlock_irqrestore(&otg->lock, flags);
}

static int check_event(struct dwc_otg2 *otg,
		u32 otg_mask,
		u32 adp_mask,
		u32 user_mask)
{
	u32 otg_events = 0;
	u32 adp_events = 0;
	u32 user_events = 0;

	get_events(otg, &otg_events, &adp_events, &user_events);
	if ((otg_events & otg_mask) ||
			(adp_events & adp_mask) ||
			(user_events & user_mask)) {
		otg_dbg(otg, "Event occurred: "
			"otg_events=%x, otg_mask=%x, "
			"adp_events=%x, adp_mask=%x"
			"user_events=%x, user_mask=%x",
			otg_events, otg_mask,
			adp_events, adp_mask,
			user_events, user_mask);
		return 1;
	}

	return 0;
}

static int sleep_until_event(struct dwc_otg2 *otg,
			u32 otg_mask, u32 adp_mask, u32 user_mask,
			u32 *otg_events, u32 *adp_events, u32 *user_events,
			int timeout)
{
	int rc = 0;
	u32 oevten = otg_mask;

	/* Enable the events */
	if (oevten)
		otg_write(otg, OEVTEN, oevten);

	/* Wait until it occurs, or timeout, or interrupt. */
	if (timeout) {
		otg_dbg(otg, "Waiting for event (timeout=%d)...\n", timeout);
		rc = sleep_main_thread_until_condition_timeout(otg,
				check_event(otg, otg_mask,
				adp_mask, user_mask), timeout);
	} else {
		otg_dbg(otg, "Waiting for event (no timeout)...\n");
		rc = sleep_main_thread_until_condition(otg,
				check_event(otg, otg_mask,
					adp_mask, user_mask));
	}

	/* Disable the events */
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, ADPEVTEN, 0);

	otg_dbg(otg, "Woke up rc=%d\n", rc);
	if (rc < 0)
		goto done;
	else
		get_and_clear_events(otg, otg_events, adp_events, user_events);

done:
	return rc;
}

static int handshake(struct dwc_otg2 *otg,
		u32 reg, u32 mask, u32 done, u32 msec)
{
	u32 result;

	otg_dbg(otg, "reg=%08x, mask=%08x, value=%08x\n", reg, mask, done);
	do {
		result = otg_read(otg, reg);
		if ((result & mask) == done)
			return 1;

		mdelay(1);
		msec -= 1;
	} while (msec > 0);

	return 0;
}

static int reset_port(struct dwc_otg2 *otg)
{
	struct usb_hcd *hcd = NULL;

	if (otg->otg.host) {
		hcd = container_of(otg->otg.host, struct usb_hcd, self);
		return hcd->driver->reset_port(hcd);
	}
	return -ENODEV;
}

static int set_peri_mode(struct dwc_otg2 *otg, int mode)
{
	u32 octl = 0;

	/* Set peri_mode */
	octl = otg_read(otg, OCTL);
	if (mode)
		octl |= OCTL_PERI_MODE;
	else
		octl &= (~OCTL_PERI_MODE);

	otg_write(otg, OCTL, octl);

/* TODO Workaround for OTG 3.0 core. Reverse when fixed. */
#if 0
	if (mode)
		return handshake(otg, OSTS,
		OSTS_PERIP_MODE, OSTS_PERIP_MODE, 100);
	else
		return handshake(otg, OSTS, OSTS_PERIP_MODE, 0, 100);
#else
	msleep(20);
	return 1;
#endif
}

static void set_sus_phy(struct dwc_otg2 *otg, int bit)
{
	u32 data = 0;

	data = otg_read(otg, GUSB2PHYCFG0);
	if (bit)
		data |= GUSB2PHYCFG_SUS_PHY;
	else
		data &= ~GUSB2PHYCFG_SUS_PHY;

	otg_write(otg, GUSB2PHYCFG0, data);

#if 1
	data = otg_read(otg, GUSB3PIPECTL0);
	if (bit)
		data |= GUSB3PIPECTL_SUS_EN;
	else
		data &= ~GUSB3PIPECTL_SUS_EN;
	otg_write(otg, GUSB3PIPECTL0, data);
#endif
}

static int start_host(struct dwc_otg2 *otg)
{
	int ret = 0;
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");

	if (!otg->otg.host) {
		otg_err(otg, "Haven't set host yet!\n");
		return -ENODEV;
	}

	if (!set_peri_mode(otg, PERI_MODE_HOST))
		otg_err(otg, "Failed to start host.");

	/* Start host driver */
	hcd = container_of(otg->otg.host, struct usb_hcd, self);
	ret = hcd->driver->start_host(hcd);

	return ret;
}

static int stop_host(struct dwc_otg2 *otg)
{
	int ret = -1;
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");
	return 0;

	if (otg->otg.host) {
		hcd = container_of(otg->otg.host, struct usb_hcd, self);
		ret = hcd->driver->stop_host(hcd);
	}

	return ret;
}

/* Sends the host release set feature request */
static void host_release(struct dwc_otg2 *otg)
{
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");

	if (otg->otg.host) {
		hcd = container_of(otg->otg.host, struct usb_hcd, self);
		hcd->driver->release_host(hcd);
	}
}

static void start_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget;

	print_debug_regs(otg);
	otg_dbg(otg, "\n");

	gadget = otg->otg.gadget;
	if (!gadget) {
		otg_err(otg, "Haven't set gadget yet!\n");
		return;
	}

	if (!set_peri_mode(otg, PERI_MODE_PERIPHERAL))
		otg_err(otg, "Failed to start peripheral.");

	gadget->ops->start_device(gadget);
	print_debug_regs(otg);
}

static void stop_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget = otg->otg.gadget;

	if (!gadget)
		return;

	print_debug_regs(otg);
	otg_dbg(otg, "\n");
	gadget->ops->stop_device(gadget);
	print_debug_regs(otg);
}

static enum usb_charger_type get_charger_type(struct dwc_otg2 *otg)
{
	return CHRG_SDP;
}

static enum dwc_otg_state do_charger_detection(struct dwc_otg2 *otg)
{
	enum usb_charger_type charger = CHRG_UNKNOWN;
	enum dwc_otg_state state = DWC_STATE_INVALID;

	charger = get_charger_type(otg);
	switch (charger) {
	case CHRG_SDP:
		otg_dbg(otg, "Detect SDP\n");
		state = DWC_STATE_B_PERIPHERAL;
		break;
	case CHRG_CDP:
		otg_dbg(otg, "Detect CDP\n");
		state = DWC_STATE_B_PERIPHERAL;
		break;
	case CHRG_DCP:
		otg_dbg(otg, "Detect DCP\n");
		break;
	case CHRG_ACA:
		otg_dbg(otg, "Detect ACA\n");
		break;
	case CHRG_UNKNOWN:
		otg_dbg(otg, "Detect UNKNOW charger. Doubt that it is self-powered USB device\n");
		break;
	};

	otg_dbg(otg, "Currently, haven't support charger detection.\n");
	return DWC_STATE_INIT;

	return state;
}

static enum dwc_otg_state do_connector_id_status(struct dwc_otg2 *otg)
{
	u32 events = 0;
	enum dwc_otg_state state = DWC_STATE_INVALID;
	u32 osts = 0;

	otg_dbg(otg, "\n");


	/* Reset ADP related registers */
	otg_write(otg, ADPCFG, 0);
	otg_write(otg, ADPCTL, 0);
	otg_write(otg, ADPEVTEN, 0);

	otg_write(otg, OCFG, 0);
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, OEVT, 0xffffffff);
	otg_write(otg, OEVTEN, OEVT_CONN_ID_STS_CHNG_EVNT);
	otg_write(otg, OCTL, OCTL_PERI_MODE);

	msleep(60);

	osts = otg_read(otg, OSTS);
	if (!(osts & OSTS_CONN_ID_STS)) {
		otg_dbg(otg, "Connector ID is A\n");
		state = DWC_STATE_A_HOST;
	} else {
		otg_dbg(otg, "Connector ID is B\n");
		if (osts & OSTS_B_SES_VLD)
			state = DWC_STATE_B_PERIPHERAL;
		else
			otg_dbg(otg, "B session invalied, so haven't connect HOST\n");
	}

	sleep_until_event(otg, OEVT_CONN_ID_STS_CHNG_EVNT \
			| OEVT_B_DEV_VBUS_CHNG_EVNT \
			| OEVT_B_DEV_SES_VLD_DET_EVNT, \
			0, 0, &events, NULL, NULL, 0);

	if (events & (OEVT_B_DEV_VBUS_CHNG_EVNT | \
				OEVT_B_DEV_SES_VLD_DET_EVNT)) {
		otg_dbg(otg, "events is vbus valid\n");
		state = DWC_STATE_B_PERIPHERAL;
	}

	if (events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "events is id change\n");
		state = DWC_STATE_A_HOST;
	}


	return state;
}

static void reset_hw(struct dwc_otg2 *otg)
{
	otg_dbg(otg, "\n");
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, OCTL, 0);
}

static enum dwc_otg_state do_a_host(struct dwc_otg2 *otg)
{
	int rc = 0;
	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;
	int ret = 0;

	set_sus_phy(otg, 0);
	otg_write(otg, GCTL, 0x45801000);

	ret = start_host(otg);

	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT;

	rc = sleep_until_event(otg,
			otg_mask, 0, user_mask,
			&otg_events, NULL, &user_events, 0);
	if (rc < 0)
		return DWC_STATE_EXIT;

	if (!ret)
		stop_host(otg);

	/* Higher priority first */
	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		return DWC_STATE_CHARGER_DETECTION;
	}

	/* Invalid state */
	return DWC_STATE_INVALID;
}

static int do_b_peripheral(struct dwc_otg2 *otg)
{
	int rc = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;
	u32 otg_events = 0;
	u32 user_events = 0;

	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT \
			| OEVT_B_DEV_VBUS_CHNG_EVNT \
			| OEVT_B_DEV_SES_VLD_DET_EVNT;
	user_mask = 0;

	rc = sleep_until_event(otg,
			otg_mask, 0, user_mask,
			&otg_events, NULL, &user_events, 0);
	if (rc < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		return DWC_STATE_CHARGER_DETECTION;
	}

	if (otg_events & OEVT_B_DEV_VBUS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_B_DEV_VBUS_CHNG_EVNT!\n");
		state = DWC_STATE_CHARGER_DETECTION;
	}

	if (otg_events & OEVT_B_DEV_SES_VLD_DET_EVNT) {
		otg_dbg(otg, "OEVT_B_DEV_SES_VLD_DET_EVNT!\n");
		state = DWC_STATE_CHARGER_DETECTION;
	}

	return DWC_STATE_INVALID;
}

int otg_main_thread(void *data)
{
	struct dwc_otg2 *otg = (struct dwc_otg2 *)data;
#ifdef DEBUG
	u32 snpsid = otg_read(otg, 0xc120);

	otg_dbg(otg, "io_priv=%p\n", otg->otg.io_priv);
	otg_dbg(otg, "c120: %x\n", snpsid);
#endif
	msleep(100);

	/* Allow the thread to be killed by a signal, but set the signal mask
	 * to block everything but INT, TERM, KILL, and USR1. */
	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	/* Allow the thread to be frozen */
	set_freezable();
	reset_hw(otg);

	if (request_irq(otg->irqnum, dwc_otg_irq, IRQF_SHARED,
				driver_name, otg) != 0) {
		otg_dbg(otg,
			"request interrupt %d failed\n", otg->irqnum);
		return -EINVAL;
	}

	otg_dbg(otg, "Thread running\n");
	while (otg->state != DWC_STATE_TERMINATED) {
		int next = DWC_STATE_INIT;
		otg_dbg(otg, "\n\n\nMain thread entering state\n");

		switch (otg->state) {
		case DWC_STATE_INIT:
			otg_dbg(otg, "DWC_STATE_INIT\n");
			next = do_connector_id_status(otg);
			break;
		case DWC_STATE_CHARGER_DETECTION:
			otg_dbg(otg, "DWC_STATE_CHARGER_DETECTION\n");
			next = do_charger_detection(otg);
			break;
		case DWC_STATE_A_HOST:
			otg_dbg(otg, "DWC_STATE_A_HOST\n");
			next = do_a_host(otg);
			stop_host(otg);
			break;
		case DWC_STATE_B_PERIPHERAL:
			otg_dbg(otg, "DWC_STATE_B_PERIPHERAL\n");
			start_peripheral(otg);
			next = do_b_peripheral(otg);

			stop_peripheral(otg);
			break;
		case DWC_STATE_EXIT:
			otg_dbg(otg, "DWC_STATE_EXIT\n");
			reset_hw(otg);
			next = DWC_STATE_TERMINATED;
			break;
		case DWC_STATE_INVALID:
			otg_dbg(otg, "DWC_STATE_INVALID!!!\n");
		default:
			otg_dbg(otg, "Unknown State %d, sleeping...\n",
					otg->state);
			sleep_main_thread(otg);
			break;
		}

		otg->prev = otg->state;
		otg->state = next;
	}

	otg_dbg(otg, "OTG main thread exiting....\n");

	return 0;
}

static void start_main_thread(struct dwc_otg2 *otg)
{
	mutex_lock(&lock);
	if (!otg->main_thread && otg->otg.gadget && otg->otg.host) {
		otg_dbg(otg, "Starting OTG main thread\n");
		otg->main_thread = kthread_create(otg_main_thread, otg, "otg");
		wake_up_process(otg->main_thread);
	}
	mutex_unlock(&lock);
}

static void stop_main_thread(struct dwc_otg2 *otg)
{
	mutex_lock(&lock);
	if (otg->main_thread) {
		otg_dbg(otg, "Stopping OTG main thread\n");
		otg->state = DWC_STATE_EXIT;
		wake_up_process(otg->main_thread);
	}
	mutex_unlock(&lock);
}

static inline struct dwc_otg2 *xceiv_to_dwc_otg2(struct otg_transceiver *x)
{
	return container_of(x, struct dwc_otg2, otg);
}

static int dwc_otg2_set_suspend(struct otg_transceiver *x, int suspend)
{
	return 0;
}

static irqreturn_t dwc_otg_irq(int irq, void *_dev)
{
	struct dwc_otg2 *otg = _dev;
	unsigned long flags;
	u32 oevten = 0;
	u32 oevt = 0;
	u32 osts = 0;
	u32 octl = 0;
	u32 ocfg = 0;
	u32 adpcfg = 0;
	u32 adpctl = 0;
	u32 adpevt = 0;
	u32 adpevten = 0;
	u32 otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT;
	u32 adp_mask = 0;

	oevt = otg_read(otg, OEVT);
	osts = otg_read(otg, OSTS);
	octl = otg_read(otg, OCTL);
	ocfg = otg_read(otg, OCFG);
	oevten = otg_read(otg, OEVTEN);

	adpcfg = otg_read(otg, ADPCFG);
	adpctl = otg_read(otg, ADPCTL);
	adpevt = otg_read(otg, ADPEVT);
	adpevten = otg_read(otg, ADPEVTEN);

	if (!(oevt & oevten) && (oevt != 0x80000000)) {
		otg_dbg(otg, "Not otg interrupt.\n");
		return IRQ_NONE;
	}


	/* Clear handled events */
	otg_write(otg, OEVT, oevt);
	otg_write(otg, ADPEVT, adpevt);

	otg_dbg(otg, "\n");
	otg_dbg(otg, "    oevt = %08x\n", oevt);
	otg_dbg(otg, "    osts = %08x\n", osts);
	otg_dbg(otg, "    octl = %08x\n", octl);
	otg_dbg(otg, "    ocfg = %08x\n", ocfg);
	otg_dbg(otg, "  oevten = %08x\n", oevten);
	otg_dbg(otg, "  adpcfg = %08x\n", adpcfg);
	otg_dbg(otg, "  adpctl = %08x\n", adpctl);
	otg_dbg(otg, "  adpevt = %08x\n", adpevt);
	otg_dbg(otg, "adpevten = %08x\n", adpevten);

	otg_dbg(otg, "oevt[DeviceMode] = %s\n",
			oevt & OEVT_DEV_MOD_EVNT ? "Device" : "Host");

	if (oevt & OEVT_CONN_ID_STS_CHNG_EVNT)
		otg_dbg(otg, "Connector ID Status Change Event\n");
	if (oevt & OEVT_HOST_ROLE_REQ_INIT_EVNT)
		otg_dbg(otg, "Host Role Request Init Notification Event\n");
	if (oevt & OEVT_HOST_ROLE_REQ_CONFIRM_EVNT)
		otg_dbg(otg, "Host Role Request Confirm Notification Event\n");
	if (oevt & OEVT_A_DEV_B_DEV_HOST_END_EVNT)
		otg_dbg(otg, "A-Device B-Host End Event\n");
	if (oevt & OEVT_A_DEV_HOST_EVNT)
		otg_dbg(otg, "A-Device Host Event\n");
	if (oevt & OEVT_A_DEV_HNP_CHNG_EVNT)
		otg_dbg(otg, "A-Device HNP Change Event\n");
	if (oevt & OEVT_A_DEV_SRP_DET_EVNT)
		otg_dbg(otg, "A-Device SRP Detect Event\n");
	if (oevt & OEVT_A_DEV_SESS_END_DET_EVNT)
		otg_dbg(otg, "A-Device Session End Detected Event\n");
	if (oevt & OEVT_B_DEV_B_HOST_END_EVNT)
		otg_dbg(otg, "B-Device B-Host End Event\n");
	if (oevt & OEVT_B_DEV_HNP_CHNG_EVNT)
		otg_dbg(otg, "B-Device HNP Change Event\n");
	if (oevt & OEVT_B_DEV_SES_VLD_DET_EVNT)
		otg_dbg(otg, "B-Device Session Valid Detect Event\n");
	if (oevt & OEVT_B_DEV_VBUS_CHNG_EVNT)
		otg_dbg(otg, "B-Device VBUS Change Event\n");

	if ((oevt & otg_mask) || (adpevt & adp_mask)) {
		/* Pass event to main thread */
		spin_lock_irqsave(&otg->lock, flags);
		otg->otg_events |= oevt;
		otg->adp_events |= adpevt;
		wakeup_main_thread(otg);
		spin_unlock_irqrestore(&otg->lock, flags);
	}

	return IRQ_HANDLED;
}

static int dwc_otg2_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct dwc_otg2 *otg;

	if (!x) {
		otg_err(otg, "%s, otg is NULL!\n", __func__);
		return -ENODEV;
	}

	otg = xceiv_to_dwc_otg2(x);
	otg_dbg(otg, "\n");

	if (!gadget) {
		otg->otg.gadget = NULL;
		stop_main_thread(otg);
		return -ENODEV;
	}

	otg->otg.gadget = gadget;
	otg->otg.state = OTG_STATE_B_IDLE;
	start_main_thread(otg);
	return 0;
}

static int dwc_otg2_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct dwc_otg2 *otg;

	if (!x) {
		otg_err(otg, "%s, otg is NULL!\n", __func__);
		return -ENODEV;
	}

	otg = xceiv_to_dwc_otg2(x);
	otg_dbg(otg, "\n");

	if (!host) {
		otg->otg.host = NULL;
		stop_main_thread(otg);
		return -ENODEV;
	}

	otg->otg.host = host;
	start_main_thread(otg);
	return 0;
}

static int dwc_otg2_received_host_release(struct otg_transceiver *x)
{
	unsigned long flags;
	struct dwc_otg2 *otg;
	if (!x) {
		otg_err(otg, "%s otg is NULL!\n", __func__);
		return -ENODEV;
	}

	otg = xceiv_to_dwc_otg2(x);
	otg_dbg(otg, "\n");

	if (!otg->otg.host || !otg->otg.gadget)
		return -ENODEV;

	spin_lock_irqsave(&otg->lock, flags);
	otg->user_events |= PCD_RECEIVED_HOST_RELEASE_EVENT;
	wakeup_main_thread(otg);
	spin_unlock_irqrestore(&otg->lock, flags);
	return 0;
}

static struct dwc_otg2 *the_transceiver;
static int dwc_otg_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	struct resource		res[2];
	struct dwc_otg2 *otg;
	struct platform_device *dwc_host, *dwc_gadget;
	unsigned long resource, len;
	int retval;
	otg_dbg(otg, "%s is calling\n", __func__);

	if (pci_enable_device(pdev) < 0) {
		otg_err(otg, "pci device enable failed\n");
		return -ENODEV;
	}

	pci_set_power_state(pdev, PCI_D0);
	pci_set_master(pdev);

	otg = kzalloc(sizeof *otg, GFP_KERNEL);
	if (!otg) {
		otg_err(otg, "Alloc otg failed\n");
		return -ENOMEM;
	}

	the_transceiver = otg;

	/* control register: BAR 0 */
	resource = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	if (!request_mem_region(resource, len, driver_name)) {
		otg_err(otg, "Request memory region failed\n");
		retval = -EBUSY;
		goto exit;
	}

	otg_dbg(otg, "dwc otg pci resouce: 0x%lu, len: 0x%lu\n", \
			resource, len);
	otg_dbg(otg, "vendor: 0x%x, device: 0x%x\n", \
			pdev->vendor, pdev->device);

	otg->otg.io_priv = ioremap_nocache(resource, len);
	if (otg->otg.io_priv == NULL) {
		otg_err(otg, "ioremap failed\n");
		retval = -EFAULT;
		goto exit;
	}

	if (!pdev->irq) {
		otg_err(otg, "No IRQ.\n");
		retval = -ENODEV;
		goto exit;
	}

	otg->dev		= &pdev->dev;
	otg->otg.dev		= otg->dev;
	otg->otg.label		= driver_name;
	otg->otg.state		= OTG_STATE_UNDEFINED;
	otg->otg.set_host	= dwc_otg2_set_host;
	otg->otg.set_peripheral	= dwc_otg2_set_peripheral;
	otg->otg.set_suspend	= dwc_otg2_set_suspend;
	otg->otg.host_release   = dwc_otg2_received_host_release;

	otg->state = DWC_STATE_INIT;
	spin_lock_init(&otg->lock);
	init_waitqueue_head(&otg->main_wq);

	otg_dbg(otg, "\n");
	retval = otg_set_transceiver(&otg->otg);
	if (retval) {
		otg_err(otg, "can't register transceiver, err: %d\n",
			retval);
		goto exit;
	}

	dwc_gadget = platform_device_alloc(DWC3_DEVICE_NAME, GADGET_DEVID);
	dwc_host = platform_device_alloc(DWC3_HOST_NAME, HOST_DEVID);

	if (!dwc_gadget || !dwc_host) {
		otg_err(otg, "couldn't allocate dwc3 device\n");
		goto exit;
	}

	memset(res, 0x00, sizeof(struct resource) * ARRAY_SIZE(res));

	res[0].start	= pci_resource_start(pdev, 0);
	res[0].end	= pci_resource_end(pdev, 0);
	res[0].name	= "dwc_usb3_io";
	res[0].flags	= IORESOURCE_MEM;

	res[1].start	= pdev->irq;
	res[1].name	= "dwc_usb3_irq";
	res[1].flags	= IORESOURCE_IRQ;

	retval = platform_device_add_resources(dwc_host, res, ARRAY_SIZE(res));
	if (retval) {
		otg_err(otg, "couldn't add resources to dwc3 device\n");
		goto exit;
	}

	retval = platform_device_add_resources(dwc_gadget,
				res, ARRAY_SIZE(res));
	if (retval) {
		otg_err(otg, "couldn't add resources to dwc3 device\n");
		goto exit;
	}

	dwc_host->dev.dma_mask = dwc_gadget->dev.dma_mask =
		pdev->dev.dma_mask;
	dwc_gadget->dev.dma_parms = dwc_host->dev.dma_parms =
		pdev->dev.dma_parms;
	dwc_host->dev.parent = dwc_gadget->dev.parent =
		&pdev->dev;

	platform_par = kzalloc(sizeof *platform_par, GFP_KERNEL);
	if (!platform_par) {
		otg_err(otg, "alloc dwc_device_par failed\n");
		return -ENOMEM;
	}
	platform_par->io_addr = otg->otg.io_priv;
	platform_par->len = len;
	platform_device_add_data(dwc_gadget, platform_par, \
			sizeof(struct dwc_device_par));
	platform_device_add_data(dwc_host, platform_par, \
			sizeof(struct dwc_device_par));

	if (platform_device_add(dwc_gadget) || platform_device_add(dwc_host)) {
		otg_err(otg, "failed to register dwc3 device\n");
		goto exit;
	}
	otg->host = dwc_host;
	otg->gadget = dwc_gadget;
	otg->irqnum = pdev->irq;

	return 0;
exit:
	if (the_transceiver)
		dwc_otg_remove(pdev);
	free_irq(otg->irqnum, NULL);

	return retval;
}


static void __devexit dwc_otg_remove(struct pci_dev *pdev)
{
	struct dwc_otg2 *otg = the_transceiver;

	if (otg->host)
		platform_device_unregister(otg->host);

	if (otg->gadget)
		platform_device_unregister(otg->gadget);

	if (pdev->irq) {
		otg_err(otg, "free_irq\n");
		free_irq(otg->irqnum, NULL);
	}

	pci_disable_device(pdev);
	otg_set_transceiver(NULL);
	otg_dbg(otg, "\n");
	kfree(otg);
}


#define PCI_DEVICE_ID_DWC 0x119E
static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{ PCI_DEVICE_CLASS(((PCI_CLASS_SERIAL_USB << 8) | 0x20), ~0),
		.vendor = PCI_VENDOR_ID_INTEL,
		.device = PCI_DEVICE_ID_DWC,
	},
	{ /* end: all zeroes */ }
};

static struct pci_driver dwc_otg_pci_driver = {
	.name =		(char *) driver_name,
	.id_table =	pci_ids,
	.probe =	dwc_otg_probe,
	.remove =	dwc_otg_remove,
	.driver = {
		.name = (char *) driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init dwc_otg_init(void)
{
	int retval;
	retval = pci_register_driver(&dwc_otg_pci_driver);
	mutex_init(&lock);

	return retval;
}
module_init(dwc_otg_init);

static void __exit dwc_otg_exit(void)
{
	pci_unregister_driver(&dwc_otg_pci_driver);
	mutex_destroy(&lock);
}
module_exit(dwc_otg_exit);

MODULE_AUTHOR("Synopsys, Inc");
MODULE_DESCRIPTION("Synopsys DWC USB 3.0 with OTG 2.0/3.0");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("2.00a");
