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
#include <linux/version.h>

#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/dwc_otg3.h>
#include <asm/intel_scu_ipc.h>

#define VERSION "2.10a"

static int otg_id = -1;
static struct mutex lock;
static const char driver_name[] = "dwc_otg3";
static void __devexit dwc_otg_remove(struct pci_dev *pdev);
static struct dwc_device_par *platform_par;
static struct dwc_otg2 *the_transceiver;

static struct {

		int hibernate;

} otg_params = {

		.hibernate = 0,

};

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
	if (!otg->main_thread)
		return;

	otg_dbg(otg, "\n");
	/* Tell the main thread that something has happened */
	otg->main_wakeup_needed = 1;
	wake_up_interruptible(&otg->main_wq);
}

static int sleep_main_thread_timeout(struct dwc_otg2 *otg, int msecs)
{
	signed long jiffies;
	int rc = msecs;

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exiting\n");
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
	rc = wait_event_freezable_timeout(otg->main_wq,
					otg->main_wakeup_needed,
					jiffies);

	if (otg->state == DWC_STATE_EXIT) {
		otg_dbg(otg, "Main thread exiting\n");
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
	int ret = 0, flg;
	u32 osts, octl;
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

	/* Power the port only for A-host */
	if (otg->state == DWC_STATE_A_HOST) {

		/* Spin osts xhciPrtPwr bit until it becomes 1 */
		osts = otg_read(otg, OSTS);
		flg = handshake(otg, OSTS,
				OSTS_XHCI_PRT_PWR,
				OSTS_XHCI_PRT_PWR,
				1000);
		if (flg) {
			otg_dbg(otg, "Port is powered by xhci-hcd\n");
			/* Set port power control bit */
			octl = otg_read(otg, OCTL);
			octl |= OCTL_PRT_PWR_CTL;
			otg_write(otg, OCTL, octl);
		} else {
			otg_dbg(otg, "Port is not powered by xhci-hcd\n");
		}
	}

	return ret;
}

static int stop_host(struct dwc_otg2 *otg)
{
	int ret = -1;
	struct usb_hcd *hcd = NULL;

	otg_dbg(otg, "\n");

	if (otg->otg.host) {
		hcd = container_of(otg->otg.host, struct usb_hcd, self);
		ret = hcd->driver->stop_host(hcd);
	}

	return ret;
}

static void start_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget;

	otg_dbg(otg, "\n");

	gadget = otg->otg.gadget;
	if (!gadget) {
		otg_err(otg, "Haven't set gadget yet!\n");
		return;
	}

	if (!set_peri_mode(otg, PERI_MODE_PERIPHERAL))
		otg_err(otg, "Failed to start peripheral.");

	gadget->ops->start_device(gadget);
}

static void stop_peripheral(struct dwc_otg2 *otg)
{
	struct usb_gadget *gadget = otg->otg.gadget;

	if (!gadget)
		return;

	otg_dbg(otg, "\n");
	gadget->ops->stop_device(gadget);
}

static int get_id(struct dwc_otg2 *otg)
{
	int ret, id = RID_UNKNOWN;
	u8 idsts;

	ret = intel_scu_ipc_ioread8(PMIC_USBIDSTS, &idsts);
	if (ret) {
		otg_err(otg, "Fail to read id\n");
		return id;
	}

	if (idsts & USBIDSTS_ID_FLOAT_STS)
		id = RID_FLOAT;
	else if (idsts & USBIDSTS_ID_GND)
		id = RID_GND;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(1))
		id = RID_A;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(2))
		id = RID_B;
	else if (idsts & USBIDSTS_ID_RARBRC_STS(3))
		id = RID_C;

	return id;
}

#ifdef CONFIG_DWC_CHARGER_DETECION
static int ulpi_read(struct dwc_otg2 *otg, const u8 reg, u8 *val)
{
	u32 val32 = 0, count = 200;


	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSBSY)
			udelay(5);
		else
			break;

		count--;
	}

	if (!count) {
		otg_err(otg, "USB2 PHY always busy!!\n");
		return -EBUSY;
	}

	count = 200;
	val32 = GUSB2PHYACC0_NEWREGREQ | GUSB2PHYACC0_REGADDR(reg);
	otg_write(otg, GUSB2PHYACC0, val32);

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSDONE) {
			*val = otg_read(otg, GUSB2PHYACC0) & \
				   GUSB2PHYACC0_REGDATA_MASK;
			otg_dbg(otg, "%s - reg 0x%x data 0x%x\n",\
					__func__, reg, *val);
			return 0;
		}

		count--;
	}

	otg_err(otg, "%s read PHY data failed.\n", __func__);

	return -ETIMEDOUT;
}


static int ulpi_write(struct dwc_otg2 *otg, const u8 reg, const u8 val)
{
	u32 val32 = 0, count = 200;


	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSBSY)
			udelay(5);
		else
			break;

		count--;
	}

	if (!count) {
		otg_err(otg, "USB2 PHY always busy!!\n");
		return -EBUSY;
	}

	count = 200;
	val32 = GUSB2PHYACC0_NEWREGREQ | GUSB2PHYACC0_REGADDR(reg)
		| GUSB2PHYACC0_REGWR | GUSB2PHYACC0_REGDATA(val);

	otg_write(otg, GUSB2PHYACC0, val32);

	while (count) {
		if (otg_read(otg, GUSB2PHYACC0) & GUSB2PHYACC0_VSTSDONE) {
			otg_dbg(otg, "%s - reg 0x%x data 0x%x write done\n",\
					__func__, reg, val);
			return val;
		}

		count--;
	}

	otg_err(otg, "%s read PHY data failed.\n", __func__);

	return -ETIMEDOUT;
}

static enum usb_charger_type aca_check(struct dwc_otg2 *otg)
{
	u8 rarbrc, vdat_det, chgd_serx_dm;
	enum usb_charger_type type = CHRG_UNKNOWN;
	int ret;

	/*Enable VDAT_DET comparator.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_SET, PWCTRL_DP_VSRC_EN);

	/*Wait >66.1ms (for TCHGD_SERX_DEB)*/
	msleep(66);

	/*Read decoded RID value ????*/
	ret = intel_scu_ipc_ioread8(PMIC_USBIDSTS, &rarbrc);
	if (ret)
		otg_err(otg, "Fail to read decoded RID value\n");
	rarbrc &= USBIDSTS_ID_RARBRC_STS(3);

	ret = ulpi_read(otg, TUSB1211_POWER_CONTROL, &vdat_det);
	if (ret < 0) {
		otg_err(otg, "ULPI read error!\n");
		return CHRG_UNKNOWN;
	}
	vdat_det &= PWCTRL_VDAT_DET;

	ret = ulpi_read(otg, TUSB1211_VENDOR_SPECIFIC4, &chgd_serx_dm);
	if (ret < 0) {
		otg_err(otg, "ULPI read error!\n");
		return CHRG_UNKNOWN;
	}
	chgd_serx_dm &= VS4_CHGD_SERX_DM;

	/* If ID_RARBRC_STS==01 &&
	 * (VDAT_DET==1 && CHGD_SERX_DM==0): ACA-Dock detected
	 * If ID_RARBRC_STS==01 &&
	 * (VDAT_DET==0 || CHGD_SERX_DM==1): ACA-A detected
	 * If ID_RARBRC_STS==10: ACA-B detected
	 * If ID_RARBRC_STS==11: ACA-C detected
	 * If ID_RARBRC_STS==00: MHL detected
	 * */
	if (rarbrc & (vdat_det && !chgd_serx_dm)) {
		/*ACA-Dock*/
		type = CHRG_ACA_DOCK;
	} else if (rarbrc && (!vdat_det || chgd_serx_dm)) {
		/*ACA-A*/
		type = CHRG_ACA_A;
	} else if (rarbrc == 0x2) {
		/*ACA-B*/
		type = CHRG_ACA_B;
	} else if (rarbrc == 0x3) {
		/*ACA-C*/
		type = CHRG_ACA_C;
	} else if (!rarbrc) {
		otg_dbg(otg, "detect MHL device\n");
		type = CHRG_MHL;
	}

	/*Disable VDAT_DET comparator.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_CLR, PWCTRL_DP_VSRC_EN);

	return type;
}

static enum usb_charger_type get_charger_type(struct dwc_otg2 *otg)
{
	u8 val, vdat_det, chgd_serx_dm;
	int ret, count = 0;
	enum usb_charger_type type = CHRG_UNKNOWN;

	/* PHY Enable: De-assert USBRST*/
	ret = intel_scu_ipc_update_register(PMIC_USBPHYCTRL, \
			PMIC_USBPHYCTRL_D0,  PMIC_USBPHYCTRL_D0);
	if (ret)
		otg_err(otg, "Fail to de-assert USBRST\n");

	/* Wait 10ms (~5ms before PHY de-asserts DIR,
	 * XXus for initial Link reg sync-up).*/
	msleep(20);

	/* ACA Enable: Enable ACA & ID detection logic.*/
	ret = intel_scu_ipc_update_register(PMIC_USBIDCTRL, \
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0, \
			USBIDCTRL_ACA_DETEN_D1 | PMIC_USBPHYCTRL_D0);
	if (ret)
		otg_err(otg, "Fail to enable ACA&ID detection logic\n");

	/* Enable DP/DM pulldowns (to ensure PHY RX comparators don't float) */
	ulpi_write(otg, TUSB1211_OTG_CTRL_SET, OTGCTRL_DMPULLDOWN |
						OTGCTRL_DPPULLDOWN);

	/* DCD Enable: Change OPMODE to 01 (Non-driving),
	 * TermSel to 0, &
	 * XcvrSel to 01 (enable FS xcvr)
	 */
	ulpi_write(otg, TUSB1211_FUNC_CTRL_SET, \
			FUNCCTRL_OPMODE(1) | FUNCCTRL_XCVRSELECT(1));
	ulpi_write(otg, TUSB1211_FUNC_CTRL_CLR, \
			FUNCCTRL_OPMODE(2) | FUNCCTRL_XCVRSELECT(2) \
			| FUNCCTRL_TERMSELECT);

	/*Disable DP pulldown (to allow weak Idp_src for DCD)*/
	ulpi_write(otg, TUSB1211_OTG_CTRL_CLR, OTGCTRL_DPPULLDOWN);

	/*Enable SW control*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_SET, PWCTRL_SW_CONTROL);

	/*Enable IDPSRC*/
	ulpi_write(otg, TUSB1211_VENDOR_SPECIFIC3_SET, VS3_CHGD_IDP_SRC_EN);

	do {
		msleep(20);
		count++;
		/*Read DP logic level.*/
		ret = ulpi_read(otg, TUSB1211_VENDOR_SPECIFIC4, &val);
		if (ret < 0) {
			otg_err(otg, "ULPI read error!\n");
			continue;
		}
		if (count > 80) {
			otg_err(otg, "ULPI read TUSB1211_VENDOR_SPECIFIC4 timeout!\n");
			break;
		}
		val &= ~VS4_CHGD_SERX_DP;
	} while (val != 0);

	/*Disable DP pullup (Idp_src)*/
	ulpi_write(otg, TUSB1211_VENDOR_SPECIFIC3_CLR, VS3_CHGD_IDP_SRC_EN);

	/*Check ID pin state.*/
	ret = intel_scu_ipc_ioread8(PMIC_USBIDCTRL, &val);
	if (ret)
		otg_err(otg, "Fail to enable ACA&ID detection logic\n");
	val &= USBIDSTS_ID_FLOAT_STS;
	if (!val) {
		type = aca_check(otg);
		goto cleanup;
	}

	/*Enable DP/DM pulldowns (to ensure PHY RX comparators don't float).*/
	ulpi_write(otg, TUSB1211_OTG_CTRL_SET, \
			OTGCTRL_DMPULLDOWN | OTGCTRL_DPPULLDOWN);

	/*Read DP/DM logic level.*/
	ret = ulpi_read(otg, TUSB1211_VENDOR_SPECIFIC4, &val);
	if (ret < 0)
		otg_err(otg, "ULPI read error!\n");

	val &= VS4_CHGD_SERX_DP | VS4_CHGD_SERX_DM;

	/*If '11': SE1 detected; goto 'Cleanup'.
	 * Else: goto 'Pri Det Enable'.
	 */
	if (val == 3)
		goto cleanup;

	/*Pri Det Enable: Enable VDPSRC.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_SET, PWCTRL_DP_VSRC_EN);

	/*Wait >106.1ms (40ms for BC Tvdpsrc_on, 66.1ms for TI CHGD_SERX_DEB).*/
	msleep(107);

	/*Pri Det Check: Check if DM > VDATREF.*/
	ret = ulpi_read(otg, TUSB1211_POWER_CONTROL, &vdat_det);
	if (ret < 0)
		otg_err(otg, "ULPI read error!\n");

	vdat_det &= PWCTRL_VDAT_DET;

	ret = ulpi_read(otg, TUSB1211_VENDOR_SPECIFIC4, &chgd_serx_dm);
	if (ret < 0)
		otg_err(otg, "ULPI read error!\n");

	chgd_serx_dm &= VS4_CHGD_SERX_DM;

	if (vdat_det == 0 || chgd_serx_dm == 1)
		type = CHRG_SDP;

	/*Disable VDPSRC.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_CLR, PWCTRL_DP_VSRC_EN);

	if (type == CHRG_SDP)
		goto cleanup;

	/*Sec Det Enable: Sec Det Enable*/
	mdelay(1);

	/*Swap DP & DM*/
	ulpi_write(otg, TUSB1211_VENDOR_SPECIFIC1_CLR, VS1_DATAPOLARITY);

	/*Enable 'VDMSRC'.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_SET, PWCTRL_DP_VSRC_EN);

	/*Wait >73ms (40ms for BC Tvdmsrc_on, 33ms for TI TVDPSRC_DEB)*/
	msleep(73);

	/*Check if DP>VDATREF.*/
	ret = ulpi_read(otg, TUSB1211_POWER_CONTROL, &val);
	if (ret < 0)
		otg_err(otg, "ULPI read error!\n");

	val &= PWCTRL_VDAT_DET;

	if (val)
		type = CHRG_CDP;
	else
		type = CHRG_DCP;

	/*Disable VDMSRC.*/
	ulpi_write(otg, TUSB1211_POWER_CONTROL_CLR, PWCTRL_DP_VSRC_EN);

	/*Swap DP & DM.*/
	ulpi_write(otg, TUSB1211_VENDOR_SPECIFIC1_SET, VS1_DATAPOLARITY);

cleanup:

	/*If DCP detected, assert VDPSRC.*/
	if (type == CHRG_DCP)
		ulpi_write(otg, TUSB1211_POWER_CONTROL_SET, \
				PWCTRL_SW_CONTROL | PWCTRL_DP_VSRC_EN);

	return type;
}

static int is_self_powered_b_device(struct dwc_otg2 *otg)
{
	return get_id(otg) == RID_GND;
}
#endif

static enum dwc_otg_state do_wait_vbus_raise(struct dwc_otg2 *otg)
{
	int ret;
	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_B_SES_VLD_EVT |
				OEVT_CONN_ID_STS_CHNG_EVNT;

	ret = sleep_until_event(otg, otg_mask, \
			0, user_mask, &otg_events,\
			NULL, &user_events, VBUS_TIMEOUT);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_B_SES_VLD_EVT) {
		otg_dbg(otg, "OEVT_B_SES_VLD_EVT\n");
		return DWC_STATE_CHARGER_DETECTION;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		return DWC_STATE_INIT;
	}

	/* timeout*/
	if (ret) {
		if (get_id(otg) == RID_GND)
			return DWC_STATE_A_HOST;
	}

	return DWC_STATE_INVALID;
}

static enum dwc_otg_state do_wait_vbus_fall(struct dwc_otg2 *otg)
{
	int ret;

	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_A_DEV_SESS_END_DET_EVNT;

	ret = sleep_until_event(otg, otg_mask, \
			0, user_mask, &otg_events,\
			NULL, &user_events, VBUS_TIMEOUT);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		return DWC_STATE_INIT;
	}

	/* timeout*/
	if (ret)
		return DWC_STATE_CHARGER_DETECTION;

	return DWC_STATE_INVALID;
}

static enum dwc_otg_state do_charging(struct dwc_otg2 *otg)
{
	int ret;
	u32 otg_events = 0;
	u32 user_events = 0;
	u32 otg_mask = 0;
	u32 user_mask = 0;

	otg_mask = OEVT_A_DEV_SESS_END_DET_EVNT |
			OEVT_CONN_ID_STS_CHNG_EVNT;

	ret = sleep_until_event(otg, otg_mask, \
			0, user_mask, &otg_events,\
			NULL, &user_events, 0);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		return DWC_STATE_INIT;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		return DWC_STATE_WAIT_VBUS_FALL;
	}

	return DWC_STATE_INVALID;
}

static enum dwc_otg_state do_charger_detection(struct dwc_otg2 *otg)
{
#ifdef CONFIG_DWC_CHARGER_DETECION
	enum dwc_otg_state state = DWC_STATE_INVALID;
	enum usb_charger_type charger = CHRG_UNKNOWN;

	charger = get_charger_type(otg);
	switch (charger) {
	case CHRG_SDP:
	case CHRG_CDP:
	case CHRG_ACA_C:
		state = DWC_STATE_B_PERIPHERAL;
		break;
	case CHRG_ACA_DOCK:
	case CHRG_ACA_A:
		state = DWC_STATE_A_HOST;
		break;
	case CHRG_DCP:
	case CHRG_ACA_B:
		state = DWC_STATE_CHARGING;
		break;
	case CHRG_UNKNOWN:
	default:
		if (is_self_powered_b_device(otg)) {
			state = DWC_STATE_A_HOST;
			break;
		}
	};

	return state;
#else
	otg_dbg(otg, "Currently, haven't support charger detection.\n");

	return DWC_STATE_INIT;
#endif
}

static int init_b_device(struct dwc_otg2 *otg)
{
	otg_dbg(otg, "\n");

	if (!set_peri_mode(otg, PERI_MODE_PERIPHERAL))
		otg_err(otg, "Failed to start peripheral.");

	return DWC_STATE_B_PERIPHERAL;
}

static int init_a_device(struct dwc_otg2 *otg)
{
	otg_write(otg, OCFG, 0);
	otg_write(otg, OCTL, 0);

	return DWC_STATE_A_HOST;
}

static enum dwc_otg_state do_connector_id_status(struct dwc_otg2 *otg)
{
	int ret;
	u32 events = 0, user_events = 0;
	u32 otg_mask = 0, user_mask = 0, phyval;
	enum dwc_otg_state state = DWC_STATE_INVALID;

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

#ifdef CONFIG_DWC_CHARGER_DETECION
	/* Force config to device mode as default */
	gctl = otg_read(otg, GCTL);
	gctl &= ~GCTL_PRT_CAP_DIR;
	gctl |= GCTL_PRT_CAP_DIR_DEV;
	otg_write(otg, GCTL, gctl);
#endif

	/* This is a hardware workaround.
	 * xHCI RxDetect state is not work well when USB3
	 * PHY under P3 state. So force PHY change to P2 when
	 * xHCI want to perform receiver detection.
	 */
	phyval = otg_read(otg, GUSB3PIPECTL0);
	phyval |= GUSB3PIPE_DISRXDETP3;
	otg_write(otg, GUSB3PIPECTL0, phyval);

	msleep(60);

#ifndef SUPPORT_USER_ID_CHANGE_EVENTS
	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT | \
			   OEVT_B_SES_VLD_EVT;
#else
	user_mask = USER_ID_B_CHANGE_EVENT | \
				USER_ID_A_CHANGE_EVENT;
#endif

#ifdef CONFIG_DWC_CHARGER_DETECION
	ret = ulpi_read(otg, TUSB1211_VENDOR_ID_LO, &val);
	if (ret < 0)
		printk(KERN_ERR "ulpi read error!\n");
	printk(KERN_ERR "Vendor ID low = 0x%x\n", val);
	ret = ulpi_read(otg, TUSB1211_VENDOR_ID_HI, &val);
	if (ret < 0)
		printk(KERN_ERR "ulpi read error!\n");
	printk(KERN_ERR "Vendor ID high = 0x%x\n", val);
	ret = ulpi_read(otg, TUSB1211_PRODUCT_ID_LO, &val);
	if (ret < 0)
		printk(KERN_ERR "ulpi read error!\n");
	printk(KERN_ERR "Product ID low = 0x%x\n", val);
	ret = ulpi_read(otg, TUSB1211_PRODUCT_ID_HI, &val);
	if (ret < 0)
		printk(KERN_ERR "ulpi read error!\n");
	printk(KERN_ERR "Product ID high = 0x%x\n", val);
#endif

	ret = sleep_until_event(otg, otg_mask, \
			0, user_mask, &events,\
			NULL, &user_events, 0);
	if (ret < 0)
		return DWC_STATE_EXIT;

	if (events & OEVT_B_DEV_SES_VLD_DET_EVNT) {
		otg_dbg(otg, "OEVT_B_DEV_SES_VLD_DET_EVNT\n");
		state = DWC_STATE_CHARGER_DETECTION;
	}

	if (events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		state = DWC_STATE_WAIT_VBUS_RAISE;
	}

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
	if (user_events & USER_ID_A_CHANGE_EVENT) {
		otg_dbg(otg, "events is user id A change\n");
		state = init_a_device(otg);
	}

	if (user_events & USER_ID_B_CHANGE_EVENT) {
		otg_dbg(otg, "events is user id B change\n");
		state = init_b_device(otg);
	}
#endif

	/** TODO: This is a workaround for latest hibernation-enabled bitfiles
     ** which have problems before initializing SRP.*/
	mdelay(50);

	return state;
}

static void reset_hw(struct dwc_otg2 *otg)
{
	u32 gctl = 0;
	otg_dbg(otg, "\n");
	otg_write(otg, OEVTEN, 0);
	otg_write(otg, OCTL, 0);
	gctl = otg_read(otg, GCTL);
	gctl |= GCTL_PRT_CAP_DIR_OTG << GCTL_PRT_CAP_DIR_SHIFT;
	if (otg_params.hibernate)
		gctl |= GCTL_GBL_HIBERNATION_EN;
	otg_write(otg, GCTL, gctl);
}

static enum dwc_otg_state do_a_host(struct dwc_otg2 *otg)
{
	int rc = 0, id = RID_UNKNOWN;
	u32 otg_events, user_events, otg_mask, user_mask;

	set_sus_phy(otg, 0);
	otg_write(otg, GCTL, 0x45801000);

	rc = start_host(otg);
	if (rc < 0) {
		otg_err(otg, "start_host failed!");
		return DWC_STATE_INVALID;
	}

stay_host:
	otg_events = 0;
	user_events = 0;
	otg_mask = 0;
	user_mask = 0;

#ifndef SUPPORT_USER_ID_CHANGE_EVENTS
	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT |
			OEVT_A_DEV_SESS_END_DET_EVNT;
#else
	user_mask = USER_ID_B_CHANGE_EVENT;
#endif

	rc = sleep_until_event(otg,
			otg_mask, 0, user_mask,
			&otg_events, NULL, &user_events, 0);
	if (rc < 0)
		return DWC_STATE_EXIT;

	/* Higher priority first */
	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		return DWC_STATE_INIT;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		id = get_id(otg);
		if ((id != RID_A) && (id != RID_GND))
			return DWC_STATE_WAIT_VBUS_FALL;
		else
			goto stay_host;
	}

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
	/* Higher priority first */
	if (user_events & USER_ID_B_CHANGE_EVENT) {
		otg_dbg(otg, "USER_ID_B_CHANGE_EVENT\n");
		return DWC_STATE_B_PERIPHERAL;
	}
#endif

	/* Invalid state */
	return DWC_STATE_INVALID;
}

static int do_b_peripheral(struct dwc_otg2 *otg)
{
	int rc = 0, id = RID_UNKNOWN;
	u32 otg_mask, user_mask, otg_events, user_events;

stay_device:
	otg_mask = 0;
	user_mask = 0;
	otg_events = 0;
	user_events = 0;

#ifndef SUPPORT_USER_ID_CHANGE_EVENTS
	otg_mask = OEVT_CONN_ID_STS_CHNG_EVNT \
			| OEVT_A_DEV_SESS_END_DET_EVNT;
#else
	user_mask = USER_ID_A_CHANGE_EVENT;
#endif

	rc = sleep_until_event(otg,
			otg_mask, 0, user_mask,
			&otg_events, NULL, &user_events, 0);
	if (rc < 0)
		return DWC_STATE_EXIT;

	if (otg_events & OEVT_A_DEV_SESS_END_DET_EVNT) {
		otg_dbg(otg, "OEVT_A_DEV_SESS_END_DET_EVNT\n");
		return DWC_STATE_INIT;
	}

	if (otg_events & OEVT_CONN_ID_STS_CHNG_EVNT) {
		otg_dbg(otg, "OEVT_CONN_ID_STS_CHNG_EVNT\n");
		id = get_id(otg);
		if ((id != RID_C) && (id != RID_FLOAT))
			return DWC_STATE_WAIT_VBUS_FALL;
		else
			goto stay_device;
	}

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
	if (user_events & USER_ID_A_CHANGE_EVENT) {
		otg_dbg(otg, "USER_ID_A_CHANGE_EVENT\n");
		return DWC_STATE_A_HOST;
	}
#endif

	return DWC_STATE_INVALID;
}

/* Charger driver may send ID change and VBus change event to OTG driver.
 * This is like IRQ handler, just the event source is from charger driver.
 * Because on Merrifield platform, the ID line and VBus line are connect to
 * PMic which can make USB controller and PHY power off to save power.
 */
static int dwc_otg_handle_notification(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct dwc_otg2		*otg = the_transceiver;
	int state, val;
	unsigned long flags;

	if (((event == USB_EVENT_ID) || (event == USB_EVENT_VBUS))
			&& !otg)
		return NOTIFY_BAD;

	val = *(int *)data;

	spin_lock_irqsave(&otg->lock, flags);
	switch (event) {
	case USB_EVENT_ID:
		otg->otg_events |= OEVT_CONN_ID_STS_CHNG_EVNT;
		state = NOTIFY_OK;
		break;
	case USB_EVENT_VBUS:
		if (val)
			otg->otg_events |= OEVT_B_DEV_SES_VLD_DET_EVNT;
		else
			otg->otg_events |= OEVT_A_DEV_SESS_END_DET_EVNT;
		state = NOTIFY_OK;
		break;
	default:
		otg_dbg(otg, "DWC OTG Notify unknow notify message\n");
		state = NOTIFY_DONE;
	}
	wakeup_main_thread(otg);
	spin_unlock_irqrestore(&otg->lock, flags);

	return state;
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

	/* Register otg notifier to monitor ID and VBus change events */
	otg_register_notifier(&otg->otg, &otg->nb);

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
		case DWC_STATE_WAIT_VBUS_RAISE:
			otg_dbg(otg, "DWC_STATE_WAIT_VBUS_RAISE\n");
			next = do_wait_vbus_raise(otg);
			break;
		case DWC_STATE_WAIT_VBUS_FALL:
			otg_dbg(otg, "DWC_STATE_WAIT_VBUS_FALL\n");
			next = do_wait_vbus_fall(otg);
			break;
		case DWC_STATE_CHARGING:
			otg_dbg(otg, "DWC_STATE_CHARGING\n");
			next = do_charging(otg);
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

	otg->main_thread = NULL;
	otg_dbg(otg, "OTG main thread exiting....\n");

	return 0;
}

static void start_main_thread(struct dwc_otg2 *otg)
{
	mutex_lock(&lock);
	if (!otg->main_thread /* && otg->otg.gadget */&& otg->otg.host) {
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
		wakeup_main_thread(otg);
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

#ifndef SUPPORT_USER_ID_CHANGE_EVENTS
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
#endif

static int dwc_otg2_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct dwc_otg2 *otg;

	if (!x) {
		printk(KERN_ERR "otg is NULL!\n");
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
		printk(KERN_ERR "otg is NULL!\n");
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
		printk(KERN_ERR "otg is NULL!\n");
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

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
static ssize_t store_otg_id(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dwc_otg2		*otg = the_transceiver;
	unsigned long flags;

	if (count != 2) {
		otg_err(otg, "return EINVAL\n");
		return -EINVAL;
	}

	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;

	switch (buf[0]) {
	case 'a':
	case 'A':
		otg_dbg(otg, "Change ID to A\n");
		otg->user_events |= USER_ID_A_CHANGE_EVENT;
		spin_lock_irqsave(&otg->lock, flags);
		wakeup_main_thread(otg);
		otg_id = 0;
		spin_unlock_irqrestore(&otg->lock, flags);
		return count;
	case 'b':
	case 'B':
		otg_dbg(otg, "Change ID to B\n");
		otg->user_events |= USER_ID_B_CHANGE_EVENT;
		spin_lock_irqsave(&otg->lock, flags);
		wakeup_main_thread(otg);
		otg_id = 1;
		spin_unlock_irqrestore(&otg->lock, flags);
		return count;
	default:
		otg_err(otg, "Just support change ID to A!\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t
show_otg_id(struct device *_dev, struct device_attribute *attr, char *buf)
{
	char				*next;
	unsigned			size, t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size,
		"USB OTG ID: %s\n",
		(otg_id ? "B" : "A")
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(otg_id, S_IRUGO|S_IWUSR|S_IWGRP,\
			show_otg_id, store_otg_id);
#endif

static int dwc_otg_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	struct resource		res[2];
	struct dwc_otg2 *otg;
	struct platform_device *dwc_host, *dwc_gadget;
	unsigned long resource, len;
	int retval;

	if (pci_enable_device(pdev) < 0) {
		printk(KERN_ERR "pci device enable failed\n");
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

	otg_dbg(otg, "Version: %s\n", VERSION);
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
	otg->nb.notifier_call = dwc_otg_handle_notification;

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
	retval = device_create_file(&pdev->dev, &dev_attr_otg_id);
	if (retval < 0) {
		otg_dbg(otg,
			"Can't register sysfs attribute: %d\n", retval);
		goto exit;
	}
#endif

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
MODULE_VERSION(VERSION);
