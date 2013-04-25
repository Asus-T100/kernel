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

#ifndef __DWC_OTG_H__
#define __DWC_OTG_H__

#include <linux/usb/intel_mid_otg.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/power_supply.h>


#define SUPPORT_USER_ID_CHANGE_EVENTS
struct dwc_device_par {
	void __iomem *io_addr;
	int len;
};

#define DWC3_DEVICE_NAME "dwc3-device"
#define DWC3_HOST_NAME "dwc3-host"
#define GADGET_DEVID 1
#define HOST_DEVID 2
#define DRIVER_VERSION "0.1"
#define PERI_MODE_PERIPHERAL	1
#define PERI_MODE_HOST		0
#define OTG_DEVICE_SUSPEND	0xfffe
#define OTG_DEVICE_RESUME	0xffff

#define ADP_RAMP_TIME_CHANGE_THRESHOLD	5
#define ADP_PROBE_TIMEOUT		5000
#define SRP_TIMEOUT			6000
#define RSP_CONFIRM_TIMEOUT 200
#define RSP_WRST_TIMEOUT 2000
#define RSP_ACK_TIMEOUT 2000
#define HNP_TIMEOUT	4000

#ifdef CONFIG_USB_DWC_OTG_XCEIV_DEBUG
#define DWC_OTG_DEBUG 1
#else
#define DWC_OTG_DEBUG 0
#endif

#define otg_dbg(d, fmt, args...)  \
	do { if (DWC_OTG_DEBUG) dev_dbg((d)->dev, \
			"%s(): " fmt , __func__, ## args); } while (0)
#define otg_vdbg(d, fmt, args...)  \
	do { if (DWC_OTG_DEBUG) dev_dbg((d)->dev, \
			"%s(): " fmt , __func__, ## args); } while (0)
#define otg_err(d, fmt, args...)  \
	do { if (DWC_OTG_DEBUG) dev_err((d)->dev, \
			"%s(): " fmt , __func__, ## args); } while (0)
#define otg_warn(d, fmt, args...)  \
	do { if (DWC_OTG_DEBUG) dev_warn((d)->dev, \
			"%s(): " fmt , __func__, ## args); } while (0)
#define otg_info(d, fmt, args...)  \
	do { if (DWC_OTG_DEBUG) dev_info((d)->dev, \
			"%s(): " fmt , __func__, ## args); } while (0)

#ifdef DEBUG
#define otg_write(o, reg, val)	do {					\
		otg_dbg(o, "OTG_WRITE: reg=0x%05x, val=0x%08x\n", reg, val); \
		if (otg->dev->power.runtime_status == RPM_SUSPENDED) { \
			dump_stack(); printk(KERN_ERR \
				"dwc otg_write: meet fabric error!\n"); } \
		writel(val, ((void *)((o)->phy.io_priv)) + reg);	\
	} while (0)

#define otg_read(o, reg) ({						\
		u32 __r; \
		if (otg->dev->power.runtime_status == RPM_SUSPENDED) { \
			dump_stack(); printk(KERN_ERR \
				"dwc otg_read: meet fabirc error!\n"); } \
		__r = readl(((void *)((o)->phy.io_priv)) + reg);	\
		otg_dbg(o, "OTG_READ: reg=0x%05x, val=0x%08x\n", reg, __r); \
		__r;							\
	})
#else
#define otg_write(o, reg, val)	do { \
	if (otg->dev->power.runtime_status == RPM_SUSPENDED) { \
		dump_stack(); printk(KERN_ERR \
		"dwc otg_write: meet fabric error!\n"); } \
		writel(val, ((void *)((o)->phy.io_priv)) + reg); \
	} while (0)
#define otg_read(o, reg)	({ \
	if (otg->dev->power.runtime_status == RPM_SUSPENDED) { \
		dump_stack(); printk(KERN_ERR \
		"dwc otg_read: meet fabirc error!\n"); } \
		readl(((void *)((o)->phy.io_priv)) + reg); \
	})
#endif

#define GUSB2PHYCFG0				0xc200
#define GUSB2PHYCFG_SUS_PHY                     0x40
#define GUSB2PHYCFG_PHYSOFTRST (1 << 31)

#define EXTEND_ULPI_REGISTER_ACCESS_MASK	0xC0
#define GUSB2PHYACC0	0xc280
#define GUSB2PHYACC0_DISULPIDRVR  (1 << 26)
#define GUSB2PHYACC0_NEWREGREQ  (1 << 25)
#define GUSB2PHYACC0_VSTSDONE  (1 << 24)
#define GUSB2PHYACC0_VSTSBSY  (1 << 23)
#define GUSB2PHYACC0_REGWR  (1 << 22)
#define GUSB2PHYACC0_REGADDR(v)  ((v & 0x3F) << 16)
#define GUSB2PHYACC0_EXTREGADDR(v)  ((v & 0x3F) << 8)
#define GUSB2PHYACC0_VCTRL(v)  ((v & 0xFF) << 8)
#define GUSB2PHYACC0_REGDATA(v)  (v & 0xFF)
#define GUSB2PHYACC0_REGDATA_MASK  0xFF
#define DATACON_TIMEOUT		750
#define DATACON_INTERVAL	10

#define GUSB3PIPECTL0                           0xc2c0
#define GUSB3PIPECTL_SUS_EN                     0x20000
#define GUSB3PIPE_DISRXDETP3                    (1 << 28)
#define GUSB3PIPECTL_PHYSOFTRST (1 << 31)

#define GHWPARAMS6				0xc158
#define GHWPARAMS6_SRP_SUPPORT_ENABLED		0x0400
#define GHWPARAMS6_HNP_SUPPORT_ENABLED		0x0800
#define GHWPARAMS6_ADP_SUPPORT_ENABLED		0x1000

#define GUCTL 0xC12C
#define GUCTL_CMDEVADDR		(1 << 15)
#define APBFC_EXIOTG3_MISC0_REG			0xF90FF85C

#define GCTL 0xc110
#define GCTL_PRT_CAP_DIR 0x3000
#define GCTL_PRT_CAP_DIR_SHIFT 12
#define GCTL_PRT_CAP_DIR_HOST 1
#define GCTL_PRT_CAP_DIR_DEV 2
#define GCTL_PRT_CAP_DIR_OTG 3
#define GCTL_GBL_HIBERNATION_EN 0x2
#define GCTL_CORESOFTRESET (1 << 11)

#define OCFG					0xcc00
#define OCFG_SRP_CAP				0x01
#define OCFG_SRP_CAP_SHIFT			0
#define OCFG_HNP_CAP				0x02
#define OCFG_HNP_CAP_SHIFT			1
#define OCFG_OTG_VERSION			0x04
#define OCFG_OTG_VERSION_SHIFT			2

#define GCTL					0xc110
#define OCTL					0xcc04
#define OCTL_HST_SET_HNP_EN			0x01
#define OCTL_HST_SET_HNP_EN_SHIFT		0
#define OCTL_DEV_SET_HNP_EN			0x02
#define OCTL_DEV_SET_HNP_EN_SHIFT		1
#define OCTL_TERM_SEL_DL_PULSE			0x04
#define OCTL_TERM_SEL_DL_PULSE_SHIFT		2
#define OCTL_SES_REQ				0x08
#define OCTL_SES_REQ_SHIFT			3
#define OCTL_HNP_REQ				0x10
#define OCTL_HNP_REQ_SHIFT			4
#define OCTL_PRT_PWR_CTL			0x20
#define OCTL_PRT_PWR_CTL_SHIFT			5
#define OCTL_PERI_MODE				0x40
#define OCTL_PERI_MODE_SHIFT			6

#define OEVT					0xcc08
#define OEVT_ERR				0x00000001
#define OEVT_ERR_SHIFT				0
#define OEVT_SES_REQ_SCS			0x00000002
#define OEVT_SES_REQ_SCS_SHIFT			1
#define OEVT_HST_NEG_SCS			0x00000004
#define OEVT_HST_NEG_SCS_SHIFT			2
#define OEVT_B_SES_VLD_EVT			0x00000008
#define OEVT_B_SES_VLD_EVT_SHIFT		3
#define OEVT_B_DEV_VBUS_CHNG_EVNT		0x00000100
#define OEVT_B_DEV_VBUS_CHNG_EVNT_SHIFT		8
#define OEVT_B_DEV_SES_VLD_DET_EVNT		0x00000200
#define OEVT_B_DEV_SES_VLD_DET_EVNT_SHIFT	9
#define OEVT_B_DEV_HNP_CHNG_EVNT		0x00000400
#define OEVT_B_DEV_HNP_CHNG_EVNT_SHIFT		10
#define OEVT_B_DEV_B_HOST_END_EVNT		0x00000800
#define OEVT_B_DEV_B_HOST_END_EVNT_SHIFT	11
#define OEVT_A_DEV_SESS_END_DET_EVNT		0x00010000
#define OEVT_A_DEV_SESS_END_DET_EVNT_SHIFT	16
#define OEVT_A_DEV_SRP_DET_EVNT			0x00020000
#define OEVT_A_DEV_SRP_DET_EVNT_SHIFT		17
#define OEVT_A_DEV_HNP_CHNG_EVNT		0x00040000
#define OEVT_A_DEV_HNP_CHNG_EVNT_SHIFT		18
#define OEVT_A_DEV_HOST_EVNT			0x00080000
#define OEVT_A_DEV_HOST_EVNT_SHIFT		19
#define OEVT_A_DEV_B_DEV_HOST_END_EVNT		0x00100000
#define OEVT_A_DEV_B_DEV_HOST_END_EVNT_SHIFT	20
#define OEVT_HOST_ROLE_REQ_INIT_EVNT            0x00400000
#define OEVT_HOST_ROLE_REQ_INIT_EVNT_SHIFT      22
#define OEVT_HOST_ROLE_REQ_CONFIRM_EVNT         0x00800000
#define OEVT_HOST_ROLE_REQ_CONFIRM_EVNT_SHIFT   23
#define OEVT_CONN_ID_STS_CHNG_EVNT		0x01000000
#define OEVT_CONN_ID_STS_CHNG_EVNT_SHIFT	24
#define OEVT_DEV_MOD_EVNT			0x80000000
#define OEVT_DEV_MOD_EVNT_SHIFT			31

#define OEVTEN					0xcc0c

#define OEVT_ALL (OEVT_CONN_ID_STS_CHNG_EVNT | \
		OEVT_HOST_ROLE_REQ_INIT_EVNT | \
		OEVT_HOST_ROLE_REQ_CONFIRM_EVNT | \
		OEVT_A_DEV_B_DEV_HOST_END_EVNT | \
		OEVT_A_DEV_HOST_EVNT | \
		OEVT_A_DEV_HNP_CHNG_EVNT | \
		OEVT_A_DEV_SRP_DET_EVNT | \
		OEVT_A_DEV_SESS_END_DET_EVNT | \
		OEVT_B_DEV_B_HOST_END_EVNT | \
		OEVT_B_DEV_HNP_CHNG_EVNT | \
		OEVT_B_DEV_SES_VLD_DET_EVNT | \
		OEVT_B_DEV_VBUS_CHNG_EVNT)

#define OSTS					0xcc10
#define OSTS_CONN_ID_STS			0x0001
#define OSTS_CONN_ID_STS_SHIFT			0
#define OSTS_A_SES_VLD				0x0002
#define OSTS_A_SES_VLD_SHIFT			1
#define OSTS_B_SES_VLD				0x0004
#define OSTS_B_SES_VLD_SHIFT			2
#define OSTS_XHCI_PRT_PWR			0x0008
#define OSTS_XHCI_PRT_PWR_SHIFT			3
#define OSTS_PERIP_MODE				0x0010
#define OSTS_PERIP_MODE_SHIFT			4
#define OSTS_OTG_STATES				0x0f00
#define OSTS_OTG_STATE_SHIFT			8

#define ADPCFG					0xcc20
#define ADPCFG_PRB_DSCHGS			0x0c000000
#define ADPCFG_PRB_DSCHG_SHIFT			26
#define ADPCFG_PRB_DELTAS			0x30000000
#define ADPCFG_PRB_DELTA_SHIFT			28
#define ADPCFG_PRB_PERS				0xc0000000
#define ADPCFG_PRB_PER_SHIFT			30

#define ADPCTL					0xcc24
#define ADPCTL_WB				0x01000000
#define ADPCTL_WB_SHIFT				24
#define ADPCTL_ADP_RES				0x02000000
#define ADPCTL_ADP_RES_SHIFT			25
#define ADPCTL_ADP_EN				0x04000000
#define ADPCTL_ADP_EN_SHIFT			26
#define ADPCTL_ENA_SNS				0x08000000
#define ADPCTL_ENA_SNS_SHIFT			27
#define ADPCTL_ENA_PRB				0x10000000
#define ADPCTL_ENA_PRB_SHIFT			28

#define ADPEVT					0xcc28
#define ADPEVT_RTIM_EVNTS			0x000007ff
#define ADPEVT_RTIM_EVNT_SHIFT			0
#define ADPEVT_ADP_RST_CMPLT_EVNT		0x02000000
#define ADPEVT_ADP_RST_CMPLT_EVNT_SHIFT		25
#define ADPEVT_ADP_TMOUT_EVNT			0x04000000
#define ADPEVT_ADP_TMOUT_EVNT_SHIFT		26
#define ADPEVT_ADP_SNS_EVNT			0x08000000
#define ADPEVT_ADP_SNS_EVNT_SHIFT		27
#define ADPEVT_ADP_PRB_EVNT			0x10000000
#define ADPEVT_ADP_PRB_EVNT_SHIFT		28

#define ADPEVTEN				0xcc2c
#define ADPEVTEN_ACC_DONE_EN			0x01000000
#define ADPEVTEN_ACC_DONE_EN_SHIFT		24
#define ADPEVTEN_ADP_RST_CMPLT_EVNT_EN		0x02000000
#define ADPEVTEN_ADP_RST_CMPLT_EVNT_EN_SHIFT	25
#define ADPEVTEN_ADP_TMOUT_EVNT_EN		0x04000000
#define ADPEVTEN_ADP_TMOUT_EVNT_EN_SHIFT	26
#define ADPEVTEN_ADP_SNS_EVNT_EN		0x08000000
#define ADPEVTEN_ADP_SNS_EVNT_EN_SHIFT		27
#define ADPEVTEN_ADP_PRB_EVNT_EN		0x10000000
#define ADPEVTEN_ADP_PRB_EVNT_EN_SHIFT		28

#define RID_A		0x01
#define RID_B		0x02
#define RID_C		0x03
#define RID_FLOAT	0x04
#define RID_GND		0x05
#define RID_UNKNOWN	0x00

/** The states for the OTG driver */
enum dwc_otg_state {
	DWC_STATE_INVALID = -1,

	/** The initial state, check the connector
	 * id status and determine what mode
	 * (A-device or B-device) to operate in. */
	DWC_STATE_INIT = 0,

	/* A-Host states */
	DWC_STATE_A_PROBE,
	DWC_STATE_A_HOST,
	DWC_STATE_A_HNP_INIT,

	/* A-Peripheral states */
	DWC_STATE_A_PERIPHERAL,

	/* B-Peripheral states */
	DWC_STATE_B_SENSE,
	DWC_STATE_B_PROBE,
	DWC_STATE_B_PERIPHERAL,
	DWC_STATE_B_HNP_INIT,

	/* B-Host states */
	DWC_STATE_B_HOST,

	/* RSP */
	DWC_STATE_B_RSP_INIT,
	DWC_STATE_CHARGER_DETECTION,

	/* VBUS */
	DWC_STATE_WAIT_VBUS_RAISE,
	DWC_STATE_WAIT_VBUS_FALL,

	/* Charging*/
	DWC_STATE_CHARGING,

	/* Exit */
	DWC_STATE_EXIT,
	DWC_STATE_TERMINATED
};

struct intel_dwc_otg_pdata {
	int is_hvp;
	int is_byt;
	int no_host_mode;
	int no_device_mode;
};

/** The main structure to keep track of OTG driver state. */
struct dwc_otg2 {
	/** OTG transceiver */
	struct usb_otg	otg;
	struct usb_phy	phy;
	struct device		*dev;
	int irqnum;

	int main_wakeup_needed;
	struct task_struct *main_thread;
	wait_queue_head_t main_wq;

	spinlock_t lock;

	/* Events */
	u32 otg_events;
	u32 adp_events;

	u32 user_events;
	/** User initiated SRP.
	 *
	 * Valid in B-device during sensing/probing. Initiates SRP signalling
	 * across the bus.
	 *
	 * Also valid as an A-device during probing. This causes the A-device to
	 * apply V-bus manually and check for a device. Can be used if the
	 * device does not support SRP and the host does not support ADP. */
#define USER_SRP_EVENT 0x1
	/** User initiated HNP (only valid in B-peripheral) */
#define USER_HNP_EVENT 0x2
	/** User has ended the session (only valid in B-peripheral) */
#define USER_END_SESSION 0x4
	/** User initiated VBUS. This will cause the A-device to turn on the
	 * VBUS and see if a device will connect (only valid in A-device during
	 * sensing/probing) */
#define USER_VBUS_ON 0x8
	/** User has initiated RSP */
#define USER_RSP_EVENT 0x10
	/** Host release event */
#define PCD_RECEIVED_HOST_RELEASE_EVENT 0x20
	/** Initial SRP */
#define INITIAL_SRP 0x40

#ifdef SUPPORT_USER_ID_CHANGE_EVENTS
	/** User space ID switch event */
#define USER_ID_A_CHANGE_EVENT 0x40
#define USER_ID_B_CHANGE_EVENT 0x80
#endif
	/** a_bus_drop event from userspace */
#define USER_A_BUS_DROP 0x100

	/* States */
	enum dwc_otg_state prev;
	enum dwc_otg_state state;
	struct platform_device *host;
	struct platform_device *gadget;

	/* Charger detection */
	struct power_supply_cable_props charging_cap;
	struct notifier_block nb;
	struct delayed_work sdp_check_work;
	struct intel_dwc_otg_pdata *otg_data;

	/* pm request to prevent enter Cx state
	 * Because it have big impact to USB performance
	 * */
	struct pm_qos_request *qos;
};

/* Invalid SDP checking timeout */
#define INVALID_SDP_TIMEOUT	(HZ * 15)
#define sleep_main_thread_until_condition_timeout(otg, condition, msecs) ({ \
		int __timeout = msecs;				\
		while (!(condition)) {				\
			otg_dbg(otg, "  ... sleeping for %d\n", __timeout); \
			__timeout = sleep_main_thread_timeout(otg, __timeout); \
			if (__timeout <= 0) {			\
				break;				\
			}					\
		}						\
		__timeout;					\
	})

#define sleep_main_thread_until_condition(otg, condition) ({	\
		int __rc = 0;					\
		do {						\
			__rc = sleep_main_thread_until_condition_timeout(otg, \
			condition, 50000); \
		} while (__rc == 0);				\
		__rc;						\
	})

#define TUSB1211_VENDOR_ID_LO					0x00
#define TUSB1211_VENDOR_ID_HI					0x01
#define TUSB1211_PRODUCT_ID_LO					0x02
#define TUSB1211_PRODUCT_ID_HI					0x03
#define TUSB1211_FUNC_CTRL						0x04
#define TUSB1211_FUNC_CTRL_SET					0x05
#define TUSB1211_FUNC_CTRL_CLR					0x06
#define TUSB1211_IFC_CTRL						0x07
#define TUSB1211_IFC_CTRL_SET					0x08
#define TUSB1211_IFC_CTRL_CLR					0x09
#define TUSB1211_OTG_CTRL						0x0A
#define TUSB1211_OTG_CTRL_SET					0x0B
#define TUSB1211_OTG_CTRL_CLR					0x0C
#define TUSB1211_USB_INT_EN_RISE				0x0D
#define TUSB1211_USB_INT_EN_RISE_SET			0x0E
#define TUSB1211_USB_INT_EN_RISE_CLR			0x0F
#define TUSB1211_USB_INT_EN_FALL				0x10
#define TUSB1211_USB_INT_EN_FALL_SET			0x11
#define TUSB1211_USB_INT_EN_FALL_CLR			0x12
#define TUSB1211_USB_INT_STS					0x13
#define TUSB1211_USB_INT_LATCH					0x14
#define TUSB1211_DEBUG							0x15
#define TUSB1211_SCRATCH_REG					0x16
#define TUSB1211_SCRATCH_REG_SET				0x17
#define TUSB1211_SCRATCH_REG_CLR				0x18
#define TUSB1211_ACCESS_EXT_REG_SET				0x2F

#define TUSB1211_VENDOR_SPECIFIC1				0x80
#define TUSB1211_VENDOR_SPECIFIC1_SET			0x81
#define TUSB1211_VENDOR_SPECIFIC1_CLR			0x82
#define TUSB1211_POWER_CONTROL					0x3D
#define TUSB1211_POWER_CONTROL_SET				0x3E
#define TUSB1211_POWER_CONTROL_CLR				0x3F

#define TUSB1211_VENDOR_SPECIFIC2				0x80
#define TUSB1211_VENDOR_SPECIFIC2_SET			0x81
#define TUSB1211_VENDOR_SPECIFIC2_CLR			0x82
#define TUSB1211_VENDOR_SPECIFIC2_STS			0x83
#define TUSB1211_VENDOR_SPECIFIC2_LATCH			0x84
#define TUSB1211_VENDOR_SPECIFIC3				0x85
#define TUSB1211_VENDOR_SPECIFIC3_SET			0x86
#define TUSB1211_VENDOR_SPECIFIC3_CLR			0x87
#define TUSB1211_VENDOR_SPECIFIC4				0x88
#define TUSB1211_VENDOR_SPECIFIC4_SET			0x89
#define TUSB1211_VENDOR_SPECIFIC4_CLR			0x8A
#define TUSB1211_VENDOR_SPECIFIC5				0x8B
#define TUSB1211_VENDOR_SPECIFIC5_SET			0x8C
#define TUSB1211_VENDOR_SPECIFIC5_CLR			0x8D
#define TUSB1211_VENDOR_SPECIFIC6				0x8E
#define TUSB1211_VENDOR_SPECIFIC6_SET			0x8F
#define TUSB1211_VENDOR_SPECIFIC6_CLR			0x90

#define VS1_DATAPOLARITY						(1 << 6)
#define VS1_ZHSDRV(v)					((v & 0x3) << 5)
#define VS1_IHSTX(v)						 ((v & 0x7))

#define VS2STS_VBUS_MNTR_STS					(1 << 7)
#define VS2STS_REG3V3IN_MNTR_STS				(1 << 6)
#define VS2STS_SVLDCONWKB_WDOG_STS				(1 << 5)
#define VS2STS_ID_FLOAT_STS						(1 << 4)
#define VS2STS_ID_RARBRC_STS(v)					((v & 0x3) << 2)
#define VS2STS_BVALID_STS						(1 << 0)

#define VS3_CHGD_IDP_SRC_EN						(1 << 6)
#define VS3_IDPULLUP_WK_EN						(1 << 5)
#define VS3_SW_USB_DET							(1 << 4)
#define VS3_DATA_CONTACT_DET_EN					(1 << 3)
#define VS3_REG3V3_VSEL(v)					   (v & 0x7)

#define VS4_ACA_DET_EN							(1 << 6)
#define VS4_RABUSIN_EN							(1 << 5)
#define VS4_R1KSERIES							(1 << 4)
#define VS4_PSW_OSOD							(1 << 3)
#define VS4_PSW_CMOS							(1 << 2)
#define VS4_CHGD_SERX_DP						(1 << 1)
#define VS4_CHGD_SERX_DM						(1 << 0)

#define VS5_AUTORESUME_WDOG_EN					(1 << 6)
#define VS5_ID_FLOAT_EN							(1 << 5)
#define VS5_ID_RES_EN							(1 << 4)
#define VS5_SVLDCONWKB_WDOG_EN					(1 << 3)
#define VS5_VBUS_MNTR_RISE_EN					(1 << 2)
#define VS5_VBUS_MNTR_FALL_EN					(1 << 1)
#define VS5_REG3V3IN_MNTR_EN					(1 << 0)

#define DEBUG_LINESTATE                       (0x3 << 0)

#define OTGCTRL_USEEXTVBUS_INDICATOR			(1 << 7)
#define OTGCTRL_DRVVBUSEXTERNAL					(1 << 6)
#define OTGCTRL_DRVVBUS							(1 << 5)
#define OTGCTRL_CHRGVBUS						(1 << 4)
#define OTGCTRL_DISCHRGVBUS						(1 << 3)
#define OTGCTRL_DMPULLDOWN						(1 << 2)
#define OTGCTRL_DPPULLDOWN						(1 << 1)
#define OTGCTRL_IDPULLUP						(1 << 0)

#define FUNCCTRL_SUSPENDM						(1 << 6)
#define FUNCCTRL_RESET							(1 << 5)
#define FUNCCTRL_OPMODE(v)				((v & 0x3) << 3)
#define FUNCCTRL_TERMSELECT						(1 << 2)
#define FUNCCTRL_XCVRSELECT(v)					(v & 0x3)

#define PWCTRL_HWDETECT							(1 << 7)
#define PWCTRL_DP_VSRC_EN						(1 << 6)
#define PWCTRL_VDAT_DET							(1 << 5)
#define PWCTRL_DP_WKPU_EN						(1 << 4)
#define PWCTRL_BVALID_FALL						(1 << 3)
#define PWCTRL_BVALID_RISE						(1 << 2)
#define PWCTRL_DET_COMP							(1 << 1)
#define PWCTRL_SW_CONTROL						(1 << 0)


#define PMIC_TLP1ESBS0I1VNNBASE		0X6B
#define PMIC_I2COVRDADDR			0x59
#define PMIC_I2COVROFFSET			0x5A
#define PMIC_USBPHYCTRL				0x30
#define PMIC_I2COVRWRDATA			0x5B
#define PMIC_I2COVRCTRL				0x58
#define PMIC_I2COVRCTL_I2CWR		0x01

#define USBPHYCTRL_D0			(1 << 0)
#define PMIC_USBIDCTRL				0x19
#define USBIDCTRL_ACA_DETEN_D1	(1 << 1)
#define USBIDCTRL_USB_IDEN_D0	(1 << 0)
#define PMIC_USBIDSTS				0x1A
#define USBIDSTS_ID_GND			(1 << 0)
#define USBIDSTS_ID_RARBRC_STS(v)	((v & 0x3)  << 1)
#define USBIDSTS_ID_FLOAT_STS	(1 << 3)
#define PMIC_USBPHYCTRL_D0		(1 << 0)

#define VBUS_TIMEOUT	300
#define PCI_DEVICE_ID_DWC 0x119E
#define PCI_DEVICE_ID_DWC_VLV 0x0F37

#endif /* __DWC_OTG_H__ */
