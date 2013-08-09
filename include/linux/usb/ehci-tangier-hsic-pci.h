#ifndef EHCI_TANGIER_HSIC_PCI_h
#define EHCI_TANGIER_HSIC_PCI_h

#include <linux/notifier.h>
#include <linux/usb.h>

#define HSIC_AUX_GPIO_NAME       "usb_hsic_aux1"
#define HSIC_WAKEUP_GPIO_NAME    "usb_hsic_aux2"
#define HSIC_HUB_RESET_TIME   10
#define HSIC_ENABLE_SIZE      2
#define HSIC_DURATION_SIZE    7
#define PM_BASE		0xff00b000
#define PM_STS		0x00
#define PM_CMD		0x04

#define PM_SS0		0x30
#define PM_SS1		0x34
#define PM_SS2		0x38
#define PM_SS3		0x3C

#define PM_SSC0		0x20
#define PM_SSC1		0x24
#define PM_SSC2		0x28
#define PM_SSC3		0x2C
#define PMU_HW_PEN0 0x108
#define PMU_HW_PEN1 0x10C

/* Port Inactivity Duratoin is default value for L2 suspend */
#define HSIC_PORT_INACTIVITYDURATION              500
/* This is the default value for L2 autosuspend enable */
#define HSIC_AUTOSUSPEND                          0
#define HSIC_BUS_INACTIVITYDURATION               500
#define HSIC_REMOTEWAKEUP                         1

struct hsic_tangier_priv {
	struct delayed_work  hsic_aux;
	wait_queue_head_t    aux_wq;
	struct mutex         hsic_mutex;
	unsigned             hsic_mutex_init:1;
	unsigned             aux_wq_init:1;
	unsigned             hsic_aux_irq_enable:1;
	unsigned             hsic_wakeup_irq_enable:1;
	unsigned             hsic_aux_finish:1;
	unsigned             hsic_enable_created:1;
	unsigned             hsic_lock_init:1;
	unsigned             hsic_stopped:1;

	unsigned             remoteWakeup_enable;
	unsigned             autosuspend_enable;
	unsigned             aux_gpio;
	unsigned             wakeup_gpio;
	unsigned             port_inactivityDuration;
	unsigned             bus_inactivityDuration;
	spinlock_t           hsic_lock;
	/* Root hub device */
	struct usb_device           *rh_dev;
	struct usb_device           *modem_dev;
	struct workqueue_struct     *work_queue;
	struct work_struct          wakeup_work;
};

enum {
	PROBE,
	REMOVE
};

#define HSIC_DPHY_D3_STATE_MASK		0xE00
#define PM_REGISTER_LENGHT			0x200
#define PM_SSC0_HSIC_D3_MODE		(0x3 << 16)
#define PM_STS_DONE					(1 << 0x8)
#define PMU_HW_PEN0_HSIC_DPHY_MASK	0xE00
#endif
