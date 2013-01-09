#ifndef EHCI_TANGIER_HSIC_PCI_h
#define EHCI_TANGIER_HSIC_PCI_h
#include <linux/notifier.h>

#define HSIC_AUX_N	173
#define HSIC_HUB_RESET_TIME   10
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

struct hsic_tangier_priv {
	struct delayed_work  hsic_aux;
	struct delayed_work  hsic_enable;
};

enum {
	PROBE,
	REMOVE
};

#define MODEM_7160_STAGE_ONE_VID	0x8087
#define MODEM_7160_STAGE_ONE_PID	0x0716
#define MODEM_7160_STAGE_TWO_VID	0x1519
#define MODEM_7160_STAGE_TWO_PID	0x0443
#define MODEM_PWR_ON                   181
#define MODEM_RESET_BB_N               182
#define HSIC_DPHY_D3_STATE_MASK		0xE00
#define PM_REGISTER_LENGHT			0x200
#define PM_SSC0_HSIC_D3_MODE		(0x3 << 16)
#define PM_STS_DONE					(1 << 0x8)
#define PMU_HW_PEN0_HSIC_DPHY_MASK	0xE00
#endif
