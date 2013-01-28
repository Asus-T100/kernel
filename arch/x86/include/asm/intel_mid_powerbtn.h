#ifndef __INTEL_MID_POWERBTN_H__
#define __INTEL_MID_POWERBTN_H__

struct intel_msic_power_btn_platform_data {
	unsigned long pbstat;
	u16 pb_level;
	u16 irq_lvl1_mask;
	u16 pb_irq;
	u16 pb_irq_mask;
	int (*irq_ack)(struct intel_msic_power_btn_platform_data *);
};

#define MSIC_PB_LEN	1
#define MSIC_PWRBTNM	(1 << 0)

#define DRIVER_NAME	"msic_power_btn"

#endif
