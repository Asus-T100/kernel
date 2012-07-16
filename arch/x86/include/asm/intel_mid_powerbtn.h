#ifndef __INTEL_MID_POWERBTN_H__
#define __INTEL_MID_POWERBTN_H__

struct intel_msic_power_btn_platform_data {
	unsigned long pbstat;
	u8 pb_level;
	u8 irq_lvl1_mask;
	u8 pb_irq;
	u8 pb_irq_mask;
	int (*irq_ack)(void *);
};

int pb_irq_ack(void *dev_id);

#endif
