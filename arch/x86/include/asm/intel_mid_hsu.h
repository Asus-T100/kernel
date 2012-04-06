#ifndef __INTEL_MID_HSU_H__
#define __INTEL_MID_HSU_H__

#define MFLD_HSU_NUM	4
struct mfld_hsu_info {
	char *name;
	int id;
	int wake_gpio;
	int rx_gpio;
	int rx_alt;
	int tx_gpio;
	int tx_alt;
	int cts_gpio;
	int cts_alt;
	int rts_gpio;
	int rts_alt;
	struct device *dev;
	irq_handler_t wake_isr;
};

extern struct mfld_hsu_info *platform_hsu_info;
extern unsigned char hsu_dma_enable;
void intel_mid_hsu_suspend(int port);
void intel_mid_hsu_resume(int port);
void intel_mid_hsu_switch(int port);
int intel_mid_hsu_init(int port, struct device *dev, irq_handler_t wake_isr);


#endif

