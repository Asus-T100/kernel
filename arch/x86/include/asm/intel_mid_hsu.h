#ifndef __INTEL_MID_HSU_H__
#define __INTEL_MID_HSU_H__

enum {
	HSU_UART,
	HSU_DMA,
};

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
	void (*wake_peer)(struct device *tty);
};

extern struct mfld_hsu_info *platform_hsu_info;
extern int platform_hsu_num;
extern unsigned char hsu_dma_enable;
extern int hsu_rx_wa;
void intel_mid_hsu_suspend(int port);
void intel_mid_hsu_set_rts(int port, int set);
void intel_mid_hsu_resume(int port);
void intel_mid_hsu_switch(int port);
int intel_mid_hsu_init(int port, struct device *dev, irq_handler_t wake_isr);
void intel_mid_hsu_port_map(int *logic_idx, int *share_idx);
void intel_mid_hsu_wake_peer(int port);
struct device *intel_mid_hsu_set_wake_peer(int port,
	void (*wake_peer)(struct device *));

#endif

