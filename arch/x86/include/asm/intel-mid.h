/*
 * intel-mid.h: Intel MID specific setup code
 *
 * (C) Copyright 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_INTEL_MID_H
#define _ASM_X86_INTEL_MID_H

#include <linux/sfi.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

extern u32 board_id;
extern int intel_mid_pci_init(void);
extern int get_gpio_by_name(const char *name);
extern void *get_oem0_table(void);
extern void intel_delayed_device_register(void *dev,
			void (*delayed_callback)(void *dev_desc));
extern int sdhci_pci_request_regulators(void);
extern void install_irq_resource(struct platform_device *pdev, int irq);
extern void intel_scu_device_register(struct platform_device *pdev);
extern struct devs_id *get_device_id(u8 type, char *name);
extern int __init sfi_parse_mrtc(struct sfi_table_header *table);
extern int sfi_mrtc_num;
extern struct sfi_rtc_table_entry sfi_mrtc_array[];
extern int intel_mid_create_property(const struct attribute *attr);

/*
 * Here defines the array of devices platform data that IAFW would export
 * through SFI "DEVS" table, we use name and type to match the device and
 * its platform data.
 */
struct devs_id {
	char name[SFI_NAME_LEN + 1];
	u8 type;
	u8 delay;
	void *(*get_platform_data)(void *info);
	void (*device_handler)(struct sfi_device_table_entry *pentry,
				struct devs_id *dev);
	/* Custom handler for devices */
	u8 trash_itp;/* true if this driver uses pin muxed with XDB connector */
};

#define SD_NAME_SIZE 16
/**
 * struct sd_board_info - template for device creation
 * @name: Initializes sdio_device.name; identifies the driver.
 * @bus_num: board-specific identifier for a given SDIO controller.
 * @board_ref_clock: Initializes sd_device.board_ref_clock;
 * @platform_data: Initializes sd_device.platform_data; the particular
 *      data stored there is driver-specific.
 *
 */
struct sd_board_info {
	char            name[SD_NAME_SIZE];
	int             bus_num;
	unsigned short  addr;
	u32             board_ref_clock;
	void            *platform_data;
};


/*
 * Medfield is the follow-up of Moorestown, it combines two chip solution into
 * one. Other than that it also added always-on and constant tsc and lapic
 * timers. Medfield is the platform name, and the chip name is called Penwell
 * we treat Medfield/Penwell as a variant of Moorestown. Penwell can be
 * identified via MSRs.
 */
enum intel_mid_cpu_type {
	INTEL_MID_CPU_CHIP_LINCROFT = 1,
	INTEL_MID_CPU_CHIP_PENWELL,
	INTEL_MID_CPU_CHIP_CLOVERVIEW,
	INTEL_MID_CPU_CHIP_TANGIER,
};

extern enum intel_mid_cpu_type __intel_mid_cpu_chip;

#ifdef CONFIG_X86_INTEL_MID

static inline enum intel_mid_cpu_type intel_mid_identify_cpu(void)
{
	return __intel_mid_cpu_chip;
}

#else /* !CONFIG_X86_INTEL_MID */

#define intel_mid_identify_cpu()    (0)

#endif /* !CONFIG_X86_INTEL_MID */

enum intel_mid_timer_options {
	INTEL_MID_TIMER_DEFAULT,
	INTEL_MID_TIMER_APBT_ONLY,
	INTEL_MID_TIMER_LAPIC_APBT,
};

enum {
	/* 0 is "unknown" so that you can write if (!mrst_platform_id()) */
	MRST_PLATFORM_LANFORD = 1,
	MRST_PLATFORM_SHCDK,
	MRST_PLATFORM_AAVA_SC,
};
extern int mrst_platform_id(void);

extern enum intel_mid_timer_options intel_mid_timer_options;

enum {
	MFLD_BOARD_UNKNOWN = 0,
	MFLD_BOARD_CDK     = 1,
	MFLD_BOARD_PR2     = 2,
};
extern int mfld_board_type(void);

/*
    MSIC Pin, Pin#, ID,     CDK     Aava     BKB      BKB      BKB   BKB  BKB
    --------  ----- ---   (Fab-B) FF-Proto Pr2Proto Pr2Volume PR2PnP PR3 PR3PnP
    GPIO0LV0  L13   BID_0    0       0        1        1        0     1    0
    GPIO0LV1  H12   BID_1    1       1        0        0        0     0    0
    GPIO0LV2  K13   BID_2    0       1        1        1        0     1    0
    GPIO0LV3  J11   BID_3    0       0        0        0        1     0    1
    GPIO0LV4  K14   FAB_ID0  1       x        1        0        0     1    1
    GPIO0LV5  J15   FAB_ID1  0       0        0        1        0     1    0
 */
enum {
	MFLD_BID_UNKNOWN = 0,
	MFLD_BID_CDK         = 0x12,
	MFLD_BID_AAVA        = 0x06,
	MFLD_BID_PR2_PROTO   = 0x15,
	MFLD_BID_PR2_PNP     = 0x08,
	MFLD_BID_PR2_VOLUME  = 0x25,
	MFLD_BID_PR3         = 0x35,
	MFLD_BID_PR3_PNP     = 0x18
};
extern u32 mfld_board_id(void);

/* Right now board id information is updated from sfi table print*/

enum {
	CTP_BID_UNKNOWN = 0,
	CTP_BID_VV	= 0x08,
	CTP_BID_PR0	= 0x04
};
extern u32 ctp_board_id(void);

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 */
#define PENWELL_FSB_FREQ_83SKU         83200
#define PENWELL_FSB_FREQ_100SKU        99840

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

extern unsigned char hsu_dma_enable;

extern struct console early_mrst_console;
extern void mrst_early_console_init(void);
extern struct console early_mrfld_console;
extern void mrfld_early_console_init(void);

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT
extern struct console early_hsu_console;
extern void hsu_early_console_init(void);
#endif

extern struct console early_pti_console;

extern void intel_scu_devices_create(int bus_id);
extern void intel_scu_devices_destroy(int bus_id);
extern void *cloverview_usb_otg_get_pdata(void);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void intel_mid_rtc_init(void);

extern int mrst_pmu_pci_set_power_state(struct pci_dev *pdev,
					pci_power_t state);

#ifdef CONFIG_X86_MDFLD
extern int pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state);
extern pci_power_t pmu_pci_choose_state(struct pci_dev *pdev);
#endif /* !CONFIG_X86_MDFLD */

#ifdef CONFIG_X86_MRST
extern int mrst_pmu_s0i3_entry(void);
extern void mrst_pmu_disable_msi(void);
extern u32 mrst_pmu_msi_is_disabled(void);
extern void mrst_pmu_enable_msi(void);
extern void mrst_reserve_memory(void);
#else
static inline void mrst_reserve_memory(void) { }
#endif	/* !CONFIG_X86_MRST */

#include <linux/cpuidle.h>
extern int mrst_s0i3(struct cpuidle_device *dev, struct cpuidle_state *state);
extern void mrst_s0i3_resume(void);
extern int mrst_pmu_invalid_cstates(void);
extern const char s0i3_trampoline_data[], s0i3_trampoline_data_end[];

extern int get_force_shutdown_occured(void);

extern const struct atomisp_platform_data *intel_get_v4l2_subdev_table(void);

extern void (*saved_shutdown)(void);

#ifdef CONFIG_X86_MRFLD
enum intel_mrfl_sim_type {
	INTEL_MRFL_CPU_SIMULATION_NONE = 0,
	INTEL_MRFL_CPU_SIMULATION_VP,
	INTEL_MRFL_CPU_SIMULATION_SLE,
};

extern enum intel_mrfl_sim_type __intel_mrfl_sim_platform;

static inline enum intel_mrfl_sim_type intel_mrfl_identify_sim(void)
{
	return __intel_mrfl_sim_platform;
}

#else /* !CONFIG_X86_MRFLD */
#define intel_mrfl_identify_sim()	(0)
#endif /* !CONFIG_X86_MRFLD */

#endif /* _ASM_X86_INTEL_MID_H */
