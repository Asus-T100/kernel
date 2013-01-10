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

/*
 * Access to message bus through three registers
 * in CUNIT(0:0:0) PCI configuration space.
 * MSGBUS_CTRL_REG(0xD0):
 *   31:24	= message bus opcode
 *   23:16	= message bus port
 *   15:8	= message bus address, low 8 bits.
 *   7:4	= message bus byte enables
 * MSGBUS_CTRL_EXT_REG(0xD8):
 *   31:8	= message bus address, high 24 bits.
 * MSGBUS_DATA_REG(0xD4):
 *   hold the data for write or read
 */
#define PCI_ROOT_MSGBUS_CTRL_REG	0xD0
#define PCI_ROOT_MSGBUS_DATA_REG	0xD4
#define PCI_ROOT_MSGBUS_CTRL_EXT_REG	0xD8
#define PCI_ROOT_MSGBUS_READ		0x10
#define PCI_ROOT_MSGBUS_WRITE		0x11
#define PCI_ROOT_MSGBUS_DWORD_ENABLE	0xf0

extern u32 board_id;
extern struct sfi_soft_platform_id spid;
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
extern u32 intel_mid_msgbus_read32_raw(u32 cmd);
extern void intel_mid_msgbus_write32_raw(u32 cmd, u32 data);
extern u32 intel_mid_msgbus_read32(u8 port, u32 addr);
extern void intel_mid_msgbus_write32(u8 port, u32 addr, u32 data);
extern u32 intel_mid_soc_stepping(void);

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

/**
 * struct intel_mid_ops - Interface between intel-mid & sub archs
 * @arch_setup:		arch_setup function to re-initialize platform
 *			structures (x86_init, x86_platform_init)
 *
 * This structure can be extended if any new interface is required
 * between intel-mid & its sub arch files.
 */

struct intel_mid_ops {
	void (*arch_setup)(void);
};

/* Helper API's for INTEL_MID_OPS_INIT */
#define DECLARE_INTEL_MID_OPS_INIT(cpuname, cpuid) [cpuid] = \
		get_##cpuname##_ops,

/* Maximum number of CPU ops */
#define MAX_CPU_OPS(a) (sizeof(a)/sizeof(void *))

/*
 * For every new cpu addition, a weak get_<cpuname>_ops() function needs be
 * declared in arch/x86/platform/intel_mid/intel_mid_weak_decls.h.
 */
#define INTEL_MID_OPS_INIT {\
	DECLARE_INTEL_MID_OPS_INIT(lincroft, INTEL_MID_CPU_CHIP_LINCROFT) \
	DECLARE_INTEL_MID_OPS_INIT(penwell, INTEL_MID_CPU_CHIP_PENWELL) \
	DECLARE_INTEL_MID_OPS_INIT(cloverview, INTEL_MID_CPU_CHIP_CLOVERVIEW) \
	DECLARE_INTEL_MID_OPS_INIT(tangier, INTEL_MID_CPU_CHIP_TANGIER) \
};


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
    MSIC Pin, Pin#, ID,     CDK     BKB      BKB      BKB    BKB   BKB
    --------  ----- ---   (Fab-B) Pr2Proto Pr2Volume PR2PnP  PR3  PR3PnP
    GPIO0LV0  L13   BID_0    0       1        1        0      1     0
    GPIO0LV1  H12   BID_1    1       0        0        0      0     0
    GPIO0LV2  K13   BID_2    0       1        1        0      1     0
    GPIO0LV3  J11   BID_3    0       0        0        1      0     1
    GPIO0LV4  K14   FAB_ID0  1       1        0        0      1     1
    GPIO0LV5  J15   FAB_ID1  0       0        1        0      1     0


    MSIC Pin, Pin#, ID,     Aava     RR    RR    RR    Joki
    --------  ----- ---   FF-Proto  DV10  DV20  DV21   EV20
    GPIO0LV0  L13   BID_0    0       1     1     1      0
    GPIO0LV1  H12   BID_1    1       0     0     0      1
    GPIO0LV2  K13   BID_2    1       1     1     1      1
    GPIO0LV3  J11   BID_3    0       0     0     0      0
    GPIO0LV4  K14   FAB_ID0  x       0     1     0      1
    GPIO0LV5  J15   FAB_ID1  0       1     0     0      0


 */

enum {
	MFLD_BID_UNKNOWN     = 0,
	MFLD_BID_CDK         = 0x12,
	MFLD_BID_AAVA        = 0x06,
	MFLD_BID_SALITPA_EV1 = 0x0A,
	MFLD_BID_PR2_PROTO   = 0x15,
	MFLD_BID_PR2_PNP     = 0x08,
	MFLD_BID_PR2_VOLUME  = 0x25,
	MFLD_BID_PR3         = 0x35,
	MFLD_BID_PR3_PNP     = 0x18,
	MFLD_BID_RR_DV10     = 0x25,
	MFLD_BID_RR_DV20     = 0x15,
	MFLD_BID_RR_DV21     = 0x05,
	MFLD_BID_LEX   = 0xe0000025,
	CLVT_BID_VV          = 0x08,
	CLVT_BID_PR0         = 0x04
};
extern u32 mfld_board_id(void);

/* Right now board id information is updated from sfi table print*/

enum {
	CTP_BID_UNKNOWN = 0,
	CTP_BID_VV	= 0x08,
	CTP_BID_PR0	= 0x04
};
extern u32 get_board_id(void);

#define spid_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr   = {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show   = _name##_show,			\
}

/* Customer_ID table */
enum {
	CUSTOMER_INTEL,
	CUSTOMER_INTEL_RSVD1,
	CUSTOMER_INTEL_RSVD2,
	CUSTOMER_INTEL_RSVD3,
	CUSTOMER_INTEL_RSVD4,
	CUSTOMER_INTEL_RSVD5,
	CUSTOMER_RSVD,
	CUSTOMER_UNKNOWN = 0xFFFF
};

/* Vendor_ID table */
enum {
	VENDOR_INTEL,
	VENDOR_INTEL_RSVD1,
	VENDOR_INTEL_RSVD2,
	VENDOR_RSVD,
	VENDOR_UNKNOWN = 0xFFFF
};

/* Manufacturer_ID table for Vendor_ID == VENDOR_INTEL */
enum {
	MANUFACTURER_FAB1,
	MANUFACTURER_FAB2,
	MANUFACTURER_FAB3,
	MANUFACTURER_FAB4,
	MANUFACTURER_FAB5,
	MANUFACTURER_FAB6,
	MANUFACTURER_FAB7,
	MANUFACTURER_RSVD,
	MANUFACTURER_UNKNOWN = 0xFFFF
};

/* Platform_Family_ID table for Vendor_ID == VENDOR_INTEL */
enum {
	INTEL_MFLD_PHONE  = 0x0000,
	INTEL_MFLD_TABLET = 0x0001,
	INTEL_CLVTP_PHONE = 0x0002,
	INTEL_CLVT_TABLET = 0x0003,
	INTEL_MRFL_PHONE  = 0x0004,
	INTEL_MRFL_TABLET = 0x0005,
	INTEL_PLATFORM_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MFLD_PHONE */
enum {
	INTEL_MFLD_PHONE_BB15_PRO = 0x0000,
	INTEL_MFLD_PHONE_BB15_ENG = 0x8000,
	INTEL_MFLD_PHONE_BB20_PRO = 0x0001,
	INTEL_MFLD_PHONE_BB20_ENG = 0x8001,
	INTEL_MFLD_PHONE_OR_PRO   = 0x0002,
	INTEL_MFLD_PHONE_OR_ENG   = 0x8002,
	INTEL_MFLD_PHONE_AT_PRO   = 0x0003,
	INTEL_MFLD_PHONE_AT_ENG   = 0x8003,
	INTEL_MFLD_PHONE_LEX_PRO  = 0x0004,
	INTEL_MFLD_PHONE_LEX_ENG  = 0x8004,
	INTEL_MFLD_PHONE_UNKNOWN  = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MFLD_TABLET */
enum {
	INTEL_MFLD_TABLET_RR_PRO  = 0x0000,
	INTEL_MFLD_TABLET_RR_ENG  = 0x8000,
	INTEL_MFLD_TABLET_FM_PRO  = 0x0001,
	INTEL_MFLD_TABLET_FM_ENG  = 0x8001,
	INTEL_MFLD_TABLET_FVA_PRO = 0x0002,
	INTEL_MFLD_TABLET_FVA_ENG = 0x8002,
	INTEL_MFLD_TABLET_SLP_PRO = 0x0003,
	INTEL_MFLD_TABLET_SLP_ENG = 0x8003,
	INTEL_MFLD_TABLET_YKB_PRO = 0x0004,
	INTEL_MFLD_TABLET_YKB_ENG = 0x8004,
	INTEL_MFLD_TABLET_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_CLVTP_PHONE */
enum {
	INTEL_CLVTP_PHONE_RHB_PRO = 0x0000,
	INTEL_CLVTP_PHONE_RHB_ENG = 0x8000,
	INTEL_CLVTP_PHONE_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_CLVT_TABLET */
enum {
	INTEL_CLVT_TABLET_TBD_PRO = 0x0000,
	INTEL_CLVT_TABLET_TBD_ENG = 0x8000,
	INTEL_CLVT_TABLET_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MRFL_PHONE */
enum {
	INTEL_MRFL_PHONE_SB_PRO = 0x0000,
	INTEL_MRFL_PHONE_SB_ENG = 0x8000,
	INTEL_MRFL_PHONE_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MRFL_TABLET */
enum {
	INTEL_MRFL_TABLET_TBD_PRO = 0x0000,
	INTEL_MRFL_TABLET_TBD_ENG = 0x8000,
	INTEL_MRFL_TABLET_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_PHONE_BB15 */
enum {
	MFLD_PHONE_BB15_PR20, /* CRAK C0 */
	MFLD_PHONE_BB15_PR31, /* CRAK D0 */
	MFLD_PHONE_BB15_PR32, /* CRAK D0 */
	MFLD_PHONE_BB15_PR33, /* CRAK D1 - 1.6GHz */
	MFLD_PHONE_BB15_PR34, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2 */
	MFLD_PHONE_BB15_PR35, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2 */
	MFLD_PHONE_BB15_PR36, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2, MSIC C2 */
	MFLD_PHONE_BB15_PR40, /* CRAK D1 - 2.0GHz */
	MFLD_PHONE_BB15_PR2A,
	MFLD_PHONE_BB15_PR3A,
	MFLD_PHONE_BB15_PR3B,
	MFLD_PHONE_BB15_4MVV,
	MFLD_PHONE_BB15_4MSV,
	MFLD_PHONE_BB15_ICDK,
	MFLD_PHONE_BB15_4MVV3,
	MFLD_PHONE_BB15_RSVD,
	MFLD_PHONE_BB15_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_PHONE_BB20 */
enum {
	MFLD_PHONE_BB20_TBD,
	MFLD_PHONE_BB20_RSVD,
	MFLD_PHONE_BB20_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_PHONE_OR */
enum {
	MFLD_PHONE_OR_NHDV1,    /* CRAK D0 - 1.6G */
	MFLD_PHONE_OR_NHDV2,    /* CRAK D1 - 1.6G */
	MFLD_PHONE_OR_NHDV3,    /* CRAK D1 - 1.6G */
	MFLD_PHONE_OR_NHDV31R,  /* CRAK D1 - 1.6G */
	MFLD_PHONE_OR_NHDV31A,  /* CAAK D1 - 1.6G */
	MFLD_PHONE_OR_NHDV30F,  /* CRAK D1 - 2.0G */
	MFLD_PHONE_OR_NHDV31A1, /* CAAK D1 - 2.0G */
	MFLD_PHONE_OR_NHDV30D,  /* CRAK D2 - 1.6G */
	MFLD_PHONE_OR_NHDV30G,  /* CRAK D2 - 2.0G */
	MFLD_PHONE_OR_NHDV31A2, /* CAAK D2 - 1.6G */
	MFLD_PHONE_OR_NHDV31A3, /* CAAK D2 - 2.0G */
	MFLD_PHONE_OR_NHDV30E,  /* CRAK D1, Samsung eMMC for part quals */
	MFLD_PHONE_OR_RSVD,
	MFLD_PHONE_OR_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_PHONE_AT */
enum {
	MFLD_PHONE_AT_LA, /* CAAK D1 */
	MFLD_PHONE_AT_LA_RSVD,
	MFLD_PHONE_AT_LA_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_PHONE_LEX */
enum {
	MFLD_PHONE_LEX_PR11, /* RYS/PNW 1GHz CREK D1 */
	MFLD_PHONE_LEX_PR1M, /* RYS/PNW 1GHz CREK D1 */
	MFLD_PHONE_LEX_PR21, /* RYS/PNW 1GHz CSEK D1 */
	MFLD_PHONE_LEX_PR2M, /* BND/PNW 1GHz CSEK D1 */
	MFLD_PHONE_LEX_DV1,  /* BND/PNW 1.2GHz CSEK D1 */
	MFLD_PHONE_LEX_RSVD,
	MFLD_PHONE_LEX_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_TABLET_RR */
enum {
	MFLD_TABLET_RR_DV10, /* CRAK D0 */
	MFLD_TABLET_RR_DV15, /* CRAK D0/D1 */
	MFLD_TABLET_RR_DV20, /* CRAK D1 */
	MFLD_TABLET_RR_DV21, /* CRAK D1 */
	MFLD_TABLET_RR_RSVD,
	MFLD_TABLET_RR_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_TABLET_FM */
enum {
	MFLD_TABLET_FM_EV20, /* CRAK D0 */
	MFLD_TABLET_FM_DV10, /* CRAK D1 */
	MFLD_TABLET_FM_RSVD,
	MFLD_TABLET_FM_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_TABLET_FVA */
enum {
	MFLD_TABLET_FVA_EV10P, /* CRAK Dx */
	MFLD_TABLET_FVA_EV10,  /* CRAK Dx */
	MFLD_TABLET_FVA_EV20,  /* CRAK Dx */
	MFLD_TABLET_FVA_DV10,  /* CRAK Dx */
	MFLD_TABLET_FVA_RSVD,
	MFLD_TABLET_FVA_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_TABLET_SLP */
enum {
	MFLD_TABLET_SLP_EV05, /* CRAK Dx */
	MFLD_TABLET_SLP_EV10, /* CRAK Dx */
	MFLD_TABLET_SLP_EV20, /* CRAK Dx */
	MFLD_TABLET_SLP_DV10, /* CRAK Dx */
	MFLD_TABLET_SLP_RSVD,
	MFLD_TABLET_SLP_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLD_TABLET_YKB */
enum {
	MFLD_TABLET_YKB_DV10, /* CRAK Dx */
	MFLD_TABLET_YKB_RSVD,
	MFLD_TABLET_YKB_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_CLVTP_PHONE_RHB */
enum {
	CLVTP_PHONE_RHB_CCVV0,  /* Clover City VV0 FAB A CLV/CLV+ A0*/
	CLVTP_PHONE_RHB_CCVV1,  /* Clover City VV1 FAB B CLV+ A0*/
	CLVTP_PHONE_RHB_CCVV2,  /* Clover City VV2 FAB C CLV+ A0*/
	CLVTP_PHONE_RHB_CLEV,   /* Clover Lake CRB EV */
	CLVTP_PHONE_RHB_PR01,   /* RHB PR0.1 CLV A0 C-CLASS */
	CLVTP_PHONE_RHB_PR02,   /* RHB PR0.2 CLV A0 C-CLASS */
	CLVTP_PHONE_RHB_PR10PM, /* CLV+ A0 */
	CLVTP_PHONE_RHB_CCVV1P, /* Clover City Pre-VV1 Fab B CLV+ A0 */
	CLVTP_PHONE_RHB_PR10P,  /* RHB Pre-PR1.0 CLV A0 C- CLASS */
	CLVTP_PHONE_RHB_PR10M,  /* RHB Macro PR1.0 CLV+ A0 */
	CLVTP_PHONE_RHB_PR10,   /* RHB PR1.0 CLV+ A0 C-CLASS */
	CLVTP_PHONE_RHB_PR15M,  /* RHB Macro PR1.5 CLV+ A0 */
	CLVTP_PHONE_RHB_PR15,   /* RHB PR1.5 CLV+ A0 C-CLASS */
	CLVTP_PHONE_RHB_PR20M,  /* RHB Macro PR2.0 CLV+ B0 */
	CLVTP_PHONE_RHB_PR20,   /* RHB PR2.0 CLV+ B0 C-CLASS */
	CLVTP_PHONE_RHB_PR30M,  /* RHB Macro PR3.0 CLV+ B0 */
	CLVTP_PHONE_RHB_CCVV3,  /* Clover City VV3 FAB D CLV+ A0 */
	CLVTP_PHONE_RHB_PR30,   /* RHB PR3.0 CLV+ B0 C-CLASS */
	CLVTP_PHONE_RHB_DV1,    /* RHB Dv1 */
	CLVTP_PHONE_RHB_PR20A,  /* CLV+ B0 C-Class-touch panel sensor GFF */
	CLVTP_PHONE_RHB_CCVV2VB, /* Clover City VV2-Victoria Bay FAB B CLV B0 */
	CLVTP_PHONE_RHB_PR19M,  /* Macro PR1.9 CLV+ B0 */
	CLVTP_PHONE_RHB_PR199M, /* Macro PR1.99 CLV+ B0 */
	CLVTP_PHONE_RHB_VBDV1,  /* VictoriaBay DV1 CLV+ B0
					Renasas Modem BRCM CWS */
	CLVTP_PHONE_RHB_PR20B,  /* CLV+ B0 C-Class-touch panel sensor
					GFF-LPDDR2 */
	CLVTP_PHONE_RHB_RSVD,
	CLVTP_PHONE_RHB_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_CLVT_TABLET_TBD */
enum {
	CLVT_TABLET_TBD_CLEVA, /* Clover Lake EV - CRB - FAB A */
	CLVT_TABLET_TBD_CLEVB, /* Clover Lake EV - CRB - FAB B */
	CLVT_TABLET_TBD_CLEVC, /* Clover Lake EV - CRB - FAB C */
	CLVT_TABLET_TBD_CLEVD, /* Clover Lake EV - CRB - FAB D */
	CLVT_TABLET_TBD_VV2SS, /* Clover City VV2-Samtab-FAB B CLV+ B0 */
	CLVT_TABLET_TBD_RSVD,
	CLVT_TABLET_TBD_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MRFL_PHONE_SB */
enum {
	MRFL_PHONE_SR_VVA,  /* SilverRidge VV FAB A */
	MRFL_PHONE_SR_VVB,  /* SilverRidge VV FAB B */
	MRFL_PHONE_SR_VVC,  /* SilverRidge VV FAB C */
	MRFL_PHONE_SR_VVD,  /* SilverRidge VV FAB D */
	MRFL_PHONE_SR_SVA,  /* SilverRidge SV FAB A */
	MRFL_PHONE_SR_SVB,  /* SilverRidge SV FAB B */
	MRFL_PHONE_SR_SVC,  /* SilverRidge SV FAB C */
	MRFL_PHONE_SR_SVD,  /* SilverRidge SV FAB D */
	MRFL_PHONE_SB_PR0M, /* Salt Bay PR0-Macro (A0) */
	MRFL_PHONE_SB_PR0,  /* Salt Bay PR0-FF (A0) */
	MRFL_PHONE_SB_PR1M, /* Salt Bay PR1-Macro (A0) */
	MRFL_PHONE_SB_PR1,  /* Salt Bay PR1-FF (A0) */
	MRFL_PHONE_SB_PR2M, /* Salt Bay PR2-Macro (B0) */
	MRFL_PHONE_SB_PR2,  /* Salt Bay PR2-FF (B0) */
	MRFL_PHONE_SB_PR3M, /* Salt Bay PR3-Macro (B0) */
	MRFL_PHONE_SB_PR3,  /* Salt Bay PR3-FF (B0) */
	MRFL_PHONE_TBD_RSVD,
	MRFL_PHONE_TBD_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MRFL_TABLET_TBD */
enum {
	MRFL_TABLET_TBD_TBD,
	MRFL_TABLET_TBD_RSVD,
	MRFL_TABLET_TBD_UNKNOWN = 0xFFFF
};

/* Macros for SPID based checks */

#define SPID_CUSTOMER_ID(customer) ( \
	(spid.customer_id == CUSTOMER_##customer))
#define SPID_VENDOR_ID(vendor) ( \
	(spid.vendor_id == VENDOR_##vendor))
#define SPID_PLATFORM_ID(vendor, platform, devtype) ( \
	(spid.platform_family_id == vendor##_##platform##_##devtype))
#define SPID_PRODUCT_ID(vendor, platform, devtype, product, type) (\
	(spid.product_line_id == \
	vendor##_##platform##_##devtype##_##product##_##type))
#define SPID_HARDWARE_ID(platform, devtype, product, hardware) (\
	(spid.hardware_id == platform##_##devtype##_##product##_##hardware))

#define INTEL_MID_BOARDV1(devtype, platform) ( \
	SPID_CUSTOMER_ID(INTEL) && \
	SPID_VENDOR_ID(INTEL) && \
	SPID_PLATFORM_ID(INTEL, platform, devtype))

#define INTEL_MID_BOARDV2(devtype, platform, product, type) ( \
	INTEL_MID_BOARDV1(devtype, platform) && \
	SPID_PRODUCT_ID(INTEL, platform, devtype, product, type))

#define INTEL_MID_BOARDV3(devtype, platform, product, type, hardware) ( \
	INTEL_MID_BOARDV2(devtype, platform, product, type) && \
	SPID_HARDWARE_ID(platform, devtype, product, hardware))



/* INTEL_MID_BOARD - Returns true if arugments matches SPID contents
 * @ level:	1, 2, 3
		- 1 for verifying platform_id,
		- 2 for verifying platform_type & product_id,
		- 3 for verifying platform_type, product_id & hardware_id.
 * @ devtype:	PHONE or TABLET
 * @ arg3:	platform_type - MFLD,CLVTP,CLVT,MRFL.
 * @ arg4:	product ID - product id supported by
		platform_type passed in arg3.
 * @ arg5:	PRO or ENG.
 * @ arg6:	hardware_id -Hardware IDs supported by above
		platform_type & product_id.
 *
 * Example:	INTEL_MID_BOARD(1,PHONE,MFLD)
 *		INTEL_MID_BOARD(2,PHONE,MFLD,BB15,PRO)
 *		INTEL_MID_BOARD(3,PHONE,MFLD,BB15,PRO,PR20),
 *
 */
#define INTEL_MID_BOARD(level, devtype, ...) ( \
	INTEL_MID_BOARDV##level(devtype, __VA_ARGS__))

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 * CLVP A0 has 100MHz FSB and CLVP B0 has 133MHz FSB.
 */
#define FSB_FREQ_83SKU		83200
#define FSB_FREQ_100SKU		99840
#define FSB_FREQ_133SKU		133000

/* Bus Select SoC Fuse value */
#define BSEL_SOC_FUSE_MASK	0x7
#define BSEL_SOC_FUSE_001	0x1 /* FSB 133MHz */
#define BSEL_SOC_FUSE_101	0x5 /* FSB 100MHz */
#define BSEL_SOC_FUSE_111	0x7 /* FSB 83MHz */

/* TNG: translate MSR values directly, scaled to hertz */
#define TNG_IDI_FREQ_83SKU             83300
#define TNG_IDI_FREQ_100SKU            100000
#define TNG_IDI_FREQ_133SKU            133300
#define TNG_IDI_FREQ_167SKU            167000
#define TNG_IDI_FREQ_200SKU            200000
#define TNG_IDI_FREQ_267SKU            267000
#define TNG_IDI_FREQ_333SKU            333000
#define TNG_IDI_FREQ_400SKU            400000

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
extern void intel_psh_devices_create(int bus_id);
extern void intel_psh_devices_destroy(int bus_id);
extern void *intel_mid_otg_get_pdata(struct pci_dev *pdev);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void intel_mid_rtc_init(void);

extern int mrst_pmu_pci_set_power_state(struct pci_dev *pdev,
					pci_power_t state);

#ifdef CONFIG_ATOM_SOC_POWER
extern int pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state);
extern pci_power_t pmu_pci_choose_state(struct pci_dev *pdev);
#else
static inline int pmu_pci_set_power_state
		(struct pci_dev *pdev, pci_power_t state) {return 0; }
static inline pci_power_t pmu_pci_choose_state
				(struct pci_dev *pdev) { return PCI_D3hot; }
#endif /* CONFIG_ATOM_SOC_POWER */

#ifdef CONFIG_INTEL_MID_RAM_CONSOLE
extern void ram_consle_reserve_memory(void);
#else
static inline void ram_consle_reserve_memory(void){ }
#endif/* !CONFIG_INTEL_MID_RAM_CONSOLE*/

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

enum intel_mrfl_sim_type {
	INTEL_MRFL_CPU_SIMULATION_NONE = 0,
	INTEL_MRFL_CPU_SIMULATION_VP,
	INTEL_MRFL_CPU_SIMULATION_SLE,
	INTEL_MRFL_CPU_SIMULATION_HVP,
};

extern enum intel_mrfl_sim_type __intel_mrfl_sim_platform;

static inline enum intel_mrfl_sim_type intel_mrfl_identify_sim(void)
{
	return __intel_mrfl_sim_platform;
}

#define INTEL_MID_IRQ_OFFSET 0x100
#endif /* _ASM_X86_INTEL_MID_H */
