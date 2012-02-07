/*
 * mrst.c: Intel Moorestown platform specific setup code
 *
 * (C) Copyright 2008 Intel Corporation
 * Author: Jacob Pan (jacob.jun.pan@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/intel_mid_pm.h>
#include <linux/usb/penwell_otg.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/reboot.h>

void intel_mid_power_off(void)
{
	intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 1);
}

unsigned long __init intel_mid_calibrate_tsc(void)
{
	unsigned long flags, fast_calibrate;

	local_irq_save(flags);
	fast_calibrate = apbt_quick_calibrate();
	local_irq_restore(flags);

	if (fast_calibrate)
		return fast_calibrate;

	return 0;
}

static void __init intel_mid_time_init(void)
{
	sfi_table_parse(SFI_SIG_MTMR, NULL, NULL, sfi_parse_mtmr);
	switch (intel_mid_timer_options) {
	case INTEL_MID_TIMER_APBT_ONLY:
		break;
	case INTEL_MID_TIMER_LAPIC_APBT:
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		break;
	default:
		if (!boot_cpu_has(X86_FEATURE_ARAT))
			break;
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		return;
	}
	/* we need at least one APB timer */
	pre_init_apic_IRQ0();
	apbt_time_init();
}

static void __cpuinit intel_mid_arch_setup(void)
{
	if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x27)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_PENWELL;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x26)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x35)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_CLOVERVIEW;
	else {
		pr_err("Unknown Moorestown CPU (%d:%d), default to Lincroft\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	}
}

/* MID systems don't have i8042 controller */
int intel_mid_i8042_detect(void)
{
	return 0;
}

/*
 * Moorestown specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_intel_mid_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop;
	x86_init.resources.reserve_resources = x86_init_noop;

	x86_init.timers.timer_init = intel_mid_time_init;
	x86_init.timers.setup_percpu_clockev = x86_init_noop;

	x86_init.irqs.pre_vector_init = x86_init_noop;

	x86_init.oem.arch_setup = intel_mid_arch_setup;

	x86_cpuinit.setup_percpu_clockev = apbt_setup_secondary_clock;

	x86_platform.calibrate_tsc = intel_mid_calibrate_tsc;
	x86_platform.i8042_detect = intel_mid_i8042_detect;
	x86_init.timers.wallclock_init = intel_mid_rtc_init;
	x86_init.pci.init = intel_mid_pci_init;
	x86_init.pci.fixup_irqs = x86_init_noop;

	legacy_pic = &null_legacy_pic;

	/* Moorestown specific power_off/restart method */
	pm_power_off = intel_mid_power_off;
	if (mfld_shutdown) {
		saved_shutdown = machine_ops.shutdown;
		machine_ops.shutdown = mfld_shutdown;
	}
	machine_ops.emergency_restart  = intel_mid_reboot;

	/* Avoid searching for BIOS MP tables */
	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;
	set_bit(MP_BUS_ISA, mp_bus_not_pci);
}

/*
 * if user does not wanarch/x86/platform/mrst/mrst.ct to use per CPU apb timer, just give it a lower rating
 * than local apic timer and skip the late per cpu timer init.
 */
static inline int __init setup_x86_intel_mid_timer(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (strcmp("apbt_only", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_APBT_ONLY;
	else if (strcmp("lapic_and_apbt", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_LAPIC_APBT;
	else {
		pr_warning("X86 INTEL_MID timer option %s not recognised"
			   " use x86_intel_mid_timer=apbt_only or lapic_and_apbt\n",
			   arg);
		return -EINVAL;
	}
	return 0;
}
__setup("x86_intel_mid_timer=", setup_x86_intel_mid_timer);

/*
 * Parsing GPIO table first, since the DEVS table will need this table
 * to map the pin name to the actual pin.
 */
static struct sfi_gpio_table_entry *gpio_table;
static int gpio_num_entry;

static int __init sfi_parse_gpio(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_gpio_table_entry *pentry;
	int num, i;

	if (gpio_table)
		return 0;
	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_gpio_table_entry);
	pentry = (struct sfi_gpio_table_entry *)sb->pentry;

	gpio_table = (struct sfi_gpio_table_entry *)
				kmalloc(num * sizeof(*pentry), GFP_KERNEL);
	if (!gpio_table)
		return -1;
	memcpy(gpio_table, pentry, num * sizeof(*pentry));
	gpio_num_entry = num;

	pr_debug("GPIO pin info:\n");
	for (i = 0; i < num; i++, pentry++)
		pr_debug("info[%2d]: controller = %16.16s, pin_name = %16.16s,"
		" pin = %d\n", i,
			pentry->controller_name,
			pentry->pin_name,
			pentry->pin_no);
	return 0;
}

static int get_gpio_by_name(const char *name)
{
	struct sfi_gpio_table_entry *pentry = gpio_table;
	int i;

	if (!pentry)
		return -1;
	for (i = 0; i < gpio_num_entry; i++, pentry++) {
		if (!strncmp(name, pentry->pin_name, SFI_NAME_LEN))
			return pentry->pin_no;
	}
	return -1;
}
/*tc35876x DSI_LVDS bridge chip and panel platform data*/
static void *tc35876x_platform_data(void *data)
{
	static struct tc35876x_platform_data pdata;
	pdata.gpio_bridge_reset = get_gpio_by_name("LCMB_RXEN");
	pdata.gpio_panel_bl_en = get_gpio_by_name("6S6P_BL_EN");
	pdata.gpio_panel_vadd = get_gpio_by_name("EN_VREG_LCD_V3P3");
	return &pdata;
}

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
	u8 trash_itp;/* true if this driver uses pin muxed with XDB connector */
};

/* the offset for the mapping of global gpio pin to irq */
#define MRST_IRQ_OFFSET 0x100

static void __init *pmic_gpio_platform_data(void *info)
{
	static struct intel_pmic_gpio_platform_data pmic_gpio_pdata;
	int gpio_base = get_gpio_by_name("pmic_gpio_base");

	if (gpio_base == -1)
		gpio_base = 64;
	pmic_gpio_pdata.gpio_base = gpio_base;
	pmic_gpio_pdata.irq_base = gpio_base + MRST_IRQ_OFFSET;
	pmic_gpio_pdata.gpiointr = 0xffffeff8;

	return &pmic_gpio_pdata;
}

static void __init *max3111_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr = get_gpio_by_name("max3111_int");

	spi_info->mode = SPI_MODE_0;
	if (intr == -1)
		return NULL;
	spi_info->irq = intr + MRST_IRQ_OFFSET;
	return NULL;
}

/* we have multiple max7315 on the board ... */
#define MAX7315_NUM 2
static void __init *max7315_platform_data(void *info)
{
	static struct pca953x_platform_data max7315_pdata[MAX7315_NUM];
	static int nr;
	struct pca953x_platform_data *max7315 = &max7315_pdata[nr];
	struct i2c_board_info *i2c_info = info;
	int gpio_base, intr;
	char base_pin_name[SFI_NAME_LEN + 1];
	char intr_pin_name[SFI_NAME_LEN + 1];

	if (nr == MAX7315_NUM) {
		pr_err("too many max7315s, we only support %d\n",
				MAX7315_NUM);
		return NULL;
	}
	/* we have several max7315 on the board, we only need load several
	 * instances of the same pca953x driver to cover them
	 */
	strcpy(i2c_info->type, "max7315");
	if (nr++) {
		sprintf(base_pin_name, "max7315_%d_base", nr);
		sprintf(intr_pin_name, "max7315_%d_int", nr);
	} else {
		strcpy(base_pin_name, "max7315_base");
		strcpy(intr_pin_name, "max7315_int");
	}

	gpio_base = get_gpio_by_name(base_pin_name);
	intr = get_gpio_by_name(intr_pin_name);

	if (gpio_base == -1)
		return NULL;
	max7315->gpio_base = gpio_base;
	if (intr != -1) {
		i2c_info->irq = intr + MRST_IRQ_OFFSET;
		max7315->irq_base = gpio_base + MRST_IRQ_OFFSET;
	} else {
		i2c_info->irq = -1;
		max7315->irq_base = -1;
	}
	return max7315;
}

static void *tca6416_platform_data(void *info)
{
	static struct pca953x_platform_data tca6416;
	struct i2c_board_info *i2c_info = info;
	int gpio_base, intr;
	char base_pin_name[SFI_NAME_LEN + 1];
	char intr_pin_name[SFI_NAME_LEN + 1];

	strcpy(i2c_info->type, "tca6416");
	strcpy(base_pin_name, "tca6416_base");
	strcpy(intr_pin_name, "tca6416_int");

	gpio_base = get_gpio_by_name(base_pin_name);
	intr = get_gpio_by_name(intr_pin_name);

	if (gpio_base == -1)
		return NULL;
	tca6416.gpio_base = gpio_base;
	if (intr != -1) {
		i2c_info->irq = intr + MRST_IRQ_OFFSET;
		tca6416.irq_base = gpio_base + MRST_IRQ_OFFSET;
	} else {
		i2c_info->irq = -1;
		tca6416.irq_base = -1;
	}
	return &tca6416;
}

static void *mpu3050_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("mpu3050_int");

	if (intr == -1)
		return NULL;

	i2c_info->irq = intr + MRST_IRQ_OFFSET;
	return NULL;
}

static void *ektf2136_spi_platform_data(void *info)
{
	static int dummy;
	struct spi_board_info *spi_info = info;
	int intr = get_gpio_by_name("ts_int");

	if (intr == -1)
		return NULL;
	spi_info->irq = intr + MRST_IRQ_OFFSET;

	/* we return a dummy pdata */
	return &dummy;
}

static void __init *emc1403_platform_data(void *info)
{
	static short intr2nd_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("thermal_int");
	int intr2nd = get_gpio_by_name("thermal_alert");

	if (intr == -1 || intr2nd == -1)
		return NULL;

	i2c_info->irq = intr + MRST_IRQ_OFFSET;
	intr2nd_pdata = intr2nd + MRST_IRQ_OFFSET;

	return &intr2nd_pdata;
}

static void __init *lis331dl_platform_data(void *info)
{
	static short intr2nd_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("accel_int");
	int intr2nd = get_gpio_by_name("accel_2");

	if (intr == -1 || intr2nd == -1)
		return NULL;

	i2c_info->irq = intr + MRST_IRQ_OFFSET;
	intr2nd_pdata = intr2nd + MRST_IRQ_OFFSET;

	return &intr2nd_pdata;
}

/* MFLD NFC controller (PN544) platform init */
#define NFC_HOST_INT_GPIO               "NFC-intr"
#define NFC_ENABLE_GPIO                 "NFC-enable"
#define NFC_FW_RESET_GPIO               "NFC-reset"

static unsigned int nfc_host_int_gpio, nfc_enable_gpio, nfc_fw_reset_gpio;

static int pn544_nfc_request_resources(struct i2c_client *client)
{
	int ret;

	ret = gpio_request(nfc_host_int_gpio, NFC_HOST_INT_GPIO);
	if (ret) {
		dev_err(&client->dev, "Request NFC INT GPIO fails %d\n", ret);
		return -1;
	}

	ret = gpio_direction_input(nfc_host_int_gpio);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_int;
	}

	ret = gpio_request(nfc_enable_gpio, NFC_ENABLE_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC Enable GPIO fails %d\n", ret);
		goto err_int;
	}

	ret = gpio_direction_output(nfc_enable_gpio, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_enable;
	}

	ret = gpio_request(nfc_fw_reset_gpio, NFC_FW_RESET_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC FW Reset GPIO fails %d\n", ret);
		goto err_enable;
	}

	ret = gpio_direction_output(nfc_fw_reset_gpio, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_fw;
	}

	return 0;
err_fw:
	gpio_free(nfc_fw_reset_gpio);
err_enable:
	gpio_free(nfc_enable_gpio);
err_int:
	gpio_free(nfc_host_int_gpio);
	return -1;
}

void *pn544_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = (struct i2c_board_info *) info;
	static struct pn544_i2c_platform_data mfld_pn544_nfc_platform_data;

	memset(&mfld_pn544_nfc_platform_data, 0x00,
		sizeof(struct pn544_i2c_platform_data));

	nfc_host_int_gpio = get_gpio_by_name(NFC_HOST_INT_GPIO);
	if (nfc_host_int_gpio == -1)
		return NULL;
	nfc_enable_gpio = get_gpio_by_name(NFC_ENABLE_GPIO);
	if (nfc_enable_gpio  == -1)
		return NULL;
	nfc_fw_reset_gpio = get_gpio_by_name(NFC_FW_RESET_GPIO);
	if (nfc_fw_reset_gpio == -1)
		return NULL;

	mfld_pn544_nfc_platform_data.irq_gpio = nfc_host_int_gpio;
	mfld_pn544_nfc_platform_data.ven_gpio = nfc_enable_gpio;
	mfld_pn544_nfc_platform_data.firm_gpio = nfc_fw_reset_gpio;

	i2c_info->irq = nfc_host_int_gpio + MRST_IRQ_OFFSET;
	mfld_pn544_nfc_platform_data.request_resources =
		pn544_nfc_request_resources;

	return &mfld_pn544_nfc_platform_data;
}

/* MFLD iCDK touchscreen data */
#define CYTTSP_GPIO_PIN 0x3E
static int cyttsp_init(int on)
{
	int ret;

	if (on) {
		ret = gpio_request(CYTTSP_GPIO_PIN, "cyttsp_irq");
		if (ret < 0) {
			pr_err("%s: gpio request failed\n", __func__);
			return ret;
		}

		ret = gpio_direction_input(CYTTSP_GPIO_PIN);
		if (ret < 0) {
			pr_err("%s: gpio direction config failed\n", __func__);
			gpio_free(CYTTSP_GPIO_PIN);
			return ret;
		}
	} else {
		gpio_free(CYTTSP_GPIO_PIN);
	}
	return 0;
}

static void *cyttsp_platform_data(void *info)
{
	static struct cyttsp_platform_data cyttsp_pdata = {
		.init = cyttsp_init,
		.mt_sync = input_mt_sync,
		.maxx = 479,
		.maxy = 853,
		.flags = 0,
		.gen = CY_GEN3,
		.use_st = 0,
		.use_mt = 1,
		.use_trk_id = 0,
		.use_hndshk = 1,
		.use_timer = 0,
		.use_sleep = 1,
		.use_gestures = 0,
		.act_intrvl = CY_ACT_INTRVL_DFLT,
		.tch_tmout = CY_TCH_TMOUT_DFLT,
		.lp_intrvl = CY_LP_INTRVL_DFLT / 2,
		.name = CY_I2C_NAME,
		.irq_gpio = CYTTSP_GPIO_PIN,
	};

	return &cyttsp_pdata;
}

static void __init *no_platform_data(void *info)
{
	return NULL;
}

static struct resource msic_resources[] = {
	{
		.start	= INTEL_MSIC_IRQ_PHYS_BASE,
		.end	= INTEL_MSIC_IRQ_PHYS_BASE + 64 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct intel_msic_platform_data msic_pdata;

static struct platform_device msic_device = {
	.name		= "intel_msic",
	.id		= -1,
	.dev		= {
		.platform_data	= &msic_pdata,
	},
	.num_resources	= ARRAY_SIZE(msic_resources),
	.resource	= msic_resources,
};

static inline bool mrst_has_msic(void)
{
	return ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL) ||
		(intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW));
}

static int msic_scu_status_change(struct notifier_block *nb,
				  unsigned long code, void *data)
{
	if (code == SCU_DOWN) {
		platform_device_unregister(&msic_device);
		return 0;
	}

	return platform_device_register(&msic_device);
}

static int __init msic_init(void)
{
	static struct notifier_block msic_scu_notifier = {
		.notifier_call	= msic_scu_status_change,
	};

	/*
	 * We need to be sure that the SCU IPC is ready before MSIC device
	 * can be registered.
	 */
	if (mrst_has_msic())
		intel_scu_notifier_add(&msic_scu_notifier);

	return 0;
}
arch_initcall(msic_init);

/*
 * msic_generic_platform_data - sets generic platform data for the block
 * @info: pointer to the SFI device table entry for this block
 * @block: MSIC block
 *
 * Function sets IRQ number from the SFI table entry for given device to
 * the MSIC platform data.
 */
static void *msic_generic_platform_data(void *info, enum intel_msic_block block)
{
	struct sfi_device_table_entry *entry = info;

	BUG_ON(block < 0 || block >= INTEL_MSIC_BLOCK_LAST);
	msic_pdata.irq[block] = entry->irq;

	return no_platform_data(info);
}

static void *msic_adc_platform_data(void *info)
{
	static struct intel_mid_gpadc_platform_data pdata;
	pdata.intr = 0xffff7fc0;
	msic_pdata.gpadc = &pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_ADC);
}


static void *hsi_modem_platform_data(void *data)
{
	int rst_out = get_gpio_by_name("ifx_mdm_rst_out");
	int pwr_on = get_gpio_by_name("ifx_mdm_pwr_on");
	int rst_pmu = get_gpio_by_name("ifx_mdm_rst_pmu");
	int fcdp_rb = get_gpio_by_name("modem-gpio2");

	static const char hsi_char_name[]	= "hsi_char";
	static const char hsi_ffl_name[]	= "hsi-ffl";

	static struct hsi_board_info hsi_info[2] = {
		[0] = {
			.name = hsi_char_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 200000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		},
		[1] = {
			.name = hsi_ffl_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 100000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		}
	};

	static struct hsi_mid_platform_data mid_info = {
		.tx_dma_channels[0] = -1,
		.tx_dma_channels[1] = 5,
		.tx_dma_channels[2] = -1,
		.tx_dma_channels[3] = -1,
		.tx_dma_channels[4] = -1,
		.tx_dma_channels[5] = -1,
		.tx_dma_channels[6] = -1,
		.tx_dma_channels[7] = -1,
		.tx_fifo_sizes[0] = -1,
		.tx_fifo_sizes[1] = 1024,
		.tx_fifo_sizes[2] = -1,
		.tx_fifo_sizes[3] = -1,
		.tx_fifo_sizes[4] = -1,
		.tx_fifo_sizes[5] = -1,
		.tx_fifo_sizes[6] = -1,
		.tx_fifo_sizes[7] = -1,
		.rx_dma_channels[0] = -1,
		.rx_dma_channels[1] = 1,
		.rx_dma_channels[2] = -1,
		.rx_dma_channels[3] = -1,
		.rx_dma_channels[4] = -1,
		.rx_dma_channels[5] = -1,
		.rx_dma_channels[6] = -1,
		.rx_dma_channels[7] = -1,
		.rx_fifo_sizes[0] = -1,
		.rx_fifo_sizes[1] = 1024,
		.rx_fifo_sizes[2] = -1,
		.rx_fifo_sizes[3] = -1,
		.rx_fifo_sizes[4] = -1,
		.rx_fifo_sizes[5] = -1,
		.rx_fifo_sizes[6] = -1,
		.rx_fifo_sizes[7] = -1,
	};

	printk(KERN_INFO "HSI platform data setup\n");

	printk(KERN_INFO "HSI mdm GPIOs %d, %d, %d, %d\n",
		rst_out, pwr_on, rst_pmu, fcdp_rb);

	mid_info.gpio_mdm_rst_out = rst_out;
	mid_info.gpio_mdm_pwr_on = pwr_on;
	mid_info.gpio_mdm_rst_bbn = rst_pmu;
	mid_info.gpio_fcdp_rb = fcdp_rb;

	hsi_info[0].platform_data = (void *)&mid_info;
	hsi_info[1].platform_data = (void *)&mid_info;

	return &hsi_info[0];
}

static void *msic_battery_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_BATTERY);
}

static void *msic_gpio_platform_data(void *info)
{
	static struct intel_msic_gpio_pdata pdata;
	int gpio = get_gpio_by_name("msic_gpio_base");

	if (gpio < 0)
		return NULL;

	pdata.gpio_base = gpio;
	msic_pdata.gpio = &pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_GPIO);
}

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C-1 to recover devices from abnormal status.
 * currently, only max17042 on I2C-1 needs such workaround */
#define I2C_1_GPIO_PIN 27
	int i;
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_GPIO);
	gpio_direction_output(I2C_1_GPIO_PIN, 0);
	gpio_set_value(I2C_1_GPIO_PIN, 1);
	udelay(10);
	gpio_set_value(I2C_1_GPIO_PIN, 0);
	udelay(10);
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_ALT_1);
#undef I2C_1_GPIO_PIN
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

static int __init msic_battery_check(struct sfi_table_header *table)
{

       if (!table) {
	pr_info("invalid battery detected\n");
	is_valid_batt = false;
	return -ENODEV;
       } else {
	pr_info("valid battery detected\n");
	is_valid_batt = true;
	return 0;
      }
}

static void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	int intr = get_gpio_by_name("max17042");

	i2c_info->irq = intr + MRST_IRQ_OFFSET;
	if (is_valid_batt) {
		platform_data.enable_current_sense = true;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		platform_data.enable_current_sense = false;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}
	platform_data.is_init_done = 0;

	platform_data.reset_i2c_lines = max17042_i2c_reset_workaround;

#ifdef CONFIG_BATTERY_INTEL_MDF
	platform_data.current_sense_enabled =
	    intel_msic_is_current_sense_enabled;
	platform_data.battery_present = intel_msic_check_battery_present;
	platform_data.battery_health = intel_msic_check_battery_health;
	platform_data.battery_status = intel_msic_check_battery_status;
	platform_data.battery_pack_temp = intel_msic_get_battery_pack_temp;
	platform_data.save_config_data = intel_msic_save_config_data;
	platform_data.restore_config_data = intel_msic_restore_config_data;

	platform_data.is_cap_shutdown_enabled =
					intel_msic_is_capacity_shutdown_en;
	platform_data.is_volt_shutdown_enabled = intel_msic_is_volt_shutdown_en;
	platform_data.is_lowbatt_shutdown_enabled =
					intel_msic_is_lowbatt_shutdown_en;
	platform_data.get_vmin_threshold = intel_msic_get_vsys_min;
#endif

	return &platform_data;
}

static void *msic_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc("sst-platform", -1);
	if (!pdev) {
		pr_err("failed to allocate audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_AUDIO);
}

static void *msic_power_btn_platform_data(void *info)
{
	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_POWER_BTN);
}

static void *msic_ocd_platform_data(void *info)
{
	static struct intel_msic_ocd_pdata pdata;
	int gpio = get_gpio_by_name("ocd_gpio");

	if (gpio < 0)
		return NULL;

	pdata.gpio = gpio;
	msic_pdata.ocd = &pdata;

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_OCD);
}

/* MFLD iCDK camera sensor GPIOs */

#define GP_CAMERA_0_POWER_DOWN		"cam0_vcm_2p8"
#define GP_CAMERA_1_POWER_DOWN		"camera_1_power"
#define GP_CAMERA_0_RESET		"camera_0_reset"
#define GP_CAMERA_1_RESET		"camera_1_reset"
/* Need modify sensor driver's platform data structure to eliminate static */
static int gp_camera0_reset;
static int gp_camera0_power_down;
static int gp_camera1_reset;
static int gp_camera1_power_down;
static int camera_vprog1_on;

/*
 * One-time gpio initialization.
 * @name: gpio name: coded in SFI table
 * @gpio: gpio pin number (bypass @name)
 * @dir: GPIOF_DIR_IN or GPIOF_DIR_OUT
 * @value: if dir = GPIOF_DIR_OUT, this is the init value for output pin
 * if dir = GPIOF_DIR_IN, this argument is ignored
 * return: a positive pin number if succeeds, otherwise a negative value
 */
static int camera_sensor_gpio(int gpio, char *name, int dir, int value)
{
	int ret, pin;

	if (gpio == -1) {
		pin = get_gpio_by_name(name);
		if (pin == -1) {
			pr_err("%s: failed to get gpio(name: %s)\n",
						__func__, name);
			return -EINVAL;
		}
	} else {
		pin = gpio;
	}

	ret = gpio_request(pin, name);
	if (ret) {
		pr_err("%s: failed to request gpio(pin %d)\n", __func__, pin);
		return -EINVAL;
	}

	if (dir == GPIOF_DIR_OUT)
		ret = gpio_direction_output(pin, value);
	else
		ret = gpio_direction_input(pin);

	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
							__func__, pin);
		gpio_free(pin);
	}

	return ret ? ret : pin;
}

/*
 * Configure MIPI CSI physical parameters.
 * @port: ATOMISP_CAMERA_PORT_PRIMARY or ATOMISP_CAMERA_PORT_SECONDARY
 * @lanes: for ATOMISP_CAMERA_PORT_PRIMARY, there could be 2 or 4 lanes
 * for ATOMISP_CAMERA_PORT_SECONDARY, there is only one lane.
 * @format: MIPI CSI pixel format, see include/linux/atomisp_platform.h
 * @bayer_order: MIPI CSI bayer order, see include/linux/atomisp_platform.h
 */
static int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *csi = NULL;

	if (flag) {
		csi = kzalloc(sizeof(*csi), GFP_KERNEL);
		if (!csi) {
			dev_err(&client->dev, "out of memory\n");
			return -ENOMEM;
		}
		csi->port = port;
		csi->num_lanes = lanes;
		csi->input_format = format;
		csi->raw_bayer_order = bayer_order;
		v4l2_set_subdev_hostdata(sd, (void *)csi);
	} else {
		csi = v4l2_get_subdev_hostdata(sd);
		kfree(csi);
	}

	return 0;
}


/*
 * MFLD PR2 primary camera sensor - MT9E013 platform data
 */
static int mt9e013_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (gp_camera0_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera0_reset = ret;
	}

	if (flag) {
		gpio_set_value(gp_camera0_reset, 0);
		msleep(20);
		gpio_set_value(gp_camera0_reset, 1);
	} else {
		gpio_set_value(gp_camera0_reset, 0);
	}

	return 0;
}

static int mt9e013_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int mt9e013_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (gp_camera0_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_POWER_DOWN,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_POWER_DOWN);
		gp_camera0_power_down = ret;
	}

	if (flag) {
		if (gp_camera0_power_down >= 0)
			gpio_set_value(gp_camera0_power_down, 1);
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
		if (gp_camera0_power_down >= 0)
			gpio_set_value(gp_camera0_power_down, 0);
	}

	return 0;
}

static int mt9e013_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static struct camera_sensor_platform_data mt9e013_sensor_platform_data = {
	.gpio_ctrl	= mt9e013_gpio_ctrl,
	.flisclk_ctrl	= mt9e013_flisclk_ctrl,
	.power_ctrl	= mt9e013_power_ctrl,
	.csi_cfg	= mt9e013_csi_configure,
};

void *mt9e013_platform_data_init(void *info)
{
	gp_camera0_reset = -1;
	gp_camera0_power_down = -1;

	return &mt9e013_sensor_platform_data;
}

/*
 * CLV PR0 primary camera sensor - OV8830 platform data
 */

#define OV8830_I2C_ADDR	(0x6C >> 1)	/* i2c address, 0x20 or 0x6C */
#define OV8830_BUS	4		/* i2c bus number */

static struct i2c_board_info ov8830_info = {
	.type = "ov8830",
	.flags = 0,
	.addr = OV8830_I2C_ADDR,
	.irq = 255,
};

static int ov8830_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (gp_camera0_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera0_reset = ret;
	}

	if (flag) {
		gpio_set_value(gp_camera0_reset, 0);
		msleep(20);
		gpio_set_value(gp_camera0_reset, 1);
	} else {
		gpio_set_value(gp_camera0_reset, 0);
	}

	return 0;
}

static int ov8830_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int ov8830_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	if (flag) {
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
	}

	return 0;
}

static int ov8830_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static struct camera_sensor_platform_data ov8830_sensor_platform_data = {
	.gpio_ctrl      = ov8830_gpio_ctrl,
	.flisclk_ctrl   = ov8830_flisclk_ctrl,
	.power_ctrl     = ov8830_power_ctrl,
	.csi_cfg        = ov8830_csi_configure,
};

void *ov8830_platform_data_init(void *info)
{
	gp_camera0_reset = -1;
	gp_camera0_power_down = -1;

	return &ov8830_sensor_platform_data;
}

/*
 * MFLD PR2 secondary camera sensor - MT9M114 platform data
 */

#define MT9M114_I2C_ADDR (0x90 >> 1)	/* i2c address, 0x90 or 0xBA */
#define MT9M114_BUS      4		/* i2c bus number */

static struct i2c_board_info mt9m114_info = {
	.type = "mt9m114",
	.flags = 0,
	.addr = MT9M114_I2C_ADDR,
	.irq = 255,
};

static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (gp_camera1_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera1_reset = ret;
	}

	if (flag)
		gpio_set_value(gp_camera1_reset, 1);
	else
		gpio_set_value(gp_camera1_reset, 0);

	return 0;
}

static int mt9m114_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int mt9e013_reset;
static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	/* Note here, there maybe a workaround to avoid I2C SDA issue */
	if (gp_camera1_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera1_power_down = ret;
	}

	if (gp_camera1_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		gp_camera1_reset = ret;
	}

	if (flag) {
		if (!mt9e013_reset) {
			mt9e013_power_ctrl(sd, 1);
			mt9e013_gpio_ctrl(sd, 0);
			mt9e013_gpio_ctrl(sd, 1);
			mt9e013_gpio_ctrl(sd, 0);
			mt9e013_power_ctrl(sd, 0);
			mt9e013_reset = 1;
		}

		gpio_set_value(gp_camera1_reset, 0);
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
		gpio_set_value(gp_camera1_power_down, 1);
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
		gpio_set_value(gp_camera1_power_down, 0);

		mt9e013_reset = 0;
	}

	return 0;
}

static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	= mt9m114_gpio_ctrl,
	.flisclk_ctrl	= mt9m114_flisclk_ctrl,
	.power_ctrl	= mt9m114_power_ctrl,
	.csi_cfg	= mt9m114_csi_configure,
};

void *mt9m114_platform_data_init(void *info)
{
	gp_camera1_reset = -1;
	gp_camera1_power_down = -1;

	return &mt9m114_sensor_platform_data;
}

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
	char		name[SD_NAME_SIZE];
	int		bus_num;
	unsigned short	addr;
	u32		board_ref_clock;
	void		*platform_data;
};

#define MAX_DELAYEDDEVS	60
static void *delayed_devs[MAX_DELAYEDDEVS];
typedef void (*delayed_callback_t)(void *dev_desc);
static delayed_callback_t delayed_callbacks[MAX_DELAYEDDEVS];
static int delayed_next_dev;

static void intel_delayed_device_register(void *dev,
				void (*delayed_callback)(void *dev_desc))
{
	delayed_devs[delayed_next_dev] = dev;
	delayed_callbacks[delayed_next_dev++] = delayed_callback;
	BUG_ON(delayed_next_dev == MAX_DELAYEDDEVS);
}

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static struct wl12xx_platform_data mid_wifi_control = {
	.board_ref_clock = 1,
	.irq = 2,
	.board_tcxo_clock = 1,
	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static struct regulator_consumer_supply wl12xx_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "0000:00:00.0", /*default value*/
};

static struct regulator_init_data wl12xx_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &wl12xx_vmmc3_supply,
};

static struct fixed_voltage_config wl12xx_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000,
	.gpio			= 75,
	.startup_delay		= 70000,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &wl12xx_vmmc3,
};

static struct platform_device wl12xx_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &wl12xx_vwlan,
	},
};

#define WL12XX_SFI_GPIO_IRQ_NAME "WLAN-interrupt"
#define WL12XX_SFI_GPIO_ENABLE_NAME "WLAN-enable"
#define ICDK_BOARD_REF_CLK 26000000
#define NCDK_BOARD_REF_CLK 38400000

extern int sdhci_pci_request_regulators(void);
void __init wl12xx_platform_data_init_post_scu(void *info)
{
	struct sd_board_info *sd_info = info;
	int wifi_irq_gpio;
	int err;

	/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(WL12XX_SFI_GPIO_IRQ_NAME);
	if (wifi_irq_gpio == -1) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return;
	}
	err = gpio_request(wifi_irq_gpio, "wl12xx");
	if (err < 0) {
		pr_err("%s: Unable to request GPIO\n", __func__);
		return;
	}
	err = gpio_direction_input(wifi_irq_gpio);
	if (err < 0) {
		pr_err("%s: Unable to set GPIO direction\n", __func__);
		return;
	}
	mid_wifi_control.irq = gpio_to_irq(wifi_irq_gpio);
	if (mid_wifi_control.irq < 0) {
		pr_err("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       mid_wifi_control.irq);
		return;
	}
	/* Set our board_ref_clock from SFI SD board info */
	if (sd_info->board_ref_clock == ICDK_BOARD_REF_CLK)
		/*iCDK board*/
		/*26Mhz TCXO clock ref*/
		mid_wifi_control.board_ref_clock = 1;
	else if (sd_info->board_ref_clock == NCDK_BOARD_REF_CLK)
		/*nCDK board*/
		/*38,4Mhz TCXO clock ref*/
		mid_wifi_control.board_ref_clock = 2;

	err = wl12xx_set_platform_data(&mid_wifi_control);
	if (err < 0)
		pr_err("error setting wl12xx data\n");

	/* this is the fake regulator that mmc stack use to power of the
	   wifi sdio card via runtime_pm apis */
	wl12xx_vwlan.gpio = get_gpio_by_name(WL12XX_SFI_GPIO_ENABLE_NAME);
	if (wl12xx_vwlan.gpio == -1) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}
	/* format vmmc reg address from sfi table */
	sprintf((char *)wl12xx_vmmc3_supply.dev_name, "0000:00:%02x.%01x",
		(sd_info->addr)>>8, sd_info->addr&0xFF);

	err = platform_device_register(&wl12xx_vwlan_device);
	if (err < 0)
		pr_err("error platform_device_register\n");

	sdhci_pci_request_regulators();
}

void __init *wl12xx_platform_data_init(void *info)
{
	struct sd_board_info *sd_info;

	sd_info = kmemdup(info, sizeof(*sd_info), GFP_KERNEL);
	if (!sd_info) {
		pr_err("MRST: fail to alloc mem for delayed wl12xx dev\n");
		return NULL;
	}
	intel_delayed_device_register(sd_info,
				      wl12xx_platform_data_init_post_scu);

	return &mid_wifi_control;
}
#else
void *wl12xx_platform_data_init(void *info)
{
	return NULL;
}
#endif

void *atmel_mxt224_platform_data_init(void *info)
{
	static struct mxt_platform_data mxt_pdata;

	mxt_pdata.numtouch       = 2;
	mxt_pdata.max_x          = 1023;
	mxt_pdata.max_y          = 975;
	mxt_pdata.orientation    = MXT_MSGB_T9_ORIENT_HORZ_FLIP;
	mxt_pdata.reset          = get_gpio_by_name("ts_rst");
	mxt_pdata.irq            = get_gpio_by_name("ts_int");

	return &mxt_pdata;
}

static struct rmi_f11_functiondata synaptics_f11_data = {
	.swap_axes = true,
};

static unsigned char synaptic_keys[31] = {1, 2, 3, 4,};
			/* {KEY_BACK,KEY_MENU,KEY_HOME,KEY_SEARCH,} */

static struct rmi_button_map synaptics_button_map = {
	.nbuttons = 31,
	.map = synaptic_keys,
};
static struct rmi_f19_functiondata  synaptics_f19_data = {
	.button_map = &synaptics_button_map,
};

#define RMI_F11_INDEX 0x11
#define RMI_F19_INDEX 0x19

static struct rmi_functiondata synaptics_functiondata[] = {
	{
		.function_index = RMI_F11_INDEX,
		.data = &synaptics_f11_data,
	},
	{
		.function_index = RMI_F19_INDEX,
		.data = &synaptics_f19_data,
	},
};

static struct rmi_functiondata_list synaptics_perfunctiondata = {
	.count = ARRAY_SIZE(synaptics_functiondata),
	.functiondata = synaptics_functiondata,
};


static struct rmi_sensordata s3202_sensordata = {
	.perfunctiondata = &synaptics_perfunctiondata,
};

void *s3202_platform_data_init(void *info)
{
	struct i2c_board_info *i2c_info = info;
	static struct rmi_i2c_platformdata s3202_platform_data = {
		.delay_ms = 50,
		.sensordata = &s3202_sensordata,
	};

	s3202_platform_data.i2c_address = i2c_info->addr;
	s3202_sensordata.attn_gpio_number = get_gpio_by_name("ts_int");
	s3202_sensordata.rst_gpio_number  = get_gpio_by_name("ts_rst");

	return &s3202_platform_data;
}

static const struct devs_id __initconst device_ids[] = {
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data},
	{"cy8ctma340", SFI_DEV_TYPE_I2C, 1, &cyttsp_platform_data},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data},
	{"ektf2136_spi", SFI_DEV_TYPE_SPI, 0, &ektf2136_spi_platform_data},
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data},
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data},
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data_init},
	/* MSIC subdevices */
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data},

	/*
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data_init},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data_init},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data_init},
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &atmel_mxt224_platform_data_init},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &s3202_platform_data_init},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data},
	{},
};

static const struct intel_v4l2_subdev_id v4l2_ids_clv[] = {
	{"ov8830", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3554", LED_FLASH, -1},
	{},
};

static const struct intel_v4l2_subdev_id v4l2_ids_mfld[] = {
	{"mt9e013", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3554", LED_FLASH, -1},
	{},
};

static const struct intel_v4l2_subdev_id *get_v4l2_ids(int *n_subdev)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW) {
		if (n_subdev)
			*n_subdev = ARRAY_SIZE(v4l2_ids_clv);
		return v4l2_ids_clv;
	} else {
		if (n_subdev)
			*n_subdev = ARRAY_SIZE(v4l2_ids_mfld);
		return v4l2_ids_mfld;
	}
}

static struct atomisp_platform_data *v4l2_subdev_table_head;

static void intel_ignore_i2c_device_register(int bus,
					     struct i2c_board_info *idev)
{
	int n_subdev;
	const struct intel_v4l2_subdev_id *vdev = get_v4l2_ids(&n_subdev);
	struct intel_v4l2_subdev_i2c_board_info *info;
	static struct intel_v4l2_subdev_table *subdev_table;
	enum intel_v4l2_subdev_type type = 0;
	enum atomisp_camera_port port;
	static int i;

	while (vdev->name[0]) {
		if (!strncmp(vdev->name, idev->type, 16)) {
			/* compare name */
			type = vdev->type;
			port = vdev->port;
			break;
		}
		vdev++;
	}

	if (!type) /* not found */
		return;

	info = kzalloc(sizeof(struct intel_v4l2_subdev_i2c_board_info),
		       GFP_KERNEL);
	if (!info) {
		pr_err("MRST: fail to alloc mem for ignored i2c dev %s\n",
		       idev->type);
		return;
	}

	info->i2c_adapter_id = bus;
	/* set platform data */
	memcpy(&info->board_info, idev, sizeof(*idev));

	if (v4l2_subdev_table_head == NULL) {
		subdev_table = kzalloc(sizeof(struct intel_v4l2_subdev_table)
			* n_subdev, GFP_KERNEL);

		if (!subdev_table) {
			pr_err("MRST: fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			return;
		}

		v4l2_subdev_table_head = kzalloc(
			sizeof(struct atomisp_platform_data), GFP_KERNEL);
		if (!v4l2_subdev_table_head) {
			pr_err("MRST: fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			kfree(subdev_table);
			return;
		}
		v4l2_subdev_table_head->subdevs = subdev_table;
	}

	memcpy(&subdev_table[i].v4l2_subdev, info, sizeof(*info));
	subdev_table[i].type = type;
	subdev_table[i].port = port;
	i++;
	kfree(info);
	return;
}

const struct atomisp_platform_data *intel_get_v4l2_subdev_table(void)
{
	if (v4l2_subdev_table_head)
		return v4l2_subdev_table_head;
	else {
		pr_err("MRST: no camera device in the SFI table\n");
		return NULL;
	}
}
EXPORT_SYMBOL_GPL(intel_get_v4l2_subdev_table);

#define MAX_IPCDEVS	24
static struct platform_device *ipc_devs[MAX_IPCDEVS];
static int ipc_next_dev;

#define MAX_SCU_SPI	24
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static int spi_next_dev;

#define MAX_SCU_I2C	24
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static int i2c_bus[MAX_SCU_I2C];
static int i2c_next_dev;

static void __init intel_scu_device_register(struct platform_device *pdev)
{
	if (ipc_next_dev == MAX_IPCDEVS)
		pr_err("too many SCU IPC devices");
	else
		ipc_devs[ipc_next_dev++] = pdev;
}

static void __init intel_scu_spi_device_register(struct spi_board_info *sdev)
{
	struct spi_board_info *new_dev;

	if (spi_next_dev == MAX_SCU_SPI) {
		pr_err("too many SCU SPI devices");
		return;
	}

	new_dev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed spi dev %s\n",
			sdev->modalias);
		return;
	}
	memcpy(new_dev, sdev, sizeof(*sdev));

	spi_devs[spi_next_dev++] = new_dev;
}

static void __init intel_scu_i2c_device_register(int bus,
						struct i2c_board_info *idev)
{
	struct i2c_board_info *new_dev;

	if (i2c_next_dev == MAX_SCU_I2C) {
		pr_err("too many SCU I2C devices");
		return;
	}

	new_dev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed i2c dev %s\n",
			idev->type);
		return;
	}
	memcpy(new_dev, idev, sizeof(*idev));

	i2c_bus[i2c_next_dev] = bus;
	i2c_devs[i2c_next_dev++] = new_dev;
}

BLOCKING_NOTIFIER_HEAD(intel_scu_notifier);
EXPORT_SYMBOL_GPL(intel_scu_notifier);

/* Called by IPC driver */
void intel_scu_devices_create(void)
{
	int i;

	for (i = 0; i < delayed_next_dev; i++)
		delayed_callbacks[i](delayed_devs[i]);

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_add(ipc_devs[i]);

	for (i = 0; i < spi_next_dev; i++)
		spi_register_board_info(spi_devs[i], 1);

	for (i = 0; i < i2c_next_dev; i++) {
		struct i2c_adapter *adapter;
		struct i2c_client *client;

		adapter = i2c_get_adapter(i2c_bus[i]);
		if (adapter) {
			client = i2c_new_device(adapter, i2c_devs[i]);
			if (!client)
				pr_err("can't create i2c device %s\n",
					i2c_devs[i]->type);
		} else
			i2c_register_board_info(i2c_bus[i], i2c_devs[i], 1);
	}
	intel_scu_notifier_post(SCU_AVAILABLE, 0L);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_create);

/* Called by IPC driver */
void intel_scu_devices_destroy(void)
{
	int i;

	intel_scu_notifier_post(SCU_DOWN, 0L);

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_del(ipc_devs[i]);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_destroy);

static void __init install_irq_resource(struct platform_device *pdev, int irq)
{
	/* Single threaded */
	static struct resource __initdata res = {
		.name = "IRQ",
		.flags = IORESOURCE_IRQ,
	};
	res.start = irq;
	platform_device_add_resources(pdev, &res, 1);
}

static void __init sfi_handle_ipc_dev(struct sfi_device_table_entry *entry)
{
	const struct devs_id *dev = device_ids;
	struct platform_device *pdev;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_IPC &&
			!strncmp(dev->name, entry->name, SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(entry);
			break;
		}
		dev++;
	}

	/*
	 * On Medfield the platform device creation is handled by the MSIC
	 * MFD driver so we don't need to do it here.
	 */
	if (mrst_has_msic())
		return;

	pdev = platform_device_alloc(entry->name, 0);
	if (pdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
			entry->name);
		return;
	}
	install_irq_resource(pdev, entry->irq);

	pdev->dev.platform_data = pdata;
	intel_scu_device_register(pdev);
}

static void __init sfi_handle_spi_dev(struct spi_board_info *spi_info)
{
	const struct devs_id *dev = device_ids;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_SPI &&
				!strncmp(dev->name, spi_info->modalias, SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(spi_info);
			break;
		}
		dev++;
	}
	spi_info->platform_data = pdata;
	if (dev->delay)
		intel_scu_spi_device_register(spi_info);
	else
		spi_register_board_info(spi_info, 1);
}

static void __init sfi_handle_i2c_dev(int bus, struct i2c_board_info *i2c_info)
{
	const struct devs_id *dev = device_ids;
	const struct intel_v4l2_subdev_id *vdev = get_v4l2_ids(NULL);
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_I2C &&
			!strncmp(dev->name, i2c_info->type, SFI_NAME_LEN)) {
			pdata = dev->get_platform_data(i2c_info);
			break;
		}
		dev++;
	}
	i2c_info->platform_data = pdata;

	while (vdev->name[0]) {
		if (!strncmp(vdev->name, i2c_info->type, 16)) {
			intel_ignore_i2c_device_register(bus, i2c_info);
			return;
		}
		vdev++;
	}

	if (dev->delay)
		intel_scu_i2c_device_register(bus, i2c_info);
	else
		i2c_register_board_info(bus, i2c_info, 1);
}

static void sfi_handle_hsi_dev(struct hsi_board_info *hsi_info)
{
	const struct devs_id *dev = device_ids;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_HSI &&
				!strncmp(dev->name, hsi_info->name, 16)) {
			pdata = dev->get_platform_data(hsi_info);
			if (itp_connected && dev->trash_itp)
				return;
			break;
		}
		dev++;
	}

	if (pdata) {
		pr_info("SFI register platform data for HSI device %s\n",
					dev->name);
		hsi_register_board_info(pdata, 2);
	}
}


static void __init sfi_handle_sd_dev(struct sd_board_info *sd_info)
{
	const struct devs_id *dev = device_ids;
	void *pdata = NULL;

	while (dev->name[0]) {
		if (dev->type == SFI_DEV_TYPE_SD &&
			!strncmp(dev->name, sd_info->name, 16)) {
			pdata = dev->get_platform_data(sd_info);
			break;
		}
		dev++;
	}
	sd_info->platform_data = pdata;
}

static int __init sfi_parse_devs(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_device_table_entry *pentry;
	struct spi_board_info spi_info;
	struct i2c_board_info i2c_info;
	struct hsi_board_info hsi_info;
	struct sd_board_info sd_info;
	struct platform_device *pdev;
	int num, i, bus;
	int ioapic;
	struct io_apic_irq_attr irq_attr;

	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_device_table_entry);
	pentry = (struct sfi_device_table_entry *)sb->pentry;

	for (i = 0; i < num; i++, pentry++) {
		int irq = pentry->irq;

		if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW &&
		    (strcmp(pentry->name, "dis71430m") == 0 ||
		     strcmp(pentry->name, "ov2720") == 0)) {
			/* Skip legacy camera entries which do not exist on this
			 * hardware. FIXME: remove as soon as SFI table is
			 * fixed.
			 */
			printk(KERN_ERR "ignoring SFI entry %16.16s\n",
				pentry->name);
			continue;
		}

		if (irq != (u8)0xff) { /* native RTE case */
			/* these SPI2 devices are not exposed to system as PCI
			 * devices, but they have separate RTE entry in IOAPIC
			 * so we have to enable them one by one here
			 */
			ioapic = mp_find_ioapic(irq);
			irq_attr.ioapic = ioapic;
			irq_attr.ioapic_pin = irq;
			irq_attr.trigger = 1;
			irq_attr.polarity = 1;
			io_apic_set_pci_routing(NULL, irq, &irq_attr);
		} else
			irq = 0; /* No irq */

		switch (pentry->type) {
		case SFI_DEV_TYPE_IPC:
			/* ID as IRQ is a hack that will go away */
			pdev = platform_device_alloc(pentry->name, irq);
			if (pdev == NULL) {
				pr_err("out of memory for SFI platform device '%s'.\n",
							pentry->name);
				continue;
			}
			install_irq_resource(pdev, irq);
			pr_debug("info[%2d]: IPC bus, name = %16.16s, "
				"irq = 0x%2x\n", i, pentry->name, irq);
			sfi_handle_ipc_dev(pdev);
			break;
		case SFI_DEV_TYPE_SPI:
			memset(&spi_info, 0, sizeof(spi_info));
			strncpy(spi_info.modalias, pentry->name, SFI_NAME_LEN);
			spi_info.irq = irq;
			spi_info.bus_num = pentry->host_num;
			spi_info.chip_select = pentry->addr;
			spi_info.max_speed_hz = pentry->max_freq;
			pr_debug("info[%2d]: SPI bus = %d, name = %16.16s, "
				"irq = 0x%2x, max_freq = %d, cs = %d\n", i,
				spi_info.bus_num,
				spi_info.modalias,
				spi_info.irq,
				spi_info.max_speed_hz,
				spi_info.chip_select);
			sfi_handle_spi_dev(&spi_info);
			break;
		case SFI_DEV_TYPE_I2C:
			memset(&i2c_info, 0, sizeof(i2c_info));
			bus = pentry->host_num;
			strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
			i2c_info.irq = irq;
			i2c_info.addr = pentry->addr;
			pr_debug("info[%2d]: I2C bus = %d, name = %16.16s, "
				"irq = 0x%2x, addr = 0x%x\n", i, bus,
				i2c_info.type,
				i2c_info.irq,
				i2c_info.addr);

			/* Ignore all sensors info for PR2 and PR3 */
			if (mfld_board_id() == MFLD_BID_PR2_PROTO ||
					mfld_board_id() == MFLD_BID_PR2_PNP ||
					mfld_board_id() == MFLD_BID_PR2_VOLUME ||
					mfld_board_id() == MFLD_BID_PR3 ||
					mfld_board_id() == MFLD_BID_PR3_PNP)
				if (bus == 5)
					break;


			sfi_handle_i2c_dev(bus, &i2c_info);
			break;
		case SFI_DEV_TYPE_SD:
			memset(&sd_info, 0, sizeof(sd_info));
			strncpy(sd_info.name, pentry->name, 16);
			sd_info.bus_num = pentry->host_num;
			sd_info.board_ref_clock = pentry->max_freq;
			sd_info.addr = pentry->addr;
			pr_info("info[%2d]: SDIO bus = %d, name = %16.16s, "
					"ref_clock = %d, addr =0x%x\n", i,
					sd_info.bus_num,
					sd_info.name,
					sd_info.board_ref_clock,
					sd_info.addr);
			sfi_handle_sd_dev(&sd_info);
			break;
		case SFI_DEV_TYPE_HSI:
			memset(&hsi_info, 0, sizeof(hsi_info));
			hsi_info.name = pentry->name;
			hsi_info.hsi_id = pentry->host_num;
			hsi_info.port = pentry->addr;
			pr_info("info[%2d]: HSI bus = %d, name = %16.16s, "
				"port = %d\n", i,
				hsi_info.hsi_id,
				hsi_info.name,
				hsi_info.port);
			sfi_handle_hsi_dev(&hsi_info);
			break;
		case SFI_DEV_TYPE_UART:
		default:
			;
		}
	}
#ifdef CONFIG_LEDS_INTEL_KPD
	pdev = platform_device_alloc("intel_kpd_led", 0);
	if (!pdev)
		pr_err("out of memory for SFI platform dev 'intel_kpd_led'.\n");
	else {
		install_irq_resource(pdev, 0xff);
		pr_info("info[%2d]: IPC bus, name = %16.16s, "
			"irq = 0x%2x\n", i, "intel_kpd_led", pentry->irq);
		intel_scu_device_register(pdev);
	}
#endif
	return 0;
}

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static u32 board_id;
static int board_id_proc_show(struct seq_file *m, void *v)
{
	char *bid;

	switch (board_id) {
	case MFLD_BID_CDK:
		bid = "cdk";        break;
	case MFLD_BID_AAVA:
		bid = "aava";       break;
	case MFLD_BID_PR2_PROTO:
	case MFLD_BID_PR2_PNP:
		bid = "pr2_proto";  break;
	case MFLD_BID_PR2_VOLUME:
		bid = "pr2_volume"; break;
	case MFLD_BID_PR3:
	case MFLD_BID_PR3_PNP:
		bid = "pr3";        break;
	default:
		bid = "unknown";    break;
	}
	seq_printf(m, "boardid=%s\n", bid);

	return 0;
}

static int board_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, board_id_proc_show, NULL);
}

static const struct file_operations board_id_proc_fops = {
	.open		= board_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

u32 mfld_board_id (void)
{
	return board_id;
}
EXPORT_SYMBOL_GPL(mfld_board_id);

static int __init sfi_parse_oemb(struct sfi_table_header *table)
{
	struct sfi_table_oemb *oemb;
	u8 sig[SFI_SIGNATURE_SIZE + 1] = {'\0'};
	u8 oem_id[SFI_OEM_ID_SIZE + 1] = {'\0'};
	u8 oem_table_id[SFI_OEM_TABLE_ID_SIZE + 1] = {'\0'};

	oemb = (struct sfi_table_oemb *) table;
	if (!oemb) {
		pr_err("%s: fail to read MFD Validation SFI OEMB Layout\n",
			__func__);
		return -ENODEV;
	}

	board_id = oemb->board_id | (oemb->board_fab << 4);
	proc_create("boardid", 0, NULL, &board_id_proc_fops);

	snprintf(sig, (SFI_SIGNATURE_SIZE + 1), "%s",
		oemb->header.sig);
	snprintf(oem_id, (SFI_OEM_ID_SIZE + 1), "%s",
		oemb->header.oem_id);
	snprintf(oem_table_id, (SFI_OEM_TABLE_ID_SIZE + 1), "%s",
		oemb->header.oem_table_id);
	pr_info("MFLD Validation SFI OEMB Layout\n");
	pr_info("\tOEMB signature            : %s\n"
		"\tOEMB length               : %d\n"
		"\tOEMB revision             : %d\n"
		"\tOEMB checksum             : 0x%X\n"
		"\tOEMB oem_id               : %s\n"
		"\tOEMB oem_table_id         : %s\n"
		"\tOEMB board_id             : 0x%02X\n"
		"\tOEMB iafw version         : %03d.%03d\n"
		"\tOEMB val_hooks version    : %03d.%03d\n"
		"\tOEMB ia suppfw version    : %03d.%03d\n"
		"\tOEMB scu runtime version  : %03d.%03d\n"
		"\tOEMB ifwi version         : %03d.%03d\n",
		sig,
		oemb->header.len,
		oemb->header.rev,
		oemb->header.csum,
		oem_id,
		oem_table_id,
		board_id,
		oemb->iafw_major_version,
		oemb->iafw_main_version,
		oemb->val_hooks_major_version,
		oemb->val_hooks_minor_version,
		oemb->ia_suppfw_major_version,
		oemb->ia_suppfw_minor_version,
		oemb->scu_runtime_major_version,
		oemb->scu_runtime_minor_version,
		oemb->ifwi_major_version,
		oemb->ifwi_minor_version);
	return 0;
}

#ifdef CONFIG_SWITCH_MID
static struct platform_device switch_device = {
	.name		= "switch-mid",
	.id		= -1,
};
#endif

static int __init intel_mid_platform_init(void)
{
#ifdef CONFIG_SWITCH_MID
	int err;
	err = platform_device_register(&switch_device);
	if (err < 0)
		pr_err("Fail to register switch-mid platform device.\n");
#endif

	/* Get MFD Validation SFI OEMB Layout */
	sfi_table_parse(SFI_SIG_OEMB, NULL, NULL, sfi_parse_oemb);
	/* Battery Check */
	if (sfi_table_parse("OEM0", NULL, NULL, msic_battery_check)) {
	pr_err("Invalid Battery: SFI OEM0 table not found\n");
	is_valid_batt = false;
	}
	sfi_table_parse(SFI_SIG_GPIO, NULL, NULL, sfi_parse_gpio);
	sfi_table_parse(SFI_SIG_DEVS, NULL, NULL, sfi_parse_devs);

	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW) {
		/* Add ov8830 driver for detection
		 * -- FIXME: remove as soon as ov8830 is defined in SFI table */
		sfi_handle_i2c_dev(OV8830_BUS, &ov8830_info);

		/* Add mt9m114 driver for detection
		 * -- FIXME: remove when the sensor is defined in SFI table */
		sfi_handle_i2c_dev(MT9M114_BUS, &mt9m114_info);
	}

	return 0;
}
arch_initcall(intel_mid_platform_init);

/*
 * we will search these buttons in SFI GPIO table (by name)
 * and register them dynamically. Please add all possible
 * buttons here, we will shrink them if no GPIO found.
 */
static struct gpio_keys_button gpio_button[] = {
	{KEY_POWER,		-1, 1, "power_btn",	EV_KEY, 0, 3000},
	{KEY_PROG1,		-1, 1, "prog_btn1",	EV_KEY, 0, 20},
	{KEY_PROG2,		-1, 1, "prog_btn2",	EV_KEY, 0, 20},
	{SW_LID,		-1, 1, "lid_switch",	EV_SW,  0, 20},
	{KEY_VOLUMEUP,		-1, 1, "vol_up",	EV_KEY, 0, 20},
	{KEY_VOLUMEDOWN,	-1, 1, "vol_down",	EV_KEY, 0, 20},
	{KEY_CAMERA,		-1, 1, "camera_full",	EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,	-1, 1, "camera_half",	EV_KEY, 0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw1",	EV_SW,  0, 20},
	{SW_KEYPAD_SLIDE,	-1, 1, "MagSw2",	EV_SW,  0, 20},
	{KEY_CAMERA,		-1, 1, "cam_capture",	EV_KEY, 0, 20},
	{KEY_CAMERA_FOCUS,	-1, 1, "cam_focus",	EV_KEY, 0, 20},
};

static struct gpio_keys_platform_data mrst_gpio_keys = {
	.buttons	= gpio_button,
	.rep		= 1,
	.nbuttons	= -1, /* will fill it after search */
};

static struct platform_device pb_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &mrst_gpio_keys,
	},
};

/* a 3 bits bit-map, from 0 to 7, default 0 */
unsigned char hsu_dma_enable;
EXPORT_SYMBOL_GPL(hsu_dma_enable);

static int __init setup_hsu_dma_enable_flag(char *p)
{
	if (!p)
		return -EINVAL;

	hsu_dma_enable = (unsigned char)memparse(p, &p);
	if (hsu_dma_enable & (~0x7))
		return -EINVAL;

	return 0;
}
early_param("hsu_dma", setup_hsu_dma_enable_flag);

#if defined(CONFIG_TI_ST) || defined(CONFIG_TI_ST_MODULE)

/* KIM related */
static int mrst_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int mrst_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static struct ti_st_plat_data kim_pdata = {
	.nshutdown_gpio	= -1,/* BT, FM, GPS gpios */
	.flow_cntrl	= 1,		/* flow control flag */
	.suspend	= mrst_kim_suspend,
	.resume		= mrst_kim_resume,
};

static struct platform_device linux_kim_device = {
	.name           = "kim", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &kim_pdata,
};

/* BT WILINK related */
static int mrst_bt_enable(struct platform_device *pdev)
{
	return 0;
}
static int mrst_bt_disable(struct platform_device *pdev)
{
	return 0;
}

static struct ti_st_plat_data bt_pdata = {
	.chip_enable    = mrst_bt_enable,
	.chip_disable   = mrst_bt_disable,
};

static struct platform_device linux_bt_device = {
	.name           = "btwilink", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &bt_pdata,
};
static int __init bluetooth_init(void)
{
	unsigned int UART_index;
	long unsigned int UART_baud_rate;
	int error_reg;

	/* KIM INIT */
	/* Get the GPIO number from the SFI table
	   if FM gpio is not provided then BT-reset line is
	   also used to enable FM
	*/
	kim_pdata.nshutdown_gpio = get_gpio_by_name("BT-reset");
	if (kim_pdata.nshutdown_gpio == -1)
		return -ENODEV;

	/* Get Share Transport uart settings */
	/* TODO : add SFI table parsing and one SFI entry for this settings */
	UART_index = 0;
	UART_baud_rate = 3500000;

	/* Share Transport uart settings */
	sprintf((char *)kim_pdata.dev_name, "/dev/ttyMFD%u", UART_index);
	kim_pdata.baud_rate = UART_baud_rate;

	pr_info("%s: Setting platform_data with UART device name:%s and "
			"UART baud rate:%lu.\n",
			__func__, kim_pdata.dev_name, kim_pdata.baud_rate);

	error_reg = platform_device_register(&linux_kim_device);
	if (error_reg < 0) {
		pr_err("platform_device_register for kim failed\n");
		goto exit_on_error;
	}

	/* BT WILINK INIT */
	error_reg = platform_device_register(&linux_bt_device);
	if (error_reg < 0)
		pr_err("platform_device_register for btwilink failed\n");
exit_on_error:
	return error_reg;

}
device_initcall(bluetooth_init);

#endif

void *cloverview_usb_otg_get_pdata(void)
{
	struct cloverview_usb_otg_pdata *pdata;

	if (__intel_mid_cpu_chip != INTEL_MID_CPU_CHIP_CLOVERVIEW)
		return NULL;

	pdata = (struct cloverview_usb_otg_pdata *)
				kmalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: out of memory.\n", __func__);
		goto failed1;
	}
	pdata->gpio_cs = get_gpio_by_name("usb_otg_phy_cs");
	if (pdata->gpio_cs == -1) {
		pr_err("%s: No gpio pin for 'usb_otg_phy_cs'\n", __func__);
		goto failed2;
	}
	pdata->gpio_reset = get_gpio_by_name("usb_otg_phy_reset");
	if (pdata->gpio_reset == -1) {
		pr_err("%s: No gpio pin for 'usb_otg_phy_reset'\n", __func__);
		goto failed2;
	}
	pr_info("%s: CS pin: gpio %d, Reset pin: gpio %d\n", __func__,
			 pdata->gpio_cs, pdata->gpio_reset);
	return pdata;

failed2:
	kfree(pdata);
failed1:
	return NULL;
}
EXPORT_SYMBOL_GPL(cloverview_usb_otg_get_pdata);

/*
 * Shrink the non-existent buttons, register the gpio button
 * device if there is some
 */
static int __init pb_keys_init(void)
{
	struct gpio_keys_button *gb = gpio_button;
	int i, num, good = 0;

	num = sizeof(gpio_button) / sizeof(struct gpio_keys_button);
	for (i = 0; i < num; i++) {
		gb[i].gpio = get_gpio_by_name(gb[i].desc);
		if (gb[i].gpio == -1)
			continue;

		if (i != good)
			gb[good] = gb[i];
		good++;
	}

	if (good) {
		mrst_gpio_keys.nbuttons = good;
		return platform_device_register(&pb_device);
	}
	return 0;
}
late_initcall(pb_keys_init);
#define HSU0_CTS (13)
#define HSU0_RTS (96 + 29)
#define HSU1_RX (64)
#define HSU1_TX (65)
#define HSU1_CTS (68)
#define HSU1_RTS (66)
#define HSU1_ALT_RX (96 + 30)
#define HSU1_ALT_TX (96 + 31)
#define HSU2_RX (67)

/* on = 1: the port1 is muxed (named as port 3) for debug output
 * on = 0: the port1 is for modem fw download.
 */
void mfld_hsu_port1_switch(int on)
{
	static int first = 1;

	if (unlikely(first)) {
		gpio_request(HSU1_RX, "hsu");
		gpio_request(HSU1_TX, "hsu");
		gpio_request(HSU1_CTS, "hsu");
		gpio_request(HSU1_RTS, "hsu");
		gpio_request(HSU1_ALT_RX, "hsu");
		gpio_request(HSU1_ALT_TX, "hsu");
		first = 0;
	}
	if (on) {
		lnw_gpio_set_alt(HSU1_RX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_TX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_CTS, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_RTS, LNW_GPIO);
		gpio_direction_input(HSU1_RX);
		gpio_direction_input(HSU1_TX);
		gpio_direction_input(HSU1_CTS);
		gpio_direction_input(HSU1_RTS);
		gpio_direction_input(HSU1_ALT_RX);
		gpio_direction_output(HSU1_ALT_TX, 0);
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_ALT_TX, LNW_ALT_1);
	} else {
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_GPIO);
		lnw_gpio_set_alt(HSU1_ALT_TX, LNW_GPIO);
		gpio_direction_input(HSU1_ALT_RX);
		gpio_direction_input(HSU1_ALT_TX);
		gpio_direction_input(HSU1_RX);
		gpio_direction_output(HSU1_TX, 0);
		gpio_direction_input(HSU1_CTS);
		gpio_direction_output(HSU1_RTS, 0);
		lnw_gpio_set_alt(HSU1_RX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_TX, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_CTS, LNW_ALT_1);
		lnw_gpio_set_alt(HSU1_RTS, LNW_ALT_2);
	}
}
EXPORT_SYMBOL_GPL(mfld_hsu_port1_switch);

void mfld_hsu_enable_wakeup(int index, struct device *dev, irq_handler_t wakeup)
{
	int ret;

	switch (index) {
	case 0:
		lnw_gpio_set_alt(HSU0_CTS, LNW_GPIO);
		ret = request_irq(gpio_to_irq(HSU0_CTS), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu0_cts_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu0: failed to register wakeup irq\n");

		/* turn off flow control */
		gpio_set_value(HSU0_RTS, 1);
		lnw_gpio_set_alt(HSU0_RTS, LNW_GPIO);
		udelay(100);
		break;
	case 1:
		lnw_gpio_set_alt(HSU1_RX, LNW_GPIO);
		udelay(100);
		ret = request_irq(gpio_to_irq(HSU1_RX), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu1_rx_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu1: failed to register wakeup irq\n");
		break;
	case 2:
		break;
	case 3:
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_GPIO);
		udelay(100);
		ret = request_irq(gpio_to_irq(HSU1_ALT_RX), wakeup,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				"hsu1_rx_wakeup", dev);
		if (ret)
			dev_err(dev, "hsu1: failed to register wakeup irq\n");
		break;
	default:
		dev_err(dev, "hsu: unknow hsu port\n");
	}
}
EXPORT_SYMBOL_GPL(mfld_hsu_enable_wakeup);

void mfld_hsu_disable_wakeup(int index, struct device *dev)
{
	switch (index) {
	case 0:
		free_irq(gpio_to_irq(HSU0_CTS), dev);
		lnw_gpio_set_alt(HSU0_CTS, LNW_ALT_1);
		lnw_gpio_set_alt(HSU0_RTS, LNW_ALT_1);
		break;
	case 1:
		free_irq(gpio_to_irq(HSU1_RX), dev);
		lnw_gpio_set_alt(HSU1_RX, LNW_ALT_1);
		break;
	case 2:
		break;
	case 3:
		free_irq(gpio_to_irq(HSU1_ALT_RX), dev);
		lnw_gpio_set_alt(HSU1_ALT_RX, LNW_ALT_1);
		break;
	default:
		dev_err(dev, "hsu: unknow hsu port\n");
	}
}
EXPORT_SYMBOL_GPL(mfld_hsu_disable_wakeup);

#define EMMC_BLK_NAME	"mmcblk0"
static int emmc_match(struct device *dev, void *data)
{
	if (strcmp(dev_name(dev), "mmcblk0") == 0)
		return 1;
	return 0;
}
int mmc_blk_rpmb_req_handle(struct mmc_rpmb_req *req)
{
	struct device *emmc = NULL;

	if (!req)
		return -EINVAL;

	emmc = class_find_device(&block_class, NULL, EMMC_BLK_NAME, emmc_match);
	if (!emmc) {
		pr_err("%s: eMMC card is not registered yet. Try it later\n",
				__func__);
		return -ENODEV;
	}

	return mmc_rpmb_req_handle(emmc, req);
}
EXPORT_SYMBOL_GPL(mmc_blk_rpmb_req_handle);
