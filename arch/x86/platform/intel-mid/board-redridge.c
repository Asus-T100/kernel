/*
 * board-redridge.c: Intel Medfield based board (Redridge)
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/spi/spi.h>
#include <linux/cyttsp.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/nfc/pn544.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/ipc_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/intel_mid_pm.h>
#include <linux/usb/penwell_otg.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/wl12xx.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/a1026.h>
#include <linux/input/lis3dh.h>
#include <linux/ms5607.h>
#include <linux/i2c-gpio.h>
#include <linux/rmi_i2c.h>
#include <linux/i2c/tc35876x.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c/atmel_mxt_ts.h>

#include <linux/atomisp_platform.h>
#include <media/v4l2-subdev.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/blkdev.h>

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

/* the offset for the mapping of global gpio pin to irq */
#define MRST_IRQ_OFFSET 0x100

enum ipc_dev_type {
	IPC_DEV_PMIC_GPIO,
	IPC_DEV_PMIC_AUDIO,
	IPC_DEV_MSIC_ADC,
	IPC_DEV_MSIC_BATTERY,
	IPC_DEV_MSIC_GPIO,
	IPC_DEV_MSIC_AUDIO,
	IPC_DEV_MSIC_POWER_BTN,
	IPC_DEV_MSIC_OCD,

	IPC_DEV_NUM,
};

static __initdata enum ipc_dev_type current_ipcdev;
static __initdata DEFINE_MUTEX(ipc_dev_lock);

static struct resource pmic_gpio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pmic_audio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msic_adc_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msic_battery_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msic_gpio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msic_audio_resources[] __initdata = {
	{
		.name  = "IRQ",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name  = "IRQ_BASE",
		.flags = IORESOURCE_MEM,
		.start = MSIC_IRQ_STATUS_OCAUDIO,
		.end   = MSIC_IRQ_STATUS_ACCDET,
	},
};

static struct resource msic_power_btn_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource msic_ocd_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

struct intel_ipc_dev_res {
	const char *name;
	int num_resources;
	struct resource *resources;
};

static struct intel_ipc_dev_res ipc_dev_res[IPC_DEV_NUM] __initdata = {
	[IPC_DEV_PMIC_GPIO]             = {
		.name                   = "pmic_gpio",
		.num_resources          = ARRAY_SIZE(pmic_gpio_resources),
		.resources              = pmic_gpio_resources,
	},
	[IPC_DEV_PMIC_AUDIO]            = {
		.name                   = "pmic_audio",
		.num_resources          = ARRAY_SIZE(pmic_audio_resources),
		.resources              = pmic_audio_resources,
	},
	[IPC_DEV_MSIC_ADC]              = {
		.name                   = "msic_adc",
		.num_resources          = ARRAY_SIZE(msic_adc_resources),
		.resources              = msic_adc_resources,
	},
	[IPC_DEV_MSIC_BATTERY]          = {
		.name                   = "msic_battery",
		.num_resources          = ARRAY_SIZE(msic_battery_resources),
		.resources              = msic_battery_resources,
	},
	[IPC_DEV_MSIC_GPIO]             = {
		.name                   = "msic_gpio",
		.num_resources          = ARRAY_SIZE(msic_gpio_resources),
		.resources              = msic_gpio_resources,
	},
	[IPC_DEV_MSIC_AUDIO]            = {
		.name                   = "msic_audio",
		.num_resources          = ARRAY_SIZE(msic_audio_resources),
		.resources              = msic_audio_resources,
	},
	[IPC_DEV_MSIC_POWER_BTN]        = {
		.name                   = "msic_power_btn",
		.num_resources          = ARRAY_SIZE(msic_power_btn_resources),
		.resources              = msic_power_btn_resources,
	},
	[IPC_DEV_MSIC_OCD]              = {
		.name                   = "msic_ocd",
		.num_resources          = ARRAY_SIZE(msic_ocd_resources),
		.resources              = msic_ocd_resources,
	},
};

/* this should be called with the holding of ipc_dev_lock */
static void handle_ipc_irq_res(enum ipc_dev_type type, int irq)
{
	struct resource *res = &ipc_dev_res[type].resources[0];

	if (res->flags & IORESOURCE_IRQ)
		res->start = irq;

	current_ipcdev = type;
}

static void __init *pmic_gpio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_pmic_gpio_platform_data pmic_gpio_pdata;
	int gpio_base = get_gpio_by_name("pmic_gpio_base");

	if (gpio_base == -1)
		gpio_base = 64;
	pmic_gpio_pdata.gpio_base = gpio_base;
	pmic_gpio_pdata.irq_base = gpio_base + MRST_IRQ_OFFSET;
	pmic_gpio_pdata.gpiointr = 0xffffeff8;

	handle_ipc_irq_res(IPC_DEV_PMIC_GPIO, entry->irq);

	return &pmic_gpio_pdata;
}

static void __init *pmic_audio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	handle_ipc_irq_res(IPC_DEV_PMIC_AUDIO, entry->irq);
	return NULL;
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

static void *msic_adc_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_mid_gpadc_platform_data pdata;
	pdata.intr = 0xffff7fc0;

	handle_ipc_irq_res(IPC_DEV_MSIC_ADC, entry->irq);

	return &pdata;
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
	struct sfi_device_table_entry *entry = info;
	handle_ipc_irq_res(IPC_DEV_MSIC_BATTERY, entry->irq);
	return NULL;
}

static void *msic_gpio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_msic_gpio_pdata pdata;

	int gpio = get_gpio_by_name("msic_gpio_base");
	if (gpio < 0)
		return NULL;

	pdata.gpio_base = gpio;
	handle_ipc_irq_res(IPC_DEV_MSIC_GPIO, entry->irq);

	return &pdata;
}

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C-1 to recover devices from abnormal status.
 * currently, only max17042 on I2C-1 needs such workaround */
#define I2C_1_GPIO_PIN 27
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_GPIO);
	gpio_direction_output(I2C_1_GPIO_PIN, 0);
	gpio_set_value(I2C_1_GPIO_PIN, 1);
	udelay(10);
	gpio_set_value(I2C_1_GPIO_PIN, 0);
	udelay(10);
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_ALT_1);
#undef I2C_1_GPIO_PIN 27
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

static bool msic_battery_check(void)
{
	if (get_oem0_table() == NULL) {
		pr_info("invalid battery detected\n");
		return false;
	} else {
		pr_info("valid battery detected\n");
		return true;
	}
	return false;
}


static void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	int intr = get_gpio_by_name("max17042");

	i2c_info->irq = intr + MRST_IRQ_OFFSET;

	if (msic_battery_check()) {
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
	int ret;
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;

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

	pdev = platform_device_alloc("sn95031", -1);
	if (!pdev) {
		pr_err("failed to allocate sn95031 platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add sn95031 platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	if (strncmp(entry->name, "msic_audio", 16) == 0)
		handle_ipc_irq_res(IPC_DEV_MSIC_AUDIO, entry->irq);

	return NULL;
}

static void *msic_power_btn_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	handle_ipc_irq_res(IPC_DEV_MSIC_POWER_BTN, entry->irq);
	return NULL;
}

static void *msic_ocd_platform_data(void *info)
{
	int gpio;
	static struct intel_msic_ocd_pdata pdata;

	gpio = get_gpio_by_name("ocd_gpio");
	if (gpio < 0)
		return NULL;

	pdata.gpio = gpio;

	handle_ipc_irq_res(IPC_DEV_MSIC_OCD, gpio + MRST_IRQ_OFFSET);

	return &pdata;
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

static void *mt9e013_platform_data_init(void *info)
{
	gp_camera0_reset = -1;
	gp_camera0_power_down = -1;

	return &mt9e013_sensor_platform_data;
}

/*
 * MFLD PR2 secondary camera sensor - MT9M114 platform data
 */
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

static void *mt9m114_platform_data_init(void *info)
{
	gp_camera1_reset = -1;
	gp_camera1_power_down = -1;

	return &mt9m114_sensor_platform_data;
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

#define TOUCH_RESET_GPIO 129
#define TOUCH_IRQ_GPIO   62

/* Atmel mxt toucscreen platform setup*/
static int atmel_mxt_init_platform_hw(void)
{
	int rc;
	int reset_gpio, int_gpio;

	reset_gpio = TOUCH_RESET_GPIO;
	int_gpio = TOUCH_IRQ_GPIO;

	/* init interrupt gpio */
	rc = gpio_request(int_gpio, "mxt_ts_intr");
	if (rc < 0)
		return rc;

	rc = gpio_direction_input(int_gpio);
	if (rc < 0)
		goto err_int;

	/* init reset gpio */
	rc = gpio_request(reset_gpio, "mxt_ts_rst");
	if (rc < 0)
		goto err_int;

	rc = gpio_direction_output(reset_gpio, 1);
	if (rc < 0)
		goto err_reset;

	/* reset the chip */
	gpio_set_value(reset_gpio, 1);
	msleep(20);
	gpio_set_value(reset_gpio, 0);
	msleep(20);
	gpio_set_value(reset_gpio, 1);
	msleep(100);

	return 0;

err_reset:
	gpio_free(reset_gpio);
err_int:
	pr_err("mxt touchscreen: configuring reset or int gpio failed\n");
	gpio_free(int_gpio);

	return rc;
}

void *mxt_platform_data_init(void *info)
{
	struct i2c_board_info *i2c_info = info;
	static struct mxt_platform_data mxt_pdata = {
		.irqflags	= IRQF_TRIGGER_FALLING,
		.init_platform_hw = atmel_mxt_init_platform_hw,
	};

	i2c_info->irq = TOUCH_IRQ_GPIO + MRST_IRQ_OFFSET;
	return &mxt_pdata;
}

#define AUDIENCE_WAKEUP_GPIO               "audience-wakeup"
#define AUDIENCE_RESET_GPIO                 "audience-reset"
static int audience_request_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		client->dev.platform_data;
	int ret;

	pr_debug("Audience: request ressource audience\n");
	if (!pdata)
		return -1;
	ret = gpio_request(pdata->gpio_a1026_wakeup, AUDIENCE_WAKEUP_GPIO);
	if (ret) {
		dev_err(&client->dev, "Request AUDIENCE WAKEUP GPIO %d fails %d\n",
			pdata->gpio_a1026_wakeup, ret);
		return -1;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_wakeup, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_wake;
	}

	ret = gpio_request(pdata->gpio_a1026_reset, AUDIENCE_RESET_GPIO);
	if (ret) {
		dev_err(&client->dev,
				"Request for Audience reset GPIO %d fails %d\n",
					pdata->gpio_a1026_reset, ret);
		goto err_wake;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_reset, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_reset;
	}
	sprintf(pdata->firmware_name, "%s", "vpimg_es305b.bin");

	return 0;
err_reset:
	gpio_free(pdata->gpio_a1026_reset);
err_wake:
	gpio_free(pdata->gpio_a1026_wakeup);
	return -1;
}

static void audience_free_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		&client->dev.platform_data;

	gpio_free(pdata->gpio_a1026_wakeup);
	gpio_free(pdata->gpio_a1026_reset);
}

static void audience_wake_up(bool state)
{
	int wakeup_gpio;

	wakeup_gpio = get_gpio_by_name(AUDIENCE_WAKEUP_GPIO);
	if (wakeup_gpio == -1) {
		pr_err("%s invalid wakeup gpio", __func__);
		return;
	}
	gpio_set_value(wakeup_gpio, state);
	pr_debug("Audience: WAKE UP %d\n", state);
}

static void audience_reset(bool state)
{
	int reset_gpio;

	reset_gpio = get_gpio_by_name(AUDIENCE_RESET_GPIO);
	if (reset_gpio == -1) {
		pr_err("%s invalid reset gpio", __func__);
		return;
	}
	gpio_set_value(reset_gpio, state);
	pr_debug("Audience: RESET %d\n", state);
}

void *audience_platform_data_init(void *info)
{
	static struct a1026_platform_data pdata;

	pdata.gpio_a1026_wakeup = get_gpio_by_name(AUDIENCE_WAKEUP_GPIO);
	pdata.gpio_a1026_reset = get_gpio_by_name(AUDIENCE_RESET_GPIO);
	pdata.request_resources	= audience_request_resources;
	pdata.free_resources	= audience_free_resources;
	pdata.wakeup			= audience_wake_up;
	pdata.reset			= audience_reset;

	return &pdata;
}

void *lis3dh_pdata_init(void *info)
{
	static struct lis3dh_acc_platform_data lis3dh_pdata;

	lis3dh_pdata.poll_interval = 200;
	lis3dh_pdata.negate_x = 1;
	lis3dh_pdata.negate_y = 0;
	lis3dh_pdata.negate_z = 0;
	lis3dh_pdata.axis_map_x = 0;
	lis3dh_pdata.axis_map_y = 1;
	lis3dh_pdata.axis_map_z = 2;
	lis3dh_pdata.gpio_int1 = 60;
	lis3dh_pdata.gpio_int2 = 61;

	return &lis3dh_pdata;

}

void *ms5607_platform_data_init(void *info)
{
	static struct ms5607_platform_data baro_pdata;

	baro_pdata.poll_interval = 100;
	baro_pdata.min_interval  = 0;

	return &baro_pdata;
};

void *gyro_pdata_init(void *info)
{
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	i2c_info->irq = get_gpio_by_name("gyro_int");

	return NULL;
}

void *bara_pdata_init(void *info)
{
	static struct ms5607_platform_data baro_pdata;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	i2c_info->irq = 0xff;

	baro_pdata.poll_interval = 100;
	baro_pdata.min_interval  = 0;

	return &baro_pdata;
}

void *als_pdata_init(void *info)
{
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	i2c_info->irq = get_gpio_by_name("AL-intr");

	return NULL;
}

static const struct intel_v4l2_subdev_id v4l2_ids_mfld[] = {
	{"mt9e013", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3554", LED_FLASH, -1},
	{},
};
static const struct intel_v4l2_subdev_id v4l2_ids_clv[] = {
	{"ov8830", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
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

void intel_ignore_i2c_device_register(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct i2c_board_info i2c_info;
	struct i2c_board_info *idev = &i2c_info;
	int bus = pentry->host_num;
	void *pdata = NULL;
	int n_subdev;
	const struct intel_v4l2_subdev_id *vdev = get_v4l2_ids(&n_subdev);
	struct intel_v4l2_subdev_i2c_board_info *info;
	static struct intel_v4l2_subdev_table *subdev_table;
	enum intel_v4l2_subdev_type type = 0;
	enum atomisp_camera_port port;
	static int i;

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
	i2c_info.irq = ((pentry->irq == (u8)0xff) ? 0 : pentry->irq);
	i2c_info.addr = pentry->addr;
	pr_info("I2C bus = %d, name = %16.16s, "
		"irq = 0x%2x, addr = 0x%x\n",
		pentry->host_num,
		i2c_info.type,
		i2c_info.irq,
		i2c_info.addr);
	pdata = dev->get_platform_data(&i2c_info);
	i2c_info.platform_data = pdata;


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

void blackbay_ipc_device_handler(struct sfi_device_table_entry *pentry,
		struct devs_id *dev)
{
	int res_num;
	struct resource *res;
	struct ipc_device *ipcdev;
	void *pdata = NULL;

	pr_info("IPC bus = %d, name = %16.16s, "
		"irq = 0x%2x\n", pentry->host_num, pentry->name, pentry->irq);

	mutex_lock(&ipc_dev_lock);

	pdata = dev->get_platform_data(pentry);

	ipcdev = ipc_device_alloc(pentry->name, -1);
	if (ipcdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
				pentry->name);
		return;
	}

	res = ipc_dev_res[current_ipcdev].resources;
	res_num = ipc_dev_res[current_ipcdev].num_resources;
	ipc_device_add_resources(ipcdev, res, res_num);

	ipcdev->dev.platform_data = pdata;
	ipc_device_add_to_list(ipcdev);

	mutex_unlock(&ipc_dev_lock);
}

/*
 * CLV PR0 primary camera sensor - OV8830 platform data
 */

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
/*tc35876x DSI_LVDS bridge chip and panel platform data*/
static void *tc35876x_platform_data(void *data)
{
	static struct tc35876x_platform_data pdata;
	pdata.gpio_bridge_reset = get_gpio_by_name("LCMB_RXEN");
	pdata.gpio_panel_bl_en = get_gpio_by_name("6S6P_BL_EN");
	pdata.gpio_panel_vadd = get_gpio_by_name("EN_VREG_LCD_V3P3");
	return &pdata;
}

struct devs_id __initconst device_ids[] = {
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&blackbay_ipc_device_handler},
	{"cy8ctma340", SFI_DEV_TYPE_I2C, 1, &cyttsp_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &pmic_audio_platform_data,
					&blackbay_ipc_device_handler},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"ektf2136_spi", SFI_DEV_TYPE_SPI, 0, &ektf2136_spi_platform_data,
						NULL},
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&blackbay_ipc_device_handler},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data_init,
						NULL},
	/* MSIC subdevices */
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&blackbay_ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&blackbay_ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&blackbay_ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&blackbay_ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&blackbay_ipc_device_handler},

	/*
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &no_platform_data,
					&intel_ignore_i2c_device_register},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data_init,
					&intel_ignore_i2c_device_register},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data_init,
					&intel_ignore_i2c_device_register},
	{"mxt1386", SFI_DEV_TYPE_I2C, 0, &mxt_platform_data_init, NULL},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data_init,
						NULL},
	{"accel", SFI_DEV_TYPE_I2C, 0, &lis3dh_pdata_init, NULL},
	{"gyro", SFI_DEV_TYPE_I2C, 0, &gyro_pdata_init, NULL},
	{"baro", SFI_DEV_TYPE_I2C, 0, &bara_pdata_init, NULL},
	{"als", SFI_DEV_TYPE_I2C, 0, &als_pdata_init, NULL},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data_init},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &s3202_platform_data_init},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data},

	{},
};

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
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

u32 mfld_board_id(void)
{
	return board_id;
}
EXPORT_SYMBOL_GPL(mfld_board_id);

int __init board_proc_init(void)
{
	proc_create("boardid", 0, NULL, &board_id_proc_fops);
	return 0;
}

early_initcall(board_proc_init);

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

#ifdef CONFIG_SWITCH_MID
static struct platform_device switch_device = {
	.name		= "switch-mid",
	.id		= -1,
};
#endif


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

#define EMMC_BLK_NAME	"mmcblk0rpmb"
static int emmc_match(struct device *dev, void *data)
{
	if (strcmp(dev_name(dev), data) == 0)
		return 1;
	return 0;
}
int mmc_blk_rpmb_req_handle(struct mmc_ioc_rpmb_req *req)
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

static int hdmi_i2c_workaround(void)
{
	int ret;
	struct platform_device *pdev;
	struct i2c_gpio_platform_data *pdata;

	/*
	 * Hard code a gpio controller platform device to take over
	 * the two gpio pins used to be controlled by i2c bus 3.
	 * This is to support HDMI EDID extension block read, which
	 * is not supported by the current i2c controller, so we use
	 * GPIO pin the simulate an i2c bus.
	 */
	pdev = platform_device_alloc("i2c-gpio", 8);
	if (!pdev) {
		pr_err("i2c-gpio: failed to alloc platform device\n");
		ret = -ENOMEM;
		goto out;
	}

	pdata = kzalloc(sizeof(struct i2c_gpio_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("i2c-gpio: failed to alloc platform data\n");
		kfree(pdev);
		ret = -ENOMEM;
		goto out;
	}
	pdata->scl_pin = 35 + 96;
	pdata->sda_pin = 36 + 96;
	pdata->sda_is_open_drain = 0;
	pdata->scl_is_open_drain = 0;
	pdev->dev.platform_data = pdata;

	platform_device_add(pdev);

	lnw_gpio_set_alt(pdata->sda_pin, LNW_GPIO);
	lnw_gpio_set_alt(pdata->scl_pin, LNW_GPIO);

out:
	return ret;
}
rootfs_initcall(hdmi_i2c_workaround);

#ifdef CONFIG_LEDS_INTEL_KPD
static int __init intel_kpd_led_init(void)
{
	int ret;
	struct ipc_board_info board_info;

	memset(&board_info, 0, sizeof(board_info));
	strncpy(board_info.name, "intel_kpd_led", 16);
	board_info.bus_id = IPC_SCU;
	board_info.id = -1;

	ret = ipc_new_device(&board_info);
	if (ret) {
		pr_err("failed to create ipc device: intel_kpd_led\n");
		return -1;
	}

	return 0;
}
fs_initcall(intel_kpd_led_init);
#endif



#ifdef CONFIG_SWITCH_MID
static int __init switch_mid_init(void)
{
	int err;
	err = platform_device_register(&switch_device);
	if (err < 0)
		pr_err("Fail to register switch-mid platform device.\n");
	return 0;
}
device_initcall(switch_mid_init);
#endif

#define MT9M114_I2C_ADDR (0x90 >> 1)	/* i2c address, 0x90 or 0xBA */
#define MT9M114_BUS      4		/* i2c bus number */

struct sfi_device_table_entry mt9m114_pentry = {
	.name		=	"mt9m114",
	.host_num	=	MT9M114_BUS,
	.irq		=	255,
	.addr		=	MT9M114_I2C_ADDR,
};
#define OV8830_I2C_ADDR	(0x6C >> 1)	/* i2c address, 0x20 or 0x6C */
#define OV8830_BUS	4		/* i2c bus number */

struct sfi_device_table_entry ov8830_pentry = {
	.name		=	"ov8830",
	.host_num	=	OV8830_BUS,
	.irq		=	255,
	.addr		=	OV8830_I2C_ADDR,
};
static int __init blackbay_i2c_init(void)
{
	struct devs_id *dev = NULL;
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW) {
		/* Add ov8830 driver for detection
		 * -- FIXME: remove as soon as ov8830 is defined in SFI table */
		dev = get_device_id(SFI_DEV_TYPE_I2C, "ov8830");
		if (dev != NULL)
			intel_ignore_i2c_device_register(&ov8830_pentry, dev);
		else
			pr_err("Dev id is NULL for %s\n", "ov8830");

		/* Add mt9m114 driver for detection
		 * -- FIXME: remove when the sensor is defined in SFI table */
		dev = get_device_id(SFI_DEV_TYPE_I2C, "mt9m114");
		if (dev != NULL)
			intel_ignore_i2c_device_register(&mt9m114_pentry, dev);
		else
			pr_err("Dev id is NULL for %s\n", "mt9m114");
	}
	return 0;
}
device_initcall(blackbay_i2c_init);

/*
 * mxt1386 initialization routines for Redridge board
 * (Should be removed once SFI tables are updated)
 */
static struct mxt_platform_data mxt1386_pdata = {
	.irqflags	= IRQF_TRIGGER_FALLING,
	.init_platform_hw = atmel_mxt_init_platform_hw,

};
static struct i2c_board_info dv10_i2c_bus0_devs[] = {
	{
		.type       = "mxt1386",
		.addr       = 0x4c,
		.irq	    = TOUCH_IRQ_GPIO + MRST_IRQ_OFFSET,
		.platform_data = &mxt1386_pdata,
	},
};
static int __init redridge_i2c_init(void)
{
       i2c_register_board_info(0, dv10_i2c_bus0_devs,
                               ARRAY_SIZE(dv10_i2c_bus0_devs));
       return 0;
}
device_initcall(redridge_i2c_init);
