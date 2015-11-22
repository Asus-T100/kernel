/*
 * platform_m10m0.c: m10m0 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_flis.h>
#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif
#ifdef CONFIG_ACPI
#include <linux/acpi_gpio.h>
#endif
#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/interrupt.h>
#ifdef CONFIG_CRYSTAL_COVE
#include <linux/mfd/intel_mid_pmic.h>
#endif
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/m10mo_atomisp.h>
#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include "platform_camera.h"
#include "platform_m10mo.h"

#define OSC_CAM_CLK 0x0
#define CLK_19P2MHz 0x0
#define CLK_19P2MHz 0x1
/* workaround - use xtal for cht */
#define CLK_19P2MHz_XTAL 0x0

#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x5d
#define VPROG_1P2V 0x5a
#define VPROG_ENABLE 0x63
#define VPROG_DISABLE 0x62
#endif

static int camera_reset = -1;
static void setup_m10mo_spi(struct m10mo_atomisp_spi_platform_data *spi_pdata,
			    void *data);
int m10mo_platform_identify_fw(void);

static struct atomisp_camera_caps m10mo_camera_caps;

static int intr_gpio = -1;		/* m10mo interrupt pin */
/*
 * Configure these based on the board. Currently we don't get these
 * from BIOS / IAFW
 */
static int cs_chip_select = -1;

static enum atomisp_input_format input_format = ATOMISP_INPUT_FORMAT_YUV422_8;

/*
 * Ext-ISP m10mo platform data
 */
static int m10mo_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
#ifndef CONFIG_ACPI
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_reset = ret;
#else
		camera_reset = acpi_get_gpio("\\_SB.GPO1", 52);
		pr_info("%s: camera_reset is %d\n", __func__, camera_reset);
		ret = gpio_request(camera_reset, GP_CAMERA_0_RESET);
		if (ret) {
			pr_err("%s: failed to request reset pin\n", __func__);
			return -EINVAL;
		}

		ret = gpio_direction_output(camera_reset, 1);
		if (ret) {
			pr_err("%s: failed to set direction for reset pin\n",
				__func__);
			gpio_free(camera_reset);
		}
#endif
	}

	if (flag) {
		/* reset = 0 is requied in case of ESD failure */
		gpio_set_value(camera_reset, 0);

		usleep_range(100, 200);

		gpio_set_value(camera_reset, 1);

		usleep_range(1000, 1500);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int m10mo_gpio_intr_ctrl(struct v4l2_subdev *sd)
{
#ifdef CONFIG_INTEL_SCU_FLIS
	/* This should be done in pin cfg XML and not here */
	config_pin_flis(ann_gp_camerasb_3, PULL, DOWN_50K);
	config_pin_flis(ann_gp_camerasb_3, MUX, MUX_EN_INPUT_EN | INPUT_EN);
#endif

	if (intr_gpio >= 0)
		return intr_gpio;

	return -EINVAL;
}

static int m10mo_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_INTEL_SOC_PMC
	int ret;
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM_CLK, (IS_CHT) ?
			CLK_19P2MHz_XTAL : CLK_19P2MHz);
		if (ret)
			return ret;
		ret = pmc_pc_configure(OSC_CAM_CLK, 1);
	} else {
		ret = pmc_pc_configure(OSC_CAM_CLK, 2);
	}
	pr_info("M10MO clock control. flag=%d ret=%d\n", flag, ret);
	return ret;
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#else
	pr_err("clock is not set.\n");
	return 0;
#endif
}

static int m10mo_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	pr_info("M10MO power control. flag=%d\n", flag);
#ifdef CONFIG_CRYSTAL_COVE
	if (flag) {
		ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_ENABLE);
		if (ret) {
			pr_err("Failed to power on V2P8SX.\n");
			return ret;
		}
		ret = intel_mid_pmic_writeb(VPROG_1P2V, VPROG_ENABLE);
		if (ret) {
			pr_err("Failed to power on V1P2SX.\n");
			/* Turn all powers off if one is failed. */
			intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE);
			return ret;
		}
		/* Wait for 8ms to make all the power supplies to be stable. */
		usleep_range(8000, 8000);
	} else {
		/* Turn all powers off even when some are failed. */
		if (intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE))
			pr_err("Failed to power off V2P8SX.\n");
		if (intel_mid_pmic_writeb(VPROG_1P2V, VPROG_DISABLE))
			pr_err("Failed to power off V1P2SX.\n");
	}
#endif
	return ret;
}

static int m10mo_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		input_format, 0, flag);
}

static struct m10mo_fw_id fw_ids[] = {
	{ "TEST",        M10MO_FW_TYPE_0 },
	{ "S13F0SAHE01", M10MO_FW_TYPE_3 },
	{ "O13F0SAHE01", M10MO_FW_TYPE_3 },
	{ "O13F0SAHF02", M10MO_FW_TYPE_4 },
	{ "S13F0SAHF01", M10MO_FW_TYPE_3 },
	{ "S13F0SAHF02", M10MO_FW_TYPE_4 },
	{ "O13F0SAHF01", M10MO_FW_TYPE_3 },
	{ "T13F0SAHE01", M10MO_FW_TYPE_1 },
	{ "T13F0SAHF01", M10MO_FW_TYPE_1 },
	{ "T13F0SAHF02", M10MO_FW_TYPE_1 },
	{ "T13F0SAHF03", M10MO_FW_TYPE_1 },
	{ "T13F0SAHF04", M10MO_FW_TYPE_1 },
	{ "T13F0SAHF05", M10MO_FW_TYPE_5 },
	{ "T13F0SAHF06", M10MO_FW_TYPE_5 },
	{ NULL, 0},
};

static u32 clock_rate[] = {
	24000000,
	19200000,
};

static u32 mipi_packet_size[] = {
	2048,
	4096,
};

static void spi_hw_resources_setup(struct m10mo_atomisp_spi_platform_data *pdata)
{
	/* Setup SPI interface */
	if (gpio_request(pdata->spi_cs_gpio, "m10mo_spi_cs")) {
		pr_err("Can't allocate gpio for m10mo spi chip select.\n");
		pr_err("Disabling FW update over the SPI\n");
		pdata->spi_enabled = false;
		return;
	}

	cs_chip_select = pdata->spi_cs_gpio;
	lnw_gpio_set_alt(cs_chip_select, LNW_GPIO);
	gpio_direction_output(cs_chip_select, 0);

#ifdef CONFIG_INTEL_SCU_FLIS
	/* Setup flis configuration if requested to do so */
	if (pdata->spi_clock_flis != -1)
		config_pin_flis(pdata->spi_clock_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_dataout_flis != -1)
		config_pin_flis(pdata->spi_dataout_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_datain_flis != -1)
		config_pin_flis(pdata->spi_datain_flis,
				MUX, MUX_EN_INPUT_EN | INPUT_EN);

	if (pdata->spi_cs_flis != -1)
		config_pin_flis(pdata->spi_cs_flis,
				MUX, MUX_EN_OUTPUT_EN | OUTPUT_EN);
#endif
}

static void spi_cs_control(u32 command)
{
	if (cs_chip_select == -1)
		return;

	/* CS must be set high during transmission */
	if (command == CS_ASSERT)
		gpio_set_value(cs_chip_select, 1);
	else
		gpio_set_value(cs_chip_select, 0);
};

static struct intel_mid_ssp_spi_chip spi_chip = {
	.burst_size	= DFLT_FIFO_BURST_SIZE,
	.timeout	= DFLT_TIMEOUT_VAL,
	.dma_enabled	= false,
	.cs_control	= spi_cs_control,
};

static int m10mo_platform_init(void)
{
	static const char gpio_name[] = "xenon_ready";
	int ret;

	if (intr_gpio == -1) {
#ifndef CONFIG_ACPI
		intr_gpio = get_gpio_by_name(gpio_name);
		if (intr_gpio == -1) {
			pr_err("Failed to get interrupt gpio\n");
			return -EINVAL;
		}
#else
		intr_gpio = acpi_get_gpio("\\_SB.GPO1", 54);
#endif
		pr_info("camera interrupt gpio: %d\n", intr_gpio);
	}

	ret = gpio_request(intr_gpio, gpio_name);
	if (ret) {
		pr_err("Failed to request interrupt gpio(pin %d)\n", intr_gpio);
		return -EINVAL;
	}

	ret = gpio_direction_input(intr_gpio);
	if (ret) {
		pr_err("failed to set interrupt gpio %d direction\n",
				intr_gpio);
		gpio_free(intr_gpio);
		intr_gpio = -1;
	}

	return ret;
}

static int m10mo_platform_deinit(void)
{
	if (intr_gpio >= 0)
		gpio_free(intr_gpio);
	if (cs_chip_select >= 0)
		gpio_free(cs_chip_select);
	intr_gpio = -1;
	cs_chip_select = -1;

	camera_sensor_gpio_free(camera_reset);
	camera_reset = -1;

	return 0;
}

static struct atomisp_camera_caps *m10mo_get_camera_caps(void)
{
	m10mo_camera_caps.sensor_num = 1;
	m10mo_camera_caps.sensor[0].stream_num = 2;
	if (m10mo_platform_identify_fw() == M10MO_FW_TYPE_2) {
		m10mo_camera_caps.sensor_num = 2;
		m10mo_camera_caps.sensor[1].stream_num = 2;
	}
	return &m10mo_camera_caps;
}

static struct m10mo_platform_data m10mo_sensor_platform_data = {
	/* Common part for all sensors used with atom isp */
	.common.gpio_ctrl	= m10mo_gpio_ctrl,
	.common.gpio_intr_ctrl	= m10mo_gpio_intr_ctrl,
	.common.flisclk_ctrl	= m10mo_flisclk_ctrl,
	.common.power_ctrl	= m10mo_power_ctrl,
	.common.csi_cfg		= m10mo_csi_configure,
	.common.platform_init	= m10mo_platform_init,
	.common.platform_deinit = m10mo_platform_deinit,
	.common.get_camera_caps = m10mo_get_camera_caps,

	/* platform data for spi flashing */
	.spi_pdata.spi_enabled	= false, /* By default SPI is not available */
	.spi_pdata.spi_bus_num	= 6, /* Board specific */
	.spi_pdata.spi_cs_gpio	= 117, /* Board specific */
	.spi_pdata.spi_speed_hz = 10000000, /* Board specific */
	/* Set flis values to -1 if the data is correct in pin cfg xml */
	.spi_pdata.spi_clock_flis	= ann_gp_ssp_6_clk, /* Board specific */
	.spi_pdata.spi_dataout_flis	= ann_gp_ssp_6_txd, /* Board specific */
	.spi_pdata.spi_datain_flis	= ann_gp_ssp_6_rxd, /* Board specific */
	.spi_pdata.spi_cs_flis		= ann_gp_ssp_6_fs, /* Board specific */

	.def_fw_type    = M10MO_FW_TYPE_0,
	.ref_clock_rate = clock_rate, /* Board specific */
	.mipi_packet_size = mipi_packet_size,
	.fw_ids		= fw_ids,
	.spi_setup	= setup_m10mo_spi,
	.identify_fw	= m10mo_platform_identify_fw,
};

int m10mo_platform_identify_fw(void)
{
#ifdef CONFIG_ACPI
	static int fw_type = -1;
	struct device *dev;
	struct i2c_client *client;

	if (fw_type != -1)
		return fw_type;

	dev = bus_find_device_by_name(&i2c_bus_type, NULL, "4-001f");
	if (dev) {
		client = to_i2c_client(dev);
		if (strncmp(client->name, "SOM33FB:00", 10) == 0) {
			m10mo_sensor_platform_data.spi_setup = NULL;
			input_format = ATOMISP_INPUT_FORMAT_YUV420_8_LEGACY;
			m10mo_camera_caps.multi_stream_ctrl = true;
			fw_type = M10MO_FW_TYPE_2;
		}
	}
	return fw_type;
#else
	return -1;
#endif
}

static struct spi_board_info m10mo_spi_info[] = {
	{
		.modalias		= "m10mo_spi",
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
		.controller_data	= &spi_chip,
		.platform_data		= &m10mo_sensor_platform_data.spi_pdata,
	}
};

static void setup_m10mo_spi(struct m10mo_atomisp_spi_platform_data *spi_pdata,
	void *data)
{
	m10mo_spi_info[0].bus_num	= spi_pdata->spi_bus_num;
	m10mo_spi_info[0].max_speed_hz	= spi_pdata->spi_speed_hz;
	spi_pdata->device_data		= data;

	spi_hw_resources_setup(spi_pdata);

	/* register SPI interface for the FW update */
	spi_register_board_info(m10mo_spi_info,
				ARRAY_SIZE(m10mo_spi_info));
}

void *m10mo_platform_data(void *info)
{
	/*
	 * Atom isp driver assumes to get pointer to struct
	 * "camera_sensor_platform_data". Nasty assumption but must honored
	 */
	return &m10mo_sensor_platform_data.common;
}

#ifdef CONFIG_VIDEO_M10MO_FAKE_SFI_TABLE
static struct sfi_device_table_entry m10mo_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x1F,
	.irq = 0x0,
	.max_freq = 0x0,
	.name = "m10mo",
};

static int __init platform_m10mo_module_init(void)
{
	struct devs_id *dev;
	dev = get_device_id(m10mo_entry.type, m10mo_entry.name);
	if (dev && dev->device_handler)
		dev->device_handler(&m10mo_entry, dev);
	return 0;
}

module_init(platform_m10mo_module_init);
#endif /* CONFIG_M10MO_FAKE_SFI_TABLE */
