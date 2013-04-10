/*
 * platform_wm5102.c: wm51020 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/intel_pmic.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include "platform_wm5102.h"
#include "platform_clv_regulator.h"

static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  11, .key = KEY_MEDIA },
	{ .max =  28, .key = KEY_MEDIA },
	{ .max =  54, .key = KEY_MEDIA },
	{ .max = 100, .key = KEY_MEDIA },
	{ .max = 186, .key = KEY_MEDIA },
	{ .max = 430, .key = KEY_MEDIA },
};

static struct arizona_pdata wm5102_pdata  = {
	.ldoena = 44,
	.irq_base = 0x50,
	.clk32k_src = ARIZONA_32KZ_MCLK2,
	.irq_flags = IRQF_TRIGGER_FALLING,
	.dmic_ref = {ARIZONA_DMIC_MICBIAS1, ARIZONA_DMIC_MICBIAS3, 0, 0},
	.inmode = {ARIZONA_INMODE_SE, ARIZONA_INMODE_DMIC, 0, 0},
	.jd_gpio5 = 0,
	.micd_ranges = micd_ctp_ranges,
	.num_micd_ranges = ARRAY_SIZE(micd_ctp_ranges),
};

static int __init add_wm5102_consumers(void)
{
	struct regulator_dev *rdev = get_regulator_dev();

	static struct regulator_consumer_supply vprog1_consumer[] = {
		REGULATOR_SUPPLY("vprog1", "4-0048"),
		REGULATOR_SUPPLY("vprog1", "4-0036"),
		REGULATOR_SUPPLY("vprog1", "4-0010"),
		REGULATOR_SUPPLY("AVDD", "1-001a"),
		REGULATOR_SUPPLY("DBVDD1", "1-001a"),
		REGULATOR_SUPPLY("DBVDD2", "wm5102-codec"),
		REGULATOR_SUPPLY("DBVDD3", "wm5102-codec"),
		REGULATOR_SUPPLY("CPVDD", "wm5102-codec"),
		REGULATOR_SUPPLY("SPKVDDL", "wm5102-codec"),
		REGULATOR_SUPPLY("SPKVDDR", "wm5102-codec"),
	};

	static struct regulator_init_data vprog1_data = {
		   .constraints = {
				.min_uV			= 1200000,
				.max_uV			= 2800000,
				.valid_ops_mask	= REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE |
							REGULATOR_CHANGE_VOLTAGE,
				.valid_modes_mask = REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
							REGULATOR_MODE_FAST,
			},
			.num_consumer_supplies	= ARRAY_SIZE(vprog1_consumer),
			.consumer_supplies	= vprog1_consumer,
	};

	static struct intel_pmic_info vprog1_info = {
			.pmic_reg   = VPROG1CNT_ADDR,
			.init_data  = &vprog1_data,
			.table_len  = ARRAY_SIZE(VPROG1_VSEL_table),
			.table      = VPROG1_VSEL_table,
	};
	static struct platform_device vprog1_device = {
		.name = "intel_regulator",
		.id = VPROG1,
		.dev = {
			.platform_data = &vprog1_info,
		},
	};

	pr_debug("Replace common regulator with wm5102 specific %08x\n", rdev);
	set_consumer_supply(&vprog1_device);

	return 0;
}

void __init *wm5102_platform_data(void *info)
{
	int gpio;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	gpio = get_gpio_by_name("gpio_codec_int");
	i2c_info->irq = gpio + INTEL_MID_IRQ_OFFSET;

	add_wm5102_consumers();

	return &wm5102_pdata;
}
