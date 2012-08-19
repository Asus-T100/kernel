/*
 * CLV regulator platform data
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/lcd.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>

/***********VPROG1 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog1_consumer[] = {
	REGULATOR_SUPPLY("vprog1", "4-0048"),
	REGULATOR_SUPPLY("vprog1", "4-0036"),
};

static struct regulator_init_data vprog1_data = {
	   .constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2800000,
			.valid_ops_mask	= REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vprog1_consumer),
		.consumer_supplies	=	vprog1_consumer,
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
/***********VPROG2 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog2_consumer[] = {
};
static struct regulator_init_data vprog2_data = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2800000,
			.valid_ops_mask	= REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
						},
		.num_consumer_supplies	=	ARRAY_SIZE(vprog2_consumer),
		.consumer_supplies	=	vprog2_consumer,
};
static struct intel_pmic_info vprog2_info = {
		.pmic_reg   = VPROG2CNT_ADDR,
		.init_data  = &vprog2_data,
		.table_len  = ARRAY_SIZE(VPROG2_VSEL_table),
		.table      = VPROG2_VSEL_table,
};
static struct platform_device vprog2_device = {
	.name = "intel_regulator",
	.id = VPROG2,
	.dev = {
		.platform_data = &vprog2_info,
	},
};
/***********VEMMC1 REGUATOR platform data*************/
static struct regulator_consumer_supply vemmc1_consumer[] = {
};
static struct regulator_init_data vemmc1_data = {
		.constraints = {
			.min_uV			= 2850000,
			.max_uV			= 2850000,
			.valid_ops_mask	=	REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
		 },
		.num_consumer_supplies	=	ARRAY_SIZE(vemmc1_consumer),
		.consumer_supplies	=	vemmc1_consumer,
};

static struct intel_pmic_info vemmc1_info = {
		.pmic_reg   = VEMMC1CNT_ADDR,
		.init_data  = &vemmc1_data,
		.table_len  = ARRAY_SIZE(VEMMC1_VSEL_table),
		.table      = VEMMC1_VSEL_table,
};

static struct platform_device vemmc1_device = {
	.name = "intel_regulator",
	.id = VEMMC1,
	.dev = {
		.platform_data = &vemmc1_info,
	},
};

/***********VEMMC2 REGUATOR platform data*************/
static struct regulator_consumer_supply vemmc2_consumer[] = {
	REGULATOR_SUPPLY("vemmc2", "2-0070"),
	REGULATOR_SUPPLY("vemmc2", NULL),
};

static struct regulator_init_data vemmc2_data = {
		.constraints = {
			.min_uV			= 2850000,
			.max_uV			= 2850000,
			.valid_ops_mask	=	REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE,
			.valid_modes_mask	=	REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
		      },
		.num_consumer_supplies		= ARRAY_SIZE(vemmc2_consumer),
		.consumer_supplies		 = vemmc2_consumer,
};

static struct intel_pmic_info vemmc2_info = {
		.pmic_reg   = VEMMC2CNT_ADDR,
		.init_data  = &vemmc2_data,
		.table_len  = ARRAY_SIZE(VEMMC2_VSEL_table),
		.table      = VEMMC2_VSEL_table,
};

static struct platform_device vemmc2_device = {
	.name = "intel_regulator",
	.id = VEMMC2,
	.dev = {
		.platform_data = &vemmc2_info,
	},
};

static struct platform_device *regulator_devices[] __initdata = {
	&vprog1_device,
	&vprog2_device,
	&vemmc1_device,
	&vemmc2_device,
};

static int __init regulator_init(void)
{
	platform_add_devices(regulator_devices,
		ARRAY_SIZE(regulator_devices));
	return 0;
}
device_initcall(regulator_init);
