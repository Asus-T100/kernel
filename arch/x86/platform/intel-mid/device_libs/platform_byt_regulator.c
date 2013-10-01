/*
 * platform_byt_regulator.c - Baytrail regulator machine drvier
 * Copyright (c) 2013, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/intel_crystal_cove_pmic.h>
#include <linux/regulator/machine.h>

/***********V2P85S REGUATOR platform data*************/
static struct regulator_consumer_supply v2p85s_consumer[] = {
};
static struct regulator_init_data v2p85s_data = {
	.constraints = {
		.name = "v2p85s",
		.min_uV			= 2565000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v2p85s_consumer),
	.consumer_supplies	= v2p85s_consumer,
};

static struct intel_pmic_info v2p85s_info = {
	.pmic_reg   = V2P85SCNT_ADDR,
	.init_data  = &v2p85s_data,
	.table_len  = ARRAY_SIZE(V2P85S_VSEL_table),
	.table      = V2P85S_VSEL_table,
};
static struct platform_device v2p85s_device = {
	.name = "intel_regulator",
	.id = V2P85S,
	.dev = {
		.platform_data = &v2p85s_info,
	},
};

/***********V2P85SX REGUATOR platform data*************/
static struct regulator_consumer_supply v2p85sx_consumer[] = {
};
static struct regulator_init_data v2p85sx_data = {
	.supply_regulator = "v2p85s",
	.constraints = {
		.name = "v2p85sx",
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v2p85sx_consumer),
	.consumer_supplies	= v2p85sx_consumer,
};

static struct intel_pmic_info v2p85sx_info = {
	.pmic_reg   = V2P85SXCNT_ADDR,
	.init_data  = &v2p85sx_data,
	.table_len  = ARRAY_SIZE(V2P85SX_VSEL_table),
	.table      = V2P85SX_VSEL_table,
};
static struct platform_device v2p85sx_device = {
	.name = "intel_regulator",
	.id = V2P85SX,
	.dev = {
		.platform_data = &v2p85sx_info,
	},
};

/***********V3P3S REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3s_consumer[] = {
};
static struct regulator_init_data v3p3s_data = {
	.constraints = {
		.name = "v3p3s",
		.min_uV			= 3332000,
		.max_uV			= 3332000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v3p3s_consumer),
	.consumer_supplies	= v3p3s_consumer,
};

static struct intel_pmic_info v3p3s_info = {
	.pmic_reg   = V3P3SCNT_ADDR,
	.init_data  = &v3p3s_data,
	.table_len  = ARRAY_SIZE(V3P3S_VSEL_table),
	.table      = V3P3S_VSEL_table,
};
static struct platform_device v3p3s_device = {
	.name = "intel_regulator",
	.id = V3P3S,
	.dev = {
		.platform_data = &v3p3s_info,
	},
};

/***********V1P8S REGUATOR platform data*************/
static struct regulator_consumer_supply v1p8s_consumer[] = {
};
static struct regulator_init_data v1p8s_data = {
	.constraints = {
		.name = "v1p8s",
		.min_uV			= 1817000,
		.max_uV			= 1817000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p8s_consumer),
	.consumer_supplies	= v1p8s_consumer,
};

static struct intel_pmic_info v1p8s_info = {
	.pmic_reg   = V1P8SCNT_ADDR,
	.init_data  = &v1p8s_data,
	.table_len  = ARRAY_SIZE(V1P8S_VSEL_table),
	.table      = V1P8S_VSEL_table,
};
static struct platform_device v1p8s_device = {
	.name = "intel_regulator",
	.id = V1P8S,
	.dev = {
		.platform_data = &v1p8s_info,
	},
};

/***********VSYS_S REGUATOR platform data*************/
static struct regulator_consumer_supply vsys_s_consumer[] = {
};
static struct regulator_init_data vsys_s_data = {
	.constraints = {
		.name = "vsys_s",
		.min_uV			= 4200000,
		.max_uV			= 4200000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vsys_s_consumer),
	.consumer_supplies	= vsys_s_consumer,
};

static struct intel_pmic_info vsys_s_info = {
	.pmic_reg   = VSYS_SCNT_ADDR,
	.init_data  = &vsys_s_data,
	.table_len  = ARRAY_SIZE(VSYS_S_VSEL_table),
	.table      = VSYS_S_VSEL_table,
};
static struct platform_device vsys_s_device = {
	.name = "intel_regulator",
	.id = VSYS_S,
	.dev = {
		.platform_data = &vsys_s_info,
	},
};
static struct platform_device *regulator_devices[] __initdata = {
	&v2p85s_device,
	&v2p85sx_device,
	&v3p3s_device,
	&v1p8s_device,
	&vsys_s_device,
};

static int __init regulator_init(void)
{
	platform_add_devices(regulator_devices,
		ARRAY_SIZE(regulator_devices));
	return 0;
}
device_initcall(regulator_init);
