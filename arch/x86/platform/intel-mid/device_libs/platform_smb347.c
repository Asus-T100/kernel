/*
 * platform_smb347.c: smb347 platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include "platform_smb347.h"

/* Redridge DV2.1 */
static struct smb347_charger_platform_data smb347_rr_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xFC,
						0x01, 0x95,
						0x02, 0x83,
						0x03, 0xE3,
						0x04, 0x3A,
						0x05, 0x1A,
						0x06, 0x65,
						0x07, 0xEF,
						0x08, 0x09,
						0x09, 0xDF,
						0x0A, 0xAB,
						0x0B, 0x5A,
						0x0C, 0xC1,
						0x0D, 0x46,
						0x00, 0x00,
					},
};

/* Salitpa EV 0.5 */
static struct smb347_charger_platform_data smb347_ev05_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_DISABLED,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xA1,
						0x01, 0x6C,
						0x02, 0x93,
						0x03, 0xE5,
						0x04, 0x3E,
						0x05, 0x16,
						0x06, 0x0C,
						0x07, 0x8c,
						0x08, 0x08,
						0x09, 0x0c,
						0x0A, 0xA4,
						0x0B, 0x13,
						0x0C, 0x81,
						0x0D, 0x02,
						0x0E, 0x20,
						0x10, 0x7F
					},
};

/* Salitpa EV 1.0 */
static struct smb347_charger_platform_data smb347_ev10_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_DISABLED,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xAA,
						0x01, 0x6C,
						0x02, 0x93,
						0x03, 0xE5,
						0x04, 0x3E,
						0x05, 0x16,
						0x06, 0x74,
						0x07, 0xCC,
						0x09, 0x0C,
						0x0A, 0xA4,
						0x0B, 0x13,
						0x0C, 0x8D,
						0x0D, 0x00,
						0x0E, 0x20,
						0x10, 0x7F
					},
};

static void *get_platform_data(void)
{
	/* Redridge all */
	if (INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO))
		return &smb347_rr_pdata;
	else if (INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO)) {
		/* Salitpa */
		/* EV 0.5 */
		if (SPID_HARDWARE_ID(MFLD, TABLET, SLP, EV05))
			return &smb347_ev05_pdata;
		/* EV 1.0 and later */
		else
			return &smb347_ev10_pdata;
	}

	return NULL;
}

void *smb347_platform_data(void *info)
{
	return get_platform_data();
}
