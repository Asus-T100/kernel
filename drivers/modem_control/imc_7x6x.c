/*
 * linux/drivers/modem_control/imc_7x6x.c
 *
 * Version 0.2
 *
 * This code includes power sequences for IMC 6260 modems.
 * There is no guarantee for other modems.
 *
 * Intel Mobile Communication protocol driver for modem boot
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *          Frederic Berat <fredericx.berat@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include "mdm_util.h"
#include "mdm_imc.h"

#include <linux/mdm_ctrl.h>

/*****************************************************************************
 *
 * Modem Power/Reset functions
 *
 ****************************************************************************/

/**
 * Perform a modem cold boot sequence:
 *
 *  - Set to HIGH the PWRDWN_N to switch ON the modem
 *  - Set to HIGH the RESET_BB_N
 *  - Do a pulse on ON1
 *
 * @ch_ctx: a reference to related channel context
 */
int mdm_ctrl_cold_boot(struct mdm_ctrl *drv)
{
	int ret = 0;
	u16 addr = CHIPCNTRL;
	u8 data;
	unsigned long flags;

	pr_warn(DRVNAME": Cold boot requested");

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_COLD_BOOT);
	flush_workqueue(drv->change_state_wq);

	/* AP request => just ignore the modem reset */
	spin_lock_irqsave(&drv->state_lck, flags);
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Toggle the RESET_BB_N */
	gpio_set_value(drv->gpio_rst_bbn, 0);

	/* Wait before doing the pulse on ON1 */
	udelay(MDM_ON1_DELAY);

	/* Write the new register value (CHIPCNTRL_OFF) */
	data = CHIPCNTRL_OFF;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME": scu_ipc_writev(ON) failed (ret: %d)", ret);
		goto out;
	}

	/* Wait before RESET_PWRDN_N to be 1 */
	msleep(MDM_COLD_RST_OFF_DELAY);

	/* Toggle the RESET_BB_N */
	gpio_set_value(drv->gpio_rst_bbn, 1);

	/* Wait before doing the pulse on ON1 */
	udelay(MDM_ON1_DELAY);

	/* Do a pulse on ON1 */
	gpio_set_value(drv->gpio_pwr_on, 1);
	udelay(MDM_ON1_DURATION);
	gpio_set_value(drv->gpio_pwr_on, 0);

	mdm_ctrl_launch_timer(&drv->flashing_timer,
				MDM_WARM_RST_FLASHING_DELAY,
				MDM_TIMER_FLASH_ENABLE);
out:
	return ret;
}

/**
 * Perform a modem cold reset:
 *
 * - Set the RESET_BB_N to low (better SIM protection)
 * - Set the EXT1P35VREN field to low  during 20ms (CHIPCNTRL PMIC register)
 * - set the EXT1P35VREN field to high during 10ms (CHIPCNTRL PMIC register)
 */
int mdm_ctrl_cold_reset(struct mdm_ctrl *drv)
{
	return mdm_ctrl_cold_boot(drv);
}

/**
 *  Perform a silent modem warm reset sequence:
 *
 *  - Do a pulse on the RESET_BB_N
 *  - No struct modification
 *  - debug purpose only
 * @ch_ctx: a reference to related channel context
 */
int mdm_ctrl_silent_warm_reset(struct mdm_ctrl *drv)
{
	gpio_set_value(drv->gpio_rst_bbn, 0);
	udelay(MDM_WARM_RST_DURATION);
	gpio_set_value(drv->gpio_rst_bbn, 1);

	return 0;
}

/**
 *  Perform a normal modem warm reset sequence:
 *
 *  - Do a pulse on the RESET_BB_N
 *
 * @ch_ctx: a reference to related channel context
 */
int mdm_ctrl_normal_warm_reset(struct mdm_ctrl *drv)
{
	unsigned long flags;

	pr_info(DRVNAME ": Normal warm reset requested\r\n");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);
	flush_workqueue(drv->change_state_wq);

	mdm_ctrl_silent_warm_reset(drv);

	mdm_ctrl_launch_timer(&drv->flashing_timer,
				MDM_WARM_RST_FLASHING_DELAY,
				MDM_TIMER_FLASH_ENABLE);

	return 0;
}

/**
 *  Perform a normal modem warm reset sequence:
 *
 *  - Do a pulse on the RESET_BB_N
 *  - Wait before return
 *
 * @ch_ctx: a reference to related channel context
 */
int mdm_ctrl_flashing_warm_reset(struct mdm_ctrl *drv)
{
	unsigned long flags;

	pr_info(DRVNAME ": Flashing warm reset requested");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the current modem state */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);
	flush_workqueue(drv->change_state_wq);

	mdm_ctrl_silent_warm_reset(drv);

	msleep(MDM_WARM_RST_FLASHING_DELAY);

	return 0;
}

/**
 *  Perform the modem switch OFF sequence:
 *		- Set to low the ON1
 *		- Write the PMIC reg
 *
 * @ch_ctx: a reference to related channel context
 */
int mdm_ctrl_power_off(struct mdm_ctrl *drv)
{
	u16 addr = CHIPCNTRL;
	u8 data;
	int ret = 0;
	unsigned long flags;

	pr_info(DRVNAME ": Power OFF requested");

	spin_lock_irqsave(&drv->state_lck, flags);
	/* AP requested reset => just ignore */
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the modem state to OFF */
	mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_OFF);
	flush_workqueue(drv->change_state_wq);

	/* Set the RESET_BB_N to 0 */
	gpio_set_value(drv->gpio_rst_bbn, 0);

	/* Write the new register value (CHIPCNTRL_OFF) */
	data = CHIPCNTRL_OFF;
	ret =  intel_scu_ipc_writev(&addr, &data, 1);
	if (ret) {
		pr_err(DRVNAME ": ipc_writev(OFF)  failed (ret: %d)", ret);
		goto out;
	}

 out:
	return ret;
}

