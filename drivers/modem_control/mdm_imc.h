/*
 * linux/drivers/modem_control/mdm_imc.h
 *
 * Version 1.0
 *
 * This code includes definition for IMC modems
 * There is no guarantee for other modems
 *
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


#ifndef _MDM_IMC_H
#define _MDM_IMC_H

/* PMIC reg to power on/off the modem. Generic values.
   Can be overwritten in imc_xxxx.c specific file */
#define CHIPCNTRL	CONFIG_MDM_CTRL_CHIPCNTRL
#define CHIPCNTRL_OFF	CONFIG_MDM_CTRL_CHIPCNTRL_OFF
#define CHIPCNTRL_ON	CONFIG_MDM_CTRL_CHIPCNTRL_ON

/* GPIO names */
#define GPIO_RST_OUT	"ifx_mdm_rst_out"
#define GPIO_PWR_ON	"ifx_mdm_pwr_on"
#define GPIO_RST_BBN	"ifx_mdm_rst_pmu"
#define GPIO_CDUMP	"modem-gpio2"

/* Delays for powering up/resetting the modem */
#define MDM_ON1_DURATION		60 /* ON1 pulse duration (usec) */
#define MDM_ON1_DELAY			200 /* ON1 wait duration (usec) */

#define MDM_COLD_RST_OFF_DELAY		20 /* Cold reset wait duration (msec) */

#define MDM_WARM_RST_DURATION		60 /* RESET_BB_N pulse delay (usec) */
#define MDM_WARM_RST_FLASHING_DELAY	30 /* RESET_BB_N wait duration (msec) */

#define MDM_WARM_RST_FLASHING_OVER	90 /* Flashing window closed (msec) */

int mdm_ctrl_cold_boot(struct mdm_ctrl *drv);
int mdm_ctrl_cold_reset(struct mdm_ctrl *drv);
int mdm_ctrl_silent_warm_reset(struct mdm_ctrl *drv);
int mdm_ctrl_normal_warm_reset(struct mdm_ctrl *drv);
int mdm_ctrl_flashing_warm_reset(struct mdm_ctrl *drv);
int mdm_ctrl_power_off(struct mdm_ctrl *drv);

#endif /* _MDM_IMC_H */
