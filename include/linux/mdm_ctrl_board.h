/*
 * mdm_ctrl_board.h
 *
 * Header for the Modem control driver.
 *
 * Copyright (C) 2010, 2011 Intel Corporation. All rights reserved.
 *
 * Contact: Frederic BERAT <fredericx.berat@intel.com>
 *          Faouaz TENOUTIT <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __MDM_CTRL_BOARD_H__
#define __MDM_CTRL_BOARD_H__

#include <asm/intel-mid.h>
#include <linux/module.h>

#define DEVICE_NAME "modem_control"

/* Supported Modem IDs*/
#define MODEM_UNSUP	0
#define MODEM_6260	6260
#define MODEM_6268	6268
#define MODEM_6360	6360
#define MODEM_7160	7160
#define MODEM_7260	7260

/* Supported PMIC IDs*/
enum {
	UNKNOWN_PMIC,
	MFLD_PMIC,
	CLVT_PMIC,
	MRFL_PMIC,
	BYT_PMIC
};

/* GPIO names */
#define GPIO_RST_OUT	"ifx_mdm_rst_out"
#define GPIO_PWR_ON	"ifx_mdm_pwr_on"
#define GPIO_RST_BBN	"ifx_mdm_rst_pmu"
#define GPIO_CDUMP	"modem-gpio2"
#define GPIO_CDUMP_MRFL	"MODEM_CORE_DUMP"

/* Retrieve modem parameters on ACPI framework */
int retrieve_modem_platform_data(struct platform_device *pdev);

/* Modem basical info
 * @modem_name: modem real name
 * @id: Modem id to retrieve sequences from
 * @pmic: pmic id
 * @cpu: cpu id
 * @cpu_name: cpu real name
 * @data: pointer to any supplementary data
 */
struct modem_base_info {
	char	modem_name[SFI_NAME_LEN + 1];
	int	id;
	void	*pmic;
	int	cpu;
	char	cpu_name[SFI_NAME_LEN + 1];
	void	*data;
};

/* struct mcd_cpu_data
 * @gpio_rst_out: Reset out gpio (self reset indicator)
 * @gpio_pwr_on: Power on gpio (ON1 - Power up pin)
 * @gpio_rst_bbn: RST_BB_N gpio (Reset pin)
 * @gpio_cdump: CORE DUMP indicator
 * @early_pwr_on: call to power_on on probe indicator
 * @early_pwr_off: call to power_off on probe indicator
 */
struct mdm_ctrl_cpu_data {
	int	gpio_rst_out;
	int	gpio_pwr_on;
	int	gpio_rst_bbn;
	int	gpio_cdump;
	bool	early_pwr_on;
	bool	early_pwr_off;
};

/* struct mcd_pmic_data
 * @id: PMIC internal Id
 * @chipctrl: PMIC regsiter
 * @chipctrlon: Pmic value for Modem ON
 * @chipctrloff: Pmic value for Power Off
 * @chipctrl_mask: Mask value for On and Off Pmic register bits.
 */
struct mdm_ctrl_pmic_data {
	int	id;
	int	chipctrl;
	int	chipctrlon;
	int	chipctrloff;
	int	chipctrl_mask;
};

/* struct mdm_ctrl_pdata
 * @modem_name: Modem name, used to select correct sequences
 * @chipctrl: PMIC base address
 * @chipctrlon: Modem power on PMIC value
 * @chipctrloff: Modem power off PMIC value
 * @pre_pwr_down_delay:Delay before powering down the modem (us)
 * @pwr_down_duration:Powering down duration (us)
 * @device_data: Device related data
 * @archdata: Architecture-dependent device data
 */
struct mdm_ctrl_pdata {
	int				modem;
	int				chipctrl;
	int				chipctrlon;
	int				chipctrloff;
	int				chipctrl_mask;
	int				pre_pwr_down_delay;
	int				pwr_down_duration;
	bool				is_mdm_ctrl_disabled;
	void				*device_data;
	struct mdm_ctrl_cpu_data	*cpu_data;
};

/* struct mdm_ctrl_device_info - Board and modem infos
 *
 * @pre_on_delay:Delay before pulse on ON1 (us)
 * @on_duration:Pulse on ON1 duration (us)
 * @pre_wflash_delay:Delay before flashing window, after warm_reset (ms)
 * @pre_cflash_delay:Delay before flashing window, after cold_reset (ms)
 * @flash_duration:Flashing window durtion (ms)int  Not used ?
 * @warm_rst_duration:Warm reset duration (ms)
 */
struct mdm_ctrl_device_info {
	int	pre_on_delay;
	int	on_duration;
	int	pre_wflash_delay;
	int	pre_cflash_delay;
	int	flash_duration;
	int	warm_rst_duration;
};

#endif /* __MDM_CTRL_BOARD_H__ */
