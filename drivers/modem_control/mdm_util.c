/*
 * linux/drivers/modem_control/mdm_util.c
 *
 * Version 1.0
 * Generic function definition.
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

/* Modem boot driver instance */
struct mdm_ctrl *mdm_drv;

/**
 * @brief Set the open device state
 *
 * @param drv:  Modem boot struct to modify
 * @param value: Value to set
 */
inline void mdm_ctrl_set_opened(struct mdm_ctrl *drv, int value)
{
	/* Set the open flag */
	drv->opened = value;
}

/**
 * @brief Return the device open state
 *
 * @param drv: mdm_ctrl struct to get the state from
 */
inline int mdm_ctrl_get_opened(struct mdm_ctrl *drv)
{
	int opened;

	/* Set the open flag */
	opened = drv->opened;
	return opened;
}

inline void mdm_ctrl_launch_work(struct mdm_ctrl *drv, int state)
{
	struct next_state *new_state;
	unsigned long flags;

	new_state = kzalloc(sizeof(struct next_state), GFP_ATOMIC);
	if (!new_state) {
		pr_err(DRVNAME": Can't allocate new_state memory");
		return;
	}
	new_state->state = state;

	spin_lock_irqsave(&drv->state_lck, flags);
	list_add_tail(&new_state->link, &drv->next_state_link);
	queue_work(drv->change_state_wq, &drv->change_state_work);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* We should unlock on RESET_OUT 0xXX or
	* on set_state(MDM_CTRL_STATE_OFF). The second one can happen on
	* mdm_ctrl_power_off request or when the modem is declared as DEAD by
	* user and willingly set to OFF.
	*/
	if (state == MDM_CTRL_STATE_IPC_READY || state == MDM_CTRL_STATE_OFF)
		wake_unlock(&drv->stay_awake);
}

/**
 * @brief Set the modem state
 *
 * @param drv:  Modem boot struct to modify
 * @param state: Modem state to set
 */
inline void mdm_ctrl_set_state(struct work_struct *work)
{
	struct mdm_ctrl *drv;
	struct next_state *new_state;
	unsigned long flags;

	drv = container_of(work, struct mdm_ctrl, change_state_work);

	while (!list_empty_careful(&drv->next_state_link)) {
		spin_lock_irqsave(&drv->state_lck, flags);
		new_state = list_first_entry(&drv->next_state_link,
					 struct next_state, link);
		list_del_init(&new_state->link);
		spin_unlock_irqrestore(&drv->state_lck, flags);

		/* Set the current modem state */
		drv->modem_state = new_state->state;
		if (likely(drv->modem_state != MDM_CTRL_STATE_UNKNOWN) &&
			 (drv->modem_state & drv->polled_states)) {

			drv->polled_state_reached = true;
			wake_up(&drv->wait_wq);
			pr_info(DRVNAME": Waking up polling 0x%x\r\n",
				 drv->modem_state);

		}
		wake_up(&drv->event);
		kfree(new_state);
	}
}

/**
 * @brief Get the local current modem state
 *
 * @param drv: mdm_ctrl struct to get the modem state from
 *
 * Note: Real current state may be different in case of self-reset
 */
inline int mdm_ctrl_get_state(struct mdm_ctrl *drv)
{
	/* Return the current modem state */
	return drv->modem_state;
}

/**
 * @brief Set the modem state to FW_DOWNLOAD_READY
 *
 *
 **/
void mdm_ctrl_enable_flashing(unsigned long int param)
{
	struct mdm_ctrl *drv = (struct mdm_ctrl *) param;

	del_timer(&drv->flashing_timer);
	if (mdm_ctrl_get_state(drv) != MDM_CTRL_STATE_IPC_READY) {
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_FW_DOWNLOAD_READY);
	}
}

/**
 * @brief Set the modem state to NONE if previous state
 * was FW_DOWNLOAD_READY
 *
 **/
void mdm_ctrl_disable_flashing(unsigned long int param)
{
	struct mdm_ctrl *drv = (struct mdm_ctrl *) param;

	if (mdm_ctrl_get_state(drv) == MDM_CTRL_STATE_FW_DOWNLOAD_READY)
		mdm_ctrl_launch_timer(&drv->flashing_timer,
					MDM_WARM_RST_FLASHING_OVER,
					MDM_TIMER_FLASH_DISABLE);
}

/**
 *
 *
 *
 **/
void mdm_ctrl_launch_timer(struct timer_list *timer, int delay,
				unsigned int timer_type)
{
	timer->data = (unsigned long int) mdm_drv;
	switch (timer_type) {
	case MDM_TIMER_FLASH_ENABLE:
		timer->function = mdm_ctrl_enable_flashing;
		break;
	case MDM_TIMER_FLASH_DISABLE:
		timer->function = mdm_ctrl_disable_flashing;
		break;
	default:
		pr_err(DRVNAME": Unrecognized timer type %d", timer_type);
		del_timer(timer);
		return;
		break;
	}
	mod_timer(timer, jiffies + msecs_to_jiffies(delay));
}

/**
 * @brief Set the RESET ongoing flag value
 *
 * @param drv:  Modem boot struct to modify
 * @param ongoing: Flag value to set
 */
inline void mdm_ctrl_set_reset_ongoing(struct mdm_ctrl *drv, int ongoing)
{
	drv->rst_ongoing = ongoing;
}

/**
 * @brief Get the RESET ongoing flag value
 *
 * @param drv: mdm_ctrl struct to get the ongoing flag value from
 */
inline int mdm_ctrl_get_reset_ongoing(struct mdm_ctrl *drv)
{

	return drv->rst_ongoing;
}

/*
 * @brief: Move the gpio value to simplify access
 *
 */
void mdm_ctrl_get_gpio(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata = drv->pdata;

	drv->gpio_rst_out = pdata->gpio_rst_out;
	drv->gpio_pwr_on  = pdata->gpio_pwr_on;
	drv->gpio_rst_bbn = pdata->gpio_rst_bbn;
	drv->gpio_cdump   = pdata->gpio_cdump;
}

void mdm_ctrl_set_func(struct mdm_ctrl *drv, int modem_type)
{
	pr_info(DRVNAME": Taking %d modem sequences", modem_type);

	switch (modem_type) {
	case MODEM_6260:
	case MODEM_6268:
		drv->mdm_ctrl_cold_boot = mdm_ctrl_cold_boot_6x6x;
		drv->mdm_ctrl_cold_reset = mdm_ctrl_cold_reset_6x6x;
		drv->mdm_ctrl_normal_warm_reset =
				mdm_ctrl_normal_warm_reset_6x6x;
		drv->mdm_ctrl_silent_warm_reset =
				mdm_ctrl_silent_warm_reset_6x6x;
		drv->mdm_ctrl_flashing_warm_reset =
				mdm_ctrl_flashing_warm_reset_6x6x;
		drv->mdm_ctrl_power_off = mdm_ctrl_power_off_6x6x;
		break;
	case MODEM_6360:
	case MODEM_7160:
		drv->mdm_ctrl_cold_boot = mdm_ctrl_cold_boot_7x6x;
		drv->mdm_ctrl_cold_reset = mdm_ctrl_cold_reset_7x6x;
		drv->mdm_ctrl_normal_warm_reset =
				mdm_ctrl_normal_warm_reset_7x6x;
		drv->mdm_ctrl_silent_warm_reset =
				mdm_ctrl_silent_warm_reset_7x6x;
		drv->mdm_ctrl_flashing_warm_reset =
				mdm_ctrl_flashing_warm_reset_7x6x;
		drv->mdm_ctrl_power_off = mdm_ctrl_power_off_7x6x;
		break;
	default:
		drv->mdm_ctrl_cold_boot = mdm_ctrl_cold_boot_6x6x;
		drv->mdm_ctrl_cold_reset = mdm_ctrl_cold_reset_6x6x;
		drv->mdm_ctrl_normal_warm_reset =
				mdm_ctrl_normal_warm_reset_6x6x;
		drv->mdm_ctrl_silent_warm_reset =
				mdm_ctrl_silent_warm_reset_6x6x;
		drv->mdm_ctrl_flashing_warm_reset =
				mdm_ctrl_flashing_warm_reset_6x6x;
		drv->mdm_ctrl_power_off = mdm_ctrl_power_off_6x6x;
		break;
	}
}

/*
 * @brief: Get platform and modem data
 *
 */
void mdm_ctrl_get_device_info(struct mdm_ctrl *drv)
{
	struct mdm_ctrl_pdata *pdata;
	int is_ctplt = 0;

#ifdef MDM_CTRL_CTPLT
	is_ctplt = 1;
#endif

	/* FIXME: Revisit on IAFW update*/
	pdata = (struct mdm_ctrl_pdata *)modem_ctrl_platform_data(&is_ctplt);

	drv->pdata = pdata;

	if (pdata->is_mdm_ctrl_disabled)
		goto out;

	mdm_ctrl_set_func(drv, pdata->modem);

out:
	return;
}
