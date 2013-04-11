/*
 * linux/drivers/modem_control/mdm_ctrl.c
 *
 * Version 1.0
 *
 * This code allows to power and reset IMC modems.
 * There is a list of commands available in include/linux/mdm_ctrl.h
 * Very first version of this code only supports the following modems :
 * - IMC6260
 * - IMC7160
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

#include "mdm_util.h"
#include "mdm_imc.h"

#define MDM_BOOT_DEVNAME	CONFIG_MDM_CTRL_DEV_NAME

#define MDM_MODEM_READY_DELAY	60 /* Modem readiness wait duration (sec) */

/*****************************************************************************
 *
 * Local driver functions
 *
 ****************************************************************************/

/**
 * This function handle the modem reset/coredump:
 *
 * @work: a reference to work queue element
 */
static void mdm_ctrl_handle_hangup(struct work_struct *work)
{
	struct mdm_ctrl *drv = mdm_drv;
	int modem_rst;

	/* Check the hangup reason */
	modem_rst = drv->hangup_causes;

	if (modem_rst & MDM_CTRL_HU_RESET)
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_WARM_BOOT);

	if (modem_rst & MDM_CTRL_HU_COREDUMP)
		mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_COREDUMP);

	flush_workqueue(drv->change_state_wq);

	pr_info(DRVNAME ": %s (reasons: 0x%X)\n", __func__, drv->hangup_causes);
}


/**
 * mdm_ctrl_coredump_it	-	Modem has signaled a core dump
 *
 */
static irqreturn_t mdm_ctrl_coredump_it(int irq, void *data)
{
	struct mdm_ctrl *drv = data;

	pr_err(DRVNAME": CORE_DUMP 0x%x", gpio_get_value(drv->gpio_cdump));

	/* Ignoring event if we are in OFF state. */
	if (mdm_ctrl_get_state(drv) == MDM_CTRL_STATE_OFF) {
		pr_err(DRVNAME": CORE_DUMP while OFF\r\n");
		goto out;
	}

	/* Ignoring if Modem reset is ongoing. */
	if (mdm_ctrl_get_reset_ongoing(drv) == 1) {
		pr_err(DRVNAME": CORE_DUMP while Modem Reset is ongoing\r\n");
		goto out;
	}

	/* Set the reason & launch the work to handle the hangup */
	drv->hangup_causes |= MDM_CTRL_HU_COREDUMP;
	queue_work(drv->hu_wq, &drv->hangup_work);

out:
	return IRQ_HANDLED;
}

/**
 * mdm_ctrl_reset_it -	Modem has changed reset state
 *
 */
static irqreturn_t mdm_ctrl_reset_it(int irq, void *data)
{
	int value, reset_ongoing;
	struct mdm_ctrl *drv = data;
	unsigned long flags;

	value = gpio_get_value(drv->gpio_rst_out);

	/* Ignoring event if we are in OFF state. */
	if (mdm_ctrl_get_state(drv) == MDM_CTRL_STATE_OFF) {
		/* Logging event in order to minimise risk of hidding bug */
		pr_err(DRVNAME": RESET_OUT 0x%x while OFF\r\n", value);
		goto out;
	}

	reset_ongoing = mdm_ctrl_get_reset_ongoing(drv);
	if (reset_ongoing) {
		pr_err(DRVNAME": RESET_OUT 0x%x\r\n", value);

		/* Rising EDGE (IPC ready) */
		if (value) {
			spin_lock_irqsave(&drv->state_lck, flags);
			/* Reset the reset ongoing flag */
			mdm_ctrl_set_reset_ongoing(drv, 0);
			spin_unlock_irqrestore(&drv->state_lck, flags);

			pr_err(DRVNAME": IPC READY !\r\n");
			mdm_ctrl_launch_work(drv, MDM_CTRL_STATE_IPC_READY);
		}

		goto out;
	}

	pr_err(DRVNAME": Unexpected RESET_OUT 0x%x\r\n", value);

	/* Unexpected reset received */
	spin_lock_irqsave(&drv->state_lck, flags);
	mdm_ctrl_set_reset_ongoing(drv, 1);
	spin_unlock_irqrestore(&drv->state_lck, flags);

	/* Set the reason & launch the work to handle the hangup */
	drv->hangup_causes |= MDM_CTRL_HU_RESET;
	queue_work(drv->hu_wq, &drv->hangup_work);

out:
	return IRQ_HANDLED;
}

/**
 *
 *
 */
static int mdm_ctrl_free_gpios(struct mdm_ctrl *drv)
{
	if (drv->irq_cdump > 0)
		free_irq(drv->irq_cdump, NULL);

	drv->irq_cdump = 0;

	if (drv->irq_reset > 0)
		free_irq(drv->irq_reset, NULL);

	drv->irq_reset = 0;

	gpio_free(drv->gpio_cdump);
	gpio_free(drv->gpio_rst_out);
	gpio_free(drv->gpio_pwr_on);
	gpio_free(drv->gpio_rst_bbn);

	return 0;
}

/**
 * @brief Configure IRQs & GPIOs
 *
 * @param ch_ctx
 * @param dev
 *
 * @return
 */
static inline int
mdm_ctrl_configure_gpio(int gpio,
			int direction,
			int value,
			const char *desc)
{
	int ret;

	ret = gpio_request(gpio, "ifxHSIModem");

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		pr_err(DRVNAME": Unable to configure GPIO%d (%s)",
			 gpio,
			 desc);
		ret = -ENODEV;
	}

	return ret;
}

/**
 * @brief This function is:
 *	- requesting all needed gpios
 *	- requesting all needed irqs
 *	- registering irqs callbacks
 *
 * @param ch_ctx : Channel context
 * @param dev : Device driver info
 */
static int
mdm_ctrl_setup_irq_gpio(struct mdm_ctrl *drv)
{
	int ret;

	/* Configure the RESET_BB gpio */
	ret = mdm_ctrl_configure_gpio(drv->gpio_rst_bbn,
			1, 0, "RST_BB");
	if (ret)
		goto free_ctx4;

	/* Configure the ON gpio */
	ret = mdm_ctrl_configure_gpio(drv->gpio_pwr_on,
			1, 0, "ON");
	if (ret)
		goto free_ctx3;

	/* Configure the RESET_OUT gpio & irq */
	ret = mdm_ctrl_configure_gpio(drv->gpio_rst_out,
			0, 0, "RST_OUT");
	if (ret)
		goto free_ctx2;

	drv->irq_reset = gpio_to_irq(drv->gpio_rst_out);
	if (drv->irq_reset < 0) {
		ret = -ENODEV;
		goto free_ctx2;
	}

	ret = request_irq(drv->irq_reset,
			mdm_ctrl_reset_it,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING |
			IRQF_NO_SUSPEND,
			DRVNAME,
			drv);
	if (ret) {
		pr_err(DRVNAME": IRQ request failed for GPIO%d (RST_OUT)",
			 drv->gpio_rst_out);
		ret = -ENODEV;
		goto free_ctx2;
	}

	/* Configure the CORE_DUMP gpio & irq */
	ret = mdm_ctrl_configure_gpio(drv->gpio_cdump,
			0, 0, "CORE_DUMP");
	if (ret)
		goto free_all;

	drv->irq_cdump = gpio_to_irq(drv->gpio_cdump);
	if (drv->irq_cdump < 0) {
		ret = -ENODEV;
		goto free_all;
	}

	ret = request_irq(drv->irq_cdump,
			mdm_ctrl_coredump_it,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, DRVNAME,
			drv);
	if (ret) {
		pr_err(DRVNAME": IRQ request failed for GPIO%d (CORE DUMP)",
			drv->gpio_cdump);
		ret = -ENODEV;
		goto free_all;
	}

	pr_info(DRVNAME ": GPIO (rst_bbn: %d, pwr_on: %d, rst_out: %d, fcdp_rb: %d)\n",
			drv->gpio_rst_bbn,
			drv->gpio_pwr_on,
			drv->gpio_rst_out,
			drv->gpio_cdump);

	pr_info(DRVNAME ": IRQ  (rst_out: %d, fcdp_rb: %d)\n",
			drv->irq_reset, drv->irq_cdump);

	return ret;

free_all:
	mdm_ctrl_free_gpios(drv);
	return ret;

free_ctx2:
	if (drv->irq_reset > 0)
		free_irq(drv->irq_reset, NULL);

	drv->irq_reset = 0;
	gpio_free(drv->gpio_rst_out);
free_ctx3:
	gpio_free(drv->gpio_pwr_on);
free_ctx4:
	gpio_free(drv->gpio_rst_bbn);

	return ret;
}

/**
 * @brief This function will clear the hangup reasons
 *
 * @return 0
 */
static int clear_hangup_reasons(void)
{
	mdm_drv->hangup_causes = MDM_CTRL_NO_HU;
	return 0;
}

static int get_hangup_reasons(void)
{
	return mdm_drv->hangup_causes;
}


/*****************************************************************************
 *
 * Char device functions
 *
 ****************************************************************************/

/*
 * Called when a process tries to open the device file
 */
static int mdm_ctrl_dev_open(struct inode *inode, struct file *filp)
{
	mutex_lock(&mdm_drv->lock);
	/* Only ONE instance of this device can be opened */
	if (mdm_ctrl_get_opened(mdm_drv)) {
		mutex_unlock(&mdm_drv->lock);
		return -EBUSY;
	}

	/* Save private data for futur use */
	filp->private_data = mdm_drv;

	/* Set the open flag */
	mdm_ctrl_set_opened(mdm_drv, 1);
	mutex_unlock(&mdm_drv->lock);
	return 0;
}

/*
 * Called when a process closes the device file.
 */
static int mdm_ctrl_dev_close(struct inode *inode, struct file *filp)
{
	struct mdm_ctrl *drv = filp->private_data;

	/* Set the open flag */
	mutex_lock(&drv->lock);
	mdm_ctrl_set_opened(drv, 0);
	mutex_unlock(&drv->lock);
	return 0;
}


/**
 * @brief
 *
 * @param filep
 * @param cmd
 * @param arg
 *
 * @return
 */
long mdm_ctrl_dev_ioctl(struct file *filp,
		unsigned int cmd,
		unsigned long arg)
{
	struct mdm_ctrl *drv = filp->private_data;
	struct mdm_ctrl_cmd cmd_params;
	long ret = 0;
	unsigned int mdm_state;
	unsigned int param;

	pr_info(DRVNAME ": ioctl request 0x%x received \r\n", cmd);
	flush_workqueue(drv->change_state_wq);
	mdm_state = mdm_ctrl_get_state(drv);

	switch (cmd) {
	case MDM_CTRL_POWER_OFF:
		drv->mdm_ctrl_power_off(drv);
		break;

	case MDM_CTRL_POWER_ON:
		if ((mdm_state == MDM_CTRL_STATE_OFF) ||
			(mdm_state == MDM_CTRL_STATE_UNKNOWN))
				drv->mdm_ctrl_cold_boot(drv);
		 else
			if (mdm_state == MDM_CTRL_STATE_COREDUMP)
				pr_err(DRVNAME": Power ON not allowed (coredump)");
			else
				pr_info(DRVNAME": Powering on while already on");
		break;

	case MDM_CTRL_WARM_RESET:
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_normal_warm_reset(drv);
		else
			pr_err(DRVNAME": Warm reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_FLASHING_WARM_RESET:
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_flashing_warm_reset(drv);
		else
			pr_err(DRVNAME": Warm reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_COLD_RESET:
		if (mdm_state != MDM_CTRL_STATE_OFF)
			drv->mdm_ctrl_cold_reset(drv);
		else
			pr_err(DRVNAME": Cold reset not allowed (Modem OFF)");
		break;

	case MDM_CTRL_SET_STATE:
		/* Read the user command params */
		ret = copy_from_user(&param,
				(void *)arg,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			goto out;
		}

		/* FIXME: Allow any state ? */
		param &=
			(MDM_CTRL_STATE_OFF |
			MDM_CTRL_STATE_COLD_BOOT |
			MDM_CTRL_STATE_WARM_BOOT |
			MDM_CTRL_STATE_COREDUMP |
			MDM_CTRL_STATE_IPC_READY|
			MDM_CTRL_STATE_FW_DOWNLOAD_READY);

		mdm_ctrl_launch_work(drv, param);
		flush_workqueue(drv->change_state_wq);
		break;

	case MDM_CTRL_GET_STATE:
		param = mdm_state;

		ret = copy_to_user((void __user *)arg,
				&param,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME ": copy to user failed ret = %ld\r\n",
					ret);
			return ret;
		}
		break;

	case MDM_CTRL_WAIT_FOR_STATE:
		ret = copy_from_user(&cmd_params,
				(void __user *)arg,
				sizeof(cmd_params));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			break;
		}
		pr_err(DRVNAME": WAIT_FOR_STATE 0x%x ! \r\n", cmd_params.param);

		ret = wait_event_interruptible_timeout(drv->event,
			drv->modem_state == cmd_params.param,
			msecs_to_jiffies(cmd_params.timeout));
		if (!ret)
			pr_err(DRVNAME": WAIT_FOR_STATE timed out ! \r\n");
		break;

	case MDM_CTRL_GET_HANGUP_REASONS:
		param = get_hangup_reasons();

		ret = copy_to_user((void __user *)arg,
				&param,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME ": copy to user failed ret = %ld\r\n",
					ret);
			return ret;
		}


		break;

	case MDM_CTRL_CLEAR_HANGUP_REASONS:
		clear_hangup_reasons();
		break;

	case MDM_CTRL_SET_POLLED_STATES:
		/* Read the user command params */
		ret = copy_from_user(&param,
				(void *)arg,
				sizeof(param));
		if (ret < 0) {
			pr_info(DRVNAME": copy from user failed ret = %ld\r\n",
					ret);
			return ret;
		}
		drv->polled_states = param;
		if (waitqueue_active(&drv->wait_wq)) {
			flush_workqueue(drv->change_state_wq);
			mdm_state = mdm_ctrl_get_state(drv);
			if (mdm_state)
				drv->polled_state_reached = ((mdm_state & param)
								== mdm_state);
			wake_up(&drv->wait_wq);
		} else {
			/* Assume that mono threaded client are probably
			 * not polling yet and that they are not interested
			 * in the current state. This state may change until
			 * they start the poll. May be an issue for some cases.
			 */
			drv->polled_state_reached = false;
		}

		pr_info(DRVNAME ": states polled = 0x%x\r\n",
				drv->polled_states);
		break;

	default:
		pr_err(DRVNAME ": ioctl command %x unknown\r\n",
				cmd);
		ret = -ENOIOCTLCMD;
	}

out:
	return ret;
}

/**
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 */
static ssize_t mdm_ctrl_dev_read(struct file *filp,
				char __user *data,
				size_t count,
				loff_t *ppos)
{
	pr_err(DRVNAME": Nothing to read\r\n");
	return -EINVAL;
}

/**
 * Called when a process writes to dev file
 */
static ssize_t mdm_ctrl_dev_write(struct file *filp,
		const char __user *data,
		size_t count,
		loff_t *ppos)
{
	pr_err(DRVNAME": Nothing to write to\r\n");
	return -EINVAL;
}


/**
 * @brief
 *
 * @param filp
 * @param wait
 *
 * @return
 */
static unsigned int mdm_ctrl_dev_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct mdm_ctrl *drv = filp->private_data;
	unsigned int ret = 0;

	/* Wait event change */
	flush_workqueue(drv->change_state_wq);
	poll_wait(filp, &drv->wait_wq, pt);

	/* State notify */
	if (drv->polled_state_reached ||
		(mdm_ctrl_get_state(drv) & drv->polled_states)) {

		drv->polled_state_reached = false;
		ret |= POLLHUP|POLLRDNORM;
		pr_info(DRVNAME ": POLLHUP occured. Current state = 0x%x\r\n",
			 mdm_ctrl_get_state(drv));
	}

	return ret;
}


/**
 * Device driver file operations
 */
static const struct file_operations mdm_ctrl_ops = {
	.open	= mdm_ctrl_dev_open,
	.read	= mdm_ctrl_dev_read,
	.write	= mdm_ctrl_dev_write,
	.poll	= mdm_ctrl_dev_poll,
	.release = mdm_ctrl_dev_close,
	.unlocked_ioctl	= mdm_ctrl_dev_ioctl
};



/**
 * mdm_ctrl_module_init - initialises the Modem Boot driver
 *
 * Returns 0 on success or an error code
 */
static int __init mdm_ctrl_module_init(void)
{
	int ret;
	struct mdm_ctrl *new_drv;

	/* Allocate channel struct data */
	new_drv = kzalloc(sizeof(struct mdm_ctrl), GFP_KERNEL);
	if (!new_drv) {
		pr_err(DRVNAME ": Out of memory(new_drv)");
		ret = -ENOMEM;
		goto out;
	}

	pr_info(DRVNAME ": Getting device infos");
	/* Pre-initialisation: Retrieve platform device data*/
	mdm_ctrl_get_device_info(new_drv);

	if (new_drv->is_mdm_ctrl_disabled) {
		ret = -ENODEV;
		goto out;
	}

	/* Initialization */
	spin_lock_init(&new_drv->state_lck);
	mutex_init(&new_drv->lock);
	init_waitqueue_head(&new_drv->event);
	init_waitqueue_head(&new_drv->wait_wq);

	INIT_LIST_HEAD(&new_drv->next_state_link);

	INIT_WORK(&new_drv->change_state_work, mdm_ctrl_set_state);
	/* Create a high priority ordered workqueue to change modem state */
	new_drv->change_state_wq =
		 create_singlethread_workqueue(DRVNAME "-cs_wq");

	if (!new_drv->change_state_wq) {
		pr_err(DRVNAME ": Unable to create set state workqueue");
		ret = -EIO;
		goto free_drv;
	}

	INIT_WORK(&new_drv->hangup_work, mdm_ctrl_handle_hangup);

	/* Create a workqueue to manage hangup */
	new_drv->hu_wq = create_singlethread_workqueue(DRVNAME "-hu_wq");
	if (!new_drv->hu_wq) {
		pr_err(DRVNAME ": Unable to create control workqueue");
		ret = -EIO;
		goto free_change_state_wq;
	}

	/* Register the device */
	ret = alloc_chrdev_region(&new_drv->tdev, 0, 1, MDM_BOOT_DEVNAME);
	if (ret) {
		pr_err(DRVNAME ": alloc_chrdev_region failed (err: %d)", ret);
		goto free_hu_wq;
	}

	new_drv->major = MAJOR(new_drv->tdev);
	cdev_init(&new_drv->cdev, &mdm_ctrl_ops);
	new_drv->cdev.owner = THIS_MODULE;

	ret = cdev_add(&new_drv->cdev, new_drv->tdev, 1);
	if (ret) {
		pr_err(DRVNAME": cdev_add failed (err: %d)", ret);
		goto unreg_reg;
	}

	new_drv->class = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(new_drv->class)) {
		pr_err(DRVNAME": class_create failed (err: %d)", ret);
		ret = -EIO;
		goto del_cdev;
	}

	new_drv->dev = device_create(new_drv->class,
			NULL,
			new_drv->tdev,
			NULL, MDM_BOOT_DEVNAME);

	if (IS_ERR(new_drv->dev)) {
		pr_err(DRVNAME": device_create failed (err: %ld)",
				PTR_ERR(new_drv->dev));
		ret = -EIO;
		goto del_class;
	}

	mdm_ctrl_launch_work(new_drv, MDM_CTRL_STATE_OFF);
	flush_workqueue(new_drv->change_state_wq);

	mdm_ctrl_get_gpio(new_drv);

	if (mdm_ctrl_setup_irq_gpio(new_drv))
		goto del_dev;

	/* Everything is OK */
	mdm_drv = new_drv;

	/* Init driver */
	init_timer(&mdm_drv->flashing_timer);

	/* Modem power off sequence */
	if (new_drv->pdata->early_pwr_off)
		mdm_drv->mdm_ctrl_power_off(new_drv);

	/* Modem cold boot sequence */
	if (new_drv->pdata->early_pwr_on)
		mdm_drv->mdm_ctrl_cold_boot(new_drv);

	return 0;

del_dev:
	device_destroy(new_drv->class, new_drv->tdev);

del_class:
	class_destroy(new_drv->class);

del_cdev:
	cdev_del(&new_drv->cdev);

unreg_reg:
	unregister_chrdev_region(new_drv->tdev, 1);

free_hu_wq:
	destroy_workqueue(new_drv->hu_wq);

free_change_state_wq:
	destroy_workqueue(new_drv->change_state_wq);

free_drv:
	kfree(new_drv);

out:
	return ret;
}

/**
 * dlp_driver_exit - frees the resources taken by the boot driver
 */
static void __exit mdm_ctrl_module_exit(void)
{
	if (!mdm_drv)
		return;

	if (mdm_drv->pdata->is_mdm_ctrl_disabled)
		goto out;

	/* Delete the modem state worqueue */
	destroy_workqueue(mdm_drv->change_state_wq);

	/* Delete the modem hangup worqueue */
	destroy_workqueue(mdm_drv->hu_wq);

	/* Unregister the device */
	cdev_del(&mdm_drv->cdev);
	unregister_chrdev_region(mdm_drv->tdev, 1);
	class_destroy(mdm_drv->class);

	/* Free IRQs & GPIOs */
	free_irq(mdm_drv->irq_cdump, NULL);
	free_irq(mdm_drv->irq_reset, NULL);

	gpio_free(mdm_drv->gpio_cdump);
	gpio_free(mdm_drv->gpio_rst_out);
	gpio_free(mdm_drv->gpio_pwr_on);
	gpio_free(mdm_drv->gpio_rst_bbn);

	del_timer(&mdm_drv->flashing_timer);

	mutex_destroy(&mdm_drv->lock);

out:
	/* Free the boot driver context */
	kfree(mdm_drv);
	mdm_drv = NULL;
}

module_init(mdm_ctrl_module_init);
module_exit(mdm_ctrl_module_exit);

/**
 * mdm_ctrl_modem_reset - Reset modem
 */
static int mdm_ctrl_modem_reset(const char *val, struct kernel_param *kp)
{
	if (mdm_drv)
		mdm_drv->mdm_ctrl_silent_warm_reset(mdm_drv);
	return 0;
}

module_param_call(modem_reset, mdm_ctrl_modem_reset, NULL, NULL, 0644);

MODULE_AUTHOR("Faouaz Tenoutit <faouazx.tenoutit@intel.com>");
MODULE_AUTHOR("Frederic Berat <fredericx.berat@intel.com>");
MODULE_DESCRIPTION("Intel Modem control driver");
MODULE_LICENSE("GPL");
