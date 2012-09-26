/*
 * rpmsg_mid_rpmsg.c - Intel RPMSG Driver
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/rpmsg.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/suspend.h>

#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>


/* Instance for generic kernel IPC calls */
static struct rpmsg_instance *rpmsg_ipc_instance;

static bool rpmsg_notifier_init;
static int  rpmsg_pm_callback(struct notifier_block *nb,
					unsigned long action,
					void *ignored);

static struct notifier_block rpmsg_pm_notifier = {
	.notifier_call = rpmsg_pm_callback,
	.priority = 2,
};

/* Suspend status*/
static bool rpmsg_suspend_status;
static DEFINE_MUTEX(rpmsg_suspend_lock);

/* Suspend status get */
static bool suspend_in_progress(void)
{
	return rpmsg_suspend_status;
}

/* Suspend status set */
static void set_suspend_status(bool status)
{
	mutex_lock(&rpmsg_suspend_lock);
	rpmsg_suspend_status = status;
	mutex_unlock(&rpmsg_suspend_lock);
}

/* RPMSG PM notifier callback */
static int rpmsg_pm_callback(struct notifier_block *nb,
					unsigned long action,
					void *ignored)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		set_suspend_status(true);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		set_suspend_status(false);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

int rpmsg_send_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub, u8 *in,
						u32 *out, u32 inlen,
						u32 outlen)
{
	int ret = 0;

	if (!instance) {
		pr_err("%s: Instance is NULL\n", __func__);
		return -EFAULT;
	}

	mutex_lock(&rpmsg_suspend_lock);

	if (!suspend_in_progress())
		wake_lock(&instance->wake_lock);

	mutex_lock(&instance->instance_lock);

	/* Prepare Tx buffer */
	instance->tx_msg->cmd = cmd;
	instance->tx_msg->sub = sub;
	instance->tx_msg->in = in;
	instance->tx_msg->out = out;
	instance->tx_msg->inlen = inlen;
	instance->tx_msg->outlen = outlen;

	/* Preapre Rx buffer */
	mutex_lock(&instance->rx_lock);
	instance->rx_msg->status = -1;
	mutex_unlock(&instance->rx_lock);
	INIT_COMPLETION(instance->reply_arrived);

	/* Send message to remote processor(SCU) using rpdev channel */
	ret = rpmsg_send_offchannel(
					instance->rpdev,
					instance->endpoint->addr,
					instance->rpdev->dst,
					instance->tx_msg,
					sizeof(*instance->tx_msg)
					);
	if (ret) {
		dev_err(&instance->rpdev->dev, "%s failed: %d\n",
						 __func__, ret);
		goto end;
	}

	if (0 == wait_for_completion_timeout(&instance->reply_arrived,
						RPMSG_TX_TIMEOUT)) {
		dev_err(&instance->rpdev->dev,
				"timeout: %d\n", ret);
		ret = -ETIMEDOUT;
		goto end;
	}

	mutex_lock(&instance->rx_lock);
	ret = instance->rx_msg->status;
	mutex_unlock(&instance->rx_lock);
end:
	mutex_unlock(&instance->instance_lock);

	if (!suspend_in_progress())
		wake_unlock(&instance->wake_lock);

	mutex_unlock(&rpmsg_suspend_lock);
	return ret;
}
EXPORT_SYMBOL(rpmsg_send_command);

int rpmsg_send_raw_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub, u8 *in,
						u32 *out, u32 inlen,
						u32 outlen, u32 sptr,
						u32 dptr)
{
	int ret = 0;

	if (!instance) {
		pr_err("%s: Instance is NULL\n", __func__);
		return -EFAULT;
	}

	mutex_lock(&instance->instance_lock);
	instance->tx_msg->sptr = sptr;
	instance->tx_msg->dptr = dptr;
	mutex_unlock(&instance->instance_lock);

	ret = rpmsg_send_command(instance, cmd, sub, in, out, inlen, outlen);

	return ret;
}
EXPORT_SYMBOL(rpmsg_send_raw_command);

int rpmsg_send_simple_command(struct rpmsg_instance *instance, u32 cmd,
						u32 sub)
{
	int ret;

	ret = rpmsg_send_command(instance, cmd, sub, NULL, NULL, 0, 0);

	return ret;
}
EXPORT_SYMBOL(rpmsg_send_simple_command);

static void rpmsg_recv_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	static int rx_count;
	struct rpmsg_instance *instance = priv;

	if (len != sizeof(struct rx_ipc_msg)) {
		dev_warn(&rpdev->dev, "%s, incorrect msg length\n", __func__);
		return;
	}

#ifdef DEBUG_RPMSG_MSG
	dev_info(&rpdev->dev, "incoming msg %d (src: 0x%x)\n", ++rx_count, src);

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
#endif

	mutex_lock(&instance->rx_lock);

	memcpy(instance->rx_msg, data, len);

	mutex_unlock(&instance->rx_lock);

	complete(&instance->reply_arrived);

}

int alloc_rpmsg_instance(struct rpmsg_channel *rpdev,
				struct rpmsg_instance **pInstance)
{
	int ret = 0;
	struct rpmsg_instance *instance;

	dev_info(&rpdev->dev, "Allocating rpmsg_instance\n");

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance) {
		ret = -ENOMEM;
		dev_err(&rpdev->dev, "kzalloc rpmsg_instance failed\n");
		goto alloc_out;
	}

	instance->rpdev = rpdev;

	instance->tx_msg = kzalloc(sizeof(struct tx_ipc_msg), GFP_KERNEL);
	if (!instance->tx_msg) {
		ret = -ENOMEM;
		dev_err(&rpdev->dev, "kzalloc instance tx_msg failed\n");
		goto error_tx_msg_create;
	}

	instance->rx_msg = kzalloc(sizeof(struct rx_ipc_msg), GFP_KERNEL);
	if (!instance->rx_msg) {
		ret = -ENOMEM;
		dev_err(&rpdev->dev, "kzalloc instance rx_msg failed\n");
		goto error_rx_msg_create;
	}

	instance->endpoint = rpmsg_create_ept(rpdev, rpmsg_recv_cb,
							instance,
							RPMSG_ADDR_ANY);
	if (!instance->endpoint) {
		dev_err(&rpdev->dev, "create instance endpoint failed\n");
		ret = -ENOMEM;
		goto error_endpoint_create;
	}

	wake_lock_init(&instance->wake_lock, WAKE_LOCK_SUSPEND,
			dev_name(&rpdev->dev));

	if (!rpmsg_notifier_init) {
		rpmsg_notifier_init = true;
		register_pm_notifier(&rpmsg_pm_notifier);
	}

	goto alloc_out;

error_endpoint_create:
	kfree(instance->rx_msg);
	instance->rx_msg = NULL;
error_rx_msg_create:
	kfree(instance->tx_msg);
	instance->tx_msg = NULL;
error_tx_msg_create:
	kfree(instance);
	instance = NULL;
alloc_out:
	*pInstance = instance;
	return ret;

}
EXPORT_SYMBOL(alloc_rpmsg_instance);

void free_rpmsg_instance(struct rpmsg_channel *rpdev,
				struct rpmsg_instance **pInstance)
{
	struct rpmsg_instance *instance = *pInstance;
	rpmsg_destroy_ept(instance->endpoint);
	kfree(instance->tx_msg);
	instance->tx_msg = NULL;
	kfree(instance->rx_msg);
	instance->rx_msg = NULL;
	kfree(instance);
	*pInstance = NULL;
	dev_info(&rpdev->dev, "Freeing rpmsg device\n");
}
EXPORT_SYMBOL(free_rpmsg_instance);

void init_rpmsg_instance(struct rpmsg_instance *instance)
{
	init_completion(&instance->reply_arrived);
	mutex_init(&instance->instance_lock);
	mutex_init(&instance->rx_lock);
}
EXPORT_SYMBOL(init_rpmsg_instance);

static int rpmsg_ipc_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	dev_info(&rpdev->dev, "Probed rpmsg_ipc device\n");

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}
	/* Allocate rpmsg instance for kernel IPC calls*/
	ret = alloc_rpmsg_instance(rpdev, &rpmsg_ipc_instance);
	if (!rpmsg_ipc_instance) {
		dev_err(&rpdev->dev, "kzalloc rpmsg_ipc instance failed\n");
		goto out;
	}
	/* Initialize rpmsg instance */
	init_rpmsg_instance(rpmsg_ipc_instance);
out:
	return ret;
}

static void __devexit rpmsg_ipc_remove(struct rpmsg_channel *rpdev)
{
	free_rpmsg_instance(rpdev, &rpmsg_ipc_instance);
	dev_info(&rpdev->dev, "Removed rpmsg_ipc device\n");
}

static void rpmsg_ipc_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id rpmsg_ipc_id_table[] = {
	{ .name	= "rpmsg_ipc" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_ipc_id_table);

static struct rpmsg_driver rpmsg_ipc = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_ipc_id_table,
	.probe		= rpmsg_ipc_probe,
	.callback	= rpmsg_ipc_cb,
	.remove		= __devexit_p(rpmsg_ipc_remove),
};

static int __init rpmsg_ipc_init(void)
{
	return register_rpmsg_driver(&rpmsg_ipc);
}
subsys_initcall(rpmsg_ipc_init);

static void __exit rpmsg_ipc_exit(void)
{
	return unregister_rpmsg_driver(&rpmsg_ipc);
}
module_exit(rpmsg_ipc_exit);

MODULE_AUTHOR("Ning Li<ning.li@intel.com>");
MODULE_DESCRIPTION("Intel IPC RPMSG Driver");
MODULE_LICENSE("GPL v2");
