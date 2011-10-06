/* drivers/i2c/chips/a1026.c - a1026 voice processor driver
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/a1026.h>
#include <linux/firmware.h>
/* FIXME: once sound moves we can fix this */
#include "../staging/intel_sst/intel_sst.h"

#define DEBUG			(0)
#define ENABLE_DIAG_IOCTLS	(0)

/*
 * This driver is based on the eS305-UG-APIGINTEL-V0 2.pdf spec
 * for the eS305 Voice Processor
 */

struct vp_ctxt {
	unsigned char *data;
	unsigned int img_size;
	struct i2c_client *i2c_dev;
	struct a1026_platform_data *pdata;
} *es305;

static int execute_cmdmsg(unsigned int msg, struct vp_ctxt *vp);
static int suspend(struct vp_ctxt *vp);
static struct mutex a1026_lock;
static int es305_opened;
static int es305_suspended;

enum bool {gpio_l, gpio_h};

static int es305_i2c_read(char *rxData, int length, struct vp_ctxt *the_vp)
{
	int rc;
	struct i2c_client *client = the_vp->i2c_dev;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(client->adapter, msgs, 1);
	if (rc < 0) {
		pr_debug("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i;
		for (i = 0; i < length; i++)
			pr_debug("%s: rx[%d] = %2x\n", __func__, i, rxData[i]);
	}
#endif

	return 0;
}

static int es305_i2c_write(char *txData, int length, struct vp_ctxt *the_vp)
{
	int rc;
	struct i2c_client *client = the_vp->i2c_dev;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc < 0) {
		pr_debug("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int;
		for (i = 0; i < length; i++)
			pr_debug("%s: tx[%d] = %2x\n", __func__, i, txData[i]);
	}
#endif

	return 0;
}

static void es305_i2c_sw_reset(unsigned int reset_cmd, struct vp_ctxt *vp)
{
	int rc;
	unsigned char msgbuf[4];

	msgbuf[0] = (reset_cmd >> 24) & 0xFF;
	msgbuf[1] = (reset_cmd >> 16) & 0xFF;
	msgbuf[2] = (reset_cmd >> 8) & 0xFF;
	msgbuf[3] = reset_cmd & 0xFF;

	pr_debug("%s: %08x\n", __func__, reset_cmd);

	rc = es305_i2c_write(msgbuf, 4, vp);
	if (!rc)
		msleep(20);
		/* 20ms is recommended polling period -- p8 spec*/
}

static int es305_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	mutex_lock(&a1026_lock);
	if (es305_opened) {
		pr_debug("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	file->private_data = es305;
	es305->img_size = 0;
	es305_opened = 1;
done:
	mutex_unlock(&a1026_lock);
	return rc;
}


static int es305_release(struct inode *inode, struct file *file)
{
	mutex_lock(&a1026_lock);
	es305_opened = 0;
	mutex_unlock(&a1026_lock);

	return 0;
}

static int suspend(struct vp_ctxt *vp)
{
	int rc;

	/* Put es305 into sleep mode */
	rc = execute_cmdmsg(A100_msg_Sleep, vp);
	if (rc < 0) {
		pr_debug("%s: suspend error\n", __func__);
		goto set_suspend_err;
	}

	es305_suspended = 1;
	msleep(120); /* 120 defined by fig 2 of eS305 as the time to wait
			before clock gating */
	rc = intel_sst_set_pll(false, SST_PLL_AUDIENCE);
	if (rc)
		pr_err("ipc clk disable command failed: %d\n", rc);
set_suspend_err:

	return rc;
}

static ssize_t es305_bootup_init(struct vp_ctxt *vp)
{
	int rc, pass = 0;
	int remaining;
	int retry = RETRY_CNT;
	int i;
	unsigned char *index;
	char buf[2];
	const struct firmware *fw_entry;

	if (request_firmware(&fw_entry, "vpimg.bin", &vp->i2c_dev->dev)) {
		dev_err(&vp->i2c_dev->dev, "Firmware not available\n");
		return -EFAULT;
	}

	if (fw_entry->size > A1026_MAX_FW_SIZE) {
		pr_err("%s: invalid es305 image size %d\n", __func__,
				fw_entry->size);
		return -EINVAL;
	}

	while (retry--) {
		/* Reset es305 chip */
		vp->pdata->reset(gpio_l);
		/* Take out of reset */
		vp->pdata->reset(gpio_h);
		msleep(50); /* Delay defined in Figure 1 of eS305 spec */


		/* Boot Cmd to es305 */
		buf[0] = A1026_msg_BOOT >> 8;
		buf[1] = A1026_msg_BOOT & 0xff;

		rc = es305_i2c_write(buf, 2, vp);
		if (rc < 0) {
			pr_debug("%s: set boot mode error (%d retries left)\n",
					__func__, retry);
			continue;
		}

		mdelay(1); /* eS305 internal delay */
		rc = es305_i2c_read(buf, 1, vp);
		if (rc < 0) {
			pr_debug("%s: boot mode ack error (%d retries left)\n",
					__func__, retry);
			continue;
		}

		if (buf[0] != A1026_msg_BOOT_ACK) {
			pr_debug("%s: not a boot-mode ack (%d retries left)\n",
					__func__, retry);
			continue;
		}
		pr_debug("%s:ACK =  %d\n",
				__func__, buf[0]);
		remaining = fw_entry->size;
		index = fw_entry->data;

		pr_debug("%s: starting to load image (%d passes)...\n",
				__func__,
				remaining);

		for (i = 0; i < remaining; i++) {
			rc = es305_i2c_write(index, 1, vp);
			index++;
			if (rc < 0)
				break;
		}
		pr_debug("%s: starting to load image (%s index)...\n",
				__func__,
				index);

		if (rc < 0) {
			pr_debug("%s: fw load error %d (%d retries left)\n",
					__func__, rc, retry);
			continue;
		}

		msleep(100); /* Delay time before issue a Sync Cmd
				BUGBUG should be 10*/

		pr_debug("%s: firmware loaded successfully\n", __func__);

		rc = execute_cmdmsg(A100_msg_Sync, vp);
		if (rc < 0) {
			pr_debug("%s: sync command error %d (%d retries left)\n",
					__func__, rc, retry);
			continue;
		}

		pass = 1;
		break;
	}
	rc = suspend(vp);
	if (pass && !rc)
		pr_debug("%s: initialized!\n", __func__);
	else
		pr_err("%s: initialization failed\n", __func__);

	release_firmware(fw_entry);
	return rc;
}

static ssize_t chk_wakeup_es305(struct vp_ctxt *the_vp)
{
	int rc = 0, retry = 3;

	if (es305_suspended == 1) {
		the_vp->pdata->wakeup(gpio_l);
		msleep(120); /* fig 3 eS305 spec.  BUGBUG should be 30 */

		do {
			rc = execute_cmdmsg(A100_msg_Sync, the_vp);
		} while ((rc < 0) && --retry);

		the_vp->pdata->wakeup(gpio_h);
		if (rc < 0) {
			pr_err("%s: failed (%d)\n", __func__, rc);
			goto wakeup_sync_err;
		}

		es305_suspended = 0;
	}
wakeup_sync_err:
	return rc;
}

int execute_cmdmsg(unsigned int msg, struct vp_ctxt *vp)
{
	int rc;
	int retries, pass = 0;
	unsigned char msgbuf[4];
	unsigned char chkbuf[4];
	unsigned int sw_reset;

	sw_reset = ((A100_msg_BootloadInitiate << 16) | RESET_IMMEDIATE);

	msgbuf[0] = (msg >> 24) & 0xFF;
	msgbuf[1] = (msg >> 16) & 0xFF;
	msgbuf[2] = (msg >> 8) & 0xFF;
	msgbuf[3] = msg & 0xFF;

	memcpy(chkbuf, msgbuf, 4);

	rc = es305_i2c_write(msgbuf, 4, vp);
	if (rc < 0) {
		pr_debug("%s: error %d\n", __func__, rc);
		es305_i2c_sw_reset(sw_reset, vp);
		return rc;
	}

	/* We don't need to get Ack after sending out a suspend command */
	if (msg == A100_msg_Sleep)
		return rc;

	retries = POLLING_RETRY_CNT;
	msleep(1); /* BUGBUG should be 20 p8 spec */
	while (retries--) {
		rc = 0;

		memset(msgbuf, 0, sizeof(msgbuf));
		rc = es305_i2c_read(msgbuf, 4, vp);
		if (rc < 0) {
			pr_debug("%s: ack-read error %d (%d retries)\n"
			, __func__, rc, retries);
			continue;
		}

		if (msgbuf[0] == 0x80  && msgbuf[1] == chkbuf[1]) {
			pass = 1;
			pr_debug("%s: ACK OF SYNC CMD\n", __func__);
			break;
		} else if (msgbuf[0] == 0xff && msgbuf[1] == 0xff) {
			pr_debug("%s: illegal cmd %08x\n", __func__, msg);
			rc = -EINVAL;
			break;
		} else if (msgbuf[0] == 0x00 && msgbuf[1] == 0x00) {
			pr_debug("%s: not ready (%d retries)\n", __func__,
					retries);
			rc = -EBUSY;
		} else {
			pr_debug("%s: cmd/ack mismatch: (%d retries left)\n",
					__func__,
					retries);
#if DEBUG
			pr_debug("%s: msgbuf[0] = %x\n", __func__, msgbuf[0]);
			pr_debug("%s: msgbuf[1] = %x\n", __func__, msgbuf[1]);
			pr_debug("%s: msgbuf[2] = %x\n", __func__, msgbuf[2]);
			pr_debug("%s: msgbuf[3] = %x\n", __func__, msgbuf[3]);
#endif
			rc = -EBUSY;
		}
		msleep(20); /* eS305 spec p. 8 : use polling */
	}

	if (!pass) {
		pr_err("%s: failed execute cmd %08x (%d)\n", __func__,
				msg, rc);
		es305_i2c_sw_reset(sw_reset, vp);
	}
	return rc;
}


static ssize_t es305_write(struct file *file, char __user *buff,
	size_t count, loff_t *offp)
{
	int rc, msg_buf_count, i, num_fourbyte;
	unsigned char *kbuf;
	unsigned int sw_reset;
	struct vp_ctxt *the_vp;
	unsigned char msgbuf[4];
	int size_cmd_snd = 4;

	the_vp = file->private_data;
	if (!the_vp)
			return -EINVAL;
	sw_reset = ((A100_msg_BootloadInitiate << 16) | RESET_IMMEDIATE);
	rc = chk_wakeup_es305(the_vp);
	if (rc < 0)
		return rc;
	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (copy_from_user(kbuf, buff, count)) {
		kfree(kbuf);
		return -EFAULT;
	}
#if DEBUG
	{

		size_cmd_snd = count;
		for (i = 0; i < count; i++)
			pr_debug("%s: kbuf %x\n", __func__, kbuf[i]);
	}
#endif
	msg_buf_count = 0;
	num_fourbyte = count / size_cmd_snd;
	for (i = 0; i < num_fourbyte ; i++) {
		rc = es305_i2c_write(&kbuf[msg_buf_count],
		size_cmd_snd, the_vp);
		if (rc < 0) {
			pr_err("ES305 CMD block write error!\n");
			es305_i2c_sw_reset(sw_reset, the_vp);
			kfree(kbuf);
			return rc;
		}
		mdelay(1);
#if DEBUG
	{
		memset(msgbuf, 0, sizeof(msgbuf));
		rc = es305_i2c_read(msgbuf, 4, the_vp);
		if (rc < 0) {
			pr_err("ES305 CMD block read error!\n");
			es305_i2c_sw_reset(sw_reset, the_vp);
			return rc;
		}
		if (!(kbuf[msg_buf_count] == msgbuf[0] &&
			kbuf[msg_buf_count + 1] == msgbuf[1] &&
			kbuf[msg_buf_count + 2] == msgbuf[2] &&
			kbuf[msg_buf_count + 3] == msgbuf[3])) {
			pr_err("E305 cmd not ack'ed\n");
			return -ENXIO;
		}
			pr_debug("ES305 WRITE:(0x%.2x%.2x%.2x%.2x)\n",
			kbuf[msg_buf_count], kbuf[msg_buf_count + 1],
			kbuf[msg_buf_count + 2], kbuf[msg_buf_count + 3]);
			pr_debug("ES305 READ:(0x%.2x%.2x%.2x%.2x)\n",
			msgbuf[0], msgbuf[1], msgbuf[2], msgbuf[3]);
		}
#endif
		msg_buf_count += 4;
	}
	kfree(kbuf);
	return count;
}

static ssize_t es305_read(struct file *file, char __user *buff,
	size_t count, loff_t *offp)
{
	unsigned char kbuf[4];
	struct vp_ctxt *the_vp;

	the_vp = file->private_data;
	if (!the_vp)
		return -EINVAL;
	es305_i2c_read(kbuf, 4, the_vp);
#if DEBUG
	{
		int i;
		for (i = 0; i < 4; i++)
			pr_debug("%s: kbuf %x\n", __func__, kbuf[i]);
	}
#endif
	if (copy_to_user(buff, kbuf, 4))
		return -EFAULT;
	return 4;
}



static long a1026_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct vp_ctxt *the_vp;
	void __user *argp = (void __user *)arg;
	int rc;
	pr_debug("%s: ioctl start\n", __func__);
	if (file && file->private_data)
		the_vp = file->private_data;
	else
		return -EINVAL;
	pr_debug("%s: ioctl vp Ok\n", __func__);

	switch (cmd) {
	case A1026_BOOTUP_INIT:
		rc = es305_bootup_init(the_vp);
		break;
	case A1026_SUSPEND:
		rc = suspend(the_vp);
		if (rc < 0)
			pr_err("suspend error\n");
		break;
	case A1026_ENABLE_CLOCK:
		pr_debug("%s:ipc clk enable command\n", __func__);
		rc = intel_sst_set_pll(true, SST_PLL_AUDIENCE);
		if (rc) {
			pr_err("ipc clk enable command failed: %d\n", rc);
			return rc;
		}
		break;
	default:
		pr_debug("%s: invalid command %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations a1026_fops = {
	.owner = THIS_MODULE,
	.open = es305_open,
	.release = es305_release,
	.write = es305_write,
	.read = es305_read,
	.unlocked_ioctl = a1026_ioctl,
};

static struct miscdevice a1026_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_es305",
	.fops = &a1026_fops,
};

static int a1026_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	struct vp_ctxt *the_vp;
	struct a1026_platform_data *pdata;
	dev_dbg(&client->dev, "probe\n");

	the_vp = kzalloc(sizeof(struct vp_ctxt), GFP_KERNEL);
	if (!the_vp) {
		rc = -ENOMEM;
		dev_err(&client->dev, "platform data is out of memory\n");
		goto err_exit;
	}
	the_vp->i2c_dev = client;
	i2c_set_clientdata(client, the_vp);

	pdata = client->dev.platform_data;
	if (!pdata) {
		rc = -EINVAL;
		dev_err(&client->dev, "platform data is invalid\n");
		goto err_kfree;
	}
	rc = pdata->request_resources(client);
	if (rc) {
		dev_err(&client->dev, "Cannot get ressources\n");
		goto err_kfree;
	}
	mutex_init(&a1026_lock);
	rc = misc_register(&a1026_device);
	if (rc) {
		dev_err(&client->dev, "es305_device register failed\n");
		goto err_misc_register;
	}

	es305 = the_vp;
	es305->pdata = pdata;
	pdata->wakeup(gpio_h);
	pdata->reset(gpio_h);

	return 0;

err_misc_register:
	mutex_destroy(&a1026_lock);
err_kfree:
	kfree(the_vp);
	i2c_set_clientdata(client, NULL);
err_exit:
	return rc;
}

static int a1026_remove(struct i2c_client *client)
{
	misc_deregister(&a1026_device);
	mutex_destroy(&a1026_lock);
	kfree(i2c_get_clientdata(client));
	i2c_set_clientdata(client, NULL);

	return 0;
}

static int a1026_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int a1026_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id a1026_id[] = {
	{ "audience_es305", 0 },
	{ }
};

static struct i2c_driver a1026_driver = {
	.probe = a1026_probe,
	.remove = a1026_remove,
	.suspend = a1026_suspend,
	.resume	= a1026_resume,
	.id_table = a1026_id,
	.driver = {
		.name = "audience_es305",
	},
};

static int __init a1026_init(void)
{
	pr_debug("AUDIENCE%s\n", __func__);

	return i2c_add_driver(&a1026_driver);
}

static void __exit a1026_exit(void)
{
	i2c_del_driver(&a1026_driver);
}

module_init(a1026_init);
module_exit(a1026_exit);

MODULE_DESCRIPTION("A1026 voice processor driver");
MODULE_LICENSE("GPL");
