/*
 *  psh.c - Merrifield PSH IA side driver
 *
 *  (C) Copyright 2012 Intel Corporation
 *  Author: Alek Du <alek.du@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA
 */

/*
 * PSH IA side driver for Merrifield Platform
 */

#define VPROG2_SENSOR

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/pci.h>
#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <asm/intel_psh_ipc.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include "psh_ia_common.h"

#ifdef VPROG2_SENSOR
#include <asm/intel_scu_ipcutil.h>
#endif

#define APP_IMR_SIZE (1024 * 126)

struct psh_plt_priv {
	struct page *imr;	/* hack as imr before Chabbi ready */
	struct device *hwmon_dev;
};

int process_send_cmd(struct psh_ia_priv *psh_ia_data,
			int ch, struct ia_cmd *cmd, int len)
{
	int i, j;
	int ret = 0;
	u8 *pcmd = (u8 *)cmd;
	struct psh_msg in;

	/* map from virtual channel to real channel */
	ch = ch - PSH2IA_CHANNEL0 + PSH_SEND_CH0;

	for (i = 0; i < len; i += 7) {
		u8 left = len - i;
		u8 *ptr = (u8 *)&in;

		memset(&in, 0, sizeof(in));

		if (left > 7) {
			left = 7;
			in.msg |= PSH_IPC_CONTINUE;
		}

		for (j = 0; j < left; j++) {
			if (j == 3)
				ptr++;
			*ptr = *pcmd;
			ptr++;
			pcmd++;
		}

		ret = intel_ia2psh_command(&in, NULL, ch, 100000);
		if (ret) {
			pr_err("sendcmd through IPC channel fail!\n");
			return -1;
		}
	}

	return 0;
}


int do_setup_ddr(struct device *dev)
{
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(dev);
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)ia_data->platform_priv;
	u32 ddr_phy = page_to_phys(ia_data->pg);
	u32 imr_phy = page_to_phys(plt_priv->imr);
	const struct firmware *fw_entry;
	struct ia_cmd cmd_user = {
		.cmd_id = CMD_SETUP_DDR,
		.sensor_id = 0,
		};

#ifdef VPROG2_SENSOR
	intel_scu_ipc_msic_vprog2(1);
#endif
	if (!request_firmware(&fw_entry, "psh.bin", dev)) {
		pr_debug("psh fw size %d virt:0x%p\n",
				fw_entry->size, fw_entry->data);
		if (fw_entry->size > APP_IMR_SIZE) {
			pr_err("psh fw size too big\n");
		} else {
			struct ia_cmd cmd = {
				.cmd_id = CMD_RESET,
				.sensor_id = 0,
				};

			memcpy(page_address(plt_priv->imr), fw_entry->data,
				fw_entry->size);
			*(u32 *)(&cmd.param) = imr_phy;
			cmd.tran_id = 0x1;
			if (ia_send_cmd(ia_data, PSH2IA_CHANNEL3, &cmd, 7))
				return -1;
			ia_data->load_in_progress = 1;
			wait_for_completion_timeout(&ia_data->cmd_load_comp,
					3 * HZ);
		}
		release_firmware(fw_entry);
	}
	*(unsigned long *)(&cmd_user.param) = ddr_phy;
	return ia_send_cmd(ia_data, PSH2IA_CHANNEL0, &cmd_user, 7);
}

static void psh2ia_channel_handle(u32 msg, u32 param, void *data)
{
	struct pci_dev *pdev = (struct pci_dev *)data;
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&pdev->dev);

	ia_process_lbuf(&pdev->dev);
	if (unlikely(ia_data->load_in_progress)) {
		ia_data->load_in_progress = 0;
		complete(&ia_data->cmd_load_comp);
	}
}

static int psh_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret = -1;
	struct psh_ia_priv *ia_data;
	struct psh_plt_priv *plt_priv;

	/* No resource for this PCI device, it's only for probe */
	/*
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "fail to enable psh pci device\n");
		goto pci_err;
	}
	*/
	plt_priv = kzalloc(sizeof(*plt_priv), GFP_KERNEL);
	if (!plt_priv) {
		dev_err(&pdev->dev, "can not allocate plt_priv\n");
		goto plt_err;
	}

	plt_priv->imr = alloc_pages(GFP_KERNEL | GFP_DMA32 | __GFP_ZERO,
			get_order(APP_IMR_SIZE));
	if (!plt_priv->imr) {
		dev_err(&pdev->dev, "can not allocate app imr buffer\n");
		goto imr_err;
	}

	ret = psh_ia_common_init(&pdev->dev, &ia_data);
	if (ret) {
		dev_err(&pdev->dev, "fail to init psh_ia_common\n");
		goto psh_ia_err;
	}

	plt_priv->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (!plt_priv->hwmon_dev) {
		dev_err(&pdev->dev, "fail to register hwmon device\n");
		goto hwmon_err;
	}

	ia_data->platform_priv = plt_priv;

	ret = intel_psh_ipc_bind(PSH_RECV_CH0, psh2ia_channel_handle, pdev);
	if (ret) {
		dev_err(&pdev->dev, "fail to bind channel\n");
		goto irq_err;
	}

	/* just put this dev into suspend status always, since this is fake */
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;

irq_err:
	hwmon_device_unregister(plt_priv->hwmon_dev);
hwmon_err:
	psh_ia_common_deinit(&pdev->dev);
psh_ia_err:
	__free_pages(plt_priv->imr, get_order(APP_IMR_SIZE));
imr_err:
	kfree(plt_priv);
plt_err:
	/* pci_dev_put(pdev);
pci_err: */
	return ret;
}

static void psh_remove(struct pci_dev *pdev)
{
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&pdev->dev);
	struct psh_plt_priv *plt_priv =
			(struct psh_plt_priv *)ia_data->platform_priv;

	__free_pages(plt_priv->imr, get_order(APP_IMR_SIZE));

	intel_psh_ipc_unbind(PSH_RECV_CH0);

	hwmon_device_unregister(plt_priv->hwmon_dev);

	kfree(plt_priv);

	psh_ia_common_deinit(&pdev->dev);

	/* pci_dev_put(pdev); */
}

#ifdef CONFIG_PM_RUNTIME
static int psh_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "runtime suspend called\n");
	return 0;
}

static int psh_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "runtime resume called\n");
	return 0;
}

#else

#define psh_ipc_runtime_suspend	NULL
#define psh_ipc_runtime_resume	NULL

#endif

static const struct dev_pm_ops psh_drv_pm_ops = {
	.runtime_suspend	= psh_runtime_suspend,
	.runtime_resume		= psh_runtime_resume,
};

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x11a4)},
	{ 0,}
};

MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver psh_driver = {
	.name = "psh",
	.driver = {
		.pm = &psh_drv_pm_ops,
	},
	.id_table = pci_ids,
	.probe	= psh_probe,
	.remove	= psh_remove,
};

static int __init psh_init(void)
{
	return pci_register_driver(&psh_driver);
}

static void __exit psh_exit(void)
{
	pci_unregister_driver(&psh_driver);
}

module_init(psh_init);
module_exit(psh_exit);
MODULE_LICENSE("GPL v2");
