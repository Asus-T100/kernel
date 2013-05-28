/* intel_sst_acpi.c - SST (LPE) driver init file for ACPI enumeration.
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 *  Authors:	Ramesh Babu K V <Ramesh.Babu@intel.com>
 *  Authors:	Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <acpi/acpi_bus.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

extern struct miscdevice lpe_ctrl;

int sst_workqueue_init(struct intel_sst_drv *ctx)
{
	pr_debug("%s", __func__);

	INIT_LIST_HEAD(&ctx->memcpy_list);
	INIT_LIST_HEAD(&ctx->libmemcpy_list);

	INIT_LIST_HEAD(&ctx->ipc_dispatch_list);
	INIT_LIST_HEAD(&ctx->block_list);
	INIT_WORK(&ctx->ipc_post_msg.wq, ctx->ops->post_message);
	INIT_WORK(&ctx->ipc_process_msg.wq, ctx->ops->process_message);
	INIT_WORK(&ctx->ipc_process_reply.wq, ctx->ops->process_reply);
	init_waitqueue_head(&ctx->wait_queue);

	ctx->mad_wq = create_singlethread_workqueue("sst_mad_wq");
	if (!ctx->mad_wq)
		goto err_wq;
	ctx->post_msg_wq =
		create_singlethread_workqueue("sst_post_msg_wq");
	if (!ctx->post_msg_wq)
		goto err_wq;
	ctx->process_msg_wq =
		create_singlethread_workqueue("sst_process_msg_wq");
	if (!ctx->process_msg_wq)
		goto err_wq;
	ctx->process_reply_wq =
		create_singlethread_workqueue("sst_proces_reply_wq");
	if (!ctx->process_reply_wq)
		goto err_wq;
	return 0;
err_wq:
	return -EBUSY;
}

void sst_init_locks(struct intel_sst_drv *ctx)
{
	mutex_init(&ctx->stream_lock);
	mutex_init(&ctx->sst_lock);
	mutex_init(&ctx->mixer_ctrl_lock);
	mutex_init(&ctx->sst_in_mem_lock);
	mutex_init(&ctx->csr_lock);

	spin_lock_init(&ctx->ipc_spin_lock);
	spin_lock_init(&ctx->block_lock);
	spin_lock_init(&ctx->pvt_id_lock);
}

int sst_destroy_workqueue(struct intel_sst_drv *ctx)
{
	pr_debug("%s", __func__);
	if (ctx->mad_wq)
		destroy_workqueue(ctx->mad_wq);
	if (ctx->post_msg_wq)
		destroy_workqueue(ctx->post_msg_wq);
	if (ctx->process_msg_wq)
		destroy_workqueue(ctx->process_msg_wq);
	if (ctx->process_reply_wq)
		destroy_workqueue(ctx->process_reply_wq);
	return 0;
}

#if IS_ENABLED(CONFIG_ACPI)
static int sst_platform_get_resources(struct intel_sst_drv *ctx,
				      struct platform_device *pdev)
{
	struct resource *rsrc;
	int irq, ret;

	pr_debug("%s", __func__);

	/* All ACPI resource request here */
	/* Get DDR addr from platform resource table */
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rsrc) {
		pr_err("Invalid DDR base from IFWI");
		return -EIO;
	}
	ctx->ddr_base = rsrc->start;
	ctx->ddr_end = rsrc->end;
	pr_debug("DDR base: %#x", ctx->ddr_base);
	ctx->ddr = devm_ioremap_nocache(ctx->dev, ctx->ddr_base,
					resource_size(rsrc));
	if (!ctx->ddr) {
		pr_err("unable to map DDR");
		return -EIO;
	}

	/* Get Shim addr from platform resource table */
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!rsrc) {
		pr_err("Invalid SHIM base from IFWI");
		return -EIO;
	}
	ctx->shim_phy_add = rsrc->start;
	pr_debug("SHIM base: %#x", ctx->shim_phy_add);
	ctx->shim = devm_ioremap_nocache(ctx->dev, ctx->shim_phy_add,
					 resource_size(rsrc));
	if (!ctx->shim) {
		pr_err("unable to map SHIM");
		return -EIO;
	}

	/* Get mailbox addr from platform resource table */
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!rsrc) {
		pr_err("Invalid Mailbox base from IFWI");
		return -EIO;
	}
	ctx->mailbox_add = rsrc->start;
	pr_debug("Mailbox base: %#x", ctx->mailbox_add);
	ctx->mailbox = devm_ioremap_nocache(ctx->dev, ctx->mailbox_add,
					    resource_size(rsrc));
	if (!ctx->mailbox) {
		pr_err("unable to map mailbox");
		return -EIO;
	}

	/* Get iram/iccm addr from platform resource table */
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!rsrc) {
		pr_err("Invalid IRAM base from IFWI");
		return -EIO;
	}
	ctx->iram_base = rsrc->start;
	ctx->iram_end =  rsrc->end;
	pr_debug("IRAM base: %#x", ctx->iram_base);
	ctx->iram = devm_ioremap_nocache(ctx->dev, ctx->iram_base,
					 resource_size(rsrc));
	if (!ctx->iram) {
		pr_err("unable to map IRAM");
		return -EIO;
	}

	/* Get dram/dccm addr from platform resource table */
	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	if (!rsrc) {
		pr_err("Invalid DRAM base from IFWI");
		return -EIO;
	}
	ctx->dram_base = rsrc->start;
	ctx->dram_end = rsrc->end;
	pr_debug("DRAM base: %#x", ctx->dram_base);
	ctx->dram = devm_ioremap_nocache(ctx->dev, ctx->dram_base,
					 resource_size(rsrc));
	if (!ctx->dram) {
		pr_err("unable to map DRAM");
		return -EIO;
	}

	/* Register the ISR */
	irq = platform_get_irq(pdev, 0);
	pr_debug("irq from pdev is:%d", irq);
	ret = devm_request_threaded_irq(ctx->dev, irq, ctx->ops->interrupt,
					ctx->ops->irq_thread, 0, SST_DRV_NAME,
					ctx);
	if (ret)
		return ret;
	pr_debug("Registered IRQ %#x\n", irq);

	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	if (!rsrc) {
		pr_err("Invalid PMCSR base from IFWI");
		return -EIO;
	}
	ctx->pmcsr_base = rsrc->start;
	pr_info("PMCSR base: %#x", ctx->pmcsr_base);
	ctx->pmcsr = devm_ioremap_nocache(ctx->dev, ctx->pmcsr_base,
					 resource_size(rsrc));
	if (!ctx->pmcsr) {
		pr_err("unable to map DRAM");
		return -EIO;
	}
	return 0;
}

int __devinit sst_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	const char *hid;
	int i, ret = 0;
	struct intel_sst_ops *ops;
	struct sst_probe_info *info;
	struct intel_sst_drv *ctx;

	ret = acpi_bus_get_device(handle, &device);
	if (ret) {
		pr_err("%s: could not get acpi device - %d\n", __func__, ret);
		return -ENODEV;
	}

	if (acpi_bus_get_status(device) || !device->status.present) {
		pr_err("%s: device has invalid status", __func__);
		return -ENODEV;
	}

	hid = acpi_device_hid(device);
	pr_debug("%s for %s", __func__, hid);
	ret = sst_alloc_drv_context(dev);
	if (ret)
		return ret;
	ctx = sst_drv_ctx;
	ctx->dev = dev;
	ctx->pci_id = SST_BYT_PCI_ID;

	/* need to save shim registers in BYT */
	ctx->shim_regs64 = devm_kzalloc(dev, sizeof(*ctx->shim_regs64),
					GFP_KERNEL);
	if (!ctx->shim_regs64)
		return -ENOMEM;


	ctx->use_32bit_ops = true;

	ret = sst_driver_ops(ctx);
	if (ret != 0)
		return -EINVAL;
	ops = ctx->ops;

	sst_init_locks(ctx);

	ctx->stream_cnt = 0;
	ctx->pb_streams = 0;
	ctx->cp_streams = 0;
	ctx->fw = NULL;
	ctx->fw_in_mem = NULL;
	ctx->use_dma = 0;
	ctx->use_lli = 0;

	if (sst_workqueue_init(ctx))
		goto do_free_wq;

	info = sst_get_acpi_driver_data(hid);
	memcpy(&ctx->info, info, sizeof(ctx->info));

	ctx->ipc_reg.ipcx = SST_PRH_IPCX;
	ctx->ipc_reg.ipcd = SST_PRH_IPCD;

	pr_debug("Got drv data max stream %d\n",
				ctx->info.max_streams);
	for (i = 1; i <= ctx->info.max_streams; i++) {
		struct stream_info *stream = &ctx->streams[i];
		mutex_init(&stream->lock);
	}

	ret = sst_platform_get_resources(ctx, pdev);
	if (ret)
		goto do_free_wq;

	/*Register LPE Control as misc driver*/
	ret = misc_register(&lpe_ctrl);
	if (ret) {
		pr_err("couldn't register control device\n");
		goto do_free_wq;
	}
	sst_shim_write64(ctx->shim, SST_IMRX, 0xFFFF0034);

	if (ctx->use_32bit_ops) {
		pr_debug("allocate mem for context save/restore\n ");
		/*allocate mem for fw context save during suspend*/
		ctx->fw_cntx = devm_kzalloc(ctx->dev, FW_CONTEXT_MEM, GFP_KERNEL);
		if (!ctx->fw_cntx) {
			ret = -ENOMEM;
			goto do_free_misc;
		}
		/*setting zero as that is valid mem to restore*/
		ctx->fw_cntx_size = 0;
	}

	platform_set_drvdata(pdev, ctx);
	register_sst(dev);
	sst_debugfs_init(ctx);
	sst_set_fw_state_locked(ctx, SST_UN_INIT);
	sst_save_shim64(ctx, ctx->shim, ctx->shim_regs64);
	pr_info("%s successfully done!\n", __func__);
	return ret;

do_free_misc:
	misc_deregister(&lpe_ctrl);
do_free_wq:
	sst_destroy_workqueue(ctx);

	sst_drv_ctx = NULL;
	platform_set_drvdata(pdev, NULL);
	pr_err("%s: failed with %d\n", __func__, ret);
	return ret;
}

/**
* intel_sst_remove - remove function
*
* @pdev:	platform device structure
*
* This function is called by OS when a device is unloaded
* This frees the interrupt etc
*/
int sst_acpi_remove(struct platform_device *pdev)
{
	struct intel_sst_drv *ctx;

	ctx = platform_get_drvdata(pdev);
	sst_debugfs_exit(ctx);
	pm_runtime_get_noresume(ctx->dev);
	pm_runtime_disable(ctx->dev);
	unregister_sst(ctx->dev);
	sst_set_fw_state_locked(ctx, SST_UN_INIT);
	misc_deregister(&lpe_ctrl);
	kfree(ctx->runtime_param.param.addr);
	flush_scheduled_work();
	sst_destroy_workqueue(ctx);
	release_firmware(ctx->fw);
	ctx->fw = NULL;
	kfree(ctx->fw_sg_list.src);
	kfree(ctx->fw_sg_list.dst);
	ctx->fw_sg_list.list_len = 0;
	kfree(ctx->fw_in_mem);
	ctx->fw_in_mem = NULL;
	sst_memcpy_free_resources();
	sst_drv_ctx = NULL;
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#else
int __devinit sst_acpi_probe(struct platform_device *pdev)
{
	return -EINVAL;
}

int sst_acpi_remove(struct platform_device *pdev)
{
	return -EINVAL;
}
#endif

MODULE_DESCRIPTION("Intel (R) SST(R) Audio Engine ACPI Driver");
MODULE_AUTHOR("Ramesh Babu K V");
MODULE_AUTHOR("Omair Mohammed Abdullah");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("sst");
