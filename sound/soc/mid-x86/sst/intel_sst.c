/*
 *  intel_sst.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10	Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com>
 *		KP Jeeja <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This driver exposes the audio engine functionalities to the ALSA
 *	 and middleware.
 *
 *  This file contains all init functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/async.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <asm/intel-mid.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_AUTHOR("Dharageswari R <dharageswari.r@intel.com>");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
MODULE_DESCRIPTION("Intel (R) SST(R) Audio Engine Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SST_DRIVER_VERSION);

/* GPIO pins used for SSP3 */
#define CLV_I2S_3_CLK_GPIO_PIN	12
#define CLV_I2S_3_FS_GPIO_PIN	13
#define CLV_I2S_3_TXD_GPIO_PIN	74
#define CLV_I2S_3_RXD_GPIO_PIN	75

/* FIXME: Remove the hardcoding after SSP changes */
#define SSP_BASE_CTP 0xFFA23000
#define SSP_SIZE_CTP 0x1000

#define DMA_BASE_CTP 0xFFAF8000
#define DMA_SIZE_CTP 0x1000

#define INFO(_iram_start, _iram_end, _iram_use,		\
		_dram_start, _dram_end, _dram_use,	\
		_imr_start, _imr_end, _imr_use,		\
		_use_elf, _max_streams) \
	((kernel_ulong_t)&(struct sst_probe_info) {\
		.iram_start = (_iram_start),		\
		.iram_end = (_iram_end),		\
		.iram_use = (_iram_use),		\
		.dram_start = (_dram_start),		\
		.dram_end = (_dram_end),		\
		.dram_use = (_dram_use),		\
		.imr_start = (_imr_start),		\
		.imr_end = (_imr_end),			\
		.imr_use = (_imr_use),			\
		.use_elf = (_use_elf),			\
		.max_streams = (_max_streams),		\
	 })

struct intel_sst_drv *sst_drv_ctx;
static struct mutex drv_ctx_lock;
struct class *sst_class;
static const struct file_operations intel_sst_fops_cntrl = {
	.owner = THIS_MODULE,
	.open = intel_sst_open_cntrl,
	.release = intel_sst_release_cntrl,
	.unlocked_ioctl = intel_sst_ioctl,
};

static struct miscdevice lpe_ctrl = {
	.minor = MISC_DYNAMIC_MINOR,/* dynamic allocation */
	.name = "intel_sst_ctrl",/* /dev/intel_sst_ctrl */
	.fops = &intel_sst_fops_cntrl
};

static inline int get_stream_id_mrfld(u32 pipe_id)
{
	int i;
	for (i = 1; i <= sst_drv_ctx->info.max_streams; i++)
		if (pipe_id == sst_drv_ctx->streams[i].pipe_id)
			return i;

	pr_err("%s: no such pipe_id(%u), FW what are you doing?",
			__func__, pipe_id);
	BUG();
	return -1;
}

static irqreturn_t intel_sst_irq_thread_mrfld(int irq, void *context)
{
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;
	union ipc_header_mrfld header;
	struct stream_info *stream;
	unsigned int size = 0, str_id;
	int msg_id;
	u32 pipe_id;

	header.full = sst_shim_read64(drv->shim, SST_IPCD);
	msg_id = header.p.header_low_payload & SST_UNSOLICITED_MSG_ID;
	pipe_id = header.p.header_low_payload >> 16;
	pr_debug("interrupt\n");
	if ((msg_id & IPC_SST_PERIOD_ELAPSED_MRFLD) &&
			(header.p.header_high.part.msg_id == IPC_CMD)) {
		sst_drv_ctx->ops->clear_interrupt();
		str_id = get_stream_id_mrfld(pipe_id);
		stream = &sst_drv_ctx->streams[str_id];
		pr_debug("Period elapsed rcvd!!!\n");
		if (stream->period_elapsed)
			stream->period_elapsed(stream->pcm_substream);
		return IRQ_HANDLED;
	}
	if (header.p.header_high.part.large)
		size = header.p.header_low_payload;
	sst_drv_ctx->ipc_process_reply.mrfld_header = header;
	memcpy_fromio(sst_drv_ctx->ipc_process_reply.mailbox,
		      drv->mailbox + SST_MAILBOX_RCV_MRFLD, size);
	queue_work(sst_drv_ctx->process_reply_wq,
			&sst_drv_ctx->ipc_process_reply.wq);
	return IRQ_HANDLED;
}

static irqreturn_t intel_sst_interrupt_mrfld(int irq, void *context)
{
	union interrupt_reg_mrfld isr;
	union ipc_header_mrfld header;
	union sst_imr_reg_mrfld imr;
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;
	irqreturn_t retval = IRQ_HANDLED;

	/* Interrupt arrived, check src */
	isr.full = sst_shim_read64(drv->shim, SST_ISRX);
	if (isr.part.done_interrupt) {
		/* Clear done bit */
		spin_lock(&sst_drv_ctx->ipc_spin_lock);
		header.full = sst_shim_read64(drv->shim, SST_IPCX);
		header.p.header_high.part.done = 0;
		sst_shim_write64(drv->shim, SST_IPCX, header.full);
		/* write 1 to clear status register */;
		isr.part.done_interrupt = 1;
		sst_shim_write64(drv->shim, SST_ISRX, isr.full);
		spin_unlock(&sst_drv_ctx->ipc_spin_lock);
		queue_work(sst_drv_ctx->post_msg_wq,
			&sst_drv_ctx->ipc_post_msg.wq);
		retval = IRQ_HANDLED;
	}
	if (isr.part.busy_interrupt) {
		spin_lock(&sst_drv_ctx->ipc_spin_lock);
		imr.full = sst_shim_read64(drv->shim, SST_IMRX);
		imr.part.busy_interrupt = 1;
		sst_shim_write64(drv->shim, SST_IMRX, imr.full);
		spin_unlock(&sst_drv_ctx->ipc_spin_lock);
		retval = IRQ_WAKE_THREAD;
	}
	return retval;
}

#ifdef MRFLD_TEST_ON_MFLD
static irqreturn_t intel_sst_irq_thread_mrfld32(int irq, void *context)
{
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;
	union ipc_header_high header;
	struct stream_info *stream;
	unsigned int str_id;
	u32 msg_id, size, pipe_id;
	int msg;

	header.full = sst_shim_read(drv->shim, SST_IPCD);
	memcpy_fromio(&msg_id,  drv->mailbox + SST_MAILBOX_RCV, sizeof(u32));
	msg = msg_id & SST_UNSOLICITED_MSG_ID;
	pipe_id = msg_id >> 16;
	pr_debug("%s: interrupt header %x Payload %x\n", __func__, header.full, msg_id);
	if ((msg_id & IPC_SST_PERIOD_ELAPSED_MRFLD) && (header.part.msg_id == IPC_CMD)) {
		pr_debug("period intr\n");
		sst_drv_ctx->ops->clear_interrupt();
		str_id = get_stream_id_mrfld(pipe_id);
		stream = &sst_drv_ctx->streams[str_id];
		if (stream->period_elapsed)
			stream->period_elapsed(stream->pcm_substream);
		return IRQ_HANDLED;
	}
	memcpy_fromio(&size, drv->mailbox + SST_MAILBOX_RCV, sizeof(u32));
	sst_drv_ctx->ipc_process_reply.mrfld_header.p.header_high = header;
	sst_drv_ctx->ipc_process_reply.mrfld_header.p.header_low_payload = size;
	if (size > SST_MAILBOX_SIZE) {
		pr_err("firmware is giving my exceptionally large sz msg %x\n", size);
		size = 0;
	} else {
		memcpy_fromio(sst_drv_ctx->ipc_process_reply.mailbox,
			drv->mailbox + SST_MAILBOX_RCV, size);
	}
	pr_debug("queue wq for intr now\n");
	queue_work(sst_drv_ctx->process_reply_wq,
			&sst_drv_ctx->ipc_process_reply.wq);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t intel_sst_irq_thread_mfld(int irq, void *context)
{
	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;
	union ipc_header header;
	struct stream_info *stream;
	unsigned int size = 0, str_id;

	header.full = sst_shim_read(drv->shim, SST_IPCD);
	if (header.part.msg_id == IPC_SST_PERIOD_ELAPSED) {
		sst_drv_ctx->ops->clear_interrupt();
		str_id = header.part.str_id;
		stream = &sst_drv_ctx->streams[str_id];
		if (stream->period_elapsed)
			stream->period_elapsed(stream->pcm_substream);
		return IRQ_HANDLED;
	}
	if (header.part.large)
		size = header.part.data;
	if (header.part.msg_id & REPLY_MSG) {
		sst_drv_ctx->ipc_process_msg.header = header;
		memcpy_fromio(sst_drv_ctx->ipc_process_msg.mailbox,
			drv->mailbox + SST_MAILBOX_RCV + 4, size);
		queue_work(sst_drv_ctx->process_msg_wq,
				&sst_drv_ctx->ipc_process_msg.wq);
	} else {
		sst_drv_ctx->ipc_process_reply.header = header;
		memcpy_fromio(sst_drv_ctx->ipc_process_reply.mailbox,
			drv->mailbox + SST_MAILBOX_RCV + 4, size);
		queue_work(sst_drv_ctx->process_reply_wq,
				&sst_drv_ctx->ipc_process_reply.wq);
	}
	return IRQ_HANDLED;
}
/**
* intel_sst_interrupt - Interrupt service routine for SST
*
* @irq:	irq number of interrupt
* @context: pointer to device structre
*
* This function is called by OS when SST device raises
* an interrupt. This will be result of write in IPC register
* Source can be busy or done interrupt
*/
static irqreturn_t intel_sst_intr_mfld(int irq, void *context)
{
	union interrupt_reg isr, imr;
	union ipc_header header;
	irqreturn_t retval = IRQ_HANDLED;

	struct intel_sst_drv *drv = (struct intel_sst_drv *) context;

	/* Interrupt arrived, check src */
	isr.full = sst_shim_read(drv->shim, SST_ISRX);
	if (isr.part.done_interrupt) {
		/* Clear done bit */
		spin_lock(&sst_drv_ctx->ipc_spin_lock);
		header.full = sst_shim_read(drv->shim, SST_IPCX);
		header.part.done = 0;
		sst_shim_write(sst_drv_ctx->shim, SST_IPCX, header.full);
		/* write 1 to clear status register */;
		isr.part.done_interrupt = 1;
		sst_shim_write(sst_drv_ctx->shim, SST_ISRX, isr.full);
		spin_unlock(&sst_drv_ctx->ipc_spin_lock);
		queue_work(sst_drv_ctx->post_msg_wq,
			&sst_drv_ctx->ipc_post_msg.wq);
		retval = IRQ_HANDLED;
	}
	if (isr.part.busy_interrupt) {
		/* mask busy interrupt */
		spin_lock(&sst_drv_ctx->ipc_spin_lock);
		imr.full = sst_shim_read(drv->shim, SST_IMRX);
		imr.part.busy_interrupt = 1;
		sst_shim_write(sst_drv_ctx->shim, SST_IMRX, imr.full);
		spin_unlock(&sst_drv_ctx->ipc_spin_lock);
		retval = IRQ_WAKE_THREAD;
	}
	return retval;
}

static struct intel_sst_ops mrfld_ops = {
	.interrupt = intel_sst_interrupt_mrfld,
	.irq_thread = intel_sst_irq_thread_mrfld,
	.clear_interrupt = intel_sst_clear_intr_mrfld,
	.start = sst_start_mrfld,
	.reset = intel_sst_reset_dsp_mrfld,
	.post_message = sst_post_message_mrfld,
	.sync_post_message = sst_sync_post_message_mrfld,
	.process_message = sst_process_message_mrfld,
	.process_reply = sst_process_reply_mrfld,
	.set_bypass = NULL,
};

#ifndef MRFLD_TEST_ON_MFLD
static struct intel_sst_ops mfld_ops = {
	.interrupt = intel_sst_intr_mfld,
	.irq_thread = intel_sst_irq_thread_mfld,
	.clear_interrupt = intel_sst_clear_intr_mfld,
	.start = sst_start_mfld,
	.reset = intel_sst_reset_dsp_mfld,
	.post_message = sst_post_message_mfld,
	.sync_post_message = sst_sync_post_message_mfld,
	.process_message = sst_process_message_mfld,
	.process_reply = sst_process_reply_mfld,
	.set_bypass = intel_sst_set_bypass_mfld,
};
#else
static struct intel_sst_ops mrfld32_ops = {
	.interrupt = intel_sst_intr_mfld,
	.irq_thread = intel_sst_irq_thread_mrfld32,
	.clear_interrupt = intel_sst_clear_intr_mfld,
	.start = sst_start_mfld,
	.reset = intel_sst_reset_dsp_mfld,
	.post_message = sst_post_message_mrfld32,
	.sync_post_message = sst_sync_post_message_mrfld32,
	.process_message = sst_process_message_mrfld,
	.process_reply = sst_process_reply_mrfld,
	.set_bypass = NULL,
};
#endif

static int sst_driver_ops(unsigned int pci_id)
{

	switch (pci_id) {
	case SST_MRFLD_PCI_ID:
		sst_drv_ctx->tstamp = SST_TIME_STAMP_MRFLD;
		sst_drv_ctx->ops = &mrfld_ops;
		return 0;
	case SST_CLV_PCI_ID:
	case SST_MFLD_PCI_ID:
		sst_drv_ctx->tstamp =  SST_TIME_STAMP;
#ifndef MRFLD_TEST_ON_MFLD
		sst_drv_ctx->ops = &mfld_ops;
#else
		sst_drv_ctx->ops = &mrfld32_ops;
#endif
		return 0;
	default:
		pr_err("SST Driver capablities missing for pci_id: %x", pci_id);
	return -EINVAL;
	};
}

/*
* intel_sst_probe - PCI probe function
*
* @pci:	PCI device structure
* @pci_id: PCI device ID structure
*
* This function is called by OS when a device is found
* This enables the device, interrupt etc
*/
static int __devinit intel_sst_probe(struct pci_dev *pci,
			const struct pci_device_id *pci_id)
{
	int i, ret = 0;
	struct intel_sst_ops *ops;
	struct sst_probe_info *info;
	int ddr_base;

	pr_debug("Probe for DID %x\n", pci->device);
	mutex_lock(&drv_ctx_lock);
	if (sst_drv_ctx) {
		pr_err("Only one sst handle is supported\n");
		mutex_unlock(&drv_ctx_lock);
		return -EBUSY;
	}

	sst_drv_ctx = kzalloc(sizeof(*sst_drv_ctx), GFP_KERNEL);
	if (!sst_drv_ctx) {
		pr_err("malloc fail\n");
		mutex_unlock(&drv_ctx_lock);
		return -ENOMEM;
	}
	mutex_unlock(&drv_ctx_lock);

	sst_drv_ctx->pci_id = pci->device;

	if (0 != sst_driver_ops(sst_drv_ctx->pci_id))
		return -EINVAL;
	mutex_init(&sst_drv_ctx->stream_lock);
	mutex_init(&sst_drv_ctx->sst_lock);
	mutex_init(&sst_drv_ctx->mixer_ctrl_lock);
	mutex_init(&sst_drv_ctx->sst_in_mem_lock);
	mutex_init(&sst_drv_ctx->csr_lock);

	sst_drv_ctx->stream_cnt = 0;
	sst_drv_ctx->pb_streams = 0;
	sst_drv_ctx->cp_streams = 0;
	sst_drv_ctx->fw = NULL;
	sst_drv_ctx->fw_in_mem = NULL;
	/* we use dma, so set to 1*/
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) {
		sst_drv_ctx->use_dma = 0;
		sst_drv_ctx->use_lli = 0;
	} else {
		sst_drv_ctx->use_dma = 1;
		sst_drv_ctx->use_lli = 1;
	}

	ops = sst_drv_ctx->ops;

	INIT_LIST_HEAD(&sst_drv_ctx->memcpy_list);
	INIT_LIST_HEAD(&sst_drv_ctx->libmemcpy_list);

	INIT_LIST_HEAD(&sst_drv_ctx->ipc_dispatch_list);
	INIT_LIST_HEAD(&sst_drv_ctx->block_list);
	INIT_WORK(&sst_drv_ctx->ipc_post_msg.wq, ops->post_message);
	INIT_WORK(&sst_drv_ctx->ipc_process_msg.wq, ops->process_message);
	INIT_WORK(&sst_drv_ctx->ipc_process_reply.wq, ops->process_reply);
	init_waitqueue_head(&sst_drv_ctx->wait_queue);

	sst_drv_ctx->mad_wq = create_singlethread_workqueue("sst_mad_wq");
	if (!sst_drv_ctx->mad_wq)
		goto do_free_drv_ctx;
	sst_drv_ctx->post_msg_wq =
		create_singlethread_workqueue("sst_post_msg_wq");
	if (!sst_drv_ctx->post_msg_wq)
		goto free_mad_wq;
	sst_drv_ctx->process_msg_wq =
		create_singlethread_workqueue("sst_process_msg_wqq");
	if (!sst_drv_ctx->process_msg_wq)
		goto free_post_msg_wq;
	sst_drv_ctx->process_reply_wq =
		create_singlethread_workqueue("sst_proces_reply_wq");
	if (!sst_drv_ctx->process_reply_wq)
		goto free_process_msg_wq;

	spin_lock_init(&sst_drv_ctx->ipc_spin_lock);
	spin_lock_init(&sst_drv_ctx->block_lock);
	spin_lock_init(&sst_drv_ctx->pvt_id_lock);

	info = (void *)pci_id->driver_data;
	memcpy(&sst_drv_ctx->info, info, sizeof(sst_drv_ctx->info));
	pr_info("Got drv data max stream %d\n",
				sst_drv_ctx->info.max_streams);
	for (i = 1; i <= sst_drv_ctx->info.max_streams; i++) {
		struct stream_info *stream = &sst_drv_ctx->streams[i];
		mutex_init(&stream->lock);
	}

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		sst_drv_ctx->device_input_mixer = SST_STREAM_DEVICE_IHF
							| SST_INPUT_STREAM_PCM;
	}

	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		pr_err("device can't be enabled\n");
		goto do_free_mem;
	}
	sst_drv_ctx->pci = pci_dev_get(pci);
	ret = pci_request_regions(pci, SST_DRV_NAME);
	if (ret)
		goto do_disable_device;
	/* map registers */
	/* SST Shim */

	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) {
		sst_drv_ctx->ddr_base = pci_resource_start(pci, 0);
		/*
		* check that the relocated IMR base matches with FW Binary
		* put temporary check till better soln is available for FW
		*/
		ddr_base = relocate_imr_addr_mrfld(sst_drv_ctx->ddr_base);
		if (ddr_base != MRFLD_FW_LSP_DDR_BASE) {
			pr_err("FW LSP DDR BASE does not match with IFWI\n");
			ret = -EINVAL;
			goto do_release_regions;
		}
		sst_drv_ctx->ddr_end = pci_resource_end(pci, 0);

		sst_drv_ctx->ddr = pci_ioremap_bar(pci, 0);
		if (!sst_drv_ctx->ddr)
			goto do_unmap_ddr;
		pr_debug("sst: DDR Ptr %p\n", sst_drv_ctx->ddr);
	} else {
		sst_drv_ctx->ddr = NULL;
	}

	/* SST Shim */
	sst_drv_ctx->shim_phy_add = pci_resource_start(pci, 1);
	sst_drv_ctx->shim = pci_ioremap_bar(pci, 1);
	if (!sst_drv_ctx->shim)
		goto do_release_regions;
	pr_debug("SST Shim Ptr %p\n", sst_drv_ctx->shim);

	/* Shared SRAM */
	sst_drv_ctx->mailbox_add = pci_resource_start(pci, 2);
	sst_drv_ctx->mailbox = pci_ioremap_bar(pci, 2);
	if (!sst_drv_ctx->mailbox)
		goto do_unmap_shim;
	pr_debug("SRAM Ptr %p\n", sst_drv_ctx->mailbox);

	/* IRAM */
	sst_drv_ctx->iram_end = pci_resource_end(pci, 3);
	sst_drv_ctx->iram_base = pci_resource_start(pci, 3);
	sst_drv_ctx->iram = pci_ioremap_bar(pci, 3);
	if (!sst_drv_ctx->iram)
		goto do_unmap_sram;
	pr_debug("IRAM Ptr %p\n", sst_drv_ctx->iram);

	/* DRAM */
	sst_drv_ctx->dram_end = pci_resource_end(pci, 4);
	sst_drv_ctx->dram_base = pci_resource_start(pci, 4);
	sst_drv_ctx->dram = pci_ioremap_bar(pci, 4);
	if (!sst_drv_ctx->dram)
		goto do_unmap_iram;
	pr_debug("DRAM Ptr %p\n", sst_drv_ctx->dram);

	/* FIXME: Support for other platforms after SSP Patch */
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {

		/* SSP Register */
		sst_drv_ctx->debugfs.ssp = ioremap(SSP_BASE_CTP, SSP_SIZE_CTP);
		if (!sst_drv_ctx->debugfs.ssp)
			goto do_unmap_dram;

		pr_debug("\n ssp io 0x%x ssp 0x%x size 0x%x",
			sst_drv_ctx->debugfs.ssp,
			SSP_BASE_CTP, SSP_SIZE_CTP);

		/* DMA Register */
		sst_drv_ctx->debugfs.dma_reg = ioremap(DMA_BASE_CTP, DMA_SIZE_CTP);
		if (!sst_drv_ctx->debugfs.dma_reg)
			goto do_unmap_ssp;

		pr_debug("\n dma io 0x%x ssp 0x%x size 0x%x",
			sst_drv_ctx->debugfs.dma_reg,
			DMA_BASE_CTP, DMA_SIZE_CTP);
	}
#ifdef CONFIG_DEBUG_FS
	sst_drv_ctx->dump_buf.iram_buf.size = pci_resource_len(pci, 3);
	sst_drv_ctx->dump_buf.iram_buf.buf = kzalloc(sst_drv_ctx->dump_buf.iram_buf.size,
						GFP_KERNEL);
	if (!sst_drv_ctx->dump_buf.iram_buf.buf) {
		pr_err("%s: no memory\n", __func__);
		ret = -ENOMEM;
		goto do_unmap;
	}

	sst_drv_ctx->dump_buf.dram_buf.size = pci_resource_len(pci, 4);
	sst_drv_ctx->dump_buf.dram_buf.buf = kzalloc(sst_drv_ctx->dump_buf.dram_buf.size,
						GFP_KERNEL);
	if (!sst_drv_ctx->dump_buf.dram_buf.buf) {
		pr_err("%s: no memory\n", __func__);
		ret = -ENOMEM;
		goto do_free_iram_buf;
	}

	pr_debug("\niram len 0x%x dram len 0x%x",
			sst_drv_ctx->dump_buf.iram_buf.size,
			sst_drv_ctx->dump_buf.dram_buf.size);
#endif
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		sst_drv_ctx->probe_bytes = kzalloc(SST_MAX_BIN_BYTES, GFP_KERNEL);
		if (!sst_drv_ctx->probe_bytes) {
			pr_err("%s: no memory\n", __func__);
			ret = -ENOMEM;
			goto do_free_dram_buf;
		}
	}

	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
	/* Register the ISR */
	ret = request_threaded_irq(pci->irq, sst_drv_ctx->ops->interrupt,
		sst_drv_ctx->ops->irq_thread, NULL, SST_DRV_NAME,
		sst_drv_ctx);
	if (ret)
		goto do_free_probe_bytes;
	pr_debug("Registered IRQ 0x%x\n", pci->irq);

	/*Register LPE Control as misc driver*/
	ret = misc_register(&lpe_ctrl);
	if (ret) {
		pr_err("couldn't register control device\n");
		goto do_free_irq;
	}
	/* default intr are unmasked so set this as masked */
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		sst_shim_write64(sst_drv_ctx->shim, SST_IMRX, 0xFFFF0034);

	if ((sst_drv_ctx->pci_id == SST_MFLD_PCI_ID) ||
			(sst_drv_ctx->pci_id == SST_CLV_PCI_ID)) {
		u32 csr;
		u32 csr2;
		u32 clkctl;

		/*allocate mem for fw context save during suspend*/
		sst_drv_ctx->fw_cntx = kzalloc(FW_CONTEXT_MEM, GFP_KERNEL);
		if (!sst_drv_ctx->fw_cntx) {
			ret = -ENOMEM;
			goto do_free_misc;
		}
		/*setting zero as that is valid mem to restore*/
		sst_drv_ctx->fw_cntx_size = 0;

		/*set lpe start clock and ram size*/
		csr = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
		csr |= 0x30000;
		/*make sure clksel set to OSC for SSP0,1 (default)*/
		csr &= 0xFFFFFFF3;
		sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr);

		/*set clock output enable for SSP0,1,3*/
		clkctl = sst_shim_read(sst_drv_ctx->shim, SST_CLKCTL);
		if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
			clkctl |= (0x7 << 16);
		else
			clkctl |= ((1<<16)|(1<<17));
		sst_shim_write(sst_drv_ctx->shim, SST_CLKCTL, clkctl);

		/* set SSP0 & SSP1 disable DMA Finish*/
		csr2 = sst_shim_read(sst_drv_ctx->shim, SST_CSR2);
		/*set SSP3 disable DMA finsh for SSSP3 */
		csr2 |= BIT(1)|BIT(2);
		sst_shim_write(sst_drv_ctx->shim, SST_CSR2, csr2);
	} else {
		/*allocate mem for fw context save during suspend*/
		sst_drv_ctx->context.iram =
			kzalloc(sst_drv_ctx->iram_end - sst_drv_ctx->iram_base, GFP_KERNEL);
		if (!sst_drv_ctx->context.iram) {
			ret = -ENOMEM;
			goto do_free_misc;
		}
		sst_drv_ctx->context.dram =
			kzalloc(sst_drv_ctx->dram_end - sst_drv_ctx->dram_base, GFP_KERNEL);
		if (!sst_drv_ctx->context.dram) {
			ret = -ENOMEM;
			kfree(sst_drv_ctx->context.iram);
			goto do_free_misc;
		}
	}

	/* GPIO_PIN 12,13,74,75 needs to be configured in
	 * ALT_FUNC_2 mode for SSP3 IOs
	 */
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		lnw_gpio_set_alt(CLV_I2S_3_CLK_GPIO_PIN, LNW_ALT_2);
		lnw_gpio_set_alt(CLV_I2S_3_FS_GPIO_PIN, LNW_ALT_2);
		lnw_gpio_set_alt(CLV_I2S_3_TXD_GPIO_PIN, LNW_ALT_2);
		lnw_gpio_set_alt(CLV_I2S_3_RXD_GPIO_PIN, LNW_ALT_2);

	}

	pci_set_drvdata(pci, sst_drv_ctx);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);
	register_sst(&pci->dev);
	sst_debugfs_init(sst_drv_ctx);
	sst_drv_ctx->qos = kzalloc(sizeof(struct pm_qos_request),
				GFP_KERNEL);
	if (!sst_drv_ctx->qos)
		goto do_free_misc;
	pm_qos_add_request(sst_drv_ctx->qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
	pr_info("%s successfully done!\n", __func__);
	return ret;

do_free_misc:
	misc_deregister(&lpe_ctrl);
do_free_irq:
	free_irq(pci->irq, sst_drv_ctx);
do_free_probe_bytes:
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		kfree(sst_drv_ctx->probe_bytes);
do_free_dram_buf:
#ifdef CONFIG_DEBUG_FS
	kfree(sst_drv_ctx->dump_buf.dram_buf.buf);
do_free_iram_buf:
	kfree(sst_drv_ctx->dump_buf.iram_buf.buf);
#endif
do_unmap:
	/* FIXME: Support for other platforms after SSP Patch */
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		iounmap(sst_drv_ctx->debugfs.dma_reg);
do_unmap_ssp:
	/* FIXME: Support for other platforms after SSP Patch */
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		iounmap(sst_drv_ctx->debugfs.ssp);
do_unmap_dram:
	iounmap(sst_drv_ctx->dram);
do_unmap_iram:
	iounmap(sst_drv_ctx->iram);
do_unmap_sram:
	iounmap(sst_drv_ctx->mailbox);
do_unmap_shim:
	iounmap(sst_drv_ctx->shim);

do_unmap_ddr:
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		iounmap(sst_drv_ctx->ddr);

do_release_regions:
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);
do_free_mem:
	destroy_workqueue(sst_drv_ctx->process_reply_wq);
free_process_msg_wq:
	destroy_workqueue(sst_drv_ctx->process_msg_wq);
free_post_msg_wq:
	destroy_workqueue(sst_drv_ctx->post_msg_wq);
free_mad_wq:
	destroy_workqueue(sst_drv_ctx->mad_wq);
do_free_drv_ctx:
	kfree(sst_drv_ctx);
	sst_drv_ctx = NULL;
	pr_err("Probe failed with %d\n", ret);
	return ret;
}

/**
* intel_sst_remove - PCI remove function
*
* @pci:	PCI device structure
*
* This function is called by OS when a device is unloaded
* This frees the interrupt etc
*/
static void __devexit intel_sst_remove(struct pci_dev *pci)
{
	sst_debugfs_exit(sst_drv_ctx);
	pm_runtime_get_noresume(&pci->dev);
	pm_runtime_forbid(&pci->dev);
	unregister_sst(&pci->dev);
	pci_dev_put(sst_drv_ctx->pci);
	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
	misc_deregister(&lpe_ctrl);
	free_irq(pci->irq, sst_drv_ctx);

	/* FIXME: Support for other platforms after SSP Patch */
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		iounmap(sst_drv_ctx->debugfs.dma_reg);
		iounmap(sst_drv_ctx->debugfs.ssp);
	}
	iounmap(sst_drv_ctx->dram);
	iounmap(sst_drv_ctx->iram);
	iounmap(sst_drv_ctx->mailbox);
	iounmap(sst_drv_ctx->shim);
#ifdef CONFIG_DEBUG_FS
	kfree(sst_drv_ctx->dump_buf.iram_buf.buf);
	kfree(sst_drv_ctx->dump_buf.dram_buf.buf);
#endif
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		kfree(sst_drv_ctx->probe_bytes);

	kfree(sst_drv_ctx->fw_cntx);
	kfree(sst_drv_ctx->runtime_param.param.addr);
	flush_scheduled_work();
	destroy_workqueue(sst_drv_ctx->process_reply_wq);
	destroy_workqueue(sst_drv_ctx->process_msg_wq);
	destroy_workqueue(sst_drv_ctx->post_msg_wq);
	destroy_workqueue(sst_drv_ctx->mad_wq);
	release_firmware(sst_drv_ctx->fw);
	pm_qos_remove_request(sst_drv_ctx->qos);
	kfree(sst_drv_ctx->qos);
	sst_drv_ctx->fw = NULL;
	kfree(sst_drv_ctx->fw_sg_list.src);
	kfree(sst_drv_ctx->fw_sg_list.dst);
	sst_drv_ctx->fw_sg_list.list_len = 0;
	kfree(sst_drv_ctx->fw_in_mem);
	sst_drv_ctx->fw_in_mem = NULL;
	sst_memcpy_free_resources();
	kfree(sst_drv_ctx);
	sst_drv_ctx = NULL;
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

static int sst_save_fw_rams(struct intel_sst_drv *sst)
{
	/* first reset, stall and bypass the core */
	sst->ops->reset();

	/* FIXME now we copy, should use DMA here but for now we cant
	 * so use mempcy instead
	 */
	memcpy_fromio(sst->context.iram, sst->iram,
			sst->iram_end - sst->iram_base);
	memcpy_fromio(sst->context.dram, sst->dram,
			sst->dram_end - sst->dram_base);
	return 0;
}

static int sst_load_fw_rams(struct intel_sst_drv *sst)
{
	struct sst_block *block;

	/* first reset, stall and bypass the core */
	sst->ops->reset();

	/* FIXME now we copy, should use DMA here but for now we cant
	 * so use mempcy instead
	 */
	block = sst_create_block(sst, 0, FW_DWNL_ID);
	if (block == NULL)
		return -ENOMEM;

	memcpy_toio(sst->iram, sst->context.iram,
			sst->iram_end - sst->iram_base);
	memcpy_toio(sst->dram, sst->context.dram,
			sst->dram_end - sst->dram_base);
	sst_set_fw_state_locked(sst, SST_FW_LOADED);

	sst->ops->start();
	if (sst_wait_timeout(sst, block)) {
		pr_err("fw download failed\n");
		/* assume FW d/l failed due to timeout*/
		sst_set_fw_state_locked(sst, SST_UN_INIT);
		sst_free_block(sst, block);
		return -EBUSY;
	}
	pr_debug("Fw loaded!");
	sst_free_block(sst, block);
	return 0;
}

static int sst_save_dsp_context2(struct intel_sst_drv *sst)
{
	unsigned int pvt_id;
	struct ipc_post *msg = NULL;
	unsigned long irq_flags;
	struct ipc_dsp_hdr dsp_hdr;
	struct sst_block *block;

	/*not supported for rest*/
	if (sst->sst_state != SST_FW_RUNNING) {
		pr_debug("fw not running no context save ...\n");
		return 0;
	}

	/*send msg to fw*/
	pvt_id = sst_assign_pvt_id(sst);
	if (sst_create_block_and_ipc_msg(&msg, true, sst, &block,
				IPC_CMD, pvt_id)) {
		pr_err("msg/block alloc failed. Not proceeding with context save\n");
		return;
	}

	sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
				IPC_QUE_ID_MED, 1, pvt_id);
	msg->mrfld_header.p.header_low_payload = sizeof(dsp_hdr);
	msg->mrfld_header.p.header_high.part.res_rqd = 1;
	sst_fill_header_dsp(&dsp_hdr, IPC_PREP_D3, PIPE_RSVD, pvt_id);
	memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);
	sst->ops->post_message(&sst->ipc_post_msg_wq);
	/*wait for reply*/
	if (sst_wait_timeout(sst, block)) {
		pr_err("sst: err fw context save timeout  ...\n");
		pr_err("not suspending FW!!!");
		sst_free_block(sst, block);
		return -EIO;
	}
	if (block->ret_code) {
		pr_err("fw responded w/ error %d", block->ret_code);
		sst_free_block(sst, block);
		return -EIO;
	}

	/* all good, so lets copy the fw */
	sst_save_fw_rams(sst);
	sst->context.saved = 0;
	pr_debug("fw context saved  ...\n");
	sst_free_block(sst, block);
	return 0;
}

static void sst_save_dsp_context(void)
{
	struct snd_sst_ctxt_params fw_context;
	unsigned int pvt_id;
	struct ipc_post *msg = NULL;
	unsigned long irq_flags;
	struct sst_block *block;
	pr_debug("%s: Enter\n", __func__);

	/*check cpu type*/
	if (sst_drv_ctx->pci_id == SST_MRST_PCI_ID)
		return;
	/*not supported for rest*/
	if (sst_drv_ctx->sst_state != SST_FW_RUNNING) {
		pr_debug("fw not running no context save ...\n");
		return;
	}

	/*send msg to fw*/
	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	if (sst_create_block_and_ipc_msg(&msg, true, sst_drv_ctx, &block,
				IPC_IA_GET_FW_CTXT, pvt_id)) {
		pr_err("msg/block alloc failed. Not proceeding with context save\n");
		return;
	}
	sst_fill_header(&msg->header, IPC_IA_GET_FW_CTXT, 1, pvt_id);
	msg->header.part.data = sizeof(fw_context) + sizeof(u32);
	fw_context.address = virt_to_phys((void *)sst_drv_ctx->fw_cntx);
	fw_context.size = FW_CONTEXT_MEM;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32),
				&fw_context, sizeof(fw_context));
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	/*wait for reply*/
	if (sst_wait_timeout(sst_drv_ctx, block))
		pr_err("sst: err fw context save timeout  ...\n");
	pr_debug("fw context saved  ...\n");
	if (block->ret_code)
		sst_drv_ctx->fw_cntx_size = 0;
	else
		sst_drv_ctx->fw_cntx_size = *sst_drv_ctx->fw_cntx;
	pr_debug("fw copied data %x\n", sst_drv_ctx->fw_cntx_size);
	sst_free_block(sst_drv_ctx, block);
	return;
}


/*
 * The runtime_suspend/resume is pretty much similar to the legacy
 * suspend/resume with the noted exception below: The PCI core takes care of
 * taking the system through D3hot and restoring it back to D0 and so there is
 * no need to duplicate that here.
 */
static int intel_sst_runtime_suspend(struct device *dev)
{
	union config_status_reg csr;

	pr_debug("runtime_suspend called\n");
	if (sst_drv_ctx->sst_state == SST_SUSPENDED) {
		pr_err("System already in Suspended state");
		return 0;
	}
	/*save fw context*/

#ifndef MRFLD_TEST_ON_MFLD
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID) {
		if (sst_save_dsp_context2(sst_drv_ctx))
			return -EBUSY;
	} else {
		sst_save_dsp_context();
	}
#endif
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) {
		/*Assert RESET on LPE Processor*/
		csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
		sst_drv_ctx->csr_value = csr.full;
		csr.full = csr.full | 0x2;
		sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	}

	/* Move the SST state to Suspended */
	sst_set_fw_state_locked(sst_drv_ctx, SST_SUSPENDED);

	flush_workqueue(sst_drv_ctx->post_msg_wq);
	flush_workqueue(sst_drv_ctx->process_msg_wq);
	flush_workqueue(sst_drv_ctx->process_reply_wq);

	return 0;
}

static int intel_sst_runtime_resume(struct device *dev)
{
	u32 csr;

	pr_debug("runtime_resume called\n");
	if (sst_drv_ctx->sst_state != SST_SUSPENDED) {
		pr_err("SST is not in suspended state\n");
		return 0;
	}

	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) {
		csr = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
		/*
		 * To restore the csr_value after S0ix and S3 states.
		 * The value 0x30000 is to enable LPE dram high and low addresses.
		 * Reference:
		 * Penwell Audio Voice Module HAS 1.61 Section - 13.12.1 -
		 * CSR - Configuration and Status Register.
		 */
		csr |= (sst_drv_ctx->csr_value | 0x30000);
		sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr);

		/* GPIO_PIN 12,13,74,75 needs to be configured in
		 * ALT_FUNC_2 mode for SSP3 IOs
		 */
		if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
			lnw_gpio_set_alt(CLV_I2S_3_CLK_GPIO_PIN, LNW_ALT_2);
			lnw_gpio_set_alt(CLV_I2S_3_FS_GPIO_PIN, LNW_ALT_2);
			lnw_gpio_set_alt(CLV_I2S_3_TXD_GPIO_PIN, LNW_ALT_2);
			lnw_gpio_set_alt(CLV_I2S_3_RXD_GPIO_PIN, LNW_ALT_2);

		}
	}

	/* When fw_clear_cache is set, clear the cached firmware copy */
	/* fw_clear_cache is set through debugfs support */
	if (atomic_read(&sst_drv_ctx->fw_clear_cache)) {
		mutex_lock(&sst_drv_ctx->sst_in_mem_lock);

		if (sst_drv_ctx->fw_in_mem) {
			pr_debug("Clearing the cached firmware\n");
			kfree(sst_drv_ctx->fw_in_mem);
			sst_drv_ctx->fw_in_mem = NULL;
			atomic_set(&sst_drv_ctx->fw_clear_cache, 0);
		}
		mutex_unlock(&sst_drv_ctx->sst_in_mem_lock);
	}

	sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID && sst_drv_ctx->context.saved) {
		/* in mrfld we have saved ram snapshot
		 * so check if snapshot is present if so download that
		 */
		sst_load_fw_rams(sst_drv_ctx);
		sst_drv_ctx->context.saved = 0;
	}

	return 0;
}

static int intel_sst_runtime_idle(struct device *dev)
{
	pr_debug("runtime_idle called\n");
	if (sst_drv_ctx->sst_state != SST_UN_INIT) {
		pm_schedule_suspend(dev, SST_SUSPEND_DELAY);
		return -EBUSY;
	} else {
		return 0;
	}
	return -EBUSY;

}

static const struct dev_pm_ops intel_sst_pm = {
	.suspend = intel_sst_runtime_suspend,
	.resume = intel_sst_runtime_resume,
	.runtime_suspend = intel_sst_runtime_suspend,
	.runtime_resume = intel_sst_runtime_resume,
	.runtime_idle = intel_sst_runtime_idle,
};

#define SST_MFLD_IRAM_START	0
#define SST_MFLD_IRAM_END	0x80000
#define SST_MFLD_DRAM_START	0x400000
#define SST_MFLD_DRAM_END	0x480000

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_sst_ids) = {
	{ PCI_VDEVICE(INTEL, SST_MRST_PCI_ID),
		INFO(0, 0, false,
			0, 0, false,
			0, 0, false,
			false, 3)},
	{ PCI_VDEVICE(INTEL, SST_MFLD_PCI_ID),
		INFO(SST_MFLD_IRAM_START, SST_MFLD_IRAM_END, true,
			SST_MFLD_DRAM_START, SST_MFLD_DRAM_END, true,
			0, 0, false,
#ifdef MRFLD_TEST_ON_MFLD
			true, 24)},
#else
			false, 5)},
#endif
	{ PCI_VDEVICE(INTEL, SST_CLV_PCI_ID),
		INFO(0, 0, false,
			0, 0, false,
			0, 0, false,
			false, 5)},
	{ PCI_VDEVICE(INTEL, SST_MRFLD_PCI_ID),
		INFO(0, 0, false,
			0, 0, false,
			0, 0, false,
			true, 23)},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_sst_ids);

static struct pci_driver driver = {
	.name = SST_DRV_NAME,
	.id_table = intel_sst_ids,
	.probe = intel_sst_probe,
	.remove = __devexit_p(intel_sst_remove),
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_sst_pm,
	},
#endif
};

/**
* intel_sst_init - Module init function
*
* Registers with PCI
* Registers with /dev
* Init all data strutures
*/
static int __init intel_sst_init(void)
{
	/* Init all variables, data structure etc....*/
	int ret = 0;
	pr_info("INFO: ******** SST DRIVER loading.. Ver: %s\n",
				       SST_DRIVER_VERSION);

	mutex_init(&drv_ctx_lock);
	/* Register with PCI */
	ret = pci_register_driver(&driver);
	if (ret)
		pr_err("PCI register failed\n");
	return ret;
}

/**
* intel_sst_exit - Module exit function
*
* Unregisters with PCI
* Unregisters with /dev
* Frees all data strutures
*/
static void __exit intel_sst_exit(void)
{
	pci_unregister_driver(&driver);

	pr_debug("driver unloaded\n");
	sst_drv_ctx = NULL;
	return;
}

module_init_async(intel_sst_init);
module_exit(intel_sst_exit);
