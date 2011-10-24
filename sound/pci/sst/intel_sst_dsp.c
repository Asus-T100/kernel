/*
 *  intel_sst_dsp.c - Intel SST Driver for audio engine
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
 *	and middleware.
 *
 *  This file contains all dsp controlling functions like firmware download,
 * setting/resetting dsp cores, etc
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/dmaengine.h>
#include <linux/intel_mid_dma.h>
#include <sound/intel_sst.h>
#include <sound/intel_sst_ioctl.h>
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"


/**
 * intel_sst_reset_dsp_mrst - Resetting SST DSP
 *
 * This resets DSP in case of MRST platfroms
 */
static int intel_sst_reset_dsp_mrst(void)
{
	union config_status_reg csr;

	pr_debug("Resetting the DSP in mrst\n");
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.full |= 0x382;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.strb_cntr_rst = 0;
	csr.part.run_stall = 0x1;
	csr.part.bypass = 0x7;
	csr.part.sst_reset = 0x1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	return 0;
}

/**
 * intel_sst_reset_dsp_medfield - Resetting SST DSP
 *
 * This resets DSP in case of Medfield platfroms
 */
static int intel_sst_reset_dsp_medfield(void)
{
	union config_status_reg csr;

	pr_debug("Resetting the DSP in medfield\n");
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.full |= 0x382;
	csr.part.run_stall = 0x1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	return 0;
}

/**
 * sst_start_mrst - Start the SST DSP processor
 *
 * This starts the DSP in MRST platfroms
 */
static int sst_start_mrst(void)
{
	union config_status_reg csr;

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.part.run_stall = 0;
	csr.part.sst_reset = 0;
	csr.part.strb_cntr_rst = 1;
	pr_debug("Setting SST to execute_mrst 0x%x\n", csr.full);
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	return 0;
}

/**
 * sst_start_medfield - Start the SST DSP processor
 *
 * This starts the DSP in MRST platfroms
 */
static int sst_start_medfield(void)
{
	union config_status_reg csr;

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.mfld_strb = 1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.run_stall = 0;
	csr.part.sst_reset = 0;
	pr_debug("Starting the DSP_medfld %x\n", csr.full);
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	pr_debug("Starting the DSP_medfld\n");

	return 0;
}

/**
 * sst_parse_module - Parse audio FW modules
 *
 * @module: FW module header
 *
 * Parses modules that need to be placed in SST IRAM and DRAM
 * and stores them in a list for transfer
 * returns error or 0 if module sizes are proper
 */
static int sst_parse_module(struct fw_module_header *module)
{
	struct dma_block_info *block;
	struct sst_firmware_list *node;
	u32 count, offset;
	unsigned long ram;

	pr_debug("module sign %s size %x blocks %x type %x\n",
			module->signature, module->mod_size,
			module->blocks, module->type);
	pr_debug("module entrypoint 0x%x\n", module->entry_point);

	block = (void *)module + sizeof(*module);

	for (count = 0; count < module->blocks; count++) {
		if (block->size <= 0) {
			pr_err("block size invalid\n");
			return -EINVAL;
		}
		switch (block->type) {
		case SST_IRAM:
			ram = sst_drv_ctx->iram_base;
			break;
		case SST_DRAM:
			ram = sst_drv_ctx->dram_base;
			break;
		default:
			pr_err("wrong ram type0x%x in block0x%x\n",
					block->type, count);
			return -EINVAL;
		}
		offset = 0;
		do {
			node = kmalloc(sizeof(*node), GFP_KERNEL);
			if (!node) {
				pr_err("mem alloc failed\n");
				return -ENOMEM;
			}
			node->dstn = ram + block->ram_offset + offset;
			node->src = virt_to_phys((void *)block + sizeof(*block) + offset);
			node->len = block->size - offset;
			node->is_last = false;
			node->trf_status = SST_DESC_NULL;
			pr_debug("DMA block src %lx, dstn %lx, size %d, offset %d\n",
				node->src, node->dstn, node->len, offset);
			if (node->len > SST_MAX_DMA_LEN) {
				pr_debug("block size exceeds %d\n", SST_MAX_DMA_LEN);
				node->len = SST_MAX_DMA_LEN;
				offset += node->len;
			} else {
				offset = 0;
				pr_debug("Node length less that %d\n", SST_MAX_DMA_LEN);
			}
			list_add_tail(&node->node, &sst_drv_ctx->fw_list);
		} while (offset > 0);
		block = (void *)block + sizeof(*block) + block->size;
	}
	if (node)
		node->is_last = true;
	return 0;
}

static bool chan_filter(struct dma_chan *chan, void *param)
{
	struct sst_dma *dma = (struct sst_dma *)param;
	bool ret = false;

	/* we only need MID_DMAC1 as that can access DSP RAMs*/
	if (chan->device->dev == &dma->dmac->dev)
		ret = true;

	return ret;
}

static int sst_alloc_dma_chan(struct sst_dma *dma)
{
	dma_cap_mask_t mask;
	struct intel_mid_dma_slave *slave = &dma->slave;
	int retval;

	pr_debug("%s\n", __func__);
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma->dmac = pci_get_device(PCI_VENDOR_ID_INTEL, PCI_DMAC_ID, NULL);
	if (!dma->dmac) {
		pr_err("Can't find DMAC %x\n", PCI_DMAC_ID);
		return -ENODEV;
	}
	dma->ch = dma_request_channel(mask, chan_filter, dma);

	slave->dma_slave.direction = DMA_FROM_DEVICE;
	slave->hs_mode = 0;
	slave->cfg_mode = LNW_DMA_MEM_TO_MEM;
	slave->dma_slave.src_addr_width = slave->dma_slave.dst_addr_width =
						DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave->dma_slave.src_maxburst = slave->dma_slave.dst_maxburst =
							LNW_DMA_MSIZE_16;

	retval = dmaengine_slave_config(dma->ch, &slave->dma_slave);
	if (retval) {
		pr_err("unable to set slave config, err %d\n", retval);
		return -EIO;
	}
	return retval;
}

static void sst_dma_transfer_complete(void *arg)
{
	struct sst_firmware_list *block = arg;
	static unsigned int count;

	pr_debug("%s, dma callback for %d\n", __func__, count);
	count++;

	pr_debug("current desc status %d\n", block->trf_status);
	if (block->trf_status != SST_DESC_SUBMITTED)
		pr_warn("desc status is showing wrongly\n");
	block->trf_status = SST_DESC_DONE;
	pr_debug("new desc status %d\n", block->trf_status);
	if (block->is_last == true) {
		count = 0;
		pr_debug("end of list callback, waking up...\n");
	}
}

static int
sst_get_dma_desc(struct list_head *head, struct sst_dma *dma)
{
	struct sst_firmware_list *block;
	enum dma_ctrl_flags flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	int i = 0;

	pr_debug("%s\n", __func__);
	list_for_each_entry(block, head, node) {
		pr_debug("prep for block %d\n", i); i++;
		block->desc = dma->ch->device->device_prep_dma_memcpy(
			dma->ch, block->dstn, block->src, block->len, flag);
		if (block->desc) {
			block->trf_status = SST_DESC_PREPARED;
			block->desc->callback = sst_dma_transfer_complete;
			block->desc->callback_param = block;
			pr_debug("set block ptrs\n");
		}
	}
	return 0;
}
/**
 * sst_parse_fw_image - parse and load FW
 *
 * @sst_fw: pointer to audio fw
 *
 * This function is called to verify and parse the FW image and save the parsed
 * image in a list for DMA
 */
static int sst_parse_fw_image(const void *sst_fw_in_mem, unsigned long size)
{
	struct fw_header *header;
	u32 count;
	int ret_val;
	struct fw_module_header *module;

	pr_debug("%s\n", __func__);
	/* Read the header information from the data pointer */
	header = (struct fw_header *)sst_fw_in_mem;
	pr_debug("header sign=%s size=%x modules=%x fmt=%x size=%x\n",
			header->signature, header->file_size, header->modules,
			header->file_format, sizeof(*header));
	/* verify FW */
	if ((strncmp(header->signature, SST_FW_SIGN, 4) != 0) ||
		(size != header->file_size + sizeof(*header))) {
		/* Invalid FW signature */
		pr_err("InvalidFW sign/filesize mismatch\n");
		return -EINVAL;
	}

	module = (void *)sst_fw_in_mem + sizeof(*header);
	for (count = 0; count < header->modules; count++) {
		/* module */
		ret_val = sst_parse_module(module);
		if (ret_val)
			return ret_val;
		module = (void *)module + sizeof(*module) + module->mod_size ;
	}

	return 0;
}

/*
 * sst_request_fw - requests audio fw from kernel and saves a copy
 *
 * This function requests the SST FW from the kernel, parses it and
 * saves a copy in the driver context
 */
int sst_request_fw(void)
{
	int retval = 0;
	char name[20];

	snprintf(name, sizeof(name), "%s%04x%s", "fw_sst_",
					sst_drv_ctx->pci_id, ".bin");

	pr_debug("Requesting %s FW now...\n", name);
	retval = request_firmware(&sst_drv_ctx->fw, name,
				 &sst_drv_ctx->pci->dev);
	if (retval) {
		pr_err("request fw failed %d\n", retval);
		return retval;
	}

	if (!sst_drv_ctx->fw_in_mem) {
		pr_debug("firmware not in memory\n");
		sst_drv_ctx->fw_in_mem = kzalloc(sst_drv_ctx->fw->size, GFP_KERNEL);
		if (!sst_drv_ctx->fw_in_mem) {
			pr_err("%s unable to allocate memory\n", __func__);
			retval = -ENOMEM;
			goto end_release;
		}
		memcpy(sst_drv_ctx->fw_in_mem, sst_drv_ctx->fw->data,
				sst_drv_ctx->fw->size);
	}

	retval = sst_parse_fw_image(sst_drv_ctx->fw_in_mem,
					sst_drv_ctx->fw->size);
	if (retval) {
		kfree(sst_drv_ctx->fw_in_mem);
		goto end_release;
	}

end_release:
	release_firmware(sst_drv_ctx->fw);
	sst_drv_ctx->fw = NULL;
	return retval;
}

int sst_dma_firmware(struct list_head *head)
{
	/* submit the descriptors */
	struct sst_firmware_list *block;
	int i = 0;
	int cnt = 0;
	pr_debug("%s\n", __func__);

	list_for_each_entry(block, head, node) {
		pr_debug("submitting desc %d\n", i); i++;
		pr_debug("before submit desc status %d\n", block->trf_status);
		block->trf_status = SST_DESC_SUBMITTED;
		block->desc->tx_submit(block->desc);
		/* wait for dma completion
		 * since this is memcpy, max block length should get completed
		 * within 2ms, since we are waiting for dma compleetion here and
		 * dont want to yeild, just poll
		 * this is required till multiblock memcpy support API is added
		 * in the dma driver
		 */
		udelay(SST_DMA_DELAY);
		pr_debug("after submit desc status %d\n", block->trf_status);
		cnt = 0;
		/* wait in loop with counter */
		while (1) {
			pr_debug("while desc status %d\n", block->trf_status);
			if (block->trf_status == SST_DESC_DONE) {
				pr_debug("desc done %d\n", (i-1));
				break;
			}
			udelay(SST_DMA_DELAY);
			cnt++;
			if (cnt >= 10) {
				pr_err("dma for block %d failed\n", (i-1));
				break;
			}
		}
		pr_debug("wait done\n");
	}
	pr_debug("submitted all desc, waiting now...\n");
	return 0;
}

void sst_dma_free_resources(struct list_head *head, struct sst_dma *dma)
{
	struct sst_firmware_list *block, *__block;

	pr_debug("entry:%s\n", __func__);

	list_for_each_entry_safe(block, __block, head, node) {
		list_del(&block->node);
		kfree(block);
	}
	dma_release_channel(dma->ch);
}

/**
 * sst_load_fw - function to load FW into DSP
 *
 * @fw: Pointer to driver loaded FW
 * @context: driver context
 *
 * Transfers the FW to DSP using DMA
 */
int sst_load_fw(const void *fw_in_mem, void *context)
{
	int ret_val;

	pr_debug("load_fw called\n");
	BUG_ON(!fw_in_mem);

	if (sst_drv_ctx->pci_id == SST_MRST_PCI_ID)
		ret_val = intel_sst_reset_dsp_mrst();
	else if (sst_drv_ctx->pci_id == SST_MFLD_PCI_ID)
		ret_val = intel_sst_reset_dsp_medfield();
	if (ret_val)
		return ret_val;

	/* get a dmac channel */
	sst_alloc_dma_chan(&sst_drv_ctx->dma);
	/* allocate desc for transfer */
	sst_get_dma_desc(&sst_drv_ctx->fw_list, &sst_drv_ctx->dma);
	ret_val = sst_dma_firmware(&sst_drv_ctx->fw_list);
	if (ret_val)
		goto free_dma;
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_LOADED);
	/* bring sst out of reset */
	if (sst_drv_ctx->pci_id == SST_MRST_PCI_ID)
		ret_val = sst_start_mrst();
	else if (sst_drv_ctx->pci_id == SST_MFLD_PCI_ID)
		ret_val = sst_start_medfield();
	if (ret_val)
		goto free_dma;

	pr_debug("fw loaded successful!!!\n");
free_dma:
	sst_dma_free_resources(&sst_drv_ctx->fw_list, &sst_drv_ctx->dma);
	return ret_val;
}

/*This function is called when any codec/post processing library
 needs to be downloaded*/
static int sst_download_library(const struct firmware *fw_lib,
				struct snd_sst_lib_download_info *lib)
{
	/* send IPC message and wait */
	int i;
	u8 pvt_id;
	struct ipc_post *msg = NULL;
	union config_status_reg csr;
	struct snd_sst_str_type str_type = {0};
	int retval = 0;
	void *codec_fw;

	if (sst_create_large_msg(&msg))
		return -ENOMEM;

	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	i = sst_get_block_stream(sst_drv_ctx);
	pr_debug("alloc block allocated = %d, pvt_id %d\n", i, pvt_id);
	if (i < 0) {
		kfree(msg);
		return -ENOMEM;
	}
	sst_drv_ctx->alloc_block[i].sst_id = pvt_id;
	sst_fill_header(&msg->header, IPC_IA_PREP_LIB_DNLD, 1, pvt_id);
	msg->header.part.data = sizeof(u32) + sizeof(str_type);
	str_type.codec_type = lib->dload_lib.lib_info.lib_type;
	/*str_type.pvt_id = pvt_id;*/
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), &str_type, sizeof(str_type));
	spin_lock(&sst_drv_ctx->list_spin_lock);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock(&sst_drv_ctx->list_spin_lock);
	sst_post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx, &sst_drv_ctx->alloc_block[i]);
	if (retval) {
		/* error */
		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		pr_err("Prep codec downloaded failed %d\n",
				retval);
		return -EIO;
	}
	pr_debug("FW responded, ready for download now...\n");
	/* downloading on success */
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_LOADED);
	csr.full = readl(sst_drv_ctx->shim + SST_CSR);
	csr.part.run_stall = 1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0x7;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	codec_fw = kzalloc(fw_lib->size, GFP_KERNEL);
	if (!codec_fw)
		return -ENOMEM;
	memcpy(codec_fw, fw_lib->data, fw_lib->size);
	retval = sst_parse_fw_image(codec_fw, fw_lib->size);
	sst_alloc_dma_chan(&sst_drv_ctx->dma);
	sst_get_dma_desc(&sst_drv_ctx->fw_list, &sst_drv_ctx->dma);
	retval = sst_dma_firmware(&sst_drv_ctx->fw_list);

	sst_dma_free_resources(&sst_drv_ctx->fw_list, &sst_drv_ctx->dma);
	kfree(codec_fw);
	if (retval)
		return retval;

	/* set the FW to running again */
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0x0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.run_stall = 0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	/* send download complete and wait */
	if (sst_create_large_msg(&msg)) {
		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		return -ENOMEM;
	}

	sst_fill_header(&msg->header, IPC_IA_LIB_DNLD_CMPLT, 1, pvt_id);
	sst_drv_ctx->alloc_block[i].sst_id = pvt_id;
	msg->header.part.data = sizeof(u32) + sizeof(*lib);
	lib->pvt_id = pvt_id;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), lib, sizeof(*lib));
	spin_lock(&sst_drv_ctx->list_spin_lock);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock(&sst_drv_ctx->list_spin_lock);
	sst_post_message(&sst_drv_ctx->ipc_post_msg_wq);
	pr_debug("Waiting for FW response Download complete\n");
	sst_drv_ctx->alloc_block[i].ops_block.condition = false;
	retval = sst_wait_timeout(sst_drv_ctx, &sst_drv_ctx->alloc_block[i]);
	if (retval) {
		/* error */
		sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		return -EIO;
	}

	pr_debug("FW success on Download complete\n");
	sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_RUNNING);
	return 0;
}

/* This function is called before downloading the codec/postprocessing
library is set for download to SST DSP*/
static int sst_validate_library(const struct firmware *fw_lib,
		struct lib_slot_info *slot,
		u32 *entry_point)
{
	struct fw_header *header;
	struct fw_module_header *module;
	struct dma_block_info *block;
	unsigned int n_blk, isize = 0, dsize = 0;
	int err = 0;

	header = (struct fw_header *)fw_lib->data;
	if (header->modules != 1) {
		pr_err("Module no mismatch found\n");
		err = -EINVAL;
		goto exit;
	}
	module = (void *)fw_lib->data + sizeof(*header);
	*entry_point = module->entry_point;
	pr_debug("Module entry point 0x%x\n", *entry_point);
	pr_debug("Module Sign %s, Size 0x%x, Blocks 0x%x Type 0x%x\n",
			module->signature, module->mod_size,
			module->blocks, module->type);

	block = (void *)module + sizeof(*module);
	for (n_blk = 0; n_blk < module->blocks; n_blk++) {
		switch (block->type) {
		case SST_IRAM:
			isize += block->size;
			break;
		case SST_DRAM:
			dsize += block->size;
			break;
		default:
			pr_err("Invalid block type for 0x%x\n", n_blk);
			err = -EINVAL;
			goto exit;
		}
		block = (void *)block + sizeof(*block) + block->size;
	}
	if (isize > slot->iram_size || dsize > slot->dram_size) {
		pr_err("library exceeds size allocated\n");
		err = -EINVAL;
		goto exit;
	} else
		pr_debug("Library is safe for download...\n");

	pr_debug("iram 0x%x, dram 0x%x, iram 0x%x, dram 0x%x\n",
			isize, dsize, slot->iram_size, slot->dram_size);
exit:
	return err;

}

/* This function is called when FW requests for a particular library download
This function prepares the library to download*/
int sst_load_library(struct snd_sst_lib_download *lib, u8 ops)
{
	char buf[20];
	const char *type, *dir;
	int len = 0, error = 0;
	u32 entry_point;
	const struct firmware *fw_lib;
	struct snd_sst_lib_download_info dload_info = {{{0},},};

	memset(buf, 0, sizeof(buf));

	pr_debug("Lib Type 0x%x, Slot 0x%x, ops 0x%x\n",
			lib->lib_info.lib_type, lib->slot_info.slot_num, ops);
	pr_debug("Version 0x%x, name %s, caps 0x%x media type 0x%x\n",
		lib->lib_info.lib_version, lib->lib_info.lib_name,
		lib->lib_info.lib_caps, lib->lib_info.media_type);

	pr_debug("IRAM Size 0x%x, offset 0x%x\n",
		lib->slot_info.iram_size, lib->slot_info.iram_offset);
	pr_debug("DRAM Size 0x%x, offset 0x%x\n",
		lib->slot_info.dram_size, lib->slot_info.dram_offset);

	switch (lib->lib_info.lib_type) {
	case SST_CODEC_TYPE_MP3:
		type = "mp3_";
		break;
	case SST_CODEC_TYPE_AAC:
		type = "aac_";
		break;
	case SST_CODEC_TYPE_AACP:
		type = "aac_v1_";
		break;
	case SST_CODEC_TYPE_eAACP:
		type = "aac_v2_";
		break;
	case SST_CODEC_TYPE_WMA9:
		type = "wma9_";
		break;
	default:
		pr_err("Invalid codec type\n");
		error = -EINVAL;
		goto wake;
	}

	if (ops == STREAM_OPS_CAPTURE)
		dir = "enc_";
	else
		dir = "dec_";
	len = strlen(type) + strlen(dir);
	strncpy(buf, type, sizeof(buf)-1);
	strncpy(buf + strlen(type), dir, sizeof(buf)-strlen(type)-1);
	len += snprintf(buf + len, sizeof(buf) - len, "%d",
			lib->slot_info.slot_num);
	len += snprintf(buf + len, sizeof(buf) - len, ".bin");

	pr_debug("Requesting %s\n", buf);

	error = request_firmware(&fw_lib, buf, &sst_drv_ctx->pci->dev);
	if (error) {
		pr_err("library load failed %d\n", error);
		goto wake;
	}
	error = sst_validate_library(fw_lib, &lib->slot_info, &entry_point);
	if (error)
		goto wake_free;

	lib->mod_entry_pt = entry_point;
	memcpy(&dload_info.dload_lib, lib, sizeof(*lib));
	error = sst_download_library(fw_lib, &dload_info);
	if (error)
		goto wake_free;

	/* lib is downloaded and init send alloc again */
	pr_debug("Library is downloaded now...\n");
wake_free:
	/* sst_wake_up_alloc_block(sst_drv_ctx, pvt_id, error, NULL); */
	release_firmware(fw_lib);
wake:
	return error;
}

