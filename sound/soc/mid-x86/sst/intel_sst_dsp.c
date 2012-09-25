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
#include <linux/pm_qos.h>
#include <linux/intel_mid_pm.h>
#include <linux/elf.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"
#include <linux/sched.h>

/**
 * intel_sst_reset_dsp_medfield - Resetting SST DSP
 *
 * This resets DSP in case of Medfield platfroms
 */
int intel_sst_reset_dsp_mfld(void)
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
 * sst_start_medfield - Start the SST DSP processor
 *
 * This starts the DSP in MRST platfroms
 */
int sst_start_mfld(void)
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
 * intel_sst_reset_dsp_mrfld - Resetting SST DSP
 *
 * This resets DSP in case of MRFLD platfroms
 */
int intel_sst_reset_dsp_mrfld(void)
{
	union config_status_reg_mrfld csr;

	pr_debug("sst: Resetting the DSP in mrfld\n");

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);

	pr_debug("value:0x%llx\n", csr.full);

	csr.full |= 0x7;
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);

	pr_debug("value:0x%llx\n", csr.full);

	csr.full &= ~(0x1);
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
	pr_debug("value:0x%llx\n", csr.full);

	return 0;
}

/**
 * sst_start_merrifield - Start the SST DSP processor
 *
 * This starts the DSP in MERRIFIELD platfroms
 */
int sst_start_mrfld(void)
{
	union config_status_reg_mrfld csr;

	pr_debug("sst: Starting the DSP in mrfld LALALALA\n");

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("value:0x%llx\n", csr.full);

	csr.full |= 0x7;
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("value:0x%llx\n", csr.full);

	csr.full &= ~(0x5);
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("sst: Starting the DSP_merrifield:%llx\n", csr.full);

	return 0;
}

/*
 * sst_fill_sglist - Fill the sg list
 *
 * @ram: virtual address of IRAM/DRAM.
 * @block: Dma block infromation
 * @sg_src: source scatterlist pointer
 * @sg_dst: Destination scatterlist pointer
 *
 * Parses modules that need to be placed in SST IRAM and DRAM
 * and stores them in a sg list for transfer
 * returns error or 0 if list creation fails or pass.
  */

int sst_fill_sglist(unsigned long ram, struct dma_block_info *block,
		struct scatterlist **sg_src, struct scatterlist **sg_dst)
{
	u32 offset;
	unsigned long dstn, src;
	int len = 0;
	pr_debug("sst_fill_sglist\n");

	offset = 0;
	do {
		dstn = (unsigned long)(ram + block->ram_offset + offset);
		src = (unsigned long)((void *)block + sizeof(*block) + offset);
		len = block->size - offset;
		pr_debug("DMA blk src%lx,dstn %lx,size %d,offset %d\n",
				src, dstn, len, offset);
		if (len > SST_MAX_DMA_LEN) {
			pr_debug("block size exceeds %d\n", SST_MAX_DMA_LEN);
			len = SST_MAX_DMA_LEN;
			offset += len;
		} else {
			offset = 0;
			pr_debug("Node length less that %d\n",
							SST_MAX_DMA_LEN);
		}

		if (!sg_src || !sg_dst)
			return -ENOMEM;
		sg_set_buf(*sg_src, (void *)src, len);
		sg_set_buf(*sg_dst, (void *)dstn, len);
		*sg_src = sg_next(*sg_src);
		*sg_dst = sg_next(*sg_dst);
	} while (offset > 0);

	return 0;
}

static inline int sst_validate_fw_elf(const struct firmware *sst_fw)
{
	Elf32_Ehdr *elf;

	BUG_ON(!sst_fw);

	elf = (Elf32_Ehdr *)sst_fw->data;

	if ((elf->e_ident[0] != 0x7F) || (elf->e_ident[1] != 'E') ||
		(elf->e_ident[2] != 'L') || (elf->e_ident[3] != 'F')) {
		pr_debug("ELF Header Not found!%d\n", sst_fw->size);
		return -EINVAL;
	}
	pr_debug("Valid ELF Header...%d\n", sst_fw->size);
	return 0;
}

void sst_download_fw_mrfld(const void *fw_in_mem)
{
	int data_size = 0, i = 0;

	Elf32_Ehdr *elf;
	Elf32_Phdr *pr;

	BUG_ON(!fw_in_mem);

	elf = (Elf32_Ehdr *)fw_in_mem;
	pr = (Elf32_Phdr *) (fw_in_mem + elf->e_phoff);
	pr_debug("sst_download_fw_mrfld.....\n");
	while (i < elf->e_phnum) {
		if (pr[i].p_type == PT_LOAD) {
			if ((pr[i].p_paddr >= sst_drv_ctx->iram_base) &&
				(pr[i].p_paddr < sst_drv_ctx->iram_end)) {
				pr_debug("iccm copying\n");
				/*work around-since only 4 byte copying
				 *is only allowed for ICCM*/
				data_size = pr[i].p_filesz %
							SST_ICCM_BOUNDARY;
				if (data_size)
					pr[i].p_filesz +=
						SST_ICCM_BOUNDARY - data_size;
				memcpy_toio(sst_drv_ctx->iram +
				pr[i].p_paddr - sst_drv_ctx->iram_base,
					(void *)fw_in_mem + pr[i].p_offset,
					pr[i].p_filesz);

			} else if ((pr[i].p_paddr >= sst_drv_ctx->dram_base)
			&& (pr[i].p_paddr < sst_drv_ctx->dram_end)) {
				pr_debug("dccm copying\n");
				memcpy_toio(sst_drv_ctx->dram +
					pr[i].p_paddr -
					sst_drv_ctx->dram_base,
					(void *)fw_in_mem + pr[i].p_offset,
					pr[i].p_filesz);
			} else if ((pr[i].p_paddr >= sst_drv_ctx->ddr_base)
				&& (pr[i].p_paddr < sst_drv_ctx->ddr_end)) {
				pr_debug("ddr copying\n");
				memcpy_toio(sst_drv_ctx->ddr +
					pr[i].p_paddr -
					sst_drv_ctx->ddr_base,
					(void *)fw_in_mem + pr[i].p_offset,
					pr[i].p_filesz);
			}
		}
		i++;
	}
}
/**
 * sst_parse_module - Parse audio FW modules
 *
 * @module: FW module header
 *
 * Count the length for scattergather list
 * and create the scattergather list of same length
 * returns error or 0 if module sizes are proper
 */
static int sst_parse_module(struct fw_module_header *module,
				struct sst_sg_list *sg_list)
{
	struct dma_block_info *block;
	u32 count;
	unsigned long ram;
	int retval, sg_len = 0;
	struct scatterlist *sg_src, *sg_dst;

	pr_debug("module sign %s size %x blocks %x type %x\n",
			module->signature, module->mod_size,
			module->blocks, module->type);
	pr_debug("module entrypoint 0x%x\n", module->entry_point);

	block = (void *)module + sizeof(*module);

	for (count = 0; count < module->blocks; count++) {
		sg_len += (block->size) / SST_MAX_DMA_LEN;
		if ((block->size) % SST_MAX_DMA_LEN)
			sg_len = sg_len + 1;
		block = (void *)block + sizeof(*block) + block->size;
	}

	sg_src = kzalloc(sizeof(*sg_src)*(sg_len), GFP_KERNEL);
	if (NULL == sg_src)
		return -ENOMEM;
	sg_init_table(sg_src, sg_len);
	sg_dst = kzalloc(sizeof(*sg_dst)*(sg_len), GFP_KERNEL);
	if (NULL == sg_dst) {
		kfree(sg_src);
		return -ENOMEM;
	}
	sg_init_table(sg_dst, sg_len);

	sg_list->src = sg_src;
	sg_list->dst = sg_dst;
	sg_list->list_len = sg_len;

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
		/*converting from physical to virtual because
		scattergather list works on virtual pointers*/
		ram = (int) phys_to_virt(ram);
		retval = sst_fill_sglist(ram, block, &sg_src, &sg_dst);
		if (retval) {
			kfree(sg_src);
			kfree(sg_dst);
			sg_src = NULL;
			sg_dst = NULL;
			return retval;
		}
		block = (void *)block + sizeof(*block) + block->size;
	}
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

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		dma->dmac = pci_get_device(PCI_VENDOR_ID_INTEL,
						PCI_DMAC_CLV_ID, NULL);
	else
		dma->dmac = pci_get_device(PCI_VENDOR_ID_INTEL,
						PCI_DMAC_MFLD_ID, NULL);

	if (!dma->dmac) {
		pr_err("Can't find DMAC\n");
		return -ENODEV;
	}
	dma->ch = dma_request_channel(mask, chan_filter, dma);
	if (!dma->ch) {
		pr_err("unable to request dma channel\n");
		return -EIO;
	}

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
		dma_release_channel(dma->ch);
		return -EIO;
	}
	return retval;
}

static void sst_dma_transfer_complete(void *arg)
{
	sst_drv_ctx  = (struct intel_sst_drv *)arg;
	pr_debug(" sst_dma_transfer_complete\n");
	if (sst_drv_ctx->dma_info_blk.on == true) {
		sst_drv_ctx->dma_info_blk.on = false;
		sst_drv_ctx->dma_info_blk.condition = true;
		wake_up(&sst_drv_ctx->wait_queue);
	}
}

/**
 * sst_parse_fw_image - parse and load FW
 *
 * @sst_fw: pointer to audio fw
 *
 * This function is called to verify and parse the FW image and save the parsed
 * image in a list for DMA
 */
static int sst_parse_fw_image(const void *sst_fw_in_mem, unsigned long size,
				struct sst_sg_list *sg_list)
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
		ret_val = sst_parse_module(module, sg_list);
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

	if (!(sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID))
		snprintf(name, sizeof(name), "%s%04x%s", "fw_sst_",
				sst_drv_ctx->pci_id, ".bin");
	else
		snprintf(name, sizeof(name), "%s%04x", "fw_sst_",
				sst_drv_ctx->pci_id);

	pr_debug("Requesting FW %s now...\n", name);
	retval = request_firmware(&sst_drv_ctx->fw, name,
				 &sst_drv_ctx->pci->dev);
	if (retval) {
		pr_err("request fw failed %d\n", retval);
		return retval;
	}
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		retval = sst_validate_fw_elf(sst_drv_ctx->fw);
	if (retval != 0) {
		pr_err("FW image invalid...\n");
		goto end_release;
	}
	sst_drv_ctx->fw_in_mem = kzalloc(sst_drv_ctx->fw->size, GFP_KERNEL);
	if (!sst_drv_ctx->fw_in_mem) {
		pr_err("%s unable to allocate memory\n", __func__);
		retval = -ENOMEM;
		goto end_release;
	}

	memcpy(sst_drv_ctx->fw_in_mem, sst_drv_ctx->fw->data,
			sst_drv_ctx->fw->size);
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) {
		retval = sst_parse_fw_image(sst_drv_ctx->fw_in_mem,
					sst_drv_ctx->fw->size,
					&sst_drv_ctx->fw_sg_list);
		if (retval) {
			kfree(sst_drv_ctx->fw_in_mem);
			goto end_release;
		}
	}
end_release:
	release_firmware(sst_drv_ctx->fw);
	sst_drv_ctx->fw = NULL;
	return retval;
}

int sst_dma_firmware(struct sst_dma *dma, struct sst_sg_list *sg_list)
{
	int retval = 0;
	enum dma_ctrl_flags flag = DMA_CTRL_ACK;
	struct scatterlist *sg_src_list, *sg_dst_list;
	int length;
	pr_debug(" sst_dma_firmware\n");

	sg_src_list = sg_list->src;
	sg_dst_list = sg_list->dst;
	length = sg_list->list_len;
	sst_drv_ctx->desc = dma->ch->device->device_prep_dma_sg(dma->ch,
					sg_dst_list, length,
					sg_src_list, length, flag);
	if (!sst_drv_ctx->desc)
		return -EFAULT;
	sst_drv_ctx->desc->callback = sst_dma_transfer_complete;
	sst_drv_ctx->desc->callback_param = sst_drv_ctx;

	sst_drv_ctx->dma_info_blk.condition = false;
	sst_drv_ctx->dma_info_blk.ret_code = 0;
	sst_drv_ctx->dma_info_blk.on = true;

	sst_drv_ctx->desc->tx_submit(sst_drv_ctx->desc);
	retval = sst_wait_timeout(sst_drv_ctx, &sst_drv_ctx->dma_info_blk);
	if (retval)
		pr_err("sst_dma_firmware..timeout!\n");

	return retval;
}

void sst_dma_free_resources(struct sst_dma *dma)
{
	pr_debug("entry:%s\n", __func__);

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
	int ret_val = 0;

	pr_debug("load_fw called\n");
	BUG_ON(!fw_in_mem);

	ret_val = sst_drv_ctx->ops->reset();
	if (ret_val)
		return ret_val;
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		sst_download_fw_mrfld(fw_in_mem);
	else {
		/* get a dmac channel */
		sst_alloc_dma_chan(&sst_drv_ctx->dma);
		 /* allocate desc for transfer and submit */
		ret_val = sst_dma_firmware(&sst_drv_ctx->dma,
					&sst_drv_ctx->fw_sg_list);
		if (ret_val)
			goto free_dma;
	}
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_LOADED);
	/* bring sst out of reset */
	ret_val = sst_drv_ctx->ops->start();
	if (ret_val)
		goto free_dma;

	pr_debug("fw loaded successful!!!\n");
free_dma:
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID)
		sst_dma_free_resources(&sst_drv_ctx->dma);
	return ret_val;
}

/*This function is called when any codec/post processing library
 needs to be downloaded*/
static int sst_download_library(const struct firmware *fw_lib,
				struct snd_sst_lib_download_info *lib)
{
	unsigned long irq_flags;

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
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[i].ops_block);
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
	retval = sst_parse_fw_image(codec_fw, fw_lib->size,
					&sst_drv_ctx->library_list);
	if (retval) {
		kfree(codec_fw);
		return retval;
	}
	sst_alloc_dma_chan(&sst_drv_ctx->dma);
	retval = sst_dma_firmware(&sst_drv_ctx->dma,
					&sst_drv_ctx->library_list);
	if (retval)
		goto free_dma;

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
		retval = -ENOMEM;
		goto free_dma;
	}

	sst_fill_header(&msg->header, IPC_IA_LIB_DNLD_CMPLT, 1, pvt_id);
	sst_drv_ctx->alloc_block[i].sst_id = pvt_id;
	msg->header.part.data = sizeof(u32) + sizeof(*lib);
	lib->pvt_id = pvt_id;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), lib, sizeof(*lib));
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	pr_debug("Waiting for FW response Download complete\n");
	sst_drv_ctx->alloc_block[i].ops_block.condition = false;
	retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[i].ops_block);
	if (retval) {
		/* error */
		sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		retval = -EIO;
		goto free_dma;
	}

	pr_debug("FW success on Download complete\n");
	sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_RUNNING);
free_dma:
	sst_dma_free_resources(&sst_drv_ctx->dma);
	kfree(sst_drv_ctx->library_list.src);
	kfree(sst_drv_ctx->library_list.dst);
	sst_drv_ctx->library_list.list_len = 0;
	kfree(codec_fw);
	return retval;
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
	/* Prevent C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, CSTATE_EXIT_LATENCY_S0i1 - 1);
	error = sst_download_library(fw_lib, &dload_info);
	/* Re-enable Deeper C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, PM_QOS_DEFAULT_VALUE);
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
