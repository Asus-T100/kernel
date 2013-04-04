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
	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.full |= 0x382;
	csr.part.run_stall = 0x1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);

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

	mutex_lock(&sst_drv_ctx->csr_lock);
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
	mutex_unlock(&sst_drv_ctx->csr_lock);

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
	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);

	pr_debug("value:0x%llx\n", csr.full);

	csr.full |= 0x7;
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);

	pr_debug("value:0x%llx\n", csr.full);

	csr.full &= ~(0x1);
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);
	pr_debug("value:0x%llx\n", csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);
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
	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("value:0x%llx\n", csr.full);

	csr.full |= 0x7;
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("value:0x%llx\n", csr.full);

	csr.part.xt_snoop = 1;
	csr.full &= ~(0x5);
	sst_shim_write64(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read64(sst_drv_ctx->shim, SST_CSR);
	pr_debug("sst: Starting the DSP_merrifield:%llx\n", csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);
	return 0;
}

/**
 * intel_sst_set_bypass - Sets/clears the bypass bits
 *
 * This sets/clears the bypass bits
 */
void intel_sst_set_bypass_mfld(bool set)
{
	union config_status_reg csr;

	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	if (set == true)
		csr.full |= 0x380;
	else
		csr.part.bypass = 0;
	pr_debug("SetupByPass set %d Val 0x%x\n", set, csr.full);
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);

}

static int sst_fill_dstn(struct intel_sst_drv *sst, struct sst_probe_info info,
			Elf32_Phdr *pr, void **dstn, unsigned int *dstn_phys, int *mem_type)
{
#ifdef MRFLD_WORD_WA
	/* work arnd-since only 4 byte align copying is only allowed for ICCM */
	if ((pr->p_paddr >= info.iram_start) && (pr->p_paddr < info.iram_end)) {
		size_t data_size = pr->p_filesz % SST_ICCM_BOUNDARY;

		if (data_size)
			pr->p_filesz += 4 - data_size;
		*dstn = sst->iram + (pr->p_paddr - info.iram_start);
		*dstn_phys = sst->iram_base + pr->p_paddr - info.iram_start;
		*mem_type = 1;
#else
	if ((pr->p_paddr >= info.iram_start) &&
			(pr->p_paddr < info.iram_end)) {

		*dstn = sst->iram + (pr->p_paddr - info.iram_start);
		*dstn_phys = sst->iram_base + pr->p_paddr - info.iram_start;
		*mem_type = 1;
#endif
	} else if ((pr->p_paddr >= info.dram_start) &&
			(pr->p_paddr < info.dram_end)) {

		*dstn = sst->dram + (pr->p_paddr - info.dram_start);
		*dstn_phys = sst->dram_base + pr->p_paddr - info.dram_start;
		*mem_type = 1;
	} else if ((pr->p_paddr >= info.imr_start) &&
			(pr->p_paddr < info.imr_end)) {

		*dstn = sst->ddr + (pr->p_paddr - info.imr_start);
		*dstn_phys =  sst->ddr_base + pr->p_paddr - info.imr_start;
		*mem_type = 0;
	} else {
	       return -EINVAL;
	}
	return 0;
}

static void sst_fill_info(struct intel_sst_drv *sst,
			struct sst_probe_info *info)
{
	/* first we setup addresses to be used for elf sections */
	if (sst->info.iram_use) {
		info->iram_start = sst->info.iram_start;
		info->iram_end = sst->info.iram_end;
	} else {
		info->iram_start = sst->iram_base;
		info->iram_end = sst->iram_end;
	}
	if (sst->info.dram_use) {
		info->dram_start = sst->info.dram_start;
		info->dram_end = sst->info.dram_end;
	} else {
		info->dram_start = sst->dram_base;
		info->dram_end = sst->dram_end;
	}
	if (sst->info.imr_use) {
		info->imr_start = sst->info.imr_start;
		info->imr_end = sst->info.imr_end;
	} else {
		info->imr_start = relocate_imr_addr_mrfld(sst->ddr_base);
		info->imr_end = relocate_imr_addr_mrfld(sst->ddr_end);
	}

	info->dma_max_len = sst->info.dma_max_len;
	pr_debug("%s: dma_max_len 0x%x", __func__, info->dma_max_len);
}

static inline int sst_validate_fw_elf(const struct firmware *sst_fw)
{
	Elf32_Ehdr *elf;

	BUG_ON(!sst_fw);

	pr_debug("IN %s\n", __func__);

	elf = (Elf32_Ehdr *)sst_fw->data;

	if ((elf->e_ident[0] != 0x7F) || (elf->e_ident[1] != 'E') ||
		(elf->e_ident[2] != 'L') || (elf->e_ident[3] != 'F')) {
		pr_debug("ELF Header Not found!%d\n", sst_fw->size);
		return -EINVAL;
	}
	pr_debug("Valid ELF Header...%d\n", sst_fw->size);
	return 0;
}

/**
 * sst_validate_fw_image - validates the firmware signature
 *
 * @sst_fw_in_mem	: pointer to audio FW
 * @size		: size of the firmware
 * @module		: points to the FW modules
 * @num_modules		: points to the num of modules
 * This function validates the header signature in the FW image
 */
static int sst_validate_fw_image(const void *sst_fw_in_mem, unsigned long size,
		struct fw_module_header **module, u32 *num_modules)
{
	struct fw_header *header;

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
	*num_modules = header->modules;
	*module = (void *)sst_fw_in_mem + sizeof(*header);

	return 0;
}

/**
 * sst_validate_library - validates the library signature
 *
 * @fw_lib			: pointer to FW library
 * @slot			: pointer to the lib slot info
 * @entry_point		: out param, which contains the module entry point
 * This function is called before downloading the codec/postprocessing
 * library
 */
static int sst_validate_library(const struct firmware *fw_lib,
		struct lib_slot_info *slot,
		u32 *entry_point)
{
	struct fw_header *header;
	struct fw_module_header *module;
	struct fw_block_info *block;
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

static bool chan_filter(struct dma_chan *chan, void *param)
{
	struct sst_dma *dma = (struct sst_dma *)param;
	bool ret = false;

	/* we only need MID_DMAC1 as that can access DSP RAMs*/
	if (chan->device->dev == &dma->dmac->dev)
		ret = true;

	return ret;
}

static unsigned int
sst_get_elf_sg_len(struct intel_sst_drv *sst, Elf32_Ehdr *elf, Elf32_Phdr *pr,
		struct sst_probe_info info)
{
	unsigned int i = 0, count = 0;

	pr_debug("in %s: dma_max_len 0x%x\n", __func__, info.dma_max_len);

	while (i < elf->e_phnum) {
		if (pr[i].p_type == PT_LOAD) {

			if ((pr[i].p_paddr >= info.iram_start) &&
					(pr[i].p_paddr < info.iram_end &&
						pr[i].p_filesz)) {
				count += (pr[i].p_filesz) / info.dma_max_len;

				if ((pr[i].p_filesz) % info.dma_max_len)
					count++;

			} else if ((pr[i].p_paddr >= info.dram_start) &&
					(pr[i].p_paddr < info.dram_end &&
						pr[i].p_filesz)) {
				count += (pr[i].p_filesz) / info.dma_max_len;

				if ((pr[i].p_filesz) % info.dma_max_len)
					count++;

			} else if ((pr[i].p_paddr >= info.imr_start) &&
					(pr[i].p_paddr < info.imr_end &&
						pr[i].p_filesz)) {
				count += (pr[i].p_filesz) / info.dma_max_len;

				if ((pr[i].p_filesz) % info.dma_max_len)
					count++;
			}
		}
		i++;
	}

	pr_debug("gotcha count %d\n", count);
	return count;
}

static int
sst_init_dma_sg_list(struct intel_sst_drv *sst, unsigned int len,
		struct scatterlist **src, struct scatterlist **dstn)
{
	struct scatterlist *sg_src = NULL, *sg_dst = NULL;

	sg_src = kzalloc(sizeof(*sg_src)*(len), GFP_KERNEL);
	if (NULL == sg_src)
		return -ENOMEM;
	sg_init_table(sg_src, len);
	sg_dst = kzalloc(sizeof(*sg_dst)*(len), GFP_KERNEL);
	if (NULL == sg_dst) {
		kfree(sg_src);
		return -ENOMEM;
	}
	sg_init_table(sg_dst, len);
	*src = sg_src;
	*dstn = sg_dst;

	return 0;
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
	else if (sst_drv_ctx->pci_id == SST_MFLD_PCI_ID)
		dma->dmac = pci_get_device(PCI_VENDOR_ID_INTEL,
						PCI_DMAC_MFLD_ID, NULL);
	else if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		dma->dmac = pci_get_device(PCI_VENDOR_ID_INTEL,
						PCI_DMAC_MRFLD_ID, NULL);

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
	sst_wake_up_block(sst_drv_ctx, 0, FW_DWNL_ID, FW_DWNL_ID, NULL, 0);
}

static inline int sst_dma_wait_for_completion(struct intel_sst_drv *sst)
{
	int ret = 0;
	struct sst_block *block;
	/* call prep and wait */
	sst->desc->callback = sst_dma_transfer_complete;
	sst->desc->callback_param = sst;

	block = sst_create_block(sst, FW_DWNL_ID, FW_DWNL_ID);
	if (block == NULL)
		return -ENOMEM;

	sst->desc->tx_submit(sst_drv_ctx->desc);
	ret = sst_wait_timeout(sst, block);
	if (ret)
		dma_wait_for_async_tx(sst_drv_ctx->desc);
	sst_free_block(sst, block);
	return ret;
}

static int sst_dma_firmware(struct sst_dma *dma, struct sst_sg_list *sg_list)
{
	int retval = 0;
	enum dma_ctrl_flags flag = DMA_CTRL_ACK;
	struct scatterlist *sg_src_list, *sg_dst_list;
	int length;
	pr_debug("%s: use_lli %d\n", __func__, sst_drv_ctx->use_lli);

	sg_src_list = sg_list->src;
	sg_dst_list = sg_list->dst;
	length = sg_list->list_len;

	/* BY default PIMR is unsmasked
	 * FW gets unmaksed dma intr too, so mask it for FW to execute on mrfld
	 */
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		sst_shim_write(sst_drv_ctx->shim, SST_PIMR, 0xFFFF0034);

	if (sst_drv_ctx->use_lli) {
		sst_drv_ctx->desc = dma->ch->device->device_prep_dma_sg(dma->ch,
					sg_dst_list, length,
					sg_src_list, length, flag);
		if (!sst_drv_ctx->desc)
			return -EFAULT;
		retval = sst_dma_wait_for_completion(sst_drv_ctx);
		if (retval)
			pr_err("sst_dma_firmware..timeout!\n");
	} else {
		struct scatterlist *sg;
		dma_addr_t src_addr, dstn_addr;
		int i = 0;

		/* dma single block mode */
		for_each_sg(sg_src_list, sg, length, i) {
			pr_debug("dma desc %d, length %d\n", i, sg->length);
			src_addr = sg_phys(sg);
			dstn_addr = sg_phys(sg_dst_list);
			if (sg_dst_list)
				sg_dst_list = sg_next(sg_dst_list);
			sst_drv_ctx->desc = dma->ch->device->device_prep_dma_memcpy(
					dma->ch, dstn_addr, src_addr, sg->length, flag);
			if (!sst_drv_ctx->desc)
				return -EFAULT;
			retval = sst_dma_wait_for_completion(sst_drv_ctx);
			if (retval)
				pr_err("sst_dma_firmware..timeout!\n");

		}
	}

	return retval;
}

/*
 * sst_fill_sglist - Fill the sg list
 *
 * @from: src address of the fw
 * @to: virtual address of IRAM/DRAM
 * @block_size: size of the block
 * @sg_src: source scatterlist pointer
 * @sg_dst: Destination scatterlist pointer
 * @fw_sg_list: Pointer to the sg_list
 * @dma_max_len: maximum len of the DMA block
 *
 * Parses modules that need to be placed in SST IRAM and DRAM
 * and stores them in a sg list for transfer
 * returns error or 0 if list creation fails or pass.
  */
static int sst_fill_sglist(unsigned long from, unsigned long to,
		u32 block_size, struct scatterlist **sg_src, struct scatterlist **sg_dstn,
		struct sst_sg_list *fw_sg_list, u32 dma_max_len)
{
	u32 offset = 0;
	int len = 0;
	unsigned long dstn, src;

	pr_debug("%s entry", __func__);

	do {
		dstn = (unsigned long) (to + offset);
		src = (unsigned long) (from + offset);

		/* split blocks to dma_max_len */

		len = block_size - offset;
		pr_debug("DMA blk src %lx,dstn %lx,len %d,offset %d, size %d\n",
			src, dstn, len, offset, block_size);
		if (len > dma_max_len) {
			pr_debug("block size exceeds %d\n", dma_max_len);
			len = dma_max_len;
			offset += len;
		} else {
			pr_debug("Node length less that %d\n", dma_max_len);
			offset = 0;
		}

		if (!sg_src || !sg_dstn)
			return -ENOMEM;

		sg_set_buf(*sg_src, (void *) src, len);
		sg_set_buf(*sg_dstn, (void *) dstn, len);
		*sg_src = sg_next(*sg_src);
		*sg_dstn = sg_next(*sg_dstn);

		/* TODO: is sg_idx required? */
		if (sst_drv_ctx->info.use_elf == true)
			fw_sg_list->sg_idx++;
	} while (offset > 0);

	return 0;
}

static int sst_parse_elf_module_dma(struct intel_sst_drv *sst, const void *fw,
		 struct sst_probe_info info, Elf32_Phdr *pr,
		 struct scatterlist **sg_src, struct scatterlist **sg_dstn,
		 struct sst_sg_list *fw_sg_list)
{
	unsigned long dstn, src;
	unsigned int dstn_phys;
	int ret_val = 0;
	int mem_type;

	ret_val = sst_fill_dstn(sst, info, pr, (void *)&dstn, &dstn_phys, &mem_type);
	if (ret_val)
		return ret_val;

	dstn = (unsigned long) phys_to_virt(dstn_phys);
	src = (unsigned long) (fw + pr->p_offset);

	ret_val = sst_fill_sglist(src, dstn, pr->p_filesz,
				sg_src, sg_dstn, fw_sg_list, sst->info.dma_max_len);

	return ret_val;
}

static int
sst_parse_elf_fw_dma(struct intel_sst_drv *sst, const void *fw_in_mem,
			struct sst_sg_list *fw_sg_list)
{
	int i = 0;

	Elf32_Ehdr *elf;
	Elf32_Phdr *pr;
	struct sst_probe_info info;
	struct scatterlist *sg_src = NULL, *sg_dst = NULL;
	unsigned int sg_len;

	BUG_ON(!fw_in_mem);

	elf = (Elf32_Ehdr *)fw_in_mem;
	pr = (Elf32_Phdr *) (fw_in_mem + elf->e_phoff);
	pr_debug("%s entry\n", __func__);

	sst_fill_info(sst, &info);

	sg_len = sst_get_elf_sg_len(sst, elf, pr, info);
	if (sg_len == 0) {
		pr_err("we got NULL sz ELF, abort\n");
		return -EIO;
	}

	if (sst_init_dma_sg_list(sst, sg_len, &sg_src, &sg_dst))
		return -ENOMEM;
	fw_sg_list->src = sg_src;
	fw_sg_list->dst = sg_dst;
	fw_sg_list->list_len = sg_len;
	fw_sg_list->sg_idx = 0;

	while (i < elf->e_phnum) {
		if ((pr[i].p_type == PT_LOAD) && (pr[i].p_filesz))
			sst_parse_elf_module_dma(sst, fw_in_mem, info, &pr[i],
					&sg_src, &sg_dst, fw_sg_list);
		i++;
	}
	return 0;
}

/**
 * sst_parse_module_dma - Parse audio FW modules and populate the dma list
 *
 * @module	: FW module header
 * @sg_list	: Pointer to the sg_list to be populated
 * Count the length for scattergather list
 * and create the scattergather list of same length
 * returns error or 0 if module sizes are proper
 */
static int sst_parse_module_dma(struct fw_module_header *module,
				struct sst_sg_list *sg_list)
{
	struct fw_block_info *block;
	u32 count;
	unsigned long ram, src;
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

	sst_init_dma_sg_list(sst_drv_ctx, sg_len, &sg_src, &sg_dst);
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
		ram = (unsigned long)(ram + block->ram_offset);
		src = (unsigned long) (void *)block + sizeof(*block);

		retval = sst_fill_sglist(src, ram,
				block->size, &sg_src, &sg_dst,
				sg_list, sst_drv_ctx->info.dma_max_len);
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

/**
 * sst_parse_fw_dma - parse the firmware image & populate the list for dma
 *
 * @sst_fw_in_mem	: pointer to audio fw
 * @size		: size of the firmware
 * @fw_list		: pointer to sst_sg_list to be populated
 * This function parses the FW image and saves the parsed image in the list
 * for dma
 */
static int sst_parse_fw_dma(const void *sst_fw_in_mem, unsigned long size,
				struct sst_sg_list *fw_list)
{
	struct fw_module_header *module;
	u32 count, num_modules;
	int ret_val;

	ret_val = sst_validate_fw_image(sst_fw_in_mem, size,
				&module, &num_modules);
	if (ret_val)
		return ret_val;

	for (count = 0; count < num_modules; count++) {
		/* module */
		ret_val = sst_parse_module_dma(module, fw_list);
		if (ret_val)
			return ret_val;
		module = (void *)module + sizeof(*module) + module->mod_size ;
	}

	return 0;
}

static void sst_dma_free_resources(struct sst_dma *dma)
{
	pr_debug("entry:%s\n", __func__);

	dma_release_channel(dma->ch);
}

void sst_fill_config(struct intel_sst_drv *sst_ctx)
{
	u32 sign;
	int len;

	sign = SST_CONFIG_SSP_SIGN;

	if (!sst_ctx->ssp_config)
		return;
	len = sst_ctx->ssp_config->size;

	memcpy_toio(sst_ctx->dram, &sign, sizeof(u32));
	memcpy_toio(sst_ctx->dram + sizeof(u32), (sst_ctx->ssp_config->bytes), len);
	memcpy_toio(sst_ctx->dram + len + sizeof(u32), &sst_ctx->shim_phy_add, sizeof(u32));
	memcpy_toio(sst_ctx->dram + len + sizeof(u64), &sst_ctx->mailbox_add, sizeof(u32));
}

/**
 * sst_do_dma - function allocs and initiates the DMA
 *
 * @sg_list: Pointer to dma list on which the dma needs to be initiated
 *
 * Triggers the DMA
 */
static int sst_do_dma(struct sst_sg_list *sg_list)
{
	int ret_val;

	/* get a dmac channel */
	sst_alloc_dma_chan(&sst_drv_ctx->dma);

	/* allocate desc for transfer and submit */
	ret_val = sst_dma_firmware(&sst_drv_ctx->dma, sg_list);

	sst_dma_free_resources(&sst_drv_ctx->dma);

	return ret_val;
}

/*
 * sst_fill_memcpy_list - Fill the memcpy list
 *
 * @memcpy_list: List to be filled
 * @destn: Destination addr to be filled in the list
 * @src: Source addr to be filled in the list
 * @size: Size to be filled in the list
 *
 * Adds the node to the list after required fields
 * are populated in the node
 */

static int sst_fill_memcpy_list(struct list_head *memcpy_list,
			void *destn, const void *src, u32 size, bool is_io)
{
	struct sst_memcpy_list *listnode;

	listnode = kzalloc(sizeof(*listnode), GFP_KERNEL);
	if (listnode == NULL)
		return -ENOMEM;
	listnode->dstn = destn;
	listnode->src = src;
	listnode->size = size;
	listnode->is_io = is_io;
	list_add_tail(&listnode->memcpylist, memcpy_list);

	return 0;
}

static int sst_parse_elf_module_memcpy(struct intel_sst_drv *sst,
		const void *fw, struct sst_probe_info info, Elf32_Phdr *pr,
		struct list_head *memcpy_list)
{
	void *dstn;
	unsigned int dstn_phys;
	int ret_val = 0;
	int mem_type;

	ret_val = sst_fill_dstn(sst, info, pr, &dstn, &dstn_phys, &mem_type);
	if (ret_val)
		return ret_val;

	ret_val = sst_fill_memcpy_list(memcpy_list,
					dstn, fw + pr->p_offset, pr->p_filesz, mem_type);
	if (ret_val)
		return ret_val;

	return 0;
}

static int
sst_parse_elf_fw_memcpy(struct intel_sst_drv *sst, const void *fw_in_mem,
			struct list_head *memcpy_list)
{
	int i = 0;

	Elf32_Ehdr *elf;
	Elf32_Phdr *pr;
	struct sst_probe_info info;

	BUG_ON(!fw_in_mem);

	elf = (Elf32_Ehdr *)fw_in_mem;
	pr = (Elf32_Phdr *) (fw_in_mem + elf->e_phoff);
	pr_debug("%s entry\n", __func__);

	sst_fill_info(sst, &info);

	while (i < elf->e_phnum) {
		if (pr[i].p_type == PT_LOAD)
			sst_parse_elf_module_memcpy(sst, fw_in_mem, info,
					&pr[i], memcpy_list);
		i++;
	}
	return 0;
}

/**
 * sst_parse_module_memcpy - Parse audio FW modules and populate the memcpy list
 *
 * @module		: FW module header
 * @memcpy_list	: Pointer to the list to be populated
 * Create the memcpy list as the number of block to be copied
 * returns error or 0 if module sizes are proper
 */
static int sst_parse_module_memcpy(struct fw_module_header *module,
				struct list_head *memcpy_list)
{
	struct fw_block_info *block;
	u32 count;
	int ret_val = 0;
	void __iomem *ram_iomem;

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
			ram_iomem = sst_drv_ctx->iram;
			break;
		case SST_DRAM:
			ram_iomem = sst_drv_ctx->dram;
			break;
		default:
			pr_err("wrong ram type0x%x in block0x%x\n",
					block->type, count);
			return -EINVAL;
		}

		ret_val = sst_fill_memcpy_list(memcpy_list,
				ram_iomem + block->ram_offset,
				(void *)block + sizeof(*block), block->size, 1);
		if (ret_val)
			return ret_val;

		block = (void *)block + sizeof(*block) + block->size;
	}
	return 0;
}

/**
 * sst_parse_fw_memcpy - parse the firmware image & populate the list for memcpy
 *
 * @sst_fw_in_mem	: pointer to audio fw
 * @size		: size of the firmware
 * @fw_list		: pointer to list_head to be populated
 * This function parses the FW image and saves the parsed image in the list
 * for memcpy
 */
static int sst_parse_fw_memcpy(const void *sst_fw_in_mem, unsigned long size,
				struct list_head *fw_list)
{
	struct fw_module_header *module;
	u32 count, num_modules;
	int ret_val;

	ret_val = sst_validate_fw_image(sst_fw_in_mem, size,
				&module, &num_modules);
	if (ret_val)
		return ret_val;

	for (count = 0; count < num_modules; count++) {
		/* module */
		ret_val = sst_parse_module_memcpy(module, fw_list);
		if (ret_val)
			return ret_val;
		module = (void *)module + sizeof(*module) + module->mod_size ;
	}

	return 0;
}

/**
 * sst_do_memcpy - function initiates the memcpy
 *
 * @memcpy_list: Pter to memcpy list on which the memcpy needs to be initiated
 *
 * Triggers the memcpy
 */
static void sst_do_memcpy(struct list_head *memcpy_list)
{
	struct sst_memcpy_list *listnode;

	list_for_each_entry(listnode, memcpy_list, memcpylist) {
		if (listnode->is_io == true)
			memcpy_toio((void __iomem *)listnode->dstn, listnode->src,
							listnode->size);
		else
			memcpy(listnode->dstn, listnode->src, listnode->size);
	}
}

void sst_memcpy_free_resources(void)
{
	struct sst_memcpy_list *listnode, *tmplistnode;

	pr_debug("entry:%s\n", __func__);

	/*Free the list*/
	if (!list_empty(&sst_drv_ctx->memcpy_list)) {
		list_for_each_entry_safe(listnode, tmplistnode,
				&sst_drv_ctx->memcpy_list, memcpylist) {
			list_del(&listnode->memcpylist);
			kfree(listnode);
		}
	}

	if (!list_empty(&sst_drv_ctx->libmemcpy_list)) {
		list_for_each_entry_safe(listnode, tmplistnode,
				&sst_drv_ctx->libmemcpy_list, memcpylist) {
			list_del(&listnode->memcpylist);
			kfree(listnode);
		}
	}
}

/*
 * sst_request_fw - requests audio fw from kernel and saves a copy
 *
 * This function requests the SST FW from the kernel, parses it and
 * saves a copy in the driver context
 */
static int sst_request_fw(struct intel_sst_drv *sst)
{
	int retval = 0;
	char name[20];

#ifndef MRFLD_TEST_ON_MFLD
	snprintf(name, sizeof(name), "%s%04x%s", "fw_sst_",
				sst->pci_id, ".bin");
#else
	snprintf(name, sizeof(name), "%s%04x%s", "fw_sst_",
				sst->pci_id, "_mt.bin");
#endif

	pr_debug("Requesting FW %s now...\n", name);
	retval = request_firmware(&sst->fw, name,
				 &sst->pci->dev);
	if (sst->fw == NULL) {
		pr_err("sst->fw is returning as null\n");
		return -EINVAL;
	}
	if (retval) {
		pr_err("request fw failed %d\n", retval);
		return retval;
	}
#ifndef MRFLD_TEST_ON_MFLD
	if (sst->pci_id == SST_MRFLD_PCI_ID)
#endif
		retval = sst_validate_fw_elf(sst->fw);
	if (retval != 0) {
		pr_err("FW image invalid...\n");
		goto end_release;
	}
	sst->fw_in_mem = kzalloc(sst->fw->size, GFP_KERNEL);
	if (!sst->fw_in_mem) {
		pr_err("%s unable to allocate memory\n", __func__);
		retval = -ENOMEM;
		goto end_release;
	}
	pr_debug("copied fw to %p", sst->fw_in_mem);
	pr_debug("phys: %x", virt_to_phys(sst->fw_in_mem));
	memcpy(sst->fw_in_mem, sst->fw->data,
			sst->fw->size);
#ifndef MRFLD_TEST_ON_MFLD
	if (sst->use_dma) {
		if (sst->info.use_elf == true)
			retval = sst_parse_elf_fw_dma(sst,
				sst->fw_in_mem, &sst->fw_sg_list);
		else
			retval = sst_parse_fw_dma(sst->fw_in_mem,
				sst->fw->size, &sst->fw_sg_list);
	} else {
		if (sst->info.use_elf == true)
			retval = sst_parse_elf_fw_memcpy(sst,
				sst->fw_in_mem, &sst->memcpy_list);
		else
			retval = sst_parse_fw_memcpy(sst->fw_in_mem,
				sst->fw->size, &sst->memcpy_list);
	}

	if (retval) {
		kfree(sst->fw_in_mem);
		goto end_release;
	}

#endif
end_release:
	release_firmware(sst->fw);
	sst->fw = NULL;
	return retval;
}

static inline void print_lib_info(struct snd_sst_lib_download_info *resp)
{
	pr_debug("codec Type %d Ver %d Built %s: %s\n",
		resp->dload_lib.lib_info.lib_type,
		resp->dload_lib.lib_info.lib_version,
		resp->dload_lib.lib_info.b_date,
		resp->dload_lib.lib_info.b_time);
}

/* sst_download_library - This function is called when any
 codec/post processing library needs to be downloaded */
static int sst_download_library(const struct firmware *fw_lib,
				struct snd_sst_lib_download_info *lib)
{
	unsigned long irq_flags;
	int ret_val = 0;

	/* send IPC message and wait */
	u8 pvt_id;
	struct ipc_post *msg = NULL;
	union config_status_reg csr;
	struct snd_sst_str_type str_type = {0};
	int retval = 0;
	void *codec_fw;
	struct sst_block *block;

	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	ret_val = sst_create_block_and_ipc_msg(&msg, true, sst_drv_ctx, &block,
				IPC_IA_PREP_LIB_DNLD, pvt_id);
	if (ret_val) {
		pr_err("library download failed\n");
		return ret_val;
	}

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
	retval = sst_wait_timeout(sst_drv_ctx, block);
	if (block->data) {
		struct snd_sst_str_type *str_type =
			(struct snd_sst_str_type *)block->data;
		if (str_type->result) {
			/* error */
			pr_err("Prep codec downloaded failed %d\n",
					str_type->result);
			retval = -EIO;
			goto free_block;
		}
	} else if (retval != 0) {
		retval = -EIO;
		goto free_block;
	}
	pr_debug("FW responded, ready for download now...\n");
	/* downloading on success */
	mutex_lock(&sst_drv_ctx->sst_lock);
	sst_drv_ctx->sst_state = SST_FW_LOADED;
	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = readl(sst_drv_ctx->shim + SST_CSR);
	csr.part.run_stall = 1;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0x7;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);

	codec_fw = kzalloc(fw_lib->size, GFP_KERNEL);
	if (!codec_fw) {
		retval = -ENOMEM;
		goto free_block_unlock;
	}
	memcpy(codec_fw, fw_lib->data, fw_lib->size);

	if (sst_drv_ctx->use_dma)
		retval = sst_parse_fw_dma(codec_fw, fw_lib->size,
				 &sst_drv_ctx->library_list);
	else
		retval = sst_parse_fw_memcpy(codec_fw, fw_lib->size,
				 &sst_drv_ctx->libmemcpy_list);

	if (retval) {
		kfree(codec_fw);
		goto free_block_unlock;
	}

	if (sst_drv_ctx->use_dma) {
		ret_val = sst_do_dma(&sst_drv_ctx->library_list);
		if (ret_val) {
			pr_err("sst_do_dma failed, abort\n");

			goto free_resources;
		}
	} else {
		sst_do_memcpy(&sst_drv_ctx->libmemcpy_list);
	}
	/* set the FW to running again */
	mutex_lock(&sst_drv_ctx->csr_lock);
	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.bypass = 0x0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);

	csr.full = sst_shim_read(sst_drv_ctx->shim, SST_CSR);
	csr.part.run_stall = 0;
	sst_shim_write(sst_drv_ctx->shim, SST_CSR, csr.full);
	mutex_unlock(&sst_drv_ctx->csr_lock);

	/* send download complete and wait */
	if (sst_create_ipc_msg(&msg, true)) {
		retval = -ENOMEM;
		goto free_resources;
	}

	block->condition = false;
	block->msg_id = IPC_IA_LIB_DNLD_CMPLT;
	sst_fill_header(&msg->header, IPC_IA_LIB_DNLD_CMPLT, 1, pvt_id);
	msg->header.part.data = sizeof(u32) + sizeof(*lib);
	lib->pvt_id = pvt_id;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), lib, sizeof(*lib));
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	pr_debug("Waiting for FW response Download complete\n");
	retval = sst_wait_timeout(sst_drv_ctx, block);

	if (block->data) {
		struct snd_sst_lib_download_info *resp = block->data;
		retval = resp->result;
		if (retval) {
			pr_err("err in lib dload %x\n", resp->result);
			sst_drv_ctx->sst_state = SST_UN_INIT;
			goto free_resources;
		} else {
			pr_debug("Codec download complete...\n");
			print_lib_info(resp);
		}
	} else if (retval) {
		/* error */
		sst_drv_ctx->sst_state = SST_UN_INIT;
		retval = -EIO;
		goto free_resources;
	}

	pr_debug("FW success on Download complete\n");
	sst_drv_ctx->sst_state = SST_FW_RUNNING;

free_resources:
	if (sst_drv_ctx->use_dma) {
		kfree(sst_drv_ctx->library_list.src);
		kfree(sst_drv_ctx->library_list.dst);
		sst_drv_ctx->library_list.list_len = 0;
	}

	kfree(codec_fw);
free_block_unlock:
	mutex_unlock(&sst_drv_ctx->sst_lock);
free_block:
	sst_free_block(sst_drv_ctx, block);
	return retval;
}

/*
 * Writing the DDR physical base to DCCM offset
 * so that FW can use it to setup TLB
 */
static void mrfld_dccm_config_write(void __iomem *dram_base, u32 ddr_base)
{
	void __iomem *addr;
	u32 bss_reset = 0;

	addr = dram_base + MRFLD_FW_DDR_BASE_OFFSET;
	memcpy_toio(addr, &ddr_base, sizeof(u32));
	bss_reset |= (1 << MRFLD_FW_BSS_RESET_BIT);
	addr = dram_base + MRFLD_FW_FEATURE_BASE_OFFSET;
	memcpy_toio(addr, &bss_reset, sizeof(u32));
	pr_debug("mrfld config written to DCCM\n");
}

/**
 * sst_load_fw - function to load FW into DSP
 *
 *
 * Transfers the FW to DSP using dma/memcpy
 */
int sst_load_fw(void)
{
	int ret_val = 0;
	struct sst_block *block;

	pr_debug("sst_load_fw\n");

	if (sst_drv_ctx->sst_state != SST_START_INIT)
		return -EAGAIN;

	if (!sst_drv_ctx->fw_in_mem) {
		ret_val = sst_request_fw(sst_drv_ctx);
		if (ret_val)
			return ret_val;
	}

	BUG_ON(!sst_drv_ctx->fw_in_mem);
	block = sst_create_block(sst_drv_ctx, 0, FW_DWNL_ID);
	if (block == NULL)
		return -ENOMEM;

	/* Prevent C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, CSTATE_EXIT_LATENCY_S0i1 - 1);

	ret_val = sst_drv_ctx->ops->reset();
	if (ret_val)
		goto restore;

	if (sst_drv_ctx->use_dma) {
		ret_val = sst_do_dma(&sst_drv_ctx->fw_sg_list);
		if (ret_val) {
			pr_err("sst_do_dma failed, abort\n");
			goto restore;
		}
	} else {
		sst_do_memcpy(&sst_drv_ctx->memcpy_list);
	}

	/* Write the DRAM config before enabling FW
	 */
	if (sst_drv_ctx->pci_id == SST_MRFLD_PCI_ID)
		mrfld_dccm_config_write(sst_drv_ctx->dram,
						sst_drv_ctx->ddr_base);

	sst_drv_ctx->sst_state = SST_FW_LOADED;
	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID)
		sst_fill_config(sst_drv_ctx);

	/* bring sst out of reset */
	ret_val = sst_drv_ctx->ops->start();
	if (ret_val)
		goto restore;

	ret_val = sst_wait_timeout(sst_drv_ctx, block);
	if (ret_val) {
		pr_err("fw download failed %d\n" , ret_val);
		/* assume FW d/l failed due to timeout*/
		ret_val = -EBUSY;

	}

restore:
	/* Re-enable Deeper C-states beyond C6 */
	pm_qos_update_request(sst_drv_ctx->qos, PM_QOS_DEFAULT_VALUE);
	sst_free_block(sst_drv_ctx, block);

	return ret_val;
}

/**
 * sst_load_library - function to load FW into DSP
 *
 * @lib: Pointer to the lib download structure
 * @ops: Contains the stream ops
 * This function is called when FW requests for a particular library download
 * This function prepares & downloads the library
 */
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
	if (fw_lib == NULL) {
		pr_err("fw_lib pointer is returning null\n");
		return -EINVAL;
	}
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
