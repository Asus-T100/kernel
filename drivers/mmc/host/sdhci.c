/*
 *  linux/drivers/mmc/host/sdhci.c - Secure Digital Host Controller Interface driver
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * Thanks to the following companies for their support:
 *
 *     - JMicron (hardware and technical support)
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <linux/leds.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include "sdhci.h"

#define DRIVER_NAME "sdhci"

#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)

#if defined(CONFIG_LEDS_CLASS) || (defined(CONFIG_LEDS_CLASS_MODULE) && \
	defined(CONFIG_MMC_SDHCI_MODULE))
#define SDHCI_USE_LEDS_CLASS
#endif

#define MAX_TUNING_LOOP 40

static unsigned int debug_quirks = 0;
static unsigned int debug_quirks2;

static void sdhci_finish_data(struct sdhci_host *);

static void sdhci_send_command(struct sdhci_host *, struct mmc_command *);
static void sdhci_finish_command(struct sdhci_host *);
static int sdhci_execute_tuning(struct mmc_host *mmc, u32 opcode);
static void sdhci_tuning_timer(unsigned long data);
static void sdhci_do_set_ios(struct sdhci_host *host, struct mmc_ios *ios);
static int sdhci_do_start_signal_voltage_switch(struct sdhci_host *,
						struct mmc_ios *, bool);

#ifdef CONFIG_PM_RUNTIME
static int sdhci_runtime_pm_get(struct sdhci_host *host);
static int sdhci_runtime_pm_put(struct sdhci_host *host);
#else
static inline int sdhci_runtime_pm_get(struct sdhci_host *host)
{
	return 0;
}
static inline int sdhci_runtime_pm_put(struct sdhci_host *host)
{
	return 0;
}
#endif

static void sdhci_dumpregs(struct sdhci_host *host)
{
	pr_info(DRIVER_NAME ": =========== REGISTER DUMP (%s)===========\n",
		mmc_hostname(host->mmc));

	pr_info(DRIVER_NAME ": Sys addr: 0x%08x | Version:  0x%08x\n",
		sdhci_readl(host, SDHCI_DMA_ADDRESS),
		sdhci_readw(host, SDHCI_HOST_VERSION));
	pr_info(DRIVER_NAME ": Blk size: 0x%08x | Blk cnt:  0x%08x\n",
		sdhci_readw(host, SDHCI_BLOCK_SIZE),
		sdhci_readw(host, SDHCI_BLOCK_COUNT));
	pr_info(DRIVER_NAME ": Argument: 0x%08x | Trn mode: 0x%08x\n",
		sdhci_readl(host, SDHCI_ARGUMENT),
		sdhci_readw(host, SDHCI_TRANSFER_MODE));
	pr_info(DRIVER_NAME ": Present:  0x%08x | Host ctl: 0x%08x\n",
		sdhci_readl(host, SDHCI_PRESENT_STATE),
		sdhci_readb(host, SDHCI_HOST_CONTROL));
	pr_info(DRIVER_NAME ": Power:    0x%08x | Blk gap:  0x%08x\n",
		sdhci_readb(host, SDHCI_POWER_CONTROL),
		sdhci_readb(host, SDHCI_BLOCK_GAP_CONTROL));
	pr_info(DRIVER_NAME ": Wake-up:  0x%08x | Clock:    0x%08x\n",
		sdhci_readb(host, SDHCI_WAKE_UP_CONTROL),
		sdhci_readw(host, SDHCI_CLOCK_CONTROL));
	pr_info(DRIVER_NAME ": Timeout:  0x%08x | Int stat: 0x%08x\n",
		sdhci_readb(host, SDHCI_TIMEOUT_CONTROL),
		sdhci_readl(host, SDHCI_INT_STATUS));
	pr_info(DRIVER_NAME ": Int enab: 0x%08x | Sig enab: 0x%08x\n",
		sdhci_readl(host, SDHCI_INT_ENABLE),
		sdhci_readl(host, SDHCI_SIGNAL_ENABLE));
	pr_info(DRIVER_NAME ": AC12 err: 0x%08x | Slot int: 0x%08x\n",
		sdhci_readw(host, SDHCI_ACMD12_ERR),
		sdhci_readw(host, SDHCI_SLOT_INT_STATUS));
	pr_info(DRIVER_NAME ": Caps:     0x%08x | Caps_1:   0x%08x\n",
		sdhci_readl(host, SDHCI_CAPABILITIES),
		sdhci_readl(host, SDHCI_CAPABILITIES_1));
	pr_info(DRIVER_NAME ": Cmd:      0x%08x | Max curr: 0x%08x\n",
		sdhci_readw(host, SDHCI_COMMAND),
		sdhci_readl(host, SDHCI_MAX_CURRENT));
	pr_info(DRIVER_NAME ": Host ctl2: 0x%08x\n",
		sdhci_readw(host, SDHCI_HOST_CONTROL2));

	if (host->flags & SDHCI_USE_ADMA)
		pr_info(DRIVER_NAME ": ADMA Err: 0x%08x | ADMA Ptr: 0x%08x\n",
		       readl(host->ioaddr + SDHCI_ADMA_ERROR),
		       readl(host->ioaddr + SDHCI_ADMA_ADDRESS));

	if (host->cmd)
		pr_info(DRIVER_NAME
				": command pending, Cmdcode: %d\n",
				host->cmd->opcode);
	else
		pr_info(DRIVER_NAME ": No command pending\n");

	if (host->data)
		pr_info(DRIVER_NAME ": data pending\n");
	else
		pr_info(DRIVER_NAME ": No data pending\n");

	pr_info(DRIVER_NAME ": pwr:     0x%x | clock:	%d\n",
				host->pwr, host->clock);
#ifdef CONFIG_PM_RUNTIME
	pr_info(DRIVER_NAME ": usage_count %d | Runtime_status %d\n",
			atomic_read(&host->mmc->parent->power.usage_count),
			host->mmc->parent->power.runtime_status);
#endif
	if (test_bit(TASKLET_STATE_SCHED, &host->finish_tasklet.state))
		pr_info(DRIVER_NAME
				": finish_tasklet pending running, state %ld\n",
				host->finish_tasklet.state);
	else
		pr_info(DRIVER_NAME
				": finish_tasklet NOT start, state %ld\n",
				host->finish_tasklet.state);

	pr_info(DRIVER_NAME ": ===========================================\n");
}

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void sdhci_clear_set_irqs(struct sdhci_host *host, u32 clear, u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	ier &= ~clear;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}

static void sdhci_unmask_irqs(struct sdhci_host *host, u32 irqs)
{
	sdhci_clear_set_irqs(host, 0, irqs);
}

static void sdhci_mask_irqs(struct sdhci_host *host, u32 irqs)
{
	sdhci_clear_set_irqs(host, irqs, 0);
}

static void sdhci_set_card_detection(struct sdhci_host *host, bool enable)
{
	u32 present, irqs;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION) ||
	    (host->mmc->caps & MMC_CAP_NONREMOVABLE) ||
	    (host->quirks2 & SDHCI_QUIRK2_BAD_SD_CD))
		return;

	present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
			      SDHCI_CARD_PRESENT;
	irqs = present ? SDHCI_INT_CARD_REMOVE : SDHCI_INT_CARD_INSERT;

	if (enable)
		sdhci_unmask_irqs(host, irqs);
	else
		sdhci_mask_irqs(host, irqs);
}

static void sdhci_enable_card_detection(struct sdhci_host *host)
{
	sdhci_set_card_detection(host, true);
}

static void sdhci_disable_card_detection(struct sdhci_host *host)
{
	sdhci_set_card_detection(host, false);
}

static void sdhci_busy_wait(struct sdhci_host *host, u32 delay)
{
	/* totally 'delay' us, each loop 4us */
	u32 loop = delay / 4;
	while (loop) {
		/* have a delay here */
		udelay(4);
		/* read register to make sure host won't be clock gated */
		sdhci_readw(host, SDHCI_HOST_VERSION);
		loop--;
	}
}

static void sdhci_reset(struct sdhci_host *host, u8 mask)
{
	unsigned long timeout;
	u32 uninitialized_var(ier);

	if (host->quirks & SDHCI_QUIRK_NO_CARD_NO_RESET) {
		if ((host->quirks2 & SDHCI_QUIRK2_BAD_SD_CD) &&
				host->ops->get_cd) {
			if (!host->ops->get_cd(host))
				return;
		} else if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			SDHCI_CARD_PRESENT))
			return;
	}

	if (host->quirks & SDHCI_QUIRK_RESTORE_IRQS_AFTER_RESET)
		ier = sdhci_readl(host, SDHCI_INT_ENABLE);

	if (host->ops->platform_reset_enter)
		host->ops->platform_reset_enter(host, mask);

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);

	if (mask & SDHCI_RESET_ALL)
		host->clock = 0;

	/* Wait max 100 ms */
	timeout = 10000;

	/* hw clears the bit when it's done */
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			pr_err("%s: Reset 0x%x never completed.\n",
				mmc_hostname(host->mmc), (int)mask);
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		udelay(10);
	}

	if (host->ops->platform_reset_exit)
		host->ops->platform_reset_exit(host, mask);

	if (host->quirks & SDHCI_QUIRK_RESTORE_IRQS_AFTER_RESET)
		sdhci_clear_set_irqs(host, SDHCI_INT_ALL_MASK, ier);

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if ((host->ops->enable_dma) && (mask & SDHCI_RESET_ALL))
			host->ops->enable_dma(host);
	}
}

static void sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);

static void sdhci_init(struct sdhci_host *host, int soft)
{
	if (soft)
		sdhci_reset(host, SDHCI_RESET_CMD|SDHCI_RESET_DATA);
	else
		sdhci_reset(host, SDHCI_RESET_ALL);

	sdhci_clear_set_irqs(host, SDHCI_INT_ALL_MASK,
		SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END_BIT |
		SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
		SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
		SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE);

	if (soft) {
		/* force clock reconfiguration */
		host->clock = 0;
		sdhci_set_ios(host->mmc, &host->mmc->ios);
	}
}

static void sdhci_reinit(struct sdhci_host *host)
{
	sdhci_init(host, 0);
	sdhci_enable_card_detection(host);
}

static void sdhci_activate_led(struct sdhci_host *host)
{
	u8 ctrl;

	if (!(host->mmc->caps2 & MMC_CAP2_LED_SUPPORT))
		return;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl |= SDHCI_CTRL_LED;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void sdhci_deactivate_led(struct sdhci_host *host)
{
	u8 ctrl;

	if (!(host->mmc->caps2 & MMC_CAP2_LED_SUPPORT))
		return;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_LED;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

#ifdef SDHCI_USE_LEDS_CLASS
static void sdhci_led_control(struct led_classdev *led,
	enum led_brightness brightness)
{
	struct sdhci_host *host = container_of(led, struct sdhci_host, led);
	unsigned long flags;

	if (!(host->mmc->caps2 & MMC_CAP2_LED_SUPPORT))
		return;

	spin_lock_irqsave(&host->lock, flags);

	if (host->runtime_suspended)
		goto out;

	if (brightness == LED_OFF)
		sdhci_deactivate_led(host);
	else
		sdhci_activate_led(host);
out:
	spin_unlock_irqrestore(&host->lock, flags);
}
#endif

/*****************************************************************************\
 *                                                                           *
 * Core functions                                                            *
 *                                                                           *
\*****************************************************************************/

static void sdhci_read_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 uninitialized_var(scratch);
	u8 *buf;

	DBG("PIO reading\n");

	blksize = host->data->blksz;
	chunk = 0;

	local_irq_save(flags);

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		while (len) {
			if (chunk == 0) {
				scratch = sdhci_readl(host, SDHCI_BUFFER);
				chunk = 4;
			}

			*buf = scratch & 0xFF;

			buf++;
			scratch >>= 8;
			chunk--;
			len--;
		}
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void sdhci_write_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 scratch;
	u8 *buf;

	DBG("PIO writing\n");

	blksize = host->data->blksz;
	chunk = 0;
	scratch = 0;

	local_irq_save(flags);

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		while (len) {
			scratch |= (u32)*buf << (chunk * 8);

			buf++;
			chunk++;
			len--;

			if ((chunk == 4) || ((len == 0) && (blksize == 0))) {
				sdhci_writel(host, scratch, SDHCI_BUFFER);
				chunk = 0;
				scratch = 0;
			}
		}
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void sdhci_transfer_pio(struct sdhci_host *host)
{
	u32 mask;

	BUG_ON(!host->data);

	if (host->blocks == 0)
		return;

	if (host->data->flags & MMC_DATA_READ)
		mask = SDHCI_DATA_AVAILABLE;
	else
		mask = SDHCI_SPACE_AVAILABLE;

	/*
	 * Some controllers (JMicron JMB38x) mess up the buffer bits
	 * for transfers < 4 bytes. As long as it is just one block,
	 * we can ignore the bits.
	 */
	if ((host->quirks & SDHCI_QUIRK_BROKEN_SMALL_PIO) &&
		(host->data->blocks == 1))
		mask = ~0;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (host->quirks & SDHCI_QUIRK_PIO_NEEDS_DELAY)
			udelay(100);

		if (host->data->flags & MMC_DATA_READ)
			sdhci_read_block_pio(host);
		else
			sdhci_write_block_pio(host);

		host->blocks--;
		if (host->blocks == 0)
			break;
	}

	DBG("PIO transfer complete.\n");
}

static char *sdhci_kmap_atomic(struct scatterlist *sg, unsigned long *flags)
{
	local_irq_save(*flags);
	return kmap_atomic(sg_page(sg)) + sg->offset;
}

static void sdhci_kunmap_atomic(void *buffer, unsigned long *flags)
{
	kunmap_atomic(buffer);
	local_irq_restore(*flags);
}

static void sdhci_set_adma_desc(u8 *desc, u32 addr, int len, unsigned cmd)
{
	__le32 *dataddr = (__le32 __force *)(desc + 4);
	__le16 *cmdlen = (__le16 __force *)desc;

	/* SDHCI specification says ADMA descriptors should be 4 byte
	 * aligned, so using 16 or 32bit operations should be safe. */

	cmdlen[0] = cpu_to_le16(cmd);
	cmdlen[1] = cpu_to_le16(len);

	dataddr[0] = cpu_to_le32(addr);
}

static int sdhci_adma_table_pre(struct sdhci_host *host,
	struct mmc_data *data)
{
	int direction;

	u8 *desc;
	u8 *align;
	dma_addr_t addr;
	dma_addr_t align_addr;
	int len, offset;

	struct scatterlist *sg;
	int i;
	char *buffer;
	unsigned long flags;

	/*
	 * The spec does not specify endianness of descriptor table.
	 * We currently guess that it is LE.
	 */

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	/*
	 * The ADMA descriptor table is mapped further down as we
	 * need to fill it with data first.
	 */

	host->align_addr = dma_map_single(mmc_dev(host->mmc),
		host->align_buffer, 128 * 4, direction);
	if (dma_mapping_error(mmc_dev(host->mmc), host->align_addr))
		goto fail;
	BUG_ON(host->align_addr & 0x3);

	host->sg_count = dma_map_sg(mmc_dev(host->mmc),
		data->sg, data->sg_len, direction);
	if (host->sg_count == 0)
		goto unmap_align;

	desc = host->adma_desc;
	align = host->align_buffer;

	align_addr = host->align_addr;

	for_each_sg(data->sg, sg, host->sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		/*
		 * The SDHCI specification states that ADMA
		 * addresses must be 32-bit aligned. If they
		 * aren't, then we use a bounce buffer for
		 * the (up to three) bytes that screw up the
		 * alignment.
		 */
		offset = (4 - (addr & 0x3)) & 0x3;
		if (offset) {
			if (data->flags & MMC_DATA_WRITE) {
				buffer = sdhci_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(align, buffer, offset);
				sdhci_kunmap_atomic(buffer, &flags);
			}

			/* tran, valid */
			sdhci_set_adma_desc(desc, align_addr, offset, 0x21);

			BUG_ON(offset > 65536);

			align += 4;
			align_addr += 4;

			desc += 8;

			addr += offset;
			len -= offset;
		}

		BUG_ON(len > 65536);

		/* tran, valid */
		sdhci_set_adma_desc(desc, addr, len, 0x21);
		desc += 8;

		/*
		 * If this triggers then we have a calculation bug
		 * somewhere. :/
		 */
		WARN_ON((desc - host->adma_desc) > (128 * 2 + 1) * 4);
	}

	if (host->quirks & SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC) {
		/*
		* Mark the last descriptor as the terminating descriptor
		*/
		if (desc != host->adma_desc) {
			desc -= 8;
			desc[0] |= 0x2; /* end */
		}
	} else {
		/*
		* Add a terminating entry.
		*/

		/* nop, end, valid */
		sdhci_set_adma_desc(desc, 0, 0, 0x3);
	}

	/*
	 * Resync align buffer as we might have changed it.
	 */
	if (data->flags & MMC_DATA_WRITE) {
		dma_sync_single_for_device(mmc_dev(host->mmc),
			host->align_addr, 128 * 4, direction);
	}

	host->adma_addr = dma_map_single(mmc_dev(host->mmc),
		host->adma_desc, (128 * 2 + 1) * 4, DMA_TO_DEVICE);
	if (dma_mapping_error(mmc_dev(host->mmc), host->adma_addr))
		goto unmap_entries;
	BUG_ON(host->adma_addr & 0x3);

	return 0;

unmap_entries:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);
unmap_align:
	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
		128 * 4, direction);
fail:
	return -EINVAL;
}

static void sdhci_adma_table_post(struct sdhci_host *host,
	struct mmc_data *data)
{
	int direction;

	struct scatterlist *sg;
	int i, size;
	u8 *align;
	char *buffer;
	unsigned long flags;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	dma_unmap_single(mmc_dev(host->mmc), host->adma_addr,
		(128 * 2 + 1) * 4, DMA_TO_DEVICE);

	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
		128 * 4, direction);

	if (data->flags & MMC_DATA_READ) {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc), data->sg,
			data->sg_len, direction);

		align = host->align_buffer;

		for_each_sg(data->sg, sg, host->sg_count, i) {
			if (sg_dma_address(sg) & 0x3) {
				size = 4 - (sg_dma_address(sg) & 0x3);

				buffer = sdhci_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(buffer, align, size);
				sdhci_kunmap_atomic(buffer, &flags);

				align += 4;
			}
		}
	}

	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);
}

static u8 sdhci_calc_timeout(struct sdhci_host *host, struct mmc_command *cmd)
{
	u8 count;
	struct mmc_data *data = cmd->data;
	unsigned target_timeout, current_timeout;

	/*
	 * If the host controller provides us with an incorrect timeout
	 * value, just skip the check and use 0xE.  The hardware may take
	 * longer to time out, but that's much better than having a too-short
	 * timeout value.
	 */
	if (host->quirks & SDHCI_QUIRK_BROKEN_TIMEOUT_VAL)
		return 0xE;

	/* Unspecified timeout, assume max */
	if (!data && !cmd->cmd_timeout_ms)
		return 0xE;

	/* timeout in us */
	if (!data)
		target_timeout = cmd->cmd_timeout_ms * 1000;
	else {
		target_timeout = data->timeout_ns / 1000;
		if (host->clock)
			target_timeout += data->timeout_clks / host->clock;
	}

	/*
	 * Figure out needed cycles.
	 * We do this in steps in order to fit inside a 32 bit int.
	 * The first step is the minimum timeout, which will have a
	 * minimum resolution of 6 bits:
	 * (1) 2^13*1000 > 2^22,
	 * (2) host->timeout_clk < 2^16
	 *     =>
	 *     (1) / (2) > 2^6
	 */
	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < target_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF)
		count = 0xE;

	return count;
}

static void sdhci_set_transfer_irqs(struct sdhci_host *host)
{
	u32 pio_irqs = SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL;
	u32 dma_irqs = SDHCI_INT_DMA_END | SDHCI_INT_ADMA_ERROR;

	if (host->flags & SDHCI_REQ_USE_DMA)
		sdhci_clear_set_irqs(host, pio_irqs, dma_irqs);
	else
		sdhci_clear_set_irqs(host, dma_irqs, pio_irqs);
}

static void sdhci_prepare_data(struct sdhci_host *host, struct mmc_command *cmd)
{
	u8 count;
	u8 ctrl;
	struct mmc_data *data = cmd->data;
	int ret;

	WARN_ON(host->data);

	if (data || (cmd->flags & MMC_RSP_BUSY)) {
		count = sdhci_calc_timeout(host, cmd);
		sdhci_writeb(host, count, SDHCI_TIMEOUT_CONTROL);
	}

	if (!data)
		return;

	/* Sanity checks */
	BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > 65535);

	host->data = data;
	host->data_early = 0;
	host->data->bytes_xfered = 0;

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))
		host->flags |= SDHCI_REQ_USE_DMA;

	/*
	 * FIXME: This doesn't account for merging when mapping the
	 * scatterlist.
	 */
	if (host->flags & SDHCI_REQ_USE_DMA) {
		int broken, i;
		struct scatterlist *sg;

		broken = 0;
		if (host->flags & SDHCI_USE_ADMA) {
			if (host->quirks & SDHCI_QUIRK_32BIT_ADMA_SIZE)
				broken = 1;
		} else {
			if (host->quirks & SDHCI_QUIRK_32BIT_DMA_SIZE)
				broken = 1;
		}

		if (unlikely(broken)) {
			for_each_sg(data->sg, sg, data->sg_len, i) {
				if (sg->length & 0x3) {
					DBG("Reverting to PIO because of "
						"transfer size (%d)\n",
						sg->length);
					host->flags &= ~SDHCI_REQ_USE_DMA;
					break;
				}
			}
		}
	}

	/*
	 * The assumption here being that alignment is the same after
	 * translation to device address space.
	 */
	if (host->flags & SDHCI_REQ_USE_DMA) {
		int broken, i;
		struct scatterlist *sg;

		broken = 0;
		if (host->flags & SDHCI_USE_ADMA) {
			/*
			 * As we use 3 byte chunks to work around
			 * alignment problems, we need to check this
			 * quirk.
			 */
			if (host->quirks & SDHCI_QUIRK_32BIT_ADMA_SIZE)
				broken = 1;
		} else {
			if (host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR)
				broken = 1;
		}

		if (unlikely(broken)) {
			for_each_sg(data->sg, sg, data->sg_len, i) {
				if (sg->offset & 0x3) {
					DBG("Reverting to PIO because of "
						"bad alignment\n");
					host->flags &= ~SDHCI_REQ_USE_DMA;
					break;
				}
			}
		}
	}

	if (host->flags & SDHCI_REQ_USE_DMA) {
		if (host->flags & SDHCI_USE_ADMA) {
			ret = sdhci_adma_table_pre(host, data);
			if (ret) {
				/*
				 * This only happens when someone fed
				 * us an invalid request.
				 */
				WARN_ON(1);
				host->flags &= ~SDHCI_REQ_USE_DMA;
			} else {
				sdhci_writel(host, host->adma_addr,
					SDHCI_ADMA_ADDRESS);
			}
		} else {
			int sg_cnt;

			sg_cnt = dma_map_sg(mmc_dev(host->mmc),
					data->sg, data->sg_len,
					(data->flags & MMC_DATA_READ) ?
						DMA_FROM_DEVICE :
						DMA_TO_DEVICE);
			if (sg_cnt == 0) {
				/*
				 * This only happens when someone fed
				 * us an invalid request.
				 */
				WARN_ON(1);
				host->flags &= ~SDHCI_REQ_USE_DMA;
			} else {
				WARN_ON(sg_cnt != 1);
				sdhci_writel(host, sg_dma_address(data->sg),
					SDHCI_DMA_ADDRESS);
			}
		}
	}

	/*
	 * Always adjust the DMA selection as some controllers
	 * (e.g. JMicron) can't do PIO properly when the selection
	 * is ADMA.
	 */
	if (host->version >= SDHCI_SPEC_200) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl &= ~SDHCI_CTRL_DMA_MASK;
		if ((host->flags & SDHCI_REQ_USE_DMA) &&
			(host->flags & SDHCI_USE_ADMA))
			ctrl |= SDHCI_CTRL_ADMA32;
		else
			ctrl |= SDHCI_CTRL_SDMA;
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	if (!(host->flags & SDHCI_REQ_USE_DMA)) {
		int flags;

		flags = SG_MITER_ATOMIC;
		if (host->data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;
		sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
		host->blocks = data->blocks;
	}

	sdhci_set_transfer_irqs(host);

	/* Set the DMA boundary value and block size */
	sdhci_writew(host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG,
		data->blksz), SDHCI_BLOCK_SIZE);
	sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
}

static void sdhci_set_transfer_mode(struct sdhci_host *host,
	struct mmc_command *cmd)
{
	u16 mode;
	struct mmc_data *data = cmd->data;

	if (data == NULL)
		return;

	WARN_ON(!host->data);

	mode = SDHCI_TRNS_BLK_CNT_EN;
	if (mmc_op_multi(cmd->opcode) || data->blocks > 1) {
		mode |= SDHCI_TRNS_MULTI;
		/*
		 * If we are sending CMD23, CMD12 never gets sent
		 * on successful completion (so no Auto-CMD12).
		 */
		if (!host->mrq->sbc && (host->flags & SDHCI_AUTO_CMD12))
			mode |= SDHCI_TRNS_AUTO_CMD12;
		else if (host->mrq->sbc && (host->flags & SDHCI_AUTO_CMD23)) {
			mode |= SDHCI_TRNS_AUTO_CMD23;
			sdhci_writel(host, host->mrq->sbc->arg, SDHCI_ARGUMENT2);
		}
	}

	if (data->flags & MMC_DATA_READ)
		mode |= SDHCI_TRNS_READ;
	if (host->flags & SDHCI_REQ_USE_DMA)
		mode |= SDHCI_TRNS_DMA;

	sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
}

static void sdhci_finish_data(struct sdhci_host *host)
{
	struct mmc_data *data;

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	if (host->flags & SDHCI_REQ_USE_DMA) {
		if (host->flags & SDHCI_USE_ADMA)
			sdhci_adma_table_post(host, data);
		else {
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, (data->flags & MMC_DATA_READ) ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE);
		}
	}

	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	/*
	 * Need to send CMD12 if -
	 * a) open-ended multiblock transfer (no CMD23)
	 * b) error in multiblock transfer
	 */
	if (data->stop &&
	    (data->error ||
	     !host->mrq->sbc)) {

		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (data->error) {
			if (host->quirks2 & SDHCI_QUIRK2_WAIT_FOR_IDLE)
				sdhci_busy_wait(host, 1000);
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);
		}

		sdhci_send_command(host, data->stop);
	} else
		tasklet_schedule(&host->finish_tasklet);
}

static void sdhci_send_command(struct sdhci_host *host, struct mmc_command *cmd)
{
	int flags;
	u32 mask;
	unsigned long timeout;

	WARN_ON(host->cmd);

	/* Wait max 10 ms */
	timeout = 1000;

	mask = SDHCI_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (host->mrq->data && (cmd == host->mrq->data->stop))
		mask &= ~SDHCI_DATA_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			pr_err("%s: Controller never released "
				"inhibit bit(s).\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			cmd->error = -EIO;
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
		timeout--;
		udelay(10);
	}

	mod_timer(&host->timer, jiffies + 10 * HZ);

	host->cmd = cmd;

	sdhci_prepare_data(host, cmd);

	sdhci_writel(host, cmd->arg, SDHCI_ARGUMENT);

	sdhci_set_transfer_mode(host, cmd);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		pr_err("%s: Unsupported response type!\n",
			mmc_hostname(host->mmc));
		cmd->error = -EINVAL;
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->flags & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
	else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->flags & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;

	/* CMD19 is special in that the Data Present Select should be set */
	if (cmd->data || cmd->opcode == MMC_SEND_TUNING_BLOCK ||
	    cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200)
		flags |= SDHCI_CMD_DATA;

	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->opcode, flags), SDHCI_COMMAND);
}

static void sdhci_finish_command(struct sdhci_host *host)
{
	int i;

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0;i < 4;i++) {
				host->cmd->resp[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
				if (i != 3)
					host->cmd->resp[i] |=
						sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
			}
		} else {
			host->cmd->resp[0] = sdhci_readl(host, SDHCI_RESPONSE);
		}
	}

	host->cmd->error = 0;

	/* Finished CMD23, now send actual command. */
	if (host->cmd == host->mrq->sbc) {
		host->cmd = NULL;
		sdhci_send_command(host, host->mrq->cmd);
	} else {

		/* Processed actual command. */
		if (host->data && host->data_early)
			sdhci_finish_data(host);

		if (!host->cmd->data)
			tasklet_schedule(&host->finish_tasklet);

		host->cmd = NULL;
	}
}

static void sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int div = 0; /* Initialized for compiler warning */
	int real_div = div, clk_mul = 1;
	u16 clk = 0;
	unsigned long timeout;

	if (clock && clock == host->clock)
		return;

	host->mmc->actual_clock = 0;

	if (host->ops->set_clock) {
		host->ops->set_clock(host, clock);
		if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK)
			return;
	}

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

	if (host->version >= SDHCI_SPEC_300) {
		/*
		 * Check if the Host Controller supports Programmable Clock
		 * Mode.
		 */
		if (host->clk_mul) {
			u16 ctrl;

			/*
			 * We need to figure out whether the Host Driver needs
			 * to select Programmable Clock Mode, or the value can
			 * be set automatically by the Host Controller based on
			 * the Preset Value registers.
			 */
			ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
			if (!(ctrl & SDHCI_CTRL_PRESET_VAL_ENABLE)) {
				for (div = 1; div <= 1024; div++) {
					if (((host->max_clk * host->clk_mul) /
					      div) <= clock)
						break;
				}
				/*
				 * Set Programmable Clock Mode in the Clock
				 * Control register.
				 */
				clk = SDHCI_PROG_CLOCK_MODE;
				real_div = div;
				clk_mul = host->clk_mul;
				div--;
			}
		} else {
			/* Version 3.00 divisors must be a multiple of 2. */
			if (host->max_clk <= clock)
				div = 1;
			else {
				for (div = 2; div < SDHCI_MAX_DIV_SPEC_300;
				     div += 2) {
					if ((host->max_clk / div) <= clock)
						break;
				}
			}
			real_div = div;
			div >>= 1;
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((host->max_clk / div) <= clock)
				break;
		}
		real_div = div;
		div >>= 1;
	}

	if (real_div)
		host->mmc->actual_clock = (host->max_clk * clk_mul) / real_div;

	clk |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 2000;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never "
				"stabilised.\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		udelay(10);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;
}

static int sdhci_set_power(struct sdhci_host *host, unsigned short power)
{
	u8 pwr = 0;

	if (power != (unsigned short)-1) {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
			break;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
		default:
			BUG();
		}
	}

	if (host->pwr == pwr)
		return -1;

	host->pwr = pwr;

	if (pwr == 0) {
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		/* disable the power by setting GPIO pin */
		if (host->quirks2 & SDHCI_QUIRK2_POWER_PIN_GPIO_MODE)
			gpio_set_value(host->gpio_pwr_en, 1);
		return 0;
	}

	/*
	 * Spec says that we should clear the power reg before setting
	 * a new value. Some controllers don't seem to like this though.
	 */
	if (!(host->quirks & SDHCI_QUIRK_SINGLE_POWER_WRITE))
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);

	/*
	 * At least the Marvell CaFe chip gets confused if we set the voltage
	 * and set turn on power at the same time, so set the voltage first.
	 */
	if (host->quirks & SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER)
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	pwr |= SDHCI_POWER_ON;

	sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	/* enable the power by setting GPIO pin */
	if (host->quirks2 & SDHCI_QUIRK2_POWER_PIN_GPIO_MODE)
		gpio_set_value(host->gpio_pwr_en, 0);

	/*
	 * Some controllers need an extra 10ms delay of 10ms before they
	 * can apply clock after applying power
	 */
	if (host->quirks & SDHCI_QUIRK_DELAY_AFTER_POWER)
		mdelay(10);

	return power;
}

/*
 * One of the Medfield eMMC controller (PCI device id 0x0823, SDIO3) is
 * a shared resource used by the SCU and the IA processors. SCU primarily
 * uses the eMMC host controller to access the eMMC device's Boot Partition,
 * while the IA CPU uses the eMMC host controller to access the eMMC device's
 * User Partition.
 *
 * After the SCU hands off the system to the IA processor, the IA processor
 * assumes ownership to the eMMC host controller. Due to absence of any
 * arbitration at the eMMC host controller, this could result in concurrent
 * eMMC host accesses resulting in bus contention and garbage data ending up
 * in either of the partitions.
 * To circumvent this from happening, eMMC host controller locking mechanism
 * is employed, where at any one given time, only one agent, SCU or IA, may be
 * allowed to access the host. This is achieved by implementing Dekker's
 * Algorithm (http://en.wikipedia.org/wiki/Dekker's_algorithm) between the
 * two processors.
 *
 * Before handing off the system to the IA processor, SCU must set up three
 * housekeeping mutex variables allocated in the shared SRAM as follows:
 *
 * eMMC_Owner = IA (SCU and IA processors - RW, 32bit)
 * IA_Req = FALSE (IA -RW, SCU - RO, 32bit)
 * SCU_Req = FALSE (IA - RO, SCU - R/W, 32bit)
 *
 * There is no hardware based access control to these variables and so code
 * executing on SCU and IA processors must follow below access rules
 * (Dekker's algorithm):
 *
 * -----------------------------------------
 * SCU Processor Implementation
 * -----------------------------------------
 * SCU_Req = TRUE;
 * while (IA_Req == TRUE) {
 *     if (eMMC_Owner != SCU){
 *         SCU_Req = FALSE;
 *         while (eMMC_Owner != SCU);
 *         SCU_Req = TRUE;
 *     }
 * }
 * // SCU now performs eMMC transactions here
 * ...
 * // When done, relinquish control to IA
 * eMMC_Owner = IA;
 * SCU_Req = FALSE;
 *
 * -----------------------------------------
 * IA Processor Implementation
 * -----------------------------------------
 * IA_Req = TRUE;
 * while (SCU_Req == TRUE) {
 *     if (eMMC_Owner != IA){
 *         IA_Req = FALSE;
 *         while (eMMC_Owner != IA);
 *         IA_Req = TRUE;
 *     }
 * }
 * //IA now performs eMMC transactions here
 * ...
 * //When done, relinquish control to SCU
 * eMMC_Owner = SCU;
 * IA_Req = FALSE;
 *
 * ----------------------------------------
 *
 * sdhci_do_acquire_ownership- implement the Dekker's algorithm on IA side
 * This function is only used for acquire ownership, not to re-cofnig host
 * controller. Since in some scenarios, re-config is not useless. We can
 * save some unused expenses.
 * @mmc: mmc host
 *
 * @return return value:
 * 0 - Acquried the ownership successfully. The last owner is IA
 * 1 - Acquried the ownership succesffully. The last owenr is SCU
 * -EBUSY - failed to acquire ownership within the timeout period
 */
static int sdhci_do_acquire_ownership(struct mmc_host *mmc)
{
	struct sdhci_host *host;
	unsigned long t1, t2;
	unsigned long flags;

	host = mmc_priv(mmc);

	if (!host->sram_addr)
		return 0;

	/* if host has sram_addr, dekker_lock is initialized */
	spin_lock_irqsave(&host->dekker_lock, flags);

	host->usage_cnt++;

	/* If IA has already hold the eMMC mutex, then just exit */
	if (readl(host->sram_addr + DEKKER_IA_REQ_OFFSET)) {
		spin_unlock_irqrestore(&host->dekker_lock, flags);
		return 0;
	}

	DBG("Acquire ownership - eMMC owner: %d, IA req: %d, SCU req: %d\n",
		readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET),
		readl(host->sram_addr + DEKKER_IA_REQ_OFFSET),
		readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET));

	writel(1, host->sram_addr + DEKKER_IA_REQ_OFFSET);

	t1 = jiffies + 10 * HZ;
	t2 = 500;

	while (readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET)) {
		if (readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET) !=
				DEKKER_OWNER_IA) {
			writel(0, host->sram_addr + DEKKER_IA_REQ_OFFSET);
			while (t2) {
				if (readl(host->sram_addr +
					DEKKER_EMMC_OWNER_OFFSET) ==
					DEKKER_OWNER_IA)
					break;
				spin_unlock_irqrestore(&host->dekker_lock,
						flags);
				usleep_range(8000, 12000);
				spin_lock_irqsave(&host->dekker_lock, flags);
				t2--;
			}
			if (t2)
				writel(1, host->sram_addr +
						DEKKER_IA_REQ_OFFSET);
			else
				goto timeout;
		}
		if (time_after(jiffies, t1))
			goto timeout;

		cpu_relax();
	}

	spin_unlock_irqrestore(&host->dekker_lock, flags);
	/*
	 * if the last owner is SCU, will do the re-config host controller
	 * in the next
	 */
	return (readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET) ==
		DEKKER_OWNER_IA) ? 1 : 0;

timeout:
	printk(KERN_ERR "eMMC mutex timeout!\n"
		"Dump Dekker's house keeping variables -"
		"eMMC owner: %d, IA req: %d, SCU req: %d\n",
		readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET),
		readl(host->sram_addr + DEKKER_IA_REQ_OFFSET),
		readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET));

	/* Release eMMC mutex anyway */
	writel(DEKKER_OWNER_SCU, host->sram_addr + DEKKER_EMMC_OWNER_OFFSET);
	writel(0, host->sram_addr + DEKKER_IA_REQ_OFFSET);

	spin_unlock_irqrestore(&host->dekker_lock, flags);

	return -EBUSY;
}

static int sdhci_acquire_ownership(struct mmc_host *mmc)
{
	int ret;

	ret = sdhci_do_acquire_ownership(mmc);
	if (ret) {
		struct sdhci_host *host;
		host = mmc_priv(mmc);
		/* Re-config HC in case SCU has changed HC reg already */
		pm_runtime_get_sync(mmc->parent);
		/*
		 * reinit host registers.
		 * include reset host controller all,
		 * reconfigure clock, pwr and other registers.
		 */
		sdhci_init(host, 0);
		host->clock = 0;
		host->pwr = 0;
		sdhci_do_set_ios(host, &host->mmc->ios);
		sdhci_do_start_signal_voltage_switch(host,
					&host->mmc->ios, false);
		pm_runtime_put(mmc->parent);
	}

	return ret;
}

static void sdhci_release_ownership(struct mmc_host *mmc)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = mmc_priv(mmc);

	if (!host->sram_addr)
		return;

	spin_lock_irqsave(&host->dekker_lock, flags);
	BUG_ON(host->usage_cnt == 0);
	host->usage_cnt--;
	if (host->usage_cnt == 0) {
		writel(DEKKER_OWNER_SCU,
		       host->sram_addr + DEKKER_EMMC_OWNER_OFFSET);
		writel(0, host->sram_addr + DEKKER_IA_REQ_OFFSET);
		DBG("Exit ownership - "
		    "eMMC owner: %d, IA req: %d, SCU req: %d\n",
		    readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET),
		    readl(host->sram_addr + DEKKER_IA_REQ_OFFSET),
		    readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET));
	}
	spin_unlock_irqrestore(&host->dekker_lock, flags);
}

/*****************************************************************************\
 *                                                                           *
 * MMC callbacks                                                             *
 *                                                                           *
\*****************************************************************************/

static void sdhci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdhci_host *host;
	bool present;
	unsigned long flags;
	u32 tuning_opcode;

	host = mmc_priv(mmc);

	sdhci_runtime_pm_get(host);

	sdhci_acquire_ownership(host->mmc);

	spin_lock_irqsave(&host->lock, flags);

	if (host->suspended) {
		pr_err("%s: %s: host is in suspend state\n",
				__func__, mmc_hostname(mmc));
		BUG_ON(1);
	}

	WARN_ON(host->mrq != NULL);

#ifndef SDHCI_USE_LEDS_CLASS
	sdhci_activate_led(host);
#endif

	/*
	 * Ensure we don't send the STOP for non-SET_BLOCK_COUNTED
	 * requests if Auto-CMD12 is enabled.
	 */
	if (!mrq->sbc && (host->flags & SDHCI_AUTO_CMD12)) {
		if (mrq->stop) {
			mrq->data->stop = NULL;
			mrq->stop = NULL;
		}
	}

	host->mrq = mrq;

	/* If polling, assume that the card is always present. */
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		present = true;
	else if ((host->quirks2 & SDHCI_QUIRK2_BAD_SD_CD) &&
			host->ops->get_cd)
		present = host->ops->get_cd(host);
	else
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;

	if (!present || host->flags & SDHCI_DEVICE_DEAD) {
		host->mrq->cmd->error = -ENOMEDIUM;
		tasklet_schedule(&host->finish_tasklet);
	} else {
		u32 present_state;

		present_state = sdhci_readl(host, SDHCI_PRESENT_STATE);
		/*
		 * Check if the re-tuning timer has already expired and there
		 * is no on-going data transfer. If so, we need to execute
		 * tuning procedure before sending command.
		 */
		if ((host->flags & SDHCI_NEEDS_RETUNING) &&
		    !(present_state & (SDHCI_DOING_WRITE | SDHCI_DOING_READ)) &&
			mrq->cmd->opcode != MMC_SEND_STATUS) {
			if (mmc->card) {
				if (mmc_card_mmc(mmc->card) &&
				    (mmc->card->ext_csd.part_config & 0x07) ==
					EXT_CSD_PART_CONFIG_RPMB)
						goto end_tuning;
				/* eMMC uses cmd21 but sd and sdio use cmd19 */
				tuning_opcode =
					mmc->card->type == MMC_TYPE_MMC ?
					MMC_SEND_TUNING_BLOCK_HS200 :
					MMC_SEND_TUNING_BLOCK;
				spin_unlock_irqrestore(&host->lock, flags);
				sdhci_execute_tuning(mmc, tuning_opcode);
				spin_lock_irqsave(&host->lock, flags);
end_tuning:
				/* Restore original mmc_request structure */
				host->mrq = mrq;
			}
		}

		if (!(sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
					SDHCI_CLOCK_CARD_EN)) {
			/*
			 * SD bus clock is stopped. no interrupts will be
			 * generate in this case. Let's try to enable this
			 * bit first
			 */
			pr_warn("%s:%s: SD bus clock not enabled enabling it\n",
					__func__, mmc_hostname(mmc));
			pr_warn("%s:%s: host->pwr 0x%x, host->clock 0x%x\n",
					__func__, mmc_hostname(mmc),
					host->pwr, host->clock);
			host->clock = 0;
			sdhci_set_clock(host, mmc->ios.clock);
			/*
			 * check whether SD bus clock is enabled
			 */
			if (!(sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
						SDHCI_CLOCK_CARD_EN)) {
				pr_warn("%s:%s: SD bus clock cannot be enabled\n",
						__func__, mmc_hostname(mmc));
				sdhci_dumpregs(host);
				host->mrq->cmd->error = -ETIMEDOUT;
				tasklet_schedule(&host->finish_tasklet);
				goto out;
			}
		}

		if (mrq->sbc && !(host->flags & SDHCI_AUTO_CMD23))
			sdhci_send_command(host, mrq->sbc);
		else
			sdhci_send_command(host, mrq->cmd);
	}

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_do_set_ios(struct sdhci_host *host, struct mmc_ios *ios)
{
	unsigned long flags;
	int vdd_bit = -1;
	u8 ctrl;

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD) {
		spin_unlock_irqrestore(&host->lock, flags);
		if (host->vmmc && ios->power_mode == MMC_POWER_OFF)
			mmc_regulator_set_ocr(host->mmc, host->vmmc, 0);
		return;
	}

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (host->quirks2 & SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8)
		host->mmc->ios.vdd = 7;

	if (ios->power_mode == MMC_POWER_OFF) {
		sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);
		sdhci_reinit(host);
	}

	sdhci_set_clock(host, ios->clock);

	if (ios->power_mode == MMC_POWER_OFF)
		vdd_bit = sdhci_set_power(host, -1);
	else
		vdd_bit = sdhci_set_power(host, ios->vdd);

	if (host->vmmc && vdd_bit != -1) {
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_regulator_set_ocr(host->mmc, host->vmmc, vdd_bit);
		spin_lock_irqsave(&host->lock, flags);
	}

	if (host->ops->platform_send_init_74_clocks)
		host->ops->platform_send_init_74_clocks(host, ios->power_mode);

	/*
	 * If your platform has 8-bit width support but is not a v3 controller,
	 * or if it requires special setup code, you should implement that in
	 * platform_8bit_width().
	 */
	if (host->ops->platform_8bit_width)
		host->ops->platform_8bit_width(host, ios->bus_width);
	else {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		if (ios->bus_width == MMC_BUS_WIDTH_8) {
			ctrl &= ~SDHCI_CTRL_4BITBUS;
			if (host->version >= SDHCI_SPEC_300)
				ctrl |= SDHCI_CTRL_8BITBUS;
		} else {
			if (host->version >= SDHCI_SPEC_300)
				ctrl &= ~SDHCI_CTRL_8BITBUS;
			if (ios->bus_width == MMC_BUS_WIDTH_4)
				ctrl |= SDHCI_CTRL_4BITBUS;
			else
				ctrl &= ~SDHCI_CTRL_4BITBUS;
		}
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if ((ios->timing == MMC_TIMING_SD_HS ||
	     ios->timing == MMC_TIMING_MMC_HS)
	    && !(host->quirks & SDHCI_QUIRK_NO_HISPD_BIT))
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	if ((host->version >= SDHCI_SPEC_300) ||
			(host->quirks2 & SDHCI_QUIRK2_V2_0_SUPPORT_DDR50)) {
		u16 clk, ctrl_2;
		unsigned int clock;

		/* In case of UHS-I modes, set High Speed Enable */
		if ((ios->timing == MMC_TIMING_MMC_HS200) ||
		    (ios->timing == MMC_TIMING_UHS_SDR50) ||
		    (ios->timing == MMC_TIMING_UHS_SDR104) ||
		    (ios->timing == MMC_TIMING_UHS_DDR50) ||
		    (ios->timing == MMC_TIMING_UHS_SDR25))
			ctrl |= SDHCI_CTRL_HISPD;

		ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (!(ctrl_2 & SDHCI_CTRL_PRESET_VAL_ENABLE)) {
			sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
			/*
			 * We only need to set Driver Strength if the
			 * preset value enable is not set.
			 */
			ctrl_2 &= ~SDHCI_CTRL_DRV_TYPE_MASK;
			if (ios->drv_type == MMC_SET_DRIVER_TYPE_A)
				ctrl_2 |= SDHCI_CTRL_DRV_TYPE_A;
			else if (ios->drv_type == MMC_SET_DRIVER_TYPE_C)
				ctrl_2 |= SDHCI_CTRL_DRV_TYPE_C;

			sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
		} else {
			/*
			 * According to SDHC Spec v3.00, if the Preset Value
			 * Enable in the Host Control 2 register is set, we
			 * need to reset SD Clock Enable before changing High
			 * Speed Enable to avoid generating clock gliches.
			 */

			/* Reset SD Clock Enable */
			clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
			clk &= ~SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

			sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

			/* Re-enable SD Clock */
			clock = host->clock;
			host->clock = 0;
			sdhci_set_clock(host, clock);
		}


		/* Reset SD Clock Enable */
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~SDHCI_CLOCK_CARD_EN;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

		if (host->ops->set_uhs_signaling)
			host->ops->set_uhs_signaling(host, ios->timing);
		else {
			ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
			/* Select Bus Speed Mode for host */
			ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
			if (ios->timing == MMC_TIMING_MMC_HS200)
				ctrl_2 |= SDHCI_CTRL_HS_SDR200;
			else if (ios->timing == MMC_TIMING_UHS_SDR12)
				ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
			else if (ios->timing == MMC_TIMING_UHS_SDR25)
				ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
			else if (ios->timing == MMC_TIMING_UHS_SDR50)
				ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
			else if (ios->timing == MMC_TIMING_UHS_SDR104)
				ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
			else if (ios->timing == MMC_TIMING_UHS_DDR50)
				ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
			sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
		}

		/*
		 * Some buggy SDHC Host Controller requires the
		 * Host Control Register High Speed Enable bit
		 * must be set after Host Control 2 Register
		 * 1.8V Signaling Enable bit set.
		 * Otherwise it will fail to work on High Speed.
		 * So, here we just write the Host control Register
		 * with the same value again to workaround the issue.
		 */
		if (host->quirks2 & SDHCI_QUIRK2_HIGH_SPEED_SET_LATE)
			sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

		/* Re-enable SD Clock */
		clock = host->clock;
		host->clock = 0;
		sdhci_set_clock(host, clock);
	} else
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

	/*
	 * Some (ENE) controllers go apeshit on some ios operation,
	 * signalling timeout and CRC errors even on CMD0. Resetting
	 * it on each ios seems to solve the problem.
	 */
	if(host->quirks & SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS)
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

	if (host->quirks2 & SDHCI_QUIRK2_BAD_SD_CD) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl |= SDHCI_CTRL_CD_SD | SDHCI_CTRL_CD_TL;
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);

	sdhci_runtime_pm_get(host);
	sdhci_acquire_ownership(mmc);
	sdhci_do_set_ios(host, ios);
	sdhci_release_ownership(mmc);
	sdhci_runtime_pm_put(host);
}

static int sdhci_check_ro(struct sdhci_host *host)
{
	unsigned long flags;
	int is_readonly;

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD)
		is_readonly = 0;
	else if (host->ops->get_ro)
		is_readonly = host->ops->get_ro(host);
	else
		is_readonly = !(sdhci_readl(host, SDHCI_PRESENT_STATE)
				& SDHCI_WRITE_PROTECT);

	spin_unlock_irqrestore(&host->lock, flags);

	/* This quirk needs to be replaced by a callback-function later */
	return host->quirks & SDHCI_QUIRK_INVERTED_WRITE_PROTECT ?
		!is_readonly : is_readonly;
}

#define SAMPLE_COUNT	5

static int sdhci_do_get_ro(struct sdhci_host *host)
{
	int i, ro_count;

	if (!(host->quirks & SDHCI_QUIRK_UNSTABLE_RO_DETECT))
		return sdhci_check_ro(host);

	ro_count = 0;
	for (i = 0; i < SAMPLE_COUNT; i++) {
		if (sdhci_check_ro(host)) {
			if (++ro_count > SAMPLE_COUNT / 2)
				return 1;
		}
		msleep(30);
	}
	return 0;
}

static void sdhci_hw_reset(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);

	if (host->ops && host->ops->hw_reset) {
		sdhci_runtime_pm_get(host);
		sdhci_acquire_ownership(mmc);
		host->ops->hw_reset(host);
		sdhci_release_ownership(mmc);
		sdhci_runtime_pm_put(host);
	}
}

static int sdhci_get_ro(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int ret;

	sdhci_runtime_pm_get(host);
	ret = sdhci_do_get_ro(host);
	sdhci_runtime_pm_put(host);
	return ret;
}

static void sdhci_enable_sdio_irq_nolock(struct sdhci_host *host, int enable)
{
	if (host->flags & SDHCI_DEVICE_DEAD)
		goto out;

	if (enable)
		host->flags |= SDHCI_SDIO_IRQ_ENABLED;
	else
		host->flags &= ~SDHCI_SDIO_IRQ_ENABLED;

	/* SDIO IRQ will be enabled as appropriate in runtime resume */
	if (host->runtime_suspended)
		goto out;

	if (enable)
		sdhci_unmask_irqs(host, SDHCI_INT_CARD_INT);
	else
		sdhci_mask_irqs(host, SDHCI_INT_CARD_INT);
out:
	mmiowb();
}

static void sdhci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host = mmc_priv(mmc);
	unsigned long flags;

	sdhci_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);
	sdhci_enable_sdio_irq_nolock(host, enable);
	spin_unlock_irqrestore(&host->lock, flags);
	sdhci_runtime_pm_put(host);
}

static int sdhci_do_start_signal_voltage_switch(struct sdhci_host *host,
					struct mmc_ios *ios, bool cmd11)
{
	u8 pwr;
	u16 clk, ctrl;
	u32 present_state;

	/*
	 * Signal Voltage Switching is only applicable for Host Controllers
	 * v3.00 and above.
	 */
	if (host->version < SDHCI_SPEC_300)
		return 0;

	/*
	 * We first check whether the request is to set signalling voltage
	 * to 3.3V. If so, we change the voltage to 3.3V and return quickly.
	 */
	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	if ((ctrl & SDHCI_CTRL_VDD_180) &&
		(ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)) {
		/* Set 1.8V Signal Enable in the Host Control2 register to 0 */
		ctrl &= ~SDHCI_CTRL_VDD_180;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
		/* 3.3v by setting GPIO pin */
		if (host->quirks2 & SDHCI_QUIRK2_POWER_PIN_GPIO_MODE)
			gpio_set_value(host->gpio_1p8_en, 0);

		/* Wait for 5ms */
		usleep_range(5000, 5500);

		/* 3.3V regulator output should be stable within 5 ms */
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_VDD_180))
			return 0;
		else {
			pr_info(DRIVER_NAME ": Switching to 3.3V "
				"signalling voltage failed\n");
			return -EIO;
		}
	} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		if (!cmd11) {
			/* Set 1.8V Signal Enable in the Host Control2 Reg */
			ctrl |= SDHCI_CTRL_VDD_180;
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
			/* 1.8v by setting GPIO pin */
			if (host->quirks2 & SDHCI_QUIRK2_POWER_PIN_GPIO_MODE)
				gpio_set_value(host->gpio_1p8_en, 1);
			return 0;
		}

		/* Stop SDCLK */
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~SDHCI_CLOCK_CARD_EN;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

		/* Check whether DAT[3:0] is 0000 */
		present_state = sdhci_readl(host, SDHCI_PRESENT_STATE);
		if (!((present_state & SDHCI_DATA_LVL_MASK) >>
		       SDHCI_DATA_LVL_SHIFT)) {
			/*
			 * Enable 1.8V Signal Enable in the Host Control2
			 * register
			 */
			ctrl |= SDHCI_CTRL_VDD_180;
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
			/* 1.8v by setting GPIO pin */
			if (host->quirks2 & SDHCI_QUIRK2_POWER_PIN_GPIO_MODE)
				gpio_set_value(host->gpio_1p8_en, 1);

			/* Wait for 5ms */
			usleep_range(5000, 5500);

			ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
			if (ctrl & SDHCI_CTRL_VDD_180) {
				/* Provide SDCLK again and wait for 1ms*/
				clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
				clk |= SDHCI_CLOCK_CARD_EN;
				sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
				usleep_range(1000, 1500);

				/*
				 * If DAT[3:0] level is 1111b, then the card
				 * was successfully switched to 1.8V signaling.
				 */
				present_state = sdhci_readl(host,
							SDHCI_PRESENT_STATE);
				if ((present_state & SDHCI_DATA_LVL_MASK) ==
				     SDHCI_DATA_LVL_MASK)
					return 0;
			}
		}

		/*
		 * If we are here, that means the switch to 1.8V signaling
		 * failed. We power cycle the card, and retry initialization
		 * sequence by setting S18R to 0.
		 */
		pwr = sdhci_readb(host, SDHCI_POWER_CONTROL);
		pwr &= ~SDHCI_POWER_ON;
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

		/* Wait for 1ms as per the spec */
		usleep_range(1000, 1500);
		pwr |= SDHCI_POWER_ON;
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

		pr_info(DRIVER_NAME ": Switching to 1.8V signalling "
			"voltage failed, retrying with S18R set to 0\n");
		return -EAGAIN;
	} else
		/* No signal voltage switch required */
		return 0;
}

static int sdhci_start_signal_voltage_switch(struct mmc_host *mmc,
	struct mmc_ios *ios, bool cmd11)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int err;

	if (host->version < SDHCI_SPEC_300)
		return 0;
	sdhci_runtime_pm_get(host);
	err = sdhci_do_start_signal_voltage_switch(host, ios, cmd11);
	sdhci_runtime_pm_put(host);
	return err;
}

static int sdhci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host;
	u16 ctrl;
	u32 ier;
	int tuning_loop_counter = MAX_TUNING_LOOP;
	unsigned long timeout;
	int err = 0;
	bool requires_tuning_nonuhs = false;

	host = mmc_priv(mmc);

	sdhci_runtime_pm_get(host);
	disable_irq(host->irq);
	spin_lock(&host->lock);

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/*
	 * The Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode when Use Tuning for SDR50 is set in the
	 * Capabilities register.
	 * If the Host Controller supports the HS200 mode then the
	 * tuning function has to be executed.
	 */
	if ((((ctrl & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR50) &&
	    (host->flags & SDHCI_SDR50_NEEDS_TUNING)) ||
	     (host->flags & SDHCI_HS200_NEEDS_TUNING))
		requires_tuning_nonuhs = true;

	if (((ctrl & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR104) ||
	    requires_tuning_nonuhs)
		ctrl |= SDHCI_CTRL_EXEC_TUNING;
	else {
		spin_unlock(&host->lock);
		enable_irq(host->irq);
		sdhci_runtime_pm_put(host);
		return 0;
	}

	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

	/*
	 * As per the Host Controller spec v3.00, tuning command
	 * generates Buffer Read Ready interrupt, so enable that.
	 *
	 * Note: The spec clearly says that when tuning sequence
	 * is being performed, the controller does not generate
	 * interrupts other than Buffer Read Ready interrupt. But
	 * to make sure we don't hit a controller bug, we _only_
	 * enable Buffer Read Ready interrupt here.
	 */
	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	sdhci_clear_set_irqs(host, ier, SDHCI_INT_DATA_AVAIL);

	/*
	 * Issue CMD19 repeatedly till Execute Tuning is set to 0 or the number
	 * of loops reaches 40 times or a timeout of 150ms occurs.
	 */
	timeout = 150;
	do {
		struct mmc_command cmd = {0};
		struct mmc_request mrq = {NULL};

		if (!tuning_loop_counter && !timeout)
			break;

		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.retries = 0;
		cmd.data = NULL;
		cmd.error = 0;

		mrq.cmd = &cmd;
		host->mrq = &mrq;

		/*
		 * In response to CMD19, the card sends 64 bytes of tuning
		 * block to the Host Controller. So we set the block size
		 * to 64 here.
		 */
		if (cmd.opcode == MMC_SEND_TUNING_BLOCK_HS200) {
			if (mmc->ios.bus_width == MMC_BUS_WIDTH_8)
				sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, 128),
					     SDHCI_BLOCK_SIZE);
			else if (mmc->ios.bus_width == MMC_BUS_WIDTH_4)
				sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, 64),
					     SDHCI_BLOCK_SIZE);
		} else {
			sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, 64),
				     SDHCI_BLOCK_SIZE);
		}

		/*
		 * The tuning block is sent by the card to the host controller.
		 * So we set the TRNS_READ bit in the Transfer Mode register.
		 * This also takes care of setting DMA Enable and Multi Block
		 * Select in the same register to 0.
		 */
		sdhci_writew(host, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);

		sdhci_send_command(host, &cmd);

		host->cmd = NULL;
		host->mrq = NULL;

		spin_unlock(&host->lock);
		enable_irq(host->irq);

		/* Wait for Buffer Read Ready interrupt */
		wait_event_interruptible_timeout(host->buf_ready_int,
					(host->tuning_done == 1),
					msecs_to_jiffies(50));
		disable_irq(host->irq);
		spin_lock(&host->lock);

		if (!host->tuning_done) {
			pr_info(DRIVER_NAME ": Timeout waiting for "
				"Buffer Read Ready interrupt during tuning "
				"procedure, falling back to fixed sampling "
				"clock\n");
			ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
			ctrl &= ~SDHCI_CTRL_TUNED_CLK;
			ctrl &= ~SDHCI_CTRL_EXEC_TUNING;
			sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

			err = -EIO;
			goto out;
		}

		host->tuning_done = 0;

		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		tuning_loop_counter--;
		timeout--;
		mdelay(1);
	} while (ctrl & SDHCI_CTRL_EXEC_TUNING);

	/*
	 * The Host Driver has exhausted the maximum number of loops allowed,
	 * so use fixed sampling frequency.
	 */
	if (!tuning_loop_counter || !timeout) {
		ctrl &= ~SDHCI_CTRL_TUNED_CLK;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
	} else {
		if (!(ctrl & SDHCI_CTRL_TUNED_CLK)) {
			pr_info(DRIVER_NAME ": Tuning procedure"
				" failed, falling back to fixed sampling"
				" clock\n");
			err = -EIO;
		}
	}

out:
	/*
	 * If this is the very first time we are here, we start the retuning
	 * timer. Since only during the first time, SDHCI_NEEDS_RETUNING
	 * flag won't be set, we check this condition before actually starting
	 * the timer.
	 */
	if (!(host->flags & SDHCI_NEEDS_RETUNING) && host->tuning_count &&
	    (host->tuning_mode == SDHCI_TUNING_MODE_1)) {
		mod_timer(&host->tuning_timer, jiffies +
			host->tuning_count * HZ);
		/* Tuning mode 1 limits the maximum data length to 4MB */
		mmc->max_blk_count = (4 * 1024 * 1024) / mmc->max_blk_size;
	} else {
		host->flags &= ~SDHCI_NEEDS_RETUNING;
		/* Reload the new initial value for timer */
		if (host->tuning_mode == SDHCI_TUNING_MODE_1)
			mod_timer(&host->tuning_timer, jiffies +
				host->tuning_count * HZ);
	}

	/*
	 * In case tuning fails, host controllers which support re-tuning can
	 * try tuning again at a later time, when the re-tuning timer expires.
	 * So for these controllers, we return 0. Since there might be other
	 * controllers who do not have this capability, we return error for
	 * them.
	 */
	if (err && host->tuning_count &&
	    host->tuning_mode == SDHCI_TUNING_MODE_1)
		err = 0;

	sdhci_clear_set_irqs(host, SDHCI_INT_DATA_AVAIL, ier);
	spin_unlock(&host->lock);
	enable_irq(host->irq);
	sdhci_runtime_pm_put(host);

	return err;
}

static void sdhci_do_enable_preset_value(struct sdhci_host *host, bool enable)
{
	u16 ctrl;
	unsigned long flags;

	/* Host Controller v3.00 defines preset value registers */
	if (host->version < SDHCI_SPEC_300)
		return;

	spin_lock_irqsave(&host->lock, flags);

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/*
	 * We only enable or disable Preset Value if they are not already
	 * enabled or disabled respectively. Otherwise, we bail out.
	 */
	if (enable && !(ctrl & SDHCI_CTRL_PRESET_VAL_ENABLE)) {
		ctrl |= SDHCI_CTRL_PRESET_VAL_ENABLE;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
		host->flags |= SDHCI_PV_ENABLED;
	} else if (!enable && (ctrl & SDHCI_CTRL_PRESET_VAL_ENABLE)) {
		ctrl &= ~SDHCI_CTRL_PRESET_VAL_ENABLE;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
		host->flags &= ~SDHCI_PV_ENABLED;
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_enable_preset_value(struct mmc_host *mmc, bool enable)
{
	struct sdhci_host *host = mmc_priv(mmc);

	sdhci_runtime_pm_get(host);
	sdhci_do_enable_preset_value(host, enable);
	sdhci_runtime_pm_put(host);
}

static int sdhci_get_cd(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int present = -EOPNOTSUPP;

	if (host->ops->get_cd) {
		sdhci_runtime_pm_get(host);
		present = host->ops->get_cd(host);
		sdhci_runtime_pm_put(host);
	}

	return present;
}

static const struct mmc_host_ops sdhci_ops = {
	.request	= sdhci_request,
	.set_ios	= sdhci_set_ios,
	.get_ro		= sdhci_get_ro,
	.hw_reset	= sdhci_hw_reset,
	.enable_sdio_irq = sdhci_enable_sdio_irq,
	.start_signal_voltage_switch	= sdhci_start_signal_voltage_switch,
	.execute_tuning			= sdhci_execute_tuning,
	.enable_preset_value		= sdhci_enable_preset_value,
	.get_cd		= sdhci_get_cd,
};

/*****************************************************************************\
 *                                                                           *
 * Tasklets                                                                  *
 *                                                                           *
\*****************************************************************************/

static void sdhci_tasklet_card(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host*)param;

	cancel_delayed_work(&host->mmc->detect);

	spin_lock_irqsave(&host->lock, flags);

	/* Check host->mrq first in case we are runtime suspended */
	if (host->mrq &&
	    !(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_CARD_PRESENT)) {
		pr_err("%s: Card removed during transfer!\n",
			mmc_hostname(host->mmc));
		pr_err("%s: Resetting controller.\n",
			mmc_hostname(host->mmc));

		if (host->quirks2 & SDHCI_QUIRK2_WAIT_FOR_IDLE)
			sdhci_busy_wait(host, 1000);
		sdhci_reset(host, SDHCI_RESET_CMD);
		sdhci_reset(host, SDHCI_RESET_DATA);

		host->mrq->cmd->error = -ENOMEDIUM;
		tasklet_schedule(&host->finish_tasklet);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_detect_change(host->mmc, msecs_to_jiffies(500));
}

static void sdhci_tasklet_finish(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;
	struct mmc_request *mrq;

	host = (struct sdhci_host*)param;

	spin_lock_irqsave(&host->lock, flags);

        /*
         * If this tasklet gets rescheduled while running, it will
         * be run again afterwards but without any active request.
         */
	if (!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if (!(host->flags & SDHCI_DEVICE_DEAD) &&
	    ((mrq->cmd && mrq->cmd->error && mrq->cmd->error != -ENOMEDIUM) ||
		 (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error))) ||
		   (host->quirks & SDHCI_QUIRK_RESET_AFTER_REQUEST))) {

		/* Some controllers need this kick or reset won't work here */
		if (host->quirks & SDHCI_QUIRK_CLOCK_BEFORE_RESET) {
			unsigned int clock;

			/* This is to force an update */
			clock = host->clock;
			host->clock = 0;
			sdhci_set_clock(host, clock);
		}

		/* Spec says we should do both at the same time, but Ricoh
		   controllers do not like that. */
		if (host->quirks2 & SDHCI_QUIRK2_WAIT_FOR_IDLE)
			sdhci_busy_wait(host, 1000);
		sdhci_reset(host, SDHCI_RESET_CMD);
		sdhci_reset(host, SDHCI_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

#ifndef SDHCI_USE_LEDS_CLASS
	sdhci_deactivate_led(host);
#endif

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	sdhci_release_ownership(host->mmc);
	mmc_request_done(host->mmc, mrq);
	sdhci_runtime_pm_put(host);
}

static void sdhci_timeout_timer(unsigned long data)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host*)data;

	spin_lock_irqsave(&host->lock, flags);

	if (host->mrq) {
		pr_err("%s: Timeout waiting for hardware "
			"interrupt.\n", mmc_hostname(host->mmc));
		sdhci_dumpregs(host);

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			sdhci_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			tasklet_schedule(&host->finish_tasklet);
		}
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_tuning_timer(unsigned long data)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host *)data;

	spin_lock_irqsave(&host->lock, flags);

	host->flags |= SDHCI_NEEDS_RETUNING;

	spin_unlock_irqrestore(&host->lock, flags);
}

/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
\*****************************************************************************/

static void sdhci_cmd_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->cmd) {
		pr_err("%s: Got command interrupt 0x%08x even "
			"though no command operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		/* sdhci_dumpregs(host); */
		return;
	}

	if (intmask & SDHCI_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_CRC | SDHCI_INT_END_BIT |
			SDHCI_INT_INDEX))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error) {
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	/*
	 * The host can send and interrupt when the busy state has
	 * ended, allowing us to wait without wasting CPU cycles.
	 * Unfortunately this is overloaded on the "data complete"
	 * interrupt, so we need to take some care when handling
	 * it.
	 *
	 * Note: The 1.0 specification is a bit ambiguous about this
	 *       feature so there might be some problems with older
	 *       controllers.
	 */
	if (host->cmd->flags & MMC_RSP_BUSY) {
		if (host->cmd->data)
			DBG("Cannot wait for busy signal when also "
				"doing a data transfer");
		else if (!(host->quirks & SDHCI_QUIRK_NO_BUSY_IRQ))
			return;

		/* The controller does not support the end-of-busy IRQ,
		 * fall through and take the SDHCI_INT_RESPONSE */
	}

	if (intmask & SDHCI_INT_RESPONSE)
		sdhci_finish_command(host);
}

#ifdef CONFIG_MMC_DEBUG
static void sdhci_show_adma_error(struct sdhci_host *host)
{
	const char *name = mmc_hostname(host->mmc);
	u8 *desc = host->adma_desc;
	__le32 *dma;
	__le16 *len;
	u8 attr;

	sdhci_dumpregs(host);

	while (true) {
		dma = (__le32 *)(desc + 4);
		len = (__le16 *)(desc + 2);
		attr = *desc;

		DBG("%s: %p: DMA 0x%08x, LEN 0x%04x, Attr=0x%02x\n",
		    name, desc, le32_to_cpu(*dma), le16_to_cpu(*len), attr);

		desc += 8;

		if (attr & 2)
			break;
	}
}
#else
static void sdhci_show_adma_error(struct sdhci_host *host) { }
#endif

static void sdhci_data_irq(struct sdhci_host *host, u32 intmask)
{
	u32 command;
	BUG_ON(intmask == 0);

	/* CMD19 generates _only_ Buffer Read Ready interrupt */
	if (intmask & SDHCI_INT_DATA_AVAIL) {
		command = SDHCI_GET_CMD(sdhci_readw(host, SDHCI_COMMAND));
		if (command == MMC_SEND_TUNING_BLOCK ||
		    command == MMC_SEND_TUNING_BLOCK_HS200) {
			host->tuning_done = 1;
			wake_up(&host->buf_ready_int);
			return;
		}
	}

	if (!host->data) {
		/*
		 * The "data complete" interrupt is also used to
		 * indicate that a busy state has ended. See comment
		 * above in sdhci_cmd_irq().
		 *
		 * "data timeout" interrupt may also happen
		 */
		if (host->cmd && (host->cmd->flags & MMC_RSP_BUSY)) {
			if (intmask & SDHCI_INT_DATA_END) {
				sdhci_finish_command(host);
				return;
			} else if (intmask & SDHCI_INT_DATA_TIMEOUT) {
				pr_err("%s: Got data interrupt 0x%08x for "
						"busy cmd %d\n",
						mmc_hostname(host->mmc),
						(unsigned)intmask,
						host->cmd->opcode);
				host->cmd->error = -ETIMEDOUT;
				tasklet_schedule(&host->finish_tasklet);
				return;
			}
		}

		pr_err("%s: Got data interrupt 0x%08x even "
			"though no data operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		sdhci_dumpregs(host);

		return;
	}

	if (intmask & SDHCI_INT_DATA_TIMEOUT)
		host->data->error = -ETIMEDOUT;
	else if (intmask & SDHCI_INT_DATA_END_BIT)
		host->data->error = -EILSEQ;
	else if ((intmask & SDHCI_INT_DATA_CRC) &&
		SDHCI_GET_CMD(sdhci_readw(host, SDHCI_COMMAND))
			!= MMC_BUS_TEST_R) {
		host->data->error = -EILSEQ;
		/*
		 * This may need tuning for HS200/SDR50 and
		 * so on.
		 */
		/* Set the re-tuning expiration flag */
		if ((host->version >= SDHCI_SPEC_300) && host->tuning_count &&
		    (host->tuning_mode == SDHCI_TUNING_MODE_1)) {
			host->mrq->cmd->retries++;
			host->flags |= SDHCI_NEEDS_RETUNING;
			pr_err("%s: encounter CRC error, needs tuning, retry %d\n",
					mmc_hostname(host->mmc),
					host->mrq->cmd->retries);
		}
	} else if (intmask & SDHCI_INT_ADMA_ERROR) {
		pr_err("%s: ADMA error\n", mmc_hostname(host->mmc));
		sdhci_show_adma_error(host);
		host->data->error = -EIO;
	}

	if (host->data->error)
		sdhci_finish_data(host);
	else {
		if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
			sdhci_transfer_pio(host);

		/*
		 * We currently don't do anything fancy with DMA
		 * boundaries, but as we can't disable the feature
		 * we need to at least restart the transfer.
		 *
		 * According to the spec sdhci_readl(host, SDHCI_DMA_ADDRESS)
		 * should return a valid address to continue from, but as
		 * some controllers are faulty, don't trust them.
		 */
		if (intmask & SDHCI_INT_DMA_END) {
			u32 dmastart, dmanow;
			dmastart = sg_dma_address(host->data->sg);
			dmanow = dmastart + host->data->bytes_xfered;
			/*
			 * Force update to the next DMA block boundary.
			 */
			dmanow = (dmanow &
				~(SDHCI_DEFAULT_BOUNDARY_SIZE - 1)) +
				SDHCI_DEFAULT_BOUNDARY_SIZE;
			host->data->bytes_xfered = dmanow - dmastart;
			DBG("%s: DMA base 0x%08x, transferred 0x%06x bytes,"
				" next 0x%08x\n",
				mmc_hostname(host->mmc), dmastart,
				host->data->bytes_xfered, dmanow);
			sdhci_writel(host, dmanow, SDHCI_DMA_ADDRESS);
		}

		if (intmask & SDHCI_INT_DATA_END) {
			if (host->cmd) {
				/*
				 * Data managed to finish before the
				 * command completed. Make sure we do
				 * things in the proper order.
				 */
				host->data_early = 1;
			} else {
				sdhci_finish_data(host);
			}
		}
	}
}

static irqreturn_t sdhci_irq(int irq, void *dev_id)
{
	irqreturn_t result;
	struct sdhci_host *host = dev_id;
	u32 intmask, unexpected = 0;
	int cardint = 0, max_loops = 16;

	spin_lock(&host->lock);

	if (host->runtime_suspended) {
		spin_unlock(&host->lock);
		DBG("%s: got irq while runtime suspended\n",
		       mmc_hostname(host->mmc));
		return IRQ_HANDLED;
	}

	intmask = sdhci_readl(host, SDHCI_INT_STATUS);

	if (!intmask || intmask == 0xffffffff) {
		result = IRQ_NONE;
		goto out;
	}

again:
	DBG("*** %s got interrupt: 0x%08x\n",
		mmc_hostname(host->mmc), intmask);

	if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE)) {
		u32 present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
			      SDHCI_CARD_PRESENT;

		/*
		 * There is a observation on i.mx esdhc.  INSERT bit will be
		 * immediately set again when it gets cleared, if a card is
		 * inserted.  We have to mask the irq to prevent interrupt
		 * storm which will freeze the system.  And the REMOVE gets
		 * the same situation.
		 *
		 * More testing are needed here to ensure it works for other
		 * platforms though.
		 */
		sdhci_mask_irqs(host, present ? SDHCI_INT_CARD_INSERT :
						SDHCI_INT_CARD_REMOVE);
		sdhci_unmask_irqs(host, present ? SDHCI_INT_CARD_REMOVE :
						  SDHCI_INT_CARD_INSERT);

		sdhci_writel(host, intmask & (SDHCI_INT_CARD_INSERT |
			     SDHCI_INT_CARD_REMOVE), SDHCI_INT_STATUS);
		intmask &= ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE);
		tasklet_schedule(&host->card_tasklet);
	}

	if (intmask & SDHCI_INT_CMD_MASK) {
		/*
		 * If encounter command conflict interrupts,
		 * before clearing it, delay 64 clocks, otherwise the interrupts
		 * will be generated again.
		 * This is just experience. SDHC spec doesn't
		 * say the command conflict interrupts will be generated
		 * again without a delay before clearing them.
		 */
		if ((intmask & SDHCI_INT_CMD_CONFLICT) ==
				SDHCI_INT_CMD_CONFLICT) {
			if (host->clock)
				udelay(64 * 1000000 / host->clock);
			else
				udelay(500);
		}
		sdhci_writel(host, intmask & SDHCI_INT_CMD_MASK,
			SDHCI_INT_STATUS);
		sdhci_cmd_irq(host, intmask & SDHCI_INT_CMD_MASK);
	}

	if (intmask & SDHCI_INT_DATA_MASK) {
		sdhci_writel(host, intmask & SDHCI_INT_DATA_MASK,
			SDHCI_INT_STATUS);
		sdhci_data_irq(host, intmask & SDHCI_INT_DATA_MASK);
	}

	intmask &= ~(SDHCI_INT_CMD_MASK | SDHCI_INT_DATA_MASK);

	intmask &= ~SDHCI_INT_ERROR;

	if (intmask & SDHCI_INT_BUS_POWER) {
		pr_err("%s: Card is consuming too much power!\n",
			mmc_hostname(host->mmc));
		sdhci_writel(host, SDHCI_INT_BUS_POWER, SDHCI_INT_STATUS);
	}

	intmask &= ~SDHCI_INT_BUS_POWER;

	if (intmask & SDHCI_INT_CARD_INT)
		cardint = 1;

	intmask &= ~SDHCI_INT_CARD_INT;

	if (intmask) {
		unexpected |= intmask;
		sdhci_writel(host, intmask, SDHCI_INT_STATUS);
	}

	result = IRQ_HANDLED;

	intmask = sdhci_readl(host, SDHCI_INT_STATUS);
	if (intmask && --max_loops)
		goto again;
out:
	spin_unlock(&host->lock);

	if (unexpected) {
		pr_err("%s: Unexpected interrupt 0x%08x.\n",
			   mmc_hostname(host->mmc), unexpected);
		sdhci_dumpregs(host);
	}
	/*
	 * We have to delay this as it calls back into the driver.
	 */
	if (cardint)
		mmc_signal_sdio_irq(host->mmc);

	return result;
}

/************************************************************************\
 *                                                                      *
 * APIs for panic record use						*
 * Note:								*
 * For panic use, please take care of sdhci_read/write.			*
 *									*
 * sdhci_read/write function are defined by sdhci host layer which	*
 * warpped the read/write function.					*
 * But before calling read/write, sdhci_read/write will try to see if	*
 * some host drivers defined special register reading/writing functions.*
 * If do, that is means read/write function defined by kernel cannot be	*
 * used, have to use the special ones.					*
 * So, if host driver are using special ones, please make sure when in	*
 * panic mode, the special ones are still good to use			*
 * So, if not, read/write defined by kernel is safe for panic using	*
 *									*
 * @For MFLD sdhci host controller driver, no special reading/writing	*
 * funtion are used							*
 *                                                                      *
\************************************************************************/

static int panic_irq_done;

static void sdhci_panic_irq_wait(struct sdhci_host *host);

static inline void sdhci_panic_finish_req(struct sdhci_host *host)
{
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	panic_irq_done = 1;
}

/*
 * assuming only use SDMA write and data length is 512Bytes
 */
static void sdhci_panic_send_cmd(struct sdhci_host *host,
		struct mmc_command *cmd)
{
	unsigned long timeout;
	u32 mask;
	int flags;

	WARN_ON(host->cmd);
	/* Wait max 10 ms */
	timeout = 10;
	mask = SDHCI_CMD_INHIBIT;
	if ((cmd->data != 0) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDHCI_DATA_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s %s: Controller never released "
				"inhibit bit(s).\n", __func__,
				mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			sdhci_panic_finish_req(host);
			return;
		}
		timeout--;
		/*
		 * seems card is not ready for the next command.
		 * We can wait for 1ms and then to have a retry
		 */
		mdelay(1);
	}

	host->cmd = cmd;

	/*
	 * set the data timeout register to be max value
	 */
	sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);
	/*
	 * prepare data
	 */
	if (cmd->data) {
		unsigned int mode;
		struct mmc_data *data = cmd->data;
		u32 pio_irqs = SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL;
		u32 dma_irqs = SDHCI_INT_DMA_END | SDHCI_INT_ADMA_ERROR;

		host->data = data;
		host->data_early = 0;
		/*
		 * update DMA address
		 */
		sdhci_writel(host, data->dmabuf, SDHCI_DMA_ADDRESS);

		if (host->version >= SDHCI_SPEC_200) {
			u8 ctrl;
			ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
			ctrl &= ~SDHCI_CTRL_DMA_MASK;
			if ((host->flags & SDHCI_REQ_USE_DMA) &&
				(host->flags & SDHCI_USE_ADMA))
				ctrl |= SDHCI_CTRL_ADMA32;
			else
				ctrl |= SDHCI_CTRL_SDMA;
			sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
		}

		if (host->flags & SDHCI_REQ_USE_DMA)
			sdhci_clear_set_irqs(host, pio_irqs, dma_irqs);
		else
			sdhci_clear_set_irqs(host, dma_irqs, pio_irqs);

		/*
		 * We do not handle DMA boundaries,
		 * so set it to max (512 KiB)
		 */
		sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, data->blksz),
				SDHCI_BLOCK_SIZE);
		sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);

		/*
		 * set transfer mode
		 */
		mode = SDHCI_TRNS_BLK_CNT_EN;
		if (data->blocks > 1) {
			if (host->quirks & SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12)
				mode |= SDHCI_TRNS_MULTI |
					SDHCI_TRNS_AUTO_CMD12;
			else
				mode |= SDHCI_TRNS_MULTI;
		}
		if (host->flags & SDHCI_REQ_USE_DMA)
			mode |= SDHCI_TRNS_DMA;

		sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
	}

	sdhci_writel(host, cmd->arg, SDHCI_ARGUMENT);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		printk(KERN_ERR "%s %s: Unsupported response type!\n",
			__func__, mmc_hostname(host->mmc));
		sdhci_panic_finish_req(host);
		return;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->flags & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
	else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->flags & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (cmd->data)
		flags |= SDHCI_CMD_DATA;

	/*
	 * send command
	 */
	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->opcode, flags), SDHCI_COMMAND);

	mmiowb();

	/*
	 * polling interrupt
	 */
	sdhci_panic_irq_wait(host);
}

static void sdhci_panic_finish_data(struct sdhci_host *host)
{
	struct mmc_data *data;

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	/*
	 * panic use, will not unmap anything here
	 */

	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	if (data->stop) {
		/*
		 * we will not be here since we use single block
		 * transfer when panic occured
		 */
		sdhci_panic_send_cmd(host, data->stop);
	} else
		sdhci_panic_finish_req(host);
}

static void sdhci_panic_finish_command(struct sdhci_host *host)
{
	int i;

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0; i < 4; i++) {
				host->cmd->resp[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
				if (i != 3)
					host->cmd->resp[i] |=
						sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
			}
		} else {
			host->cmd->resp[0] = sdhci_readl(host, SDHCI_RESPONSE);
		}
	}

	host->cmd->error = 0;

	if (host->data && host->data_early)
		sdhci_panic_finish_data(host);

	if (!host->cmd->data)
		sdhci_panic_finish_req(host);

	host->cmd = NULL;
}

/*
 * sdhci_panic_data_irq: handle data irq in panic mode
 *
 * When host is in panic mode, host driver need to poll its interrupt
 * status register. Once looked up some cmd irqs, call this function
 * to handle.
 */
static void sdhci_panic_cmd_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (intmask & SDHCI_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_CRC | SDHCI_INT_END_BIT |
			SDHCI_INT_INDEX))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error) {
		sdhci_panic_finish_req(host);
		return;
	}

	if (host->cmd->flags & MMC_RSP_BUSY) {
		if (host->cmd->data)
			DBG("Cannot wait for busy signal when also "
				"doing a data transfer");
		else if (!(host->quirks & SDHCI_QUIRK_NO_BUSY_IRQ))
			return;
	}

	if (intmask & SDHCI_INT_RESPONSE)
		sdhci_panic_finish_command(host);
}

/*
 * sdhci_panic_data_irq: handle data irq in panic mode
 *
 * When host is in panic mode, host driver need to poll its interrupt
 * status register. Once looked up some data irqs, call this function
 * to handle.
 */
static void sdhci_panic_data_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->data) {
		/*
		 * The "data complete" interrupt is also used to
		 * indicate that a busy state has ended. See comment
		 * above in sdhci_cmd_irq().
		 */
		if (host->cmd && (host->cmd->flags & MMC_RSP_BUSY)) {
			if (intmask & SDHCI_INT_DATA_END) {
				sdhci_panic_finish_command(host);
				return;
			}
		}

		printk(KERN_ERR "%s %s: Got data interrupt 0x%08x even "
			"though no data operation was in progress.\n",
			__func__, mmc_hostname(host->mmc), (unsigned)intmask);
		sdhci_dumpregs(host);

		return;
	}

	if (intmask & SDHCI_INT_DATA_TIMEOUT)
		host->data->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_END_BIT))
		host->data->error = -EILSEQ;
	else if (intmask & SDHCI_INT_ADMA_ERROR) {
		printk(KERN_ERR "%s: ADMA error\n", mmc_hostname(host->mmc));
		host->data->error = -EIO;
	}

	if (host->data->error)
		sdhci_panic_finish_data(host);
	else {
		if (intmask & SDHCI_INT_DMA_END)
			sdhci_writel(host, sdhci_readl(host, SDHCI_DMA_ADDRESS),
				SDHCI_DMA_ADDRESS);

		if (intmask & SDHCI_INT_DATA_END) {
			if (host->cmd)
				host->data_early = 1;
			else
				sdhci_panic_finish_data(host);
		}
	}
}

/*
 * sdhci_panic_irq_wait: irq handler for panic record
 */
static void sdhci_panic_irq_wait(struct sdhci_host *host)
{
	u32 intmask;
	panic_irq_done = 0;
retry:
	intmask = sdhci_readl(host, SDHCI_INT_STATUS);

	if (!intmask || intmask == 0xffffffff)
		goto retry;

	DBG("***%s got interrupt: 0x%08x\n",
		__func__, intmask);

	if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE)) {
		sdhci_writel(host, intmask & (SDHCI_INT_CARD_INSERT |
			SDHCI_INT_CARD_REMOVE), SDHCI_INT_STATUS);
		/*
		 * do nothing for card detect
		 */
	}

	intmask &= ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE);

	if (intmask & SDHCI_INT_CMD_MASK) {
		sdhci_writel(host, intmask & SDHCI_INT_CMD_MASK,
			SDHCI_INT_STATUS);
		sdhci_panic_cmd_irq(host, intmask & SDHCI_INT_CMD_MASK);
	}

	if (intmask & SDHCI_INT_DATA_MASK) {
		sdhci_writel(host, intmask & SDHCI_INT_DATA_MASK,
			SDHCI_INT_STATUS);
		sdhci_panic_data_irq(host, intmask & SDHCI_INT_DATA_MASK);
	}

	intmask &= ~(SDHCI_INT_CMD_MASK | SDHCI_INT_DATA_MASK);

	intmask &= ~SDHCI_INT_ERROR;

	if (intmask & SDHCI_INT_BUS_POWER) {
		printk(KERN_ERR "%s %s: Card is consuming too much power!\n",
			__func__, mmc_hostname(host->mmc));
		sdhci_writel(host, SDHCI_INT_BUS_POWER, SDHCI_INT_STATUS);
	}

	intmask &= ~SDHCI_INT_BUS_POWER;

	if (intmask & SDHCI_INT_CARD_INT) {
		sdhci_writel(host, intmask & SDHCI_INT_CARD_INT,
			SDHCI_INT_STATUS);
		/*
		 * do nothing for this irq
		 */
		intmask &= ~SDHCI_INT_CARD_INT;
	}

	if (intmask) {
		printk(KERN_ERR "%s %s: Unexpected interrupt 0x%08x.\n",
			__func__, mmc_hostname(host->mmc), intmask);
		sdhci_dumpregs(host);

		sdhci_writel(host, intmask, SDHCI_INT_STATUS);
	}

	mmiowb();
	if (!panic_irq_done)
		goto retry;
}

static void sdhci_mfld_panic_set_ios(struct mmc_panic_host *mmc)
{
	struct sdhci_host *host;
	struct mmc_ios *ios = &mmc->ios;
	u8 ctrl;

	host = (struct sdhci_host *)mmc->priv;

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF)
		pr_info("%s: we are in panic, why need power off?\n", __func__);

	sdhci_set_clock(host, ios->clock);

	if (ios->power_mode == MMC_POWER_OFF)
		sdhci_set_power(host, -1);
	else
		sdhci_set_power(host, ios->vdd);

	if (host->ops->platform_send_init_74_clocks)
		host->ops->platform_send_init_74_clocks(host, ios->power_mode);

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if (ios->bus_width == MMC_BUS_WIDTH_8)
		ctrl |= SDHCI_CTRL_8BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_8BITBUS;

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	if (ios->timing && !(host->quirks & SDHCI_QUIRK_NO_HISPD_BIT))
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

	/*
	 * Some (ENE) controllers go apeshit on some ios operation,
	 * signalling timeout and CRC errors even on CMD0. Resetting
	 * it on each ios seems to solve the problem.
	 */
	if (host->quirks & SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS)
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

	mmiowb();
}

static void sdhci_panic_reinit_host(struct mmc_panic_host *mmc)
{
	struct sdhci_host *host = mmc->priv;
	sdhci_init(host, 0);
	host->pwr = 0; /* force power reprogram */
	host->clock = 0; /* force clock reprogram */
	sdhci_mfld_panic_set_ios(mmc);
	mmiowb();
}

static void sdhci_mfld_panic_request(struct mmc_panic_host *panic_mmc,
		struct mmc_request *mrq)
{
	struct sdhci_host *host;
	bool present;
	host = (struct sdhci_host *)panic_mmc->priv;

	WARN_ON(host->mrq != NULL);

	/*
	 * only support single block data DMA write
	 */
	if (mrq->cmd->data) {
		if (mrq->cmd->data->blocks != 1 ||
			mrq->cmd->data->flags & MMC_DATA_READ)
			mrq->cmd->error = -EINVAL;
	}

	if (host->flags & SDHCI_USE_ADMA)
		host->flags &= ~SDHCI_USE_ADMA;

	if (host->quirks & SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12) {
		if (mrq->stop) {
			mrq->data->stop = NULL;
			mrq->stop = NULL;
		}
	}

	host->mrq = mrq;

	/* If polling, assume that the card is always present. */
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		present = true;
	else
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;

	if (!present) {
		host->mrq->cmd->error = -ENOMEDIUM;
		sdhci_panic_finish_req(host);
	} else
		sdhci_panic_send_cmd(host, mrq->cmd);

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if (mrq->cmd->error || (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error))) ||
		   (host->quirks & SDHCI_QUIRK_RESET_AFTER_REQUEST)) {
		pr_err("%s: request handle failed\n", __func__);
		sdhci_dumpregs(host);
		sdhci_panic_reinit_host(panic_mmc);
	}
}

/*
 * The same like sdhci_acquire_ownership, used for IA to get the ownership
 * before using host controller. Since this function is called in panic mode,
 * so we can not use msleep() like sdhci_acquire_ownership does, use mdelay()
 * instead.
 */
static int sdhci_mfld_panic_acquire_ownership(struct sdhci_host *host)
{
	unsigned long t1, t2;
	if (!host->sram_addr)
		return DEKKER_OWNER_IA;

	/* If IA has already hold the eMMC mutex, then just exit */
	if (readl(host->sram_addr + DEKKER_IA_REQ_OFFSET))
		return 0;

	writel(1, host->sram_addr + DEKKER_IA_REQ_OFFSET);

	t1 = 100;
	t2 = 500;

	while (readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET)) {
		if (readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET) !=
				DEKKER_OWNER_IA) {
			writel(0, host->sram_addr + DEKKER_IA_REQ_OFFSET);
			while (t2) {
				if (readl(host->sram_addr +
					DEKKER_EMMC_OWNER_OFFSET) ==
					DEKKER_OWNER_IA)
					break;
				mdelay(10);
				t2--;
			}
			if (t2)
				writel(1, host->sram_addr +
						DEKKER_IA_REQ_OFFSET);
			else
				goto timeout;
		}
		/*
		 * if we go to here, that means SCU FW is releasing the
		 * ownership, so we just wait for a short time here.
		 */
		if (t1) {
			mdelay(10);
			t1--;
		} else
			goto timeout;
	}

	DBG("Acquire ownership - eMMC owner: %d, IA req: %d, SCU req: %d\n",
		readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET),
		readl(host->sram_addr + DEKKER_IA_REQ_OFFSET),
		readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET));

	return (readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET) ==
		DEKKER_OWNER_IA) ? DEKKER_OWNER_SCU : DEKKER_OWNER_IA;
timeout:

	pr_warn("%s: Timeout to hold eMMC mutex\n", __func__);
	return -EBUSY;
}

static int sdhci_mfld_panic_power_on(struct mmc_panic_host *panic_host)
{
	int ret;
	struct mmc_host *mmc = panic_host->mmc;
	struct sdhci_host *host = panic_host->priv;

	if (host->runtime_suspended) {
		/*
		 * power host controller
		 */
		pm_runtime_get_noresume(mmc->parent);

		if (host->ops->power_up_host) {
			ret = host->ops->power_up_host(host);
			if (ret)
				return ret;
		}
		sdhci_panic_reinit_host(panic_host);
		host->runtime_suspended = false;
	}

	return 0;
}

static int sdhci_mfld_panic_hold_mutex(struct mmc_panic_host *panic_host)
{
	struct sdhci_host *host = panic_host->priv;
	int ret;

	ret = sdhci_mfld_panic_acquire_ownership(host);
	if (ret == DEKKER_OWNER_SCU) {
		if (host->ops->power_up_host) {
			ret = host->ops->power_up_host(host);
			if (ret)
				return ret;
		}
		sdhci_panic_reinit_host(panic_host);
		return 0;
	} else if (ret == DEKKER_OWNER_IA)
		return sdhci_mfld_panic_power_on(panic_host);

	return ret;
}

static void sdhci_mfld_panic_release_mutex(struct mmc_panic_host *panic_host)
{
	struct sdhci_host *host = panic_host->priv;

	if (!host->sram_addr)
		return;

	writel(DEKKER_OWNER_SCU,
	       host->sram_addr + DEKKER_EMMC_OWNER_OFFSET);
	writel(0, host->sram_addr + DEKKER_IA_REQ_OFFSET);
	DBG("Exit ownership - eMMC owner: %d, IA req: %d, SCU req: %d\n",
	    readl(host->sram_addr + DEKKER_EMMC_OWNER_OFFSET),
	    readl(host->sram_addr + DEKKER_IA_REQ_OFFSET),
	    readl(host->sram_addr + DEKKER_SCU_REQ_OFFSET));
}

static void sdhci_mfld_panic_prepare(struct mmc_panic_host *panic_host)
{
	struct mmc_host *mmc = panic_host->mmc;
	struct sdhci_host *host = panic_host->priv;
	/*
	 * assume host is powered off
	 */
	host->runtime_suspended = true;

#ifdef CONFIG_PM_RUNTIME
	/*
	 * disable runtime pm directly
	 */
	mmc->parent->power.disable_depth = 1;
#endif
}

static int sdhci_mfld_panic_setup(struct mmc_panic_host *panic_host)
{
	struct sdhci_host *host;

	host = mmc_priv(panic_host->mmc);
	panic_host->priv = (void *)host;

	return 0;
}

const struct mmc_host_panic_ops sdhci_panic_ops = {
	.request	= sdhci_mfld_panic_request,
	.prepare	= sdhci_mfld_panic_prepare,
	.setup		= sdhci_mfld_panic_setup,
	.set_ios	= sdhci_mfld_panic_set_ios,
	.power_on	= sdhci_mfld_panic_power_on,
	.hold_mutex	= sdhci_mfld_panic_hold_mutex,
	.release_mutex	= sdhci_mfld_panic_release_mutex,
};

void sdhci_alloc_panic_host(struct sdhci_host *host)
{
	if (!host->mmc)
		return;
	mmc_alloc_panic_host(host->mmc, &sdhci_panic_ops);
}
EXPORT_SYMBOL_GPL(sdhci_alloc_panic_host);

/*****************************************************************************\
 *                                                                           *
 * Suspend/resume                                                            *
 *                                                                           *
\*****************************************************************************/

static void sdhci_set_emmc_state(struct sdhci_host *host, uint32_t state)
{
	/* Only if there is dekker mutex available */
	if (!host->sram_addr)
		return;
	writel(state, host->sram_addr + DEKKER_EMMC_STATE);
}

#ifdef CONFIG_PM

int sdhci_suspend_host(struct sdhci_host *host)
{
	int ret;
	bool has_tuning_timer;
	unsigned long flags;

	if (host->ops->platform_suspend)
		host->ops->platform_suspend(host);

	sdhci_acquire_ownership(host->mmc);

	sdhci_disable_card_detection(host);

	/* Disable tuning since we are suspending */
	has_tuning_timer = host->version >= SDHCI_SPEC_300 &&
		host->tuning_count && host->tuning_mode == SDHCI_TUNING_MODE_1;
	if (has_tuning_timer) {
		del_timer_sync(&host->tuning_timer);
		host->flags &= ~SDHCI_NEEDS_RETUNING;
	}

	ret = mmc_suspend_host(host->mmc);
	if (ret) {
		if (has_tuning_timer) {
			host->flags |= SDHCI_NEEDS_RETUNING;
			mod_timer(&host->tuning_timer, jiffies +
					host->tuning_count * HZ);
		}

		sdhci_enable_card_detection(host);

		goto out;
	}

	free_irq(host->irq, host);

	/* Card succesfully suspended. Tell information to SCU */
	sdhci_set_emmc_state(host, DEKKER_EMMC_CHIP_SUSPENDED);

	spin_lock_irqsave(&host->lock, flags);
	host->suspended = true;
	spin_unlock_irqrestore(&host->lock, flags);
out:
	sdhci_release_ownership(host->mmc);
	return ret;
}

EXPORT_SYMBOL_GPL(sdhci_suspend_host);

int sdhci_resume_host(struct sdhci_host *host)
{
	int ret;
	unsigned long flags;

	if (host->quirks2 & SDHCI_QUIRK2_CARD_CD_DELAY) {
		int loop = 0;
		unsigned int present;
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
			SDHCI_CARD_PRESENT;
		/*
		 * delay 10ms to wait for present register stable
		 * try 5 loops, and each loops will wait 2ms
		 */
		while (!present && loop < 5) {
			/* BYT eMMC4.5 silicon issue workaround: 4599639 */
			mdelay(2);
			present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;
			loop++;
		}
		if (loop == 5) {
			WARN_ON(1);
			pr_warn("%s %s: PRESENT bit16 is not recover\n",
					__func__, mmc_hostname(host->mmc));
		}
	}

	sdhci_acquire_ownership(host->mmc);

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if (host->ops->enable_dma)
			host->ops->enable_dma(host);
	}

	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,
			  mmc_hostname(host->mmc), host);
	if (ret)
		goto out;

	if ((host->mmc->pm_flags & MMC_PM_KEEP_POWER) &&
	    (host->quirks2 & SDHCI_QUIRK2_HOST_OFF_CARD_ON)) {
		/* Card keeps power but host controller does not */
		sdhci_init(host, 0);
		host->pwr = 0;
		host->clock = 0;
		sdhci_do_set_ios(host, &host->mmc->ios);
		sdhci_do_start_signal_voltage_switch(host,
					&host->mmc->ios, false);
	} else {
		sdhci_init(host, (host->mmc->pm_flags & MMC_PM_KEEP_POWER));
		mmiowb();
	}

	spin_lock_irqsave(&host->lock, flags);
	host->suspended = false;
	spin_unlock_irqrestore(&host->lock, flags);

	ret = mmc_resume_host(host->mmc);
	sdhci_enable_card_detection(host);

	if (host->ops->platform_resume)
		host->ops->platform_resume(host);

	/* Set the re-tuning expiration flag */
	if ((host->version >= SDHCI_SPEC_300) && host->tuning_count &&
	    (host->tuning_mode == SDHCI_TUNING_MODE_1))
		host->flags |= SDHCI_NEEDS_RETUNING;

	/* Card back in active state */
	sdhci_set_emmc_state(host, DEKKER_EMMC_CHIP_ACTIVE);
out:
	sdhci_release_ownership(host->mmc);
	return ret;
}

EXPORT_SYMBOL_GPL(sdhci_resume_host);

void sdhci_enable_irq_wakeups(struct sdhci_host *host)
{
	u8 val;
	val = sdhci_readb(host, SDHCI_WAKE_UP_CONTROL);
	val |= SDHCI_WAKE_ON_INT;
	sdhci_writeb(host, val, SDHCI_WAKE_UP_CONTROL);
}

EXPORT_SYMBOL_GPL(sdhci_enable_irq_wakeups);

#endif /* CONFIG_PM */

#ifdef CONFIG_PM_RUNTIME

static int sdhci_runtime_pm_get(struct sdhci_host *host)
{
	return pm_runtime_get_sync(host->mmc->parent);
}

static int sdhci_runtime_pm_put(struct sdhci_host *host)
{
	pm_runtime_mark_last_busy(host->mmc->parent);
	return pm_runtime_put_autosuspend(host->mmc->parent);
}

int sdhci_runtime_suspend_host(struct sdhci_host *host)
{
	unsigned long flags;
	int ret = 0;

	sdhci_do_acquire_ownership(host->mmc);
	/* Disable tuning since we are suspending */
	if (host->version >= SDHCI_SPEC_300 &&
	    host->tuning_mode == SDHCI_TUNING_MODE_1) {
		del_timer_sync(&host->tuning_timer);
		host->flags &= ~SDHCI_NEEDS_RETUNING;
	}

	spin_lock_irqsave(&host->lock, flags);
	sdhci_mask_irqs(host, SDHCI_INT_ALL_MASK);
	spin_unlock_irqrestore(&host->lock, flags);

	synchronize_irq(host->irq);

	spin_lock_irqsave(&host->lock, flags);
	host->runtime_suspended = true;
	spin_unlock_irqrestore(&host->lock, flags);

	sdhci_release_ownership(host->mmc);
	return ret;
}
EXPORT_SYMBOL_GPL(sdhci_runtime_suspend_host);

int sdhci_runtime_resume_host(struct sdhci_host *host)
{
	unsigned long flags;
	int ret = 0, host_flags = host->flags;

	if (host->quirks2 & SDHCI_QUIRK2_CARD_CD_DELAY) {
		int loop = 0;
		unsigned int present;
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
			SDHCI_CARD_PRESENT;
		/*
		 * delay 10ms to wait for present register stable
		 * try 5 loops, and each loops will wait 2ms
		 */
		while (!present && loop < 5) {
			/* BYT eMMC4.5 silicon issue workaround: 4599639 */
			mdelay(2);
			present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;
			loop++;
		}
		if (loop == 5) {
			WARN_ON(1);
			pr_warn("%s %s: PRESENT bit16 is not recover\n",
					__func__, mmc_hostname(host->mmc));
		}
	}

	sdhci_do_acquire_ownership(host->mmc);

	if (host_flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if (host->ops->enable_dma)
			host->ops->enable_dma(host);
	}

	sdhci_init(host, 0);

	/* Force clock and power re-program */
	host->pwr = 0;
	host->clock = 0;
	sdhci_do_set_ios(host, &host->mmc->ios);
	sdhci_do_start_signal_voltage_switch(host,
				&host->mmc->ios, false);

	if (host_flags & SDHCI_PV_ENABLED)
		sdhci_do_enable_preset_value(host, true);

	/* Set the re-tuning expiration flag */
	if ((host->version >= SDHCI_SPEC_300) && host->tuning_count &&
	    (host->tuning_mode == SDHCI_TUNING_MODE_1))
		host->flags |= SDHCI_NEEDS_RETUNING;

	spin_lock_irqsave(&host->lock, flags);

	host->runtime_suspended = false;

	/* Enable SDIO IRQ */
	if ((host->flags & SDHCI_SDIO_IRQ_ENABLED))
		sdhci_enable_sdio_irq_nolock(host, true);

	/* Enable Card Detection */
	sdhci_enable_card_detection(host);

	spin_unlock_irqrestore(&host->lock, flags);

	sdhci_release_ownership(host->mmc);
	return ret;
}
EXPORT_SYMBOL_GPL(sdhci_runtime_resume_host);

#endif

/*****************************************************************************\
 *                                                                           *
 * Device allocation/registration                                            *
 *                                                                           *
\*****************************************************************************/

struct sdhci_host *sdhci_alloc_host(struct device *dev,
	size_t priv_size)
{
	struct mmc_host *mmc;
	struct sdhci_host *host;

	WARN_ON(dev == NULL);

	mmc = mmc_alloc_host(sizeof(struct sdhci_host) + priv_size, dev);
	if (!mmc)
		return ERR_PTR(-ENOMEM);

	host = mmc_priv(mmc);
	host->mmc = mmc;

	return host;
}

EXPORT_SYMBOL_GPL(sdhci_alloc_host);

/**
 *      sdhci_pci_request_regulators - try requesting regulator of
 *                                     a sdhci device
 *
 *      We take care of race conditions here between sdhci_add_host() (probe)
 *      and platform code that may kick a retry at anytime during boot.
 */
int sdhci_try_get_regulator(struct sdhci_host *host)
{
	struct regulator *vmmc;
	unsigned long flags;
	if (!host->vmmc) {
		vmmc = regulator_get(mmc_dev(host->mmc), "vmmc");
		if (!IS_ERR(vmmc)) {
			spin_lock_irqsave(&host->lock, flags);
			if (!host->vmmc) {
				host->vmmc = vmmc;
				spin_unlock_irqrestore(&host->lock, flags);
				return 0;
			} else {
			  /* race! we got the regulator twice */
				spin_unlock_irqrestore(&host->lock, flags);
				regulator_put(vmmc);
			}
		}
	}
	return -EAGAIN;
}
EXPORT_SYMBOL_GPL(sdhci_try_get_regulator);

int sdhci_add_host(struct sdhci_host *host)
{
	struct mmc_host *mmc;
	u32 caps[2];
	u32 max_current_caps;
	unsigned int ocr_avail;
	int ret;

	WARN_ON(host == NULL);
	if (host == NULL)
		return -EINVAL;

	mmc = host->mmc;

	if (debug_quirks)
		host->quirks = debug_quirks;
	if (debug_quirks2)
		host->quirks2 = debug_quirks2;

	sdhci_reset(host, SDHCI_RESET_ALL);

	host->version = sdhci_readw(host, SDHCI_HOST_VERSION);
	host->version = (host->version & SDHCI_SPEC_VER_MASK)
				>> SDHCI_SPEC_VER_SHIFT;
	if (host->version > SDHCI_SPEC_300) {
		pr_err("%s: Unknown controller version (%d). "
			"You may experience problems.\n", mmc_hostname(mmc),
			host->version);
	}

	caps[0] = (host->quirks & SDHCI_QUIRK_MISSING_CAPS) ? host->caps :
		sdhci_readl(host, SDHCI_CAPABILITIES);

	if (host->quirks2 & SDHCI_QUIRK2_CAN_VDD_300)
		caps[0] |= SDHCI_CAN_VDD_300;
	if (host->quirks2 & SDHCI_QUIRK2_CAN_VDD_330)
		caps[0] |= SDHCI_CAN_VDD_330;

	caps[1] = (host->version >= SDHCI_SPEC_300) ?
		sdhci_readl(host, SDHCI_CAPABILITIES_1) : 0;

	if (host->quirks & SDHCI_QUIRK_FORCE_DMA)
		host->flags |= SDHCI_USE_SDMA;
	else if (!(caps[0] & SDHCI_CAN_DO_SDMA))
		DBG("Controller doesn't have SDMA capability\n");
	else
		host->flags |= SDHCI_USE_SDMA;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_DMA) &&
		(host->flags & SDHCI_USE_SDMA)) {
		DBG("Disabling DMA as it is marked broken\n");
		host->flags &= ~SDHCI_USE_SDMA;
	}

	if ((host->version >= SDHCI_SPEC_200) &&
		(caps[0] & SDHCI_CAN_DO_ADMA2))
		host->flags |= SDHCI_USE_ADMA;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_ADMA) &&
		(host->flags & SDHCI_USE_ADMA)) {
		DBG("Disabling ADMA as it is marked broken\n");
		host->flags &= ~SDHCI_USE_ADMA;
	}

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if (host->ops->enable_dma) {
			if (host->ops->enable_dma(host)) {
				pr_warning("%s: No suitable DMA "
					"available. Falling back to PIO.\n",
					mmc_hostname(mmc));
				host->flags &=
					~(SDHCI_USE_SDMA | SDHCI_USE_ADMA);
			}
		}
	}

	if (host->flags & SDHCI_USE_ADMA) {
		/*
		 * We need to allocate descriptors for all sg entries
		 * (128) and potentially one alignment transfer for
		 * each of those entries.
		 */
		host->adma_desc = kmalloc((128 * 2 + 1) * 4, GFP_KERNEL);
		host->align_buffer = kmalloc(128 * 4, GFP_KERNEL);
		if (!host->adma_desc || !host->align_buffer) {
			kfree(host->adma_desc);
			kfree(host->align_buffer);
			pr_warning("%s: Unable to allocate ADMA "
				"buffers. Falling back to standard DMA.\n",
				mmc_hostname(mmc));
			host->flags &= ~SDHCI_USE_ADMA;
		}
	}

	/*
	 * If we use DMA, then it's up to the caller to set the DMA
	 * mask, but PIO does not need the hw shim so we set a new
	 * mask here in that case.
	 */
	if (!(host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))) {
		host->dma_mask = DMA_BIT_MASK(64);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
	}

	if (host->version >= SDHCI_SPEC_300)
		host->max_clk = (caps[0] & SDHCI_CLOCK_V3_BASE_MASK)
			>> SDHCI_CLOCK_BASE_SHIFT;
	else
		host->max_clk = (caps[0] & SDHCI_CLOCK_BASE_MASK)
			>> SDHCI_CLOCK_BASE_SHIFT;

	host->max_clk *= 1000000;
	if (host->max_clk == 0 || host->quirks &
			SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN) {
		if (!host->ops->get_max_clock) {
			pr_err("%s: Hardware doesn't specify base clock "
			       "frequency.\n", mmc_hostname(mmc));
			return -ENODEV;
		}
		host->max_clk = host->ops->get_max_clock(host);
	}

	/*
	 * In case of Host Controller v3.00, find out whether clock
	 * multiplier is supported.
	 */
	host->clk_mul = (caps[1] & SDHCI_CLOCK_MUL_MASK) >>
			SDHCI_CLOCK_MUL_SHIFT;

	/*
	 * In case the value in Clock Multiplier is 0, then programmable
	 * clock mode is not supported, otherwise the actual clock
	 * multiplier is one more than the value of Clock Multiplier
	 * in the Capabilities Register.
	 */
	if (host->clk_mul)
		host->clk_mul += 1;

	/*
	 * Set host parameters.
	 */
	mmc->ops = &sdhci_ops;
	mmc->f_max = host->max_clk;
	if (host->ops->get_min_clock)
		mmc->f_min = host->ops->get_min_clock(host);
	else if (host->version >= SDHCI_SPEC_300) {
		if (host->clk_mul) {
			mmc->f_min = (host->max_clk * host->clk_mul) / 1024;
			mmc->f_max = host->max_clk * host->clk_mul;
		} else
			mmc->f_min = host->max_clk / SDHCI_MAX_DIV_SPEC_300;
	} else
		mmc->f_min = host->max_clk / SDHCI_MAX_DIV_SPEC_200;

	host->timeout_clk =
		(caps[0] & SDHCI_TIMEOUT_CLK_MASK) >> SDHCI_TIMEOUT_CLK_SHIFT;
	if (host->timeout_clk == 0) {
		if (host->ops->get_timeout_clock) {
			host->timeout_clk = host->ops->get_timeout_clock(host);
		} else if (!(host->quirks &
				SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK)) {
			pr_err("%s: Hardware doesn't specify timeout clock "
			       "frequency.\n", mmc_hostname(mmc));
			return -ENODEV;
		}
	}
	if (caps[0] & SDHCI_TIMEOUT_CLK_UNIT)
		host->timeout_clk *= 1000;

	if (host->quirks & SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK)
		host->timeout_clk = mmc->f_max / 1000;

	mmc->max_discard_to = (1 << 27) / host->timeout_clk;

	mmc->caps |= MMC_CAP_SDIO_IRQ | MMC_CAP_ERASE | MMC_CAP_CMD23;

	mmc->caps |= MMC_CAP_POWER_OFF_CARD;

	if (host->quirks & SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12)
		host->flags |= SDHCI_AUTO_CMD12;

	/* Auto-CMD23 stuff only works in ADMA or PIO. */
	if ((host->version >= SDHCI_SPEC_300) &&
		!(host->quirks2 & SDHCI_QUIRK2_BROKEN_AUTO_CMD23) &&
		((host->flags & SDHCI_USE_ADMA) ||
		!(host->flags & SDHCI_USE_SDMA))) {
		host->flags |= SDHCI_AUTO_CMD23;
		DBG("%s: Auto-CMD23 available\n", mmc_hostname(mmc));
	} else {
		DBG("%s: Auto-CMD23 unavailable\n", mmc_hostname(mmc));
	}

	/*
	 * A controller may support 8-bit width, but the board itself
	 * might not have the pins brought out.  Boards that support
	 * 8-bit width must set "mmc->caps |= MMC_CAP_8_BIT_DATA;" in
	 * their platform code before calling sdhci_add_host(), and we
	 * won't assume 8-bit width for hosts without that CAP.
	 */
	if (!(host->quirks & SDHCI_QUIRK_FORCE_1_BIT_DATA))
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (!(host->quirks2 & SDHCI_QUIRK2_DISABLE_HIGH_SPEED) &&
		(caps[0] & SDHCI_CAN_DO_HISPD))
		mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION) &&
	    mmc_card_is_removable(mmc))
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	if (host->quirks2 & SDHCI_QUIRK2_NO_1_8_V)
		caps[1] &= ~(SDHCI_SUPPORT_SDR104 | SDHCI_SUPPORT_SDR50 |
		       SDHCI_SUPPORT_DDR50);

	/* Any UHS-I mode in caps implies SDR12 and SDR25 support. */
	if (caps[1] & (SDHCI_SUPPORT_SDR104 | SDHCI_SUPPORT_SDR50 |
		       SDHCI_SUPPORT_DDR50))
		mmc->caps |= MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25;

	/* SDR104 supports also implies SDR50 support */
	if (caps[1] & SDHCI_SUPPORT_SDR104)
		mmc->caps |= MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_SDR50;
	else if (caps[1] & SDHCI_SUPPORT_SDR50)
		mmc->caps |= MMC_CAP_UHS_SDR50;

	if ((caps[1] & SDHCI_SUPPORT_DDR50) ||
			(host->quirks2 & SDHCI_QUIRK2_V2_0_SUPPORT_DDR50))
		mmc->caps |= MMC_CAP_UHS_DDR50;

	/* Does the host need tuning for SDR50? */
	if (caps[1] & SDHCI_USE_SDR50_TUNING)
		host->flags |= SDHCI_SDR50_NEEDS_TUNING;

	/* Does the host need tuning for HS200? */
	if (mmc->caps2 & MMC_CAP2_HS200)
		host->flags |= SDHCI_HS200_NEEDS_TUNING;

	/* Driver Type(s) (A, C, D) supported by the host */
	if (caps[1] & SDHCI_DRIVER_TYPE_A)
		mmc->caps |= MMC_CAP_DRIVER_TYPE_A;
	if (caps[1] & SDHCI_DRIVER_TYPE_C)
		mmc->caps |= MMC_CAP_DRIVER_TYPE_C;
	if (caps[1] & SDHCI_DRIVER_TYPE_D)
		mmc->caps |= MMC_CAP_DRIVER_TYPE_D;

	/* Initial value for re-tuning timer count */
	host->tuning_count = (caps[1] & SDHCI_RETUNING_TIMER_COUNT_MASK) >>
			      SDHCI_RETUNING_TIMER_COUNT_SHIFT;
	if (host->tuning_count == 0 && host->ops->get_tuning_count)
		host->tuning_count = host->ops->get_tuning_count(host);

	/*
	 * In case Re-tuning Timer is not disabled, the actual value of
	 * re-tuning timer will be 2 ^ (n - 1).
	 */
	if (host->tuning_count)
		host->tuning_count = 1 << (host->tuning_count - 1);

	/* Re-tuning mode supported by the Host Controller */
	host->tuning_mode = (caps[1] & SDHCI_RETUNING_MODE_MASK) >>
			     SDHCI_RETUNING_MODE_SHIFT;

	ocr_avail = 0;
	/*
	 * According to SD Host Controller spec v3.00, if the Host System
	 * can afford more than 150mA, Host Driver should set XPC to 1. Also
	 * the value is meaningful only if Voltage Support in the Capabilities
	 * register is set. The actual current value is 4 times the register
	 * value.
	 */
	max_current_caps = sdhci_readl(host, SDHCI_MAX_CURRENT);

	if (caps[0] & SDHCI_CAN_VDD_330) {
		int max_current_330;

		ocr_avail |= MMC_VDD_32_33 | MMC_VDD_33_34;

		max_current_330 = ((max_current_caps &
				   SDHCI_MAX_CURRENT_330_MASK) >>
				   SDHCI_MAX_CURRENT_330_SHIFT) *
				   SDHCI_MAX_CURRENT_MULTIPLIER;

		if (max_current_330 > 150)
			mmc->caps |= MMC_CAP_SET_XPC_330;
	}
	if (caps[0] & SDHCI_CAN_VDD_300) {
		int max_current_300;

		ocr_avail |= MMC_VDD_29_30 | MMC_VDD_30_31;

		max_current_300 = ((max_current_caps &
				   SDHCI_MAX_CURRENT_300_MASK) >>
				   SDHCI_MAX_CURRENT_300_SHIFT) *
				   SDHCI_MAX_CURRENT_MULTIPLIER;

		if (max_current_300 > 150)
			mmc->caps |= MMC_CAP_SET_XPC_300;
	}
	if (caps[0] & SDHCI_CAN_VDD_180) {
		int max_current_180;

		ocr_avail |= MMC_VDD_165_195;

		max_current_180 = ((max_current_caps &
				   SDHCI_MAX_CURRENT_180_MASK) >>
				   SDHCI_MAX_CURRENT_180_SHIFT) *
				   SDHCI_MAX_CURRENT_MULTIPLIER;

		if (max_current_180 > 150)
			mmc->caps |= MMC_CAP_SET_XPC_180;

		/* Maximum current capabilities of the host at 1.8V */
		if (max_current_180 >= 800)
			mmc->caps |= MMC_CAP_MAX_CURRENT_800;
		else if (max_current_180 >= 600)
			mmc->caps |= MMC_CAP_MAX_CURRENT_600;
		else if (max_current_180 >= 400)
			mmc->caps |= MMC_CAP_MAX_CURRENT_400;
		else
			mmc->caps |= MMC_CAP_MAX_CURRENT_200;
	}

	if (host->quirks2 & SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8)
		ocr_avail |= MMC_VDD_20_21;

	mmc->ocr_avail = ocr_avail;
	mmc->ocr_avail_sdio = ocr_avail;
	if (host->ocr_avail_sdio)
		mmc->ocr_avail_sdio &= host->ocr_avail_sdio;
	mmc->ocr_avail_sd = ocr_avail;
	if (host->ocr_avail_sd)
		mmc->ocr_avail_sd &= host->ocr_avail_sd;
	else /* normal SD controllers don't support 1.8V */
		mmc->ocr_avail_sd &= ~MMC_VDD_165_195;
	mmc->ocr_avail_mmc = ocr_avail;
	if (host->ocr_avail_mmc)
		mmc->ocr_avail_mmc &= host->ocr_avail_mmc;

	if (mmc->ocr_avail == 0) {
		pr_err("%s: Hardware doesn't report any "
			"support voltages.\n", mmc_hostname(mmc));
		return -ENODEV;
	}

	spin_lock_init(&host->lock);

	/*
	 * Maximum number of segments. Depends on if the hardware
	 * can do scatter/gather or not.
	 */
	if (host->flags & SDHCI_USE_ADMA)
		mmc->max_segs = 128;
	else if (host->flags & SDHCI_USE_SDMA)
		mmc->max_segs = 1;
	else /* PIO */
		mmc->max_segs = 128;

	/*
	 * Maximum number of sectors in one transfer. Limited by DMA boundary
	 * size (512KiB).
	 */
	mmc->max_req_size = 524288;

	/*
	 * Maximum segment size. Could be one segment with the maximum number
	 * of bytes. When doing hardware scatter/gather, each entry cannot
	 * be larger than 64 KiB though.
	 */
	if (host->flags & SDHCI_USE_ADMA) {
		if (host->quirks & SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC)
			mmc->max_seg_size = 65535;
		else
			mmc->max_seg_size = 65536;
	} else {
		mmc->max_seg_size = mmc->max_req_size;
	}

	/*
	 * Maximum block size. This varies from controller to controller and
	 * is specified in the capabilities register.
	 */
	if (host->quirks & SDHCI_QUIRK_FORCE_BLK_SZ_2048) {
		mmc->max_blk_size = 2;
	} else {
		mmc->max_blk_size = (caps[0] & SDHCI_MAX_BLOCK_MASK) >>
				SDHCI_MAX_BLOCK_SHIFT;
		if (mmc->max_blk_size >= 3) {
			pr_warning("%s: Invalid maximum block size, "
				"assuming 512 bytes\n", mmc_hostname(mmc));
			mmc->max_blk_size = 0;
		}
	}

	mmc->max_blk_size = 512 << mmc->max_blk_size;

	/*
	 * Maximum block count.
	 */
	mmc->max_blk_count = (host->quirks & SDHCI_QUIRK_NO_MULTIBLOCK) ? 1 : 65535;

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->card_tasklet,
		sdhci_tasklet_card, (unsigned long)host);
	tasklet_init(&host->finish_tasklet,
		sdhci_tasklet_finish, (unsigned long)host);

	setup_timer(&host->timer, sdhci_timeout_timer, (unsigned long)host);

	if (host->version >= SDHCI_SPEC_300) {
		init_waitqueue_head(&host->buf_ready_int);

		/* Initialize re-tuning timer */
		init_timer(&host->tuning_timer);
		host->tuning_timer.data = (unsigned long)host;
		host->tuning_timer.function = sdhci_tuning_timer;
	}

	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,
		mmc_hostname(mmc), host);
	if (ret)
		goto untasklet;

	sdhci_try_get_regulator(host);

	sdhci_do_acquire_ownership(mmc);

	sdhci_init(host, 0);

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif

#ifdef SDHCI_USE_LEDS_CLASS
	if (mmc->caps2 & MMC_CAP2_LED_SUPPORT) {
		snprintf(host->led_name, sizeof(host->led_name),
			"%s::", mmc_hostname(mmc));
		host->led.name = host->led_name;
		host->led.brightness = LED_OFF;
		host->led.default_trigger = mmc_hostname(mmc);
		host->led.brightness_set = sdhci_led_control;

		ret = led_classdev_register(mmc_dev(mmc), &host->led);
		if (ret)
			goto reset;
	}
#endif

	mmiowb();

	mmc_add_host(mmc);

	pr_info("%s: SDHCI controller on %s [%s] using %s\n",
		mmc_hostname(mmc), host->hw_name, dev_name(mmc_dev(mmc)),
		(host->flags & SDHCI_USE_ADMA) ? "ADMA" :
		(host->flags & SDHCI_USE_SDMA) ? "DMA" : "PIO");

	sdhci_enable_card_detection(host);

	sdhci_release_ownership(mmc);

	return 0;

#ifdef SDHCI_USE_LEDS_CLASS
reset:
	if (mmc->caps2 & MMC_CAP2_LED_SUPPORT) {
		sdhci_reset(host, SDHCI_RESET_ALL);
		free_irq(host->irq, host);
	}
	sdhci_release_ownership(mmc);

#endif
untasklet:
	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	return ret;
}

EXPORT_SYMBOL_GPL(sdhci_add_host);

void sdhci_remove_host(struct sdhci_host *host, int dead)
{
	unsigned long flags;

	if (dead) {
		spin_lock_irqsave(&host->lock, flags);

		host->flags |= SDHCI_DEVICE_DEAD;

		if (host->mrq) {
			pr_err("%s: Controller removed during "
				" transfer!\n", mmc_hostname(host->mmc));

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
		}

		spin_unlock_irqrestore(&host->lock, flags);
	}

	sdhci_disable_card_detection(host);

	mmc_remove_host(host->mmc);

#ifdef SDHCI_USE_LEDS_CLASS
	if (host->mmc->caps2 & MMC_CAP2_LED_SUPPORT)
		led_classdev_unregister(&host->led);
#endif

	if (!dead)
		sdhci_reset(host, SDHCI_RESET_ALL);

	free_irq(host->irq, host);

	del_timer_sync(&host->timer);
	if (host->version >= SDHCI_SPEC_300)
		del_timer_sync(&host->tuning_timer);

	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	if (host->vmmc)
		regulator_put(host->vmmc);

	kfree(host->adma_desc);
	kfree(host->align_buffer);

	host->adma_desc = NULL;
	host->align_buffer = NULL;
}

EXPORT_SYMBOL_GPL(sdhci_remove_host);

void sdhci_free_host(struct sdhci_host *host)
{
	mmc_free_host(host->mmc);
}

EXPORT_SYMBOL_GPL(sdhci_free_host);

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init sdhci_drv_init(void)
{
	pr_info(DRIVER_NAME
		": Secure Digital Host Controller Interface driver\n");
	pr_info(DRIVER_NAME ": Copyright(c) Pierre Ossman\n");

	return 0;
}

static void __exit sdhci_drv_exit(void)
{
}

module_init(sdhci_drv_init);
module_exit(sdhci_drv_exit);

module_param(debug_quirks, uint, 0444);
module_param(debug_quirks2, uint, 0444);

MODULE_AUTHOR("Pierre Ossman <pierre@ossman.eu>");
MODULE_DESCRIPTION("Secure Digital Host Controller Interface core driver");
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(debug_quirks, "Force certain quirks.");
MODULE_PARM_DESC(debug_quirks2, "Force certain other quirks.");
