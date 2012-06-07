/*
 *  linux/drivers/mmc/core/mmc_ops.h
 *
 *  Copyright 2006-2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "mmc_ops.h"

static int _mmc_select_card(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);

	cmd.opcode = MMC_SELECT_CARD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	}

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	return 0;
}

int mmc_select_card(struct mmc_card *card)
{
	BUG_ON(!card);

	return _mmc_select_card(card->host, card);
}

int mmc_deselect_cards(struct mmc_host *host)
{
	return _mmc_select_card(host, NULL);
}

int mmc_card_sleepawake(struct mmc_host *host, int sleep)
{
	struct mmc_command cmd = {0};
	struct mmc_card *card = host->card;
	int err;

	if (sleep)
		mmc_deselect_cards(host);

	cmd.opcode = MMC_SLEEP_AWAKE;
	cmd.arg = card->rca << 16;
	if (sleep)
		cmd.arg |= 1 << 15;

	cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	/*
	 * If the host does not wait while the card signals busy, then we will
	 * will have to wait the sleep/awake timeout.  Note, we cannot use the
	 * SEND_STATUS command to poll the status because that command (and most
	 * others) is invalid while the card sleeps.
	 */
	if (!(host->caps & MMC_CAP_WAIT_WHILE_BUSY))
		mmc_delay(DIV_ROUND_UP(card->ext_csd.sa_timeout, 10000));

	if (!sleep)
		err = mmc_select_card(card);

	return err;
}

int mmc_go_idle(struct mmc_host *host)
{
	int err;
	struct mmc_command cmd = {0};

	/*
	 * Non-SPI hosts need to prevent chipselect going active during
	 * GO_IDLE; that would put chips into SPI mode.  Remind them of
	 * that in case of hardware that won't pull up DAT3/nCS otherwise.
	 *
	 * SPI hosts ignore ios.chip_select; it's managed according to
	 * rules that must accommodate non-MMC slaves which this layer
	 * won't even know about.
	 */
	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_HIGH);
		mmc_delay(1);
	}

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	mmc_delay(1);

	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_DONTCARE);
		mmc_delay(1);
	}

	host->use_spi_crc = 0;

	return err;
}

int mmc_send_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd = {0};
	int i, err = 0;

	BUG_ON(!host);

	cmd.opcode = MMC_SEND_OP_COND;
	cmd.arg = mmc_host_is_spi(host) ? 0 : ocr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = mmc_wait_for_cmd(host, &cmd, 0);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (mmc_host_is_spi(host)) {
			if (!(cmd.resp[0] & R1_SPI_IDLE))
				break;
		} else {
			if (cmd.resp[0] & MMC_CARD_BUSY)
				break;
		}

		err = -ETIMEDOUT;

		mmc_delay(10);
	}

	if (rocr && !mmc_host_is_spi(host))
		*rocr = cmd.resp[0];

	return err;
}

int mmc_all_send_cid(struct mmc_host *host, u32 *cid)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);
	BUG_ON(!cid);

	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cid, cmd.resp, sizeof(u32) * 4);

	return 0;
}

int mmc_set_relative_addr(struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	return 0;
}

static int
mmc_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);
	BUG_ON(!cxd);

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int
mmc_send_cxd_data(struct mmc_card *card, struct mmc_host *host,
		u32 opcode, void *buf, unsigned len)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	void *data_buf;

	/* dma onto stack is unsafe/nonportable, but callers to this
	 * routine normally provide temporary on-stack buffers ...
	 */
	data_buf = kmalloc(len, GFP_KERNEL);
	if (data_buf == NULL)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, data_buf, len);

	if (opcode == MMC_SEND_CSD || opcode == MMC_SEND_CID) {
		/*
		 * The spec states that CSR and CID accesses have a timeout
		 * of 64 clock cycles.
		 */
		data.timeout_ns = 0;
		data.timeout_clks = 64;
	} else
		mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(host, &mrq);

	memcpy(buf, data_buf, len);
	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

int mmc_send_csd(struct mmc_card *card, u32 *csd)
{
	int ret, i;

	if (!mmc_host_is_spi(card->host))
		return mmc_send_cxd_native(card->host, card->rca << 16,
				csd, MMC_SEND_CSD);

	ret = mmc_send_cxd_data(card, card->host, MMC_SEND_CSD, csd, 16);
	if (ret)
		return ret;

	for (i = 0;i < 4;i++)
		csd[i] = be32_to_cpu(csd[i]);

	return 0;
}

int mmc_send_cid(struct mmc_host *host, u32 *cid)
{
	int ret, i;

	if (!mmc_host_is_spi(host)) {
		if (!host->card)
			return -EINVAL;
		return mmc_send_cxd_native(host, host->card->rca << 16,
				cid, MMC_SEND_CID);
	}

	ret = mmc_send_cxd_data(NULL, host, MMC_SEND_CID, cid, 16);
	if (ret)
		return ret;

	for (i = 0;i < 4;i++)
		cid[i] = be32_to_cpu(cid[i]);

	return 0;
}

int mmc_send_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	return mmc_send_cxd_data(card, card->host, MMC_SEND_EXT_CSD,
			ext_csd, 512);
}

int mmc_spi_read_ocr(struct mmc_host *host, int highcap, u32 *ocrp)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_READ_OCR;
	cmd.arg = highcap ? (1 << 30) : 0;
	cmd.flags = MMC_RSP_SPI_R3;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	*ocrp = cmd.resp[1];
	return err;
}

int mmc_spi_set_crc(struct mmc_host *host, int use_crc)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_CRC_ON_OFF;
	cmd.flags = MMC_RSP_SPI_R1;
	cmd.arg = use_crc;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (!err)
		host->use_spi_crc = use_crc;
	return err;
}

/**
 *	mmc_switch - modify EXT_CSD register
 *	@card: the MMC card associated with the data transfer
 *	@set: cmd set values
 *	@index: EXT_CSD register index
 *	@value: value to program into EXT_CSD register
 *	@timeout_ms: timeout (ms) for operation performed by register write,
 *                   timeout of zero implies maximum possible timeout
 *
 *	Modifies the EXT_CSD register for selected card.
 */
int mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
	       unsigned int timeout_ms)
{
	int err;
	struct mmc_command cmd = {0};
	u32 status;

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	cmd.cmd_timeout_ms = timeout_ms;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/* Must check status to be sure of no errors */
	do {
		err = mmc_send_status(card, &status);
		if (err)
			return err;
		if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;
		if (mmc_host_is_spi(card->host))
			break;
	} while (R1_CURRENT_STATE(status) == R1_STATE_PRG);

	if (mmc_host_is_spi(card->host)) {
		if (status & R1_SPI_ILLEGAL_COMMAND)
			return -EBADMSG;
	} else {
		if (status & 0xFDFFA000)
			printk(KERN_WARNING "%s: unexpected status %#x after "
			       "switch", mmc_hostname(card->host), status);
		if (status & R1_SWITCH_ERROR)
			return -EBADMSG;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_switch);

int mmc_send_status(struct mmc_card *card, u32 *status)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}

static int
mmc_send_bus_test(struct mmc_card *card, struct mmc_host *host, u8 opcode,
		  u8 len)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	u8 *data_buf;
	u8 *test_buf;
	int i, err;
	static u8 testdata_8bit[8] = { 0x55, 0xaa, 0, 0, 0, 0, 0, 0 };
	static u8 testdata_4bit[4] = { 0x5a, 0, 0, 0 };

	/* dma onto stack is unsafe/nonportable, but callers to this
	 * routine normally provide temporary on-stack buffers ...
	 */
	data_buf = kmalloc(len, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	if (len == 8)
		test_buf = testdata_8bit;
	else if (len == 4)
		test_buf = testdata_4bit;
	else {
		printk(KERN_ERR "%s: Invalid bus_width %d\n",
		       mmc_hostname(host), len);
		kfree(data_buf);
		return -EINVAL;
	}

	if (opcode == MMC_BUS_TEST_W)
		memcpy(data_buf, test_buf, len);

	mrq.cmd = &cmd;
	mrq.data = &data;
	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	if (opcode == MMC_BUS_TEST_R)
		data.flags = MMC_DATA_READ;
	else
		data.flags = MMC_DATA_WRITE;

	data.sg = &sg;
	data.sg_len = 1;
	sg_init_one(&sg, data_buf, len);
	mmc_wait_for_req(host, &mrq);
	err = 0;
	if (opcode == MMC_BUS_TEST_R) {
		for (i = 0; i < len / 4; i++)
			if ((test_buf[i] ^ data_buf[i]) != 0xff) {
				err = -EIO;
				break;
			}
	}
	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return err;
}

int mmc_bus_test(struct mmc_card *card, u8 bus_width)
{
	int err, width;

	if (bus_width == MMC_BUS_WIDTH_8)
		width = 8;
	else if (bus_width == MMC_BUS_WIDTH_4)
		width = 4;
	else if (bus_width == MMC_BUS_WIDTH_1)
		return 0; /* no need for test */
	else
		return -EINVAL;

	/*
	 * Ignore errors from BUS_TEST_W.  BUS_TEST_R will fail if there
	 * is a problem.  This improves chances that the test will work.
	 */
	mmc_send_bus_test(card, card->host, MMC_BUS_TEST_W, width);
	err = mmc_send_bus_test(card, card->host, MMC_BUS_TEST_R, width);
	return err;
}

int mmc_send_hpi_cmd(struct mmc_card *card, u32 *status)
{
	struct mmc_command cmd = {0};
	unsigned int opcode;
	unsigned int flags;
	int err;

	opcode = card->ext_csd.hpi_cmd;
	if (opcode == MMC_STOP_TRANSMISSION)
		flags = MMC_RSP_R1 | MMC_CMD_AC;
	else if (opcode == MMC_SEND_STATUS)
		flags = MMC_RSP_R1 | MMC_CMD_AC;

	cmd.opcode = opcode;
	cmd.arg = card->rca << 16 | 1;
	cmd.flags = flags;
	cmd.cmd_timeout_ms = card->ext_csd.out_of_int_time;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_warn("%s: error %d interrupting operation. "
			"HPI command response %#x\n", mmc_hostname(card->host),
			err, cmd.resp[0]);
		return err;
	}
	if (status)
		*status = cmd.resp[0];

	return 0;
}

static int mmc_rpmb_send_command(struct mmc_card *card, u8 *buf, __u16 blks,
		__u16 type, u8 req_type)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_command sbc = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	u8 *transfer_buf = NULL;

	mrq.sbc = &sbc;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;
	transfer_buf = kzalloc(512 * blks, GFP_KERNEL);
	if (!transfer_buf)
		return -ENOMEM;

	/*
	 * set CMD23
	 */
	sbc.opcode = MMC_SET_BLOCK_COUNT;
	sbc.arg = blks;
	if ((req_type == RPMB_REQ) && (type == RPMB_WRITE_DATA ||
			type == RPMB_PROGRAM_KEY))
		sbc.arg |= 1 << 31;
	sbc.flags = MMC_RSP_R1 | MMC_CMD_AC;

	/*
	 * set CMD25/18
	 */
	sg_init_one(&sg, transfer_buf, 512 * blks);
	if (req_type == RPMB_REQ) {
		cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		sg_copy_from_buffer(&sg, 1, buf, 512 * blks);
		data.flags |= MMC_DATA_WRITE;
	} else {
		cmd.opcode = MMC_READ_MULTIPLE_BLOCK;
		data.flags |= MMC_DATA_READ;
	}
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = blks;
	data.sg = &sg;
	data.sg_len = 1;

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (req_type != RPMB_REQ)
		sg_copy_to_buffer(&sg, 1, buf, 512 * blks);

	kfree(transfer_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;
	return 0;
}

void mmc_rpmb_post_frame(struct mmc_rpmb_req *p_req)
{
	int i;
	__u8 *ptr, *buf_frame = p_req->frame;

	if (!p_req->ready || !buf_frame)
		return;
	/*
	 * Regarding to the check rules, here is the post
	 * rules
	 * All will return result.
	 * GET_WRITE_COUNTER:
	 *		must: write counter, nonce
	 *		optional: MAC
	 * WRITE_DATA:
	 *		must: MAC, write counter
	 * READ_DATA:
	 *		must: nonce, data
	 *		optional: MAC
	 * PROGRAM_KEY:
	 *		must: Nothing
	 *
	 * Except READ_DATA, all of these operations only need to parse
	 * one frame. READ_DATA needs blks frames to get DATA
	 */

	memcpy(p_req->result, buf_frame + RPMB_RES_BEG, 2);
	*p_req->result = be16_to_cpup(p_req->result);

	if (p_req->type == RPMB_PROGRAM_KEY)
		goto out;

	if (p_req->type == RPMB_GET_WRITE_COUNTER ||
			p_req->type == RPMB_WRITE_DATA) {
		memcpy(p_req->wc, buf_frame + RPMB_WCOUNTER_BEG, 4);
		*p_req->wc = be32_to_cpup(p_req->wc);
	}

	if (p_req->type == RPMB_GET_WRITE_COUNTER ||
			p_req->type == RPMB_READ_DATA) {
		/* nonce copy */
		ptr = buf_frame + RPMB_NONCE_END;
		for (i = 0; i < 16; i++, ptr--)
			p_req->nonce[i] = *ptr;
	}
	/*
	 * Take MAC within the last package
	 */
	if (p_req->type == RPMB_READ_DATA) {
		int j;
		__u8 *data = p_req->data;
		for (i = 0; i < p_req->blk_cnt; i++) {
			ptr = buf_frame + i * 512 + RPMB_DATA_END;
			for (j = 0; j < 256; j++, ptr--, data++)
				*data = *ptr;
		}
		/*
		 * MAC stored in the last package
		 */
		if (p_req->mac) {
			ptr = buf_frame + 512 * i + RPMB_MAC_END;
			for (i = 0; i < 32; i++, ptr--)
				p_req->mac[i] = *ptr;
		}
	} else if (p_req->mac) {
		ptr = buf_frame + RPMB_MAC_END;
		for (i = 0; i < 32; i++, ptr--)
			p_req->mac[i] = *ptr;
	}
out:
	kfree(buf_frame);
	p_req->frame = NULL;
	return;
}
EXPORT_SYMBOL_GPL(mmc_rpmb_post_frame);

static int mmc_rpmb_request_check(struct mmc_card *card,
		struct mmc_rpmb_req *p_req)
{
	/*
	 * Some paramter is a must for the operation. Different
	 * operation expect different paramters. Below code is
	 * used for checking this.
	 *
	 * All operations will need result.
	 * GET_WRITE_COUNTER:
	 *		must: write counter, nonce
	 *		optional: MAC
	 * WRITE_DATA:
	 *		must: MAC, data, write counter
	 * READ_DATA:
	 *		must: nonce, data
	 *		optional: MAC
	 * PROGRAM_KEY:
	 *		must: MAC
	 *
	 * So here, we only check the 'must' paramters
	 */
	if (!p_req->result) {
		pr_err("%s: Type %d has NULL pointer for result\n",
				mmc_hostname(card->host), p_req->type);
		return -EINVAL;
	}

	if (p_req->type == RPMB_GET_WRITE_COUNTER) {
		if (!p_req->nonce || !p_req->wc) {
			pr_err("%s: Type %d has NULL pointer for nonce/wc\n",
					mmc_hostname(card->host), p_req->type);
			return -EINVAL;
		}
		p_req->blk_cnt = 1;
	} else if (p_req->type == RPMB_WRITE_DATA ||
			p_req->type == RPMB_READ_DATA) {
		if ((__u32)(p_req->addr + p_req->blk_cnt) >
				card->ext_csd.rpmb_size) {
			pr_err("%s Type %d: beyond the RPMB partition rang "
					"addr %d, blk_cnt %d, rpmb_size %d\n",
					mmc_hostname(card->host),
					p_req->type,
					p_req->addr,
					p_req->blk_cnt,
					card->ext_csd.rpmb_size);
			return -EINVAL;
		}
		if (!p_req->data) {
			pr_err("%s: Type %d has NULL pointer for data\n",
					mmc_hostname(card->host), p_req->type);
			return -EINVAL;
		}
		if (p_req->type == RPMB_WRITE_DATA) {
			if (!p_req->wc || !p_req->mac) {
				pr_err("%s: Type %d has NULL pointer for"
						" write counter/MAC\n",
						mmc_hostname(card->host),
						p_req->type);
				return -EINVAL;
			}
		} else {
			if (!p_req->nonce) {
				pr_err("%s: Type %d has NULL pointer for"
						" nonce\n",
						mmc_hostname(card->host),
						p_req->type);
				return -EINVAL;
			}
		}
	} else if (p_req->type == RPMB_PROGRAM_KEY) {
		if (!p_req->mac) {
			pr_err("%s: Type %d has NULL pointer for MAC\n",
					mmc_hostname(card->host), p_req->type);
			return -EINVAL;
		}
		p_req->blk_cnt = 1;
	} else
		return -EOPNOTSUPP;

	if (p_req->blk_cnt == 0) {
		pr_err("%s: Type %d has zero block count\n",
				mmc_hostname(card->host), p_req->blk_cnt);
		return -EINVAL;
	}

	if (p_req->blk_cnt > card->rpmb_max_req) {
		pr_err("%s: Type %d has invalid block count, "
				"cannot large than %d\n",
				mmc_hostname(card->host),
				p_req->blk_cnt,
				card->rpmb_max_req);
		return -EINVAL;
	}

	return 0;
}

/*
 * prepare the request of RPMB frame
 * RPMB frame is MSB first
 * convert needed bytes
 * return how many frames will be prepared
 */
int mmc_rpmb_pre_frame(struct mmc_rpmb_req *p_req,
		struct mmc_card *card)
{
	int i, j, ret;
	__u8 *ptr = NULL, *buf_frame;
	__u8 *data = p_req->data;
	__u16 blk_cnt, addr, type;
	__u32 w_counter;

	/*
	 * make sure these two items are clear
	 */
	p_req->ready = 0;
	p_req->frame = NULL;

	ret = mmc_rpmb_request_check(card, p_req);
	if (ret)
		return ret;

	buf_frame = kzalloc(512 * p_req->blk_cnt, GFP_KERNEL);
	if (!buf_frame) {
		pr_err("%s: cannot allocate frame for type %d\n",
				mmc_hostname(card->host), p_req->type);
		return -ENOMEM;
	}

	type = cpu_to_be16p(&p_req->type);
	if (p_req->type == RPMB_GET_WRITE_COUNTER ||
			p_req->type == RPMB_READ_DATA) {
		/*
		 * One package prepared
		 * This request needs Nonce and type
		 * If is data read, then also need addr
		 */
		memcpy(buf_frame + RPMB_TYPE_BEG, &type, 2);
		if (p_req->type == RPMB_READ_DATA) {
			addr = cpu_to_be16p(&p_req->addr);
			memcpy(buf_frame + RPMB_ADDR_BEG, &addr, 2);
		}
		/* convert Nonce code */
		ptr = buf_frame + RPMB_NONCE_END;
		for (i = 0; i < 16; i++, ptr--)
			*ptr = p_req->nonce[i];
	} else if (p_req->type == RPMB_WRITE_DATA) {
		/*
		 * multiple package prepared
		 * This request nees blk_cnt, addr, write_counter,
		 * data and mac
		 */
		blk_cnt = cpu_to_be16p(&p_req->blk_cnt);
		addr = cpu_to_be16p(&p_req->addr);
		w_counter = cpu_to_be32p(p_req->wc);
		for (i = 0; i < p_req->blk_cnt; i++) {
			memcpy(buf_frame + i * 512 + RPMB_TYPE_BEG,
					&type, 2);
			memcpy(buf_frame + i * 512 + RPMB_BLKS_BEG,
					&blk_cnt, 2);
			memcpy(buf_frame + i * 512 + RPMB_ADDR_BEG,
					&addr, 2);
			memcpy(buf_frame + i * 512 + RPMB_WCOUNTER_BEG,
					&w_counter, 4);
			ptr = buf_frame + i * 512 + RPMB_DATA_END;
			for (j = 0; j < 256; j++, ptr--, data++)
				*ptr = *data;
		}
		ptr = buf_frame + 512 * (i - 1) + RPMB_MAC_END;
		/* convert MAC code */
		for (i = 0; i < 32; i++, ptr--)
			*ptr = p_req->mac[i];
	} else if (p_req->type == RPMB_PROGRAM_KEY) {
		/*
		 * One package prepared
		 * This request only need mac
		 */
		memcpy(buf_frame + RPMB_TYPE_BEG, &type, 2);
		ptr = buf_frame + RPMB_MAC_END;
		/* convert MAC code */
		for (i = 0; i < 32; i++, ptr--)
			*ptr = p_req->mac[i];
	} else {
		pr_err("%s: We shouldn't be here\n", mmc_hostname(card->host));
		kfree(buf_frame);
		return -EINVAL;
	}
	p_req->ready = 1;
	p_req->frame = buf_frame;
	return 0;
}
EXPORT_SYMBOL_GPL(mmc_rpmb_pre_frame);

int mmc_rpmb_partition_ops(struct mmc_rpmb_req *p_req,
		struct mmc_card *card)
{
	int err = 0;
	__u16 type, blks;
	__u8 *buf_frame = p_req->frame;

	if (!p_req->ready || !buf_frame) {
		pr_err("%s: mmc_rpmb_req is not prepared\n",
				mmc_hostname(card->host));
		return -EINVAL;
	}

	type = p_req->type;
	blks = p_req->blk_cnt;

	/*
	 * STEP 1: send request to RPMB partition
	 */
	if (type == RPMB_READ_DATA)
		err = mmc_rpmb_send_command(card, buf_frame, 1, type, RPMB_REQ);
	else
		err = mmc_rpmb_send_command(card, buf_frame,
				blks, type, RPMB_REQ);
	if (err) {
		pr_err("%s: request write counter failed (%d)\n",
			mmc_hostname(card->host), err);
		goto out;
	}

	memset(buf_frame, 0, 512 * blks);
	/*
	 * STEP 2: check write result
	 * Only for WRITE_DATA or Program key
	 */
	if (type == RPMB_WRITE_DATA ||
			type == RPMB_PROGRAM_KEY) {
		buf_frame[RPMB_TYPE_BEG + 1] = RPMB_RESULT_READ;
		err = mmc_rpmb_send_command(card, buf_frame, 1,
				RPMB_RESULT_READ, RPMB_REQ);
		if (err) {
			pr_err("%s: request write counter failed (%d)\n",
				mmc_hostname(card->host), err);
			goto out;
		}
	}

	/*
	 * STEP 3: get response from RPMB partition
	 */

	if (type == RPMB_READ_DATA)
		err = mmc_rpmb_send_command(card, buf_frame,
				blks, type, RPMB_RESP);
	else
		err = mmc_rpmb_send_command(card, buf_frame,
				1, type, RPMB_RESP);
	if (err) {
		pr_err("%s: response write counter failed (%d)\n",
			mmc_hostname(card->host), err);
	}
out:
	return err;
}
EXPORT_SYMBOL_GPL(mmc_rpmb_partition_ops);
