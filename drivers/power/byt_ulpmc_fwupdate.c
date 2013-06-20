/*
 * Intel Baytrail ULPMC FW Update driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/miscdevice.h>
#include <linux/atomic.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/power/byt_ulpmc_battery.h>

#define DRIVER_NAME	"ulpmc-fwupdate"

#define BSL_MESSAGE_CODE		0x3B

#define CMD_ENTER_BSL_MODE		0xF0
#define CMD_RX_DATA_BLOCK		0x10
#define CMD_RX_DATA_BLOCK_FAST		0x1B
#define CMD_RX_PASSWORD			0x11
#define CMD_ERASE_SEGMENT		0x12
#define CMD_UNLOCK_LOCK_INFO		0x13
#define CMD_RESERVED			0x14
#define CMD_MASS_ERASE			0x15
#define CMD_CRC_CHECK			0x16
#define CMD_LOAD_PC			0x17
#define CMD_TX_DATA_BLOCK		0x18
#define CMD_TX_BSL_VERSION		0x19
#define CMD_TX_BUFFER_SIZE		0x1A

#define ULPMC_CMD_HEADER		0x80
#define ULPMC_RST_PMMCTL0_REG		0x120
#define PMMCTL0_REG_VAL_LSB		0x04
#define PMMCTL0_REG_VAL_MSB		0xA5

#define MASS_ERASE_RESPONSE_LENGTH		(6 + 2)
#define CRC_CHECK_RESPONSE_LENGTH		(6 + 3)
#define TX_BSL_VERSION_RESPONSE_LENGTH		(6 + 5)
#define TX_BUFFER_SIZE_RESPONSE_LENGTH		(6 + 3)
#define LOAD_PC_RESPONSE_LENGTH			(6 + 2)
#define RX_DATA_BLOCK_RESPONSE_LENGTH		(6 + 2)
#define RX_PASSWORD_RESPONSE_LENGTH		(6 + 2)
#define ERASE_SEGMENT_RESPONSE_LENGTH		(6 + 2)
#define UNLOCK_LOCK_INFO_RESPONSE_LENGTH	(6 + 2)

#define ULPMC_SEGMENT_OFFSET_B			0x1900
#define ULPMC_SEGMENT_OFFSET_C			0x1880
#define ULPMC_SEGMENT_OFFSET_D			0x1800

#define TX_DATA_BLOCK_RESPONSE_LENGTH		6	/* variable length */

#define RX_TX_BUFFER_LEN			256
#define TX_DATA_BUFFER_SIZE			240
#define INPUT_DATA_LEN				32
#define TEMP_BUFFER_LEN				8

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT	3

static u8 input_data[INPUT_DATA_LEN];
static u8 *file_buf;
static u32 fdata_len;
static u32 fcur_idx;
static u32 fcur_addr;

static struct platform_device *pdev;

struct ulpmc_fwu_info {
	struct platform_device *pdev;
	struct i2c_client	*client;
	struct notifier_block	nb;

	atomic_t		fcount;
	u16			crc;
	struct miscdevice	ulpmc_mdev;
};

static struct ulpmc_fwu_info *fwu_ptr;

static void crc_init(struct ulpmc_fwu_info *fwu)
{
	fwu->crc = 0xffff;
}

static void crc_ccitt_update(struct ulpmc_fwu_info *fwu, u8 x)
{
	u16 crc_new =
			(fwu->crc >> 8) | (fwu->crc << 8);
	crc_new ^= x;
	crc_new ^= (crc_new & 0xff) >> 4;
	crc_new ^= crc_new << 12;
	crc_new ^= (crc_new & 0xff) << 5;
	fwu->crc = crc_new;
}

static u16 crc_ccitt_crc(struct ulpmc_fwu_info *fwu)
{
	return fwu->crc;
}

static int do_i2c_transfer(struct ulpmc_fwu_info *fwu,
				struct i2c_msg *msg, int num)
{
	int ret, nc;

	for (nc = 0; nc < NR_RETRY_CNT; nc++) {
		ret = i2c_transfer(fwu->client->adapter, msg, num);
		if (ret < 0)
			continue;
		else
			break;
	}

	return ret;
}

static int fill_BSLCmd_packet(struct ulpmc_fwu_info *fwu, u8 cmd, u32 addr,
				u8 *data, u16 size, u8 *buf, u16 *buflen)
{
	u16 len = 1, chksum, widx = 0;
	u32 i;

	if ((cmd == 0) && (addr == 0)) {
		dev_err(&fwu->pdev->dev, "error:%s\n", __func__);
		return -EINVAL;
	}

	if (cmd == CMD_RX_DATA_BLOCK ||
		cmd == CMD_RX_DATA_BLOCK_FAST ||
		cmd == CMD_ERASE_SEGMENT ||
		cmd == CMD_CRC_CHECK ||
		cmd == CMD_LOAD_PC ||
		cmd == CMD_TX_DATA_BLOCK) {
		/* adding Size of aata and address size(3 Bytes) */
		len += size + 3;
	} else {
		/* adding the size of the data */
		len += size;
	}

	buf[widx++] = ULPMC_CMD_HEADER;
	buf[widx++] = len & 0xFF;
	buf[widx++] = (len >> 8) & 0xFF;
	/* initialize CRC here */
	crc_init(fwu);
	buf[widx++] = cmd;
	crc_ccitt_update(fwu, cmd);
	if (addr != 0) {
		buf[widx++] = addr & 0xFF;
		crc_ccitt_update(fwu, (u8)(addr & 0xFF));
		buf[widx++] = (addr >> 8) & 0xFF;
		crc_ccitt_update(fwu, (u8)((addr >> 8) & 0xFF));
		buf[widx++] = (addr >> 16) & 0xFF;
		crc_ccitt_update(fwu, (u8)((addr >> 16) & 0xFF));
	}
	for (i = 0; i < size; i++) {
		buf[widx++] = data[i];
		crc_ccitt_update(fwu, data[i]);
	}
	chksum = crc_ccitt_crc(fwu);
	buf[widx++] = chksum & 0xFF;
	buf[widx++] = (chksum >> 8) & 0xFF;

	*buflen = widx;
	return 0;
}

static int recieve_BSLResponse(struct ulpmc_fwu_info *fwu, u16 read_len,
				u8 *data, u16 *len)
{
	int ret;
	u8 res_buf[RX_TX_BUFFER_LEN];
	struct i2c_msg msg;
	u16 i, res_len, crcval, rcvd_crcval;

	mdelay(100);

	memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
	/* send mass erase cmd */
	msg.addr = fwu->client->addr;
	msg.flags = I2C_M_RD;
	msg.buf = res_buf;
	msg.len = read_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "i2c tx failed:%d\n", ret);
		return ret;
	}

	if ((res_buf[0] != 0x0) || (res_buf[1] != ULPMC_CMD_HEADER)) {
		dev_err(&fwu->pdev->dev, "BSL Response failed:[0]:%x,[1]:%x\n",
							res_buf[0], res_buf[1]);
		goto bsl_response_failed;
	}

	res_len = (u16)res_buf[2] | ((u16)res_buf[3] << 8);
	*len = res_len;

	if (res_len > read_len) {
		dev_err(&fwu->pdev->dev, "Len mismatch:read_len%x,res_len:%x\n",
							read_len, res_len);
		goto bsl_response_failed;
	}

	crc_init(fwu);
	for (i = 0; i < res_len; i++) {
		data[i] = res_buf[4 + i];
		crc_ccitt_update(fwu, data[i]);
	}
	crcval = crc_ccitt_crc(fwu);
	rcvd_crcval = (u16)res_buf[4 + res_len] |
			(u16)(res_buf[4 + res_len + 1] << 8);

	if (crcval != rcvd_crcval) {
		dev_err(&fwu->pdev->dev, "BSL Response CRC chk failed\n");
		goto bsl_response_failed;
	}

	if ((data[0] == BSL_MESSAGE_CODE) && (data[1] != 0x0)) {
		dev_err(&fwu->pdev->dev, "BSL Core response error\n");
		goto bsl_response_failed;
	}

	return 0;

bsl_response_failed:
	return -EIO;
}

static int enter_bsl_mode(struct  ulpmc_fwu_info *fwu)
{
	int ret;
	u8 wbuf[TEMP_BUFFER_LEN];
	u16 idx = 0;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	memset(wbuf, 0x0, TEMP_BUFFER_LEN);
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	wbuf[idx] = CMD_ENTER_BSL_MODE;
	msg.buf = &wbuf[idx];
	msg.len = 1;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0)
		dev_err(&fwu->pdev->dev, "i2c tx failed\n");

	/* start up delay */
	mdelay(250);

	return ret;
}

static int do_mass_erase(struct  ulpmc_fwu_info *fwu)
{
	int ret;
	u8 cmd_buf[RX_TX_BUFFER_LEN], res_buf[RX_TX_BUFFER_LEN];
	u16 cmd_len, res_len, read_len;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	mdelay(100);

	memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
	ret = fill_BSLCmd_packet(fwu, CMD_MASS_ERASE, 0, NULL, 0,
						cmd_buf, &cmd_len);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
		return ret;
	}
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = cmd_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "i2c tx failed:%d\n", ret);
		return ret;
	}

	/* read response */
	read_len = MASS_ERASE_RESPONSE_LENGTH;
	ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);

	return ret;
}

static int check_password(struct  ulpmc_fwu_info *fwu)
{
	int ret;
	u8 cmd_buf[RX_TX_BUFFER_LEN], res_buf[RX_TX_BUFFER_LEN];
	u16 cmd_len, res_len, read_len;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	mdelay(100);

	memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(input_data, 0xFF, INPUT_DATA_LEN);
	ret = fill_BSLCmd_packet(fwu, CMD_RX_PASSWORD, 0, input_data,
					INPUT_DATA_LEN, cmd_buf, &cmd_len);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
		return ret;
	}
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = cmd_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "i2c tx failed\n");
		return ret;
	}

	/* read response */
	read_len = RX_PASSWORD_RESPONSE_LENGTH;
	ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);

	return ret;
}

static int erase_segment(struct  ulpmc_fwu_info *fwu, u32 seg_offset)
{
	int ret;
	u8 cmd_buf[RX_TX_BUFFER_LEN], res_buf[RX_TX_BUFFER_LEN];
	u16 cmd_len, res_len, read_len;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	mdelay(100);

	memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(input_data, 0x0, INPUT_DATA_LEN);
	ret = fill_BSLCmd_packet(fwu, CMD_ERASE_SEGMENT, seg_offset,
				input_data, 0x2, cmd_buf, &cmd_len);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
		return ret;
	}
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = cmd_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "i2c tx failed\n");
		return ret;
	}

	/* read response */
	read_len = ERASE_SEGMENT_RESPONSE_LENGTH;
	ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);

	return ret;
}

static int check_bsl_version(struct  ulpmc_fwu_info *fwu)
{
	int ret;
	u8 cmd_buf[RX_TX_BUFFER_LEN], res_buf[RX_TX_BUFFER_LEN];
	u16 cmd_len, res_len, read_len;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	mdelay(100);

	memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
	memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
	ret = fill_BSLCmd_packet(fwu, CMD_TX_BSL_VERSION, 0, NULL, 0,
						cmd_buf, &cmd_len);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
		return ret;
	}
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = cmd_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "i2c tx failed\n");
		return ret;
	}

	/* read response */
	read_len = TX_BSL_VERSION_RESPONSE_LENGTH;
	ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);

	return ret;
}

static int parse_udata(struct ulpmc_fwu_info *fwu,
				u32 *addr, u8 *data, u16 *len)
{
	u16 i, j;
	u8 temp_buf[TEMP_BUFFER_LEN];
	u32 temp_addr = 0, temp_data = 0;
	u16 cur_datalen = 0;
	bool addr_flag;
	long val;

	if (fcur_idx >= fdata_len)
		goto len_mismatch_err;

	if (file_buf[fcur_idx] == '@')
		addr_flag = true;
	else
		addr_flag = false;

	for (i = fcur_idx;
		(i < fdata_len) && (cur_datalen < TX_DATA_BUFFER_SIZE); i++) {
		if (file_buf[i] == '@') {
			if (i != fcur_idx)
				break;
			i++;
			for (j = 0; j < TEMP_BUFFER_LEN; i++) {
				if (i >= fdata_len)
					goto len_mismatch_err;

				if (isxdigit(file_buf[i])) {
					temp_buf[j++] = file_buf[i];
				} else if (file_buf[i] == '\n') {
					temp_buf[j] = '\0';
					if (kstrtol(temp_buf, 16, &val))
						goto str_conv_failed;
					temp_addr = (u32)val;
					break;
				}
			}
		} else {

			if (i >= fdata_len ||
				cur_datalen >= TX_DATA_BUFFER_SIZE)
				break;

			if (isxdigit(file_buf[i])) {
				temp_buf[0] = file_buf[i++];
				temp_buf[1] = file_buf[i];
				temp_buf[2] = '\0';

				if (kstrtol(temp_buf, 16, &val))
					goto str_conv_failed;
				temp_data = (u32)val;
				data[cur_datalen++] = (u8)temp_data;
			}
		}
	}

	fcur_idx = i;
	*len = cur_datalen;

	if (addr_flag) {
		*addr = temp_addr;
		fcur_addr = *addr;
	} else {
		*addr = 0;
	}

	return 0;

str_conv_failed:
	dev_err(&fwu->pdev->dev, "kstrtol error:%s:\n", temp_buf);
	return -EINVAL;

len_mismatch_err:
	dev_err(&fwu->pdev->dev, "len mismatch:%s\n", __func__);
	return -EINVAL;
}

static int ulpmc_SW_por_reset(struct ulpmc_fwu_info *fwu)
{
	int ret;
	u8 cmd_buf[RX_TX_BUFFER_LEN], data[RX_TX_BUFFER_LEN];
	u16 cmd_len, data_len = 0;
	u32 dst_addr;
	struct i2c_msg msg;

	mdelay(100);

	memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
	dst_addr = ULPMC_RST_PMMCTL0_REG;
	data[data_len++] = PMMCTL0_REG_VAL_LSB;
	data[data_len++] = PMMCTL0_REG_VAL_MSB;

	ret = fill_BSLCmd_packet(fwu, CMD_RX_DATA_BLOCK, dst_addr,
				data, data_len, cmd_buf, &cmd_len);
	if (ret < 0) {
		dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
		return ret;
	}
	msg.addr = fwu->client->addr;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = cmd_len;
	ret = do_i2c_transfer(fwu, &msg, 1);
	if (ret < 0)
		dev_err(&fwu->pdev->dev, "i2c tx failed\n");

	return ret;
}

static int updateFW(struct ulpmc_fwu_info *fwu, u8 *udata, u32 ulen)
{
	int ret, i;
	u8 cmd_buf[RX_TX_BUFFER_LEN], res_buf[RX_TX_BUFFER_LEN];
	u16 cmd_len, res_len, read_len;
	u16 crcval, rcvd_crcval;
	u32 src_addr = 0, dst_addr;
	u8 data[TX_DATA_BUFFER_SIZE];
	u16 data_len, length;
	struct i2c_msg msg;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	while (fcur_idx < fdata_len) {
		ret = parse_udata(fwu, &src_addr, data, &length);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "parse data error\n");
			goto fw_update_failed;
		}

		if (src_addr != 0) {
			dst_addr = src_addr;
		} else if (src_addr == 0) {
			fcur_addr = fcur_addr + 0xF0;
			dst_addr = fcur_addr;
		}

		memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
		memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
		cmd_len = 0;
		res_len = 0;
		data_len = length;

		ret = fill_BSLCmd_packet(fwu, CMD_RX_DATA_BLOCK, dst_addr,
					data, data_len, cmd_buf, &cmd_len);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
			return ret;
		}

		msg.addr = fwu->client->addr;
		msg.flags = 0;
		msg.buf = cmd_buf;
		msg.len = cmd_len;
		ret = do_i2c_transfer(fwu, &msg, 1);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "i2c tx failed\n");
			goto fw_update_failed;
		}

		/* read response */
		read_len = RX_DATA_BLOCK_RESPONSE_LENGTH;
		ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "read response error\n");
			goto fw_update_failed;
		}

		crc_init(fwu);
		/* calculate checksum of the data and clear the data */
		for (i = 0; i < data_len; i++) {
			crc_ccitt_update(fwu, data[i]);
			data[i] = 0x0;
		}
		crcval = crc_ccitt_crc(fwu);

		/* crc check */
		mdelay(100);
		memset(cmd_buf, 0x0, RX_TX_BUFFER_LEN);
		memset(res_buf, 0x0, RX_TX_BUFFER_LEN);
		cmd_len = 0;
		res_len = 0;
		data[0] = data_len & 0xFF;
		data[1] = (data_len >> 8) & 0xFF;
		data_len = 2;

		ret = fill_BSLCmd_packet(fwu, CMD_CRC_CHECK, dst_addr,
					data, data_len, cmd_buf, &cmd_len);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "bsl failed:%d\n", ret);
			return ret;
		}

		msg.addr = fwu->client->addr;
		msg.flags = 0;
		msg.buf = cmd_buf;
		msg.len = cmd_len;
		ret = do_i2c_transfer(fwu, &msg, 1);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "i2c tx failed\n");
			goto fw_update_failed;
		}

		/* read response */
		read_len = CRC_CHECK_RESPONSE_LENGTH;
		ret = recieve_BSLResponse(fwu, read_len, res_buf, &res_len);
		if (ret < 0) {
			dev_err(&fwu->pdev->dev, "read response error\n");
			goto fw_update_failed;
		}

		rcvd_crcval = (u16)res_buf[1] | ((u16)res_buf[2] << 8);
		if (crcval != rcvd_crcval) {
			dev_err(&fwu->pdev->dev, "crc match failed\n");
			ret = -EINVAL;
			goto fw_update_failed;
		}

		mdelay(10);
	}

	ret = ulpmc_SW_por_reset(fwu);
	if (ret < 0)
		dev_err(&fwu->pdev->dev, "SW POR failed\n");

	/* add start up delay */
	mdelay(250);

fw_update_failed:
	return ret;
}

static int start_fw_update(struct  ulpmc_fwu_info *fwu, u8 *data, u32 data_len)
{
	int ret;

	dev_info(&fwu->pdev->dev, ":%s\n", __func__);

	/* enter BSL mode */
	ret = enter_bsl_mode(fwu);
	if (ret < 0)
		goto fwupdate_fail;

	/* do mass erase */
	ret = do_mass_erase(fwu);
	if (ret < 0)
		goto fwupdate_fail;

	/* check password */
	ret = check_password(fwu);
	if (ret < 0)
		goto fwupdate_fail;

	/* erase segmnet B */
	ret = erase_segment(fwu, ULPMC_SEGMENT_OFFSET_B);
	if (ret < 0)
		goto fwupdate_fail;

	/* erase segmnet C */
	ret = erase_segment(fwu, ULPMC_SEGMENT_OFFSET_C);
	if (ret < 0)
		goto fwupdate_fail;

	/* erase segmnet D */
	ret = erase_segment(fwu, ULPMC_SEGMENT_OFFSET_D);
	if (ret < 0)
		goto fwupdate_fail;

	/* check BSL version */
	ret = check_bsl_version(fwu);
	if (ret < 0)
		goto fwupdate_fail;

	/* update firmware */
	ret = updateFW(fwu, data, data_len);
	if (ret < 0)
		goto fwupdate_fail;

	dev_info(&fwu->pdev->dev, "fw update successful\n");
	return 0;

fwupdate_fail:
	dev_err(&fwu->pdev->dev, "fw update failed\n");
	return ret;
}

static int dev_file_open(struct inode *i, struct file *f)
{
	struct	ulpmc_fwu_info *fwu = fwu_ptr;

	if (atomic_read(&fwu->fcount))
		return -EBUSY;

	atomic_inc(&fwu->fcount);
	return 0;
}

static int dev_file_close(struct inode *i, struct file *f)
{
	struct	ulpmc_fwu_info *fwu = fwu_ptr;

	atomic_dec(&fwu->fcount);
	return 0;
}

static ssize_t dev_file_write(struct file *f, const char __user *buf,
			size_t len, loff_t *off)
{
	struct	ulpmc_fwu_info *fwu = fwu_ptr;
	u8	*udata;
	int	ret = 0;

	if (len <= 0)
		return -EINVAL;

	dev_info(&fwu->pdev->dev, "writing %d bytes..\n", len);

	udata = kzalloc(len, GFP_KERNEL);
	if (!udata) {
		dev_err(&fwu->pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	/* copy the data to kernel memory */
	if (copy_from_user(udata, buf, len)) {
		dev_err(&fwu->pdev->dev, "failed to copy usr data\n");
		ret = -EFAULT;
		goto fw_setup_failed;
	}

	/* set up the update process */
	ulpmc_fwupdate_enter();
	fwu->client = ulpmc_get_i2c_client();
	if (!fwu->client) {
		dev_err(&fwu->pdev->dev, "failed to get i2c client\n");
		ret = -EINVAL;
		goto fw_setup_failed;
	}

	/* init global data */
	file_buf = udata;
	fdata_len = len;
	fcur_idx = 0;
	fcur_addr = 0;

	/* start the fw update procedure */
	ret = start_fw_update(fwu, udata, len);
	ulpmc_fwupdate_exit();

fw_setup_failed:
	kfree(udata);
	if (ret >= 0)
		ret = len;
	return ret;
}

static const struct file_operations ulpmc_fops = {
	.owner = THIS_MODULE,
	.open = &dev_file_open,
	.release = &dev_file_close,
	.write = &dev_file_write,
};

static int ulpmc_fwupdate_probe(struct platform_device *pdev)
{
	struct ulpmc_fwu_info *fwu;
	int ret = 0;

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	fwu->pdev = pdev;
	platform_set_drvdata(pdev, fwu);
	fwu_ptr = fwu;

	fwu->ulpmc_mdev.name = DRIVER_NAME;
	fwu->ulpmc_mdev.fops = &ulpmc_fops;
	fwu->ulpmc_mdev.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&fwu->ulpmc_mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "msic dev register failed\n");
		kfree(fwu);
		return ret;
	}

	dev_info(&pdev->dev, "probe done:\n");
	return 0;
}

static int ulpmc_fwupdate_remove(struct platform_device *pdev)
{
	struct ulpmc_fwu_info *fwu = dev_get_drvdata(&pdev->dev);

	misc_deregister(&fwu->ulpmc_mdev);
	kfree(fwu);
	return 0;
}

static struct platform_driver ulpmc_fwupdate_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ulpmc_fwupdate_probe,
	.remove = ulpmc_fwupdate_remove,
};

static int __init ulpmc_fwu_init(void)
{
	int err;

	err = platform_driver_register(&ulpmc_fwupdate_driver);
	if (err < 0) {
		err = -ENOMEM;
		pr_err("Driver registartion failed\n");
		goto exit;
	}

	pdev = platform_device_alloc(DRIVER_NAME, 0);
	if (!pdev) {
		err = -ENOMEM;
		pr_err("Device allocation failed\n");
		goto exit;
	}

	err = platform_device_add(pdev);
	if (err) {
		pr_err("Device addition failed (%d)\n", err);
		goto exit;
	}
exit:
	return err;
}
fs_initcall(ulpmc_fwu_init);

static void __exit ulpmc_fwu_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&ulpmc_fwupdate_driver);
}
module_exit(ulpmc_fwu_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("BYT ULPMC FW Update driver");
MODULE_LICENSE("GPL");
