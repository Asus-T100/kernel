/*-------------------------------------------------------------------------
 * INTEL CONFIDENTIAL
 *
 * Copyright 2012 Intel Corporation All Rights Reserved.
 *
 * This source code and all documentation related to the source code
 * ("Material") contains trade secrets and proprietary and confidential
 * information of Intel and its suppliers and licensors. The Material is
 * deemed highly confidential, and is protected by worldwide copyright and
 * trade secret laws and treaty provisions. No part of the Material may be
 * used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No license under any patent, copyright, trade secret or other
 * intellectual property right is granted to or conferred upon you by
 * disclosure or delivery of the Materials, either expressly, by
 * implication, inducement, estoppel or otherwise. Any license under such
 * intellectual property rights must be express and approved by Intel in
 * writing.
 *-------------------------------------------------------------------------
 */

/*
 * Author: Wang Dong, Alek Du, Even Xu
 * Group: PSI, MCG
 */


#include <linux/ctype.h>
#include "psh_ia_common.h"

#define TOLOWER(x) ((x) | 0x20)
/* translate string to unsigned long value */
unsigned long _strtoul(const char *str, char **endp, unsigned int base)
{
	unsigned long value = 0;

	if (base == 0) {
		if (str[0] == '0') {
			if (TOLOWER(str[1]) == 'x' && isxdigit(str[2]))
				base = 16;
			else
				base = 8;
		} else
			base = 10;
	}

	if (base == 16 && str[0] == '0' && TOLOWER(str[1]) == 'x')
		str += 2;

	while (isxdigit(*str)) {
		unsigned int temp_value;

		if ('0' <= *str && *str <= '9')
			temp_value = *str - '0';
		else
			temp_value = TOLOWER(*str) - 'a' + 10;

		if (temp_value >= base)
			break;

		value = value * base + temp_value;
		str++;
	}

	if (endp)
		*endp = (char *)str;

	return value;
}

long trans_strtol(const char *str, char **endp, unsigned int base)
{
	if (*str == '-')
		return -_strtoul(str + 1, endp, base);

	return _strtoul(str, endp, base);
}

struct psh_ia_priv *psh_ia_data;

char *phy_sensor_name[] = {
	"ACCEL",
	"GYRO ",
	"COMPS",
	"BARO ",
	"ALS_P",
	"PS_P ",
	"TCPHY",
};

char *abs_sensor_name[] = {
	"COMAG",
	"TERMC",
	"GSSPT",
	"PHYAC",
	"9DOF ",
	"BIST ",
	"GSFLK",
	"GRAVI",
	"ORIEN",
	"LACCL",
	"RVECT",
	"COMPC",
	"GYROC",
	"PEDOM",
	"MAGHD",
};

char *port_name[] = {
	"CSPRT",
	"GSPRT",
	"EVPRT",
};

const char *ia_get_sensor_name(u8 id)
{
	if (id == 0)
		return "_PSH_";
	else if (id > 0 && id <= sizeof(phy_sensor_name))
		return phy_sensor_name[id - 1];
	else if (id > 100 && id <= (100 + sizeof(abs_sensor_name)))
		return abs_sensor_name[id - 101];
	else if (id > 200 && id <= (200 + sizeof(port_name)))
		return port_name[id - 201];
	else
		return "?????";
}

void ia_lbuf_read_init(struct loop_buffer *lbuf,
		u8 *buf, u16 size, update_finished_f uf)
{
	lbuf->addr = buf;
	lbuf->length = size;
	lbuf->off_head = lbuf->off_tail = 0;
	lbuf->update_finished = uf;
	lbuf->in_reading = 0;
}

void ia_lbuf_read_reset(struct loop_buffer *lbuf)
{
	lbuf->off_head = lbuf->off_tail = 0;
	lbuf->in_reading = 0;
}

int ia_lbuf_read_next(struct loop_buffer *lbuf,
		u8 **buf, u16 *size)
{
	struct frame_head *fhead =
			(struct frame_head *)(lbuf->addr + lbuf->off_head);

	*buf = NULL;
	*size = 0;

	if (lbuf->in_reading) {
		lbuf->in_reading = 0;

		/* go over previous frame has been read */
		lbuf->off_head += frame_size(fhead->length);
		lbuf->off_tail = lbuf->off_head;
		fhead = (struct frame_head *)(lbuf->addr + lbuf->off_head);
	}

	if (fhead->sign == LBUF_DISCARD_SIGN) {
		fhead = (struct frame_head *)lbuf->addr;
		lbuf->off_head = lbuf->off_tail = 0;
	}

	if (fhead->sign == LBUF_CELL_SIGN) {
		if (fhead->length > LBUF_MAX_CELL_SIZE)
			goto f_out;

		*buf = lbuf->addr + lbuf->off_head + sizeof(*fhead);
		*size = fhead->length;
		lbuf->in_reading = 1;
	}

f_out:
	if (!lbuf->in_reading) {
		/* no more data frame, inform FW to update its HEAD */
		lbuf->update_finished(lbuf->off_head);
	}

	return !lbuf->in_reading;
}

void ia_circ_reset_off(struct circ_buf *circ)
{
	circ->head = 0;
	circ->tail = 0;
}

void ia_circ_put_data(struct circ_buf *circ, const char *buf, u32 size)
{
	int tail_size, cnt;

	if (CIRC_SPACE(circ->head, circ->tail, CIRC_SIZE) < size)
		return;
	tail_size = CIRC_SPACE_TO_END(circ->head, circ->tail, CIRC_SIZE);
	cnt = size;
	if (cnt > tail_size)
		cnt = tail_size;
	memcpy(circ->buf + circ->head, buf, cnt);
	cnt = size - cnt;
	if (cnt)
		memcpy(circ->buf, buf + tail_size, cnt);
	circ->head += size;
	circ->head &= (CIRC_SIZE - 1);
}

int ia_circ_get_data(struct circ_buf *circ, char *buf, u32 size)
{
	int avail, avail_tail, cnt;

	avail = CIRC_CNT(circ->head, circ->tail, CIRC_SIZE);
	if (!avail)
		return 0;
	avail_tail = CIRC_CNT_TO_END(circ->head, circ->tail, CIRC_SIZE);
	if (avail_tail) {
		cnt = size;
		if (cnt > avail_tail)
			cnt = avail_tail;
		memcpy(buf, circ->buf + circ->tail, cnt);
		size -= cnt;
		avail -= cnt;
		circ->tail += cnt;
		if (!avail || !size)
			return cnt;
	}
	cnt = size;
	if (cnt > avail)
		cnt = avail;
	memcpy(buf + avail_tail, circ->buf, cnt);
	circ->tail += cnt;
	circ->tail &= (CIRC_SIZE - 1);
	return avail_tail + cnt;
}

int ia_send_cmd(int ch, struct ia_cmd *cmd, int len)
{
	int ret;

	mutex_lock(&psh_ia_data->cmd_mutex);
	ret = process_send_cmd(ch, cmd, len);
	mutex_unlock(&psh_ia_data->cmd_mutex);
	if (ret)
		return ret;

	if (cmd->cmd_id == CMD_RESET) {
		if (psh_ia_data->reset_in_progress) {
			wait_for_completion(&psh_ia_data->cmd_reset_comp);
			psh_ia_data->reset_in_progress = 0;
		}

		ia_lbuf_read_reset(&psh_ia_data->lbuf);
		ia_circ_reset_off(&psh_ia_data->circ);
	} else if (cmd->cmd_id == CMD_GET_STATUS)
		wait_for_completion(&psh_ia_data->get_status_comp);

	return 0;
}

int ia_update_finished(u16 offset)
{
	struct ia_cmd cmd_buf = {
		.cmd_id = CMD_UPDATE_DDR,
		.sensor_id = 0,
	};

	*(u16 *)cmd_buf.param = offset;
	return ia_send_cmd(PSH2IA_CHANNEL0, &cmd_buf, 7);
}

ssize_t ia_start_control(struct device *dev,
			struct device_attribute *attr,
			const char *str, size_t count)
{
	struct ia_cmd cmd_user = { 0 };
	u8 *ptr = (u8 *)&cmd_user;
	char *s;
	long val;
	int token = 0;
	int ret;

	while (*str && (token < sizeof(cmd_user))) {
		val = trans_strtol(str, &s, 0);
		if (str == s) {
			str++;
			continue;
		}
		str = s;
		*ptr++ = (u8)val;
		token++;
	}

	if (cmd_user.cmd_id == CMD_SETUP_DDR) {
		ret = do_setup_ddr(dev);
		if (ret) {
			pr_err("do_setup_ddr failed\n");
			return ret;
		} else {
			ia_lbuf_read_reset(&psh_ia_data->lbuf);
			return count;
		}
	}

	ret = ia_send_cmd(PSH2IA_CHANNEL0, &cmd_user, token);
	if (ret) {
		pr_err("send cmd failed\n");
		return ret;
	} else
		return count;
}

ssize_t ia_read_data_size(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int avail = CIRC_CNT(psh_ia_data->circ.head,
				psh_ia_data->circ.tail, CIRC_SIZE);

	return snprintf(buf, PAGE_SIZE, "%d\n", avail);
}

ssize_t ia_read_data(struct file *file, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	return ia_circ_get_data(&psh_ia_data->circ, buf, count);
}

ssize_t ia_read_debug_data(struct file *file, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	return ia_circ_get_data(&psh_ia_data->circ_dbg, buf, count);
}

ssize_t ia_set_dbg_mask(struct device *dev,
			struct device_attribute *attr,
			const char *str, size_t count)
{
	struct ia_cmd cmd;
	struct cmd_debug_param *param = (struct cmd_debug_param *)cmd.param;
	int token = 0;
	char *s;
	long val;

	while (*str) {
		val = trans_strtol(str, &s, 0);
		if (str == s) {
			str++;
			continue;
		}
		switch (token) {
		case 0:
			param->mask_out = val;
			break;
		case 1:
			param->mask_level = val;
			break;
		default:
			break;
		}
		str = s;
		if (++token == 2)
			break;
	}

	if (token == 2) {
		int ret;
		cmd.cmd_id = CMD_DEBUG;
		cmd.sensor_id = 0;
		param->sub_cmd = SCMD_DEBUG_SET_MASK;
		ret = ia_send_cmd(PSH2IA_CHANNEL0, &cmd, 10);
		if (ret) {
			pr_err("set_dbg_mask failed when send_cmd\n");
			return ret;
		} else
			return count;
	} else {
		pr_err("wrong input for \"<mask_out> <mask_level>\"\n");
		return -1;
	}
}

ssize_t ia_get_dbg_mask(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ia_cmd cmd;
	struct cmd_debug_param *param = (struct cmd_debug_param *)cmd.param;
	int ret;

	cmd.cmd_id = CMD_DEBUG;
	cmd.sensor_id = 0;
	param->sub_cmd = SCMD_DEBUG_GET_MASK;
	ret = ia_send_cmd(PSH2IA_CHANNEL0, &cmd, 8);
	if (ret) {
		pr_err("get_dbg_mask failed when send_cmd\n");
		return ret;
	}

	if (!wait_for_completion_timeout(&psh_ia_data->cmpl,
						msecs_to_jiffies(100)))
		return snprintf(buf, PAGE_SIZE, "no response\n");

	return snprintf(buf, PAGE_SIZE, "mask_out:%d mask_level:%d\n",
			psh_ia_data->dbg_mask.mask_out,
			psh_ia_data->dbg_mask.mask_level);
}

void ia_handle_snr_info(struct circ_buf *circ, const struct snr_info *sinfo)
{
#define STR_BUFF_SIZE 256
	char buf[STR_BUFF_SIZE];
	ssize_t str_size;
	int i;

	str_size = snprintf(buf, STR_BUFF_SIZE,
			"***** Sensor %5s(%d) Status *****\n",
			sinfo->name, sinfo->id);
	ia_circ_put_data(circ, buf, str_size);

	str_size = snprintf(buf, STR_BUFF_SIZE,
			"  freq=%d, freq_max=%d\n"
			"  status=0x%x,  slide=%d\n"
			"  data_cnt=%d,  priv=0x%x\n"
			"  attri=0x%x, health=%d\n",
			sinfo->freq, sinfo->freq_max,
			sinfo->status, sinfo->slide,
			sinfo->data_cnt, sinfo->priv,
			sinfo->attri, sinfo->health);
	ia_circ_put_data(circ, buf, str_size);

	for (i = 0; i < sinfo->link_num; i++) {
		const struct link_info *linfo = &sinfo->linfo[i];
		str_size = snprintf(buf, STR_BUFF_SIZE,
			"    %s%s=%5s, slide=%d\n",
			(linfo->ltype == LINK_AS_REPORTER) ?
						"REPORTER" : "CLIENT",
			(linfo->ltype == LINK_AS_MONITOR) ?
						"(M)" : "",
			ia_get_sensor_name(linfo->id),
			linfo->slide);

		ia_circ_put_data(circ, buf, str_size);
	}

	str_size = snprintf(buf, STR_BUFF_SIZE,
			"*****************************\n\n");
	ia_circ_put_data(circ, buf, str_size);
}

ssize_t ia_set_status_mask(struct device *dev,
			struct device_attribute *attr,
			const char *str, size_t count)
{
	char *s;
	long val = 0;

	while (*str) {
		val = trans_strtol(str, &s, 0);
		if (str == s) {
			str++;
			continue;
		} else
			break;
	}
	psh_ia_data->status_bitmask = val;

	pr_debug("set status_bitmask as 0x%x\n", psh_ia_data->status_bitmask);
	return count;
}

ssize_t ia_get_status_mask(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "status_mask=0x%x\n",
						psh_ia_data->status_bitmask);
}

ssize_t ia_trig_get_status(struct device *dev,
			struct device_attribute *attr,
			const char *str, size_t count)
{
	struct ia_cmd cmd;
	struct get_status_param *param = (struct get_status_param *)cmd.param;
	int ret;

	if (str[0] == 'm')
		param->snr_bitmask = psh_ia_data->status_bitmask;
	else if (str[0] == 'a')
		param->snr_bitmask = (u32)-1;
	else if (str[0] == 'r')
		param->snr_bitmask = ((u32)-1) & ~SNR_RUNONLY_BITMASK;

	cmd.cmd_id = CMD_GET_STATUS;
	cmd.sensor_id = 0;
	ret = ia_send_cmd(PSH2IA_CHANNEL0, &cmd, 7);
	if (ret) {
		pr_err("trig_get_status failed when send_cmd\n");
		return ret;
	}

	return count;
}

static SENSOR_DEVICE_ATTR(status_mask, S_IWUSR | S_IRUGO,
				ia_get_status_mask, ia_set_status_mask, 0);
static SENSOR_DEVICE_ATTR(status_trig, S_IWUSR, NULL, ia_trig_get_status, 1);
static SENSOR_DEVICE_ATTR(debug, S_IWUSR | S_IRUGO,
				ia_get_dbg_mask, ia_set_dbg_mask, 0);
static SENSOR_DEVICE_ATTR(control, S_IWUSR, NULL, ia_start_control, 1);
static SENSOR_DEVICE_ATTR(data_size, S_IRUGO, ia_read_data_size, NULL, 2);
static struct bin_attribute bin_attr = {
	.attr = { .name = "data", .mode = S_IRUGO },
	.read = ia_read_data
};
static struct bin_attribute dbg_attr = {
	.attr = { .name = "trace", .mode = S_IRUGO },
	.read = ia_read_debug_data
};

void ia_process_lbuf(struct device *dev)
{
	u8 *dbuf = NULL;
	u16 size = 0;

	while (!ia_lbuf_read_next(&psh_ia_data->lbuf, &dbuf, &size)) {
		struct cmd_resp *resp = (struct cmd_resp *)dbuf;
		if (resp->type == RESP_BIST_RESULT) {
			if (psh_ia_data->reset_in_progress) {
				complete(&psh_ia_data->cmd_reset_comp);
				continue;
			}
		} else if (resp->type == RESP_DEBUG_MSG) {
			ia_circ_put_data(&psh_ia_data->circ_dbg,
					resp->buf, resp->data_len);
			continue;
		} else if (resp->type == RESP_GET_STATUS) {
			const struct snr_info *sinfo =
					(struct snr_info *)resp->buf;

			if (!resp->data_len)
				complete(&psh_ia_data->get_status_comp);
			else if (SNR_INFO_SIZE(sinfo) == resp->data_len)
				ia_handle_snr_info(&psh_ia_data->circ_dbg,
						sinfo);
			else {
				pr_err("Wrong RESP_GET_STATUS!\n");
				continue;
			}
		} else if (resp->type == RESP_DEBUG_GET_MASK) {
			memcpy(&psh_ia_data->dbg_mask, resp->buf,
					sizeof(psh_ia_data->dbg_mask));
			complete(&psh_ia_data->cmpl);
			continue;
		}
		pr_debug("one DDR frame, data of sensor %d, size %d\n",
				resp->sensor_id, size);
		ia_circ_put_data(&psh_ia_data->circ, dbuf, size);
	}

	sysfs_notify(&dev->kobj, NULL, "data_size");
}

int psh_ia_common_init(struct device *dev, struct psh_ia_priv **data)
{
	int ret = -1;

	psh_ia_data = kzalloc(sizeof(*psh_ia_data), GFP_KERNEL);
	if (!psh_ia_data) {
		dev_err(dev, "can not allocate psh_ia_data\n");
		goto priv_err;
	}
	*data = psh_ia_data;

	psh_ia_data->pg = alloc_pages(GFP_KERNEL | GFP_DMA32 | __GFP_ZERO,
			get_order(BUF_IA_DDR_SIZE));
	if (!psh_ia_data->pg) {
		dev_err(dev, "can not allocate ddr buffer\n");
		goto pg_err;
	}

	mutex_init(&psh_ia_data->cmd_mutex);
	init_completion(&psh_ia_data->cmpl);
	init_completion(&psh_ia_data->get_status_comp);
	init_completion(&psh_ia_data->cmd_reset_comp);

	psh_ia_data->reset_in_progress = 0;

	ia_lbuf_read_init(&psh_ia_data->lbuf, page_address(psh_ia_data->pg),
				BUF_IA_DDR_SIZE, ia_update_finished);

	psh_ia_data->circ.buf = kmalloc(CIRC_SIZE, GFP_KERNEL);
	if (!psh_ia_data->circ.buf) {
		dev_err(dev, "can not allocate circ buffer\n");
		goto circ_err;
	}

	psh_ia_data->circ_dbg.buf = kmalloc(CIRC_SIZE, GFP_KERNEL);
	if (!psh_ia_data->circ_dbg.buf) {
		dev_err(dev, "can not allocate circ buffer\n");
		goto dbg_err;
	}

	psh_ia_data->status_bitmask = ((u32)-1) & ~SNR_RUNONLY_BITMASK;

	ret = sysfs_create_file(&dev->kobj,
			&sensor_dev_attr_status_mask.dev_attr.attr);
	ret += sysfs_create_file(&dev->kobj,
			&sensor_dev_attr_status_trig.dev_attr.attr);
	ret += sysfs_create_file(&dev->kobj,
			&sensor_dev_attr_debug.dev_attr.attr);
	ret += sysfs_create_file(&dev->kobj,
			&sensor_dev_attr_control.dev_attr.attr);
	ret += sysfs_create_file(&dev->kobj,
			&sensor_dev_attr_data_size.dev_attr.attr);
	ret += sysfs_create_bin_file(&dev->kobj, &bin_attr);
	ret += sysfs_create_bin_file(&dev->kobj, &dbg_attr);
	if (ret) {
		dev_err(dev, "can not create sysfs\n");
		goto sysfs_err;
	}

	return 0;

sysfs_err:
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_status_mask.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_status_trig.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_debug.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_control.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_data_size.dev_attr.attr);
	sysfs_remove_bin_file(&dev->kobj, &bin_attr);
	sysfs_remove_bin_file(&dev->kobj, &dbg_attr);

	kfree(psh_ia_data->circ_dbg.buf);
dbg_err:
	kfree(psh_ia_data->circ.buf);
circ_err:
	__free_pages(psh_ia_data->pg, get_order(BUF_IA_DDR_SIZE));
pg_err:
	kfree(psh_ia_data);
priv_err:
	return ret;
}

void psh_ia_common_deinit(struct device *dev)
{
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_status_mask.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_status_trig.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_debug.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_control.dev_attr.attr);
	sysfs_remove_file(&dev->kobj,
		&sensor_dev_attr_data_size.dev_attr.attr);
	sysfs_remove_bin_file(&dev->kobj, &bin_attr);
	sysfs_remove_bin_file(&dev->kobj, &dbg_attr);

	kfree(psh_ia_data->circ.buf);

	kfree(psh_ia_data->circ_dbg.buf);

	__free_pages(psh_ia_data->pg, get_order(BUF_IA_DDR_SIZE));

	kfree(psh_ia_data);
}

