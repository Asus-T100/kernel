/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/idr.h>

#include "asus_battery.h"
#include "bq27520_battery_core.h"
#include "bq27520_proc_fs.h"
#include "bq27520_battery_upt_i2c.h"

extern struct battery_dev_info bq27520_dev_info;
extern struct mutex bq27520_dev_info_mutex;

extern struct battery_dev_info batt_upt_dev_info;
extern struct mutex batt_dev_upt_mutex;

static char proc_buf[512];
static int return_val;

extern int bq27520_asus_battery_dev_read_chemical_id(void);

#define BQ_DUMP(...)                        \
do {                                        \
    local_len = sprintf(page, __VA_ARGS__); \
    len += local_len;                       \
    page += local_len;                      \
} while(0);

/* max number of characters per line in the dffs file is 296
   For instance:
   W: 16 04 EA FF 33 06 FA 33 8C FA 33 39 FE 33 41 FE 33 FF FF 3F FF FF 3F FF
   FF 3F FF FF 3F FF FF 3F FF FF 3F FF FF 3F FF FF 3F FF FF 3F FF FF 3F FF FF
   3F 29 03 3F 58 CB 33 C1 00 08 00 00 00 00 00 00 FF AA 0E FB A7 0E FF A6 0E
   FF A1 0E FF A0 0E FE A3 0E 6C A2 0E D2 FF 3A FF A1 0E 63 A0 0E FF A3 0E
*/
#define MAX_NUM_OF_CHAR_PER_LINE            296
/*
   max number of charaters per line except command is 293
   (ex. "W: " contained 3 charaters)
*/
#define MAX_NUM_OF_CHAR_PER_LINE_EXC_CMD    (MAX_NUM_OF_CHAR_PER_LINE - 3)

int enterRomMode_test;
const char *ROM_MODE_WRITE = "W: 16";
const char *NORMAL_MODE_WRITE = "W: AA";

ssize_t explain_dffs_string(char *ori_msg, unsigned long len)
{
    char conv_msg[MAX_NUM_OF_CHAR_PER_LINE_EXC_CMD] = {0};

    char *delimit=" ";
    char *p;
    u8 msg_count = 0;
    uint8_t msg_val[96] = {0};
    uint8_t data_to_check = 0;
    int status;

    char *tmp_msg_tok;
    char *msg_token;
    char *nxt_token;
    int i;
    char cmd;
    int slave_addr;
    u8 regs_addr;
    u8 cmd_c_addr;
    u8 read_count;
    int wait_time;

    struct i2c_client *client_upt = NULL;
    struct i2c_client *client = NULL;
    struct i2c_client *i2c_tmp = NULL;

    mutex_lock(&batt_dev_upt_mutex);
    client_upt = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);
    if (!client_upt|| !client_upt->adapter)
        return -ENODEV;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);
    if (!client|| !client->adapter)
        return -ENODEV;

    cmd = ori_msg[0];
    BAT_DBG("cmd:%c\n", cmd);

    BAT_DBG("%s\n", ori_msg);
    if (!strncmp(ROM_MODE_WRITE, ori_msg, 5)) {
        i2c_tmp = client_upt;
        BAT_DBG("client_upt\n");
    }
    else if (!strncmp(NORMAL_MODE_WRITE, ori_msg, 5)) {

        if (bq27520_is_rom_mode()) {
            BAT_DBG_E("* Ignore it as recv normal mode command"
                " but it's ROM mode *\n");
            return len;
        }

        i2c_tmp = client;
        BAT_DBG("client\n");
    }
    else {
        i2c_tmp = NULL;
        BAT_DBG("client is NULL\n");
    }

    for (i=0; i<MAX_NUM_OF_CHAR_PER_LINE_EXC_CMD; i++)
        conv_msg[i] = ori_msg[i+3];

    /*  check command type:
        if X need use decimal type to read msleep
        if command is C or W, slave address should
        be 0x16
    */

    BAT_DBG("command handling... \n");

    if ('X' == cmd) {
        tmp_msg_tok = conv_msg;
        msg_token = strsep(&tmp_msg_tok, delimit);
        BAT_DBG("wait command:%s\n", msg_token);
        wait_time = (int)simple_strtol(msg_token, NULL, 10);
        BAT_DBG("X: %d\n", wait_time);

        msleep(wait_time);

        if (4000 == wait_time) {
            enterRomMode_test = 0;
            TIgauge_LockStep();
        }

        return -EPIPE;
    }
    else if (('W' == cmd) || ('C' == cmd)) {
        tmp_msg_tok = conv_msg;
        msg_token = strsep(&tmp_msg_tok, delimit);
        nxt_token = tmp_msg_tok;
        BAT_DBG("slave addr string:%s\n", msg_token);
        slave_addr = (int)simple_strtol(msg_token, NULL, 16);
        BAT_DBG("slave addr string:0x%X\n", slave_addr);

        p = strsep(&nxt_token, delimit);
        BAT_DBG("reg addr sting:%s\n", p);
        regs_addr = (u8)simple_strtol(p, NULL, 16);
        BAT_DBG("reg addr string:0x%02X\n", regs_addr);

        /*  msg_val[0];I2C slave address will be 16
            msg_val[1];reg  address
            depend on command type(W/C) do different process.
            do real I2C command(W/C)
        */
        while ((p=strsep(&nxt_token, delimit))) {
            BAT_DBG("%s\n", p);
            msg_val[msg_count] = (uint8_t)simple_strtol(p, NULL, 16);
            BAT_DBG("[%d]0x%02X\n", msg_count, msg_val[msg_count]);
            msg_count++;
        }

        /* command parser */
        if ('W' == cmd) {
            status = TIgauge_i2c_write(i2c_tmp, regs_addr,
                        msg_count, msg_val);
            if (status > 0) {
                BAT_DBG("W: success\n");
                return len;
            }
            else {
                BAT_DBG("W: fail\n");
                return -EAGAIN;
            }
        }
        else if ('C' == cmd) {
            for (read_count=0; read_count<msg_count; read_count++) {
                cmd_c_addr = regs_addr + read_count;

                /* Only Compare in ROM mode */
                status = TIgauge_i2c_read(client_upt, cmd_c_addr,
                            1, &data_to_check);

                if (status > 0) {
                    BAT_DBG("C: success\n");
                }
                else {
                    BAT_DBG("C: fail\n");
                    return -EAGAIN;
                }

                if (data_to_check != msg_val[read_count]) {
                    BAT_DBG("C: fail\n");
                    return -EAGAIN;
                }
                else {
                    BAT_DBG("C: success\n");
                    return -EFBIG;
                }
            }
        }
    }
    else {
        BAT_DBG("%s\n", ori_msg);
    }

    BAT_DBG("command handled\n");
    return len;
}

void exitRomMode(void)
{
    long rt;
    char OutRomMode_1[] = "W: 16 00 05";
    char OutRomMode_2[] = "W: 16 64 05 00";
    char OutRomMode_3[] = "X: 170";
    char OutRomMode_4[] = "C: 16 04 9D 2E CF A2";
    char OutRomMode_5[] = "W: 16 00 0F";
    char OutRomMode_6[] = "W: 16 64 0F 00";
    char OutRomMode_7[] = "X: 4000";

    rt = explain_dffs_string(OutRomMode_1, strlen(OutRomMode_1));
    BAT_DBG("return_1:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_2, strlen(OutRomMode_2));
    BAT_DBG("return_2:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_3, strlen(OutRomMode_3));
    BAT_DBG("return_3:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_4, strlen(OutRomMode_4));
    BAT_DBG("return_4:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_5, strlen(OutRomMode_5));
    BAT_DBG("return_5:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_6, strlen(OutRomMode_6));
    BAT_DBG("return_6:%ld\n",rt);
    rt = explain_dffs_string(OutRomMode_7, strlen(OutRomMode_7));
    BAT_DBG("return_7:%ld\n",rt);
}

int bq27520_proc_protocol_read(char *page, char **start, off_t off,
                            int count, int *eof, void *date)
{
    int len;

    if (return_val < 0 && return_val >= -255) {
        len = sprintf(page, "False %d\n", return_val);
    } else if (return_val == PROC_TRUE) {
        len = sprintf(page, "True");
    } else if (return_val == PROC_FALSE) {
        len = sprintf(page, "False");
    } else {
        len = sprintf(page, "0x%08X\n", return_val);
    }

    return len;
}

int bq27520_proc_protocol_write(struct file *file, const char *buffer,
                            unsigned long count, void *data)
{
    int reg_off=0, value=0, ret=0;
    char mode, len;
    int byte_len=1;
    static struct i2c_client *client=NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    dev_info(&client->dev, "%s\n", __func__);

    if (count > sizeof(proc_buf)) {
        dev_err(&client->dev, "data error\n");
        return_val = -EINVAL;
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buffer, count)) {
        dev_err(&client->dev, "read data from user space error\n");
        return_val = -EFAULT;
        return -EFAULT;
    }

    sscanf(proc_buf, "%c%c %02X %02X\n", &mode, &len, &reg_off, &value);
    if (len == 'b') {
        byte_len = 1;
    } else if (len == 'w') {
        byte_len = 2;
    }

    if (mode == 'C' && len == 'R') { //compare in ROM mode
        ret = bq27520_rom_mode_cmp(reg_off, value);
        if (ret  == PROC_TRUE || ret == PROC_FALSE) {
            return_val = ret;
            ret = 0;
        }

    } else if (mode == 'c' && len == 'r') { //compare in normal mode
        ret = bq27520_cmp_i2c(reg_off, value);
        if (ret  == PROC_TRUE || ret == PROC_FALSE) {
            return_val = ret;
            ret = 0;
        }

    } else if (mode == 'X' && len == 'R') { //wait in ROM mode
        int m_sec = reg_off;

        ret = bq27520_rom_mode_wait(m_sec);

    } else if (mode == 'w') {
        dev_info(&client->dev, "%s write reg:%02X, value: %04X, len: %d\n",
                                __func__, reg_off, value, byte_len);
        ret = bq27520_write_i2c(client, reg_off, value, byte_len % 2);

    } else if (mode == 'W') {
        dev_info(&client->dev, "%s ROM write reg:%02X, value: %04X, len: %d\n",
                                __func__, reg_off, value, byte_len);
        ret = bq27520_rom_mode_write_i2c(reg_off, value, byte_len % 2);

    } else if (mode == 'r') {
        ret = bq27520_read_i2c(client, reg_off, &value, byte_len % 2);
        dev_info(&client->dev, "%s read reg:%02X, value: %04X, len: %d\n",
                                __func__, reg_off, value, byte_len);
        return_val = value;

    } else if (mode == 'R') {
        ret = bq27520_rom_mode_read_i2c(reg_off, &value, byte_len % 2);
        dev_info(&client->dev, "%s ROM read reg:%02X, value: %04X, len: %d\n",
                                __func__, reg_off, value, byte_len);
        return_val = value;

    } else {
        dev_info(&client->dev, "mode fail\n");
        return_val = -EACCES;
    }

    if (ret) {
        dev_err(&client->dev, "i2c operation  fail. %d\n", ret);
        return_val = -EIO;
        return -EIO;
    }

    memset(proc_buf, 0x00, sizeof(proc_buf));

    return count;
}

char str_batt_update_status[][40] = {
    "UPDATE_PROCESS_FAIL",
    "UPDATE_ERR_MATCH_OP_BUF",
    "UPDATE_CHECK_MODE_FAIL",
    "UPDATE_VOLT_NOT_ENOUGH",
    "UPDATE_NONE",
    "UPDATE_OK",
    "UPDATE_FROM_ROM_MODE",
};

int bq27520_proc_info_dump_read(char *page, char **start, off_t off,
                            int count, int *eof, void *date)
{
    int tmp_buf;
    int len, local_len;
    static int bq_batt_percentage = 0;
    static int bq_batt_volt = 0;
    static int bq_batt_current = 0;
    static int bq_batt_temp = 0;
    static int bq_batt_remaining_capacity = 0;
    static int bq_batt_full_charge_capacity = 0;
    static int bq_batt_nominal_available_capacity = 0;
    static int bq_batt_chemical_id = 0;
    static int bq_batt_fw_version = 0;
    static int bq_batt_fw_cfg_version = 0;
    static int bq_batt_cc = 0;
    struct battery_dev_info tmp_dev_info;

    mutex_lock(&bq27520_dev_info_mutex);
    tmp_dev_info = bq27520_dev_info;
    mutex_unlock(&bq27520_dev_info_mutex);

    len = local_len = 0;

    if (tmp_dev_info.status != DEV_INIT_OK) {
        BAT_DBG_E("(%s) status = %d\n", __func__, tmp_dev_info.update_status);
        BQ_DUMP("update_status %s\n",
        str_batt_update_status[tmp_dev_info.update_status-UPDATE_PROCESS_FAIL]);
        return len;
    }

    tmp_buf = bq27520_asus_battery_dev_read_percentage();
    if (tmp_buf >= 0) bq_batt_percentage = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_volt();
    if (tmp_buf >= 0) bq_batt_volt = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_current();
    if (tmp_buf >= 0) bq_batt_current = tmp_buf - 0x10000;

    tmp_buf = bq27520_asus_battery_dev_read_temp();
    if (tmp_buf >= 0) bq_batt_temp = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_remaining_capacity();
    if (tmp_buf >= 0) bq_batt_remaining_capacity = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_full_charge_capacity();
    if (tmp_buf >= 0) bq_batt_full_charge_capacity = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_nominal_available_capacity();
    if (tmp_buf >= 0) bq_batt_nominal_available_capacity = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_chemical_id();
    if (tmp_buf >= 0) bq_batt_chemical_id = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_fw_version();
    if (tmp_buf >= 0) bq_batt_fw_version = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_fw_cfg_version();
    if (tmp_buf >= 0) bq_batt_fw_cfg_version = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_cycle_count();
    if (tmp_buf >= 0) bq_batt_cc = tmp_buf;

    BQ_DUMP("LMD(mAh): %d\n", bq_batt_full_charge_capacity);
    BQ_DUMP("NAC(mAh): %d\n", bq_batt_nominal_available_capacity);
    BQ_DUMP("RM(mAh): %d\n", bq_batt_remaining_capacity);
    BQ_DUMP("RSOC: %d\n", bq_batt_percentage);
    BQ_DUMP("voltage(mV): %d\n", bq_batt_volt);
    BQ_DUMP("average_current(mA): %d\n", bq_batt_current);
    BQ_DUMP("temp: %d\n", bq_batt_temp);
    BQ_DUMP("chemical_id: 0x%04X\n", bq_batt_chemical_id);
    BQ_DUMP("fw_version: 0x%04X\n", bq_batt_fw_version);
    BQ_DUMP("fw_cfg_version: 0x%04X\n", bq_batt_fw_cfg_version);
    BQ_DUMP("CC(num): %d\n", bq_batt_cc);
    BQ_DUMP("update_status: %s\n",
        str_batt_update_status[tmp_dev_info.update_status-UPDATE_PROCESS_FAIL]);

    return len;
}

int bq27520_proc_info_dump_write(struct file *file, const char *buffer,
                            unsigned long count, void *data)
{
    return count;
}

int test_op_fail = 0;
int test_data_fail=0;
int test_loop_count=0;

int bq27520_proc_test_case_read(char *page, char **start, off_t off,
                            int count, int *eof, void *date)
{
    int len, local_len;

    len = local_len = 0;

    BQ_DUMP("op_fail: %d, data_fail: %d, count: %d\n",
        test_op_fail,
        test_data_fail,
        test_loop_count
    );
}

int bq27520_proc_test_case_write(struct file *file, const char *buffer,
                            unsigned long count, void *data)
{
    static struct i2c_client *client=NULL;
    char mode[30]={0};
    int loop_count;
    int sub_mode;
    int i;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    dev_info(&client->dev, "%s\n", __func__);

    if (count > sizeof(proc_buf)) {
        dev_err(&client->dev, "data error\n");
        return_val = -EINVAL;
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buffer, count)) {
        dev_err(&client->dev, "read data from user space error\n");
        return_val = -EFAULT;
        return -EFAULT;
    }

    sscanf(proc_buf, "%s %d %d\n", mode, &loop_count, &sub_mode);
    BAT_DBG("Test %s, count %d\n", mode, loop_count);

    if (!strcmp(mode, "I2C")) {
        int prev_data, cur_data;

        test_op_fail = 0;
        test_loop_count = 0;
        test_data_fail = 0;
        test_loop_count = loop_count;

        prev_data = bq27520_asus_battery_dev_read_chemical_id();
        for (i=0; i<loop_count; i++) {
            if ((i & 0x0FF) == 0) {
                BAT_DBG("Count %d\n", i);
            }
            cur_data = bq27520_asus_battery_dev_read_chemical_id();
            if (cur_data < 0) {
                test_op_fail++;
            } else if (cur_data != prev_data) {
                test_data_fail++;
            }
            udelay(1000);
        }
        BAT_DBG("Done\n");
    } else {
        BAT_DBG("Unknown command.\n");
    }
}

ssize_t bq27520_proc_bridge_write(struct file *filp, const char __user *buff,
                            unsigned long len, void *data)
{
    char ori_msg[MAX_NUM_OF_CHAR_PER_LINE] = {0};
    long rt;

    if (len > MAX_NUM_OF_CHAR_PER_LINE) {
        len = MAX_NUM_OF_CHAR_PER_LINE;
    }

    if (copy_from_user(ori_msg, buff, len)) {
        return -EFAULT;
    }

    rt = explain_dffs_string(ori_msg, len);

    return rt;
}

int bq27520_register_proc_fs_test(void)
{
    struct proc_dir_entry *entry=NULL;
    struct battery_dev_info tmp_dev_info;

    mutex_lock(&bq27520_dev_info_mutex);
    tmp_dev_info = bq27520_dev_info;
    mutex_unlock(&bq27520_dev_info_mutex);

    entry = create_proc_entry("bq27520_test_proc_protocol", 0644, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create bq27520_test_proc_protocol\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_protocol_read;
    entry->write_proc = bq27520_proc_protocol_write;

    entry = create_proc_entry("bq27520_test_info_dump", 0644, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create bq27520_test_info_dump\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_info_dump_read;
    entry->write_proc = bq27520_proc_info_dump_write;

    entry = create_proc_entry("bq27520_test_case", 0644, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create bq27520_test_case\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_test_case_read;
    entry->write_proc = bq27520_proc_test_case_write;

    return 0;
}
