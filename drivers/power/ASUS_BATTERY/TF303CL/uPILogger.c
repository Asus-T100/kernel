/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "uPILogger.h"

extern struct battery_info_reply batt_info;

static int bq27520_proc_info_dump_read(char *page, char **start, off_t off, int count, int *eof, void *date) {
        int len, local_len;
        static int bq_batt_percentage = 0;
        static int bq_batt_volt = 0;
        static int bq_batt_current = 0;
        static int bq_batt_temp = 0;
        static int bq_batt_remaining_capacity = 0;
        static int bq_batt_full_charge_capacity = 0;

        len = local_len = 0;

        bq_batt_full_charge_capacity = bq27520_asus_battery_dev_read_full_charge_capacity();

        bq_batt_remaining_capacity = bq27520_asus_battery_dev_read_remaining_capacity();
        bq_batt_percentage         = bq27520_asus_battery_dev_read_percentage();
        bq_batt_volt               = bq27520_asus_battery_dev_read_volt();
        bq_batt_current            = bq27520_asus_battery_dev_read_current();
        bq_batt_temp               = bq27520_asus_battery_dev_read_temp();

        if (bq_batt_current >= 0) bq_batt_current  = bq_batt_current  - 0x10000;

        BQ_DUMP("LMD(mAh): %d\n", bq_batt_full_charge_capacity);
        BQ_DUMP("NAC(mAh): %d\n", bq_batt_remaining_capacity);
        BQ_DUMP("RSOC: %d\n", bq_batt_percentage);
        BQ_DUMP("USOC: %d\n", batt_info.percentage);
        BQ_DUMP("voltage(mV): %d\n", bq_batt_volt);
        BQ_DUMP("average_current(mA): %d\n", bq_batt_current);
        BQ_DUMP("temp: %d\n", bq_batt_temp);

        return len;

}

int bq27520_register_upilogger_proc_fs(void) {
        struct proc_dir_entry *entry=NULL;

        entry = create_proc_entry("bq27520_test_info_dump", 0666, NULL);
        if (!entry) {
            printk("[%s]Unable to create bq27520_test_info_dump \n", __FUNCTION__);
            return -EINVAL;
        }
        entry->read_proc = bq27520_proc_info_dump_read;
        entry->write_proc = NULL;

        return 0;
}
