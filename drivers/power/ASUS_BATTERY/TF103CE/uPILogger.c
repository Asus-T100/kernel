/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "uPILogger.h"
#if defined(CONFIG_UPI_BATTERY)
#include "ug31xx/ug31xx_gauge.h"

extern void set_ug31xx_polling(int s);
#endif

extern struct battery_info_reply batt_info;

static int ug31_proc_info_dump_read(struct seq_file *m, void *p) {
        int len, local_len;
        static int bq_batt_percentage = 0;
        static int bq_batt_volt = 0;
        static int bq_batt_current = 0;
        static int bq_batt_temp = 0;
        static int bq_batt_remaining_capacity = 0;
        static int bq_batt_full_charge_capacity = 0;

        len = local_len = 0;
#if defined(CONFIG_UPI_BATTERY)
        set_ug31xx_polling(2);

        bq_batt_full_charge_capacity = (int) ug31_module.get_full_charge_capacity();
        bq_batt_remaining_capacity = (int) ug31_module.get_remaining_capacity();
        bq_batt_percentage         = (int) ug31_module.get_relative_state_of_charge();
        bq_batt_volt               = (int) ug31_module.get_voltage();
        bq_batt_current            = (int) ug31_module.get_current();
        bq_batt_temp               = (int) ug31_module.get_external_temperature();
#endif

        seq_printf(m,"LMD: %d\n", bq_batt_full_charge_capacity);
        seq_printf(m,"NAC: %d\n", bq_batt_remaining_capacity);
        seq_printf(m,"RSOC: %d\n", bq_batt_percentage);
        seq_printf(m,"voltage(mV): %d\n", bq_batt_volt);
        seq_printf(m,"average_current(mA): %d\n", bq_batt_current);
        seq_printf(m,"IT: %d\n", bq_batt_temp);

        return len;

}

static ssize_t proc_ug31_test_write(struct file *file, const char *buffer, size_t count, loff_t *data) {

    printk(" %s:\n", __func__);
#if defined(CONFIG_UPI_BATTERY)
    if (buffer[0] == '2') {
        set_ug31xx_polling(2);
    } else if (buffer[0] == '60') {
        set_ug31xx_polling(60);
    } else {
        set_ug31xx_polling(-1);
    }
#endif

    return count;
}

static int proc_ug31_test_info_dump_open(struct inode *inode, struct file *file) {
	return single_open(file, ug31_proc_info_dump_read, NULL);
}

static const struct file_operations proc_ug31_test_info_dump_ops = {
	.open		= proc_ug31_test_info_dump_open,
	.read		= seq_read,
        .write          = proc_ug31_test_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};

int ug31xx_register_upilogger_proc_fs(void) {
        struct proc_dir_entry *entry = NULL;

        entry = proc_create("ug31_test_info_dump", 0666, NULL, &proc_ug31_test_info_dump_ops);
        if (!entry) {
            printk("[%s]Unable to create ug31_test_info_dump \n", __FUNCTION__);
            return -EINVAL;
        }

        return 0;
}

