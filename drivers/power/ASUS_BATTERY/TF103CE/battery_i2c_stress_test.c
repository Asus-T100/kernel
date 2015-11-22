/*
 * Copyright (c) 2014, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "battery_i2c_stress_test.h"
#include "smb345_external_include.h"
#include "ug31xx/ug31xx_gauge.h"

static int stress_test_poll_mode = 0;
struct delayed_work battery_stress_test_poll_work;
static struct workqueue_struct *battery_stress_test_work_queue;
static wait_queue_head_t poll_wait_queue_head_t;
static bool flag_pollin = true;

static void i2c_stress_test_fuction() {
    //TODO
    int ret = 0, voltage = 3700;

    voltage = (int)ug31_module.get_voltage_now();
    if (voltage < 0) {
        printk("------------> [upi gauge] i2c fail !!!!! <--------------------\n");
    } else {
        printk("------------> [upi gauge] stress test voltage  = %d mV , polling = %d sec <--------------------\n", voltage , stress_test_poll_mode/HZ);
    }

    ret = smb345_get_charging_status();
    if (ret >= 0)
        printk("------------> [smb345 charger] status successful , polling = %d sec <--------------------\n", stress_test_poll_mode/HZ);
    else
        printk("------------> [smb345 charger] status fail !!!!! <--------------------\n");

}

static void battery_stress_test_poll(struct work_struct * work) {

    i2c_stress_test_fuction();

    queue_delayed_work(battery_stress_test_work_queue, &battery_stress_test_poll_work, stress_test_poll_mode);
}

int battery_open(struct inode *inode, struct file *filp) {
	printk("battery i2c : %s\n", __func__);
	return 0;
}

int battery_release(struct inode *inode, struct file *filp) {
	printk("battery i2c : %s\n", __func__);
	return 0;
}

static unsigned int battery_poll(struct file *filp, poll_table *wait){
	unsigned int mask = 0;
	poll_wait(filp, &poll_wait_queue_head_t, wait);
	if (flag_pollin==true) {
		mask |= POLLIN;
		flag_pollin=false;
	}
	printk("battery i2c : %s\n", __func__);
	return mask;
}

long battery_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	int err = 1;
	printk("--------------------------> %s <-------------------\n", __func__);
	if (_IOC_TYPE(cmd) != BATTERY_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > BATTERY_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case BATTERY_POLL_DATA:
			if (arg == BATTERY_IOCTL_START_HEAVY){
				printk("battery  : ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(battery_stress_test_work_queue, &battery_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == BATTERY_IOCTL_START_NORMAL){
				printk("battery  : ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(battery_stress_test_work_queue, &battery_stress_test_poll_work, stress_test_poll_mode);
			} else if  (arg == BATTERY_IOCTL_END){
				printk("battery : ioctl end\n");
				cancel_delayed_work_sync(&battery_stress_test_poll_work);
			} else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
        }

        return 0;
}

struct file_operations battery_fops = {
	.open = battery_open,
	.release = battery_release,
	.poll = battery_poll,
	.unlocked_ioctl = battery_ioctl,
};

struct miscdevice battery_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "battery",
  .fops = &battery_fops
};


int register_battery_stress_test(void) {
        int rtn;

        /* stress test */
	battery_stress_test_work_queue = create_singlethread_workqueue("i2c_battery_wq");
	if(!battery_stress_test_work_queue) {
		printk("battery i2c stress : unable to create i2c stress test workqueue\n");
	}
	INIT_DELAYED_WORK(&battery_stress_test_poll_work, battery_stress_test_poll);

	rtn = misc_register(&battery_misc);
	if(rtn < 0) {
		printk("[%s] Unable to register  battery misc \n", __func__);
		misc_deregister(&battery_misc);
	}

        return 0;
}

