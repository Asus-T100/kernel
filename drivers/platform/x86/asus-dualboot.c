#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define CMOS_BANK2_IDX    0x72
#define CMOS_BANK2_DATA   0x73

//test write to cmos
static ssize_t ossw_write(struct file *file, const char __user *buf, 
                          size_t len, loff_t *offset)
{
        char word[256];
        char ref1[] = "1";
        char ref2[] = "1\n";

        pr_alert ("<asus-wanya> ossw_write: entered, len = %d\n", len);

        memset(word, 0, 256);
        copy_from_user (word, buf, len);
        if (strncmp(ref1, word, len) && strncmp(ref2, word, len)) {
            pr_alert ("<asus-wanya> unsupported command(%s)\n", word);
            return len;
        }

        outb(0x70, CMOS_BANK2_IDX);
        outb(0x19, CMOS_BANK2_DATA);

        return len;
}

//test write to cmos
static const struct file_operations ossw_file_ops = {
	.owner = THIS_MODULE,
	.write = ossw_write,
};

/*
 * Platform device
 */
static int __devinit asus_dualboot_platform_probe(struct platform_device *device)
{
	struct proc_dir_entry *entry;

        //test write to cmos
	entry = create_proc_entry("OSSW", S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if (!entry) {
		pr_err("(ossw) failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &ossw_file_ops;
}


static struct platform_driver asus_dualboot_platform_driver = {
	.driver = {
		.name = "asus-dualboot",
		.owner = THIS_MODULE,
	},
	.probe = asus_dualboot_platform_probe,
};

static struct platform_device *asus_dualboot_platform_device;

static int __init asus_dualboot_init(void)
{
	int err;

	pr_info("asus_dualboot_init\n");

	err = platform_driver_register(&asus_dualboot_platform_driver);
	if (err) {
		pr_err("Unable to register platform driver\n");
		goto error_platform_register;
	}

	asus_dualboot_platform_device = platform_device_alloc("asus-dualboot", -1);
	if (!asus_dualboot_platform_device) {
		err = -ENOMEM;
		goto error_device_alloc;
	}

	err = platform_device_add(asus_dualboot_platform_device);
	if (err)
		goto error_device_add;

	return 0;

error_device_add:
	platform_device_put(asus_dualboot_platform_device);
error_device_alloc:
	platform_driver_unregister(&asus_dualboot_platform_driver);
error_platform_register:

	return err;
}

static void __exit asus_dualboot_exit(void)
{
        remove_proc_entry("OSSW", NULL);

	platform_device_unregister(asus_dualboot_platform_device);
	platform_driver_unregister(&asus_dualboot_platform_driver);

	pr_info("asus_dualboot_exit\n");
	return;
}

module_init(asus_dualboot_init);
module_exit(asus_dualboot_exit);
