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

static int __init asus_dualboot_init(void)
{
	int err;
	struct proc_dir_entry *entry;

	pr_info("asus_dualboot_init\n");

        //test write to cmos
	entry = create_proc_entry("OSSW", S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if (!entry) {
		pr_err("(ossw) failed to create proc entry\n");
		return 0;
	}

	entry->proc_fops = &ossw_file_ops;

	return 0;
}

static void __exit asus_dualboot_exit(void)
{
        remove_proc_entry("OSSW", NULL);

	pr_info("asus_dualboot_exit\n");
	return;
}

module_init(asus_dualboot_init);
module_exit(asus_dualboot_exit);
