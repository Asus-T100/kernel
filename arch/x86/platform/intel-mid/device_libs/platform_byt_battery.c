//<asus-ych20130904>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power/byt_battery.h>

static struct byt_platform_data byt_pdata;

static struct i2c_board_info __initdata byt_i2c_device = {
	I2C_BOARD_INFO("asus_byt", 0x66),
	.platform_data = &byt_pdata,
};

static int __init byt_i2c_init(void)
{
	byt_pdata.gpio = 0x18;
	int x = i2c_register_board_info(1, &byt_i2c_device, 1);
	printk(KERN_ALERT"i2c_register_board_info : %d \n", x);
	return x;
}
module_init(byt_i2c_init);

