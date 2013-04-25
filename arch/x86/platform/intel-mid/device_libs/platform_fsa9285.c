
#include <linux/i2c.h>
#include <linux/kernel.h>

static struct i2c_board_info __initdata fsa9285_i2c_device = {
	I2C_BOARD_INFO("fsa9285", 0x25),
};

static int __init fsa9285_i2c_init(void)
{
	return i2c_register_board_info(3, &fsa9285_i2c_device, 1);
}
module_init(fsa9285_i2c_init);

