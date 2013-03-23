
#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/intel_vlv2.h>

/* FIXME: should be put into SFI table */
static int __init register_psh_i2c_dev(void)
{
	static int psh_gpios[2] = { 59, 95 };
	struct i2c_board_info info = {
		I2C_BOARD_INFO("psh_byt_i2c", 0x19),
		.irq = VV_GPIO_IRQBASE + VV_NGPIO_SCORE + VV_NGPIO_NCORE + 3,
		.platform_data = (void *)psh_gpios,
	};

	i2c_register_board_info(5, &info, 1);

	return 0;
}

fs_initcall(register_psh_i2c_dev);
