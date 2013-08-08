
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power/byt_ulpmc_battery.h>

static struct ulpmc_platform_data ulpmc_pdata;

static struct i2c_board_info __initdata ulpmc_i2c_device = {
	I2C_BOARD_INFO("ulpmc", 0x78),
	.platform_data = &ulpmc_pdata,
};

static int __init ulpmc_i2c_init(void)
{
	ulpmc_pdata.gpio = 0x12; /* GPIOS_18 */
	ulpmc_pdata.volt_sh_min = 3400;	/* 3400mV */
	/*
	 * cc limit values will be
	 * further tuned on ffrd later.
	 */
	ulpmc_pdata.cc_lim0 = 2500;	/* 2500mA */
	ulpmc_pdata.cc_lim1 = 2000;	/* 2000mA */
	ulpmc_pdata.cc_lim2 = 1500;	/* 1500mA */
	ulpmc_pdata.cc_lim3 = 500;	/* 500mA */
	ulpmc_pdata.temp_ul = 50;	/* 50 degrees */
	ulpmc_pdata.temp_ll = 0;	/* 0 degrees */
	/*
	 * In BYT FFRD10 we have two batteries with
	 * each battery has an internal resistance (Rbatt)
	 * of ~150mOhms. As the batteries are connected
	 * in parallel effective Rbatt will become 75mOhms.
	 */
	ulpmc_pdata.rbatt = 75;		/* 75 mOhms */
	snprintf(ulpmc_pdata.battid, BATTID_LEN, "INT-BYT");
	snprintf(ulpmc_pdata.extcon_devname,
			EXTCON_NAME_LEN, "fsa9285");
	ulpmc_pdata.version = BYTULPMCFGV4;
	return i2c_register_board_info(1, &ulpmc_i2c_device, 1);
}
module_init(ulpmc_i2c_init);

