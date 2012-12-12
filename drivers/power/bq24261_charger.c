/*
 * bq24261_charger.c - BQ24261 Charger I2C client driver
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jenny TC <jenny.tc@intel.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/power/bq24261_charger.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#include <asm/intel_scu_ipc.h>
#include <linux/usb/dwc_otg3.h>
#define DEV_NAME "bq24261_charger"
#define CHRG_TERM_WORKER_DELAY (30 * HZ)

/* BQ24261 registers */
#define BQ24261_STAT_CTRL0_ADDR		0x00
#define BQ24261_CTRL_ADDR		0x01
#define BQ24261_BATT_VOL_CTRL_ADDR	0x02
#define BQ24261_VENDOR_REV_ADDR		0x03
#define BQ24261_TERM_FCC_ADDR		0x04
#define BQ24261_VINDPM_STAT_ADDR	0x05
#define BQ24261_ST_NTC_MON_ADDR		0x06

#define BQ24261_FAULT_MASK		0x07
#define BQ24261_STAT_MASK		(0x03 << 4)
#define BQ24261_BOOST_MASK		(0x01 << 6)
#define BQ24261_TMR_RST_MASK		(0x01 << 7)

#define BQ24261_ENABLE_BOOST		(0x01 << 6)

#define BQ24261_VOVP			0x01
#define BQ24261_LOW_SUPPLY		0x02
#define BQ24261_THERMAL_SHUTDOWN	0x03
#define BQ24261_BATT_TEMP_FAULT		0x04
#define BQ24261_TIMER_FAULT		0x05
#define BQ24261_BATT_OVP		0x06
#define BQ24261_NO_BATTERY		0x07
#define BQ24261_STAT_READY		0x00

#define BQ24261_STAT_CHRG_PRGRSS	(0x01 << 4)
#define BQ24261_STAT_CHRG_DONE		(0x02 << 4)
#define BQ24261_STAT_FAULT		(0x03 << 4)

#define BQ24261_CE_MASK			(0x01 << 1)
#define BQ24261_CE_DISABLE		(0x01 << 1)

#define BQ24261_HZ_MASK			(0x01)
#define BQ24261_HZ_ENABLE		(0x01)

#define BQ24261_CD_STATUS_MASK		(0x01 << 3)

#define BQ24261_ICHRG_MASK		(0x1F << 3)
#define BQ24261_ICHRG_100mA		(0x01 << 3)
#define BQ24261_ICHRG_200mA		(0x01 << 4)
#define BQ24261_ICHRG_400mA		(0x01 << 5)
#define BQ24261_ICHRG_800mA		(0x01 << 6)
#define BQ24261_ICHRG_1600mA		(0x01 << 7)

#define BQ24261_ITERM_MASK		(0x03)
#define BQ24261_ITERM_50mA		(0x01 << 1)
#define BQ24261_ITERM_100mA		(0x01 << 2)
#define BQ24261_ITERM_200mA		(0x01 << 3)

#define BQ24261_VBREG_MASK		(0x3F << 2)
#define BQ24261_VBREG_20mV		(0x01 << 2)
#define BQ24261_VBREG_40mV		(0x01 << 3)
#define BQ24261_VBREG_80mV		(0x01 << 4)
#define BQ24261_VBREG_160mV		(0x01 << 5)
#define BQ24261_VBREG_320mV		(0x01 << 6)
#define BQ24261_VBREG_640mV		(0x01 << 7)

#define BQ24261_INLMT_MASK		(0x03 << 4)
#define BQ24261_INLMT_100		0x00
#define BQ24261_INLMT_150		(0x01 << 4)
#define BQ24261_INLMT_500		(0x02 << 4)
#define BQ24261_INLMT_900		(0x03 << 4)
#define BQ24261_INLMT_1500		(0x04 << 4)
#define BQ24261_INLMT_2500		(0x06 << 4)

#define BQ24261_TE_MASK			(0x01 << 2)
#define BQ24261_TE_ENABLE		(0x01 << 2)

#define BQ24261_VENDOR_MASK		(0x07 << 5)
#define BQ24261_VENDOR			(0x02 << 5)
#define BQ24261_REV_MASK		(0x07)
#define BQ24261_REV			(0x02)

u16 bq24261_inlmt[][2] = {
	{100, BQ24261_INLMT_100}
	,
	{150, BQ24261_INLMT_150}
	,
	{500, BQ24261_INLMT_500}
	,
	{900, BQ24261_INLMT_900}
	,
	{1500, BQ24261_INLMT_1500}
	,
	{2500, BQ24261_INLMT_2500}
	,
};

u16 bq24261_iterm[][2] = {
	{50, 0x00}
	,
	{100, BQ24261_ITERM_50mA}
	,
	{150, BQ24261_ITERM_100mA}
	,
	{200, BQ24261_ITERM_100mA | BQ24261_ITERM_50mA}
	,
	{250, BQ24261_ITERM_200mA}
	,
	{300, BQ24261_ITERM_200mA | BQ24261_ITERM_50mA}
	,
	{350, BQ24261_ITERM_200mA | BQ24261_ITERM_100mA}
	,
	{400, BQ24261_ITERM_200mA | BQ24261_ITERM_100mA | BQ24261_ITERM_50mA}
	,
};

u16 bq24261_cc[][2] = {

	{500, 0x00}
	,
	{600, BQ24261_ICHRG_100mA}
	,
	{700, BQ24261_ICHRG_200mA}
	,
	{800, BQ24261_ICHRG_100mA | BQ24261_ICHRG_200mA}
	,
	{900, BQ24261_ICHRG_400mA}
	,
	{1000, BQ24261_ICHRG_400mA | BQ24261_ICHRG_100mA}
	,
	{1100, BQ24261_ICHRG_400mA | BQ24261_ICHRG_200mA}
	,
	{1200, BQ24261_ICHRG_400mA | BQ24261_ICHRG_200mA | BQ24261_ICHRG_100mA}
	,
	{1300, BQ24261_ICHRG_800mA}
	,
	{1400, BQ24261_ICHRG_800mA | BQ24261_ICHRG_100mA}
	,
	{1500, BQ24261_ICHRG_800mA | BQ24261_ICHRG_200mA}
	,
};

u16 bq24261_cv[][2] = {

	{3500, 0x00}
	,
	{3600, BQ24261_VBREG_20mV | BQ24261_VBREG_80mV}
	,
	{3700, BQ24261_VBREG_160mV | BQ24261_VBREG_40mV}
	,
	{3800,
	 BQ24261_VBREG_160mV | BQ24261_VBREG_80mV | BQ24261_VBREG_40mV |
	 BQ24261_VBREG_20mV}
	,
	{3900, BQ24261_VBREG_320mV | BQ24261_VBREG_80mV}
	,
	{4000, BQ24261_VBREG_320mV | BQ24261_VBREG_160mV | BQ24261_VBREG_20mV}
	,
	{4040,
	 BQ24261_VBREG_320mV | BQ24261_VBREG_160mV | BQ24261_VBREG_40mV |
	 BQ24261_VBREG_20mV}
	,
	{4060, BQ24261_VBREG_320mV | BQ24261_VBREG_160mV | BQ24261_VBREG_80mV}
	,
	{4100,
	 BQ24261_VBREG_320mV | BQ24261_VBREG_160mV | BQ24261_VBREG_80mV |
	 BQ24261_VBREG_40mV}
	,
	{4140, BQ24261_VBREG_640mV}
	,
	{4160, BQ24261_VBREG_640mV | BQ24261_VBREG_20mV}
	,
	{4200, BQ24261_VBREG_640mV | BQ24261_VBREG_40mV | BQ24261_VBREG_20mV}
	,
};

static enum power_supply_property bq24261_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_ENABLE_CHARGING,
	POWER_SUPPLY_PROP_ENABLE_CHARGER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
	POWER_SUPPLY_PROP_CABLE_TYPE,
};

struct bq24261_otg_event {
	struct list_head node;
	struct otg_bc_cap *cap;
};

struct bq24261_charger {

	struct mutex lock;
	struct i2c_client *client;
	struct bq24261_plat_data *pdata;
	struct power_supply psy_usb;
	struct delayed_work sw_term_work;
	struct notifier_block otg_nb;
	struct otg_transceiver *transceiver;
	struct work_struct otg_work;
	struct work_struct irq_work;
	struct list_head otg_queue;
	struct list_head irq_queue;
	spinlock_t otg_queue_lock;
	void __iomem *irq_iomap;

	int chrgr_health;
	int bat_status;
	int bat_health;
	int cc;
	int cv;
	int inlmt;
	int max_cc;
	int max_cv;
	int iterm;
	int cable_type;
	bool is_charging_enabled;
	bool is_charger_enabled;
	bool is_fault;
	bool is_bat_full;
	bool is_vsys_on;
	bool boost_mode;
};

struct i2c_client *bq24261_client;
static inline int get_battery_voltage(int *volt);
static inline int get_battery_current(int *cur);

enum power_supply_type get_power_supply_type(
		enum power_supply_charger_cable_type cable)
{

	switch (cable) {

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return POWER_SUPPLY_TYPE_USB_CDP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
		return POWER_SUPPLY_TYPE_USB_ACA;
	case POWER_SUPPLY_CHARGER_TYPE_AC:
		return POWER_SUPPLY_TYPE_MAINS;
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
	default:
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_USB;
}

static void lookup_regval(u16 tbl[][2], size_t size, u16 in_val, u8 *out_val)
{
	int i;
	for (i = 1; i < size; ++i)
		if (in_val < tbl[i][0])
			break;

	*out_val = (u8) tbl[i - 1][1];
}

void bq24261_cc_to_reg(int cc, u8 *reg_val)
{
	return lookup_regval(bq24261_cc, ARRAY_SIZE(bq24261_cc), cc, reg_val);

}
EXPORT_SYMBOL_GPL(bq24261_cc_to_reg);

void bq24261_cv_to_reg(int cv, u8 *reg_val)
{
	return lookup_regval(bq24261_cv, ARRAY_SIZE(bq24261_cv), cv, reg_val);
}
EXPORT_SYMBOL_GPL(bq24261_cv_to_reg);

static inline void bq24261_inlmt_to_reg(int inlmt, u8 *regval)
{
	return lookup_regval(bq24261_inlmt, ARRAY_SIZE(bq24261_inlmt),
			     inlmt, regval);
}

static inline void bq24261_iterm_to_reg(int iterm, u8 *regval)
{
	return lookup_regval(bq24261_iterm, ARRAY_SIZE(bq24261_iterm),
			     iterm, regval);
}

static inline int bq24261_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in reading reg %d\n", ret,
			reg);

	return ret;
}


static inline void bq24261_dump_regs(bool dump_master)
{
	int i;
	int ret;
	int bat_cur, bat_volt;
	struct bq24261_charger *chip = i2c_get_clientdata(bq24261_client);

	ret = get_battery_current(&bat_cur);
	if (ret)
		dev_err(&bq24261_client->dev,
			"%s: Error in getting battery current", __func__);
	else
		dev_info(&bq24261_client->dev, "Battery Current=%dmA\n",
				(bat_cur/1000));

	ret = get_battery_voltage(&bat_volt);
	if (ret)
		dev_err(&bq24261_client->dev,
			"%s: Error in getting battery voltage", __func__);
	else
		dev_info(&bq24261_client->dev, "Battery VOlatge=%dmV\n",
			(bat_volt/1000));


	dev_info(&bq24261_client->dev, "BQ24261 Register dump\n");

	dev_info(&bq24261_client->dev, "*======================*\n");
	for (i = 0; i < 7; ++i) {
		ret = bq24261_read_reg(bq24261_client, i);
		if (ret < 0)
			dev_err(&bq24261_client->dev,
				"Error in reading REG 0x%X\n", i);
		else
			dev_info(&bq24261_client->dev,
				"0x%X=0x%X ", i, ret);
	}
	dev_info(&bq24261_client->dev, "*======================*\n");

	if (chip->pdata->dump_master_regs && dump_master)
			chip->pdata->dump_master_regs();

}


#ifdef CONFIG_DEBUG_FS
static int bq24261_reg_show(struct seq_file *seq, void *unused)
{
	int val;
	u8 reg;

	reg = *((u8 *)seq->private);
	val = bq24261_read_reg(bq24261_client, reg);

	seq_printf(seq, "0x%02x\n", val);
	return 0;
}

static int bq24261_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq24261_reg_show, inode->i_private);
}

static u32 bq24261_register_set[] = {
	BQ24261_STAT_CTRL0_ADDR,
	BQ24261_CTRL_ADDR,
	BQ24261_BATT_VOL_CTRL_ADDR,
	BQ24261_VENDOR_REV_ADDR,
	BQ24261_TERM_FCC_ADDR,
	BQ24261_VINDPM_STAT_ADDR,
	BQ24261_ST_NTC_MON_ADDR,
};

static struct dentry *bq24261_dbgfs_dir;

static const struct file_operations bq24261_dbg_fops = {
	.open = bq24261_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static void bq24261_debugfs_init(void)
{
	struct dentry *fentry;
	u32 count = ARRAY_SIZE(bq24261_register_set);
	u32 i;
	char name[6] = {0};

	bq24261_dbgfs_dir = debugfs_create_dir(DEV_NAME, NULL);
	if (bq24261_dbgfs_dir == NULL)
		goto debugfs_root_exit;

	for (i = 0; i < count; i++) {
		sprintf(name, "%02x", bq24261_register_set[i]);
		fentry = debugfs_create_file(name, S_IRUGO,
						bq24261_dbgfs_dir,
						&bq24261_register_set[i],
						&bq24261_dbg_fops);
		if (fentry == NULL)
			goto debugfs_err_exit;
	}
	dev_err(&bq24261_client->dev, "Debugfs created successfully!!\n");
	return;

debugfs_err_exit:
	debugfs_remove_recursive(bq24261_dbgfs_dir);
debugfs_root_exit:
	dev_err(&bq24261_client->dev, "Error Creating debugfs!!\n");
	return;
}

static void bq24261_debugfs_exit(void)
{
	if (bq24261_dbgfs_dir)
		debugfs_remove_recursive(bq24261_dbgfs_dir);

	return;
}

#else
static void bq24261_debugfs_init(void)
{
	return;
}

static void bq24261_debugfs_exit(void)
{
	return;
}
#endif

static inline int bq24261_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in writing %d to reg %d\n",
			ret, data, reg);

	return ret;
}

static inline int bq24261_read_modify_reg(struct i2c_client *client, u8 reg,
					  u8 mask, u8 val)
{
	int ret;

	ret = bq24261_read_reg(client, reg);
	if (ret < 0)
		return ret;
	ret = (ret & ~mask) | (mask & val);
	return bq24261_write_reg(client, reg, ret);
}

static inline int bq24261_enable_charging(
	struct bq24261_charger *chip, bool val)
{
	u8 data;
	if (chip->pdata->enable_charging)
		return chip->pdata->enable_charging(val);

	data = val ? ~BQ24261_CE_DISABLE :
		BQ24261_CE_DISABLE;

	return bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
				       BQ24261_CE_MASK, data);
}

static inline int bq24261_enable_charger(
	struct bq24261_charger *chip, int val)
{
	u8 data;
	dev_dbg(&chip->client->dev, "%s\n", __func__);
	if (chip->pdata->enable_charger)
		return chip->pdata->enable_charger(val);

	data = val ? ~BQ24261_HZ_ENABLE : BQ24261_HZ_ENABLE;

	return bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
				       BQ24261_HZ_MASK, data);
}

static inline int bq24261_set_cc(struct bq24261_charger *chip, int cc)
{
	u8 reg_val;
	if (chip->pdata->set_cc)
		return chip->pdata->set_cc(cc);

	bq24261_cc_to_reg(cc, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_TERM_FCC_ADDR,
				       BQ24261_ICHRG_MASK, reg_val);
}

static inline int bq24261_set_cv(struct bq24261_charger *chip, int cv)
{
	u8 reg_val;

	if (chip->pdata->set_cv)
		return chip->pdata->set_cv(cv);

	bq24261_cv_to_reg(cv, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_BATT_VOL_CTRL_ADDR,
				       BQ24261_VBREG_MASK, reg_val);
}

static inline int bq24261_set_inlmt(struct bq24261_charger *chip, int inlmt)
{
	u8 reg_val;

	if (chip->pdata->set_inlmt)
		return chip->pdata->set_inlmt(inlmt);

	bq24261_inlmt_to_reg(inlmt, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
				       BQ24261_INLMT_MASK, reg_val);

}

static inline int bq24261_set_iterm(struct bq24261_charger *chip, int iterm)
{
	u8 reg_val;

	if (chip->pdata->set_iterm)
		return chip->pdata->set_iterm(iterm);

	bq24261_iterm_to_reg(iterm, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_TERM_FCC_ADDR,
				       BQ24261_ITERM_MASK, reg_val);
}

static inline int bq24261_enable_hw_charge_term(
	struct bq24261_charger *chip, bool val)
{
	u8 data;

	data = val ? BQ24261_TE_ENABLE : ~BQ24261_TE_ENABLE;

	return bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
				       BQ24261_TE_MASK, data);
}

static inline int bq24261_enable_boost_mode(
	struct bq24261_charger *chip, int val)
{
	int ret = 0;


	if (val) {

		/* TODO: Support different Host Mode Current limits */

		bq24261_enable_charger(chip, true);
		ret =
		    bq24261_read_modify_reg(chip->client,
					    BQ24261_STAT_CTRL0_ADDR,
					    BQ24261_BOOST_MASK,
					    BQ24261_ENABLE_BOOST);
		if (ret)
			dev_err(&chip->client->dev,
				"Error(%d) in enabling Boost mode\n", ret);
		else
			chip->boost_mode = true;

		dev_info(&chip->client->dev, "Boost Mode enabled\n");
	} else {
		ret =
		    bq24261_read_modify_reg(chip->client,
					    BQ24261_STAT_CTRL0_ADDR,
					    BQ24261_BOOST_MASK,
					    ~BQ24261_ENABLE_BOOST);
		bq24261_write_reg(chip->client, BQ24261_STAT_CTRL0_ADDR, 0x00);
		if (ret)
			dev_err(&chip->client->dev,
				"Error(%d) in disabling Boost mode\n", ret);
		else {

			/* if charging need not to be enabled, disable
			* the charger else keep the charger on
			*/
			if (!chip->is_charging_enabled)
				bq24261_enable_charger(chip, false);
			chip->boost_mode = false;
			dev_info(&chip->client->dev, "Boost Mode disabled\n");
		}
		/* Notify power supply subsystem to enable charging
		 *  if needed. Eg. if DC adapter is connected
		 */
		power_supply_changed(&chip->psy_usb);
	}

	return ret;
}

static inline bool bq24261_is_vsys_on(struct bq24261_charger *chip)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = bq24261_read_reg(client, BQ24261_STAT_CTRL0_ADDR);
	if (ret < 0) {
		dev_err(&client->dev,
			"Error(%d) in reading BQ24261_CTRL_ADDR\n", ret);
		return false;
	}

	if ((ret & BQ24261_HZ_MASK) == BQ24261_HZ_ENABLE) {
		dev_err(&client->dev, "Charger in Hi Z Mode\n");
		bq24261_dump_regs(true);
		return false;
	}

	ret = bq24261_read_reg(client, BQ24261_VINDPM_STAT_ADDR);
	if (ret < 0) {
		dev_err(&client->dev,
			"Error(%d) in reading BQ24261_VINDPM_STAT_ADDR\n", ret);
		return false;
	}

	if (ret & BQ24261_CD_STATUS_MASK) {
		dev_err(&client->dev, "CD line asserted\n");
		bq24261_dump_regs(true);
		return false;
	}

	return true;
}

int bq24261_get_bat_status(void)
{

	struct bq24261_charger *chip;

	if (!bq24261_client)
		return -ENODEV;

	chip = i2c_get_clientdata(bq24261_client);

	if (chip->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (chip->is_bat_full && !chip->is_fault && chip->max_cc)
		return POWER_SUPPLY_STATUS_FULL;

	if (chip->is_fault || !chip->is_charging_enabled || !chip->max_cc)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	return POWER_SUPPLY_STATUS_CHARGING;
}
EXPORT_SYMBOL(bq24261_get_bat_status);

static inline bool bq24261_is_online(struct bq24261_charger *chip)
{
	if (chip->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
		return false;
	else if (!chip->is_charger_enabled)
		return false;
	/* BQ24261 gives interrupt only on stop/resume charging.
	 * If charging is already stopped, we need to query the hardware
	 * to see charger is still active and can supply vsys or not.
	 */
	else if (chip->is_fault || !chip->is_charging_enabled)
		return bq24261_is_vsys_on(chip);
	else
		return chip->is_vsys_on;
}

static int bq24261_usb_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct bq24261_charger *chip = container_of(psy,
						    struct bq24261_charger,
						    psy_usb);
	int ret = 0;

	mutex_lock(&chip->lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_ENABLE_CHARGING:

		ret = bq24261_enable_charging(chip, val);

		if (ret)
			dev_err(&chip->client->dev,
				"Error(%d) in %s charging", ret,
				(val->intval ? "enable" : "disable"));
		else
			chip->is_charging_enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		ret = bq24261_enable_charger(chip, val->intval);

		if (ret)
			dev_err(&chip->client->dev,
				"Error(%d) in %s charger", ret,
				(val->intval ? "enable" : "disable"));
		else
			chip->is_charger_enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		ret = bq24261_set_cc(chip, val->intval);
		if (!ret)
			chip->cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		ret = bq24261_set_cv(chip, val->intval);
		if (!ret)
			chip->cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		chip->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		chip->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		ret = bq24261_set_iterm(chip, val->intval);
		if (!ret)
			chip->iterm = val->intval;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		chip->cable_type = val->intval;
		chip->psy_usb.type = get_power_supply_type(chip->cable_type);
		break;
	case POWER_SUPPLY_PROP_INLMT:
		ret = bq24261_set_inlmt(chip, val->intval);
		if (!ret)
			chip->inlmt = val->intval;
		break;
	default:
		ret = -ENODATA;
	}

	mutex_unlock(&chip->lock);
	power_supply_changed(&chip->psy_usb);
	return ret;
}

static int bq24261_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct bq24261_charger *chip = container_of(psy,
						    struct bq24261_charger,
						    psy_usb);

	mutex_lock(&chip->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (chip->cable_type
			       != POWER_SUPPLY_CHARGER_TYPE_NONE);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24261_is_online(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->chrgr_health;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		val->intval = chip->max_cc;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		val->intval = chip->max_cv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		val->intval = chip->cc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		val->intval = chip->cv;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		val->intval = chip->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = chip->iterm;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = chip->cable_type;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		if (chip->boost_mode)
			val->intval = false;
		else
			val->intval = chip->is_charging_enabled;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = chip->is_charger_enabled;
		break;

	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->lock);
	return 0;
}

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval;

	return ret;
}

static inline int get_battery_current(int *cur)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		*cur = val.intval;

	return ret;
}

static inline bool _is_battery_full(int pbat_cur, int bat_cur,
			int pbat_volt, int bat_volt)
{
	struct bq24261_charger *chip;

	chip = i2c_get_clientdata(bq24261_client);
	return ((pbat_cur <= chip->iterm) && (bat_cur <= chip->iterm) &&
		(pbat_volt <= chip->cv) && (bat_volt <= chip->cv));
}

static inline bool is_battery_full(struct bq24261_charger *chip)
{
	static int pbat_volt, pbat_cur;
	int bat_cur = 0, bat_volt = 0;
	int ret;

	/* Return true if any communication failure is encountered.
	 * This is to ensure safety by avoiding charging the battery
	 * incase it's FULL
	 */

	ret = get_battery_current(&bat_cur);
	if (ret) {
		dev_err(&chip->client->dev,
			"%s: Error in getting battery current."
			"Reporting battery status as FULL to avoid battery reaching over voltage\n",
			__func__);
		return true;
	}
	ret = get_battery_voltage(&bat_volt);
	if (ret) {
		dev_err(&chip->client->dev,
			"%s: Error in getting battery voltage."
			"Reporting battery status as FULL to avoid battery reaching over voltage\n",
			__func__);
		return true;
	}

	if (_is_battery_full(pbat_cur, bat_cur, pbat_volt, bat_volt)) {
		pbat_volt = pbat_cur = 0;
		dev_info(&chip->client->dev,
			"%s:Battery is Full volt=%d current=%d:\n",
				__func__, bat_volt, bat_cur);
		return true;
	}

	dev_info(&chip->client->dev,
		"%s:Battery is not Full volt=%d current=%d:\n",
			__func__, bat_volt, bat_cur);
	pbat_volt = bat_volt;
	pbat_cur = bat_cur;

	return false;
}

static void bq24261_sw_charge_term_worker(struct work_struct *work)
{

	struct bq24261_charger *chip = container_of(work,
						    struct bq24261_charger,
						    sw_term_work.work);

	if (is_battery_full(chip)) {
		chip->is_bat_full = true;
		/* Battery is full. When charging is resumed hw charge
		 * termination should be used
		 */
		bq24261_enable_hw_charge_term(chip, true);
		power_supply_changed(&chip->psy_usb);

	} else {
		schedule_delayed_work(&chip->sw_term_work,
				      CHRG_TERM_WORKER_DELAY);
	}
}

int bq24261_get_bat_health(void)
{

	struct bq24261_charger *chip;

	if (!bq24261_client)
		return -ENODEV;

	chip = i2c_get_clientdata(bq24261_client);

	return chip->bat_health;
}
EXPORT_SYMBOL(bq24261_get_bat_health);

static int bq24261_handle_irq(struct bq24261_charger *chip, u8 stat_reg)
{
	struct i2c_client *client = chip->client;

	dev_dbg(&client->dev, "%s:%d\n", __func__, __LINE__);

	/* If fault was set previously and it's not set now,
	 *  then recover excpetions
	 */

	if (chip->is_fault && ((stat_reg & BQ24261_STAT_MASK)
				!= BQ24261_STAT_FAULT)) {
		dev_info(&client->dev, "Fault recovered\n");
		chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
		chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		chip->is_fault = false;
		bq24261_set_inlmt(chip, chip->inlmt);
		if (chip->is_charger_enabled)
			bq24261_enable_charger(chip, true);
		power_supply_changed(&chip->psy_usb);
		bq24261_dump_regs(false);
		return 0;
	}

	switch (stat_reg & BQ24261_STAT_MASK) {
	case BQ24261_STAT_READY:
		dev_info(&client->dev, "Charger Status: Ready\n");
		break;
	case BQ24261_STAT_CHRG_PRGRSS:
		dev_info(&client->dev, "Charger Status: Charge Progress\n");
		break;
	case BQ24261_STAT_CHRG_DONE:
		dev_info(&client->dev, "Charger Status: Charge Done\n");

		if (is_battery_full(chip))
			chip->bat_status = POWER_SUPPLY_STATUS_FULL;
		else {
			/* Battery is not full. Disable hardware charge
			 * termination and send power_supply_changed
			 * notification inorder to restart charging
			 */
			dev_info(&chip->client->dev, "Battery is Not Full. Starting SW charge term\n");
			bq24261_enable_hw_charge_term(chip, false);
			chip->is_charging_enabled = false;
			schedule_delayed_work(&chip->sw_term_work, 0);
			power_supply_changed(&chip->psy_usb);
			bq24261_dump_regs(false);
		}

		break;
	case BQ24261_STAT_FAULT:
		chip->is_fault = true;
		dev_err(&client->dev, "Charger Status: Charge Fault\n");
		break;
	}

	if (stat_reg & BQ24261_BOOST_MASK)
		dev_info(&client->dev, "Boost Mode\n");

	if ((stat_reg & BQ24261_STAT_MASK) == BQ24261_STAT_FAULT) {
		bool dump_master = true;

		switch (stat_reg & BQ24261_FAULT_MASK) {
		case BQ24261_VOVP:
			chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			dev_err(&client->dev, "Charger OVP Fault\n");
			break;

		case BQ24261_LOW_SUPPLY:
			dump_master = false;
			chip->chrgr_health = POWER_SUPPLY_HEALTH_DEAD;
			dev_err(&client->dev, "Charger Low Supply Fault\n");
			break;

		case BQ24261_THERMAL_SHUTDOWN:
			chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			dev_err(&client->dev, "Charger Low Supply Fault\n");
			break;

		case BQ24261_BATT_TEMP_FAULT:
			chip->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			dev_err(&client->dev, "Battery Temperature Fault\n");
			break;

		case BQ24261_TIMER_FAULT:
			chip->bat_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			chip->chrgr_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			dev_err(&client->dev, "Charger Timer Fault\n");
			break;

		case BQ24261_BATT_OVP:
			chip->bat_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			dev_err(&client->dev, "Battery Over Voltage Fault\n");
			break;
		case BQ24261_NO_BATTERY:
			dev_err(&client->dev, "No Battery Connected\n");
			break;

		}

		bq24261_dump_regs(dump_master);
		power_supply_changed(&chip->psy_usb);
	}

	chip->is_vsys_on = bq24261_is_vsys_on(chip);

	return 0;
}

static void bq24261_irq_worker(struct work_struct *work)
{
	struct bq24261_charger *chip =
	    container_of(work, struct bq24261_charger, irq_work);
	int ret;

	/*Lock to ensure that interrupt register readings are done
	* and processed sequentially. The interrupt Fault registers
	* are read on clear and without sequential processing double
	* fault interrupts or fault recovery cannot be handlled propely
	*/

	mutex_lock(&chip->lock);

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	ret = bq24261_read_reg(chip->client, BQ24261_STAT_CTRL0_ADDR);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"Error (%d) in reading BQ24261_STAT_CTRL0_ADDR\n", ret);
	else
		bq24261_handle_irq(chip, ret);

	mutex_unlock(&chip->lock);
}

static irqreturn_t bq24261_thread_handler(int id, void *data)
{
	struct bq24261_charger *chip = (struct bq24261_charger *)data;

	/* FIXME: Move this to platform layer */
#define IRQLVL1_MASK_ADDR		0x0c
#define IRQLVL1_CHRGR_MASK		(0x01 << 5)


	dev_dbg(&chip->client->dev,
		"%s:Clearing IRQLVL1_MASK_ADDR\n", __func__);
	intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
			IRQLVL1_CHRGR_MASK);

	queue_work(system_nrt_wq, &chip->irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t bq24261_irq_handler(int irq, void *data)
{
	struct bq24261_charger *chip = (struct bq24261_charger *)data;
	u8 intr_stat;

	if (chip->irq_iomap) {
		intr_stat = ioread8(chip->irq_iomap);
		if ((intr_stat & chip->pdata->irq_mask)) {
			dev_dbg(&chip->client->dev, "%s\n", __func__);
			return IRQ_WAKE_THREAD;
		}
	}

	return IRQ_NONE;
}

static void bq24261_boostmode_worker(struct work_struct *work)
{
	struct bq24261_charger *chip =
	    container_of(work, struct bq24261_charger, otg_work);
	struct bq24261_otg_event *evt, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chip->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &chip->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&chip->otg_queue_lock, flags);

		dev_info(&chip->client->dev,
			"%s:%d state=%d\n", __FILE__, __LINE__,
				evt->cap->chrg_state);
		mutex_lock(&chip->lock);
		if (evt->cap->chrg_state == OTG_CHR_STATE_HOST)
			bq24261_enable_boost_mode(chip, 1);
		else if (evt->cap->chrg_state == OTG_CHR_STATE_DISCONNECTED)
			bq24261_enable_boost_mode(chip, 0);
		mutex_unlock(&chip->lock);
		spin_lock_irqsave(&chip->otg_queue_lock, flags);
		kfree(evt);

	}
	spin_unlock_irqrestore(&chip->otg_queue_lock, flags);
}

static int otg_handle_notification(struct notifier_block *nb,
				   unsigned long event, void *param)
{

	struct bq24261_charger *chip =
	    container_of(nb, struct bq24261_charger, otg_nb);
	struct bq24261_otg_event *evt;

	dev_dbg(&chip->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&chip->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->cap = (struct otg_bc_cap *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&chip->otg_queue_lock);
	list_add_tail(&evt->node, &chip->otg_queue);
	spin_unlock(&chip->otg_queue_lock);

	queue_work(system_nrt_wq, &chip->otg_work);
	return NOTIFY_OK;
}

static inline int register_otg_notifications(struct bq24261_charger *chip)
{

	int retval;

	INIT_LIST_HEAD(&chip->otg_queue);
	INIT_WORK(&chip->otg_work, bq24261_boostmode_worker);
	spin_lock_init(&chip->otg_queue_lock);

	chip->otg_nb.notifier_call = otg_handle_notification;

	chip->transceiver = otg_get_transceiver();
	if (!chip->transceiver) {
		dev_err(&chip->client->dev, "failed to get otg transceiver\n");
		return -EINVAL;
	}
	retval = otg_register_notifier(chip->transceiver, &chip->otg_nb);
	if (retval) {
		dev_err(&chip->client->dev,
			"failed to register otg notifier\n");
		return -EINVAL;
	}

	return 0;
}

static int bq24261_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct bq24261_charger *chip;
	int ret;

	adapter = to_i2c_adapter(client->dev.parent);

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform data is null");
		return -EFAULT;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"I2C adapter %s doesn'tsupport BYTE DATA transfer\n",
			adapter->name);
		return -EIO;
	}

	ret = bq24261_read_reg(client, BQ24261_VENDOR_REV_ADDR);
	if (ret < 0) {
		dev_err(&client->dev,
			"Error (%d) in reading BQ24261_VENDOR_REV_ADDR\n", ret);
		return ret;
	}

	if (((ret & BQ24261_VENDOR_MASK) != BQ24261_VENDOR) ||
	    ((ret & BQ24261_REV_MASK) != BQ24261_REV)) {
		dev_err(&client->dev,
			"Invalid Vendor/Revision number in BQ24261_VENDOR_REV_ADDR: %d",
			ret);
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->pdata = client->dev.platform_data;

	/* Remap IRQ map address to read the IRQ status */
	if ((chip->pdata->irq_map) && (chip->pdata->irq_mask)) {
		chip->irq_iomap = ioremap_nocache(chip->pdata->irq_map, 8);
		if (!chip->irq_iomap) {
			dev_err(&client->dev, "Failed: ioremap_nocache\n");
			return -EFAULT;
		}
	}

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	chip->psy_usb.name = DEV_NAME;
	chip->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	chip->psy_usb.properties = bq24261_usb_props;
	chip->psy_usb.num_properties = ARRAY_SIZE(bq24261_usb_props);
	chip->psy_usb.get_property = bq24261_usb_get_property;
	chip->psy_usb.set_property = bq24261_usb_set_property;
	chip->psy_usb.supplied_to = chip->pdata->supplied_to;
	chip->psy_usb.num_supplicants = chip->pdata->num_supplicants;
	chip->psy_usb.throttle_states = chip->pdata->throttle_states;
	chip->psy_usb.num_throttle_states = chip->pdata->num_throttle_states;
	chip->psy_usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
	chip->max_cc = 1500;

	mutex_init(&chip->lock);
	ret = power_supply_register(&client->dev, &chip->psy_usb);
	if (ret) {
		dev_err(&client->dev, "Failed: power supply register (%d)\n",
			ret);
		iounmap(chip->irq_iomap);
		return ret;
	}

	INIT_DELAYED_WORK(&chip->sw_term_work, bq24261_sw_charge_term_worker);

	INIT_WORK(&chip->irq_work, bq24261_irq_worker);
	if (chip->client->irq) {
		ret = request_threaded_irq(chip->client->irq,
					   bq24261_irq_handler,
					   bq24261_thread_handler,
					   IRQF_SHARED,
					   DEV_NAME, chip);
		if (ret) {
			dev_err(&client->dev, "Failed: request_irq (%d)\n",
				ret);
			iounmap(chip->irq_iomap);
			power_supply_unregister(&chip->psy_usb);
			return ret;
		}
	/* FIXME: Move this to platform layer */
#define MCHGRIRQ0_ADDR			0x12
		/* unmask charger interrupts in second level IRQ register*/
		intel_scu_ipc_update_register(MCHGRIRQ0_ADDR, 0x00,
				chip->pdata->irq_mask);
	}

	if (register_otg_notifications(chip))
		dev_err(&client->dev, "Error in registering OTG notifications. Unable to supply power to Host\n");

	bq24261_client = client;
	power_supply_changed(&chip->psy_usb);
	bq24261_debugfs_init();

	return 0;
}

static int __devexit bq24261_remove(struct i2c_client *client)
{
	struct bq24261_charger *chip = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, chip);

	flush_scheduled_work();
	if (chip->irq_iomap)
		iounmap(chip->irq_iomap);
	if (chip->transceiver)
		otg_unregister_notifier(chip->transceiver, &chip->otg_nb);

	power_supply_unregister(&chip->psy_usb);
	bq24261_debugfs_exit();
	return 0;
}

static int bq24261_suspend(struct device *dev)
{
	struct bq24261_charger *chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "bq24261 suspend\n");
	return 0;
}

static int bq24261_resume(struct device *dev)
{
	struct bq24261_charger *chip = dev_get_drvdata(dev);

	dev_dbg(&chip->client->dev, "bq24261 resume\n");
	return 0;
}

static int bq24261_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24261_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24261_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops bq24261_pm_ops = {
	.suspend = bq24261_suspend,
	.resume = bq24261_resume,
	.runtime_suspend = bq24261_runtime_suspend,
	.runtime_resume = bq24261_runtime_resume,
	.runtime_idle = bq24261_runtime_idle,
};

static const struct i2c_device_id bq24261_id[] = {
	{DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq24261_id);

static struct i2c_driver bq24261_driver = {
	.driver = {
		   .name = DEV_NAME,
		   .pm = &bq24261_pm_ops,
		   },
	.probe = bq24261_probe,
	.remove = __devexit_p(bq24261_remove),
	.id_table = bq24261_id,
};

static int __init bq24261_init(void)
{
	return i2c_add_driver(&bq24261_driver);
}

late_initcall(bq24261_init);

static void __exit bq24261_exit(void)
{
	i2c_del_driver(&bq24261_driver);
}

module_exit(bq24261_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("BQ24261 Charger Driver");
MODULE_LICENSE("GPL");
