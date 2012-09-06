/*
 * basin_cove_charger.c - Intel MID Basin Cove PMIC Charger Driver
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/ipc_device.h>
#include <linux/usb/otg.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/power/charger_helper.h>
#include <linux/power_supply.h>
#include <asm/intel_basincove_gpadc.h>
#include <asm/intel_scu_ipc.h>
#include <linux/io.h>
#include <linux/power/intel_mid_powersupply.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/sfi.h>
#include <linux/async.h>
#include <linux/reboot.h>

#include "basin_cove_charger.h"

#define CHARGER_PS_NAME "bcove_charger"
#define DRIVER_NAME "bcove_chrgr"

#define BQ24260_NAME "bq24260"
#define PMIC_SRAM_INTR_ADDR 0xFFFFF616

static DEFINE_MUTEX(pmic_lock);

static struct bc_chrgr_drv_context chc;

/*
 * Basin Cove Charger power supply  properties
 */
static enum power_supply_property bc_chrgr_ps_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};


u16 bcove_inlmt[][2] = {
	{ 100, CHGRCTRL1_FUSB_INLMT_100_MASK},
	{ 150, CHGRCTRL1_FUSB_INLMT_150_MASK},
	{ 500, CHGRCTRL1_FUSB_INLMT_500_MASK},
	{ 900, CHGRCTRL1_FUSB_INLMT_900_MASK},
	{ 1500, CHGRCTRL1_FUSB_INLMT_1500_MASK},
};


/*external charger specific definitions */
static int bq24260_disable_charging(u8 dev_addr);
static int bq24260_enable_charging(u8 dev_addr);
static int bq24260_handle_irq(u8 dev_addr);
static int bq24260_cc_to_reg(int cc, u8 *reg_val);
static int bq24260_cv_to_reg(int cv, u8 *reg_val);
static bool bq24260_is_batt_charging(u8 dev_addr);

u16 bq24260_cc[][2] = {

	{500, 0x00},
	{600, BQ24260_ICHRG_100mA},
	{700, BQ24260_ICHRG_200mA},
	{800, BQ24260_ICHRG_100mA | BQ24260_ICHRG_200mA},
	{900, BQ24260_ICHRG_400mA},
	{1000, BQ24260_ICHRG_400mA | BQ24260_ICHRG_100mA},
	{1100, BQ24260_ICHRG_400mA | BQ24260_ICHRG_200mA},
	{1200, BQ24260_ICHRG_400mA | BQ24260_ICHRG_200mA | BQ24260_ICHRG_100mA},
	{1300, BQ24260_ICHRG_800mA},
	{1400, BQ24260_ICHRG_800mA | BQ24260_ICHRG_100mA},
	{1500, BQ24260_ICHRG_800mA | BQ24260_ICHRG_200mA},
};

u16 bq24260_cv[][2] = {

	{3500, 0x00},
	{3600, BQ24260_VBREG_20mV | BQ24260_VBREG_80mV},
	{3700, BQ24260_VBREG_160mV | BQ24260_VBREG_40mV},
	{3800,
	 BQ24260_VBREG_160mV | BQ24260_VBREG_80mV | BQ24260_VBREG_40mV |
	 BQ24260_VBREG_20mV},
	{3900, BQ24260_VBREG_320mV | BQ24260_VBREG_80mV},
	{4000, BQ24260_VBREG_320mV | BQ24260_VBREG_160mV | BQ24260_VBREG_20mV},
	{4040,
	 BQ24260_VBREG_320mV | BQ24260_VBREG_160mV | BQ24260_VBREG_40mV |
	 BQ24260_VBREG_20mV},
	{4060, BQ24260_VBREG_320mV | BQ24260_VBREG_160mV | BQ24260_VBREG_80mV},
	{4100,
	 BQ24260_VBREG_320mV | BQ24260_VBREG_160mV | BQ24260_VBREG_80mV |
	 BQ24260_VBREG_40mV},
	{4140, BQ24260_VBREG_640mV},
	{4160, BQ24260_VBREG_640mV | BQ24260_VBREG_20mV},
	{4200, BQ24260_VBREG_640mV | BQ24260_VBREG_40mV | BQ24260_VBREG_20mV},
};


struct ext_charger bq24260_chrgr = {
	.enable_charging = bq24260_enable_charging,
	.disable_charging = bq24260_disable_charging,
	.handle_irq = bq24260_handle_irq,
	.cc_to_reg = bq24260_cc_to_reg,
	.cv_to_reg = bq24260_cv_to_reg,
	.is_batt_charging = bq24260_is_batt_charging,
};

static char *bcove_charger_power_supplied_to[] = {"max170xx_battery",};

#ifdef CONFIG_DEBUG_FS

static int ext_chrgr_reg_open(struct inode *inode, struct file *file);
static int pmic_chrgr_reg_open(struct inode *inode, struct file *file);
static int pmic_chrgr_tt_reg_open(struct inode *inode, struct file *file);

static inline int bcove_extchrgr_read(u8 dev_id, u8 offset, u8 *data);
static inline int bcove_read_tt(u8 addr, u8 *data);

static const struct file_operations ext_chrgr_reg_fops = {
	.open = ext_chrgr_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations pmic_chrgr_reg_fops = {
	.open = pmic_chrgr_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations pmic_chrgr_tt_reg_fops = {
	.open = pmic_chrgr_tt_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static u8 basin_cove_regs[] = {
	CHGRIRQ0_ADDR,
	SCHGRIRQ0_ADDR,
	MCHGRIRQ0_ADDR,
	LOWBATTDET0_ADDR,
	LOWBATTDET1_ADDR,
	BATTDETCTRL_ADDR,
	VBUSDETCTRL_ADDR,
	VDCINDETCTRL_ADDR,
	CHRGRIRQ1_ADDR,
	SCHGRIRQ1_ADDR,
	MCHGRIRQ1_ADDR,
	CHGRCTRL0_ADDR,
	CHGRCTRL1_ADDR,
	CHGRSTATUS_ADDR,
	THRMBATZONE_ADDR,
	THRMZN0L_ADDR,
	THRMZN0H_ADDR,
	THRMZN1L_ADDR,
	THRMZN1H_ADDR,
	THRMZN2L_ADDR,
	THRMZN2H_ADDR,
	THRMZN3L_ADDR,
	THRMZN3H_ADDR,
	THRMZN4L_ADDR,
	THRMZN4H_ADDR,
};

static u8 basin_cove_tt_regs[] = {
	TT_I2CDADDR_ADDR,
	TT_CHGRINIT0OS_ADDR,
	TT_CHGRINIT1OS_ADDR,
	TT_CHGRINIT2OS_ADDR,
	TT_CHGRINIT3OS_ADDR,
	TT_CHGRINIT4OS_ADDR,
	TT_CHGRINIT5OS_ADDR,
	TT_CHGRINIT6OS_ADDR,
	TT_CHGRINIT7OS_ADDR,
	TT_USBINPUTICCOS_ADDR,
	TT_USBINPUTICCMASK_ADDR,
	TT_CHRCVOS_ADDR,
	TT_CHRCVMASK_ADDR,
	TT_CHRCCOS_ADDR,
	TT_CHRCCMASK_ADDR,
	TT_LOWCHROS_ADDR,
	TT_LOWCHRMASK_ADDR,
	TT_WDOGRSTOS_ADDR,
	TT_WDOGRSTMASK_ADDR,
	TT_CHGRENOS_ADDR,
	TT_CHGRENMASK_ADDR,
	TT_CUSTOMFIELDEN_ADDR,
	TT_CHGRINIT0VAL_ADDR,
	TT_CHGRINIT1VAL_ADDR,
	TT_CHGRINIT2VAL_ADDR,
	TT_CHGRINIT3VAL_ADDR,
	TT_CHGRINIT4VAL_ADDR,
	TT_CHGRINIT5VAL_ADDR,
	TT_CHGRINIT6VAL_ADDR,
	TT_CHGRINIT7VAL_ADDR,
	TT_USBINPUTICC100VAL_ADDR,
	TT_USBINPUTICC150VAL_ADDR,
	TT_USBINPUTICC500VAL_ADDR,
	TT_USBINPUTICC900VAL_ADDR,
	TT_USBINPUTICC1500VAL_ADDR,
	TT_CHRCVEMRGLOWVAL_ADDR,
	TT_CHRCVCOLDVAL_ADDR,
	TT_CHRCVCOOLVAL_ADDR,
	TT_CHRCVWARMVAL_ADDR,
	TT_CHRCVHOTVAL_ADDR,
	TT_CHRCVEMRGHIVAL_ADDR,
	TT_CHRCCEMRGLOWVAL_ADDR,
	TT_CHRCCCOLDVAL_ADDR,
	TT_CHRCCCOOLVAL_ADDR,
	TT_CHRCCWARMVAL_ADDR,
	TT_CHRCCHOTVAL_ADDR,
	TT_CHRCCEMRGHIVAL_ADDR,
	TT_LOWCHRENVAL_ADDR,
	TT_LOWCHRDISVAL_ADDR,
};

static u16 bq24260_regs[] = {
	BQ24260_STAT_CTRL0_ADDR,
	BQ24260_CTRL_ADDR,
	BQ24260_BATT_VOL_CTRL_ADDR,
	BQ24260_VENDOR_REV_ADDR,
	BQ24260_TERM_FCC_ADDR,
	BQ24260_VINDPM_DPPM_STATUS_ADDR,
	BQ24260_ST_NTC_MON_ADDR,
};

struct dentry *charger_debug_dir;

static int pmic_chrgr_tt_reg_show(struct seq_file *seq, void *unused)
{
	int ret;
	u8 addr;
	u8 val;

	addr = *(u8 *)seq->private;

	ret = bcove_read_tt(addr, &val);
	if (ret != 0) {
		dev_err(chc.dev,
			"Error reading the register 0x%04x\n",
			addr);
		return -EIO;
	}

	seq_printf(seq, "0x%02x\n", val);

	return 0;
}

static int pmic_chrgr_reg_show(struct seq_file *seq, void *unused)
{
	int ret;
	u16 addr;
	u8 val;

	addr = *(u16 *)seq->private;

	ret = intel_scu_ipc_ioread8(addr, &val);
	if (ret != 0) {
		dev_err(chc.dev,
			"Error reading the register 0x%04x\n",
			addr);
		return -EIO;
	}

	seq_printf(seq, "0x%02x\n", val);

	return 0;
}

static int ext_chrgr_reg_show(struct seq_file *seq, void *unused)
{
	int ret;
	u8 addr;
	u8 val;

	addr = *(u8 *)seq->private;

	ret = bcove_extchrgr_read(chc.ext_chrgr_addr, addr, &val);
	if (ret != 0) {
		dev_err(chc.dev,
			"Error reading the register 0x%02x\n",
			addr);
		return -EIO;
	}

	seq_printf(seq, "0x%02x\n", val);

	return 0;
}

static int pmic_chrgr_tt_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_chrgr_tt_reg_show, inode->i_private);
}

static int pmic_chrgr_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_chrgr_reg_show, inode->i_private);
}

static int ext_chrgr_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, ext_chrgr_reg_show, inode->i_private);
}

static void bcove_debugfs_init(void)
{
	struct dentry *fentry;
	struct dentry *ext_charger_dir;
	struct dentry *pmic_regs_dir;
	struct dentry *pmic_tt_regs_dir;

	u32 reg_index;
	u32 bq24260_reg_cnt = ARRAY_SIZE(bq24260_regs);
	u32 pmic_reg_cnt = ARRAY_SIZE(basin_cove_regs);
	u32 pmic_tt_reg_cnt = ARRAY_SIZE(basin_cove_tt_regs);
	char name[6] = {0};

	/* Creating a directory under debug fs for charger */
	charger_debug_dir = debugfs_create_dir(CHARGER_PS_NAME , NULL) ;
	if (charger_debug_dir == NULL)
		goto debugfs_root_exit;

	/* Create a directory for external charger registers */
	ext_charger_dir = debugfs_create_dir("ext_chrgr_regs",
			charger_debug_dir);

	if (ext_charger_dir == NULL)
		goto debugfs_err_exit;

	for (reg_index = 0; reg_index < bq24260_reg_cnt; reg_index++) {

		snprintf(name, sizeof(name), "%02x",
				bq24260_regs[reg_index]);

		fentry = debugfs_create_file(name,
				S_IRUGO,
				ext_charger_dir,
				&bq24260_regs[reg_index],
				&ext_chrgr_reg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	/* Create a directory for pmic charger registers */
	pmic_regs_dir = debugfs_create_dir("pmic_chrgr_regs",
			charger_debug_dir);

	if (pmic_regs_dir == NULL)
		goto debugfs_err_exit;

	for (reg_index = 0; reg_index < pmic_reg_cnt; reg_index++) {

		snprintf(name, sizeof(name), "%04x",
				basin_cove_regs[reg_index]);

		fentry = debugfs_create_file(name,
				S_IRUGO,
				pmic_regs_dir,
				&basin_cove_regs[reg_index],
				&pmic_chrgr_reg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	/* Create a directory for pmic tt charger registers */
	pmic_tt_regs_dir = debugfs_create_dir("pmic_chrgr_tt_regs",
			charger_debug_dir);

	if (pmic_tt_regs_dir == NULL)
		goto debugfs_err_exit;

	for (reg_index = 0; reg_index < pmic_tt_reg_cnt; reg_index++) {

		snprintf(name, sizeof(name), "%04x",
				basin_cove_tt_regs[reg_index]);

		fentry = debugfs_create_file(name,
				S_IRUGO,
				pmic_tt_regs_dir,
				&basin_cove_tt_regs[reg_index],
				&pmic_chrgr_tt_reg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	dev_info(chc.dev, "Debugfs created successfully!!");
	return;

debugfs_err_exit:
	debugfs_remove_recursive(charger_debug_dir);
debugfs_root_exit:
	dev_err(chc.dev, "Error creating debugfs entry!!");
	return;
}

static void bcove_debugfs_exit(void)
{
	if (charger_debug_dir != NULL)
		debugfs_remove_recursive(charger_debug_dir);
}

#else

static void bcove_debugfs_init(void)
{
	return;
}

static void bcove_debugfs_exit(void)
{
	return;
}

#endif


/* Generic function definitions */
static void lookup_regval(u16 tbl[][2], size_t size, u16 in_val, u8 *out_val)
{
	int i;
	for (i = 1; i < size; ++i)
		if (in_val < tbl[i][0])
			break;

	*out_val = (u8)tbl[i-1][1];
}

static int interpolate_y(int dx1x0, int dy1y0, int dxx0, int y0)
{
	return y0 + DIV_ROUND_CLOSEST((dxx0 * dy1y0), dx1x0);
}

static int interpolate_x(int dy1y0, int dx1x0, int dyy0, int x0)
{
	return x0 + DIV_ROUND_CLOSEST((dyy0 * dx1x0), dy1y0);
}

#define is_valid_temp(tmp)\
	(!(tmp > adc_tbl[0].temp ||\
		tmp < adc_tbl[ARRAY_SIZE(adc_tbl) - 1].temp))

#define is_valid_adc_code(val)\
	(!(val < adc_tbl[0].adc_val ||\
		val > adc_tbl[ARRAY_SIZE(adc_tbl) - 1].adc_val))

#define ADC_TO_TEMP 1
#define TEMP_TO_ADC 0

#define CONVERT_ADC_TO_TEMP(adc_val, temp)\
	adc_temp_conv(adc_val, temp, ADC_TO_TEMP)

#define CONVERT_TEMP_TO_ADC(temp, adc_val)\
	adc_temp_conv(temp, adc_val, TEMP_TO_ADC)

static int adc_temp_conv(int in_val, int *out_val, int conv)
{
	int tbl_row_cnt = ARRAY_SIZE(adc_tbl), i;

	if (conv == ADC_TO_TEMP) {
		if (!is_valid_adc_code(in_val))
			return -ERANGE;

		if (in_val == adc_tbl[tbl_row_cnt-1].adc_val)
			i = tbl_row_cnt - 1;
		else {
			for (i = 0; i < tbl_row_cnt; ++i)
				if (in_val < adc_tbl[i].adc_val)
					break;
		}

		*out_val =
		    interpolate_y((adc_tbl[i].adc_val - adc_tbl[i - 1].adc_val),
				  (adc_tbl[i].temp - adc_tbl[i - 1].temp),
				  (in_val - adc_tbl[i - 1].adc_val),
				  adc_tbl[i - 1].temp);
	} else {
		if (!is_valid_temp(in_val))
			return -ERANGE;

		if (in_val == adc_tbl[tbl_row_cnt-1].temp)
			i = tbl_row_cnt - 1;
		else {
			for (i = 0; i < tbl_row_cnt; ++i)
				if (in_val > adc_tbl[i].temp)
					break;
		}

		*out_val =
		    interpolate_x((adc_tbl[i].temp - adc_tbl[i - 1].temp),
				  (adc_tbl[i].adc_val - adc_tbl[i - 1].adc_val),
				  (in_val - adc_tbl[i - 1].temp),
				  adc_tbl[i - 1].adc_val);
	}
	return 0;
}

/* PMIC External charger function definitions */
static int __bcove_extchrgr_read(u8 dev_id, u8 offset, u8 *data)
{
	int ret = 0;

	ret = intel_scu_ipc_iowrite8(I2COVRDADDR_ADDR, dev_id);
	if (unlikely(ret))
		return ret;

	ret = intel_scu_ipc_iowrite8(I2COVROFFSET_ADDR, offset);
	if (unlikely(ret))
		return ret;

	atomic_set(&chc.i2c_rw, 0);

	ret = intel_scu_ipc_iowrite8(I2COVRCTRL_ADDR, I2COVRCTRL_I2C_RD);
	if (unlikely(ret))
		return ret;

	ret = wait_event_timeout(chc.i2c_wait,
				 (atomic_read(&chc.i2c_rw) == I2C_RD), HZ);
	if (unlikely(!ret)) {
		dev_err(chc.dev, "External I2C read timed out:%d\n", ret);
		return  -ETIMEDOUT;
	}

	return intel_scu_ipc_ioread8(I2COVRRDDATA_ADDR, data);
}

static inline int bcove_extchrgr_read(u8 dev_id, u8 offset, u8 *data)
{
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_extchrgr_read(dev_id, offset, data);
	mutex_unlock(&pmic_lock);
	return ret;
}

static int __bcove_extchrgr_write(u8 dev_id, u8 offset, u8 data)
{
	int ret = 0;


	ret = intel_scu_ipc_iowrite8(I2COVRDADDR_ADDR, dev_id);
	if (unlikely(ret))
		return ret;

	ret = intel_scu_ipc_iowrite8(I2COVROFFSET_ADDR, offset);
	if (unlikely(ret))
		return ret;

	ret = intel_scu_ipc_iowrite8(I2COVRWRDATA_ADDR, data);
	if (unlikely(ret))
		return ret;

	atomic_set(&chc.i2c_rw, 0);

	ret = intel_scu_ipc_iowrite8(I2COVRCTRL_ADDR, I2COVRCTRL_I2C_WR);
	if (unlikely(ret))
		return ret;

	ret = wait_event_timeout(chc.i2c_wait,
				 (atomic_read(&chc.i2c_rw) == I2C_WR), HZ);
	if (unlikely(!ret)) {
		dev_err(chc.dev, "External I2C write timed out:%d\n", ret);
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int bcove_extchrgr_write(u8 dev_id, u8 offset, u8 data)
{
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_extchrgr_write(dev_id, offset, data);
	mutex_unlock(&pmic_lock);
	return ret;
}

static int bcove_ext_chrgr_update_reg(u8 dev_id, u8 offset, u8 mask, u8 value)
{
	u8 data;
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_extchrgr_read(dev_id, offset, &data);
	if (ret)
		goto exit;
	data = (data & mask) | value;

	ret = __bcove_extchrgr_write(dev_id, offset, data);
exit:
	mutex_unlock(&pmic_lock);
	return ret;
}

static int __bcove_write_tt(u8 addr, u8 data)
{
	int ret;
	ret = intel_scu_ipc_iowrite8(CHRTTADDR_ADDR, addr);
	if (unlikely(ret))
		return ret;
	return intel_scu_ipc_iowrite8(CHRTTDATA_ADDR, data);
}

static inline int bcove_write_tt(u8 addr, u8 data)
{
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_write_tt(addr, data);
	mutex_unlock(&pmic_lock);
	return ret;
}

static int __bcove_read_tt(u8 addr, u8 *data)
{
	int ret;
	ret = intel_scu_ipc_iowrite8(CHRTTADDR_ADDR, addr);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	return intel_scu_ipc_ioread8(CHRTTDATA_ADDR, data);
}

static inline int bcove_read_tt(u8 addr, u8 *data)
{
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_read_tt(addr, data);
	mutex_unlock(&pmic_lock);
	return ret;
}

static int bcove_update_tt(u8 addr, u8 mask, u8 data)
{
	u8 tdata;
	int ret;
	mutex_lock(&pmic_lock);
	ret = __bcove_read_tt(addr, &data);
	if (unlikely(ret))
		goto exit;
	tdata = (tdata & mask) | data;
	ret = __bcove_write_tt(addr, data);
exit:
	mutex_unlock(&pmic_lock);
	return ret;
}


/* External charger specific definitions */
static int bq24260_cc_to_reg(int cc, u8 *reg_val)
{
	lookup_regval(bq24260_cc, ARRAY_SIZE(bq24260_cc), cc, reg_val);

	/* Return 0 in all cases just to keep it
	 * aligned with callback function prototype */
	return 0;
}
static int bq24260_cv_to_reg(int cv, u8 *reg_val)
{
	lookup_regval(bq24260_cv, ARRAY_SIZE(bq24260_cv), cv, reg_val);

	/* Return 0 in all cases just to keep it
	 * aligned with callback function prototype */
	return 0;
}

static bool bq24260_is_batt_charging(u8 dev_addr)
{
	u8 data;
	bool stat = 0;
	int ret =  bcove_extchrgr_read(dev_addr,
				BQ24260_STAT_CTRL0_ADDR, &data);
	if (!ret)
		stat = ((data & BQ24260_STAT_MASK) == BQ24260_STAT_CHRG_PRGRSS);
	else
		dev_err(chc.dev, "BQ24261: Error in reading charger status. Reporting battery status as not charging\n");
	return stat;
}

static int bq24260_disable_charging(u8 dev_addr)
{
	return bcove_ext_chrgr_update_reg(dev_addr,
			BQ24260_CTRL_ADDR, (~BQ24260_CE_MASK & 0xFF),
					  BQ24260_CE_DISABLE);
}

static int bq24260_enable_charging(u8 dev_addr)
{

	return bcove_ext_chrgr_update_reg(dev_addr,
			BQ24260_CTRL_ADDR, (~BQ24260_CE_MASK & 0XFF),
					  BQ24260_CE_ENABLE);
}

static void bq24260_fault_recovered(void)
{

	dev_dbg(chc.dev, "%s:%d\n", __func__, __LINE__);
	charger_helper_report_exception(chc.ch_handle,
					CH_SOURCE_TYPE_CHARGER,
					POWER_SUPPLY_HEALTH_GOOD, false);

	/* BQ24261 reports only the OVP exception for battery. If fault
	 *  bit is not set then recover exception
	 */

	charger_helper_report_exception(chc.ch_handle,
					CH_SOURCE_TYPE_BATTERY,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, true);
}

static int bq24260_handle_irq(u8 dev_addr)
{
	u8 stat_reg;
	static bool prev_fault_stat;
	int ret = 0;

	dev_dbg(chc.dev, "%s:%d\n", __func__, __LINE__);

	ret = bcove_extchrgr_read(dev_addr, BQ24260_STAT_CTRL0_ADDR,
						&stat_reg);
	if (ret)
		return ret;

	/* If fault was set previously and it's not set now,
	 *  then recover excpetions
	 */

	if (prev_fault_stat && !(stat_reg & BQ24260_FAULT_MASK)) {
		bq24260_fault_recovered();
		prev_fault_stat = false;
		return 0;
	}

	switch (stat_reg & BQ24260_STAT_MASK) {
	case BQ24260_STAT_READY:
		dev_info(chc.dev, "BQ24261: Charger Status: Ready\n");
		break;
	case BQ24260_STAT_CHRG_PRGRSS:
		dev_info(chc.dev, "BQ24261: Charger Status: Charge Progress\n");
		break;
	case BQ24260_STAT_CHRG_DONE:
		charger_helper_report_battery_full(chc.ch_handle);
		dev_info(chc.dev, "BQ24261: Charger Status: Charge Done\n");
		break;
	case BQ24260_STAT_FAULT:
		prev_fault_stat = true;
		dev_err(chc.dev, "BQ24261: Charger Status: Charge Faut\n");
		break;
	}

	if (stat_reg & BQ24260_FAULT_MASK) {
		switch (stat_reg & BQ24260_FAULT_MASK) {
		case BQ24260_VOVP:
			charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_CHARGER,
						POWER_SUPPLY_HEALTH_OVERVOLTAGE,
						0);
			dev_err(chc.dev, "BQ24261: Charger OVP Fault\n");
			break;

		case BQ24260_LOW_SUPPLY:
			charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_CHARGER,
						POWER_SUPPLY_HEALTH_DEAD,
						0);
			dev_err(chc.dev, "BQ24261: Charger Low Supply Fault\n");
			break;

		case BQ24260_THERMAL_SHUTDOWN:
			charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_CHARGER,
						POWER_SUPPLY_HEALTH_OVERHEAT,
						0);
			dev_err(chc.dev, "BQ24261: Charger Low Supply Fault\n");
			break;

			/* This shouldn't happen since the battery
			 * temperature event is handled by PMIC
			 */
		case BQ24260_BATT_TEMP_FAULT:
			dev_err(chc.dev,
				"BQ24261: Battery Temperature Fault\n");
			break;

		case BQ24260_TIMER_FAULT:
			charger_helper_report_exception(chc.ch_handle,
					CH_SOURCE_TYPE_BATTERY,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
					0);
			dev_err(chc.dev, "BQ24261: Charger Timer Fault\n");
			break;

		case BQ24260_BATT_OVP:
			charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_BATTERY,
						POWER_SUPPLY_HEALTH_OVERVOLTAGE,
						0);
			dev_err(chc.dev,
				"BQ24261: Battery Over Voltage Fault\n");
			break;
		case BQ24260_NO_BATTERY:
			dev_err(chc.dev, "BQ24261: No Battery Connected\n");
			break;

		}
	}

	if (stat_reg & BQ24260_BOOST_MASK)
		dev_err(chc.dev, "BQ24261: Boot Mode\n");

	return 0;
}

/* External Charger I2C communication APIs */
static void bcove_bat_zone_changed(void)
{
	u8 data = 0;
	int retval;
	static int prev_zone;
	int cur_zone;

	retval = intel_scu_ipc_ioread8(THRMBATZONE_ADDR, &data);
	if (retval) {
		dev_err(chc.dev, "Error in reading battery zone\n");
		return;
	}

	cur_zone = data & THRMBATZONE_MASK;
	dev_info(chc.dev, "Thermal Zone changed. Current zone is %d\n",
		 (data & THRMBATZONE_MASK));

	/* if current zone and previous zone are same and if they are
	 *  the top and bottom zones then report OVERHEAT
	 */
	if ((prev_zone == cur_zone) && ((prev_zone == BCOVE_BZONE_LOW) ||
					(prev_zone == BCOVE_BZONE_HIGH)))
		charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_CHARGER,
						POWER_SUPPLY_HEALTH_OVERHEAT,
						0);
	prev_zone = cur_zone;
}

static void bcove_extchrgr_read_complete(bool stat)
{

	atomic_set(&chc.i2c_rw, I2C_RD);
	wake_up(&chc.i2c_wait);

}

static void bcove_extchrgr_write_complete(bool stat)
{

	atomic_set(&chc.i2c_rw, I2C_WR);
	wake_up(&chc.i2c_wait);

}


static void bcove_battery_overheat_handler(bool stat)
{
	if (stat)
		charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_BATTERY,
						POWER_SUPPLY_HEALTH_OVERHEAT,
						false);
	else
		charger_helper_report_exception(chc.ch_handle,
						CH_SOURCE_TYPE_BATTERY,
						POWER_SUPPLY_HEALTH_OVERHEAT,
						true);
}

static void bcove_handle_ext_chrgr_irq(bool stat)
{
	int ret;
	dev_dbg(chc.dev, "%s:%d stat=%d\n", __func__, __LINE__, stat);
	if (stat) {
		ret = chc.ext_chrgr->handle_irq(chc.ext_chrgr_addr);
		if (ret)
			dev_err(chc.dev,
				"Error in handling external charger irq\n");
	}
}



/*charger framework callback functions */
static int disable_charger(enum charger_type chrg_type)
{
	dev_dbg(chc.dev, "%s\n", __func__);
	return intel_scu_ipc_update_register(CHGRCTRL0_ADDR, EXTCHRDIS_ENABLE,
				      CHGRCTRL0_EXTCHRDIS_MASK);
}

static int enable_charging(enum charger_type chrg_type)
{
	dev_dbg(chc.dev, "%s\n", __func__);

	intel_scu_ipc_update_register(CHGRCTRL0_ADDR, EXTCHRDIS_DISABLE,
				      CHGRCTRL0_EXTCHRDIS_MASK);
	return chc.ext_chrgr->enable_charging(chc.ext_chrgr_addr);

}

static int disable_charging(enum charger_type chrg_type)
{
	dev_dbg(chc.dev, "%s\n", __func__);
	return chc.ext_chrgr->disable_charging(chc.ext_chrgr_addr);
}

static int bc_set_ilimmA(int ilim_mA)
{
	u8 mask;

	lookup_regval(bcove_inlmt, ARRAY_SIZE(bcove_inlmt),
			ilim_mA, &mask);

	return intel_scu_ipc_update_register(CHGRCTRL1_ADDR, 0xFF, mask);
}

static void charger_callback(int charger_type)
{
	chc.psy.type = charger_type;
	power_supply_changed(&chc.psy);
}

static bool is_battery_charging(void)
{
	return chc.ext_chrgr->is_batt_charging(chc.ext_chrgr_addr);
}

/**
 * bcove_read_adc_val - read ADC value of specified sensors
 * @channel: channel of the sensor to be sampled
 * @sensor_val: pointer to the charger property to hold sampled value
 * @chc :  battery info pointer
 *
 * Returns 0 if success
 */

static int bcove_read_adc_val(int channel, int *sensor_val,
			      struct bc_chrgr_drv_context *chc)
{

	int ret, adc_val;
	struct gpadc_result *adc_res;
	adc_res = kzalloc(sizeof(struct gpadc_result), GFP_KERNEL);
	if (!adc_res)
		return -ENOMEM;
	ret = intel_basincove_gpadc_sample(channel, adc_res);
	if (ret) {
		dev_err(chc->dev, "gpadc_sample failed:%d\n", ret);
		goto exit;
	}

	adc_val = GPADC_RSL(channel, adc_res);
	switch (channel) {
	case GPADC_BATTEMP0:
		ret = CONVERT_ADC_TO_TEMP(adc_val, sensor_val);
		break;
	default:
		dev_err(chc->dev, "invalid sensor%d", channel);
		ret = -EINVAL;
	}
exit:
	kfree(adc_res);
	return ret;
}

/**
 * bc_chrgr_get_property - charger power supply get property
 * @psy: charger power supply context
 * @psp: charger property
 * @val: charger property value
 * Context: can sleep
 *
 * Basin Cove power supply property needs to be provided to power_supply
 * subsystem for it to provide the information to users.
 */
static int bc_chrgr_ps_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct bc_chrgr_drv_context *chc =
	    container_of(psy, struct bc_chrgr_drv_context, psy);

	if (!chc->ch_handle)
		return -EINVAL;

	return charger_helper_get_property(chc->ch_handle,
					   CH_SOURCE_TYPE_CHARGER, psp, val);
}

/* Exported Functions to use with Fuel Gauge driver */

int bc_check_battery_health(void)
{
	union power_supply_propval val;
	int retval;
	/* check if basincove  charger is ready */
	if (!chc.ch_handle) {
		dev_err(chc.dev, "Invalid charger helper handle\n");
		return -EINVAL;
	}

	retval =
	    charger_helper_get_property(chc.ch_handle,
					CH_SOURCE_TYPE_BATTERY,
					POWER_SUPPLY_PROP_HEALTH, &val);
	if (retval)
		return retval;

	return val.intval;
}
EXPORT_SYMBOL(bc_check_battery_health);

int bc_check_battery_status(void)
{
	union power_supply_propval val;
	int retval;

	if (!chc.ch_handle)
		return -ENODEV;

	retval = charger_helper_get_property(chc.ch_handle,
					     CH_SOURCE_TYPE_BATTERY,
					     POWER_SUPPLY_PROP_STATUS, &val);
	if (retval)
		return retval;

	return val.intval;
}
EXPORT_SYMBOL(bc_check_battery_status);

int bc_get_battery_pack_temp(int *temp)
{

	/* check if basincove  charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	if (chc.invalid_batt)
		return -ENODEV;
	return bcove_read_adc_val(GPADC_BATTEMP0, temp, &chc);
}
EXPORT_SYMBOL(bc_get_battery_pack_temp);

void handle_interrupt(u8 int_reg, u8 stat_reg, struct interrupt_info int_info[],
		      int int_info_size)
{
	int i;
	bool int_stat;
	char *log_msg;


	for (i = 0; i < int_info_size; ++i) {

		/*continue if interrupt register bit is not set */
		if (!(int_reg & int_info[i].int_reg_mask))
			continue;

		/*log message if interrupt bit is set */
		if (int_info[i].log_msg_int_reg_true)
			dev_err(chc.dev, "%s",
				int_info[i].log_msg_int_reg_true);

		/* interrupt bit is set.call int handler. */
		if (int_info[i].int_handle)
			int_info[i].int_handle();

		/* continue if stat_reg_mask is zero which
		 *  means ignore status register
		 */
		if (!(int_info[i].stat_reg_mask))
			continue;

		dev_dbg(chc.dev,
			"stat_reg=%X int_info[i].stat_reg_mask=%X",
			stat_reg, int_info[i].stat_reg_mask);

		/* check if the interrupt status is true */
		int_stat = (stat_reg & int_info[i].stat_reg_mask);

		/* log message */
		log_msg = int_stat ? int_info[i].log_msg_stat_true :
		    int_info[i].log_msg_stat_false;

		if (log_msg)
			dev_err(chc.dev, "%s", log_msg);

		/*call status handler function */
		if (int_info[i].stat_handle)
			int_info[i].stat_handle(int_stat);

	}
}

static irqreturn_t bc_thread_handler(int id, void *data)
{
	struct bc_chrgr_drv_context *chc = data;
	u8 chgrirq0_stat, chgrirq0_int;

	if (intel_scu_ipc_ioread8(SCHGRIRQ0_ADDR, &chgrirq0_stat)) {
		dev_err(chc->dev,
			"%s(): Error in reading SCHGRIRQ0_ADDR\n", __func__);
		goto end;
	}

	/* read interrupt register */
	chgrirq0_int = ioread8(chc->pmic_intr_iomap);
	dev_dbg(chc->dev, "SCHGRIQ0=%X chgrirq0%X\n",
		chgrirq0_stat, chgrirq0_int);

	handle_interrupt(chgrirq0_int, chgrirq0_stat, chgrirq0_info,
			 ARRAY_SIZE(chgrirq0_info));
end:
	/*clear first level IRQ */
	intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
			IRQLVL1_CHRGR_MASK);
	return IRQ_HANDLED;

}

static int bcove_init(struct batt_charging_profile bcprof)
{
	int ret, i, temp_mon_ranges;
	u8 addr_tzone, addr_cv, addr_cc, reg_val;
	u16 adc_val;

	ret = intel_scu_ipc_update_register(CHGRCTRL0_ADDR, SWCONTROL_ENABLE,
					    CHGRCTRL0_SWCONTROL_MASK);
	if (ret)
		return ret;

	/*Configure Temp Zone, CC and CV */
	addr_tzone = THRMZN4H_ADDR;

	/*Ignore Emegency Charging Zones */
	addr_cc = TT_CHRCCHOTVAL_ADDR;
	addr_cv = TT_CHRCVHOTVAL_ADDR;

	temp_mon_ranges = min_t(u16, bcprof.temp_mon_ranges,
				BATT_PROF_MAX_TEMP_NR_RNG);

	for (i = 0; i < temp_mon_ranges; ++i) {

		ret =
		    CONVERT_TEMP_TO_ADC(bcprof.temp_mon_range[i].temp_up_lim,
					(int *)&adc_val);
		if (unlikely(ret))
			return ret;
		ret = bcove_update_tt(addr_tzone, 0x03, adc_val >> 8);
		if (unlikely(ret))
			return ret;

		ret = bcove_write_tt((addr_tzone + 1),
					       (adc_val & 0xFF));
		if (unlikely(ret))
			return ret;

		addr_tzone -= 2;

		ret = chc.ext_chrgr->cc_to_reg(bcprof.temp_mon_range[i].
					     full_chrg_cur, &reg_val);
		if (ret)
			return ret;
		ret = bcove_write_tt(addr_cc, reg_val);
		if (unlikely(ret))
			return ret;
		addr_cc--;

		ret = chc.ext_chrgr->cv_to_reg(bcprof.temp_mon_range[i].
					     full_chrg_vol, &reg_val);
		if (ret)
			return ret;

		ret = bcove_write_tt(addr_cv, reg_val);
		if (unlikely(ret))
			return ret;
		addr_cv--;

		/* Write lowest temp limit */
		if (i == (bcprof.temp_mon_ranges - 1)) {
			ret =
			    CONVERT_TEMP_TO_ADC(bcprof.temp_mon_range[i].
						temp_up_lim, (int *)&adc_val);
			if (unlikely(ret))
				return ret;

			ret = intel_scu_ipc_update_register(addr_tzone,
					(adc_val >> 8 & 0xFF), 0x03);
			if (unlikely(ret))
				return ret;

			ret = intel_scu_ipc_iowrite8(addr_tzone+1,
							(adc_val & 0xFF));
		}

	}
	return ret;

}

static int otg_handle_notification(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	charger_helper_charger_port_changed(chc.ch_handle,
					    (struct power_supply_charger_cap *)
					    data);
	return NOTIFY_OK;

}

static struct bc_chrgr_drv_context chc = {

	.psy = {
		.name = CHARGER_PS_NAME,
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = bc_chrgr_ps_props,
		.num_properties = ARRAY_SIZE(bc_chrgr_ps_props),
		.get_property = bc_chrgr_ps_get_property,
		.supplied_to = bcove_charger_power_supplied_to,
		.num_supplicants = ARRAY_SIZE(bcove_charger_power_supplied_to),
		},

	.ch_charger = {
		       .flags = USB_CHRGR_SUPPORTED | USB_VSYS_SUPPORTED,
		       .enable_charging = enable_charging,
		       .disable_charging = disable_charging,
		       .disable_charger  = disable_charger,
		       .set_ilimmA = bc_set_ilimmA,
		       .is_battery_charging = is_battery_charging,
		       .charger_callback = charger_callback,
		       },
	.otg_nb = {
		   .notifier_call = otg_handle_notification,
		   }
};

static inline void print_battery_profile(struct batt_charging_profile bprof)
{
	int i, temp_mon_ranges;

	dev_info(chc.dev, "ChrgProf: batt_id:%s\n", bprof.batt_id);
	dev_info(chc.dev, "ChrgProf: battery_type:%u\n", bprof.battery_type);
	dev_info(chc.dev, "ChrgProf: capacity:%u\n", bprof.capacity);
	dev_info(chc.dev, "ChrgProf: voltage_max:%u\n", bprof.voltage_max);
	dev_info(chc.dev, "ChrgProf: chrg_term_mA:%u\n", bprof.chrg_term_mA);
	dev_info(chc.dev, "ChrgProf: low_batt_mV:%u\n", bprof.low_batt_mV);
	dev_info(chc.dev, "ChrgProf: disch_tmp_ul:%u\n", bprof.disch_tmp_ul);
	dev_info(chc.dev, "ChrgProf: disch_tmp_ll:%u\n", bprof.disch_tmp_ll);
	dev_info(chc.dev, "ChrgProf: temp_mon_ranges:%u\n",
			bprof.temp_mon_ranges);
	temp_mon_ranges = min_t(u16, bprof.temp_mon_ranges,
			BATT_PROF_MAX_TEMP_NR_RNG);

	for (i = 0; i < temp_mon_ranges; ++i) {
		dev_info(chc.dev, "ChrgProf: temp_up_lim[%d]:%d\n",
				i, bprof.temp_mon_range[i].temp_up_lim);
		dev_info(chc.dev, "ChrgProf: full_chrg_vol[%d]:%d\n",
				i, bprof.temp_mon_range[i].full_chrg_vol);
		dev_info(chc.dev, "ChrgProf: full_chrg_cur[%d]:%d\n",
				i, bprof.temp_mon_range[i].full_chrg_cur);
		dev_info(chc.dev, "ChrgProf: maint_chrgr_vol_ll[%d]:%d\n",
				i, bprof.temp_mon_range[i].maint_chrg_vol_ll);
		dev_info(chc.dev, "ChrgProf: maint_chrgr_vol_ul[%d]:%d\n",
				i, bprof.temp_mon_range[i].maint_chrg_vol_ul);
		dev_info(chc.dev, "ChrgProf: maint_chrg_cur[%d]:%d\n",
				i, bprof.temp_mon_range[i].maint_chrg_cur);
	}
	dev_info(chc.dev, "ChrgProf: temp_low_lim:%d\n", bprof.temp_low_lim);
}
/**
 * bc_charger_probe - basin cove charger probe function
 * @ipcdev: basin cove ipc device structure
 * Context: can sleep
 *
 * basin cove charger driver initializes its internal data
 * structure and other  infrastructure components for it
 * to work as expected.
 */
static int bc_chrgr_probe(struct ipc_device *ipcdev)
{
	int retval = 0;
	struct batt_charging_profile bcprof;

	if (!ipcdev)
		return -ENOMEM;
	chc.dev = &ipcdev->dev;
	chc.irq = ipc_get_irq(ipcdev, 0);
	ipc_set_drvdata(ipcdev, &chc);

	/*FIXME: Make charger selection dynamic */
	chc.ext_chrgr = &bq24260_chrgr;

	/* Read Charger I2C slave Address */
	retval = bcove_read_tt(TT_I2CDADDR_ADDR, &chc.ext_chrgr_addr);
	if (unlikely(retval))
		return retval;

	if (get_batt_charging_profile(&bcprof) < 0)
		chc.invalid_batt = true;
	else {
		print_battery_profile(bcprof);
		retval = bcove_init(bcprof);
		if (retval) {
			dev_err(chc.dev, "Error in Initializing PMIC\n");
			return retval;
		}
	}

	/*register with charger helper */
	chc.ch_charger.dev = &ipcdev->dev;
	chc.ch_charger.invalid_battery = chc.invalid_batt;
	chc.ch_handle = charger_helper_register_charger(&chc.ch_charger);

	if (!chc.ch_handle) {
		dev_err(&ipcdev->dev, "Error in charger_helper_register\n");
		return -EIO;
	}
	chc.pmic_intr_iomap = ioremap_nocache(PMIC_SRAM_INTR_ADDR, 8);
	if (!chc.pmic_intr_iomap) {
		dev_err(&ipcdev->dev, "ioremap Failed\n");
		retval = -ENOMEM;
		goto ioremap_failed;
	}
	/* unmask charger interrupts in second level IRQ register*/
	retval = intel_scu_ipc_iowrite8(MCHGRIRQ0_ADDR, 0x00);
	if (unlikely(retval))
		goto unmask_irq_failed;

	/* unmask IRQLVL1 register */
	retval = intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
			IRQLVL1_CHRGR_MASK);
	if (unlikely(retval))
		goto unmask_irq_failed;


	/* register interrupt */
	retval = request_threaded_irq(chc.irq, NULL,
				      bc_thread_handler, 0, DRIVER_NAME, &chc);
	if (retval) {
		dev_err(&ipcdev->dev, "Error in request_threaded_irq(irq(%d)\n",
			chc.irq);
		goto req_irq_failed;
	}

	init_waitqueue_head(&chc.i2c_wait);

	retval = power_supply_register(&ipcdev->dev, &chc.psy);
	if (retval)
		goto psy_reg_failed;

	chc.transceiver = otg_get_transceiver();
	if (!chc.transceiver) {
		dev_err(chc.dev, "failure to get otg transceiver\n");
		goto otg_reg_failed;
	}
	retval = otg_register_notifier(chc.transceiver, &chc.otg_nb);
	if (retval) {
		dev_err(chc.dev, "failure to register otg notifier\n");
		goto otg_reg_failed;
	}

	bcove_debugfs_init();

	return 0;

otg_reg_failed:
	power_supply_unregister(&chc.psy);
psy_reg_failed:
	free_irq(chc.irq, &chc);
unmask_irq_failed:
req_irq_failed:
	iounmap(chc.pmic_intr_iomap);
ioremap_failed:
	charger_helper_unregister_charger(chc.ch_handle);
	return retval;
}

static void bc_chrgr_do_exit_ops(struct bc_chrgr_drv_context *chc)
{
	/*TODO:
	 * If charger is connected send IPC message to SCU to continue charging
	 */

}

/**
 * bc_charger_remove - basin cove charger finalize
 * @ipcdev: basin cove charger ipc device structure
 * Context: can sleep
 *
 * Basin cove charger finalizes its internal data structure and other
 * infrastructure components that it initialized in
 * bc_chrgr_probe.
 */
static int bc_chrgr_remove(struct ipc_device *ipcdev)
{
	struct bc_chrgr_drv_context *chc = ipc_get_drvdata(ipcdev);

	if (chc) {
		bc_chrgr_do_exit_ops(chc);
		if (chc->transceiver) {
			otg_unregister_notifier(chc->transceiver, &chc->otg_nb);
			otg_put_transceiver(chc->transceiver);
		}

		free_irq(chc->irq, chc);
		charger_helper_unregister_charger(chc->ch_handle);
		power_supply_unregister(&chc->psy);
		iounmap(chc->pmic_intr_iomap);

	}

	bcove_debugfs_exit();

	return 0;
}

#ifdef CONFIG_PM
static int bc_chrgr_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bc_chrgr_suspend    NULL
#define bc_chgr_resume     NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bc_chrgr_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bc_chrgr_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bc_chrgr_runtime_suspend	NULL
#define bc_chrgr_runtime_resume		NULL
#define bc_chrgr_runtime_idle		NULL
#endif
/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct dev_pm_ops bc_chrgr_pm_ops = {
	.suspend = bc_chrgr_suspend,
	.resume = bc_chrgr_resume,
	.runtime_suspend = bc_chrgr_runtime_suspend,
	.runtime_resume = bc_chrgr_runtime_resume,
	.runtime_idle = bc_chrgr_runtime_idle,
};

static struct ipc_driver bc_chrgr_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &bc_chrgr_pm_ops,
		   },
	.probe = bc_chrgr_probe,
	.remove = __devexit_p(bc_chrgr_remove),
};

static int __init bc_chrgr_init(void)
{
	int ret;
	ret = ipc_driver_register(&bc_chrgr_driver);

	return ret;
}

static void __exit bc_chrgr_exit(void)
{

	ipc_driver_unregister(&bc_chrgr_driver);
}

/* Defer init call so that dependant drivers will be loaded. Using  async
 * for parallel driver initialization */
late_initcall_async(bc_chrgr_init);
module_exit(bc_chrgr_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("Basin Cove Charger  Driver");
MODULE_LICENSE("GPL");
