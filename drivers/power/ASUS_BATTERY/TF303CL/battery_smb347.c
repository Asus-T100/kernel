/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen tom_sheng@asus.com
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/power/smb347-asus-charger.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>	/* Needed for KERN_DEBUG */
#include <linux/wakelock.h>
#include <linux/extcon/extcon-fsa9285.h>
#include <asm/intel-mid.h>
#include <linux/acpi.h>


#include "smb347_external_include.h"
#include "asus_battery.h"

/*
 * usb notify callback
 */
#define USB_NOTIFY_CALLBACK

/*
 *  config max current
 */
int max_current_set = 0;

int inok_gpio = 207;
EXPORT_SYMBOL(max_current_set);

/*
 *  config function
 */
#define CANCEL_SOFT_HOT_TEMP_LIMIT   0
#define SMB347_INTERRUPT             0
#define SMB347_IRQ_INIT              0
/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS_FUNCS			0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB		BIT(2)
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN		0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C		0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW		BIT(5)
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK	0x03
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_FAULT_IRQ_OTG_UV			BIT(5)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT		BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_OTG_ENABLED			BIT(4)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_D				0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT		BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ		BIT(3)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
#define IRQSTAT_F				0x3a
#define IRQSTAT_F_OTG_UV_IRQ			BIT(5)
#define IRQSTAT_F_OTG_UV_STAT			BIT(4)

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_HOLDOFF_STAT			BIT(3)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHG_TERM				BIT(5)
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_E					0x3f

#define STATUS_UPDATE_INTERVAL			(HZ * 60)	/*60 sec */

#define TF303CL_CHARGER_ACPI                    1

struct smb347_otg_event {
	struct list_head	node;
	bool			param;
};

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @running: the driver is up and running
 * @dentry: for debugfs
 * @otg: pointer to OTG transceiver if any
 * @otg_nb: notifier for OTG notifications
 * @otg_work: work struct for OTG notifications
 * @otg_queue: queue holding received OTG notifications
 * @otg_queue_lock: protects concurrent access to @otg_queue
 * @otg_last_param: last parameter value from OTG notification
 * @otg_enabled: OTG VBUS is enabled
 * @otg_battery_uv: OTG battery undervoltage condition is on
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	bool			mains_online;
	bool			usb_online;
	bool			charging_enabled;
	bool			running;
	struct dentry		*dentry;
	struct otg_transceiver	*otg;
	struct notifier_block	otg_nb;
	struct work_struct	otg_work;
	struct list_head	otg_queue;
	spinlock_t		otg_queue_lock;
	bool			otg_enabled;
	bool			otg_battery_uv;
	const struct smb347_charger_platform_data	*pdata;
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock_smb347;
};

static struct smb347_charger *smb347_dev;
static bool ischargerSuspend = false;
static bool isUSBSuspendNotify = false;

#define DEBUG 1
#define DRIVER_VERSION			"1.1.0"

#define SMB347_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG			0x00
#define INPUT_CURRENT_LIMIT_REG	0x01
#define VAR_FUNC_REG			0x02
#define FLOAT_VOLTAGE_REG		0x03
#define CHG_CTRL_REG			0x04
#define STAT_TIMER_REG			0x05
#define PIN_ENABLE_CTRL_REG		0x06
#define THERM_CTRL_A_REG		0x07
#define SYSOK_USB3_SELECT_REG	0x08
#define OTHER_CTRL_A_REG		0x09
#define OTG_TLIM_THERM_CNTRL_REG				0x0A
#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG	0x0B
#define FAULT_INTERRUPT_REG		0x0C
#define STATUS_INTERRUPT_REG	0x0D
//#define SYSOK_REG		0x0E chris: only smb349 contain this register
#define I2C_BUS_SLAVE_REG		0x0E //chris: add
#define CMD_A_REG		0x30
#define CMD_B_REG		0x31
#define CMD_C_REG		0x33
#define INTERRUPT_A_REG		0x35
#define INTERRUPT_B_REG		0x36
#define INTERRUPT_C_REG		0x37
#define INTERRUPT_D_REG		0x38
#define INTERRUPT_E_REG		0x39
#define INTERRUPT_F_REG		0x3A
#define STATUS_A_REG	0x3B
#define STATUS_B_REG	0x3C
#define STATUS_C_REG	0x3D
#define STATUS_D_REG	0x3E
#define STATUS_E_REG	0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK		SMB347_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT		BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK			SMB347_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK		SMB347_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK			SMB347_MASK(3, 5)
#define TERMINATION_CURRENT_MASK		SMB347_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK	SMB347_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK				SMB347_MASK(6, 0)
#define CHG_ENABLE_BIT			BIT(1)
#define VOLATILE_W_PERM_BIT		BIT(7)
#define USB_SELECTION_BIT		BIT(1)
#define SYSTEM_FET_ENABLE_BIT	BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT			BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT	BIT(2)
#define BATT_OV_END_CHG_BIT		BIT(1)
#define VCHG_FUNCTION			BIT(0)
#define CURR_TERM_END_CHG_BIT	BIT(6)

#define OTGID_PIN_CONTROL_MASK	SMB347_MASK(2, 6)			// OTHER_CTRL_A_REG
#define OTGID_PIN_CONTROL_BITS	BIT(6)			// OTHER_CTRL_A_REG (RID Disable, OTG Pin Control)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK	SMB347_MASK(2, 2)	// OTG_TLIM_THERM_CNTRL_REG
#define OTG_CURRENT_LIMIT_750mA	(BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_250mA	BIT(2)

#define CFG_40V                                 BIT(0)|BIT(3)|BIT(4)
#define CFG_411V				BIT(1)|BIT(2)|BIT(3)|BIT(4)
#define CFG_420V				BIT(0)|BIT(1)|BIT(5)
#define CFG_435V				BIT(1)|BIT(3)|BIT(5)
#define CFG_432V				BIT(0)|BIT(3)|BIT(5)
#define CFG_FAST_CHARGE            0x69
#define CFG_SOFT_LIMIT   	   0x0a
#define CFG_SOFT_700mA   	   BIT(6)
#define CFG_AICL        	   BIT(4)
#define CFG_500mA        	   BIT(0)
#define CFG_1200mA        	   BIT(2)
#define CFG_1500mA        	   BIT(2) | BIT(0)
#define CFG_1800mA        	   BIT(2) | BIT(1)


extern int entry_mode;
#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS

struct battery_info_reply batt_info;
struct wake_lock wakelock_smb347;
struct wake_lock wakelock_smb347_t;    // for wake_lokc_timout() useage

struct workqueue_struct *charger_work_queue = NULL;
struct delayed_work charger_work;

/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
	300,
	500,
	700,
	900,
	1200,
	1500,
	1800,
	2000,
	2200,
	2500,
};

/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val) {
	return (val >= size) ? tbl[size-1] : tbl[val];
}

static int smb347_read_reg(struct i2c_client *client, int reg,
				u8 *val, int ifDebug) {
	s32 ret;
	struct smb347_charger *smb347_chg;

	smb347_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb347_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb347_chg->client->dev,
			"i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	if (ifDebug) pr_info("Reg%02Xh = " BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(*val));

	return 0;
}

static int smb347_write_reg(struct i2c_client *client, int reg,
						u8 val) {
	s32 ret;
	struct smb347_charger *smb347_chg;

	smb347_chg = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(smb347_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb347_chg->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb347_update_write(struct smb347_charger *smb, int reg, u8 aV, u8 oV) {
	s32 rc;
	u8 temp;

	rc = smb347_read_reg(smb->client, reg, &temp, 0);
	if (rc) {
		pr_err("smb347_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}

        temp &= (~aV);
        temp |= oV;

	rc = smb347_write_reg(smb->client, reg, temp);
	if (rc) {
		pr_err("smb347_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb347_read(struct smb347_charger *smb, u8 reg) {
	int ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to read reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val) {
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to write reg 0x%x: %d\n",
			 reg, ret);

	return ret;
}

/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable) {
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

static int otg(int toggle) {
	int ret;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		return ret;

	if (toggle) {
                /* 1.Disable OTG */
                ret = smb347_update_write( smb347_dev,
                                           CMD_A_REG,
                                           BIT(4),
                                           0);
		if (ret) {
			pr_err("Failed to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}

                /* 2.Set OTG current limit to 250mA & UVLO 2.7V*/
		// Refer to SMB347 Application Note 72 to solve serious problems
                ret = smb347_update_write( smb347_dev,
                                           OTG_TLIM_THERM_CNTRL_REG,
                                           BIT(0)|BIT(1)|BIT(2)|BIT(3),
                                           BIT(2));
		if (ret) {
			pr_err("Failed to set OTG current limit 250mA. ret=%d\n", ret);
			return ret;
		}

                /* 3.Enable OTG Function*/
                ret = smb347_update_write( smb347_dev,
                                           CMD_A_REG,
                                           0,
                                           BIT(4));
		if (ret) {
			pr_err("Failed to set OTG Enable bit ret=%d\n", ret);
			return ret;
		}

                /* 4.Set OTG current limit to 750mA & UVLO 2.7V*/
		// Refer to SMB347 Application Note 72 to solve serious problems
                ret = smb347_update_write( smb347_dev,
                                           OTG_TLIM_THERM_CNTRL_REG,
                                           BIT(0)|BIT(1)|BIT(2)|BIT(3),
                                           BIT(2)|BIT(3));
		if (ret) {
			pr_err("Failed to set OTG current limit 750mA. ret=%d\n", ret);
			return ret;
		}

                /* 5.OTG function change to PIN control */
                ret = smb347_update_write( smb347_dev,
                                           OTHER_CTRL_A_REG,
                                           BIT(6)|BIT(7),
                                           BIT(6));
		if (ret) {
			pr_err("Failed to set OTGID_PIN_CONTROL_BITS rc=%d\n", ret);
			return ret;
		}
	} else {
           // Hardware will do 1.Disable OTG function 2. Reset Charger IC & Load Defauft setting
	}

	smb347_dev->otg_enabled = (toggle > 0 ? true : false);
	return 0;
}

extern void bq27541_battery_callback(unsigned usb_cable_state);

static int g_usb_state;
int setSMB347Charger(int usb_state) {
        g_usb_state = usb_state;
        queue_delayed_work(charger_work_queue, &charger_work, 0);

	return 0;
}
EXPORT_SYMBOL(setSMB347Charger);

static void do_charger(struct work_struct *work) {
        int ret = 0;

        mutex_lock(&smb347_dev->lock);
        batt_info.cable_status = NO_CABLE;
        mutex_unlock(&smb347_dev->lock);

	switch (g_usb_state) {
	case USB_IN:
		pr_info("usb_state: USB_IN\n");
                bq27541_battery_callback(USB_PC);
                batt_info.cable_status = USB_PC;
		//usb_to_battery_callback(USB_PC);
		break;
	case AC_IN:
		pr_info("usb_state: AC_IN\n");
                bq27541_battery_callback(USB_ADAPTER);
                batt_info.cable_status = USB_ADAPTER;
		//usb_to_battery_callback(USB_ADAPTER);
		break;
	case CABLE_OUT:
		pr_info("usb_state: CABLE_OUT\n");
                bq27541_battery_callback(NO_CABLE);
                batt_info.cable_status = NO_CABLE;
		//usb_to_battery_callback(NO_CABLE);
                //max_current_set = 0;
		break;
	case ENABLE_5V:
		pr_info("usb_state: ENABLE_5V\n");
		ret = otg(1);
		break;
	case DISABLE_5V:
		pr_info("usb_state: DISABLE_5V\n");
		ret = otg(0);
                //max_current_set = 0;
		break;
	default:
		pr_info("ERROR: wrong usb state value = %d\n", g_usb_state);
		ret = 1;
                max_current_set = 0;
	}

        msleep(10);
        /* TF303CL 
           1.set fast charge current =2.5A set pre-charge current =200mA
           2.set cold soft limit current = 1200mA
           3.Set Battery OV does not end charging cycle.
           4.Set Float Voltage = 4.2V
        */
        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
            smb347_set_fast_charge();
            smb347_set_battery_OV();
            smb347_set_voltage(4200);
            /*SOC control JEITA*/
            smb347_control_JEITA(true);
            if (batt_info.cable_status == USB_ADAPTER) {
                /* TF303CL set I_USB_IN=1800mA */
                smb347_AC_in_current(1800, USB5_1_HC_MAX);
            }
        }

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER) {
                if (!wake_lock_active(&wakelock_smb347)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock_smb347 -> wake lock\n", __func__);
                    wake_lock(&wakelock_smb347);
                }
            } else if (batt_info.cable_status == NO_CABLE) {
                if (wake_lock_active(&wakelock_smb347)) {
                   BAT_DBG(" %s: asus_battery_power_wakelock_smb347 -> wake unlock\n", __func__);
                   wake_lock_timeout(&wakelock_smb347_t, 3*HZ);  // timeout value as same as the <charger.exe>\asus_global.h #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                   wake_unlock(&wakelock_smb347);
                } else { // for PC case
                   wake_lock_timeout(&wakelock_smb347_t, 3*HZ);
                }
            }
        }
}

int smb347_control_JEITA(bool on) {

        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_warn("%s: *** control_JEITA: %s ***\n", __func__, on ? "on" : "off");
	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_TEMP_LIMIT register */
        if (on) {
            // 1. set Hard Hot Limit = 72 Deg.C  & Soft Hot Limit = 59 Deg.C
            ret = smb347_update_write(smb347_dev,
                                      CFG_TEMP_LIMIT,
                                      0,
                                      BIT(5)|BIT(4)|BIT(1)|BIT(0));
            if (ret < 0)
		goto out;
            // 2. set Soft Hot Limit Behavior = No Response
            ret = smb347_update_write(smb347_dev,
                                      CFG_THERM,
                                      BIT(1)|BIT(0),
                                      0);
            if (ret < 0)
		goto out;
        } else {
            // 1. set Hard Hot Limit = 53 Deg.C  & Soft Hot Limit = 47 Deg.C
            ret = smb347_update_write(smb347_dev,
                                      CFG_TEMP_LIMIT,
                                      BIT(5)|BIT(4)|BIT(1),
                                      BIT(0));
            if (ret < 0)
		goto out;
            // 2. set Soft Hot Limit Behavior = Float Voltage Compensation
            ret = smb347_update_write(smb347_dev,
                                      CFG_THERM,
                                      BIT(0),
                                      BIT(1));
            if (ret < 0)
		goto out;

        }

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

int smb347_get_aicl_result() {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info(" ***  %s ***\n", __func__);
	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config STAT_E register */
	ret = smb347_read(smb347_dev, STAT_E);
	if (ret < 0)
		goto out;

        ret &= 0x0f;
        pr_info("smb347 get aicl result = 0x%02x\n", ret);
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

int smb347_set_battery_OV() {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info(" ***  %s ***\n", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config VAR_FUNC_REG register */
	ret = smb347_read(smb347_dev, VAR_FUNC_REG);
	if (ret < 0)
		goto out;

	ret &= ~BIT(1);
        pr_info("write  battery 0V does not charger cycle = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, VAR_FUNC_REG, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

int smb347_set_voltage(int v) {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_warn("%s: *** charging voltage: %d mV ***\n", __func__, v);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_FLOAT_VOLTAGE register */
	ret = smb347_read(smb347_dev, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		goto out;

	/*
	 * Make the voltage functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= (BIT(6)|BIT(7));
        switch (v) {
            case 4000:
               /* voltage = 4.0v */
	       ret |= CFG_40V;
               break;
            case 4110:
               /* voltage = 4.11v */
	       ret |= CFG_411V;
               break;
            case 4200:
               /* voltage = 4.20v */
	       ret |= CFG_420V;
               break;
            case 4350:
               /* voltage = 4.35v */
	       ret |= CFG_435V;
               break;
            case 4320:
            default:
               /* voltage = 4.32v */
               ret |= CFG_432V;
               break;
        }

        pr_info("write voltage = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, CFG_FLOAT_VOLTAGE, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

int smb347_set_fast_charge(void) {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("== %s ==", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Set Fast Charge current = 2500mA (00h[7:5]="111"), Config CFG_CHARGE_CURRENT register */
        ret = smb347_update_write( smb347_dev, CFG_CHARGE_CURRENT, 0, BIT(5)|BIT(6)|BIT(7));
	if (ret < 0)
		goto out;

        /* Set cold soft limit current = 900mA (0Ah[7:6]="11"),Config CFG_SOFT_LIMIT register */
        ret = smb347_update_write( smb347_dev, CFG_SOFT_LIMIT, 0, BIT(6)|BIT(7));
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb347_dev->lock);
	return ret;

}

static int smb347_set_max_current(int c) {
       int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("== %s ==", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/*1.Disable AICL Config VAR_FUNC_REG register */
        ret = smb347_update_write( smb347_dev, VAR_FUNC_REG, BIT(4), 0);
	if (ret < 0)
		goto out;

        /* 2. Config INPUT_CURRENT_LIMIT_REG register */
	ret = smb347_read(smb347_dev, INPUT_CURRENT_LIMIT_REG);
	if (ret < 0)
		goto out;

	ret &= 0xf0;
        switch (c) {
           case 500:
              ret |= CFG_500mA;
              break;
           case 1500:
              ret |= CFG_1500mA;
              break;
           case 1800:
              ret |= CFG_1800mA;
              break;
           case 1200:
           default:
	      ret |= CFG_1200mA;
              break;
        }
        pr_info("set AC IN = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, INPUT_CURRENT_LIMIT_REG, ret);
	if (ret < 0)
		goto out;

	/*3.enable AICL Config VAR_FUNC_REG register */
        ret = smb347_update_write( smb347_dev, VAR_FUNC_REG, 0, BIT(4));
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

static int smb347_set_USB5_1_HC_max_current(int c) {
        int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("== %s ==", __func__);

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

        /* 1. Set I_USB_IN =1.2A (01h[3:0]="0100") */
	ret = smb347_read(smb347_dev, INPUT_CURRENT_LIMIT_REG);
	if (ret < 0)
		goto out;

	ret &= 0xf0;
        switch (c) {
           case 1500:
              ret |= CFG_1500mA;
              break;
           case 1800:
              ret |= CFG_1800mA;
              break;
           case 1200:
           default:
	      ret |= CFG_1200mA;
              break;
        }
        pr_info("set AC IN = 0x%02x\n", ret);
	ret = smb347_write(smb347_dev, INPUT_CURRENT_LIMIT_REG, ret);
	if (ret < 0)
		goto out;

	/*2. set USB to HC mode (31h[1:0]="11") */
        ret = smb347_update_write( smb347_dev, 0x31, 0, BIT(0)|BIT(1));
	if (ret < 0)
		goto out;

        /*3. set USB5/1/HC to register control (06h[4]="0") */
        ret = smb347_update_write( smb347_dev, 0x06, BIT(4), 0);
	if (ret < 0)
		goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

int smb347_AC_in_current(int c, int type) {
    if (type == USB5_1_HC_MAX) {
        return smb347_set_USB5_1_HC_max_current(c);
    } else {
        return smb347_set_max_current(c);
    }
}

int smb347_charging_toggle(bool on) {
	int ret = 0;

	if (!smb347_dev) {
		pr_info("Warning: smb347_dev is null due to probe function has error\n");
		return 1;
	}

	if (!on)
	pr_warn("%s: *** charging toggle: %s ***\n", __func__, on ? "ON" : "OFF");

	mutex_lock(&smb347_dev->lock);

	ret = smb347_set_writable(smb347_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_PIN register */
        if (on) {
            ret = smb347_update_write( smb347_dev,
                                       CFG_PIN,
                                       0,
                                       BIT(6)|BIT(5));
        } else {
            ret = smb347_update_write( smb347_dev,
                                       CFG_PIN,
                                       BIT(6)|BIT(5),
                                       0);
        }

        if (ret < 0)
	      goto out;
out:
	mutex_unlock(&smb347_dev->lock);
	return ret;
}

static irqreturn_t smb347_inok_interrupt(int irq, void *data) {
	struct smb347_charger *smb = data;

	irqreturn_t ret = IRQ_NONE;

	pm_runtime_get_sync(&smb->client->dev);

	dev_warn(&smb->client->dev, "%s\n", __func__);
	if (gpio_get_value_cansleep(inok_gpio)) {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (HIGH) <<<\n", __func__);
	} else {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (LOW) <<<\n", __func__);
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}

static int smb347_inok_gpio_init(struct smb347_charger *smb) {
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(inok_gpio);

	ret = gpio_request_one(inok_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0) {
		printk(KERN_DEBUG "smb347: request INOK gpio fail!\n");
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb347_inok_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					smb->client->name,
					smb);
	if (ret < 0) {
		printk(KERN_DEBUG "smb347: config INOK gpio as IRQ fail!\n");
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

bool smb347_has_charger_error(void) {
	int ret;

	if (!smb347_dev)
		return -EINVAL;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb347_get_charging_status(void) {
	int ret, status;

	if (!smb347_dev)
		return -EINVAL;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return ret;

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	return status;
}
//EXPORT_SYMBOL_GPL(smb347_get_charging_status);

static int smb347_debugfs_show(struct seq_file *s, void *data) {
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "=========  smb345 =========\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");
	ret = smb347_read(smb, CMD_A);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_A, BYTETOBINARY(ret));
	ret = smb347_read(smb, CMD_B);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_B, BYTETOBINARY(ret));
	ret = smb347_read(smb, CMD_C);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_C, BYTETOBINARY(ret));
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file) {
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//smb3xx_max_current      ------START
static ssize_t smb3xx_max_current_write(struct file *file, const char *buffer, size_t len, loff_t *data) {
    printk(" %s:\n", __func__);

    if (buffer[0] == '1') {
        // set max current setting
        max_current_set = 1;
    } else {
        max_current_set = 0;
    }

    return len;
}

static int smb3xx_max_current_read(struct seq_file *m, void *p) {
    int len;

    if (max_current_set == 1)
        len = seq_printf(m,"smb3xx_max_current: 1800 mA\n");
    else
        len = seq_printf(m,"smb3xx_max_current: 1200 mA\n");
    return len;
}

static int smb3xx_max_current_open(struct inode *inode, struct file *file) {
	return single_open(file, smb3xx_max_current_read, NULL);
}

static const struct file_operations smb3xx_max_current_ops = {
	.open		= smb3xx_max_current_open,
	.read		= seq_read,
        .write          = smb3xx_max_current_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};
//smb3xx_max_current      ------END

int smb3xx_register_max_current_proc_fs(void) {
        struct proc_dir_entry *entry=NULL;

        entry = proc_create("smb3xx_max_current", 0664, NULL, &smb3xx_max_current_ops);
        if (!entry) {
            printk("[%s]Unable to create smb3xx max current \n", __FUNCTION__);
            return -EINVAL;
        }

        return 0;
}

#ifdef USB_NOTIFY_CALLBACK
extern unsigned int query_cable_status(void);

static int cable_status_notify(struct notifier_block *self, unsigned long action, void *dev) {

   if (ischargerSuspend) {
       printk(KERN_INFO "%s chager is suspend but USB still notify !!!\n", __func__);
       wake_lock(&wakelock_smb347);
       isUSBSuspendNotify = true;
       return NOTIFY_OK;
   }

   switch (action) {
      case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_SDP !!!\n", __func__);
          setSMB347Charger(USB_IN);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_CDP !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_DCP !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_SE1:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_SE1 !!!\n", __func__);
          setSMB347Charger(AC_IN);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED !!!\n", __func__);
          setSMB347Charger(ENABLE_5V);
          break;

      case POWER_SUPPLY_CHARGER_TYPE_NONE:
          printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_NONE !!!\n", __func__);
          setSMB347Charger(CABLE_OUT);
	  break;

      default:
          printk(KERN_INFO "%s no status = %d !!!\n", __func__, (int)action);
	  break;
   }
   return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier = {
	.notifier_call = cable_status_notify,
};

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);
#endif

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev, struct device_attribute *attr, char *buf) {
	int ret;

	if (!smb347_dev) {
		pr_info("%s: ERROR: smb347_dev is null due to probe function has error\n",  __func__);
		return sprintf(buf, "%d\n", -EINVAL);
	}

	ret = smb347_read(smb347_dev, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;
	return sprintf(buf, "%d\n",
			hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev, struct device_attribute *attr, char *buf) {
	int ret;

	ret = smb347_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#if TF303CL_CHARGER_ACPI
static struct smb347_charger_platform_data smb347_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.max_charge_current		= 3360000,
	.max_charge_voltage		= 4200000,
	.otg_uvlo_voltage		= 3300000,
	.chip_temp_threshold		= 120,
	.soft_cold_temp_limit		= 5,
	.soft_hot_temp_limit		= 50,
	.hard_cold_temp_limit		= 5,
	.hard_hot_temp_limit		= 55,
	.suspend_on_hard_temp_limit	= true,
	.soft_temp_limit_compensation	= SMB347_SOFT_TEMP_COMPENSATE_CURRENT
					| SMB347_SOFT_TEMP_COMPENSATE_VOLTAGE,
	.charge_current_compensation	= 900000,
	.use_mains			= true,
	.irq_gpio			= -1,
	.inok_gpio			= -1,
};
#endif

static int smb347_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	const struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;

	printk(KERN_DEBUG "==== smb345_probe ====\n");
#if TF303CL_CHARGER_ACPI
        pdata = &smb347_pdata;
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);
	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

	/* enable register writing - Tom */
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	// ++ Refer to SMB347 Application Note 72 to solve serious problems - tom - disable otg and set 250 mA
	ret = smb347_update_write(smb, 0x30, BIT(4), 0);
	if (ret < 0)
		return ret;

	ret = smb347_update_write(smb,
							0x0A,
							BIT(3)|BIT(2)|BIT(1)|BIT(0),
							BIT(2));
	if (ret < 0)
		return ret;

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

	/* INOK pin configuration */
        inok_gpio = pdata->inok_gpio;
	if (inok_gpio >= 0) {
		ret = smb347_inok_gpio_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize INOK gpio: %d\n", ret);
		}
	}

	smb->running = true;
	smb->dentry = debugfs_create_file("smb345-regs", S_IRUSR, NULL, smb,  &smb347_debugfs_fops);
	smb347_dev = smb;

        charger_work_queue = create_singlethread_workqueue("charger_workqueue");
        INIT_DELAYED_WORK(&charger_work, do_charger);

        ret = smb3xx_register_max_current_proc_fs();
        if (ret < 0) {
		dev_warn(dev, "failed to register_max_current_proc_fs: %d\n", ret);
	}

#ifdef USB_NOTIFY_CALLBACK
	cable_status_register_client(&cable_status_notifier);
        cable_status_notify( NULL, query_cable_status(), dev);
#endif

	wake_lock_init(&wakelock_smb347, WAKE_LOCK_SUSPEND, "smb347_wakelock");
	wake_lock_init(&wakelock_smb347_t, WAKE_LOCK_SUSPEND, "smb347_wakelock_timeout");

	ret = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);
	if (ret < 0) {
		dev_warn(dev, "failed to sysfs_create_group: %d\n", ret);
	}

        printk(KERN_DEBUG "==== smb345_probe done ====\n");
	return 0;
}

static int smb347_remove(struct i2c_client *client) {
	struct smb347_charger *smb = i2c_get_clientdata(client);
#ifdef USB_NOTIFY_CALLBACK
	cable_status_unregister_client(&cable_status_notifier);
#endif
	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

	pm_runtime_get_noresume(&smb->client->dev);

	return 0;
}

void smb347_shutdown(struct i2c_client *client) {
	dev_info(&client->dev, "%s\n", __func__);

        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
             /* SOC control JEIAT*/
             smb347_control_JEITA(false);
        }

	/* Disable OTG during shutdown */
	otg(0);
	return;
}

#ifdef CONFIG_PM
static int smb347_prepare(struct device *dev) {
	struct smb347_charger *smb = dev_get_drvdata(dev);

	/*
	 * disable irq here doesn't mean smb347 interrupt
	 * can't wake up system. smb347 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0) disable_irq(smb->client->irq);

	dev_info(&smb->client->dev, "smb347 suspend\n");
	return 0;
}

static void smb347_complete(struct device *dev) {
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb->client->irq > 0)
		enable_irq(smb->client->irq);

	dev_info(&smb->client->dev, "smb347 complete\n");
}
#else
#define smb347_prepare NULL
#define smb347_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb347_runtime_suspend(struct device *dev) {
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_resume(struct device *dev) {
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_idle(struct device *dev) {
	dev_info(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb347_runtime_suspend	NULL
#define smb347_runtime_resume	NULL
#define smb347_runtime_idle	NULL
#endif

static int smb347_suspend(struct device *dev) {
    printk("%s called\n", __func__);

    if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
        /* SOC control JEIAT*/
        smb347_control_JEITA(false);
    }
    ischargerSuspend = true;
    return 0;
}

static int smb347_resume(struct device *dev) {
    printk("%s called\n", __func__);

    if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
        /* SOC control JEIAT*/
        smb347_control_JEITA(true);
    }
    ischargerSuspend = false;
    if (isUSBSuspendNotify) {
       isUSBSuspendNotify = false;
#ifdef USB_NOTIFY_CALLBACK
       cable_status_notify( NULL, query_cable_status(), dev);
#endif
       wake_unlock(&wakelock_smb347);
    }
    return 0;
}


static const struct i2c_device_id smb347_id[] = {
	{ "SMB0345", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static const struct dev_pm_ops smb347_pm_ops = {
	.prepare		= smb347_prepare,
	.complete		= smb347_complete,
	.suspend                = smb347_suspend,
	.resume	                = smb347_resume,
	.runtime_suspend	= smb347_runtime_suspend,
	.runtime_resume		= smb347_runtime_resume,
	.runtime_idle		= smb347_runtime_idle,
};

static struct i2c_driver smb347_driver = {
	.driver = {
		.name	= "SMB0345",
		.owner	= THIS_MODULE,
		.pm	= &smb347_pm_ops,
#if TF303CL_CHARGER_ACPI
                .acpi_match_table = ACPI_PTR(smb347_id),
#endif
	},
	.probe		= smb347_probe,
	.remove		= smb347_remove,
	.shutdown	= smb347_shutdown,
	.id_table	= smb347_id,
};

static int __init smb347_init(void) {
	printk(KERN_DEBUG "++++ Inside smb347_init ++++\n");
	return i2c_add_driver(&smb347_driver);
}
late_initcall(smb347_init);

static void __exit smb347_exit(void) {
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Tom Shen <tom_shen@asus.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
