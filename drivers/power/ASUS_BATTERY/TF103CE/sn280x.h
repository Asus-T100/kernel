/*
 * sn280x_charger.h: platform data structure for SN280X driver
 *
 * (C) Copyright 2012 Intel Corporation
 * (C) Copyright 2013, ASUSTek, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __SN280X_CHARGER_H__
#define __SN280X_CHARGER_H__

/*---------------------------------------------------------------------------*/

#define I2C_RETRY_COUNT                                     3
#define I2C_RETRY_DELAY                                     5

#define SN280X_MASK(N_BITS, POS)  ((unsigned char)(((1 << N_BITS) - 1) << POS))

#define SN280X_N_REGISTERS                                  11

#define SN280X_REG_INPUT_SOURCE_CONTROL                     0x00
#define SN280X_REG_POWER_ON_CONFIGURATION                   0x01
#define SN280X_REG_CHARGE_CURRENT_CONTROL                   0x02
#define SN280X_REG_PRECHARGE_TERMINATION                    0x03
#define SN280X_REG_CHARGE_VOLTAGE_CONTROL                   0x04
#define SN280X_REG_CHARGE_TERMINATION_TIMER                 0x05
#define SN280X_REG_BOOST_VOL_THERMAL_REGULATION             0x06
#define SN280X_REG_MISC_OPERATION_CONTROL                   0x07
#define SN280X_REG_SYSTEM_STATUS                            0x08
#define SN280X_REG_NEW_FAULT                                0x09
#define SN280X_REG_VENDOR_PART_REV_STATUS                   0x0A

#define SN280X_MASK_INPUT_VOLTAGE_LIMIT                     SN280X_MASK(4, 3)
#define SN280X_MASK_INPUT_CURRENT_LIMIT                     SN280X_MASK(3, 0)
#define SN280X_MASK_CHARGER_CONFIGURATION                   SN280X_MASK(2, 4)
#define SN280X_MASK_CHARGER_CONFIGURATION_JEITA             SN280X_MASK(1, 4)
#define SN280X_MASK_BOOST_CURRENT_LIMIT                     SN280X_MASK(1, 0)
#define SN280X_MASK_FAST_CHARGE_CURRENT_LIMIT               SN280X_MASK(6, 2)
#define SN280X_MASK_TERMINATION_CURRENT_LIMIT               SN280X_MASK(4, 0)
#define SN280X_MASK_CHARGE_VOLTAGE_LIMIT                    SN280X_MASK(6, 2)
#define SN280X_MASK_I2C_WATCHDOG_TIMER_LIMIT                SN280X_MASK(2, 4)
#define SN280X_MASK_BOOST_VOLTAGE                           SN280X_MASK(4, 4)
#define SN280X_MASK_VBUS_STAT                               SN280X_MASK(2, 6)
#define SN280X_MASK_CHRG_STAT                               SN280X_MASK(2, 4)

#define SN280X_VAL_INPUT_VOLTAGE_LIMIT_AC_IN                0x30
#define SN280X_VAL_INPUT_CURRENT_LIMIT_AC_IN                0x05
#define SN280X_VAL_CHARGER_CONFIGURATION_OTG                0x20
#define SN280X_VAL_CHARGER_CONFIGURATION_OTG_DISABLE        0x10
#define SN280X_VAL_BOOST_CURRENT_LIMIT_OTG                  0x00
#define SN280X_VAL_FST_CHRG_CURRENT_LIMIT_AC_IN             0x50
#define SN280X_VAL_TERMI_CURRENT_LIMIT_AC_IN                0x01
#define SN280X_VAL_CHARGE_VOLTAGE_LIMIT_AC_IN               0xD4
#define SN280X_VAL_I2C_WATCHDOG_TIMER_LIMIT                 0x00
#define SN280X_VAL_BOOST_VOLTAGE_OTG                        0x70
#define SN280X_VAL_VBUS_STAT_OTG                            0xC0
#define SN280X_VAL_VBUS_STAT_USB_IN                         0x40
#define SN280X_VAL_VBUS_STAT_AC_IN                          0x80

#define SN280X_JEITA_VAL_CHARGER_CONFIG_DISABLE             0x00
#define SN280X_JEITA_VAL_CHARGER_CONFIG_ENABLE              0x10
#define SN280X_JEITA_VAL_FST_CHRG_CURRENT_LIMIT_LOW_TEMP    0x34
#define SN280X_JEITA_VAL_FST_CHRG_CURRENT_LIMIT_HIGH_TEMP   0x50
#define SN280X_JEITA_VAL_CHARGE_VOLTAGE_LIMIT_LOW_TEMP      0xD4
#define SN280X_JEITA_VAL_CHARGE_VOLTAGE_LIMIT_HIGH_TEMP     0x94

int sn280x_set_boost_otg(int enable);
int sn280x_set_chrgr_type(int usb_state);
int sn280x_soc_control_jeita(void);
int sn280x_get_charging_status(void);
int sn280x_chgr_state(int usb_state);

/*---------------------------------------------------------------------------*/

struct sn280x_plat_data {
    char **supplied_to;
    size_t num_supplicants;
    struct power_supply_throttle *throttle_states;
    size_t num_throttle_states;
    int safety_timer;
    int boost_mode_mA;
    bool is_ts_enabled;

    int (*enable_charging) (bool val);
    int (*enable_charger) (bool val);
    int (*set_inlmt) (int val);
    int (*set_cc) (int val);
    int (*set_cv) (int val);
    int (*set_iterm) (int val);
    int (*enable_vbus) (int val);
    void (*dump_master_regs) (void);
};

extern void sn280x_cv_to_reg(int, u8*);
extern void sn280x_cc_to_reg(int, u8*);

#ifdef CONFIG_SN280X_CHARGER
extern int sn280x_get_bat_health(void);
extern int sn280x_get_bat_status(void);
#else
static int sn280x_get_bat_health(void)
{
    return 0;
}
static int sn280x_get_bat_status(void)
{
    return 0;
}
#endif

#endif
