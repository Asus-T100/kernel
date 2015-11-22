#ifndef _ASUS_EC_POWER_H_
#define _ASUS_EC_POWER_H_
#include <linux/kernel.h>

typedef enum {
    RET_EC_OK = 0,
    RET_EC_FAIL = -1, // i2c fail
    RET_EC_UPDATE = -2, // EC firmware updateing,can not do any i2c command
    RET_GAUGE_FAIL = -3,
} ret_type_t;

int asus_pad_batt_write_charger_reg(u8 reg_lsb,u8 lsb);
int asus_pad_batt_read_charger_reg(u8 reg);
int batt_gaguefw_write(u8 *buf);
int ec_power_changed_all();

/* Restart Charge F/P ratio When system suspend
   F/P<=0.8, F<=P*0.8, F<=P*80/100, THEN 80 is defined as below
*/
#define RESTART_PHONE_BATT_CHARGE_RATIO    80
/* Stop Charge F/P ratio When system suspend
   F/P<=1.1, F<=P*1.1, F<=P*110/100, THEN 110 is defined as below
*/
#define STOP_PHONE_BATT_CHARGE_RATIO       110
#endif
