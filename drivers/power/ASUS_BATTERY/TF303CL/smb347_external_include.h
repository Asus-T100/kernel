#ifndef _SMB347_EXTERNAL_INCLUDE_H
#define _SMB347_EXTERNAL_INCLUDE_H 1

typedef enum {
    USB_IN,
    AC_IN,
    CABLE_OUT,
    ENABLE_5V,
    DISABLE_5V,
} cable_status;

enum {
    TEMP_0,
    TEMP_0_45,
    TEMP_45_55,
    TEMP_55,
};

enum {
    USB5_1_HC_MAX,
    USE_AICL,
};

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB347Charger(int usb_state);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb347_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb347_charging_toggle(bool on);

/* To set fast charge
 *
 * return 0 means success */
extern int smb347_set_fast_charge(void);

/* To set ac in current
 *
 * return 0 means success */
extern int smb347_AC_in_current(int c, int type);

/* To control JEITA
 *
 * return 0 means success */
extern int smb347_control_JEITA(bool on);


/* To set voltage
 *
 * return 0 means success */
extern int smb347_set_voltage(int v);


/* To set Battery OV
 *
 * return 0 means success */
extern int smb347_set_battery_OV(void);

/*
 * To get AICL result
 */
extern int smb347_get_aicl_result(void);

/* To know if charger has an error
 *
 * return true means charger has an error */
bool smb347_has_charger_error(void);

#define LG "1"
#define COSLIGHT "0"
#define SMB345_DEV_NAME "smb345"

#endif /* _SMB347_EXTERNAL_INCLUDE_H */

