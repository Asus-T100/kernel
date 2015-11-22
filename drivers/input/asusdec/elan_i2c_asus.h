#ifndef _ELAN_I2C_ASUS_H
#define _ELAN_I2C_ASUS_H
#include "asusdec.h"
#define ELAN_DEBUG			1
#if ELAN_DEBUG
#define ELAN_INFO(format, arg...)	\
	printk(KERN_INFO "elan_i2c_asus: [%s] " format , __FUNCTION__ , ## arg)
#else
#define ELAN_INFO(format, arg...)
#endif


#define ELAN_ERR(format, arg...)	\
	printk(KERN_ERR "elan_i2c_asus: [%s] " format , __FUNCTION__ , ## arg)

#define CONVERSION_TIME_MS		50
#define ELANTOUCHPAD		727

#define HID_DESC_LENGTH		30
#define HID_REPORT_ID_OFFSET	2

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH		2
#define ETP_MAX_FINGERS		5
#define ETP_REPORT_DESC_LENGTH	79
#define ETP_REPORT_LENGTH	30
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_FINGER_DATA_LEN	5

#define ETP_REPORT_ID		0x5d

#define HID_CMD_REGISTER	0x0005
#define ETP_CMD_REGISTER	0x0300

#define CMD_RESET		0x0100
#define CMD_WAKE_UP		0x0800
#define CMD_SLEEP		0x0801
#define CMD_ENABLE_ABS		0x0001

#define REG_DESC		0x0001
#define REG_REPORT_DESC		0x0002
#define REG_INPUT_REPORT		0x0003
#define REG_FW_VERSION		0x0102
#define REG_XY_TRACE_NUM	0x0105
#define REG_X_AXIS_MAX		0x0106
#define REG_Y_AXIS_MAX		0x0107
#define REG_RESOLUTION		0x0108

int elan_i2c_initialize(struct i2c_client *client);
int elan_i2c_enable(struct i2c_client *client);
int elan_i2c_disable(struct i2c_client *client);
void elan_i2c_report_absolute(struct elan_i2c_data *data,
				     u8 *packet);
int elan_i2c_input_dev_create(struct elan_i2c_data *data);
int elan_i2c_check_packet(u8 *packet);
void elan_i2c_report_standard(struct asusdec_chip *ec_chip,
				     u8 *packet);
#if 0
int elan_i2c_get_input_report(struct i2c_client *client, u8 *val);
#endif

/* The main device structure */
#endif
