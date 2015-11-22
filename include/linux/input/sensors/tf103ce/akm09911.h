/*
 * Definitions for akm09911 compass chip.
 */
#ifndef AKM09911_H
#define AKM09911_H

#include <linux/ioctl.h>

/* Device specific constant values */
#define AK09911_REG_WIA1			0x00
#define AK09911_REG_WIA2			0x01
#define AK09911_REG_INFO1			0x02
#define AK09911_REG_INFO2			0x03
#define AK09911_REG_ST1				0x10
#define AK09911_REG_HXL				0x11
#define AK09911_REG_HXH				0x12
#define AK09911_REG_HYL				0x13
#define AK09911_REG_HYH				0x14
#define AK09911_REG_HZL				0x15
#define AK09911_REG_HZH				0x16
#define AK09911_REG_TMPS			0x17
#define AK09911_REG_ST2				0x18
#define AK09911_REG_CNTL1			0x30
#define AK09911_REG_CNTL2			0x31
#define AK09911_REG_CNTL3			0x32

#define AK09911_FUSE_ASAX			0x60
#define AK09911_FUSE_ASAY			0x61
#define AK09911_FUSE_ASAZ			0x62

#define AK09911_MODE_SNG_MEASURE	0x01
#define AK09911_MODE_SELF_TEST		0x10
#define AK09911_MODE_FUSE_ACCESS	0x1F
#define AK09911_MODE_POWERDOWN		0x00
#define AK09911_RESET_DATA			0x01

#define AK09911_REGS_SIZE		13
#define AK09911_WIA1_VALUE		0x48
#define AK09911_WIA2_VALUE		0x05

/* To avoid device dependency, convert to general name */
//#define AKM_I2C_NAME			"akm09911"
// to match IFWI I2C name
#define AKM_I2C_NAME			"AKM9911"
#define AKM_MISCDEV_NAME		"akm09911_dev"
#define AKM_SYSCLS_NAME			"compass"
#define AKM_SYSDEV_NAME			"akm09911"
#define AKM_REG_MODE			AK09911_REG_CNTL2
#define AKM_REG_RESET			AK09911_REG_CNTL3
#define AKM_REG_STATUS			AK09911_REG_ST1
#define AKM_WIA_VALE			0x48
#define AKM_MEASURE_TIME_US		10000
#define AKM_DRDY_IS_HIGH(x)		((x) & 0x01)
#define AKM_SENSOR_INFO_SIZE	2
#define AKM_SENSOR_CONF_SIZE	3
#define AKM_SENSOR_DATA_SIZE	9

#define AKM_YPR_DATA_SIZE		23
#define AKM_RWBUF_SIZE			16
#define AKM_REGS_SIZE			AK09911_REGS_SIZE
#define AKM_REGS_1ST_ADDR		AK09911_REG_WIA1
#define AKM_FUSE_1ST_ADDR		AK09911_FUSE_ASAX

#define AKM_MODE_SNG_MEASURE	AK09911_MODE_SNG_MEASURE
#define AKM_MODE_SELF_TEST		AK09911_MODE_SELF_TEST
#define AKM_MODE_FUSE_ACCESS	AK09911_MODE_FUSE_ACCESS
#define AKM_MODE_POWERDOWN		AK09911_MODE_POWERDOWN
#define AKM_RESET_DATA			AK09911_RESET_DATA

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define MAGUC_DATA_FLAG     2
#define FUSION_DATA_FLAG	3
#define AKM_NUM_SENSORS		4

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define MAGUC_DATA_READY    (1<<(MAGUC_DATA_FLAG))
#define FUSION_DATA_READY	(1<<(FUSION_DATA_FLAG))

#define AKMIO				0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_READ				_IOWR(AKMIO, 0x01, char)
#define ECS_IOCTL_WRITE				_IOW(AKMIO, 0x02, char)
#define ECS_IOCTL_RESET				_IO(AKMIO, 0x03)
#define ECS_IOCTL_SET_MODE			_IOW(AKMIO, 0x10, char)
#define ECS_IOCTL_SET_YPR			_IOW(AKMIO, 0x11, int32_t[AKM_YPR_DATA_SIZE])
#define ECS_IOCTL_GET_INFO			_IOR(AKMIO, 0x20, unsigned char[AKM_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF			_IOR(AKMIO, 0x21, unsigned char[AKM_SENSOR_CONF_SIZE])
#define ECS_IOCTL_GET_DATA			_IOR(AKMIO, 0x22, unsigned char[AKM_SENSOR_DATA_SIZE])
#define ECS_IOCTL_GET_OPEN_STATUS	_IOR(AKMIO, 0x23, int)
#define ECS_IOCTL_GET_CLOSE_STATUS	_IOR(AKMIO, 0x24, int)
#define ECS_IOCTL_GET_DELAY			_IOR(AKMIO, 0x25, long long int)
#define ECS_IOCTL_GET_LAYOUT		_IOR(AKMIO, 0x26, char)
#define ECS_IOCTL_GET_ACCEL			_IOR(AKMIO, 0x30, short[3])

struct akm09911_platform_data {
	char layout;
	int gpio_DRDY;
	int gpio_RSTN;
};

#define AKM_I2C_ADAPTER			0x05
#define AKM_I2C_ADDRESS			0x0c
static struct akm09911_platform_data akm09911_pdata = {
	.layout = 8,
	.gpio_RSTN = 0,
};
static struct i2c_board_info akm09911_board_info = {
	I2C_BOARD_INFO(AKM_I2C_NAME, AKM_I2C_ADDRESS),
	.flags = I2C_CLIENT_WAKE,
	.platform_data = &akm09911_pdata,
	.irq = 0,
};

#endif

