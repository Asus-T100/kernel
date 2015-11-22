#ifndef _TC358774_H
#define _TC358774_H

#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

#define PPI_TX_RX_TA		0x013C
#define PPI_LPTXTIMCNT		0x0114
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define PPI_D2S_CLRSIPOCOUNT	0x016C
#define PPI_D3S_CLRSIPOCOUNT	0x0170
#define PPI_LANEENABLE		0x0134
#define DSI_LANEENABLE		0x0210
#define PPI_STARTPPI		0x0104
#define DSI_STARTPPI		0x0204
/* Video Path Registers */
#define VPCTRL			0x0450
#define HTIM1			0x0454
#define HTIM2			0x0458
#define VTIM1			0x045C
#define VTIM2			0x0460
#define VFUEN			0x0464
/* LVDS Registers */
#define LVMX0003		0x0480
#define LVMX0407		0x0484
#define LVMX0811		0x0488
#define LVMX1215		0x048C
#define LVMX1619		0x0490
#define LVMX2023		0x0494
#define LVMX2427		0x0498
#define LVCFG			0x049C
#define LVPHY0			0x04A0
#define LVPHY1			0x04A4
/* System Registers */
#define SYSSTAT			0x0500
#define SYSRST			0x0504
/* Chip/Rev Registers */
#define IDREG			0x0580
/* DEBUG Register */
#define DEBUG00         0x05A0

#define BRIDGE_I2C_ADAPTER	2
#define BRIDGE_I2C_ADDR	0x0F
#define BRIDGE_STANDBY 128
#define BRIDGE_RESET 69

extern struct i2c_client *bridge_i2c_client;
extern struct i2c_adapter *bridge_i2c_adapter;
extern void tc358774_configure_lvds_bridge(void);
extern void initial_bridge(void);
extern bool TS_bridge_cfg;
int tc358774_regw(struct i2c_client *client, u16 reg, u32 value);
int tc358774_regr(struct i2c_client *client, u16 reg, u32 *value);

#endif
