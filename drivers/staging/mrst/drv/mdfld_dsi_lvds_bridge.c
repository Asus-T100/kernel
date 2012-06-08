/*
 * Copyright © 2011 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_lvds_bridge.h"
#include "psb_drv.h"
#include <asm/intel_scu_ipc.h>
#include "psb_powermgmt.h"

#define CONFIG_LVDS_HARD_RESET
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE

/* All these pins removed on DV1.0 */
#define GPIO_MIPI_LCD_BIAS_EN	-1
#define GPIO_MIPI_PANEL_RESET	-1
#define GPIO_MIPI_LCD_STBYB	-1
#define GPIO_MIPI_LCD_COLOR_EN	-1

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004
#define CLW_DPHYCONTRX		0x0020
#define D0W_DPHYCONTRX		0x0024
#define D1W_DPHYCONTRX		0x0028
#define D2W_DPHYCONTRX		0x002C
#define D3W_DPHYCONTRX		0x0030
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define D2W_CNTRL		0x004C
#define D3W_CNTRL		0x0050
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
#define PPI_LANEENABLE		0x0134
#define PPI_TX_RX_TA		0x013C
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D2S_ATMR		0x014C
#define PPI_D3S_ATMR		0x0150
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define PPI_D2S_CLRSIPOCOUNT	0x016C
#define PPI_D3S_CLRSIPOCOUNT	0x0170
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define D2S_PRE			0x018C
#define D3S_PRE			0x0190
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define D2S_PREP		0x01AC
#define D3S_PREP		0x01B0
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define D2S_ZERO		0x01CC
#define D3S_ZERO		0x01D0
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230

/* DSI General Registers */
#define DSIERRCNT		0x0300

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define RDPKTLN			0x0404

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

/* GPIO Registers */
#define GPIOC			0x0520
#define GPIOO			0x0524
#define GPIOI			0x0528

/* I2C Registers */
#define I2CTIMCTRL		0x0540
#define I2CMADDR		0x0544
#define WDATAQ			0x0548
#define RDATAQ			0x054C

/* Chip/Rev Registers */
#define IDREG			0x0580

/* Input muxing for registers LVMX0003...LVMX2427 */
enum {
	INPUT_R0,	/* 0 */
	INPUT_R1,
	INPUT_R2,
	INPUT_R3,
	INPUT_R4,
	INPUT_R5,
	INPUT_R6,
	INPUT_R7,
	INPUT_G0,	/* 8 */
	INPUT_G1,
	INPUT_G2,
	INPUT_G3,
	INPUT_G4,
	INPUT_G5,
	INPUT_G6,
	INPUT_G7,
	INPUT_B0,	/* 16 */
	INPUT_B1,
	INPUT_B2,
	INPUT_B3,
	INPUT_B4,
	INPUT_B5,
	INPUT_B6,
	INPUT_B7,
	INPUT_HSYNC,	/* 24 */
	INPUT_VSYNC,
	INPUT_DE,
	LOGIC_0,
	/* 28...31 undefined */
};

#define INPUT_MUX(lvmx03, lvmx02, lvmx01, lvmx00)		\
	(FLD_VAL(lvmx03, 29, 24) | FLD_VAL(lvmx02, 20, 16) |	\
	FLD_VAL(lvmx01, 12, 8) | FLD_VAL(lvmx00, 4, 0))
struct i2c_client *cmi_lcd_i2c_client;
static struct i2c_client *tc35876x_client;

struct cmi_lcd_data {
	bool enabled;
	struct mutex lock; /* for enabled */
};

int lvds_disp_init;

struct dsi_lvds_bridge_instance {
	struct i2c_client *client;
	struct work_struct wqueue;
	struct work_struct pre_init_work;
	struct work_struct test_work;
} *dsi_lvds_inst;

/**
 * tc35876x_regw - Write DSI-LVDS bridge register using I2C
 * @client: struct i2c_client to use
 * @reg: register address
 * @value: value to write
 *
 * Returns 0 on success, or a negative error value.
 */
static int tc35876x_regw(struct i2c_client *client, u16 reg, u32 value)
{
	int r;
	u8 tx_data[] = {
		/* NOTE: Register address big-endian, data little-endian. */
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

/**
 * tc35876x_regr - Read DSI-LVDS bridge register using I2C
 * @client: struct i2c_client to use
 * @reg: register address
 * @value: pointer for storing the value
 *
 * Returns 0 on success, or a negative error value.
 */
static int tc35876x_regr(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};
	u8 rx_data[4];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0] << 24 | rx_data[1] << 16 |
		rx_data[2] << 8 | rx_data[3];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}
static int tc35876x_bridge_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	dev_info(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c_check_functionality() failed\n",
			__func__);
		return -ENODEV;
	}

	gpio_request(GPIO_MIPI_BRIDGE_RESET, "tc35876x bridge reset");
	gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0);

	gpio_request(GPIO_MIPI_LCD_BL_EN, "tc35876x panel bl en");
	gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 0);

	tc35876x_client = client;

	return 0;
}

static int tc35876x_bridge_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s\n", __func__);

	gpio_free(GPIO_MIPI_BRIDGE_RESET);
	gpio_free(GPIO_MIPI_LCD_BL_EN);

	tc35876x_client = NULL;

	return 0;
}


static const struct i2c_device_id tc35876x_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc35876x_bridge_id);
static struct i2c_driver tc35876x_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = tc35876x_bridge_id,
	.probe = tc35876x_bridge_probe,
	.remove = __devexit_p(tc35876x_bridge_remove),
};

static int DSI_I2C_ByteRead(u16 reg, int count);
static int DSI_I2C_ByteWrite(u16 reg, u32 data, int count);

static bool lvds_suspend_state;

/************************************************************************** *\
 * FUNCTION: dsi_lvds_suspend_lvds_bridge
 *
 * DESCRIPTION:  This function is called by psb power management
 *				 during early suspend.
\* ************************************************************************* */
void dsi_lvds_suspend_lvds_bridge(struct drm_device *dev)
{
	printk(KERN_INFO "[DISPLAY ] Enter %s\n", __func__);

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
	usleep_range(500, 1000);

	lvds_suspend_state = true;
}

/************************************************************************** *\
 * FUNCTION: dsi_lvds_resumed_lvds_bridge
 *
 * DESCRIPTION:  This function is called by psb power management
 *				 during late resume.
\* ************************************************************************* */
void dsi_lvds_resume_lvds_bridge(struct drm_device *dev)
{
	printk(KERN_INFO "[DISPLAY ] Enter %s\n", __func__);

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 1);

	/* VADD */
	if (gpio_direction_output(GPIO_MIPI_LCD_VADD, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_VADD, 1);
	msleep(10);

	/* RESET */
	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);
	usleep_range(500, 1000);

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
	usleep_range(500, 1000);

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);
	usleep_range(500, 1000);

	lvds_suspend_state = false;
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_set_bridge_reset_state
 *
 * DESCRIPTION:  This function uses GPIO to force in and out of reset state.
\* ************************************************************************* */
void dsi_lvds_set_bridge_reset_state(int state)
{
	printk(KERN_INFO "[DISPLAY ] %s: state = %d, gpio = %d\n", __func__, state, gpio_get_value(GPIO_MIPI_BRIDGE_RESET));

	if (state) {
		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
		/* FIXME:
		 * per spec, the min period of reset signal is 50 nano secs,
		 * but no detailed description. Here wait 0.5~1ms for safe.
		 */
		usleep_range(500, 1000);
	} else {

		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);  /*Pull MIPI Bridge reset pin to Low */
		usleep_range(500, 1000);
		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);  /*Pull MIPI Bridge reset pin to High */
		usleep_range(500, 1000);
	}
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_configure_lvds_bridge
 *
 * DESCRIPTION:  This function uses I2C interface to set bridge registers.
 *				to configure timings and MIPI lanes.
\* ************************************************************************* */
void dsi_lvds_configure_lvds_bridge(struct drm_device *dev)
{
	struct i2c_client *i2c = tc35876x_client;
	u32 id = 0;
	u32 ppi_lptxtimecnt;
	u32 txtagocnt;
	u32 txtasurecnt;

	if (lvds_disp_init) {
		printk(KERN_ALERT "[DISPLAY] %s is already initialized\n",
				__func__);
		return;
	}

	printk(KERN_INFO "[DISPLAY ]%s: Enter\n", __func__);

	tc35876x_regr(i2c, IDREG, &id);
	printk(KERN_INFO "[DISPLAY] tc35876x ID 0x%08x\n", id);

	ppi_lptxtimecnt = 4;
	txtagocnt = (5 * ppi_lptxtimecnt - 3) / 4;
	txtasurecnt = 3 * ppi_lptxtimecnt / 2;

	tc35876x_regw(i2c, PPI_TX_RX_TA, FLD_VAL(txtagocnt, 26, 16) |
		FLD_VAL(txtasurecnt, 10, 0));
	tc35876x_regw(i2c, PPI_LPTXTIMECNT, FLD_VAL(ppi_lptxtimecnt, 10, 0));

	tc35876x_regw(i2c, PPI_D0S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D1S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D2S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));
	tc35876x_regw(i2c, PPI_D3S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0));

	/* Enabling MIPI & PPI lanes, Enable 4 lanes */
	tc35876x_regw(i2c, PPI_LANEENABLE,
		BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
	tc35876x_regw(i2c, DSI_LANEENABLE,
		BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
	tc35876x_regw(i2c, PPI_STARTPPI, BIT0);
	tc35876x_regw(i2c, DSI_STARTDSI, BIT0);

	/* Setting LVDS output frequency */
	tc35876x_regw(i2c, LVPHY0, FLD_VAL(1, 20, 16) |
		FLD_VAL(2, 15, 14) | FLD_VAL(6, 4, 0)); /* 0x00048006 */

	/* Setting video panel control register,0x00000120 VTGen=ON ?!?!? */
	tc35876x_regw(i2c, VPCTRL, BIT(8) | BIT(5));

	/* Horizontal back porch and horizontal pulse width. 0x00280028 */
	tc35876x_regw(i2c, HTIM1, FLD_VAL(40, 24, 16) | FLD_VAL(40, 8, 0));

	/* Horizontal front porch and horizontal active video size. 0x00500500*/
	tc35876x_regw(i2c, HTIM2, FLD_VAL(80, 24, 16) | FLD_VAL(1280, 10, 0));

	/* Vertical back porch and vertical sync pulse width. 0x000e000a */
	tc35876x_regw(i2c, VTIM1, FLD_VAL(14, 23, 16) | FLD_VAL(10, 7, 0));

	/* Vertical front porch and vertical display size. 0x000e0320 */
	tc35876x_regw(i2c, VTIM2, FLD_VAL(14, 23, 16) | FLD_VAL(800, 10, 0));

	/* Set above HTIM1, HTIM2, VTIM1, and VTIM2 at next VSYNC. */
	tc35876x_regw(i2c, VFUEN, BIT0);

	/* Soft reset LCD controller. */
	tc35876x_regw(i2c, SYSRST, BIT2);

	/* LVDS-TX input muxing */
	tc35876x_regw(i2c, LVMX0003,
		INPUT_MUX(INPUT_R5, INPUT_R4, INPUT_R3, INPUT_R2));
	tc35876x_regw(i2c, LVMX0407,
		INPUT_MUX(INPUT_G2, INPUT_R7, INPUT_R1, INPUT_R6));
	tc35876x_regw(i2c, LVMX0811,
		INPUT_MUX(INPUT_G1, INPUT_G0, INPUT_G4, INPUT_G3));
	tc35876x_regw(i2c, LVMX1215,
		INPUT_MUX(INPUT_B2, INPUT_G7, INPUT_G6, INPUT_G5));
	tc35876x_regw(i2c, LVMX1619,
		INPUT_MUX(INPUT_B4, INPUT_B3, INPUT_B1, INPUT_B0));
	tc35876x_regw(i2c, LVMX2023,
		INPUT_MUX(LOGIC_0,  INPUT_B7, INPUT_B6, INPUT_B5));
	tc35876x_regw(i2c, LVMX2427,
		INPUT_MUX(INPUT_R0, INPUT_DE, INPUT_VSYNC, INPUT_HSYNC));

	/* Enable LVDS transmitter. */
	tc35876x_regw(i2c, LVCFG, BIT0);

	/* Clear notifications. Don't write reserved bits. Was write 0xffffffff
	 * to 0x0288, must be in error?! */
	tc35876x_regw(i2c, DSI_INTCLR, FLD_MASK(31, 30) | FLD_MASK(22, 0));

	lvds_disp_init = 1;
	printk(KERN_INFO "[DISPLAY]%s: Exit\n", __func__);

}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_toshiba_bridge_panel_off
 *
 * DESCRIPTION:  This function uses GPIO to turn OFF panel.
\* ************************************************************************* */
void dsi_lvds_toshiba_bridge_panel_off(void)
{
	printk(KERN_INFO "[DISPLAY ] %s\n", __func__);

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 0))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 0);
	usleep_range(1000, 2000);

	/* try to turn vadd off */
	vlcm_vadd_put();
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_toshiba_bridge_panel_on
 *
 * DESCRIPTION:  This function uses GPIO to turn ON panel.
 \* ************************************************************************* */
void dsi_lvds_toshiba_bridge_panel_on(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	printk(KERN_INFO "[DISPLAY ] %s\n", __func__);

	/* get vadd count, and make sure vadd is on */
	vlcm_vadd_get();

	if (cmi_lcd_i2c_client) {
		int ret;
		PSB_DEBUG_ENTRY("setting TCON\n");
		/* Bit 4 is average_saving. Setting it to 1, the brightness is
		 * referenced to the average of the frame content. 0 means
		 * reference to the maximum of frame contents. Bits 3:0 are
		 * allow_distort. When set to a nonzero value, all color values
		 * between 255-allow_distort*2 and 255 are mapped to the
		 * 255-allow_distort*2 value.
		 */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_ALLOW_DISTORT, 0x32);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_BYPASS_PWMI, 0);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
		/* Set minimum brightness value - this is tunable */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_PWM_MIN, 0x1C);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);

		/* changing CABC PWM frequency to 5 Khz */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_FREQ_DIVIDER_HI, 0xE0);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_FREQ_DIVIDER_LO, 0x64);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);

		/* PANEL_MODIFY_RGB to 0x00 to get rid of flicker */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_MODIFY_RGB, 0x00);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
		/* Enable PWMO generate by internal frequency */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_PWM_CONTROL, 0x01);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);

		/* Set maximum duty of PWMO by pwm_set */
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_PWM_REF, 0x00);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
	}

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 1);

	mdfld_dsi_brightness_control(dev_priv->dev, 0,
			dev_priv->brightness_adjusted);
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_init_lvds_bridge
 *
 * DESCRIPTION:  This function does all one time init. Currently defining
 *				  GPIOs only.
\* ************************************************************************* */
void dsi_lvds_init_lvds_bridge(struct drm_device *dev)
{
	int ret;
	ret = gpio_request(GPIO_MIPI_LCD_VADD, "display");
	if (ret)
		printk(KERN_ALERT "[DISPLAY]%s: GPIO request failed [%d]\n",
			__func__, GPIO_MIPI_LCD_VADD);

	ret = gpio_request(GPIO_MIPI_BRIDGE_RESET, "display");
	if (ret)
		printk(KERN_ALERT "[DISPLAY]%s: GPIO request failed [%d]\n",
				__func__, GPIO_MIPI_BRIDGE_RESET);

	ret = gpio_request(GPIO_MIPI_LCD_BL_EN, "display");
	if (ret)
		printk(KERN_ALERT "[DISPLAY]%s: GPIO request failed [%d]\n",
				__func__, GPIO_MIPI_LCD_BL_EN);
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_deinit_lvds_bridge
 *
 * DESCRIPTION:  This function does is called during deinit time.
 *
\* ************************************************************************* */
void dsi_lvds_deinit_lvds_bridge(struct drm_device *dev)
{
	if (!lvds_disp_init) {
		printk(KERN_ALERT "[DISPLAY ] %s has not initialized\n", __func__);
		return;
	}

	printk(KERN_INFO "[DISPLAY ] Enter %s\n", __func__);

#ifdef CONFIG_LVDS_HARD_RESET
	lvds_disp_init = 0;
#endif
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_bridge_get_display_params
 *
 * DESCRIPTION:  This function is a callback to get the panel resolution and
 *					timing settings.
 *
\* ************************************************************************* */
void dsi_lvds_bridge_get_display_params(struct drm_display_mode *mode)
{
	mode->hdisplay = 1280;
	mode->vdisplay = 800;
	mode->hsync_start = mode->hdisplay + 80;
	mode->hsync_end = mode->hsync_start + 40;
	mode->htotal = mode->hsync_end + 40;
	/* Original settings
	mode->vsync_start = mode->vdisplay + 8;
	mode->vsync_end = mode->vsync_start + 7;
	mode->vtotal = mode->vsync_end + 8;
	mode->clock = 16500;
	*/
	mode->vsync_start = mode->vdisplay + 14;
	mode->vsync_end = mode->vsync_start + 10;
	mode->vtotal = mode->vsync_end + 14;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->htotal * mode->vtotal / 1000;

	printk(KERN_INFO "[DISPLAY]: hdisplay(w) is %d\n", mode->hdisplay);
	printk(KERN_INFO "[DISPLAY]: vdisplay(h) is %d\n", mode->vdisplay);
	printk(KERN_INFO "[DISPLAY]: HSS is %d\n", mode->hsync_start);
	printk(KERN_INFO "[DISPLAY]: HSE is %d\n", mode->hsync_end);
	printk(KERN_INFO "[DISPLAY]: htotal is %d\n", mode->htotal);
	printk(KERN_INFO "[DISPLAY]: VSS is %d\n", mode->vsync_start);
	printk(KERN_INFO "[DISPLAY]: VSE is %d\n", mode->vsync_end);
	printk(KERN_INFO "[DISPLAY]: vtotal is %d\n", mode->vtotal);
	printk(KERN_INFO "[DISPLAY]: clock is %d\n", mode->clock);
}

/* ************************************************************************* *\
 * FUNCTION: __DSI_I2C_ByteRead
 *
 * DESCRIPTION:  Local functions to process read req for I2C registers
 *
\* ************************************************************************* */
static int __DSI_I2C_ByteRead(u16 reg, int count)
{
	char rxData[4] = {0};
	char regData[2] = {0};
	struct i2c_msg msgs[] = {
		{
		 .addr = dsi_lvds_inst->client->addr,
		 .flags = 0,
		 .len = 2,
		 },
		{
		 .addr = dsi_lvds_inst->client->addr,
		 .flags = I2C_M_RD,
		 .len = count - 2,
		 },
	};

	regData[0] = (reg & 0xFF00) >> 8;
	regData[1] = reg & 0xFF;

	msgs[0].buf = regData;
	msgs[1].buf = rxData;

	printk(KERN_INFO "Register: 0x%x\n", reg);
	if (i2c_transfer(dsi_lvds_inst->client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "[DISPLAY] %s: transfer error\n", __func__);
		return -EIO;
	} else if (count > 2) {
		int i = 0;
		for (i = count - 3; i > -1; --i)
			printk(KERN_INFO "%02x ", rxData[i]);
		printk(KERN_INFO "\n");
		return rxData[0];
	}
	return 0;
}

/* ************************************************************************* *\
 * FUNCTION: DSI_I2C_ByteRead
 *
 * DESCRIPTION:  Local functions to read I2C registers
 *
\* ************************************************************************* */
static int DSI_I2C_ByteRead(u16 reg, int count)
{
	if (dsi_lvds_inst->client)
		return __DSI_I2C_ByteRead(reg, count);
	else
		return -EIO;
}

/* ************************************************************************* *\
 * FUNCTION: __DSI_I2C_ByteWrite
 *
 * DESCRIPTION:  Local functions to process write req for I2C registers
 *
\* ************************************************************************* */
static int __DSI_I2C_ByteWrite(u16 reg, u32 data, int count)
{
	char txData[6] = {0};
	struct i2c_msg msg[] = {
		{
		 .addr = dsi_lvds_inst->client->addr,
		 .flags = 0,
		 .len = count,
		 },
	};

	/*Set the register */
	txData[0] = (reg & 0xFF00) >> 8;
	txData[1] = reg & 0xFF;

	if (count == 6) {
		/*Set the data */
		txData[2] = (data & 0xFF);
		txData[3] = (data & 0xFF00) >> 8;
		txData[4] = (data & 0xFF0000) >> 16;
		txData[5] = (data & 0xFF000000) >> 24;
	} else {
		/* Not valid for this bridge chipset */
	}

	printk(KERN_INFO "[DISPLAY] %s: addr = %x, reg = %x, data = %x\n",
		__func__, dsi_lvds_inst->client->addr, reg, data);

	msg[0].buf = txData;

	if (i2c_transfer(dsi_lvds_inst->client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "[DISPLAY] %s: transfer error\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}

/* ************************************************************************* *\
 * FUNCTION: DSI_I2C_ByteWrite
 *
 * DESCRIPTION:  Local functions to issue write req for I2C registers
 *
\* ************************************************************************* */
static int DSI_I2C_ByteWrite(u16 reg, u32 data, int count)
{
	if (dsi_lvds_inst->client)
		return __DSI_I2C_ByteWrite(reg, data, count);
	else
		return -EIO;
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_bridge_probe
 *
 * DESCRIPTION:  Probe function for LVDS bridge.
 *
\* ************************************************************************* */
static int dsi_lvds_bridge_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[DISPLAY] %s: Check I2C functionality failed.\n", __func__);
		return -ENODEV;
	}

	dsi_lvds_inst = kzalloc(sizeof(struct dsi_lvds_bridge_instance), GFP_KERNEL);

	if (dsi_lvds_inst == NULL) {
		printk(KERN_ERR "[DISPLAY] %s: Can not allocate memory.\n", __func__);
		return -ENOMEM;
	}

	dsi_lvds_inst->client = client;

	i2c_set_clientdata(client, dsi_lvds_inst);

	dsi_lvds_inst->client->addr = 0x0F;

	printk(KERN_INFO "[DISPLAY] %s: Exit\n", __func__);

	return 0;
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_bridge_remove
 *
 * DESCRIPTION:  Function to free driver instance.
 *
\* ************************************************************************* */
static int dsi_lvds_bridge_remove(struct i2c_client *client)
{
	printk(KERN_INFO "[DISPLAY] %s\n", __func__);

	dev_set_drvdata(&client->dev, 0);
	kfree(dsi_lvds_inst);
	return 0;
}

static const struct i2c_device_id dsi_lvds_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dsi_lvds_bridge_id);

static struct i2c_driver dsi_lvds_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = dsi_lvds_bridge_id,
	.probe    = dsi_lvds_bridge_probe,
	.remove   = dsi_lvds_bridge_remove,
};

static int dsi_lvds_dev_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{

	printk(KERN_INFO "[DISPLAY] %s: MIPI DSI driver IOCTL, cmd = %d.\n", __func__, cmd);

	return 0;
}

static const struct file_operations mipi_dsi_dev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dsi_lvds_dev_ioctl,
};

static struct miscdevice dsi_lvds_bridge_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mipi_dsi",
	.fops = &mipi_dsi_dev_fops,
};

/* LCD panel I2C */
static int cmi_lcd_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct cmi_lcd_data *lcd_data;

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	dev_info(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c_check_functionality() failed\n",
			__func__);
		return -ENODEV;
	}

	lcd_data = devm_kzalloc(&client->dev, sizeof(*lcd_data), GFP_KERNEL);
	if (!lcd_data) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&lcd_data->lock);

	i2c_set_clientdata(client, lcd_data);

	cmi_lcd_i2c_client = client;

	return 0;
}

static int cmi_lcd_i2c_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s\n", __func__);

	cmi_lcd_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id cmi_lcd_i2c_id[] = {
	{ "cmi-lcd", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cmi_lcd_i2c_id);

static struct i2c_driver cmi_lcd_i2c_driver = {
	.driver = {
		.name = "cmi-lcd",
	},
	.id_table = cmi_lcd_i2c_id,
	.probe = cmi_lcd_i2c_probe,
	.remove = __devexit_p(cmi_lcd_i2c_remove),
};


/* HACK to create I2C device while it's not created by platform code */
#define CMI_LCD_I2C_ADAPTER	2
#define CMI_LCD_I2C_ADDR	0x60

static int cmi_lcd_hack_create_device(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = "cmi-lcd",
		.addr = CMI_LCD_I2C_ADDR,
	};

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	adapter = i2c_get_adapter(CMI_LCD_I2C_ADAPTER);
	if (!adapter) {
		pr_err("%s: i2c_get_adapter(%d) failed\n", __func__,
			CMI_LCD_I2C_ADAPTER);
		return -EINVAL;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		pr_err("%s: i2c_new_device() failed\n", __func__);
		i2c_put_adapter(adapter);
		return -EINVAL;
	}

	return 0;
}

static void mdfld_dsi_lvds_brightness_init(void)
{
	int ret;
	u8 pwmctrl;

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	/* Make sure the PWM reference is the 19.2 MHz system clock. Read first
	 * instead of setting directly to catch potential conflicts between PWM
	 * users. */
	ret = intel_scu_ipc_ioread8(GPIOPWMCTRL, &pwmctrl);
	if (ret || pwmctrl != 0x01) {
		if (ret)
			pr_err("%s: GPIOPWMCTRL read failed\n", __func__);
		else
			PSB_DEBUG_ENTRY("GPIOPWMCTRL was not set to system"\
					"clock (pwmctrl = 0x%02x)\n", pwmctrl);

		ret = intel_scu_ipc_iowrite8(GPIOPWMCTRL, 0x01);
		if (ret)
			pr_err("%s: GPIOPWMCTRL set failed\n", __func__);
	}

	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
	if (!ret)
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

	if (ret)
		pr_err("%s: PWM0CLKDIV set failed\n", __func__);
	else
		PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_bridge_init
 *
 * DESCRIPTION:  Driver Init function for lvds bridge
 *
\* ************************************************************************* */
static int __init dsi_lvds_bridge_init(void)
{
	int ret = 0;

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	ret = cmi_lcd_hack_create_device();
	if (ret)
		pr_err("%s: cmi_lcd_hack_create_device() faild!\n", __func__);

	ret = i2c_add_driver(&cmi_lcd_i2c_driver);
	if (ret)
		pr_err("%s: add LCD I2C	driver faild!\n", __func__);

	ret = i2c_add_driver(&tc35876x_bridge_i2c_driver);

	printk(KERN_INFO "[DISPLAY] %s: Exit, ret = %d\n", __func__, ret);
#ifdef CONFIG_SUPPORT_HOST_PWM0
	/* Init the PWM0 of host, if the backlight brightness is controlled
	* by internel pwm of the panel, ignore this operation */
	mdfld_dsi_lvds_brightness_init();
#endif
	return 0;
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_bridge_exit
 *
 * DESCRIPTION:  Driver exit function for lvds bridge
 *
\* ************************************************************************* */
static void __exit dsi_lvds_bridge_exit(void)
{
	printk(KERN_INFO "[DISPLAY] %s\n", __func__);

	i2c_del_driver(&tc35876x_bridge_i2c_driver);

	if (cmi_lcd_i2c_client)
		i2c_del_driver(&cmi_lcd_i2c_driver);
}


module_init(dsi_lvds_bridge_init);
module_exit(dsi_lvds_bridge_exit);
#endif
