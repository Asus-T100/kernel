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
#include <asm/intel_scu_ipc.h>

#define CONFIG_LVDS_HARD_RESET
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE

/*GPIO Pins */
#define GPIO_MIPI_BRIDGE_RESET	115

#define GPIO_MIPI_LCD_BL_EN	112  /* DV1.0 GP_CORE_016 (+96 = GPIO number), 6S6P_BL_EN */
#define GPIO_MIPI_LCD_VADD	110
/* All these pins removed on DV1.0 */
#define GPIO_MIPI_LCD_BIAS_EN	-1
#define GPIO_MIPI_PANEL_RESET	-1
#define GPIO_MIPI_LCD_STBYB	-1
#define GPIO_MIPI_LCD_COLOR_EN	-1

struct i2c_client *cmi_lcd_i2c_client;

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
	msleep(10);

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
	msleep(20);

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
	msleep(20);

	if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);
	msleep(20);

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
		msleep(10);
	} else {

		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);  /*Pull MIPI Bridge reset pin to Low */
		msleep(20);
		if (gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1))
			gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);  /*Pull MIPI Bridge reset pin to High */
		msleep(40);
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
	if (lvds_disp_init) {
		printk(KERN_ALERT "[DISPLAY ] %s is already initialized\n", __func__);
		return;
	}

	printk(KERN_INFO "[DISPLAY ]%s: Enter\n", __func__);

	DSI_I2C_ByteWrite(0x013C, 0x00050006, 6);  /*PPI_TX_RX_TA, BTA parameters */
	DSI_I2C_ByteRead(0x013C, 4);
	DSI_I2C_ByteWrite(0x0114, 0x00000004, 6);  /*PPI_LPTXTIMCNT */
	DSI_I2C_ByteWrite(0x0164, 0x00000001, 6);  /*PPI_D0S_CLRSIPOCOUNT */
	DSI_I2C_ByteWrite(0x0168, 0x00000001, 6);  /*PPI_D1S_CLRSIPOCOUNT */
	DSI_I2C_ByteWrite(0x016c, 0x00000001, 6);  /*PPI_D2S_CLRSIPOCOUNT */
	DSI_I2C_ByteWrite(0x0170, 0x00000001, 6);  /*PPI_D3S_CLRSIPOCOUNT */
	/*Enabling MIPI & PPI lanes, Enable 4 lanes */
	DSI_I2C_ByteWrite(0x0134, 0x0000001F, 6);  /*PPI_LANEENABLE */
	DSI_I2C_ByteWrite(0x0210, 0x0000001F, 6);  /*DSI_LANEENABLE */
	DSI_I2C_ByteWrite(0x0104, 0x00000001, 6);  /*PPI_SARTPPI */
	DSI_I2C_ByteWrite(0x0204, 0x00000001, 6);  /*DSI_SARTPPI */

	/*Setting LVDS output frequency */
	DSI_I2C_ByteWrite(0x04A0, 0x00048006, 6);  /*LVDS PHY Register 0 (LVPHY0) */

	/*Calculating video panel control settings */
	/*Setting video panel control register */
	DSI_I2C_ByteWrite(0x0450, 0x00000120, 6);  /*VPCTRL, Video Path Control, VTGen=ON */

	/*Setting display timing registers */
	DSI_I2C_ByteWrite(0x0454, 0x00280028, 6);  /*HTIM1, HBPR=100, HPW=10 */

	/* value = ((hsync_start - hdisplay)<<16 | hdisplay) */
	DSI_I2C_ByteWrite(0x0458, ((1360 - 1280)<<16 | 1280), 6);  /*HTIM2, HFPR=190, HDISPR=1024 */
	DSI_I2C_ByteWrite(0x045c, 0x00080007, 6);  /*VTIM1, VBPR=8, VPW=7 */

	/* value = ((vsync_start - vdisplay)<<16 | vdisplay */
	DSI_I2C_ByteWrite(0x0460, ((808 - 8)<<16 | 800), 6);  /*VTIM2, VFPR=8, VDISPR=800 */
	DSI_I2C_ByteWrite(0x0464, 0x00000001, 6);  /*VFUEN */

	/*Setting LVDS bit arrangement */
	DSI_I2C_ByteWrite(0x0480, 0x05040302, 6);  /*LVMX0003 */
	DSI_I2C_ByteWrite(0x0484, 0x0A070106, 6);  /*LVMX0407 */
	DSI_I2C_ByteWrite(0x0488, 0x09080C0B, 6);  /*LVMX0811 */
	DSI_I2C_ByteWrite(0x048C, 0x120F0E0D, 6);  /*LVMX1215 */
	DSI_I2C_ByteWrite(0x0490, 0x14131110, 6);  /*LVMX1619 */
	DSI_I2C_ByteWrite(0x0494, 0x1B171615, 6);  /*LVMX2023 */
	DSI_I2C_ByteWrite(0x0498, 0x001A1918, 6);  /*LVMX2427 */
	DSI_I2C_ByteWrite(0x049c, 0x00000001, 6); /*LVCFG */


	DSI_I2C_ByteWrite(0x0288, 0xFFFFFFFF, 6); /*DSI_INTCLR */

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
	mdelay(1);

	if (gpio_direction_output(GPIO_MIPI_LCD_VADD, 0))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_VADD, 0);
}

/* ************************************************************************* *\
 * FUNCTION: dsi_lvds_toshiba_bridge_panel_on
 *
 * DESCRIPTION:  This function uses GPIO to turn ON panel.
 \* ************************************************************************* */
void dsi_lvds_toshiba_bridge_panel_on(void)
{
	printk(KERN_INFO "[DISPLAY ] %s\n", __func__);

	if (gpio_direction_output(GPIO_MIPI_LCD_VADD, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_VADD, 1);
	msleep(260);

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
				PANEL_PWM_MIN, 0x35);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);

		/*changing CABC PWM frequency to 5 Khz */
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

		/*PANEL_MODIFY_RGB to 0x00 to get rid of flicker*/
		ret = i2c_smbus_write_byte_data(cmi_lcd_i2c_client,
				PANEL_MODIFY_RGB, 0x00);
		if (ret < 0)
			dev_err(&cmi_lcd_i2c_client->dev,
					"i2c write failed (%d)\n", ret);
	}

	if (gpio_direction_output(GPIO_MIPI_LCD_BL_EN, 1))
		gpio_set_value_cansleep(GPIO_MIPI_LCD_BL_EN, 1);
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
	mode->clock = 33324;
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

	ret = i2c_add_driver(&dsi_lvds_bridge_i2c_driver);

	printk(KERN_INFO "[DISPLAY] %s: Exit, ret = %d\n", __func__, ret);

	mdfld_dsi_lvds_brightness_init();
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

	misc_deregister(&dsi_lvds_bridge_dev);

	i2c_del_driver(&dsi_lvds_bridge_i2c_driver);
}


module_init(dsi_lvds_bridge_init);
module_exit(dsi_lvds_bridge_exit);

#endif
