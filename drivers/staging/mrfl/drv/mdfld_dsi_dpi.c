/*
 * Copyright © 2010 Intel Corporation
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
 * Authors:
 * jim liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include "psb_drv.h"

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY

/*
 * GPIO Pins
 * GP_AON_34 is for test on iCDK.
 * KBD_MKIN0(GP_AON_34), DISP1_RST_N
 */
/*#define GPIO_MIPI_BRIDGE_RESET 34*/

/*GP_CORE_019 (+96 = GPIO number)*/
#define GPIO_MIPI_BRIDGE_RESET 115
/*GP_CORE_032, DISP0_RST_N (+96 = GPIO number)*/
#define GPIO_MIPI_PANEL_RESET 128

#define SPI_CS0 0x8
#define SPI_CLK 0x4
#define SPI_DO  0x2

static int gGpioOutput;
int disp_init;
bool dsi_device_ready;

#define IOCTL_LCM_POWER_OFF 0
#define IOCTL_LCM_POWER_ON  1

struct drm_encoder *gencoder;

struct tc358762_info {
	struct i2c_client *client;
	struct work_struct wqueue;
	struct work_struct pre_init_work;
	struct work_struct test_work;
} *tc358762;

static void mdfld_dsi_dpi_shut_down(struct mdfld_dsi_dpi_output *output,
				    int pipe);

static int DSI_I2C_ByteRead(u16 reg, int count);
static int DSI_I2C_ByteWrite(u16 reg, u32 data, int count);

static void mdfld_wait_for_LP_DATA_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);
	if (pipe == 2)
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(GEN_FIFO_STAT_REG) &
				     LP_DATA_FIFO_FULL)) {
		udelay(10);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: LP CMD FIFO was never cleared!\n");
}

static void mdfld_wait_for_LP_CTRL_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = GEN_FIFO_STAT_REG;
	int timeout = 0;
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	udelay(500);

	if (pipe == 2)
		gen_fifo_stat_reg = GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) && (REG_READ(GEN_FIFO_STAT_REG) &
				     LP_CTRL_FIFO_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: LP CMD FIFO was never cleared!\n");
}
#endif

static void mdfld_wait_for_HS_DATA_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
	int timeout = 0;

#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	if (pipe == 2)
		gen_fifo_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
	       (REG_READ(gen_fifo_stat_reg) & DSI_FIFO_GEN_HS_DATA_FULL)) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: HS Data FIFO was never cleared!\n");
}

static void mdfld_wait_for_HS_CTRL_FIFO(struct drm_device *dev, u32 pipe)
{
	u32 gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	if (pipe == 2)
		gen_fifo_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
	       (REG_READ(gen_fifo_stat_reg) & DSI_FIFO_GEN_HS_CTRL_FULL)) {
		udelay(100);
		timeout++;
	}
	if (timeout == 20000)
		DRM_INFO("MIPI: HS CMD FIFO was never cleared!\n");
}

static void mdfld_wait_for_SPL_PKG_SENT(struct drm_device *dev, u32 pipe)
{
	u32 intr_stat_reg = MIPIA_INTR_STAT_REG;
	int timeout = 0;
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	if (pipe == 2)
		intr_stat_reg += MIPIC_REG_OFFSET;

	udelay(500);

	/* This will time out after approximately 2+ seconds */
	while ((timeout < 20000) &&
	       (!(REG_READ(intr_stat_reg) & DSI_INTR_STATE_SPL_PKG_SENT))) {
		udelay(100);
		timeout++;
	}

	if (timeout == 20000)
		DRM_INFO("MIPI: SPL_PKT was not sent successfully!\n");
}

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
void dsi_set_bridge_reset_state(int state)
{
	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s: state = %d\n", __func__, state);

#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	if (state) {
		gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0);
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
		mdelay(10);
	} else {
		gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 0);
		/*Pull MIPI Bridge reset pin to Low */
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 0);
		mdelay(20);
		gpio_direction_output(GPIO_MIPI_BRIDGE_RESET, 1);
		/*Pull MIPI Bridge reset pin to High */
		gpio_set_value_cansleep(GPIO_MIPI_BRIDGE_RESET, 1);
		mdelay(40);
	}
}

void dsi_set_device_ready_state(struct drm_device *dev, int state, int pipe)
{
	u32 reg_offset = pipe ? MIPIC_REG_OFFSET : 0;

	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s: state = %d, pipe = %d\n",
			__func__, state, pipe);

	if (state)
		REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000001);
	else
		REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000000);
}

void dsi_send_turn_on_packet(struct drm_device *dev)
{
	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

	REG_WRITE(DPI_CONTROL_REG, DPI_TURN_ON);

	/* Short delay to wait that display turns on */
	mdelay(10);
}

void dsi_send_shutdown_packet(struct drm_device *dev)
{
	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

	REG_WRITE(DPI_CONTROL_REG, DPI_SHUT_DOWN);
}

void dsi_set_ptarget_state(struct drm_device *dev, int state)
{
	u32 pp_sts_reg;

	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s: state = %d\n", __func__, state);
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	if (state) {
		REG_WRITE(PP_CONTROL, (REG_READ(PP_CONTROL) | POWER_TARGET_ON));
		do {
			pp_sts_reg = REG_READ(PP_STATUS);
		} while ((pp_sts_reg & (PP_ON | PP_READY)) == PP_READY);
	} else {
		REG_WRITE(PP_CONTROL,
			  (REG_READ(PP_CONTROL) & ~POWER_TARGET_ON));
		do {
			pp_sts_reg = REG_READ(PP_STATUS);
		} while (pp_sts_reg & PP_ON);
	}
}

void dsi_set_pipe_plane_enable_state(struct drm_device *dev,
				     int state, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	u32 temp_reg;
	u32 pipeconf_reg = PIPEACONF;
	u32 dspcntr_reg = DSPACNTR;
	u32 mipi_reg = MIPI;
	u32 reg_offset = 0;

	u32 pipeconf = dev_priv->pipeconf;
	u32 dspcntr = dev_priv->dspcntr;
	u32 mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE | SEL_FLOPPED_HSTX;

	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s: state = %d, pipe = %d\n",
			__func__, state, pipe);

	if (pipe) {
		pipeconf_reg = PIPECCONF;
		dspcntr_reg = DSPCCNTR;
		mipi_reg = MIPI_C;
		reg_offset = MIPIC_REG_OFFSET;
	} else {
		mipi |= 2;
	}

	if (state) {
		/*Enable MIPI Port */
		/*REG_WRITE(mipi_reg, mipi); */
		/*REG_READ(mipi_reg); */

		/*Set up pipe */
		REG_WRITE(pipeconf_reg, pipeconf);
		/*REG_READ(pipeconf_reg); */

		/*Set up display plane */
		REG_WRITE(dspcntr_reg, dspcntr);
		/*REG_READ(dspcntr_reg); */
	} else {
		/*Disable PIPE */
		REG_WRITE(pipeconf_reg, 0);
		mdfld_wait_for_PIPEA_DISABLE(dev, pipe);
		mdfld_wait_for_DPI_CTRL_FIFO(dev, pipe);

		/*Disable MIPI Port */
		/*REG_WRITE(mipi_reg, (REG_READ(mipi_reg) & ~BIT31)); */
		/*REG_READ(mipi_reg); */
	}
}

static void toshiba_spi_write_byte(char dc, u8 data)
{
	u32 bit;
	int bnum;
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	/* gpio_set_value_cansleep(spi_sclk, 0); // clk low */
	gGpioOutput &= (~SPI_CLK);
	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);

	/* dc: 0 for command, 1 for parameter */
	/* gpio_set_value_cansleep(spi_mosi, dc); */
	if (dc == 0) {
		gGpioOutput &= (~SPI_DO);
	} else {
		gGpioOutput |= SPI_DO;
	}
	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
	udelay(1);		/* at least 20 ns */

	/* gpio_set_value_cansleep(spi_sclk, 1); // clk high */
	gGpioOutput |= SPI_CLK;
	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
	udelay(1);		/* at least 20 ns */

	bnum = 8;		/* 8 data bits */
	bit = 0x80;
	while (bnum) {
		/* gpio_set_value_cansleep(spi_sclk, 0); // clk low */
		gGpioOutput &= (~SPI_CLK);
		DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);

		if (data & bit) {
			/* gpio_set_value_cansleep(spi_mosi, 1); */
			gGpioOutput |= SPI_DO;
		} else {
			/* gpio_set_value_cansleep(spi_mosi, 0); */
			gGpioOutput &= (~SPI_DO);
		}
		DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
		udelay(1);	/* at least 20 ns */

		/* gpio_set_value_cansleep(spi_sclk, 1); // clk high */
		gGpioOutput |= SPI_CLK;
		DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
		udelay(1);	/* at least 20 ns */

		bit >>= 1;
		bnum--;
	}
}

static void toshiba_spi_write(char cmd, u32 data, int num)
{
	char *bp;
	/* gpio_set_value_cansleep(spi_cs, 1);   // cs high */
	gGpioOutput |= SPI_CS0;
	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	/* command byte first */
	toshiba_spi_write_byte(0, cmd);

	/* followed by parameter bytes */
	if (num) {
		bp = (char *)&data;
		bp += (num - 1);
		while (num) {
			toshiba_spi_write_byte(1, *bp);
			num--;
			bp--;
		}
	}
	/* gpio_set_value_cansleep(spi_cs, 0);   // cs low */
	gGpioOutput &= (~SPI_CS0);
	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
	udelay(1);
}

void toshiba_bridge_spi_panel_off(void)
{
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s\n", __func__);
	toshiba_spi_write(0x28, 0, 0);	/* display off */
	mdelay(1);
	toshiba_spi_write(0xb8, 0x8002, 2);	/* output control */
	mdelay(1);
	toshiba_spi_write(0x10, 0x00, 1);	/* sleep mode in */
	mdelay(85);
	toshiba_spi_write(0xb0, 0x00, 1);	/* deep standby in */
	mdelay(1);

	gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
	gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);
	mdelay(1);
}

void toshiba_bridge_spi_panel_init(struct drm_device *dev)
{
	int value = 100;
	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s\n", __func__);
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	DSI_I2C_ByteWrite(0x0480, 0x0000000F, 6);

	gGpioOutput = 0;
	gGpioOutput |= SPI_CLK;

	DSI_I2C_ByteWrite(0x0484, gGpioOutput, 6);
	udelay(1);		/* at least 20 ns */

	gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
	gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);
	value = gpio_get_value(GPIO_MIPI_PANEL_RESET);
	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s - Panel reset is %d\n", __func__,
			value);
	mdelay(5);		/* 10 */
	gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
	gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 1);
	value = gpio_get_value(GPIO_MIPI_PANEL_RESET);
	PSB_DEBUG_ENTRY("[DISPLAY TRK] %s - Panel reset is %d\n", __func__,
			value);

	mdelay(2);
	toshiba_spi_write(0, 0, 0);
	mdelay(6);
	toshiba_spi_write(0, 0, 0);
	mdelay(6);
	toshiba_spi_write(0, 0, 0);
	mdelay(6);
	toshiba_spi_write(0xba, 0x11, 1);	/* 1011 1010 0001 0001 */
	toshiba_spi_write(0x36, 0x00, 1);
	toshiba_spi_write(0x3a, 0x60, 1);
	toshiba_spi_write(0xb1, 0x5d, 1);
	toshiba_spi_write(0xb2, 0x33, 1);
	toshiba_spi_write(0xb3, 0x22, 1);
	toshiba_spi_write(0xb4, 0x02, 1);
	toshiba_spi_write(0xb5, 0x1e, 1);	/* vcs -- adjust brightness */
	toshiba_spi_write(0xb6, 0x27, 1);
	toshiba_spi_write(0xb7, 0x03, 1);	/* DPL=0: Reads data at the falling edge of PCLK */
	toshiba_spi_write(0xb9, 0x24, 1);
	toshiba_spi_write(0xbd, 0xa1, 1);
	toshiba_spi_write(0xbb, 0x00, 1);
	toshiba_spi_write(0xbf, 0x01, 1);
	toshiba_spi_write(0xbe, 0x00, 1);
	toshiba_spi_write(0xc0, 0x11, 1);
	toshiba_spi_write(0xc1, 0x11, 1);
	toshiba_spi_write(0xc2, 0x11, 1);
	toshiba_spi_write(0xc3, 0x3232, 2);
	toshiba_spi_write(0xc4, 0x3232, 2);
	toshiba_spi_write(0xc5, 0x3232, 2);
	toshiba_spi_write(0xc6, 0x3232, 2);
	toshiba_spi_write(0xc7, 0x6445, 2);
	toshiba_spi_write(0xc8, 0x44, 1);
	toshiba_spi_write(0xc9, 0x52, 1);
	toshiba_spi_write(0xca, 0x00, 1);
	toshiba_spi_write(0xec, 0x0200, 2);
	toshiba_spi_write(0xcf, 0x01, 1);
	toshiba_spi_write(0xd0, 0x1004, 2);
	toshiba_spi_write(0xd1, 0x01, 1);
	toshiba_spi_write(0xd2, 0x001a, 2);
	toshiba_spi_write(0xd3, 0x001a, 2);
	toshiba_spi_write(0xd4, 0x207a, 2);
	toshiba_spi_write(0xd5, 0x18, 1);

	toshiba_spi_write(0xe2, 0x18, 1);
	toshiba_spi_write(0xe3, 0, 1);
	toshiba_spi_write(0xe4, 0x0003, 2);
	toshiba_spi_write(0xe5, 0x0003, 2);
	toshiba_spi_write(0xe6, 0x04, 1);
	toshiba_spi_write(0xe7, 0x030c, 2);
	toshiba_spi_write(0xe8, 0x03, 1);
	toshiba_spi_write(0xe9, 0x20, 1);
	toshiba_spi_write(0xea, 0x0404, 2);

	toshiba_spi_write(0xef, 0x3200, 2);
/*      mdelay(32); */
	toshiba_spi_write(0xbc, 0x80, 1);	/* wvga pass through */
	toshiba_spi_write(0x3b, 0x00, 1);

	toshiba_spi_write(0xb9, 0x24, 1);

	toshiba_spi_write(0xb0, 0x16, 1);
	toshiba_spi_write(0xb8, 0xfff5, 2);
	toshiba_spi_write(0x11, 0, 0);
	toshiba_spi_write(0x29, 0, 0);
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_init_TOSHIBA_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
void mdfld_init_TOSHIBA_MIPI(struct drm_device *dev)
{
	u32 gen_data[2];
	u16 wc = 0;
	u8 vc = 0;
	u32 gen_data_intel = 0x200105;

	if (disp_init) {
		PSB_DEBUG_ENTRY("[DISPLAY TRK] %s has initialized\n", __func__);
		return;
	}

	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	/*DSI Basic Parameter */
	/*SYSPMCTRL, Normal Operation */
	DSI_I2C_ByteWrite(0x047C, 0x00000000, 6);
	mdelay(20);

	/*LANEENABLE, Enable 2 lanes and clock lane */
	DSI_I2C_ByteWrite(0x0210, 0x00000007, 6);
	/*D0S_CLRSIPOCOUNT */
	DSI_I2C_ByteWrite(0x0164, 0x00000003, 6);
	/*D1S_CLRSIPOCOUNT */
	DSI_I2C_ByteWrite(0x0168, 0x00000003, 6);
	/*D0S_ATMR, Analog timer setup for lane 0 */
	DSI_I2C_ByteWrite(0x0144, 0x00000000, 6);
	/*D1S_ATMR, Analog timer setup for lane 1 */
	DSI_I2C_ByteWrite(0x0148, 0x00000000, 6);
	 /*LPTXTIMCNT*/ DSI_I2C_ByteWrite(0x0114, 0x00000004, 6);

	/*
	 * SPI/DBI-C Master Setting
	 */
	/*SPICMR/SPICTRL, POL=1, PHA=0, SPI Mode Enable */
	DSI_I2C_ByteWrite(0x0450, 0x00000040, 6);
	/*SPITCR2/SPITCR1, PRS=1 cg_spi_clk/8 */
	DSI_I2C_ByteWrite(0x0454, 0x00000122, 6);

	/*
	 * LCDC Setting
	 * PORT/LCDCTRL, RGB888 24-bit color, Non-Burst Mode,
	 * DCLK_POL=0(Non_invert), VSYNC_POL=0(Low), DE_POL=0(High),
	 * HSYNC_POL=0(Low)
	 */

	/*RGB888=x150, RBG666=x110 */
	DSI_I2C_ByteWrite(0x0420, 0x00000150, 6);

	/*Below setting is for VTGen On */
/*
	DSI_I2C_ByteWrite(0x0420, 0x00000152, 6);

	//HSR, HSync Pulse Width = 8
	//HBPR, HSync Back Porch Width = 8
	DSI_I2C_ByteWrite(0x0424, 0x00080008, 6);

	//HDISPR, Horizontal Display Size = 480
	//HRPR, Horizontal Front Porch Size = 16
	DSI_I2C_ByteWrite(0x0428, 0x001001E0, 6);

	//VSR, VSync Pulse Width = 2
	//VBPR, VSync Back Porch Width = 2
	DSI_I2C_ByteWrite(0x042C, 0x00020002, 6);

	//VDISPR, Vertical Display Size = 800
	//VFPR, Vertical Front Porch Width = 4
	DSI_I2C_ByteWrite(0x0430, 0x00040320, 6);

	DSI_I2C_ByteWrite(0x0434, 0x00000001, 6);  //VFUEN, Used on Burst Mode
*/

	 /*SYSCTRL*/ DSI_I2C_ByteWrite(0x0464, 0x0000020A, 6);

	/*
	 * DSI Start
	 */

	/*STARTPPI, Start PPI */
	DSI_I2C_ByteWrite(0x0104, 0x00000001, 6);
	/*STARTDSI, Start DSI */
	DSI_I2C_ByteWrite(0x0204, 0x00000001, 6);

	/*
	 * PLL Frequency Change
	 */

	/*SYSPLL3, Set PCLK to 28.8 MHz */
	DSI_I2C_ByteWrite(0x0470, 0x48260000, 6);
	mdelay(5);

	/*Check INTSTATUS register */
	DSI_I2C_ByteRead(0x0220, 6);

	/*Init Panel */
	toshiba_bridge_spi_panel_init(0);

#ifdef MIPI_DEBUG_LOG
	/*Print value of Toshiba TC358762 MIPI bridge registers */
	DSI_I2C_ByteRead(0x047C, 6);
	DSI_I2C_ByteRead(0x0210, 6);
	DSI_I2C_ByteRead(0x0164, 6);
	DSI_I2C_ByteRead(0x0168, 6);
	DSI_I2C_ByteRead(0x0144, 6);
	DSI_I2C_ByteRead(0x0148, 6);
	DSI_I2C_ByteRead(0x0114, 6);
	DSI_I2C_ByteRead(0x0450, 6);
	DSI_I2C_ByteRead(0x0454, 6);
	DSI_I2C_ByteRead(0x0420, 6);
	DSI_I2C_ByteRead(0x0464, 6);
	DSI_I2C_ByteRead(0x0204, 6);
	DSI_I2C_ByteRead(0x0470, 6);
#endif

	disp_init = 1;
}

void mdfld_deinit_TOSHIBA_MIPI(struct drm_device *dev)
{
	if (!disp_init) {
		PSB_DEBUG_ENTRY("[DISPLAY TRK] %s has not initialized\n",
				__func__);
		return;
	}
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

	/*De-init Panel */
	toshiba_bridge_spi_panel_off();

	/*SYSPMCTRL, Sleep Mode */
	DSI_I2C_ByteWrite(0x047C, 0x00000080, 6);

	disp_init = 0;
}

static void mdfld_dsi_configure_down(struct mdfld_dsi_encoder *dsi_encoder,
				     int pipe)
{
	struct mdfld_dsi_dpi_output *dpi_output =
	    MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

	if (!dev_priv->dpi_panel_on) {
		PSB_DEBUG_ENTRY("[DISPLAY] %s: DPI Panel is Already Off\n",
				__func__);
		return;
	}
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	mdfld_deinit_TOSHIBA_MIPI(dev);	/* De-init MIPI bridge and Panel */
	dsi_set_bridge_reset_state(1);	/* Pull Low Reset */

	dsi_set_pipe_plane_enable_state(dev, 0, pipe);	/* Disable pipe and plane */

/*      dsi_set_ptarget_state(dev, 0);  //Disable PTARGET */

	mdfld_dsi_dpi_shut_down(dpi_output, pipe);	/* Send shut down command */

	dsi_set_device_ready_state(dev, 0, pipe);	/* Clear device ready state */

	dev_priv->dpi_panel_on = false;
}

static void mdfld_dsi_configure_up(struct mdfld_dsi_encoder *dsi_encoder,
				   int pipe)
{
	struct mdfld_dsi_dpi_output *dpi_output =
	    MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("[DISPLAY TRK] Enter %s\n", __func__);

	if (dev_priv->dpi_panel_on) {
		PSB_DEBUG_ENTRY("[DISPLAY] %s: DPI Panel is Already On\n",
				__func__);
		return;
	}
#if 1 /* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	/* For resume path sequence */
/*      dsi_set_pipe_plane_enable_state(dev, 0, pipe); */
/*      dsi_set_ptarget_state(dev, 0); */
	mdfld_dsi_dpi_shut_down(dpi_output, pipe);
	dsi_set_device_ready_state(dev, 0, pipe);	/* Clear Device Ready Bit */

	dsi_set_device_ready_state(dev, 1, pipe);	/* Set device ready state */
	dsi_set_bridge_reset_state(0);	/* Pull High Reset */
	mdfld_init_TOSHIBA_MIPI(dev);	/* Init MIPI Bridge and Panel */
	mdfld_dsi_dpi_turn_on(dpi_output, pipe);	/* Send turn on command */
/*      dsi_set_ptarget_state(dev, 1);  //Enable PTARGET */
	dsi_set_pipe_plane_enable_state(dev, 1, pipe);	/* Enable plane and pipe */

	dev_priv->dpi_panel_on = true;
}

#endif

/* ************************************************************************* *\
 * FUNCTION: mdfld_dsi_tpo_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
void mdfld_dsi_tpo_ic_init(struct mdfld_dsi_config *dsi_config, u32 pipe)
{
	struct drm_device *dev = dsi_config->dev;
	u32 dcsChannelNumber = dsi_config->channel_num;
	u32 gen_data_reg = MIPIA_HS_GEN_DATA_REG;
	u32 gen_ctrl_reg = MIPIA_HS_GEN_CTRL_REG;
	u32 gen_ctrl_val = GEN_LONG_WRITE;

	DRM_INFO("Enter mrst init TPO MIPI display.\n");
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */

	if (pipe == 2) {
		gen_data_reg = HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		gen_ctrl_reg = HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
	}

	gen_ctrl_val |= dcsChannelNumber << DCS_CHANNEL_NUMBER_POS;

	/* Flip page order */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00008036);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x02 << WORD_COUNTS_POS));

	/* 0xF0 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5af0);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* Write protection key */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5af1);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xFC */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x005a5afc);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xB7 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x770000b7);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000044);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x05 << WORD_COUNTS_POS));

	/* 0xB6 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000a0ab6);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));

	/* 0xF2 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x081010f2);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x4a070708);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000000c5);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xF8 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x024003f8);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x01030a04);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x0e020220);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000004);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x0d << WORD_COUNTS_POS));

	/* 0xE2 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x398fc3e2);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x0000916f);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x06 << WORD_COUNTS_POS));

	/* 0xB0 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000000b0);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x02 << WORD_COUNTS_POS));

	/* 0xF4 */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x240242f4);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x78ee2002);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2a071050);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x507fee10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x10300710);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x14 << WORD_COUNTS_POS));

	/* 0xBA */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x19fe07ba);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x101c0a31);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000010);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xBB */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x28ff07bb);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x24280a31);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000034);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x09 << WORD_COUNTS_POS));

	/* 0xFB */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535d05fb);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1b1a2130);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x221e180e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x131d2120);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535d0508);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1c1a2131);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x231f160d);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x111b2220);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x535c2008);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1f1d2433);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2c251a10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2c34372d);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000023);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x31 << WORD_COUNTS_POS));

	/* 0xFA */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x525c0bfa);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1c1c232f);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x2623190e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x18212625);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x545d0d0e);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1e1d2333);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x26231a10);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x1a222725);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x545d280f);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x21202635);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x31292013);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x31393d33);
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x00000029);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x31 << WORD_COUNTS_POS));

	/* Set DM */
	mdfld_wait_for_HS_DATA_FIFO(dev, pipe);
	REG_WRITE(gen_data_reg, 0x000100f7);
	mdfld_wait_for_HS_CTRL_FIFO(dev, pipe);
	REG_WRITE(gen_ctrl_reg, gen_ctrl_val | (0x03 << WORD_COUNTS_POS));
}

static u16 mdfld_dsi_dpi_to_byte_clock_count(int pixel_clock_count,
					     int num_lane, int bpp)
{
	return (u16) ((pixel_clock_count * bpp) / (num_lane * 8));
}

/*
 * Calculate the dpi time basing on a given drm mode @mode
 * return 0 on success.
 * FIXME: I was using proposed mode value for calculation, may need to
 * use crtc mode values later
 */
int mdfld_dsi_dpi_timing_calculation(struct drm_display_mode *mode,
				     struct mdfld_dsi_dpi_timing *dpi_timing,
				     int num_lane, int bpp)
{
	int pclk_hsync, pclk_hfp, pclk_hbp, pclk_hactive;
	int pclk_vsync, pclk_vfp, pclk_vbp, pclk_vactive;

	if (!mode || !dpi_timing) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY
	    ("pclk %d, hdisplay %d, hsync_start %d, hsync_end %d, htotal %d\n",
	     mode->clock, mode->hdisplay, mode->hsync_start, mode->hsync_end,
	     mode->htotal);
	PSB_DEBUG_ENTRY
	    ("vdisplay %d, vsync_start %d, vsync_end %d, vtotal %d\n",
	     mode->vdisplay, mode->vsync_start, mode->vsync_end, mode->vtotal);

	pclk_hactive = mode->hdisplay;
	pclk_hfp = mode->hsync_start - mode->hdisplay;
	pclk_hsync = mode->hsync_end - mode->hsync_start;
	pclk_hbp = mode->htotal - mode->hsync_end;

	pclk_vactive = mode->vdisplay;
	pclk_vfp = mode->vsync_start - mode->vdisplay;
	pclk_vsync = mode->vsync_end - mode->vsync_start;
	pclk_vbp = mode->vtotal - mode->vsync_end;

#ifdef MIPI_DEBUG_LOG
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hactive = %d\n", __func__,
			pclk_hactive);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hfp = %d\n", __func__, pclk_hfp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hsync = %d\n", __func__,
			pclk_hsync);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_hbp = %d\n", __func__, pclk_hbp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vactive = %d\n", __func__,
			pclk_vactive);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vfp = %d\n", __func__, pclk_vfp);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vsync = %d\n", __func__,
			pclk_vsync);
	PSB_DEBUG_ENTRY("[DISPLAY] %s: pclk_vbp = %d\n", __func__, pclk_vbp);
#endif
	/*
	 * byte clock counts were calculated by following formula
	 * bclock_count = pclk_count * bpp / num_lane / 8
	 */
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	dpi_timing->hsync_count = 10;
	dpi_timing->hbp_count = 22;
	dpi_timing->hfp_count = 265;
	dpi_timing->hactive_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_hactive, num_lane, bpp);
	dpi_timing->vsync_count = 2;
	dpi_timing->vbp_count = 2;
	dpi_timing->vfp_count = 4;
#else
	dpi_timing->hsync_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_hsync, num_lane, bpp);
	dpi_timing->hbp_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_hbp, num_lane, bpp);
	dpi_timing->hfp_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_hfp, num_lane, bpp);
	dpi_timing->hactive_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_hactive, num_lane, bpp);
	dpi_timing->vsync_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_vsync, num_lane, bpp);
	dpi_timing->vbp_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_vbp, num_lane, bpp);
	dpi_timing->vfp_count =
	    mdfld_dsi_dpi_to_byte_clock_count(pclk_vfp, num_lane, bpp);
#endif
	PSB_DEBUG_ENTRY("DPI timings: %d, %d, %d, %d, %d, %d, %d\n",
			dpi_timing->hsync_count, dpi_timing->hbp_count,
			dpi_timing->hfp_count, dpi_timing->hactive_count,
			dpi_timing->vsync_count, dpi_timing->vbp_count,
			dpi_timing->vfp_count);

	return 0;
}

void mdfld_dsi_dpi_controller_init(struct mdfld_dsi_config *dsi_config,
				   int pipe)
{
	struct drm_device *dev = dsi_config->dev;
	u32 reg_offset = pipe ? MIPIC_REG_OFFSET : 0;
	int lane_count = dsi_config->lane_count;
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct drm_display_mode *mode = dsi_config->mode;
	u32 val = 0;

	PSB_DEBUG_ENTRY("Init DPI interface on pipe %d...\n", pipe);

	/*un-ready device */
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000000);

	/*init dsi adapter before kicking off */
	REG_WRITE((MIPIA_CONTROL_REG + reg_offset), 0x00000018);

	/*enable all interrupts */
	REG_WRITE((MIPIA_INTR_EN_REG + reg_offset), 0xffffffff);

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/*set up func_prg */
	val |= lane_count;
	val |= dsi_config->channel_num << DSI_DPI_VIRT_CHANNEL_OFFSET;

	switch (dsi_config->bpp) {
	case 16:
		val |= DSI_DPI_COLOR_FORMAT_RGB565;
		break;
	case 18:
		val |= DSI_DPI_COLOR_FORMAT_RGB666;
		break;
	case 24:
		val |= DSI_DPI_COLOR_FORMAT_RGB888;
		break;
	default:
		DRM_ERROR("unsupported color format, bpp = %d\n",
			  dsi_config->bpp);
	}
	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);

	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset), 0x90000);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset), 0xffff);

	/*max value: 20 clock cycles of txclkesc */
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset), 0xa);

	/*min 21 txclkesc, max: ffffh */
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset), 0xff);

	REG_WRITE((MIPIA_DPI_RESOLUTION_REG + reg_offset), 0x32001e0);

	/*set DPI timing registers */
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					 dsi_config->lane_count,
					 dsi_config->bpp);

	REG_WRITE((MIPIA_HSYNC_COUNT_REG + reg_offset),
		  dpi_timing.hsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HBP_COUNT_REG + reg_offset),
		  dpi_timing.hbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HFP_COUNT_REG + reg_offset),
		  dpi_timing.hfp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HACTIVE_COUNT_REG + reg_offset),
		  dpi_timing.hactive_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VSYNC_COUNT_REG + reg_offset),
		  dpi_timing.vsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VBP_COUNT_REG + reg_offset),
		  dpi_timing.vbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VFP_COUNT_REG + reg_offset),
		  dpi_timing.vfp_count & DSI_DPI_TIMING_MASK);

	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);

	/*min: 7d0 max: 4e20 */
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0xfff);

	/*set up video mode */
	val = 0;
	val = dsi_config->video_mode;	/* | DSI_DPI_COMPLETE_LAST_LINE; */
	REG_WRITE((MIPIA_VIDEO_MODE_FORMAT_REG + reg_offset), val);

	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);

	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);

	/*TODO: figure out how to setup these registers */
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3808);

	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset),
		  (0xa << 16) | 0x14);
#else
	/*set up func_prg */
	val |= lane_count;
	val |= dsi_config->channel_num << DSI_DPI_VIRT_CHANNEL_OFFSET;

	switch (dsi_config->bpp) {
	case 16:
		val |= DSI_DPI_COLOR_FORMAT_RGB565;
		break;
	case 18:
		val |= DSI_DPI_COLOR_FORMAT_RGB666;
		break;
	case 24:
		val |= DSI_DPI_COLOR_FORMAT_RGB888;
		break;
	default:
		DRM_ERROR("unsupported color format, bpp = %d\n",
			  dsi_config->bpp);
	}
	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);

	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset),
		  (mode->vtotal * mode->htotal * dsi_config->bpp /
		   (8 * lane_count)) & DSI_HS_TX_TIMEOUT_MASK);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset),
		  0xffff & DSI_LP_RX_TIMEOUT_MASK);

	/*max value: 20 clock cycles of txclkesc */
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset),
		  0x14 & DSI_TURN_AROUND_TIMEOUT_MASK);

	/*min 21 txclkesc, max: ffffh */
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset),
		  0xffff & DSI_RESET_TIMER_MASK);

	REG_WRITE((MIPIA_DPI_RESOLUTION_REG + reg_offset),
		  mode->vdisplay << 16 | mode->hdisplay);

	/*set DPI timing registers */
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					 dsi_config->lane_count,
					 dsi_config->bpp);

	REG_WRITE((MIPIA_HSYNC_COUNT_REG + reg_offset),
		  dpi_timing.hsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HBP_COUNT_REG + reg_offset),
		  dpi_timing.hbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HFP_COUNT_REG + reg_offset),
		  dpi_timing.hfp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HACTIVE_COUNT_REG + reg_offset),
		  dpi_timing.hactive_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VSYNC_COUNT_REG + reg_offset),
		  dpi_timing.vsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VBP_COUNT_REG + reg_offset),
		  dpi_timing.vbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VFP_COUNT_REG + reg_offset),
		  dpi_timing.vfp_count & DSI_DPI_TIMING_MASK);

	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);

	/*min: 7d0 max: 4e20 */
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0x000007d0);

	/*set up video mode */
	val = 0;
	val = dsi_config->video_mode | DSI_DPI_COMPLETE_LAST_LINE;
	REG_WRITE((MIPIA_VIDEO_MODE_FORMAT_REG + reg_offset), val);

	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);

	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);

	/*TODO: figure out how to setup these registers */
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3408);

	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset),
		  (0xa << 16) | 0x14);
#endif
	/*set device ready */
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000001);
}

void mdfld_dsi_dpi_set_color_mode(struct mdfld_dsi_config *dsi_config, bool on)
{
	struct mdfld_dsi_pkg_sender *sender =
	    mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	u32 spk_pkg = (on == true) ? MDFLD_DSI_DPI_SPK_COLOR_MODE_ON :
	    MDFLD_DSI_DPI_SPK_COLOR_MODE_OFF;

	PSB_DEBUG_ENTRY("Turn  color mode %s  pkg value= %d...\n",
			(on ? "on" : "off"), spk_pkg);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return;
	}

	/*send turn on/off color mode packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, spk_pkg);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return;
	}
	PSB_DEBUG_ENTRY("Turn  color mode %s successful.\n",
			(on ? "on" : "off"));
	return;
}

void mdfld_dsi_dpi_turn_on(struct mdfld_dsi_dpi_output *output, int pipe)
{
	struct drm_device *dev = output->dev;
	/* struct drm_psb_private * dev_priv = dev->dev_private; */
	u32 reg_offset = 0;

	PSB_DEBUG_ENTRY("pipe %d panel state %d\n", pipe, output->panel_on);

	if (output->panel_on)
		return;

	if (pipe)
		reg_offset = MIPIC_REG_OFFSET;

	/* clear special packet sent bit */
	if (REG_READ(MIPIA_INTR_STAT_REG + reg_offset) &
	    DSI_INTR_STATE_SPL_PKG_SENT) {
		REG_WRITE((MIPIA_INTR_STAT_REG + reg_offset),
			  DSI_INTR_STATE_SPL_PKG_SENT);
	}

	/*send turn on package */
	REG_WRITE((MIPIA_DPI_CONTROL_REG + reg_offset),
		  DSI_DPI_CTRL_HS_TURN_ON);

	/*wait for SPL_PKG_SENT interrupt */
	mdfld_wait_for_SPL_PKG_SENT(dev, pipe);

	if (REG_READ(MIPIA_INTR_STAT_REG + reg_offset) &
	    DSI_INTR_STATE_SPL_PKG_SENT) {
		REG_WRITE((MIPIA_INTR_STAT_REG + reg_offset),
			  DSI_INTR_STATE_SPL_PKG_SENT);
	}

	output->panel_on = 1;

	/* FIXME the following is disabled to WA the X slow start issue for TMD panel */
	/* if(pipe == 2) */
	/*      dev_priv->dpi_panel_on2 = true; */
	/* else if (pipe == 0) */
	/*      dev_priv->dpi_panel_on = true; */
}

/**
 * Power on sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_on(struct mdfld_dsi_config *dsi_config,
				struct panel_funcs *p_funcs)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int retry;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	/*HW-Reset */
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config, RESET_FROM_OSPM_RESUME);

	/*Enable DSI PLL */
	if (!(REG_READ(regs->dpll_reg) & BIT31)) {
		if (ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if (ctx->cck_div)
				dpll = dpll | BIT11;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT12;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT13;
			REG_WRITE(regs->dpll_reg, dpll);
			dpll = dpll | BIT31;
			REG_WRITE(regs->dpll_reg, dpll);
		} else {
			REG_WRITE(regs->dpll_reg, 0x0);
			REG_WRITE(regs->fp_reg, 0x0);
			REG_WRITE(regs->fp_reg, ctx->fp);
			REG_WRITE(regs->dpll_reg, ((ctx->dpll) & ~BIT30));
			udelay(2);
			val = REG_READ(regs->dpll_reg);
			REG_WRITE(regs->dpll_reg, (val | BIT31));

			/*wait for PLL lock on pipe */
			retry = 10000;
			while (--retry && !(REG_READ(PIPEACONF) & BIT29))
				udelay(3);
			if (!retry) {
				DRM_ERROR("PLL failed to lock on pipe\n");
				err = -EAGAIN;
				goto power_on_err;
			}
		}
	}

	/*D-PHY parameter */
	REG_WRITE(regs->dphy_param_reg, ctx->dphy_param);

	/*Configure DSI controller */
	REG_WRITE(regs->mipi_control_reg, ctx->mipi_control);
	REG_WRITE(regs->intr_en_reg, ctx->intr_en);
	REG_WRITE(regs->hs_tx_timeout_reg, ctx->hs_tx_timeout);
	REG_WRITE(regs->lp_rx_timeout_reg, ctx->lp_rx_timeout);
	REG_WRITE(regs->turn_around_timeout_reg, ctx->turn_around_timeout);
	REG_WRITE(regs->device_reset_timer_reg, ctx->device_reset_timer);
	REG_WRITE(regs->high_low_switch_count_reg, ctx->high_low_switch_count);
	REG_WRITE(regs->init_count_reg, ctx->init_count);
	REG_WRITE(regs->eot_disable_reg, ctx->eot_disable);
	REG_WRITE(regs->lp_byteclk_reg, ctx->lp_byteclk);
	REG_WRITE(regs->clk_lane_switch_time_cnt_reg,
		  ctx->clk_lane_switch_time_cnt);
	REG_WRITE(regs->video_mode_format_reg, ctx->video_mode_format);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);

	/*DSI timing */
	REG_WRITE(regs->dpi_resolution_reg, ctx->dpi_resolution);
	REG_WRITE(regs->hsync_count_reg, ctx->hsync_count);
	REG_WRITE(regs->hbp_count_reg, ctx->hbp_count);
	REG_WRITE(regs->hfp_count_reg, ctx->hfp_count);
	REG_WRITE(regs->hactive_count_reg, ctx->hactive_count);
	REG_WRITE(regs->vsync_count_reg, ctx->vsync_count);
	REG_WRITE(regs->vbp_count_reg, ctx->vbp_count);
	REG_WRITE(regs->vfp_count_reg, ctx->vfp_count);

	/*Setup pipe timing */
	REG_WRITE(regs->htotal_reg, ctx->htotal);
	REG_WRITE(regs->hblank_reg, ctx->hblank);
	REG_WRITE(regs->hsync_reg, ctx->hsync);
	REG_WRITE(regs->vtotal_reg, ctx->vtotal);
	REG_WRITE(regs->vblank_reg, ctx->vblank);
	REG_WRITE(regs->vsync_reg, ctx->vsync);
	REG_WRITE(regs->pipesrc_reg, ctx->pipesrc);

	REG_WRITE(regs->dsppos_reg, ctx->dsppos);
	REG_WRITE(regs->dspstride_reg, ctx->dspstride);

	/*Setup plane */
	REG_WRITE(regs->dspsize_reg, ctx->dspsize);
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	REG_WRITE(regs->vgacntr_reg, ctx->vgacntr);

	/*Enable DSI Controller */
	REG_WRITE(regs->device_ready_reg, ctx->device_ready | BIT0);
	/*set low power output hold */
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT16));

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init)
		p_funcs->drv_ic_init(dsi_config, 0);

	/**
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */
	if (p_funcs && p_funcs->power_on)
		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}

	/*Enable MIPI Port */
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT31));

	/*Enable pipe */
	val = ctx->pipeconf;
	val &= ~0x000c0000;
	val |= BIT31;
	REG_WRITE(regs->pipeconf_reg, val);
	REG_WRITE(regs->pipestat_reg, ctx->pipestat |
		  PIPE_VBLANK_INTERRUPT_ENABLE);

	/*Wait for pipe enabling */
	retry = 10000;
	while (--retry && !(REG_READ(regs->pipeconf_reg) & BIT30))
		udelay(3);

	if (!retry) {
		DRM_ERROR("Failed to enable pipe\n");
		err = -EAGAIN;
		goto power_on_err;
	}

	/*enable plane */
	REG_WRITE(regs->dspcntr_reg, (ctx->dspcntr | BIT31));
	if (p_funcs->set_brightness(dsi_config, ctx->lastbrightnesslevel))
		DRM_ERROR("Failed to set panel brightness\n");

 power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power off sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_off(struct mdfld_dsi_config *dsi_config,
				 struct panel_funcs *p_funcs)
{
	u32 val = 0;
	u32 tmp = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int retry;
	int pipe0_enabled;
	int pipe2_enabled;
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs->set_brightness(dsi_config, 0))
		DRM_ERROR("Failed to set panel brightness\n");

	/*save the plane informaton, for it will updated */
	ctx->dspsurf = REG_READ(regs->dspsurf_reg);
	ctx->dsplinoff = REG_READ(regs->dsplinoff_reg);
	ctx->pipestat = REG_READ(regs->pipestat_reg);

	tmp = REG_READ(regs->pipeconf_reg);

	/*Disable panel */
	val = REG_READ(regs->dspcntr_reg);
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));
	/*Disable overlay & cursor panel assigned to this pipe */
	REG_WRITE(regs->pipeconf_reg, (tmp | (0x000c0000)));

	/*Disable pipe */
	val = REG_READ(regs->pipeconf_reg);
	REG_WRITE(regs->pipeconf_reg, (val & ~BIT31));

	/*wait for pipe disabling */
	retry = 100000;
	while (--retry && (REG_READ(regs->pipeconf_reg) & BIT30))
		udelay(5);

	if (!retry) {
		DRM_ERROR("Failed to disable pipe\n");
		err = -EAGAIN;
		goto power_off_err;
	}

	/**
	 * Different panel may have different ways to have
	 * panel turned off. Support it!
	 */
	if (p_funcs && p_funcs->power_off) {
		if (p_funcs->power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}

	/*Disable MIPI port */
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT31));
	/*clear Low power output hold */
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT16));
	/*Disable DSI controller */
	REG_WRITE(regs->device_ready_reg, (ctx->device_ready & ~BIT0));

	/*Disable DSI PLL */
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg, 0x0);
		/*power gate pll */
		REG_WRITE(regs->dpll_reg, BIT30);
	}

 power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Setup Display Controller to turn on/off a video mode panel.
 * Most of the video mode MIPI panel should follow the power on/off
 * sequence in this function.
 * NOTE: do NOT modify this function for new panel Enabling. Register
 * new panel function callbacks to make this function available for a
 * new video mode panel
 */
static int __mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_connector *dsi_connector;
	struct mdfld_dsi_dpi_output *dpi_output;
	struct mdfld_dsi_config *dsi_config;
	struct panel_funcs *p_funcs;
	int pipe;
	struct drm_device *dev;

	PSB_DEBUG_ENTRY("%s: mode %s\n", __func__, (on ? "on" : "off"));

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	p_funcs = dpi_output->p_funcs;
	pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_encoder);
	dev = dsi_config->dev;

	if (dsi_connector->status != connector_status_connected)
		return 0;

	spin_lock(&dsi_config->context_lock);

	if (on && !dsi_config->dsi_hw_context.panel_on) {
		if (__dpi_panel_power_on(dsi_config, p_funcs)) {
			DRM_ERROR("Failed to power on\n");
			goto set_power_err;
		}
		dsi_config->dsi_hw_context.panel_on = 1;
		/*if power on , then default turn off color mode,
		   let panel in full color */
		mdfld_dsi_dpi_set_color_mode(dsi_config, false);
	} else if (!on && dsi_config->dsi_hw_context.panel_on) {
		if (dpi_output->first_boot) {
			PSB_DEBUG_ENTRY
			    (" Skip turn off, if first time boot and panel have enabled\n");
			dpi_output->first_boot = 0;
			goto fun_exit;
		}
		if (__dpi_panel_power_off(dsi_config, p_funcs)) {
			DRM_ERROR("Failed to power off\n");
			goto set_power_err;
		}
		dsi_config->dsi_hw_context.panel_on = 0;
	}
 fun_exit:
	spin_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("successfully\n");
	return 0;
 set_power_err:
	spin_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("unsuccessfully!!!!\n");
	return -EAGAIN;
}

void mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_encoder_get_config(dsi_encoder);
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	u32 mipi_reg = MIPI;
	u32 pipeconf_reg = PIPEACONF;

	PSB_DEBUG_ENTRY("set power %s on pipe %d\n", on ? "On" : "Off", pipe);
	if (pipe)
		if (!(dev_priv->panel_desc & DISPLAY_B) ||
		    !(dev_priv->panel_desc & DISPLAY_C))
			return;

	if (pipe) {
		mipi_reg = MIPI_C;
		pipeconf_reg = PIPECCONF;
	}
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/*start up display island if it was shutdown */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	if (on) {
		if (get_panel_type(dev, pipe) == TMD_VID) {
			if (dsi_device_ready) {
				ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
				return;
			}

			mdfld_dsi_configure_up(dsi_encoder, pipe);
			dsi_device_ready = true;
		} else {
			/*enable mipi port */
			REG_WRITE(mipi_reg, (REG_READ(mipi_reg) | BIT31));
			REG_READ(mipi_reg);

			mdfld_dsi_dpi_turn_on(dpi_output, pipe);
			mdfld_dsi_tpo_ic_init(dsi_config, pipe);
		}

		if (pipe == 2) {
			dev_priv->dpi_panel_on2 = true;
		} else {
			dev_priv->dpi_panel_on = true;
		}

	} else {
		if (get_panel_type(dev, pipe) == TMD_VID) {
			if (!dsi_device_ready) {
				ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
				return;
			}

			mdfld_dsi_configure_down(dsi_encoder, pipe);
			dsi_device_ready = false;
		} else {
			mdfld_dsi_dpi_shut_down(dpi_output, pipe);

			/*disable mipi port */
			REG_WRITE(mipi_reg, (REG_READ(mipi_reg) & ~BIT31));
			REG_READ(mipi_reg);
		}

		if (pipe == 2) {
			dev_priv->dpi_panel_on2 = false;
		} else {
			dev_priv->dpi_panel_on = false;
		}

	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
#else
	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_power(encoder, on);

#endif
}

void mdfld_dsi_dpi_dpms(struct drm_encoder *encoder, int mode)
{
	PSB_DEBUG_ENTRY("%s\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));
#if 0
	if (!gbdispstatus) {
		PSB_DEBUG_ENTRY
		    ("panel in suspend status, skip turn on/off from DMPS");
		return;
	}
#endif

	if (mode == DRM_MODE_DPMS_ON)
		mdfld_dsi_dpi_set_power(encoder, true);
	else
		mdfld_dsi_dpi_set_power(encoder, false);
}

bool mdfld_dsi_dpi_mode_fixup(struct drm_encoder *encoder,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_display_mode *fixed_mode = dsi_config->fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if (fixed_mode) {
		adjusted_mode->hdisplay = fixed_mode->hdisplay;
		adjusted_mode->hsync_start = fixed_mode->hsync_start;
		adjusted_mode->hsync_end = fixed_mode->hsync_end;
		adjusted_mode->htotal = fixed_mode->htotal;
		adjusted_mode->vdisplay = fixed_mode->vdisplay;
		adjusted_mode->vsync_start = fixed_mode->vsync_start;
		adjusted_mode->vsync_end = fixed_mode->vsync_end;
		adjusted_mode->vtotal = fixed_mode->vtotal;
		adjusted_mode->clock = fixed_mode->clock;
		drm_mode_set_crtcinfo(adjusted_mode, CRTC_INTERLACE_HALVE_V);
	}

	return true;
}

void mdfld_dsi_dpi_prepare(struct drm_encoder *encoder)
{
	PSB_DEBUG_ENTRY("\n");

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	mdfld_dsi_dpi_set_power(encoder, false);
#else
	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
#endif
}

void mdfld_dsi_dpi_commit(struct drm_encoder *encoder)
{
	PSB_DEBUG_ENTRY("\n");
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dpi_output *dpi_output;

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	mdfld_dsi_dpi_set_power(encoder, false);
#else
	/*Everything is ready, commit DSI hw context to HW */
	__mdfld_dsi_dpi_set_power(encoder, true);
#endif
	dpi_output->first_boot = 0;
}

void dsi_debug_MIPI_reg(struct drm_device *dev)
{
	u32 temp_val = 0;

	PSB_DEBUG_ENTRY("[DISPLAY] Enter %s\n", __func__);
	temp_val = REG_READ(MIPI);
	PSB_DEBUG_ENTRY("[DISPLAY] MIPI = %x\n", temp_val);

	/* set the lane speed */
	temp_val = REG_READ(MIPI_CONTROL_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] MIPI_CONTROL_REG = %x\n", temp_val);

	/* Enable all the error interrupt */
	temp_val = REG_READ(INTR_EN_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] INTR_EN_REG = %x\n", temp_val);
	temp_val = REG_READ(TURN_AROUND_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] TURN_AROUND_TIMEOUT_REG = %x\n", temp_val);
	temp_val = REG_READ(DEVICE_RESET_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DEVICE_RESET_REG = %x\n", temp_val);
	temp_val = REG_READ(INIT_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] INIT_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(DSI_FUNC_PRG_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DSI_FUNC_PRG_REG = %x\n", temp_val);

	temp_val = REG_READ(DPI_RESOLUTION_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPI_RESOLUTION_REG = %x\n", temp_val);

	temp_val = REG_READ(VERT_SYNC_PAD_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_SYNC_PAD_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(VERT_BACK_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_BACK_PORCH_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(VERT_FRONT_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VERT_FRONT_PORCH_COUNT_REG = %x\n",
			temp_val);

	temp_val = REG_READ(HORIZ_SYNC_PAD_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_SYNC_PAD_COUNT_REG = %x\n", temp_val);
	temp_val = REG_READ(HORIZ_BACK_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_BACK_PORCH_COUNT_REG = %x\n",
			temp_val);
	temp_val = REG_READ(HORIZ_FRONT_PORCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_FRONT_PORCH_COUNT_REG = %x\n",
			temp_val);
	temp_val = REG_READ(HORIZ_ACTIVE_AREA_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HORIZ_ACTIVE_AREA_COUNT_REG = %x\n",
			temp_val);

	temp_val = REG_READ(VIDEO_FMT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] VIDEO_FMT_REG = %x\n", temp_val);

	temp_val = REG_READ(HS_TX_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HS_TX_TIMEOUT_REG = %x\n", temp_val);
	temp_val = REG_READ(LP_RX_TIMEOUT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] LP_RX_TIMEOUT_REG = %x\n", temp_val);

	temp_val = REG_READ(HIGH_LOW_SWITCH_COUNT_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] HIGH_LOW_SWITCH_COUNT_REG = %x\n", temp_val);

	temp_val = REG_READ(EOT_DISABLE_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] EOT_DISABLE_REG = %x\n", temp_val);

	temp_val = REG_READ(LP_BYTECLK_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] LP_BYTECLK_REG = %x\n", temp_val);
	temp_val = REG_READ(MAX_RET_PAK_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] MAX_RET_PAK_REG = %x\n", temp_val);
	temp_val = REG_READ(DPI_CONTROL_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPI_CONTROL_REG = %x\n", temp_val);
	temp_val = REG_READ(DPHY_PARAM_REG);
	PSB_DEBUG_ENTRY("[DISPLAY] DPHY_PARAM_REG = %x\n", temp_val);
/*	temp_val = REG_READ(PIPEACONF);
	PSB_DEBUG_ENTRY("[DISPLAY] PIPEACONF = %x\n", temp_val);
	temp_val = REG_READ(DSPACNTR);
	PSB_DEBUG_ENTRY("[DISPLAY] DSPACNTR = %x\n", temp_val);
*/
}

/**
 * Setup DPI timing for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static void __mdfld_dsi_dpi_set_timing(struct mdfld_dsi_config *config,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct mdfld_dsi_hw_context *ctx;

	if (!config) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	mode = adjusted_mode;
	ctx = &config->dsi_hw_context;

	spin_lock(&config->context_lock);

	/*dpi resolution */
	ctx->dpi_resolution = (mode->vdisplay << 16 | mode->hdisplay);

	/*Calculate DPI timing */
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					 config->lane_count, config->bpp);

	/*update HW context with new DPI timings */
	ctx->hsync_count = dpi_timing.hsync_count;
	ctx->hbp_count = dpi_timing.hbp_count;
	ctx->hfp_count = dpi_timing.hfp_count;
	ctx->hactive_count = dpi_timing.hactive_count;
	ctx->vsync_count = dpi_timing.vsync_count;
	ctx->vbp_count = dpi_timing.vbp_count;
	ctx->vfp_count = dpi_timing.vfp_count;

	/*setup mipi port configuration */
	if (config->pipe == 0)
		ctx->mipi = PASS_FROM_SPHY_TO_AFE | config->lane_config;

	spin_unlock(&config->context_lock);
}

void mdfld_dsi_dpi_mode_set(struct drm_encoder *encoder,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_encoder_get_config(dsi_encoder);
#ifdef MIPI_DEBUG_LOG
	struct drm_device *dev = dsi_config->dev;
#endif
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);

	u32 pipeconf_reg = PIPEACONF;
	u32 dspcntr_reg = DSPACNTR;
	u32 mipi_reg = MIPI;
	u32 reg_offset = 0;

	u32 mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE | SEL_FLOPPED_HSTX;

	PSB_DEBUG_ENTRY("set mode %dx%d on pipe %d",
			mode->hdisplay, mode->vdisplay, pipe);

	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	if (pipe) {
		pipeconf_reg = PIPECCONF;
		dspcntr_reg = DSPCCNTR;
		mipi_reg = MIPI_C;
		reg_offset = MIPIC_REG_OFFSET;
	} else {
		mipi |= 2;
	}

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/*start up display island if it was shutdown */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	/*set up mipi port FIXME: do at init time */
	REG_WRITE(mipi_reg, mipi);
	REG_READ(mipi_reg);

	/*set up DSI controller DPI interface */
	mdfld_dsi_dpi_controller_init(dsi_config, pipe);

	if (get_panel_type(dev, pipe) == TMD_VID) {
		/*Pull High Reset */
		dsi_set_bridge_reset_state(0);
		/*Init MIPI Bridge and Panel */
		mdfld_init_TOSHIBA_MIPI(dev);
		dsi_device_ready = true;
	} else {
		/*turn on DPI interface */
		mdfld_dsi_dpi_turn_on(dpi_output, pipe);
	}

	/*set up pipe */
	REG_WRITE(pipeconf_reg, pipeconf);
	REG_READ(pipeconf_reg);

	/*set up display plane */
	REG_WRITE(dspcntr_reg, dspcntr);
	REG_READ(dspcntr_reg);

	msleep(20);		/* FIXME: this should wait for vblank */

	PSB_DEBUG_ENTRY("State %x, power %d\n",
			REG_READ(MIPIA_INTR_STAT_REG + reg_offset),
			dpi_output->panel_on);

	if (get_panel_type(dev, pipe) == TMD_VID) {
		/*mdfld_dsi_dpi_turn_on(dpi_output, pipe); */
	} else {
		/* init driver ic */
		mdfld_dsi_tpo_ic_init(dsi_config, pipe);
		/*init backlight */
		mdfld_dsi_brightness_init(dsi_config, pipe);
	}

#ifdef MIPI_DEBUG_LOG
	dsi_debug_MIPI_reg(dev);
#endif

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
#else
	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_timing(dsi_config, mode, adjusted_mode);
#endif
}

void mdfld_dsi_dpi_save(struct drm_encoder *encoder)
{
#if 1				/* FIXME MRFLD */
	return;
#endif				/* FIXME MRFLD */
	printk(KERN_ALERT "%s\n", __func__);

	if (!encoder)
		return;

	/*turn off */
	__mdfld_dsi_dpi_set_power(encoder, false);
}

void mdfld_dsi_dpi_restore(struct drm_encoder *encoder)
{
	printk(KERN_ALERT "%s\n", __func__);

	if (!encoder)
		return;

	/*turn on */
	__mdfld_dsi_dpi_set_power(encoder, true);
}

/**
 * Exit from DSR
 */
void mdfld_dsi_dpi_exit_idle(struct drm_device *dev,
			     u32 update_src,
			     void *p_surfaceAddr, bool check_hw_on_only)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_ENTRY("\n");

	if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
		DRM_ERROR("hw begin failed\n");
		return;
	}

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
	if (dev_priv->b_is_in_idle) {
		/* update the surface base address. */
		if (p_surfaceAddr) {
			REG_WRITE(DSPASURF, *((u32 *) p_surfaceAddr));
#if defined(CONFIG_MID_DUAL_MIPI)
			REG_WRITE(DSPCSURF, *((u32 *) p_surfaceAddr));
#endif
		}

		mid_enable_pipe_event(dev_priv, 0);
		psb_enable_pipestat(dev_priv, 0, PIPE_VBLANK_INTERRUPT_ENABLE);
		dev_priv->b_is_in_idle = false;
		dev_priv->dsr_idle_count = 0;
	}
	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
}

/*
 * Init DSI DPI encoder.
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DPI encoder, NULL on error
 */
struct mdfld_dsi_encoder *mdfld_dsi_dpi_init(struct drm_device *dev, struct mdfld_dsi_connector
					     *dsi_connector,
					     struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_display_mode *fixed_mode = NULL;
	int pipe;
	int ret;

	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	if (!dsi_connector || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}
	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;

#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/*panel hard-reset */
	if (p_funcs->reset) {
		ret = p_funcs->reset(dsi_config, RESET_FROM_BOOT_UP);
		if (ret) {
			DRM_ERROR("Panel %d hard-reset failed\n", pipe);
			return NULL;
		}
	}

	/*detect panel connection stauts */
	if (p_funcs->detect) {
		ret = p_funcs->detect(dsi_config, pipe);
		if (ret) {
			DRM_INFO("Detecting Panel %d, Not connected\n", pipe);
			dsi_connector->status = connector_status_disconnected;
		} else {
			PSB_DEBUG_ENTRY("Panel %d is connected\n", pipe);
			dsi_connector->status = connector_status_connected;
		}

		if (dsi_connector->status == connector_status_disconnected &&
		    pipe == 0) {
			DRM_ERROR("Primary panel disconnected\n");
			return NULL;
		}
	} else {
		/*use the default config */
		if (pipe == 0)
			dsi_connector->status = connector_status_connected;
		else
			dsi_connector->status = connector_status_disconnected;
	}

	/*init DSI controller */
	if (p_funcs->dsi_controller_init)
		p_funcs->dsi_controller_init(dsi_config, pipe, 0);
#else
	/* Enable MIPI panel  by default for PR1
	 * platform where panel detection code
	 * doesn't ready.
	 */
	dsi_connector->status = connector_status_connected;
#endif

	/**
	 * TODO: can we keep these code out of display driver as
	 * it will make display driver hard to be maintained
	 */
	if (dsi_connector->status == connector_status_connected) {
		if (pipe == 0)
			dev_priv->panel_desc |= DISPLAY_A;
		if (pipe == 2)
			dev_priv->panel_desc |= DISPLAY_C;
	}

	dpi_output = kzalloc(sizeof(struct mdfld_dsi_dpi_output), GFP_KERNEL);
	if (!dpi_output) {
		DRM_ERROR("No memory\n");
		return NULL;
	}
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	if (dsi_connector->pipe)
		dpi_output->panel_on = 0;
#endif

	dpi_output->dev = dev;
	dpi_output->p_funcs = p_funcs;
	dpi_output->first_boot = 1;

	/*get fixed mode */
	fixed_mode = dsi_config->fixed_mode;

#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	/*detect power status of the connected panel */
	if (p_funcs->get_panel_power_state) {
		ret = p_funcs->get_panel_power_state(dsi_config, pipe);
		if (ret == MDFLD_DSI_PANEL_POWER_OFF)
			dsi_config->dsi_hw_context.panel_on = 0;
		else
			dsi_config->dsi_hw_context.panel_on = 1;
	} else {
		/*use the default config */
		if (pipe == 0)
			dsi_config->dsi_hw_context.panel_on = 1;
		else
			dsi_config->dsi_hw_context.panel_on = 0;
	}
#endif
	/*create drm encoder object */
	connector = &dsi_connector->base.base;
	encoder = &dpi_output->base.base;
	drm_encoder_init(dev,
			 encoder,
			 p_funcs->encoder_funcs, DRM_MODE_ENCODER_MIPI);
	drm_encoder_helper_add(encoder, p_funcs->encoder_helper_funcs);

	/*attach to given connector */
	drm_mode_connector_attach_encoder(connector, encoder);

	/*set possible crtcs and clones */
	if (dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dpi_exit_idle;
#if defined(CONFIG_MID_DSI_DPU) || defined(CONFIG_MID_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif				/*CONFIG_MID_DSI_DSR */

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	dev_priv->dpi_panel_on = true;
	gencoder = encoder;

	gpio_request(GPIO_MIPI_BRIDGE_RESET, "display");
	gpio_request(GPIO_MIPI_PANEL_RESET, "display");
#endif

	PSB_DEBUG_ENTRY("successfully\n");

	return &dpi_output->base;
}

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
static int __DSI_I2C_ByteRead(u16 reg, int count)
{
	char rxData[4] = { 0 };
	char regData[2] = { 0 };
	struct i2c_msg msgs[] = {
		{
		 .addr = tc358762->client->addr,
		 .flags = 0,
		 .len = 2,
		 },
		{
		 .addr = tc358762->client->addr,
		 .flags = I2C_M_RD,
		 .len = count - 2,
		 },
	};

	regData[0] = (reg & 0xFF00) >> 8;
	regData[1] = reg & 0xFF;

	msgs[0].buf = regData;
	msgs[1].buf = rxData;

	printk(KERN_ERR "Register: 0x%x\n", reg);
	if (i2c_transfer(tc358762->client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "[DISPLAY] %s: transfer error\n", __func__);
		return -EIO;
	} else {
		int i = 0;
		for (i = 0; i < count - 2; i++)
			printk(KERN_ERR "%02x ", rxData[i]);
		printk(KERN_ERR "\n");
		return rxData[0];
	}
}

static int DSI_I2C_ByteRead(u16 reg, int count)
{
	if (tc358762->client)
		return __DSI_I2C_ByteRead(reg, count);
	else
		return -EIO;
}

static int __DSI_I2C_ByteWrite(u16 reg, u32 data, int count)
{
	char txData[6] = { 0 };
	int i = 0;
	struct i2c_msg msg[] = {
		{
		 .addr = tc358762->client->addr,
		 .flags = 0,
		 .len = count,
		 },
	};

	/* Set the register */
	txData[0] = (reg & 0xFF00) >> 8;
	txData[1] = reg & 0xFF;

	if (count == 6) {
		/* Set the data */
		txData[2] = (data & 0xFF);
		txData[3] = (data & 0xFF00) >> 8;
		txData[4] = (data & 0xFF0000) >> 16;
		txData[5] = (data & 0xFF000000) >> 24;
	} else {
		/* TODO */
	}

	msg[0].buf = txData;

	if (i2c_transfer(tc358762->client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "[DISPLAY] %s: transfer error\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}

static int DSI_I2C_ByteWrite(u16 reg, u32 data, int count)
{
	if (tc358762->client)
		return __DSI_I2C_ByteWrite(reg, data, count);
	else
		return -EIO;
}

static int tc358762_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	/*I2C Check */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR
		       "[DISPLAY] %s: Check I2C functionality failed.\n",
		       __func__);
		return -ENODEV;
	}

	tc358762 = kzalloc(sizeof(struct tc358762_info), GFP_KERNEL);

	if (tc358762 == NULL) {
		printk(KERN_ERR "[DISPLAY] %s: Can not allocate memory.\n",
		       __func__);
		return -ENOMEM;
	}

	tc358762->client = client;

	i2c_set_clientdata(client, tc358762);

	tc358762->client->addr = 0x0B;

	return 0;
}

static int tc358762_remove(struct i2c_client *client)
{
	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	dev_set_drvdata(&client->dev, 0);
	kfree(tc358762);
	return 0;
}

static const struct i2c_device_id tc358762_id[] = {
	{"tc358762", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358762_id);

static struct i2c_driver tc358762_i2c_driver = {
	.driver = {
		   .name = "tc358762",
		   },
	.id_table = tc358762_id,
	.probe = tc358762_probe,
	.remove = tc358762_remove,
};

static int mipi_dsi_dev_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
#if defined(CONFIG_SUPPORT_TMD_MIPI_600X1024_DISPLAY) \
	|| defined(CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY)
	struct drm_encoder *encoder = gencoder;
#endif

	PSB_DEBUG_ENTRY("[DISPLAY] %s: MIPI DSI driver IOCTL, cmd = %d.\n",
			__func__, cmd);

	switch (cmd) {
#if defined(CONFIG_SUPPORT_TMD_MIPI_600X1024_DISPLAY) \
	|| defined(CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY)
	case IOCTL_LCM_POWER_ON:
		mdfld_dsi_dpi_set_power(encoder, 1);
		break;
	case IOCTL_LCM_POWER_OFF:
		mdfld_dsi_dpi_set_power(encoder, 0);
		break;
#endif
	default:
		printk(KERN_ERR
		       "[DISPLAY] %s: MIPI DSI driver not support IOCTL.\n",
		       __func__);
		break;
	}

	return 0;
}

static const struct file_operations mipi_dsi_dev_fops = {
	.owner = THIS_MODULE,
	.ioctl = mipi_dsi_dev_ioctl,
};

static struct miscdevice mipi_dsi_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mipi_dsi",
	.fops = &mipi_dsi_dev_fops,
};

static int __init mipi_dsi_init(void)
{
	int ret = 0;
	struct i2c_adapter *adapter_tc358762;
	struct i2c_client *client;
	struct i2c_board_info info;

	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	ret = misc_register(&mipi_dsi_dev);
	if (ret) {
		printk(KERN_ERR "[DISPLAY] %s: Can not register misc device.\n",
		       __func__);
		return ret;
	}

	return i2c_add_driver(&tc358762_i2c_driver);
}

static void __exit mipi_dsi_exit(void)
{
	PSB_DEBUG_ENTRY("[DISPLAY] %s\n", __func__);

	misc_deregister(&mipi_dsi_dev);

	i2c_del_driver(&tc358762_i2c_driver);
}

module_init(mipi_dsi_init);
module_exit(mipi_dsi_exit);

#endif				/*-- CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY --*/
/* DIV5-MM-DISPLAY-NC-LCM_INIT-00-]- */
