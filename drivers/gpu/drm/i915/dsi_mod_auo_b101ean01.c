/*
 * Copyright c 2013 Intel Corporation
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
 * Author: Chris Tsai <chrisx.tsai@intel.com>
 *
 *
 */
#include <linux/module.h>
#include <linux/init.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include <asm/intel-mid.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_auo_b101ean01.h"
#include "tx358774.h"
#include <linux/mfd/intel_mid_pmic.h>


struct i2c_client *bridge_i2c_client;
struct i2c_adapter *bridge_i2c_adapter;
bool TS_bridge_cfg = false;

/* I2C stress test */
static struct workqueue_struct *query_bridge_wq;
static struct delayed_work query_bridge_work;
int stress_test_poll_mode = 0;

#define TC358774_IOC_MAGIC 0x0F
#define TC358774_IOC_MAXNR 2
#define TC358774_POLL_DATA _IOR(TC358774_IOC_MAGIC,2,int )

#define TC358774_IOCTL_START_HEAVY 2
#define TC358774_IOCTL_START_NORMAL 1
#define TC358774_IOCTL_END 0

#define TC358774_START_NORMAL    msecs_to_jiffies(5000)  //0.2HZ;
#define TC358774_START_HEAVY	 msecs_to_jiffies(1000)  //1HZ;
 
void clear_error(void)
{
	struct i2c_client *i2c = bridge_i2c_client;
	u32 reg = 0;

	tc358774_regw(i2c, 0x300, 0xffff0000);
	tc358774_regr(i2c, 0x500, &reg);
	tc358774_regw(i2c, 0x300, 0xc0800000);
	tc358774_regr(i2c, 0x500, &reg);
}

void query_0x500(void)
{
	struct i2c_client *i2c = bridge_i2c_client;
	u32 reg = 0;

	tc358774_regr(i2c, 0x500, &reg);
	if (reg&0x3){
		printk("Error: 0x500 is 0x%x\n", reg);
		clear_error();
	}
}
#if 0
static void work_fn(struct work_struct *w)
{
	query_0x500();
	queue_delayed_work(query_bridge_wq, &query_bridge_work, 3 * HZ);
}
#endif


/*   MIPI to LVDS bridge type : 0-Toshiba; 1-TI(DSI83)...  */
#define MIPI_BRIDGE_TYPE (0)

static int bridge_hack_create_device(void)
{
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(BRIDGE_I2C_ADAPTER);
	if (!adapter) {
		pr_err("%s: i2c_get_adapter(%d) failed\n", __func__,
			BRIDGE_I2C_ADDR);
		return -EINVAL;
	}
	bridge_i2c_adapter = adapter;

	return 0;
}


int tc358774_regw(struct i2c_client *client, u16 reg, u32 value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		return -EAGAIN;
	}


	return 0;
}

int tc358774_regr(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};
	u8 rx_data[4];
	struct i2c_msg msgs[] = {
		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		return -EAGAIN;
	}

	*value = rx_data[3] << 24 | rx_data[2] << 16 |
		rx_data[1] << 8 | rx_data[0];


	return 0;
}

void tc358774_configure_lvds_bridge(void)
{
	struct i2c_client *i2c = bridge_i2c_client;
	u32 id = 0;

	tc358774_regr(i2c, IDREG, &id);
	printk("tc358774 ID 0x%08x\n", id);

	tc358774_regw(i2c, PPI_TX_RX_TA, 0x00020003); //0x00030005
	tc358774_regw(i2c, PPI_LPTXTIMCNT, 0x00000002);//0x00000003
	tc358774_regw(i2c, PPI_D0S_CLRSIPOCOUNT, 0x00000008);
	tc358774_regw(i2c, PPI_D1S_CLRSIPOCOUNT, 0x00000008);
	tc358774_regw(i2c, PPI_D2S_CLRSIPOCOUNT, 0x00000008);
	tc358774_regw(i2c, PPI_D3S_CLRSIPOCOUNT, 0x00000008);
	tc358774_regw(i2c, PPI_LANEENABLE, 0x0000001F);
	tc358774_regw(i2c, DSI_LANEENABLE, 0x0000001F);
	tc358774_regw(i2c, PPI_STARTPPI, 0x00000001);
	tc358774_regw(i2c, DSI_STARTPPI, 0x00000001);
	
	tc358774_regw(i2c, VPCTRL, 0x02000120);
	tc358774_regw(i2c, HTIM1, 0x00100020);
	tc358774_regw(i2c, HTIM2, 0x00300500);
	tc358774_regw(i2c, VTIM1, 0x00040004);
	tc358774_regw(i2c, VTIM2, 0x000C0320);
	tc358774_regw(i2c, VFUEN, 0x00000001);
	tc358774_regw(i2c, LVPHY0, 0x00448006);
	usleep_range(100,150);
	tc358774_regw(i2c, LVPHY0, 0x00048006);
	tc358774_regw(i2c, SYSRST, 0x00000004);
	
	tc358774_regw(i2c, LVMX0003, 0x03020100);
	tc358774_regw(i2c, LVMX0407, 0x08050704);
	tc358774_regw(i2c, LVMX0811, 0x0F0E0A09);
	tc358774_regw(i2c, LVMX1215, 0x100D0C0B);
	tc358774_regw(i2c, LVMX1619, 0x12111716);
	tc358774_regw(i2c, LVMX2023, 0x1B151413);
	tc358774_regw(i2c, LVMX2427, 0x061A1918);
	
	tc358774_regw(i2c, LVCFG, 0x00000031);
}

void tc358774_i2c_stress_test(struct work_struct *work)
{
	struct i2c_client *i2c = bridge_i2c_client;
	u32 id = 0;

	tc358774_regr(i2c, IDREG, &id);
	tc358774_regw(i2c, DEBUG00, 0x00000100);
	msleep(2000);
	DRM_INFO("[DISPLAY] : tc358774 ID 0x%08x\n",id );

    queue_delayed_work(query_bridge_wq, &query_bridge_work, stress_test_poll_mode);
	
}

static int sn65dsi83_regw(struct i2c_client *client, u8 reg, u8 value)
{
	int r;
	int ret = -1;
	int nRetry = 3;
	u8 tx_data[] = {
		reg & 0xff,
		value & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = 0x2D,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};
	
	do {
		r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));
		if (r < 0) {
		//	dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
		//		__func__, reg, value, r);
			pr_info("eGGG : i2c_transfer error = %d\n",r);
			if(r == -110) //Error waiting for write ready
				msleep(25);
		}else if (r < ARRAY_SIZE(msgs))
		//	dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
		//		__func__, reg, value, r);
			pr_info("eGGG : i2c_transfer error = %d\n",r);
		else {
			ret = 0;
			break;
		}
		msleep(25);
		
	}while(nRetry--);

	if (ret) printk("MDSS:LVDS:%s: Failed!! reg=0x%x, value=0x%x\n", __func__,reg,value);
	return ret;
}


static int sn65dsi83_regr(struct i2c_client *client, u8 reg, u8 *value)
{
	int r;
	int ret = -1;
	int nRetry = 3;
	u8 tx_data[] = {
		reg & 0xff,
	};
	u8 rx_data[1];
	struct i2c_msg msgs[] = {
		{
			.addr = 0x2D,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = 0x2D,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};
	do {
		r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));
		if (r < 0) {
		//	dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
		//		__func__, reg, value, r);
			pr_info("eGGG : i2c_transfer error = %d\n",r);
			if(r == -110) //Error waiting for write ready
				msleep(25);
		}else if (r < ARRAY_SIZE(msgs))
		//	dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
		//		__func__, reg, value, r);
			pr_info("eGGG : i2c_transfer error = %d\n",r);
		else {
			ret = 0;
			break;
		}
		msleep(25);
		
	}while(nRetry--);
	*value = rx_data[0];

	return ret;
}

void sn65dsi83_dump_lvds_bridge(void)
{
	struct i2c_client *i2c = bridge_i2c_client;

	u8 CSR_ADDR[] = {0x9,0x0A,0x0B,0x0D,0x10,0x11,0x12,0x13,0x18,0x19,0x1A,0x1B,0};
	u8* csr_addr = CSR_ADDR;
	u8	i,val;

	while (*csr_addr){
		sn65dsi83_regr(i2c, *csr_addr, &val);
		printk("0x%02X=>0x%02X \n", *csr_addr,val);
		csr_addr++;
	}
	for(i=0x20; i<=0x3E;i++) {
		sn65dsi83_regr(i2c, i, &val);
		printk("0x%02X=>0x%02X \n", i,val);
	}
		sn65dsi83_regr(i2c, 0xE0, &val);
		printk("0xE0=>0x%02X \n",val);
		sn65dsi83_regr(i2c, 0xE1, &val);
		printk("0xE1=>0x%02X \n",val);
		sn65dsi83_regr(i2c, 0xE5, &val);
		printk("0xE5=>0x%02X \n", val);
}

void sn65dsi83_configure_lvds_bridge(void)
{
	struct i2c_client *i2c = bridge_i2c_client;

	sn65dsi83_regw(i2c, 0x09, 0x00 );
	sn65dsi83_regw(i2c, 0x0A, 0x05 );
	sn65dsi83_regw(i2c, 0x0B, 0x10 );  
	sn65dsi83_regw(i2c, 0x0D, 0x00 );
	sn65dsi83_regw(i2c, 0x10, 0x26 );
	sn65dsi83_regw(i2c, 0x11, 0x00 );
	sn65dsi83_regw(i2c, 0x12, 0x2A );  
	sn65dsi83_regw(i2c, 0x13, 0x00 );
	sn65dsi83_regw(i2c, 0x18, 0x78 );
	sn65dsi83_regw(i2c, 0x19, 0x40 );
	sn65dsi83_regw(i2c, 0x1A, 0x01 ); 
	sn65dsi83_regw(i2c, 0x1B, 0x10 );
	sn65dsi83_regw(i2c, 0x20, 0x00 );
	sn65dsi83_regw(i2c, 0x21, 0x05 );
	sn65dsi83_regw(i2c, 0x22, 0x00 );  
	sn65dsi83_regw(i2c, 0x23, 0x00 );
	sn65dsi83_regw(i2c, 0x24, 0x00 );
	sn65dsi83_regw(i2c, 0x25, 0x00 );
	sn65dsi83_regw(i2c, 0x26, 0x00 );  
	sn65dsi83_regw(i2c, 0x27, 0x00 );
	sn65dsi83_regw(i2c, 0x28, 0x21 );
	sn65dsi83_regw(i2c, 0x29, 0x00 );
	sn65dsi83_regw(i2c, 0x2A, 0x00 );
	sn65dsi83_regw(i2c, 0x2B, 0x00 );		
	sn65dsi83_regw(i2c, 0x2C, 0x20 );
	sn65dsi83_regw(i2c, 0x2D, 0x00 );
	sn65dsi83_regw(i2c, 0x2E, 0x00 );  
	sn65dsi83_regw(i2c, 0x2F, 0x00 );
	sn65dsi83_regw(i2c, 0x30, 0x06 );
	sn65dsi83_regw(i2c, 0x31, 0x00 );
	sn65dsi83_regw(i2c, 0x32, 0x00 );  
	sn65dsi83_regw(i2c, 0x33, 0x00 );
	sn65dsi83_regw(i2c, 0x34, 0x40 );
	sn65dsi83_regw(i2c, 0x35, 0x00 );
	sn65dsi83_regw(i2c, 0x36, 0x00 );
	sn65dsi83_regw(i2c, 0x37, 0x00 );
	sn65dsi83_regw(i2c, 0x38, 0x00 );
	sn65dsi83_regw(i2c, 0x39, 0x00 );
	sn65dsi83_regw(i2c, 0x3A, 0x00 );
	sn65dsi83_regw(i2c, 0x3B, 0x00 );
	sn65dsi83_regw(i2c, 0x3C, 0x00 );
	sn65dsi83_regw(i2c, 0x3D, 0x00 );
	sn65dsi83_regw(i2c, 0x3E, 0x00 );

	return;
}

void sn65dsi83_after_configure_bridge(void){
	
	
	struct i2c_client *i2c = bridge_i2c_client;
//	int	gpio = 0;
	u8 pll_lock_value = 0;
	u8 reg_value = 0; 
//Test	msleep(150); // about 9 frames
// Use I2C to set the PLL_EN.
//	ti_lvds_regw(i2c, 0x0D, 0x01, 0x01);
	sn65dsi83_regr(i2c, 0x0D, &reg_value);
	reg_value |= 0x01;
	sn65dsi83_regw(i2c, 0x0D, reg_value);
// Use I2C to check the PLL_LOCK.
	{
		int wait_count = 10;
		while(((pll_lock_value&0x80)!=0x80) && wait_count--){
			msleep(5);
			sn65dsi83_regr(i2c, 0x0A, &pll_lock_value);
			printk("eGGG : pll_lock_value = %x, wait_count=%d \n", pll_lock_value,wait_count);
		}
	}

	sn65dsi83_regr(i2c, 0x09, &reg_value);
	reg_value |= 0x01;
	sn65dsi83_regw(i2c, 0x09, reg_value);
	printk("eGGG : Do SOFT_RESET!!\n");

	return;
}

static void  b101ean01_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector) {
		DRM_DEBUG_KMS("Invalid input to get_info\n");
		return;
	}

	if (pipe == 0) {
		connector->display_info.width_mm = B101EAN01_PANEL_WIDTH;
		connector->display_info.height_mm = B101EAN01_PANEL_HEIGHT;
	}

	return;
}

/*
static void b101ean01_msgbus_reset()
{
	u32 msg_bus_port;
	u32 msg_bus_reg;
	u32 val;

	DRM_DEBUG_KMS("\n");


	msg_bus_port = 0x14;
	msg_bus_reg = 0x6d;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);

	msg_bus_port = 0x14;
	msg_bus_reg = 0x6f;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);

	msg_bus_port = 0x14;
	msg_bus_reg = 0x6f;
	val = intel_mid_msgbus_read32(msg_bus_port, msg_bus_reg);
	val  &= 0xFFFCFFFF;
	val  |= 0x00010000;
	intel_mid_msgbus_write32(msg_bus_port, msg_bus_reg, val);
}
*/

static void b101ean01_destroy(struct intel_dsi_device *dsi)
{
	gpio_free(BRIDGE_STANDBY);
	gpio_free(BRIDGE_RESET);
	
	cancel_delayed_work_sync(&query_bridge_work);
	destroy_workqueue(query_bridge_wq);
}

static void b101ean01_dump_regs(struct intel_dsi_device *dsi)
{
}


static void b101ean01_create_resources(struct intel_dsi_device *dsi)
{
}

static struct drm_display_mode *b101ean01_get_modes(
	struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode = NULL;
	
	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("panel: No memory\n");
		return NULL;
	}

	/* Hardcode 1280x800 */
	if (MIPI_BRIDGE_TYPE == 0) {
	mode->hdisplay = 1280;
	mode->hsync_start = mode->hdisplay + 64;  //32 front porch
	mode->hsync_end = mode->hsync_start + 0;  //+20 pulse width Fixme: Event mode has no end pkt?
	mode->htotal = mode->hsync_end + 32;  // back porch

	mode->vdisplay = 800;
	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 0;//+4 Fixme: Event mode has no end pkt?
	mode->vtotal = mode->vsync_end + 8;
	} else {
	mode->hdisplay = 1280;
	mode->hsync_start = mode->hdisplay + 32;  //32 front porch
	mode->hsync_end = mode->hsync_start + 32;  //+20 pulse width Fixme: Event mode has no end pkt?
	mode->htotal = mode->hsync_end + 64;  // back porch

	mode->vdisplay = 800;
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 4;//+4 Fixme: Event mode has no end pkt?
	mode->vtotal = mode->vsync_end + 8;
	}
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
	mode->htotal / 1000; //Fixme: Should clock hardcodes to 206.8MHZ?


	/* Configure */
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}


static bool b101ean01_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static enum drm_connector_status b101ean01_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

static bool b101ean01_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {		    
	return true;
}

static int b101ean01_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void b101ean01_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
  
  	//Fixme: need implement ULPS here
  
	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
  
}

void initial_bridge(void)
{
		if (MIPI_BRIDGE_TYPE == 0) {
			tc358774_configure_lvds_bridge();
		} else {
			sn65dsi83_configure_lvds_bridge();
		}
	//queue_delayed_work(query_bridge_wq, &query_bridge_work, 3 * HZ);
}

void b101ean01_send_otp_cmds(struct intel_dsi_device *dsi)
{		
/*
	if (TS_bridge_cfg != true){
		if (MIPI_BRIDGE_TYPE == 0) {
			tc358774_configure_lvds_bridge();
		} else {
			sn65dsi83_configure_lvds_bridge();
		}
		TS_bridge_cfg = true;
	}
	queue_delayed_work(query_bridge_wq, &query_bridge_work, 3 * HZ);
*/
}

static long
tc358774_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	if (_IOC_TYPE(cmd) != TC358774_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > TC358774_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case TC358774_POLL_DATA:
			if (arg == TC358774_IOCTL_START_HEAVY){
				DRM_INFO("TC358774 : ioctl heavy\n");
				stress_test_poll_mode = TC358774_START_HEAVY;
				queue_delayed_work(query_bridge_wq, &query_bridge_work, stress_test_poll_mode);
			}
			else if (arg == TC358774_IOCTL_START_NORMAL){
				DRM_INFO("TC358774 : ioctl normal\n");
				stress_test_poll_mode = TC358774_START_NORMAL;
				queue_delayed_work(query_bridge_wq, &query_bridge_work, stress_test_poll_mode);
			}
			else if  (arg == TC358774_IOCTL_END){
				DRM_INFO("TC358774 : ioctl end\n");
				cancel_delayed_work_sync(&query_bridge_work);
			}
			else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}

static struct file_operations tc358774_fops = {
.owner = THIS_MODULE,
.open = simple_open,
.unlocked_ioctl = tc358774_ioctl,
};

static struct miscdevice tc358774_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tc358774",
	.fops = &tc358774_fops,
};

bool b101ean01_init(struct intel_dsi_device *dsi)
{
	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */
	int err;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	
	gpio_request(BRIDGE_STANDBY, "DSI2LVDS bridge standby");
	gpio_request(BRIDGE_RESET, "DSI2LVDS bridge BRIDGE_RESET");
	
	bridge_hack_create_device();
	
	if (!dsi) {
		DRM_DEBUG_KMS("Init: Invalid input to b101ean01S_init\n");
		return false;
	}

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->dsi_clock_freq = 412;
	intel_dsi->eotp_pkt = 0;
	//intel_dsi->clock_stop = 2;
	intel_dsi->operation_mode = DSI_VIDEO_MODE;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->turn_arnd_val = 0x22; //0x14
	intel_dsi->rst_timer_val = 0xff;
	intel_dsi->hs_to_lp_count = 0x23;
	intel_dsi->dphy_reg = 0x3B0F2E13;//0x1E103311
	intel_dsi->port = 0; /* PORT_A by default */
	intel_dsi->burst_mode_ratio = 100;
	intel_dsi->lp_byte_clk = 0x3;
	intel_dsi->bw_timer = 0;
	intel_dsi->clk_lp_to_hs_count = 0x1E;//0x1F
	intel_dsi->clk_hs_to_lp_count = 0x0D;//0x0E
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->backlight_on_delay = 220;
	intel_dsi->backlight_off_delay = 200;
/*	
	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eot_disable = 0;
	intel_dsi->port_bits = 0;
	intel_dsi->dsi_clock_freq = 420;
	intel_dsi->video_mode_type = DSI_VIDEO_BURST;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB666_LOOSE;
	intel_dsi->escape_clk_div = ESCAPE_CLOCK_DIVIDER_1;
	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->turn_arnd_val = 0x3f;
	intel_dsi->rst_timer_val = 0xff;
	intel_dsi->init_count = 0x7d0;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 4;
	intel_dsi->bw_timer = 0;
	intel_dsi->clk_lp_to_hs_count = 0x24;
	intel_dsi->clk_hs_to_lp_count = 0x0F;
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->dphy_reg = 0x3F10430D;

	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;
*/	

	/* Program MIPI reset */
	//b101ean01_msgbus_reset(); //<asus-ych20130905>

    //<asus-ethan20131011+>
	//intel_mid_pmic_writeb(0x52,1);//PANEL_EN
	//intel_mid_pmic_writeb(0x51,1);//BACKLIGHT_EN
    //<asus-ethan20131011->

	/***** i2c stress test *****/
	err = misc_register(&tc358774_dev);
	if (err) {
		DRM_INFO("[DISPLAY]: tc358774_dev register failed");
		goto destroy_wq;
	}

	query_bridge_wq = create_singlethread_workqueue("tc358774_wq");
	if(!query_bridge_wq){
		DRM_INFO("[DISPLAY]: unable to create i2c stress test workqueue\n");
		goto destroy_wq;
	}
	INIT_DELAYED_WORK(&query_bridge_work, tc358774_i2c_stress_test);

	return true;
	
destroy_wq:
	destroy_workqueue(query_bridge_wq);

	return true;
}

void b101ean01_panel_reset(struct intel_dsi_device *dsi)
{	
	gpio_direction_output(BRIDGE_STANDBY,1);
	usleep_range(10,15);
	gpio_direction_output(BRIDGE_RESET,1);
	usleep_range(2000,2500);
	//intel_mid_pmic_writeb(0x52,1);//PANEL_EN
        //msleep(300);
}

void  b101ean01_disable_panel_power(struct intel_dsi_device *dsi)
{
	gpio_direction_output(BRIDGE_RESET,0);
	usleep_range(2000, 2500);
	gpio_direction_output(BRIDGE_STANDBY, 0);
}

void b101ean01_panel_enable(struct intel_dsi_device *dsi)
{
        intel_mid_pmic_writeb(PMIC_PANEL_EN, 0x01);
	initial_bridge();
	usleep_range(1000,2000);
}
 
void tc358774_suspend(void)
{
	u32 reg;
	struct i2c_client *i2c = bridge_i2c_client;
	
	tc358774_regr(i2c, LVCFG, &reg);
	reg &= 0xfffffffe;
	tc358774_regw(i2c, LVCFG, reg);
	
	tc358774_regr(i2c, LVPHY0, &reg);
	reg &= 0xff;
	reg |= 0x00448100;
	tc358774_regw(i2c, LVPHY0, reg);
}

void b101ean01_disable(struct intel_dsi_device *dsi)
{
 		tc358774_suspend();
}	

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops b101ean01_dsi_display_ops = {
	.init = b101ean01_init,
	.get_info = b101ean01_get_panel_info,
	.create_resources = b101ean01_create_resources,
	.dpms = b101ean01_dpms,
	.mode_valid = b101ean01_mode_valid,
	.mode_fixup = b101ean01_mode_fixup,
	.panel_reset = b101ean01_panel_reset,
	.enable = b101ean01_panel_enable,
  .disable_panel_power = b101ean01_disable_panel_power,
	//.mode_set = b101ean01_mode_set,
	.disable = b101ean01_disable,
	.detect = b101ean01_detect,
	.send_otp_cmds = b101ean01_send_otp_cmds,
	.get_hw_state = b101ean01_get_hw_state,
	.get_modes = b101ean01_get_modes,
	.destroy = b101ean01_destroy,
	.dump_regs = b101ean01_dump_regs,
};
